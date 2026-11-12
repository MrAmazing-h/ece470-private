#!/usr/bin/env python3
"""Lab 5 starter workflow: frozen camera snapshot, Lab 4 IK, color stacks.

Default mode only observes images. Motion requires --execute or --park-only.
Color thresholds use the measured profile in config.json.
"""

import argparse
import copy
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from rclpy.signals import SignalHandlerOptions
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

from .blob_search import CameraCalibration, Detection, analyze_native_image as analyze_image
from .lab5_func import (encoder_angles, execute_tasks, load_settings,
                       move_block_vision as pick_and_place, plan_tasks,
                       stack_target, validate_motion_settings, vertical_targets, waypoints)


class RunLog:
    """Save settings, events, and images in one timestamped directory."""

    def __init__(self, base, settings, mode):
        self.directory = Path(base)/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.directory.mkdir(parents=True)
        self.record = dict(mode=mode, settings=settings, events=[])
        self.emit('started', {})

    def emit(self, event, values):
        """Print and persist one event, including failed attempts."""
        entry = dict(time_utc=datetime.now(timezone.utc).isoformat(), event=event, **values)
        self.record['events'].append(entry)
        print(event+': '+json.dumps(values, allow_nan=False), flush=True)
        (self.directory/'run.json').write_text(json.dumps(self.record, indent=2, allow_nan=False)+'\n')

    def save_frame(self, frame):
        """Save the full image, centroid overlay, and masks."""
        image, overlay, masks, _ = frame
        for name, pixels in dict(camera=image, centroids=overlay, **masks).items():
            cv2.imwrite(str(self.directory/(name+'.png')), pixels)


class UR3e(Node):
    """Keep the starter callbacks and motion methods; gate command interfaces."""

    def __init__(self, settings, log, motion_enabled=False, show=False,
                 calibration=None, image_topic='/image_raw', points_file=None):
        super().__init__('ur3e_lab5')
        self.settings, self.log = settings, log
        self.calibration, self.show, self.points_file = calibration, show, points_file
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.current_positions, self.joint_time = None, 0.0
        self.analog_value, self.io_time, self.command_time = None, 0.0, 0.0
        self.valve_output, self.valve_time = None, 0.0
        self.carrying, self.last_xyz, self.suction_enabled = False, None, False
        self.latest, self.image_time, self.image_error = None, 0.0, None
        self.green_blocks, self.yellow_blocks = [], []
        self.frozen_view, self.stopped = False, False
        self.bridge = CvBridge()
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10
        ) if motion_enabled else None
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io') if motion_enabled else None
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.io_state_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_state_callback, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.callback_executor = SingleThreadedExecutor(context=self.context)
        self.callback_executor.add_node(self)

    def destroy_node(self):
        """Release the persistent executor before destroying ROS entities."""
        self.callback_executor.remove_node(self)
        self.callback_executor.shutdown()
        return super().destroy_node()

    def emit(self, event, values):
        """Share the planner's event interface."""
        self.log.emit(event, values)

    def joint_state_callback(self, msg):
        """Accept complete finite feedback in the required joint order."""
        try:
            positions = [float(msg.position[msg.name.index(name)]) for name in self.joint_names]
        except (ValueError, IndexError):
            return
        if np.all(np.isfinite(positions)):
            self.current_positions, self.joint_time = positions, time.monotonic()

    def io_state_callback(self, msg):
        """Read the configured analog pin, regardless of message order."""
        for state in msg.analog_in_states:
            if state.pin == self.settings['gripper']['analog_pin'] and math.isfinite(state.state):
                self.analog_value, self.io_time = float(state.state), time.monotonic()
        for state in getattr(msg, 'digital_out_states', []):
            if state.pin == self.settings['gripper']['output_pin']:
                self.valve_output, self.valve_time = bool(state.state), time.monotonic()

    def image_callback(self, msg):
        """Detect both colors from one frame; freeze execution separately."""
        if self.frozen_view:
            return
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = analyze_image(raw, self.settings, self.calibration)
            self.latest, self.image_time, self.image_error = frame, time.monotonic(), None
            self.green_blocks = [d.world_xy for d in frame[3] if d.color == 'green']
            self.yellow_blocks = [d.world_xy for d in frame[3] if d.color == 'yellow']
            if self.show:
                cv2.imshow('Lab5 centroids', frame[1])
                cv2.setMouseCallback('Lab5 centroids', self.record_pixel)
                for color, mask in frame[2].items():
                    cv2.imshow('Lab5 mask '+color, mask)
        except (ValueError, CvBridgeError) as error:
            self.image_error = str(error)
            self.get_logger().error(self.image_error)
        finally:
            self.handle_key()

    def handle_key(self):
        """Pump the GUI; q exits, f freezes, s saves the current frame."""
        if not getattr(self, 'show', False):
            return
        key = cv2.waitKey(1) & 0xff
        if key in [ord('q'), 27]:
            self.stopped = True
        elif key == ord('f'):
            self.frozen_view = not self.frozen_view
        elif key == ord('s') and self.latest is not None:
            self.log.save_frame(self.latest)

    def record_pixel(self, event, col, row, flags, userdata):
        """Print a pixel; optionally append blank world fields to CSV."""
        if event != cv2.EVENT_LBUTTONDOWN or self.latest is None:
            return
        xy = self.calibration.image_to_world(col, row) if self.calibration else None
        self.emit('clicked_pixel', dict(col=col, row=row, world_xy=xy))
        if self.points_file:
            path = Path(self.points_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            header = not path.exists() or path.stat().st_size == 0
            with path.open('a', newline='') as stream:
                writer = csv.writer(stream)
                if header:
                    writer.writerow(['col', 'row', 'x_world_m', 'y_world_m'])
                writer.writerow([col, row, '', ''])

    def spin_for(self, seconds):
        """Service callbacks for a bounded interval."""
        end = time.monotonic()+seconds
        while rclpy.ok() and not self.stopped and time.monotonic() < end:
            # A fixed timeout preserves the ready-callback iterator and avoids starvation.
            self.callback_executor.spin_once(timeout_sec=.01)
            if self.frozen_view:
                self.handle_key()
        if not rclpy.ok() or self.stopped:
            raise KeyboardInterrupt

    def wait_until_ready(self, require_io=True, timeout=15.0):
        """Require a controller and fresh state before sending trajectories."""
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            joints = self.current_positions is not None and time.monotonic()-self.joint_time < self.settings['motion']['feedback_max_age_seconds']
            controller = self.trajectory_pub is not None and self.trajectory_pub.get_subscription_count() > 0
            io = (self.io_client is not None and self.io_client.service_is_ready() and
                  self.analog_value is not None and time.monotonic()-self.io_time < self.settings['gripper']['max_age_seconds'])
            if joints and controller and (io or not require_io):
                return
            self.spin_for(.1)
        raise RuntimeError('Robot/controller/IO unavailable or stale. Check driver and ROS domain.')

    def move_arm(self, target):
        """Send a bounded quintic trajectory; verify fresh measured arrival."""
        if self.trajectory_pub is None:
            raise RuntimeError('Arm commands are disabled in this mode.')
        if (len(target) != 6 or not np.all(np.isfinite(target)) or
                max(abs(float(v)) for v in target) > 2*math.pi):
            raise ValueError('Invalid encoder target; expected six radians within +/-2pi.')
        limit = self.settings['motion']
        if self.current_positions is None or time.monotonic()-self.joint_time > limit['feedback_max_age_seconds']:
            raise RuntimeError('Joint feedback is stale.')
        if self.trajectory_pub.get_subscription_count() == 0:
            raise RuntimeError('Trajectory controller is unavailable.')
        delta = max(abs(a-b) for a, b in zip(self.current_positions, target))
        duration = max(1.0, 1.875*delta/limit['velocity_limit'],
                       math.sqrt(10/math.sqrt(3)*delta/limit['acceleration_limit']))
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        for positions, seconds in [(self.current_positions, 0), (target, duration)]:
            point = JointTrajectoryPoint()
            point.positions = list(map(float, positions))
            point.velocities, point.accelerations = [0.0]*6, [0.0]*6
            ns = round(seconds*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1_000_000_000)
            msg.points.append(point)
        sent = time.monotonic()
        self.trajectory_pub.publish(msg)
        self.emit('joint_command', dict(encoder_rad=list(map(float, target)), duration_s=duration))
        deadline = sent+max(30, 5*duration+5)
        while time.monotonic() < deadline:
            self.spin_for(.05)
            if time.monotonic()-self.joint_time > limit['feedback_max_age_seconds']:
                raise RuntimeError('Joint feedback became stale during motion.')
            if self.carrying and not self.grip_confirmed():
                raise RuntimeError('Suction lost during motion; inspect the work area.')
            if self.joint_time > sent and all(abs(a-b) < limit['joint_tolerance_rad'] for a, b in zip(self.current_positions, target)):
                return True
        raise RuntimeError('Motion timed out before measured arrival.')

    def move_xyz(self, xyz):
        """Use Lab 4 IK; split vertical travel into short Cartesian segments."""
        for sample in vertical_targets(self.last_xyz, xyz):
            if not self.move_arm(encoder_angles(sample, self.settings['task']['yaw_deg'])):
                return False
        self.last_xyz = list(xyz)
        return True

    def set_io(self, pin, state):
        """Set a digital output and require an acknowledged service result."""
        if self.io_client is None:
            raise RuntimeError('Suction commands are disabled in this mode.')
        req = SetIO.Request()
        req.fun, req.pin, req.state = 1, int(pin), float(state)
        future = self.io_client.call_async(req)
        end = time.monotonic()+3
        while not future.done() and time.monotonic() < end:
            self.spin_for(.02)
        if not future.done():
            future.cancel()
            raise RuntimeError('SetIO service timed out.')
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError('SetIO service rejected the command.')
        self.command_time = time.monotonic()
        self.emit('suction_command', dict(pin=int(pin), enabled=bool(state)))
        return result

    def suction(self, enabled):
        """Switch suction through the configured digital output."""
        if self.io_client is None:
            raise RuntimeError('Suction commands are disabled in this mode.')
        self.suction_enabled = bool(enabled)
        self.set_io(self.settings['gripper']['output_pin'], enabled)

    def grip_confirmed(self):
        """Classify fresh analog feedback using threshold and polarity."""
        grip = self.settings['gripper']
        if self.analog_value is None or time.monotonic()-self.io_time > grip['max_age_seconds']:
            raise RuntimeError('Suction feedback is stale or missing.')
        above = self.analog_value >= grip['threshold']
        return above if grip['holding_above_threshold'] else not above

    def wait_for_grip(self, expected):
        """Debounce feedback newer than the last output command."""
        grip = self.settings['gripper']
        end, stable_since, fresh = time.monotonic()+grip['timeout_seconds'], None, False
        while time.monotonic() < end:
            self.spin_for(.02)
            if self.io_time <= self.command_time:
                continue
            fresh = True
            matches = self.grip_confirmed() == expected
            stable_since = (stable_since or self.io_time) if matches else None
            if stable_since is not None and self.io_time-stable_since >= grip['debounce_seconds']:
                self.emit('suction_feedback', dict(holding=expected, analog=self.analog_value))
                return True
        if not fresh:
            raise RuntimeError('No new suction feedback after the command.')
        return False

    def move_block_vision(self, start_xy, target_xyz, color=None):
        """Complete the starter helper using IK and suction feedback."""
        return pick_and_place(self, start_xy, target_xyz, self.settings, color)

    def shutdown_suction(self):
        """Close the valve on exit and verify output and released-vacuum feedback."""
        if self.io_client is None:
            return
        self.stopped, self.frozen_view, self.show = False, True, False
        self.suction(False)
        if not self.wait_for_grip(False):
            raise RuntimeError('Valve OFF requested, but vacuum release was not confirmed.')
        deadline = time.monotonic()+2.0
        while self.valve_time <= self.command_time and time.monotonic() < deadline:
            self.spin_for(.02)
        if self.valve_time <= self.command_time or self.valve_output is not False:
            raise RuntimeError('Valve OFF was not confirmed by fresh digital output feedback.')
        self.carrying = False
        self.emit('valve_closed', dict(output_pin=self.settings['gripper']['output_pin'], confirmed=True))


def log_plan(detections, settings, log, first_only=False):
    """Validate the full scene, then optionally plan just one test pickup."""
    selected = plan_tasks(detections, settings)
    run_settings = copy.deepcopy(settings)
    if first_only:
        scene_count = len(selected)
        selected = selected[:1]
        run_settings['task']['expected_blocks'] = 1
        log.emit('single_block_test', dict(scene_block_count=scene_count,
                                          selected_block=selected[0].as_dict()))
    levels, blocks = dict(green=0, yellow=0), []
    for block in selected:
        target = stack_target(block.color, levels[block.color], settings)
        levels[block.color] += 1
        poses = [dict(name=name, xyz_m=xyz, encoder_rad=encoder_angles(xyz, settings['task']['yaw_deg']))
                 for name, xyz in waypoints(block.world_xy, target, settings, block.color)]
        blocks.append(dict(detection=block.as_dict(), poses=poses))
    log.emit('dry_run_plan', dict(blocks=blocks))
    return selected, run_settings


def main(args=None):
    """Select a mode; offline checks never initialize ROS."""
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    for mode in ['vision-only', 'dry-run', 'execute', 'park-only', 'io-only']:
        modes.add_argument('--'+mode, action='store_true')
    parser.add_argument('--config', default=os.environ.get('LAB5_CONFIG'))
    parser.add_argument('--image', help='Saved image; uses the same starter detector as live mode.')
    parser.add_argument('--detections', help='Offline JSON world detections; dry-run only, no color processing.')
    parser.add_argument('--no-gui', action='store_true')
    parser.add_argument('--duration', type=float, default=0, help='Preview seconds; zero runs until exit.')
    parser.add_argument('--expected-blocks', type=int,
                        help='Expected visible scene count, not the number to pick.')
    parser.add_argument('--first-only', action='store_true',
                        help='Validate the full scene, then pick only the first block.')
    parser.add_argument('--pause-after-snapshot', action='store_true')
    parser.add_argument('--calibration-points', help='CSV for clicked pixels; fill world meters manually.')
    parser.add_argument('--image-topic', default='/image_raw')
    parser.add_argument('--log-dir', default=os.environ.get('LAB5_LOG_DIR', 'lab5_runs'))
    argv = sys.argv if args is None else ['lab5_exec', *args]
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    motion = options.execute or options.park_only
    try:
        settings = load_settings(options.config)
        if options.expected_blocks is not None:
            if options.expected_blocks <= 0:
                raise ValueError('--expected-blocks must be positive.')
            settings['task']['expected_blocks'] = options.expected_blocks
        if not math.isfinite(options.duration) or options.duration < 0:
            raise ValueError('--duration must be nonnegative.')
        if options.pause_after_snapshot and not options.execute:
            raise ValueError('--pause-after-snapshot requires --execute.')
        if options.first_only and not (options.execute or options.dry_run):
            raise ValueError('--first-only requires --execute or --dry-run.')
        if options.calibration_points and (motion or options.dry_run or options.io_only):
            raise ValueError('--calibration-points is for vision-only mode.')
        if options.image and (motion or options.detections or options.io_only):
            raise ValueError('--image cannot be used with hardware commands or detections.')
        if options.detections and not options.dry_run:
            raise ValueError('--detections requires --dry-run.')
        if motion and (not options.config or settings['simulation_only']):
            raise ValueError('Hardware commands require a measured, non-simulation configuration.')
        camera = settings['camera']
        calibration = CameraCalibration.from_dict(camera) if any(camera[k] is not None for k in ['beta', 'theta', 'tx', 'ty']) else None
        if options.execute or options.dry_run:
            calibration = validate_motion_settings(settings)
        if options.park_only:
            away = settings['task']['go_away_encoder_deg']
            if len(away) != 6 or not np.all(np.isfinite(away)) or max(map(abs, away)) > 360:
                raise ValueError('Invalid go_away_encoder_deg.')
            for key in ['velocity_limit', 'acceleration_limit', 'joint_tolerance_rad', 'feedback_max_age_seconds']:
                value = settings['motion'][key]
                if value is None or not math.isfinite(value) or value <= 0:
                    raise ValueError('Invalid motion.'+key)
    except (ValueError, TypeError, KeyError, OSError) as error:
        parser.error(str(error))

    mode = next((name for name in ['execute', 'park_only', 'dry_run', 'io_only'] if getattr(options, name)), 'vision_only')
    log = RunLog(options.log_dir, settings, mode)
    if options.image or options.detections:
        try:
            if options.image:
                frame = analyze_image(cv2.imread(options.image), settings, calibration)
                log.save_frame(frame)
                detections = frame[3]
            else:
                detections = [Detection(**d) for d in json.loads(Path(options.detections).read_text())]
            log.emit('snapshot', dict(blocks=[d.as_dict() for d in detections]))
            if options.dry_run:
                log_plan(detections, settings, log, options.first_only)
            log.emit('finished', dict(arm_commands_sent=0, suction_commands_sent=0))
        except (ValueError, TypeError, KeyError, OSError) as error:
            log.emit('error', dict(message=str(error)))
            raise SystemExit(1)
        return

    if motion:
        try:
            input('Check Remote/Normal state, the driver, and clear the arm path.\n'
                  'This mode WILL MOVE the arm. Enter continues, Ctrl-C cancels: ')
        except (KeyboardInterrupt, EOFError):
            log.emit('cancelled', {})
            return
    # Keep ROS alive during Ctrl-C cleanup so the valve can still be closed.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node, exit_code = None, 0
    try:
        show = not options.no_gui and bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        node = UR3e(settings, log, motion, show, calibration, options.image_topic, options.calibration_points)
        if motion:
            node.wait_until_ready(require_io=options.execute)
            node.move_arm(np.radians(settings['task']['go_away_encoder_deg']).tolist())
        if options.park_only:
            log.emit('parked', {})
            return
        if options.execute or options.dry_run:
            node.spin_for(settings['task']['settle_seconds'])
            deadline = time.monotonic()+8
            while node.latest is None and time.monotonic() < deadline:
                node.spin_for(.1)
            if (node.latest is None or node.image_error is not None or
                    time.monotonic()-node.image_time > settings['task']['frame_max_age_seconds']):
                raise RuntimeError('No fresh valid camera image: '+str(node.image_error))
            frozen = list(node.latest[3])
            node.frozen_view = True
            log.save_frame(node.latest)
            log.emit('snapshot', dict(blocks=[d.as_dict() for d in frozen]))
            frozen, run_settings = log_plan(frozen, settings, log, options.first_only)
            if options.execute:
                if options.pause_after_snapshot:
                    input('Snapshot fixed and arm parked. Remove a block only while stationary.\n'
                          'Clear the work area, then press Enter to start pickup: ')
                result = execute_tasks(node, frozen, run_settings)
                node.move_arm(np.radians(settings['task']['go_away_encoder_deg']).tolist())
                exit_code = 1 if result['missing'] else 0
        else:
            start, report_time = time.monotonic(), 0.0
            print('Read-only: no trajectory publisher or suction client.')
            while not node.stopped and (options.duration == 0 or time.monotonic()-start < options.duration):
                node.spin_for(.1)
                if options.io_only and time.monotonic()-report_time >= 1:
                    fresh = node.analog_value is not None and time.monotonic()-node.io_time < settings['gripper']['max_age_seconds']
                    log.emit('io_sample', dict(analog=node.analog_value, fresh=fresh))
                    report_time = time.monotonic()
                if not options.io_only and node.latest is None and time.monotonic()-start > 8:
                    raise RuntimeError('No valid image received: '+str(node.image_error))
            if node.latest is not None:
                log.save_frame(node.latest)
        log.emit('finished', dict(exit_code=exit_code))
    except (KeyboardInterrupt, EOFError):
        log.emit('cancelled', {})
    except (RuntimeError, ValueError) as error:
        log.emit('error', dict(message=str(error)))
        exit_code = 1
    finally:
        if node is not None:
            if node.io_client is not None and (options.execute or node.suction_enabled):
                try:
                    node.shutdown_suction()
                except (RuntimeError, KeyboardInterrupt) as error:
                    log.emit('valve_shutdown_failed', dict(message=str(error)))
                    exit_code = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
