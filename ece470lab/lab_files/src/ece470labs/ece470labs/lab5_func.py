"""Lab 5 configuration, preflight planning, and pick/place sequencing."""

import copy
import json
import math
from pathlib import Path

import numpy as np

from .blob_search import CameraCalibration, NATIVE_VISION_DEFAULTS
from .lab_func import forward_kinematics, lab_ik_angles


DEFAULT_SETTINGS = {
    'simulation_only': False,
    'camera': {'beta': None, 'theta': None, 'tx': None, 'ty': None,
               'width': 640, 'height': 480, 'flip_horizontal': False},
    'vision': {**copy.deepcopy(NATIVE_VISION_DEFAULTS), 'flip_horizontal': False},
    'task': {
        'block_height_m': None, 'table_z_m': 0.0, 'contact_offset_m': 0.0,
        'pickup_offset_m': {'green': 0.0, 'yellow': 0.0},
        'safe_z_m': 0.15, 'yaw_deg': 90.0,
        'stack_xy_m': {'green': None, 'yellow': None},
        'workspace_xy_m': None,
        'stack_clearance_m': 0.05,
        'expected_blocks': 4, 'max_per_color': 3,
        'settle_seconds': 5.0, 'frame_max_age_seconds': 1.0,
        'go_away_encoder_deg': [270, -90, 90, -90, -90, 135],
    },
    'gripper': {'output_pin': 0, 'analog_pin': 0, 'threshold': 2.0,
                'holding_above_threshold': True, 'timeout_seconds': 1.5,
                'debounce_seconds': 0.15, 'max_age_seconds': 0.5},
    'motion': {'velocity_limit': 0.5, 'acceleration_limit': 0.4,
               'joint_tolerance_rad': 0.003, 'feedback_max_age_seconds': 1.0},
}


def default_settings():
    """Return independent settings with unmeasured fields left empty."""
    return copy.deepcopy(DEFAULT_SETTINGS)


def load_settings(path=None):
    """Merge a bench JSON file and reject unknown configuration keys."""
    result = default_settings()
    if path:
        with Path(path).open() as stream:
            supplied = json.load(stream)
        def merge(target, source):
            for key, value in source.items():
                if key not in target:
                    raise ValueError('Unknown configuration key: '+str(key))
                if isinstance(target[key], dict) and isinstance(value, dict):
                    merge(target[key], value)
                else:
                    target[key] = value
        merge(result, supplied)
    return result


def save_settings(path, settings):
    """Write settings without accepting NaN or infinity."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(settings, indent=2, allow_nan=False)+'\n')


def _finite(value, label):
    """Require a measured finite numeric value."""
    if value is None or not math.isfinite(float(value)):
        raise ValueError('Set a finite '+label+' in the configuration.')
    return float(value)


def inside_workspace(xy, workspace):
    """Check world XY against [xmin,xmax,ymin,ymax], in meters."""
    return workspace[0] <= xy[0] <= workspace[1] and workspace[2] <= xy[1] <= workspace[3]


def validate_motion_settings(settings):
    """Reject missing calibration, placement sites, or invalid motion limits."""
    calibration = CameraCalibration.from_dict(settings['camera'])
    if bool(settings['vision']['flip_horizontal']) != calibration.flip_horizontal:
        raise ValueError('Image flip differs from the measured calibration.')
    task, grip, motion = settings['task'], settings['gripper'], settings['motion']
    height = _finite(task['block_height_m'], 'task.block_height_m')
    if height <= 0:
        raise ValueError('Block height must be positive.')
    for key in ['table_z_m', 'contact_offset_m', 'safe_z_m', 'yaw_deg',
                'settle_seconds', 'frame_max_age_seconds']:
        _finite(task[key], 'task.'+key)
    for key in ['expected_blocks', 'max_per_color']:
        if not isinstance(task[key], int) or task[key] <= 0:
            raise ValueError('task.'+key+' must be a positive integer.')
    if task['expected_blocks'] > 2*task['max_per_color']:
        raise ValueError('Expected count exceeds the two stack capacities.')
    if task['settle_seconds'] < 0 or task['frame_max_age_seconds'] <= 0:
        raise ValueError('Image timing values are invalid.')
    surface = task['table_z_m']+task['contact_offset_m']
    if surface+height <= task['table_z_m']:
        raise ValueError('Contact offset must not put the cup below the table.')
    for color in ['green', 'yellow']:
        offset = _finite(task.get('pickup_offset_m', {}).get(color, 0.0),
                         'task.pickup_offset_m.'+color)
        if surface+height+offset <= task['table_z_m']:
            raise ValueError('Pickup offset must not put the cup below the table.')
    if task['safe_z_m'] <= max(task['table_z_m'], surface)+height*task['max_per_color']+0.02:
        raise ValueError('safe_z_m must clear a full stack by more than 0.02 m.')
    bounds = task['workspace_xy_m']
    if bounds is None or len(bounds) != 4 or not np.all(np.isfinite(bounds)):
        raise ValueError('Set task.workspace_xy_m to [xmin,xmax,ymin,ymax] in measured world meters.')
    if bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
        raise ValueError('Workspace bounds are reversed.')
    if _finite(task['stack_clearance_m'], 'task.stack_clearance_m') <= 0:
        raise ValueError('Stack clearance must be positive.')
    for color in ['green', 'yellow']:
        xy = task['stack_xy_m'][color]
        if xy is None or len(xy) != 2 or not np.all(np.isfinite(xy)) or not inside_workspace(xy, bounds):
            raise ValueError('Set task.stack_xy_m.'+color+' inside the measured workspace.')
    if np.linalg.norm(np.array(task['stack_xy_m']['green'])-task['stack_xy_m']['yellow']) < 0.04:
        raise ValueError('Stack centers must be separated by at least 0.04 m; allow for actual block size.')
    away = task['go_away_encoder_deg']
    if len(away) != 6 or not np.all(np.isfinite(away)) or max(abs(float(v)) for v in away) > 360:
        raise ValueError('go_away_encoder_deg needs six finite encoder angles within +/-360 degrees.')
    for key in ['velocity_limit', 'acceleration_limit', 'joint_tolerance_rad', 'feedback_max_age_seconds']:
        if _finite(motion[key], 'motion.'+key) <= 0:
            raise ValueError('Motion limits and timing must be positive.')
    _finite(grip['threshold'], 'gripper.threshold')
    for key in ['timeout_seconds', 'max_age_seconds', 'debounce_seconds']:
        if _finite(grip[key], 'gripper.'+key) <= 0:
            raise ValueError('Gripper timing must be positive.')
    for key in ['output_pin', 'analog_pin']:
        if not isinstance(grip[key], int) or not 0 <= grip[key] <= 17:
            raise ValueError('Invalid gripper pin: '+key)
    return calibration


def encoder_angles(xyz, yaw):
    """Solve with the existing Lab 4 geometry and independently check FK."""
    angles = lab_ik_angles(*xyz, yaw)
    transform = forward_kinematics(*angles)
    rad = math.radians(yaw)
    expected = [[math.cos(rad), -math.sin(rad), 0],
                [math.sin(rad), math.cos(rad), 0], [0, 0, 1]]
    if (np.linalg.norm(transform[:3, 3]/1000-np.array(xyz)) > 1e-8 or
            not np.allclose(transform[:3, :3], expected, atol=1e-8, rtol=0)):
        raise ValueError('IK/FK verification failed for '+str(xyz))
    encoders = (np.array(angles)+[math.pi, 0, 0, -math.pi/2, 0, 0]).tolist()
    if max(abs(value) for value in encoders) > 2*math.pi+1e-8:
        raise ValueError('Planned encoder angle exceeds +/-360 degrees.')
    return encoders


def waypoints(start_xy, target_xyz, settings, color=None):
    """Apply a color-specific pickup offset; keep placement heights independent."""
    task = settings['task']
    pick_z = task['table_z_m']+task['block_height_m']+task['contact_offset_m']
    pick_z += task.get('pickup_offset_m', {}).get(color, 0.0)
    safe_z = task['safe_z_m']
    return [('source_above', [*start_xy, safe_z]),
            ('source_pick', [*start_xy, pick_z]),
            ('source_lift', [*start_xy, safe_z]),
            ('target_above', [*target_xyz[:2], safe_z]),
            ('target_place', list(target_xyz)),
            ('target_lift', [*target_xyz[:2], safe_z])]


def vertical_targets(start_xyz, target_xyz, step_m=0.06):
    """Use at most three vertical segments; horizontal transfer stays elevated."""
    target = np.array(target_xyz, dtype=float)
    if start_xyz is None or np.linalg.norm(target[:2]-np.array(start_xyz)[:2]) >= 1e-8:
        return [target]
    start = np.array(start_xyz, dtype=float)
    count = min(3, max(1, math.ceil(abs(target[2]-start[2])/step_m)))
    return [start+(target-start)*i/count for i in range(1, count+1)]


def stack_target(color, completed, settings):
    """Place on the next layer using successful placements only."""
    task = settings['task']
    return [*task['stack_xy_m'][color], task['table_z_m']+task['contact_offset_m']+
            (completed+1)*task['block_height_m']]


def plan_tasks(detections, settings):
    """Freeze both colors and validate all paths, including skipped-pick heights."""
    validate_motion_settings(settings)
    task = settings['task']
    selected = [d for d in detections if d.color in ['green', 'yellow']]
    outside = [d for d in selected if d.world_xy is None or
               not inside_workspace(d.world_xy, task['workspace_xy_m'])]
    if outside:
        raise ValueError('Detected blocks outside the configured workspace or without world coordinates. Adjust ROI/calibration.')
    if len(selected) != task['expected_blocks']:
        raise ValueError(f"Expected {task['expected_blocks']} blocks; detected {len(selected)}. Check masks and ROI.")
    for index, block in enumerate(selected):
        for other in selected[index+1:]:
            if np.linalg.norm(np.array(block.world_xy)-other.world_xy) < 0.01:
                raise ValueError('Overlapping block detections: check color selection; one object may be detected twice.')
    for color, target in task['stack_xy_m'].items():
        if any(np.linalg.norm(np.array(block.world_xy)-target) < task['stack_clearance_m']
               for block in selected):
            raise ValueError(f'Keep the {color} stack site empty: move source blocks at least '
                             f"{task['stack_clearance_m']*100:.0f} cm from its center.")
    selected.sort(key=lambda d: (d.color, d.world_xy[0], d.world_xy[1]))
    for color in ['green', 'yellow']:
        count = sum(d.color == color for d in selected)
        if count > task['max_per_color']:
            raise ValueError('Too many '+color+' blocks for the configured stack.')
        for detection in [d for d in selected if d.color == color]:
            for level in range(count):
                target = stack_target(color, level, settings)
                previous = None
                for _, xyz in waypoints(detection.world_xy, target, settings, detection.color):
                    for sample in vertical_targets(previous, xyz):
                        encoder_angles(sample, task['yaw_deg'])
                    previous = xyz
    return selected


def move_block_vision(robot, start_xy, target_xyz, settings, color=None):
    """Return False for a missing source block; abort other hardware failures.

    The robot adapter supplies move_xyz, suction, wait_for_grip,
    grip_confirmed, emit, and a carrying flag. No ROS dependency is needed here.
    """
    points = dict(waypoints(start_xy, target_xyz, settings, color))
    def move(name):
        if not robot.move_xyz(points[name]):
            raise RuntimeError('Motion failed at '+name)
    robot.suction(False)
    if not robot.wait_for_grip(False):
        raise RuntimeError('Vacuum did not release before pickup; check suction feedback.')
    move('source_above')
    move('source_pick')
    robot.suction(True)
    if not robot.wait_for_grip(True):
        print('no block found', flush=True)
        robot.suction(False)
        if not robot.wait_for_grip(False):
            raise RuntimeError('Vacuum did not release after a failed pickup.')
        move('source_lift')
        robot.emit('missing_block', {'source_xy': list(start_xy)})
        return False
    robot.carrying = True
    move('source_lift')
    if not robot.grip_confirmed():
        raise RuntimeError('Suction lost after lift; stop and inspect the dropped block.')
    move('target_above')
    if not robot.grip_confirmed():
        raise RuntimeError('Suction lost during transport; stop and inspect the work area.')
    move('target_place')
    robot.suction(False)
    if not robot.wait_for_grip(False):
        raise RuntimeError('Block did not release at the stack.')
    robot.carrying = False
    move('target_lift')
    return True


def execute_tasks(robot, detections, settings):
    """Use a frozen list; stack levels increase only after successful placement."""
    frozen = plan_tasks(list(detections), settings)
    completed, missing = {'green': 0, 'yellow': 0}, []
    for block in frozen:
        target = stack_target(block.color, completed[block.color], settings)
        robot.emit('pick_plan', dict(block=block.as_dict(), target_xyz=target))
        if move_block_vision(robot, block.world_xy, target, settings, block.color):
            completed[block.color] += 1
            robot.emit('placed', {'color': block.color, 'target_xyz': target})
        else:
            missing.append(block.as_dict())
    result = dict(placed=completed, missing=missing)
    robot.emit('summary', result)
    return result
