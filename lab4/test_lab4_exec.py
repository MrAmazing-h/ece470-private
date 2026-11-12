"""Exercise command and trajectory behavior without connecting to a robot."""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                      'ece470lab/lab_files/src/ece470labs'))
from ece470labs import lab4_exec as app  # noqa: E402


@pytest.mark.parametrize('ros_args', [[], ['--ros-args', '-r', '__node:=test']])
def test_dry_run_does_not_initialize_ros(monkeypatch, capsys, ros_args):
    def forbidden(*args, **kwargs):
        raise AssertionError('Dry run must not initialize ROS')
    monkeypatch.setattr(app.rclpy, 'init', forbidden)
    app.main(['0.1', '0.1', '0.15', '90', '--dry-run', *ros_args])
    assert 'World gripper position (m): [0.1  0.1  0.15]' in capsys.readouterr().out


@pytest.mark.parametrize('args', [
    ['0.1', '0.1', '0.15'],
    ['2', '2', '2', '90'],
    ['nan', '0.1', '0.15', '90'],
])
def test_invalid_command_exits_before_robot_access(monkeypatch, args):
    def forbidden(*args, **kwargs):
        raise AssertionError('Invalid command must not initialize ROS')
    monkeypatch.setattr(app.rclpy, 'init', forbidden)
    with pytest.raises(SystemExit) as error:
        app.main(args)
    assert error.value.code == 2


def test_feedback_is_ordered_and_partial_messages_are_ignored():
    fake = SimpleNamespace(joint_names=['a', 'b', 'c'], current_positions=None)
    app.UR3e.joint_state_callback(
        fake, SimpleNamespace(name=['c', 'a', 'b'], position=[3., 1., 2.]))
    assert fake.current_positions == [1., 2., 3.]
    app.UR3e.joint_state_callback(
        fake, SimpleNamespace(name=['a'], position=[9.]))
    assert fake.current_positions == [1., 2., 3.]


def test_trajectory_reaches_goal_with_bounded_quintic_dynamics(monkeypatch):
    messages = []
    initial = [0.] * 6
    target = [6., -4., 3., -2., 1., 0.]
    logger = SimpleNamespace(error=lambda *_: None, info=lambda *_: None)
    fake = SimpleNamespace(
        current_positions=initial.copy(), joint_names=list('abcdef'),
        trajectory_pub=SimpleNamespace(publish=messages.append),
        get_logger=lambda: logger)
    monkeypatch.setattr(app.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(app.rclpy, 'spin_once',
                        lambda node, **kw: setattr(node, 'current_positions', target))
    assert app.UR3e.move_arm(fake, target)
    message, = messages
    assert message.joint_names == fake.joint_names
    start, goal = message.points
    assert list(start.positions) == initial
    assert list(goal.positions) == target
    assert list(goal.velocities) == [0.] * 6
    assert list(goal.accelerations) == [0.] * 6
    duration = goal.time_from_start.sec + goal.time_from_start.nanosec / 1e9
    # Sample the interpolating quintic to verify actual peak dynamics.
    fraction = np.linspace(0, 1, 10001)
    velocity = 6 / duration * (30*fraction**2 - 60*fraction**3 + 30*fraction**4)
    acceleration = 6 / duration**2 * (60*fraction - 180*fraction**2 + 120*fraction**3)
    assert np.max(np.abs(velocity)) <= 1.0 + 1e-9
    assert np.max(np.abs(acceleration)) <= 0.8 + 1e-9
