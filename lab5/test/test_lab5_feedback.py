"""Exercise the ROS adapter with fake time and feedback; no ROS node or robot."""

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from ece470labs import lab5_exec as app
from ece470labs.lab5_func import default_settings


class Clock:
    """Advance test time without sleeping."""

    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now


def adapter(monkeypatch):
    """Build only the fields used by the hardware methods."""
    clock = Clock()
    monkeypatch.setattr(app, 'time', clock)
    sent, events = [], []
    fake = SimpleNamespace(
        settings=default_settings(), joint_names=['joint'+str(i) for i in range(6)],
        current_positions=[0.0]*6, joint_time=clock.now, carrying=False,
        trajectory_pub=SimpleNamespace(publish=sent.append, get_subscription_count=lambda:1),
        emit=lambda name, data:events.append((name, data)))
    return fake, clock, sent, events


def test_large_motion_respects_limits_and_requires_new_arrival_feedback(monkeypatch):
    fake, clock, sent, _ = adapter(monkeypatch)
    target = [4.7, -.8, .3, 0., 0., 0.]
    def feedback(seconds):
        clock.now += seconds
        fake.current_positions = target
        fake.joint_time = clock.now
    fake.spin_for = feedback
    assert app.UR3e.move_arm(fake, target)
    assert len(sent) == 1
    start, finish = sent[0].points
    duration = finish.time_from_start.sec+finish.time_from_start.nanosec/1e9
    # Sample the controller's zero-velocity/acceleration quintic interpolation.
    times = np.linspace(0, duration, 10001)
    u = times/duration
    positions = np.outer(10*u**3-15*u**4+6*u**5, np.array(target))
    velocity = np.gradient(positions, times, axis=0)
    acceleration = np.gradient(velocity, times, axis=0)
    assert abs(velocity).max() <= fake.settings['motion']['velocity_limit']+1e-5
    assert abs(acceleration).max() <= fake.settings['motion']['acceleration_limit']+1e-5
    assert duration > 8  # The old starter cap would violate these limits.
    assert list(start.positions) == [0.]*6 and list(finish.positions) == target


def test_feedback_loss_aborts_the_current_motion(monkeypatch):
    fake, clock, sent, _ = adapter(monkeypatch)
    fake.settings['motion']['feedback_max_age_seconds'] = .1
    fake.spin_for = lambda seconds:setattr(clock, 'now', clock.now+seconds)
    with pytest.raises(RuntimeError, match='became stale'):
        app.UR3e.move_arm(fake, [.4]*6)
    assert len(sent) == 1


def test_suction_loss_is_checked_during_transport(monkeypatch):
    fake, clock, sent, _ = adapter(monkeypatch)
    fake.carrying, fake.grip_confirmed = True, lambda:False
    def feedback(seconds):
        clock.now += seconds
        fake.joint_time = clock.now
    fake.spin_for = feedback
    with pytest.raises(RuntimeError, match='Suction lost during motion'):
        app.UR3e.move_arm(fake, [.4]*6)
    assert len(sent) == 1


@pytest.mark.parametrize('holding_above,holding_value', [(True, 3.), (False, .2)])
def test_suction_debounce_requires_multiple_fresh_samples(monkeypatch, holding_above, holding_value):
    fake, clock, _, events = adapter(monkeypatch)
    grip = fake.settings['gripper']
    grip['holding_above_threshold'] = holding_above
    grip['debounce_seconds'] = .15
    fake.command_time, fake.io_time, fake.analog_value = clock.now, clock.now, holding_value
    fake.grip_confirmed = lambda:app.UR3e.grip_confirmed(fake)
    samples = []
    def feedback(seconds):
        clock.now += seconds
        fake.io_time = clock.now
        samples.append(clock.now)
    fake.spin_for = feedback
    assert app.UR3e.wait_for_grip(fake, True)
    assert samples[-1]-samples[0] >= .15
    assert events[-1][0] == 'suction_feedback'


def test_one_held_sample_cannot_pass_debounce_without_new_feedback(monkeypatch):
    fake, clock, _, _ = adapter(monkeypatch)
    fake.command_time, fake.io_time, fake.analog_value = clock.now, clock.now, 3.
    fake.settings['gripper']['timeout_seconds'] = .3
    fake.grip_confirmed = lambda:app.UR3e.grip_confirmed(fake)
    def feedback(seconds):
        clock.now += seconds
        if fake.io_time == fake.command_time:
            fake.io_time = clock.now
    fake.spin_for = feedback
    assert not app.UR3e.wait_for_grip(fake, True)


def test_set_io_rejection_stops_the_sequence(monkeypatch):
    fake, clock, _, events = adapter(monkeypatch)
    fake.io_client = SimpleNamespace(call_async=lambda request:SimpleNamespace(
        done=lambda:True, result=lambda:SimpleNamespace(success=False)))
    with pytest.raises(RuntimeError, match='rejected'):
        app.UR3e.set_io(fake, 0, 1.)
    assert not events


def test_exit_closes_valve_and_waits_for_new_output_feedback(monkeypatch):
    fake, clock, _, events = adapter(monkeypatch)
    fake.io_client, fake.carrying = object(), True
    fake.stopped, fake.frozen_view, fake.show = True, False, True
    fake.valve_output, fake.valve_time = True, clock.now
    outputs = []
    def suction(enabled):
        outputs.append(enabled)
        fake.command_time = clock.now
    def feedback(seconds):
        clock.now += seconds
        fake.valve_output, fake.valve_time = False, clock.now
    fake.suction, fake.spin_for = suction, feedback
    fake.wait_for_grip = lambda expected:not expected
    app.UR3e.shutdown_suction(fake)
    assert outputs == [False]
    assert not fake.carrying and not fake.stopped and not fake.show
    assert events[-1] == ('valve_closed', {'output_pin': 0, 'confirmed': True})


def test_exit_reports_failure_when_vacuum_does_not_release(monkeypatch):
    fake, clock, _, events = adapter(monkeypatch)
    fake.io_client = object()
    outputs = []
    fake.suction, fake.wait_for_grip = outputs.append, lambda expected:False
    with pytest.raises(RuntimeError, match='release was not confirmed'):
        app.UR3e.shutdown_suction(fake)
    assert outputs == [False]
    assert not events


def test_exit_does_not_claim_closed_without_digital_feedback(monkeypatch):
    fake, clock, _, events = adapter(monkeypatch)
    fake.io_client = object()
    fake.command_time, fake.valve_time, fake.valve_output = clock.now, 0., True
    fake.suction = lambda enabled:None
    fake.wait_for_grip = lambda expected:True
    fake.spin_for = lambda seconds:setattr(clock, 'now', clock.now+seconds)
    with pytest.raises(RuntimeError, match='digital output feedback'):
        app.UR3e.shutdown_suction(fake)
    assert not events
