"""Numerical IK regression tests; no ROS node or robot is needed."""

from contextlib import redirect_stdout
import io
from pathlib import Path
import sys

import numpy as np
import pytest

PACKAGE = (Path(__file__).resolve().parents[1] / 'ece470lab' / 'lab_files' /
           'src' / 'ece470labs')
sys.path.insert(0, str(PACKAGE))
from ece470labs.lab_func import (  # noqa: E402
    forward_kinematics, lab_ik_angles, lab_invk,
)


@pytest.mark.parametrize('pose', [
    (0.1, 0.1, 0.15, 90),
    (0.2, 0.0, 0.1, 0),
    (0.0, 0.25, 0.2, -90),
    (-0.3, 0.2, 0.15, 45),
    (-0.3, -0.05, 0.12, 180),
    (0.1, 0.1, 0.15, 450),
    (0.1, 0.1, 0.15, -270),
])
def test_pose_round_trip_and_encoder_offsets(pose):
    angles = lab_ik_angles(*pose)
    transform = forward_kinematics(*angles)
    np.testing.assert_allclose(transform[:3, 3] / 1000, pose[:3], atol=1e-12)
    yaw = np.radians(pose[3])
    expected = [[np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    np.testing.assert_allclose(transform[:3, :3], expected, atol=1e-12)
    assert angles[2] >= 0
    assert angles[4] == -np.pi / 2
    assert abs(sum(angles[1:4])) < 1e-12
    with redirect_stdout(io.StringIO()):
        encoders = lab_invk(*pose)
    np.testing.assert_allclose(np.array(encoders) - angles,
                               [np.pi, 0, 0, -np.pi / 2, 0, 0], atol=1e-12)


def test_forward_generated_poses_across_waist_quadrants():
    # Generate targets using the pre-existing POE model, independently of IK.
    rng = np.random.default_rng(470)
    for _ in range(200):
        t1 = rng.uniform(-np.pi, np.pi)
        t2 = rng.uniform(-2.2, -0.3)
        t3 = rng.uniform(0.3, 2.2)
        t6 = rng.uniform(-np.pi, np.pi)
        original = forward_kinematics(t1, t2, t3, -t2-t3, -np.pi/2, t6)
        yaw = np.degrees(np.arctan2(original[1, 0], original[0, 0]))
        solved = lab_ik_angles(*(original[:3, 3] / 1000), yaw)
        recovered = forward_kinematics(*solved)
        np.testing.assert_allclose(recovered, original, atol=1e-8)


@pytest.mark.parametrize('distance', [0.244 + 0.213, 0.244 - 0.213])
def test_straight_and_folded_elbow_boundaries(distance):
    pose = (distance - 0.0115, 0.281, 0.011, 0)
    result = forward_kinematics(*lab_ik_angles(*pose))
    np.testing.assert_allclose(result[:3, 3] / 1000, pose[:3], atol=1e-9)


def test_waist_tangent_boundary_and_signed_radial_coordinate():
    pose = (-0.0965, 0.281, 0.211, 0)
    result = forward_kinematics(*lab_ik_angles(*pose))
    np.testing.assert_allclose(result[:3, 3] / 1000, pose[:3], atol=1e-9)


@pytest.mark.parametrize('pose', [
    (-0.0965, 0.15, 0.15, 0),  # inside the waist-offset cylinder
    (2.0, 2.0, 2.0, 90),      # beyond the outer reach
    (0.0035, 0.281, 0.011, 0),  # inside the two-link inner reach
    (float('nan'), 0.1, 0.15, 90),
    (0.1, float('inf'), 0.15, 90),
    (0.1, 0.1, 0.15, float('-inf')),
])
def test_invalid_poses_are_rejected(pose):
    with pytest.raises(ValueError):
        lab_ik_angles(*pose)
