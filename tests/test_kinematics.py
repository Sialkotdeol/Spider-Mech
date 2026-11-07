"""
Kinematics tests.

The round-trip test is the important one: if leg_ik and leg_fk ever
disagree, the 3D view stops describing what the servos are doing, and
every calibration you tuned against it becomes wrong.
"""

import math
import random

import numpy as np
import pytest

import config
import kinematics as kin


@pytest.fixture
def free_femur(monkeypatch):
    """Tests of the general 3-DOF solver run with the femur unlocked."""
    monkeypatch.setattr(config, "FEMUR_LOCK_DEG", None)


def test_ik_fk_round_trip(free_femur):
    """IK then FK must return the point we asked for."""
    random.seed(20240501)
    worst = 0.0
    for _ in range(5000):
        x = random.uniform(90, 215)
        y = random.uniform(-70, 70)
        z = random.uniform(-165, -20)
        angles = kin.leg_ik(x, y, z, apply_limits=False)
        if not angles.reachable:
            continue
        worst = max(worst, math.dist((x, y, z), kin.leg_fk(angles)))
    assert worst < 1e-6, f"worst round-trip error {worst} mm"


def test_leg_chain_ends_at_the_foot():
    angles = kin.leg_ik(170, 15, -120)
    assert np.allclose(kin.leg_chain(angles)[3], kin.leg_fk(angles))


def test_leg_chain_segment_lengths():
    """Each drawn segment must be its real physical length."""
    angles = kin.leg_ik(180, -25, -100)
    p0, p1, p2, p3 = kin.leg_chain(angles)
    assert math.dist(p0, p1) == pytest.approx(config.COXA_LENGTH_MM)
    assert math.dist(p1, p2) == pytest.approx(config.FEMUR_LENGTH_MM)
    assert math.dist(p2, p3) == pytest.approx(config.TIBIA_LENGTH_MM)


def test_unreachable_targets_are_flagged_and_clamped(free_femur):
    far = kin.leg_ik(600, 0, 0, apply_limits=False)
    assert not far.reachable
    near = kin.leg_ik(config.COXA_LENGTH_MM, 0, 0, apply_limits=False)
    assert not near.reachable
    # Clamped, not NaN - a NaN here would reach the servos.
    for angles in (far, near):
        for value in angles.as_tuple():
            assert math.isfinite(value)


def test_joint_limits_are_enforced():
    angles = kin.leg_ik(80, 90, -20)
    for joint, value in zip(config.JOINTS, angles.as_tuple()):
        low, high = config.JOINT_LIMITS_DEG[joint]
        assert low - 1e-9 <= value <= high + 1e-9


@pytest.mark.parametrize("leg_id", config.LEG_IDS)
def test_body_ik_identity_at_neutral_pose(leg_id):
    """With the body at rest, body_ik must be a no-op."""
    stance = config.STANCE_POSES["stand"]
    foot_body = kin.default_foot_body(leg_id, stance)
    local = kin.body_ik(leg_id, foot_body, np.zeros(3), 0, 0, 0)
    assert np.allclose(local, [stance["x"], stance["y"], stance["z"]], atol=1e-9)


@pytest.mark.parametrize("leg_id", config.LEG_IDS)
def test_body_frame_conversions_are_inverses(leg_id):
    point = np.array([170.0, 20.0, -110.0])
    assert np.allclose(kin.body_to_leg_local(leg_id, kin.leg_local_to_body(leg_id, point)), point)


def test_body_translation_moves_feet_the_opposite_way():
    """Raising the body must extend the legs, not lift the feet."""
    stance = config.STANCE_POSES["stand"]
    foot_body = kin.default_foot_body("L1", stance)
    lifted = kin.body_ik("L1", foot_body, np.array([0.0, 0.0, 30.0]), 0, 0, 0)
    assert lifted[2] == pytest.approx(stance["z"] - 30.0)


def test_rotation_matrix_is_orthonormal():
    R = kin.rotation_matrix(12, -8, 25)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)


@pytest.mark.parametrize("stance", list(config.STANCE_POSES))
def test_every_stance_is_reachable(stance):
    pose = config.STANCE_POSES[stance]
    angles = kin.leg_ik(pose["x"], pose["y"], pose["z"])
    assert angles.reachable and not angles.limited, f"stance '{stance}' is out of reach"


# ------------------------------------------------------ femur lock

def test_locked_femur_is_always_zero_and_keeps_foot_height(monkeypatch):
    monkeypatch.setattr(config, "FEMUR_LOCK_DEG", 0.0)
    for z in (-20, -60, -90, -120):
        angles = kin.leg_ik(200, 20, z)
        assert angles.femur_deg == 0.0
        assert kin.leg_fk(angles)[2] == pytest.approx(z, abs=1e-6)


def test_locked_femur_still_flags_far_targets(monkeypatch):
    monkeypatch.setattr(config, "FEMUR_LOCK_DEG", 0.0)
    assert not kin.leg_ik(600, 0, 0).reachable
    assert not kin.leg_ik(config.COXA_LENGTH_MM, 0, 0).reachable


def test_every_stance_has_the_femur_at_its_lock():
    if config.FEMUR_LOCK_DEG is None:
        pytest.skip("femur not locked")
    for pose in config.STANCE_POSES.values():
        angles = kin.leg_ik(pose["x"], pose["y"], pose["z"])
        assert angles.femur_deg == config.FEMUR_LOCK_DEG
        assert angles.reachable and not angles.limited
