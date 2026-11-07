"""
Control-loop tests.

Everything here runs against the simulated hardware, which is the
point: the safety interlocks, auto-level and per-leg overrides can all
be proved correct before a single servo is wired up.
"""

import math

import numpy as np
import pytest

import calibration
import config
import controller as ctrl
import kinematics as kin
import moves
import state as st
from hardware import create_hardware


@pytest.fixture
def robot():
    calibration.reset()
    st.state.__init__()          # fresh defaults
    st.state.reset_stance_feet()
    hardware = create_hardware()
    controller = ctrl.Controller(hardware)
    controller.settle = lambda ticks=150, dt=0.02: [
        (st.touch(), controller.tick(dt)) for _ in range(ticks)]
    controller.settle()
    yield controller
    hardware.close()


def servo_angles(robot):
    return robot.hw.servos.snapshot()


# ------------------------------------------------------------- basics

def test_settles_into_the_default_stance(robot):
    stance = config.STANCE_POSES[config.DEFAULT_STANCE]
    for leg in config.LEG_IDS:
        foot = st.state.foot_targets[leg]
        assert foot["x"] == pytest.approx(stance["x"], abs=0.5)
        assert foot["z"] == pytest.approx(stance["z"], abs=0.5)


def test_no_servo_is_ever_commanded_out_of_range(robot):
    for name in config.GAITS:
        st.state.gait_name = name
        st.state.walk_x = 1.0
        st.state.turn = 0.4
        robot.settle(240)
        for channel, angle in servo_angles(robot).items():
            assert 0.0 <= angle <= 180.0, f"{name}: channel {channel} at {angle}"


@pytest.mark.parametrize("name", list(config.GAITS))
def test_walking_never_leaves_the_workspace(name, robot):
    st.state.gait_name = name
    st.state.walk_x, st.state.walk_y, st.state.turn = 0.8, 0.4, 0.3
    seen = set()
    for _ in range(400):
        st.touch()
        robot.tick(0.02)
        seen.update(st.state.unreachable)
    assert not seen, f"{name} drove legs out of reach: {sorted(seen)}"


def test_gait_phase_advances_only_while_walking(robot):
    st.state.walk_x = 0.0
    robot.settle(60)
    phase = st.state.gait_phase
    robot.settle(60)
    assert st.state.gait_phase == phase
    st.state.walk_x = 1.0
    robot.settle(60)
    assert st.state.gait_phase != phase


# ----------------------------------------------------------- watchdog

def test_stale_link_stops_the_robot(robot):
    """
    The watchdog deliberately measures real elapsed seconds, not
    accumulated loop dt: if the control loop itself stalls, we still
    want motion cut. So this backdates the last-command timestamp
    rather than stepping a simulated clock.
    """
    st.state.walk_x = 1.0
    robot.settle(30)
    assert st.state.walking

    st.state.last_command_time -= config.CONTROL_TIMEOUT_SECONDS + 0.5
    assert st.is_stale()
    for _ in range(10):
        robot.tick(0.02)
    assert not st.state.walking

    # And it recovers the moment a command arrives.
    st.touch()
    robot.settle(30)
    assert st.state.walking


def test_auto_sit_after_a_long_idle(robot):
    st.state.auto_sit = True
    st.state.stance_name = "stand"
    robot.settle(30)
    assert st.state.stance_name == "stand"

    st.state.last_command_time -= config.AUTO_SIT_IDLE_SECONDS + 1.0
    robot.tick(0.02)
    assert st.state.stance_name == "sit"

    # A fresh command wakes it, and it does not sit again immediately.
    st.touch()
    st.state.stance_name = "stand"
    robot.settle(30)
    assert st.state.stance_name == "stand"


def test_auto_sit_stays_off_when_disabled(robot):
    st.state.auto_sit = False
    st.state.last_command_time -= config.AUTO_SIT_IDLE_SECONDS + 1.0
    robot.tick(0.02)
    assert st.state.stance_name == config.DEFAULT_STANCE


# -------------------------------------------------------- interlocks

def test_hold_pos_freezes_the_joints(robot):
    frozen = dict(st.state.joint_angles["L1"])
    st.state.hold_pos = True
    st.state.walk_x = 1.0
    robot.settle(120)
    assert st.state.joint_angles["L1"] == frozen


def test_torque_off_stops_driving_the_servos(robot):
    robot.set_torque(False)
    before = servo_angles(robot)
    st.state.walk_x = 1.0
    robot.settle(120)
    assert servo_angles(robot) == before
    robot.set_torque(True)
    robot.settle(120)
    assert servo_angles(robot) != before


def test_estop_stops_and_can_be_cleared(robot):
    st.state.walk_x = 1.0
    robot.settle(40)
    robot.estop()
    assert st.state.estop and st.state.walk_x == 0.0
    frozen = servo_angles(robot)
    robot.settle(60)
    assert servo_angles(robot) == frozen
    robot.clear_estop()
    assert not st.state.estop


def test_slew_limit_caps_joint_speed(robot):
    """A step command must be ramped, never sent straight through."""
    st.state.stance_name = "sit"
    robot.settle(400)
    st.state.stance_name = "wide"
    previous = dict(st.state.joint_angles)
    dt = 0.02
    for _ in range(120):
        st.touch()
        robot.tick(dt)
        for leg in config.LEG_IDS:
            for joint in config.JOINTS:
                delta = abs(st.state.joint_angles[leg][joint] - previous[leg][joint])
                assert delta <= config.SERVO_MAX_SLEW_DEG_PER_S * dt + 1e-6
        previous = {leg: dict(v) for leg, v in st.state.joint_angles.items()}


# -------------------------------------------------------- auto-level

def test_auto_level_counters_a_tilted_floor(robot):
    st.state.auto_level = True
    robot.hw.imu.set_ground_tilt(10.0, -6.0)
    robot.settle(400)
    assert st.state.level_roll_correction < -4.0
    assert st.state.level_pitch_correction > 2.0


def test_auto_level_off_means_no_correction(robot):
    robot.hw.imu.set_ground_tilt(10.0, -6.0)
    st.state.auto_level = False
    robot.settle(400)
    assert st.state.level_roll_correction == pytest.approx(0.0, abs=1e-6)


def test_auto_level_correction_is_bounded(robot):
    st.state.auto_level = True
    robot.hw.imu.set_ground_tilt(34.0, 34.0)
    robot.settle(600)
    limit = config.AUTO_LEVEL_MAX_CORRECTION_DEG
    assert abs(st.state.level_roll_correction) <= limit + 1e-6
    assert abs(st.state.level_pitch_correction) <= limit + 1e-6


# ------------------------------------------------------------- moves

@pytest.mark.parametrize("name", ["jump", "bounce", "dive"])
def test_moves_run_and_finish(name, robot):
    assert robot.start_move(name)
    ticks = 0
    while st.state.active_move and ticks < 600:
        st.touch()
        robot.tick(0.02)
        ticks += 1
    assert st.state.active_move is None
    robot.settle(200)
    stance = config.STANCE_POSES[config.DEFAULT_STANCE]
    assert st.state.foot_targets["L1"]["z"] == pytest.approx(stance["z"], abs=1.0)


def test_flex_loops_until_stopped(robot):
    robot.start_move("flex")
    robot.settle(600)
    assert st.state.active_move == "flex"
    robot.stop_move()
    assert st.state.active_move is None


def test_a_move_stops_the_robot_walking(robot):
    st.state.walk_x = 1.0
    robot.settle(30)
    robot.start_move("jump")
    assert st.state.walk_x == 0.0


def test_jump_moves_the_body_further_than_stance_alone(robot):
    baseline = st.state.foot_targets["L1"]["z"]
    robot.start_move("jump")
    lowest = highest = baseline
    while st.state.active_move:
        st.touch()
        robot.tick(0.02)
        z = st.state.foot_targets["L1"]["z"]
        lowest, highest = min(lowest, z), max(highest, z)
    assert highest - lowest > 80


# --------------------------------------------------------- overrides

def test_manual_joints_bypass_ik_for_one_leg_only(robot):
    others = {leg: dict(st.state.joint_angles[leg]) for leg in config.LEG_IDS if leg != "L1"}
    st.state.manual_joints["L1"] = {"coxa": 15.0, "femur": 30.0, "tibia": 70.0}
    robot.settle(300)
    lock = kin.femur_lock()
    expected_femur = 30.0 if lock is None else lock      # a locked femur ignores the command
    assert st.state.joint_angles["L1"] == {"coxa": 15.0, "femur": expected_femur, "tibia": 70.0}
    for leg, angles in others.items():
        assert st.state.joint_angles[leg] == angles


def test_manual_foot_is_solved_through_ik(robot):
    st.state.manual_feet["R2"] = {"x": 190.0, "y": 30.0, "z": -100.0}
    robot.settle(300)
    expected = kin.leg_ik(190.0, 30.0, -100.0)
    assert st.state.joint_angles["R2"]["femur"] == pytest.approx(expected.femur_deg, abs=0.05)


def test_clearing_an_override_returns_the_leg_to_stance(robot):
    st.state.manual_joints["L1"] = {"coxa": 20.0, "femur": 40.0, "tibia": 60.0}
    robot.settle(200)
    st.state.manual_joints.clear()
    robot.settle(400)
    stance = config.STANCE_POSES[config.DEFAULT_STANCE]
    assert st.state.foot_targets["L1"]["x"] == pytest.approx(stance["x"], abs=0.5)


# ------------------------------------------------------- calibration

def test_calibration_maps_kinematics_to_servo_units(robot):
    robot.settle(150)
    angles = st.state.joint_angles["L1"]
    channels = config.SERVO_CHANNELS["L1"]
    servos = servo_angles(robot)
    for joint in config.JOINTS:
        expected = calibration.to_servo_angle("L1", joint, angles[joint])
        assert servos[channels[joint]] == pytest.approx(expected, abs=0.6)


def test_flipping_direction_mirrors_the_servo(robot):
    calibration.update("L1", "coxa", offset=90, direction=1)
    forward = calibration.to_servo_angle("L1", "coxa", 30.0)
    calibration.update("L1", "coxa", direction=-1)
    mirrored = calibration.to_servo_angle("L1", "coxa", 30.0)
    assert forward - 90 == pytest.approx(-(mirrored - 90))
    calibration.reset()


def test_default_calibration_fits_every_joint_inside_servo_travel():
    """
    The kinematic limits must map inside 0-180 with no clamping, or a
    joint parks against its mechanical stop on the very first command.
    """
    for leg in config.LEG_IDS:
        for joint in config.JOINTS:
            entry = calibration.get(leg, joint)
            for limit in config.JOINT_LIMITS_DEG[joint]:
                raw = entry["direction"] * limit + entry["offset"]
                assert 0.0 <= raw <= 180.0, f"{leg}.{joint} maps {limit} to {raw}"


# ------------------------------------------------------ battery clearance

def battery_gap(legs=None):
    """Air gap (mm) under the battery's lowest corner, measured from the
    plane through the standing feet as the legs actually solved them."""
    feet = [kin.leg_local_to_body(leg, np.array([f["x"], f["y"], f["z"]]))
            for leg, f in st.state.foot_targets.items() if legs is None or leg in legs]
    centre = np.mean(feet, axis=0)
    # Ground plane through the feet (least squares), normal pointing up.
    normal = np.linalg.svd(np.array(feet) - centre)[2][-1]
    if normal[2] < 0:
        normal = -normal
    hl, hw = config.BATTERY_HALF_LENGTH_MM, config.BATTERY_HALF_WIDTH_MM
    corners = [np.array([x, y, -config.BATTERY_DROP_MM]) for x in (-hl, hl) for y in (-hw, hw)]
    return min(float(normal @ (c - centre)) for c in corners)


@pytest.mark.parametrize("name", list(config.STANCE_POSES))
def test_every_stance_keeps_the_battery_off_the_ground(name, robot):
    st.state.stance_name = name
    robot.settle(400)
    assert battery_gap() >= config.BATTERY_CLEARANCE_MM - 0.5
    assert not st.state.unreachable


@pytest.mark.parametrize("pose", [
    {"body_z": -config.BODY_Z_LIMIT_MM},
    {"body_pitch": config.BODY_ANGLE_LIMIT_DEG},
    {"body_pitch": -config.BODY_ANGLE_LIMIT_DEG},
    {"body_roll": config.BODY_ANGLE_LIMIT_DEG},
    {"body_z": -config.BODY_Z_LIMIT_MM, "body_pitch": config.BODY_ANGLE_LIMIT_DEG},
])
@pytest.mark.parametrize("stance", ["sit", "crouch", "stand"])
def test_body_pose_cannot_put_the_battery_on_the_ground(pose, stance, robot):
    st.state.stance_name = stance
    robot.settle(400)
    for field, value in pose.items():
        setattr(st.state, field, value)
    robot.settle(50)
    assert battery_gap() >= config.BATTERY_CLEARANCE_MM - 0.5


def test_battery_lift_is_zero_when_there_is_room():
    assert ctrl.battery_lift(np.zeros(3), 0.0, 0.0, 0.0, -110.0) == 0.0
    # Sitting at exactly the minimum height needs no lift either.
    floor = -(config.BATTERY_DROP_MM + config.BATTERY_CLEARANCE_MM)
    assert ctrl.battery_lift(np.zeros(3), 0.0, 0.0, 0.0, floor) == pytest.approx(0.0, abs=1e-9)
    assert ctrl.battery_lift(np.zeros(3), 0.0, 0.0, 0.0, floor + 20.0) == pytest.approx(20.0)


@pytest.mark.parametrize("name", list(moves.MOVES))
def test_moves_never_drop_the_battery_onto_the_ground(name, robot):
    assert robot.start_move(name)
    worst = float("inf")
    for _ in range(400):
        st.touch()
        robot.tick(0.02)
        if len(st.state.grounded) == len(config.LEG_IDS):
            worst = min(worst, battery_gap())
    assert worst >= config.BATTERY_CLEARANCE_MM - 1.0


def test_walking_keeps_the_battery_clear(robot):
    st.state.walk_x = 1.0
    worst = float("inf")
    for _ in range(300):
        st.touch()
        robot.tick(0.02)
        worst = min(worst, battery_gap(st.state.grounded))
    assert worst >= config.BATTERY_CLEARANCE_MM - 0.5
