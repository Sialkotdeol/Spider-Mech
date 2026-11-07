"""
The control loop.

Every tick, in this order:

  1. read sensors (IMU each tick, power at a slower rate)
  2. work out the auto-level correction
  3. slew the stance toward whichever stance pose is selected
  4. advance the gait phase, if a walk command is active
  5. build each foot's target in the body frame
  6. add the active move's overlay
  7. resolve the total body pose
  8. body IK -> leg IK per leg, honouring per-leg manual overrides
  9. rate-limit every joint, calibrate, and write to the servo bus
 10. publish what happened into shared state for the UI

Ordering matters: overrides win over gaits, safety interlocks (e-stop,
hold, torque) win over everything, and the rate limiter sits last so
nothing upstream can command a joint to move faster than a servo can
physically follow.
"""

import logging
import math
import time
from typing import Dict, Optional, Tuple

import numpy as np

import calibration
import config
import gaits
import kinematics as kin
import moves
import state as st
from hardware import Hardware
from hardware.base import ImuSample

logger = logging.getLogger("controller")

POWER_READ_INTERVAL = 0.25   # seconds between power-sensor reads


class Controller:
    def __init__(self, hardware: Hardware):
        self.hw = hardware
        self.running = False
        # Last kinematic angle commanded per joint - the rate limiter's memory.
        self._last_angles: Dict[Tuple[str, str], float] = {}
        self._last_tick = time.time()
        self._last_power_read = 0.0
        self._loop_hz = 0.0
        self._auto_sat_down = False
        self._walk_seconds = 0.0          # how long the current drive input has been held

        with st.lock():
            st.state.reset_stance_feet()

    # ------------------------------------------------------------ sensors

    def _read_sensors(self, now: float) -> Optional[ImuSample]:
        sample = None
        try:
            sample = self.hw.imu.read()
        except Exception:
            logger.exception("IMU read failed")

        if sample is not None:
            with st.lock():
                st.state.imu_roll = round(sample.roll_deg, 2)
                st.state.imu_pitch = round(sample.pitch_deg, 2)
                st.state.imu_yaw = round(sample.yaw_deg, 2)

        if now - self._last_power_read >= POWER_READ_INTERVAL:
            self._last_power_read = now
            try:
                power = self.hw.power.read()
                with st.lock():
                    st.state.battery_voltage = power.voltage
                    st.state.battery_current = power.current
            except Exception:
                logger.exception("power read failed")
        return sample

    def _auto_level(self, sample: Optional[ImuSample], dt: float) -> Tuple[float, float]:
        """
        Proportional correction that rotates the body opposite to the
        measured tilt, so it stays horizontal on a slope. Rate-limited
        and clamped, because a hard correction on a noisy IMU is how a
        robot shakes itself apart.
        """
        s = st.state
        target_roll = target_pitch = 0.0
        if s.auto_level and sample is not None:
            limit = config.AUTO_LEVEL_MAX_CORRECTION_DEG
            target_roll = max(-limit, min(limit, -sample.roll_deg * config.AUTO_LEVEL_GAIN))
            target_pitch = max(-limit, min(limit, -sample.pitch_deg * config.AUTO_LEVEL_GAIN))

        max_step = config.AUTO_LEVEL_SLEW_DEG_PER_S * dt
        roll = _slew(s.level_roll_correction, target_roll, max_step)
        pitch = _slew(s.level_pitch_correction, target_pitch, max_step)
        with st.lock():
            s.level_roll_correction = roll
            s.level_pitch_correction = pitch
        return roll, pitch

    # ------------------------------------------------------------- stance

    def _update_stance(self, dt: float) -> None:
        """Slide the resting foot positions toward the selected stance."""
        s = st.state
        target = config.STANCE_POSES.get(s.stance_name, config.STANCE_POSES[config.DEFAULT_STANCE])
        max_step = config.STANCE_SLEW_MM_PER_S * dt
        with st.lock():
            for leg in config.LEG_IDS:
                current = s.stance_feet.setdefault(leg, dict(target))
                for axis in ("x", "y", "z"):
                    current[axis] = _slew(current[axis], target[axis], max_step)

    # -------------------------------------------------------------- tick

    def tick(self, dt: float) -> None:
        now = time.time()
        s = st.state

        sample = self._read_sensors(now)
        level_roll, level_pitch = self._auto_level(sample, dt)
        self._update_stance(dt)
        self._check_auto_sit()

        # --- safety interlocks, highest priority first ---------------
        if s.estop or not s.torque:
            # Servos are released (or frozen); publish state and stop.
            self._walk_seconds = 0.0
            self._publish(grounded=[], unreachable=[])
            return
        if s.hold_pos:
            # Keep re-sending the last angles so the servos hold under
            # load, but do not recompute anything.
            self._resend_last()
            self._publish(grounded=list(s.grounded), unreachable=[])
            return

        # --- walking -------------------------------------------------
        stale = st.is_stale()
        walk_x, walk_y, turn = (0.0, 0.0, 0.0) if stale else (s.walk_x, s.walk_y, s.turn)
        walking = (abs(walk_x) + abs(walk_y) + abs(turn)) > 0.02

        if walking:
            self._walk_seconds += dt
            ramp = min(1.0, config.GAIT_RAMP_START * math.exp(self._walk_seconds / config.GAIT_RAMP_TAU_S))
            with st.lock():
                s.gait_phase = gaits.advance_phase(s.gait_phase, dt, s.gait_name, s.speed * ramp)
            steps = gaits.all_steps(
                s.gait_phase, s.gait_name, walk_x, walk_y, turn,
                s.step_height, config.GAIT_STANCE_REACH_MM * s.stride)
            grounded = [leg for leg, step in steps.items() if step.grounded]
        else:
            self._walk_seconds = 0.0
            steps, grounded = {}, list(config.LEG_IDS)

        # --- active move overlay -------------------------------------
        frame = self._advance_move(dt)

        # --- total body pose -----------------------------------------
        translation = np.array([
            _clamp(s.body_x, -config.BODY_TRANSLATE_LIMIT_MM, config.BODY_TRANSLATE_LIMIT_MM) + frame.body_x,
            _clamp(s.body_y, -config.BODY_TRANSLATE_LIMIT_MM, config.BODY_TRANSLATE_LIMIT_MM) + frame.body_y,
            _clamp(s.body_z, -config.BODY_Z_LIMIT_MM, config.BODY_Z_LIMIT_MM) + frame.body_z,
        ])
        limit = config.BODY_ANGLE_LIMIT_DEG
        roll = _clamp(s.body_roll, -limit, limit) + frame.roll_deg + level_roll
        pitch = _clamp(s.body_pitch, -limit, limit) + frame.pitch_deg + level_pitch
        yaw = _clamp(s.body_yaw, -limit, limit) + frame.yaw_deg
        translation[2] += battery_lift(translation, roll, pitch, yaw, _ground_z(s))

        # --- per-leg solve -------------------------------------------
        unreachable = []
        joint_angles: Dict[str, dict] = {}
        foot_targets: Dict[str, dict] = {}

        for leg_id in config.LEG_IDS:
            manual = s.manual_joints.get(leg_id)
            if manual:
                # Direct servo control from the 3D view: no IK at all.
                lock = kin.femur_lock()
                angles = kin.LegAngles(float(manual.get("coxa", 0.0)),
                                       lock if lock is not None else float(manual.get("femur", 0.0)),
                                       float(manual.get("tibia", 0.0)))
                foot = kin.leg_fk(angles)
                foot_targets[leg_id] = {"x": round(foot[0], 2), "y": round(foot[1], 2),
                                         "z": round(foot[2], 2)}
            else:
                local = s.manual_feet.get(leg_id) or s.stance_feet[leg_id]
                foot_body = kin.leg_local_to_body(
                    leg_id, np.array([local["x"], local["y"], local["z"]], dtype=float))

                step = steps.get(leg_id)
                if step is not None:
                    foot_body = foot_body + np.array([step.dx, step.dy, step.dz])
                extra = frame.feet.get(leg_id)
                if extra:
                    foot_body = foot_body + np.array(extra, dtype=float)

                target_local = kin.body_ik(leg_id, foot_body, translation, roll, pitch, yaw)
                angles = kin.leg_ik(target_local[0], target_local[1], target_local[2])
                foot_targets[leg_id] = {"x": round(float(target_local[0]), 2),
                                         "y": round(float(target_local[1]), 2),
                                         "z": round(float(target_local[2]), 2)}
                if not angles.reachable:
                    unreachable.append(leg_id)

            self._write_leg(leg_id, angles, dt)
            joint_angles[leg_id] = {
                "coxa": round(angles.coxa_deg, 2),
                "femur": round(angles.femur_deg, 2),
                "tibia": round(angles.tibia_deg, 2),
            }

        self.hw.servos.flush(move_time_ms=200)

        with st.lock():
            s.joint_angles = joint_angles
            s.foot_targets = foot_targets
            s.walking = walking
        self._publish(grounded=grounded, unreachable=unreachable)

    # --------------------------------------------------------- internals

    def _write_leg(self, leg_id: str, angles: kin.LegAngles, dt: float) -> None:
        """Rate-limit, calibrate, and queue one leg's three servos."""
        max_step = config.SERVO_MAX_SLEW_DEG_PER_S * dt
        for joint, value in (("coxa", angles.coxa_deg),
                             ("femur", angles.femur_deg),
                             ("tibia", angles.tibia_deg)):
            key = (leg_id, joint)
            previous = self._last_angles.get(key)
            limited = value if previous is None else _slew(previous, value, max_step)
            self._last_angles[key] = limited
            setattr(angles, f"{joint}_deg", limited)
            self.hw.servos.set_angle(calibration.channel(leg_id, joint),
                                     calibration.to_servo_angle(leg_id, joint, limited))

    def _resend_last(self) -> None:
        for (leg_id, joint), value in self._last_angles.items():
            self.hw.servos.set_angle(calibration.channel(leg_id, joint),
                                     calibration.to_servo_angle(leg_id, joint, value))
        self.hw.servos.flush(move_time_ms=200)	

    def _advance_move(self, dt: float) -> moves.MoveFrame:
        """
        Step the active move by dt. Move time accumulates from the same
        dt the rest of the loop integrates, rather than from the wall
        clock, so a move stays in step with the gait even if the loop
        is running slow.
        """
        s = st.state
        if not s.active_move:
            return moves.MoveFrame()
        move = moves.get(s.active_move)
        if move is None:
            with st.lock():
                s.active_move = None
                s.move_elapsed = 0.0
            return moves.MoveFrame()

        with st.lock():
            s.move_elapsed += dt
            elapsed = s.move_elapsed
        if move.finished(elapsed):
            with st.lock():
                s.active_move = None
                s.move_elapsed = 0.0
            return moves.MoveFrame()
        return move.frame_at(elapsed)

    def _check_auto_sit(self) -> None:
        """Sit down and release torque after a long idle, then wake on command."""
        s = st.state
        if not s.auto_sit or config.AUTO_SIT_IDLE_SECONDS <= 0:
            if self._auto_sat_down:
                self._auto_sat_down = False
            return
        idle = st.idle_seconds()
        if not self._auto_sat_down and idle > config.AUTO_SIT_IDLE_SECONDS:
            logger.info("auto-sit: idle for %.0fs, sitting down", idle)
            with st.lock():
                s.stance_name = "sit"
                s.walk_x = s.walk_y = s.turn = 0.0
            self._auto_sat_down = True
        elif self._auto_sat_down and idle < config.AUTO_SIT_IDLE_SECONDS:
            self._auto_sat_down = False

    def _publish(self, grounded, unreachable) -> None:
        with st.lock():
            st.state.grounded = grounded
            st.state.unreachable = unreachable
            st.state.loop_hz = self._loop_hz
        # Feed the simulated power monitor a sense of how hard the robot
        # is working, so its readings respond to what you actually do.
        activity = getattr(self.hw.power, "activity", None)
        if activity is not None:
            load = 0.0
            if st.state.walking:
                load = 0.35 + 0.65 * min(1.0, st.state.speed)
            if st.state.active_move:
                load = max(load, 0.8)
            if not st.state.torque:
                load = 0.0
            self.hw.power.activity = load

    # ------------------------------------------------------------ actions

    def start_move(self, name: str) -> bool:
        move = moves.get(name)
        if move is None:
            return False
        with st.lock():
            st.state.active_move = name
            st.state.move_elapsed = 0.0
            if move.requires_ground:
                st.state.walk_x = st.state.walk_y = st.state.turn = 0.0
        st.touch()
        return True

    def stop_move(self) -> None:
        with st.lock():
            st.state.active_move = None
            st.state.move_elapsed = 0.0

    def set_torque(self, enabled: bool) -> None:
        with st.lock():
            st.state.torque = bool(enabled)
        try:
            self.hw.servos.set_torque(bool(enabled))
        except Exception:
            logger.exception("set_torque failed")
        if enabled:
            # Forget the rate limiter's history so re-energised servos
            # ease back to their target instead of snapping.
            self._last_angles.clear()
        st.touch()

    def estop(self) -> None:
        with st.lock():
            s = st.state
            s.estop = True
            s.walk_x = s.walk_y = s.turn = 0.0
            s.active_move = None
            s.move_elapsed = 0.0
        self.beep("estop")
        logger.warning("E-STOP engaged")

    def clear_estop(self) -> None:
        with st.lock():
            st.state.estop = False
        self._last_angles.clear()
        st.touch()
        logger.info("E-stop cleared")

    def beep(self, tone: str = "beep") -> None:
        tones = config.BEEP_TONES.get(tone, config.BEEP_TONES["beep"])
        try:
            self.hw.buzzer.play(tones)
        except Exception:
            logger.exception("buzzer failed")

    def calibrate_level(self) -> None:
        try:
            self.hw.imu.calibrate_level()
        except Exception:
            logger.exception("IMU level calibration failed")

    # ---------------------------------------------------------- main loop

    async def run(self) -> None:
        import asyncio

        self.running = True
        period = 1.0 / config.TICK_RATE_HZ
        self._last_tick = time.time()
        logger.info("control loop running at %d Hz", config.TICK_RATE_HZ)

        while self.running:
            start = time.time()
            dt = min(max(start - self._last_tick, 1e-4), 0.25)
            self._last_tick = start
            self._loop_hz = 1.0 / dt
            try:
                self.tick(dt)
            except Exception:
                logger.exception("control loop error")
            await asyncio.sleep(max(0.0, period - (time.time() - start)))

    def stop(self) -> None:
        self.running = False


def _ground_z(s) -> float:
    """Ground height in the body frame: where the standing feet are."""
    feet = [foot["z"] for foot in s.stance_feet.values()]
    return sum(feet) / len(feet) if feet else config.STANCE_POSES[config.DEFAULT_STANCE]["z"]


def battery_lift(translation, roll: float, pitch: float, yaw: float, ground_z: float) -> float:
    """How far the body must rise (mm) so the lowest corner of the
    battery stays BATTERY_CLEARANCE_MM above the ground. 0 when it
    already clears. Tilting the body swings one end of the battery
    down, so the check uses all four bottom corners."""
    R = kin.rotation_matrix(roll, pitch, yaw)
    hl, hw = config.BATTERY_HALF_LENGTH_MM, config.BATTERY_HALF_WIDTH_MM
    lowest = min(
        (R @ np.array([x, y, -config.BATTERY_DROP_MM]))[2]
        for x in (-hl, hl) for y in (-hw, hw)
    ) + translation[2]
    return max(0.0, ground_z + config.BATTERY_CLEARANCE_MM - lowest)


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else (hi if value > hi else value)


def _slew(current: float, target: float, max_step: float) -> float:
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)
