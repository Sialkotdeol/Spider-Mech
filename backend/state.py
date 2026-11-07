"""
Shared robot state.

One process, one instance, guarded by a re-entrant lock. Everything
the API reads and the control loop writes lives here, so there is
exactly one description of what the robot is currently trying to do.

Field groups:
  command   - what the operator asked for (gait, body pose, stance)
  behaviour - the standing toggles (torque, hold, auto-level, auto-sit)
  override  - per-leg manual control coming from the 3D view
  feedback  - what the control loop and sensors last measured
"""

import dataclasses
import threading
import time
from typing import Dict, List, Optional

import config

_lock = threading.RLock()


@dataclasses.dataclass
class RobotState:
    # ---- command ----------------------------------------------------
    gait_name: str = config.DEFAULT_GAIT
    walk_x: float = 0.0          # -1..1, forward / back
    walk_y: float = 0.0          # -1..1, strafe left / right
    turn: float = 0.0            # -1..1, yaw rate
    speed: float = 0.5           # 0..1
    step_height: float = config.GAIT_STEP_HEIGHT_MM
    stride: float = 1.0          # 0..1.5 multiplier on stance reach

    stance_name: str = config.DEFAULT_STANCE
    # Where each foot currently rests in its own leg frame; slews
    # toward the selected stance instead of snapping.
    stance_feet: Dict[str, dict] = dataclasses.field(default_factory=dict)

    body_x: float = 0.0
    body_y: float = 0.0
    body_z: float = 0.0
    body_roll: float = 0.0
    body_pitch: float = 0.0
    body_yaw: float = 0.0
    axis_mode: str = config.DEFAULT_AXIS_MODE

    # ---- behaviour --------------------------------------------------
    torque: bool = True
    hold_pos: bool = False
    auto_level: bool = False
    auto_sit: bool = False
    estop: bool = False

    # ---- override ---------------------------------------------------
    # leg_id -> {"coxa","femur","tibia"}: drive this leg's servos
    # directly, bypassing IK. Set by dragging a joint in the 3D view.
    manual_joints: Dict[str, dict] = dataclasses.field(default_factory=dict)
    # leg_id -> {"x","y","z"}: hold this foot at a fixed leg-local
    # point, still through IK. Set by dragging a foot in the 3D view.
    manual_feet: Dict[str, dict] = dataclasses.field(default_factory=dict)

    # ---- move -------------------------------------------------------
    active_move: Optional[str] = None
    move_elapsed: float = 0.0

    # ---- feedback ---------------------------------------------------
    gait_phase: float = 0.0
    walking: bool = False
    # leg_id -> {"coxa","femur","tibia"} kinematic angles actually sent
    joint_angles: Dict[str, dict] = dataclasses.field(default_factory=dict)
    # leg_id -> {"x","y","z"} resolved foot target, leg-local frame
    foot_targets: Dict[str, dict] = dataclasses.field(default_factory=dict)
    grounded: List[str] = dataclasses.field(default_factory=list)
    unreachable: List[str] = dataclasses.field(default_factory=list)

    imu_roll: Optional[float] = None
    imu_pitch: Optional[float] = None
    imu_yaw: Optional[float] = None
    level_roll_correction: float = 0.0
    level_pitch_correction: float = 0.0

    battery_voltage: Optional[float] = None
    battery_current: Optional[float] = None

    last_command_time: float = dataclasses.field(default_factory=time.time)
    loop_hz: float = 0.0
    connected_clients: int = 0

    # ---- helpers ----------------------------------------------------

    def reset_stance_feet(self, stance: Optional[dict] = None) -> None:
        stance = stance or config.STANCE_POSES[self.stance_name]
        self.stance_feet = {leg: dict(stance) for leg in config.LEG_IDS}


state = RobotState()
state.reset_stance_feet()


def lock():
    return _lock


def touch() -> None:
    with _lock:
        state.last_command_time = time.time()


def idle_seconds() -> float:
    with _lock:
        return time.time() - state.last_command_time


def is_stale() -> bool:
    return idle_seconds() > config.CONTROL_TIMEOUT_SECONDS


def snapshot() -> dict:
    """A JSON-serialisable view of everything, for the API and the UI."""
    with _lock:
        s = state
        return {
            "gait": {
                "name": s.gait_name, "walk_x": s.walk_x, "walk_y": s.walk_y,
                "turn": s.turn, "speed": s.speed, "step_height": s.step_height,
                "stride": s.stride, "phase": round(s.gait_phase, 4),
                "walking": s.walking, "grounded": list(s.grounded),
            },
            "stance": {"name": s.stance_name, "feet": dict(s.stance_feet)},
            "body": {
                "x": s.body_x, "y": s.body_y, "z": s.body_z,
                "roll": s.body_roll, "pitch": s.body_pitch, "yaw": s.body_yaw,
                "axis_mode": s.axis_mode,
            },
            "behaviour": {
                "torque": s.torque, "hold_pos": s.hold_pos,
                "auto_level": s.auto_level, "auto_sit": s.auto_sit,
                "estop": s.estop,
            },
            "override": {
                "manual_joints": dict(s.manual_joints),
                "manual_feet": dict(s.manual_feet),
            },
            "move": {"active": s.active_move, "elapsed": round(s.move_elapsed, 3)},
            "joints": dict(s.joint_angles),
            "feet": dict(s.foot_targets),
            "unreachable": list(s.unreachable),
            "imu": {"roll": s.imu_roll, "pitch": s.imu_pitch, "yaw": s.imu_yaw,
                     "level_roll": round(s.level_roll_correction, 2),
                     "level_pitch": round(s.level_pitch_correction, 2)},
            "power": {"voltage": s.battery_voltage, "current": s.battery_current},
            "health": {
                "stale": is_stale(), "idle_seconds": round(idle_seconds(), 1),
                "loop_hz": round(s.loop_hz, 1), "clients": s.connected_clients,
            },
        }
