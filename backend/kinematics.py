"""
Kinematics for Spider Mech.

Three layers:

1. leg_ik(x, y, z) - per-leg 3-DOF analytic IK. The target is in the
   leg's own local frame: origin at the coxa mount, x = outward from
   the body, y = forward/back, z = up (feet sit at negative z).

2. leg_fk(angles) / leg_chain(angles) - forward kinematics. leg_fk
   returns just the foot; leg_chain returns every joint position along
   the leg, which is what the 3D viewer draws.

3. body_ik(...) - applies a desired body pose (translation + roll/
   pitch/yaw) to a foot target that is fixed in the body frame, so the
   body can tilt, level and shift while the feet stay planted.

Angles use a plain mathematical convention: 0 degrees means the
segment points straight out along its rest direction, positive rotates
up / counter-clockwise. Servo calibration (per-joint direction and
offset) is applied separately in hardware/servo_map.py - deliberately
NOT baked in here, so this module stays testable independent of any
one robot's mounting quirks.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

import config


@dataclass
class LegAngles:
    coxa_deg: float
    femur_deg: float
    tibia_deg: float
    reachable: bool = True   # False if the target was outside the workspace
    limited: bool = False    # True if a joint limit clamped the result

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.coxa_deg, self.femur_deg, self.tibia_deg)

    def as_dict(self) -> dict:
        return {"coxa": self.coxa_deg, "femur": self.femur_deg, "tibia": self.tibia_deg}


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


# How far (mm) a locked-femur foot may land from its requested x/y
# before the target is reported as out of reach.
LOCKED_FEMUR_REACH_TOLERANCE_MM = 50.0


def femur_lock():
    """The kinematic angle every femur is held at, or None if unlocked."""
    lock = getattr(config, "FEMUR_LOCK_DEG", None)
    return None if lock is None else float(lock)


def _leg_ik_femur_locked(x: float, y: float, z: float, femur_deg: float,
                         apply_limits: bool) -> LegAngles:
    """
    IK with the femur fixed at `femur_deg`. The coxa still aims the leg
    at the target and the tibia sets the foot height; the foot's
    distance from the body follows from those two, so it can differ
    from the requested x/y (reported through `limited`).
    """
    coxa_rad = math.atan2(y, x)
    f = math.radians(femur_deg)
    knee_z = config.FEMUR_LENGTH_MM * math.sin(f)
    drop = (z - knee_z) / config.TIBIA_LENGTH_MM      # sin of the shin's angle
    reachable = -1.0 <= drop <= 1.0
    shin = math.asin(_clamp(drop, -1.0, 1.0))        # shin angle from horizontal, pointing out
    angles = LegAngles(
        coxa_deg=math.degrees(coxa_rad),
        femur_deg=femur_deg,
        tibia_deg=math.degrees(f - shin),
        reachable=reachable,
    )
    limited = False
    if apply_limits:
        for name, attr in (("coxa", "coxa_deg"), ("tibia", "tibia_deg")):
            lo_lim, hi_lim = config.JOINT_LIMITS_DEG[name]
            value = getattr(angles, attr)
            capped = _clamp(value, lo_lim, hi_lim)
            if abs(capped - value) > 1e-9:
                limited = True
                setattr(angles, attr, capped)
    # The foot cannot be placed closer or further than the locked
    # geometry allows. Small misses are normal (a raised foot swings
    # outward) and only count as `limited`; a big miss means the target
    # is really out of reach.
    foot = leg_fk(angles)
    miss = math.hypot(foot[0] - x, foot[1] - y)
    if miss > 1.0:
        limited = True
    if miss > LOCKED_FEMUR_REACH_TOLERANCE_MM:
        angles.reachable = False
    angles.limited = limited
    return angles


def leg_ik(x: float, y: float, z: float, apply_limits: bool = True) -> LegAngles:
    """Solve 3-DOF IK for one leg. x, y, z in mm, in the leg-local frame."""
    lock = femur_lock()
    if lock is not None:
        return _leg_ik_femur_locked(x, y, z, lock, apply_limits)
    coxa_rad = math.atan2(y, x)

    horiz = math.hypot(x, y) - config.COXA_LENGTH_MM
    dist = math.hypot(horiz, z)

    # Clamp the target into the annulus the leg can actually reach.
    reachable = True
    lo = config.MIN_FEMUR_TO_FOOT_MM + 1e-6
    hi = config.MAX_FEMUR_TO_FOOT_MM - 1e-6
    clamped = _clamp(dist, lo, hi)
    if abs(clamped - dist) > 1e-6:
        reachable = False
    dist = clamped

    femur = config.FEMUR_LENGTH_MM
    tibia = config.TIBIA_LENGTH_MM

    # Angle of the femur-joint-to-foot line above horizontal.
    angle_to_foot = math.atan2(z, horiz)

    # Law of cosines, inputs clamped to [-1, 1] to survive floating
    # point drift right at the edge of the workspace.
    cos_femur_offset = (femur ** 2 + dist ** 2 - tibia ** 2) / (2 * femur * dist)
    femur_offset = math.acos(_clamp(cos_femur_offset, -1.0, 1.0))

    cos_knee = (femur ** 2 + tibia ** 2 - dist ** 2) / (2 * femur * tibia)
    knee_internal = math.acos(_clamp(cos_knee, -1.0, 1.0))

    angles = LegAngles(
        coxa_deg=math.degrees(coxa_rad),
        femur_deg=math.degrees(angle_to_foot + femur_offset),
        tibia_deg=math.degrees(math.pi - knee_internal),
        reachable=reachable,
    )

    if apply_limits:
        limited = False
        for name, attr in (("coxa", "coxa_deg"), ("femur", "femur_deg"), ("tibia", "tibia_deg")):
            lo_lim, hi_lim = config.JOINT_LIMITS_DEG[name]
            value = getattr(angles, attr)
            capped = _clamp(value, lo_lim, hi_lim)
            if abs(capped - value) > 1e-9:
                limited = True
                setattr(angles, attr, capped)
        angles.limited = limited

    return angles


def leg_chain(angles: LegAngles) -> List[np.ndarray]:
    """
    Every joint position along one leg, in the leg-local frame:
    [coxa origin, femur joint, tibia joint (knee), foot].

    This is the geometry the 3D viewer draws, and it is the same
    formula leg_fk uses - so what you see on screen is exactly what
    the IK believes the leg is doing.
    """
    c = math.radians(angles.coxa_deg)
    f = math.radians(angles.femur_deg)
    t = math.radians(angles.tibia_deg)

    cos_c, sin_c = math.cos(c), math.sin(c)

    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([config.COXA_LENGTH_MM * cos_c,
                   config.COXA_LENGTH_MM * sin_c,
                   0.0])

    femur_h = config.FEMUR_LENGTH_MM * math.cos(f)
    femur_z = config.FEMUR_LENGTH_MM * math.sin(f)
    p2 = p1 + np.array([femur_h * cos_c, femur_h * sin_c, femur_z])

    # The tibia hangs at (femur angle - knee angle) from horizontal.
    tibia_h = config.TIBIA_LENGTH_MM * math.cos(f - t)
    tibia_z = config.TIBIA_LENGTH_MM * math.sin(f - t)
    p3 = p2 + np.array([tibia_h * cos_c, tibia_h * sin_c, tibia_z])

    return [p0, p1, p2, p3]


def leg_fk(angles: LegAngles) -> Tuple[float, float, float]:
    """Foot position in the leg-local frame. The inverse of leg_ik."""
    return tuple(leg_chain(angles)[3])


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Body rotation matrix, R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, yw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(yw), math.sin(yw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def yaw_matrix(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def leg_mount_point(leg_id: str) -> np.ndarray:
    """Position of a leg's coxa mount in the body frame (mm)."""
    angle = math.radians(config.LEG_MOUNT_ANGLES_DEG[leg_id])
    r = config.BODY_RADIUS_MM
    return np.array([r * math.cos(angle), r * math.sin(angle), 0.0])


def leg_local_to_body(leg_id: str, local: np.ndarray) -> np.ndarray:
    """Convert a point in a leg's local frame into the body frame."""
    R = yaw_matrix(config.LEG_MOUNT_ANGLES_DEG[leg_id])
    return R @ np.asarray(local, dtype=float) + leg_mount_point(leg_id)


def body_to_leg_local(leg_id: str, body_point: np.ndarray) -> np.ndarray:
    """Convert a body-frame point into a leg's local frame."""
    R = yaw_matrix(config.LEG_MOUNT_ANGLES_DEG[leg_id])
    return R.T @ (np.asarray(body_point, dtype=float) - leg_mount_point(leg_id))


def default_foot_body(leg_id: str, stance: dict) -> np.ndarray:
    """Where a leg's foot rests, in the body frame, for a given stance."""
    local = np.array([stance["x"], stance["y"], stance["z"]], dtype=float)
    return leg_local_to_body(leg_id, local)


def body_ik(leg_id: str,
            foot_position_body: np.ndarray,
            body_translation: np.ndarray,
            roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Given a foot position fixed in the body frame and a desired body
    pose, return that target in the leg's LOCAL frame (what leg_ik
    expects). This is the layer that lets the body tilt, level and
    shift while the feet stay planted on the ground.
    """
    R = rotation_matrix(roll_deg, pitch_deg, yaw_deg)
    mount_world = R @ leg_mount_point(leg_id) + np.asarray(body_translation, dtype=float)
    # Bring the foot into the rotated body frame, then into the leg's
    # own heading so leg_ik's x-axis really is "outward".
    in_body = R.T @ (np.asarray(foot_position_body, dtype=float) - mount_world)
    return yaw_matrix(config.LEG_MOUNT_ANGLES_DEG[leg_id]).T @ in_body


def solve_leg(leg_id: str,
              foot_position_body: np.ndarray,
              body_translation: np.ndarray,
              roll_deg: float, pitch_deg: float, yaw_deg: float) -> LegAngles:
    """body_ik followed by leg_ik - the full chain for one leg."""
    local = body_ik(leg_id, foot_position_body, body_translation,
                    roll_deg, pitch_deg, yaw_deg)
    return leg_ik(local[0], local[1], local[2])
