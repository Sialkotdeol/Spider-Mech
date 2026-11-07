"""
Servo calibration: the bridge between kinematic angles and servo units.

Kinematics produces angles in a clean mathematical convention. A real
servo needs a different number, because it was mounted at some angle,
possibly mirrored, and has mechanical stops. That mapping lives here
and nowhere else:

    servo_angle = direction * kinematic_angle + offset,   clamped

Calibration is persisted to calibration.json so it survives restarts,
and is edited live from the UI's Calibrate tab - you should never have
to hand-edit config.py once the robot is physically built.
"""

import json
import logging
import os
import threading
from typing import Dict, Optional

import config

logger = logging.getLogger("calibration")

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "calibration.json")

_lock = threading.RLock()


def _default_table() -> Dict[str, Dict[str, dict]]:
    return {
        leg: {joint: dict(values) for joint, values in joints.items()}
        for leg, joints in config.JOINT_CALIBRATION.items()
    }


_table: Dict[str, Dict[str, dict]] = _default_table()


def load() -> Dict[str, Dict[str, dict]]:
    """Load calibration.json over the defaults, keeping unknown legs out."""
    global _table
    with _lock:
        _table = _default_table()
        if not os.path.exists(CALIBRATION_FILE):
            return _table
        try:
            with open(CALIBRATION_FILE) as handle:
                saved = json.load(handle)
        except (OSError, json.JSONDecodeError):
            logger.exception("could not read %s - using defaults", CALIBRATION_FILE)
            return _table

        for leg, joints in saved.items():
            if leg not in _table or not isinstance(joints, dict):
                continue
            for joint, values in joints.items():
                if joint not in _table[leg] or not isinstance(values, dict):
                    continue
                for key in ("offset", "direction", "min", "max"):
                    if key in values:
                        _table[leg][joint][key] = float(values[key])
        logger.info("loaded calibration from %s", CALIBRATION_FILE)
        return _table


def save() -> None:
    with _lock:
        tmp = CALIBRATION_FILE + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(_table, handle, indent=2, sort_keys=True)
        os.replace(tmp, CALIBRATION_FILE)   # atomic: never a half-written file
        logger.info("saved calibration to %s", CALIBRATION_FILE)


def table() -> Dict[str, Dict[str, dict]]:
    with _lock:
        return {leg: {joint: dict(v) for joint, v in joints.items()}
                for leg, joints in _table.items()}


def get(leg_id: str, joint: str) -> dict:
    with _lock:
        return dict(_table[leg_id][joint])


def fitted_offset(joint: str, direction: float, min_deg: float, max_deg: float) -> float:
    """
    The offset that centres the joint's whole kinematic range (from
    config.JOINT_LIMITS_DEG) inside servo min..max. With it, servo min
    and max decide where the joint can go: every kinematic angle the
    3D model can show lands on the servo, nothing is cut off, as long
    as max - min is at least as wide as the kinematic range.
    """
    lo, hi = config.JOINT_LIMITS_DEG[joint]
    return round((min_deg + max_deg) / 2.0 - direction * (lo + hi) / 2.0, 2)


def fit(leg_id: Optional[str] = None, joint: Optional[str] = None) -> Dict[str, Dict[str, dict]]:
    """Recentre the offset from min/max for one joint, one leg, or all 18."""
    with _lock:
        for leg in ([leg_id] if leg_id else list(_table)):
            if leg not in _table:
                raise KeyError(f"unknown leg '{leg}'")
            for j in ([joint] if joint else list(_table[leg])):
                if j not in _table[leg]:
                    raise KeyError(f"unknown joint '{j}'")
                e = _table[leg][j]
                e["offset"] = fitted_offset(j, e["direction"], e["min"], e["max"])
        return table()


def update(leg_id: str, joint: str, offset: Optional[float] = None,
           direction: Optional[float] = None,
           min_deg: Optional[float] = None, max_deg: Optional[float] = None) -> dict:
    """
    Change one joint's calibration. Changing min, max or direction
    (without also giving an offset) recentres the offset from min/max,
    so min and max always decide the joint's servo range. Set the
    offset afterwards to trim the neutral pose.
    """
    if leg_id not in _table:
        raise KeyError(f"unknown leg '{leg_id}'")
    if joint not in _table[leg_id]:
        raise KeyError(f"unknown joint '{joint}'")
    with _lock:
        entry = _table[leg_id][joint]
        if offset is not None:
            entry["offset"] = float(offset)
        if direction is not None:
            # Only +1 or -1 is meaningful; anything else would silently
            # scale the kinematics and make the 3D view disagree with the robot.
            entry["direction"] = 1.0 if float(direction) >= 0 else -1.0
        if min_deg is not None:
            entry["min"] = float(min_deg)
        if max_deg is not None:
            entry["max"] = float(max_deg)
        if entry["min"] > entry["max"]:
            entry["min"], entry["max"] = entry["max"], entry["min"]
        if offset is None and (min_deg is not None or max_deg is not None
                               or direction is not None):
            entry["offset"] = fitted_offset(joint, entry["direction"],
                                            entry["min"], entry["max"])
        return dict(entry)


def reset(leg_id: Optional[str] = None) -> None:
    global _table
    with _lock:
        defaults = _default_table()
        if leg_id is None:
            _table = defaults
        else:
            _table[leg_id] = defaults[leg_id]


def to_servo_angle(leg_id: str, joint: str, kinematic_deg: float) -> float:
    """Map a kinematic joint angle to a clamped servo angle."""
    with _lock:
        entry = _table[leg_id][joint]
        angle = entry["direction"] * kinematic_deg + entry["offset"]
        return max(entry["min"], min(entry["max"], angle))


def to_kinematic_angle(leg_id: str, joint: str, servo_deg: float) -> float:
    """Inverse of to_servo_angle - used when reading servo feedback."""
    with _lock:
        entry = _table[leg_id][joint]
        return (servo_deg - entry["offset"]) / entry["direction"]


def channel(leg_id: str, joint: str) -> int:
    return config.SERVO_CHANNELS[leg_id][joint]


load()
