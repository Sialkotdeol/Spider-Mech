"""
Named pose storage.

A pose captures what the robot is doing - stance, body pose, gait
selection and any per-leg overrides - so a position you tuned by hand
in the 3D view can be recalled later. Stored as plain JSON next to the
code, so it is easy to inspect, edit, copy to another robot, or commit.
"""

import json
import logging
import os
import threading
from typing import Dict, List, Optional

import config
import state as st

logger = logging.getLogger("poses")

POSES_FILE = os.path.join(os.path.dirname(__file__), "poses.json")
_lock = threading.Lock()


def _read() -> Dict[str, dict]:
    if not os.path.exists(POSES_FILE):
        return {}
    try:
        with open(POSES_FILE) as handle:
            data = json.load(handle)
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        logger.exception("could not read %s", POSES_FILE)
        return {}


def _write(data: Dict[str, dict]) -> None:
    tmp = POSES_FILE + ".tmp"
    with open(tmp, "w") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
    os.replace(tmp, POSES_FILE)


def list_poses() -> List[str]:
    with _lock:
        return sorted(_read().keys())


def save_pose(name: str, snapshot: dict) -> str:
    entry = {
        "stance": snapshot["stance"]["name"],
        "stance_feet": snapshot["stance"]["feet"],
        "body": snapshot["body"],
        "gait": snapshot["gait"]["name"],
        "manual_joints": snapshot["override"]["manual_joints"],
        "manual_feet": snapshot["override"]["manual_feet"],
    }
    with _lock:
        data = _read()
        data[name] = entry
        _write(data)
    logger.info("saved pose '%s'", name)
    return name
def get_pose(name: str) -> Optional[dict]:
    with _lock:
        return _read().get(name)
def delete_pose(name: str) -> bool:
    with _lock:
        data = _read()
        if name not in data:
            return False
        del data[name]
        _write(data)
    return True
def apply_pose(pose: dict) -> None:
    with st.lock():
        s = st.state
        if pose.get("stance") in config.STANCE_POSES:
            s.stance_name = pose["stance"]
        feet = pose.get("stance_feet") or {}
        for leg in config.LEG_IDS:
            if leg in feet:
                s.stance_feet[leg] = dict(feet[leg])
        body = pose.get("body") or {}
        for field, attr in (("x", "body_x"), ("y", "body_y"), ("z", "body_z"),
                            ("roll", "body_roll"), ("pitch", "body_pitch"), ("yaw", "body_yaw")):
            if field in body:
                setattr(s, attr, float(body[field]))
        if pose.get("gait") in config.GAITS:
            s.gait_name = pose["gait"]
        s.manual_joints = {k: dict(v) for k, v in (pose.get("manual_joints") or {}).items()
                            if k in config.LEG_IDS}
        s.manual_feet = {k: dict(v) for k, v in (pose.get("manual_feet") or {}).items()
                          if k in config.LEG_IDS}
        s.walk_x = s.walk_y = s.turn = 0.0
