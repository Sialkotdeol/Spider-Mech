"""
Spider Mech control server.

    uvicorn main:app --host 0.0.0.0 --port 8000

Serves the web UI at / and exposes the robot over REST and a
WebSocket. The WebSocket is the one to use for anything continuous
(driving, dragging a servo in the 3D view) - REST is there for
one-shot commands and for scripting the robot from the shell.

Every endpoint works identically whether the servos are real or
simulated; nothing in this file knows which.
"""

import asyncio
import json
import logging
import os
import time
from contextlib import asynccontextmanager
from typing import List

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles

import api_models as models
import calibration
import config
import gaits
import gamepad
import kinematics as kin
import moves
import poses
import state as st
from controller import Controller
from hardware import create_hardware

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")

hardware = create_hardware()
controller = Controller(hardware)

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "frontend")


@asynccontextmanager
async def lifespan(app: FastAPI):
    task = asyncio.create_task(controller.run())
    telemetry = asyncio.create_task(_telemetry_loop())
    pad = asyncio.create_task(gamepad.run(_handle_pad_message)) if config.GAMEPAD_ENABLED else None
    logger.info("Spider Mech ready - hardware: %s", hardware.backends)
    for warning in hardware.warnings:
        logger.warning("  %s", warning)
    try:
        yield
    finally:
        controller.stop()
        for t in (task, telemetry, pad):
            if t is not None:
                t.cancel()
        hardware.close()
        logger.info("Spider Mech stopped")


app = FastAPI(title="Spider Mech Control", version="2.0", lifespan=lifespan)


@app.middleware("http")
async def no_stale_ui(request, call_next):
    """Make the browser re-check the UI files on every load, so a copy of
    new code to the Pi always reaches the page. Without this, a cached
    app.js can keep running against a newer index.html."""
    response = await call_next(request)
    if not request.url.path.startswith("/api"):
        response.headers["Cache-Control"] = "no-cache"
    return response
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


# ============================================================ description

@app.get("/api/config")
def get_config():
    """
    Everything the UI needs to draw and drive this specific robot.
    The 3D view builds its model from this, so changing config.py
    changes the on-screen hexapod with no front-end edits.
    """
    return {
        "geometry": {
            "coxa": config.COXA_LENGTH_MM,
            "femur": config.FEMUR_LENGTH_MM,
            "tibia": config.TIBIA_LENGTH_MM,
            "body_radius": config.BODY_RADIUS_MM,
            "body_thickness": config.BODY_PLATE_THICKNESS_MM,
            "reach_min": config.MIN_FEMUR_TO_FOOT_MM,
            "reach_max": config.MAX_FEMUR_TO_FOOT_MM,
        },
        "legs": [
            {
                "id": leg,
                "mount_angle": config.LEG_MOUNT_ANGLES_DEG[leg],
                "mount_point": list(kin.leg_mount_point(leg)),
                "channels": config.SERVO_CHANNELS[leg],
                "tripod": "A" if leg in config.TRIPOD_A else "B",
                "side": "left" if leg.startswith("L") else "right",
            }
            for leg in config.LEG_IDS
        ],
        "joints": list(config.JOINTS),
        "joint_limits": {k: list(v) for k, v in config.JOINT_LIMITS_DEG.items()},
        "femur_lock": kin.femur_lock(),
        "gaits": gaits.catalog(),
        "moves": moves.catalog(),
        "stances": {name: dict(pose) for name, pose in config.STANCE_POSES.items()},
        "axis_modes": [
            {"name": name, "label": mode["label"], "description": mode["description"],
             "h": _axis_summary(mode["h"]), "v": _axis_summary(mode["v"])}
            for name, mode in config.AXIS_MODES.items()
        ],
        "limits": {
            "body_translate": config.BODY_TRANSLATE_LIMIT_MM,
            "body_z": config.BODY_Z_LIMIT_MM,
            "body_angle": config.BODY_ANGLE_LIMIT_DEG,
            "step_height": 90.0,
        },
        "hardware": {
            "backends": hardware.backends,
            "warnings": hardware.warnings,
            "all_simulated": hardware.all_simulated,
            "supports_torque": hardware.servos.supports_torque_control,
            "supports_feedback": hardware.servos.supports_feedback,
        },
        "tick_rate": config.TICK_RATE_HZ,
    }


def _axis_summary(entry):
    items = entry if isinstance(entry, list) else [entry]
    return [{"field": i["field"], "scale": i["scale"], "unit": i["unit"]} for i in items]


@app.get("/api/status")
def get_status():
    return st.snapshot()


# ================================================================= drive

@app.post("/api/drive")
def set_drive(body: models.DriveBody):
    with st.lock():
        s = st.state
        for field in ("walk_x", "walk_y", "turn", "speed", "step_height", "stride"):
            value = getattr(body, field)
            if value is not None:
                setattr(s, field, float(value))
    st.touch()
    return st.snapshot()["gait"]


@app.post("/api/gait")
def set_gait(body: models.GaitBody):
    if body.name not in config.GAITS:
        raise HTTPException(404, f"unknown gait '{body.name}'")
    with st.lock():
        st.state.gait_name = body.name
    st.touch()
    return gaits.describe(body.name)


@app.post("/api/stance")
def set_stance(body: models.StanceBody):
    if body.name not in config.STANCE_POSES:
        raise HTTPException(404, f"unknown stance '{body.name}'")
    with st.lock():
        st.state.stance_name = body.name
        # A stance change means the operator is present; clear any
        # manual overrides so the whole robot moves as one.
        st.state.manual_joints.clear()
        st.state.manual_feet.clear()
    st.touch()
    return {"stance": body.name, "target": config.STANCE_POSES[body.name]}


# ============================================================== body pose

@app.post("/api/body")
def set_body(body: models.BodyPoseBody):
    with st.lock():
        s = st.state
        for field, attr in (("x", "body_x"), ("y", "body_y"), ("z", "body_z"),
                            ("roll", "body_roll"), ("pitch", "body_pitch"), ("yaw", "body_yaw")):
            value = getattr(body, field)
            if value is not None:
                setattr(s, attr, float(value))
    st.touch()
    return st.snapshot()["body"]


@app.post("/api/axis")
def set_axis(body: models.AxisBody):
    """Apply a two-axis pad reading through the selected axis mode."""
    mode_name = body.mode or st.state.axis_mode
    if mode_name not in config.AXIS_MODES:
        raise HTTPException(404, f"unknown axis mode '{mode_name}'")
    _apply_axis(mode_name, body.h, body.v)
    return st.snapshot()["body"]


def _apply_axis(mode_name: str, h: float, v: float) -> None:
    mode = config.AXIS_MODES[mode_name]
    with st.lock():
        st.state.axis_mode = mode_name
        for reading, entry in ((h, mode["h"]), (v, mode["v"])):
            for item in (entry if isinstance(entry, list) else [entry]):
                setattr(st.state, item["field"], reading * item["scale"])
    st.touch()


@app.post("/api/axis/mode/{mode_name}")
def set_axis_mode(mode_name: str):
    if mode_name not in config.AXIS_MODES:
        raise HTTPException(404, f"unknown axis mode '{mode_name}'")
    with st.lock():
        st.state.axis_mode = mode_name
    st.touch()
    return {"axis_mode": mode_name}


@app.post("/api/body/center")
def center_body():
    """Return the body to a neutral pose without touching the feet."""
    with st.lock():
        s = st.state
        s.body_x = s.body_y = s.body_z = 0.0
        s.body_roll = s.body_pitch = s.body_yaw = 0.0
    st.touch()
    return st.snapshot()["body"]


# ============================================================== behaviour

@app.post("/api/behaviour")
def set_behaviour(body: models.BehaviourBody):
    if body.torque is not None:
        controller.set_torque(body.torque)
    with st.lock():
        s = st.state
        if body.hold_pos is not None:
            s.hold_pos = bool(body.hold_pos)
        if body.auto_level is not None:
            s.auto_level = bool(body.auto_level)
        if body.auto_sit is not None:
            s.auto_sit = bool(body.auto_sit)
    st.touch()
    return st.snapshot()["behaviour"]


@app.post("/api/move/{name}")
def start_move(name: str):
    if name == "stop":
        controller.stop_move()
        return {"active": None}
    if not controller.start_move(name):
        raise HTTPException(404, f"unknown move '{name}'")
    return {"active": name}


@app.post("/api/beep")
def beep(body: models.BeepBody = models.BeepBody()):
    controller.beep(body.tone)
    st.touch()
    return {"tone": body.tone}


@app.post("/api/estop")
def estop():
    controller.estop()
    return st.snapshot()["behaviour"]


@app.post("/api/estop/clear")
def clear_estop():
    controller.clear_estop()
    return st.snapshot()["behaviour"]


# =================================================== per-leg manual control

@app.post("/api/leg/{leg_id}/joints")
def set_leg_joints(leg_id: str, body: models.JointsBody):
    """
    Drive one leg's three servos directly - this is what dragging a
    joint in the 3D view calls. The other five legs keep doing
    whatever they were doing.
    """
    _require_leg(leg_id)
    angles = {}
    for joint, value in (("coxa", body.coxa), ("femur", body.femur), ("tibia", body.tibia)):
        lo, hi = config.JOINT_LIMITS_DEG[joint]
        angles[joint] = max(lo, min(hi, float(value)))
    if kin.femur_lock() is not None:
        angles["femur"] = kin.femur_lock()
    with st.lock():
        st.state.manual_joints[leg_id] = angles
        st.state.manual_feet.pop(leg_id, None)
        # Direct joint control and walking must not fight each other.
        st.state.walk_x = st.state.walk_y = st.state.turn = 0.0
    st.touch()
    foot = kin.leg_fk(kin.LegAngles(angles["coxa"], angles["femur"], angles["tibia"]))
    return {"leg": leg_id, "joints": angles,
            "foot": {"x": round(foot[0], 2), "y": round(foot[1], 2), "z": round(foot[2], 2)}}


@app.post("/api/leg/{leg_id}/foot")
def set_leg_foot(leg_id: str, body: models.FootBody):
    """Hold one foot at a leg-local point, solved through IK."""
    _require_leg(leg_id)
    angles = kin.leg_ik(body.x, body.y, body.z)
    with st.lock():
        st.state.manual_feet[leg_id] = {"x": body.x, "y": body.y, "z": body.z}
        st.state.manual_joints.pop(leg_id, None)
        st.state.walk_x = st.state.walk_y = st.state.turn = 0.0
    st.touch()
    return {"leg": leg_id, "foot": {"x": body.x, "y": body.y, "z": body.z},
            "joints": angles.as_dict(), "reachable": angles.reachable, "limited": angles.limited}


@app.get("/api/leg/{leg_id}")
def get_leg(leg_id: str):
    _require_leg(leg_id)
    snapshot = st.snapshot()
    return {
        "leg": leg_id,
        "joints": snapshot["joints"].get(leg_id, {}),
        "foot": snapshot["feet"].get(leg_id, {}),
        "channels": config.SERVO_CHANNELS[leg_id],
        "calibration": calibration.table()[leg_id],
        "override": ("joints" if leg_id in st.state.manual_joints
                      else "foot" if leg_id in st.state.manual_feet else None),
    }


@app.delete("/api/leg/{leg_id}/override")
def clear_leg_override(leg_id: str):
    _require_leg(leg_id)
    with st.lock():
        st.state.manual_joints.pop(leg_id, None)
        st.state.manual_feet.pop(leg_id, None)
    st.touch()
    return {"leg": leg_id, "override": None}


@app.post("/api/legs/joint")
def set_joint_on_all_legs(body: models.AllLegsJointBody):
    """
    Set one joint (e.g. every femur) to the same kinematic angle on all
    six legs. The other two joints of each leg stay where they are, so
    "all femurs to 0°" puts every thigh horizontal without moving the
    hips or knees. Each leg is held there until released.
    """
    if body.joint not in config.JOINTS:
        raise HTTPException(404, f"unknown joint '{body.joint}'")
    lo, hi = config.JOINT_LIMITS_DEG[body.joint]
    value = max(lo, min(hi, float(body.value)))
    if body.joint == "femur" and kin.femur_lock() is not None:
        value = kin.femur_lock()
    with st.lock():
        for leg_id in config.LEG_IDS:
            current = (st.state.manual_joints.get(leg_id)
                       or st.state.joint_angles.get(leg_id)
                       or {j: 0.0 for j in config.JOINTS})
            angles = {j: float(current.get(j, 0.0)) for j in config.JOINTS}
            angles[body.joint] = value
            if kin.femur_lock() is not None:
                angles["femur"] = kin.femur_lock()
            st.state.manual_joints[leg_id] = angles
            st.state.manual_feet.pop(leg_id, None)
        st.state.walk_x = st.state.walk_y = st.state.turn = 0.0
        result = {leg: dict(st.state.manual_joints[leg]) for leg in config.LEG_IDS}
    st.touch()
    return {"joint": body.joint, "value": value, "joints": result}


@app.delete("/api/legs/override")
def clear_all_overrides():
    with st.lock():
        st.state.manual_joints.clear()
        st.state.manual_feet.clear()
    st.touch()
    return {"cleared": True}


@app.post("/api/ik")
def solve_ik(body: models.FootBody):
    """Solve IK for a leg-local point without commanding anything."""
    angles = kin.leg_ik(body.x, body.y, body.z)
    return {"joints": angles.as_dict(), "reachable": angles.reachable, "limited": angles.limited}


# ============================================================ calibration

@app.get("/api/calibration")
def get_calibration():
    return {"calibration": calibration.table(), "channels": config.SERVO_CHANNELS}


@app.post("/api/calibration")
def update_calibration(body: models.CalibrationBody):
    try:
        entry = calibration.update(body.leg, body.joint, body.offset,
                                   body.direction, body.min, body.max)
    except KeyError as exc:
        raise HTTPException(404, str(exc))
    return {"leg": body.leg, "joint": body.joint, "calibration": entry}


@app.post("/api/calibration/fit")
def fit_calibration(body: models.CalibrationFitBody):
    """Recentre offsets from servo min/max (all 18 motors if no leg/joint given)."""
    try:
        return {"calibration": calibration.fit(body.leg, body.joint)}
    except KeyError as exc:
        raise HTTPException(404, str(exc))


@app.post("/api/calibration/save")
def save_calibration():
    calibration.save()
    return {"saved": True, "path": calibration.CALIBRATION_FILE}


@app.post("/api/calibration/reset")
def reset_calibration():
    calibration.reset()
    return {"calibration": calibration.table()}


# ================================================================== poses

@app.get("/api/pose")
def list_poses():
    return {"poses": poses.list_poses()}


@app.post("/api/pose/save")
def save_pose(body: models.PoseSaveBody):
    return {"saved": poses.save_pose(body.name, st.snapshot())}


@app.post("/api/pose/load/{name}")
def load_pose(name: str):
    pose = poses.get_pose(name)
    if pose is None:
        raise HTTPException(404, f"no saved pose '{name}'")
    poses.apply_pose(pose)
    st.touch()
    return {"loaded": name}


@app.delete("/api/pose/{name}")
def delete_pose(name: str):
    if not poses.delete_pose(name):
        raise HTTPException(404, f"no saved pose '{name}'")
    return {"deleted": name}


# =================================================== sensors and simulation

@app.post("/api/imu/level")
def zero_imu():
    """Tell the IMU that however the robot is sitting right now is level."""
    controller.calibrate_level()
    return {"levelled": True}


@app.get("/api/sim/tilt")
def get_sim_tilt():
    """Read the simulated ground tilt back, so the UI's sliders show
    the tilt that is actually in effect after a page reload."""
    imu = hardware.imu
    if not hasattr(imu, "set_ground_tilt"):
        return {"simulated": False, "roll": 0.0, "pitch": 0.0}
    return {"simulated": True,
            "roll": imu.ground_roll_deg, "pitch": imu.ground_pitch_deg}


@app.post("/api/sim/tilt")
def sim_tilt(body: models.TiltBody):
    """
    Tilt the simulated ground plane. Only meaningful with the
    simulated IMU - it is how you test auto-level with no robot.
    """
    setter = getattr(hardware.imu, "set_ground_tilt", None)
    if setter is None:
        raise HTTPException(400, "the active IMU backend is real hardware; tilt the robot instead")
    setter(body.roll, body.pitch)
    st.touch()
    return {"roll": body.roll, "pitch": body.pitch}


@app.get("/api/servos")
def get_servos():
    """Raw servo channel angles, as last written to the bus."""
    snapshot = getattr(hardware.servos, "snapshot", None)
    channels = snapshot() if snapshot else {
        ch: hardware.servos.read_angle(ch) for ch in range(hardware.servos.channel_count)
    }
    return {"channels": channels, "backend": hardware.servos.name}


# =============================================================== WebSocket

_clients: List[WebSocket] = []


@app.websocket("/ws/control")
async def ws_control(ws: WebSocket):
    await ws.accept()
    _clients.append(ws)
    with st.lock():
        st.state.connected_clients = len(_clients)
    st.touch()
    try:
        await ws.send_text(json.dumps({"type": "hello", "config": get_config()}))
        while True:
            raw = await ws.receive_text()
            try:
                message = json.loads(raw)
            except json.JSONDecodeError:
                continue
            # One malformed command (e.g. a null or non-numeric value)
            # must not drop the operator's link to the robot.
            try:
                _handle_message(message)
            except (TypeError, ValueError, AttributeError) as exc:
                logger.warning("ignored bad websocket message %r: %s", raw[:200], exc)
    except WebSocketDisconnect:
        pass
    except Exception:
        logger.exception("websocket error")
    finally:
        if ws in _clients:
            _clients.remove(ws)
        with st.lock():
            st.state.connected_clients = len(_clients)


def _handle_message(message: dict) -> None:
    kind = message.get("type")
    s = st.state

    if kind == "drive":
        with st.lock():
            for field in ("walk_x", "walk_y", "turn", "speed", "step_height", "stride"):
                if field in message:
                    setattr(s, field, float(message[field]))
        st.touch()

    elif kind == "body":
        with st.lock():
            for field, attr in (("x", "body_x"), ("y", "body_y"), ("z", "body_z"),
                                ("roll", "body_roll"), ("pitch", "body_pitch"),
                                ("yaw", "body_yaw")):
                if field in message:
                    setattr(s, attr, float(message[field]))
        st.touch()

    elif kind == "axis":
        mode = message.get("mode") or s.axis_mode
        if mode in config.AXIS_MODES:
            _apply_axis(mode, float(message.get("h", 0.0)), float(message.get("v", 0.0)))

    elif kind == "gait":
        if message.get("name") in config.GAITS:
            with st.lock():
                s.gait_name = message["name"]
            st.touch()

    elif kind == "stance":
        if message.get("name") in config.STANCE_POSES:
            with st.lock():
                s.stance_name = message["name"]
                s.manual_joints.clear()
                s.manual_feet.clear()
            st.touch()

    elif kind == "leg_joints":
        leg_id = message.get("leg")
        if leg_id in config.LEG_IDS:
            angles = {}
            for joint in config.JOINTS:
                lo, hi = config.JOINT_LIMITS_DEG[joint]
                angles[joint] = max(lo, min(hi, float(message.get(joint, 0.0))))
            if kin.femur_lock() is not None:
                angles["femur"] = kin.femur_lock()
            with st.lock():
                s.manual_joints[leg_id] = angles
                s.manual_feet.pop(leg_id, None)
                s.walk_x = s.walk_y = s.turn = 0.0
            st.touch()

    elif kind == "leg_foot":
        leg_id = message.get("leg")
        if leg_id in config.LEG_IDS:
            with st.lock():
                s.manual_feet[leg_id] = {"x": float(message.get("x", 0.0)),
                                          "y": float(message.get("y", 0.0)),
                                          "z": float(message.get("z", 0.0))}
                s.manual_joints.pop(leg_id, None)
                s.walk_x = s.walk_y = s.turn = 0.0
            st.touch()

    elif kind == "clear_override":
        leg_id = message.get("leg")
        with st.lock():
            if leg_id in config.LEG_IDS:
                s.manual_joints.pop(leg_id, None)
                s.manual_feet.pop(leg_id, None)
            else:
                s.manual_joints.clear()
                s.manual_feet.clear()
        st.touch()

    elif kind == "behaviour":
        if "torque" in message:
            controller.set_torque(bool(message["torque"]))
        with st.lock():
            for field in ("hold_pos", "auto_level", "auto_sit"):
                if field in message:
                    setattr(s, field, bool(message[field]))
        st.touch()

    elif kind == "move":
        name = message.get("name")
        if name == "stop":
            controller.stop_move()
        else:
            controller.start_move(name)

    elif kind == "beep":
        controller.beep(message.get("tone", "beep"))
        st.touch()

    elif kind == "estop":
        controller.estop()

    elif kind == "estop_clear":
        controller.clear_estop()

    elif kind == "ping":
        st.touch()


def _handle_pad_message(message: dict) -> None:
    """The PS5 controller speaks the same messages as the web UI."""
    try:
        _handle_message(message)
    except (TypeError, ValueError, AttributeError) as exc:
        logger.warning("ignored bad controller message %r: %s", message, exc)


async def _telemetry_loop() -> None:
    period = 1.0 / config.TELEMETRY_RATE_HZ
    while True:
        await asyncio.sleep(period)
        if not _clients:
            continue
        payload = json.dumps({"type": "telemetry", **st.snapshot()})
        for ws in list(_clients):
            try:
                await ws.send_text(payload)
            except Exception:
                if ws in _clients:
                    _clients.remove(ws)


# ================================================================ static UI

def _require_leg(leg_id: str) -> None:
    if leg_id not in config.LEG_IDS:
        raise HTTPException(404, f"unknown leg '{leg_id}' (have {', '.join(config.LEG_IDS)})")


@app.get("/favicon.ico", include_in_schema=False)
def favicon():
    """Browsers probe /favicon.ico regardless of the <link> tag; answer
    it with the SVG rather than letting it 404 in the console."""
    icon = os.path.join(FRONTEND_DIR, "favicon.svg")
    if os.path.exists(icon):
        return FileResponse(icon, media_type="image/svg+xml")
    return HTMLResponse(status_code=404)


@app.get("/")
def serve_ui():
    index = os.path.join(FRONTEND_DIR, "index.html")
    if os.path.exists(index):
        return FileResponse(index)
    return HTMLResponse("<h1>Spider Mech</h1><p>frontend/index.html not found.</p>", status_code=404)


if os.path.isdir(FRONTEND_DIR):
    app.mount("/", StaticFiles(directory=FRONTEND_DIR, html=True), name="frontend")
