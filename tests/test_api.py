"""
API tests.

The UI is written against this contract, so these lock down the
shapes it depends on as well as the behaviour.
"""

import pytest
from fastapi.testclient import TestClient

import config


@pytest.fixture(scope="module")
def client(tmp_path_factory):
    # Keep saved poses and calibration out of the source tree.
    import calibration
    import poses
    directory = tmp_path_factory.mktemp("state")
    calibration.CALIBRATION_FILE = str(directory / "calibration.json")
    poses.POSES_FILE = str(directory / "poses.json")

    import main
    with TestClient(main.app) as test_client:
        yield test_client


# ------------------------------------------------------- description

def test_config_describes_the_robot(client):
    cfg = client.get("/api/config").json()
    assert {leg["id"] for leg in cfg["legs"]} == set(config.LEG_IDS)
    assert {g["name"] for g in cfg["gaits"]} == set(config.GAITS)
    assert {m["name"] for m in cfg["moves"]} == {"jump", "bounce", "flex", "dive"}
    assert set(cfg["stances"]) == set(config.STANCE_POSES)
    assert {a["name"] for a in cfg["axis_modes"]} == set(config.AXIS_MODES)
    assert cfg["hardware"]["all_simulated"] is True
    # The 3D view builds the model from these; missing any breaks it.
    for key in ("coxa", "femur", "tibia", "body_radius", "reach_min", "reach_max"):
        assert key in cfg["geometry"]
    for leg in cfg["legs"]:
        assert set(leg["channels"]) == set(config.JOINTS)
        assert len(leg["mount_point"]) == 3


def test_status_has_everything_the_ui_reads(client):
    status = client.get("/api/status").json()
    for section in ("gait", "stance", "body", "behaviour", "override",
                    "move", "joints", "feet", "imu", "power", "health"):
        assert section in status, f"missing '{section}'"
    for leg in config.LEG_IDS:
        assert set(status["joints"][leg]) == set(config.JOINTS)


# ------------------------------------------------------------- drive

def test_drive_and_gait_selection(client):
    assert client.post("/api/gait", json={"name": "wave"}).json()["name"] == "wave"
    gait = client.post("/api/drive", json={"walk_x": 0.8, "turn": -0.2, "speed": 0.7}).json()
    assert gait["walk_x"] == pytest.approx(0.8)
    assert gait["turn"] == pytest.approx(-0.2)
    assert client.get("/api/status").json()["gait"]["name"] == "wave"


def test_unknown_gait_is_rejected(client):
    assert client.post("/api/gait", json={"name": "moonwalk"}).status_code == 404


def test_out_of_range_drive_is_rejected(client):
    assert client.post("/api/drive", json={"speed": 4.0}).status_code == 422
    assert client.post("/api/drive", json={"walk_x": -3.0}).status_code == 422


@pytest.mark.parametrize("name", list(config.STANCE_POSES))
def test_every_stance_is_selectable(name, client):
    assert client.post("/api/stance", json={"name": name}).json()["stance"] == name


# --------------------------------------------------------- body pose

@pytest.mark.parametrize("mode", list(config.AXIS_MODES))
def test_axis_modes_drive_their_own_fields(mode, client):
    client.post("/api/body/center")
    body = client.post("/api/axis", json={"mode": mode, "h": 1.0, "v": 1.0}).json()
    assert body["axis_mode"] == mode
    # Whatever the mode maps, something must have moved.
    assert any(abs(body[k]) > 1e-9 for k in ("x", "y", "z", "roll", "pitch", "yaw"))
    client.post("/api/body/center")


def test_axis_mode_mapping_matches_the_config(client):
    client.post("/api/body/center")
    body = client.post("/api/axis", json={"mode": "uv", "h": 0.5, "v": -0.5}).json()
    assert body["roll"] == pytest.approx(0.5 * config.AXIS_MODES["uv"]["h"]["scale"])
    assert body["pitch"] == pytest.approx(-0.5 * config.AXIS_MODES["uv"]["v"]["scale"])
    assert body["x"] == pytest.approx(0.0)


def test_xy_uv_drives_translation_and_tilt_together(client):
    client.post("/api/body/center")
    body = client.post("/api/axis", json={"mode": "xy_uv", "h": 1.0, "v": 1.0}).json()
    assert abs(body["y"]) > 1.0 and abs(body["roll"]) > 1.0
    client.post("/api/body/center")


def test_centre_body_zeroes_the_pose(client):
    client.post("/api/body", json={"roll": 10, "z": 20})
    body = client.post("/api/body/center").json()
    assert all(body[k] == 0.0 for k in ("x", "y", "z", "roll", "pitch", "yaw"))


# --------------------------------------------------------- behaviour

def test_behaviour_toggles_round_trip(client):
    for flag in ("torque", "hold_pos", "auto_level", "auto_sit"):
        assert client.post("/api/behaviour", json={flag: True}).json()[flag] is True
        assert client.post("/api/behaviour", json={flag: False}).json()[flag] is False


def test_estop_engages_and_clears(client):
    assert client.post("/api/estop").json()["estop"] is True
    assert client.get("/api/status").json()["gait"]["walk_x"] == 0.0
    assert client.post("/api/estop/clear").json()["estop"] is False


@pytest.mark.parametrize("name", ["jump", "bounce", "flex", "dive"])
def test_moves_start(name, client):
    assert client.post(f"/api/move/{name}").json()["active"] == name
    client.post("/api/move/stop")


def test_unknown_move_is_rejected(client):
    assert client.post("/api/move/backflip").status_code == 404


def test_beep(client):
    assert client.post("/api/beep", json={"tone": "ack"}).json()["tone"] == "ack"


# --------------------------------------------------------- per-leg

def test_leg_joints_set_an_override_and_stop_the_walk(client):
    client.post("/api/drive", json={"walk_x": 1.0})
    result = client.post("/api/leg/L1/joints", json={"coxa": 20, "femur": 25, "tibia": 95}).json()
    assert result["joints"]["coxa"] == 20
    status = client.get("/api/status").json()
    assert "L1" in status["override"]["manual_joints"]
    assert status["gait"]["walk_x"] == 0.0
    client.delete("/api/legs/override")


def test_leg_joints_are_clamped_to_the_joint_limits(client):
    result = client.post("/api/leg/L2/joints", json={"coxa": 900, "femur": -900, "tibia": 900}).json()
    for joint, value in result["joints"].items():
        low, high = config.JOINT_LIMITS_DEG[joint]
        assert low <= value <= high
    client.delete("/api/legs/override")


def test_one_joint_can_be_set_on_every_leg(client):
    client.post("/api/leg/L2/joints", json={"coxa": 10, "femur": -90, "tibia": 80})
    result = client.post("/api/legs/joint", json={"joint": "femur", "value": 0}).json()
    assert result["value"] == 0
    held = client.get("/api/status").json()["override"]["manual_joints"]
    assert set(held) == set(config.LEG_IDS)
    for leg in config.LEG_IDS:
        assert held[leg]["femur"] == 0
    # The other joints of a leg are left where they were.
    assert held["L2"]["coxa"] == 10 and held["L2"]["tibia"] == 80
    assert client.post("/api/legs/joint", json={"joint": "elbow", "value": 0}).status_code == 404
    client.delete("/api/legs/override")


def test_leg_foot_goes_through_ik(client):
    result = client.post("/api/leg/R2/foot", json={"x": 190, "y": 30, "z": -100}).json()
    assert result["reachable"] is True
    assert set(result["joints"]) == set(config.JOINTS)
    client.delete("/api/legs/override")


def test_unreachable_foot_is_reported_not_crashed(client):
    result = client.post("/api/leg/R3/foot", json={"x": 900, "y": 0, "z": 0}).json()
    assert result["reachable"] is False
    client.delete("/api/legs/override")


def test_overrides_clear_individually_and_together(client):
    client.post("/api/leg/L1/joints", json={"coxa": 5, "femur": 5, "tibia": 90})
    client.post("/api/leg/L3/foot", json={"x": 170, "y": 0, "z": -110})
    client.delete("/api/leg/L1/override")
    override = client.get("/api/status").json()["override"]
    assert "L1" not in override["manual_joints"] and "L3" in override["manual_feet"]
    client.delete("/api/legs/override")
    override = client.get("/api/status").json()["override"]
    assert not override["manual_joints"] and not override["manual_feet"]


def test_unknown_leg_is_rejected(client):
    assert client.get("/api/leg/Z9").status_code == 404
    assert client.post("/api/leg/Z9/joints", json={"coxa": 0, "femur": 0, "tibia": 0}).status_code == 404


def test_setting_a_stance_releases_overrides(client):
    client.post("/api/leg/L1/joints", json={"coxa": 5, "femur": 5, "tibia": 90})
    client.post("/api/stance", json={"name": "stand"})
    assert not client.get("/api/status").json()["override"]["manual_joints"]


# ------------------------------------------------------ calibration

def test_calibration_updates_and_persists(client):
    client.post("/api/calibration", json={"leg": "L1", "joint": "coxa",
                                          "offset": 93.5, "direction": -1})
    entry = client.get("/api/calibration").json()["calibration"]["L1"]["coxa"]
    assert entry["offset"] == 93.5 and entry["direction"] == -1.0
    assert client.post("/api/calibration/save").json()["saved"] is True
    client.post("/api/calibration/reset")
    assert client.get("/api/calibration").json()["calibration"]["L1"]["coxa"]["direction"] == 1


def test_calibration_direction_is_normalised_to_plus_or_minus_one(client):
    entry = client.post("/api/calibration", json={"leg": "R1", "joint": "femur",
                                                  "direction": 7}).json()["calibration"]
    assert entry["direction"] == 1.0
    client.post("/api/calibration/reset")


def test_unknown_calibration_target_is_rejected(client):
    assert client.post("/api/calibration", json={"leg": "ZZ", "joint": "coxa"}).status_code == 404
    assert client.post("/api/calibration", json={"leg": "L1", "joint": "elbow"}).status_code == 404


# ------------------------------------------------------------ poses

def test_poses_save_load_and_delete(client):
    client.post("/api/stance", json={"name": "wide"})
    client.post("/api/pose/save", json={"name": "test-pose"})
    assert "test-pose" in client.get("/api/pose").json()["poses"]
    client.post("/api/stance", json={"name": "sit"})
    client.post("/api/pose/load/test-pose")
    assert client.get("/api/status").json()["stance"]["name"] == "wide"
    client.delete("/api/pose/test-pose")
    assert "test-pose" not in client.get("/api/pose").json()["poses"]


def test_missing_pose_is_rejected(client):
    assert client.post("/api/pose/load/nope").status_code == 404
    assert client.delete("/api/pose/nope").status_code == 404


# ---------------------------------------------------------- sensors

def test_simulated_tilt_round_trips(client):
    client.post("/api/sim/tilt", json={"roll": 9, "pitch": -5})
    tilt = client.get("/api/sim/tilt").json()
    assert tilt["roll"] == 9 and tilt["pitch"] == -5
    client.post("/api/sim/tilt", json={"roll": 0, "pitch": 0})


def test_tilt_is_range_checked(client):
    assert client.post("/api/sim/tilt", json={"roll": 90}).status_code == 422


def test_servo_channels_are_reported_in_range(client):
    channels = client.get("/api/servos").json()["channels"]
    assert len(channels) >= len(config.LEG_IDS) * 3
    for angle in channels.values():
        assert 0.0 <= angle <= 180.0


def test_imu_level_zeroes_the_reading(client):
    client.post("/api/sim/tilt", json={"roll": 8, "pitch": 4})
    assert client.post("/api/imu/level").json()["levelled"] is True


# ------------------------------------------------------------- ui

def test_the_ui_and_its_assets_are_served(client):
    assert client.get("/").status_code == 200
    for asset in ("/css/app.css", "/js/app.js", "/js/hexapod.js",
                  "/js/render3d.js", "/js/kinematics.js", "/js/math3d.js",
                  "/js/net.js", "/js/ui.js", "/favicon.ico"):
        assert client.get(asset).status_code == 200, asset


# ------------------------------------------------------- websocket

def test_websocket_greets_with_config_and_streams_telemetry(client):
    with client.websocket_connect("/ws/control") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello"
        assert "geometry" in hello["config"]

        ws.send_json({"type": "gait", "name": "ripple"})
        ws.send_json({"type": "drive", "walk_x": 0.5})
        for _ in range(40):
            message = ws.receive_json()
            if message["type"] == "telemetry" and message["gait"]["name"] == "ripple":
                assert message["gait"]["walk_x"] == pytest.approx(0.5)
                break
        else:
            pytest.fail("no telemetry reflected the command")


def test_websocket_leg_override_and_release(client):
    with client.websocket_connect("/ws/control") as ws:
        ws.receive_json()
        ws.send_json({"type": "leg_joints", "leg": "R1", "coxa": 10, "femur": 20, "tibia": 80})
        for _ in range(40):
            message = ws.receive_json()
            if message["type"] == "telemetry" and "R1" in message["override"]["manual_joints"]:
                break
        else:
            pytest.fail("override never appeared in telemetry")

        ws.send_json({"type": "clear_override", "leg": None})
        for _ in range(40):
            message = ws.receive_json()
            if message["type"] == "telemetry" and not message["override"]["manual_joints"]:
                break
        else:
            pytest.fail("override was never released")


def test_websocket_ignores_malformed_frames(client):
    with client.websocket_connect("/ws/control") as ws:
        ws.receive_json()
        ws.send_text("not json at all")
        ws.send_json({"type": "nonsense"})
        ws.send_json({"type": "gait", "name": "does-not-exist"})
        ws.send_json({"type": "ping"})
        assert ws.receive_json()["type"] == "telemetry"


def test_min_max_decide_the_offset_for_every_motor(client):
    # A bad femur offset (29.5 cuts off half the travel) is replaced by
    # the one that centres the kinematic range inside servo min..max.
    client.post("/api/calibration", json={"leg": "L2", "joint": "femur", "offset": 29.5})
    table = client.post("/api/calibration/fit", json={}).json()["calibration"]
    for leg in config.LEG_IDS:
        for joint in config.JOINTS:
            entry = table[leg][joint]
            lo, hi = config.JOINT_LIMITS_DEG[joint]
            ends = sorted((entry["direction"] * lo + entry["offset"],
                           entry["direction"] * hi + entry["offset"]))
            assert entry["min"] - 1e-6 <= ends[0] and ends[1] <= entry["max"] + 1e-6, (leg, joint)
    assert table["L2"]["femur"]["offset"] == 90
    client.post("/api/calibration/reset")


def test_changing_min_or_max_recentres_the_offset(client):
    entry = client.post("/api/calibration", json={
        "leg": "R1", "joint": "femur", "min_deg": 10, "max_deg": 170}).json()["calibration"]
    assert entry["offset"] == 90
    entry = client.post("/api/calibration", json={
        "leg": "L1", "joint": "tibia", "min_deg": 0, "max_deg": 180}).json()["calibration"]
    assert entry["offset"] == 10          # 0..160 centred in 0..180, direction +1
    # An explicit offset is still a trim the user can set afterwards.
    entry = client.post("/api/calibration", json={
        "leg": "L1", "joint": "tibia", "offset": 15}).json()["calibration"]
    assert entry["offset"] == 15
    client.post("/api/calibration/reset")
