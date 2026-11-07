"""
PS5 controller mapping tests. The mapper takes raw Linux input events,
so these run on any machine, with no controller and no evdev.
"""

import pytest

import config
import gamepad as gp
import state as st


@pytest.fixture(autouse=True)
def fresh_state():
    st.state.__init__()


def test_centred_pad_sends_nothing():
    pad = gp.PadMapper()
    for code in gp.STICKS:
        pad.event(gp.EV_ABS, code, 128)
    assert pad.tick() == []


def test_left_stick_up_walks_forward_and_release_stops():
    pad = gp.PadMapper()
    pad.event(gp.EV_ABS, gp.ABS_Y, 0)            # stick fully up
    (drive,) = pad.tick()
    assert drive["type"] == "drive"
    assert drive["walk_x"] == pytest.approx(1.0, abs=0.01)
    assert drive["walk_y"] == 0.0 and drive["turn"] == 0.0
    # Held still: keeps re-sending so the control timeout never fires.
    assert pad.tick()[0]["walk_x"] > 0.9
    pad.event(gp.EV_ABS, gp.ABS_Y, 128)
    assert pad.tick() == [{"type": "drive", "walk_x": 0.0, "walk_y": 0.0, "turn": 0.0}]
    assert pad.tick() == []


def test_left_stick_left_strafes_left_and_right_stick_left_turns_left():
    pad = gp.PadMapper()
    pad.event(gp.EV_ABS, gp.ABS_X, 0)
    pad.event(gp.EV_ABS, gp.ABS_RX, 0)
    (drive,) = pad.tick()
    # Same signs as the keyboard: A = walk_y +1, Q = turn +1.
    assert drive["walk_y"] > 0.9 and drive["turn"] > 0.9


def test_small_drift_is_inside_the_dead_zone():
    pad = gp.PadMapper()
    drift = int(128 + 127 * config.GAMEPAD_DEADZONE * 0.8)
    pad.event(gp.EV_ABS, gp.ABS_X, drift)
    assert pad.tick() == []


def test_l1_turns_right_stick_into_body_pose():
    pad = gp.PadMapper()
    pad.event(gp.EV_KEY, gp.BTN_TL, 1)
    pad.event(gp.EV_ABS, gp.ABS_RY, 0)           # right stick up
    (axis,) = pad.tick()
    assert axis["type"] == "axis" and axis["v"] > 0.9 and axis["h"] == 0.0
    pad.event(gp.EV_KEY, gp.BTN_TL, 0)
    assert {"type": "axis", "h": 0.0, "v": 0.0} in pad.tick()


def test_buttons_send_their_commands_on_press_only():
    pad = gp.PadMapper()
    assert pad.event(gp.EV_KEY, gp.BTN_SOUTH, 1) == [{"type": "stance", "name": "stand"}]
    assert pad.event(gp.EV_KEY, gp.BTN_SOUTH, 0) == []
    assert pad.event(gp.EV_KEY, gp.BTN_MODE, 1) == [{"type": "estop"}]
    assert pad.event(gp.EV_KEY, gp.BTN_TR2, 1) == [{"type": "move", "name": "jump"}]


def test_button_commands_name_real_stances_and_moves():
    import moves
    for message in gp.BUTTONS.values():
        if message["type"] == "stance":
            assert message["name"] in config.STANCE_POSES
        if message["type"] == "move":
            assert moves.get(message["name"]) is not None


def test_dpad_cycles_gait_and_steps_speed():
    pad = gp.PadMapper()
    names = list(config.GAITS)
    st.state.gait_name = names[0]
    assert pad.event(gp.EV_ABS, gp.ABS_HAT0X, 1) == [{"type": "gait", "name": names[1]}]
    assert pad.event(gp.EV_ABS, gp.ABS_HAT0X, -1) == [{"type": "gait", "name": names[-1]}]
    assert pad.event(gp.EV_ABS, gp.ABS_HAT0X, 0) == []
    st.state.speed = 0.5
    assert pad.event(gp.EV_ABS, gp.ABS_HAT0Y, -1) == [{"type": "drive", "speed": 0.6}]
    st.state.speed = 1.0
    assert pad.event(gp.EV_ABS, gp.ABS_HAT0Y, -1) == [{"type": "drive", "speed": 1.0}]


def test_r1_cycles_axis_mode():
    pad = gp.PadMapper()
    names = list(config.AXIS_MODES)
    st.state.axis_mode = names[-1]
    (message,) = pad.event(gp.EV_KEY, gp.BTN_TR, 1)
    assert message["mode"] == names[0]


def test_disconnect_stops_a_walking_robot():
    pad = gp.PadMapper()
    pad.event(gp.EV_ABS, gp.ABS_Y, 0)
    pad.tick()
    assert pad.release() == [{"type": "drive", "walk_x": 0.0, "walk_y": 0.0, "turn": 0.0}]


def test_pad_messages_drive_the_server_state():
    import main
    pad = gp.PadMapper()
    pad.event(gp.EV_ABS, gp.ABS_Y, 0)
    for message in pad.tick() + pad.event(gp.EV_KEY, gp.BTN_EAST, 1):
        main._handle_pad_message(message)
    assert st.state.walk_x > 0.9
    assert st.state.stance_name == "sit"
