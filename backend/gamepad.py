"""
PS5 (DualSense) controller support, read on the Pi over Bluetooth.

Runs inside the server as a background task and feeds the same command
messages the web UI sends over the WebSocket, so the controller and the
laptop can be used together - whichever moved last wins.

Pair the controller once with bluetoothctl (see GAMEPAD.md). Any pad
the Linux kernel exposes as a gamepad works (PS4, Xbox, 8BitDo), not
only the DualSense. Needs python-evdev; without it, or off Linux, this
module just logs that the controller is off and returns.

    Left stick        walk (forward/back, strafe)
    Right stick X     turn
    Hold L1           right stick moves the body (current axis mode)
    R1                next axis mode (X-Y, Z-W, U-V, XY-UV)
    D-pad left/right  previous / next gait
    D-pad up/down     speed +/- 10 %
    Cross / Circle    stand / sit
    Triangle / Square tall / crouch
    R2 / L2           jump / bounce
    L3 / R3           flex / dive
    PS button         E-STOP
    Options           clear E-stop
    Create            beep
"""

import asyncio
import logging
from typing import Callable, Dict, List, Optional, Tuple

import config
import state as st

logger = logging.getLogger("gamepad")

# Linux input event codes (include/uapi/linux/input-event-codes.h). They
# are kernel ABI and never change, so the mapping below can be tested
# on a machine without evdev.
EV_KEY, EV_ABS = 1, 3
ABS_X, ABS_Y, ABS_RX, ABS_RY = 0, 1, 3, 4
ABS_HAT0X, ABS_HAT0Y = 16, 17
BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST = 304, 305, 307, 308
BTN_TL, BTN_TR, BTN_TL2, BTN_TR2 = 310, 311, 312, 313
BTN_SELECT, BTN_START, BTN_MODE = 314, 315, 316
BTN_THUMBL, BTN_THUMBR = 317, 318

STICKS = (ABS_X, ABS_Y, ABS_RX, ABS_RY)

# Button -> the WebSocket message it sends on press.
BUTTONS: Dict[int, dict] = {
    BTN_SOUTH:  {"type": "stance", "name": "stand"},    # Cross
    BTN_EAST:   {"type": "stance", "name": "sit"},      # Circle
    BTN_NORTH:  {"type": "stance", "name": "tall"},     # Triangle
    BTN_WEST:   {"type": "stance", "name": "crouch"},   # Square
    BTN_TR2:    {"type": "move", "name": "jump"},       # R2
    BTN_TL2:    {"type": "move", "name": "bounce"},     # L2
    BTN_THUMBL: {"type": "move", "name": "flex"},       # L3
    BTN_THUMBR: {"type": "move", "name": "dive"},       # R3
    BTN_MODE:   {"type": "estop"},                      # PS button
    BTN_START:  {"type": "estop_clear"},                # Options
    BTN_SELECT: {"type": "beep"},                       # Create
}

Message = dict


class PadMapper:
    """Turns raw pad events into command messages. Holds no I/O, so the
    tests drive it directly."""

    def __init__(self, ranges: Optional[Dict[int, Tuple[int, int]]] = None):
        self.ranges = {code: (0, 255) for code in STICKS}
        self.ranges.update(ranges or {})
        self.sticks = {code: 0.0 for code in STICKS}
        self.body_held = False          # L1
        self._drive_active = False
        self._axis_active = False

    # ---------------------------------------------------------- events

    def event(self, kind: int, code: int, value: int) -> List[Message]:
        """Handle one event; return the one-shot messages it triggers.
        Stick movement is sent by tick(), not here."""
        if kind == EV_ABS and code in STICKS:
            self.sticks[code] = self._normalise(code, value)
        elif kind == EV_ABS and code in (ABS_HAT0X, ABS_HAT0Y) and value:
            return [self._dpad(code, value)]
        elif kind == EV_KEY and code == BTN_TL:
            self.body_held = bool(value)
        elif kind == EV_KEY and value == 1:
            if code == BTN_TR:
                return [self._next_axis_mode()]
            if code in BUTTONS:
                return [dict(BUTTONS[code])]
        return []

    def tick(self) -> List[Message]:
        """Called at GAMEPAD_RATE_HZ. Re-sends stick commands while a stick
        is held (so the control timeout never drops a held stick), and
        one zero when it is released. A centred pad sends nothing, which
        leaves the laptop free to drive."""
        out: List[Message] = []
        walk_x = -self.sticks[ABS_Y]
        walk_y = -self.sticks[ABS_X]
        turn = 0.0 if self.body_held else -self.sticks[ABS_RX]
        if walk_x or walk_y or turn:
            out.append({"type": "drive", "walk_x": walk_x, "walk_y": walk_y, "turn": turn})
            self._drive_active = True
        elif self._drive_active:
            out.append({"type": "drive", "walk_x": 0.0, "walk_y": 0.0, "turn": 0.0})
            self._drive_active = False

        h = self.sticks[ABS_RX] if self.body_held else 0.0
        v = -self.sticks[ABS_RY] if self.body_held else 0.0
        if h or v:
            out.append({"type": "axis", "h": h, "v": v})
            self._axis_active = True
        elif self._axis_active:
            out.append({"type": "axis", "h": 0.0, "v": 0.0})
            self._axis_active = False
        return out

    def release(self) -> List[Message]:
        """The pad went away: centre everything and stop walking."""
        self.sticks = {code: 0.0 for code in STICKS}
        self.body_held = False
        return self.tick()

    # --------------------------------------------------------- helpers

    def _normalise(self, code: int, value: int) -> float:
        lo, hi = self.ranges[code]
        centre, half = (lo + hi) / 2.0, (hi - lo) / 2.0
        x = max(-1.0, min(1.0, (value - centre) / half)) if half else 0.0
        dead = config.GAMEPAD_DEADZONE
        if abs(x) <= dead:
            return 0.0
        # Rescale so the output starts at 0 just outside the dead zone.
        return round(((abs(x) - dead) / (1.0 - dead)) * (1 if x > 0 else -1), 3)

    def _dpad(self, code: int, value: int) -> Message:
        with st.lock():
            gait, speed = st.state.gait_name, st.state.speed
        if code == ABS_HAT0X:
            names = list(config.GAITS)
            index = names.index(gait) if gait in names else 0
            return {"type": "gait", "name": names[(index + value) % len(names)]}
        # Hat Y is -1 for up.
        return {"type": "drive", "speed": round(max(0.1, min(1.0, speed - 0.1 * value)), 2)}

    def _next_axis_mode(self) -> Message:
        with st.lock():
            mode = st.state.axis_mode
        names = list(config.AXIS_MODES)
        index = names.index(mode) if mode in names else -1
        return {"type": "axis", "mode": names[(index + 1) % len(names)], "h": 0.0, "v": 0.0}


# ================================================================ device I/O

def _find_pad(evdev):
    """The first input device with a left stick and face buttons. The
    DualSense also creates 'Motion Sensors' and 'Touchpad' devices; they
    have no BTN_SOUTH, so they are skipped."""
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
        except OSError:
            continue
        caps = device.capabilities()
        abs_codes = [c[0] if isinstance(c, tuple) else c for c in caps.get(EV_ABS, [])]
        if ABS_X in abs_codes and BTN_SOUTH in caps.get(EV_KEY, []):
            return device
        device.close()
    return None


async def _pump(mapper: PadMapper, handle: Callable[[Message], None]) -> None:
    period = 1.0 / config.GAMEPAD_RATE_HZ
    while True:
        await asyncio.sleep(period)
        for message in mapper.tick():
            handle(message)


async def run(handle: Callable[[Message], None]) -> None:
    """Find a pad, feed its events to `handle`, and keep looking for it
    again whenever it disconnects."""
    try:
        import evdev
    except ImportError:
        logger.info("PS5 controller off: python-evdev not installed (pip install evdev)")
        return

    announced = False
    while True:
        device = _find_pad(evdev)
        if device is None:
            if not announced:
                logger.info("no controller yet - press the PS button to connect it")
                announced = True
            await asyncio.sleep(2.0)
            continue
        announced = False
        logger.info("controller connected: %s (%s)", device.name, device.path)
        ranges = {}
        for code in STICKS:
            try:
                info = device.absinfo(code)
                ranges[code] = (info.min, info.max)
            except (OSError, KeyError):
                pass
        mapper = PadMapper(ranges)
        pump = asyncio.create_task(_pump(mapper, handle))
        try:
            async for event in device.async_read_loop():
                for message in mapper.event(event.type, event.code, event.value):
                    handle(message)
        except OSError:
            logger.warning("controller disconnected - robot stopped")
        finally:
            pump.cancel()
            for message in mapper.release():
                handle(message)
            device.close()
