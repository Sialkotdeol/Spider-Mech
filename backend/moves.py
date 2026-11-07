"""
Scripted moves: JUMP, BOUNCE, FLEX, DIVE.

A move is a time-parameterised overlay on top of whatever the robot is
already doing. It contributes a body-pose delta and optional per-foot
deltas; the controller adds those to the current stance and gait
output, so a move composes with everything else rather than fighting
it.

Moves are deliberately bounded trajectories. Even JUMP is expressed as
a body-height profile with a hard travel limit, so the worst case for
an untuned robot is an abrupt squat, never a command that drives a
joint past its stop.

A word on JUMP: whether the robot actually leaves the ground depends
entirely on your servos' torque and the machine's mass - many hobby
hexapods will not. The trajectory is correct either way (crouch, fast
extend, tuck, absorb on landing), and it is safe to run on a robot
that never lifts off; it simply looks like a sharp hop.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import config

# Hard cap on how far a move may drive the body vertically, on top of
# the operator's own body-height command.
MOVE_BODY_Z_LIMIT_MM = 85.0
MOVE_BODY_ANGLE_LIMIT_DEG = 32.0


@dataclass
class MoveFrame:
    body_x: float = 0.0
    body_y: float = 0.0
    body_z: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    feet: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)

    def clamped(self) -> "MoveFrame":
        z = max(-MOVE_BODY_Z_LIMIT_MM, min(MOVE_BODY_Z_LIMIT_MM, self.body_z))
        limit = MOVE_BODY_ANGLE_LIMIT_DEG
        return MoveFrame(
            body_x=self.body_x, body_y=self.body_y, body_z=z,
            roll_deg=max(-limit, min(limit, self.roll_deg)),
            pitch_deg=max(-limit, min(limit, self.pitch_deg)),
            yaw_deg=max(-limit, min(limit, self.yaw_deg)),
            feet=self.feet,
        )


@dataclass
class Move:
    name: str
    label: str
    description: str
    duration: float                     # seconds for one pass
    sample: Callable[[float], MoveFrame]  # normalised time 0..1 -> frame
    loop: bool = False
    requires_ground: bool = True        # stop walking before running it

    def frame_at(self, elapsed: float) -> MoveFrame:
        if self.duration <= 0:
            return MoveFrame()
        t = elapsed / self.duration
        t = (t % 1.0) if self.loop else min(t, 1.0)
        return self.sample(t).clamped()

    def finished(self, elapsed: float) -> bool:
        return (not self.loop) and elapsed >= self.duration


def _ease_out(t: float) -> float:
    return 1.0 - (1.0 - t) ** 3


def _ease_in_out(t: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, t)))


def _segment(t: float, start: float, end: float) -> float:
    """Normalise t into 0..1 across the [start, end] slice of a move."""
    if t <= start:
        return 0.0
    if t >= end:
        return 1.0
    return (t - start) / (end - start)


# ------------------------------------------------------------------ JUMP

def _jump(t: float) -> MoveFrame:
    """Crouch, extend hard, tuck the feet, then absorb the landing."""
    crouch_depth = -55.0
    launch_height = 80.0

    if t < 0.28:                                   # load up
        z = crouch_depth * _ease_in_out(_segment(t, 0.0, 0.28))
        feet_lift = 0.0
    elif t < 0.42:                                 # explosive extension
        s = _segment(t, 0.28, 0.42)
        z = crouch_depth + (launch_height - crouch_depth) * _ease_out(s)
        feet_lift = 0.0
    elif t < 0.62:                                 # airborne: tuck the legs up
        s = _segment(t, 0.42, 0.62)
        z = launch_height * (1.0 - s * s)
        feet_lift = 45.0 * math.sin(math.pi * s)
    elif t < 0.82:                                 # touchdown, absorb
        s = _segment(t, 0.62, 0.82)
        z = -40.0 * math.sin(math.pi * s)
        feet_lift = 0.0
    else:                                          # settle back to stance
        s = _segment(t, 0.82, 1.0)
        z = -40.0 * math.sin(math.pi * 1.0) * (1 - s)
        feet_lift = 0.0

    feet = {leg: (0.0, 0.0, -feet_lift) for leg in config.LEG_IDS} if feet_lift else {}
    return MoveFrame(body_z=z, feet=feet)


# ---------------------------------------------------------------- BOUNCE

BOUNCE_CYCLES = 3


def _bounce(t: float) -> MoveFrame:
    """A springy vertical oscillation that decays to nothing."""
    envelope = math.sin(math.pi * t) ** 0.7          # fade in and out
    return MoveFrame(body_z=34.0 * envelope * math.sin(2 * math.pi * BOUNCE_CYCLES * t))


# ------------------------------------------------------------------ FLEX

def _flex(t: float) -> MoveFrame:
    """
    Rolls, pitches and yaws the body through a slow figure while the
    feet stay planted - the clearest demonstration that body IK works,
    and a useful warm-up to check every joint moves freely.
    """
    a = 2 * math.pi * t
    return MoveFrame(
        roll_deg=14.0 * math.sin(a),
        pitch_deg=11.0 * math.sin(2 * a),
        yaw_deg=9.0 * math.sin(a + math.pi / 3),
        body_z=16.0 * math.sin(2 * a + math.pi / 2),
    )


# ------------------------------------------------------------------ DIVE

def _dive(t: float) -> MoveFrame:
    """Pitch nose-down and lunge the body forward, hold, then recover."""
    if t < 0.35:
        s = _ease_in_out(_segment(t, 0.0, 0.35))
    elif t < 0.65:
        s = 1.0                                     # hold the dive
    else:
        s = 1.0 - _ease_in_out(_segment(t, 0.65, 1.0))
    return MoveFrame(pitch_deg=-24.0 * s, body_x=32.0 * s, body_z=-26.0 * s)


MOVES: Dict[str, Move] = {
    "jump": Move("jump", "Jump",
                 "Crouch, extend hard, tuck, absorb the landing.",
                 duration=1.30, sample=_jump),
    "bounce": Move("bounce", "Bounce",
                   "Springy vertical oscillation on planted feet.",
                   duration=1.80, sample=_bounce),
    "flex": Move("flex", "Flex",
                 "Roll, pitch and yaw the body over planted feet. Loops until stopped.",
                 duration=4.20, sample=_flex, loop=True),
    "dive": Move("dive", "Dive",
                 "Pitch nose-down and lunge forward, hold, then recover.",
                 duration=2.40, sample=_dive),
}


def get(name: str) -> Optional[Move]:
    return MOVES.get(name)


def catalog() -> List[dict]:
    return [{"name": m.name, "label": m.label, "description": m.description,
             "duration": m.duration, "loop": m.loop}
            for m in MOVES.values()]
