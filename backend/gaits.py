"""
Gait engine for Spider Mech.

One generic cycle generator drives every gait. A gait is nothing but
the data in config.GAITS: a phase offset per leg, a duty factor (how
much of the cycle a foot spends on the ground), and scale factors.
Tripod, triple, ripple, wave and crab all fall out of that - adding a
new gait means adding a dict entry, not writing code.

Per leg, per tick:
  * STANCE - the foot is planted. Relative to the body it slides
    backwards along the travel direction, which is what actually
    pushes the robot forwards.
  * SWING - the foot is in the air, returning to the front along a
    smooth arc, lifted by a sine profile so it touches down softly.

Turning is handled properly rather than with a left/right sign hack:
each foot's travel direction gets a tangential component around the
body centre, so the whole robot rotates about its own axis and
translation and rotation can be commanded at the same time.

All offsets returned are in the BODY frame, in millimetres, relative
to that leg's neutral stance position.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import config
import kinematics as kin

# How far a foot swings tangentially at full turn command.
TURN_REACH_MM = 55.0


@dataclass
class LegStep:
    leg_id: str
    dx: float           # body-frame offset from the neutral stance position
    dy: float
    dz: float
    grounded: bool      # True during stance
    phase: float        # this leg's own 0..1 position in the cycle


def get_gait(name: str) -> dict:
    return config.GAITS.get(name, config.GAITS[config.DEFAULT_GAIT])


def gait_names() -> List[str]:
    return list(config.GAITS.keys())


def cycle_seconds(name: str, speed: float) -> float:
    """How long one full cycle takes at the given 0..1 speed."""
    gait = get_gait(name)
    speed = max(0.05, min(1.0, speed))
    return config.GAIT_CYCLE_SECONDS * gait.get("period", 1.0) / speed


def advance_phase(phase: float, dt: float, name: str, speed: float) -> float:
    return (phase + dt / cycle_seconds(name, speed)) % 1.0


def _travel_vector(leg_id: str, gait: dict, direction_x: float,
                   direction_y: float, turn: float, stance_reach: float) -> Tuple[float, float]:
    """
    The body-frame vector this foot sweeps through during one stance,
    combining translation and rotation.
    """
    # Crab gaits rotate the commanded heading away from "forwards".
    crab_deg = gait.get("crab", 0.0)
    if crab_deg:
        a = math.radians(crab_deg)
        cos_a, sin_a = math.cos(a), math.sin(a)
        direction_x, direction_y = (direction_x * cos_a - direction_y * sin_a,
                                    direction_x * sin_a + direction_y * cos_a)

    magnitude = math.hypot(direction_x, direction_y)
    if magnitude > 1.0:                    # a joystick corner must not out-reach a joystick edge
        direction_x, direction_y = direction_x / magnitude, direction_y / magnitude
        magnitude = 1.0

    reach = stance_reach * gait.get("stride", 1.0)
    vx = direction_x * reach
    vy = direction_y * reach

    if turn:
        # Tangent to the circle around the body centre at this leg's mount.
        mount = kin.leg_mount_point(leg_id)
        radius = math.hypot(mount[0], mount[1]) or 1.0
        tangent_x, tangent_y = -mount[1] / radius, mount[0] / radius
        turn_reach = TURN_REACH_MM * gait.get("stride", 1.0) * turn
        vx += tangent_x * turn_reach
        vy += tangent_y * turn_reach

    # Never ask a leg for more travel than its stride allows.
    total = math.hypot(vx, vy)
    limit = reach + TURN_REACH_MM * gait.get("stride", 1.0)
    if total > limit and total > 0:
        vx, vy = vx / total * limit, vy / total * limit

    return vx, vy


def leg_step(leg_id: str, phase: float, gait: dict,
             direction_x: float, direction_y: float, turn: float,
             step_height: float, stance_reach: float) -> LegStep:
    """Where one foot should be, this instant, relative to its neutral stance."""
    offset = gait["offsets"].get(leg_id, 0.0)
    duty = min(max(gait.get("duty", 0.5), 0.05), 0.95)
    local_phase = (phase + offset) % 1.0

    vx, vy = _travel_vector(leg_id, gait, direction_x, direction_y, turn, stance_reach)
    lift = step_height * gait.get("lift", 1.0)

    if local_phase < duty:
        # Stance: slide from +half-stride to -half-stride, i.e. backwards.
        t = local_phase / duty
        scale = 0.5 - t
        return LegStep(leg_id, vx * scale, vy * scale, 0.0, True, local_phase)

    # Swing: return to the front, smoothed at both ends so the foot
    # does not snap into motion the instant it leaves the ground.
    t = (local_phase - duty) / (1.0 - duty)
    smooth = 0.5 - 0.5 * math.cos(math.pi * t)
    scale = -0.5 + smooth
    return LegStep(leg_id, vx * scale, vy * scale, lift * math.sin(math.pi * t), False, local_phase)


def all_steps(phase: float, gait_name: str, direction_x: float, direction_y: float,
              turn: float, step_height: float,
              stance_reach: float = None) -> Dict[str, LegStep]:
    gait = get_gait(gait_name)
    stance_reach = config.GAIT_STANCE_REACH_MM if stance_reach is None else stance_reach
    return {
        leg_id: leg_step(leg_id, phase, gait, direction_x, direction_y,
                         turn, step_height, stance_reach)
        for leg_id in config.LEG_IDS
    }


def grounded_legs(phase: float, gait_name: str) -> List[str]:
    """Which feet are on the ground at this phase - the support set."""
    gait = get_gait(gait_name)
    duty = gait.get("duty", 0.5)
    return [leg for leg in config.LEG_IDS
            if ((phase + gait["offsets"].get(leg, 0.0)) % 1.0) < duty]


def min_support_count(gait_name: str, samples: int = 200) -> int:
    """
    Fewest feet ever on the ground during a cycle. Anything below 3 is
    a gait that cannot be statically stable - worth knowing before you
    run it on a robot that weighs something.
    """
    return min(len(grounded_legs(i / samples, gait_name)) for i in range(samples))


def describe(name: str) -> dict:
    gait = get_gait(name)
    return {
        "name": name,
        "label": gait.get("label", name),
        "description": gait.get("description", ""),
        "duty": round(gait.get("duty", 0.5), 4),
        "stride": gait.get("stride", 1.0),
        "lift": gait.get("lift", 1.0),
        "period": gait.get("period", 1.0),
        "crab": gait.get("crab", 0.0),
        "offsets": {leg: round(gait["offsets"].get(leg, 0.0), 4) for leg in config.LEG_IDS},
        "min_support": min_support_count(name),
    }


def catalog() -> List[dict]:
    return [describe(name) for name in gait_names()]
