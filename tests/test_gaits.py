"""
Gait engine tests.

These check the properties that matter on a real machine: the robot
never falls over because too few feet are down, foot paths never jump
(which would be a servo slamming), and turning actually rotates rather
than translating.
"""

import math

import pytest

import config
import gaits
import kinematics as kin

ALL_GAITS = list(config.GAITS)


@pytest.mark.parametrize("name", ALL_GAITS)
def test_every_gait_is_statically_stable(name):
    """At least three feet on the ground at all times, always."""
    assert gaits.min_support_count(name, samples=720) >= 3


@pytest.mark.parametrize("name", ALL_GAITS)
def test_foot_paths_are_continuous(name):
    """
    No discontinuity anywhere in the cycle, including the wrap from
    phase 1.0 back to 0.0 - a jump there would be a servo step command.
    """
    samples = 1440
    previous = None
    worst = 0.0
    for i in range(samples + 1):
        steps = gaits.all_steps((i % samples) / samples, name, 1.0, 0.4, 0.3, 40)
        if previous is not None:
            for leg in config.LEG_IDS:
                a, b = steps[leg], previous[leg]
                worst = max(worst, math.dist((a.dx, a.dy, a.dz), (b.dx, b.dy, b.dz)))
        previous = steps
    assert worst < 1.5, f"{name}: foot jumped {worst:.2f} mm in one sample"


@pytest.mark.parametrize("name", ALL_GAITS)
def test_feet_are_on_the_ground_during_stance(name):
    for i in range(360):
        for step in gaits.all_steps(i / 360, name, 1.0, 0.0, 0.0, 40).values():
            if step.grounded:
                assert step.dz == 0.0
            else:
                assert step.dz >= 0.0


@pytest.mark.parametrize("name", ALL_GAITS)
def test_swing_lifts_the_foot(name):
    peak = max(gaits.all_steps(i / 360, name, 1.0, 0.0, 0.0, 40)["L1"].dz for i in range(360))
    expected = 40 * config.GAITS[name].get("lift", 1.0)
    assert peak == pytest.approx(expected, rel=0.02)


@pytest.mark.parametrize("name", ALL_GAITS)
def test_no_command_means_no_movement(name):
    for step in gaits.all_steps(0.3, name, 0.0, 0.0, 0.0, 40).values():
        assert step.dx == pytest.approx(0.0)
        assert step.dy == pytest.approx(0.0)


def test_turning_is_tangential():
    """
    A pure turn must sweep each foot along the tangent of a circle
    about the body centre - that is what rotates the robot in place
    instead of dragging it sideways.
    """
    steps = gaits.all_steps(0.0, "tripod", 0.0, 0.0, 1.0, 40)
    for leg_id, step in steps.items():
        mount = kin.leg_mount_point(leg_id)
        radius = math.hypot(mount[0], mount[1])
        tangent = (-mount[1] / radius, mount[0] / radius)
        travel = math.hypot(step.dx, step.dy)
        if travel < 1e-6:
            continue
        alignment = (step.dx * tangent[0] + step.dy * tangent[1]) / travel
        assert abs(alignment) == pytest.approx(1.0, abs=1e-6), f"{leg_id} is not moving tangentially"


def test_crab_walk_travels_sideways():
    forward = gaits.all_steps(0.0, "tripod", 1.0, 0.0, 0.0, 40)["L1"]
    crab = gaits.all_steps(0.0, "crab", 1.0, 0.0, 0.0, 40)["L1"]
    assert abs(forward.dx) > 1.0 and abs(forward.dy) < 1e-9
    assert abs(crab.dy) > 1.0 and abs(crab.dx) < 1e-9


def test_diagonal_command_is_not_faster_than_straight():
    """A joystick pushed into a corner must not out-run the edge."""
    straight = gaits.all_steps(0.0, "tripod", 1.0, 0.0, 0.0, 40)["L1"]
    diagonal = gaits.all_steps(0.0, "tripod", 1.0, 1.0, 0.0, 40)["L1"]
    assert math.hypot(diagonal.dx, diagonal.dy) <= math.hypot(straight.dx, straight.dy) + 1e-9


@pytest.mark.parametrize("name", ALL_GAITS)
def test_tripod_is_the_fastest_gait(name):
    assert gaits.cycle_seconds(name, 1.0) >= gaits.cycle_seconds("tripod", 1.0) - 1e-9


def test_speed_scales_the_cycle_time():
    assert gaits.cycle_seconds("tripod", 0.5) == pytest.approx(2 * gaits.cycle_seconds("tripod", 1.0))


def test_phase_wraps():
    phase = gaits.advance_phase(0.9, 10.0, "tripod", 1.0)
    assert 0.0 <= phase < 1.0


@pytest.mark.parametrize("name", ALL_GAITS)
def test_catalog_describes_every_gait(name):
    described = gaits.describe(name)
    assert described["label"] and described["description"]
    assert set(described["offsets"]) == set(config.LEG_IDS)
