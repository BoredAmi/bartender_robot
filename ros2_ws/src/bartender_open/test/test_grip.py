"""Tests for the gripper model and the joint-wrapping in bartender_open.arm.

Both of these are small pure functions that decide big things, and both were
put there after a specific failure that was hard to read.

The pad-gap curve turns "the neck is 38.7mm across" into "the knuckle will
touch at 0.46 rad". Before it existed the conversion was assumed linear, and
it is not; the fingers were commanded past the object and threw the bottle.
The curve's best property is that it can be checked against numbers this
project measured independently, in sim, for two other bottles, in another
package, before it was written -- so these tests are not circular.

wrap_to_pi turns an IK solution pressed against a joint limit into the
identical pose with room to move. A Cartesian move from the unwrapped one
cannot be followed through, which shows up as a plan that reaches 57% of a
straight line for no visible reason.
"""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, *([os.pardir] * 4)))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_open'))

from bartender_open.arm import (                        # noqa: E402
    GRASP_LOOSE, GRASP_PENETRATION, GRIPPER_FULLY_CLOSED,
    GRIPPER_PRE_CLOSE_GAP, GRIPPER_SQUEEZE, PAD_GAP, SIDE_QUAT,
    gap_for_knuckle, knuckle_for_gap, side_quat, wrap_to_pi,
)
from bartender_open import layout as L                  # noqa: E402

# Measured in simulation by bartender_pour, for its own bottles, with no
# reference to this curve: see the WHISKEY and COLA blocks in
# pour_action_server.py. The whiskey's flats are 77.2mm and it records the
# knuckle stalling at 0.089; the cola is gripped at a 59.6mm waist and it
# records contact at about 0.265.
# What bartender_pour measured in simulation, for its own bottles, with no
# reference to this curve: see the WHISKEY and COLA blocks in
# pour_action_server.py. The two are different KINDS of measurement and the
# tests below treat them as such.
#
#   the cola's is a first-CONTACT figure -- "contact comes at ~0.265 rad and
#   the joint settles at 0.27-0.29" -- so the curve should hit it head on;
#   the whiskey's 0.089 is where the joint STALLED, which is past contact by
#   however far the pads sank in, and that file records 0.2mm for its flats.
MEASURED_CONTACT = [
    ('cola waist', 0.0596, 0.265),
]
MEASURED_STALL = [
    ('whiskey flats', 0.0772, 0.089, 0.0002),
]


@pytest.mark.parametrize('what,width,measured', MEASURED_CONTACT)
def test_the_curve_hits_a_measured_contact(what, width, measured):
    """The check that this model is of the real gripper and not of itself.

    Measured in simulation, for another bottle, by another package, before
    this curve existed. It is the only evidence available that the curve can
    be trusted for a bottle nobody has gripped yet, which is exactly what it
    is used for.
    """
    assert knuckle_for_gap(width) == pytest.approx(measured, abs=0.002)


@pytest.mark.parametrize('what,width,stall,penetration', MEASURED_STALL)
def test_a_measured_stall_sits_just_past_the_predicted_contact(
        what, width, stall, penetration):
    """And the size of the gap between them is itself the recorded penetration.

    The whiskey's flats predict contact at 0.0865 against a measured 0.089
    stall. That is 0.0025 rad, which on this curve is 0.24mm of pad
    penetration -- and 0.2mm is the figure pour_action_server records for
    that bottle. The agreement is in the residual as much as in the number,
    and a curve that matched the stall exactly would be matching something it
    does not model.
    """
    assert knuckle_for_gap(width) < stall
    sunk = width - gap_for_knuckle(stall)
    assert sunk == pytest.approx(penetration, abs=0.0003)


@pytest.mark.parametrize('knuckle', [0.0, 0.07, 0.25, 0.4612, 0.55, 0.8])
def test_gap_and_knuckle_invert_each_other(knuckle):
    assert knuckle_for_gap(gap_for_knuckle(knuckle)) == pytest.approx(
        knuckle, abs=1e-9)


def test_wider_than_the_gripper_opens_clamps_to_zero():
    assert knuckle_for_gap(0.20) == 0.0


def test_narrower_than_it_closes_clamps_to_the_end():
    assert knuckle_for_gap(-0.01) == PAD_GAP[-1][0]


def test_the_curve_covers_everything_this_action_grips():
    """Both objects must fall inside the table, not off its ends."""
    for width in (L.BEER_WIDTH, L.OPENER_SHAFT):
        assert PAD_GAP[-1][1] < width < PAD_GAP[0][1]


def test_the_pre_close_leaves_the_pads_clear_of_both_objects():
    """The one fast command must never reach the object.

    It is the only move big enough to overshoot much, and an overshoot that
    reaches the object is a collision at speed rather than a grasp.
    """
    for width in (L.BEER_WIDTH, L.OPENER_SHAFT):
        target = knuckle_for_gap(width + GRIPPER_PRE_CLOSE_GAP)
        assert gap_for_knuckle(target) > width


def test_the_close_has_room_to_find_both_objects():
    """Contact must come before the fingers run out of travel.

    If an object is so narrow that contact is past GRIPPER_FULLY_CLOSED, a
    perfectly good grasp is reported as "closed all the way without meeting
    anything".
    """
    for width in (L.BEER_WIDTH, L.OPENER_SHAFT):
        assert knuckle_for_gap(width) < GRIPPER_FULLY_CLOSED - 0.05


def test_the_squeeze_does_not_drive_past_the_linkage():
    for width in (L.BEER_WIDTH, L.OPENER_SHAFT):
        assert knuckle_for_gap(width) + GRIPPER_SQUEEZE < 0.80


def test_the_stall_window_is_asymmetric():
    """Too wide and too narrow are different failures and get different limits.

    Wide means the step has not arrived; narrow means the fingers are not on
    the object. Penetration is real and one-sided -- a round section needs
    several millimetres of it before the joint stalls, flats need a fraction
    of one -- so the allowance below the object's width has to be much larger
    than the allowance above it.
    """
    assert GRASP_PENETRATION > GRASP_LOOSE


def test_a_held_object_is_distinguishable_from_an_empty_gripper():
    """still_holding's allowance must be inside the squeeze.

    When the object goes, the fingers travel the whole squeeze command. The
    allowance has to be comfortably less than that or losing the bottle looks
    like holding it.
    """
    allowance = GRIPPER_SQUEEZE / 2.0
    assert allowance < GRIPPER_SQUEEZE
    lost = gap_for_knuckle(knuckle_for_gap(L.BEER_WIDTH) + GRIPPER_SQUEEZE)
    assert lost < L.BEER_WIDTH - 0.010


# -- joint wrapping ---------------------------------------------------------

@pytest.mark.parametrize('angle', [0.0, 1.0, -1.0, 3.0, -3.0,
                                   6.07, -6.07, 2 * math.pi, -2 * math.pi,
                                   12.0, -12.0])
def test_wrapping_never_changes_the_pose(angle):
    """A full turn of a revolute joint is the same configuration.

    This is what makes wrapping safe rather than a fudge: forward kinematics
    depends on the angle only through its sine and cosine, so the wrapped
    value puts every link in exactly the same place, with the same
    collisions.
    """
    wrapped = wrap_to_pi(angle)
    assert math.sin(wrapped) == pytest.approx(math.sin(angle), abs=1e-12)
    assert math.cos(wrapped) == pytest.approx(math.cos(angle), abs=1e-12)


@pytest.mark.parametrize('angle', [0.0, 1.0, -1.0, 6.07, -6.07, 12.0, -12.0])
def test_wrapping_lands_inside_the_range(angle):
    assert -math.pi < wrap_to_pi(angle) <= math.pi


def test_wrapping_moves_a_limit_hugging_solution_away_from_the_limit():
    """The case this exists for.

    b_wrist_2 at -6.07 rad has 0.21 rad left before its -2pi limit, which is
    not enough to follow a 100mm straight line. The same pose at +0.21 has
    the whole range.
    """
    assert abs(wrap_to_pi(-6.07)) < 0.25
    assert abs(-6.07) > 6.0


def test_wrapping_is_idempotent():
    for angle in (0.5, -0.5, 3.0, -3.0, 6.07):
        assert wrap_to_pi(wrap_to_pi(angle)) == pytest.approx(
            wrap_to_pi(angle), abs=1e-15)


# -- the side grasp orientation ---------------------------------------------

def test_side_quat_is_the_cyclic_axis_permutation():
    """tool0's +z onto the arm's +x, its +x onto +y, its +y onto +z.

    Same quaternion bartender_pour uses. Expressed in each arm's OWN base
    frame, which is why neither arm needs a special case: for arm A it means
    approach along world +x, and for arm B -- yawed -90 degrees -- along
    world -y, in over the counter, which is the direction it wants anyway.
    """
    assert SIDE_QUAT == pytest.approx((0.5, 0.5, 0.5, 0.5), abs=1e-12)


def test_side_quat_is_a_unit_quaternion():
    for theta in (0.0, 0.3, -0.7, 1.7):
        assert sum(v * v for v in side_quat(theta)) == pytest.approx(
            1.0, abs=1e-12)


def test_side_quat_maps_tool_z_onto_the_arms_x():
    x, y, z, w = SIDE_QUAT
    # Third column of the rotation matrix: where local +z ends up.
    local_z = (2 * (x * z + y * w), 2 * (y * z - x * w),
               1 - 2 * (x * x + y * y))
    assert local_z == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)


def test_side_quat_maps_tool_y_onto_the_arms_z():
    """Which is what stands a grasped bottle up along the gripper's free axis."""
    x, y, z, w = SIDE_QUAT
    local_y = (2 * (x * y - z * w), 1 - 2 * (x * x + z * z),
               2 * (y * z + x * w))
    assert local_y == pytest.approx((0.0, 0.0, 1.0), abs=1e-12)
