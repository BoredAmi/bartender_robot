"""Tests for the two-armed opening layout.

What these protect is agreement between things that have no runtime link.

The description places arm B relative to arm A. The world file places arm B's
pedestal, the beer, the opener and the cap in world coordinates.
bartender_open/layout.py states all of it again in each arm's own frame, and
restates the beer, cap and opener dimensions that make_beer_and_opener.py
decides. Nothing checks any of that when the simulator runs. A pedestal 50mm
from where the arm actually is looks fine; an opener aimed at a cap that is
20mm from where the layout says it is looks like bad luck with the physics.

The second thing is reach. Every pose here is inside a UR5e's 850mm, and the
beer is deliberately about the same distance from both arms because both have
to work on it at once. That is easy to lose by nudging a station 100mm to make
room for something.
"""
import importlib.util
import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, *([os.pardir] * 4)))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_open'))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo',
                                'scripts'))

from bartender_open import layout as L                 # noqa: E402
import make_beer_and_opener as gen                     # noqa: E402

WORLD = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo', 'worlds',
                     'bar_world.sdf')
XACRO = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_description', 'urdf',
                     'bartender.urdf.xacro')
MODELS = os.path.join(REPO, 'models')

UR5E_REACH = 0.85
# The counter is 1.2 x 0.6, centred on the world origin, top at 0.9.
COUNTER = (1.2, 0.6)
COUNTER_Z = 0.9


def _load_render_script():
    """Import render_bartender_urdf.py by path, or None if it is not here.

    It lives in another package's scripts/ directory, which is not on any
    import path: bartender_description is CMake and installs no module.
    """
    path = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_description',
                        'scripts', 'render_bartender_urdf.py')
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location('_render_for_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def world_poses():
    root = ET.parse(WORLD).getroot()
    return {inc.find('name').text:
            [float(v) for v in inc.find('pose').text.split()]
            for inc in root.iter('include')}


# -- the two frames agree ---------------------------------------------------

def test_arm_b_origin_matches_the_description():
    """Compare layout's world position for arm B with the xacro's.

    These are the same point written two ways: the xacro attaches arm B to
    the same `world` link arm A hangs off, so its xyz is relative to arm A's
    base, while everything in the world file and in layout is absolute. Get
    one of them wrong and the arm is simply somewhere else than the pedestal
    under it, and than every pose computed for it.
    """
    text = open(XACRO).read()
    xyz = [float(v) for v in
           text.split('name="arm_b_xyz" value="')[1].split('"')[0].split()]
    assert xyz == pytest.approx(list(L.ARM_B_IN_A), abs=1e-9)

    yaw = float(text.split('name="arm_b_rpy" value="')[1]
                .split('"')[0].split()[2])
    assert yaw == pytest.approx(L.ARM_B_YAW, abs=1e-9)


def test_arm_b_stands_on_its_pedestal():
    pose = world_poses()['arm_b_pedestal']
    assert pose[:2] == pytest.approx(list(L.ARM_B_ORIGIN[:2]), abs=1e-9)
    # The pedestal is 0.9 tall with its origin on the floor, so its top face
    # is exactly where the arm's base is.
    assert pose[2] + 0.90 == pytest.approx(L.ARM_B_ORIGIN[2], abs=1e-9)


def test_both_arm_bases_sit_at_counter_height():
    """The one coincidence the whole layout leans on.

    Both arms are mounted level with the counter top, so a z in either arm's
    frame means "height above the counter" and the two agree. Every height in
    layout.py is written that way -- grasp heights measured up a bottle from
    its base, a seated bell measured down from a cap -- and none of them says
    which arm it is for, because it does not have to.
    """
    assert L.ARM_A_ORIGIN[2] == COUNTER_Z
    assert L.ARM_B_ORIGIN[2] == COUNTER_Z


@pytest.mark.parametrize('station,model', [('beer', 'beer_bottle'),
                                           ('opener', 'bottle_opener')])
def test_station_matches_the_world(station, model):
    pose = world_poses()[model]
    assert pose[:2] == pytest.approx(list(L.STATIONS[station]), abs=1e-9)


def test_the_beer_stand_is_under_the_beer():
    """A stand offset from its bottle is a stand the bottle lands on the rim of."""
    assert (world_poses()['beer_stand'][:2]
            == pytest.approx(world_poses()['beer_bottle'][:2], abs=1e-9))


def test_the_holster_is_under_the_opener():
    assert (world_poses()['opener_holster'][:2]
            == pytest.approx(world_poses()['bottle_opener'][:2], abs=1e-9))


def test_the_cap_starts_on_the_bottles_mouth():
    """Not a millimetre lower.

    The cap and the bottle are separate models held together by a
    DetachableJoint. While they are joined they are one skeleton and overlap
    is survivable; the instant the joint lets go they are two bodies, and two
    bodies interpenetrating is how the cap gets fired across the room. So it
    sits exactly ON the lip.
    """
    beer = world_poses()['beer_bottle']
    cap = world_poses()['beer_cap']
    assert cap[:2] == pytest.approx(beer[:2], abs=1e-9)
    assert cap[2] == pytest.approx(beer[2] + L.BEER_HEIGHT, abs=1e-6)


def test_the_opener_rests_on_its_rim():
    """Its origin is the bell rim, and the rim sits on the counter.

    The post is shorter than the bell is deep, on purpose, so this is the
    only place it can come to rest. Placing the opener at the post's top
    instead leaves it hanging 14mm in the air at t=0 and it drops.
    """
    pose = world_poses()['bottle_opener']
    assert pose[2] == pytest.approx(COUNTER_Z + L.OPENER_REST_RIM_Z, abs=1e-9)
    assert gen.post_top() < gen.bell_height()


# -- the numbers restated from the generator --------------------------------

@pytest.mark.parametrize('name,mine,theirs', [
    ('BEER_HEIGHT', lambda: L.BEER_HEIGHT, lambda: gen.BEER_HEIGHT),
    ('BEER_BODY_RADIUS', lambda: L.BEER_BODY_RADIUS,
     lambda: gen.BEER_BODY_RADIUS),
    ('BEER_GRIP_BAND', lambda: L.BEER_GRIP_BAND, lambda: gen.BEER_GRIP_BAND),
    # Rounded to a tenth of a millimetre where the generator measures it off
    # the mesh to full precision. That is deliberate -- it is the width the
    # gripper is told to close on, and a gripper that cares about the seventh
    # decimal place of a bottle is not a gripper -- so this one is checked to
    # the precision it is written at.
    ('BEER_GRIP_RADIUS', lambda: L.BEER_GRIP_RADIUS,
     lambda: round(gen.beer_grip_radius(), 4)),
    ('CAP_HEIGHT', lambda: L.CAP_HEIGHT, lambda: gen.CAP_HEIGHT),
    ('CAP_RADIUS', lambda: L.CAP_RADIUS, lambda: gen.CAP_RADIUS),
    ('BELL_HEIGHT', lambda: L.BELL_HEIGHT, lambda: gen.bell_height()),
    ('BELL_CLEARANCE', lambda: L.BELL_CLEARANCE, lambda: gen.BELL_CLEARANCE),
    ('BELL_CAPTURE', lambda: L.BELL_CAPTURE, lambda: gen.bell_capture()),
    ('BELL_OUTER_RADIUS', lambda: L.BELL_OUTER_RADIUS,
     lambda: gen.bell_outer_radius()),
    ('OPENER_HEIGHT', lambda: L.OPENER_HEIGHT, lambda: gen.opener_height()),
    ('OPENER_GRIP_Z', lambda: L.OPENER_GRIP_Z, lambda: gen.shaft_grip_z()),
    ('OPENER_SHAFT', lambda: L.OPENER_SHAFT, lambda: gen.SHAFT),
    ('HOLSTER_POST_TOP', lambda: L.HOLSTER_POST_TOP, lambda: gen.post_top()),
])
def test_dimension_matches_the_generator(name, mine, theirs):
    """layout.py restates the generator's numbers; they must still be its numbers.

    Restated rather than imported because the generator is a standalone
    script in another package and is not installed as a module. This is what
    stops the copy drifting, and drift here is silent: the opener would be
    aimed a few millimetres wrong and the bell would land on the cap instead
    of over it.
    """
    assert mine() == pytest.approx(theirs(), abs=1e-9)


def test_the_beer_grip_band_is_the_straight_part_of_the_neck():
    """The band the pads close on must be the widest thing near it.

    If the shoulder below it is not wider, there is no step, and without the
    step this is just another smooth cylinder between two flat pads -- which
    is the grip that threw this bottle 3.8 metres.
    """
    bands = {(z0, z1): r for z0, z1, r in gen.beer_bands()}
    grip = bands[L.BEER_GRIP_BAND]
    below = [r for (z0, z1), r in bands.items() if z1 <= L.BEER_GRIP_BAND[0]]
    above = [r for (z0, z1), r in bands.items() if z0 >= L.BEER_GRIP_BAND[1]]
    assert min(below) > grip + 0.005, 'no step under the grip band'
    assert max(above) < grip, 'the neck must narrow above the grip'


def test_the_pads_clear_the_shoulder_below_the_grip_band():
    """The pad is wider than the band, so it must overhang the free end.

    Above 0.200 the bottle tapers away and an overhanging pad never touches.
    Below 0.170 is the shoulder step, and a pad reaching onto it grips that
    instead: the fingers stall at 41.6mm rather than the neck's 38.7mm -- a
    real grasp, but a different one from run to run, which no grip check can
    accept. The clearance has to beat the ~3mm a Cartesian move lands within.
    """
    pad_low = L.BEER_GRASP_HEIGHT - L.PAD_HALF_WIDTH
    assert pad_low > L.BEER_GRIP_BAND[0] + 0.003
    # The pad really is wider than the band -- if that stops being true, the
    # overhang reasoning above is no longer what is keeping this safe.
    assert 2 * L.PAD_HALF_WIDTH > L.BEER_GRIP_BAND[1] - L.BEER_GRIP_BAND[0]


def test_the_pad_still_contacts_most_of_the_neck_band():
    """Moving the grip up the neck trades contact for shoulder clearance.

    It can only be traded so far: the band is all the contact this bottle
    has, and the beer is the grasp that already fails most.
    """
    contact = min(L.BEER_GRASP_HEIGHT + L.PAD_HALF_WIDTH, L.BEER_GRIP_BAND[1])
    contact -= max(L.BEER_GRASP_HEIGHT - L.PAD_HALF_WIDTH, L.BEER_GRIP_BAND[0])
    assert contact > 0.024


def test_the_pad_width_agrees_with_the_description():
    """PAD_HALF_WIDTH is a copy, and copies drift.

    render_bartender_urdf.py derives the pad width from a table that carries
    its own copy of BEER_GRASP_HEIGHT, so a change to either file that is not
    made in both leaves the arm planning against a gripper it does not have.
    Loaded by path because bartender_description is a CMake package with no
    importable module; skipped rather than failed when it is not beside us,
    so an installed-only test run does not report a phantom failure.
    """
    render = _load_render_script()
    if render is None:
        pytest.skip('bartender_description sources are not in this tree')
    assert render.PAD_HALF_WIDTH == pytest.approx(L.PAD_HALF_WIDTH, abs=1e-9)
    beer = [row for row in render.GRIP_BANDS if row[0] == 'beer'][0]
    assert beer[3] == pytest.approx(L.BEER_GRASP_HEIGHT, abs=1e-9)
    assert (beer[1], beer[2]) == pytest.approx(L.BEER_GRIP_BAND, abs=1e-9)
    # ...and the width really is the limit that table implies, not a number
    # someone picked and left behind when a bottle moved.
    assert render.PAD_HALF_WIDTH == pytest.approx(
        render.pad_half_width_limit(), abs=1e-9)


# -- the geometry of the press ----------------------------------------------

def test_a_seated_bell_is_deeper_than_the_test_for_it():
    assert L.SEAT_DEPTH_MIN < L.BELL_HEIGHT
    # ...and past the chamfer, into the straight bore, which is what
    # distinguishes "over the cap" from "resting on it".
    assert L.SEAT_DEPTH_MIN > gen.BELL_LEAD_H


def test_the_bell_clears_the_cap_and_funnels_it():
    assert L.BELL_CLEARANCE > 0.0
    assert L.BELL_CAPTURE > L.BELL_CLEARANCE
    assert L.SEAT_OFFSET_MAX == pytest.approx(L.BELL_CAPTURE, abs=1e-9)


def test_the_bell_clears_the_pads_holding_the_bottle():
    """Arm B's bell must not reach down to arm A's fingers.

    Both are on the bottle's axis, so there is no lateral clearance at all
    between them -- the bell's 23.6mm outer radius swallows fingers closed to
    a 38.7mm neck. All the clearance there is, is height.
    """
    pad_top = L.BEER_GRIP_BAND[1] + L.HOLD_LIFT
    assert L.seated_rim_z() > pad_top + 0.02


def test_the_lift_clears_the_stand():
    assert L.HOLD_LIFT > L.STAND_HEIGHT


def test_the_transit_clears_a_capped_bottle_in_its_stand():
    assert L.APPROACH_Z > L.cap_top_above_base() + 0.10


def test_the_keepout_is_below_the_transit_and_above_the_bottle():
    """Its height is what makes it usable rather than merely safe.

    The keep-out has to stop a plan dipping towards the bottle while still
    allowing the one pose every approach ends at, which is directly above it.
    That only works if the volume stops short of the transit height.
    """
    assert L.BEER_KEEPOUT_HEIGHT > L.cap_top_above_base()
    assert L.BEER_KEEPOUT_HEIGHT < L.APPROACH_Z - 0.05
    assert L.BEER_KEEPOUT_RADIUS > L.BEER_BODY_RADIUS


def test_the_shift_limit_allows_the_push_and_catches_a_knock():
    """It must be well over the give under a press and well under a knock.

    The give is the arm's own compliance plus a round neck settling in the
    pads; measured between 3.7 and 19.5mm on presses that seated the bell.
    """
    assert L.BOTTLE_SHIFT_MAX > 4 * L.PRESS_TRAVEL
    assert L.BOTTLE_SHIFT_MAX < L.HOLD_LIFT


def test_a_bottle_left_standing_fails_the_held_check():
    """The check a standing bottle cannot fake.

    Push down on a bottle resting in its well and it does not move either --
    the counter takes the load. So the gate cannot be about movement alone;
    it has to ask whether the bottle is off the counter, which is only true
    if the other arm is holding it.
    """
    assert L.MIN_HELD_CLEARANCE > L.STAND_HEIGHT
    assert L.MIN_HELD_CLEARANCE < L.HOLD_LIFT


def test_a_cap_still_on_the_bottle_does_not_count_as_off():
    """CAP_FREE_MIN has to exceed how far a cap can be and still be sitting there.

    It is a disc resting on the lip, so "still on" is not exactly zero: it
    was measured settling 13mm away, leaning against the neck, after a detach
    with no flick. The threshold has to be above that.
    """
    assert L.CAP_FREE_MIN > 0.020


# -- reach and the counter --------------------------------------------------

@pytest.mark.parametrize('station', sorted(L.STATIONS))
def test_station_is_on_the_counter(station):
    x, y = L.STATIONS[station]
    assert abs(x) < COUNTER[0] / 2.0 - L.BEER_STAND_RADIUS
    assert abs(y) < COUNTER[1] / 2.0 - L.BEER_STAND_RADIUS


def test_the_pedestal_is_off_the_counter():
    """Arm B stands beside the worktop, not on it."""
    assert L.ARM_B_ORIGIN[1] - 0.15 > COUNTER[1] / 2.0


@pytest.mark.parametrize('station,arm', [('beer', 'a'), ('beer', 'b'),
                                         ('opener', 'b')])
def test_station_is_within_reach(station, arm):
    here = L.station_in_arm(station, arm)
    assert L.reach(here) < UR5E_REACH * 0.8


def test_the_beer_is_about_equally_far_from_both_arms():
    """Both arms work on this bottle at the same time, so neither may be stretched.

    It is the only station that has to satisfy two arms at once, which is
    what fixed it where it is.
    """
    from_a = L.reach(L.station_in_arm('beer', 'a'))
    from_b = L.reach(L.station_in_arm('beer', 'b'))
    assert abs(from_a - from_b) < 0.05


def test_the_working_poses_are_within_reach():
    """Not just the stations: the flange poses actually commanded."""
    held = L.side_grasp_tool0(L.beer_grip_point(L.HOLD_LIFT))
    assert L.reach(held) < UR5E_REACH * 0.8
    over = L.side_grasp_tool0(L.opener_over_cap_grip(L.seated_rim_z()))
    assert L.reach(over) < UR5E_REACH * 0.8
    holster = L.side_grasp_tool0(L.opener_grip_point(L.OPENER_REST_RIM_Z))
    assert L.reach(holster) < UR5E_REACH * 0.8


def test_the_beer_is_clear_of_the_other_stations():
    """Its stand must not overlap the whiskey's, and the arms need room."""
    whiskey = (0.15, 0.15)
    beer = L.STATIONS['beer']
    apart = math.dist(whiskey, beer)
    assert apart > 0.0740 + L.BEER_STAND_RADIUS + 0.02


# -- frame conversion -------------------------------------------------------

@pytest.mark.parametrize('point', [(0.0, 0.0, 0.9), (0.3, -0.2, 1.1),
                                   (-0.5, 0.4, 0.95)])
@pytest.mark.parametrize('arm', ['a', 'b'])
def test_to_arm_and_to_world_are_inverses(point, arm):
    origin, yaw = ((L.ARM_A_ORIGIN, L.ARM_A_YAW) if arm == 'a'
                   else (L.ARM_B_ORIGIN, L.ARM_B_YAW))
    there = L.to_arm(point, origin, yaw)
    assert L.to_world(there, origin, yaw) == pytest.approx(point, abs=1e-12)


def test_arm_b_faces_the_counter():
    """Its +x must point back across the worktop, not away from it.

    Yaw only decides where joint angles point, but getting it backwards puts
    every side grasp's approach direction 180 degrees out, and the arm tries
    to reach the counter through its own shoulder.
    """
    ahead = L.to_world((1.0, 0.0, 0.0), L.ARM_B_ORIGIN, L.ARM_B_YAW)
    direction = (ahead[0] - L.ARM_B_ORIGIN[0], ahead[1] - L.ARM_B_ORIGIN[1])
    assert direction[1] < -0.9, 'arm B should face world -y'


def test_side_grasp_tool0_sets_the_flange_back_from_the_pads():
    grip = (0.4, 0.1, 0.2)
    flange = L.side_grasp_tool0(grip)
    assert flange[1:] == pytest.approx(grip[1:], abs=1e-12)
    assert grip[0] - flange[0] == pytest.approx(L.GRIP_AHEAD_OF_TOOL0, abs=1e-12)


# -- crossing the counter with the opener ------------------------------------

def test_the_carried_opener_clears_the_tallest_bottle():
    """The height that matters is the bell rim's, not the flange's.

    The opener hangs OPENER_GRIP_Z below the pads, so a flange height that
    looks generous is not. At the old 0.40 the rim sat 17.5mm over the cola's
    spout and 22.5mm over the whiskey's, and it hit.
    """
    rim = L.OPENER_TRANSIT_Z - L.OPENER_GRIP_Z
    assert rim - L.COUNTER_TALLEST == pytest.approx(L.TRANSIT_CLEARANCE)
    assert rim > L.COUNTER_TALLEST + 0.05


def test_the_tallest_bottle_really_is_the_tallest():
    """COUNTER_TALLEST is a copy of a number that lives in the models.

    If a taller bottle is added, or one grows a spout, this is the line that
    is supposed to fail rather than the opener being the thing that finds out.
    """
    tops = {}
    for name in ('jack_daniels_bottle', 'cola_bottle', 'beer_bottle'):
        path = os.path.join(MODELS, name, 'model.sdf')
        if not os.path.exists(path):
            continue
        top = 0.0
        for col in ET.parse(path).getroot().iter('collision'):
            pose = col.find('pose')
            p = [float(v) for v in pose.text.split()] if pose is not None \
                else [0.0] * 6
            cyl = col.find('geometry/cylinder')
            box = col.find('geometry/box')
            if cyl is not None:
                top = max(top, p[2] + float(cyl.find('length').text) / 2.0)
            elif box is not None:
                size = [float(v) for v in box.find('size').text.split()]
                top = max(top, p[2] + size[2] / 2.0)
        tops[name] = top
    assert tops, 'no bottle models found to check against'
    assert L.COUNTER_TALLEST == pytest.approx(max(tops.values()), abs=1e-6), (
        f'COUNTER_TALLEST is {L.COUNTER_TALLEST}, but the models top out at '
        f'{max(tops.values()):.4f} ({max(tops, key=tops.get)})')


def test_the_opener_cruises_higher_than_an_empty_gripper():
    """Carrying something low-hanging needs more height, not the same."""
    assert L.OPENER_TRANSIT_Z > L.APPROACH_Z


def test_arm_b_can_reach_its_stations_at_cruising_height():
    """Raising the transit is only free while it stays inside the arm.

    Going up costs reach, and arm B works at the far end of the counter.
    """
    for station in ('opener', 'beer'):
        x, y, _ = L.station_in_arm(station, 'b')
        tool = L.side_grasp_tool0((x, y, 0.0))
        out = math.hypot(math.hypot(tool[0], tool[1]), L.OPENER_TRANSIT_Z)
        assert out < UR5E_REACH - 0.10, (
            f'arm B would be {out:.3f}m from its base over the {station} '
            f'at cruising height')
