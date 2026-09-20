"""Tests for GET /world's assembly.

Driven by a plain dict standing in for live poses -- world.build() itself
needs no ROS, only a pose_lookup callable, so this is what exercises that
contract.
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    *([os.pardir] * 4), 'ros2_ws', 'src', 'bartender_open'))

from bartender_api import world                             # noqa: E402
from bartender_open import layout as L                       # noqa: E402


def _stations_by_id(doc):
    return {s['id']: s for s in doc['stations']}


def test_every_named_station_appears_exactly_once():
    doc = world.build(lambda name: None)
    stations = _stations_by_id(doc)
    for name in L.STATIONS:
        assert name in stations
    assert len(stations) == len(set(s['id'] for s in doc['stations']))


def test_occupied_follows_the_live_pose_not_a_guess():
    """A station with no live pose is unoccupied; one with a pose is."""
    live = {'beer_bottle': (0.08, 0.0, 0.9)}
    doc = world.build(lambda name: live.get(name))
    stations = _stations_by_id(doc)
    assert stations['beer']['occupied'] is True
    assert stations['beer']['pose']['xyz'] == [0.08, 0.0, 0.9]
    assert stations['whiskey']['occupied'] is False
    assert stations['whiskey']['pose'] is None


def test_the_station_to_model_name_translation_is_used():
    """'whiskey' the station must be looked up as 'jack_daniels_bottle'.

    The two vocabularies do not match (see world.STATION_MODEL); a lookup
    that used the station name directly would silently show every bottle
    as unoccupied forever, since 'whiskey' never appears on the pose topic.
    """
    seen = []

    def pose_lookup(model_name):
        seen.append(model_name)
        return None

    world.build(pose_lookup)
    assert 'jack_daniels_bottle' in seen
    assert 'whiskey' not in seen


def test_pose_carries_source_and_confidence():
    """Every live pose must say where it came from.

    See CONTROL_API.md:

    "Every pose carries source and confidence... when perception replaces
    it the field already exists, and callers written against it do not
    change." A pose missing either field breaks that promise silently.
    """
    doc = world.build(lambda name: (0.0, 0.0, 0.9) if name == 'beer_bottle'
                      else None)
    beer = _stations_by_id(doc)['beer']
    assert beer['pose']['source'] == 'sim_ground_truth'
    assert beer['pose']['confidence'] == 1.0


def test_reachable_by_is_computed_not_hand_written():
    """Bottle stations must equal layout.servicing_arms(), never a stale copy."""
    doc = world.build(lambda name: None)
    stations = _stations_by_id(doc)
    for name in ('whiskey', 'cola', 'beer'):
        xy = L.STATIONS[name]
        assert stations[name]['reachable_by'] == L.servicing_arms(xy)


def test_the_glass_is_reachable_even_though_nothing_grips_it():
    """Regression test: servicing_arms() alone says no arm reaches the glass.

    Which is wrong -- a bottle is tilted OVER the glass, not side-grasped,
    and the real pour skill reaches it every day. See reach.py and
    feasibility.py's identical fix for /can.
    """
    doc = world.build(lambda name: None)
    glass = _stations_by_id(doc)['glass']
    assert glass['reachable_by'] == ['a']
    opener = _stations_by_id(doc)['opener']
    assert opener['reachable_by'] == ['b']


def test_empty_slots_are_stations_too():
    """Free slots matter too -- "where could I put this down" needs them."""
    doc = world.build(lambda name: None)
    empty = [s for s in doc['stations'] if s['kind'] == 'empty_slot']
    assert len(empty) == len(L.free_slots())
    for s in empty:
        assert s['occupied'] is False
        assert s['pose'] is None


def test_empty_slot_ids_match_the_lines_own_index_scheme():
    doc = world.build(lambda name: None)
    empty_ids = {s['id'] for s in doc['stations'] if s['kind'] == 'empty_slot'}
    expected = {f'slot_{round(y / L.SLOT_PITCH):+d}'
                for y, name in L.BOTTLE_SLOTS if name is None}
    assert empty_ids == expected


def test_arms_report_their_real_origins():
    doc = world.build(lambda name: None)
    by_id = {a['id']: a for a in doc['arms']}
    assert by_id['a']['origin'] == list(L.ARM_A_ORIGIN)
    assert by_id['b']['origin'] == list(L.ARM_B_ORIGIN)


def test_counter_matches_layout():
    doc = world.build(lambda name: None)
    assert doc['counter']['top_z'] == L.COUNTER_Z
    assert doc['counter']['size'] == list(L.COUNTER_SIZE)
