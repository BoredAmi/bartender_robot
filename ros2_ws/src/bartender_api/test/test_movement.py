"""Tests for movement.py -- goto / jog / gripper over Pendant.dispatch().

Pendant itself needs a live rclpy TeachNode to actually move anything, so
these tests split in two: the pre-validation MovementBridge does BEFORE
ever calling dispatch (real Pendant, real PointStore, a bare stand-in
node), and the classification of dispatch's captured text AFTER it runs
(dispatch itself faked, since exercising the real one needs move_group).
Both halves are real code paths, just on either side of the one call this
module cannot unit-test without a simulator.
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    *([os.pardir] * 4), 'ros2_ws', 'src', 'bartender_teach'))

from bartender_api import movement                          # noqa: E402
from bartender_teach.point_store import Point, PointStore    # noqa: E402
from bartender_teach.teach_points import ARMS                # noqa: E402


def _bridge(with_point=True):
    store = PointStore('/tmp/does-not-matter.yaml')
    if with_point:
        store.add(Point('whiskey_approach',
                        {j: 0.0 for j in ARMS['a'].joints}))
    node = types.SimpleNamespace(
        get_logger=lambda: types.SimpleNamespace(error=lambda m: None))
    return movement.MovementBridge(node, store)


# -- pre-validation: refused before dispatch ever runs -----------------

def test_goto_refuses_an_unknown_point_without_touching_dispatch():
    bridge = _bridge(with_point=False)
    calls = []
    bridge.pendant.dispatch = lambda line: calls.append(line)
    result = bridge.goto('a', 'no_such_point')
    assert result['ok'] is False
    assert 'no point named' in result['message']
    assert calls == []


def test_goto_refuses_an_unknown_arm():
    bridge = _bridge()
    result = bridge.goto('c', 'whiskey_approach')
    assert result['ok'] is False
    assert 'no arm' in result['message']


def test_jog_refuses_an_unknown_axis():
    bridge = _bridge()
    result = bridge.jog('a', 'q', 10)
    assert result['ok'] is False
    assert 'unknown jog axis' in result['message']


def test_jog_refuses_a_non_numeric_amount():
    bridge = _bridge()
    result = bridge.jog('a', 'z', 'a lot')
    assert result['ok'] is False
    assert 'must be a number' in result['message']


def test_jog_refuses_a_linear_amount_past_max_jog_mm():
    bridge = _bridge()
    result = bridge.jog('a', 'z', movement.MAX_JOG_MM + 1)
    assert result['ok'] is False
    assert 'Refusing rather than clamping' in result['message']


def test_jog_refuses_a_rotation_amount_past_max_jog_deg():
    bridge = _bridge()
    result = bridge.jog('a', 'rz', movement.MAX_JOG_DEG + 1)
    assert result['ok'] is False
    assert 'Refusing rather than clamping' in result['message']


def test_jog_accepts_every_real_axis_shape_within_bound():
    """One reading past the refusal for each family cmd_jog recognises."""
    bridge = _bridge()
    for axis in ('x', 'y', 'z', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz',
                 'j1', 'j6'):
        bridge.pendant.dispatch = lambda line: print('  ok')
        result = bridge.jog('a', axis, 1.0)
        assert result['ok'] is True, f'axis {axis} was refused: {result}'


def test_jog_refuses_a_joint_index_out_of_range():
    bridge = _bridge()
    result = bridge.jog('a', 'j7', 1.0)
    assert result['ok'] is False


def test_gripper_refuses_a_position_outside_the_usable_band():
    bridge = _bridge()
    result = bridge.gripper('a', 0.0)
    assert result['ok'] is False
    assert 'outside' in result['message']


def test_gripper_refuses_a_non_numeric_position():
    bridge = _bridge()
    result = bridge.gripper('a', 'wide open')
    assert result['ok'] is False
    assert 'must be a number' in result['message']


# -- locking: refused, not queued ---------------------------------------

def test_a_second_command_is_refused_while_one_is_running():
    bridge = _bridge()
    assert bridge._lock.acquire(blocking=False)
    try:
        result = bridge.gripper('a', 0.3)
    finally:
        bridge._lock.release()
    assert result['ok'] is False
    assert 'busy' in result['message']


# -- classification of dispatch's own captured text ----------------------

def test_a_successful_report_line_classifies_as_ok():
    text = '  now driving arm A ...\n  moving to whiskey_approach ...\n  at whiskey_approach\n'
    assert movement._classify(text) is True


def test_a_failed_report_line_classifies_as_not_ok():
    text = "  jog z +20mm ...\n  FAILED: flange is 12.0mm from where it was sent\n"
    assert movement._classify(text) is False


def test_an_unreachable_pose_service_classifies_as_not_ok():
    """_require_pose's refusal: no "FAILED:" prefix, but still a failure."""
    text = ('  cannot read tool0 through /compute_fk, so there is no pose '
            'to jog FROM. Is move_group running? Joint jogs still work.\n')
    assert movement._classify(text) is False


def test_empty_output_is_not_ok():
    """Dispatch always prints something for a real command; nothing is a bug."""
    assert movement._classify('') is False


# -- end to end through the real Pendant, dispatch's prints faked --------

def test_goto_runs_arm_select_then_goto_through_dispatch():
    bridge = _bridge()
    lines = []

    def fake_dispatch(line):
        lines.append(line)
        print('  at whiskey_approach' if line.startswith('goto') else '  ok')

    bridge.pendant.dispatch = fake_dispatch
    result = bridge.goto('a', 'whiskey_approach')
    assert lines == ['arm a', 'goto whiskey_approach']
    assert result == {'ok': True, 'message': 'ok\n  at whiskey_approach'}
