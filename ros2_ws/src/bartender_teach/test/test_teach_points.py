"""Tests for the parts of the pendant that do not need a robot.

The quaternion helpers and the command parsing are where a bug would be
silent: a wrong rotation still produces a perfectly valid pose, and the arm
goes somewhere nobody asked for. The motion calls themselves are stubbed --
what is checked here is WHICH pose the pendant asks for, and which commands it
refuses.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bartender_teach.point_store import Point, PointStore   # noqa: E402
from bartender_teach.teach_points import (                  # noqa: E402
    ARMS, ARM_JOINTS, MAX_JOG_DEG, MAX_JOG_MM, Pendant,
    quat_about, quat_mul, quat_rotate,
)

ARM_A = ARMS['a']
ARM_B = ARMS['b']

SAMPLE = dict(zip(ARM_JOINTS, [0.079, -1.905, 2.337, -0.432, 1.65, 0.0]))
IDENTITY = (0.0, 0.0, 0.0, 1.0)
# The side grasp's tool orientation: the cyclic axis permutation that maps
# tool0's local +Z onto base +X, local +X onto base +Y, local +Y onto base +Z.
SIDE_QUAT = (0.5, 0.5, 0.5, 0.5)


# -- quaternion helpers -----------------------------------------------------

def test_identity_rotates_nothing():
    assert quat_rotate(IDENTITY, (1.0, 2.0, 3.0)) == pytest.approx((1, 2, 3))


@pytest.mark.parametrize('axis,vec,expect', [
    ('z', (1, 0, 0), (0, 1, 0)),
    ('z', (0, 1, 0), (-1, 0, 0)),
    ('x', (0, 1, 0), (0, 0, 1)),
    ('y', (0, 0, 1), (1, 0, 0)),
])
def test_quarter_turns_are_right_handed(axis, vec, expect):
    axes = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}
    q = quat_about(axes[axis], math.pi / 2)
    assert quat_rotate(q, vec) == pytest.approx(expect, abs=1e-12)


def test_side_quat_is_the_permutation_pour_action_server_documents():
    """If this fails, `jog tz` runs the tool in the wrong direction."""
    assert quat_rotate(SIDE_QUAT, (0, 0, 1)) == pytest.approx((1, 0, 0), abs=1e-12)
    assert quat_rotate(SIDE_QUAT, (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-12)
    assert quat_rotate(SIDE_QUAT, (0, 1, 0)) == pytest.approx((0, 0, 1), abs=1e-12)


def test_rotation_preserves_length():
    q = quat_about((1 / math.sqrt(3),) * 3, 0.7)
    v = (0.3, -1.2, 0.8)
    assert (sum(c * c for c in quat_rotate(q, v))
            == pytest.approx(sum(c * c for c in v)))


def test_quat_mul_matches_composed_rotation():
    a = quat_about((0, 0, 1), 0.4)
    b = quat_about((1, 0, 0), -0.9)
    v = (0.2, 0.5, -0.3)
    assert quat_rotate(quat_mul(a, b), v) == pytest.approx(
        quat_rotate(a, quat_rotate(b, v)), abs=1e-12)


# -- a pendant with the robot stubbed out -----------------------------------

class FakeNode:
    """Records what it was asked to do instead of doing it.

    Every call records the ARM it was made against as well as its arguments,
    because with two arms "the pendant asked for the right pose" is only half
    the question -- it has to have asked the right arm for it.
    """

    def __init__(self, pose=((0.5, 0.1, 0.3), IDENTITY)):
        self._pose = pose
        self.joint_moves = []
        self.cartesian = []
        self.gripper = []
        self.arms_asked = []

    def arm_joints(self, arm=ARM_A):
        self.arms_asked.append(arm.key)
        return dict(zip(arm.joints, SAMPLE.values()))

    def gripper_position(self, arm=ARM_A):
        return 0.1

    def tool_pose(self, joints=None, arm=ARM_A):
        return self._pose

    def move_to_joints(self, positions, label='', arm=ARM_A):
        self.joint_moves.append(dict(positions))
        self.arms_moved = getattr(self, 'arms_moved', [])
        self.arms_moved.append(arm.key)
        return True, ''

    def move_cartesian(self, pose, avoid_collisions=True, label='', arm=ARM_A):
        self.cartesian.append((pose, avoid_collisions))
        self.arms_jogged = getattr(self, 'arms_jogged', [])
        self.arms_jogged.append(arm.key)
        return True, ''

    def command_gripper(self, position, arm=ARM_A):
        self.gripper.append(position)
        self.arms_gripped = getattr(self, 'arms_gripped', [])
        self.arms_gripped.append(arm.key)
        return True, ''


@pytest.fixture
def pendant(tmp_path):
    return Pendant(FakeNode(), PointStore(str(tmp_path / 'p.yaml')))


def xyz(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def quat(pose):
    return (pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)


# -- jogging ----------------------------------------------------------------

def test_base_jog_moves_the_named_axis_only(pendant):
    pendant.dispatch('jog z 20')
    pose, _ = pendant.node.cartesian[0]
    assert xyz(pose) == pytest.approx((0.5, 0.1, 0.32))
    assert quat(pose) == pytest.approx(IDENTITY)


def test_jog_is_in_millimetres(pendant):
    pendant.dispatch('jog x 100')
    assert xyz(pendant.node.cartesian[0][0]) == pytest.approx((0.6, 0.1, 0.3))


def test_tool_jog_follows_the_tool_orientation(tmp_path):
    """Tool-frame jogs must follow the tool, not the base.

    The tz axis is the approach direction. Under the side grasp's orientation
    that is base +X, so `jog tz 50` must run the tool 50mm toward the bottle,
    not 50mm upward.
    """
    node = FakeNode(pose=((0.5, 0.1, 0.3), SIDE_QUAT))
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('jog tz 50')
    assert xyz(node.cartesian[0][0]) == pytest.approx((0.55, 0.1, 0.3), abs=1e-9)


def test_rotation_jog_holds_position(pendant):
    pendant.dispatch('jog ry 30')
    pose, _ = pendant.node.cartesian[0]
    assert xyz(pose) == pytest.approx((0.5, 0.1, 0.3))
    assert quat(pose) == pytest.approx(
        quat_about((0, 1, 0), math.radians(30)), abs=1e-12)


def test_joint_jog_adds_to_that_joint_in_degrees(pendant):
    pendant.dispatch('jog j1 10')
    moved = pendant.node.joint_moves[0]
    assert moved['shoulder_pan_joint'] == pytest.approx(
        SAMPLE['shoulder_pan_joint'] + math.radians(10))
    for j in ARM_JOINTS[1:]:
        assert moved[j] == pytest.approx(SAMPLE[j])


@pytest.mark.parametrize('line', [
    f'jog z {MAX_JOG_MM + 1}',
    f'jog x -{MAX_JOG_MM + 1}',
    f'jog tz {MAX_JOG_MM + 1}',
    f'jog j1 {MAX_JOG_DEG + 1}',
    f'jog rx -{MAX_JOG_DEG + 1}',
])
def test_oversized_jogs_are_refused_not_clamped(pendant, line):
    pendant.dispatch(line)
    assert pendant.node.cartesian == []
    assert pendant.node.joint_moves == []


@pytest.mark.parametrize('line', [
    'jog', 'jog z', 'jog z 1 2', 'jog w 10', 'jog j9 10', 'jog j0 10',
    'jog z banana', 'jog qz 10',
])
def test_bad_jog_commands_move_nothing(pendant, line):
    pendant.dispatch(line)
    assert pendant.node.cartesian == []
    assert pendant.node.joint_moves == []


def test_jog_at_the_limit_is_allowed(pendant):
    pendant.dispatch(f'jog z {MAX_JOG_MM}')
    assert len(pendant.node.cartesian) == 1


# -- safety -----------------------------------------------------------------

def test_cartesian_jogs_are_collision_checked_by_default(pendant):
    pendant.dispatch('jog z 10')
    assert pendant.node.cartesian[0][1] is True


def test_safety_off_is_passed_through(pendant):
    pendant.dispatch('safety off')
    pendant.dispatch('jog z 10')
    assert pendant.node.cartesian[0][1] is False
    pendant.dispatch('safety on')
    pendant.dispatch('jog z 10')
    assert pendant.node.cartesian[1][1] is True


def test_bad_safety_argument_does_not_change_the_setting(pendant):
    pendant.dispatch('safety maybe')
    assert pendant.safety is True


# -- saving -----------------------------------------------------------------

def test_save_writes_immediately(pendant):
    pendant.dispatch('save grasp beside the whiskey')
    assert os.path.exists(pendant.store.path)
    point = PointStore.load(pendant.store.path).get('grasp')
    assert point.joints_in_order(ARM_JOINTS) == pytest.approx(
        [SAMPLE[j] for j in ARM_JOINTS])
    assert point.note == 'beside the whiskey'
    assert point.gripper == pytest.approx(0.1)
    assert point.pose['xyz'] == [0.5, 0.1, 0.3]


def test_save_does_not_clobber_but_resave_does(pendant):
    pendant.dispatch('save grasp first')
    pendant.dispatch('save grasp second')
    assert pendant.store.get('grasp').note == 'first'
    pendant.dispatch('resave grasp second')
    assert pendant.store.get('grasp').note == 'second'


def test_save_wraps_branches_out_of_range(tmp_path):
    node = FakeNode()
    node.arm_joints = lambda arm=ARM_A: dict(SAMPLE, wrist_3_joint=7.0)
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('save far')
    assert abs(p.store.get('far').joints['wrist_3_joint']) <= math.pi


def test_save_without_fk_still_records_joints(tmp_path):
    node = FakeNode()
    node.tool_pose = lambda joints=None, arm=ARM_A: None
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('save nofk')
    point = PointStore.load(p.store.path).get('nofk')
    assert point.pose is None
    assert point.joints_in_order(ARM_JOINTS)


def test_rm_persists(pendant):
    pendant.dispatch('save a')
    pendant.dispatch('rm a')
    assert len(PointStore.load(pendant.store.path)) == 0


# -- goto, gripper, dispatch ------------------------------------------------

def test_goto_sends_the_stored_joints(pendant):
    pendant.store.add(Point('target', SAMPLE))
    pendant.dispatch('goto target')
    assert pendant.node.joint_moves[0] == pytest.approx(SAMPLE)


def test_goto_unknown_point_moves_nothing(pendant):
    pendant.dispatch('goto nowhere')
    assert pendant.node.joint_moves == []


def test_goto_incomplete_point_moves_nothing(pendant):
    pendant.store.add(Point('half', {'shoulder_pan_joint': 0.1}))
    pendant.dispatch('goto half')
    assert pendant.node.joint_moves == []


def test_gripper_commands(pendant):
    pendant.dispatch('open')
    pendant.dispatch('close')
    pendant.dispatch('close 0.42')
    assert pendant.node.gripper == pytest.approx([0.0, 0.5, 0.42])


@pytest.mark.parametrize('line', ['quit', 'exit', 'q', 'QUIT'])
def test_quit_words(pendant, line):
    assert pendant.dispatch(line) is False


@pytest.mark.parametrize('line', ['', '   ', 'help', 'state', 'list', 'file',
                                  'nonsense', 'show', 'rm', 'export',
                                  'save', 'goto', 'jog z "unclosed'])
def test_no_command_can_end_the_session(pendant, line):
    """A typo must never drop the operator out of the pendant.

    Quitting mid-session means re-teaching every point recorded in it.
    """
    assert pendant.dispatch(line) is True


def test_export_emits_a_pastable_snippet(pendant, capsys):
    pendant.store.add(Point('target', SAMPLE))
    pendant.dispatch('export target')
    out = capsys.readouterr().out
    assert 'approach_joints=[' in out
    assert '0.0790' in out and '-1.9050' in out


# -- tool centre points -----------------------------------------------------

def test_default_tool_is_the_flange(pendant):
    assert pendant.tool.name == 'tool0'


def test_tool_lists_without_selecting(pendant, capsys):
    pendant.dispatch('tool')
    out = capsys.readouterr().out
    assert 'whiskey_spout' in out and 'cola_spout' in out
    assert pendant.tool.name == 'tool0'


def test_tool_selects(pendant):
    pendant.dispatch('tool whiskey_spout')
    assert pendant.tool.name == 'whiskey_spout'


def test_unknown_tool_lists_the_alternatives_and_changes_nothing(pendant, capsys):
    pendant.dispatch('tool spanner')
    assert 'whiskey_spout' in capsys.readouterr().out
    assert pendant.tool.name == 'tool0'


def test_rotation_jog_holds_the_selected_tools_tip(tmp_path):
    """The whole request: turn the bottle, keep the spout where it is."""
    from bartender_teach.tool_frames import get_tool, tcp_from_tool0
    node = FakeNode(pose=((0.5, 0.1, 0.3), SIDE_QUAT))
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('tool whiskey_spout')
    tool = get_tool('whiskey_spout')
    before, _ = tcp_from_tool0((0.5, 0.1, 0.3), SIDE_QUAT, tool)

    p.dispatch('jog ry 30')
    pose, _ = node.cartesian[0]
    after, _ = tcp_from_tool0(xyz(pose), quat(pose), tool)
    assert after == pytest.approx(before, abs=1e-9)
    # ...and the flange really did move, so this is not a no-op.
    assert math.dist(xyz(pose), (0.5, 0.1, 0.3)) > 0.02


def test_rotation_jog_with_tool0_is_unchanged(pendant):
    """Adding tool frames must not alter the default behaviour."""
    pendant.dispatch('jog ry 30')
    pose, _ = pendant.node.cartesian[0]
    assert xyz(pose) == pytest.approx((0.5, 0.1, 0.3))


def test_translation_jog_is_the_same_whatever_the_tool(tmp_path):
    moved = []
    for tool in ('tool0', 'whiskey_spout'):
        node = FakeNode(pose=((0.5, 0.1, 0.3), SIDE_QUAT))
        p = Pendant(node, PointStore(str(tmp_path / f'{tool}.yaml')))
        p.dispatch(f'tool {tool}')
        p.dispatch('jog z 20')
        moved.append(xyz(node.cartesian[0][0]))
    assert moved[0] == pytest.approx(moved[1])


def test_saved_point_records_the_active_tool(pendant):
    pendant.dispatch('tool cola_spout')
    pendant.dispatch('save p')
    assert PointStore.load(pendant.store.path).get('p').tool == 'cola_spout'


def test_state_shows_the_tool_tip_when_one_is_selected(pendant, capsys):
    pendant.dispatch('tool whiskey_spout')
    capsys.readouterr()
    pendant.dispatch('state')
    assert 'whiskey_spout' in capsys.readouterr().out


# -- two arms ----------------------------------------------------------------

def test_the_pendant_starts_on_arm_a(pendant):
    """Arm A is the pouring arm and everything that came before is its.

    A pendant that came up on arm B would make every pre-existing habit --
    and every note in pour_action_server -- quietly wrong.
    """
    assert pendant.arm.key == 'a'
    assert pendant.arm.group == 'ur_manipulator'


def test_selecting_arm_b_moves_every_command_to_it(pendant):
    pendant.dispatch('arm b')
    pendant.dispatch('jog j1 5')
    pendant.dispatch('close 0.3')
    assert pendant.node.arms_moved == ['b']
    assert pendant.node.arms_gripped == ['b']
    # ...and it jogged a b_ joint, not arm A's.
    assert all(n.startswith('b_') for n in pendant.node.joint_moves[-1])


def test_an_unknown_arm_is_refused_and_changes_nothing(pendant):
    with pytest.raises(ValueError, match='no arm'):
        pendant.cmd_arm(['c'])
    assert pendant.arm.key == 'a'


def test_arm_b_plans_in_its_own_base_frame(pendant):
    """b_base_link, not base_link.

    The arms are 0.55 x 0.62 apart, so a Cartesian jog sent in the wrong
    frame lands most of a metre away. This is the mistake the Arm record
    exists to make impossible.
    """
    assert ARM_B.frame == 'b_base_link'
    assert ARM_B.eef_link == 'b_tool0'
    assert ARM_A.frame == 'base_link'


def test_goto_uses_the_arm_the_point_was_taught_on(tmp_path):
    """Not the selected arm -- a point belongs to exactly one arm."""
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('b_rest', dict(zip(ARM_B.joints, [0.0, -2.0, 1.6, -1.17,
                                                      -1.57, 0.0])),
                    group=ARM_B.group))
    p = Pendant(FakeNode(), store)          # selected arm is A
    p.dispatch('goto b_rest')
    assert p.node.arms_moved == ['b']
    assert all(n.startswith('b_') for n in p.node.joint_moves[-1])


def test_a_point_saved_on_arm_b_records_arm_bs_group(tmp_path):
    node = FakeNode()
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('arm b')
    p.dispatch('save over_the_cap')
    saved = PointStore.load(p.store.path).get('over_the_cap')
    assert saved.group == 'b_ur_manipulator'
    assert all(n.startswith('b_') for n in saved.joints)


def test_switching_arms_drops_a_selected_tool(tmp_path):
    """A tool offset is arm A's bottle spout; applying it to arm B is wrong."""
    from bartender_teach.tool_frames import TOOLS
    p = Pendant(FakeNode(), PointStore(str(tmp_path / 'p.yaml')))
    p.tool = list(TOOLS.values())[0]
    p.dispatch('arm b')
    assert p.tool.name == 'tool0'


def test_list_shows_both_arms_points(tmp_path, capsys):
    """`list` answers "what does the robot know", so it never hides an arm."""
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('a_point', dict(zip(ARM_A.joints, [0.1] * 6)),
                    group=ARM_A.group))
    store.add(Point('b_point', dict(zip(ARM_B.joints, [0.2] * 6)),
                    group=ARM_B.group))
    p = Pendant(FakeNode(), store)
    p.dispatch('list')
    out = capsys.readouterr().out
    assert 'a_point' in out and 'b_point' in out
    assert 'arm A' in out and 'arm B' in out
