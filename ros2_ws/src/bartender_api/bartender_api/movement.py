"""Phase C, scoped to simple movement: goto / jog / gripper over Pendant.

Wraps `Pendant.dispatch()` rather than reimplementing it -- see
docs/CONTROL_API.md's "The movement API should wrap Pendant.dispatch(), not
reimplement it." That is what already encodes every bound this project has
found a reason for: MAX_JOG_MM/MAX_JOG_DEG (refuse, don't clamp), the
gripper's refusal to park on its lower limit, and `goto` always driving the
arm a point was taught on rather than whichever one is selected.

`dispatch()` itself has no structured result -- it prints, and the only
line shapes it is known to produce are documented on `_classify` below.
This module pre-validates everything it can (axis name, jog bound, gripper
bound, point existence) so dispatch is only ever called with something it
should accept, which is what keeps `_classify`'s text-matching honest
rather than a guess.

Two absolute-target primitives from docs/CONTROL_API.md's sketch --
`/move/joints` (an arbitrary joint vector) and `/move/tool` (an arbitrary
Cartesian pose) -- are deliberately NOT built here. Pendant has no verb for
either; `goto` only drives to a taught point and `jog` only moves
relative to where the arm already is. Adding them would mean either a new
Pendant command (real work, not done here) or calling TeachNode's
move_to_joints/move_cartesian directly and losing every bound dispatch
provides for free. "Simple movement for now" is goto, jog, and the
gripper -- the three things Pendant already does safely.
"""
import contextlib
import io
import threading

from bartender_teach.point_store import PointStoreError
from bartender_teach.teach_points import (
    ARMS, GRIPPER_OPEN_POS, GRIPPER_UPPER_LIMIT, MAX_JOG_DEG, MAX_JOG_MM,
    Pendant,
)

# j1..j6 are handled separately (any of the arm's six joints); these are the
# rest of what Pendant.cmd_jog recognises -- see AXES and cmd_jog itself.
_LINEAR_AXES = {'x', 'y', 'z', 'tx', 'ty', 'tz'}
_ROTATION_AXES = {'rx', 'ry', 'rz'}


def _is_joint_axis(axis):
    return axis.startswith('j') and axis[1:].isdigit() and 1 <= int(axis[1:]) <= 6


def _jog_limit(axis):
    """(limit, unit) for `axis` -- MAX_JOG_MM for linear, MAX_JOG_DEG else.

    Mirrors cmd_jog's own branches exactly, against the same two constants,
    so a request this rejects is one dispatch would also have refused.
    """
    if axis in _LINEAR_AXES:
        return MAX_JOG_MM, 'mm'
    return MAX_JOG_DEG, 'deg'


def _refuse(message):
    return {'ok': False, 'message': message}


def _classify(text):
    """Report whether dispatch's captured output means the command worked.

    dispatch() has no structured (ok, message) result at its own boundary
    -- see its docstring -- so this reads the two shapes its call sites
    actually produce: Pendant._report's "FAILED: {why}" for a motion that
    was attempted and failed, and _require_pose's "cannot read ... through
    /compute_fk" for the one runtime refusal this module does not
    pre-validate away (move_group not running). Everything else dispatch
    could refuse -- an unknown axis, a jog past its bound, an unknown point,
    a gripper position off its band -- is caught before dispatch ever runs;
    see goto()/jog()/gripper() below. An empty buffer (should not happen if
    a line was actually dispatched) is treated as failure rather than a
    silent success.
    """
    if not text.strip():
        return False
    return 'FAILED:' not in text and 'cannot read' not in text


class MovementBridge:
    """Runs Pendant commands for the HTTP layer, one at a time.

    Mirrors teach_gui.py's Bridge exactly, for the same reason:
    dispatch() blocks the calling thread until the robot finishes moving,
    and two motion goals interleaved on one arm is the failure this exists
    to prevent. This bridge and the browser pendant's each hold their own
    lock in their own process, so the one thing this does NOT prevent is
    the browser pendant and this API racing each other on the same arm --
    a real, known gap, no different from the one that already exists
    between the browser and terminal pendants.
    """

    def __init__(self, node, store):
        self.node = node
        self.pendant = Pendant(node, store)
        self._lock = threading.Lock()

    def _run(self, arm, line):
        if arm not in ARMS:
            return _refuse(f"no arm {arm!r}; known: {', '.join(ARMS)}")
        if not self._lock.acquire(blocking=False):
            return _refuse('busy: a command is already running')
        try:
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                self.pendant.dispatch(f'arm {arm}')
                self.pendant.dispatch(line)
            text = buf.getvalue()
        except Exception as exc:                            # noqa: BLE001
            # dispatch() already swallows the failures it knows about;
            # anything reaching here is a bug and must not take the whole
            # server down with it.
            self.node.get_logger().error(
                f'movement command {line!r} raised: {exc}')
            return {'ok': False,
                    'message': f'unexpected error: {type(exc).__name__}: {exc}'}
        finally:
            self._lock.release()
        return {'ok': _classify(text), 'message': text.strip()}

    def goto(self, arm, point):
        if not point:
            return _refuse('goto needs a point name')
        try:
            self.pendant.store.get(point)
        except PointStoreError as exc:
            return _refuse(str(exc))
        return self._run(arm, f'goto {point}')

    def jog(self, arm, axis, amount):
        axis = axis.lower() if isinstance(axis, str) else ''
        if not (axis in _LINEAR_AXES or axis in _ROTATION_AXES
                or _is_joint_axis(axis)):
            return _refuse(
                f'unknown jog axis {axis!r}. Expected j1..j6, x/y/z, '
                f'tx/ty/tz or rx/ry/rz.')
        try:
            amount = float(amount)
        except (TypeError, ValueError):
            return _refuse(f'jog amount must be a number, got {amount!r}')
        limit, unit = _jog_limit(axis)
        if abs(amount) > limit:
            return _refuse(
                f'{amount:g}{unit} exceeds the {limit:g}{unit} jog limit. '
                f'Refusing rather than clamping -- send it in steps if '
                f'that is really what you meant.')
        return self._run(arm, f'jog {axis} {amount:g}')

    def gripper(self, arm, position):
        try:
            position = float(position)
        except (TypeError, ValueError):
            return _refuse(
                f'gripper position must be a number, got {position!r}')
        if not GRIPPER_OPEN_POS <= position <= GRIPPER_UPPER_LIMIT:
            return _refuse(
                f'gripper position {position:.4f} is outside '
                f'{GRIPPER_OPEN_POS:.2f}..{GRIPPER_UPPER_LIMIT:.2f}; resting '
                f'on the lower joint limit stops it responding for the '
                f'rest of the run.')
        return self._run(arm, f'close {position:g}')
