# bartender_teach

A teach pendant for recording named arm configurations, and the small library
that reads them back. Two front ends, one set of commands.

```bash
ros2 run bartender_teach teach_gui    # browser UI -> http://127.0.0.1:8080
ros2 run bartender_teach teach        # terminal, same commands typed
```

## Why

Every pose the pour cycle uses started as a literal in
`bartender_pour/pour_action_server.py` — the bottles' `approach_joints`,
`HOME_JOINTS`. Each one was obtained by calling `/compute_ik` by hand, reading
a branch off the console, normalising it into `[-pi, pi]`, and pasting it in.
That is slow, and it is wrong in a way nobody notices until the arm sweeps
somewhere it should not: `/compute_ik` returns arbitrary branches, often out
near ±2π, which describe the same pose but drag the gripper through
`forearm_link` on the way.

This does the same job by driving the robot to the pose and naming it.

## The browser UI

```
ros2 run bartender_teach teach_gui
  teach pendant:  http://127.0.0.1:8080
  points file:    /home/.../src/bartender_teach/config/taught_points.yaml  (3 point(s))
```

Live joint angles and tool pose, +/- buttons for every joint and every
Cartesian axis with a selectable step, gripper controls, the point list with
Go and Delete, and a box to save where the arm is now. Arrow keys jog base Z
and Y. There is also a command box that takes anything the terminal pendant
takes, so nothing is hidden behind the buttons.

**It owns no robot logic.** Every control posts a command string to
`Pendant.dispatch()` -- the same entry point the terminal uses -- and the page
shows whatever that prints. So the jog bounds, the branch wrapping, the
refusals and the safety flag are all shared by construction, a fix reaches both
front ends at once, and the tests over `Pendant` cover the GUI too.

A second command while one is running is **refused, not queued**. The buttons
grey out while the arm moves, but a double-click, a stale tab or a second
browser can still race, and two motion goals interleaved on one arm is the
exact failure this tool exists to prevent.

It binds **127.0.0.1** by default, and that is a safety decision rather than
laziness: the page moves a robot arm and has no authentication at all.
`--host 0.0.0.0` puts it on the network -- reasonable on an isolated robot LAN,
a bad idea anywhere else -- and it prints a warning when you do. `--port`
changes the port, `--file` the point file.

No dependencies beyond what the package already needs: `http.server` is
stdlib, the page is one self-contained file with no CDN, so it keeps working
with the network down.

## Terminal session

```
$ ros2 run bartender_teach teach
  /home/.../src/bartender_teach/config/taught_points.yaml  (3 point(s))
teach> goto whiskey_approach
  moving to whiskey_approach ...
  at whiskey_approach
teach> jog z -50
  jog z -50mm ...
  jogged z -50mm
teach> jog tz 30              # tz is the tool's approach axis
  jogged tz +30mm
teach> save whiskey_pregrasp  backed off 100mm along +X
  saved whiskey_pregrasp -> /home/.../config/taught_points.yaml
teach> export whiskey_pregrasp
  # whiskey_pregrasp -- backed off 100mm along +X
  approach_joints=[0.0790, -1.9050, 2.3370, -0.4320, 1.6500, 0.0],
```

`help` lists every command. The ones worth knowing:

| | |
|---|---|
| `arm [a\|b]` | which arm to drive, or say which is selected |
| `state` | joints, flange pose, gripper, right now |
| `list` | every point, both arms |
| `save NAME [note]` | record where the arm is; `resave` to overwrite |
| `goto NAME` | plan and move there in joint space |
| `jog j1..j6 DEG` | one joint |
| `jog x\|y\|z MM` | straight line along a base axis |
| `jog tx\|ty\|tz MM` | straight line along a flange axis — `tz` is approach |
| `jog rx\|ry\|rz DEG` | rotate about a base axis, position held |
| `tool [NAME]` | list tool centre points, or select one |
| `open` / `close [POS]` | gripper |
| `export [NAME]` | print as a `pour_action_server` source snippet |

## Two arms

The robot has two, so the pendant does too. `arm a` and `arm b` choose which
one everything acts on, and the prompt says which is live (`teach[b]>`).

Three things are worth knowing, because each is a way to move the wrong arm:

- **Jog and pose axes are in the selected arm's own base frame.** Arm B's is
  `b_base_link`, and the two arms sit 0.55 × 0.62 m apart — a jog issued in
  the wrong frame lands most of a metre from where it was asked for. `state`
  names the frame it is reporting in, every time.
- **`goto` uses the arm the point was taught on, not the selected one.** A
  point is a set of named joint values and those names say which arm it is;
  there is exactly one right answer, so it just goes, and says so when that
  is not the arm you had selected.
- **`list` shows both arms, always.** It answers "what does the robot know",
  and hiding half of that behind a mode is how you teach a second point that
  already exists.

Points record their own `group:` in the file (`ur_manipulator` or
`b_ur_manipulator`), so one file holds both arms. Selecting an arm also drops
any selected tool centre point back to the flange — a spout is a property of
the bottle one arm is carrying, and silently applying it to the other arm's
flange would put the tip a hand's width from where the page says it is.

## Which points exist, and who reads them

Every joint configuration the running system uses is in the file, and both
action servers read it, preferring a taught point over their own literal:

| point | arm | used by |
|---|---|---|
| `home` | A | `bartender_pour` `HOME_JOINTS`, `bartender_open` `ARM_A_HOME`, SRDF `home` |
| `b_home` | B | `bartender_open` `ARM_B_HOME`, SRDF `b_home` |
| `whiskey_approach` | A | `bartender_pour` `WHISKEY.approach_joints` |
| `cola_approach` | A | `bartender_pour` `COLA.approach_joints` |

The literals are kept as fallbacks so a bare or broken workspace still runs,
and `test_seeded_points.py` insists the two copies agree — so retuning a pose
means editing the literal too, and the test names the one you missed.

## Tool centre points

A tool frame is a rigid offset from `tool0`. Selecting one changes what
"rotate" means:

```
teach> tool whiskey_spout
  tool is now whiskey_spout; rotation jogs will hold its tip still
teach> jog ry 15
  jogged ry +15deg about whiskey_spout
```

Measured on the robot, with `jog ry 15`:

| selected | flange moves | spout tip moves |
|---|---|---|
| `tool0` | 0 mm (orientation only) | swings through an arc |
| `whiskey_spout` | 64 mm | 4.5 mm |

That 4.5 mm is the arm's own tracking error — the transform is exact, and a
test sweeps it over arbitrary rotations and both bottles asserting the tip
does not move at all.

A grasped bottle is a tool whose tip is the pour spout fitted in its neck, so
a pour is a rotation about that tip: the spout stays over the glass while the
bottle swings around it. `tool0` is the default and makes every transform the
identity, so nothing changes until you ask for it.

Translation jogs are deliberately *not* tool-dependent — a pure translation
moves flange and tip by the same vector whatever is selected, and a jog that
quietly depended on the tool would be a bug. There is a test for that too.

The offsets live in [`tool_frames.py`](bartender_teach/tool_frames.py), derived
from the bottle models rather than measured by hand; `pour_action_server` uses
the same module, so the pour and the pendant cannot disagree about where a
spout is.

## Behaviour worth knowing about

**Points are saved the instant you type `save`,** not on quit. A teaching
session is the one artefact here that cannot be reproduced by re-running
something, so the write is atomic (temp file + `os.replace`) and immediate.

**Branches are wrapped into `[-pi, pi]` on save, and it says so.** This is the
`/compute_ik` problem above. The wrap only ever removes whole turns, so the
pose is unchanged and only the path to it gets shorter.

**Cartesian jogs are collision checked by default.** That is the opposite of
what `pour_action_server` does, deliberately: that file turns checking off
because its close work is straight lines computed in one shot toward a known
object, whereas here a person is typing distances and a typo should be stopped
by the planner rather than by the bottle. `safety off` is there for teaching a
point that really is in contact, and every jog after it says so.

**Oversized jogs are refused, not clamped** (`MAX_JOG_MM`, `MAX_JOG_DEG`).
Clamping would turn `jog z 500` — a typo for `50` — into a move that quietly
does something other than what was typed, and catching that typo is the whole
reason for the bound.

**Nothing you type can end the session.** A bad point name, an unparseable
line, an out-of-range jog: all print and return to the prompt, because
dropping out means re-teaching everything recorded so far.

## Where points are stored

`config/taught_points.yaml`, in the **source** tree, chosen ahead of the
installed copy. The obvious way to lose a teaching session is to teach a dozen
points into `install/bartender_teach/share/...`, run `colcon build`, and watch
the install directory be repopulated from source. Writing to source means a
rebuild copies the points forward instead of over them.

`$BARTENDER_POINTS` overrides this, and so does `--file`.

Angles are radians. `joints` is a mapping, not a list, because
`/joint_states` publishes joints in controller-registration order — which is
not MoveGroup's order, and a list would record that ordering silently and hand
back a pose that looks plausible and moves the wrong joints.

## How the pour server uses them

`pour_action_server` prefers a taught point over its own literal for `home`,
`whiskey_approach` and `cola_approach`, and logs which source won for each at
startup:

```
[pour_action_server]: point home: taught, from /home/.../taught_points.yaml
```

The literals are kept rather than deleted, and they are what runs if the file
is missing, unreadable, or silent about that point — the server has to come up
on a bare workspace, and a point file is an editable text file a person can
get wrong. The shipped file is seeded with those exact numbers, so on an
untouched workspace both paths give identical motion; `test_seeded_points.py`
asserts that, and is meant to fail if you retune one copy and not the other.

## Tests

```
cd src/bartender_teach && python3 -m pytest test/ -q
```

594 tests, no robot needed — the motion calls are stubbed, so what is checked
is *which* pose the pendant asks for. The quaternion helpers are tested
against known rotations because that is where a bug would be silent: a wrong
rotation still produces a perfectly valid pose, and the arm goes somewhere
nobody asked for. The browser UI is covered too: the bridge against a fake
robot, and the HTTP layer against a real server on an ephemeral port, so the
routing, status codes and JSON shape are checked the way a browser meets them.

Unit tests are not sufficient on their own here, and both halves have the
scars to prove it -- driving the pendant against a live robot found that
Cartesian jogs planned from a start state the arm had already left, and the
HTTP tests found that a JSON body which was a list crashed the handler instead
of answering 400.

If pytest dies with `ModuleNotFoundError: No module named '_pytest.scope'`,
that is a pre-existing clash between the system pytest and a user-installed
`anyio` plugin, unrelated to this package. Run with
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
