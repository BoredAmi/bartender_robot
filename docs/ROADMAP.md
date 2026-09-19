# Roadmap

Where the project is, what is blocking it, and what is worth building next.

Dates are deliberately absent. The ordering and the exit criteria are the
useful parts.

## Where it is now

Working, in simulation, measured:

- **Pour** — `PourDrink` makes a whiskey and coke end to end: 45 ml spirit,
  135 ml mixer, both bottles returned upright into their wells.
- **Open** — `OpenBottle` takes the cap off a beer with both arms, one
  holding and one pressing.
- **Teach pendant** — terminal and browser, two arms, tool centre points,
  taught points, and recorded pipelines.
- **950 tests**, none of which need a robot.
- Two recordings of successful runs in `recordings/`.

## What is actually blocking autonomy

Be honest about these before planning anything clever on top. An agent
cannot compensate for a robot that reports success when it has not moved.

### 1. Moves stop short while the controller reports success

**Severity: highest.** Traced twice, identical to 0.1 mm, so it is
deterministic rather than noisy: MoveIt computed a Cartesian path at
**100 %**, `b_ur_arm_controller` reported *"Goal reached, success!"*, and
the flange was **267 mm** from where it was sent — `(0.3291, 0.0208,
0.3406)` instead of `(0.365, 0, 0.077)`. The only thing that noticed was the
skill's own `arrived_at` check.

Leading hypothesis: the arm controllers have no per-joint goal constraints,
so the controller declares success wherever it stops. Not yet confirmed.

**Exit criterion:** a commanded move either arrives within tolerance or
reports failure, with a test that drives a deliberately-blocked move.

### 2. Grippers stop following

Goals accepted (`ok=True`), joint never moves. Observed on both arms. A
fresh stack tracks correctly (0.35 → 0.436, 0.5 → 0.436); a stack that has
been up a while stopped responding entirely. `Arm.grasp` now detects and
names this (`GRIPPER_NOT_FOLLOWING`), but detection is not a fix.

**Exit criterion:** understood well enough to say whether it is a sim
artefact or a controller-configuration bug.

### 3. The open is unreliable

Roughly half of clean runs fail, in several different places. Partly a
symptom of 1 and 2.

### 4. The cola slips out mid-pour

Documented at length in `pour_action_server.py` as the largest remaining
defect in the default path. At the 97° pour angle friction carries the whole
bottle weight; the grip check finds the fingers closed to the full commanded
angle with the bottle gone. Intermittent.

### 5. Smaller, known

- `bartender_open` does not publish the whiskey or the cola into its
  planning scene. The redesign removed the geometry that made it bite; the
  hole remains.
- Gripper commands are deliberately fire-and-forget, so a pipeline `grip`
  step returns while the fingers are still travelling. Use a `wait` step
  after a grip that matters.

## Phases

### Phase A — make the robot honest *(prerequisite for everything)*

Fix defect 1. Investigate 2. Without these, every layer above inherits
"success" that means nothing, and an autonomous caller will build plans on
sand.

**Exit:** a move that did not arrive fails. Three consecutive opens succeed.

### Phase B — the read-only API

`GET /world`, `GET /state`, `POST /can`, and the error taxonomy applied to
the two existing skills. See `docs/CONTROL_API.md`.

No motion, so no risk, and immediately useful: it is what you would put in a
VLM's prompt, and `/can` plus real error codes would have shortened several
past debugging sessions on their own.

**Exit:** a caller can describe the bar and rule out an impossible action
without touching the robot.

### Phase C — the acting API

`POST /do` with jobs and cancellation, movement primitives over
`Pendant.dispatch()`, pipelines as callable verbs, a global stop, and an
action budget.

**Exit:** a script that is not a ROS node can make a whiskey and coke.

### Phase D — more bar

The open question from the layout work: **more bottles.** The line holds
five slots at 0.15 m pitch, three filled, two free — and the two free ones
are on arm B's side, which cannot reach the glass. Three options, in
increasing order of work:

1. **Two spirits in the free slots.** Arm B can pick them; nothing pours
   them. Smallest change, least useful.
2. **Give arm B its own glass.** The two free slots are the *exact mirrors*
   of the whiskey and cola as seen from arm B (r = 0.539 and 0.586, same
   arm-frame coordinates), so a glass at the mirror point `(-0.04, 0.55)`
   makes arm B's station geometrically identical to arm A's and the taught
   joint angles transfer unchanged. The work is in `pour_action_server`,
   which hard-codes `base_link` and `ur_manipulator`.
3. **Re-pitch the line.** Arms out to `y = ±0.50`, pitch down to 0.13, seven
   slots. Buys the most positions, but 130 mm against the whiskey's 103 mm
   across-corners is tight, and every taught point needs re-measuring.

**Recommendation: 2.** It doubles the bar's capability rather than its
inventory, and the mirror symmetry means the geometry is already proven.

### Phase E — perception

Replace `layout.py`'s fixed station poses with something that looks. The API
is already shaped for it: every pose carries `source` and `confidence`, so
callers written in Phase B keep working when ground truth becomes a camera.

`layout.py` stays — it becomes the *prior* and the reachability oracle
rather than the source of pose truth.

### Phase F — hardware

The boundary is `ros2_control`. Recalibrate `layout.py`, replace the
hardware interface, and find out which of the sim's conveniences were load-
bearing. `DetachableJoint` for the cap is the obvious one that does not
survive.

## Ideas worth considering

Not committed to. Roughly by value-per-effort.

**A recipe layer.** Above the skills: "negroni" → a sequence of pours. The
pipeline machinery in `bartender_teach` is most of this already; it needs
quantities and a notion of ingredients rather than stations.

**Replay a failed job.** Every job already knows its stage and its
measurements. Storing them and offering `POST /job/{id}/replay` would make
the flaky failures far easier to characterise — right now every
investigation starts by rebuilding the situation by hand.

**A soak harness.** `relaunch.sh` and friends in the scratchpad are ad hoc
and have been rewritten several times this project. A proper `make soak
N=20` that runs a skill N times on fresh stacks and reports a success rate
would turn "the open is flaky" into a number, and would tell you whether a
fix worked.

**Generate `GET /skills` from the `.action` files.** Keeps the agent-facing
schema honest for free, and makes adding a skill a one-place change.

**An MCP front end** once the verb set settles. Maps nearly one-to-one onto
`/skills`.

**Voice or a chat front end.** Cheap once the API exists; the actions
already emit human-readable `state` feedback designed for narration.

**A second glass, and two drinks at once.** Falls out of Phase D option 2.

**Force feedback instead of position-only grasping.** Several of the grip
problems here are really "we can only measure where the fingers are, not
what they are holding". A simulated force sensor would be a research
direction rather than a fix.

## Non-goals, for now

- **Real-time control.** Everything here is plan-then-execute and should
  stay that way.
- **Multi-robot.** Two arms on one controller is enough.
- **A mobile base.** The bar comes to the robot.
