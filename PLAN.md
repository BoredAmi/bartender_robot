# Bartender Robot — Project Plan (Phase 1: Simulated Pouring)

## Goal
Get a UR5e + parallel gripper to reliably pick up a bottle and pour a measured
amount into a glass, fully in Gazebo simulation, driven by ROS2 nodes — before
touching a real robot or adding any user-facing interaction (voice/UI/orders
come later, out of scope for this phase).

## Decisions locked in
- **End effector:** Robotiq 2F-85 parallel gripper grasps the bottle by the
  neck, arm tilts the wrist to pour. (vs. fixed spout / welded bottle)
- **Motion stack:** MoveIt 2 for planning + `ros2_control` for execution.
  Chosen over hand-rolled joint trajectories for easier extension later
  (pick-and-place of multiple bottles, real-robot reuse, collision checking).
- **Simulator:** Gazebo Sim (Fortress, via `ros_gz`) — already installed on
  this machine (`ign`/`gz`, `libignition-gazebo6*`). Not Gazebo Classic.

## Environment (verified on this machine)
- Ubuntu 22.04, ROS2 Humble, `colcon` present.
- `ros_gz*` present (Gazebo Sim bridge). `gazebo-ros2-control` (classic) is
  also available if we ever need it, but we standardize on `gz_ros2_control`.
- Not yet installed, needed: `ros-humble-moveit`, `ros-humble-ur-description`,
  `ros-humble-ur-simulation-gz`, `ros-humble-ur-moveit-config`,
  `ros-humble-robotiq-description`, `ros-humble-robotiq-controllers`,
  `ros-humble-ros2-control`, `ros-humble-ros2-controllers`,
  `ros-humble-gz-ros2-control`.
- Existing assets: `models/Jack Daniel Bottle.obj` (needs converting into a
  proper Gazebo model: SDF + collision mesh + inertial properties + material).
- Not a git repo yet — initialize once workspace layout is in place.

## Workspace layout
```
bartender_robot/
├── PLAN.md
├── ros2_ws/
│   └── src/
│       ├── bartender_description/   # xacro/URDF: UR5e + Robotiq 2F-85 + bar scene
│       ├── bartender_gazebo/        # world SDF, launch files, spawn scripts
│       ├── bartender_moveit_config/ # generated MoveIt config (moveit_setup_assistant)
│       ├── bartender_bringup/       # top-level launch: sim + control + moveit
│       └── bartender_pour/          # ROS2 nodes: pour skill, glass/bottle TF, pour FSM
├── models/                          # Gazebo/SDF models (bottle, glass, bar counter)
└── code/                            # (existing, currently empty — fold into ros2_ws or repurpose for scripts/notebooks)
```
Recommend consolidating `code/` into the ROS2 workspace above rather than
keeping a separate loose folder, to avoid two sources of truth.

## Phased milestones

### Phase 0 — Environment & scaffolding
- `apt install` the packages listed above.
- `git init`, add `.gitignore` (build/, install/, log/).
- Create `ros2_ws` with the package skeletons above (`ros2 pkg create`).
- Sanity check: launch stock `ur_simulation_gz` demo to confirm UR5e spawns
  and moves in Gazebo Sim on this machine before writing any custom code.

### Phase 1 — Robot + gripper description
- Xacro-compose UR5e (`ur_description`) with Robotiq 2F-85
  (`robotiq_description`) at the tool flange.
- Add `ros2_control` tags + `gz_ros2_control` plugin so both arm joints and
  gripper fingers are controllable in sim.
- Verify: spawn in empty Gazebo world, move joints via
  `joint_trajectory_controller`, open/close gripper via its controller.

### Phase 2 — Bar scene
- Convert `Jack Daniel Bottle.obj` into a simulatable model: decimated
  collision mesh, inertial tags, SDF model with a `<link>`; add a simple
  cylindrical glass model and a bar-counter/table model.
- Build a Gazebo world placing robot, bottle stand, and glass at known,
  fixed poses (deferred: perception — for now poses are hardcoded/known TF
  frames, no vision needed to get first pour working).

### Phase 3 — MoveIt integration
- Generate MoveIt config for the UR5e+gripper combo (arm planning group +
  gripper group), tuned for the sim controllers from Phase 1.
- Verify in RViz: plan/execute arm motions, plan/execute gripper open/close,
  against the live Gazebo sim.

### Phase 4 — Pour skill (the core deliverable)
- New node/package `bartender_pour` implementing a simple state machine:
  1. Move to pre-grasp pose above bottle.
  2. Approach + close gripper (grasp bottle neck).
  3. Retreat, move to pre-pour pose above glass.
  4. Tilt wrist to pour angle, hold for a computed duration (time-based
     "measured pour" proxy — no real fluid sim), return to upright.
  5. Move back to bottle stand, release, return to home.
- Expose as a ROS2 action server (e.g. `PourDrink.action` with target glass
  ID / pour amount) so it's a clean interface for the later
  user-interaction layer to call — but no interaction layer built yet.
- Success criteria: action call reliably completes the pick-tilt-pour-return
  cycle in Gazebo across repeated runs without collisions or drops.

### Phase 5 — Validation & hardening
- Add basic collision checking margins, retry/error states (missed grasp,
  planning failure) reported back through the action's result/feedback.
- Log/replay a few full runs; tune pour angle & timing against a visual
  "how much appears to have poured" check (a full fluid simulation is out
  of scope for this phase).
- Document exact joint/tilt values and timing so the same skill can later
  be re-tuned for the real UR5e without a rewrite.

## Explicitly out of scope for this phase
- Real UR5e hardware bring-up (Phase 2 of the overall project).
- Any user interaction (voice, GUI, order queue, drink menu logic).
- Perception (camera-based bottle/glass detection) — poses are fixed/known.
- Real liquid/fluid simulation — pour is modeled as timed tilt, not volume.

## Progress

- **Phase 0 (partial):** repo initialized, `ros2_ws/src` scaffolded with six
  packages (`bartender_description`, `bartender_gazebo`,
  `bartender_moveit_config`, `bartender_bringup`, `bartender_pour`,
  `bartender_pour_interfaces`), all building cleanly with `colcon build`.
  **Blocked on the user**: the apt install of `ros-humble-moveit`,
  `ur-description`, `ur-simulation-gz`, `ur-moveit-config`,
  `robotiq-description`, `robotiq-controllers`, `ros2-control`,
  `ros2-controllers`, `gz-ros2-control` needs `sudo` and hasn't run yet
  (command is in README.md). The stock `ur_simulation_gz` demo baseline
  check is still pending that install.
- **Phase 1 (drafted, unverified):** `bartender_description/urdf/bartender.urdf.xacro`
  composes `ur_macro.xacro` (UR5e) + `robotiq_2f_85_macro.urdf.xacro`
  (gripper) + `gz_ros2_control` plugin; `config/controllers.yaml` defines
  `joint_state_broadcaster` + `ur_arm_controller`
  (`joint_trajectory_controller`) + `gripper_controller`
  (`GripperActionController`). Written against Humble's documented macro
  APIs but not yet rendered against the real installed packages --
  `scripts/verify_description.sh` checks this once they're installed.
- **Phase 2 (done, verified):** `models/jack_daniels_bottle` (converted from
  the supplied OBJ: background plane stripped, rescaled to a realistic
  24.5cm bottle, recentered at its base, cylinder collision),
  `models/serving_glass`, `models/bar_counter` all exist as SDF models;
  `bar_world.sdf` places them together with the robot spawn point.
  Actually loaded in Gazebo Sim (`ign gazebo -s -r`) on this machine: all
  four models (ground_plane, bar_counter, jack_daniels_bottle,
  serving_glass) spawn with no errors/warnings in the log, and the bottle
  and glass stay put on the counter (pose unchanged) after 2000 physics
  iterations -- collision geometry and counter height are consistent.
- **Phase 3:** not started -- needs Phase 0's install and MoveIt Setup
  Assistant run (see `bartender_moveit_config/README.md`).
- **Phase 4 (drafted, unverified):** `bartender_pour_interfaces/action/PourDrink.action`
  defined; `bartender_pour/pour_action_server.py` implements the
  pick -> tilt -> pour -> return state machine as an action server, driving
  MoveGroup (joint-space goals against placeholder waypoints) and
  GripperCommand. Needs real joint waypoints and a live sim/MoveIt to
  actually exercise.
- **Phase 5:** not started.

## Next step
1. Run the apt install command in README.md (needs sudo; not runnable from
   this session).
2. `colcon build`, then run `scripts/verify_description.sh` and fix any
   macro-API mismatches it reports.
3. Launch `ros2 launch bartender_gazebo sim.launch.py` and confirm the
   composed robot spawns correctly in the bar world.
4. Generate `bartender_moveit_config` via the Setup Assistant.
5. Jog the arm in RViz/MoveIt to capture real values for
   `WAYPOINTS_RAD` in `pour_action_server.py`, then run the full
   `bartender_bringup` launch and call the `pour_drink` action.
