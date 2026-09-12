# Bartender Robot

UR5e + Robotiq 2F-85 bartender, taught to pour a drink in Gazebo Sim
simulation before anything touches a real robot or a user-facing interface.
See [PLAN.md](PLAN.md) for the full phased plan; this file covers setup and
day-to-day commands.

**Status: the full pick -> tilt -> pour -> return cycle works end-to-end in
simulation**, driven by calling the `pour_drink` action. What's still
placeholder: the arm's waypoint joint values aren't tuned to the bottle's
real position yet (see "Status" below), so the gripper closes near the
bottle but doesn't actually grasp/lift it.

## One-time setup

```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-ur-description \
  ros-humble-ur-simulation-gz \
  ros-humble-ur-moveit-config \
  ros-humble-robotiq-description \
  ros-humble-robotiq-controllers \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`scripts/verify_description.sh` checks the xacro macro names/params this
project assumes against what's actually installed -- useful if you update
any of the above packages later and something stops rendering.

## Package layout

- `bartender_description` -- xacro composing UR5e + Robotiq 2F-85 +
  ros2_control/gz_ros2_control wiring.
- `bartender_gazebo` -- the bar world (SDF) and sim launch/spawn.
- `bartender_moveit_config` -- hand-written MoveIt config (see its own
  README.md for why, and for a known RViz-only display bug).
- `bartender_bringup` -- top-level launch tying sim + control + MoveIt +
  the pour skill together.
- `bartender_pour_interfaces` -- the `PourDrink` action definition.
- `bartender_pour` -- the pour skill's action server (pick -> tilt -> pour
  -> return state machine).
- `models/` -- Gazebo models: `jack_daniels_bottle`, `serving_glass`,
  `bar_counter`.

## Running the sim

```bash
# scene + robot spawn only (no controllers/MoveIt/pour skill):
ros2 launch bartender_gazebo sim.launch.py

# full stack: sim + controllers + move_group + pour_drink action server
ros2 launch bartender_bringup bartender_sim.launch.py
```

Once the full stack is up (give it ~30s -- Gazebo, controller spawners, and
move_group all start with deliberate delays so they don't race each other),
trigger a pour:

```bash
ros2 action send_goal /pour_drink bartender_pour_interfaces/action/PourDrink \
  "{bottle_id: jack_daniels_bottle, glass_id: serving_glass, pour_amount_ml: 45.0}" \
  --feedback
```

If a `ros2` CLI command run immediately after a fresh `colcon build`
reports "The passed action type is invalid" or otherwise misbehaves, it's a
transient ament-index/discovery glitch, not a real error -- just retry it.

## Status / what's still placeholder

- **Verified working end-to-end**: full robot (UR5e + gripper) spawns in
  the bar world; all 3 controllers come up active; `move_group` plans and
  executes joint-space goals; the `pour_drink` action runs its whole
  pick -> tilt -> pour -> return cycle and returns `success: true`.
- `bartender_pour/pour_action_server.py`'s `WAYPOINTS_RAD` joint values are
  still placeholders, not tuned to the bottle/glass's actual poses in
  `bar_world.sdf` -- the bottle's pose is confirmed unchanged after a pour
  (gripper closes near it but doesn't actually contact/lift it). Jog the
  arm in RViz/MoveIt against the running sim to capture real values, and
  keep the SRDF's `home` group_state in sync (see
  `bartender_moveit_config/README.md`).
- No perception, no user interaction, no real fluid simulation -- see
  PLAN.md "Explicitly out of scope for this phase".
