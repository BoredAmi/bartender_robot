# Bartender Robot

UR5e + Robotiq 2F-85 bartender, taught to pour a drink in Gazebo Sim
simulation before anything touches a real robot or a user-facing interface.
See [PLAN.md](PLAN.md) for the full phased plan; this file covers setup and
day-to-day commands.

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

Then run `scripts/verify_description.sh` once -- it checks the xacro macro
names/params this project assumes against what's actually installed, and
renders+validates the full URDF. Fix anything it flags before moving on;
xacro macro APIs occasionally drift between package releases.

MoveIt config still needs to be generated interactively -- see
`ros2_ws/src/bartender_moveit_config/README.md`.

## Package layout

- `bartender_description` -- xacro composing UR5e + Robotiq 2F-85 +
  ros2_control/gz_ros2_control wiring.
- `bartender_gazebo` -- the bar world (SDF) and sim launch/spawn.
- `bartender_moveit_config` -- MoveIt config (generate via Setup Assistant).
- `bartender_bringup` -- top-level launch tying sim + control + MoveIt +
  the pour skill together.
- `bartender_pour_interfaces` -- the `PourDrink` action definition.
- `bartender_pour` -- the pour skill's action server (pick -> tilt -> pour
  -> return state machine).
- `models/` -- Gazebo models: `jack_daniels_bottle`, `serving_glass`,
  `bar_counter`.

## Running the sim

```bash
# scene + robot spawn only (no controllers/MoveIt yet):
ros2 launch bartender_gazebo sim.launch.py

# full stack once controllers.yaml joint names are verified:
ros2 launch bartender_bringup bartender_sim.launch.py
```

## Status / what's still placeholder

- `bartender_description/urdf/bartender.urdf.xacro` and
  `config/controllers.yaml` are written against the ur_description /
  robotiq_description macro APIs as documented for ROS2 Humble, but have not
  been rendered/tested yet (those packages weren't installed on this
  machine when this was written). Run `scripts/verify_description.sh` first.
- `bartender_pour/pour_action_server.py`'s `WAYPOINTS_RAD` joint values are
  placeholders -- capture real ones by jogging the arm in RViz/MoveIt once
  the sim + MoveIt are running.
- `bartender_moveit_config` is an empty skeleton pending Setup Assistant
  generation.
- No perception, no user interaction, no real fluid simulation -- see
  PLAN.md "Explicitly out of scope for this phase".
