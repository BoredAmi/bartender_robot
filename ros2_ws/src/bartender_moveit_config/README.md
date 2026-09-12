# bartender_moveit_config

Placeholder package. This needs to be generated with the MoveIt Setup
Assistant, not hand-written -- the SRDF (planning groups, self-collision
disable pairs, virtual joints) is exactly the kind of thing that's error-prone
to author by hand and the Assistant computes/validates it interactively.

## How to generate it (after Phase 0 apt install + workspace build)

```bash
source /opt/ros/humble/setup.bash
source ~/personal_projects/bartender_robot/ros2_ws/install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

1. "Create New MoveIt Configuration Package" -> select
   `bartender_description/urdf/bartender.urdf.xacro`.
2. Self-Collisions: generate the default collision matrix (sampling).
3. Planning Groups: add group `ur_manipulator` with the 6 UR joints
   (kinematic chain base_link -> tool0, solver: KDL or the UR-specific IK
   plugin if available); add group `gripper` with the gripper joint.
4. Robot Poses: define at least `home` matching
   `bartender_pour/pour_action_server.py`'s `WAYPOINTS_RAD['home']`.
5. ROS2 Controllers: point at
   `bartender_description/config/controllers.yaml` (arm: `ur_arm_controller`,
   gripper: `gripper_controller`).
6. Generate into this directory (`bartender_moveit_config`), overwriting the
   package.xml/CMakeLists placeholders here.

Once generated, uncomment the `bartender_moveit_config` include in
`bartender_bringup/launch/bartender_sim.launch.py`, and confirm
`ARM_GROUP_NAME` in `pour_action_server.py` matches the group name you chose
in step 3.
