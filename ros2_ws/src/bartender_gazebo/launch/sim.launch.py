"""Launch Gazebo Sim with the bar scene and spawn the robot into it.

Loads the bar world (Fortress), spawns the two-armed bartender from
bartender_description's xacro, and bridges /clock plus the topics the
beer-opening sequence reads and writes.

This is Phase 2's world/scene launch. It does NOT start ros2_control
controllers or MoveIt -- see bartender_bringup for the full stack.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description=('Run Gazebo server only, no GUI (avoids the Ignition Qt '
                     'GUI, useful when scripting/tuning against the sim).'),
    )
    pkg_gazebo = get_package_share_directory('bartender_gazebo')
    pkg_description = get_package_share_directory('bartender_description')

    # bartender_robot/models sits one level above the ros2_ws source tree;
    # add it to GZ_SIM_RESOURCE_PATH so `model://jack_daniels_bottle` etc.
    # resolve when Gazebo loads the world.
    models_path = os.path.normpath(
        os.path.join(pkg_gazebo, '..', '..', '..', '..', '..', 'models'))
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = models_path if not existing_resource_path else \
        existing_resource_path + os.pathsep + models_path

    world_path = os.path.join(pkg_gazebo, 'worlds', 'bar_world.sdf')

    # ROS2 Humble's setup.bash does not add /opt/ros/humble/lib to Gazebo's
    # own plugin search path, so gz_ros2_control-system (referenced by
    # bartender.urdf.xacro's <gazebo><plugin> tag) fails to load with
    # "couldn't find shared library" unless this is set explicitly.
    ros_lib_path = '/opt/ros/humble/lib'
    existing_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = ros_lib_path if not existing_plugin_path else \
        existing_plugin_path + os.pathsep + ros_lib_path

    gz_sim_env = {
        'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': gz_plugin_path,
    }
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        additional_env=gz_sim_env,
        output='screen',
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world_path],
        additional_env=gz_sim_env,
        output='screen',
        condition=IfCondition(headless),
    )

    # Rendered through render_bartender_urdf.py rather than xacro directly:
    # it runs xacro and then cuts the cylinder-holding groove into the gripper
    # pads, which has to replace the stock fingertip collision and so cannot be
    # done in xacro alone. move_group.launch.py renders the same way, so the
    # planner and the physics engine agree on the gripper's shape.
    xacro_file = os.path.join(pkg_description, 'urdf', 'bartender.urdf.xacro')
    render_script = os.path.join(pkg_description, 'scripts', 'render_bartender_urdf.py')
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='python3'), ' ', render_script, ' ',
                 xacro_file, ' sim_ignition:=true']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bartender_ur5e',
            # Arm A's base, and therefore the origin of base_link, the
            # frame bartender_pour plans everything in. On the bar top
            # (z=0.9), at one end of the bar and 0.40 off its centreline.
            # Arm B stands 1.06 down the bar and 0.80 across, turned to
            # face back at arm A; the description states that relative to
            # this point. See the bar layout block in worlds/bar_world.sdf.
            '-x', '-0.45', '-y', '-0.40', '-z', '0.9',
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Two more bridges, both for the beer-opening sequence. Separate node
    # from the clock bridge only because that one is Phase 2 infrastructure
    # and these are not; there is no technical reason they could not share.
    #
    # dynamic_pose/info is SceneBroadcaster's running report of where every
    # NON-STATIC model in the world actually is. It is bridged under its own
    # name rather than onto /tf on purpose: it arrives at 60Hz and carries
    # every bottle, glass and cap in the scene, and dumping that into /tf
    # would bury the robot's own transforms. bartender_open subscribes to it
    # directly, and it is how that node knows whether the beer moved while it
    # was being pushed on -- which is the whole test for "the other arm is
    # really holding it".
    #
    # The detach topic goes the other way, ROS to Gazebo (`]`), and is what
    # actually releases the cap. See make_beer_and_opener.py's docstring for
    # why the release is commanded rather than pried.
    beer_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/bar_world/dynamic_pose/info'
            '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/beer/cap/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
        ],
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/bartender/arm_a/wrist_camera/image_raw'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/bartender/arm_b/wrist_camera/image_raw'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        output='screen',
    )

    return LaunchDescription([
        headless_arg,
        gz_sim,
        gz_sim_headless,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,
        beer_bridge,
        camera_bridge,
    ])
