"""Launch Gazebo Sim (Fortress) with the bar world, spawn the bartender
robot from bartender_description's xacro, and bridge /clock to ROS2.

This is Phase 2's world/scene launch. It does NOT start ros2_control
controllers or MoveIt -- see bartender_bringup for the full stack.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bartender_gazebo')
    pkg_description = get_package_share_directory('bartender_description')

    # bartender_robot/models sits one level above the ros2_ws source tree;
    # add it to GZ_SIM_RESOURCE_PATH so `model://jack_daniels_bottle` etc.
    # resolve when Gazebo loads the world.
    models_path = os.path.normpath(os.path.join(pkg_gazebo, '..', '..', '..', '..', '..', 'models'))
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = models_path if not existing_resource_path else \
        existing_resource_path + os.pathsep + models_path

    world_path = os.path.join(pkg_gazebo, 'worlds', 'bar_world.sdf')

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen',
    )

    xacro_file = os.path.join(pkg_description, 'urdf', 'bartender.urdf.xacro')
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file, ' sim_gazebo:=true']),
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
            '-x', '-0.4', '-y', '0.0', '-z', '0.9',
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,
    ])
