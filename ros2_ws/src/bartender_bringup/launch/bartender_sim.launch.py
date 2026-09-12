"""Full Phase 1 stack: Gazebo Sim + bar scene + robot spawn, ros2_control
controller spawners, MoveIt move_group, and the pour_drink action server.

MoveIt integration (the `bartender_moveit_config` include below) only works
once that package has been generated with the MoveIt Setup Assistant -- see
README.md "Generating the MoveIt config". Until then, run just the
controller-spawning half of this file for arm/gripper sim testing.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bartender_gazebo')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')
        )
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    ur_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    # Give Gazebo + the spawned robot's controller_manager time to come up
    # before asking it to spawn controllers, rather than racing it.
    delayed_controller_spawners = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            ur_arm_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    pour_action_server = Node(
        package='bartender_pour',
        executable='pour_action_server',
        output='screen',
    )

    return LaunchDescription([
        sim,
        delayed_controller_spawners,
        pour_action_server,
        # TODO once bartender_moveit_config exists (see README):
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #     os.path.join(get_package_share_directory('bartender_moveit_config'),
        #                  'launch', 'move_group.launch.py'))),
    ])
