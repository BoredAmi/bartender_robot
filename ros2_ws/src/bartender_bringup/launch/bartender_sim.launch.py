"""Full Phase 1 stack: Gazebo Sim + bar scene + robot spawn, ros2_control
controller spawners, MoveIt move_group, and the pour_drink action server.
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

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bartender_moveit_config'),
                         'launch', 'move_group.launch.py')
        ),
        # RViz's MotionPlanning display currently throws a kinematics
        # parameter type error on startup here (rviz-only; move_group
        # itself loads the same kinematics.yaml fine) -- off by default
        # until that's tracked down. Pass launch_rviz:=true to re-enable.
        launch_arguments={'launch_rviz': 'false'}.items(),
    )
    # move_group needs the ur_arm_controller/gripper_controller action
    # servers to exist before it will accept execution requests.
    delayed_move_group = TimerAction(period=8.0, actions=[move_group])

    return LaunchDescription([
        sim,
        delayed_controller_spawners,
        pour_action_server,
        delayed_move_group,
    ])
