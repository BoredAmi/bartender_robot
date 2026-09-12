"""Start MoveIt's move_group node (+ optional RViz) for the bartender
UR5e + Robotiq 2F-85, against the controllers spawned by
bartender_bringup/launch/bartender_sim.launch.py.

Hand-written instead of MoveIt-Setup-Assistant-generated -- see
bartender_moveit_config/srdf/bartender.srdf for why -- but follows the same
structure/parameter set the Assistant's generated launch file would use.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_share_dir, relative_path):
    with open(os.path.join(package_share_dir, relative_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_description = get_package_share_directory('bartender_description')
    pkg_moveit_config = get_package_share_directory('bartender_moveit_config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
    ]

    xacro_file = os.path.join(pkg_description, 'urdf', 'bartender.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_file, ' sim_ignition:=true']),
            value_type=str,
        )
    }

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            open(os.path.join(pkg_moveit_config, 'srdf', 'bartender.srdf')).read(),
            value_type=str,
        )
    }

    # Passed as a file path (not parsed into a dict): ROS2's "/**:
    # ros__parameters:" wildcard YAML syntax is only resolved when a node
    # parameter is given a file path, not when pre-parsed into a plain dict.
    robot_description_kinematics = os.path.join(pkg_moveit_config, 'config', 'kinematics.yaml')
    robot_description_planning = {
        'robot_description_planning': load_yaml(pkg_moveit_config, os.path.join('config', 'joint_limits.yaml'))
    }

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(
        load_yaml(pkg_moveit_config, os.path.join('config', 'ompl_planning.yaml'))
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml(
            pkg_moveit_config, os.path.join('config', 'moveit_controllers.yaml')
        )['moveit_simple_controller_manager'],
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(launch_rviz),
        arguments=['-d', os.path.join(pkg_moveit_config, 'rviz', 'view_robot.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
