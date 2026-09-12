"""PourDrink action server: a fixed pick -> tilt -> pour -> return state
machine for Phase 1 (one known bottle, one known glass, no perception).

Motion is joint-space waypoints sent through MoveIt's /move_action
(moveit_msgs/action/MoveGroup) rather than derived Cartesian goals, since
bottle/glass poses are fixed and known -- this keeps the first working pour
simple. Swapping in pose-based goals (from TF/perception) later only
touches _move_to_named_waypoint / _move_to_pose, not the state machine.

Nested action calls (this server calling the MoveGroup and GripperCommand
action clients) require a MultiThreadedExecutor with a ReentrantCallbackGroup
-- see main().
"""
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions

from bartender_pour_interfaces.action import PourDrink

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

ARM_GROUP_NAME = 'ur_manipulator'      # TODO: confirm against bartender_moveit_config's SRDF group name
GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'  # TODO: confirm against installed robotiq_description

# TODO: these are placeholders. Once the sim is running, jog the arm to each
# pose in RViz/MoveIt (with the gripper positioned over the real bottle/glass
# locations from bar_world.sdf) and replace with the recorded joint values
# from `ros2 topic echo /joint_states`.
WAYPOINTS_RAD = {
    'home':          [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
    'above_bottle':  [0.0, -1.20, -1.00, -1.50, -1.57, 0.0],
    'grasp_bottle':  [0.0, -1.10, -1.10, -1.50, -1.57, 0.0],
    'above_glass':   [0.60, -1.20, -1.00, -1.50, -1.57, 0.0],
    'pour_tilt':     [0.60, -1.20, -1.00, -1.00, -1.57, 1.20],
}

GRIPPER_OPEN_POS = 0.0    # meters
GRIPPER_CLOSED_POS = 0.04  # meters, TODO: tune to actually clamp the bottle neck
GRIPPER_MAX_EFFORT = 20.0

POUR_DURATION_S = 2.0
ASSUMED_ML_PER_SECOND = 25.0  # rough proxy for "amount poured", no fluid sim

JOINT_TOLERANCE = 0.01
PLANNING_TIME_S = 5.0


class PourActionServer(Node):

    def __init__(self):
        super().__init__('pour_action_server')
        cb_group = ReentrantCallbackGroup()

        self._move_group_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=cb_group)
        self._gripper_client = ActionClient(
            self, GripperCommand, 'gripper_controller/gripper_cmd', callback_group=cb_group)

        self._action_server = ActionServer(
            self, PourDrink, 'pour_drink',
            execute_callback=self.execute_callback,
            callback_group=cb_group,
        )
        self.get_logger().info('pour_action_server ready')

    # ---- motion helpers -------------------------------------------------

    def _move_to_named_waypoint(self, name: str) -> bool:
        positions = WAYPOINTS_RAD[name]
        joint_constraints = [
            JointConstraint(
                joint_name=joint, position=pos,
                tolerance_above=JOINT_TOLERANCE, tolerance_below=JOINT_TOLERANCE,
                weight=1.0,
            )
            for joint, pos in zip(ARM_JOINTS, positions)
        ]

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest(
            group_name=ARM_GROUP_NAME,
            goal_constraints=[Constraints(joint_constraints=joint_constraints)],
            allowed_planning_time=PLANNING_TIME_S,
            num_planning_attempts=5,
        )
        goal.planning_options = PlanningOptions(plan_only=False)

        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_action server not available')
            return False

        send_future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'MoveGroup goal to "{name}" rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error(f'MoveGroup goal to "{name}" timed out')
            return False

        error_code = result.result.error_code.val
        if error_code != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            self.get_logger().error(f'MoveGroup goal to "{name}" failed, error_code={error_code}')
            return False
        return True

    def _command_gripper(self, position: float) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('gripper action server not available')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        send_future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        return result_future.result() is not None

    # ---- the pour state machine ------------------------------------------

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        feedback = PourDrink.Feedback()

        def step(state: str, progress: float, fn, *args) -> bool:
            feedback.state = state
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'pour_drink: {state}')
            return fn(*args)

        steps = [
            ('opening_gripper', 0.05, self._command_gripper, GRIPPER_OPEN_POS),
            ('approaching_bottle', 0.15, self._move_to_named_waypoint, 'above_bottle'),
            ('grasping_bottle', 0.30, self._move_to_named_waypoint, 'grasp_bottle'),
            ('closing_gripper', 0.40, self._command_gripper, GRIPPER_CLOSED_POS),
            ('lifting_bottle', 0.50, self._move_to_named_waypoint, 'above_bottle'),
            ('moving_to_glass', 0.65, self._move_to_named_waypoint, 'above_glass'),
        ]

        for state, progress, fn, arg in steps:
            if not step(state, progress, fn, arg):
                goal_handle.abort()
                return self._fail_result(f'failed at step: {state}')

        # pour
        feedback.state = 'pouring'
        feedback.progress = 0.75
        goal_handle.publish_feedback(feedback)
        if not self._move_to_named_waypoint('pour_tilt'):
            goal_handle.abort()
            return self._fail_result('failed at step: pouring (tilt)')

        pour_time = min(POUR_DURATION_S, max(0.1, request.pour_amount_ml / ASSUMED_ML_PER_SECOND))
        time.sleep(pour_time)
        estimated_ml = pour_time * ASSUMED_ML_PER_SECOND

        return_steps = [
            ('returning_upright', 0.85, self._move_to_named_waypoint, 'above_glass'),
            ('returning_bottle', 0.92, self._move_to_named_waypoint, 'above_bottle'),
            ('releasing_bottle', 0.96, self._command_gripper, GRIPPER_OPEN_POS),
            ('returning_home', 0.99, self._move_to_named_waypoint, 'home'),
        ]
        for state, progress, fn, arg in return_steps:
            if not step(state, progress, fn, arg):
                goal_handle.abort()
                return self._fail_result(f'failed at step: {state}')

        goal_handle.succeed()
        result = PourDrink.Result()
        result.success = True
        result.message = 'pour complete'
        result.estimated_poured_ml = estimated_ml
        return result

    @staticmethod
    def _fail_result(message: str) -> PourDrink.Result:
        result = PourDrink.Result()
        result.success = False
        result.message = message
        result.estimated_poured_ml = 0.0
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PourActionServer()
    # >=2 threads: one to run the action server callback, one free to
    # service the nested MoveGroup/GripperCommand client spins it triggers.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
