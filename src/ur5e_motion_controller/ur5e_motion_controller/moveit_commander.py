#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger  # optional, for debug
from example_interfaces.srv import SetBool  # not used, just example

from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_commander.robot_trajectory import RobotTrajectory

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveItErrorCodes

from std_srvs.srv import Empty
from your_package_name.srv import MoveToPose  # You will define this service


class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__("move_to_pose_node")
        rclpy.get_default_context().on_shutdown(self.shutdown)

        # Initialize MoveIt interfaces
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "ur_manipulator"  # Change if needed
        self.move_group = MoveGroupCommander(self.group_name)

        # Optional: Set planning params
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)

        # Create service
        self.srv = self.create_service(
            MoveToPose, "move_to_pose", self.move_to_pose_callback
        )

        self.get_logger().info("MoveToPoseNode ready.")

    def move_to_pose_callback(self, request, response):
        target_pose: PoseStamped = request.target_pose
        self.get_logger().info(f"Received target pose:\n{target_pose}")

        # Set the target pose
        self.move_group.set_pose_target(target_pose)

        # Plan
        success, plan, planning_time, error_code = self.move_group.plan()
        if not success or error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Planning failed")
            response.success = False
            response.message = "Planning failed"
            return response

        self.get_logger().info("Plan successful, executing...")

        # Execute
        execution_result = self.move_group.execute(plan, wait=True)

        if not execution_result:
            self.get_logger().error("Execution failed")
            response.success = False
            response.message = "Execution failed"
        else:
            self.get_logger().info("Motion executed successfully")
            response.success = True
            response.message = "Success"

        self.move_group.clear_pose_targets()
        return response

    def shutdown(self):
        self.get_logger().info("Shutting down MoveToPoseNode")
        self.move_group.stop()
        self.move_group.clear_pose_targets()


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
