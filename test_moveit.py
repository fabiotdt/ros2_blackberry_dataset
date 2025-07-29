import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from move_to_pose_srv.srv import MoveToPose
import random
import numpy as np
from scipy.spatial.transform import Rotation


class RandomPoseSender(Node):

    def __init__(self):
        super().__init__("random_pose_sender")

        self.client = self.create_client(MoveToPose, "/move_to_pose")
        self.has_sent_request = False
        self.timer = self.create_timer(0.1, self.timer_callback)

    def generate_random_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"

        # Random position within reachable workspace
        # Pos low -0.49403; 0.14853; 1.0647
        # Pos up 0.55591; 0.37717; 1.3693
        # Rot up 90 0 0
        # Rot low 270 -150 150
        pose.pose.position.x = random.uniform(-0.49403, 0.55591)
        pose.pose.position.y = random.uniform(0.14853, 0.37717)
        pose.pose.position.z = random.uniform(1.0647, 1.3693)

        # Random orientation (Euler to Quaternion using scipy)
        euler_x = random.uniform(90, 270)  # Roll
        euler_y = random.uniform(-150, 0)  # Pitch
        euler_z = random.uniform(0, 150)
        quat = Rotation.from_euler(
            "xyz", [euler_x, euler_y, euler_z], degrees=True
        ).as_quat()

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    def timer_callback(self):
        if self.has_sent_request:
            return

        if not self.client.service_is_ready():
            self.get_logger().info("Waiting for /move_to_pose service...")
            return

        random_pose = self.generate_random_pose()
        request = MoveToPose.Request()
        request.pose = random_pose

        self.get_logger().info(
            f"Sending random pose to robot:\n"
            f"Position: {random_pose.pose.position}\n"
            f"Orientation: {random_pose.pose.orientation}"
        )

        future = self.client.call_async(request)
        self.has_sent_request = True

        def response_callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info(
                        "Robot successfully reached the random pose."
                    )
                else:
                    self.get_logger().error("Robot failed to reach the pose.")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
            rclpy.shutdown()

        future.add_done_callback(response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = RandomPoseSender()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
