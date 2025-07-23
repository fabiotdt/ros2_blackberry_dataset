import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

import tf2_ros
import time
from move_to_pose_srv.srv import MoveToPose


class TfToPosePublisher(Node):

    def __init__(self):
        super().__init__("tf_to_pose_publisher")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.has_published = False

        # Create a client to the MoveToPose service
        self.client = self.create_client(MoveToPose, "/move_to_pose")

    def timer_callback(self):
        if self.has_published:
            return

        try:
            # Lookup transform from base_link to blackberry
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "blackberry",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            # Extract translation and rotation
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Apply small displacement (e.g., 0.1m in x)
            displaced_pose = PoseStamped()
            displaced_pose.header.stamp = self.get_clock().now().to_msg()
            displaced_pose.header.frame_id = "base_link"

            displaced_pose.pose.position.x = trans.x + 0.05  # displacement
            displaced_pose.pose.position.y = trans.y
            displaced_pose.pose.position.z = trans.z - 0.2  # displacement
            displaced_pose.pose.orientation = rot  # keep original orientation

            req = MoveToPose.Request()
            req.pose = displaced_pose
            self.client.call_async(req)

            self.get_logger().info(
                "Call srv with pose: "
                + str(displaced_pose.pose.position)
                + " with orientation: "
                + str(displaced_pose.pose.orientation)
            )
            self.has_published = True
            rclpy.shutdown()
            exit(0)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TfToPosePublisher()
    while not node.client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for MoveToPose service...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
