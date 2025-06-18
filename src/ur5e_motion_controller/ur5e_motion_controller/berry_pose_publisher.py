from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import rclpy

class BerryPosePublisher(Node):
    def __init__(self):
        super().__init__('berry_pose_publisher')
        self.pose_pub = self.create_publisher(PoseStamped, '/berry_pose', 10)
        
        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically publish the pose
        self.timer = self.create_timer(0.001, self.publish_berry_pose)

    def publish_berry_pose(self):
        try:
            # Get the transform from 'base_link' to 'berry'
            transform = self.tf_buffer.lookup_transform('base_link', 'blackberry', rclpy.time.Time())
            
            # Create a PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            
            pose_msg.pose.orientation = transform.transform.rotation
            
            # Publish the pose
            self.pose_pub.publish(pose_msg)
        except TransformException as e:
            self.get_logger().warn(f"Could not transform 'blackberry' to 'base_link': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BerryPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
