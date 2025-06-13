import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial.transform import Rotation as R

POSITION_TOLERANCE = 0.001  # 1 cm
ORIENTATION_TOLERANCE = 0.05  # Acceptable angular difference in radians


class ArmStatePublisher(Node):

    def __init__(self):
        super().__init__('arm_state_publisher')

        # Target pose (set externally)
        self.target_pose = None

        # Subscribe to current pose
        self.current_pose_sub = self.create_subscription(
            PoseStamped,
            '/arm_current_pose',
            self.current_pose_callback,
            10
        )

        # Subscribe to final goal pose from MotionDataLoop
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/arm_target_goal',
            self.target_pose_callback,
            10
        )

        # Publisher for state (reached or not)
        self.state_pub = self.create_publisher(Bool, '/arm_state_reached', 10)

    def target_pose_callback(self, msg: PoseStamped):
        self.target_pose = msg

    def current_pose_callback(self, msg: PoseStamped):
        if self.target_pose is None:
            return

        if self.is_pose_reached(msg, self.target_pose):
            reached_msg = Bool()
            reached_msg.data = True
            self.state_pub.publish(reached_msg)
            self.get_logger().info("Target pose reached.")
        else:
            reached_msg = Bool()
            reached_msg.data = False
            self.state_pub.publish(reached_msg)

    def is_pose_reached(self, current: PoseStamped, target: PoseStamped):
        # Position check
        c_pos = np.array([current.pose.position.x, current.pose.position.y, current.pose.position.z])
        t_pos = np.array([target.pose.position.x, target.pose.position.y, target.pose.position.z])
        pos_dist = np.linalg.norm(c_pos - t_pos)

        # Orientation check
        c_quat = np.array([
            current.pose.orientation.x,
            current.pose.orientation.y,
            current.pose.orientation.z,
            current.pose.orientation.w
        ])
        t_quat = np.array([
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w
        ])

        c_rot = R.from_quat(c_quat)
        t_rot = R.from_quat(t_quat)

        angle_diff = c_rot.inv() * t_rot
        ang_dist = angle_diff.magnitude()

        
        return pos_dist < POSITION_TOLERANCE and ang_dist < ORIENTATION_TOLERANCE


def main(args=None):
    rclpy.init(args=args)
    node = ArmStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
