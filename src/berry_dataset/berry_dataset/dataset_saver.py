import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

from std_srvs.srv import Trigger  # Trigger service
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from .berry_dataset import BerryDataset
from .utils import CreatePointCloud


class BerrySaver(Node):
    def __init__(self):
        super().__init__('berry_saver')
        self.bridge = CvBridge()
        self.dataset = BerryDataset('/media/fabio_tdt/f68d1f51-42bc-4a54-a541-aa1050e013b1/home/user/berry_dataset')

        # Subscriptions
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.depin_sub = self.create_subscription(CameraInfo, '/camera/depth_intrin', self.depin_callback, 10)
        self.idx_sub = self.create_subscription(Int32, '/pose_index', self.idx_callback, 10)

        # Service
        self.trigger_srv = self.create_service(Trigger, 'save_berry_data', self.handle_trigger)

        # Data holders
        self.color_image = None
        self.depth_image = None
        self.depin_intrin = None
        
        # Berry pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Transformation: arm to camera
        self.R_ac = np.array([[1, 0, 0],
                              [0, 0, 1],
                              [0, 1, 0]])
        self.p_ac = np.array([100, 200, 300])  # in mm
        self.T_ac = np.eye(4)
        self.T_ac[:3, :3] = self.R_ac
        self.T_ac[:3, 3] = self.p_ac
        self.get_logger().info("BerrySaver node initialized.")

    def color_callback(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def depin_callback(self, intrin_msg: CameraInfo):
        self.depin_intrin = intrin_msg

    def idx_callback(self, msg):
        self.dataset.idx = msg.data
        self.get_logger().info(f"Dataset index updated to {self.dataset.idx}")

    def current_pose_callback(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ori = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot = R.from_quat(ori).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        self.arm_T = T

    def rototranslate(self, T_ba):
        T_bc = self.T_ac.dot(T_ba)
        return T_bc

    def handle_trigger(self, request, response):
        if self.color_image is None or self.depth_image is None or self.depin_intrin is None:
            response.success = False
            response.message = "Missing image or camera info data."
            self.get_logger().warn(response.message)
            return response
        transform_cam1_bb = None
        transform_base_bb = None
        # if self.arm_T is None:
        while(transform_cam1_bb is None and transform_base_bb is None):
            try:
                # Get the transform from 'cam1' to 'blackberry'
                transform_cam1_bb = self.tf_buffer.lookup_transform('cam1', 'blackberry', rclpy.time.Time())
                transform_base_bb = self.tf_buffer.lookup_transform('base_link', 'blackberry', rclpy.time.Time())
                # Publish the pose
            except TransformException as e:
                self.get_logger().warn(f"Could not transform 'blackberry' to 'cam1' or 'base_link': {e}")

        try:
            pcd = CreatePointCloud.from_images(self.color_image, self.depth_image, self.depin_intrin)

            self.dataset.save_image(self.color_image)
            self.dataset.save_depth_image(self.depth_image)
            self.dataset.save_pointcloud(pcd)

            # T_ba = np.array(self.arm_T).reshape(4, 4)
            # arm_T_str = str(T_ba)

            # convert to matrix 4x4
            T_cam1_bb = np.eye(4)
            T_cam1_bb[:3, :3] = R.from_quat([
                transform_cam1_bb.transform.rotation.x,
                transform_cam1_bb.transform.rotation.y,
                transform_cam1_bb.transform.rotation.z,
                transform_cam1_bb.transform.rotation.w
            ]).as_matrix()
            T_cam1_bb[:3, 3] = [
                transform_cam1_bb.transform.translation.x,
                transform_cam1_bb.transform.translation.y,
                transform_cam1_bb.transform.translation.z
            ]

            T_base_bb = np.eye(4)
            T_base_bb[:3, :3] = R.from_quat([
                transform_base_bb.transform.rotation.x,
                transform_base_bb.transform.rotation.y,
                transform_base_bb.transform.rotation.z,
                transform_base_bb.transform.rotation.w
            ]).as_matrix()
            T_base_bb[:3, 3] = [
                transform_base_bb.transform.translation.x,
                transform_base_bb.transform.translation.y,
                transform_base_bb.transform.translation.z
            ]
            self.get_logger().info(f"Transform from 'base_link' to 'blackberry': {T_base_bb}")
            self.get_logger().info(f"Transform from 'cam1' to 'blackberry': {T_cam1_bb}")

            self.dataset.save_data(
                T_base_bb.tolist(),
                T_cam1_bb.tolist(),
                self.depin_intrin.k
            )

            self.get_logger().info(f"Saved berry dataset ID {self.dataset.idx}")
            self.dataset.idx += 1

            response.success = True
            response.message = f"Data saved with index {self.dataset.idx - 1}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to save data: {str(e)}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = BerrySaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
