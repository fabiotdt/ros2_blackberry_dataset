
# berry_dataset/berry_dataset/dataset_saver.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Float64MultiArray, String
import re


from .berry_dataset import BerryDataset
from .utils import CreatePointCloud

class BerrySaver(Node):
    def __init__(self):
        super().__init__('berry_saver')
        self.bridge = CvBridge()
        self.dataset = BerryDataset('../data')
        
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        #self.depin_sub = self.create_subscription(String, '/camera/depth_intrin', self.depin_callback, 10)

        self.depin_sub = self.create_subscription(CameraInfo, '/camera/depth_intrin', self.depin_callback, 10)

        self.color_image = None
        self.depth_image = None
        self.depin_intrin = None  # intrinsinc parameters as string

        self.arm_T = None
        self.trigger = False

        self.trigger_sub = self.create_subscription(Bool, '/ur_trigger', self.trigger_callback, 10)
        self.arm_T_sub = self.create_subscription(Float64MultiArray, '/ur_arm_T_matrix', self.arm_matrix_callback, 10)

        #self.timer = self.create_timer(2.0, self.save_data)

    def transform_frame(self, T):
        # TODO: transform reference frame: from arm base frame to camera frame
        pass

    def color_callback(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    
    def depin_callback(self, intrin_msg: CameraInfo):
        self.depin_intrin = intrin_msg

    def trigger_callback(self, msg):
        self.get_logger().info(f"Received trigger: {msg.data}")

        self.trigger = msg.data
        if self.trigger:
            self.save_data()

    def arm_matrix_callback(self, msg):
        self.arm_T = msg.data 

    def save_data(self):
        if self.color_image is not None and self.depth_image is not None and self.depin_intrin is not None:

            pcd = CreatePointCloud.from_images(self.color_image, self.depth_image, self.depin_intrin)

            self.dataset.save_image(self.color_image)
            self.dataset.save_depth_image(self.depth_image)
            self.dataset.save_pointcloud(pcd)

            arm_T_str = str(self.arm_T)
            self.dataset.save_data(arm_T_str, arm_T_str, self.depin_intrin)
            self.get_logger().info(f"Saved berry dataset ID {self.dataset.idx}")
            self.dataset.idx += 1

        else:
            self.get_logger().warn("Waiting for color, depth, and camera info messages.")


def main(args=None):
    rclpy.init(args=args)
    node = BerrySaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()