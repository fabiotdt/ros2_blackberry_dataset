
# berry_dataset/berry_dataset/dataset_saver.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Float64MultiArray, String
import re
import numpy as np
from geometry_msgs.msg import PoseStamped
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R

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
        #self.arm_T_sub = self.create_subscription(Float64MultiArray, '/ur_arm_T_matrix', self.arm_matrix_callback, 10)
        self.arm_T_sub = self.create_subscription(PoseStamped, '/berry_pose', self.current_pose_callback, 10)
        
        # TODO: change this with the calibration matrix!
        self.R_ac = np.array([[1,0,0],
                             [0,0,1],
                             [0,1,0]]) # Rotation matrix of the arm frame with respect to the camera frame
        
        self.p_ac = np.array([100,200,300]) # Translation vector of the camera frame relative to the arm frame in mm

        self.T_ac = np.eye(4)
        self.T_ac[:3, :3] = self.R_ac
        self.T_ac[:3, 3] = self.p_ac
    
    def rototranslate(self, T_ba): # T matrix of berry with respect to the arm frame
        
        T_bc = self.T_ac.dot(T_ba)  # Transform from arm frame to camera frame
        return T_bc
    
    def current_pose_callback(self, msg):
        # Store current pose as SE3 for use in ctraj
        # self.get_logger().info("Received current pose.")
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ori = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot = R.from_quat(ori).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        
        self.arm_T = T

    def color_callback(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    
    def depin_callback(self, intrin_msg: CameraInfo):
        self.depin_intrin = intrin_msg

    def trigger_callback(self, msg):
        self.get_logger().info(f"Received trigger: {msg.data}")

        self.trigger = msg.data
        if self.trigger and self.arm_T is not None:
            self.save_data()

    def save_data(self):
        if self.color_image is not None and self.depth_image is not None and self.depin_intrin is not None:

            pcd = CreatePointCloud.from_images(self.color_image, self.depth_image, self.depin_intrin)

            self.dataset.save_image(self.color_image)
            self.dataset.save_depth_image(self.depth_image)
            self.dataset.save_pointcloud(pcd)
            
            self.get_logger().info(f"Next random target:\n{self.arm_T}")
            T_ba = np.array(self.arm_T).reshape(4, 4)
            T_bc = self.rototranslate(T_ba)

            arm_T_str = str(T_bc)
            self.dataset.save_data(arm_T_str, arm_T_str, self.depin_intrin) # I have two arm_T_str because I have arm pose and berry pose, later I will differenciate them
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