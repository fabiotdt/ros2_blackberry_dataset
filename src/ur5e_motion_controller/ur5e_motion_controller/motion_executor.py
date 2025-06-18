import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Bool, Float64MultiArray
import threading
import sys, select, termios, tty
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


#https://github.com/Hydran00/Ultrasound-Setup/tree/main_noft
# Cartesian controller
# --> https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers
# Universal Robots drivers
# --> https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
# --> https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from roboticstoolbox.tools.trajectory import ctraj
from spatialmath import SE3


class MotionDataLoop(Node):
    
    def __init__(self):
        super().__init__('motion_and_dataset_loop')

        # Create a publisher for the Cartesian motion controller --> publish the target pose
        self.pose_pub = self.create_publisher(PoseStamped, '/target_frame', 10)
        # Create a publisher for the goal pose --> publish the target goal pose
        self.goal_pub = self.create_publisher(PoseStamped, '/arm_target_goal', 10)

        # Create a subscriber to get the current pose of the arm
        self.current_pose = None
        self.create_subscription(PoseStamped, '/berry_pose', self.current_pose_callback, 10)
        # Create a subscriber to get the arm state
        """self.create_subscription(Bool, '/arm_state_reached', self.state_reached_callback, 10)
        self.waiting_for_state = False"""

        # Trigger the dataset_saver
        self.trigger_pub = self.create_publisher(Bool, '/ur_trigger', 10)
        self.matrix_pub = self.create_publisher(Float64MultiArray, '/ur_arm_T_matrix', 10) # Maybe don't need this --> the dataset_saver can get the target pose from the //berry_pose topic

        self._shutdown_requested = False
        self._paused = True
        self.in_progress = False

        # Start new thread where to run the keyboard 
        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.daemon = True
        self.thread.start()

        # Periodically check for launching a trajectory
        self.timer = self.create_timer(1.0, self.try_start_loop)
        
        self.rotation_index= 0
        self.position_index = 0
        self.trajectory_index = 0
        self.Ts = []
        self.point_idx = 0        

    def current_pose_callback(self, msg):
        # Store current pose as SE3 for use in ctraj
        # self.get_logger().info("Received current pose.")
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ori = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot = R.from_quat(ori).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        self.current_pose = SE3(T)

    def state_reached_callback(self, msg):
        if msg.data and self.waiting_for_state:
            self.waiting_for_state = False
            self.publish_next_pose()

    def keyboard_loop(self):
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

            print("Press 's' to START the pipeline, 'p' to PAUSA it and 'q' to QUIT.")
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 's':
                        self._paused = False
                        self.get_logger().info("Pipeline STARTED")
                    elif key == 'p':
                        self._paused = True
                        self.get_logger().info("Pipeline PAUSED")
                    elif key == 'q':
                        print("Quitting...")
                        self.get_logger().info("Shutdown requested by user.")
                        self._shutdown_requested = True
                        break
        except termios.error:
            print("Keyboard input not available. Run this node in a terminal.")
        finally:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def try_start_loop(self):
        if self._paused or self.in_progress:
            return
        
        self.in_progress = True
        self.get_logger().info("Starting new motion-data cycle")
        self.create_random_grid()
        self.send_random_trajectory()
    
    def create_random_grid(self):
        
        # Define your workspace bounds in mm
        ymin, ymax = 0.400, 0.650    # change according to the arm orientation
        xmin, xmax = -0.450, 0.450     # change according to the arm orientation
        zmin, zmax = 0.010, 0.600

        spacing = 0.10  # 10 cm

        # Create ranges
        x_vals = np.arange(xmin, xmax + spacing, spacing)
        y_vals = np.arange(ymin, ymax + spacing, spacing)
        z_vals = np.arange(zmin, zmax + spacing, spacing)

        # Create a 3D grid of points
        self.X, self.Y, self.Z = np.meshgrid(x_vals, y_vals, z_vals, indexing='ij')  # shape: (len(x_vals), len(y_vals), len(z_vals))

    """def create_random_rotation(self):
        
        rpy = np.random.uniform(-np.pi, np.pi, 3)  # Random roll, pitch, yaw
        rotmat = R.from_euler('xyz', rpy).as_matrix()  # Convert to rotation matrix
        return rotmat"""
    
    def create_pose_rotation(self, pose_index):

        angle = np.pi / 4  # 45 degrees in radians

        euler_angles_list = [
            (0, 0, 0),                   # Center
            (angle, 0, 0),               # Middle of side - X axis
            (0, angle, 0),               # Middle of side - Y axis
            (0, 0, angle),               # Middle of side - Z axis
            (-angle, 0, 0),              # Middle of side - negative X axis
            (angle, angle, angle),       # Corner 1
            (-angle, angle, angle),      # Corner 2
            (angle, -angle, angle),      # Corner 3
            (angle, angle, -angle),      # Corner 4
        ]

        if pose_index < 0 or pose_index >= len(euler_angles_list):
            raise ValueError("pose_index must be between 0 and 8 inclusive.")

        rpy = euler_angles_list[pose_index]
        rotmat = R.from_euler('xyz', rpy).as_matrix()
                
        rotmat = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, -1, 0]
        ])      
        
        return rotmat

    def send_random_trajectory(self):

        # In send_random_trajectory
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available yet.")
            return

        T1 = SE3(self.current_pose)
        self.T2 = np.eye(4)

        self.T2[:3, :3] = self.create_pose_rotation(self.rotation_index)  # Set the rotation part
        
        # Define the first rotation index
        self.rotation_index += 1

        if self.rotation_index == 9:  # After 9 rotations, reset to 0
            if self.position_index >= self.X.size - 1:
                self.get_logger().info("Reached the end of the grid. Resetting position index.")

            self.rotation_index = 0
            self.position_index += 1

        self.T2[:3, 3] = [self.X.ravel()[self.position_index], 
                self.Y.ravel()[self.position_index], 
                self.Z.ravel()[self.position_index]]

        self.get_logger().info(f"Next random target:\n{self.T2}")

        self.Ts = ctraj(T1, SE3(self.T2), 500 * 10)

        self.trajectory_timer = self.create_timer(0.002, self.publish_next_pose) 

    def publish_next_pose(self):
        
        if self.trajectory_index >= len(self.Ts):
            self.trajectory_timer.cancel()
            # self.motion_done()
            return

        T = self.Ts[self.trajectory_index]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link' # base_link or 'world'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = np.array(T)[:3, 3]
        q = R.from_matrix(np.array(T)[:3, :3]).as_quat()
        q = q / np.linalg.norm(q) # Normalize quaternion
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q

        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f"Published pose {self.trajectory_index + 1}/{len(self.Ts)}: {pose_msg.pose.position.x}, {pose_msg.pose.position.y}, {pose_msg.pose.position.z}")
        
        if self.trajectory_index == len(self.Ts) - 1:
            self.goal_pub.publish(pose_msg)
            self.get_logger().info("Published final target goal pose.")
            self.trajectory_timer.cancel()
            return

        self.trajectory_index += 1

    def motion_done(self):

        self.get_logger().info("Motion completed.")   

        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)
        
        T_msg = Float64MultiArray()
        T_msg.data = self.T2.flatten().tolist()
        self.matrix_pub.publish(T_msg)
        
        # Call the resume loop after a delay of --> 3 seconds
        self.get_logger().info("waiting.")
        threading.Timer(3.0, self.resume_loop).start()

    def resume_loop(self):
        self.get_logger().info("Ready for next motion-data cycle.")
        # Clear the cache and reset the trajectory index
        self.Ts.clear()
        self.send_random_trajectory()
        #self.in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = MotionDataLoop()

    # try:
    while rclpy.ok() and not node._shutdown_requested:
        rclpy.spin_once(node)
        if node.trajectory_index == len(node.Ts) - 1:
            node.send_random_trajectory()
    # rclpy.spin(node)
    # finally:
    node.destroy_node()
    rclpy.shutdown()
    