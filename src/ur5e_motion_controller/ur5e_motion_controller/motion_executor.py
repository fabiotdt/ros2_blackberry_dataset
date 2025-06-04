import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Bool, Float64MultiArray
import threading
import sys, select, termios, tty
import time

#https://github.com/Hydran00/Ultrasound-Setup/tree/main_noft
# Cartesian controller
# --> https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers
# Universal Robots drivers
# --> https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from roboticstoolbox.tools.trajectory import ctraj


class MotionDataLoop(Node):
    
    def __init__(self):
        super().__init__('motion_and_dataset_loop')
        # Create a publisher for the Cartesian motion controller
        self.pose_pub = self.create_publisher(PoseStamped, '/cartesian_motion_controller/cartesian_command', 10)

        # Trigger the dataset_saver
        self.trigger_pub = self.create_publisher(Bool, '/ur_trigger', 10)
        self.matrix_pub = self.create_publisher(Float64MultiArray, '/ur_arm_T_matrix', 10)

        self._shutdown_requested = False
        self._paused = True
        self.in_progress = False

        # Start new thread where to run the keyboard 
        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.daemon = True
        self.thread.start()

        # Prediocially check for launching a trajectory
        self.timer = self.create_timer(1.0, self.try_start_loop)

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
        self.send_random_trajectory()

    def send_random_trajectory(self):
        
        T1 = np.eye(4)
        self.T2 = T1.copy()
        self.T2[:3, 3] = np.random.uniform([0.4, -0.2, 0.4], [0.6, 0.2, 0.6]) # Changhe the range of searchspace accroding to the arm span

        self.get_logger().info(f"Next random target:\n{self.T2}")

        self.Ts = ctraj(T1, self.T2, 20)
        self.trajectory_index = 0
        self.trajectory_timer = self.create_timer(0.1, self.publish_next_pose) 

    def publish_next_pose(self):
        if self.trajectory_index >= len(self.Ts):
            self.trajectory_timer.cancel()
            self.motion_done()
            return

        T = self.Ts[self.trajectory_index]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = T[:3, 3]
        q = R.from_matrix(T[:3, :3]).as_quat()
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q

        self.pose_pub.publish(pose_msg)
        self.trajectory_index += 1

    def motion_done(self):

        self.get_logger().info("Motion completed.")   

        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)
        
        T_msg = Float64MultiArray()
        T_msg.data = self.T2.flatten().tolist()
        self.matrix_pub.publish(T_msg)
        
        # Use a threading.Timer for one-shot behavior since rclpy.create_timer does not support oneshot
        threading.Timer(1.0, self.resume_loop).start()

    def resume_loop(self):
        self.get_logger().info("Ready for next motion-data cycle.")
        self.in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = MotionDataLoop()

    try:
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    