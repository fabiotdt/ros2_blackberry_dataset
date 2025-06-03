
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float64MultiArray
import threading
import sys, select, termios, tty
import time

from cartesian_controller_msgs.action import FollowCartesianTrajectory
from cartesian_controller_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from roboticstoolbox.tools.trajectory import ctraj
import os
from urdfpy import URDF
from roboticstoolbox import ERobot

class MotionDataLoop(Node):
    
    def __init__(self):
        super().__init__('motion_and_dataset_loop')

        # Load the URDF file for your robotic arm
        urdf_path = '..\resource\ur5e.urdf'  # Update this path to your URDF file
        robot_urdf = URDF.load(urdf_path)

        # Convert URDF to a Robotics Toolbox ERobot for kinematics
        robot = ERobot.URDF(urdf_path)
        self.get_logger().info(f"Loaded robot: {robot.name}")

        self.client = ActionClient(self, FollowCartesianTrajectory, '/cartesian_motion_controller/follow_cartesian_trajectory')

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
        
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server not available yet.")
            return

        self.in_progress = True
        self.get_logger().info("Starting new motion-data cycle")
        self.send_random_trajectory()

    def send_random_trajectory(self):
        T1 = np.eye(4)
        self.T2 = T1.copy()
        self.T2[:3, 3] = np.random.uniform([0.4, -0.2, 0.4], [0.6, 0.2, 0.6]) # Changhe the range of searchspace accroding to the arm span

        self.get_logger().info(f"Next random target:\n{self.T2}")

        Ts = ctraj(T1, self.T2, 20)

        traj = CartesianTrajectory()
        traj.header.frame_id = 'base_link'

        for T in Ts:
            point = CartesianTrajectoryPoint()
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = T[:3, 3]
            q = R.from_matrix(T[:3, :3]).as_quat()
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            point.pose = pose.pose
            traj.points.append(point)

        goal = FollowCartesianTrajectory.Goal()
        goal.trajectory = traj

        self.client.send_goal_async(goal).add_done_callback(self.goal_sent)

    def goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.in_progress = False
            return
        self.get_logger().info("Goal accepted")
        goal_handle.get_result_async().add_done_callback(self.motion_done)

    def motion_done(self, future):
        
        result = future.result()
        if result.result.error_code != 0:
            self.get_logger().warn(f"Motion failed with code: {result.result.error_code}")
        else:
            self.get_logger().info("Motion completed successfully.")
            
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
    