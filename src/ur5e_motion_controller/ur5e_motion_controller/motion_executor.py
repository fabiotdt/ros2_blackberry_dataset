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

        # Trigger the dataset_saver
        self.trigger_pub = self.create_publisher(Bool, '/ur_trigger', 10)

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
        except KeyboardInterrupt:
            print("KeyboardInterrupt received in keyboard_loop.")
            self._shutdown_requested = True
        except Exception as e:
            print(f"Unhandled exception in keyboard_loop: {e}")
            self._shutdown_requested = True
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
        
        self.create_random_grid()       # Create the grid of points in the workspace
        self.create_random_pose()       # Create the random poses with different rotations
        self.send_random_trajectory()   # Send the first trajectory to the arm
    
    def create_random_grid(self):
        
        # Define your workspace bounds in mm
        ymin, ymax = 0.350, 0.550    
        xmin, xmax = -0.350, 0.350     
        zmin, zmax = 0.100, 0.500

        spacing = 0.10  # 10 cm

        # Create ranges
        x_vals = np.arange(xmin, xmax + spacing, spacing)
        y_vals = np.arange(ymin, ymax + spacing, spacing)
        z_vals = np.arange(zmin, zmax + spacing, spacing)

        # Create a 3D grid of points
        self.X, self.Y, self.Z = np.meshgrid(x_vals, y_vals, z_vals, indexing='ij') # shape: (len(x_vals), len(y_vals), len(z_vals))


    def create_random_pose(self):

        # Discretize roll and pitch with 8 different rotations, (-45 deg, 0, 45),(-45 deg, 0, 45)
        roll_disc = [-45.0, 0.0, 45.0]
        pitch_disc = [-45.0, 0.0, 45.0]

        yaw = -90.0
        # create meshgrid of all combinations
        self.rotation_combinations = np.array(np.meshgrid(roll_disc, pitch_disc)).T.reshape(-1, 2)
        
        self.rotation_combinations = np.hstack((
            self.rotation_combinations[:, 0:1],                 # (N,1)
            np.zeros((self.rotation_combinations.shape[0], 1)), # (N,1)
            self.rotation_combinations[:, 1:2]                  # (N,1)
        ))
        
        self.rotation_combinations[:,0] += yaw  # Add yaw to the first column


    def send_random_trajectory(self):

        # In send_random_trajectory
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available yet.")
            return

        T1 = SE3(self.current_pose)
        self.T2 = np.eye(4)

        self.T2[:3,:3] = R.from_euler('xyz', self.rotation_combinations[self.rotation_index % 9], degrees=True).as_matrix()  # Set the rotation part
        # Define the first rotation index
        self.rotation_index += 1

        if self.rotation_index == 9:
            
            self.rotation_index = 0  # Reset rotation index after 9 iterations
            self.position_index += 1

            #if self.position_index >= self.X.size - 1:
            if self.position_index == 1:
                self.get_logger().info("Reached the end of the grid.")
                return
            

        self.T2[:3, 3] = [self.X.ravel()[self.position_index], 
                self.Y.ravel()[self.position_index], 
                self.Z.ravel()[self.position_index]]

        self.Ts = ctraj(T1, SE3(self.T2), 500 * 10)

        self.trajectory_timer = self.create_timer(0.002, self.publish_next_pose) 

    def publish_next_pose(self):
        
        T = self.Ts[self.trajectory_index]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link' # base_link or 'world'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = np.array(T)[:3, 3]
        q = R.from_matrix(np.array(T)[:3, :3]).as_quat()
        q = q / np.linalg.norm(q) # Normalize quaternion
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q

        self.pose_pub.publish(pose_msg)
        
        if self.trajectory_index == len(self.Ts) - 1:
            self.goal_pub.publish(pose_msg)
            self.get_logger().info("Published final target goal pose.")
            
            time.sleep(2)

            trigger_msg = Bool()
            trigger_msg.data = True
            self.trigger_pub.publish(trigger_msg)

            time.sleep(4)

            self.trajectory_timer.cancel()
            return

        self.trajectory_index += 1

    def motion_done(self):

        self.get_logger().info("Motion completed.")   

        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)
       
    def resume_loop(self): # If you would like to make it recursive
        self.get_logger().info("Ready for next motion-data cycle.")
        # Clear the cache and reset the trajectory index
        self.Ts.clear()
        #self.send_random_trajectory()
        #self.in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = MotionDataLoop()

    # try:
    while rclpy.ok() and not node._shutdown_requested:
        rclpy.spin_once(node)
        if node.trajectory_index == len(node.Ts) - 1: # TS is a list containing all the points in the trajectories
            # Tell the arm that the motion is done
            #node.motion_done()
            # Wait 3 sec before sending the next trajectory
            #time.sleep(3)
            # Send the next trajectory
            node.send_random_trajectory()
    node.destroy_node()
    rclpy.shutdown()
    