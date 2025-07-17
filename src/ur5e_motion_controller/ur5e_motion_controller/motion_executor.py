import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Bool, Float64MultiArray
from std_msgs.msg import Int32
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
from ur_dashboard_msgs.msg import SafetyMode


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

        self.current_pose_is_safe = True
        self.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode', self.safety_callback, 10)

        self.idx_pub = self.create_publisher(Int32, '/pose_index', 10)


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
        
        self.trajectory_index = 0
        self.pose_index = 0 # Index for the current pose in the trajectory
        self.Ts = []

        self.pose_grid_filename = 'src/ur5e_motion_controller/ur5e_motion_controller/dx_poses.npy' #'src/ur5e_motion_controller/ur5e_motion_controller/dx_poses.npy' pose_grid
    
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

    def safety_callback(self, msg):

        if msg.mode == SafetyMode.NORMAL:
            self.get_logger().info("Safety status: NORMAL")
            self.current_pose_is_safe = True
        else:
            self.get_logger().warn(f"Safety issue detected: mode={msg.mode}")
            self.current_pose_is_safe = False

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
        
        self.load_random_pose()  # Load the pose grid from file
        self.find_next_incomplete_pose_index() # Find the first incomplete pose index
        self.send_random_trajectory()   # Send the first trajectory to the arm

    def load_random_pose(self, ):       
        
        try:
            poses = np.load(self.pose_grid_filename , allow_pickle=True)
            self.get_logger().info(f"Pose grid loaded from {self.pose_grid_filename}")
            self.pose_list = []
            for pose in poses:
                self.pose_list.append({
                    'position': pose['position'],
                    'orientation': pose['orientation'],
                    'completed': pose['completed'],
                    'is_safe': pose['orientation']
                })
            
            return True
        
        except Exception as e:
            self.get_logger().error(f"Failed to load pose grid from {self.pose_grid_filename}: {e}")
            return False
        
    def find_next_incomplete_pose_index(self):
        for i, pose in enumerate(self.pose_list):
            if not pose['completed']:
                self.pose_index = i
                self.get_logger().info(f"Starting from pose index {self.pose_index} out of {len(self.pose_list)}")
                return
        self.position_index = len(self.pose_list)  # All done

    def send_random_trajectory(self):

        # In send_random_trajectory
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available yet.")
            return
                
        if self.pose_index >= len(self.pose_list):
            self.get_logger().info("All poses processed.")
            return
        
        self.idx_pub.publish(Int32(data = self.pose_index))

        T1 = SE3(self.current_pose)
        
        self.T2 = np.eye(4)
        self.T2[:3, 3] = self.pose_list[self.pose_index]['position']
        self.T2[:3, :3] = self.pose_list[self.pose_index]['orientation']

        self.Ts = ctraj(T1, SE3(self.T2), 8000)

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
            self.get_logger().info(f"Published final target goal pose of pose {self.pose_index}")
            
            time.sleep(2)

            trigger_msg = Bool()
            trigger_msg.data = True
            self.trigger_pub.publish(trigger_msg)

            time.sleep(2)

            self.pose_list[self.pose_index]['completed'] = True
            self.pose_list[self.pose_index]['is_safe'] = self.current_pose_is_safe

            # Write the updated pose list back to the file
            np.save(self.pose_grid_filename , self.pose_list)
            self.pose_index += 1

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
    