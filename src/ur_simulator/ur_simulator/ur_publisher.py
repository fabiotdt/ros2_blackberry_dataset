import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
import numpy as np
import threading
import sys, select, termios, tty
import time

class URSimulator(Node):
    def __init__(self):
        super().__init__('ur_fake_publisher')
        self.trigger_pub = self.create_publisher(Bool, '/ur_trigger', 10)
        self.matrix_pub = self.create_publisher(Float64MultiArray, '/ur_arm_T_matrix', 10)
        self._shutdown_requested = False
        self._paused = True

        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def keyboard_loop(self):
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

            print("Press 't' to simulate UR trigger, 'q' to quit.")
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 't':
                        self.publish_fake_data()
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

    def publish_fake_data(self):
        self.get_logger().info("Publishing UR trigger and fake T-matrix...")

        # Publish trigger
        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)

        # Dummy 4x4 transformation matrix as flat list (row-major)
        T_matrix = np.eye(4).flatten()  # Replace with real matrix later
        T_msg = Float64MultiArray()
        T_msg.data = T_matrix.tolist()
        self.matrix_pub.publish(T_msg)

def main(args=None):
    rclpy.init(args=args)
    node = URSimulator()
    try:
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down node.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

