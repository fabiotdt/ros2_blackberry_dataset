# realsense_streamer/realsense_streamer/RealSenseStreamer.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import numpy as np
import cv2


class RealSenseStreamer(Node):
    def __init__(self, w=1280, h=720, fps=30, display=True):
        super().__init__('realsense_streamer')
        self.bridge = CvBridge()
        self.display = display

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable color and depth streams
        self.config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)

        self.align = rs.align(rs.stream.color)

        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.start_stream()

    def start_stream(self):
        self.pipeline.start(self.config)
        for _ in range(5):
            self.pipeline.wait_for_frames()
        self.get_logger().info("Started RealSense camera stream")

    def stop_stream(self):
        self.pipeline.stop()
        self.get_logger().info("Stopped RealSense camera stream")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warn("Frames not received")
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)

        if self.display:
            cv2.imshow("RealSense Color Stream", color_image)
            key = cv2.waitKey(1)
            if key == 27:  # ESC to quit
                self.stop_stream()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, stopping...")
    finally:
        node.stop_stream()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()