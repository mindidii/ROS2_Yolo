#!/usr/bin/env python3
import argparse
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CalibrationImageCapture(Node):
    def __init__(self, args):
        super().__init__("eo_calibration_image_capture")
        self.bridge = CvBridge()
        self.topic = args.topic
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.pattern_size = (args.cols, args.rows)
        self.max_images = args.max_images
        self.min_interval_sec = args.min_interval_sec
        self.save_debug = args.save_debug
        self.expected_width = args.expected_width
        self.expected_height = args.expected_height
        self.saved_count = 0
        self.last_saved_time = 0.0
        self.last_size_warn_time = 0.0

        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.handle_image,
            10)
        self.get_logger().info(
            f"Capturing EO calibration images from {self.topic} "
            f"with chessboard inner corners={self.pattern_size}")
        self.get_logger().info(
            f"Expected raw frame size: {self.expected_width}x{self.expected_height}")
        self.get_logger().info(f"Output directory: {self.output_dir}")

    def handle_image(self, msg):
        if self.saved_count >= self.max_images:
            self.get_logger().info("Finished capturing calibration images")
            rclpy.shutdown()
            return

        if msg.width != self.expected_width or msg.height != self.expected_height:
            now = time.monotonic()
            if now - self.last_size_warn_time > 2.0:
                self.get_logger().warn(
                    f"Skipping frame with unexpected size "
                    f"{msg.width}x{msg.height}; expected "
                    f"{self.expected_width}x{self.expected_height}")
                self.last_size_warn_time = now
            return

        now = time.monotonic()
        if now - self.last_saved_time < self.min_interval_sec:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, self.pattern_size)
        if not found:
            return

        self.saved_count += 1
        self.last_saved_time = now
        image_path = self.output_dir / f"eo_calib_{self.saved_count:03d}.png"
        cv2.imwrite(str(image_path), frame)

        if self.save_debug:
            debug = frame.copy()
            cv2.drawChessboardCorners(debug, self.pattern_size, corners, found)
            cv2.imwrite(str(self.output_dir / f"eo_calib_{self.saved_count:03d}_corners.png"), debug)

        self.get_logger().info(
            f"Saved {image_path} "
            f"{msg.width}x{msg.height} ({self.saved_count}/{self.max_images})")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture EO chessboard images for 1280x720 camera calibration.")
    parser.add_argument("--topic", default="/camera/eo")
    parser.add_argument("--output-dir", default="/tmp/eo_calibration_images")
    parser.add_argument("--cols", type=int, default=8, help="Inner chessboard corners per row")
    parser.add_argument("--rows", type=int, default=5, help="Inner chessboard corners per column")
    parser.add_argument("--max-images", type=int, default=30)
    parser.add_argument("--min-interval-sec", type=float, default=0.7)
    parser.add_argument("--expected-width", type=int, default=1280)
    parser.add_argument("--expected-height", type=int, default=720)
    parser.add_argument("--save-debug", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CalibrationImageCapture(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
