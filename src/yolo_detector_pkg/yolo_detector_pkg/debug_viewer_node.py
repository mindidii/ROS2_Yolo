import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentinel_interfaces.msg import Detection2DArray


class DebugDetectionViewerNode(Node):
    def __init__(self):
        super().__init__('debug_detection_viewer')

        self.declare_parameter('image_topic', '/video/eo/preprocessed')
        self.declare_parameter('detection_topic', '/detections/eo')
        self.declare_parameter('window_name', 'EO Detection Debug')
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('draw_center', True)
        self.declare_parameter('show_fps', True)
        self.declare_parameter('resize_width', 0)

        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.max_detection_age_sec = float(self.get_parameter('max_detection_age_sec').value)
        self.draw_center = bool(self.get_parameter('draw_center').value)
        self.show_fps = bool(self.get_parameter('show_fps').value)
        self.resize_width = int(self.get_parameter('resize_width').value)

        self.bridge = CvBridge()
        self.latest_detections = None
        self.latest_detection_time = 0.0
        self.frame_count = 0
        self.fps = 0.0
        self.fps_start = time.monotonic()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            10,
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.on_detections,
            10,
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f'Debug detection viewer started: image={self.image_topic}, '
            f'detections={self.detection_topic}'
        )
        self.get_logger().info('Press q or ESC in the OpenCV window to close the viewer.')

    def on_detections(self, msg: Detection2DArray):
        self.latest_detections = msg
        self.latest_detection_time = time.monotonic()

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        self._update_fps()
        self._draw_overlay(frame)

        if self.resize_width > 0 and frame.shape[1] > 0:
            scale = self.resize_width / float(frame.shape[1])
            height = max(1, int(frame.shape[0] * scale))
            frame = cv2.resize(frame, (self.resize_width, height), interpolation=cv2.INTER_AREA)

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            self.get_logger().info('Viewer close requested.')
            rclpy.shutdown()

    def _draw_overlay(self, frame):
        detections_msg = self.latest_detections
        detection_age = time.monotonic() - self.latest_detection_time
        detections = []
        if detections_msg is not None and detection_age <= self.max_detection_age_sec:
            detections = list(detections_msg.detections)

        for detection in detections:
            x1 = self._clip(int(round(detection.x1)), 0, frame.shape[1] - 1)
            y1 = self._clip(int(round(detection.y1)), 0, frame.shape[0] - 1)
            x2 = self._clip(int(round(detection.x2)), 0, frame.shape[1] - 1)
            y2 = self._clip(int(round(detection.y2)), 0, frame.shape[0] - 1)
            if x2 <= x1 or y2 <= y1:
                continue

            color = self._color_for_class(int(detection.class_id))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            label = f'{detection.class_name} {detection.score:.2f}'
            self._draw_label(frame, label, x1, y1, color)

            if self.draw_center:
                cx = int((x1 + x2) * 0.5)
                cy = int((y1 + y2) * 0.5)
                cv2.drawMarker(
                    frame,
                    (cx, cy),
                    color,
                    markerType=cv2.MARKER_CROSS,
                    markerSize=12,
                    thickness=2,
                )

        status = f'detections={len(detections)}'
        if detections_msg is None:
            status = 'waiting for detections'
        elif detection_age > self.max_detection_age_sec:
            status = f'detections stale ({detection_age:.2f}s)'

        lines = [status]
        if self.show_fps:
            lines.append(f'viewer fps={self.fps:.1f}')
        self._draw_status(frame, lines)

    def _draw_label(self, frame, text, x, y, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 1
        (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)
        label_y = max(text_h + baseline + 4, y)
        cv2.rectangle(
            frame,
            (x, label_y - text_h - baseline - 4),
            (x + text_w + 6, label_y + baseline - 2),
            color,
            -1,
        )
        cv2.putText(
            frame,
            text,
            (x + 3, label_y - 4),
            font,
            scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def _draw_status(self, frame, lines):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thickness = 1
        y = 24
        for line in lines:
            cv2.putText(
                frame,
                line,
                (12, y),
                font,
                scale,
                (20, 20, 20),
                thickness + 2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                line,
                (12, y),
                font,
                scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
            y += 24

    def _update_fps(self):
        self.frame_count += 1
        now = time.monotonic()
        elapsed = now - self.fps_start
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_start = now

    @staticmethod
    def _clip(value, low, high):
        return max(low, min(value, high))

    @staticmethod
    def _color_for_class(class_id):
        palette = [
            (80, 220, 60),
            (40, 170, 255),
            (255, 120, 60),
            (220, 80, 220),
            (60, 220, 220),
        ]
        return palette[class_id % len(palette)]


def main(args=None):
    rclpy.init(args=args)
    node = DebugDetectionViewerNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

