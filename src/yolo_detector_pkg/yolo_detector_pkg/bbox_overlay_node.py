from collections import deque
import threading

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from sentinel_interfaces.msg import Detection2DArray


class BBoxOverlayNode(Node):
    def __init__(self):
        super().__init__('bbox_overlay_node')

        self.declare_parameter('image_topic', '/yolo/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('annotated_image_topic', '/yolo/annotated_image')
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('font_scale', 0.5)
        self.declare_parameter('min_score', 0.0)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.annotated_image_topic = (
            self.get_parameter('annotated_image_topic').get_parameter_value().string_value
        )
        self.sync_queue_size = max(
            1,
            int(self.get_parameter('sync_queue_size').get_parameter_value().integer_value),
        )
        self.line_thickness = max(
            1,
            int(self.get_parameter('line_thickness').get_parameter_value().integer_value),
        )
        self.font_scale = max(
            0.1,
            float(self.get_parameter('font_scale').get_parameter_value().double_value),
        )
        self.min_score = float(self.get_parameter('min_score').get_parameter_value().double_value)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image_buffer = {}
        self.detection_buffer = {}
        self.queued_stamps = set()
        self.pending_pairs = deque()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.detection_callback,
            10,
        )
        self.annotated_pub = self.create_publisher(Image, self.annotated_image_topic, 10)
        self.timer = self.create_timer(0.02, self.publish_next)

        self.get_logger().info('BBoxOverlayNode started')
        self.get_logger().info(f'image_topic          : {self.image_topic}')
        self.get_logger().info(f'detection_topic      : {self.detection_topic}')
        self.get_logger().info(f'annotated_image_topic: {self.annotated_image_topic}')

    def image_callback(self, msg):
        stamp_ns = self._header_to_ns(msg.header)
        if stamp_ns is None:
            self.get_logger().warning('Received image without valid stamp; dropping frame')
            return

        with self.lock:
            self.image_buffer[stamp_ns] = msg
            self._try_queue_pair_locked(stamp_ns)
            self._trim_buffers_locked()

    def detection_callback(self, msg):
        stamp_ns = self._stamp_to_ns(msg.stamp)
        if stamp_ns is None:
            self.get_logger().warning('Received detections without valid stamp; dropping detections')
            return

        with self.lock:
            self.detection_buffer[stamp_ns] = msg
            self._try_queue_pair_locked(stamp_ns)
            self._trim_buffers_locked()

    def publish_next(self):
        with self.lock:
            if not self.pending_pairs:
                return
            stamp_ns, image_msg, detection_msg = self.pending_pairs.popleft()
            self.queued_stamps.discard(stamp_ns)

        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            annotated = self._draw_detections(image, detection_msg)
            output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            output.header = image_msg.header
            self.annotated_pub.publish(output)
        except Exception as exc:
            self.get_logger().error(f'Failed to draw detections: {exc}')

    def _draw_detections(self, image, detection_msg):
        annotated = image.copy()
        height, width = annotated.shape[:2]

        for detection in detection_msg.detections:
            score = float(detection.score)
            if score < self.min_score:
                continue

            bbox = self._clip_bbox(detection, width, height)
            if bbox is None:
                continue

            x1, y1, x2, y2 = bbox
            color = self._color_for_class(detection.class_name)
            label = self._format_label(detection.class_name, score)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, self.line_thickness)
            self._draw_label(annotated, label, x1, y1, color)

        return annotated

    def _draw_label(self, image, label, x, y, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = max(1, self.line_thickness - 1)
        (text_w, text_h), baseline = cv2.getTextSize(label, font, self.font_scale, thickness)
        pad = 4
        box_y1 = max(0, y - text_h - baseline - 2 * pad)
        box_y2 = min(image.shape[0] - 1, box_y1 + text_h + baseline + 2 * pad)
        box_x2 = min(image.shape[1] - 1, x + text_w + 2 * pad)

        cv2.rectangle(image, (x, box_y1), (box_x2, box_y2), color, -1)
        cv2.putText(
            image,
            label,
            (x + pad, box_y2 - baseline - pad),
            font,
            self.font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    @staticmethod
    def _clip_bbox(detection, width, height):
        if width <= 0 or height <= 0:
            return None

        x1 = int(round(max(0.0, min(float(detection.x1), float(width - 1)))))
        y1 = int(round(max(0.0, min(float(detection.y1), float(height - 1)))))
        x2 = int(round(max(0.0, min(float(detection.x2), float(width - 1)))))
        y2 = int(round(max(0.0, min(float(detection.y2), float(height - 1)))))

        if x2 <= x1 or y2 <= y1:
            return None

        return x1, y1, x2, y2

    @staticmethod
    def _format_label(class_name, score):
        if score <= 1.0:
            return f'{class_name} {score * 100.0:.0f}%'
        return f'{class_name} {score:.0f}%'

    @staticmethod
    def _color_for_class(class_name):
        seed = sum(ord(ch) for ch in class_name)
        return (
            60 + (seed * 37) % 180,
            60 + (seed * 17) % 180,
            60 + (seed * 29) % 180,
        )

    def _try_queue_pair_locked(self, stamp_ns):
        image_msg = self.image_buffer.get(stamp_ns)
        detection_msg = self.detection_buffer.get(stamp_ns)

        if image_msg is None or detection_msg is None:
            return
        if stamp_ns in self.queued_stamps:
            return

        self.pending_pairs.append((stamp_ns, image_msg, detection_msg))
        self.queued_stamps.add(stamp_ns)

    def _trim_buffers_locked(self):
        self._trim_mapping_locked(self.image_buffer)
        self._trim_mapping_locked(self.detection_buffer)

        while len(self.pending_pairs) > self.sync_queue_size:
            stamp_ns, _, _ = self.pending_pairs.popleft()
            self.queued_stamps.discard(stamp_ns)

    def _trim_mapping_locked(self, mapping):
        while len(mapping) > self.sync_queue_size:
            oldest_stamp = min(mapping)
            del mapping[oldest_stamp]

    @staticmethod
    def _header_to_ns(header):
        if header is None:
            return None
        return BBoxOverlayNode._stamp_to_ns(header.stamp)

    @staticmethod
    def _stamp_to_ns(stamp):
        if stamp is None:
            return None
        try:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = BBoxOverlayNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
