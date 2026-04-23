import os
from collections import deque
import threading
import traceback

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from sentinel_interfaces.msg import FrameInfo
from sentinel_interfaces.msg import Detection
from sentinel_interfaces.msg import Detection2DArray
from sentinel_interfaces.msg import FrameSize
from sentinel_interfaces.msg import YoloStatus
from sentinel_interfaces.srv import SetBoolFlag
from sentinel_interfaces.srv import SetThreshold

from yolo_detector_pkg.converters import to_detection_array_msg
from yolo_detector_pkg.converters import to_driver_detection_msg
from yolo_detector_pkg.converters import to_frame_size_msg
from yolo_detector_pkg.converters import to_status_msg
from yolo_detector_pkg.detector import YoloDetector


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        package_share_dir = get_package_share_directory('yolo_detector_pkg')
        default_model_path = _find_default_model_path(package_share_dir)

        self.declare_parameter('image_topic', '/video/raw')
        self.declare_parameter('frame_info_topic', '/video/frame_info')
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('enabled', True)
        self.declare_parameter('inference_period_sec', 0.05)
        self.declare_parameter('synced_image_topic', '/yolo/image_raw')
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('driver_detection_topic', '/driver/detection')
        self.declare_parameter('driver_frame_size_topic', '/driver/frame_size')
        self.declare_parameter('track_max_missed_frames', 10)
        self.declare_parameter('track_max_center_distance_px', 200.0)
        self.declare_parameter('status_topic', '/yolo/status')
        self.declare_parameter('enable_service', '/yolo/enable')
        self.declare_parameter('threshold_service', '/yolo/set_threshold')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_info_topic = self.get_parameter('frame_info_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        self.input_height = self.get_parameter('input_height').get_parameter_value().integer_value
        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        self.inference_period_sec = self.get_parameter('inference_period_sec').get_parameter_value().double_value
        self.synced_image_topic = self.get_parameter('synced_image_topic').get_parameter_value().string_value
        self.sync_queue_size = max(
            1,
            int(self.get_parameter('sync_queue_size').get_parameter_value().integer_value)
        )
        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.driver_detection_topic = (
            self.get_parameter('driver_detection_topic').get_parameter_value().string_value
        )
        self.driver_frame_size_topic = (
            self.get_parameter('driver_frame_size_topic').get_parameter_value().string_value
        )
        self.track_max_missed_frames = max(
            1,
            int(self.get_parameter('track_max_missed_frames').get_parameter_value().integer_value)
        )
        self.track_max_center_distance_px = max(
            1.0,
            float(
                self.get_parameter(
                    'track_max_center_distance_px'
                ).get_parameter_value().double_value
            )
        )
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.enable_service = self.get_parameter('enable_service').get_parameter_value().string_value
        self.threshold_service = self.get_parameter('threshold_service').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.image_buffer = {}
        self.frame_info_buffer = {}
        self.pending_pairs = deque()
        self.queued_stamps = set()
        self.last_processed_stamp_ns = None
        self.tracked_detection = None
        self.tracked_center = None
        self.tracked_class_name = None
        self.tracked_missed_frames = 0

        self.detector = YoloDetector(
            model_path=self.model_path,
            input_width=int(self.input_width),
            input_height=int(self.input_height),
            conf_threshold=float(self.conf_threshold),
        )
        self.last_error = ''
        self.inference_fps = 0.0
        self.frame_count = 0
        self.input_image_count = 0
        self.queued_pair_count = 0
        self.detection_publish_count = 0
        self.last_fps_time = self.get_clock().now()

        self._load_model()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.frame_info_sub = self.create_subscription(
            FrameInfo,
            self.frame_info_topic,
            self.frame_info_callback,
            qos_profile_sensor_data
        )

        self.detection_pub = self.create_publisher(Detection2DArray, self.detection_topic, 10)
        self.driver_detection_pub = self.create_publisher(Detection, self.driver_detection_topic, 10)
        self.driver_frame_size_pub = self.create_publisher(
            FrameSize,
            self.driver_frame_size_topic,
            10
        )
        self.synced_image_pub = self.create_publisher(Image, self.synced_image_topic, 10)
        self.status_pub = self.create_publisher(YoloStatus, self.status_topic, 10)

        self.enable_srv = self.create_service(
            SetBoolFlag,
            self.enable_service,
            self.handle_enable
        )

        self.threshold_srv = self.create_service(
            SetThreshold,
            self.threshold_service,
            self.handle_set_threshold
        )

        self.inference_timer = self.create_timer(self.inference_period_sec, self.run_inference)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('YoloDetectorNode started')
        self.get_logger().info(f'image_topic      : {self.image_topic}')
        self.get_logger().info(f'frame_info_topic : {self.frame_info_topic}')
        self.get_logger().info(f'synced_image_topic: {self.synced_image_topic}')
        self.get_logger().info(f'detection_topic  : {self.detection_topic}')
        self.get_logger().info(f'driver_detection_topic: {self.driver_detection_topic}')
        self.get_logger().info(f'driver_frame_size_topic: {self.driver_frame_size_topic}')
        self.get_logger().info(f'track_max_missed_frames: {self.track_max_missed_frames}')
        self.get_logger().info(
            f'track_max_center_distance_px: {self.track_max_center_distance_px:.1f}'
        )
        self.get_logger().info(f'status_topic     : {self.status_topic}')
        self.get_logger().info(f'model_path       : {self.model_path}')

    def _load_model(self):
        try:
            self.detector.load()
            self.last_error = ''
            self.get_logger().info('ONNX model loaded successfully')

        except Exception as e:
            self.last_error = str(e)
            self.get_logger().error(f'Failed to load model: {e}')

    def image_callback(self, msg: Image):
        stamp_ns = self._header_to_ns(msg.header)
        if stamp_ns is None:
            self.get_logger().warning('Received image without a valid header stamp; dropping frame')
            return

        self.input_image_count += 1
        self.get_logger().info(
            f'YOLO input [{self.image_topic}]: {msg.width}x{msg.height} '
            f'encoding={msg.encoding} count={self.input_image_count}',
            throttle_duration_sec=2.0,
        )

        with self.lock:
            self.image_buffer[stamp_ns] = msg
            self._try_queue_pair_locked(stamp_ns)
            self._trim_buffers_locked()

    def frame_info_callback(self, msg: FrameInfo):
        stamp_ns = self._stamp_to_ns(msg.stamp)
        if stamp_ns is None:
            self.get_logger().warning('Received FrameInfo without a valid stamp; dropping frame info')
            return

        with self.lock:
            self.frame_info_buffer[stamp_ns] = msg
            self._try_queue_pair_locked(stamp_ns)
            self._trim_buffers_locked()

    def handle_enable(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f'YOLO enabled set to {self.enabled}'
        self.get_logger().info(response.message)
        return response

    def handle_set_threshold(self, request, response):
        self.conf_threshold = float(request.threshold)
        self.detector.set_conf_threshold(self.conf_threshold)
        response.success = True
        response.message = f'Confidence threshold set to {self.conf_threshold:.2f}'
        self.get_logger().info(response.message)
        return response

    def run_inference(self):
        if not self.enabled:
            return
        if not self.detector.is_loaded():
            return

        with self.lock:
            if not self.pending_pairs:
                return
            stamp_ns, image_msg, frame_info_msg = self.pending_pairs.popleft()
            self.queued_stamps.discard(stamp_ns)

        if stamp_ns == self.last_processed_stamp_ns:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            detections = self.detector.detect(cv_image)
            self.synced_image_pub.publish(image_msg)
            self.publish_detections(image_msg, frame_info_msg, detections)
            self.publish_driver_frame_size(image_msg)
            self.publish_driver_detections(image_msg, detections)
            self.last_processed_stamp_ns = stamp_ns
            self._update_fps()
            self.get_logger().info(
                f'YOLO inference [{self.detection_topic}]: detections={len(detections)} '
                f'processed={self.detection_publish_count}',
                throttle_duration_sec=2.0,
            )

        except Exception as e:
            self.last_error = str(e)
            self.get_logger().error(f'Inference failed: {e}')
            self.get_logger().debug(traceback.format_exc())

    def publish_detections(self, image_msg: Image, frame_info_msg: FrameInfo, detections):
        msg = to_detection_array_msg(image_msg, frame_info_msg, detections)
        self.detection_pub.publish(msg)
        self.detection_publish_count += 1

    def publish_driver_detections(self, image_msg: Image, detections):
        tracked_detection = self._select_tracked_detection(detections)
        if tracked_detection is None:
            return

        msg = to_driver_detection_msg(image_msg, tracked_detection)
        if msg is not None:
            self.driver_detection_pub.publish(msg)

    def publish_driver_frame_size(self, image_msg: Image):
        msg = to_frame_size_msg(image_msg)
        self.driver_frame_size_pub.publish(msg)

    def _select_tracked_detection(self, detections):
        valid_detections = [det for det in detections if self._bbox_center(det) is not None]
        if not valid_detections:
            self._handle_tracking_miss()
            return None

        if self.tracked_center is None:
            return self._start_tracking(valid_detections[0])

        candidates = [
            det for det in valid_detections
            if det.class_name == self.tracked_class_name
        ]
        if not candidates:
            candidates = valid_detections

        best = min(
            candidates,
            key=lambda det: self._center_distance_sq(self.tracked_center, self._bbox_center(det))
        )
        distance_sq = self._center_distance_sq(self.tracked_center, self._bbox_center(best))
        max_distance_sq = self.track_max_center_distance_px * self.track_max_center_distance_px

        if distance_sq > max_distance_sq:
            self._handle_tracking_miss()
            return None

        self._update_tracked_detection(best)
        return best

    def _start_tracking(self, detection):
        self._update_tracked_detection(detection)
        self.get_logger().info(
            f'Tracking first detected object on {self.driver_detection_topic}: '
            f'class={detection.class_name} score={detection.score:.2f}'
        )
        return detection

    def _update_tracked_detection(self, detection):
        self.tracked_detection = detection
        self.tracked_center = self._bbox_center(detection)
        self.tracked_class_name = detection.class_name
        self.tracked_missed_frames = 0

    def _handle_tracking_miss(self):
        if self.tracked_center is None:
            return

        self.tracked_missed_frames += 1
        if self.tracked_missed_frames >= self.track_max_missed_frames:
            self.get_logger().info(
                f'Tracked object lost on {self.driver_detection_topic}; waiting for next first detection'
            )
            self.tracked_detection = None
            self.tracked_center = None
            self.tracked_class_name = None
            self.tracked_missed_frames = 0

    @staticmethod
    def _bbox_center(detection):
        try:
            x1 = float(detection.x1)
            y1 = float(detection.y1)
            x2 = float(detection.x2)
            y2 = float(detection.y2)
        except Exception:
            return None

        if x2 <= x1 or y2 <= y1:
            return None

        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @staticmethod
    def _center_distance_sq(center_a, center_b):
        dx = center_a[0] - center_b[0]
        dy = center_a[1] - center_b[1]
        return dx * dx + dy * dy

    def publish_status(self):
        msg = to_status_msg(
            enabled=self.enabled,
            model_loaded=self.detector.is_loaded(),
            conf_threshold=self.conf_threshold,
            last_error=self.last_error,
        )
        self.status_pub.publish(msg)

    def _update_fps(self):
        self.frame_count += 1
        now = self.get_clock().now()
        elapsed = (now - self.last_fps_time).nanoseconds / 1e9

        if elapsed >= 1.0:
            self.inference_fps = self.frame_count / elapsed
            self.get_logger().info(f'Inference OK | fps={self.inference_fps:.2f}')
            self.frame_count = 0
            self.last_fps_time = now

    def _try_queue_pair_locked(self, stamp_ns):
        image_msg = self.image_buffer.get(stamp_ns)
        frame_info_msg = self.frame_info_buffer.get(stamp_ns)

        if image_msg is None or frame_info_msg is None:
            return
        if stamp_ns in self.queued_stamps:
            return

        self.pending_pairs.append((stamp_ns, image_msg, frame_info_msg))
        self.queued_stamps.add(stamp_ns)
        self.queued_pair_count += 1
        self.get_logger().info(
            f'YOLO synced pair queued [{self.image_topic} + {self.frame_info_topic}] '
            f'pending={len(self.pending_pairs)} total={self.queued_pair_count}',
            throttle_duration_sec=2.0,
        )

    def _trim_buffers_locked(self):
        self._trim_mapping_locked(self.image_buffer)
        self._trim_mapping_locked(self.frame_info_buffer)

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
        try:
            return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        except Exception:
            return None

    @staticmethod
    def _stamp_to_ns(stamp):
        if stamp is None:
            return None
        try:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except Exception:
            return None


def _find_default_model_path(package_share_dir: str) -> str:
    model_dir = os.path.join(package_share_dir, 'model')
    preferred_names = [
        'last.onnx',
        'yolo26m.onnx',
        'yolo26l.onnx',
        'yolo11l.onnx',
    ]

    for filename in preferred_names:
        candidate = os.path.join(model_dir, filename)
        if os.path.exists(candidate):
            return candidate

    if os.path.isdir(model_dir):
        for filename in sorted(os.listdir(model_dir)):
            if filename.endswith('.onnx'):
                return os.path.join(model_dir, filename)

    return os.path.join(model_dir, 'last.onnx')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
