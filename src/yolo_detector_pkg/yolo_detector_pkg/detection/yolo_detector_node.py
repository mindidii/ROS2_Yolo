from collections import deque
from dataclasses import replace
import threading
import time
import traceback

import rclpy
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

from yolo_detector_pkg.common.converters import to_detection_array_msg
from yolo_detector_pkg.common.converters import to_driver_detection_msg
from yolo_detector_pkg.common.converters import to_frame_size_msg
from yolo_detector_pkg.common.converters import to_status_msg
from yolo_detector_pkg.detection.detector import YoloDetector


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('image_topic', '/video/raw')
        self.declare_parameter('frame_info_topic', '/video/frame_info')
        self.declare_parameter('require_frame_info', False)
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.2)
        self.declare_parameter('class_filter', '')
        self.declare_parameter('allowed_class_ids', '0,1')
        self.declare_parameter('class_conf_thresholds', '0:0.2,1:0.5')
        self.declare_parameter('class_min_area_ratios', '0:0.0002,1:0.00002')
        self.declare_parameter('drone_max_area_ratio', 0.80)
        self.declare_parameter('drone_max_width_ratio', 0.80)
        self.declare_parameter('drone_max_height_ratio', 0.80)
        self.declare_parameter('smooth_alpha', 0.35)
        self.declare_parameter('hold_class_ids', '1')
        self.declare_parameter('drone_hold_missing_frames', 10)
        self.declare_parameter('drone_hold_min_conf', 0.2)
        self.declare_parameter('stale_track_frames', 20)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('enabled', True)
        self.declare_parameter('inference_period_sec', 0.05)
        self.declare_parameter('inference_frame_stride', 3)
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('process_latest_only', True)
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('driver_detection_topic', '/driver/detection')
        self.declare_parameter('publish_driver_detection', True)
        self.declare_parameter('driver_frame_size_topic', '/driver/frame_size')
        self.declare_parameter('detection_hold_sec', 0.5)
        self.declare_parameter('track_max_missed_frames', 10)
        self.declare_parameter('track_max_center_distance_px', 200.0)
        self.declare_parameter('status_topic', '/yolo/status')
        self.declare_parameter('enable_service', '/yolo/enable')
        self.declare_parameter('threshold_service', '/yolo/set_threshold')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_info_topic = self.get_parameter('frame_info_topic').get_parameter_value().string_value
        self.require_frame_info = (
            self.get_parameter('require_frame_info').get_parameter_value().bool_value
        )
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value.strip()
        if not self.model_path:
            raise ValueError('model_path parameter is required. Set it in the YOLO YAML config.')
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.class_filter = self._parse_class_filter(
            self.get_parameter('class_filter').get_parameter_value().string_value
        )
        self.allowed_class_ids = self._parse_int_set(
            self.get_parameter('allowed_class_ids').get_parameter_value().string_value
        )
        self.class_conf_thresholds = self._parse_float_map(
            self.get_parameter('class_conf_thresholds').get_parameter_value().string_value
        )
        self.class_min_area_ratios = self._parse_float_map(
            self.get_parameter('class_min_area_ratios').get_parameter_value().string_value
        )
        self.drone_max_area_ratio = float(
            self.get_parameter('drone_max_area_ratio').get_parameter_value().double_value
        )
        self.drone_max_width_ratio = float(
            self.get_parameter('drone_max_width_ratio').get_parameter_value().double_value
        )
        self.drone_max_height_ratio = float(
            self.get_parameter('drone_max_height_ratio').get_parameter_value().double_value
        )
        self.smooth_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter('smooth_alpha').get_parameter_value().double_value))
        )
        self.hold_class_ids = self._parse_int_set(
            self.get_parameter('hold_class_ids').get_parameter_value().string_value
        )
        self.drone_hold_missing_frames = max(
            0,
            int(self.get_parameter('drone_hold_missing_frames').get_parameter_value().integer_value)
        )
        self.drone_hold_min_conf = float(
            self.get_parameter('drone_hold_min_conf').get_parameter_value().double_value
        )
        self.stale_track_frames = max(
            1,
            int(self.get_parameter('stale_track_frames').get_parameter_value().integer_value)
        )
        self.input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        self.input_height = self.get_parameter('input_height').get_parameter_value().integer_value
        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        self.inference_period_sec = self.get_parameter('inference_period_sec').get_parameter_value().double_value
        self.inference_frame_stride = max(
            1,
            int(self.get_parameter('inference_frame_stride').get_parameter_value().integer_value)
        )
        self.sync_queue_size = max(
            1,
            int(self.get_parameter('sync_queue_size').get_parameter_value().integer_value)
        )
        self.process_latest_only = (
            self.get_parameter('process_latest_only').get_parameter_value().bool_value
        )
        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.driver_detection_topic = (
            self.get_parameter('driver_detection_topic').get_parameter_value().string_value
        )
        self.publish_driver_detection = (
            self.get_parameter('publish_driver_detection').get_parameter_value().bool_value
        )
        self.driver_frame_size_topic = (
            self.get_parameter('driver_frame_size_topic').get_parameter_value().string_value
        )
        self.detection_hold_sec = max(
            0.0,
            float(self.get_parameter('detection_hold_sec').get_parameter_value().double_value)
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
        self.processed_pair_count = 0
        self.tracked_detection = None
        self.tracked_center = None
        self.tracked_class_name = None
        self.tracked_missed_frames = 0
        self.last_detections = []
        self.last_detection_time = self.get_clock().now()
        self.frame_index = 0
        self.tracked_track_id = None
        self.smoothed_boxes = {}
        self.box_velocities = {}
        self.last_confs = {}
        self.last_seen = {}

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

        self.frame_info_sub = None
        if self.frame_info_topic:
            self.frame_info_sub = self.create_subscription(
                FrameInfo,
                self.frame_info_topic,
                self.frame_info_callback,
                qos_profile_sensor_data
            )

        self.detection_pub = self.create_publisher(Detection2DArray, self.detection_topic, 10)
        self.driver_detection_pub = None
        if self.publish_driver_detection and self.driver_detection_topic:
            self.driver_detection_pub = self.create_publisher(
                Detection,
                self.driver_detection_topic,
                10,
            )
        self.driver_frame_size_pub = self.create_publisher(
            FrameSize,
            self.driver_frame_size_topic,
            10
        )
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
        self.get_logger().info(f'class_filter    : {sorted(self.class_filter) if self.class_filter else []}')
        self.get_logger().info(f'allowed_class_ids: {sorted(self.allowed_class_ids)}')
        self.get_logger().info(f'class_conf_thresholds: {self.class_conf_thresholds}')
        self.get_logger().info(f'class_min_area_ratios: {self.class_min_area_ratios}')
        self.get_logger().info(f'hold_class_ids  : {sorted(self.hold_class_ids)}')
        self.get_logger().info(f'require_frame_info: {self.require_frame_info}')
        self.get_logger().info(f'process_latest_only: {self.process_latest_only}')
        self.get_logger().info(f'inference_frame_stride: {self.inference_frame_stride}')
        self.get_logger().info(f'detection_topic  : {self.detection_topic}')
        self.get_logger().info(f'driver_detection_topic: {self.driver_detection_topic}')
        self.get_logger().info(f'publish_driver_detection: {self.publish_driver_detection}')
        self.get_logger().info(f'driver_frame_size_topic: {self.driver_frame_size_topic}')
        self.get_logger().info(f'detection_hold_sec: {self.detection_hold_sec:.2f}')
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
            providers = self.detector.get_execution_providers()
            backend_name = providers[0] if providers else 'unknown backend'
            self.get_logger().info(f'Model loaded successfully with {backend_name}')
            self.get_logger().info(
                f'Model execution providers: {providers}'
            )

        except Exception as e:
            self.last_error = str(e)
            self.enabled = False
            self.get_logger().error(
                f'Failed to load model; YOLO node will stay alive but disabled: {e}'
            )
            self.get_logger().error(
                'If this is a TensorRT .engine deserialization error, rebuild the engine '
                'on this machine with the currently installed TensorRT/CUDA runtime.'
            )
            self.get_logger().debug(traceback.format_exc())

    # 이미지가 들어오면 timestamp 기준으로 버퍼에 저장 
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

    # 이미지와 매칭되는 frame info를 저장
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
        if request.data and not self.detector.is_loaded():
            self._load_model()

        if request.data and not self.detector.is_loaded():
            response.success = False
            response.message = f'YOLO model is not loaded: {self.last_error}'
            self.get_logger().error(response.message)
            return response

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
            if self.process_latest_only and len(self.pending_pairs) > 1:
                self._drop_stale_pending_pairs_locked()
            stamp_ns, image_msg, frame_info_msg = self.pending_pairs.popleft()
            self.queued_stamps.discard(stamp_ns)

        if stamp_ns == self.last_processed_stamp_ns:
            return

        self.processed_pair_count += 1
        self.frame_index += 1
        self._prune_track_state()
        should_run_detector = (
            (self.processed_pair_count - 1) % self.inference_frame_stride == 0
        )

        try:
            if not should_run_detector:
                self.publish_detections(image_msg, frame_info_msg, [])
                self.publish_driver_frame_size(image_msg)
                self.publish_driver_detections(image_msg, [])
                self.last_processed_stamp_ns = stamp_ns
                return

            total_start = time.perf_counter()
            convert_start = time.perf_counter()
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            convert_ms = (time.perf_counter() - convert_start) * 1000.0

            detect_start = time.perf_counter()
            detections = self.detector.detect(cv_image)
            detections = self._filter_detections(detections, image_msg)
            detect_ms = (time.perf_counter() - detect_start) * 1000.0

            publish_start = time.perf_counter()
            self.publish_detections(image_msg, frame_info_msg, detections)
            self.publish_driver_frame_size(image_msg)
            self.publish_driver_detections(image_msg, detections)
            publish_ms = (time.perf_counter() - publish_start) * 1000.0
            total_ms = (time.perf_counter() - total_start) * 1000.0

            self.last_processed_stamp_ns = stamp_ns
            self._update_fps()
            self.get_logger().info(
                f'YOLO inference [{self.detection_topic}]: detections={len(detections)} '
                f'stride={self.inference_frame_stride} '
                f'processed={self.detection_publish_count}',
                throttle_duration_sec=2.0,
            )
            self.get_logger().info(
                f'YOLO latency [{self.detection_topic}]: '
                f'convert={convert_ms:.2f}ms detect={detect_ms:.2f}ms '
                f'publish={publish_ms:.2f}ms total={total_ms:.2f}ms',
                throttle_duration_sec=2.0,
            )

        except Exception as e:
            self.last_error = str(e)
            self.get_logger().error(f'Inference failed: {e}')
            self.get_logger().debug(traceback.format_exc())

    
    
    
    @staticmethod
    def _parse_class_filter(class_filter: str):
        return {
            item.strip().lower()
            for item in str(class_filter).split(',')
            if item.strip()
        }

    @staticmethod
    def _parse_int_set(value: str):
        return {
            int(item.strip())
            for item in str(value).split(',')
            if item.strip()
        }

    @staticmethod
    def _parse_float_map(value: str):
        parsed = {}
        for item in str(value).split(','):
            if not item.strip() or ':' not in item:
                continue
            key, raw_value = item.split(':', 1)
            parsed[int(key.strip())] = float(raw_value.strip())
        return parsed

    def _filter_detections(self, detections, image_msg: Image):
        frame_area = max(1.0, float(image_msg.width) * float(image_msg.height))
        filtered = []

        for det in detections:
            class_id = int(det.class_id)
            if self.allowed_class_ids and class_id not in self.allowed_class_ids:
                continue
            if self.class_filter and str(det.class_name).lower() not in self.class_filter:
                continue

            conf_threshold = self.class_conf_thresholds.get(class_id, self.conf_threshold)
            if float(det.score) < conf_threshold:
                continue

            width = max(0.0, float(det.x2) - float(det.x1))
            height = max(0.0, float(det.y2) - float(det.y1))
            if width <= 0.0 or height <= 0.0:
                continue

            area_ratio = (width * height) / frame_area
            min_area_ratio = self.class_min_area_ratios.get(class_id, 0.0)
            if area_ratio < min_area_ratio:
                continue

            width_ratio = width / max(1.0, float(image_msg.width))
            height_ratio = height / max(1.0, float(image_msg.height))
            if class_id == 1 and (
                area_ratio > self.drone_max_area_ratio
                or width_ratio > self.drone_max_width_ratio
                or height_ratio > self.drone_max_height_ratio
            ):
                continue

            filtered.append(det)

        return filtered

    def publish_detections(self, image_msg: Image, frame_info_msg: FrameInfo, detections):
        self._update_detection_hold(detections)
        msg = to_detection_array_msg(image_msg, frame_info_msg, detections)
        self.detection_pub.publish(msg)
        self.detection_publish_count += 1

    def _update_detection_hold(self, detections):
        if detections:
            self.last_detections = list(detections)
            self.last_detection_time = self.get_clock().now()
            return

        if not self.last_detections or self.detection_hold_sec <= 0.0:
            return

        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > self.detection_hold_sec:
            self.last_detections = []

    def publish_driver_detections(self, image_msg: Image, detections):
        if self.driver_detection_pub is None:
            return

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
            held_detection = self._hold_tracked_detection()
            if held_detection is not None:
                return held_detection
            self._handle_tracking_miss()
            return None

        if self.tracked_center is None:
            return self._start_tracking(valid_detections[0])

        candidates = [
            det for det in valid_detections
            if int(det.class_id) == int(self.tracked_detection.class_id)
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

        return self._update_tracked_detection(best)

    def _start_tracking(self, detection):
        tracked = self._update_tracked_detection(detection)
        self.get_logger().info(
            f'Tracking first detected object on {self.driver_detection_topic}: '
            f'track_id={self.tracked_track_id} class={tracked.class_name} score={tracked.score:.2f}'
        )
        return tracked

    def _update_tracked_detection(self, detection):
        track_id = self._track_id_for_detection(detection)
        smoothed_detection = self._smooth_detection(track_id, detection)
        self.tracked_detection = smoothed_detection
        self.tracked_center = self._bbox_center(smoothed_detection)
        self.tracked_class_name = smoothed_detection.class_name
        self.tracked_track_id = track_id
        self.tracked_missed_frames = 0
        self.last_confs[track_id] = float(detection.score)
        self.last_seen[track_id] = self.frame_index
        return smoothed_detection

    def _hold_tracked_detection(self):
        if self.tracked_detection is None or self.tracked_track_id is None:
            return None

        class_id = int(self.tracked_detection.class_id)
        if class_id not in self.hold_class_ids:
            return None

        last_conf = self.last_confs.get(self.tracked_track_id, float(self.tracked_detection.score))
        if last_conf < self.drone_hold_min_conf:
            return None

        if self.tracked_missed_frames >= self.drone_hold_missing_frames:
            return None

        self.tracked_missed_frames += 1
        held_detection = self._predict_detection(self.tracked_track_id, self.tracked_detection)
        self.tracked_detection = held_detection
        self.tracked_center = self._bbox_center(held_detection)
        return held_detection

    def _handle_tracking_miss(self):
        if self.tracked_center is None:
            return

        self.tracked_missed_frames += 1
        if self.tracked_missed_frames > self.stale_track_frames:
            self.get_logger().info(
                f'Tracked object lost on {self.driver_detection_topic}; waiting for next first detection'
            )
            self.tracked_detection = None
            self.tracked_center = None
            self.tracked_class_name = None
            self.tracked_track_id = None
            self.tracked_missed_frames = 0

    def _smooth_detection(self, track_id, detection):
        bbox = (float(detection.x1), float(detection.y1), float(detection.x2), float(detection.y2))
        previous = self.smoothed_boxes.get(track_id)
        if previous is None:
            smoothed = bbox
            velocity = (0.0, 0.0, 0.0, 0.0)
        else:
            smoothed = tuple(
                (1.0 - self.smooth_alpha) * previous[i] + self.smooth_alpha * bbox[i]
                for i in range(4)
            )
            velocity = tuple(smoothed[i] - previous[i] for i in range(4))

        self.smoothed_boxes[track_id] = smoothed
        self.box_velocities[track_id] = velocity
        return replace(
            detection,
            x1=smoothed[0],
            y1=smoothed[1],
            x2=smoothed[2],
            y2=smoothed[3],
        )

    def _predict_detection(self, track_id, detection):
        velocity = self.box_velocities.get(track_id, (0.0, 0.0, 0.0, 0.0))
        return replace(
            detection,
            x1=float(detection.x1) + velocity[0],
            y1=float(detection.y1) + velocity[1],
            x2=float(detection.x2) + velocity[2],
            y2=float(detection.y2) + velocity[3],
        )

    def _prune_track_state(self):
        stale_track_ids = [
            track_id for track_id, last_seen in self.last_seen.items()
            if self.frame_index - last_seen > self.stale_track_frames
        ]
        for track_id in stale_track_ids:
            self.smoothed_boxes.pop(track_id, None)
            self.box_velocities.pop(track_id, None)
            self.last_confs.pop(track_id, None)
            self.last_seen.pop(track_id, None)
            if self.tracked_track_id == track_id:
                self.tracked_detection = None
                self.tracked_center = None
                self.tracked_class_name = None
                self.tracked_track_id = None
                self.tracked_missed_frames = 0

    @staticmethod
    def _track_id_for_detection(detection):
        tracker_id = 0
        return (int(detection.class_id), tracker_id)

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

        if image_msg is None:
            return
        if self.frame_info_topic and frame_info_msg is None:
            return
        if self.require_frame_info and frame_info_msg is None:
            return
        if stamp_ns in self.queued_stamps:
            return

        if self.process_latest_only and self.pending_pairs:
            # Real-time mode: keep only the newest frame so queue lag does not accumulate.
            self._drop_stale_pending_pairs_locked()

        self.pending_pairs.append((stamp_ns, image_msg, frame_info_msg))
        self.queued_stamps.add(stamp_ns)
        self.queued_pair_count += 1
        if frame_info_msg is None:
            self.get_logger().warning(
                f'No matching FrameInfo for image on {self.image_topic}; '
                'running inference with frame_id=0',
                throttle_duration_sec=5.0,
            )
            self.get_logger().info(
                f'YOLO image queued [{self.image_topic}] '
                f'pending={len(self.pending_pairs)} total={self.queued_pair_count}',
                throttle_duration_sec=2.0,
            )
            return

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

    def _drop_stale_pending_pairs_locked(self):
        while len(self.pending_pairs) > 1:
            stale_stamp_ns, _, _ = self.pending_pairs.popleft()
            self.queued_stamps.discard(stale_stamp_ns)

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
