import csv
import time
import traceback
from collections import OrderedDict
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentinel_interfaces.msg import Detection2D
from sentinel_interfaces.msg import Detection2DArray
from sentinel_interfaces.msg import FrameInfo


class UltralyticsYoloNode(Node):
    def __init__(self):
        super().__init__('ultralytics_yolo_node')

        self.declare_parameter('model_path', '/ros2_ws/src/yolo_detector_pkg/model/best2.pt')
        self.declare_parameter('image_topic', '/video/eo/preprocessed')
        self.declare_parameter('frame_info_topic', '/video/eo/preprocessed/frame_info')
        self.declare_parameter('detection_topic', '/detections/eo')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '0')
        self.declare_parameter('half', False)
        self.declare_parameter('max_det', 300)
        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_empty', True)
        self.declare_parameter('image_queue_size', 1)
        self.declare_parameter('log_period_sec', 2.0)
        self.declare_parameter('frame_info_cache_size', 60)
        self.declare_parameter('allowed_class_ids', '')
        self.declare_parameter('class_filter', '')
        self.declare_parameter('secondary_model_path', '')
        self.declare_parameter('secondary_allowed_class_ids', '')
        self.declare_parameter('secondary_class_filter', '')
        self.declare_parameter('secondary_conf_threshold', -1.0)
        self.declare_parameter('secondary_output_class_id', -1)
        self.declare_parameter('secondary_output_class_name', '')
        self.declare_parameter('dual_model_mode', 'sequential')
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('latency_log_path', '')
        self.declare_parameter('latency_log_every_n', 1)

        self.model_path = self.get_parameter('model_path').value
        self.image_topic = self.get_parameter('image_topic').value
        self.frame_info_topic = self.get_parameter('frame_info_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = str(self.get_parameter('device').value)
        self.half = bool(self.get_parameter('half').value)
        self.max_det = int(self.get_parameter('max_det').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.publish_empty = bool(self.get_parameter('publish_empty').value)
        self.image_queue_size = max(1, int(self.get_parameter('image_queue_size').value))
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.frame_info_cache_size = int(self.get_parameter('frame_info_cache_size').value)
        self.allowed_class_ids = self._parse_allowed_class_ids(
            str(self.get_parameter('allowed_class_ids').value)
        )
        self.class_filter = self._parse_class_filter(
            str(self.get_parameter('class_filter').value)
        )
        self.predict_classes = (
            sorted(self.allowed_class_ids) if self.allowed_class_ids is not None else None
        )
        self.secondary_model_path = str(self.get_parameter('secondary_model_path').value).strip()
        self.secondary_allowed_class_ids = self._parse_allowed_class_ids(
            str(self.get_parameter('secondary_allowed_class_ids').value)
        )
        self.secondary_class_filter = self._parse_class_filter(
            str(self.get_parameter('secondary_class_filter').value)
        )

        self.secondary_predict_classes = (
            sorted(self.secondary_allowed_class_ids)
            if self.secondary_allowed_class_ids is not None else None
        )
        secondary_conf_threshold = float(self.get_parameter('secondary_conf_threshold').value)
        self.secondary_conf_threshold = (
            secondary_conf_threshold if secondary_conf_threshold >= 0.0 else self.conf_threshold
        )
        self.secondary_output_class_id = int(
            self.get_parameter('secondary_output_class_id').value
        )
        self.secondary_output_class_name = str(
            self.get_parameter('secondary_output_class_name').value
        ).strip()
        self.dual_model_mode = str(self.get_parameter('dual_model_mode').value).strip().lower()
        if self.dual_model_mode not in {'sequential', 'alternate'}:
            self.get_logger().warn(
                f'Unsupported dual_model_mode={self.dual_model_mode}; using sequential'
            )
            self.dual_model_mode = 'sequential'
        self.process_every_n_frames = max(
            1,
            int(self.get_parameter('process_every_n_frames').value),
        )
        self.latency_log_path = str(self.get_parameter('latency_log_path').value).strip()
        self.latency_log_every_n = max(1, int(self.get_parameter('latency_log_every_n').value))
        self.latency_log_file = None
        self.latency_log_writer = None
        self.latency_log_counter = 0
        self._open_latency_log()

        self.bridge = CvBridge()
        self.frame_info_by_stamp = OrderedDict()
        self.last_log_time = 0.0
        self.frame_count = 0
        self.received_frame_count = 0
        self.next_model_index = 0
        self.primary_cached_detections = []
        self.secondary_cached_detections = []

        self.model = self._load_model(self.model_path)
        self.secondary_model = self._load_secondary_model()
        self.detection_pub = self.create_publisher(Detection2DArray, self.detection_topic, 10)

        self.frame_info_sub = self.create_subscription(
            FrameInfo,
            self.frame_info_topic,
            self._on_frame_info,
            30,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            self.image_queue_size,
        )

        self.get_logger().info('Ultralytics YOLO node started')
        self.get_logger().info(f'model_path      : {self.model_path}')
        self.get_logger().info(f'image_topic     : {self.image_topic}')
        self.get_logger().info(f'frame_info_topic: {self.frame_info_topic}')
        self.get_logger().info(f'detection_topic : {self.detection_topic}')
        self.get_logger().info(
            f'imgsz={self.imgsz} conf={self.conf_threshold} iou={self.iou_threshold} '
            f'device={self.device} half={self.half} '
            f'process_every_n_frames={self.process_every_n_frames} '
            f'image_queue_size={self.image_queue_size}'
        )
        self.get_logger().info(
            f'allowed_class_ids={self.allowed_class_ids} class_filter={sorted(self.class_filter)}'
        )
        if self.secondary_model is not None:
            self.get_logger().info(
                f'secondary_model_path={self.secondary_model_path} '
                f'classes={self.secondary_predict_classes} '
                f'class_filter={sorted(self.secondary_class_filter)} '
                f'conf={self.secondary_conf_threshold} '
                f'output_class=({self.secondary_output_class_id}, '
                f'{self.secondary_output_class_name}) '
                f'mode={self.dual_model_mode}'
            )

    def _load_model(self, model_path):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'Ultralytics is not installed. Install it with: python3 -m pip install ultralytics'
            ) from exc

        load_start = time.perf_counter()
        model = YOLO(model_path)
        load_ms = (time.perf_counter() - load_start) * 1000.0
        self.get_logger().info(f'Loaded Ultralytics model {model_path} in {load_ms:.2f}ms')
        return model

    def _load_secondary_model(self):
        if not self.secondary_model_path:
            return None
        if not Path(self.secondary_model_path).is_file():
            self.get_logger().warn(
                f'Secondary model not found; skipping it: {self.secondary_model_path}'
            )
            return None
        return self._load_model(self.secondary_model_path)

    def _on_frame_info(self, msg: FrameInfo):
        stamp_ns = self._stamp_to_ns(msg.stamp)
        self.frame_info_by_stamp[stamp_ns] = msg
        while len(self.frame_info_by_stamp) > self.frame_info_cache_size:
            self.frame_info_by_stamp.popitem(last=False)

    def _on_image(self, msg: Image):
        if not self.enabled:
            return

        self.received_frame_count += 1
        if (self.received_frame_count - 1) % self.process_every_n_frames != 0:
            return

        total_start = time.perf_counter()
        callback_start_ros = self.get_clock().now()
        try:
            convert_start = time.perf_counter()
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            convert_ms = (time.perf_counter() - convert_start) * 1000.0

            infer_start = time.perf_counter()
            ran_models = []
            if self.secondary_model is not None and self.dual_model_mode == 'alternate':
                if self.next_model_index == 0:
                    self.primary_cached_detections = self._predict_primary(cv_image)
                    ran_models.append('primary')
                else:
                    self.secondary_cached_detections = self._predict_secondary(cv_image)
                    ran_models.append('secondary')
                self.next_model_index = 1 - self.next_model_index
                detections = (
                    list(self.primary_cached_detections)
                    + list(self.secondary_cached_detections)
                )
            else:
                detections = self._predict_primary(cv_image)
                ran_models.append('primary')

                if self.secondary_model is not None:
                    detections.extend(self._predict_secondary(cv_image))
                    ran_models.append('secondary')

            infer_ms = (time.perf_counter() - infer_start) * 1000.0
            publish_start = time.perf_counter()
            if detections or self.publish_empty:
                self.detection_pub.publish(self._to_detection_array_msg(msg, detections))
            publish_ms = (time.perf_counter() - publish_start) * 1000.0
            total_ms = (time.perf_counter() - total_start) * 1000.0

            self.frame_count += 1
            self._log_latency(
                len(detections),
                convert_ms,
                infer_ms,
                publish_ms,
                total_ms,
                ran_models,
            )
            self._write_latency_rows(
                msg,
                len(detections),
                convert_ms,
                infer_ms,
                publish_ms,
                total_ms,
                callback_start_ros,
            )

        except Exception as exc:
            self.get_logger().error(f'Ultralytics inference failed: {exc}')
            self.get_logger().debug(traceback.format_exc())

    def _predict_primary(self, cv_image):
        results = self.model.predict(
            source=cv_image,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            half=self.half,
            max_det=self.max_det,
            classes=self.predict_classes,
            verbose=False,
        )
        return self._results_to_detection_msgs(
            results,
            allowed_class_ids=self.allowed_class_ids,
            class_filter=self.class_filter,
        )

    def _predict_secondary(self, cv_image):
        if self.secondary_model is None:
            return []

        results = self.secondary_model.predict(
            source=cv_image,
            imgsz=self.imgsz,
            conf=self.secondary_conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            half=self.half,
            max_det=self.max_det,
            classes=self.secondary_predict_classes,
            verbose=False,
        )
        return self._results_to_detection_msgs(
            results,
            allowed_class_ids=self.secondary_allowed_class_ids,
            class_filter=self.secondary_class_filter,
            output_class_id=self.secondary_output_class_id,
            output_class_name=self.secondary_output_class_name,
        )

    def _results_to_detection_msgs(
        self,
        results,
        allowed_class_ids=None,
        class_filter=None,
        output_class_id=-1,
        output_class_name='',
    ):
        if not results:
            return []

        result = results[0]
        names = getattr(result, 'names', {}) or {}
        boxes = getattr(result, 'boxes', None)
        if boxes is None:
            return []

        detections = []
        for box in boxes:
            cls_id = int(box.cls[0].item())
            score = float(box.conf[0].item())
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]

            class_name = str(names.get(cls_id, f'class_{cls_id}'))
            if allowed_class_ids is not None and cls_id not in allowed_class_ids:
                continue
            if class_filter and class_name.lower() not in class_filter:
                continue

            det = Detection2D()
            det.class_id = output_class_id if output_class_id >= 0 else cls_id
            det.class_name = output_class_name if output_class_name else class_name
            det.score = score
            det.x1 = x1
            det.y1 = y1
            det.x2 = x2
            det.y2 = y2
            detections.append(det)

        return detections

    def _to_detection_array_msg(self, image_msg: Image, detections):
        msg = Detection2DArray()
        msg.stamp = image_msg.header.stamp
        msg.frame_id = self._lookup_frame_id(image_msg)
        msg.detections = list(detections)
        return msg

    def _lookup_frame_id(self, image_msg: Image):
        stamp_ns = self._stamp_to_ns(image_msg.header.stamp)
        frame_info = self.frame_info_by_stamp.get(stamp_ns)
        if frame_info is None:
            return 0
        return int(frame_info.frame_id)

    def _log_latency(self, num_detections, convert_ms, infer_ms, publish_ms, total_ms, ran_models):
        now = time.monotonic()
        if now - self.last_log_time < self.log_period_sec:
            return
        self.last_log_time = now
        model_label = '+'.join(ran_models) if ran_models else 'none'
        self.get_logger().info(
            f'Ultralytics YOLO latency [{self.detection_topic}]: '
            f'detections={num_detections} frames={self.frame_count} '
            f'mode={self.dual_model_mode} ran={model_label} '
            f'convert={convert_ms:.2f}ms infer={infer_ms:.2f}ms '
            f'publish={publish_ms:.2f}ms total={total_ms:.2f}ms'
        )

    def _open_latency_log(self):
        if not self.latency_log_path:
            return

        try:
            self.latency_log_file = open(self.latency_log_path, 'a', newline='', buffering=1)
            self.latency_log_writer = csv.writer(self.latency_log_file)
            self.latency_log_writer.writerow([
                'time_ns',
                'node',
                'stream',
                'frame_id',
                'stamp_ns',
                'metric',
                'value_ms',
                'detections',
                'model_path',
            ])
            self.get_logger().info(f'Latency log enabled: {self.latency_log_path}')
        except OSError as exc:
            self.latency_log_file = None
            self.latency_log_writer = None
            self.get_logger().warn(
                f'Failed to open latency log file {self.latency_log_path}: {exc}'
            )

    def _write_latency_rows(
        self,
        image_msg,
        num_detections,
        convert_ms,
        infer_ms,
        publish_ms,
        total_ms,
        callback_start_ros,
    ):
        if self.latency_log_writer is None:
            return
        if self.latency_log_counter % self.latency_log_every_n != 0:
            self.latency_log_counter += 1
            return
        self.latency_log_counter += 1

        stamp_ns = self._stamp_to_ns(image_msg.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        frame_id = self._lookup_frame_id(image_msg)
        stream = 'ir' if '/ir' in self.detection_topic else 'eo'
        capture_to_yolo_start_ms = (
            (callback_start_ros.nanoseconds - stamp_ns) / 1_000_000.0
            if stamp_ns > 0 else 0.0
        )
        rows = [
            ('capture_to_yolo_start_ms', capture_to_yolo_start_ms),
            ('yolo_convert_ms', convert_ms),
            ('yolo_infer_ms', infer_ms),
            ('yolo_publish_ms', publish_ms),
            ('yolo_total_ms', total_ms),
            ('capture_to_yolo_publish_ms', capture_to_yolo_start_ms + total_ms),
        ]
        for metric, value_ms in rows:
            self.latency_log_writer.writerow([
                now_ns,
                'ultralytics_yolo',
                stream,
                frame_id,
                stamp_ns,
                metric,
                f'{value_ms:.3f}',
                num_detections,
                self.model_path,
            ])

    def destroy_node(self):
        if self.latency_log_file is not None:
            self.latency_log_file.close()
            self.latency_log_file = None
            self.latency_log_writer = None
        super().destroy_node()


    @staticmethod
    def _parse_allowed_class_ids(value):
        value = value.strip()
        if not value:
            return None

        allowed = set()
        for item in value.split(','):
            item = item.strip()
            if not item:
                continue
            allowed.add(int(item))
        return allowed

    @staticmethod
    def _parse_class_filter(value):
        return {item.strip().lower() for item in value.split(',') if item.strip()}

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = UltralyticsYoloNode()
    try:
        rclpy.spin(node) # 노드를 실행하면서 callback 처리
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
