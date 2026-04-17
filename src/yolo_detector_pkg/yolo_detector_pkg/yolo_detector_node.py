import os
import threading
import traceback

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from sentinel_interfaces.msg import FrameInfo
from sentinel_interfaces.msg import Detection2DArray
from sentinel_interfaces.msg import YoloStatus
from sentinel_interfaces.srv import SetBoolFlag
from sentinel_interfaces.srv import SetThreshold

from yolo_detector_pkg.converters import to_detection_array_msg
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

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_info_topic = self.get_parameter('frame_info_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        self.input_height = self.get_parameter('input_height').get_parameter_value().integer_value
        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        self.inference_period_sec = self.get_parameter('inference_period_sec').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_image_msg = None
        self.latest_frame_info_msg = None
        self.last_processed_stamp_ns = None

        self.detector = YoloDetector(
            model_path=self.model_path,
            input_width=int(self.input_width),
            input_height=int(self.input_height),
            conf_threshold=float(self.conf_threshold),
        )
        self.last_error = ''
        self.inference_fps = 0.0
        self.frame_count = 0
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

        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.status_pub = self.create_publisher(YoloStatus, '/yolo/status', 10)

        self.enable_srv = self.create_service(
            SetBoolFlag,
            '/yolo/enable',
            self.handle_enable
        )

        self.threshold_srv = self.create_service(
            SetThreshold,
            '/yolo/set_threshold',
            self.handle_set_threshold
        )

        self.inference_timer = self.create_timer(self.inference_period_sec, self.run_inference)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('YoloDetectorNode started')
        self.get_logger().info(f'image_topic      : {self.image_topic}')
        self.get_logger().info(f'frame_info_topic : {self.frame_info_topic}')
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
        with self.lock:
            self.latest_image_msg = msg

    def frame_info_callback(self, msg: FrameInfo):
        with self.lock:
            self.latest_frame_info_msg = msg

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
            image_msg = self.latest_image_msg
            frame_info_msg = self.latest_frame_info_msg

        if image_msg is None:
            return

        stamp_ns = self._header_to_ns(image_msg.header)
        if stamp_ns is not None and stamp_ns == self.last_processed_stamp_ns:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            detections = self.detector.detect(cv_image)
            self.publish_detections(image_msg, frame_info_msg, detections)

            if stamp_ns is not None:
                self.last_processed_stamp_ns = stamp_ns

            self._update_fps()

        except Exception as e:
            self.last_error = str(e)
            self.get_logger().error(f'Inference failed: {e}')
            self.get_logger().debug(traceback.format_exc())

    def publish_detections(self, image_msg: Image, frame_info_msg: FrameInfo, detections):
        msg = to_detection_array_msg(image_msg, frame_info_msg, detections)
        self.detection_pub.publish(msg)

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

    @staticmethod
    def _header_to_ns(header):
        if header is None:
            return None
        try:
            return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        except Exception:
            return None


def _find_default_model_path(package_share_dir: str) -> str:
    model_dir = os.path.join(package_share_dir, 'model')
    preferred_names = [
        'yolo26m.onnx',
        'yolo26l.onnx',
        'yolo11l.onnx',
        'best.onnx',
    ]

    for filename in preferred_names:
        candidate = os.path.join(model_dir, filename)
        if os.path.exists(candidate):
            return candidate

    if os.path.isdir(model_dir):
        for filename in sorted(os.listdir(model_dir)):
            if filename.endswith('.onnx'):
                return os.path.join(model_dir, filename)

    return os.path.join(model_dir, 'best.onnx')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
