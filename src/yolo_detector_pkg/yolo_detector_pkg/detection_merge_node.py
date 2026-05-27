import time
from collections import OrderedDict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sentinel_interfaces.msg import Detection2DArray


class DetectionMergeNode(Node):
    def __init__(self):
        super().__init__('detection_merge_node')

        self.declare_parameter('first_detection_topic', '/detections/eo/drone')
        self.declare_parameter('second_detection_topic', '/detections/eo/person')
        self.declare_parameter('merged_detection_topic', '/detections/eo')
        self.declare_parameter('publish_policy', 'first_immediate_with_latest_second')
        self.declare_parameter('max_second_age_ms', 250.0)
        self.declare_parameter('cache_size', 30)
        self.declare_parameter('log_period_sec', 2.0)

        self.first_detection_topic = self.get_parameter('first_detection_topic').value
        self.second_detection_topic = self.get_parameter('second_detection_topic').value
        self.merged_detection_topic = self.get_parameter('merged_detection_topic').value
        self.publish_policy = str(self.get_parameter('publish_policy').value).strip().lower()
        self.max_second_age_ms = float(self.get_parameter('max_second_age_ms').value)
        self.cache_size = max(1, int(self.get_parameter('cache_size').value))
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        if self.publish_policy not in {'synchronized', 'first_immediate_with_latest_second'}:
            self.get_logger().warn(
                f'Unsupported publish_policy={self.publish_policy}; using synchronized'
            )
            self.publish_policy = 'synchronized'

        self.first_by_stamp = OrderedDict()
        self.second_by_stamp = OrderedDict()
        self.latest_second_msg = None
        self.last_log_time = 0.0
        self.merge_count = 0

        self.pub = self.create_publisher(
            Detection2DArray,
            self.merged_detection_topic,
            10,
        )
        self.first_sub = self.create_subscription(
            Detection2DArray,
            self.first_detection_topic,
            self._on_first,
            10,
        )
        self.second_sub = self.create_subscription(
            Detection2DArray,
            self.second_detection_topic,
            self._on_second,
            10,
        )

        self.get_logger().info(
            'Detection merge node started: '
            f'{self.first_detection_topic} + {self.second_detection_topic} '
            f'-> {self.merged_detection_topic} '
            f'policy={self.publish_policy} max_second_age_ms={self.max_second_age_ms}'
        )

    def _on_first(self, msg):
        if self.publish_policy == 'first_immediate_with_latest_second':
            self._publish_first_with_latest_second(msg)
            return
        self._store_and_try_publish(self.first_by_stamp, self.second_by_stamp, msg)

    def _on_second(self, msg):
        self.latest_second_msg = msg
        if self.publish_policy == 'first_immediate_with_latest_second':
            return
        self._store_and_try_publish(self.second_by_stamp, self.first_by_stamp, msg)

    def _store_and_try_publish(self, own_cache, other_cache, msg):
        stamp_ns = self._stamp_to_ns(msg.stamp)
        own_cache[stamp_ns] = msg
        self._trim_cache(own_cache)

        other_msg = other_cache.pop(stamp_ns, None)
        if other_msg is None:
            return

        own_msg = own_cache.pop(stamp_ns)
        self._publish_merged(own_msg, other_msg)

    def _publish_first_with_latest_second(self, first_msg):
        second_msg = self._fresh_latest_second(first_msg)
        self._publish_merged(first_msg, second_msg)

    def _fresh_latest_second(self, first_msg):
        if self.latest_second_msg is None:
            return None

        first_stamp_ns = self._stamp_to_ns(first_msg.stamp)
        second_stamp_ns = self._stamp_to_ns(self.latest_second_msg.stamp)
        age_ms = abs(first_stamp_ns - second_stamp_ns) / 1_000_000.0
        if age_ms > self.max_second_age_ms:
            return None
        return self.latest_second_msg

    def _publish_merged(self, first_msg, second_msg):
        msg = Detection2DArray()
        msg.stamp = first_msg.stamp
        msg.frame_id = first_msg.frame_id
        second_detections = list(second_msg.detections) if second_msg is not None else []
        msg.detections = list(first_msg.detections) + second_detections
        self.pub.publish(msg)

        self.merge_count += 1
        self._log_merge(len(first_msg.detections), len(second_detections), len(msg.detections))

    def _trim_cache(self, cache):
        while len(cache) > self.cache_size:
            cache.popitem(last=False)

    def _log_merge(self, first_count, second_count, total_count):
        now = time.monotonic()
        if now - self.last_log_time < self.log_period_sec:
            return
        self.last_log_time = now
        self.get_logger().info(
            f'Detection merge [{self.merged_detection_topic}]: '
            f'merges={self.merge_count} first={first_count} '
            f'second={second_count} total={total_count}'
        )

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionMergeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
