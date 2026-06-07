import math
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
        self.declare_parameter('max_sync_time_diff_ms', 50.0)
        self.declare_parameter('max_cache_age_ms', 1000.0)

        self.first_detection_topic = self.get_parameter('first_detection_topic').value
        self.second_detection_topic = self.get_parameter('second_detection_topic').value
        self.merged_detection_topic = self.get_parameter('merged_detection_topic').value
        self.publish_policy = str(self.get_parameter('publish_policy').value).strip().lower()
        self.max_second_age_ms = float(self.get_parameter('max_second_age_ms').value)
        self.cache_size = max(1, int(self.get_parameter('cache_size').value))
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.max_sync_time_diff_ns = int(
            max(0.0, float(self.get_parameter('max_sync_time_diff_ms').value)) * 1_000_000
        )
        self.max_cache_age_ns = int(
            max(0.0, float(self.get_parameter('max_cache_age_ms').value)) * 1_000_000
        )

        if self.publish_policy not in {
            'synchronized',
            'first_immediate_with_latest_second',
            'both_immediate_with_latest_other',
        }:
            self.get_logger().warn(
                f'Unsupported publish_policy={self.publish_policy}; using synchronized'
            )
            self.publish_policy = 'synchronized'

        self.first_by_stamp = OrderedDict()
        self.second_by_stamp = OrderedDict()
        self.latest_first_msg = None
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
            f'policy={self.publish_policy} max_second_age_ms={self.max_second_age_ms} '
            f'max_sync_time_diff_ms={self.max_sync_time_diff_ns / 1_000_000:.1f} '
            f'max_cache_age_ms={self.max_cache_age_ns / 1_000_000:.1f}'
        )

    def _on_first(self, msg):
        self.latest_first_msg = msg
        if self.publish_policy in {
            'first_immediate_with_latest_second',
            'both_immediate_with_latest_other',
        }:
            self._publish_first_with_latest_second(msg)
            return
        self._store_and_try_publish(self.first_by_stamp, self.second_by_stamp, msg)

    def _on_second(self, msg):
        self.latest_second_msg = msg
        if self.publish_policy == 'first_immediate_with_latest_second':
            return
        if self.publish_policy == 'both_immediate_with_latest_other':
            self._publish_second_with_latest_first(msg)
            return
        self._store_and_try_publish(self.second_by_stamp, self.first_by_stamp, msg)

    def _store_and_try_publish(self, own_cache, other_cache, msg):
        stamp_ns = self._stamp_to_ns(msg.stamp)
        own_cache[stamp_ns] = msg
        self._trim_cache(own_cache)
        self._evict_old_cache_entries(own_cache, stamp_ns)
        self._evict_old_cache_entries(other_cache, stamp_ns)

        other_stamp_ns, other_msg = self._find_nearest(other_cache, stamp_ns)
        if other_msg is None:
            return

        other_cache.pop(other_stamp_ns)
        own_msg = own_cache.pop(stamp_ns, msg)
        self._publish_merged(own_msg, other_msg)

    def _find_nearest(self, cache, stamp_ns):
        if not cache:
            return None, None
        closest_ns = min(cache.keys(), key=lambda k: abs(k - stamp_ns))
        if abs(closest_ns - stamp_ns) > self.max_sync_time_diff_ns:
            return None, None
        return closest_ns, cache[closest_ns]

    def _evict_old_cache_entries(self, cache, reference_ns):
        to_remove = [k for k in cache if reference_ns - k > self.max_cache_age_ns]
        for k in to_remove:
            cache.pop(k, None)

    def _publish_first_with_latest_second(self, first_msg):
        second_msg = self._fresh_latest_msg(first_msg, self.latest_second_msg)
        self._publish_merged(first_msg, second_msg, first_msg)

    def _publish_second_with_latest_first(self, second_msg):
        first_msg = self._fresh_latest_msg(second_msg, self.latest_first_msg)
        self._publish_merged(first_msg, second_msg, second_msg)

    def _fresh_latest_msg(self, trigger_msg, latest_msg):
        if latest_msg is None:
            return None

        trigger_stamp_ns = self._stamp_to_ns(trigger_msg.stamp)
        latest_stamp_ns = self._stamp_to_ns(latest_msg.stamp)
        age_ms = abs(trigger_stamp_ns - latest_stamp_ns) / 1_000_000.0
        if age_ms > self.max_second_age_ms:
            return None
        return latest_msg

    def _publish_merged(self, first_msg, second_msg, stamp_msg=None):
        stamp_msg = stamp_msg or first_msg or second_msg
        if stamp_msg is None:
            return

        msg = Detection2DArray()
        msg.stamp = stamp_msg.stamp
        msg.frame_id = stamp_msg.frame_id
        first_detections = (
            self._valid_detections(first_msg.detections) if first_msg is not None else []
        )
        second_detections = (
            self._valid_detections(second_msg.detections) if second_msg is not None else []
        )
        msg.detections = first_detections + second_detections
        self.pub.publish(msg)

        self.merge_count += 1
        self._log_merge(len(first_detections), len(second_detections), len(msg.detections))

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
    def _valid_detections(detections):
        valid = []
        for det in detections:
            x1 = float(det.x1)
            y1 = float(det.y1)
            x2 = float(det.x2)
            y2 = float(det.y2)
            score = float(det.score)
            if not all(math.isfinite(v) for v in (x1, y1, x2, y2, score)):
                continue
            if x2 <= x1 or y2 <= y1:
                continue
            valid.append(det)
        return valid

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
