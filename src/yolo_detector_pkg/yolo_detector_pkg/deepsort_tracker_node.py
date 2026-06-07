from collections import OrderedDict, deque
import math
import time

import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from deep_sort_realtime.deepsort_tracker import DeepSort

from sentinel_interfaces.msg import Detection2DArray
from sentinel_interfaces.msg import MotorAngle
from sentinel_interfaces.msg import TrackedDetection2D
from sentinel_interfaces.msg import TrackedDetection2DArray


class DeepSortTrackerNode(Node):
    def __init__(self):
        super().__init__('deepsort_tracker_node')

        self.declare_parameter('image_topic', '/video/eo/preprocessed')
        self.declare_parameter('detection_topic', '/detections/eo')
        self.declare_parameter('tracks_topic', '/tracks/eo')
        self.declare_parameter('max_age', 30)
        self.declare_parameter('min_confidence', 0.05)
        self.declare_parameter('embedder_gpu', True)
        self.declare_parameter('image_cache_size', 60)
        self.declare_parameter('image_queue_size', 1)
        self.declare_parameter('max_frame_time_diff_ms', 100.0)
        self.declare_parameter('publish_prediction_tracks', False)
        self.declare_parameter('min_confirm_hits', 2)
        self.declare_parameter('motor_angle_topic', '/motor/angle/get')
        self.declare_parameter('camera_fx', 977.871299)
        self.declare_parameter('camera_fy', 973.856636)
        self.declare_parameter('pan_counts_per_degree', 1.0)
        self.declare_parameter('tilt_counts_per_degree', 1.0)
        self.declare_parameter('pan_wrap_counts', 65536.0)
        self.declare_parameter('tilt_wrap_counts', 65536.0)
        self.declare_parameter('pan_pixel_sign', -1.0)
        self.declare_parameter('tilt_pixel_sign', -1.0)
        self.declare_parameter('external_reacquire_window_sec', 3.0)
        self.declare_parameter('external_reacquire_max_distance_px', 120.0)
        self.declare_parameter('external_reacquire_same_class', True)
        self.declare_parameter('hold_class_ids', '')
        self.declare_parameter('hold_missing_frames', 0)
        self.declare_parameter('hold_missing_sec', 0.0)
        self.declare_parameter('id_cooldown_sec', 6.0)
        self.declare_parameter('stats_period_sec', 5.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.tracks_topic = self.get_parameter('tracks_topic').value
        self.max_age = int(self.get_parameter('max_age').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.embedder_gpu = bool(self.get_parameter('embedder_gpu').value)
        self.image_cache_size = int(self.get_parameter('image_cache_size').value)
        self.image_queue_size = max(1, int(self.get_parameter('image_queue_size').value))
        self.max_frame_time_diff_ns = int(
            max(0.0, float(self.get_parameter('max_frame_time_diff_ms').value)) * 1_000_000
        )
        self.publish_prediction_tracks = bool(
            self.get_parameter('publish_prediction_tracks').value
        )
        self.min_confirm_hits = max(1, int(self.get_parameter('min_confirm_hits').value))
        self.motor_angle_topic = self.get_parameter('motor_angle_topic').value
        self.camera_fx = float(self.get_parameter('camera_fx').value)
        self.camera_fy = float(self.get_parameter('camera_fy').value)
        self.pan_counts_per_degree = max(
            1.0e-6, float(self.get_parameter('pan_counts_per_degree').value)
        )
        self.tilt_counts_per_degree = max(
            1.0e-6, float(self.get_parameter('tilt_counts_per_degree').value)
        )
        self.pan_wrap_counts = max(1.0, float(self.get_parameter('pan_wrap_counts').value))
        self.tilt_wrap_counts = max(1.0, float(self.get_parameter('tilt_wrap_counts').value))
        self.pan_pixel_sign = float(self.get_parameter('pan_pixel_sign').value)
        self.tilt_pixel_sign = float(self.get_parameter('tilt_pixel_sign').value)
        self.external_reacquire_window_sec = float(
            self.get_parameter('external_reacquire_window_sec').value
        )
        self.external_reacquire_max_distance_px = float(
            self.get_parameter('external_reacquire_max_distance_px').value
        )
        self.external_reacquire_same_class = bool(
            self.get_parameter('external_reacquire_same_class').value
        )
        self.hold_class_ids = self._parse_class_ids(
            self.get_parameter('hold_class_ids').value
        )
        self.hold_missing_frames = max(
            0, int(self.get_parameter('hold_missing_frames').value)
        )
        self.hold_missing_sec = max(
            0.0, float(self.get_parameter('hold_missing_sec').value)
        )
        self.id_cooldown_sec = max(
            self.external_reacquire_window_sec + 1.0,
            float(self.get_parameter('id_cooldown_sec').value),
        )
        self.stats_period_sec = max(
            1.0, float(self.get_parameter('stats_period_sec').value)
        )

        self.tracker = DeepSort(
            max_age=self.max_age,
            n_init=self.min_confirm_hits,
            embedder_gpu=self.embedder_gpu,
        )

        self.class_names: dict[int, str] = {}
        self.internal_track_classes: dict[str, int] = {}
        self.internal_to_external_id: dict[str, int] = {}
        self.external_track_states: dict[int, dict] = {}
        self.held_tracks: dict[int, dict] = {}
        self.current_motor_angle = None
        self.available_external_ids: deque = deque(range(1, 255))
        self.released_id_cooldown: dict[int, float] = {}
        self.bridge = CvBridge()
        self.image_cache: OrderedDict[int, np.ndarray] = OrderedDict()

        self._stats_reacquire_count = 0

        self.pub = self.create_publisher(TrackedDetection2DArray, self.tracks_topic, 10)
        image_qos = QoSProfile(
            depth=self.image_queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.on_image, image_qos
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, self.detection_topic, self.on_detections, 1
        )
        self.motor_angle_sub = self.create_subscription(
            MotorAngle, self.motor_angle_topic, self.on_motor_angle, 10
        )
        self.create_timer(self.stats_period_sec, self._log_stats)

        self.get_logger().info(
            f'DeepSORT tracker started: '
            f'{self.detection_topic} + {self.image_topic} -> {self.tracks_topic}'
        )
        self.get_logger().info(
            f'max_age={self.max_age} min_confidence={self.min_confidence} '
            f'min_confirm_hits={self.min_confirm_hits} '
            f'embedder_gpu={self.embedder_gpu} '
            f'image_queue_size={self.image_queue_size} '
            f'max_frame_dt={self.max_frame_time_diff_ns / 1_000_000.0:.1f}ms '
            f'publish_prediction_tracks={self.publish_prediction_tracks}'
        )
        self.get_logger().info(
            f'External ID reacquire: topic={self.motor_angle_topic} '
            f'window={self.external_reacquire_window_sec:.1f}s '
            f'max_dist={self.external_reacquire_max_distance_px:.1f}px'
        )
        self.get_logger().info(
            f'ID cooldown: {self.id_cooldown_sec:.1f}s '
            f'(reacquire window: {self.external_reacquire_window_sec:.1f}s)'
        )
        if self.hold_class_ids and (
            self.hold_missing_frames > 0 or self.hold_missing_sec > 0.0
        ):
            self.get_logger().info(
                f'Track hold enabled: classes={sorted(self.hold_class_ids)} '
                f'missing_frames={self.hold_missing_frames} '
                f'missing_sec={self.hold_missing_sec:.1f}'
            )

    def on_image(self, msg: Image):
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f'imgmsg_to_cv2 failed: {e}')
            return
        self.image_cache[stamp_ns] = cv_image
        while len(self.image_cache) > self.image_cache_size:
            self.image_cache.popitem(last=False)

    def on_detections(self, msg: Detection2DArray):
        stamp_ns = self._stamp_to_ns(msg.stamp)
        frame = self.image_cache.get(stamp_ns)
        if frame is None:
            frame = self._find_closest_frame(stamp_ns)
        if frame is None:
            self.get_logger().warning(
                f'No image frame within {self.max_frame_time_diff_ns / 1_000_000:.0f}ms '
                f'of detection stamp, skipping'
            )
            return
        detections = self._to_deepsort_detections(msg.detections)
        tracks = self.tracker.update_tracks(detections, frame=frame)
        self._release_removed_external_ids()
        self._flush_cooldown_ids()
        self._publish_tracks(msg, tracks)

    def on_motor_angle(self, msg):
        self.current_motor_angle = (int(msg.pan), int(msg.tilt))

    def _find_closest_frame(self, stamp_ns: int):
        if not self.image_cache:
            return None
        closest_ns = min(self.image_cache.keys(), key=lambda k: abs(k - stamp_ns))
        if abs(closest_ns - stamp_ns) > self.max_frame_time_diff_ns:
            return None
        return self.image_cache[closest_ns]

    def _to_deepsort_detections(self, detections):
        result = []
        for det in detections:
            x1, y1, x2, y2 = float(det.x1), float(det.y1), float(det.x2), float(det.y2)
            if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
                continue
            if x2 <= x1 or y2 <= y1:
                continue
            score = float(det.score)
            if not math.isfinite(score):
                continue
            if score < self.min_confidence:
                continue
            class_id = int(det.class_id)
            self.class_names[class_id] = str(det.class_name)
            result.append(([x1, y1, x2 - x1, y2 - y1], score, class_id))
        return result

    def _publish_tracks(self, source_msg, tracks):
        msg = TrackedDetection2DArray()
        msg.stamp = source_msg.stamp
        msg.frame_id = source_msg.frame_id
        now = time.monotonic()
        active_internal_ids = {
            self._track_internal_id(t) for t in tracks if t.is_confirmed()
        }

        for track in tracks:
            if not track.is_confirmed():
                continue
            ltrb = self._track_ltrb_for_publish(track)
            if ltrb is None:
                continue
            l, t, r, b = map(float, ltrb)
            det_conf = float(track.det_conf) if track.det_conf is not None else -1.0
            if not all(math.isfinite(v) for v in (l, t, r, b, det_conf)):
                continue
            if r <= l or b <= t:
                continue

            internal_id = self._track_internal_id(track)
            class_id = self._track_class_id(track, internal_id)
            external_id = self._external_track_id(
                internal_id, (l, t, r, b), int(class_id), now, active_internal_ids
            )
            if external_id is None:
                continue

            self._remember_external_state(external_id, (l, t, r, b), int(class_id), now)
            out = TrackedDetection2D()
            out.track_id = int(external_id)
            out.class_id = int(class_id)
            out.class_name = self.class_names.get(int(class_id), str(class_id))
            out.score = det_conf
            out.x1 = l
            out.y1 = t
            out.x2 = r
            out.y2 = b
            msg.tracks.append(out)

        self._apply_track_hold(msg, now)
        self.pub.publish(msg)

    def _apply_track_hold(self, msg, now):
        if not self.hold_class_ids:
            return
        if self.hold_missing_frames <= 0 and self.hold_missing_sec <= 0.0:
            return

        active_track_ids = {int(track.track_id) for track in msg.tracks}
        for track in msg.tracks:
            if int(track.class_id) not in self.hold_class_ids:
                continue
            self.held_tracks[int(track.track_id)] = {
                'track': self._copy_track(track),
                'remaining': self.hold_missing_frames,
                'expires_at': now + self.hold_missing_sec,
            }

        for track_id, state in list(self.held_tracks.items()):
            if track_id in active_track_ids:
                continue
            remaining = int(state['remaining'])
            expires_at = float(state.get('expires_at', 0.0))
            if remaining <= 0 and now >= expires_at:
                self.held_tracks.pop(track_id, None)
                continue
            msg.tracks.append(self._copy_track(state['track'], stale=True))
            if remaining > 0:
                state['remaining'] = remaining - 1
            if state['remaining'] <= 0 and now >= expires_at:
                self.held_tracks.pop(track_id, None)

    def _track_ltrb_for_publish(self, track):
        if track.det_conf is not None:
            ltrb = track.to_ltrb(orig=True, orig_strict=True)
            if ltrb is not None:
                return ltrb
            return track.to_ltrb()
        if self.publish_prediction_tracks:
            return track.to_ltrb()
        return None

    @staticmethod
    def _copy_track(track, stale=False):
        out = TrackedDetection2D()
        out.track_id = int(track.track_id)
        out.class_id = int(track.class_id)
        out.class_name = str(track.class_name)
        score = float(track.score)
        if not math.isfinite(score):
            score = 0.0
        out.score = -max(abs(score), 1.0e-6) if stale else score
        out.x1 = float(track.x1)
        out.y1 = float(track.y1)
        out.x2 = float(track.x2)
        out.y2 = float(track.y2)
        return out

    def _external_track_id(self, internal_id, ltrb, class_id, now, active_internal_ids):
        mapped = self.internal_to_external_id.get(internal_id)
        if mapped is not None:
            return mapped

        reacquired = self._reacquire_external_track_id(
            ltrb, class_id, now, active_internal_ids
        )
        if reacquired is not None:
            self.released_id_cooldown.pop(reacquired, None)
            self._remap_external_track_id(internal_id, reacquired)
            return reacquired

        if not self.available_external_ids:
            self.get_logger().warning('No free external track IDs in range 1..254')
            return None
        external_id = self.available_external_ids.popleft()
        self.internal_to_external_id[internal_id] = external_id
        return external_id

    def _reacquire_external_track_id(self, ltrb, class_id, now, active_internal_ids):
        l, t, r, b = ltrb
        cx = float((l + r) * 0.5)
        cy = float((t + b) * 0.5)
        if not all(math.isfinite(v) for v in (cx, cy)):
            return None

        active_external_ids = {
            eid
            for iid, eid in self.internal_to_external_id.items()
            if iid in active_internal_ids
        }
        candidate_ids = (
            set(self.external_track_states.keys()) | set(self.released_id_cooldown.keys())
        )

        best_external_id = None
        best_score = None
        best_distance = None
        window = max(self.external_reacquire_window_sec, 1e-6)

        for external_id in candidate_ids:
            if external_id in active_external_ids:
                continue

            state = self.external_track_states.get(external_id)
            if state is None:
                continue

            age = now - float(state['last_seen'])
            if age > self.external_reacquire_window_sec:
                continue

            if self.external_reacquire_same_class:
                if int(state['class_id']) != int(class_id):
                    continue

            predicted_cx, predicted_cy = self._motor_compensated_center(state)
            if not all(math.isfinite(v) for v in (predicted_cx, predicted_cy)):
                continue
            distance = math.hypot(cx - predicted_cx, cy - predicted_cy)

            # Age-penalized score: older candidates ranked as if farther away
            age_ratio = min(1.0, age / window)
            score = distance * (1.0 + age_ratio)

            if best_score is None or score < best_score:
                best_score = score
                best_distance = distance
                best_external_id = external_id

        if best_external_id is None or best_distance is None:
            return None
        if best_distance > self.external_reacquire_max_distance_px:
            return None

        self._stats_reacquire_count += 1
        self.get_logger().info(
            f'Reacquired external track id={best_external_id} '
            f'dist={best_distance:.1f}px class={class_id}'
        )
        return best_external_id

    def _remap_external_track_id(self, internal_id, external_id):
        for old_iid, old_eid in list(self.internal_to_external_id.items()):
            if old_eid == external_id:
                self.internal_to_external_id.pop(old_iid)
        self.internal_to_external_id[internal_id] = external_id

    def _remember_external_state(self, external_id, ltrb, class_id, now):
        l, t, r, b = ltrb
        cx = float((l + r) * 0.5)
        cy = float((t + b) * 0.5)
        if not all(math.isfinite(v) for v in (cx, cy)):
            return
        self.external_track_states[external_id] = {
            'cx': cx,
            'cy': cy,
            'class_id': int(class_id),
            'motor_angle': self.current_motor_angle,
            'last_seen': now,
        }

    def _motor_compensated_center(self, state):
        if self.current_motor_angle is None or state.get('motor_angle') is None:
            return float(state['cx']), float(state['cy'])

        current_pan, current_tilt = self.current_motor_angle
        last_pan, last_tilt = state['motor_angle']
        delta_pan_counts = self._circular_delta(
            current_pan, last_pan, self.pan_wrap_counts
        )
        delta_tilt_counts = self._circular_delta(
            current_tilt, last_tilt, self.tilt_wrap_counts
        )
        delta_pan_rad = math.radians(delta_pan_counts / self.pan_counts_per_degree)
        delta_tilt_rad = math.radians(delta_tilt_counts / self.tilt_counts_per_degree)
        dx = self.pan_pixel_sign * self.camera_fx * math.tan(delta_pan_rad)
        dy = self.tilt_pixel_sign * self.camera_fy * math.tan(delta_tilt_rad)
        if not all(math.isfinite(v) for v in (dx, dy)):
            return float(state['cx']), float(state['cy'])
        return float(state['cx']) + dx, float(state['cy']) + dy

    def _release_removed_external_ids(self):
        live_internal_ids = self._live_internal_track_ids()
        if live_internal_ids is None:
            return

        removed = [
            iid for iid in self.internal_to_external_id
            if iid not in live_internal_ids
        ]
        now = time.monotonic()
        for internal_id in removed:
            self.internal_track_classes.pop(internal_id, None)
            external_id = self.internal_to_external_id.pop(internal_id)
            if external_id not in self.released_id_cooldown:
                self.released_id_cooldown[external_id] = now
                self.get_logger().debug(
                    f'External ID {external_id} entered cooldown '
                    f'({self.id_cooldown_sec:.1f}s)'
                )

    def _flush_cooldown_ids(self):
        now = time.monotonic()
        for external_id, released_at in list(self.released_id_cooldown.items()):
            if now - released_at >= self.id_cooldown_sec:
                self.released_id_cooldown.pop(external_id)
                self.external_track_states.pop(external_id, None)
                self.available_external_ids.append(external_id)
                self.get_logger().debug(
                    f'External ID {external_id} returned to pool after cooldown'
                )

    def _live_internal_track_ids(self):
        tracker = getattr(self.tracker, 'tracker', None)
        tracks = getattr(tracker, 'tracks', None)
        if tracks is None:
            return None
        return {
            self._track_internal_id(track)
            for track in tracks
            if not (hasattr(track, 'is_deleted') and track.is_deleted())
        }

    def _track_class_id(self, track, internal_id):
        if track.det_class is not None:
            class_id = int(track.det_class)
            self.internal_track_classes[internal_id] = class_id
            return class_id
        return self.internal_track_classes.get(internal_id, -1)

    def _log_stats(self):
        active_tracks = len(self.internal_to_external_id)
        held_count = len(self.held_tracks)
        cooldown_count = len(self.released_id_cooldown)
        self.get_logger().info(
            f'DeepSORT stats [{self.tracks_topic}]: '
            f'active={active_tracks} held={held_count} '
            f'cooldown={cooldown_count} '
            f'reacquire_total={self._stats_reacquire_count}'
        )

    @staticmethod
    def _track_internal_id(track):
        return str(track.track_id)

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _circular_delta(current, previous, wrap):
        half_wrap = float(wrap) * 0.5
        return ((float(current) - float(previous) + half_wrap) % float(wrap)) - half_wrap

    @staticmethod
    def _parse_class_ids(value):
        value = str(value).strip()
        if not value:
            return set()
        class_ids = set()
        for item in value.split(','):
            item = item.strip()
            if not item:
                continue
            class_ids.add(int(item))
        return class_ids


def main(args=None):
    rclpy.init(args=args)
    node = DeepSortTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
