from argparse import Namespace
import math
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sentinel_interfaces.msg import Detection2DArray
from sentinel_interfaces.msg import MotorAngle
from sentinel_interfaces.msg import TrackedDetection2D
from sentinel_interfaces.msg import TrackedDetection2DArray

from ultralytics.trackers.byte_tracker import BYTETracker


class ByteTrackDetections:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = np.asarray(xyxy, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(conf, dtype=np.float32)
        self.cls = np.asarray(cls, dtype=np.float32)

    def __len__(self):
        return int(self.conf.shape[0])

    def __getitem__(self, index):
        return ByteTrackDetections(self.xyxy[index], self.conf[index], self.cls[index])

    @property
    def xywh(self):
        if len(self) == 0:
            return np.empty((0, 4), dtype=np.float32)
        xywh = self.xyxy.copy()
        xywh[:, 0] = (self.xyxy[:, 0] + self.xyxy[:, 2]) * 0.5
        xywh[:, 1] = (self.xyxy[:, 1] + self.xyxy[:, 3]) * 0.5
        xywh[:, 2] = self.xyxy[:, 2] - self.xyxy[:, 0]
        xywh[:, 3] = self.xyxy[:, 3] - self.xyxy[:, 1]
        return xywh


class ByteTrackTrackerNode(Node):
    def __init__(self):
        super().__init__('bytetrack_tracker_node')

        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('tracks_topic', '/tracks')
        self.declare_parameter('high_score_threshold', 0.35)
        self.declare_parameter('low_score_threshold', 0.10)
        self.declare_parameter('match_iou_threshold', 0.30)
        self.declare_parameter('low_match_iou_threshold', 0.20)
        self.declare_parameter('track_buffer_frames', 30)
        self.declare_parameter('min_confirm_hits', 2)
        self.declare_parameter('class_aware_matching', False)
        self.declare_parameter('fuse_score', True)
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
        # [추가] 해제된 ID를 풀에 반환하기 전 대기 시간 (reacquire_window보다 길게 설정)
        self.declare_parameter('id_cooldown_sec', 6.0)

        self.detection_topic = (
            self.get_parameter('detection_topic').get_parameter_value().string_value
        )
        self.tracks_topic = (
            self.get_parameter('tracks_topic').get_parameter_value().string_value
        )
        self.high_score_threshold = float(
            self.get_parameter('high_score_threshold').get_parameter_value().double_value
        )
        self.low_score_threshold = float(
            self.get_parameter('low_score_threshold').get_parameter_value().double_value
        )
        self.match_iou_threshold = float(
            self.get_parameter('match_iou_threshold').get_parameter_value().double_value
        )
        self.low_match_iou_threshold = float(
            self.get_parameter('low_match_iou_threshold').get_parameter_value().double_value
        )
        self.track_buffer_frames = max(
            1,
            int(self.get_parameter('track_buffer_frames').get_parameter_value().integer_value),
        )
        self.min_confirm_hits = max(
            1,
            int(self.get_parameter('min_confirm_hits').get_parameter_value().integer_value),
        )
        self.class_aware_matching = (
            self.get_parameter('class_aware_matching').get_parameter_value().bool_value
        )
        self.fuse_score = self.get_parameter('fuse_score').get_parameter_value().bool_value
        self.motor_angle_topic = (
            self.get_parameter('motor_angle_topic').get_parameter_value().string_value
        )
        self.camera_fx = float(
            self.get_parameter('camera_fx').get_parameter_value().double_value
        )
        self.camera_fy = float(
            self.get_parameter('camera_fy').get_parameter_value().double_value
        )
        self.pan_counts_per_degree = max(
            1.0e-6,
            float(
                self.get_parameter(
                    'pan_counts_per_degree'
                ).get_parameter_value().double_value
            ),
        )
        self.tilt_counts_per_degree = max(
            1.0e-6,
            float(
                self.get_parameter(
                    'tilt_counts_per_degree'
                ).get_parameter_value().double_value
            ),
        )
        self.pan_wrap_counts = max(
            1.0,
            float(self.get_parameter('pan_wrap_counts').get_parameter_value().double_value),
        )
        self.tilt_wrap_counts = max(
            1.0,
            float(self.get_parameter('tilt_wrap_counts').get_parameter_value().double_value),
        )
        self.pan_pixel_sign = float(
            self.get_parameter('pan_pixel_sign').get_parameter_value().double_value
        )
        self.tilt_pixel_sign = float(
            self.get_parameter('tilt_pixel_sign').get_parameter_value().double_value
        )
        self.external_reacquire_window_sec = float(
            self.get_parameter(
                'external_reacquire_window_sec'
            ).get_parameter_value().double_value
        )
        self.external_reacquire_max_distance_px = float(
            self.get_parameter(
                'external_reacquire_max_distance_px'
            ).get_parameter_value().double_value
        )
        self.external_reacquire_same_class = (
            self.get_parameter(
                'external_reacquire_same_class'
            ).get_parameter_value().bool_value
        )
        self.hold_class_ids = self._parse_class_ids(
            self.get_parameter('hold_class_ids').get_parameter_value().string_value
        )
        self.hold_missing_frames = max(
            0,
            int(self.get_parameter('hold_missing_frames').get_parameter_value().integer_value),
        )
        self.hold_missing_sec = max(
            0.0,
            float(self.get_parameter('hold_missing_sec').get_parameter_value().double_value),
        )
        # [추가] id_cooldown_sec: reacquire_window_sec보다 반드시 길어야 함
        self.id_cooldown_sec = max(
            self.external_reacquire_window_sec + 1.0,
            float(self.get_parameter('id_cooldown_sec').get_parameter_value().double_value),
        )

        self.tracker = BYTETracker(self._tracker_args())
        self.class_names: dict[int, str] = {}
        self.internal_to_external_id: dict[int, int] = {}
        self.external_track_states: dict[int, dict] = {}
        self.held_tracks: dict[int, dict] = {}
        self.current_motor_angle = None
        self.available_external_ids = list(range(1, 255))

        # [추가] 해제된 external ID의 cooldown 관리
        # { external_id: released_at(monotonic) }
        self.released_id_cooldown: dict[int, float] = {}

        self.pub = self.create_publisher(TrackedDetection2DArray, self.tracks_topic, 10)
        self.sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.on_detections,
            10,
        )
        self.motor_angle_sub = self.create_subscription(
            MotorAngle,
            self.motor_angle_topic,
            self.on_motor_angle,
            10,
        )

        self.get_logger().info(
            f'ByteTrack tracker started: {self.detection_topic} -> {self.tracks_topic}'
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

    def _tracker_args(self):
        return Namespace(
            track_high_thresh=self.high_score_threshold,
            track_low_thresh=self.low_score_threshold,
            new_track_thresh=self.high_score_threshold,
            track_buffer=self.track_buffer_frames,
            match_thresh=1.0 - self.match_iou_threshold,
            fuse_score=self.fuse_score,
        )

    def on_detections(self, msg):
        detections = self._to_bytetrack_detections(msg.detections)
        tracks = self.tracker.update(detections)
        self._release_removed_external_ids()
        # [추가] cooldown 만료된 ID를 풀에 반환
        self._flush_cooldown_ids()
        self._publish_tracks(msg, tracks)

    def on_motor_angle(self, msg):
        self.current_motor_angle = (int(msg.pan), int(msg.tilt))

    def _to_bytetrack_detections(self, detections):
        boxes = []
        scores = []
        classes = []
        for det in detections:
            x1 = float(det.x1)
            y1 = float(det.y1)
            x2 = float(det.x2)
            y2 = float(det.y2)
            if x2 <= x1 or y2 <= y1:
                continue
            score = float(det.score)
            if score < self.low_score_threshold:
                continue
            class_id = int(det.class_id)
            if self.class_aware_matching:
                class_id = int(det.class_id)
            self.class_names[class_id] = str(det.class_name)
            boxes.append((x1, y1, x2, y2))
            scores.append(score)
            classes.append(class_id)
        return ByteTrackDetections(boxes, scores, classes)

    def _publish_tracks(self, source_msg, tracks):
        msg = TrackedDetection2DArray()
        msg.stamp = source_msg.stamp
        msg.frame_id = source_msg.frame_id
        now = time.monotonic()
        active_internal_ids = {
            int(track[4]) for track in tracks
            if len(track) >= 7
        }

        for track in tracks:
            if len(track) < 7:
                continue
            x1, y1, x2, y2 = (float(v) for v in track[:4])
            if x2 <= x1 or y2 <= y1:
                continue

            internal_id = int(track[4])
            external_id = self._external_track_id(
                internal_id,
                track,
                now,
                active_internal_ids,
            )
            if external_id is None:
                continue

            class_id = int(track[6])
            self._remember_external_state(external_id, track, now)
            out = TrackedDetection2D()
            out.track_id = int(external_id)
            out.class_id = class_id
            out.class_name = self.class_names.get(class_id, str(class_id))
            out.score = float(track[5])
            out.x1 = x1
            out.y1 = y1
            out.x2 = x2
            out.y2 = y2
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

    @staticmethod
    def _copy_track(track, stale=False):
        out = TrackedDetection2D()
        out.track_id = int(track.track_id)
        out.class_id = int(track.class_id)
        out.class_name = str(track.class_name)
        score = float(track.score)
        out.score = -max(abs(score), 1.0e-6) if stale else score
        out.x1 = float(track.x1)
        out.y1 = float(track.y1)
        out.x2 = float(track.x2)
        out.y2 = float(track.y2)
        return out

    def _external_track_id(self, internal_id, track, now, active_internal_ids):
        # 이미 매핑된 internal ID면 그대로 반환
        mapped = self.internal_to_external_id.get(internal_id)
        if mapped is not None:
            return mapped

        # [방법 3] 새 ID 할당 전에 항상 reacquire를 먼저 시도
        # cooldown 중인 ID도 reacquire 후보로 사용 가능
        reacquired = self._reacquire_external_track_id(
            track,
            now,
            active_internal_ids,
        )
        if reacquired is not None:
            # reacquire 성공 → cooldown에서 제거하고 즉시 재사용
            self.released_id_cooldown.pop(reacquired, None)
            self._remap_external_track_id(internal_id, reacquired)
            return reacquired

        # [방법 2] reacquire 실패 → cooldown이 끝난 ID만 새로 할당
        # available_external_ids는 cooldown 만료 후에만 채워지므로
        # 여기서 꺼내는 ID는 충분히 오래된 것만 해당됨
        if not self.available_external_ids:
            self.get_logger().warning('No free external track IDs in range 1..254')
            return None
        external_id = self.available_external_ids.pop(0)
        self.internal_to_external_id[internal_id] = external_id
        return external_id

    def _reacquire_external_track_id(self, track, now, active_internal_ids):
        class_id = int(track[6])
        cx = float((track[0] + track[2]) * 0.5)
        cy = float((track[1] + track[3]) * 0.5)

        # 현재 활성 트랙이 이미 사용 중인 external ID 집합
        active_external_ids = {
            external_id
            for internal_id, external_id in self.internal_to_external_id.items()
            if internal_id in active_internal_ids
        }

        # [변경] reacquire 후보 범위를 external_track_states + cooldown 중인 ID 모두 포함
        # cooldown 중인 ID는 아직 풀에 반환되지 않았으므로 재사용 가능한 상태
        candidate_ids = set(self.external_track_states.keys()) | set(self.released_id_cooldown.keys())

        best_external_id = None
        best_distance = None
        for external_id in candidate_ids:
            # 이미 다른 트랙이 쓰고 있으면 스킵
            if external_id in active_external_ids:
                continue

            state = self.external_track_states.get(external_id)
            if state is None:
                continue

            age = now - float(state['last_seen'])
            if age > self.external_reacquire_window_sec:
                continue

            if self.external_reacquire_same_class:
                if int(state['class_id']) != class_id:
                    continue

            predicted_cx, predicted_cy = self._motor_compensated_center(state)
            distance = math.hypot(cx - predicted_cx, cy - predicted_cy)
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_external_id = external_id

        if best_external_id is None or best_distance is None:
            return None
        if best_distance > self.external_reacquire_max_distance_px:
            return None

        self.get_logger().info(
            f'Reacquired external track id={best_external_id} '
            f'dist={best_distance:.1f}px class={class_id}'
        )
        return best_external_id

    def _remap_external_track_id(self, internal_id, external_id):
        for old_internal_id, old_external_id in list(self.internal_to_external_id.items()):
            if old_external_id == external_id:
                self.internal_to_external_id.pop(old_internal_id)
        self.internal_to_external_id[internal_id] = external_id

    def _remember_external_state(self, external_id, track, now):
        self.external_track_states[external_id] = {
            'cx': float((track[0] + track[2]) * 0.5),
            'cy': float((track[1] + track[3]) * 0.5),
            'class_id': int(track[6]),
            'motor_angle': self.current_motor_angle,
            'last_seen': now,
        }

    def _motor_compensated_center(self, state):
        if self.current_motor_angle is None or state.get('motor_angle') is None:
            return float(state['cx']), float(state['cy'])

        current_pan, current_tilt = self.current_motor_angle
        last_pan, last_tilt = state['motor_angle']
        delta_pan_counts = self._circular_delta(
            current_pan,
            last_pan,
            self.pan_wrap_counts,
        )
        delta_tilt_counts = self._circular_delta(
            current_tilt,
            last_tilt,
            self.tilt_wrap_counts,
        )
        delta_pan_rad = math.radians(delta_pan_counts / self.pan_counts_per_degree)
        delta_tilt_rad = math.radians(delta_tilt_counts / self.tilt_counts_per_degree)
        dx = self.pan_pixel_sign * self.camera_fx * math.tan(delta_pan_rad)
        dy = self.tilt_pixel_sign * self.camera_fy * math.tan(delta_tilt_rad)
        return float(state['cx']) + dx, float(state['cy']) + dy

    def _release_removed_external_ids(self):
        """
        ByteTrack에서 완전히 제거된 트랙의 internal ID를 정리한다.
        [변경] external ID를 풀에 즉시 반환하지 않고 cooldown 딕셔너리로 옮긴다.
        cooldown 기간 동안은 reacquire에만 사용되고, 새 객체에는 할당되지 않는다.
        """
        live_internal_ids = {
            int(track.track_id)
            for track in self.tracker.tracked_stracks + self.tracker.lost_stracks
        }
        removed_internal_ids = [
            internal_id for internal_id in self.internal_to_external_id
            if internal_id not in live_internal_ids
        ]
        now = time.monotonic()
        for internal_id in removed_internal_ids:
            external_id = self.internal_to_external_id.pop(internal_id)
            # [변경] 풀에 즉시 반환하는 대신 cooldown 등록
            if external_id not in self.released_id_cooldown:
                self.released_id_cooldown[external_id] = now
                self.get_logger().debug(
                    f'External ID {external_id} entered cooldown '
                    f'({self.id_cooldown_sec:.1f}s)'
                )

    def _flush_cooldown_ids(self):
        """
        [추가] cooldown이 만료된 external ID를 풀에 반환한다.
        이 시점 이후에야 해당 ID가 새 객체에 할당될 수 있다.
        """
        now = time.monotonic()
        for external_id, released_at in list(self.released_id_cooldown.items()):
            if now - released_at >= self.id_cooldown_sec:
                self.released_id_cooldown.pop(external_id)
                self.external_track_states.pop(external_id, None)
                self.available_external_ids.append(external_id)
                self.get_logger().debug(
                    f'External ID {external_id} returned to pool after cooldown'
                )

    @staticmethod
    def _circular_delta(current, previous, wrap):
        half_wrap = float(wrap) * 0.5
        return ((float(current) - float(previous) + half_wrap) % float(wrap)) - half_wrap

    @staticmethod
    def _parse_class_ids(value):
        value = value.strip()
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
    node = ByteTrackTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
