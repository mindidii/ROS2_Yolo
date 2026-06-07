import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from sentinel_interfaces.msg import Detection
from sentinel_interfaces.msg import FrameInfo
from sentinel_interfaces.msg import TrackedDetection2DArray

MODE_SCAN = 0
MODE_MANUAL = 1
MODE_TRACKING = 2
TRACK_ID_AUTO = 0xFF
STREAM_EO = 0
STREAM_IR = 1

LOST_HOLD_SEC = 3.0


class TrackSelectorNode(Node):
    def __init__(self):
        super().__init__('track_selector_node')

        self.declare_parameter('tracks_topic', '/tracks/eo')
        self.declare_parameter('tracks_topic_eo', '/tracks/eo')
        self.declare_parameter('tracks_topic_ir', '/tracks/ir')
        self.declare_parameter('system_mode_topic', '/system/mode')
        self.declare_parameter('system_track_id_topic', '/system/track_id')
        self.declare_parameter('stream_select_topic', '/system/stream_select')
        self.declare_parameter('driver_detection_topic', '/driver/detection')
        self.declare_parameter('frame_info_topic', '/camera/eo/raw/frame_info')
        self.declare_parameter('frame_info_topic_eo', '/camera/eo/raw/frame_info')
        self.declare_parameter('frame_info_topic_ir', '/camera/ir/frame_info')
        self.declare_parameter('auto_select_policy', 'first_detected')
        self.declare_parameter('lost_hold_sec', LOST_HOLD_SEC)
        self.declare_parameter('bbox_output_hold_sec', 0.25)

        legacy_tracks_topic = self.get_parameter('tracks_topic').get_parameter_value().string_value
        self.tracks_topic_eo = self.get_parameter('tracks_topic_eo').get_parameter_value().string_value
        self.tracks_topic_ir = self.get_parameter('tracks_topic_ir').get_parameter_value().string_value
        if self.tracks_topic_eo == '/tracks/eo' and legacy_tracks_topic != '/tracks/eo':
            self.tracks_topic_eo = legacy_tracks_topic
        self.system_mode_topic = self.get_parameter('system_mode_topic').get_parameter_value().string_value
        self.system_track_id_topic = self.get_parameter('system_track_id_topic').get_parameter_value().string_value
        self.stream_select_topic = self.get_parameter('stream_select_topic').get_parameter_value().string_value
        self.driver_detection_topic = self.get_parameter('driver_detection_topic').get_parameter_value().string_value
        legacy_frame_info_topic = self.get_parameter('frame_info_topic').get_parameter_value().string_value
        self.frame_info_topic_eo = self.get_parameter('frame_info_topic_eo').get_parameter_value().string_value
        self.frame_info_topic_ir = self.get_parameter('frame_info_topic_ir').get_parameter_value().string_value
        if self.frame_info_topic_eo == '/camera/eo/raw/frame_info' and legacy_frame_info_topic != '/camera/eo/raw/frame_info':
            self.frame_info_topic_eo = legacy_frame_info_topic
        self.auto_select_policy = self.get_parameter('auto_select_policy').get_parameter_value().string_value
        self.lost_hold_sec = float(
            self.get_parameter('lost_hold_sec').get_parameter_value().double_value
        )
        self.bbox_output_hold_sec = max(
            0.0,
            float(self.get_parameter('bbox_output_hold_sec').get_parameter_value().double_value),
        )

        # 상태값
        self.system_mode = MODE_SCAN
        self.track_enabled = False
        self.selected_stream = STREAM_EO
        self.requested_track_id = TRACK_ID_AUTO
        self.selected_track_ids = {STREAM_EO: None, STREAM_IR: None}
        self.latest_tracks = {STREAM_EO: [], STREAM_IR: []}
        self.frame_sizes = {STREAM_EO: None, STREAM_IR: None}
        self._debug_counter = 0

        # 트랙 소실 관련 상태
        self._lost_since = {STREAM_EO: None, STREAM_IR: None}
        self._is_holding = {STREAM_EO: False, STREAM_IR: False}
        self._last_fresh_selected = {STREAM_EO: None, STREAM_IR: None}
        self._last_fresh_selected_time = {STREAM_EO: None, STREAM_IR: None}

        # [제거] motor compensation 관련 파라미터/상태 전부 삭제
        # current_motor_angle, last_target_state, camera_fx/fy,
        # pan/tilt 파라미터, motor_reacquire_* 파라미터 모두 제거.
        # ID 연속성은 bytetrack_tracker_node가 전담한다.

        self.tracks_eo_sub = self.create_subscription(
            TrackedDetection2DArray, self.tracks_topic_eo,
            lambda msg: self.on_tracks(msg, STREAM_EO), 10)
        self.tracks_ir_sub = self.create_subscription(
            TrackedDetection2DArray, self.tracks_topic_ir,
            lambda msg: self.on_tracks(msg, STREAM_IR), 10)
        self.mode_sub = self.create_subscription(
            UInt8, self.system_mode_topic, self.on_system_mode, 10)
        self.track_id_sub = self.create_subscription(
            UInt8, self.system_track_id_topic, self.on_system_track_id, 10)
        self.stream_select_sub = self.create_subscription(
            UInt8, self.stream_select_topic, self.on_stream_select, 10)
        self.frame_info_eo_sub = self.create_subscription(
            FrameInfo,
            self.frame_info_topic_eo,
            lambda msg: self.on_frame_info(msg, STREAM_EO),
            10,
        )
        self.frame_info_ir_sub = self.create_subscription(
            FrameInfo,
            self.frame_info_topic_ir,
            lambda msg: self.on_frame_info(msg, STREAM_IR),
            10,
        )
        self.cmd_track_sub = self.create_subscription(
            Bool, '/system/cmd_track', self.on_cmd_track, 10)
        self.driver_detection_pub = self.create_publisher(
            Detection, self.driver_detection_topic, 10)
        self.active_track_id_pub = self.create_publisher(
            UInt8, '/system/active_track_id', 10)

        self.get_logger().info(
            f'TrackSelectorNode started: lost_hold_sec={self.lost_hold_sec}s '
            f'bbox_output_hold_sec={self.bbox_output_hold_sec}s '
            f'frame_info_eo={self.frame_info_topic_eo} '
            f'frame_info_ir={self.frame_info_topic_ir}'
        )
        self.get_logger().info(
            'ID reacquire 책임: bytetrack_tracker_node 전담 '
            '(track_selector는 ID를 신뢰하고 추종)'
        )

    # =========================================================
    # 콜백
    # =========================================================
    def on_system_mode(self, msg):
        mode = int(msg.data)
        if mode not in (MODE_SCAN, MODE_MANUAL, MODE_TRACKING):
            self.get_logger().warning(f'Ignoring invalid system mode: {mode}')
            return
        self.system_mode = mode
        if self.system_mode != MODE_TRACKING:
            self._clear_selected_track_ids()

    def on_cmd_track(self, msg):
        prev = self.track_enabled
        self.track_enabled = msg.data
        if prev != self.track_enabled:
            self.get_logger().info(f'Track: {"ON" if self.track_enabled else "OFF"}')
        if self.track_enabled:
            self._publish_selected_from_cache()
        else:
            self._clear_selected_track_ids()

    def on_system_track_id(self, msg):
        track_id = int(msg.data)
        if track_id != TRACK_ID_AUTO and not (0 <= track_id <= 254):
            self.get_logger().warning(f'Ignoring invalid track id: {track_id}')
            return

        self.requested_track_id = track_id
        self._reset_hold_state(self.selected_stream)

        if track_id == TRACK_ID_AUTO:
            self.selected_track_ids[self.selected_stream] = None
            self.get_logger().info(
                f'Requested track: AUTO '
                f'(stream={self._stream_name(self.selected_stream)})'
            )
        else:
            self.selected_track_ids[self.selected_stream] = track_id
            self.get_logger().info(
                f'Requested track id: {track_id} '
                f'(stream={self._stream_name(self.selected_stream)})'
            )

        self._publish_selected_from_cache()

    def on_stream_select(self, msg):
        stream = int(msg.data)
        if stream not in (STREAM_EO, STREAM_IR):
            self.get_logger().warning(f'Ignoring invalid stream select: {stream}')
            return
        if stream == self.selected_stream:
            return
        self.selected_stream = stream
        if self.requested_track_id == TRACK_ID_AUTO:
            self.selected_track_ids[self.selected_stream] = None
        self.get_logger().info(f'Selected stream: {self._stream_name(self.selected_stream)}')
        self._publish_selected_from_cache()

    def on_frame_info(self, msg, stream):
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            self.get_logger().warning(
                f'Ignoring invalid frame_info size: width={width}, height={height} '
                f'stream={self._stream_name(stream)}'
            )
            return
        self.frame_sizes[stream] = (width, height)

    def on_tracks(self, msg, stream):
        self.latest_tracks[stream] = list(msg.tracks)
        if stream == self.selected_stream:
            self._debug_counter += 1
            if self._debug_counter % 30 == 1:
                ids = [int(t.track_id) for t in msg.tracks]
                self.get_logger().info(
                    f'[DBG] stream={self._stream_name(stream)} '
                    f'n={len(msg.tracks)} ids={ids} '
                    f'holding={self._is_holding[stream]}'
                )
        if stream != self.selected_stream:
            return
        self._publish_selected(msg.tracks, stream)

    def _publish_selected_from_cache(self):
        if not self.track_enabled:
            return
        self._publish_selected(
            self.latest_tracks.get(self.selected_stream, []),
            self.selected_stream,
        )

    # =========================================================
    # 핵심: _publish_selected
    # bytetrack_tracker_node가 ID 연속성을 보장하므로
    # 여기서는 ID를 신뢰하고 추종만 한다.
    # ID가 사라지면 → hold → 초과 시 추적 중단.
    # 다른 ID로 임의 전환하는 로직은 없다.
    # =========================================================
    def _publish_selected(self, tracks, stream):
        if not self.track_enabled:
            return

        selected = self._select_track(tracks, stream)

        if selected is None:
            if self.selected_track_ids.get(stream) is None:
                self._publish_active_track_id(TRACK_ID_AUTO)
                return

            # ── 트랙 소실 처리 ──
            if not self._is_holding[stream]:
                self._lost_since[stream] = time.monotonic()
                self._is_holding[stream] = True
                self.get_logger().info(
                    f'[HOLD] 트랙 소실 감지 '
                    f'(stream={self._stream_name(stream)}), '
                    f'{self.lost_hold_sec}초 대기 시작'
                )

            elapsed = time.monotonic() - self._lost_since[stream]

            if elapsed < self.lost_hold_sec:
                # 대기 중: 발행 중단 → 모터는 마지막 명령 위치 유지
                return

            # 대기 시간 초과 → 진짜 소실로 확정, 추적 중단
            self.get_logger().info(
                f'[HOLD] 대기 시간 초과 '
                f'(stream={self._stream_name(stream)}), 추적 중단'
            )
            self._is_holding[stream] = False
            self._lost_since[stream] = None
            self.selected_track_ids[stream] = None
            self._publish_active_track_id(TRACK_ID_AUTO)
            return

        now = time.monotonic()
        selected_is_fresh = self._track_is_fresh(selected)

        if not selected_is_fresh:
            if not self._is_holding[stream]:
                self._lost_since[stream] = now
                self._is_holding[stream] = True
                self.get_logger().info(
                    f'[HOLD] 트랙 stale 상태 진입 '
                    f'(stream={self._stream_name(stream)}), '
                    f'{self.lost_hold_sec}초 ID 유지 시작'
                )

            elapsed = now - self._lost_since[stream]
            if elapsed >= self.lost_hold_sec:
                self.get_logger().info(
                    f'[HOLD] stale 유지 시간 초과 '
                    f'(stream={self._stream_name(stream)}), 추적 중단'
                )
                self._is_holding[stream] = False
                self._lost_since[stream] = None
                self.selected_track_ids[stream] = None
                self._publish_active_track_id(TRACK_ID_AUTO)
                return

        # ── 트랙 재발견 ──
        if self._is_holding[stream] and selected_is_fresh:
            elapsed = time.monotonic() - self._lost_since[stream]
            self.get_logger().info(
                f'[HOLD] 트랙 재발견 '
                f'(stream={self._stream_name(stream)}, '
                f'소실 후 {elapsed:.1f}s), 추적 재개'
            )
            self._is_holding[stream] = False
            self._lost_since[stream] = None

        if not self._track_has_valid_bbox(selected):
            self.get_logger().warning(
                f'[DBG] invalid bbox: ({selected.x1},{selected.y1})'
                f'-({selected.x2},{selected.y2})'
            )
            return

        if selected_is_fresh:
            self._last_fresh_selected[stream] = selected
            self._last_fresh_selected_time[stream] = now
            self._publish_driver_detection(selected, stream)
        else:
            self._publish_driver_detection_if_bbox_hold_active(stream, now)

        self._publish_active_track_id(int(selected.track_id))

    # =========================================================
    # 트랙 선택
    # =========================================================
    def _select_track(self, tracks, stream):
        valid_tracks = [t for t in tracks if self._track_is_valid_candidate(t)]
        if not valid_tracks:
            return None

        # 명시적 ID 요청
        if self.requested_track_id != TRACK_ID_AUTO:
            for track in valid_tracks:
                if int(track.track_id) == self.requested_track_id:
                    self.selected_track_ids[stream] = int(track.track_id)
                    return track
            return None

        # 기존 선택 ID 유지
        selected_track_id = self.selected_track_ids.get(stream)
        if selected_track_id is not None:
            for track in valid_tracks:
                if int(track.track_id) == selected_track_id:
                    return track
            # ID가 목록에 없으면 None 반환 → 소실 처리로 넘어감
            # bytetrack reacquire가 성공했다면 다음 프레임에 같은 ID로 돌아옴
            return None

        # 신규 자동 선택
        fresh_tracks = [t for t in valid_tracks if self._track_is_fresh(t)]
        if not fresh_tracks:
            return None
        selected = self._auto_select(fresh_tracks)
        self.selected_track_ids[stream] = int(selected.track_id)
        return selected

    def _auto_select(self, tracks):
        if self.auto_select_policy == 'largest_area':
            return max(
                tracks,
                key=lambda t: max(0.0, float(t.x2 - t.x1)) * max(0.0, float(t.y2 - t.y1)),
            )
        if self.auto_select_policy == 'highest_score':
            return max(tracks, key=lambda t: float(t.score))
        return tracks[0]

    # =========================================================
    # 유틸
    # =========================================================
    def _publish_active_track_id(self, track_id):
        msg = UInt8()
        msg.data = track_id
        self.active_track_id_pub.publish(msg)

    def _publish_driver_detection(self, track, stream):
        bbox = self._clamped_bbox(track, stream)
        if bbox is None:
            self.get_logger().warning(
                f'[DBG] bbox outside frame, skip driver detection: '
                f'({track.x1},{track.y1})-({track.x2},{track.y2}) '
                f'frame={self.frame_sizes.get(stream)} '
                f'stream={self._stream_name(stream)}'
            )
            return

        x1, y1, x2, y2 = bbox
        cx = float((x1 + x2) / 2.0)
        cy = float((y1 + y2) / 2.0)
        if cx < 50.0 or cx > 1230.0:
            return
        out = Detection()
        out.cx = cx
        out.cy = cy
        self.driver_detection_pub.publish(out)

    def _clamped_bbox(self, track, stream):
        try:
            x1 = float(track.x1)
            y1 = float(track.y1)
            x2 = float(track.x2)
            y2 = float(track.y2)
        except (TypeError, ValueError):
            return None

        if not all(map(math.isfinite, (x1, y1, x2, y2))):
            return None

        frame_size = self.frame_sizes.get(stream)
        if frame_size is not None:
            frame_width, frame_height = frame_size
            max_x = max(0.0, float(frame_width) - 1.0)
            max_y = max(0.0, float(frame_height) - 1.0)
            x1 = min(max(x1, 0.0), max_x)
            x2 = min(max(x2, 0.0), max_x)
            y1 = min(max(y1, 0.0), max_y)
            y2 = min(max(y2, 0.0), max_y)

        if x2 <= x1 or y2 <= y1:
            return None
        return x1, y1, x2, y2

    def _publish_driver_detection_if_bbox_hold_active(self, stream, now):
        last_track = self._last_fresh_selected.get(stream)
        last_seen = self._last_fresh_selected_time.get(stream)
        if last_track is None or last_seen is None:
            return
        if now - last_seen > self.bbox_output_hold_sec:
            return
        self._publish_driver_detection(last_track, stream)

    def _clear_selected_track_ids(self):
        self.selected_track_ids = {STREAM_EO: None, STREAM_IR: None}
        self._lost_since = {STREAM_EO: None, STREAM_IR: None}
        self._is_holding = {STREAM_EO: False, STREAM_IR: False}
        self._last_fresh_selected = {STREAM_EO: None, STREAM_IR: None}
        self._last_fresh_selected_time = {STREAM_EO: None, STREAM_IR: None}
        self._publish_active_track_id(TRACK_ID_AUTO)

    def _reset_hold_state(self, stream):
        self._lost_since[stream] = None
        self._is_holding[stream] = False
        self._last_fresh_selected[stream] = None
        self._last_fresh_selected_time[stream] = None

    @staticmethod
    def _track_is_fresh(track):
        score = float(track.score)
        return math.isfinite(score) and score >= 0.0

    @staticmethod
    def _track_is_valid_candidate(track):
        try:
            track_id = int(track.track_id)
        except (TypeError, ValueError):
            return False
        return 0 <= track_id <= 254 and TrackSelectorNode._track_has_valid_bbox(track)

    @staticmethod
    def _track_has_valid_bbox(track):
        try:
            x1 = float(track.x1)
            y1 = float(track.y1)
            x2 = float(track.x2)
            y2 = float(track.y2)
        except (TypeError, ValueError):
            return False
        if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
            return False
        return x2 > x1 and y2 > y1

    @staticmethod
    def _stream_name(stream):
        return 'EO' if stream == STREAM_EO else 'IR'


def main(args=None):
    rclpy.init(args=args)
    node = TrackSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
