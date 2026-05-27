import time  # 추가

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from sentinel_interfaces.msg import Detection
from sentinel_interfaces.msg import TrackedDetection2DArray

MODE_SCAN = 0
MODE_MANUAL = 1
MODE_TRACKING = 2
TRACK_ID_AUTO = 0xFF
STREAM_EO = 0
STREAM_IR = 1

LOST_HOLD_SEC = 3.0  # 추가: 트랙 소실 후 대기 시간


class TrackSelectorNode(Node):
    def __init__(self):
        super().__init__('track_selector_node')

        # 기존 파라미터 선언 동일
        self.declare_parameter('tracks_topic', '/tracks/eo')
        self.declare_parameter('tracks_topic_eo', '/tracks/eo')
        self.declare_parameter('tracks_topic_ir', '/tracks/ir')
        self.declare_parameter('system_mode_topic', '/system/mode')
        self.declare_parameter('system_track_id_topic', '/system/track_id')
        self.declare_parameter('stream_select_topic', '/system/stream_select')
        self.declare_parameter('driver_detection_topic', '/driver/detection')
        self.declare_parameter('auto_select_policy', 'first_detected')
        self.declare_parameter('lost_hold_sec', LOST_HOLD_SEC)  # 추가

        # 기존 파라미터 읽기 동일
        legacy_tracks_topic = self.get_parameter('tracks_topic').get_parameter_value().string_value
        self.tracks_topic_eo = self.get_parameter('tracks_topic_eo').get_parameter_value().string_value
        self.tracks_topic_ir = self.get_parameter('tracks_topic_ir').get_parameter_value().string_value
        if self.tracks_topic_eo == '/tracks/eo' and legacy_tracks_topic != '/tracks/eo':
            self.tracks_topic_eo = legacy_tracks_topic
        self.system_mode_topic = self.get_parameter('system_mode_topic').get_parameter_value().string_value
        self.system_track_id_topic = self.get_parameter('system_track_id_topic').get_parameter_value().string_value
        self.stream_select_topic = self.get_parameter('stream_select_topic').get_parameter_value().string_value
        self.driver_detection_topic = self.get_parameter('driver_detection_topic').get_parameter_value().string_value
        self.auto_select_policy = self.get_parameter('auto_select_policy').get_parameter_value().string_value
        self.lost_hold_sec = float(  # 추가
            self.get_parameter('lost_hold_sec').get_parameter_value().double_value
        )

        # 기존 상태값
        self.system_mode = MODE_SCAN
        self.track_enabled = False
        self.selected_stream = STREAM_EO
        self.requested_track_id = TRACK_ID_AUTO
        self.selected_track_ids = {STREAM_EO: None, STREAM_IR: None}
        self.latest_tracks = {STREAM_EO: [], STREAM_IR: []}
        self._debug_counter = 0

        # 추가: 트랙 소실 관련 상태
        self._lost_since = {
            STREAM_EO: None,  # 소실 시작 시각 (None이면 소실 아님)
            STREAM_IR: None,
        }
        self._is_holding = {
            STREAM_EO: False,  # 대기 중인지 여부
            STREAM_IR: False,
        }

        # 기존 구독/발행 동일
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
        self.cmd_track_sub = self.create_subscription(
            Bool, '/system/cmd_track', self.on_cmd_track, 10)
        self.driver_detection_pub = self.create_publisher(
            Detection, self.driver_detection_topic, 10)
        self.active_track_id_pub = self.create_publisher(
            UInt8, '/system/active_track_id', 10)

        self.get_logger().info(
            f'TrackSelectorNode started: lost_hold_sec={self.lost_hold_sec}s'
        )

    # =========================================================
    # 기존 콜백 (변경 없음)
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
        self.requested_track_id = track_id
        if track_id == TRACK_ID_AUTO:
            self.selected_track_ids[self.selected_stream] = None
        else:
            self.selected_track_ids[self.selected_stream] = track_id
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
            self.selected_stream
        )

    # =========================================================
    # 핵심 수정: _publish_selected
    # =========================================================
    def _publish_selected(self, tracks, stream):
        if not self.track_enabled:
            return

        selected = self._select_track(tracks, stream)

        if selected is None:
            # ── 트랙 소실 처리 ──
            if not self._is_holding[stream]:
                # 소실 시작
                self._lost_since[stream] = time.monotonic()
                self._is_holding[stream] = True
                self.get_logger().info(
                    f'[HOLD] 트랙 소실 감지, {self.lost_hold_sec}초 대기 시작'
                )

            elapsed = time.monotonic() - self._lost_since[stream]

            if elapsed < self.lost_hold_sec:
                # 대기 중: 발행 중단 (모터 정지 유지)
                self.get_logger().info(
                    f'[HOLD] 대기 중 {elapsed:.1f}s / {self.lost_hold_sec}s',
                )
                return

            # 대기 시간 초과: 새 트랙 탐색 허용
            self.get_logger().info('[HOLD] 대기 시간 초과, 새 트랙 탐색')
            self._is_holding[stream] = False
            self._lost_since[stream] = None
            self.selected_track_ids[stream] = None
            self._publish_active_track_id(TRACK_ID_AUTO)
            return

        # ── 트랙 재발견 ──
        if self._is_holding[stream]:
            elapsed = time.monotonic() - self._lost_since[stream]
            self.get_logger().info(
                f'[HOLD] 트랙 재발견 (소실 후 {elapsed:.1f}s), 추적 재개'
            )
            self._is_holding[stream] = False
            self._lost_since[stream] = None

        # bbox 유효성 검사
        if selected.x2 <= selected.x1 or selected.y2 <= selected.y1:
            self.get_logger().warning(
                f'[DBG] invalid bbox: ({selected.x1},{selected.y1})'
                f'-({selected.x2},{selected.y2})'
            )
            return

        out = Detection()
        out.cx = float((selected.x1 + selected.x2) / 2.0)
        out.cy = float((selected.y1 + selected.y2) / 2.0)
        self.driver_detection_pub.publish(out)
        self._publish_active_track_id(int(selected.track_id))

    # =========================================================
    # 기존 메서드 (변경 없음)
    # =========================================================
    def _select_track(self, tracks, stream):
        valid_tracks = [t for t in tracks if 0 <= int(t.track_id) <= 254]
        if not valid_tracks:
            return None
        if self.requested_track_id != TRACK_ID_AUTO:
            for track in valid_tracks:
                if int(track.track_id) == self.requested_track_id:
                    self.selected_track_ids[stream] = int(track.track_id)
                    return track
            return None
        selected_track_id = self.selected_track_ids.get(stream)
        if selected_track_id is not None:
            for track in valid_tracks:
                if int(track.track_id) == selected_track_id:
                    return track
            return None
        selected = self._auto_select(valid_tracks)
        self.selected_track_ids[stream] = int(selected.track_id)
        return selected

    def _auto_select(self, tracks):
        if self.auto_select_policy == 'largest_area':
            return max(tracks,
                key=lambda t: max(0.0, float(t.x2-t.x1)) * max(0.0, float(t.y2-t.y1)))
        if self.auto_select_policy == 'highest_score':
            return max(tracks, key=lambda t: float(t.score))
        return tracks[0]

    def _publish_active_track_id(self, track_id):
        msg = UInt8()
        msg.data = track_id
        self.active_track_id_pub.publish(msg)

    def _clear_selected_track_ids(self):
        self.selected_track_ids = {STREAM_EO: None, STREAM_IR: None}
        self._lost_since = {STREAM_EO: None, STREAM_IR: None}
        self._is_holding = {STREAM_EO: False, STREAM_IR: False}
        self._publish_active_track_id(TRACK_ID_AUTO)

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
