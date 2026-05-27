from argparse import Namespace

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sentinel_interfaces.msg import Detection2DArray
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

        self.tracker = BYTETracker(self._tracker_args())
        self.class_names: dict[int, str] = {}
        self.internal_to_external_id: dict[int, int] = {}
        self.available_external_ids = list(range(1, 255))

        self.pub = self.create_publisher(TrackedDetection2DArray, self.tracks_topic, 10)
        self.sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.on_detections,
            10,
        )

        self.get_logger().info(
            f'ByteTrack tracker started: {self.detection_topic} -> {self.tracks_topic}'
        )

    def _tracker_args(self):
        # Ultralytics BYTETracker uses distance thresholds, so IoU 0.30 becomes
        # a matching distance threshold of 0.70.
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
        self._publish_tracks(msg, tracks)

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

        for track in tracks:
            if len(track) < 7:
                continue
            x1, y1, x2, y2 = (float(v) for v in track[:4])
            if x2 <= x1 or y2 <= y1:
                continue

            internal_id = int(track[4])
            external_id = self._external_track_id(internal_id)
            if external_id is None:
                continue

            class_id = int(track[6])
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

        self.pub.publish(msg)

    def _external_track_id(self, internal_id):
        mapped = self.internal_to_external_id.get(internal_id)
        if mapped is not None:
            return mapped
        if not self.available_external_ids:
            self.get_logger().warning('No free external track IDs in range 1..254')
            return None
        external_id = self.available_external_ids.pop(0)
        self.internal_to_external_id[internal_id] = external_id
        return external_id

    def _release_removed_external_ids(self):
        live_internal_ids = {
            int(track.track_id)
            for track in self.tracker.tracked_stracks + self.tracker.lost_stracks
        }
        removed_internal_ids = [
            internal_id for internal_id in self.internal_to_external_id
            if internal_id not in live_internal_ids
        ]
        for internal_id in removed_internal_ids:
            self.internal_to_external_id.pop(internal_id)


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
