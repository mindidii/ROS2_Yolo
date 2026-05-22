from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sentinel_interfaces.msg import Detection2DArray
from sentinel_interfaces.msg import TrackedDetection2D
from sentinel_interfaces.msg import TrackedDetection2DArray


@dataclass
class Track:
    track_id: int
    bbox: tuple[float, float, float, float]
    score: float
    class_id: int
    class_name: str
    age: int = 1
    hits: int = 1
    missed: int = 0
    confirmed: bool = False
    velocity: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    class_votes: dict[int, tuple[str, float]] = field(default_factory=dict)

    def predicted_bbox(self):
        return tuple(self.bbox[i] + self.velocity[i] for i in range(4))

    def update(self, detection):
        old_bbox = self.bbox
        self.bbox = detection.bbox
        self.velocity = tuple(self.bbox[i] - old_bbox[i] for i in range(4))
        self.score = detection.score
        self.age += 1
        self.hits += 1
        self.missed = 0
        _, previous_score = self.class_votes.get(detection.class_id, (detection.class_name, 0.0))
        self.class_votes[detection.class_id] = (
            detection.class_name,
            previous_score + max(detection.score, 0.01),
        )
        self.class_id, (self.class_name, _) = max(
            self.class_votes.items(),
            key=lambda item: item[1][1],
        )

    def mark_missed(self):
        self.bbox = self.predicted_bbox()
        self.age += 1
        self.missed += 1


@dataclass(frozen=True)
class Detection:
    bbox: tuple[float, float, float, float]
    score: float
    class_id: int
    class_name: str


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

        self.next_track_id = 1
        self.tracks: list[Track] = []

        self.pub = self.create_publisher(TrackedDetection2DArray, self.tracks_topic, 10)
        self.sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.on_detections,
            10,
        )

        self.get_logger().info(
            f'ByteTrack-style tracker started: {self.detection_topic} -> {self.tracks_topic}'
        )

    def on_detections(self, msg):
        detections = self._valid_detections(msg.detections)
        high_detections = [d for d in detections if d.score >= self.high_score_threshold]
        low_detections = [
            d for d in detections
            if self.low_score_threshold <= d.score < self.high_score_threshold
        ]

        unmatched_tracks, unmatched_high = self._match_and_update(
            list(range(len(self.tracks))),
            high_detections,
            self.match_iou_threshold,
        )
        unmatched_tracks, _ = self._match_and_update(
            unmatched_tracks,
            low_detections,
            self.low_match_iou_threshold,
        )

        for track_index in unmatched_tracks:
            self.tracks[track_index].mark_missed()

        for det_index in unmatched_high:
            self._start_track(high_detections[det_index])

        self.tracks = [
            track for track in self.tracks
            if track.missed <= self.track_buffer_frames
        ]
        for track in self.tracks:
            if track.hits >= self.min_confirm_hits:
                track.confirmed = True

        self._publish_tracks(msg)

    def _valid_detections(self, detections):
        valid = []
        for det in detections:
            bbox = (float(det.x1), float(det.y1), float(det.x2), float(det.y2))
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue
            valid.append(
                Detection(
                    bbox=bbox,
                    score=float(det.score),
                    class_id=int(det.class_id),
                    class_name=str(det.class_name),
                )
            )
        return valid

    def _match_and_update(self, track_indices, detections, iou_threshold):
        if not track_indices or not detections:
            return track_indices, list(range(len(detections)))

        candidate_pairs = []
        for track_index in track_indices:
            track = self.tracks[track_index]
            track_bbox = track.predicted_bbox()
            for det_index, detection in enumerate(detections):
                if self.class_aware_matching and detection.class_id != track.class_id:
                    continue
                iou = self._iou(track_bbox, detection.bbox)
                if iou >= iou_threshold:
                    candidate_pairs.append((iou, track_index, det_index))

        candidate_pairs.sort(reverse=True, key=lambda item: item[0])
        matched_tracks = set()
        matched_detections = set()

        for _, track_index, det_index in candidate_pairs:
            if track_index in matched_tracks or det_index in matched_detections:
                continue
            self.tracks[track_index].update(detections[det_index])
            matched_tracks.add(track_index)
            matched_detections.add(det_index)

        unmatched_tracks = [
            track_index for track_index in track_indices
            if track_index not in matched_tracks
        ]
        unmatched_detections = [
            det_index for det_index in range(len(detections))
            if det_index not in matched_detections
        ]
        return unmatched_tracks, unmatched_detections

    def _start_track(self, detection):
        track = Track(
            track_id=self.next_track_id,
            bbox=detection.bbox,
            score=detection.score,
            class_id=detection.class_id,
            class_name=detection.class_name,
            class_votes={
                detection.class_id: (detection.class_name, max(detection.score, 0.01))
            },
        )
        if self.min_confirm_hits <= 1:
            track.confirmed = True
        self.next_track_id += 1
        self.tracks.append(track)

    def _publish_tracks(self, source_msg):
        msg = TrackedDetection2DArray()
        msg.stamp = source_msg.stamp
        msg.frame_id = source_msg.frame_id

        for track in self.tracks:
            if not track.confirmed or track.missed > 0:
                continue
            out = TrackedDetection2D()
            out.track_id = int(track.track_id)
            out.class_id = int(track.class_id)
            out.class_name = str(track.class_name)
            out.score = float(track.score)
            out.x1 = float(track.bbox[0])
            out.y1 = float(track.bbox[1])
            out.x2 = float(track.bbox[2])
            out.y2 = float(track.bbox[3])
            msg.tracks.append(out)

        self.pub.publish(msg)

    @staticmethod
    def _iou(box_a, box_b):
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b

        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        iw = max(0.0, ix2 - ix1)
        ih = max(0.0, iy2 - iy1)
        intersection = iw * ih
        if intersection <= 0.0:
            return 0.0

        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - intersection
        if union <= 0.0:
            return 0.0
        return intersection / union


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
