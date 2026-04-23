from sentinel_interfaces.msg import Detection
from sentinel_interfaces.msg import Detection2D
from sentinel_interfaces.msg import Detection2DArray
from sentinel_interfaces.msg import FrameSize
from sentinel_interfaces.msg import YoloStatus

# yolo 추론 결과를 ROS 메시지로 변환하는 함수들
def to_detection_array_msg(image_msg, frame_info_msg, detections):
    # 빈 Detection2DArray 메시지 생성
    msg = Detection2DArray()
    # 이미지 타임스탬프와 프레임 ID 설정
    msg.stamp = image_msg.header.stamp
    msg.frame_id = int(frame_info_msg.frame_id) if frame_info_msg is not None else 0

    # det -> Detection2D 
    for det in detections:
        detection_msg = Detection2D()
        detection_msg.class_name = str(det.class_name)
        detection_msg.score = float(det.score)
        detection_msg.x1 = float(det.x1)
        detection_msg.y1 = float(det.y1)
        detection_msg.x2 = float(det.x2)
        detection_msg.y2 = float(det.y2)
        msg.detections.append(detection_msg) # 배열에 추가 
    
    return msg


def to_driver_detection_msg(image_msg, detection):
    frame_w = min(int(image_msg.width), 65535)
    frame_h = min(int(image_msg.height), 65535)

    clipped = _clip_bbox(detection, frame_w, frame_h)
    if clipped is None:
        return None

    x1, y1, x2, y2 = clipped
    msg = Detection()
    msg.cx = float((x1 + x2) / 2.0)
    msg.cy = float((y1 + y2) / 2.0)
    return msg


def to_frame_size_msg(image_msg):
    msg = FrameSize()
    msg.frame_w = min(int(image_msg.width), 65535)
    msg.frame_h = min(int(image_msg.height), 65535)
    return msg


def to_driver_detection_msgs(image_msg, detections):
    msgs = []

    for det in detections:
        msg = to_driver_detection_msg(image_msg, det)
        if msg is None:
            continue

        msgs.append(msg)
        break

    return msgs


def _clip_bbox(det, frame_w, frame_h):
    if frame_w <= 0 or frame_h <= 0:
        return None

    x1 = max(0.0, min(float(det.x1), float(frame_w - 1)))
    y1 = max(0.0, min(float(det.y1), float(frame_h - 1)))
    x2 = max(0.0, min(float(det.x2), float(frame_w - 1)))
    y2 = max(0.0, min(float(det.y2), float(frame_h - 1)))

    if x2 <= x1 or y2 <= y1:
        return None

    return x1, y1, x2, y2


# 노드 상태 -> ROS2 메시지 변환
def to_status_msg(enabled: bool, model_loaded: bool, conf_threshold: float, last_error: str):
    # 노드의 현재 상태를 YoloStatus 메시지로 변환
    msg = YoloStatus()
    msg.enabled = bool(enabled)
    msg.model_loaded = bool(model_loaded)
    msg.conf_threshold = float(conf_threshold)
    msg.last_error = last_error
    return msg
