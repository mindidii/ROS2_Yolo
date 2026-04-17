from sentinel_interfaces.msg import Detection2D
from sentinel_interfaces.msg import Detection2DArray
from sentinel_interfaces.msg import YoloStatus


def to_detection_array_msg(image_msg, frame_info_msg, detections):
    msg = Detection2DArray()
    msg.stamp = image_msg.header.stamp
    msg.frame_id = int(frame_info_msg.frame_id) if frame_info_msg is not None else 0

    for det in detections:
        detection_msg = Detection2D()
        detection_msg.class_name = str(det.class_name)
        detection_msg.score = float(det.score)
        detection_msg.x1 = float(det.x1)
        detection_msg.y1 = float(det.y1)
        detection_msg.x2 = float(det.x2)
        detection_msg.y2 = float(det.y2)
        msg.detections.append(detection_msg)

    return msg


def to_status_msg(enabled: bool, model_loaded: bool, conf_threshold: float, last_error: str):
    msg = YoloStatus()
    msg.enabled = bool(enabled)
    msg.model_loaded = bool(model_loaded)
    msg.conf_threshold = float(conf_threshold)
    msg.last_error = last_error
    return msg
