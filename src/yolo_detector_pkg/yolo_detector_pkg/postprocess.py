import numpy as np

# 전체 후처리의 시작점 
def decode_yolo_output(outputs, conf_threshold, scale_x, scale_y, num_classes, iou_threshold=0.45):
    pred = _reshape_predictions(outputs[0])
    if pred is None:
        return []

    detections = []
    # 각 row를 하나씩 해석 
    for row in pred:
        decoded = _decode_row(row, conf_threshold, num_classes)
        if decoded is None:
            continue

        if decoded["format"] == "xyxy":
            x1 = decoded["x1"] * scale_x
            y1 = decoded["y1"] * scale_y
            x2 = decoded["x2"] * scale_x
            y2 = decoded["y2"] * scale_y
            class_id = decoded["class_id"]
            score = decoded["score"]
        else:
            class_id = decoded["class_id"]
            score = decoded["score"]
            cx = decoded["cx"]
            cy = decoded["cy"]
            w = decoded["w"]
            h = decoded["h"]

            x1 = (cx - w / 2.0) * scale_x
            y1 = (cy - h / 2.0) * scale_y
            x2 = (cx + w / 2.0) * scale_x
            y2 = (cy + h / 2.0) * scale_y

        detections.append({
            "class_id": class_id,
            "score": score,
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
        })

    detections = _apply_nms(detections, iou_threshold)
    detections.sort(key=lambda d: d["score"], reverse=True)
    return detections[:50]


def _reshape_predictions(pred):
    pred = np.asarray(pred)
    pred = np.squeeze(pred)

    if pred.ndim == 1:
        return pred.reshape(1, -1)

    if pred.ndim != 2:
        return None

    # Normalize to (num_predictions, num_features).
    if pred.shape[0] <= 128 and pred.shape[1] > pred.shape[0]:
        pred = pred.transpose(1, 0)

    return pred


def _decode_row(row, conf_threshold, num_classes):
    row = np.asarray(row, dtype=np.float32)
    num_features = int(row.shape[0])

    if num_features < 5:
        return None

    if num_features == 6:
        score = float(row[4])
        if score >= conf_threshold:
            return {
                "format": "xyxy",
                "x1": float(row[0]),
                "y1": float(row[1]),
                "x2": float(row[2]),
                "y2": float(row[3]),
                "score": score,
                "class_id": int(round(float(row[5]))),
            }
        return None

    cx, cy, w, h = [float(v) for v in row[:4]]

    # 1-class custom export: [cx, cy, w, h, score]
    if num_features == 5:
        class_id = 0
        score = float(row[4])
    # YOLOv8/YOLO11 common export: [cx, cy, w, h, class_scores...].
    # Some models export many classes (for example 80 COCO classes) even when
    # this package only names/uses a subset, so infer this layout from the full
    # feature width instead of the configured local class-name count alone.
    elif num_features == 4 + num_classes or num_features == 84:
        class_scores = row[4:]
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])
    # YOLOv5-style export: [cx, cy, w, h, obj, class_scores...]
    elif num_features >= 5 + num_classes:
        objectness = float(row[4])
        class_scores = row[5:]
        class_id = int(np.argmax(class_scores))
        score = objectness * float(class_scores[class_id])
    else:
        # Fallback for uncommon layouts.
        class_scores = row[4:]
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])

    if score < conf_threshold:
        return None

    return {
        "format": "cxcywh",
        "class_id": class_id,
        "score": score,
        "cx": cx,
        "cy": cy,
        "w": w,
        "h": h,
    }


def _apply_nms(detections, iou_threshold):
    if not detections:
        return []

    kept = []

    for class_id in sorted({det["class_id"] for det in detections}):
        class_detections = [det for det in detections if det["class_id"] == class_id]
        class_detections.sort(key=lambda d: d["score"], reverse=True)

        while class_detections:
            best = class_detections.pop(0)
            kept.append(best)
            class_detections = [
                det for det in class_detections
                if _bbox_iou(best, det) < iou_threshold
            ]

    return kept


def _bbox_iou(det_a, det_b):
    x1 = max(det_a["x1"], det_b["x1"])
    y1 = max(det_a["y1"], det_b["y1"])
    x2 = min(det_a["x2"], det_b["x2"])
    y2 = min(det_a["y2"], det_b["y2"])

    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter_area = inter_w * inter_h

    area_a = max(0.0, det_a["x2"] - det_a["x1"]) * max(0.0, det_a["y2"] - det_a["y1"])
    area_b = max(0.0, det_b["x2"] - det_b["x1"]) * max(0.0, det_b["y2"] - det_b["y1"])
    union = area_a + area_b - inter_area

    if union <= 0.0:
        return 0.0

    return inter_area / union
