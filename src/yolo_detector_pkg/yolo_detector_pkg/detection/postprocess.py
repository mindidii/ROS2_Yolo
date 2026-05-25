import numpy as np

# 전체 후처리의 시작점
def decode_yolo_output(outputs, conf_threshold, transform, num_classes, iou_threshold=0.45):
    pred, channel_first = _reshape_predictions(outputs[0])
    if pred is None:
        return []

    detections = []
    # 각 row를 하나씩 해석 
    for row in pred:
        decoded = _decode_row(row, conf_threshold, num_classes, channel_first)
        if decoded is None:
            continue

        if decoded["format"] == "xyxy":
            x1, y1, x2, y2 = _restore_box(
                decoded["x1"],
                decoded["y1"],
                decoded["x2"],
                decoded["y2"],
                transform,
            )
            class_id = decoded["class_id"]
            score = decoded["score"]
        else:
            class_id = decoded["class_id"]
            score = decoded["score"]
            cx = decoded["cx"]
            cy = decoded["cy"]
            w = decoded["w"]
            h = decoded["h"]

            x1, y1, x2, y2 = _restore_box(
                cx - w / 2.0,
                cy - h / 2.0,
                cx + w / 2.0,
                cy + h / 2.0,
                transform,
            )

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


def _restore_box(x1, y1, x2, y2, transform):
    orig_w = float(transform["orig_w"])
    orig_h = float(transform["orig_h"])

    if transform.get("mode") == "resize":
        scale_x = float(transform["scale_x"])
        scale_y = float(transform["scale_y"])
        x1 = float(x1) * scale_x
        y1 = float(y1) * scale_y
        x2 = float(x2) * scale_x
        y2 = float(y2) * scale_y
    else:
        gain = max(float(transform["gain"]), 1e-9)
        pad_x = float(transform["pad_x"])
        pad_y = float(transform["pad_y"])
        x1 = (float(x1) - pad_x) / gain
        y1 = (float(y1) - pad_y) / gain
        x2 = (float(x2) - pad_x) / gain
        y2 = (float(y2) - pad_y) / gain

    x1 = min(max(x1, 0.0), orig_w)
    y1 = min(max(y1, 0.0), orig_h)
    x2 = min(max(x2, 0.0), orig_w)
    y2 = min(max(y2, 0.0), orig_h)

    return x1, y1, x2, y2


def _reshape_predictions(pred):
    pred = np.asarray(pred)
    pred = np.squeeze(pred)

    if pred.ndim == 1:
        return pred.reshape(1, -1), False

    if pred.ndim != 2:
        return None, False

    # Normalize to (num_predictions, num_features).
    channel_first = pred.shape[0] <= 128 and pred.shape[1] > pred.shape[0]
    if channel_first:
        pred = pred.transpose(1, 0)

    return pred, channel_first


def _decode_row(row, conf_threshold, num_classes, channel_first=False):
    row = np.asarray(row, dtype=np.float32)
    num_features = int(row.shape[0])

    if num_features < 5:
        return None

    # TensorRT/ONNX raw YOLO exports commonly have shape (1, 4 + classes, 8400).
    # After transposing that becomes a 6-feature row for 2-class models, which
    # must be decoded as cx/cy/w/h + class scores, not xyxy + score + class_id.
    if num_features == 6 and not (channel_first or num_features == 4 + num_classes):
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
