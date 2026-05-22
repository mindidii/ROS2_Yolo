# YOLO Detector GUI Interface

이 문서는 GUI 담당자가 현재 tracker 중심 구조에서 구독할 영상, 추적 결과, 상태, 서비스 정보를 한 번에 확인하기 위한 정리 문서다.

현재 시스템은 IR/EO 카메라를 분리해서 YOLO 노드 2개를 실행한다.

- `yolo_detector_ir_node`
- `yolo_detector_eo_node`

각 YOLO 노드는 입력 영상과 `FrameInfo`의 `stamp`가 같은 프레임만 매칭해서 추론한다.

현재 데이터 흐름은 아래와 같다.

```text
YOLO node -> /detections/* -> ByteTrack tracker node -> /tracks/*
```

따라서 GUI bbox overlay는 기본적으로 `/tracks/*`를 구독한다. `/detections/*`는 tracker 입력 및 디버깅용 내부 토픽으로 둔다.

## 1. GUI에서 주로 구독할 토픽

### IR

| 용도 | 토픽 | 타입 |
|---|---|---|
| 영상 표시 | `/camera/ir` | `sensor_msgs/msg/Image` |
| bbox + track_id 표시 | `/tracks/ir` | `sentinel_interfaces/msg/TrackedDetection2DArray` |
| 대표 객체 1개 중심 좌표 | `/driver/ir/detection` | `sentinel_interfaces/msg/Detection` |
| YOLO 상태 | `/yolo/ir/status` | `sentinel_interfaces/msg/YoloStatus` |

### EO

| 용도 | 토픽 | 타입 |
|---|---|---|
| 영상 표시 | `/video/eo/preprocessed` | `sensor_msgs/msg/Image` |
| bbox + track_id 표시 | `/tracks/eo` | `sentinel_interfaces/msg/TrackedDetection2DArray` |
| 대표 객체 1개 중심 좌표 | `/driver/eo/detection` | `sentinel_interfaces/msg/Detection` |
| YOLO 상태 | `/yolo/eo/status` | `sentinel_interfaces/msg/YoloStatus` |

GUI에서 영상과 bbox를 함께 사용할 때는 영상 `header.stamp`와 `/tracks/*`의 `stamp`를 기준으로 매칭하면 된다.

## 2. YOLO 입력 토픽

EO YOLO 노드는 전처리된 영상을 입력으로 사용하고, IR YOLO 노드는 현재 `/camera/ir` 영상을 직접 입력으로 사용한다.

| 카메라 | 입력 영상 | 입력 프레임 정보 |
|---|---|---|
| IR | `/camera/ir` | `/camera/ir/frame_info` |
| EO | `/video/eo/preprocessed` | `/video/eo/preprocessed/frame_info` |

현재 `image_preprocess_node`는 EO 영상 보정 후 `/video/eo/preprocessed`로 재발행한다. 설정상 출력 해상도는 EO 입력 해상도와 동일하게 사용한다.

YOLO 모델 입력 크기는 설정상 `640x640`이지만, publish되는 detection 좌표는 원본 이미지 픽셀 좌표계 기준이다. 예를 들어 현재 영상이 `1280x720`이면 detection 좌표도 `1280x720` 기준으로 해석한다.

## 3. tracker 입력 탐지 결과: `Detection2DArray`

토픽:

- `/detections/ir`
- `/detections/eo`

타입:

```text
sentinel_interfaces/msg/Detection2DArray
```

메시지 정의:

```text
builtin_interfaces/Time stamp
uint32 frame_id
Detection2D[] detections
```

필드 의미:

| 필드 | 의미 |
|---|---|
| `stamp` | 해당 detection이 대응하는 영상 프레임 timestamp |
| `frame_id` | 영상 프레임 번호 |
| `detections` | tracker에 입력할 한 프레임의 YOLO 탐지 객체 배열 |

GUI가 track_id 포함 bbox를 표시하는 경우 `/detections/*`를 직접 구독할 필요는 없다. 이 토픽은 `bytetrack_tracker_*_node`가 구독해서 `/tracks/*`를 만들기 위한 입력 토픽이다.

## 3-1. GUI bbox 표시용 추적 결과: `TrackedDetection2DArray`

토픽:

- `/tracks/ir`
- `/tracks/eo`

타입:

```text
sentinel_interfaces/msg/TrackedDetection2DArray
```

메시지 정의:

```text
builtin_interfaces/Time stamp
uint32 frame_id
TrackedDetection2D[] tracks
```

개별 track:

```text
int32 track_id
int32 class_id
string class_name
float32 score
float32 x1
float32 y1
float32 x2
float32 y2
```

GUI는 `tracks[]`를 순회하면서 `x1`, `y1`, `x2`, `y2`로 bbox를 그리고, `track_id`, `class_name`, `score`를 label로 표시하면 된다.

주의:

- tracker는 confirmed track만 publish한다.
- 현재 tracker 설정은 `min_confirm_hits: 2`라서 객체가 탐지된 직후 1프레임 정도는 `/detections/*`에는 있어도 `/tracks/*`에는 없을 수 있다.
- 안정적인 ID 표시가 우선이면 `/tracks/*`만 사용한다.
- 감지 즉시 표시가 꼭 필요하면 `/tracks/*`가 비어 있을 때 `/detections/*`를 fallback으로 사용할 수 있다.

## 4. 개별 bbox: `Detection2D`

메시지 정의:

```text
string class_name
float32 score
float32 x1
float32 y1
float32 x2
float32 y2
```

좌표 의미:

| 필드 | 의미 |
|---|---|
| `class_name` | 객체 클래스 이름. 예: `person` |
| `score` | confidence score |
| `x1`, `y1` | bbox 좌상단 픽셀 좌표 |
| `x2`, `y2` | bbox 우하단 픽셀 좌표 |

좌표계:

```text
(0, 0) ----------------> x
  |
  |
  v
  y
```

주의:

- `/detections/*`는 tracker 입력용 YOLO 결과를 전달한다.
- bbox가 영상 경계를 살짝 벗어나는 경우 음수나 영상 크기보다 큰 값이 나올 수 있다.
- GUI overlay에서는 화면 밖 좌표를 표시 영역 안으로 clamp해서 그리는 것이 안전하다.

예시:

```yaml
stamp:
  sec: 123
  nanosec: 456000000
frame_id: 520
detections:
- class_name: person
  score: 0.87
  x1: 420.0
  y1: 110.0
  x2: 710.0
  y2: 680.0
```

## 5. 대표 객체 중심 좌표: `Detection`

토픽:

- `/driver/ir/detection`
- `/driver/eo/detection`

타입:

```text
sentinel_interfaces/msg/Detection
```

메시지 정의:

```text
float32 cx
float32 cy
uint16 frame_w
uint16 frame_h
```

필드 의미:

| 필드 | 의미 |
|---|---|
| `cx` | 대표 bbox의 중심 x 좌표 |
| `cy` | 대표 bbox의 중심 y 좌표 |
| `frame_w` | 해당 영상의 width |
| `frame_h` | 해당 영상의 height |

현재 동작:

- 여러 객체가 탐지되어도 대표 객체 1개만 publish한다.
- 대표 객체는 현재 후처리 결과에서 score가 가장 높은 첫 번째 유효 객체다.
- driver용 좌표는 영상 범위 안으로 clamp된 bbox를 기준으로 계산한다.
- 따라서 `cx`, `cy`는 정상적으로는 아래 범위 안에 있어야 한다.

```text
0 <= cx < frame_w
0 <= cy < frame_h
```

예시:

```yaml
cx: 640.0
cy: 360.0
frame_w: 1280
frame_h: 720
```

GUI에서 모터/조준 방향을 시각화하려면 이 토픽을 사용하면 된다. bbox 전체를 그릴 때는 `/tracks/*`를 사용한다.

## 6. YOLO 상태: `YoloStatus`

토픽:

- `/yolo/ir/status`
- `/yolo/eo/status`

타입:

```text
sentinel_interfaces/msg/YoloStatus
```

메시지 정의:

```text
bool enabled
bool model_loaded
float32 conf_threshold
string last_error
```

필드 의미:

| 필드 | 의미 |
|---|---|
| `enabled` | YOLO 추론 활성화 여부 |
| `model_loaded` | ONNX 모델 로드 성공 여부 |
| `conf_threshold` | 현재 confidence threshold |
| `last_error` | 최근 오류 메시지. 정상일 때 빈 문자열 |

GUI 표시 예:

- `enabled=false`: YOLO 꺼짐
- `model_loaded=false`: 모델 파일 로드 실패
- `last_error`가 비어 있지 않음: 에러 배너 또는 로그 표시

## 7. GUI에서 호출 가능한 서비스

### 7.1 YOLO 활성화/비활성화

IR:

```text
/yolo/ir/enable
```

EO:

```text
/yolo/eo/enable
```

타입:

```text
sentinel_interfaces/srv/SetBoolFlag
```

서비스 정의:

```text
bool data
---
bool success
string message
```

CLI 예시:

```bash
ros2 service call /yolo/ir/enable sentinel_interfaces/srv/SetBoolFlag "{data: true}"
ros2 service call /yolo/ir/enable sentinel_interfaces/srv/SetBoolFlag "{data: false}"
```

GUI 사용:

- 토글 ON: `{data: true}`
- 토글 OFF: `{data: false}`

### 7.2 Confidence threshold 변경

IR:

```text
/yolo/ir/set_threshold
```

EO:

```text
/yolo/eo/set_threshold
```

타입:

```text
sentinel_interfaces/srv/SetThreshold
```

서비스 정의:

```text
float32 threshold
---
bool success
string message
```

CLI 예시:

```bash
ros2 service call /yolo/ir/set_threshold sentinel_interfaces/srv/SetThreshold "{threshold: 0.35}"
ros2 service call /yolo/eo/set_threshold sentinel_interfaces/srv/SetThreshold "{threshold: 0.50}"
```

GUI 사용:

- threshold slider 또는 numeric input 권장
- 일반 범위는 `0.0 ~ 1.0`
- 현재 기본값은 `0.25`

## 8. 현재 설정값

설정 파일:

- `src/sentinel_bringup/config/yolo_detector_ir.yaml`
- `src/sentinel_bringup/config/yolo_detector_eo.yaml`

공통 주요 설정:

| 파라미터 | 현재 값 |
|---|---|
| `model_path` | `/ros2_ws/src/yolo_detector_pkg/model/last.engine` |
| `input_width` | `640` |
| `input_height` | `640` |
| `conf_threshold` | `0.35` |
| `enabled` | `true` |
| `inference_period_sec` | `0.05` |
| `sync_queue_size` | `30` |

IR 토픽 설정:

| 파라미터 | 값 |
|---|---|
| `image_topic` | `/camera/ir` |
| `frame_info_topic` | `/camera/ir/frame_info` |
| `detection_topic` | `/detections/ir` |
| `driver_detection_topic` | `/driver/ir/detection` |
| `driver_frame_size_topic` | `/driver/ir/frame_size` |
| `status_topic` | `/yolo/ir/status` |

EO 토픽 설정:

| 파라미터 | 값 |
|---|---|
| `image_topic` | `/video/eo/preprocessed` |
| `frame_info_topic` | `/video/eo/preprocessed/frame_info` |
| `detection_topic` | `/detections/eo` |
| `driver_detection_topic` | `/driver/eo/detection` |
| `driver_frame_size_topic` | `/driver/eo/frame_size` |
| `status_topic` | `/yolo/eo/status` |

## 9. GUI 동기화 규칙

권장 매칭 방식:

1. `/camera/ir.header.stamp`와 `/tracks/ir.stamp`가 같은 메시지를 같은 프레임으로 처리한다.
2. `/video/eo/preprocessed.header.stamp`와 `/tracks/eo.stamp`가 같은 메시지를 같은 프레임으로 처리한다.
3. `frame_id`는 디버깅용으로 사용하고, 동기화 기준은 `stamp`를 우선한다.

추적 결과가 없는 프레임도 `/tracks/*`가 빈 배열로 publish될 수 있다.

```yaml
tracks: []
```

이 경우 GUI는 bbox를 지우거나 "no detection" 상태로 표시하면 된다.

## 10. 확인 명령어

토픽 목록:

```bash
ros2 topic list | grep -E "yolo|detection"
```

IR detection 확인:

```bash
ros2 topic echo /tracks/ir
ros2 topic echo /driver/ir/detection
ros2 topic echo /yolo/ir/status
```

EO detection 확인:

```bash
ros2 topic echo /tracks/eo
ros2 topic echo /driver/eo/detection
ros2 topic echo /yolo/eo/status
```

영상 표시:

```bash
cd /ros2_ws
./run_rqt_image_view.sh
```

토픽 주기:

```bash
ros2 topic hz /tracks/ir
ros2 topic hz /tracks/eo
```

## 11. GUI 구현 시 주의사항

- `/detections/*`는 tracker 입력용 YOLO 객체 배열이다.
- `/tracks/*`는 GUI bbox overlay용 track 배열이다.
- `/driver/*/detection`은 대표 객체 1개 중심 좌표다.
- bbox overlay는 `/tracks/*`를 사용한다.
- 조준점, crosshair, 모터 방향 표시 등은 `/driver/*/detection`을 사용한다.
- `/tracks/*`의 bbox 좌표는 화면 밖으로 약간 벗어날 수 있으므로 GUI 렌더링 시 clamp하는 것이 안전하다.
- `/driver/*/detection`의 `cx`, `cy`는 현재 코드에서 clamp된 bbox 기준으로 계산된다.
- YOLO 추론이 꺼진 상태에서는 detection publish가 멈출 수 있다.
- 모델 로드 실패 시 `/yolo/*/status.model_loaded=false`와 `last_error`를 확인한다.
