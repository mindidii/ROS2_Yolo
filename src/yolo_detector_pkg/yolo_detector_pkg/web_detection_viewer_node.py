import threading
import time
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from sentinel_interfaces.msg import Detection2DArray


class FrameStore:
    def __init__(self):
        self.condition = threading.Condition()
        self.frame = None
        self.sequence = 0

    def set_frame(self, frame):
        with self.condition:
            self.frame = frame
            self.sequence += 1
            self.condition.notify_all()

    def wait_for_frame(self, last_sequence, timeout=1.0):
        with self.condition:
            if self.sequence == last_sequence:
                self.condition.wait(timeout)
            return self.sequence, self.frame


class WebDetectionViewerNode(Node):
    def __init__(self):
        super().__init__('web_detection_viewer')

        self.declare_parameter('image_topic', '/video/eo/preprocessed')
        self.declare_parameter('detection_topic', '/detections/eo')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('resize_width', 960)
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('draw_center', True)
        self.declare_parameter('show_fps', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.resize_width = int(self.get_parameter('resize_width').value)
        self.max_detection_age_sec = float(self.get_parameter('max_detection_age_sec').value)
        self.draw_center = bool(self.get_parameter('draw_center').value)
        self.show_fps = bool(self.get_parameter('show_fps').value)

        self.bridge = CvBridge()
        self.frame_store = FrameStore()
        self.latest_detections = None
        self.latest_detection_time = 0.0
        self.frame_count = 0
        self.fps = 0.0
        self.fps_start = time.monotonic()

        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.on_detections,
            10,
        )

        self.httpd = self._make_server()
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()

        self.get_logger().info(
            f'Web detection viewer started: image={self.image_topic}, '
            f'detections={self.detection_topic}'
        )
        self.get_logger().info(f'Open http://localhost:{self.port}/ in a browser')

    def destroy_node(self):
        if hasattr(self, 'httpd'):
            self.httpd.shutdown()
            self.httpd.server_close()
        super().destroy_node()

    def _make_server(self):
        frame_store = self.frame_store

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    self._send_index()
                    return
                if self.path == '/stream.mjpg':
                    self._send_stream()
                    return
                self.send_error(404)

            def log_message(self, fmt, *args):
                return

            def _send_index(self):
                body = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>YOLO Detection Viewer</title>
  <style>
    html, body {{
      margin: 0;
      height: 100%;
      background: #111;
      color: #eee;
      font-family: Arial, sans-serif;
    }}
    header {{
      height: 40px;
      display: flex;
      align-items: center;
      gap: 16px;
      padding: 0 12px;
      background: #202020;
      font-size: 14px;
    }}
    main {{
      height: calc(100% - 40px);
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
    }}
    img {{
      max-width: 100%;
      max-height: 100%;
      object-fit: contain;
    }}
  </style>
</head>
<body>
  <header>
    <strong>YOLO Detection Viewer</strong>
    <span>{self.server.image_topic}</span>
    <span>{self.server.detection_topic}</span>
  </header>
  <main><img src="/stream.mjpg"></main>
</body>
</html>
"""
                encoded = body.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _send_stream(self):
                self.send_response(200)
                self.send_header('Age', '0')
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=frame',
                )
                self.end_headers()

                last_sequence = 0
                while True:
                    sequence, frame = frame_store.wait_for_frame(last_sequence)
                    if frame is None or sequence == last_sequence:
                        continue
                    last_sequence = sequence
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError):
                        break

        server = ThreadingHTTPServer((self.host, self.port), Handler)
        server.image_topic = self.image_topic
        server.detection_topic = self.detection_topic
        return server

    def on_detections(self, msg):
        self.latest_detections = msg
        self.latest_detection_time = time.monotonic()

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        self._update_fps()
        self._draw_overlay(frame)

        if self.resize_width > 0 and frame.shape[1] > self.resize_width:
            scale = self.resize_width / float(frame.shape[1])
            height = max(1, int(frame.shape[0] * scale))
            frame = cv2.resize(frame, (self.resize_width, height), interpolation=cv2.INTER_AREA)

        ok, encoded = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if ok:
            self.frame_store.set_frame(encoded.tobytes())

    def _draw_overlay(self, frame):
        detections_msg = self.latest_detections
        detection_age = time.monotonic() - self.latest_detection_time
        detections = []
        if detections_msg is not None and detection_age <= self.max_detection_age_sec:
            detections = list(detections_msg.detections)

        for detection in detections:
            x1 = self._clip(int(round(detection.x1)), 0, frame.shape[1] - 1)
            y1 = self._clip(int(round(detection.y1)), 0, frame.shape[0] - 1)
            x2 = self._clip(int(round(detection.x2)), 0, frame.shape[1] - 1)
            y2 = self._clip(int(round(detection.y2)), 0, frame.shape[0] - 1)
            if x2 <= x1 or y2 <= y1:
                continue

            color = self._color_for_class(int(detection.class_id))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            self._draw_label(frame, f'{detection.class_name} {detection.score:.2f}', x1, y1, color)

            if self.draw_center:
                cx = int((x1 + x2) * 0.5)
                cy = int((y1 + y2) * 0.5)
                cv2.drawMarker(
                    frame,
                    (cx, cy),
                    color,
                    markerType=cv2.MARKER_CROSS,
                    markerSize=12,
                    thickness=2,
                )

        status = f'detections={len(detections)}'
        if detections_msg is None:
            status = 'waiting for detections'
        elif detection_age > self.max_detection_age_sec:
            status = f'detections stale ({detection_age:.2f}s)'

        lines = [status]
        if self.show_fps:
            lines.append(f'viewer fps={self.fps:.1f}')
        self._draw_status(frame, lines)

    def _draw_label(self, frame, text, x, y, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 1
        (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)
        label_y = max(text_h + baseline + 4, y)
        cv2.rectangle(
            frame,
            (x, label_y - text_h - baseline - 4),
            (x + text_w + 6, label_y + baseline - 2),
            color,
            -1,
        )
        cv2.putText(
            frame,
            text,
            (x + 3, label_y - 4),
            font,
            scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def _draw_status(self, frame, lines):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thickness = 1
        y = 24
        for line in lines:
            cv2.putText(
                frame,
                line,
                (12, y),
                font,
                scale,
                (20, 20, 20),
                thickness + 2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                line,
                (12, y),
                font,
                scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
            y += 24

    def _update_fps(self):
        self.frame_count += 1
        now = time.monotonic()
        elapsed = now - self.fps_start
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_start = now

    @staticmethod
    def _clip(value, low, high):
        return max(low, min(value, high))

    @staticmethod
    def _color_for_class(class_id):
        palette = [
            (80, 220, 60),
            (40, 170, 255),
            (255, 120, 60),
            (220, 80, 220),
            (60, 220, 220),
        ]
        return palette[class_id % len(palette)]


def main(args=None):
    rclpy.init(args=args)
    node = WebDetectionViewerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
