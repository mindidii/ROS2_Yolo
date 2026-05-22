#!/usr/bin/env python3
"""
ROS2 토픽 → UDP 릴레이 (video_rx 컨테이너 안에서 실행)

사용법:
    python3 ros2_udp_relay.py --gui-host 192.168.0.39
    python3 ros2_udp_relay.py --gui-host 192.168.0.39 --topic /camera/eo --gui-port 6000
"""

import argparse
import socket
import struct
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

GUI_HEADER_FORMAT = "!QIIHH"
FRAGMENT_MAGIC = b"IMGF"
FRAGMENT_FORMAT = "!4sQIIHHHH"
FRAGMENT_SIZE = struct.calcsize(FRAGMENT_FORMAT)
MAX_UDP_PAYLOAD = 60000


def build_packets(jpeg: bytes, w: int, h: int, idx: int, stamp_ns: int) -> list[bytes]:
    header = struct.pack(GUI_HEADER_FORMAT, stamp_ns, idx, len(jpeg), w, h)
    pkt = header + jpeg
    if len(pkt) <= MAX_UDP_PAYLOAD:
        return [pkt]

    frag_payload = max(1024, MAX_UDP_PAYLOAD - FRAGMENT_SIZE)
    count = (len(jpeg) + frag_payload - 1) // frag_payload
    pkts = []
    for i in range(count):
        s = i * frag_payload
        e = min(s + frag_payload, len(jpeg))
        hdr = struct.pack(FRAGMENT_FORMAT, FRAGMENT_MAGIC,
                          stamp_ns, idx, len(jpeg), w, h, i, count)
        pkts.append(hdr + jpeg[s:e])
    return pkts


class RelayNode(Node):
    def __init__(self, topic, gui_host, gui_port, jpeg_quality):
        super().__init__("udp_relay")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self.addr = (gui_host, gui_port)
        self.quality = jpeg_quality
        self.idx = 0
        self.fps_count = 0
        self.fps_time = time.monotonic()

        self.sub = self.create_subscription(
            Image, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"Relaying {topic} → {gui_host}:{gui_port}")

    def on_image(self, msg: Image):
        # decode
        enc = msg.encoding.lower()
        h, w = int(msg.height), int(msg.width)
        arr = np.frombuffer(msg.data, dtype=np.uint8)

        if enc == "bgr8":
            frame = arr.reshape(h, w, 3)
        elif enc == "rgb8":
            frame = cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        elif enc == "mono8":
            frame = cv2.cvtColor(arr.reshape(h, w), cv2.COLOR_GRAY2BGR)
        else:
            return

        ok, jpeg = cv2.imencode(".jpg", frame,
                                [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return

        self.idx += 1
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        for pkt in build_packets(jpeg.tobytes(), w, h, self.idx, stamp_ns):
            self.sock.sendto(pkt, self.addr)

        self.fps_count += 1
        now = time.monotonic()
        if now - self.fps_time >= 2.0:
            fps = self.fps_count / (now - self.fps_time)
            self.get_logger().info(f"{fps:.1f} fps | sent={self.idx}")
            self.fps_count = 0
            self.fps_time = now


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/ir")
    parser.add_argument("--gui-host", required=True)
    parser.add_argument("--gui-port", type=int, default=6001)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    args = parser.parse_args()

    rclpy.init()
    node = RelayNode(args.topic, args.gui_host, args.gui_port, args.jpeg_quality)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# """
# /dev/video0 → UDP 릴레이

# 사용법:
#     python3 cam_udp_sender.py --gui-host 192.168.0.39
#     python3 cam_udp_sender.py --gui-host 192.168.0.39 --device 0 --gui-port 6000 --fps 30
# """

# import argparse
# import socket
# import struct
# import time

# import cv2

# GUI_HEADER_FORMAT = "!QIIHH"
# FRAGMENT_MAGIC = b"IMGF"
# FRAGMENT_FORMAT = "!4sQIIHHHH"
# FRAGMENT_SIZE = struct.calcsize(FRAGMENT_FORMAT)
# MAX_UDP_PAYLOAD = 60000


# def build_packets(jpeg: bytes, w: int, h: int, idx: int, stamp_ns: int) -> list[bytes]:
#     header = struct.pack(GUI_HEADER_FORMAT, stamp_ns, idx, len(jpeg), w, h)
#     pkt = header + jpeg
#     if len(pkt) <= MAX_UDP_PAYLOAD:
#         return [pkt]

#     frag_payload = max(1024, MAX_UDP_PAYLOAD - FRAGMENT_SIZE)
#     count = (len(jpeg) + frag_payload - 1) // frag_payload
#     pkts = []
#     for i in range(count):
#         s = i * frag_payload
#         e = min(s + frag_payload, len(jpeg))
#         hdr = struct.pack(FRAGMENT_FORMAT, FRAGMENT_MAGIC,
#                           stamp_ns, idx, len(jpeg), w, h, i, count)
#         pkts.append(hdr + jpeg[s:e])
#     return pkts


# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--device", type=int, default=0, help="/dev/video0")
#     parser.add_argument("--gui-host", required=True)
#     parser.add_argument("--gui-port", type=int, default=6000)
#     parser.add_argument("--jpeg-quality", type=int, default=85)
#     parser.add_argument("--fps", type=int, default=30)
#     parser.add_argument("--width", type=int, default=1280)
#     parser.add_argument("--height", type=int, default=720)
#     args = parser.parse_args()

#     cap = cv2.VideoCapture(args.device)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
#     cap.set(cv2.CAP_PROP_FPS, args.fps)

#     if not cap.isOpened():
#         print(f"ERROR: /dev/video{args.device} 열 수 없음")
#         return

#     actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#     actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#     print(f"Camera opened: {actual_w}x{actual_h}")

#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
#     addr = (args.gui_host, args.gui_port)
#     print(f"Sending to {args.gui_host}:{args.gui_port}")

#     idx = 0
#     fps_count = 0
#     fps_time = time.monotonic()
#     interval = 1.0 / args.fps

#     try:
#         while True:
#             t0 = time.monotonic()
#             ret, frame = cap.read()
#             if not ret:
#                 continue

#             ok, jpeg = cv2.imencode(
#                 ".jpg", frame,
#                 [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality],
#             )
#             if not ok:
#                 continue

#             idx += 1
#             stamp_ns = int(time.time() * 1_000_000_000)
#             h, w = frame.shape[:2]

#             for pkt in build_packets(jpeg.tobytes(), w, h, idx, stamp_ns):
#                 sock.sendto(pkt, addr)

#             fps_count += 1
#             now = time.monotonic()
#             if now - fps_time >= 2.0:
#                 fps = fps_count / (now - fps_time)
#                 print(f"{fps:.1f} fps | sent={idx}")
#                 fps_count = 0
#                 fps_time = now

#             # 프레임 레이트 조절
#             elapsed = time.monotonic() - t0
#             if elapsed < interval:
#                 time.sleep(interval - elapsed)

#     except KeyboardInterrupt:
#         print("\nStopped.")
#     finally:
#         cap.release()
#         sock.close()


# if __name__ == "__main__":
#     main()