#!/usr/bin/env python3
"""测试 ZMQ 图像传输是否正常。

支持两种图像服务端协议，通过 --mode 切换：

- bridge2 (默认): 连接 Bridge2 (ros2_deploy_bridge.py) 的 ZMQ PUB 端口
  (默认 5560)，接收三段消息 (metadata(JSON) + RGB + depth)。
  前提：Bridge2 已启动。
- raw: 连接 image_server.py 的 ImageServer PUB 端口 (默认 5558)，
  接收单段 JPEG 消息；若服务端开启 Unit_Test，会带 12 字节
  struct('dI') 时间戳/帧号头，本脚本会自动剥离。

保存前 N 帧到 test_output/ 目录，超时则报错退出。

Usage:
  python3 image/test_zmq_image.py                                 # Bridge2 @ 5560
  python3 image/test_zmq_image.py --mode raw --port 5558          # ImageServer @ 5558
"""

import argparse
import os
import struct
import sys
import time

import cv2
import numpy as np
import zmq

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5560
SAVE_COUNT = 3
TIMEOUT_S = 10.0


def main():
    parser = argparse.ArgumentParser(description="Test ZMQ image reception (Bridge2 or ImageServer)")
    parser.add_argument("--host", type=str, default=DEFAULT_HOST, help="ZMQ host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="ZMQ image port")
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S, help="Timeout in seconds")
    parser.add_argument(
        "--mode",
        choices=("bridge2", "raw"),
        default="bridge2",
        help="bridge2: multipart JSON+RGB+depth (default 5560); raw: single JPEG from ImageServer (default 5558)",
    )
    args = parser.parse_args()

    save_dir = os.path.join(SCRIPT_DIR, "test_output", "zmq_image")
    os.makedirs(save_dir, exist_ok=True)

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://{args.host}:{args.port}")
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print(f"Connecting to ZMQ tcp://{args.host}:{args.port} (mode={args.mode}) ...")

    frame_count = 0
    first_frame_time = None
    start = time.time()
    raw_mode = args.mode == "raw"
    # ImageServer prepends struct('dI') = 12 bytes when started with Unit_Test=True
    header_size = struct.calcsize("dI") if raw_mode else 0
    jpeg_magic = b"\xff\xd8\xff"

    try:
        while frame_count < SAVE_COUNT:
            if time.time() - start > args.timeout:
                print(f"ERROR: Timeout — no frame received within {args.timeout}s")
                socket.close()
                context.term()
                sys.exit(1)

            if raw_mode:
                # ImageServer protocol: single-part JPEG, optionally with a
                # 12-byte struct header. Detect the header by absence of the
                # JPEG magic at the start.
                try:
                    msg = socket.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    time.sleep(0.01)
                    continue

                jpg_bytes = msg
                if not jpg_bytes.startswith(jpeg_magic) and len(jpg_bytes) > header_size:
                    if jpg_bytes[header_size:header_size + 3] == jpeg_magic:
                        jpg_bytes = jpg_bytes[header_size:]
                    else:
                        print("WARNING: frame is not a JPEG and has no recognizable header, skipping")
                        continue

                bgr = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    print("WARNING: failed to decode JPEG frame")
                    continue

                height, width = bgr.shape[:2]
                if first_frame_time is None:
                    first_frame_time = time.time()
                    print(f"First frame: {width}x{height}, format=jpeg")

                frame_count += 1
                path = os.path.join(save_dir, f"frame_{frame_count:03d}.jpg")
                cv2.imwrite(path, bgr)
                print(f"Saved: {path}")
                continue

            # Bridge2 protocol: metadata(JSON) + RGB + optional depth
            try:
                metadata = socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.01)
                continue

            rgb_bytes = socket.recv()
            # Depth is optional
            try:
                depth_bytes = socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                depth_bytes = b""

            width = metadata.get("width", 0)
            height = metadata.get("height", 0)
            fmt = metadata.get("format", "unknown")

            if first_frame_time is None:
                first_frame_time = time.time()
                depth_info = f", depth={len(depth_bytes)} bytes" if depth_bytes else ", no depth"
                print(f"First frame: {width}x{height}, format={fmt}{depth_info}")

            if width <= 0 or height <= 0:
                continue

            frame_count += 1

            # RGB → BGR for OpenCV saving
            rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((height, width, 3))
            bgr = rgb[:, :, ::-1].copy()
            path = os.path.join(save_dir, f"frame_{frame_count:03d}.jpg")
            cv2.imwrite(path, bgr)
            print(f"Saved: {path}")

            if depth_bytes:
                depth = np.frombuffer(depth_bytes, dtype=np.uint16).reshape((height, width))
                depth_path = os.path.join(save_dir, f"frame_{frame_count:03d}_depth.png")
                depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                cv2.imwrite(depth_path, depth_norm)
                print(f"Saved depth: {depth_path}")

    except KeyboardInterrupt:
        pass

    if frame_count > 0:
        elapsed = time.time() - first_frame_time if first_frame_time else 0
        fps = frame_count / elapsed if elapsed > 0 else 0
        print(f"Total frames: {frame_count}, avg FPS: {fps:.1f}")
    else:
        print("No frames received.")
        sys.exit(1)

    socket.close()
    context.term()


if __name__ == "__main__":
    main()
