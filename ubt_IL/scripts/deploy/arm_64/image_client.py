#!/usr/bin/env python3
"""无界面相机客户端 - 验证 image_server (ZMQ 5558 PUB) -> 客户端 通路。

不使用 cv2.imshow，无需图形界面 / DISPLAY，适合 SSH 环境。
跑在 env_vla（与 LeRobot 同环境），验证的正是 LeRobot ImageServerCamera 连 image_server
的通路。

image_server 默认 Unit_Test=True，每帧消息格式：
    struct.pack('dI', timestamp, frame_id) + JPEG   # 12 字节头 + JPEG
"""
import argparse
import logging
import struct
import time
from collections import deque

import cv2
import numpy as np
import zmq

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s: %(message)s")
log = logging.getLogger("camera_client")

HEADER = struct.calcsize("dI")  # 12: double timestamp + uint32 frame_id


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--address", default="127.0.0.1", help="image_server 地址 (默认本机)")
    ap.add_argument("--port", type=int, default=5558)
    ap.add_argument("--count", type=int, default=0, help="收够 N 帧后退出 (0=持续, Ctrl-C 停)")
    ap.add_argument(
        "--no-header",
        action="store_true",
        help="server 不带 12B 头 (默认带, 匹配 image_server Unit_Test=True)",
    )
    ap.add_argument(
        "--show",
        action="store_true",
        help="弹窗显示画面 (cv2.imshow, 需 X11 转发: ssh -X / -Y, 或本机 DISPLAY=:0)",
    )
    args = ap.parse_args()

    if args.show and not __import__("os").environ.get("DISPLAY"):
        log.warning("启用了 --show 但 DISPLAY 未设置！X11 转发未开，cv2.imshow 会崩溃。")
        log.warning("解决: 用 ssh -X/-Y 登录, 或本机 'export DISPLAY=:0' 后再运行。")

    with_header = not args.no_header

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://{args.address}:{args.port}")
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    log.info(f"连接 tcp://{args.address}:{args.port} (SUB) | header={with_header} | 等待帧...")

    window = 1.0  # 滑动窗口(秒)用于实时 fps
    recv_times = deque()
    lats = deque()
    n = 0
    lost = 0
    last_fid = None
    t0 = time.time()

    try:
        while True:
            msg = sock.recv()
            t_recv = time.time()
            fid = None
            lat = None
            jpg = msg
            if with_header:
                if len(msg) < HEADER:
                    log.warning(f"消息过短({len(msg)}B)，跳过")
                    continue
                ts, fid = struct.unpack("dI", msg[:HEADER])
                jpg = msg[HEADER:]
                lat = (t_recv - ts) * 1000.0
                if last_fid is not None:
                    gap = fid - last_fid - 1
                    if gap > 0:
                        lost += gap
                        log.warning(f"丢帧 {gap} (fid {last_fid}->{fid})")
                last_fid = fid

            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                log.warning("JPEG 解码失败")
                continue

            n += 1
            recv_times.append(t_recv)
            lats.append(lat)
            while recv_times and recv_times[0] < t_recv - window:
                recv_times.popleft()
                lats.popleft()

            if n <= 3 or n % 30 == 0:
                fps = len(recv_times) / window
                latstr = f"lat={lat:.1f}ms fid={fid}" if lat is not None else ""
                log.info(f"#{n} shape={img.shape[1]}x{img.shape[0]} fps≈{fps:.1f} {latstr}")

            if args.show:
                cv2.imshow("camera (press q to quit)", img)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    log.info("按 q 退出")
                    break

            if args.count and n >= args.count:
                break
    except KeyboardInterrupt:
        log.info("用户中断")
    finally:
        sock.close(linger=0)
        if args.show:
            cv2.destroyAllWindows()
        dur = time.time() - t0
        fps = len(recv_times) / window if recv_times else (n / dur if dur > 0 else 0)
        valid_lats = [l for l in lats if l is not None]
        avg_lat = sum(valid_lats) / len(valid_lats) if valid_lats else None
        summary = (
            f"— 总结 — 帧数={n}  时长={dur:.1f}s  fps≈{fps:.1f}  丢帧≈{lost}"
            + (f"  平均延迟≈{avg_lat:.1f}ms" if avg_lat is not None else "")
        )
        log.info(summary)
        print(
            "=> 相机通路 OK ✓"
            if n > 0
            else "=> 未收到任何帧 ✗ (检查 image_server 是否已启动 / 地址端口 / 相机)"
        )


if __name__ == "__main__":
    main()
