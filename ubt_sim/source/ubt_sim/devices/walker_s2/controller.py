from __future__ import annotations

from typing import Any
import json
import struct
import time

import cv2
import numpy as np

from ..device_base import DeviceBase
from .action_process import reset_hold_targets, to_controller_data, to_ros_data

try:
    import zmq

    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False


class WalkerS2Controller(DeviceBase):
    """Controller for Walker S2 that receives ROS2 commands through ZMQ."""

    def __init__(self, env, **kwargs):
        super().__init__()
        self.env = env
        self.device = env.device
        self.reset_requested = False
        self.part_randomization_request: dict[str, Any] | None = None
        self._action: dict[str, Any] = {"body": {}}
        self._jpeg_frame_count = 0
        self.jpeg_unit_test = kwargs.get("jpeg_unit_test", True)

        self._camera_names: list[str] = kwargs.get("camera_names", [])

        self.cmd_port = int(kwargs.get("cmd_port", 5655))
        self.status_port = int(kwargs.get("status_port", 5656))
        self.image_port = int(kwargs.get("image_port", 5657))
        self.jpeg_image_port = int(kwargs.get("jpeg_image_port", 5658))

        if HAS_ZMQ:
            self.context = zmq.Context()

            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.setsockopt(zmq.RCVHWM, 1)
            self.sub_socket.connect(f"tcp://127.0.0.1:{self.cmd_port}")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.SNDHWM, 1)
            self.pub_socket.bind(f"tcp://*:{self.status_port}")

            self.img_socket = self.context.socket(zmq.PUB)
            self.img_socket.setsockopt(zmq.SNDHWM, 4)
            self.img_socket.bind(f"tcp://*:{self.image_port}")

            self.jpeg_socket = self.context.socket(zmq.PUB)
            self.jpeg_socket.setsockopt(zmq.SNDHWM, 1)
            self.jpeg_socket.bind(f"tcp://*:{self.jpeg_image_port}")

            print(f"[INFO] Walker S2 ZMQ command sub connected to tcp://127.0.0.1:{self.cmd_port}")
            print(f"[INFO] Walker S2 ZMQ status pub bound to tcp://*:{self.status_port}")
            print(f"[INFO] Walker S2 ZMQ image pub bound to tcp://*:{self.image_port}")
            print(f"[INFO] Walker S2 ZMQ JPEG pub bound to tcp://*:{self.jpeg_image_port}")
        else:
            print("[WARNING] zmq not found. Walker S2 ZMQ control will not be available.")

    def __str__(self) -> str:
        return "Walker S2 ZMQ Controller"

    def reset(self):
        self._action = {"body": {}}
        self.reset_requested = False
        self.part_randomization_request = None
        reset_hold_targets()

    def add_callback(self, key, func):
        pass

    def _merge_command(self, msg: dict[str, Any]) -> None:
        if msg.get("reset"):
            self.reset_requested = True
            return

        if "randomize_part_sorting_pieces" in msg:
            payload = msg.get("randomize_part_sorting_pieces")
            if payload is True or payload is None:
                payload = {}
            if isinstance(payload, dict):
                self.part_randomization_request = payload
            else:
                print("[WARN] Ignoring invalid part randomization payload; expected object or true.")
            return

        if "body" in msg:
            body = msg.get("body") or {}
            if isinstance(body, dict):
                # Replace rather than update to avoid stale joints from
                # previous messages accumulating when the controller uses
                # publish_changed_only.  HoldTargetManager (action_process)
                # already persists the full set of joint targets; _action
                # only needs to represent the current frame's command.
                self._action["body"] = dict(body)
            else:
                self._action["body"] = body

        for key in ["left_hand", "right_hand", "left_grip", "right_grip"]:
            if key in msg:
                self._action[key] = msg[key]

    def pop_part_randomization_request(self) -> dict[str, Any] | None:
        request = self.part_randomization_request
        self.part_randomization_request = None
        return request

    def _send_status(self) -> None:
        status = to_ros_data(self.env, self._action)
        self.pub_socket.send_json(status, flags=zmq.NOBLOCK)

    def _send_camera_data(self) -> None:
        """Send raw RGB frames for all configured cameras via ZMQ multipart."""
        for cam_name in self._camera_names:
            if cam_name not in self.env.scene.keys():
                continue
            camera = self.env.scene[cam_name]
            try:
                rgb_tensor = camera.data.output.get("rgb") if camera.data.output is not None else None
                if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                    continue
                rgb = rgb_tensor[0].cpu().numpy()
                metadata = {
                    "width": int(rgb.shape[1]),
                    "height": int(rgb.shape[0]),
                    "format": "raw",
                    "camera": cam_name,
                }
                self.img_socket.send_json(metadata, flags=zmq.SNDMORE | zmq.NOBLOCK)
                self.img_socket.send(rgb.tobytes(), flags=zmq.SNDMORE | zmq.NOBLOCK)
                self.img_socket.send(b"", flags=zmq.NOBLOCK)
            except Exception:
                continue

    def _send_jpeg_camera_data(self) -> None:
        """Send JPEG-encoded frames for all configured cameras via ZMQ."""
        for cam_name in self._camera_names:
            if cam_name not in self.env.scene.keys():
                continue
            camera = self.env.scene[cam_name]
            try:
                rgb_tensor = camera.data.output.get("rgb") if camera.data.output is not None else None
                if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                    continue
                rgb = rgb_tensor[0].cpu().numpy()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                ret, buf = cv2.imencode(".jpg", bgr)
                if not ret:
                    continue
                msg = buf.tobytes()
                if self.jpeg_unit_test:
                    msg = struct.pack("dI", time.time(), self._jpeg_frame_count) + msg
                    self._jpeg_frame_count += 1
                self.jpeg_socket.send(msg, flags=zmq.NOBLOCK)
            except Exception:
                continue

    def advance(self) -> dict[str, Any]:
        if HAS_ZMQ:
            while True:
                try:
                    msg = self.sub_socket.recv_json(flags=zmq.NOBLOCK)
                    self._merge_command(msg)
                except zmq.Again:
                    break
                except json.JSONDecodeError:
                    break

            try:
                self._send_status()
            except Exception:
                pass
            try:
                self._send_camera_data()
            except Exception:
                pass
            try:
                self._send_jpeg_camera_data()
            except Exception:
                pass

        return {"walker_s2": to_controller_data(self._action, self.env)}

    def display_controls(self):
        if HAS_ZMQ:
            print("Walker S2 Controller: ROS2 SDK interface enabled via ZMQ bridge")
            print(f"  - Command Sub: tcp://127.0.0.1:{self.cmd_port}")
            print(f"  - Status Pub:  tcp://127.0.0.1:{self.status_port}")
            print(f"  - Image Pub:   tcp://*:{self.image_port} (raw multipart)")
            print(f"  - JPEG Pub:    tcp://*:{self.jpeg_image_port} (image_client compatible)")
