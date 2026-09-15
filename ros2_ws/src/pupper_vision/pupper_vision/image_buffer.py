"""LatestImage: keep the newest CompressedImage from a topic and decode it on demand."""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class LatestImage:
    def __init__(self, node, topic: str = "/camera/image_raw/compressed") -> None:
        self._node = node
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._msg: Optional[CompressedImage] = None
        self._mono: float = 0.0  # monotonic time at reception
        self._count = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self._sub = node.create_subscription(CompressedImage, topic, self._on_msg, qos)

    def _on_msg(self, msg: CompressedImage) -> None:
        with self._cv:
            self._msg = msg
            self._mono = time.monotonic()
            self._count += 1
            self._cv.notify_all()

    @property
    def age_s(self) -> float:
        with self._lock:
            return float("inf") if self._msg is None else time.monotonic() - self._mono

    def wait_for_fresh(self, min_stamp_monotonic: float, timeout_s: float) -> bool:
        """Block until a frame received after ``min_stamp_monotonic`` exists. True if one arrived."""
        deadline = time.monotonic() + timeout_s
        with self._cv:
            while not (self._msg is not None and self._mono >= min_stamp_monotonic):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cv.wait(remaining)
            return True

    def latest(self) -> Optional[CompressedImage]:
        with self._lock:
            return self._msg

    def decode(self, rgb: bool = True) -> Optional[np.ndarray]:
        """Decode the newest frame with cv2 (BGR by default from imdecode; converted to RGB when ``rgb``)."""
        import cv2

        msg = self.latest()
        if msg is None:
            return None
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return None
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if rgb else img
