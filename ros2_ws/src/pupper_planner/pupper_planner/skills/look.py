"""LookSkill: settle -> fresh frame -> equirect -> Gemini -> objects with ROS-sign headings.

Used by the ``look`` plan step, by go_to_object/find_object, and by the /plan_executor/look service.
Blocking (~1-2 s); call from a worker thread, never from the executor's timer callbacks.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from pupper_vision.equirect import EquirectProjector
from pupper_vision.gemini_look import GeminiLook, boxes_to_objects
from pupper_vision.image_buffer import LatestImage

logger = logging.getLogger("pupper_planner.look")


@dataclass
class LookParams:
    settle_s: float = 0.4
    max_age_s: float = 0.6
    frame_timeout_s: float = 2.5
    camera_height_m: float = 0.2
    default_stop_distance_m: float = 0.8


@dataclass
class LookOutput:
    ok: bool
    text: str = ""
    objects: List[Dict[str, Any]] = field(default_factory=list)
    reason: str = ""

    def objects_json(self) -> str:
        return json.dumps(self.objects)


class LookSkill:
    def __init__(self, latest: LatestImage, projector: EquirectProjector, gemini: GeminiLook, params: LookParams) -> None:
        self.latest = latest
        self.projector = projector
        self.gemini = gemini
        self.params = params
        self._lock = threading.Lock()  # one Gemini call at a time

    def look(
        self,
        prompt: str,
        max_age_s: Optional[float] = None,
        stop_distance_m: Optional[float] = None,
        settle_s: Optional[float] = None,
        cancel: Optional[threading.Event] = None,
    ) -> LookOutput:
        p = self.params
        settle = p.settle_s if settle_s is None else settle_s
        max_age = p.max_age_s if (max_age_s is None or max_age_s <= 0) else max_age_s
        stop_d = p.default_stop_distance_m if stop_distance_m is None else stop_distance_m
        if not self.gemini.available:
            return LookOutput(False, reason="NO_API_KEY")
        t0 = time.monotonic()
        while time.monotonic() - t0 < settle:
            if cancel is not None and cancel.is_set():
                return LookOutput(False, reason="CANCELLED")
            time.sleep(0.02)
        # A frame received after (now - max_age) is fresh enough; otherwise wait for the next one.
        if not self.latest.wait_for_fresh(time.monotonic() - max_age, p.frame_timeout_s):
            return LookOutput(False, reason="NO_CAMERA_FRAME")
        img = self.latest.decode(rgb=True)
        if img is None:
            return LookOutput(False, reason="BAD_FRAME")
        pano = self.projector.project(img)
        with self._lock:
            try:
                res = self.gemini.look(pano, prompt, rgb=True)
            except Exception as e:  # network / API errors
                logger.warning("gemini look failed: %s", e)
                return LookOutput(False, reason=f"GEMINI:{type(e).__name__}")
        objs = boxes_to_objects(res.boxes, self.projector, stop_d, p.camera_height_m)
        logger.info("look took %.2fs: %d objects; %s", time.monotonic() - t0, len(objs), res.text[:120])
        return LookOutput(True, text=res.text, objects=objs)
