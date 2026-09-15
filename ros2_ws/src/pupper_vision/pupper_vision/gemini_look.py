"""Gemini "look": one image + prompt -> free text + labelled boxes with ROS-sign headings.

Ported from the agent's gemini_interface.py / gemini_utils.py. Boxes are normalized 0-1000 in
``[ymin, xmin, ymax, xmax]`` order; ``transform_to_pixels`` converts to equirect pixels.
"""
from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np

logger = logging.getLogger("pupper_vision.gemini_look")

MODEL_NAME = "gemini-3.5-flash-lite"
DEFAULT_ENV_FILE = "/home/pi/pupperv3-monorepo/ai/llm-ui/agent-starter-python/.env.local"
THUMBNAIL_PX = 800

SYSTEM_INSTRUCTIONS = """You are the eyes of a small robot dog. The image is an equirectangular panorama from a fisheye
camera mounted 20 cm above the floor: the horizontal centre of the image is straight ahead, the left edge is 90 degrees
to the robot's left, the right edge 90 degrees to its right, the vertical centre is the horizon.
Answer the prompt in one or two short sentences, then on a new line return ONLY a JSON array of the objects the prompt
asks about (max 10). Each element: {"label": str, "box_2d": [ymin, xmin, ymax, xmax] with coordinates normalized to
0-1000, "close": true if the object is within about 1 metre of the robot else false}.
If nothing matching is visible return an empty array []. Never return masks or code fences."""


@dataclass
class BoundingBox:
    """Normalized 0-1000 box, image space (x left->right, y top->bottom)."""

    x1: float
    y1: float
    x2: float
    y2: float
    label: str
    close: Optional[bool] = None


@dataclass
class PixelBoundingBox:
    x1: int
    y1: int
    x2: int
    y2: int
    label: str
    close: Optional[bool] = None


@dataclass
class LookResult:
    text: str
    boxes: List[BoundingBox]
    raw: str = ""


def load_api_key(path: Optional[str] = None) -> Optional[str]:
    """GOOGLE_API_KEY from the environment, else from a dotenv-style file (the agent's .env.local)."""
    key = os.environ.get("GOOGLE_API_KEY")
    if key:
        return key
    path = path or DEFAULT_ENV_FILE
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                if k.strip() == "GOOGLE_API_KEY":
                    return v.strip().strip('"').strip("'") or None
    except OSError:
        logger.warning("could not read API key file %s", path)
    return None


def _strip_fences(text: str) -> str:
    text = text.strip()
    text = re.sub(r"```(?:json)?\s*", "", text)
    return text.replace("```", "")


def parse_bounding_boxes(response_text: str) -> List[BoundingBox]:
    """Lenient parse: finds the first JSON array of objects; accepts ``box_2d`` or legacy ``point``."""
    if not response_text:
        return []
    cleaned = _strip_fences(response_text)
    m = re.search(r"\[\s*\{.*?\}\s*\]", cleaned, re.DOTALL)
    if not m:
        return []
    try:
        items = json.loads(m.group(0))
    except json.JSONDecodeError:
        logger.warning("gemini returned unparseable JSON: %s", m.group(0)[:200])
        return []
    boxes: List[BoundingBox] = []
    for it in items:
        if not isinstance(it, dict) or "label" not in it:
            continue
        close = it.get("close")
        close = bool(close) if isinstance(close, bool) else None
        if "box_2d" in it and isinstance(it["box_2d"], list) and len(it["box_2d"]) == 4:
            y1, x1, y2, x2 = (float(c) for c in it["box_2d"])
            boxes.append(BoundingBox(x1=x1, y1=y1, x2=x2, y2=y2, label=str(it["label"]), close=close))
        elif "point" in it and isinstance(it["point"], list) and len(it["point"]) == 2:
            y, x = (float(c) for c in it["point"])
            boxes.append(BoundingBox(x1=x - 5, y1=y - 5, x2=x + 5, y2=y + 5, label=str(it["label"]), close=close))
    return boxes


def strip_json(response_text: str) -> str:
    """The free-text part of a response with the JSON array removed."""
    cleaned = _strip_fences(response_text or "")
    cleaned = re.sub(r"\[\s*(\{.*?\}\s*)?\]", "", cleaned, flags=re.DOTALL)
    return " ".join(cleaned.split()).strip()


def transform_to_pixels(bbox: BoundingBox, image_width: int, image_height: int, input_range: int = 1000) -> PixelBoundingBox:
    def cx(val, size):
        return max(0, min(int(val / input_range * size), size))

    return PixelBoundingBox(
        x1=cx(bbox.x1, image_width),
        y1=cx(bbox.y1, image_height),
        x2=cx(bbox.x2, image_width),
        y2=cx(bbox.y2, image_height),
        label=bbox.label,
        close=bbox.close,
    )


def boxes_to_objects(boxes: List[BoundingBox], projector, stop_distance_m: float, camera_height_m: float = 0.2) -> List[Dict[str, Any]]:
    """Boxes -> objects_json dicts. Heading is ROS sign (+left). ``close`` = Gemini close OR floor range <= stop."""
    out: List[Dict[str, Any]] = []
    for b in boxes:
        px = transform_to_pixels(b, projector.width, projector.height)
        cu = (px.x1 + px.x2) / 2.0
        cv_ = (px.y1 + px.y2) / 2.0
        heading, elevation = projector.pixel_to_heading_elevation(cu, cv_)
        _, bottom_elev = projector.pixel_to_heading_elevation(cu, px.y2)
        dist = projector.floor_distance(bottom_elev, camera_height_m)
        close = bool(b.close) or (dist is not None and dist <= stop_distance_m)
        out.append(
            {
                "label": b.label,
                "heading_deg": round(heading, 1),
                "elevation_deg": round(elevation, 1),
                "close": close,
                "distance_est_m": None if dist is None else round(dist, 2),
                "bbox": [px.x1, px.y1, px.x2, px.y2],
            }
        )
    return out


class GeminiLook:
    """Sync Gemini client. Call ``look`` from a worker thread (it blocks ~1-2 s)."""

    def __init__(self, api_key: Optional[str], model: str = MODEL_NAME, temperature: float = 0.3) -> None:
        self.model = model
        self.temperature = temperature
        self._client = None
        if api_key:
            from google import genai  # imported lazily so tests do not need the SDK

            self._client = genai.Client(api_key=api_key)
        else:
            logger.error("GeminiLook created without an API key; look() will fail")

    @property
    def available(self) -> bool:
        return self._client is not None

    def look(self, image: np.ndarray, prompt: str, rgb: bool = True) -> LookResult:
        if self._client is None:
            raise RuntimeError("Gemini API key not configured")
        from PIL import Image
        from google.genai import types

        arr = image if rgb else image[:, :, ::-1]
        pil = Image.fromarray(np.ascontiguousarray(arr))
        pil.thumbnail((THUMBNAIL_PX, THUMBNAIL_PX))
        resp = self._client.models.generate_content(
            model=self.model,
            contents=[pil, prompt],
            config=types.GenerateContentConfig(temperature=self.temperature, system_instruction=SYSTEM_INSTRUCTIONS),
        )
        raw = resp.text or ""
        return LookResult(text=strip_json(raw), boxes=parse_bounding_boxes(raw), raw=raw)
