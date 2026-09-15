import math

import numpy as np

from pupper_vision.equirect import EquirectProjector
from pupper_vision.fisheye_utils import convert_boxes_to_elevation_heading
from pupper_vision.gemini_look import BoundingBox, boxes_to_objects


def make_projector():
    return EquirectProjector(width=800, height=720, h_fov_deg=180.0, v_fov_deg=180.0)


def test_centre_pixel_is_straight_ahead():
    p = make_projector()
    heading, elevation = p.pixel_to_heading_elevation((p.width - 1) / 2, (p.height - 1) / 2)
    assert abs(heading) < 1e-6
    assert abs(elevation) < 1e-6


def test_heading_sign_is_ros_left_positive():
    p = make_projector()
    heading_left, _ = p.pixel_to_heading_elevation(0, (p.height - 1) / 2)
    heading_right, _ = p.pixel_to_heading_elevation(p.width - 1, (p.height - 1) / 2)
    assert heading_left > 0 and abs(heading_left - 90.0) < 1e-6
    assert heading_right < 0 and abs(heading_right + 90.0) < 1e-6


def test_elevation_sign_is_up_positive():
    p = make_projector()
    _, top = p.pixel_to_heading_elevation(0, 0)
    _, bottom = p.pixel_to_heading_elevation(0, p.height - 1)
    assert top > 0 and bottom < 0


def test_floor_distance():
    assert abs(EquirectProjector.floor_distance(-45.0, 0.2) - 0.2) < 1e-9  # tan 45 = 1
    assert abs(EquirectProjector.floor_distance(-10.0, 0.2) - 0.2 / math.tan(math.radians(10))) < 1e-9
    assert EquirectProjector.floor_distance(-2.0) is None
    assert EquirectProjector.floor_distance(5.0) is None


def test_project_shape():
    p = make_projector()
    img = np.zeros((1050, 1400, 3), dtype=np.uint8)
    out = p.project(img)
    assert out.shape == (720, 800, 3)


def test_convert_boxes_fixed_signature():
    class B:
        x1, y1, x2, y2, label = 0, 0, 799, 719, "thing"

    objs = convert_boxes_to_elevation_heading([B()], 800, 720, 180.0, 180.0)
    assert abs(objs[0]["heading_deg"]) < 1e-6 and abs(objs[0]["elevation_deg"]) < 1e-6


def test_boxes_to_objects_close_from_floor_distance():
    p = make_projector()
    # box bottom edge at row ~ 700 of 720 -> elevation ~ -85 deg -> range ~ 0.02 m -> close
    b = BoundingBox(x1=450, y1=800, x2=550, y2=975, label="bed", close=False)
    objs = boxes_to_objects([b], p, stop_distance_m=0.8)
    assert objs[0]["label"] == "bed"
    assert objs[0]["close"] is True
    assert objs[0]["distance_est_m"] is not None and objs[0]["distance_est_m"] < 0.8
    assert abs(objs[0]["heading_deg"]) < 0.5


def test_boxes_to_objects_heading_sign():
    p = make_projector()
    left = BoundingBox(x1=0, y1=400, x2=100, y2=500, label="left")
    right = BoundingBox(x1=900, y1=400, x2=1000, y2=500, label="right")
    objs = boxes_to_objects([left, right], p, stop_distance_m=0.8)
    assert objs[0]["heading_deg"] > 0 and objs[1]["heading_deg"] < 0
