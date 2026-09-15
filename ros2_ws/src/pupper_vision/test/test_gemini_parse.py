from pupper_vision.gemini_look import parse_bounding_boxes, strip_json, transform_to_pixels, load_api_key


def test_parse_box_2d_with_fences():
    txt = 'I see a bed on the right.\n```json\n[{"label": "bed", "box_2d": [500, 600, 900, 900], "close": true}]\n```'
    boxes = parse_bounding_boxes(txt)
    assert len(boxes) == 1
    b = boxes[0]
    assert (b.y1, b.x1, b.y2, b.x2) == (500, 600, 900, 900)
    assert b.close is True
    assert strip_json(txt) == "I see a bed on the right."


def test_parse_point_legacy_and_empty():
    assert parse_bounding_boxes('[{"point": [500, 500], "label": "lamp"}]')[0].label == "lamp"
    assert parse_bounding_boxes("Nothing here. []") == []
    assert parse_bounding_boxes("") == []
    assert parse_bounding_boxes("[{broken") == []


def test_transform_to_pixels_clamps():
    from pupper_vision.gemini_look import BoundingBox

    px = transform_to_pixels(BoundingBox(x1=-10, y1=0, x2=1200, y2=500, label="x"), 800, 720)
    assert (px.x1, px.y1, px.x2, px.y2) == (0, 0, 800, 360)


def test_load_api_key_from_file(tmp_path, monkeypatch):
    monkeypatch.delenv("GOOGLE_API_KEY", raising=False)
    f = tmp_path / ".env.local"
    f.write_text("OPENAI_API_KEY=abc\nGOOGLE_API_KEY=\"secret\"\n")
    assert load_api_key(str(f)) == "secret"
    assert load_api_key(str(tmp_path / "missing")) is None
    monkeypatch.setenv("GOOGLE_API_KEY", "envkey")
    assert load_api_key(None) == "envkey"
