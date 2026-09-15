# pupper_vision

Library (no node): fisheye → equirectangular projection with a cached remap (`equirect.py`, 800×720, 180°×180°, ROS heading sign), the Gemini "look" client (`gemini_look.py`, `gemini-3.5-flash-lite`, boxes normalised 0–1000), floor-contact distance estimate, and a latest-frame buffer (`image_buffer.py`). Used by `pupper_planner`; the Hailo detector keeps its own copy of `fisheye_utils.py` for now.
