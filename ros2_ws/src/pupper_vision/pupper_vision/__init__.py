"""pupper_vision: cached fisheye->equirect projection, Gemini look client, latest-image buffer.

Sign conventions used throughout this package:
- ``fisheye_utils`` headings are image-native: +right.
- ``EquirectProjector.pixel_to_heading_elevation`` returns ROS/REP-103 headings: +left.
Everything downstream (planner, motion server, agent) uses the ROS sign.
"""
