"""Animation catalogue shared by the agent tools and the executor bridge.

Kept free of livekit/rclpy imports so it can be used from any process.
"""

import logging
from pathlib import Path

logger = logging.getLogger("animations")


# Animation name mapping with descriptions for the AI assistant
ANIMATION_NAMES = {
    "twerk": {
        "csv_name": "twerk_recording_2025-09-04_16-14-51_0",
        "description": "Makes the robot twerk by moving its hips in a rhythmic motion",
    },
    "lie_sit_lie": {
        "csv_name": "lie_sit_lie_recording_2025-09-03_12-44-08_0",
        "description": "From lying position, sits up and then lies back down",
    },
    "stand_sit_shake_sit_stand": {
        "csv_name": "stand_sit_shake_sit_stand_recording_2025-09-03_12-47-18_0",
        "description": "From standing, sits down, shakes body, sits, then stands back up",
    },
    "upward_dog": {
        "csv_name": "upward_dog_recording_2025-10-22_17-17-07",
        "description": "From lying position, moves into an upward dog yoga pose and back down to lying",
    },
    # "stand_sit_stand": {
    #     "csv_name": "stand_sit_stand_recording_2025-09-03_12-46-36_0",
    #     "description": "From standing position, sits down and then stands back up",
    # },
    "superman": {
        "csv_name": "superman_recording_2025-10-22_17-47-41",
        "description": "From lying position, lifts arms and legs off the ground to mimic flying like Superman",
    },
    "pee": {
        "csv_name": "pee2_recording_2025-10-22_17-41-45",
        "description": "From standing position, lifts leg and mimics urination motion",
    },
    "lie_downward_dog": {
        "csv_name": "lie_downward_dog_recording_2025-09-04_16-08-00_0",
        "description": "From lying position, moves into a downward dog yoga pose",
    },
    # "stand_downward_dog": {
    #     "csv_name": "stand_downward_dog_recording_2025-09-04_16-09-51_0",
    #     "description": "From standing position, moves into a downward dog yoga pose",
    # },
    # "push_up": {
    #     "csv_name": "push_up_recording_2025-09-04_16-11-34_0",
    #     "description": "From standing position, performs a push-up motion by lowering and raising the body",
    # },
    "sneeze": {
        "csv_name": "sneeze_recording_2025-09-04_16-13-54_0",
        "description": "From standing position, mimics a sneezing motion with head and body movement",
    },
    "spider": {
        "csv_name": "spider_recording_2025-09-04_16-12-38_0",
        "description": "From lying position, moves legs in a silly spider-like motion",
    },
    "swim": {
        "csv_name": "swim_recording_2025-09-04_16-10-45_0",
        "description": "From lying position, performs silly swimming motions with the legs",
    },
}


# Animation playback frame rate constant (Hz)
ANIMATION_FRAME_RATE = 40.0
FADE_IN_DURATION = 1.0


def get_animation_duration(csv_filename: str) -> float:
    """Calculate animation duration from CSV file based on frame count.

    Args:
        csv_filename: Name of the CSV file (without path)

    Returns:
        Duration in seconds based on frame count and ANIMATION_FRAME_RATE
    """
    base_path = (
        Path(__file__).parent.parent.parent.parent.parent
        / "ros2_ws"
        / "src"
        / "animation_controller_py"
        / "launch"
        / "animations"
    )
    csv_path = base_path / f"{csv_filename}.csv"

    try:
        with open(csv_path, "r") as f:
            # Count rows excluding header
            row_count = sum(1 for _ in f) - 1  # Subtract 1 for header

            if row_count <= 0:
                raise ValueError(f"Animation CSV {csv_filename} is empty or has no data rows.")

            # Calculate duration based on frame count and frame rate
            duration = row_count / ANIMATION_FRAME_RATE + FADE_IN_DURATION

            logger.info(
                f"Animation {csv_filename} duration: {duration:.2f} seconds ({row_count} frames at {ANIMATION_FRAME_RATE} Hz)"
            )
            return duration

    except Exception as e:
        logger.warning(f"Could not calculate duration for animation {csv_filename}: {e}")
        # Fallback: estimate based on typical frame count and rate
        # Most animations seem to be around 10-15 seconds
        return 0.1
