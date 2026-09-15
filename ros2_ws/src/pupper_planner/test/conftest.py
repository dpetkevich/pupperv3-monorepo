import sys
from unittest.mock import MagicMock

# The rclpy-free modules under test must not import ROS; stub anything that might be pulled in transitively.
for m in ["rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "rclpy.callback_groups", "rclpy.executors",
          "action_msgs", "action_msgs.msg", "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv",
          "sensor_msgs", "sensor_msgs.msg", "controller_manager_msgs", "controller_manager_msgs.srv",
          "pupper_interfaces", "pupper_interfaces.action", "pupper_interfaces.srv", "pupper_interfaces.msg",
          "ament_index_python", "ament_index_python.packages"]:
    sys.modules.setdefault(m, MagicMock())

# make the sibling pupper_vision source tree importable for import smoke tests
import os
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "pupper_vision")))
