"""Import smoke tests for the rclpy-bound modules (ROS modules are MagicMock-stubbed by conftest)."""
import importlib


def test_import_look_and_ros_skills():
    importlib.import_module("pupper_planner.skills.look")
    importlib.import_module("pupper_planner.skills.ros_skills")
    importlib.import_module("pupper_planner.service_bridge")
    importlib.import_module("pupper_planner.lat")


def test_import_plan_executor():
    # Node is a MagicMock instance under the stubs; if subclassing it is impossible we only check the source compiles.
    try:
        importlib.import_module("pupper_planner.plan_executor")
    except TypeError as e:  # metaclass/subclass of a mock instance
        import py_compile, os
        py_compile.compile(os.path.join(os.path.dirname(__file__), "..", "pupper_planner", "plan_executor.py"), doraise=True)
