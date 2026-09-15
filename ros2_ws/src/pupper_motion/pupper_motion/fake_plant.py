"""Fake robot for developing the motion server and plan executor without hardware.

Simulates: the IMU (yaw integrates the commanded wz through a first-order lag, published as
/imu_sensor_broadcaster/imu at 100 Hz with a fused quaternion + gyro), the controller_manager services
(list_controllers / switch_controller with a real active/inactive state), and the neural controller's
imu latency topic. Run the real cmd_vel_mux alongside it; this node consumes /cmd_vel (the mux output).

    ros2 run pupper_motion fake_plant
"""
import math
import time

import rclpy
from rclpy.node import Node

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

CONTROLLERS = ["neural_controller", "neural_controller_three_legged", "forward_kp_controller", "forward_kd_controller", "forward_position_controller"]


class FakePlant(Node):
    def __init__(self) -> None:
        super().__init__("fake_plant")
        self.declare_parameter("yaw_lag_s", 0.15)
        self.declare_parameter("start_active", False)
        self.declare_parameter("yaw_scale", 1.0)   # simulate a turn calibration error
        self.declare_parameter("imu_pub_hz", 100.0)  # robot's imu_sensor_broadcaster republishes at 260 Hz
        self.declare_parameter("imu_sample_hz", 100.0)  # BNO08x new samples at 100 Hz
        self.yaw_lag = float(self.get_parameter("yaw_lag_s").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.state = {c: "inactive" for c in CONTROLLERS}
        if bool(self.get_parameter("start_active").value):
            self.state["neural_controller"] = "active"
        self.cmd = Twist()
        self.last_cmd_time = 0.0
        self.yaw = 0.0
        self.wz = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.imu_pub = self.create_publisher(Imu, "/imu_sensor_broadcaster/imu", 10)
        self.lat_pub = self.create_publisher(Float32, "/neural_controller/imu_latency_seconds", 10)
        self.create_service(ListControllers, "/controller_manager/list_controllers", self._list)
        self.create_service(SwitchController, "/controller_manager/switch_controller", self._switch)
        self._t = time.monotonic()
        self._last_imu = Imu()
        self._sample_period = 1.0 / float(self.get_parameter("imu_sample_hz").value)
        self._last_sample_t = 0.0
        self.create_timer(1.0 / float(self.get_parameter("imu_pub_hz").value), self._tick)
        self.create_timer(1.0, lambda: self.get_logger().info(f"yaw={math.degrees(self.yaw):.1f} deg wz={self.wz:.2f} active={self.state['neural_controller']}"))

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = msg
        self.last_cmd_time = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self._t
        self._t = now
        target = self.cmd.angular.z if self.state["neural_controller"] == "active" else 0.0
        self.wz += (target - self.wz) * min(1.0, dt / self.yaw_lag)
        self.yaw += self.yaw_scale * self.wz * dt
        if now - self._last_sample_t >= self._sample_period - 1e-4:
            self._last_sample_t = now
            m = Imu()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.orientation.z = math.sin(self.yaw / 2.0)
            m.orientation.w = math.cos(self.yaw / 2.0)
            m.angular_velocity.z = self.wz
            m.linear_acceleration.z = 9.81
            self._last_imu = m
        self.imu_pub.publish(self._last_imu)  # like the real broadcaster: repeats the last sample between updates
        self.lat_pub.publish(Float32(data=0.005))

    def _list(self, _req, res):
        for name, st in self.state.items():
            c = ControllerState()
            c.name = name
            c.state = st
            c.type = "fake"
            res.controller.append(c)
        return res

    def _switch(self, req, res):
        for c in req.deactivate_controllers:
            if c in self.state:
                self.state[c] = "inactive"
        for c in req.activate_controllers:
            if c in self.state:
                self.state[c] = "active"
        self.get_logger().info(f"switch: activate={list(req.activate_controllers)} deactivate={list(req.deactivate_controllers)}")
        res.ok = True
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    n = FakePlant()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
