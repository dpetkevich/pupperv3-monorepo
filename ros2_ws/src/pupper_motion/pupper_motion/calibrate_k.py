"""Tape-measure calibration for the pose integrator (k_vx, k_vy, sigma_frac) and a yaw sanity check.

Usage (on the robot, joystick idle, hard floor, a tape and floor marks ready):
    ros2 run pupper_motion calibrate_k            # interactive
    ros2 run pupper_motion calibrate_k --write ros2_ws/src/neural_controller/launch/config.yaml

Procedure: MoveMeters 2.0 x3 and 1.0 x3 (you tape each), backward 1.5, strafe +1.0 and -1.0, then
TurnDegrees 90/180/360 against floor marks. Prints k_vx, k_vy, sigma_frac and the yaw error.
"""
import argparse
import math
import re
import statistics
import sys
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pupper_interfaces.action import MoveMeters, TurnDegrees


class Calib(Node):
    def __init__(self) -> None:
        super().__init__("calibrate_k")
        self.move = ActionClient(self, MoveMeters, "/motion_server/move_meters")
        self.turn = ActionClient(self, TurnDegrees, "/motion_server/turn_degrees")

    def run(self, client, goal):
        client.wait_for_server(timeout_sec=5.0)
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("goal rejected (busy / estop / invalid?)")
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result


def ask(prompt: str) -> Optional[float]:
    while True:
        s = input(prompt).strip().lower()
        if s in ("s", "skip", ""):
            return None
        try:
            return float(s)
        except ValueError:
            print("number, or 's' to skip")


def ratio_stats(pairs: List[tuple]) -> tuple:
    ratios = [m / c for c, m in pairs if c]
    return (statistics.mean(ratios), statistics.pstdev(ratios) / statistics.mean(ratios) if len(ratios) > 1 else 0.0, ratios)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", help="config.yaml to update in place (motion_server: block)")
    ap.add_argument("--quick", action="store_true", help="one 2 m move, one 90 deg turn")
    args = ap.parse_args(argv)
    rclpy.init()
    n = Calib()
    fwd: List[tuple] = []
    lat: List[tuple] = []
    turns: List[tuple] = []
    plan = [(2.0, 0.0)] if args.quick else [(2.0, 0.0)] * 3 + [(1.0, 0.0)] * 3 + [(-1.5, 0.0), (0.0, 1.0), (0.0, -1.0)]
    try:
        for dx, dy in plan:
            input(f"\nMark the robot's position, then press Enter to run MoveMeters dx={dx} dy={dy} ...")
            r = n.run(n.move, MoveMeters.Goal(dx=float(dx), dy=float(dy), speed_mps=0.0, hold_heading=True))
            if r is None or not r.success:
                print(f"move failed: {getattr(r, 'reason', '?')}")
                continue
            print(f"server thinks it moved {r.moved_m:.2f} m (sigma {r.sigma_m:.2f}), heading drift {r.heading_drift_deg:.1f} deg")
            meas = ask("tape-measured distance in metres (s to skip): ")
            if meas is None:
                continue
            (fwd if dy == 0.0 else lat).append((abs(dx) if dy == 0.0 else abs(dy), meas))
        for deg in ([90.0] if args.quick else [90.0, 180.0, 360.0]):
            input(f"\nNote the heading (floor mark), then press Enter to run TurnDegrees {deg} ...")
            r = n.run(n.turn, TurnDegrees.Goal(degrees=float(deg), speed_dps=0.0))
            if r is None or not r.success:
                print(f"turn failed: {getattr(r, 'reason', '?')}")
                continue
            print(f"IMU says turned {r.turned_degrees:.1f} deg (error {r.error_degrees:.1f})")
            meas = ask("measured turn in degrees against the floor mark (s to skip): ")
            if meas is not None:
                turns.append((deg, meas))
    finally:
        rclpy.shutdown()

    print("\n=== results ===")
    k_vx = k_vy = None
    sigma_frac = None
    if fwd:
        k_vx, sigma_frac, ratios = ratio_stats(fwd)
        print(f"k_vx = {k_vx:.3f}  sigma_frac = {sigma_frac:.3f}  (ratios {['%.2f' % r for r in ratios]})")
        one = [m / c for c, m in fwd if c == 1.0]
        two = [m / c for c, m in fwd if c == 2.0]
        if one and two and abs(statistics.mean(one) - statistics.mean(two)) / statistics.mean(two) > 0.10:
            print("1 m and 2 m ratios differ by > 10 %: adjust lag_tau (bigger if short moves come up short)")
    if lat:
        k_vy, _, ratios = ratio_stats(lat)
        print(f"k_vy = {k_vy:.3f}  (ratios {['%.2f' % r for r in ratios]})")
    for deg, meas in turns:
        err = meas - deg
        flag = "OK" if abs(err) <= 5.0 else "BAD -> consider yaw_alpha: 0.0 (gyro only)"
        print(f"turn {deg:.0f}: measured {meas:.0f} (error {err:+.1f}) {flag}")

    if args.write and (k_vx or k_vy or sigma_frac):
        text = open(args.write).read()

        def sub(key, val):
            nonlocal text
            pat = re.compile(rf"^(\s*{key}:\s*)[-0-9.]+", re.M)
            if pat.search(text):
                text = pat.sub(lambda m: f"{m.group(1)}{val:.3f}", text, count=1)
            else:
                print(f"{key} not found in {args.write}; add it under motion_server: ros__parameters:")

        if k_vx:
            sub("k_vx", k_vx)
        if k_vy:
            sub("k_vy", k_vy)
        if sigma_frac is not None:
            sub("sigma_frac", max(0.1, min(0.6, sigma_frac)))
        open(args.write, "w").write(text)
        print(f"updated {args.write}")


if __name__ == "__main__":
    main(sys.argv[1:])
