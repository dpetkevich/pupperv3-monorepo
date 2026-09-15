"""Which entity makes rclpy's MultiThreadedExecutor busy-spin? Adds entities one kind at a time and prints CPU%."""
import os, sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from pupper_interfaces.action import TurnDegrees


def cpu_pct(seconds=2.0):
    p = os.getpid()
    def ticks():
        with open(f"/proc/{p}/stat") as f:
            v = f.read().split()
        return int(v[13]) + int(v[14])
    t0, c0 = time.time(), ticks(); time.sleep(seconds); t1, c1 = time.time(), ticks()
    return 100.0 * (c1 - c0) / os.sysconf("SC_CLK_TCK") / (t1 - t0)


def run(label, build, executor_cls, threads=4):
    rclpy.init()
    node = Node("probe")
    keep = build(node)
    ex = executor_cls(num_threads=threads) if executor_cls is MultiThreadedExecutor else executor_cls()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()
    time.sleep(0.5)
    print(f"{label:55s} cpu={cpu_pct():6.1f}%", flush=True)
    ex.shutdown(timeout_sec=0.5); node.destroy_node(); rclpy.shutdown(); time.sleep(0.3)


def timers(n, group=None):
    def b(node):
        return [node.create_timer(0.02, lambda: None, callback_group=group) for _ in range(n)]
    return b

def imu_sub(group=None):
    def b(node):
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        return node.create_subscription(Imu, "/imu_sensor_broadcaster/imu", lambda m: None, qos, callback_group=group)
    return b

def actions(n, group=None):
    def b(node):
        return [ActionServer(node, TurnDegrees, f"/probe/a{i}", execute_callback=lambda gh: TurnDegrees.Result(), callback_group=group) for i in range(n)]
    return b

def combo(*builders):
    def b(node):
        return [f(node) for f in builders]
    return b

MT = MultiThreadedExecutor
ST = SingleThreadedExecutor
print("stream:", sys.argv[1] if len(sys.argv) > 1 else "?")
run("imu sub, MultiThreaded(4)", imu_sub(), MT)
run("imu sub, SingleThreaded", imu_sub(), ST)
run("imu sub + 3 timers, SingleThreaded", combo(imu_sub(), timers(3)), ST)
