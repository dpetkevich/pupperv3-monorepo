import threading
import time
from types import SimpleNamespace

from pupper_planner import action_bridge
from pupper_planner.action_bridge import send_and_wait


class FakeFuture:
    def __init__(self):
        self._cbs = []
        self._result = None
        self.done = False

    def add_done_callback(self, cb):
        if self.done:
            cb(self)
        else:
            self._cbs.append(cb)

    def set_result(self, r):
        self._result = r
        self.done = True
        for cb in self._cbs:
            cb(self)

    def result(self):
        return self._result


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = FakeFuture()
        self.cancelled = False

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancelled = True
        f = FakeFuture()
        f.set_result(None)
        # the server finishes the goal as CANCELED shortly after
        threading.Timer(0.05, lambda: self.result_future.set_result(SimpleNamespace(status=5, result="cancel-result"))).start()
        return f


class FakeClient:
    def __init__(self, handle, ready=True):
        self.handle = handle
        self.ready = ready
        self.goal_future = FakeFuture()

    def server_is_ready(self):
        return self.ready

    def wait_for_server(self, timeout_sec):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        self.fb = feedback_callback
        threading.Timer(0.01, lambda: self.goal_future.set_result(self.handle)).start()
        return self.goal_future


def test_success_with_feedback():
    h = FakeHandle()
    c = FakeClient(h)
    got = []
    threading.Timer(0.05, lambda: c.fb(SimpleNamespace(feedback="fb1"))).start()
    threading.Timer(0.1, lambda: h.result_future.set_result(SimpleNamespace(status=4, result="ok"))).start()
    st, res = send_and_wait(c, "goal", got.append, time.monotonic() + 2, threading.Event())
    assert (st, res) == ("SUCCEEDED", "ok") and got == ["fb1"]


def test_timeout_cancels_child():
    h = FakeHandle()
    c = FakeClient(h)
    t0 = time.monotonic()
    st, res = send_and_wait(c, "goal", None, time.monotonic() + 0.2, threading.Event())
    assert st == "TIMEOUT" and h.cancelled and res == "cancel-result"
    assert time.monotonic() - t0 < 1.5


def test_cancel_event_cancels_child():
    h = FakeHandle()
    c = FakeClient(h)
    ev = threading.Event()
    threading.Timer(0.1, ev.set).start()
    st, _ = send_and_wait(c, "goal", None, time.monotonic() + 5, ev)
    assert st == "CANCELED" and h.cancelled


def test_rejected_and_unavailable():
    st, _ = send_and_wait(FakeClient(FakeHandle(accepted=False)), "g", None, time.monotonic() + 1, None)
    assert st == "REJECTED"
    st, _ = send_and_wait(FakeClient(FakeHandle(), ready=False), "g", None, time.monotonic() + 1, None)
    assert st == "UNAVAILABLE"


def test_aborted_status():
    h = FakeHandle()
    c = FakeClient(h)
    threading.Timer(0.05, lambda: h.result_future.set_result(SimpleNamespace(status=6, result="r"))).start()
    st, res = send_and_wait(c, "goal", None, time.monotonic() + 2, None)
    assert st == "ABORTED" and res == "r"
