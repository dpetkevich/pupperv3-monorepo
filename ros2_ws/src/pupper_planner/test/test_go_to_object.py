import threading

from pupper_planner.skills.go_to_object import filter_label, find_object, go_to_object


class World:
    """Scripted looks: each call to look() pops the next object list."""

    def __init__(self, looks):
        self.looks = list(looks)
        self.turns = []
        self.moves = []

    def look(self, label):
        return self.looks.pop(0) if self.looks else []

    def turn(self, deg):
        self.turns.append(deg)
        return True, "", deg

    def move(self, m):
        self.moves.append(m)
        return True, "", m


def obj(h, d=None, close=False):
    return {"label": "bed", "heading_deg": h, "distance_est_m": d, "close": close}


def test_absent_then_search_then_walk_and_arrive():
    w = World([[], [], [obj(30, 2.5)], [obj(2, 1.2)], [obj(0, 0.6)]])
    r = go_to_object("bed", w.look, w.turn, w.move, threading.Event(), stop_distance_m=0.8)
    assert r.ok and r.reason == "ARRIVED"
    assert w.turns == [60.0, 30]            # one search step, then the correction toward the bed
    assert w.moves == [1.5, 0.4]            # clamp(2.5-0.8)=1.5 then clamp(1.2-0.8)=0.4
    assert "reached the bed" in r.summary


def test_found_off_axis_turns_and_within_tolerance_does_not():
    w = World([[obj(-20, 3.0)], [obj(5, 0.5)]])
    r = go_to_object("bed", w.look, w.turn, w.move, threading.Event())
    assert r.ok and w.turns == [-20]


def test_close_flag_from_gemini_arrives_without_distance():
    w = World([[obj(0, None, close=True)]])
    r = go_to_object("bed", w.look, w.turn, w.move, threading.Event())
    assert r.ok and w.moves == [] and r.summary == "reached the bed"


def test_no_distance_uses_default_push():
    w = World([[obj(0)], [obj(0, None, True)]])
    go_to_object("bed", w.look, w.turn, w.move, threading.Event())
    assert w.moves == [0.8]


def test_not_found_after_full_search():
    w = World([])
    r = go_to_object("bed", w.look, w.turn, w.move, threading.Event())
    assert not r.ok and r.reason == "NOT_FOUND" and w.turns == [60.0] * 6 and w.moves == []
    assert "could not find the bed" in r.summary


def test_iteration_cap():
    w = World([[obj(0, 5.0)]] * 10)
    r = go_to_object("bed", w.look, w.turn, w.move, threading.Event(), max_iterations=3)
    assert not r.ok and r.reason == "MAX_ITERATIONS" and len(w.moves) == 3


def test_cancel_and_child_failure():
    ev = threading.Event()
    ev.set()
    r = go_to_object("bed", World([]).look, None, None, ev)
    assert r.reason == "CANCELLED"

    w = World([[obj(40, 3.0)]])
    w.turn = lambda d: (False, "ESTOP", 0.0)
    r = go_to_object("bed", w.look, w.turn, w.move, threading.Event())
    assert not r.ok and r.reason == "ESTOP"


def test_find_object_stops_at_360():
    w = World([])
    r = find_object("bed", w.look, w.turn, threading.Event(), search_step_deg=90)
    assert not r.ok and w.turns == [90, 90, 90, 90]


def test_filter_label():
    objs = [{"label": "queen bed"}, {"label": "wooden chair"}, {"label": "bedside lamp"}]
    assert [o["label"] for o in filter_label(objs, "bed")] == ["queen bed"]
    assert [o["label"] for o in filter_label(objs, "the chair")] == ["wooden chair"]
