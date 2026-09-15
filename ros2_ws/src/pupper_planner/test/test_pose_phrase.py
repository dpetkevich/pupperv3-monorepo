from pupper_planner.phrases import pose_phrase


def test_pose_phrase_reads_after_im():
    f = pose_phrase
    assert f(0.1, 0.0, 5) == "right where I started, facing the same way as when I started"
    assert f(1.8, 0.0, 175) == "about 1.8 metres from where I started, facing the other way"
    assert f(0.5, 0.3, -90) == "less than a metre from where I started, turned about 90 degrees to the right of my starting heading"
    assert f(3.0, 4.0, 400) == "about 5.0 metres from where I started, turned about 40 degrees to the left of my starting heading"
