import random

from capture_vlm_frames import DISTRACTOR_P, GLASS_HOME, GLASS_JITTER, held_bottle, sample_scene

REST = {'whiskey': 0.90, 'cola': 0.90, 'beer': 0.90}


def test_held_needs_closed_fingers_and_a_lifted_bottle():
    lifted = {'whiskey': 0.95, 'cola': 0.90, 'beer': 0.90}
    assert held_bottle(0.45, lifted, REST) == 'whiskey'
    assert held_bottle(0.05, lifted, REST) is None
    assert held_bottle(0.45, dict(REST), REST) is None


def test_two_bottles_off_their_stands_is_not_a_grasp():
    assert held_bottle(0.45, {'whiskey': 0.95, 'cola': 0.95, 'beer': 0.90}, REST) is None


def test_scenes_stay_in_bounds_and_mix_distractors():
    rng = random.Random(0)
    scenes = [sample_scene(rng) for _ in range(2000)]
    for s in scenes:
        assert abs(s['glass'][0] - GLASS_HOME[0]) <= GLASS_JITTER
        assert abs(s['glass'][1] - GLASS_HOME[1]) <= GLASS_JITTER
        assert set(s['shown']) <= {'whiskey', 'cola'}
    share = sum(s['distractor'] is not None for s in scenes) / len(scenes)
    assert abs(share - DISTRACTOR_P) < 0.03
