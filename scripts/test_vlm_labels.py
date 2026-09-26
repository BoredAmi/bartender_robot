import json

import numpy as np
from PIL import Image

from vlm_labels import LABEL_IDS, bbox, export, is_val, scene_label


def _scene(h=100, w=200):
    labels = np.zeros((h, w), np.uint8)
    labels[10:60, 20:40] = LABEL_IDS['whiskey']
    labels[50:90, 150:170] = LABEL_IDS['glass']
    return labels


def test_bbox_is_normalised_to_1000():
    assert bbox(_scene(), LABEL_IDS['whiskey']) == [100, 100, 200, 600]


def test_sliver_is_not_visible():
    labels = _scene()
    labels[10:60, 20:40] = LABEL_IDS['distractor']
    labels[10:12, 20:40] = LABEL_IDS['whiskey']
    label = scene_label(labels, ['whiskey'], None)
    assert label['bottles'] == [{'name': 'whiskey', 'visible': False, 'bbox': None}]


def test_distractor_against_a_target_is_an_obstruction():
    labels = _scene()
    assert not scene_label(labels, ['whiskey'], None)['obstruction']
    labels[10:60, 41:60] = LABEL_IDS['distractor']
    assert scene_label(labels, ['whiskey'], None)['obstruction']


def test_distractor_far_away_is_not_an_obstruction():
    labels = _scene()
    labels[70:95, 60:90] = LABEL_IDS['distractor']
    assert not scene_label(labels, ['whiskey'], None)['obstruction']


def test_export_keeps_each_scene_in_one_split(tmp_path):
    raw = tmp_path / 'raw'
    for i in range(40):
        scene = raw / f's{i:03d}'
        scene.mkdir(parents=True)
        (scene / 'scene.json').write_text(json.dumps({'bottles': ['whiskey']}))
        for frame in ('0000', '0001'):
            (scene / f'{frame}.json').write_text(json.dumps({'in_gripper': 'whiskey'}))
            Image.fromarray(_scene()).save(scene / f'{frame}_overhead_labels.png')

    counts = export(raw, tmp_path / 'out')

    assert counts['train'] + counts['val'] == 80
    val_scenes = {r['images'][0].split('/')[0]
                  for r in map(json.loads, (tmp_path / 'out' / 'val.jsonl').read_text().splitlines())}
    assert val_scenes == {f's{i:03d}' for i in range(40) if is_val(f's{i:03d}')}
    first = json.loads((tmp_path / 'out' / 'train.jsonl').read_text().splitlines()[0])
    answer = json.loads(first['messages'][1]['content'][0]['text'])
    assert answer['in_gripper'] == 'whiskey'
    assert first['images'][0].endswith('_overhead_rgb.png')
