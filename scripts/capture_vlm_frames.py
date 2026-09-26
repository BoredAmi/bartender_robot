#!/usr/bin/env python3
"""Capture randomized bar scenes from the running sim for the VLM dataset (see vlm_labels.py).

Run inside the sim container while bartender_sim.launch.py is up:
    docker exec bartender-robot bash -lc \\
        "source ros2_ws/install/setup.bash && python3 scripts/capture_vlm_frames.py data/vlm_raw --scenes 400"
"""
import argparse
import json
import math
import random
import subprocess
import time
from pathlib import Path

import numpy as np

WORLD = 'bar_world'
CAMERAS = {
    'overhead': '/bartender/overhead_camera',
    'stand': '/bartender/stand_camera',
    'wrist': '/bartender/arm_a/wrist_camera',
}
RGB_TOPIC = {'overhead': 'image_raw', 'stand': 'rgb', 'wrist': 'image_raw'}
MODELS = {'whiskey': 'jack_daniels_bottle', 'cola': 'cola_bottle', 'beer': 'beer_bottle'}
# Stand centres from bar_world.sdf. Bottles only rotate in place: each sits in
# a 26mm-deep stand recess, so a sideways nudge would tip it.
STANDS = {'whiskey': (0.08, -0.30), 'cola': (0.08, -0.15)}
GLASS_HOME = (0.20, -0.55)
# The glass has open counter around it; 5cm keeps it clear of the whiskey
# stand (0.25m away) and inside the overhead camera's view.
GLASS_JITTER = 0.05
DISTRACTOR_P = 0.30
POUR_P = 0.40
# Robotiq 2F-85 knuckle: 0 open, ~0.8 fully shut; on a bottle neck it stops
# around 0.45. Above 0.2 and the bottle lifted = held.
KNUCKLE_CLOSED = 0.2
LIFTED = 0.02
HIDDEN = (5.0, 0.0, 0.2)
DISTRACTOR_SDF = '/tmp/vlm_distractor.sdf'


def held_bottle(knuckle, bottle_z, rest_z):
    """Which bottle the gripper holds: closed fingers and a bottle off its stand."""
    if knuckle < KNUCKLE_CLOSED:
        return None
    lifted = [name for name, z in bottle_z.items() if z - rest_z[name] > LIFTED]
    return lifted[0] if len(lifted) == 1 else None


def sample_scene(rng):
    """Random static scene: which bottles show, their yaw, glass offset, distractor."""
    shown = [b for b in STANDS if rng.random() < 0.7]
    distractor = None
    if rng.random() < DISTRACTOR_P:
        # Half near a target (obstruction positives), half anywhere on arm A's side.
        tx, ty = rng.choice([*[STANDS[b] for b in shown], GLASS_HOME])
        near = rng.random() < 0.5
        distractor = ((tx - rng.uniform(0.06, 0.12), ty + rng.uniform(-0.05, 0.05)) if near
                      else (rng.uniform(-0.25, 0.25), rng.uniform(-0.75, 0.2)))
    return {
        'shown': shown,
        'yaw': {b: rng.uniform(-math.pi, math.pi) for b in shown},
        'glass': (GLASS_HOME[0] + rng.uniform(-GLASS_JITTER, GLASS_JITTER),
                  GLASS_HOME[1] + rng.uniform(-GLASS_JITTER, GLASS_JITTER)),
        'distractor': distractor,
    }


def _ign(service, reqtype, req, reptype='ignition.msgs.Boolean'):
    subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/{service}', '--reqtype', reqtype,
                    '--reptype', reptype, '--timeout', '3000', '--req', req],
                   check=True, capture_output=True)


def set_pose(model, x, y, z, yaw=0.0):
    _ign('set_pose', 'ignition.msgs.Pose',
         f'name: "{model}" position {{x: {x} y: {y} z: {z}}} '
         f'orientation {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}')


def spawn_distractor(x, y):
    Path(DISTRACTOR_SDF).write_text(
        '<sdf version="1.9"><model name="distractor"><link name="link">'
        '<inertial><mass>0.2</mass></inertial>'
        '<collision name="c"><geometry><box><size>0.06 0.06 0.18</size></box></geometry></collision>'
        '<visual name="v"><geometry><box><size>0.06 0.06 0.18</size></box></geometry>'
        '<material><diffuse>0.2 0.4 0.8 1</diffuse></material></visual></link>'
        '<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>20</label></plugin>'
        '</model></sdf>')
    _ign('create', 'ignition.msgs.EntityFactory',
         f'sdf_filename: "{DISTRACTOR_SDF}" pose {{position {{x: {x} y: {y} z: 0.99}}}}')


def remove_distractor():
    try:
        _ign('remove', 'ignition.msgs.Entity', 'name: "distractor" type: MODEL')
    except subprocess.CalledProcessError:
        pass  # nothing spawned in this scene


class Capture:
    def __init__(self):
        import rclpy
        from rclpy.action import ActionClient
        from sensor_msgs.msg import Image, JointState
        from tf2_msgs.msg import TFMessage
        from bartender_pour_interfaces.action import PourDrink

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('capture_vlm_frames')
        self.latest = {}
        self.poses = {}
        self.knuckle = 0.0
        for cam, base in CAMERAS.items():
            for kind, topic in (('rgb', f'{base}/{RGB_TOPIC[cam]}'),
                                ('labels', f'{base}/segmentation/labels_map')):
                self.node.create_subscription(
                    Image, topic, lambda m, k=(cam, kind): self.latest.__setitem__(k, m), 2)
        self.node.create_subscription(
            TFMessage, f'/world/{WORLD}/dynamic_pose/info',
            lambda m: self.poses.update({t.child_frame_id: t.transform.translation for t in m.transforms}), 2)
        self.node.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.pour = ActionClient(self.node, PourDrink, 'pour_drink')
        self.PourDrink = PourDrink

    def _on_joints(self, msg):
        if 'robotiq_85_left_knuckle_joint' in msg.name:
            self.knuckle = msg.position[msg.name.index('robotiq_85_left_knuckle_joint')]

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def frames(self, timeout=10.0):
        """Fresh RGB + label pairs rendered on the same tick, one per camera."""
        self.latest.clear()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            pairs = {c: (self.latest.get((c, 'rgb')), self.latest.get((c, 'labels'))) for c in CAMERAS}
            if all(r and l and r.header.stamp == l.header.stamp for r, l in pairs.values()):
                return pairs
        raise TimeoutError('cameras did not deliver matching frames')

    def bottle_z(self):
        return {b: self.poses[m].z for b, m in MODELS.items() if m in self.poses}


def save(scene_dir, index, pairs, in_gripper):
    from PIL import Image as PILImage
    for cam, (rgb, labels) in pairs.items():
        img = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, -1)
        lab = np.frombuffer(labels.data, np.uint8).reshape(labels.height, labels.width, -1)[:, :, 0]
        PILImage.fromarray(img[:, :, :3]).save(scene_dir / f'{index:04d}_{cam}_rgb.png')
        PILImage.fromarray(lab).save(scene_dir / f'{index:04d}_{cam}_labels.png')
    (scene_dir / f'{index:04d}.json').write_text(json.dumps({'in_gripper': in_gripper}))


def reset(shown=('whiskey', 'cola'), yaw=None, glass=GLASS_HOME):
    for bottle, (x, y) in STANDS.items():
        if bottle in shown:
            set_pose(MODELS[bottle], x, y, 0.9, (yaw or {}).get(bottle, 0.0))
        else:
            set_pose(MODELS[bottle], *HIDDEN)
    set_pose('serving_glass', *glass, 0.9)
    remove_distractor()


def static_scene(cap, scene_dir, rng):
    scene = sample_scene(rng)
    reset(scene['shown'], scene['yaw'], scene['glass'])
    if scene['distractor']:
        spawn_distractor(*scene['distractor'])
    cap.spin(1.5)  # let the physics settle before rendering
    save(scene_dir, 0, cap.frames(), None)
    return [*scene['shown'], 'beer']


def pour_scene(cap, scene_dir, max_frames=120):
    reset()
    cap.spin(1.5)
    rest_z = cap.bottle_z()
    goal = cap.PourDrink.Goal(bottle_id='whiskey', glass_id='serving_glass', pour_amount_ml=40.0)
    cap.pour.wait_for_server(timeout_sec=60)
    accepted = cap.pour.send_goal_async(goal)
    finished = None
    for i in range(max_frames):
        pairs = cap.frames()
        save(scene_dir, i, pairs, held_bottle(cap.knuckle, cap.bottle_z(), rest_z))
        if finished is None and accepted.done():
            finished = accepted.result().get_result_async()
        if finished is not None and finished.done():
            break
        cap.spin(1.0)
    return ['whiskey', 'cola', 'beer']


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('out', type=Path)
    parser.add_argument('--scenes', type=int, default=400)
    parser.add_argument('--seed', type=int, default=0)
    opts = parser.parse_args()
    rng = random.Random(opts.seed)
    cap = Capture()
    cap.spin(3.0)
    for n in range(opts.scenes):
        scene_dir = opts.out / f's{opts.seed:02d}_{n:05d}'
        scene_dir.mkdir(parents=True, exist_ok=True)
        pour = rng.random() < POUR_P
        bottles = pour_scene(cap, scene_dir) if pour else static_scene(cap, scene_dir, rng)
        (scene_dir / 'scene.json').write_text(json.dumps({'bottles': bottles, 'glasses': ['glass']}))
        print(f'{scene_dir.name} {"pour" if pour else "static"} {len(list(scene_dir.glob("*.json"))) - 1} frames',
              flush=True)


if __name__ == '__main__':
    main()
