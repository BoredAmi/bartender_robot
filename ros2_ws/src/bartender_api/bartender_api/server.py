"""HTTP entry point for bartender_api.

    ros2 run bartender_api server

    GET  /world      what is on the bar, and who can reach it
    GET  /state      joints, tool pose, gripper, per arm
    POST /can        {"verb": "pour"|"open", "args": {...}}  -- "could you?"
    POST /move/point {"arm": "a", "point": "whiskey_approach"}
    POST /move/jog   {"arm": "a", "axis": "z", "amount": 20}
    POST /gripper    {"arm": "b", "position": 0.25}

Phase B (world/state/can) is read-only, no motion, no new risk. The three
movement routes are Phase C, scoped down to "simple movement" -- see
movement.py's own docstring for exactly what that does and does not cover
(no arbitrary joint or Cartesian targets; goto a taught point, jog
relative to where the arm is, or move the gripper -- the three things
Pendant already does with every bound this project has found a reason
for). `/do`, jobs, pipelines and a global stop are still not built.

Same pattern as bartender_teach/teach_gui.py, and for the same reasons: no
dependencies (http.server is stdlib), works from a laptop on the robot's
network, and degrades gracefully if anything about it misbehaves. The
movement routes reuse teach_gui's own locking pattern (one command at a
time, refused not queued) via movement.MovementBridge.

Binding
-------
127.0.0.1 by default. This process now moves the robot -- binding wide
open is a bad idea outside an isolated robot LAN. --host 0.0.0.0 prints a
warning every time.
"""
import argparse
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from bartender_teach.point_store import (
    PointStore, PointStoreError, default_points_path,
)
from bartender_teach.teach_points import TeachNode

from . import feasibility, movement, state_view, world

# Same topic, same QoS, same reasoning as open_action_server's _on_poses:
# depth 1 and best-effort, because a queued backlog is a lie about where
# something is, not a queue worth keeping.
POSE_TOPIC = '/world/bar_world/dynamic_pose/info'
FRESH = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class PoseCache(Node):
    """Caches the latest pose of every model on POSE_TOPIC, ground truth."""

    def __init__(self):
        super().__init__('bartender_api_poses')
        self._poses = {}
        self._lock = threading.Lock()
        self.create_subscription(TFMessage, POSE_TOPIC, self._on_poses, FRESH)

    def _on_poses(self, msg):
        with self._lock:
            for tf in msg.transforms:
                t = tf.transform.translation
                self._poses[tf.child_frame_id] = (t.x, t.y, t.z)

    def get(self, name):
        with self._lock:
            return self._poses.get(name)


def make_handler(pose_cache, teach_node, move_bridge):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def _send(self, code, payload):
            raw = json.dumps(payload).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            # Every response here reflects live state; a cached copy would
            # be worse than no answer at all.
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(raw)

        def _read_body(self):
            """Return (body, None) or (None, error-payload) -- send nothing."""
            try:
                n = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(n) or b'{}')
            except (ValueError, json.JSONDecodeError):
                return None, {'error': 'bad request body'}
            # Valid JSON is not necessarily an object -- see teach_gui.py's
            # identical check and the AttributeError it replaced.
            if not isinstance(body, dict):
                return None, {'error': 'body must be a JSON object'}
            return body, None

        def do_GET(self):
            path = self.path.split('?')[0].rstrip('/') or '/'
            if path == '/world':
                self._send(200, world.build(pose_cache.get))
            elif path == '/state':
                self._send(200, state_view.build(teach_node))
            else:
                self._send(404, {'error': 'not found'})

        def do_POST(self):
            path = self.path.split('?')[0].rstrip('/')
            body, error = self._read_body()
            if error is not None:
                self._send(400, error)
                return
            if path == '/can':
                self._do_can(body)
            elif path == '/move/point':
                self._do_move('goto', body, ('point',))
            elif path == '/move/jog':
                self._do_move('jog', body, ('axis', 'amount'))
            elif path == '/gripper':
                self._do_move('gripper', body, ('position',))
            else:
                self._send(404, {'error': 'not found'})

        def _do_can(self, body):
            verb = body.get('verb', '')
            if not isinstance(verb, str) or not verb:
                self._send(400, {'error': 'no verb'})
                return
            args = body.get('args', {})
            if not isinstance(args, dict):
                self._send(400, {'error': 'args must be a JSON object'})
                return
            self._send(200, feasibility.can(verb, args))

        def _do_move(self, kind, body, field_names):
            arm = body.get('arm', '')
            if not isinstance(arm, str) or not arm:
                self._send(400, {'error': 'no arm'})
                return
            fields = [body.get(name) for name in field_names]
            if any(f is None for f in fields):
                self._send(400, {'error': f'need {", ".join(field_names)}'})
                return
            result = getattr(move_bridge, kind)(arm, *fields)
            self._send(200 if result['ok'] else 409, result)

        def log_message(self, fmt, *args):
            """Silence per-request logging; a poller would bury real output."""

    return Handler


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='bartender_api', description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='127.0.0.1',
                        help='interface to bind (default 127.0.0.1; '
                             '0.0.0.0 exposes this to the network)')
    parser.add_argument('--port', type=int, default=8090)
    # ros2 run passes --ros-args through; argparse must not choke on it.
    opts, _ = parser.parse_known_args(sys.argv[1:] if args is None else args)

    rclpy.init(args=None)
    pose_cache = PoseCache()
    teach_node = TeachNode(cache_pose=True)
    executor = MultiThreadedExecutor()
    executor.add_node(pose_cache)
    executor.add_node(teach_node)
    threading.Thread(target=executor.spin, daemon=True).start()

    if not teach_node.wait_for_state(timeout=10.0):
        print('warning: no /joint_states after 10s -- serving anyway, '
              '/state will show both arms disconnected', file=sys.stderr)

    # Same fallback as pour_action_server.py: a point file that is missing
    # or unreadable must not stop the server from starting -- goto just has
    # no points to offer until it is fixed, which /move/point's own refusal
    # already reports per request.
    try:
        store = PointStore.load(default_points_path())
    except PointStoreError as exc:
        print(f'warning: {exc} -- starting with no taught points',
              file=sys.stderr)
        store = PointStore(default_points_path())
    move_bridge = movement.MovementBridge(teach_node, store)

    try:
        server = ThreadingHTTPServer(
            (opts.host, opts.port),
            make_handler(pose_cache, teach_node, move_bridge))
    except OSError as exc:
        print(f'cannot bind {opts.host}:{opts.port}: {exc}', file=sys.stderr)
        executor.shutdown()
        pose_cache.destroy_node()
        teach_node.destroy_node()
        rclpy.try_shutdown()
        return 1

    shown = '127.0.0.1' if opts.host in ('0.0.0.0', '') else opts.host
    print(f'\n  bartender_api:  http://{shown}:{opts.port}')
    print('    GET  /world')
    print('    GET  /state')
    print('    POST /can        {"verb": "pour"|"open", "args": {...}}')
    print('    POST /move/point {"arm": "a", "point": "whiskey_approach"}')
    print('    POST /move/jog   {"arm": "a", "axis": "z", "amount": 20}')
    print('    POST /gripper    {"arm": "b", "position": 0.25}')
    if opts.host not in ('127.0.0.1', 'localhost'):
        print(f'\n  WARNING: bound to {opts.host}. This process moves the '
              f'robot arms (/move/*, /gripper).\n           Only do this on '
              f'a trusted, isolated robot LAN.')
    print('\n  Ctrl-C to stop.\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nstopping')
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown()
        pose_cache.destroy_node()
        teach_node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
