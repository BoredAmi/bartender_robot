"""GET /world: what is there, and who can reach it.

Static geometry from bartender_open.layout; live occupancy and pose from
whatever is actually subscribed to /world/bar_world/dynamic_pose/info (see
server.py) -- the same ground-truth topic open_action_server reads. See
docs/CONTROL_API.md, "1. World".
"""
from bartender_open import layout as L

from .reach import arms_within_reach

# A station's world (x, y) is decided once, in layout.py; its Gazebo MODEL
# NAME is decided separately, by whatever generated it (make_beer_and_opener,
# make_bottle_stands, bar_world.sdf's own <include> blocks), and the two
# vocabularies do not match -- 'whiskey' the station is 'jack_daniels_bottle'
# the model. This is the one place that translation has to happen for
# /world to answer "is something there" from live poses.
STATION_MODEL = {
    'whiskey': 'jack_daniels_bottle',
    'cola': 'cola_bottle',
    'beer': 'beer_bottle',
    'glass': 'serving_glass',
    'opener': 'bottle_opener',
}

ARMS = (
    {'id': 'a', 'origin': list(L.ARM_A_ORIGIN), 'yaw': L.ARM_A_YAW,
     'serves_world_y': list(L.APPROACH_WINDOW), 'skills': ['pour', 'hold']},
    {'id': 'b', 'origin': list(L.ARM_B_ORIGIN), 'yaw': L.ARM_B_YAW,
     'serves_world_y': list(L.APPROACH_WINDOW), 'skills': ['open', 'hold']},
)


def _kind_of(name):
    if name == 'glass':
        return 'vessel'
    if name == 'opener':
        return 'tool'
    return 'bottle'


def _reachable_by(kind, xy):
    """Arms that can reach a station, by the check its kind actually needs.

    A 'bottle' station is picked up by a fixed side-grasp off the line, so
    layout.servicing_arms (APPROACH_WINDOW) is the right question. Nothing
    grips the glass (a bottle is tilted over it) or descends onto the
    opener the same way (straight down, not a side approach), so those use
    plain radial reach instead -- see reach.py for why this split exists
    and what happens if the two are conflated (reachable_by came back []
    for both, which a real bottle successfully poured into every day).
    """
    if kind == 'bottle':
        return L.servicing_arms(xy)
    return arms_within_reach(xy)


def _slot_id(xy):
    """Name an empty line slot the way layout.slot_y's own index would.

    slot_y(index) = index * SLOT_PITCH, so this is just that inverted --
    kept in step with BOTTLE_SLOTS rather than re-deriving a naming scheme,
    so a re-pitched line (Phase D option 3) renames itself for free.
    """
    index = round(xy[1] / L.SLOT_PITCH)
    return f'slot_{index:+d}'


def build(pose_lookup):
    """Assemble the /world document.

    `pose_lookup(model_name) -> (x, y, z) or None` is the only live input,
    so this function itself needs no ROS to test -- see test_world.py,
    which drives it with a plain dict.
    """
    stations = []
    for name in sorted(L.STATIONS):
        xy = L.STATIONS[name]
        kind = _kind_of(name)
        live = pose_lookup(STATION_MODEL.get(name, name))
        stations.append({
            'id': name,
            'kind': kind,
            'xy': list(xy),
            'reachable_by': _reachable_by(kind, xy),
            'occupied': live is not None,
            'pose': None if live is None else {
                'xyz': list(live),
                'source': 'sim_ground_truth',
                'confidence': 1.0,
            },
        })
    for xy in L.free_slots():
        stations.append({
            'id': _slot_id(xy),
            'kind': 'empty_slot',
            'xy': list(xy),
            'reachable_by': L.servicing_arms(xy),
            'occupied': False,
            'pose': None,
        })
    return {
        'frame': 'world',
        'counter': {
            'centre': list(L.COUNTER_CENTRE),
            'size': list(L.COUNTER_SIZE),
            'top_z': L.COUNTER_Z,
        },
        'arms': [dict(a) for a in ARMS],
        'stations': stations,
    }
