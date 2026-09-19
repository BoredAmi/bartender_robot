# Control API — proposal

**Status: proposed, not built.** This is the design to argue with before
anyone writes `bartender_api`. Nothing in the repo implements it yet.

A single HTTP/JSON surface so that something which is not a ROS node — a
VLM, a planner, a phone, a test harness — can find out what is on the bar,
ask whether a thing is possible, and make the robot do it.

## Why a separate layer at all

Three reasons, in order of how much they matter:

1. **A VLM cannot speak ROS.** rclpy in the model's process is not an
   option, and a DDS client on the far side of a network is worse.
2. **Callers need to ask before acting.** A planner that discovers
   reachability by issuing a goal and watching it fail is a planner that
   knocks bottles over. The geometry needed to answer "can you?" already
   exists in `layout.py` and is not exposed anywhere.
3. **It pins a contract.** The skills' internals churn; an agent built
   against them should not have to.

## What already exists to build on

This is deliberately thin — most of it is assembly, not invention.

| need | what exists today |
|---|---|
| where things are, who can reach them | `bartender_open/layout.py`: `STATIONS`, `BOTTLE_SLOTS`, `free_slots()`, `servicing_arms()`, `APPROACH_WINDOW` |
| live object poses | `/world/bar_world/dynamic_pose/info` (bridged, 60 Hz, every non-static model) |
| composite skills | `PourDrink`, `OpenBottle` actions, with feedback and measured results |
| primitive motion, with bounds and refusals | `bartender_teach.Pendant.dispatch()` — jog limits, branch wrapping, two-arm frame safety, one-command-at-a-time locking |
| named poses and sequences | `bartender_teach` points and pipelines (`config/taught_points.yaml`) |
| an HTTP server pattern with no dependencies | `bartender_teach/teach_gui.py` — stdlib `http.server`, localhost by default |

**The movement API should wrap `Pendant.dispatch()`, not reimplement it.**
The pendant already encodes every bound and refusal that stops a typo from
becoming a collision, and the browser GUI proves the pattern: it owns no
robot logic, so a fix reaches every front end at once. A second motion
implementation would be a second set of bugs.

## Four surfaces

### 1. World — "what is there, and who can reach it"

```
GET /world
```

```json
{
  "frame": "world",
  "counter": { "centre": [0.08, 0.0], "size": [1.76, 1.60], "top_z": 0.9 },
  "arms": [
    { "id": "a", "origin": [-0.45, -0.40, 0.9], "yaw": 0.0,
      "serves_world_y": [-0.30, 0.10], "skills": ["pour", "hold"] },
    { "id": "b", "origin": [0.61, 0.40, 0.9], "yaw": 3.14159,
      "serves_world_y": [-0.10, 0.30], "skills": ["open", "hold"] }
  ],
  "stations": [
    { "id": "whiskey", "kind": "bottle", "xy": [0.08, -0.30], "slot": -2,
      "reachable_by": ["a"], "occupied": true,
      "pose": { "xyz": [0.08, -0.30, 0.900], "tilt_deg": 0.2,
                "source": "sim_ground_truth", "confidence": 1.0,
                "stamp": "2026-09-19T21:02:11Z" } },
    { "id": "glass", "kind": "vessel", "xy": [0.20, -0.55],
      "reachable_by": ["a"], "occupied": true, "pose": { "...": null } },
    { "id": "slot_+1", "kind": "empty_slot", "xy": [0.08, 0.15],
      "reachable_by": ["b"], "occupied": false, "pose": null }
  ]
}
```

Three things this must get right:

- **`reachable_by` is computed, never written down.** It comes from
  `layout.servicing_arms()`, so it cannot disagree with the robot.
- **Empty slots are stations too.** "Where could I put this down" is a
  question a planner will ask, and the free slots are surveyed positions,
  not leftover tabletop.
- **Every pose carries `source` and `confidence`.** Today the source is
  simulator ground truth and confidence is 1.0. When perception replaces it
  the field already exists, and callers written against it do not change.

### 2. Feasibility — "could you?"

```
POST /can      { "verb": "pour", "args": { "bottle": "gin", "glass": "glass" } }
```

```json
{
  "ok": false,
  "reasons": [
    { "code": "OUT_OF_APPROACH_WINDOW",
      "detail": "gin is at world y=+0.15; arm A serves [-0.30,+0.10]",
      "measurements": { "station_y": 0.15, "window": [-0.30, 0.10] } },
    { "code": "NO_ARM_CAN_DO_BOTH",
      "detail": "arm B reaches gin but the glass is 1.035m from its base (limit 0.85)",
      "measurements": { "glass_range_from_b": 1.035, "reach_limit": 0.85 } }
  ],
  "alternatives": [
    { "verb": "pour", "args": { "bottle": "whiskey", "glass": "glass" } }
  ]
}
```

No motion, no planning-time cost beyond geometry. This is the endpoint a
VLM should call on every candidate action before committing to a plan, and
the one that makes the difference between an agent that reasons about the
bar and an agent that flails at it.

`alternatives` is optional and cheap here (same-kind stations that *are*
feasible). It is worth including: it turns a refusal into a next step.

### 3. Movement — primitives, for when no skill fits

```
GET  /state                     joints, tool pose, gripper, per arm
POST /move/point   { "arm":"a", "point":"whiskey_approach" }
POST /move/joints  { "arm":"a", "joints":[...], "speed":0.3 }
POST /move/tool    { "arm":"a", "xyz":[0.53,0.10,0.12], "quat":[...],
                     "frame":"base_link", "collision_checked":true }
POST /move/jog     { "arm":"a", "axis":"tz", "mm":-40 }
POST /gripper      { "arm":"b", "position":0.25 }
```

Rules, inherited from the pendant and non-negotiable for an autonomous
caller:

- **Frames are explicit.** `frame` is required on `/move/tool`; there is no
  default. Arm B's axes point the other way and a silent default is the bug
  this project keeps finding.
- **Bounds are refused, never clamped.** `jog z 500` when you meant `50` is
  a typo, and clamping turns it into a move that quietly does something
  else.
- **One motion at a time, refused not queued.** Two motion goals interleaved
  on one arm is the failure the pendant's lock exists to prevent.

### 4. Skills and pipelines — the useful verbs

```
GET  /skills                     what exists, with argument schemas
POST /do    { "verb":"open", "args":{ "bottle":"beer", "stow_after":true } }
GET  /job/{id}
POST /job/{id}/cancel

GET  /pipelines                  taught sequences, from taught_points.yaml
POST /pipelines/{name}/run       { "dry": false }
```

`/do` is async and returns a job immediately, because a pour takes minutes:

```json
{ "job": "j-17", "verb": "open", "state": "running", "progress": 0.12,
  "stage": "gripping the opener" }
```

`GET /skills` returning argument schemas is what lets a VLM use the robot
without being retrained when a skill is added. Generate it from the
`.action` files rather than writing it twice.

**Pipelines are the cheap extensibility.** A human teaches a sequence on the
pendant, and it becomes a callable verb with no new code. See
`ros2_ws/src/bartender_teach/README.md`.

## The error taxonomy — the part that matters most

This is the highest-value piece of the whole proposal and it is mostly
mechanical to build, because the measurements already exist.

Today a failure comes back as prose:

> `the beer moved 6.0mm while being pushed on`
> `arm B could not pick up the opener`
> `the opener is not seated on the cap: 280.7mm down (needs 14) and 344.9mm off centre`

Excellent for a human reading a log. Unusable for a planner, which cannot
tell "try again" from "never going to work" from "something is broken".
Every failure should carry a code, the stage it happened in, the numbers,
and whether retrying is sensible:

```json
{
  "state": "failed",
  "error": {
    "code": "OBJECT_NOT_SEATED",
    "stage": "pressing the opener onto the cap",
    "detail": "280.7mm down (needs 14) and 344.9mm off centre (allows 6.5)",
    "measurements": { "depth_mm": 280.7, "depth_required_mm": 14,
                      "offset_mm": 344.9, "offset_allowed_mm": 6.5 },
    "retryable": true,
    "suggest": "re-home both arms and retry; three consecutive failures mean restart the stack"
  }
}
```

A starting set, drawn from failures this project has actually produced:

| code | means | retryable |
|---|---|---|
| `UNKNOWN_STATION` | no such bottle/glass/slot | no |
| `OUT_OF_APPROACH_WINDOW` | geometry says no arm can side-grasp it | no |
| `OUT_OF_REACH` | past the arm's radius | no |
| `NO_ARM_CAN_DO_BOTH` | one arm reaches A, another reaches B, neither both | no |
| `ALREADY_OPEN` | premise wrong; the cap is off and can't re-attach in sim | no |
| `PLAN_FAILED` | MoveIt found nothing | yes |
| `MOVE_STOPPED_SHORT` | controller said success, flange is elsewhere | yes |
| `GRIPPER_NOT_FOLLOWING` | goal accepted, joint never moved | yes, then restart |
| `GRASP_LOST` | had it, dropped it mid-sequence | yes |
| `OBJECT_NOT_SEATED` | got there, geometry check failed | yes |
| `OBJECT_DISTURBED` | the workpiece moved more than allowed | yes |
| `SCENE_STALE` | no model poses; the bridge is down | no |

`MOVE_STOPPED_SHORT` and `GRIPPER_NOT_FOLLOWING` are not hypothetical —
see `docs/ROADMAP.md`. An agent that cannot distinguish them from "the plan
was bad" will retry forever.

## Safety

The page moves a robot arm. `teach_gui` already takes the right line and
the API should copy it exactly:

- **Bind `127.0.0.1` by default.** `--host 0.0.0.0` is reasonable on an
  isolated robot LAN and a bad idea anywhere else; print a warning.
- **No authentication is not a plan.** Before this is exposed to anything
  off-host, a shared token in a header is the minimum.
- **Every job cancellable**, and a global `POST /stop`.
- **`dry` on everything that moves**, so a planner can rehearse.
- **Rate-limited and serialised.** Refuse a second motion, do not queue it.

An autonomous caller should also be given a budget — maximum actions per
minute, maximum consecutive failures before it must stop and ask — enforced
server-side. A VLM in a retry loop against a robot is the failure mode worth
designing against from the start.

## Transport: HTTP now, MCP later

HTTP/JSON first, because `teach_gui` shows it costs nothing (stdlib only,
no CDN, works with the network down) and because every client can speak it.

An **MCP server** is the natural second front end once the verb set settles:
the tool-per-verb shape maps onto `/skills` almost exactly, and it would let
a model call the robot without bespoke glue. Do not build it first — the
verb set and the error taxonomy are what need to be right, and those are
easier to iterate over plain HTTP.

## Build order

1. `GET /world` and `GET /state` — read-only, no risk, immediately useful
   for prompting a VLM.
2. `POST /can` — pure geometry over `layout.py`.
3. The error taxonomy, applied to the two existing skills. Do this *before*
   `/do`, so `/do` is born with it.
4. `POST /do` + jobs over the existing actions.
5. Movement primitives over `Pendant.dispatch()`.
6. Pipelines.
7. MCP front end.

Steps 1–3 are worth doing on their own even if nothing autonomous ever
arrives: `/can` and a real error taxonomy would have made several of this
project's debugging sessions much shorter.
