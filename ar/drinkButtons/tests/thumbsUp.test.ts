// Run: node tests/thumbsUp.test.ts   (Node >= 23 strips the types natively)
// Lives outside Assets/ so Lens Studio never tries to compile node:assert.
import assert from "node:assert/strict";
import { type HandJoints, HoldDetector, isThumbsUp } from "../Assets/Scripts/thumbsUp.ts";

// Poses in cm, wrist at origin, knuckles pointing -Z (fist held sideways in front of the user).
const thumbsUp: HandJoints = {
  wrist: { x: 0, y: 0, z: 0 },
  thumbKnuckle: { x: 0, y: 3, z: -4 },
  thumbTip: { x: 0, y: 8, z: -4.5 },
  indexKnuckle: { x: 0, y: 1.5, z: -9 },
  indexTip: { x: 2, y: 1.5, z: -6 },
  middleKnuckle: { x: 0, y: 0, z: -9 },
  middleTip: { x: 2, y: 0, z: -6 },
  ringKnuckle: { x: 0, y: -1.5, z: -8.7 },
  ringTip: { x: 2, y: -1.5, z: -6 },
  pinkyKnuckle: { x: 0, y: -3, z: -8 },
  pinkyTip: { x: 2, y: -3, z: -5.5 },
};
const fist = { ...thumbsUp, thumbTip: { x: 1.5, y: 1, z: -6.5 } };
const thumbsDown = { ...thumbsUp, thumbKnuckle: { x: 0, y: -3, z: -4 }, thumbTip: { x: 0, y: -8, z: -4.5 } };
const pointing = { ...fist, indexTip: { x: 0, y: 1.5, z: -16 } };
const openHand: HandJoints = {
  wrist: { x: 0, y: 0, z: 0 },
  thumbKnuckle: { x: -4, y: 4, z: 0 },
  thumbTip: { x: -7, y: 8, z: 0 },
  indexKnuckle: { x: -2, y: 9, z: 0 },
  indexTip: { x: -2.5, y: 17, z: 0 },
  middleKnuckle: { x: 0, y: 9.5, z: 0 },
  middleTip: { x: 0, y: 18, z: 0 },
  ringKnuckle: { x: 2, y: 9, z: 0 },
  ringTip: { x: 2.5, y: 16.5, z: 0 },
  pinkyKnuckle: { x: 3.5, y: 8, z: 0 },
  pinkyTip: { x: 4.5, y: 14, z: 0 },
};
const scaled = (h: HandJoints, k: number) =>
  Object.fromEntries(Object.entries(h).map(([n, p]) => [n, { x: p.x * k, y: p.y * k, z: p.z * k }])) as HandJoints;
const shifted = (h: HandJoints, dx: number, dy: number, dz: number) =>
  Object.fromEntries(Object.entries(h).map(([n, p]) => [n, { x: p.x + dx, y: p.y + dy, z: p.z + dz }])) as HandJoints;

assert.equal(isThumbsUp(thumbsUp), true, "thumbs up");
assert.equal(isThumbsUp(scaled(thumbsUp, 1.4)), true, "big hand");
assert.equal(isThumbsUp(shifted(thumbsUp, 30, 140, -50)), true, "anywhere in the world");
assert.equal(isThumbsUp(fist), false, "fist");
assert.equal(isThumbsUp(thumbsDown), false, "thumbs down");
assert.equal(isThumbsUp(pointing), false, "pointing");
assert.equal(isThumbsUp(openHand), false, "open hand");

const hold = new HoldDetector(0.4);
assert.equal(hold.update(true, 0), false, "not yet");
assert.equal(hold.update(true, 0.3), false, "still not");
assert.equal(hold.update(true, 0.4), true, "fires at holdSeconds");
assert.equal(hold.update(true, 2), false, "fires once per hold");
assert.equal(hold.update(false, 2.1), false, "released");
assert.equal(hold.update(true, 2.2), false, "restarts timer");
assert.equal(hold.update(true, 2.7), true, "fires again after release");

console.log("thumbsUp: all checks passed");
