// No Lens Studio imports on purpose: tests/thumbsUp.test.ts runs this file under plain Node.

export type Point = { x: number; y: number; z: number };

export type HandJoints = {
  wrist: Point;
  thumbKnuckle: Point;
  thumbTip: Point;
  indexKnuckle: Point;
  indexTip: Point;
  middleKnuckle: Point;
  middleTip: Point;
  ringKnuckle: Point;
  ringTip: Point;
  pinkyKnuckle: Point;
  pinkyTip: Point;
};

// Ratios of hand size (wrist→middleKnuckle), so they hold for any hand and any distance.
// Real hands vary: tune these on device, they are exposed as @inputs on DrinkMenu.
export type ThumbsUpParams = {
  curlRatio: number; // curled finger: tip→wrist < curlRatio × knuckle→wrist (open hand ≈ 1.9, fist ≈ 1.1)
  thumbUpMin: number; // thumbTip must be this far above thumbKnuckle, world +Y
};

export const DEFAULT_PARAMS: ThumbsUpParams = { curlRatio: 1.4, thumbUpMin: 0.3 };

const dist = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);

export function isThumbsUp(j: HandJoints, p: ThumbsUpParams = DEFAULT_PARAMS): boolean {
  const scale = dist(j.middleKnuckle, j.wrist);
  if (scale <= 0) return false;
  const fingers: [Point, Point][] = [
    [j.indexKnuckle, j.indexTip],
    [j.middleKnuckle, j.middleTip],
    [j.ringKnuckle, j.ringTip],
    [j.pinkyKnuckle, j.pinkyTip],
  ];
  const curled = fingers.every(([knuckle, tip]) => dist(tip, j.wrist) < p.curlRatio * dist(knuckle, j.wrist));
  const thumbUp = j.thumbTip.y - j.thumbKnuckle.y > p.thumbUpMin * scale;
  const thumbHighest = fingers.every(([knuckle, tip]) => j.thumbTip.y > Math.max(knuckle.y, tip.y));
  return curled && thumbUp && thumbHighest;
}

// Fires once per continuous hold of `holdSeconds`; the gesture must drop before it can fire again,
// so one long thumbs-up is one order.
export class HoldDetector {
  private since: number | null = null;
  private fired = false;
  holdSeconds: number;

  constructor(holdSeconds: number) {
    this.holdSeconds = holdSeconds;
  }

  update(active: boolean, now: number): boolean {
    if (!active) {
      this.since = null;
      this.fired = false;
      return false;
    }
    if (this.since === null) this.since = now;
    if (this.fired || now - this.since < this.holdSeconds) return false;
    this.fired = true;
    return true;
  }
}
