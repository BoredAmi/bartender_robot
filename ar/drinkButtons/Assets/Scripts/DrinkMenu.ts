import { Interactable } from "SpectaclesInteractionKit.lspkg/Components/Interaction/Interactable/Interactable";
import { HandInputData } from "SpectaclesInteractionKit.lspkg/Providers/HandInputData/HandInputData";
import TrackedHand from "SpectaclesInteractionKit.lspkg/Providers/HandInputData/TrackedHand";
import WorldCameraFinderProvider from "SpectaclesInteractionKit.lspkg/Providers/CameraProvider/WorldCameraFinderProvider";
import { HoldDetector, isThumbsUp } from "./thumbsUp";

const DRINKS = ["beer", "cyder", "jagermeister", "whiskey", "cola"] as const;
type Drink = (typeof DRINKS)[number];

@component
export class DrinkMenu extends BaseScriptComponent {
  @input
  @hint("Beer, Cyder, Jagermeister, Whiskey, Cola -- in that order; each needs a collider")
  buttons!: SceneObject[];

  @input
  status!: Text;

  @input
  @hint("Finger counts as curled when tip-to-wrist < this x knuckle-to-wrist")
  curlRatio: number = 1.4;

  @input
  @hint("Thumb tip must rise this fraction of hand size above the thumb knuckle")
  thumbUpMin: number = 0.3;

  @input
  holdSeconds: number = 0.4;

  @input
  @hint("How far in front of the wearer the menu appears, cm")
  spawnDistance: number = 80;

  @input
  @hint("Seconds the order confirmation stays up before the menu hides")
  closeDelay: number = 1.5;

  @input
  @hint("Editor preview only: thumbs-up cannot be simulated, so a click confirms")
  debugConfirmOnPinch: boolean = false;

  private armed: Drink | null = null;
  private hands: TrackedHand[] = [];
  private hold!: HoldDetector;
  private isOpen = false;
  private hideAfterOrder!: DelayedCallbackEvent;
  private gestures: GestureModule = require("LensStudio:GestureModule");

  onAwake() {
    this.hold = new HoldDetector(this.holdSeconds);
    // SIK singletons and Interactables are only ready after every onAwake has run.
    this.createEvent("OnStartEvent").bind(() => this.start());
    this.createEvent("UpdateEvent").bind(() => this.checkThumbsUp());
    this.hideAfterOrder = this.createEvent("DelayedCallbackEvent");
    this.hideAfterOrder.bind(() => {
      if (!this.isOpen) this.setVisible(false);
    });
    // Palm taps come from hand tracking only, so the preview opens the menu with a click.
    if (global.deviceInfoSystem.isEditor()) this.createEvent("TapEvent").bind(() => this.open());
  }

  private start() {
    const handData = HandInputData.getInstance();
    this.hands = [handData.getHand("left"), handData.getHand("right")];
    this.buttons.forEach((b, i) => {
      const interactable = (b.getComponent(Interactable.getTypeName()) ??
        b.createComponent(Interactable.getTypeName())) as Interactable;
      // Sticky on purpose: curling the hand into a thumbs-up collapses the pointing ray,
      // so the drink must stay armed after the ray leaves the button.
      interactable.onHoverEnter.add(() => this.arm(DRINKS[i]));
      interactable.onTriggerStart.add(() => {
        if (!this.debugConfirmOnPinch) return;
        this.arm(DRINKS[i]);
        this.order(DRINKS[i]);
      });
    });
    this.setVisible(false);
    // HandType is undefined outside Spectacles (e.g. a phone-mode preview); the editor click still opens the menu.
    if (!GestureModule.HandType) return;
    for (const hand of [GestureModule.HandType.Left, GestureModule.HandType.Right]) {
      this.gestures.getPalmTapDownEvent(hand).add(() => this.open());
    }
  }

  private open() {
    if (this.isOpen) return;
    this.isOpen = true;
    this.armed = null;
    this.buttons.forEach((b) => b.getTransform().setLocalScale(vec3.one()));
    this.status.text = "Point at a drink, then thumbs up";
    this.placeInFrontOfWearer();
    this.setVisible(true);
  }

  private setVisible(visible: boolean) {
    // Children, not this object: disabling it would also stop this script's own events.
    this.sceneObject.children.forEach((c) => (c.enabled = visible));
  }

  private placeInFrontOfWearer() {
    const head = WorldCameraFinderProvider.getInstance();
    const t = this.getTransform();
    t.setWorldPosition(head.getForwardPositionParallelToGround(this.spawnDistance));
    const toHead = head.getWorldPosition().sub(t.getWorldPosition());
    t.setWorldRotation(quat.lookAt(new vec3(toHead.x, 0, toHead.z).normalize(), vec3.up()));
  }

  private arm(drink: Drink) {
    if (!this.isOpen) return;
    this.armed = drink;
    this.buttons.forEach((b, i) => {
      const s = DRINKS[i] === drink ? 1.15 : 1;
      b.getTransform().setLocalScale(new vec3(s, s, s));
    });
    this.status.text = `${label(drink)} -- thumbs up to order`;
  }

  private checkThumbsUp() {
    const params = { curlRatio: this.curlRatio, thumbUpMin: this.thumbUpMin };
    const up = this.hands.some((h) => h.isTracked() && isThumbsUp(joints(h), params));
    if (this.hold.update(up, getTime()) && this.isOpen && this.armed) this.order(this.armed);
  }

  // The one seam for the robot link: replace the body with the HTTP call to bartender_api.
  private order(drink: Drink) {
    print(`ORDER ${drink}`);
    this.status.text = `Ordered: ${label(drink)} ✓`;
    // Closed at once so a still-held thumbs-up or a stray hover cannot place a second order.
    this.isOpen = false;
    this.armed = null;
    this.hideAfterOrder.reset(this.closeDelay);
  }
}

const label = (d: Drink) => ({ beer: "Beer", cyder: "Cyder", jagermeister: "Jagermeister", whiskey: "Whiskey", cola: "Cola" })[d];

const joints = (h: TrackedHand) => ({
  wrist: h.wrist.position,
  thumbKnuckle: h.thumbKnuckle.position,
  thumbTip: h.thumbTip.position,
  indexKnuckle: h.indexKnuckle.position,
  indexTip: h.indexTip.position,
  middleKnuckle: h.middleKnuckle.position,
  middleTip: h.middleTip.position,
  ringKnuckle: h.ringKnuckle.position,
  ringTip: h.ringTip.position,
  pinkyKnuckle: h.pinkyKnuckle.position,
  pinkyTip: h.pinkyTip.position,
});
