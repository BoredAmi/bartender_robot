# Drink menu lens (Spectacles)

Three floating buttons -- Beer, Cyder, Jagermeister. Point at one (hand ray hover arms it), hold a
thumbs-up for 0.4 s to order. Pinch does not order. The order currently only logs `ORDER <drink>`;
`DrinkMenu.order()` is where the robot call goes.

- `Assets/Scripts/thumbsUp.ts` -- pure thumbs-up classifier + hold timer (no Lens Studio imports)
- `Assets/Scripts/DrinkMenu.ts` -- the component: buttons, sticky arming, per-frame gesture check
- `tests/thumbsUp.test.ts` -- logic check

Needs SpectaclesInteractionKit installed and the SIK prefab in the scene.

## Testing

1. **Logic** (no Lens Studio): `node tests/thumbsUp.test.ts` (Node >= 23).
2. **Lens Studio preview**: tick `debugConfirmOnPinch` on DrinkMenu. Mouse over a button -> it grows
   and the status says "Beer -- thumbs up to order"; click -> log `ORDER beer`. Untick before shipping.
3. **On device** (Preview on Spectacles, watch Logger):
   - each drink: hover, then thumbs up -> one `ORDER <drink>`; hold longer -> still one
   - pinch on a button -> no order
   - fist, open hand, pointing, thumbs down -> no order
   - thumbs up before hovering anything -> no order
   - works with left and right hand
   - false triggers / misses -> tune `curlRatio`, `thumbUpMin`, `holdSeconds` on the component

## Robot link (not built)

- Robot: add `POST /order {"drink": ...}` to `ros2_ws/src/bartender_api` next to `/move/*`, same
  409-when-busy convention, plus a shared token before binding beyond localhost.
- Real setup is one UR5, not the sim's two arms: the beer-opening skill (`OpenBottle`) is two-armed and
  cannot run as is. The sim also only has beer/whiskey/cola stations -- Cyder and Jagermeister need stations.
- Lens: `order()` -> `InternetModule.fetch` POST. Spectacles needs HTTPS, so expose :8090 through a
  tunnel/Tailscale rather than plain LAN HTTP.
