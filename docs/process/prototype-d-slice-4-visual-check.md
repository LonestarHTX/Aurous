# Prototype D Slice 4 Visual Checklist

Status: required review evidence before planning Slice 5.

Scope: `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate`, 60k samples, 40 plates,
checkpoints 0/100/200/400, placeholder crust physics.

## Checklist

- Persistent crust aligns with current or historical divergent owner edges.
- Crust corridors are spatially coherent, not speckled.
- No obvious crust appears on globally convergent interfaces.
- Crust ids persist across checkpoints.
- Event growth is bounded and does not show component or record explosion.
- `CrustEventOverlay.png` activity geographically matches `DivergentBoundary.png`.
- D crust avoids high material-owner-mismatch zones except minor contact ambiguity.

## Numeric Proxies

- Tiny D crust component sample fraction is at most `0.10` at steps 100, 200,
  and 400. A tiny component is `<= 10` projected D crust samples. This catches
  true salt-and-pepper projection speckle without rejecting the valid early
  morphology where multiple independent ridge corridors have not yet merged.
- Largest connected component fraction, top-three component fraction, component
  count, and components per 1000 crust samples are logged as morphology
  evidence, not pass/fail gates. Early many-corridor morphology and late merged
  network morphology are both valid if the tiny-component fraction stays low.
- Every `CrustId` present at checkpoint N is still present at checkpoint N+1.
- `CrustEventLog.Num() <= CurrentStep * 4 * PlateCount`.
- D crust overlap with high carried continental material is `0`.
- D crust overlap with `MaterialOwnerMismatch` samples is logged and capped at
  `<= 10%`.
  Current projection treats mismatch samples as an exclusion zone, so the
  expected value is `0`; any nonzero value is investigation evidence even if it
  remains below the cap.

## MaterialOwnerMismatch Rationale

The `10%` cap is a hedge for third-plate intrusion, narrow contact ambiguity, and
projection-grid edge effects. A small overlap can happen at complex triple
junctions without implying that D has turned material mismatch into spreading
authority. Broad overlap would mean persistent crust is being placed in the same
zones where C says carried material and current ownership disagree, which is the
V9 failure mode reappearing under a new name.

## Expected Artifact Roots

- `Saved/MapExports/SidecarPrototypeD/visual_60k40/step_000/`
- `Saved/MapExports/SidecarPrototypeD/visual_60k40/step_100/`
- `Saved/MapExports/SidecarPrototypeD/visual_60k40/step_200/`
- `Saved/MapExports/SidecarPrototypeD/visual_60k40/step_400/`
- `Saved/MapExports/SidecarPrototypeD/visual_60k40_below_threshold/`
