# Prototype D Slice 6 Cooling-Law Plan

Status: closed implementation plan for ADR 0003.

Scope: add age-derived D ocean-crust elevation cooling. Do not change event
detection, coalescing, crust creation, ownership, boundaries, subduction,
collision, rifting, uplift, erosion, or balance.

## Summary

Slice 6 replaces placeholder D ocean-crust elevation with the ADR 0003 cooling
law:

```text
AgeMy = max(0, CurrentStep - BirthStep) * DeltaTimeMy
ElevationKm = clamp(-1.0 - 0.04 * AgeMy, -6.0, -1.0)
```

The slope is the paper Appendix A oceanic elevation damping constant converted
from `4e-2 mm / yr` to `0.04 km / My`. Thickness remains the existing `7 km`
crust-thickness convention.

Slice 6 implements damping as an age-derived projection function, not as
per-step elevation mutation. For crust born at ridge crest elevation, the result
is equivalent, and projection stays read-only with respect to D authority.

## Key Changes

- Add cooling config defaults:
  - `bEnableDOceanCrustCoolingLaw = true`
  - `OceanicRidgeElevationKm = -1.0`
  - `OceanicAbyssalPlainElevationKm = -6.0`
  - `OceanicElevationDampingKmPerMy = 0.04`
  - `OceanicCrustThicknessKm = 7.0`
- Add a pure helper, for example:
  - `ComputeOceanCrustCoolingElevationKm(AgeMy)`
- Apply cooling only in `Phase3ProjectPersistentOceanCrust`.
- Keep Phase 3 as a post-material projection phase.
- On high-CW or material/owner mismatch samples, skip all D writes for that
  sample. Do not write D elevation and then override it later.
- Add `OceanCrustElevation.png` as a D-only diagnostic overlay with a stable
  `[-6 km, -1 km]` scale.
- Keep the existing scalar overlay colormap for `OceanCrustElevation.png`.
  Black can mean either no projected D crust or abyssal D crust at `-6 km`; the
  visual checklist must cross-reference `OceanCrustAge.png` and
  `OceanCrustId.png`.
- Keep `Elevation.png` and `ElevationEnhanced.png` as final composed planet
  elevation views. They now include cooled D elevation only where Phase 3 writes
  D crust.

## Analytic Test Design

Every Slice 6 elevation assertion must compute expected values independently
from persistent crust age and config constants:

```text
ExpectedAgeMy =
  max(0, CurrentStep - BirthStep) * DeltaTimeMy

ExpectedElevationKm =
  clamp(
    OceanicRidgeElevationKm -
      OceanicElevationDampingKmPerMy * ExpectedAgeMy,
    OceanicAbyssalPlainElevationKm,
    OceanicRidgeElevationKm)
```

Tests must not use projected elevation or event-reported diagnostics as the
source of expected elevation.

## Gates

- Existing filters remain green:
  - `Aurous.TectonicPlanet.SidecarPrototypeC`
  - `Aurous.TectonicPlanet.SidecarPrototypeD`
  - `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate`
  - `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon`
- C ownership invariants remain exact.
- `ProjectToPlanet` does not mutate sidecar authority.
- D projection never writes `PlateId` or `bIsBoundary`.
- D projection skips high-CW and material/owner mismatch samples.
- D crust overlap with high-CW samples remains `0`.
- D crust overlap with material/owner mismatch remains `<= 0.10`.
- Projected age equals independent step math.
- Projected elevation equals the clamp formula within tolerance.
- Projected elevation is always within `[-6.0, -1.0] km`.
- Projected thickness remains within `[6.9, 7.1] km`.
- Age `0 My` samples project near `-1 km`.
- Age `>= 125 My` samples project near `-6 km`.
- Depth increases monotonically with age under the linear model. Do not require
  an Earth-like nonlinear curve.
- At step 400 and later, projected D crust has a negative age/elevation
  correlation.
- No NaN/Inf appears in persistent records or projected D fields.
- Tiny D crust component sample fraction remains `<= 0.10`.

## Exports

At every D visual and long-horizon checkpoint, export the existing D review set:

- `OceanCrustId.png`
- `OceanCrustAge.png`
- `OceanCrustThickness.png`
- `CrustEventOverlay.png`
- `DivergentBoundary.png`

Add:

- `OceanCrustElevation.png`

Context overlays remain useful:

- `Elevation.png`
- `ElevationEnhanced.png`
- `MaterialOwnerMismatch.png`
- `ContinentalWeight.png`
- `BoundaryMask.png`
- `PlateId.png`

## Visual Review

Use the 60k/40 long-horizon checkpoints from Slice 5a:

- `step_100`
- `step_200`
- `step_400`
- `step_800`
- `step_1000`

Answer yes/no:

- newer crust near active ridges is shallower
- older crust trends toward abyssal-plain depth
- `OceanCrustElevation.png` agrees with `OceanCrustAge.png`
- black regions in `OceanCrustElevation.png` are interpreted with
  `OceanCrustAge.png` / `OceanCrustId.png`, not by elevation alone
- depth increases monotonically with age under the linear model
- the gradient follows crust age, not current `PlateId`
- D elevation avoids high-CW continents and mismatch-excluded zones
- D crust remains coherent rather than speckled

If metrics pass but the overlay is visually incoherent, stop and investigate.
Do not tune the formula or mask only to satisfy the image.

## Validation Commands

Build `AurousEditor`, then run:

- `Aurous.TectonicPlanet.SidecarPrototypeC`
- `Aurous.TectonicPlanet.SidecarPrototypeD`
- `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate`
- `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon`

Use the existing Aurous tooling:

- `$aurous-test-metrics` for runtime and numeric summaries
- `$aurous-overlay-contactsheet` for cooling overlays
- `$aurous-diagnostic-diff` for checkpoint-to-checkpoint export changes
- `$git-push-verify` after any push

## Assumptions

- Slice 6 is the final D physics-expression slice before planning Prototype E.
- The linear damping law is intentionally simple and paper-cited.
- Balance is still not tested; no subduction or crust consumption exists yet.
- High-resolution 5b evidence is useful but not required to implement Slice 6.
