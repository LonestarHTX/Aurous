# 0003: Oceanic Crust Cooling Law

Status: Accepted
Date: 2026-04-28
Depends on: ADR 0001, ADR 0002, Prototype D Slice 5a

## Context

Prototype D now creates, persists, projects, exports, and stress-tests divergent
ocean crust. Slice 5a proves the current placeholder system can run 60k/40 to
step 1000 without breaking C ownership, C boundaries, D persistence, projection
idempotence, or visual coherence.

The remaining placeholder is physical expression. D ocean crust currently
projects with constant values:

- age from persistent crust state
- thickness `7 km`
- elevation `-1 km`

This is enough to prove identity and persistence, but it cannot express the
ridge-to-abyssal-plain gradient expected from aging ocean crust. Before Slice 6
changes projection behavior, the cooling law needs an architectural contract.

The paper gives three directly relevant constants in Appendix A:

- `zr`: highest oceanic ridge elevation, `-1 km`
- `za`: abyssal plains elevation, `-6 km`
- `o`: oceanic elevation damping, `4e-2 mm / yr`

The damping constant is used directly as a linear first-pass cooling slope:

```text
4e-2 mm / yr = 0.04 km / My
```

This is paper-cited from Appendix A, not an uncited tuning value. A future
square-root or half-space cooling law still requires its own ADR because the
current paper constant table does not provide the coefficients for that model.

The implementation reframes the paper's damping rate as an age-derived
projection function rather than a per-step mutation. For crust born at ridge
crest elevation, subtracting `0.04 km / My` every simulated My and computing
`ridge - 0.04 * AgeMy` during projection are mathematically equivalent, but the
age-derived form keeps projection read-only and avoids storing derived elevation
as sidecar authority.

The paper describes oceanic crust damping due to aging and density increase,
but does not provide an explicit age-dependent oceanic crust thickness constant.
Aurous already uses `7 km` as its oceanic crust-thickness convention in legacy
and sidecar code.

## Decision

Slice 6 replaces constant D ocean-crust elevation with a deterministic
age-derived cooling projection.

For projected D ocean crust:

```text
AgeMy = max(0, CurrentTimeMy - BirthTimeMy)
OceanicElevationDampingKmPerMy = 0.04
ElevationKm = clamp(
  RidgeElevationKm - (OceanicElevationDampingKmPerMy * AgeMy),
  AbyssalPlainElevationKm,
  RidgeElevationKm)
```

With the selected constants:

```text
RidgeElevationKm = -1.0
AbyssalPlainElevationKm = -6.0
OceanicElevationDampingKmPerMy = 0.04
```

The formula reaches abyssal-plain depth at `125 My`:

```text
(-1.0 - -6.0) / 0.04 = 125 My
```

Projected D ocean-crust thickness remains `7 km` in Slice 6.

This is intentional. In this codebase, `Thickness` is crust thickness, not
thermal lithosphere thickness. The paper does not provide a separate
age-dependent crust-thickness growth constant. Slice 6 must not invent one by
silently overloading `Thickness`. If a later prototype needs thermal
lithosphere thickness, it should add a separately named field and ADR.

Cooling is a projection law derived from persistent age. It must not create,
delete, split, merge, reattribute, or consume crust records.

## Preserved Invariants

Slice 6 must preserve all ADR 0001 and ADR 0002 invariants:

- C owner remains nearest currently rotated plate center.
- C boundary remains raw owner adjacency.
- D projection never writes `Sample.PlateId`.
- D projection never writes `Sample.bIsBoundary`.
- D projection never overwrites carried material with
  `ContinentalWeight >= OverlayContinentalWeightThreshold`.
- D projection continues to exclude C material/owner mismatch samples.
- D event detection still consumes owner-edge kinematics, not projection
  classifications.
- `ProjectToPlanet` remains read-only with respect to sidecar authority.
- No V6/V9 remesh, repair, recovery, active-zone, or fallback oceanization path
  is introduced.

Any violation blocks Slice 6.

## Authority Model

Persistent authority remains in sidecar-owned D state:

- crust identity
- birth step/time
- last updated step
- birth edges and ribbon support
- event log
- persistent age source
- area and coalescing state

Cooling output is derived from `AgeMy` and config constants. The derived
projected elevation is not a new independent authority source.

Slice 6 may add config fields:

```text
bEnableDOceanCrustCoolingLaw = true
OceanicRidgeElevationKm = -1.0
OceanicAbyssalPlainElevationKm = -6.0
OceanicElevationDampingKmPerMy = 0.04
OceanicCrustThicknessKm = 7.0
```

If existing tests need to compare placeholder behavior, they may explicitly
disable `bEnableDOceanCrustCoolingLaw`, but the accepted D path after Slice 6
uses cooling by default when D ocean-crust projection is enabled.

## Projection Rule

Cooling is applied only in `Phase3ProjectPersistentOceanCrust`, after C
ownership and carried material projection.

For every projected sample selected by D ocean-crust projection:

- projected age = `max(0, CurrentTimeMy - BirthTimeMy)`
- projected elevation = cooling formula above
- projected thickness = `OceanicCrustThicknessKm`
- projected crust type remains oceanic
- projected ridge/event/debug fields remain diagnostic output

The D projection phase must continue to inspect already projected
`ContinentalWeight` and material/owner mismatch diagnostics before writing D
fields. Cooling must not bypass the existing exclusion checks.

If a projected sample has
`ContinentalWeight >= OverlayContinentalWeightThreshold`, Phase 3 skips all D
ocean-crust writes for that sample. It does not write D elevation and then rely
on a later high-CW override. The already projected carried material remains the
sample's final output.

The same skip rule applies to material/owner mismatch exclusions. D cooling is
allowed to write only to samples that pass the existing D projection mask.

## Exports

Slice 6 keeps all mandatory D exports from ADR 0002:

- `OceanCrustAge.png`
- `OceanCrustThickness.png`
- `OceanCrustId.png`
- `CrustEventOverlay.png`
- `DivergentBoundary.png`

Slice 6 adds one cooling-specific export:

- `OceanCrustElevation.png`

`OceanCrustElevation.png` is a D-only diagnostic view. It renders only projected
D ocean crust, using a stable scale from `-6 km` to `-1 km`, and uses the empty
D-overlay background for samples where Phase 3 skipped or found no D crust.

The elevation diagnostic intentionally keeps the existing scalar-overlay
colormap. That means black can mean either "no projected D crust here" or
"projected D crust at abyssal-plain depth." Reviewers must cross-reference
`OceanCrustElevation.png` with `OceanCrustAge.png` and `OceanCrustId.png` when
judging cooled abyssal crust. This preserves the accepted export scale and
avoids introducing a new visual encoding after Slice 6 validation.

`Elevation.png` and `ElevationEnhanced.png` remain the final composed planet
elevation views. After Slice 6, they should include age-derived D ocean depth
where Phase 3 writes D crust, while protected high-CW and mismatch-excluded
samples continue to show the C material projection.

## Acceptance Gates

Slice 6 must keep these existing gates green:

- `Aurous.TectonicPlanet.SidecarPrototypeC`
- `Aurous.TectonicPlanet.SidecarPrototypeD`
- `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate`
- `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon`

Slice 6 updates D projection assertions:

- projected D age still matches `(CurrentStep - BirthStep) * DeltaTimeMy`
- projected D elevation equals the cooling formula within numeric tolerance
- projected D elevation is always in `[-6.0, -1.0] km`
- projected D thickness remains in `[6.9, 7.1] km`
- samples at age `0 My` project near `-1 km`
- samples at age `125 My` or older project near `-6 km`
- no NaN/Inf appears in persistent records or projected D fields
- D crust overlap with high-CW samples remains `0`
- D crust overlap with material/owner mismatch samples remains `<= 0.10`
- tiny D crust component sample fraction remains `<= 0.10`

Slice 6 adds cooling-gradient gates on the 60k/40 long-horizon path:

- at least two non-empty projected D age bins exist after step 400
- mean elevation for young crust is shallower than mean elevation for old crust
- young crust, defined as `AgeMy <= 25`, has mean elevation above `-2.25 km`
- mature crust, defined as `AgeMy >= 125`, has mean elevation below `-5.75 km`
- per-sample projected depth increases monotonically with age according to the
  linear cooling formula
- the correlation between projected age and projected elevation is negative for
  projected D crust samples at step 400 and later

The implementation must compute expected values from age and config constants,
not from projection diagnostics agreeing with themselves.

## Visual Review

Slice 6 visual review uses the same artifact roots as Slice 5a:

- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_100/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_200/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_400/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_800/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_1000/`

Review questions:

- Are ridge-adjacent/newer crust corridors visibly shallower?
- Do older crust regions trend toward abyssal-plain depth?
- Does depth increase monotonically with age according to the selected linear
  model, without requiring an Earth-like nonlinear profile?
- Does the gradient follow crust age rather than current `PlateId` colors?
- Does cooling avoid high-CW continents and material/owner mismatch zones?
- Does the D crust remain coherent rather than speckled?
- Does `OceanCrustElevation.png` agree with `OceanCrustAge.png` by eye?
- Where `OceanCrustElevation.png` is black, does `OceanCrustAge.png` or
  `OceanCrustId.png` distinguish abyssal D crust from no-crust background?

If metrics pass but the elevation image is incoherent, Slice 6 fails for
investigation rather than tuning.

## Implementation Slice

Slice 6 should be one behavior slice:

1. Add cooling config constants.
2. Add a pure helper, for example `ComputeOceanCrustCoolingElevationKm(AgeMy)`.
3. Apply it only in `Phase3ProjectPersistentOceanCrust`.
4. Add `OceanCrustElevation.png` export support.
5. Update D tests from placeholder elevation bounds to cooling formula checks.
6. Keep thickness at `7 km` and keep tests explicit about that choice.
7. Run C, D, VisualGate, and LongHorizon.

No event detection, coalescing, crust creation, or ownership code should change
in Slice 6 unless a cooling-specific test exposes a direct defect.

## Alternatives Considered

### Keep `-1 km` everywhere

Rejected. It preserves stability but cannot show ocean-floor aging or abyssal
plain development.

### Half-space square-root cooling

Deferred. It is geophysically recognizable, but the local paper constants in
this repository provide a direct linear damping value and no square-root
coefficients. Introducing half-space constants now would create another
uncited tuning surface.

### Age-dependent `Thickness`

Rejected for Slice 6. The existing field represents crust thickness, and the
paper does not provide an age-dependent crust-thickness law. Thermal lithosphere
thickness can be added later as a separate field if needed.

### Mutate persistent crust records with cooled elevation each step

Rejected. Elevation cooling is a deterministic projection from persistent age.
Writing derived elevation back into authority would add mutation without adding
tectonic state.

## Non-Goals

Slice 6 does not implement:

- subduction or crust consumption
- trench depth projection
- sediment accumulation
- erosion
- sea-level or bathymetry balancing
- passive margins
- hot spots
- thermal lithosphere thickness
- balance or steady-state ocean fraction
- high-resolution 5b validation

## Follow-Up Work

After Slice 6 passes:

- decide whether 5b high-resolution long-horizon evidence is needed before E
- draft the next ADR for subduction/consumption as Prototype E
- consider a separate thermal-lithosphere field only if thickness becomes
  load-bearing for subduction or visual amplification
