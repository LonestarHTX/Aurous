# 0004: Subduction Crust Consumption

Status: Accepted
Date: 2026-04-28
Depends on: ADR 0001, ADR 0002, ADR 0003, Prototype D Slice 6

## Context

Prototype C freezes the ownership/projection foundation. Prototype D adds
persistent divergent ocean crust, projects it, stress-tests it, and gives it a
paper-cited age-derived cooling law.

D deliberately does not solve balance. It creates oceanic crust at divergent
boundaries and never destroys it. Long-horizon runs can remain coherent, but
without a consumption process the persistent ocean-crust store will grow without
bound and the planet cannot approach a tectonic lifecycle.

Prototype E introduces the first explicit destruction path: subduction consumes
persistent D ocean crust at convergent boundaries.

This ADR is intentionally scoped to crust consumption first. It does not try to
solve full mountain building, slab pull, collision, terrane transfer, erosion,
or final land/ocean balance in one step. The first question is narrower:

```text
Can Aurous consume persistent D ocean crust through named convergent events
without breaking C ownership, C boundaries, D crust identity, or projection
idempotence?
```

No Prototype E code should land before this ADR is accepted and the Prototype E
adversarial ADR review plus 30-day pre-mortem have been completed.

## Decision

Add sidecar-owned subduction consumption state and a named
`SubductionConsumptionEvent`.

E reads C's recomputed owner-edge geometry and D's persistent ocean-crust
records, identifies convergent interfaces that intersect active D ocean crust,
and records explicit consumption of that crust. Consumption reduces active
projectable ocean-crust area. It does not delete history by default.

Subduction is an event mutation, not a projection repair. It may mutate only
sidecar-owned E/D crust-consumption state and append event-log entries.

E must not mutate:

- C `Sample.PlateId`
- C `Sample.bIsBoundary`
- C material/owner mismatch diagnostics
- C material support or ownership
- D divergent birth events or original crust identity

The default runtime path remains off until E tests opt in.

## Preserved Invariants

E must preserve every invariant from ADR 0001, ADR 0002, and ADR 0003:

- C owner remains nearest currently rotated plate center.
- C boundary remains raw owner adjacency.
- `FTectonicPlanet.Samples` remains output/cache, not tectonic authority.
- D crust identity remains sidecar-owned.
- Projection is read-only with respect to sidecar authority.
- D projection never writes `Sample.PlateId` or `Sample.bIsBoundary`.
- D cooling remains a projection function derived from persistent age.
- No V6/V9 remesh, repair, recovery, active-zone, or fallback oceanization path
  is introduced.

Any violation blocks E.

## New Persistent State

E may extend the existing D crust module rather than creating a parallel
authority system.

Minimum new fields on persistent ocean-crust records:

- `ConsumedAreaKm2`
- `ActiveAreaKm2`
- `ConsumedFraction`
- `LastConsumedStep`
- `FullyConsumedStep`
- `bIsFullyConsumed`
- `ActiveIntervalsBySourceEdge`

`ActiveIntervalsBySourceEdge` is the accepted first spatial-clipping
representation. It is keyed by canonical `FSidecarBoundaryEdgeKey`, matching
D's edge-keyed birth/support model. Values are canonical edge-local active
intervals in normalized edge parameter space, `[0, 1]`. Consumed intervals are
removed or shortened deterministically by subduction events. This lets E
preserve partially consumed crust without introducing a second ownership system
or mutating C sample ownership.

Slice 1 interval contract:

- Interval endpoint convention is closed numeric support `[StartT, EndT]` with
  `0 <= StartT < EndT <= 1`.
- `ActiveIntervalToleranceT = 1.0e-9` for endpoint comparison, empty-interval
  removal, and hash normalization.
- An interval with `EndT - StartT <= ActiveIntervalToleranceT` is empty and
  must not be stored.
- Intervals for one edge are sorted by `StartT`, then `EndT`.
- Overlapping intervals, or intervals separated by
  `<= ActiveIntervalToleranceT`, are merged during canonicalization.
- `ActiveIntervalsBySourceEdge` is canonicalized by sorting edge keys
  ascending, then canonicalizing each edge's interval array.
- Hashing must use the canonical sorted edge order and canonical sorted interval
  order. Endpoint values are clamped to `[0, 1]` and normalized to
  `ActiveIntervalToleranceT` before hashing.
- NaN/Inf endpoints are invalid and must fail tests.

Minimum new event type:

- `ETectonicSidecarCrustEventType::SubductionConsumption`

Minimum new event input/record shape:

- `SubductingCrustId`
- sorted plate pair
- overriding plate id
- subducting plate id, when known
- source owner-edge ids
- consumed area
- pre-consumption active area
- post-consumption active area
- pre-consumption consumed area
- post-consumption consumed area
- consumed interval deltas by source edge
- post-consumption active intervals for every touched source edge
- convergence speed
- edge length
- event step/time
- crust age at consumption
- projected elevation at consumption
- optional debug sample ids

All new persistent state is sidecar authority and must be included in
`ComputeSidecarAuthorityHash`.

The event log remains append-order-sensitive. The crust store remains canonical
state hashable independent of container order by sorting on `CrustId`.

The event log is replay-sufficient for E consumption. Given the D-created crust
store state before the first E event, replaying E `SubductionConsumption` records
in append order must reproduce the same `ActiveIntervalsBySourceEdge`,
`ActiveAreaKm2`, `ConsumedAreaKm2`, `ConsumedFraction`, `LastConsumedStep`,
`FullyConsumedStep`, and `bIsFullyConsumed` values as the canonical crust store.
Each E apply path must mutate the crust store and append the corresponding event
record through the same event-shaped function. The test-only
`ApplySubductionConsumptionEventForTest` path must call the same mutation helper
as runtime E, not a separate store rewrite path.

## Consumption Rule

`SubductionConsumptionEvent` is the only E mutation path.

Inputs:

- sorted owner edges from C
- normal separation speed from C/sidecar kinematics
- D ocean-crust records and their active support
- projected carried continental weight only as an eligibility/exclusion signal
- D crust age/elevation from persistent age plus ADR 0003 cooling formula

Baseline convergence threshold:

```text
NormalSeparationKmPerMy <= -10.0
```

The sign convention follows ADR 0002's boundary velocity helper: positive is
divergence, negative is convergence.

Like D's dual-divergence rule, E uses a dual-convergence rule:

- the boundary edge must be locally convergent along the canonical edge normal
- the plate pair's rotated centers must be converging above the configured
  threshold
- both tests are required before consuming crust

This prevents locally convergent edge arcs on an otherwise separating interface
from destroying ocean crust.

Created divergent crust may be consumed only if it is active and intersects the
accepted convergent edge/support region. Projection-only ocean classifications,
generic fallback ocean, and C material/owner mismatch samples are not
consumable crust.

## Subduction Hierarchy

Initial E hierarchy:

- oceanic-oceanic convergence: older active D ocean crust subducts beneath
  younger active D ocean crust
- oceanic-continental convergence: active D ocean crust subducts beneath
  meaningful carried continental material
- continental-continental convergence: no E consumption event; collision is
  deferred to a later ADR
- oceanic-transform or below-threshold convergence: no E consumption event

Meaningful continental material uses the same default threshold as D projection:

```text
ContinentalWeight >= OverlayContinentalWeightThreshold
```

If two candidate D crust records have equal age within tolerance, the
tie-breaker is deterministic and conservative:

1. older `BirthStep` subducts first
2. lower `CrustId` subducts if birth step also ties

This rule exists only to keep tests deterministic; it should be rare in normal
motion.

## Area Accounting

Consumption area is computed independently from event diagnostics:

```text
ConsumedAreaKm2 = EdgeLengthKm * ConvergenceSpeedKmPerMy * DeltaTimeMy
```

`ConvergenceSpeedKmPerMy` is the positive magnitude of accepted negative normal
separation speed.

The consumed area is capped by the active area remaining on the target crust
record.

Named field invariants:

- `ActiveAreaKm2 + ConsumedAreaKm2 == CreatedAreaKm2`, within area tolerance.
- `ConsumedFraction == ConsumedAreaKm2 / CreatedAreaKm2` when
  `CreatedAreaKm2 > 0`.
- `ConsumedFraction == 0` when `CreatedAreaKm2 <= 0`.
- `ActiveAreaKm2 >= 0` and `ConsumedAreaKm2 >= 0`.
- `ConsumedAreaKm2` is monotonic nondecreasing for a crust record.
- `ActiveAreaKm2` is monotonic nonincreasing for a crust record after E begins
  consuming it.
- `bIsFullyConsumed == true` iff `ActiveAreaKm2 <= AreaToleranceKm2`.
- `FullyConsumedStep` is set exactly once, at the first step where the record
  becomes fully consumed.

`AreaToleranceKm2` defaults to `1.0e-6` for Slice 1 and may be revisited only
with an ADR/test-plan note if numerical scale makes that inappropriate.

E does not physically delete crust records in the first implementation. A
fully consumed record remains in the store with:

- `ActiveAreaKm2 == 0`
- `ConsumedFraction == 1`
- `bIsFullyConsumed == true`
- `FullyConsumedStep` set

Projection must not render fully consumed crust. Partially consumed crust may
project only its active support from `ActiveIntervalsBySourceEdge`.

Slice 1 and Slice 2 use edge-local area accounting only. A subduction event
consumes area against the target crust record through the accepted owner edge
and its active interval support. Component-aware area aggregation is a later
optimization and must not be introduced until the edge-local model is proven by
tests.

This tombstone-first rule preserves auditability and avoids changing crust ids
or event history while E is being validated.

## Projection Rule

E projection is not part of Slice 1.

When projection eventually lands:

- consumed crust must not project as active ocean crust
- `Sample.PlateId` must remain C-owned
- `Sample.bIsBoundary` must remain C-owned
- C material/owner mismatch remains diagnostic/exclusion evidence
- D cooling still computes from persistent age for active crust only
- E may add diagnostic output fields, but not new ownership authority

Potential E projection outputs:

- active ocean-crust area mask
- consumed crust mask
- subduction-front mask
- trench candidate mask
- consumed crust id/debug id

Trench location diagnostics are in scope for E projection. `SubductionFront.png`
and `TrenchCandidate.png` show where accepted consumption interfaces are, so the
reviewer can see whether E events align with convergent boundaries.

Trench bathymetry, trench elevation changes, and overriding-plate uplift are
deferred. E consumption may produce diagnostic trench locations, but it must not
introduce mountain building or bathymetry features without a later ADR.

## Mandatory E Exports

Once E projection/exports land, acceptance exports must include:

- `SubductionFront.png`
- `CrustConsumption.png`
- `ActiveOceanCrustArea.png`
- `ConsumedCrustId.png`
- `TrenchCandidate.png`
- existing D overlays:
  - `OceanCrustAge.png`
  - `OceanCrustElevation.png`
  - `OceanCrustThickness.png`
  - `OceanCrustId.png`
  - `CrustEventOverlay.png`
- existing C context overlays:
  - `BoundaryMask.png`
  - `PlateId.png`
  - `ContinentalWeight.png`
  - `MaterialOwnerMismatch.png`

PNG exports remain secondary evidence. Raw sidecar arrays and independent tests
are acceptance authority.

## Independent Test Discipline

E tests must compute expected results from C/D authority, not E diagnostics.

Required independent computations:

- owner edges from C owner adjacency
- convergence speed from sidecar plate kinematics
- edge length from current world-space great-circle distance
- expected consumed area from edge length, convergence speed, and elapsed time
- expected target crust by age/continental hierarchy
- expected active area after consumption from prior active area minus expected
  consumed area
- expected interval clipping by subtracting consumed interval deltas from the
  prior canonical active intervals
- expected event-log replay state from append-order replay, compared with the
  canonical crust store
- expected projection exclusion for fully consumed crust

No E test may pass only because E diagnostics agree with themselves.

## Acceptance Gates

E must keep these existing gates green:

- `Aurous.TectonicPlanet.SidecarPrototypeC`
- exact core D filter `^Aurous.TectonicPlanet.SidecarPrototypeD$`
- `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate`
- `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon`

New E gates, once slices land:

- exact filter `^Aurous.TectonicPlanet.SidecarPrototypeE$`
- default config creates zero subduction events
- below-threshold convergence creates zero consumption
- divergent motion creates zero consumption
- transform/perpendicular motion creates zero consumption
- controlled oceanic-oceanic convergence consumes older active D crust first
- controlled oceanic-continental convergence consumes D ocean crust, not
  continental carried material
- continental-continental convergence creates no E consumption event
- consumed area matches independent analytic area within tolerance
- active area is monotonic nonincreasing for consumed crust
- total created area remains >= total consumed area
- fully consumed crust stops projecting
- event log hash changes when append order changes
- crust store hash is stable under record reorder by `CrustId`
- repeated `ProjectToPlanet` does not mutate consumption state
- no NaN/Inf in persistent E fields or projected E diagnostics
- no V6/V9 repair/recovery/ocean-fill authority tokens in E code
- active intervals canonicalize to identical hashes after edge/interval reorder
- event-log replay reconstructs store consumption state after seeded events
- test-only seeding and runtime seeding use the same event-shaped apply helper

## Implementation Slices

### Slice 1: State Scaffold

Add state, hashing, and test-only seeding only.

Allowed:

- event enum value
- consumption fields on crust records
- per-edge active interval state
- `ApplySubductionConsumptionEventForTest`
- authority hash coverage
- idempotence tests
- source hygiene tests

Forbidden:

- runtime detection
- projection changes
- exports
- terrain/elevation mutation
- slab pull
- collision

Slice 1 names and hygiene:

- Test filter: `^Aurous.TectonicPlanet.SidecarPrototypeE$`.
- Config defaults live on `FTectonicSidecarConfig` next to D defaults unless
  Slice 1 proves the E config surface is already large enough to justify a
  nested struct. Defaults remain off.
- E mutation APIs must match `^Apply[A-Z][A-Za-z]*Event(ForTest)?$`.
- Repair-shaped mutation names are review blockers in E files:
  `Recover`, `Repair`, `Heal`, `Backfill`, `Resync`, `Promote`,
  `Reclassify`.
- E mutation/detection files must not reference projection authority tags:
  `OceanFallback`, `DivergentOceanFill`, `MaterialOwnerMismatch` as a
  consumable source, `QueryOwnership`, `ActiveZone`, `QuietInterior`, `V6`,
  or `V9`.

### Slice 2: Convergent Detection And Consumption

Add opt-in runtime detection.

Allowed:

- `bEnableSubductionConsumptionEvents = false`
- `SubductionConvergenceMinKmPerMy = 10.0`
- one guarded call site after plate transforms advance
- deterministic component grouping over sorted owner edges
- production `ApplySubductionConsumptionEvent`
- edge-local area accounting through per-edge active intervals
- analytic area tests

Forbidden:

- D projection changes
- trench rendering
- uplift
- slab pull
- continental collision

### Slice 3: Projection And Exports

Project consumed/active E state as diagnostics.

Allowed:

- active/consumed masks
- subduction-front exports
- trench-location candidate exports
- consumed crust debug ids
- exclusion of fully consumed crust from D ocean projection

Forbidden:

- writing `PlateId`
- writing `bIsBoundary`
- overriding high-CW carried material
- adding terrain uplift/trenches as accepted behavior

### Slice 4: Long-Horizon Creation/Consumption Stress

Run D creation plus E consumption together.

Goal:

- prove event counts, active area, consumed area, and hashes remain bounded and
  deterministic at 60k/40
- show that consumption slows or balances unbounded D area growth without
  destabilizing C/D invariants

This is still not final planetary balance.

## Non-Goals

Prototype E does not implement:

- continental collision
- terrane transfer
- sutures
- uplift/orogeny
- trench bathymetry as final terrain
- slab pull or plate force feedback; slab pull belongs in a separate Prototype F
  ADR, not a later E slice
- erosion
- sediment accretion
- sea-level or global water balance
- deletion/pruning of historical event records
- V6/V9 remesh exclusion logic

## Accepted Answers From Proposed Review

1. Partially consumed crust is represented by per-edge active intervals keyed by
   canonical owner edge. This parallels D's edge-keyed support model.
2. Slice 1 and Slice 2 use edge-local area accounting. Component-aware area is a
   later optimization, not part of the first E proof.
3. E projection should show trench/front locations as diagnostics. Bathymetry
   features and uplift are deferred to a later ADR.
4. Slab pull is Prototype F. It must not be folded into E.
5. D high-resolution 5b evidence is not a prerequisite for E. It can run in
   parallel as confidence evidence.

## Accepted Answers From Pre-Code Review

1. Active intervals must have a canonical endpoint, sort, merge, empty-removal,
   tolerance, and hash contract before Slice 1 code lands.
2. The E event log is replay-sufficient for consumption state, not merely audit
   narration.
3. `ApplySubductionConsumptionEventForTest` must use the same event-shaped apply
   helper as runtime E.
4. `ConsumedFraction` is defined from `ConsumedAreaKm2 / CreatedAreaKm2`, and
   `ActiveAreaKm2 + ConsumedAreaKm2 == CreatedAreaKm2` is a named invariant.
5. The Slice 1 test filter, config placement, and E source-hygiene naming rules
   are part of the acceptance contract.

## Follow-Up Work

Before any Prototype E code lands:

- confirm the updated pre-code review findings above are represented in the
  Slice 1 implementation plan
- define the exact C++ field names against `TectonicSidecarCrust.h` using the
  contracts in this ADR
- keep D high-resolution 5b running in parallel as confidence evidence, not as
  an E Slice 1 blocker
