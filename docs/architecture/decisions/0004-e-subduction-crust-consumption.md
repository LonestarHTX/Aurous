# 0004: Subduction Crust Consumption

Status: Proposed
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

No Prototype E code should land before this ADR is accepted.

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

Minimum new event type:

- `ETectonicSidecarCrustEventType::SubductionConsumption`

Minimum new event input/record shape:

- `SubductingCrustId`
- sorted plate pair
- overriding plate id
- subducting plate id, when known
- source owner-edge ids
- consumed area
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

E does not physically delete crust records in the first implementation. A
fully consumed record remains in the store with:

- `ActiveAreaKm2 == 0`
- `ConsumedFraction == 1`
- `bIsFullyConsumed == true`
- `FullyConsumedStep` set

Projection must not render fully consumed crust. Partially consumed crust may
project only its active support. The exact spatial clipping representation is an
open question before E projection lands.

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

Trench elevation and overriding-plate uplift are deferred. E consumption may
produce diagnostic trench candidates, but it must not introduce mountain
building or trench bathymetry without a later ADR.

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
- expected projection exclusion for fully consumed crust

No E test may pass only because E diagnostics agree with themselves.

## Acceptance Gates

E must keep these existing gates green:

- `Aurous.TectonicPlanet.SidecarPrototypeC`
- exact core D filter `^Aurous.TectonicPlanet.SidecarPrototypeD$`
- `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate`
- `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon`

New E gates, once slices land:

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

## Implementation Slices

### Slice 1: State Scaffold

Add state, hashing, and test-only seeding only.

Allowed:

- event enum value
- consumption fields on crust records
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

### Slice 2: Convergent Detection And Consumption

Add opt-in runtime detection.

Allowed:

- `bEnableSubductionConsumptionEvents = false`
- `SubductionConvergenceMinKmPerMy = 10.0`
- one guarded call site after plate transforms advance
- deterministic component grouping over sorted owner edges
- production `ApplySubductionConsumptionEvent`
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
- slab pull or plate force feedback
- erosion
- sediment accretion
- sea-level or global water balance
- deletion/pruning of historical event records
- V6/V9 remesh exclusion logic

## Open Questions Before Accepted

1. Exact spatial clipping representation for partially consumed crust:
   per-edge active intervals, per-crust consumed support fragments, or a
   separate consumed-edge list.
2. Whether E Slice 2 should consume by edge-local area only or by connected
   convergent component area.
3. Whether the first projection slice should show trench candidates only, or
   defer all trench visuals until an uplift/trench ADR.
4. Whether slab pull gets its own Prototype F ADR or a later E slice after
   consumption is proven.
5. Whether high-resolution 5b D evidence is required before accepting E.

## Follow-Up Work

Before marking this ADR Accepted:

- answer the open questions above
- define Slice 1 field names against `TectonicSidecarCrust.h`
- write a source-hygiene rule for E mutation APIs
- define the exact test filter name, likely
  `Aurous.TectonicPlanet.SidecarPrototypeE`
- decide whether the `FTectonicSidecarConfig` E defaults should live next to D
  config or in a nested E config struct
