# 0002: Persistent Divergent Ocean Crust

Status: Proposed
Date: 2026-04-27
Depends on: ADR 0001, Prototype C freeze

## Context

Prototype C proves clean ownership, clean boundaries, and decoupled carried
material. It does not prove basin opening as persistent crust. Prototype B could
show a spatial ocean corridor, but that corridor was projected fresh from
current geometry and had no identity, age, or thickness history.

Prototype D starts by adding one persistent tectonic process: divergent ocean
crust creation.

No Prototype D code lands before this ADR is accepted.

## Decision

Add sidecar-owned persistent ocean crust state created only by a named
`DivergentSpreadingEvent`.

D reads C's recomputed boundary signals, identifies divergent plate-pair
interfaces, and creates or extends persistent ocean crust records. Projection
may render that crust into `FTectonicPlanet`, but projected samples remain
cache/output only.

D must not mutate C ownership, C boundaries, or C material/owner mismatch
behavior.

## C Invariants Preserved

D must preserve all ADR 0001 invariants:

- owner = nearest currently rotated plate center
- boundary = raw owner adjacency
- ownership gaps and overlaps remain zero
- material does not write ownership
- ownership does not write material
- projection is read-only with respect to sidecar truth
- `FTectonicPlanet.Samples` is not tectonic authority
- no V6/V9 repair, active-zone recovery, or generic oceanization path

Any violation blocks D.

## New Persistent State

Add:

- `FSidecarOceanCrustStore`
- `FSidecarCrustEventLog`

Both are owned by `FTectonicPlanetSidecar`.

Persistent ocean crust must not live as authority in `FTectonicPlanet.Samples`.

Minimum persistent record:

- `CrustId`
- `EventType = DivergentSpreading`
- `PlateA`, `PlateB`
- `BirthStep`, `BirthTimeMy`
- `LastUpdatedStep`
- source boundary/interface sample ids or segment ids
- ridge direction / spreading direction
- support geometry
- created area
- oceanic age
- oceanic thickness
- projected elevation seed
- parent ridge/event id

All persistent D state must be included in sidecar authority hashing.

## Representation Choice

D1 uses per-event strip/support records.

Rejected for D1:

- per-sample annotation as authority, because it risks recreating global sample authority
- per-plate fragment ownership, because it prematurely answers accretion/subduction ownership questions
- new strip plates, because plate-count growth is a later architectural decision

Per-event strip/support keeps ocean crust persistent without changing C
ownership.

The exact strip/support representation is still open before this ADR can move
to Accepted.

## Event Rule

`DivergentSpreadingEvent` is the only mutation path in this slice.

Inputs from C:

- owner adjacency
- boundary plate pair ids
- relative plate velocity
- divergent boundary flags/scores
- interface samples or edges

Baseline event threshold:

- normal separation speed >= `10 mm/yr`
- measured along the local boundary normal or pair separation direction
- evaluated per boundary edge/sample and aggregated by plate pair

The event-isolation tests must prove no event fires below threshold.

Event writes only to persistent D crust/event state.

Event must not:

- change `Sample.PlateId`
- change `Sample.bIsBoundary`
- repair material/owner mismatch
- create crust from projection misses
- create crust from generic ocean fallback
- call V6/V9 remesh or recovery paths

## Crust Identity Rule

New divergent ocean crust belongs to the divergent crust event / plate pair, not
to either C owner plate.

Projection may show the crust in whichever Voronoi owner currently covers that
sample, but persistent crust identity remains event-owned until a later named
event consumes or transfers it.

No silent reattribution.

## Cadence

D1 evaluates divergent events every sidecar step.

Events are coalesced by plate pair / ridge id so a spreading ridge extends an
existing persistent crust record rather than creating unrelated fragments every
step.

A later ADR may introduce adaptive or periodic event cadence.

## Third-Plate Intrusion

Known unresolved D1 edge case:

If a third plate's Voronoi cell later covers persistent ocean crust created
between plates A and B, the crust remains event-owned. It is not orphaned,
reattributed, consumed, or transferred unless a later named event handles that
case.

This is explicitly deferred to subduction/collision/accretion work.

## Projection Rule

Projection combines:

- C ownership
- C carried material projection
- D persistent ocean crust projection

If D ocean crust projects onto a sample, visible projected fields may include:

- oceanic crust type
- age
- thickness
- elevation
- ridge direction
- source event id/debug classification

These are projected outputs only. They must not become source of truth.

## Mandatory D Exports

D acceptance exports must include:

- `OceanCrustAge.png`
- `OceanCrustThickness.png`
- `OceanCrustId.png`
- `CrustEventOverlay.png`
- `DivergentBoundary.png`
- all mandatory C overlays

## Independent Test Discipline

D tests must compute expected results independently from the implementation's
diagnostics.

Examples:

- expected crust age = elapsed My since `BirthTimeMy`
- expected creation only when normal separation speed exceeds threshold
- expected crust area from boundary length, spreading rate, and elapsed time within tolerance
- expected zero events below threshold
- expected `PlateId` and `bIsBoundary` from C formulas, not D diagnostics

No D test may pass only because D diagnostics agree with themselves.

## Acceptance Gates

D must pass all C freeze tests unchanged.

New D gates:

- divergent boundary creates nonzero persistent ocean crust
- crust identity persists at `R+1`, `R+10`, and `R+25`
- crust age increases with elapsed time
- crust thickness/elevation remain finite and bounded
- no ocean crust is created when divergence is disabled
- no ocean crust is created below threshold
- no ocean crust is created from projection misses or fallback
- basin corridor is spatially coherent, not speckled
- all D mutations appear in an event log
- `ProjectToPlanet` remains idempotent with respect to sidecar truth

## Implementation Slices

D should land in slices:

1. persistent state model, event log, authority hash, no event firing
2. divergent event detection and creation tests
3. projection and mandatory exports
4. long-horizon basin-opening gate

Each slice must keep C invariants green.

Slice 1 is the only D work allowed after this ADR becomes Accepted: persistent
state model, event log, authority hash, and tests proving the new state is
sidecar-authoritative and idempotent under projection. No event detection,
projection, or exports in slice 1.

## Non-Goals

D1 does not implement:

- subduction consumption
- continental collision
- terrane accretion
- rifting/splitting plates
- uplift tuning
- erosion/balance
- full oceanic crust lifecycle
- third-plate crust consumption or reattribution

## Consequences

Good:

- basin opening becomes persistent, not just a projection artifact
- ocean age/thickness can be tested directly
- D begins adding tectonic process state without weakening C
- crust identity is event-owned, so C ownership stays clean

Risks:

- D may accidentally become a second ownership system
- projection output may tempt future code to read `FTectonicPlanet.Samples` as truth
- per-event support geometry may need refinement after D1

Mitigation:

Every mutation must be event-named, sidecar-owned, logged, independently tested,
and covered by idempotence/hash tests.

## Open Questions Before Accepted

These must be resolved before this ADR moves from Proposed to Accepted:

- Exact support geometry for per-event strips. Candidate representations must be
  enumerated and compared before choosing one: spherical polygon, persistent
  sample id set, parametric ribbon, boundary edge list, or another explicitly
  justified representation.
- Exact area tolerance for independent creation-area tests.
- Exact coalescing key for ridge/event identity.
- Event-log hash policy tied to the coalescing key: order-sensitive vs
  sorted-by-id, per-event hash vs aggregate store hash, and which fields define
  deterministic equality.
- Whether D1 uses boundary samples, boundary edges, or both.
- Whether initial D projection supports only oceanic fallback regions or can
  overlay carried material where divergent crust exists.
- Numerical baseline for `projected elevation seed`, or an explicit paper
  citation and deferral if the value is not selected in D1.
