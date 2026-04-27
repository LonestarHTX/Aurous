# 0002: Persistent Divergent Ocean Crust

Status: Accepted
Date: 2026-04-27
Depends on: ADR 0001, Prototype C freeze

## Context

Prototype C proves clean ownership, clean boundaries, and decoupled carried
material. It does not prove basin opening as persistent crust. Prototype B could
show a spatial ocean corridor, but that corridor was projected fresh from
current geometry and had no identity, age, or thickness history.

Prototype D starts by adding one persistent tectonic process: divergent ocean
crust creation.

No Prototype D code landed before this ADR was accepted. After acceptance, only
D Slice 1 may land first: persistent state model, event log, authority hash, and
tests. No event detection, projection, exports, or ocean creation are allowed in
Slice 1.

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
- source boundary edge ids and optional debug sample ids
- ridge direction / spreading direction
- support geometry: boundary edge list plus derived parametric ribbon
- created area
- oceanic age
- oceanic thickness
- projected elevation seed
- parent ridge/event id

All persistent D state must be included in sidecar authority hashing.

## Representation Choice

D1 uses per-event strip/support records represented as a boundary edge list plus
a derived parametric ribbon.

Boundary edge endpoints are stored in world-space unit sphere coordinates at the
event `BirthStep`. They persist thereafter and do not transform with plates
after creation. The crust is event-owned, not plate-owned.

The projected ribbon support is derived from the stored birth edge list by
sweeping each edge along the boundary-normal spreading direction by the
cumulative spreading width:

`WidthKm = NormalSeparationKmPerMy * AgeMy`

The angular width is `WidthKm / PlanetRadiusKm`. Slice 2 tests must compute the
expected analytic persistent area from boundary length, spreading rate, and
elapsed time independently from D diagnostics.

Rejected for D1:

- spherical polygon support, because it is more geometry than D1 needs and
  invites polygon repair before the event model is proven
- per-sample annotation as authority, because it risks recreating global sample authority
- pure parametric ribbon, because it is under-anchored without persisted birth edges
- per-plate fragment ownership, because it prematurely answers accretion/subduction ownership questions
- new strip plates, because plate-count growth is a later architectural decision

Per-event strip/support keeps ocean crust persistent without changing C
ownership.

## Event Rule

`DivergentSpreadingEvent` is the only mutation path in this slice.

Inputs from C:

- owner adjacency
- boundary plate pair ids
- relative plate velocity
- divergent boundary flags/scores
- boundary edges; samples may be used only as debug/projection helpers

Baseline event threshold:

- normal separation speed >= `10 mm/yr`
- measured along the local boundary normal or pair separation direction
- evaluated per boundary edge and aggregated by plate pair

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

After Slice 2 introduces event detection, D evaluates divergent events every
sidecar step. Slice 1 does not evaluate or fire events.

Events are coalesced by:

- sorted plate pair `(min(PlateA, PlateB), max(PlateA, PlateB))`
- connected divergent boundary component id
- ridge generation id

The connected component id is the lowest canonical undirected boundary-edge key
in the component at the component's first accepted event. Later components match
an existing ridge if the sorted plate pair matches and the current component
shares at least one boundary edge with the ridge's last accepted edge set, or if
its closest edge midpoint is within `2 * SampleSpacingRad` of the stored ribbon
support.

The ridge generation id starts at zero and increments only when a previously
matched ridge has been inactive for `RidgeGenerationGapSteps = 5` sidecar steps
and then fires again. Shape changes alone do not increment the generation id.

A later ADR may introduce adaptive or periodic event cadence.

## Event Log And Hash Policy

`FSidecarCrustEventLog` is append-order-sensitive. Each event hash
canonicalizes unordered fields inside the event, including sorted plate pair and
sorted source edge ids, then the aggregate log hash folds events in append
order. This preserves replay history.

`FSidecarOceanCrustStore` is canonical-state hashable independent of container
iteration order. Store hashing sorts crust records by `CrustId` and hashes all
authoritative fields. This makes state equivalence deterministic even if
storage order changes.

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

Initial D projection must not overwrite meaningful carried continental
material. Meaningful continental material is defined as projected carried
`ContinentalWeight >= 0.5` by default. If this becomes configurable, the config
field is `OverlayContinentalWeightThreshold` with default `0.5`.

The D projection elevation seed, once projection lands in a later slice, is:

- age: `0 My`
- thickness: `7 km`
- elevation: `-1 km`

The `-1 km` ridge elevation follows the paper parameter table value `zr`
(`Highest oceanic ridge elevation`) in `docs/ProceduralTectonicPlanets.txt`. The
`7 km` thickness is the existing Aurous sidecar/V6 oceanic-thickness convention
and remains a first-pass placeholder until the cooling/thickness law is defined
by a later ADR.

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
- expected persistent crust area from boundary length, spreading rate, and
  elapsed time with analytic relative error <= 1%
- expected projected/lattice area with tolerance derived from sample spacing,
  never from an uncited fixed magic number
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

## Resolved Acceptance Questions

The Proposed ADR open questions were resolved before acceptance:

- Support geometry is boundary edge list plus derived parametric ribbon, with
  spherical polygon, sample-id authority, pure ribbon, per-plate fragment, and
  strip-plate representations rejected for D1.
- Persistent birth edge endpoints are stored in world-space unit sphere
  coordinates at `BirthStep` and do not transform with plates after creation.
- Analytic persistent area tolerance is <= 1%; projected area tolerance must be
  tied to sample spacing.
- Ridge coalescing key is sorted plate pair, connected divergent boundary
  component id, and ridge generation id.
- Event log hashing is order-sensitive; crust store hashing is sorted by
  `CrustId`.
- Boundary edges are authoritative for event creation; samples are debug and
  projection helpers only.
- Initial D projection must not overwrite carried material with
  `ContinentalWeight >= 0.5`.
- Projected elevation seed uses age `0 My`, thickness `7 km`, and elevation
  `-1 km`; ridge elevation is paper-cited, thickness is a D1 placeholder.
