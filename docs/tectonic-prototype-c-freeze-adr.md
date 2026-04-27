# ADR: Prototype C Freeze

Date: 2026-04-27

## Decision

Prototype C is the frozen ownership and projection foundation for the sidecar
tectonics path.

Prototype C proves:

- each canonical sample has exactly one owner from nearest currently rotated plate center
- boundaries are raw one-ring owner adjacency
- carried material is projected separately from ownership
- material/owner mismatch is diagnostic evidence only
- projection writes to `FTectonicPlanet` and diagnostic caches only
- V6/v9 ownership recovery, active-zone repair, and generic oceanization are not part of C

Prototype C does not prove persistent ocean crust, age gradients, thickness
accumulation, subduction, collision, rifting, uplift, or long-horizon balance.
Those are Prototype D and later work.

## Frozen Invariants

- Owner formula: `argmax(dot(sample position, rotated plate center))`, with lower `PlateId` as deterministic tie-break.
- Boundary formula: `bIsBoundary == any one-ring neighbor has a different owner`.
- `OwnershipGapFraction == 0` and `OwnershipOverlapFraction == 0`.
- Material source may differ from current owner. Mismatch is never a repair trigger.
- Material never writes ownership. Ownership never writes material.
- `ProjectToPlanet` is read-only with respect to sidecar truth.
- `FTectonicPlanet.Samples` is projection/cache output, not tectonic authority.
- `OceanFallback` and `DivergentOceanFill` are projection classifications only in C.
- Material mutation after C must happen only through explicit named D events.

## Blocker Bar

Freeze is blocked by any code-read or test finding that demonstrates:

- C can reach V6/v9 ownership, remesh, active-zone, quiet-interior, or fallback-recovery paths
- material can influence `PlateId`
- ownership can rewrite material support/source
- generic oceanization creates persistent sidecar material
- projection mutates sidecar truth

Findings outside those invariant violations are tracked as follow-up tasks unless
they invalidate the test evidence.

## Hardening PR Rule

The freeze-hardening PR is verification work, not tectonic behavior work.

Allowed production changes:

- remove unused locals or add comments
- add guards that preserve the existing C mode
- rename/split visualization modes so exports are honest
- add diagnostic-only comments

Not allowed:

- changing how C computes `PlateId`
- changing how C computes `bIsBoundary`
- changing how C projects material
- changing diagnostic formulas
- adding tectonic material mutation

Any such behavior change requires a separate PR and re-validation against the
current C export baseline.

## Red-Flag Investigation

### `OceanFallback`

Where written:

- `Source/Aurous/Private/TectonicPlanetSidecar.cpp:1743-1774`, inside `FTectonicPlanetSidecar::ProjectVoronoiOwnershipDecoupledMaterialToPlanet`.
- Classification enum is declared in `Source/Aurous/Public/TectonicPlanetSidecar.h:17-23`.

What it touches:

- output `FSample` fields on `OutPlanet`
- projection scratch flags
- mutable diagnostic cache `LastMaterialClassifications`

What it does not touch:

- `FTectonicSidecarPlate::MaterialSamples`
- `FTectonicSidecarPlate::FootprintTriangles`
- owner assignment
- sidecar plate transforms

Conclusion:

`OceanFallback` is kept as a projected non-mutating miss classification. In C it
does not create persistent ocean material or mutate sidecar truth. If D creates
ocean crust, it must do so through a named persistent event, not this projection
classification.

### `MeaningfulHitContainmentScore`

Where read:

- `Source/Aurous/Private/TectonicPlanetSidecar.cpp:1419`, inside explicit-footprint mode.
- `Source/Aurous/Private/TectonicPlanetSidecar.cpp:1696`, inside Voronoi ownership plus decoupled material mode.
- Config field is declared in `Source/Aurous/Public/TectonicPlanetSidecar.h:41`.

What it touches:

- overlap counts and overlap diagnostic flags
- projected material overlap reporting

What it does not touch:

- owner assignment in Prototype C
- boundary assignment
- sidecar material support
- persistent material state

Conclusion:

`MeaningfulHitContainmentScore` is kept as a diagnostic overlap threshold. It is
not ownership authority and does not mutate sidecar truth.

## D Extension Contract

C is Data:

- recomputed owners
- recomputed boundaries
- projected material samples
- mismatch / overlap / divergent-boundary diagnostics
- exports and visualization

D is Crust:

- persistent ocean crust identity
- crust age and thickness
- source event id
- terrane/suture/accretion state
- subduction/consumption/collision/rift event logs

D may read C signals to form event candidates. D must not mutate C ownership or
derive ownership from D material state. D persistent state must live in
sidecar-owned crust/event stores, not in `FTectonicPlanet.Samples` as authority.

## Internal Pre-Mortem

If C fails after freeze, likely symptoms are:

- a future projection mode accidentally becomes the Sidecar C default
- a visualization/export mode makes material classification look like ownership gaps
- a D event writes projected sample output back into sidecar truth
- a helper shared with V6 introduces repair semantics without naming V6 directly
- a test trusts sidecar diagnostics instead of independently recomputing the formula

Mitigations in the freeze-hardening PR:

- all-sample independent owner and boundary recomputation
- source-token contamination test for obvious V6/v9 repair symbols
- explicit code-read requirement for shared helpers whose names are not V6-specific
- explicit actor projection-mode guard
- export-mode split for material classification and material overlap
- ADR blocker bar for any coupling or mutation finding

## Follow-Up

Track a future task to extract the C path into its own class once A/B diagnostic
modes can be deprecated. Deprecation criteria:

- A/B evidence is no longer needed in normal automation
- C freeze tests remain green on the extracted class
- D event code has a separate persistent crust/event store
