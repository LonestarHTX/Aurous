# Tectonic Minimal Acceptance Tests

**Status:** Current hard gates for any future tectonic architecture, prototype, or remesh mode.

**Purpose:** Prevent the project from accepting a system that looks good on one proxy while failing the actual product requirements.

## 1. Principles

These are acceptance tests, not diagnostics-only curiosities.

If a prototype fails one of the hard gates below, it is not "stylized but acceptable." It failed the tectonic bar.

The tests are intentionally simple:

1. moving continents
2. coherent plate regions
3. thin meaningful boundaries
4. coherent basin opening
5. bounded land-ocean balance

## 2. Drift Gate

Continental interiors must visibly and materially move with their plates.

Required evidence:

1. expected plate-frame drift and ownership/material-footprint drift are of the same order
2. step-0 continental-core overlap drops materially over time
3. materially anchored fraction becomes low, not dominant
4. `PlateId`, `CrustType`, and combined summary exports show continental bodies traversing the globe rather than growing in place

Minimum read:

- a viewer should be able to compare `step 0 -> step 100 -> step 200 -> step 400` and see real continental transport

Failure examples:

- healthy velocity arrows, but continents still visually stay in place
- drift metrics improve only by noisy local reassignment while plate regions disintegrate

## 3. Region-Coherence Gate

Plate-ID regions must remain globally legible as regions, not as speckled ownership noise.

Required evidence:

1. large plates retain dominant connected components
2. plate-ID maps do not devolve into salt-and-pepper ownership fields
3. interior samples do not look boundary-like across broad areas

Useful proxies:

- largest connected component fraction per large plate
- fragmentation/island count proxy
- isolated ownership speckle proxy

Failure examples:

- large regions dissolve into scattered patches
- ownership churn becomes visually area-filling even when drift looks numerically better

## 4. Boundary Gate

Boundaries must be thin, connected, and sparse.

Required evidence:

1. boundary coverage stays low relative to total samples
2. boundaries appear as networks or corridors, not broad continents-sized masks
3. boundary and overlap masks remain localized near true tectonic interfaces

Useful proxies:

- boundary fraction / boundary coverage
- overlap coverage
- boundary connectedness / mean thickness proxy

Failure examples:

- half the world reads as boundary
- a boundary mask becomes area-filling after an ownership or oceanization change

## 5. Basin-Opening Gate

When a real rift occurs, it must open a coherent oceanic corridor between child plates.

Required evidence:

1. pair-scoped oceanic creation becomes nonzero on the rift child pair
2. a visible oceanic corridor appears between the children
3. the corridor persists for `+10` and `+25` steps
4. opening remains pair-local rather than becoming global speckle

Prototype B note:

Prototype B can satisfy this gate only as a spatial projection proof: a corridor is present at each checkpoint, but the projected ocean fill is created fresh from current footprint geometry. It does not prove persistent identifiable oceanic crust, age gradients, or thickness accumulation.

Prototype C note:

Prototype C intentionally does not attempt this gate. It freezes clean Voronoi ownership, raw adjacency boundaries, and decoupled carried material so later tectonic processes have a stable foundation.

Prototype D note:

Persistent identifiable oceanic crust, age gradients, and thickness accumulation remain Prototype D requirements.

Required visual exports around each audited rift:

- `CombinedTectonicSummary.png`
- `CrustType.png`
- `PlateId.png`
- `ElevationEnhanced.png`

Checkpoints:

- `R`
- `R+1`
- `R+5`
- `R+10`
- `R+25`

Failure examples:

- metrics show oceanic creation, but maps show only scattered blue speckle
- a boundary separates, but no coherent basin forms

## 6. Balance Gate

The system must remain in a bounded land-ocean regime over long horizons.

Required evidence:

1. CAF stays in a plausible bounded range for the target design
2. oceanic crust remains substantial through long-horizon checks
3. the system does not run monotonically toward near-total continentalization

Minimum long-horizon checkpoints:

- `step 200`
- `step 300`
- `step 400`
- `step 500`

Required outputs:

- CAF
- oceanic fraction
- subaerial fraction
- largest subaerial component
- collision count
- rift count

For the current project goal, `CAF ~0.9+` is a failure, even if it is better than a previous `CAF ~0.99`.

## 7. Visual Gate

The tectonic maps must be legible without a long explanation.

Required maps for baseline evaluation:

- `PlateId.png`
- `CrustType.png`
- `ElevationEnhanced.png`
- `CombinedTectonicSummary.png`

If the result needs an elaborate metric argument to defend obviously incoherent visuals, it failed.

Examples of immediate visual rejection:

- salt-and-pepper ownership
- area-filling boundary masks
- global speckled oceanization
- continents that clearly do not travel

## 8. Interpretation Rules

These rules are here to prevent the project from lying to itself.

1. Nonzero velocity arrows do not prove continental drift.
2. Lower churn does not prove plate quality if continents are anchored.
3. Higher drift does not prove success if plate regions and boundaries collapse.
4. Nonzero oceanic creation does not prove basin opening if the visual result is speckled.
5. Improvement relative to a catastrophic baseline does not equal acceptance.

## 9. Minimum Promotion Checklist

Before promoting any future prototype or architecture direction, all of these must be true:

1. continental interiors visibly advect
2. plate-ID regions remain region-coherent
3. boundaries stay thin and localized
4. post-rift basins open coherently and persist
5. long-horizon CAF remains in a bounded acceptable regime

If one of those fails, the prototype remains a diagnostic result, not a promoted direction.

Prototype C freeze exception:

Prototype C is promotable only as the ownership/material foundation. It is not a full tectonic promotion and is not expected to satisfy the basin-opening or balance gates until Prototype D and later persistent crust processes exist.
