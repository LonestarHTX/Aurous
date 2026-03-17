# Milestone 2 — Codex Prompt Sequence

Three prompts implementing oceanic crust generation at divergent boundaries. Each prompt builds on the previous and is independently testable.

---

## Prompt 1: Boundary Classification Infrastructure

### Goal

Implement the velocity-based boundary classification that distinguishes divergent, convergent, and transform boundaries. This is foundation infrastructure for both M2 (oceanic generation at divergent boundaries) and M3 (subduction at convergent boundaries).

### Context

This is an Unreal Engine 5.7 C++ project. The `FTectonicPlanet` class has canonical samples with `bIsBoundary` flags, a permanent Delaunay adjacency graph, and plates with rigid rotation (axis + angular speed + cumulative quaternion). Boundary samples are already identified — any sample with a Delaunay neighbor on a different plate is flagged as boundary. What's missing is *classifying* those boundaries as convergent, divergent, or transform.

The architecture doc (Section 6) defines the classification:
- Compute a **boundary normal** at each boundary sample: mean direction to all cross-plate Delaunay neighbors (1-ring), projected onto the tangent plane at the sample position, normalized
- Compute **relative surface velocity** between the two plates at the sample position: `v_rel = v_plate_i(p) - v_plate_j(p)` where surface velocity is `v(p) = omega × p` (cross product of plate's rotation axis scaled by angular speed with the sample position)
- Classify: `dot(v_rel, n_boundary) < 0` → convergent, `> 0` → divergent, predominantly tangential → transform

### What to Build

**1. Surface velocity computation.**

Add a method to compute a plate's surface velocity at any point on the sphere: `v(p) = (RotationAxis * AngularSpeed) × p`. This is a simple cross product — the angular velocity vector crossed with the position gives the linear velocity direction and magnitude at that point.

**2. Boundary normal computation.**

For each boundary sample, compute the boundary normal by averaging the directions to all Delaunay neighbors that belong to a different plate. Project onto the tangent plane at the sample position (`n_proj = n - dot(n, p) * p`), then normalize. Store per-sample. If a boundary sample has neighbors on multiple different plates, use all cross-plate neighbors together — the resulting normal points "outward" from the boundary.

**3. Boundary classification enum and per-sample storage.**

Add an enum `EBoundaryType { None, Convergent, Divergent, Transform }` and store it per canonical sample. Compute during Phase 6 (after boundary flags are updated) or as a new sub-phase. For non-boundary samples, set to `None`.

Transform classification: if the absolute dot product of the relative velocity with the boundary normal is less than some fraction (e.g. 0.3) of the relative velocity magnitude, classify as transform. Otherwise use the sign to pick convergent vs divergent.

**4. Flanking plate identification for gap samples.**

Gap samples need to know which two plates flank them. Add a helper that, for a given gap sample, examines its Delaunay neighbors to find the two most common plate IDs. These are the flanking plates whose relative motion created the gap. Store both flanking plate IDs on gap samples for use in Phase 4 ridge direction computation.

**5. Debug visualization.**

Add a new `EPlanetDebugMode` entry for `BoundaryType` that colors boundary samples: red for convergent, blue for divergent, green for transform, black for non-boundary. Add it to the debug mode dropdown in the control panel.

### Files to Modify

- `Source/Aurous/Public/TectonicPlanet.h` — new enum, new fields on FCanonicalSample, new methods
- `Source/Aurous/Private/TectonicPlanet.cpp` — boundary normal computation, velocity computation, classification logic
- `Source/Aurous/Public/TectonicPlanetActor.h` — new debug mode enum entry
- `Source/Aurous/Private/TectonicPlanetActor.cpp` — debug color for boundary type
- `Source/Aurous/Private/Editor/SAurousTectonicControlPanel.cpp` — add BoundaryType to debug mode dropdown

### Acceptance Criteria

1. Each boundary sample has a computed boundary normal (tangent-plane projected, normalized)
2. Each boundary sample is classified as convergent, divergent, or transform
3. Surface velocity computation is correct (cross product of angular velocity with position)
4. Gap samples identify their two flanking plates from Delaunay neighbor analysis
5. BoundaryType debug mode shows colored boundaries in the viewport
6. Classification updates each reconciliation alongside boundary flag updates
7. All existing tests pass
8. New tests: verify boundary normal points away from the plate interior, verify classification matches expected direction for a known plate pair (two plates moving apart should produce divergent boundaries between them)

---

## Prompt 2: Phase 4 — Oceanic Crust Generation at Divergent Boundaries

### Goal

Replace the Phase 4 gap detection stub with proper oceanic crust initialization. Gap samples become new oceanic crust with elevation set by a ridge blending profile, ridge direction computed from flanking plate motion, and age initialized to zero.

### Context

Prompt 1 provides boundary classification and flanking plate identification. The current Phase 4 stub (in `Reconcile()` around line 962) assigns gap samples to the nearest plate with flat -6.0 km elevation. This prompt replaces that with the architecture doc's oceanic generation model (Section 5.3).

The key formula for ridge elevation blending:
```
α = d_Γ(p) / (d_Γ(p) + d_P(p))
z(p) = α · z̄(p) + (1 - α) · z_Γ(p)
```
Where:
- `d_Γ(p)` = distance from sample to the ridge line (midline of the gap zone)
- `d_P(p)` = distance from sample to the nearest plate border (non-gap boundary sample)
- `z̄(p)` = interpolated elevation from the nearest plate border samples
- `z_Γ(p)` = ridge template elevation (parameter `z_r = -1.0 km` from the architecture doc)
- `α` blends: at the ridge (d_Γ ≈ 0), elevation ≈ z_Γ (ridge height). At the plate border (d_P ≈ 0), elevation ≈ z̄ (existing border elevation). Smooth transition in between.

### What to Build

**1. Ridge line estimation for gap zones.**

The ridge line is the approximate midline of the gap between two diverging plates. For each gap sample, estimate its distance to the ridge by comparing distances to the two flanking plates' nearest boundary samples. The ridge is roughly equidistant from both plates' borders, so: `d_Γ(p) ≈ |d_to_plate_A(p) - d_to_plate_B(p)| / 2` where distances are to the nearest non-gap boundary sample of each flanking plate.

A practical approach: for each gap sample, find its nearest non-gap neighbor belonging to each flanking plate (walk Delaunay neighbors). The ridge position is the midpoint, and the distances follow from that.

**2. Border elevation interpolation.**

For each gap sample, `z̄(p)` is the elevation of the nearest non-gap boundary sample (or an average of the nearest few). This provides the "plate edge" elevation that the ridge profile blends toward at the border.

**3. Ridge elevation blending.**

Apply the formula: `z(p) = α · z̄(p) + (1 - α) · z_Γ(p)` with `z_Γ = -1.0` (ridge elevation from parameters). Samples near the ridge center get elevation close to -1.0 km. Samples near the plate border get elevation close to the existing border value. This produces a smooth ridge profile.

**4. Ridge direction computation.**

Per the architecture doc: `r(p) = normalize((p - q) × p)` where q is the projection of p onto the ridge line. In practice, the ridge direction at a gap sample is perpendicular to the relative motion of the flanking plates, projected onto the tangent plane. Compute as: take the relative velocity vector of the two flanking plates at the sample position, cross it with the sample position (surface normal), project onto tangent plane, and normalize. This gives the ridge-parallel direction.

Store in `RidgeDirection` on the canonical sample.

**5. Flanking plate assignment.**

Assign each gap sample to the nearer of its two flanking plates (by distance to nearest boundary sample of each).

**6. Boundary-triangle gap fallback.**

Some gap samples aren't true divergent gaps — they fall in narrow cracks between discarded boundary triangles but are surrounded by a single plate's samples. Detect these: if a gap sample's Delaunay neighbors are predominantly (>80%) from a single plate, treat it as a boundary-triangle artifact rather than a divergent gap. Assign it to that plate, copy elevation from nearest neighbor, don't set ridge properties.

**7. Updated gap sample initialization.**

For true divergent gap samples:
- `CrustType = Oceanic`
- `Age = 0`
- `Elevation` = ridge blending formula
- `Thickness = 7.0` (standard oceanic crust thickness)
- `RidgeDirection` = computed as above
- `PlateId` = nearer flanking plate
- `bGapDetected = true`

### Files to Modify

- `Source/Aurous/Private/TectonicPlanet.cpp` — replace Phase 4 stub with full implementation
- `Source/Aurous/Public/TectonicPlanet.h` — any new helper method declarations

### Acceptance Criteria

1. Gap samples at divergent boundaries get ridge-blended elevation (not flat -6.0)
2. Ridge elevation profile is smooth — samples near ridge center are around -1.0 km, samples near plate border approach the border elevation
3. Ridge direction is computed and stored, perpendicular to relative plate motion
4. Gap samples are assigned to the nearer flanking plate
5. Boundary-triangle artifact gaps are handled separately (no ridge properties, just nearest-plate assignment)
6. Elevation export map shows smooth ridge profiles at divergent boundaries (export and inspect)
7. All existing tests pass
8. New tests: create a configuration with two plates diverging, run enough steps for gaps to form, verify gap sample elevations follow the blending formula, verify ridge directions are tangent to the sphere and perpendicular to relative motion

---

## Prompt 3: Oceanic Crust Aging and Dampening

### Goal

Implement per-timestep oceanic crust aging and elevation dampening so ocean floor subsides as it moves away from ridges, producing the classic bathymetric profile of deepening ocean floor.

### Context

Prompts 1–2 establish divergent boundary classification and oceanic crust initialization at ridges. New oceanic crust starts at age 0 with ridge elevation (~-1.0 km). In reality, as oceanic crust ages it cools, densifies, and sinks — producing deeper ocean floor far from ridges. The architecture doc (Section 7) defines the dampening formula.

Currently the per-timestep motion step (`AdvancePlateMotionStep`) has a comment stub: `// M6 crust evolution hooks (erosion, dampening, etc.) run here.` This prompt wires in oceanic dampening. Continental erosion and sediment accretion remain stubs for M6.

### What to Build

**1. Oceanic age increment.**

Each timestep, increment `Age` on all carried samples with `CrustType == Oceanic` by the timestep duration (2 My). This runs during the per-timestep motion step on the carried sample workspaces (not canonical samples — those only update at reconciliation).

**2. Oceanic elevation dampening.**

Per the architecture doc formula:
```
z(p, t+δt) = z(p, t) - (1 - z(p, t) / z_t) · ε_o · δt
```
Where:
- `z_t = -10.0` km (oceanic trench elevation, the asymptotic floor)
- `ε_o = 0.04` mm/year (dampening rate)
- `δt = 2` My = 2,000,000 years

This means the dampening amount per step is: `(1 - z / z_t) · ε_o · δt`. As elevation approaches `z_t`, the dampening term approaches zero (asymptotic). Fresh ridge crust at -1 km dampens faster than old abyssal crust at -6 km.

Apply this to all oceanic carried samples each timestep.

**3. Parameters as configurable constants.**

Define the key parameters as named constants or configurable fields on `FTectonicPlanet` (not magic numbers in the formula):
- `OceanicDampeningRate` (ε_o) = 0.04
- `OceanicTrenchElevation` (z_t) = -10.0
- `TimestepDuration` (δt) = 2,000,000 (years, for age) or 2.0 (My, for dampening calc — be consistent with units)
- `RidgeElevation` (z_r) = -1.0 (from Prompt 2, but ensure it's a named parameter)
- `AbyssalPlainElevation` (z_a) = -6.0

**4. Continental erosion and sediment accretion stubs.**

Add the formulas from the architecture doc as commented-out or no-op stubs in the same timestep location, with clear TODO markers for M6:
- Continental erosion: `z -= (z / z_c) · ε_c · δt`
- Sediment accretion: `z += ε_f · δt`

Don't activate them — just place the structure so M6 has a clear insertion point.

**5. CrustAge debug visualization.**

Add a new `EPlanetDebugMode` entry for `CrustAge` that maps age to a color ramp (young = warm/red, old = cool/blue). Add to the control panel dropdown. This is the primary way to visually verify that oceanic crust ages correctly — young crust near ridges, old crust far from ridges.

### Files to Modify

- `Source/Aurous/Public/TectonicPlanet.h` — parameter fields, any new method declarations
- `Source/Aurous/Private/TectonicPlanet.cpp` — dampening in motion step, parameter initialization
- `Source/Aurous/Public/TectonicPlanetActor.h` — new debug mode
- `Source/Aurous/Private/TectonicPlanetActor.cpp` — CrustAge color mapping
- `Source/Aurous/Private/Editor/SAurousTectonicControlPanel.cpp` — add CrustAge to dropdown

### Acceptance Criteria

1. Oceanic crust age increments each timestep on carried samples
2. Oceanic elevation dampens toward `z_t` asymptotically each timestep
3. After many timesteps, oceanic crust far from ridges has elevation near -6 km (abyssal), while fresh ridge crust is near -1 km
4. Dampening formula matches the architecture doc exactly
5. All parameters are named constants, not magic numbers
6. Continental erosion and sediment accretion stubs are in place with M6 TODOs
7. CrustAge debug mode shows a meaningful color ramp
8. Elevation export map after 20+ steps shows deepening ocean floor away from ridges
9. All existing tests pass
10. New tests: verify age increments correctly over N steps, verify dampening produces expected elevation change for known input values, verify dampening asymptotes (crust at z_t doesn't dampen further)
