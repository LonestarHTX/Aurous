# Milestone 1 — Codex Prompt Sequence

Four prompts implementing plate motion and the full 7-phase reconciliation pipeline. Each prompt builds on the previous and is independently testable.

---

## Prompt 1: Plate Motion Infrastructure

### Goal

Implement rigid plate rotation, the per-plate carried sample workspace, per-timestep motion updates, and the displacement-based reconciliation trigger.

### Context

This is an Unreal Engine 5.7 C++ project. The `FTectonicPlanet` class from M0 has canonical samples (fixed positions on a unit sphere), a permanent Spherical Delaunay Triangulation, per-vertex adjacency, and plates with assigned samples. All code follows UE/Epic C++ conventions (tabs, PascalCase, F-prefix structs).

The architecture doc defines the motion model: each plate is a rigid body that rotates around an axis through the planet center. Between reconciliations, plates carry mutable copies of their samples' evolving crust attributes. Sample positions are never stored per-plate — they're computed on demand as `rotate(canonical_position, plate_quaternion)`.

### What to Build

**1. Per-plate carried sample workspace.**

Each plate needs a mutable workspace for the crust attributes of its owned samples. This is NOT a full copy of `FCanonicalSample` — it stores the canonical sample index plus the mutable fields that evolve between reconciliations: elevation, thickness, age, ridge direction, fold direction, orogeny type, orogeny age, crust type.

Define a struct for this (e.g. `FCarriedSampleData`) and give each `FPlate` an array of them, populated from the canonical samples during initialization and after each reconciliation.

The key insight: positions are never carried. When you need a plate's sample positions (for triangle soup construction), compute them on the fly as `RotateVector(Samples[Index].Position, PlateRotation)`.

**2. Plate rotation state.**

Each plate needs a cumulative rotation represented as an `FQuat`. Initialize to identity. Each timestep, compose a delta rotation (angle = `AngularSpeed`, axis = `RotationAxis`) onto the cumulative quaternion.

**3. Per-timestep motion step.**

```
For each plate:
    compose delta rotation onto cumulative quaternion
    (crust evolution updates — erosion, dampening — are M6, stub them as no-ops for now)
```

This is the only thing that runs every timestep. No inter-plate interaction.

**4. Displacement tracking and reconciliation trigger.**

Track the maximum angular displacement across all plates since the last reconciliation. Trigger reconciliation when:

```
max_displacement > 0.5 × average_sample_spacing
```

Average sample spacing on a unit sphere with N points is `sqrt(4 * PI / N)`. At 500k this is ~0.005 radians, so reconciliation triggers roughly every 3–6 timesteps depending on plate speeds.

Expose a method like `ShouldReconcile()` that checks this condition, and `ResetDisplacementTracking()` to call after reconciliation completes.

**5. SimulationStep method.**

A top-level `StepSimulation()` method on `FTectonicPlanet` that:
- Advances all plates by one timestep (motion step)
- Checks the reconciliation trigger
- If triggered, calls a reconciliation method (stub it to just reset displacement tracking for now — Prompt 3 fills it in)
- Increments a timestep counter

### Files to Modify

- `Source/Aurous/Public/TectonicPlanet.h` — new structs, new methods on FPlate and FTectonicPlanet
- `Source/Aurous/Private/TectonicPlanet.cpp` — implementations

### Acceptance Criteria

1. Each plate has a cumulative rotation quaternion that evolves over timesteps
2. `StepSimulation()` advances all plates by one timestep
3. After enough timesteps, `ShouldReconcile()` returns true based on displacement threshold
4. Carried sample workspaces are populated from canonical data during `InitializePlates()`
5. Rotated positions are computed on demand, never stored per-plate
6. All existing M0 tests still pass
7. New tests verifying: rotation accumulates correctly over multiple steps, displacement tracking triggers at the expected threshold, carried sample workspace contains correct initial values

---

## Prompt 2: Spatial Query Infrastructure

### Goal

Build the spatial query toolbox that reconciliation will use: per-plate triangle soup construction from rotated positions, BVH acceleration via UE5's `TMeshAABBTree3`, spherical bounding cap broad-phase culling, and point-in-triangle containment queries using Shewchuk's `orient3d`.

### Context

This builds on Prompt 1. Plates now have cumulative rotations and carried sample workspaces. The global SDT triangles are classified as interior (all 3 verts same plate) or boundary (verts span plates) from M0.

The containment query answers: "given a point on the unit sphere, which plate's rotated triangle soup contains it, and what are the barycentric coordinates within the containing triangle?" This is the foundation of reconciliation Phase 2.

### What to Build

**1. Triangle soup construction.**

For each plate, build a "moved triangle soup" by:
- Taking the plate's interior triangles (from M0's classification)
- Computing rotated vertex positions on the fly: `RotateVector(canonical_position, plate_quaternion)`
- The soup is a set of triangle indices into the global SDT plus their rotated vertex positions

This gets rebuilt at the start of each reconciliation (Phase 1). It does NOT require retriangulation — same triangle connectivity, just rotated vertex positions.

**2. FPlateTriangleSoupAdapter for TMeshAABBTree3.**

Write a mesh adapter struct that wraps a plate's rotated triangle soup data and satisfies the `TMeshAABBTree3` template requirements. Use the existing `FRealtimeMeshDynamicMeshAdapter` in `Plugins/RealtimeMeshComponent/Source/RealtimeMeshExt/Public/RealtimeMeshDynamicMeshAdapter.h` as a reference for the required interface: `VertexCount()`, `TriangleCount()`, `MaxTriangleID()`, `GetTriangle()`, `GetVertex()`, `GetTriVertices()`, `IsTriangle()`, `GetTriNormal()`, `GetChangeStamp()`.

The adapter should reference the rotated position data and soup triangle list without copying them.

Typedef `TMeshAABBTree3<FPlateTriangleSoupAdapter>` as the per-plate BVH type. Each plate stores one of these, rebuilt each reconciliation.

The header for `TMeshAABBTree3` is `Spatial/MeshAABBTree3.h` from GeometryCore (already a dependency).

**3. Spherical bounding caps.**

Each plate gets a bounding cap: a center direction (mean of its rotated sample positions, normalized) and a half-angle (max angular distance from center to any sample). A point can only be inside a plate's soup if `dot(point, cap_center) >= cos(cap_half_angle)`. This is one dot product per plate — culls ~95% of plates per query point.

Compute caps during Phase 1 alongside soup construction.

**4. Containment query.**

The full query for a single canonical sample position:

1. **Broad phase:** Test the point against each plate's bounding cap. Skip plates that fail.
2. **BVH query:** For surviving plates, call `FindNearestTriangle()` on the plate's BVH to get a candidate triangle.
3. **Exact containment:** Verify the point is actually inside the candidate triangle using three `orient3d` calls (Shewchuk predicates). The test: for spherical triangle ABC and query point P, check `sign(orient3d(O, A, B, P))`, `sign(orient3d(O, B, C, P))`, `sign(orient3d(O, C, A, P))` where O is the origin. If all three signs agree, P is inside ABC.
4. **Barycentric coordinates:** Once the containing triangle is confirmed, compute barycentric coordinates of P within the triangle. Use planar barycentrics — curvature error is negligible (~0.002% at our triangle sizes per the architecture doc).

The query returns: containing plate ID, containing triangle index, barycentric coordinates, and whether multiple plates contain the point (overlap) or no plate contains it (gap).

**5. Expose a method like `QueryContainment(FVector Position)` that returns this result struct.**

### Files to Modify/Create

- `Source/Aurous/Public/TectonicPlanet.h` — result struct, method declarations
- `Source/Aurous/Private/TectonicPlanet.cpp` — implementations
- `Source/Aurous/Private/PlateTriangleSoupAdapter.h` (or inline in TectonicPlanet.h) — the BVH adapter

### Acceptance Criteria

1. Triangle soups are built from interior triangles with on-the-fly rotated positions
2. `TMeshAABBTree3` builds successfully over each plate's soup via the adapter
3. Bounding caps correctly exclude distant plates (verifiable: a point on plate 0 should fail the cap test for most other plates)
4. Containment query correctly identifies which plate contains a given point
5. Barycentric coordinates sum to ~1.0 and are all non-negative for contained points
6. Points in gaps (no plate contains them) are correctly identified
7. Shewchuk predicates are used for the containment test, not naive floating-point
8. All M0 and Prompt 1 tests still pass
9. New tests verifying: soup construction produces correct rotated positions, containment query finds the right plate for known points, barycentric coordinates are correct for known triangle/point configurations, broad-phase culling works

---

## Prompt 3: 7-Phase Reconciliation Pipeline

### Goal

Implement the full reconciliation pipeline that runs every R timesteps. This is the core of M1 — it reassigns canonical sample ownership after plates have moved, interpolates attributes from the moved plate data, and detects gaps/overlaps.

### Context

Prompts 1–2 provide the motion step, triangle soup construction, BVH building, and containment queries. This prompt wires them into the 7-phase reconciliation pipeline described in the architecture doc (Section 4.2).

All reconciliation writes target a NEW buffer — the old canonical sample array remains readable until an atomic swap at the end. This is the double-buffering model. Implement it as two `TArray<FCanonicalSample>` with a swap index.

### The 7 Phases

**Phase 1 — Build moved triangle soups and BVHs.**

For each plate: extract interior triangles, compute rotated vertex positions, build `TMeshAABBTree3`, compute spherical bounding cap. This is the Prompt 2 infrastructure called at the start of reconciliation. Parallelize across plates.

**Phase 2 — Ownership reassignment with boundary-scoped hysteresis.**

For each canonical sample, run the containment query (Prompt 2). This yields the containing plate and triangle, barycentric coordinates, and gap/overlap status.

Hysteresis rule — only applies to samples that were boundary samples in the previous step (`bIsBoundary == true` or `OwnershipMargin` below a confidence threshold). For these samples, if the containing plate differs from `PrevPlateId`, reassignment occurs only if `OwnershipMargin` exceeds `hysteresis_threshold` (configurable, default 0.15).

The ownership margin is: `(distance_to_second_nearest - distance_to_nearest) / average_sample_spacing`. This was fixed in the M0 review.

Interior samples always accept their containment result without stickiness.

If a boundary sample's previous owner no longer contains it (gap or different plate only), reassignment is forced regardless of margin.

This phase is embarrassingly parallel across samples — use `ParallelFor`.

**Phase 3 — Attribute interpolation.**

For each canonical sample that received a containing triangle in Phase 2, interpolate crust attributes from the triangle's three carried-sample vertices using barycentric coordinates.

Three interpolation conventions:
- **Continuous fields** (elevation, thickness, age, orogeny_age): standard barycentric blend of the three vertex values
- **Categorical fields** (crust_type, orogeny_type): majority vote from the three vertices
- **Direction fields** (fold_direction, ridge_direction): blend the three 3D vectors with barycentric weights, project the result onto the tangent plane at the sample position (`v_proj = v - dot(v, p) * p`), renormalize. If the blended magnitude is near zero (cancellation), fall back to the direction of the vertex with the largest barycentric weight.

Write interpolated values to the NEW buffer.

**Phase 4 — Gap detection (stub).**

Any sample not contained by any plate's soup is in a gap. For M1, handle these minimally:
- Assign to the nearest plate by proximity (dot product to plate cap centers)
- Set elevation to a fallback value (abyssal plain depth, -6.0 km)
- Set crust type to oceanic
- Flag as `gap_detected = true` (add this field to FCanonicalSample or use a separate array)

M2 will replace this with proper oceanic crust generation, ridge profiles, and age initialization.

**Phase 5 — Overlap detection (stub).**

A sample contained by multiple plates is in an overlap zone. For M1:
- Log which plates overlap at each sample
- Flag as `overlap_detected = true` with the competing plate IDs stored
- Keep the sample assigned to its current (or nearest) plate
- Optionally: compute the relative velocity of the overlapping plates and the boundary normal to classify as convergent vs. non-convergent. The math is cheap and validates the boundary normal computation early. But don't trigger any subduction or collision response — that's M3/M4.

**Phase 6 — Update plate membership.**

Rebuild each plate's `SampleIndices` and carried sample workspaces from the new canonical sample assignments. Update `PrevPlateId = PlateId` and `OwnershipMargin` for all samples. Reclassify triangles as interior vs. boundary. Recompute `bIsBoundary` flags from the adjacency graph.

**Phase 7 — Terrane detection (stub).**

For M1, this is a no-op stub. Just log that terrane detection would run here. M4 fills it in with connected component analysis and identity tracking.

### Double Buffering

Maintain two sample arrays. Reconciliation reads from the current buffer and writes to the back buffer. After all 7 phases complete, atomically swap the buffer index. The render thread always reads from the current buffer.

### Files to Modify

- `Source/Aurous/Public/TectonicPlanet.h` — new fields (double buffer, gap/overlap flags), reconciliation method declarations
- `Source/Aurous/Private/TectonicPlanet.cpp` — the 7-phase pipeline implementation

### Acceptance Criteria

1. Reconciliation runs as a single `Reconcile()` method that executes all 7 phases in order
2. After reconciliation, every sample has a valid `PlateId` (no orphans)
3. Samples that don't change plate assignment retain their attributes smoothly (no discontinuities)
4. Hysteresis prevents boundary chatter — run 20+ reconciliations and verify boundary samples don't flip back and forth
5. Barycentric interpolation produces smooth attribute transitions (continuous fields blend, categoricals use majority vote, directions are projected and renormalized)
6. Gap samples are detected and assigned to nearest plate with fallback values
7. Overlap samples are detected and flagged
8. Double buffering: reconciliation writes don't corrupt the readable buffer
9. All prior tests pass
10. New tests: run 10+ `StepSimulation()` calls, verify no orphans after each reconciliation, verify boundary stability, verify attribute continuity

---

## Prompt 4: Simulation Loop + Actor Integration

### Goal

Wire the simulation loop together with the actor's mesh rendering, run on a background thread, and add per-phase timing instrumentation.

### Context

Prompts 1–3 provide the motion step and reconciliation pipeline. This prompt creates the runtime simulation loop, pushes updated data to the actor's mesh each frame, and instruments the pipeline for performance validation.

### What to Build

**1. Background simulation thread.**

The simulation runs on a background thread via UE5's async task system (`FAsyncTask` or `Async()`). The game thread should not block on simulation. The simulation thread:
- Calls `StepSimulation()` in a loop
- When reconciliation fires, it writes to the back buffer and swaps
- Signals the game thread that new data is available (atomic flag or similar)

**2. Actor mesh update.**

The actor (on the game thread) checks each tick whether new data is available. If so, it updates the mesh vertex attributes — positions don't change (the canonical positions are fixed), but colors need to update to reflect new plate assignments, elevations, and boundary flags.

Use `EditMeshInPlace` on the RealtimeMesh to update vertex colors based on the current debug mode, same as `UpdateDebugColors()` but driven by the simulation producing new data.

**3. Simulation controls on the actor.**

Add properties to the actor for:
- Start/stop simulation
- Simulation speed (timesteps per second)
- Current timestep counter (read-only display)
- Whether reconciliation triggered this frame (read-only display)

**4. Per-phase timing instrumentation.**

Add timing around each of the 7 reconciliation phases. Log and store the results so they can be inspected. The architecture doc predicts these ranges for 500k single-threaded:

| Phase | Expected |
|---|---|
| Phase 1 (soup + BVH) | 5–20 ms |
| Phase 2 (containment) | 10–30 ms |
| Phase 3 (interpolation) | < 5 ms |
| Phase 4 (gap detection) | < 1 ms |
| Phase 5 (overlap detection) | < 1 ms |
| Phase 6 (membership update) | < 5 ms |
| Phase 7 (terrane stub) | ~0 ms |
| **Total** | **15–55 ms** |

Log per-phase times each reconciliation. If any phase is dramatically outside these ranges, that's a signal something is wrong with the implementation (e.g., broad-phase culling not working, BVH building N times instead of once per plate).

**5. End-to-end validation.**

After wiring everything together, run 100+ timesteps and verify:
- No orphan samples after any reconciliation
- No boundary chatter (boundary samples don't flip back and forth)
- Smooth attribute transitions across plate boundaries
- Stable plate boundaries that move with the plates
- Total reconciliation time is in the expected range for 500k

### Files to Modify/Create

- `Source/Aurous/Public/TectonicPlanetActor.h` — simulation control properties, tick override
- `Source/Aurous/Private/TectonicPlanetActor.cpp` — background thread, mesh update, timing
- `Source/Aurous/Public/TectonicPlanet.h` — timing storage, simulation controls
- `Source/Aurous/Private/TectonicPlanet.cpp` — timing instrumentation in reconciliation

### Acceptance Criteria

1. Simulation runs on a background thread, does not block the editor/game thread
2. Actor mesh updates reflect plate motion (colors shift as boundaries move)
3. Simulation can be started/stopped from the actor's details panel
4. Per-phase timing is logged each reconciliation
5. Total reconciliation time at 500k is under 60ms single-threaded (architecture doc target: 15–55ms)
6. 100+ timesteps run without crashes, orphan samples, or boundary instability
7. All prior tests pass
8. New tests: run 50+ timesteps, verify orphan-free, verify per-phase timing is logged, verify thread safety (no data races on buffer swap)
