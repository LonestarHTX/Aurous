# Procedural Tectonic Planets — Architecture Document v5

**Based on:** Cortial et al., "Procedural Tectonic Planets" (2019)
**Amplification based on:** Cortial et al., "Real-Time Hyper-Amplification of Planets" (2020)
**Target:** Unreal Engine 5.7
**Date:** March 2026

**Status:** Constrained reimplementation of the paper's architecture, informed by v4 lessons.
**Current stable point:** See [Preserve Ownership Stable Point](./preserve-ownership-stable-point.md) for the March 2026 ownership-refresh decision and [M1-M5 Lessons Learned](./m1-m5-lessons-learned.md) for the current checkpoint, caveats, and restart guidance.

---

## 0. Why v5

v4 accumulated architectural policy in the wrong places. The three clearest paper divergences — boundary triangles split with synthetic vertices in containment soups, resampling triggered at sub-sample displacement, and a 10-stage ownership repair pipeline — are interconnected. Each compensates for the others. Correcting any one of them in isolation requires renegotiating the rest.

A constrained reimplementation is faster than incremental repair because:

- The paper's architecture is simpler than v4's current implementation.
- v4's paper-aligned pieces (Fibonacci/Delaunay substrate, carried-sample per-step loop, gap-based oceanic generation, per-step uplift/dampening) are worth preserving as concepts, but the control flow around them has diverged.
- The lessons from v4 — what failed, what was compensatory, what was load-bearing — are the most valuable output of that effort.

v4 remains available as a read-only reference and benchmarking harness. Code is ported to v5 only when the paper or a measured failure justifies it.

### 0.1 Verified v4 Divergences From Paper

| Issue | Paper says | v4 does | Evidence |
|---|---|---|---|
| Containment geometry | All triangles partitioned — boundary triangles assigned to majority-vertex plate, each vertex rotated by its own plate (paper p. 9: "the plate they intersect") | Splits boundary triangles with synthetic midpoint/centroid vertices, creating massive overlaps | `TectonicPlanet.cpp` L8581–L8655 |
| Resampling cadence | Every 10–60 timesteps (paper p. 9: "every 10–60 iterations") | Threshold = 0.5 × average sample spacing ≈ 16 km; plate step ≈ 100–300 km; effectively every timestep | `TectonicPlanet.cpp` L2014, L2694 |
| Ownership repair | Paper has no equivalent; repartition by new assignments | 10-stage sanitize/connectivity/rescue pipeline in Phase 6 | `TectonicPlanet.cpp` L3052–L3809 |
| Continental erosion | Per-timestep: `z -= (z/z_c) · ε_c · δt` (paper Section 4.5) | TODO comment, not implemented | `TectonicPlanet.cpp` L2472 |
| Sediment accretion | Per-timestep: `z += ε_f · δt` (paper Section 4.5) | TODO comment, not implemented | `TectonicPlanet.cpp` L2472 |
| Distance-to-front | Incrementally overestimated per-timestep: `d(p,t+δt) ≥ d(p,t) + ‖s(p)‖δt` (paper p. 9) | Recomputed from scratch at reconcile time only | `TectonicPlanet.cpp` L8094 |
| Continental initialization | Per-sample continental_weight; terranes are connected continental regions within plates (paper p. 5) | Whole plates marked continental | `TectonicPlanet.cpp` L2249, L2372 |
| Boundary warping | Noise-warped geodesic distances to plate centroids (paper p. 5) | Unwarped nearest-centroid Voronoi | `TectonicPlanet.cpp` L2057, L2099 |

### 0.2 Do Not Repeat v4

These are explicit prohibitions for v5 development. Each one traces to a specific v4 failure mode.

1. **Do not split boundary triangles with synthetic vertices.** v4 split boundary triangles with synthetic midpoint/centroid vertices. This created massive overlaps at every plate boundary, producing noisy ownership and jagged plate edges. Boundary triangles are included in containment soups unsplit, assigned to their majority-vertex plate, with each vertex rotated by its own plate's rotation.

2. **Do not trigger resampling from sub-sample displacement.** v4 used 0.5 × sample spacing, causing reconciliation every timestep at 500k. The paper resamples every 10–60 steps. Frequent resampling compounds boundary noise and majority-vote CrustType erosion.

3. **Do not build an ownership repair pipeline.** v4's Phase 6 grew to 10 stages (sanitize, connectivity enforcement, protected plate rescue, etc.) to compensate for upstream containment and cadence problems. In v5, if ownership produces artifacts, fix the containment or cadence — do not add repair passes.

4. **Do not initialize whole plates as continental.** v4 marked entire plates as continental, making CrustType maps look like PlateId masks. The paper models crust type per-sample, with terranes as connected continental regions within plates.

5. **Do not add hysteresis, contender support, fast path, or similar ownership heuristics in the first baseline.** These were compensatory mechanisms in v4. The paper does not describe them. If the baseline produces ownership noise, diagnose the root cause first.

6. **Do not use exporter-side repair as evidence of simulation health.** v4 exporters applied nearest-sample fills and fallback weights that masked simulation defects. Exporters must faithfully render the coarse state.

7. **Do not conflate "not yet amplified" with "broken."** The paper's visuals (Figures 1, 13, 14, 16, 17) are amplified terrain T, not raw crust C. Raw C at 35 km resolution is coarse by nature. The test is whether raw C shows recognizable continents, ocean basins, mountain ranges, and coherent plate boundaries — not whether it looks like the paper's rendered planets. Figure 12 (left panel) is the fair comparison target for raw C.

---

## 1. Core Principle

The planet is a fixed set of N sample points on a sphere. These points never move, never change count, and never change topology. They are the authoritative crust state.

Plates are rigid bodies defined by geodetic movement. Each plate carries copies of the crust attributes for its member samples. Between resamplings, plates move and per-sample attributes are updated locally (uplift, erosion, dampening). Periodically, the fixed canonical samples are repopulated from the moved plate data via barycentric interpolation. Gaps become new oceanic crust. Overlaps indicate convergent zones.

**Paper reference:** Section 6, p. 9: "The crust is initialized as a set of attributed points p over the sphere... Instead of incrementally remeshing the planet at every step, which would be computationally demanding, we compute the movement of the plates and perform a global resampling every 10–60 iterations."

The global Spherical Delaunay Triangulation is built once and reused permanently — for adjacency, rendering, BFS distance fields, terrane analysis, and as the source topology for plate triangle soups. It is never rebuilt.

---

## 2. Data Model

### 2.1 Canonical Sample

A fixed array of N points distributed over the unit sphere via Fibonacci sampling, allocated once at initialization.

**Paper reference:** Section 6, p. 9: "To generate a near-optimal distribution of the sample points, we rely on Fibonacci sampling of the sphere."

Each sample stores:

| Field | Type | Description | Paper ref |
|---|---|---|---|
| `position` | Vector3 | Fixed location on unit sphere | Section 6 |
| `plate_id` | int | Current owning plate | Section 6 |
| `continental_weight` | float | Continuous [0,1]: 1.0 = continental, 0.0 = oceanic. Barycentric-interpolated. `crust_type` is derived: continental if ≥ 0.5, oceanic otherwise | Table 1: x_C |
| `elevation` | float | Surface elevation (km, relative to sea level = 0) | Table 1: z |
| `thickness` | float | Crust thickness (km) | Table 1: e |
| `age` | float | Crust age (My). Oceanic: age since formation. Continental: orogeny age a_c | Table 1: a_o, a_c |
| `ridge_direction` | Vector3 | Local ridge direction (oceanic, tangent to sphere) | Table 1: r |
| `fold_direction` | Vector3 | Local fold direction (continental, tangent to sphere) | Table 1: f |
| `orogeny_type` | enum | None, Andean, or Himalayan | Table 1: o |
| `terrane_id` | int | Stable terrane identifier (continental only, -1 if oceanic) | Section 4.2 |
| `is_boundary` | bool | True if any Delaunay neighbor has different plate_id | Section 6 |

**Fields removed from v4:** `prev_plate_id`, `ownership_margin`. These supported hysteresis, which is not part of the v5 baseline. If needed later, they can be added as a measured deviation.

**Derived per-step fields** (not stored on canonical samples, computed on carried data):

| Field | Type | Description | Paper ref |
|---|---|---|---|
| `subduction_distance_km` | float | Distance to nearest subduction front | Section 4.1, p. 9 |
| `subduction_speed` | float | Convergence speed at nearest front | Section 4.1 |

### 2.2 Global Triangulation

Spherical Delaunay Triangulation built once via `TConvexHull3<double>::Solve()` from UE5 GeometryCore. The 3D convex hull of points on a unit sphere is mathematically equivalent to the SDT.

**Paper reference:** Section 6, p. 9: "We then construct a global Spherical Delaunay Triangulation, that we partition into a set of initial plates P."

This triangulation provides:

- **Neighbor graph:** Delaunay adjacency for boundary detection, BFS distance fields, terrane connectivity.
- **Render mesh:** The triangulation serves directly as the planet's visual surface mesh. Vertex attributes update each step; topology never changes.
- **Plate triangle soups:** Interior triangles (all 3 vertices same plate) are extracted per plate and rotated by the plate's cumulative motion for containment queries.
- **Boundary triangles:** Triangles spanning two or more plates, tracked separately for collision detection. Never placed in containment soups.

### 2.3 Plate Triangle Soups (Containment Only)

After each resampling assigns samples to plates, classify canonical triangles:

- **Interior triangle:** All three vertices belong to the same plate. Assigned to that plate's containment soup.
- **Boundary triangle:** Vertices span two or more plates. **Excluded from all containment soups.** Tracked separately for collision detection (Section 5.4).

When a plate moves, its containment soup moves rigidly — same connectivity, rotated vertex positions. A BVH is built over each plate's moved soup.

**Paper reference:** Section 6, p. 9: "The parameters of the other sample points are computed using barycentric interpolation of crust data from the plate they intersect." Boundary triangles are used for collision: "We detect plate collisions efficiently by tracking the intersection between the boundary triangles of the plates."

**What happens at boundaries:** Discarded boundary triangles create narrow gaps between plate soups. This is correct. Those gaps are exactly where the resampling pipeline classifies samples as divergent (new oceanic crust) or convergent (subduction/collision). Interior samples are cleanly contained by their plate's soup.

**v4 lesson:** v4 split boundary triangles with synthetic midpoints and included them in soups. This created massive overlap regions at every plate boundary, producing noisy ownership and triggering the compensatory Phase 6 repair pipeline. v5 must not repeat this.

### 2.4 Boundary Triangle Tracking (Collision Detection)

Boundary triangles serve collision detection and convergence tracking. They are a separate system from containment soups.

**Paper reference:** Section 6, p. 9: "We detect plate collisions efficiently by tracking the intersection between the boundary triangles of the plates. We also rely on a bounding box hierarchy for every plate to accelerate intersection tests. For each tracked converging triangle, we also update the distance of its vertices to the convergence front at every time step by overestimating: d(p, t + δt) ≥ d(p,t) + ‖s(p)‖δt."

Each boundary triangle is classified:

| Classification | Condition | Used for |
|---|---|---|
| Converging | `dot(v_rel, n_boundary) < 0` | Subduction/collision front tracking |
| Diverging | `dot(v_rel, n_boundary) > 0` | Divergent zone identification |
| Transform | Relative velocity predominantly tangential | Transform boundary marking |

Converging boundary triangles maintain per-vertex distance-to-front estimates that are incrementally advanced per-timestep. This avoids expensive BFS recomputation every step — full BFS runs only during periodic resampling, and the per-step overestimate keeps the subduction uplift distance field approximately correct between resamplings.

### 2.5 Plate

Each plate stores:

| Field | Type | Description | Paper ref |
|---|---|---|---|
| `id` | int | Unique identifier | — |
| `rotation_axis` | Vector3 | Normalized axis through planet center | Section 4, p. 5: w |
| `angular_speed` | float | Radians per timestep (ω) | Section 4, p. 5: ω |
| `cumulative_rotation` | Quaternion | Accumulated rigid rotation since last resampling | Section 4 |
| `member_samples` | Array<int> | Canonical sample indices owned by this plate | Section 6 |
| `carried_samples` | Array | Copied crust attributes for member samples, mutated per-step | Section 4 |
| `interior_triangles` | Array<int> | Indices into global SDT (containment soup source) | Section 6 |
| `boundary_triangles` | Array<int> | Indices into global SDT (collision tracking) | Section 6 |
| `soup_bvh` | BVH | Built over rotated interior triangles | Section 6 |
| `bounding_cap` | (center, cos_angle) | Spherical bounding cap for broad-phase culling | Section 3.3 v4 |

### 2.6 Terranes

A terrane is a connected region of continental crust within a plate. Terranes resist subduction, participate in continental collision, and can detach from one plate to suture onto another.

**Paper reference:** Section 4, p. 5: "a terrane R in a plate P will be defined as a connected region of continental crust. Therefore, terranes can range in size from small isolated islands to entire continents."

Each terrane carries stable identity across resamplings:

| Field | Type | Description |
|---|---|---|
| `terrane_id` | int | Globally unique, stable across resamplings |
| `anchor_sample` | int | Representative canonical sample near centroid |
| `centroid` | Vector3 | Rolling average position |
| `area` | float | Approximate area (sample count × cell area) |
| `plate_id` | int | Current owning plate |

**Identity persistence rules** (after each resampling):

1. If a detected component contains a previous terrane's anchor → inherits that identity.
2. If anchor lost → match to component member nearest the terrane's last known centroid.
3. If no match → new terrane, new globally unique ID.
4. If a component contains anchors from multiple terranes → merger. Largest contributing terrane's ID preserved.

### 2.7 Forced-Subduction / Collision Tracking State

Continental collision tracking is stored at the `FTectonicPlanet` level, but it is **terrane-scoped**, not plate-pair-scoped and not per-sample state.

The baseline tracking key is `(TerraneId, OpposingPlateId)`. This allows one plate to carry multiple terranes colliding with different neighbors at different times.

| Field | Type | Description | Paper ref |
|---|---|---|---|
| `collision_tracking` | Map<(TerraneId, OpposingPlateId), FCollisionTracking> | Active forced-subduction tracks | Section 6, p. 9 |

Each `FCollisionTracking` entry:

| Field | Type | Description |
|---|---|---|
| `terrane_id` | int | Tracked terrane undergoing forced subduction |
| `source_plate_id` | int | Plate currently carrying the terrane |
| `opposing_plate_id` | int | Plate the terrane is converging into |
| `interpenetration_distance_km_exact` | float | Exact overlap depth measured at the most recent resampling |
| `interpenetration_distance_km_estimate` | float | Per-step advanced estimate used between resamplings |
| `convergence_rate_km_per_step` | float | Estimated penetration growth per step from relative convergence speed |
| `step_first_detected` | int | Timestep when continental-continental contact was first detected |
| `step_threshold_crossed_estimate` | float | Estimated timestep when `d_collision = 300 km` was crossed; `-1` if not yet crossed |

**Lifecycle:** Created when Phase 5 first classifies a terrane/opposing-plate contact as continental-continental forced subduction. Exact interpenetration is computed at each resampling. Between resamplings, the estimate is advanced per-step using the tracked convergence rate. Cleared when the collision event fires, when the terrane changes owner, or when the contact stops converging.

**Timing rule:** Collision events still execute during resampling because they require membership rebuild. However, the event parameters use the estimated threshold-crossing time, not merely the detection time. If first detection already finds `interpenetration_distance_km_exact > d_collision`, the crossing time is back-calculated within the current resampling window.

**Paper reference:** Section 6, p. 9: "If two continental triangles of two different plates intersect, we switch the tracking mode to continental collision. The collision event is triggered if the interpenetration distance between the two plates is greater than a user-defined threshold, 300 km in our system."

---

## 3. Simulation Loop

Each timestep represents δt = 2 My. The loop has two modes: a per-timestep update (every step) and a periodic resampling (every R steps).

**Paper reference:** Section 4, p. 5: "The iterative process is based on a discrete time-step δt set to 2 My."

### 3.1 Per-Timestep Update (Cheap, Every Step)

This is the inner loop. Most operations are per-sample on carried data; the only non-sample work is advancement of active forced-subduction tracks. No global repartitioning.

**Paper reference:** The paper applies subduction uplift (Section 4.1), fold direction updates (Section 4.1), slab pull (Section 4.1), continental erosion (Section 4.5), oceanic dampening (Section 4.5), and sediment accretion (Section 4.5) every timestep.

```
for each plate P:
    rotate P.cumulative_rotation by angular_speed around rotation_axis
    apply slab pull correction to rotation_axis

for each active collision track C:
    previous_d = C.interpenetration_distance_km_estimate
    C.interpenetration_distance_km_estimate += C.convergence_rate_km_per_step
    if C.step_threshold_crossed_estimate < 0 and
       C.interpenetration_distance_km_estimate >= d_collision:
        C.step_threshold_crossed_estimate =
            current_step - 1 +
            (d_collision - previous_d) / max(C.convergence_rate_km_per_step, epsilon)

for each plate P:
    for each carried sample S in P:
        advance subduction distance estimate:
            S.subduction_distance_km += ||s(S.position)|| * delta_t_km

        if S is in subduction influence zone:
            apply subduction uplift:
                ũ_j(p) = u₀ · f(d(p)) · g(v(p)) · h(z̃(p))
                S.elevation = min(S.elevation + ũ_j * delta_t, z_c)
            update fold direction:
                S.fold_direction += β · (s_i(p) - s_j(p)) · delta_t
                project onto tangent plane, renormalize

        apply continental erosion (if continental):
            S.elevation -= (S.elevation / z_c) · ε_c · delta_t

        apply oceanic dampening (if oceanic):
            S.elevation -= (1 - S.elevation / z_a) · ε_o · delta_t

        apply sediment accretion (if in trench):
            S.elevation += ε_f · delta_t

        age crust:
            S.age += delta_t
```

**Per-step distance-to-front overestimation:** Between resamplings, `subduction_distance_km` is advanced conservatively: `d(p, t+δt) ≥ d(p,t) + ‖s(p)‖δt`. This is an overestimate (the sample may have moved closer to the front, not farther), but it keeps the uplift distance field approximately correct without running BFS every step. Full BFS from subduction front samples runs during periodic resampling and resets the exact values.

**Paper reference:** Section 6, p. 9: "For each tracked converging triangle, we also update the distance of its vertices to the convergence front at every time step by overestimating."

**Per-step interpenetration advancement:** Active forced-subduction tracks use the same pattern. Once a terrane/opposing-plate contact has been detected, its interpenetration estimate advances every step between resamplings. This does not fire the collision event mid-cycle, but it records an estimated threshold-crossing step so the eventual resampling-time event can use the correct crossing moment instead of the later detection moment.

### 3.2 Periodic Resampling (Expensive, Every R Steps)

Global resampling runs when enough motion has accumulated. The paper describes this as "every 10–60 iterations, depending on the observed maximum plate speed."

**Paper reference:** Section 6, p. 9: "we compute the movement of the plates and perform a global resampling every 10–60 iterations."

**Trigger rule:** Resampling is triggered by a fixed step counter, not by displacement. The counter `R` is computed once at initialization and updated only when the plate count or maximum speed changes materially (e.g., after rifting).

```
inputs:
    S     = average_sample_spacing_km          (derived from N: S ≈ R_planet * sqrt(4π / N))
    D_max = max_plate_displacement_per_step_km (= ω_max * R_planet * δt, where ω_max is max angular speed)

rule:
    R = clamp(round(S / D_max * K), R_min, R_max)

constants:
    K     = 10       (target: R steps of motion ≈ 10 × sample spacings)
    R_min = 10       (hard floor: never resample more often than every 10 steps)
    R_max = 60       (hard ceiling: never go longer than 60 steps)
```

**Worked examples:**

| Resolution | N | S (km) | D_max (km) | R (raw) | R (clamped) | Motion between resamplings |
|---|---|---|---|---|---|---|
| 100 km | 60k | 102 | 200 | round(5.1) = 5 | 10 (R_min) | ~2000 km ≈ 20 spacings |
| 78 km | 100k | 78 | 200 | round(3.9) = 4 | 10 (R_min) | ~2000 km ≈ 26 spacings |
| 50 km | 250k | 50 | 200 | round(2.5) = 3 | 10 (R_min) | ~2000 km ≈ 40 spacings |
| 35 km | 500k | 35 | 200 | round(1.75) = 2 | 10 (R_min) | ~2000 km ≈ 57 spacings |
| 35 km | 500k | 35 | 100 | round(3.5) = 4 | 10 (R_min) | ~1000 km ≈ 29 spacings |

At all production resolutions, R = R_min = 10. The formula matters only if plate speeds are very slow or resolutions very high.

**Why R_min = 10 maps to the paper's window:** The paper says "every 10–60 iterations" at 500k (p. 9). At 500k with v₀ = 100 mm/yr and δt = 2 My, each step moves plates ~200 km. Over 10 steps, that's ~2000 km ≈ 57 sample spacings. The paper achieves good results with this much motion between resamplings because per-step erosion/dampening smooth artifacts continuously.

**v4 lesson:** v4 used displacement threshold = 0.5 × sample spacing ≈ 16 km, triggering resampling every timestep at 500k. This compounded boundary noise 10–60× faster than the paper's design. v5 enforces R_min = 10.

**Non-paper design choice:** The paper says "10–60 iterations" without specifying the exact trigger. We use a fixed step counter rather than displacement-based triggering, because the paper's frequency depends on maximum plate speed (a known quantity at any point in the simulation), not on accumulated displacement. The counter `R` and the inputs used to compute it are logged at every resampling for monitoring.

Resampling runs in the following order:

#### Phase 1 — Build Containment Soups and BVHs (All Triangles Partitioned)

Every canonical triangle is assigned to exactly one plate:

- **Interior triangle** (all 3 vertices same plate): assigned to that plate.
- **Boundary triangle** (vertices spanning 2+ plates): assigned to the plate that owns the majority of its vertices (2-of-3 or 3-of-3). Ties broken by lowest plate_id.

For each plate, build its containment soup:

1. Collect all triangles assigned to this plate (interior + boundary).
2. Rotate **all** vertices by the **owning plate's** cumulative rotation (uniform rigid rotation). For boundary triangles, this means foreign vertices (canonically owned by another plate) are "dragged along" with the owning plate. The triangle moves rigidly and never deforms — no stretching, no shearing, no degeneracy. This is correct: each plate's triangulation is a rigid body.
3. Build Cartesian AABB BVH over the rotated triangle soup.
4. Compute spherical bounding cap for broad-phase culling.

**Why uniform rotation, not per-vertex:** Per-vertex rotation (each vertex by its own plate's rotation) causes boundary triangles to stretch by the convergence/divergence distance between plates. After 10 steps at fast plate speeds, boundary triangles can stretch to cover thousands of km, creating degenerate containment geometry that scrambles ownership across the entire sphere. Uniform rotation keeps triangles rigid and well-formed. The "error" is that foreign vertices are in slightly wrong positions, but this only affects the very edge of containment geometry — exactly where imprecision is tolerable.

**Carried samples for foreign vertices:** At repartition time (Phase 6), each plate must store carried sample entries for ALL vertices in its triangulation — including foreign vertices from boundary triangles. Foreign vertex data is copied from the canonical sample state. During per-step updates (AdvanceStep), these foreign carried samples are rotated and updated alongside the plate's own members. The interpolation error from applying the wrong plate's rotation to foreign vertices is minor and confined to narrow boundary zones.

**Cost:** O(T) total across all plates. No retriangulation, no triangle splitting, no per-vertex rotation lookup.

#### Phase 2 — Ownership Assignment (Containment Query)

For each canonical sample position, determine which plate's soup contains it.

**Broad-phase:** Cull candidate plates via spherical bounding cap test (one dot product per plate).

**Containment query:** For each surviving candidate plate, query its BVH using Shewchuk-predicate point-in-triangle tests. This yields:

- The containing plate and triangle (if exactly one plate contains the sample)
- Barycentric coordinates within that triangle
- Whether multiple plates contain the sample (**overlap** — convergent zone)
- Whether no plate contains the sample (**gap** — divergent zone or boundary-triangle gap)

**Assignment rule:** Simple. The containing plate wins. If multiple plates contain the sample (overlap), assign to the plate whose containing triangle yields the highest minimum barycentric coordinate (most interior to its triangle). If no plate contains the sample, mark as gap.

**Boundary semantics with all triangles partitioned:**

Because every triangle is assigned to exactly one plate and moves rigidly with that plate, the containment geometry reflects plate dynamics cleanly:

| Case | Condition | Resolution |
|---|---|---|
| Interior hit | Exactly 1 plate's soup contains the sample | Assign to that plate |
| Overlap | 2+ plates' soups contain the sample | Assign to plate with highest min(bary) score |
| Divergent gap | No plate contains the sample; flanking plates are diverging | New oceanic crust (Phase 4) |
| Boundary-region gap | No plate contains the sample; flanking plates are NOT diverging (boundary triangles have shifted with their plates, leaving minor uncovered zones at transform/oblique boundaries) | Assign to plate that owns the majority of this sample's Delaunay neighbors (tie-break: lowest plate_id); interpolate from nearest carried sample of winning plate |
| Edge/vertex degeneracy | Sample lies exactly on a triangle edge or vertex (Shewchuk orient3d returns 0) | The sample is contained by any triangle incident to that edge/vertex. If multiple plates claim it, use the overlap rule |

**Critical invariant:** Boundary triangles are included in containment soups unsplit, assigned to their majority-vertex plate. All vertices are rotated uniformly by the owning plate's rotation (rigid body). No boundary triangle may ever be split with synthetic vertices, and no per-vertex rotation may be applied. Any code path that creates synthetic vertices, sub-triangles, or rotates different vertices of the same triangle by different plates is a v5 violation.

**No hysteresis, no contender support, no fast path in the baseline.** The paper does not describe these mechanisms. If ownership noise appears, diagnose whether it stems from cadence or containment geometry before adding heuristics.

**v4 lesson:** v4 added hysteresis, contender support guard, and fast path to compensate for boundary-triangle splitting overlaps and too-frequent resampling. With unsplit boundary triangles and infrequent resampling, these should be unnecessary.

#### Phase 3 — Attribute Interpolation

For each canonical sample that received a containing triangle in Phase 2, interpolate crust attributes from the containing triangle's three vertices using barycentric coordinates. All three vertices' carried sample data comes from the **containing plate's** carried sample array — no cross-plate lookup.

**Paper reference:** Section 6, p. 9: "The parameters of the other sample points are computed using barycentric interpolation of crust data from **the plate they intersect**." This is singular — one plate provides all the data. For boundary triangles, the containing plate stores carried samples for all its vertices, including foreign vertices whose canonical data was copied at repartition time (Phase 6).

**Continuous fields** (elevation, thickness, age, continental_weight): standard barycentric blend.

```
value = w0 * V0.value + w1 * V1.value + w2 * V2.value
```

`continental_weight` is interpolated as a continuous field, not voted on as a category. The derived `crust_type` (continental if `continental_weight >= 0.5`, oceanic otherwise) is never interpolated directly.

**Why continuous, not majority vote:** Majority vote on a binary field erodes under repeated resampling. A coastal triangle with 2 continental and 1 oceanic vertex always votes "continental" regardless of barycentric position, but the reverse triangle (1 continental, 2 oceanic) always votes "oceanic." Over many resamplings with slight geometric shifts, this creates a random walk at coastlines. Continuous interpolation + threshold is stable: the same barycentric position always produces the same result, and per-step erosion/dampening (Section 3.1) governs coastline evolution through elevation, not through resampling artifacts.

**Categorical fields** (orogeny_type): majority vote from three vertices.

**Direction fields** (fold_direction, ridge_direction): blend in 3D Cartesian, project onto tangent plane at sample position, renormalize.

**Paper reference:** Section 4, p. 5 (direction vectors are tangent to sphere). The paper uses barycentric interpolation for "crust data" (p. 9) — continuous `continental_weight` follows this framing directly.

```
blended = w0 * V0.direction + w1 * V1.direction + w2 * V2.direction
projected = blended - dot(blended, sample_normal) * sample_normal
result = normalize(projected)
if length(projected) < degeneracy_threshold:
    result = vertex with largest barycentric weight's direction
```

**Terrane identity** (terrane_id): select from vertex with largest barycentric weight. This is a per-event identity, not a continuous field.

#### Phase 4 — Gap Detection and Oceanic Crust Generation

Samples not contained by any plate's soup fall into two categories:

1. **Divergent gaps:** Samples between plates that are moving apart (boundary triangles have moved away with their owning plate, leaving uncovered divergent zones). These receive new oceanic crust.
2. **Boundary-region gaps:** Samples near transform or oblique boundaries where rigid plate motion has shifted boundary triangles away, leaving minor uncovered zones. These are not true divergent zones.

**Paper reference:** Section 4.3, pp. 7–8: oceanic crust generation at diverging plate boundaries. Section 6, p. 9: "The parameters of the samples located between diverging plates are computed using the method described in Section 4.3."

**Divergent gap resolution:**

For gap samples flanked by two plates whose relative velocity has a positive boundary-normal component (diverging):

```
α = d_Γ(p) / (d_Γ(p) + d_P(p))
z(p) = α · z̄(p) + (1 - α) · z_Γ(p)
```

Where `d_Γ` is distance to the ridge, `d_P` is distance to the nearest plate border, `z̄` is interpolated border elevation, and `z_Γ` is the template ridge profile.

- `continental_weight` = 0.0 (oceanic)
- `age` = 0
- `ridge_direction` = `(p - q) × p` where q is projection of p onto ridge line
- `plate_id` = assigned to nearer flanking plate

**Boundary-region gap fallback:**

Gap samples where flanking plates are not diverging are boundary-region artifacts (boundary triangles have shifted with their owning plate, leaving minor uncovered zones at transform/oblique boundaries). These are resolved by assigning to the plate that owns the majority of the sample's Delaunay neighbors. Tie-break: lowest plate_id. Their crust attributes are interpolated from the nearest carried sample of the winning plate.

This is the same rule as the boundary-semantics table in Phase 2 (Section 3.2). There is exactly one deterministic fallback for boundary-region gaps, and it is always Delaunay-neighbor majority with lowest-plate_id tie-break.

**Non-paper design choice:** The paper does not explicitly describe this fallback. With all triangles partitioned and uniform rigid rotation, these gaps are small (boundary triangles cover most of the sphere; gaps appear only where rigid motion shifts boundary coverage away from the original boundary zone). The heuristic is purely topological: no distance computations, deterministic.

#### Phase 5 — Overlap Detection and Convergence Classification

Samples contained by multiple plates' soups are in convergent zones (natural overlap where rigid plate motion causes neighboring plates' boundary triangles to cover the same region).

**Paper reference:** Section 4, p. 5: "subduction (Section 4.1) and continental collision (Section 4.2) which both take place at convergent boundaries."

For each overlapping sample, classify the interaction:

| Overlap type | Condition | Result |
|---|---|---|
| Oceanic-oceanic | Both plates oceanic at sample | Subduction (older plate subducts) |
| Oceanic-continental | One oceanic, one continental | Subduction (oceanic subducts) |
| Continental-continental | Both continental | Forced subduction; begin or refresh `(TerraneId, OpposingPlateId)` tracking (see Phase 9) |

**Convergence verification:** Overlap alone is not sufficient. Verify that the plates' relative velocity has a negative boundary-normal component: `dot(v_rel, n_boundary) < 0`.

**Boundary normal computation:** Mean direction to all cross-plate Delaunay neighbors (1-ring), projected onto tangent plane, normalized.

**Paper reference:** Section 4.1, p. 6 (subduction trigger); Section 4.2, p. 7 (collision trigger). Section 6, p. 9: "If two continental triangles of two different plates intersect, we switch the tracking mode to continental collision."

#### Phase 6 — Repartition Membership

Rebuild each plate's member sample list and carried sample workspace from the new ownership assignments.

**This phase is simple in v5.** It does three things:

1. Rebuild `plate.member_samples` from canonical samples carrying each `plate_id`.
2. Rebuild `plate.carried_samples` by copying current crust attributes from canonical samples.
3. Recompute `is_boundary` flags from Delaunay neighbor graph (any neighbor with different `plate_id`).

**No sanitization. No connectivity enforcement. No protected plate rescue.** If ownership produces fragmented plates, the cause is upstream (containment or cadence), and the fix is upstream.

**v4 lesson:** v4's Phase 6 grew to 10 sub-stages compensating for boundary-triangle splitting noise. With unsplit boundary triangles (per-vertex rotation) and infrequent resampling, a simple repartition should suffice. If it doesn't, that's a signal to investigate containment geometry, not to add repair passes.

#### Phase 7 — Terrane Detection

Run connected component analysis over continental samples within each plate using the Delaunay neighbor graph. Match detected components to tracked terranes via identity persistence rules (Section 2.6).

**Paper reference:** Section 4, p. 5: "a terrane R in a plate P will be defined as a connected region of continental crust."

Update anchor samples, centroids, and areas. Detect mergers, splits, and new terranes.

#### Phase 8 — Subduction Field Propagation

Compute exact BFS distance-to-front for subduction boundaries. This resets the per-step overestimates to precise values.

**Paper reference:** Section 4.1, p. 6: distance-to-front for uplift computation.

For each convergent boundary sample (from Phase 5 overlap classification), seed BFS outward through the overriding plate's Delaunay graph, accumulating geodesic arc lengths along edges. This produces smooth distance values up to `r_s = 1800 km`.

Assign subduction roles:
- Subducting plate samples near front: mark as trench (elevation driven toward `z_t`)
- Overriding plate samples within influence: receive uplift profile in per-step loop

Store distance and convergence speed on carried samples for the per-step uplift computation (Section 3.1).

#### Phase 9 — Continental Collision Tracking and Events

Update forced-subduction tracking state and fire collision events for tracked terrane/opposing-plate contacts that exceed the interpenetration threshold.

**Paper reference:** Section 4.2, pp. 7–8; Section 6, p. 9.

**Step 1 — Update tracking state:**

For each `(TerraneId, OpposingPlateId)` contact flagged as continental-continental forced subduction in Phase 5:
- If not already tracked: create a new `FCollisionTracking` entry (Section 2.7).
- Compute exact interpenetration distance from the overlap region geometry and store it in `interpenetration_distance_km_exact`.
- Update `interpenetration_distance_km_estimate` to at least the exact value from this resampling.
- Update `convergence_rate_km_per_step` from the local relative convergence speed.
- If the terrane changes owner, loses identity, or the contact is no longer converging: clear the tracking entry.

**Step 2 — Fire collision events:**

For each tracked entry where either the per-step estimate or the exact resampling measurement reaches `d_collision` (300 km):

1. **Determine threshold-crossing time `t_cross`:**
   - If `step_threshold_crossed_estimate >= 0`, use that stored value.
   - Else if first detection already finds `interpenetration_distance_km_exact > d_collision`, back-calculate
     `t_cross ≈ t_now - (D_exact - d_collision) / max(v_rel, epsilon)`,
     clamped to the current resampling interval.
   - Else use `t_now` (threshold reached on this resampling).

2. **Evaluate collision parameters at `t_cross`, not at raw detection time.**
   - Influence radius uses convergence speed at `t_cross`.
   - Surge timing, logs, and diagnostics use `t_cross`.

3. **Slab break:** Terrane R detaches from plate P_i. Its samples switch `plate_id` to plate P_j.
4. **Elevation surge:** `Δz(p) = Δ_c · A · f(d(p, R))` where `f(x) = (1 - (x/r)²)²` for `x < r`, else 0. The elevation of p is **instantly** augmented: `z(p) = min(z(p) + Δz(p), z_c)`.
5. **Influence radius:** `r = r_c · √(v(q)/v₀ · A/A₀)`, using the back-calculated crossing speed.
6. **Fold direction:** Set perpendicular to collision front: `f(p) = (n × (p-q)/‖p-q‖) × n` where n is surface normal.
7. **Orogeny type:** Himalayan. Age reset to 0.
8. **Clear tracking:** Remove the `FCollisionTracking` entry for this terrane/opposing-plate contact.
9. **Refresh:** Rebuild membership and carried samples for affected plates. Re-detect terranes. Re-check for new collisions created by topology change.

The sequential loop with per-event refresh is necessary because each collision changes plate boundaries.

**Timing caveat and mitigation:** With `R = 10`, first continental contact can be discovered up to `R` steps late. The combination of per-step interpenetration advancement and resampling-time back-calculation keeps the 300 km threshold meaningful even though the slab-break event still executes during resampling.

**No per-step Himalayan uplift.** The paper treats collision as a single instantaneous surge (Section 4.2, p. 8: "the elevation of p is instantly augmented"). Mountain persistence comes from: (a) the surge magnitude, (b) slow continental erosion decay (Section 3.1), and (c) further collisions from continued convergence. v4 added continuous post-collision uplift to compensate for every-step resampling washing out the surge; with v5's R=10, this is unnecessary.

#### Phase 10 — Plate Rifting

Poisson-triggered plate fracturing. Rifting is a discrete event processed during resampling.

**Paper reference:** Section 4.4, p. 8: "Plate rifting is a discrete event which fragments a plate into smaller plates."

**Trigger:** `P = λ · e^(-λ)` where `λ = λ₀ · f(x_P) · A/A₀`.

**Process:**

1. Select plate P. Fracture into n sub-plates (n ∈ [2, 4]).
2. Distribute n random centroids within P's domain.
3. Compute spherical Voronoi cells. Warp boundaries with coherent noise.
4. Reassign samples to sub-plates by Voronoi cell.
5. Assign random diverging geodetic movements.
6. Sub-plates inherit terrane fragments based on received samples.

**Paper reference:** Section 4.4, p. 8: "More irregular continent shapes can be obtained by warping the geodesic distances to the centroids using a simple noise function" — applies to rifting fracture boundaries too.

After rifting: rebuild soups, BVHs, membership for all affected plates. New plates start accumulating motion for the next resampling cycle.

#### After All Phases — Finalize

- Reset cumulative rotation tracking for the next resampling interval.
- Reset timestep-since-resampling counter.
- Log diagnostics: continental area fraction, overlap count, gap count, plate count, component count per plate, timing per phase.
- Export checkpoint maps if configured.

---

## 4. Initialization

### 4.1 Plate Seeding

Distribute plate centroids via farthest-point sampling on the sphere (same approach as v4). This produces well-separated initial plates.

**Paper reference:** Section 4, p. 5: "distributing a user-prescribed number of plate centroids c_i over the sphere and defining the plates as the spherical Voronoi cells of the set {c_i}."

### 4.2 Boundary Warping

Warp Voronoi cell boundaries with coherent noise during plate assignment.

**Paper reference:** Section 4, p. 5: "More irregular continent shapes can be obtained by warping the geodetic distances to the centroids using a simple noise function."

For each sample, compute warped distance to each centroid: `d_warped(p, c_i) = d_geodesic(p, c_i) · (1 + amplitude · noise(p))` where `noise` is 3D simplex noise evaluated at the sample position. Assign sample to the plate with minimum warped distance.

**Parameters:** `amplitude` ∈ [0.1, 0.3] (start with 0.2). Higher values produce more irregular boundaries.

### 4.3 Per-Sample Continental Assignment

Continental crust is assigned per-sample, not per-plate. A fraction of plates are designated as "continental-seeded." For each continental-seeded plate, samples within a noise-warped radius of the plate centroid are marked continental; the rest remain oceanic.

**Paper reference:** Section 4, p. 5: crust type x_C is a per-sample function, and terranes are "connected regions of continental crust" within plates. The paper's Figure 4 shows a plate with both continental and oceanic regions.

**Process:**

1. Select a subset of plates as continental-seeded (targeting a configured continental area fraction, typically 0.25–0.35).
2. For each continental-seeded plate, define a continental radius: `r_continent = sqrt(A_target / π)` where `A_target` is the target continental area for this plate.
3. For each sample in the plate, compute noise-warped distance to plate centroid. If `d_warped < r_continent`, set `continental_weight = 1.0`; otherwise `continental_weight = 0.0`.
4. Samples with `continental_weight >= 0.5` receive initial elevation drawn from a gentle distribution centered around +0.5 km (low continental shelf).
5. Samples with `continental_weight < 0.5` receive initial elevation from the abyssal plain profile: `z_a = -6 km` plus small noise.

This produces irregular continental shapes within plates from the start, with natural coastlines and mixed oceanic-continental plates.

**v4 lesson:** v4 marked entire plates as continental, making CrustType maps indistinguishable from PlateId maps. Per-sample assignment is essential for visual plausibility even at step 0.

### 4.4 Initial Plate Motion

Assign random geodetic movements to each plate. Each plate gets a random rotation axis (uniformly distributed on the sphere) and a random angular speed (uniformly distributed in `[0.3 · ω_max, ω_max]`).

**Paper reference:** Section 4, p. 5: "We then initialize the movement of the plates G_i, which can be also defined by the user or randomly generated."

### 4.5 Initial Elevation Profile

| Crust type | Initial elevation | Notes |
|---|---|---|
| Continental interior | +0.5 km ± noise | Low continental shelf |
| Continental margin | +0.2 km ± noise | Transitional |
| Oceanic (young) | z_a = -6 km ± noise | Abyssal plain |
| Oceanic (old) | z_a = -6 km ± noise | Same; aging applies per-step |

Low-frequency coherent noise (amplitude ~0.3 km, wavelength ~1000 km) is applied globally to break up uniform patterns.

**Paper reference:** Section 4.5, p. 8: "we apply low frequency coherent noise to add more realistic variations over the planet."

---

## 5. Tectonic Processes

### 5.1 Subduction

**Trigger:** Convergent overlap (Phase 5) where an oceanic plate meets any other plate.

**Paper reference:** Section 4.1, pp. 5–7.

**Continuous uplift** (applied per-timestep on carried data):

```
ũ_j(p) = u₀ · f(d(p)) · g(v(p)) · h(z̃(p))
```

**Transfer functions:**

- `f(d)` — Piecewise cubic distance profile. Maximum at a control distance from the subduction front, fading to zero at `r_s = 1800 km`.
- `g(v)` — Linear speed scaling: `g(v) = v / v₀`.
- `h(z̃)` — Quadratic elevation weighting: `h(z̃) = z̃²` where `z̃ = (z - z_t) / (z_c - z_t)`. Elevations above sea level have high impact; abyssal depths have minimal impact.

**Elevation update:**

```
z(p, t+δt) = min(z(p,t) + ũ_j(p) · δt, z_c)
```

**Fold direction update:**

```
f(p, t+δt) = f(p,t) + β · (s_i(p) - s_j(p)) · δt
```

Followed by tangent-plane projection and renormalization.

**Slab pull:**

```
w_i(t+δt) = w_i(t) + ε · Σ (c_i × q_k) / |c_i × q_k| · δt
```

Where `q_k` are subduction front samples and `ε ≪ 1`.

**Distance-to-front lifecycle:**

The subduction distance field has a three-phase lifecycle that must be understood as a unit:

1. **Seeding (during resampling Phase 5):** Convergent overlap samples are identified. Samples on the overriding plate side of the convergence are tagged as subduction front samples with `subduction_distance_km = 0`.

2. **Exact computation (during resampling Phase 8):** BFS propagation from front samples outward through the overriding plate's Delaunay graph, accumulating geodesic arc lengths along edges. This produces exact distance values up to `r_s = 1800 km`. Samples beyond `r_s` are marked outside subduction influence. The BFS also records the convergence speed at the nearest front for each sample.

3. **Per-step overestimation (between resamplings, Section 3.1):** Each timestep, every sample with a valid subduction distance advances its estimate: `d(p, t+δt) = d(p,t) + ‖s(p)‖ · δt_km`. This is an overestimate — the sample may have moved closer to the front, not farther. The overestimate ensures uplift influence never drops to zero prematurely between resamplings. It may extend uplift slightly too far, which is acceptable. The exact BFS at the next resampling resets all values.

**Transition rule:** When a new resampling runs, all per-step overestimates are discarded and replaced by fresh BFS values. There is no blending or smoothing between the old overestimates and the new exact values.

**Orogeny type:** Andean. Age set to 0 at initiation.

### 5.2 Continental Collision

Continental collision follows a two-phase progression: forced subduction, then a discrete collision event.

**Paper reference:** Section 4.2, pp. 7–8. Section 6, p. 9: "If two continental triangles of two different plates intersect, we switch the tracking mode to continental collision. The collision event is triggered if the interpenetration distance between the two plates is greater than a user-defined threshold, 300 km in our system."

#### Phase A — Forced Subduction (Continuous)

When Phase 5 classifies a convergent overlap as continental-continental, subduction does not stop. The system switches to forced-subduction tracking mode for the `(TerraneId, OpposingPlateId)` contact: forced subduction continues (the continental crust resists subduction but the plates keep converging), and the interpenetration distance between the terrane and the opposing plate is tracked per-step.

**Paper reference:** Section 4, p. 6: "we allow partial subduction when we find a continental-continental configuration; this initiates a forced subduction, which then evolves into a continental collision where terranes collide."

Interpenetration distance is the maximum overlap depth between the two continental masses, measured as the furthest extent of the terrane into the opposing plate's interior. This is computed exactly at each resampling from the overlap region geometry. Between resamplings, it is advanced as an estimate using the tracked convergence rate.

If first detection already finds `d_pen > d_collision`, the collision does not wait for another cycle. The event fires on that resampling, but the threshold-crossing time is back-calculated:

`t_cross ≈ t_now - (d_pen - d_collision) / max(v_rel, epsilon)`

clamped to the current resampling interval.

#### Phase B — Collision Event (Discrete)

**Trigger:** Interpenetration distance exceeds threshold `d_collision = 300 km`.

This is a **discrete, instantaneous event**:

1. **Slab break:** Terrane R detaches from plate P_i. Its samples switch `plate_id` to plate P_j.
2. **Elevation surge:** `Δz(p) = Δ_c · A · f(d(p, R))` where `f(x) = (1 - (x/r)²)²` for `x < r`, else 0.
3. **Influence radius:** `r = r_c · √(v(q)/v₀ · A/A₀)`.
4. **Fold direction:** Set perpendicular to collision front: `f(p) = (n × (p-q)/‖p-q‖) × n` where n is surface normal.
5. **Orogeny type:** Himalayan, age = 0.
6. **Elevation ceiling:** `z(p) = min(z(p) + Δz(p), z_c)`.
7. **Refresh:** Rebuild membership and carried samples for affected plates. Re-detect terranes. Clear collision tracking state for this terrane/opposing-plate contact.

**Paper reference:** Section 4.2, p. 8: "the elevation of p is instantly augmented: z(p, t+δt) = z(p, t) + Δz(p)."

There is no per-step Himalayan uplift. The paper treats collision as a single instantaneous surge. Mountain building occurs because: (a) the surge is large, (b) continental erosion (Section 3.1) decays mountains slowly, and (c) continued convergence may trigger further collisions with other terranes.

**v4 lesson:** v4 added per-step Himalayan uplift because the discrete surge was washed out by every-step resampling. With v5's infrequent resampling (R=10), the surge persists for 10+ steps before being re-interpolated. If mountains are still too low, the fix is tuning `Δ_c` and `r_c`, not adding continuous uplift.

### 5.3 Oceanic Crust Generation

**Trigger:** Gap samples at divergent boundaries (Phase 4).

**Paper reference:** Section 4.3, pp. 7–8.

**Elevation profile:**

```
α = d_Γ(p) / (d_Γ(p) + d_P(p))
z(p) = α · z̄(p) + (1 - α) · z_Γ(p)
```

**Ridge direction:** `r(p) = (p - q) × p` where q is projection onto ridge line.

**Plate assignment:** Nearer flanking plate.

### 5.4 Plate Rifting

**Trigger:** Poisson process with continental area scaling.

**Paper reference:** Section 4.4, p. 8.

```
P = λ · e^(-λ)    where λ = λ₀ · f(x_P) · A / A₀
```

Fracture into n sub-plates (n ∈ [2, 4]) via spherical Voronoi with noise-warped boundaries. Assign random diverging movements. Sub-plates inherit terrane fragments.

### 5.5 Continental Erosion and Oceanic Dampening

**Per-timestep processes.** These are the paper's natural smoothing mechanisms.

**Paper reference:** Section 4.5, pp. 8–9.

**Continental erosion:**

```
z(p, t+δt) = z(p,t) - (z(p,t) / z_c) · ε_c · δt
```

Higher elevations erode faster. This naturally smooths resampling artifacts on continental surfaces.

**Oceanic dampening:**

```
z(p, t+δt) = z(p,t) - (1 - z(p,t) / z_a) · ε_o · δt
```

Pulls oceanic elevation toward abyssal plain depth `z_a = -6 km`.

**Sediment accretion:**

```
z(p, t+δt) = z(p,t) + ε_f · δt
```

Fills trenches gradually with sediment.

**Low-frequency noise:** Applied globally to break up uniform patterns.

---

## 6. Boundary Detection

Boundaries emerge from the Delaunay neighbor graph. They are never stored as explicit geometry.

**Paper reference:** Section 6, p. 9.

**Definition:** A sample is a boundary sample if any Delaunay neighbor has a different `plate_id`. The `is_boundary` flag is recomputed after each resampling Phase 6.

**Boundary classification** from relative velocity:

- **Convergent:** `dot(v_rel, n_boundary) < 0`
- **Divergent:** `dot(v_rel, n_boundary) > 0`
- **Transform:** Relative velocity predominantly tangential

**Boundary normal:** Mean direction to all cross-plate Delaunay neighbors (1-ring), projected onto tangent plane, normalized.

**BVH acceleration:** Each plate's Cartesian AABB BVH (built over interior triangles in Phase 1) accelerates containment queries. Broad-phase culling via spherical bounding caps further reduces cost.

---

## 7. Rendering

### 7.1 Mesh Strategy

The global SDT serves directly as the Procedural Mesh Component. Built once, never rebuilt.

Each step, update vertex attributes only:
- Position (radial displacement based on elevation)
- Normal (recomputed from neighbor elevations)
- Custom data (crust type, plate ID, age, terrane ID)

### 7.2 Debug Visualization

Material-switchable display modes:
- Elevation (color ramp)
- Plate ID (unique color per plate)
- Crust type (oceanic vs continental)
- Crust age
- Orogeny type
- Terrane ID
- Boundary overlay
- Distance-to-front

### 7.3 Amplification (Future — Not Part of v5 Baseline)

Amplification (Cortial et al. 2020) transforms coarse crust C into detailed terrain T. This is a separate stage that consumes finalized crust data. It is not implemented in the v5 baseline and should not be confused with crust simulation quality.

**The v5 acceptance criterion is that raw crust C is visually coherent** — recognizable continents, ocean basins, mountain ranges, smooth plate boundaries. Not that it looks like the paper's amplified renders.

---

## 8. Map Exporters

Exporters render the canonical sample state to 2D projections (equirectangular, Mollweide, Winkel Tripel).

**Exporters must faithfully represent the simulation state.** They must not apply nearest-sample fills, ownership smoothing, or fallback interpolation that masks defects. If an exporter produces artifacts, the cause should be diagnosed in the simulation, not patched in the exporter.

**v4 lesson:** v4 exporters applied nearest-triangle lookup with fallback weights and nearest-sample fills for unresolved pixels. These masked simulation problems, making it harder to distinguish exporter artifacts from genuine crust defects.

**Acceptable exporter behavior:**
- Supersampled anti-aliasing for smooth edges (rendering quality, not data repair)
- Correct map projection math
- Color ramp selection for different fields

**Unacceptable exporter behavior:**
- Filling unresolved pixels with neighbor votes
- Smoothing ownership across boundaries
- Applying any transform that changes the apparent crust state

---

## 9. Key Parameters

From the paper's Appendix A (Table 3):

| Parameter | Symbol | Value | Description |
|---|---|---|---|
| Timestep | δt | 2 My | Simulation timestep |
| Planet radius | R | 6370 km | |
| Oceanic ridge elevation | z_r | -1 km | Highest oceanic ridge |
| Abyssal plain elevation | z_a | -6 km | Default ocean floor |
| Oceanic trench elevation | z_t | -10 km | Deepest trench |
| Max continental altitude | z_c | 10 km | Elevation ceiling clamp |
| Subduction influence distance | r_s | 1800 km | Max distance for uplift |
| Collision influence distance | r_c | 4200 km | Max collision uplift distance |
| Collision coefficient | Δ_c | 1.3 × 10⁻⁵ km⁻¹ | Discrete collision surge |
| Max plate speed | v₀ | 100 mm/year | |
| Oceanic dampening | ε_o | 4 × 10⁻² mm/year | Per-timestep |
| Continental erosion | ε_c | 3 × 10⁻² mm/year | Per-timestep |
| Sediment accretion | ε_f | 3 × 10⁻¹ mm/year | Per-timestep |
| Subduction uplift | u₀ | 6 × 10⁻¹ mm/year | Base uplift rate |
| Resampling interval min | R_min | 10 | Non-paper: minimum steps between resamplings |
| Resampling interval max | R_max | 60 | Non-paper: maximum steps between resamplings |

### 9.1 Performance Budgets (Paper Table 2)

Average time-step execution time in seconds for a planet with 40 plates and continental coverage 0.3. Planet radius 6370 km. These are the **paper's measured budgets** and serve as our baseline targets.

| Phase | 60k (102 km) | 100k (78 km) | 250k (50 km) | 500k (35 km) |
|---|---|---|---|---|
| Subduction | 0.08 | 0.14 | 0.06 | 0.65 |
| Continental collision | 0.02 | 0.04 | 0.09 | 0.62 |
| Elevation | 0.09 | 0.10 | 0.29 | 0.62 |
| **Per-step total** | **0.19** | **0.28** | **1.24** | **1.90** |
| Oceanic crust generation | 0.58 | 1.22 | 3.58 | 13.10 |
| Plate rifting | 0.23 | 1.50 | 1.50 | 7.70 |

**Notes from the paper:**
- Oceanic crust generation and plate rifting are the most expensive processes; the others run at interactive rates for resolutions ≈ 100 km.
- Rifting and oceanic crust generation occur every 20–120 My depending on plate speed (i.e., every R=10–60 steps), so their cost is amortized.
- The paper computes crust on CPU in parallel across threads; GPU is used only for amplification (M7+).
- All per-milestone benchmarks must report phase-by-phase timing at 500k/40 plates (Stress40 scenario) and compare against this table.

---

## 10. Threading and Determinism

- All resampling writes target a new buffer. No in-place mutation during resampling.
- Stable tie-breaking: when distances are equal, prefer lower plate_id, then lower sample index.
- Per-sample crust updates (erosion, dampening, uplift) are embarrassingly parallel.
- Plate motion is per-plate with no inter-plate dependencies.
- Compile with `/fp:strict` (MSVC) for deterministic floating-point.
- Double-buffering: one buffer written by simulation, one read by rendering. Atomic swap on completion.

---

## 11. Milestone Plan

### M0 — Substrate and Static Initialization

**Goal:** Fibonacci sphere, global SDT, plate seeding with boundary warping, per-sample continental assignment, debug visualization.

**Validates:** Sampling quality, SDT correctness, noise-warped Voronoi boundaries, per-sample crust type producing organic continent shapes distinct from plate boundaries.

**Acceptance:** Step 0 PlateId and CrustType maps show irregular boundaries and continents that don't match plate shapes.

### M1 — Per-Timestep Loop (No Resampling)

**Goal:** Plate motion, continental erosion, oceanic dampening, sediment accretion, per-step distance-to-front advancement. No resampling yet — just verify that carried data evolves correctly.

**Validates:** Per-step processes produce smooth, stable elevation changes. Continental erosion lowers peaks. Oceanic dampening pulls ocean floor toward equilibrium. No ownership changes yet.

**Acceptance:**
- Run 50 steps without resampling. Elevation map shows gradual continental erosion and oceanic dampening. No discontinuities.
- Per-step timing at 500k/40 plates (Stress40) reported and compared against Section 9.1 "Elevation" row (0.62s budget at 500k). M1 has no subduction or collision, so only the elevation/erosion budget applies.

### M2 — Periodic Resampling Baseline

**Goal:** Interior-only containment soups, BVH containment queries, barycentric interpolation, gap detection, simple repartition. No subduction, collision, or rifting events yet.

**Validates:** Core resampling loop at paper cadence. Plate boundaries remain smooth after 10+ resamplings. Divergent gaps correctly generate oceanic crust. No ownership repair needed.

**Acceptance:**
- Run 100+ steps (10+ resamplings). PlateId map shows smooth, slowly-evolving boundaries. CrustType shows stable continents. Overlap and gap rates stay low and bounded. No fragmented plates.
- Resampling (oceanic crust generation) timing at Stress40 reported against Section 9.1 budget (13.10s at 500k).

### M3 — Subduction

**Goal:** Convergent boundary detection, subduction role assignment, BFS distance-to-front (exact at resampling, overestimated per-step), subduction uplift, fold direction, slab pull.

**Validates:** Mountain ranges appear parallel to convergent boundaries. Oceanic trenches form. Distance-to-front field is smooth. Uplift profile matches paper's piecewise cubic.

**Acceptance:**
- Subduction fronts produce Andean-type mountain chains. Elevation map shows recognizable orogeny along convergent boundaries.
- Per-step total (subduction + elevation) at Stress40 reported against Section 9.1 combined budget (0.65 + 0.62 = 1.27s at 500k).

### M4 — Continental Collision

**Goal:** Terrane detection with identity persistence, terrane-level forced-subduction tracking, threshold-crossing timing correction, slab break, instantaneous elevation surge.

**Implementation note (March 2026):** The current stable implementation uses `PreserveOwnershipPeriodic` for periodic maintenance and executes preserve-mode collision follow-up from cached boundary-contact evidence. See [Preserve Ownership Stable Point](./preserve-ownership-stable-point.md) for the current status and caveats.

**Validates:** Collision follows the paper's two-phase progression (forced subduction → discrete event at 300 km threshold). Threshold crossing remains meaningful despite `R=10` because the crossing step is estimated between resamplings and back-calculated on first late detection. Terranes transfer between plates. Elevation surge persists across resamplings without continuous uplift.

**Acceptance:**
- Continental collision produces localized high-elevation regions. Terrane identity persists across resamplings. Logged collision backdating remains bounded by `R` steps.
- Full per-step total (subduction + collision + elevation) at Stress40 reported against Section 9.1 budget (1.90s at 500k).

### M5 — Plate Rifting

**Goal:** Poisson-triggered rifting, spherical Voronoi sub-plate generation with noise-warped boundaries, diverging motion assignment, terrane inheritance.

**Validates:** Supercontinent breakup produces new ocean basins. Sub-plates inherit terrane fragments correctly.

**Acceptance:**
- Rifting creates new divergent boundaries that generate oceanic crust in subsequent resamplings.
- Rifting timing at Stress40 reported against Section 9.1 budget (7.70s at 500k).

### M6 — Full Simulation Tuning

**Goal:** All processes running with paper parameters. Long-duration validation.

**Validates:** Complete tectonic cycle: rift → drift → subduct → collide → rift. 500+ My runs remain stable and visually plausible.

**Acceptance:**
- Continental area stays within configured band (20–40%) over 500 My.
- Tectonic cycle observable: supercontinent assembly and breakup.
- Raw crust C shows recognizable continents, mountain ranges, ocean basins.
- Comparison against paper Figure 12 (left panel) shows comparable coarse-crust quality.
- All per-step and per-resampling phase timings at Stress40 meet or beat Section 9.1 budgets. Any remaining deviations must be profiled, explained, and documented.

### M7 — Amplification

**Goal:** Cortial et al. 2020 hyper-amplification. CPU preprocessing (river networks, Horton-Strahler), GPU hierarchical subdivision, oceanic noise. Separate from tectonic simulation.

### M8 — Interactive Authoring

**Goal:** Pause/resume, prescribe plate movements, trigger rifting, switch crust types.

---

## 12. Acceptance Tests and Invariants

### 12.1 Per-Resampling Invariants

These are checked automatically after every resampling. Violation is a bug, not a tuning issue.

1. **Every sample has a valid plate_id** (0 ≤ plate_id < plate_count).
2. **Every plate has at least 1 member sample** (dead plates are removed from the active set).
3. **Overlap rate < 5% of total samples** (with interior-only soups and paper cadence, overlaps should be rare).
4. **Gap rate < 10% of total samples** (most gaps become oceanic crust or boundary-triangle fallback).
5. **No sample changes plate_id outside of resampling** (per-step loop does not modify ownership).

### 12.2 Multi-Step Health Checks

Logged every resampling, alarmed if thresholds are exceeded for 3 consecutive resamplings.

1. **Continental area fraction** stays in [0.15, 0.50] unless a collision/rifting event explains the shift.
2. **No plate has more than 1 connected component** of non-gap samples for 3 consecutive resamplings without a rifting event explaining it.
3. **Boundary smoothness:** ratio of boundary samples to total samples stays below 15% (jagged boundaries inflate this ratio).
4. **Elevation range:** at least 5% of samples have elevation > 0 (continents exist) and at least 40% have elevation < z_a/2 (deep ocean exists).

### 12.3 Validation Scenarios

Run these before declaring a milestone complete:

| Scenario | Samples | Plates | Steps | Resamplings | Purpose |
|---|---|---|---|---|---|
| PaperCore | 500k | 7 | 100 | ~10 | Paper-like baseline, no events |
| Smoke7 | 500k | 7 | 100 | ~10 | Full events, low plate count |
| Stress40 | 500k | 40 | 50 | ~5 | High plate count stress test |
| LongRun | 500k | 15 | 500 | ~50 | Tectonic cycle stability (500 My) |

**Performance gate:** The Stress40 scenario (500k samples, 40 plates) is the performance reference configuration matching the paper's Table 2. Per-phase timing at this scale must be reported for every milestone and compared against Section 9.1 budgets. A milestone may ship with timing above budget if the deviation is understood and documented, but the goal is to meet or beat the paper's numbers.

**Minimum resolution:** No validation scenario or automation test may use fewer than 60k samples (102 km resolution). This is the paper's lowest measured resolution. Results below this threshold are not representative of the algorithm's behavior and must not be used for correctness or performance validation. The 60k configuration serves as the fast-iteration tier; 500k (Stress40) serves as the performance tier.

### 12.4 Sample-Derived Metrics as Source of Truth

All health metrics are computed from canonical sample state, not from exported images. Exported maps are a visualization of the canonical state, not a replacement for it.

**Required per-resampling log output** (computed from canonical samples, not from pixel counts):

```
[Resample R=N Step=T]
  continental_area_fraction = <float>    (count of continental_weight >= 0.5 / N)
  overlap_count             = <int>      (samples contained by 2+ plates)
  gap_count                 = <int>      (samples contained by 0 plates)
  divergent_gap_count       = <int>      (gaps resolved as new oceanic crust)
  numerical_gap_count       = <int>      (gaps resolved by fallback)
  plate_count               = <int>      (active plates)
  max_components_per_plate  = <int>      (connected component analysis on non-gap samples)
  boundary_sample_fraction  = <float>    (is_boundary==true / N)
  elevation_min             = <float>
  elevation_max             = <float>
  elevation_mean            = <float>
  subduction_front_count    = <int>      (front samples with d=0)
  active_collision_tracks   = <int>      (tracked `(TerraneId, OpposingPlateId)` contacts)
  collision_event_count     = <int>      (collisions triggered this resampling)
  collision_backdated_count = <int>      (collisions whose threshold crossing predated detection)
  max_collision_backdate_steps = <float> (largest `t_now - t_cross` this resampling)
  resampling_interval_R     = <int>      (steps since last resampling)
  phase_timing_ms           = <per-phase breakdown>
```

**Cross-check rule:** When map exports are produced alongside metrics, the exporter must log its own derived statistics (e.g., pixel-counted continental fraction) and these must be compared against the sample-derived values. A discrepancy > 5% between sample-derived and export-derived continental area fraction is an exporter bug, not a simulation feature.

### 12.5 Same-Run Checkpoint Requirements

**All checkpoint exports used for milestone validation must come from a single continuous run.** Mixing exports from different runs, seeds, or code versions is not valid evidence.

Each validation run produces:

1. **A run ID** (timestamp + seed + sample count + plate count), written to the log header and to each exported filename.
2. **Checkpoint exports** at fixed steps: Step 0, Step 10, Step 25, Step 50, Step 100 (and Step 250, Step 500 for LongRun).
3. **Per-resampling metric logs** (Section 12.4) written to a single log file tagged with the run ID.
4. **Map exports** at each checkpoint: PlateId, CrustType, Elevation, BoundaryMask, OverlapMask, GapMask.

**Filename convention:** `{RunID}/Step_{N}/{FieldName}.png`

**Validation rule:** A milestone is accepted only when all checkpoint exports from a single run satisfy the acceptance criteria. Exports from prior runs, different seeds, or interrupted sessions are not valid evidence. If a run is interrupted, restart from Step 0.

### 12.6 Amplification Exclusion

**No milestone from M0 through M6 may use amplified terrain T as evidence of success or failure.** All acceptance criteria for M0–M6 are evaluated on raw coarse crust C.

The fair comparison target for raw C quality is the paper's Figure 12 (left panel), which shows unamplified crust data. The paper's rendered planet images (Figures 1, 13, 14, 16, 17) are amplified terrain T and are not valid comparison targets until M7.

If raw crust C looks unconvincing at M6 completion, the fix is in the simulation, not in adding amplification prematurely.

---

## 13. What to Port From v4

### Port (concepts and utilities)

- Fibonacci sphere sampling
- `TConvexHull3<double>` SDT construction
- Shewchuk's `predicates.c` integration
- Neighbor graph extraction from convex hull
- Cartesian AABB BVH (or equivalent)
- Spherical bounding cap computation and culling
- Barycentric coordinate computation
- Orient3d containment predicate
- Profiling/timing infrastructure
- Map exporter projection math (equirectangular, Mollweide, Winkel Tripel)
- Elevation color ramp

### Do Not Port (v4-specific control logic)

- Boundary triangle splitting / synthetic midpoint construction
- Phase 6 ownership repair pipeline (sanitize, connectivity, rescue)
- Hysteresis threshold and ownership margin
- Contender support guard
- Fast path optimization
- Protected plate rescue system
- Continental area floor stabilizer
- `prev_plate_id` / ownership stickiness
- Any ablation/benchmark infrastructure that depends on the above

### Port Only If Measured Failure Demands It

- Ownership hysteresis (if boundaries chatter without it)
- Connectivity enforcement (if plates fragment without it)
- Protected plate minimum floor (if plates die without it)

In each case, the measured failure must be documented with same-run map exports and logged metrics before the mechanism is added. The mechanism must be flagged as a non-paper deviation in code comments.

### 13.1 No Silent Repair — Escalation Rule

If v5 baseline behavior fails a milestone acceptance criterion, the response is **not** to add a repair pass. The escalation order is:

1. **First:** Check whether the failure is an exporter artifact or a log/export mismatch. Cross-check exported maps against sample-derived metrics (Section 12.4).
2. **Second:** Check whether the failure traces to a parameter value (resampling interval, noise amplitude, elevation constant). Adjust and re-run.
3. **Third:** Check whether the failure traces to a containment geometry issue (BVH construction, predicate robustness, soup membership). Fix the geometry.
4. **Fourth:** If the failure persists after steps 1–3, **stop implementation and revisit architecture.** Write up the failure with same-run evidence (maps + metrics + logs) and reassess whether the paper's approach is sufficient or whether a specific non-paper mechanism is justified.
5. **Only after step 4:** Add a non-paper mechanism (hysteresis, connectivity, rescue). Document the failure evidence, the mechanism, and why it is necessary in a deviation log.

**The rule:** No repair pass may be added during implementation flow. Every non-paper mechanism requires a stop, a writeup, and an explicit architecture decision. This prevents the incremental accretion that turned v4's Phase 6 into a 10-stage pipeline.

---

## 14. Open Questions

1. **Resampling interval tuning.** R_min = 10 is a starting estimate. The paper says 10–60 but doesn't specify the exact trigger. If R = 10 produces too much drift, decrease. If boundaries are clean, increase.

2. **Boundary-triangle gap fallback.** The "assign to nearest plate if surrounded by same-plate neighbors" heuristic needs testing. If it produces artifacts, alternatives include: assign to plate with majority of Delaunay neighbors, or interpolate from nearest 3 carried samples.

3. **Continental area balance.** The paper mentions the user can "prescribe global constraints, such as the relative continental to oceanic ratio." If continental area drifts too far, a gentle restoring force (converting boundary oceanic samples to continental when area drops below floor) may be needed. This should be a measured addition, not a default.

4. **Overlap resolution when multiple plates contain a sample.** The baseline uses highest minimum barycentric coordinate. The paper doesn't specify. If this produces artifacts, alternative: assign to plate with most Delaunay neighbors (similar to v4's contender support, but simpler).

5. **Per-step distance-to-front overestimate accuracy.** The paper says the overestimate is sufficient. If subduction uplift bands are too wide between resamplings, the overestimate constant can be tuned (e.g., multiply by 0.8 for a tighter estimate).

6. **Terrane merger semantics at collision boundaries.** When two terranes merge, which orogeny history prevails? Use largest contributing terrane's metadata.
