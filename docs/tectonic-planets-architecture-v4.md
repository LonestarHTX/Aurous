# Procedural Tectonic Planets — Architecture Document v4

**Based on:** Cortial et al., "Procedural Tectonic Planets" (2019)  
**Amplification based on:** Cortial et al., "Real-Time Hyper-Amplification of Planets" (2020)  
**Target:** Unreal Engine 5  
**Date:** February 2026

---

## 1. Core Architectural Principle

The planet is a fixed set of sample points on a sphere. These points never move, never change count, and never change topology. They are the authoritative representation of the planet's crust.

Plates are rigid bodies that carry copies of sample data as they rotate over the reference sphere. Between reconciliation steps, plate-carried samples may overlap or leave gaps. This is expected and correct.

Periodically, the fixed canonical samples are repopulated from plate-carried data. For each canonical point, we find which plate's moved triangle soup contains it, then interpolate crust attributes via barycentric coordinates from the containing triangle. Gaps become new oceanic crust. Overlaps trigger subduction or collision logic.

**Resampling is ownership reassignment with interpolated attribute transfer, not geometry surgery.**

The global Spherical Delaunay Triangulation is built once over the canonical points and reused permanently — for adjacency, rendering, BFS distance fields, terrane analysis, and as the source topology for plate triangle soups. It is never rebuilt.

---

## 2. Why This Architecture

Previous attempts failed at the remeshing step — trying to rebuild spherical Delaunay triangulations while preserving plate assignments, interpolating crust parameters across topology changes, and handling edge cases where boundary triangles span multiple plates.

The paper itself supports this separation. Section 6 states the crust is "a set of attributed points p over the sphere." Plates move "over the reference sphere according to a rigid geodetic movement." They "perform a global resampling every 10–60 iterations." Triangulation is described as a derived structure for spatial queries and rendering, not as the data model.

Our architecture makes that implicit separation explicit while faithfully implementing the paper's actual methods: barycentric interpolation for attribute transfer, BVH-accelerated containment queries, and Delaunay-based adjacency for boundary detection and distance fields.

---

## 3. Data Model

### 3.1 Canonical Sample Set

A fixed array of N points distributed over the unit sphere via Fibonacci sampling. This array is allocated once at initialization and never resized or repositioned.

Each canonical sample stores:

| Field | Type | Description |
|---|---|---|
| `position` | Vector3 | Fixed location on unit sphere |
| `plate_id` | int | Owning plate (reassigned each reconciliation) |
| `prev_plate_id` | int | Owner from previous reconciliation (for hysteresis) |
| `ownership_margin` | float | Distance ratio between nearest and second-nearest plate |
| `is_boundary` | bool | True if any Delaunay neighbor has a different plate_id |
| `crust_type` | enum | Oceanic or Continental |
| `elevation` | float | Surface elevation relative to sea level |
| `thickness` | float | Crust thickness |
| `age` | float | Crust age (meaning depends on type) |
| `ridge_direction` | Vector3 | Local ridge direction (oceanic only) |
| `fold_direction` | Vector3 | Local fold direction (continental only) |
| `orogeny_type` | enum | None, Andean, or Himalayan |
| `orogeny_age` | float | Age of most recent orogeny event |
| `terrane_id` | int | Stable terrane identifier (continental only, -1 if oceanic) |

**Target resolution:** 60k samples for early milestones (~100 km spacing), scaling to 500k (~35 km) for production quality.

### 3.2 Global Triangulation

A Spherical Delaunay Triangulation is built over the canonical Fibonacci points at initialization. Because these points are fixed, this triangulation is computed once and never rebuilt.

**Implementation:** The 3D convex hull of points on a unit sphere is mathematically equivalent to the spherical Delaunay triangulation. UE5 provides `TConvexHull3<double>` in `Runtime/GeometryCore/CompGeom/ConvexHull3.h` — a first-class runtime API with double-precision support. Feed it Fibonacci sphere points, call `Solve()`, and the output triangles are the SDT. No third-party libraries required.

This triangulation provides:

- **Neighbor graph:** Each sample knows its Delaunay neighbors. This is the authoritative adjacency for boundary detection, distance-to-front computation, and terrane connectivity analysis.
- **Render mesh:** The triangulation directly serves as the planet's visual surface mesh. Vertex attributes are updated each step; topology never changes.
- **Plate triangle soups:** Each plate's moved geometry is derived from this triangulation by extracting the subset of triangles whose vertices all belong to that plate and applying the plate's rigid rotation to the vertex positions. No per-plate retriangulation is needed.

### 3.3 Plate Triangle Soups

This is the key mechanism that avoids per-plate Spherical Delaunay rebuilds.

After each reconciliation assigns samples to plates, we classify canonical triangles:

- **Interior triangle:** All three vertices belong to the same plate. This triangle is assigned to that plate.
- **Boundary triangle:** Vertices span two or more plates. This triangle is discarded from all plate soups.

When a plate moves, its triangle soup moves rigidly with it — same connectivity, rotated vertex positions. A BVH is built over each plate's moved triangle soup for point-in-triangle containment queries during reconciliation.

**BVH implementation:** Standard Cartesian AABB BVH in 3D space. At the angular scale of our triangles (0.005–0.016 radians), spherical-native acceleration structures (S2 cells, spherical caps) offer negligible advantage over Cartesian AABBs — curvature waste is ~10⁻¹⁰. SAH construction requires no modification for spherical geometry. Morton-code LBVH (Linear BVH) builds 2–5× faster than SAH if construction time matters; recommended for production. A lightweight header-only library (TinyBVH or madmann91/bvh) is sufficient.

**Containment predicates:** Point-in-spherical-triangle testing uses the triple scalar product method: check `sign(dot(P, cross(A,B)))` for each triangle edge. This reduces to three `orient3d(O, A, B, P)` calls. **Shewchuk's adaptive-precision predicates** (`predicates.c`, public domain, single C file, no dependencies) are required for numerical robustness. Naive double arithmetic will produce boundary failures at 500k queries. Shewchuk's predicates handle >99% of cases at ~2× naive cost, invoking exact arithmetic only when needed.

**Broad-phase culling:** Before BVH traversal, cull plates that cannot contain the query point. Two options, ordered by implementation simplicity:

1. **Spherical bounding caps per plate** (one dot product test per plate). Culls ~95% of points per plate for uniformly-sized plates. Sort plates smallest-first so the first hit terminates the search. Simpler to implement; start here.
2. **S2 cell grid at level 7–8** (Google S2 Geometry library, Apache 2.0). Provides O(1) plate candidate lookup per point, reducing 40 BVH traversals to 1–3 on average. More complex integration; adopt if profiling shows bounding caps are insufficient.

**What happens at boundaries:** Discarded boundary triangles create narrow gaps between plate soups at their edges. This is correct behavior — those gaps are exactly where the reconciliation pipeline classifies samples as being in divergent zones (new oceanic crust) or convergent zones (subduction/collision). Interior samples are cleanly contained by their plate's soup and receive full barycentric interpolation.

**Cost:** At each reconciliation, per-plate triangle soups are rebuilt by re-classifying canonical triangles based on updated plate_id assignments and constructing BVHs over the rotated subsets. This is O(T) where T is the total triangle count — far cheaper than a full Spherical Delaunay computation.

### 3.4 Plate

Each plate is a logical grouping. A plate stores:

| Field | Type | Description |
|---|---|---|
| `id` | int | Unique identifier |
| `rotation_axis` | Vector3 | Normalized axis through planet center |
| `angular_speed` | float | Radians per timestep |
| `carried_samples` | Array | Indices into canonical array + rotated positions + copied crust data |
| `triangle_soup` | Array | Indices of canonical triangles assigned to this plate |
| `soup_bvh` | BVH | Bounding volume hierarchy over moved triangle soup |
| `terranes` | Array | Tracked continental regions within this plate |

**Carried sample collision metadata:**

Each carried sample also preserves collision state across reconciliations, enabling per-step Himalayan uplift between reconciliation events:

| Field | Type | Description |
|---|---|---|
| `collision_distance_km` | float | Sample's distance to the collision front at time of collision |
| `collision_convergence_speed_mm_per_year` | float | Plate convergence speed at time of collision |
| `collision_opposing_plate_id` | int | The other plate involved in the collision |
| `collision_influence_radius_km` | float | Influence radius computed at collision time |
| `is_collision_front` | bool | Whether this sample was on the collision front |

These fields are set during Phase 8 collision events and carried through the plate motion loop, allowing ongoing Himalayan uplift to continue between reconciliations without re-detecting the collision.

### 3.5 Terranes

A terrane is a connected region of continental crust within a plate. Terranes resist subduction, participate in continental collision, and can detach from one plate to suture onto another.

**Identity and tracking:**

Each terrane carries stable identity across reconciliations:

| Field | Type | Description |
|---|---|---|
| `terrane_id` | int | Globally unique, stable across reconciliations |
| `anchor_sample` | int | Index of a representative canonical sample near the terrane centroid |
| `centroid` | Vector3 | Rolling average position, updated each reconciliation |
| `area` | float | Approximate area (sample count × average cell area) |
| `plate_id` | int | Current owning plate |

**Identity persistence rules:**

After each reconciliation, terrane detection runs as connected component analysis over continental samples within each plate, using the Delaunay neighbor graph. Each detected component is matched to a known terrane:

1. If a component contains a previous terrane's anchor sample → inherits that identity.
2. If anchor was lost (sample flipped oceanic or changed plate) → match to component member nearest the terrane's last known centroid.
3. If no match → new terrane, new globally unique ID.
4. If a component contains anchors from multiple previous terranes → merger. Largest contributing terrane's ID is preserved; others recorded as merged.

---

## 4. Simulation Loop

Each timestep represents 2 My (million years). The loop has two modes: a motion step (every timestep) and a reconciliation step (every R timesteps, adaptive).

### 4.1 Per-Timestep: Motion and Crust Updates

```
for each plate P:
    rotate all carried samples by P.angular_speed around P.rotation_axis
    apply slab pull correction to rotation axis

for each plate P:
    for each carried sample S in P:
        apply continental erosion (if continental)
        apply oceanic dampening (if oceanic)
        apply sediment accretion (if in trench)
```

These are per-sample scalar updates. No topology involvement.

### 4.2 Every R Timesteps: Reconciliation

Reconciliation runs in nine phases. All writes target a new buffer; the old buffer remains readable until the swap.

**Phase 1 — Build per-plate moved triangle soups and BVHs.**

For each plate:
1. Collect all canonical triangles where all three vertices belong to this plate (interior triangles).
2. Apply the plate's rigid rotation to the vertex positions of those triangles.
3. Build a Cartesian AABB BVH over the rotated triangle soup.
4. Compute spherical bounding cap for the plate (for broad-phase culling in Phase 2).

This is O(T) total across all plates and does not involve any retriangulation.

**Phase 2 — Ownership reassignment with boundary-scoped hysteresis.**

For each canonical sample position, first cull candidate plates via spherical bounding cap tests (one dot product per plate), then query surviving plates' BVHs using Shewchuk-predicate containment tests to find which plate's moved triangle soup contains that point. This yields:

- The containing plate and triangle (if any)
- Barycentric coordinates within that triangle
- Whether multiple plates contain the point (overlap)
- Whether no plate contains the point (gap)

**Hysteresis rule:**

Hysteresis applies only to samples that were boundary samples in the previous step (i.e., `is_boundary == true` or `ownership_margin` below a boundary confidence cap). Interior samples always accept their containment result without stickiness.

For boundary samples: if the containing plate differs from `prev_plate_id`, reassignment occurs only if the ownership margin exceeds `hysteresis_threshold` (0.1–0.2, tuned empirically). The ownership margin is:

```
margin = (distance_to_second_nearest_plate - distance_to_nearest_plate) 
         / average_sample_spacing
```

If the sample's previous owner's soup no longer contains it (the sample falls in a gap or is only contained by other plates), reassignment is forced regardless of margin.

**Phase 3 — Attribute interpolation.**

For each canonical sample that received a containing triangle in Phase 2, interpolate crust attributes from the triangle's three vertices using barycentric coordinates.

**Continuous fields** (elevation, thickness, age, orogeny_age): standard barycentric blend.

**Categorical fields** (crust_type, orogeny_type): majority vote from the three vertices.

**Direction fields** (fold_direction, ridge_direction): blend in 3D Cartesian space, then project once. Direction vectors are stored as 3D vectors satisfying v · p = 0 (tangent to sphere). This approach is singularity-free everywhere including poles.

1. Barycentric blend the three vertex direction vectors directly in 3D.
2. Project the blended result onto the tangent plane at the canonical sample position: `v_proj = v - dot(v, p) * p`.
3. Renormalize.
4. If the blended magnitude is below a degeneracy threshold (indicating near-cancellation), fall back to the direction of the vertex with the largest barycentric weight.

**Phase 4 — Gap detection.**

Any canonical sample not contained by any plate's moved triangle soup is in a divergent zone. Initialize as new oceanic crust:

- `crust_type` = oceanic
- `age` = 0
- `elevation` = blended between ridge profile z_Γ and interpolated border elevation using factor α
- `ridge_direction` = computed from relative motion of the two flanking plates
- `plate_id` = assigned to the nearer flanking plate

Additionally, samples that fall in the narrow gaps left by discarded boundary triangles but are clearly interior to a plate (surrounded by that plate's samples on all sides) are assigned by nearest moved sample as a fallback. This handles the geometric artifact of boundary triangle removal without affecting the divergent zone logic.

**Phase 5 — Overlap detection and convergence classification.**

A sample is in a **convergent zone** if and only if:

1. Two or more plates' moved triangle soups contain the sample (spatial overlap), **AND**
2. The relative velocity of those plates at that location has a negative component along the boundary normal (plates converging)

**Boundary normal computation:** For stability, the boundary normal at a sample is not the direction to a single neighboring-plate sample. Instead, compute the mean direction vector to all Delaunay neighbors belonging to other plates (the 1-ring cross-plate neighbors), project onto the tangent plane at the sample, and normalize. This produces a smoother, less jittery normal than a single-neighbor direction.

The velocity test: `dot(v_rel, n_boundary) < 0` where `v_rel = s_i(p) - s_j(p)` is the relative surface velocity of the two plates.

Convergent samples are classified for interaction:

- Oceanic-oceanic → subduction (older plate subducts)
- Oceanic-continental → subduction (oceanic subducts)
- Continental-continental → continental collision (check terrane involvement)

**Phase 6 — Update plate membership.**

Rebuild each plate's `carried_samples` array from canonical samples carrying its plate_id. Update `ownership_margin` for all samples. Recompute `is_boundary` flags from the Delaunay neighbor graph.

**Phase 7 — Terrane detection and identity tracking.**

Run connected component analysis over continental samples within each plate using the Delaunay neighbor graph. Match detected components to tracked terranes via the identity persistence rules (Section 3.5). Update anchor samples, centroids, and areas.

**Phase 8 — Continental collision.**

Detect and apply continental collision events. This phase runs as a sequential event loop:

```
while (collision detected):
    1. Identify next collision (convergent boundary with continental overlap between distinct terranes)
    2. Select donor terrane (smaller) and receiver plate
    3. Apply terrane detachment: donor samples switch plate_id to receiver plate
    4. Apply elevation surge with compactly supported falloff (Section 5.2)
    5. Set collision metadata on affected samples (distance, speed, influence radius)
    6. Refresh canonical state:
       - Sanitize plate connectivity (enforce single connected component per plate)
       - Rescue protected plates (ensure minimum plate persistence)
       - Rebuild membership, carried samples, classification, spatial structures
       - Re-detect terranes
    7. Re-check for new collisions created by topology changes
```

The sequential loop with per-event topology refresh is necessary because each collision changes plate boundaries, potentially creating or exposing new collision fronts. Batching collisions without refresh risks stale topology.

**Phase 9 — Subduction field propagation.**

Compute BFS distance-to-front for subduction boundaries and apply per-reconciliation subduction uplift (Andean orogeny). This runs after Phase 8 so that collision-modified boundaries are reflected in the distance fields.

After Phase 9, carried samples are rebuilt from the final canonical state, and `prev_plate_id` is written back for all samples.

### 4.3 Reconciliation Frequency

Reconciliation is triggered when max plate displacement since the last reconciliation exceeds a fraction of average sample spacing:

```
trigger when: max_displacement > 0.5 × average_sample_spacing
```

With max plate speed v₀ = 100 mm/year and timestep δt = 2 My:

- Per-step displacement: ~200 km
- At 100 km spacing (60k points): reconcile every 1 step
- At 35 km spacing (500k points): reconcile every 3–6 steps

At 60k, reconciliation runs every timestep. At 500k, reconciliation runs every 3–6 timesteps but is cheap enough to run every timestep if desired.

**Measured performance estimates (modern desktop CPU, circa 2024–2025):**

| Operation | 60k | 500k | Notes |
|---|---|---|---|
| Triangle soup classification | < 1 ms | < 5 ms | O(T), single scan |
| BVH construction (all plates) | < 2 ms | 4–14 ms | LBVH; SAH is 10–40 ms |
| Broad-phase culling + containment | < 3 ms | 10–30 ms | With spherical cap culling |
| Barycentric interpolation | < 1 ms | < 5 ms | O(N), trivial |
| BFS distance fields | < 2 ms | < 10 ms | O(N+E) |
| Terrane detection | < 1 ms | < 5 ms | Connected components, O(N+E) |
| **Total per reconciliation** | **< 10 ms** | **15–55 ms** | **Single-threaded** |
| **Total (16-core parallel)** | **< 3 ms** | **3–10 ms** | **ParallelFor** |

These numbers cover the core ownership-and-interpolation pipeline (Phases 1–7). Pre-M5, the full reconciliation step additionally includes Phase 8 collision with per-event topology refresh, Phase 9 subduction propagation, and pre-rifting invariant enforcement (Section 4.4), which materially exceed the core budget at high plate counts. These costs are tracked in benchmarks and expected to improve at M6 with batched refresh and algorithmic optimization.

### 4.4 Pre-Rifting Invariants

Until M5 introduces plate rifting (which creates and destroys plates dynamically), the simulation enforces several structural invariants that would otherwise be maintained by the full plate lifecycle:

- **Plate connectivity:** Each plate must remain a single connected component on the Delaunay graph. If collision or terrane transfer fragments a plate, the largest component is kept and orphaned fragments are reassigned to adjacent plates.
- **Protected plate persistence:** Plates initialized at startup must retain a minimum sample count floor (derived from total samples / plate count). If a plate shrinks below this floor, samples are rescued from neighboring plates to prevent plate death before rifting mechanics exist to manage plate creation/destruction.
- **Continental area floor stabilizer:** Total continental area fraction must not drop below a configured minimum (currently 25%). If terrane transfers or collision events reduce continental area below this threshold, the stabilizer converts oceanic boundary samples to continental to compensate.

These invariants are **temporary scaffolding**. At M5, plate rifting will provide the natural mechanisms for plate birth, death, and area rebalancing. At that point, connectivity enforcement and protected plate persistence should be relaxed or removed, and the continental area floor may be replaced by rifting-driven area feedback.

---

## 5. Tectonic Processes

All four processes operate on canonical samples using crust attributes and the Delaunay neighbor graph.

### 5.1 Subduction

**Trigger:** Convergent zone (Phase 5) where an oceanic plate meets any other plate and the oceanic plate is denser (older).

**Continuous uplift:** The overriding plate's samples near the subduction boundary receive uplift each timestep:

```
ũⱼ(p) = u₀ · f(d(p)) · g(v(p)) · h(z̃(p))
```

**Transfer functions:**

- `f(d)` — Piecewise cubic distance profile. Maximum at a control distance from the subduction front, fading to zero at r_s = 1800 km.
- `g(v)` — Linear speed scaling: `g(v) = v / v₀`.
- `h(z̃)` — Quadratic elevation weighting: `h(z̃) = z̃²` where `z̃ = (z - z_t) / (z_c - z_t)`.

**Distance-to-front:** BFS from subduction boundary samples outward through the Delaunay neighbor graph, accumulating true geodesic arc lengths along edges. This produces smooth, continuous distance values.

**Elevation update:**

```
z(p, t+δt) = min(z(p, t) + ũⱼ(p) · δt, z_c)
```

The elevation ceiling clamp `z_c` (max continental altitude, 10 km) prevents unbounded uplift accumulation at long-lived subduction zones.

**Fold direction update:**

```
f(p, t+δt) = f(p, t) + β · (sᵢ(p) - sⱼ(p)) · δt
```

Followed by projection onto tangent plane (`v_proj = v - dot(v, p) * p`) and renormalization to prevent drift.

**Slab pull:** The subducting plate's rotation axis is nudged toward the subduction front:

```
wᵢ(t+δt) = wᵢ(t) + ε · Σ (cᵢ × qₖ) / |cᵢ × qₖ| · δt
```

Where qₖ are subduction front samples and ε ≪ 1.

**Forced subduction:** When oceanic terranes or high-relief features enter the subduction zone, they produce greater deformation of the overriding plate. Orogeny type is set to Andean.

### 5.2 Continental Collision

**Trigger:** Convergent zone where two continental terranes from different plates overlap.

**This is a discrete event.** When terrane R from plate Pᵢ first contacts plate Pⱼ (determined by terrane identity tracking — first-contact vs. ongoing):

1. **Slab break:** Terrane R detaches from Pᵢ. Its samples switch plate_id to Pⱼ.

2. **Elevation surge:**

```
Δz(p) = Δc · A · f(d(p, R))
f(x) = (1 - (x/r)²)²  for x < r, else 0
```

3. **Influence radius:**

```
r = r_c · √(v(q) / v₀ · A / A₀)
```

4. **Fold direction:** Set perpendicular to the collision front via tangent-plane cross product with the surface normal.

5. **Orogeny type:** Himalayan. Age reset to 0.

6. **Elevation ceiling:** `z(p) = min(z(p) + Δz(p), z_c)`. The same ceiling clamp as subduction prevents collision surges from exceeding geophysical bounds.

**Per-step Himalayan uplift:**

Between reconciliations, samples with collision metadata (set during Phase 8) receive continuous uplift each timestep:

```
ũ_H(p) = u₀ · f_H(d_col(p), r_col) · g(v_col(p)) · h(z̃(p))
z(p, t+δt) = min(z(p, t) + ũ_H(p) · δt, z_c)
```

Where `d_col`, `v_col`, and `r_col` are the collision distance, convergence speed, and influence radius carried from the collision event (Section 3.4). This produces ongoing mountain building after the initial discrete surge, using the same transfer functions as subduction uplift but keyed to the collision geometry rather than a live boundary.

### 5.3 Oceanic Crust Generation

**Trigger:** Gap samples identified in reconciliation Phase 4.

**Elevation profile:** Blended between template ridge shape and interpolated plate border elevation:

```
α = d_Γ(p) / (d_Γ(p) + d_P(p))
z(p) = α · z̄(p) + (1 - α) · z_Γ(p)
```

**Ridge direction:** `r(p) = (p - q) × p` where q is the projection of p onto the ridge line. Stored for procedural amplification of transform faults.

**Plate assignment:** New samples assigned to the nearer flanking plate.

### 5.4 Plate Rifting

**Trigger:** Poisson process:

```
P = λ · e^(-λ)
λ = λ₀ · f(x_P) · A / A₀
```

**Process:**

1. Select plate P. Fracture into n sub-plates (n ∈ [2, 4]).
2. Distribute n random centroids within P's domain.
3. Compute spherical Voronoi cells. Warp boundaries with coherent noise.
4. Reassign samples to sub-plates by Voronoi cell.
5. Assign random diverging geodetic movements.
6. Sub-plates inherit terrane fragments based on received samples.

---

## 6. Boundary Detection

Boundaries emerge from the Delaunay neighbor graph and are never stored as explicit geometry.

**Definition:** A canonical sample is a boundary sample if any Delaunay neighbor has a different plate_id. The `is_boundary` flag is recomputed after each reconciliation Phase 6.

**Boundary classification** uses relative velocity:

- **Convergent:** `dot(v_rel, n_boundary) < 0`
- **Divergent:** `dot(v_rel, n_boundary) > 0`
- **Transform:** Relative velocity predominantly tangential to boundary.

**Boundary normal:** Computed as the mean direction vector to all cross-plate Delaunay neighbors (1-ring), projected onto the tangent plane and normalized. This is more stable than a single-neighbor direction.

**Distance-to-boundary:** BFS on Delaunay graph from boundary samples outward, accumulating geodesic arc lengths. Produces smooth distance fields for subduction uplift.

**BVH acceleration:** Each plate's Cartesian AABB BVH (built over its moved triangle soup in reconciliation Phase 1) accelerates spatial queries. Broad-phase plate culling via spherical bounding caps further reduces query cost.

---

## 7. Erosion and Dampening

Continuous per-sample processes, every timestep.

**Continental erosion:**

```
z(p, t+δt) = z(p, t) - (z(p, t) / z_c) · ε_c · δt
```

**Oceanic dampening:**

```
z(p, t+δt) = z(p, t) - (1 - z(p, t) / z_t) · ε_o · δt
```

**Sediment accretion:**

```
z(p, t+δt) = z(p, t) + ε_f · δt
```

**Low-frequency noise:** Applied globally to break up uniform patterns.

---

## 8. Amplification

Post-simulation step converting coarse crust C into high-resolution terrain T. Decoupled from simulation. Based on Cortial et al., "Real-Time Hyper-Amplification of Planets" (2020).

### 8.1 Overview

The follow-up paper provides a GPU-based hierarchical subdivision system that takes a ~50 km resolution crust model and produces ~50 cm ground detail — an amplification factor of ~10⁵ × 10⁵. It generates river networks, valleys, lakes, mountain ranges, plateaus, hills, gullies, and drainage patterns, all procedurally from crust control parameters, all in real time.

This replaces the original paper's SRTM exemplar approach for continental terrain. Oceanic terrain still uses procedural noise (Gabor or similar).

### 8.2 Input Control Maps

The hyper-amplification system consumes control maps derived from our crust simulation:

- **Crust elevation C** — directly from canonical sample elevation
- **Orogeny age A** — directly from canonical sample orogeny_age
- **Humidity W** — derived from latitude and distance-to-coast (not in simulation; added as a post-processing step or artist-paintable overlay)
- **Plateau P and Hill B presence** — derived from crust parameters or artist-paintable overlays

### 8.3 Preprocessing (CPU)

1. Generate Poisson disk sampling of the crust surface at ~50 km spacing
2. Build spherical Delaunay triangulation over Poisson samples
3. Regularize coastal triangles for estuary generation
4. Grow river network from coastlines inward using reverse-flow algorithm
5. Compute Horton-Strahler indices and flow estimates for river segments

### 8.4 Real-Time Subdivision (GPU)

Hierarchical rule-based dyadic subdivision on the GPU. Each edge split is governed by production rules keyed to vertex and edge types (river, terrain, lake, sea, gully). Rules control:

- River network densification (new tributaries at each subdivision level)
- Elevation computation via compute graphs (blending mountain type generators based on A, W, P, B)
- Horizontal displacement for natural irregularity
- Lake generation connected to river network

After subdivision, a post-processing pass carves riverbeds and valleys using flow-scaled cross-section templates, producing geomorphologically consistent drainage patterns.

### 8.5 Oceanic Amplification

Oceanic terrain uses procedural noise rather than subdivision rules:

- 3D Gabor noise added to oceanic elevation
- Amplitude/frequency scale with crust age
- Orientation aligns with ridge direction
- Young crust near ridges gets accentuated fault features

### 8.6 Performance

On 2016 hardware (GTX 1080 Ti): ~25 fps at ground level, subdivision runs every ~10 frames at ~80 ms per pass, up to 4M triangles at maximum detail. Modern hardware would significantly exceed this.

---

## 9. Rendering in UE5

### 9.1 Mesh Strategy

The Spherical Delaunay Triangulation serves directly as the Procedural Mesh Component. Built once, never rebuilt.

Each step, update vertex attributes only:

- Position (radial displacement based on elevation)
- Normal (recomputed from Delaunay neighbor elevations)
- Custom data channels (crust type, plate ID, age, terrane ID)

### 9.2 Ocean Surface

Separate translucent sphere at sea level. Crust mesh extends below for ocean floor geometry.

### 9.3 Amplification Rendering

Per Cortial et al. 2020: GPU compute shader pipeline performs hierarchical subdivision every ~10 frames, generating up to 4M triangles at ground level. Output streamed directly to graphics pipeline for deferred shading. Double-precision vertex positions required for centimeter-scale stability at planetary radius — reconstruct world position from camera-relative fragment distance in fragment shader. Log-z depth buffering for seamless orbital-to-ground transitions.

### 9.4 Debug Visualization

Material-switchable display modes:

- Elevation (color ramp: trench → sea level → peaks)
- Plate ID (unique color per plate)
- Crust type (oceanic vs continental)
- Crust age (young → old gradient)
- Orogeny type (None / Andean / Himalayan)
- Terrane ID (unique color per terrane)
- Boundary overlay (convergent / divergent / transform, color-coded)
- Ownership margin (boundary confidence gradient)
- Distance-to-front (BFS distance field from boundaries)
- `is_boundary` flag (binary overlay)

---

## 10. Milestone Plan

### Milestone 0 — Sphere, Triangulation, Plates, and Rendering

**Goal:** Static foundation — canonical sampling, Delaunay, plate initialization, UE5 mesh.

- Generate Fibonacci sphere sampling (60k points)
- Build Spherical Delaunay Triangulation via `TConvexHull3<double>::Solve()` (once, permanent)
- Extract neighbor graph from convex hull output
- Integrate Shewchuk's `predicates.c` into the project (dependency for Milestone 1)
- Initialize 5–8 plates via spherical Voronoi from random centroids
- Assign canonical samples to plates
- Classify triangles as interior (all vertices same plate) vs boundary
- Build Procedural Mesh Component from Delaunay triangulation
- Implement debug visualization: plate ID, boundary overlay
- No simulation — static foundation only

**Validates:** Fibonacci quality, `TConvexHull3` correctness at 60k+ points (check for degenerate slivers), triangle classification, UE5 mesh pipeline.

### Milestone 1 — Plate Motion and Reconciliation

**Goal:** Plates move, reconciliation reassigns ownership with barycentric interpolation.

- Assign geodetic movements to plates
- Implement rigid rotation of carried samples
- Build per-plate moved triangle soups from canonical triangulation (interior triangles only, rotated)
- Build Cartesian AABB BVH over each plate's soup (header-only lib: TinyBVH or madmann91/bvh)
- Compute spherical bounding caps per plate for broad-phase culling
- Implement reconciliation Phases 1–7:
  - Broad-phase cap culling → BVH containment queries (Shewchuk predicates)
  - Boundary-scoped hysteresis
  - Barycentric attribute interpolation (3D blend + project for directions)
  - Gap/overlap classification
  - Plate membership rebuild
  - Boundary flag recomputation
  - Terrane detection stub (identity tracking deferred to Milestone 4)
- Profile reconciliation cost at 60k (expect < 10 ms single-threaded)
- Run 100+ timesteps and verify:
  - No orphan samples
  - No boundary chatter
  - Smooth attribute transitions
  - Stable plate boundaries

**Validates:** Core reconciliation, triangle soup approach, BVH + broad-phase performance, hysteresis, interpolation, Shewchuk predicate robustness.

### Milestone 2 — Oceanic Crust Generation

**Goal:** Diverging plates produce new ocean floor.

- Implement boundary classification (convergent / divergent via velocity test with averaged normals)
- During Phase 4, initialize gap samples as oceanic crust
- Implement ridge elevation profile and α blending
- Ridge direction computation
- Oceanic dampening per timestep
- Handle boundary-triangle-gap fallback (Phase 4 detail)
- Verify: spreading ridges, aging ocean floor, no ridge discontinuities

**Validates:** Gap detection, divergent classification, oceanic initialization, dampening.

### Milestone 3 — Subduction

**Goal:** Convergent oceanic-continental boundaries produce uplift and trenches.

- Implement convergent boundary detection (overlap + velocity test)
- Geodesic BFS distance-to-front (arc-length accumulation on Delaunay edges)
- Uplift transfer functions f(d), g(v), h(z̃)
- Per-timestep elevation update
- Fold direction update (tangent-space, renormalized)
- Slab pull axis correction
- Forced subduction for oceanic terranes
- Verify: mountain ranges parallel to boundaries, oceanic trenches, smooth uplift profiles

**Validates:** Convergence detection, distance fields, uplift curves, slab pull.

### Milestone 4 — Continental Collision

**Goal:** Continental-continental convergence produces Himalayan orogeny.

- Full terrane detection with identity persistence (anchor samples, centroid tracking, merger rules)
- Detect continental overlap in Phase 5
- Slab break: terrane detachment and reassignment
- Discrete elevation surge with compactly supported falloff
- Influence radius scaling
- Orogeny classification and fold direction
- Verify: collision mountain ranges, one-time event per terrane contact, identity persistence

**Validates:** Terrane tracking, collision events, discrete elevation, identity system.

### Milestone 5 — Plate Rifting

**Goal:** Large plates fracture stochastically.

- Poisson-triggered rifting with continental area scaling
- Spherical Voronoi sub-plate generation with noise-warped boundaries
- Sample reassignment to sub-plates
- Terrane inheritance through fragmentation
- Diverging motion assignment
- Verify: supercontinent breakup, new basins, terrane survival, dynamic plate count

**Validates:** Plate creation, Voronoi fracturing, terrane fragmentation.

### Milestone 6 — Full Simulation

**Goal:** All processes running, tuned to paper parameters.

- Continental erosion, sediment accretion, low-frequency noise
- Tune all constants to Table 3
- Long-duration runs (500+ My):
  - Tectonic cycle: rift → drift → collide → rift
  - Mountain formation and erosion
  - Ocean basin opening and closing
  - Numerical stability
  - Visual plausibility

**Validates:** Full loop, parameter balance, long-term stability.

### Milestone 7 — Amplification

**Goal:** Coarse crust → detailed terrain via Cortial et al. 2020 hyper-amplification.

- Implement CPU preprocessing: Poisson disk sampling, river network generation, Horton-Strahler indices
- Derive humidity W and landform type controls (P, B) from crust parameters
- Implement GPU hierarchical rule-based subdivision (production rules R1–R5, internal edge rules)
- Implement elevation compute graphs for each vertex/edge configuration
- Implement river/valley cross-section carving post-process
- Implement lake generation connected to river network
- Implement oceanic amplification via procedural Gabor noise
- UE5 GPU compute shader pipeline integration
- Verify: seamless orbital-to-ground zoom, hydrologically consistent landscapes, no tiling artifacts

### Milestone 8 — User Control and Authoring

**Goal:** Interactive planetary authoring per paper Section 7.1.

- Pause/resume simulation
- Prescribe plate movements
- Trigger rifting at specified locations
- Switch crust types for regions
- Adjust plate count and continental/oceanic ratio
- Real-time debug visualization during authoring

---

## 11. Key Parameters

From the paper's Appendix A (Table 3):

| Parameter | Value | Description |
|---|---|---|
| δt | 2 My | Simulation timestep |
| R | 6370 km | Planet radius |
| z_r | -1 km | Oceanic ridge elevation |
| z_a | -6 km | Abyssal plain elevation |
| z_t | -10 km | Oceanic trench elevation |
| z_c | 10 km | Max continental altitude |
| r_s | 1800 km | Max subduction influence distance |
| r_c | 4200 km | Max collision influence distance |
| Δ_c | 1.3 × 10⁻⁵ km⁻¹ | Collision coefficient |
| v₀ | 100 mm/year | Max plate speed |
| ε_o | 4 × 10⁻² mm/year | Oceanic elevation dampening |
| ε_c | 3 × 10⁻² mm/year | Continental erosion |
| ε_f | 3 × 10⁻¹ mm/year | Sediment accretion |
| u₀ | 6 × 10⁻¹ mm/year | Base subduction uplift |
| hysteresis_threshold | 0.1–0.2 | Boundary ownership stickiness (tuned empirically) |

---

## 12. Threading and Determinism

**Constraints:**

- All reconciliation writes target a new buffer. No in-place mutation during reconciliation.
- Stable tie-breaking: when distances are equal, prefer lower plate_id, then lower sample index.
- Per-sample crust updates (erosion, dampening, uplift) are embarrassingly parallel and order-independent.
- Plate motion is per-plate with no inter-plate dependencies.
- Compile with `/fp:strict` (MSVC) or `-ffp-contract=off` (GCC/Clang) to ensure deterministic floating-point results across parallel execution.

**Execution:**

- Simulation runs on a background thread (UE5 async task).
- Reconciliation queries are independent per-sample reads against immutable plate state, writing to pre-allocated output arrays. Parallelized via `ParallelFor` with no atomics or ordering dependencies.
- Render thread reads from the most recently completed canonical buffer.
- Double-buffering: one buffer written by simulation, one read by rendering. Atomic swap on completion.

---

## 13. Resolved Questions

**Spherical Delaunay implementation.** ✅ Resolved. Use `TConvexHull3<double>` from UE5's GeometryCore. Native, runtime-capable, O(N log N). No third-party libraries needed.

**Performance at 500k.** ✅ Resolved (core pipeline). The ownership-and-interpolation pipeline (Phases 1–7) is 15–55 ms single-threaded, 3–10 ms parallel. Broad-phase culling via spherical bounding caps reduces BVH queries from O(plates) to O(1–3) per sample. Full reconciliation including collision and invariant enforcement is higher; see Section 4.3 note.

**Containment query robustness.** ✅ Resolved. Shewchuk's adaptive-precision predicates (`predicates.c`) handle edge cases at 500k queries. Public domain, single C file, no dependencies.

**Barycentric interpolation on curved surfaces.** ✅ Resolved. Planar barycentrics are sufficient. Curvature error is O(h²/12) ≈ 0.002% at max triangle size, translating to ~0.2 m on a 10 km elevation range. Not worth correcting.

**Direction vector interpolation.** ✅ Resolved. Blend 3D Cartesian vectors directly with barycentric weights, project onto tangent plane once at query point, renormalize. Singularity-free everywhere including poles.

---

## 14. Open Questions

**Hysteresis threshold tuning.** The 0.1–0.2 range is a starting estimate. Too low → chatter persists. Too high → boundaries become artificially sticky during fast convergence. Empirical tuning in Milestone 1.

**Boundary-triangle fallback in Phase 4.** Samples that fall in the narrow gaps left by discarded boundary triangles need a fallback assignment. The proposed "assign by nearest moved sample if surrounded by same-plate neighbors" heuristic needs testing to confirm it doesn't interfere with divergent gap detection.

**Terrane merger semantics.** When two terranes from the same plate merge post-collision, which orogeny history prevails? The anchor-based system handles identity; attribute blending at merger boundaries needs specification.

**Rifting fracture quality.** Voronoi + noise may not produce convincing rift geometry at coarse resolution. Alternative approaches (random walk, stress-field cracks) remain available without architectural change.

**TConvexHull3 stress test.** All Fibonacci points are equidistant from the origin (coplanar with the unit sphere). The convex hull algorithm must handle this near-coplanarity at double precision. Expected to be fine given angular separation at 500k, but must be validated early in Milestone 0 — check for degenerate slivers and confirm all points appear in the output.

**S2 cell grid integration.** If spherical bounding cap culling proves insufficient at high plate counts (30–40 plates), Google S2 Geometry library integration may be needed. Apache 2.0 licensed but adds a dependency. Evaluate after Milestone 1 profiling.

**Humidity and landform type derivation.** The hyper-amplification paper requires humidity W, plateau P, and hill B control maps that the tectonic simulation does not produce. These must either be derived algorithmically (latitude + distance-to-coast for W; crust parameters for P and B) or provided as artist-paintable overlays. Specification needed before Milestone 7.
