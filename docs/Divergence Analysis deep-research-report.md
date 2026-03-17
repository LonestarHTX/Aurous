# Divergence Analysis for UE5 v5 Architecture vs Cortial 2020 Thesis and 2019 Paper

## Sources and access notes

Your v5 architecture spec was compared against entity["book","Synthèse de terrain à l'échelle planétaire","phd thesis 2020"] by entity["people","Yann Cortial","computer graphics researcher"] (doctoral work at entity["organization","INSA Lyon","lyon, france"] / entity["organization","LIRIS","lyon, france"]). citeturn7view0turn10view0turn13view0turn37view0  
I was not able to retrieve the full text PDF of the 2019 *Procedural Tectonic Planets* paper directly in this environment (it is hosted behind access controls in the sources I could reach), so all “paper vs v5” divergences below are grounded in the thesis where it restates the full algorithm and (explicitly) gives formulas and implementation details that the paper summarizes. citeturn38view0turn37view0

## Resampling and reconciliation pipeline divergences

The thesis’ “divergence handling” is not a local boundary patch; it is a **periodic global remeshing** scheme designed to (a) avoid incremental Delaunay growth and (b) guarantee **watertight global coverage** (“étanche”) by construction. citeturn10view0turn12view3  
Your Section 3 architecture implements a **containment-driven ownership pipeline** (per-plate soups, interior-only containment, neighborhood-majority guard, and self-copy fallbacks), which differs in several load-bearing ways.

### Divergences that change topology/coverage behavior

| v5 architecture location | What v5 does | What the thesis does | Why this is a divergence | Impact |
|---|---|---|---|---|
| **2.3 Plate Triangle Soups (Containment Only)** + **3.2 Phase 4 gap handling** | Excludes boundary triangles from containment soups (“discarded boundary triangles”), creating “boundary-region gaps” that must be repaired by self-copy / neighbor-majority plate assignment. | Represents each plate as a **partial spherical Delaunay triangulation** extracted from the global TDS by **copy/duplicate + re-index + re-compact**, explicitly adding “empty neighborhood” at plate borders; during global remeshing, tiles are re-partitioned and duplicated into new plate triangulations “de manière étanche”. citeturn13view0turn12view3turn10view0 | In the thesis, the primary invariant is **complete coverage via triangulation duplication** (no “coverage holes” near boundaries). In v5, holes are expected and repaired heuristically. | **Critical** (this is exactly the class of bug you’ve already hit: gaps/stippling/degenerate behavior at boundaries). |
| **3.2 Phase 4 — “Boundary-region gap fallback”** | If “not-contained” samples are not in a *divergent* configuration, v5 assigns them by **Delaunay-neighbor majority** and then interpolates/copies from the winning plate. | In the thesis, “no valid intersection” during remeshing sampling is treated as the vertex being in a **divergence zone**; it then computes oceanic crust from nearest boundary points on *two distinct plates*, and finally assigns the nearest plate index (no separate category of “boundary-region gaps”). citeturn10view0turn12view3 | v5 introduces a third state (“gap but not divergence”), which is not described in the thesis’ remeshing algorithm—because the thesis’ geometry pipeline is meant to avoid creating that state in the first place. | **Critical**: this can silently “paper over” structural boundary errors until subduction/collision uses those samples and fails visually. |
| **3.2 overall orchestration** | Resampling is a multi-phase procedure driven by containment + later repair and re-partition, with explicit “no sanitization” philosophy in Phase 6. | Divergence is solved **globally** at an *episodic period* and is explicitly framed as “re-mesh the entire planet” rather than inserting additional points into per-plate triangulations. citeturn10view0turn16view2 | The thesis relies on the *global remesh* to restore consistency and prevent incremental drift artifacts. v5’s pipeline does not adopt that same “global remesh as the source of truth” structure; instead it tries to maintain correctness locally via containment and guards. | **Risky → Critical** depending on how many “boundary-region gaps” are actually produced at 40 plates. |

### Divergences in the sampling/ownership primitive

| v5 architecture location | What v5 does | What the thesis does | Why this is a divergence | Impact |
|---|---|---|---|---|
| **3.2 ownership assignment (Phases 1–3 & 6)** | Uses per-plate BVH containment queries against (interior) triangle soups, then resolves ambiguity with a neighbor-majority guard and fallback behaviors. | During remeshing, for each vertex **p** of the global TDS: cast a **ray from the planet center through p**, test ray–triangle intersections against each plate BVH; if intersection is valid, barycentrically interpolate crust parameters from the intersected triangle; otherwise treat as divergence and create new crust from boundary distances. citeturn10view0turn15view0 | The thesis’ ownership is defined by a **ray intersection functional** (center → surface), not by “point containment in plate-local soups”. That matters at boundaries and during overlaps because “which triangle the ray hits” is the disambiguation rule. | **Risky**: if your containment predicate and triangle sets are not identical to the thesis’ intersection sets, ownership will differ most where you can least afford it (boundaries). |
| **Triangle exclusion during sampling** | v5 does not describe excluding subduction/collision geometry from ownership tests; it mainly excludes boundary triangles from soups. | During remeshing sampling, intersections that hit **a triangle in subduction or collision are ignored** (not counted as a valid hit). citeturn10view0turn15view0 | This is an explicit thesis rule to prevent remeshing from “sampling from geometry that is conceptually gone / not authoritative.” If you don’t replicate it, your resample may suck “wrong crust” from subducting/trench triangles or collision geometry. | **Critical** once you implement M3/M4: it can seed wrong crust fields that then propagate. |

### Consequences of your interior-only + repair approach you may not have anticipated

These follow directly from what the thesis’ pipeline is trying to prevent by design (global remesh + duplication to “étanche”), and from the places where the thesis explicitly rejects expensive “exactness” in favor of stable heuristics.

**Boundary-induced mislabeling cascades (critical):** In v5, a boundary gap first becomes a local ownership patch (neighbor-majority / self-copy), but that patched `plate_id` then feeds downstream computations that assume `plate_id` is physically meaningful (terrane connected components, subduction-front seeding, BFS distance propagation, collision tracking). The thesis avoids this by ensuring each global TDS vertex is deterministically assigned by ray hit or treated as divergence. citeturn10view0turn12view3

**False “divergence” artifacts vs missed true divergence (risky→critical):** In the thesis, divergence is detected exactly when no valid intersection exists; then oceanic crust is generated using the two nearest boundary points on distinct plates (q1, q2) and a ridge midpoint approximation. citeturn10view0  
In v5, divergence requires (a) not-contained and (b) a velocity-based test that the gap is truly divergent; otherwise you do a topology-only fallback. This creates two new failure modes:  
- *Missed ridges*: true divergence zones that you “patch” instead of generating new oceanic crust (leaving “borrowed” crust at what should be newborn ridge).  
- *Phantom ridges*: containment holes that pass your divergence test and generate ridge crust where the thesis would still find a valid intersection if boundary triangles were included.

**Sensitivity to plate count (likely critical at 40 plates):** The thesis reports benchmarks explicitly with **40 plates** as an initial condition (i.e., it expects the method to work at that count). citeturn15view1  
Interior-only soups increase the ratio of “boundary-adjacent area” to “interior area” as plate count grows, so the fraction of samples that can fall into your “boundary-region gap fallback” rises sharply—meaning your fixes are exercised disproportionately at 40 plates, exactly the regime you care about.

**Downstream graph computations depend on boundary correctness (critical):** Your plan relies on Delaunay-neighbor relations for boundary identification and BFS distance fields; but those computations assume the plate partition on the global graph is coherent. The thesis tries to preserve coherence by operating triangle-wise with explicit “tracking lists” seeded from boundary triangles and neighbor traversal. citeturn14view1turn9view4

## Subduction divergences

Your subduction section (5.1) mirrors the thesis’ high-level model shape (uplift as a product of distance/speed/elevation transfer functions), but several **implementation-level choices differ**, and some thesis “gotchas” are not present in v5.

### Distance-to-front computation is not the same problem in the thesis

| v5 architecture location | What v5 does | What the thesis does | Why this is a divergence | Impact |
|---|---|---|---|---|
| **3.2 Phase 8 + 5.1 “Exact computation (BFS on Delaunay graph)”** | Computes an “exact” distance-to-front via BFS propagation over the Delaunay neighbor graph each resampling, then uses per-step overestimation between resamplings. | Treats the distance to the convergence/subduction front as an **explicitly approximate quantity** advanced by plate motion: \( d(p,t+\delta t)=d(p)+s(p)\delta t \), seeded at 0 when a triangle enters the tracking list; the thesis states that computing the “real” nearest-front distance would require costly mesh traversal and discrimination among multiple plates, so it keeps the approximation because it is “extremely fast” and “produces good results.” citeturn9view4 | BFS distance over a static adjacency graph answers “graph distance to current front,” whereas the thesis’ d(p) is closer to “penetration time × speed” for tracked convergent triangles (a state variable). These differ especially when fronts shift or multiple overlaps exist. | **Risky → Critical**: if you combine BFS with your boundary-gap repairs, you can get discontinuous distance fields (stippling) and wrong uplift envelopes. |

### Uplift profile and transfer functions differ in form (even if the intent matches)

The thesis explicitly gives the implementation of the transfer functions:

- \( f(d(p)) \) implemented as a **product of exponentials** with a maximum away from the front and asymptote to 0 at \( r_s \). citeturn37view0  
- \( g(v)=v/v_0 \) linear. citeturn37view0  
- \( h(z_{ei}) = z_{ei}^2 \) where \( z_{ei}(p)=(z_i(p)-z_t)/(z_c-z_t) \). citeturn37view0  

| v5 architecture location | What v5 does | Divergence vs thesis | Impact |
|---|---|---|---|
| **5.1 f(d)** | “Piecewise cubic distance profile” (no explicit formula in v5). | Thesis uses a specific exponential product; if you don’t match the “maximum at intermediate distance” behavior and decay profile, uplift shapes will differ. citeturn37view0 | **Risky** (visual signature differences; may also alter collision/subduction balance if uplift feeds later logic). |
| **5.1 h(z̃)** | v5 doesn’t specify the exact normalization and square used in thesis. | If you omit normalization by \(z_t, z_c\) and square-law emphasis, terrane/plateau relief will not amplify uplift as intended. citeturn37view0 | **Risky** (uplift may look “flat” or be too sensitive). |

### Slab pull axis update is unusually specific in the thesis

The thesis defines slab pull as a front-accumulated influence that **reorients the subducting plate’s rotation axis** toward the front:

\[
w_i(t+\delta t)=w_i(t)+\varepsilon\sum_k \frac{c_i\times q_k}{\|c_i\times q_k\|}\,\delta t
\]

where \(c_i\) is the plate centroid and \(q_k\) are points on the subduction front; it notes \(\varepsilon \ll 1\) so single points have little influence but extended fronts do. citeturn37view0  

| v5 architecture location | What v5 does | Divergence vs thesis | Impact |
|---|---|---|---|
| **3.1 per-step loop (“apply slab pull correction to rotation_axis”)** | Mentions slab pull correction but does not specify an axis update rule or how front samples qk are chosen/weighted. | The thesis’ slab pull is not “generic torque accumulation”; it’s this *very* particular “sum of normalized cross products toward front points” update. If you derive a more physical torque model, you will diverge. citeturn37view0 | **Risky**: plate trajectories may diverge rapidly from the reference behavior; this affects where and when convergence boundaries form. |

### Non-obvious thesis behavior not present in v5 subduction spec

| Thesis behavior | Where the thesis states it | Why it matters | v5 status | Impact |
|---|---|---|---|---|
| **Subduction trench rendering uses two elevations per vertex**: one “real” elevation for rendering (trench) and one “historical” elevation preserved for later uplift computations after the triangle goes under the overriding plate. | The thesis describes that subducting oceanic triangles near the convergence front are turned into trenches for rendering, while the “historical” elevation is preserved to compute uplift after the triangle passes under and is no longer visible. citeturn9view4 | If you use the trench-modified elevation as the input to uplift (instead of preserving pre-trench topography), you will systematically underdrive uplift or change its character. | Not described in v5. | **Critical** for “looks right” subduction mountains/trenches. |
| **Convergence tracking is triangle-wise and neighbor-propagated** (tracking lists seeded from boundary triangles, expanding to neighbors when triangles become active). | “Suivi… liste initialement créée à partir des triangles frontières… puis voisins… récursivement.” citeturn14view1turn9view4 | This is a stability mechanism: it constrains expensive intersection tests to a moving band near boundaries and defines how subduction propagates across the plate mesh. | v5 is sample-wise with later BFS. | **Risky**: you may get different front morphology and different performance characteristics. |

## Continental collision divergences

Your v5 collision event formulas (Δz quartic falloff, centroid-based fold direction, influence radius scaling) are closely aligned with the thesis’ *model equations*. citeturn37view0  
The divergences are mainly in **event triggering, terrane membership definition, and sequencing**.

### Triggering and “what counts as collision” differs in implementation detail

| v5 architecture location | What v5 does | What the thesis does | Why this is a divergence | Impact |
|---|---|---|---|---|
| **2.7 collision tracking + 3.2 Phase 9** | Collision threshold is reached when a tracked terrane/opposing pair’s **interpenetration distance** (overlap depth) exceeds 300 km, with a per-step estimate and resampling-time exact measurement. | Collision acceptance is based on at least one triangle exceeding a **distance-to-front** threshold (300 km) in the convergence tracking framework. citeturn9view4 | “Overlap depth” and “penetration distance-to-front (state variable)” are not equivalent, especially under tangential motion or complex overlap. | **Risky → Critical**: collision may fire too early/late or in the wrong places. |
| **5.2 Phase A** | Treats continental–continental overlap as “forced subduction” leading to collision after the distance threshold. | Explicitly allows **obduction** in continental–continental cases, and states collision happens “if and only if” the encountered opposing continental mass is significant; otherwise obduction continues and terranes can be subducted. citeturn37view0 | If you don’t implement the “significant opposing continent mass” test, you will over-trigger Himalayan collisions for small/insignificant contacts. | **Risky**: wrong frequency/scale of mountain belts; can become **critical** when it destabilizes plate motion via slab pull / suture changes. |

### Terrane detection details missing in v5 that affect collision correctness

| v5 architecture location | What v5 does | What the thesis does | Impact |
|---|---|---|---|
| **3.2 Phase 7 — Terrane detection** | Connected components over “continental samples” only (as written). | Terrane detection is connected components of continental triangles **plus a specific treatment to include trapped interior seas** (“mers intérieures”) inside terranes so that later collisions treat the resulting mass as one coherent continent. citeturn9view4 | **Critical**: without interior-sea inclusion, terranes can fragment unexpectedly, causing incorrect collision regions and later mis-collisions. |

### Collision scheduling differs (the thesis deliberately throttles)

The thesis explicitly states it stops after processing **one** such continental collision per timestep to avoid “edge effects” from intensive topology operations and to preserve performance. citeturn14view2  
v5’s Phase 9 description does not include this throttle (it iterates over tracked entries). This is a direct behavioral divergence.

- **Impact:** **Risky** (performance + determinism) and can become **Critical** if multiple collisions in one resampling window produce compounded topology/ownership changes that your pipeline cannot reconcile cleanly.

## Plate rifting and oceanic crust generation divergences

### Rifting trigger probability is not the thesis’ model

| v5 architecture location | What v5 does | What the thesis does | Why this is a divergence | Impact |
|---|---|---|---|---|
| **5.4 Plate Rifting (trigger)** | Uses a Poisson-derived-looking expression \(P=\lambda e^{-\lambda}\) with \(\lambda=\lambda_0 f(x_P)A/A_0\). | Uses a **Bernoulli** random variable with parameter \(p_i=\min(1,\,p\,x_C\,A/A_0)\) (with a note that if parameters were constant the sum across plates would behave approximately like a Poisson process). citeturn10view0 | The event rate and scaling are different; \( \lambda e^{-\lambda} \) is the Poisson probability of **exactly one** event, not “at least one,” and does not match the thesis’ clamp-to-1 Bernoulli. | **Critical**: rifting frequency is a macro-control knob; wrong math shifts the whole tectonic regime (supercontinent suppression, ridge frequency, etc.). |

### Rifting geometry mostly matches, but movement initialization is underspecified in v5

The thesis’ rifting implementation fractures a plate into \(n\in[2,4]\) sub-plates using Voronoi cells with coherent-noise warping, **then gives the new plates globally divergent directions** by a defined construction based on the centroids. citeturn10view0  

v5 matches: n range, centroid picking, Voronoi partition, coherent noise warp. But v5 says “Assign random diverging geodetic movements” without the thesis’ direction formula:

- Thesis direction construction: compute for each new centroid \(c_j\) a vector \(w_{ij}=q_j\times c_j\) where \(q_j\) is derived from the other centroids (so motions are divergent), then normalize; initial divergence speed is “quelques millimètres par an” and later grows due to slab pull. citeturn10view0  

**Impact:** **Risky**. Random axes can accidentally create immediate convergence/transform pairs between newborn plates, which alters ridge geometry and can inject artifacts into the next resampling.

### Oceanic crust generation: ridge point and “two-boundary constraint” are missing in v5

| v5 architecture location | What v5 does | What the thesis does | Why this is a divergence | Impact |
|---|---|---|---|---|
| **3.2 Phase 4 + 5.3** | Uses distances \(d_\Gamma\) and \(d_P\) and a ridge projection point \(q\) (“projection onto ridge line”) but does not specify how that ridge line is constructed or how the two flanking plates are chosen beyond local tests. | For a vertex in a divergence zone, explicitly finds \(q_1\) = closest point on a plate boundary, then \(q_2\) = closest boundary point on a **different** plate; ridge point is approximated as the spherical midpoint \(q_\Gamma = R\frac{q_1+q_2}{\|q_1+q_2\|}\). citeturn10view0 | The “two nearest distinct plate boundaries” rule is the core geometric definition of the ridge location in the thesis’ implementation. If you don’t replicate it, ridge placement and blending weights will differ. | **Risky**: ridge offset/tilt artifacts; can become **critical** if ridge direction feeds later logic or if gaps are misclassified. |

### Divergence cadence and global remesh timing logic differs

The thesis sets the divergence/remeshing period \(\Delta T\) dynamically based on the maximum observed plate speed \(v_m\), interpolating between 32 Ma and 128 Ma. citeturn16view2  
v5 maps this to a fixed “every R steps” resampling cadence (with rationale notes), which is a simplification.

- **Impact:** **Cosmetic → Risky**. If your cadence is too frequent, you pay unnecessary cost and may constantly reset state (distance fields, collision thresholds). If too infrequent, gaps/overlaps grow larger and your boundary-gap repairs are exercised harder.

## Force model and motion integration divergences

### Slab pull and motion update: thesis gives a concrete axis update rule; v5 leaves it abstract

As noted above, the thesis specifies a front-point accumulation that updates the plate’s rotation axis using normalized cross products from the plate centroid to front samples. citeturn37view0  
v5 describes “apply slab pull correction,” but does not yet specify:

- how \(q_k\) front samples are selected and stored,
- whether contributions are normalized (as in the thesis),
- whether the update is scaled by \(\varepsilon\delta t\),
- whether angular speed \( \omega \) is also increased (“intensifier la vitesse…”) or only the axis is reoriented. citeturn37view0  

**Impact:** **Risky**. Slab pull is one of the few mechanisms in the model that makes plate speeds evolve meaningfully over time; if it diverges, your convergence/divergence regime diverges.

### Max plate speed v₀ appears as a parameter but enforcement is underspecified

The thesis defines \(v_0\) as the reference/maximum speed used in \(g(v)=v/v_0\). citeturn37view0  
v5 lists \(v_0 = 100 \text{ mm/year}\) as a key parameter, but the architecture does not specify an explicit **velocity cap / clamp mechanism** beyond appearing in transfer functions.

- **Impact:** **Risky**: “runaway” speeds will compress the divergence timing logic and amplify overlap artifacts (and may destabilize rifting probability if it depends on speed).

### Subduction/collision state reset across global remesh differs

The thesis explicitly states that after global remeshing, **subductions are reinitialized** because subducted triangles disappear, and the **subduction matrix is invalidated and rebuilt**. citeturn10view0turn12view3  
v5’s pipeline recomputes distance fields and tracking each resampling but does not explicitly state that subduction state is *dropped* as an invariant of remeshing.

- **Impact:** **Critical** if you assume continuity of subduction state across resampling while the thesis assumes discontinuity. This changes long-term evolution and can create “ghost subduction” influences that persist longer than intended (or the opposite: prematurely erased effects).

## Non-obvious thesis details that are easy to miss but matter

These are not “general advice”; they are thesis-specific mechanics that (if missing) frequently explain “it implemented but looks wrong.”

**The “distance-to-front” used in tracking is intentionally inexact.** The thesis explicitly chooses an overestimation update rule rather than computing nearest-front distances via mesh traversal, because traversal would be too expensive and complicated under multiple interacting plates. citeturn9view4  
If you compute an exact BFS distance field, you are not “just optimizing,” you are changing what the model means and what thresholds (like 300 km) correspond to.

**Subduction trench rendering preserves a hidden “historical elevation” for later uplift influence.** Each vertex in subduction effectively carries (at least) two heights: rendered trench height and the preserved pre-trench height used for later uplift when the subducted crust passes under the overriding plateau. citeturn9view4  
This is a classic source of “uplift looks dead / wrong” if not implemented.

**Collision terranes include trapped interior seas.** The thesis explicitly adds a special treatment so inland seas created during convergence are included inside terrane components for later collision logic. citeturn9view4  
If you skip this, terrane graphs can fragment in ways that look like random bugs.

**Only one continental collision is processed per timestep in the thesis implementation.** This is a deliberate stability/performance choice (“effets de bord” and topology thrash). citeturn14view2  
If your implementation can fire many collision events in one step/window, you will diverge in both performance and evolution.

**Global divergence remeshing explicitly ignores triangles already marked as subduction/collision during ray casting.** citeturn10view0turn15view0  
If your resampling samples from those triangles anyway (or if your interior-only soups create holes that force fallbacks), you can accidentally “resurrect” crust state from geometry that the thesis treats as non-authoritative.