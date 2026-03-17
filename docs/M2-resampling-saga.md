# M2 Resampling Saga — Decision Log

## Problem Statement

At periodic resampling (every 10-60 simulation steps), canonical sample positions are checked against each plate's rotated containment mesh. Samples that hit a plate get barycentric interpolation from that plate's data. Samples that miss all plates ("no-hit") become gaps.

At 500k samples / 40 plates, ~20% of samples are no-hit at each resampling. The code was converting ALL no-hits to fresh oceanic crust (`ContinentalWeight = 0.0`), systematically destroying continents — visible as zig-zag boundary scars and Swiss-cheese holes after 2-3 resamplings.

## What We Know vs What We Infer

Each iteration is tagged:
- **CODE:** What the code changed
- **PROVED:** What the logs/results definitively showed
- **INFERENCE:** What we concluded but remains unproven

## Iteration History

### M2a-c: Initial Implementation (Interior-Only Soups)
- **CODE:** Only triangles where all 3 vertices belong to the same plate enter containment soups. Mixed boundary triangles excluded.
- **PROVED:** Continental destruction after 5 resamplings at 500k/40 plates. The resampling pipeline works mechanically but has a destructive gap-handling path.
- **INFERENCE:** The gap rate is too high and/or the gap handling is too aggressive.

### M2d: Per-Vertex Rotation
- **CODE:** Boundary triangle vertices rotated by their individual owning plate's rotation.
- **PROVED:** Geometric shearing — violates rigid body assumption. Worse than M2c.
- **INFERENCE:** You cannot rotate different vertices of the same triangle by different quaternions.

### M2e: Uniform Rigid Rotation of Boundary Triangles
- **CODE:** Boundary triangles assigned to one plate. ALL vertices rotated by that plate's rotation.
- **PROVED:** gap_rate=0.1883 (vs 0.196 interior-only). Marginal improvement only.
- **INFERENCE:** Under rigid whole-triangle assignment, boundary triangles move into the plate's interior rather than filling boundary gaps. This specific assignment model does not materially reduce no-hit count.

### M2f: Interior-Only + Policy Fixes
- **CODE:** Back to interior-only. Added: self-copy for non-divergent gaps (preserve existing canonical data). Continent-biased overlap scoring (100 × continental_count - oceanic_count).
- **PROVED:** Stabilized continental area counts. BUT crescent arc artifacts at boundaries.
- **INFERENCE:** Self-copy preserves counts but creates visual artifacts because it uses stale canonical data (not updated by plate motion). The data source for gap-filled samples matters.

### M2g: Plate-Switch Guard
- **CODE:** Neighbor-majority filter for single-containment samples that would switch plates.
- **PROVED:** Tests pass, visual arcs persist at 60k.
- **INFERENCE:** Cosmetic fix, doesn't address the structural gap problem.

### M2h: Ghost Vertex Duplication
- **CODE:** Boundary triangles duplicated into EVERY involved plate's soup.
- **PROVED:** Retention 0.8912. ~86k-94k hidden non-divergent fallbacks, ~66k-70k filtered overlaps.
- **INFERENCE:** Duplication causes overlap explosion that overwhelms overlap resolution.

### M2i: True Global TDS Partition
- **CODE:** Every global triangle assigned to exactly one plate's soup. No drops, no duplicates. Majority-vertex rule. Plate-local carried samples for all assigned vertices.
- **PROVED:**
  - `total_soup_tris=999996` (matches global count), `dropped=0`, `duplicated=0`, `missing_carried=0`
  - `no_valid_hit_count` dropped from 105,810 to 101,897 (3,913 improvement, 0.8%)
  - Only 17,912 of 999,996 triangles (1.8%) are boundary
  - Stage 2 gate failed
- **INFERENCE:** Under rigid whole-triangle assignment, plate motion dominates the no-hit population. Simply reassigning mixed triangles does not fix containment. However, this does NOT rule out geometric subdivision/clipping of mixed triangles, alternative containment representations, or the possibility that Driftworld's interior-only model depends on details we haven't matched.

### M2j: Gap Classification
- **CODE:** Split no-hit samples into divergent (plates moving apart → oceanic) vs non-divergent (convergent/transform → assign to nearest plate with fresh carried data via triangle projection). Snapshot-based classification to avoid order-dependent mutations.
- **PROVED:**
  - Non-divergent population is real: 7,823 samples (7.7% of no-hits), all recovered via triangle projection
  - Only 1,616 non-divergent samples were continental
  - Divergent continental gaps: 19,745 — the dominant loss channel
  - First-resampling retention: 0.8698 (below 0.90 target)
- **INFERENCE:** Gap classification is correct engineering but insufficient alone. The remaining loss is from divergent continental gaps — genuine rift zones where continents are splitting.

### M2k: Containment Recovery via FindNearestTriangle
- **CODE:** After exact PIT containment misses, query `TMeshAABBTree3::FindNearestTriangle()` across all plate soups. If nearest triangle distance < `ContainmentRecoveryTolerance` (0.003), recover the sample as a containment hit with clamped/renormalized barycentrics. Unrecovered samples flow into M2j classification. Full accounting invariant asserted.
- **PROVED:**
  - Recovery is material: 6,194 samples recovered (6.1% of no-hits)
  - **Recovery is overwhelmingly oceanic**: `recovery_continental_count=109` out of 6,194
  - Near-edge containment misses are NOT the cause of continental destruction
  - true_gap_count reduced from 101,897 to 95,703
  - First-resampling retention: 0.8704 (+0.0006 from M2j)
  - Compounding benefit: step 50 continental 86,081 vs M2j's 80,468 (+7%)
  - Distance distribution: mean=0.001397, p95=0.002818, max=0.003000
- **INFERENCE:** The 19,745 divergent continental gaps are in wide gap zones far from any plate's geometry, not near-edge containment misses. The remaining continental loss is physical — divergent boundaries creating oceanic crust where continents split. M3 subduction uplift is the mechanism that balances this loss.

## Final State (M2 Complete)

M2 resampling is mechanically correct. The pipeline:
1. Exact Shewchuk PIT containment (primary)
2. FindNearestTriangle recovery within tolerance (near-edge misses)
3. Gap classification: divergent → oceanic, non-divergent → nearest plate projection
4. Overlap resolution: continent-biased scoring

Continental retention per resampling: **0.87** at 500k/40 plates. The remaining ~13% loss is physical divergent-boundary rifting. Continental creation (M3 subduction uplift, M4 collision) is required to balance this.

## Reference Model (Validated)

### Paper (Cortial et al. 2019)
- Global spherical Delaunay triangulation computed once, reused
- Plates rebuilt by "partitioning the triangulation according to sample assignments"
- Samples either "intersect a plate" (interpolation) or "fall between diverging plates" (oceanic generation)
- Mixed-triangle owner rule is underspecified

### Thesis (Cortial)
- Per-plate sub-triangulations extracted by "dupliquer, ré-indicer et re-compacter" (duplicate, reindex, compact)
- Whole-triangle submesh model, not clipped-boundary
- Chapter 4 ghost vertices are for LOD rendering subdivision, NOT tectonic remeshing

### Driftworld (hecubah/driftworld-tectonics)
- **Interior-only triangles** (Planet.cs#L842, #L1158)
- Mixed triangles excluded. No assignment, no duplication, no splitting.
- ALL no-hit samples → divergent gap → oceanic crust (CSVertexDataInterpolation.compute#L291)
- Does NOT resample periodically — only on collision/rift events
- Starts all-oceanic (InitialContinentalProbability=0.0); continents created by M3+ subduction/collision
- Has BVH miss-recovery: PIT tolerance (0.001f) + nearest-barycenter walk
- Author consulted directly with Cortial

## Approaches Ruled Out (with scope)

1. **Per-vertex rotation** — violates rigid body (M2d). Definitively ruled out.
2. **Ghost vertex duplication into all plates** — overlap explosion (M2h). Ruled out in current form.
3. **Rigid whole-triangle assignment of boundary triangles** — does not materially reduce no-hit count (M2e + M2i). Ruled out for this specific model.
4. **Stale canonical self-copy for gaps** — creates crescent arc artifacts (M2f). Ruled out as data source.
5. **Near-edge containment recovery as continental fix** — recovery is real but overwhelmingly oceanic; only 109 continental samples of 6,194 recovered (M2k). Not the fix for continental loss.
6. **Gap classification alone** — non-divergent population is real but too small to change retention (M2j). Correct engineering, not sufficient alone.

## Approaches NOT Ruled Out (Deferred to M3+)

1. **Accept gap rate, compensate with M3+** — subduction/collision add continental area back. This is the current plan. **SELECTED.**
2. **Geometric subdivision/clipping of mixed boundary triangles** — not reference-supported, not tested. Engineering fallback if M3+ compensation is insufficient.
3. **Per-plate local Delaunay for containment** — not reference-proven. Fallback if needed.

## Key Metrics (500k / 40 plates, step 10)

| Metric | M2c (interior-only) | M2i (full partition) | M2j (gap classification) | M2k (containment recovery) |
|---|---|---|---|---|
| exact_single_hit_count | — | — | — | 373,273 |
| exact_multi_hit_count | — | — | — | 24,830 |
| recovery_containment_count | — | — | — | 6,194 |
| true_gap_count / no_valid_hit | 105,810 (21.2%) | 101,897 (20.4%) | 101,897 (20.4%) | 95,703 (19.1%) |
| divergent_gap_count | — | — | 94,074 | 88,865* |
| non_divergent_gap_count | — | — | 7,823 | 6,838* |
| continental_step_10 | — | — | 131,149 | 131,250 |
| first_resample_retention | — | — | 0.8698 | 0.8704 |
| continental_step_50 | — | — | 80,468 | 86,081 |

*Estimated from true_gap_count reduction; exact split not reported for M2k true gaps.
