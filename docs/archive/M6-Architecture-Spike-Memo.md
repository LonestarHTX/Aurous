# M6 Architecture Spike Preparation

**Goal:** Prepare the smallest credible architecture spike if isolated CW-transfer falsification (`M6aj`) fails to fix the step-300/400 map degradation.

## 1. Spike Options

**Spike A: Periodic Global Authoritative Resampling**
*   **Pros:** Directly aligns with the original Cortial paper. Provides the cleanest architectural falsification by testing if the rigid-triangle `v5` specification (unsplit boundary triangles, resampling every `R=10` steps) natively prevents boundary degradation. Lowest implementation effort (mostly involves deleting code).
*   **Cons:** Re-exposes the simulation to global ownership churn, risking the boundary degradation observed in `v4` and `M1-M5` if the base containment geometry isn't flawlessly robust.

**Spike B: Event-scoped Authoritative Resampling**
*   **Pros:** Honors the core `M1-M5` lesson ("ownership churn was the real boundary problem"). Preserves the stable interiors of plates and cleanly isolates topology changes to active rifts and subduction/collision zones.
*   **Cons:** Significantly higher implementation effort. Requires complex new logic to isolate local patches, resample them, and seamlessly stitch them back into the preserved global mesh without creating artificial tears or topological "no-man's-lands".

## 2. Keep vs Remove

**Core Systems to Keep:**
*   Plate motion, kinematics, and random initialization (Voronoi + noise).
*   Global triangulation/substrate and base structural seeding.
*   Containment soup construction (rigidly rotated, unsplit boundary triangles) and BVH generation.
*   Continuous geodynamic processes (erosion, oceanic dampening, uplift profiling).
*   Event triggers and tectonic classification logic (Poisson rifting, convergent overlap detection).

**Patch-Stack Systems to Remove:**
*   `PreserveOwnershipPeriodic` and any fallback/recovery containment queries.
*   Preserve-boundary previous-owner hysteresis (`TryResolvePreserveBoundaryPreviousOwnerHysteresis`).
*   Same-plate CW retention and threshold-crossing prevention patches (`PreserveOwnershipCWRetainFlags`, `FullResolutionSamePlateCWRetainedCount`).
*   Rift final-owner CW reconciliation, localized rollbacks, and stray fragment suppression.
*   Any cached boundary-contact persistence tracking intended to bridge `PreserveOwnership` iterations.

## 3. Recommended First Spike

**Recommendation:** Spike A (Periodic Global Authoritative Resampling)

**Defense:** Spike A is the only choice that cleanly answers the underlying architectural question: *Can the paper's barycentric resampling actually work at scale without a massive patch stack?* It has the smallest implementation effort because it consists entirely of deleting the `M1-M5` compensatory layers. It removes all dependence on the current patch stack, effectively returning the codebase to the purest interpretation of the `v5` specification. Conversely, Spike B requires building complex new localized-stitching infrastructure, which introduces new variables and risks masking the root architecture issue with new bugs.

## 4. Minimal Implementation Slice

**Smallest prototype worth building:**
*   **Bypass:** All ownership repair passes, hysteresis tracking, CW retention patches, and deferred event follow-ups.
*   **Keep:** The pure, unadulterated resampling loop. Every 10 steps: rigidly rotate plate interior soups -> query BVH -> assign ownership -> barycentric interpolation for hits -> oceanize divergent gaps.
*   **Simplifications:** Allow tectonic events (rifts, collisions) to execute strictly based on the raw overlapping/gapping geometry found at the global resample tick. Do not attempt to back-calculate precise mid-interval threshold crossings.
*   **What not to solve in v1:** Do not attempt to fix minor coastal aliasing or minor terrane drift. If the broad macro-structure holds together without collapsing, the spike is a success.

## 5. Success Gate

**Criteria for beating the current M6 line:**
*   **Which checkpoints to compare:** Step 300 and Step 400 maps.
*   **Maps that matter most:** `PlateId`, `CrustType`, and `Elevation`. They must show recognizable continents, distinct ocean basins, and stable plate boundaries without catastrophic stippling or jagged tearing.
*   **Metrics that matter:** 
    *   `continental_area_fraction` must stabilize within the 20–40% band (proving that global barycentric interpolation does not inherently erode continents to 0%).
    *   `gap_rate` stays strictly bounded < 10% and `overlap_rate` < 5% without any repair pipelines.
*   **Clear Win:** The spike demonstrates that the pure `v5` global resampling mechanism, stripped of all hysteresis and CW-retention hacks, maintains global continental balance and visual coherence better than the heavily patched `PreserveOwnershipPeriodic` baseline.
