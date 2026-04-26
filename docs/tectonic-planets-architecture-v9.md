# Procedural Tectonic Planets - Architecture Memo v9

April 2026 note: this memo records why the project pivoted into `v9`, but it is no longer the current architecture position.

The later advection, long-horizon, and divergent-fill audits showed that the `v9` ownership-authority model does not satisfy the actual product bar:

- anchored ownership kills visible continental drift
- aggressive ownership transfer restores drift metrics but destroys coherent plate regions and thin boundaries
- revived oceanization can become speckled instead of opening coherent basins

Use these documents as the current decision point instead:

- `docs/tectonic-architecture-failure-memo-2026-04.md`
- `docs/tectonic-minimal-acceptance-tests.md`

**Based on:** Cortial et al., "Procedural Tectonic Planets" (2019), thesis follow-up, and Aurous V6/V8 experimental results  
**Target:** Unreal Engine 5.7  
**Date:** March 2026

**Status:** Decision memo after thesis-regime falsification, mixed-triangle A/B/C, high-density spot checks, retention validation, and the implementation-faithfulness audit.  
**Purpose:** Freeze what has now been disproven, explain the actual failure mode, and define the next architecture that can plausibly recover boundary coherence.

---

## 0. Executive Summary

`v9` exists because the current ownership model has now failed enough tests in enough regimes that the project needs a new source of truth.

The hard findings are:

1. The current V6 branch can improve some proxy metrics, but it still fails the actual requirement: coherent plates with thin boundaries and meaningful tectonic events.
2. The `60k / 7` regime was a bad thesis comparison, but moving to `60k / 40` did not rescue the model. Boundary localization improved, but ownership churn remained catastrophic and collision count stayed at zero.
3. Mixed-triangle handling is not the deciding factor. Exclude-mixed, duplicate-whole, and partition-split all still collapse under the current ownership/query model.
4. Higher density helps somewhat but does not solve the problem cheaply or decisively. At `250k / 40`, first-remesh churn still remained around `0.50`, and solve cost became impractical for architecture iteration.
5. The implementation-faithfulness audit did not uncover a bounded thesis bug that rescues the system. Assigned ownership often failed to reconstruct into authoritative owner coverage at scale.
6. The final reconstruction-model spike then removed that excuse: when soup coverage was forced to `100%`, churn got worse rather than better. Coverage contradiction was real, but fixing coverage did not fix coherence. The authority model itself is the remaining failure.

The key audit result was initially the one that changed the architecture:

> At `60k / 40 / step 100`, only about `2.5%` of assigned samples were covered by their assigned plate's current query geometry, and only about `2.3%` were covered by the assigned plate's rebuilt soup geometry.

This means the system is maintaining two contradictory states:

- the canonical ownership field says a sample belongs to plate `P`
- the rebuilt geometry for plate `P` does not actually represent that sample spatially

That contradiction was serious, but it was not the whole story.

The final soup spike showed:

- `owner_soup_contained: 99.2% -> 100%`
- `owner_query_exact_hit: 97.5% -> 100%`
- `ownership_churn: 0.9455 -> 0.9776`

Meaning:

- self-coverage can be made effectively perfect
- yet ownership still collapses because competing plates also cover the same regions
- the remesh still re-decides ownership from scratch

So the final diagnosis is stronger than the original audit:

- the system first suffered from ownership/geometry contradiction
- but even when coverage contradiction is removed, competition-driven re-resolution still destroys coherence

Once this exists at scale, rotation and overlap turn it into:

- interior churn
- repeat synthetic resynthesis
- exploding `BoundaryMask`
- zero meaningful collisions

That is the `v9` decision:

- stop using the remesh query as the sole authority for ownership
- make canonical ownership the source of truth
- let explicit tectonic events change ownership
- keep query/remesh as evidence and projection, not authority

---

## 1. What Is Now Disproven

The following branches have enough evidence behind them that they should not be reopened casually:

### 1.1 `60k / 7` as a thesis comparison

This regime was structurally misleading:

- too few plates
- monopoly-prone participation
- poor boundary/interior ratio
- collisions starved

It is no longer a valid argument either for or against the thesis architecture.

### 1.2 Mixed-triangle policy as the main answer

At `60k / 40`, all three tested policies still produced catastrophic churn:

- exclude-mixed
- duplicate-whole
- partition-split

The policies changed coherence and leakage somewhat, but not enough to rescue ownership stability.

Conclusion:

- mixed-triangle policy matters
- but it is not the primary blocker

### 1.3 Small local geometry fixes

The following have now been tried and are no longer the leading path:

- local support duplication
- synthetic-region fan support
- bounded nearest-owner/near-miss thinking
- whole-triangle duplication on the current retained V6 stack

These may move some metrics, but they do not restore coherent ownership.

### 1.4 Density as the primary answer

Density helps, but not enough:

- `250k / 40` reduced first-remesh churn versus `60k / 40`
- but churn remained far too high
- coherence did not improve enough to justify treating density as the clean solution
- solve cost became prohibitive for iterative architecture work

Conclusion:

- density is a stress lever, not the core fix

### 1.5 Thesis-faithfulness bug hunt as the main path

The implementation-faithfulness audit checked the strongest remaining suspects:

- per-timestep BVH rebuild
- ray/query semantics
- cap pruning
- query vs soup mismatch
- reconstruction coverage

The conclusion was not:

- "one small bug explains everything"

The conclusion was:

- assigned ownership is not reconstructing into owner geometry at all
- the authority model itself is broken for this bar

---

## 2. The Actual Failure Mode

The project originally framed this as "boundary coherence."

That is true as a symptom, but too shallow as a cause.

The real failure is:

1. canonical samples are assigned an owner
2. plate geometry is rebuilt from reconstruction rules that may or may not preserve that ownership spatially
3. even when coverage is repaired, multiple plates still geometrically compete for the same samples
4. rotated geometry is queried as if it were authoritative ownership truth
5. ownership is re-decided from scratch every remesh
6. misses, overlaps, and reassignments destroy plate interiors
7. the boundary field expands until "boundary" covers much of the planet

The current model is therefore mixing two incompatible ideas:

- **sample-level ownership**
- **geometry-level ownership reconstruction**

without guaranteeing that they represent the same plate after each remesh.

One manifestation of this was geometric orphaning:

- `OwnerPlateId = P`
- but no rebuilt plate-`P` geometry actually covers them

But the final spike established that the deeper problem is not orphaning alone.

It is:

- **query-driven competition as ownership authority**

Even with full self-coverage, the model still asks all overlapping candidates to compete again every remesh. That is what makes churn catastrophic by construction.

This is deeper than "rotation gaps."

Rotation creates the pressure.  
The ownership/reconstruction mismatch makes the pressure fatal.

---

## 3. The `v9` Architecture Position

`v9` treats ownership as topological state, not as a byproduct of remesh coverage.

### 3.1 Source of truth

The authoritative ownership state lives on the canonical global sample field:

- each canonical sample has an `OwnerPlateId`
- crust/material state is attached to that owner
- ownership persists until an explicit tectonic rule changes it

Per-plate geometry is no longer the source of ownership truth.

It becomes:

- a derived representation
- a query surface
- a transfer/evidence structure
- an event-detection aid

### 3.2 Authority order

Ownership decisions must use this precedence:

1. explicit tectonic event rules
2. existing owner persistence
3. query evidence only inside explicit active zones

What must disappear is the current implicit rule:

> remesh query miss => ownership loss => generic fill/reassignment

That rule is the engine of churn.

### 3.3 Maintenance vs topology change

`v9` makes a distinction that earlier project phases already hinted at:

- **maintenance** keeps ownership coherent
- **events** change ownership

That means:

- periodic remesh does not globally rewrite ownership
- divergence creates new crust explicitly
- convergence/subduction changes ownership explicitly
- collision transfers/sutures explicitly
- rifting repartitions explicitly

Ownership should not change just because a rotating mesh fails to cover a fixed point on one remesh pass.

---

## 4. Core Invariants

The following are not "nice to have." They are architecture requirements.

### 4.1 Interior stability

Plate interiors stay with their owner by default.

Interior ownership changes should be rare and attributable to named tectonic causes.

### 4.2 Narrow active boundaries

Boundaries are thin active zones, not broad residual regions of query failure.

If `BoundaryMask` expands across interiors, the architecture is failing.

### 4.3 Explicit divergence

New oceanic crust is created because an explicit divergence zone says so, not because any generic miss occurred.

### 4.4 Explicit convergence/collision

Subduction, collision, and terrane transfer happen because plate-pair event logic says so, not because noisy overlap or miss behavior happens to imply them.

### 4.5 Geometry follows authority

If a sample is owned by plate `P`, rebuilt plate geometry must be a projection of that ownership field.

The system may use geometry to refine and detect local tectonic behavior, but geometry must not contradict the authoritative ownership field at scale.

### 4.6 Events must be real

By step `200` of the standard harness, collisions and other tectonic events must be nonzero and meaningful.

Zero-collision worlds are not acceptable tectonic baselines.

---

## 5. What Query Still Does

`v9` does not throw away the thesis machinery.

The Eulerian query still has value, but it is demoted.

### 5.1 Valid uses of query geometry

- detect plate contact and separation
- detect overlap and convergent geometry
- estimate divergence bands
- transfer fields from moved geometry where appropriate
- support event detection and local deformation context

### 5.2 Invalid use of query geometry

The remesh query must no longer be allowed to do this globally:

- redefine ownership of quiet interiors
- convert generic misses into ownership loss
- imply open ocean outside explicit divergence logic

This is the central `v9` break from the current model.

---

## 6. Ownership-Change Mechanisms

`v9` only allows ownership changes through explicit causes.

Every ownership change should be attributable to one of:

- divergence creation
- subduction override/consumption
- collision transfer/suture
- rift repartition
- explicit narrow-band boundary migration

What should approach zero is:

- ownership changed because the generic remesh query lost coverage

If that bucket remains dominant, `v9` has failed.

---

## 7. State Model

At minimum, each canonical sample needs:

- `OwnerPlateId`
- crust/material state:
  - continental weight
  - crust type
  - age
  - thickness
  - elevation
  - ridge/fold directions
- boundary/event classification

Recommended debug/provenance state:

- `LastOwnershipChangeCause`
- `LastOwnershipChangeStep`
- `CoverageDeficitCount`
- `LastPlatePairContext`

Per-plate meshes remain derived caches:

- member samples
- rebuilt geometry
- BVH
- frontiers
- event-support structures

---

## 8. Operational Loop

The `v9` loop should be:

1. Advance rigid plate motion.
2. Rebuild or refresh per-plate query/event geometry.
3. Detect active tectonic zones by plate-pair motion/contact.
4. For each canonical sample:
   - outside active zones: keep owner
   - in divergence zone: create/assign new oceanic crust explicitly
   - in convergent/subduction zone: apply explicit override/transfer rules
   - in collision zone: apply explicit collision transfer/suture rules
   - in rift zone: repartition explicitly
5. Rebuild per-plate geometry from the authoritative ownership field.
6. Run diagnostics and export.

The key difference from the current model is that ownership persistence is the default, not the fallback.

---

## 9. Rollout Strategy

`v9` should not be attempted as one giant rewrite.

### Phase 1: Ownership authority

Implement a mode where:

- ownership is preserved outside explicit active tectonic zones
- generic miss no longer revokes ownership
- coverage deficits are logged, not converted into topology change

This phase should already answer whether the new authority model can keep interiors quiet.

#### Phase 1 design requirement: active-zone detection must be narrow and causal

This is the hardest design constraint in `v9`.

If active zones are too broad:

- everything becomes "active"
- ownership still churns
- `v9` degenerates back into query-driven rewriting

If active zones are too narrow:

- the world freezes
- divergence/collision/subduction stop changing topology

Therefore Phase 1 must define active tectonic zones explicitly from plate-pair evidence, not from generic ownership noise:

- divergent pair with separating motion and frontier proximity
- convergent/subducting pair with compressive motion and persistent contact
- collision pair with persistent continental contact
- explicit rift footprint

The wrong trigger is:

- "sample has multiple geometric candidates" or
- "sample was a miss"

Those are evidence inputs, not zone definitions.

Phase 1 is only valid if the active-zone classifier remains narrow enough that most of the planet is unambiguously inactive most of the time.

### Phase 2: Explicit divergence ownership

Move all new crust creation into explicit divergence-band logic.

### Phase 3: Explicit convergence/collision ownership

Move ownership changes in convergent zones into event logic rather than generic remesh resolution.

### Phase 4: Explicit rifting ownership

Keep rift repartition explicit and deterministic.

### Phase 5: Cleanup and calibration

Only after ownership authority is stable should tuning/performance become primary again.

---

## 10. Acceptance Gates

`v9` is only acceptable if all of the following improve materially:

### Boundary coherence

- `interior_leakage`
- `mean_conflict_ring_dist`
- `BoundaryMask` stays thin and structured

### Ownership stability

- `ownership_churn_fraction` drops sharply
- interior churn becomes rare
- `OwnershipChurnMask` darkens across interiors

### Synthetic stability

- `repeat_synthetic_miss` drops
- `MissLineageMask` stops whitening across plate interiors

### Tectonic meaning

- nonzero collision count by step `200`
- nonzero meaningful subduction/convergent behavior
- no monopoly-driven pseudo-planet

### Visual truth

- `PlateId` remains legible
- continents stop perforating into archipelagos
- boundaries read as lines/zones, not planetary noise

---

## 11. Hard Failure Conditions

Abort or revise `v9` if:

- ownership still changes broadly without explicit event causes
- `BoundaryMask` still expands across interiors
- collisions remain absent by step `200`
- the world freezes and loses real tectonic change
- the architecture merely renames query-driven ownership loss rather than removing it

---

## 12. Final Position

The project has now gone as far as it reasonably can with:

- query-driven ownership
- bounded retention
- mixed-triangle experiments
- cadence tuning
- density spot checks
- thesis-faithfulness audits

The remaining path to coherence is not another local patch.

The remaining path is:

> canonical ownership is truth  
> tectonic events change truth  
> geometry/query provide evidence and projection

That is the `v9` charter.
