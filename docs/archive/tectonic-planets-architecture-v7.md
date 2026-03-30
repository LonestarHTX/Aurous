# Procedural Tectonic Planets - Architecture Memo v7

**Based on:** Cortial et al., "Procedural Tectonic Planets" (2019)  
**Target:** Unreal Engine 5.7  
**Date:** March 2026  

**Status:** Decision memo after `v6` Phase 3 and paper reread.  
**Purpose:** Decide whether boundary persistence can be recovered inside the current periodic-resolve model or whether boundaries must become first-class state.

---

## 0. Executive Summary

The main problem is still boundary persistence.

The project has now established four facts:

1. Clean step-0 boundaries degrade under repeated periodic ownership rewriting.
2. `v6` Phase 3 can reduce overlaps, recoveries, and fragmentation dramatically, but it still consumes the boundary zone and destroys continental balance.
3. The paper does not rely on a generic periodic solver to preserve boundaries as a side effect. It keeps boundaries alive through explicit local tectonic processes: ridge generation, subduction, collision, terrane transfer, and rifting.
4. The current implementation already duplicates boundary triangles into `v6` query geometry, so the remaining failure is not explained by "boundary triangles are missing" alone.

The most likely remaining implementation defect is a **query/transfer asymmetry**:

- `v6` query geometry sees duplicated unsplit boundary triangles on every incident plate
- carried-state transfer is still rebuilt from single-owner soup assignment

This means ownership is queried against multi-owner boundary geometry, but material is still transferred from one-owner carried state. That is a strong candidate for cumulative boundary blur.

This creates the `v7` fork:

- **Option A:** make boundaries first-class state and explicit local processes
- **Option B:** run one small falsification experiment first by making boundary transfer symmetric with boundary query geometry

**Recommendation:** run Option B first. If it fails, stop tuning the generic periodic boundary solver and move to Option A.

---

## 1. What `v6` Actually Proved

`v6` Phase 3 proved something important, but not what was originally hoped.

At `60k / 7 / seed 42`, Phase 3 improved topology-style metrics sharply:

- step-200 overlaps fell from roughly `14428` to `1831`
- step-200 coherence reassignment fell from roughly `2260` to `60`
- step-200 max components fell from roughly `55` to `4`

But the same run also showed the real failure:

- step-50 boundary decisions: about `32718`
- step-200 boundary decisions: about `2544`
- step-200 continental area fraction: about `0.0746`

Interpretation:

- the solver got much better at forcing every sample to a stable final owner
- the map got much worse at keeping a living boundary zone

So `v6` Phase 3 did **not** prove that the boundary problem is solved. It proved that aggressive explicit resolution can clean up ownership while still killing the tectonic frontier.

The problem is no longer "can `v6` decide ownership?"  
The problem is "can `v6` keep boundaries alive while doing so?"

---

## 2. What the Paper Actually Does

The paper is not organized around a single generic ownership solver that maintains all tectonic structure by itself.

Instead, it treats the major boundary phenomena as explicit local processes:

- **oceanic crust generation** at divergent ridges
- **subduction** at convergent oceanic boundaries
- **continental collision** with terrane transfer
- **plate rifting** as an explicit fracture event
- **boundary triangle tracking** for detecting interactions between plates

This matters because the paper does not appear to ask a generic periodic interpolation pass to preserve sharp plate boundaries on its own.

The periodic crust update is part of the model, but the tectonically meaningful boundary behavior is kept alive through named mechanisms. Boundaries are not just "whatever is left over after every sample got a final ordinary owner."

This is the biggest architectural mismatch between the paper and current `v6`.

Current `v6` asks one periodic solver to do all of the following at once:

- resolve interior ownership
- resolve boundary ownership
- decide divergence/convergence outcomes
- keep the frontier visually legible
- preserve continental structure

The paper splits those responsibilities much more explicitly.

---

## 3. Clarification: Boundary Triangles Are Not Simply Missing

One tempting explanation was that the project created a dead zone by dropping boundary triangles from plate soups.

That is **not** the full story in the current code.

The current implementation already does important boundary-triangle work:

- legacy soup assignment gives every triangle some assigned plate owner
- boundary triangles are also tracked per involved plate
- `v6` query geometry duplicates unsplit boundary triangles onto every incident plate

Therefore, the current failure cannot be reduced to:

> "Just include boundary triangles and the problem goes away."

That experiment is effectively already half in the code path.

What is still asymmetric is **transfer**, not **query**.

---

## 4. The Query/Transfer Asymmetry

This is the clearest concrete defect still visible.

### 4.1 Query side

`v6` query geometry duplicates boundary triangles onto each incident plate, so containment tests can legitimately see overlapping plate claims near boundaries.

This is good and paper-aligned.

### 4.2 Transfer side

Carried-state rebuilding still comes from the plate soup ownership assignment, which is effectively single-owner material state.

This means the runtime can do the following:

1. hit a duplicated boundary triangle during ownership resolution
2. decide a final owner based on that boundary-aware geometry
3. transfer fields from carried data that does not symmetrically represent the same boundary geometry

That mismatch can blur threshold-sensitive fields and consume boundary structure even when the ownership decision itself is "correct."

This is consistent with the observed failure pattern:

- ownership metrics improve
- boundary persistence does not
- continental weight still drifts downward
- the frontier collapses instead of remaining a clean active band

### 4.3 Meaning

Before concluding that global periodic resampling is impossible, the project should first test whether this asymmetry is the real remaining bug.

If the answer is yes, the architecture survives.

If the answer is no, then the problem is deeper: the generic periodic-solve representation itself is wrong for boundaries.

---

## 5. Boundary Persistence Is the Actual Requirement

The project has repeatedly drifted toward secondary metrics:

- continental area fraction
- overlap count
- zero-hit count
- coherence reassignment count
- component count

These are useful diagnostics, but they are not the root requirement.

The actual requirement is:

> boundaries should remain clean, legible, and geologically structured over repeated solves, closer to the clean step-0 state than to the degraded late-run `v5` and `v6` states.

That implies:

- the boundary zone must not collapse toward zero
- the frontier must not turn into a blurred interpolation band
- the solver must not consume contested structure faster than tectonic processes replenish it

If a design improves overlap counts while erasing the boundary field, it has failed the real requirement.

---

## 6. The `v7` Fork

`v7` is not "another threshold tuning pass."

It is an architecture decision with two credible paths.

### 6.1 Option A: Explicit Boundary State

Make boundaries first-class state instead of hoping they emerge from generic sample resolution.

Core idea:

- periodic solve handles **interior** samples
- a **boundary corridor** is explicitly preserved or rebuilt each solve
- explicit local processes maintain that corridor:
  - ridge generation
  - subduction
  - continental collision / terrane transfer
  - rifting

Consequences:

- boundary samples are no longer ordinary samples that should be fully resolved away
- boundary thickness, continuity, and process identity become deliberate state
- the design moves closer to the structure suggested by the paper

Pros:

- aligned with the actual tectonic phenomena in the paper
- directly targets the boundary persistence requirement
- avoids asking one generic solver to carry too many responsibilities

Cons:

- larger rewrite
- introduces explicit boundary-state machinery
- harder to falsify quickly

### 6.2 Option B: Symmetric Boundary Transfer Falsification Test

Keep the `v6` periodic ownership solver, but repair the asymmetry between query and transfer.

Core idea:

- query geometry already duplicates boundary triangles across incident plates
- carried samples / transfer sources should do the same, or otherwise provide equivalent boundary-local source data
- retained/reassigned boundary samples must transfer from boundary-aware data, not generic single-owner fallback

This is the smallest honest experiment still available.

Pros:

- small implementation compared to Option A
- directly tests whether the remaining failure is plumbing rather than architecture
- gives a clean go/no-go signal

Cons:

- may still fail if generic periodic boundary resolution is fundamentally too destructive
- does not solve the paper mismatch if explicit boundary state is actually required

---

## 7. Recommended Path

### 7.1 Immediate recommendation

Run **Option B first**.

Reason:

- it is the smallest remaining falsification test
- it targets the clearest concrete defect still visible
- it can settle whether the current architecture is still viable without jumping straight to a larger redesign

### 7.2 Hard rule after Option B

If symmetric boundary transfer does **not** materially restore boundary persistence, stop tuning the generic periodic boundary solver.

At that point, `v7` should move to **Option A**:

- explicit boundary corridor
- explicit boundary-local processes
- interior solve separated from boundary maintenance

No more threshold tuning should happen before that decision.

---

## 8. Minimal Option B Slice

The smallest useful `v7` experiment is:

1. keep the current `v6` ownership/query path
2. make boundary transfer sources symmetric with the duplicated boundary query geometry
3. ensure boundary-retained and boundary-reassigned samples can transfer from boundary-local sources
4. avoid degrading threshold-sensitive continental state through single-owner fallback transfer

This should be treated purely as a falsification slice.

It does **not** need:

- actor/editor integration
- 500k tuning
- final process-complete rifting/subduction/collision behavior

It only needs to answer:

> If boundary query and boundary transfer are finally symmetric, do clean persistent boundaries survive materially better?

---

## 9. Success Gate for `v7`

`v7` should be judged primarily by boundary persistence, not by raw solver neatness.

### 9.1 Primary visual gate

By step `200`, and especially step `300/400`:

- boundaries still read as continuous tectonic frontiers
- continents remain coherent landmasses
- the frontier does not collapse into silence
- the map does not look cleaner only because the boundary band was consumed

### 9.2 Secondary support metrics

These metrics support the visual call:

- continental area fraction
- boundary decision sample count
- boundary sample fraction
- overlap count
- zero-hit count
- coherence reassignment count
- max components per plate
- `CW` threshold-mismatch counts

### 9.3 Failure condition

`v7` fails if:

- symmetric boundary transfer still leaves boundaries collapsing over repeated solves
- the boundary zone still shrinks away instead of remaining a persistent structured frontier
- the map remains dependent on consuming ambiguity rather than preserving tectonic structure

If that happens, the project should stop trying to save generic periodic boundary resolution and move to explicit boundary state.

---

## 10. Final Position

The project is no longer stuck because it lacks another clever threshold tweak.

It is stuck because the architecture question is now clear.

The paper suggests:

- periodic crust updates are real
- but tectonic boundaries remain alive through explicit local processes and tracked boundary structure

Current `v6` instead asks one generic periodic solver to preserve boundaries by itself.

That has not worked.

The cleanest path still open is:

1. run one final falsification test by making boundary transfer symmetric with boundary query geometry
2. if that fails, promote boundaries to first-class state and build the next architecture around explicit boundary-local processes

That is the `v7` decision.
