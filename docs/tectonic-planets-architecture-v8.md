# Procedural Tectonic Planets - Architecture Memo v8

**Based on:** Cortial et al., "Procedural Tectonic Planets" (2019) and thesis follow-up  
**Target:** Unreal Engine 5.7  
**Date:** March 2026  

**Status:** Findings memo after `v7` diagnosis, copied-frontier identity fix, first-cycle attribution, and minimal tectonic-maintenance experiment.  
**Purpose:** Freeze what is now known, retire dead branches, and define the next implementation focus.

---

## 0. Executive Summary

`v8` is not a speculative architecture memo.

It records four hard findings:

1. The copied-frontier remesh now passes the zero-drift identity check.
2. Pure remeshing on the current fixed substrate has a real erosive land-budget bias under motion.
3. A minimal thesis-style tectonic maintenance pass can materially offset that erosion and keep continental area in a plausible range.
4. Boundary readability and plate coherence are still failing, primarily through overlap / multi-hit / winner instability rather than land-budget collapse alone, and the thesis-supported destructive subduction/collision exclusion is still missing.

That means the project is no longer asking:

> "Can remeshing alone preserve the planet?"

That question is answered:

- **No for land budget**
- **Not yet for coherence**

The project is now asking two narrower questions:

1. How should thesis-style tectonic maintenance be made less crude once the remesh is coherent enough to support it?
2. How should overlap / multi-hit / winner behavior be improved so boundaries stop fragmenting into noise, starting with thesis-style exclusion of destructive subduction/collision geometry during remesh?

**`v8` recommendation:**

- keep the copied-frontier remesh as the active testbed
- keep the maintenance pass as evidence that tectonic source/sink processes are load-bearing
- shift engineering effort to remesh coherence, with destructive subduction/collision exclusion as the next concrete target and multi-hit / winner-resolution immediately after

---

## 1. What `v7` Actually Established

`v7` was not a final solution, but it was not a bust.

It separated failure modes that had previously been conflated:

- stationary remesh correctness
- first-cycle remesh damage
- long-run cumulative degradation
- land-budget erosion
- fragmentation / coherence loss

### 1.1 Identity gate

The copied-frontier remesh originally failed a zero-drift identity test because exact vertex/edge ray hits were dropped by the raw radial ray path.

That bug is now fixed.

The copied-frontier remesh at zero drift now shows:

- `continental_area_fraction = 0.291033`
- `hit_count = 60000`
- `miss_count = 0`
- `oceanic_creation = 0`
- `transfer_fallback = 0`

Interpretation:

- the remesh pipeline is now mechanically trustworthy at zero motion
- later failures are not explained by an identity bug anymore

### 1.2 First-cycle damage

After one real drift interval plus one remesh, the copied-frontier path still degrades immediately:

- `continental_area_fraction = 0.257867`
- `miss_count = 10625`
- `oceanic_creation = 10625`
- `cw_threshold_mismatch = 88`
- `max_components_after = 106`

This proved that damage enters on the first real motion cycle.

### 1.3 First-cycle attribution

The first-cycle attribution split the damage into two different mechanisms:

- **land-budget erosion**
- **fragmentation / overlap chaos**

The first-cycle loss numbers showed:

- `continental_samples_lost = 5314`
- `continental_loss_from_oceanic_creation = 1141`
- `continental_loss_from_hit = 4173`
- `continental_loss_from_hit_multi_winner = 1009`
- `continental_loss_from_hit_single_winner = 3164`

The same run also showed:

- `fragmented_component_count = 452`
- `fragmented_components_touching_multi_hit_winner = 452`
- `fragmented_components_strongly_multi_hit = 451`

Interpretation:

- fragmentation is overwhelmingly a multi-hit / overlap / winner-resolution problem
- land-budget erosion is not mainly wrong-plate capture, cap rejection, or fallback
- most first-cycle land loss comes from same-plate interior resampling onto already subcontinental source material

That was the key change in understanding:

> pure remeshing is mechanically correct at rest, but still erosive under motion even when the "right" plate wins

---

## 2. What the Minimal Process Experiment Proved

The copied-frontier process experiment added a narrow convergent-margin maintenance pass on top of the remesh path.

The remesh contract itself stayed unchanged:

- direct hit-triangle transfer
- true miss oceanization
- zero fallback

The only new behavior was a post-remesh, post-repartition convergent-margin maintenance step using existing subduction-distance / subduction-speed information and Andean-style conversion rates.

### 2.1 First-cycle effect

At step 25:

- baseline net continental change: `-1990`
- process net continental change: `-1015`

Additional detail:

- gross remesh losses from oceanic creation: unchanged at `1141`
- gross remesh losses from hit-side outcomes: unchanged at `4173`
- gross remesh gains: unchanged at `3324`
- post-remesh tectonic recoveries: `975`

Interpretation:

- the process did **not** fix the remesh
- it offset remesh erosion afterward
- that is exactly the thesis-style claim that tectonic processes are maintenance terms, not mere decoration

### 2.2 Long-run effect

At step 200:

- remesh-only CAF: `0.099300`
- remesh + process CAF: `0.298667`

That is the first time in the project that continental area stayed in a plausible range rather than collapsing toward zero.

This is the strongest single architectural result so far.

### 2.3 What did not improve

The process pass did not solve remesh quality:

- step-200 `max_components_after`: `793 -> 850`
- step-200 `multi_hit_count`: `27254 -> 27468`
- step-200 `cw_threshold_mismatch`: `1290 -> 4012`

Interpretation:

- the process pass rescued land budget
- it did not rescue coherence
- it likely interacts badly with unresolved overlap chaos and currently overforces continental recovery in some regions

So the current process layer is **valid evidence**, not a final implementation.

---

## 3. The Current State of the Project

The project now has a much cleaner state description.

### 3.1 What is trusted

The following conclusions are now strong enough to build on:

- zero-drift copied-frontier remesh identity is fixed
- pure remeshing under motion is erosive for continental budget
- a tectonic maintenance source term can materially counteract that erosion
- long-run CAF collapse is not the right primary score anymore

### 3.2 What is still broken

The following remains unresolved:

- destructive subduction/collision exclusion during remesh is still absent
- overlap / multi-hit / winner-resolution under motion
- first-cycle fragmentation
- long-run boundary readability
- interaction between crude maintenance and unstable remesh outcomes

### 3.3 What has been retired

The following approaches should be considered non-viable or at least non-preferred:

- `v5` preserve / repair stacks as architecture
- current whole mixed-triangle copied-frontier duplication as a **final** frontier construction rule
- convex-hull rehulling as a thesis substitute
- any argument that remesh-alone should maintain continental budget in this model

Clarification:

- copied-frontier duplication remains the current working base because it is the least broken remesh representation available
- but it should be treated as an interim testbed, not as the intended final frontier construction

---

## 4. Updated Reading of the Thesis

The thesis now reads more clearly in light of the experiments.

The important thesis-level claim is not:

> remeshing alone should preserve everything

It is closer to:

> periodic remeshing carries the crust state, while tectonic processes maintain geological structure and budget through time

The project has now produced evidence consistent with that interpretation:

- remesh alone erodes
- tectonic maintenance helps

The thesis also gives a more specific answer to convergent-boundary overlap than the current implementation does:

1. **Pre-remesh BVH rebuild:** each plate rebuilds its boundary / query geometry while ignoring triangles in subduction
2. **Ray-test exclusion:** even if a ray still intersects a triangle marked as subduction or collision, that hit is ignored
3. **Post-remesh cleanup:** subducted triangles disappear during remeshing and subduction state is reconstructed afterward

The research reading also clarified that the thesis does **not** appear to classify destructive triangles statelessly from scratch each solve.
It tracks subduction / collision as persistent per-triangle state:

- convergent fronts seed tracking lists
- those lists propagate inward across neighboring triangles over time
- polarity determines which side is subducting
- collision is a distinct mode from oceanic subduction

That means the current `v8` implementation will likely need to approximate this in stages:

- near term: narrower stateless destructive classification using polarity, convergence strength, and continental/oceanic distinction
- later: tracked destructive-state infrastructure if the stateless approximation remains too broad

In current runs this still shows up as a live deviation:

- `destructive_filter_applied = 0`

That now looks less like a minor thesis mismatch and more like the most concrete missing ingredient for remesh coherence at convergent boundaries.

The thesis also handles divergence more specifically than the current blanket miss-oceanization path:

- on a true miss, it searches for the two nearest frontier points from different plates
- then generates structured new oceanic crust from that ridge context rather than just forcing `CW = 0` and a fixed deep-ocean elevation

More concretely, the thesis miss path is:

1. find nearest frontier point `q1`
2. find nearest frontier point `q2` on a different plate
3. compute spherical midpoint `q_gamma`
4. derive an interpolation factor from distance to the ridge midpoint vs nearest frontier
5. synthesize ridge-shaped oceanic crust:
   - `CW = 0`
   - `Age = 0`
   - structured elevation from border/ridge context
   - standard oceanic thickness
   - ridge direction from geometry rather than a generic fallback

The practical consequence is important:

- in the thesis, once coverage and destructive exclusion are correct, a miss is treated as structured divergence fill
- in the current code, miss handling is still much blunter
- so any future miss-heavy regime after destructive exclusion must be interpreted with that missing structured fill layer in mind

This is still a secondary deviation relative to the destructive-filter gap, but it remains a likely future quality improvement for miss behavior once convergent overlap is under control.

What is not yet validated is whether the current remesh representation is coherent enough to support that maintenance gracefully.

So the thesis is no longer primarily a question about whether tectonic processes matter.
That question is effectively answered.

The remaining thesis-alignment question is:

> how cleanly can the remesh handle overlap, winner choice, and plate coherence under motion?

---

## 5. `v8` Architecture Position

`v8` should be treated as a **two-track architecture**.

### 5.1 Track A: Tectonic maintenance is load-bearing

This is now accepted.

The model should retain a tectonic maintenance layer for land-budget stability.

That layer will eventually need:

- better calibration
- cleaner process boundaries
- tighter interaction with remesh and event timing

But the central question is over:

- **maintenance is necessary**

### 5.2 Track B: Remesh coherence is now the primary open technical problem

This becomes the next engineering priority.

The main open problem is not whether remeshing exists.
It is whether remeshing can produce stable, readable, geologically plausible ownership/material structure under motion.

Current evidence says the biggest remaining technical problem is:

- overlap / multi-hit / winner instability, especially because destructive subduction/collision geometry is still allowed to participate in remesh queries

This is the part still destroying boundary readability and fragmenting plates.

---

## 6. Immediate Plan

### 6.1 Keep

Keep the following as the active working base:

- copied-frontier remesh with zero-drift identity fix
- minimal process maintenance pass as a validated source-term experiment
- current attribution / diagnostic harnesses

### 6.2 Next coding focus

The next focused engineering task should be:

- implement and evaluate thesis-style destructive subduction/collision exclusion during remesh, then re-measure overlap / multi-hit / winner-resolution behavior under motion

Priority goals:

1. remove subducting/colliding geometry from remesh competition where the thesis says it should be invisible
2. reduce first-cycle fragmentation
3. reduce strongly multi-hit-driven component explosion
4. improve boundary coherence / readability
5. then re-evaluate `cw_threshold_mismatch` and process calibration once remesh chaos is lower

### 6.3 What not to do next

Do **not** spend the next cycle on:

- another broad architecture reset
- more preserve / rollback style repair
- major process tuning before remesh coherence improves
- paper reread without a concrete implementation question

---

## 7. Success Criteria Going Forward

The project now needs to judge progress on two separate axes.

### 7.1 Land-budget axis

Questions:

- does continental area stay in a plausible range?
- does net continental loss stop collapsing monotonically?
- can tectonic maintenance stabilize the budget without obvious overcorrection?

### 7.2 Coherence axis

Questions:

- do plate interiors remain coherent?
- do boundaries remain readable?
- does multi-hit behavior stop fragmenting plates into hundreds of components?
- does the remesh produce stable ownership/material structure under motion?

The project should not treat one axis as a proxy for the other.

Good CAF with terrible coherence is not success.
Clean topology with collapsing CAF is not success.

`v8` requires both tracks to converge.

---

## 8. Decision

`v8` decision:

- **Do not** throw away the remesh path.
- **Do not** go back to remesh-alone thinking.
- **Do** keep the tectonic maintenance result as a real architectural finding.
- **Do** shift the main technical focus to remesh coherence, starting with thesis-style destructive subduction/collision exclusion and then multi-hit / overlap / winner-resolution.

In plain terms:

> The project has finally shown that tectonic processes are necessary for the land budget, and that remesh coherence is now the main remaining blocker for readable boundaries, with destructive-boundary exclusion as the clearest next thesis-aligned fix.

That is the `v8` charter.
