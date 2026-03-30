# Procedural Tectonic Planets — Architecture Document v6

**Based on:** Cortial et al., "Procedural Tectonic Planets" (2019)  
**Target:** Unreal Engine 5.7  
**Date:** March 2026  

**Status:** Fresh paper-core redesign spec.  
**Purpose:** Replace the v5 ownership/resampling patch stack with a new authoritative tectonic substrate built in parallel.

---

## 0. Why v6

v5 reached the point where the next useful work is no longer another repair layer.

Three facts are now established:

1. The preserve/localize/rollback architecture accumulated too many state-repair layers.
2. A naive return to periodic global resampling through the existing `FullResolution` path also failed.
3. The main implementation problem is not "tectonics are missing." It is that ownership, material transfer, and boundary reconciliation do not share one authoritative state transition.

v6 exists to rebuild that transition cleanly.

This is **not** a cleanup of v5.  
This is **not** "v5 with fewer patches."  
This is a new paper-core implementation beside the old one.

### 0.1 What v6 is trying to prove

v6 should answer one question cleanly:

> Can a final-owner-first periodic tectonic substrate produce coherent continents and stable boundaries without preserve/localize/rollback repair layers?

If yes, v6 becomes the new base.  
If no, the implementation assumptions around the paper need another revision.

### 0.2 What v6 is not trying to prove yet

v6 v1 is **not** trying to solve:

- production performance
- 500k-scale operation
- final amplified rendering
- perfect parity with every detail of Driftworld
- final tuning of collision/rift rates
- full cleanup/removal of the legacy implementation

v6 v1 is a correctness architecture.

---

## 1. Lessons From v5

These are the non-negotiable lessons the new architecture must respect.

### 1.1 Owner/material desynchronization is fatal

v5 repeatedly created states where:

- `PlateId` said one thing
- `ContinentalWeight` / elevation / thickness came from another owner
- later rollback or reconciliation tried to patch the mismatch

v6 must never allow fields to be transferred from a provisional owner and "fixed later."

### 1.2 Threshold-sensitive fields cannot be treated like generic scalars

`ContinentalWeight` is continuous in storage but discrete in consequence.

- `CW >= 0.5` means continental
- `CW < 0.5` means oceanic

Barycentric transfer can be valid for smooth scalar fields, but threshold-sensitive fields need explicit transfer semantics. v6 must treat this as a first-class design decision, not an afterthought.

### 1.3 Boundary repair became architecture

By late v5, the following were all load-bearing:

- preserve same-owner fast path
- fallback retention
- previous-owner hysteresis
- rift localization rollback
- final-owner `CW` reconciliation

That is evidence that the state transition itself was not authoritative enough.  
v6 must move the authority into the primary solve, not into a repair stack.

### 1.4 Event-local state cannot be created on global provisional state

Rift and collision followups in v5 created ownership/material states that were later localized or reconciled. This produced phantom gains, fake losses, and topology churn.

v6 must use one rule:

> resolve final state first, then transfer state from that final result once.

### 1.5 Paper alignment is about the core loop, not every incidental implementation detail

The paper is the target architecture.  
Reference implementations are useful evidence, but not law.

v6 should follow the paper's core tectonic logic:

- fixed global substrate
- moving rigid plates
- periodic resampling
- gaps/divergence and overlaps/convergence resolved during reconciliation

But it must also spell out the implementation details the paper leaves implicit.

---

## 2. Core Thesis

v6 uses a **single authoritative periodic resampling solve**.

At each periodic resample:

1. Plates have moved.
2. Every canonical sample is classified against the moved plate geometry.
3. Each sample gets exactly one final resolved ownership/state outcome:
   - owned by one plate
   - divergent gap / new oceanic crust
   - overlap / convergent resolution
4. All sample attributes are transferred from that final resolved state exactly once.
5. Membership and derived tectonic fields are rebuilt from the resolved result.

There is no periodic preserve path.  
There is no periodic rollback.  
There is no interpolate-first-reconcile-later flow.

---

## 3. Non-Negotiable Invariants

These invariants are the design contract for v6.

### 3.1 One authoritative owner per sample per resample

After each periodic solve, every canonical sample must have exactly one final owner outcome:

- a plate id
- or an explicit divergent/oceanic result

Overlap/convergent cases must also resolve to exactly one final winning plate on the same solve.

No sample may carry provisional ownership past the end of the solve.

### 3.2 Final owner decided before field transfer

For every sample:

- owner/state resolution happens first
- field transfer happens second

No field may be transferred from an owner that is later overwritten.

### 3.3 One transfer pass per field per resample

Each field is transferred once from the final resolved state.

No later rollback, restoration, or reconciliation should be needed on the periodic path.

### 3.4 Threshold-sensitive fields use explicit transfer semantics

v6 must define field transfer by category:

- threshold-sensitive identity-like fields
- smooth scalar fields
- categorical labels

The implementation must not silently reuse one interpolation rule for all of them.

### 3.5 Periodic path and event path share the same authority model

Events may have additional logic, but they may not violate the same authority rule:

- decide final owner/state
- then transfer fields from that final result

No special path may create provisional state that is later patched back.

---

## 4. v6 Data Model

v6 keeps the same high-level world representation:

- fixed canonical samples on the sphere
- one global spherical Delaunay triangulation
- rigid moving plates
- per-plate carried state between resamples

What changes is the ownership/material transition model.

### 4.1 Canonical samples

Canonical samples remain the authoritative world-state store.

Each sample keeps:

- fixed sphere position
- current `PlateId`
- `ContinentalWeight`
- elevation
- thickness
- age
- ridge/fold directions
- orogeny type
- terrane id
- boundary flags / derived tags as needed

### 4.2 Plate state

Each plate keeps:

- rigid motion parameters
- current member sample set
- carried sample state for those members
- any geometry/query acceleration structures needed by the periodic solve

### 4.3 Periodic solve scratch state

v6 introduces explicit scratch state for each periodic resample.

For each canonical sample:

- owner candidates
- gap/overlap classification
- final resolved ownership/state
- transfer source descriptor

This scratch state exists only during the solve and prevents provisional owner/material state from leaking into canonical state.

### 4.4 Transfer source descriptor

Every final resolved sample should conceptually carry a transfer descriptor:

- final owner plate (if any)
- source triangle or source sample
- barycentric weights if a continuous interpolation is appropriate
- explicit divergent/oceanic creation tag if not owned

This makes the later field transfer rule deterministic and field-specific.

---

## 5. Periodic Resampling Pipeline

This is the heart of v6.

### 5.1 Overview

At each periodic resample:

1. Move plates.
2. Build current-time plate geometry/query state.
3. Classify every canonical sample against moved plates.
4. Resolve every sample to a final owner/state.
5. Transfer fields from that final state.
6. Rebuild plate membership and carried samples.
7. Recompute derived tectonic fields.

### 5.2 Step 1: Advance rigid plate motion

Between periodic resamples, the per-step loop still advances:

- plate rotations
- uplift / erosion / dampening
- other continuous tectonic processes

This part of the architecture is retained from v5.

### 5.3 Step 2: Build current-time query geometry

For each moved plate, build the geometry/query structures needed for authoritative classification.

v6 does **not** commit yet to the current v5 soup partition as the destination architecture.
For v1, existing BVH/soup machinery may be reused as scaffolding, but the design goal is:

- clean classification of canonical samples against moved plate geometry
- minimal dependence on repair logic

### 5.4 Step 3: Sample classification

For each canonical sample, gather all meaningful current-time plate interactions:

- contained by one plate
- contained by multiple plates
- contained by none
- near boundary / recoverable

This step produces candidate ownership evidence only.
It does not yet mutate canonical sample state.

### 5.5 Step 4: Final state resolution

Resolve each sample into one final state:

- single-owner interior sample
- overlap/convergent sample resolved to one owner
- divergent gap becoming new oceanic crust
- non-divergent unresolved case handled by explicit resolution policy

This is where boundary policy belongs.

The output of this step is the final authoritative owner/state for each sample.

#### Phase split for Step 4

v6 Phase 1 uses the simplest possible deterministic boundary policy so ownership stability can be tested before physical fidelity is layered back in:

- if one clear owner candidate exists, that plate wins
- if multiple owner candidates exist, resolve to exactly one winner with a deterministic tie-break
- if no owner candidate exists, use the nearest recoverable owner instead of immediate oceanic creation

Phase 3 then replaces this temporary ownership-only policy with the physically meaningful boundary policy:

- divergent gaps become new oceanic crust
- convergent overlaps use the chosen convergence-resolution rule
- non-divergent unresolved cases use an explicit final policy rather than generic fallback

Phase 1 is therefore allowed to be physically blunt as long as it is deterministic and authoritative.

### 5.6 Step 5: Field transfer

Transfer every field from the final resolved state only.

No field transfer occurs before this step.
No field is transferred from a provisional owner.

### 5.7 Step 6: Repartition and rebuild

Once canonical state is final:

- rebuild plate membership
- rebuild carried samples

Query geometry belongs to Step 2 of the next periodic solve, not here.  
Step 6 only commits the new authoritative state and rebuilds the owning/member structures derived from it.

### 5.8 Step 7: Recompute derived fields

After authoritative ownership/material transfer:

- recompute subduction distance
- recompute boundary classifications
- recompute terranes
- recompute other dependent derived fields

Derived fields should be consumers of the final solved state, not participants in repairing it.

---

## 6. Field Transfer Rules

v6 must stop pretending every field is the same kind of quantity.

### 6.1 Identity-like / threshold-sensitive fields

These fields are not safe to treat as generic barycentric scalars.

Examples:

- `ContinentalWeight` / effective crust identity

#### v6 v1 rule

For v1, threshold-sensitive fields should use explicit, non-diffusive transfer semantics tied to the final owner.

Default v1 rule:

- choose the dominant source from the final owner's transfer descriptor
- transfer directly from that source

This is intentionally conservative.
If later refinement is needed, it should be an explicit second step, not an accidental side-effect of generic interpolation.

This section describes transfer only. Continuous creation/destruction pathways such as Andean conversion and collision/uplift effects remain separate tectonic processes and are retained from v5 unless explicitly replaced later.

### 6.2 Smooth scalar fields

Examples:

- elevation
- thickness
- age

These may use continuous interpolation from the final owner's source descriptor, as long as that owner has already been finalized.

### 6.3 Directional / categorical fields

Examples:

- ridge direction
- fold direction
- terrane id
- orogeny type

These need field-specific rules:

- majority vote or dominant-source transfer for categorical labels
- normalized vector blending only where mathematically meaningful for directional fields

v6 must specify these explicitly rather than inheriting whatever a helper currently does.

---

## 7. Boundary Policy

The periodic solver lives or dies on this section.

### 7.1 Boundaries are resolved, not repaired

v5 treated many boundary outcomes as damage to be patched later.
v6 must instead make the periodic boundary decision explicit in the solver.

### 7.2 Three boundary outcomes

At periodic resampling, a boundary-adjacent sample must resolve into one of:

1. **Owner retained**
   - final owner remains previous owner because current-time geometric evidence supports it

2. **Owner reassigned**
   - final owner changes to a competing plate because current-time evidence supports the new owner

3. **Gap/oceanic creation**
   - final state is divergent/new oceanic material

There should be no fourth state of:

- "temporarily wrong, will be reconciled later"

### 7.3 Strong continental boundaries require explicit policy

v5 showed that strongly continental samples at boundaries are both visually and geologically sensitive.

v6 must make their boundary policy explicit:

- either they remain because the final owner solve says so
- or they are legitimately reassigned/oceanized

They should not disappear because they fell off a maintenance path.

---

## 8. Events

v6 should not reintroduce the v5 mistake of letting event plumbing invent a second authority model.

### 8.1 Event rule

Events may alter geometry/topology input, but they must feed into the same authority pattern:

1. define changed tectonic configuration
2. solve final owner/state
3. transfer fields from that final state once

#### v6 v1 event timing

v6 v1 takes the simpler position:

- events may update plate topology/lifecycle between periodic solves
- ownership/material consequences are applied by the next periodic authoritative solve
- if an event is detected on a step that also performs a periodic solve, the event update happens first and that same periodic solve resolves the final result

There is no separate event followup resample with its own authority model in v6 v1.

### 8.2 v6 v1 event scope

v6 v1 does not need to perfect the entire event architecture immediately.
But it must avoid:

- provisional owner assignment
- later localization rollback
- later field reconciliation

### 8.3 Rifts

For v1:

- keep rift triggering and child lifecycle simple
- but do not allow child ownership/material state to be transferred before final child ownership is decided

### 8.4 Collisions

For v1:

- keep collision detection and uplift mechanisms
- but ensure any ownership/material change resolves through the same final-owner-first rule

---

## 9. What v6 Reuses vs Replaces

### 9.1 Reuse

v6 should reuse:

- fixed canonical substrate
- global triangulation / adjacency
- initialization and seeding
- rigid plate kinematics
- per-step uplift / erosion / dampening
- existing continuous creation/destruction processes such as Andean conversion and collision uplift/surge, provided they consume authoritative ownership rather than repair it
- exporters and visualization
- harness / logging infrastructure

### 9.2 Replace

v6 should replace the following architectural ideas from v5:

- periodic preserve ownership maintenance
- periodic same-owner fast path / fallback repair
- previous-owner hysteresis as architecture
- localization/rollback as normal event infrastructure
- post-hoc `CW` reconciliation for periodic correctness
- interpolate-first-fix-later control flow

### 9.3 Scaffolding allowed in v1

v1 may temporarily reuse:

- existing BVH/soup machinery
- existing repartition/rebuild scaffolding
- existing test/export infrastructure

But these are implementation aids, not architectural commitments.

---

## 10. Validation Plan

v6 should be judged by maps first, metrics second.

### 10.1 Primary checkpoints

- step `100`
- step `200`
- step `300`
- step `400`

### 10.2 Primary maps

- `CrustType`
- `ContinentalWeight`
- `Elevation`
- `PlateId`
- `BoundaryMask`

### 10.3 Primary visual questions

1. Are step-300/400 continents still coherent landmasses?
2. Are boundaries visually clean enough to read as tectonic structure rather than confetti?
3. Does landmass change look geologically continuous rather than popping/teleporting?

### 10.4 Secondary metrics

Secondary metrics should support the visual decision:

- continental area fraction
- gap rate
- overlap rate
- max components per plate
- final max components
- boundary sample fraction
- event counts

### 10.5 v6 v1 success gate

v6 v1 is promising if:

- step-300/400 maps are visibly more coherent than the final v5 line
- late-run landmass collapse is materially reduced
- boundaries no longer require a patch stack to stay readable

v6 v1 fails if:

- the same ownership/material pathologies reappear
- step-300/400 remain visually chaotic
- the new solver still needs rollback/reconciliation layers to function

---

## 11. Implementation Plan

v6 should be built in phases, not all at once.

### Phase 1: Ownership-only authoritative periodic solve

Deliver:

- new v6 periodic solve skeleton
- authoritative sample classification
- final owner/state resolution with a temporary simple boundary policy
- no preserve/rollback machinery

Goal:

- prove ownership/topology behavior is cleaner before worrying about full field fidelity

Temporary Phase 1 policy:

- zero-hit samples use nearest recoverable owner
- multi-hit samples resolve to exactly one deterministic winner
- no physically meaningful divergent/convergent semantics required yet

### Phase 2: Field transfer rules

Deliver:

- final-owner-first transfer
- explicit rule split for threshold-sensitive vs smooth fields

Goal:

- prove material state no longer desynchronizes from ownership

### Phase 3: Gap/overlap resolution

Deliver:

- explicit divergent and convergent resolution in the new solver

Goal:

- keep boundary outcomes legible without repair layers

### Phase 4: Events

Deliver:

- collision and rift integration into the same authority model

Goal:

- no deferred provisional state followed by rollback

### Phase 5: Legacy comparison

Deliver:

- side-by-side comparison against final v5/M6aj line

Goal:

- make the architecture decision from evidence, not momentum

### Iteration budget

v6 v1 is not a production performance project, but it still needs to be practical to iterate on.

Target expectation:

- a `60k / 7 / seed 42` periodic resample should remain comfortably under ~5 seconds on the development machine

If the first v6 periodic solve is dramatically slower than that, iteration quality will collapse and the architecture will need simplification before deeper tuning.

---

## 12. File / Code Organization

v6 should live beside v5, not inside it.

Recommended files:

- `Source/Aurous/Public/TectonicPlanetV6.h`
- `Source/Aurous/Private/TectonicPlanetV6.cpp`
- `Source/Aurous/Private/Tests/TectonicPlanetV6Tests.cpp`

Optional supporting files if needed:

- `Source/Aurous/Public/TectonicPlanetV6Types.h`
- `Source/Aurous/Private/TectonicPlanetV6Ownership.cpp`
- `Source/Aurous/Private/TectonicPlanetV6Transfer.cpp`

The v6 code path should not depend on legacy preserve/rollback helpers except where reused intentionally as temporary scaffolding.

---

## 13. Final Position

v5 did valuable work:

- it identified the real failure modes
- it disproved naive global resampling through the old stack
- it showed which fields and boundaries are fragile

But v5 also showed that the current tectonic substrate architecture is too repair-heavy to be trusted as the final paper-core.

v6 should therefore be built as:

- a new paper-core implementation
- in new files
- inside the existing plugin
- with a final-owner-first periodic resampling solver
- judged by step-300/400 map coherence above all else

That is the cleanest path still open to "build the paper" rather than continue repairing an exhausted interpretation of it.
