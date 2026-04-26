# Tectonic Architecture Reset - Plate-Authoritative Prototype

**Status:** Proposed replacement architecture after the April 2026 failure review.

**Purpose:** Define the next architecture clearly enough that implementation does not drift back into ownership-persistence heuristics.

## 1. Executive Summary

The current tectonic system fails because the fixed global ownership field is being used as both:

1. the carrier of moving tectonic material
2. the authoritative final plate partition

Those roles conflict.

Stable ownership kills drift. Aggressive reassignment restores drift metrics but destroys coherent plate regions and thin boundaries.

The replacement architecture should make **moving plate objects** authoritative, carry continental material **on the plate**, and treat the global lattice as a **projection/output surface only**.

This proposal is strengthened by the later paper-faithful prototype result:

- a naive global geometry-authoritative reassignment path was tested and failed in this codebase
- therefore the next architecture should not be another "fresh assignment every remesh on the same lattice carrier" variant
- the next prototype must change the carrier model itself

## 2. Problem Statement

Current evidence shows:

- internal plate kinematics exist
- anchored ownership suppresses visible continental drift
- local ownership transfer produces noisy, area-filling ownership and boundary fields
- revived oceanization can become speckled rather than basin-like
- long-horizon land/ocean balance remains wrong

Conclusion:

- this is an architecture problem, not a tuning problem

## 3. Goals

- continental interiors visibly advect across the globe
- plate regions remain coherent by construction
- boundaries remain thin, connected, and localized
- post-rift basin opening can be implemented as a pair-local process
- the architecture reuses existing geometry/query infrastructure where useful
- the global lattice remains available for rendering, diagnostics, and downstream systems

## 4. Non-Goals

- do not solve full Earth realism in the first prototype
- do not preserve compatibility with V9 ownership-authority behavior
- do not implement full convergence/divergence/collision logic in Prototype A
- do not tune uplift, erosion, or collision constants in this phase
- do not use generic miss/gap behavior as a substitute for explicit tectonic interfaces
- do not build another naive global reassignment path on top of the current query/assignment carrier model

## 5. Core Architecture Decision

### 5.1 Source of Truth

The authoritative tectonic state is:

- moving plate geometry
- plate-local material state attached to those plates
- explicit plate-pair interfaces

The global sample lattice is not the source of truth for tectonic identity.

### 5.2 Authority Rules

- plate identity comes from the plate object
- continental material moves with the plate
- oceanic crust is created and consumed only at explicit interfaces
- the lattice stores projected results, not canonical tectonic state
- no persistent per-sample ownership is used as the main transport carrier

## 6. Runtime Layers

### 6.1 Kinematics Layer

Responsibilities:

- store plate rigid-rotation state
- advance plate transforms each step
- maintain neighbor/interface candidate relationships

Authoritative data:

- plate transform
- plate geometry in plate-local space
- current world-space transform

### 6.2 Material Layer

Responsibilities:

- carry continental terranes and other crustal material on the plate
- keep material in plate-local coordinates
- provide local field sampling for projection and interface mutation

Authoritative data:

- continental masks / terrane patches
- oceanic crust fields
- age, thickness, elevation, ridge/orogeny metadata

### 6.3 Interface Solver

Responsibilities:

- determine divergent / convergent / transform interaction per plate pair
- create or consume material only in narrow interface strips
- keep tectonic mutation pair-scoped and local

Authoritative data:

- interface classification
- interface strip geometry
- mutation outputs for each plate pair

### 6.4 Projection Layer

Responsibilities:

- rasterize current plate/material state to the global lattice
- emit `PlateId`, `CrustType`, elevation, diagnostics, and exports
- remain downstream of tectonic authority

Authoritative data:

- none for tectonic evolution
- only projected outputs

## 7. Data Model

### 7.1 PlateState

Fields:

- `PlateId`
- local spherical patch / mesh
- rigid transform state
- neighbor/interface references
- local material collections

### 7.2 MaterialPatch

Fields:

- owning plate id
- local coordinates on plate
- crust type
- continental weight
- elevation
- thickness
- age / tectonic metadata

### 7.3 InterfaceRecord

Fields:

- plate pair ids
- interface type
- interface strip geometry
- relative motion summary
- mutation counters / diagnostics

### 7.4 ProjectionGrid

Fields:

- sample position
- projected `PlateId`
- projected crust/material fields
- derived boundary flags
- diagnostics-only masks

## 8. Update Loop

### 8.1 Prototype A Loop

1. advance plate rigid transforms
2. update world-space plate geometry
3. sample plate-local material through transformed geometry
4. project the current plate/material state onto the global lattice
5. derive projected boundaries from projected neighboring plate regions
6. emit diagnostics and exports

Prototype A deliberately does not perform tectonic material exchange.

### 8.2 Prototype B Loop

Add:

- explicit divergent interface strips
- coherent oceanic corridor creation only along separating plate pairs

### 8.3 Prototype C Loop

Add:

- convergent consumption of oceanic crust
- terrane accretion / continental collision mutation
- localized uplift/orogeny updates

## 9. Reuse From Current Codebase

Reuse:

- plate rotation machinery
- per-plate mesh/query infrastructure
- BVH/query acceleration
- barycentric field interpolation
- export pipeline and map generation
- diagnostics harness structure

Do not reuse as authority:

- persistent per-sample ownership
- same-owner retention / hysteresis
- active-zone ownership gating
- generic miss-recovery ownership logic
- generic frontier-pair fill as a basin-opening mechanism
- global best-hit reassignment of the canonical lattice as the primary tectonic carrier

## 10. Prototype A Scope

Prototype A should answer one question only:

Can we get:

- visibly moving continents
- coherent plate regions
- thin boundaries

without using persistent lattice ownership as the tectonic source of truth?

Prototype A should not try to solve:

- subduction
- terrane transfer
- rifting
- ocean-basin life cycle
- long-horizon land/ocean equilibrium

## 11. Acceptance Criteria

Use the hard gates in `docs/tectonic-minimal-acceptance-tests.md`.

Prototype A must pass at least:

- Drift Gate
- Region-Coherence Gate
- Boundary Gate
- Visual Gate

Prototype B adds:

- Basin-Opening Gate

Prototype C adds:

- Balance Gate

## 12. Risks

- plate-local material representation may be harder than expected to update cleanly
- projection may introduce aliasing if plate/material sampling is too sparse
- explicit interface strips may require new geometry tooling
- existing code may tempt partial reuse of the old authority model; that should be resisted

## 13. Immediate Implementation Plan

1. define the authoritative plate-local state objects
2. route projection/export from plate/material state rather than lattice ownership
3. implement Prototype A with no tectonic exchange
4. run drift/coherence/boundary acceptance tests
5. only then add divergent basin opening as Prototype B

## 14. Decision Rule

- If Prototype A gives moving continents with coherent regions and thin boundaries, continue on this architecture.
- If it fails, fail it explicitly against the acceptance tests and revise the carrier model.
- Do not fall back into ownership-persistence heuristics without a new written justification.
