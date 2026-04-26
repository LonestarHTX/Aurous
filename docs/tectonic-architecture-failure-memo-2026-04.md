# Tectonic Architecture Failure Memo - April 2026

**Status:** Current decision memo after the long-horizon failure audit, post-rift divergent-fill probe, advection/ownership audit, and the bounded interior-advection and oceanization recovery attempts.

**Purpose:** Freeze what is now known to be false, record the exact architectural contradiction, and keep the project from looping on more ownership heuristics.

## 1. Executive Summary

The current tectonic stack is not failing because one constant is off.

It is failing because the current ownership architecture is trying to make one fixed canonical sample field do two incompatible jobs:

1. carry moving tectonic material through time
2. remain a clean, stable final ownership map with thin meaningful boundaries

The audits now show that the current architecture cannot satisfy both requirements at once.

The hard findings are:

1. Internal plate kinematics exist.
2. Under anchored ownership, continental interiors do not materially advect with their plates.
3. Under aggressive ownership transfer, drift metrics improve, but plate regions and boundary quality collapse.
4. Post-rift oceanization can be revived, but it can also become speckled and non-basin-like.
5. Long-horizon land-ocean balance remains badly wrong.

This is not a tuning problem anymore.

It is an architecture problem.

## 2. Evidence

### 2.1 Internal plate motion exists

The combined-map arrow exports and speed logs showed healthy plate velocities rather than near-zero motion:

- `Saved/Logs/Aurous-V9PaperSelectiveConfirmation-stdout.log`
- `Saved/Logs/Aurous-V9250kSubmergedShoulderFix-stdout.log`

This ruled out the "plates are not moving at all" theory.

### 2.2 Anchored ownership killed visible continental drift

The advection/ownership audit established that plate/query geometry was moving while the material ownership footprint stayed almost fixed:

- `Saved/Logs/Aurous-V9AdvectionOwnershipAudit-stdout.log`

Representative baseline result:

- step-400 expected mean drift: about `8950 km`
- ownership-footprint drift proxy: about `524 km`
- step-0 continental-core overlap for representative plates: about `0.92` to `0.98`

That is direct evidence that the visible/material continent was staying put while geometry rotated underneath it.

### 2.3 Aggressive transfer fixed drift metrics but broke topology

The bounded interior-advection change corrected the anchored-interior failure numerically:

- footprint drift rose to the same order as expected plate-frame drift
- materially anchored fraction collapsed
- step-0 overlap dropped sharply

But the new boundary and ownership visuals showed a different failure:

- plate ID regions lost coherence
- boundary masks spread into broad area-fill
- the world started reading as samplewise ownership shuffling rather than moving coherent plates

This is the opposite failure mode:

- drift metrics improved
- plate topology degraded

### 2.4 Revived oceanization was not enough and could become speckled

The post-rift probe proved that separating child boundaries existed and that the kept V6 path was initially failing to convert divergence into oceanic outcomes:

- `Saved/Logs/Aurous-V9PostRiftDivergentFillProbe-stdout.log`

After the bounded divergent-fill bypass fix, oceanic creation became nonzero and local opening improved.

But the resulting visual field could still look globally speckled rather than like coherent new ocean basins.

That means:

1. the dead mechanism was real
2. reviving it helped
3. the system still lacks coherent, pair-local basin opening

### 2.5 Long-horizon balance is still wrong

Long-horizon runs remain far from the target regime:

- the original `250k / 40 / seed 42 / step 400` run reached `CAF=0.9773`
- later bounded fixes reduced the catastrophic end state somewhat, but still left the model in a land-heavy, non-acceptable regime

This is not a style issue. It is a tectonic balance failure.

### 2.6 Naive paper-faithful global remesh also failed in this codebase

The separate paper-faithful prototype audit closed the loop on the remaining "maybe we just abandoned the paper too early" question:

- `Saved/Logs/Aurous-PaperGeometryPrototypeAudit.log`
- `Saved/MapExports/V6PaperGeometryAuthoritativePrototypeAudit/...`

The prototype was deliberately simple and geometry-authoritative:

1. full plate soup meshes were rotated
2. every sample was globally queried against current geometry
3. the best geometry hit won
4. fields transferred through the existing triangle interpolation path
5. miss samples fell through the existing oceanic creation path
6. the plate partition was rebuilt from those fresh assignments

That path did not beat the kept V9 direction.

Instead, it failed with a different but equally disqualifying signature:

- very large miss counts
- very large multi-hit counts
- extreme churn
- poor coherence
- heavy leakage
- rapid continental mass collapse / runaway oceanization

Representative result from the audit:

- `250k step100`: `167,814` hits, `82,186` misses, `61,187` multi-hits
- `250k step200`: `166,238` hits, `83,762` misses, `63,744` multi-hits
- `250k step400`: `160,751` hits, `89,249` misses, `65,779` multi-hits

Morphology collapsed with it:

- `CAF 250k`: `0.2980 -> 0.0941 -> 0.0166 -> 0.0006` at `step 0 / 100 / 200 / 400`

This matters because it rules out another false branch:

- the current problem is not just that V9 departed from the paper
- a naive geometry-authoritative global reassignment path is also not viable in this codebase through the current query/assignment carrier model

## 3. The Actual Contradiction

The project has been trying to make one ownership field act as both:

1. the transport carrier for moving continental material
2. the authoritative clean plate partition on a fixed global lattice

Those goals pull in opposite directions in the current implementation:

- persistence and hysteresis keep regions coherent, but anchor continents in place
- frequent reassignment allows drift, but breaks regions into noisy reclassification fields

That is why the project keeps looping between two bad states:

1. coherent but frozen
2. moving but incoherent

As long as the same canonical ownership field is asked to satisfy both jobs directly, more threshold tuning is unlikely to produce a real solution.

## 4. What Is Now Disproven

The following lines of attack should not be reopened casually:

### 4.1 Ownership persistence as the core answer

It does improve stability metrics, but it also suppresses the product requirement that matters most: visible continental drift.

### 4.2 Local ownership-transfer heuristics as the core answer

They can improve drift numbers, but they do not preserve coherent plate regions and thin boundaries reliably enough.

### 4.3 "One more threshold" thinking

The evidence now points to a structural contradiction, not a missing scalar.

### 4.4 Treating this as a visualization problem

The diagnostics proved the defect is in material/ownership evolution, not in how maps are rendered.

### 4.5 "Just do the paper" as a drop-in replacement

That question has now been tested directly enough for this codebase.

The paper-faithful global geometry-authoritative remesh prototype did not rescue the system. It produced more apparent early drift, but with catastrophic miss/overlap behavior, heavy fragmentation, and near-total continental collapse.

That does not prove the paper is wrong.

It does prove that a naive paper-like drop-in remesh path is not a better replacement here.

## 5. Architectural Position Going Forward

The next system should be built around these principles:

1. Plate geometry must be authoritative enough that continents visibly advect.
2. The final lattice export must not be the thing that anchors tectonic material in place.
3. Boundary behavior must be resolved from geometric interaction, not from globally persistent ownership memory.
4. Divergent opening must be pair-local and coherent, not a generic global reclassification effect.
5. Diagnostics must be first-class, not added after the fact.

That does not commit the project to one exact implementation yet.

It does rule out continuing to patch the current ownership-authority architecture as the main path.

The current proposed replacement outline is:

- `docs/tectonic-architecture-reset-plate-authoritative-prototype.md`

## 6. Immediate Next-Step Policy

Now that the paper-faithful remesh falsification run is complete, treat these as the current working rules:

1. Do not promote the current V9 ownership-authority path as "solved."
2. Do not spend another cleanup/fix cycle on ownership heuristics.
3. Do not spend another cycle on naive global geometry-authoritative reassignment inside the current carrier model.
4. Judge any next architecture by hard acceptance tests, not by isolated proxy metrics.

## 7. Bottom Line

The current tectonic system is not one bug away from being correct.

The project now knows something more valuable:

- internal kinematics exist
- drift and coherent ownership are not jointly solved in the current architecture
- local heuristic patching keeps changing the failure shape rather than removing the failure
- naive paper-faithful global reassignment also fails in this codebase

That is enough to stop looping.
