# Follow-Up Prompt: Thesis Structured Divergence Fill

You’re working in `/mnt/c/Users/Michael/Documents/Unreal Projects/Aurous`.

Repo constraints:
- Work only in `/mnt/c/Users/Michael/Documents/Unreal Projects/Aurous`.
- The working tree is dirty. Preserve unrelated local changes.
- Do not revert anything you did not intentionally change.
- Do not wire anything into `ATectonicPlanetActor`, the editor panel, or the runtime actor path.
- Do not port `v5` preserve/repair logic into this work.
- This is a focused copied-frontier miss-handling task, not a new architecture spike.

Mindset:
Treat the thesis as the spec for miss handling.
The current copied-frontier path now has three established facts:
- zero-drift identity passes
- tectonic maintenance is necessary for land budget
- destructive exclusion is a real coherence lever

The next likely missing layer is thesis-style structured divergence fill.
Do not redesign the remesh architecture in this task.
Do not remove destructive exclusion.

Why this task exists:
- The thesis treats remesh misses as structured divergence fill, not blanket oceanization.
- Current code still handles misses too bluntly:
  - `CW = 0`
  - flat deep-ocean elevation
  - generic ridge-direction fallback
- After destructive exclusion, miss growth becomes a central quality problem.
- We need to test whether thesis-style miss synthesis makes those misses geologically coherent enough to keep the path viable.

Read these first:
- `docs/tectonic-planets-architecture-v8.md`
- `docs/Boundary-Triangle Handling deep-research-report.md`
- `Source/Aurous/Public/TectonicPlanetV6.h`
- `Source/Aurous/Private/TectonicPlanetV6.cpp`
  - copied-frontier remesh path
  - miss handling / oceanic creation path
  - any existing frontier extraction helpers
  - any ridge/elevation helpers
- `Source/Aurous/Private/TectonicPlanet.cpp`
  - read-only reference for any existing ridge/oceanic helpers
- `Source/Aurous/Private/Tests/TectonicPlanetV6Tests.cpp`
- Relevant logs:
  - `Saved/Logs/V6CopiedFrontierDestructive-console.txt`
  - `Saved/Logs/V6CopiedFrontierProcessDestructive-console.txt`

Thesis miss algorithm to approximate:
For a remesh sample `p` with no valid hit:
1. find nearest frontier point `q1` on any plate
2. find nearest frontier point `q2` on a different plate
3. compute spherical midpoint:
   - `q_gamma = R * (q1 + q2) / ||q1 + q2||`
4. derive a blend factor from distance to the ridge midpoint versus nearest frontier
5. synthesize new oceanic crust:
   - `CW = 0`
   - `Age = 0`
   - elevation from a ridge-shaped profile blended with border elevations
   - standard oceanic thickness
   - ridge direction from local geometry, not a generic velocity-only fallback
6. assign to the nearest plate as needed for bookkeeping

Task:
Implement the smallest credible thesis-style structured divergence fill for the copied-frontier path.

Core requirements:
1. Replace blanket miss→oceanization with structured divergence fill.
   - Do not use flat fixed deep-ocean elevation if a ridge/border-based value can be derived.
   - Use frontier geometry to define the fill.

2. Build or reuse a per-plate frontier point representation suitable for nearest-frontier queries.
   - Keep this bounded. It does not need to become a whole new plate architecture.

3. For each miss, distinguish the two relevant miss contexts if feasible:
   - true divergence-style miss
   - destructive-exclusion-induced miss
   You do not need different fill behavior immediately unless clearly justified, but report the split if you can.

4. Keep the copied-frontier remesh contract otherwise unchanged:
   - direct hit-triangle transfer for hits
   - zero fallback target
   - maintenance pass may remain enabled for process-branch comparisons, but be explicit

5. Keep this bounded:
   - do not redesign multi-hit winner logic here
   - do not redesign remesh query geometry here
   - do not port a full event system

Instrumentation/reporting required:
At minimum report before/after for:
- first-cycle copied-frontier control
- long-run copied-frontier baseline
- process-enabled baseline too if affected

Report:
- `continental_area_fraction`
- `max_components_after`
- `cw_threshold_mismatch`
- `hit_count`
- `miss_count`
- `multi_hit_count`
- `oceanic_creation`
- `transfer_fallback`
- destructive-filter stats if the branch includes them

Also report structured-fill-specific metrics if feasible:
- count of misses using structured divergence fill
- nearest-frontier-pair success count
- miss samples where a second frontier from a different plate was found
- average / median ridge-distance blend factor
- number of misses classified as destructive-exclusion-induced vs true divergence if available

Success criteria:
Call the result promising only if most of these happen:
- first-cycle miss handling becomes geologically less destructive
- long-run CAF improves materially versus blunt miss→oceanize on the same remesh branch
- remesh coherence does not catastrophically regress
- the new fill is clearly more thesis-aligned than the current blanket oceanization

Important:
- This task is about miss quality, not directly about multi-hit winner resolution.
- If structured fill improves land budget but coherence remains bad, say that clearly.
- If structured fill barely helps because destructive classification is still too broad, say that clearly.

Tests:
Add/update focused tests that validate:
- nearest-frontier-pair lookup works
- a miss produces structured oceanic crust instead of blanket default values
- ridge direction / elevation are derived from geometry in the intended way
- ordinary hit-triangle transfer still works unchanged

Verification:
1. Build `AurousEditor`
2. Run focused copied-frontier structured-fill tests
3. Rerun:
   - first-cycle diagnostic
   - copied-frontier long-run baseline
   - process-enabled baseline too if affected
4. Report before/after clearly

Deliverables:
- code changes
- tests added/updated
- build/test results
- first-cycle and long-run before/after metrics
- concise explanation of the structured divergence fill implementation
- clear statement of what improved and what did not
- recommendation for the next step after this experiment
- list of files changed

Hard decision rule:
- If structured fill materially improves the miss-heavy destructive-filter regime, say clearly that thesis-style miss synthesis is a necessary missing layer.
- If it barely helps, say clearly whether the remaining problem is still over-broad destructive classification or a deeper remesh-coherence issue.
- Do not overclaim success if multi-hit fragmentation remains unresolved.

Do not stop at analysis. Implement the structured fill if feasible, run the tests, and report the result clearly.
