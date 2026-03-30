# M6 Tuning Scorecard

**Date:** March 20, 2026  
**Scope:** M6a instrumentation audit, `60k / 7 / seed 42` baseline, and provisional tuning criteria  
**Status:** Baseline capture only. No parameter tuning in this milestone.

## Scope Notes

- This document records the frozen "before" snapshot for M6 tuning.
- The baseline run is `200` steps only. It is **not** the final M6 acceptance run.
- Longer `500+` step acceptance runs and multi-seed validation come later.

## Section 12.4 Metric Audit

### Directly covered in current code

| Section 12.4 metric | Current implementation | Status |
|---|---|---|
| `continental_area_fraction` | `FResamplingStats.ContinentalAreaFraction`, log key `continental_area_fraction` | Added in M6a |
| `overlap_count` | `FResamplingStats.OverlapCount`, log key `overlap_count` | Already present |
| `gap_count` | `FResamplingStats.GapCount`, log key `gap_count` | Already present |
| `divergent_gap_count` | `FResamplingStats.DivergentGapCount`, log key `divergent_gap_count` | Already present |
| `numerical_gap_count` | `FResamplingStats.NonDivergentGapCount`, log key `numerical_gap_count` | Remapped |
| `plate_count` | `FResamplingStats.PlateCount`, log key `plate_count` | Added in M6a |
| `max_components_per_plate` | `FResamplingStats.MaxComponentsPerPlate`, log key `max_components_per_plate` | Added in M6a |
| `boundary_sample_fraction` | `FResamplingStats.BoundarySampleFraction`, log key `boundary_sample_fraction` | Added in M6a |
| `elevation_min` | `FResamplingStats.ElevationMinKm`, log key `elevation_min` | Added in M6a |
| `elevation_max` | `FResamplingStats.ElevationMaxKm`, log key `elevation_max` | Added in M6a |
| `elevation_mean` | `FResamplingStats.ElevationMeanKm`, log key `elevation_mean` | Added in M6a |
| `subduction_front_count` | `FResamplingStats.SubductionSeedSampleCount`, log key `subduction_front_count` | Remapped |
| `collision_event_count` | `FResamplingStats.CollisionCount`, log key `collision_event_count` | Already present |
| `resampling_interval_R` | `FResamplingStats.Interval`, log key `resampling_interval_R` | Already present |
| `phase_timing_ms` | `[ResamplePhaseTiming ...]` line with per-phase keys | Added in M6a |

### Remapped because the collision model changed

| Older spec concept | Current implementation mapping | Notes |
|---|---|---|
| `active_collision_tracks` | `FResamplingStats.ActiveCollisionTrackCount`, log key `active_collision_tracks` | Current preserve-mode collision tracks are pair-level entries in `BoundaryContactPersistenceByPair`, not `(TerraneId, OpposingPlateId)` tracks |
| `numerical_gap_count` | `NonDivergentGapCount` | Current terminology is "non-divergent gap" rather than "numerical gap" |
| `subduction_front_count` | `SubductionSeedSampleCount` | Current Dijkstra seeds are the best proxy for front samples with `d=0` |

### N/A under the current preserve-mode collision path

| Older spec metric | Current status | Why |
|---|---|---|
| `collision_backdated_count` | N/A | Preserve-mode collision uses boundary-contact persistence plus cached same-step follow-up, not threshold-crossing backdating |
| `max_collision_backdate_steps` | N/A | Same reason |

## Machine-Readable Log Format

Every resampling now emits:

- `[ResampleMetrics R=<index> Step=<step>] key=value ...`
- `[ResamplePhaseTiming R=<index> Step=<step>] key=value ...`

These are derived from canonical sample state and are intended for later A/B parsing.

## Frozen Baseline

**Run:** `M6Baseline60k7-seed42-samples60000-plates7`  
**Configuration:** `60k / 7 / seed 42`, `PreserveOwnershipPeriodic`, automatic rifting enabled, `200` steps  
**Automation:** `Aurous.TectonicPlanet.M6Baseline60k7`

### Run Summary

| Metric | Baseline value |
|---|---:|
| total resamples | `48` |
| total collision events | `21` |
| total rift events | `22` |
| first collision step | `20` |
| first rift step | `1` |
| min continental area fraction | `0.000233` |
| max continental area fraction | `0.308400` |
| final continental area fraction | `0.000233` |
| min boundary sample fraction | `0.041533` |
| max boundary sample fraction | `0.192333` |
| final boundary sample fraction | `0.043483` |
| min gap rate | `0.000000` |
| max gap rate | `0.230067` |
| final gap rate | `0.058267` |
| min overlap rate | `0.000000` |
| max overlap rate | `0.023033` |
| final overlap rate | `0.001250` |
| min plate count | `7` |
| max plate count | `49` |
| final plate count | `49` |
| max components per plate observed | `37` |
| min mean elevation | `-4.111436 km` |
| max mean elevation | `-2.580058 km` |
| final mean elevation | `-3.944098 km` |
| stable plate ids valid | `0` |
| ownership changes outside resampling | `0` |

### Checkpoint Snapshot

| Step | ResamplingSteps | Trigger / Ownership | Boundary fraction | Gap count | Overlap count | CC overlap count | Continental count | Mean elevation |
|---|---|---|---:|---:|---:|---:|---:|---:|
| `0` | `[]` | `None / FullResolution` | `0.041533` | `0` | `0` | `0` | `17462` | `-4.111436` |
| `50` | `[1,2,3,5,10,20,20,21,22,23,24,27,29,30,30,40,40,42,43,44,45,46,48,50]` | `RiftFollowup / FullResolution` | `0.179750` | `3354` | `1335` | `90` | `7317` | `-2.617509` |
| `100` | `[1,2,3,5,10,20,20,21,22,23,24,27,29,30,30,40,40,42,43,44,45,46,48,50,52,60,60,63,65,70,70,72,75,80,80,90,90,100]` | `Periodic / PreserveOwnership` | `0.150333` | `10556` | `529` | `0` | `441` | `-3.144001` |
| `150` | `[1,2,3,5,10,20,20,21,22,23,24,27,29,30,30,40,40,42,43,44,45,46,48,50,52,60,60,63,65,70,70,72,75,80,80,90,90,100,110,120,130,140,150]` | `Periodic / PreserveOwnership` | `0.076100` | `6178` | `38` | `0` | `55` | `-3.548540` |
| `200` | `[1,2,3,5,10,20,20,21,22,23,24,27,29,30,30,40,40,42,43,44,45,46,48,50,52,60,60,63,65,70,70,72,75,80,80,90,90,100,110,120,130,140,150,160,170,180,190,200]` | `Periodic / PreserveOwnership` | `0.043483` | `3496` | `75` | `0` | `14` | `-3.944098` |

## M6b Stabilization Re-Baseline

**Run:** `M6Baseline60k7-seed42-samples60000-plates7` after the M6b structural fixes  
**Scope:** stable-id repair + automatic-rift suppression, same `60k / 7 / seed 42 / 200 step` configuration

### Root cause and structural fix

- `stable_plate_ids_valid = 0` was caused by dead plates remaining in the active `Plates` array after membership rebuilds. Stable sample ids still referred to real plate ids, but empty active plates broke the invariant and inflated plate-count / fragmentation diagnostics.
- M6b fixes this by pruning empty plates during `RepartitionMembership()`.
- Automatic rifting was structurally too eager because it scanned all eligible parents each step, allowed `2-4` automatic children, and used a shape-heavy probability path. M6b suppresses this by:
  - evaluating exactly one parent per step: the largest eligible plate
  - using a monotonic `P = 1 - exp(-lambda)` model
  - clamping automatic rifts to binary splits only

### M6a vs M6b

| Metric | M6a | M6b | Delta |
|---|---:|---:|---:|
| total rifts | `22` | `4` | `-18` |
| first rift step | `1` | `16` | later |
| total collisions | `21` | `14` | `-7` |
| final plate count | `49` | `11` | `-38` |
| max plate count | `49` | `11` | `-38` |
| min / max / final continental area fraction | `0.000233 / 0.308400 / 0.000233` | `0.003717 / 0.291033 / 0.003717` | improved, still bad |
| min / max / final gap rate | `0.000000 / 0.230067 / 0.058267` | `0.000000 / 0.122483 / 0.097067` | peak improved, final worsened |
| max components per plate | `37` | `21` | `-16` |
| stable plate ids valid | `0` | `1` | fixed |

### M6b checkpoints

| Step | Trigger / Ownership | Boundary fraction | Gap count | Overlap count | Continental count | Mean elevation |
|---|---|---:|---:|---:|---:|---:|
| `50` | `CollisionFollowup / FullResolution` | `0.061150` | `0` | `0` | `14265` | `-2.914449` |
| `100` | `CollisionFollowup / FullResolution` | `0.068033` | `0` | `0` | `7983` | `-2.983634` |
| `150` | `CollisionFollowup / FullResolution` | `0.101317` | `0` | `0` | `4241` | `-3.104574` |
| `200` | `Periodic / PreserveOwnership` | `0.080950` | `5824` | `43` | `223` | `-3.600100` |

### M6b interpretation

- The rift cascade is materially suppressed and plate identity is now valid.
- Continental collapse is improved, but it is still severe.
- The next clear tuning target after M6b is **continental balance**, not another identity/rift-regime pass.

## Provisional Numeric Scorecard

### Direct thresholds from the architecture docs

| Criterion | Threshold | Baseline status |
|---|---|---|
| overlap rate | `< 5%` | **Pass** at final `0.125%`; worst observed `2.3033%` |
| gap rate | `< 10%` | **Fail** worst observed `23.0067%`; final `5.8267%` |
| continental area fraction over long runs | `20–40%` target band over M6-scale runs | **Fail** by step `100+`; final `0.0233%` |
| no fragmented plates | qualitative, backed by `max_components_per_plate` | **Fail** (`37`) |
| valid `PlateId` / no empty active plates | invariant | **Fail** (`stable_plate_ids_valid=0`) |
| no ownership changes outside resampling | invariant | **Pass** (`0`) |
| exporter/sample mismatch for continental fraction | `< 5%` | **Not evaluated here**; exporter cross-check not yet wired into the harness |

### Criteria that remain baseline-derived for now

These do **not** have fixed architecture thresholds yet. They are first recorded here so tuning can compare against a stable before-state.

| Criterion | Baseline read |
|---|---|
| first rift step | `1` |
| first collision step | `20` |
| rift event count over 200 steps | `22` |
| collision event count over 200 steps | `21` |
| plate-count evolution | `7 -> 49` by step `200` |
| boundary sample fraction envelope | `0.041533 -> 0.192333 -> 0.043483` |
| mean elevation envelope | `-4.111436 -> -2.580058 -> -3.944098` |

## First Tuning Candidates

These are the clearest problems in the current baseline and should be first-class M6 tuning candidates:

1. Automatic rifting is too eager.
   - First rift at step `1`
   - `22` rift events by step `200`
   - Plate count grows from `7` to `49`

2. Continental area collapses.
   - `0.308400` max early in the run
   - `0.000233` final at step `200`

3. Gap rate is not bounded tightly enough.
   - Worst observed `23.0067%`

4. Plate fragmentation remains severe.
   - `max_components_per_plate_observed = 37`
   - `stable_plate_ids_valid = 0`

5. Collision/rift cadence is probably not yet producing a plausible tectonic cycle.
   - The cycle is "observable" in the sense that both events happen repeatedly, but the current event frequency and continental collapse indicate the system is not yet healthy.

## Visual Validation Status

- The harness exported trustworthy maps at steps `0`, `50`, `100`, `150`, and `200`:
  - `PlateId`
  - `BoundaryMask`
  - `CrustType`
  - `ContinentalWeight`
  - `Elevation`
- Export root:
  - `Saved/MapExports/M6Baseline60k7-seed42-samples60000-plates7`
- `GapMask` and `OverlapMask` were intentionally **not** used for M6a validation because the current exporter paths are still placeholder/non-trustworthy.
- Visual validation for gap/overlap fields still needs a separate exporter fix or manual/editor-side inspection.

## Multi-Seed and 500k Next Hooks

### Multi-seed

- `Aurous.TectonicPlanet.M6Baseline60k7` now accepts a seed through its `Parameters` string.
- Seed `42` is the frozen first baseline.
- Future M6 acceptance should rerun the same harness on additional seeds instead of introducing a different test path.

### 500k spot-check

- The new `[ResampleMetrics ...]` and `[ResamplePhaseTiming ...]` lines are emitted by production code, so existing `500k` harnesses can now act as behavior spot-checks without new logging work.
- The immediate hook is:
  - `Aurous.TectonicPlanet.StabilityAfterFiveResamplings500k40`
- That is **not** the full M6 acceptance run; it is the prepared scale-survival spot-check path while tuning still happens at `60k`.
