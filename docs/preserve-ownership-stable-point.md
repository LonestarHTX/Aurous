# Preserve Ownership Stable Point

**Date:** March 17, 2026
**Scope:** Ownership refresh architecture after M2/M3/M4 experimentation
**Status:** Current stable point for ongoing tectonic work

## Decision

Keep `PreserveOwnershipPeriodic` as the periodic maintenance architecture.

This is the first mode that:
- keeps boundaries clean enough to look structurally coherent
- keeps maintenance healthy (`gap_count=0`, `overlap_count=0` in the 60k/7/42 decision run)
- still allows controlled ownership change during periodic refresh

Do not reopen general ownership architecture unless a later tectonic event proves it necessary.

## What We Proved

### 1. Periodic full resampling is the fragmentation accumulator

The resampling cadence A/B showed:
- `PeriodicFull` step 10 boundary fraction: `0.111733`
- `PeriodicFull` step 50 boundary fraction: `0.177383`
- single-resample-only step 10 and step 50 `BoundaryMask` were byte-identical

Conclusion:
- repeated periodic ownership rewrites are what accumulate the spaghetti boundary field

### 2. Pure event-driven mode is only a causality test

`EventDrivenOnly` kept boundaries clean because ownership stopped changing, but it also let maintenance fail:
- step 50 boundary fraction: `0.041533`
- step 50 gap count: `12623`
- step 50 overlap count: `2978`

Conclusion:
- freezing ownership is not a shippable solution

### 3. Overlap-only stabilization is too weak

`HybridStablePeriodic` preserved maintenance but only modestly improved boundaries:
- step 50 boundary fraction: `0.167267` vs `0.177383` for `PeriodicFull`

Conclusion:
- the problem is broader than explicit overlap winner logic

### 4. Preserve-ownership periodic refresh is the first strong result

`PreserveOwnershipPeriodic` keeps current ownership by default:
- same-plate exact containment first
- same-plate nearest-triangle recovery second
- full multi-plate ownership query only as fallback

Decision-run results at 60k / 7 plates / seed 42:

| Mode | Step | Boundary Fraction | Gap | Overlap | Continental |
|---|---:|---:|---:|---:|---:|
| `PeriodicFull` | 50 | `0.177383` | 0 | 0 | 14902 |
| `PreserveOwnershipPeriodic` | 50 | `0.047583` | 0 | 0 | 13917 |
| `PeriodicFull` | 100 | `0.187700` | 0 | 0 | 12708 |
| `PreserveOwnershipPeriodic` | 100 | `0.050050` | 0 | 0 | 11495 |
| `PeriodicFull` | 110 | `0.189983` | 0 | 0 | 12109 |
| `PreserveOwnershipPeriodic` | 110 | `0.049183` | 0 | 0 | 11232 |

Preserve-mode periodic refresh is not frozen:
- about `54k` samples refresh from the same plate by exact containment each resample
- about `500` samples refresh from the same plate by nearest-triangle recovery
- about `5.4k-6.0k` samples still need fallback full query
- about `2.3k-3.0k` samples still change `plate_id` during a maintenance resample

Conclusion:
- this is a real maintenance mode, not a map-freezing trick

## Current Interpretation

The ownership problem is no longer the main blocker.

The M4 event path was completed in two preserve-specific stages:
- boundary-contact persistence trigger
- cached boundary-contact collision execution in the same-step `CollisionFollowup`

In the updated decision run:
- `PreserveOwnershipPeriodic` reaches a real `collision_count > 0` event by step 100
- the first preserve collision executes from cached boundary-contact evidence on pair `(1,3)`
- preserve mode remains dramatically cleaner than `PeriodicFull` through the collision event

Representative updated checkpoints:

| Mode | Step | Boundary Fraction | Continental | Gap | Overlap |
|---|---:|---:|---:|---:|---:|
| `PeriodicFull` | 100 | `0.187700` | 12708 | 0 | 0 |
| `PreserveOwnershipPeriodic` | 100 | `0.058117` | 11594 | 0 | 0 |
| `PeriodicFull` | 110 | `0.189983` | 12109 | 0 | 0 |
| `PreserveOwnershipPeriodic` | 110 | `0.056567` | 11050 | 0 | 0 |

Preserve collision execution at step 100:
- `used_cached_boundary_contact_collision=true`
- `seed_count=3`
- `terrane_seed_count=2`
- `terrane_recovered=4289`
- `surge_affected=231`
- `surge_radius_rad=0.123714`
- `surge_mean_elev_delta=3.708039`

Conclusion:
- `PreserveOwnershipPeriodic` is now strong enough to treat as the stable maintenance architecture
- M4 is functionally implemented at the 60k correctness tier, even if collision plausibility/tuning still deserves another pass later

## Stable Point

Treat the following as stable until disproven by a better event-trigger result:

1. `PreserveOwnershipPeriodic` is the preferred periodic maintenance architecture.
2. `PeriodicFull` remains the baseline/reference mode for comparison only.
3. Full ownership resolution should be reserved for structural events such as collision follow-up and future rifting.
4. Boundary-contact persistence + cached boundary-contact collision execution are the preserve-mode M4 event path.
5. `EventDrivenOnly`, `HybridStablePeriodic`, and overlap-hysteresis remain useful experiment coverage, not preferred production behavior.

## Paper Alignment

This is a deliberate engineering divergence from the most literal reading of the paper's periodic resampling rule.

The paper/reference model still guides:
- fixed canonical samples
- rigid plate motion between resamples
- periodic global resampling
- barycentric transfer from moved plate geometry
- discrete collision/rifting events

The current divergence is narrower:
- during periodic maintenance, ownership is preserved by default when the current plate can still refresh the sample
- broad ownership reassignment is deferred to explicit structural events

This is acceptable as long as it continues to produce the target product:
- coherent plate boundaries
- healthy maintenance
- collision/rifting events that still produce paper-like tectonic evolution

## Immediate Follow-Up

Pause large architectural changes.

The immediate work after this stable point should be:
- clean up docs, comments, and test surface around the chosen resampling architecture
- keep M4 tuning/plausibility concerns separate from ownership architecture
- only then decide whether to revisit collision shaping before starting M5
