# Aurous Evidence Manifest

This manifest indexes validation artifacts that matter for the current tectonic
architecture. It does not make generated images authoritative by themselves;
acceptance still comes from tests and raw diagnostics.

## Load-Bearing Evidence

| Evidence | Commit | Location | What It Supports |
| --- | --- | --- | --- |
| Prototype C 60k/40 checkpoints | `0ff3ff0` | `Saved/MapExports/SidecarPrototypeC/60k40/step_000`, `step_100`, `step_200`, `step_400` | Visual confirmation of clean owner loops, coherent material transport, and long-horizon C projection stability. |
| Prototype C 250k/40 smoke | `0ff3ff0` | `Saved/MapExports/SidecarPrototypeC/250k40_smoke/step_040` | Higher-resolution smoke: exports succeed, owners valid, mandatory overlays present, no fabricated material. |
| Prototype C freeze automation log | `0ff3ff0` | `Saved/Logs/SidecarPrototypeC_FreezeHardening.log` | C freeze tests, including independent owner/boundary recomputation and 250k validity. |
| Prototype A diagnostic automation log | `0ff3ff0` | `Saved/Logs/SidecarPrototypeA_FreezeHardening.log` | A remains runnable as a diagnostic harness. |
| Prototype B diagnostic automation log | `0ff3ff0` | `Saved/Logs/SidecarPrototypeB_FreezeHardening.log` | B remains runnable as a projection proof harness. |
| Architecture failure memo | docs commit history | `docs/tectonic-architecture-failure-memo-2026-04.md` | Explains why V6/V9 ownership authority and remesh repair are not active architecture. |

## Mandatory Prototype C Overlay Files

For C acceptance exports, each exported checkpoint should include:

- `BoundaryMask.png`
- `PlateId.png`
- `ContinentalWeight.png`
- `MaterialClassification.png`
- `MaterialSource.png`
- `MaterialOwnerMismatch.png`
- `MaterialOverlap.png`
- `DivergentBoundary.png`

Base Mollweide maps are useful visual evidence. The sidecar material overlays are
the honesty maps that prevent material projection from being mistaken for owner
authority.

## Historical Evidence

Keep for reference, not as active validation:

- V6/V9 step runs cited by `docs/tectonic-architecture-failure-memo-2026-04.md`.
- Older paper-faithful remesh prototype logs.
- Prototype A and B map exports beyond the current hardening logs.
- v4-v9 architecture notes and M1-M5 lessons.

## Prunable Candidates

Do not delete in this cleanup pass. Candidate categories for a later explicit
pruning decision:

- unreferenced old `Saved/MapExports` runs
- repeated A/B exports not cited by docs or tests
- stale log files with no linked memo or commit hash

Before pruning, update this manifest and preserve any artifact referenced by an
accepted ADR, failure memo, or `docs/STATE.md`.

## Evidence Policy

- Every load-bearing artifact should list the commit that produced it.
- New D evidence should be added here in the same PR that updates `docs/STATE.md`.
- PNGs are secondary evidence. Tests and raw diagnostics are the acceptance source.
- Filled/rasterized exports must not be used alone to prove raw C invariants.
