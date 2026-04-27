# Aurous Project State

Last update: 2026-04-27
Baseline commit: `0ff3ff0` (`Harden Prototype C freeze invariants`)
Active branch at update time: `codex/sidecar-c-actor-control-panel`

## Active Architecture

Prototype C is the active foundation: Voronoi ownership plus decoupled material
inside the plate-authoritative sidecar.

Canonical contract:
`docs/architecture/decisions/0001-c-freeze-voronoi-ownership-decoupled-material.md`

Short version:

- owner = nearest currently rotated plate center, lower `PlateId` tie-break
- boundary = raw sample adjacency owner change
- material is carried separately and projected for output
- material/owner mismatch is diagnostic evidence only
- `FTectonicPlanet` is output/cache, not tectonic authority
- V6/V9 remesh and ownership repair are historical, not active

## What Is Validated

Prototype C freeze tests pass at the baseline commit.

Validated evidence:

- `AurousEditor` builds.
- `Aurous.TectonicPlanet.SidecarPrototypeA` passes as a diagnostic harness.
- `Aurous.TectonicPlanet.SidecarPrototypeB` passes as a diagnostic harness.
- `Aurous.TectonicPlanet.SidecarPrototypeC` passes freeze tests.
- C exports exist for 60k/40 at steps 0, 100, 200, and 400.
- C exports exist for 250k/40 smoke at step 40.
- All mandatory C overlays are exported: material classification, material source,
  material/owner mismatch, material overlap, and divergent boundary.

Key hardening metrics:

| Metric | Value |
| --- | --- |
| C automation runtime | about 32 seconds |
| Rigid expected drift | 5600.00 km |
| Rigid projected error | 4.88 km |
| Rigid projected tolerance | 368.81 km |
| 60k/40 mean drift at step 40 | 929.17 km |
| 250k/40 mean drift at step 40 | 876.35 km |
| 60k/40 boundary fraction | 0.10177 |
| 250k/40 boundary fraction | 0.05032 |

## What Is Pending

Priority order:

1. Finish/merge the C cleanup and freeze documentation.
2. Write ADR 0002 for Prototype D before implementation.
3. Implement the first D slice as persistent crust events, likely divergent
   ocean crust identity, age, and thickness.
4. Keep C invariants green while D reads C signals.
5. Track, but do not rush, extraction of C into its own class once A/B can be deprecated.

Prototype C does not solve persistent ocean crust, age gradients, thickness
accumulation, subduction, collision, rifting, uplift, erosion, or long-horizon
land/ocean balance.

## Falsified Or Superseded Directions

- V6/V9 ownership-authority: falsified by the April 2026 architecture failure memo.
- Query/remesh ownership as material authority: falsified by churn, anchoring, and boundary-area failures.
- Prototype A material-support diagnostic: preserved as a diagnostic harness, not an architecture.
- Prototype B explicit-footprint projection: preserved as a projection proof, not the ownership foundation.
- Older v4-v9 architecture docs: historical unless explicitly referenced by the current ADRs.

## Resolution Coverage

| Resolution | Current C coverage |
| --- | --- |
| 60k/40 | Freeze tests and checkpoints 0/100/200/400 exported |
| 100k | Not yet run on C |
| 250k/40 | Smoke step 40 exported and validated |
| 500k | Not yet run on C |

## Reading Order

1. `docs/STATE.md`
2. `docs/architecture/decisions/0001-c-freeze-voronoi-ownership-decoupled-material.md`
3. `docs/tectonic-minimal-acceptance-tests.md`
4. `docs/tectonic-architecture-failure-memo-2026-04.md`
5. `docs/tectonic-architecture-reset-plate-authoritative-prototype.md`
6. `docs/ProceduralTectonicPlanets.txt`
