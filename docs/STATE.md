# Aurous Project State

Last update: 2026-04-28
Baseline commit: `0ff3ff0` (`Harden Prototype C freeze invariants`)
Active branch at update time: `codex/sidecar-c-actor-control-panel`

## Active Architecture

Prototype C is the active foundation: Voronoi ownership plus decoupled material
inside the plate-authoritative sidecar. Prototype D is active on top of C for
persistent divergent ocean crust through Slice 6: event-owned crust identity,
projection overlays, long-horizon stress evidence, and paper-cited linear
age-derived ocean-crust cooling.

Canonical contract:
`docs/architecture/decisions/0001-c-freeze-voronoi-ownership-decoupled-material.md`

D contracts:

- `docs/architecture/decisions/0002-d-persistent-divergent-ocean-crust.md`
- `docs/architecture/decisions/0003-d-oceanic-crust-cooling-law.md`

Short version:

- owner = nearest currently rotated plate center, lower `PlateId` tie-break
- boundary = raw sample adjacency owner change
- material is carried separately and projected for output
- material/owner mismatch is diagnostic evidence only
- `FTectonicPlanet` is output/cache, not tectonic authority
- V6/V9 remesh and ownership repair are historical, not active

## What Is Validated

Prototype C freeze tests pass at the baseline commit.

Prototype D Slice 6 passes local validation on the active branch.

Validated evidence:

- `AurousEditor` builds.
- `Aurous.TectonicPlanet.SidecarPrototypeA` passes as a diagnostic harness.
- `Aurous.TectonicPlanet.SidecarPrototypeB` passes as a diagnostic harness.
- `Aurous.TectonicPlanet.SidecarPrototypeC` passes freeze tests.
- C exports exist for 60k/40 at steps 0, 100, 200, and 400.
- C exports exist for 250k/40 smoke at step 40.
- All mandatory C overlays are exported: material classification, material source,
  material/owner mismatch, material overlap, and divergent boundary.
- `Aurous.TectonicPlanet.SidecarPrototypeD` passes with the exact anchored
  automation filter `^Aurous.TectonicPlanet.SidecarPrototypeD$`.
- `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate` passes and exports D
  overlays through step 400.
- `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon` passes 60k/40 through
  step 1000.
- Slice 6 exports `OceanCrustElevation.png` and validates projected D elevation
  against `clamp(-1.0 - 0.04 * AgeMy, -6.0, -1.0)`.
- Visual review verified the cooling gradient: young ridge crust is bright and
  older abyssal crust darkens as expected.

Key hardening metrics:

| Metric | Value |
| --- | --- |
| C automation runtime | about 32 seconds |
| D exact automation runtime | about 30 seconds |
| D visual gate runtime | about 87 seconds |
| D long-horizon 60k/40 runtime | about 110 seconds |
| Rigid expected drift | 5600.00 km |
| Rigid projected error | 4.88 km |
| Rigid projected tolerance | 368.81 km |
| 60k/40 mean drift at step 40 | 929.17 km |
| 250k/40 mean drift at step 40 | 876.35 km |
| 60k/40 boundary fraction | 0.10177 |
| 250k/40 boundary fraction | 0.05032 |

## What Is Pending

Priority order:

1. Draft the next ADR for Prototype E: subduction / crust consumption.
2. Preserve C invariants while adding any E behavior.
3. Treat D balance limits as expected until subduction exists; D creates crust
   and does not consume it.
4. Decide whether 5b high-resolution D long-horizon evidence is needed before E.
5. Track, but do not rush, extraction of C into its own class once A/B can be deprecated.

Prototype C does not solve persistent ocean crust. Prototype D now solves the
first persistent divergent-ocean-crust slice, including age-derived cooling, but
does not solve subduction, collision, rifting, uplift, erosion, or long-horizon
land/ocean balance.

## Reproduce The C Baseline

1. Build `AurousEditor`.
2. Run automation group `Aurous.TectonicPlanet.SidecarPrototypeC`.
3. For visual evidence, export maps with the Sidecar C actor or control panel.
4. Expected local artifact roots:
   - `Saved/Logs/SidecarPrototypeC_FreezeHardening.log`
   - `Saved/MapExports/SidecarPrototypeC/60k40/`
   - `Saved/MapExports/SidecarPrototypeC/250k40_smoke/`
5. Compare runtime and metrics against the validation table above.
6. Verify mandatory overlays listed in `docs/evidence/MANIFEST.md`.

`Saved/` is gitignored. These artifacts are local reproducibility evidence, not
committed repository files.

## Reproduce The D Visual Gate

1. Build `AurousEditor`.
2. Run automation group `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate`.
3. Expected local artifact roots:
   - `Saved/MapExports/SidecarPrototypeD/visual_60k40/step_000/`
   - `Saved/MapExports/SidecarPrototypeD/visual_60k40/step_100/`
   - `Saved/MapExports/SidecarPrototypeD/visual_60k40/step_200/`
   - `Saved/MapExports/SidecarPrototypeD/visual_60k40/step_400/`
   - `Saved/MapExports/SidecarPrototypeD/visual_60k40_below_threshold/`
4. Generate review sheets with `$aurous-overlay-contactsheet` for each
   checkpoint directory.
5. Compare checkpoint drift with `$aurous-diagnostic-diff`, especially
   `step_100 -> step_200` and `step_200 -> step_400`.
6. Answer the Slice 4 visual checklist in
   `docs/process/prototype-d-slice-4-visual-check.md`.

## Reproduce The D Long-Horizon Stress Gate

Slice 5a:

1. Build `AurousEditor`.
2. Run automation group `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon`.
3. Expected local artifact root:
   - `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/`
4. Generate contact sheets with `$aurous-overlay-contactsheet` for steps 0, 100,
   200, 400, 800, and 1000.
5. Compare drift with `$aurous-diagnostic-diff`, especially
   `step_400 -> step_800` and `step_800 -> step_1000`.
6. Answer the Slice 5 checklist in
   `docs/process/prototype-d-slice-5-long-horizon-check.md`.

Slice 5b:

- Run automation group `Aurous.TectonicPlanet.SidecarPrototypeDHighResLongHorizon`
  when overnight or separate-session machine time is available.
- Expected local artifact roots:
  - `Saved/MapExports/SidecarPrototypeD/long_horizon_100k40/`
  - `Saved/MapExports/SidecarPrototypeD/long_horizon_250k40/`
  - `Saved/MapExports/SidecarPrototypeD/long_horizon_500k40/`

## Reproduce The D Cooling-Law Gate

1. Build `AurousEditor`.
2. Run the exact core D filter:
   - `^Aurous.TectonicPlanet.SidecarPrototypeD$`
3. Run:
   - `Aurous.TectonicPlanet.SidecarPrototypeDVisualGate`
   - `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon`
4. Confirm `OceanCrustElevation.png` exists in the D visual and long-horizon
   checkpoint exports.
5. Generate contact sheets with `$aurous-overlay-contactsheet`. The standard
   contact sheet includes `OceanCrustElevation.png`.
6. Interpret black in `OceanCrustElevation.png` by cross-referencing
   `OceanCrustAge.png` or `OceanCrustId.png`: black can mean either no projected
   D crust or abyssal D crust at `-6 km`.
7. Verify the visual gradient: young ridge crust should be bright and old crust
   should darken toward abyssal depth.

## Repository Hygiene Snapshot

Last checked: 2026-04-27.

- Remote heads: `main`, `v5-m2b`, `codex/sidecar-c-actor-control-panel`.
- Local branches also include historical/stale `master`, `v5-m0a`, `v5-m2a`,
  `claude/*`, and active `codex/sidecar-c-actor-control-panel`.
- Active worktrees: main repo, `claude/relaxed-easley-255895`,
  `claude/stoic-edison-882d64`.
- Recommendation: treat `main` as canonical unless the user decides otherwise;
  do not delete local or remote branches without an explicit cleanup request.

## Falsified Or Superseded Directions

- V6/V9 ownership-authority: falsified by the April 2026 architecture failure memo.
- Query/remesh ownership as material authority: falsified by churn, anchoring, and boundary-area failures.
- Prototype A material-support diagnostic: preserved as a diagnostic harness, not an architecture.
- Prototype B explicit-footprint projection: preserved as a projection proof, not the ownership foundation.
- Older v4-v9 architecture docs: historical unless explicitly referenced by the current ADRs.

## Resolution Coverage

| Resolution | Current C/D coverage |
| --- | --- |
| 60k/40 | C freeze tests and D Slice 6 visual/long-horizon checkpoints exported |
| 100k | Not yet run on C |
| 250k/40 | Smoke step 40 exported and validated |
| 500k | D high-res long-horizon artifacts exist from Slice 5b, but 5b is confidence evidence rather than a Slice 6 blocker |

## Reading Order

1. `docs/STATE.md`
2. `docs/architecture/decisions/0001-c-freeze-voronoi-ownership-decoupled-material.md`
3. `docs/architecture/decisions/0002-d-persistent-divergent-ocean-crust.md`
4. `docs/architecture/decisions/0003-d-oceanic-crust-cooling-law.md`
5. `docs/tectonic-minimal-acceptance-tests.md`
6. `docs/tectonic-architecture-failure-memo-2026-04.md`
7. `docs/tectonic-architecture-reset-plate-authoritative-prototype.md`
8. `docs/ProceduralTectonicPlanets.txt`
