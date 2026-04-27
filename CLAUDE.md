# CLAUDE.md

This file gives Claude Code and other coding agents the current repo truth before
they touch Aurous. If a statement here conflicts with `docs/STATE.md` or the
accepted ADRs, update this file immediately.

## Reading Order

If you are new to this project, read in this order:

1. `docs/STATE.md` - current status, validated gates, pending work.
2. `docs/architecture/decisions/0001-c-freeze-voronoi-ownership-decoupled-material.md` - current architectural contract.
3. `docs/tectonic-minimal-acceptance-tests.md` - acceptance gates.
4. `docs/tectonic-architecture-failure-memo-2026-04.md` - why V6/V9 failed.
5. `docs/tectonic-architecture-reset-plate-authoritative-prototype.md` - reset history and prototype ladder.
6. `docs/ProceduralTectonicPlanets.txt` - source paper notes.

## Current Architecture

Aurous is now on the plate-authoritative sidecar path. The active foundation is
Prototype C: Voronoi ownership plus decoupled material.

Prototype C invariants:

- every canonical sample owner is the nearest currently rotated plate center
- lower `PlateId` wins exact owner ties
- `bIsBoundary` is raw one-ring owner adjacency
- ownership gaps and ownership overlaps are impossible by construction
- material is carried separately from ownership and projected for output
- material/owner mismatch is diagnostic evidence only
- `FTectonicPlanet` is a disposable projection/cache, not tectonic authority
- V6/V9 ownership repair, active-zone recovery, quiet-interior retention, and generic oceanization are not part of C

V6/V9 and older remesh-authority approaches are historical. Do not extend them
for new tectonic work. Keep them buildable only as long as tests and references
need them.

## Current Validation

Baseline commit: `0ff3ff0` (`Harden Prototype C freeze invariants`).

Validated on the sidecar branch:

- `AurousEditor` builds.
- `Aurous.TectonicPlanet.SidecarPrototypeA` passes as a diagnostic harness.
- `Aurous.TectonicPlanet.SidecarPrototypeB` passes as a diagnostic harness.
- `Aurous.TectonicPlanet.SidecarPrototypeC` passes freeze tests.
- Prototype C 60k/40 checkpoints exported at steps 0, 100, 200, and 400.
- Prototype C 250k/40 smoke export succeeds at step 40.
- C automation runtime from the hardening pass: about 32 seconds.

Important C freeze metrics from that pass:

- rigid material carrier expected drift: 5600.00 km
- projected centroid error: 4.88 km
- projected tolerance: 368.81 km
- 60k/40 mean drift at step 40: 929.17 km
- 250k/40 mean drift at step 40: 876.35 km
- 60k/40 boundary fraction: 0.10177
- 250k/40 boundary fraction: 0.05032

Load-bearing visual evidence is indexed in `docs/evidence/MANIFEST.md`.

## Current Priority

Do not add new tectonic processes until Prototype C is frozen and the cleanup
docs point to one source of truth.

Next architectural work is Prototype D: persistent crust events on top of C's
stable ownership/projection foundation. D should start with its own ADR before
implementation, likely focused on persistent divergent ocean crust identity,
age, and thickness.

## Build And Test

- Engine: Unreal Engine 5.7 (`EngineAssociation` in `Aurous.uproject`).
- Build editor target with Unreal `Build.bat`.
- Run automation through `UnrealEditor-Cmd.exe` with `-NullRHI`.
- Game module tests live in `Source/Aurous/Private/Tests/`.

Useful test groups:

- `Aurous.TectonicPlanet.SidecarPrototypeA`
- `Aurous.TectonicPlanet.SidecarPrototypeB`
- `Aurous.TectonicPlanet.SidecarPrototypeC`
- `Aurous.TectonicPlanet`

## Code Style

- Follow Unreal/Epic C++ conventions.
- Use `PascalCase` for types, methods, and properties.
- Use Unreal prefixes: `A`, `U`, `F`, `I`.
- Module uses explicit/shared PCHs and no unity builds.
- Keep config in `Config/Default*.ini` when reasonable.
- Use short imperative commit subjects.

## File Organization

Active sidecar foundation:

- `Source/Aurous/Public/TectonicPlanetSidecar.h`
- `Source/Aurous/Private/TectonicPlanetSidecar.cpp`
- `Source/Aurous/Public/TectonicPlanetSidecarActor.h`
- `Source/Aurous/Private/TectonicPlanetSidecarActor.cpp`
- `Source/Aurous/Private/Tests/TectonicPlanetSidecarTests.cpp`

Historical V6/V9 path:

- `Source/Aurous/Public/TectonicPlanetV6.h`
- `Source/Aurous/Private/TectonicPlanetV6.cpp`
- `Source/Aurous/Private/Tests/TectonicPlanetV6Tests.cpp`

Shared/legacy substrate:

- `Source/Aurous/Public/TectonicPlanet.h`
- `Source/Aurous/Private/TectonicPlanet.cpp`
- `Source/Aurous/Private/TectonicMollweideExporter.cpp`

Current docs:

- `docs/STATE.md`
- `docs/architecture/decisions/`
- `docs/tectonic-minimal-acceptance-tests.md`
- `docs/tectonic-architecture-failure-memo-2026-04.md`
- `docs/evidence/MANIFEST.md`

## Repository Hygiene

After any push, verify the remote received it. Do not trust an agent-reported
"pushed" line by itself. Either:

- run `git ls-remote origin <branch>` and confirm the remote SHA matches local `HEAD`
- or visit the GitHub branch URL and confirm the latest commit

The missed push during the C review cycle blocked an external review and cost
time. Treat remote verification as part of the push.

Worktree hygiene:

- run `git worktree list` before deleting or pruning anything
- only prune entries that Git marks `prunable`
- do not delete branches or worktrees unless the user explicitly asked for that cleanup
- keep active agent worktrees until their branch has been merged or deliberately abandoned

## Plugin Notes

`Plugins/RealtimeMeshComponent/` is a bundled third-party runtime mesh plugin.
Do not modify or migrate it as part of tectonic architecture work unless that is
the explicit task.
