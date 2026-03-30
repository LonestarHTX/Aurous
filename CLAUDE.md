# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Aurous is an Unreal Engine 5.7 project implementing procedural tectonic planet generation based on Cortial et al., "Procedural Tectonic Planets" (2019) and the Cortial PhD thesis "Synthèse de terrain à l'échelle planétaire" (2020). The project implements a v9 ownership-authority tectonic simulation with stable boundary coherence at 60k samples / 40 plates.

## Build & Development

- **Engine:** Unreal Engine 5.7 (set via `EngineAssociation` in `Aurous.uproject`)
- **Open in editor:** Launch `Aurous.uproject` from Epic Launcher
- **Build editor (CLI):** `Engine/Build/BatchFiles/Build.bat AurousEditor Win64 Development -Project="<path>/Aurous.uproject" -WaitMutex`
- **Build from VS:** Open `Aurous.sln`, build `AurousEditor` in `Development Editor | Win64`
- **Run tests:** `UnrealEditor-Cmd.exe "<path>/Aurous.uproject" -unattended -nop4 -NullRHI -nosplash -nosound -ExecCmds="Automation RunTests Aurous.TectonicPlanet; Quit"`
- **Game module tests** go in `Source/Aurous/Private/Tests/`

## Code Style

- Follow Unreal/Epic C++ conventions: tabs, `PascalCase` for types/methods/properties
- Unreal prefixes: `A` (Actor), `U` (UObject), `F` (struct/value type), `I` (interface)
- Module uses explicit PCH (`PCHUsageMode.UseExplicitOrSharedPCHs`), no Unity builds (`bUseUnity = false`)
- Prefer config in `Config/Default*.ini` over hardcoded values
- Commits: short imperative subjects (e.g., "Add tectonic planet mesh generator actor")

## Module Structure

- **`Source/Aurous/`** — Primary game module. Dependencies: Core, CoreUObject, Engine, InputCore, EnhancedInput
- **`Plugins/RealtimeMeshComponent/`** — Bundled third-party plugin (v5.3.2) for runtime mesh generation. Do not modify unless necessary.
- **`Config/`** — Engine configured for DX12 SM6, ray tracing, Substrate material system, Virtual Shadow Maps

## Architecture — v9 Ownership Authority Model

The full architecture spec is in `docs/tectonic-planets-architecture-v9.md`. Historical architecture docs (v6-v8) are in `docs/archive/`.

### Core Principle
The planet is a **fixed canonical sample set** (60k Fibonacci sphere points) with a global Spherical Delaunay Triangulation that never changes topology. **Ownership is persistent** — each sample has an authoritative plate owner that persists across remesh cycles. Ownership changes only through explicit tectonic events or at narrow active boundary zones. Rotated plate geometry provides evidence for event detection, not ownership authority.

### Key Architectural Invariants (v9)
1. **Interior stability** — plate interiors keep their owner by default. Interior ownership changes should be rare and attributable to named tectonic causes.
2. **Narrow active boundaries** — boundaries are thin active zones defined by plate-pair tectonic evidence (convergent/divergent velocity), not broad residual query failure regions.
3. **Explicit tectonic events** — divergence creates new crust, convergence/collision transfers ownership, rifting repartitions. Generic query misses do not revoke ownership.
4. **Geometry follows authority** — per-plate meshes are derived from the authoritative ownership field, not the other way around.

### Data Flow
1. **Global Spherical Delaunay Triangulation** — built once. Provides adjacency, render mesh, and source topology.
2. **Per-plate meshes** — rebuilt from global TDS at each remesh. Partitioned mixed-triangle mode (each triangle assigned to one plate).
3. **Active zone classifier** — Phase 1d persistent pair-local classifier identifies tectonically active plate pairs from convergent/divergent boundary evidence. Samples in the active zone get re-evaluated; samples outside keep their owner.
4. **Frontier-pair fill** — miss samples find two nearest plate frontier points (q1, q2), compute ridge midpoint, synthesize structured oceanic crust.
5. **Tectonic maintenance** — post-remesh convergent-margin continental recovery.

### Tectonic Processes (implemented)
- **Subduction:** Continuous uplift via distance transfer function, BFS distance-to-front, slab pull rotation correction
- **Continental Collision:** Discrete event — terrane detachment, elevation surge with compactly supported falloff (collision pipeline NOT YET wired into V6 solve path — next priority)
- **Oceanic Generation:** Frontier-pair fill with ridge-shaped elevation profile
- **Plate Rifting:** Poisson-triggered fracture into sub-plates via spherical Voronoi + noise (event pipeline not yet wired into V6 solve path)

### Key Parameters (thesis-aligned)
- Timestep: 2 My; Planet radius: 6371 km
- **Initial plate speed: 5–20 mm/yr** (grows via slab pull); Max plate speed (v₀): 100 mm/yr
- Subduction influence (r_s): 1800 km; Collision influence (r_c): 4200 km
- Collision interpenetration threshold: 300 km
- Standard test regime: **60k samples, 40 plates, seed 42, cadence 16 steps, continental fraction 0.30**
- Remesh cadence: 16 steps (32 Ma) — thesis range is 16–64 steps adaptive

### Current Validation (step 200, 60k/40)
- Ownership churn: 3.8%
- Boundary coherence: 0.942
- Interior leakage: 14.5%
- Max components: 57
- CAF: 0.461
- All 40 plates active, balanced participation
- Collision count: 0 (pipeline not wired — next priority)

## File Organization

### Core Implementation
- **`Source/Aurous/Public/TectonicPlanet.h`** — Legacy planet struct (FSample, FPlate, FCarriedSample, tectonic process declarations)
- **`Source/Aurous/Private/TectonicPlanet.cpp`** — Legacy tectonic processes (subduction, collision, rifting, slab pull, resampling)
- **`Source/Aurous/Public/TectonicPlanetV6.h`** — V6/V9 solve modes, diagnostics, active zone classifier, ownership authority
- **`Source/Aurous/Private/TectonicPlanetV6.cpp`** — V6/V9 solve path (remesh, fill, classifier, all diagnostic computation)

### Tests
- **`Source/Aurous/Private/Tests/TectonicPlanetV6Tests.cpp`** — V6/V9 test harnesses (Phase 1d validation, A/B/C comparisons, regime falsification, step 200 validation)
- **`Source/Aurous/Private/Tests/TectonicPlanetSubstrateTests.cpp`** — Legacy substrate/resampling tests

### Architecture Docs
- **`docs/tectonic-planets-architecture-v9.md`** — Active architecture (ownership authority model)
- **`docs/tectonic-planets-architecture-v8.md`** — Historical (Eulerian query experiments)
- **`docs/archive/`** — Historical v6, v7 architecture memos, stale prompts, scorecards

### Reference Material
- **`docs/ProceduralTectonicPlanets/`** — Published paper (Cortial et al., 2019)
- **`docs/Synthèse de terrain à léchelle planétaire/`** — PhD thesis (Cortial, 2020)
- **`docs/driftworld-tectonics/`** — Independent implementation reference (Delong, 2022)
- **`docs/Real Time Hyper-Amplification of Planets/`** — Amplification paper (Cortial et al., 2020)

## Diagnostic Suite

The project includes comprehensive diagnostics that should be used as gates for all experiments:

- **Boundary coherence** — interior leakage, mean conflict ring distance, coherence score
- **Ownership churn** — per-remesh fraction of samples changing plate, boundary vs interior
- **Miss lineage** — repeat resynthesis tracking, synthetic crust stability
- **Coverage persistence** — owner geometry coverage audit, true-hole classification
- **Competitive participation** — plates with hits, largest share, monopoly detection
- **Active zone attribution** — cause buckets (divergence, convergent, collision, rift, generic query)
- **Visual exports** — Mollweide projection PNGs (PlateId, BoundaryMask, OwnershipChurnMask, MissLineageMask, ContinentalWeight, Elevation, SubductionDistance)

## Next Priority

Wire the collision/subduction event pipeline from TectonicPlanet.cpp into the V6 solve path. Currently the V6 path computes subduction distance fields and slab pull but does not run convergent triangle tracking, collision candidate detection, terrane detection/slab break/suture, or rifting events. Zero collisions through step 200 is a disconnected pipeline issue, not a coherence problem.
