# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Aurous is an Unreal Engine 5.7 project implementing procedural tectonic planet generation based on Cortial et al., "Procedural Tectonic Planets" (2019) and the Cortial PhD thesis "Synthèse de terrain à l'échelle planétaire" (2020). The project implements a v9 ownership-authority tectonic simulation with thesis-scale continental relief at 60k samples / 40 plates.

**Relationship to the paper:** This is a paper-inspired successor architecture, not a strict reimplementation. The v9 ownership-authority model intentionally diverges from the paper's query-driven repartition (which caused 50%+ churn at our scale). The substrate, constants, tectonic processes, and remesh algorithm are paper-faithful; the ownership model is our own design, documented in `docs/tectonic-planets-architecture-v9.md`.

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

**Divergence from the paper:** The paper uses query-driven repartition — each remesh re-queries all samples and assigns ownership based on plate mesh intersection. This caused unacceptable churn (50%+) at our scale. The v9 model retains ownership by default and only re-evaluates samples in the active zone (~5.8%). This is a deliberate architectural choice, not a bug.

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
5. **Tectonic maintenance** — post-remesh convergent-margin continental recovery, locality-gated to active band.
6. **Active-band field continuity** — field clamping and previous-owner-compatible recovery at remesh to prevent elevation/CW discontinuities under thesis-scale relief.

### Tectonic Processes

**All four tectonic processes implemented and active in V6 solve path:**
- **Subduction:** Paper-aligned uplift ũ = u₀·f(d)·g(v)·h(z̃), BFS distance-to-front, slab pull rotation correction
- **Oceanic Generation:** Frontier-pair fill with ridge-shaped elevation profile
- **Continental Collision:** Shadow tracker with persistent pair qualification, bounded multi-event execution (2-3 per remesh), coherent terrane detection via bounded BFS, paper-faithful radial biweight kernel elevation surge. Validated at step 200 with accumulating mountain-building. Ridge surge variant behind toggle (paper deviation, not default).
- **Plate Rifting:** Poisson-triggered fracture into sub-plates via spherical Voronoi + noise. Wired into V6 with deferred follow-up to copied-frontier remesh. Post-rift V6 path proven end-to-end (forced rift validation). Default Poisson rate is low at 60k — will fire more naturally at higher resolution.
- **Erosion:** Continental erosion (ε_c = 0.03 mm/yr), oceanic damping (ε_o = 0.04 mm/yr), trench accretion — all paper-aligned and active every timestep.

### Key Parameters (paper-aligned, all verified 2026-04-11)
- Timestep: 2 My; Planet radius: 6371 km
- **u₀ = 0.6 km/My** (subduction base uplift); h(z̃) = z̃² (elevation transfer); g(v) = v/v₀ (speed transfer)
- f(d) = cubic distance transfer with control distance 0.10 rad, max distance 0.283 rad (~1800 km)
- **Initial plate speed: 5–20 mm/yr** (grows via slab pull); Max plate speed (v₀): 100 mm/yr
- Subduction influence (r_s): 1800 km; Collision influence (r_c): 4200 km
- Erosion: ε_c × δt = 0.06 km/step; Damping: ε_o × δt = 0.08 km/step; Accretion: 0.6 km/step
- Elevation ceiling: 10 km; Trench: -10 km; Abyssal plain: -6 km
- Collision coefficient: 0.013; Collision biweight kernel: f(x) = (1-(x/r)²)²
- Standard test regime: **60k samples, 40 plates, seed 42, cadence 16 steps, continental fraction 0.30**
- Remesh cadence: 16 steps (32 Ma) — fixed, not adaptive (paper uses adaptive 16–64 steps)

### Current Validation (step 100, 60k/40, as of 2026-04-11)
- Ownership churn: 2.4%
- Boundary coherence: 0.9403
- Interior leakage: 15.9%
- Continental mean elevation: 1.30 km
- Continental p95 elevation: 4.65 km
- Samples above 2 km: 4,017; above 5 km: 530+
- Collision events: 8 cumulative, coherent terrane transfer active
- All 40 plates active, balanced participation

### Paper Alignment Audit (2026-04-11)
- **Erosion/Elevation/Motion:** ALL MATCH — every parameter verified against thesis Table 3.2
- **Remesh:** HIGH FIDELITY — copied frontier, ray/BVH query, barycentric transfer, frontier-pair fill, destructive exclusion, multi-hit coherence resolution all match
- **Subduction:** ~95% match — full uplift formula ũ = u₀·f·g·h implemented, cubic f(d) confirmed paper-aligned
- **Collision:** PARTIAL — biweight kernel, terrane BFS, surge formula match; execution connected but still underpowered; detection approach differs (shadow tracker vs paper discrete events)
- **Rifting:** Paper-faithful in legacy code, disabled in V6 path
- **Ownership model:** DELIBERATE DIVERGENCE — v9 ownership authority vs paper query-driven repartition

### Known Simplifications vs Paper
- **Fixed remesh cadence** (16 steps) instead of paper's adaptive formula
- **60k samples** for iteration; paper showcases at 300k-500k
- **Rendering** shows colorized sphere, not elevation-displaced terrain (hyper-amplification is a later milestone)

## File Organization

### Core Implementation
- **`Source/Aurous/Public/TectonicPlanet.h`** — Legacy planet struct (FSample, FPlate, FCarriedSample, tectonic process declarations)
- **`Source/Aurous/Private/TectonicPlanet.cpp`** — Legacy tectonic processes (subduction, collision, rifting, slab pull, resampling, erosion)
- **`Source/Aurous/Public/TectonicPlanetV6.h`** — V6/V9 solve modes, diagnostics, active zone classifier, ownership authority, collision execution
- **`Source/Aurous/Private/TectonicPlanetV6.cpp`** — V6/V9 solve path (remesh, fill, classifier, collision execution, tectonic maintenance, all diagnostic computation)

### Tests
- **`Source/Aurous/Private/Tests/TectonicPlanetV6Tests.cpp`** — V6/V9 test harnesses (Phase 1d validation, elevation budget, collision execution, A/B comparisons, step-100/200 validation)
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
- **Collision execution** — per-event terrane size, transfer count, elevation delta, influence radius, xi
- **Continental elevation** — mean, p95, max, samples above 2 km / 5 km
- **Visual exports** — Mollweide projection PNGs (PlateId, BoundaryMask, OwnershipChurnMask, MissLineageMask, ContinentalWeight, Elevation, SubductionDistance, CollisionTransferMask, CollisionElevationDeltaMask)

## Current Priority

Long-run tectonic-cycle validation (step 500+). All four tectonic processes are connected and individually validated. Quiet-interior continental retention guard preserves broad continental mass (step-200 CAF 0.28 vs 0.19 baseline, largest component 9558 vs 2407). Next: verify the full tectonic lifecycle works together — plates split, spread, collide, accrete, and shrink in a believable cycle. Watch items: plate count evolution, natural rift incidence, continental shape quality (width not just area), collision/rift interaction over time. After long-run: optimization and resolution ladder (60k → 100k → 250k).
