# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Aurous is an Unreal Engine 5.7 project implementing procedural tectonic planet generation based on Cortial et al., "Procedural Tectonic Planets" (2019) and "Real-Time Hyper-Amplification of Planets" (2020). The project is in early stages — the game module currently contains only bootstrap code.

## Build & Development

- **Engine:** Unreal Engine 5.7 (set via `EngineAssociation` in `Aurous.uproject`)
- **Open in editor:** Launch `Aurous.uproject` from Epic Launcher
- **Build editor (CLI):** `Engine/Build/BatchFiles/Build.bat AurousEditor Win64 Development -Project="<path>/Aurous.uproject" -WaitMutex`
- **Build from VS:** Open `Aurous.sln`, build `AurousEditor` in `Development Editor | Win64`
- **Run tests:** `UnrealEditor-Cmd.exe "<path>/Aurous.uproject" -unattended -nop4 -ExecCmds="Automation RunTests RealtimeMesh; Quit"`
- **Game module tests** go in `Source/Aurous/Private/Tests/`

## Code Style

- Follow Unreal/Epic C++ conventions: tabs, `PascalCase` for types/methods/properties
- Unreal prefixes: `A` (Actor), `U` (UObject), `F` (struct/value type), `I` (interface)
- Module uses explicit PCH (`PCHUsageMode.UseExplicitOrSharedPCHs`), no Unity builds (`bUseUnity = false`)
- Prefer config in `Config/Default*.ini` over hardcoded values
- Commits: short imperative subjects (e.g., "Add tectonic planet mesh generator actor")

## Module Structure

- **`Source/Aurous/`** — Primary game module. Dependencies: Core, CoreUObject, Engine, InputCore, EnhancedInput
- **`Plugins/RealtimeMeshComponent/`** — Bundled third-party plugin (v5.3.2) for runtime mesh generation. 8 modules including runtime, editor, tests, examples, spatial, nanite, and extensions. Do not modify unless necessary.
- **`Config/`** — Engine configured for DX12 SM6, ray tracing, Substrate material system, Virtual Shadow Maps

## Architecture — Procedural Tectonic Planets

The full architecture spec is in `tectonic-planets-architecture-v4.md` (735 lines). Key concepts:

### Core Principle
The planet is a **fixed canonical sample set** (60k–500k Fibonacci sphere points) that never moves or changes topology. Plates are rigid bodies that carry copies of sample data as they rotate. Periodically, canonical samples are repopulated from plate data via **reconciliation** (ownership reassignment + barycentric interpolation).

### Data Flow
1. **Global Spherical Delaunay Triangulation** — built once via `TConvexHull3<double>::Solve()` (UE5 GeometryCore). Provides adjacency, render mesh, and source topology for plate triangle soups. Never rebuilt.
2. **Plate Triangle Soups** — each plate gets interior triangles (all 3 vertices same plate) from the global triangulation, rotated by the plate's rigid motion. Boundary triangles (spanning plates) are discarded.
3. **BVH Containment** — Cartesian AABB BVH per plate over rotated soups. Broad-phase culling via spherical bounding caps. Point-in-triangle via `orient3d` using **Shewchuk's adaptive-precision predicates** (required for robustness at 500k).
4. **7-Phase Reconciliation** — soup/BVH build → ownership reassignment (with boundary hysteresis) → barycentric interpolation → gap detection (new oceanic crust) → overlap/convergence classification → membership update → terrane detection.

### Tectonic Processes
- **Subduction:** Continuous uplift via piecewise cubic distance profiles, BFS distance-to-front on Delaunay graph, slab pull rotation correction
- **Continental Collision:** Discrete event — terrane detachment, elevation surge with compactly supported falloff, Himalayan orogeny
- **Oceanic Generation:** Gap samples become new oceanic crust with ridge elevation blending
- **Plate Rifting:** Poisson-triggered fracture into sub-plates via spherical Voronoi + noise

### Threading Model
- Simulation on background thread (UE5 async task)
- Reconciliation writes to new buffer; atomic swap with render buffer (double-buffered)
- Per-sample operations parallelized via `ParallelFor`
- Compile with `/fp:strict` (MSVC) for deterministic floating-point

### Interpolation Conventions
- Continuous fields (elevation, thickness, age): standard barycentric blend
- Categorical fields (crust_type, orogeny_type): majority vote
- Direction fields (fold/ridge direction): blend 3D Cartesian vectors, project onto tangent plane at sample position, renormalize

### Key Parameters
- Timestep: 2 My; Planet radius: 6370 km
- Max plate speed: 100 mm/year; Subduction influence: 1800 km; Collision influence: 4200 km
- Hysteresis threshold: 0.1–0.2 (empirical tuning)
- Full parameter table in architecture doc Section 11

## Milestone Roadmap

The project follows milestones 0–8 defined in the architecture doc (Section 10):
- **M0:** Fibonacci sampling, Delaunay triangulation, plate init, UE5 mesh rendering
- **M1:** Plate motion, full 7-phase reconciliation with BVH + Shewchuk predicates
- **M2:** Oceanic crust generation at divergent boundaries
- **M3:** Subduction with uplift, distance fields, slab pull
- **M4:** Continental collision with terrane tracking
- **M5:** Plate rifting
- **M6:** Full simulation tuning
- **M7:** GPU hyper-amplification (hierarchical subdivision, river networks)
- **M8:** Interactive authoring tools
