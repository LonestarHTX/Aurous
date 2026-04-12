# Aurous

Procedural tectonic planet generation in Unreal Engine 5.7, based on Cortial et al., *Procedural Tectonic Planets* (2019) and the Cortial PhD thesis *Synthese de terrain a l'echelle planetaire* (2020).

Aurous simulates the full tectonic lifecycle -- plate motion, subduction, continental collision, oceanic crust generation, and plate rifting -- on a spherical planet with up to 250,000+ sample points and 40 tectonic plates.

## Current State

The tectonic simulation is complete and validated at multiple resolutions:

| Resolution | Samples | Per-Solve | Status |
|-----------|---------|----------|--------|
| Low (102 km) | 60,000 | 0.27s | Fast iteration rung |
| Medium (78 km) | 100,000 | 0.48s | Validated fidelity rung (3 seeds) |
| High (50 km) | 250,000 | 1.30s | Forward work rung |

All four tectonic processes are connected and produce geologically plausible results:

- **Subduction** -- paper-aligned uplift with BFS distance fields and slab pull rotation correction
- **Continental Collision** -- shadow tracker, coherent terrane detection, multi-event execution with radial biweight kernel
- **Oceanic Generation** -- frontier-pair divergence fill with ridge-shaped elevation profiles
- **Plate Rifting** -- Poisson-triggered fracture via spherical Voronoi with noise-warped boundaries

## Architecture

Aurous uses a **v9 ownership-authority model** that intentionally diverges from the paper's query-driven repartition approach. In the paper's model, every remesh re-queries all samples and assigns ownership from plate geometry. This caused 50%+ ownership churn at all tested scales (60k-500k). The v9 model makes ownership persistent by default, with re-evaluation limited to a narrow active boundary zone (~5.8% of samples).

Key architectural innovations beyond the paper:

- **Phase 1d active zone classifier** -- persistent pair-local boundary detection from convergent/divergent velocity evidence
- **Quiet-interior field preservation** -- prevents interpolation drift of CW, elevation, and thickness for stable interior samples (the root cause of continental ribbonization in naive transfer)
- **Selective elevation preservation** -- full restore for low-elevation continental plains, blended for mountains, allowing natural erosion of peaks while preventing continental sinking
- **Active-band field continuity** -- clamping and previous-owner-compatible recovery at remesh boundaries under thesis-scale relief

## Performance

Aurous exceeds the paper's published benchmarks at all tested resolutions:

| Bucket | Paper 60k | Paper 100k | Aurous 60k | Aurous 100k |
|--------|----------|-----------|-----------|------------|
| Subduction | 80ms | 140ms | 3ms | 6ms |
| Collision | 20ms | 40ms | 1ms | 2ms |
| Elevation | 90ms | 100ms | 3ms | 5ms |
| Oceanic crust | 580ms | 1,220ms | 12ms | 23ms |
| **Total (amortized/step)** | **190ms** | **280ms** | **25ms** | **46ms** |

Key optimizations: indexed decrease-key subduction BFS (177x fewer queue operations), plate-level bounding-cap pruning (93% fewer BVH queries), cached adjacency edge distances, and diagnostic attribution gating.

## Validation

Step-200 metrics at 100k samples, 40 plates:

| Metric | Value |
|--------|-------|
| Boundary coherence | 0.9726 |
| Interior leakage | 7.3% |
| Ownership churn | 2.1% |
| Continental mean elevation | 1.55 km |
| Continental p95 elevation | 3.37 km |
| Largest subaerial continent | 31,727 samples |
| Broad-seed survival | 82.8% |
| Collision events (cumulative) | 16+ |

## Paper Alignment

All physics constants verified against Cortial thesis Table 3.2:

- Timestep: 2 My | Planet radius: 6,371 km | Max plate speed: 100 mm/yr
- Subduction uplift: u0 = 0.6 km/My with f(d), g(v), h(z) transfer functions
- Erosion: 0.03 mm/yr | Oceanic damping: 0.04 mm/yr | Accretion: 0.3 mm/yr
- Subduction influence: 1,800 km | Collision influence: 4,200 km
- Elevation ceiling: 10 km | Trench depth: -10 km | Abyssal plain: -6 km

Known divergences from the paper (documented and intentional):
- Ownership-authority model instead of query-driven repartition
- Quiet-interior field preservation (paper has no equivalent)
- Fixed remesh cadence (16 steps) instead of adaptive
- Ridge surge toggle available as experimental non-paper variant

## Build

Requires Unreal Engine 5.7.

```
# Build from command line
Engine/Build/BatchFiles/Build.bat AurousEditor Win64 Development -Project="<path>/Aurous.uproject"

# Run tests
UnrealEditor-Cmd.exe "<path>/Aurous.uproject" -unattended -nop4 -NullRHI -nosplash -nosound -ExecCmds="Automation RunTests Aurous.TectonicPlanet; Quit"
```

## In-Editor Usage

1. Place a **Tectonic Planet V6 Preview** actor in your level
2. Click **Generate** in the Details panel
3. Use **Advance 1/16/100 Steps** to evolve the simulation
4. Switch visualization: Elevation, PlateId, ContinentalWeight, BoundaryMask
5. Toggle **Displace By Elevation** for 3D terrain relief

## References

- Cortial, Y., Peytavie, A., Galin, E., & Guerin, E. (2019). *Procedural Tectonic Planets.* Computer Graphics Forum, 38(7).
- Cortial, Y. (2020). *Synthese de terrain a l'echelle planetaire.* PhD thesis, INSA Lyon.
- Cortial, Y., Peytavie, A., Galin, E., & Guerin, E. (2020). *Real-Time Hyper-Amplification of Planets.* IEEE TVCG.

## License

Private repository. All rights reserved.
