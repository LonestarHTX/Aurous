# Aurous

Procedural tectonic planet generation in Unreal Engine 5.7, based on Cortial
et al., *Procedural Tectonic Planets* (2019) and the Cortial PhD thesis
*Synthese de terrain a l'echelle planetaire* (2020).

## Start Here

The current active architecture is Prototype C: a plate-authoritative sidecar
with clean Voronoi ownership and decoupled material projection.

Read these in order:

1. `docs/STATE.md` - current project state.
2. `docs/architecture/decisions/0001-c-freeze-voronoi-ownership-decoupled-material.md` - frozen C contract.
3. `docs/tectonic-minimal-acceptance-tests.md` - acceptance gates.
4. `docs/tectonic-architecture-failure-memo-2026-04.md` - why V6/V9 failed.
5. `docs/ProceduralTectonicPlanets.txt` - source paper notes.

## Current State

Prototype C is the first accepted foundation for the active tectonics path. It
does not claim a full tectonic lifecycle. It proves the ownership/material
carrier problem that blocked the project:

- clean, exclusive, exhaustive plate ownership from nearest rotated plate center
- raw adjacency boundary loops without post-process drawing or repair masks
- coherent carried material projected separately from ownership
- no V6/V9 remesh repair authority in the C sidecar path

Validated baseline: `0ff3ff0` (`Harden Prototype C freeze invariants`).

Current validation:

| Area | Status |
| --- | --- |
| C freeze tests | Passing |
| A/B diagnostic tests | Passing |
| 60k/40 C checkpoints | Exported at steps 0, 100, 200, 400 |
| 250k/40 C smoke | Exported at step 40 |
| Persistent ocean crust | Prototype D work |
| Subduction/collision/rifting/uplift | Prototype D+ work |

## Architecture

The active path is the plate-authoritative sidecar:

- `FTectonicPlanetSidecar` owns sidecar plate state and carried material.
- `FTectonicPlanet` receives projected output and remains a cache/carrier for
  visualization, diagnostics, tests, and exports.
- Prototype C ownership is `argmax(dot(sample position, rotated plate center))`
  with lower `PlateId` as deterministic tie-break.
- Prototype C boundaries are exactly one-ring owner adjacency.
- Material may disagree with owner. That mismatch is evidence, not repair input.

V6/V9 ownership-authority architecture is preserved as historical evidence and
should not be extended for new tectonic work. See the failure memo for why.

## Build

Requires Unreal Engine 5.7.

```bat
Engine\Build\BatchFiles\Build.bat AurousEditor Win64 Development -Project="<path>\Aurous.uproject"
```

Run tests with `UnrealEditor-Cmd.exe`:

```bat
UnrealEditor-Cmd.exe "<path>\Aurous.uproject" -unattended -nop4 -NullRHI -nosplash -nosound -ExecCmds="Automation RunTests Aurous.TectonicPlanet.SidecarPrototypeC; Quit"
```

Useful automation groups:

- `Aurous.TectonicPlanet.SidecarPrototypeA`
- `Aurous.TectonicPlanet.SidecarPrototypeB`
- `Aurous.TectonicPlanet.SidecarPrototypeC`
- `Aurous.TectonicPlanet`

## In-Editor Usage

For the current sidecar path:

1. Place a **Tectonic Planet Sidecar C** actor in the level.
2. Generate from the actor details or the Aurous tectonic control panel.
3. Use the control panel to step the sidecar simulation.
4. Inspect `PlateId`, `BoundaryMask`, `ContinentalWeight`, `MaterialSource`,
   `MaterialOwnerMismatch`, `MaterialOverlap`, and `DivergentBoundary` outputs.

The legacy V6 preview actor remains available for historical comparisons only.

## Evidence

Load-bearing C evidence is indexed in `docs/evidence/MANIFEST.md`. Generated
maps live under `Saved/MapExports/SidecarPrototypeC/` and are local artifacts,
not source-controlled truth by themselves.

## References

- Cortial, Y., Peytavie, A., Galin, E., & Guerin, E. (2019). *Procedural Tectonic Planets.* Computer Graphics Forum, 38(7).
- Cortial, Y. (2020). *Synthese de terrain a l'echelle planetaire.* PhD thesis, INSA Lyon.
- Cortial, Y., Peytavie, A., Galin, E., & Guerin, E. (2020). *Real-Time Hyper-Amplification of Planets.* IEEE TVCG.

## License

Private repository. All rights reserved.
