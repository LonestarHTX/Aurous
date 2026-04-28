# Prototype D Slice 5 Long-Horizon Checklist

Status: required review evidence before Prototype D cooling-law planning.

Scope: placeholder crust physics only. No cooling law, subduction, collision,
rifting, uplift, erosion, balance, or actor UI changes.

## Filters

- `Aurous.TectonicPlanet.SidecarPrototypeDLongHorizon`
  - 60k samples, 40 plates
  - checkpoints 0, 100, 200, 400, 800, 1000
  - export root `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/`
- `Aurous.TectonicPlanet.SidecarPrototypeDHighResLongHorizon`
  - 100k/40 checkpoints 0, 250, 500, 1000
  - 250k/40 checkpoints 0, 250, 500, 1000
  - 500k/40 checkpoints 0, 500, 1000
  - intended for overnight or separate-session runs

The high-res filter intentionally does not use the suffix
`SidecarPrototypeDLongHorizonHighRes`: Unreal automation treats the 60k filter
name as a prefix group, so that spelling makes the 60k run launch high-res work
too.

## Numeric Gates

- C ownership invariants remain exact at every checkpoint.
- `ProjectToPlanet` does not mutate sidecar authority, crust count, or event
  count.
- D projection does not write `PlateId` or `bIsBoundary`.
- D projection does not overwrite high carried continental material.
- Step 0 has zero crust/events; nonzero checkpoints project D crust.
- Crust ids persist from checkpoint N to checkpoint N+1.
- Crust records, events, birth edges, and persistent area are monotonic.
- `CrustEventLog.Num() <= CurrentStep * 4 * PlateCount`.
- `OceanCrustStore.Num() <= CrustEventLog.Num()`.
- Projected D age, thickness, and elevation are finite and match placeholder
  rules: age from step math, thickness `[6.9, 7.1]`, elevation `[-1.1, -0.9]`.
- Tiny D crust component sample fraction stays `<= 0.10`.
- D crust overlap with high-CW material is `0`.
- D crust overlap with material-owner mismatch is logged and capped at `<= 0.10`.

## Stress Signals

- Slice 5a fails if total 60k runtime exceeds `180s`.
- High-res runtime is logged and warns above `45min`; it is not failed solely for
  runtime unless the process times out.
- Working-set and virtual-memory snapshots are logged at every checkpoint.
- Memory growth warnings indicate memory is growing faster than birth-edge
  growth and should be investigated before the next tectonic process.
- Mismatch overlap is logged at every checkpoint. Any increasing-at-every-step
  trend is a warning even when the cap still passes.

## Visual Review

Generate contact sheets for all Slice 5a checkpoints and final high-res
checkpoints. Answer yes/no:

- persistent crust remains aligned with current or historical divergent edges
- crust corridors remain coherent rather than speckled
- crust ids persist and do not flicker between checkpoints
- event overlay remains geographically consistent with divergent boundaries
- D crust avoids high material-owner-mismatch zones
- crust/event growth looks bounded, not explosive

## Expected Artifact Roots

- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_000/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_100/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_200/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_400/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_800/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_60k40/step_1000/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_100k40/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_250k40/`
- `Saved/MapExports/SidecarPrototypeD/long_horizon_500k40/`
