---
name: aurous-state
description: Load current Aurous project truth quickly. Use when Codex is starting Aurous work, needs the current Prototype C/D architecture context, should read docs/STATE.md and ADR 0001/0002 before code changes, or needs a compact repo-state snapshot without re-discovering the project history.
---

# Aurous State

## Quick Workflow

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Show-AurousStateContext.ps1'
```

Then read the referenced docs as needed. Start with:

```text
docs/STATE.md
docs/architecture/decisions/0001-c-freeze-voronoi-ownership-decoupled-material.md
docs/architecture/decisions/0002-d-persistent-divergent-ocean-crust.md
```

Current high-level truth:

- Prototype C is the frozen architecture: plate-authoritative sidecar, nearest-center Voronoi ownership, raw adjacency boundaries, decoupled material projection.
- V6/V9 are historical and should not be extended for new tectonic work.
- New D work should preserve C invariants and follow ADR 0002.
- Key automation filters include `Aurous.TectonicPlanet.SidecarPrototypeC` and `Aurous.TectonicPlanet.SidecarPrototypeD`.
