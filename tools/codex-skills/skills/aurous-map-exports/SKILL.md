---
name: aurous-map-exports
description: Inspect Aurous Saved/MapExports artifacts. Use when Codex needs to find latest generated map export runs, summarize PNG layers/steps, compare export directories, surface absolute image paths for rendering in the Codex app, or understand smoke-test visual artifacts without manually crawling thousands of files.
---

# Aurous Map Exports

## Quick Workflow

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Get-AurousMapExportSummary.ps1'
```

Limit or filter:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Get-AurousMapExportSummary.ps1' -Latest 5 -NameFilter 'SidecarPrototypeC'
```

Use absolute PNG paths in Markdown image tags when showing images in the Codex app:

```markdown
![BoundaryMask](C:/absolute/path/BoundaryMask.png)
```

Common layers include `PlateId`, `BoundaryMask`, `Elevation`, `CrustType`, `ContinentalWeight`, `OverlapMask`, `GapMask`, `SubductionDistance`, and `3D_*` views.
