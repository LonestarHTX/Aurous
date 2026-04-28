---
name: aurous-diagnostic-diff
description: Compare Aurous automation or map export artifacts. Use when Codex needs to diff two Saved/MapExports folders, check overlay presence, compare PNG hashes/dimensions, surface added/removed/changed layers, or summarize diagnostic metric lines between Prototype C/D runs.
---

# Aurous Diagnostic Diff

## Quick Workflow

Compare two export folders:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Compare-AurousDiagnostics.ps1' -Before '<before-dir>' -After '<after-dir>'
```

For Markdown output:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Compare-AurousDiagnostics.ps1' -Before '<before-dir>' -After '<after-dir>' -Markdown
```

The helper compares file hashes, PNG dimensions, overlay presence, added/removed/changed files, and diagnostic text lines containing runtime, boundary fraction, mass error, drift, ocean crust, or event metrics.

Useful D overlays to watch: `OceanCrustAge`, `OceanCrustThickness`, `OceanCrustId`, `CrustEventOverlay`, and `DivergentBoundary`.
