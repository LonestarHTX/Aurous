---
name: aurous-overlay-contactsheet
description: Generate visual contact sheets for Aurous map exports. Use when Codex needs a fast visual sanity check across key C/D overlay PNGs, including BoundaryMask, ContinentalWeight, MaterialOwnerMismatch, OceanCrustAge, OceanCrustElevation, OceanCrustThickness, OceanCrustId, CrustEventOverlay, and DivergentBoundary.
---

# Aurous Overlay Contact Sheet

## Quick Workflow

Generate a contact sheet from an export directory:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\New-AurousOverlayContactSheet.ps1' -ExportDir '<export-dir>'
```

The helper writes a PNG under `Saved\Diagnostics` by default and emits JSON with included and missing layers.

Use the returned absolute `output_path` in Markdown when showing the image in the Codex app:

```markdown
![Aurous overlay contact sheet](C:/absolute/path/contact-sheet.png)
```

Prefer this before manually opening many overlay images.
