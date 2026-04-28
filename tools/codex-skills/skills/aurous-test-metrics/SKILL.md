---
name: aurous-test-metrics
description: Parse Aurous Unreal automation logs into stable metrics. Use when Codex needs JSON or Markdown summaries of C/D test runtime, pass/fail lines, diagnostic drift/boundary/projected sample metrics, ocean crust or event counts, and recent automation failure evidence.
---

# Aurous Test Metrics

## Quick Workflow

Parse recent logs as JSON:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Get-AurousTestMetrics.ps1'
```

Emit a Markdown table:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Get-AurousTestMetrics.ps1' -Markdown
```

Target a specific log:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Get-AurousTestMetrics.ps1' -Path 'C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs\SidecarPrototypeC.log'
```

Use this after build/test runs when the final answer needs compact evidence instead of raw log tails.
