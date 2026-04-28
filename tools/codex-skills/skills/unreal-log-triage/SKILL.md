---
name: unreal-log-triage
description: Parse and summarize Unreal Engine logs for Aurous. Use when Codex needs to inspect Saved/Logs build or automation logs, extract fatal/error/warning/ensure lines, find automation test failures, summarize recent logs, or report concise failure evidence without dumping entire Unreal log files.
---

# Unreal Log Triage

## Quick Workflow

Triaging the default build log:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Invoke-UnrealLogTriage.ps1'
```

Triaging latest logs:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Invoke-UnrealLogTriage.ps1' -Path 'C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs' -Latest 5
```

Triaging a specific automation log:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Invoke-UnrealLogTriage.ps1' -Path 'C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs\SidecarPrototypeC.log'
```

Report findings by category and include the log path. Avoid pasting huge log tails unless the user asks.
