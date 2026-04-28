---
name: unreal-tests
description: Run and diagnose Aurous Unreal automation tests on Windows. Use when Codex needs to check known test filters, run UnrealEditor-Cmd with Automation RunTests, capture Saved/Logs automation output, default to the SidecarPrototypeC freeze test, or summarize Unreal automation failures for this repo.
---

# Unreal Tests

## Quick Workflow

Use `-CheckOnly` first:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Run-AurousAutomationTests.ps1' -CheckOnly
```

Run the current default C freeze test:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Run-AurousAutomationTests.ps1'
```

Run a different filter:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Run-AurousAutomationTests.ps1' -TestFilter 'Aurous.TectonicPlanet.SidecarPrototypeD'
```

Default filter:

```text
Aurous.TectonicPlanet.SidecarPrototypeC
```

The helper uses:

```text
C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor-Cmd.exe
C:\Users\Michael\Documents\Unreal Projects\Aurous\Aurous.uproject
```

It writes logs under `Saved\Logs` and emits compact JSON with status, exit code, matching known tests, notable failure lines, and tail output.

Do not run the full `Aurous.TectonicPlanet` suite by default; it is large. Prefer a targeted filter from the task.
