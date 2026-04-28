---
name: unreal-build
description: Build and diagnose the Aurous Unreal Engine editor target on Windows. Use when Codex needs to compile AurousEditor, check Unreal build prerequisites, write or tail Saved/Logs/Build.log, detect running UnrealEditor/UnrealEditor-Cmd locks before building, or summarize Unreal Build.bat failures for this repo.
---

# Unreal Build

## Quick Workflow

Use the helper script for Aurous editor builds:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Run-AurousEditorBuild.ps1'
```

Use `-CheckOnly` to verify paths and editor-process preflight without launching a build:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Run-AurousEditorBuild.ps1' -CheckOnly
```

The default target is:

```text
AurousEditor Win64 Development
```

The default project and engine paths are:

```text
C:\Users\Michael\Documents\Unreal Projects\Aurous\Aurous.uproject
C:\Program Files\Epic Games\UE_5.7
```

The build log is written to:

```text
C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs\Build.log
```

## Behavior

The helper:

1. Checks `Build.bat` and `Aurous.uproject` exist.
2. Checks for `UnrealEditor` and `UnrealEditor-Cmd`.
3. Refuses to build while either Unreal process is running unless `-AllowRunningEditor` is passed.
4. Runs Unreal through `cmd.exe /d /s /c`, matching this repo's Windows command hygiene.
5. Writes `Saved\Logs\Build.log`.
6. Emits compact JSON with status, exit code, duration, log path, notable error lines, and log tail.

Do not kill editor processes automatically. If a locking process must be stopped, ask the user first and prefer:

```powershell
powershell.exe -NoProfile -Command 'Stop-Process -Id 123456 -Force'
```

## Manual Build Shape

If running manually outside the helper, use the repo-approved shape:

```powershell
cmd.exe /c ""C:\Program Files\Epic Games\UE_5.7\Engine\Build\BatchFiles\Build.bat" AurousEditor Win64 Development "C:\Users\Michael\Documents\Unreal Projects\Aurous\Aurous.uproject" > "C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs\Build.log" 2>&1"
```

Tail the log with:

```powershell
powershell.exe -NoProfile -Command 'Get-Content "C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs\Build.log" -Tail 120'
```
