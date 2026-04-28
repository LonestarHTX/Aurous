---
name: aurous-slice-release
description: Finish an Aurous C/D implementation slice with a repeatable checklist. Use when Codex is preparing to close a D slice, needs the ordered build/test/export/metrics/review/commit/push/remote-verification ritual, or needs to stage intended files without losing unrelated work.
---

# Aurous Slice Release

## Quick Workflow

Generate the checklist:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\New-AurousSliceReleaseChecklist.ps1' -SliceName 'D Slice 3'
```

The helper does not build, test, stage, commit, push, or contact the network. It emits the commands and evidence slots to work through.

Core order:

1. Read `docs/STATE.md` and ADR 0001/0002.
2. Run ADR guardrail check.
3. Build `AurousEditor`.
4. Run targeted C and D automation filters.
5. Parse logs and metrics.
6. Inspect exports with diagnostic diff and contact sheet when maps changed.
7. Stage only intended files.
8. Commit, push, then verify remote SHA with `$git-push-verify`.
