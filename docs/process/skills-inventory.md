# Codex Skills Inventory

This repo keeps a mirror of the Aurous-specific Codex skills in `tools/codex-skills/skills`.
The runtime install location is still `C:\Users\Michael\.codex\skills`; the repo mirror is here
so a fresh agent install can inspect, restore, or compare the expected skill set.

## Restore Or Check

Check the repo mirror against the local Codex runtime install:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/Test-AurousCodexSkills.ps1
```

Restore the mirrored skills into the local Codex runtime install:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/Install-AurousCodexSkills.ps1
```

Preview the restore without writing:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/Install-AurousCodexSkills.ps1 -Preview
```

## Mirrored Aurous Skills

| Skill | Purpose | Side-effect-free check |
| --- | --- | --- |
| `$aurous-state` | Load `docs/STATE.md`, ADR 0001/0002 pointers, branch/head, and key test filters. | `Show-AurousStateContext.ps1` |
| `$aurous-map-exports` | Summarize `Saved/MapExports` runs and PNG layer outputs without crawling manually. | `Get-AurousMapExportSummary.ps1 -Latest 2` |
| `$aurous-diagnostic-diff` | Compare two export/log folders by file hashes, overlay presence, PNG dimensions, and parsed diagnostic metric lines. | `Compare-AurousDiagnostics.ps1 -Before <path> -After <path>` |
| `$aurous-overlay-contactsheet` | Generate a PNG contact sheet of key C/D overlay images, including `OceanCrustElevation.png`, from an export directory. | `New-AurousOverlayContactSheet.ps1 -ExportDir <path>` |
| `$aurous-adr-check` | Scan the current diff for ADR 0001/0002 guardrail risks and required verification reminders. | `Test-AurousAdrGuardrails.ps1` |
| `$aurous-test-metrics` | Parse Unreal automation logs into stable JSON or Markdown metrics summaries. | `Get-AurousTestMetrics.ps1` |
| `$aurous-slice-release` | Produce the D-slice finish checklist: build, C/D tests, exports, metrics, commit, push, verify. | `New-AurousSliceReleaseChecklist.ps1 -SliceName <name>` |
| `$git-push-verify` | Compare local `HEAD` to `git ls-remote` after a push. | `Test-GitPushVerified.ps1 -CheckOnly` |
| `$unreal-build` | Build or preflight `AurousEditor Win64 Development` through the repo-approved `Build.bat` shape. | `Run-AurousEditorBuild.ps1 -CheckOnly` |
| `$unreal-tests` | Run or preflight targeted Unreal automation tests, defaulting to the C freeze filter. | `Run-AurousAutomationTests.ps1 -CheckOnly` |
| `$unreal-log-triage` | Parse Unreal build or automation logs into fatal/error/warning/ensure buckets. | `Invoke-UnrealLogTriage.ps1` |

## Machine-Local Skills

These are intentionally not mirrored here because they describe Michael's Codex desktop
installation rather than the Aurous repo:

- `$codex-windows-toolchain`
- `$ripgrep-windows`

AGENTS.md may still reference them because this repo is usually worked on from that machine.
If they are missing, a future agent should recreate them from the current machine state rather
than treating the repo mirror as authoritative.
