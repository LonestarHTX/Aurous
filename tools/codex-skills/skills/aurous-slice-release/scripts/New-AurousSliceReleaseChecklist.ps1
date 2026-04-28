param(
	[string]$RepoRoot = 'C:\Users\Michael\Documents\Unreal Projects\Aurous',
	[string]$SliceName = 'Aurous slice',
	[string[]]$TestFilters = @(
		'Aurous.TectonicPlanet.SidecarPrototypeC',
		'Aurous.TectonicPlanet.SidecarPrototypeD'
	)
)

$ErrorActionPreference = 'Continue'

Push-Location $RepoRoot
try {
	$Branch = (& git branch --show-current 2>$null) -as [string]
	$Head = (& git rev-parse HEAD 2>$null) -as [string]
	$Status = @(& git status --short 2>$null | ForEach-Object { [string]$_ })
} finally {
	Pop-Location
}

$Checklist = @(
	[pscustomobject]@{
		step = 1
		name = 'state-and-adr-read'
		command = 'Read docs/STATE.md and docs/architecture/decisions/0001-*.md plus 0002-*.md'
		evidence = 'State/ADR constraints understood before changing behavior'
	}
	[pscustomobject]@{
		step = 2
		name = 'adr-guardrails'
		command = "powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/aurous-adr-check/scripts/Test-AurousAdrGuardrails.ps1"
		evidence = 'No unreviewed C/D guardrail problems'
	}
	[pscustomobject]@{
		step = 3
		name = 'build'
		command = "powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/unreal-build/scripts/Run-AurousEditorBuild.ps1"
		evidence = 'AurousEditor build status ok'
	}
	[pscustomobject]@{
		step = 4
		name = 'automation-tests'
		command = ($TestFilters | ForEach-Object { "powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/unreal-tests/scripts/Run-AurousAutomationTests.ps1 -TestFilter '$_'" })
		evidence = 'Targeted C/D automation filters pass'
	}
	[pscustomobject]@{
		step = 5
		name = 'log-and-metrics'
		command = @(
			"powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/unreal-log-triage/scripts/Invoke-UnrealLogTriage.ps1",
			"powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/aurous-test-metrics/scripts/Get-AurousTestMetrics.ps1 -Markdown"
		)
		evidence = 'Concise failure and metrics evidence captured'
	}
	[pscustomobject]@{
		step = 6
		name = 'exports-and-visuals'
		command = @(
			"powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/aurous-map-exports/scripts/Get-AurousMapExportSummary.ps1 -Latest 5",
			"powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/aurous-diagnostic-diff/scripts/Compare-AurousDiagnostics.ps1 -Before '<baseline-export>' -After '<new-export>'",
			"powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/aurous-overlay-contactsheet/scripts/New-AurousOverlayContactSheet.ps1 -ExportDir '<new-export>'"
		)
		evidence = 'Export overlays present and visually sane when map output changed'
	}
	[pscustomobject]@{
		step = 7
		name = 'stage-intended-files'
		command = 'git status --short, then git add only the intended paths'
		evidence = 'No unrelated user/agent changes staged'
	}
	[pscustomobject]@{
		step = 8
		name = 'commit-push-verify'
		command = @(
			'git commit -m "<message>"',
			'git push -u origin <branch>',
			"powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/skills/git-push-verify/scripts/Test-GitPushVerified.ps1"
		)
		evidence = 'Remote SHA matches local HEAD after push'
	}
)

[pscustomobject]@{
	status = 'ok'
	slice_name = $SliceName
	repo_root = $RepoRoot
	branch = ($Branch -as [string]).Trim()
	head = ($Head -as [string]).Trim()
	working_tree_status = $Status
	test_filters = $TestFilters
	checklist = $Checklist
} | ConvertTo-Json -Depth 7
