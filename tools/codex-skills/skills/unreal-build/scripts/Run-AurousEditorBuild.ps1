param(
	[string]$ProjectPath = 'C:\Users\Michael\Documents\Unreal Projects\Aurous\Aurous.uproject',
	[string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.7',
	[string]$Target = 'AurousEditor',
	[string]$Platform = 'Win64',
	[string]$Configuration = 'Development',
	[string]$LogPath,
	[int]$TailLines = 120,
	[switch]$CheckOnly,
	[switch]$AllowRunningEditor
)

$ErrorActionPreference = 'Stop'

function New-Result {
	param(
		[string]$Status,
		[int]$ExitCode = 0,
		[object]$Extra = @{}
	)

	$Base = [ordered]@{
		status = $Status
		exit_code = $ExitCode
		project_path = $ProjectPath
		engine_root = $EngineRoot
		target = $Target
		platform = $Platform
		configuration = $Configuration
		log_path = $LogPath
		build_bat = $BuildBat
		build_command = $BuildCommand
		running_unreal_processes = $RunningUnreal
	}

	foreach ($Item in $Extra.GetEnumerator()) {
		$Base[$Item.Key] = $Item.Value
	}

	[pscustomobject]$Base
}

if ([string]::IsNullOrWhiteSpace($LogPath)) {
	$LogPath = Join-Path (Split-Path -Parent $ProjectPath) 'Saved\Logs\Build.log'
}

$BuildBat = Join-Path $EngineRoot 'Engine\Build\BatchFiles\Build.bat'
$RepoRoot = Split-Path -Parent $ProjectPath
$BuildCommand = 'cmd.exe /d /s /c ""' + $BuildBat + '" ' + $Target + ' ' + $Platform + ' ' + $Configuration + ' "' + $ProjectPath + '""'

$Missing = @()
if (-not (Test-Path -LiteralPath $BuildBat -PathType Leaf)) { $Missing += $BuildBat }
if (-not (Test-Path -LiteralPath $ProjectPath -PathType Leaf)) { $Missing += $ProjectPath }

$RunningUnreal = @(
	Get-Process -Name UnrealEditor,UnrealEditor-Cmd -ErrorAction SilentlyContinue |
		Select-Object Id, ProcessName, Path
)

if ($Missing.Count -gt 0) {
	New-Result -Status 'missing_prerequisites' -ExitCode 2 -Extra ([ordered]@{ missing = $Missing }) |
		ConvertTo-Json -Depth 6
	exit 2
}

if ($RunningUnreal.Count -gt 0 -and -not $AllowRunningEditor) {
	New-Result -Status 'blocked_running_unreal' -ExitCode 3 -Extra ([ordered]@{
		message = 'UnrealEditor or UnrealEditor-Cmd is running. Close it or rerun with -AllowRunningEditor if this is intentional.'
	}) | ConvertTo-Json -Depth 6
	exit 3
}

if ($CheckOnly) {
	New-Result -Status 'ok_check_only' -Extra ([ordered]@{ repo_root = $RepoRoot }) |
		ConvertTo-Json -Depth 6
	exit 0
}

New-Item -ItemType Directory -Force -Path (Split-Path -Parent $LogPath) | Out-Null

$CmdPayload = '"' + $BuildBat + '" ' + $Target + ' ' + $Platform + ' ' + $Configuration + ' "' + $ProjectPath + '" 2>&1'
$Arguments = '/d /s /c "' + $CmdPayload + '"'
$StartedAt = Get-Date

$ProcessInfo = [System.Diagnostics.ProcessStartInfo]::new()
$ProcessInfo.FileName = 'cmd.exe'
$ProcessInfo.Arguments = $Arguments
$ProcessInfo.WorkingDirectory = $RepoRoot
$ProcessInfo.UseShellExecute = $false
$ProcessInfo.RedirectStandardOutput = $true
$ProcessInfo.RedirectStandardError = $false

$Process = [System.Diagnostics.Process]::new()
$Process.StartInfo = $ProcessInfo
[void]$Process.Start()
$Output = $Process.StandardOutput.ReadToEnd()
$Process.WaitForExit()

$Duration = [Math]::Round(((Get-Date) - $StartedAt).TotalSeconds, 3)
[System.IO.File]::WriteAllText($LogPath, $Output, [System.Text.UTF8Encoding]::new($false))

$LogLines = if (Test-Path -LiteralPath $LogPath) {
	@(Get-Content -LiteralPath $LogPath | ForEach-Object { [string]$_ })
} else {
	@()
}
$ErrorPatterns = @(
	' error C[0-9]+:',
	' fatal error ',
	'\berror:',
	'UnrealBuildTool failed',
	'BUILD FAILED',
	'AutomationTool exiting with ExitCode=[1-9]',
	'MSB[0-9]+: error'
)
$ErrorRegex = '(' + ($ErrorPatterns -join '|') + ')'
$NotableErrors = @($LogLines | Select-String -Pattern $ErrorRegex -CaseSensitive:$false | Select-Object -Last 60 | ForEach-Object { [string]$_.Line })
$Tail = @($LogLines | Select-Object -Last $TailLines | ForEach-Object { [string]$_ })
$Status = if ($Process.ExitCode -eq 0) { 'ok' } else { 'failed' }

New-Result -Status $Status -ExitCode $Process.ExitCode -Extra ([ordered]@{
	duration_seconds = $Duration
	notable_errors = $NotableErrors
	tail = $Tail
}) | ConvertTo-Json -Depth 6

exit $Process.ExitCode
