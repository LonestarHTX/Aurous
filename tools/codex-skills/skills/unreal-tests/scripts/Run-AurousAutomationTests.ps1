param(
	[string]$ProjectPath = 'C:\Users\Michael\Documents\Unreal Projects\Aurous\Aurous.uproject',
	[string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.7',
	[string]$TestFilter = 'Aurous.TectonicPlanet.SidecarPrototypeC',
	[string]$LogPath,
	[int]$TailLines = 120,
	[switch]$CheckOnly,
	[switch]$AllowRunningEditor
)

$ErrorActionPreference = 'Stop'

function ConvertTo-SafeName {
	param([string]$Value)
	return ($Value -replace '[^A-Za-z0-9_.-]+', '-').Trim('-')
}

function Get-KnownAutomationTests {
	param([string]$RepoRoot)
	$TestRoot = Join-Path $RepoRoot 'Source\Aurous\Private\Tests'
	if (-not (Test-Path -LiteralPath $TestRoot)) { return @() }
	Get-ChildItem -LiteralPath $TestRoot -Recurse -File -Filter '*.cpp' |
		Select-String -Pattern '"(Aurous\.TectonicPlanet\.[^"]+)"' |
		ForEach-Object { $_.Matches | ForEach-Object { $_.Groups[1].Value } } |
		Sort-Object -Unique
}

$RepoRoot = Split-Path -Parent $ProjectPath
$EditorCmd = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
if ([string]::IsNullOrWhiteSpace($LogPath)) {
	$LogPath = Join-Path $RepoRoot ('Saved\Logs\' + (ConvertTo-SafeName $TestFilter) + '.log')
}
$StdoutPath = $LogPath + '.stdout.txt'

$Missing = @()
if (-not (Test-Path -LiteralPath $EditorCmd -PathType Leaf)) { $Missing += $EditorCmd }
if (-not (Test-Path -LiteralPath $ProjectPath -PathType Leaf)) { $Missing += $ProjectPath }
$RunningUnreal = @(Get-Process -Name UnrealEditor,UnrealEditor-Cmd -ErrorAction SilentlyContinue | Select-Object Id, ProcessName, Path)
$KnownTests = @(Get-KnownAutomationTests -RepoRoot $RepoRoot)
$MatchingTests = @($KnownTests | Where-Object { $_ -eq $TestFilter -or $_.StartsWith($TestFilter + '.') })

$ExecCmds = "Automation RunTests $TestFilter; Quit"
$Arguments = @(
	"`"$ProjectPath`"",
	'-unattended',
	'-nopause',
	'-nosplash',
	'-nullrhi',
	'-nosound',
	'-nop4',
	"-ExecCmds=`"$ExecCmds`"",
	'-TestExit="Automation Test Queue Empty"',
	"-log=`"$LogPath`""
) -join ' '

function Emit-Result {
	param([string]$Status, [int]$ExitCode = 0, [hashtable]$Extra = @{})
	$Base = [ordered]@{
		status = $Status
		exit_code = $ExitCode
		project_path = $ProjectPath
		editor_cmd = $EditorCmd
		test_filter = $TestFilter
		log_path = $LogPath
		stdout_path = $StdoutPath
		arguments = $Arguments
		running_unreal_processes = $RunningUnreal
		matching_known_tests = $MatchingTests
	}
	foreach ($Item in $Extra.GetEnumerator()) { $Base[$Item.Key] = $Item.Value }
	[pscustomobject]$Base | ConvertTo-Json -Depth 6
}

if ($Missing.Count -gt 0) {
	Emit-Result -Status 'missing_prerequisites' -ExitCode 2 -Extra @{ missing = $Missing }
	exit 2
}

if ($RunningUnreal.Count -gt 0 -and -not $AllowRunningEditor) {
	Emit-Result -Status 'blocked_running_unreal' -ExitCode 3 -Extra @{ message = 'UnrealEditor or UnrealEditor-Cmd is running.' }
	exit 3
}

if ($CheckOnly) {
	Emit-Result -Status 'ok_check_only' -Extra @{ known_test_count = $KnownTests.Count }
	exit 0
}

New-Item -ItemType Directory -Force -Path (Split-Path -Parent $LogPath) | Out-Null
$StartedAt = Get-Date
$ProcessInfo = [System.Diagnostics.ProcessStartInfo]::new()
$ProcessInfo.FileName = $EditorCmd
$ProcessInfo.Arguments = $Arguments
$ProcessInfo.WorkingDirectory = $RepoRoot
$ProcessInfo.UseShellExecute = $false
$ProcessInfo.RedirectStandardOutput = $true
$ProcessInfo.RedirectStandardError = $true
$Process = [System.Diagnostics.Process]::new()
$Process.StartInfo = $ProcessInfo
[void]$Process.Start()
$Stdout = $Process.StandardOutput.ReadToEnd()
$Stderr = $Process.StandardError.ReadToEnd()
$Process.WaitForExit()
$Duration = [Math]::Round(((Get-Date) - $StartedAt).TotalSeconds, 3)
[System.IO.File]::WriteAllText($StdoutPath, ($Stdout + "`n" + $Stderr), [System.Text.UTF8Encoding]::new($false))

$ParsePath = if (Test-Path -LiteralPath $LogPath) { $LogPath } else { $StdoutPath }
$Lines = @(Get-Content -LiteralPath $ParsePath -ErrorAction SilentlyContinue | ForEach-Object { [string]$_ })
$FailureRegex = '(Fatal error|LogAutomation.*Error|Automation.*Fail|Test Failed|Error:| failed\b|UnrealEditor-Cmd.*exiting with)'
$NotableFailures = @($Lines | Select-String -Pattern $FailureRegex -CaseSensitive:$false | Select-Object -Last 80 | ForEach-Object { [string]$_.Line })
$SummaryRegex = '(Automation Test Queue Empty|LogAutomationController|LogAutomationCommandLine|Tests Passed|Tests Failed|Total Tests)'
$SummaryLines = @($Lines | Select-String -Pattern $SummaryRegex -CaseSensitive:$false | Select-Object -Last 80 | ForEach-Object { [string]$_.Line })
$Tail = @($Lines | Select-Object -Last $TailLines | ForEach-Object { [string]$_ })
$Status = if ($Process.ExitCode -eq 0 -and $NotableFailures.Count -eq 0) { 'ok' } elseif ($Process.ExitCode -eq 0) { 'completed_with_notable_failures' } else { 'failed' }

Emit-Result -Status $Status -ExitCode $Process.ExitCode -Extra @{
	duration_seconds = $Duration
	parsed_log_path = $ParsePath
	summary_lines = $SummaryLines
	notable_failures = $NotableFailures
	tail = $Tail
}
exit $Process.ExitCode
