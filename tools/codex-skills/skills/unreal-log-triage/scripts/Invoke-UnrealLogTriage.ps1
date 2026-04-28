param(
	[string]$Path = 'C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs\Build.log',
	[int]$Latest = 1,
	[int]$MaxMatches = 80,
	[int]$TailLines = 80
)

$ErrorActionPreference = 'Continue'

function Resolve-LogFiles {
	param([string]$InputPath, [int]$Count)
	if (Test-Path -LiteralPath $InputPath -PathType Leaf) {
		return @(Get-Item -LiteralPath $InputPath)
	}
	if (Test-Path -LiteralPath $InputPath -PathType Container) {
		return @(Get-ChildItem -LiteralPath $InputPath -Recurse -File -Include '*.log','*.txt' | Sort-Object LastWriteTime -Descending | Select-Object -First $Count)
	}
	return @(Get-ChildItem -Path $InputPath -File -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First $Count)
}

$Files = @(Resolve-LogFiles -InputPath $Path -Count $Latest)

$FatalPattern = 'Fatal error|Unhandled Exception|EXCEPTION_ACCESS_VIOLATION|Crash'
$ErrorPattern = '(^|[^\w])(Error|error C\d+|fatal error|MSB\d+: error|Log.*: Error:)'
$WarningPattern = '(^|[^\w])(Warning|Log.*: Warning:)'
$EnsurePattern = 'Ensure condition failed|LogOutputDevice: Error: Ensure'
$AutomationPattern = 'Automation.*(Fail|Error)|Test Failed|Tests Failed|LogAutomation.*Error'

$Results = foreach ($File in $Files) {
	$Lines = @(Get-Content -LiteralPath $File.FullName -ErrorAction SilentlyContinue | ForEach-Object { [string]$_ })
	$Fatal = @($Lines | Select-String -Pattern $FatalPattern -CaseSensitive:$false | Select-Object -Last $MaxMatches | ForEach-Object { [string]$_.Line })
	$Errors = @($Lines | Select-String -Pattern $ErrorPattern -CaseSensitive:$false | Select-Object -Last $MaxMatches | ForEach-Object { [string]$_.Line })
	$Warnings = @($Lines | Select-String -Pattern $WarningPattern -CaseSensitive:$false | Select-Object -Last $MaxMatches | ForEach-Object { [string]$_.Line })
	$Ensures = @($Lines | Select-String -Pattern $EnsurePattern -CaseSensitive:$false | Select-Object -Last $MaxMatches | ForEach-Object { [string]$_.Line })
	$Automation = @($Lines | Select-String -Pattern $AutomationPattern -CaseSensitive:$false | Select-Object -Last $MaxMatches | ForEach-Object { [string]$_.Line })
	$Status = if ($Fatal.Count -gt 0) { 'fatal' } elseif ($Errors.Count -gt 0 -or $Automation.Count -gt 0) { 'errors' } elseif ($Ensures.Count -gt 0) { 'ensures' } elseif ($Warnings.Count -gt 0) { 'warnings_only' } else { 'ok_no_notable_lines' }

	[pscustomobject]@{
		status = $Status
		path = $File.FullName
		last_write_time = $File.LastWriteTime
		line_count = $Lines.Count
		fatal_count = $Fatal.Count
		error_count = $Errors.Count
		warning_count = $Warnings.Count
		ensure_count = $Ensures.Count
		automation_failure_count = $Automation.Count
		fatal = $Fatal
		errors = $Errors
		ensures = $Ensures
		automation_failures = $Automation
		warnings_sample = @($Warnings | Select-Object -Last 20 | ForEach-Object { [string]$_ })
		tail = @($Lines | Select-Object -Last $TailLines | ForEach-Object { [string]$_ })
	}
}

[pscustomobject]@{
	status = if ($Results.Count -eq 0) { 'no_logs_found' } elseif (($Results | Where-Object { $_.status -in @('fatal','errors') }).Count -gt 0) { 'problems_found' } else { 'ok' }
	input_path = $Path
	logs = $Results
} | ConvertTo-Json -Depth 7
