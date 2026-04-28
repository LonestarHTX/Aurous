param(
	[string]$Path = 'C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs',
	[int]$Latest = 5,
	[string]$NameFilter = '',
	[int]$MaxDiagnosticLines = 60,
	[switch]$Markdown
)

$ErrorActionPreference = 'Continue'

function Resolve-LogFiles {
	param([string]$InputPath, [int]$Count, [string]$Filter)
	if (Test-Path -LiteralPath $InputPath -PathType Leaf) {
		return @(Get-Item -LiteralPath $InputPath)
	}
	if (Test-Path -LiteralPath $InputPath -PathType Container) {
		return @(
			Get-ChildItem -LiteralPath $InputPath -Recurse -File -Include '*.log','*.txt' |
				Where-Object { [string]::IsNullOrWhiteSpace($Filter) -or $_.Name -like "*$Filter*" } |
				Sort-Object LastWriteTime -Descending |
				Select-Object -First $Count
		)
	}
	return @(
		Get-ChildItem -Path $InputPath -File -ErrorAction SilentlyContinue |
			Where-Object { [string]::IsNullOrWhiteSpace($Filter) -or $_.Name -like "*$Filter*" } |
			Sort-Object LastWriteTime -Descending |
			Select-Object -First $Count
	)
}

function Select-LastMatch {
	param([string[]]$Lines, [string]$Pattern)
	$Match = $Lines | Select-String -Pattern $Pattern -CaseSensitive:$false | Select-Object -Last 1
	if ($null -eq $Match) { return $null }
	return [string]$Match.Line
}

function Get-FirstNumber {
	param([string]$Text)
	if ($Text -match '(-?\d+(?:\.\d+)?)') { return [double]$Matches[1] }
	return $null
}

$Files = @(Resolve-LogFiles -InputPath $Path -Count $Latest -Filter $NameFilter)
$DiagnosticPattern = '(boundary\s*fraction|BoundaryFraction|drift|projected|sample count|sample_count|OceanCrust|CrustEvent|event count|mass\s*error|MassError|AuthorityHash|Tests Passed|Tests Failed|Total Tests|Automation.*(Fail|Error))'

$Logs = foreach ($File in $Files) {
	$Lines = @(Get-Content -LiteralPath $File.FullName -ErrorAction SilentlyContinue | ForEach-Object { [string]$_ })
	$Joined = $Lines -join "`n"
	$PassedLine = Select-LastMatch -Lines $Lines -Pattern 'Tests Passed|Passed:\s*\d+|Automation.*Passed'
	$FailedLine = Select-LastMatch -Lines $Lines -Pattern 'Tests Failed|Failed:\s*\d+|Automation.*Fail|Test Failed'
	$TotalLine = Select-LastMatch -Lines $Lines -Pattern 'Total Tests|Tests Total|Total:\s*\d+'
	$RuntimeLine = Select-LastMatch -Lines $Lines -Pattern 'runtime|duration|Total execution time|completed in'
	$FatalCount = @($Lines | Select-String -Pattern 'Fatal error|Unhandled Exception|EXCEPTION_ACCESS_VIOLATION' -CaseSensitive:$false).Count
	$ErrorCount = @($Lines | Select-String -Pattern 'Log.*: Error:|Automation.*Error|Test Failed|Tests Failed| error C\d+:' -CaseSensitive:$false).Count
	$WarningCount = @($Lines | Select-String -Pattern 'Log.*: Warning:|Warning:' -CaseSensitive:$false).Count
	$Diagnostics = @(
		$Lines |
			Select-String -Pattern $DiagnosticPattern -CaseSensitive:$false |
			Select-Object -Last $MaxDiagnosticLines |
			ForEach-Object { [string]$_.Line.Trim() }
	)
	$Status = if ($FatalCount -gt 0 -or $ErrorCount -gt 0 -or ($FailedLine -and $FailedLine -notmatch '0')) {
		'problems'
	} elseif ($PassedLine -or $Diagnostics.Count -gt 0) {
		'has_metrics'
	} else {
		'no_metrics_found'
	}
	[pscustomobject]@{
		status = $Status
		path = $File.FullName
		name = $File.Name
		last_write_time = $File.LastWriteTime
		line_count = $Lines.Count
		fatal_count = $FatalCount
		error_count = $ErrorCount
		warning_count = $WarningCount
		passed_line = $PassedLine
		failed_line = $FailedLine
		total_line = $TotalLine
		runtime_line = $RuntimeLine
		runtime_number = if ($RuntimeLine) { Get-FirstNumber -Text $RuntimeLine } else { $null }
		diagnostic_lines = $Diagnostics
	}
}

$Result = [pscustomobject]@{
	status = if (($Logs | Where-Object { $_.status -eq 'problems' }).Count -gt 0) { 'problems_found' } elseif ($Logs.Count -eq 0) { 'no_logs_found' } else { 'ok' }
	input_path = $Path
	name_filter = $NameFilter
	logs = $Logs
}

if ($Markdown) {
	"| Log | Status | Errors | Warnings | Runtime |"
	"| --- | --- | ---: | ---: | --- |"
	foreach ($Log in $Logs) {
		$Runtime = if ($Log.runtime_line) { $Log.runtime_line.Replace('|', '\|') } else { '' }
		"| ``$($Log.name)`` | $($Log.status) | $($Log.error_count) | $($Log.warning_count) | $Runtime |"
	}
} else {
	$Result | ConvertTo-Json -Depth 7
}
