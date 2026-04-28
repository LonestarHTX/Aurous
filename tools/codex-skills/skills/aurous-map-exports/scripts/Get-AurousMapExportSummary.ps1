param(
	[string]$Root = 'C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\MapExports',
	[int]$Latest = 10,
	[string]$NameFilter = '',
	[int]$MaxStepDirsPerRun = 8
)

$ErrorActionPreference = 'Continue'

if (-not (Test-Path -LiteralPath $Root)) {
	[pscustomobject]@{ status = 'missing_root'; root = $Root } | ConvertTo-Json -Depth 4
	exit 1
}

$Runs = Get-ChildItem -LiteralPath $Root -Directory |
	Where-Object { [string]::IsNullOrWhiteSpace($NameFilter) -or $_.Name -like "*$NameFilter*" } |
	Sort-Object LastWriteTime -Descending |
	Select-Object -First $Latest

$Summaries = foreach ($Run in $Runs) {
	$Pngs = @(Get-ChildItem -LiteralPath $Run.FullName -Recurse -File -Filter '*.png' -ErrorAction SilentlyContinue)
	$StepGroups = @(
		$Pngs |
			Group-Object DirectoryName |
			ForEach-Object {
				$Dir = Get-Item -LiteralPath $_.Name
				[pscustomobject]@{
					path = $Dir.FullName
					name = $Dir.Name
					last_write_time = $Dir.LastWriteTime
					png_count = $_.Count
					layers = @($_.Group | ForEach-Object { $_.BaseName } | Sort-Object -Unique)
				}
			} |
			Sort-Object last_write_time -Descending |
			Select-Object -First $MaxStepDirsPerRun
	)
	[pscustomobject]@{
		name = $Run.Name
		path = $Run.FullName
		last_write_time = $Run.LastWriteTime
		png_count = $Pngs.Count
		latest_step_dirs = $StepGroups
	}
}

[pscustomobject]@{
	status = 'ok'
	root = $Root
	latest = $Latest
	name_filter = $NameFilter
	runs = @($Summaries)
} | ConvertTo-Json -Depth 8
