param(
	[Parameter(Mandatory = $true)][string]$Before,
	[Parameter(Mandatory = $true)][string]$After,
	[string[]]$KeyLayers = @(
		'PlateId',
		'BoundaryMask',
		'ContinentalWeight',
		'MaterialOwnerMismatch',
		'MaterialOverlap',
		'DivergentBoundary',
		'OceanCrustAge',
		'OceanCrustThickness',
		'OceanCrustId',
		'CrustEventOverlay'
	),
	[int]$MaxChangedFiles = 80,
	[int]$MaxMetricLines = 80,
	[switch]$Markdown
)

$ErrorActionPreference = 'Stop'

function Get-RelativePath {
	param([string]$Root, [string]$Path)
	$RootFull = [System.IO.Path]::GetFullPath($Root)
	if (-not $RootFull.EndsWith([System.IO.Path]::DirectorySeparatorChar)) {
		$RootFull += [System.IO.Path]::DirectorySeparatorChar
	}
	$PathFull = [System.IO.Path]::GetFullPath($Path)
	return $PathFull.Substring($RootFull.Length)
}

function Get-PngSize {
	param([string]$Path)
	$Bytes = [byte[]]::new(24)
	$Stream = [System.IO.File]::OpenRead($Path)
	try {
		if ($Stream.Read($Bytes, 0, 24) -lt 24) { return $null }
		$Signature = [byte[]](137,80,78,71,13,10,26,10)
		for ($Index = 0; $Index -lt $Signature.Length; $Index++) {
			if ($Bytes[$Index] -ne $Signature[$Index]) { return $null }
		}
		$Width = ([int]$Bytes[16] -shl 24) -bor ([int]$Bytes[17] -shl 16) -bor ([int]$Bytes[18] -shl 8) -bor [int]$Bytes[19]
		$Height = ([int]$Bytes[20] -shl 24) -bor ([int]$Bytes[21] -shl 16) -bor ([int]$Bytes[22] -shl 8) -bor [int]$Bytes[23]
		return [pscustomobject]@{ width = $Width; height = $Height }
	} finally {
		$Stream.Dispose()
	}
}

function Get-ArtifactFiles {
	param([string]$Root)
	$Files = @(
		Get-ChildItem -LiteralPath $Root -Recurse -File -ErrorAction SilentlyContinue |
			Where-Object { $_.Extension -in @('.png', '.json', '.txt', '.log', '.csv', '.md') } |
			Sort-Object FullName
	)
	foreach ($File in $Files) {
		$Relative = Get-RelativePath -Root $Root -Path $File.FullName
		$Hash = (Get-FileHash -LiteralPath $File.FullName -Algorithm SHA256).Hash
		$PngSize = if ($File.Extension -ieq '.png') { Get-PngSize -Path $File.FullName } else { $null }
		[pscustomobject]@{
			relative_path = $Relative
			name = $File.Name
			layer = $File.BaseName
			extension = $File.Extension.ToLowerInvariant()
			length = $File.Length
			last_write_time = $File.LastWriteTime
			hash = $Hash
			width = if ($null -ne $PngSize) { $PngSize.width } else { $null }
			height = if ($null -ne $PngSize) { $PngSize.height } else { $null }
			path = $File.FullName
		}
	}
}

function Get-KeyLayerPresence {
	param([object[]]$Files, [string[]]$Layers)
	foreach ($Layer in $Layers) {
		$Matches = @($Files | Where-Object { $_.extension -eq '.png' -and ($_.layer -eq $Layer -or $_.layer -like "$Layer*") })
		[pscustomobject]@{
			layer = $Layer
			present = $Matches.Count -gt 0
			count = $Matches.Count
			paths = @($Matches | Select-Object -First 8 -ExpandProperty relative_path)
		}
	}
}

function Get-DiagnosticMetricLines {
	param([string]$Root, [int]$Limit)
	$Pattern = '(runtime|duration|boundary\s*fraction|BoundaryFraction|mass\s*error|MassError|drift|projected|sample count|sample_count|OceanCrust|CrustEvent|event count|AuthorityHash|Tests Passed|Tests Failed|Total Tests)'
	$TextFiles = @(
		Get-ChildItem -LiteralPath $Root -Recurse -File -ErrorAction SilentlyContinue |
			Where-Object { $_.Extension -in @('.txt', '.log', '.json', '.csv', '.md') } |
			Sort-Object LastWriteTime -Descending
	)
	$Rows = foreach ($File in $TextFiles) {
		$Lines = @(Get-Content -LiteralPath $File.FullName -ErrorAction SilentlyContinue | ForEach-Object { [string]$_ })
		$LineNumber = 0
		foreach ($Line in $Lines) {
			$LineNumber++
			if ($Line -match $Pattern) {
				[pscustomobject]@{
					file = Get-RelativePath -Root $Root -Path $File.FullName
					line = $LineNumber
					text = $Line.Trim()
				}
			}
		}
	}
	return @($Rows | Select-Object -First $Limit)
}

function Compare-FileSets {
	param([object[]]$BeforeFiles, [object[]]$AfterFiles, [int]$Limit)
	$BeforeMap = @{}
	$AfterMap = @{}
	foreach ($File in $BeforeFiles) { $BeforeMap[$File.relative_path] = $File }
	foreach ($File in $AfterFiles) { $AfterMap[$File.relative_path] = $File }
	$AllPaths = @($BeforeMap.Keys + $AfterMap.Keys | Sort-Object -Unique)
	$Rows = foreach ($Path in $AllPaths) {
		$BeforeFile = $BeforeMap[$Path]
		$AfterFile = $AfterMap[$Path]
		$Status = if ($null -eq $BeforeFile) {
			'added'
		} elseif ($null -eq $AfterFile) {
			'removed'
		} elseif ($BeforeFile.hash -ne $AfterFile.hash) {
			'changed'
		} else {
			'unchanged'
		}
		[pscustomobject]@{
			status = $Status
			relative_path = $Path
			before_hash = if ($null -ne $BeforeFile) { $BeforeFile.hash } else { $null }
			after_hash = if ($null -ne $AfterFile) { $AfterFile.hash } else { $null }
			before_length = if ($null -ne $BeforeFile) { $BeforeFile.length } else { $null }
			after_length = if ($null -ne $AfterFile) { $AfterFile.length } else { $null }
			before_size = if ($null -ne $BeforeFile -and $BeforeFile.width) { "$($BeforeFile.width)x$($BeforeFile.height)" } else { $null }
			after_size = if ($null -ne $AfterFile -and $AfterFile.width) { "$($AfterFile.width)x$($AfterFile.height)" } else { $null }
		}
	}
	return [pscustomobject]@{
		added_count = @($Rows | Where-Object { $_.status -eq 'added' }).Count
		removed_count = @($Rows | Where-Object { $_.status -eq 'removed' }).Count
		changed_count = @($Rows | Where-Object { $_.status -eq 'changed' }).Count
		unchanged_count = @($Rows | Where-Object { $_.status -eq 'unchanged' }).Count
		notable_files = @($Rows | Where-Object { $_.status -ne 'unchanged' } | Select-Object -First $Limit)
	}
}

if (-not (Test-Path -LiteralPath $Before -PathType Container)) { throw "Before path is not a directory: $Before" }
if (-not (Test-Path -LiteralPath $After -PathType Container)) { throw "After path is not a directory: $After" }

$BeforeRoot = (Resolve-Path -LiteralPath $Before).Path
$AfterRoot = (Resolve-Path -LiteralPath $After).Path
$BeforeFiles = @(Get-ArtifactFiles -Root $BeforeRoot)
$AfterFiles = @(Get-ArtifactFiles -Root $AfterRoot)
$Comparison = Compare-FileSets -BeforeFiles $BeforeFiles -AfterFiles $AfterFiles -Limit $MaxChangedFiles
$BeforeLayers = @(Get-KeyLayerPresence -Files $BeforeFiles -Layers $KeyLayers)
$AfterLayers = @(Get-KeyLayerPresence -Files $AfterFiles -Layers $KeyLayers)

$Result = [pscustomobject]@{
	status = if ($Comparison.added_count -eq 0 -and $Comparison.removed_count -eq 0 -and $Comparison.changed_count -eq 0) { 'no_file_changes' } else { 'differences_found' }
	before = [pscustomobject]@{
		path = $BeforeRoot
		file_count = $BeforeFiles.Count
		png_count = @($BeforeFiles | Where-Object { $_.extension -eq '.png' }).Count
		key_layers = $BeforeLayers
		metric_lines = @(Get-DiagnosticMetricLines -Root $BeforeRoot -Limit $MaxMetricLines)
	}
	after = [pscustomobject]@{
		path = $AfterRoot
		file_count = $AfterFiles.Count
		png_count = @($AfterFiles | Where-Object { $_.extension -eq '.png' }).Count
		key_layers = $AfterLayers
		metric_lines = @(Get-DiagnosticMetricLines -Root $AfterRoot -Limit $MaxMetricLines)
	}
	comparison = $Comparison
}

if ($Markdown) {
	"## Aurous Diagnostic Diff"
	""
	"| Field | Before | After |"
	"| --- | ---: | ---: |"
	"| Files | $($Result.before.file_count) | $($Result.after.file_count) |"
	"| PNGs | $($Result.before.png_count) | $($Result.after.png_count) |"
	""
	"| Status | Count |"
	"| --- | ---: |"
	"| Added | $($Comparison.added_count) |"
	"| Removed | $($Comparison.removed_count) |"
	"| Changed | $($Comparison.changed_count) |"
	"| Unchanged | $($Comparison.unchanged_count) |"
	""
	"| Key Layer | Before | After |"
	"| --- | --- | --- |"
	foreach ($Layer in $KeyLayers) {
		$BeforePresent = ($BeforeLayers | Where-Object { $_.layer -eq $Layer }).present
		$AfterPresent = ($AfterLayers | Where-Object { $_.layer -eq $Layer }).present
		"| `$Layer` | $BeforePresent | $AfterPresent |"
	}
	""
	if ($Comparison.notable_files.Count -gt 0) {
		"### Notable Files"
		foreach ($File in $Comparison.notable_files) {
			"- $($File.status): ``$($File.relative_path)``"
		}
	}
} else {
	$Result | ConvertTo-Json -Depth 8
}
