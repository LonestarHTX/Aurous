param(
	[string]$RepoRoot = 'C:\Users\Michael\Documents\Unreal Projects\Aurous',
	[switch]$IncludeUntracked = $true,
	[int]$MaxFindings = 80
)

$ErrorActionPreference = 'Continue'

function Get-AddedTextForPath {
	param([string]$Path, [bool]$IsUntracked)
	$Extension = [System.IO.Path]::GetExtension($Path)
	if ($Extension -notin @('.h', '.cpp', '.cs', '.md', '.txt', '.ps1', '.uproject', '.uplugin')) {
		return ''
	}
	if ($IsUntracked) {
		$Full = Join-Path (Get-Location).Path $Path
		if (Test-Path -LiteralPath $Full -PathType Leaf) {
			return (Get-Content -LiteralPath $Full -Raw -ErrorAction SilentlyContinue)
		}
		return ''
	}
	$Diff = @(
		& git diff --no-ext-diff --unified=0 -- $Path 2>$null
		& git diff --cached --no-ext-diff --unified=0 -- $Path 2>$null
	)
	return (($Diff | Where-Object { $_ -match '^\+' -and $_ -notmatch '^\+\+\+' } | ForEach-Object { $_.Substring(1) }) -join "`n")
}

function New-Finding {
	param([string]$Level, [string]$Rule, [string]$Message, [string]$Path = '')
	[pscustomobject]@{
		level = $Level
		rule = $Rule
		path = $Path
		message = $Message
	}
}

$Adr1 = Join-Path $RepoRoot 'docs\architecture\decisions\0001-c-freeze-voronoi-ownership-decoupled-material.md'
$Adr2 = Join-Path $RepoRoot 'docs\architecture\decisions\0002-d-persistent-divergent-ocean-crust.md'

Push-Location $RepoRoot
try {
	$Tracked = @(@(
		& git diff --name-only 2>$null
		& git diff --cached --name-only 2>$null
	) | Sort-Object -Unique)
	$Untracked = if ($IncludeUntracked) { @(& git ls-files --others --exclude-standard 2>$null) } else { @() }
	$AllFilesBuffer = @()
	$AllFilesBuffer += @($Tracked)
	$AllFilesBuffer += @($Untracked)
	$AllFiles = @($AllFilesBuffer | Sort-Object -Unique)

	$Findings = @()
	if (-not (Test-Path -LiteralPath $Adr1 -PathType Leaf)) {
		$Findings += New-Finding -Level 'problem' -Rule 'adr-0001-present' -Message 'ADR 0001 is missing.'
	}
	if (-not (Test-Path -LiteralPath $Adr2 -PathType Leaf)) {
		$Findings += New-Finding -Level 'problem' -Rule 'adr-0002-present' -Message 'ADR 0002 is missing.'
	}

	$TouchedSource = @($AllFiles | Where-Object { $_ -like 'Source/Aurous/*' -or $_ -like 'Source\Aurous\*' })
	$TouchedTests = @($AllFiles | Where-Object { $_ -like '*Tests*' })
	$TouchedDocs = @($AllFiles | Where-Object { $_ -like 'docs/*' -or $_ -like 'docs\*' })

	foreach ($File in $AllFiles) {
		$IsUntracked = $Untracked -contains $File
		$Text = Get-AddedTextForPath -Path $File -IsUntracked $IsUntracked
		if ([string]::IsNullOrWhiteSpace($Text)) { continue }

		if ($File -like 'Source/Aurous/*' -or $File -like 'Source\Aurous\*') {
			if ($Text -match '\b(V6|V9)\b') {
				$Findings += New-Finding -Level 'problem' -Rule 'no-v6-v9-source-regression' -Path $File -Message 'Added source text references V6/V9; these are historical and should not be extended.'
			}
			if ($Text -match '\b(PlateId|bIsBoundary|BoundaryMask|Nearest.*Plate|Voronoi|OwnerPlate)\b') {
				$Findings += New-Finding -Level 'warning' -Rule 'c-ownership-boundary-touched' -Path $File -Message 'C ownership/boundary terms changed; run the Prototype C freeze tests and inspect BoundaryMask/export diffs.'
			}
			if ($Text -match '\b(OceanCrust|CrustEvent|Persistent|AuthorityHash|Divergent)\b') {
				$Findings += New-Finding -Level 'warning' -Rule 'd-persistent-state-touched' -Path $File -Message 'D persistent/event terms changed; confirm authority hash coverage and D idempotence tests.'
			}
			if ($Text -match '\b(OceanCrust|CrustEvent|Persistent)\b' -and $Text -notmatch '\b(Hash|AuthorityHash|Compute.*Hash)\b') {
				$Findings += New-Finding -Level 'warning' -Rule 'hash-coverage-reminder' -Path $File -Message 'Persistent D terms changed without added hash text in the same diff hunk; verify the new state is authority-hash-covered.'
			}
		}
	}

	if ($TouchedSource.Count -gt 0 -and $TouchedTests.Count -eq 0) {
		$Findings += New-Finding -Level 'warning' -Rule 'tests-not-touched' -Message 'Source changed without test file changes; confirm existing C/D automation covers the behavior.'
	}

	if (($TouchedSource | Where-Object { $_ -match 'Sidecar|TectonicPlanet' }).Count -gt 0 -and ($TouchedDocs | Where-Object { $_ -match '0001|0002|STATE' }).Count -eq 0) {
		$Findings += New-Finding -Level 'warning' -Rule 'architecture-docs-reminder' -Message 'Tectonic source changed without STATE/ADR updates; confirm no architecture contract changed.'
	}

	$Findings = @($Findings | Select-Object -First $MaxFindings)
	$Status = if (($Findings | Where-Object { $_.level -eq 'problem' }).Count -gt 0) {
		'problems'
	} elseif ($Findings.Count -gt 0) {
		'warnings'
	} else {
		'ok'
	}

	[pscustomobject]@{
		status = $Status
		repo_root = $RepoRoot
		changed_files = $AllFiles
		findings = $Findings
		required_verification = @(
			'Build AurousEditor',
			'Run Aurous.TectonicPlanet.SidecarPrototypeC for C freeze coverage when ownership/boundary paths changed',
			'Run Aurous.TectonicPlanet.SidecarPrototypeD when D persistent/event state changed',
			'Inspect map exports or contact sheets when overlay/projection behavior changed'
		)
		adr_paths = @($Adr1, $Adr2)
	} | ConvertTo-Json -Depth 6
} finally {
	Pop-Location
}
