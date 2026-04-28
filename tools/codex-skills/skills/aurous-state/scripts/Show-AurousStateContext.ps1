param(
	[string]$RepoRoot = 'C:\Users\Michael\Documents\Unreal Projects\Aurous',
	[int]$StateLines = 120
)

$ErrorActionPreference = 'Continue'
$StatePath = Join-Path $RepoRoot 'docs\STATE.md'
$Adr1 = Join-Path $RepoRoot 'docs\architecture\decisions\0001-c-freeze-voronoi-ownership-decoupled-material.md'
$Adr2 = Join-Path $RepoRoot 'docs\architecture\decisions\0002-d-persistent-divergent-ocean-crust.md'

Push-Location $RepoRoot
try {
	$Branch = (& git branch --show-current 2>$null) -as [string]
	$Head = (& git rev-parse --short HEAD 2>$null) -as [string]
	$Status = @(& git status --short 2>$null | ForEach-Object { [string]$_ })
} finally {
	Pop-Location
}

$StateHead = if (Test-Path -LiteralPath $StatePath) {
	@(Get-Content -LiteralPath $StatePath -TotalCount $StateLines | ForEach-Object { [string]$_ })
} else {
	@()
}

[pscustomobject]@{
	repo_root = $RepoRoot
	branch = $Branch.Trim()
	head = $Head.Trim()
	git_status = $Status
	state_path = $StatePath
	state_head = $StateHead
	adr_paths = @(
		[pscustomobject]@{ name = 'ADR 0001'; path = $Adr1; exists = Test-Path -LiteralPath $Adr1 },
		[pscustomobject]@{ name = 'ADR 0002'; path = $Adr2; exists = Test-Path -LiteralPath $Adr2 }
	)
	key_tests = @(
		'Aurous.TectonicPlanet.SidecarPrototypeA',
		'Aurous.TectonicPlanet.SidecarPrototypeB',
		'Aurous.TectonicPlanet.SidecarPrototypeC',
		'Aurous.TectonicPlanet.SidecarPrototypeD'
	)
} | ConvertTo-Json -Depth 5
