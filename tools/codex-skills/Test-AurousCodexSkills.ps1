param(
	[string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path,
	[string]$CodexSkillsRoot = (Join-Path $env:USERPROFILE '.codex\skills')
)

$ErrorActionPreference = 'Stop'

$MirrorRoot = Join-Path $PSScriptRoot 'skills'
$AgentsPath = Join-Path $RepoRoot 'AGENTS.md'

function Get-RelativePath {
	param([string]$Root, [string]$Path)
	$RootFull = [System.IO.Path]::GetFullPath($Root).TrimEnd('\') + '\'
	$PathFull = [System.IO.Path]::GetFullPath($Path)
	return $PathFull.Substring($RootFull.Length)
}

function Get-SkillFrontmatterStatus {
	param([string]$SkillName, [string]$SkillPath)
	$SkillMd = Join-Path $SkillPath 'SKILL.md'
	if (-not (Test-Path -LiteralPath $SkillMd -PathType Leaf)) {
		return 'missing_skill_md'
	}
	$Text = Get-Content -LiteralPath $SkillMd -Raw
	if ($Text -notmatch "(?m)^name:\s*$([regex]::Escape($SkillName))\s*$") {
		return 'missing_or_wrong_name'
	}
	if ($Text -notmatch '(?m)^description:\s*\S') {
		return 'missing_description'
	}
	return 'ok'
}

$MirrorSkills = @(
	Get-ChildItem -LiteralPath $MirrorRoot -Directory |
		Sort-Object Name
)

$SkillResults = foreach ($Skill in $MirrorSkills) {
	$InstalledPath = Join-Path $CodexSkillsRoot $Skill.Name
	$MirrorFiles = @(
		Get-ChildItem -LiteralPath $Skill.FullName -Recurse -File |
			Sort-Object FullName
	)
	$ComparedFiles = foreach ($MirrorFile in $MirrorFiles) {
		$Relative = Get-RelativePath -Root $Skill.FullName -Path $MirrorFile.FullName
		$InstalledFile = Join-Path $InstalledPath $Relative
		$MirrorHash = (Get-FileHash -LiteralPath $MirrorFile.FullName -Algorithm SHA256).Hash
		$InstalledHash = if (Test-Path -LiteralPath $InstalledFile -PathType Leaf) {
			(Get-FileHash -LiteralPath $InstalledFile -Algorithm SHA256).Hash
		} else {
			$null
		}
		[pscustomobject]@{
			path = $Relative
			status = if ($null -eq $InstalledHash) { 'missing' } elseif ($MirrorHash -eq $InstalledHash) { 'ok' } else { 'stale' }
			mirror_hash = $MirrorHash
			installed_hash = $InstalledHash
		}
	}
	$Frontmatter = Get-SkillFrontmatterStatus -SkillName $Skill.Name -SkillPath $Skill.FullName
	$FileStatuses = @($ComparedFiles | Select-Object -ExpandProperty status -Unique)
	$Status = if (-not (Test-Path -LiteralPath $InstalledPath -PathType Container)) {
		'missing_install'
	} elseif ($Frontmatter -ne 'ok') {
		$Frontmatter
	} elseif ($FileStatuses -contains 'missing' -or $FileStatuses -contains 'stale') {
		'drift'
	} else {
		'ok'
	}
	[pscustomobject]@{
		skill = $Skill.Name
		status = $Status
		mirror_path = $Skill.FullName
		installed_path = $InstalledPath
		files = $ComparedFiles
	}
}

$ExpectedSkillNames = @($MirrorSkills.Name + @('codex-windows-toolchain', 'ripgrep-windows')) | Sort-Object -Unique
$ReferencedSkills = @()
if (Test-Path -LiteralPath $AgentsPath -PathType Leaf) {
	$AgentsText = Get-Content -LiteralPath $AgentsPath -Raw
	$ReferencedSkills = @(
		[regex]::Matches($AgentsText, '\$([A-Za-z][A-Za-z0-9_-]+)') |
			ForEach-Object { $_.Groups[1].Value } |
			Where-Object { $ExpectedSkillNames -contains $_ } |
			Sort-Object -Unique
	)
}

$ReferencedResults = foreach ($Name in $ReferencedSkills) {
	[pscustomobject]@{
		skill = $Name
		in_repo_mirror = Test-Path -LiteralPath (Join-Path $MirrorRoot $Name) -PathType Container
		installed = Test-Path -LiteralPath (Join-Path $CodexSkillsRoot $Name) -PathType Container
	}
}

$Overall = if (($SkillResults | Where-Object { $_.status -ne 'ok' }).Count -gt 0) {
	'drift_or_missing'
} elseif (($ReferencedResults | Where-Object { -not $_.installed }).Count -gt 0) {
	'referenced_skill_missing'
} else {
	'ok'
}

[pscustomobject]@{
	status = $Overall
	repo_root = $RepoRoot
	mirror_root = $MirrorRoot
	codex_skills_root = $CodexSkillsRoot
	skills = $SkillResults
	referenced_skills = $ReferencedResults
} | ConvertTo-Json -Depth 8
