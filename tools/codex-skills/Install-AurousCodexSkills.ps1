param(
	[string]$CodexSkillsRoot = (Join-Path $env:USERPROFILE '.codex\skills'),
	[switch]$Preview
)

$ErrorActionPreference = 'Stop'

$MirrorRoot = Join-Path $PSScriptRoot 'skills'
if (-not (Test-Path -LiteralPath $MirrorRoot -PathType Container)) {
	throw "Missing skill mirror: $MirrorRoot"
}

$SkillDirs = @(
	Get-ChildItem -LiteralPath $MirrorRoot -Directory |
		Sort-Object Name
)

$Results = foreach ($Skill in $SkillDirs) {
	$Target = Join-Path $CodexSkillsRoot $Skill.Name
	if ($Preview) {
		[pscustomobject]@{
			skill = $Skill.Name
			status = if (Test-Path -LiteralPath $Target -PathType Container) { 'would_update' } else { 'would_install' }
			source = $Skill.FullName
			target = $Target
		}
		continue
	}

	New-Item -ItemType Directory -Force -Path $Target | Out-Null
	Copy-Item -Path (Join-Path $Skill.FullName '*') -Destination $Target -Recurse -Force
	[pscustomobject]@{
		skill = $Skill.Name
		status = 'installed'
		source = $Skill.FullName
		target = $Target
	}
}

[pscustomobject]@{
	status = if ($Preview) { 'preview' } else { 'installed' }
	codex_skills_root = $CodexSkillsRoot
	skills = $Results
} | ConvertTo-Json -Depth 5
