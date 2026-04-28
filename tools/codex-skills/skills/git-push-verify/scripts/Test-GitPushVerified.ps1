param(
	[string]$RepoRoot = 'C:\Users\Michael\Documents\Unreal Projects\Aurous',
	[string]$Remote = 'origin',
	[string]$Branch = '',
	[switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
Push-Location $RepoRoot
try {
	if ([string]::IsNullOrWhiteSpace($Branch)) {
		$Branch = (& git branch --show-current).Trim()
	}
	$LocalHead = (& git rev-parse HEAD).Trim()
	$Remotes = @(& git remote | ForEach-Object { [string]$_ })
	$RemoteUrl = (& git remote get-url $Remote 2>$null)
	$DirtyStatus = @(& git status --short | ForEach-Object { [string]$_ })

	if ($CheckOnly) {
		[pscustomobject]@{
			status = 'ok_check_only'
			repo_root = $RepoRoot
			branch = $Branch
			remote = $Remote
			remote_url = ($RemoteUrl -as [string]).Trim()
			local_head = $LocalHead
			remotes = $Remotes
			dirty_status = $DirtyStatus
		} | ConvertTo-Json -Depth 5
		exit 0
	}

	$RemoteLine = (& git ls-remote $Remote "refs/heads/$Branch" 2>&1) -as [string]
	$RemoteHead = if ($RemoteLine -match '^([0-9a-fA-F]{40})\s+') { $Matches[1] } else { $null }
	$Status = if ($RemoteHead -eq $LocalHead) { 'verified' } elseif ($null -eq $RemoteHead) { 'remote_branch_not_found' } else { 'mismatch' }
	$Exit = if ($Status -eq 'verified') { 0 } else { 1 }

	[pscustomobject]@{
		status = $Status
		repo_root = $RepoRoot
		branch = $Branch
		remote = $Remote
		remote_url = ($RemoteUrl -as [string]).Trim()
		local_head = $LocalHead
		remote_head = $RemoteHead
		raw_remote = $RemoteLine
		dirty_status = $DirtyStatus
	} | ConvertTo-Json -Depth 5
	exit $Exit
} finally {
	Pop-Location
}
