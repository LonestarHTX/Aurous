---
name: git-push-verify
description: Verify that a git push actually reached the remote. Use after pushing any Aurous branch or PR branch, when Codex needs to compare local HEAD to origin via git ls-remote, avoid trusting a reported push without remote SHA verification, or diagnose branch/remote mismatch.
---

# Git Push Verify

## Quick Workflow

After pushing, run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Test-GitPushVerified.ps1'
```

For a specific branch or remote:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Test-GitPushVerified.ps1' -Branch 'codex/example' -Remote origin
```

Use `-CheckOnly` to inspect local branch/remotes without contacting the remote.

The expected success state is `status: "verified"` with identical `local_head` and `remote_head`.
