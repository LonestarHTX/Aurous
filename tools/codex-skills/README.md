# Aurous Codex Skills

This directory mirrors the Aurous-specific skills expected by `AGENTS.md`.

- `skills/` contains the skill folders that can be copied into `C:\Users\Michael\.codex\skills`.
- `Test-AurousCodexSkills.ps1` compares the mirror to the local Codex runtime install.
- `Install-AurousCodexSkills.ps1` copies the mirror into the local Codex runtime install.

Run checks from the repo root:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/Test-AurousCodexSkills.ps1
```

Preview installation:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/codex-skills/Install-AurousCodexSkills.ps1 -Preview
```
