---
name: aurous-adr-check
description: Check Aurous changes against Prototype C/D ADR guardrails. Use before or during implementation to scan the current diff for risky C ownership or boundary edits, V6/V9 regressions, D persistent-state hash coverage reminders, missing test edits, and required C/D verification commands.
---

# Aurous ADR Check

## Quick Workflow

Run against the current working tree:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '<skill-dir>\scripts\Test-AurousAdrGuardrails.ps1'
```

This is a heuristic guardrail scan, not a substitute for review. Treat `problems` as blockers until inspected, and treat `warnings` as verification reminders.

Typical follow-up tests:

```text
Aurous.TectonicPlanet.SidecarPrototypeC
Aurous.TectonicPlanet.SidecarPrototypeD
```
