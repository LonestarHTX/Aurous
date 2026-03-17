# Frontier Ownership Experiment

The codebase briefly carried an anchored boundary-frontier ownership prototype intended to replace global ownership repair with boundary-local ownership growth.

It was removed from the active baseline because the experiment failed the kill criteria:
- slower than the legacy ownership path
- frequent rollback instead of stable local ownership resolution
- verifier and cleanup still found detached fragments
- protected-plate preservation was not structural
- `Stress40` failed the protected-plate floor

Key lessons:
- Dirty seeding was too broad, so the frontier often expanded toward global work instead of staying local.
- Anchor validation was too restrictive, which led to anchor starvation on small or stressed protected plates.
- Protected-plate preservation by late revert was not structural; it reintroduced rescue-style reasoning after ownership assignment.
- Detached components still appeared, so the prototype still depended on verifier or cleanup logic to establish correctness.
- A future ownership redesign must make connectivity and protected-plate survival arise from the update rule itself, without any global proof or repair pass in the shipping path.
