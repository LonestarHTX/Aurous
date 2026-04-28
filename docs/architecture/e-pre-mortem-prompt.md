# Prototype E Adversarial Review And Pre-Mortem Prompt

Use this after ADR 0004 is Accepted and before any Prototype E Slice 1 code
lands.

## GPT-5.5 Pro Adversarial ADR Review

```text
Review the accepted Prototype E ADR before implementation.

Inspect the current branch against:

- docs/architecture/decisions/0001-c-freeze-voronoi-ownership-decoupled-material.md
- docs/architecture/decisions/0002-d-persistent-divergent-ocean-crust.md
- docs/architecture/decisions/0003-d-oceanic-crust-cooling-law.md
- docs/architecture/decisions/0004-e-subduction-crust-consumption.md

Do not edit files. Do not propose implementation code. Treat this as an
adversarial architecture review before E Slice 1.

Answer whether ADR 0004 is specific enough to implement Slice 1 without
recreating the V6/V9 failure modes. Use file/function/line specificity where
the current code affects the answer.

Focus on:

- subduction consumption becoming ownership repair or remesh recovery
- consumption state becoming a second owner/boundary system
- per-edge active intervals being underspecified or nondeterministic
- edge-local area accounting producing tautological tests
- projection misses, fallback ocean, or material/owner mismatch being consumed
  as if they were persistent D crust
- tombstoned crust records being dropped from authority hashing
- event logs disagreeing with canonical crust-store state
- E projection writing `Sample.PlateId`, `Sample.bIsBoundary`, or high-CW
  carried material
- trench-location diagnostics becoming bathymetry or uplift by accident
- slab pull or plate-force feedback leaking into E instead of a Prototype F ADR
- tests trusting E diagnostics instead of independently recomputing expected
  consumption from C/D authority
- D high-resolution 5b evidence being treated as a blocker for E Slice 1

Return:

1. Verdict: Accepted as-is, Accepted after doc fixes, or Not ready for E Slice 1.
2. Findings ordered by severity, with exact ADR sections and code references
   where possible.
3. Ambiguities that would let two implementers build incompatible Slice 1s.
4. Missing tests or non-independent test risks.
5. A short recommendation list limited to pre-code ADR/test-plan fixes.
```

## 30-Day Pre-Mortem

```text
Assume Prototype E Slice 1 ships, then Prototype E as a whole fails its gates 30
days from now.

Inspect the current branch against ADR 0001, ADR 0002, ADR 0003, and ADR 0004.
Produce an adversarial pre-mortem with file/function/line specificity where
possible.

Focus on:

- consumed-crust state mutating C ownership or C boundary semantics
- `FTectonicPlanet.Samples` becoming source of truth again
- active intervals becoming nondeterministic because edge ordering or interval
  clipping is not canonical
- consumption events consuming generic fallback ocean, projection misses, or
  C material/owner mismatch samples
- tombstones or consumed intervals missing from `ComputeSidecarAuthorityHash`
- event-log replay disagreeing with the crust store
- projection mutating sidecar authority
- trench diagnostics turning into terrain behavior before a bathymetry ADR
- slab pull or plate-force feedback being folded into E
- tests passing because E diagnostics agree with themselves instead of
  independently recomputing convergence, target crust, and consumed area
- Slice 1 accidentally introducing runtime detection, projection, exports, or
  consumption outside the test-only seeding API

Answer:

1. What symptom would appear first?
2. What root cause would produce it?
3. Which test or invariant should have caught it?
4. Which ADR clause is ambiguous or incomplete?
5. What small change should land before the next E slice?

Do not recommend adding collision, rifting, uplift, erosion, slab pull, terrain
polish, or final balance. This review is only about preserving C/D while adding
explicit sidecar-owned subduction consumption state.
```
