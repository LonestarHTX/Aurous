# Prototype D Pre-Mortem Prompt

Use this before D Slice 2, and preferably before any D implementation beyond
the Slice 1 state/hash scaffold.

```text
Assume Prototype D Slice 1 ships, then Prototype D as a whole fails its gates 30
days from now.

Inspect the current branch against ADR 0001 and ADR 0002. Produce an
adversarial pre-mortem with file/function/line specificity where possible.

Focus on:

- persistent crust state becoming a second ownership system
- event logs disagreeing with the canonical crust store
- projection mutating sidecar authority
- `FTectonicPlanet.Samples` becoming source of truth again
- C owner/boundary invariants weakening silently
- ocean crust being created from fallback, projection misses, or material-owner
  mismatch instead of named events
- event coalescing producing nondeterminism
- authority hashing missing fields that should be persistent
- tests trusting D diagnostics instead of independently recomputing expected
  state from the ADR
- Slice 1 accidentally introducing event detection, projection, exports, or
  ocean creation

Answer:

1. What symptom would appear first?
2. What root cause would produce it?
3. Which test or invariant should have caught it?
4. Which ADR clause is ambiguous or incomplete?
5. What small change should land before the next D slice?

Do not recommend adding subduction, collision, rifting, uplift, erosion, or
visual polish. This review is only about preserving C while adding persistent
divergent ocean crust state.
```
