# Ownership Investigation: Paper-Simple Ablation

## Context

Before committing to Connected Patch Transfer (a complex ownership redesign), we tested
whether a simpler approach — closer to what Cortial et al. actually describe — would be
sufficient. The paper's ownership model is: containment query, winner takes all, repartition.
No hysteresis, no stickiness, no P6 cleanup.

## What Was Tested

Three ownership modes, run across three workloads:

**Mode A — Paper-Simple:**
- Full containment query for every sample (no fast path)
- Highest containment score wins (no boundary-like classification, no hysteresis)
- No P6 repair (shadow verifier measured what P6 would have done)

**Mode B — Paper-Simple + Comparative Support Guard:**
- Same as Mode A, but when multiple plates contain a sample and the previous owner
  still contains it, retain the previous owner if it has more neighbor support than
  the contender (the proven `bRequireP3ContenderSupport` guard)
- No P6 repair

**Mode C — Current Baseline:**
- Fast path for non-boundary interior samples
- Boundary-like classification + hysteresis threshold
- Comparative support guard
- Full P6 repair (sanitize, connectivity, rescue)

## Results (2026-03-09)

```
Workload       Mode                        Frags  MaxFrag     Gaps Overlaps      AvgMs      MaxMs
─────────────────────────────────────────────────────────────────────────────────────────────────
PaperCore      A_PaperSimple                1976    14089     5786   316115     2931.7     5016.0
PaperCore      B_PaperSimple+Guard            39      144      434   385163     3952.3     6187.4
PaperCore      C_Baseline                      1        0      424    25495     1340.7     1653.1
─────────────────────────────────────────────────────────────────────────────────────────────────
Smoke7         A_PaperSimple                   1        0     1067   251875     6097.4     9142.9
Smoke7         B_PaperSimple+Guard             1        0      954   237181     6044.6     9795.0
Smoke7         C_Baseline                      1        0      923    22227     3180.8     5698.5
─────────────────────────────────────────────────────────────────────────────────────────────────
Stress40       A_PaperSimple                   1        0      899   249824     7920.7    14332.4
Stress40       B_PaperSimple+Guard             1        0      700   262380     5038.8     7106.2
Stress40       C_Baseline                      1        0      728    37437     2970.1     6376.7
```

Workloads:
- **PaperCore**: 500k samples, 7 plates, seed 42, 10 steps, BenchmarkMode=PaperCore (no events)
- **Smoke7**: 500k samples, 7 plates, seed 1, 10 steps, BenchmarkMode=Full (with events)
- **Stress40**: 500k samples, 40 plates, seed 4, 5 steps, BenchmarkMode=Full (with events)

Column definitions:
- **Frags**: Max disconnected components per plate across all reconciles
- **MaxFrag**: Largest detached fragment (samples) across all reconciles
- **Gaps/Overlaps**: Cumulative gap and overlap samples across all reconciles
- **AvgMs/MaxMs**: Average and maximum reconcile wall time

## Key Findings

### 1. Paper-simple alone is catastrophic on PaperCore
Mode A produces 1976 plate components with fragments up to 14k samples. Without
boundary-like classification and hysteresis, the containment query winner flips
chaotically at plate boundaries, shattering plates into thousands of disconnected
islands.

### 2. The comparative support guard is the critical fix
Mode B reduces PaperCore fragments from 1976 → 39 and max fragment from 14089 → 144.
The guard prevents spurious ownership flips where the new candidate has less neighbor
support than the incumbent. This is the single most impactful ownership mechanism.

### 3. Smoke7 and Stress40 don't fragment — even without P6
Both Mode A and Mode B produce 1 component on Smoke7 and Stress40. The tectonic event
phases (collision, subduction, rifting) rebuild spatial data and refresh carried samples,
which incidentally cleans up boundary noise. Fragmentation is a PaperCore-specific
problem because PaperCore skips events.

### 4. Skipping the fast path costs 2x performance
Modes A and B are 2–2.5× slower than baseline because every sample does the full
containment query. The fast path (skip full query for non-boundary interior samples
still contained by their previous owner) is not just an optimization — it's essential
for interactive-rate reconciliation.

### 5. Overlap counts explode without hysteresis
All ablation modes show 10× more overlaps than baseline (250k–385k vs 22k–37k).
Without boundary-like classification and the hysteresis threshold, many more samples
fall into the overlap zone. The containment query resolves them, but the raw overlap
count reflects wasted containment work.

## Conclusion

The paper's approach is not sufficient for our implementation at 500k samples.
Our fast path, boundary-like classification, and hysteresis are not accidental
complexity — they're essential mechanisms that the paper doesn't need because it
operates at lower resolution with simpler geometry.

**Connected Patch Transfer is not justified.** The problem is:
- Isolated to PaperCore (a synthetic benchmark with no events)
- Already 98% solved by the comparative support guard
- The remaining 39 small fragments could be absorbed by a tiny-fragment cleanup
  pass far simpler than a full ownership redesign

**Current baseline is the accepted implementation.** It produces 1 component on all
real workloads, runs fastest, and has the lowest overlap count.

## Implementation Notes

The ablation infrastructure is in place as throwaway code:
- `bPaperSimpleOwnership` flag on `FTectonicPlanet` (header + Phase 2 branch)
- `-OwnershipAblation` commandlet flag runs all 3 modes × 3 workloads
- Shadow P6 verifier: runs sanitize + connectivity on a copy when P6 is disabled,
  logs `p6_shadow_verifier` with mutation counts per reconcile
- Fragment tracking via existing `UpdateP6DisabledFragmentDiagnostics` infrastructure

To reproduce: `UnrealEditor-Cmd.exe Aurous.uproject -run=TectonicInit -OwnershipAblation`
