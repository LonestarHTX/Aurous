# Connected Patch Transfer Single-Reconcile Spec

Design-only note for the next ownership-maintenance prototype. This preserves the consolidated Connected Patch Transfer (CPT) spec, including the deferred-steady-state addendum.

## Scope

This spec plugs into the current global Phase 2 ownership evidence pass in `Source/Aurous/Private/TectonicPlanet.cpp` and uses the existing sample, plate, adjacency, and ownership journal structures in `Source/Aurous/Public/TectonicPlanet.h` and `Source/Aurous/Private/TectonicPlanet.cpp`.

It is intentionally correctness-first:

- no global sanitize/connectivity/rescue in the shipping path
- protected-plate preservation is kernel-based, not revert-based
- connectivity must be guaranteed by patch legality, not post-hoc proof
- contested-band growth is explicitly capped
- transfer ordering is deterministic
- singleton/tiny patches are handled explicitly

This spec is grounded in:

- `tectonic-planets-architecture-v4.md`
- `ProceduralTectonicPlanets.txt`
- anchored-frontier benchmark logs in `Saved/Logs/`
- P6 mutation / fragment / trace logs in `Saved/Logs/`

## Constants

- `CoreHopMinProtected = 2`
- `CoreHopMinRetirable = 1`
- `CoreMarginMin = 2 * BoundaryConfidenceThreshold`
- `KernelTargetProtected(P) = clamp(round(0.01 * Plates[P].InitialSampleCount), 64, 4096)`
- `KernelTargetRetirable(P) = clamp(round(0.002 * Plates[P].InitialSampleCount), 8, 512)`
- `BandWidth = 1` for ordinary motion reconcile
- `BandWidth = 2` only for explicit topology-event reconcile
- `BandCeiling = min(4 * I_active + 128, floor(0.10 * N))`
- `BandCeilingEvent = min(6 * I_active + 256, floor(0.15 * N))`
- `MinTransferPatchSize = 4`
- after tiny-patch coalescing, fail if `tiny_patch_count > 0.5 * patch_count`
- after tiny-patch coalescing, fail if `tiny_patch_samples > 0.2 * contested_band_samples`
- MVP transfer depth is `0` only: a patch may transfer only if it already touches the recipient's committed domain

## State Machine

### 0. Evidence Snapshot

Input state:

- `ReadSamples`
- Delaunay adjacency
- current plate soups / BVHs
- current global Phase 2 containment and support outputs

Operation:

- build immutable `ReconcileEvidence[s]` with:
- `TopPreferredPlate`
- `PrevOwnerStillContains`
- `NumContainingPlates`
- `bGap`
- `bOverlap`
- `OwnershipMargin`
- `SupportDelta = PrevOwnerSupportCount - BestContenderSupportCount`
- `bJunctionLike`

Output state:

- read-only ownership evidence for this reconcile
- ownership itself is unchanged

Edge cases:

- if a sample has `3+` viable non-donor recipients, tag it `bJunctionLike`

Complexity bound:

- unchanged current Phase 2 global cost, effectively `O(N)`

### 1. Protected Kernels

Input state:

- previous kernel descriptors per plate
- current plate membership
- `ReconcileEvidence`
- explicit topology-event tags

Operation:

- for ordinary motion reconcile, kernels are immutable and only validated
- rebuild a kernel only if plate `P` is tagged `KernelRebuildRequired`
- `CoreEligible(P)` is the connected subgraph of owned, non-gap, high-margin, boundary-distant interior samples
- rebuilt kernel is the connected component of `CoreEligible(P)` containing the previous kernel anchor if possible
- otherwise use the eligible component nearest the plate identity anchor
- clip the rebuilt kernel to `KernelTarget*`

Output state:

- one connected kernel per active plate

Edge cases:

- after rift, each child plate must inherit a connected kernel subset from the parent
- if a protected child cannot receive `KernelTargetProtected`, the rift event is invalid
- terrane transfer may not intersect a protected kernel
- core loss outside explicit merge or death is failure, not legitimate plate death

Complexity bound:

- ordinary reconcile: `O(sum |Kernel(P)|)` on touched protected plates
- rebuild: `O(|D(P)| + |E(D(P))|)` per topology-affected plate

### 2. Construct Contested Band

Input state:

- `ReadSamples`
- `ReconcileEvidence`
- kernels

Operation:

- exact primary seed set `S0` is:
- samples with `bGap`
- samples with `bOverlap`
- samples where `TopPreferredPlate != CurrentOwner`
- samples with owner disagreement and `abs(SupportDelta) <= 1`
- low margin alone is not a seed
- expand `S0` by `BandWidth` hops only through samples owned by the seed owner or one of the seed's candidate recipients
- compute `I_active` from previous-step boundary samples on touched interfaces incident to `S0`

Output state:

- contested band `B`
- touched plate set `T`
- active-interface measure `I_active`

Edge cases:

- isolated deep-interior disagreement with no active interface is `InteriorSeedFailure`
- if `|B| > BandCeiling`, abort as `BandCeilingExceeded`

Complexity bound:

- `O(|S0| + |B| + |E(B)|)`

### 3. Extract Donor Patches

Input state:

- contested band `B`
- `ReadSamples`
- adjacency

Operation:

- for each donor plate `A`, compute connected components of `{ s in B | ReadSamples[s].PlateId == A }`
- each component is a donor patch `Q`
- record:
- `AdjacentExternalDomains(Q)` from non-band neighbors
- `VoteCounts(Q)` from `TopPreferredPlate`
- `DonorCoreDistance(Q)` as minimum hop distance to `Kernel(A)`

Output state:

- donor-owned connected patches

Edge cases:

- if `|Q| < 4`, merge only into an edge-adjacent larger patch with the same donor and the same sole non-donor external domain
- if that merge is not possible, mark `Q` as `TinyDeferred`
- if a patch touches more than one non-donor external domain, mark it `JunctionDeferred`

Complexity bound:

- `O(|B| + |E(B)|)`

### 4. Assign Candidate Recipient

Input state:

- donor patches
- `ReconcileEvidence`

Operation:

- patch `Q` becomes `TransferCandidate(A->B)` only if:
- `AdjacentExternalDomains(Q) - {A} = {B}`
- `VoteCounts(Q)[B] > VoteCounts(Q)[A_retention]`
- `A_retention` counts samples where the previous owner still contains and support or seed state favors retaining `A`
- otherwise classify `Q` as `RetainDeferred`, `GapDeferred`, `TinyDeferred`, or `JunctionDeferred`

Output state:

- each patch has one deterministic state for this reconcile

Edge cases:

- no patch carries two recipients in MVP
- ambiguous multi-way cases defer

Complexity bound:

- `O(sum |Q|)`

### 5. Legality Tests

Input state:

- `TransferCandidate(A->B)` patches
- kernels
- donor local graphs

Operation:

- transfer is legal only if all four pass:
- `RecipientAttachment`: `Q` has at least one adjacency edge into `CommittedDomain(B)`, where `CommittedDomain(B) = Kernel(B) + uncontested non-band samples of B`
- `DonorRemainderConnectivity`: build `LocalCheckGraph(A)` from A-owned band samples, uncontested A-owned one-ring shell samples touching the band, and a synthetic root tied to `Kernel(A)` and shell; remove `Q`; BFS from root; every remaining local A-owned sample must still be reachable
- `ProtectedKernel`: `Q` must not intersect `Kernel(A)`
- `OutOfBand`: `Q` must lie wholly inside the original contested band

Output state:

- `LegalTransferPatch` or `IllegalDeferredPatch`

Edge cases:

- if `LocalCheckGraph(A)` is already disconnected before any transfer, fail immediately
- a patch attached to `B` only through another contested patch is illegal in this reconcile and must defer

Complexity bound:

- exact MVP legality is local BFS, `O(|LocalCheckGraph(A)| + |E(LocalCheckGraph(A))|)` per evaluated patch

### 6. Deterministic Transfer Ordering

Input state:

- all legal transfer patches

Operation:

- process donors in ascending `PlateId`
- within each donor, sort by:
- `DonorCoreDistance desc`
- `EvidenceDelta desc`
- `PatchSize desc`
- `RecipientPlateId asc`
- `MinSampleIndex asc`
- commit greedily in that order
- do not recompute recipient attachment from newly moved patches in the same reconcile

Output state:

- deterministic accepted transfer set
- deterministic deferred set

Edge cases:

- if patch `X` becomes legal only after patch `Y` moves first, `X` defers by design

Complexity bound:

- `O(P log P)` sort cost plus already-counted legality checks

### 7. Commit Transfers

Input state:

- accepted transfer set
- `ReadSamples`

Operation:

- only samples in accepted patches change ownership
- kernel samples and uncontested samples carry forward unchanged
- emit ownership-change journal entries as `(SampleIndex, OldPlateId, NewPlateId)`

Output state:

- provisional `WriteSamples`
- ownership-change journal

Edge cases:

- any ownership mutation outside the original band is `OutOfBandMutation`

Complexity bound:

- `O(changed_samples)`

### 8. Gap and Overlap Finalization

Input state:

- deferred patches
- provisional `WriteSamples`
- `ReconcileEvidence`
- outside-band neighbors

Operation:

- a deferred patch may become a true gap only if every sample in it has `bGap = true` and `PrevOwnerStillContains = false`
- resolve a true gap using only outside-band flanking neighbors and Phase 2 containing sets
- assign regenerated oceanic ownership to the nearer flanking plate and mark normal gap-generation data
- all other deferred patches retain previous owner

Output state:

- final ownership for all band samples

Edge cases:

- if a deferred patch is neither legally retainable nor a valid gap, it is `OrphanPatchFailure`
- no sample outside the original band may become newly contested in the same reconcile

Complexity bound:

- `O(|deferred_samples| + |E(deferred)|)`

### 9. Protected Check and Downstream Refresh

Input state:

- final `WriteSamples`
- kernels
- journal

Operation:

- directly verify every protected kernel sample still belongs to its plate
- feed the journal into the existing localized refresh path
- membership rebuild, carried-sample rebuild, canonical centers, triangle classification, spatial rebuild, boundary flags, and terrane refresh are all scoped to dirty plates and their one-ring halo

Output state:

- validated ownership delta
- dirty plate set for downstream work

Edge cases:

- kernel loss is failure, never repair
- a protected plate reduced to only its kernel is allowed only if an explicit topology event produced that state

Complexity bound:

- protected check: `O(sum |Kernel(P)|)` on protected touched plates
- downstream work remains dirty-scoped, not a global repair pass

### 10. Debug Verifier

Input state:

- final `WriteSamples`
- kernels
- contested-band telemetry
- dirty closure

Operation:

- verify invariants directly, without sanitize/connectivity/rescue
- for each dirty plate and dirty-plate neighbor, run connected-component analysis on non-gap owned samples using adjacency
- require exactly one component per active plate
- check kernel membership
- check `|B| <= BandCeiling`
- check tiny-patch ratios below threshold
- check that no sample outside `B` changed owner

Output state:

- pass or fail verdict with exact reason

Edge cases:

- legacy P6 may be run only as research telemetry on a copy, never as correctness

Complexity bound:

- `O(sum_{P in dirty closure} (|D(P)| + |E(D(P))|))`

## Deferred-But-Valid Steady State

- a deferred patch is valid only if the previous owner can keep it while remaining core-connected
- a deferred patch must stay wholly inside the original contested band
- a deferred patch must be neither a legal transfer nor a valid same-step gap
- deferral is not a pending-claim queue; ownership stays with the previous owner and the next reconcile recomputes from fresh evidence
- expected steady-state behavior is peel-layer lag: an outer legal patch transfers now and a newly exposed patch may become legal next reconcile
- one-step delay is normal
- two-step delay can still be normal near low-margin interfaces
- `JunctionDeferred` patches may persist longest, but only as small `3+` plate neighborhoods
- `TinyDeferred` patches must remain a minority and should usually disappear within `1-2` reconciles
- a pairwise deferred patch with the same donor, same sole recipient, and stable pro-recipient evidence may not persist beyond `3` reconciles
- any non-junction deferred lineage older than `5` reconciles is `InterfaceStallFailure`
- deferred samples above `25%` of contested-band samples in nominal runs, or above `40%` in stress runs, for `3` consecutive reconciles is failure
- deferred regions that expand in radius, drive the band toward `BandCeiling`, or keep generating mostly `1-3` sample residues are failure
- lineage is tracked only for telemetry: `Q_t` and `Q_t+1` are the same lineage if donor matches, recipient-set classification matches, and Jaccard overlap on sample indices is `>= 0.5`
- no automatic widening, no chained claims, no global fallback, and no rescue pass are allowed in response to persistent deferral
- ordinary motion reconcile may not self-promote into topology-event behavior just because a boundary is stuck

## Failure Conditions

- `BandCeilingExceeded`: contested band exceeds ceiling
- `KernelLoss`: any protected kernel sample changes owner
- `DetachedComponent`: more than one non-gap connected component on any dirty or touched plate
- `PatchDegeneracy`: after coalescing, tiny patches are a majority of patches or exceed `20%` of band samples
- `InteriorSeedFailure`: isolated interior disagreement seed not attached to an active interface
- `OrphanPatchFailure`: no legal transfer, no legal retain, and not a valid gap
- `OutOfBandMutation`: any ownership mutation outside the original contested band
- `InterfaceStallFailure`: deferred-but-valid pairwise patches persist past the allowed age or ratio thresholds
- transfer deadlock is not failure by itself; if all remaining patches deterministically classify to retain previous owner or valid gap, reconcile succeeds with zero transfers
- deadlock becomes failure only if any unresolved patch becomes orphaned or stalled

## Summary

This is the intended implementation contract:

- immutable kernels
- narrow capped contested band
- donor-owned connected patches
- exact local legality tests
- deterministic depth-0 transfer ordering
- direct invariant verification
- no shipping path that depends on Phase 6 discovering mistakes after the fact
