#pragma once

#include "CoreMinimal.h"
#include "PlateTriangleSoupAdapter.h"

enum class EOrogenyType : uint8
{
	None,
	Andean,
	Himalayan,
};

enum class EResampleTriggerReason : uint8
{
	None,
	Periodic,
	CollisionFollowup,
	RiftFollowup,
	SafetyValve,
	Manual,
};

// Policy-level control over when periodic maintenance resamples run.
// `PeriodicFull` remains the legacy baseline/default for regression comparison.
// `EventDrivenOnly` and `HybridStablePeriodic` are retained for experiment coverage.
// `PreserveOwnershipPeriodic` is the current preferred maintenance architecture.
// `PeriodicGlobalAuthoritativeSpike` is the isolated architecture-spike path for
// periodic global authoritative resampling without preserve ownership.
enum class EResamplingPolicy : uint8
{
	PeriodicFull,
	EventDrivenOnly,
	HybridStablePeriodic,
	PreserveOwnershipPeriodic,
	PeriodicGlobalAuthoritativeSpike,
};

// Internal ownership behavior used by a specific resample invocation.
// `PreserveOwnership` is the preferred maintenance mode.
enum class EResampleOwnershipMode : uint8
{
	FullResolution,
	StableOverlaps,
	PreserveOwnership,
};

enum class EGapResolutionPath : uint8
{
	None,
	NonDivergentProjection,
	NonDivergentNearestCopy,
	NonDivergentFallbackOceanized,
	DivergentOceanized,
};

struct FSample
{
	FVector3d Position = FVector3d::ZeroVector;
	int32 PlateId = INDEX_NONE;
	float ContinentalWeight = 0.0f;
	float Elevation = 0.0f;
	float Thickness = 0.0f;
	float Age = 0.0f;
	FVector3d RidgeDirection = FVector3d::ZeroVector;
	FVector3d FoldDirection = FVector3d::ZeroVector;
	EOrogenyType OrogenyType = EOrogenyType::None;
	int32 TerraneId = INDEX_NONE;
	float SubductionDistanceKm = -1.0f;
	bool bIsBoundary = false;
};

struct FCarriedSample
{
	int32 CanonicalSampleIndex = INDEX_NONE;
	float ContinentalWeight = 0.0f;
	float Elevation = 0.0f;
	float Thickness = 0.0f;
	float Age = 0.0f;
	FVector3d RidgeDirection = FVector3d::ZeroVector;
	FVector3d FoldDirection = FVector3d::ZeroVector;
	EOrogenyType OrogenyType = EOrogenyType::None;
	int32 TerraneId = INDEX_NONE;
	float SubductionDistanceKm = -1.0f;
	float SubductionSpeed = 0.0f;
};

struct FSphericalBoundingCap
{
	FVector3d Center = FVector3d::ZeroVector;
	double CosAngle = 1.0;
};

struct FTerrane
{
	int32 TerraneId = INDEX_NONE;
	int32 AnchorSample = INDEX_NONE;
	FVector3d Centroid = FVector3d::ZeroVector;
	double AreaKm2 = 0.0;
	int32 PlateId = INDEX_NONE;
};

struct FCollisionCandidate
{
	int32 SampleIndex = INDEX_NONE;
	int32 OverridingPlateId = INDEX_NONE;
	int32 SubductingPlateId = INDEX_NONE;
};

struct FCollisionEvent
{
	bool bDetected = false;
	int32 TerraneId = INDEX_NONE;
	int32 OverridingPlateId = INDEX_NONE;
	int32 SubductingPlateId = INDEX_NONE;
	FVector3d CollisionCenter = FVector3d::ZeroVector;
	TArray<int32> CollisionSampleIndices;
	TArray<int32> TerraneSampleIndices;
};

struct FBoundaryContactPersistence
{
	// Pair-level streak tracking for clean preserve-mode continental contact.
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	int32 ConsecutiveResamples = 0;
	int32 LastObservedStep = INDEX_NONE;
	int32 LastCandidateCount = 0;
	int32 LastLargestZoneSize = 0;
	int32 PeakLargestZoneSize = 0;
};

struct FPendingBoundaryContactCollisionEvent
{
	// Cached preserve-mode contact evidence consumed by same-step follow-up.
	bool bValid = false;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	TArray<int32> BoundarySeedSampleIndices;
	FVector3d ContactCenter = FVector3d::ZeroVector;
	int32 PeakZoneSize = 0;
	int32 ConsecutiveResamples = 0;
};

struct FPendingGeometricCollisionEvent
{
	// Cached preserve-mode geometric overlap evidence consumed by same-step follow-up.
	bool bValid = false;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	int32 OverridingPlateId = INDEX_NONE;
	int32 SubductingPlateId = INDEX_NONE;
	TArray<int32> SamplesFromAInsideB;
	TArray<int32> SamplesFromBInsideA;
	TArray<int32> OverlapSampleIndices;
	FVector3d ContactCenter = FVector3d::ZeroVector;
	int32 OverlapSampleCount = 0;
	int32 TerraneEstimate = 0;
	uint8 PolaritySelectionRule = 0;
	bool bPolarityChosenFromDirectionalEvidence = false;
	double TotalConvergenceMagnitudeAInsideBKmPerMy = 0.0;
	double TotalConvergenceMagnitudeBInsideAKmPerMy = 0.0;
	double TotalConvergenceMagnitudeKmPerMy = 0.0;
	double MaxConvergenceMagnitudeKmPerMy = 0.0;
	double EffectiveConvergenceKmPerMy = 0.0;
	double AccumulatedPenetrationKm = 0.0;
	double MeanOverlapDepthKm = 0.0;
	double MaxOverlapDepthKm = 0.0;
	int32 ObservationCount = 0;
};

struct FGeometricCollisionPairRecurrenceState
{
	// Persistent preserve-mode penetration tracking for normalized geometric candidate pairs.
	int32 ObservationCount = 0;
	int32 LastObservedStep = INDEX_NONE;
	int32 LastReceiverPlateId = INDEX_NONE;
	int32 LastDonorPlateId = INDEX_NONE;
	int32 LastOverlapSampleCount = 0;
	int32 LastTerraneEstimate = 0;
	double AccumulatedPenetrationKm = 0.0;
	double LastEffectiveConvergenceKmPerMy = 0.0;
	double LastMeanConvergenceMagnitudeKmPerMy = 0.0;
	double LastMaxConvergenceMagnitudeKmPerMy = 0.0;
	double LastMeanOverlapDepthKm = 0.0;
	double LastMaxOverlapDepthKm = 0.0;
};

struct FPendingRiftEvent
{
	bool bValid = false;
	bool bAutomatic = false;
	bool bForcedByTest = false;
	int32 ParentPlateId = INDEX_NONE;
	int32 ChildCount = 0;
	int32 ChildPlateA = INDEX_NONE;
	int32 ChildPlateB = INDEX_NONE;
	int32 ParentSampleCount = 0;
	int32 ParentContinentalSampleCount = 0;
	int32 ChildSampleCountA = 0;
	int32 ChildSampleCountB = 0;
	int32 EventSeed = 0;
	TArray<int32> ChildPlateIds;
	TArray<int32> ChildAnchorSampleIndices;
	TArray<int32> ChildSampleCounts;
	TArray<int32> FormerParentSampleIndices;
	TArray<int32> FormerParentTerraneIds;
	double ParentContinentalFraction = 0.0;
	double TriggerProbability = 0.0;
	double TriggerDraw = 0.0;
	double RiftMs = 0.0;
};

struct FSubductionComputationDiagnostics
{
	int32 SubductionFieldComputeCount = 0;
	int32 SlabPullComputeCount = 0;
	int32 ConvergentEdgeBuildCount = 0;
	int32 ReusedConvergentEdgeSetCount = 0;
	int32 ConvergentEdgeCount = 0;
	int32 SeedSampleCount = 0;
	int32 InfluencedSampleCount = 0;
	int32 SlabPullPlateCount = 0;
	int32 SlabPullTotalFrontSamples = 0;
	int32 CachedAdjacencyEdgeDistanceCount = 0;
	int64 CachedAdjacencyEdgeLookupCount = 0;
	double ConvergentEdgeBuildMs = 0.0;
	double SubductionFieldMs = 0.0;
	double SlabPullMs = 0.0;
};

struct FResamplingStats
{
	EResampleTriggerReason TriggerReason = EResampleTriggerReason::None;
	EResampleOwnershipMode OwnershipMode = EResampleOwnershipMode::FullResolution;
	int32 Step = 0;
	int32 Interval = 0;
	int32 PlateCount = 0;
	int32 ContinentalSampleCount = 0;
	int32 ContinentalSamplesBefore = 0;
	int32 ContinentalSamplesAfter = 0;
	int32 ContinentalSamplesLost = 0;
	int32 FormerContinentalSamplesTurnedOceanic = 0;
	int32 FormerContinentalDivergentGapCount = 0;
	int32 FormerContinentalNonDivergentGapCount = 0;
	int32 FormerContinentalNonDivergentGapProjectionResolvedCount = 0;
	int32 FormerContinentalNonDivergentGapNearestCopyResolvedCount = 0;
	int32 FormerContinentalNonDivergentFallbackOceanizedCount = 0;
	int32 FormerContinentalDivergentOceanizedCount = 0;
	int32 FormerContinentalProjectionRecoveredNonContinentalFinalCount = 0;
	int32 FormerContinentalProjectionRecoveredSamePlateNonContinentalFinalCount = 0;
	int32 FormerContinentalProjectionRecoveredChangedPlateNonContinentalFinalCount = 0;
	int32 FormerContinentalProjectionRecoveredPreserveModeNonContinentalFinalCount = 0;
	int32 FormerContinentalProjectionRecoveredFullResolutionNonContinentalFinalCount = 0;
	int32 FormerContinentalNearestCopyRecoveredNonContinentalFinalCount = 0;
	int32 FormerContinentalNonGapReclassifiedNonContinentalFinalCount = 0;
	int32 FormerContinentalChangedPlateNonContinentalFinalCount = 0;
	int32 FormerContinentalFullResolutionNonContinentalFinalCount = 0;
	int32 FullResolutionSamePlateNonContinentalFinalCount = 0;
	int32 FullResolutionChangedPlateNonContinentalFinalCount = 0;
	int32 FullResolutionCollisionFollowupSamePlateNonContinentalCount = 0;
	int32 FullResolutionCollisionFollowupChangedPlateNonContinentalCount = 0;
	int32 FullResolutionRiftFollowupSamePlateNonContinentalCount = 0;
	int32 FullResolutionRiftFollowupChangedPlateNonContinentalCount = 0;
	int32 FullResolutionOtherTriggerSamePlateNonContinentalCount = 0;
	int32 FullResolutionOtherTriggerChangedPlateNonContinentalCount = 0;
	int32 FormerContinentalPreserveModeNonGapNonContinentalFinalCount = 0;
	int32 FormerContinentalFallbackQueryNonContinentalFinalCount = 0;
	int32 FullResolutionSamePlateCWRetainedCount = 0;
	int32 FullResolutionSamePlateCWThresholdCrossingPreventedCount = 0;
	int32 FullResolutionCollisionFollowupSamePlateCWRetainedCount = 0;
	int32 FullResolutionRiftFollowupSamePlateCWRetainedCount = 0;
	int32 FullResolutionOtherTriggerSamePlateCWRetainedCount = 0;
	int32 AndeanContinentalGainCount = 0;
	int32 CollisionContinentalGainCount = 0;
	int32 NetContinentalSampleDelta = 0;
	int32 BoundarySampleCount = 0;
	int32 MaxComponentsPerPlate = 0;
	int32 ActiveCollisionTrackCount = 0;
	int32 ExactSingleHitCount = 0;
	int32 ExactMultiHitCount = 0;
	int32 RecoveryContainmentCount = 0;
	int32 RecoveryContinentalCount = 0;
	int32 TrueGapCount = 0;
	int32 InteriorSoupTriangleCount = 0;
	int32 BoundarySoupTriangleCount = 0;
	int32 TotalSoupTriangleCount = 0;
	int32 DroppedMixedTriangleCount = 0;
	int32 DuplicatedSoupTriangleCount = 0;
	int32 ForeignLocalVertexCount = 0;
	int32 GapCount = 0;
	int32 DivergentGapCount = 0;
	int32 NonDivergentGapCount = 0;
	int32 DivergentContinentalGapCount = 0;
	int32 NonDivergentContinentalGapCount = 0;
	int32 NonDivergentGapTriangleProjectionCount = 0;
	int32 NonDivergentGapNearestCopyCount = 0;
	int32 OverlapCount = 0;
	int32 ValidContainmentCount = 0;
	int32 MultiContainmentCount = 0;
	int32 NoValidHitCount = 0;
	int32 MissingLocalCarriedLookupCount = 0;
	int32 OceanicOceanicOverlapCount = 0;
	int32 OceanicContinentalOverlapCount = 0;
	int32 ContinentalContinentalOverlapCount = 0;
	int32 NonConvergentOverlapCount = 0;
	int32 HysteresisRetainedCount = 0;
	int32 HysteresisReassignedCount = 0;
	int32 PreserveOwnershipSamePlateHitCount = 0;
	int32 PreserveOwnershipSamePlateRecoveryCount = 0;
	int32 PreserveOwnershipFallbackQueryCount = 0;
	int32 PreserveOwnershipPlateChangedCount = 0;
	int32 PreserveOwnershipCWRetainedCount = 0;
	int32 PreserveOwnershipCWThresholdCrossingPreventedCount = 0;
	int32 PreserveOwnershipFallbackSamePlateRecontainedCount = 0;
	int32 PreserveOwnershipFallbackSamePlateRetainedCount = 0;
	int32 PreserveOwnershipFallbackChangedOwnerCount = 0;
	int32 PreserveOwnershipFallbackGapCount = 0;
	int32 PreserveOwnershipFallbackDivergentOceanizationCount = 0;
	int32 PreserveOwnershipContinentalLossCountAfterFallback = 0;
	int32 PreserveOwnershipPreviousOwnerHysteresisApplicationCount = 0;
	int32 PreserveOwnershipStronglyContinentalBoundarySavedCount = 0;
	int32 PreserveOwnershipFallbackChangedOwnerNonGapLossCount = 0;
	int32 PreserveOwnershipFallbackDivergentLossCount = 0;
	int32 PreserveOwnershipFallbackNonDivergentProjectionLossCount = 0;
	int32 PreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount = 0;
	int32 PreserveOwnershipFallbackStrongLossGE090Count = 0;
	int32 PreserveOwnershipFallbackStrongLossGE075Count = 0;
	int32 PreserveOwnershipFallbackStrongLossGE050Count = 0;
	TArray<int32> PreserveOwnershipFallbackLossPairPreviousPlateIds;
	TArray<int32> PreserveOwnershipFallbackLossPairFinalPlateIds;
	TArray<int32> PreserveOwnershipFallbackLossPairCounts;
	int32 TerraneCount = 0;
	int32 NewTerraneCount = 0;
	int32 MergedTerraneCount = 0;
	int32 SubductionFrontEdgeCount = 0;
	int32 SubductionSeedSampleCount = 0;
	int32 SubductionInfluencedCount = 0;
	int32 SlabPullPlateCount = 0;
	int32 SlabPullTotalFrontSamples = 0;
	int32 CollisionCount = 0;
	int32 CollisionDeferredCount = 0;
	int32 CollisionTerraneId = INDEX_NONE;
	int32 CollisionTerraneSampleCount = 0;
	int32 CollisionOverridingPlateId = INDEX_NONE;
	int32 CollisionSubductingPlateId = INDEX_NONE;
	int32 CollisionSurgeAffectedCount = 0;
	int32 CollisionCWBoostedCount = 0;
	int32 RiftCount = 0;
	int32 RiftParentPlateId = INDEX_NONE;
	int32 RiftChildCount = 0;
	int32 RiftChildPlateA = INDEX_NONE;
	int32 RiftChildPlateB = INDEX_NONE;
	int32 RiftParentSampleCount = 0;
	int32 RiftChildSampleCountA = 0;
	int32 RiftChildSampleCountB = 0;
	int32 RiftMinChildSampleCount = 0;
	int32 RiftMaxChildSampleCount = 0;
	int32 RiftTotalChildBoundaryContactEdges = 0;
	int32 RiftTerraneCountBefore = 0;
	int32 RiftTerraneCountAfter = 0;
	int32 RiftTerraneFragmentsOnChildA = 0;
	int32 RiftTerraneFragmentsOnChildB = 0;
	int32 RiftTerraneSplitCount = 0;
	int32 RiftTerranePreservedCount = 0;
	int32 RiftParentContinentalSampleCount = 0;
	int32 RiftEventSeed = 0;
	int32 RiftDivergentChildBoundaryEdgeCount = 0;
	int32 RiftLocalizedNonChildSampleRestoreCount = 0;
	int32 RiftLocalizedGapSampleRestoreCount = 0;
	int32 RiftLocalizedCWRestoredCount = 0;
	int32 RiftLocalizedCWPhantomPreventedCount = 0;
	int32 RiftLocalizedCWContinentalPreventedCount = 0;
	int32 RiftInterpolationCreatedGainCount = 0;
	int32 RiftFinalOwnerMismatchGainCountAfterLocalization = 0;
	int32 RiftFinalOwnerCWReconciledCount = 0;
	int32 RiftFinalOwnerMismatchGainCountBeforeReconciliation = 0;
	int32 RiftFinalOwnerMismatchGainCountAfterReconciliation = 0;
	int32 RiftFinalOwnerMismatchContinentalPreventedCount = 0;
	int32 RiftSameOwnerChildInterpolationGainCountBeforeReconciliation = 0;
	int32 RiftSameOwnerChildInterpolationGainCountAfterReconciliation = 0;
	int32 RiftFinalGainStartedBelow025Count = 0;
	int32 RiftFinalGainStartedBelow040Count = 0;
	int32 RiftFinalGainStartedBelow050Count = 0;
	int32 RiftChildStrayFragmentDetectedCount = 0;
	int32 RiftChildStrayFragmentReassignedCount = 0;
	int32 RiftLargestStrayChildFragmentSize = 0;
	int32 RiftStrayChildFragmentReassignedToSiblingCount = 0;
	int32 RiftStrayChildFragmentReassignedToOtherNeighborCount = 0;
	int32 RiftStrayFragmentReassignedByAdjacentSiblingCount = 0;
	int32 RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount = 0;
	int32 RiftStrayFragmentForcedNonChildAssignmentCount = 0;
	int32 RiftStrayFragmentZeroSiblingAdjacencyCount = 0;
	int32 RiftStrayFragmentPositiveSiblingAdjacencyCount = 0;
	int32 RiftStrayFragmentRecipientCandidateConsideredCount = 0;
	int32 RiftStrayFragmentRecipientIncoherenceRejectedCount = 0;
	int32 RiftStrayFragmentIncoherentForcedAssignmentCount = 0;
	int32 RiftLargestFragmentCausingRecipientGrowthSize = 0;
	TArray<int32> RiftChildPlateIds;
	TArray<int32> RiftChildAnchorSampleIndices;
	TArray<int32> RiftChildSampleCounts;
	TArray<int32> RiftLocalizedRestoredPreviousPlateIds;
	TArray<int32> RiftLocalizedRestoredPreviousPlateCounts;
	TArray<int32> RiftChildComponentCountsBeforeSuppression;
	TArray<int32> RiftChildComponentCountsAfterSuppression;
	TArray<int32> RiftChildComponentCountsBefore;
	TArray<int32> RiftChildComponentCountsAfter;
	TArray<int32> RiftChildTerraneFragmentCounts;
	TArray<int32> RiftStrayFragmentRecipientPlateIds;
	TArray<int32> RiftStrayFragmentRecipientTypeCodes;
	TArray<int32> RiftStrayFragmentSiblingEdgeCounts;
	TArray<int32> RiftStrayFragmentSizes;
	TArray<int32> RiftStrayFragmentRecipientComponentsBefore;
	TArray<int32> RiftStrayFragmentRecipientComponentsAfter;
	TArray<int32> RiftPositiveGrowthPlateIds;
	TArray<int32> RiftPositiveGrowthPlateTypeCodes;
	TArray<int32> RiftPositiveGrowthPlateComponentsBefore;
	TArray<int32> RiftPositiveGrowthPlateComponentsAfter;
	TArray<int32> RiftPositiveGrowthPlateLargestFragmentSizes;
	TArray<int32> RiftPreTerraneIds;
	TArray<int32> RiftPreTerraneTouchedChildCounts;
	bool bRiftWasAutomatic = false;
	int32 BoundaryContactCollisionCandidateCount = 0;
	int32 BoundaryContactCollisionPairCount = 0;
	int32 BoundaryContactLargestZoneSize = 0;
	int32 BoundaryContactTriggerPlateA = INDEX_NONE;
	int32 BoundaryContactTriggerPlateB = INDEX_NONE;
	int32 BoundaryContactBestPlateA = INDEX_NONE;
	int32 BoundaryContactBestPlateB = INDEX_NONE;
	int32 BoundaryContactBestZoneSize = 0;
	int32 BoundaryContactBestPersistenceCount = 0;
	int32 BoundaryContactPersistencePairA = INDEX_NONE;
	int32 BoundaryContactPersistencePairB = INDEX_NONE;
	int32 BoundaryContactPersistenceCount = 0;
	int32 GeometricCollisionCandidateCount = 0;
	int32 GeometricCollisionQualifiedCount = 0;
	int32 GeometricCollisionQualifiedButDonorAmbiguousCount = 0;
	int32 GeometricCollisionQualifiedButDonorSeedEmptyCount = 0;
	int32 GeometricCollisionQualifiedUsingDirectionalDonorCount = 0;
	int32 GeometricCollisionQualifiedUsingFallbackDonorRuleCount = 0;
	int32 GeometricCollisionRejectedByMassFilterCount = 0;
	int32 GeometricCollisionRejectedByOverlapDepthCount = 0;
	int32 GeometricCollisionRejectedByPersistentPenetrationCount = 0;
	int32 GeometricCollisionRejectedByOverlapDepthTotalOverlapSampleCount = 0;
	int32 GeometricCollisionRejectedByOverlapDepthMaxOverlapSampleCount = 0;
	int32 GeometricCollisionRejectedByOverlapDepthTotalTerraneEstimate = 0;
	int32 GeometricCollisionRejectedByOverlapDepthMaxTerraneEstimate = 0;
	int32 GeometricCollisionRejectedByPersistentPenetrationTotalOverlapSampleCount = 0;
	int32 GeometricCollisionRejectedByPersistentPenetrationMaxOverlapSampleCount = 0;
	int32 GeometricCollisionRejectedByPersistentPenetrationTotalTerraneEstimate = 0;
	int32 GeometricCollisionRejectedByPersistentPenetrationMaxTerraneEstimate = 0;
	int32 GeometricCollisionRejectedByPersistentPenetrationTotalObservationCount = 0;
	int32 GeometricCollisionRejectedByPersistentPenetrationMaxObservationCount = 0;
	double GeometricCollisionRejectedByOverlapDepthTotalMeanConvergenceKmPerMy = 0.0;
	double GeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy = 0.0;
	double GeometricCollisionRejectedByPersistentPenetrationTotalMeanConvergenceKmPerMy = 0.0;
	double GeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy = 0.0;
	double GeometricCollisionRejectedByPersistentPenetrationTotalAccumulatedPenetrationKm = 0.0;
	double GeometricCollisionRejectedByPersistentPenetrationMaxAccumulatedPenetrationKm = 0.0;
	int32 GeometricCollisionRejectedByEmptyTerraneCount = 0;
	int32 GeometricCollisionRejectedByRoleResolutionCount = 0;
	int32 GeometricCollisionRejectedBySeedDirectionCount = 0;
	int32 GeometricCollisionQualifiedDirectionalCount = 0;
	int32 GeometricCollisionDirectionalSeedCount = 0;
	int32 GeometricCollisionDirectionalSeedCountOpposite = 0;
	int32 GeometricCollisionSeedDirectionAuditCount = 0;
	int32 GeometricCollisionSeedDirectionMismatchCount = 0;
	int32 GeometricCollisionOnlyOverridingBucketPopulatedCount = 0;
	int32 GeometricCollisionOnlySubductingBucketPopulatedCount = 0;
	int32 GeometricCollisionBothDirectionalBucketsPopulatedCount = 0;
	int32 GeometricCollisionNeitherDirectionalBucketPopulatedCount = 0;
	int32 GeometricCollisionPolarityChosenFromDirectionalEvidenceCount = 0;
	int32 GeometricCollisionPolarityChosenFromBidirectionalTieBreakCount = 0;
	int32 GeometricCollisionFallbackUsedCount = 0;
	int32 GeometricCollisionOverlapSampleCount = 0;
	int32 GeometricCollisionPairCount = 0;
	int32 GeometricCollisionQualifiedPairCount = 0;
	int32 GeometricCollisionLargestTerraneEstimate = 0;
	int32 GeometricCollisionBestPlateA = INDEX_NONE;
	int32 GeometricCollisionBestPlateB = INDEX_NONE;
	int32 GeometricCollisionBestOverlapSampleCount = 0;
	int32 GeometricCollisionBestTerraneEstimate = 0;
	int32 GeometricCollisionBestSubductingOverlapSampleCount = 0;
	int32 GeometricCollisionBestOpposingSupportCount = 0;
	int32 CachedBoundaryContactSeedCount = 0;
	int32 CachedBoundaryContactTerraneSeedCount = 0;
	int32 CachedBoundaryContactTerraneRecoveredCount = 0;
	int32 CachedBoundaryContactPlateA = INDEX_NONE;
	int32 CachedBoundaryContactPlateB = INDEX_NONE;
	int32 GeometricCollisionExecutedPlateA = INDEX_NONE;
	int32 GeometricCollisionExecutedPlateB = INDEX_NONE;
	int32 GeometricCollisionExecutedOverlapSampleCount = 0;
	int32 GeometricCollisionExecutedTerraneEstimate = 0;
	int32 GeometricCollisionExecutedTerraneRecoveredCount = 0;
	int32 GeometricCollisionExecutedCollisionGainCount = 0;
	int32 GeometricCollisionExecutedCWBoostedCount = 0;
	int32 GeometricCollisionExecutedSurgeAffectedCount = 0;
	int32 GeometricCollisionExecutedFromDirectionalPolarityCount = 0;
	int32 BoundaryContactFallbackCollisionGainCount = 0;
	int32 BoundaryContactFallbackTerraneRecoveredCount = 0;
	int32 BoundaryContactFallbackCWBoostedCount = 0;
	int32 BoundaryContactFallbackSurgeAffectedCount = 0;
	double GeometricCollisionExecutedOverlapDepthKm = 0.0;
	double GeometricCollisionExecutedMaxOverlapDepthKm = 0.0;
	double GeometricCollisionExecutedMeanConvergenceKmPerMy = 0.0;
	double GeometricCollisionExecutedMaxConvergenceKmPerMy = 0.0;
	double GeometricCollisionExecutedAccumulatedPenetrationKm = 0.0;
	double GeometricCollisionRejectedByOverlapDepthTotalDepthKm = 0.0;
	double GeometricCollisionRejectedByOverlapDepthMaxDepthKm = 0.0;
	double GeometricCollisionRejectedByPersistentPenetrationTotalDepthKm = 0.0;
	double GeometricCollisionRejectedByPersistentPenetrationMaxDepthKm = 0.0;
	int32 GeometricCollisionExecutedDonorTerraneLocalityLimitedCount = 0;
	int32 GeometricCollisionExecutedObservationCount = 0;
	int32 TopologyGlobalMaxComponentsBefore = 0;
	int32 TopologyGlobalMaxComponentsAfter = 0;
	int32 TopologyPrimaryAffectedPlateId = INDEX_NONE;
	int32 TopologyPrimaryAffectedPlateComponentsBefore = 0;
	int32 TopologyPrimaryAffectedPlateComponentsAfter = 0;
	int32 TopologySecondaryAffectedPlateId = INDEX_NONE;
	int32 TopologySecondaryAffectedPlateComponentsBefore = 0;
	int32 TopologySecondaryAffectedPlateComponentsAfter = 0;
	int32 RiftParentComponentsBefore = 0;
	int32 RiftParentComponentsAfter = 0;
	int32 RiftAffectedPlateCountBefore = 0;
	int32 RiftAffectedPlateCountAfter = 0;
	int32 CollisionProposedGlobalMaxComponentsAfter = 0;
	int32 CollisionProposedDonorComponentsBefore = 0;
	int32 CollisionProposedDonorComponentsAfter = 0;
	int32 CollisionProposedReceiverComponentsBefore = 0;
	int32 CollisionProposedReceiverComponentsAfter = 0;
	int32 CollisionProposedDonorLargestRemainingComponentSize = 0;
	int32 CollisionProposedDonorNewFragmentCount = 0;
	int32 CollisionProposedDonorSmallestNewFragmentSize = 0;
	int32 CollisionProposedDonorLargestNewFragmentSize = 0;
	double CollisionProposedDonorMeanNewFragmentSize = 0.0;
	int32 CollisionProposedReceiverDisconnectedFragmentCount = 0;
	int32 CollisionProposedReceiverLargestNewDisconnectedFragmentSize = 0;
	int32 CollisionProposedTerraneSampleCount = 0;
	int32 CollisionDonorComponentsBefore = 0;
	int32 CollisionDonorComponentsAfter = 0;
	int32 CollisionDonorLargestRemainingComponentSize = 0;
	int32 CollisionDonorNewFragmentCount = 0;
	int32 CollisionDonorSmallestNewFragmentSize = 0;
	int32 CollisionDonorLargestNewFragmentSize = 0;
	double CollisionDonorMeanNewFragmentSize = 0.0;
	int32 CollisionAcceptedTerraneSampleCount = 0;
	int32 CollisionReceiverComponentsBefore = 0;
	int32 CollisionReceiverComponentsAfter = 0;
	int32 CollisionReceiverDisconnectedFragmentCount = 0;
	int32 CollisionReceiverLargestNewDisconnectedFragmentSize = 0;
	int32 CollisionTrimmedByDonorProtectionCount = 0;
	int32 CollisionRejectedByDonorProtectionCount = 0;
	int32 CollisionTrimmedByDonorComponentCapCount = 0;
	int32 CollisionTrimmedByDonorFragmentFloorCount = 0;
	int32 CollisionRejectedByDonorComponentCapCount = 0;
	int32 CollisionRejectedByDonorFragmentFloorCount = 0;
	int32 CollisionTrimmedByReceiverProtectionCount = 0;
	int32 CollisionRejectedByReceiverProtectionCount = 0;
	double CollisionTransferTrimRatio = 0.0;
	int32 GeometricCollisionRepeatedCandidatePlateA = INDEX_NONE;
	int32 GeometricCollisionRepeatedCandidatePlateB = INDEX_NONE;
	int32 GeometricCollisionRepeatedCandidateObservationCount = 0;
	int32 GeometricCollisionRepeatedCandidateOverlapSampleCount = 0;
	int32 GeometricCollisionRepeatedCandidateTerraneEstimate = 0;
	double GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm = 0.0;
	double GeometricCollisionRepeatedCandidateEffectiveConvergenceKmPerMy = 0.0;
	double GeometricCollisionRepeatedCandidateMeanConvergenceKmPerMy = 0.0;
	double GeometricCollisionRepeatedCandidateMaxConvergenceKmPerMy = 0.0;
	double GeometricCollisionRepeatedCandidateMeanOverlapDepthKm = 0.0;
	double GeometricCollisionRepeatedCandidateMaxOverlapDepthKm = 0.0;
	double GeometricCollisionDonorLocalityClampKm = 0.0;
	double GeometricCollisionInfluenceRadiusScale = 1.0;
	double GeometricCollisionPersistentPenetrationThresholdKm = 0.0;
	int32 CollisionRepeatedPairCount = 0;
	int32 CollisionRepeatedPairWithinCooldownCount = 0;
	bool bBoundaryContactCollisionTriggered = false;
	bool bBoundaryContactPersistenceTriggered = false;
	bool bGeometricCollisionBestPassedMassFilter = false;
	bool bUsedCachedBoundaryContactCollision = false;
	bool bUsedGeometricCollisionExecution = false;
	bool bCollisionReceiverTransferContiguousWithExistingTerritory = false;
	double ContinentalAreaFraction = 0.0;
	double BoundarySampleFraction = 0.0;
	double GapRate = 0.0;
	double OverlapRate = 0.0;
	double ElevationMinKm = 0.0;
	double ElevationMaxKm = 0.0;
	double ElevationMeanKm = 0.0;
	double RecoveryMeanDistance = 0.0;
	double RecoveryP95Distance = 0.0;
	double RecoveryMaxDistance = 0.0;
	double SubductionMeanDistanceKm = 0.0;
	double SubductionMaxDistanceKm = 0.0;
	double SlabPullMaxAxisChangeRad = 0.0;
	double CollisionSurgeRadiusRad = 0.0;
	double CollisionSurgeMeanElevationDelta = 0.0;
	double RiftMs = 0.0;
	double RiftMeanChildSampleCount = 0.0;
	double RiftParentContinentalFraction = 0.0;
	double RiftTriggerProbability = 0.0;
	double SoupBuildMs = 0.0;
	double OwnershipQueryMs = 0.0;
	double InterpolationMs = 0.0;
	double GapResolutionMs = 0.0;
	double OverlapClassificationMs = 0.0;
	double RepartitionMs = 0.0;
	double CollisionMs = 0.0;
	double SubductionDistanceFieldMs = 0.0;
	double SlabPullMs = 0.0;
	double TerraneDetectionMs = 0.0;
	double TotalMs = 0.0;
};

struct FPlate
{
	FPlate()
		: SoupAdapter(&SoupData)
	{
	}

	FPlate(const FPlate& Other)
		: Id(Other.Id)
		, RotationAxis(Other.RotationAxis)
		, AngularSpeed(Other.AngularSpeed)
		, OverlapScore(Other.OverlapScore)
		, SlabPullCorrectionAxis(Other.SlabPullCorrectionAxis)
		, SlabPullFrontSampleCount(Other.SlabPullFrontSampleCount)
		, LastRiftStep(Other.LastRiftStep)
		, CumulativeRotation(Other.CumulativeRotation)
		, MemberSamples(Other.MemberSamples)
		, CarriedSamples(Other.CarriedSamples)
		, CanonicalToCarriedIndex(Other.CanonicalToCarriedIndex)
		, InteriorTriangles(Other.InteriorTriangles)
		, BoundaryTriangles(Other.BoundaryTriangles)
		, SoupTriangles(Other.SoupTriangles)
		, BoundingCap(Other.BoundingCap)
		, SoupData(Other.SoupData)
		, SoupAdapter(&SoupData)
	{
	}

	FPlate(FPlate&& Other) noexcept
		: Id(Other.Id)
		, RotationAxis(Other.RotationAxis)
		, AngularSpeed(Other.AngularSpeed)
		, OverlapScore(Other.OverlapScore)
		, SlabPullCorrectionAxis(Other.SlabPullCorrectionAxis)
		, SlabPullFrontSampleCount(Other.SlabPullFrontSampleCount)
		, LastRiftStep(Other.LastRiftStep)
		, CumulativeRotation(Other.CumulativeRotation)
		, MemberSamples(MoveTemp(Other.MemberSamples))
		, CarriedSamples(MoveTemp(Other.CarriedSamples))
		, CanonicalToCarriedIndex(MoveTemp(Other.CanonicalToCarriedIndex))
		, InteriorTriangles(MoveTemp(Other.InteriorTriangles))
		, BoundaryTriangles(MoveTemp(Other.BoundaryTriangles))
		, SoupTriangles(MoveTemp(Other.SoupTriangles))
		, BoundingCap(Other.BoundingCap)
		, SoupData(MoveTemp(Other.SoupData))
		, SoupAdapter(&SoupData)
	{
	}

	FPlate& operator=(const FPlate& Other)
	{
		if (this != &Other)
		{
			Id = Other.Id;
			RotationAxis = Other.RotationAxis;
			AngularSpeed = Other.AngularSpeed;
			OverlapScore = Other.OverlapScore;
			SlabPullCorrectionAxis = Other.SlabPullCorrectionAxis;
			SlabPullFrontSampleCount = Other.SlabPullFrontSampleCount;
			LastRiftStep = Other.LastRiftStep;
			CumulativeRotation = Other.CumulativeRotation;
			MemberSamples = Other.MemberSamples;
			CarriedSamples = Other.CarriedSamples;
			CanonicalToCarriedIndex = Other.CanonicalToCarriedIndex;
			InteriorTriangles = Other.InteriorTriangles;
			BoundaryTriangles = Other.BoundaryTriangles;
			SoupTriangles = Other.SoupTriangles;
			BoundingCap = Other.BoundingCap;
			SoupData = Other.SoupData;
			SoupAdapter = FPlateTriangleSoupAdapter(&SoupData);
			SoupBVH = FPlateTriangleSoupBVH{};
		}
		return *this;
	}

	FPlate& operator=(FPlate&& Other) noexcept
	{
		if (this != &Other)
		{
			Id = Other.Id;
			RotationAxis = Other.RotationAxis;
			AngularSpeed = Other.AngularSpeed;
			OverlapScore = Other.OverlapScore;
			SlabPullCorrectionAxis = Other.SlabPullCorrectionAxis;
			SlabPullFrontSampleCount = Other.SlabPullFrontSampleCount;
			LastRiftStep = Other.LastRiftStep;
			CumulativeRotation = Other.CumulativeRotation;
			MemberSamples = MoveTemp(Other.MemberSamples);
			CarriedSamples = MoveTemp(Other.CarriedSamples);
			CanonicalToCarriedIndex = MoveTemp(Other.CanonicalToCarriedIndex);
			InteriorTriangles = MoveTemp(Other.InteriorTriangles);
			BoundaryTriangles = MoveTemp(Other.BoundaryTriangles);
			SoupTriangles = MoveTemp(Other.SoupTriangles);
			BoundingCap = Other.BoundingCap;
			SoupData = MoveTemp(Other.SoupData);
			SoupAdapter = FPlateTriangleSoupAdapter(&SoupData);
			SoupBVH = FPlateTriangleSoupBVH{};
		}
		return *this;
	}

	int32 Id = INDEX_NONE;
	FVector3d RotationAxis = FVector3d(0.0, 0.0, 1.0);
	double AngularSpeed = 0.0;
	int32 OverlapScore = 0;
	FVector3d SlabPullCorrectionAxis = FVector3d::ZeroVector;
	int32 SlabPullFrontSampleCount = 0;
	int32 LastRiftStep = INDEX_NONE;
	FQuat4d CumulativeRotation = FQuat4d::Identity;
	TArray<int32> MemberSamples;
	TArray<FCarriedSample> CarriedSamples;
	TMap<int32, int32> CanonicalToCarriedIndex;
	TArray<int32> InteriorTriangles;
	TArray<int32> BoundaryTriangles;
	TArray<int32> SoupTriangles;
	FSphericalBoundingCap BoundingCap;
	FPlateTriangleSoupData SoupData;
	FPlateTriangleSoupAdapter SoupAdapter;
	FPlateTriangleSoupBVH SoupBVH;
};

struct AUROUS_API FTectonicPlanetRuntimeConfig
{
	EResamplingPolicy ResamplingPolicy = EResamplingPolicy::PeriodicFull;
	bool bEnableAutomaticRifting = true;
	bool bEnableWarpedRiftBoundaries = true;
	int32 AutomaticRiftMinParentSamples = 2048;
	int32 AutomaticRiftMinContinentalSamples = 256;
	int32 AutomaticRiftCooldownSteps = 20;
	double AutomaticRiftBaseRatePerMy = 0.01;
	double AutomaticRiftMinContinentalFraction = 0.10;
	double RiftBoundaryWarpAmplitude = 0.18;
	double RiftBoundaryWarpFrequency = 1.5;
	double AndeanContinentalConversionRatePerMy = 0.005;
};

struct AUROUS_API FTectonicPlanet
{
	TArray<FSample> Samples;
	TArray<FPlate> Plates;
	TArray<FTerrane> Terranes;
	TArray<FIntVector> TriangleIndices;
	TArray<TArray<int32>> SampleAdjacency;
	TArray<TArray<double>> SampleAdjacencyEdgeDistancesKm;
	TArray<TArray<int32>> TriangleAdjacency;
	double PlanetRadiusKm = 6371.0;
	double ContainmentRecoveryTolerance = 0.003;
	int32 SampleCountConfig = 0;
	int32 PlateCountConfig = 0;
	int32 InitialPlateCountConfig = 0;
	int32 SimulationSeed = 0;
	int32 NextPlateId = 0;
	int32 NextTerraneId = 0;
	int32 CurrentStep = 0;
	int32 LastComputedResampleInterval = 0;
	int32 MaxResampleCount = INDEX_NONE;      // Test harness only.
	int32 MaxStepsWithoutResampling = INDEX_NONE; // Safety valve / experiments.
	int32 AutomaticRiftMinParentSamples = 2048;
	int32 AutomaticRiftMinContinentalSamples = 256;
	int32 AutomaticRiftCooldownSteps = 20;
	int32 CollisionPairCooldownResamples = 1;
	double AutomaticRiftBaseRatePerMy = 0.01;
	double AutomaticRiftMinContinentalFraction = 0.10;
	double AndeanContinentalConversionRatePerMy = 0.005;
	double GeometricCollisionMinOverlapDepthKm = 300.0;
	double GeometricCollisionMinPersistentPenetrationKm = 300.0;
	double GeometricCollisionDonorLocalityClampKm = 2250.0;
	double GeometricCollisionInfluenceRadiusScale = 2.1;
	int32 GeometricCollisionDonorMaxComponentIncrease = 10;
	int32 GeometricCollisionDonorMinNewFragmentSampleCount = 0;
	bool bEnableSlabPull = true;
	bool bEnableAndeanContinentalConversion = true;
	bool bEnableSubmergedContinentalRelaxation = false;
	double SubmergedContinentalRelaxationRatePerStep = 0.005; // CW decay per step for deeply submerged (elev < -2 km) continental crust
	bool bEnableOverlapHysteresis = false; // Legacy experiment path retained for tests.
	bool bEnableContinentalCollision = true;
	bool bEnableAutomaticRifting = true;
	bool bEnableWarpedRiftBoundaries = true;
	bool bDeferRiftFollowupResamplingToV6 = false;
	bool bUseVertexLevelSoupInclusionForTest = false; // Spike: include triangle in every plate that owns >= 1 vertex.
	bool bUseCachedSubductionAdjacencyEdgeDistancesForTest = true;
	double SubductionBaseUpliftKmPerMyForTest = -1.0; // < 0 uses the compiled default.
	bool bDisableSubductionElevationTransferForTest = false;
	double RiftBoundaryWarpAmplitude = 0.18;
	double RiftBoundaryWarpFrequency = 1.5;
	EResamplingPolicy ResamplingPolicy = EResamplingPolicy::PeriodicFull;
	EResampleTriggerReason LastResampleTriggerReason = EResampleTriggerReason::None;
	EResampleOwnershipMode LastResampleOwnershipMode = EResampleOwnershipMode::FullResolution;
	bool bPendingFullResolutionResample = false;
	bool bPendingBoundaryContactPersistenceReset = false;
	FResamplingStats LastResamplingStats;
	TArray<FResamplingStats> ResamplingHistory;
	TArray<int32> ResamplingSteps;
	// Preserve-mode collision triggering state. Cleared after follow-up execution.
	TMap<uint64, FBoundaryContactPersistence> BoundaryContactPersistenceByPair;
	TMap<uint64, int32> CollisionLastExecutionOrdinalByPair;
	TMap<uint64, FGeometricCollisionPairRecurrenceState> GeometricCollisionPairRecurrenceByKey;
	FPendingBoundaryContactCollisionEvent PendingBoundaryContactCollisionEvent;
	FPendingGeometricCollisionEvent PendingGeometricCollisionEvent;
	FPendingRiftEvent PendingRiftEvent;
	FSubductionComputationDiagnostics LastSubductionDiagnostics;
	int32 ResamplingExecutionOrdinal = 0;

	void Initialize(int32 InSampleCount, double InPlanetRadiusKm);
	void InitializePlates(int32 InPlateCount, int32 InRandomSeed, float InBoundaryWarpAmplitude, float InContinentalFraction);
	void AdvanceStep();
	void TriggerEventResampling(EResampleTriggerReason Reason);
	bool TriggerForcedRift(int32 ParentPlateId, int32 ChildCount, int32 Seed = 0);
	int32 FindPlateArrayIndexById(int32 PlateId) const;
	int32 FindLargestEligibleAutomaticRiftParentId(
		int32* OutContinentalSampleCount = nullptr,
		double* OutContinentalFraction = nullptr) const;
	double ComputeAutomaticRiftProbabilityForSampleCount(int32 ParentSampleCount, double ContinentalFraction) const;
	void ComputePlateScores();
	void BuildContainmentSoups();
	void QueryOwnership(
		TArray<int32>& OutNewPlateIds,
		TArray<int32>& OutContainingTriangles,
		TArray<FVector3d>& OutBarycentricCoords,
		TArray<uint8>& OutGapFlags,
		TArray<uint8>& OutOverlapFlags,
		int32& OutGapCount,
		int32& OutOverlapCount,
		TArray<TArray<int32>>* OutOverlapPlateIds = nullptr,
		FResamplingStats* OutStats = nullptr,
		EResampleOwnershipMode OwnershipMode = EResampleOwnershipMode::FullResolution,
		TArray<uint8>* OutPreserveOwnershipCWRetainFlags = nullptr,
		TArray<uint8>* OutPreserveOwnershipFallbackQueryFlags = nullptr) const;
	void InterpolateFromCarried(
		const TArray<int32>& NewPlateIds,
		const TArray<int32>& ContainingTriangles,
		const TArray<FVector3d>& BarycentricCoords,
		TArray<float>& OutSubductionDistances,
		TArray<float>& OutSubductionSpeeds,
		int32* OutMissingLocalCarriedLookupCount = nullptr);
	// Testable seam for the narrow M6d fix: preserve CW only on same-plate preserve refreshes.
	void ApplyPreserveOwnershipContinentalWeightRetention(
		const TArray<uint8>& PreserveOwnershipCWRetainFlags,
		const TArray<float>& PreviousContinentalWeights,
		EResampleOwnershipMode OwnershipMode,
		EResampleTriggerReason TriggerReason,
		FResamplingStats* InOutStats = nullptr);
	// Testable seam for the narrow M6ah fix: preserve CW when a preserve-mode
	// fallback path resolves back onto the same owner after query/gap handling.
	void ApplyPreserveOwnershipFallbackSamePlateRetention(
		const TArray<int32>& NewPlateIds,
		const TArray<int32>& PreviousPlateAssignments,
		const TArray<float>& PreviousContinentalWeights,
		const TArray<uint8>& PreserveOwnershipFallbackQueryFlags,
		const TArray<EGapResolutionPath>& GapResolutionPaths,
		EResampleOwnershipMode OwnershipMode,
		EResampleTriggerReason TriggerReason,
		FResamplingStats* InOutStats = nullptr);
	// Testable seam for the narrow M6u fix: preserve CW only for same-plate
	// full-resolution non-gap refreshes when interpolation alone would extinguish
	// formerly continental crust.
	void ApplyFullResolutionSamePlateContinentalWeightRetention(
		const TArray<int32>& NewPlateIds,
		const TArray<int32>& PreviousPlateAssignments,
		const TArray<float>& PreviousContinentalWeights,
		const TArray<EGapResolutionPath>& GapResolutionPaths,
		EResampleOwnershipMode OwnershipMode,
		EResampleTriggerReason TriggerReason,
		FResamplingStats* InOutStats = nullptr);
	void ApplyRiftFollowupLocalizationOverride(
		TArray<int32>& InOutNewPlateIds,
		const TArray<int32>& PreviousPlateAssignments,
		const TArray<float>& PreviousContinentalWeights,
		TArray<uint8>* InOutGapFlags = nullptr,
		TArray<uint8>* InOutOverlapFlags = nullptr,
		TArray<TArray<int32>>* InOutOverlapPlateIds = nullptr,
		TArray<float>* InOutSubductionDistances = nullptr,
		TArray<float>* InOutSubductionSpeeds = nullptr,
		TArray<EGapResolutionPath>* InOutGapResolutionPaths = nullptr,
		TArray<uint8>* OutLocalizedRestoreFlags = nullptr,
		FResamplingStats* InOutStats = nullptr);
	void ApplyRiftFollowupFinalOwnerContinentalWeightReconciliation(
		const TArray<int32>& InterpolationPlateIds,
		const TArray<int32>& FinalPlateIds,
		const TArray<int32>& PreviousPlateAssignments,
		const TArray<float>& PreviousContinentalWeights,
		const TArray<uint8>& LocalizedRestoreFlags,
		FResamplingStats* InOutStats = nullptr);
	void ApplyRiftChildCoherenceProtection(
		TArray<int32>& InOutNewPlateIds,
		const TArray<int32>& PreviousPlateAssignments,
		TArray<uint8>* InOutGapFlags = nullptr,
		TArray<uint8>* InOutOverlapFlags = nullptr,
		TArray<TArray<int32>>* InOutOverlapPlateIds = nullptr,
		TArray<EGapResolutionPath>* InOutGapResolutionPaths = nullptr,
		FResamplingStats* InOutStats = nullptr) const;
	void ResolveGaps(
		TArray<int32>& NewPlateIds,
		const TArray<uint8>& GapFlags,
		TArray<float>& InOutSubductionDistances,
		TArray<float>& InOutSubductionSpeeds,
		FResamplingStats* InOutStats = nullptr,
		TArray<EGapResolutionPath>* OutGapResolutionPaths = nullptr);
	void RepartitionMembership(
		const TArray<int32>& NewPlateIds,
		const TArray<float>* InSubductionDistances = nullptr,
		const TArray<float>* InSubductionSpeeds = nullptr);
	void CollectCollisionCandidates(
		const TArray<uint8>& OverlapFlags,
		const TArray<TArray<int32>>& OverlapPlateIds,
		TArray<FCollisionCandidate>& OutCandidates) const;
	bool DetectGeometricCollisionOverlapDiagnostics(
		const TArray<int32>& OwningPlateIds,
		FResamplingStats* InOutStats = nullptr,
		FPendingGeometricCollisionEvent* OutPendingEvent = nullptr);
	bool DetectBoundaryContactCollisionTrigger(
		FResamplingStats* InOutStats = nullptr,
		FPendingBoundaryContactCollisionEvent* OutPendingEvent = nullptr) const;
	bool UpdateBoundaryContactCollisionPersistence(
		FResamplingStats& InOutStats,
		int32 ResampleInterval);
	void ResetBoundaryContactCollisionPersistence();
	bool DetectAndApplyCollision(
		EResampleTriggerReason TriggerReason,
		const TArray<uint8>& OverlapFlags,
		const TArray<TArray<int32>>& OverlapPlateIds,
		const TArray<int32>& PreviousPlateIds,
		const TArray<float>& PreviousContinentalWeights,
		const TArray<int32>& PreviousTerraneAssignments,
		TArray<int32>& InOutNewPlateIds,
		FCollisionEvent& OutEvent,
		FResamplingStats* InOutStats = nullptr);
	void ApplyCollisionElevationSurge(
		const FCollisionEvent& CollisionEvent,
		FResamplingStats* InOutStats = nullptr);
	void ComputeSubductionDistanceField(FResamplingStats* InOutStats = nullptr);
	void ComputeSlabPullCorrections(FResamplingStats* InOutStats = nullptr);
	void ComputeSubductionState(FResamplingStats* InOutStats = nullptr);
	static double SubductionDistanceTransfer(double DistanceRad, double ControlDistanceRad, double MaxDistanceRad);
	static double CollisionBiweightKernel(double DistanceRad, double RadiusRad);
	void ClassifyOverlaps(
		const TArray<uint8>& OverlapFlags,
		const TArray<int32>& NewPlateIds,
		FResamplingStats& Stats) const;
	void DetectTerranes(const TArray<FTerrane>& PreviousTerranes);
	void PerformResampling(
		EResampleOwnershipMode OwnershipMode = EResampleOwnershipMode::FullResolution,
		EResampleTriggerReason TriggerReason = EResampleTriggerReason::None);
	int32 ComputeResampleInterval() const;
	bool TryTriggerAutomaticRift();
	bool TriggerForcedRiftInternal(
		int32 ParentPlateId,
		int32 ChildCount,
		int32 Seed,
		bool bAutomatic,
		double TriggerProbability,
		double TriggerDraw,
		int32 ParentContinentalSampleCount,
		double ParentContinentalFraction);
	bool TriggerForcedRiftInternal(
		int32 ParentPlateId,
		int32 ChildCount,
		int32 Seed,
		bool bAutomatic,
		double TriggerProbability,
		int32 ParentContinentalSampleCount,
		double ParentContinentalFraction);
	bool IsPlateEligibleForAutomaticRift(
		const FPlate& Plate,
		int32 ChildCount,
		int32& OutContinentalSampleCount,
		double& OutContinentalFraction) const;
};

AUROUS_API FTectonicPlanetRuntimeConfig GetM6BaselineRuntimeConfig();
AUROUS_API FTectonicPlanetRuntimeConfig GetArchitectureSpikeARuntimeConfig();
AUROUS_API FTectonicPlanetRuntimeConfig CaptureTectonicPlanetRuntimeConfig(const FTectonicPlanet& Planet);
AUROUS_API void ApplyTectonicPlanetRuntimeConfig(
	FTectonicPlanet& Planet,
	const FTectonicPlanetRuntimeConfig& Config);
AUROUS_API FString DescribeTectonicPlanetRuntimeConfig(const FTectonicPlanetRuntimeConfig& Config);
