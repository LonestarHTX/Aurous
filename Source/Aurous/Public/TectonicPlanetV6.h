#pragma once

#include "CoreMinimal.h"
#include "TectonicPlanet.h"

enum class ETectonicPlanetV6SolveTrigger : uint8
{
	None,
	Periodic,
	Manual,
	Rift,
};

enum class ETectonicPlanetV6PeriodicSolveMode : uint8
{
	Phase3Authoritative,
	ThesisRemeshSpike,
	ThesisCopiedFrontierSpike,
	ThesisCopiedFrontierProcessSpike,
	ThesisPartitionedFrontierSpike,
	ThesisPartitionedFrontierProcessSpike,
	ThesisPartitionedFrontierPropagationSpike,
	ThesisPartitionedFrontierPropagationProcessSpike,
	ThesisPlateSubmeshSpike,
};

enum class ETectonicPlanetV6BoundaryOutcome : uint8
{
	None,
	RetainedOwner,
	ReassignedOwner,
	DivergentOceanic,
};

enum class ETectonicPlanetV6ResolutionKind : uint8
{
	None,
	SingleCandidate,
	OverlapWinner,
	BoundaryRetained,
	BoundaryReassigned,
	BoundaryOceanic,
	NearestTriangleRecovery,
	NearestMemberRecovery,
	ExplicitFallback,
	ThesisRemeshHit,
	ThesisRemeshMissOceanic,
	ThesisRemeshMissDestructiveExclusion,
	ThesisRemeshMissAmbiguous,
	ThesisRemeshRetainedSyntheticCoverage,
	ThesisRemeshRetainedOutsideActiveZone,
	ThesisRemeshTransferFallback,
};

enum class ETectonicPlanetV6ActiveZoneCause : uint8
{
	None,
	DivergenceFill,
	ConvergentSubduction,
	CollisionContact,
	Rift,
	GenericQueryCompetition,
	UnknownOther,
};

enum class ETectonicPlanetV6ActiveZoneClassifierMode : uint8
{
	BroadBoundaryBand,
	NarrowTectonicPairs,
	PersistentPairLocal,
	PersistentPairLocalTightFreshAdmission,
};

enum class ETectonicPlanetV6PaperSurrogateFieldMode : uint8
{
	FullState,
	ContinentalWeightElevationThickness,
	ContinentalWeightThickness,
	ContinentalWeightThicknessSelectiveElevation,
};

enum class ETectonicPlanetV6TransferSourceKind : uint8
{
	None,
	Triangle,
	SingleSource,
	StructuredSynthetic,
	OceanicCreation,
	Defaulted,
};

enum class ETectonicPlanetV6CategoricalTransferKind : uint8
{
	None,
	MajorityVote,
	DominantSource,
};

enum class ETectonicPlanetV6DirectionalTransferKind : uint8
{
	None,
	WeightedBlend,
	DominantSourceFallback,
	SingleSourceProjection,
	ZeroVectorFallback,
};

struct AUROUS_API FTectonicPlanetV6OwnerCandidate
{
	int32 PlateId = INDEX_NONE;
	int32 TriangleIndex = INDEX_NONE;
	FVector3d Barycentric = FVector3d(-1.0, -1.0, -1.0);
	double FitScore = -TNumericLimits<double>::Max();
};

struct AUROUS_API FTectonicPlanetV6RecoveryCandidate
{
	int32 PlateId = INDEX_NONE;
	int32 TriangleIndex = INDEX_NONE;
	int32 LocalTriangleIndex = INDEX_NONE;
	FVector3d Barycentric = FVector3d(-1.0, -1.0, -1.0);
	double DistanceRadians = TNumericLimits<double>::Max();
};

struct AUROUS_API FTectonicPlanetV6BoundaryMotionSample
{
	int32 PlateId = INDEX_NONE;
	FVector3d SurfaceVelocity = FVector3d::ZeroVector;
	FVector3d NeighborTangent = FVector3d::ZeroVector;
	int32 NeighborVoteCount = 0;
};

struct AUROUS_API FTectonicPlanetV6TransferResolutionCounts
{
	int32 ExactSingleHit = 0;
	int32 OverlapWinner = 0;
	int32 BoundaryRetained = 0;
	int32 BoundaryReassigned = 0;
	int32 BoundaryOceanic = 0;
	int32 TriangleRecovery = 0;
	int32 CoherenceReassigned = 0;
	int32 Other = 0;
};

struct AUROUS_API FTectonicPlanetV6BoundaryOutcomeTransferStats
{
	int32 SampleCount = 0;
	int32 TriangleTransferCount = 0;
	int32 TriangleRecoveryTransferCount = 0;
	int32 SingleSourceTransferCount = 0;
	int32 NearestMemberSingleSourceTransferCount = 0;
	int32 ExplicitFallbackSingleSourceTransferCount = 0;
	int32 OceanicCreationCount = 0;
	int32 DefaultTransferCount = 0;
	int32 ContinentalWeightThresholdMismatchCount = 0;
};

struct AUROUS_API FTectonicPlanetV6DestructiveTrackingLifecycleStats
{
	int32 EntryCandidateCount = 0;
	int32 EntrySeedConvergentEdgeCandidateCount = 0;
	int32 EntryOverlapTestedCount = 0;
	int32 EntryAdmittedCount = 0;
	int32 EntrySeedConvergentEdgeAdmittedCount = 0;
	int32 EntrySeedConvergentEdgeSurvivedOneTimestepCount = 0;
	int32 EntrySeedConvergentEdgeSurvivedToRemeshCount = 0;
	int32 EntryRejectedNoOverlapCount = 0;
	int32 EntryRejectedAuthorizationCount = 0;
	int32 EntryRejectedAlreadyTrackedCount = 0;
	int32 EntryAdmittedSubductionCount = 0;
	int32 EntryAdmittedCollisionCount = 0;
	int32 ActiveAdvanceCount = 0;
	int32 ActiveOverlapConfirmedCount = 0;
	int32 ActiveOverlapRejectedCount = 0;
	int32 NeighborCandidateGeneratedCount = 0;
	int32 NeighborCandidateTestedCount = 0;
	int32 NeighborAdmittedCount = 0;
	int32 NeighborRejectedNoOverlapCount = 0;
	int32 NeighborRejectedAuthorizationCount = 0;
	int32 NeighborRejectedAlreadyTrackedCount = 0;
	int32 NeighborAdmittedSubductionCount = 0;
	int32 NeighborAdmittedCollisionCount = 0;
	int32 TopologyNeighborCandidateGeneratedCount = 0;
	int32 TopologyNeighborAdmittedCount = 0;
	int32 TopologyNeighborExpiredBeforeRemeshCount = 0;
	int32 DirectionalNeighborCandidateConsideredCount = 0;
	int32 DirectionalNeighborAdmittedCount = 0;
	int32 DirectionalNeighborRejectedNotInwardCount = 0;
	int32 DirectionalNeighborSurvivedToRemeshCount = 0;
	int32 ExpiredSubductionCount = 0;
	int32 ExpiredCollisionCount = 0;
};

struct AUROUS_API FTectonicPlanetV6TransferDebugInfo
{
	int32 DominantSourceCanonicalSampleIndex = INDEX_NONE;
	int32 DominantSourceTriangleCorner = INDEX_NONE;
	ETectonicPlanetV6TransferSourceKind SourceKind = ETectonicPlanetV6TransferSourceKind::None;
	ETectonicPlanetV6CategoricalTransferKind TerraneTransferKind = ETectonicPlanetV6CategoricalTransferKind::None;
	ETectonicPlanetV6CategoricalTransferKind OrogenyTransferKind = ETectonicPlanetV6CategoricalTransferKind::None;
	ETectonicPlanetV6DirectionalTransferKind RidgeDirectionTransferKind = ETectonicPlanetV6DirectionalTransferKind::None;
	ETectonicPlanetV6DirectionalTransferKind FoldDirectionTransferKind = ETectonicPlanetV6DirectionalTransferKind::None;
	bool bContinentalWeightWouldCrossThresholdUnderBarycentricBlend = false;
	bool bUsedTriangleRecovery = false;
	bool bUsedBoundarySymmetricTriangleTransfer = false;
	bool bUsedCopiedFrontierTriangleTransfer = false;
	bool bUsedPlateSubmeshTriangleTransfer = false;
	bool bUsedNearestMemberFallback = false;
	bool bUsedExplicitFallback = false;
};

struct AUROUS_API FTectonicPlanetV6ResolvedSample
{
	int32 FinalPlateId = INDEX_NONE;
	int32 PreviousPlateId = INDEX_NONE;
	int32 PreCoherencePlateId = INDEX_NONE;
	int32 BoundaryOtherPlateId = INDEX_NONE;
	int32 ActiveZonePrimaryPlateId = INDEX_NONE;
	int32 ActiveZoneSecondaryPlateId = INDEX_NONE;
	int32 SourceTriangleIndex = INDEX_NONE;
	int32 SourceLocalTriangleIndex = INDEX_NONE;
	int32 SourceCanonicalSampleIndex = INDEX_NONE;
	int32 ExactCandidateCount = 0;
	FVector3d SourceBarycentric = FVector3d(-1.0, -1.0, -1.0);
	double WinningFitScore = -TNumericLimits<double>::Max();
	double RecoveryDistanceRadians = -1.0;
	double BoundaryRelativeNormalVelocity = 0.0;
	ETectonicPlanetV6ResolutionKind ResolutionKind = ETectonicPlanetV6ResolutionKind::None;
	ETectonicPlanetV6ActiveZoneCause ActiveZoneCause = ETectonicPlanetV6ActiveZoneCause::None;
	ETectonicPlanetV6BoundaryOutcome BoundaryOutcome = ETectonicPlanetV6BoundaryOutcome::None;
	bool bHasStructuredSyntheticFill = false;
	bool bRetainedSyntheticCoverage = false;
	bool bActiveZoneSample = false;
	bool bBoundaryDecision = false;
	bool bCoherenceReassigned = false;
	bool bOutsideActiveZoneQueryMiss = false;
	bool bOutsideActiveZoneCoverageDeficit = false;
	bool bAuthorityRetainedOutsideActiveZone = false;
	FCarriedSample StructuredSyntheticFillSample;
	FTectonicPlanetV6TransferDebugInfo TransferDebug;
};

struct AUROUS_API FTectonicPlanetV6PhaseTiming
{
	double PreSolveCaptureMs = 0.0;
	double SampleAdjacencyBuildMs = 0.0;
	double ActiveZoneMaskMs = 0.0;
	double CopiedFrontierMeshBuildMs = 0.0;
	double QueryGeometryBuildMs = 0.0;
	double FrontierPointSetBuildMs = 0.0;
	double ResolveTransferLoopMs = 0.0;
	double HitSearchMs = 0.0;
	double ZeroHitRecoveryMs = 0.0;
	double DirectHitTransferMs = 0.0;
	double FallbackTransferMs = 0.0;
	double QuietInteriorPreservationMs = 0.0;
	double AttributionMs = 0.0;
	double RepartitionMembershipMs = 0.0;
	double PlateScoresMs = 0.0;
	double SubductionDistanceFieldMs = 0.0;
	double SlabPullMs = 0.0;
	double TerraneDetectionMs = 0.0;
	double ComponentAuditMs = 0.0;
	double RebuildCopiedFrontierMeshesMs = 0.0;
	double CollisionShadowMs = 0.0;
	double CollisionExecutionMs = 0.0;
};

struct AUROUS_API FTectonicPlanetV6PeriodicSolveStats
{
	ETectonicPlanetV6SolveTrigger Trigger = ETectonicPlanetV6SolveTrigger::None;
	ETectonicPlanetV6PeriodicSolveMode SolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative;
	int32 Step = 0;
	int32 Interval = 0;
	int32 SolveIndex = 0;
	int32 PlateCount = 0;
	double SolveMilliseconds = 0.0;
	int32 HitCount = 0;
	int32 MissCount = 0;
	int32 MultiHitCount = 0;
	int32 SingleCandidateWinnerCount = 0;
	int32 MultiCandidateWinnerCount = 0;
	int32 ZeroCandidateCount = 0;
	int32 ZeroCandidateTriangleRecoveryCount = 0;
	int32 ZeroCandidateMemberRecoveryCount = 0;
	int32 ExplicitFallbackCount = 0;
	int32 CoherenceReassignedSampleCount = 0;
	int32 CoherenceRemovedComponentCount = 0;
	int32 CoherenceLargestRemovedComponentSize = 0;
	int32 BoundarySampleCount = 0;
	int32 BoundaryDecisionSampleCount = 0;
	int32 BoundaryRetainedCount = 0;
	int32 BoundaryReassignedCount = 0;
	int32 BoundaryOceanicCount = 0;
	int32 BoundaryLimitedFallbackCount = 0;
	int32 GapCount = 0;
	int32 OverlapCount = 0;
	int32 TriangleTransferCount = 0;
	int32 DirectHitTriangleTransferCount = 0;
	int32 SingleSourceTransferCount = 0;
	int32 OceanicCreationCount = 0;
	int32 DefaultTransferCount = 0;
	int32 TransferFallbackCount = 0;
	int32 NearestMemberFallbackTransferCount = 0;
	int32 ExplicitFallbackTransferCount = 0;
	int32 PlateLocalVertexCount = 0;
	int32 PlateLocalTriangleCount = 0;
	int32 CopiedFrontierVertexCount = 0;
	int32 CopiedFrontierTriangleCount = 0;
	int32 CopiedFrontierCarriedSampleCount = 0;
	int32 CopiedFrontierHitCount = 0;
	int32 PlateSubmeshFrontierVertexCount = 0;
	int32 PlateSubmeshFrontierTriangleCount = 0;
	int32 PlateSubmeshFrontierCarriedSampleCount = 0;
	int32 PlateSubmeshFrontierHitCount = 0;
	int32 PlateSubmeshRetriangulatedTriangleCount = 0;
	int32 PlateSubmeshComponentCount = 0;
	int32 PlateSubmeshWholeMixedTriangleDuplicationCount = 0;
	int32 InteriorHitCount = 0;
	int32 SubductionFieldComputeCount = 0;
	int32 SlabPullComputeCount = 0;
	int32 ConvergentEdgeBuildCount = 0;
	int32 ReusedConvergentEdgeSetCount = 0;
	int32 SubductionConvergentEdgeCount = 0;
	int32 SlabPullConvergentEdgeCount = 0;
	int32 SubductionSeedSampleCount = 0;
	int32 SubductionInfluencedCount = 0;
	int32 SlabPullFrontSampleCount = 0;
	int32 CachedAdjacencyEdgeDistanceCount = 0;
	int64 CachedAdjacencyEdgeLookupCount = 0;
	int64 SubductionQueuePushCount = 0;
	int64 SubductionQueuePopCount = 0;
	int64 SubductionRelaxationCount = 0;
	double SubductionConvergentEdgeBuildMs = 0.0;
	double SubductionSeedInitializationMs = 0.0;
	double SubductionPropagationMs = 0.0;
	double SubductionFinalizeMs = 0.0;
	double SlabPullConvergentEdgeBuildMs = 0.0;
	double SlabPullFrontierBuildMs = 0.0;
	double SlabPullApplyMs = 0.0;
	int64 HitSearchPlateCandidateCountTotal = 0;
	int32 HitSearchPlateCandidateCountMax = 0;
	int32 HitSearchPrunedSampleCount = 0;
	int64 RecoveryCandidatePlateCandidateCountTotal = 0;
	int32 RecoveryCandidatePlateCandidateCountMax = 0;
	int32 RecoveryCandidateGatherSampleCount = 0;
	int32 RecoveryCandidatePrunedSampleCount = 0;
	int64 RecoveryMissPlateCandidateCountTotal = 0;
	int32 RecoveryMissPlateCandidateCountMax = 0;
	int32 RecoveryMissSampleCount = 0;
	int32 DestructiveTriangleGeometryExcludedCount = 0;
	int32 DestructiveTriangleRejectedCount = 0;
	int32 TrackedDestructiveTriangleCount = 0;
	int32 TrackedSubductionTriangleCount = 0;
	int32 TrackedCollisionTriangleCount = 0;
	int32 TrackedDestructiveTriangleNewlySeededCount = 0;
	int32 TrackedDestructiveTrianglePropagatedCount = 0;
	int32 TrackedDestructiveTriangleExpiredCount = 0;
	int32 TrackedDestructiveTriangleClearedCount = 0;
	FTectonicPlanetV6DestructiveTrackingLifecycleStats TrackingLifecycleStats;
	int32 MissTrueDivergenceGapCount = 0;
	int32 MissDestructiveExclusionGapCount = 0;
	int32 MissAmbiguousGapCount = 0;
	int32 SamplesWithPartialDestructiveCandidateRemovalCount = 0;
	int32 FragmentedComponentsAdjacentToDestructiveExclusionGapCount = 0;
	int32 CategoricalMajorityTransferCount = 0;
	int32 CategoricalDominantTransferCount = 0;
	int32 DirectionalFallbackCount = 0;
	int32 ContinentalWeightThresholdMismatchCount = 0;
	int32 MaxComponentsBeforeCoherence = 0;
	int32 MaxComponentsPerPlate = 0;
	double ContinentalAreaFraction = 0.0;
	int32 QuietInteriorContinentalRetentionCount = 0;
	int32 QuietInteriorContinentalRetentionTriangleCount = 0;
	int32 QuietInteriorContinentalRetentionSingleSourceCount = 0;
	int32 ContinentalBreadthPreservationCount = 0;
	int32 ContinentalBreadthPreservationStrongInteriorCount = 0;
	int32 ContinentalBreadthPreservationModerateInteriorCount = 0;
	int32 PaperSurrogateOwnershipOverrideCount = 0;
	int32 PaperSurrogateOwnershipOverrideSubaerialCount = 0;
	int32 PaperSurrogateOwnershipOverrideSubmergedCount = 0;
	int32 PaperSurrogateOwnershipOverrideTriangleCount = 0;
	int32 PaperSurrogateOwnershipOverrideSingleSourceCount = 0;
	int32 PaperSurrogateOwnershipOverrideStrongInteriorCount = 0;
	int32 PaperSurrogateOwnershipOverrideModerateInteriorCount = 0;
	int32 PaperSurrogateOwnershipOverrideLowElevationBandCount = 0;
	int32 PaperSurrogateOwnershipOverrideModerateElevationBandCount = 0;
	int32 PaperSurrogateOwnershipOverrideHighElevationBandCount = 0;
	int32 PaperSurrogateQuietInteriorDirectHitEligibleCount = 0;
	double PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum = 0.0;
	double PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum = 0.0;
	double PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum = 0.0;
	double PaperSurrogateQuietInteriorDirectHitPreSolveElevationSum = 0.0;
	double PaperSurrogateQuietInteriorDirectHitPostTransferElevationSum = 0.0;
	double PaperSurrogateQuietInteriorDirectHitFinalElevationSum = 0.0;
	double PaperSurrogateQuietInteriorDirectHitPreSolveThicknessSum = 0.0;
	double PaperSurrogateQuietInteriorDirectHitPostTransferThicknessSum = 0.0;
	double PaperSurrogateQuietInteriorDirectHitFinalThicknessSum = 0.0;
	bool bDestructiveTriangleExclusionApplied = false;
	FTectonicPlanetV6PhaseTiming PhaseTiming;
	FTectonicPlanetV6TransferResolutionCounts TriangleTransferCountsByResolution;
	FTectonicPlanetV6TransferResolutionCounts SingleSourceTransferCountsByResolution;
	FTectonicPlanetV6TransferResolutionCounts ContinentalWeightThresholdMismatchCountsByResolution;
	FTectonicPlanetV6BoundaryOutcomeTransferStats BoundaryRetainedTransferStats;
	FTectonicPlanetV6BoundaryOutcomeTransferStats BoundaryReassignedTransferStats;
	FTectonicPlanetV6BoundaryOutcomeTransferStats BoundaryOceanicTransferStats;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierPlateMesh
{
	int32 PlateId = INDEX_NONE;
	TArray<FVector3d> BaseVertices;
	TArray<int32> LocalToCanonicalVertex;
	TMap<int32, int32> CanonicalToLocalVertex;
	TArray<FCarriedSample> LocalCarriedSamples;
	TArray<uint8> LocalVertexCopiedFrontierFlags;
	TArray<UE::Geometry::FIndex3i> LocalTriangles;
	TArray<int32> GlobalTriangleIndices;
	TMap<int32, int32> GlobalToLocalTriangle;
	TArray<uint8> LocalTriangleCopiedFrontierFlags;
	int32 CopiedFrontierVertexCount = 0;
	int32 CopiedFrontierTriangleCount = 0;
	int32 CopiedFrontierCarriedSampleCount = 0;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierAdjacentTriangleDiagnostic
{
	int32 GlobalTriangleIndex = INDEX_NONE;
	TArray<int32> PlateIdsWithCopies;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierMissPlateDiagnostic
{
	int32 PlateId = INDEX_NONE;
	bool bBoundingCapRejected = false;
	bool bHasAdjacentTriangleCopy = false;
	bool bRawRayHit = false;
	bool bRawRayHitIgnoringCap = false;
	bool bContainingTriangleHit = false;
	bool bContainingAdjacentTriangleHit = false;
	bool bNearVertex = false;
	bool bNearEdge = false;
	bool bNearDegenerate = false;
	int32 AdjacentTriangleCopyCount = 0;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierMissDiagnostic
{
	int32 SampleIndex = INDEX_NONE;
	FVector3d Position = FVector3d::ZeroVector;
	int32 PreviousPlateId = INDEX_NONE;
	bool bBoundarySample = false;
	bool bAtCopiedFrontierVertex = false;
	bool bCoverageGap = false;
	bool bRawRayMissRecoveredByContainingTriangle = false;
	bool bBoundingCapRejectedContainingTriangle = false;
	bool bNearVertex = false;
	bool bNearEdge = false;
	bool bNearDegenerate = false;
	int32 AdjacentGlobalTriangleCount = 0;
	int32 PlateMeshCountWithAdjacentTriangleCopies = 0;
	int32 BoundingCapRejectedPlateCount = 0;
	int32 RawRayHitPlateCount = 0;
	int32 ContainingTrianglePlateCount = 0;
	int32 AdjacentContainingTrianglePlateCount = 0;
	TArray<int32> AdjacentGlobalTriangleIndices;
	TArray<FTectonicPlanetV6CopiedFrontierAdjacentTriangleDiagnostic> AdjacentTriangleDiagnostics;
	TArray<FTectonicPlanetV6CopiedFrontierMissPlateDiagnostic> PlateDiagnostics;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierRayDiagnostics
{
	int32 SampleCount = 0;
	int32 RawRayMissCount = 0;
	int32 BoundaryRawRayMissCount = 0;
	int32 CoverageGapMissCount = 0;
	int32 BoundingCapRejectedRawRayHitMissCount = 0;
	int32 BoundingCapRejectedContainingTriangleMissCount = 0;
	int32 ContainingTriangleRecoveredMissCount = 0;
	int32 AdjacentContainingTriangleRecoveredMissCount = 0;
	int32 NearVertexMissCount = 0;
	int32 NearEdgeMissCount = 0;
	int32 NearDegenerateMissCount = 0;
	TArray<FTectonicPlanetV6CopiedFrontierMissDiagnostic> MissDiagnostics;
};

enum class ETectonicPlanetV6CopiedFrontierMotionClass : uint8
{
	None,
	Weak,
	Divergent,
	Convergent,
};

enum class ETectonicPlanetV6CopiedFrontierGapAttribution : uint8
{
	None,
	TrueDivergence,
	DestructiveExclusion,
	Ambiguous,
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierRepresentativeMiss
{
	int32 SampleIndex = INDEX_NONE;
	int32 PreviousPlateId = INDEX_NONE;
	int32 PrimaryNearestPlateId = INDEX_NONE;
	int32 SecondaryNearestPlateId = INDEX_NONE;
	double PrimaryNearestTriangleDistance = -1.0;
	double SecondaryNearestTriangleDistance = -1.0;
	double NearestCopiedFrontierTriangleDistance = -1.0;
	double RelativeNormalVelocity = 0.0;
	int32 UnfilteredCandidateCount = 0;
	int32 UnfilteredDestructiveCandidateCount = 0;
	bool bAdjacentToCopiedFrontierGeometry = false;
	bool bNearMultiHitRegion = false;
	ETectonicPlanetV6CopiedFrontierMotionClass MotionClass = ETectonicPlanetV6CopiedFrontierMotionClass::None;
	ETectonicPlanetV6CopiedFrontierGapAttribution GapAttribution = ETectonicPlanetV6CopiedFrontierGapAttribution::None;
};

struct AUROUS_API FTectonicPlanetV6PlateCount
{
	int32 PlateId = INDEX_NONE;
	int32 Count = 0;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierHitLossPatternCount
{
	FString Pattern;
	int32 Count = 0;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierRepresentativeHitLoss
{
	int32 SampleIndex = INDEX_NONE;
	int32 PreviousPlateId = INDEX_NONE;
	int32 FinalPlateId = INDEX_NONE;
	int32 DominantSourceCanonicalSampleIndex = INDEX_NONE;
	int32 DominantSourcePreviousPlateId = INDEX_NONE;
	int32 NoCapHitCount = 0;
	int32 PreviousPlateNoCapHitCount = 0;
	int32 OtherPlateNoCapHitCount = 0;
	int32 SourceTriangleUniquePreviousPlateCount = 0;
	int32 SourceTriangleContinentalCornerCount = 0;
	float DominantSourceContinentalWeight = 0.0f;
	float TransferredContinentalWeight = 0.0f;
	double WinningFitScore = -TNumericLimits<double>::Max();
	double NearestCopiedFrontierTriangleDistance = -1.0;
	bool bSingleHit = false;
	bool bSamePlateWinner = false;
	bool bCopiedFrontierWinningTriangle = false;
	bool bNearFrontier = false;
	bool bAdjacentToCopiedFrontierGeometry = false;
	bool bDominantSourceWasContinental = false;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierFragmentSizeBuckets
{
	int32 Size1 = 0;
	int32 Size2To4 = 0;
	int32 Size5To16 = 0;
	int32 Size17Plus = 0;
};

struct AUROUS_API FTectonicPlanetV6CopiedFrontierSolveAttribution
{
	int32 SolveStep = 0;
	int32 Interval = 0;
	int32 HitCount = 0;
	int32 MissCount = 0;
	int32 OceanicCreationCount = 0;
	int32 SingleHitWinnerCount = 0;
	int32 MultiHitWinnerCount = 0;
	int32 CopiedFrontierHitCount = 0;
	int32 InteriorHitCount = 0;
	int32 HitCwThresholdMismatchCount = 0;
	int32 CopiedFrontierHitCwThresholdMismatchCount = 0;
	int32 InteriorHitCwThresholdMismatchCount = 0;
	int32 MissAdjacentToCopiedFrontierGeometryCount = 0;
	int32 MissNearMultiHitRegionCount = 0;
	int32 MissDivergentMotionCount = 0;
	int32 MissConvergentMotionCount = 0;
	int32 MissWeakOrNoMotionCount = 0;
	int32 MissAdjacentFrontierAndDivergentCount = 0;
	int32 MissAdjacentFrontierAndWeakCount = 0;
	int32 MissAdjacentFrontierAndConvergentCount = 0;
	int32 MissAdjacentFrontierAndNearMultiHitCount = 0;
	int32 MissNearestCopiedFrontierDistanceLe100KmCount = 0;
	int32 MissNearestCopiedFrontierDistanceLe500KmCount = 0;
	int32 MissNearestCopiedFrontierDistanceLe1000KmCount = 0;
	int32 MissNearestCopiedFrontierDistanceLe4000KmCount = 0;
	int32 MissNearestCopiedFrontierDistanceGt4000KmCount = 0;
	int32 MissLikelyCoverageFailureCount = 0;
	int32 MissLikelyTrueDivergenceGapCount = 0;
	int32 MissTrueDivergenceGapCount = 0;
	int32 TrueDivergenceStructuredSyntheticCount = 0;
	int32 MissDestructiveExclusionGapCount = 0;
	int32 MissAmbiguousGapCount = 0;
	int32 MissFrontierPairStructuredSyntheticCount = 0;
	int32 MissFrontierPairVertexApproximationCount = 0;
	int32 DestructiveExclusionOverrideContinuityCount = 0;
	int32 DestructiveExclusionStructuredSyntheticCount = 0;
	int32 AmbiguousStructuredSyntheticCount = 0;
	int32 DestructiveExclusionFallbackOceanicCount = 0;
	int32 SamplesWithPartialDestructiveCandidateRemovalCount = 0;
	int32 ContinentalSamplesBefore = 0;
	int32 ContinentalSamplesAfter = 0;
	int32 ContinentalSamplesGained = 0;
	int32 ContinentalSamplesLost = 0;
	int32 ContinentalLossFromOceanicCreationCount = 0;
	int32 ContinentalLossFromHitCount = 0;
	int32 ContinentalLossFromHitCwThresholdMismatchCount = 0;
	int32 ContinentalLossFromHitMultiWinnerCount = 0;
	int32 ContinentalLossFromHitSingleWinnerCount = 0;
	int32 MultiHitWinnerSamePlateCount = 0;
	int32 MultiHitWinnerCrossPlateCount = 0;
	int32 MultiHitWinnerCopiedFrontierHitCount = 0;
	int32 MultiHitWinnerInteriorHitCount = 0;
	int32 MultiHitWinnerNearFrontierCount = 0;
	int32 MultiHitSamplesWithTrackedCandidateCount = 0;
	int32 MultiHitSamplesWithinTrackedTriangleCount = 0;
	int32 MultiHitSamplesWithinTrackedOneRingCount = 0;
	int32 MultiHitSamplesWithinTrackedTwoRingCount = 0;
	int32 MultiHitCandidateSamePlateOnlyCount = 0;
	int32 MultiHitCandidateCrossPlateOnlyCount = 0;
	int32 MultiHitCandidateMixedPlateCount = 0;
	int32 MultiHitCandidateCopiedFrontierOnlyCount = 0;
	int32 MultiHitCandidateInteriorOnlyCount = 0;
	int32 MultiHitCandidateMixedGeometryCount = 0;
	int32 MultiHitAdjacentToDestructiveGapCount = 0;
	int32 MultiHitAdjacentToContinuityHandledDestructiveGapCount = 0;
	int32 OverlapCoherenceSupportPrunedSampleCount = 0;
	int32 OverlapCoherencePreviousPlateStabilizedSampleCount = 0;
	int32 OverlapCoherenceSuppressedCandidateCount = 0;
	int32 TectonicMaintenanceAppliedCount = 0;
	int32 TectonicMaintenanceContinentalRecoveredCount = 0;
	int32 TectonicMaintenanceContinentalGainCount = 0;
	int32 TectonicMaintenanceSamePlateRecoveredCount = 0;
	int32 TectonicMaintenanceCrossPlateRecoveredCount = 0;
	int32 TectonicMaintenanceAndeanTaggedCount = 0;
	int32 TectonicMaintenanceElevationBoostCount = 0;
	int32 TectonicMaintenanceThicknessBoostCount = 0;
	int32 ActiveBandTriangleFieldContinuityClampCount = 0;
	int32 ActiveBandSyntheticFieldPreserveCount = 0;
	int32 ActiveBandOceanicFieldPreserveCount = 0;
	int32 ActiveBandPreviousOwnerCompatibleRecoveryCount = 0;
	int32 ActiveBandSyntheticLoopBreakCount = 0;
	int32 ActiveBandSyntheticSingleSourceRecoveryCount = 0;
	int32 SingleHitContinentalLossSamePlateCount = 0;
	int32 SingleHitContinentalLossCrossPlateCount = 0;
	int32 SingleHitContinentalLossCopiedFrontierHitCount = 0;
	int32 SingleHitContinentalLossInteriorHitCount = 0;
	int32 SingleHitContinentalLossNearFrontierCount = 0;
	int32 SingleHitContinentalLossInteriorRegionCount = 0;
	int32 SingleHitContinentalLossDominantSourceContinentalCount = 0;
	int32 SingleHitContinentalLossDominantSourceSubcontinentalCount = 0;
	int32 SingleHitContinentalLossAlternativeNoCapHitCount = 0;
	int32 SingleHitContinentalLossPreviousPlateNoCapHitCount = 0;
	int32 SingleHitContinentalLossOtherPlateNoCapHitCount = 0;
	int32 SingleHitContinentalLossSourceTriangleMixedPreviousPlateCount = 0;
	int32 SingleHitContinentalLossSourceTriangleMixedContinentalCount = 0;
	int32 MultiHitContinentalLossSamePlateCount = 0;
	int32 MultiHitContinentalLossCrossPlateCount = 0;
	int32 MultiHitContinentalLossCopiedFrontierHitCount = 0;
	int32 MultiHitContinentalLossInteriorHitCount = 0;
	int32 MultiHitContinentalLossNearFrontierCount = 0;
	int32 FragmentedComponentCount = 0;
	int32 FragmentedComponentsTouchingOceanicCreationCount = 0;
	int32 FragmentedComponentsDominatedByOceanicCreationCount = 0;
	int32 FragmentedComponentsTouchingCopiedFrontierHitCount = 0;
	int32 FragmentedComponentsDominatedByCopiedFrontierHitCount = 0;
	int32 FragmentedComponentsTouchingMultiHitWinnerCount = 0;
	int32 FragmentedComponentsStronglyAssociatedWithMultiHitWinnerCount = 0;
	int32 FragmentedComponentsMajorityCopiedFrontierHitWithCwMismatchCount = 0;
	int32 FragmentedComponentsAdjacentToDestructiveExclusionGapCount = 0;
	FTectonicPlanetV6CopiedFrontierFragmentSizeBuckets FragmentSizeBuckets;
	TArray<FTectonicPlanetV6PlateCount> SingleHitLossCountsByPreviousPlate;
	TArray<FTectonicPlanetV6PlateCount> SingleHitLossCountsByFinalPlate;
	TArray<FTectonicPlanetV6CopiedFrontierHitLossPatternCount> TopSingleHitLossPatterns;
	TArray<FTectonicPlanetV6CopiedFrontierHitLossPatternCount> TopMultiHitPatterns;
	TArray<FTectonicPlanetV6CopiedFrontierRepresentativeMiss> RepresentativeMisses;
	TArray<FTectonicPlanetV6CopiedFrontierRepresentativeHitLoss> RepresentativeSingleHitLosses;
};

struct AUROUS_API FTectonicPlanetV6PlateSubmesh
{
	int32 PlateId = INDEX_NONE;
	TArray<FVector3d> BaseVertices;
	TArray<int32> LocalToCanonicalVertex;
	TMap<int32, int32> CanonicalToLocalVertex;
	TArray<FCarriedSample> LocalCarriedSamples;
	TArray<uint8> LocalVertexFrontierFlags;
	TArray<UE::Geometry::FIndex3i> LocalTriangles;
	TArray<uint8> LocalTriangleFrontierFlags;
	int32 FrontierVertexCount = 0;
	int32 FrontierTriangleCount = 0;
	int32 FrontierCarriedSampleCount = 0;
	int32 RetriangulatedTriangleCount = 0;
	int32 ComponentCount = 0;
	int32 WholeMixedTriangleDuplicationCount = 0;
};

struct AUROUS_API FTectonicPlanetV6BoundaryCoherenceDiagnostic
{
	// Boundary band: samples with >=1 neighbor on a different plate (from current assignments)
	int32 BoundaryBandSampleCount = 0;

	// 1. Boundary continuity — connected components within the boundary band
	int32 BoundaryBandComponentCount = 0;
	int32 LargestBoundaryBandComponentSize = 0;
	int32 SmallestBoundaryBandComponentSize = 0;
	int32 SingletonBoundaryBandComponentCount = 0;

	// 2. Boundary width/spread — ring-distance distribution of conflict samples
	//    "Conflict" = multi-hit OR miss OR destructive-exclusion from resolved samples
	int32 TotalConflictSamples = 0;
	int32 ConflictSamplesAtRing0 = 0;
	int32 ConflictSamplesAtRing1 = 0;
	int32 ConflictSamplesAtRing2Plus = 0;
	double MeanConflictRingDistance = 0.0;

	// 3. Interior leakage — conflict activity away from the boundary band
	int32 MultiHitOnBoundaryCount = 0;
	int32 MultiHitInteriorCount = 0;
	int32 MissOnBoundaryCount = 0;
	int32 MissInteriorCount = 0;
	int32 DestructiveExclusionOnBoundaryCount = 0;
	int32 DestructiveExclusionInteriorCount = 0;
	int32 TotalInteriorConflictCount = 0;
	double InteriorLeakageFraction = 0.0;

	// 4. Composite boundary coherence score [0, 1]; 1 = perfectly coherent
	double BoundaryCoherenceScore = 0.0;
};

struct AUROUS_API FTectonicPlanetV6PlatePairChurnFlow
{
	int32 FromPlateId = INDEX_NONE;
	int32 ToPlateId = INDEX_NONE;
	int32 Count = 0;
};

struct AUROUS_API FTectonicPlanetV6OwnershipChurnDiagnostic
{
	int32 TotalChurnCount = 0;
	double ChurnFraction = 0.0;
	int32 BoundaryBandChurnCount = 0;
	int32 InteriorChurnCount = 0;
	// Churn by ring distance from boundary band
	int32 ChurnAtRing0 = 0;
	int32 ChurnAtRing1 = 0;
	int32 ChurnAtRing2Plus = 0;
	double MeanChurnRingDistance = 0.0;
	// Top plate-pair flows (up to 3)
	FTectonicPlanetV6PlatePairChurnFlow TopFlows[3];
};

struct AUROUS_API FTectonicPlanetV6CompetitiveParticipationDiagnostic
{
	int32 PlatesWithAnyHit = 0;
	int32 TotalPlateCount = 0;
	int32 TotalHitSamples = 0;
	int32 LargestPlateHitCount = 0;
	int32 LargestPlateId = INDEX_NONE;
	double LargestPlateHitShare = 0.0;
	// Plates with >= 1% of total hits
	int32 PlatesAboveOnePercentThreshold = 0;
	// Plates with >= 5% of total hits
	int32 PlatesAboveFivePercentThreshold = 0;
};

struct AUROUS_API FTectonicPlanetV6MissLineageDiagnostic
{
	int32 MissesThisCycle = 0;
	int32 FirstTimeSynthetic = 0;
	int32 SecondTimeSynthetic = 0;
	int32 RepeatedSynthetic3Plus = 0;
	double MeanResynthesisDepth = 0.0;
	// Split by boundary vs interior
	int32 BoundaryMissFirstTime = 0;
	int32 BoundaryMissRepeat = 0;
	int32 InteriorMissFirstTime = 0;
	int32 InteriorMissRepeat = 0;
};

struct AUROUS_API FTectonicPlanetV6InteriorInstabilityDiagnostic
{
	// Diagnostic scope is ring >= 2 from the current boundary band.
	int32 InteriorRingThreshold = 2;

	// Interior miss classification.
	int32 InteriorMissCount = 0;
	int32 InteriorMissPreviousOwnerNoLocalCoverageApproxCount = 0;
	int32 InteriorMissOtherPlateValidHitNearbyApproxCount = 0;
	int32 InteriorMissPreviousSyntheticRepeatCount = 0;
	int32 InteriorMissPreviousNaturalCount = 0;
	int32 InteriorMissPreviousOwnerGeometryAbsentCount = 0;
	int32 InteriorMissPreviousOwnerGeometryNearbyRecoveryCount = 0;
	int32 InteriorMissPreviousOwnerGeometryRejectedCount = 0;
	int32 InteriorMissPreviousOwnerNeighborHitCount = 0;
	int32 InteriorMissSinglePlateParticipationCount = 0;
	int32 InteriorMissMultiPlateParticipationCount = 0;
	int32 InteriorMissAssignedToLargestParticipationPlateCount = 0;

	// Interior ownership churn classification.
	int32 InteriorChurnCount = 0;
	int32 InteriorChurnPreviousOwnerNowMissingCount = 0;
	int32 InteriorChurnValidOwnerToValidOwnerCount = 0;
	int32 InteriorChurnPreviousOwnerStillHadValidHitCount = 0;
	int32 InteriorChurnValidOwnerWithoutPreviousOwnerHitCount = 0;
	int32 InteriorChurnPreviousSyntheticCount = 0;
	int32 InteriorChurnCurrentSyntheticCount = 0;
	int32 InteriorChurnEitherSyntheticCount = 0;
	int32 InteriorChurnNearRepeatResynthesisCount = 0;
	int32 InteriorChurnPreviousOwnerNeighborHitCount = 0;
	int32 InteriorChurnSinglePlateParticipationCount = 0;
	int32 InteriorChurnMultiPlateParticipationCount = 0;
	int32 InteriorChurnAbsorbedByLargestParticipationPlateCount = 0;
	FTectonicPlanetV6PlatePairChurnFlow TopInteriorFlows[3];

	// Stability of samples that were synthetic in the previous remesh interval.
	int32 InteriorPreviousSyntheticSampleCount = 0;
	int32 InteriorPreviousSyntheticSurvivedUnchangedCount = 0;
	int32 InteriorPreviousSyntheticReassignedCount = 0;
	int32 InteriorPreviousSyntheticMissedAgainCount = 0;
};

struct AUROUS_API FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic
{
	static constexpr double VeryNearCoverageThresholdKm = 25.0;
	static constexpr double ModerateCoverageThresholdKm = 100.0;

	// All samples that were synthetic in the previous remesh interval.
	int32 PreviousSyntheticSampleCount = 0;
	int32 PreviousSyntheticInteriorSampleCount = 0;

	// Next-cycle outcome for those samples.
	int32 SameOwnerValidHitCount = 0;
	int32 DifferentOwnerValidHitCount = 0;
	int32 MissCount = 0;

	// Previous-owner query context at the next remesh.
	int32 PreviousOwnerExactHitCount = 0;
	int32 PreviousOwnerIgnoringBoundingCapHitCount = 0;
	int32 PreviousOwnerRecoveryCoverageCount = 0;
	int32 PreviousOwnerCanonicalVertexInMeshCount = 0;
	int32 PreviousOwnerAdjacentTriangleInMeshCount = 0;
	int32 PreviousOwnerVertexButNoAdjacentTriangleCount = 0;
	int32 PreviousOwnerAdjacentTriangleButNoExactHitCount = 0;
	int32 PreviousOwnerAdjacentTriangleButIgnoringCapHitCount = 0;
	int32 PreviousOwnerAdjacentTriangleVeryNearMissCount = 0;
	int32 PreviousOwnerAdjacentTriangleModerateMissCount = 0;
	int32 PreviousOwnerAdjacentTriangleTrueLocalHoleCount = 0;
	int32 PreviousOwnerAdjacentTriangleNoDistanceCount = 0;

	// Local support-shape context for previous-owner adjacent coverage.
	int32 PreviousOwnerAdjacentTriangleTotalSupportCount = 0;
	int32 PreviousOwnerAdjacentTriangleTotalMixedSupportCount = 0;
	int32 PreviousOwnerAdjacentTriangleTotalMinoritySupportCount = 0;
	int32 PreviousOwnerAdjacentTriangleAnyMixedSupportSampleCount = 0;
	int32 PreviousOwnerAdjacentTriangleAllMixedSupportSampleCount = 0;
	int32 PreviousOwnerAdjacentTriangleAllMinoritySupportSampleCount = 0;
	int32 PreviousOwnerAdjacentTriangleNoExactHitAnyMixedSupportSampleCount = 0;
	int32 PreviousOwnerAdjacentTriangleNoExactHitAllMixedSupportSampleCount = 0;
	int32 PreviousOwnerAdjacentTriangleNoExactHitAllMinoritySupportSampleCount = 0;
	int32 PreviousOwnerAdjacentTriangleVeryNearMissAllMinoritySupportCount = 0;
	int32 PreviousOwnerAdjacentTriangleTrueLocalHoleAllMinoritySupportCount = 0;

	// Miss-side classification for previously synthetic samples.
	int32 MissWithPreviousOwnerGeometryNearbyCount = 0;
	int32 MissWithNoPreviousOwnerGeometryNearbyCount = 0;
	int32 MissWithPreviousOwnerAdjacentTrianglePresentCount = 0;
	int32 MissWithPreviousOwnerAdjacentTriangleAbsentCount = 0;

	// Valid-owner reassignment classification for previously synthetic samples.
	int32 DifferentOwnerValidHitWhilePreviousOwnerStillHadExactHitCount = 0;
	int32 DifferentOwnerValidHitWhilePreviousOwnerOnlyHadRecoveryCount = 0;
	int32 DifferentOwnerValidHitAbsorbedByLargestParticipationPlateCount = 0;

	// Retention-layer attribution for the pragmatic synthetic-gap experiment.
	int32 CurrentRetainedSyntheticCoverageCount = 0;
	int32 PreviousRetainedSyntheticCoverageSampleCount = 0;
	int32 PreviousRetainedSyntheticCoverageStabilizedSameOwnerCount = 0;
	int32 PreviousRetainedSyntheticCoverageMissCount = 0;
	int32 PreviousRetainedSyntheticCoverageReassignedCount = 0;
};

struct AUROUS_API FTectonicPlanetV6FrontRetreatDiagnostic
{
	// Mean SubductionDistanceKm across all samples with SubductionDistanceKm >= 0
	double CurrentMeanSubductionDistanceKm = 0.0;
	double PreviousMeanSubductionDistanceKm = 0.0;
	double DeltaMeanSubductionDistanceKm = 0.0;
	int32 CurrentSubductionSampleCount = 0;
	int32 PreviousSubductionSampleCount = 0;
	// Per-plate: tracked triangle count as proxy for front extent
	int32 TrackedTriangleCount = 0;
	int32 PreviousTrackedTriangleCount = 0;
	int32 DeltaTrackedTriangleCount = 0;
};

struct AUROUS_API FTectonicPlanetV6OwnerCoverageAudit
{
	int32 AssignedSampleCount = 0;
	int32 AssignedInteriorSampleCount = 0;
	int32 OwnerQueryExactHitCount = 0;
	int32 OwnerQueryIgnoringCapHitCount = 0;
	int32 OwnerQueryMissCount = 0;
	int32 OwnerQueryMissIgnoringCapCount = 0;
	int32 OwnerSoupContainmentCount = 0;
	int32 OwnerSoupMissCount = 0;
	int32 OwnerSoupContainmentButQueryMissCount = 0;
	int32 OwnerSoupContainmentButQueryMissIgnoringCapCount = 0;
	TArray<int32> ExampleInteriorQueryMissSampleIndices;
	TArray<int32> ExampleInteriorSoupContainedButQueryMissSampleIndices;
};

struct AUROUS_API FTectonicPlanetV6ActiveZoneDiagnostic
{
	int32 ActiveSampleCount = 0;
	int32 InactiveSampleCount = 0;
	int32 ActiveSeedSampleCount = 0;
	int32 ActiveCarryoverSampleCount = 0;
	int32 ActivePairCount = 0;
	int32 FreshSeedActivePairCount = 0;
	int32 PersistentCarryoverActivePairCount = 0;
	int32 FreshPairCandidateCount = 0;
	int32 FreshPairAdmittedCount = 0;
	int32 FreshPairRejectedSupportCount = 0;
	int32 FreshPairRejectedVelocityCount = 0;
	int32 FreshPairRejectedDominanceCount = 0;
	int32 FreshPairAdmittedDivergenceCount = 0;
	int32 FreshPairAdmittedConvergentSubductionCount = 0;
	int32 FreshPairAdmittedCollisionContactCount = 0;
	int32 FreshPairAdmittedRiftCount = 0;
	int32 ActiveBoundaryBandCount = 0;
	int32 ActiveInteriorCount = 0;
	int32 ActiveDivergenceSampleCount = 0;
	int32 ActiveConvergentSubductionSampleCount = 0;
	int32 ActiveCollisionContactSampleCount = 0;
	int32 ActiveRiftSampleCount = 0;
	int32 ActiveGenericQueryCompetitionSampleCount = 0;
	int32 ActiveUnknownOtherSampleCount = 0;
	int32 FreshExpandedSampleCount = 0;
	int32 CarryoverExpandedSampleCount = 0;
	double ActiveFraction = 0.0;
	int32 OwnershipChangesInsideActiveZone = 0;
	int32 OwnershipChangesOutsideActiveZone = 0;
	int32 OutsideZoneQueryMissCount = 0;
	int32 OutsideZoneCoverageDeficitCount = 0;
	int32 OwnershipChangeDivergenceFillCount = 0;
	int32 OwnershipChangeConvergentSubductionCount = 0;
	int32 OwnershipChangeCollisionContactCount = 0;
	int32 OwnershipChangeRiftCount = 0;
	int32 OwnershipChangeGenericQueryCompetitionCount = 0;
	int32 OwnershipChangeUnknownOtherCount = 0;
};

struct AUROUS_API FTectonicPlanetV6CollisionShadowTopPair
{
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	int32 ObservationCount = 0;
	double AccumulatedPenetrationKm = 0.0;
	double MeanConvergenceKmPerMy = 0.0;
	double MaxConvergenceKmPerMy = 0.0;
	int32 SupportSampleCount = 0;
	int32 SupportTriangleCount = 0;
	int32 ContinentalSupportPlateACount = 0;
	int32 ContinentalSupportPlateBCount = 0;
	int32 ContinentalQualifiedSampleCount = 0;
	int32 SubductionSampleCount = 0;
	int32 CollisionSampleCount = 0;
	bool bQualified = false;
};

struct AUROUS_API FTectonicPlanetV6CollisionShadowDiagnostic
{
	int32 TrackedConvergentPairCount = 0;
	int32 TrackedConvergentRegionCount = 0;
	int32 TrackedSubductionSampleCount = 0;
	int32 TrackedSubductionTriangleCount = 0;
	int32 TrackedCollisionSampleCount = 0;
	int32 TrackedCollisionTriangleCount = 0;
	int32 CollisionShadowCandidateCount = 0;
	int32 CollisionShadowQualifiedCount = 0;
	int32 PersistentObservedPairCount = 0;
	int32 ContinentalQualifiedCandidateCount = 0;
	int32 CandidateRejectedBySupportCount = 0;
	int32 CandidateRejectedByThresholdCount = 0;
	int32 CandidateRejectedByContinentalityCount = 0;
	int32 CandidateRejectedByPersistenceCount = 0;
	int32 BestCandidatePlateA = INDEX_NONE;
	int32 BestCandidatePlateB = INDEX_NONE;
	int32 BestCandidateObservationCount = 0;
	double BestCandidateAccumulatedPenetrationKm = 0.0;
	double BestCandidateMeanConvergenceKmPerMy = 0.0;
	double BestCandidateMaxConvergenceKmPerMy = 0.0;
	int32 BestCandidateSupportSampleCount = 0;
	int32 BestCandidateSupportTriangleCount = 0;
	int32 BestCandidateContinentalQualifiedSampleCount = 0;
	TArray<FTectonicPlanetV6CollisionShadowTopPair> TopPairs;
};

struct AUROUS_API FTectonicPlanetV6CollisionExecutionDiagnostic
{
	int32 ExecutedCollisionCount = 0;
	int32 CumulativeExecutedCollisionCount = 0;
	int32 CumulativeCollisionAffectedSampleCount = 0;
	int32 CumulativeCollisionAffectedSampleVisits = 0;
	int32 CumulativeCollisionDrivenContinentalGainCount = 0;
	int32 CumulativeCollisionDrivenOwnershipChangeCount = 0;
	int32 CumulativeCollisionTransferredSampleCount = 0;
	int32 CumulativeCollisionTransferredSampleVisits = 0;
	int32 CumulativeCollisionTransferredContinentalSampleCount = 0;
	int32 ExecutedPlateA = INDEX_NONE;
	int32 ExecutedPlateB = INDEX_NONE;
	int32 ExecutedOverridingPlateId = INDEX_NONE;
	int32 ExecutedSubductingPlateId = INDEX_NONE;
	int32 ExecutedObservationCount = 0;
	double ExecutedAccumulatedPenetrationKm = 0.0;
	double ExecutedMeanConvergenceKmPerMy = 0.0;
	double ExecutedMaxConvergenceKmPerMy = 0.0;
	int32 ExecutedSupportSampleCount = 0;
	int32 ExecutedSupportTriangleCount = 0;
	int32 ExecutedContinentalSupportPlateACount = 0;
	int32 ExecutedContinentalSupportPlateBCount = 0;
	int32 ExecutedContinentalQualifiedSampleCount = 0;
	int32 ExecutedSeedSampleCount = 0;
	int32 ExecutedCollisionSeedSampleCount = 0;
	int32 CollisionAffectedSampleCount = 0;
	int32 CollisionDrivenContinentalGainCount = 0;
	int32 CollisionDrivenOwnershipChangeCount = 0;
	int32 CollisionTransferredSampleCount = 0;
	int32 CollisionTransferredContinentalSampleCount = 0;
	int32 CooldownSuppressedQualifiedCount = 0;
	int32 PlateConflictSuppressedQualifiedCount = 0;
	int32 OverlapSuppressedQualifiedCount = 0;
	int32 QualifiedButUnexecutedCount = 0;
	int32 ExecutedEffectiveMassSampleCount = 0;
	int32 ExecutedTransferRejectedByLocalityCount = 0;
	int32 ExecutedTransferRejectedByContinentalityCount = 0;
	int32 ExecutedTransferRejectedByCapCount = 0;
	int32 ExecutedTransferBoundaryLocalSampleCount = 0;
	int32 ExecutedTransferCandidateSupportCount = 0;
	int32 ExecutedTransferAnchorSeedCount = 0;
	double ExecutedInfluenceRadiusRad = 0.0;
	double ExecutedTransferInfluenceRadiusRad = 0.0;
	double ExecutedMeanElevationDeltaKm = 0.0;
	double ExecutedMaxElevationDeltaKm = 0.0;
	double ExecutedStrengthScale = 1.0;
	double CumulativeMeanElevationDeltaKm = 0.0;
	double CumulativeMaxElevationDeltaKm = 0.0;
	double ExecutedDonorPlateShareBefore = 0.0;
	double ExecutedDonorPlateShareAfter = 0.0;
	double ExecutedDonorPlateShareDelta = 0.0;
	double ExecutedRecipientPlateShareBefore = 0.0;
	double ExecutedRecipientPlateShareAfter = 0.0;
	double ExecutedRecipientPlateShareDelta = 0.0;
	bool bExecutedFromShadowQualifiedState = false;

	// Thesis-shaped terrane quality diagnostics
	int32 ExecutedDonorTerraneComponentSize = 0;
	int32 ExecutedTransferredComponentSize = 0;
	int32 ExecutedTransferRejectedByConnectivityCount = 0;
	double ExecutedTerraneAreaSr = 0.0;
	double ExecutedXi = 0.0;
	double ExecutedRelativeSpeedKmPerMy = 0.0;
	FVector3d ExecutedSutureAxis = FVector3d::ZeroVector;
	double ExecutedSutureHalfWidthRad = 0.0;
	double ExecutedSutureBeltLengthEstimateRad = 0.0;
	double ExecutedMeanAbsAlongSutureRad = 0.0;
	double ExecutedMeanAbsPerpendicularRad = 0.0;
	int32 ExecutedRidgeCoreAffectedSampleCount = 0;
	int32 ExecutedRidgeFlankAffectedSampleCount = 0;
};

struct AUROUS_API FTectonicPlanetV6RiftDiagnostic
{
	bool bTriggeredThisSolve = false;
	bool bAutomatic = false;
	bool bForcedByTest = false;
	bool bOwnershipAppliedDirectlyByEvent = false;
	bool bCopiedFrontierRebuiltBeforeSolve = false;
	bool bPlateSubmeshRebuiltBeforeSolve = false;
	bool bPostRiftSolveRan = false;
	bool bChildBoundaryClassifiedDivergent = false;
	bool bParentPlateStillPresent = false;
	bool bChildPlateAAlive = false;
	bool bChildPlateBAlive = false;
	int32 Step = 0;
	int32 CumulativeRiftCount = 0;
	int32 ParentPlateId = INDEX_NONE;
	int32 ChildPlateA = INDEX_NONE;
	int32 ChildPlateB = INDEX_NONE;
	int32 ParentSampleCount = 0;
	int32 ParentContinentalSampleCount = 0;
	int32 ChildSampleCountA = 0;
	int32 ChildSampleCountB = 0;
	int32 PostRiftPlateCount = 0;
	int32 ChildBoundaryContactEdgeCount = 0;
	int32 ChildBoundaryDivergentEdgeCount = 0;
	int32 ChildBoundaryConvergentEdgeCount = 0;
	int32 ChildBoundaryRiftActiveSampleCount = 0;
	int32 ChildBoundaryDivergenceActiveSampleCount = 0;
	int32 CurrentChildSampleCountA = 0;
	int32 CurrentChildSampleCountB = 0;
	double ParentContinentalFraction = 0.0;
	double TriggerProbability = 0.0;
	double TriggerDraw = 0.0;
	double RiftMilliseconds = 0.0;
	double ChildBoundaryMeanRelativeNormalVelocityKmPerMy = 0.0;
	double ChildBoundaryMaxAbsRelativeNormalVelocityKmPerMy = 0.0;
	TArray<int32> ChildPlateIds;
	TArray<int32> ChildSampleCounts;
};

struct AUROUS_API FTectonicPlanetV6PlatePairBoundaryMotionDiagnostic
{
	bool bPairIsDivergent = false;
	int32 ContactEdgeCount = 0;
	int32 DivergentEdgeCount = 0;
	int32 ConvergentEdgeCount = 0;
	double MeanRelativeNormalVelocityKmPerMy = 0.0;
	double MaxAbsRelativeNormalVelocityKmPerMy = 0.0;
};

struct AUROUS_API FTectonicPlanetV6
{
	void Initialize(
		int32 InSampleCount,
		int32 InPlateCount,
		int32 InRandomSeed,
		float InBoundaryWarpAmplitude,
		float InContinentalFraction,
		double InPlanetRadiusKm = 6371.0);

	void AdvanceStep();
	void AdvanceSteps(int32 StepCount);
	void PerformAuthoritativePeriodicSolve(
		ETectonicPlanetV6SolveTrigger Trigger = ETectonicPlanetV6SolveTrigger::Periodic);

	int32 ComputePeriodicSolveInterval() const;
	int32 GetPeriodicSolveCount() const;
	bool IsPhase1AuthoritativePeriodicPath() const;
	ETectonicPlanetV6PeriodicSolveMode GetPeriodicSolveMode() const;
	FTectonicPlanetV6PeriodicSolveStats BuildCurrentDiagnosticSnapshotForTest() const;
	FTectonicPlanetV6CopiedFrontierRayDiagnostics BuildCopiedFrontierRayDiagnosticsForTest(
		int32 MaxMissSamples = 8) const;
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& GetLastCopiedFrontierSolveAttributionForTest() const;
	const TArray<uint8>& GetTrackedCopiedFrontierDestructiveKindsForTest() const;
	void RefreshCopiedFrontierDestructiveTrackingForTest();
	void SetCopiedFrontierIntervalPropagationWaveCapForTest(int32 InMaxWaves);
	FTectonicPlanetV6BoundaryCoherenceDiagnostic ComputeBoundaryCoherenceDiagnosticForTest() const;
	FTectonicPlanetV6OwnershipChurnDiagnostic ComputeOwnershipChurnDiagnosticForTest() const;
	FTectonicPlanetV6CompetitiveParticipationDiagnostic ComputeCompetitiveParticipationDiagnosticForTest() const;
	FTectonicPlanetV6MissLineageDiagnostic ComputeMissLineageDiagnosticForTest() const;
	FTectonicPlanetV6InteriorInstabilityDiagnostic ComputeInteriorInstabilityDiagnosticForTest() const;
	FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic ComputeSyntheticCoveragePersistenceDiagnosticForTest() const;
	FTectonicPlanetV6FrontRetreatDiagnostic ComputeFrontRetreatDiagnosticForTest() const;
	FTectonicPlanetV6OwnerCoverageAudit ComputeCurrentOwnerCoverageAuditForTest() const;
	FTectonicPlanetV6ActiveZoneDiagnostic ComputeActiveZoneDiagnosticForTest() const;
	FTectonicPlanetV6CollisionShadowDiagnostic ComputeCollisionShadowDiagnosticForTest() const;
	FTectonicPlanetV6CollisionExecutionDiagnostic ComputeCollisionExecutionDiagnosticForTest() const;
	FTectonicPlanetV6RiftDiagnostic ComputeRiftDiagnosticForTest() const;
	FTectonicPlanetV6PlatePairBoundaryMotionDiagnostic ComputePlatePairBoundaryMotionDiagnosticForTest(
		int32 PlateA,
		int32 PlateB) const;
	const TArray<FTectonicPlanetV6ResolvedSample>& GetLastResolvedSamplesForTest() const { return LastResolvedSamples; }
	const TArray<uint8>& GetMissLineageCountsForTest() const { return MissLineageCounts; }
	const TArray<uint8>& GetCollisionShadowPersistenceMaskForTest() const
	{
		return CurrentSolveCollisionShadowPersistenceMask;
	}
	const TArray<uint8>& GetCollisionExecutionMaskForTest() const
	{
		return CurrentSolveCollisionExecutionMask;
	}
	const TArray<uint8>& GetCollisionTransferMaskForTest() const
	{
		return CurrentSolveCollisionTransferMask;
	}
	const TArray<uint8>& GetCollisionCumulativeTransferMaskForTest() const
	{
		return CumulativeCollisionTransferMask;
	}
	const TArray<uint8>& GetCollisionCumulativeExecutionMaskForTest() const
	{
		return CumulativeCollisionExecutionMask;
	}
	const TArray<float>& GetCollisionCumulativeElevationDeltaMaskForTest() const
	{
		return CumulativeCollisionElevationDeltaMaskKm;
	}
	const TArray<float>& GetCollisionCumulativeContinentalGainMaskForTest() const
	{
		return CumulativeCollisionContinentalGainMask;
	}
	const TArray<int32>& GetCurrentSolvePreSolvePlateIdsForTest() const
	{
		return CurrentSolvePreSolvePlateIds;
	}
	const TArray<float>& GetCurrentSolvePreSolveContinentalWeightsForTest() const
	{
		return CurrentSolvePreSolveContinentalWeights;
	}
	const TArray<float>& GetCurrentSolvePreSolveElevationsForTest() const
	{
		return CurrentSolvePreSolveElevations;
	}
	const TArray<uint8>& GetCurrentSolvePreviousSyntheticFlagsForTest() const
	{
		return CurrentSolvePreviousSyntheticFlags;
	}
	const TArray<uint8>& GetCurrentSolvePreviousTransferSourceKindValuesForTest() const
	{
		return CurrentSolvePreviousTransferSourceKindValues;
	}
	const TArray<uint8>& GetCurrentSolvePreviousResolutionKindValuesForTest() const
	{
		return CurrentSolvePreviousResolutionKindValues;
	}
	const TArray<uint8>& GetCurrentSolvePreviousOwnerFilteredHitFlagsForTest() const
	{
		return CurrentSolvePreviousOwnerFilteredHitFlags;
	}
	const TArray<uint8>& GetCurrentSolvePreviousOwnerUnfilteredHitFlagsForTest() const
	{
		return CurrentSolvePreviousOwnerUnfilteredHitFlags;
	}
	const TArray<uint8>& GetCurrentSolvePreviousOwnerRecoveryFlagsForTest() const
	{
		return CurrentSolvePreviousOwnerRecoveryFlags;
	}
	const TArray<uint8>& GetCurrentSolveActiveZoneFlagsForTest() const
	{
		return CurrentSolveActiveZoneFlags;
	}
	const TArray<uint8>& GetCurrentSolveActiveZoneCauseValuesForTest() const
	{
		return CurrentSolveActiveZoneCauseValues;
	}
	const TArray<uint8>& GetThesisCollisionTerraneComponentMaskForTest() const
	{
		return CurrentSolveThesisCollisionTerraneComponentMask;
	}

	const FTectonicPlanet& GetPlanet() const;
	FTectonicPlanet& GetPlanetMutable();
	const FTectonicPlanetV6PeriodicSolveStats& GetLastSolveStats() const;
	const TArray<FTectonicPlanetV6ResolvedSample>& GetLastResolvedSamples() const;
	const TArray<int32>& GetPeriodicSolveSteps() const;
	void SetPeriodicSolveModeForTest(
		ETectonicPlanetV6PeriodicSolveMode InMode,
		int32 InFixedIntervalSteps = 25);
	void SetSyntheticCoverageRetentionForTest(bool bEnable);
	void SetWholeTriangleBoundaryDuplicationForTest(bool bEnable);
	void SetExcludeMixedTrianglesForTest(bool bEnable);
	void SetPerTimestepContainmentSoupRebuildForTest(bool bEnable);
	void SetV9Phase1AuthorityForTest(bool bEnable, int32 InActiveBoundaryRingCount = 1);
	void SetV9Phase1ActiveZoneClassifierModeForTest(ETectonicPlanetV6ActiveZoneClassifierMode InMode);
	void SetV9Phase1PersistentActivePairHorizonForTest(int32 InPersistenceHorizon);
	void SetV9CollisionShadowForTest(bool bEnable);
	void SetV9CollisionExecutionForTest(bool bEnable);
	void SetV9CollisionExecutionEnhancedConsequencesForTest(bool bEnable);
	void SetV9CollisionExecutionStructuralTransferForTest(bool bEnable);
	void SetV9CollisionExecutionRefinedStructuralTransferForTest(bool bEnable);
	void SetV9ThesisShapedCollisionExecutionForTest(bool bEnable);
	void SetV9ThesisShapedCollisionRidgeSurgeForTest(bool bEnable);
	void SetV9QuietInteriorContinentalRetentionForTest(bool bEnable);
	void SetV9ContinentalBreadthPreservationForTest(bool bEnable);
	void SetV9PaperSurrogateOwnershipForTest(bool bEnable);
	void SetV9PaperSurrogateFieldModeForTest(ETectonicPlanetV6PaperSurrogateFieldMode InMode);
	void SetSubmergedContinentalRelaxationForTest(bool bEnable, double RatePerStep = 0.005);
	void SetAutomaticRiftingForTest(bool bEnable);
	void SetPhaseTimingForTest(bool bEnable);
	void SetDetailedCopiedFrontierAttributionForTest(bool bEnable);
	void SetPlateCandidatePruningForTest(bool bEnable);
	bool ForceLargestEligibleAutomaticRiftForTest(int32 ChildCount = 2, int32 Seed = 0);
	void SetUseLinearConvergentMaintenanceSpeedFactorForTest(bool bEnableLinear);
	void SetUseLinearConvergentMaintenanceInfluenceForTest(bool bEnableLinear);

	static FTectonicPlanetV6ResolvedSample ResolvePhase1OwnershipForTest(
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		int32 NearestMemberFallbackPlateId = INDEX_NONE,
		int32 NearestMemberFallbackSampleIndex = INDEX_NONE,
		int32 ExplicitFallbackPlateId = INDEX_NONE,
		int32 ExplicitFallbackSampleIndex = INDEX_NONE);

	static FTectonicPlanetV6ResolvedSample ResolveBoundaryStateForTest(
		int32 PreviousPlateId,
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
		bool bPreviouslyContinental = false,
		const FVector3d& QueryPoint = FVector3d(0.0, 0.0, 1.0));

	static void CollectBoundaryTrianglePlateIdsForTest(
		int32 VertexPlateIdA,
		int32 VertexPlateIdB,
		int32 VertexPlateIdC,
		TArray<int32>& OutPlateIds);

	static void ApplyCoherencePassForTest(
		const TArray<TArray<int32>>& SampleAdjacency,
		TArray<int32>& InOutPlateIds,
		int32 MaxComponentSize,
		int32& OutReassignedSampleCount,
		int32& OutRemovedComponentCount,
		int32& OutLargestRemovedComponentSize,
		int32& OutFinalMaxComponentsPerPlate);

	static void ApplyTriangleTransferForTest(
		FSample& InOutSample,
		const FCarriedSample& V0,
		const FCarriedSample& V1,
		const FCarriedSample& V2,
		const FVector3d& RawBarycentric,
		float& OutSubductionDistanceKm,
		float& OutSubductionSpeed,
		FTectonicPlanetV6TransferDebugInfo& OutTransferDebug);

private:
	bool HandlePendingAutomaticRiftAfterAdvance();
	void PerformThesisRemeshSpikeSolve(ETectonicPlanetV6SolveTrigger Trigger);
	void PerformThesisCopiedFrontierSpikeSolve(ETectonicPlanetV6SolveTrigger Trigger);
	void PerformThesisPlateSubmeshSpikeSolve(ETectonicPlanetV6SolveTrigger Trigger);
	void RebuildThesisCopiedFrontierMeshes();
	void RebuildThesisPlateSubmeshMeshes();

	FTectonicPlanet Planet;
	FTectonicPlanetV6PeriodicSolveStats LastSolveStats;
	TArray<FTectonicPlanetV6ResolvedSample> LastResolvedSamples;
	TArray<int32> PeriodicSolveSteps;
	int32 PeriodicSolveCount = 0;
	ETectonicPlanetV6PeriodicSolveMode PeriodicSolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative;
	int32 ThesisRemeshFixedIntervalSteps = 25;
	TArray<FTectonicPlanetV6CopiedFrontierPlateMesh> ThesisCopiedFrontierMeshes;
	TArray<FTectonicPlanetV6PlateSubmesh> ThesisPlateSubmeshMeshes;
	FTectonicPlanetV6CopiedFrontierSolveAttribution LastCopiedFrontierSolveAttribution;
	TArray<uint8> CopiedFrontierTrackedDestructiveKinds;
	TArray<int32> CopiedFrontierTrackedPreferredContinuationPlateIds;
	TArray<int32> CopiedFrontierTrackedDestructiveSourcePlateIds;
	TArray<float> CopiedFrontierTrackedDestructiveDistancesKm;
	TArray<uint8> CopiedFrontierTrackedDestructiveSeedOriginFlags;
	TArray<uint8> CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags;
	TArray<uint8> CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags;
	FTectonicPlanetV6DestructiveTrackingLifecycleStats CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats;
	int32 CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount = 0;
	int32 CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount = 0;
	int32 CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount = 0;
	int32 CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
	int32 CopiedFrontierIntervalPropagationWaveCapForTest = INDEX_NONE;

	// Persistent diagnostic state
	TArray<uint8> MissLineageCounts; // per-sample miss streak counter, capped at 255
	TArray<uint8> PreviousSolveSyntheticFlags;
	TArray<uint8> PreviousSolveRetainedSyntheticCoverageFlags;
	TArray<uint8> PreviousSolveTransferSourceKindValues;
	TArray<uint8> PreviousSolveResolutionKindValues;
	TArray<int32> CurrentSolvePreSolvePlateIds;
	TArray<float> CurrentSolvePreSolveContinentalWeights;
	TArray<float> CurrentSolvePreSolveElevations;
	TArray<uint8> CurrentSolvePreviousSyntheticFlags;
	TArray<uint8> CurrentSolvePreviousRetainedSyntheticCoverageFlags;
	TArray<uint8> CurrentSolvePreviousTransferSourceKindValues;
	TArray<uint8> CurrentSolvePreviousResolutionKindValues;
	TArray<uint8> CurrentSolveRetainedSyntheticCoverageFlags;
	TArray<uint8> CurrentSolvePreviousOwnerFilteredHitFlags;
	TArray<uint8> CurrentSolvePreviousOwnerUnfilteredHitFlags;
	TArray<uint8> CurrentSolvePreviousOwnerIgnoringBoundingCapHitFlags;
	TArray<uint8> CurrentSolvePreviousOwnerRecoveryFlags;
	TArray<uint8> CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags;
	TArray<uint8> CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags;
	TArray<float> CurrentSolvePreviousOwnerAdjacentTriangleNearestDistanceKm;
	TArray<uint16> CurrentSolvePreviousOwnerAdjacentTriangleSupportCounts;
	TArray<uint16> CurrentSolvePreviousOwnerAdjacentTriangleMixedSupportCounts;
	TArray<uint16> CurrentSolvePreviousOwnerAdjacentTriangleMinoritySupportCounts;
	TArray<uint8> CurrentSolveLocalParticipationPlateCounts;
	TArray<uint8> CurrentSolveActiveZoneFlags;
	TArray<uint8> CurrentSolveActiveZoneSeedFlags;
	TArray<uint8> CurrentSolveActiveZoneCarryoverFlags;
	TArray<uint8> CurrentSolveActiveZoneFreshExpandedFlags;
	TArray<uint8> CurrentSolveActiveZoneCarryoverExpandedFlags;
	TArray<uint8> CurrentSolveActiveZoneCauseValues;
	TArray<int32> CurrentSolveActiveZonePrimaryPlateIds;
	TArray<int32> CurrentSolveActiveZoneSecondaryPlateIds;
	TMap<uint64, int32> PersistentV9ActivePairRemainingSolveIntervals;
	TMap<uint64, uint8> PersistentV9ActivePairCauseValues;
	int32 CurrentSolveActivePairCount = 0;
	int32 CurrentSolveFreshSeedActivePairCount = 0;
	int32 CurrentSolvePersistentCarryoverActivePairCount = 0;
	int32 CurrentSolveFreshPairCandidateCount = 0;
	int32 CurrentSolveFreshPairAdmittedCount = 0;
	int32 CurrentSolveFreshPairRejectedSupportCount = 0;
	int32 CurrentSolveFreshPairRejectedVelocityCount = 0;
	int32 CurrentSolveFreshPairRejectedDominanceCount = 0;
	int32 CurrentSolveFreshPairAdmittedDivergenceCount = 0;
	int32 CurrentSolveFreshPairAdmittedConvergentSubductionCount = 0;
	int32 CurrentSolveFreshPairAdmittedCollisionContactCount = 0;
	int32 CurrentSolveFreshPairAdmittedRiftCount = 0;
	TArray<uint8> CurrentSolveOutsideActiveZoneQueryMissFlags;
	TArray<uint8> CurrentSolveOutsideActiveZoneCoverageDeficitFlags;
	TArray<uint8> CurrentSolveCollisionShadowTrackedFlags;
	TArray<uint8> CurrentSolveCollisionShadowQualifiedFlags;
	TArray<uint8> CurrentSolveCollisionShadowPersistenceMask;
	FTectonicPlanetV6CollisionShadowDiagnostic CurrentSolveCollisionShadowDiagnostic;
	TArray<uint8> CurrentSolveCollisionExecutionMask;
	TArray<uint8> CurrentSolveCollisionTransferMask;
	TArray<uint8> CumulativeCollisionExecutionMask;
	TArray<uint8> CumulativeCollisionTransferMask;
	TArray<float> CumulativeCollisionElevationDeltaMaskKm;
	TArray<float> CumulativeCollisionContinentalGainMask;
	FTectonicPlanetV6CollisionExecutionDiagnostic CurrentSolveCollisionExecutionDiagnostic;
	FTectonicPlanetV6RiftDiagnostic CurrentSolveRiftDiagnostic;
	TMap<uint64, FGeometricCollisionPairRecurrenceState> V9CollisionShadowPairRecurrenceByKey;
	TMap<uint64, int32> V9CollisionExecutionLastSolveIndexByKey;
	int32 V9CollisionExecutionCumulativeCount = 0;
	int32 V9CollisionExecutionCumulativeAffectedSampleVisits = 0;
	int32 V9CollisionExecutionCumulativeContinentalGainCount = 0;
	int32 V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	int32 V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	int32 V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
	double V9CollisionExecutionCumulativeElevationDeltaKm = 0.0;
	double V9CollisionExecutionCumulativeMaxElevationDeltaKm = 0.0;
	bool bEnableSyntheticCoverageRetentionForTest = false;
	bool bForceWholeTriangleBoundaryDuplicationForTest = false;
	bool bForceExcludeMixedTrianglesForTest = false;
	bool bForcePerTimestepContainmentSoupRebuildForTest = false;
	bool bEnableV9Phase1AuthorityForTest = false;
	bool bEnableV9CollisionShadowForTest = false;
	bool bEnableV9CollisionExecutionForTest = false;
	bool bEnableV9CollisionExecutionEnhancedConsequencesForTest = false;
	bool bEnableV9CollisionExecutionStructuralTransferForTest = false;
	bool bEnableV9CollisionExecutionRefinedStructuralTransferForTest = false;
	bool bEnableV9ThesisShapedCollisionExecutionForTest = false;
	bool bEnableV9ThesisShapedCollisionRidgeSurgeForTest = false;
	bool bEnableV9QuietInteriorContinentalRetentionForTest = false;
	bool bEnableV9ContinentalBreadthPreservationForTest = false;
	bool bEnableV9PaperSurrogateOwnershipForTest = false;
	ETectonicPlanetV6PaperSurrogateFieldMode V9PaperSurrogateFieldModeForTest =
		ETectonicPlanetV6PaperSurrogateFieldMode::FullState;
	bool bEnableAutomaticRiftingForTest = false;
	bool bEnablePhaseTimingForTest = false;
	bool bEnableDetailedCopiedFrontierAttributionForTest = true;
	bool bUsePlateCandidatePruningForTest = true;
	bool bUseLinearConvergentMaintenanceSpeedFactorForTest = true;
	bool bUseLinearConvergentMaintenanceInfluenceForTest = true;
	TArray<uint8> CurrentSolveThesisCollisionTerraneComponentMask;
	int32 V9Phase1ActiveBoundaryRingCountForTest = 1;
	int32 V9Phase1PersistentActivePairHorizonForTest = 2;
	ETectonicPlanetV6ActiveZoneClassifierMode V9Phase1ActiveZoneClassifierModeForTest =
		ETectonicPlanetV6ActiveZoneClassifierMode::BroadBoundaryBand;
	double PreviousIntervalMeanSubductionDistanceKm = -1.0;
	int32 PreviousIntervalSubductionSampleCount = 0;
	int32 PreviousIntervalTrackedTriangleCount = 0;
	int32 V9RiftCumulativeCount = 0;
};
