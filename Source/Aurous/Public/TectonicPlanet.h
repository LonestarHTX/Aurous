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
enum class EResamplingPolicy : uint8
{
	PeriodicFull,
	EventDrivenOnly,
	HybridStablePeriodic,
	PreserveOwnershipPeriodic,
};

// Internal ownership behavior used by a specific resample invocation.
// `PreserveOwnership` is the preferred maintenance mode.
enum class EResampleOwnershipMode : uint8
{
	FullResolution,
	StableOverlaps,
	PreserveOwnership,
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

struct FPendingRiftEvent
{
	bool bValid = false;
	bool bAutomatic = false;
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
	TArray<int32> ChildSampleCounts;
	TArray<int32> FormerParentSampleIndices;
	TArray<int32> FormerParentTerraneIds;
	double ParentContinentalFraction = 0.0;
	double TriggerProbability = 0.0;
	double RiftMs = 0.0;
};

struct FResamplingStats
{
	int32 Step = 0;
	int32 Interval = 0;
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
	TArray<int32> RiftChildPlateIds;
	TArray<int32> RiftChildSampleCounts;
	TArray<int32> RiftChildTerraneFragmentCounts;
	TArray<int32> RiftPreTerraneIds;
	TArray<int32> RiftPreTerraneTouchedChildCounts;
	bool bRiftWasAutomatic = false;
	int32 BoundaryContactCollisionCandidateCount = 0;
	int32 BoundaryContactCollisionPairCount = 0;
	int32 BoundaryContactLargestZoneSize = 0;
	int32 BoundaryContactTriggerPlateA = INDEX_NONE;
	int32 BoundaryContactTriggerPlateB = INDEX_NONE;
	int32 BoundaryContactPersistencePairA = INDEX_NONE;
	int32 BoundaryContactPersistencePairB = INDEX_NONE;
	int32 BoundaryContactPersistenceCount = 0;
	int32 CachedBoundaryContactSeedCount = 0;
	int32 CachedBoundaryContactTerraneSeedCount = 0;
	int32 CachedBoundaryContactTerraneRecoveredCount = 0;
	int32 CachedBoundaryContactPlateA = INDEX_NONE;
	int32 CachedBoundaryContactPlateB = INDEX_NONE;
	bool bBoundaryContactCollisionTriggered = false;
	bool bBoundaryContactPersistenceTriggered = false;
	bool bUsedCachedBoundaryContactCollision = false;
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

struct AUROUS_API FTectonicPlanet
{
	TArray<FSample> Samples;
	TArray<FPlate> Plates;
	TArray<FTerrane> Terranes;
	TArray<FIntVector> TriangleIndices;
	TArray<TArray<int32>> SampleAdjacency;
	TArray<TArray<int32>> TriangleAdjacency;
	double PlanetRadiusKm = 6371.0;
	double ContainmentRecoveryTolerance = 0.003;
	int32 SampleCountConfig = 0;
	int32 PlateCountConfig = 0;
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
	double AutomaticRiftBaseLambdaPerStep = 0.01;
	double AutomaticRiftParentSizeExponent = 1.0;
	double AutomaticRiftContinentalExponent = 1.0;
	double AutomaticRiftMinContinentalFraction = 0.10;
	bool bEnableSlabPull = true;
	bool bEnableAndeanContinentalConversion = true;
	bool bEnableOverlapHysteresis = false; // Legacy experiment path retained for tests.
	bool bEnableContinentalCollision = true;
	bool bEnableAutomaticRifting = true;
	bool bEnableWarpedRiftBoundaries = true;
	double RiftBoundaryWarpAmplitude = 0.18;
	double RiftBoundaryWarpFrequency = 1.5;
	EResamplingPolicy ResamplingPolicy = EResamplingPolicy::PeriodicFull;
	EResampleTriggerReason LastResampleTriggerReason = EResampleTriggerReason::None;
	EResampleOwnershipMode LastResampleOwnershipMode = EResampleOwnershipMode::FullResolution;
	bool bPendingFullResolutionResample = false;
	bool bPendingBoundaryContactPersistenceReset = false;
	FResamplingStats LastResamplingStats;
	TArray<int32> ResamplingSteps;
	// Preserve-mode collision triggering state. Cleared after follow-up execution.
	TMap<uint64, FBoundaryContactPersistence> BoundaryContactPersistenceByPair;
	FPendingBoundaryContactCollisionEvent PendingBoundaryContactCollisionEvent;
	FPendingRiftEvent PendingRiftEvent;

	void Initialize(int32 InSampleCount, double InPlanetRadiusKm);
	void InitializePlates(int32 InPlateCount, int32 InRandomSeed, float InBoundaryWarpAmplitude, float InContinentalFraction);
	void AdvanceStep();
	void TriggerEventResampling(EResampleTriggerReason Reason);
	bool TriggerForcedRift(int32 ParentPlateId, int32 ChildCount, int32 Seed = 0);
	int32 FindPlateArrayIndexById(int32 PlateId) const;
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
		EResampleOwnershipMode OwnershipMode = EResampleOwnershipMode::FullResolution) const;
	void InterpolateFromCarried(
		const TArray<int32>& NewPlateIds,
		const TArray<int32>& ContainingTriangles,
		const TArray<FVector3d>& BarycentricCoords,
		TArray<float>& OutSubductionDistances,
		TArray<float>& OutSubductionSpeeds,
		int32* OutMissingLocalCarriedLookupCount = nullptr);
	void ResolveGaps(
		TArray<int32>& NewPlateIds,
		const TArray<uint8>& GapFlags,
		TArray<float>& InOutSubductionDistances,
		TArray<float>& InOutSubductionSpeeds,
		FResamplingStats* InOutStats = nullptr);
	void RepartitionMembership(
		const TArray<int32>& NewPlateIds,
		const TArray<float>* InSubductionDistances = nullptr,
		const TArray<float>* InSubductionSpeeds = nullptr);
	void CollectCollisionCandidates(
		const TArray<uint8>& OverlapFlags,
		const TArray<TArray<int32>>& OverlapPlateIds,
		TArray<FCollisionCandidate>& OutCandidates) const;
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
		FResamplingStats* InOutStats = nullptr) const;
	void ApplyCollisionElevationSurge(
		const FCollisionEvent& CollisionEvent,
		FResamplingStats* InOutStats = nullptr);
	void ComputeSubductionDistanceField(FResamplingStats* InOutStats = nullptr);
	void ComputeSlabPullCorrections(FResamplingStats* InOutStats = nullptr);
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
		int32 ParentContinentalSampleCount,
		double ParentContinentalFraction);
	bool IsPlateEligibleForAutomaticRift(
		const FPlate& Plate,
		int32 ChildCount,
		int32& OutContinentalSampleCount,
		double& OutContinentalFraction) const;
	double ComputeAutomaticRiftProbability(const FPlate& Plate, double ContinentalFraction) const;
};
