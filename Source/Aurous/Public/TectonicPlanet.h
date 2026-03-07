#pragma once

#include "CoreMinimal.h"
#include "TectonicPlanet.generated.h"

UENUM()
enum class ECrustType : uint8
{
	Oceanic,
	Continental
};

UENUM()
enum class EOrogenyType : uint8
{
	None,
	Andean,
	Himalayan
};

UENUM()
enum class ESubductionRole : uint8
{
	None,
	Overriding,
	Subducting
};

UENUM()
enum class EBoundaryType : uint8
{
	None,
	Convergent,
	Divergent,
	Transform
};

UENUM()
enum class EPlatePersistencePolicy : uint8
{
	Protected,
	Retirable
};

UENUM()
enum class EContinentalStabilizerMode : uint8
{
	Legacy,
	Incremental,
	Shadow
};

// One sample point on the canonical sphere.
struct FCanonicalSample
{
	FVector Position = FVector::ZeroVector;
	int32 PlateId = -1;
	int32 PrevPlateId = -1;
	float OwnershipMargin = 0.0f;
	bool bIsBoundary = false;
	ECrustType CrustType = ECrustType::Continental;
	float Elevation = 0.0f;
	float Thickness = 0.0f;
	float Age = 0.0f;
	FVector RidgeDirection = FVector::ZeroVector;
	FVector FoldDirection = FVector::ZeroVector;
	EOrogenyType OrogenyType = EOrogenyType::None;
	float OrogenyAge = 0.0f;
	int32 TerraneId = -1;
	FVector BoundaryNormal = FVector::ZeroVector;
	EBoundaryType BoundaryType = EBoundaryType::None;
	bool bGapDetected = false;
	int32 FlankingPlateIdA = INDEX_NONE;
	int32 FlankingPlateIdB = INDEX_NONE;
	bool bOverlapDetected = false;
	ESubductionRole SubductionRole = ESubductionRole::None;
	int32 SubductionOpposingPlateId = INDEX_NONE;
	float SubductionDistanceKm = -1.0f;
	float SubductionConvergenceSpeedMmPerYear = 0.0f;
	bool bIsSubductionFront = false;
	float CollisionDistanceKm = -1.0f;
	float CollisionConvergenceSpeedMmPerYear = 0.0f;
	bool bIsCollisionFront = false;
	uint8 NumOverlapPlateIds = 0;
	int32 OverlapPlateIds[4] = { INDEX_NONE, INDEX_NONE, INDEX_NONE, INDEX_NONE };
};

// Triangle from the global SDT (indices into Samples array).
struct FDelaunayTriangle
{
	int32 V[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };
};

// Mutable crust attributes carried by a plate between reconciliations.
// Positions are never stored here; they are computed on demand by rotating
// the canonical sample position using the plate's cumulative rotation.
struct FCarriedSampleData
{
	int32 CanonicalSampleIndex = INDEX_NONE;
	float Elevation = 0.0f;
	float Thickness = 0.0f;
	float Age = 0.0f;
	FVector RidgeDirection = FVector::ZeroVector;
	FVector FoldDirection = FVector::ZeroVector;
	EOrogenyType OrogenyType = EOrogenyType::None;
	float OrogenyAge = 0.0f;
	ECrustType CrustType = ECrustType::Continental;
	ESubductionRole SubductionRole = ESubductionRole::None;
	int32 SubductionOpposingPlateId = INDEX_NONE;
	float SubductionDistanceKm = -1.0f;
	float SubductionConvergenceSpeedMmPerYear = 0.0f;
	bool bIsSubductionFront = false;
	float CollisionDistanceKm = -1.0f;
	float CollisionConvergenceSpeedMmPerYear = 0.0f;
	bool bIsCollisionFront = false;
};

// Result of a containment query against moved plate triangle soups.
struct FContainmentQueryResult
{
	bool bFoundContainingPlate = false;
	bool bGap = true;
	bool bOverlap = false;
	int32 PlateId = INDEX_NONE;
	int32 TriangleIndex = INDEX_NONE; // Global SDT triangle index
	FVector Barycentric = FVector::ZeroVector;
	int32 NumCapCandidates = 0;
	int32 NumContainingPlates = 0;
	int32 ContainingPlateIds[4] = { INDEX_NONE, INDEX_NONE, INDEX_NONE, INDEX_NONE };
};

struct FReconcilePhaseTimings
{
	double SampleBufferCopyMs = 0.0;
	double Phase1BuildSpatialMs = 0.0;
	double Phase1SoupExtractTotalMs = 0.0;
	double Phase1SoupExtractMaxMs = 0.0;
	double Phase1CapBuildTotalMs = 0.0;
	double Phase1CapBuildMaxMs = 0.0;
	double Phase1BVHBuildTotalMs = 0.0;
	double Phase1BVHBuildMaxMs = 0.0;
	double Phase2OwnershipMs = 0.0;
	double Phase3InterpolationMs = 0.0;
	double Phase4GapMs = 0.0;
	double Phase5OverlapMs = 0.0;
	double Phase6MembershipMs = 0.0;
	double Phase6SanitizeOwnershipMs = 0.0;
	double Phase6PersistenceMs = 0.0;
	double Phase6RebuildMembershipMs = 0.0;
	double Phase6RebuildCarriedMs = 0.0;
	double Phase6ClassifyTrianglesMs = 0.0;
	double Phase6SpatialRebuildMs = 0.0;
	double Phase7SubductionMs = 0.0;
	double Phase7TerraneMs = 0.0;
	double Phase8CollisionMs = 0.0;
	double Phase8PostCollisionRefreshMs = 0.0;
	double ContinentalStabilizerMs = 0.0;
	double StabilizerPromoteToMinimumMs = 0.0;
	double StabilizerBuildComponentsMs = 0.0;
	double StabilizerComponentPruneMs = 0.0;
	double StabilizerPromoteToTargetMs = 0.0;
	double StabilizerTrimMs = 0.0;
	double PrevPlateWritebackMs = 0.0;
	double Phase9SubductionMs = 0.0;
	double PhaseSumMs = 0.0;
	double UnaccountedMs = 0.0;
	double UnaccountedPct = 0.0;
	double TotalMs = 0.0;
	int32 StabilizerPromotionCount = 0;
	int32 StabilizerComponentDemotionCount = 0;
	int32 StabilizerTrimDemotionCount = 0;
	int32 StabilizerHeapPushCount = 0;
	int32 StabilizerHeapStalePopCount = 0;
	bool bContinentalStabilizerShadowMatched = true;
	int32 ContinentalStabilizerShadowMismatchSampleIndex = INDEX_NONE;
	FString ContinentalStabilizerShadowMismatchField;
	int32 GapSamples = 0;
	int32 ArtifactGapResolvedSamples = 0;
	int32 DivergentGapSamples = 0;
	int32 OverlapSamples = 0;
	int32 Phase2FastPathResolvedSamples = 0;
	int32 Phase2FullQuerySamples = 0;
	int32 Phase6SpatialDirtyPlateCount = 0;
	int32 Phase6SpatialRebuiltPlateCount = 0;
};

struct FPlate
{
	int32 Id = -1;
	FVector RotationAxis = FVector::UpVector;
	float AngularSpeed = 0.0f;
	FQuat CumulativeRotation = FQuat::Identity;
	double AngularDisplacementSinceReconcile = 0.0;
	TArray<int32> SampleIndices;
	TArray<FCarriedSampleData> CarriedSamples;
	TArray<int32> InteriorTriangles;
	TArray<int32> BoundaryTriangles;
	EPlatePersistencePolicy PersistencePolicy = EPlatePersistencePolicy::Protected;
	FVector IdentityAnchorDirection = FVector::ForwardVector;
	int32 InitialSampleCount = 0;
	FVector CanonicalCenterDirection = FVector::ForwardVector;

	FVector GetRotatedCanonicalPosition(const FVector& CanonicalPosition) const
	{
		return CumulativeRotation.RotateVector(CanonicalPosition);
	}
};

class AUROUS_API FTectonicPlanet
{
public:
	FTectonicPlanet();
	~FTectonicPlanet();
	FTectonicPlanet(const FTectonicPlanet& Other);
	FTectonicPlanet& operator=(const FTectonicPlanet& Other);
	FTectonicPlanet(FTectonicPlanet&& Other) noexcept;
	FTectonicPlanet& operator=(FTectonicPlanet&& Other) noexcept;

	void Initialize(int32 NumSamples = 500000);

	const TArray<FCanonicalSample>& GetSamples() const;
	const TArray<FDelaunayTriangle>& GetTriangles() const { return Triangles; }
	const TArray<TArray<int32>>& GetAdjacency() const { return Adjacency; }
	const TArray<FPlate>& GetPlates() const { return Plates; }
	TArray<FPlate>& GetPlates() { return Plates; }
	int32 GetNumSamples() const;

	void InitializePlates(int32 NumPlates = 7, int32 RandomSeed = 42);
	void ClassifyTriangles();
	void UpdateBoundaryFlags();

	// Motion / timestep simulation
	void AdvancePlateMotionStep();
	bool ShouldReconcile() const;
	void ResetDisplacementTracking();
	void StepSimulation();
	void Reconcile();

	// On-demand rotated position lookup (positions are never stored per-plate).
	FVector GetRotatedSamplePosition(const FPlate& Plate, int32 CanonicalSampleIndex) const;
	FVector ComputeSurfaceVelocity(int32 PlateId, const FVector& Position) const;

	// Spatial query infrastructure (rebuilt at reconciliation start; lazily rebuilt on demand if dirty).
	void RebuildSpatialQueryData();
	FContainmentQueryResult QueryContainment(const FVector& Position) const;
	bool GetPlateSoupTriangleAndVertexCounts(int32 PlateId, int32& OutTriangleCount, int32& OutVertexCount) const;
	bool GetPlateSoupRotatedVertex(int32 PlateId, int32 CanonicalSampleIndex, FVector& OutRotatedPosition) const;
	bool GetPlateBoundingCap(int32 PlateId, FVector& OutCenterDirection, double& OutCosHalfAngle) const;

	// Simulation state accessors
	double GetAverageSampleSpacing() const { return AverageSampleSpacing; }
	double GetReconcileDisplacementThreshold() const { return ReconcileDisplacementThreshold; }
	double GetMaxAngularDisplacementSinceReconcile() const { return MaxAngularDisplacementSinceReconcile; }
	int64 GetTimestepCounter() const { return TimestepCounter; }
	int32 GetReconcileCount() const { return ReconcileCount; }
	bool WasReconcileTriggeredLastStep() const { return bReconcileTriggeredLastStep; }
	int32 GetBoundarySampleCount() const { return BoundarySampleCount; }
	double GetBoundaryMeanDepthHops() const { return BoundaryMeanDepthHops; }
	int32 GetBoundaryMaxDepthHops() const { return BoundaryMaxDepthHops; }
	int32 GetBoundaryDeepSampleCount() const { return BoundaryDeepSampleCount; }
	int32 GetContinentalSampleCount() const { return ContinentalSampleCount; }
	int32 GetContinentalPlateCount() const { return ContinentalPlateCount; }
	double GetContinentalAreaFraction() const { return ContinentalAreaFraction; }
	int32 GetContinentalComponentCount() const { return ContinentalComponentCount; }
	int32 GetLargestContinentalComponentSize() const { return LargestContinentalComponentSize; }
	int32 GetMaxPlateComponentCount() const { return MaxPlateComponentCount; }
	int32 GetDetachedPlateFragmentSampleCount() const { return DetachedPlateFragmentSampleCount; }
	int32 GetLargestDetachedPlateFragmentSize() const { return LargestDetachedPlateFragmentSize; }
	int32 GetSubductionFrontSampleCount() const { return SubductionFrontSampleCount; }
	int32 GetAndeanSampleCount() const { return AndeanSampleCount; }
	int32 GetTrackedTerraneCount() const { return TrackedTerraneCount; }
	int32 GetActiveTerraneCount() const { return ActiveTerraneCount; }
	int32 GetMergedTerraneCount() const { return MergedTerraneCount; }
	int32 GetCollisionEventCount() const { return CollisionEventCount; }
	int32 GetHimalayanSampleCount() const { return HimalayanSampleCount; }
	int32 GetPendingCollisionSampleCount() const { return PendingCollisionSampleCount; }
	float GetMaxSubductionDistanceKm() const { return MaxSubductionDistanceKm; }
	int32 GetMinProtectedPlateSampleCount() const { return MinProtectedPlateSampleCount; }
	int32 GetEmptyProtectedPlateCount() const { return EmptyProtectedPlateCount; }
	int32 GetRescuedProtectedPlateCount() const { return RescuedProtectedPlateCount; }
	int32 GetRescuedProtectedSampleCount() const { return RescuedProtectedSampleCount; }
	int32 GetRepeatedlyRescuedProtectedSampleCount() const { return RepeatedlyRescuedProtectedSampleCount; }
	const FReconcilePhaseTimings& GetLastReconcileTimings() const { return LastReconcileTimings; }
	float GetHysteresisThreshold() const { return HysteresisThreshold; }
	void SetHysteresisThreshold(float InValue) { HysteresisThreshold = FMath::Max(0.0f, InValue); }
	float GetBoundaryConfidenceThreshold() const { return BoundaryConfidenceThreshold; }
	void SetBoundaryConfidenceThreshold(float InValue) { BoundaryConfidenceThreshold = FMath::Max(0.0f, InValue); }
	int32 GetLastGapSampleCount() const { return LastGapSampleCount; }
	EContinentalStabilizerMode GetContinentalStabilizerMode() const { return ContinentalStabilizerMode; }
	void SetContinentalStabilizerMode(const EContinentalStabilizerMode InMode) { ContinentalStabilizerMode = (InMode == EContinentalStabilizerMode::Incremental) ? InMode : EContinentalStabilizerMode::Incremental; }
	int32 GetLastOverlapSampleCount() const { return LastOverlapSampleCount; }
	double GetTimestepDurationYears() const { return TimestepDurationYears; }
	double GetTimestepDurationMy() const { return TimestepDurationMy; }
	float GetOceanicDampeningRateMmPerYear() const { return OceanicDampeningRateMmPerYear; }
	float GetOceanicTrenchElevation() const { return OceanicTrenchElevationKm; }
	float GetRidgeElevation() const { return RidgeElevationKm; }
	float GetAbyssalPlainElevation() const { return AbyssalPlainElevationKm; }
	float GetSubductionPeakDistanceKm() const { return SubductionPeakDistanceKm; }
	float GetSubductionTrenchRadiusKm() const { return SubductionTrenchRadiusKm; }
	float GetFoldBlendAtMaxSpeed() const { return FoldBlendAtMaxSpeed; }
	float GetSlabPullAxisMaxDegreesPerStep() const { return SlabPullAxisMaxDegreesPerStep; }
	double GetTargetContinentalAreaFraction() const { return TargetContinentalAreaFraction; }
	void SetTargetContinentalAreaFraction(const double InValue) { TargetContinentalAreaFraction = FMath::Clamp(InValue, 0.0, 1.0); }
	double GetMinContinentalAreaFraction() const { return MinContinentalAreaFraction; }
	void SetMinContinentalAreaFraction(const double InValue) { MinContinentalAreaFraction = FMath::Clamp(InValue, 0.0, 1.0); }
	double GetMaxContinentalAreaFraction() const { return MaxContinentalAreaFraction; }
	void SetMaxContinentalAreaFraction(const double InValue) { MaxContinentalAreaFraction = FMath::Clamp(InValue, 0.0, 1.0); }
	double GetMinContinentalPlateFraction() const { return MinContinentalPlateFraction; }
	void SetMinContinentalPlateFraction(const double InValue) { MinContinentalPlateFraction = FMath::Clamp(InValue, 0.0, 1.0); }

private:
	struct FSpatialQueryData;
	struct FPhase2SampleState;
	struct FTerraneRecord
	{
		int32 TerraneId = INDEX_NONE;
		int32 PlateId = INDEX_NONE;
		int32 AnchorSampleIndex = INDEX_NONE;
		FVector CentroidDirection = FVector::ZeroVector;
		double AreaKm2 = 0.0;
		int32 SampleCount = 0;
		int32 MergedIntoTerraneId = INDEX_NONE;
		int32 LastSeenReconcile = -1;
		bool bActive = false;
	};
	struct FCollisionEventRecord
	{
		int32 DonorTerraneId = INDEX_NONE;
		int32 ReceiverTerraneId = INDEX_NONE;
		int32 ReceiverPlateId = INDEX_NONE;
		int32 ReconcileOrdinal = -1;
		int32 ContactSampleCount = 0;
		float MeanConvergenceSpeedMmPerYear = 0.0f;
	};

	void GenerateFibonacciSphere(int32 N);
	void BuildDelaunayTriangulation();
	void BuildAdjacencyGraph();
	void RebuildAdjacencyEdgeDistanceCache(const TArray<FCanonicalSample>& InSamples);
	void RebuildCarriedSampleWorkspaces();
	void RebuildCarriedSampleWorkspacesForSamples(const TArray<FCanonicalSample>& InSamples);
	void InvalidateSpatialQueryData();
	void BuildSpatialQueryDataInternal() const;
	double UpdateSpatialCapsForCurrentRotationsInternal() const;
	void EnsureSpatialQueryDataBuilt() const;
	const TArray<FCanonicalSample>& GetReadableSamplesInternal() const;
	TArray<FCanonicalSample>& GetWritableSamplesInternal(int32 WriteBufferIndex);
	void UpdateBoundaryFlagsForSamples(TArray<FCanonicalSample>& InOutSamples);
	void UpdateBoundaryClassificationForSamples(TArray<FCanonicalSample>& InOutSamples);
	void UpdateGapFlankingPlatesForSamples(TArray<FCanonicalSample>& InOutSamples);
	void ClassifyTrianglesForSamples(TArray<FCanonicalSample>& InOutSamples);
	void UpdateBoundaryAndContinentDiagnosticsForSamples(const TArray<FCanonicalSample>& InSamples);
	void StabilizeContinentalCrustForSamples(
		const TArray<FCanonicalSample>& PreviousSamples,
		TArray<FCanonicalSample>& InOutSamples,
		FReconcilePhaseTimings* InOutTimings = nullptr) const;
	void StabilizeContinentalCrustForSamplesIncremental(
		const TArray<FCanonicalSample>& PreviousSamples,
		TArray<FCanonicalSample>& InOutSamples,
		FReconcilePhaseTimings* InOutTimings = nullptr) const;
	bool ResolveGapSamplePhase4(
		int32 SampleIndex,
		const TArray<uint8>& GapFlags,
		TArray<FCanonicalSample>& InOutSamples,
		bool& bOutDivergentGapCreated) const;
	void RunBoundaryLikeLocalOwnershipSanitizePass(
		const TArray<FPhase2SampleState>& Phase2States,
		TArray<FCanonicalSample>& InOutSamples,
		int32 NumIterations,
		const TArray<uint8>* SeedSampleFlags = nullptr) const;
	// Pre-M5 invariant: non-gap ownership for each plate must remain connected until rifting exists.
	void EnforceConnectedPlateOwnershipForSamples(TArray<FCanonicalSample>& InOutSamples, const TArray<uint8>* CandidatePlateFlags = nullptr) const;
	bool RescueProtectedPlateOwnershipForSamples(TArray<FCanonicalSample>& InOutSamples, const TArray<uint8>* CandidatePlateFlags = nullptr);
	void RebuildPlateMembershipFromSamples(const TArray<FCanonicalSample>& InSamples, const TArray<uint8>* DirtyPlateFlags = nullptr);
	void UpdatePlateCanonicalCentersFromSamples(const TArray<FCanonicalSample>& InSamples, const TArray<uint8>* DirtyPlateFlags = nullptr);
	bool IsPlateActive(int32 PlateId) const;
	bool IsPlateProtected(int32 PlateId) const;
	int32 GetInitialPlateFloorSamples(int32 TotalSampleCount, int32 PlateCount) const;
	int32 GetPersistentPlateFloorSamples(int32 PlateId) const;
	int32 FindNearestPlateByCap(const FVector& Direction) const;
	float ComputeOwnershipMarginFromCaps(const FVector& Direction) const;
	bool ResolveDominantConvergentInteractionForSample(
		const TArray<FCanonicalSample>& InSamples,
		int32 SampleIndex,
		int32& OutOpposingPlateId,
		int32& OutRepresentativeNeighborIndex,
		float& OutConvergenceSpeedMmPerYear) const;
	void DetectTerranesForSamples(TArray<FCanonicalSample>& InOutSamples);
	bool ApplyContinentalCollisionEventsForSamples(TArray<FCanonicalSample>& InOutSamples, TArray<uint8>* OutDirtyPlateFlags = nullptr);
	void RefreshCanonicalStateAfterCollision(
		const TArray<FPhase2SampleState>& Phase2States,
		TArray<FCanonicalSample>& InOutSamples,
		const TArray<uint8>* DirtyPlateFlags = nullptr);
	void RefreshCanonicalStateAfterCollision(TArray<FCanonicalSample>& InOutSamples);
	void UpdateSubductionFieldsForSamples(TArray<FCanonicalSample>& InOutSamples);
	void RefreshSubductionMetricsFromCarriedSamples();
	void RefreshTerraneMetrics();
	FTerraneRecord* FindTerraneRecordById(int32 TerraneId);
	const FTerraneRecord* FindTerraneRecordById(int32 TerraneId) const;
	static uint64 MakeCollisionHistoryKey(int32 TerraneIdA, int32 TerraneIdB);
	static FVector ResolveDirectionField(
		const FVector& SamplePosition,
		const FVector& V0,
		const FVector& V1,
		const FVector& V2,
		const FVector& Barycentric);
	static ECrustType MajorityCrustType(ECrustType A, ECrustType B, ECrustType C);
	static EOrogenyType MajorityOrogenyType(EOrogenyType A, EOrogenyType B, EOrogenyType C);
	static FVector3d ComputePlanarBarycentric(const FVector3d& A, const FVector3d& B, const FVector3d& C, const FVector3d& P);

	TArray<FCanonicalSample> SampleBuffers[2];
	TAtomic<int32> ReadableSampleBufferIndex { 0 };
	TArray<FDelaunayTriangle> Triangles;
	TArray<TArray<int32>> Adjacency;
	TArray<TArray<float>> AdjacencyEdgeDistancesKm;
	TArray<FPlate> Plates;
	TArray<FTerraneRecord> TerraneRecords;
	TArray<FCollisionEventRecord> CollisionEvents;
	TSet<uint64> CollisionHistoryKeys;
	mutable TUniquePtr<FSpatialQueryData> SpatialQueryData;

	double AverageSampleSpacing = 0.0;
	double AverageCellAreaKm2 = 0.0;
	double InitialMeanPlateAreaKm2 = 0.0;
	double ReconcileDisplacementThreshold = 0.0;
	double MaxAngularDisplacementSinceReconcile = 0.0;
	int64 TimestepCounter = 0;
	int32 ReconcileCount = 0;
	int32 NextTerraneId = 0;
	bool bReconcileTriggeredLastStep = false;
	int32 BoundarySampleCount = 0;
	double BoundaryMeanDepthHops = 0.0;
	int32 BoundaryMaxDepthHops = 0;
	int32 BoundaryDeepSampleCount = 0;
	int32 ContinentalSampleCount = 0;
	int32 ContinentalPlateCount = 0;
	int32 InitialContinentalPlateCount = 0;
	double ContinentalAreaFraction = 0.0;
	int32 ContinentalComponentCount = 0;
	int32 LargestContinentalComponentSize = 0;
	int32 MaxPlateComponentCount = 0;
	int32 DetachedPlateFragmentSampleCount = 0;
	int32 LargestDetachedPlateFragmentSize = 0;
	int32 SubductionFrontSampleCount = 0;
	int32 AndeanSampleCount = 0;
	int32 TrackedTerraneCount = 0;
	int32 ActiveTerraneCount = 0;
	int32 MergedTerraneCount = 0;
	int32 CollisionEventCount = 0;
	int32 HimalayanSampleCount = 0;
	int32 PendingCollisionSampleCount = 0;
	float MaxSubductionDistanceKm = 0.0f;
	int32 MinProtectedPlateSampleCount = 0;
	int32 EmptyProtectedPlateCount = 0;
	int32 RescuedProtectedPlateCount = 0;
	int32 RescuedProtectedSampleCount = 0;
	int32 RepeatedlyRescuedProtectedSampleCount = 0;
	int32 LastGapSampleCount = 0;
	int32 LastOverlapSampleCount = 0;
	float HysteresisThreshold = 0.15f;
	float BoundaryConfidenceThreshold = 0.15f;
	double TargetContinentalAreaFraction = 0.30;
	double MinContinentalAreaFraction = 0.25;
	double MaxContinentalAreaFraction = 0.40;
	double MinContinentalPlateFraction = 0.15;
	EContinentalStabilizerMode ContinentalStabilizerMode = EContinentalStabilizerMode::Incremental;
	FReconcilePhaseTimings LastReconcileTimings;

	static constexpr float OceanicDampeningRateMmPerYear = 0.04f;
	static constexpr float OceanicTrenchElevationKm = -10.0f;
	static constexpr double TimestepDurationYears = 2000000.0;
	static constexpr double TimestepDurationMy = 2.0;
	static constexpr float RidgeElevationKm = -1.0f;
	static constexpr float AbyssalPlainElevationKm = -6.0f;
	static constexpr float OceanicCrustThicknessKm = 7.0f;
	static constexpr float InitialOceanicPlateElevationKm = -4.0f;
	static constexpr double PlanetRadius = 6370.0;
	static constexpr float SubductionInfluenceRadiusKm = 1800.0f;
	static constexpr float SubductionPeakDistanceKm = 350.0f;
	static constexpr float SubductionTrenchRadiusKm = 400.0f;
	static constexpr float BaseSubductionUpliftMmPerYear = 0.6f;
	static constexpr float MaxSubductionSpeedMmPerYear = 100.0f;
	static constexpr float MaxContinentalElevationKm = 10.0f;
	static constexpr float FoldBlendAtMaxSpeed = 0.2f;
	static constexpr float SlabPullAxisMaxDegreesPerStep = 0.25f;

#if WITH_DEV_AUTOMATION_TESTS
	friend struct FTectonicPlanetTestAccess;
#endif
};



