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
enum class EBoundaryType : uint8
{
	None,
	Convergent,
	Divergent,
	Transform
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
	double Phase6RebuildMembershipMs = 0.0;
	double Phase6RebuildCarriedMs = 0.0;
	double Phase6ClassifyTrianglesMs = 0.0;
	double Phase6SpatialRebuildMs = 0.0;
	double Phase7TerraneMs = 0.0;
	double TotalMs = 0.0;
	int32 GapSamples = 0;
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
	const FReconcilePhaseTimings& GetLastReconcileTimings() const { return LastReconcileTimings; }
	float GetHysteresisThreshold() const { return HysteresisThreshold; }
	void SetHysteresisThreshold(float InValue) { HysteresisThreshold = FMath::Max(0.0f, InValue); }
	float GetBoundaryConfidenceThreshold() const { return BoundaryConfidenceThreshold; }
	void SetBoundaryConfidenceThreshold(float InValue) { BoundaryConfidenceThreshold = FMath::Max(0.0f, InValue); }
	int32 GetLastGapSampleCount() const { return LastGapSampleCount; }
	int32 GetLastOverlapSampleCount() const { return LastOverlapSampleCount; }

private:
	struct FSpatialQueryData;
	struct FPhase2SampleState;

	void GenerateFibonacciSphere(int32 N);
	void BuildDelaunayTriangulation();
	void BuildAdjacencyGraph();
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
	void RebuildPlateMembershipFromSamples(const TArray<FCanonicalSample>& InSamples);
	int32 FindNearestPlateByCap(const FVector& Direction) const;
	float ComputeOwnershipMarginFromCaps(const FVector& Direction) const;
	void RunTerraneDetectionStub() const;
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
	TArray<FPlate> Plates;
	mutable TUniquePtr<FSpatialQueryData> SpatialQueryData;

	double AverageSampleSpacing = 0.0;
	double ReconcileDisplacementThreshold = 0.0;
	double MaxAngularDisplacementSinceReconcile = 0.0;
	int64 TimestepCounter = 0;
	int32 ReconcileCount = 0;
	bool bReconcileTriggeredLastStep = false;
	int32 BoundarySampleCount = 0;
	int32 LastGapSampleCount = 0;
	int32 LastOverlapSampleCount = 0;
	float HysteresisThreshold = 0.15f;
	float BoundaryConfidenceThreshold = 0.15f;
	FReconcilePhaseTimings LastReconcileTimings;

	static constexpr double PlanetRadius = 6370.0;
};
