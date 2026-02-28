#include "TectonicPlanet.h"

#include "Async/ParallelFor.h"
#include "Async/TaskGraphInterfaces.h"
#include "CompGeom/ConvexHull3.h"
#include "HAL/PlatformTime.h"
#include "Math/RandomStream.h"
#include "PlateTriangleSoupAdapter.h"
#include "ShewchukPredicates.h"

namespace
{
	bool GShewchukPredicatesInitialized = false;

	uint64 HashTriangleIndexList(const TArray<int32>& TriangleIndices)
	{
		uint64 Hash = 1469598103934665603ull; // FNV-1a offset basis
		for (const int32 TriangleIndex : TriangleIndices)
		{
			Hash ^= static_cast<uint64>(static_cast<uint32>(TriangleIndex));
			Hash *= 1099511628211ull; // FNV-1a prime
		}
		Hash ^= static_cast<uint64>(TriangleIndices.Num());
		Hash *= 1099511628211ull;
		return Hash;
	}

	bool IsPointInsideSphericalTriangleRobust(const FVector3d& A, const FVector3d& B, const FVector3d& C, const FVector3d& P)
	{
		const double O[3] = { 0.0, 0.0, 0.0 };
		const double AD[3] = { A.X, A.Y, A.Z };
		const double BD[3] = { B.X, B.Y, B.Z };
		const double CD[3] = { C.X, C.Y, C.Z };
		const double PD[3] = { P.X, P.Y, P.Z };
		const double TriangleOrientation = orient3d(O, AD, BD, CD);
		if (TriangleOrientation == 0.0)
		{
			return false;
		}

		const double S0 = orient3d(O, AD, BD, PD);
		const double S1 = orient3d(O, BD, CD, PD);
		const double S2 = orient3d(O, CD, AD, PD);
		const bool bPositiveWinding = TriangleOrientation > 0.0;
		if (bPositiveWinding)
		{
			return S0 >= 0.0 && S1 >= 0.0 && S2 >= 0.0;
		}

		return S0 <= 0.0 && S1 <= 0.0 && S2 <= 0.0;
	}

	bool FindContainingTriangleInBVH(
		const FPlateTriangleSoupBVH& BVH,
		const FPlateTriangleSoupAdapter& Adapter,
		const FVector3d& QueryPoint,
		int32& OutLocalTriangleId,
		FVector3d& OutA,
		FVector3d& OutB,
		FVector3d& OutC)
	{
		OutLocalTriangleId = INDEX_NONE;
		if (BVH.RootIndex < 0)
		{
			return false;
		}

		TArray<int32, TInlineAllocator<64>> NodeStack;
		NodeStack.Add(BVH.RootIndex);

		while (NodeStack.Num() > 0)
		{
			const int32 NodeIndex = NodeStack.Pop(EAllowShrinking::No);
			if (!BVH.box_contains(NodeIndex, QueryPoint))
			{
				continue;
			}

			const int32 ListStartIndex = BVH.BoxToIndex[NodeIndex];
			if (ListStartIndex < BVH.TrianglesEnd)
			{
				const int32 NumTriangles = BVH.IndexList[ListStartIndex];
				for (int32 TriangleListIndex = 1; TriangleListIndex <= NumTriangles; ++TriangleListIndex)
				{
					const int32 LocalTriangleId = BVH.IndexList[ListStartIndex + TriangleListIndex];
					if (!Adapter.IsTriangle(LocalTriangleId))
					{
						continue;
					}

					Adapter.GetTriVertices(LocalTriangleId, OutA, OutB, OutC);
					if (IsPointInsideSphericalTriangleRobust(OutA, OutB, OutC, QueryPoint))
					{
						OutLocalTriangleId = LocalTriangleId;
						return true;
					}
				}
				continue;
			}

			const int32 ChildRecord = BVH.IndexList[ListStartIndex];
			if (ChildRecord < 0)
			{
				const int32 ChildNode = (-ChildRecord) - 1;
				if (ChildNode >= 0 && BVH.box_contains(ChildNode, QueryPoint))
				{
					NodeStack.Add(ChildNode);
				}
				continue;
			}

			const int32 ChildA = ChildRecord - 1;
			const int32 ChildB = BVH.IndexList[ListStartIndex + 1] - 1;
			const bool bContainsA = (ChildA >= 0) && BVH.box_contains(ChildA, QueryPoint);
			const bool bContainsB = (ChildB >= 0) && BVH.box_contains(ChildB, QueryPoint);

			if (!bContainsA && !bContainsB)
			{
				continue;
			}

			if (bContainsA && bContainsB)
			{
				// Traverse the nearer box first for faster early-out.
				const double DistA = BVH.BoxDistanceSqr(ChildA, QueryPoint);
				const double DistB = BVH.BoxDistanceSqr(ChildB, QueryPoint);
				if (DistA <= DistB)
				{
					NodeStack.Add(ChildB);
					NodeStack.Add(ChildA);
				}
				else
				{
					NodeStack.Add(ChildA);
					NodeStack.Add(ChildB);
				}
				continue;
			}

			NodeStack.Add(bContainsA ? ChildA : ChildB);
		}

		return false;
	}

	FVector MakeRandomUnitVector(FRandomStream& Random)
	{
		const double Z = Random.FRandRange(-1.0f, 1.0f);
		const double Phi = Random.FRandRange(0.0f, 2.0f * PI);
		const double RadiusXY = FMath::Sqrt(FMath::Max(0.0, 1.0 - Z * Z));
		const double SinPhi = FMath::Sin(Phi);
		const double CosPhi = FMath::Cos(Phi);
		return FVector(RadiusXY * CosPhi, RadiusXY * SinPhi, Z);
	}
}

struct FTectonicPlanet::FSpatialQueryData
{
	struct FPlateSpatialState
	{
		int32 PlateId = INDEX_NONE;
		FPlateTriangleSoupData SoupData;
		TUniquePtr<FPlateTriangleSoupAdapter> Adapter;
		TUniquePtr<FPlateTriangleSoupBVH> BVH;
		FVector CapCenterDirection = FVector::ZeroVector;
		double CapHalfAngle = 0.0;
		double CapCosHalfAngle = -1.0;
		bool bHasValidCap = false;
	};

	TArray<FPlateSpatialState> PlateStates;
	TArray<uint8> DirtyPlateFlags;
	bool bDirty = true;
	bool bCapsDirty = true;
	int32 LastDirtyPlateCount = 0;
	int32 LastRebuiltPlateCount = 0;
	double LastSoupExtractTotalMs = 0.0;
	double LastSoupExtractMaxMs = 0.0;
	double LastCapBuildTotalMs = 0.0;
	double LastCapBuildMaxMs = 0.0;
	double LastBVHBuildTotalMs = 0.0;
	double LastBVHBuildMaxMs = 0.0;
};

struct FTectonicPlanet::FPhase2SampleState
{
	int32 AssignedPlateId = INDEX_NONE;
	int32 TriangleIndex = INDEX_NONE;
	FVector Barycentric = FVector::ZeroVector;
	float OwnershipMargin = 0.0f;
	bool bGap = true;
	bool bOverlap = false;
	int32 NumContainingPlates = 0;
	int32 ContainingPlateIds[4] = { INDEX_NONE, INDEX_NONE, INDEX_NONE, INDEX_NONE };
};

FTectonicPlanet::FTectonicPlanet()
	: SpatialQueryData(MakeUnique<FSpatialQueryData>())
{
}

FTectonicPlanet::~FTectonicPlanet() = default;

FTectonicPlanet::FTectonicPlanet(const FTectonicPlanet& Other)
	: Triangles(Other.Triangles)
	, Adjacency(Other.Adjacency)
	, Plates(Other.Plates)
	, SpatialQueryData(MakeUnique<FSpatialQueryData>())
	, AverageSampleSpacing(Other.AverageSampleSpacing)
	, ReconcileDisplacementThreshold(Other.ReconcileDisplacementThreshold)
	, MaxAngularDisplacementSinceReconcile(Other.MaxAngularDisplacementSinceReconcile)
	, TimestepCounter(Other.TimestepCounter)
	, ReconcileCount(Other.ReconcileCount)
	, bReconcileTriggeredLastStep(Other.bReconcileTriggeredLastStep)
	, LastGapSampleCount(Other.LastGapSampleCount)
	, LastOverlapSampleCount(Other.LastOverlapSampleCount)
	, HysteresisThreshold(Other.HysteresisThreshold)
	, BoundaryConfidenceThreshold(Other.BoundaryConfidenceThreshold)
	, LastReconcileTimings(Other.LastReconcileTimings)
{
	SampleBuffers[0] = Other.SampleBuffers[0];
	SampleBuffers[1] = Other.SampleBuffers[1];
	ReadableSampleBufferIndex.Store(Other.ReadableSampleBufferIndex.Load());
	SpatialQueryData->bDirty = true;
	SpatialQueryData->DirtyPlateFlags.Init(1, Plates.Num());
	SpatialQueryData->bCapsDirty = true;
}

FTectonicPlanet& FTectonicPlanet::operator=(const FTectonicPlanet& Other)
{
	if (this != &Other)
	{
		SampleBuffers[0] = Other.SampleBuffers[0];
		SampleBuffers[1] = Other.SampleBuffers[1];
		ReadableSampleBufferIndex.Store(Other.ReadableSampleBufferIndex.Load());
		Triangles = Other.Triangles;
		Adjacency = Other.Adjacency;
		Plates = Other.Plates;
		AverageSampleSpacing = Other.AverageSampleSpacing;
		ReconcileDisplacementThreshold = Other.ReconcileDisplacementThreshold;
		MaxAngularDisplacementSinceReconcile = Other.MaxAngularDisplacementSinceReconcile;
		TimestepCounter = Other.TimestepCounter;
		ReconcileCount = Other.ReconcileCount;
		bReconcileTriggeredLastStep = Other.bReconcileTriggeredLastStep;
		LastGapSampleCount = Other.LastGapSampleCount;
		LastOverlapSampleCount = Other.LastOverlapSampleCount;
		HysteresisThreshold = Other.HysteresisThreshold;
		BoundaryConfidenceThreshold = Other.BoundaryConfidenceThreshold;
		LastReconcileTimings = Other.LastReconcileTimings;

		if (!SpatialQueryData)
		{
			SpatialQueryData = MakeUnique<FSpatialQueryData>();
		}
		SpatialQueryData->PlateStates.Reset();
		SpatialQueryData->bDirty = true;
		SpatialQueryData->DirtyPlateFlags.Init(1, Plates.Num());
		SpatialQueryData->bCapsDirty = true;
	}

	return *this;
}

FTectonicPlanet::FTectonicPlanet(FTectonicPlanet&& Other) noexcept
	: Triangles(MoveTemp(Other.Triangles))
	, Adjacency(MoveTemp(Other.Adjacency))
	, Plates(MoveTemp(Other.Plates))
	, SpatialQueryData(MoveTemp(Other.SpatialQueryData))
	, AverageSampleSpacing(Other.AverageSampleSpacing)
	, ReconcileDisplacementThreshold(Other.ReconcileDisplacementThreshold)
	, MaxAngularDisplacementSinceReconcile(Other.MaxAngularDisplacementSinceReconcile)
	, TimestepCounter(Other.TimestepCounter)
	, ReconcileCount(Other.ReconcileCount)
	, bReconcileTriggeredLastStep(Other.bReconcileTriggeredLastStep)
	, LastGapSampleCount(Other.LastGapSampleCount)
	, LastOverlapSampleCount(Other.LastOverlapSampleCount)
	, HysteresisThreshold(Other.HysteresisThreshold)
	, BoundaryConfidenceThreshold(Other.BoundaryConfidenceThreshold)
	, LastReconcileTimings(Other.LastReconcileTimings)
{
	SampleBuffers[0] = MoveTemp(Other.SampleBuffers[0]);
	SampleBuffers[1] = MoveTemp(Other.SampleBuffers[1]);
	ReadableSampleBufferIndex.Store(Other.ReadableSampleBufferIndex.Load());
	if (!SpatialQueryData)
	{
		SpatialQueryData = MakeUnique<FSpatialQueryData>();
	}
}

FTectonicPlanet& FTectonicPlanet::operator=(FTectonicPlanet&& Other) noexcept
{
	if (this != &Other)
	{
		SampleBuffers[0] = MoveTemp(Other.SampleBuffers[0]);
		SampleBuffers[1] = MoveTemp(Other.SampleBuffers[1]);
		ReadableSampleBufferIndex.Store(Other.ReadableSampleBufferIndex.Load());
		Triangles = MoveTemp(Other.Triangles);
		Adjacency = MoveTemp(Other.Adjacency);
		Plates = MoveTemp(Other.Plates);
		SpatialQueryData = MoveTemp(Other.SpatialQueryData);
		AverageSampleSpacing = Other.AverageSampleSpacing;
		ReconcileDisplacementThreshold = Other.ReconcileDisplacementThreshold;
		MaxAngularDisplacementSinceReconcile = Other.MaxAngularDisplacementSinceReconcile;
		TimestepCounter = Other.TimestepCounter;
		ReconcileCount = Other.ReconcileCount;
		bReconcileTriggeredLastStep = Other.bReconcileTriggeredLastStep;
		LastGapSampleCount = Other.LastGapSampleCount;
		LastOverlapSampleCount = Other.LastOverlapSampleCount;
		HysteresisThreshold = Other.HysteresisThreshold;
		BoundaryConfidenceThreshold = Other.BoundaryConfidenceThreshold;
		LastReconcileTimings = Other.LastReconcileTimings;

		if (!SpatialQueryData)
		{
			SpatialQueryData = MakeUnique<FSpatialQueryData>();
		}
	}

	return *this;
}

const TArray<FCanonicalSample>& FTectonicPlanet::GetSamples() const
{
	return GetReadableSamplesInternal();
}

int32 FTectonicPlanet::GetNumSamples() const
{
	return GetReadableSamplesInternal().Num();
}

void FTectonicPlanet::Initialize(int32 NumSamples)
{
	checkf(NumSamples >= 4, TEXT("FTectonicPlanet::Initialize requires at least 4 samples."));

	ReadableSampleBufferIndex.Store(0);
	SampleBuffers[0].Reset();
	SampleBuffers[1].Reset();
	Triangles.Reset();
	Adjacency.Reset();
	Plates.Reset();
	AverageSampleSpacing = 0.0;
	ReconcileDisplacementThreshold = 0.0;
	MaxAngularDisplacementSinceReconcile = 0.0;
	TimestepCounter = 0;
	ReconcileCount = 0;
	bReconcileTriggeredLastStep = false;
	BoundarySampleCount = 0;
	LastGapSampleCount = 0;
	LastOverlapSampleCount = 0;
	LastReconcileTimings = FReconcilePhaseTimings{};
	InvalidateSpatialQueryData();

	if (!GShewchukPredicatesInitialized)
	{
		exactinit();
		GShewchukPredicatesInitialized = true;
	}

	GenerateFibonacciSphere(NumSamples);
	BuildDelaunayTriangulation();
	BuildAdjacencyGraph();
	AverageSampleSpacing = FMath::Sqrt((4.0 * static_cast<double>(PI)) / static_cast<double>(GetReadableSamplesInternal().Num()));
	ReconcileDisplacementThreshold = 0.5 * AverageSampleSpacing;
	UpdateBoundaryFlags();

	// Keep both buffers coherent until first reconciliation swap.
	SampleBuffers[1] = SampleBuffers[0];
}

void FTectonicPlanet::InitializePlates(int32 NumPlates, int32 RandomSeed)
{
	TArray<FCanonicalSample>& Samples = SampleBuffers[ReadableSampleBufferIndex.Load()];
	checkf(Samples.Num() > 0, TEXT("InitializePlates requires samples. Call Initialize() first."));
	checkf(Triangles.Num() > 0, TEXT("InitializePlates requires a triangulation. Call Initialize() first."));
	checkf(NumPlates > 0, TEXT("InitializePlates requires NumPlates > 0."));

	const int32 ClampedNumPlates = FMath::Min(NumPlates, Samples.Num());
	FRandomStream Rng(RandomSeed);
	const double LocalAverageSampleSpacing = (Samples.Num() > 0)
		? FMath::Sqrt((4.0 * static_cast<double>(PI)) / static_cast<double>(Samples.Num()))
		: 1.0;

	TArray<FVector> Centroids;
	Centroids.Reserve(ClampedNumPlates);
	for (int32 PlateIndex = 0; PlateIndex < ClampedNumPlates; ++PlateIndex)
	{
		Centroids.Add(MakeRandomUnitVector(Rng).GetSafeNormal());
	}

	Plates.Reset();
	Plates.SetNum(ClampedNumPlates);
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		FPlate& Plate = Plates[PlateIndex];
		Plate = FPlate{};
		Plate.Id = PlateIndex;
		Plate.RotationAxis = MakeRandomUnitVector(Rng).GetSafeNormal();
		Plate.AngularSpeed = Rng.FRandRange(0.5f, 1.5f) * 3.14e-2f;
	}

	TSet<int32> ContinentalPlates;
	const int32 NumContinentalPlates = FMath::Clamp(Rng.RandRange(1, 3), 1, ClampedNumPlates);
	while (ContinentalPlates.Num() < NumContinentalPlates)
	{
		ContinentalPlates.Add(Rng.RandRange(0, ClampedNumPlates - 1));
	}

	int32 NumAssignedContinental = 0;
	int32 NumAssignedOceanic = 0;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		double BestDot = -TNumericLimits<double>::Max();
		double SecondBestDot = -TNumericLimits<double>::Max();
		int32 BestPlateIndex = 0;

		const FVector& Position = Samples[SampleIndex].Position;
		for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
		{
			const double Dot = FVector::DotProduct(Position, Centroids[PlateIndex]);
			if (Dot > BestDot)
			{
				SecondBestDot = BestDot;
				BestDot = Dot;
				BestPlateIndex = PlateIndex;
			}
			else if (Dot > SecondBestDot)
			{
				SecondBestDot = Dot;
			}
		}

		FCanonicalSample& Sample = Samples[SampleIndex];
		Sample.PlateId = BestPlateIndex;
		Sample.PrevPlateId = BestPlateIndex;
		Sample.BoundaryNormal = FVector::ZeroVector;
		Sample.BoundaryType = EBoundaryType::None;
		Sample.bGapDetected = false;
		Sample.FlankingPlateIdA = INDEX_NONE;
		Sample.FlankingPlateIdB = INDEX_NONE;
		Sample.bOverlapDetected = false;
		Sample.NumOverlapPlateIds = 0;
		for (int32 OverlapIndex = 0; OverlapIndex < UE_ARRAY_COUNT(Sample.OverlapPlateIds); ++OverlapIndex)
		{
			Sample.OverlapPlateIds[OverlapIndex] = INDEX_NONE;
		}

		const double BestChordDistance = FMath::Sqrt(FMath::Max(0.0, 2.0 - 2.0 * BestDot));
		const double SecondChordDistance = (SecondBestDot <= -1.0)
			? BestChordDistance
			: FMath::Sqrt(FMath::Max(0.0, 2.0 - 2.0 * SecondBestDot));
		const double MarginNumerator = FMath::Max(0.0, SecondChordDistance - BestChordDistance);
		Sample.OwnershipMargin = (LocalAverageSampleSpacing > UE_DOUBLE_SMALL_NUMBER)
			? static_cast<float>(MarginNumerator / LocalAverageSampleSpacing)
			: 0.0f;

		if (ContinentalPlates.Contains(BestPlateIndex))
		{
			Sample.CrustType = ECrustType::Continental;
			Sample.Elevation = 0.5f;
			Sample.Thickness = 35.0f;
			++NumAssignedContinental;
		}
		else
		{
			Sample.CrustType = ECrustType::Oceanic;
			Sample.Elevation = -4.0f;
			Sample.Thickness = 7.0f;
			Sample.TerraneId = -1;
			++NumAssignedOceanic;
		}
	}

	ResetDisplacementTracking();
	RebuildPlateMembershipFromSamples(Samples);
	RebuildCarriedSampleWorkspacesForSamples(Samples);
	ClassifyTrianglesForSamples(Samples);
	InvalidateSpatialQueryData();
	RebuildSpatialQueryData();
	SampleBuffers[1 - ReadableSampleBufferIndex.Load()] = Samples;

	UE_LOG(LogTemp, Log, TEXT("Initialized %d plates (seed=%d): continental plates=%d continental samples=%d oceanic samples=%d"),
		Plates.Num(), RandomSeed, ContinentalPlates.Num(), NumAssignedContinental, NumAssignedOceanic);
}

void FTectonicPlanet::ClassifyTriangles()
{
	TArray<FCanonicalSample>& Samples = SampleBuffers[ReadableSampleBufferIndex.Load()];
	ClassifyTrianglesForSamples(Samples);
}

void FTectonicPlanet::UpdateBoundaryFlags()
{
	TArray<FCanonicalSample>& Samples = SampleBuffers[ReadableSampleBufferIndex.Load()];
	UpdateBoundaryFlagsForSamples(Samples);
}

void FTectonicPlanet::AdvancePlateMotionStep()
{
	if (Plates.Num() == 0)
	{
		return;
	}

	for (FPlate& Plate : Plates)
	{
		const FVector Axis = Plate.RotationAxis.GetSafeNormal();
		const FQuat DeltaRotation(Axis, Plate.AngularSpeed);
		Plate.CumulativeRotation = (DeltaRotation * Plate.CumulativeRotation).GetNormalized();

		Plate.AngularDisplacementSinceReconcile += FMath::Abs(static_cast<double>(Plate.AngularSpeed));
		MaxAngularDisplacementSinceReconcile = FMath::Max(MaxAngularDisplacementSinceReconcile, Plate.AngularDisplacementSinceReconcile);

		// M6 crust evolution hooks (erosion, dampening, etc.) run here.
	}

	if (SpatialQueryData)
	{
		SpatialQueryData->bCapsDirty = true;
	}
}

bool FTectonicPlanet::ShouldReconcile() const
{
	return MaxAngularDisplacementSinceReconcile > ReconcileDisplacementThreshold;
}

void FTectonicPlanet::ResetDisplacementTracking()
{
	MaxAngularDisplacementSinceReconcile = 0.0;
	for (FPlate& Plate : Plates)
	{
		Plate.AngularDisplacementSinceReconcile = 0.0;
	}
}

void FTectonicPlanet::StepSimulation()
{
	bReconcileTriggeredLastStep = false;
	AdvancePlateMotionStep();

	if (ShouldReconcile())
	{
		Reconcile();
		bReconcileTriggeredLastStep = true;
	}

	++TimestepCounter;
}

void FTectonicPlanet::Reconcile()
{
	if (Plates.Num() == 0 || GetNumSamples() == 0)
	{
		return;
	}

	FReconcilePhaseTimings Timings;
	const double ReconcileStartTime = FPlatformTime::Seconds();

	const int32 ReadBufferIndex = ReadableSampleBufferIndex.Load();
	const int32 WriteBufferIndex = 1 - ReadBufferIndex;
	const TArray<FCanonicalSample>& ReadSamples = SampleBuffers[ReadBufferIndex];
	TArray<FCanonicalSample>& WriteSamples = GetWritableSamplesInternal(WriteBufferIndex);
	WriteSamples = ReadSamples;

	const double Phase1Start = FPlatformTime::Seconds();
	if (!SpatialQueryData)
	{
		SpatialQueryData = MakeUnique<FSpatialQueryData>();
	}

	if (SpatialQueryData->bDirty)
	{
		BuildSpatialQueryDataInternal();
		Timings.Phase1SoupExtractTotalMs = SpatialQueryData->LastSoupExtractTotalMs;
		Timings.Phase1SoupExtractMaxMs = SpatialQueryData->LastSoupExtractMaxMs;
		Timings.Phase1BVHBuildTotalMs = SpatialQueryData->LastBVHBuildTotalMs;
		Timings.Phase1BVHBuildMaxMs = SpatialQueryData->LastBVHBuildMaxMs;
	}

	const double CapUpdateMs = SpatialQueryData->bCapsDirty ? UpdateSpatialCapsForCurrentRotationsInternal() : 0.0;
	Timings.Phase1CapBuildTotalMs = CapUpdateMs;
	Timings.Phase1CapBuildMaxMs = CapUpdateMs;
	Timings.Phase1BuildSpatialMs = (FPlatformTime::Seconds() - Phase1Start) * 1000.0;

	TArray<FVector> PlateCapCenters;
	PlateCapCenters.SetNum(Plates.Num());
	TArray<const FSpatialQueryData::FPlateSpatialState*> SpatialStateByPlateId;
	SpatialStateByPlateId.Init(nullptr, Plates.Num());
	if (SpatialQueryData)
	{
		for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
		{
			if (Plates.IsValidIndex(PlateState.PlateId))
			{
				SpatialStateByPlateId[PlateState.PlateId] = &PlateState;
				if (PlateState.bHasValidCap)
				{
					PlateCapCenters[PlateState.PlateId] = PlateState.CapCenterDirection.GetSafeNormal();
				}
			}
		}
	}

	for (const FPlate& Plate : Plates)
	{
		if (PlateCapCenters[Plate.Id].IsNearlyZero())
		{
			PlateCapCenters[Plate.Id] = Plate.CumulativeRotation.RotateVector(Plate.RotationAxis).GetSafeNormal();
		}
	}

	const auto ComputeMarginFromCapCenters = [this, &PlateCapCenters](const FVector& Direction) -> float
	{
		double BestDot = -TNumericLimits<double>::Max();
		double SecondBestDot = -TNumericLimits<double>::Max();
		for (const FVector& Center : PlateCapCenters)
		{
			const double Dot = FVector::DotProduct(Direction, Center);
			if (Dot > BestDot)
			{
				SecondBestDot = BestDot;
				BestDot = Dot;
			}
			else if (Dot > SecondBestDot)
			{
				SecondBestDot = Dot;
			}
		}

		if (BestDot <= -1.0)
		{
			return 0.0f;
		}

		const double BestChordDistance = FMath::Sqrt(FMath::Max(0.0, 2.0 - 2.0 * BestDot));
		const double SecondChordDistance = (SecondBestDot <= -1.0)
			? BestChordDistance
			: FMath::Sqrt(FMath::Max(0.0, 2.0 - 2.0 * SecondBestDot));
		const double MarginNumerator = FMath::Max(0.0, SecondChordDistance - BestChordDistance);
		return (AverageSampleSpacing > UE_DOUBLE_SMALL_NUMBER)
			? static_cast<float>(MarginNumerator / AverageSampleSpacing)
			: 0.0f;
	};

	const auto FindNearestPlateFromCapCenters = [&PlateCapCenters](const FVector& Direction) -> int32
	{
		double BestDot = -TNumericLimits<double>::Max();
		int32 BestPlateId = 0;
		for (int32 PlateId = 0; PlateId < PlateCapCenters.Num(); ++PlateId)
		{
			const double Dot = FVector::DotProduct(Direction, PlateCapCenters[PlateId]);
			if (Dot > BestDot)
			{
				BestDot = Dot;
				BestPlateId = PlateId;
			}
		}
		return BestPlateId;
	};

	const FSpatialQueryData* LocalSpatialQueryData = SpatialQueryData.Get();

	auto QuerySinglePlateContainmentFromSpatialData =
		[this, &SpatialStateByPlateId](const FVector& Position, const int32 PlateId, FContainmentQueryResult& OutResult) -> bool
	{
		OutResult = FContainmentQueryResult{};
		if (!Plates.IsValidIndex(PlateId) || !SpatialStateByPlateId.IsValidIndex(PlateId))
		{
			return false;
		}

		const FSpatialQueryData::FPlateSpatialState* PlateState = SpatialStateByPlateId[PlateId];
		if (!PlateState || !PlateState->BVH || !PlateState->Adapter)
		{
			return false;
		}

		const FVector LocalDirection = Plates[PlateId].CumulativeRotation.UnrotateVector(Position);
		const FVector3d QueryPointLocal(LocalDirection.X, LocalDirection.Y, LocalDirection.Z);
		FVector3d A;
		FVector3d B;
		FVector3d C;
		int32 LocalTriangleId = INDEX_NONE;
		if (!FindContainingTriangleInBVH(*PlateState->BVH, *PlateState->Adapter, QueryPointLocal, LocalTriangleId, A, B, C))
		{
			return false;
		}

		OutResult.bFoundContainingPlate = true;
		OutResult.bGap = false;
		OutResult.bOverlap = false;
		OutResult.PlateId = PlateId;
		OutResult.TriangleIndex = PlateState->SoupData.GlobalTriangleIndices[LocalTriangleId];
		OutResult.NumContainingPlates = 1;
		OutResult.ContainingPlateIds[0] = PlateId;

		FVector3d Barycentric = ComputePlanarBarycentric(A, B, C, QueryPointLocal);
		Barycentric.X = FMath::Max(0.0, Barycentric.X);
		Barycentric.Y = FMath::Max(0.0, Barycentric.Y);
		Barycentric.Z = FMath::Max(0.0, Barycentric.Z);
		const double BarySum = Barycentric.X + Barycentric.Y + Barycentric.Z;
		if (BarySum > UE_DOUBLE_SMALL_NUMBER)
		{
			Barycentric /= BarySum;
		}
		OutResult.Barycentric = FVector(Barycentric.X, Barycentric.Y, Barycentric.Z);
		return true;
	};

	auto QueryContainmentFromSpatialData = [this, LocalSpatialQueryData](const FVector& Position) -> FContainmentQueryResult
	{
		FContainmentQueryResult Result;
		if (!LocalSpatialQueryData || LocalSpatialQueryData->PlateStates.Num() == 0)
		{
			return Result;
		}

		const FVector QueryDirection = Position;
		double BestContainmentScore = -TNumericLimits<double>::Max();
		for (const FSpatialQueryData::FPlateSpatialState& PlateState : LocalSpatialQueryData->PlateStates)
		{
			if (!PlateState.bHasValidCap)
			{
				continue;
			}

			const double DotToCap = FVector::DotProduct(QueryDirection, PlateState.CapCenterDirection);
			if (DotToCap + 1e-12 < PlateState.CapCosHalfAngle)
			{
				continue;
			}

			++Result.NumCapCandidates;
			if (!PlateState.BVH || !PlateState.Adapter || !Plates.IsValidIndex(PlateState.PlateId))
			{
				continue;
			}

			const FVector LocalDirection = Plates[PlateState.PlateId].CumulativeRotation.UnrotateVector(QueryDirection);
			const FVector3d QueryPointLocal(LocalDirection.X, LocalDirection.Y, LocalDirection.Z);
			FVector3d A;
			FVector3d B;
			FVector3d C;
			int32 LocalTriangleId = INDEX_NONE;
			if (!FindContainingTriangleInBVH(*PlateState.BVH, *PlateState.Adapter, QueryPointLocal, LocalTriangleId, A, B, C))
			{
				continue;
			}

			++Result.NumContainingPlates;
			if (Result.NumContainingPlates <= UE_ARRAY_COUNT(Result.ContainingPlateIds))
			{
				Result.ContainingPlateIds[Result.NumContainingPlates - 1] = PlateState.PlateId;
			}

			if (DotToCap > BestContainmentScore)
			{
				BestContainmentScore = DotToCap;
				Result.PlateId = PlateState.PlateId;
				Result.TriangleIndex = PlateState.SoupData.GlobalTriangleIndices[LocalTriangleId];
				FVector3d Barycentric = ComputePlanarBarycentric(A, B, C, QueryPointLocal);
				Barycentric.X = FMath::Max(0.0, Barycentric.X);
				Barycentric.Y = FMath::Max(0.0, Barycentric.Y);
				Barycentric.Z = FMath::Max(0.0, Barycentric.Z);
				const double BarySum = Barycentric.X + Barycentric.Y + Barycentric.Z;
				if (BarySum > UE_DOUBLE_SMALL_NUMBER)
				{
					Barycentric /= BarySum;
				}
				Result.Barycentric = FVector(Barycentric.X, Barycentric.Y, Barycentric.Z);
			}
		}

		Result.bFoundContainingPlate = Result.NumContainingPlates > 0;
		Result.bGap = !Result.bFoundContainingPlate;
		Result.bOverlap = Result.NumContainingPlates > 1;
		return Result;
	};

	const double Phase2Start = FPlatformTime::Seconds();
	TArray<FPhase2SampleState> Phase2States;
	Phase2States.SetNum(ReadSamples.Num());
	TAtomic<int32> FastPathResolvedSamples(0);
	TAtomic<int32> FullQuerySamples(0);
	ParallelFor(ReadSamples.Num(), [this, &ReadSamples, &Phase2States, &ComputeMarginFromCapCenters, &QueryContainmentFromSpatialData, &QuerySinglePlateContainmentFromSpatialData, &FastPathResolvedSamples, &FullQuerySamples](int32 SampleIndex)
	{
		const FCanonicalSample& SourceSample = ReadSamples[SampleIndex];
		FPhase2SampleState& State = Phase2States[SampleIndex];
		State = FPhase2SampleState{};

		const float Margin = ComputeMarginFromCapCenters(SourceSample.Position);
		State.OwnershipMargin = Margin;
		const bool bBoundaryLike = SourceSample.bIsBoundary || (SourceSample.OwnershipMargin < BoundaryConfidenceThreshold);

		if (Plates.IsValidIndex(SourceSample.PrevPlateId))
		{
			FContainmentQueryResult FastPathResult;
			if (QuerySinglePlateContainmentFromSpatialData(SourceSample.Position, SourceSample.PrevPlateId, FastPathResult))
			{
				State.bGap = false;
				State.bOverlap = false;
				State.TriangleIndex = FastPathResult.TriangleIndex;
				State.Barycentric = FastPathResult.Barycentric;
				State.NumContainingPlates = 1;
				State.ContainingPlateIds[0] = SourceSample.PrevPlateId;
				State.AssignedPlateId = SourceSample.PrevPlateId;
				++FastPathResolvedSamples;
				return;
			}
		}

		++FullQuerySamples;
		const FContainmentQueryResult Containment = QueryContainmentFromSpatialData(SourceSample.Position);
		State.bGap = Containment.bGap;
		State.bOverlap = Containment.bOverlap;
		State.TriangleIndex = Containment.TriangleIndex;
		State.Barycentric = Containment.Barycentric;
		State.NumContainingPlates = Containment.NumContainingPlates;
		for (int32 IdIndex = 0; IdIndex < UE_ARRAY_COUNT(State.ContainingPlateIds); ++IdIndex)
		{
			State.ContainingPlateIds[IdIndex] = Containment.ContainingPlateIds[IdIndex];
		}

		if (!Containment.bFoundContainingPlate)
		{
			State.AssignedPlateId = SourceSample.PlateId;
			return;
		}

		const int32 CandidatePlateId = Containment.PlateId;
		if (!bBoundaryLike)
		{
			State.AssignedPlateId = CandidatePlateId;
			return;
		}

		const int32 PreviousOwner = SourceSample.PrevPlateId;
		if (CandidatePlateId == PreviousOwner)
		{
			State.AssignedPlateId = CandidatePlateId;
			return;
		}

		bool bPreviousOwnerStillContains = false;
		for (int32 IdIndex = 0; IdIndex < UE_ARRAY_COUNT(State.ContainingPlateIds); ++IdIndex)
		{
			if (State.ContainingPlateIds[IdIndex] == PreviousOwner)
			{
				bPreviousOwnerStillContains = true;
				break;
			}
		}

		if (!bPreviousOwnerStillContains)
		{
			State.AssignedPlateId = CandidatePlateId;
			return;
		}

		State.AssignedPlateId = (Margin > HysteresisThreshold) ? CandidatePlateId : PreviousOwner;
	});
	Timings.Phase2FastPathResolvedSamples = FastPathResolvedSamples.Load();
	Timings.Phase2FullQuerySamples = FullQuerySamples.Load();
	Timings.Phase2OwnershipMs = (FPlatformTime::Seconds() - Phase2Start) * 1000.0;

	const double Phase3Start = FPlatformTime::Seconds();
	TArray<TMap<int32, const FCarriedSampleData*>> CarriedLookupByPlate;
	CarriedLookupByPlate.SetNum(Plates.Num());
	for (const FPlate& Plate : Plates)
	{
		if (!CarriedLookupByPlate.IsValidIndex(Plate.Id))
		{
			continue;
		}

		TMap<int32, const FCarriedSampleData*>& Lookup = CarriedLookupByPlate[Plate.Id];
		Lookup.Reserve(Plate.CarriedSamples.Num());
		for (const FCarriedSampleData& Carried : Plate.CarriedSamples)
		{
			Lookup.Add(Carried.CanonicalSampleIndex, &Carried);
		}
	}

	ParallelFor(WriteSamples.Num(), [this, &ReadSamples, &WriteSamples, &Phase2States, &CarriedLookupByPlate](int32 SampleIndex)
	{
		const FCanonicalSample& SourceSample = ReadSamples[SampleIndex];
		FCanonicalSample& DestSample = WriteSamples[SampleIndex];
		const FPhase2SampleState& State = Phase2States[SampleIndex];

		DestSample.PlateId = State.AssignedPlateId;
		DestSample.PrevPlateId = SourceSample.PrevPlateId;
		DestSample.OwnershipMargin = State.OwnershipMargin;
		DestSample.BoundaryNormal = FVector::ZeroVector;
		DestSample.BoundaryType = EBoundaryType::None;
		DestSample.bGapDetected = false;
		DestSample.FlankingPlateIdA = INDEX_NONE;
		DestSample.FlankingPlateIdB = INDEX_NONE;
		DestSample.bOverlapDetected = false;
		DestSample.NumOverlapPlateIds = 0;
		for (int32 OverlapIndex = 0; OverlapIndex < UE_ARRAY_COUNT(DestSample.OverlapPlateIds); ++OverlapIndex)
		{
			DestSample.OverlapPlateIds[OverlapIndex] = INDEX_NONE;
		}

		if (State.bGap || !Plates.IsValidIndex(State.AssignedPlateId) || !Triangles.IsValidIndex(State.TriangleIndex))
		{
			return;
		}

		const FDelaunayTriangle& Triangle = Triangles[State.TriangleIndex];
		const TMap<int32, const FCarriedSampleData*>& Lookup = CarriedLookupByPlate[State.AssignedPlateId];
		const FCarriedSampleData* V0 = Lookup.FindRef(Triangle.V[0]);
		const FCarriedSampleData* V1 = Lookup.FindRef(Triangle.V[1]);
		const FCarriedSampleData* V2 = Lookup.FindRef(Triangle.V[2]);
		if (!V0 || !V1 || !V2)
		{
			return;
		}

		const FVector Bary = State.Barycentric;
		const float W0 = Bary.X;
		const float W1 = Bary.Y;
		const float W2 = Bary.Z;

		DestSample.Elevation = W0 * V0->Elevation + W1 * V1->Elevation + W2 * V2->Elevation;
		DestSample.Thickness = W0 * V0->Thickness + W1 * V1->Thickness + W2 * V2->Thickness;
		DestSample.Age = W0 * V0->Age + W1 * V1->Age + W2 * V2->Age;
		DestSample.OrogenyAge = W0 * V0->OrogenyAge + W1 * V1->OrogenyAge + W2 * V2->OrogenyAge;

		DestSample.CrustType = MajorityCrustType(V0->CrustType, V1->CrustType, V2->CrustType);
		DestSample.OrogenyType = MajorityOrogenyType(V0->OrogenyType, V1->OrogenyType, V2->OrogenyType);
		DestSample.RidgeDirection = ResolveDirectionField(DestSample.Position, V0->RidgeDirection, V1->RidgeDirection, V2->RidgeDirection, Bary);
		DestSample.FoldDirection = ResolveDirectionField(DestSample.Position, V0->FoldDirection, V1->FoldDirection, V2->FoldDirection, Bary);

		const int32 TerraneId0 = ReadSamples[Triangle.V[0]].TerraneId;
		const int32 TerraneId1 = ReadSamples[Triangle.V[1]].TerraneId;
		const int32 TerraneId2 = ReadSamples[Triangle.V[2]].TerraneId;
		if (W0 >= W1 && W0 >= W2)
		{
			DestSample.TerraneId = TerraneId0;
		}
		else if (W1 >= W0 && W1 >= W2)
		{
			DestSample.TerraneId = TerraneId1;
		}
		else
		{
			DestSample.TerraneId = TerraneId2;
		}
	});
	Timings.Phase3InterpolationMs = (FPlatformTime::Seconds() - Phase3Start) * 1000.0;

	const double Phase4Start = FPlatformTime::Seconds();
	int32 GapSamples = 0;
	for (int32 SampleIndex = 0; SampleIndex < WriteSamples.Num(); ++SampleIndex)
	{
		const FPhase2SampleState& State = Phase2States[SampleIndex];
		FCanonicalSample& Sample = WriteSamples[SampleIndex];
		if (!State.bGap)
		{
			continue;
		}

		const int32 NearestPlate = FindNearestPlateFromCapCenters(Sample.Position);
		Sample.PlateId = NearestPlate;
		Sample.CrustType = ECrustType::Oceanic;
		Sample.Elevation = -6.0f;
		Sample.Thickness = 7.0f;
		Sample.Age = 0.0f;
		Sample.bGapDetected = true;
		++GapSamples;
	}
	Timings.GapSamples = GapSamples;
	Timings.Phase4GapMs = (FPlatformTime::Seconds() - Phase4Start) * 1000.0;

	const double Phase5Start = FPlatformTime::Seconds();
	int32 OverlapSamples = 0;
	for (int32 SampleIndex = 0; SampleIndex < WriteSamples.Num(); ++SampleIndex)
	{
		const FPhase2SampleState& State = Phase2States[SampleIndex];
		FCanonicalSample& Sample = WriteSamples[SampleIndex];
		if (!State.bOverlap || State.NumContainingPlates < 2)
		{
			continue;
		}

		Sample.bOverlapDetected = true;
		Sample.NumOverlapPlateIds = static_cast<uint8>(FMath::Clamp(State.NumContainingPlates, 0, UE_ARRAY_COUNT(Sample.OverlapPlateIds)));
		for (int32 OverlapIndex = 0; OverlapIndex < UE_ARRAY_COUNT(Sample.OverlapPlateIds); ++OverlapIndex)
		{
			Sample.OverlapPlateIds[OverlapIndex] = State.ContainingPlateIds[OverlapIndex];
		}
		++OverlapSamples;
	}
	Timings.OverlapSamples = OverlapSamples;
	Timings.Phase5OverlapMs = (FPlatformTime::Seconds() - Phase5Start) * 1000.0;

	const double Phase6Start = FPlatformTime::Seconds();
	const double Phase6SanitizeStart = FPlatformTime::Seconds();
	for (FCanonicalSample& Sample : WriteSamples)
	{
		if (!Plates.IsValidIndex(Sample.PlateId))
		{
			Sample.PlateId = FindNearestPlateFromCapCenters(Sample.Position);
		}
		Sample.PrevPlateId = Sample.PlateId;
	}
	Timings.Phase6SanitizeOwnershipMs = (FPlatformTime::Seconds() - Phase6SanitizeStart) * 1000.0;

	const double Phase6MembershipRebuildStart = FPlatformTime::Seconds();
	RebuildPlateMembershipFromSamples(WriteSamples);
	Timings.Phase6RebuildMembershipMs = (FPlatformTime::Seconds() - Phase6MembershipRebuildStart) * 1000.0;

	const double Phase6CarriedStart = FPlatformTime::Seconds();
	RebuildCarriedSampleWorkspacesForSamples(WriteSamples);
	Timings.Phase6RebuildCarriedMs = (FPlatformTime::Seconds() - Phase6CarriedStart) * 1000.0;

	const double Phase6ClassifyStart = FPlatformTime::Seconds();
	ClassifyTrianglesForSamples(WriteSamples);
	Timings.Phase6ClassifyTrianglesMs = (FPlatformTime::Seconds() - Phase6ClassifyStart) * 1000.0;

	ReadableSampleBufferIndex.Store(WriteBufferIndex);

	const double Phase6SpatialRebuildStart = FPlatformTime::Seconds();
	RebuildSpatialQueryData();
	Timings.Phase6SpatialRebuildMs = (FPlatformTime::Seconds() - Phase6SpatialRebuildStart) * 1000.0;
	if (SpatialQueryData)
	{
		Timings.Phase6SpatialDirtyPlateCount = SpatialQueryData->LastDirtyPlateCount;
		Timings.Phase6SpatialRebuiltPlateCount = SpatialQueryData->LastRebuiltPlateCount;
	}

	Timings.Phase6MembershipMs = (FPlatformTime::Seconds() - Phase6Start) * 1000.0;

	const double Phase7Start = FPlatformTime::Seconds();
	RunTerraneDetectionStub();
	Timings.Phase7TerraneMs = (FPlatformTime::Seconds() - Phase7Start) * 1000.0;

	LastGapSampleCount = GapSamples;
	LastOverlapSampleCount = OverlapSamples;
	LastReconcileTimings = Timings;
	LastReconcileTimings.TotalMs = (FPlatformTime::Seconds() - ReconcileStartTime) * 1000.0;

	++ReconcileCount;
	ResetDisplacementTracking();

	UE_LOG(LogTemp, Log,
		TEXT("Reconcile timings (ms): P1=%.3f P2=%.3f P3=%.3f P4=%.3f P5=%.3f P6=%.3f P7=%.3f Total=%.3f Gap=%d Overlap=%d"),
		LastReconcileTimings.Phase1BuildSpatialMs,
		LastReconcileTimings.Phase2OwnershipMs,
		LastReconcileTimings.Phase3InterpolationMs,
		LastReconcileTimings.Phase4GapMs,
		LastReconcileTimings.Phase5OverlapMs,
		LastReconcileTimings.Phase6MembershipMs,
		LastReconcileTimings.Phase7TerraneMs,
		LastReconcileTimings.TotalMs,
		LastReconcileTimings.GapSamples,
		LastReconcileTimings.OverlapSamples);
	UE_LOG(LogTemp, Log,
		TEXT("Reconcile subphase (ms): P1 Soup sum/max=%.3f/%.3f Cap sum/max=%.3f/%.3f BVH sum/max=%.3f/%.3f | P6 Sanitize=%.3f Membership=%.3f Carried=%.3f Classify=%.3f SpatialRebuild=%.3f"),
		LastReconcileTimings.Phase1SoupExtractTotalMs,
		LastReconcileTimings.Phase1SoupExtractMaxMs,
		LastReconcileTimings.Phase1CapBuildTotalMs,
		LastReconcileTimings.Phase1CapBuildMaxMs,
		LastReconcileTimings.Phase1BVHBuildTotalMs,
		LastReconcileTimings.Phase1BVHBuildMaxMs,
		LastReconcileTimings.Phase6SanitizeOwnershipMs,
		LastReconcileTimings.Phase6RebuildMembershipMs,
		LastReconcileTimings.Phase6RebuildCarriedMs,
		LastReconcileTimings.Phase6ClassifyTrianglesMs,
		LastReconcileTimings.Phase6SpatialRebuildMs);
	UE_LOG(LogTemp, Log,
		TEXT("Reconcile spatial rebuild stats: dirty_plates=%d rebuilt_plates=%d"),
		LastReconcileTimings.Phase6SpatialDirtyPlateCount,
		LastReconcileTimings.Phase6SpatialRebuiltPlateCount);
	UE_LOG(LogTemp, Log,
		TEXT("Reconcile phase2 fast-path: resolved=%d full_query=%d"),
		LastReconcileTimings.Phase2FastPathResolvedSamples,
		LastReconcileTimings.Phase2FullQuerySamples);
}

FVector FTectonicPlanet::GetRotatedSamplePosition(const FPlate& Plate, int32 CanonicalSampleIndex) const
{
	const TArray<FCanonicalSample>& Samples = GetReadableSamplesInternal();
	checkf(Samples.IsValidIndex(CanonicalSampleIndex), TEXT("Invalid canonical sample index %d"), CanonicalSampleIndex);
	return Plate.GetRotatedCanonicalPosition(Samples[CanonicalSampleIndex].Position);
}

FVector FTectonicPlanet::ComputeSurfaceVelocity(int32 PlateId, const FVector& Position) const
{
	if (!Plates.IsValidIndex(PlateId))
	{
		return FVector::ZeroVector;
	}

	const FPlate& Plate = Plates[PlateId];
	const FVector AngularVelocity = Plate.RotationAxis.GetSafeNormal() * static_cast<double>(Plate.AngularSpeed);
	return FVector::CrossProduct(AngularVelocity, Position);
}

void FTectonicPlanet::RebuildSpatialQueryData()
{
	BuildSpatialQueryDataInternal();
	UpdateSpatialCapsForCurrentRotationsInternal();
}

FContainmentQueryResult FTectonicPlanet::QueryContainment(const FVector& Position) const
{
	FContainmentQueryResult Result;
	const TArray<FCanonicalSample>& Samples = GetReadableSamplesInternal();
	if (Plates.Num() == 0 || Samples.Num() == 0 || Triangles.Num() == 0)
	{
		return Result;
	}

	const FVector QueryDirection = Position.GetSafeNormal();
	if (QueryDirection.IsNearlyZero())
	{
		return Result;
	}

	EnsureSpatialQueryDataBuilt();
	if (!SpatialQueryData)
	{
		return Result;
	}

	double BestContainmentScore = -TNumericLimits<double>::Max();
	for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
	{
		if (!PlateState.bHasValidCap)
		{
			continue;
		}

		const double DotToCap = FVector::DotProduct(QueryDirection, PlateState.CapCenterDirection);
		if (DotToCap + 1e-12 < PlateState.CapCosHalfAngle)
		{
			continue;
		}

		++Result.NumCapCandidates;
		if (!PlateState.BVH || !PlateState.Adapter)
		{
			continue;
		}

		if (!Plates.IsValidIndex(PlateState.PlateId))
		{
			continue;
		}

		const FVector LocalDirection = Plates[PlateState.PlateId].CumulativeRotation.UnrotateVector(QueryDirection);
		const FVector3d QueryPointLocal(LocalDirection.X, LocalDirection.Y, LocalDirection.Z);
		FVector3d A;
		FVector3d B;
		FVector3d C;
		int32 LocalTriangleId = INDEX_NONE;
		if (!FindContainingTriangleInBVH(*PlateState.BVH, *PlateState.Adapter, QueryPointLocal, LocalTriangleId, A, B, C))
		{
			continue;
		}

		++Result.NumContainingPlates;
		if (Result.NumContainingPlates <= UE_ARRAY_COUNT(Result.ContainingPlateIds))
		{
			Result.ContainingPlateIds[Result.NumContainingPlates - 1] = PlateState.PlateId;
		}

		if (DotToCap > BestContainmentScore)
		{
			BestContainmentScore = DotToCap;
			Result.PlateId = PlateState.PlateId;
			Result.TriangleIndex = PlateState.SoupData.GlobalTriangleIndices[LocalTriangleId];
			FVector3d Barycentric = ComputePlanarBarycentric(A, B, C, QueryPointLocal);
			Barycentric.X = FMath::Max(0.0, Barycentric.X);
			Barycentric.Y = FMath::Max(0.0, Barycentric.Y);
			Barycentric.Z = FMath::Max(0.0, Barycentric.Z);
			const double BarySum = Barycentric.X + Barycentric.Y + Barycentric.Z;
			if (BarySum > UE_DOUBLE_SMALL_NUMBER)
			{
				Barycentric /= BarySum;
			}
			Result.Barycentric = FVector(Barycentric.X, Barycentric.Y, Barycentric.Z);
		}
	}

	Result.bFoundContainingPlate = Result.NumContainingPlates > 0;
	Result.bGap = !Result.bFoundContainingPlate;
	Result.bOverlap = Result.NumContainingPlates > 1;
	return Result;
}

bool FTectonicPlanet::GetPlateSoupTriangleAndVertexCounts(int32 PlateId, int32& OutTriangleCount, int32& OutVertexCount) const
{
	OutTriangleCount = 0;
	OutVertexCount = 0;

	EnsureSpatialQueryDataBuilt();
	if (!SpatialQueryData)
	{
		return false;
	}

	for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
	{
		if (PlateState.PlateId == PlateId)
		{
			OutTriangleCount = PlateState.SoupData.LocalTriangles.Num();
			OutVertexCount = PlateState.SoupData.RotatedVertices.Num();
			return true;
		}
	}

	return false;
}

bool FTectonicPlanet::GetPlateSoupRotatedVertex(int32 PlateId, int32 CanonicalSampleIndex, FVector& OutRotatedPosition) const
{
	OutRotatedPosition = FVector::ZeroVector;

	EnsureSpatialQueryDataBuilt();
	if (!SpatialQueryData)
	{
		return false;
	}

	for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
	{
		if (PlateState.PlateId != PlateId)
		{
			continue;
		}

		const int32* LocalVertexIndex = PlateState.SoupData.CanonicalToLocalVertex.Find(CanonicalSampleIndex);
		if (!LocalVertexIndex || !PlateState.SoupData.RotatedVertices.IsValidIndex(*LocalVertexIndex))
		{
			return false;
		}

		if (!Plates.IsValidIndex(PlateId))
		{
			return false;
		}

		const FVector3d Canonical = PlateState.SoupData.RotatedVertices[*LocalVertexIndex];
		OutRotatedPosition = Plates[PlateId].CumulativeRotation.RotateVector(FVector(Canonical.X, Canonical.Y, Canonical.Z));
		return true;
	}

	return false;
}

bool FTectonicPlanet::GetPlateBoundingCap(int32 PlateId, FVector& OutCenterDirection, double& OutCosHalfAngle) const
{
	OutCenterDirection = FVector::ZeroVector;
	OutCosHalfAngle = -1.0;

	EnsureSpatialQueryDataBuilt();
	if (!SpatialQueryData)
	{
		return false;
	}

	for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
	{
		if (PlateState.PlateId == PlateId && PlateState.bHasValidCap)
		{
			OutCenterDirection = PlateState.CapCenterDirection;
			OutCosHalfAngle = PlateState.CapCosHalfAngle;
			return true;
		}
	}

	return false;
}

const TArray<FCanonicalSample>& FTectonicPlanet::GetReadableSamplesInternal() const
{
	return SampleBuffers[ReadableSampleBufferIndex.Load()];
}

TArray<FCanonicalSample>& FTectonicPlanet::GetWritableSamplesInternal(int32 WriteBufferIndex)
{
	check(WriteBufferIndex == 0 || WriteBufferIndex == 1);
	return SampleBuffers[WriteBufferIndex];
}

void FTectonicPlanet::ClassifyTrianglesForSamples(TArray<FCanonicalSample>& InOutSamples)
{
	const int32 NumPlates = Plates.Num();
	const int32 NumTriangles = Triangles.Num();
	if (NumPlates == 0 || NumTriangles == 0)
	{
		for (FPlate& Plate : Plates)
		{
			Plate.InteriorTriangles.Reset();
			Plate.BoundaryTriangles.Reset();
		}
		for (FCanonicalSample& Sample : InOutSamples)
		{
			Sample.bIsBoundary = false;
			Sample.BoundaryNormal = FVector::ZeroVector;
			Sample.BoundaryType = EBoundaryType::None;
			Sample.FlankingPlateIdA = INDEX_NONE;
			Sample.FlankingPlateIdB = INDEX_NONE;
		}
		BoundarySampleCount = 0;
		InvalidateSpatialQueryData();
		return;
	}

	TArray<uint64> PreviousInteriorHashes;
	TArray<int32> PreviousInteriorCounts;
	PreviousInteriorHashes.SetNumZeroed(NumPlates);
	PreviousInteriorCounts.SetNumZeroed(NumPlates);
	for (int32 PlateIndex = 0; PlateIndex < NumPlates; ++PlateIndex)
	{
		PreviousInteriorCounts[PlateIndex] = Plates[PlateIndex].InteriorTriangles.Num();
		PreviousInteriorHashes[PlateIndex] = HashTriangleIndexList(Plates[PlateIndex].InteriorTriangles);
	}

	struct FChunkTriangleBins
	{
		TArray<TArray<int32>> InteriorByPlate;
		TArray<TArray<int32>> BoundaryByPlate;
		TArray<int32> BoundaryVertices;
	};

	const int32 NumWorkers = FMath::Max(1, FTaskGraphInterface::Get().GetNumWorkerThreads() + 1);
	const int32 NumChunks = FMath::Clamp(NumWorkers, 1, NumTriangles);
	const int32 ChunkSize = FMath::DivideAndRoundUp(NumTriangles, NumChunks);

	TArray<FChunkTriangleBins> ChunkBins;
	ChunkBins.SetNum(NumChunks);
	ParallelFor(NumChunks, [this, &InOutSamples, &ChunkBins, NumPlates, NumTriangles, ChunkSize](int32 ChunkIndex)
	{
		const int32 StartTriangle = ChunkIndex * ChunkSize;
		const int32 EndTriangle = FMath::Min(StartTriangle + ChunkSize, NumTriangles);
		if (StartTriangle >= EndTriangle)
		{
			return;
		}

		FChunkTriangleBins& Bins = ChunkBins[ChunkIndex];
		Bins.InteriorByPlate.SetNum(NumPlates);
		Bins.BoundaryByPlate.SetNum(NumPlates);
		const int32 ChunkTriangleCount = EndTriangle - StartTriangle;
		const int32 EstimatedInteriorPerPlate = FMath::Max(1, ChunkTriangleCount / NumPlates);
		const int32 EstimatedBoundaryPerPlate = FMath::Max(1, (ChunkTriangleCount * 2) / NumPlates);
		for (int32 PlateIndex = 0; PlateIndex < NumPlates; ++PlateIndex)
		{
			Bins.InteriorByPlate[PlateIndex].Reserve(EstimatedInteriorPerPlate);
			Bins.BoundaryByPlate[PlateIndex].Reserve(EstimatedBoundaryPerPlate);
		}
		Bins.BoundaryVertices.Reserve(ChunkTriangleCount * 3);

		for (int32 TriangleIndex = StartTriangle; TriangleIndex < EndTriangle; ++TriangleIndex)
		{
			const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
			const int32 P0 = InOutSamples[Triangle.V[0]].PlateId;
			const int32 P1 = InOutSamples[Triangle.V[1]].PlateId;
			const int32 P2 = InOutSamples[Triangle.V[2]].PlateId;
			checkSlow(Plates.IsValidIndex(P0));
			checkSlow(Plates.IsValidIndex(P1));
			checkSlow(Plates.IsValidIndex(P2));

			if (P0 == P1 && P1 == P2)
			{
				Bins.InteriorByPlate[P0].Add(TriangleIndex);
				continue;
			}

			if (P0 == P1)
			{
				Bins.BoundaryByPlate[P0].Add(TriangleIndex);
				Bins.BoundaryByPlate[P2].Add(TriangleIndex);
			}
			else if (P1 == P2)
			{
				Bins.BoundaryByPlate[P1].Add(TriangleIndex);
				Bins.BoundaryByPlate[P0].Add(TriangleIndex);
			}
			else if (P0 == P2)
			{
				Bins.BoundaryByPlate[P0].Add(TriangleIndex);
				Bins.BoundaryByPlate[P1].Add(TriangleIndex);
			}
			else
			{
				Bins.BoundaryByPlate[P0].Add(TriangleIndex);
				Bins.BoundaryByPlate[P1].Add(TriangleIndex);
				Bins.BoundaryByPlate[P2].Add(TriangleIndex);
			}

			Bins.BoundaryVertices.Add(Triangle.V[0]);
			Bins.BoundaryVertices.Add(Triangle.V[1]);
			Bins.BoundaryVertices.Add(Triangle.V[2]);
		}
	}, EParallelForFlags::Unbalanced);

	TArray<int32> InteriorCounts;
	TArray<int32> BoundaryCounts;
	InteriorCounts.Init(0, NumPlates);
	BoundaryCounts.Init(0, NumPlates);
	for (const FChunkTriangleBins& Bins : ChunkBins)
	{
		for (int32 PlateIndex = 0; PlateIndex < NumPlates; ++PlateIndex)
		{
			InteriorCounts[PlateIndex] += Bins.InteriorByPlate.IsValidIndex(PlateIndex) ? Bins.InteriorByPlate[PlateIndex].Num() : 0;
			BoundaryCounts[PlateIndex] += Bins.BoundaryByPlate.IsValidIndex(PlateIndex) ? Bins.BoundaryByPlate[PlateIndex].Num() : 0;
		}
	}

	for (int32 PlateIndex = 0; PlateIndex < NumPlates; ++PlateIndex)
	{
		Plates[PlateIndex].InteriorTriangles.SetNumUninitialized(InteriorCounts[PlateIndex], EAllowShrinking::No);
		Plates[PlateIndex].BoundaryTriangles.SetNumUninitialized(BoundaryCounts[PlateIndex], EAllowShrinking::No);
	}

	TArray<int32> InteriorWriteCursors;
	TArray<int32> BoundaryWriteCursors;
	InteriorWriteCursors.Init(0, NumPlates);
	BoundaryWriteCursors.Init(0, NumPlates);
	for (const FChunkTriangleBins& Bins : ChunkBins)
	{
		for (int32 PlateIndex = 0; PlateIndex < NumPlates; ++PlateIndex)
		{
			const TArray<int32>& InteriorLocal = Bins.InteriorByPlate[PlateIndex];
			const int32 InteriorCount = InteriorLocal.Num();
			if (InteriorCount > 0)
			{
				int32* DestPtr = Plates[PlateIndex].InteriorTriangles.GetData() + InteriorWriteCursors[PlateIndex];
				FMemory::Memcpy(DestPtr, InteriorLocal.GetData(), sizeof(int32) * InteriorCount);
				InteriorWriteCursors[PlateIndex] += InteriorCount;
			}

			const TArray<int32>& BoundaryLocal = Bins.BoundaryByPlate[PlateIndex];
			const int32 BoundaryCount = BoundaryLocal.Num();
			if (BoundaryCount > 0)
			{
				int32* DestPtr = Plates[PlateIndex].BoundaryTriangles.GetData() + BoundaryWriteCursors[PlateIndex];
				FMemory::Memcpy(DestPtr, BoundaryLocal.GetData(), sizeof(int32) * BoundaryCount);
				BoundaryWriteCursors[PlateIndex] += BoundaryCount;
			}
		}
	}

	for (FCanonicalSample& Sample : InOutSamples)
	{
		Sample.bIsBoundary = false;
	}
	for (const FChunkTriangleBins& Bins : ChunkBins)
	{
		for (const int32 BoundaryVertex : Bins.BoundaryVertices)
		{
			if (InOutSamples.IsValidIndex(BoundaryVertex))
			{
				InOutSamples[BoundaryVertex].bIsBoundary = true;
			}
		}
	}

	int32 LocalBoundarySampleCount = 0;
	for (const FCanonicalSample& Sample : InOutSamples)
	{
		LocalBoundarySampleCount += Sample.bIsBoundary ? 1 : 0;
	}
	BoundarySampleCount = LocalBoundarySampleCount;
	UpdateBoundaryClassificationForSamples(InOutSamples);
	UpdateGapFlankingPlatesForSamples(InOutSamples);

	if (!SpatialQueryData)
	{
		SpatialQueryData = MakeUnique<FSpatialQueryData>();
	}

	bool bAnyDirtyPlate = false;
	if (SpatialQueryData->PlateStates.Num() != NumPlates)
	{
		SpatialQueryData->DirtyPlateFlags.Init(1, NumPlates);
		bAnyDirtyPlate = NumPlates > 0;
	}
	else
	{
		if (SpatialQueryData->DirtyPlateFlags.Num() != NumPlates)
		{
			SpatialQueryData->DirtyPlateFlags.Init(0, NumPlates);
		}

		for (int32 PlateIndex = 0; PlateIndex < NumPlates; ++PlateIndex)
		{
			const int32 NewCount = Plates[PlateIndex].InteriorTriangles.Num();
			const uint64 NewHash = HashTriangleIndexList(Plates[PlateIndex].InteriorTriangles);
			const bool bInteriorChanged = (NewCount != PreviousInteriorCounts[PlateIndex]) || (NewHash != PreviousInteriorHashes[PlateIndex]);
			if (bInteriorChanged)
			{
				SpatialQueryData->DirtyPlateFlags[PlateIndex] = 1;
			}

			if (SpatialQueryData->DirtyPlateFlags[PlateIndex] != 0)
			{
				bAnyDirtyPlate = true;
			}
		}
	}

	SpatialQueryData->bDirty = bAnyDirtyPlate;
	SpatialQueryData->bCapsDirty = true;
}

void FTectonicPlanet::UpdateBoundaryFlagsForSamples(TArray<FCanonicalSample>& InOutSamples)
{
	if (Adjacency.Num() != InOutSamples.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("UpdateBoundaryFlags skipped: adjacency (%d) does not match samples (%d)."), Adjacency.Num(), InOutSamples.Num());
		return;
	}

	int32 LocalBoundarySampleCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
	{
		bool bBoundary = false;
		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InOutSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			if (InOutSamples[NeighborIndex].PlateId != InOutSamples[SampleIndex].PlateId)
			{
				bBoundary = true;
				break;
			}
		}

		InOutSamples[SampleIndex].bIsBoundary = bBoundary;
		LocalBoundarySampleCount += bBoundary ? 1 : 0;
	}

	BoundarySampleCount = LocalBoundarySampleCount;
	UpdateBoundaryClassificationForSamples(InOutSamples);
	UpdateGapFlankingPlatesForSamples(InOutSamples);
}

void FTectonicPlanet::UpdateBoundaryClassificationForSamples(TArray<FCanonicalSample>& InOutSamples)
{
	if (Adjacency.Num() != InOutSamples.Num())
	{
		for (FCanonicalSample& Sample : InOutSamples)
		{
			Sample.BoundaryNormal = FVector::ZeroVector;
			Sample.BoundaryType = EBoundaryType::None;
		}
		return;
	}

	constexpr int32 MaxNeighborPlateBins = 16;
	constexpr double TransformThresholdRatio = 0.3;
	constexpr double MinRelativeSpeed = 1e-12;
	for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
	{
		FCanonicalSample& Sample = InOutSamples[SampleIndex];
		Sample.BoundaryNormal = FVector::ZeroVector;
		Sample.BoundaryType = EBoundaryType::None;
		if (!Sample.bIsBoundary || !Plates.IsValidIndex(Sample.PlateId))
		{
			continue;
		}

		const FVector Position = Sample.Position.GetSafeNormal();
		if (Position.IsNearlyZero())
		{
			continue;
		}

		FVector MeanCrossDirection = FVector::ZeroVector;
		FVector FallbackCrossDirection = FVector::ZeroVector;
		int32 NeighborPlateIds[MaxNeighborPlateBins];
		int32 NeighborPlateCounts[MaxNeighborPlateBins];
		int32 NumNeighborPlates = 0;
		for (int32 NeighborSlot = 0; NeighborSlot < MaxNeighborPlateBins; ++NeighborSlot)
		{
			NeighborPlateIds[NeighborSlot] = INDEX_NONE;
			NeighborPlateCounts[NeighborSlot] = 0;
		}

		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InOutSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const FCanonicalSample& NeighborSample = InOutSamples[NeighborIndex];
			const int32 NeighborPlateId = NeighborSample.PlateId;
			if (!Plates.IsValidIndex(NeighborPlateId) || NeighborPlateId == Sample.PlateId)
			{
				continue;
			}

			const FVector DirectionToCrossPlate = (NeighborSample.Position - Sample.Position).GetSafeNormal();
			if (!DirectionToCrossPlate.IsNearlyZero())
			{
				MeanCrossDirection += DirectionToCrossPlate;
				if (FallbackCrossDirection.IsNearlyZero())
				{
					FallbackCrossDirection = DirectionToCrossPlate;
				}
			}

			int32 ExistingSlot = INDEX_NONE;
			for (int32 SlotIndex = 0; SlotIndex < NumNeighborPlates; ++SlotIndex)
			{
				if (NeighborPlateIds[SlotIndex] == NeighborPlateId)
				{
					ExistingSlot = SlotIndex;
					break;
				}
			}

			if (ExistingSlot != INDEX_NONE)
			{
				++NeighborPlateCounts[ExistingSlot];
			}
			else if (NumNeighborPlates < MaxNeighborPlateBins)
			{
				NeighborPlateIds[NumNeighborPlates] = NeighborPlateId;
				NeighborPlateCounts[NumNeighborPlates] = 1;
				++NumNeighborPlates;
			}
		}

		if (NumNeighborPlates == 0)
		{
			continue;
		}

		FVector ProjectedBoundaryNormal = MeanCrossDirection - FVector::DotProduct(MeanCrossDirection, Position) * Position;
		if (ProjectedBoundaryNormal.IsNearlyZero())
		{
			ProjectedBoundaryNormal = FallbackCrossDirection - FVector::DotProduct(FallbackCrossDirection, Position) * Position;
		}
		if (ProjectedBoundaryNormal.IsNearlyZero())
		{
			continue;
		}

		ProjectedBoundaryNormal.Normalize();
		Sample.BoundaryNormal = ProjectedBoundaryNormal;

		int32 DominantPlateId = NeighborPlateIds[0];
		int32 DominantCount = NeighborPlateCounts[0];
		for (int32 SlotIndex = 1; SlotIndex < NumNeighborPlates; ++SlotIndex)
		{
			const int32 PlateId = NeighborPlateIds[SlotIndex];
			const int32 Count = NeighborPlateCounts[SlotIndex];
			if (Count > DominantCount || (Count == DominantCount && PlateId < DominantPlateId))
			{
				DominantPlateId = PlateId;
				DominantCount = Count;
			}
		}

		const FVector RelativeVelocity = ComputeSurfaceVelocity(Sample.PlateId, Position) - ComputeSurfaceVelocity(DominantPlateId, Position);
		const double RelativeSpeed = RelativeVelocity.Size();
		if (RelativeSpeed <= MinRelativeSpeed)
		{
			Sample.BoundaryType = EBoundaryType::Transform;
			continue;
		}

		const double NormalComponent = FVector::DotProduct(RelativeVelocity, ProjectedBoundaryNormal);
		if (FMath::Abs(NormalComponent) < (TransformThresholdRatio * RelativeSpeed))
		{
			Sample.BoundaryType = EBoundaryType::Transform;
		}
		else
		{
			Sample.BoundaryType = (NormalComponent < 0.0) ? EBoundaryType::Convergent : EBoundaryType::Divergent;
		}
	}
}

void FTectonicPlanet::UpdateGapFlankingPlatesForSamples(TArray<FCanonicalSample>& InOutSamples)
{
	if (Adjacency.Num() != InOutSamples.Num())
	{
		for (FCanonicalSample& Sample : InOutSamples)
		{
			Sample.FlankingPlateIdA = INDEX_NONE;
			Sample.FlankingPlateIdB = INDEX_NONE;
		}
		return;
	}

	constexpr int32 MaxNeighborPlateBins = 16;
	for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
	{
		FCanonicalSample& Sample = InOutSamples[SampleIndex];
		Sample.FlankingPlateIdA = INDEX_NONE;
		Sample.FlankingPlateIdB = INDEX_NONE;
		if (!Sample.bGapDetected)
		{
			continue;
		}

		int32 NeighborPlateIds[MaxNeighborPlateBins];
		int32 NeighborPlateCounts[MaxNeighborPlateBins];
		int32 NumNeighborPlates = 0;
		for (int32 NeighborSlot = 0; NeighborSlot < MaxNeighborPlateBins; ++NeighborSlot)
		{
			NeighborPlateIds[NeighborSlot] = INDEX_NONE;
			NeighborPlateCounts[NeighborSlot] = 0;
		}

		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InOutSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = InOutSamples[NeighborIndex].PlateId;
			if (!Plates.IsValidIndex(NeighborPlateId))
			{
				continue;
			}

			int32 ExistingSlot = INDEX_NONE;
			for (int32 SlotIndex = 0; SlotIndex < NumNeighborPlates; ++SlotIndex)
			{
				if (NeighborPlateIds[SlotIndex] == NeighborPlateId)
				{
					ExistingSlot = SlotIndex;
					break;
				}
			}

			if (ExistingSlot != INDEX_NONE)
			{
				++NeighborPlateCounts[ExistingSlot];
			}
			else if (NumNeighborPlates < MaxNeighborPlateBins)
			{
				NeighborPlateIds[NumNeighborPlates] = NeighborPlateId;
				NeighborPlateCounts[NumNeighborPlates] = 1;
				++NumNeighborPlates;
			}
		}

		if (NumNeighborPlates <= 0)
		{
			continue;
		}

		int32 BestIndexA = INDEX_NONE;
		int32 BestIndexB = INDEX_NONE;
		for (int32 SlotIndex = 0; SlotIndex < NumNeighborPlates; ++SlotIndex)
		{
			if (BestIndexA == INDEX_NONE ||
				NeighborPlateCounts[SlotIndex] > NeighborPlateCounts[BestIndexA] ||
				(NeighborPlateCounts[SlotIndex] == NeighborPlateCounts[BestIndexA] && NeighborPlateIds[SlotIndex] < NeighborPlateIds[BestIndexA]))
			{
				BestIndexB = BestIndexA;
				BestIndexA = SlotIndex;
				continue;
			}

			if (BestIndexB == INDEX_NONE ||
				NeighborPlateCounts[SlotIndex] > NeighborPlateCounts[BestIndexB] ||
				(NeighborPlateCounts[SlotIndex] == NeighborPlateCounts[BestIndexB] && NeighborPlateIds[SlotIndex] < NeighborPlateIds[BestIndexB]))
			{
				BestIndexB = SlotIndex;
			}
		}

		if (BestIndexA != INDEX_NONE)
		{
			Sample.FlankingPlateIdA = NeighborPlateIds[BestIndexA];
		}
		if (BestIndexB != INDEX_NONE)
		{
			Sample.FlankingPlateIdB = NeighborPlateIds[BestIndexB];
		}
	}
}

void FTectonicPlanet::RebuildPlateMembershipFromSamples(const TArray<FCanonicalSample>& InSamples)
{
	TArray<int32> PlateSampleCounts;
	PlateSampleCounts.Init(0, Plates.Num());
	for (const FCanonicalSample& Sample : InSamples)
	{
		if (Plates.IsValidIndex(Sample.PlateId))
		{
			++PlateSampleCounts[Sample.PlateId];
		}
	}

	for (FPlate& Plate : Plates)
	{
		Plate.SampleIndices.SetNumUninitialized(PlateSampleCounts[Plate.Id], EAllowShrinking::No);
	}
	TArray<int32> PlateWriteCursor;
	PlateWriteCursor.Init(0, Plates.Num());

	for (int32 SampleIndex = 0; SampleIndex < InSamples.Num(); ++SampleIndex)
	{
		const int32 PlateId = InSamples[SampleIndex].PlateId;
		if (Plates.IsValidIndex(PlateId))
		{
			const int32 WriteIndex = PlateWriteCursor[PlateId]++;
			Plates[PlateId].SampleIndices[WriteIndex] = SampleIndex;
		}
	}

	for (const FPlate& Plate : Plates)
	{
		if (Plate.SampleIndices.Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("Plate %d is empty after membership rebuild."), Plate.Id);
		}
	}
}

void FTectonicPlanet::RebuildCarriedSampleWorkspaces()
{
	RebuildCarriedSampleWorkspacesForSamples(GetReadableSamplesInternal());
}

void FTectonicPlanet::RebuildCarriedSampleWorkspacesForSamples(const TArray<FCanonicalSample>& InSamples)
{
	ParallelFor(Plates.Num(), [this, &InSamples](int32 PlateIndex)
	{
		FPlate& Plate = Plates[PlateIndex];
		const int32 NumPlateSamples = Plate.SampleIndices.Num();
		Plate.CarriedSamples.SetNumUninitialized(NumPlateSamples, EAllowShrinking::No);

		for (int32 LocalIndex = 0; LocalIndex < NumPlateSamples; ++LocalIndex)
		{
			const int32 SampleIndex = Plate.SampleIndices[LocalIndex];
			checkSlow(InSamples.IsValidIndex(SampleIndex));
			const FCanonicalSample& Sample = InSamples[SampleIndex];
			FCarriedSampleData& Carried = Plate.CarriedSamples[LocalIndex];
			Carried.CanonicalSampleIndex = SampleIndex;
			Carried.Elevation = Sample.Elevation;
			Carried.Thickness = Sample.Thickness;
			Carried.Age = Sample.Age;
			Carried.RidgeDirection = Sample.RidgeDirection;
			Carried.FoldDirection = Sample.FoldDirection;
			Carried.OrogenyType = Sample.OrogenyType;
			Carried.OrogenyAge = Sample.OrogenyAge;
			Carried.CrustType = Sample.CrustType;
		}
	}, EParallelForFlags::Unbalanced);
}

void FTectonicPlanet::InvalidateSpatialQueryData()
{
	if (!SpatialQueryData)
	{
		SpatialQueryData = MakeUnique<FSpatialQueryData>();
	}

	SpatialQueryData->bDirty = true;
	SpatialQueryData->bCapsDirty = true;
	SpatialQueryData->DirtyPlateFlags.Init(1, Plates.Num());
	SpatialQueryData->LastDirtyPlateCount = Plates.Num();
	SpatialQueryData->LastRebuiltPlateCount = 0;
}

void FTectonicPlanet::BuildSpatialQueryDataInternal() const
{
	if (!SpatialQueryData)
	{
		SpatialQueryData = MakeUnique<FSpatialQueryData>();
	}

	const TArray<FCanonicalSample>& Samples = GetReadableSamplesInternal();
	if (SpatialQueryData->PlateStates.Num() != Plates.Num())
	{
		SpatialQueryData->PlateStates.SetNum(Plates.Num());
		SpatialQueryData->DirtyPlateFlags.Init(1, Plates.Num());
		SpatialQueryData->bDirty = true;
	}
	else if (SpatialQueryData->DirtyPlateFlags.Num() != Plates.Num())
	{
		SpatialQueryData->DirtyPlateFlags.Init(1, Plates.Num());
		SpatialQueryData->bDirty = true;
	}

	TArray<int32> PlatesToRebuild;
	PlatesToRebuild.Reserve(Plates.Num());
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		const FPlate& Plate = Plates[PlateIndex];
		const FSpatialQueryData::FPlateSpatialState& ExistingState = SpatialQueryData->PlateStates[PlateIndex];
		const bool bHasInteriorTriangles = Plate.InteriorTriangles.Num() > 0;
		const bool bMissingState = ExistingState.PlateId != Plate.Id;
		const bool bMissingSpatial = bHasInteriorTriangles && (!ExistingState.Adapter || !ExistingState.BVH);
		const bool bMarkedDirty = SpatialQueryData->DirtyPlateFlags[PlateIndex] != 0;
		if (bMarkedDirty || bMissingState || bMissingSpatial)
		{
			PlatesToRebuild.Add(PlateIndex);
		}
	}

	SpatialQueryData->LastDirtyPlateCount = PlatesToRebuild.Num();
	if (PlatesToRebuild.Num() == 0)
	{
		SpatialQueryData->bDirty = false;
		SpatialQueryData->LastRebuiltPlateCount = 0;
		SpatialQueryData->LastSoupExtractTotalMs = 0.0;
		SpatialQueryData->LastSoupExtractMaxMs = 0.0;
		SpatialQueryData->LastCapBuildTotalMs = 0.0;
		SpatialQueryData->LastCapBuildMaxMs = 0.0;
		SpatialQueryData->LastBVHBuildTotalMs = 0.0;
		SpatialQueryData->LastBVHBuildMaxMs = 0.0;
		return;
	}

	TArray<double> SoupExtractMsPerPlate;
	TArray<double> CapBuildMsPerPlate;
	TArray<double> BVHBuildMsPerPlate;
	SoupExtractMsPerPlate.Init(0.0, Plates.Num());
	CapBuildMsPerPlate.Init(0.0, Plates.Num());
	BVHBuildMsPerPlate.Init(0.0, Plates.Num());

	ParallelFor(PlatesToRebuild.Num(), [this, &Samples, &PlatesToRebuild, &SoupExtractMsPerPlate, &CapBuildMsPerPlate, &BVHBuildMsPerPlate](int32 WorkIndex)
	{
		const int32 PlateIndex = PlatesToRebuild[WorkIndex];
		const double SoupStart = FPlatformTime::Seconds();
		const FPlate& Plate = Plates[PlateIndex];
		FSpatialQueryData::FPlateSpatialState& PlateState = SpatialQueryData->PlateStates[PlateIndex];
		PlateState = FSpatialQueryData::FPlateSpatialState{};
		PlateState.PlateId = Plate.Id;
		PlateState.SoupData.PlateId = Plate.Id;
		PlateState.SoupData.ChangeStamp = static_cast<uint64>(ReconcileCount) + 1;

		PlateState.SoupData.GlobalTriangleIndices.Reserve(Plate.InteriorTriangles.Num());
		PlateState.SoupData.LocalTriangles.Reserve(Plate.InteriorTriangles.Num());
		PlateState.SoupData.CanonicalToLocalVertex.Reserve(Plate.InteriorTriangles.Num() * 2);
		PlateState.SoupData.LocalToCanonicalVertex.Reserve(Plate.InteriorTriangles.Num() * 2);
		PlateState.SoupData.RotatedVertices.Reserve(Plate.InteriorTriangles.Num() * 2);

		for (const int32 TriangleIndex : Plate.InteriorTriangles)
		{
			checkSlow(Triangles.IsValidIndex(TriangleIndex));
			const FDelaunayTriangle& GlobalTriangle = Triangles[TriangleIndex];
			int32 LocalVertices[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };

			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				const int32 CanonicalSampleIndex = GlobalTriangle.V[Corner];
				checkSlow(Samples.IsValidIndex(CanonicalSampleIndex));

				const int32* ExistingLocalVertex = PlateState.SoupData.CanonicalToLocalVertex.Find(CanonicalSampleIndex);
				if (ExistingLocalVertex)
				{
					LocalVertices[Corner] = *ExistingLocalVertex;
					continue;
				}

				const int32 NewLocalVertex = PlateState.SoupData.RotatedVertices.Num();
				PlateState.SoupData.CanonicalToLocalVertex.Add(CanonicalSampleIndex, NewLocalVertex);
				PlateState.SoupData.LocalToCanonicalVertex.Add(CanonicalSampleIndex);
				const FVector& CanonicalPosition = Samples[CanonicalSampleIndex].Position;
				PlateState.SoupData.RotatedVertices.Add(FVector3d(CanonicalPosition.X, CanonicalPosition.Y, CanonicalPosition.Z));
				LocalVertices[Corner] = NewLocalVertex;
			}

			if (LocalVertices[0] == LocalVertices[1] ||
				LocalVertices[1] == LocalVertices[2] ||
				LocalVertices[0] == LocalVertices[2])
			{
				continue;
			}

			PlateState.SoupData.GlobalTriangleIndices.Add(TriangleIndex);
			PlateState.SoupData.LocalTriangles.Add(UE::Geometry::FIndex3i(LocalVertices[0], LocalVertices[1], LocalVertices[2]));
		}
		SoupExtractMsPerPlate[PlateIndex] = (FPlatformTime::Seconds() - SoupStart) * 1000.0;

		CapBuildMsPerPlate[PlateIndex] = 0.0;

		const double BvhStart = FPlatformTime::Seconds();
		if (PlateState.SoupData.LocalTriangles.Num() > 0 && PlateState.SoupData.RotatedVertices.Num() > 0)
		{
			PlateState.Adapter = MakeUnique<FPlateTriangleSoupAdapter>(&PlateState.SoupData);
			PlateState.BVH = MakeUnique<FPlateTriangleSoupBVH>();
			PlateState.BVH->SetMesh(PlateState.Adapter.Get(), false);
			PlateState.BVH->SetBuildOptions(16);
			PlateState.BVH->Build();
		}
		BVHBuildMsPerPlate[PlateIndex] = (FPlatformTime::Seconds() - BvhStart) * 1000.0;
	}, EParallelForFlags::Unbalanced);

	int32 TotalSoupTriangles = 0;
	int32 TotalSoupVertices = 0;
	double SoupExtractTotalMs = 0.0;
	double SoupExtractMaxMs = 0.0;
	double CapBuildTotalMs = 0.0;
	double CapBuildMaxMs = 0.0;
	double BVHBuildTotalMs = 0.0;
	double BVHBuildMaxMs = 0.0;
	for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
	{
		TotalSoupTriangles += PlateState.SoupData.LocalTriangles.Num();
		TotalSoupVertices += PlateState.SoupData.RotatedVertices.Num();
	}
	for (const int32 PlateIndex : PlatesToRebuild)
	{
		SoupExtractTotalMs += SoupExtractMsPerPlate[PlateIndex];
		SoupExtractMaxMs = FMath::Max(SoupExtractMaxMs, SoupExtractMsPerPlate[PlateIndex]);
		CapBuildTotalMs += CapBuildMsPerPlate[PlateIndex];
		CapBuildMaxMs = FMath::Max(CapBuildMaxMs, CapBuildMsPerPlate[PlateIndex]);
		BVHBuildTotalMs += BVHBuildMsPerPlate[PlateIndex];
		BVHBuildMaxMs = FMath::Max(BVHBuildMaxMs, BVHBuildMsPerPlate[PlateIndex]);
	}

	for (const int32 PlateIndex : PlatesToRebuild)
	{
		SpatialQueryData->DirtyPlateFlags[PlateIndex] = 0;
	}

	bool bAnyDirtyPlateRemaining = false;
	for (const uint8 bPlateDirty : SpatialQueryData->DirtyPlateFlags)
	{
		if (bPlateDirty != 0)
		{
			bAnyDirtyPlateRemaining = true;
			break;
		}
	}

	SpatialQueryData->bDirty = bAnyDirtyPlateRemaining;
	SpatialQueryData->bCapsDirty = true;
	SpatialQueryData->LastRebuiltPlateCount = PlatesToRebuild.Num();
	SpatialQueryData->LastSoupExtractTotalMs = SoupExtractTotalMs;
	SpatialQueryData->LastSoupExtractMaxMs = SoupExtractMaxMs;
	SpatialQueryData->LastCapBuildTotalMs = CapBuildTotalMs;
	SpatialQueryData->LastCapBuildMaxMs = CapBuildMaxMs;
	SpatialQueryData->LastBVHBuildTotalMs = BVHBuildTotalMs;
	SpatialQueryData->LastBVHBuildMaxMs = BVHBuildMaxMs;

	UE_LOG(LogTemp, Log, TEXT("SpatialQuery: rebuilt canonical BVHs for %d/%d plates, triangles=%d, vertices=%d"),
		PlatesToRebuild.Num(),
		SpatialQueryData->PlateStates.Num(),
		TotalSoupTriangles,
		TotalSoupVertices);
}

double FTectonicPlanet::UpdateSpatialCapsForCurrentRotationsInternal() const
{
	if (!SpatialQueryData || SpatialQueryData->PlateStates.Num() == 0)
	{
		return 0.0;
	}

	const TArray<FCanonicalSample>& Samples = GetReadableSamplesInternal();
	const double CapStart = FPlatformTime::Seconds();
	ParallelFor(SpatialQueryData->PlateStates.Num(), [this, &Samples](int32 PlateStateIndex)
	{
		FSpatialQueryData::FPlateSpatialState& PlateState = SpatialQueryData->PlateStates[PlateStateIndex];
		PlateState.bHasValidCap = false;
		PlateState.CapCenterDirection = FVector::ZeroVector;
		PlateState.CapHalfAngle = 0.0;
		PlateState.CapCosHalfAngle = -1.0;

		if (!Plates.IsValidIndex(PlateState.PlateId))
		{
			return;
		}

		const FPlate& Plate = Plates[PlateState.PlateId];
		if (Plate.SampleIndices.Num() == 0)
		{
			return;
		}

		const FQuat PlateRotation = Plate.CumulativeRotation;
		FVector MeanDirection = FVector::ZeroVector;
		for (const int32 SampleIndex : Plate.SampleIndices)
		{
			checkSlow(Samples.IsValidIndex(SampleIndex));
			MeanDirection += PlateRotation.RotateVector(Samples[SampleIndex].Position);
		}

		if (MeanDirection.IsNearlyZero())
		{
			const int32 FallbackSampleIndex = Plate.SampleIndices[0];
			MeanDirection = PlateRotation.RotateVector(Samples[FallbackSampleIndex].Position);
		}

		if (MeanDirection.IsNearlyZero())
		{
			return;
		}

		PlateState.CapCenterDirection = MeanDirection.GetSafeNormal();
		double MinDotToCenter = 1.0;
		for (const int32 SampleIndex : Plate.SampleIndices)
		{
			const FVector RotatedPosition = PlateRotation.RotateVector(Samples[SampleIndex].Position);
			const double Dot = FVector::DotProduct(PlateState.CapCenterDirection, RotatedPosition);
			MinDotToCenter = FMath::Min(MinDotToCenter, Dot);
		}

		PlateState.CapCosHalfAngle = FMath::Clamp(MinDotToCenter, -1.0, 1.0);
		PlateState.CapHalfAngle = FMath::Acos(PlateState.CapCosHalfAngle);
		PlateState.bHasValidCap = true;
	}, EParallelForFlags::Unbalanced);

	SpatialQueryData->bCapsDirty = false;
	return (FPlatformTime::Seconds() - CapStart) * 1000.0;
}

void FTectonicPlanet::EnsureSpatialQueryDataBuilt() const
{
	if (!SpatialQueryData)
	{
		SpatialQueryData = MakeUnique<FSpatialQueryData>();
	}

	if (SpatialQueryData->bDirty)
	{
		BuildSpatialQueryDataInternal();
	}
	if (SpatialQueryData->bCapsDirty)
	{
		UpdateSpatialCapsForCurrentRotationsInternal();
	}
}

int32 FTectonicPlanet::FindNearestPlateByCap(const FVector& Direction) const
{
	EnsureSpatialQueryDataBuilt();
	if (!SpatialQueryData || SpatialQueryData->PlateStates.Num() == 0)
	{
		return 0;
	}

	double BestDot = -TNumericLimits<double>::Max();
	int32 BestPlateId = 0;
	for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
	{
		const FVector Center = PlateState.bHasValidCap
			? PlateState.CapCenterDirection
			: Plates[PlateState.PlateId].CumulativeRotation.RotateVector(Plates[PlateState.PlateId].RotationAxis).GetSafeNormal();
		const double Dot = FVector::DotProduct(Direction, Center);
		if (Dot > BestDot)
		{
			BestDot = Dot;
			BestPlateId = PlateState.PlateId;
		}
	}

	return BestPlateId;
}

float FTectonicPlanet::ComputeOwnershipMarginFromCaps(const FVector& Direction) const
{
	EnsureSpatialQueryDataBuilt();
	if (!SpatialQueryData || SpatialQueryData->PlateStates.Num() == 0)
	{
		return 0.0f;
	}

	double BestDot = -TNumericLimits<double>::Max();
	double SecondBestDot = -TNumericLimits<double>::Max();
	for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
	{
		const FVector Center = PlateState.bHasValidCap
			? PlateState.CapCenterDirection
			: Plates[PlateState.PlateId].CumulativeRotation.RotateVector(Plates[PlateState.PlateId].RotationAxis).GetSafeNormal();
		const double Dot = FVector::DotProduct(Direction, Center);
		if (Dot > BestDot)
		{
			SecondBestDot = BestDot;
			BestDot = Dot;
		}
		else if (Dot > SecondBestDot)
		{
			SecondBestDot = Dot;
		}
	}

	if (BestDot <= -1.0)
	{
		return 0.0f;
	}

	const double BestChordDistance = FMath::Sqrt(FMath::Max(0.0, 2.0 - 2.0 * BestDot));
	const double SecondChordDistance = (SecondBestDot <= -1.0)
		? BestChordDistance
		: FMath::Sqrt(FMath::Max(0.0, 2.0 - 2.0 * SecondBestDot));
	const double MarginNumerator = FMath::Max(0.0, SecondChordDistance - BestChordDistance);
	if (AverageSampleSpacing <= UE_DOUBLE_SMALL_NUMBER)
	{
		return 0.0f;
	}

	return static_cast<float>(MarginNumerator / AverageSampleSpacing);
}

void FTectonicPlanet::RunTerraneDetectionStub() const
{
	// TODO[M4]: Replace stub with terrane connected-component detection on continental crust.
	// TODO[M4]: Add stable terrane identity tracking across reconciliations (split/merge handling).
	UE_LOG(LogTemp, Verbose, TEXT("Reconcile Phase 7: Terrane detection stub (M1 no-op)."));
}

FVector FTectonicPlanet::ResolveDirectionField(
	const FVector& SamplePosition,
	const FVector& V0,
	const FVector& V1,
	const FVector& V2,
	const FVector& Barycentric)
{
	const FVector Blended = Barycentric.X * V0 + Barycentric.Y * V1 + Barycentric.Z * V2;
	const FVector Normal = SamplePosition.GetSafeNormal();
	FVector TangentProjected = Blended - FVector::DotProduct(Blended, Normal) * Normal;
	if (TangentProjected.SizeSquared() > KINDA_SMALL_NUMBER)
	{
		return TangentProjected.GetSafeNormal();
	}

	if (Barycentric.X >= Barycentric.Y && Barycentric.X >= Barycentric.Z)
	{
		TangentProjected = V0 - FVector::DotProduct(V0, Normal) * Normal;
	}
	else if (Barycentric.Y >= Barycentric.X && Barycentric.Y >= Barycentric.Z)
	{
		TangentProjected = V1 - FVector::DotProduct(V1, Normal) * Normal;
	}
	else
	{
		TangentProjected = V2 - FVector::DotProduct(V2, Normal) * Normal;
	}

	return TangentProjected.GetSafeNormal();
}

ECrustType FTectonicPlanet::MajorityCrustType(ECrustType A, ECrustType B, ECrustType C)
{
	if (A == B || A == C)
	{
		return A;
	}
	if (B == C)
	{
		return B;
	}
	return A;
}

EOrogenyType FTectonicPlanet::MajorityOrogenyType(EOrogenyType A, EOrogenyType B, EOrogenyType C)
{
	if (A == B || A == C)
	{
		return A;
	}
	if (B == C)
	{
		return B;
	}
	return A;
}

FVector3d FTectonicPlanet::ComputePlanarBarycentric(const FVector3d& A, const FVector3d& B, const FVector3d& C, const FVector3d& P)
{
	const FVector3d Normal = UE::Geometry::VectorUtil::Normal(A, B, C);
	const double PlaneOffset = FVector::DotProduct(Normal, A);
	const double RayDot = FVector::DotProduct(Normal, P);

	FVector3d ProjectedP = P;
	if (FMath::Abs(RayDot) > UE_DOUBLE_SMALL_NUMBER)
	{
		const double Scale = PlaneOffset / RayDot;
		ProjectedP = Scale * P;
	}
	else
	{
		const double SignedDistance = FVector::DotProduct(P - A, Normal);
		ProjectedP = P - SignedDistance * Normal;
	}

	const FVector3d V0 = B - A;
	const FVector3d V1 = C - A;
	const FVector3d V2 = ProjectedP - A;

	const double Dot00 = FVector::DotProduct(V0, V0);
	const double Dot01 = FVector::DotProduct(V0, V1);
	const double Dot02 = FVector::DotProduct(V0, V2);
	const double Dot11 = FVector::DotProduct(V1, V1);
	const double Dot12 = FVector::DotProduct(V1, V2);
	const double Denominator = Dot00 * Dot11 - Dot01 * Dot01;
	if (FMath::Abs(Denominator) <= 1e-20)
	{
		return FVector3d(-1.0, -1.0, -1.0);
	}

	const double InvDenominator = 1.0 / Denominator;
	const double V = (Dot11 * Dot02 - Dot01 * Dot12) * InvDenominator;
	const double W = (Dot00 * Dot12 - Dot01 * Dot02) * InvDenominator;
	const double U = 1.0 - V - W;
	return FVector3d(U, V, W);
}

void FTectonicPlanet::GenerateFibonacciSphere(int32 N)
{
	checkf(N > 0, TEXT("GenerateFibonacciSphere requires N > 0."));
	TArray<FCanonicalSample>& Samples = SampleBuffers[ReadableSampleBufferIndex.Load()];
	Samples.SetNum(N);

	const double GoldenRatio = (1.0 + FMath::Sqrt(5.0)) * 0.5;
	double MinRadius = TNumericLimits<double>::Max();
	double MaxRadius = 0.0;

	for (int32 Index = 0; Index < N; ++Index)
	{
		const double U = (static_cast<double>(Index) + 0.5) / static_cast<double>(N);
		const double Theta = FMath::Acos(1.0 - 2.0 * U);
		const double Phi = (2.0 * static_cast<double>(PI) * static_cast<double>(Index)) / GoldenRatio;

		double SinTheta = 0.0;
		double CosTheta = 0.0;
		double SinPhi = 0.0;
		double CosPhi = 0.0;
		FMath::SinCos(&SinTheta, &CosTheta, Theta);
		FMath::SinCos(&SinPhi, &CosPhi, Phi);

		FCanonicalSample& Sample = Samples[Index];
		Sample = FCanonicalSample{};
		Sample.Position = FVector(SinTheta * CosPhi, SinTheta * SinPhi, CosTheta);

		const double Radius = Sample.Position.Size();
		MinRadius = FMath::Min(MinRadius, Radius);
		MaxRadius = FMath::Max(MaxRadius, Radius);
	}

	UE_LOG(LogTemp, Log, TEXT("Fibonacci sphere: %d samples, |p| in [%.15f, %.15f]"), N, MinRadius, MaxRadius);
}

void FTectonicPlanet::BuildDelaunayTriangulation()
{
	Triangles.Reset();
	const TArray<FCanonicalSample>& Samples = GetReadableSamplesInternal();
	if (Samples.Num() < 4)
	{
		return;
	}

	TArray<FVector3d> HullPoints;
	HullPoints.Reserve(Samples.Num());
	for (const FCanonicalSample& Sample : Samples)
	{
		HullPoints.Add(FVector3d(Sample.Position.X, Sample.Position.Y, Sample.Position.Z));
	}

	UE::Geometry::TConvexHull3<double> Hull;
	const bool bHullBuilt = Hull.Solve(TArrayView<const FVector3d>(HullPoints));
	checkf(bHullBuilt, TEXT("Failed to build convex hull / spherical Delaunay triangulation."));

	const auto& HullTriangles = Hull.GetTriangles();
	Triangles.Reserve(HullTriangles.Num());

	TArray<uint8> VertexSeen;
	VertexSeen.Init(0, Samples.Num());

	int32 DegenerateTriangleCount = 0;
	for (const auto& HullTriangle : HullTriangles)
	{
		FDelaunayTriangle Triangle;
		Triangle.V[0] = HullTriangle[0];
		Triangle.V[1] = HullTriangle[1];
		Triangle.V[2] = HullTriangle[2];

		if (Triangle.V[0] == Triangle.V[1] || Triangle.V[1] == Triangle.V[2] || Triangle.V[2] == Triangle.V[0])
		{
			++DegenerateTriangleCount;
			continue;
		}

		const FVector& A = Samples[Triangle.V[0]].Position;
		const FVector& B = Samples[Triangle.V[1]].Position;
		const FVector& C = Samples[Triangle.V[2]].Position;
		const double TwiceArea = FVector::CrossProduct(B - A, C - A).Size();
		if (TwiceArea <= static_cast<double>(SMALL_NUMBER))
		{
			++DegenerateTriangleCount;
			continue;
		}

		VertexSeen[Triangle.V[0]] = 1;
		VertexSeen[Triangle.V[1]] = 1;
		VertexSeen[Triangle.V[2]] = 1;
		Triangles.Add(Triangle);
	}

	int32 MissingVertexCount = 0;
	for (const uint8 bSeen : VertexSeen)
	{
		MissingVertexCount += (bSeen == 0) ? 1 : 0;
	}

	const int32 ExpectedTriangleCount = 2 * Samples.Num() - 4;
	UE_LOG(LogTemp, Log, TEXT("SDT: %d samples -> %d triangles (expected ~%d), skipped degenerate=%d, missing verts=%d"),
		Samples.Num(), Triangles.Num(), ExpectedTriangleCount, DegenerateTriangleCount, MissingVertexCount);

	ensureMsgf(MissingVertexCount == 0, TEXT("Spherical Delaunay hull did not reference %d vertices."), MissingVertexCount);
}

void FTectonicPlanet::BuildAdjacencyGraph()
{
	const TArray<FCanonicalSample>& Samples = GetReadableSamplesInternal();
	Adjacency.Reset();
	Adjacency.SetNum(Samples.Num());

	auto AddEdge = [this](int32 A, int32 B)
	{
		if (A == B || !Adjacency.IsValidIndex(A) || !Adjacency.IsValidIndex(B))
		{
			return;
		}

		Adjacency[A].AddUnique(B);
		Adjacency[B].AddUnique(A);
	};

	for (const FDelaunayTriangle& Triangle : Triangles)
	{
		AddEdge(Triangle.V[0], Triangle.V[1]);
		AddEdge(Triangle.V[1], Triangle.V[2]);
		AddEdge(Triangle.V[2], Triangle.V[0]);
	}

	int32 ZeroValenceVertices = 0;
	int32 MinValence = MAX_int32;
	int32 MaxValence = 0;
	int64 TotalValence = 0;

	for (const TArray<int32>& Neighbors : Adjacency)
	{
		const int32 Valence = Neighbors.Num();
		ZeroValenceVertices += (Valence == 0) ? 1 : 0;
		MinValence = FMath::Min(MinValence, Valence);
		MaxValence = FMath::Max(MaxValence, Valence);
		TotalValence += Valence;
	}

	const double AverageValence = (Adjacency.Num() > 0) ? static_cast<double>(TotalValence) / static_cast<double>(Adjacency.Num()) : 0.0;
	UE_LOG(LogTemp, Log, TEXT("SDT adjacency: %d vertices, valence min/max/avg = %d/%d/%.3f, zero-valence=%d"),
		Adjacency.Num(),
		(MinValence == MAX_int32) ? 0 : MinValence,
		MaxValence,
		AverageValence,
		ZeroValenceVertices);

	ensureMsgf(ZeroValenceVertices == 0, TEXT("Adjacency graph has %d zero-valence vertices."), ZeroValenceVertices);
}
