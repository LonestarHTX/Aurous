#include "TectonicPlanet.h"

#include "Async/ParallelFor.h"
#include "Async/TaskGraphInterfaces.h"
#include "CompGeom/ConvexHull3.h"
#include "HAL/PlatformTime.h"
#include "Math/RandomStream.h"
#include "PlateTriangleSoupAdapter.h"
#include "ShewchukPredicates.h"
#include "TectonicPlanetOwnershipUtils.h"

namespace
{
	bool GShewchukPredicatesInitialized = false;
	constexpr double MaxAcceptableReconcileUnaccountedPct = 10.0;

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

	uint64 HashPlateSoupTriangleLists(const FPlate& Plate)
	{
		uint64 Hash = HashTriangleIndexList(Plate.InteriorTriangles);
		Hash ^= 0x9e3779b97f4a7c15ull;
		Hash *= 1099511628211ull;
		Hash ^= HashTriangleIndexList(Plate.BoundaryTriangles);
		Hash *= 1099511628211ull;
		Hash ^= static_cast<uint64>(Plate.InteriorTriangles.Num() + Plate.BoundaryTriangles.Num());
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

	FVector3d NormalizeContainmentBarycentric(const FVector3d& RawBarycentric)
	{
		FVector3d Barycentric(
			FMath::Max(0.0, RawBarycentric.X),
			FMath::Max(0.0, RawBarycentric.Y),
			FMath::Max(0.0, RawBarycentric.Z));
		const double BarySum = Barycentric.X + Barycentric.Y + Barycentric.Z;
		if (BarySum > UE_DOUBLE_SMALL_NUMBER)
		{
			Barycentric /= BarySum;
		}
		return Barycentric;
	}

	double ComputeContainmentScore(const FVector3d& Barycentric)
	{
		return FMath::Min3(Barycentric.X, Barycentric.Y, Barycentric.Z);
	}

	bool IsBetterContainmentCandidate(const double CandidateScore, const int32 CandidatePlateId, const double BestScore, const int32 BestPlateId)
	{
		if (CandidateScore > BestScore + UE_DOUBLE_SMALL_NUMBER)
		{
			return true;
		}

		if (CandidateScore + UE_DOUBLE_SMALL_NUMBER < BestScore)
		{
			return false;
		}

		if (BestPlateId == INDEX_NONE)
		{
			return true;
		}

		return CandidatePlateId < BestPlateId;
	}

	bool IsPreferredPlateForTriangle(
		const TArray<FCanonicalSample>& Samples,
		const FDelaunayTriangle& Triangle,
		const int32 CandidatePlateId,
		const FVector3d& Barycentric)
	{
		return TectonicPlanetOwnership::IsPreferredWeightedPlateOwner(Samples, Triangle, CandidatePlateId, Barycentric);
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

	struct FInfluencePathCandidate
	{
		bool bValid = false;
		float DistanceKm = TNumericLimits<float>::Max();
		float InfluenceWeight = 0.0f;
		int32 SecondaryId = INDEX_NONE;
		int32 FrontSampleIndex = INDEX_NONE;
	};

	struct FInfluenceSeed
	{
		int32 SampleIndex = INDEX_NONE;
		int32 PlateId = INDEX_NONE;
		int32 SecondaryId = INDEX_NONE;
		float InfluenceWeight = 0.0f;
	};

	struct FInfluenceQueueEntry
	{
		int32 SampleIndex = INDEX_NONE;
		int32 PlateId = INDEX_NONE;
		int32 SecondaryId = INDEX_NONE;
		int32 FrontSampleIndex = INDEX_NONE;
		float DistanceKm = 0.0f;
		float InfluenceWeight = 0.0f;

		bool operator<(const FInfluenceQueueEntry& Other) const
		{
			return DistanceKm > Other.DistanceKm;
		}
	};

	float SmoothStep01(const float X)
	{
		const float Clamped = FMath::Clamp(X, 0.0f, 1.0f);
		return (3.0f * Clamped * Clamped) - (2.0f * Clamped * Clamped * Clamped);
	}

	float ComputeSubductionInfluence(const float DistanceKm, const float PeakDistanceKm, const float RadiusKm)
	{
		if (DistanceKm < 0.0f || DistanceKm > RadiusKm)
		{
			return 0.0f;
		}

		if (DistanceKm <= PeakDistanceKm)
		{
			return SmoothStep01(DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, PeakDistanceKm));
		}

		const float FalloffAlpha = (DistanceKm - PeakDistanceKm) / FMath::Max(KINDA_SMALL_NUMBER, RadiusKm - PeakDistanceKm);
		return 1.0f - SmoothStep01(FalloffAlpha);
	}

	float ComputeSpeedInfluence(const float SpeedMmPerYear, const float MaxSpeedMmPerYear)
	{
		return FMath::Clamp(SpeedMmPerYear / FMath::Max(KINDA_SMALL_NUMBER, MaxSpeedMmPerYear), 0.0f, 1.0f);
	}

	float ComputeElevationInfluence(const float ElevationKm, const float TrenchElevationKm, const float MaxElevationKm)
	{
		const float Normalized = FMath::Clamp(
			(ElevationKm - TrenchElevationKm) / FMath::Max(KINDA_SMALL_NUMBER, MaxElevationKm - TrenchElevationKm),
			0.0f,
			1.0f);
		return Normalized * Normalized;
	}

	float ComputeTrenchInfluence(const float DistanceKm, const float RadiusKm)
	{
		if (DistanceKm < 0.0f || DistanceKm > RadiusKm)
		{
			return 0.0f;
		}

		const float Normalized = FMath::Clamp(1.0f - (DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, RadiusKm)), 0.0f, 1.0f);
		return Normalized * Normalized;
	}

	bool IsBetterInfluenceCandidate(
		const FInfluencePathCandidate& Candidate,
		const FInfluencePathCandidate& Best,
		const float RoleRadiusKm)
	{
		if (!Candidate.bValid)
		{
			return false;
		}
		if (!Best.bValid)
		{
			return true;
		}

		const float CandidateNormalizedDistance = Candidate.DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, RoleRadiusKm);
		const float BestNormalizedDistance = Best.DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, RoleRadiusKm);
		if (CandidateNormalizedDistance + KINDA_SMALL_NUMBER < BestNormalizedDistance)
		{
			return true;
		}
		if (BestNormalizedDistance + KINDA_SMALL_NUMBER < CandidateNormalizedDistance)
		{
			return false;
		}
		if (Candidate.InfluenceWeight > Best.InfluenceWeight + KINDA_SMALL_NUMBER)
		{
			return true;
		}
		if (Best.InfluenceWeight > Candidate.InfluenceWeight + KINDA_SMALL_NUMBER)
		{
			return false;
		}
		if (Best.SecondaryId == INDEX_NONE)
		{
			return true;
		}
		if (Candidate.SecondaryId != Best.SecondaryId)
		{
			return Candidate.SecondaryId < Best.SecondaryId;
		}
		return Candidate.FrontSampleIndex < Best.FrontSampleIndex;
	}

	bool IsInfluenceCandidateBetterAcrossRoles(
		const FInfluencePathCandidate& Candidate,
		const float CandidateRadiusKm,
		const FInfluencePathCandidate& Best,
		const float BestRadiusKm)
	{
		if (!Candidate.bValid)
		{
			return false;
		}
		if (!Best.bValid)
		{
			return true;
		}

		const float CandidateNormalizedDistance = Candidate.DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, CandidateRadiusKm);
		const float BestNormalizedDistance = Best.DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, BestRadiusKm);
		if (CandidateNormalizedDistance + KINDA_SMALL_NUMBER < BestNormalizedDistance)
		{
			return true;
		}
		if (BestNormalizedDistance + KINDA_SMALL_NUMBER < CandidateNormalizedDistance)
		{
			return false;
		}
		if (Candidate.InfluenceWeight > Best.InfluenceWeight + KINDA_SMALL_NUMBER)
		{
			return true;
		}
		if (Best.InfluenceWeight > Candidate.InfluenceWeight + KINDA_SMALL_NUMBER)
		{
			return false;
		}
		if (Best.SecondaryId == INDEX_NONE)
		{
			return true;
		}
		if (Candidate.SecondaryId != Best.SecondaryId)
		{
			return Candidate.SecondaryId < Best.SecondaryId;
		}
		return Candidate.FrontSampleIndex < Best.FrontSampleIndex;
	}

		int32 CountSetFlags(const TArray<uint8>& Flags)
	{
		int32 Count = 0;
		for (const uint8 Flag : Flags)
		{
			Count += (Flag != 0) ? 1 : 0;
		}
		return Count;
	}

	template <typename TraversalPredicateType>
	void PropagateInfluenceSeedsIntoExistingCandidates(
		const TArray<FCanonicalSample>& Samples,
		const TArray<TArray<int32>>& Adjacency,
		const TArray<TArray<float>>& EdgeDistancesKm,
		const TArray<FInfluenceSeed>& Seeds,
		const float RadiusKm,
		const TraversalPredicateType& TraversalPredicate,
		TArray<FInfluencePathCandidate>& InOutBestCandidates)
	{
		if (InOutBestCandidates.Num() != Samples.Num())
		{
			InOutBestCandidates.SetNum(Samples.Num());
			for (FInfluencePathCandidate& Candidate : InOutBestCandidates)
			{
				Candidate = FInfluencePathCandidate{};
			}
		}

		TArray<FInfluenceQueueEntry> Queue;
		Queue.Reserve(Seeds.Num());
		for (const FInfluenceSeed& Seed : Seeds)
		{
			if (!Samples.IsValidIndex(Seed.SampleIndex))
			{
				continue;
			}

			const FCanonicalSample& SeedSample = Samples[Seed.SampleIndex];
			if (!TraversalPredicate(SeedSample, Seed))
			{
				continue;
			}

			FInfluencePathCandidate SeedCandidate;
			SeedCandidate.bValid = true;
			SeedCandidate.DistanceKm = 0.0f;
			SeedCandidate.InfluenceWeight = Seed.InfluenceWeight;
			SeedCandidate.SecondaryId = Seed.SecondaryId;
			SeedCandidate.FrontSampleIndex = Seed.SampleIndex;
			if (IsBetterInfluenceCandidate(SeedCandidate, InOutBestCandidates[Seed.SampleIndex], RadiusKm))
			{
				InOutBestCandidates[Seed.SampleIndex] = SeedCandidate;

				FInfluenceQueueEntry QueueEntry;
				QueueEntry.SampleIndex = Seed.SampleIndex;
				QueueEntry.PlateId = Seed.PlateId;
				QueueEntry.SecondaryId = Seed.SecondaryId;
				QueueEntry.FrontSampleIndex = Seed.SampleIndex;
				QueueEntry.DistanceKm = 0.0f;
				QueueEntry.InfluenceWeight = Seed.InfluenceWeight;
				Queue.HeapPush(QueueEntry);
			}
		}

		while (Queue.Num() > 0)
		{
			FInfluenceQueueEntry Entry;
			Queue.HeapPop(Entry);
			if (!Samples.IsValidIndex(Entry.SampleIndex))
			{
				continue;
			}

			const FInfluencePathCandidate& CurrentBest = InOutBestCandidates[Entry.SampleIndex];
			if (!CurrentBest.bValid ||
				CurrentBest.SecondaryId != Entry.SecondaryId ||
				CurrentBest.FrontSampleIndex != Entry.FrontSampleIndex ||
				!FMath::IsNearlyEqual(CurrentBest.DistanceKm, Entry.DistanceKm, 1e-3f))
			{
				continue;
			}

			checkSlow(Adjacency.IsValidIndex(Entry.SampleIndex));
			checkSlow(EdgeDistancesKm.IsValidIndex(Entry.SampleIndex));
			const TArray<int32>& NeighborIndices = Adjacency[Entry.SampleIndex];
			const TArray<float>& NeighborEdgeDistancesKm = EdgeDistancesKm[Entry.SampleIndex];
			checkSlow(NeighborIndices.Num() == NeighborEdgeDistancesKm.Num());

			for (int32 NeighborSlot = 0; NeighborSlot < NeighborIndices.Num(); ++NeighborSlot)
			{
				const int32 NeighborIndex = NeighborIndices[NeighborSlot];
				if (!Samples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				const FCanonicalSample& NeighborSample = Samples[NeighborIndex];
				if (!TraversalPredicate(NeighborSample, Entry))
				{
					continue;
				}

				const float NewDistanceKm = Entry.DistanceKm + NeighborEdgeDistancesKm[NeighborSlot];
				if (NewDistanceKm > RadiusKm)
				{
					continue;
				}

				FInfluencePathCandidate Candidate;
				Candidate.bValid = true;
				Candidate.DistanceKm = NewDistanceKm;
				Candidate.InfluenceWeight = Entry.InfluenceWeight;
				Candidate.SecondaryId = Entry.SecondaryId;
				Candidate.FrontSampleIndex = Entry.FrontSampleIndex;
				if (!IsBetterInfluenceCandidate(Candidate, InOutBestCandidates[NeighborIndex], RadiusKm))
				{
					continue;
				}

				InOutBestCandidates[NeighborIndex] = Candidate;

				FInfluenceQueueEntry NextEntry = Entry;
				NextEntry.SampleIndex = NeighborIndex;
				NextEntry.DistanceKm = NewDistanceKm;
				Queue.HeapPush(NextEntry);
			}
		}
	}

	template <typename TraversalPredicateType>
	void PropagateInfluenceSeeds(
		const TArray<FCanonicalSample>& Samples,
		const TArray<TArray<int32>>& Adjacency,
		const TArray<TArray<float>>& EdgeDistancesKm,
		const TArray<FInfluenceSeed>& Seeds,
		const float RadiusKm,
		const TraversalPredicateType& TraversalPredicate,
		TArray<FInfluencePathCandidate>& OutBestCandidates)
	{
		OutBestCandidates.SetNum(Samples.Num());
		for (FInfluencePathCandidate& Candidate : OutBestCandidates)
		{
			Candidate = FInfluencePathCandidate{};
		}

		PropagateInfluenceSeedsIntoExistingCandidates(
			Samples,
			Adjacency,
			EdgeDistancesKm,
			Seeds,
			RadiusKm,
			TraversalPredicate,
			OutBestCandidates);
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
	, AdjacencyEdgeDistancesKm(Other.AdjacencyEdgeDistancesKm)
	, Plates(Other.Plates)
	, TerraneRecords(Other.TerraneRecords)
	, CollisionEvents(Other.CollisionEvents)
	, CollisionHistoryKeys(Other.CollisionHistoryKeys)
	, SpatialQueryData(MakeUnique<FSpatialQueryData>())
	, AverageSampleSpacing(Other.AverageSampleSpacing)
	, AverageCellAreaKm2(Other.AverageCellAreaKm2)
	, InitialMeanPlateAreaKm2(Other.InitialMeanPlateAreaKm2)
	, ReconcileDisplacementThreshold(Other.ReconcileDisplacementThreshold)
	, MaxAngularDisplacementSinceReconcile(Other.MaxAngularDisplacementSinceReconcile)
	, TimestepCounter(Other.TimestepCounter)
	, ReconcileCount(Other.ReconcileCount)
	, NextTerraneId(Other.NextTerraneId)
	, bReconcileTriggeredLastStep(Other.bReconcileTriggeredLastStep)
	, BoundarySampleCount(Other.BoundarySampleCount)
	, BoundaryMeanDepthHops(Other.BoundaryMeanDepthHops)
	, BoundaryMaxDepthHops(Other.BoundaryMaxDepthHops)
	, BoundaryDeepSampleCount(Other.BoundaryDeepSampleCount)
	, ContinentalSampleCount(Other.ContinentalSampleCount)
, ContinentalPlateCount(Other.ContinentalPlateCount)
, InitialContinentalPlateCount(Other.InitialContinentalPlateCount)
, ContinentalAreaFraction(Other.ContinentalAreaFraction)
	, ContinentalComponentCount(Other.ContinentalComponentCount)
	, LargestContinentalComponentSize(Other.LargestContinentalComponentSize)
	, MaxPlateComponentCount(Other.MaxPlateComponentCount)
	, DetachedPlateFragmentSampleCount(Other.DetachedPlateFragmentSampleCount)
	, LargestDetachedPlateFragmentSize(Other.LargestDetachedPlateFragmentSize)
	, SubductionFrontSampleCount(Other.SubductionFrontSampleCount)
	, AndeanSampleCount(Other.AndeanSampleCount)
	, TrackedTerraneCount(Other.TrackedTerraneCount)
	, ActiveTerraneCount(Other.ActiveTerraneCount)
	, MergedTerraneCount(Other.MergedTerraneCount)
	, CollisionEventCount(Other.CollisionEventCount)
	, HimalayanSampleCount(Other.HimalayanSampleCount)
	, PendingCollisionSampleCount(Other.PendingCollisionSampleCount)
	, MaxSubductionDistanceKm(Other.MaxSubductionDistanceKm)
	, MinProtectedPlateSampleCount(Other.MinProtectedPlateSampleCount)
	, EmptyProtectedPlateCount(Other.EmptyProtectedPlateCount)
	, RescuedProtectedPlateCount(Other.RescuedProtectedPlateCount)
	, RescuedProtectedSampleCount(Other.RescuedProtectedSampleCount)
	, RepeatedlyRescuedProtectedSampleCount(Other.RepeatedlyRescuedProtectedSampleCount)
	, LastGapSampleCount(Other.LastGapSampleCount)
	, LastOverlapSampleCount(Other.LastOverlapSampleCount)
	, HysteresisThreshold(Other.HysteresisThreshold)
	, BoundaryConfidenceThreshold(Other.BoundaryConfidenceThreshold)
	, TargetContinentalAreaFraction(Other.TargetContinentalAreaFraction)
	, MinContinentalAreaFraction(Other.MinContinentalAreaFraction)
	, MaxContinentalAreaFraction(Other.MaxContinentalAreaFraction)
	, MinContinentalPlateFraction(Other.MinContinentalPlateFraction)
	, ContinentalStabilizerMode(Other.ContinentalStabilizerMode)
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
		AdjacencyEdgeDistancesKm = Other.AdjacencyEdgeDistancesKm;
		Plates = Other.Plates;
		TerraneRecords = Other.TerraneRecords;
		CollisionEvents = Other.CollisionEvents;
		CollisionHistoryKeys = Other.CollisionHistoryKeys;
		AverageSampleSpacing = Other.AverageSampleSpacing;
		AverageCellAreaKm2 = Other.AverageCellAreaKm2;
		InitialMeanPlateAreaKm2 = Other.InitialMeanPlateAreaKm2;
		ReconcileDisplacementThreshold = Other.ReconcileDisplacementThreshold;
		MaxAngularDisplacementSinceReconcile = Other.MaxAngularDisplacementSinceReconcile;
		TimestepCounter = Other.TimestepCounter;
		ReconcileCount = Other.ReconcileCount;
		NextTerraneId = Other.NextTerraneId;
		bReconcileTriggeredLastStep = Other.bReconcileTriggeredLastStep;
		BoundarySampleCount = Other.BoundarySampleCount;
		BoundaryMeanDepthHops = Other.BoundaryMeanDepthHops;
		BoundaryMaxDepthHops = Other.BoundaryMaxDepthHops;
		BoundaryDeepSampleCount = Other.BoundaryDeepSampleCount;
		ContinentalSampleCount = Other.ContinentalSampleCount;
	ContinentalPlateCount = Other.ContinentalPlateCount;
	InitialContinentalPlateCount = Other.InitialContinentalPlateCount;
	ContinentalAreaFraction = Other.ContinentalAreaFraction;
		ContinentalComponentCount = Other.ContinentalComponentCount;
		LargestContinentalComponentSize = Other.LargestContinentalComponentSize;
		MaxPlateComponentCount = Other.MaxPlateComponentCount;
		DetachedPlateFragmentSampleCount = Other.DetachedPlateFragmentSampleCount;
		LargestDetachedPlateFragmentSize = Other.LargestDetachedPlateFragmentSize;
		SubductionFrontSampleCount = Other.SubductionFrontSampleCount;
		AndeanSampleCount = Other.AndeanSampleCount;
		TrackedTerraneCount = Other.TrackedTerraneCount;
		ActiveTerraneCount = Other.ActiveTerraneCount;
		MergedTerraneCount = Other.MergedTerraneCount;
		CollisionEventCount = Other.CollisionEventCount;
		HimalayanSampleCount = Other.HimalayanSampleCount;
		PendingCollisionSampleCount = Other.PendingCollisionSampleCount;
		MaxSubductionDistanceKm = Other.MaxSubductionDistanceKm;
		MinProtectedPlateSampleCount = Other.MinProtectedPlateSampleCount;
		EmptyProtectedPlateCount = Other.EmptyProtectedPlateCount;
		RescuedProtectedPlateCount = Other.RescuedProtectedPlateCount;
		RescuedProtectedSampleCount = Other.RescuedProtectedSampleCount;
		RepeatedlyRescuedProtectedSampleCount = Other.RepeatedlyRescuedProtectedSampleCount;
		LastGapSampleCount = Other.LastGapSampleCount;
		LastOverlapSampleCount = Other.LastOverlapSampleCount;
		HysteresisThreshold = Other.HysteresisThreshold;
		BoundaryConfidenceThreshold = Other.BoundaryConfidenceThreshold;
		TargetContinentalAreaFraction = Other.TargetContinentalAreaFraction;
		MinContinentalAreaFraction = Other.MinContinentalAreaFraction;
		MaxContinentalAreaFraction = Other.MaxContinentalAreaFraction;
		MinContinentalPlateFraction = Other.MinContinentalPlateFraction;
		ContinentalStabilizerMode = Other.ContinentalStabilizerMode;
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
	, AdjacencyEdgeDistancesKm(MoveTemp(Other.AdjacencyEdgeDistancesKm))
	, Plates(MoveTemp(Other.Plates))
	, TerraneRecords(MoveTemp(Other.TerraneRecords))
	, CollisionEvents(MoveTemp(Other.CollisionEvents))
	, CollisionHistoryKeys(MoveTemp(Other.CollisionHistoryKeys))
	, SpatialQueryData(MoveTemp(Other.SpatialQueryData))
	, AverageSampleSpacing(Other.AverageSampleSpacing)
	, AverageCellAreaKm2(Other.AverageCellAreaKm2)
	, InitialMeanPlateAreaKm2(Other.InitialMeanPlateAreaKm2)
	, ReconcileDisplacementThreshold(Other.ReconcileDisplacementThreshold)
	, MaxAngularDisplacementSinceReconcile(Other.MaxAngularDisplacementSinceReconcile)
	, TimestepCounter(Other.TimestepCounter)
	, ReconcileCount(Other.ReconcileCount)
	, NextTerraneId(Other.NextTerraneId)
	, bReconcileTriggeredLastStep(Other.bReconcileTriggeredLastStep)
	, BoundarySampleCount(Other.BoundarySampleCount)
	, BoundaryMeanDepthHops(Other.BoundaryMeanDepthHops)
	, BoundaryMaxDepthHops(Other.BoundaryMaxDepthHops)
	, BoundaryDeepSampleCount(Other.BoundaryDeepSampleCount)
	, ContinentalSampleCount(Other.ContinentalSampleCount)
, ContinentalPlateCount(Other.ContinentalPlateCount)
, InitialContinentalPlateCount(Other.InitialContinentalPlateCount)
, ContinentalAreaFraction(Other.ContinentalAreaFraction)
	, ContinentalComponentCount(Other.ContinentalComponentCount)
	, LargestContinentalComponentSize(Other.LargestContinentalComponentSize)
	, MaxPlateComponentCount(Other.MaxPlateComponentCount)
	, DetachedPlateFragmentSampleCount(Other.DetachedPlateFragmentSampleCount)
	, LargestDetachedPlateFragmentSize(Other.LargestDetachedPlateFragmentSize)
	, SubductionFrontSampleCount(Other.SubductionFrontSampleCount)
	, AndeanSampleCount(Other.AndeanSampleCount)
	, TrackedTerraneCount(Other.TrackedTerraneCount)
	, ActiveTerraneCount(Other.ActiveTerraneCount)
	, MergedTerraneCount(Other.MergedTerraneCount)
	, CollisionEventCount(Other.CollisionEventCount)
	, HimalayanSampleCount(Other.HimalayanSampleCount)
	, PendingCollisionSampleCount(Other.PendingCollisionSampleCount)
	, MaxSubductionDistanceKm(Other.MaxSubductionDistanceKm)
	, MinProtectedPlateSampleCount(Other.MinProtectedPlateSampleCount)
	, EmptyProtectedPlateCount(Other.EmptyProtectedPlateCount)
	, RescuedProtectedPlateCount(Other.RescuedProtectedPlateCount)
	, RescuedProtectedSampleCount(Other.RescuedProtectedSampleCount)
	, RepeatedlyRescuedProtectedSampleCount(Other.RepeatedlyRescuedProtectedSampleCount)
	, LastGapSampleCount(Other.LastGapSampleCount)
	, LastOverlapSampleCount(Other.LastOverlapSampleCount)
	, HysteresisThreshold(Other.HysteresisThreshold)
	, BoundaryConfidenceThreshold(Other.BoundaryConfidenceThreshold)
	, TargetContinentalAreaFraction(Other.TargetContinentalAreaFraction)
	, MinContinentalAreaFraction(Other.MinContinentalAreaFraction)
	, MaxContinentalAreaFraction(Other.MaxContinentalAreaFraction)
	, MinContinentalPlateFraction(Other.MinContinentalPlateFraction)
	, ContinentalStabilizerMode(Other.ContinentalStabilizerMode)
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
		AdjacencyEdgeDistancesKm = MoveTemp(Other.AdjacencyEdgeDistancesKm);
		Plates = MoveTemp(Other.Plates);
		TerraneRecords = MoveTemp(Other.TerraneRecords);
		CollisionEvents = MoveTemp(Other.CollisionEvents);
		CollisionHistoryKeys = MoveTemp(Other.CollisionHistoryKeys);
		SpatialQueryData = MoveTemp(Other.SpatialQueryData);
		AverageSampleSpacing = Other.AverageSampleSpacing;
		AverageCellAreaKm2 = Other.AverageCellAreaKm2;
		InitialMeanPlateAreaKm2 = Other.InitialMeanPlateAreaKm2;
		ReconcileDisplacementThreshold = Other.ReconcileDisplacementThreshold;
		MaxAngularDisplacementSinceReconcile = Other.MaxAngularDisplacementSinceReconcile;
		TimestepCounter = Other.TimestepCounter;
		ReconcileCount = Other.ReconcileCount;
		NextTerraneId = Other.NextTerraneId;
		bReconcileTriggeredLastStep = Other.bReconcileTriggeredLastStep;
		BoundarySampleCount = Other.BoundarySampleCount;
		BoundaryMeanDepthHops = Other.BoundaryMeanDepthHops;
		BoundaryMaxDepthHops = Other.BoundaryMaxDepthHops;
		BoundaryDeepSampleCount = Other.BoundaryDeepSampleCount;
		ContinentalSampleCount = Other.ContinentalSampleCount;
	ContinentalPlateCount = Other.ContinentalPlateCount;
	InitialContinentalPlateCount = Other.InitialContinentalPlateCount;
	ContinentalAreaFraction = Other.ContinentalAreaFraction;
		ContinentalComponentCount = Other.ContinentalComponentCount;
		LargestContinentalComponentSize = Other.LargestContinentalComponentSize;
		MaxPlateComponentCount = Other.MaxPlateComponentCount;
		DetachedPlateFragmentSampleCount = Other.DetachedPlateFragmentSampleCount;
		LargestDetachedPlateFragmentSize = Other.LargestDetachedPlateFragmentSize;
		SubductionFrontSampleCount = Other.SubductionFrontSampleCount;
		AndeanSampleCount = Other.AndeanSampleCount;
		TrackedTerraneCount = Other.TrackedTerraneCount;
		ActiveTerraneCount = Other.ActiveTerraneCount;
		MergedTerraneCount = Other.MergedTerraneCount;
		CollisionEventCount = Other.CollisionEventCount;
		HimalayanSampleCount = Other.HimalayanSampleCount;
		PendingCollisionSampleCount = Other.PendingCollisionSampleCount;
		MaxSubductionDistanceKm = Other.MaxSubductionDistanceKm;
		MinProtectedPlateSampleCount = Other.MinProtectedPlateSampleCount;
		EmptyProtectedPlateCount = Other.EmptyProtectedPlateCount;
		RescuedProtectedPlateCount = Other.RescuedProtectedPlateCount;
		RescuedProtectedSampleCount = Other.RescuedProtectedSampleCount;
		RepeatedlyRescuedProtectedSampleCount = Other.RepeatedlyRescuedProtectedSampleCount;
		LastGapSampleCount = Other.LastGapSampleCount;
		LastOverlapSampleCount = Other.LastOverlapSampleCount;
		HysteresisThreshold = Other.HysteresisThreshold;
		BoundaryConfidenceThreshold = Other.BoundaryConfidenceThreshold;
		TargetContinentalAreaFraction = Other.TargetContinentalAreaFraction;
		MinContinentalAreaFraction = Other.MinContinentalAreaFraction;
		MaxContinentalAreaFraction = Other.MaxContinentalAreaFraction;
		MinContinentalPlateFraction = Other.MinContinentalPlateFraction;
		ContinentalStabilizerMode = Other.ContinentalStabilizerMode;
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

bool FTectonicPlanet::IsPlateActive(const int32 PlateId) const
{
	return Plates.IsValidIndex(PlateId) && Plates[PlateId].SampleIndices.Num() > 0;
}

bool FTectonicPlanet::IsPlateProtected(const int32 PlateId) const
{
	return Plates.IsValidIndex(PlateId) && Plates[PlateId].PersistencePolicy == EPlatePersistencePolicy::Protected;
}

int32 FTectonicPlanet::GetInitialPlateFloorSamples(const int32 TotalSampleCount, const int32 PlateCount) const
{
	if (TotalSampleCount <= 0 || PlateCount <= 0)
	{
		return 0;
	}

	return FMath::Max(64, FMath::FloorToInt(0.30 * static_cast<double>(TotalSampleCount) / static_cast<double>(PlateCount)));
}

int32 FTectonicPlanet::GetPersistentPlateFloorSamples(const int32 PlateId) const
{
	if (!Plates.IsValidIndex(PlateId) || Plates[PlateId].PersistencePolicy != EPlatePersistencePolicy::Protected)
	{
		return 0;
	}

	return FMath::Max(32, FMath::CeilToInt(0.05 * static_cast<double>(FMath::Max(0, Plates[PlateId].InitialSampleCount))));
}

uint64 FTectonicPlanet::MakeCollisionHistoryKey(const int32 TerraneIdA, const int32 TerraneIdB)
{
	const uint32 Low = static_cast<uint32>(FMath::Min(TerraneIdA, TerraneIdB));
	const uint32 High = static_cast<uint32>(FMath::Max(TerraneIdA, TerraneIdB));
	return (static_cast<uint64>(Low) << 32) | static_cast<uint64>(High);
}

FTectonicPlanet::FTerraneRecord* FTectonicPlanet::FindTerraneRecordById(const int32 TerraneId)
{
	for (FTerraneRecord& Record : TerraneRecords)
	{
		if (Record.TerraneId == TerraneId)
		{
			return &Record;
		}
	}

	return nullptr;
}

const FTectonicPlanet::FTerraneRecord* FTectonicPlanet::FindTerraneRecordById(const int32 TerraneId) const
{
	for (const FTerraneRecord& Record : TerraneRecords)
	{
		if (Record.TerraneId == TerraneId)
		{
			return &Record;
		}
	}

	return nullptr;
}

void FTectonicPlanet::Initialize(int32 NumSamples)
{
	checkf(NumSamples >= 4, TEXT("FTectonicPlanet::Initialize requires at least 4 samples."));

	ReadableSampleBufferIndex.Store(0);
	SampleBuffers[0].Reset();
	SampleBuffers[1].Reset();
	Triangles.Reset();
	Adjacency.Reset();
	AdjacencyEdgeDistancesKm.Reset();
	Plates.Reset();
	TerraneRecords.Reset();
	CollisionEvents.Reset();
	CollisionHistoryKeys.Reset();
	AverageSampleSpacing = 0.0;
	AverageCellAreaKm2 = 0.0;
	InitialMeanPlateAreaKm2 = 0.0;
	ReconcileDisplacementThreshold = 0.0;
	MaxAngularDisplacementSinceReconcile = 0.0;
	TimestepCounter = 0;
	ReconcileCount = 0;
	NextTerraneId = 0;
	bReconcileTriggeredLastStep = false;
	BoundarySampleCount = 0;
	BoundaryMeanDepthHops = 0.0;
	BoundaryMaxDepthHops = 0;
	BoundaryDeepSampleCount = 0;
	ContinentalSampleCount = 0;
	ContinentalPlateCount = 0;
	InitialContinentalPlateCount = 0;
	ContinentalAreaFraction = 0.0;
	ContinentalComponentCount = 0;
	LargestContinentalComponentSize = 0;
	MaxPlateComponentCount = 0;
	DetachedPlateFragmentSampleCount = 0;
	LargestDetachedPlateFragmentSize = 0;
	LastGapSampleCount = 0;
	LastOverlapSampleCount = 0;
	TrackedTerraneCount = 0;
	ActiveTerraneCount = 0;
	MergedTerraneCount = 0;
	CollisionEventCount = 0;
	HimalayanSampleCount = 0;
	PendingCollisionSampleCount = 0;
	SubductionFrontSampleCount = 0;
	AndeanSampleCount = 0;
	MaxSubductionDistanceKm = 0.0f;
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
	AverageCellAreaKm2 = (4.0 * static_cast<double>(PI) * PlanetRadius * PlanetRadius) / static_cast<double>(GetReadableSamplesInternal().Num());
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
	FRandomStream LayoutRng(RandomSeed);
	FRandomStream PlateMotionRng(RandomSeed ^ 0x5f3759df);
	const double LocalAverageSampleSpacing = (Samples.Num() > 0)
		? FMath::Sqrt((4.0 * static_cast<double>(PI)) / static_cast<double>(Samples.Num()))
		: 1.0;
	const int32 InitialPlateFloorSamples = GetInitialPlateFloorSamples(Samples.Num(), ClampedNumPlates);
	const double ClampedMinContinentalAreaFraction = FMath::Clamp(MinContinentalAreaFraction, 0.0, 1.0);
	const double ClampedMaxContinentalAreaFraction = FMath::Clamp(MaxContinentalAreaFraction, ClampedMinContinentalAreaFraction, 1.0);
	const double ClampedTargetContinentalAreaFraction = FMath::Clamp(TargetContinentalAreaFraction, ClampedMinContinentalAreaFraction, ClampedMaxContinentalAreaFraction);
	const double ClampedMinContinentalPlateFraction = FMath::Clamp(MinContinentalPlateFraction, 0.0, 1.0);

	TArray<FVector> SelectedCentroids;
	TArray<int32> SelectedPlateAssignments;
	TArray<double> SelectedBestDots;
	TArray<double> SelectedSecondBestDots;
	TArray<int32> SelectedPlateCounts;
	SelectedPlateAssignments.SetNumUninitialized(Samples.Num());
	SelectedBestDots.SetNumUninitialized(Samples.Num());
	SelectedSecondBestDots.SetNumUninitialized(Samples.Num());
	SelectedPlateCounts.Init(0, ClampedNumPlates);

	bool bFoundValidLayout = false;
	int32 BestRejectedMinPlateCount = 0;
	constexpr int32 MaxInitializationAttempts = 4;

	for (int32 AttemptIndex = 0; AttemptIndex < MaxInitializationAttempts; ++AttemptIndex)
	{
		const int32 FirstCentroidSampleIndex = LayoutRng.RandRange(0, Samples.Num() - 1);
		TArray<FVector> AttemptCentroids;
		AttemptCentroids.Reserve(ClampedNumPlates);
		AttemptCentroids.Add(Samples[FirstCentroidSampleIndex].Position.GetSafeNormal());

		while (AttemptCentroids.Num() < ClampedNumPlates)
		{
			double BestNearestCentroidDot = TNumericLimits<double>::Max();
			int32 BestCentroidSampleIndex = INDEX_NONE;
			for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
			{
				const FVector CandidateDirection = Samples[SampleIndex].Position.GetSafeNormal();
				double NearestCentroidDot = -TNumericLimits<double>::Max();
				for (const FVector& ExistingCentroid : AttemptCentroids)
				{
					NearestCentroidDot = FMath::Max(NearestCentroidDot, FVector::DotProduct(CandidateDirection, ExistingCentroid));
				}

				if (BestCentroidSampleIndex == INDEX_NONE ||
					NearestCentroidDot < BestNearestCentroidDot - UE_DOUBLE_SMALL_NUMBER ||
					(FMath::IsNearlyEqual(NearestCentroidDot, BestNearestCentroidDot, UE_DOUBLE_SMALL_NUMBER) && SampleIndex < BestCentroidSampleIndex))
				{
					BestNearestCentroidDot = NearestCentroidDot;
					BestCentroidSampleIndex = SampleIndex;
				}
			}

			check(BestCentroidSampleIndex != INDEX_NONE);
			AttemptCentroids.Add(Samples[BestCentroidSampleIndex].Position.GetSafeNormal());
		}

		TArray<int32> AttemptPlateAssignments;
		TArray<double> AttemptBestDots;
		TArray<double> AttemptSecondBestDots;
		TArray<int32> AttemptPlateCounts;
		AttemptPlateAssignments.SetNumUninitialized(Samples.Num());
		AttemptBestDots.SetNumUninitialized(Samples.Num());
		AttemptSecondBestDots.SetNumUninitialized(Samples.Num());
		AttemptPlateCounts.Init(0, ClampedNumPlates);

		for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
		{
			double BestDot = -TNumericLimits<double>::Max();
			double SecondBestDot = -TNumericLimits<double>::Max();
			int32 BestPlateIndex = 0;

			const FVector& Position = Samples[SampleIndex].Position;
			for (int32 PlateIndex = 0; PlateIndex < AttemptCentroids.Num(); ++PlateIndex)
			{
				const double Dot = FVector::DotProduct(Position, AttemptCentroids[PlateIndex]);
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

			AttemptPlateAssignments[SampleIndex] = BestPlateIndex;
			AttemptBestDots[SampleIndex] = BestDot;
			AttemptSecondBestDots[SampleIndex] = SecondBestDot;
			++AttemptPlateCounts[BestPlateIndex];
		}

		int32 MinPlateCount = TNumericLimits<int32>::Max();
		for (const int32 PlateCount : AttemptPlateCounts)
		{
			MinPlateCount = FMath::Min(MinPlateCount, PlateCount);
		}
		BestRejectedMinPlateCount = FMath::Max(BestRejectedMinPlateCount, MinPlateCount);

		if (MinPlateCount < InitialPlateFloorSamples)
		{
			continue;
		}

		SelectedCentroids = MoveTemp(AttemptCentroids);
		SelectedPlateAssignments = MoveTemp(AttemptPlateAssignments);
		SelectedBestDots = MoveTemp(AttemptBestDots);
		SelectedSecondBestDots = MoveTemp(AttemptSecondBestDots);
		SelectedPlateCounts = MoveTemp(AttemptPlateCounts);
		bFoundValidLayout = true;
		break;
	}

	checkf(bFoundValidLayout,
		TEXT("InitializePlates failed to find a valid farthest-point plate layout after %d attempts (seed=%d, requested plates=%d, floor=%d, best rejected min=%d)."),
		MaxInitializationAttempts,
		RandomSeed,
		ClampedNumPlates,
		InitialPlateFloorSamples,
		BestRejectedMinPlateCount);

	Plates.Reset();
	Plates.SetNum(ClampedNumPlates);
	int32 InitialSampleCountSum = 0;
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		FPlate& Plate = Plates[PlateIndex];
		Plate = FPlate{};
		Plate.Id = PlateIndex;
		Plate.PersistencePolicy = EPlatePersistencePolicy::Protected;
		Plate.IdentityAnchorDirection = SelectedCentroids[PlateIndex];
		Plate.CanonicalCenterDirection = SelectedCentroids[PlateIndex];
		Plate.InitialSampleCount = SelectedPlateCounts[PlateIndex];
		InitialSampleCountSum += Plate.InitialSampleCount;
		Plate.RotationAxis = MakeRandomUnitVector(PlateMotionRng).GetSafeNormal();
		Plate.AngularSpeed = PlateMotionRng.FRandRange(0.5f, 1.5f) * 3.14e-2f;
	}
	InitialMeanPlateAreaKm2 = (Plates.Num() > 0)
		? (static_cast<double>(InitialSampleCountSum) * AverageCellAreaKm2 / static_cast<double>(Plates.Num()))
		: 0.0;

	const int32 LowPlateFallbackMin = (ClampedNumPlates < 7)
		? 1
		: FMath::Max(2, FMath::RoundToInt(ClampedMinContinentalPlateFraction * static_cast<double>(ClampedNumPlates)));
	const int32 TargetContinentalPlateCount = FMath::Clamp(
		FMath::RoundToInt(ClampedTargetContinentalAreaFraction * static_cast<double>(ClampedNumPlates)),
		LowPlateFallbackMin,
		ClampedNumPlates);

	struct FContinentalCandidateLayout
	{
		TArray<int32> PlateIds;
		int32 ContinentalSampleCount = 0;
		double ContinentalSampleFraction = 0.0;
		double MinPairwiseAngularSeparation = 0.0;
	};

	auto ComputeMinPairwiseAngularSeparation = [this](const TArray<int32>& PlateIds) -> double
	{
		double MinAngle = PI;
		if (PlateIds.Num() <= 1)
		{
			return MinAngle;
		}

		for (int32 PlateIdIndexA = 0; PlateIdIndexA < PlateIds.Num(); ++PlateIdIndexA)
		{
			const int32 PlateIdA = PlateIds[PlateIdIndexA];
			if (!Plates.IsValidIndex(PlateIdA))
			{
				continue;
			}

			for (int32 PlateIdIndexB = PlateIdIndexA + 1; PlateIdIndexB < PlateIds.Num(); ++PlateIdIndexB)
			{
				const int32 PlateIdB = PlateIds[PlateIdIndexB];
				if (!Plates.IsValidIndex(PlateIdB))
				{
					continue;
				}

				const double Dot = FMath::Clamp(
					static_cast<double>(FVector::DotProduct(Plates[PlateIdA].CanonicalCenterDirection, Plates[PlateIdB].CanonicalCenterDirection)),
					-1.0,
					1.0);
				MinAngle = FMath::Min(MinAngle, FMath::Acos(Dot));
			}
		}

		return MinAngle;
	};

	auto IsLexicographicallyLowerPlateList = [](const TArray<int32>& CandidatePlateIds, const TArray<int32>& BestPlateIds) -> bool
	{
		if (BestPlateIds.Num() <= 0)
		{
			return true;
		}

		const int32 CompareCount = FMath::Min(CandidatePlateIds.Num(), BestPlateIds.Num());
		for (int32 CompareIndex = 0; CompareIndex < CompareCount; ++CompareIndex)
		{
			if (CandidatePlateIds[CompareIndex] != BestPlateIds[CompareIndex])
			{
				return CandidatePlateIds[CompareIndex] < BestPlateIds[CompareIndex];
			}
		}

		return CandidatePlateIds.Num() < BestPlateIds.Num();
	};

	FContinentalCandidateLayout BestContinentalLayout;
	bool bFoundContinentalLayout = false;
	for (int32 StartPlateId = 0; StartPlateId < ClampedNumPlates; ++StartPlateId)
	{
		FContinentalCandidateLayout CandidateLayout;
		CandidateLayout.PlateIds.Reserve(TargetContinentalPlateCount);
		CandidateLayout.PlateIds.Add(StartPlateId);
		while (CandidateLayout.PlateIds.Num() < TargetContinentalPlateCount)
		{
			double BestNearestSelectedDot = TNumericLimits<double>::Max();
			int32 BestCandidatePlateId = INDEX_NONE;
			for (int32 CandidatePlateId = 0; CandidatePlateId < ClampedNumPlates; ++CandidatePlateId)
			{
				if (CandidateLayout.PlateIds.Contains(CandidatePlateId))
				{
					continue;
				}

				double NearestSelectedDot = -TNumericLimits<double>::Max();
				for (const int32 SelectedPlateId : CandidateLayout.PlateIds)
				{
					NearestSelectedDot = FMath::Max(
						NearestSelectedDot,
						static_cast<double>(FVector::DotProduct(Plates[CandidatePlateId].CanonicalCenterDirection, Plates[SelectedPlateId].CanonicalCenterDirection)));
				}

				if (BestCandidatePlateId == INDEX_NONE ||
					NearestSelectedDot < BestNearestSelectedDot - UE_DOUBLE_SMALL_NUMBER ||
					(FMath::IsNearlyEqual(NearestSelectedDot, BestNearestSelectedDot, UE_DOUBLE_SMALL_NUMBER) && CandidatePlateId < BestCandidatePlateId))
				{
					BestNearestSelectedDot = NearestSelectedDot;
					BestCandidatePlateId = CandidatePlateId;
				}
			}

			check(BestCandidatePlateId != INDEX_NONE);
			CandidateLayout.PlateIds.Add(BestCandidatePlateId);
		}

		CandidateLayout.PlateIds.Sort();
		for (const int32 PlateId : CandidateLayout.PlateIds)
		{
			CandidateLayout.ContinentalSampleCount += Plates[PlateId].InitialSampleCount;
		}

		CandidateLayout.ContinentalSampleFraction =
			(Samples.Num() > 0)
				? (static_cast<double>(CandidateLayout.ContinentalSampleCount) / static_cast<double>(Samples.Num()))
				: 0.0;
		if (CandidateLayout.ContinentalSampleFraction + UE_DOUBLE_SMALL_NUMBER < ClampedMinContinentalAreaFraction ||
			CandidateLayout.ContinentalSampleFraction - UE_DOUBLE_SMALL_NUMBER > ClampedMaxContinentalAreaFraction)
		{
			continue;
		}

		CandidateLayout.MinPairwiseAngularSeparation = ComputeMinPairwiseAngularSeparation(CandidateLayout.PlateIds);
		const double CandidateFractionError = FMath::Abs(CandidateLayout.ContinentalSampleFraction - ClampedTargetContinentalAreaFraction);
		const double BestFractionError = FMath::Abs(BestContinentalLayout.ContinentalSampleFraction - ClampedTargetContinentalAreaFraction);
		const bool bBetterCandidate =
			!bFoundContinentalLayout ||
			(CandidateFractionError < BestFractionError - UE_DOUBLE_SMALL_NUMBER) ||
			(FMath::IsNearlyEqual(CandidateFractionError, BestFractionError, UE_DOUBLE_SMALL_NUMBER) &&
				CandidateLayout.MinPairwiseAngularSeparation > BestContinentalLayout.MinPairwiseAngularSeparation + UE_DOUBLE_SMALL_NUMBER) ||
			(FMath::IsNearlyEqual(CandidateFractionError, BestFractionError, UE_DOUBLE_SMALL_NUMBER) &&
				FMath::IsNearlyEqual(CandidateLayout.MinPairwiseAngularSeparation, BestContinentalLayout.MinPairwiseAngularSeparation, UE_DOUBLE_SMALL_NUMBER) &&
				IsLexicographicallyLowerPlateList(CandidateLayout.PlateIds, BestContinentalLayout.PlateIds));
		if (bBetterCandidate)
		{
			BestContinentalLayout = MoveTemp(CandidateLayout);
			bFoundContinentalLayout = true;
		}
	}

	checkf(
		bFoundContinentalLayout,
		TEXT("InitializePlates failed to find a continental plate layout within the configured area band (seed=%d, plates=%d, target plates=%d, target area=%.3f, band=[%.3f, %.3f])."),
		RandomSeed,
		ClampedNumPlates,
		TargetContinentalPlateCount,
		ClampedTargetContinentalAreaFraction,
		ClampedMinContinentalAreaFraction,
		ClampedMaxContinentalAreaFraction);

	TSet<int32> ContinentalPlates;
	for (const int32 ContinentalPlateId : BestContinentalLayout.PlateIds)
	{
		ContinentalPlates.Add(ContinentalPlateId);
	}
	const int32 NumContinentalPlates = ContinentalPlates.Num();
	InitialContinentalPlateCount = NumContinentalPlates;

	int32 NumAssignedContinental = 0;
	int32 NumAssignedOceanic = 0;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const double BestDot = SelectedBestDots[SampleIndex];
		const double SecondBestDot = SelectedSecondBestDots[SampleIndex];
		const int32 BestPlateIndex = SelectedPlateAssignments[SampleIndex];

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
			Sample.Elevation = InitialOceanicPlateElevationKm;
			Sample.Thickness = OceanicCrustThicknessKm;
			Sample.TerraneId = -1;
			++NumAssignedOceanic;
		}
	}

	ResetDisplacementTracking();
	RebuildPlateMembershipFromSamples(Samples);
	UpdatePlateCanonicalCentersFromSamples(Samples);
	ClassifyTrianglesForSamples(Samples);
	DetectTerranesForSamples(Samples);
	UpdateSubductionFieldsForSamples(Samples);
	RebuildCarriedSampleWorkspacesForSamples(Samples);
	InvalidateSpatialQueryData();
	RebuildSpatialQueryData();
	SampleBuffers[1 - ReadableSampleBufferIndex.Load()] = Samples;

	TArray<int32> InitialPlateSizes = SelectedPlateCounts;
	InitialPlateSizes.Sort();
	const int32 MinInitialPlateSize = InitialPlateSizes.Num() > 0 ? InitialPlateSizes[0] : 0;
	const int32 MedianInitialPlateSize = InitialPlateSizes.Num() > 0 ? InitialPlateSizes[InitialPlateSizes.Num() / 2] : 0;
	const int32 MaxInitialPlateSize = InitialPlateSizes.Num() > 0 ? InitialPlateSizes.Last() : 0;
	
const double InitialContinentalAreaFractionPct = (Samples.Num() > 0)
		? (100.0 * static_cast<double>(NumAssignedContinental) / static_cast<double>(Samples.Num()))
		: 0.0;
	UE_LOG(LogTemp, Log,
		TEXT("InitializePlates: plates=%d seed=%d initialSize[min/median/max]=%d/%d/%d continentalArea=%.3f%% continentalPlates=%d"),
		ClampedNumPlates,
		RandomSeed,
		MinInitialPlateSize,
		MedianInitialPlateSize,
		MaxInitialPlateSize,
		InitialContinentalAreaFractionPct,
		InitialContinentalPlateCount);
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

	constexpr double MmToKm = 1.0e-6;
	const double DampeningStepKm = static_cast<double>(OceanicDampeningRateMmPerYear) * MmToKm * TimestepDurationYears;
	const double BaseSubductionUpliftStepKm = static_cast<double>(BaseSubductionUpliftMmPerYear) * MmToKm * TimestepDurationYears;
	const TArray<FCanonicalSample>& ReadSamples = GetReadableSamplesInternal();

	for (FPlate& Plate : Plates)
	{
		const FVector Axis = Plate.RotationAxis.GetSafeNormal();
		const FQuat DeltaRotation(Axis, Plate.AngularSpeed);
		Plate.CumulativeRotation = (DeltaRotation * Plate.CumulativeRotation).GetNormalized();

		Plate.AngularDisplacementSinceReconcile += FMath::Abs(static_cast<double>(Plate.AngularSpeed));
		MaxAngularDisplacementSinceReconcile = FMath::Max(MaxAngularDisplacementSinceReconcile, Plate.AngularDisplacementSinceReconcile);

		for (FCarriedSampleData& CarriedSample : Plate.CarriedSamples)
		{
			const FCanonicalSample* CanonicalSample = ReadSamples.IsValidIndex(CarriedSample.CanonicalSampleIndex)
				? &ReadSamples[CarriedSample.CanonicalSampleIndex]
				: nullptr;
			const FVector MovedPosition = CanonicalSample
				? Plate.CumulativeRotation.RotateVector(CanonicalSample->Position).GetSafeNormal()
				: FVector::ZeroVector;

			if (CarriedSample.CrustType == ECrustType::Oceanic)
			{
				CarriedSample.Age += static_cast<float>(TimestepDurationMy);
				const double ElevationKm = static_cast<double>(CarriedSample.Elevation);
				const double DampeningMultiplier = 1.0 - (ElevationKm / static_cast<double>(OceanicTrenchElevationKm));
				const double UpdatedElevationKm = ElevationKm - DampeningMultiplier * DampeningStepKm;
				CarriedSample.Elevation = static_cast<float>(UpdatedElevationKm);
			}
			else
			{
				// TODO[M6]: Continental erosion hook:
				// z -= (z / z_c) * epsilon_c * delta_t
				// TODO[M6]: Sediment accretion hook:
				// z += epsilon_f * delta_t
			}

			bool bAppliedAndeanUplift = false;
			if (CarriedSample.SubductionRole == ESubductionRole::Overriding &&
				CarriedSample.CrustType == ECrustType::Continental &&
				CarriedSample.SubductionDistanceKm >= 0.0f &&
				Plates.IsValidIndex(CarriedSample.SubductionOpposingPlateId) &&
				!MovedPosition.IsNearlyZero())
			{
				const float DistanceInfluence = ComputeSubductionInfluence(
					CarriedSample.SubductionDistanceKm,
					SubductionPeakDistanceKm,
					SubductionInfluenceRadiusKm);
				const float SpeedInfluence = ComputeSpeedInfluence(
					CarriedSample.SubductionConvergenceSpeedMmPerYear,
					MaxSubductionSpeedMmPerYear);
				const float ElevationInfluence = ComputeElevationInfluence(
					CarriedSample.Elevation,
					OceanicTrenchElevationKm,
					MaxContinentalElevationKm);
				const float UpliftStepKm = static_cast<float>(BaseSubductionUpliftStepKm) * DistanceInfluence * SpeedInfluence * ElevationInfluence;
				CarriedSample.Elevation += UpliftStepKm;

				const FVector OverridingVelocity = ComputeSurfaceVelocity(Plate.Id, MovedPosition);
				const FVector SubductingVelocity = ComputeSurfaceVelocity(CarriedSample.SubductionOpposingPlateId, MovedPosition);
				FVector FoldTarget = SubductingVelocity - OverridingVelocity;
				FoldTarget = FoldTarget - FVector::DotProduct(FoldTarget, MovedPosition) * MovedPosition;
				FoldTarget = FoldTarget.GetSafeNormal();
				if (!FoldTarget.IsNearlyZero())
				{
					FVector ExistingFold = CarriedSample.FoldDirection;
					ExistingFold = ExistingFold - FVector::DotProduct(ExistingFold, MovedPosition) * MovedPosition;
					ExistingFold = ExistingFold.GetSafeNormal();
					if (ExistingFold.IsNearlyZero())
					{
						ExistingFold = FoldTarget;
					}

					const float BlendAlpha = FoldBlendAtMaxSpeed * SpeedInfluence;
					FVector BlendedFold = FMath::Lerp(ExistingFold, FoldTarget, BlendAlpha);
					BlendedFold = BlendedFold - FVector::DotProduct(BlendedFold, MovedPosition) * MovedPosition;
					CarriedSample.FoldDirection = BlendedFold.GetSafeNormal();
				}

				if (CarriedSample.OrogenyType == EOrogenyType::None)
				{
					CarriedSample.OrogenyType = EOrogenyType::Andean;
					CarriedSample.OrogenyAge = 0.0f;
				}
				else
				{
					CarriedSample.OrogenyAge += static_cast<float>(TimestepDurationMy);
				}

				bAppliedAndeanUplift = true;
			}

			if (CarriedSample.SubductionRole == ESubductionRole::Subducting &&
				CarriedSample.CrustType == ECrustType::Oceanic &&
				CarriedSample.SubductionDistanceKm >= 0.0f)
			{
				const float SpeedInfluence = ComputeSpeedInfluence(
					CarriedSample.SubductionConvergenceSpeedMmPerYear,
					MaxSubductionSpeedMmPerYear);
				const float TrenchStepKm =
					0.5f *
					static_cast<float>(BaseSubductionUpliftStepKm) *
					ComputeTrenchInfluence(CarriedSample.SubductionDistanceKm, SubductionTrenchRadiusKm) *
					SpeedInfluence;
				CarriedSample.Elevation = FMath::Max(CarriedSample.Elevation - TrenchStepKm, OceanicTrenchElevationKm);
			}

			if (!bAppliedAndeanUplift && CarriedSample.OrogenyType != EOrogenyType::None)
			{
				CarriedSample.OrogenyAge += static_cast<float>(TimestepDurationMy);
			}
		}

		if (Plate.CarriedSamples.Num() > 0 && !Plate.CanonicalCenterDirection.IsNearlyZero())
		{
			const FVector MovedCenter = Plate.CumulativeRotation.RotateVector(Plate.CanonicalCenterDirection).GetSafeNormal();
			FVector MeanSlabPullVote = FVector::ZeroVector;
			float SlabPullWeightSum = 0.0f;
			int32 NumSubductingFrontVotes = 0;
			for (const FCarriedSampleData& CarriedSample : Plate.CarriedSamples)
			{
				if (CarriedSample.SubductionRole != ESubductionRole::Subducting || !CarriedSample.bIsSubductionFront)
				{
					continue;
				}
				if (!ReadSamples.IsValidIndex(CarriedSample.CanonicalSampleIndex))
				{
					continue;
				}

				const FVector FrontPosition = Plate.CumulativeRotation.RotateVector(ReadSamples[CarriedSample.CanonicalSampleIndex].Position).GetSafeNormal();
				FVector VoteDirection = FVector::CrossProduct(MovedCenter, FrontPosition);
				VoteDirection = VoteDirection.GetSafeNormal();
				const float VoteWeight = ComputeSpeedInfluence(
					CarriedSample.SubductionConvergenceSpeedMmPerYear,
					MaxSubductionSpeedMmPerYear);
				if (!VoteDirection.IsNearlyZero() && VoteWeight > UE_SMALL_NUMBER)
				{
					MeanSlabPullVote += VoteDirection * VoteWeight;
					SlabPullWeightSum += VoteWeight;
					++NumSubductingFrontVotes;
				}
			}

			MeanSlabPullVote = MeanSlabPullVote.GetSafeNormal();
			if (!MeanSlabPullVote.IsNearlyZero() && SlabPullWeightSum > UE_SMALL_NUMBER)
			{
				const float FrontCoverage = static_cast<float>(NumSubductingFrontVotes) / static_cast<float>(Plate.CarriedSamples.Num());
				const float MeanSpeedInfluence = SlabPullWeightSum / static_cast<float>(NumSubductingFrontVotes);
				const float PullStrength = FMath::Clamp(
					MeanSpeedInfluence * FMath::Sqrt(FMath::Max(FrontCoverage, 0.0f)),
					0.0f,
					1.0f);
				if (PullStrength <= UE_SMALL_NUMBER)
				{
					continue;
				}

				FVector CurrentAxis = Plate.RotationAxis.GetSafeNormal();
				float Dot = FVector::DotProduct(CurrentAxis, MeanSlabPullVote);
				Dot = FMath::Clamp(Dot, -1.0f, 1.0f);
				const float Angle = FMath::Acos(Dot);
				if (Angle > KINDA_SMALL_NUMBER)
				{
					const float MaxStepRadians = FMath::DegreesToRadians(SlabPullAxisMaxDegreesPerStep * PullStrength);
					const float RotationAngle = FMath::Min(Angle, MaxStepRadians);
					FVector RotationAxis = FVector::CrossProduct(CurrentAxis, MeanSlabPullVote).GetSafeNormal();
					if (RotationAxis.IsNearlyZero())
					{
						RotationAxis = FVector::CrossProduct(CurrentAxis, FVector::UpVector).GetSafeNormal();
						if (RotationAxis.IsNearlyZero())
						{
							RotationAxis = FVector::CrossProduct(CurrentAxis, FVector::RightVector).GetSafeNormal();
						}
					}

					if (!RotationAxis.IsNearlyZero())
					{
						const FQuat AxisAdjustment(RotationAxis, RotationAngle);
						Plate.RotationAxis = AxisAdjustment.RotateVector(CurrentAxis).GetSafeNormal();
					}
				}
			}
		}
	}

	RefreshSubductionMetricsFromCarriedSamples();

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
	const double CopyStart = FPlatformTime::Seconds();
	WriteSamples = ReadSamples;
	Timings.SampleBufferCopyMs = (FPlatformTime::Seconds() - CopyStart) * 1000.0;

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
	TArray<uint8> ActivePlateFlags;
	ActivePlateFlags.Init(0, Plates.Num());
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
					ActivePlateFlags[PlateState.PlateId] = 1;
				}
			}
		}
	}

	for (const FPlate& Plate : Plates)
	{
		if (PlateCapCenters[Plate.Id].IsNearlyZero() && Plate.SampleIndices.Num() > 0)
		{
			const FVector FallbackCenter = Plate.CumulativeRotation.RotateVector(Plate.CanonicalCenterDirection).GetSafeNormal();
			if (!FallbackCenter.IsNearlyZero())
			{
				PlateCapCenters[Plate.Id] = FallbackCenter;
				ActivePlateFlags[Plate.Id] = 1;
			}
		}
	}

	const auto ComputeMarginFromCapCenters = [this, &PlateCapCenters, &ActivePlateFlags](const FVector& Direction) -> float
	{
		double BestDot = -TNumericLimits<double>::Max();
		double SecondBestDot = -TNumericLimits<double>::Max();
		for (int32 PlateId = 0; PlateId < PlateCapCenters.Num(); ++PlateId)
		{
			if (!ActivePlateFlags.IsValidIndex(PlateId) || ActivePlateFlags[PlateId] == 0)
			{
				continue;
			}
			const FVector& Center = PlateCapCenters[PlateId];
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

	const FSpatialQueryData* LocalSpatialQueryData = SpatialQueryData.Get();
	auto QuerySinglePlateContainmentFromSpatialData =
		[this, &SpatialStateByPlateId, &ReadSamples](const FVector& Position, const int32 PlateId, FContainmentQueryResult& OutResult) -> bool
	{
		OutResult = FContainmentQueryResult{};
		if (!IsPlateActive(PlateId) || !SpatialStateByPlateId.IsValidIndex(PlateId))
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
		FVector3d A; FVector3d B; FVector3d C;
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
		const FVector3d Barycentric = NormalizeContainmentBarycentric(ComputePlanarBarycentric(A, B, C, QueryPointLocal));
		const int32 GlobalTriangleIndex = PlateState->SoupData.GlobalTriangleIndices[LocalTriangleId];
		if (!Triangles.IsValidIndex(GlobalTriangleIndex) || !IsPreferredPlateForTriangle(ReadSamples, Triangles[GlobalTriangleIndex], PlateId, Barycentric))
		{
			return false;
		}
		OutResult.Barycentric = FVector(Barycentric.X, Barycentric.Y, Barycentric.Z);
		return true;
	};
	auto QueryContainmentFromSpatialData = [this, LocalSpatialQueryData, &ReadSamples](const FVector& Position) -> FContainmentQueryResult
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
			if (!PlateState.bHasValidCap || !IsPlateActive(PlateState.PlateId))
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
			FVector3d A; FVector3d B; FVector3d C;
			int32 LocalTriangleId = INDEX_NONE;
			if (!FindContainingTriangleInBVH(*PlateState.BVH, *PlateState.Adapter, QueryPointLocal, LocalTriangleId, A, B, C))
			{
				continue;
			}
			const FVector3d Barycentric = NormalizeContainmentBarycentric(ComputePlanarBarycentric(A, B, C, QueryPointLocal));
			const int32 GlobalTriangleIndex = PlateState.SoupData.GlobalTriangleIndices[LocalTriangleId];
			if (!Triangles.IsValidIndex(GlobalTriangleIndex) || !IsPreferredPlateForTriangle(ReadSamples, Triangles[GlobalTriangleIndex], PlateState.PlateId, Barycentric))
			{
				continue;
			}
			++Result.NumContainingPlates;
			if (Result.NumContainingPlates <= UE_ARRAY_COUNT(Result.ContainingPlateIds))
			{
				Result.ContainingPlateIds[Result.NumContainingPlates - 1] = PlateState.PlateId;
			}
			const double ContainmentScore = ComputeContainmentScore(Barycentric);
			if (IsBetterContainmentCandidate(ContainmentScore, PlateState.PlateId, BestContainmentScore, Result.PlateId))
			{
				BestContainmentScore = ContainmentScore;
				Result.PlateId = PlateState.PlateId;
				Result.TriangleIndex = GlobalTriangleIndex;
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
		if (!bBoundaryLike && Plates.IsValidIndex(SourceSample.PrevPlateId))
		{
			FContainmentQueryResult FastPathResult;
			if (QuerySinglePlateContainmentFromSpatialData(SourceSample.Position, SourceSample.PrevPlateId, FastPathResult))
			{
				State.bGap = false; State.bOverlap = false;
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
		State.bGap = Containment.bGap; State.bOverlap = Containment.bOverlap;
		State.TriangleIndex = Containment.TriangleIndex; State.Barycentric = Containment.Barycentric;
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
		if (!bBoundaryLike || CandidatePlateId == SourceSample.PrevPlateId)
		{
			State.AssignedPlateId = CandidatePlateId;
			return;
		}
		bool bPreviousOwnerStillContains = false;
		for (int32 IdIndex = 0; IdIndex < UE_ARRAY_COUNT(State.ContainingPlateIds); ++IdIndex)
		{
			if (State.ContainingPlateIds[IdIndex] == SourceSample.PrevPlateId)
			{
				bPreviousOwnerStillContains = true;
				break;
			}
		}
		State.AssignedPlateId = (!bPreviousOwnerStillContains || Margin > HysteresisThreshold) ? CandidatePlateId : SourceSample.PrevPlateId;
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
		auto CopyFromCarriedSample = [&ReadSamples, &DestSample](const FCarriedSampleData& SourceCarried)
		{
			DestSample.Elevation = SourceCarried.Elevation; DestSample.Thickness = SourceCarried.Thickness; DestSample.Age = SourceCarried.Age;
			DestSample.OrogenyAge = SourceCarried.OrogenyAge; DestSample.CrustType = SourceCarried.CrustType; DestSample.OrogenyType = SourceCarried.OrogenyType;
			DestSample.RidgeDirection = SourceCarried.RidgeDirection; DestSample.FoldDirection = SourceCarried.FoldDirection;
			DestSample.SubductionRole = SourceCarried.SubductionRole; DestSample.SubductionOpposingPlateId = SourceCarried.SubductionOpposingPlateId;
			DestSample.SubductionDistanceKm = SourceCarried.SubductionDistanceKm; DestSample.SubductionConvergenceSpeedMmPerYear = SourceCarried.SubductionConvergenceSpeedMmPerYear;
			DestSample.bIsSubductionFront = SourceCarried.bIsSubductionFront; DestSample.CollisionDistanceKm = SourceCarried.CollisionDistanceKm;
			DestSample.CollisionConvergenceSpeedMmPerYear = SourceCarried.CollisionConvergenceSpeedMmPerYear; DestSample.bIsCollisionFront = SourceCarried.bIsCollisionFront;
			DestSample.TerraneId = ReadSamples.IsValidIndex(SourceCarried.CanonicalSampleIndex) ? ReadSamples[SourceCarried.CanonicalSampleIndex].TerraneId : INDEX_NONE;
		};
		auto TryCopyNearestAssignedPlateSample = [this, &ReadSamples, &SourceSample, &CopyFromCarriedSample](const int32 PlateId) -> bool
		{
			if (!Plates.IsValidIndex(PlateId)) { return false; }
			const FPlate& Plate = Plates[PlateId]; const FVector SampleDirection = SourceSample.Position.GetSafeNormal();
			const FCarriedSampleData* BestCarried = nullptr; double BestDot = -TNumericLimits<double>::Max();
			for (const FCarriedSampleData& CandidateCarried : Plate.CarriedSamples)
			{
				if (!ReadSamples.IsValidIndex(CandidateCarried.CanonicalSampleIndex)) { continue; }
				const double CandidateDot = FVector::DotProduct(SampleDirection, ReadSamples[CandidateCarried.CanonicalSampleIndex].Position.GetSafeNormal());
				if (!BestCarried || CandidateDot > BestDot + UE_DOUBLE_SMALL_NUMBER || (FMath::IsNearlyEqual(CandidateDot, BestDot, UE_DOUBLE_SMALL_NUMBER) && CandidateCarried.CanonicalSampleIndex < BestCarried->CanonicalSampleIndex))
				{ BestCarried = &CandidateCarried; BestDot = CandidateDot; }
			}
			if (!BestCarried) { return false; }
			CopyFromCarriedSample(*BestCarried); return true;
		};
		DestSample.PlateId = State.AssignedPlateId; DestSample.PrevPlateId = SourceSample.PrevPlateId; DestSample.OwnershipMargin = State.OwnershipMargin;
		DestSample.BoundaryNormal = FVector::ZeroVector; DestSample.BoundaryType = EBoundaryType::None; DestSample.bGapDetected = false;
		DestSample.FlankingPlateIdA = INDEX_NONE; DestSample.FlankingPlateIdB = INDEX_NONE; DestSample.bOverlapDetected = false;
		DestSample.SubductionRole = ESubductionRole::None; DestSample.SubductionOpposingPlateId = INDEX_NONE; DestSample.SubductionDistanceKm = -1.0f;
		DestSample.SubductionConvergenceSpeedMmPerYear = 0.0f; DestSample.bIsSubductionFront = false; DestSample.CollisionDistanceKm = -1.0f;
		DestSample.CollisionConvergenceSpeedMmPerYear = 0.0f; DestSample.bIsCollisionFront = false; DestSample.NumOverlapPlateIds = 0;
		for (int32 OverlapIndex = 0; OverlapIndex < UE_ARRAY_COUNT(DestSample.OverlapPlateIds); ++OverlapIndex) { DestSample.OverlapPlateIds[OverlapIndex] = INDEX_NONE; }
		if (State.bGap || !Plates.IsValidIndex(State.AssignedPlateId) || !Triangles.IsValidIndex(State.TriangleIndex)) { return; }
		const FDelaunayTriangle& Triangle = Triangles[State.TriangleIndex]; const TMap<int32, const FCarriedSampleData*>& Lookup = CarriedLookupByPlate[State.AssignedPlateId];
		const FCarriedSampleData* V0 = Lookup.FindRef(Triangle.V[0]); const FCarriedSampleData* V1 = Lookup.FindRef(Triangle.V[1]); const FCarriedSampleData* V2 = Lookup.FindRef(Triangle.V[2]);
		if (!V0 || !V1 || !V2) { TryCopyNearestAssignedPlateSample(State.AssignedPlateId); return; }
		const FVector Bary = State.Barycentric; const float W0 = Bary.X; const float W1 = Bary.Y; const float W2 = Bary.Z;
		DestSample.Elevation = W0 * V0->Elevation + W1 * V1->Elevation + W2 * V2->Elevation;
		DestSample.Thickness = W0 * V0->Thickness + W1 * V1->Thickness + W2 * V2->Thickness;
		DestSample.Age = W0 * V0->Age + W1 * V1->Age + W2 * V2->Age;
		DestSample.OrogenyAge = W0 * V0->OrogenyAge + W1 * V1->OrogenyAge + W2 * V2->OrogenyAge;
		if (W0 >= W1 && W0 >= W2) { DestSample.CrustType = V0->CrustType; DestSample.OrogenyType = V0->OrogenyType; }
		else if (W1 >= W0 && W1 >= W2) { DestSample.CrustType = V1->CrustType; DestSample.OrogenyType = V1->OrogenyType; }
		else { DestSample.CrustType = V2->CrustType; DestSample.OrogenyType = V2->OrogenyType; }
		DestSample.RidgeDirection = ResolveDirectionField(DestSample.Position, V0->RidgeDirection, V1->RidgeDirection, V2->RidgeDirection, Bary);
		DestSample.FoldDirection = ResolveDirectionField(DestSample.Position, V0->FoldDirection, V1->FoldDirection, V2->FoldDirection, Bary);
		const int32 TerraneId0 = ReadSamples[Triangle.V[0]].TerraneId; const int32 TerraneId1 = ReadSamples[Triangle.V[1]].TerraneId; const int32 TerraneId2 = ReadSamples[Triangle.V[2]].TerraneId;
		if (W0 >= W1 && W0 >= W2) { DestSample.TerraneId = TerraneId0; DestSample.CollisionDistanceKm = V0->CollisionDistanceKm; DestSample.CollisionConvergenceSpeedMmPerYear = V0->CollisionConvergenceSpeedMmPerYear; DestSample.bIsCollisionFront = V0->bIsCollisionFront; }
		else if (W1 >= W0 && W1 >= W2) { DestSample.TerraneId = TerraneId1; DestSample.CollisionDistanceKm = V1->CollisionDistanceKm; DestSample.CollisionConvergenceSpeedMmPerYear = V1->CollisionConvergenceSpeedMmPerYear; DestSample.bIsCollisionFront = V1->bIsCollisionFront; }
		else { DestSample.TerraneId = TerraneId2; DestSample.CollisionDistanceKm = V2->CollisionDistanceKm; DestSample.CollisionConvergenceSpeedMmPerYear = V2->CollisionConvergenceSpeedMmPerYear; DestSample.bIsCollisionFront = V2->bIsCollisionFront; }
	});
	Timings.Phase3InterpolationMs = (FPlatformTime::Seconds() - Phase3Start) * 1000.0;

	const double Phase4Start = FPlatformTime::Seconds();
	TArray<uint8> GapFlags; GapFlags.Init(0, Phase2States.Num());
	for (int32 SampleIndex = 0; SampleIndex < Phase2States.Num(); ++SampleIndex) { GapFlags[SampleIndex] = Phase2States[SampleIndex].bGap ? 1 : 0; }
	int32 GapSamples = 0; int32 ArtifactGapResolvedSamples = 0; int32 DivergentGapSamples = 0;
	for (int32 SampleIndex = 0; SampleIndex < WriteSamples.Num(); ++SampleIndex)
	{
		if (!Phase2States[SampleIndex].bGap) { continue; }
		bool bDivergentGapCreated = false; ResolveGapSamplePhase4(SampleIndex, GapFlags, WriteSamples, bDivergentGapCreated);
		if (bDivergentGapCreated) { ++GapSamples; ++DivergentGapSamples; } else { ++ArtifactGapResolvedSamples; }
	}
	Timings.GapSamples = GapSamples; Timings.ArtifactGapResolvedSamples = ArtifactGapResolvedSamples; Timings.DivergentGapSamples = DivergentGapSamples;
	Timings.Phase4GapMs = (FPlatformTime::Seconds() - Phase4Start) * 1000.0;
	const double StabilizerStart = FPlatformTime::Seconds();
	StabilizeContinentalCrustForSamples(ReadSamples, WriteSamples, &Timings);
	Timings.ContinentalStabilizerMs = (FPlatformTime::Seconds() - StabilizerStart) * 1000.0;

	const double Phase5Start = FPlatformTime::Seconds();
	int32 OverlapSamples = 0;
	for (int32 SampleIndex = 0; SampleIndex < WriteSamples.Num(); ++SampleIndex)
	{
		const FPhase2SampleState& State = Phase2States[SampleIndex]; FCanonicalSample& Sample = WriteSamples[SampleIndex];
		if (!State.bOverlap || State.NumContainingPlates < 2) { continue; }
		Sample.bOverlapDetected = true; Sample.NumOverlapPlateIds = static_cast<uint8>(FMath::Clamp(State.NumContainingPlates, 0, UE_ARRAY_COUNT(Sample.OverlapPlateIds)));
		for (int32 OverlapIndex = 0; OverlapIndex < UE_ARRAY_COUNT(Sample.OverlapPlateIds); ++OverlapIndex) { Sample.OverlapPlateIds[OverlapIndex] = State.ContainingPlateIds[OverlapIndex]; }
		++OverlapSamples;
	}
	Timings.OverlapSamples = OverlapSamples; Timings.Phase5OverlapMs = (FPlatformTime::Seconds() - Phase5Start) * 1000.0;

	const double Phase6SanitizeStart = FPlatformTime::Seconds();
	RescuedProtectedPlateCount = 0; RescuedProtectedSampleCount = 0; RepeatedlyRescuedProtectedSampleCount = 0;
	for (FCanonicalSample& Sample : WriteSamples) { if (!Plates.IsValidIndex(Sample.PlateId)) { Sample.PlateId = FindNearestPlateByCap(Sample.Position); } }
	RunBoundaryLikeLocalOwnershipSanitizePass(Phase2States, WriteSamples, 2);
	const double Phase6PersistenceStart = FPlatformTime::Seconds();
	EnforceConnectedPlateOwnershipForSamples(WriteSamples);
	const bool bRescuedProtectedOwnership = RescueProtectedPlateOwnershipForSamples(WriteSamples);
	if (bRescuedProtectedOwnership) { EnforceConnectedPlateOwnershipForSamples(WriteSamples); }
	Timings.Phase6PersistenceMs = (FPlatformTime::Seconds() - Phase6PersistenceStart) * 1000.0;
	RunBoundaryLikeLocalOwnershipSanitizePass(Phase2States, WriteSamples, 1);
	EnforceConnectedPlateOwnershipForSamples(WriteSamples);
	const bool bRescuedProtectedOwnershipAfterFinalSanitize = RescueProtectedPlateOwnershipForSamples(WriteSamples);
	if (bRescuedProtectedOwnershipAfterFinalSanitize) { EnforceConnectedPlateOwnershipForSamples(WriteSamples); }
	Timings.Phase6SanitizeOwnershipMs = (FPlatformTime::Seconds() - Phase6SanitizeStart) * 1000.0;

	const double Phase6MembershipRebuildStart = FPlatformTime::Seconds();
	RebuildPlateMembershipFromSamples(WriteSamples); UpdatePlateCanonicalCentersFromSamples(WriteSamples);
	Timings.Phase6RebuildMembershipMs = (FPlatformTime::Seconds() - Phase6MembershipRebuildStart) * 1000.0;
	const double Phase6ClassifyStart = FPlatformTime::Seconds();
	ClassifyTrianglesForSamples(WriteSamples); Timings.Phase6ClassifyTrianglesMs = (FPlatformTime::Seconds() - Phase6ClassifyStart) * 1000.0;
	ReadableSampleBufferIndex.Store(WriteBufferIndex);
	const double Phase6SpatialRebuildStart = FPlatformTime::Seconds();
	RebuildSpatialQueryData(); Timings.Phase6SpatialRebuildMs = (FPlatformTime::Seconds() - Phase6SpatialRebuildStart) * 1000.0;
	if (SpatialQueryData) { Timings.Phase6SpatialDirtyPlateCount = SpatialQueryData->LastDirtyPlateCount; Timings.Phase6SpatialRebuiltPlateCount = SpatialQueryData->LastRebuiltPlateCount; }

	const double Phase7TerraneStart = FPlatformTime::Seconds();
	DetectTerranesForSamples(WriteSamples); Timings.Phase7TerraneMs = (FPlatformTime::Seconds() - Phase7TerraneStart) * 1000.0;
	const double Phase8CollisionStart = FPlatformTime::Seconds();
	TArray<uint8> CollisionDirtyPlateFlags;
	const bool bAppliedCollision = ApplyContinentalCollisionEventsForSamples(WriteSamples, &CollisionDirtyPlateFlags);
	Timings.Phase8CollisionMs = (FPlatformTime::Seconds() - Phase8CollisionStart) * 1000.0;
	if (bAppliedCollision)
	{
		const double Phase8RefreshStart = FPlatformTime::Seconds();
		RefreshCanonicalStateAfterCollision(Phase2States, WriteSamples, &CollisionDirtyPlateFlags);
		Timings.Phase8PostCollisionRefreshMs = (FPlatformTime::Seconds() - Phase8RefreshStart) * 1000.0;
	}
	const double PrevPlateWritebackStart = FPlatformTime::Seconds();
	for (FCanonicalSample& Sample : WriteSamples) { Sample.PrevPlateId = Sample.PlateId; }
	Timings.PrevPlateWritebackMs = (FPlatformTime::Seconds() - PrevPlateWritebackStart) * 1000.0;
	const double Phase9Start = FPlatformTime::Seconds();
	UpdateSubductionFieldsForSamples(WriteSamples); Timings.Phase9SubductionMs = (FPlatformTime::Seconds() - Phase9Start) * 1000.0; Timings.Phase7SubductionMs = Timings.Phase9SubductionMs;
	const double Phase6CarriedStart = FPlatformTime::Seconds();
	RebuildCarriedSampleWorkspacesForSamples(WriteSamples); Timings.Phase6RebuildCarriedMs = (FPlatformTime::Seconds() - Phase6CarriedStart) * 1000.0;
	ReadableSampleBufferIndex.Store(WriteBufferIndex);
	Timings.Phase6MembershipMs = Timings.Phase6SanitizeOwnershipMs + Timings.Phase6RebuildMembershipMs + Timings.Phase6ClassifyTrianglesMs + Timings.Phase6RebuildCarriedMs + Timings.Phase6SpatialRebuildMs;

	LastGapSampleCount = GapSamples; LastOverlapSampleCount = OverlapSamples; LastReconcileTimings = Timings;
	LastReconcileTimings.TotalMs = (FPlatformTime::Seconds() - ReconcileStartTime) * 1000.0;
	LastReconcileTimings.PhaseSumMs = LastReconcileTimings.SampleBufferCopyMs + LastReconcileTimings.Phase1BuildSpatialMs + LastReconcileTimings.Phase2OwnershipMs + LastReconcileTimings.Phase3InterpolationMs + LastReconcileTimings.Phase4GapMs + LastReconcileTimings.ContinentalStabilizerMs + LastReconcileTimings.Phase5OverlapMs + LastReconcileTimings.Phase6MembershipMs + LastReconcileTimings.Phase7TerraneMs + LastReconcileTimings.Phase8CollisionMs + LastReconcileTimings.Phase8PostCollisionRefreshMs + LastReconcileTimings.PrevPlateWritebackMs + LastReconcileTimings.Phase9SubductionMs;
	LastReconcileTimings.UnaccountedMs = FMath::Max(0.0, LastReconcileTimings.TotalMs - LastReconcileTimings.PhaseSumMs);
	LastReconcileTimings.UnaccountedPct = (LastReconcileTimings.TotalMs > UE_DOUBLE_SMALL_NUMBER) ? (100.0 * LastReconcileTimings.UnaccountedMs / LastReconcileTimings.TotalMs) : 0.0;
	if (LastReconcileTimings.UnaccountedPct > MaxAcceptableReconcileUnaccountedPct)
	{
		UE_LOG(LogTemp, Warning, TEXT("Reconcile exceeded unaccounted budget: %.2f%% unaccounted (%.3f / %.3f ms)."), LastReconcileTimings.UnaccountedPct, LastReconcileTimings.UnaccountedMs, LastReconcileTimings.TotalMs);
	}

	++ReconcileCount;
	ResetDisplacementTracking();

	UE_LOG(LogTemp, Log, TEXT("Reconcile timings (ms): Copy=%.3f P1=%.3f P2=%.3f P3=%.3f P4=%.3f Stabilize=%.3f P5=%.3f P6=%.3f P7=%.3f P8=%.3f P8b=%.3f Writeback=%.3f P9=%.3f Sum=%.3f Unaccounted=%.3f (%.2f%%) Total=%.3f Gap=%d ArtifactResolved=%d DivergentGap=%d Overlap=%d"), LastReconcileTimings.SampleBufferCopyMs, LastReconcileTimings.Phase1BuildSpatialMs, LastReconcileTimings.Phase2OwnershipMs, LastReconcileTimings.Phase3InterpolationMs, LastReconcileTimings.Phase4GapMs, LastReconcileTimings.ContinentalStabilizerMs, LastReconcileTimings.Phase5OverlapMs, LastReconcileTimings.Phase6MembershipMs, LastReconcileTimings.Phase7TerraneMs, LastReconcileTimings.Phase8CollisionMs, LastReconcileTimings.Phase8PostCollisionRefreshMs, LastReconcileTimings.PrevPlateWritebackMs, LastReconcileTimings.Phase9SubductionMs, LastReconcileTimings.PhaseSumMs, LastReconcileTimings.UnaccountedMs, LastReconcileTimings.UnaccountedPct, LastReconcileTimings.TotalMs, LastReconcileTimings.GapSamples, LastReconcileTimings.ArtifactGapResolvedSamples, LastReconcileTimings.DivergentGapSamples, LastReconcileTimings.OverlapSamples);
	UE_LOG(LogTemp, Log, TEXT("Reconcile subphase (ms): P1 Soup sum/max=%.3f/%.3f Cap sum/max=%.3f/%.3f BVH sum/max=%.3f/%.3f | Stabilizer min/build/prune/promote/trim=%.3f/%.3f/%.3f/%.3f/%.3f counts[promote/component/trim]=%d/%d/%d heap[push/stale]=%d/%d | P6 Sanitize=%.3f Membership=%.3f Carried=%.3f Classify=%.3f SpatialRebuild=%.3f | P7 Terrane=%.3f P8 Collision=%.3f P8b Refresh=%.3f P9 Subduction=%.3f"), LastReconcileTimings.Phase1SoupExtractTotalMs, LastReconcileTimings.Phase1SoupExtractMaxMs, LastReconcileTimings.Phase1CapBuildTotalMs, LastReconcileTimings.Phase1CapBuildMaxMs, LastReconcileTimings.Phase1BVHBuildTotalMs, LastReconcileTimings.Phase1BVHBuildMaxMs, LastReconcileTimings.StabilizerPromoteToMinimumMs, LastReconcileTimings.StabilizerBuildComponentsMs, LastReconcileTimings.StabilizerComponentPruneMs, LastReconcileTimings.StabilizerPromoteToTargetMs, LastReconcileTimings.StabilizerTrimMs, LastReconcileTimings.StabilizerPromotionCount, LastReconcileTimings.StabilizerComponentDemotionCount, LastReconcileTimings.StabilizerTrimDemotionCount, LastReconcileTimings.StabilizerHeapPushCount, LastReconcileTimings.StabilizerHeapStalePopCount, LastReconcileTimings.Phase6SanitizeOwnershipMs, LastReconcileTimings.Phase6RebuildMembershipMs, LastReconcileTimings.Phase6RebuildCarriedMs, LastReconcileTimings.Phase6ClassifyTrianglesMs, LastReconcileTimings.Phase6SpatialRebuildMs, LastReconcileTimings.Phase7TerraneMs, LastReconcileTimings.Phase8CollisionMs, LastReconcileTimings.Phase8PostCollisionRefreshMs, LastReconcileTimings.Phase9SubductionMs);
	UE_LOG(LogTemp, Log, TEXT("Reconcile spatial rebuild stats: dirty_plates=%d rebuilt_plates=%d"), LastReconcileTimings.Phase6SpatialDirtyPlateCount, LastReconcileTimings.Phase6SpatialRebuiltPlateCount);
	UE_LOG(LogTemp, Log, TEXT("Reconcile phase2 fast-path: resolved=%d full_query=%d"), LastReconcileTimings.Phase2FastPathResolvedSamples, LastReconcileTimings.Phase2FullQuerySamples);
	const double SampleCountForDiagnostics = static_cast<double>(FMath::Max(1, WriteSamples.Num()));
	const double GapPct = 100.0 * static_cast<double>(LastReconcileTimings.GapSamples) / SampleCountForDiagnostics;
	const double OverlapPct = 100.0 * static_cast<double>(LastReconcileTimings.OverlapSamples) / SampleCountForDiagnostics;
	const double BoundaryDeepPct = (BoundarySampleCount > 0) ? (100.0 * static_cast<double>(BoundaryDeepSampleCount) / static_cast<double>(BoundarySampleCount)) : 0.0;
	const double LargestContinentalPct = (ContinentalSampleCount > 0) ? (100.0 * static_cast<double>(LargestContinentalComponentSize) / static_cast<double>(ContinentalSampleCount)) : 0.0;
	const double ContinentalAreaPct = 100.0 * ContinentalAreaFraction;
	UE_LOG(LogTemp, Log, TEXT("Reconcile diagnostics: Gap=%.3f%% Overlap=%.3f%% BoundaryMeanDepth=%.3f BoundaryMaxDepth=%d BoundaryDeep=%d (%.3f%%) ContinentalPlates=%d ContinentalArea=%.3f%% ContinentalComponents=%d LargestContinent=%d/%d (%.3f%%) MaxPlateComponents=%d DetachedPlateFragments=%d LargestDetachedFragment=%d SubductionFronts=%d Andean=%d PendingCollision=%d MaxSubductionDistance=%.1fkm"), GapPct, OverlapPct, BoundaryMeanDepthHops, BoundaryMaxDepthHops, BoundaryDeepSampleCount, BoundaryDeepPct, ContinentalPlateCount, ContinentalAreaPct, ContinentalComponentCount, LargestContinentalComponentSize, ContinentalSampleCount, LargestContinentalPct, MaxPlateComponentCount, DetachedPlateFragmentSampleCount, LargestDetachedPlateFragmentSize, SubductionFrontSampleCount, AndeanSampleCount, PendingCollisionSampleCount, MaxSubductionDistanceKm);
}

bool FTectonicPlanet::ResolveGapSamplePhase4(
	int32 SampleIndex,
	const TArray<uint8>& GapFlags,
	TArray<FCanonicalSample>& InOutSamples,
	bool& bOutDivergentGapCreated) const
{
	bOutDivergentGapCreated = false;
	if (!InOutSamples.IsValidIndex(SampleIndex) || !Adjacency.IsValidIndex(SampleIndex))
	{
		return false;
	}

	constexpr float RidgeTemplateElevationKm = RidgeElevationKm;
	constexpr float OceanicFallbackElevationKm = AbyssalPlainElevationKm;
	constexpr float OceanicThicknessKm = OceanicCrustThicknessKm;
	constexpr int32 MaxNeighborPlateBins = 16;
	constexpr double DivergentGapThresholdRatio = 0.3;

	FCanonicalSample& Sample = InOutSamples[SampleIndex];
	const FVector Position = Sample.Position.GetSafeNormal();
	const int32 PreviousOwner = Sample.PrevPlateId;

	auto ClearGapClassification = [&Sample]()
	{
		Sample.bGapDetected = false;
		Sample.FlankingPlateIdA = INDEX_NONE;
		Sample.FlankingPlateIdB = INDEX_NONE;
		Sample.RidgeDirection = FVector::ZeroVector;
	};

	auto CopyResolvedGapFromNeighbor = [&InOutSamples, &Sample, &ClearGapClassification](const int32 AssignedPlateId, const int32 SourceNeighborIndex)
	{
		if (AssignedPlateId != INDEX_NONE)
		{
			Sample.PlateId = AssignedPlateId;
		}
		ClearGapClassification();
		if (!InOutSamples.IsValidIndex(SourceNeighborIndex))
		{
			return;
		}

		const FCanonicalSample& SourceNeighbor = InOutSamples[SourceNeighborIndex];
		Sample.CrustType = SourceNeighbor.CrustType;
		Sample.Elevation = SourceNeighbor.Elevation;
		Sample.Thickness = SourceNeighbor.Thickness;
		Sample.Age = SourceNeighbor.Age;
		Sample.RidgeDirection = SourceNeighbor.RidgeDirection;
		Sample.FoldDirection = SourceNeighbor.FoldDirection;
		Sample.OrogenyType = SourceNeighbor.OrogenyType;
		Sample.OrogenyAge = SourceNeighbor.OrogenyAge;
		Sample.TerraneId = SourceNeighbor.TerraneId;
		Sample.CollisionDistanceKm = SourceNeighbor.CollisionDistanceKm;
		Sample.CollisionConvergenceSpeedMmPerYear = SourceNeighbor.CollisionConvergenceSpeedMmPerYear;
		Sample.bIsCollisionFront = SourceNeighbor.bIsCollisionFront;
	};

	if (Position.IsNearlyZero())
	{
		CopyResolvedGapFromNeighbor(Plates.IsValidIndex(PreviousOwner) ? PreviousOwner : FindNearestPlateByCap(Sample.Position), INDEX_NONE);
		return true;
	}

	TArray<int32, TInlineAllocator<32>> FirstRing;
	for (const int32 NeighborIndex : Adjacency[SampleIndex])
	{
		if (InOutSamples.IsValidIndex(NeighborIndex))
		{
			FirstRing.AddUnique(NeighborIndex);
		}
	}

	TArray<int32, TInlineAllocator<96>> Neighborhood;
	Neighborhood.Append(FirstRing);
	for (const int32 NeighborIndex : FirstRing)
	{
		if (!Adjacency.IsValidIndex(NeighborIndex))
		{
			continue;
		}

		for (const int32 SecondRingIndex : Adjacency[NeighborIndex])
		{
			if (SecondRingIndex != SampleIndex && InOutSamples.IsValidIndex(SecondRingIndex))
			{
				Neighborhood.AddUnique(SecondRingIndex);
			}
		}
	}

	int32 NeighborPlateIds[MaxNeighborPlateBins];
	int32 NeighborPlateCounts[MaxNeighborPlateBins];
	int32 NumNeighborPlates = 0;
	for (int32 SlotIndex = 0; SlotIndex < MaxNeighborPlateBins; ++SlotIndex)
	{
		NeighborPlateIds[SlotIndex] = INDEX_NONE;
		NeighborPlateCounts[SlotIndex] = 0;
	}

	auto AccumulateNeighborPlate = [&NeighborPlateIds, &NeighborPlateCounts, &NumNeighborPlates](const int32 PlateId)
	{
		int32 ExistingSlot = INDEX_NONE;
		for (int32 SlotIndex = 0; SlotIndex < NumNeighborPlates; ++SlotIndex)
		{
			if (NeighborPlateIds[SlotIndex] == PlateId)
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
			NeighborPlateIds[NumNeighborPlates] = PlateId;
			NeighborPlateCounts[NumNeighborPlates] = 1;
			++NumNeighborPlates;
		}
	};

	for (const int32 NeighborIndex : FirstRing)
	{
		if (!InOutSamples.IsValidIndex(NeighborIndex))
		{
			continue;
		}
		if (GapFlags.IsValidIndex(NeighborIndex) && GapFlags[NeighborIndex] != 0)
		{
			continue;
		}

		const int32 NeighborPlateId = InOutSamples[NeighborIndex].PlateId;
		if (Plates.IsValidIndex(NeighborPlateId))
		{
			AccumulateNeighborPlate(NeighborPlateId);
		}
	}

	auto FindNearestSamePlateSample = [this, &InOutSamples, &GapFlags, &Neighborhood, &Sample](const int32 PlateId, const bool bRequireBoundary, int32& OutSampleIndex, double& OutDistSq)
	{
		OutSampleIndex = INDEX_NONE;
		OutDistSq = TNumericLimits<double>::Max();
		if (!Plates.IsValidIndex(PlateId))
		{
			return;
		}

		for (const int32 CandidateIndex : Neighborhood)
		{
			if (!InOutSamples.IsValidIndex(CandidateIndex))
			{
				continue;
			}
			if (GapFlags.IsValidIndex(CandidateIndex) && GapFlags[CandidateIndex] != 0)
			{
				continue;
			}

			const FCanonicalSample& Candidate = InOutSamples[CandidateIndex];
			if (Candidate.PlateId != PlateId)
			{
				continue;
			}
			if (bRequireBoundary && !Candidate.bIsBoundary)
			{
				continue;
			}

			const double DistSq = FVector::DistSquared(Sample.Position, Candidate.Position);
			if (DistSq < OutDistSq)
			{
				OutDistSq = DistSq;
				OutSampleIndex = CandidateIndex;
			}
		}
	};

	auto FindDominantPlateId = [&NeighborPlateIds, &NeighborPlateCounts, NumNeighborPlates]() -> int32
	{
		int32 DominantPlateId = INDEX_NONE;
		int32 DominantPlateCount = 0;
		for (int32 SlotIndex = 0; SlotIndex < NumNeighborPlates; ++SlotIndex)
		{
			const int32 PlateId = NeighborPlateIds[SlotIndex];
			const int32 PlateCount = NeighborPlateCounts[SlotIndex];
			if (DominantPlateId == INDEX_NONE ||
				PlateCount > DominantPlateCount ||
				(PlateCount == DominantPlateCount && PlateId < DominantPlateId))
			{
				DominantPlateId = PlateId;
				DominantPlateCount = PlateCount;
			}
		}
		return DominantPlateId;
	};

	if (NumNeighborPlates <= 0)
	{
		CopyResolvedGapFromNeighbor(Plates.IsValidIndex(PreviousOwner) ? PreviousOwner : FindNearestPlateByCap(Position), INDEX_NONE);
		return true;
	}

	if (NumNeighborPlates == 1)
	{
		int32 NearestSampleIndex = INDEX_NONE;
		double NearestSampleDistSq = TNumericLimits<double>::Max();
		FindNearestSamePlateSample(NeighborPlateIds[0], false, NearestSampleIndex, NearestSampleDistSq);
		CopyResolvedGapFromNeighbor(NeighborPlateIds[0], NearestSampleIndex);
		return true;
	}

	if (NumNeighborPlates >= 3)
	{
		const int32 DominantPlateId = FindDominantPlateId();
		int32 NearestSampleIndex = INDEX_NONE;
		double NearestSampleDistSq = TNumericLimits<double>::Max();
		FindNearestSamePlateSample(DominantPlateId, false, NearestSampleIndex, NearestSampleDistSq);
		CopyResolvedGapFromNeighbor(DominantPlateId, NearestSampleIndex);
		return true;
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

	int32 FlankPlateA = (BestIndexA != INDEX_NONE) ? NeighborPlateIds[BestIndexA] : INDEX_NONE;
	int32 FlankPlateB = (BestIndexB != INDEX_NONE) ? NeighborPlateIds[BestIndexB] : INDEX_NONE;
	if (!Plates.IsValidIndex(FlankPlateA))
	{
		FlankPlateA = Plates.IsValidIndex(PreviousOwner) ? PreviousOwner : FindNearestPlateByCap(Position);
	}
	if (!Plates.IsValidIndex(FlankPlateB) || FlankPlateB == FlankPlateA)
	{
		FlankPlateB = INDEX_NONE;
	}

	int32 BorderSampleA = INDEX_NONE;
	int32 BorderSampleB = INDEX_NONE;
	double DistSqA = TNumericLimits<double>::Max();
	double DistSqB = TNumericLimits<double>::Max();
	FindNearestSamePlateSample(FlankPlateA, true, BorderSampleA, DistSqA);
	FindNearestSamePlateSample(FlankPlateB, true, BorderSampleB, DistSqB);
	if (!InOutSamples.IsValidIndex(BorderSampleA))
	{
		FindNearestSamePlateSample(FlankPlateA, false, BorderSampleA, DistSqA);
	}
	if (!InOutSamples.IsValidIndex(BorderSampleB))
	{
		FindNearestSamePlateSample(FlankPlateB, false, BorderSampleB, DistSqB);
	}

	int32 AssignedPlateId = INDEX_NONE;
	if (InOutSamples.IsValidIndex(BorderSampleA) && InOutSamples.IsValidIndex(BorderSampleB))
	{
		AssignedPlateId = (DistSqA <= DistSqB) ? FlankPlateA : FlankPlateB;
	}
	else if (InOutSamples.IsValidIndex(BorderSampleA))
	{
		AssignedPlateId = FlankPlateA;
	}
	else if (InOutSamples.IsValidIndex(BorderSampleB))
	{
		AssignedPlateId = FlankPlateB;
	}
	else if (Plates.IsValidIndex(PreviousOwner))
	{
		AssignedPlateId = PreviousOwner;
	}
	else
	{
		AssignedPlateId = FindNearestPlateByCap(Position);
	}

	const int32 NearestResolvedFlankSample =
		(InOutSamples.IsValidIndex(BorderSampleA) && InOutSamples.IsValidIndex(BorderSampleB))
			? ((DistSqA <= DistSqB) ? BorderSampleA : BorderSampleB)
			: (InOutSamples.IsValidIndex(BorderSampleA) ? BorderSampleA : BorderSampleB);
	const bool bHasValidFlankPair =
		Plates.IsValidIndex(FlankPlateA) &&
		Plates.IsValidIndex(FlankPlateB) &&
		FlankPlateA != FlankPlateB &&
		InOutSamples.IsValidIndex(BorderSampleA) &&
		InOutSamples.IsValidIndex(BorderSampleB);

	FVector RidgeDirection = FVector::ZeroVector;
	bool bDivergentGap = false;
	if (bHasValidFlankPair)
	{
		const FVector RelativeVelocity = ComputeSurfaceVelocity(FlankPlateA, Position) - ComputeSurfaceVelocity(FlankPlateB, Position);
		const double RelativeSpeed = RelativeVelocity.Size();
		FVector AcrossGap = InOutSamples[BorderSampleB].Position - InOutSamples[BorderSampleA].Position;
		AcrossGap = AcrossGap - FVector::DotProduct(AcrossGap, Position) * Position;
		AcrossGap = AcrossGap.GetSafeNormal();
		if (!AcrossGap.IsNearlyZero() && RelativeSpeed > 1e-8)
		{
			const double NormalComponent = FVector::DotProduct(RelativeVelocity, AcrossGap);
			bDivergentGap = NormalComponent < -(DivergentGapThresholdRatio * RelativeSpeed);
			RidgeDirection = FVector::CrossProduct(RelativeVelocity, Position);
		}
	}

	if (!bHasValidFlankPair || !bDivergentGap)
	{
		CopyResolvedGapFromNeighbor(AssignedPlateId, NearestResolvedFlankSample);
		return true;
	}

	float BorderElevation = OceanicFallbackElevationKm;
	if (InOutSamples.IsValidIndex(BorderSampleA) && InOutSamples.IsValidIndex(BorderSampleB))
	{
		const float ElevationA = InOutSamples[BorderSampleA].Elevation;
		const float ElevationB = InOutSamples[BorderSampleB].Elevation;
		const double DistA = FMath::Sqrt(FMath::Max(0.0, DistSqA));
		const double DistB = FMath::Sqrt(FMath::Max(0.0, DistSqB));
		const double WeightA = 1.0 / FMath::Max(1e-6, DistA);
		const double WeightB = 1.0 / FMath::Max(1e-6, DistB);
		const double WeightSum = WeightA + WeightB;
		BorderElevation = (WeightSum > UE_DOUBLE_SMALL_NUMBER)
			? static_cast<float>((WeightA * ElevationA + WeightB * ElevationB) / WeightSum)
			: 0.5f * (ElevationA + ElevationB);
	}
	else if (InOutSamples.IsValidIndex(BorderSampleA))
	{
		BorderElevation = InOutSamples[BorderSampleA].Elevation;
	}
	else if (InOutSamples.IsValidIndex(BorderSampleB))
	{
		BorderElevation = InOutSamples[BorderSampleB].Elevation;
	}

	float RidgeBlendedElevation = BorderElevation;
	if (InOutSamples.IsValidIndex(BorderSampleA) && InOutSamples.IsValidIndex(BorderSampleB))
	{
		const double DistA = FMath::Sqrt(FMath::Max(0.0, DistSqA));
		const double DistB = FMath::Sqrt(FMath::Max(0.0, DistSqB));
		const double DistanceToRidge = 0.5 * FMath::Abs(DistA - DistB);
		const double DistanceToBorder = FMath::Min(DistA, DistB);
		const double AlphaDenominator = DistanceToRidge + DistanceToBorder;
		const double Alpha = (AlphaDenominator > UE_DOUBLE_SMALL_NUMBER)
			? FMath::Clamp(DistanceToRidge / AlphaDenominator, 0.0, 1.0)
			: 0.0;
		RidgeBlendedElevation = static_cast<float>(Alpha * BorderElevation + (1.0 - Alpha) * static_cast<double>(RidgeTemplateElevationKm));
	}

	if (RidgeDirection.IsNearlyZero() && InOutSamples.IsValidIndex(BorderSampleA) && InOutSamples.IsValidIndex(BorderSampleB))
	{
		FVector AcrossGap = InOutSamples[BorderSampleB].Position - InOutSamples[BorderSampleA].Position;
		AcrossGap = AcrossGap - FVector::DotProduct(AcrossGap, Position) * Position;
		AcrossGap = AcrossGap.GetSafeNormal();
		RidgeDirection = FVector::CrossProduct(AcrossGap, Position);
	}
	RidgeDirection = RidgeDirection - FVector::DotProduct(RidgeDirection, Position) * Position;
	RidgeDirection = RidgeDirection.GetSafeNormal();

	Sample.PlateId = AssignedPlateId;
	Sample.CrustType = ECrustType::Oceanic;
	Sample.Elevation = RidgeBlendedElevation;
	Sample.Thickness = OceanicThicknessKm;
	Sample.Age = 0.0f;
	Sample.RidgeDirection = RidgeDirection;
	Sample.FoldDirection = FVector::ZeroVector;
	Sample.OrogenyType = EOrogenyType::None;
	Sample.OrogenyAge = 0.0f;
	Sample.TerraneId = INDEX_NONE;
	Sample.CollisionDistanceKm = -1.0f;
	Sample.CollisionConvergenceSpeedMmPerYear = 0.0f;
	Sample.bIsCollisionFront = false;
	Sample.bGapDetected = true;
	Sample.FlankingPlateIdA = FlankPlateA;
	Sample.FlankingPlateIdB = FlankPlateB;
	bOutDivergentGapCreated = true;
	return true;
}

namespace
{
	struct FContinentalStabilizerComponent
	{
		TArray<int32> Members;
		int32 PreviousContinentalCount = 0;
		int32 MinSampleIndex = TNumericLimits<int32>::Max();
	};

	struct FContinentalStabilizerPromotionCandidate
	{
		int32 SampleIndex = INDEX_NONE;
		int32 CurrentContinentalNeighborCount = 0;
		int32 PreviousContinentalNeighborCount = 0;
	};

	struct FContinentalStabilizerTrimCandidate
	{
		int32 SampleIndex = INDEX_NONE;
		bool bWasPreviouslyOceanic = false;
		int32 CurrentContinentalNeighborCount = 0;
		int32 PreviousContinentalNeighborCount = 0;
	};

	struct FContinentalStabilizerWorkingState
	{
		TArray<uint8> PreviousContinentalFlags;
		TArray<uint8> CurrentContinentalFlags;
		TArray<int32> PreviousContinentalNeighborCounts;
		TArray<int32> CurrentContinentalNeighborCounts;
		TArray<int32> Generations;
		int32 PreviousContinentalCount = 0;
		int32 CurrentContinentalCount = 0;
	};

	struct FContinentalStabilizerPromotionHeapEntry
	{
		int32 SampleIndex = INDEX_NONE;
		int32 Generation = 0;
		int32 CurrentContinentalNeighborCount = 0;
		int32 PreviousContinentalNeighborCount = 0;
	};

	struct FContinentalStabilizerTrimHeapEntry
	{
		int32 SampleIndex = INDEX_NONE;
		int32 Generation = 0;
		bool bWasPreviouslyOceanic = false;
		int32 CurrentContinentalNeighborCount = 0;
		int32 PreviousContinentalNeighborCount = 0;
	};

	struct FContinentalStabilizerSharedContext
	{
		const TArray<TArray<int32>>& Adjacency;
		const TArray<FCanonicalSample>& PreviousSamples;
		TArray<FCanonicalSample>& Samples;
		const float AbyssalPlainElevationKm;
		const float OceanicCrustThicknessKm;

		void CopyOceanicTemplateToSample(const FCanonicalSample& TemplateSample, const int32 SampleIndex) const
		{
			if (!Samples.IsValidIndex(SampleIndex))
			{
				return;
			}

			FCanonicalSample& Sample = Samples[SampleIndex];
			Sample.CrustType = ECrustType::Oceanic;
			Sample.Elevation = TemplateSample.Elevation;
			Sample.Thickness = TemplateSample.Thickness;
			Sample.Age = TemplateSample.Age;
			Sample.RidgeDirection = TemplateSample.RidgeDirection;
			Sample.FoldDirection = TemplateSample.FoldDirection;
			Sample.OrogenyType = EOrogenyType::None;
			Sample.OrogenyAge = 0.0f;
			Sample.TerraneId = INDEX_NONE;
			Sample.CollisionDistanceKm = -1.0f;
			Sample.CollisionConvergenceSpeedMmPerYear = 0.0f;
			Sample.bIsCollisionFront = false;
		}

		void ResetSampleToOceanicDefaults(const int32 SampleIndex) const
		{
			if (!Samples.IsValidIndex(SampleIndex))
			{
				return;
			}

			FCanonicalSample& Sample = Samples[SampleIndex];
			Sample.CrustType = ECrustType::Oceanic;
			Sample.Elevation = AbyssalPlainElevationKm;
			Sample.Thickness = OceanicCrustThicknessKm;
			Sample.Age = 0.0f;
			Sample.RidgeDirection = FVector::ZeroVector;
			Sample.FoldDirection = FVector::ZeroVector;
			Sample.OrogenyType = EOrogenyType::None;
			Sample.OrogenyAge = 0.0f;
			Sample.TerraneId = INDEX_NONE;
			Sample.CollisionDistanceKm = -1.0f;
			Sample.CollisionConvergenceSpeedMmPerYear = 0.0f;
			Sample.bIsCollisionFront = false;
		}

		void PromoteSampleFromPrevious(const int32 SampleIndex) const
		{
			if (!PreviousSamples.IsValidIndex(SampleIndex) || !Samples.IsValidIndex(SampleIndex))
			{
				return;
			}

			const FCanonicalSample& PreviousSample = PreviousSamples[SampleIndex];
			if (PreviousSample.CrustType != ECrustType::Continental)
			{
				return;
			}

			FCanonicalSample& Sample = Samples[SampleIndex];
			Sample.CrustType = ECrustType::Continental;
			Sample.Elevation = PreviousSample.Elevation;
			Sample.Thickness = PreviousSample.Thickness;
			Sample.Age = PreviousSample.Age;
			Sample.RidgeDirection = PreviousSample.RidgeDirection;
			Sample.FoldDirection = PreviousSample.FoldDirection;
			Sample.OrogenyType = PreviousSample.OrogenyType;
			Sample.OrogenyAge = PreviousSample.OrogenyAge;
			Sample.TerraneId = PreviousSample.TerraneId;
			Sample.CollisionDistanceKm = -1.0f;
			Sample.CollisionConvergenceSpeedMmPerYear = 0.0f;
			Sample.bIsCollisionFront = false;
		}

		void DemoteSampleToOceanic(const int32 SampleIndex) const
		{
			if (!Samples.IsValidIndex(SampleIndex))
			{
				return;
			}

			if (PreviousSamples.IsValidIndex(SampleIndex) && PreviousSamples[SampleIndex].CrustType == ECrustType::Oceanic)
			{
				CopyOceanicTemplateToSample(PreviousSamples[SampleIndex], SampleIndex);
				return;
			}

			int32 BestNeighborIndex = INDEX_NONE;
			double BestDistSq = TNumericLimits<double>::Max();
			auto ConsiderOceanicCandidate = [this, SampleIndex, &BestNeighborIndex, &BestDistSq](const int32 CandidateIndex)
			{
				if (!Samples.IsValidIndex(CandidateIndex))
				{
					return;
				}

				const FCanonicalSample& Candidate = Samples[CandidateIndex];
				if (Candidate.CrustType != ECrustType::Oceanic || Candidate.bGapDetected)
				{
					return;
				}

				const double CandidateDistSq = FVector::DistSquared(Samples[SampleIndex].Position, Candidate.Position);
				if (BestNeighborIndex == INDEX_NONE ||
					CandidateDistSq < BestDistSq - UE_DOUBLE_SMALL_NUMBER ||
					(FMath::IsNearlyEqual(CandidateDistSq, BestDistSq, UE_DOUBLE_SMALL_NUMBER) && CandidateIndex < BestNeighborIndex))
				{
					BestNeighborIndex = CandidateIndex;
					BestDistSq = CandidateDistSq;
				}
			};

			if (Adjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					ConsiderOceanicCandidate(NeighborIndex);
				}
			}

			if (BestNeighborIndex == INDEX_NONE && Adjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					if (!Adjacency.IsValidIndex(NeighborIndex))
					{
						continue;
					}

					for (const int32 SecondRingIndex : Adjacency[NeighborIndex])
					{
						if (SecondRingIndex != SampleIndex)
						{
							ConsiderOceanicCandidate(SecondRingIndex);
						}
					}
				}
			}

			if (BestNeighborIndex != INDEX_NONE)
			{
				CopyOceanicTemplateToSample(Samples[BestNeighborIndex], SampleIndex);
				return;
			}

			ResetSampleToOceanicDefaults(SampleIndex);
		}

		int32 CountCurrentContinentalNeighbors(const TArray<uint8>& CurrentContinentalFlags, const int32 SampleIndex) const
		{
			if (!Adjacency.IsValidIndex(SampleIndex))
			{
				return 0;
			}

			int32 NeighborCount = 0;
			for (const int32 NeighborIndex : Adjacency[SampleIndex])
			{
				if (Samples.IsValidIndex(NeighborIndex) &&
					CurrentContinentalFlags.IsValidIndex(NeighborIndex) &&
					CurrentContinentalFlags[NeighborIndex] != 0 &&
					!Samples[NeighborIndex].bGapDetected)
				{
					++NeighborCount;
				}
			}
			return NeighborCount;
		}

		int32 CountPreviousContinentalNeighbors(const TArray<uint8>& PreviousContinentalFlags, const int32 SampleIndex) const
		{
			if (!Adjacency.IsValidIndex(SampleIndex))
			{
				return 0;
			}

			int32 NeighborCount = 0;
			for (const int32 NeighborIndex : Adjacency[SampleIndex])
			{
				if (PreviousContinentalFlags.IsValidIndex(NeighborIndex) && PreviousContinentalFlags[NeighborIndex] != 0)
				{
					++NeighborCount;
				}
			}
			return NeighborCount;
		}

		void BuildCurrentContinentalNeighborRefreshSet(const int32 SampleIndex, TArray<int32>& OutAffectedSampleIndices) const
		{
			OutAffectedSampleIndices.Reset();
			if (!Samples.IsValidIndex(SampleIndex))
			{
				return;
			}

			OutAffectedSampleIndices.Reserve(Adjacency.IsValidIndex(SampleIndex) ? (Adjacency[SampleIndex].Num() + 1) : 1);
			OutAffectedSampleIndices.Add(SampleIndex);
			if (Adjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					if (Samples.IsValidIndex(NeighborIndex))
					{
						OutAffectedSampleIndices.AddUnique(NeighborIndex);
					}
				}
			}
		}

		void RefreshCurrentContinentalNeighborCountsForAffectedSamples(
			const TArray<uint8>& CurrentContinentalFlags,
			TArray<int32>& InOutCurrentContinentalNeighborCounts,
			const TArray<int32>& AffectedSampleIndices) const
		{
			if (InOutCurrentContinentalNeighborCounts.Num() != Samples.Num())
			{
				return;
			}

			for (const int32 AffectedSampleIndex : AffectedSampleIndices)
			{
				if (Samples.IsValidIndex(AffectedSampleIndex))
				{
					InOutCurrentContinentalNeighborCounts[AffectedSampleIndex] = CountCurrentContinentalNeighbors(CurrentContinentalFlags, AffectedSampleIndex);
				}
			}
		}

		void RefreshCurrentContinentalNeighborCounts(
			const TArray<uint8>& CurrentContinentalFlags,
			TArray<int32>& InOutCurrentContinentalNeighborCounts,
			const int32 SampleIndex) const
		{
			if (!Samples.IsValidIndex(SampleIndex) || InOutCurrentContinentalNeighborCounts.Num() != Samples.Num())
			{
				return;
			}

			TArray<int32> AffectedSampleIndices;
			BuildCurrentContinentalNeighborRefreshSet(SampleIndex, AffectedSampleIndices);
			RefreshCurrentContinentalNeighborCountsForAffectedSamples(CurrentContinentalFlags, InOutCurrentContinentalNeighborCounts, AffectedSampleIndices);
		}

		void BuildContinentalComponents(
			const TArray<uint8>& CurrentContinentalFlags,
			const TArray<uint8>& PreviousContinentalFlags,
			TArray<FContinentalStabilizerComponent>& OutComponents) const
		{
			const int32 NumSamples = Samples.Num();
			OutComponents.Reset();
			if (NumSamples <= 0)
			{
				return;
			}

			TArray<uint8> Visited;
			Visited.Init(0, NumSamples);
			TArray<int32> Stack;
			Stack.Reserve(256);

			for (int32 SeedIndex = 0; SeedIndex < NumSamples; ++SeedIndex)
			{
				if (Visited[SeedIndex] != 0 ||
					!CurrentContinentalFlags.IsValidIndex(SeedIndex) ||
					CurrentContinentalFlags[SeedIndex] == 0 ||
					Samples[SeedIndex].bGapDetected)
				{
					continue;
				}

				FContinentalStabilizerComponent& Component = OutComponents.Emplace_GetRef();
				Visited[SeedIndex] = 1;
				Stack.Reset();
				Stack.Add(SeedIndex);
				while (Stack.Num() > 0)
				{
					const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
					Component.Members.Add(SampleIndex);
					Component.MinSampleIndex = FMath::Min(Component.MinSampleIndex, SampleIndex);
					Component.PreviousContinentalCount +=
						(PreviousContinentalFlags.IsValidIndex(SampleIndex) && PreviousContinentalFlags[SampleIndex] != 0) ? 1 : 0;

					if (!Adjacency.IsValidIndex(SampleIndex))
					{
						continue;
					}

					for (const int32 NeighborIndex : Adjacency[SampleIndex])
					{
						if (!Samples.IsValidIndex(NeighborIndex) ||
							Visited[NeighborIndex] != 0 ||
							!CurrentContinentalFlags.IsValidIndex(NeighborIndex) ||
							CurrentContinentalFlags[NeighborIndex] == 0 ||
							Samples[NeighborIndex].bGapDetected)
						{
							continue;
						}

						Visited[NeighborIndex] = 1;
						Stack.Add(NeighborIndex);
					}
				}
			}
		}
	};

	template <typename TEntry, typename THigherPriorityPredicate>
	void HeapSiftUp(TArray<TEntry>& Heap, int32 HeapIndex, const THigherPriorityPredicate& IsHigherPriority)
	{
		while (HeapIndex > 0)
		{
			const int32 ParentIndex = (HeapIndex - 1) / 2;
			if (!IsHigherPriority(Heap[HeapIndex], Heap[ParentIndex]))
			{
				break;
			}
			Swap(Heap[HeapIndex], Heap[ParentIndex]);
			HeapIndex = ParentIndex;
		}
	}
	template <typename TEntry, typename THigherPriorityPredicate>
	void HeapSiftDown(TArray<TEntry>& Heap, int32 HeapIndex, const THigherPriorityPredicate& IsHigherPriority)
	{
		for (;;)
		{
			const int32 LeftChildIndex = (HeapIndex * 2) + 1;
			if (LeftChildIndex >= Heap.Num())
			{
				break;
			}
			int32 BestChildIndex = LeftChildIndex;
			const int32 RightChildIndex = LeftChildIndex + 1;
			if (RightChildIndex < Heap.Num() && IsHigherPriority(Heap[RightChildIndex], Heap[LeftChildIndex]))
			{
				BestChildIndex = RightChildIndex;
			}
			if (!IsHigherPriority(Heap[BestChildIndex], Heap[HeapIndex]))
			{
				break;
			}
			Swap(Heap[HeapIndex], Heap[BestChildIndex]);
			HeapIndex = BestChildIndex;
		}
	}
	template <typename TEntry, typename THigherPriorityPredicate>
	void HeapPushEntry(TArray<TEntry>& Heap, TEntry Entry, const THigherPriorityPredicate& IsHigherPriority)
	{
		Heap.Add(MoveTemp(Entry));
		HeapSiftUp(Heap, Heap.Num() - 1, IsHigherPriority);
	}
	template <typename TEntry, typename THigherPriorityPredicate>
	bool HeapTryPopEntry(TArray<TEntry>& Heap, TEntry& OutEntry, const THigherPriorityPredicate& IsHigherPriority)
	{
		if (Heap.Num() <= 0)
		{
			return false;
		}
		OutEntry = Heap[0];
		const int32 LastIndex = Heap.Num() - 1;
		if (LastIndex == 0)
		{
			Heap.Pop(EAllowShrinking::No);
			return true;
		}
		Heap[0] = MoveTemp(Heap[LastIndex]);
		Heap.Pop(EAllowShrinking::No);
		HeapSiftDown(Heap, 0, IsHigherPriority);
		return true;
	}
	template <typename TEntry>
	const TEntry* HeapPeekEntry(const TArray<TEntry>& Heap)
	{
		return Heap.Num() > 0 ? &Heap[0] : nullptr;
	}

	void ResetStabilizerSubphaseTimings(FReconcilePhaseTimings* InOutTimings)
	{
		if (!InOutTimings)
		{
			return;
		}

		InOutTimings->StabilizerPromoteToMinimumMs = 0.0;
		InOutTimings->StabilizerBuildComponentsMs = 0.0;
		InOutTimings->StabilizerComponentPruneMs = 0.0;
		InOutTimings->StabilizerPromoteToTargetMs = 0.0;
		InOutTimings->StabilizerTrimMs = 0.0;
		InOutTimings->StabilizerPromotionCount = 0;
		InOutTimings->StabilizerComponentDemotionCount = 0;
		InOutTimings->StabilizerTrimDemotionCount = 0;
		InOutTimings->StabilizerHeapPushCount = 0;
		InOutTimings->StabilizerHeapStalePopCount = 0;
	}

	bool InitializeContinentalStabilizerWorkingState(
		const FContinentalStabilizerSharedContext& SharedContext,
		const bool bBuildCurrentNeighborCounts,
		FContinentalStabilizerWorkingState& OutState)
	{
		const int32 NumSamples = SharedContext.Samples.Num();
		if (NumSamples <= 0 || SharedContext.PreviousSamples.Num() != NumSamples || SharedContext.Adjacency.Num() != NumSamples)
		{
			return false;
		}

		OutState = FContinentalStabilizerWorkingState{};
		OutState.PreviousContinentalFlags.Init(0, NumSamples);
		OutState.CurrentContinentalFlags.Init(0, NumSamples);
		OutState.PreviousContinentalNeighborCounts.Init(0, NumSamples);
		if (bBuildCurrentNeighborCounts)
		{
			OutState.CurrentContinentalNeighborCounts.Init(0, NumSamples);
		}
		OutState.Generations.Init(0, NumSamples);

		for (int32 SampleIndex = 0; SampleIndex < NumSamples; ++SampleIndex)
		{
			const bool bPreviousContinental = SharedContext.PreviousSamples[SampleIndex].CrustType == ECrustType::Continental;
			const bool bCurrentContinental = SharedContext.Samples[SampleIndex].CrustType == ECrustType::Continental;
			OutState.PreviousContinentalFlags[SampleIndex] = bPreviousContinental ? 1 : 0;
			OutState.CurrentContinentalFlags[SampleIndex] = bCurrentContinental ? 1 : 0;
			OutState.PreviousContinentalCount += bPreviousContinental ? 1 : 0;
			OutState.CurrentContinentalCount += bCurrentContinental ? 1 : 0;
		}

		for (int32 SampleIndex = 0; SampleIndex < NumSamples; ++SampleIndex)
		{
			OutState.PreviousContinentalNeighborCounts[SampleIndex] =
				SharedContext.CountPreviousContinentalNeighbors(OutState.PreviousContinentalFlags, SampleIndex);
			if (bBuildCurrentNeighborCounts)
			{
				OutState.CurrentContinentalNeighborCounts[SampleIndex] =
					SharedContext.CountCurrentContinentalNeighbors(OutState.CurrentContinentalFlags, SampleIndex);
			}
		}

		return true;
	}

	void RunScanBasedContinentalStabilizer(
		const FContinentalStabilizerSharedContext& SharedContext,
		const int32 InitialContinentalPlateCount,
		const double MinContinentalAreaFraction,
		const bool bUseCachedCurrentNeighborCounts,
		FReconcilePhaseTimings* InOutTimings)
	{
		ResetStabilizerSubphaseTimings(InOutTimings);

		const int32 NumSamples = SharedContext.Samples.Num();
		if (NumSamples <= 0 || SharedContext.PreviousSamples.Num() != NumSamples || SharedContext.Adjacency.Num() != NumSamples)
		{
			return;
		}

		FContinentalStabilizerWorkingState WorkingState;
		if (!InitializeContinentalStabilizerWorkingState(SharedContext, bUseCachedCurrentNeighborCounts, WorkingState))
		{
			return;
		}

		const double ClampedMinimumContinentalFraction = FMath::Clamp(MinContinentalAreaFraction, 0.0, 1.0);
		const int32 MinimumContinentalSamples = FMath::Clamp(
			FMath::CeilToInt(ClampedMinimumContinentalFraction * static_cast<double>(NumSamples)),
			0,
			NumSamples);
		const int32 MaximumContinentalComponents =
			(InitialContinentalPlateCount > 0)
				? (3 * InitialContinentalPlateCount)
				: TNumericLimits<int32>::Max();

		auto GetCurrentContinentalNeighborCount = [&SharedContext, &WorkingState, bUseCachedCurrentNeighborCounts](const int32 SampleIndex) -> int32
		{
			if (bUseCachedCurrentNeighborCounts)
			{
				return WorkingState.CurrentContinentalNeighborCounts.IsValidIndex(SampleIndex)
					? WorkingState.CurrentContinentalNeighborCounts[SampleIndex]
					: 0;
			}
			return SharedContext.CountCurrentContinentalNeighbors(WorkingState.CurrentContinentalFlags, SampleIndex);
		};

		auto GetPreviousContinentalNeighborCount = [&WorkingState](const int32 SampleIndex) -> int32
		{
			return WorkingState.PreviousContinentalNeighborCounts.IsValidIndex(SampleIndex)
				? WorkingState.PreviousContinentalNeighborCounts[SampleIndex]
				: 0;
		};

		auto ApplyPromotion = [&SharedContext, &WorkingState, bUseCachedCurrentNeighborCounts, InOutTimings](const int32 SampleIndex) -> bool
		{
			if (!WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) || WorkingState.CurrentContinentalFlags[SampleIndex] != 0)
			{
				return false;
			}

			SharedContext.PromoteSampleFromPrevious(SampleIndex);
			if (!SharedContext.Samples.IsValidIndex(SampleIndex) || SharedContext.Samples[SampleIndex].CrustType != ECrustType::Continental)
			{
				return false;
			}

			WorkingState.CurrentContinentalFlags[SampleIndex] = 1;
			++WorkingState.CurrentContinentalCount;
			if (bUseCachedCurrentNeighborCounts)
			{
				SharedContext.RefreshCurrentContinentalNeighborCounts(
					WorkingState.CurrentContinentalFlags,
					WorkingState.CurrentContinentalNeighborCounts,
					SampleIndex);
			}
			if (InOutTimings)
			{
				++InOutTimings->StabilizerPromotionCount;
			}
			return true;
		};

		auto ApplyDemotion = [&SharedContext, &WorkingState, bUseCachedCurrentNeighborCounts, InOutTimings](const int32 SampleIndex, const bool bComponentDemotion) -> bool
		{
			if (!WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) || WorkingState.CurrentContinentalFlags[SampleIndex] == 0)
			{
				return false;
			}

			SharedContext.DemoteSampleToOceanic(SampleIndex);
			if (!SharedContext.Samples.IsValidIndex(SampleIndex) || SharedContext.Samples[SampleIndex].CrustType != ECrustType::Oceanic)
			{
				return false;
			}

			WorkingState.CurrentContinentalFlags[SampleIndex] = 0;
			--WorkingState.CurrentContinentalCount;
			if (bUseCachedCurrentNeighborCounts)
			{
				SharedContext.RefreshCurrentContinentalNeighborCounts(
					WorkingState.CurrentContinentalFlags,
					WorkingState.CurrentContinentalNeighborCounts,
					SampleIndex);
			}
			if (InOutTimings)
			{
				if (bComponentDemotion)
				{
					++InOutTimings->StabilizerComponentDemotionCount;
				}
				else
				{
					++InOutTimings->StabilizerTrimDemotionCount;
				}
			}
			return true;
		};

		auto FindBestPromotionCandidate = [&SharedContext, &WorkingState, &GetCurrentContinentalNeighborCount, &GetPreviousContinentalNeighborCount](const bool bRequireCurrentAdjacency)
		{
			FContinentalStabilizerPromotionCandidate BestCandidate;
			for (int32 SampleIndex = 0; SampleIndex < SharedContext.Samples.Num(); ++SampleIndex)
			{
				if ((WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) && WorkingState.CurrentContinentalFlags[SampleIndex] != 0) ||
					SharedContext.Samples[SampleIndex].bGapDetected ||
					!WorkingState.PreviousContinentalFlags.IsValidIndex(SampleIndex) ||
					WorkingState.PreviousContinentalFlags[SampleIndex] == 0)
				{
					continue;
				}

				FContinentalStabilizerPromotionCandidate Candidate;
				Candidate.SampleIndex = SampleIndex;
				Candidate.CurrentContinentalNeighborCount = GetCurrentContinentalNeighborCount(SampleIndex);
				Candidate.PreviousContinentalNeighborCount = GetPreviousContinentalNeighborCount(SampleIndex);
				if (bRequireCurrentAdjacency && Candidate.CurrentContinentalNeighborCount <= 0)
				{
					continue;
				}

				const bool bBetterCandidate =
					BestCandidate.SampleIndex == INDEX_NONE ||
					Candidate.CurrentContinentalNeighborCount > BestCandidate.CurrentContinentalNeighborCount ||
					(Candidate.CurrentContinentalNeighborCount == BestCandidate.CurrentContinentalNeighborCount &&
						Candidate.PreviousContinentalNeighborCount > BestCandidate.PreviousContinentalNeighborCount) ||
					(Candidate.CurrentContinentalNeighborCount == BestCandidate.CurrentContinentalNeighborCount &&
						Candidate.PreviousContinentalNeighborCount == BestCandidate.PreviousContinentalNeighborCount &&
						Candidate.SampleIndex < BestCandidate.SampleIndex);
				if (bBetterCandidate)
				{
					BestCandidate = Candidate;
				}
			}

			return BestCandidate;
		};

		auto TryPromoteContinentalFrontier = [&FindBestPromotionCandidate, &ApplyPromotion](const bool bRequireCurrentAdjacency) -> bool
		{
			const FContinentalStabilizerPromotionCandidate Candidate = FindBestPromotionCandidate(bRequireCurrentAdjacency);
			if (Candidate.SampleIndex == INDEX_NONE)
			{
				return false;
			}

			return ApplyPromotion(Candidate.SampleIndex);
		};

		const double PromoteToMinimumStart = FPlatformTime::Seconds();
		if (WorkingState.CurrentContinentalCount < MinimumContinentalSamples)
		{
			while (WorkingState.CurrentContinentalCount < MinimumContinentalSamples)
			{
				const bool bHaveCurrentContinents = WorkingState.CurrentContinentalCount > 0;
				if (TryPromoteContinentalFrontier(bHaveCurrentContinents))
				{
					continue;
				}

				if (!bHaveCurrentContinents || !TryPromoteContinentalFrontier(false))
				{
					break;
				}
			}
		}
		if (InOutTimings)
		{
			InOutTimings->StabilizerPromoteToMinimumMs = (FPlatformTime::Seconds() - PromoteToMinimumStart) * 1000.0;
		}

		const int32 TargetContinentalCount = FMath::Clamp(
			FMath::Min(WorkingState.CurrentContinentalCount, WorkingState.PreviousContinentalCount),
			0,
			NumSamples);

		const double BuildComponentsStart = FPlatformTime::Seconds();
		TArray<FContinentalStabilizerComponent> Components;
		SharedContext.BuildContinentalComponents(
			WorkingState.CurrentContinentalFlags,
			WorkingState.PreviousContinentalFlags,
			Components);
		if (InOutTimings)
		{
			InOutTimings->StabilizerBuildComponentsMs = (FPlatformTime::Seconds() - BuildComponentsStart) * 1000.0;
		}
		if (Components.Num() <= 0)
		{
			return;
		}

		Components.Sort([](const FContinentalStabilizerComponent& A, const FContinentalStabilizerComponent& B)
		{
			if (A.PreviousContinentalCount != B.PreviousContinentalCount)
			{
				return A.PreviousContinentalCount < B.PreviousContinentalCount;
			}
			if (A.Members.Num() != B.Members.Num())
			{
				return A.Members.Num() < B.Members.Num();
			}
			return A.MinSampleIndex < B.MinSampleIndex;
		});

		const double ComponentPruneStart = FPlatformTime::Seconds();
		int32 RemainingComponentCount = Components.Num();
		for (const FContinentalStabilizerComponent& Component : Components)
		{
			const bool bExceedsComponentCap =
				MaximumContinentalComponents != TNumericLimits<int32>::Max() &&
				RemainingComponentCount > MaximumContinentalComponents;
			const bool bExceedsTargetCount = WorkingState.CurrentContinentalCount > TargetContinentalCount;
			if (!bExceedsComponentCap && !bExceedsTargetCount)
			{
				break;
			}

			for (const int32 SampleIndex : Component.Members)
			{
				ApplyDemotion(SampleIndex, true);
			}
			--RemainingComponentCount;
		}
		if (InOutTimings)
		{
			InOutTimings->StabilizerComponentPruneMs = (FPlatformTime::Seconds() - ComponentPruneStart) * 1000.0;
		}

		const double PromoteToTargetStart = FPlatformTime::Seconds();
		while (WorkingState.CurrentContinentalCount < TargetContinentalCount)
		{
			const bool bRequireCurrentAdjacency = WorkingState.CurrentContinentalCount > 0;
			if (TryPromoteContinentalFrontier(bRequireCurrentAdjacency))
			{
				continue;
			}

			if (!bRequireCurrentAdjacency || !TryPromoteContinentalFrontier(false))
			{
				break;
			}
		}
		if (InOutTimings)
		{
			InOutTimings->StabilizerPromoteToTargetMs = (FPlatformTime::Seconds() - PromoteToTargetStart) * 1000.0;
		}

		auto FindBestTrimCandidate = [&SharedContext, &WorkingState, &GetCurrentContinentalNeighborCount, &GetPreviousContinentalNeighborCount](const int32 MaxNeighborCount)
		{
			FContinentalStabilizerTrimCandidate BestCandidate;
			for (int32 SampleIndex = 0; SampleIndex < SharedContext.Samples.Num(); ++SampleIndex)
			{
				if (!WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) ||
					WorkingState.CurrentContinentalFlags[SampleIndex] == 0 ||
					SharedContext.Samples[SampleIndex].bGapDetected)
				{
					continue;
				}

				FContinentalStabilizerTrimCandidate Candidate;
				Candidate.SampleIndex = SampleIndex;
				Candidate.bWasPreviouslyOceanic =
					!WorkingState.PreviousContinentalFlags.IsValidIndex(SampleIndex) ||
					WorkingState.PreviousContinentalFlags[SampleIndex] == 0;
				Candidate.CurrentContinentalNeighborCount = GetCurrentContinentalNeighborCount(SampleIndex);
				Candidate.PreviousContinentalNeighborCount = GetPreviousContinentalNeighborCount(SampleIndex);
				if (Candidate.CurrentContinentalNeighborCount > MaxNeighborCount)
				{
					continue;
				}

				const bool bBetterCandidate =
					BestCandidate.SampleIndex == INDEX_NONE ||
					(Candidate.bWasPreviouslyOceanic && !BestCandidate.bWasPreviouslyOceanic) ||
					(Candidate.bWasPreviouslyOceanic == BestCandidate.bWasPreviouslyOceanic &&
						Candidate.CurrentContinentalNeighborCount < BestCandidate.CurrentContinentalNeighborCount) ||
					(Candidate.bWasPreviouslyOceanic == BestCandidate.bWasPreviouslyOceanic &&
						Candidate.CurrentContinentalNeighborCount == BestCandidate.CurrentContinentalNeighborCount &&
						Candidate.PreviousContinentalNeighborCount < BestCandidate.PreviousContinentalNeighborCount) ||
					(Candidate.bWasPreviouslyOceanic == BestCandidate.bWasPreviouslyOceanic &&
						Candidate.CurrentContinentalNeighborCount == BestCandidate.CurrentContinentalNeighborCount &&
						Candidate.PreviousContinentalNeighborCount == BestCandidate.PreviousContinentalNeighborCount &&
						Candidate.SampleIndex < BestCandidate.SampleIndex);
				if (bBetterCandidate)
				{
					BestCandidate = Candidate;
				}
			}

			return BestCandidate;
		};

		const double TrimStart = FPlatformTime::Seconds();
		for (int32 MaxNeighborCount = 1; WorkingState.CurrentContinentalCount > TargetContinentalCount && MaxNeighborCount <= 3; ++MaxNeighborCount)
		{
			while (WorkingState.CurrentContinentalCount > TargetContinentalCount)
			{
				const FContinentalStabilizerTrimCandidate TrimCandidate = FindBestTrimCandidate(MaxNeighborCount);
				if (TrimCandidate.SampleIndex == INDEX_NONE)
				{
					break;
				}

				ApplyDemotion(TrimCandidate.SampleIndex, false);
			}
		}

		while (WorkingState.CurrentContinentalCount > TargetContinentalCount)
		{
			const FContinentalStabilizerTrimCandidate TrimCandidate = FindBestTrimCandidate(TNumericLimits<int32>::Max());
			if (TrimCandidate.SampleIndex == INDEX_NONE)
			{
				break;
			}

			ApplyDemotion(TrimCandidate.SampleIndex, false);
		}
		if (InOutTimings)
		{
			InOutTimings->StabilizerTrimMs = (FPlatformTime::Seconds() - TrimStart) * 1000.0;
		}
	}
}

static void RunQueuedContinentalStabilizer(
	const FContinentalStabilizerSharedContext& SharedContext,
	const int32 InitialContinentalPlateCount,
	const double MinContinentalAreaFraction,
	FReconcilePhaseTimings* InOutTimings)
{
	ResetStabilizerSubphaseTimings(InOutTimings);

	const int32 NumSamples = SharedContext.Samples.Num();
	if (NumSamples <= 0 || SharedContext.PreviousSamples.Num() != NumSamples || SharedContext.Adjacency.Num() != NumSamples)
	{
		return;
	}

	FContinentalStabilizerWorkingState WorkingState;
	if (!InitializeContinentalStabilizerWorkingState(SharedContext, true, WorkingState))
	{
		return;
	}

	const double ClampedMinimumContinentalFraction = FMath::Clamp(MinContinentalAreaFraction, 0.0, 1.0);
	const int32 MinimumContinentalSamples = FMath::Clamp(
		FMath::CeilToInt(ClampedMinimumContinentalFraction * static_cast<double>(NumSamples)),
		0,
		NumSamples);
	const int32 MaximumContinentalComponents =
		(InitialContinentalPlateCount > 0)
			? (3 * InitialContinentalPlateCount)
			: TNumericLimits<int32>::Max();

	const auto PromotionEntryHigherPriority = [](const FContinentalStabilizerPromotionHeapEntry& A, const FContinentalStabilizerPromotionHeapEntry& B)
	{
		if (A.CurrentContinentalNeighborCount != B.CurrentContinentalNeighborCount)
		{
			return A.CurrentContinentalNeighborCount > B.CurrentContinentalNeighborCount;
		}
		if (A.PreviousContinentalNeighborCount != B.PreviousContinentalNeighborCount)
		{
			return A.PreviousContinentalNeighborCount > B.PreviousContinentalNeighborCount;
		}
		return A.SampleIndex < B.SampleIndex;
	};

	const auto TrimEntryHigherPriority = [](const FContinentalStabilizerTrimHeapEntry& A, const FContinentalStabilizerTrimHeapEntry& B)
	{
		if (A.bWasPreviouslyOceanic != B.bWasPreviouslyOceanic)
		{
			return A.bWasPreviouslyOceanic && !B.bWasPreviouslyOceanic;
		}
		if (A.CurrentContinentalNeighborCount != B.CurrentContinentalNeighborCount)
		{
			return A.CurrentContinentalNeighborCount < B.CurrentContinentalNeighborCount;
		}
		if (A.PreviousContinentalNeighborCount != B.PreviousContinentalNeighborCount)
		{
			return A.PreviousContinentalNeighborCount < B.PreviousContinentalNeighborCount;
		}
		return A.SampleIndex < B.SampleIndex;
	};

	TArray<FContinentalStabilizerPromotionHeapEntry> PromotionHeap;
	TArray<FContinentalStabilizerTrimHeapEntry> TrimHeapLessEqualOne;
	TArray<FContinentalStabilizerTrimHeapEntry> TrimHeapLessEqualTwo;
	TArray<FContinentalStabilizerTrimHeapEntry> TrimHeapLessEqualThree;
	TArray<FContinentalStabilizerTrimHeapEntry> TrimHeapAny;
	PromotionHeap.Reserve(NumSamples);
	TrimHeapLessEqualOne.Reserve(NumSamples);
	TrimHeapLessEqualTwo.Reserve(NumSamples);
	TrimHeapLessEqualThree.Reserve(NumSamples);
	TrimHeapAny.Reserve(NumSamples);
	TArray<FContinentalStabilizerTrimHeapEntry>* TrimHeaps[4] = {
		&TrimHeapLessEqualOne,
		&TrimHeapLessEqualTwo,
		&TrimHeapLessEqualThree,
		&TrimHeapAny
	};

	auto RecordHeapPushes = [InOutTimings](const int32 NumPushes)
	{
		if (InOutTimings)
		{
			InOutTimings->StabilizerHeapPushCount += NumPushes;
		}
	};

	auto RecordStalePop = [InOutTimings]()
	{
		if (InOutTimings)
		{
			++InOutTimings->StabilizerHeapStalePopCount;
		}
	};

	auto EnqueuePromotionCandidate = [&](const int32 SampleIndex)
	{
		if (!SharedContext.Samples.IsValidIndex(SampleIndex) ||
			!WorkingState.PreviousContinentalFlags.IsValidIndex(SampleIndex) ||
			WorkingState.PreviousContinentalFlags[SampleIndex] == 0 ||
			!WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) ||
			WorkingState.CurrentContinentalFlags[SampleIndex] != 0 ||
			SharedContext.Samples[SampleIndex].bGapDetected ||
			!WorkingState.CurrentContinentalNeighborCounts.IsValidIndex(SampleIndex) ||
			!WorkingState.PreviousContinentalNeighborCounts.IsValidIndex(SampleIndex) ||
			!WorkingState.Generations.IsValidIndex(SampleIndex))
		{
			return;
		}

		FContinentalStabilizerPromotionHeapEntry Entry;
		Entry.SampleIndex = SampleIndex;
		Entry.Generation = WorkingState.Generations[SampleIndex];
		Entry.CurrentContinentalNeighborCount = WorkingState.CurrentContinentalNeighborCounts[SampleIndex];
		Entry.PreviousContinentalNeighborCount = WorkingState.PreviousContinentalNeighborCounts[SampleIndex];
		HeapPushEntry(PromotionHeap, Entry, PromotionEntryHigherPriority);
		RecordHeapPushes(1);
	};

	auto EnqueueTrimCandidate = [&](const int32 SampleIndex)
	{
		if (!SharedContext.Samples.IsValidIndex(SampleIndex) ||
			!WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) ||
			WorkingState.CurrentContinentalFlags[SampleIndex] == 0 ||
			SharedContext.Samples[SampleIndex].bGapDetected ||
			!WorkingState.CurrentContinentalNeighborCounts.IsValidIndex(SampleIndex) ||
			!WorkingState.PreviousContinentalNeighborCounts.IsValidIndex(SampleIndex) ||
			!WorkingState.Generations.IsValidIndex(SampleIndex))
		{
			return;
		}

		FContinentalStabilizerTrimHeapEntry Entry;
		Entry.SampleIndex = SampleIndex;
		Entry.Generation = WorkingState.Generations[SampleIndex];
		Entry.bWasPreviouslyOceanic =
			!WorkingState.PreviousContinentalFlags.IsValidIndex(SampleIndex) ||
			WorkingState.PreviousContinentalFlags[SampleIndex] == 0;
		Entry.CurrentContinentalNeighborCount = WorkingState.CurrentContinentalNeighborCounts[SampleIndex];
		Entry.PreviousContinentalNeighborCount = WorkingState.PreviousContinentalNeighborCounts[SampleIndex];

		int32 NumPushes = 0;
		if (Entry.CurrentContinentalNeighborCount <= 1)
		{
			HeapPushEntry(*TrimHeaps[0], Entry, TrimEntryHigherPriority);
			++NumPushes;
		}
		if (Entry.CurrentContinentalNeighborCount <= 2)
		{
			HeapPushEntry(*TrimHeaps[1], Entry, TrimEntryHigherPriority);
			++NumPushes;
		}
		if (Entry.CurrentContinentalNeighborCount <= 3)
		{
			HeapPushEntry(*TrimHeaps[2], Entry, TrimEntryHigherPriority);
			++NumPushes;
		}
		HeapPushEntry(*TrimHeaps[3], Entry, TrimEntryHigherPriority);
		++NumPushes;
		RecordHeapPushes(NumPushes);
	};

	auto EnqueueSampleIfEligible = [&](const int32 SampleIndex)
	{
		EnqueuePromotionCandidate(SampleIndex);
		EnqueueTrimCandidate(SampleIndex);
	};

	auto IsPromotionEntryCurrent = [&](const FContinentalStabilizerPromotionHeapEntry& Entry) -> bool
	{
		const int32 SampleIndex = Entry.SampleIndex;
		return SharedContext.Samples.IsValidIndex(SampleIndex) &&
			WorkingState.Generations.IsValidIndex(SampleIndex) &&
			Entry.Generation == WorkingState.Generations[SampleIndex] &&
			WorkingState.PreviousContinentalFlags.IsValidIndex(SampleIndex) &&
			WorkingState.PreviousContinentalFlags[SampleIndex] != 0 &&
			WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) &&
			WorkingState.CurrentContinentalFlags[SampleIndex] == 0 &&
			!SharedContext.Samples[SampleIndex].bGapDetected &&
			WorkingState.CurrentContinentalNeighborCounts.IsValidIndex(SampleIndex) &&
			Entry.CurrentContinentalNeighborCount == WorkingState.CurrentContinentalNeighborCounts[SampleIndex] &&
			WorkingState.PreviousContinentalNeighborCounts.IsValidIndex(SampleIndex) &&
			Entry.PreviousContinentalNeighborCount == WorkingState.PreviousContinentalNeighborCounts[SampleIndex];
	};

	auto IsTrimEntryCurrent = [&](const FContinentalStabilizerTrimHeapEntry& Entry, const int32 MaxNeighborCount) -> bool
	{
		const int32 SampleIndex = Entry.SampleIndex;
		const bool bWasPreviouslyOceanic =
			!WorkingState.PreviousContinentalFlags.IsValidIndex(SampleIndex) ||
			WorkingState.PreviousContinentalFlags[SampleIndex] == 0;
		return SharedContext.Samples.IsValidIndex(SampleIndex) &&
			WorkingState.Generations.IsValidIndex(SampleIndex) &&
			Entry.Generation == WorkingState.Generations[SampleIndex] &&
			WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) &&
			WorkingState.CurrentContinentalFlags[SampleIndex] != 0 &&
			!SharedContext.Samples[SampleIndex].bGapDetected &&
			Entry.bWasPreviouslyOceanic == bWasPreviouslyOceanic &&
			WorkingState.CurrentContinentalNeighborCounts.IsValidIndex(SampleIndex) &&
			Entry.CurrentContinentalNeighborCount == WorkingState.CurrentContinentalNeighborCounts[SampleIndex] &&
			WorkingState.PreviousContinentalNeighborCounts.IsValidIndex(SampleIndex) &&
			Entry.PreviousContinentalNeighborCount == WorkingState.PreviousContinentalNeighborCounts[SampleIndex] &&
			Entry.CurrentContinentalNeighborCount <= MaxNeighborCount;
	};

	TArray<int32> AffectedSampleIndices;
	auto RefreshAffectedSampleQueues = [&](const int32 MutatedSampleIndex)
	{
		SharedContext.BuildCurrentContinentalNeighborRefreshSet(MutatedSampleIndex, AffectedSampleIndices);
		SharedContext.RefreshCurrentContinentalNeighborCountsForAffectedSamples(
			WorkingState.CurrentContinentalFlags,
			WorkingState.CurrentContinentalNeighborCounts,
			AffectedSampleIndices);
		for (const int32 AffectedSampleIndex : AffectedSampleIndices)
		{
			if (WorkingState.Generations.IsValidIndex(AffectedSampleIndex))
			{
				++WorkingState.Generations[AffectedSampleIndex];
			}
		}
		for (const int32 AffectedSampleIndex : AffectedSampleIndices)
		{
			EnqueueSampleIfEligible(AffectedSampleIndex);
		}
	};

	auto ApplyPromotion = [&](const int32 SampleIndex) -> bool
	{
		if (!WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) || WorkingState.CurrentContinentalFlags[SampleIndex] != 0)
		{
			return false;
		}

		SharedContext.PromoteSampleFromPrevious(SampleIndex);
		if (!SharedContext.Samples.IsValidIndex(SampleIndex) || SharedContext.Samples[SampleIndex].CrustType != ECrustType::Continental)
		{
			return false;
		}

		WorkingState.CurrentContinentalFlags[SampleIndex] = 1;
		++WorkingState.CurrentContinentalCount;
		RefreshAffectedSampleQueues(SampleIndex);
		if (InOutTimings)
		{
			++InOutTimings->StabilizerPromotionCount;
		}
		return true;
	};

	auto ApplyDemotion = [&](const int32 SampleIndex, const bool bComponentDemotion) -> bool
	{
		if (!WorkingState.CurrentContinentalFlags.IsValidIndex(SampleIndex) || WorkingState.CurrentContinentalFlags[SampleIndex] == 0)
		{
			return false;
		}

		SharedContext.DemoteSampleToOceanic(SampleIndex);
		if (!SharedContext.Samples.IsValidIndex(SampleIndex) || SharedContext.Samples[SampleIndex].CrustType != ECrustType::Oceanic)
		{
			return false;
		}

		WorkingState.CurrentContinentalFlags[SampleIndex] = 0;
		--WorkingState.CurrentContinentalCount;
		RefreshAffectedSampleQueues(SampleIndex);
		if (InOutTimings)
		{
			if (bComponentDemotion)
			{
				++InOutTimings->StabilizerComponentDemotionCount;
			}
			else
			{
				++InOutTimings->StabilizerTrimDemotionCount;
			}
		}
		return true;
	};

	auto TryTakeBestPromotionCandidate = [&](const bool bRequireCurrentAdjacency, int32& OutSampleIndex) -> bool
	{
		OutSampleIndex = INDEX_NONE;
		for (;;)
		{
			const FContinentalStabilizerPromotionHeapEntry* RootEntry = HeapPeekEntry(PromotionHeap);
			if (!RootEntry)
			{
				return false;
			}
			if (!IsPromotionEntryCurrent(*RootEntry))
			{
				FContinentalStabilizerPromotionHeapEntry DiscardedEntry;
				HeapTryPopEntry(PromotionHeap, DiscardedEntry, PromotionEntryHigherPriority);
				RecordStalePop();
				continue;
			}
			if (bRequireCurrentAdjacency && RootEntry->CurrentContinentalNeighborCount <= 0)
			{
				return false;
			}

			FContinentalStabilizerPromotionHeapEntry SelectedEntry;
			HeapTryPopEntry(PromotionHeap, SelectedEntry, PromotionEntryHigherPriority);
			OutSampleIndex = SelectedEntry.SampleIndex;
			return true;
		}
	};

	auto TryPromoteContinentalFrontier = [&](const bool bRequireCurrentAdjacency) -> bool
	{
		int32 SampleIndex = INDEX_NONE;
		return TryTakeBestPromotionCandidate(bRequireCurrentAdjacency, SampleIndex) && ApplyPromotion(SampleIndex);
	};

	auto TryTakeBestTrimCandidate = [&](const int32 MaxNeighborCount, int32& OutSampleIndex) -> bool
	{
		const int32 TrimHeapIndex = (MaxNeighborCount <= 1) ? 0 : (MaxNeighborCount <= 2) ? 1 : (MaxNeighborCount <= 3) ? 2 : 3;
		TArray<FContinentalStabilizerTrimHeapEntry>& TrimHeap = *TrimHeaps[TrimHeapIndex];
		OutSampleIndex = INDEX_NONE;
		for (;;)
		{
			const FContinentalStabilizerTrimHeapEntry* RootEntry = HeapPeekEntry(TrimHeap);
			if (!RootEntry)
			{
				return false;
			}
			if (!IsTrimEntryCurrent(*RootEntry, MaxNeighborCount))
			{
				FContinentalStabilizerTrimHeapEntry DiscardedEntry;
				HeapTryPopEntry(TrimHeap, DiscardedEntry, TrimEntryHigherPriority);
				RecordStalePop();
				continue;
			}

			FContinentalStabilizerTrimHeapEntry SelectedEntry;
			HeapTryPopEntry(TrimHeap, SelectedEntry, TrimEntryHigherPriority);
			OutSampleIndex = SelectedEntry.SampleIndex;
			return true;
		}
	};

	for (int32 SampleIndex = 0; SampleIndex < NumSamples; ++SampleIndex)
	{
		EnqueueSampleIfEligible(SampleIndex);
	}

	const double PromoteToMinimumStart = FPlatformTime::Seconds();
	if (WorkingState.CurrentContinentalCount < MinimumContinentalSamples)
	{
		while (WorkingState.CurrentContinentalCount < MinimumContinentalSamples)
		{
			const bool bHaveCurrentContinents = WorkingState.CurrentContinentalCount > 0;
			if (TryPromoteContinentalFrontier(bHaveCurrentContinents))
			{
				continue;
			}

			if (!bHaveCurrentContinents || !TryPromoteContinentalFrontier(false))
			{
				break;
			}
		}
	}
	if (InOutTimings)
	{
		InOutTimings->StabilizerPromoteToMinimumMs = (FPlatformTime::Seconds() - PromoteToMinimumStart) * 1000.0;
	}

	const int32 TargetContinentalCount = FMath::Clamp(
		FMath::Min(WorkingState.CurrentContinentalCount, WorkingState.PreviousContinentalCount),
		0,
		NumSamples);

	const double BuildComponentsStart = FPlatformTime::Seconds();
	TArray<FContinentalStabilizerComponent> Components;
	SharedContext.BuildContinentalComponents(
		WorkingState.CurrentContinentalFlags,
		WorkingState.PreviousContinentalFlags,
		Components);
	if (InOutTimings)
	{
		InOutTimings->StabilizerBuildComponentsMs = (FPlatformTime::Seconds() - BuildComponentsStart) * 1000.0;
	}
	if (Components.Num() <= 0)
	{
		return;
	}

	Components.Sort([](const FContinentalStabilizerComponent& A, const FContinentalStabilizerComponent& B)
	{
		if (A.PreviousContinentalCount != B.PreviousContinentalCount)
		{
			return A.PreviousContinentalCount < B.PreviousContinentalCount;
		}
		if (A.Members.Num() != B.Members.Num())
		{
			return A.Members.Num() < B.Members.Num();
		}
		return A.MinSampleIndex < B.MinSampleIndex;
	});

	const double ComponentPruneStart = FPlatformTime::Seconds();
	int32 RemainingComponentCount = Components.Num();
	for (const FContinentalStabilizerComponent& Component : Components)
	{
		const bool bExceedsComponentCap =
			MaximumContinentalComponents != TNumericLimits<int32>::Max() &&
			RemainingComponentCount > MaximumContinentalComponents;
		const bool bExceedsTargetCount = WorkingState.CurrentContinentalCount > TargetContinentalCount;
		if (!bExceedsComponentCap && !bExceedsTargetCount)
		{
			break;
		}

		for (const int32 SampleIndex : Component.Members)
		{
			ApplyDemotion(SampleIndex, true);
		}
		--RemainingComponentCount;
	}
	if (InOutTimings)
	{
		InOutTimings->StabilizerComponentPruneMs = (FPlatformTime::Seconds() - ComponentPruneStart) * 1000.0;
	}

	const double PromoteToTargetStart = FPlatformTime::Seconds();
	while (WorkingState.CurrentContinentalCount < TargetContinentalCount)
	{
		const bool bRequireCurrentAdjacency = WorkingState.CurrentContinentalCount > 0;
		if (TryPromoteContinentalFrontier(bRequireCurrentAdjacency))
		{
			continue;
		}

		if (!bRequireCurrentAdjacency || !TryPromoteContinentalFrontier(false))
		{
			break;
		}
	}
	if (InOutTimings)
	{
		InOutTimings->StabilizerPromoteToTargetMs = (FPlatformTime::Seconds() - PromoteToTargetStart) * 1000.0;
	}

	const double TrimStart = FPlatformTime::Seconds();
	for (int32 MaxNeighborCount = 1; WorkingState.CurrentContinentalCount > TargetContinentalCount && MaxNeighborCount <= 3; ++MaxNeighborCount)
	{
		while (WorkingState.CurrentContinentalCount > TargetContinentalCount)
		{
			int32 TrimSampleIndex = INDEX_NONE;
			if (!TryTakeBestTrimCandidate(MaxNeighborCount, TrimSampleIndex))
			{
				break;
			}

			ApplyDemotion(TrimSampleIndex, false);
		}
	}

	while (WorkingState.CurrentContinentalCount > TargetContinentalCount)
	{
		int32 TrimSampleIndex = INDEX_NONE;
		if (!TryTakeBestTrimCandidate(TNumericLimits<int32>::Max(), TrimSampleIndex))
		{
			break;
		}

		ApplyDemotion(TrimSampleIndex, false);
	}
	if (InOutTimings)
	{
		InOutTimings->StabilizerTrimMs = (FPlatformTime::Seconds() - TrimStart) * 1000.0;
	}
}
void FTectonicPlanet::StabilizeContinentalCrustForSamples(
	const TArray<FCanonicalSample>& PreviousSamples,
	TArray<FCanonicalSample>& InOutSamples,
	FReconcilePhaseTimings* InOutTimings) const
{
	ResetStabilizerSubphaseTimings(InOutTimings);
	if (InOutTimings)
	{
		InOutTimings->bContinentalStabilizerShadowMatched = true;
		InOutTimings->ContinentalStabilizerShadowMismatchSampleIndex = INDEX_NONE;
		InOutTimings->ContinentalStabilizerShadowMismatchField.Reset();
	}

	StabilizeContinentalCrustForSamplesIncremental(PreviousSamples, InOutSamples, InOutTimings);
}

void FTectonicPlanet::StabilizeContinentalCrustForSamplesIncremental(
	const TArray<FCanonicalSample>& PreviousSamples,
	TArray<FCanonicalSample>& InOutSamples,
	FReconcilePhaseTimings* InOutTimings) const
{
	const FContinentalStabilizerSharedContext SharedContext{
		Adjacency,
		PreviousSamples,
		InOutSamples,
		AbyssalPlainElevationKm,
		OceanicCrustThicknessKm
	};
	RunQueuedContinentalStabilizer(
		SharedContext,
		InitialContinentalPlateCount,
		MinContinentalAreaFraction,
		InOutTimings);
}
void FTectonicPlanet::RunBoundaryLikeLocalOwnershipSanitizePass(
	const TArray<FPhase2SampleState>& Phase2States,
	TArray<FCanonicalSample>& InOutSamples,
	int32 NumIterations,
	const TArray<uint8>* SeedSampleFlags) const
{
	if (Adjacency.Num() != InOutSamples.Num() || NumIterations <= 0)
	{
		return;
	}

	const bool bUseSeedFlags = SeedSampleFlags && SeedSampleFlags->Num() == InOutSamples.Num();
	constexpr int32 MaxCleanupPlateBins = 16;
	TArray<int32> CleanupAssignments;
	CleanupAssignments.Init(INDEX_NONE, InOutSamples.Num());
	for (int32 CleanupIteration = 0; CleanupIteration < NumIterations; ++CleanupIteration)
	{
		TArray<int32> ProjectedNonGapCountsByPlate;
		ProjectedNonGapCountsByPlate.Init(0, Plates.Num());
		for (const FCanonicalSample& Sample : InOutSamples)
		{
			if (!Sample.bGapDetected && Plates.IsValidIndex(Sample.PlateId))
			{
				++ProjectedNonGapCountsByPlate[Sample.PlateId];
			}
		}

		bool bAnyCleanupAssignments = false;
		for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
		{
			CleanupAssignments[SampleIndex] = INDEX_NONE;
			if (bUseSeedFlags && (*SeedSampleFlags)[SampleIndex] == 0)
			{
				continue;
			}
			if (!Adjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FCanonicalSample& Sample = InOutSamples[SampleIndex];
			if (!Plates.IsValidIndex(Sample.PlateId) || Sample.bGapDetected)
			{
				continue;
			}

			const FPhase2SampleState* State = Phase2States.IsValidIndex(SampleIndex) ? &Phase2States[SampleIndex] : nullptr;
			const bool bBoundaryLike = Sample.bOverlapDetected ||
				(State && (State->bOverlap || State->bGap || (State->OwnershipMargin < (BoundaryConfidenceThreshold * 2.0f))));
			if (!bBoundaryLike)
			{
				continue;
			}

			int32 NeighborPlateIds[MaxCleanupPlateBins];
			int32 NeighborPlateCounts[MaxCleanupPlateBins];
			int32 NumNeighborPlates = 0;
			int32 OwnPlateNeighborCount = 0;
			int32 TotalValidNeighbors = 0;
			for (int32 SlotIndex = 0; SlotIndex < MaxCleanupPlateBins; ++SlotIndex)
			{
				NeighborPlateIds[SlotIndex] = INDEX_NONE;
				NeighborPlateCounts[SlotIndex] = 0;
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

				++TotalValidNeighbors;
				if (NeighborPlateId == Sample.PlateId)
				{
					++OwnPlateNeighborCount;
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

				if (ExistingSlot == INDEX_NONE && NumNeighborPlates < MaxCleanupPlateBins)
				{
					ExistingSlot = NumNeighborPlates++;
					NeighborPlateIds[ExistingSlot] = NeighborPlateId;
				}
				if (ExistingSlot != INDEX_NONE)
				{
					++NeighborPlateCounts[ExistingSlot];
				}
			}

			if (TotalValidNeighbors <= 0)
			{
				continue;
			}

			int32 BestNeighborPlateId = INDEX_NONE;
			int32 BestNeighborCount = 0;
			for (int32 SlotIndex = 0; SlotIndex < NumNeighborPlates; ++SlotIndex)
			{
				if (NeighborPlateCounts[SlotIndex] > BestNeighborCount)
				{
					BestNeighborCount = NeighborPlateCounts[SlotIndex];
					BestNeighborPlateId = NeighborPlateIds[SlotIndex];
				}
			}

			const bool bWeakLocalSupport = OwnPlateNeighborCount <= 2;
			const bool bStrongNeighborMajority = BestNeighborCount >= FMath::Max(4, OwnPlateNeighborCount + 3);
			const bool bNeighborSupermajority = (BestNeighborCount * 2) >= (TotalValidNeighbors + 2);
			if (!Plates.IsValidIndex(BestNeighborPlateId) || !bWeakLocalSupport || !bStrongNeighborMajority || !bNeighborSupermajority)
			{
				continue;
			}

			if (IsPlateProtected(Sample.PlateId))
			{
				const int32 PersistentFloorSamples = GetPersistentPlateFloorSamples(Sample.PlateId);
				if (ProjectedNonGapCountsByPlate[Sample.PlateId] <= PersistentFloorSamples)
				{
					continue;
				}
			}

			CleanupAssignments[SampleIndex] = BestNeighborPlateId;
			if (ProjectedNonGapCountsByPlate.IsValidIndex(Sample.PlateId))
			{
				--ProjectedNonGapCountsByPlate[Sample.PlateId];
			}
			if (ProjectedNonGapCountsByPlate.IsValidIndex(BestNeighborPlateId))
			{
				++ProjectedNonGapCountsByPlate[BestNeighborPlateId];
			}
			bAnyCleanupAssignments = true;
		}

		if (!bAnyCleanupAssignments)
		{
			break;
		}

		for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
		{
			if (Plates.IsValidIndex(CleanupAssignments[SampleIndex]))
			{
				InOutSamples[SampleIndex].PlateId = CleanupAssignments[SampleIndex];
			}
		}
	}
}

void FTectonicPlanet::EnforceConnectedPlateOwnershipForSamples(TArray<FCanonicalSample>& InOutSamples, const TArray<uint8>* CandidatePlateFlags) const
{
	if (Plates.Num() == 0 || Adjacency.Num() != InOutSamples.Num())
	{
		return;
	}

	// Pre-rifting invariant: before Milestone 5 adds rifting, each plate's non-gap ownership must stay connected.
	constexpr int32 MaxConnectivityIterations = 8;

	struct FPlateComponentInfo
	{
		TArray<int32> Members;
		int32 PrevMatchCount = 0;
		int32 MinSampleIndex = TNumericLimits<int32>::Max();
	};

	auto FindPrimaryComponentIndex = [this](const TArray<FPlateComponentInfo>& Components, const int32 PlateId) -> int32
	{
		const int32 PersistentFloorSamples = IsPlateProtected(PlateId) ? GetPersistentPlateFloorSamples(PlateId) : 0;
		int32 BestIndex = INDEX_NONE;
		for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
		{
			const FPlateComponentInfo& Candidate = Components[ComponentIndex];
			const bool bCandidateSatisfiesFloor = PersistentFloorSamples <= 0 || Candidate.Members.Num() >= PersistentFloorSamples;
			if (BestIndex == INDEX_NONE)
			{
				BestIndex = ComponentIndex;
				continue;
			}

			const FPlateComponentInfo& Best = Components[BestIndex];
			const bool bBestSatisfiesFloor = PersistentFloorSamples <= 0 || Best.Members.Num() >= PersistentFloorSamples;
			if ((bCandidateSatisfiesFloor && !bBestSatisfiesFloor) ||
				(bCandidateSatisfiesFloor == bBestSatisfiesFloor && Candidate.PrevMatchCount > Best.PrevMatchCount) ||
				(bCandidateSatisfiesFloor == bBestSatisfiesFloor && Candidate.PrevMatchCount == Best.PrevMatchCount && Candidate.Members.Num() > Best.Members.Num()) ||
				(bCandidateSatisfiesFloor == bBestSatisfiesFloor && Candidate.PrevMatchCount == Best.PrevMatchCount && Candidate.Members.Num() == Best.Members.Num() && Candidate.MinSampleIndex < Best.MinSampleIndex))
			{
				BestIndex = ComponentIndex;
			}
		}
		return BestIndex;
	};

	auto SelectBestNeighborPlate = [this](const TMap<int32, int32>& EdgeCounts) -> int32
	{
		int32 BestNeighborPlateId = INDEX_NONE;
		int32 BestEdgeCount = -1;
		for (const TPair<int32, int32>& Entry : EdgeCounts)
		{
			if (!Plates.IsValidIndex(Entry.Key))
			{
				continue;
			}

			if (Entry.Value > BestEdgeCount ||
				(Entry.Value == BestEdgeCount && (BestNeighborPlateId == INDEX_NONE || Entry.Key < BestNeighborPlateId)))
			{
				BestNeighborPlateId = Entry.Key;
				BestEdgeCount = Entry.Value;
			}
		}
		return BestNeighborPlateId;
	};

	TArray<uint8> Visited;
	TArray<int32> Stack;
	TArray<int32> Reassignments;
	TArray<int32> SampleComponentIndices;
	TArray<uint8> PrimaryComponentSampleFlags;
	TArray<int32> PrimaryComponentIndicesByPlate;

	for (int32 ConnectivityIteration = 0; ConnectivityIteration < MaxConnectivityIterations; ++ConnectivityIteration)
	{
		Visited.Init(0, InOutSamples.Num());
		TArray<TArray<FPlateComponentInfo>> ComponentsByPlate;
		ComponentsByPlate.SetNum(Plates.Num());
		SampleComponentIndices.Init(INDEX_NONE, InOutSamples.Num());
		PrimaryComponentSampleFlags.Init(0, InOutSamples.Num());
		PrimaryComponentIndicesByPlate.Init(INDEX_NONE, Plates.Num());
		TArray<int32> CurrentNonGapCountsByPlate;
		CurrentNonGapCountsByPlate.Init(0, Plates.Num());

		for (int32 SeedIndex = 0; SeedIndex < InOutSamples.Num(); ++SeedIndex)
		{
			if (Visited[SeedIndex] != 0 || InOutSamples[SeedIndex].bGapDetected || !Plates.IsValidIndex(InOutSamples[SeedIndex].PlateId) || (CandidatePlateFlags && (!CandidatePlateFlags->IsValidIndex(InOutSamples[SeedIndex].PlateId) || (*CandidatePlateFlags)[InOutSamples[SeedIndex].PlateId] == 0)))
			{
				continue;
			}

			const int32 PlateId = InOutSamples[SeedIndex].PlateId;
			const int32 ComponentIndex = ComponentsByPlate[PlateId].Num();
			FPlateComponentInfo Component;
			Visited[SeedIndex] = 1;
			++CurrentNonGapCountsByPlate[PlateId];
			Stack.Reset();
			Stack.Add(SeedIndex);
			while (Stack.Num() > 0)
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				SampleComponentIndices[SampleIndex] = ComponentIndex;
				Component.Members.Add(SampleIndex);
				Component.MinSampleIndex = FMath::Min(Component.MinSampleIndex, SampleIndex);
				Component.PrevMatchCount += (InOutSamples[SampleIndex].PrevPlateId == PlateId) ? 1 : 0;
				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					if (!InOutSamples.IsValidIndex(NeighborIndex) || Visited[NeighborIndex] != 0)
					{
						continue;
					}
					if (InOutSamples[NeighborIndex].bGapDetected || InOutSamples[NeighborIndex].PlateId != PlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					++CurrentNonGapCountsByPlate[PlateId];
					Stack.Add(NeighborIndex);
				}
			}

			ComponentsByPlate[PlateId].Add(MoveTemp(Component));
		}

		bool bAnyFragmentedPlate = false;
		for (int32 PlateId = 0; PlateId < ComponentsByPlate.Num(); ++PlateId)
		{
			if (CandidatePlateFlags && (!CandidatePlateFlags->IsValidIndex(PlateId) || (*CandidatePlateFlags)[PlateId] == 0))
			{
				continue;
			}
			const TArray<FPlateComponentInfo>& Components = ComponentsByPlate[PlateId];
			if (Components.Num() <= 0)
			{
				continue;
			}

			const int32 PrimaryComponentIndex = FindPrimaryComponentIndex(Components, PlateId);
			PrimaryComponentIndicesByPlate[PlateId] = PrimaryComponentIndex;
			if (PrimaryComponentIndex != INDEX_NONE)
			{
				for (const int32 SampleIndex : Components[PrimaryComponentIndex].Members)
				{
					PrimaryComponentSampleFlags[SampleIndex] = 1;
				}
			}

			bAnyFragmentedPlate |= Components.Num() > 1;
		}

		if (!bAnyFragmentedPlate)
		{
			break;
		}

		Reassignments.Init(INDEX_NONE, InOutSamples.Num());
		TArray<int32> ProjectedNonGapCountsByPlate = CurrentNonGapCountsByPlate;
		bool bAnyReassignments = false;
		for (int32 PlateId = 0; PlateId < ComponentsByPlate.Num(); ++PlateId)
		{
			if (CandidatePlateFlags && (!CandidatePlateFlags->IsValidIndex(PlateId) || (*CandidatePlateFlags)[PlateId] == 0))
			{
				continue;
			}
			const TArray<FPlateComponentInfo>& Components = ComponentsByPlate[PlateId];
			if (Components.Num() <= 1)
			{
				continue;
			}

			const int32 PrimaryComponentIndex = PrimaryComponentIndicesByPlate[PlateId];
			for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
			{
				if (ComponentIndex == PrimaryComponentIndex)
				{
					continue;
				}

				const FPlateComponentInfo& Component = Components[ComponentIndex];
				TMap<int32, int32> PrimaryEdgeCounts;
				TMap<int32, int32> ExternalEdgeCounts;
				for (const int32 SampleIndex : Component.Members)
				{
					for (const int32 NeighborIndex : Adjacency[SampleIndex])
					{
						if (!InOutSamples.IsValidIndex(NeighborIndex) || InOutSamples[NeighborIndex].bGapDetected)
						{
							continue;
						}

						const int32 NeighborPlateId = InOutSamples[NeighborIndex].PlateId;
						if (!Plates.IsValidIndex(NeighborPlateId) || NeighborPlateId == PlateId)
						{
							continue;
						}

						ExternalEdgeCounts.FindOrAdd(NeighborPlateId) += 1;
						if (PrimaryComponentSampleFlags[NeighborIndex] != 0 &&
							SampleComponentIndices[NeighborIndex] == PrimaryComponentIndicesByPlate[NeighborPlateId])
						{
							PrimaryEdgeCounts.FindOrAdd(NeighborPlateId) += 1;
						}
					}
				}

				int32 BestNeighborPlateId = SelectBestNeighborPlate(PrimaryEdgeCounts);
				if (!Plates.IsValidIndex(BestNeighborPlateId))
				{
					BestNeighborPlateId = SelectBestNeighborPlate(ExternalEdgeCounts);
				}
				if (!Plates.IsValidIndex(BestNeighborPlateId))
				{
					continue;
				}

				if (IsPlateProtected(PlateId))
				{
					const int32 PersistentFloorSamples = GetPersistentPlateFloorSamples(PlateId);
					if (ProjectedNonGapCountsByPlate[PlateId] - Component.Members.Num() < PersistentFloorSamples)
					{
						continue;
					}
				}

				for (const int32 SampleIndex : Component.Members)
				{
					Reassignments[SampleIndex] = BestNeighborPlateId;
				}
				ProjectedNonGapCountsByPlate[PlateId] -= Component.Members.Num();
				bAnyReassignments = true;
			}
		}

		if (!bAnyReassignments)
		{
			break;
		}

		for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
		{
			if (Plates.IsValidIndex(Reassignments[SampleIndex]))
			{
				InOutSamples[SampleIndex].PlateId = Reassignments[SampleIndex];
			}
		}
	}
}

bool FTectonicPlanet::RescueProtectedPlateOwnershipForSamples(TArray<FCanonicalSample>& InOutSamples, const TArray<uint8>* CandidatePlateFlags)
{
	if (Plates.Num() == 0 || Adjacency.Num() != InOutSamples.Num())
	{
		return false;
	}

	struct FPlateComponentInfo
	{
		TArray<int32> Members;
		int32 PrevMatchCount = 0;
		int32 MinSampleIndex = TNumericLimits<int32>::Max();
	};

	auto FindPrimaryComponentIndex = [this](const TArray<FPlateComponentInfo>& Components, const int32 PlateId) -> int32
	{
		const int32 PersistentFloorSamples = IsPlateProtected(PlateId) ? GetPersistentPlateFloorSamples(PlateId) : 0;
		int32 BestIndex = INDEX_NONE;
		for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
		{
			const FPlateComponentInfo& Candidate = Components[ComponentIndex];
			const bool bCandidateSatisfiesFloor = PersistentFloorSamples <= 0 || Candidate.Members.Num() >= PersistentFloorSamples;
			if (BestIndex == INDEX_NONE)
			{
				BestIndex = ComponentIndex;
				continue;
			}

			const FPlateComponentInfo& Best = Components[BestIndex];
			const bool bBestSatisfiesFloor = PersistentFloorSamples <= 0 || Best.Members.Num() >= PersistentFloorSamples;
			if ((bCandidateSatisfiesFloor && !bBestSatisfiesFloor) ||
				(bCandidateSatisfiesFloor == bBestSatisfiesFloor && Candidate.PrevMatchCount > Best.PrevMatchCount) ||
				(bCandidateSatisfiesFloor == bBestSatisfiesFloor && Candidate.PrevMatchCount == Best.PrevMatchCount && Candidate.Members.Num() > Best.Members.Num()) ||
				(bCandidateSatisfiesFloor == bBestSatisfiesFloor && Candidate.PrevMatchCount == Best.PrevMatchCount && Candidate.Members.Num() == Best.Members.Num() && Candidate.MinSampleIndex < Best.MinSampleIndex))
			{
				BestIndex = ComponentIndex;
			}
		}
		return BestIndex;
	};

	auto BuildPlateComponents = [this, &InOutSamples](const int32 PlateId, TArray<FPlateComponentInfo>& OutComponents, int32& OutNonGapCount)
	{
		OutComponents.Reset();
		OutNonGapCount = 0;

		TArray<uint8> Visited;
		Visited.Init(0, InOutSamples.Num());
		TArray<int32> Stack;
		Stack.Reserve(256);

		for (int32 SeedIndex = 0; SeedIndex < InOutSamples.Num(); ++SeedIndex)
		{
			if (Visited[SeedIndex] != 0)
			{
				continue;
			}

			const FCanonicalSample& SeedSample = InOutSamples[SeedIndex];
			if (SeedSample.bGapDetected || SeedSample.PlateId != PlateId)
			{
				continue;
			}

			FPlateComponentInfo& Component = OutComponents.Emplace_GetRef();
			Visited[SeedIndex] = 1;
			Stack.Reset();
			Stack.Add(SeedIndex);
			while (Stack.Num() > 0)
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				const FCanonicalSample& Sample = InOutSamples[SampleIndex];
				Component.Members.Add(SampleIndex);
				Component.MinSampleIndex = FMath::Min(Component.MinSampleIndex, SampleIndex);
				Component.PrevMatchCount += (Sample.PrevPlateId == PlateId) ? 1 : 0;
				++OutNonGapCount;

				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					if (!InOutSamples.IsValidIndex(NeighborIndex) || Visited[NeighborIndex] != 0)
					{
						continue;
					}

					const FCanonicalSample& NeighborSample = InOutSamples[NeighborIndex];
					if (NeighborSample.bGapDetected || NeighborSample.PlateId != PlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}
		}
	};

	TArray<int32> CurrentNonGapCountsByPlate;
	CurrentNonGapCountsByPlate.Init(0, Plates.Num());
	for (const FCanonicalSample& Sample : InOutSamples)
	{
		if (!Sample.bGapDetected && Plates.IsValidIndex(Sample.PlateId))
		{
			++CurrentNonGapCountsByPlate[Sample.PlateId];
		}
	}

	TArray<int32> PlatesNeedingRescue;
	for (const FPlate& Plate : Plates)
	{
		if (CandidatePlateFlags && (!CandidatePlateFlags->IsValidIndex(Plate.Id) || (*CandidatePlateFlags)[Plate.Id] == 0))
		{
			continue;
		}
		if (Plate.PersistencePolicy != EPlatePersistencePolicy::Protected)
		{
			continue;
		}

		const int32 PersistentFloorSamples = GetPersistentPlateFloorSamples(Plate.Id);
		if (PersistentFloorSamples <= 0)
		{
			continue;
		}

		TArray<FPlateComponentInfo> Components;
		int32 NonGapCount = 0;
		BuildPlateComponents(Plate.Id, Components, NonGapCount);
		const int32 PrimaryComponentIndex = FindPrimaryComponentIndex(Components, Plate.Id);
		const int32 PrimaryComponentSize =
			(PrimaryComponentIndex != INDEX_NONE && Components.IsValidIndex(PrimaryComponentIndex))
				? Components[PrimaryComponentIndex].Members.Num()
				: 0;
		if (PrimaryComponentSize < PersistentFloorSamples)
		{
			PlatesNeedingRescue.Add(Plate.Id);
		}
	}

	if (PlatesNeedingRescue.Num() <= 0)
	{
		return false;
	}

	PlatesNeedingRescue.Sort([&CurrentNonGapCountsByPlate](const int32 A, const int32 B)
	{
		const int32 CountA = CurrentNonGapCountsByPlate.IsValidIndex(A) ? CurrentNonGapCountsByPlate[A] : 0;
		const int32 CountB = CurrentNonGapCountsByPlate.IsValidIndex(B) ? CurrentNonGapCountsByPlate[B] : 0;
		return (CountA == CountB) ? (A < B) : (CountA < CountB);
	});

	auto CanBorrowFromDonorPlate = [this, &CurrentNonGapCountsByPlate](const int32 DonorPlateId) -> bool
	{
		if (!Plates.IsValidIndex(DonorPlateId) || !CurrentNonGapCountsByPlate.IsValidIndex(DonorPlateId) || CurrentNonGapCountsByPlate[DonorPlateId] <= 0)
		{
			return false;
		}

		if (!IsPlateProtected(DonorPlateId))
		{
			return true;
		}

		return (CurrentNonGapCountsByPlate[DonorPlateId] - 1) >= GetPersistentPlateFloorSamples(DonorPlateId);
	};

	TArray<uint8> RegionFlags;
	RegionFlags.Init(0, InOutSamples.Num());
	TArray<int32> RegionSamples;
	RegionSamples.Reserve(256);

	bool bAnyRescueApplied = false;
	for (const int32 PlateId : PlatesNeedingRescue)
	{
		if (CandidatePlateFlags && (!CandidatePlateFlags->IsValidIndex(PlateId) || (*CandidatePlateFlags)[PlateId] == 0))
		{
			continue;
		}
		if (!IsPlateProtected(PlateId))
		{
			continue;
		}

		for (const int32 SampleIndex : RegionSamples)
		{
			if (RegionFlags.IsValidIndex(SampleIndex))
			{
				RegionFlags[SampleIndex] = 0;
			}
		}
		RegionSamples.Reset();

		TArray<FPlateComponentInfo> Components;
		int32 CurrentNonGapCount = 0;
		BuildPlateComponents(PlateId, Components, CurrentNonGapCount);
		if (CurrentNonGapCountsByPlate.IsValidIndex(PlateId))
		{
			CurrentNonGapCountsByPlate[PlateId] = CurrentNonGapCount;
		}

		const int32 PersistentFloorSamples = GetPersistentPlateFloorSamples(PlateId);
		const int32 PrimaryComponentIndex = FindPrimaryComponentIndex(Components, PlateId);
		if (PrimaryComponentIndex != INDEX_NONE && Components.IsValidIndex(PrimaryComponentIndex))
		{
			for (const int32 SampleIndex : Components[PrimaryComponentIndex].Members)
			{
				RegionFlags[SampleIndex] = 1;
				RegionSamples.Add(SampleIndex);
			}
		}

		if (RegionSamples.Num() >= PersistentFloorSamples)
		{
			continue;
		}

		const FVector RescueAnchor = !Plates[PlateId].CanonicalCenterDirection.IsNearlyZero()
			? Plates[PlateId].CanonicalCenterDirection.GetSafeNormal()
			: (Plates[PlateId].IdentityAnchorDirection.IsNearlyZero()
				? FVector::ForwardVector
				: Plates[PlateId].IdentityAnchorDirection.GetSafeNormal());

		int32 RescuedSamplesForPlate = 0;
		auto ClaimSampleForPlate = [this, &InOutSamples, &CurrentNonGapCountsByPlate, &RegionFlags, &RegionSamples, PlateId, &RescuedSamplesForPlate](const int32 SampleIndex) -> bool
		{
			if (!InOutSamples.IsValidIndex(SampleIndex))
			{
				return false;
			}

			FCanonicalSample& Sample = InOutSamples[SampleIndex];
			if (Sample.bGapDetected || Sample.PlateId == PlateId || !Plates.IsValidIndex(Sample.PlateId))
			{
				return false;
			}

			const int32 DonorPlateId = Sample.PlateId;
			if (CurrentNonGapCountsByPlate.IsValidIndex(DonorPlateId))
			{
				--CurrentNonGapCountsByPlate[DonorPlateId];
			}
			if (CurrentNonGapCountsByPlate.IsValidIndex(PlateId))
			{
				++CurrentNonGapCountsByPlate[PlateId];
			}

			Sample.PlateId = PlateId;
			if (!RegionFlags[SampleIndex])
			{
				RegionFlags[SampleIndex] = 1;
				RegionSamples.Add(SampleIndex);
			}

			++RescuedSamplesForPlate;
			++RescuedProtectedSampleCount;
			return true;
		};

		if (RegionSamples.Num() == 0)
		{
			int32 BestSeedSampleIndex = INDEX_NONE;
			double BestAnchorDot = -TNumericLimits<double>::Max();
			int32 BestDonorSurplus = TNumericLimits<int32>::Lowest();
			for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
			{
				const FCanonicalSample& CandidateSample = InOutSamples[SampleIndex];
				if (CandidateSample.bGapDetected || !Plates.IsValidIndex(CandidateSample.PlateId) || CandidateSample.PlateId == PlateId)
				{
					continue;
				}

				if (!CanBorrowFromDonorPlate(CandidateSample.PlateId))
				{
					continue;
				}

				const int32 DonorFloor = IsPlateProtected(CandidateSample.PlateId) ? GetPersistentPlateFloorSamples(CandidateSample.PlateId) : 0;
				const int32 DonorSurplus = CurrentNonGapCountsByPlate[CandidateSample.PlateId] - DonorFloor;
				const double AnchorDot = FVector::DotProduct(CandidateSample.Position, RescueAnchor);
				if (AnchorDot > BestAnchorDot + UE_DOUBLE_SMALL_NUMBER ||
					(FMath::IsNearlyEqual(AnchorDot, BestAnchorDot, UE_DOUBLE_SMALL_NUMBER) && DonorSurplus > BestDonorSurplus) ||
					(FMath::IsNearlyEqual(AnchorDot, BestAnchorDot, UE_DOUBLE_SMALL_NUMBER) && DonorSurplus == BestDonorSurplus && (BestSeedSampleIndex == INDEX_NONE || SampleIndex < BestSeedSampleIndex)))
				{
					BestSeedSampleIndex = SampleIndex;
					BestAnchorDot = AnchorDot;
					BestDonorSurplus = DonorSurplus;
				}
			}

			if (BestSeedSampleIndex != INDEX_NONE)
			{
				ClaimSampleForPlate(BestSeedSampleIndex);
			}
		}

		while (RegionSamples.Num() > 0 && RegionSamples.Num() < PersistentFloorSamples)
		{
			TMap<int32, int32> CandidateAdjacencyCounts;
			for (const int32 RegionSampleIndex : RegionSamples)
			{
				for (const int32 NeighborIndex : Adjacency[RegionSampleIndex])
				{
					if (!InOutSamples.IsValidIndex(NeighborIndex) || RegionFlags[NeighborIndex] != 0)
					{
						continue;
					}

					const FCanonicalSample& CandidateSample = InOutSamples[NeighborIndex];
					if (CandidateSample.bGapDetected || !Plates.IsValidIndex(CandidateSample.PlateId) || CandidateSample.PlateId == PlateId)
					{
						continue;
					}

					if (!CanBorrowFromDonorPlate(CandidateSample.PlateId))
					{
						continue;
					}

					CandidateAdjacencyCounts.FindOrAdd(NeighborIndex) += 1;
				}
			}

			int32 BestCandidateIndex = INDEX_NONE;
			int32 BestAdjacencyCount = -1;
			float BestOwnershipMargin = TNumericLimits<float>::Max();
			double BestAnchorDot = -TNumericLimits<double>::Max();
			for (const TPair<int32, int32>& Entry : CandidateAdjacencyCounts)
			{
				const int32 CandidateIndex = Entry.Key;
				const FCanonicalSample& CandidateSample = InOutSamples[CandidateIndex];
				const double AnchorDot = FVector::DotProduct(CandidateSample.Position, RescueAnchor);
				if (Entry.Value > BestAdjacencyCount ||
					(Entry.Value == BestAdjacencyCount && CandidateSample.OwnershipMargin < BestOwnershipMargin - KINDA_SMALL_NUMBER) ||
					(Entry.Value == BestAdjacencyCount && FMath::IsNearlyEqual(CandidateSample.OwnershipMargin, BestOwnershipMargin, KINDA_SMALL_NUMBER) && AnchorDot > BestAnchorDot + UE_DOUBLE_SMALL_NUMBER) ||
					(Entry.Value == BestAdjacencyCount && FMath::IsNearlyEqual(CandidateSample.OwnershipMargin, BestOwnershipMargin, KINDA_SMALL_NUMBER) && FMath::IsNearlyEqual(AnchorDot, BestAnchorDot, UE_DOUBLE_SMALL_NUMBER) && (BestCandidateIndex == INDEX_NONE || CandidateIndex < BestCandidateIndex)))
				{
					BestCandidateIndex = CandidateIndex;
					BestAdjacencyCount = Entry.Value;
					BestOwnershipMargin = CandidateSample.OwnershipMargin;
					BestAnchorDot = AnchorDot;
				}
			}

			if (BestCandidateIndex == INDEX_NONE || !ClaimSampleForPlate(BestCandidateIndex))
			{
				break;
			}
		}

		if (RescuedSamplesForPlate > 0)
		{
			++RescuedProtectedPlateCount;
			bAnyRescueApplied = true;
		}
	}

	return bAnyRescueApplied;
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

		const FVector3d Barycentric = NormalizeContainmentBarycentric(ComputePlanarBarycentric(A, B, C, QueryPointLocal));
		const int32 GlobalTriangleIndex = PlateState.SoupData.GlobalTriangleIndices[LocalTriangleId];
		if (!Triangles.IsValidIndex(GlobalTriangleIndex) || !IsPreferredPlateForTriangle(Samples, Triangles[GlobalTriangleIndex], PlateState.PlateId, Barycentric))
		{
			continue;
		}

		++Result.NumContainingPlates;
		if (Result.NumContainingPlates <= UE_ARRAY_COUNT(Result.ContainingPlateIds))
		{
			Result.ContainingPlateIds[Result.NumContainingPlates - 1] = PlateState.PlateId;
		}

		const double ContainmentScore = ComputeContainmentScore(Barycentric);
		if (IsBetterContainmentCandidate(ContainmentScore, PlateState.PlateId, BestContainmentScore, Result.PlateId))
		{
			BestContainmentScore = ContainmentScore;
			Result.PlateId = PlateState.PlateId;
			Result.TriangleIndex = GlobalTriangleIndex;
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
		BoundaryMeanDepthHops = 0.0;
		BoundaryMaxDepthHops = 0;
		BoundaryDeepSampleCount = 0;
		ContinentalSampleCount = 0;
		ContinentalComponentCount = 0;
		LargestContinentalComponentSize = 0;
		MaxPlateComponentCount = 0;
		DetachedPlateFragmentSampleCount = 0;
		LargestDetachedPlateFragmentSize = 0;
		SubductionFrontSampleCount = 0;
		AndeanSampleCount = 0;
		TrackedTerraneCount = 0;
		ActiveTerraneCount = 0;
		MergedTerraneCount = 0;
		CollisionEventCount = CollisionEvents.Num();
		HimalayanSampleCount = 0;
		PendingCollisionSampleCount = 0;
		MaxSubductionDistanceKm = 0.0f;
		InvalidateSpatialQueryData();
		return;
	}

	TArray<uint64> PreviousSoupHashes;
	TArray<int32> PreviousSoupCounts;
	PreviousSoupHashes.SetNumZeroed(NumPlates);
	PreviousSoupCounts.SetNumZeroed(NumPlates);
	for (int32 PlateIndex = 0; PlateIndex < NumPlates; ++PlateIndex)
	{
		const FPlate& Plate = Plates[PlateIndex];
		PreviousSoupCounts[PlateIndex] = Plate.InteriorTriangles.Num() + Plate.BoundaryTriangles.Num();
		PreviousSoupHashes[PlateIndex] = HashPlateSoupTriangleLists(Plate);
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
	UpdateBoundaryAndContinentDiagnosticsForSamples(InOutSamples);

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
			const FPlate& Plate = Plates[PlateIndex];
			const int32 NewCount = Plate.InteriorTriangles.Num() + Plate.BoundaryTriangles.Num();
			const uint64 NewHash = HashPlateSoupTriangleLists(Plate);
			const bool bSoupChanged = (NewCount != PreviousSoupCounts[PlateIndex]) || (NewHash != PreviousSoupHashes[PlateIndex]);
			if (bSoupChanged)
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
	UpdateBoundaryAndContinentDiagnosticsForSamples(InOutSamples);
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
		if (!Sample.bGapDetected)
		{
			Sample.FlankingPlateIdA = INDEX_NONE;
			Sample.FlankingPlateIdB = INDEX_NONE;
			continue;
		}

		const bool bHasValidFlankA = Plates.IsValidIndex(Sample.FlankingPlateIdA);
		const bool bHasValidFlankB = Plates.IsValidIndex(Sample.FlankingPlateIdB) && Sample.FlankingPlateIdB != Sample.FlankingPlateIdA;
		if (bHasValidFlankA && bHasValidFlankB)
		{
			continue;
		}

		Sample.FlankingPlateIdA = INDEX_NONE;
		Sample.FlankingPlateIdB = INDEX_NONE;

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

void FTectonicPlanet::UpdateBoundaryAndContinentDiagnosticsForSamples(const TArray<FCanonicalSample>& InSamples)
{
	BoundaryMeanDepthHops = 0.0;
	BoundaryMaxDepthHops = 0;
	BoundaryDeepSampleCount = 0;
	ContinentalSampleCount = 0;
	ContinentalPlateCount = 0;
	ContinentalAreaFraction = 0.0;
	ContinentalComponentCount = 0;
	LargestContinentalComponentSize = 0;
	MaxPlateComponentCount = 0;
	DetachedPlateFragmentSampleCount = 0;
	LargestDetachedPlateFragmentSize = 0;

	const int32 NumSamples = InSamples.Num();
	if (NumSamples <= 0)
	{
		return;
	}

	TArray<uint8> PlatesWithContinentalSamples;
	PlatesWithContinentalSamples.Init(0, Plates.Num());
	for (const FCanonicalSample& Sample : InSamples)
	{
		ContinentalSampleCount += (Sample.CrustType == ECrustType::Continental) ? 1 : 0;
		if (Sample.CrustType == ECrustType::Continental && PlatesWithContinentalSamples.IsValidIndex(Sample.PlateId))
		{
			PlatesWithContinentalSamples[Sample.PlateId] = 1;
		}
	}

	ContinentalAreaFraction = static_cast<double>(ContinentalSampleCount) / static_cast<double>(NumSamples);
	for (const uint8 bHasContinentalSamples : PlatesWithContinentalSamples)
	{
		ContinentalPlateCount += (bHasContinentalSamples != 0) ? 1 : 0;
	}

	if (Adjacency.Num() != NumSamples)
	{
		return;
	}

	if (BoundarySampleCount > 0)
	{
		TArray<int32> DistanceToInterior;
		DistanceToInterior.Init(INDEX_NONE, NumSamples);
		TArray<int32> Queue;
		Queue.Reserve(NumSamples);
		for (int32 SampleIndex = 0; SampleIndex < NumSamples; ++SampleIndex)
		{
			if (!InSamples[SampleIndex].bIsBoundary)
			{
				DistanceToInterior[SampleIndex] = 0;
				Queue.Add(SampleIndex);
			}
		}

		for (int32 QueueIndex = 0; QueueIndex < Queue.Num(); ++QueueIndex)
		{
			const int32 SampleIndex = Queue[QueueIndex];
			const int32 NextDistance = DistanceToInterior[SampleIndex] + 1;
			for (const int32 NeighborIndex : Adjacency[SampleIndex])
			{
				if (!InSamples.IsValidIndex(NeighborIndex) || DistanceToInterior[NeighborIndex] != INDEX_NONE)
				{
					continue;
				}

				DistanceToInterior[NeighborIndex] = NextDistance;
				Queue.Add(NeighborIndex);
			}
		}

		double TotalBoundaryDepth = 0.0;
		for (int32 SampleIndex = 0; SampleIndex < NumSamples; ++SampleIndex)
		{
			if (!InSamples[SampleIndex].bIsBoundary)
			{
				continue;
			}

			int32 DepthHops = DistanceToInterior[SampleIndex];
			if (DepthHops == INDEX_NONE)
			{
				DepthHops = NumSamples;
			}

			TotalBoundaryDepth += static_cast<double>(DepthHops);
			BoundaryMaxDepthHops = FMath::Max(BoundaryMaxDepthHops, DepthHops);
			BoundaryDeepSampleCount += (DepthHops >= 2) ? 1 : 0;
		}

		BoundaryMeanDepthHops = TotalBoundaryDepth / static_cast<double>(BoundarySampleCount);
	}

	if (ContinentalSampleCount > 0)
	{
		TArray<uint8> Visited;
		Visited.Init(0, NumSamples);
		TArray<int32> Stack;
		Stack.Reserve(256);
		for (int32 SeedIndex = 0; SeedIndex < NumSamples; ++SeedIndex)
		{
			if (Visited[SeedIndex] != 0 || InSamples[SeedIndex].CrustType != ECrustType::Continental)
			{
				continue;
			}

			++ContinentalComponentCount;
			int32 ComponentSize = 0;
			Stack.Reset();
			Stack.Add(SeedIndex);
			Visited[SeedIndex] = 1;
			while (Stack.Num() > 0)
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				++ComponentSize;
				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					if (!InSamples.IsValidIndex(NeighborIndex) || Visited[NeighborIndex] != 0 || InSamples[NeighborIndex].CrustType != ECrustType::Continental)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			LargestContinentalComponentSize = FMath::Max(LargestContinentalComponentSize, ComponentSize);
		}
	}

	if (Plates.Num() <= 0)
	{
		return;
	}

	struct FPlateComponentInfo
	{
		int32 Size = 0;
		int32 PrevMatchCount = 0;
		int32 MinSampleIndex = TNumericLimits<int32>::Max();
	};

	auto SelectPrimaryComponentIndex = [](const TArray<FPlateComponentInfo>& Components) -> int32
	{
		int32 BestIndex = INDEX_NONE;
		for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
		{
			const FPlateComponentInfo& Candidate = Components[ComponentIndex];
			if (BestIndex == INDEX_NONE)
			{
				BestIndex = ComponentIndex;
				continue;
			}

			const FPlateComponentInfo& Best = Components[BestIndex];
			if (Candidate.PrevMatchCount > Best.PrevMatchCount ||
				(Candidate.PrevMatchCount == Best.PrevMatchCount && Candidate.Size > Best.Size) ||
				(Candidate.PrevMatchCount == Best.PrevMatchCount && Candidate.Size == Best.Size && Candidate.MinSampleIndex < Best.MinSampleIndex))
			{
				BestIndex = ComponentIndex;
			}
		}
		return BestIndex;
	};

	TArray<uint8> VisitedNonGap;
	VisitedNonGap.Init(0, NumSamples);
	TArray<int32> Stack;
	Stack.Reserve(256);
	TArray<TArray<FPlateComponentInfo>> ComponentsByPlate;
	ComponentsByPlate.SetNum(Plates.Num());
	for (int32 SeedIndex = 0; SeedIndex < NumSamples; ++SeedIndex)
	{
		if (VisitedNonGap[SeedIndex] != 0 || InSamples[SeedIndex].bGapDetected || !Plates.IsValidIndex(InSamples[SeedIndex].PlateId))
		{
			continue;
		}

		const int32 PlateId = InSamples[SeedIndex].PlateId;
		FPlateComponentInfo Component;
		Stack.Reset();
		Stack.Add(SeedIndex);
		VisitedNonGap[SeedIndex] = 1;
		while (Stack.Num() > 0)
		{
			const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
			++Component.Size;
			Component.PrevMatchCount += (InSamples[SampleIndex].PrevPlateId == PlateId) ? 1 : 0;
			Component.MinSampleIndex = FMath::Min(Component.MinSampleIndex, SampleIndex);
			for (const int32 NeighborIndex : Adjacency[SampleIndex])
			{
				if (!InSamples.IsValidIndex(NeighborIndex) || VisitedNonGap[NeighborIndex] != 0)
				{
					continue;
				}
				if (InSamples[NeighborIndex].bGapDetected || InSamples[NeighborIndex].PlateId != PlateId)
				{
					continue;
				}

				VisitedNonGap[NeighborIndex] = 1;
				Stack.Add(NeighborIndex);
			}
		}

		ComponentsByPlate[PlateId].Add(Component);
	}

	for (const TArray<FPlateComponentInfo>& Components : ComponentsByPlate)
	{
		if (Components.Num() <= 0)
		{
			continue;
		}

		MaxPlateComponentCount = FMath::Max(MaxPlateComponentCount, Components.Num());
		if (Components.Num() <= 1)
		{
			continue;
		}

		const int32 PrimaryComponentIndex = SelectPrimaryComponentIndex(Components);
		for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
		{
			if (ComponentIndex == PrimaryComponentIndex)
			{
				continue;
			}

			DetachedPlateFragmentSampleCount += Components[ComponentIndex].Size;
			LargestDetachedPlateFragmentSize = FMath::Max(LargestDetachedPlateFragmentSize, Components[ComponentIndex].Size);
		}
	}
}

void FTectonicPlanet::RebuildPlateMembershipFromSamples(const TArray<FCanonicalSample>& InSamples, const TArray<uint8>* DirtyPlateFlags)
{
	TArray<int32> PlateSampleCounts;
	PlateSampleCounts.Init(0, Plates.Num());
	TArray<int32> PlateNonGapSampleCounts;
	PlateNonGapSampleCounts.Init(0, Plates.Num());
	for (const FCanonicalSample& Sample : InSamples)
	{
		if (Plates.IsValidIndex(Sample.PlateId))
		{
			++PlateSampleCounts[Sample.PlateId];
			if (!Sample.bGapDetected)
			{
				++PlateNonGapSampleCounts[Sample.PlateId];
			}
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

	MinProtectedPlateSampleCount = TNumericLimits<int32>::Max();
	EmptyProtectedPlateCount = 0;
	for (const FPlate& Plate : Plates)
	{
		if (DirtyPlateFlags && (!DirtyPlateFlags->IsValidIndex(Plate.Id) || (*DirtyPlateFlags)[Plate.Id] == 0))
		{
			continue;
		}
		if (Plate.PersistencePolicy != EPlatePersistencePolicy::Protected)
		{
			continue;
		}

		const int32 PlateId = Plate.Id;
		const int32 NonGapSampleCount = PlateNonGapSampleCounts.IsValidIndex(PlateId) ? PlateNonGapSampleCounts[PlateId] : 0;
		MinProtectedPlateSampleCount = FMath::Min(MinProtectedPlateSampleCount, NonGapSampleCount);
		if (NonGapSampleCount == 0)
		{
			++EmptyProtectedPlateCount;
			UE_LOG(LogTemp, Warning, TEXT("Protected plate %d has no non-gap ownership after membership rebuild."), PlateId);
		}
	}

	if (MinProtectedPlateSampleCount == TNumericLimits<int32>::Max())
	{
		MinProtectedPlateSampleCount = 0;
	}
}

void FTectonicPlanet::UpdatePlateCanonicalCentersFromSamples(const TArray<FCanonicalSample>& InSamples, const TArray<uint8>* DirtyPlateFlags)
{
	ParallelFor(Plates.Num(), [this, &InSamples](int32 PlateIndex)
	{
		FPlate& Plate = Plates[PlateIndex];
		FVector MeanDirection = FVector::ZeroVector;
		for (const int32 SampleIndex : Plate.SampleIndices)
		{
			if (InSamples.IsValidIndex(SampleIndex))
			{
				MeanDirection += InSamples[SampleIndex].Position;
			}
		}

		if (!MeanDirection.IsNearlyZero())
		{
			Plate.CanonicalCenterDirection = MeanDirection.GetSafeNormal();
		}
		else if (!Plate.CanonicalCenterDirection.IsNearlyZero())
		{
			Plate.CanonicalCenterDirection = Plate.CanonicalCenterDirection.GetSafeNormal();
		}
		else
		{
			Plate.CanonicalCenterDirection = Plate.IdentityAnchorDirection.IsNearlyZero()
				? FVector::ForwardVector
				: Plate.IdentityAnchorDirection.GetSafeNormal();
		}
	}, EParallelForFlags::Unbalanced);
}

bool FTectonicPlanet::ResolveDominantConvergentInteractionForSample(
	const TArray<FCanonicalSample>& InSamples,
	const int32 SampleIndex,
	int32& OutOpposingPlateId,
	int32& OutRepresentativeNeighborIndex,
	float& OutConvergenceSpeedMmPerYear) const
{
	OutOpposingPlateId = INDEX_NONE;
	OutRepresentativeNeighborIndex = INDEX_NONE;
	OutConvergenceSpeedMmPerYear = 0.0f;

	if (!InSamples.IsValidIndex(SampleIndex) || !Adjacency.IsValidIndex(SampleIndex))
	{
		return false;
	}

	const FCanonicalSample& Sample = InSamples[SampleIndex];
	if (!Sample.bIsBoundary || !Sample.bOverlapDetected || Sample.BoundaryType != EBoundaryType::Convergent || !Plates.IsValidIndex(Sample.PlateId))
	{
		return false;
	}

	const FVector Position = Sample.Position.GetSafeNormal();
	const FVector BoundaryNormal = Sample.BoundaryNormal.GetSafeNormal();
	if (Position.IsNearlyZero() || BoundaryNormal.IsNearlyZero())
	{
		return false;
	}

	constexpr int32 MaxNeighborPlateBins = 16;
	constexpr double KmToMm = 1.0e6;
	int32 CandidatePlateIds[MaxNeighborPlateBins];
	int32 NumCandidatePlates = 0;
	for (int32 SlotIndex = 0; SlotIndex < MaxNeighborPlateBins; ++SlotIndex)
	{
		CandidatePlateIds[SlotIndex] = INDEX_NONE;
	}

	auto AddUniquePlateId = [&CandidatePlateIds, &NumCandidatePlates](const int32 PlateId)
	{
		if (PlateId == INDEX_NONE)
		{
			return;
		}

		for (int32 Index = 0; Index < NumCandidatePlates; ++Index)
		{
			if (CandidatePlateIds[Index] == PlateId)
			{
				return;
			}
		}

		if (NumCandidatePlates < MaxNeighborPlateBins)
		{
			CandidatePlateIds[NumCandidatePlates++] = PlateId;
		}
	};

	for (int32 OverlapIndex = 0; OverlapIndex < Sample.NumOverlapPlateIds; ++OverlapIndex)
	{
		const int32 CandidatePlateId = Sample.OverlapPlateIds[OverlapIndex];
		if (CandidatePlateId != Sample.PlateId && Plates.IsValidIndex(CandidatePlateId) && IsPlateActive(CandidatePlateId))
		{
			AddUniquePlateId(CandidatePlateId);
		}
	}

	if (NumCandidatePlates == 0)
	{
		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = InSamples[NeighborIndex].PlateId;
			if (NeighborPlateId != Sample.PlateId && Plates.IsValidIndex(NeighborPlateId) && IsPlateActive(NeighborPlateId))
			{
				AddUniquePlateId(NeighborPlateId);
			}
		}
	}

	int32 BestOpposingNeighborCount = -1;
	float BestConvergenceSpeed = -1.0f;
	double BestRepresentativeNeighborDistSq = TNumericLimits<double>::Max();
	for (int32 CandidateIndex = 0; CandidateIndex < NumCandidatePlates; ++CandidateIndex)
	{
		const int32 CandidatePlateId = CandidatePlateIds[CandidateIndex];
		if (!Plates.IsValidIndex(CandidatePlateId))
		{
			continue;
		}

		int32 DirectNeighborCount = 0;
		int32 RepresentativeNeighborIndex = INDEX_NONE;
		double RepresentativeNeighborDistSq = TNumericLimits<double>::Max();
		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InSamples.IsValidIndex(NeighborIndex) || InSamples[NeighborIndex].PlateId != CandidatePlateId)
			{
				continue;
			}

			++DirectNeighborCount;
			const double DistSq = FVector::DistSquared(Sample.Position, InSamples[NeighborIndex].Position);
			if (DistSq < RepresentativeNeighborDistSq)
			{
				RepresentativeNeighborDistSq = DistSq;
				RepresentativeNeighborIndex = NeighborIndex;
			}
		}

		const FVector RelativeVelocity = ComputeSurfaceVelocity(Sample.PlateId, Position) - ComputeSurfaceVelocity(CandidatePlateId, Position);
		const float CandidateConvergenceSpeed = static_cast<float>(
			FMath::Max(0.0, -static_cast<double>(FVector::DotProduct(RelativeVelocity, BoundaryNormal))) *
			PlanetRadius *
			KmToMm /
			TimestepDurationYears);

		const bool bBetterCandidate =
			(OutOpposingPlateId == INDEX_NONE) ||
			(DirectNeighborCount > BestOpposingNeighborCount) ||
			(DirectNeighborCount == BestOpposingNeighborCount && CandidateConvergenceSpeed > BestConvergenceSpeed + KINDA_SMALL_NUMBER) ||
			(DirectNeighborCount == BestOpposingNeighborCount && FMath::IsNearlyEqual(CandidateConvergenceSpeed, BestConvergenceSpeed, KINDA_SMALL_NUMBER) && CandidatePlateId < OutOpposingPlateId);
		if (bBetterCandidate)
		{
			OutOpposingPlateId = CandidatePlateId;
			BestOpposingNeighborCount = DirectNeighborCount;
			BestConvergenceSpeed = CandidateConvergenceSpeed;
			OutRepresentativeNeighborIndex = RepresentativeNeighborIndex;
			BestRepresentativeNeighborDistSq = RepresentativeNeighborDistSq;
		}
		else if (CandidatePlateId == OutOpposingPlateId && RepresentativeNeighborDistSq < BestRepresentativeNeighborDistSq)
		{
			OutRepresentativeNeighborIndex = RepresentativeNeighborIndex;
			BestRepresentativeNeighborDistSq = RepresentativeNeighborDistSq;
		}
	}

	OutConvergenceSpeedMmPerYear = FMath::Max(0.0f, BestConvergenceSpeed);
	return Plates.IsValidIndex(OutOpposingPlateId) && InSamples.IsValidIndex(OutRepresentativeNeighborIndex);
}

void FTectonicPlanet::RefreshSubductionMetricsFromCarriedSamples()
{
	int32 LocalFrontSampleCount = 0;
	int32 LocalAndeanSampleCount = 0;
	int32 LocalHimalayanSampleCount = 0;
	float LocalMaxSubductionDistanceKm = 0.0f;

	for (const FPlate& Plate : Plates)
	{
		for (const FCarriedSampleData& CarriedSample : Plate.CarriedSamples)
		{
			LocalFrontSampleCount += CarriedSample.bIsSubductionFront ? 1 : 0;
			LocalAndeanSampleCount += (CarriedSample.OrogenyType == EOrogenyType::Andean) ? 1 : 0;
			LocalHimalayanSampleCount += (CarriedSample.OrogenyType == EOrogenyType::Himalayan) ? 1 : 0;
			if (CarriedSample.SubductionRole != ESubductionRole::None && CarriedSample.SubductionDistanceKm >= 0.0f)
			{
				LocalMaxSubductionDistanceKm = FMath::Max(LocalMaxSubductionDistanceKm, CarriedSample.SubductionDistanceKm);
			}
		}
	}

	SubductionFrontSampleCount = LocalFrontSampleCount;
	AndeanSampleCount = LocalAndeanSampleCount;
	HimalayanSampleCount = LocalHimalayanSampleCount;
	MaxSubductionDistanceKm = LocalMaxSubductionDistanceKm;
}

void FTectonicPlanet::UpdateSubductionFieldsForSamples(TArray<FCanonicalSample>& InOutSamples)
{
	SubductionFrontSampleCount = 0;
	PendingCollisionSampleCount = 0;
	MaxSubductionDistanceKm = 0.0f;
	AndeanSampleCount = 0;
	HimalayanSampleCount = 0;

	for (FCanonicalSample& Sample : InOutSamples)
	{
		Sample.SubductionRole = ESubductionRole::None;
		Sample.SubductionOpposingPlateId = INDEX_NONE;
		Sample.SubductionDistanceKm = -1.0f;
		Sample.SubductionConvergenceSpeedMmPerYear = 0.0f;
		Sample.bIsSubductionFront = false;
		AndeanSampleCount += (Sample.OrogenyType == EOrogenyType::Andean) ? 1 : 0;
		HimalayanSampleCount += (Sample.OrogenyType == EOrogenyType::Himalayan) ? 1 : 0;
	}

	if (Plates.Num() == 0 || Adjacency.Num() != InOutSamples.Num())
	{
		return;
	}

	bool bAdjacencyEdgeCacheValid = (AdjacencyEdgeDistancesKm.Num() == Adjacency.Num());
	for (int32 SampleIndex = 0; bAdjacencyEdgeCacheValid && SampleIndex < Adjacency.Num(); ++SampleIndex)
	{
		bAdjacencyEdgeCacheValid = AdjacencyEdgeDistancesKm[SampleIndex].Num() == Adjacency[SampleIndex].Num();
	}
	if (!bAdjacencyEdgeCacheValid)
	{
		RebuildAdjacencyEdgeDistanceCache(InOutSamples);
	}
	if (AdjacencyEdgeDistancesKm.Num() != InOutSamples.Num())
	{
		return;
	}

	TArray<TArray<FInfluenceSeed>> OverridingSeedsByPlate;
	TArray<TArray<FInfluenceSeed>> SubductingSeedsByPlate;
	OverridingSeedsByPlate.SetNum(Plates.Num());
	SubductingSeedsByPlate.SetNum(Plates.Num());
	for (TArray<FInfluenceSeed>& PlateSeeds : OverridingSeedsByPlate)
	{
		PlateSeeds.Reserve(FMath::Max(1, BoundarySampleCount / FMath::Max(1, Plates.Num())));
	}
	for (TArray<FInfluenceSeed>& PlateSeeds : SubductingSeedsByPlate)
	{
		PlateSeeds.Reserve(FMath::Max(1, BoundarySampleCount / FMath::Max(1, Plates.Num())));
	}

	for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
	{
		FCanonicalSample& Sample = InOutSamples[SampleIndex];
		int32 BestOpposingPlateId = INDEX_NONE;
		int32 BestRepresentativeNeighborIndex = INDEX_NONE;
		float BestConvergenceSpeed = 0.0f;
		if (!ResolveDominantConvergentInteractionForSample(
			InOutSamples,
			SampleIndex,
			BestOpposingPlateId,
			BestRepresentativeNeighborIndex,
			BestConvergenceSpeed))
		{
			continue;
		}

		if (!Plates.IsValidIndex(BestOpposingPlateId) || !InOutSamples.IsValidIndex(BestRepresentativeNeighborIndex) || !Plates.IsValidIndex(Sample.PlateId))
		{
			continue;
		}

		const FCanonicalSample& OpposingSample = InOutSamples[BestRepresentativeNeighborIndex];
		if (Sample.CrustType == ECrustType::Continental && OpposingSample.CrustType == ECrustType::Continental)
		{
			++PendingCollisionSampleCount;
			continue;
		}

		bool bSampleSubducting = false;
		if (Sample.CrustType != OpposingSample.CrustType)
		{
			bSampleSubducting = (Sample.CrustType == ECrustType::Oceanic);
		}
		else if (Sample.CrustType == ECrustType::Oceanic)
		{
			if (Sample.Age > OpposingSample.Age + KINDA_SMALL_NUMBER)
			{
				bSampleSubducting = true;
			}
			else if (OpposingSample.Age > Sample.Age + KINDA_SMALL_NUMBER)
			{
				bSampleSubducting = false;
			}
			else
			{
				bSampleSubducting = Sample.PlateId > BestOpposingPlateId;
			}
		}
		else
		{
			++PendingCollisionSampleCount;
			continue;
		}

		FInfluenceSeed Seed;
		Seed.SampleIndex = SampleIndex;
		Seed.PlateId = Sample.PlateId;
		Seed.SecondaryId = BestOpposingPlateId;
		Seed.InfluenceWeight = FMath::Max(0.0f, BestConvergenceSpeed);
		if (bSampleSubducting)
		{
			SubductingSeedsByPlate[Sample.PlateId].Add(Seed);
		}
		else
		{
			OverridingSeedsByPlate[Sample.PlateId].Add(Seed);
		}
	}

	TArray<FInfluencePathCandidate> BestOverridingCandidates;
	TArray<FInfluencePathCandidate> BestSubductingCandidates;
	auto ResetCandidates = [&InOutSamples](TArray<FInfluencePathCandidate>& Candidates)
	{
		Candidates.SetNum(InOutSamples.Num());
		for (FInfluencePathCandidate& Candidate : Candidates)
		{
			Candidate = FInfluencePathCandidate{};
		}
	};
	ResetCandidates(BestOverridingCandidates);
	ResetCandidates(BestSubductingCandidates);

	auto CanTraverse = [](const FCanonicalSample& TraversedSample, const auto& SeedOrEntry)
	{
		return !TraversedSample.bGapDetected && TraversedSample.PlateId == SeedOrEntry.PlateId;
	};
	auto PropagateSeedsByPlate = [this, &InOutSamples, &CanTraverse](
		const TArray<TArray<FInfluenceSeed>>& SeedsByPlate,
		const float RadiusKm,
		TArray<FInfluencePathCandidate>& OutCandidates)
	{
		ParallelFor(Plates.Num(), [this, &InOutSamples, &SeedsByPlate, RadiusKm, &CanTraverse, &OutCandidates](int32 PlateIndex)
		{
			const TArray<FInfluenceSeed>& PlateSeeds = SeedsByPlate[PlateIndex];
			if (PlateSeeds.Num() <= 0)
			{
				return;
			}

			PropagateInfluenceSeedsIntoExistingCandidates(
				InOutSamples,
				Adjacency,
				AdjacencyEdgeDistancesKm,
				PlateSeeds,
				RadiusKm,
				CanTraverse,
				OutCandidates);
		}, EParallelForFlags::Unbalanced);
	};

	PropagateSeedsByPlate(OverridingSeedsByPlate, SubductionInfluenceRadiusKm, BestOverridingCandidates);
	PropagateSeedsByPlate(SubductingSeedsByPlate, SubductionTrenchRadiusKm, BestSubductingCandidates);

	for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
	{
		FCanonicalSample& Sample = InOutSamples[SampleIndex];
		const FInfluencePathCandidate& OverridingCandidate = BestOverridingCandidates[SampleIndex];
		const FInfluencePathCandidate& SubductingCandidate = BestSubductingCandidates[SampleIndex];

		const bool bUseOverriding = IsInfluenceCandidateBetterAcrossRoles(
			OverridingCandidate,
			SubductionInfluenceRadiusKm,
			SubductingCandidate,
			SubductionTrenchRadiusKm);
		const bool bUseSubducting = !bUseOverriding && SubductingCandidate.bValid;

		if (!bUseOverriding && !bUseSubducting)
		{
			continue;
		}

		const FInfluencePathCandidate& ChosenCandidate = bUseOverriding ? OverridingCandidate : SubductingCandidate;
		Sample.SubductionRole = bUseOverriding ? ESubductionRole::Overriding : ESubductionRole::Subducting;
		Sample.SubductionOpposingPlateId = ChosenCandidate.SecondaryId;
		Sample.SubductionDistanceKm = ChosenCandidate.DistanceKm;
		Sample.SubductionConvergenceSpeedMmPerYear = ChosenCandidate.InfluenceWeight;
		Sample.bIsSubductionFront = (ChosenCandidate.FrontSampleIndex == SampleIndex);
		SubductionFrontSampleCount += Sample.bIsSubductionFront ? 1 : 0;
		MaxSubductionDistanceKm = FMath::Max(MaxSubductionDistanceKm, ChosenCandidate.DistanceKm);
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
			Carried.SubductionRole = Sample.SubductionRole;
			Carried.SubductionOpposingPlateId = Sample.SubductionOpposingPlateId;
			Carried.SubductionDistanceKm = Sample.SubductionDistanceKm;
			Carried.SubductionConvergenceSpeedMmPerYear = Sample.SubductionConvergenceSpeedMmPerYear;
			Carried.bIsSubductionFront = Sample.bIsSubductionFront;
			Carried.CollisionDistanceKm = Sample.CollisionDistanceKm;
			Carried.CollisionConvergenceSpeedMmPerYear = Sample.CollisionConvergenceSpeedMmPerYear;
			Carried.bIsCollisionFront = Sample.bIsCollisionFront;
		}
	}, EParallelForFlags::Unbalanced);

	RefreshSubductionMetricsFromCarriedSamples();
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
		const int32 SoupTriangleCount = Plate.InteriorTriangles.Num() + Plate.BoundaryTriangles.Num();
		const bool bHasSoupTriangles = SoupTriangleCount > 0;
		const bool bMissingState = ExistingState.PlateId != Plate.Id;
		const bool bMissingSpatial = bHasSoupTriangles && (!ExistingState.Adapter || !ExistingState.BVH);
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

		const int32 BaseSoupTriangleCount = Plate.InteriorTriangles.Num() + Plate.BoundaryTriangles.Num();
		const int32 EstimatedSoupTriangleCount = Plate.InteriorTriangles.Num() + (2 * Plate.BoundaryTriangles.Num());
		const int32 EstimatedSoupVertexCount = (Plate.InteriorTriangles.Num() * 3) + (Plate.BoundaryTriangles.Num() * 6);
		PlateState.SoupData.GlobalTriangleIndices.Reserve(EstimatedSoupTriangleCount);
		PlateState.SoupData.LocalTriangles.Reserve(EstimatedSoupTriangleCount);
		PlateState.SoupData.CanonicalToLocalVertex.Reserve(BaseSoupTriangleCount * 2);
		PlateState.SoupData.LocalToCanonicalVertex.Reserve(EstimatedSoupVertexCount);
		PlateState.SoupData.RotatedVertices.Reserve(EstimatedSoupVertexCount);

		auto FindOrAddCanonicalVertex = [&Samples, &PlateState](const int32 CanonicalSampleIndex) -> int32
		{
			const int32* ExistingLocalVertex = PlateState.SoupData.CanonicalToLocalVertex.Find(CanonicalSampleIndex);
			if (ExistingLocalVertex)
			{
				return *ExistingLocalVertex;
			}

			checkSlow(Samples.IsValidIndex(CanonicalSampleIndex));
			const int32 NewLocalVertex = PlateState.SoupData.RotatedVertices.Num();
			PlateState.SoupData.CanonicalToLocalVertex.Add(CanonicalSampleIndex, NewLocalVertex);
			PlateState.SoupData.LocalToCanonicalVertex.Add(CanonicalSampleIndex);
			const FVector& CanonicalPosition = Samples[CanonicalSampleIndex].Position;
			PlateState.SoupData.RotatedVertices.Add(FVector3d(CanonicalPosition.X, CanonicalPosition.Y, CanonicalPosition.Z));
			return NewLocalVertex;
		};

		auto AddSyntheticVertex = [&PlateState](const FVector3d& CanonicalPosition) -> int32
		{
			const FVector3d NormalizedPosition = CanonicalPosition.GetSafeNormal();
			if (NormalizedPosition.IsNearlyZero())
			{
				return INDEX_NONE;
			}

			const int32 NewLocalVertex = PlateState.SoupData.RotatedVertices.Num();
			PlateState.SoupData.LocalToCanonicalVertex.Add(INDEX_NONE);
			PlateState.SoupData.RotatedVertices.Add(NormalizedPosition);
			return NewLocalVertex;
		};

		auto AppendLocalTriangle = [&PlateState](const int32 GlobalTriangleIndex, const int32 LocalVertex0, const int32 LocalVertex1, const int32 LocalVertex2)
		{
			if (LocalVertex0 == INDEX_NONE || LocalVertex1 == INDEX_NONE || LocalVertex2 == INDEX_NONE)
			{
				return;
			}
			if (LocalVertex0 == LocalVertex1 || LocalVertex1 == LocalVertex2 || LocalVertex0 == LocalVertex2)
			{
				return;
			}

			PlateState.SoupData.GlobalTriangleIndices.Add(GlobalTriangleIndex);
			PlateState.SoupData.LocalTriangles.Add(UE::Geometry::FIndex3i(LocalVertex0, LocalVertex1, LocalVertex2));
		};

		auto AppendWholeTriangleToSoup = [this, &FindOrAddCanonicalVertex, &AppendLocalTriangle](const int32 TriangleIndex)
		{
			checkSlow(Triangles.IsValidIndex(TriangleIndex));
			const FDelaunayTriangle& GlobalTriangle = Triangles[TriangleIndex];
			const int32 LocalVertex0 = FindOrAddCanonicalVertex(GlobalTriangle.V[0]);
			const int32 LocalVertex1 = FindOrAddCanonicalVertex(GlobalTriangle.V[1]);
			const int32 LocalVertex2 = FindOrAddCanonicalVertex(GlobalTriangle.V[2]);
			AppendLocalTriangle(TriangleIndex, LocalVertex0, LocalVertex1, LocalVertex2);
		};

		auto AppendBoundaryTriangleRegionToSoup = [this, &Samples, &Plate, &FindOrAddCanonicalVertex, &AddSyntheticVertex, &AppendLocalTriangle](const int32 TriangleIndex)
		{
			checkSlow(Triangles.IsValidIndex(TriangleIndex));
			const FDelaunayTriangle& GlobalTriangle = Triangles[TriangleIndex];
			const int32 VertexSampleIndices[3] = { GlobalTriangle.V[0], GlobalTriangle.V[1], GlobalTriangle.V[2] };
			const int32 VertexPlateIds[3] = {
				Samples[VertexSampleIndices[0]].PlateId,
				Samples[VertexSampleIndices[1]].PlateId,
				Samples[VertexSampleIndices[2]].PlateId };
			const FVector3d CanonicalPositions[3] = {
				FVector3d(Samples[VertexSampleIndices[0]].Position.X, Samples[VertexSampleIndices[0]].Position.Y, Samples[VertexSampleIndices[0]].Position.Z),
				FVector3d(Samples[VertexSampleIndices[1]].Position.X, Samples[VertexSampleIndices[1]].Position.Y, Samples[VertexSampleIndices[1]].Position.Z),
				FVector3d(Samples[VertexSampleIndices[2]].Position.X, Samples[VertexSampleIndices[2]].Position.Y, Samples[VertexSampleIndices[2]].Position.Z) };
			const int32 LocalVertices[3] = {
				FindOrAddCanonicalVertex(VertexSampleIndices[0]),
				FindOrAddCanonicalVertex(VertexSampleIndices[1]),
				FindOrAddCanonicalVertex(VertexSampleIndices[2]) };

			int32 MatchingCorners[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };
			int32 NonMatchingCorners[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };
			int32 MatchingCount = 0;
			int32 NonMatchingCount = 0;
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				if (VertexPlateIds[Corner] == Plate.Id)
				{
					MatchingCorners[MatchingCount++] = Corner;
				}
				else
				{
					NonMatchingCorners[NonMatchingCount++] = Corner;
				}
			}

			if (MatchingCount == 3)
			{
				AppendLocalTriangle(TriangleIndex, LocalVertices[0], LocalVertices[1], LocalVertices[2]);
				return;
			}
			if (MatchingCount == 0)
			{
				return;
			}

			auto AddMidpointVertex = [&CanonicalPositions, &AddSyntheticVertex](const int32 CornerA, const int32 CornerB) -> int32
			{
				return AddSyntheticVertex(CanonicalPositions[CornerA] + CanonicalPositions[CornerB]);
			};

			if (MatchingCount == 2)
			{
				const int32 OwnedCornerA = MatchingCorners[0];
				const int32 OwnedCornerB = MatchingCorners[1];
				const int32 OtherCorner = NonMatchingCorners[0];
				const int32 MidA = AddMidpointVertex(OwnedCornerA, OtherCorner);
				const int32 MidB = AddMidpointVertex(OwnedCornerB, OtherCorner);
				AppendLocalTriangle(TriangleIndex, LocalVertices[OwnedCornerA], LocalVertices[OwnedCornerB], MidB);
				AppendLocalTriangle(TriangleIndex, LocalVertices[OwnedCornerA], MidB, MidA);
				return;
			}

			const int32 OwnedCorner = MatchingCorners[0];
			const int32 OtherCornerA = NonMatchingCorners[0];
			const int32 OtherCornerB = NonMatchingCorners[1];
			const int32 MidA = AddMidpointVertex(OwnedCorner, OtherCornerA);
			const int32 MidB = AddMidpointVertex(OwnedCorner, OtherCornerB);
			if (VertexPlateIds[OtherCornerA] == VertexPlateIds[OtherCornerB])
			{
				AppendLocalTriangle(TriangleIndex, LocalVertices[OwnedCorner], MidA, MidB);
				return;
			}

			const int32 CentroidVertex = AddSyntheticVertex(CanonicalPositions[0] + CanonicalPositions[1] + CanonicalPositions[2]);
			AppendLocalTriangle(TriangleIndex, LocalVertices[OwnedCorner], MidA, CentroidVertex);
			AppendLocalTriangle(TriangleIndex, LocalVertices[OwnedCorner], CentroidVertex, MidB);
		};

		for (const int32 TriangleIndex : Plate.InteriorTriangles)
		{
			AppendWholeTriangleToSoup(TriangleIndex);
		}
		for (const int32 TriangleIndex : Plate.BoundaryTriangles)
		{
			AppendBoundaryTriangleRegionToSoup(TriangleIndex);
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
	int32 BestPlateId = INDEX_NONE;
	for (const FSpatialQueryData::FPlateSpatialState& PlateState : SpatialQueryData->PlateStates)
	{
		if (!IsPlateActive(PlateState.PlateId))
		{
			continue;
		}

		const FVector Center = PlateState.bHasValidCap
			? PlateState.CapCenterDirection
			: Plates[PlateState.PlateId].CumulativeRotation.RotateVector(Plates[PlateState.PlateId].CanonicalCenterDirection).GetSafeNormal();
		const double Dot = FVector::DotProduct(Direction, Center);
		if (Dot > BestDot)
		{
			BestDot = Dot;
			BestPlateId = PlateState.PlateId;
		}
	}

	return BestPlateId == INDEX_NONE ? 0 : BestPlateId;
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
		if (!IsPlateActive(PlateState.PlateId))
		{
			continue;
		}

		const FVector Center = PlateState.bHasValidCap
			? PlateState.CapCenterDirection
			: Plates[PlateState.PlateId].CumulativeRotation.RotateVector(Plates[PlateState.PlateId].CanonicalCenterDirection).GetSafeNormal();
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

void FTectonicPlanet::RefreshTerraneMetrics()
{
	TrackedTerraneCount = TerraneRecords.Num();
	ActiveTerraneCount = 0;
	MergedTerraneCount = 0;
	CollisionEventCount = CollisionEvents.Num();
	for (const FTerraneRecord& Record : TerraneRecords)
	{
		ActiveTerraneCount += Record.bActive ? 1 : 0;
		MergedTerraneCount += (Record.MergedIntoTerraneId != INDEX_NONE) ? 1 : 0;
	}
}

void FTectonicPlanet::DetectTerranesForSamples(TArray<FCanonicalSample>& InOutSamples)
{
	const int32 CurrentReconcileOrdinal = ReconcileCount + 1;
	for (FCanonicalSample& Sample : InOutSamples)
	{
		if (Sample.CrustType != ECrustType::Continental || Sample.bGapDetected || !Plates.IsValidIndex(Sample.PlateId))
		{
			Sample.TerraneId = INDEX_NONE;
		}
	}

	struct FDetectedTerraneComponent
	{
		int32 PlateId = INDEX_NONE;
		TArray<int32> Members;
		FVector CentroidDirection = FVector::ZeroVector;
		double AreaKm2 = 0.0;
		int32 AnchorSampleIndex = INDEX_NONE;
		int32 AssignedTerraneId = INDEX_NONE;
		TMap<int32, int32> PriorTerraneSampleCounts;
		TArray<int32> AnchorTerraneIds;
	};

	TMultiMap<int32, int32> PriorAnchorTerraneIdsBySample;
	for (const FTerraneRecord& Record : TerraneRecords)
	{
		if (Record.bActive && Record.AnchorSampleIndex != INDEX_NONE)
		{
			PriorAnchorTerraneIdsBySample.Add(Record.AnchorSampleIndex, Record.TerraneId);
		}
	}

	TArray<FDetectedTerraneComponent> Components;
	if (Adjacency.Num() == InOutSamples.Num())
	{
		TArray<uint8> Visited;
		Visited.Init(0, InOutSamples.Num());
		TArray<int32> Stack;
		Stack.Reserve(256);
		for (int32 SeedIndex = 0; SeedIndex < InOutSamples.Num(); ++SeedIndex)
		{
			const FCanonicalSample& SeedSample = InOutSamples[SeedIndex];
			if (Visited[SeedIndex] != 0 ||
				SeedSample.CrustType != ECrustType::Continental ||
				SeedSample.bGapDetected ||
				!Plates.IsValidIndex(SeedSample.PlateId))
			{
				continue;
			}

			FDetectedTerraneComponent& Component = Components.Emplace_GetRef();
			Component.PlateId = SeedSample.PlateId;
			FVector MeanDirection = FVector::ZeroVector;

			Visited[SeedIndex] = 1;
			Stack.Reset();
			Stack.Add(SeedIndex);
			while (Stack.Num() > 0)
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				FCanonicalSample& Sample = InOutSamples[SampleIndex];
				Component.Members.Add(SampleIndex);
				MeanDirection += Sample.Position;
				if (Sample.TerraneId != INDEX_NONE)
				{
					Component.PriorTerraneSampleCounts.FindOrAdd(Sample.TerraneId) += 1;
				}

				TArray<int32> AnchoredTerranes;
				PriorAnchorTerraneIdsBySample.MultiFind(SampleIndex, AnchoredTerranes);
				for (const int32 AnchoredTerraneId : AnchoredTerranes)
				{
					Component.AnchorTerraneIds.AddUnique(AnchoredTerraneId);
				}

				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					if (!InOutSamples.IsValidIndex(NeighborIndex) || Visited[NeighborIndex] != 0)
					{
						continue;
					}

					const FCanonicalSample& Neighbor = InOutSamples[NeighborIndex];
					if (Neighbor.CrustType != ECrustType::Continental ||
						Neighbor.bGapDetected ||
						Neighbor.PlateId != Component.PlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			Component.CentroidDirection = MeanDirection.GetSafeNormal();
			Component.AreaKm2 = static_cast<double>(Component.Members.Num()) * AverageCellAreaKm2;
			double BestAnchorDot = -TNumericLimits<double>::Max();
			for (const int32 SampleIndex : Component.Members)
			{
				const double CandidateDot = FVector::DotProduct(InOutSamples[SampleIndex].Position, Component.CentroidDirection);
				if (Component.AnchorSampleIndex == INDEX_NONE ||
					CandidateDot > BestAnchorDot + UE_DOUBLE_SMALL_NUMBER ||
					(FMath::IsNearlyEqual(CandidateDot, BestAnchorDot, UE_DOUBLE_SMALL_NUMBER) && SampleIndex < Component.AnchorSampleIndex))
				{
					Component.AnchorSampleIndex = SampleIndex;
					BestAnchorDot = CandidateDot;
				}
			}
		}
	}

	TSet<int32> MatchedTerraneIds;
	TMap<int32, int32> MergedIntoTerraneById;
	auto GetPriorTerranePriority = [this](const FDetectedTerraneComponent& Component, const int32 TerraneId) -> int32
	{
		const int32* PriorContribution = Component.PriorTerraneSampleCounts.Find(TerraneId);
		if (PriorContribution)
		{
			return *PriorContribution;
		}

		if (const FTerraneRecord* Record = FindTerraneRecordById(TerraneId))
		{
			return Record->SampleCount;
		}

		return 0;
	};

	for (FDetectedTerraneComponent& Component : Components)
	{
		if (Component.AnchorTerraneIds.Num() <= 0)
		{
			continue;
		}

		int32 WinningTerraneId = INDEX_NONE;
		int32 WinningPriority = -1;
		for (const int32 CandidateTerraneId : Component.AnchorTerraneIds)
		{
			const int32 CandidatePriority = GetPriorTerranePriority(Component, CandidateTerraneId);
			if (WinningTerraneId == INDEX_NONE ||
				CandidatePriority > WinningPriority ||
				(CandidatePriority == WinningPriority && CandidateTerraneId < WinningTerraneId))
			{
				WinningTerraneId = CandidateTerraneId;
				WinningPriority = CandidatePriority;
			}
		}

		Component.AssignedTerraneId = WinningTerraneId;
		for (const int32 CandidateTerraneId : Component.AnchorTerraneIds)
		{
			MatchedTerraneIds.Add(CandidateTerraneId);
			if (CandidateTerraneId != WinningTerraneId)
			{
				MergedIntoTerraneById.Add(CandidateTerraneId, WinningTerraneId);
			}
		}
	}

	auto FindBestFallbackTerraneId = [this, &MatchedTerraneIds, &InOutSamples](const FDetectedTerraneComponent& Component, const bool bRestrictToSamePlate) -> int32
	{
		int32 BestTerraneId = INDEX_NONE;
		double BestMatchDot = -TNumericLimits<double>::Max();
		int32 BestSampleCount = -1;
		for (const FTerraneRecord& Record : TerraneRecords)
		{
			if (!Record.bActive || Record.MergedIntoTerraneId != INDEX_NONE || MatchedTerraneIds.Contains(Record.TerraneId))
			{
				continue;
			}
			if (bRestrictToSamePlate && Record.PlateId != Component.PlateId)
			{
				continue;
			}

			double CandidateBestDot = -TNumericLimits<double>::Max();
			for (const int32 SampleIndex : Component.Members)
			{
				CandidateBestDot = FMath::Max(
					CandidateBestDot,
					static_cast<double>(FVector::DotProduct(InOutSamples[SampleIndex].Position, Record.CentroidDirection)));
			}

			if (BestTerraneId == INDEX_NONE ||
				CandidateBestDot > BestMatchDot + UE_DOUBLE_SMALL_NUMBER ||
				(FMath::IsNearlyEqual(CandidateBestDot, BestMatchDot, UE_DOUBLE_SMALL_NUMBER) && Record.SampleCount > BestSampleCount) ||
				(FMath::IsNearlyEqual(CandidateBestDot, BestMatchDot, UE_DOUBLE_SMALL_NUMBER) && Record.SampleCount == BestSampleCount && Record.TerraneId < BestTerraneId))
			{
				BestTerraneId = Record.TerraneId;
				BestMatchDot = CandidateBestDot;
				BestSampleCount = Record.SampleCount;
			}
		}
		return BestTerraneId;
	};

	for (FDetectedTerraneComponent& Component : Components)
	{
		if (Component.AssignedTerraneId != INDEX_NONE)
		{
			continue;
		}

		int32 FallbackTerraneId = FindBestFallbackTerraneId(Component, true);
		if (FallbackTerraneId == INDEX_NONE)
		{
			FallbackTerraneId = FindBestFallbackTerraneId(Component, false);
		}
		if (FallbackTerraneId != INDEX_NONE)
		{
			Component.AssignedTerraneId = FallbackTerraneId;
			MatchedTerraneIds.Add(FallbackTerraneId);
		}
	}

	for (FDetectedTerraneComponent& Component : Components)
	{
		if (Component.AssignedTerraneId == INDEX_NONE)
		{
			Component.AssignedTerraneId = NextTerraneId++;
		}

		for (const int32 SampleIndex : Component.Members)
		{
			InOutSamples[SampleIndex].TerraneId = Component.AssignedTerraneId;
		}
	}

	for (FTerraneRecord& Record : TerraneRecords)
	{
		Record.bActive = false;
	}

	for (const TPair<int32, int32>& MergeEntry : MergedIntoTerraneById)
	{
		if (FTerraneRecord* MergedRecord = FindTerraneRecordById(MergeEntry.Key))
		{
			MergedRecord->bActive = false;
			MergedRecord->MergedIntoTerraneId = MergeEntry.Value;
			MergedRecord->LastSeenReconcile = CurrentReconcileOrdinal;
		}
	}

	for (const FDetectedTerraneComponent& Component : Components)
	{
		FTerraneRecord* Record = FindTerraneRecordById(Component.AssignedTerraneId);
		if (!Record)
		{
			Record = &TerraneRecords.Emplace_GetRef();
			Record->TerraneId = Component.AssignedTerraneId;
		}

		Record->PlateId = Component.PlateId;
		Record->AnchorSampleIndex = Component.AnchorSampleIndex;
		Record->CentroidDirection = Component.CentroidDirection;
		Record->AreaKm2 = Component.AreaKm2;
		Record->SampleCount = Component.Members.Num();
		Record->MergedIntoTerraneId = INDEX_NONE;
		Record->LastSeenReconcile = CurrentReconcileOrdinal;
		Record->bActive = true;
	}

	RefreshTerraneMetrics();
}

bool FTectonicPlanet::ApplyContinentalCollisionEventsForSamples(TArray<FCanonicalSample>& InOutSamples, TArray<uint8>* OutDirtyPlateFlags)
{
	if (OutDirtyPlateFlags)
	{
		OutDirtyPlateFlags->Init(0, Plates.Num());
	}
	if (Adjacency.Num() != InOutSamples.Num() || TerraneRecords.Num() <= 0)
	{
		return false;
	}
	if (AdjacencyEdgeDistancesKm.Num() != InOutSamples.Num())
	{
		RebuildAdjacencyEdgeDistanceCache(InOutSamples);
	}
	if (AdjacencyEdgeDistancesKm.Num() != InOutSamples.Num())
	{
		return false;
	}

	struct FCollisionPairCandidate
	{
		int32 TerraneIdA = INDEX_NONE;
		int32 TerraneIdB = INDEX_NONE;
		TArray<int32> FrontSampleIndices;
		int32 ContactSampleCount = 0;
		double ConvergenceSpeedSum = 0.0;
	};

	TArray<FCollisionPairCandidate> PairCandidates;
	TMap<uint64, int32> PairIndexByKey;
	for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = InOutSamples[SampleIndex];
		if (Sample.CrustType != ECrustType::Continental || Sample.TerraneId == INDEX_NONE)
		{
			continue;
		}

		int32 OpposingPlateId = INDEX_NONE;
		int32 OpposingSampleIndex = INDEX_NONE;
		float ConvergenceSpeedMmPerYear = 0.0f;
		if (!ResolveDominantConvergentInteractionForSample(
			InOutSamples,
			SampleIndex,
			OpposingPlateId,
			OpposingSampleIndex,
			ConvergenceSpeedMmPerYear))
		{
			continue;
		}

		if (!InOutSamples.IsValidIndex(OpposingSampleIndex))
		{
			continue;
		}

		const FCanonicalSample& OpposingSample = InOutSamples[OpposingSampleIndex];
		if (OpposingSample.CrustType != ECrustType::Continental ||
			OpposingSample.TerraneId == INDEX_NONE ||
			OpposingSample.TerraneId == Sample.TerraneId)
		{
			continue;
		}

		const uint64 PairKey = MakeCollisionHistoryKey(Sample.TerraneId, OpposingSample.TerraneId);
		if (CollisionHistoryKeys.Contains(PairKey))
		{
			continue;
		}

		int32* ExistingPairIndex = PairIndexByKey.Find(PairKey);
		if (!ExistingPairIndex)
		{
			FCollisionPairCandidate& Pair = PairCandidates.Emplace_GetRef();
			Pair.TerraneIdA = FMath::Min(Sample.TerraneId, OpposingSample.TerraneId);
			Pair.TerraneIdB = FMath::Max(Sample.TerraneId, OpposingSample.TerraneId);
			Pair.FrontSampleIndices.Reserve(16);
			PairIndexByKey.Add(PairKey, PairCandidates.Num() - 1);
			ExistingPairIndex = PairIndexByKey.Find(PairKey);
		}

		FCollisionPairCandidate& Pair = PairCandidates[*ExistingPairIndex];
		Pair.ContactSampleCount += 1;
		Pair.ConvergenceSpeedSum += static_cast<double>(ConvergenceSpeedMmPerYear);
		Pair.FrontSampleIndices.AddUnique(SampleIndex);
		Pair.FrontSampleIndices.AddUnique(OpposingSampleIndex);
	}

	PairCandidates.Sort([this](const FCollisionPairCandidate& A, const FCollisionPairCandidate& B)
	{
		if (A.ContactSampleCount != B.ContactSampleCount)
		{
			return A.ContactSampleCount > B.ContactSampleCount;
		}

		const double MeanSpeedA = (A.ContactSampleCount > 0) ? (A.ConvergenceSpeedSum / static_cast<double>(A.ContactSampleCount)) : 0.0;
		const double MeanSpeedB = (B.ContactSampleCount > 0) ? (B.ConvergenceSpeedSum / static_cast<double>(B.ContactSampleCount)) : 0.0;
		if (!FMath::IsNearlyEqual(MeanSpeedA, MeanSpeedB, UE_DOUBLE_SMALL_NUMBER))
		{
			return MeanSpeedA > MeanSpeedB;
		}

		if (A.TerraneIdA != B.TerraneIdA)
		{
			return A.TerraneIdA < B.TerraneIdA;
		}
		return A.TerraneIdB < B.TerraneIdB;
	});

	TSet<int32> TerranesUsedThisReconcile;
	bool bAppliedAnyCollision = false;
	for (const FCollisionPairCandidate& Pair : PairCandidates)
	{
		if (TerranesUsedThisReconcile.Contains(Pair.TerraneIdA) || TerranesUsedThisReconcile.Contains(Pair.TerraneIdB))
		{
			continue;
		}

		const FTerraneRecord* TerraneRecordA = FindTerraneRecordById(Pair.TerraneIdA);
		const FTerraneRecord* TerraneRecordB = FindTerraneRecordById(Pair.TerraneIdB);
		if (!TerraneRecordA || !TerraneRecordB || !TerraneRecordA->bActive || !TerraneRecordB->bActive)
		{
			continue;
		}

		const bool bATerraneIsSmaller =
			(TerraneRecordA->AreaKm2 < TerraneRecordB->AreaKm2 - UE_DOUBLE_SMALL_NUMBER) ||
			(FMath::IsNearlyEqual(TerraneRecordA->AreaKm2, TerraneRecordB->AreaKm2, UE_DOUBLE_SMALL_NUMBER) && TerraneRecordA->TerraneId < TerraneRecordB->TerraneId);
		const FTerraneRecord& DonorTerrane = bATerraneIsSmaller ? *TerraneRecordA : *TerraneRecordB;
		const FTerraneRecord& ReceiverTerrane = bATerraneIsSmaller ? *TerraneRecordB : *TerraneRecordA;
		if (OutDirtyPlateFlags)
		{
			if (OutDirtyPlateFlags->IsValidIndex(DonorTerrane.PlateId)) { (*OutDirtyPlateFlags)[DonorTerrane.PlateId] = 1; }
			if (OutDirtyPlateFlags->IsValidIndex(ReceiverTerrane.PlateId)) { (*OutDirtyPlateFlags)[ReceiverTerrane.PlateId] = 1; }
		}
		if (!Plates.IsValidIndex(ReceiverTerrane.PlateId))
		{
			continue;
		}

		TArray<int32> DonorSampleIndices;
		DonorSampleIndices.Reserve(DonorTerrane.SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
		{
			if (InOutSamples[SampleIndex].TerraneId == DonorTerrane.TerraneId)
			{
				DonorSampleIndices.Add(SampleIndex);
			}
		}
		if (DonorSampleIndices.Num() <= 0)
		{
			continue;
		}

		const float MeanConvergenceSpeedMmPerYear = (Pair.ContactSampleCount > 0)
			? static_cast<float>(Pair.ConvergenceSpeedSum / static_cast<double>(Pair.ContactSampleCount))
			: 0.0f;
		const double CollisionAreaKm2 = static_cast<double>(DonorSampleIndices.Num()) * AverageCellAreaKm2;
		const double AreaNormalizationKm2 = FMath::Max(InitialMeanPlateAreaKm2, AverageCellAreaKm2);
		const double AreaScale = FMath::Clamp(CollisionAreaKm2 / AreaNormalizationKm2, 0.0, 1.0);
		const double SpeedScale = FMath::Clamp(
			static_cast<double>(MeanConvergenceSpeedMmPerYear) / static_cast<double>(MaxSubductionSpeedMmPerYear),
			0.0,
			1.0);
		const float CollisionRadiusKm = static_cast<float>(FMath::Clamp(
			4200.0 * FMath::Sqrt(SpeedScale * AreaScale),
			0.0,
			4200.0));

		for (const int32 SampleIndex : DonorSampleIndices)
		{
			if (OutDirtyPlateFlags && Adjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					if (InOutSamples.IsValidIndex(NeighborIndex) && OutDirtyPlateFlags->IsValidIndex(InOutSamples[NeighborIndex].PlateId))
					{
						(*OutDirtyPlateFlags)[InOutSamples[NeighborIndex].PlateId] = 1;
					}
				}
			}
			InOutSamples[SampleIndex].PlateId = ReceiverTerrane.PlateId;
		}

		TArray<FInfluenceSeed> CollisionSeeds;
		CollisionSeeds.Reserve(Pair.FrontSampleIndices.Num());
		for (const int32 FrontSampleIndex : Pair.FrontSampleIndices)
		{
			if (!InOutSamples.IsValidIndex(FrontSampleIndex))
			{
				continue;
			}

			FInfluenceSeed& Seed = CollisionSeeds.Emplace_GetRef();
			Seed.SampleIndex = FrontSampleIndex;
			Seed.PlateId = ReceiverTerrane.PlateId;
			Seed.SecondaryId = DonorTerrane.TerraneId;
			Seed.InfluenceWeight = MeanConvergenceSpeedMmPerYear;
		}

		TArray<FInfluencePathCandidate> CollisionCandidates;
		auto CanTraverse = [ReceiverPlateId = ReceiverTerrane.PlateId](const FCanonicalSample& TraversedSample, const auto& SeedOrEntry)
		{
			return !TraversedSample.bGapDetected &&
				TraversedSample.PlateId == ReceiverPlateId &&
				TraversedSample.CrustType == ECrustType::Continental &&
				SeedOrEntry.PlateId == ReceiverPlateId;
		};
		PropagateInfluenceSeeds(
			InOutSamples,
			Adjacency,
			AdjacencyEdgeDistancesKm,
			CollisionSeeds,
			CollisionRadiusKm,
			CanTraverse,
			CollisionCandidates);

		FVector MeanCompressionDirection = FVector::ZeroVector;
		for (const int32 FrontSampleIndex : Pair.FrontSampleIndices)
		{
			if (InOutSamples.IsValidIndex(FrontSampleIndex))
			{
				MeanCompressionDirection += InOutSamples[FrontSampleIndex].BoundaryNormal;
			}
		}
		if (MeanCompressionDirection.IsNearlyZero())
		{
			MeanCompressionDirection = (ReceiverTerrane.CentroidDirection - DonorTerrane.CentroidDirection).GetSafeNormal();
		}
		MeanCompressionDirection = MeanCompressionDirection.GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
		{
			const FInfluencePathCandidate& Candidate = CollisionCandidates[SampleIndex];
			if (!Candidate.bValid || Candidate.DistanceKm < 0.0f || CollisionRadiusKm <= UE_SMALL_NUMBER)
			{
				continue;
			}

			const double RadiusRatio = static_cast<double>(Candidate.DistanceKm) / static_cast<double>(CollisionRadiusKm);
			if (RadiusRatio >= 1.0)
			{
				continue;
			}

			const double Falloff = FMath::Square(1.0 - (RadiusRatio * RadiusRatio));
			const float ElevationSurgeKm = static_cast<float>(1.3e-5 * CollisionAreaKm2 * Falloff);
			if (ElevationSurgeKm <= KINDA_SMALL_NUMBER)
			{
				continue;
			}

			FCanonicalSample& Sample = InOutSamples[SampleIndex];
			Sample.Elevation += ElevationSurgeKm;
			Sample.CollisionDistanceKm = Candidate.DistanceKm;
			Sample.CollisionConvergenceSpeedMmPerYear = MeanConvergenceSpeedMmPerYear;
			Sample.bIsCollisionFront = (Candidate.FrontSampleIndex == SampleIndex);
			Sample.OrogenyType = EOrogenyType::Himalayan;
			Sample.OrogenyAge = 0.0f;

			FVector FoldDirection = FVector::CrossProduct(Sample.Position.GetSafeNormal(), MeanCompressionDirection);
			FoldDirection = FoldDirection - FVector::DotProduct(FoldDirection, Sample.Position) * Sample.Position;
			if (!FoldDirection.IsNearlyZero())
			{
				Sample.FoldDirection = FoldDirection.GetSafeNormal();
			}
		}

		const uint64 PairKey = MakeCollisionHistoryKey(Pair.TerraneIdA, Pair.TerraneIdB);
		CollisionHistoryKeys.Add(PairKey);
		FCollisionEventRecord& EventRecord = CollisionEvents.Emplace_GetRef();
		EventRecord.DonorTerraneId = DonorTerrane.TerraneId;
		EventRecord.ReceiverTerraneId = ReceiverTerrane.TerraneId;
		EventRecord.ReceiverPlateId = ReceiverTerrane.PlateId;
		EventRecord.ReconcileOrdinal = ReconcileCount + 1;
		EventRecord.ContactSampleCount = Pair.ContactSampleCount;
		EventRecord.MeanConvergenceSpeedMmPerYear = MeanConvergenceSpeedMmPerYear;
		TerranesUsedThisReconcile.Add(Pair.TerraneIdA);
		TerranesUsedThisReconcile.Add(Pair.TerraneIdB);
		bAppliedAnyCollision = true;
	}

	CollisionEventCount = CollisionEvents.Num();
	return bAppliedAnyCollision;
}

void FTectonicPlanet::RefreshCanonicalStateAfterCollision(
	const TArray<FPhase2SampleState>& Phase2States,
	TArray<FCanonicalSample>& InOutSamples,
	const TArray<uint8>* DirtyPlateFlags)
{
	const int32 DirtyPlateCount = (DirtyPlateFlags && DirtyPlateFlags->Num() == Plates.Num()) ? CountSetFlags(*DirtyPlateFlags) : 0;
	const bool bUseDirtyPlateFlags =
		DirtyPlateCount > 0 &&
		(static_cast<double>(DirtyPlateCount) / static_cast<double>(FMath::Max(1, Plates.Num()))) <= 0.5;

	TArray<uint8> SeedSampleFlags;
	const TArray<uint8>* SeedSampleFlagsPtr = nullptr;
	if (bUseDirtyPlateFlags)
	{
		SeedSampleFlags.Init(0, InOutSamples.Num());
		int32 DirtySampleCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < InOutSamples.Num(); ++SampleIndex)
		{
			const FCanonicalSample& Sample = InOutSamples[SampleIndex];
			const FPhase2SampleState* State = Phase2States.IsValidIndex(SampleIndex) ? &Phase2States[SampleIndex] : nullptr;
			bool bSeedSample = !Plates.IsValidIndex(Sample.PlateId);
			if (!bSeedSample && DirtyPlateFlags->IsValidIndex(Sample.PlateId) && (*DirtyPlateFlags)[Sample.PlateId] != 0)
			{
				bSeedSample = true;
			}
			if (!bSeedSample && (Sample.bGapDetected || Sample.bOverlapDetected))
			{
				bSeedSample = true;
			}
			if (!bSeedSample && State && (State->bGap || State->bOverlap || (State->OwnershipMargin < (BoundaryConfidenceThreshold * 2.0f))))
			{
				bSeedSample = true;
			}
			if (!bSeedSample && Adjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Adjacency[SampleIndex])
				{
					if (!InOutSamples.IsValidIndex(NeighborIndex))
					{
						continue;
					}

					const int32 NeighborPlateId = InOutSamples[NeighborIndex].PlateId;
					if (!Plates.IsValidIndex(NeighborPlateId) || (DirtyPlateFlags->IsValidIndex(NeighborPlateId) && (*DirtyPlateFlags)[NeighborPlateId] != 0))
					{
						bSeedSample = true;
						break;
					}
				}
			}

			if (bSeedSample)
			{
				SeedSampleFlags[SampleIndex] = 1;
				++DirtySampleCount;
			}
		}

		const double DirtySampleRatio = static_cast<double>(DirtySampleCount) / static_cast<double>(FMath::Max(1, InOutSamples.Num()));
		if (DirtySampleCount > 0 && DirtySampleRatio <= 0.25)
		{
			SeedSampleFlagsPtr = &SeedSampleFlags;
		}
	}

	for (FCanonicalSample& Sample : InOutSamples)
	{
		if (!Plates.IsValidIndex(Sample.PlateId))
		{
			Sample.PlateId = FindNearestPlateByCap(Sample.Position);
		}
	}

	RunBoundaryLikeLocalOwnershipSanitizePass(Phase2States, InOutSamples, 1, SeedSampleFlagsPtr);
	EnforceConnectedPlateOwnershipForSamples(InOutSamples, bUseDirtyPlateFlags ? DirtyPlateFlags : nullptr);
	const bool bRescuedProtectedOwnership = RescueProtectedPlateOwnershipForSamples(InOutSamples, bUseDirtyPlateFlags ? DirtyPlateFlags : nullptr);
	if (bRescuedProtectedOwnership)
	{
		EnforceConnectedPlateOwnershipForSamples(InOutSamples, bUseDirtyPlateFlags ? DirtyPlateFlags : nullptr);
	}
	RunBoundaryLikeLocalOwnershipSanitizePass(Phase2States, InOutSamples, 1, SeedSampleFlagsPtr);
	EnforceConnectedPlateOwnershipForSamples(InOutSamples, bUseDirtyPlateFlags ? DirtyPlateFlags : nullptr);
	const bool bRescuedProtectedOwnershipAfterFinalSanitize = RescueProtectedPlateOwnershipForSamples(InOutSamples, bUseDirtyPlateFlags ? DirtyPlateFlags : nullptr);
	if (bRescuedProtectedOwnershipAfterFinalSanitize)
	{
		EnforceConnectedPlateOwnershipForSamples(InOutSamples, bUseDirtyPlateFlags ? DirtyPlateFlags : nullptr);
	}
	RebuildPlateMembershipFromSamples(InOutSamples, bUseDirtyPlateFlags ? DirtyPlateFlags : nullptr);
	UpdatePlateCanonicalCentersFromSamples(InOutSamples, bUseDirtyPlateFlags ? DirtyPlateFlags : nullptr);
	ClassifyTrianglesForSamples(InOutSamples);
	RebuildSpatialQueryData();
	DetectTerranesForSamples(InOutSamples);
}
void FTectonicPlanet::RefreshCanonicalStateAfterCollision(TArray<FCanonicalSample>& InOutSamples)
{
	TArray<FPhase2SampleState> EmptyPhase2States;
	EmptyPhase2States.SetNum(InOutSamples.Num());
	RefreshCanonicalStateAfterCollision(EmptyPhase2States, InOutSamples, nullptr);
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

	RebuildAdjacencyEdgeDistanceCache(Samples);

	ensureMsgf(ZeroValenceVertices == 0, TEXT("Adjacency graph has %d zero-valence vertices."), ZeroValenceVertices);
}

void FTectonicPlanet::RebuildAdjacencyEdgeDistanceCache(const TArray<FCanonicalSample>& InSamples)
{
	AdjacencyEdgeDistancesKm.Reset();
	if (Adjacency.Num() != InSamples.Num())
	{
		return;
	}

	AdjacencyEdgeDistancesKm.SetNum(InSamples.Num());
	ParallelFor(Adjacency.Num(), [this, &InSamples](int32 SampleIndex)
	{
		const FVector& SamplePosition = InSamples[SampleIndex].Position;
		const TArray<int32>& Neighbors = Adjacency[SampleIndex];
		TArray<float>& NeighborDistancesKm = AdjacencyEdgeDistancesKm[SampleIndex];
		NeighborDistancesKm.SetNumUninitialized(Neighbors.Num());

		for (int32 NeighborSlot = 0; NeighborSlot < Neighbors.Num(); ++NeighborSlot)
		{
			const int32 NeighborIndex = Neighbors[NeighborSlot];
			checkSlow(InSamples.IsValidIndex(NeighborIndex));

			const double Dot = FMath::Clamp(static_cast<double>(FVector::DotProduct(SamplePosition, InSamples[NeighborIndex].Position)), -1.0, 1.0);
			NeighborDistancesKm[NeighborSlot] = static_cast<float>(PlanetRadius * FMath::Acos(Dot));
		}
	}, EParallelForFlags::Unbalanced);
}






















