#include "TectonicPlanet.h"

#include "Algo/Sort.h"
#include "Async/ParallelFor.h"
#include "CompGeom/ConvexHull3.h"
#include "Distance/DistPoint3Triangle3.h"
#include "HAL/PlatformTime.h"
#include "Math/RandomStream.h"
#include "ShewchukPredicates.h"
#include "VectorUtil.h"

namespace
{
	constexpr double DeltaTimeMyears = 2.0;
	constexpr double TimeStepYears = DeltaTimeMyears * 1.0e6;
	constexpr double ErosionRateKmPerStep = 0.06;
	constexpr double DampeningRateKmPerStep = 0.08;
	constexpr double AccretionRateKmPerStep = 0.6;
	constexpr double MaxDistanceAdvanceKmPerStep = 200.0;
	constexpr double SubductionMaxDistanceRad = 0.283;
	constexpr double SubductionControlDistanceRad = 0.10;
	constexpr double SubductionMaxDistanceKm = SubductionMaxDistanceRad * 6371.0;
	constexpr double SubductionControlDistanceKm = SubductionControlDistanceRad * 6371.0;
	constexpr double SubductionBaseUpliftKmPerMy = 0.0006;
	constexpr double MaxPlateSpeedKmPerMy = 100.0;
	constexpr double ConvergentThresholdRatio = -0.3;
	constexpr double SlabPullEpsilon = 0.1;
	constexpr double MaxAxisChangePerStepRad = 0.05;
	constexpr double AndeanContinentalConversionRate = 0.005;
	constexpr double CollisionGlobalDistanceRad = 0.659;
	constexpr double CollisionCoefficient = 0.013;
	constexpr double CollisionCWBoost = 0.3;
	constexpr double MinCollisionRadiusRad = 0.05;
	constexpr int32 MinCollisionOverlapSamples = 4;
	constexpr int32 MaxCollisionHops = 3;
	constexpr int32 MinRiftChildSamples = 256;
	constexpr int32 MinForcedRiftChildren = 2;
	constexpr int32 MaxForcedRiftChildren = 4;
	constexpr int32 MinBoundaryContactCollisionEdges = 4;
	constexpr int32 MinBoundaryContactPersistenceResamples = 3;
	constexpr double ElevationCeilingKm = 10.0;
	constexpr double AbyssalPlainElevationKm = -6.0;
	constexpr double TrenchElevationKm = -10.0;
	constexpr double InitialContinentalElevationKm = 0.5;
	constexpr double InitialOceanicElevationKm = -6.0;
	constexpr double GlobalNoiseAmplitudeKm = 0.3;
	constexpr double BoundaryNoiseFrequency = 3.0;
	constexpr double ContinentalNoiseFrequency = 4.0;
	constexpr double ElevationNoiseFrequency = 2.0;
	constexpr double ContinentalThicknessKm = 35.0;
	constexpr double OceanicThicknessKm = 7.0;
	constexpr double RidgeElevationKm = -1.0;
	constexpr double MaxPlateSpeedMmPerYear = 100.0;
	constexpr double MinPlateAngularSpeedFactor = 0.3;
	constexpr double ResampleTriggerK = 10.0;
	constexpr int32 ResampleIntervalMin = 10;
	constexpr int32 ResampleIntervalMax = 60;
	constexpr double BoundingCapMargin = 1.0e-6;
	constexpr double TriangleEpsilon = 1.0e-12;
	constexpr double OverlapScoreEpsilon = 0.1;
	constexpr double DirectionDegeneracyThreshold = 1.0e-8;
	constexpr double GapSeparationDirectionEpsilon = 1.0e-10;
	constexpr double DivergenceEpsilon = 1.0e-10;

	uint32 MixBits(uint32 Value)
	{
		Value ^= Value >> 16;
		Value *= 0x7feb352dU;
		Value ^= Value >> 15;
		Value *= 0x846ca68bU;
		Value ^= Value >> 16;
		return Value;
	}

	int32 MakeDeterministicSeed(const int32 A, const int32 B, const int32 C = 0)
	{
		uint32 Seed = MixBits(static_cast<uint32>(A));
		Seed ^= MixBits(static_cast<uint32>(B) + 0x9e3779b9U);
		Seed ^= MixBits(static_cast<uint32>(C) + 0x85ebca6bU);
		return static_cast<int32>(Seed & 0x7fffffffU);
	}

	FVector3d MakeNoiseOffset(const int32 SeedA, const int32 SeedB, const int32 Salt)
	{
		FRandomStream Random(MakeDeterministicSeed(SeedA, SeedB, Salt));
		return FVector3d(
			Random.FRandRange(-10.0f, 10.0f),
			Random.FRandRange(-10.0f, 10.0f),
			Random.FRandRange(-10.0f, 10.0f));
	}

	FVector3d MakeRandomUnitVector(FRandomStream& Random)
	{
		const double Z = Random.FRandRange(-1.0f, 1.0f);
		const double Phi = Random.FRandRange(0.0f, 2.0f * PI);
		const double RadiusXY = FMath::Sqrt(FMath::Max(0.0, 1.0 - Z * Z));
		return FVector3d(
			RadiusXY * FMath::Cos(Phi),
			RadiusXY * FMath::Sin(Phi),
			Z);
	}

	double ComputeGeodesicDistance(const FVector3d& A, const FVector3d& B)
	{
		return FMath::Acos(FMath::Clamp(A.Dot(B), -1.0, 1.0));
	}

	double ComputeNoiseValue(const FVector3d& Position, const FVector3d& Offset, const double Frequency)
	{
		return static_cast<double>(FMath::PerlinNoise3D(FVector((Position * Frequency) + Offset)));
	}

	double ComputeWarpedDistance(
		const FVector3d& Position,
		const FVector3d& Centroid,
		const int32 GlobalSeed,
		const int32 PlateId,
		const int32 Salt,
		const double Amplitude,
		const double Frequency)
	{
		const double BaseDistance = ComputeGeodesicDistance(Position, Centroid);
		if (Amplitude <= 0.0)
		{
			return BaseDistance;
		}

		const FVector3d Offset = MakeNoiseOffset(GlobalSeed, PlateId, Salt);
		const double Noise = ComputeNoiseValue(Position, Offset, Frequency);
		const double WarpMultiplier = FMath::Max(0.05, 1.0 + (Amplitude * Noise));
		return BaseDistance * WarpMultiplier;
	}

	void AddUniqueNeighbor(TArray<int32>& Neighbors, const int32 NeighborIndex)
	{
		if (NeighborIndex != INDEX_NONE && !Neighbors.Contains(NeighborIndex))
		{
			Neighbors.Add(NeighborIndex);
		}
	}

	FSphericalBoundingCap BuildBoundingCapFromVertices(const TArray<FVector3d>& Vertices)
	{
		FSphericalBoundingCap Cap;
		if (Vertices.IsEmpty())
		{
			return Cap;
		}

		FVector3d CenterSum = FVector3d::ZeroVector;
		for (const FVector3d& Vertex : Vertices)
		{
			CenterSum += Vertex;
		}

		Cap.Center = CenterSum.GetSafeNormal();
		if (Cap.Center.IsNearlyZero())
		{
			Cap.Center = Vertices[0].GetSafeNormal();
		}

		double MinDot = 1.0;
		for (const FVector3d& Vertex : Vertices)
		{
			MinDot = FMath::Min(MinDot, Cap.Center.Dot(Vertex.GetSafeNormal()));
		}

		Cap.CosAngle = FMath::Clamp(MinDot - BoundingCapMargin, -1.0, 1.0);
		return Cap;
	}

	FSphericalBoundingCap BuildBoundingCapFromMembers(const FTectonicPlanet& Planet, const TArray<int32>& MemberSamples)
	{
		TArray<FVector3d> Vertices;
		Vertices.Reserve(MemberSamples.Num());
		for (const int32 SampleIndex : MemberSamples)
		{
			if (Planet.Samples.IsValidIndex(SampleIndex))
			{
				Vertices.Add(Planet.Samples[SampleIndex].Position.GetSafeNormal());
			}
		}

		return BuildBoundingCapFromVertices(Vertices);
	}

	uint64 MakeUndirectedEdgeKey(const int32 A, const int32 B)
	{
		const uint32 MinIndex = static_cast<uint32>(FMath::Min(A, B));
		const uint32 MaxIndex = static_cast<uint32>(FMath::Max(A, B));
		return (static_cast<uint64>(MinIndex) << 32) | static_cast<uint64>(MaxIndex);
	}

	FVector3d ComputePlanarBarycentric(const FVector3d& A, const FVector3d& B, const FVector3d& C, const FVector3d& P)
	{
		const FVector3d Normal = UE::Geometry::VectorUtil::Normal(A, B, C);
		const double PlaneOffset = Normal.Dot(A);
		const double RayDot = Normal.Dot(P);

		FVector3d ProjectedPoint = P;
		if (FMath::Abs(RayDot) > UE_DOUBLE_SMALL_NUMBER)
		{
			ProjectedPoint = P * (PlaneOffset / RayDot);
		}
		else
		{
			ProjectedPoint = P - ((P - A).Dot(Normal) * Normal);
		}

		const FVector3d V0 = B - A;
		const FVector3d V1 = C - A;
		const FVector3d V2 = ProjectedPoint - A;

		const double Dot00 = V0.Dot(V0);
		const double Dot01 = V0.Dot(V1);
		const double Dot02 = V0.Dot(V2);
		const double Dot11 = V1.Dot(V1);
		const double Dot12 = V1.Dot(V2);
		const double Denominator = Dot00 * Dot11 - Dot01 * Dot01;
		if (FMath::Abs(Denominator) <= 1.0e-20)
		{
			return FVector3d(-1.0, -1.0, -1.0);
		}

		const double InvDenominator = 1.0 / Denominator;
		const double V = (Dot11 * Dot02 - Dot01 * Dot12) * InvDenominator;
		const double W = (Dot00 * Dot12 - Dot01 * Dot02) * InvDenominator;
		const double U = 1.0 - V - W;
		return FVector3d(U, V, W);
	}

	FVector3d NormalizeBarycentric(const FVector3d& RawBarycentric)
	{
		FVector3d Clamped(
			FMath::Max(0.0, RawBarycentric.X),
			FMath::Max(0.0, RawBarycentric.Y),
			FMath::Max(0.0, RawBarycentric.Z));
		const double Sum = Clamped.X + Clamped.Y + Clamped.Z;
		if (Sum > UE_DOUBLE_SMALL_NUMBER)
		{
			Clamped /= Sum;
		}
		return Clamped;
	}

	struct FSubductionQueueEntry
	{
		int32 SampleIndex = INDEX_NONE;
		double DistanceKm = 0.0;
		float SpeedKmPerMy = 0.0f;
	};

	struct FSubductionQueueLess
	{
		bool operator()(const FSubductionQueueEntry& Left, const FSubductionQueueEntry& Right) const
		{
			if (!FMath::IsNearlyEqual(Left.DistanceKm, Right.DistanceKm, TriangleEpsilon))
			{
				return Left.DistanceKm > Right.DistanceKm;
			}

			if (!FMath::IsNearlyEqual(Left.SpeedKmPerMy, Right.SpeedKmPerMy, UE_KINDA_SMALL_NUMBER))
			{
				return Left.SpeedKmPerMy < Right.SpeedKmPerMy;
			}

			return Left.SampleIndex > Right.SampleIndex;
		}
	};

	struct FConvergentBoundaryEdge
	{
		int32 SampleIndexA = INDEX_NONE;
		int32 SampleIndexB = INDEX_NONE;
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		int32 OverridingPlateId = INDEX_NONE;
		int32 SubductingPlateId = INDEX_NONE;
		int32 OverridingSampleIndex = INDEX_NONE;
		int32 SubductingSampleIndex = INDEX_NONE;
		double NormalComponent = 0.0;
		float ConvergenceSpeedKmPerMy = 0.0f;
	};

	struct FCollisionComponent
	{
		int32 OverridingPlateId = INDEX_NONE;
		int32 SubductingPlateId = INDEX_NONE;
		TArray<int32> SampleIndices;
		int32 MinSampleIndex = INDEX_NONE;
	};

	struct FBoundaryContactCollisionCandidate
	{
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		int32 SampleIndexA = INDEX_NONE;
		int32 SampleIndexB = INDEX_NONE;
		double ConvergenceMagnitude = 0.0;
		int32 MinSampleIndex = INDEX_NONE;
	};

	struct FBoundaryContactCollisionZone
	{
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		int32 CandidateCount = 0;
		double TotalConvergenceMagnitude = 0.0;
		int32 MinSampleIndex = INDEX_NONE;
		TArray<int32> SeedSampleIndices;
		FVector3d ContactCenterSum = FVector3d::ZeroVector;
	};

	bool IsFiniteVector(const FVector3d& Vector)
	{
		return FMath::IsFinite(Vector.X) && FMath::IsFinite(Vector.Y) && FMath::IsFinite(Vector.Z);
	}

	const FPlate* FindPlateById(const FTectonicPlanet& Planet, const int32 PlateId, int32* OutPlateIndex = nullptr)
	{
		if (OutPlateIndex != nullptr)
		{
			*OutPlateIndex = INDEX_NONE;
		}

		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			if (Planet.Plates[PlateIndex].Id == PlateId)
			{
				if (OutPlateIndex != nullptr)
				{
					*OutPlateIndex = PlateIndex;
				}
				return &Planet.Plates[PlateIndex];
			}
		}

		return nullptr;
	}

	FPlate* FindPlateById(FTectonicPlanet& Planet, const int32 PlateId, int32* OutPlateIndex = nullptr)
	{
		if (OutPlateIndex != nullptr)
		{
			*OutPlateIndex = INDEX_NONE;
		}

		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			if (Planet.Plates[PlateIndex].Id == PlateId)
			{
				if (OutPlateIndex != nullptr)
				{
					*OutPlateIndex = PlateIndex;
				}
				return &Planet.Plates[PlateIndex];
			}
		}

		return nullptr;
	}

	bool IsOverridingPlate(const FTectonicPlanet& Planet, const int32 CandidatePlateId, const int32 OtherPlateId)
	{
		const FPlate* CandidatePlate = FindPlateById(Planet, CandidatePlateId);
		const FPlate* OtherPlate = FindPlateById(Planet, OtherPlateId);
		check(CandidatePlate != nullptr);
		check(OtherPlate != nullptr);

		return
			CandidatePlate->OverlapScore > OtherPlate->OverlapScore ||
			(CandidatePlate->OverlapScore == OtherPlate->OverlapScore &&
				CandidatePlateId < OtherPlateId);
	}

	const TCHAR* GetResampleTriggerReasonName(const EResampleTriggerReason Reason)
	{
		switch (Reason)
		{
		case EResampleTriggerReason::Periodic:
			return TEXT("Periodic");
		case EResampleTriggerReason::CollisionFollowup:
			return TEXT("CollisionFollowup");
		case EResampleTriggerReason::RiftFollowup:
			return TEXT("RiftFollowup");
		case EResampleTriggerReason::SafetyValve:
			return TEXT("SafetyValve");
		case EResampleTriggerReason::Manual:
			return TEXT("Manual");
		case EResampleTriggerReason::None:
		default:
			return TEXT("None");
		}
	}

	const TCHAR* GetResampleOwnershipModeName(const EResampleOwnershipMode OwnershipMode)
	{
		switch (OwnershipMode)
		{
		case EResampleOwnershipMode::PreserveOwnership:
			return TEXT("PreserveOwnership");
		case EResampleOwnershipMode::StableOverlaps:
			return TEXT("StableOverlaps");
		case EResampleOwnershipMode::FullResolution:
		default:
			return TEXT("FullResolution");
		}
	}

	bool TryComputeConvergentBoundaryEdgeMetrics(
		const FTectonicPlanet& Planet,
		const int32 SampleIndexA,
		const int32 SampleIndexB,
		double& OutNormalComponent,
		float& OutConvergenceSpeedKmPerMy)
	{
		OutNormalComponent = 0.0;
		OutConvergenceSpeedKmPerMy = 0.0f;

		if (!Planet.Samples.IsValidIndex(SampleIndexA) || !Planet.Samples.IsValidIndex(SampleIndexB))
		{
			return false;
		}

		const int32 PlateA = Planet.Samples[SampleIndexA].PlateId;
		const int32 PlateB = Planet.Samples[SampleIndexB].PlateId;
		const FPlate* PlateAData = FindPlateById(Planet, PlateA);
		const FPlate* PlateBData = FindPlateById(Planet, PlateB);
		if (PlateAData == nullptr || PlateBData == nullptr || PlateA == PlateB)
		{
			return false;
		}

		const FVector3d PosA = Planet.Samples[SampleIndexA].Position.GetSafeNormal();
		const FVector3d PosB = Planet.Samples[SampleIndexB].Position.GetSafeNormal();
		FVector3d Midpoint = (PosA + PosB).GetSafeNormal();
		if (Midpoint.IsNearlyZero())
		{
			Midpoint = PosA;
		}

		const FVector3d AxisA = PlateAData->RotationAxis.IsNearlyZero()
			? FVector3d::ZeroVector
			: PlateAData->RotationAxis.GetSafeNormal();
		const FVector3d AxisB = PlateBData->RotationAxis.IsNearlyZero()
			? FVector3d::ZeroVector
			: PlateBData->RotationAxis.GetSafeNormal();
		const FVector3d AngVelA = AxisA * PlateAData->AngularSpeed;
		const FVector3d AngVelB = AxisB * PlateBData->AngularSpeed;
		const FVector3d SurfVelA = FVector3d::CrossProduct(AngVelA, Midpoint);
		const FVector3d SurfVelB = FVector3d::CrossProduct(AngVelB, Midpoint);
		const FVector3d RelVel = SurfVelB - SurfVelA;
		const double RelSpeed = RelVel.Length();
		if (RelSpeed <= UE_DOUBLE_SMALL_NUMBER)
		{
			return false;
		}

		const FVector3d Separation = (PosB - PosA).GetSafeNormal();
		const double NormalComponent = RelVel.Dot(Separation);
		const double Ratio = NormalComponent / RelSpeed;
		if (Ratio >= ConvergentThresholdRatio)
		{
			return false;
		}

		OutNormalComponent = NormalComponent;
		OutConvergenceSpeedKmPerMy = static_cast<float>(
			(FMath::Abs(NormalComponent) * Planet.PlanetRadiusKm) / FMath::Max(DeltaTimeMyears, UE_DOUBLE_SMALL_NUMBER));
		return true;
	}

	TArray<FConvergentBoundaryEdge> BuildConvergentBoundaryEdges(const FTectonicPlanet& Planet)
	{
		TArray<FConvergentBoundaryEdge> ConvergentEdges;
		if (Planet.SampleAdjacency.Num() != Planet.Samples.Num() || Planet.Plates.IsEmpty())
		{
			return ConvergentEdges;
		}

		for (int32 SampleIndexA = 0; SampleIndexA < Planet.SampleAdjacency.Num(); ++SampleIndexA)
		{
			const int32 PlateA = Planet.Samples.IsValidIndex(SampleIndexA) ? Planet.Samples[SampleIndexA].PlateId : INDEX_NONE;
			if (FindPlateById(Planet, PlateA) == nullptr)
			{
				continue;
			}

			for (const int32 SampleIndexB : Planet.SampleAdjacency[SampleIndexA])
			{
				if (SampleIndexB <= SampleIndexA || !Planet.Samples.IsValidIndex(SampleIndexB))
				{
					continue;
				}

				const int32 PlateB = Planet.Samples[SampleIndexB].PlateId;
				if (FindPlateById(Planet, PlateB) == nullptr || PlateA == PlateB)
				{
					continue;
				}

				double NormalComponent = 0.0;
				float ConvergenceSpeedKmPerMy = 0.0f;
				if (!TryComputeConvergentBoundaryEdgeMetrics(
						Planet,
						SampleIndexA,
						SampleIndexB,
						NormalComponent,
						ConvergenceSpeedKmPerMy))
				{
					continue;
				}

				const bool bPlateAOverriding = IsOverridingPlate(Planet, PlateA, PlateB);
				FConvergentBoundaryEdge& Edge = ConvergentEdges.AddDefaulted_GetRef();
				Edge.SampleIndexA = SampleIndexA;
				Edge.SampleIndexB = SampleIndexB;
				Edge.PlateA = PlateA;
				Edge.PlateB = PlateB;
				Edge.OverridingPlateId = bPlateAOverriding ? PlateA : PlateB;
				Edge.SubductingPlateId = bPlateAOverriding ? PlateB : PlateA;
				Edge.OverridingSampleIndex = bPlateAOverriding ? SampleIndexA : SampleIndexB;
				Edge.SubductingSampleIndex = bPlateAOverriding ? SampleIndexB : SampleIndexA;
				Edge.NormalComponent = NormalComponent;
				Edge.ConvergenceSpeedKmPerMy = ConvergenceSpeedKmPerMy;
			}
		}

		return ConvergentEdges;
	}

	uint64 MakeBoundaryContactPlatePairKey(const int32 PlateA, const int32 PlateB)
	{
		const uint32 MinPlateId = static_cast<uint32>(FMath::Min(PlateA, PlateB));
		const uint32 MaxPlateId = static_cast<uint32>(FMath::Max(PlateA, PlateB));
		return (static_cast<uint64>(MinPlateId) << 32) | static_cast<uint64>(MaxPlateId);
	}

	bool IsBoundaryContactZoneStronger(
		const FBoundaryContactCollisionZone& Left,
		const FBoundaryContactCollisionZone& Right)
	{
		if (Left.CandidateCount != Right.CandidateCount)
		{
			return Left.CandidateCount > Right.CandidateCount;
		}
		if (!FMath::IsNearlyEqual(
				Left.TotalConvergenceMagnitude,
				Right.TotalConvergenceMagnitude,
				UE_DOUBLE_SMALL_NUMBER))
		{
			return Left.TotalConvergenceMagnitude > Right.TotalConvergenceMagnitude;
		}
		if (Left.PlateA != Right.PlateA)
		{
			return Left.PlateA < Right.PlateA;
		}
		if (Left.PlateB != Right.PlateB)
		{
			return Left.PlateB < Right.PlateB;
		}
		return Left.MinSampleIndex < Right.MinSampleIndex;
	}

	int32 ChooseDominantPreviousTerraneId(
		const TArray<int32>& PreviousTerraneAssignments,
		const TArray<int32>& TerraneSampleIndices);

	bool TryBuildCollisionEventFromBoundaryContactSeeds(
		const FTectonicPlanet& Planet,
		const TArray<int32>& BoundarySeedSampleIndices,
		const int32 PlateA,
		const int32 PlateB,
		const FVector3d& ContactCenterHint,
		const TArray<int32>& PreviousPlateIds,
		const TArray<float>& PreviousContinentalWeights,
		const TArray<int32>& PreviousTerraneAssignments,
		TArray<int32>& InOutNewPlateIds,
		FCollisionEvent& OutEvent,
		int32& OutTerraneSeedCount,
		int32& OutRecoveredTerraneCount,
		const TCHAR*& OutFailureReason)
	{
		OutEvent = FCollisionEvent{};
		OutTerraneSeedCount = 0;
		OutRecoveredTerraneCount = 0;
		OutFailureReason = TEXT("none");

		if (FindPlateById(Planet, PlateA) == nullptr ||
			FindPlateById(Planet, PlateB) == nullptr ||
			PlateA == PlateB)
		{
			OutFailureReason = TEXT("invalid_pair");
			return false;
		}

		const int32 OverridingPlateId = IsOverridingPlate(Planet, PlateA, PlateB) ? PlateA : PlateB;
		const int32 SubductingPlateId = OverridingPlateId == PlateA ? PlateB : PlateA;

		TArray<int32> TerraneSeedSampleIndices;
		for (const int32 SampleIndex : BoundarySeedSampleIndices)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex) ||
				!PreviousPlateIds.IsValidIndex(SampleIndex) ||
				!PreviousContinentalWeights.IsValidIndex(SampleIndex))
			{
				continue;
			}

			if (PreviousPlateIds[SampleIndex] == SubductingPlateId &&
				PreviousContinentalWeights[SampleIndex] >= 0.5f)
			{
				TerraneSeedSampleIndices.AddUnique(SampleIndex);
			}
		}

		OutTerraneSeedCount = TerraneSeedSampleIndices.Num();
		if (TerraneSeedSampleIndices.IsEmpty())
		{
			OutFailureReason = TEXT("no_subducting_side_seeds");
			return false;
		}

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		TArray<int32> TerraneSampleIndices;
		for (const int32 SeedSampleIndex : TerraneSeedSampleIndices)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) || Visited[SeedSampleIndex] != 0)
			{
				continue;
			}

			TArray<int32, TInlineAllocator<64>> Stack;
			Stack.Add(SeedSampleIndex);
			Visited[SeedSampleIndex] = 1;

			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				TerraneSampleIndices.Add(SampleIndex);

				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						Visited[NeighborIndex] != 0 ||
						!PreviousPlateIds.IsValidIndex(NeighborIndex) ||
						!PreviousContinentalWeights.IsValidIndex(NeighborIndex) ||
						PreviousPlateIds[NeighborIndex] != SubductingPlateId ||
						PreviousContinentalWeights[NeighborIndex] < 0.5f)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}
		}

		OutRecoveredTerraneCount = TerraneSampleIndices.Num();
		if (TerraneSampleIndices.IsEmpty())
		{
			OutFailureReason = TEXT("empty_terrane");
			return false;
		}

		for (const int32 SampleIndex : TerraneSampleIndices)
		{
			if (InOutNewPlateIds.IsValidIndex(SampleIndex))
			{
				InOutNewPlateIds[SampleIndex] = OverridingPlateId;
			}
		}

		FVector3d CollisionCenter = ContactCenterHint.GetSafeNormal();
		if (CollisionCenter.IsNearlyZero())
		{
			FVector3d CollisionCenterSum = FVector3d::ZeroVector;
			for (const int32 SampleIndex : BoundarySeedSampleIndices)
			{
				if (Planet.Samples.IsValidIndex(SampleIndex))
				{
					CollisionCenterSum += Planet.Samples[SampleIndex].Position.GetSafeNormal();
				}
			}
			CollisionCenter = CollisionCenterSum.GetSafeNormal();
		}
		if (CollisionCenter.IsNearlyZero() && !TerraneSeedSampleIndices.IsEmpty())
		{
			CollisionCenter = Planet.Samples[TerraneSeedSampleIndices[0]].Position.GetSafeNormal();
		}
		if (CollisionCenter.IsNearlyZero())
		{
			OutFailureReason = TEXT("invalid_contact_center");
			return false;
		}

		OutEvent.bDetected = true;
		OutEvent.OverridingPlateId = OverridingPlateId;
		OutEvent.SubductingPlateId = SubductingPlateId;
		OutEvent.CollisionSampleIndices = BoundarySeedSampleIndices;
		OutEvent.TerraneSampleIndices = TerraneSampleIndices;
		OutEvent.TerraneId = ChooseDominantPreviousTerraneId(PreviousTerraneAssignments, TerraneSampleIndices);
		OutEvent.CollisionCenter = CollisionCenter;
		return true;
	}

	FVector3d ComputePlateCentroidOnUnitSphere(const FTectonicPlanet& Planet, const FPlate& Plate)
	{
		FVector3d CentroidSum = FVector3d::ZeroVector;
		for (const int32 SampleIndex : Plate.MemberSamples)
		{
			if (Planet.Samples.IsValidIndex(SampleIndex))
			{
				CentroidSum += Planet.Samples[SampleIndex].Position.GetSafeNormal();
			}
		}

		return CentroidSum.GetSafeNormal();
	}

	FVector3d ComputeSampleSetCentroidOnUnitSphere(const FTectonicPlanet& Planet, const TArray<int32>& SampleIndices)
	{
		FVector3d CentroidSum = FVector3d::ZeroVector;
		for (const int32 SampleIndex : SampleIndices)
		{
			if (Planet.Samples.IsValidIndex(SampleIndex))
			{
				CentroidSum += Planet.Samples[SampleIndex].Position.GetSafeNormal();
			}
		}
		return CentroidSum.GetSafeNormal();
	}

	FString JoinIntArrayForLog(const TArray<int32>& Values)
	{
		TArray<FString> Parts;
		Parts.Reserve(Values.Num());
		for (const int32 Value : Values)
		{
			Parts.Add(FString::FromInt(Value));
		}

		return FString::Join(Parts, TEXT(","));
	}

	bool ChooseForcedRiftCentroidSamples(
		const FTectonicPlanet& Planet,
		const TArray<int32>& ParentMembers,
		const int32 ChildCount,
		const int32 Seed,
		TArray<int32>& OutCentroidSamples)
	{
		OutCentroidSamples.Reset();
		if (ChildCount < 2 || ParentMembers.Num() < ChildCount)
		{
			return false;
		}

		const int32 FirstMemberOffset =
			FMath::Abs(Seed) % ParentMembers.Num();
		if (!ParentMembers.IsValidIndex(FirstMemberOffset) ||
			!Planet.Samples.IsValidIndex(ParentMembers[FirstMemberOffset]))
		{
			return false;
		}

		OutCentroidSamples.Add(ParentMembers[FirstMemberOffset]);
		TSet<int32> SelectedCentroidSamples;
		SelectedCentroidSamples.Add(OutCentroidSamples[0]);

		while (OutCentroidSamples.Num() < ChildCount)
		{
			int32 BestSampleIndex = INDEX_NONE;
			double BestMinDistance = -1.0;
			for (const int32 CandidateSampleIndex : ParentMembers)
			{
				if (!Planet.Samples.IsValidIndex(CandidateSampleIndex) ||
					SelectedCentroidSamples.Contains(CandidateSampleIndex))
				{
					continue;
				}

				double MinDistanceToExistingCentroids = TNumericLimits<double>::Max();
				for (const int32 ExistingCentroidSampleIndex : OutCentroidSamples)
				{
					MinDistanceToExistingCentroids = FMath::Min(
						MinDistanceToExistingCentroids,
						ComputeGeodesicDistance(
							Planet.Samples[CandidateSampleIndex].Position.GetSafeNormal(),
							Planet.Samples[ExistingCentroidSampleIndex].Position.GetSafeNormal()));
				}

				if (MinDistanceToExistingCentroids > BestMinDistance + TriangleEpsilon ||
					(FMath::IsNearlyEqual(MinDistanceToExistingCentroids, BestMinDistance, TriangleEpsilon) &&
						(BestSampleIndex == INDEX_NONE || CandidateSampleIndex < BestSampleIndex)))
				{
					BestSampleIndex = CandidateSampleIndex;
					BestMinDistance = MinDistanceToExistingCentroids;
				}
			}

			if (!Planet.Samples.IsValidIndex(BestSampleIndex))
			{
				return false;
			}

			OutCentroidSamples.Add(BestSampleIndex);
			SelectedCentroidSamples.Add(BestSampleIndex);
		}

		return OutCentroidSamples.Num() == ChildCount;
	}

	bool PartitionParentMembersByCentroids(
		const FTectonicPlanet& Planet,
		const TArray<int32>& ParentMembers,
		const TArray<int32>& CentroidSampleIndices,
		const int32 EventSeed,
		const bool bUseWarpedBoundaries,
		const double WarpAmplitude,
		const double WarpFrequency,
		TArray<TArray<int32>>& OutChildMemberSamples)
	{
		OutChildMemberSamples.Reset();
		if (CentroidSampleIndices.Num() < 2)
		{
			return false;
		}

		TArray<FVector3d> CentroidPositions;
		CentroidPositions.Reserve(CentroidSampleIndices.Num());
		for (const int32 CentroidSampleIndex : CentroidSampleIndices)
		{
			if (!Planet.Samples.IsValidIndex(CentroidSampleIndex))
			{
				return false;
			}

			CentroidPositions.Add(Planet.Samples[CentroidSampleIndex].Position.GetSafeNormal());
		}

		OutChildMemberSamples.SetNum(CentroidPositions.Num());
		for (const int32 SampleIndex : ParentMembers)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FVector3d SamplePosition = Planet.Samples[SampleIndex].Position.GetSafeNormal();
			int32 BestChildIndex = INDEX_NONE;
			double BestDistance = TNumericLimits<double>::Max();
			for (int32 ChildIndex = 0; ChildIndex < CentroidPositions.Num(); ++ChildIndex)
			{
				const double DistanceToCentroid = bUseWarpedBoundaries
					? ComputeWarpedDistance(
						SamplePosition,
						CentroidPositions[ChildIndex],
						EventSeed,
						CentroidSampleIndices[ChildIndex],
						303,
						WarpAmplitude,
						WarpFrequency)
					: ComputeGeodesicDistance(SamplePosition, CentroidPositions[ChildIndex]);
				if (DistanceToCentroid < BestDistance - TriangleEpsilon ||
					(FMath::IsNearlyEqual(DistanceToCentroid, BestDistance, TriangleEpsilon) &&
						(BestChildIndex == INDEX_NONE ||
							CentroidSampleIndices[ChildIndex] < CentroidSampleIndices[BestChildIndex])))
				{
					BestChildIndex = ChildIndex;
					BestDistance = DistanceToCentroid;
				}
			}

			if (!OutChildMemberSamples.IsValidIndex(BestChildIndex))
			{
				return false;
			}

			OutChildMemberSamples[BestChildIndex].Add(SampleIndex);
		}

		for (const TArray<int32>& ChildMembers : OutChildMemberSamples)
		{
			if (ChildMembers.Num() < MinRiftChildSamples)
			{
				return false;
			}
		}

		return true;
	}

	struct FRiftTerraneMetrics
	{
		int32 TerraneCountBefore = 0;
		int32 TerraneCountAfter = 0;
		int32 TerraneFragmentsOnChildA = 0;
		int32 TerraneFragmentsOnChildB = 0;
		int32 TerraneSplitCount = 0;
		int32 TerranePreservedCount = 0;
		TArray<int32> ChildTerraneFragmentCounts;
		TArray<int32> PreRiftTerraneIds;
		TArray<int32> PreRiftTerraneTouchedChildCounts;
	};

	FRiftTerraneMetrics ComputeRiftTerraneMetrics(
		const FTectonicPlanet& Planet,
		const FPendingRiftEvent& RiftEvent)
	{
		FRiftTerraneMetrics Metrics;
		if (!RiftEvent.bValid ||
			RiftEvent.FormerParentSampleIndices.Num() != RiftEvent.FormerParentTerraneIds.Num() ||
			RiftEvent.ChildPlateIds.Num() != RiftEvent.ChildSampleCounts.Num())
		{
			return Metrics;
		}

		TSet<int32> ChildPlateIdSet;
		for (const int32 ChildPlateId : RiftEvent.ChildPlateIds)
		{
			if (ChildPlateId != INDEX_NONE)
			{
				ChildPlateIdSet.Add(ChildPlateId);
			}
		}

		TMap<int32, TSet<int32>> TouchedChildPlateIdsByPreRiftTerrane;
		for (int32 FormerSampleOffset = 0; FormerSampleOffset < RiftEvent.FormerParentSampleIndices.Num(); ++FormerSampleOffset)
		{
			const int32 SampleIndex = RiftEvent.FormerParentSampleIndices[FormerSampleOffset];
			const int32 PreRiftTerraneId = RiftEvent.FormerParentTerraneIds[FormerSampleOffset];
			if (PreRiftTerraneId == INDEX_NONE || !Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 CurrentPlateId = Planet.Samples[SampleIndex].PlateId;
			if (ChildPlateIdSet.Contains(CurrentPlateId))
			{
				TouchedChildPlateIdsByPreRiftTerrane.FindOrAdd(PreRiftTerraneId).Add(CurrentPlateId);
			}
		}

		TArray<int32> SortedPreRiftTerraneIds;
		TouchedChildPlateIdsByPreRiftTerrane.GenerateKeyArray(SortedPreRiftTerraneIds);
		SortedPreRiftTerraneIds.Sort();
		Metrics.PreRiftTerraneIds = SortedPreRiftTerraneIds;
		Metrics.TerraneCountBefore = SortedPreRiftTerraneIds.Num();
		for (const int32 PreRiftTerraneId : SortedPreRiftTerraneIds)
		{
			const TSet<int32>* TouchedChildPlateIds =
				TouchedChildPlateIdsByPreRiftTerrane.Find(PreRiftTerraneId);
			const int32 TouchedChildCount = TouchedChildPlateIds != nullptr ? TouchedChildPlateIds->Num() : 0;
			Metrics.PreRiftTerraneTouchedChildCounts.Add(TouchedChildCount);
			if (TouchedChildCount > 1)
			{
				++Metrics.TerraneSplitCount;
			}
			else if (TouchedChildCount == 1)
			{
				++Metrics.TerranePreservedCount;
			}
		}

		TMap<int32, TSet<int32>> PostRiftTerraneIdsByChildPlate;
		for (const FSample& Sample : Planet.Samples)
		{
			if (Sample.TerraneId == INDEX_NONE || !ChildPlateIdSet.Contains(Sample.PlateId))
			{
				continue;
			}

			PostRiftTerraneIdsByChildPlate.FindOrAdd(Sample.PlateId).Add(Sample.TerraneId);
		}

		for (int32 ChildIndex = 0; ChildIndex < RiftEvent.ChildPlateIds.Num(); ++ChildIndex)
		{
			const int32 ChildPlateId = RiftEvent.ChildPlateIds[ChildIndex];
			const int32 ChildFragmentCount =
				PostRiftTerraneIdsByChildPlate.Contains(ChildPlateId)
					? PostRiftTerraneIdsByChildPlate.FindChecked(ChildPlateId).Num()
					: 0;
			Metrics.ChildTerraneFragmentCounts.Add(ChildFragmentCount);
			Metrics.TerraneCountAfter += ChildFragmentCount;
		}

		Metrics.TerraneFragmentsOnChildA =
			Metrics.ChildTerraneFragmentCounts.IsValidIndex(0)
				? Metrics.ChildTerraneFragmentCounts[0]
				: 0;
		Metrics.TerraneFragmentsOnChildB =
			Metrics.ChildTerraneFragmentCounts.IsValidIndex(1)
				? Metrics.ChildTerraneFragmentCounts[1]
				: 0;
		return Metrics;
	}

	void ComputePlateContinentalSummary(
		const FTectonicPlanet& Planet,
		const FPlate& Plate,
		int32& OutContinentalSampleCount,
		double& OutContinentalFraction)
	{
		OutContinentalSampleCount = 0;
		OutContinentalFraction = 0.0;
		if (Plate.MemberSamples.IsEmpty())
		{
			return;
		}

		for (const int32 SampleIndex : Plate.MemberSamples)
		{
			if (Planet.Samples.IsValidIndex(SampleIndex) &&
				Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f)
			{
				++OutContinentalSampleCount;
			}
		}

		OutContinentalFraction =
			static_cast<double>(OutContinentalSampleCount) /
			static_cast<double>(Plate.MemberSamples.Num());
	}

	struct FRiftBoundaryActivitySummary
	{
		int32 ContactEdgeCount = 0;
		int32 DivergentEdgeCount = 0;
	};

	FRiftBoundaryActivitySummary ComputeChildBoundaryActivitySummary(
		const FTectonicPlanet& Planet,
		const TArray<int32>& ChildPlateIds)
	{
		FRiftBoundaryActivitySummary Summary;
		TSet<int32> ChildPlateIdSet;
		for (const int32 ChildPlateId : ChildPlateIds)
		{
			if (ChildPlateId != INDEX_NONE)
			{
				ChildPlateIdSet.Add(ChildPlateId);
			}
		}

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex) || !Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 PlateAId = Planet.Samples[SampleIndex].PlateId;
			if (!ChildPlateIdSet.Contains(PlateAId))
			{
				continue;
			}

			const FPlate* PlateA = FindPlateById(Planet, PlateAId);
			if (PlateA == nullptr)
			{
				continue;
			}

			const FVector3d PosA = Planet.Samples[SampleIndex].Position.GetSafeNormal();
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (NeighborIndex <= SampleIndex || !Planet.Samples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				const int32 PlateBId = Planet.Samples[NeighborIndex].PlateId;
				if (PlateBId == PlateAId || !ChildPlateIdSet.Contains(PlateBId))
				{
					continue;
				}

				const FPlate* PlateB = FindPlateById(Planet, PlateBId);
				if (PlateB == nullptr)
				{
					continue;
				}

				++Summary.ContactEdgeCount;

				const FVector3d PosB = Planet.Samples[NeighborIndex].Position.GetSafeNormal();
				FVector3d Midpoint = (PosA + PosB).GetSafeNormal();
				if (Midpoint.IsNearlyZero())
				{
					Midpoint = PosA;
				}

				const FVector3d AxisA = PlateA->RotationAxis.IsNearlyZero()
					? FVector3d::ZeroVector
					: PlateA->RotationAxis.GetSafeNormal();
				const FVector3d AxisB = PlateB->RotationAxis.IsNearlyZero()
					? FVector3d::ZeroVector
					: PlateB->RotationAxis.GetSafeNormal();
				const FVector3d RelativeVelocity =
					FVector3d::CrossProduct(AxisB * PlateB->AngularSpeed, Midpoint) -
					FVector3d::CrossProduct(AxisA * PlateA->AngularSpeed, Midpoint);
				const double RelativeSpeed = RelativeVelocity.Length();
				if (RelativeSpeed <= UE_DOUBLE_SMALL_NUMBER)
				{
					continue;
				}

				const FVector3d Separation = (PosB - PosA).GetSafeNormal();
				const double Ratio = RelativeVelocity.Dot(Separation) / RelativeSpeed;
				if (Ratio > 0.3)
				{
					++Summary.DivergentEdgeCount;
				}
			}
		}

		return Summary;
	}

	int32 CountBoundaryContactEdgesBetweenChildPlates(
		const FTectonicPlanet& Planet,
		const TArray<int32>& ChildPlateIds)
	{
		return ComputeChildBoundaryActivitySummary(Planet, ChildPlateIds).ContactEdgeCount;
	}

	bool TryComputeForcedRiftChildAxes(
		const FVector3d& ParentCentroid,
		const FVector3d& ParentRotationAxis,
		const TArray<FVector3d>& ChildCentroids,
		TArray<FVector3d>& OutChildAxes)
	{
		OutChildAxes.Reset();
		if (ChildCentroids.Num() < 2)
		{
			return false;
		}

		auto ProjectOntoTangentPlane = [](const FVector3d& Vector, const FVector3d& Normal)
		{
			return Vector - (Vector.Dot(Normal) * Normal);
		};

		const FVector3d NormalizedParentAxis =
			ParentRotationAxis.IsNearlyZero()
				? FVector3d::ZeroVector
				: ParentRotationAxis.GetSafeNormal();

		if (ChildCentroids.Num() == 2)
		{
			const FVector3d ChildCentroidA = ChildCentroids[0].GetSafeNormal();
			const FVector3d ChildCentroidB = ChildCentroids[1].GetSafeNormal();
			if (ChildCentroidA.IsNearlyZero() || ChildCentroidB.IsNearlyZero())
			{
				return false;
			}

			FVector3d FractureCenter = (ChildCentroidA + ChildCentroidB).GetSafeNormal();
			if (FractureCenter.IsNearlyZero())
			{
				FractureCenter = !ParentCentroid.IsNearlyZero() ? ParentCentroid.GetSafeNormal() : ChildCentroidA;
			}

			FVector3d SeparationDirection =
				ProjectOntoTangentPlane(ChildCentroidB - ChildCentroidA, FractureCenter).GetSafeNormal();
			if (SeparationDirection.IsNearlyZero())
			{
				SeparationDirection =
					ProjectOntoTangentPlane(ChildCentroidB - ChildCentroidA, ChildCentroidA).GetSafeNormal();
			}
			if (SeparationDirection.IsNearlyZero())
			{
				return false;
			}

			const FVector3d DivergentAxisA =
				FVector3d::CrossProduct(FractureCenter, -SeparationDirection).GetSafeNormal();
			const FVector3d DivergentAxisB =
				FVector3d::CrossProduct(FractureCenter, SeparationDirection).GetSafeNormal();
			if (DivergentAxisA.IsNearlyZero() ||
				DivergentAxisB.IsNearlyZero() ||
				!IsFiniteVector(DivergentAxisA) ||
				!IsFiniteVector(DivergentAxisB))
			{
				return false;
			}

			OutChildAxes = { DivergentAxisA, DivergentAxisB };
			if (!NormalizedParentAxis.IsNearlyZero())
			{
				for (FVector3d& ChildAxis : OutChildAxes)
				{
					const FVector3d BlendedAxis =
						((0.05 * NormalizedParentAxis) + (0.95 * ChildAxis)).GetSafeNormal();
					if (!BlendedAxis.IsNearlyZero() && IsFiniteVector(BlendedAxis))
					{
						ChildAxis = BlendedAxis;
					}
				}
			}

			return true;
		}

		TArray<FVector3d> PureDivergentAxes;
		PureDivergentAxes.Reserve(ChildCentroids.Num());
		for (int32 ChildIndex = 0; ChildIndex < ChildCentroids.Num(); ++ChildIndex)
		{
			const FVector3d ChildCentroid = ChildCentroids[ChildIndex].GetSafeNormal();
			if (ChildCentroid.IsNearlyZero())
			{
				return false;
			}

			FVector3d DesiredVelocityDirection = FVector3d::ZeroVector;
			for (int32 OtherChildIndex = 0; OtherChildIndex < ChildCentroids.Num(); ++OtherChildIndex)
			{
				if (OtherChildIndex == ChildIndex)
				{
					continue;
				}

				const FVector3d AwayFromOtherChild =
					ProjectOntoTangentPlane(ChildCentroid - ChildCentroids[OtherChildIndex], ChildCentroid);
				if (!AwayFromOtherChild.IsNearlyZero())
				{
					DesiredVelocityDirection += AwayFromOtherChild.GetSafeNormal();
				}
			}

			if (DesiredVelocityDirection.IsNearlyZero() && !ParentCentroid.IsNearlyZero())
			{
				DesiredVelocityDirection =
					ProjectOntoTangentPlane(ChildCentroid - ParentCentroid, ChildCentroid);
			}

			if (DesiredVelocityDirection.IsNearlyZero())
			{
				return false;
			}

			const FVector3d DivergentAxis =
				FVector3d::CrossProduct(
					ChildCentroid,
					DesiredVelocityDirection.GetSafeNormal()).GetSafeNormal();
			if (DivergentAxis.IsNearlyZero() || !IsFiniteVector(DivergentAxis))
			{
				return false;
			}

			PureDivergentAxes.Add(DivergentAxis);
		}

		OutChildAxes = PureDivergentAxes;
		for (int32 ChildIndex = 0; ChildIndex < OutChildAxes.Num(); ++ChildIndex)
		{
			if (NormalizedParentAxis.IsNearlyZero())
			{
				continue;
			}

			const FVector3d BlendedAxis =
				((0.10 * NormalizedParentAxis) + (0.90 * OutChildAxes[ChildIndex])).GetSafeNormal();
			if (!BlendedAxis.IsNearlyZero() && IsFiniteVector(BlendedAxis))
			{
				OutChildAxes[ChildIndex] = BlendedAxis;
			}
		}

		bool bNeedPureFallback = false;
		for (int32 ChildIndex = 0; ChildIndex < OutChildAxes.Num(); ++ChildIndex)
		{
			for (int32 OtherChildIndex = ChildIndex + 1; OtherChildIndex < OutChildAxes.Num(); ++OtherChildIndex)
			{
				if (OutChildAxes[ChildIndex].Dot(OutChildAxes[OtherChildIndex]) > 0.995)
				{
					bNeedPureFallback = true;
					break;
				}
			}

			if (bNeedPureFallback)
			{
				break;
			}
		}

		if (bNeedPureFallback)
		{
			OutChildAxes = PureDivergentAxes;
		}

		for (const FVector3d& ChildAxis : OutChildAxes)
		{
			if (ChildAxis.IsNearlyZero() || !IsFiniteVector(ChildAxis))
			{
				return false;
			}
		}

		return true;
	}

	int32 CountAndeanSamples(const FTectonicPlanet& Planet)
	{
		int32 AndeanCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			AndeanCount += Sample.OrogenyType == EOrogenyType::Andean ? 1 : 0;
		}
		return AndeanCount;
	}

	FVector3d ClampAndNormalizeBarycentric(const FVector3d& RawBarycentric)
	{
		FVector3d Clamped(
			FMath::Clamp(RawBarycentric.X, 0.0, 1.0),
			FMath::Clamp(RawBarycentric.Y, 0.0, 1.0),
			FMath::Clamp(RawBarycentric.Z, 0.0, 1.0));
		const double Sum = Clamped.X + Clamped.Y + Clamped.Z;
		if (Sum > UE_DOUBLE_SMALL_NUMBER)
		{
			Clamped /= Sum;
		}
		return Clamped;
	}

	double ComputeContainmentScore(const FVector3d& Barycentric)
	{
		return FMath::Min3(Barycentric.X, Barycentric.Y, Barycentric.Z);
	}

	bool IsIdentityRotation(const FQuat4d& Rotation)
	{
		return
			FMath::IsNearlyEqual(Rotation.X, 0.0, 1.0e-9) &&
			FMath::IsNearlyEqual(Rotation.Y, 0.0, 1.0e-9) &&
			FMath::IsNearlyEqual(Rotation.Z, 0.0, 1.0e-9) &&
			FMath::IsNearlyEqual(Rotation.W, 1.0, 1.0e-9);
	}

	FVector3d ProjectOntoTangent(const FVector3d& Vector, const FVector3d& Normal)
	{
		return Vector - (Vector.Dot(Normal) * Normal);
	}

	int32 PickDominantBarycentricIndex(const FVector3d& Barycentric)
	{
		if (Barycentric.Y > Barycentric.X && Barycentric.Y >= Barycentric.Z)
		{
			return 1;
		}

		if (Barycentric.Z > Barycentric.X && Barycentric.Z > Barycentric.Y)
		{
			return 2;
		}

		return 0;
	}

	EOrogenyType MajorityOrogenyType(
		const EOrogenyType A,
		const EOrogenyType B,
		const EOrogenyType C,
		const FVector3d& Barycentric)
	{
		if (A == B || A == C)
		{
			return A;
		}
		if (B == C)
		{
			return B;
		}

		switch (PickDominantBarycentricIndex(Barycentric))
		{
		case 1:
			return B;
		case 2:
			return C;
		default:
			return A;
		}
	}

	FVector3d InterpolateDirection(
		const FVector3d& A,
		const FVector3d& B,
		const FVector3d& C,
		const FVector3d& Barycentric,
		const FVector3d& SurfaceNormal)
	{
		const FVector3d Blended =
			(Barycentric.X * A) +
			(Barycentric.Y * B) +
			(Barycentric.Z * C);
		const FVector3d Projected = ProjectOntoTangent(Blended, SurfaceNormal);
		if (Projected.SquaredLength() >= DirectionDegeneracyThreshold * DirectionDegeneracyThreshold)
		{
			return Projected.GetSafeNormal();
		}

		switch (PickDominantBarycentricIndex(Barycentric))
		{
		case 1:
		{
			const FVector3d Fallback = ProjectOntoTangent(B, SurfaceNormal);
			return Fallback.SquaredLength() > DirectionDegeneracyThreshold * DirectionDegeneracyThreshold
				? Fallback.GetSafeNormal()
				: FVector3d::ZeroVector;
		}
		case 2:
		{
			const FVector3d Fallback = ProjectOntoTangent(C, SurfaceNormal);
			return Fallback.SquaredLength() > DirectionDegeneracyThreshold * DirectionDegeneracyThreshold
				? Fallback.GetSafeNormal()
				: FVector3d::ZeroVector;
		}
		default:
		{
			const FVector3d Fallback = ProjectOntoTangent(A, SurfaceNormal);
			return Fallback.SquaredLength() > DirectionDegeneracyThreshold * DirectionDegeneracyThreshold
				? Fallback.GetSafeNormal()
				: FVector3d::ZeroVector;
		}
		}
	}

	FVector3d ComputePlateSurfaceVelocity(const FPlate& Plate, const FVector3d& QueryPoint, const double PlanetRadius)
	{
		if (Plate.RotationAxis.IsNearlyZero() || Plate.AngularSpeed <= 0.0)
		{
			return FVector3d::ZeroVector;
		}

		return FVector3d::CrossProduct(Plate.RotationAxis.GetSafeNormal(), QueryPoint) * (Plate.AngularSpeed * PlanetRadius);
	}

	bool IsPointInsideSphericalTriangleRobust(const FVector3d& A, const FVector3d& B, const FVector3d& C, const FVector3d& P)
	{
		const double Origin[3] = { 0.0, 0.0, 0.0 };
		const double AD[3] = { A.X, A.Y, A.Z };
		const double BD[3] = { B.X, B.Y, B.Z };
		const double CD[3] = { C.X, C.Y, C.Z };
		const double PD[3] = { P.X, P.Y, P.Z };
		const double TriangleOrientation = orient3d(Origin, AD, BD, CD);
		if (TriangleOrientation == 0.0)
		{
			return false;
		}

		const double S0 = orient3d(Origin, AD, BD, PD);
		const double S1 = orient3d(Origin, BD, CD, PD);
		const double S2 = orient3d(Origin, CD, AD, PD);
		if (TriangleOrientation > 0.0)
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

		while (!NodeStack.IsEmpty())
		{
			const int32 NodeIndex = NodeStack.Pop(EAllowShrinking::No);
			if (!BVH.box_contains(NodeIndex, QueryPoint))
			{
				continue;
			}

			const int32 ListStartIndex = BVH.BoxToIndex[NodeIndex];
			if (ListStartIndex < BVH.TrianglesEnd)
			{
				const int32 TriangleCount = BVH.IndexList[ListStartIndex];
				for (int32 TriangleListIndex = 1; TriangleListIndex <= TriangleCount; ++TriangleListIndex)
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
				const double DistanceA = BVH.BoxDistanceSqr(ChildA, QueryPoint);
				const double DistanceB = BVH.BoxDistanceSqr(ChildB, QueryPoint);
				if (DistanceA <= DistanceB)
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

	struct FPlateVoteSummary
	{
		int32 PlateId = INDEX_NONE;
		int32 VoteCount = 0;
		double ContinentalWeightSum = 0.0;
		FVector3d TangentSum = FVector3d::ZeroVector;
	};

	enum class EGapDisposition : uint8
	{
		None,
		Divergent,
		NonDivergent
	};

	struct FGapResolutionDecision
	{
		EGapDisposition Disposition = EGapDisposition::None;
		int32 PrimaryPlateId = INDEX_NONE;
		int32 SecondaryPlateId = INDEX_NONE;
		bool bWasContinental = false;
	};

	struct FRecoveredContainmentHit
	{
		int32 PlateId = INDEX_NONE;
		int32 GlobalTriangleIndex = INDEX_NONE;
		FVector3d Barycentric = FVector3d(-1.0, -1.0, -1.0);
		double Distance = TNumericLimits<double>::Max();
	};

	void GatherNeighborPlateSummaries(
		const FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const TArray<int32>& PlateAssignments,
		TArray<FPlateVoteSummary, TInlineAllocator<8>>& OutSummaries)
	{
		OutSummaries.Reset();
		TMap<int32, int32> PlateToSummaryIndex;
		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;

		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!PlateAssignments.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 PlateId = PlateAssignments[NeighborIndex];
			if (FindPlateById(Planet, PlateId) == nullptr)
			{
				continue;
			}

			int32 SummaryIndex = INDEX_NONE;
			if (const int32* ExistingIndex = PlateToSummaryIndex.Find(PlateId))
			{
				SummaryIndex = *ExistingIndex;
			}
			else
			{
				SummaryIndex = OutSummaries.AddDefaulted();
				OutSummaries[SummaryIndex].PlateId = PlateId;
				PlateToSummaryIndex.Add(PlateId, SummaryIndex);
			}

			FPlateVoteSummary& Summary = OutSummaries[SummaryIndex];
			++Summary.VoteCount;
			Summary.ContinentalWeightSum += Planet.Samples[NeighborIndex].ContinentalWeight;
			const FVector3d TangentDirection =
				ProjectOntoTangent(Planet.Samples[NeighborIndex].Position - QueryPoint, QueryPoint);
			if (!TangentDirection.IsNearlyZero())
			{
				Summary.TangentSum += TangentDirection.GetSafeNormal();
			}
		}

		Algo::Sort(OutSummaries, [](const FPlateVoteSummary& Left, const FPlateVoteSummary& Right)
		{
			if (Left.VoteCount != Right.VoteCount)
			{
				return Left.VoteCount > Right.VoteCount;
			}
			return Left.PlateId < Right.PlateId;
		});
	}

	const FPlateVoteSummary* FindPlateVoteSummary(
		const TArray<FPlateVoteSummary, TInlineAllocator<8>>& Summaries,
		const int32 PlateId)
	{
		for (const FPlateVoteSummary& Summary : Summaries)
		{
			if (Summary.PlateId == PlateId)
			{
				return &Summary;
			}
		}

		return nullptr;
	}

	void ClassifyOverlapsImpl(
		const FTectonicPlanet& Planet,
		const TArray<uint8>& OverlapFlags,
		const TArray<int32>& NewPlateIds,
		const TArray<TArray<int32>>* OverlapPlateIds,
		FResamplingStats& Stats)
	{
		check(OverlapFlags.Num() == Planet.Samples.Num());
		check(NewPlateIds.Num() == Planet.Samples.Num());

		TArray<int32> CurrentPlateAssignments;
		CurrentPlateAssignments.Reserve(Planet.Samples.Num());
		for (const FSample& Sample : Planet.Samples)
		{
			CurrentPlateAssignments.Add(Sample.PlateId);
		}

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (OverlapFlags[SampleIndex] == 0)
			{
				continue;
			}

			const int32 WinningPlateId = NewPlateIds[SampleIndex];
			const FPlate* WinningPlate = FindPlateById(Planet, WinningPlateId);
			if (WinningPlate == nullptr)
			{
				++Stats.NonConvergentOverlapCount;
				continue;
			}

			TArray<FPlateVoteSummary, TInlineAllocator<8>> CurrentNeighborSummaries;
			GatherNeighborPlateSummaries(Planet, SampleIndex, CurrentPlateAssignments, CurrentNeighborSummaries);

			TArray<FPlateVoteSummary, TInlineAllocator<8>> ResolvedNeighborSummaries;
			bool bResolvedNeighborSummariesBuilt = false;

			auto FindSummaryForPlate = [&](const int32 PlateId) -> const FPlateVoteSummary*
			{
				if (const FPlateVoteSummary* Summary = FindPlateVoteSummary(CurrentNeighborSummaries, PlateId))
				{
					return Summary;
				}

				if (!bResolvedNeighborSummariesBuilt)
				{
					GatherNeighborPlateSummaries(Planet, SampleIndex, NewPlateIds, ResolvedNeighborSummaries);
					bResolvedNeighborSummariesBuilt = true;
				}

				return FindPlateVoteSummary(ResolvedNeighborSummaries, PlateId);
			};

			const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
			const FVector3d WinningVelocity =
				ComputePlateSurfaceVelocity(*WinningPlate, QueryPoint, Planet.PlanetRadiusKm);
			const bool bWinningPlateContinental = Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f;
			const FPlateVoteSummary* WinningSummary = FindSummaryForPlate(WinningPlateId);

			enum class EOverlapClassification : uint8
			{
				None,
				OceanicOceanic,
				OceanicContinental,
				ContinentalContinental
			};

			EOverlapClassification BestClassification = EOverlapClassification::None;
			int32 BestVoteCount = -1;
			double BestConvergenceMagnitude = -1.0;
			int32 BestOpposingPlateId = INDEX_NONE;
			TArray<int32, TInlineAllocator<8>> CandidateOpposingPlateIds;

			if (OverlapPlateIds != nullptr && OverlapPlateIds->IsValidIndex(SampleIndex))
			{
				for (const int32 PlateId : (*OverlapPlateIds)[SampleIndex])
				{
					if (PlateId != WinningPlateId)
					{
						CandidateOpposingPlateIds.AddUnique(PlateId);
					}
				}
			}

			if (CandidateOpposingPlateIds.IsEmpty())
			{
				for (const FPlateVoteSummary& Summary : CurrentNeighborSummaries)
				{
					if (Summary.PlateId != WinningPlateId)
					{
						CandidateOpposingPlateIds.Add(Summary.PlateId);
					}
				}
			}

			if (CandidateOpposingPlateIds.IsEmpty())
			{
				if (!bResolvedNeighborSummariesBuilt)
				{
					GatherNeighborPlateSummaries(Planet, SampleIndex, NewPlateIds, ResolvedNeighborSummaries);
					bResolvedNeighborSummariesBuilt = true;
				}

				for (const FPlateVoteSummary& Summary : ResolvedNeighborSummaries)
				{
					if (Summary.PlateId != WinningPlateId)
					{
						CandidateOpposingPlateIds.Add(Summary.PlateId);
					}
				}
			}

			for (const int32 OpposingPlateId : CandidateOpposingPlateIds)
			{
				const FPlate* OpposingPlate = FindPlateById(Planet, OpposingPlateId);
				if (OpposingPlate == nullptr)
				{
					continue;
				}

				const FPlateVoteSummary* Summary = FindSummaryForPlate(OpposingPlateId);
				if (Summary == nullptr || Summary->VoteCount <= 0)
				{
					continue;
				}

				FVector3d BoundaryNormal = FVector3d::ZeroVector;
				if (WinningSummary != nullptr &&
					WinningSummary->VoteCount > 0 &&
					!WinningSummary->TangentSum.IsNearlyZero())
				{
					BoundaryNormal =
						ProjectOntoTangent(WinningSummary->TangentSum - Summary->TangentSum, QueryPoint).GetSafeNormal();
				}

				if (BoundaryNormal.IsNearlyZero())
				{
					BoundaryNormal = ProjectOntoTangent(-Summary->TangentSum, QueryPoint).GetSafeNormal();
				}
				if (BoundaryNormal.IsNearlyZero())
				{
					continue;
				}

				const FVector3d OpposingVelocity =
					ComputePlateSurfaceVelocity(*OpposingPlate, QueryPoint, Planet.PlanetRadiusKm);
				const double RelativeNormalVelocity = (WinningVelocity - OpposingVelocity).Dot(BoundaryNormal);
				const double ConvergenceMagnitude = FMath::Abs(RelativeNormalVelocity);
				if (ConvergenceMagnitude <= UE_DOUBLE_SMALL_NUMBER)
				{
					continue;
				}

				const bool bOpposingPlateContinental =
					(Summary->ContinentalWeightSum / static_cast<double>(Summary->VoteCount)) >= 0.5;

				EOverlapClassification Classification = EOverlapClassification::OceanicContinental;
				if (!bWinningPlateContinental && !bOpposingPlateContinental)
				{
					Classification = EOverlapClassification::OceanicOceanic;
				}
				else if (bWinningPlateContinental && bOpposingPlateContinental)
				{
					Classification = EOverlapClassification::ContinentalContinental;
				}

				if (Summary->VoteCount > BestVoteCount ||
					(Summary->VoteCount == BestVoteCount && ConvergenceMagnitude > BestConvergenceMagnitude + 1.0e-12) ||
					(Summary->VoteCount == BestVoteCount &&
						FMath::IsNearlyEqual(ConvergenceMagnitude, BestConvergenceMagnitude, 1.0e-12) &&
						(BestOpposingPlateId == INDEX_NONE || OpposingPlateId < BestOpposingPlateId)))
				{
					BestClassification = Classification;
					BestVoteCount = Summary->VoteCount;
					BestConvergenceMagnitude = ConvergenceMagnitude;
					BestOpposingPlateId = OpposingPlateId;
				}
			}

			switch (BestClassification)
			{
			case EOverlapClassification::OceanicOceanic:
				++Stats.OceanicOceanicOverlapCount;
				break;
			case EOverlapClassification::OceanicContinental:
				++Stats.OceanicContinentalOverlapCount;
				break;
			case EOverlapClassification::ContinentalContinental:
				++Stats.ContinentalContinentalOverlapCount;
				break;
			case EOverlapClassification::None:
			default:
				++Stats.NonConvergentOverlapCount;
				break;
			}
		}
	}

	const FCarriedSample* FindCarriedSampleForCanonicalVertex(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 CanonicalSampleIndex)
	{
		if (!Planet.Samples.IsValidIndex(CanonicalSampleIndex))
		{
			return nullptr;
		}

		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr)
		{
			return nullptr;
		}
		const int32* CarriedIndexPtr = Plate->CanonicalToCarriedIndex.Find(CanonicalSampleIndex);
		if (CarriedIndexPtr == nullptr)
		{
			return nullptr;
		}

		const int32 CarriedIndex = *CarriedIndexPtr;
		if (!Plate->CarriedSamples.IsValidIndex(CarriedIndex))
		{
			return nullptr;
		}

		const FCarriedSample& CarriedSample = Plate->CarriedSamples[CarriedIndex];
		return CarriedSample.CanonicalSampleIndex == CanonicalSampleIndex
			? &CarriedSample
			: nullptr;
	}

	bool TryComputeRecoveredContinentalWeight(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 GlobalTriangleIndex,
		const FVector3d& RawBarycentric,
		double& OutContinentalWeight)
	{
		OutContinentalWeight = 0.0;
		if (FindPlateById(Planet, PlateId) == nullptr || !Planet.TriangleIndices.IsValidIndex(GlobalTriangleIndex))
		{
			return false;
		}

		const FIntVector& Triangle = Planet.TriangleIndices[GlobalTriangleIndex];
		const FCarriedSample* V0 = FindCarriedSampleForCanonicalVertex(Planet, PlateId, Triangle.X);
		const FCarriedSample* V1 = FindCarriedSampleForCanonicalVertex(Planet, PlateId, Triangle.Y);
		const FCarriedSample* V2 = FindCarriedSampleForCanonicalVertex(Planet, PlateId, Triangle.Z);
		if (V0 == nullptr || V1 == nullptr || V2 == nullptr)
		{
			return false;
		}

		const FVector3d Barycentric = ClampAndNormalizeBarycentric(RawBarycentric);
		OutContinentalWeight =
			(Barycentric.X * static_cast<double>(V0->ContinentalWeight)) +
			(Barycentric.Y * static_cast<double>(V1->ContinentalWeight)) +
			(Barycentric.Z * static_cast<double>(V2->ContinentalWeight));
		return true;
	}

	bool TryComputeContainedPlateContinentalWeight(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		const FVector3d& QueryPoint,
		double& OutContinentalWeight)
	{
		OutContinentalWeight = 0.0;
		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr)
		{
			return false;
		}
		if (Plate->SoupData.LocalTriangles.IsEmpty())
		{
			return false;
		}

		int32 LocalTriangleId = INDEX_NONE;
		FVector3d A;
		FVector3d B;
		FVector3d C;
		if (!FindContainingTriangleInBVH(Plate->SoupBVH, Plate->SoupAdapter, QueryPoint, LocalTriangleId, A, B, C))
		{
			return false;
		}

		const FVector3d Barycentric = NormalizeBarycentric(ComputePlanarBarycentric(A, B, C, QueryPoint));
		if (ComputeContainmentScore(Barycentric) <= OverlapScoreEpsilon ||
			!Plate->SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId))
		{
			return false;
		}

		return TryComputeRecoveredContinentalWeight(
			Planet,
			PlateId,
			Plate->SoupData.GlobalTriangleIndices[LocalTriangleId],
			Barycentric,
			OutContinentalWeight);
	}

	bool TryFindNearestTriangleRecoveryHit(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		const FVector3d& QueryPoint,
		FRecoveredContainmentHit& OutHit)
	{
		OutHit = FRecoveredContainmentHit{};
		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr)
		{
			return false;
		}
		if (Plate->SoupBVH.RootIndex < 0 || Plate->SoupData.LocalTriangles.IsEmpty())
		{
			return false;
		}

		double DistanceSqr = TNumericLimits<double>::Max();
		const int32 LocalTriangleId = Plate->SoupBVH.FindNearestTriangle(QueryPoint, DistanceSqr);
		if (!Plate->SoupData.LocalTriangles.IsValidIndex(LocalTriangleId) ||
			!Plate->SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId))
		{
			return false;
		}

		FVector3d A;
		FVector3d B;
		FVector3d C;
		Plate->SoupAdapter.GetTriVertices(LocalTriangleId, A, B, C);
		const UE::Geometry::TTriangle3<double> Triangle(A, B, C);
		UE::Geometry::FDistPoint3Triangle3d DistanceQuery(QueryPoint, Triangle);
		DistanceQuery.ComputeResult();

		OutHit.PlateId = Plate->Id;
		OutHit.GlobalTriangleIndex = Plate->SoupData.GlobalTriangleIndices[LocalTriangleId];
		OutHit.Barycentric = ClampAndNormalizeBarycentric(FVector3d(
			DistanceQuery.TriangleBaryCoords.X,
			DistanceQuery.TriangleBaryCoords.Y,
			DistanceQuery.TriangleBaryCoords.Z));
		OutHit.Distance = FMath::Sqrt(FMath::Max(DistanceSqr, 0.0));
		return true;
	}

	bool TryFindNearestTriangleRecoveryHit(
		const FTectonicPlanet& Planet,
		const FVector3d& QueryPoint,
		FRecoveredContainmentHit& OutHit)
	{
		OutHit = FRecoveredContainmentHit{};

		bool bFoundHit = false;
		FRecoveredContainmentHit BestHit;
		for (const FPlate& Plate : Planet.Plates)
		{
			FRecoveredContainmentHit CandidateHit;
			if (!TryFindNearestTriangleRecoveryHit(Planet, Plate.Id, QueryPoint, CandidateHit))
			{
				continue;
			}

			const bool bCloser = CandidateHit.Distance + TriangleEpsilon < BestHit.Distance;
			const bool bTie = FMath::IsNearlyEqual(CandidateHit.Distance, BestHit.Distance, TriangleEpsilon);
			if (bFoundHit && !bCloser && !(bTie && CandidateHit.PlateId < BestHit.PlateId))
			{
				continue;
			}

			BestHit = CandidateHit;
			bFoundHit = true;
		}

		if (!bFoundHit)
		{
			return false;
		}

		OutHit = BestHit;
		return true;
	}

	FVector3d ProjectDirectionToSurface(const FVector3d& Direction, const FVector3d& SurfaceNormal)
	{
		const FVector3d Projected = ProjectOntoTangent(Direction, SurfaceNormal);
		return Projected.SquaredLength() > DirectionDegeneracyThreshold * DirectionDegeneracyThreshold
			? Projected.GetSafeNormal()
			: FVector3d::ZeroVector;
	}

	void ApplyCarriedSampleAttributes(
		FSample& Sample,
		const FVector3d& SurfaceNormal,
		const FCarriedSample& Source,
		float& OutSubductionDistanceKm,
		float& OutSubductionSpeed)
	{
		Sample.ContinentalWeight = FMath::Clamp(Source.ContinentalWeight, 0.0f, 1.0f);
		Sample.Elevation = FMath::Clamp(Source.Elevation, static_cast<float>(TrenchElevationKm), static_cast<float>(ElevationCeilingKm));
		Sample.Thickness = FMath::Max(0.0f, Source.Thickness);
		Sample.Age = FMath::Max(0.0f, Source.Age);
		Sample.OrogenyType = Source.OrogenyType;
		Sample.TerraneId = Source.TerraneId;
		Sample.RidgeDirection = ProjectDirectionToSurface(Source.RidgeDirection, SurfaceNormal);
		Sample.FoldDirection = ProjectDirectionToSurface(Source.FoldDirection, SurfaceNormal);
		OutSubductionDistanceKm = Source.SubductionDistanceKm;
		OutSubductionSpeed = Source.SubductionSpeed;
	}

	void ApplyInterpolatedCarriedAttributes(
		FSample& Sample,
		const FVector3d& SurfaceNormal,
		const FCarriedSample& V0,
		const FCarriedSample& V1,
		const FCarriedSample& V2,
		const FVector3d& RawBarycentric,
		float& OutSubductionDistanceKm,
		float& OutSubductionSpeed)
	{
		const FVector3d Barycentric = NormalizeBarycentric(RawBarycentric);

		Sample.Elevation = static_cast<float>(
			(Barycentric.X * V0.Elevation) +
			(Barycentric.Y * V1.Elevation) +
			(Barycentric.Z * V2.Elevation));
		Sample.Thickness = static_cast<float>(
			(Barycentric.X * V0.Thickness) +
			(Barycentric.Y * V1.Thickness) +
			(Barycentric.Z * V2.Thickness));
		Sample.Age = static_cast<float>(
			(Barycentric.X * V0.Age) +
			(Barycentric.Y * V1.Age) +
			(Barycentric.Z * V2.Age));
		Sample.ContinentalWeight = static_cast<float>(FMath::Clamp(
			(Barycentric.X * V0.ContinentalWeight) +
			(Barycentric.Y * V1.ContinentalWeight) +
			(Barycentric.Z * V2.ContinentalWeight),
			0.0,
			1.0));
		Sample.OrogenyType = MajorityOrogenyType(V0.OrogenyType, V1.OrogenyType, V2.OrogenyType, Barycentric);
		Sample.RidgeDirection = InterpolateDirection(
			V0.RidgeDirection,
			V1.RidgeDirection,
			V2.RidgeDirection,
			Barycentric,
			SurfaceNormal);
		Sample.FoldDirection = InterpolateDirection(
			V0.FoldDirection,
			V1.FoldDirection,
			V2.FoldDirection,
			Barycentric,
			SurfaceNormal);

		switch (PickDominantBarycentricIndex(Barycentric))
		{
		case 1:
			Sample.TerraneId = V1.TerraneId;
			break;
		case 2:
			Sample.TerraneId = V2.TerraneId;
			break;
		default:
			Sample.TerraneId = V0.TerraneId;
			break;
		}

		Sample.Elevation = FMath::Clamp(Sample.Elevation, static_cast<float>(TrenchElevationKm), static_cast<float>(ElevationCeilingKm));
		Sample.Thickness = FMath::Max(0.0f, Sample.Thickness);
		Sample.Age = FMath::Max(0.0f, Sample.Age);
		OutSubductionDistanceKm = static_cast<float>(
			(Barycentric.X * V0.SubductionDistanceKm) +
			(Barycentric.Y * V1.SubductionDistanceKm) +
			(Barycentric.Z * V2.SubductionDistanceKm));
		OutSubductionSpeed = static_cast<float>(
			(Barycentric.X * V0.SubductionSpeed) +
			(Barycentric.Y * V1.SubductionSpeed) +
			(Barycentric.Z * V2.SubductionSpeed));
	}

	bool TryApplyNearestTriangleProjectionFromPlate(
		FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const int32 PlateId,
		TArray<float>& InOutSubductionDistances,
		TArray<float>& InOutSubductionSpeeds)
	{
		if (!Planet.Samples.IsValidIndex(SampleIndex) ||
			InOutSubductionDistances.Num() != Planet.Samples.Num() ||
			InOutSubductionSpeeds.Num() != Planet.Samples.Num())
		{
			return false;
		}

		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr)
		{
			return false;
		}
		if (Plate->SoupBVH.RootIndex < 0 || Plate->SoupData.LocalTriangles.IsEmpty())
		{
			return false;
		}

		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		double NearestDistanceSqr = TNumericLimits<double>::Max();
		const int32 LocalTriangleId = Plate->SoupBVH.FindNearestTriangle(QueryPoint, NearestDistanceSqr);
		if (!Plate->SoupData.LocalTriangles.IsValidIndex(LocalTriangleId))
		{
			return false;
		}

		const UE::Geometry::FIndex3i LocalTriangle = Plate->SoupData.LocalTriangles[LocalTriangleId];
		if (!Plate->SoupData.LocalToCanonicalVertex.IsValidIndex(LocalTriangle.A) ||
			!Plate->SoupData.LocalToCanonicalVertex.IsValidIndex(LocalTriangle.B) ||
			!Plate->SoupData.LocalToCanonicalVertex.IsValidIndex(LocalTriangle.C))
		{
			return false;
		}

		const int32 CanonicalVertex0 = Plate->SoupData.LocalToCanonicalVertex[LocalTriangle.A];
		const int32 CanonicalVertex1 = Plate->SoupData.LocalToCanonicalVertex[LocalTriangle.B];
		const int32 CanonicalVertex2 = Plate->SoupData.LocalToCanonicalVertex[LocalTriangle.C];
		const FCarriedSample* V0 = FindCarriedSampleForCanonicalVertex(Planet, PlateId, CanonicalVertex0);
		const FCarriedSample* V1 = FindCarriedSampleForCanonicalVertex(Planet, PlateId, CanonicalVertex1);
		const FCarriedSample* V2 = FindCarriedSampleForCanonicalVertex(Planet, PlateId, CanonicalVertex2);
		if (V0 == nullptr || V1 == nullptr || V2 == nullptr)
		{
			return false;
		}

		FVector3d A;
		FVector3d B;
		FVector3d C;
		Plate->SoupAdapter.GetTriVertices(LocalTriangleId, A, B, C);
		const UE::Geometry::TTriangle3<double> Triangle(A, B, C);
		UE::Geometry::FDistPoint3Triangle3d DistanceQuery(QueryPoint, Triangle);
		DistanceQuery.ComputeResult();

		FSample& Sample = Planet.Samples[SampleIndex];
		ApplyInterpolatedCarriedAttributes(
			Sample,
			QueryPoint.GetSafeNormal(),
			*V0,
			*V1,
			*V2,
			FVector3d(
				DistanceQuery.TriangleBaryCoords.X,
				DistanceQuery.TriangleBaryCoords.Y,
				DistanceQuery.TriangleBaryCoords.Z),
			InOutSubductionDistances[SampleIndex],
			InOutSubductionSpeeds[SampleIndex]);
		return true;
	}

	bool TryApplyNearestCarriedCopyFromPlate(
		FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const int32 PlateId,
		TArray<float>& InOutSubductionDistances,
		TArray<float>& InOutSubductionSpeeds)
	{
		if (!Planet.Samples.IsValidIndex(SampleIndex) ||
			InOutSubductionDistances.Num() != Planet.Samples.Num() ||
			InOutSubductionSpeeds.Num() != Planet.Samples.Num())
		{
			return false;
		}

		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr)
		{
			return false;
		}
		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		const FCarriedSample* BestCarriedSample = nullptr;
		double BestDistance = TNumericLimits<double>::Max();

		for (const FCarriedSample& CarriedSample : Plate->CarriedSamples)
		{
			if (!Planet.Samples.IsValidIndex(CarriedSample.CanonicalSampleIndex) ||
				CarriedSample.CanonicalSampleIndex == SampleIndex)
			{
				continue;
			}

			const FVector3d RotatedPosition =
				Plate->CumulativeRotation.RotateVector(Planet.Samples[CarriedSample.CanonicalSampleIndex].Position).GetSafeNormal();
			const double Distance = ComputeGeodesicDistance(QueryPoint, RotatedPosition);
			if (Distance < BestDistance)
			{
				BestDistance = Distance;
				BestCarriedSample = &CarriedSample;
			}
		}

		if (BestCarriedSample == nullptr)
		{
			return false;
		}

		FSample& Sample = Planet.Samples[SampleIndex];
		ApplyCarriedSampleAttributes(
			Sample,
			QueryPoint.GetSafeNormal(),
			*BestCarriedSample,
			InOutSubductionDistances[SampleIndex],
			InOutSubductionSpeeds[SampleIndex]);
		return true;
	}

	bool FindNearestMemberSample(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		const FVector3d& QueryPoint,
		int32& OutCanonicalSampleIndex,
		double& OutGeodesicDistanceRadians)
	{
		OutCanonicalSampleIndex = INDEX_NONE;
		OutGeodesicDistanceRadians = TNumericLimits<double>::Max();
		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr)
		{
			return false;
		}
		for (const int32 MemberSampleIndex : Plate->MemberSamples)
		{
			if (!Planet.Samples.IsValidIndex(MemberSampleIndex))
			{
				continue;
			}

			const FVector3d RotatedPosition =
				Plate->CumulativeRotation.RotateVector(Planet.Samples[MemberSampleIndex].Position).GetSafeNormal();
			const double Distance = ComputeGeodesicDistance(QueryPoint, RotatedPosition);
			if (Distance < OutGeodesicDistanceRadians)
			{
				OutGeodesicDistanceRadians = Distance;
				OutCanonicalSampleIndex = MemberSampleIndex;
			}
		}

		return OutCanonicalSampleIndex != INDEX_NONE;
	}

	bool FindNearestNeighborForPlate(
		const FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const TArray<int32>& PlateAssignments,
		const int32 PlateId,
		int32& OutNeighborIndex,
		double& OutDistanceRadians)
	{
		OutNeighborIndex = INDEX_NONE;
		OutDistanceRadians = TNumericLimits<double>::Max();
		if (FindPlateById(Planet, PlateId) == nullptr)
		{
			return false;
		}

		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!PlateAssignments.IsValidIndex(NeighborIndex) || PlateAssignments[NeighborIndex] != PlateId)
			{
				continue;
			}

			const double Distance = ComputeGeodesicDistance(QueryPoint, Planet.Samples[NeighborIndex].Position);
			if (Distance < OutDistanceRadians)
			{
				OutDistanceRadians = Distance;
				OutNeighborIndex = NeighborIndex;
			}
		}

		return OutNeighborIndex != INDEX_NONE;
	}

	void PopulateCarriedSampleFromCanonical(
		const FTectonicPlanet& Planet,
		const int32 CanonicalSampleIndex,
		FCarriedSample& OutCarriedSample)
	{
		if (!Planet.Samples.IsValidIndex(CanonicalSampleIndex))
		{
			return;
		}

		const FSample& Sample = Planet.Samples[CanonicalSampleIndex];
		OutCarriedSample.CanonicalSampleIndex = CanonicalSampleIndex;
		OutCarriedSample.ContinentalWeight = Sample.ContinentalWeight;
		OutCarriedSample.Elevation = Sample.Elevation;
		OutCarriedSample.Thickness = Sample.Thickness;
		OutCarriedSample.Age = Sample.Age;
		OutCarriedSample.RidgeDirection = Sample.RidgeDirection;
		OutCarriedSample.FoldDirection = Sample.FoldDirection;
		OutCarriedSample.OrogenyType = Sample.OrogenyType;
		OutCarriedSample.TerraneId = Sample.TerraneId;
		OutCarriedSample.SubductionDistanceKm = Sample.SubductionDistanceKm;
	}

	void ResetSoupState(FPlate& Plate)
	{
		Plate.SoupData = FPlateTriangleSoupData{};
		Plate.SoupAdapter = FPlateTriangleSoupAdapter{};
		Plate.SoupBVH = FPlateTriangleSoupBVH{};
	}

	void SyncCanonicalAttributesFromCarried(FTectonicPlanet& Planet)
	{
		for (const FPlate& Plate : Planet.Plates)
		{
			for (const FCarriedSample& CarriedSample : Plate.CarriedSamples)
			{
				if (!Planet.Samples.IsValidIndex(CarriedSample.CanonicalSampleIndex))
				{
					continue;
				}

				FSample& CanonicalSample = Planet.Samples[CarriedSample.CanonicalSampleIndex];
				if (CanonicalSample.PlateId != Plate.Id)
				{
					continue;
				}

				CanonicalSample.ContinentalWeight = CarriedSample.ContinentalWeight;
				CanonicalSample.Elevation = CarriedSample.Elevation;
				CanonicalSample.Thickness = CarriedSample.Thickness;
				CanonicalSample.Age = CarriedSample.Age;
				CanonicalSample.RidgeDirection = CarriedSample.RidgeDirection;
				CanonicalSample.FoldDirection = CarriedSample.FoldDirection;
				CanonicalSample.OrogenyType = CarriedSample.OrogenyType;
				CanonicalSample.TerraneId = CarriedSample.TerraneId;
				CanonicalSample.SubductionDistanceKm = CarriedSample.SubductionDistanceKm;
			}
		}
	}

	void RebuildMembershipFromCanonical(FTectonicPlanet& Planet)
	{
		for (FPlate& Plate : Planet.Plates)
		{
			Plate.MemberSamples.Reset();
		}

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const int32 PlateId = Planet.Samples[SampleIndex].PlateId;
			if (FPlate* Plate = FindPlateById(Planet, PlateId))
			{
				Plate->MemberSamples.Add(SampleIndex);
			}
		}
	}

	bool IsInteriorTriangleForPlateAssignments(const FTectonicPlanet& Planet, const FIntVector& Triangle)
	{
		if (!Planet.Samples.IsValidIndex(Triangle.X) ||
			!Planet.Samples.IsValidIndex(Triangle.Y) ||
			!Planet.Samples.IsValidIndex(Triangle.Z))
		{
			return false;
		}

		const int32 PlateA = Planet.Samples[Triangle.X].PlateId;
		const int32 PlateB = Planet.Samples[Triangle.Y].PlateId;
		const int32 PlateC = Planet.Samples[Triangle.Z].PlateId;
		return FindPlateById(Planet, PlateA) != nullptr && PlateA == PlateB && PlateB == PlateC;
	}

	int32 ChooseSoupPlateIdForTriangle(const FTectonicPlanet& Planet, const FIntVector& Triangle)
	{
		TArray<TPair<int32, int32>, TInlineAllocator<3>> PlateCounts;
		auto AccumulatePlate = [&Planet, &PlateCounts](const int32 PlateId)
		{
			if (FindPlateById(Planet, PlateId) == nullptr)
			{
				return;
			}

			for (TPair<int32, int32>& PlateCount : PlateCounts)
			{
				if (PlateCount.Key == PlateId)
				{
					++PlateCount.Value;
					return;
				}
			}

			PlateCounts.Emplace(PlateId, 1);
		};

		AccumulatePlate(Planet.Samples.IsValidIndex(Triangle.X) ? Planet.Samples[Triangle.X].PlateId : INDEX_NONE);
		AccumulatePlate(Planet.Samples.IsValidIndex(Triangle.Y) ? Planet.Samples[Triangle.Y].PlateId : INDEX_NONE);
		AccumulatePlate(Planet.Samples.IsValidIndex(Triangle.Z) ? Planet.Samples[Triangle.Z].PlateId : INDEX_NONE);

		int32 AssignedPlateId = INDEX_NONE;
		int32 BestCount = -1;
		for (const TPair<int32, int32>& PlateCount : PlateCounts)
		{
			if (PlateCount.Value > BestCount ||
				(PlateCount.Value == BestCount &&
					(AssignedPlateId == INDEX_NONE || PlateCount.Key < AssignedPlateId)))
			{
				AssignedPlateId = PlateCount.Key;
				BestCount = PlateCount.Value;
			}
		}

		return AssignedPlateId;
	}

	void RebuildCarriedSamplesFromSoupVertices(FTectonicPlanet& Planet)
	{
		for (FPlate& Plate : Planet.Plates)
		{
			Plate.CarriedSamples.Reset();
			Plate.CanonicalToCarriedIndex.Reset();
			Plate.CarriedSamples.Reserve(Plate.SoupTriangles.Num() * 3);
			for (const int32 TriangleIndex : Plate.SoupTriangles)
			{
				if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
				{
					continue;
				}

				const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
				const int32 CanonicalVertices[3] = { Triangle.X, Triangle.Y, Triangle.Z };
				for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
				{
					const int32 CanonicalSampleIndex = CanonicalVertices[CornerIndex];
					if (!Planet.Samples.IsValidIndex(CanonicalSampleIndex) ||
						Plate.CanonicalToCarriedIndex.Contains(CanonicalSampleIndex))
					{
						continue;
					}

					FCarriedSample CarriedSample;
					PopulateCarriedSampleFromCanonical(Planet, CanonicalSampleIndex, CarriedSample);
					const int32 CarriedIndex = Plate.CarriedSamples.Add(CarriedSample);
					Plate.CanonicalToCarriedIndex.Add(CanonicalSampleIndex, CarriedIndex);
				}
			}
		}
	}

	void PopulateSoupPartitionDiagnostics(const FTectonicPlanet& Planet, FResamplingStats& OutStats)
	{
		OutStats.InteriorSoupTriangleCount = 0;
		OutStats.BoundarySoupTriangleCount = 0;
		OutStats.TotalSoupTriangleCount = 0;
		OutStats.DroppedMixedTriangleCount = 0;
		OutStats.DuplicatedSoupTriangleCount = 0;
		OutStats.ForeignLocalVertexCount = 0;

		TArray<uint8> TriangleAssignmentCounts;
		TriangleAssignmentCounts.Init(0, Planet.TriangleIndices.Num());
		for (const FPlate& Plate : Planet.Plates)
		{
			for (const int32 TriangleIndex : Plate.SoupTriangles)
			{
				++OutStats.TotalSoupTriangleCount;
				if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
				{
					continue;
				}

				++TriangleAssignmentCounts[TriangleIndex];
				if (IsInteriorTriangleForPlateAssignments(Planet, Planet.TriangleIndices[TriangleIndex]))
				{
					++OutStats.InteriorSoupTriangleCount;
				}
				else
				{
					++OutStats.BoundarySoupTriangleCount;
				}
			}

			for (const FCarriedSample& CarriedSample : Plate.CarriedSamples)
			{
				if (!Planet.Samples.IsValidIndex(CarriedSample.CanonicalSampleIndex))
				{
					continue;
				}

				OutStats.ForeignLocalVertexCount +=
					Planet.Samples[CarriedSample.CanonicalSampleIndex].PlateId != Plate.Id ? 1 : 0;
			}
		}

		for (int32 TriangleIndex = 0; TriangleIndex < Planet.TriangleIndices.Num(); ++TriangleIndex)
		{
			if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
			{
				continue;
			}

			const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
			const bool bIsMixedTriangle = !IsInteriorTriangleForPlateAssignments(Planet, Triangle);
			if (bIsMixedTriangle && TriangleAssignmentCounts[TriangleIndex] == 0)
			{
				++OutStats.DroppedMixedTriangleCount;
			}

			if (TriangleAssignmentCounts[TriangleIndex] > 1)
			{
				OutStats.DuplicatedSoupTriangleCount += TriangleAssignmentCounts[TriangleIndex] - 1;
			}
		}
	}

	void RecomputeBoundaryFlags(FTectonicPlanet& Planet)
	{
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			bool bIsBoundary = (Sample.PlateId == INDEX_NONE);
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!Planet.Samples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				if (Planet.Samples[NeighborIndex].PlateId != Sample.PlateId)
				{
					bIsBoundary = true;
					break;
				}
			}
			Sample.bIsBoundary = bIsBoundary;
		}
	}

	void ClassifyPlateTriangles(FTectonicPlanet& Planet)
	{
		for (FPlate& Plate : Planet.Plates)
		{
			Plate.InteriorTriangles.Reset();
			Plate.BoundaryTriangles.Reset();
			Plate.SoupTriangles.Reset();
		}

		for (int32 TriangleIndex = 0; TriangleIndex < Planet.TriangleIndices.Num(); ++TriangleIndex)
		{
			const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
			if (!Planet.Samples.IsValidIndex(Triangle.X) || !Planet.Samples.IsValidIndex(Triangle.Y) || !Planet.Samples.IsValidIndex(Triangle.Z))
			{
				continue;
			}

			const int32 PlateA = Planet.Samples[Triangle.X].PlateId;
			const int32 PlateB = Planet.Samples[Triangle.Y].PlateId;
			const int32 PlateC = Planet.Samples[Triangle.Z].PlateId;
			const int32 AssignedPlateId = ChooseSoupPlateIdForTriangle(Planet, Triangle);
			if (FPlate* AssignedPlate = FindPlateById(Planet, AssignedPlateId))
			{
				AssignedPlate->SoupTriangles.Add(TriangleIndex);
			}

			if (FindPlateById(Planet, PlateA) != nullptr && PlateA == PlateB && PlateB == PlateC)
			{
				if (FPlate* InteriorPlate = FindPlateById(Planet, PlateA))
				{
					InteriorPlate->InteriorTriangles.Add(TriangleIndex);
				}
				continue;
			}

			TArray<int32, TInlineAllocator<3>> InvolvedPlates;
			if (FindPlateById(Planet, PlateA) != nullptr)
			{
				InvolvedPlates.AddUnique(PlateA);
			}
			if (FindPlateById(Planet, PlateB) != nullptr)
			{
				InvolvedPlates.AddUnique(PlateB);
			}
			if (FindPlateById(Planet, PlateC) != nullptr)
			{
				InvolvedPlates.AddUnique(PlateC);
			}

			for (const int32 PlateId : InvolvedPlates)
			{
				if (FPlate* BoundaryPlate = FindPlateById(Planet, PlateId))
				{
					BoundaryPlate->BoundaryTriangles.Add(TriangleIndex);
				}
			}
		}
	}

	void RecomputePlateBoundingCaps(FTectonicPlanet& Planet)
	{
		for (FPlate& Plate : Planet.Plates)
		{
			Plate.BoundingCap = BuildBoundingCapFromMembers(Planet, Plate.MemberSamples);
		}
	}

	struct FTerraneComponent
	{
		int32 PlateId = INDEX_NONE;
		TArray<int32> Members;
		TArray<int32> AnchorTerraneIds;
		TMap<int32, int32> PreviousTerraneMemberCounts;
		FVector3d Centroid = FVector3d::ZeroVector;
		double AreaKm2 = 0.0;
	};

	struct FTerraneDetectionDiagnostics
	{
		int32 NewTerraneCount = 0;
		int32 MergedTerraneCount = 0;
	};

	void ClearCanonicalTerraneAssignments(FTectonicPlanet& Planet)
	{
		for (FSample& Sample : Planet.Samples)
		{
			Sample.TerraneId = INDEX_NONE;
		}
	}

	int32 ChooseAnchorSampleForComponent(const FTectonicPlanet& Planet, const FTerraneComponent& Component)
	{
		if (Component.Members.IsEmpty())
		{
			return INDEX_NONE;
		}

		int32 BestSampleIndex = Component.Members[0];
		double BestDot = -TNumericLimits<double>::Max();
		for (const int32 SampleIndex : Component.Members)
		{
			const double Dot = Planet.Samples[SampleIndex].Position.GetSafeNormal().Dot(Component.Centroid);
			if (Dot > BestDot)
			{
				BestDot = Dot;
				BestSampleIndex = SampleIndex;
			}
		}

		return BestSampleIndex;
	}

	void BuildTerraneComponents(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PreviousTerraneAssignments,
		const TMap<int32, int32>& AnchorTerraneIdsBySample,
		TArray<FTerraneComponent>& OutComponents)
	{
		OutComponents.Reset();

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		const double CellAreaKm2 =
			Planet.Samples.IsEmpty()
				? 0.0
				: (4.0 * PI * Planet.PlanetRadiusKm * Planet.PlanetRadiusKm) / static_cast<double>(Planet.Samples.Num());

		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (Visited[SeedSampleIndex] != 0)
			{
				continue;
			}

			const FSample& SeedSample = Planet.Samples[SeedSampleIndex];
			if (SeedSample.PlateId == INDEX_NONE || SeedSample.ContinentalWeight < 0.5f)
			{
				Visited[SeedSampleIndex] = 1;
				continue;
			}

			FTerraneComponent& Component = OutComponents.AddDefaulted_GetRef();
			Component.PlateId = SeedSample.PlateId;
			Component.AreaKm2 = 0.0;
			FVector3d CentroidSum = FVector3d::ZeroVector;

			TArray<int32, TInlineAllocator<64>> Stack;
			Stack.Add(SeedSampleIndex);
			Visited[SeedSampleIndex] = 1;

			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				Component.Members.Add(SampleIndex);
				CentroidSum += Planet.Samples[SampleIndex].Position;

				if (PreviousTerraneAssignments.IsValidIndex(SampleIndex))
				{
					const int32 PreviousTerraneId = PreviousTerraneAssignments[SampleIndex];
					if (PreviousTerraneId != INDEX_NONE)
					{
						++Component.PreviousTerraneMemberCounts.FindOrAdd(PreviousTerraneId);
					}
				}

				if (const int32* AnchorTerraneId = AnchorTerraneIdsBySample.Find(SampleIndex))
				{
					Component.AnchorTerraneIds.AddUnique(*AnchorTerraneId);
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) || Visited[NeighborIndex] != 0)
					{
						continue;
					}

					const FSample& NeighborSample = Planet.Samples[NeighborIndex];
					if (NeighborSample.PlateId != Component.PlateId || NeighborSample.ContinentalWeight < 0.5f)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			Component.Centroid = CentroidSum.GetSafeNormal();
			if (Component.Centroid.IsNearlyZero())
			{
				Component.Centroid = Planet.Samples[SeedSampleIndex].Position.GetSafeNormal();
			}
			Component.AreaKm2 = static_cast<double>(Component.Members.Num()) * CellAreaKm2;
		}
	}

	int32 ChoosePreservedTerraneIdForMerger(const FTerraneComponent& Component)
	{
		int32 BestTerraneId = INDEX_NONE;
		int32 BestContribution = -1;
		for (const int32 TerraneId : Component.AnchorTerraneIds)
		{
			const int32 Contribution = Component.PreviousTerraneMemberCounts.FindRef(TerraneId);
			if (Contribution > BestContribution || (Contribution == BestContribution && (BestTerraneId == INDEX_NONE || TerraneId < BestTerraneId)))
			{
				BestContribution = Contribution;
				BestTerraneId = TerraneId;
			}
		}

		return BestTerraneId;
	}

	int32 FindBestCentroidFallbackComponent(
		const FTectonicPlanet& Planet,
		const FTerrane& PreviousTerrane,
		const TArray<FTerraneComponent>& Components,
		const TArray<int32>& AssignedTerraneIds)
	{
		int32 BestComponentIndex = INDEX_NONE;
		double BestDistance = TNumericLimits<double>::Max();

		for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
		{
			if (AssignedTerraneIds.IsValidIndex(ComponentIndex) && AssignedTerraneIds[ComponentIndex] != INDEX_NONE)
			{
				continue;
			}

			const FTerraneComponent& Component = Components[ComponentIndex];
			for (const int32 SampleIndex : Component.Members)
			{
				const double Distance = ComputeGeodesicDistance(PreviousTerrane.Centroid, Planet.Samples[SampleIndex].Position);
				if (Distance < BestDistance ||
					(FMath::IsNearlyEqual(Distance, BestDistance, 1.0e-12) &&
						(BestComponentIndex == INDEX_NONE || ComponentIndex < BestComponentIndex)))
				{
					BestDistance = Distance;
					BestComponentIndex = ComponentIndex;
				}
			}
		}

		return BestComponentIndex;
	}

	void SyncCarriedTerraneIdsFromCanonical(FTectonicPlanet& Planet)
	{
		for (FPlate& Plate : Planet.Plates)
		{
			for (FCarriedSample& CarriedSample : Plate.CarriedSamples)
			{
				if (!Planet.Samples.IsValidIndex(CarriedSample.CanonicalSampleIndex))
				{
					continue;
				}

				const FSample& CanonicalSample = Planet.Samples[CarriedSample.CanonicalSampleIndex];
				if (CanonicalSample.PlateId == Plate.Id)
				{
					CarriedSample.TerraneId = CanonicalSample.TerraneId;
				}
			}
		}
	}

	uint64 MakeCollisionPlatePairKey(const int32 OverridingPlateId, const int32 SubductingPlateId)
	{
		return (static_cast<uint64>(static_cast<uint32>(OverridingPlateId)) << 32) |
			static_cast<uint32>(SubductingPlateId);
	}

	int32 ChooseDominantPreviousTerraneId(
		const TArray<int32>& PreviousTerraneAssignments,
		const TArray<int32>& TerraneSampleIndices)
	{
		TMap<int32, int32> TerraneCounts;
		for (const int32 SampleIndex : TerraneSampleIndices)
		{
			if (!PreviousTerraneAssignments.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 PreviousTerraneId = PreviousTerraneAssignments[SampleIndex];
			if (PreviousTerraneId != INDEX_NONE)
			{
				++TerraneCounts.FindOrAdd(PreviousTerraneId);
			}
		}

		int32 BestTerraneId = INDEX_NONE;
		int32 BestCount = -1;
		for (const TPair<int32, int32>& TerraneCount : TerraneCounts)
		{
			if (TerraneCount.Value > BestCount ||
				(TerraneCount.Value == BestCount &&
					(BestTerraneId == INDEX_NONE || TerraneCount.Key < BestTerraneId)))
			{
				BestTerraneId = TerraneCount.Key;
				BestCount = TerraneCount.Value;
			}
		}

		return BestTerraneId;
	}

	void BuildCollisionComponents(
		const FTectonicPlanet& Planet,
		const TArray<FCollisionCandidate>& Candidates,
		TArray<FCollisionComponent>& OutComponents)
	{
		OutComponents.Reset();

		TMap<uint64, TArray<int32>> SamplesByPair;
		for (const FCollisionCandidate& Candidate : Candidates)
		{
			if (!Planet.Samples.IsValidIndex(Candidate.SampleIndex) ||
				FindPlateById(Planet, Candidate.OverridingPlateId) == nullptr ||
				FindPlateById(Planet, Candidate.SubductingPlateId) == nullptr)
			{
				continue;
			}

			SamplesByPair.FindOrAdd(
				MakeCollisionPlatePairKey(Candidate.OverridingPlateId, Candidate.SubductingPlateId)).AddUnique(Candidate.SampleIndex);
		}

		for (TPair<uint64, TArray<int32>>& PairSamples : SamplesByPair)
		{
			TMap<int32, int32> CandidateLocalIndices;
			for (int32 CandidateIndex = 0; CandidateIndex < PairSamples.Value.Num(); ++CandidateIndex)
			{
				CandidateLocalIndices.Add(PairSamples.Value[CandidateIndex], CandidateIndex);
			}

			TArray<int32> Parent;
			Parent.SetNumUninitialized(PairSamples.Value.Num());
			for (int32 CandidateIndex = 0; CandidateIndex < Parent.Num(); ++CandidateIndex)
			{
				Parent[CandidateIndex] = CandidateIndex;
			}

			auto FindRoot = [&Parent](int32 CandidateIndex)
			{
				int32 Root = CandidateIndex;
				while (Parent[Root] != Root)
				{
					Root = Parent[Root];
				}

				while (Parent[CandidateIndex] != CandidateIndex)
				{
					const int32 Next = Parent[CandidateIndex];
					Parent[CandidateIndex] = Root;
					CandidateIndex = Next;
				}

				return Root;
			};

			auto UnionCandidates = [&Parent, &FindRoot](const int32 CandidateIndexA, const int32 CandidateIndexB)
			{
				const int32 RootA = FindRoot(CandidateIndexA);
				const int32 RootB = FindRoot(CandidateIndexB);
				if (RootA != RootB)
				{
					if (RootA < RootB)
					{
						Parent[RootB] = RootA;
					}
					else
					{
						Parent[RootA] = RootB;
					}
				}
			};

			for (int32 SourceCandidateIndex = 0; SourceCandidateIndex < PairSamples.Value.Num(); ++SourceCandidateIndex)
			{
				const int32 SourceSampleIndex = PairSamples.Value[SourceCandidateIndex];
				if (!Planet.SampleAdjacency.IsValidIndex(SourceSampleIndex))
				{
					continue;
				}

				TSet<int32> VisitedSamples;
				TArray<int32, TInlineAllocator<64>> Frontier;
				Frontier.Add(SourceSampleIndex);
				VisitedSamples.Add(SourceSampleIndex);

				for (int32 Hop = 0; Hop < MaxCollisionHops && !Frontier.IsEmpty(); ++Hop)
				{
					TArray<int32, TInlineAllocator<128>> NextFrontier;
					for (const int32 FrontierSampleIndex : Frontier)
					{
						if (!Planet.SampleAdjacency.IsValidIndex(FrontierSampleIndex))
						{
							continue;
						}

						for (const int32 NeighborIndex : Planet.SampleAdjacency[FrontierSampleIndex])
						{
							if (!Planet.Samples.IsValidIndex(NeighborIndex) || VisitedSamples.Contains(NeighborIndex))
							{
								continue;
							}

							VisitedSamples.Add(NeighborIndex);
							NextFrontier.Add(NeighborIndex);

							if (const int32* NeighborCandidateIndex = CandidateLocalIndices.Find(NeighborIndex))
							{
								UnionCandidates(SourceCandidateIndex, *NeighborCandidateIndex);
							}
						}
					}

					Frontier = MoveTemp(NextFrontier);
				}
			}

			TMap<int32, int32> ComponentIndexByRoot;
			for (int32 CandidateIndex = 0; CandidateIndex < PairSamples.Value.Num(); ++CandidateIndex)
			{
				const int32 Root = FindRoot(CandidateIndex);
				int32* ExistingComponentIndex = ComponentIndexByRoot.Find(Root);
				if (ExistingComponentIndex == nullptr)
				{
					FCollisionComponent& Component = OutComponents.AddDefaulted_GetRef();
					Component.OverridingPlateId = static_cast<int32>(PairSamples.Key >> 32);
					Component.SubductingPlateId = static_cast<int32>(PairSamples.Key & 0xffffffffU);
					Component.MinSampleIndex = PairSamples.Value[CandidateIndex];
					ExistingComponentIndex = &ComponentIndexByRoot.Add(Root, OutComponents.Num() - 1);
				}

				FCollisionComponent& Component = OutComponents[*ExistingComponentIndex];
				Component.SampleIndices.Add(PairSamples.Value[CandidateIndex]);
				Component.MinSampleIndex = FMath::Min(Component.MinSampleIndex, PairSamples.Value[CandidateIndex]);
			}
		}
	}

	void SyncCarriedSamplesFromCanonicalIndices(
		FTectonicPlanet& Planet,
		const TArray<int32>& SampleIndices)
	{
		for (const int32 SampleIndex : SampleIndices)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			for (FPlate& Plate : Planet.Plates)
			{
				const int32* CarriedIndexPtr = Plate.CanonicalToCarriedIndex.Find(SampleIndex);
				if (CarriedIndexPtr == nullptr || !Plate.CarriedSamples.IsValidIndex(*CarriedIndexPtr))
				{
					continue;
				}

				PopulateCarriedSampleFromCanonical(Planet, SampleIndex, Plate.CarriedSamples[*CarriedIndexPtr]);
			}
		}
	}

	void DetectTerranesImpl(
		FTectonicPlanet& Planet,
		const TArray<FTerrane>& PreviousTerranes,
		const TArray<int32>& PreviousTerraneAssignments,
		FTerraneDetectionDiagnostics* OutDiagnostics)
	{
		if (OutDiagnostics != nullptr)
		{
			*OutDiagnostics = FTerraneDetectionDiagnostics{};
		}

		Planet.Terranes.Reset();
		ClearCanonicalTerraneAssignments(Planet);

		TMap<int32, int32> AnchorTerraneIdsBySample;
		for (const FTerrane& PreviousTerrane : PreviousTerranes)
		{
			if (PreviousTerrane.AnchorSample != INDEX_NONE)
			{
				AnchorTerraneIdsBySample.Add(PreviousTerrane.AnchorSample, PreviousTerrane.TerraneId);
			}
		}

		TArray<FTerraneComponent> Components;
		BuildTerraneComponents(Planet, PreviousTerraneAssignments, AnchorTerraneIdsBySample, Components);
		TArray<int32> AssignedTerraneIds;
		AssignedTerraneIds.Init(INDEX_NONE, Components.Num());
		TSet<int32> MatchedPreviousTerraneIds;

		for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
		{
			const FTerraneComponent& Component = Components[ComponentIndex];
			if (Component.AnchorTerraneIds.IsEmpty())
			{
				continue;
			}

			const int32 PreservedTerraneId = ChoosePreservedTerraneIdForMerger(Component);
			AssignedTerraneIds[ComponentIndex] = PreservedTerraneId;
			for (const int32 TerraneId : Component.AnchorTerraneIds)
			{
				MatchedPreviousTerraneIds.Add(TerraneId);
			}

			if (OutDiagnostics != nullptr && Component.AnchorTerraneIds.Num() > 1)
			{
				++OutDiagnostics->MergedTerraneCount;
			}
		}

		TArray<int32> PreviousTerraneOrder;
		PreviousTerraneOrder.Reserve(PreviousTerranes.Num());
		for (int32 PreviousTerraneIndex = 0; PreviousTerraneIndex < PreviousTerranes.Num(); ++PreviousTerraneIndex)
		{
			PreviousTerraneOrder.Add(PreviousTerraneIndex);
		}
		Algo::Sort(PreviousTerraneOrder, [&PreviousTerranes](const int32 LeftIndex, const int32 RightIndex)
		{
			return PreviousTerranes[LeftIndex].TerraneId < PreviousTerranes[RightIndex].TerraneId;
		});

		for (const int32 PreviousTerraneIndex : PreviousTerraneOrder)
		{
			const FTerrane& PreviousTerrane = PreviousTerranes[PreviousTerraneIndex];
			if (MatchedPreviousTerraneIds.Contains(PreviousTerrane.TerraneId))
			{
				continue;
			}

			const int32 ComponentIndex = FindBestCentroidFallbackComponent(Planet, PreviousTerrane, Components, AssignedTerraneIds);
			if (ComponentIndex == INDEX_NONE)
			{
				continue;
			}

			AssignedTerraneIds[ComponentIndex] = PreviousTerrane.TerraneId;
			MatchedPreviousTerraneIds.Add(PreviousTerrane.TerraneId);
		}

		for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
		{
			if (AssignedTerraneIds[ComponentIndex] != INDEX_NONE)
			{
				continue;
			}

			AssignedTerraneIds[ComponentIndex] = Planet.NextTerraneId++;
			if (OutDiagnostics != nullptr)
			{
				++OutDiagnostics->NewTerraneCount;
			}
		}

		Planet.Terranes.Reserve(Components.Num());
		for (int32 ComponentIndex = 0; ComponentIndex < Components.Num(); ++ComponentIndex)
		{
			const FTerraneComponent& Component = Components[ComponentIndex];
			const int32 TerraneId = AssignedTerraneIds[ComponentIndex];
			if (TerraneId == INDEX_NONE || Component.Members.IsEmpty())
			{
				continue;
			}

			FTerrane Terrane;
			Terrane.TerraneId = TerraneId;
			Terrane.PlateId = Component.PlateId;
			Terrane.Centroid = Component.Centroid;
			Terrane.AreaKm2 = Component.AreaKm2;
			Terrane.AnchorSample = ChooseAnchorSampleForComponent(Planet, Component);
			if (Terrane.AnchorSample == INDEX_NONE)
			{
				Terrane.AnchorSample = Component.Members[0];
			}

			for (const int32 SampleIndex : Component.Members)
			{
				Planet.Samples[SampleIndex].TerraneId = TerraneId;
			}

			Planet.Terranes.Add(Terrane);
		}

		Algo::Sort(Planet.Terranes, [](const FTerrane& Left, const FTerrane& Right)
		{
			return Left.TerraneId < Right.TerraneId;
		});
	}

	void InitializeScalarState(FTectonicPlanet& Planet, const int32 RandomSeed)
	{
		const FVector3d ElevationNoiseOffset = MakeNoiseOffset(RandomSeed, 0, 301);
		const FVector3d ContinentalNoiseOffset = MakeNoiseOffset(RandomSeed, 0, 401);

		for (FSample& Sample : Planet.Samples)
		{
			const double BaseNoise = ComputeNoiseValue(Sample.Position, ContinentalNoiseOffset, 7.0);
			const double GlobalNoise = GlobalNoiseAmplitudeKm * ComputeNoiseValue(Sample.Position, ElevationNoiseOffset, ElevationNoiseFrequency);
			const bool bContinental = Sample.ContinentalWeight >= 0.5f;
			Sample.Elevation = static_cast<float>(
				(bContinental ? InitialContinentalElevationKm : InitialOceanicElevationKm) +
				(bContinental ? 0.1 : 0.2) * BaseNoise +
				GlobalNoise);
			Sample.Thickness = static_cast<float>(bContinental ? ContinentalThicknessKm : OceanicThicknessKm);
			Sample.Age = 0.0f;
			Sample.RidgeDirection = FVector3d::ZeroVector;
			Sample.FoldDirection = FVector3d::ZeroVector;
			Sample.OrogenyType = EOrogenyType::None;
		}
	}

	void AssignInitialPlateMotion(FTectonicPlanet& Planet, const int32 RandomSeed)
	{
		FRandomStream Random(MakeDeterministicSeed(RandomSeed, Planet.Plates.Num(), 77));
		const double MaxAngularSpeed = MaxDistanceAdvanceKmPerStep / FMath::Max(Planet.PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);
		const double MinAngularSpeed = MinPlateAngularSpeedFactor * MaxAngularSpeed;

		for (FPlate& Plate : Planet.Plates)
		{
			Plate.RotationAxis = MakeRandomUnitVector(Random).GetSafeNormal();
			Plate.AngularSpeed = Random.FRandRange(static_cast<float>(MinAngularSpeed), static_cast<float>(MaxAngularSpeed));
			Plate.CumulativeRotation = FQuat4d::Identity;
			ResetSoupState(Plate);
		}
	}
}

void FTectonicPlanet::Initialize(const int32 InSampleCount, const double InPlanetRadiusKm)
{
	SampleCountConfig = InSampleCount;
	PlanetRadiusKm = InPlanetRadiusKm;
	PlateCountConfig = 0;
	NextPlateId = 0;
	CurrentStep = 0;
	NextTerraneId = 0;
	LastComputedResampleInterval = 0;
	MaxResampleCount = INDEX_NONE;
	MaxStepsWithoutResampling = INDEX_NONE;
	bEnableSlabPull = true;
	bEnableAndeanContinentalConversion = true;
	bEnableOverlapHysteresis = false;
	bEnableContinentalCollision = true;
	ResamplingPolicy = EResamplingPolicy::PeriodicFull;
	LastResampleTriggerReason = EResampleTriggerReason::None;
	LastResampleOwnershipMode = EResampleOwnershipMode::FullResolution;
	bPendingFullResolutionResample = false;
	bPendingBoundaryContactPersistenceReset = false;
	LastResamplingStats = FResamplingStats{};
	ResamplingSteps.Reset();
	BoundaryContactPersistenceByPair.Reset();
	PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	PendingRiftEvent = FPendingRiftEvent{};
	SimulationSeed = 0;
	Samples.Reset();
	Plates.Reset();
	Terranes.Reset();
	TriangleIndices.Reset();
	SampleAdjacency.Reset();
	TriangleAdjacency.Reset();

	if (InSampleCount <= 0)
	{
		return;
	}

	Samples.SetNum(InSampleCount);
	const double GoldenRatio = (1.0 + FMath::Sqrt(5.0)) * 0.5;
	for (int32 SampleIndex = 0; SampleIndex < InSampleCount; ++SampleIndex)
	{
		const double U = (static_cast<double>(SampleIndex) + 0.5) / static_cast<double>(InSampleCount);
		const double Theta = FMath::Acos(1.0 - 2.0 * U);
		const double Phi = (2.0 * PI * static_cast<double>(SampleIndex)) / GoldenRatio;

		FSample Sample;
		Sample.Position = FVector3d(
			FMath::Sin(Theta) * FMath::Cos(Phi),
			FMath::Sin(Theta) * FMath::Sin(Phi),
			FMath::Cos(Theta));
		Samples[SampleIndex] = Sample;
	}

	if (Samples.Num() < 4)
	{
		SampleAdjacency.SetNum(Samples.Num());
		TriangleAdjacency.SetNum(0);
		return;
	}

	TArray<FVector3d> HullPoints;
	HullPoints.Reserve(Samples.Num());
	for (const FSample& Sample : Samples)
	{
		HullPoints.Add(Sample.Position);
	}

	UE::Geometry::TConvexHull3<double> Hull;
	const bool bBuiltHull = Hull.Solve(TArrayView<const FVector3d>(HullPoints));
	checkf(bBuiltHull, TEXT("Failed to build convex hull for spherical Delaunay triangulation."));

	const TArray<UE::Geometry::FIndex3i>& HullTriangles = Hull.GetTriangles();
	TriangleIndices.Reserve(HullTriangles.Num());
	for (const UE::Geometry::FIndex3i& Triangle : HullTriangles)
	{
		if (Triangle.A == Triangle.B || Triangle.B == Triangle.C || Triangle.C == Triangle.A)
		{
			continue;
		}

		TriangleIndices.Add(FIntVector(Triangle.A, Triangle.B, Triangle.C));
	}

	SampleAdjacency.SetNum(Samples.Num());
	for (const FIntVector& Triangle : TriangleIndices)
	{
		AddUniqueNeighbor(SampleAdjacency[Triangle.X], Triangle.Y);
		AddUniqueNeighbor(SampleAdjacency[Triangle.X], Triangle.Z);
		AddUniqueNeighbor(SampleAdjacency[Triangle.Y], Triangle.X);
		AddUniqueNeighbor(SampleAdjacency[Triangle.Y], Triangle.Z);
		AddUniqueNeighbor(SampleAdjacency[Triangle.Z], Triangle.X);
		AddUniqueNeighbor(SampleAdjacency[Triangle.Z], Triangle.Y);
	}

	TriangleAdjacency.SetNum(TriangleIndices.Num());
	TMap<uint64, int32> EdgeToTriangle;
	for (int32 TriangleIndex = 0; TriangleIndex < TriangleIndices.Num(); ++TriangleIndex)
	{
		const FIntVector& Triangle = TriangleIndices[TriangleIndex];
		const int32 EdgeVertices[3][2] = {
			{ Triangle.X, Triangle.Y },
			{ Triangle.Y, Triangle.Z },
			{ Triangle.Z, Triangle.X }
		};

		for (int32 EdgeIndex = 0; EdgeIndex < 3; ++EdgeIndex)
		{
			const uint64 EdgeKey = MakeUndirectedEdgeKey(EdgeVertices[EdgeIndex][0], EdgeVertices[EdgeIndex][1]);
			if (const int32* OtherTriangleIndex = EdgeToTriangle.Find(EdgeKey))
			{
				AddUniqueNeighbor(TriangleAdjacency[TriangleIndex], *OtherTriangleIndex);
				AddUniqueNeighbor(TriangleAdjacency[*OtherTriangleIndex], TriangleIndex);
			}
			else
			{
				EdgeToTriangle.Add(EdgeKey, TriangleIndex);
			}
		}
	}
}

void FTectonicPlanet::InitializePlates(
	const int32 InPlateCount,
	const int32 InRandomSeed,
	const float InBoundaryWarpAmplitude,
	const float InContinentalFraction)
{
	checkf(!Samples.IsEmpty(), TEXT("InitializePlates requires canonical samples. Call Initialize() first."));

	PlateCountConfig = FMath::Clamp(InPlateCount, 1, Samples.Num());
	CurrentStep = 0;
	LastResamplingStats = FResamplingStats{};
	LastResampleTriggerReason = EResampleTriggerReason::None;
	LastResampleOwnershipMode = EResampleOwnershipMode::FullResolution;
	bPendingFullResolutionResample = false;
	bPendingBoundaryContactPersistenceReset = false;
	ResamplingSteps.Reset();
	BoundaryContactPersistenceByPair.Reset();
	PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	PendingRiftEvent = FPendingRiftEvent{};
	Terranes.Reset();
	SimulationSeed = InRandomSeed;
	NextPlateId = 0;
	NextTerraneId = 0;

	FRandomStream Random(InRandomSeed);
	TArray<int32> CentroidSampleIndices;
	CentroidSampleIndices.Reserve(PlateCountConfig);
	CentroidSampleIndices.Add(Random.RandRange(0, Samples.Num() - 1));

	while (CentroidSampleIndices.Num() < PlateCountConfig)
	{
		double BestDistance = -1.0;
		int32 BestSampleIndex = INDEX_NONE;

		for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
		{
			const FVector3d& Position = Samples[SampleIndex].Position;
			double MinDistanceToSeed = TNumericLimits<double>::Max();
			for (const int32 CentroidIndex : CentroidSampleIndices)
			{
				MinDistanceToSeed = FMath::Min(MinDistanceToSeed, ComputeGeodesicDistance(Position, Samples[CentroidIndex].Position));
			}

			if (MinDistanceToSeed > BestDistance)
			{
				BestDistance = MinDistanceToSeed;
				BestSampleIndex = SampleIndex;
			}
		}

		if (BestSampleIndex == INDEX_NONE)
		{
			break;
		}

		CentroidSampleIndices.Add(BestSampleIndex);
	}

	Plates.SetNum(PlateCountConfig);
	for (int32 PlateIndex = 0; PlateIndex < PlateCountConfig; ++PlateIndex)
	{
		FPlate& Plate = Plates[PlateIndex];
		Plate = FPlate{};
		Plate.Id = NextPlateId++;
	}

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		FSample& Sample = Samples[SampleIndex];
		double BestDistance = TNumericLimits<double>::Max();
		int32 BestPlateId = INDEX_NONE;

		for (int32 PlateIndex = 0; PlateIndex < CentroidSampleIndices.Num(); ++PlateIndex)
		{
			const FVector3d& Centroid = Samples[CentroidSampleIndices[PlateIndex]].Position;
			const double WarpedDistance = ComputeWarpedDistance(
				Sample.Position,
				Centroid,
				InRandomSeed,
				PlateIndex,
				101,
				InBoundaryWarpAmplitude,
				BoundaryNoiseFrequency);
			if (WarpedDistance < BestDistance)
			{
				BestDistance = WarpedDistance;
				BestPlateId = Plates[PlateIndex].Id;
			}
		}

		Sample.PlateId = BestPlateId;
		Sample.ContinentalWeight = 0.0f;
		Sample.Elevation = 0.0f;
		Sample.Thickness = 0.0f;
		Sample.Age = 0.0f;
		Sample.RidgeDirection = FVector3d::ZeroVector;
		Sample.FoldDirection = FVector3d::ZeroVector;
		Sample.OrogenyType = EOrogenyType::None;
		Sample.TerraneId = INDEX_NONE;
		Sample.bIsBoundary = false;
	}

	RebuildMembershipFromCanonical(*this);
	RecomputeBoundaryFlags(*this);
	ClassifyPlateTriangles(*this);
	RecomputePlateBoundingCaps(*this);

	TArray<int32> PlateOrder;
	PlateOrder.Reserve(Plates.Num());
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		PlateOrder.Add(PlateIndex);
	}

	Algo::Sort(PlateOrder, [this](const int32 LeftPlate, const int32 RightPlate)
	{
		return Plates[LeftPlate].MemberSamples.Num() > Plates[RightPlate].MemberSamples.Num();
	});

	const int32 SeededPlateCount = FMath::Clamp(
		FMath::CeilToInt(static_cast<float>(PlateCountConfig) * FMath::Clamp(InContinentalFraction, 0.0f, 1.0f) * 1.5f),
		1,
		FMath::Max(1, PlateCountConfig));
	TArray<uint8> bPlateSeeded;
	bPlateSeeded.Init(0, Plates.Num());
	for (int32 SeedIndex = 0; SeedIndex < SeededPlateCount && SeedIndex < PlateOrder.Num(); ++SeedIndex)
	{
		bPlateSeeded[PlateOrder[SeedIndex]] = 1;
	}

	const double ContinentRadiusRadians = FMath::Clamp(
		2.0 * FMath::Sqrt(FMath::Clamp(static_cast<double>(InContinentalFraction), 0.0, 1.0) / static_cast<double>(SeededPlateCount)),
		0.0,
		PI);

	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		FPlate& Plate = Plates[PlateIndex];
		const FVector3d PlateCentroid = Plate.BoundingCap.Center.IsNearlyZero()
			? Samples[CentroidSampleIndices[PlateIndex]].Position
			: Plate.BoundingCap.Center;

		for (const int32 SampleIndex : Plate.MemberSamples)
		{
			FSample& Sample = Samples[SampleIndex];
			Sample.ContinentalWeight = 0.0f;

			if (!bPlateSeeded.IsValidIndex(PlateIndex) || bPlateSeeded[PlateIndex] == 0)
			{
				continue;
			}

			const double WarpedDistance = ComputeWarpedDistance(
				Sample.Position,
				PlateCentroid,
				InRandomSeed,
				Plate.Id,
				202,
				InBoundaryWarpAmplitude,
				ContinentalNoiseFrequency);
			if (WarpedDistance < ContinentRadiusRadians)
			{
				Sample.ContinentalWeight = 1.0f;
			}
		}
	}

	InitializeScalarState(*this, InRandomSeed);
	DetectTerranes(TArray<FTerrane>{});
	RebuildCarriedSamplesFromSoupVertices(*this);
	ComputePlateScores();
	AssignInitialPlateMotion(*this, InRandomSeed);
	ComputeSubductionDistanceField();
	ComputeSlabPullCorrections();
	LastComputedResampleInterval = ComputeResampleInterval();
}

void FTectonicPlanet::ComputePlateScores()
{
	for (FPlate& Plate : Plates)
	{
		int32 ContinentalSampleCount = 0;
		int32 OceanicSampleCount = 0;
		for (const int32 SampleIndex : Plate.MemberSamples)
		{
			if (!Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			if (Samples[SampleIndex].ContinentalWeight >= 0.5f)
			{
				++ContinentalSampleCount;
			}
			else
			{
				++OceanicSampleCount;
			}
		}

		Plate.OverlapScore = (100 * ContinentalSampleCount) - OceanicSampleCount;
	}
}

void FTectonicPlanet::AdvanceStep()
{
	const double StepStartTime = FPlatformTime::Seconds();
	const double SlabPullPerturbation =
		(!Samples.IsEmpty() && !Plates.IsEmpty())
			? (SlabPullEpsilon * static_cast<double>(Plates.Num()) / static_cast<double>(Samples.Num()))
			: 0.0;

	for (FPlate& Plate : Plates)
	{
		if (bEnableSlabPull && !Plate.SlabPullCorrectionAxis.IsNearlyZero())
		{
			FVector3d AxisDelta = Plate.SlabPullCorrectionAxis * (SlabPullPerturbation * DeltaTimeMyears);
			const double AxisDeltaMagnitude = AxisDelta.Length();
			if (AxisDeltaMagnitude > MaxAxisChangePerStepRad)
			{
				AxisDelta = AxisDelta.GetSafeNormal() * MaxAxisChangePerStepRad;
			}

			const FVector3d CandidateAxis = (Plate.RotationAxis + AxisDelta).GetSafeNormal();
			if (!CandidateAxis.IsNearlyZero() && IsFiniteVector(CandidateAxis))
			{
				Plate.RotationAxis = CandidateAxis;
			}
		}

		if (!Plate.RotationAxis.IsNearlyZero() && Plate.AngularSpeed > 0.0)
		{
			const FQuat4d DeltaRotation(Plate.RotationAxis.GetSafeNormal(), Plate.AngularSpeed);
			Plate.CumulativeRotation = (DeltaRotation * Plate.CumulativeRotation).GetNormalized();
		}

		for (FCarriedSample& CarriedSample : Plate.CarriedSamples)
		{
			CarriedSample.Age += static_cast<float>(DeltaTimeMyears);
			if (CarriedSample.SubductionDistanceKm >= 0.0f)
			{
				const double DistanceRad =
					static_cast<double>(CarriedSample.SubductionDistanceKm) / FMath::Max(PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);
				const double UpliftKm =
					SubductionBaseUpliftKmPerMy *
					SubductionDistanceTransfer(DistanceRad, SubductionControlDistanceRad, SubductionMaxDistanceRad) *
					(static_cast<double>(CarriedSample.SubductionSpeed) / MaxPlateSpeedKmPerMy) *
					DeltaTimeMyears;
				CarriedSample.Elevation += static_cast<float>(UpliftKm);
				if (UpliftKm > 0.0 && CarriedSample.Elevation > 0.0f && CarriedSample.OrogenyType == EOrogenyType::None)
				{
					CarriedSample.OrogenyType = EOrogenyType::Andean;
				}
			}

			if (bEnableAndeanContinentalConversion &&
				CarriedSample.OrogenyType == EOrogenyType::Andean &&
				CarriedSample.Elevation > 0.0f &&
				CarriedSample.ContinentalWeight < 1.0f)
			{
				CarriedSample.ContinentalWeight = FMath::Min(
					1.0f,
					CarriedSample.ContinentalWeight + static_cast<float>(AndeanContinentalConversionRate * DeltaTimeMyears));
			}

			if (CarriedSample.ContinentalWeight >= 0.5f)
			{
				CarriedSample.Elevation -= static_cast<float>((CarriedSample.Elevation / ElevationCeilingKm) * ErosionRateKmPerStep);
			}
			else
			{
				CarriedSample.Elevation -= static_cast<float>(
					(1.0 - (CarriedSample.Elevation / AbyssalPlainElevationKm)) * DampeningRateKmPerStep);
			}

			if (CarriedSample.Elevation < TrenchElevationKm)
			{
				CarriedSample.Elevation += static_cast<float>(AccretionRateKmPerStep);
			}

			CarriedSample.Elevation = FMath::Clamp(CarriedSample.Elevation, static_cast<float>(TrenchElevationKm), static_cast<float>(ElevationCeilingKm));
		}
	}

	SyncCanonicalAttributesFromCarried(*this);
	++CurrentStep;

	const int32 ResampleInterval = ComputeResampleInterval();
	LastComputedResampleInterval = ResampleInterval;
	const bool bResampleLimitReached = MaxResampleCount != INDEX_NONE && ResamplingSteps.Num() >= MaxResampleCount;
	bool bTriggeredAutomaticRift = false;
	if (!bResampleLimitReached && bEnableAutomaticRifting)
	{
		bTriggeredAutomaticRift = TryTriggerAutomaticRift();
	}
	if (!bResampleLimitReached && !bTriggeredAutomaticRift)
	{
		switch (ResamplingPolicy)
		{
		case EResamplingPolicy::PeriodicFull:
			if (CurrentStep > 0 && ResampleInterval > 0 && (CurrentStep % ResampleInterval) == 0)
			{
				PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::Periodic);
			}
			break;

		case EResamplingPolicy::EventDrivenOnly:
		{
			const int32 LastResampleStep = ResamplingSteps.IsEmpty() ? 0 : ResamplingSteps.Last();
			const int32 StepsSinceLastResample = CurrentStep - LastResampleStep;
			if (MaxStepsWithoutResampling != INDEX_NONE && StepsSinceLastResample >= MaxStepsWithoutResampling)
			{
				TriggerEventResampling(EResampleTriggerReason::SafetyValve);
			}
			break;
		}

		case EResamplingPolicy::HybridStablePeriodic:
			if (CurrentStep > 0 && ResampleInterval > 0 && (CurrentStep % ResampleInterval) == 0)
			{
				PerformResampling(EResampleOwnershipMode::StableOverlaps, EResampleTriggerReason::Periodic);
			}
			break;

		case EResamplingPolicy::PreserveOwnershipPeriodic:
			if (CurrentStep > 0 && ResampleInterval > 0 && (CurrentStep % ResampleInterval) == 0)
			{
				PerformResampling(EResampleOwnershipMode::PreserveOwnership, EResampleTriggerReason::Periodic);
			}
			break;
		}
	}

	if (bPendingFullResolutionResample)
	{
		bPendingFullResolutionResample = false;
		if (!(MaxResampleCount != INDEX_NONE && ResamplingSteps.Num() >= MaxResampleCount))
		{
			PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
		}
	}

	const double StepDurationMs = (FPlatformTime::Seconds() - StepStartTime) * 1000.0;
	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Step %d] advance_step_ms=%.3f andean_count=%d"),
		CurrentStep,
		StepDurationMs,
		CountAndeanSamples(*this));
}

void FTectonicPlanet::TriggerEventResampling(const EResampleTriggerReason Reason)
{
	const bool bResampleLimitReached = MaxResampleCount != INDEX_NONE && ResamplingSteps.Num() >= MaxResampleCount;
	if (bResampleLimitReached)
	{
		return;
	}

	PerformResampling(EResampleOwnershipMode::FullResolution, Reason);
}

bool FTectonicPlanet::IsPlateEligibleForAutomaticRift(
	const FPlate& Plate,
	const int32 ChildCount,
	int32& OutContinentalSampleCount,
	double& OutContinentalFraction) const
{
	OutContinentalSampleCount = 0;
	OutContinentalFraction = 0.0;
	if (ChildCount < MinForcedRiftChildren || ChildCount > MaxForcedRiftChildren)
	{
		return false;
	}

	const int32 MinimumSampleCount = FMath::Max(AutomaticRiftMinParentSamples, ChildCount * MinRiftChildSamples);
	if (Plate.MemberSamples.Num() < MinimumSampleCount)
	{
		return false;
	}

	if (AutomaticRiftCooldownSteps > 0 &&
		Plate.LastRiftStep != INDEX_NONE &&
		(CurrentStep - Plate.LastRiftStep) < AutomaticRiftCooldownSteps)
	{
		return false;
	}

	ComputePlateContinentalSummary(*this, Plate, OutContinentalSampleCount, OutContinentalFraction);
	return OutContinentalSampleCount >= AutomaticRiftMinContinentalSamples &&
		OutContinentalFraction >= AutomaticRiftMinContinentalFraction;
}

double FTectonicPlanet::ComputeAutomaticRiftProbability(const FPlate& Plate, const double ContinentalFraction) const
{
	if (Samples.IsEmpty() || Plates.IsEmpty())
	{
		return 0.0;
	}

	const double AveragePlateSampleCount =
		static_cast<double>(Samples.Num()) / static_cast<double>(FMath::Max(Plates.Num(), 1));
	const double SizeScale = FMath::Clamp(
		static_cast<double>(Plate.MemberSamples.Num()) / FMath::Max(AveragePlateSampleCount, 1.0),
		0.25,
		4.0);
	const double ContinentalScale = FMath::Clamp(
		ContinentalFraction / FMath::Max(AutomaticRiftMinContinentalFraction, 0.01),
		0.25,
		4.0);
	const double Lambda =
		FMath::Max(0.0, AutomaticRiftBaseLambdaPerStep) *
		FMath::Pow(SizeScale, AutomaticRiftParentSizeExponent) *
		FMath::Pow(ContinentalScale, AutomaticRiftContinentalExponent);
	return FMath::Clamp(1.0 - FMath::Exp(-Lambda), 0.0, 0.95);
}

bool FTectonicPlanet::TryTriggerAutomaticRift()
{
	if (!bEnableAutomaticRifting || Plates.IsEmpty())
	{
		return false;
	}

	struct FAutomaticRiftCandidate
	{
		int32 ParentPlateId = INDEX_NONE;
		int32 ChildCount = 0;
		int32 EventSeed = 0;
		int32 ParentSampleCount = 0;
		int32 ContinentalSampleCount = 0;
		double ContinentalFraction = 0.0;
		double Probability = 0.0;
		double TriggerMargin = -1.0;
	};

	FAutomaticRiftCandidate BestCandidate;
	TArray<const FPlate*> SortedPlates;
	SortedPlates.Reserve(Plates.Num());
	for (const FPlate& Plate : Plates)
	{
		SortedPlates.Add(&Plate);
	}
	SortedPlates.Sort([](const FPlate& Left, const FPlate& Right)
	{
		if (Left.MemberSamples.Num() != Right.MemberSamples.Num())
		{
			return Left.MemberSamples.Num() > Right.MemberSamples.Num();
		}
		return Left.Id < Right.Id;
	});

	for (const FPlate* Plate : SortedPlates)
	{
		if (Plate == nullptr)
		{
			continue;
		}

		TArray<int32> EligibleChildCounts;
		for (int32 ChildCount = MinForcedRiftChildren; ChildCount <= MaxForcedRiftChildren; ++ChildCount)
		{
			int32 ContinentalSampleCount = 0;
			double ContinentalFraction = 0.0;
			if (IsPlateEligibleForAutomaticRift(*Plate, ChildCount, ContinentalSampleCount, ContinentalFraction))
			{
				EligibleChildCounts.Add(ChildCount);
			}
		}

		if (EligibleChildCounts.IsEmpty())
		{
			continue;
		}

		FRandomStream ChildCountRandom(MakeDeterministicSeed(SimulationSeed, CurrentStep, Plate->Id));
		const int32 ChildCount =
			EligibleChildCounts[ChildCountRandom.RandRange(0, EligibleChildCounts.Num() - 1)];
		int32 ContinentalSampleCount = 0;
		double ContinentalFraction = 0.0;
		if (!IsPlateEligibleForAutomaticRift(*Plate, ChildCount, ContinentalSampleCount, ContinentalFraction))
		{
			continue;
		}

		const double Probability = ComputeAutomaticRiftProbability(*Plate, ContinentalFraction);
		FRandomStream TriggerRandom(MakeDeterministicSeed(SimulationSeed, CurrentStep, MakeDeterministicSeed(Plate->Id, ChildCount, 701)));
		const double Draw = TriggerRandom.FRand();
		if (Draw >= Probability)
		{
			continue;
		}

		const double TriggerMargin = Probability - Draw;
		if (BestCandidate.ParentPlateId == INDEX_NONE ||
			TriggerMargin > BestCandidate.TriggerMargin + TriangleEpsilon ||
			(FMath::IsNearlyEqual(TriggerMargin, BestCandidate.TriggerMargin, TriangleEpsilon) &&
				(Plate->MemberSamples.Num() > BestCandidate.ParentSampleCount ||
					(Plate->MemberSamples.Num() == BestCandidate.ParentSampleCount &&
						Plate->Id < BestCandidate.ParentPlateId))))
		{
			BestCandidate.ParentPlateId = Plate->Id;
			BestCandidate.ChildCount = ChildCount;
			BestCandidate.EventSeed = MakeDeterministicSeed(SimulationSeed, CurrentStep, MakeDeterministicSeed(Plate->Id, ChildCount, 991));
			BestCandidate.ParentSampleCount = Plate->MemberSamples.Num();
			BestCandidate.ContinentalSampleCount = ContinentalSampleCount;
			BestCandidate.ContinentalFraction = ContinentalFraction;
			BestCandidate.Probability = Probability;
			BestCandidate.TriggerMargin = TriggerMargin;
		}
	}

	if (BestCandidate.ParentPlateId == INDEX_NONE)
	{
		return false;
	}

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[AutoRift Step=%d] parent=%d child_count=%d parent_samples=%d parent_continental_samples=%d parent_continental_fraction=%.4f probability=%.6f event_seed=%d"),
		CurrentStep,
		BestCandidate.ParentPlateId,
		BestCandidate.ChildCount,
		BestCandidate.ParentSampleCount,
		BestCandidate.ContinentalSampleCount,
		BestCandidate.ContinentalFraction,
		BestCandidate.Probability,
		BestCandidate.EventSeed);

	return TriggerForcedRiftInternal(
		BestCandidate.ParentPlateId,
		BestCandidate.ChildCount,
		BestCandidate.EventSeed,
		true,
		BestCandidate.Probability,
		BestCandidate.ContinentalSampleCount,
		BestCandidate.ContinentalFraction);
}

int32 FTectonicPlanet::FindPlateArrayIndexById(const int32 PlateId) const
{
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		if (Plates[PlateIndex].Id == PlateId)
		{
			return PlateIndex;
		}
	}

	return INDEX_NONE;
}

bool FTectonicPlanet::TriggerForcedRift(const int32 ParentPlateId, const int32 ChildCount, const int32 Seed)
{
	return TriggerForcedRiftInternal(ParentPlateId, ChildCount, Seed, false, 0.0, 0, 0.0);
}

bool FTectonicPlanet::TriggerForcedRiftInternal(
	const int32 ParentPlateId,
	int32 ChildCount,
	const int32 Seed,
	const bool bAutomatic,
	const double TriggerProbability,
	const int32 ParentContinentalSampleCount,
	const double ParentContinentalFraction)
{
	const double RiftStartTime = FPlatformTime::Seconds();
	ChildCount = FMath::Clamp(ChildCount, MinForcedRiftChildren, MaxForcedRiftChildren);
	const int32 ParentPlateIndex = FindPlateArrayIndexById(ParentPlateId);
	if (!Plates.IsValidIndex(ParentPlateIndex))
	{
		return false;
	}

	const FPlate ParentPlate = Plates[ParentPlateIndex];
	if (ParentPlate.MemberSamples.Num() < (ChildCount * MinRiftChildSamples))
	{
		return false;
	}

	TArray<int32> ParentMembers = ParentPlate.MemberSamples;
	ParentMembers.Sort();
	TArray<int32> FormerParentTerraneIds;
	FormerParentTerraneIds.Reserve(ParentMembers.Num());
	for (const int32 SampleIndex : ParentMembers)
	{
		FormerParentTerraneIds.Add(
			Samples.IsValidIndex(SampleIndex)
				? Samples[SampleIndex].TerraneId
				: INDEX_NONE);
	}

	int32 MeasuredParentContinentalSampleCount = 0;
	double MeasuredParentContinentalFraction = 0.0;
	ComputePlateContinentalSummary(*this, ParentPlate, MeasuredParentContinentalSampleCount, MeasuredParentContinentalFraction);
	const int32 EffectiveParentContinentalSampleCount =
		bAutomatic ? ParentContinentalSampleCount : MeasuredParentContinentalSampleCount;
	const double EffectiveParentContinentalFraction =
		bAutomatic ? ParentContinentalFraction : MeasuredParentContinentalFraction;
	const int32 RiftEventSeed = MakeDeterministicSeed(SimulationSeed, ParentPlateId, Seed);

	TArray<int32> CentroidSampleIndices;
	if (!ChooseForcedRiftCentroidSamples(*this, ParentMembers, ChildCount, Seed, CentroidSampleIndices))
	{
		return false;
	}

	TArray<TArray<int32>> ChildMemberSamples;
	if (!PartitionParentMembersByCentroids(
			*this,
			ParentMembers,
			CentroidSampleIndices,
			RiftEventSeed,
			bEnableWarpedRiftBoundaries,
			RiftBoundaryWarpAmplitude,
			RiftBoundaryWarpFrequency,
			ChildMemberSamples))
	{
		return false;
	}

	TArray<FVector3d> ChildCentroids;
	ChildCentroids.Reserve(ChildMemberSamples.Num());
	for (const TArray<int32>& ChildMembers : ChildMemberSamples)
	{
		const FVector3d ChildCentroid = ComputeSampleSetCentroidOnUnitSphere(*this, ChildMembers);
		if (ChildCentroid.IsNearlyZero())
		{
			return false;
		}

		ChildCentroids.Add(ChildCentroid);
	}

	const FVector3d ParentCentroid = ComputePlateCentroidOnUnitSphere(*this, ParentPlate);
	TArray<FVector3d> ChildAxes;
	if (!TryComputeForcedRiftChildAxes(
			ParentCentroid,
			ParentPlate.RotationAxis,
			ChildCentroids,
			ChildAxes))
	{
		return false;
	}

	TArray<int32> ChildPlateIds;
	ChildPlateIds.Reserve(ChildCount);
	TArray<int32> ChildSampleCounts;
	ChildSampleCounts.Reserve(ChildCount);
	TArray<FPlate> NewChildPlates;
	NewChildPlates.Reserve(ChildCount);
	const double MaxRiftAngularSpeed =
		MaxDistanceAdvanceKmPerStep / FMath::Max(PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);
	const double ChildAngularSpeedScale = ChildCount == 2 ? 1.35 : 1.20;
	for (int32 ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
	{
		FPlate ChildPlate;
		ChildPlate.Id = NextPlateId++;
		ChildPlate.RotationAxis = ChildAxes[ChildIndex];
		ChildPlate.AngularSpeed = FMath::Min(ParentPlate.AngularSpeed * ChildAngularSpeedScale, MaxRiftAngularSpeed);
		ChildPlate.LastRiftStep = CurrentStep;
		ChildPlate.CumulativeRotation = ParentPlate.CumulativeRotation;
		ChildPlate.MemberSamples = ChildMemberSamples[ChildIndex];
		ChildPlateIds.Add(ChildPlate.Id);
		ChildSampleCounts.Add(ChildPlate.MemberSamples.Num());

		for (const int32 SampleIndex : ChildPlate.MemberSamples)
		{
			if (Samples.IsValidIndex(SampleIndex))
			{
				Samples[SampleIndex].PlateId = ChildPlate.Id;
			}
		}

		NewChildPlates.Add(MoveTemp(ChildPlate));
	}

	Plates.RemoveAt(ParentPlateIndex);
	for (FPlate& ChildPlate : NewChildPlates)
	{
		Plates.Add(MoveTemp(ChildPlate));
	}
	PlateCountConfig = Plates.Num();
	ResetBoundaryContactCollisionPersistence();

	TArray<int32> PostRiftPlateIds;
	PostRiftPlateIds.Reserve(Samples.Num());
	for (const FSample& Sample : Samples)
	{
		PostRiftPlateIds.Add(Sample.PlateId);
	}

	RepartitionMembership(PostRiftPlateIds);
	PendingRiftEvent = FPendingRiftEvent{};
	PendingRiftEvent.bValid = true;
	PendingRiftEvent.bAutomatic = bAutomatic;
	PendingRiftEvent.ParentPlateId = ParentPlateId;
	PendingRiftEvent.ChildCount = ChildCount;
	PendingRiftEvent.ChildPlateA = ChildPlateIds.IsValidIndex(0) ? ChildPlateIds[0] : INDEX_NONE;
	PendingRiftEvent.ChildPlateB = ChildPlateIds.IsValidIndex(1) ? ChildPlateIds[1] : INDEX_NONE;
	PendingRiftEvent.ParentSampleCount = ParentMembers.Num();
	PendingRiftEvent.ParentContinentalSampleCount = EffectiveParentContinentalSampleCount;
	PendingRiftEvent.ChildSampleCountA = ChildSampleCounts.IsValidIndex(0) ? ChildSampleCounts[0] : 0;
	PendingRiftEvent.ChildSampleCountB = ChildSampleCounts.IsValidIndex(1) ? ChildSampleCounts[1] : 0;
	PendingRiftEvent.EventSeed = RiftEventSeed;
	PendingRiftEvent.ChildPlateIds = ChildPlateIds;
	PendingRiftEvent.ChildSampleCounts = ChildSampleCounts;
	PendingRiftEvent.FormerParentSampleIndices = ParentMembers;
	PendingRiftEvent.FormerParentTerraneIds = MoveTemp(FormerParentTerraneIds);
	PendingRiftEvent.ParentContinentalFraction = EffectiveParentContinentalFraction;
	PendingRiftEvent.TriggerProbability = TriggerProbability;
	PendingRiftEvent.RiftMs = (FPlatformTime::Seconds() - RiftStartTime) * 1000.0;

	const FString ChildPlateIdsString = JoinIntArrayForLog(ChildPlateIds);
	const FString ChildSampleCountsString = JoinIntArrayForLog(ChildSampleCounts);

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Rift Step=%d] mode=%s parent=%d child_count=%d child_ids=(%s) parent_samples=%d parent_continental_samples=%d parent_continental_fraction=%.4f event_seed=%d child_samples=(%s)"),
		CurrentStep,
		bAutomatic ? TEXT("automatic") : TEXT("manual"),
		ParentPlateId,
		ChildCount,
		*ChildPlateIdsString,
		ParentMembers.Num(),
		EffectiveParentContinentalSampleCount,
		EffectiveParentContinentalFraction,
		RiftEventSeed,
		*ChildSampleCountsString);

	PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::RiftFollowup);
	return true;
}

void FTectonicPlanet::BuildContainmentSoups()
{
	for (FPlate& Plate : Plates)
	{
		ResetSoupState(Plate);
		Plate.SoupData.PlateId = Plate.Id;
		Plate.SoupData.ChangeStamp = 1;

		if (Plate.SoupTriangles.IsEmpty())
		{
			Plate.BoundingCap = FSphericalBoundingCap{};
			continue;
		}

		Plate.SoupData.GlobalTriangleIndices.Reserve(Plate.SoupTriangles.Num());
		Plate.SoupData.LocalTriangles.Reserve(Plate.SoupTriangles.Num());
		Plate.SoupData.RotatedVertices.Reserve(Plate.SoupTriangles.Num() * 3);
		const FQuat4d& PlateRotation = Plate.CumulativeRotation;

		for (const int32 GlobalTriangleIndex : Plate.SoupTriangles)
		{
			if (!TriangleIndices.IsValidIndex(GlobalTriangleIndex))
			{
				continue;
			}

			const FIntVector& GlobalTriangle = TriangleIndices[GlobalTriangleIndex];
			const int32 CanonicalVertices[3] = { GlobalTriangle.X, GlobalTriangle.Y, GlobalTriangle.Z };
			int32 LocalVertices[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };

			for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
			{
				const int32 CanonicalVertexIndex = CanonicalVertices[CornerIndex];
				int32* ExistingLocalIndex = Plate.SoupData.CanonicalToLocalVertex.Find(CanonicalVertexIndex);
				if (ExistingLocalIndex != nullptr)
				{
					LocalVertices[CornerIndex] = *ExistingLocalIndex;
					continue;
				}

				const int32 NewLocalVertexIndex = Plate.SoupData.RotatedVertices.Num();
				if (!Samples.IsValidIndex(CanonicalVertexIndex))
				{
					continue;
				}

				const FVector3d RotatedPosition =
					PlateRotation.RotateVector(Samples[CanonicalVertexIndex].Position).GetSafeNormal();
				Plate.SoupData.CanonicalToLocalVertex.Add(CanonicalVertexIndex, NewLocalVertexIndex);
				Plate.SoupData.LocalToCanonicalVertex.Add(CanonicalVertexIndex);
				Plate.SoupData.RotatedVertices.Add(RotatedPosition);
				LocalVertices[CornerIndex] = NewLocalVertexIndex;
			}

			if (LocalVertices[0] == INDEX_NONE || LocalVertices[1] == INDEX_NONE || LocalVertices[2] == INDEX_NONE)
			{
				continue;
			}

			const int32 LocalTriangleIndex = Plate.SoupData.LocalTriangles.Num();
			Plate.SoupData.GlobalTriangleIndices.Add(GlobalTriangleIndex);
			Plate.SoupData.LocalTriangles.Add(UE::Geometry::FIndex3i(LocalVertices[0], LocalVertices[1], LocalVertices[2]));
			Plate.SoupData.GlobalToLocalTriangle.Add(GlobalTriangleIndex, LocalTriangleIndex);
		}

		Plate.SoupData.ChangeStamp++;
		Plate.SoupAdapter = FPlateTriangleSoupAdapter(&Plate.SoupData);
		Plate.SoupBVH.SetMesh(&Plate.SoupAdapter, true);
		Plate.BoundingCap = BuildBoundingCapFromVertices(Plate.SoupData.RotatedVertices);
	}
}

void FTectonicPlanet::QueryOwnership(
	TArray<int32>& OutNewPlateIds,
	TArray<int32>& OutContainingTriangles,
	TArray<FVector3d>& OutBarycentricCoords,
	TArray<uint8>& OutGapFlags,
	TArray<uint8>& OutOverlapFlags,
	int32& OutGapCount,
	int32& OutOverlapCount,
	TArray<TArray<int32>>* OutOverlapPlateIds,
	FResamplingStats* OutStats,
	const EResampleOwnershipMode OwnershipMode) const
{
	OutNewPlateIds.Init(INDEX_NONE, Samples.Num());
	OutContainingTriangles.Init(INDEX_NONE, Samples.Num());
	OutBarycentricCoords.Init(FVector3d(-1.0, -1.0, -1.0), Samples.Num());
	OutGapFlags.Init(0, Samples.Num());
	OutOverlapFlags.Init(0, Samples.Num());
	TArray<uint8> RecoveryFlags;
	TArray<uint8> RecoveryContinentalFlags;
	TArray<double> RecoveryDistances;
	RecoveryFlags.Init(0, Samples.Num());
	RecoveryContinentalFlags.Init(0, Samples.Num());
	RecoveryDistances.Init(-1.0, Samples.Num());
	if (OutOverlapPlateIds != nullptr)
	{
		OutOverlapPlateIds->SetNum(Samples.Num());
	}
	if (OutStats != nullptr)
	{
		OutStats->ExactSingleHitCount = 0;
		OutStats->ExactMultiHitCount = 0;
		OutStats->RecoveryContainmentCount = 0;
		OutStats->RecoveryContinentalCount = 0;
		OutStats->TrueGapCount = 0;
		OutStats->RecoveryMeanDistance = 0.0;
		OutStats->RecoveryP95Distance = 0.0;
		OutStats->RecoveryMaxDistance = 0.0;
		OutStats->HysteresisRetainedCount = 0;
		OutStats->HysteresisReassignedCount = 0;
		OutStats->PreserveOwnershipSamePlateHitCount = 0;
		OutStats->PreserveOwnershipSamePlateRecoveryCount = 0;
		OutStats->PreserveOwnershipFallbackQueryCount = 0;
		OutStats->PreserveOwnershipPlateChangedCount = 0;
	}

	TAtomic<int32> HysteresisRetainedCount(0);
	TAtomic<int32> HysteresisReassignedCount(0);
	TAtomic<int32> PreserveOwnershipSamePlateHitCount(0);
	TAtomic<int32> PreserveOwnershipSamePlateRecoveryCount(0);
	TAtomic<int32> PreserveOwnershipFallbackQueryCount(0);
	const bool bUseStableOverlaps =
		OwnershipMode == EResampleOwnershipMode::StableOverlaps || bEnableOverlapHysteresis;
	const bool bUsePreserveOwnership = OwnershipMode == EResampleOwnershipMode::PreserveOwnership;

	bool bAllRotationsIdentity = true;
	for (const FPlate& Plate : Plates)
	{
		if (!IsIdentityRotation(Plate.CumulativeRotation))
		{
			bAllRotationsIdentity = false;
			break;
		}
	}

	ParallelFor(
		Samples.Num(),
		[this, bAllRotationsIdentity, bUseStableOverlaps, bUsePreserveOwnership, &OutNewPlateIds, &OutContainingTriangles, &OutBarycentricCoords, &OutGapFlags, &OutOverlapFlags, OutOverlapPlateIds, &RecoveryFlags, &RecoveryContinentalFlags, &RecoveryDistances, &HysteresisRetainedCount, &HysteresisReassignedCount, &PreserveOwnershipSamePlateHitCount, &PreserveOwnershipSamePlateRecoveryCount, &PreserveOwnershipFallbackQueryCount](const int32 SampleIndex)
	{
		const FVector3d QueryPoint = Samples[SampleIndex].Position;
		const int32 CurrentPlateId = Samples[SampleIndex].PlateId;

		const FPlate* CurrentPlatePtr = FindPlateById(*this, CurrentPlateId);
		if (bUsePreserveOwnership && CurrentPlatePtr == nullptr)
		{
			++PreserveOwnershipFallbackQueryCount;
		}
		else if (bUsePreserveOwnership && CurrentPlatePtr != nullptr)
		{
			const FPlate& CurrentPlate = *CurrentPlatePtr;
			if (CurrentPlate.SoupData.LocalTriangles.IsEmpty())
			{
				++PreserveOwnershipFallbackQueryCount;
			}
			else
			{
				int32 LocalTriangleId = INDEX_NONE;
				FVector3d A;
				FVector3d B;
				FVector3d C;
				const bool bInsideCurrentBoundingCap =
					CurrentPlate.BoundingCap.Center.IsNearlyZero() ||
					QueryPoint.Dot(CurrentPlate.BoundingCap.Center) >= CurrentPlate.BoundingCap.CosAngle;
				if (bInsideCurrentBoundingCap &&
					FindContainingTriangleInBVH(CurrentPlate.SoupBVH, CurrentPlate.SoupAdapter, QueryPoint, LocalTriangleId, A, B, C))
				{
					OutNewPlateIds[SampleIndex] = CurrentPlateId;
					OutContainingTriangles[SampleIndex] = CurrentPlate.SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId)
						? CurrentPlate.SoupData.GlobalTriangleIndices[LocalTriangleId]
						: INDEX_NONE;
					OutBarycentricCoords[SampleIndex] = NormalizeBarycentric(ComputePlanarBarycentric(A, B, C, QueryPoint));
					++PreserveOwnershipSamePlateHitCount;
					return;
				}

				FRecoveredContainmentHit RecoveryHit;
				if (TryFindNearestTriangleRecoveryHit(*this, CurrentPlateId, QueryPoint, RecoveryHit) &&
					RecoveryHit.Distance < ContainmentRecoveryTolerance)
				{
					OutNewPlateIds[SampleIndex] = CurrentPlateId;
					OutContainingTriangles[SampleIndex] = RecoveryHit.GlobalTriangleIndex;
					OutBarycentricCoords[SampleIndex] = RecoveryHit.Barycentric;
					RecoveryFlags[SampleIndex] = 1;
					RecoveryDistances[SampleIndex] = RecoveryHit.Distance;

					double RecoveredContinentalWeight = 0.0;
					if (TryComputeRecoveredContinentalWeight(
							*this,
							CurrentPlateId,
							RecoveryHit.GlobalTriangleIndex,
							RecoveryHit.Barycentric,
							RecoveredContinentalWeight) &&
						RecoveredContinentalWeight >= 0.5)
					{
						RecoveryContinentalFlags[SampleIndex] = 1;
					}

					++PreserveOwnershipSamePlateRecoveryCount;
					return;
				}

				++PreserveOwnershipFallbackQueryCount;
			}
		}

		int32 BestPlateId = INDEX_NONE;
		int32 BestTriangleIndex = INDEX_NONE;
		FVector3d BestBarycentric(-1.0, -1.0, -1.0);
		double BestScore = -TNumericLimits<double>::Max();
		int32 ContainingPlateCount = 0;
		int32 MeaningfulContainingPlateCount = 0;
		TArray<int32, TInlineAllocator<4>> MeaningfulContainingPlateIds;
		TArray<int32, TInlineAllocator<4>> MeaningfulTriangleIndices;
		TArray<FVector3d, TInlineAllocator<4>> MeaningfulBarycentrics;
		bool bCurrentPlateContained = false;
		int32 CurrentPlateTriangleIndex = INDEX_NONE;
		FVector3d CurrentPlateBarycentric(-1.0, -1.0, -1.0);

		for (const FPlate& Plate : Plates)
		{
			if (Plate.SoupData.LocalTriangles.IsEmpty())
			{
				continue;
			}

			if (!Plate.BoundingCap.Center.IsNearlyZero() &&
				QueryPoint.Dot(Plate.BoundingCap.Center) < Plate.BoundingCap.CosAngle)
			{
				continue;
			}

			int32 LocalTriangleId = INDEX_NONE;
			FVector3d A;
			FVector3d B;
			FVector3d C;
			if (!FindContainingTriangleInBVH(Plate.SoupBVH, Plate.SoupAdapter, QueryPoint, LocalTriangleId, A, B, C))
			{
				continue;
			}

			++ContainingPlateCount;
			const FVector3d Barycentric = NormalizeBarycentric(ComputePlanarBarycentric(A, B, C, QueryPoint));
			const double Score = ComputeContainmentScore(Barycentric);
			if (Score > OverlapScoreEpsilon)
			{
				++MeaningfulContainingPlateCount;
				MeaningfulContainingPlateIds.Add(Plate.Id);
				MeaningfulTriangleIndices.Add(Plate.SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId)
					? Plate.SoupData.GlobalTriangleIndices[LocalTriangleId]
					: INDEX_NONE);
				MeaningfulBarycentrics.Add(Barycentric);
			}
			const bool bIsBetterCandidate =
				(BestPlateId == INDEX_NONE) ||
				(Score > BestScore + TriangleEpsilon) ||
				(FMath::IsNearlyEqual(Score, BestScore, TriangleEpsilon) && (BestPlateId == INDEX_NONE || Plate.Id < BestPlateId));
			if (!bIsBetterCandidate)
			{
				if (Plate.Id == CurrentPlateId)
				{
					bCurrentPlateContained = true;
					CurrentPlateTriangleIndex = Plate.SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId)
						? Plate.SoupData.GlobalTriangleIndices[LocalTriangleId]
						: INDEX_NONE;
					CurrentPlateBarycentric = Barycentric;
				}
				continue;
			}

			BestScore = Score;
			BestPlateId = Plate.Id;
			BestTriangleIndex = Plate.SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId)
				? Plate.SoupData.GlobalTriangleIndices[LocalTriangleId]
				: INDEX_NONE;
			BestBarycentric = Barycentric;
			if (Plate.Id == CurrentPlateId)
			{
				bCurrentPlateContained = true;
				CurrentPlateTriangleIndex = BestTriangleIndex;
				CurrentPlateBarycentric = BestBarycentric;
			}
		}

		if (bAllRotationsIdentity && bCurrentPlateContained)
		{
			OutNewPlateIds[SampleIndex] = CurrentPlateId;
			OutContainingTriangles[SampleIndex] = CurrentPlateTriangleIndex;
			OutBarycentricCoords[SampleIndex] = CurrentPlateBarycentric;
			return;
		}

		if (ContainingPlateCount == 0)
		{
			FRecoveredContainmentHit RecoveryHit;
			if (TryFindNearestTriangleRecoveryHit(*this, QueryPoint, RecoveryHit) &&
				RecoveryHit.Distance < ContainmentRecoveryTolerance)
			{
				OutNewPlateIds[SampleIndex] = RecoveryHit.PlateId;
				OutContainingTriangles[SampleIndex] = RecoveryHit.GlobalTriangleIndex;
				OutBarycentricCoords[SampleIndex] = RecoveryHit.Barycentric;
				RecoveryFlags[SampleIndex] = 1;
				RecoveryDistances[SampleIndex] = RecoveryHit.Distance;

				double RecoveredContinentalWeight = 0.0;
				if (TryComputeRecoveredContinentalWeight(
						*this,
						RecoveryHit.PlateId,
						RecoveryHit.GlobalTriangleIndex,
						RecoveryHit.Barycentric,
						RecoveredContinentalWeight) &&
					RecoveredContinentalWeight >= 0.5)
				{
					RecoveryContinentalFlags[SampleIndex] = 1;
				}
				return;
			}

			OutGapFlags[SampleIndex] = 1;
			return;
		}

		if (!bAllRotationsIdentity && MeaningfulContainingPlateCount > 1)
		{
			OutOverlapFlags[SampleIndex] = 1;
			if (OutOverlapPlateIds != nullptr)
			{
				TArray<int32>& SampleOverlapPlateIds = (*OutOverlapPlateIds)[SampleIndex];
				SampleOverlapPlateIds.Reserve(MeaningfulContainingPlateIds.Num());
				for (const int32 PlateId : MeaningfulContainingPlateIds)
				{
					SampleOverlapPlateIds.Add(PlateId);
				}
			}

			if (bUseStableOverlaps)
			{
				int32 CurrentPlateContainmentIndex = INDEX_NONE;
				for (int32 ContainmentIndex = 0; ContainmentIndex < MeaningfulContainingPlateIds.Num(); ++ContainmentIndex)
				{
					if (MeaningfulContainingPlateIds[ContainmentIndex] == CurrentPlateId)
					{
						CurrentPlateContainmentIndex = ContainmentIndex;
						break;
					}
				}

				if (CurrentPlateContainmentIndex != INDEX_NONE)
				{
					OutNewPlateIds[SampleIndex] = CurrentPlateId;
					OutContainingTriangles[SampleIndex] = MeaningfulTriangleIndices[CurrentPlateContainmentIndex];
					OutBarycentricCoords[SampleIndex] = MeaningfulBarycentrics[CurrentPlateContainmentIndex];
					++HysteresisRetainedCount;
					return;
				}

				++HysteresisReassignedCount;
			}

			int32 WinnerIndex = 0;
			int32 WinnerOverlapScore = TNumericLimits<int32>::Lowest();
			for (int32 CandidateIndex = 0; CandidateIndex < MeaningfulContainingPlateIds.Num(); ++CandidateIndex)
			{
				const int32 CandidatePlateId = MeaningfulContainingPlateIds[CandidateIndex];
				const FPlate* CandidatePlate = FindPlateById(*this, CandidatePlateId);
				if (CandidatePlate == nullptr)
				{
					continue;
				}

				const int32 CandidateOverlapScore = CandidatePlate->OverlapScore;
				if (CandidateOverlapScore > WinnerOverlapScore ||
					(CandidateOverlapScore == WinnerOverlapScore &&
						CandidatePlateId < MeaningfulContainingPlateIds[WinnerIndex]))
				{
					WinnerOverlapScore = CandidateOverlapScore;
					WinnerIndex = CandidateIndex;
				}
			}

			OutNewPlateIds[SampleIndex] = MeaningfulContainingPlateIds[WinnerIndex];
			OutContainingTriangles[SampleIndex] = MeaningfulTriangleIndices[WinnerIndex];
			OutBarycentricCoords[SampleIndex] = MeaningfulBarycentrics[WinnerIndex];
			return;
		}

		OutNewPlateIds[SampleIndex] = BestPlateId;
		OutContainingTriangles[SampleIndex] = BestTriangleIndex;
		OutBarycentricCoords[SampleIndex] = BestBarycentric;
	});

	OutGapCount = 0;
	OutOverlapCount = 0;
	int32 RecoveryContainmentCount = 0;
	int32 RecoveryContinentalCount = 0;
	TArray<double> RecoveredDistances;
	RecoveredDistances.Reserve(Samples.Num());
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		OutGapCount += OutGapFlags[SampleIndex] != 0 ? 1 : 0;
		OutOverlapCount += OutOverlapFlags[SampleIndex] != 0 ? 1 : 0;
		RecoveryContainmentCount += RecoveryFlags[SampleIndex] != 0 ? 1 : 0;
		RecoveryContinentalCount += RecoveryContinentalFlags[SampleIndex] != 0 ? 1 : 0;
		if (RecoveryFlags[SampleIndex] != 0 && RecoveryDistances[SampleIndex] >= 0.0)
		{
			RecoveredDistances.Add(RecoveryDistances[SampleIndex]);
		}
	}

	const int32 ExactSingleHitCount = Samples.Num() - OutGapCount - OutOverlapCount - RecoveryContainmentCount;
	const int32 HysteresisRetained = HysteresisRetainedCount.Load();
	const int32 HysteresisReassigned = HysteresisReassignedCount.Load();
	const int32 PreserveOwnershipSamePlateHits = PreserveOwnershipSamePlateHitCount.Load();
	const int32 PreserveOwnershipSamePlateRecoveries = PreserveOwnershipSamePlateRecoveryCount.Load();
	const int32 PreserveOwnershipFallbackQueries = PreserveOwnershipFallbackQueryCount.Load();
	check(ExactSingleHitCount >= 0);
	check(ExactSingleHitCount + OutOverlapCount + RecoveryContainmentCount + OutGapCount == Samples.Num());
	if (bUseStableOverlaps)
	{
		check(HysteresisRetained + HysteresisReassigned == OutOverlapCount);
	}

	double RecoveryMeanDistance = 0.0;
	double RecoveryP95Distance = 0.0;
	double RecoveryMaxDistance = 0.0;
	if (!RecoveredDistances.IsEmpty())
	{
		for (const double Distance : RecoveredDistances)
		{
			RecoveryMeanDistance += Distance;
			RecoveryMaxDistance = FMath::Max(RecoveryMaxDistance, Distance);
		}
		RecoveryMeanDistance /= static_cast<double>(RecoveredDistances.Num());

		Algo::Sort(RecoveredDistances);
		const int32 P95Index = FMath::Clamp(FMath::CeilToInt(0.95 * static_cast<double>(RecoveredDistances.Num())) - 1, 0, RecoveredDistances.Num() - 1);
		RecoveryP95Distance = RecoveredDistances[P95Index];
	}

	if (OutStats != nullptr)
	{
		OutStats->ExactSingleHitCount = ExactSingleHitCount;
		OutStats->ExactMultiHitCount = OutOverlapCount;
		OutStats->RecoveryContainmentCount = RecoveryContainmentCount;
		OutStats->RecoveryContinentalCount = RecoveryContinentalCount;
		OutStats->TrueGapCount = OutGapCount;
		OutStats->RecoveryMeanDistance = RecoveryMeanDistance;
		OutStats->RecoveryP95Distance = RecoveryP95Distance;
		OutStats->RecoveryMaxDistance = RecoveryMaxDistance;
		OutStats->HysteresisRetainedCount = HysteresisRetained;
		OutStats->HysteresisReassignedCount = HysteresisReassigned;
		OutStats->PreserveOwnershipSamePlateHitCount = PreserveOwnershipSamePlateHits;
		OutStats->PreserveOwnershipSamePlateRecoveryCount = PreserveOwnershipSamePlateRecoveries;
		OutStats->PreserveOwnershipFallbackQueryCount = PreserveOwnershipFallbackQueries;
	}
}

void FTectonicPlanet::InterpolateFromCarried(
	const TArray<int32>& NewPlateIds,
	const TArray<int32>& ContainingTriangles,
	const TArray<FVector3d>& BarycentricCoords,
	TArray<float>& OutSubductionDistances,
	TArray<float>& OutSubductionSpeeds,
	int32* OutMissingLocalCarriedLookupCount)
{
	check(NewPlateIds.Num() == Samples.Num());
	check(ContainingTriangles.Num() == Samples.Num());
	check(BarycentricCoords.Num() == Samples.Num());

	OutSubductionDistances.Init(-1.0f, Samples.Num());
	OutSubductionSpeeds.Init(0.0f, Samples.Num());

	TAtomic<int32> MissingCarriedLookupCount(0);
	ParallelFor(Samples.Num(), [this, &NewPlateIds, &ContainingTriangles, &BarycentricCoords, &OutSubductionDistances, &OutSubductionSpeeds, &MissingCarriedLookupCount](const int32 SampleIndex)
	{
		const int32 PlateId = NewPlateIds[SampleIndex];
		const int32 TriangleIndex = ContainingTriangles[SampleIndex];
		const FPlate* ContainingPlate = FindPlateById(*this, PlateId);
		if (ContainingPlate == nullptr)
		{
			return;
		}

		if (!TriangleIndices.IsValidIndex(TriangleIndex))
		{
			return;
		}

		if (!ContainingPlate->SoupData.GlobalToLocalTriangle.Contains(TriangleIndex))
		{
			++MissingCarriedLookupCount;
			return;
		}

		const FIntVector& Triangle = TriangleIndices[TriangleIndex];
		const FCarriedSample* V0 = FindCarriedSampleForCanonicalVertex(*this, PlateId, Triangle.X);
		const FCarriedSample* V1 = FindCarriedSampleForCanonicalVertex(*this, PlateId, Triangle.Y);
		const FCarriedSample* V2 = FindCarriedSampleForCanonicalVertex(*this, PlateId, Triangle.Z);
		if (V0 == nullptr || V1 == nullptr || V2 == nullptr)
		{
			++MissingCarriedLookupCount;
			return;
		}

		FSample& Sample = Samples[SampleIndex];
		ApplyInterpolatedCarriedAttributes(
			Sample,
			Sample.Position.GetSafeNormal(),
			*V0,
			*V1,
			*V2,
			BarycentricCoords[SampleIndex],
			OutSubductionDistances[SampleIndex],
			OutSubductionSpeeds[SampleIndex]);
	});

	const int32 MissingLocalCarriedLookupCount = MissingCarriedLookupCount.Load();
	if (OutMissingLocalCarriedLookupCount != nullptr)
	{
		*OutMissingLocalCarriedLookupCount = MissingLocalCarriedLookupCount;
	}

	if (MissingLocalCarriedLookupCount > 0)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("InterpolateFromCarried skipped %d samples because their containing plate mesh was missing local carried vertices for the containing triangle."),
			MissingLocalCarriedLookupCount);
	}
}

void FTectonicPlanet::ResolveGaps(
	TArray<int32>& NewPlateIds,
	const TArray<uint8>& GapFlags,
	TArray<float>& InOutSubductionDistances,
	TArray<float>& InOutSubductionSpeeds,
	FResamplingStats* InOutStats)
{
	check(NewPlateIds.Num() == Samples.Num());
	check(GapFlags.Num() == Samples.Num());
	check(InOutSubductionDistances.Num() == Samples.Num());
	check(InOutSubductionSpeeds.Num() == Samples.Num());

	if (InOutStats != nullptr)
	{
		InOutStats->DivergentGapCount = 0;
		InOutStats->NonDivergentGapCount = 0;
		InOutStats->DivergentContinentalGapCount = 0;
		InOutStats->NonDivergentContinentalGapCount = 0;
		InOutStats->NonDivergentGapTriangleProjectionCount = 0;
		InOutStats->NonDivergentGapNearestCopyCount = 0;
	}

	const TArray<int32> SnapshotPlateIds = NewPlateIds;
	TArray<FGapResolutionDecision> GapDecisions;
	GapDecisions.SetNum(Samples.Num());
	int32 TotalGapSamples = 0;
	for (const uint8 GapFlag : GapFlags)
	{
		TotalGapSamples += GapFlag != 0 ? 1 : 0;
	}

	auto InitializeOceanicGapSample =
		[this, &NewPlateIds, &InOutSubductionDistances, &InOutSubductionSpeeds](
			const int32 SampleIndex,
			const int32 OwningPlateId,
			const float ElevationKm,
			const FVector3d& RidgeDirection)
		{
			FSample& Sample = Samples[SampleIndex];
			Sample.ContinentalWeight = 0.0f;
			Sample.Age = 0.0f;
			Sample.Thickness = static_cast<float>(OceanicThicknessKm);
			Sample.Elevation = ElevationKm;
			Sample.OrogenyType = EOrogenyType::None;
			Sample.TerraneId = INDEX_NONE;
			Sample.RidgeDirection = RidgeDirection;
			Sample.FoldDirection = FVector3d::ZeroVector;
			InOutSubductionDistances[SampleIndex] = -1.0f;
			InOutSubductionSpeeds[SampleIndex] = 0.0f;
			if (FindPlateById(*this, OwningPlateId) != nullptr)
			{
				NewPlateIds[SampleIndex] = OwningPlateId;
			}
		};

	auto ApplyDivergentGap = [this, &NewPlateIds, &InitializeOceanicGapSample](const int32 SampleIndex)
	{
		TArray<FPlateVoteSummary, TInlineAllocator<8>> NeighborSummaries;
		GatherNeighborPlateSummaries(*this, SampleIndex, NewPlateIds, NeighborSummaries);
		if (NeighborSummaries.IsEmpty())
		{
			InitializeOceanicGapSample(
				SampleIndex,
				INDEX_NONE,
				static_cast<float>(RidgeElevationKm),
				FVector3d::ZeroVector);
			return;
		}

		const int32 PlateA = NeighborSummaries[0].PlateId;
		const int32 PlateB = NeighborSummaries.Num() > 1 ? NeighborSummaries[1].PlateId : INDEX_NONE;
		const FPlate* PlateAData = FindPlateById(*this, PlateA);
		const FPlate* PlateBData = FindPlateById(*this, PlateB);
		if (PlateAData == nullptr || PlateBData == nullptr)
		{
			InitializeOceanicGapSample(
				SampleIndex,
				PlateA,
				static_cast<float>(RidgeElevationKm),
				FVector3d::ZeroVector);
			return;
		}

		const FVector3d QueryPoint = Samples[SampleIndex].Position;
		int32 NearestNeighborA = INDEX_NONE;
		int32 NearestNeighborB = INDEX_NONE;
		double NeighborDistanceA = TNumericLimits<double>::Max();
		double NeighborDistanceB = TNumericLimits<double>::Max();
		FindNearestNeighborForPlate(*this, SampleIndex, NewPlateIds, PlateA, NearestNeighborA, NeighborDistanceA);
		FindNearestNeighborForPlate(*this, SampleIndex, NewPlateIds, PlateB, NearestNeighborB, NeighborDistanceB);

		int32 NearestMemberA = INDEX_NONE;
		int32 NearestMemberB = INDEX_NONE;
		double MemberDistanceA = TNumericLimits<double>::Max();
		double MemberDistanceB = TNumericLimits<double>::Max();
		FindNearestMemberSample(*this, PlateA, QueryPoint, NearestMemberA, MemberDistanceA);
		FindNearestMemberSample(*this, PlateB, QueryPoint, NearestMemberB, MemberDistanceB);

		const bool bHasNeighborA = NearestNeighborA != INDEX_NONE;
		const bool bHasNeighborB = NearestNeighborB != INDEX_NONE;
		const double DP = FMath::Min(
			bHasNeighborA ? NeighborDistanceA : TNumericLimits<double>::Max(),
			bHasNeighborB ? NeighborDistanceB : TNumericLimits<double>::Max());
		const double DGamma = FMath::IsFinite(DP) ? (0.5 * DP) : 0.0;

		float BorderElevation = static_cast<float>(RidgeElevationKm);
		if (bHasNeighborA || bHasNeighborB)
		{
			const int32 NearestNeighborIndex =
				(!bHasNeighborA)
					? NearestNeighborB
					: (!bHasNeighborB || NeighborDistanceA <= NeighborDistanceB ? NearestNeighborA : NearestNeighborB);
			BorderElevation = Samples[NearestNeighborIndex].Elevation;
		}
		else if (Samples.IsValidIndex(NearestMemberA))
		{
			BorderElevation = Samples[NearestMemberA].Elevation;
		}
		else if (Samples.IsValidIndex(NearestMemberB))
		{
			BorderElevation = Samples[NearestMemberB].Elevation;
		}

		const double AlphaDenominator = DGamma + DP;
		const double Alpha = AlphaDenominator > UE_DOUBLE_SMALL_NUMBER ? FMath::Clamp(DGamma / AlphaDenominator, 0.0, 1.0) : 0.0;
		const float ElevationKm = static_cast<float>(FMath::Clamp(
			(Alpha * static_cast<double>(BorderElevation)) + ((1.0 - Alpha) * RidgeElevationKm),
			TrenchElevationKm,
			ElevationCeilingKm));

		const FVector3d RelativeVelocity =
			ComputePlateSurfaceVelocity(*PlateAData, QueryPoint, PlanetRadiusKm) -
			ComputePlateSurfaceVelocity(*PlateBData, QueryPoint, PlanetRadiusKm);
		const FVector3d RidgeDirection = ProjectOntoTangent(RelativeVelocity, QueryPoint);
		const FVector3d NormalizedRidgeDirection = RidgeDirection.SquaredLength() > DirectionDegeneracyThreshold * DirectionDegeneracyThreshold
			? RidgeDirection.GetSafeNormal()
			: FVector3d::ZeroVector;

		int32 OwningPlateId = INDEX_NONE;
		if (MemberDistanceA < MemberDistanceB)
		{
			OwningPlateId = PlateA;
		}
		else if (MemberDistanceB < MemberDistanceA)
		{
			OwningPlateId = PlateB;
		}
		else
		{
			OwningPlateId = FMath::Min(PlateA, PlateB);
		}

		if (FindPlateById(*this, OwningPlateId) == nullptr)
		{
			OwningPlateId = FMath::Min(PlateA, PlateB);
		}

		InitializeOceanicGapSample(SampleIndex, OwningPlateId, ElevationKm, NormalizedRidgeDirection);
	};

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		if (GapFlags[SampleIndex] == 0)
		{
			continue;
		}

		FGapResolutionDecision& Decision = GapDecisions[SampleIndex];
		Decision.bWasContinental = Samples[SampleIndex].ContinentalWeight >= 0.5f;

		TArray<FPlateVoteSummary, TInlineAllocator<8>> NeighborSummaries;
		GatherNeighborPlateSummaries(*this, SampleIndex, SnapshotPlateIds, NeighborSummaries);
		if (NeighborSummaries.IsEmpty())
		{
			continue;
		}

		const int32 PlateA = NeighborSummaries[0].PlateId;
		const int32 PlateB = NeighborSummaries.Num() > 1 ? NeighborSummaries[1].PlateId : INDEX_NONE;
		const FPlate* PlateAData = FindPlateById(*this, PlateA);
		const FPlate* PlateBData = FindPlateById(*this, PlateB);
		if (PlateAData == nullptr || PlateBData == nullptr)
		{
			Decision.Disposition = EGapDisposition::NonDivergent;
			Decision.PrimaryPlateId = PlateA;
			continue;
		}

		const FVector3d QueryPoint = Samples[SampleIndex].Position;
		int32 NearestMemberA = INDEX_NONE;
		int32 NearestMemberB = INDEX_NONE;
		double MemberDistanceA = TNumericLimits<double>::Max();
		double MemberDistanceB = TNumericLimits<double>::Max();
		FindNearestMemberSample(*this, PlateA, QueryPoint, NearestMemberA, MemberDistanceA);
		FindNearestMemberSample(*this, PlateB, QueryPoint, NearestMemberB, MemberDistanceB);

		if (MemberDistanceA < MemberDistanceB ||
			(FMath::IsNearlyEqual(MemberDistanceA, MemberDistanceB, 1.0e-12) && PlateA < PlateB))
		{
			Decision.PrimaryPlateId = PlateA;
			Decision.SecondaryPlateId = PlateB;
		}
		else
		{
			Decision.PrimaryPlateId = PlateB;
			Decision.SecondaryPlateId = PlateA;
		}

		if (NearestMemberA == INDEX_NONE || NearestMemberB == INDEX_NONE)
		{
			Decision.Disposition = EGapDisposition::NonDivergent;
			continue;
		}

		const FVector3d NearestPositionA =
			PlateAData->CumulativeRotation.RotateVector(Samples[NearestMemberA].Position).GetSafeNormal();
		const FVector3d NearestPositionB =
			PlateBData->CumulativeRotation.RotateVector(Samples[NearestMemberB].Position).GetSafeNormal();
		const FVector3d SeparationVector = ProjectOntoTangent(NearestPositionB - NearestPositionA, QueryPoint);
		if (SeparationVector.SquaredLength() <= GapSeparationDirectionEpsilon * GapSeparationDirectionEpsilon)
		{
			Decision.Disposition = EGapDisposition::NonDivergent;
			continue;
		}

		const FVector3d SeparationDirection = SeparationVector.GetSafeNormal();
		const FVector3d VelocityA = ComputePlateSurfaceVelocity(*PlateAData, QueryPoint, PlanetRadiusKm);
		const FVector3d VelocityB = ComputePlateSurfaceVelocity(*PlateBData, QueryPoint, PlanetRadiusKm);
		const double DivergenceScore = (VelocityB - VelocityA).Dot(SeparationDirection);
		Decision.Disposition = DivergenceScore > DivergenceEpsilon
			? EGapDisposition::Divergent
			: EGapDisposition::NonDivergent;
	}

	int32 NonDivergentFallbackOceanizedCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		if (GapFlags[SampleIndex] == 0)
		{
			continue;
		}

		const FGapResolutionDecision& Decision = GapDecisions[SampleIndex];
		if (Decision.Disposition == EGapDisposition::NonDivergent)
		{
			bool bResolved = false;
			const int32 CandidatePlateIds[2] = { Decision.PrimaryPlateId, Decision.SecondaryPlateId };
			for (const int32 CandidatePlateId : CandidatePlateIds)
			{
				if (FindPlateById(*this, CandidatePlateId) == nullptr)
				{
					continue;
				}

				if (TryApplyNearestTriangleProjectionFromPlate(*this, SampleIndex, CandidatePlateId, InOutSubductionDistances, InOutSubductionSpeeds))
				{
					NewPlateIds[SampleIndex] = CandidatePlateId;
					bResolved = true;
					if (InOutStats != nullptr)
					{
						++InOutStats->NonDivergentGapCount;
						InOutStats->NonDivergentContinentalGapCount += Decision.bWasContinental ? 1 : 0;
						++InOutStats->NonDivergentGapTriangleProjectionCount;
					}
					break;
				}

				if (TryApplyNearestCarriedCopyFromPlate(*this, SampleIndex, CandidatePlateId, InOutSubductionDistances, InOutSubductionSpeeds))
				{
					NewPlateIds[SampleIndex] = CandidatePlateId;
					bResolved = true;
					if (InOutStats != nullptr)
					{
						++InOutStats->NonDivergentGapCount;
						InOutStats->NonDivergentContinentalGapCount += Decision.bWasContinental ? 1 : 0;
						++InOutStats->NonDivergentGapNearestCopyCount;
					}
					break;
				}
			}

			if (bResolved)
			{
				continue;
			}

			++NonDivergentFallbackOceanizedCount;
		}

		ApplyDivergentGap(SampleIndex);
		if (InOutStats != nullptr)
		{
			++InOutStats->DivergentGapCount;
			InOutStats->DivergentContinentalGapCount += Decision.bWasContinental ? 1 : 0;
		}
	}

	if (InOutStats != nullptr)
	{
		check(InOutStats->DivergentGapCount + InOutStats->NonDivergentGapCount == TotalGapSamples);
	}

	if (NonDivergentFallbackOceanizedCount > 0)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("ResolveGaps fell back to divergent oceanization for %d non-divergent no-hit samples because no same-plate reprojection/copy source was available."),
			NonDivergentFallbackOceanizedCount);
	}
}

double FTectonicPlanet::SubductionDistanceTransfer(
	const double DistanceRad,
	const double ControlDistanceRad,
	const double MaxDistanceRad)
{
	if (DistanceRad < 0.0 || DistanceRad >= MaxDistanceRad)
	{
		return 0.0;
	}

	const auto Primitive = [ControlDistanceRad, MaxDistanceRad](const double X)
	{
		return (X * X * X / 3.0) -
			((ControlDistanceRad + MaxDistanceRad) * X * X / 2.0) +
			(ControlDistanceRad * MaxDistanceRad * X);
	};

	const double PrimitiveAtDistance = Primitive(DistanceRad);
	const double PrimitiveAtControl = Primitive(ControlDistanceRad);
	const double PrimitiveAtMax = Primitive(MaxDistanceRad);
	const double Denominator = PrimitiveAtControl - PrimitiveAtMax;
	if (FMath::Abs(Denominator) <= UE_DOUBLE_SMALL_NUMBER)
	{
		return 0.0;
	}

	return (PrimitiveAtDistance - PrimitiveAtMax) / Denominator;
}

double FTectonicPlanet::CollisionBiweightKernel(const double DistanceRad, const double RadiusRad)
{
	if (DistanceRad < 0.0 || RadiusRad <= UE_DOUBLE_SMALL_NUMBER || DistanceRad >= RadiusRad)
	{
		return 0.0;
	}

	const double NormalizedDistance = DistanceRad / RadiusRad;
	const double OneMinusSquared = 1.0 - (NormalizedDistance * NormalizedDistance);
	return OneMinusSquared * OneMinusSquared;
}

void FTectonicPlanet::ComputeSubductionDistanceField(FResamplingStats* InOutStats)
{
	const double StartTime = FPlatformTime::Seconds();
	const double MaxDistanceKmForPlanet = SubductionMaxDistanceRad * PlanetRadiusKm;
	const double DistanceComparisonEpsilon = 1.0e-6;

	TArray<double> DistanceKmBySample;
	DistanceKmBySample.Init(TNumericLimits<double>::Max(), Samples.Num());
	TArray<float> SpeedKmPerMyBySample;
	SpeedKmPerMyBySample.Init(0.0f, Samples.Num());

	int32 FrontEdgeCount = 0;
	TMap<int32, float> SeedSpeedBySample;

	for (FSample& Sample : Samples)
	{
		Sample.SubductionDistanceKm = -1.0f;
	}

	const TArray<FConvergentBoundaryEdge> ConvergentEdges = BuildConvergentBoundaryEdges(*this);
	for (const FConvergentBoundaryEdge& Edge : ConvergentEdges)
	{
		++FrontEdgeCount;
		float& SeedSpeedKmPerMy = SeedSpeedBySample.FindOrAdd(Edge.OverridingSampleIndex);
		SeedSpeedKmPerMy = FMath::Max(SeedSpeedKmPerMy, Edge.ConvergenceSpeedKmPerMy);
	}

	TArray<FSubductionQueueEntry> Queue;
	Queue.Reserve(SeedSpeedBySample.Num());
	for (const TPair<int32, float>& Seed : SeedSpeedBySample)
	{
		const int32 SampleIndex = Seed.Key;
		if (!Samples.IsValidIndex(SampleIndex))
		{
			continue;
		}

		DistanceKmBySample[SampleIndex] = 0.0;
		SpeedKmPerMyBySample[SampleIndex] = Seed.Value;
		Queue.HeapPush(
			FSubductionQueueEntry{ SampleIndex, 0.0, Seed.Value },
			FSubductionQueueLess());
	}

	while (!Queue.IsEmpty())
	{
		FSubductionQueueEntry Entry;
		Queue.HeapPop(Entry, FSubductionQueueLess(), EAllowShrinking::No);
		if (!Samples.IsValidIndex(Entry.SampleIndex))
		{
			continue;
		}

		if (Entry.DistanceKm > DistanceKmBySample[Entry.SampleIndex] + DistanceComparisonEpsilon ||
			Entry.DistanceKm > MaxDistanceKmForPlanet + DistanceComparisonEpsilon)
		{
			continue;
		}

		const int32 PlateId = Samples[Entry.SampleIndex].PlateId;
		if (FindPlateById(*this, PlateId) == nullptr)
		{
			continue;
		}

		const FVector3d Position = Samples[Entry.SampleIndex].Position.GetSafeNormal();
		for (const int32 NeighborIndex : SampleAdjacency[Entry.SampleIndex])
		{
			if (!Samples.IsValidIndex(NeighborIndex) || Samples[NeighborIndex].PlateId != PlateId)
			{
				continue;
			}

			const double EdgeDistanceKm =
				ComputeGeodesicDistance(Position, Samples[NeighborIndex].Position.GetSafeNormal()) * PlanetRadiusKm;
			const double CandidateDistanceKm = Entry.DistanceKm + EdgeDistanceKm;
			if (CandidateDistanceKm > MaxDistanceKmForPlanet + DistanceComparisonEpsilon)
			{
				continue;
			}

			const bool bBetterDistance = CandidateDistanceKm + DistanceComparisonEpsilon < DistanceKmBySample[NeighborIndex];
			const bool bTieButFaster =
				FMath::IsNearlyEqual(CandidateDistanceKm, DistanceKmBySample[NeighborIndex], DistanceComparisonEpsilon) &&
				Entry.SpeedKmPerMy > SpeedKmPerMyBySample[NeighborIndex] + UE_KINDA_SMALL_NUMBER;
			if (!bBetterDistance && !bTieButFaster)
			{
				continue;
			}

			DistanceKmBySample[NeighborIndex] = CandidateDistanceKm;
			SpeedKmPerMyBySample[NeighborIndex] = Entry.SpeedKmPerMy;
			Queue.HeapPush(
				FSubductionQueueEntry{ NeighborIndex, CandidateDistanceKm, Entry.SpeedKmPerMy },
				FSubductionQueueLess());
		}
	}

	int32 InfluencedCount = 0;
	double DistanceSumKm = 0.0;
	double MaxObservedDistanceKm = 0.0;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const bool bInfluenced =
			FMath::IsFinite(DistanceKmBySample[SampleIndex]) &&
			DistanceKmBySample[SampleIndex] <= MaxDistanceKmForPlanet + DistanceComparisonEpsilon;
		Samples[SampleIndex].SubductionDistanceKm = bInfluenced
			? static_cast<float>(DistanceKmBySample[SampleIndex])
			: -1.0f;
		if (!bInfluenced)
		{
			SpeedKmPerMyBySample[SampleIndex] = 0.0f;
			continue;
		}

		++InfluencedCount;
		DistanceSumKm += DistanceKmBySample[SampleIndex];
		MaxObservedDistanceKm = FMath::Max(MaxObservedDistanceKm, DistanceKmBySample[SampleIndex]);
	}

	for (FPlate& Plate : Plates)
	{
		for (FCarriedSample& CarriedSample : Plate.CarriedSamples)
		{
			const int32 SampleIndex = CarriedSample.CanonicalSampleIndex;
			if (!Samples.IsValidIndex(SampleIndex))
			{
				CarriedSample.SubductionDistanceKm = -1.0f;
				CarriedSample.SubductionSpeed = 0.0f;
				continue;
			}

			CarriedSample.SubductionDistanceKm = Samples[SampleIndex].SubductionDistanceKm;
			CarriedSample.SubductionSpeed = SpeedKmPerMyBySample[SampleIndex];
		}
	}

	if (InOutStats != nullptr)
	{
		InOutStats->SubductionFrontEdgeCount = FrontEdgeCount;
		InOutStats->SubductionSeedSampleCount = SeedSpeedBySample.Num();
		InOutStats->SubductionInfluencedCount = InfluencedCount;
		InOutStats->SubductionMeanDistanceKm =
			InfluencedCount > 0 ? (DistanceSumKm / static_cast<double>(InfluencedCount)) : 0.0;
		InOutStats->SubductionMaxDistanceKm = MaxObservedDistanceKm;
		InOutStats->SubductionDistanceFieldMs = (FPlatformTime::Seconds() - StartTime) * 1000.0;
	}
}

void FTectonicPlanet::ComputeSlabPullCorrections(FResamplingStats* InOutStats)
{
	const double StartTime = FPlatformTime::Seconds();
	const TArray<FConvergentBoundaryEdge> ConvergentEdges = BuildConvergentBoundaryEdges(*this);
	const double Perturbation =
		(!Samples.IsEmpty() && !Plates.IsEmpty())
			? (SlabPullEpsilon * static_cast<double>(Plates.Num()) / static_cast<double>(Samples.Num()))
			: 0.0;

	TMap<int32, TSet<int32>> FrontSamplesByPlate;
	for (const FConvergentBoundaryEdge& Edge : ConvergentEdges)
	{
		FrontSamplesByPlate.FindOrAdd(Edge.SubductingPlateId).Add(Edge.SubductingSampleIndex);
	}

	int32 SlabPullPlateCount = 0;
	int32 TotalFrontSamples = 0;
	double MaxAxisChangeRad = 0.0;
	for (FPlate& Plate : Plates)
	{
		Plate.SlabPullCorrectionAxis = FVector3d::ZeroVector;
		Plate.SlabPullFrontSampleCount = 0;

		const FVector3d Centroid = ComputePlateCentroidOnUnitSphere(*this, Plate);
		const TSet<int32>* FrontSamples = FrontSamplesByPlate.Find(Plate.Id);
		if (Centroid.IsNearlyZero() || FrontSamples == nullptr)
		{
			continue;
		}

		FVector3d CorrectionSum = FVector3d::ZeroVector;
		for (const int32 SampleIndex : *FrontSamples)
		{
			if (!Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FVector3d Correction =
				FVector3d::CrossProduct(Centroid, Samples[SampleIndex].Position.GetSafeNormal()).GetSafeNormal();
			if (Correction.IsNearlyZero() || !IsFiniteVector(Correction))
			{
				continue;
			}

			CorrectionSum += Correction;
		}

		Plate.SlabPullCorrectionAxis = CorrectionSum;
		Plate.SlabPullFrontSampleCount = FrontSamples->Num();
		TotalFrontSamples += Plate.SlabPullFrontSampleCount;
		if (CorrectionSum.IsNearlyZero())
		{
			continue;
		}

		++SlabPullPlateCount;
		const double RawAxisChangeRad = Perturbation * CorrectionSum.Length() * DeltaTimeMyears;
		MaxAxisChangeRad = FMath::Max(MaxAxisChangeRad, FMath::Min(RawAxisChangeRad, MaxAxisChangePerStepRad));
	}

	if (InOutStats != nullptr)
	{
		InOutStats->SlabPullPlateCount = SlabPullPlateCount;
		InOutStats->SlabPullTotalFrontSamples = TotalFrontSamples;
		InOutStats->SlabPullMaxAxisChangeRad = MaxAxisChangeRad;
		InOutStats->SlabPullMs = (FPlatformTime::Seconds() - StartTime) * 1000.0;
	}
}

void FTectonicPlanet::RepartitionMembership(
	const TArray<int32>& NewPlateIds,
	const TArray<float>* InSubductionDistances,
	const TArray<float>* InSubductionSpeeds)
{
	check(NewPlateIds.Num() == Samples.Num());

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const int32 NewPlateId = NewPlateIds[SampleIndex];
		Samples[SampleIndex].PlateId = FindPlateById(*this, NewPlateId) != nullptr ? NewPlateId : INDEX_NONE;
	}

	RebuildMembershipFromCanonical(*this);
	RecomputeBoundaryFlags(*this);
	ClassifyPlateTriangles(*this);
	RecomputePlateBoundingCaps(*this);
	RebuildCarriedSamplesFromSoupVertices(*this);
	ComputePlateScores();

	if (InSubductionDistances != nullptr || InSubductionSpeeds != nullptr)
	{
		for (FPlate& Plate : Plates)
		{
			for (FCarriedSample& CarriedSample : Plate.CarriedSamples)
			{
				const int32 SampleIndex = CarriedSample.CanonicalSampleIndex;
				if (InSubductionDistances != nullptr && InSubductionDistances->IsValidIndex(SampleIndex))
				{
					CarriedSample.SubductionDistanceKm = (*InSubductionDistances)[SampleIndex];
				}
				if (InSubductionSpeeds != nullptr && InSubductionSpeeds->IsValidIndex(SampleIndex))
				{
					CarriedSample.SubductionSpeed = (*InSubductionSpeeds)[SampleIndex];
				}
			}
		}
	}

	for (FPlate& Plate : Plates)
	{
		ResetSoupState(Plate);
	}
}

void FTectonicPlanet::CollectCollisionCandidates(
	const TArray<uint8>& OverlapFlags,
	const TArray<TArray<int32>>& OverlapPlateIds,
	TArray<FCollisionCandidate>& OutCandidates) const
{
	OutCandidates.Reset();
	if (!bEnableContinentalCollision ||
		OverlapFlags.Num() != Samples.Num() ||
		OverlapPlateIds.Num() != Samples.Num())
	{
		return;
	}

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		if (OverlapFlags[SampleIndex] == 0 || !OverlapPlateIds.IsValidIndex(SampleIndex))
		{
			continue;
		}

		TArray<int32, TInlineAllocator<4>> CandidatePlateIds;
		for (const int32 PlateId : OverlapPlateIds[SampleIndex])
		{
			if (FindPlateById(*this, PlateId) != nullptr)
			{
				CandidatePlateIds.AddUnique(PlateId);
			}
		}

		if (CandidatePlateIds.Num() < 2)
		{
			continue;
		}

		TArray<int32, TInlineAllocator<4>> ContinentalPlateIds;
		const FVector3d QueryPoint = Samples[SampleIndex].Position;
		for (const int32 PlateId : CandidatePlateIds)
		{
			double ContinentalWeight = 0.0;
			if (TryComputeContainedPlateContinentalWeight(*this, PlateId, QueryPoint, ContinentalWeight) &&
				ContinentalWeight >= 0.5)
			{
				ContinentalPlateIds.Add(PlateId);
			}
		}

		for (int32 PlateIndexA = 0; PlateIndexA < ContinentalPlateIds.Num(); ++PlateIndexA)
		{
			for (int32 PlateIndexB = PlateIndexA + 1; PlateIndexB < ContinentalPlateIds.Num(); ++PlateIndexB)
			{
				const int32 PlateA = ContinentalPlateIds[PlateIndexA];
				const int32 PlateB = ContinentalPlateIds[PlateIndexB];
				const bool bPlateAOverriding = IsOverridingPlate(*this, PlateA, PlateB);

				FCollisionCandidate& Candidate = OutCandidates.AddDefaulted_GetRef();
				Candidate.SampleIndex = SampleIndex;
				Candidate.OverridingPlateId = bPlateAOverriding ? PlateA : PlateB;
				Candidate.SubductingPlateId = bPlateAOverriding ? PlateB : PlateA;
			}
		}
	}
}

bool FTectonicPlanet::DetectBoundaryContactCollisionTrigger(
	FResamplingStats* InOutStats,
	FPendingBoundaryContactCollisionEvent* OutPendingEvent) const
{
	if (InOutStats != nullptr)
	{
		InOutStats->BoundaryContactCollisionCandidateCount = 0;
		InOutStats->BoundaryContactCollisionPairCount = 0;
		InOutStats->BoundaryContactLargestZoneSize = 0;
		InOutStats->BoundaryContactTriggerPlateA = INDEX_NONE;
		InOutStats->BoundaryContactTriggerPlateB = INDEX_NONE;
		InOutStats->bBoundaryContactCollisionTriggered = false;
	}
	if (OutPendingEvent != nullptr)
	{
		*OutPendingEvent = FPendingBoundaryContactCollisionEvent{};
	}

	if (!bEnableContinentalCollision || SampleAdjacency.Num() != Samples.Num() || Plates.IsEmpty())
	{
		return false;
	}

	TArray<FBoundaryContactCollisionCandidate> Candidates;
	TSet<uint64> CandidatePairKeys;
	TMap<uint64, TArray<int32>> CandidateIndicesByPair;

	for (int32 SampleIndexA = 0; SampleIndexA < SampleAdjacency.Num(); ++SampleIndexA)
	{
		if (!Samples.IsValidIndex(SampleIndexA))
		{
			continue;
		}

		const FSample& SampleA = Samples[SampleIndexA];
		if (SampleA.ContinentalWeight < 0.5f)
		{
			continue;
		}

		for (const int32 SampleIndexB : SampleAdjacency[SampleIndexA])
		{
			if (SampleIndexB <= SampleIndexA || !Samples.IsValidIndex(SampleIndexB))
			{
				continue;
			}

			const FSample& SampleB = Samples[SampleIndexB];
			if (SampleB.PlateId == SampleA.PlateId ||
				SampleB.ContinentalWeight < 0.5f ||
				FindPlateById(*this, SampleA.PlateId) == nullptr ||
				FindPlateById(*this, SampleB.PlateId) == nullptr)
			{
				continue;
			}

			double NormalComponent = 0.0;
			float ConvergenceSpeedKmPerMy = 0.0f;
			if (!TryComputeConvergentBoundaryEdgeMetrics(
					*this,
					SampleIndexA,
					SampleIndexB,
					NormalComponent,
					ConvergenceSpeedKmPerMy))
			{
				continue;
			}

			FBoundaryContactCollisionCandidate& Candidate = Candidates.AddDefaulted_GetRef();
			Candidate.PlateA = FMath::Min(SampleA.PlateId, SampleB.PlateId);
			Candidate.PlateB = FMath::Max(SampleA.PlateId, SampleB.PlateId);
			Candidate.SampleIndexA = SampleIndexA;
			Candidate.SampleIndexB = SampleIndexB;
			Candidate.ConvergenceMagnitude = static_cast<double>(ConvergenceSpeedKmPerMy);
			Candidate.MinSampleIndex = FMath::Min(SampleIndexA, SampleIndexB);

			const int32 CandidateIndex = Candidates.Num() - 1;
			const uint64 PairKey = MakeBoundaryContactPlatePairKey(Candidate.PlateA, Candidate.PlateB);
			CandidatePairKeys.Add(PairKey);
			CandidateIndicesByPair.FindOrAdd(PairKey).Add(CandidateIndex);
		}
	}

	if (InOutStats != nullptr)
	{
		InOutStats->BoundaryContactCollisionCandidateCount = Candidates.Num();
		InOutStats->BoundaryContactCollisionPairCount = CandidatePairKeys.Num();
	}

	if (Candidates.IsEmpty())
	{
		return false;
	}

	bool bHasBestZone = false;
	FBoundaryContactCollisionZone BestZone;
	for (const TPair<uint64, TArray<int32>>& PairEntry : CandidateIndicesByPair)
	{
		const TArray<int32>& PairCandidateIndices = PairEntry.Value;
		if (PairCandidateIndices.IsEmpty())
		{
			continue;
		}

		TMap<int32, TArray<int32>> CandidateLocalIndicesByEndpoint;
		CandidateLocalIndicesByEndpoint.Reserve(PairCandidateIndices.Num() * 2);
		for (int32 LocalCandidateIndex = 0; LocalCandidateIndex < PairCandidateIndices.Num(); ++LocalCandidateIndex)
		{
			const FBoundaryContactCollisionCandidate& Candidate = Candidates[PairCandidateIndices[LocalCandidateIndex]];
			CandidateLocalIndicesByEndpoint.FindOrAdd(Candidate.SampleIndexA).Add(LocalCandidateIndex);
			CandidateLocalIndicesByEndpoint.FindOrAdd(Candidate.SampleIndexB).Add(LocalCandidateIndex);
		}

		TArray<int32> Parents;
		Parents.Reserve(PairCandidateIndices.Num());
		for (int32 LocalCandidateIndex = 0; LocalCandidateIndex < PairCandidateIndices.Num(); ++LocalCandidateIndex)
		{
			Parents.Add(LocalCandidateIndex);
		}

		auto FindRoot = [&Parents](int32 Index)
		{
			while (Parents[Index] != Index)
			{
				Parents[Index] = Parents[Parents[Index]];
				Index = Parents[Index];
			}
			return Index;
		};

		auto UnionLocalCandidates = [&FindRoot, &Parents](const int32 LeftIndex, const int32 RightIndex)
		{
			const int32 LeftRoot = FindRoot(LeftIndex);
			const int32 RightRoot = FindRoot(RightIndex);
			if (LeftRoot != RightRoot)
			{
				Parents[RightRoot] = LeftRoot;
			}
		};

		for (int32 LocalCandidateIndex = 0; LocalCandidateIndex < PairCandidateIndices.Num(); ++LocalCandidateIndex)
		{
			const FBoundaryContactCollisionCandidate& Candidate = Candidates[PairCandidateIndices[LocalCandidateIndex]];
			const int32 CandidateEndpoints[2] = { Candidate.SampleIndexA, Candidate.SampleIndexB };
			for (const int32 EndpointSampleIndex : CandidateEndpoints)
			{
				if (const TArray<int32>* SharedEndpointCandidates =
						CandidateLocalIndicesByEndpoint.Find(EndpointSampleIndex))
				{
					for (const int32 OtherLocalCandidateIndex : *SharedEndpointCandidates)
					{
						UnionLocalCandidates(LocalCandidateIndex, OtherLocalCandidateIndex);
					}
				}

				if (!SampleAdjacency.IsValidIndex(EndpointSampleIndex))
				{
					continue;
				}

				for (const int32 NeighborSampleIndex : SampleAdjacency[EndpointSampleIndex])
				{
					if (const TArray<int32>* AdjacentEndpointCandidates =
							CandidateLocalIndicesByEndpoint.Find(NeighborSampleIndex))
					{
						for (const int32 OtherLocalCandidateIndex : *AdjacentEndpointCandidates)
						{
							UnionLocalCandidates(LocalCandidateIndex, OtherLocalCandidateIndex);
						}
					}
				}
			}
		}

		TMap<int32, FBoundaryContactCollisionZone> ZonesByRoot;
		for (int32 LocalCandidateIndex = 0; LocalCandidateIndex < PairCandidateIndices.Num(); ++LocalCandidateIndex)
		{
			const int32 RootIndex = FindRoot(LocalCandidateIndex);
			const FBoundaryContactCollisionCandidate& Candidate = Candidates[PairCandidateIndices[LocalCandidateIndex]];
			FBoundaryContactCollisionZone& Zone = ZonesByRoot.FindOrAdd(RootIndex);
			Zone.PlateA = Candidate.PlateA;
			Zone.PlateB = Candidate.PlateB;
			++Zone.CandidateCount;
			Zone.TotalConvergenceMagnitude += Candidate.ConvergenceMagnitude;
			Zone.MinSampleIndex = Zone.MinSampleIndex == INDEX_NONE
				? Candidate.MinSampleIndex
				: FMath::Min(Zone.MinSampleIndex, Candidate.MinSampleIndex);
			Zone.SeedSampleIndices.AddUnique(Candidate.SampleIndexA);
			Zone.SeedSampleIndices.AddUnique(Candidate.SampleIndexB);
			if (Samples.IsValidIndex(Candidate.SampleIndexA))
			{
				Zone.ContactCenterSum += Samples[Candidate.SampleIndexA].Position.GetSafeNormal();
			}
			if (Samples.IsValidIndex(Candidate.SampleIndexB))
			{
				Zone.ContactCenterSum += Samples[Candidate.SampleIndexB].Position.GetSafeNormal();
			}
		}

		for (const TPair<int32, FBoundaryContactCollisionZone>& ZoneEntry : ZonesByRoot)
		{
			if (!bHasBestZone || IsBoundaryContactZoneStronger(ZoneEntry.Value, BestZone))
			{
				BestZone = ZoneEntry.Value;
				bHasBestZone = true;
			}
		}
	}

	if (InOutStats != nullptr && bHasBestZone)
	{
		InOutStats->BoundaryContactLargestZoneSize = BestZone.CandidateCount;
		InOutStats->BoundaryContactTriggerPlateA = BestZone.PlateA;
		InOutStats->BoundaryContactTriggerPlateB = BestZone.PlateB;
	}
	if (OutPendingEvent != nullptr && bHasBestZone)
	{
		OutPendingEvent->bValid = !BestZone.SeedSampleIndices.IsEmpty();
		OutPendingEvent->PlateA = BestZone.PlateA;
		OutPendingEvent->PlateB = BestZone.PlateB;
		OutPendingEvent->BoundarySeedSampleIndices = BestZone.SeedSampleIndices;
		OutPendingEvent->ContactCenter = BestZone.ContactCenterSum.GetSafeNormal();
	}

	const bool bTriggered = bHasBestZone && BestZone.CandidateCount >= MinBoundaryContactCollisionEdges;
	if (InOutStats != nullptr)
	{
		InOutStats->bBoundaryContactCollisionTriggered = bTriggered;
	}

	if (bTriggered)
	{
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[BoundaryContactCollision Step=%d] plate_pair=(%d,%d) zone_size=%d threshold=%d"),
			CurrentStep,
			BestZone.PlateA,
			BestZone.PlateB,
			BestZone.CandidateCount,
			MinBoundaryContactCollisionEdges);
	}

	return bTriggered;
}

bool FTectonicPlanet::UpdateBoundaryContactCollisionPersistence(
	FResamplingStats& InOutStats,
	const int32 ResampleInterval)
{
	InOutStats.BoundaryContactPersistencePairA = INDEX_NONE;
	InOutStats.BoundaryContactPersistencePairB = INDEX_NONE;
	InOutStats.BoundaryContactPersistenceCount = 0;
	InOutStats.bBoundaryContactPersistenceTriggered = false;

	if (ResamplingPolicy != EResamplingPolicy::PreserveOwnershipPeriodic)
	{
		ResetBoundaryContactCollisionPersistence();
		return false;
	}

	for (auto PersistenceIt = BoundaryContactPersistenceByPair.CreateIterator(); PersistenceIt; ++PersistenceIt)
	{
		const FBoundaryContactPersistence& Persistence = PersistenceIt.Value();
		if (Persistence.LastObservedStep == INDEX_NONE ||
			CurrentStep <= Persistence.LastObservedStep ||
			CurrentStep - Persistence.LastObservedStep > ResampleInterval)
		{
			PersistenceIt.RemoveCurrent();
		}
	}

	if (InOutStats.BoundaryContactTriggerPlateA == INDEX_NONE ||
		InOutStats.BoundaryContactTriggerPlateB == INDEX_NONE ||
		InOutStats.BoundaryContactLargestZoneSize <= 0)
	{
		ResetBoundaryContactCollisionPersistence();
		return false;
	}

	const uint64 PairKey = MakeBoundaryContactPlatePairKey(
		InOutStats.BoundaryContactTriggerPlateA,
		InOutStats.BoundaryContactTriggerPlateB);
	const bool bHadExistingObservation = BoundaryContactPersistenceByPair.Contains(PairKey);

	for (auto PersistenceIt = BoundaryContactPersistenceByPair.CreateIterator(); PersistenceIt; ++PersistenceIt)
	{
		if (PersistenceIt.Key() != PairKey)
		{
			PersistenceIt.RemoveCurrent();
		}
	}

	FBoundaryContactPersistence& Persistence = BoundaryContactPersistenceByPair.FindOrAdd(PairKey);
	const bool bConsecutiveObservation =
		bHadExistingObservation &&
		Persistence.LastObservedStep != INDEX_NONE &&
		CurrentStep > Persistence.LastObservedStep &&
		CurrentStep - Persistence.LastObservedStep <= ResampleInterval;
	if (!bConsecutiveObservation)
	{
		Persistence.ConsecutiveResamples = 0;
		Persistence.PeakLargestZoneSize = 0;
	}

	Persistence.PlateA = InOutStats.BoundaryContactTriggerPlateA;
	Persistence.PlateB = InOutStats.BoundaryContactTriggerPlateB;
	++Persistence.ConsecutiveResamples;
	Persistence.LastObservedStep = CurrentStep;
	Persistence.LastCandidateCount = InOutStats.BoundaryContactCollisionCandidateCount;
	Persistence.LastLargestZoneSize = InOutStats.BoundaryContactLargestZoneSize;
	Persistence.PeakLargestZoneSize = FMath::Max(
		Persistence.PeakLargestZoneSize,
		InOutStats.BoundaryContactLargestZoneSize);

	InOutStats.BoundaryContactPersistencePairA = Persistence.PlateA;
	InOutStats.BoundaryContactPersistencePairB = Persistence.PlateB;
	InOutStats.BoundaryContactPersistenceCount = Persistence.ConsecutiveResamples;

	// Persistence lets a thin preserve-mode contact mature into a structural event
	// without lowering the immediate spatial threshold all the way to noisy singletons.
	const int32 PersistenceSpatialThreshold = FMath::Max(1, MinBoundaryContactCollisionEdges - 1);
	const bool bPersistenceTriggered =
		Persistence.ConsecutiveResamples >= MinBoundaryContactPersistenceResamples &&
		Persistence.PeakLargestZoneSize >= PersistenceSpatialThreshold;
	InOutStats.bBoundaryContactPersistenceTriggered = bPersistenceTriggered;

	if (bPersistenceTriggered)
	{
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[BoundaryContactCollisionPersistence Step=%d] plate_pair=(%d,%d) consecutive_resamples=%d peak_zone_size=%d current_zone_size=%d threshold=%d"),
			CurrentStep,
			Persistence.PlateA,
			Persistence.PlateB,
			Persistence.ConsecutiveResamples,
			Persistence.PeakLargestZoneSize,
			Persistence.LastLargestZoneSize,
			PersistenceSpatialThreshold);
	}

	return bPersistenceTriggered;
}

void FTectonicPlanet::ResetBoundaryContactCollisionPersistence()
{
	BoundaryContactPersistenceByPair.Reset();
	PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	bPendingBoundaryContactPersistenceReset = false;
}

bool FTectonicPlanet::DetectAndApplyCollision(
	const EResampleTriggerReason TriggerReason,
	const TArray<uint8>& OverlapFlags,
	const TArray<TArray<int32>>& OverlapPlateIds,
	const TArray<int32>& PreviousPlateIds,
	const TArray<float>& PreviousContinentalWeights,
	const TArray<int32>& PreviousTerraneAssignments,
	TArray<int32>& InOutNewPlateIds,
	FCollisionEvent& OutEvent,
	FResamplingStats* InOutStats) const
{
	OutEvent = FCollisionEvent{};
	if (InOutStats != nullptr)
	{
		InOutStats->CollisionCount = 0;
		InOutStats->CollisionDeferredCount = 0;
		InOutStats->CollisionTerraneId = INDEX_NONE;
		InOutStats->CollisionTerraneSampleCount = 0;
		InOutStats->CollisionOverridingPlateId = INDEX_NONE;
		InOutStats->CollisionSubductingPlateId = INDEX_NONE;
		InOutStats->CollisionSurgeAffectedCount = 0;
		InOutStats->CollisionSurgeRadiusRad = 0.0;
		InOutStats->CollisionSurgeMeanElevationDelta = 0.0;
		InOutStats->bUsedCachedBoundaryContactCollision = false;
		InOutStats->CachedBoundaryContactSeedCount = 0;
		InOutStats->CachedBoundaryContactTerraneSeedCount = 0;
		InOutStats->CachedBoundaryContactTerraneRecoveredCount = 0;
		InOutStats->CachedBoundaryContactPlateA = INDEX_NONE;
		InOutStats->CachedBoundaryContactPlateB = INDEX_NONE;
	}

	if (!bEnableContinentalCollision ||
		OverlapFlags.Num() != Samples.Num() ||
		OverlapPlateIds.Num() != Samples.Num() ||
		PreviousPlateIds.Num() != Samples.Num() ||
		PreviousContinentalWeights.Num() != Samples.Num() ||
		PreviousTerraneAssignments.Num() != Samples.Num() ||
		InOutNewPlateIds.Num() != Samples.Num())
	{
		return false;
	}

	if (TriggerReason == EResampleTriggerReason::CollisionFollowup &&
		PendingBoundaryContactCollisionEvent.bValid)
	{
		int32 TerraneSeedCount = 0;
		int32 TerraneRecoveredCount = 0;
		const TCHAR* FailureReason = TEXT("none");
		if (InOutStats != nullptr)
		{
			InOutStats->CachedBoundaryContactSeedCount =
				PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices.Num();
			InOutStats->CachedBoundaryContactPlateA = PendingBoundaryContactCollisionEvent.PlateA;
			InOutStats->CachedBoundaryContactPlateB = PendingBoundaryContactCollisionEvent.PlateB;
		}

		if (TryBuildCollisionEventFromBoundaryContactSeeds(
				*this,
				PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices,
				PendingBoundaryContactCollisionEvent.PlateA,
				PendingBoundaryContactCollisionEvent.PlateB,
				PendingBoundaryContactCollisionEvent.ContactCenter,
				PreviousPlateIds,
				PreviousContinentalWeights,
				PreviousTerraneAssignments,
				InOutNewPlateIds,
				OutEvent,
				TerraneSeedCount,
				TerraneRecoveredCount,
				FailureReason))
		{
			if (InOutStats != nullptr)
			{
				InOutStats->CollisionCount = 1;
				InOutStats->CollisionTerraneId = OutEvent.TerraneId;
				InOutStats->CollisionTerraneSampleCount = OutEvent.TerraneSampleIndices.Num();
				InOutStats->CollisionOverridingPlateId = OutEvent.OverridingPlateId;
				InOutStats->CollisionSubductingPlateId = OutEvent.SubductingPlateId;
				InOutStats->bUsedCachedBoundaryContactCollision = true;
				InOutStats->CachedBoundaryContactTerraneSeedCount = TerraneSeedCount;
				InOutStats->CachedBoundaryContactTerraneRecoveredCount = TerraneRecoveredCount;
			}

			UE_LOG(
				LogTemp,
				Log,
				TEXT("[CachedBoundaryContactCollision Step=%d] pair=(%d,%d) seed_count=%d terrane_seed_count=%d terrane_recovered=%d over_plate=%d sub_plate=%d"),
				CurrentStep,
				PendingBoundaryContactCollisionEvent.PlateA,
				PendingBoundaryContactCollisionEvent.PlateB,
				PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices.Num(),
				TerraneSeedCount,
				TerraneRecoveredCount,
				OutEvent.OverridingPlateId,
				OutEvent.SubductingPlateId);
			return true;
		}

		if (InOutStats != nullptr)
		{
			InOutStats->CachedBoundaryContactTerraneSeedCount = TerraneSeedCount;
			InOutStats->CachedBoundaryContactTerraneRecoveredCount = TerraneRecoveredCount;
		}

		UE_LOG(
			LogTemp,
			Log,
			TEXT("[CachedBoundaryContactCollisionRejected Step=%d] pair=(%d,%d) seed_count=%d rejection_reason=%s"),
			CurrentStep,
			PendingBoundaryContactCollisionEvent.PlateA,
			PendingBoundaryContactCollisionEvent.PlateB,
			PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices.Num(),
			FailureReason);
	}

	TArray<FCollisionCandidate> Candidates;
	CollectCollisionCandidates(OverlapFlags, OverlapPlateIds, Candidates);

	TSet<uint64> CandidatePairKeys;
	for (const FCollisionCandidate& Candidate : Candidates)
	{
		if (FindPlateById(*this, Candidate.OverridingPlateId) != nullptr &&
			FindPlateById(*this, Candidate.SubductingPlateId) != nullptr)
		{
			CandidatePairKeys.Add(MakeCollisionPlatePairKey(Candidate.OverridingPlateId, Candidate.SubductingPlateId));
		}
	}

	auto LogCollisionDiag = [this](const int32 CandidateCount,
		const int32 PairCount,
		const int32 LargestOverridingPlateId,
		const int32 LargestSubductingPlateId,
		const int32 LargestRelaxedComponentSize,
		const int32 BfsTerraneSize,
		const TCHAR* RejectionReason)
	{
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[CollisionDiag Step=%d] cc_candidates=%d plate_pairs=%d largest_pair=(%d,%d) largest_relaxed_component=%d bfs_terrane_size=%d rejection_reason=%s"),
			CurrentStep,
			CandidateCount,
			PairCount,
			LargestOverridingPlateId,
			LargestSubductingPlateId,
			LargestRelaxedComponentSize,
			BfsTerraneSize,
			RejectionReason);
	};

	if (Candidates.IsEmpty())
	{
		LogCollisionDiag(0, 0, INDEX_NONE, INDEX_NONE, 0, 0, TEXT("no_cc_candidates"));
		return false;
	}

	TArray<FCollisionComponent> RawComponents;
	BuildCollisionComponents(*this, Candidates, RawComponents);

	const auto CollisionComponentSort = [](const FCollisionComponent& Left, const FCollisionComponent& Right)
	{
		if (Left.SampleIndices.Num() != Right.SampleIndices.Num())
		{
			return Left.SampleIndices.Num() > Right.SampleIndices.Num();
		}
		if (Left.OverridingPlateId != Right.OverridingPlateId)
		{
			return Left.OverridingPlateId < Right.OverridingPlateId;
		}
		if (Left.SubductingPlateId != Right.SubductingPlateId)
		{
			return Left.SubductingPlateId < Right.SubductingPlateId;
		}
		return Left.MinSampleIndex < Right.MinSampleIndex;
	};

	Algo::Sort(RawComponents, CollisionComponentSort);

	int32 LargestComponentOverridingPlateId = INDEX_NONE;
	int32 LargestComponentSubductingPlateId = INDEX_NONE;
	int32 LargestComponentSize = 0;
	if (!RawComponents.IsEmpty())
	{
		LargestComponentOverridingPlateId = RawComponents[0].OverridingPlateId;
		LargestComponentSubductingPlateId = RawComponents[0].SubductingPlateId;
		LargestComponentSize = RawComponents[0].SampleIndices.Num();
	}

	TArray<FCollisionComponent> Components;
	for (const FCollisionComponent& Component : RawComponents)
	{
		if (Component.SampleIndices.Num() >= MinCollisionOverlapSamples)
		{
			Components.Add(Component);
		}
	}

	if (Components.IsEmpty())
	{
		LogCollisionDiag(
			Candidates.Num(),
			CandidatePairKeys.Num(),
			LargestComponentOverridingPlateId,
			LargestComponentSubductingPlateId,
			LargestComponentSize,
			0,
			TEXT("components_below_threshold"));
		return false;
	}

	Algo::Sort(Components, CollisionComponentSort);

	if (InOutStats != nullptr)
	{
		InOutStats->CollisionDeferredCount = FMath::Max(Components.Num() - 1, 0);
	}

	const FCollisionComponent& SelectedComponent = Components[0];
	FVector3d CollisionCenterSum = FVector3d::ZeroVector;
	for (const int32 SampleIndex : SelectedComponent.SampleIndices)
	{
		CollisionCenterSum += Samples[SampleIndex].Position.GetSafeNormal();
	}

	int32 TerraneSeedCount = 0;
	int32 TerraneRecoveredCount = 0;
	const TCHAR* FailureReason = TEXT("none");
	if (!TryBuildCollisionEventFromBoundaryContactSeeds(
			*this,
			SelectedComponent.SampleIndices,
			SelectedComponent.OverridingPlateId,
			SelectedComponent.SubductingPlateId,
			CollisionCenterSum.GetSafeNormal(),
			PreviousPlateIds,
			PreviousContinentalWeights,
			PreviousTerraneAssignments,
			InOutNewPlateIds,
			OutEvent,
			TerraneSeedCount,
			TerraneRecoveredCount,
			FailureReason))
	{
		LogCollisionDiag(
			Candidates.Num(),
			CandidatePairKeys.Num(),
			LargestComponentOverridingPlateId,
			LargestComponentSubductingPlateId,
			LargestComponentSize,
			TerraneRecoveredCount,
			FailureReason);
		return false;
	}

	if (InOutStats != nullptr)
	{
		InOutStats->CollisionCount = 1;
		InOutStats->CollisionTerraneId = OutEvent.TerraneId;
		InOutStats->CollisionTerraneSampleCount = OutEvent.TerraneSampleIndices.Num();
		InOutStats->CollisionOverridingPlateId = OutEvent.OverridingPlateId;
		InOutStats->CollisionSubductingPlateId = OutEvent.SubductingPlateId;
	}

	LogCollisionDiag(
		Candidates.Num(),
		CandidatePairKeys.Num(),
		LargestComponentOverridingPlateId,
		LargestComponentSubductingPlateId,
		LargestComponentSize,
		OutEvent.TerraneSampleIndices.Num(),
		TEXT("none"));

	return true;
}

void FTectonicPlanet::ApplyCollisionElevationSurge(
	const FCollisionEvent& CollisionEvent,
	FResamplingStats* InOutStats)
{
	if (!bEnableContinentalCollision ||
		!CollisionEvent.bDetected ||
		Samples.IsEmpty() ||
		FindPlateById(*this, CollisionEvent.OverridingPlateId) == nullptr ||
		FindPlateById(*this, CollisionEvent.SubductingPlateId) == nullptr)
	{
		return;
	}

	FVector3d CollisionCenter = CollisionEvent.CollisionCenter.GetSafeNormal();
	if (CollisionCenter.IsNearlyZero() && !CollisionEvent.CollisionSampleIndices.IsEmpty())
	{
		CollisionCenter = Samples[CollisionEvent.CollisionSampleIndices[0]].Position.GetSafeNormal();
	}
	if (CollisionCenter.IsNearlyZero())
	{
		return;
	}

	const FPlate* OverridingPlate = FindPlateById(*this, CollisionEvent.OverridingPlateId);
	const FPlate* SubductingPlate = FindPlateById(*this, CollisionEvent.SubductingPlateId);
	check(OverridingPlate != nullptr);
	check(SubductingPlate != nullptr);
	const FVector3d OverridingAxis = OverridingPlate->RotationAxis.IsNearlyZero()
		? FVector3d::ZeroVector
		: OverridingPlate->RotationAxis.GetSafeNormal();
	const FVector3d SubductingAxis = SubductingPlate->RotationAxis.IsNearlyZero()
		? FVector3d::ZeroVector
		: SubductingPlate->RotationAxis.GetSafeNormal();
	const FVector3d OverridingVelocity =
		FVector3d::CrossProduct(OverridingAxis * OverridingPlate->AngularSpeed, CollisionCenter);
	const FVector3d SubductingVelocity =
		FVector3d::CrossProduct(SubductingAxis * SubductingPlate->AngularSpeed, CollisionCenter);
	const double RelativeSpeedKmPerMy =
		((OverridingVelocity - SubductingVelocity).Length() * PlanetRadiusKm) / FMath::Max(DeltaTimeMyears, UE_DOUBLE_SMALL_NUMBER);

	const double AverageSampleAreaSr = (4.0 * PI) / static_cast<double>(Samples.Num());
	const double Xi =
		FMath::Sqrt(FMath::Max(RelativeSpeedKmPerMy, 0.0) / FMath::Max(MaxPlateSpeedKmPerMy, UE_DOUBLE_SMALL_NUMBER)) *
		static_cast<double>(CollisionEvent.TerraneSampleIndices.Num()) *
		AverageSampleAreaSr;
	const double SurgeRadiusRad = FMath::Max(
		CollisionGlobalDistanceRad * FMath::Min(Xi, 1.0),
		MinCollisionRadiusRad);

	TArray<int32> AffectedSampleIndices;
	AffectedSampleIndices.Reserve(Samples.Num() / 8);
	int32 SurgeAffectedCount = 0;
	int32 HimalayanAffectedCount = 0;
	int32 CWBoostedCount = 0;
	double TotalElevationDelta = 0.0;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		FSample& Sample = Samples[SampleIndex];
		const double DistanceRad = ComputeGeodesicDistance(Sample.Position.GetSafeNormal(), CollisionCenter);
		if (DistanceRad >= SurgeRadiusRad)
		{
			continue;
		}

		const double Kernel = CollisionBiweightKernel(DistanceRad, SurgeRadiusRad);
		if (Kernel <= 0.0)
		{
			continue;
		}

		const float PreviousElevation = Sample.Elevation;
		const float PreviousContinentalWeight = Sample.ContinentalWeight;
		const double ElevationDeltaKm =
			CollisionCoefficient *
			(static_cast<double>(CollisionEvent.TerraneSampleIndices.Num()) / static_cast<double>(Samples.Num())) *
			Kernel *
			PlanetRadiusKm;

		Sample.Elevation = FMath::Clamp(
			static_cast<float>(static_cast<double>(Sample.Elevation) + ElevationDeltaKm),
			static_cast<float>(TrenchElevationKm),
			static_cast<float>(ElevationCeilingKm));
		if (Sample.Elevation < 0.0f)
		{
			Sample.Elevation = 0.0f;
		}
		if (Sample.Elevation > 0.0f)
		{
			Sample.OrogenyType = EOrogenyType::Himalayan;
		}

		Sample.ContinentalWeight = 1.0f;

		TotalElevationDelta += static_cast<double>(Sample.Elevation - PreviousElevation);
		CWBoostedCount += Sample.ContinentalWeight > PreviousContinentalWeight ? 1 : 0;
		HimalayanAffectedCount += Sample.OrogenyType == EOrogenyType::Himalayan ? 1 : 0;
		AffectedSampleIndices.Add(SampleIndex);
		++SurgeAffectedCount;
	}

	// Keep carried state aligned so the cleanup survives the next canonical sync.
	SyncCarriedSamplesFromCanonicalIndices(*this, AffectedSampleIndices);

	if (InOutStats != nullptr)
	{
		InOutStats->CollisionSurgeAffectedCount = SurgeAffectedCount;
		InOutStats->CollisionSurgeRadiusRad = SurgeRadiusRad;
		InOutStats->CollisionSurgeMeanElevationDelta =
			SurgeAffectedCount > 0
				? TotalElevationDelta / static_cast<double>(SurgeAffectedCount)
				: 0.0;
	}

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Collision Step=%d] terrane_id=%d sub_plate=%d over_plate=%d terrane_samples=%d surge_radius_rad=%.6f affected=%d mean_elev_delta=%.6f himalayan_count=%d cw_boosted=%d"),
		CurrentStep,
		CollisionEvent.TerraneId,
		CollisionEvent.SubductingPlateId,
		CollisionEvent.OverridingPlateId,
		CollisionEvent.TerraneSampleIndices.Num(),
		SurgeRadiusRad,
		SurgeAffectedCount,
		SurgeAffectedCount > 0 ? TotalElevationDelta / static_cast<double>(SurgeAffectedCount) : 0.0,
		HimalayanAffectedCount,
		CWBoostedCount);
}

void FTectonicPlanet::ClassifyOverlaps(
	const TArray<uint8>& OverlapFlags,
	const TArray<int32>& NewPlateIds,
	FResamplingStats& Stats) const
{
	ClassifyOverlapsImpl(*this, OverlapFlags, NewPlateIds, nullptr, Stats);
}

void FTectonicPlanet::DetectTerranes(const TArray<FTerrane>& PreviousTerranes)
{
	TArray<int32> PreviousTerraneAssignments;
	PreviousTerraneAssignments.Reserve(Samples.Num());
	for (const FSample& Sample : Samples)
	{
		PreviousTerraneAssignments.Add(Sample.TerraneId);
	}

	DetectTerranesImpl(*this, PreviousTerranes, PreviousTerraneAssignments, nullptr);
	SyncCarriedTerraneIdsFromCanonical(*this);
}

void FTectonicPlanet::PerformResampling(
	const EResampleOwnershipMode OwnershipMode,
	const EResampleTriggerReason TriggerReason)
{
	ComputePlateScores();
	const int32 ResampleInterval = ComputeResampleInterval();
	LastComputedResampleInterval = ResampleInterval;
	const double AverageSampleSpacingKm =
		Samples.IsEmpty()
			? 0.0
			: PlanetRadiusKm * FMath::Sqrt((4.0 * PI) / static_cast<double>(Samples.Num()));

	double MaxAngularSpeed = 0.0;
	for (const FPlate& Plate : Plates)
	{
		MaxAngularSpeed = FMath::Max(MaxAngularSpeed, Plate.AngularSpeed);
	}
	const double MaxDisplacementKmPerStep = MaxAngularSpeed * PlanetRadiusKm;

	FResamplingStats Stats;
	Stats.Step = CurrentStep;
	Stats.Interval = ResampleInterval;
	Stats.RiftCount = 0;
	Stats.RiftParentPlateId = INDEX_NONE;
	Stats.RiftChildCount = 0;
	Stats.RiftChildPlateA = INDEX_NONE;
	Stats.RiftChildPlateB = INDEX_NONE;
	Stats.RiftParentSampleCount = 0;
	Stats.RiftChildSampleCountA = 0;
	Stats.RiftChildSampleCountB = 0;
	Stats.RiftMinChildSampleCount = 0;
	Stats.RiftMaxChildSampleCount = 0;
	Stats.RiftTotalChildBoundaryContactEdges = 0;
	Stats.RiftTerraneCountBefore = 0;
	Stats.RiftTerraneCountAfter = 0;
	Stats.RiftTerraneFragmentsOnChildA = 0;
	Stats.RiftTerraneFragmentsOnChildB = 0;
	Stats.RiftTerraneSplitCount = 0;
	Stats.RiftTerranePreservedCount = 0;
	Stats.RiftParentContinentalSampleCount = 0;
	Stats.RiftEventSeed = 0;
	Stats.RiftDivergentChildBoundaryEdgeCount = 0;
	Stats.bRiftWasAutomatic = false;
	Stats.RiftMs = 0.0;
	Stats.RiftParentContinentalFraction = 0.0;
	Stats.RiftTriggerProbability = 0.0;
	if (TriggerReason == EResampleTriggerReason::RiftFollowup && PendingRiftEvent.bValid)
	{
		Stats.RiftCount = 1;
		Stats.bRiftWasAutomatic = PendingRiftEvent.bAutomatic;
		Stats.RiftParentPlateId = PendingRiftEvent.ParentPlateId;
		Stats.RiftChildCount = PendingRiftEvent.ChildCount;
		Stats.RiftChildPlateA = PendingRiftEvent.ChildPlateA;
		Stats.RiftChildPlateB = PendingRiftEvent.ChildPlateB;
		Stats.RiftParentSampleCount = PendingRiftEvent.ParentSampleCount;
		Stats.RiftParentContinentalSampleCount = PendingRiftEvent.ParentContinentalSampleCount;
		Stats.RiftChildSampleCountA = PendingRiftEvent.ChildSampleCountA;
		Stats.RiftChildSampleCountB = PendingRiftEvent.ChildSampleCountB;
		Stats.RiftEventSeed = PendingRiftEvent.EventSeed;
		Stats.RiftChildPlateIds = PendingRiftEvent.ChildPlateIds;
		Stats.RiftChildSampleCounts = PendingRiftEvent.ChildSampleCounts;
		Stats.RiftParentContinentalFraction = PendingRiftEvent.ParentContinentalFraction;
		Stats.RiftTriggerProbability = PendingRiftEvent.TriggerProbability;
		if (!Stats.RiftChildSampleCounts.IsEmpty())
		{
			Stats.RiftMinChildSampleCount = Stats.RiftChildSampleCounts[0];
			Stats.RiftMaxChildSampleCount = Stats.RiftChildSampleCounts[0];
			int64 TotalChildSamples = 0;
			for (const int32 ChildSampleCount : Stats.RiftChildSampleCounts)
			{
				Stats.RiftMinChildSampleCount = FMath::Min(Stats.RiftMinChildSampleCount, ChildSampleCount);
				Stats.RiftMaxChildSampleCount = FMath::Max(Stats.RiftMaxChildSampleCount, ChildSampleCount);
				TotalChildSamples += ChildSampleCount;
			}
			Stats.RiftMeanChildSampleCount =
				static_cast<double>(TotalChildSamples) /
				static_cast<double>(Stats.RiftChildSampleCounts.Num());
		}
		Stats.RiftMs = PendingRiftEvent.RiftMs;
	}
	const double TotalStartTime = FPlatformTime::Seconds();
	const TArray<FTerrane> PreviousTerranes = Terranes;
	TArray<int32> PreviousPlateAssignments;
	PreviousPlateAssignments.Reserve(Samples.Num());
	TArray<float> PreviousContinentalWeights;
	PreviousContinentalWeights.Reserve(Samples.Num());
	TArray<int32> PreviousTerraneAssignments;
	PreviousTerraneAssignments.Reserve(Samples.Num());
	for (const FSample& Sample : Samples)
	{
		PreviousPlateAssignments.Add(Sample.PlateId);
		PreviousContinentalWeights.Add(Sample.ContinentalWeight);
		PreviousTerraneAssignments.Add(Sample.TerraneId);
	}

	const double SoupStartTime = FPlatformTime::Seconds();
	BuildContainmentSoups();
	Stats.SoupBuildMs = (FPlatformTime::Seconds() - SoupStartTime) * 1000.0;
	PopulateSoupPartitionDiagnostics(*this, Stats);

	TArray<int32> NewPlateIds;
	TArray<int32> ContainingTriangles;
	TArray<FVector3d> BarycentricCoords;
	TArray<uint8> GapFlags;
	TArray<uint8> OverlapFlags;
	TArray<TArray<int32>> OverlapPlateIds;
	TArray<float> InterpolatedSubductionDistances;
	TArray<float> InterpolatedSubductionSpeeds;
	const double OwnershipStartTime = FPlatformTime::Seconds();
	QueryOwnership(
		NewPlateIds,
		ContainingTriangles,
		BarycentricCoords,
		GapFlags,
		OverlapFlags,
		Stats.GapCount,
		Stats.OverlapCount,
		&OverlapPlateIds,
		&Stats,
		OwnershipMode);
	Stats.OwnershipQueryMs = (FPlatformTime::Seconds() - OwnershipStartTime) * 1000.0;
	check(Stats.TrueGapCount == Stats.GapCount);
	check(Stats.ExactMultiHitCount == Stats.OverlapCount);
	check(Stats.ExactSingleHitCount + Stats.ExactMultiHitCount + Stats.RecoveryContainmentCount + Stats.TrueGapCount == Samples.Num());
	Stats.NoValidHitCount = Stats.TrueGapCount;
	Stats.MultiContainmentCount = Stats.ExactMultiHitCount;
	Stats.ValidContainmentCount = Stats.ExactSingleHitCount + Stats.RecoveryContainmentCount;
	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Ownership Step=%d] exact_single_hit_count=%d exact_multi_hit_count=%d hysteresis_retained_count=%d hysteresis_reassigned_count=%d recovery_containment_count=%d recovery_continental_count=%d true_gap_count=%d recovery_mean_distance=%.6f recovery_p95_distance=%.6f recovery_max_distance=%.6f"),
		CurrentStep,
		Stats.ExactSingleHitCount,
		Stats.ExactMultiHitCount,
		Stats.HysteresisRetainedCount,
		Stats.HysteresisReassignedCount,
		Stats.RecoveryContainmentCount,
		Stats.RecoveryContinentalCount,
		Stats.TrueGapCount,
		Stats.RecoveryMeanDistance,
		Stats.RecoveryP95Distance,
		Stats.RecoveryMaxDistance);

	const double InterpolationStartTime = FPlatformTime::Seconds();
	InterpolateFromCarried(
		NewPlateIds,
		ContainingTriangles,
		BarycentricCoords,
		InterpolatedSubductionDistances,
		InterpolatedSubductionSpeeds,
		&Stats.MissingLocalCarriedLookupCount);
	Stats.InterpolationMs = (FPlatformTime::Seconds() - InterpolationStartTime) * 1000.0;

	const double GapResolutionStartTime = FPlatformTime::Seconds();
	ResolveGaps(NewPlateIds, GapFlags, InterpolatedSubductionDistances, InterpolatedSubductionSpeeds, &Stats);
	Stats.GapResolutionMs = (FPlatformTime::Seconds() - GapResolutionStartTime) * 1000.0;

	if (TriggerReason == EResampleTriggerReason::RiftFollowup && PendingRiftEvent.bValid)
	{
		// M5a keeps the immediate topology refresh local to the freshly split parent:
		// refresh interpolation/soups/gap handling globally, but preserve existing
		// non-gap ownership elsewhere so plate birth does not cause broad territory churn.
		for (int32 SampleIndex = 0; SampleIndex < NewPlateIds.Num(); ++SampleIndex)
		{
			if (GapFlags.IsValidIndex(SampleIndex) && GapFlags[SampleIndex] == 0)
			{
				NewPlateIds[SampleIndex] = PreviousPlateAssignments[SampleIndex];
			}
		}
	}

	const double OverlapClassificationStartTime = FPlatformTime::Seconds();
	ClassifyOverlapsImpl(*this, OverlapFlags, NewPlateIds, &OverlapPlateIds, Stats);
	Stats.OverlapClassificationMs = (FPlatformTime::Seconds() - OverlapClassificationStartTime) * 1000.0;

	FCollisionEvent CollisionEvent;
	const double CollisionStartTime = FPlatformTime::Seconds();
	const bool bUsePreserveBoundaryContactTrigger = OwnershipMode == EResampleOwnershipMode::PreserveOwnership;
	if (!bUsePreserveBoundaryContactTrigger)
	{
		DetectAndApplyCollision(
			TriggerReason,
			OverlapFlags,
			OverlapPlateIds,
			PreviousPlateAssignments,
			PreviousContinentalWeights,
			PreviousTerraneAssignments,
			NewPlateIds,
			CollisionEvent,
			&Stats);
	}
	else
	{
		Stats.CollisionCount = 0;
		Stats.CollisionDeferredCount = 0;
		Stats.CollisionTerraneId = INDEX_NONE;
		Stats.CollisionTerraneSampleCount = 0;
		Stats.CollisionOverridingPlateId = INDEX_NONE;
		Stats.CollisionSubductingPlateId = INDEX_NONE;
		Stats.CollisionSurgeAffectedCount = 0;
		Stats.CollisionSurgeRadiusRad = 0.0;
		Stats.CollisionSurgeMeanElevationDelta = 0.0;
	}

	const double RepartitionStartTime = FPlatformTime::Seconds();
	RepartitionMembership(NewPlateIds, &InterpolatedSubductionDistances, &InterpolatedSubductionSpeeds);
	Stats.RepartitionMs = (FPlatformTime::Seconds() - RepartitionStartTime) * 1000.0;
	if (OwnershipMode == EResampleOwnershipMode::PreserveOwnership)
	{
		for (int32 SampleIndex = 0; SampleIndex < NewPlateIds.Num(); ++SampleIndex)
		{
			Stats.PreserveOwnershipPlateChangedCount +=
				(NewPlateIds[SampleIndex] != PreviousPlateAssignments[SampleIndex]) ? 1 : 0;
		}
	}

	if (!bUsePreserveBoundaryContactTrigger)
	{
		ApplyCollisionElevationSurge(CollisionEvent, &Stats);
	}
	ComputePlateScores();
	if (bUsePreserveBoundaryContactTrigger)
	{
		FPendingBoundaryContactCollisionEvent BoundaryContactCollisionEvent;
		const bool bImmediateBoundaryContactTrigger =
			DetectBoundaryContactCollisionTrigger(&Stats, &BoundaryContactCollisionEvent);
		const bool bPersistenceBoundaryContactTrigger = UpdateBoundaryContactCollisionPersistence(Stats, ResampleInterval);
		if (bImmediateBoundaryContactTrigger || bPersistenceBoundaryContactTrigger)
		{
			if (BoundaryContactCollisionEvent.bValid)
			{
				BoundaryContactCollisionEvent.PeakZoneSize = Stats.BoundaryContactLargestZoneSize;
				BoundaryContactCollisionEvent.ConsecutiveResamples =
					FMath::Max(Stats.BoundaryContactPersistenceCount, 1);
				PendingBoundaryContactCollisionEvent = BoundaryContactCollisionEvent;
			}
			bPendingFullResolutionResample = true;
			if (bPersistenceBoundaryContactTrigger)
			{
				bPendingBoundaryContactPersistenceReset = true;
			}
		}
	}
	Stats.CollisionMs = (FPlatformTime::Seconds() - CollisionStartTime) * 1000.0;
	if (!bUsePreserveBoundaryContactTrigger &&
		OwnershipMode != EResampleOwnershipMode::FullResolution &&
		CollisionEvent.bDetected)
	{
		bPendingFullResolutionResample = true;
	}

	ComputeSubductionDistanceField(&Stats);
	ComputeSlabPullCorrections(&Stats);

	FTerraneDetectionDiagnostics TerraneDetectionDiagnostics;
	const double TerraneDetectionStartTime = FPlatformTime::Seconds();
	DetectTerranesImpl(*this, PreviousTerranes, PreviousTerraneAssignments, &TerraneDetectionDiagnostics);
	SyncCarriedTerraneIdsFromCanonical(*this);
	Stats.TerraneDetectionMs = (FPlatformTime::Seconds() - TerraneDetectionStartTime) * 1000.0;
	Stats.TerraneCount = Terranes.Num();
	Stats.NewTerraneCount = TerraneDetectionDiagnostics.NewTerraneCount;
	Stats.MergedTerraneCount = TerraneDetectionDiagnostics.MergedTerraneCount;
	if (TriggerReason == EResampleTriggerReason::RiftFollowup && PendingRiftEvent.bValid)
	{
		const FRiftTerraneMetrics RiftTerraneMetrics = ComputeRiftTerraneMetrics(*this, PendingRiftEvent);
		Stats.RiftTerraneCountBefore = RiftTerraneMetrics.TerraneCountBefore;
		Stats.RiftTerraneCountAfter = RiftTerraneMetrics.TerraneCountAfter;
		Stats.RiftTerraneFragmentsOnChildA = RiftTerraneMetrics.TerraneFragmentsOnChildA;
		Stats.RiftTerraneFragmentsOnChildB = RiftTerraneMetrics.TerraneFragmentsOnChildB;
		Stats.RiftTerraneSplitCount = RiftTerraneMetrics.TerraneSplitCount;
		Stats.RiftTerranePreservedCount = RiftTerraneMetrics.TerranePreservedCount;
		Stats.RiftChildTerraneFragmentCounts = RiftTerraneMetrics.ChildTerraneFragmentCounts;
		Stats.RiftPreTerraneIds = RiftTerraneMetrics.PreRiftTerraneIds;
		Stats.RiftPreTerraneTouchedChildCounts = RiftTerraneMetrics.PreRiftTerraneTouchedChildCounts;
		const FRiftBoundaryActivitySummary RiftBoundarySummary =
			ComputeChildBoundaryActivitySummary(*this, PendingRiftEvent.ChildPlateIds);
		Stats.RiftTotalChildBoundaryContactEdges = RiftBoundarySummary.ContactEdgeCount;
		Stats.RiftDivergentChildBoundaryEdgeCount = RiftBoundarySummary.DivergentEdgeCount;

		const FString ChildPlateIdsString = JoinIntArrayForLog(Stats.RiftChildPlateIds);
		const FString ChildSampleCountsString = JoinIntArrayForLog(Stats.RiftChildSampleCounts);
		const FString ChildFragmentCountsString = JoinIntArrayForLog(Stats.RiftChildTerraneFragmentCounts);
		const FString PreRiftTerraneIdsString = JoinIntArrayForLog(Stats.RiftPreTerraneIds);
		const FString PreRiftTouchedChildCountsString = JoinIntArrayForLog(Stats.RiftPreTerraneTouchedChildCounts);

		UE_LOG(
			LogTemp,
			Log,
			TEXT("[Rift Step=%d] mode=%s parent=%d child_count=%d child_ids=(%s) parent_samples=%d parent_continental_samples=%d parent_continental_fraction=%.4f trigger_probability=%.6f event_seed=%d child_samples=(%s) min_child_samples=%d max_child_samples=%d mean_child_samples=%.2f child_boundary_contact_edges=%d child_boundary_divergent_edges=%d terranes_before=%d terranes_after=%d terrane_split_count=%d terrane_preserved_count=%d child_fragments=(%s) pre_terrane_ids=(%s) pre_terrane_touched_child_counts=(%s)"),
			CurrentStep,
			Stats.bRiftWasAutomatic ? TEXT("automatic") : TEXT("manual"),
			Stats.RiftParentPlateId,
			Stats.RiftChildCount,
			*ChildPlateIdsString,
			Stats.RiftParentSampleCount,
			Stats.RiftParentContinentalSampleCount,
			Stats.RiftParentContinentalFraction,
			Stats.RiftTriggerProbability,
			Stats.RiftEventSeed,
			*ChildSampleCountsString,
			Stats.RiftMinChildSampleCount,
			Stats.RiftMaxChildSampleCount,
			Stats.RiftMeanChildSampleCount,
			Stats.RiftTotalChildBoundaryContactEdges,
			Stats.RiftDivergentChildBoundaryEdgeCount,
			Stats.RiftTerraneCountBefore,
			Stats.RiftTerraneCountAfter,
			Stats.RiftTerraneSplitCount,
			Stats.RiftTerranePreservedCount,
			*ChildFragmentCountsString,
			*PreRiftTerraneIdsString,
			*PreRiftTouchedChildCountsString);
	}

	for (FPlate& Plate : Plates)
	{
		Plate.CumulativeRotation = FQuat4d::Identity;
	}

	Stats.TotalMs = (FPlatformTime::Seconds() - TotalStartTime) * 1000.0;
	LastResamplingStats = Stats;
	LastResampleTriggerReason = TriggerReason;
	LastResampleOwnershipMode = OwnershipMode;
	ResamplingSteps.Add(CurrentStep);

	if (TriggerReason == EResampleTriggerReason::CollisionFollowup && bPendingBoundaryContactPersistenceReset)
	{
		ResetBoundaryContactCollisionPersistence();
	}
	else if (TriggerReason == EResampleTriggerReason::CollisionFollowup)
	{
		PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	}
	if (TriggerReason == EResampleTriggerReason::RiftFollowup)
	{
		PendingRiftEvent = FPendingRiftEvent{};
	}

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Resample Step=%d] trigger_reason=%s ownership_mode=%s R=%d spacing_km=%.3f max_displacement_km_per_step=%.3f interior_soup_tris=%d boundary_soup_tris=%d total_soup_tris=%d dropped_mixed_triangle_count=%d duplicated_soup_triangle_count=%d foreign_local_vertex_count=%d exact_single_hit_count=%d exact_multi_hit_count=%d hysteresis_retained_count=%d hysteresis_reassigned_count=%d preserve_same_plate_hit_count=%d preserve_same_plate_recovery_count=%d preserve_fallback_query_count=%d preserve_plate_changed_count=%d recovery_containment_count=%d recovery_continental_count=%d true_gap_count=%d recovery_mean_distance=%.6f recovery_p95_distance=%.6f recovery_max_distance=%.6f missing_local_carried_lookup_count=%d gap_count=%d divergent_gap_count=%d non_divergent_gap_count=%d divergent_continental_gap_count=%d non_divergent_continental_gap_count=%d non_divergent_gap_triangle_projection_count=%d non_divergent_gap_nearest_copy_count=%d overlap_count=%d valid_containment_count=%d multi_containment_count=%d no_valid_hit_count=%d oo_overlap_count=%d oc_overlap_count=%d cc_overlap_count=%d nonconvergent_overlap_count=%d rift_count=%d rift_parent_plate_id=%d rift_child_plate_a=%d rift_child_plate_b=%d rift_parent_sample_count=%d rift_child_sample_count_a=%d rift_child_sample_count_b=%d boundary_contact_collision_candidate_count=%d boundary_contact_collision_pair_count=%d boundary_contact_largest_zone_size=%d boundary_contact_trigger_plate_a=%d boundary_contact_trigger_plate_b=%d boundary_contact_collision_triggered=%d boundary_contact_persistence_pair_a=%d boundary_contact_persistence_pair_b=%d boundary_contact_persistence_count=%d boundary_contact_persistence_triggered=%d subduction_front_edge_count=%d subduction_seed_sample_count=%d subduction_influenced_count=%d subduction_mean_distance_km=%.3f subduction_max_distance_km=%.3f slab_pull_plate_count=%d slab_pull_total_front_samples=%d slab_pull_max_axis_change_rad=%.6f collision_count=%d collision_deferred_count=%d collision_terrane_id=%d collision_terrane_samples=%d collision_over_plate=%d collision_sub_plate=%d collision_surge_affected=%d collision_surge_radius_rad=%.6f collision_surge_mean_elev_delta=%.6f used_cached_boundary_contact_collision=%d cached_boundary_contact_seed_count=%d cached_boundary_contact_terrane_seed_count=%d cached_boundary_contact_terrane_recovered_count=%d cached_boundary_contact_plate_a=%d cached_boundary_contact_plate_b=%d terrane_count=%d new_terrane_count=%d merged_terrane_count=%d rift_ms=%.3f soup_ms=%.3f ownership_ms=%.3f interpolation_ms=%.3f gap_resolution_ms=%.3f overlap_classification_ms=%.3f repartition_ms=%.3f collision_ms=%.3f dijkstra_ms=%.3f slab_pull_ms=%.3f terrane_detection_ms=%.3f total_ms=%.3f"),
		CurrentStep,
		GetResampleTriggerReasonName(TriggerReason),
		GetResampleOwnershipModeName(OwnershipMode),
		ResampleInterval,
		AverageSampleSpacingKm,
		MaxDisplacementKmPerStep,
		Stats.InteriorSoupTriangleCount,
		Stats.BoundarySoupTriangleCount,
		Stats.TotalSoupTriangleCount,
		Stats.DroppedMixedTriangleCount,
		Stats.DuplicatedSoupTriangleCount,
		Stats.ForeignLocalVertexCount,
		Stats.ExactSingleHitCount,
		Stats.ExactMultiHitCount,
		Stats.HysteresisRetainedCount,
		Stats.HysteresisReassignedCount,
		Stats.PreserveOwnershipSamePlateHitCount,
		Stats.PreserveOwnershipSamePlateRecoveryCount,
		Stats.PreserveOwnershipFallbackQueryCount,
		Stats.PreserveOwnershipPlateChangedCount,
		Stats.RecoveryContainmentCount,
		Stats.RecoveryContinentalCount,
		Stats.TrueGapCount,
		Stats.RecoveryMeanDistance,
		Stats.RecoveryP95Distance,
		Stats.RecoveryMaxDistance,
		Stats.MissingLocalCarriedLookupCount,
		Stats.GapCount,
		Stats.DivergentGapCount,
		Stats.NonDivergentGapCount,
		Stats.DivergentContinentalGapCount,
		Stats.NonDivergentContinentalGapCount,
		Stats.NonDivergentGapTriangleProjectionCount,
		Stats.NonDivergentGapNearestCopyCount,
		Stats.OverlapCount,
		Stats.ValidContainmentCount,
		Stats.MultiContainmentCount,
		Stats.NoValidHitCount,
		Stats.OceanicOceanicOverlapCount,
		Stats.OceanicContinentalOverlapCount,
		Stats.ContinentalContinentalOverlapCount,
		Stats.NonConvergentOverlapCount,
		Stats.RiftCount,
		Stats.RiftParentPlateId,
		Stats.RiftChildPlateA,
		Stats.RiftChildPlateB,
		Stats.RiftParentSampleCount,
		Stats.RiftChildSampleCountA,
		Stats.RiftChildSampleCountB,
		Stats.BoundaryContactCollisionCandidateCount,
		Stats.BoundaryContactCollisionPairCount,
		Stats.BoundaryContactLargestZoneSize,
		Stats.BoundaryContactTriggerPlateA,
		Stats.BoundaryContactTriggerPlateB,
		Stats.bBoundaryContactCollisionTriggered ? 1 : 0,
		Stats.BoundaryContactPersistencePairA,
		Stats.BoundaryContactPersistencePairB,
		Stats.BoundaryContactPersistenceCount,
		Stats.bBoundaryContactPersistenceTriggered ? 1 : 0,
		Stats.SubductionFrontEdgeCount,
		Stats.SubductionSeedSampleCount,
		Stats.SubductionInfluencedCount,
		Stats.SubductionMeanDistanceKm,
		Stats.SubductionMaxDistanceKm,
		Stats.SlabPullPlateCount,
		Stats.SlabPullTotalFrontSamples,
		Stats.SlabPullMaxAxisChangeRad,
		Stats.CollisionCount,
		Stats.CollisionDeferredCount,
		Stats.CollisionTerraneId,
		Stats.CollisionTerraneSampleCount,
		Stats.CollisionOverridingPlateId,
		Stats.CollisionSubductingPlateId,
		Stats.CollisionSurgeAffectedCount,
		Stats.CollisionSurgeRadiusRad,
		Stats.CollisionSurgeMeanElevationDelta,
		Stats.bUsedCachedBoundaryContactCollision ? 1 : 0,
		Stats.CachedBoundaryContactSeedCount,
		Stats.CachedBoundaryContactTerraneSeedCount,
		Stats.CachedBoundaryContactTerraneRecoveredCount,
		Stats.CachedBoundaryContactPlateA,
		Stats.CachedBoundaryContactPlateB,
		Stats.TerraneCount,
		Stats.NewTerraneCount,
		Stats.MergedTerraneCount,
		Stats.RiftMs,
		Stats.SoupBuildMs,
		Stats.OwnershipQueryMs,
		Stats.InterpolationMs,
		Stats.GapResolutionMs,
		Stats.OverlapClassificationMs,
		Stats.RepartitionMs,
		Stats.CollisionMs,
		Stats.SubductionDistanceFieldMs,
		Stats.SlabPullMs,
		Stats.TerraneDetectionMs,
		Stats.TotalMs);
}

int32 FTectonicPlanet::ComputeResampleInterval() const
{
	if (Samples.IsEmpty())
	{
		return ResampleIntervalMin;
	}

	double MaxAngularSpeed = 0.0;
	for (const FPlate& Plate : Plates)
	{
		MaxAngularSpeed = FMath::Max(MaxAngularSpeed, Plate.AngularSpeed);
	}

	const double AverageSampleSpacingKm =
		PlanetRadiusKm * FMath::Sqrt((4.0 * PI) / static_cast<double>(Samples.Num()));
	const double MaxDisplacementKmPerStep = FMath::Max(MaxAngularSpeed * PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);
	const int32 RawInterval = FMath::RoundToInt((AverageSampleSpacingKm / MaxDisplacementKmPerStep) * ResampleTriggerK);
	return FMath::Clamp(RawInterval, ResampleIntervalMin, ResampleIntervalMax);
}
