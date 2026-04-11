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
	constexpr double SubductionBaseUpliftKmPerMy = 0.6;
	constexpr double MaxPlateSpeedKmPerMy = 100.0;
	constexpr double ConvergentThresholdRatio = -0.3;
	constexpr double SlabPullEpsilon = 0.1;
	constexpr double MaxAxisChangePerStepRad = 0.05;
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
	constexpr int32 MinGeometricCollisionOverlapContinentalSamples = 3;
	constexpr int32 MinGeometricCollisionOpposingContinentalSupport = 3;
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
	constexpr double InitialPlateSpeedMinKmPerStep = 10.0;
	constexpr double InitialPlateSpeedMaxKmPerStep = 40.0;
	constexpr double ResampleTriggerK = 10.0;
	constexpr int32 ResampleIntervalMin = 10;
	constexpr int32 ResampleIntervalMax = 60;
	constexpr double BoundingCapMargin = 1.0e-6;
	constexpr double TriangleEpsilon = 1.0e-12;
	constexpr double OverlapScoreEpsilon = 0.1;
	constexpr double PreserveBoundaryStrongContinentalThreshold = 0.9;
	constexpr double PreserveBoundaryRecoveredContinentalThreshold = 0.5;
	constexpr double DirectionDegeneracyThreshold = 1.0e-8;
	constexpr double GapSeparationDirectionEpsilon = 1.0e-10;
	constexpr double DivergenceEpsilon = 1.0e-10;

	double ComputeSubductionElevationTransfer(const double ElevationKm)
	{
		const double NormalizedElevation = FMath::Clamp(
			(ElevationKm - TrenchElevationKm) / (ElevationCeilingKm - TrenchElevationKm),
			0.0,
			1.0);
		return NormalizedElevation * NormalizedElevation;
	}

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

	struct FGeometricCollisionOverlapHit
	{
		int32 OwningPlateId = INDEX_NONE;
		int32 ContainingPlateId = INDEX_NONE;
		int32 SampleIndex = INDEX_NONE;
		double ConvergenceMagnitudeKmPerMy = 0.0;
		int32 MinSampleIndex = INDEX_NONE;
	};

	struct FGeometricCollisionPairDiagnostics
	{
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		int32 TotalOverlapSampleCount = 0;
		int32 SubductingOverlapSampleCount = 0;
		int32 OpposingContinentalSupportCount = 0;
		int32 TerraneEstimate = 0;
		int32 MinSampleIndex = INDEX_NONE;
		double TotalConvergenceMagnitudeKmPerMy = 0.0;
		double TotalConvergenceMagnitudeAInsideBKmPerMy = 0.0;
		double TotalConvergenceMagnitudeBInsideAKmPerMy = 0.0;
		double MaxConvergenceMagnitudeKmPerMy = 0.0;
		double MeanOverlapDepthKm = 0.0;
		double MaxOverlapDepthKm = 0.0;
		bool bPassedMassFilter = false;
		TArray<int32> SamplesFromAInsideB;
		TArray<int32> SamplesFromBInsideA;
	};

	double GetGeometricCollisionPairMeanConvergenceMagnitudeKmPerMy(
		const FGeometricCollisionPairDiagnostics& PairDiagnostics)
	{
		return PairDiagnostics.TotalOverlapSampleCount > 0
			? PairDiagnostics.TotalConvergenceMagnitudeKmPerMy /
				static_cast<double>(PairDiagnostics.TotalOverlapSampleCount)
			: 0.0;
	}

	double GetGeometricCollisionDirectionalMeanConvergenceMagnitudeKmPerMy(
		const FGeometricCollisionPairDiagnostics& PairDiagnostics,
		const int32 DonorPlateId)
	{
		if (DonorPlateId == PairDiagnostics.PlateA)
		{
			return PairDiagnostics.SamplesFromAInsideB.Num() > 0
				? PairDiagnostics.TotalConvergenceMagnitudeAInsideBKmPerMy /
					static_cast<double>(PairDiagnostics.SamplesFromAInsideB.Num())
				: 0.0;
		}
		if (DonorPlateId == PairDiagnostics.PlateB)
		{
			return PairDiagnostics.SamplesFromBInsideA.Num() > 0
				? PairDiagnostics.TotalConvergenceMagnitudeBInsideAKmPerMy /
					static_cast<double>(PairDiagnostics.SamplesFromBInsideA.Num())
				: 0.0;
		}
		return 0.0;
	}

	enum class EGeometricCollisionDonorSelectionRule : uint8
	{
		None = 0,
		DirectionalSingleBucket,
		DirectionalCount,
		DirectionalConvergence,
		FallbackLowerPlateScore,
		FallbackDirectionalCount,
		FallbackConvergence,
		FallbackStablePairTieBreak,
	};

	enum class EGeometricDirectionalBucketPopulation : uint8
	{
		Neither = 0,
		OnlyOverriding,
		OnlySubducting,
		Both,
	};

	EGeometricDirectionalBucketPopulation ClassifyGeometricDirectionalBucketPopulation(
		const int32 SubductingDirectionalSeedCount,
		const int32 OverridingDirectionalSeedCount)
	{
		const bool bHasSubductingDirectionalSeeds = SubductingDirectionalSeedCount > 0;
		const bool bHasOverridingDirectionalSeeds = OverridingDirectionalSeedCount > 0;
		if (bHasSubductingDirectionalSeeds && bHasOverridingDirectionalSeeds)
		{
			return EGeometricDirectionalBucketPopulation::Both;
		}
		if (bHasOverridingDirectionalSeeds)
		{
			return EGeometricDirectionalBucketPopulation::OnlyOverriding;
		}
		if (bHasSubductingDirectionalSeeds)
		{
			return EGeometricDirectionalBucketPopulation::OnlySubducting;
		}
		return EGeometricDirectionalBucketPopulation::Neither;
	}

	const TCHAR* GetGeometricDirectionalBucketPopulationName(
		const EGeometricDirectionalBucketPopulation Population)
	{
		switch (Population)
		{
		case EGeometricDirectionalBucketPopulation::OnlyOverriding:
			return TEXT("only_overriding");
		case EGeometricDirectionalBucketPopulation::OnlySubducting:
			return TEXT("only_subducting");
		case EGeometricDirectionalBucketPopulation::Both:
			return TEXT("both");
		case EGeometricDirectionalBucketPopulation::Neither:
		default:
			return TEXT("neither");
		}
	}

	const TCHAR* GetGeometricCollisionDonorSelectionRuleName(
		const EGeometricCollisionDonorSelectionRule Rule)
	{
		switch (Rule)
		{
		case EGeometricCollisionDonorSelectionRule::DirectionalSingleBucket:
			return TEXT("directional_single_bucket");
		case EGeometricCollisionDonorSelectionRule::DirectionalCount:
			return TEXT("directional_count");
		case EGeometricCollisionDonorSelectionRule::DirectionalConvergence:
			return TEXT("directional_convergence");
		case EGeometricCollisionDonorSelectionRule::FallbackLowerPlateScore:
			return TEXT("fallback_lower_plate_score");
		case EGeometricCollisionDonorSelectionRule::FallbackDirectionalCount:
			return TEXT("fallback_directional_count");
		case EGeometricCollisionDonorSelectionRule::FallbackConvergence:
			return TEXT("fallback_convergence");
		case EGeometricCollisionDonorSelectionRule::FallbackStablePairTieBreak:
			return TEXT("fallback_stable_pair_tie_break");
		case EGeometricCollisionDonorSelectionRule::None:
		default:
			return TEXT("none");
		}
	}

	bool IsDirectionalGeometricCollisionDonorSelectionRule(const EGeometricCollisionDonorSelectionRule Rule)
	{
		return
			Rule == EGeometricCollisionDonorSelectionRule::DirectionalSingleBucket ||
			Rule == EGeometricCollisionDonorSelectionRule::DirectionalCount ||
			Rule == EGeometricCollisionDonorSelectionRule::DirectionalConvergence;
	}

	double ComputeContainmentScore(const FVector3d& Barycentric);
	FVector3d ProjectOntoTangent(const FVector3d& Vector, const FVector3d& Normal);
	FVector3d ComputePlateSurfaceVelocity(const FPlate& Plate, const FVector3d& QueryPoint, const double PlanetRadius);
	bool FindContainingTriangleInBVH(
		const FPlateTriangleSoupBVH& BVH,
		const FPlateTriangleSoupAdapter& Adapter,
		const FVector3d& QueryPoint,
		int32& OutLocalTriangleId,
		FVector3d& OutA,
		FVector3d& OutB,
		FVector3d& OutC);
		bool FindNearestMemberSample(
			const FTectonicPlanet& Planet,
			const int32 PlateId,
			const FVector3d& QueryPoint,
			int32& OutCanonicalSampleIndex,
			double& OutGeodesicDistanceRadians);
		struct FRecoveredContainmentHit;
		bool TryFindNearestTriangleRecoveryHit(
			const FTectonicPlanet& Planet,
			const int32 PlateId,
			const FVector3d& QueryPoint,
			FRecoveredContainmentHit& OutHit);

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

	void PruneEmptyPlates(FTectonicPlanet& Planet)
	{
		const int32 RemovedPlateCount = Planet.Plates.RemoveAll([](const FPlate& Plate)
		{
			return Plate.MemberSamples.IsEmpty();
		});
		if (RemovedPlateCount <= 0)
		{
			return;
		}

		for (auto PersistenceIt = Planet.BoundaryContactPersistenceByPair.CreateIterator(); PersistenceIt; ++PersistenceIt)
		{
			const FBoundaryContactPersistence& Persistence = PersistenceIt.Value();
			if (FindPlateById(Planet, Persistence.PlateA) == nullptr ||
				FindPlateById(Planet, Persistence.PlateB) == nullptr)
			{
				PersistenceIt.RemoveCurrent();
			}
		}
		for (auto RecurrenceIt = Planet.GeometricCollisionPairRecurrenceByKey.CreateIterator(); RecurrenceIt; ++RecurrenceIt)
		{
			const uint64 PairKey = RecurrenceIt.Key();
			const int32 PlateA = static_cast<int32>(PairKey >> 32);
			const int32 PlateB = static_cast<int32>(PairKey & 0xffffffffu);
			if (FindPlateById(Planet, PlateA) == nullptr || FindPlateById(Planet, PlateB) == nullptr)
			{
				RecurrenceIt.RemoveCurrent();
			}
		}

		if (Planet.PendingBoundaryContactCollisionEvent.bValid &&
			(FindPlateById(Planet, Planet.PendingBoundaryContactCollisionEvent.PlateA) == nullptr ||
				FindPlateById(Planet, Planet.PendingBoundaryContactCollisionEvent.PlateB) == nullptr))
		{
			Planet.PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
		}
		if (Planet.PendingGeometricCollisionEvent.bValid &&
			(FindPlateById(Planet, Planet.PendingGeometricCollisionEvent.PlateA) == nullptr ||
				FindPlateById(Planet, Planet.PendingGeometricCollisionEvent.PlateB) == nullptr))
		{
			Planet.PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};
		}
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

		const TCHAR* GetResamplingPolicyName(const EResamplingPolicy Policy)
		{
			switch (Policy)
			{
			case EResamplingPolicy::PeriodicGlobalAuthoritativeSpike:
				return TEXT("PeriodicGlobalAuthoritativeSpike");
			case EResamplingPolicy::PreserveOwnershipPeriodic:
				return TEXT("PreserveOwnershipPeriodic");
			case EResamplingPolicy::HybridStablePeriodic:
				return TEXT("HybridStablePeriodic");
			case EResamplingPolicy::EventDrivenOnly:
				return TEXT("EventDrivenOnly");
			case EResamplingPolicy::PeriodicFull:
			default:
				return TEXT("PeriodicFull");
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

	const TCHAR* GetRiftRecipientTypeName(const int32 TypeCode)
	{
		switch (TypeCode)
		{
		case 1:
			return TEXT("sibling_child");
		case 2:
			return TEXT("other_neighbor");
		default:
			return TEXT("other_affected_plate");
		}
	}

	FString JoinRiftRecipientTypeCodesForLog(const TArray<int32>& TypeCodes)
	{
		TArray<FString> Labels;
		Labels.Reserve(TypeCodes.Num());
		for (const int32 TypeCode : TypeCodes)
		{
			Labels.Add(GetRiftRecipientTypeName(TypeCode));
		}
		return FString::Join(Labels, TEXT(","));
	}

	struct FPlateComponentCountSummary
	{
		TMap<int32, int32> ComponentsByPlate;
		int32 GlobalMaxComponents = 0;
	};

	struct FCollisionReceiverContiguityDiagnostics
	{
		int32 ReceiverComponentsBefore = 0;
		int32 ReceiverComponentsAfter = 0;
		bool bContiguousWithExistingTerritory = false;
		int32 DisconnectedFragmentCount = 0;
		int32 LargestNewDisconnectedFragmentSize = 0;
	};

	struct FSinglePlateComponentDiagnostics
	{
		int32 ComponentCount = 0;
		int32 LargestComponentSize = 0;
		int32 LargestSecondaryComponentSize = 0;
	};

	struct FDonorFragmentationDiagnostics
	{
		int32 ComponentsBefore = 0;
		int32 ComponentsAfter = 0;
		int32 LargestRemainingComponentSize = 0;
		int32 NewFragmentCount = 0;
		int32 SmallestNewFragmentSize = 0;
		int32 LargestNewFragmentSize = 0;
		double MeanNewFragmentSize = 0.0;
	};

	struct FCollisionTransferConnectivityDiagnostics
	{
		int32 GlobalMaxComponentsBefore = 0;
		int32 GlobalMaxComponentsAfter = 0;
		int32 DonorComponentsBefore = 0;
		int32 DonorComponentsAfter = 0;
		int32 ReceiverComponentsBefore = 0;
		int32 ReceiverComponentsAfter = 0;
		int32 DonorLargestRemainingComponentSize = 0;
		int32 DonorNewFragmentCount = 0;
		int32 DonorSmallestNewFragmentSize = 0;
		int32 DonorLargestNewFragmentSize = 0;
		double DonorMeanNewFragmentSize = 0.0;
		bool bReceiverTransferContiguousWithExistingTerritory = false;
		int32 ReceiverDisconnectedFragmentCount = 0;
		int32 ReceiverLargestNewDisconnectedFragmentSize = 0;
	};

	struct FCollisionTransferProtectionDiagnostics
	{
		FCollisionTransferConnectivityDiagnostics Proposed;
		FCollisionTransferConnectivityDiagnostics Applied;
		int32 ProposedTerraneSampleCount = 0;
		int32 AppliedTerraneSampleCount = 0;
		double AppliedTrimRatio = 0.0;
		bool bTrimmedByDonorProtection = false;
		bool bRejectedByDonorProtection = false;
		bool bTrimmedByDonorComponentCap = false;
		bool bTrimmedByDonorFragmentFloor = false;
		bool bRejectedByDonorComponentCap = false;
		bool bRejectedByDonorFragmentFloor = false;
		bool bTrimmedByReceiverProtection = false;
		bool bRejectedByReceiverProtection = false;
	};

	struct FCollisionDonorProtectionResult
	{
		bool bPasses = true;
		bool bFailedComponentCap = false;
		bool bFailedFragmentFloor = false;
	};

	struct FRiftPlateAssignmentComponent
	{
		TArray<int32> SampleIndices;
		TMap<int32, int32> BoundaryCountsByPlate;
	};

	enum class ERiftRecipientTypeCode : int32
	{
		OtherAffectedPlate = 0,
		SiblingChild = 1,
		OtherNeighbor = 2,
	};

	struct FRiftFragmentDestinationSelectionResult
	{
		int32 DestinationPlateId = INDEX_NONE;
		bool bUsedSiblingChild = false;
		bool bUsedZeroAdjacencySiblingFallback = false;
		bool bUsedNonChildFallback = false;
		bool bHadPositiveSiblingAdjacency = false;
		int32 BestSiblingEdgeCount = 0;
		int32 CandidateCountConsidered = 0;
		int32 IncoherenceRejectedCount = 0;
	};

	void CollectPlateAssignmentComponents(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 PlateId,
		TArray<FRiftPlateAssignmentComponent>& OutComponents)
	{
		OutComponents.Reset();
		if (PlateId == INDEX_NONE || PlateAssignments.Num() != Planet.Samples.Num())
		{
			return;
		}

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) ||
				Visited[SeedSampleIndex] != 0 ||
				PlateAssignments[SeedSampleIndex] != PlateId)
			{
				continue;
			}

			FRiftPlateAssignmentComponent& Component = OutComponents.Emplace_GetRef();
			TArray<int32, TInlineAllocator<128>> Stack;
			Stack.Add(SeedSampleIndex);
			Visited[SeedSampleIndex] = 1;
			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				Component.SampleIndices.Add(SampleIndex);
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex))
					{
						continue;
					}

					const int32 NeighborPlateId = PlateAssignments[NeighborIndex];
					if (NeighborPlateId == PlateId)
					{
						if (Visited[NeighborIndex] == 0)
						{
							Visited[NeighborIndex] = 1;
							Stack.Add(NeighborIndex);
						}
						continue;
					}

					if (NeighborPlateId != INDEX_NONE)
					{
						++Component.BoundaryCountsByPlate.FindOrAdd(NeighborPlateId);
					}
				}
			}
		}
	}

	int32 CountPlateAssignmentComponents(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 PlateId)
	{
		TArray<FRiftPlateAssignmentComponent> Components;
		CollectPlateAssignmentComponents(Planet, PlateAssignments, PlateId, Components);
		return Components.Num();
	}

	int32 FindRiftChildPlateIndex(
		const FPendingRiftEvent& RiftEvent,
		const int32 ChildPlateId)
	{
		for (int32 ChildIndex = 0; ChildIndex < RiftEvent.ChildPlateIds.Num(); ++ChildIndex)
		{
			if (RiftEvent.ChildPlateIds[ChildIndex] == ChildPlateId)
			{
				return ChildIndex;
			}
		}

		return INDEX_NONE;
	}

	int32 CountAssignedSamplesForPlate(
		const TArray<int32>& PlateAssignments,
		const int32 PlateId)
	{
		int32 Count = 0;
		for (const int32 AssignedPlateId : PlateAssignments)
		{
			Count += AssignedPlateId == PlateId ? 1 : 0;
		}
		return Count;
	}

	int32 SelectDeterministicRiftFragmentDestination(
		const FPendingRiftEvent& RiftEvent,
		const int32 SourceChildPlateId,
		const TMap<int32, int32>& BoundaryCountsByPlate,
		bool& bOutUsedSiblingChild)
	{
		bOutUsedSiblingChild = false;
		int32 BestSiblingPlateId = INDEX_NONE;
		int32 BestSiblingEdgeCount = -1;
		for (const int32 CandidateChildPlateId : RiftEvent.ChildPlateIds)
		{
			if (CandidateChildPlateId == INDEX_NONE || CandidateChildPlateId == SourceChildPlateId)
			{
				continue;
			}

			const int32 EdgeCount = BoundaryCountsByPlate.FindRef(CandidateChildPlateId);
			if (EdgeCount <= 0)
			{
				continue;
			}

			if (EdgeCount > BestSiblingEdgeCount ||
				(EdgeCount == BestSiblingEdgeCount &&
					(BestSiblingPlateId == INDEX_NONE || CandidateChildPlateId < BestSiblingPlateId)))
			{
				BestSiblingPlateId = CandidateChildPlateId;
				BestSiblingEdgeCount = EdgeCount;
			}
		}

		if (BestSiblingPlateId != INDEX_NONE)
		{
			bOutUsedSiblingChild = true;
			return BestSiblingPlateId;
		}

		int32 BestNeighborPlateId = INDEX_NONE;
		int32 BestNeighborEdgeCount = -1;
		for (const TPair<int32, int32>& Pair : BoundaryCountsByPlate)
		{
			const int32 CandidatePlateId = Pair.Key;
			const int32 EdgeCount = Pair.Value;
			if (CandidatePlateId == INDEX_NONE ||
				CandidatePlateId == SourceChildPlateId ||
				EdgeCount <= 0)
			{
				continue;
			}

			if (EdgeCount > BestNeighborEdgeCount ||
				(EdgeCount == BestNeighborEdgeCount &&
					(BestNeighborPlateId == INDEX_NONE || CandidatePlateId < BestNeighborPlateId)))
			{
				BestNeighborPlateId = CandidatePlateId;
				BestNeighborEdgeCount = EdgeCount;
			}
		}

		return BestNeighborPlateId;
	}

	bool DoesRiftFragmentAttachToRecipientPrimaryBody(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 RecipientPlateId,
		const FRiftPlateAssignmentComponent& Fragment)
	{
		if (RecipientPlateId == INDEX_NONE)
		{
			return false;
		}

		TArray<FRiftPlateAssignmentComponent> RecipientComponents;
		CollectPlateAssignmentComponents(Planet, PlateAssignments, RecipientPlateId, RecipientComponents);
		if (RecipientComponents.IsEmpty())
		{
			return false;
		}

		int32 PrimaryComponentIndex = INDEX_NONE;
		const int32 ChildIndex = FindRiftChildPlateIndex(Planet.PendingRiftEvent, RecipientPlateId);
		if (ChildIndex != INDEX_NONE)
		{
			const int32 AnchorSampleIndex =
				Planet.PendingRiftEvent.ChildAnchorSampleIndices.IsValidIndex(ChildIndex)
					? Planet.PendingRiftEvent.ChildAnchorSampleIndices[ChildIndex]
					: INDEX_NONE;
			if (Planet.Samples.IsValidIndex(AnchorSampleIndex))
			{
				for (int32 ComponentIndex = 0; ComponentIndex < RecipientComponents.Num(); ++ComponentIndex)
				{
					if (RecipientComponents[ComponentIndex].SampleIndices.Contains(AnchorSampleIndex))
					{
						PrimaryComponentIndex = ComponentIndex;
						break;
					}
				}
			}
		}

		if (PrimaryComponentIndex == INDEX_NONE)
		{
			int32 LargestComponentSize = -1;
			int32 LowestSampleIndex = TNumericLimits<int32>::Max();
			for (int32 ComponentIndex = 0; ComponentIndex < RecipientComponents.Num(); ++ComponentIndex)
			{
				const FRiftPlateAssignmentComponent& Component = RecipientComponents[ComponentIndex];
				const int32 ComponentSize = Component.SampleIndices.Num();
				int32 ComponentLowestSampleIndex = TNumericLimits<int32>::Max();
				for (const int32 SampleIndex : Component.SampleIndices)
				{
					ComponentLowestSampleIndex = FMath::Min(ComponentLowestSampleIndex, SampleIndex);
				}
				if (ComponentSize > LargestComponentSize ||
					(ComponentSize == LargestComponentSize &&
						ComponentLowestSampleIndex < LowestSampleIndex))
				{
					LargestComponentSize = ComponentSize;
					LowestSampleIndex = ComponentLowestSampleIndex;
					PrimaryComponentIndex = ComponentIndex;
				}
			}
		}

		if (!RecipientComponents.IsValidIndex(PrimaryComponentIndex))
		{
			return false;
		}

		TSet<int32> PrimaryComponentSamples;
		for (const int32 SampleIndex : RecipientComponents[PrimaryComponentIndex].SampleIndices)
		{
			PrimaryComponentSamples.Add(SampleIndex);
		}

		for (const int32 SampleIndex : Fragment.SampleIndices)
		{
			if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!PlateAssignments.IsValidIndex(NeighborIndex) ||
					PlateAssignments[NeighborIndex] != RecipientPlateId)
				{
					continue;
				}

				if (PrimaryComponentSamples.Contains(NeighborIndex))
				{
					return true;
				}
			}
		}

		return false;
	}

	int32 CountPlateComponentsAfterRiftFragmentAssignment(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const FRiftPlateAssignmentComponent& Fragment,
		const int32 DestinationPlateId)
	{
		TArray<int32> AdjustedAssignments = PlateAssignments;
		for (const int32 SampleIndex : Fragment.SampleIndices)
		{
			if (AdjustedAssignments.IsValidIndex(SampleIndex))
			{
				AdjustedAssignments[SampleIndex] = DestinationPlateId;
			}
		}

		return CountPlateAssignmentComponents(Planet, AdjustedAssignments, DestinationPlateId);
	}

	FRiftFragmentDestinationSelectionResult SelectRecipientAwareRiftFragmentDestination(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 SourceChildPlateId,
		const FRiftPlateAssignmentComponent& Fragment)
	{
		FRiftFragmentDestinationSelectionResult Result;

		TArray<TPair<int32, int32>> AdjacentSiblingCandidates;
		TArray<TPair<int32, int32>> NeighborCandidates;
		for (const TPair<int32, int32>& Pair : Fragment.BoundaryCountsByPlate)
		{
			const int32 CandidatePlateId = Pair.Key;
			const int32 EdgeCount = Pair.Value;
			if (CandidatePlateId == INDEX_NONE ||
				CandidatePlateId == SourceChildPlateId ||
				EdgeCount <= 0)
			{
				continue;
			}

			if (Planet.PendingRiftEvent.ChildPlateIds.Contains(CandidatePlateId))
			{
				AdjacentSiblingCandidates.Add(TPair<int32, int32>(CandidatePlateId, EdgeCount));
			}
			else
			{
				NeighborCandidates.Add(TPair<int32, int32>(CandidatePlateId, EdgeCount));
			}
		}

		const auto CandidateSort = [](const TPair<int32, int32>& A, const TPair<int32, int32>& B)
		{
			return
				A.Value > B.Value ||
				(A.Value == B.Value && A.Key < B.Key);
		};
		AdjacentSiblingCandidates.Sort(CandidateSort);
		NeighborCandidates.Sort(CandidateSort);

		if (!AdjacentSiblingCandidates.IsEmpty())
		{
			Result.DestinationPlateId = AdjacentSiblingCandidates[0].Key;
			Result.bUsedSiblingChild = true;
			Result.bHadPositiveSiblingAdjacency = true;
			Result.BestSiblingEdgeCount = AdjacentSiblingCandidates[0].Value;
			return Result;
		}

		TArray<int32> SiblingFallbackCandidates;
		for (const int32 CandidateChildPlateId : Planet.PendingRiftEvent.ChildPlateIds)
		{
			if (CandidateChildPlateId == INDEX_NONE || CandidateChildPlateId == SourceChildPlateId)
			{
				continue;
			}
			SiblingFallbackCandidates.Add(CandidateChildPlateId);
		}

		if (!SiblingFallbackCandidates.IsEmpty())
		{
			SiblingFallbackCandidates.Sort([&PlateAssignments](const int32 A, const int32 B)
			{
				const int32 SampleCountA = CountAssignedSamplesForPlate(PlateAssignments, A);
				const int32 SampleCountB = CountAssignedSamplesForPlate(PlateAssignments, B);
				return
					SampleCountA > SampleCountB ||
					(SampleCountA == SampleCountB && A < B);
			});
			Result.DestinationPlateId = SiblingFallbackCandidates[0];
			Result.bUsedSiblingChild = true;
			Result.bUsedZeroAdjacencySiblingFallback = true;
			Result.bHadPositiveSiblingAdjacency = false;
			Result.BestSiblingEdgeCount = 0;
			return Result;
		}

		if (!NeighborCandidates.IsEmpty())
		{
			Result.DestinationPlateId = NeighborCandidates[0].Key;
			Result.bUsedSiblingChild = false;
			Result.bUsedNonChildFallback = true;
		}
		return Result;
	}

	void ComputePlateComponentCountSummary(
		const FTectonicPlanet& Planet,
		const TArray<int32>* OverridePlateAssignments,
		FPlateComponentCountSummary& OutSummary)
	{
		OutSummary.ComponentsByPlate.Reset();
		OutSummary.GlobalMaxComponents = 0;
		if (Planet.Samples.IsEmpty())
		{
			return;
		}

		const auto GetPlateId = [&Planet, OverridePlateAssignments](const int32 SampleIndex)
		{
			if (OverridePlateAssignments != nullptr)
			{
				return OverridePlateAssignments->IsValidIndex(SampleIndex)
					? (*OverridePlateAssignments)[SampleIndex]
					: INDEX_NONE;
			}
			return Planet.Samples.IsValidIndex(SampleIndex)
				? Planet.Samples[SampleIndex].PlateId
				: INDEX_NONE;
		};

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) || Visited[SeedSampleIndex] != 0)
			{
				continue;
			}

			const int32 PlateId = GetPlateId(SeedSampleIndex);
			if (PlateId == INDEX_NONE)
			{
				continue;
			}

			TArray<int32, TInlineAllocator<128>> Stack;
			Stack.Add(SeedSampleIndex);
			Visited[SeedSampleIndex] = 1;

			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						Visited[NeighborIndex] != 0 ||
						GetPlateId(NeighborIndex) != PlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			const int32 ComponentCount = ++OutSummary.ComponentsByPlate.FindOrAdd(PlateId);
			OutSummary.GlobalMaxComponents = FMath::Max(OutSummary.GlobalMaxComponents, ComponentCount);
		}
	}

	FCollisionReceiverContiguityDiagnostics ComputeCollisionReceiverContiguityDiagnostics(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PreviousPlateAssignments,
		const TArray<int32>* AfterPlateAssignments,
		const int32 ReceiverPlateId,
		const TArray<int32>& TransferredTerraneSampleIndices,
		const FPlateComponentCountSummary& BeforeSummary,
		const FPlateComponentCountSummary& AfterSummary)
	{
		FCollisionReceiverContiguityDiagnostics Diagnostics;
		Diagnostics.ReceiverComponentsBefore = BeforeSummary.ComponentsByPlate.FindRef(ReceiverPlateId);
		Diagnostics.ReceiverComponentsAfter = AfterSummary.ComponentsByPlate.FindRef(ReceiverPlateId);
		if (ReceiverPlateId == INDEX_NONE || TransferredTerraneSampleIndices.IsEmpty())
		{
			return Diagnostics;
		}

		TArray<uint8> TransferredMask;
		TransferredMask.Init(0, Planet.Samples.Num());
		for (const int32 SampleIndex : TransferredTerraneSampleIndices)
		{
			if (TransferredMask.IsValidIndex(SampleIndex))
			{
				TransferredMask[SampleIndex] = 1;
			}
		}

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) ||
				Visited[SeedSampleIndex] != 0 ||
				((AfterPlateAssignments != nullptr)
					 ? (AfterPlateAssignments->IsValidIndex(SeedSampleIndex)
							? (*AfterPlateAssignments)[SeedSampleIndex]
							: INDEX_NONE)
					 : Planet.Samples[SeedSampleIndex].PlateId) != ReceiverPlateId)
			{
				continue;
			}

			bool bHasTransferredSamples = false;
			bool bHasPreExistingReceiverSamples = false;
			int32 ComponentSize = 0;
			TArray<int32, TInlineAllocator<128>> Stack;
			Stack.Add(SeedSampleIndex);
			Visited[SeedSampleIndex] = 1;

			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				++ComponentSize;
				bHasTransferredSamples |= TransferredMask[SampleIndex] != 0;
				bHasPreExistingReceiverSamples |=
					PreviousPlateAssignments.IsValidIndex(SampleIndex) &&
					PreviousPlateAssignments[SampleIndex] == ReceiverPlateId;

				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						Visited[NeighborIndex] != 0 ||
						((AfterPlateAssignments != nullptr)
							 ? (AfterPlateAssignments->IsValidIndex(NeighborIndex)
									? (*AfterPlateAssignments)[NeighborIndex]
									: INDEX_NONE)
							 : Planet.Samples[NeighborIndex].PlateId) != ReceiverPlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			if (!bHasTransferredSamples)
			{
				continue;
			}

			if (bHasPreExistingReceiverSamples)
			{
				Diagnostics.bContiguousWithExistingTerritory = true;
			}
			else
			{
				++Diagnostics.DisconnectedFragmentCount;
				Diagnostics.LargestNewDisconnectedFragmentSize = FMath::Max(
					Diagnostics.LargestNewDisconnectedFragmentSize,
					ComponentSize);
			}
		}

		return Diagnostics;
	}

	FSinglePlateComponentDiagnostics ComputeSinglePlateComponentDiagnostics(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 PlateId)
	{
		FSinglePlateComponentDiagnostics Diagnostics;
		if (PlateId == INDEX_NONE || PlateAssignments.Num() != Planet.Samples.Num())
		{
			return Diagnostics;
		}

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) ||
				Visited[SeedSampleIndex] != 0 ||
				PlateAssignments[SeedSampleIndex] != PlateId)
			{
				continue;
			}

			int32 ComponentSize = 0;
			TArray<int32, TInlineAllocator<128>> Stack;
			Stack.Add(SeedSampleIndex);
			Visited[SeedSampleIndex] = 1;
			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				++ComponentSize;
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						Visited[NeighborIndex] != 0 ||
						PlateAssignments[NeighborIndex] != PlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			++Diagnostics.ComponentCount;
			if (ComponentSize > Diagnostics.LargestComponentSize)
			{
				Diagnostics.LargestSecondaryComponentSize = Diagnostics.LargestComponentSize;
				Diagnostics.LargestComponentSize = ComponentSize;
			}
			else if (ComponentSize > Diagnostics.LargestSecondaryComponentSize)
			{
				Diagnostics.LargestSecondaryComponentSize = ComponentSize;
			}
		}

		return Diagnostics;
	}

	FDonorFragmentationDiagnostics ComputeDonorFragmentationDiagnostics(
		const FTectonicPlanet& Planet,
		const TArray<int32>& BeforeAssignments,
		const TArray<int32>& AfterAssignments,
		const int32 DonorPlateId)
	{
		FDonorFragmentationDiagnostics Diagnostics;
		if (DonorPlateId == INDEX_NONE ||
			BeforeAssignments.Num() != Planet.Samples.Num() ||
			AfterAssignments.Num() != Planet.Samples.Num())
		{
			return Diagnostics;
		}

		TArray<int32> BeforeComponentIds;
		BeforeComponentIds.Init(INDEX_NONE, Planet.Samples.Num());
		TArray<int32> BeforeComponentSizes;
		TArray<uint8> VisitedBefore;
		VisitedBefore.Init(0, Planet.Samples.Num());
		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) ||
				VisitedBefore[SeedSampleIndex] != 0 ||
				BeforeAssignments[SeedSampleIndex] != DonorPlateId)
			{
				continue;
			}

			const int32 ComponentId = BeforeComponentSizes.Num();
			int32 ComponentSize = 0;
			TArray<int32, TInlineAllocator<128>> Stack;
			Stack.Add(SeedSampleIndex);
			VisitedBefore[SeedSampleIndex] = 1;
			BeforeComponentIds[SeedSampleIndex] = ComponentId;
			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				++ComponentSize;
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						VisitedBefore[NeighborIndex] != 0 ||
						BeforeAssignments[NeighborIndex] != DonorPlateId)
					{
						continue;
					}

					VisitedBefore[NeighborIndex] = 1;
					BeforeComponentIds[NeighborIndex] = ComponentId;
					Stack.Add(NeighborIndex);
				}
			}

			BeforeComponentSizes.Add(ComponentSize);
		}

		Diagnostics.ComponentsBefore = BeforeComponentSizes.Num();
		TArray<TArray<int32>> AfterFragmentSizesByBeforeComponent;
		AfterFragmentSizesByBeforeComponent.SetNum(BeforeComponentSizes.Num());

		TArray<uint8> VisitedAfter;
		VisitedAfter.Init(0, Planet.Samples.Num());
		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) ||
				VisitedAfter[SeedSampleIndex] != 0 ||
				AfterAssignments[SeedSampleIndex] != DonorPlateId)
			{
				continue;
			}

			int32 ComponentSize = 0;
			int32 SourceBeforeComponentId = INDEX_NONE;
			TArray<int32, TInlineAllocator<128>> Stack;
			Stack.Add(SeedSampleIndex);
			VisitedAfter[SeedSampleIndex] = 1;
			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				++ComponentSize;
				if (BeforeComponentIds.IsValidIndex(SampleIndex))
				{
					const int32 BeforeComponentId = BeforeComponentIds[SampleIndex];
					if (BeforeComponentId != INDEX_NONE)
					{
						SourceBeforeComponentId = BeforeComponentId;
					}
				}

				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						VisitedAfter[NeighborIndex] != 0 ||
						AfterAssignments[NeighborIndex] != DonorPlateId)
					{
						continue;
					}

					VisitedAfter[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			++Diagnostics.ComponentsAfter;
			Diagnostics.LargestRemainingComponentSize = FMath::Max(
				Diagnostics.LargestRemainingComponentSize,
				ComponentSize);
			if (AfterFragmentSizesByBeforeComponent.IsValidIndex(SourceBeforeComponentId))
			{
				AfterFragmentSizesByBeforeComponent[SourceBeforeComponentId].Add(ComponentSize);
			}
		}

		int64 TotalNewFragmentSamples = 0;
		for (TArray<int32>& FragmentSizes : AfterFragmentSizesByBeforeComponent)
		{
			if (FragmentSizes.Num() <= 1)
			{
				continue;
			}

			FragmentSizes.Sort(TGreater<int32>());
			for (int32 FragmentIndex = 1; FragmentIndex < FragmentSizes.Num(); ++FragmentIndex)
			{
				const int32 FragmentSize = FragmentSizes[FragmentIndex];
				++Diagnostics.NewFragmentCount;
				TotalNewFragmentSamples += FragmentSize;
				Diagnostics.SmallestNewFragmentSize =
					Diagnostics.SmallestNewFragmentSize == 0
						? FragmentSize
						: FMath::Min(Diagnostics.SmallestNewFragmentSize, FragmentSize);
				Diagnostics.LargestNewFragmentSize = FMath::Max(
					Diagnostics.LargestNewFragmentSize,
					FragmentSize);
			}
		}

		if (Diagnostics.NewFragmentCount > 0)
		{
			Diagnostics.MeanNewFragmentSize =
				static_cast<double>(TotalNewFragmentSamples) /
				static_cast<double>(Diagnostics.NewFragmentCount);
		}

		return Diagnostics;
	}

	FCollisionTransferConnectivityDiagnostics ComputeCollisionTransferConnectivityDiagnostics(
		const FTectonicPlanet& Planet,
		const TArray<int32>& BeforeAssignments,
		const int32 ReceiverPlateId,
		const int32 DonorPlateId,
		const TArray<int32>& TransferredTerraneSampleIndices)
	{
		FCollisionTransferConnectivityDiagnostics Diagnostics;
		if (BeforeAssignments.Num() != Planet.Samples.Num())
		{
			return Diagnostics;
		}

		TArray<int32> AfterAssignments = BeforeAssignments;
		for (const int32 SampleIndex : TransferredTerraneSampleIndices)
		{
			if (AfterAssignments.IsValidIndex(SampleIndex))
			{
				AfterAssignments[SampleIndex] = ReceiverPlateId;
			}
		}

		FPlateComponentCountSummary BeforeSummary;
		FPlateComponentCountSummary AfterSummary;
		ComputePlateComponentCountSummary(Planet, &BeforeAssignments, BeforeSummary);
		ComputePlateComponentCountSummary(Planet, &AfterAssignments, AfterSummary);

		Diagnostics.GlobalMaxComponentsBefore = BeforeSummary.GlobalMaxComponents;
		Diagnostics.GlobalMaxComponentsAfter = AfterSummary.GlobalMaxComponents;
		Diagnostics.DonorComponentsBefore = BeforeSummary.ComponentsByPlate.FindRef(DonorPlateId);
		Diagnostics.DonorComponentsAfter = AfterSummary.ComponentsByPlate.FindRef(DonorPlateId);
		Diagnostics.ReceiverComponentsBefore = BeforeSummary.ComponentsByPlate.FindRef(ReceiverPlateId);
		Diagnostics.ReceiverComponentsAfter = AfterSummary.ComponentsByPlate.FindRef(ReceiverPlateId);

		const FDonorFragmentationDiagnostics DonorDiagnostics = ComputeDonorFragmentationDiagnostics(
			Planet,
			BeforeAssignments,
			AfterAssignments,
			DonorPlateId);
		Diagnostics.DonorLargestRemainingComponentSize =
			DonorDiagnostics.LargestRemainingComponentSize;
		Diagnostics.DonorNewFragmentCount = DonorDiagnostics.NewFragmentCount;
		Diagnostics.DonorSmallestNewFragmentSize = DonorDiagnostics.SmallestNewFragmentSize;
		Diagnostics.DonorLargestNewFragmentSize = DonorDiagnostics.LargestNewFragmentSize;
		Diagnostics.DonorMeanNewFragmentSize = DonorDiagnostics.MeanNewFragmentSize;

		const FCollisionReceiverContiguityDiagnostics ReceiverDiagnostics =
			ComputeCollisionReceiverContiguityDiagnostics(
				Planet,
				BeforeAssignments,
				&AfterAssignments,
				ReceiverPlateId,
				TransferredTerraneSampleIndices,
				BeforeSummary,
				AfterSummary);
		Diagnostics.bReceiverTransferContiguousWithExistingTerritory =
			ReceiverDiagnostics.bContiguousWithExistingTerritory;
		Diagnostics.ReceiverDisconnectedFragmentCount =
			ReceiverDiagnostics.DisconnectedFragmentCount;
		Diagnostics.ReceiverLargestNewDisconnectedFragmentSize =
			ReceiverDiagnostics.LargestNewDisconnectedFragmentSize;
		return Diagnostics;
	}

	FCollisionDonorProtectionResult EvaluateCollisionDonorConnectivityProtection(
		const FTectonicPlanet& Planet,
		const FCollisionTransferConnectivityDiagnostics& Diagnostics)
	{
		FCollisionDonorProtectionResult Result;
		const int32 DonorComponentIncrease = FMath::Max(
			0,
			Diagnostics.DonorComponentsAfter - Diagnostics.DonorComponentsBefore);
		Result.bFailedComponentCap =
			DonorComponentIncrease > Planet.GeometricCollisionDonorMaxComponentIncrease;

		const bool bFragmentFloorEnabled =
			Planet.GeometricCollisionDonorMinNewFragmentSampleCount > 0;
		Result.bFailedFragmentFloor =
			bFragmentFloorEnabled &&
			Diagnostics.DonorNewFragmentCount > 0 &&
			Diagnostics.DonorSmallestNewFragmentSize <
				Planet.GeometricCollisionDonorMinNewFragmentSampleCount;
		Result.bPasses = !Result.bFailedComponentCap && !Result.bFailedFragmentFloor;
		return Result;
	}

	bool PassesCollisionDonorConnectivityProtection(
		const FTectonicPlanet& Planet,
		const FCollisionTransferConnectivityDiagnostics& Diagnostics)
	{
		return EvaluateCollisionDonorConnectivityProtection(Planet, Diagnostics).bPasses;
	}

	bool PassesCollisionReceiverConnectivityProtection(
		const FCollisionTransferConnectivityDiagnostics& Diagnostics)
	{
		return true;
	}

	void BuildCollisionTransferSubsetByDistanceThreshold(
		const TArray<int32>& SourceTerraneSampleIndices,
		const TArray<double>& BestDistancesKm,
		const double MaxDistanceKm,
		TArray<int32>& OutSubset)
	{
		OutSubset.Reset();
		OutSubset.Reserve(SourceTerraneSampleIndices.Num());
		for (const int32 SampleIndex : SourceTerraneSampleIndices)
		{
			if (!BestDistancesKm.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const double DistanceKm = BestDistancesKm[SampleIndex];
			if (!FMath::IsFinite(DistanceKm) || DistanceKm > MaxDistanceKm + UE_DOUBLE_SMALL_NUMBER)
			{
				continue;
			}

			OutSubset.Add(SampleIndex);
		}
	}

	void PopulateCanonicalSampleMetrics(const FTectonicPlanet& Planet, FResamplingStats& OutStats)
	{
		OutStats.PlateCount = Planet.Plates.Num();
		OutStats.ContinentalSampleCount = 0;
		OutStats.BoundarySampleCount = 0;
		OutStats.MaxComponentsPerPlate = 0;
		OutStats.ActiveCollisionTrackCount = Planet.BoundaryContactPersistenceByPair.Num();
		OutStats.ContinentalAreaFraction = 0.0;
		OutStats.BoundarySampleFraction = 0.0;
		OutStats.GapRate = 0.0;
		OutStats.OverlapRate = 0.0;
		OutStats.ElevationMinKm = 0.0;
		OutStats.ElevationMaxKm = 0.0;
		OutStats.ElevationMeanKm = 0.0;

		if (Planet.Samples.IsEmpty())
		{
			return;
		}

		double ElevationSumKm = 0.0;
		OutStats.ElevationMinKm = TNumericLimits<double>::Max();
		OutStats.ElevationMaxKm = -TNumericLimits<double>::Max();
		for (const FSample& Sample : Planet.Samples)
		{
			OutStats.ContinentalSampleCount += Sample.ContinentalWeight >= 0.5f ? 1 : 0;
			OutStats.BoundarySampleCount += Sample.bIsBoundary ? 1 : 0;
			const double ElevationKm = static_cast<double>(Sample.Elevation);
			OutStats.ElevationMinKm = FMath::Min(OutStats.ElevationMinKm, ElevationKm);
			OutStats.ElevationMaxKm = FMath::Max(OutStats.ElevationMaxKm, ElevationKm);
			ElevationSumKm += ElevationKm;
		}

		const double SampleCount = static_cast<double>(Planet.Samples.Num());
		OutStats.ContinentalAreaFraction = static_cast<double>(OutStats.ContinentalSampleCount) / SampleCount;
		OutStats.BoundarySampleFraction = static_cast<double>(OutStats.BoundarySampleCount) / SampleCount;
		OutStats.GapRate = static_cast<double>(OutStats.GapCount) / SampleCount;
		OutStats.OverlapRate = static_cast<double>(OutStats.OverlapCount) / SampleCount;
		OutStats.ElevationMeanKm = ElevationSumKm / SampleCount;

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		for (const FPlate& Plate : Planet.Plates)
		{
			int32 ComponentCount = 0;
			for (const int32 SeedSampleIndex : Plate.MemberSamples)
			{
				if (!Planet.Samples.IsValidIndex(SeedSampleIndex) || Visited[SeedSampleIndex] != 0)
				{
					continue;
				}

				TArray<int32, TInlineAllocator<128>> Stack;
				Stack.Add(SeedSampleIndex);
				Visited[SeedSampleIndex] = 1;

				while (!Stack.IsEmpty())
				{
					const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
					if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
					{
						continue;
					}

					for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
					{
						if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
							Visited[NeighborIndex] != 0 ||
							Planet.Samples[NeighborIndex].PlateId != Plate.Id)
						{
							continue;
						}

						Visited[NeighborIndex] = 1;
						Stack.Add(NeighborIndex);
					}
				}

				++ComponentCount;
			}

			OutStats.MaxComponentsPerPlate = FMath::Max(OutStats.MaxComponentsPerPlate, ComponentCount);
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

	bool IsGeometricCollisionPairStronger(
		const FGeometricCollisionPairDiagnostics& Left,
		const FGeometricCollisionPairDiagnostics& Right)
	{
		if (Left.bPassedMassFilter != Right.bPassedMassFilter)
		{
			return Left.bPassedMassFilter;
		}
		if (Left.TerraneEstimate != Right.TerraneEstimate)
		{
			return Left.TerraneEstimate > Right.TerraneEstimate;
		}
		if (Left.TotalOverlapSampleCount != Right.TotalOverlapSampleCount)
		{
			return Left.TotalOverlapSampleCount > Right.TotalOverlapSampleCount;
		}
		if (!FMath::IsNearlyEqual(
				Left.TotalConvergenceMagnitudeKmPerMy,
				Right.TotalConvergenceMagnitudeKmPerMy,
				UE_DOUBLE_SMALL_NUMBER))
		{
			return Left.TotalConvergenceMagnitudeKmPerMy > Right.TotalConvergenceMagnitudeKmPerMy;
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

	bool IsRepeatedGeometricCollisionCandidateStronger(
		const FGeometricCollisionPairRecurrenceState& LeftState,
		const FGeometricCollisionPairRecurrenceState& RightState,
		const FGeometricCollisionPairDiagnostics& LeftPair,
		const FGeometricCollisionPairDiagnostics& RightPair)
	{
		if (LeftState.ObservationCount != RightState.ObservationCount)
		{
			return LeftState.ObservationCount > RightState.ObservationCount;
		}
		if (!FMath::IsNearlyEqual(
				LeftState.AccumulatedPenetrationKm,
				RightState.AccumulatedPenetrationKm,
				UE_DOUBLE_SMALL_NUMBER))
		{
			return LeftState.AccumulatedPenetrationKm > RightState.AccumulatedPenetrationKm;
		}
		if (LeftPair.TerraneEstimate != RightPair.TerraneEstimate)
		{
			return LeftPair.TerraneEstimate > RightPair.TerraneEstimate;
		}
		if (LeftPair.PlateA != RightPair.PlateA)
		{
			return LeftPair.PlateA < RightPair.PlateA;
		}
		if (LeftPair.PlateB != RightPair.PlateB)
		{
			return LeftPair.PlateB < RightPair.PlateB;
		}
		return LeftPair.MinSampleIndex < RightPair.MinSampleIndex;
	}

	bool TryFindMeaningfulContainmentHitInPlate(
		const FTectonicPlanet& Planet,
		const FPlate& Plate,
		const FVector3d& QueryPoint)
	{
		if (Plate.SoupData.LocalTriangles.IsEmpty())
		{
			return false;
		}
		if (!Plate.BoundingCap.Center.IsNearlyZero() &&
			QueryPoint.Dot(Plate.BoundingCap.Center) < Plate.BoundingCap.CosAngle)
		{
			return false;
		}

		int32 LocalTriangleId = INDEX_NONE;
		FVector3d A;
		FVector3d B;
		FVector3d C;
		if (!FindContainingTriangleInBVH(Plate.SoupBVH, Plate.SoupAdapter, QueryPoint, LocalTriangleId, A, B, C))
		{
			return false;
		}

		const FVector3d Barycentric = NormalizeBarycentric(ComputePlanarBarycentric(A, B, C, QueryPoint));
		return ComputeContainmentScore(Barycentric) > OverlapScoreEpsilon;
	}

	double ComputeGeometricOverlapConvergenceMagnitudeKmPerMy(
		const FTectonicPlanet& Planet,
		const int32 OwningPlateId,
		const int32 ContainingPlateId,
		const FVector3d& QueryPoint)
	{
		const FPlate* OwningPlate = FindPlateById(Planet, OwningPlateId);
		const FPlate* ContainingPlate = FindPlateById(Planet, ContainingPlateId);
		if (OwningPlate == nullptr || ContainingPlate == nullptr || OwningPlateId == ContainingPlateId)
		{
			return 0.0;
		}

		int32 NearestContainingMember = INDEX_NONE;
		double NearestContainingDistance = TNumericLimits<double>::Max();
		if (!FindNearestMemberSample(
				Planet,
				ContainingPlateId,
				QueryPoint,
				NearestContainingMember,
				NearestContainingDistance) ||
			!Planet.Samples.IsValidIndex(NearestContainingMember))
		{
			return 0.0;
		}

		const FVector3d ContainingPosition =
			ContainingPlate->CumulativeRotation.RotateVector(Planet.Samples[NearestContainingMember].Position).GetSafeNormal();
		const FVector3d SeparationDirection =
			ProjectOntoTangent(ContainingPosition - QueryPoint, QueryPoint).GetSafeNormal();
		if (SeparationDirection.IsNearlyZero())
		{
			return 0.0;
		}

		const FVector3d RelativeVelocity =
			ComputePlateSurfaceVelocity(*ContainingPlate, QueryPoint, Planet.PlanetRadiusKm) -
			ComputePlateSurfaceVelocity(*OwningPlate, QueryPoint, Planet.PlanetRadiusKm);
		const double ClosingSpeed = -RelativeVelocity.Dot(SeparationDirection);
		if (ClosingSpeed <= UE_DOUBLE_SMALL_NUMBER)
		{
			return 0.0;
		}

		return ClosingSpeed / FMath::Max(DeltaTimeMyears, UE_DOUBLE_SMALL_NUMBER);
	}

	int32 EstimateGeometricCollisionTerraneSize(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 PlateId,
		const TArray<int32>& SeedSampleIndices)
	{
		if (PlateAssignments.Num() != Planet.Samples.Num() || PlateId == INDEX_NONE)
		{
			return 0;
		}

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		TArray<int32, TInlineAllocator<128>> Stack;
		for (const int32 SeedSampleIndex : SeedSampleIndices)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) ||
				Visited[SeedSampleIndex] != 0 ||
				PlateAssignments[SeedSampleIndex] != PlateId ||
				Planet.Samples[SeedSampleIndex].ContinentalWeight < 0.5f)
			{
				continue;
			}

			Visited[SeedSampleIndex] = 1;
			Stack.Add(SeedSampleIndex);
		}

		int32 TerraneSize = 0;
		while (!Stack.IsEmpty())
		{
			const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
			++TerraneSize;
			if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
					Visited[NeighborIndex] != 0 ||
					PlateAssignments[NeighborIndex] != PlateId ||
					Planet.Samples[NeighborIndex].ContinentalWeight < 0.5f)
				{
					continue;
				}

				Visited[NeighborIndex] = 1;
				Stack.Add(NeighborIndex);
			}
		}

		return TerraneSize;
	}

	void CollectBoundarySamplesForPlate(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 PlateId,
		TArray<int32>& OutBoundarySampleIndices)
	{
		OutBoundarySampleIndices.Reset();
		if (PlateAssignments.Num() != Planet.Samples.Num() || PlateId == INDEX_NONE)
		{
			return;
		}

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (PlateAssignments[SampleIndex] != PlateId || !Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			bool bIsBoundarySample = false;
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!PlateAssignments.IsValidIndex(NeighborIndex))
				{
					continue;
				}
				if (PlateAssignments[NeighborIndex] != PlateId)
				{
					bIsBoundarySample = true;
					break;
				}
			}

			if (bIsBoundarySample)
			{
				OutBoundarySampleIndices.Add(SampleIndex);
			}
		}
	}

	bool ComputeDirectionalOverlapDepthStatsKm(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 ReceiverPlateId,
		const TArray<int32>& DirectionalSeedSamples,
		double& OutMeanDepthKm,
		double& OutMaxDepthKm)
	{
		OutMeanDepthKm = 0.0;
		OutMaxDepthKm = 0.0;
		if (ReceiverPlateId == INDEX_NONE ||
			DirectionalSeedSamples.IsEmpty() ||
			PlateAssignments.Num() != Planet.Samples.Num())
		{
			return false;
		}

		TArray<int32> ReceiverBoundarySamples;
		CollectBoundarySamplesForPlate(Planet, PlateAssignments, ReceiverPlateId, ReceiverBoundarySamples);
		if (ReceiverBoundarySamples.IsEmpty())
		{
			return false;
		}

		double TotalDepthKm = 0.0;
		int32 ValidSeedCount = 0;
		for (const int32 SeedSampleIndex : DirectionalSeedSamples)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex))
			{
				continue;
			}

			double MinBoundaryDistanceKm = TNumericLimits<double>::Max();
			const FVector3d SeedPosition = Planet.Samples[SeedSampleIndex].Position.GetSafeNormal();
			for (const int32 BoundarySampleIndex : ReceiverBoundarySamples)
			{
				if (!Planet.Samples.IsValidIndex(BoundarySampleIndex))
				{
					continue;
				}

				const double DistanceKm =
					ComputeGeodesicDistance(
						SeedPosition,
						Planet.Samples[BoundarySampleIndex].Position.GetSafeNormal()) * Planet.PlanetRadiusKm;
				MinBoundaryDistanceKm = FMath::Min(MinBoundaryDistanceKm, DistanceKm);
			}

			if (!FMath::IsFinite(MinBoundaryDistanceKm) || MinBoundaryDistanceKm == TNumericLimits<double>::Max())
			{
				continue;
			}

			TotalDepthKm += MinBoundaryDistanceKm;
			OutMaxDepthKm = FMath::Max(OutMaxDepthKm, MinBoundaryDistanceKm);
			++ValidSeedCount;
		}

		if (ValidSeedCount <= 0)
		{
			OutMaxDepthKm = 0.0;
			return false;
		}

		OutMeanDepthKm = TotalDepthKm / static_cast<double>(ValidSeedCount);
		return true;
	}

	void AccumulateGeometricCollisionFailureReason(
		FResamplingStats& InOutStats,
		const TCHAR* FailureReason)
	{
		if (FailureReason == nullptr)
		{
			return;
		}

		if (FCString::Strcmp(FailureReason, TEXT("donor_seed_empty")) == 0 ||
			FCString::Strcmp(FailureReason, TEXT("no_subducting_side_seeds")) == 0)
		{
			++InOutStats.GeometricCollisionQualifiedButDonorSeedEmptyCount;
			return;
		}

		if (FCString::Strcmp(FailureReason, TEXT("empty_terrane")) == 0)
		{
			++InOutStats.GeometricCollisionRejectedByEmptyTerraneCount;
			return;
		}

		if (FCString::Strcmp(FailureReason, TEXT("overlap_too_shallow")) == 0)
		{
			++InOutStats.GeometricCollisionRejectedByOverlapDepthCount;
			return;
		}
		if (FCString::Strcmp(FailureReason, TEXT("persistent_penetration_below_threshold")) == 0)
		{
			++InOutStats.GeometricCollisionRejectedByPersistentPenetrationCount;
			return;
		}

		if (FCString::Strcmp(FailureReason, TEXT("donor_ambiguous")) == 0)
		{
			++InOutStats.GeometricCollisionQualifiedButDonorAmbiguousCount;
			return;
		}

		if (FCString::Strcmp(FailureReason, TEXT("invalid_pair")) == 0 ||
			FCString::Strcmp(FailureReason, TEXT("invalid_contact_center")) == 0)
		{
			++InOutStats.GeometricCollisionRejectedByRoleResolutionCount;
		}
	}

	bool AssignGeometricCollisionDirectionalBucketsForRoles(
		const int32 PlateA,
		const int32 PlateB,
		const TArray<int32>& SamplesFromAInsideB,
		const TArray<int32>& SamplesFromBInsideA,
		const int32 OverridingPlateId,
		const int32 SubductingPlateId,
		const TArray<int32>*& OutDirectionalSeedSamples,
		const TArray<int32>*& OutOppositeDirectionalSamples)
	{
		OutDirectionalSeedSamples = nullptr;
		OutOppositeDirectionalSamples = nullptr;
		if (PlateA == PlateB)
		{
			return false;
		}

		if (OverridingPlateId == PlateA && SubductingPlateId == PlateB)
		{
			OutDirectionalSeedSamples = &SamplesFromBInsideA;
			OutOppositeDirectionalSamples = &SamplesFromAInsideB;
			return true;
		}
		if (OverridingPlateId == PlateB && SubductingPlateId == PlateA)
		{
			OutDirectionalSeedSamples = &SamplesFromAInsideB;
			OutOppositeDirectionalSamples = &SamplesFromBInsideA;
			return true;
		}
		return false;
	}

	bool ResolveGeometricCollisionDonorReceiver(
		const FTectonicPlanet& Planet,
		const FGeometricCollisionPairDiagnostics& PairDiagnostics,
		int32& OutReceiverPlateId,
		int32& OutDonorPlateId,
		const TArray<int32>*& OutDonorDirectionalSeedSamples,
		const TArray<int32>*& OutOppositeDirectionalSamples,
		EGeometricCollisionDonorSelectionRule& OutRule)
	{
		OutReceiverPlateId = INDEX_NONE;
		OutDonorPlateId = INDEX_NONE;
		OutDonorDirectionalSeedSamples = nullptr;
		OutOppositeDirectionalSamples = nullptr;
		OutRule = EGeometricCollisionDonorSelectionRule::None;

		if (FindPlateById(Planet, PairDiagnostics.PlateA) == nullptr ||
			FindPlateById(Planet, PairDiagnostics.PlateB) == nullptr ||
			PairDiagnostics.PlateA == PairDiagnostics.PlateB)
		{
			return false;
		}

		const int32 AInsideBCount = PairDiagnostics.SamplesFromAInsideB.Num();
		const int32 BInsideACount = PairDiagnostics.SamplesFromBInsideA.Num();
		if (AInsideBCount <= 0 && BInsideACount <= 0)
		{
			return false;
		}

		if (AInsideBCount > 0 && BInsideACount <= 0)
		{
			OutReceiverPlateId = PairDiagnostics.PlateB;
			OutDonorPlateId = PairDiagnostics.PlateA;
			OutRule = EGeometricCollisionDonorSelectionRule::DirectionalSingleBucket;
		}
		else if (BInsideACount > 0 && AInsideBCount <= 0)
		{
			OutReceiverPlateId = PairDiagnostics.PlateA;
			OutDonorPlateId = PairDiagnostics.PlateB;
			OutRule = EGeometricCollisionDonorSelectionRule::DirectionalSingleBucket;
		}
		else if (AInsideBCount != BInsideACount)
		{
			const bool bAIsDonor = AInsideBCount > BInsideACount;
			OutReceiverPlateId = bAIsDonor ? PairDiagnostics.PlateB : PairDiagnostics.PlateA;
			OutDonorPlateId = bAIsDonor ? PairDiagnostics.PlateA : PairDiagnostics.PlateB;
			OutRule = EGeometricCollisionDonorSelectionRule::DirectionalCount;
		}
		else if (!FMath::IsNearlyEqual(
					 PairDiagnostics.TotalConvergenceMagnitudeAInsideBKmPerMy,
					 PairDiagnostics.TotalConvergenceMagnitudeBInsideAKmPerMy,
					 UE_DOUBLE_SMALL_NUMBER))
		{
			const bool bAIsDonor =
				PairDiagnostics.TotalConvergenceMagnitudeAInsideBKmPerMy >
				PairDiagnostics.TotalConvergenceMagnitudeBInsideAKmPerMy;
			OutReceiverPlateId = bAIsDonor ? PairDiagnostics.PlateB : PairDiagnostics.PlateA;
			OutDonorPlateId = bAIsDonor ? PairDiagnostics.PlateA : PairDiagnostics.PlateB;
			OutRule = EGeometricCollisionDonorSelectionRule::DirectionalConvergence;
		}
		else
		{
			const FPlate* PlateA = FindPlateById(Planet, PairDiagnostics.PlateA);
			const FPlate* PlateB = FindPlateById(Planet, PairDiagnostics.PlateB);
			if (PlateA == nullptr || PlateB == nullptr)
			{
				return false;
			}

			if (PlateA->OverlapScore != PlateB->OverlapScore)
			{
				const bool bAIsDonor = PlateA->OverlapScore < PlateB->OverlapScore;
				OutReceiverPlateId = bAIsDonor ? PairDiagnostics.PlateB : PairDiagnostics.PlateA;
				OutDonorPlateId = bAIsDonor ? PairDiagnostics.PlateA : PairDiagnostics.PlateB;
				OutRule = EGeometricCollisionDonorSelectionRule::FallbackLowerPlateScore;
			}
			else
			{
				OutReceiverPlateId = PairDiagnostics.PlateA;
				OutDonorPlateId = PairDiagnostics.PlateB;
				OutRule = EGeometricCollisionDonorSelectionRule::FallbackStablePairTieBreak;
			}
		}

		return AssignGeometricCollisionDirectionalBucketsForRoles(
			PairDiagnostics.PlateA,
			PairDiagnostics.PlateB,
			PairDiagnostics.SamplesFromAInsideB,
			PairDiagnostics.SamplesFromBInsideA,
			OutReceiverPlateId,
			OutDonorPlateId,
			OutDonorDirectionalSeedSamples,
			OutOppositeDirectionalSamples);
	}

	int32 ChooseDominantPreviousTerraneId(
		const TArray<int32>& PreviousTerraneAssignments,
		const TArray<int32>& TerraneSampleIndices);

	bool TryBuildCollisionEventFromSeedSamples(
		const FTectonicPlanet& Planet,
		const TArray<int32>& CollisionSeedSampleIndices,
		const int32 OverridingPlateId,
		const int32 SubductingPlateId,
		const FVector3d& ContactCenterHint,
		const TArray<int32>& PreviousPlateIds,
		const TArray<float>& PreviousContinentalWeights,
		const TArray<int32>& PreviousTerraneAssignments,
		TArray<int32>& InOutNewPlateIds,
		FCollisionEvent& OutEvent,
		int32& OutTerraneSeedCount,
		int32& OutRecoveredTerraneCount,
		int32& OutLocalityLimitedCount,
		const TCHAR*& OutFailureReason,
		FCollisionTransferProtectionDiagnostics* OutProtectionDiagnostics = nullptr)
	{
		OutEvent = FCollisionEvent{};
		OutTerraneSeedCount = 0;
		OutRecoveredTerraneCount = 0;
		OutLocalityLimitedCount = 0;
		OutFailureReason = TEXT("none");
		if (OutProtectionDiagnostics != nullptr)
		{
			*OutProtectionDiagnostics = FCollisionTransferProtectionDiagnostics{};
		}

		if (FindPlateById(Planet, OverridingPlateId) == nullptr ||
			FindPlateById(Planet, SubductingPlateId) == nullptr ||
			OverridingPlateId == SubductingPlateId)
		{
			OutFailureReason = TEXT("invalid_pair");
			return false;
		}

		TArray<int32> TerraneSeedSampleIndices;
		for (const int32 SampleIndex : CollisionSeedSampleIndices)
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

		const int32 UnboundedTerraneSize = EstimateGeometricCollisionTerraneSize(
			Planet,
			PreviousPlateIds,
			SubductingPlateId,
			TerraneSeedSampleIndices);

		TArray<uint8> AllOverlapSeedMask;
		AllOverlapSeedMask.Init(0, Planet.Samples.Num());
		for (const int32 SeedSampleIndex : TerraneSeedSampleIndices)
		{
			if (AllOverlapSeedMask.IsValidIndex(SeedSampleIndex))
			{
				AllOverlapSeedMask[SeedSampleIndex] = 1;
			}
		}

		const auto BuildLocalityLimitedTerraneFromSeeds =
			[&Planet, &PreviousPlateIds, &PreviousContinentalWeights, &AllOverlapSeedMask, SubductingPlateId](
				const TArray<int32>& SelectedSeedSampleIndices,
				const bool bRestrictThroughOtherOverlapSeeds,
				TArray<double>& OutBestDistancesKm,
				TArray<int32>& OutTerraneSampleIndices)
			{
				OutBestDistancesKm.Init(TNumericLimits<double>::Max(), Planet.Samples.Num());
				OutTerraneSampleIndices.Reset();
				OutTerraneSampleIndices.Reserve(SelectedSeedSampleIndices.Num());

				TArray<uint8> SelectedSeedMask;
				if (bRestrictThroughOtherOverlapSeeds)
				{
					SelectedSeedMask.Init(0, Planet.Samples.Num());
					for (const int32 SeedSampleIndex : SelectedSeedSampleIndices)
					{
						if (SelectedSeedMask.IsValidIndex(SeedSampleIndex))
						{
							SelectedSeedMask[SeedSampleIndex] = 1;
						}
					}
				}

				TArray<int32> Frontier;
				Frontier.Reserve(SelectedSeedSampleIndices.Num());
				for (const int32 SeedSampleIndex : SelectedSeedSampleIndices)
				{
					if (!Planet.Samples.IsValidIndex(SeedSampleIndex) ||
						!PreviousPlateIds.IsValidIndex(SeedSampleIndex) ||
						!PreviousContinentalWeights.IsValidIndex(SeedSampleIndex) ||
						PreviousPlateIds[SeedSampleIndex] != SubductingPlateId ||
						PreviousContinentalWeights[SeedSampleIndex] < 0.5f ||
						OutBestDistancesKm[SeedSampleIndex] <= 0.0)
					{
						continue;
					}

					OutBestDistancesKm[SeedSampleIndex] = 0.0;
					Frontier.Add(SeedSampleIndex);
				}

				TArray<uint8> AddedToTerrane;
				AddedToTerrane.Init(0, Planet.Samples.Num());
				const double MaxPropagationKm = Planet.GeometricCollisionDonorLocalityClampKm;
				while (!Frontier.IsEmpty())
				{
					int32 BestFrontierIndex = 0;
					double BestDistanceKm = OutBestDistancesKm[Frontier[0]];
					for (int32 FrontierIndex = 1; FrontierIndex < Frontier.Num(); ++FrontierIndex)
					{
						const double CandidateDistanceKm = OutBestDistancesKm[Frontier[FrontierIndex]];
						if (CandidateDistanceKm < BestDistanceKm)
						{
							BestDistanceKm = CandidateDistanceKm;
							BestFrontierIndex = FrontierIndex;
						}
					}

					const int32 SampleIndex = Frontier[BestFrontierIndex];
					Frontier.RemoveAtSwap(BestFrontierIndex, 1, EAllowShrinking::No);
					if (!Planet.Samples.IsValidIndex(SampleIndex) ||
						AddedToTerrane[SampleIndex] != 0 ||
						BestDistanceKm > MaxPropagationKm)
					{
						continue;
					}

					AddedToTerrane[SampleIndex] = 1;
					OutTerraneSampleIndices.Add(SampleIndex);

					if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
					{
						continue;
					}

					const FVector3d SamplePosition = Planet.Samples[SampleIndex].Position.GetSafeNormal();
					for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
					{
						if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
							!PreviousPlateIds.IsValidIndex(NeighborIndex) ||
							!PreviousContinentalWeights.IsValidIndex(NeighborIndex) ||
							PreviousPlateIds[NeighborIndex] != SubductingPlateId ||
							PreviousContinentalWeights[NeighborIndex] < 0.5f)
						{
							continue;
						}

						if (bRestrictThroughOtherOverlapSeeds &&
							AllOverlapSeedMask.IsValidIndex(NeighborIndex) &&
							AllOverlapSeedMask[NeighborIndex] != 0 &&
							(!SelectedSeedMask.IsValidIndex(NeighborIndex) ||
								SelectedSeedMask[NeighborIndex] == 0))
						{
							continue;
						}

						const double EdgeDistanceKm =
							ComputeGeodesicDistance(
								SamplePosition,
								Planet.Samples[NeighborIndex].Position.GetSafeNormal()) * Planet.PlanetRadiusKm;
						const double CandidateDistanceKm = BestDistanceKm + EdgeDistanceKm;
						if (CandidateDistanceKm > MaxPropagationKm)
						{
							continue;
						}

						if (CandidateDistanceKm + UE_DOUBLE_SMALL_NUMBER < OutBestDistancesKm[NeighborIndex])
						{
							OutBestDistancesKm[NeighborIndex] = CandidateDistanceKm;
							Frontier.Add(NeighborIndex);
						}
					}
				}
			};

		TArray<double> BestDistancesKm;
		TArray<int32> TerraneSampleIndices;
		BuildLocalityLimitedTerraneFromSeeds(
			TerraneSeedSampleIndices,
			false,
			BestDistancesKm,
			TerraneSampleIndices);

		OutRecoveredTerraneCount = TerraneSampleIndices.Num();
		if (TerraneSampleIndices.IsEmpty())
		{
			OutFailureReason = TEXT("empty_terrane");
			return false;
		}

		const auto EvaluateTransferSubset =
			[&Planet, &PreviousPlateIds, OverridingPlateId, SubductingPlateId](
				const TArray<int32>& CandidateSampleIndices,
				FCollisionTransferConnectivityDiagnostics& OutDiagnostics)
			{
				OutDiagnostics = ComputeCollisionTransferConnectivityDiagnostics(
					Planet,
					PreviousPlateIds,
					OverridingPlateId,
					SubductingPlateId,
					CandidateSampleIndices);
				return PassesCollisionDonorConnectivityProtection(Planet, OutDiagnostics) &&
					PassesCollisionReceiverConnectivityProtection(OutDiagnostics);
			};

		FCollisionTransferConnectivityDiagnostics ProposedDiagnostics;
		const bool bFullTransferPassesProtection =
			EvaluateTransferSubset(TerraneSampleIndices, ProposedDiagnostics);
		if (OutProtectionDiagnostics != nullptr)
		{
			OutProtectionDiagnostics->Proposed = ProposedDiagnostics;
			OutProtectionDiagnostics->ProposedTerraneSampleCount = TerraneSampleIndices.Num();
		}

		const FCollisionDonorProtectionResult ProposedDonorProtectionResult =
			EvaluateCollisionDonorConnectivityProtection(Planet, ProposedDiagnostics);
		const bool bProposedFailsDonorProtection =
			!ProposedDonorProtectionResult.bPasses;
		const bool bProposedFailsReceiverProtection =
			!PassesCollisionReceiverConnectivityProtection(ProposedDiagnostics);

		TArray<int32> FinalTerraneSampleIndices = TerraneSampleIndices;
		FCollisionTransferConnectivityDiagnostics AppliedDiagnostics = ProposedDiagnostics;
		if (!bFullTransferPassesProtection)
		{
			TArray<int32> BestProtectedSubset;
			FCollisionTransferConnectivityDiagnostics BestProtectedDiagnostics;
			const auto TryAdoptProtectedSubset =
				[&BestProtectedSubset, &BestProtectedDiagnostics, &TerraneSampleIndices](
					const TArray<int32>& CandidateSubset,
					const FCollisionTransferConnectivityDiagnostics& CandidateDiagnostics)
				{
					if (CandidateSubset.IsEmpty() ||
						CandidateSubset.Num() >= TerraneSampleIndices.Num())
					{
						return false;
					}

					if (BestProtectedSubset.IsEmpty() ||
						CandidateSubset.Num() > BestProtectedSubset.Num())
					{
						BestProtectedSubset = CandidateSubset;
						BestProtectedDiagnostics = CandidateDiagnostics;
						return true;
					}

					return false;
				};

			for (const int32 SeedSampleIndex : TerraneSeedSampleIndices)
			{
				TArray<double> SingleSeedDistancesKm;
				TArray<int32> SingleSeedTerraneSampleIndices;
				BuildLocalityLimitedTerraneFromSeeds(
					TArray<int32>{ SeedSampleIndex },
					true,
					SingleSeedDistancesKm,
					SingleSeedTerraneSampleIndices);

				FCollisionTransferConnectivityDiagnostics CandidateDiagnostics;
				if (!EvaluateTransferSubset(SingleSeedTerraneSampleIndices, CandidateDiagnostics))
				{
					continue;
				}

				TryAdoptProtectedSubset(
					SingleSeedTerraneSampleIndices,
					CandidateDiagnostics);
			}

			TArray<double> CandidateDistanceThresholdsKm;
			TArray<double> SortedTerraneDistancesKm;
			SortedTerraneDistancesKm.Reserve(TerraneSampleIndices.Num());
			for (const int32 SampleIndex : TerraneSampleIndices)
			{
				if (BestDistancesKm.IsValidIndex(SampleIndex) &&
					FMath::IsFinite(BestDistancesKm[SampleIndex]))
				{
					SortedTerraneDistancesKm.Add(BestDistancesKm[SampleIndex]);
				}
			}
			SortedTerraneDistancesKm.Sort();

			const TArray<double> DistancePercentiles = {
				0.85,
				0.70,
				0.55,
				0.40,
				0.25,
				0.10,
				0.0
			};
			double LastThresholdKm = -1.0;
			for (const double Percentile : DistancePercentiles)
			{
				if (SortedTerraneDistancesKm.IsEmpty())
				{
					break;
				}

				const int32 ThresholdIndex = FMath::Clamp(
					FMath::FloorToInt(
						static_cast<double>(SortedTerraneDistancesKm.Num() - 1) * Percentile),
					0,
					SortedTerraneDistancesKm.Num() - 1);
				const double ThresholdKm = SortedTerraneDistancesKm[ThresholdIndex];
				if (!CandidateDistanceThresholdsKm.IsEmpty() &&
					FMath::IsNearlyEqual(ThresholdKm, LastThresholdKm, UE_DOUBLE_SMALL_NUMBER))
				{
					continue;
				}

				CandidateDistanceThresholdsKm.Add(ThresholdKm);
				LastThresholdKm = ThresholdKm;
			}

			TArray<int32> CandidateSubset;
			for (const double ThresholdKm : CandidateDistanceThresholdsKm)
			{
				BuildCollisionTransferSubsetByDistanceThreshold(
					TerraneSampleIndices,
					BestDistancesKm,
					ThresholdKm,
					CandidateSubset);
				if (CandidateSubset.IsEmpty() || CandidateSubset.Num() >= FinalTerraneSampleIndices.Num())
				{
					continue;
				}

				FCollisionTransferConnectivityDiagnostics CandidateDiagnostics;
				if (!EvaluateTransferSubset(CandidateSubset, CandidateDiagnostics))
				{
					continue;
				}

				TryAdoptProtectedSubset(CandidateSubset, CandidateDiagnostics);
			}

			if (!BestProtectedSubset.IsEmpty())
			{
				FinalTerraneSampleIndices = BestProtectedSubset;
				AppliedDiagnostics = BestProtectedDiagnostics;
				if (OutProtectionDiagnostics != nullptr)
				{
					OutProtectionDiagnostics->bTrimmedByDonorProtection = bProposedFailsDonorProtection;
					OutProtectionDiagnostics->bTrimmedByDonorComponentCap =
						ProposedDonorProtectionResult.bFailedComponentCap;
					OutProtectionDiagnostics->bTrimmedByDonorFragmentFloor =
						ProposedDonorProtectionResult.bFailedFragmentFloor;
					OutProtectionDiagnostics->bTrimmedByReceiverProtection = bProposedFailsReceiverProtection;
					OutProtectionDiagnostics->AppliedTerraneSampleCount =
						FinalTerraneSampleIndices.Num();
					OutProtectionDiagnostics->AppliedTrimRatio =
						TerraneSampleIndices.IsEmpty()
							? 0.0
							: 1.0 - (
								static_cast<double>(FinalTerraneSampleIndices.Num()) /
								static_cast<double>(TerraneSampleIndices.Num()));
				}
			}
			else
			{
				if (OutProtectionDiagnostics != nullptr)
				{
					OutProtectionDiagnostics->bRejectedByDonorProtection = bProposedFailsDonorProtection;
					OutProtectionDiagnostics->bRejectedByDonorComponentCap =
						ProposedDonorProtectionResult.bFailedComponentCap;
					OutProtectionDiagnostics->bRejectedByDonorFragmentFloor =
						ProposedDonorProtectionResult.bFailedFragmentFloor;
					OutProtectionDiagnostics->bRejectedByReceiverProtection = bProposedFailsReceiverProtection;
					OutProtectionDiagnostics->AppliedTerraneSampleCount = 0;
					OutProtectionDiagnostics->AppliedTrimRatio = 1.0;
				}
				if (bProposedFailsDonorProtection && bProposedFailsReceiverProtection)
				{
					OutFailureReason = TEXT("donor_and_receiver_connectivity_protection");
				}
				else if (bProposedFailsDonorProtection)
				{
					OutFailureReason = TEXT("donor_connectivity_protection");
				}
				else
				{
					OutFailureReason = TEXT("receiver_connectivity_protection");
				}
				return false;
			}
		}

		OutRecoveredTerraneCount = FinalTerraneSampleIndices.Num();
		OutLocalityLimitedCount = FMath::Max(0, UnboundedTerraneSize - OutRecoveredTerraneCount);
		if (OutProtectionDiagnostics != nullptr)
		{
			OutProtectionDiagnostics->Applied = AppliedDiagnostics;
			if (OutProtectionDiagnostics->AppliedTerraneSampleCount <= 0)
			{
				OutProtectionDiagnostics->AppliedTerraneSampleCount =
					FinalTerraneSampleIndices.Num();
			}
			if (OutProtectionDiagnostics->AppliedTrimRatio <= 0.0 &&
				FinalTerraneSampleIndices.Num() == TerraneSampleIndices.Num())
			{
				OutProtectionDiagnostics->AppliedTrimRatio = 0.0;
			}
		}

		for (const int32 SampleIndex : FinalTerraneSampleIndices)
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
			for (const int32 SampleIndex : CollisionSeedSampleIndices)
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
		OutEvent.CollisionSampleIndices = CollisionSeedSampleIndices;
		OutEvent.TerraneSampleIndices = FinalTerraneSampleIndices;
		OutEvent.TerraneId = ChooseDominantPreviousTerraneId(PreviousTerraneAssignments, FinalTerraneSampleIndices);
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

	bool TryComputeAuthoritativeContinentalWeightForPlate(
		const FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const int32 PlateId,
		float& OutContinentalWeight)
	{
		OutContinentalWeight = 0.0f;
		if (!Planet.Samples.IsValidIndex(SampleIndex) || FindPlateById(Planet, PlateId) == nullptr)
		{
			return false;
		}

		double ContinentalWeight = 0.0;
		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		if (TryComputeContainedPlateContinentalWeight(Planet, PlateId, QueryPoint, ContinentalWeight))
		{
			OutContinentalWeight = static_cast<float>(FMath::Clamp(ContinentalWeight, 0.0, 1.0));
			return true;
		}

		FRecoveredContainmentHit RecoveryHit;
		if (TryFindNearestTriangleRecoveryHit(Planet, PlateId, QueryPoint, RecoveryHit) &&
			TryComputeRecoveredContinentalWeight(
				Planet,
				PlateId,
				RecoveryHit.GlobalTriangleIndex,
				RecoveryHit.Barycentric,
				ContinentalWeight))
		{
			OutContinentalWeight = static_cast<float>(FMath::Clamp(ContinentalWeight, 0.0, 1.0));
			return true;
		}

		int32 NearestMemberSampleIndex = INDEX_NONE;
		double NearestMemberDistance = TNumericLimits<double>::Max();
		if (!FindNearestMemberSample(
				Planet,
				PlateId,
				QueryPoint,
				NearestMemberSampleIndex,
				NearestMemberDistance))
		{
			return false;
		}

		const FCarriedSample* NearestCarriedSample =
			FindCarriedSampleForCanonicalVertex(Planet, PlateId, NearestMemberSampleIndex);
		if (NearestCarriedSample == nullptr)
		{
			return false;
		}

		OutContinentalWeight =
			FMath::Clamp(NearestCarriedSample->ContinentalWeight, 0.0f, 1.0f);
		return true;
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

	int32 CountAdjacentSamplesAssignedToPlate(
		const FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const int32 PlateId)
	{
		if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			return 0;
		}

		int32 Count = 0;
		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!Planet.Samples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			Count += Planet.Samples[NeighborIndex].PlateId == PlateId ? 1 : 0;
		}

		return Count;
	}

	bool TryResolvePreserveBoundaryPreviousOwnerHysteresis(
		const FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const int32 PreviousPlateId,
		const int32 WinningPlateId,
		FRecoveredContainmentHit& OutRecoveryHit)
	{
		OutRecoveryHit = FRecoveredContainmentHit{};
		if (!Planet.Samples.IsValidIndex(SampleIndex) ||
			PreviousPlateId == INDEX_NONE ||
			WinningPlateId == INDEX_NONE ||
			PreviousPlateId == WinningPlateId)
		{
			return false;
		}

		const double PreviousContinentalWeight = FMath::Clamp(
			static_cast<double>(Planet.Samples[SampleIndex].ContinentalWeight),
			0.0,
			1.0);
		if (PreviousContinentalWeight < PreserveBoundaryStrongContinentalThreshold)
		{
			return false;
		}

		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		if (!TryFindNearestTriangleRecoveryHit(Planet, PreviousPlateId, QueryPoint, OutRecoveryHit))
		{
			return false;
		}

		double RecoveredContinentalWeight = 0.0;
		if (!TryComputeRecoveredContinentalWeight(
				Planet,
				PreviousPlateId,
				OutRecoveryHit.GlobalTriangleIndex,
				OutRecoveryHit.Barycentric,
				RecoveredContinentalWeight) ||
			RecoveredContinentalWeight < PreserveBoundaryRecoveredContinentalThreshold)
		{
			return false;
		}

		const int32 PreviousOwnerNeighborVotes =
			CountAdjacentSamplesAssignedToPlate(Planet, SampleIndex, PreviousPlateId);
		const int32 WinningOwnerNeighborVotes =
			CountAdjacentSamplesAssignedToPlate(Planet, SampleIndex, WinningPlateId);
		return PreviousOwnerNeighborVotes > 0 &&
			PreviousOwnerNeighborVotes >= WinningOwnerNeighborVotes;
	}

	uint64 MakeOwnerTransitionKey(const int32 PreviousPlateId, const int32 FinalPlateId)
	{
		return
			(static_cast<uint64>(static_cast<uint32>(PreviousPlateId)) << 32) |
			static_cast<uint32>(FinalPlateId);
	}

	int32 DecodeOwnerTransitionPreviousPlateId(const uint64 Key)
	{
		return static_cast<int32>(static_cast<uint32>(Key >> 32));
	}

	int32 DecodeOwnerTransitionFinalPlateId(const uint64 Key)
	{
		return static_cast<int32>(static_cast<uint32>(Key & 0xffffffffU));
	}

	FString JoinOwnerTransitionCountsForLog(
		const TArray<int32>& PreviousPlateIds,
		const TArray<int32>& FinalPlateIds,
		const TArray<int32>& Counts)
	{
		if (PreviousPlateIds.Num() != FinalPlateIds.Num() ||
			PreviousPlateIds.Num() != Counts.Num() ||
			Counts.IsEmpty())
		{
			return TEXT("none");
		}

		TArray<FString> Parts;
		Parts.Reserve(Counts.Num());
		for (int32 Index = 0; Index < Counts.Num(); ++Index)
		{
			Parts.Add(FString::Printf(
				TEXT("%d->%d:%d"),
				PreviousPlateIds[Index],
				FinalPlateIds[Index],
				Counts[Index]));
		}

		return FString::Join(Parts, TEXT(","));
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
		const int32 DominantBarycentricIndex = PickDominantBarycentricIndex(Barycentric);
		const FCarriedSample& DominantSource =
			DominantBarycentricIndex == 1
				? V1
				: (DominantBarycentricIndex == 2 ? V2 : V0);

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
		Sample.ContinentalWeight =
			FMath::Clamp(DominantSource.ContinentalWeight, 0.0f, 1.0f);
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

		switch (DominantBarycentricIndex)
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

			if (Planet.bUseVertexLevelSoupInclusionForTest)
			{
				// Spike path: include triangle in every plate that owns at least one vertex.
				TArray<int32, TInlineAllocator<3>> SoupPlates;
				if (FindPlateById(Planet, PlateA) != nullptr)
				{
					SoupPlates.AddUnique(PlateA);
				}
				if (FindPlateById(Planet, PlateB) != nullptr)
				{
					SoupPlates.AddUnique(PlateB);
				}
				if (FindPlateById(Planet, PlateC) != nullptr)
				{
					SoupPlates.AddUnique(PlateC);
				}
				for (const int32 SoupPlateId : SoupPlates)
				{
					if (FPlate* SoupPlate = FindPlateById(Planet, SoupPlateId))
					{
						SoupPlate->SoupTriangles.Add(TriangleIndex);
					}
				}
			}
			else
			{
				// Legacy path: majority-vote single-owner assignment.
				const int32 AssignedPlateId = ChooseSoupPlateIdForTriangle(Planet, Triangle);
				if (FPlate* AssignedPlate = FindPlateById(Planet, AssignedPlateId))
				{
					AssignedPlate->SoupTriangles.Add(TriangleIndex);
				}
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
		const double MaxInitialAngularSpeed = InitialPlateSpeedMaxKmPerStep / FMath::Max(Planet.PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);
		const double MinInitialAngularSpeed = InitialPlateSpeedMinKmPerStep / FMath::Max(Planet.PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);

		for (FPlate& Plate : Planet.Plates)
		{
			Plate.RotationAxis = MakeRandomUnitVector(Random).GetSafeNormal();
			Plate.AngularSpeed = Random.FRandRange(static_cast<float>(MinInitialAngularSpeed), static_cast<float>(MaxInitialAngularSpeed));
			Plate.CumulativeRotation = FQuat4d::Identity;
			ResetSoupState(Plate);
		}
	}
}

FTectonicPlanetRuntimeConfig GetM6BaselineRuntimeConfig()
{
	FTectonicPlanetRuntimeConfig Config;
	Config.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	Config.bEnableAutomaticRifting = true;
	Config.bEnableWarpedRiftBoundaries = true;
	Config.AutomaticRiftMinParentSamples = 1500;
	Config.AutomaticRiftMinContinentalSamples = 64;
	Config.AutomaticRiftCooldownSteps = 20;
	Config.AutomaticRiftBaseRatePerMy = 0.18;
	Config.AutomaticRiftMinContinentalFraction = 0.05;
	Config.RiftBoundaryWarpAmplitude = 0.25;
	Config.RiftBoundaryWarpFrequency = 1.5;
	Config.AndeanContinentalConversionRatePerMy = 0.005;
	return Config;
}

FTectonicPlanetRuntimeConfig GetArchitectureSpikeARuntimeConfig()
{
	FTectonicPlanetRuntimeConfig Config = GetM6BaselineRuntimeConfig();
	Config.ResamplingPolicy = EResamplingPolicy::PeriodicGlobalAuthoritativeSpike;
	return Config;
}

FTectonicPlanetRuntimeConfig CaptureTectonicPlanetRuntimeConfig(const FTectonicPlanet& Planet)
{
	FTectonicPlanetRuntimeConfig Config;
	Config.ResamplingPolicy = Planet.ResamplingPolicy;
	Config.bEnableAutomaticRifting = Planet.bEnableAutomaticRifting;
	Config.bEnableWarpedRiftBoundaries = Planet.bEnableWarpedRiftBoundaries;
	Config.AutomaticRiftMinParentSamples = Planet.AutomaticRiftMinParentSamples;
	Config.AutomaticRiftMinContinentalSamples = Planet.AutomaticRiftMinContinentalSamples;
	Config.AutomaticRiftCooldownSteps = Planet.AutomaticRiftCooldownSteps;
	Config.AutomaticRiftBaseRatePerMy = Planet.AutomaticRiftBaseRatePerMy;
	Config.AutomaticRiftMinContinentalFraction = Planet.AutomaticRiftMinContinentalFraction;
	Config.RiftBoundaryWarpAmplitude = Planet.RiftBoundaryWarpAmplitude;
	Config.RiftBoundaryWarpFrequency = Planet.RiftBoundaryWarpFrequency;
	Config.AndeanContinentalConversionRatePerMy = Planet.AndeanContinentalConversionRatePerMy;
	return Config;
}

void ApplyTectonicPlanetRuntimeConfig(
	FTectonicPlanet& Planet,
	const FTectonicPlanetRuntimeConfig& Config)
{
	Planet.ResamplingPolicy = Config.ResamplingPolicy;
	Planet.bEnableAutomaticRifting = Config.bEnableAutomaticRifting;
	Planet.bEnableWarpedRiftBoundaries = Config.bEnableWarpedRiftBoundaries;
	Planet.AutomaticRiftMinParentSamples = Config.AutomaticRiftMinParentSamples;
	Planet.AutomaticRiftMinContinentalSamples = Config.AutomaticRiftMinContinentalSamples;
	Planet.AutomaticRiftCooldownSteps = Config.AutomaticRiftCooldownSteps;
	Planet.AutomaticRiftBaseRatePerMy = Config.AutomaticRiftBaseRatePerMy;
	Planet.AutomaticRiftMinContinentalFraction = Config.AutomaticRiftMinContinentalFraction;
	Planet.RiftBoundaryWarpAmplitude = Config.RiftBoundaryWarpAmplitude;
	Planet.RiftBoundaryWarpFrequency = Config.RiftBoundaryWarpFrequency;
	Planet.AndeanContinentalConversionRatePerMy = Config.AndeanContinentalConversionRatePerMy;
}

FString DescribeTectonicPlanetRuntimeConfig(const FTectonicPlanetRuntimeConfig& Config)
{
	return FString::Printf(
		TEXT("resampling_policy=%s automatic_rifting=%d automatic_rift_base_rate_per_my=%.3f automatic_rift_min_parent_samples=%d automatic_rift_min_continental_samples=%d automatic_rift_min_continental_fraction=%.3f automatic_rift_cooldown_steps=%d rift_boundary_warp_amplitude=%.3f rift_boundary_warp_frequency=%.3f andean_rate_per_my=%.3f"),
		GetResamplingPolicyName(Config.ResamplingPolicy),
		Config.bEnableAutomaticRifting ? 1 : 0,
		Config.AutomaticRiftBaseRatePerMy,
		Config.AutomaticRiftMinParentSamples,
		Config.AutomaticRiftMinContinentalSamples,
		Config.AutomaticRiftMinContinentalFraction,
		Config.AutomaticRiftCooldownSteps,
		Config.RiftBoundaryWarpAmplitude,
		Config.RiftBoundaryWarpFrequency,
		Config.AndeanContinentalConversionRatePerMy);
}

void FTectonicPlanet::Initialize(const int32 InSampleCount, const double InPlanetRadiusKm)
{
	SampleCountConfig = InSampleCount;
	PlanetRadiusKm = InPlanetRadiusKm;
	PlateCountConfig = 0;
	InitialPlateCountConfig = 0;
	NextPlateId = 0;
	CurrentStep = 0;
	NextTerraneId = 0;
	LastComputedResampleInterval = 0;
	MaxResampleCount = INDEX_NONE;
	MaxStepsWithoutResampling = INDEX_NONE;
	bEnableSlabPull = true;
	bEnableAndeanContinentalConversion = true;
	AndeanContinentalConversionRatePerMy = 0.005;
	bEnableOverlapHysteresis = false;
	bEnableContinentalCollision = true;
	ResamplingPolicy = EResamplingPolicy::PeriodicFull;
	bDeferRiftFollowupResamplingToV6 = false;
	LastResampleTriggerReason = EResampleTriggerReason::None;
	LastResampleOwnershipMode = EResampleOwnershipMode::FullResolution;
	bPendingFullResolutionResample = false;
	bPendingBoundaryContactPersistenceReset = false;
	LastResamplingStats = FResamplingStats{};
	ResamplingHistory.Reset();
	ResamplingSteps.Reset();
	BoundaryContactPersistenceByPair.Reset();
	CollisionLastExecutionOrdinalByPair.Reset();
	GeometricCollisionPairRecurrenceByKey.Reset();
	PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};
	PendingRiftEvent = FPendingRiftEvent{};
	ResamplingExecutionOrdinal = 0;
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
	InitialPlateCountConfig = PlateCountConfig;
	CurrentStep = 0;
	LastResamplingStats = FResamplingStats{};
	LastResampleTriggerReason = EResampleTriggerReason::None;
	LastResampleOwnershipMode = EResampleOwnershipMode::FullResolution;
	bPendingFullResolutionResample = false;
	bPendingBoundaryContactPersistenceReset = false;
	ResamplingHistory.Reset();
	ResamplingSteps.Reset();
	BoundaryContactPersistenceByPair.Reset();
	CollisionLastExecutionOrdinalByPair.Reset();
	GeometricCollisionPairRecurrenceByKey.Reset();
	PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};
	PendingRiftEvent = FPendingRiftEvent{};
	ResamplingExecutionOrdinal = 0;
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
				const double BaseUpliftKmPerMy =
					SubductionBaseUpliftKmPerMyForTest >= 0.0
						? SubductionBaseUpliftKmPerMyForTest
						: SubductionBaseUpliftKmPerMy;
				const double DistanceRad =
					static_cast<double>(CarriedSample.SubductionDistanceKm) / FMath::Max(PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);
				const double DistanceTransfer =
					SubductionDistanceTransfer(DistanceRad, SubductionControlDistanceRad, SubductionMaxDistanceRad);
				const double SpeedTransfer =
					static_cast<double>(CarriedSample.SubductionSpeed) / MaxPlateSpeedKmPerMy;
				const double ElevationTransfer =
					bDisableSubductionElevationTransferForTest
						? 1.0
						: ComputeSubductionElevationTransfer(static_cast<double>(CarriedSample.Elevation));
				const double UpliftKm =
					BaseUpliftKmPerMy *
					DistanceTransfer *
					SpeedTransfer *
					ElevationTransfer *
					DeltaTimeMyears;
				CarriedSample.Elevation += static_cast<float>(UpliftKm);
				const bool bStrongAndeanUplift =
					DistanceTransfer >= 0.5 &&
					SpeedTransfer >= 0.15 &&
					UpliftKm >= 0.015 &&
					CarriedSample.Elevation >= 0.25f;
				if (bStrongAndeanUplift && CarriedSample.OrogenyType == EOrogenyType::None)
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
					CarriedSample.ContinentalWeight +
						static_cast<float>(AndeanContinentalConversionRatePerMy * DeltaTimeMyears));
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
		case EResamplingPolicy::PeriodicGlobalAuthoritativeSpike:
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

double FTectonicPlanet::ComputeAutomaticRiftProbabilityForSampleCount(
	const int32 ParentSampleCount,
	const double ContinentalFraction) const
{
	if (Samples.IsEmpty() || ParentSampleCount <= 0)
	{
		return 0.0;
	}

	const double ParentSampleShare =
		FMath::Clamp(static_cast<double>(ParentSampleCount) / static_cast<double>(Samples.Num()), 0.0, 1.0);
	const double ContinentalScale = FMath::Clamp(
		ContinentalFraction / FMath::Max(AutomaticRiftMinContinentalFraction, 0.01),
		1.0,
		4.0);
	const int32 InitialPlateCount = FMath::Max(InitialPlateCountConfig, 1);
	const double Lambda =
		FMath::Max(0.0, AutomaticRiftBaseRatePerMy) *
		DeltaTimeMyears *
		ParentSampleShare *
		ContinentalScale /
		static_cast<double>(InitialPlateCount);
	return FMath::Clamp(1.0 - FMath::Exp(-Lambda), 0.0, 0.95);
}

int32 FTectonicPlanet::FindLargestEligibleAutomaticRiftParentId(
	int32* OutContinentalSampleCount,
	double* OutContinentalFraction) const
{
	if (OutContinentalSampleCount != nullptr)
	{
		*OutContinentalSampleCount = 0;
	}
	if (OutContinentalFraction != nullptr)
	{
		*OutContinentalFraction = 0.0;
	}

	int32 BestParentPlateId = INDEX_NONE;
	int32 BestParentSampleCount = -1;
	int32 BestContinentalSampleCount = 0;
	double BestContinentalFraction = 0.0;
	for (const FPlate& Plate : Plates)
	{
		int32 ContinentalSampleCount = 0;
		double ContinentalFraction = 0.0;
		if (!IsPlateEligibleForAutomaticRift(Plate, MinForcedRiftChildren, ContinentalSampleCount, ContinentalFraction))
		{
			continue;
		}
		if (Plate.MemberSamples.Num() > BestParentSampleCount ||
			(Plate.MemberSamples.Num() == BestParentSampleCount &&
				(BestParentPlateId == INDEX_NONE || Plate.Id < BestParentPlateId)))
		{
			BestParentPlateId = Plate.Id;
			BestParentSampleCount = Plate.MemberSamples.Num();
			BestContinentalSampleCount = ContinentalSampleCount;
			BestContinentalFraction = ContinentalFraction;
		}
	}
	if (OutContinentalSampleCount != nullptr)
	{
		*OutContinentalSampleCount = BestContinentalSampleCount;
	}
	if (OutContinentalFraction != nullptr)
	{
		*OutContinentalFraction = BestContinentalFraction;
	}
	return BestParentPlateId;
}

bool FTectonicPlanet::TryTriggerAutomaticRift()
{
	if (!bEnableAutomaticRifting || Plates.IsEmpty())
	{
		return false;
	}

	int32 ContinentalSampleCount = 0;
	double ContinentalFraction = 0.0;
	const int32 ParentPlateId =
		FindLargestEligibleAutomaticRiftParentId(&ContinentalSampleCount, &ContinentalFraction);
	if (ParentPlateId == INDEX_NONE)
	{
		return false;
	}

	const FPlate* ParentPlate = FindPlateById(*this, ParentPlateId);
	if (ParentPlate == nullptr)
	{
		return false;
	}

	const int32 ChildCount = 2; // M6b stabilization: automatic rifts stay binary to suppress fragmentation cascades.
	const double Probability =
		ComputeAutomaticRiftProbabilityForSampleCount(ParentPlate->MemberSamples.Num(), ContinentalFraction);
	const int32 EventSeed = MakeDeterministicSeed(SimulationSeed, CurrentStep, MakeDeterministicSeed(ParentPlateId, ChildCount, 991));
	FRandomStream TriggerRandom(MakeDeterministicSeed(SimulationSeed, CurrentStep, MakeDeterministicSeed(ParentPlateId, ChildCount, 701)));
	const double Draw = TriggerRandom.FRand();
	const bool bTriggered = Draw < Probability;

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[AutoRiftCheck Step=%d] parent=%d child_count=%d parent_samples=%d parent_continental_samples=%d parent_continental_fraction=%.4f probability=%.6f draw=%.6f triggered=%d event_seed=%d"),
		CurrentStep,
		ParentPlateId,
		ChildCount,
		ParentPlate->MemberSamples.Num(),
		ContinentalSampleCount,
		ContinentalFraction,
		Probability,
		Draw,
		bTriggered ? 1 : 0,
		EventSeed);
	if (!bTriggered)
	{
		return false;
	}

	return TriggerForcedRiftInternal(
		ParentPlateId,
		ChildCount,
		EventSeed,
		true,
		Probability,
		Draw,
		ContinentalSampleCount,
		ContinentalFraction);
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
	return TriggerForcedRiftInternal(ParentPlateId, ChildCount, Seed, false, 0.0, 0.0, 0, 0.0);
}

bool FTectonicPlanet::TriggerForcedRiftInternal(
	const int32 ParentPlateId,
	const int32 ChildCount,
	const int32 Seed,
	const bool bAutomatic,
	const double TriggerProbability,
	const int32 ParentContinentalSampleCount,
	const double ParentContinentalFraction)
{
	return TriggerForcedRiftInternal(
		ParentPlateId,
		ChildCount,
		Seed,
		bAutomatic,
		TriggerProbability,
		0.0,
		ParentContinentalSampleCount,
		ParentContinentalFraction);
}

bool FTectonicPlanet::TriggerForcedRiftInternal(
	const int32 ParentPlateId,
	int32 ChildCount,
	const int32 Seed,
	const bool bAutomatic,
	const double TriggerProbability,
	const double TriggerDraw,
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
	PendingRiftEvent.ChildAnchorSampleIndices = CentroidSampleIndices;
	PendingRiftEvent.ChildSampleCounts = ChildSampleCounts;
	PendingRiftEvent.FormerParentSampleIndices = ParentMembers;
	PendingRiftEvent.FormerParentTerraneIds = MoveTemp(FormerParentTerraneIds);
	PendingRiftEvent.ParentContinentalFraction = EffectiveParentContinentalFraction;
	PendingRiftEvent.TriggerProbability = TriggerProbability;
	PendingRiftEvent.TriggerDraw = TriggerDraw;
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

	if (!bDeferRiftFollowupResamplingToV6)
	{
		PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::RiftFollowup);
	}
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
	const EResampleOwnershipMode OwnershipMode,
	TArray<uint8>* OutPreserveOwnershipCWRetainFlags,
	TArray<uint8>* OutPreserveOwnershipFallbackQueryFlags) const
{
	OutNewPlateIds.Init(INDEX_NONE, Samples.Num());
	OutContainingTriangles.Init(INDEX_NONE, Samples.Num());
	OutBarycentricCoords.Init(FVector3d(-1.0, -1.0, -1.0), Samples.Num());
	OutGapFlags.Init(0, Samples.Num());
	OutOverlapFlags.Init(0, Samples.Num());
	TArray<uint8> RecoveryFlags;
	TArray<uint8> RecoveryContinentalFlags;
	TArray<uint8> PreserveOwnershipPreviousOwnerHysteresisFlags;
	TArray<double> RecoveryDistances;
	RecoveryFlags.Init(0, Samples.Num());
	RecoveryContinentalFlags.Init(0, Samples.Num());
	PreserveOwnershipPreviousOwnerHysteresisFlags.Init(0, Samples.Num());
	RecoveryDistances.Init(-1.0, Samples.Num());
	if (OutOverlapPlateIds != nullptr)
	{
		OutOverlapPlateIds->SetNum(Samples.Num());
	}
	if (OutPreserveOwnershipCWRetainFlags != nullptr)
	{
		OutPreserveOwnershipCWRetainFlags->Init(0, Samples.Num());
	}
	if (OutPreserveOwnershipFallbackQueryFlags != nullptr)
	{
		OutPreserveOwnershipFallbackQueryFlags->Init(0, Samples.Num());
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
		OutStats->PreserveOwnershipCWRetainedCount = 0;
		OutStats->PreserveOwnershipCWThresholdCrossingPreventedCount = 0;
		OutStats->PreserveOwnershipPreviousOwnerHysteresisApplicationCount = 0;
		OutStats->PreserveOwnershipStronglyContinentalBoundarySavedCount = 0;
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
		[this, bAllRotationsIdentity, bUseStableOverlaps, bUsePreserveOwnership, &OutNewPlateIds, &OutContainingTriangles, &OutBarycentricCoords, &OutGapFlags, &OutOverlapFlags, OutOverlapPlateIds, OutPreserveOwnershipCWRetainFlags, OutPreserveOwnershipFallbackQueryFlags, &RecoveryFlags, &RecoveryContinentalFlags, &PreserveOwnershipPreviousOwnerHysteresisFlags, &RecoveryDistances, &HysteresisRetainedCount, &HysteresisReassignedCount, &PreserveOwnershipSamePlateHitCount, &PreserveOwnershipSamePlateRecoveryCount, &PreserveOwnershipFallbackQueryCount](const int32 SampleIndex)
	{
		const FVector3d QueryPoint = Samples[SampleIndex].Position;
		const int32 CurrentPlateId = Samples[SampleIndex].PlateId;

		const FPlate* CurrentPlatePtr = FindPlateById(*this, CurrentPlateId);
		if (bUsePreserveOwnership && CurrentPlatePtr == nullptr)
		{
			if (OutPreserveOwnershipFallbackQueryFlags != nullptr)
			{
				(*OutPreserveOwnershipFallbackQueryFlags)[SampleIndex] = 1;
			}
			++PreserveOwnershipFallbackQueryCount;
		}
		else if (bUsePreserveOwnership && CurrentPlatePtr != nullptr)
		{
			const FPlate& CurrentPlate = *CurrentPlatePtr;
			if (CurrentPlate.SoupData.LocalTriangles.IsEmpty())
			{
				if (OutPreserveOwnershipFallbackQueryFlags != nullptr)
				{
					(*OutPreserveOwnershipFallbackQueryFlags)[SampleIndex] = 1;
				}
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
					if (OutPreserveOwnershipCWRetainFlags != nullptr)
					{
						(*OutPreserveOwnershipCWRetainFlags)[SampleIndex] = 1;
					}
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
					if (OutPreserveOwnershipCWRetainFlags != nullptr)
					{
						(*OutPreserveOwnershipCWRetainFlags)[SampleIndex] = 1;
					}

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

				if (OutPreserveOwnershipFallbackQueryFlags != nullptr)
				{
					(*OutPreserveOwnershipFallbackQueryFlags)[SampleIndex] = 1;
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

		const bool bPreserveFallbackQueried =
			bUsePreserveOwnership &&
			OutPreserveOwnershipFallbackQueryFlags != nullptr &&
			(*OutPreserveOwnershipFallbackQueryFlags)[SampleIndex] != 0;
		if (bPreserveFallbackQueried &&
			MeaningfulContainingPlateCount <= 1 &&
			BestPlateId != INDEX_NONE &&
			BestPlateId != CurrentPlateId)
		{
			FRecoveredContainmentHit HysteresisRecoveryHit;
			if (TryResolvePreserveBoundaryPreviousOwnerHysteresis(
					*this,
					SampleIndex,
					CurrentPlateId,
					BestPlateId,
					HysteresisRecoveryHit))
			{
				OutNewPlateIds[SampleIndex] = CurrentPlateId;
				OutContainingTriangles[SampleIndex] = HysteresisRecoveryHit.GlobalTriangleIndex;
				OutBarycentricCoords[SampleIndex] = HysteresisRecoveryHit.Barycentric;
				if (OutPreserveOwnershipCWRetainFlags != nullptr)
				{
					(*OutPreserveOwnershipCWRetainFlags)[SampleIndex] = 1;
				}
				PreserveOwnershipPreviousOwnerHysteresisFlags[SampleIndex] = 1;
				return;
			}
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
	int32 PreserveOwnershipPreviousOwnerHysteresisApplications = 0;
	TArray<double> RecoveredDistances;
	RecoveredDistances.Reserve(Samples.Num());
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		OutGapCount += OutGapFlags[SampleIndex] != 0 ? 1 : 0;
		OutOverlapCount += OutOverlapFlags[SampleIndex] != 0 ? 1 : 0;
		RecoveryContainmentCount += RecoveryFlags[SampleIndex] != 0 ? 1 : 0;
		RecoveryContinentalCount += RecoveryContinentalFlags[SampleIndex] != 0 ? 1 : 0;
		PreserveOwnershipPreviousOwnerHysteresisApplications +=
			PreserveOwnershipPreviousOwnerHysteresisFlags[SampleIndex] != 0 ? 1 : 0;
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
	const int32 PreserveOwnershipStronglyContinentalBoundarySaves =
		PreserveOwnershipPreviousOwnerHysteresisApplications;
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
		OutStats->PreserveOwnershipPreviousOwnerHysteresisApplicationCount =
			PreserveOwnershipPreviousOwnerHysteresisApplications;
		OutStats->PreserveOwnershipStronglyContinentalBoundarySavedCount =
			PreserveOwnershipStronglyContinentalBoundarySaves;
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

void FTectonicPlanet::ApplyPreserveOwnershipContinentalWeightRetention(
	const TArray<uint8>& PreserveOwnershipCWRetainFlags,
	const TArray<float>& PreviousContinentalWeights,
	const EResampleOwnershipMode OwnershipMode,
	const EResampleTriggerReason TriggerReason,
	FResamplingStats* InOutStats)
{
	if (OwnershipMode != EResampleOwnershipMode::PreserveOwnership ||
		TriggerReason == EResampleTriggerReason::CollisionFollowup ||
		TriggerReason == EResampleTriggerReason::RiftFollowup)
	{
		return;
	}

	check(PreserveOwnershipCWRetainFlags.Num() == Samples.Num());
	check(PreviousContinentalWeights.Num() == Samples.Num());

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		if (PreserveOwnershipCWRetainFlags[SampleIndex] == 0)
		{
			continue;
		}

		const float PreviousContinentalWeight = FMath::Clamp(PreviousContinentalWeights[SampleIndex], 0.0f, 1.0f);
		const float InterpolatedContinentalWeight = Samples[SampleIndex].ContinentalWeight;
		if (InOutStats != nullptr)
		{
			++InOutStats->PreserveOwnershipCWRetainedCount;
			InOutStats->PreserveOwnershipCWThresholdCrossingPreventedCount +=
				(PreviousContinentalWeight >= 0.5f && InterpolatedContinentalWeight < 0.5f) ? 1 : 0;
		}

		Samples[SampleIndex].ContinentalWeight = PreviousContinentalWeight;
	}
}

void FTectonicPlanet::ApplyPreserveOwnershipFallbackSamePlateRetention(
	const TArray<int32>& NewPlateIds,
	const TArray<int32>& PreviousPlateAssignments,
	const TArray<float>& PreviousContinentalWeights,
	const TArray<uint8>& PreserveOwnershipFallbackQueryFlags,
	const TArray<EGapResolutionPath>& GapResolutionPaths,
	const EResampleOwnershipMode OwnershipMode,
	const EResampleTriggerReason TriggerReason,
	FResamplingStats* InOutStats)
{
	if (OwnershipMode != EResampleOwnershipMode::PreserveOwnership ||
		TriggerReason == EResampleTriggerReason::CollisionFollowup ||
		TriggerReason == EResampleTriggerReason::RiftFollowup)
	{
		return;
	}

	check(NewPlateIds.Num() == Samples.Num());
	check(PreviousPlateAssignments.Num() == Samples.Num());
	check(PreviousContinentalWeights.Num() == Samples.Num());
	check(PreserveOwnershipFallbackQueryFlags.Num() == Samples.Num());
	check(GapResolutionPaths.Num() == Samples.Num());

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		if (PreserveOwnershipFallbackQueryFlags[SampleIndex] == 0)
		{
			continue;
		}

		const int32 PreviousPlateId = PreviousPlateAssignments[SampleIndex];
		const int32 FinalPlateId = NewPlateIds[SampleIndex];
		const EGapResolutionPath GapResolutionPath = GapResolutionPaths[SampleIndex];
		const bool bSamePlate = FinalPlateId == PreviousPlateId;
		const bool bGapResolved = GapResolutionPath != EGapResolutionPath::None;
		const bool bDivergentOceanized =
			GapResolutionPath == EGapResolutionPath::DivergentOceanized;

		if (InOutStats != nullptr)
		{
			InOutStats->PreserveOwnershipFallbackSamePlateRecontainedCount +=
				(bSamePlate && GapResolutionPath == EGapResolutionPath::None) ? 1 : 0;
			InOutStats->PreserveOwnershipFallbackChangedOwnerCount +=
				bSamePlate ? 0 : 1;
			InOutStats->PreserveOwnershipFallbackGapCount += bGapResolved ? 1 : 0;
			InOutStats->PreserveOwnershipFallbackDivergentOceanizationCount +=
				bDivergentOceanized ? 1 : 0;
		}

		const bool bEligibleSamePlateRetention =
			bSamePlate &&
			GapResolutionPath != EGapResolutionPath::NonDivergentFallbackOceanized &&
			GapResolutionPath != EGapResolutionPath::DivergentOceanized;
		if (!bEligibleSamePlateRetention)
		{
			continue;
		}

		const float PreviousContinentalWeight =
			FMath::Clamp(PreviousContinentalWeights[SampleIndex], 0.0f, 1.0f);
		const float InterpolatedContinentalWeight = Samples[SampleIndex].ContinentalWeight;
		Samples[SampleIndex].ContinentalWeight = PreviousContinentalWeight;

		if (InOutStats != nullptr)
		{
			++InOutStats->PreserveOwnershipFallbackSamePlateRetainedCount;
			++InOutStats->PreserveOwnershipCWRetainedCount;
			InOutStats->PreserveOwnershipCWThresholdCrossingPreventedCount +=
				(PreviousContinentalWeight >= 0.5f && InterpolatedContinentalWeight < 0.5f) ? 1 : 0;
		}
	}

	if (InOutStats == nullptr)
	{
		return;
	}

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		if (PreserveOwnershipFallbackQueryFlags[SampleIndex] == 0)
		{
			continue;
		}

		const float PreviousContinentalWeight =
			FMath::Clamp(PreviousContinentalWeights[SampleIndex], 0.0f, 1.0f);
		const bool bContinentalLossAfterFallback =
			PreviousContinentalWeight >= 0.5f &&
			Samples[SampleIndex].ContinentalWeight < 0.5f;
		InOutStats->PreserveOwnershipContinentalLossCountAfterFallback +=
			bContinentalLossAfterFallback ? 1 : 0;
	}
}

void FTectonicPlanet::ApplyFullResolutionSamePlateContinentalWeightRetention(
	const TArray<int32>& NewPlateIds,
	const TArray<int32>& PreviousPlateAssignments,
	const TArray<float>& PreviousContinentalWeights,
	const TArray<EGapResolutionPath>& GapResolutionPaths,
	const EResampleOwnershipMode OwnershipMode,
	const EResampleTriggerReason TriggerReason,
	FResamplingStats* InOutStats)
{
	if (OwnershipMode != EResampleOwnershipMode::FullResolution)
	{
		return;
	}

	check(NewPlateIds.Num() == Samples.Num());
	check(PreviousPlateAssignments.Num() == Samples.Num());
	check(PreviousContinentalWeights.Num() == Samples.Num());
	check(GapResolutionPaths.Num() == Samples.Num());

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		if (GapResolutionPaths[SampleIndex] != EGapResolutionPath::None)
		{
			continue;
		}

		if (NewPlateIds[SampleIndex] != PreviousPlateAssignments[SampleIndex])
		{
			continue;
		}

		const float PreviousContinentalWeight =
			FMath::Clamp(PreviousContinentalWeights[SampleIndex], 0.0f, 1.0f);
		if (PreviousContinentalWeight < 0.5f)
		{
			continue;
		}

		const float InterpolatedContinentalWeight = Samples[SampleIndex].ContinentalWeight;
		if (InterpolatedContinentalWeight >= 0.5f)
		{
			continue;
		}

		if (InOutStats != nullptr)
		{
			++InOutStats->FullResolutionSamePlateCWRetainedCount;
			++InOutStats->FullResolutionSamePlateCWThresholdCrossingPreventedCount;
			switch (TriggerReason)
			{
			case EResampleTriggerReason::CollisionFollowup:
				++InOutStats->FullResolutionCollisionFollowupSamePlateCWRetainedCount;
				break;
			case EResampleTriggerReason::RiftFollowup:
				++InOutStats->FullResolutionRiftFollowupSamePlateCWRetainedCount;
				break;
			default:
				++InOutStats->FullResolutionOtherTriggerSamePlateCWRetainedCount;
				break;
			}
		}

		Samples[SampleIndex].ContinentalWeight = PreviousContinentalWeight;
	}
}

void FTectonicPlanet::ApplyRiftFollowupLocalizationOverride(
	TArray<int32>& InOutNewPlateIds,
	const TArray<int32>& PreviousPlateAssignments,
	const TArray<float>& PreviousContinentalWeights,
	TArray<uint8>* InOutGapFlags,
	TArray<uint8>* InOutOverlapFlags,
	TArray<TArray<int32>>* InOutOverlapPlateIds,
	TArray<float>* InOutSubductionDistances,
	TArray<float>* InOutSubductionSpeeds,
	TArray<EGapResolutionPath>* InOutGapResolutionPaths,
	TArray<uint8>* OutLocalizedRestoreFlags,
	FResamplingStats* InOutStats)
{
	if (!PendingRiftEvent.bValid)
	{
		return;
	}

	check(InOutNewPlateIds.Num() == Samples.Num());
	check(PreviousPlateAssignments.Num() == Samples.Num());
	check(PreviousContinentalWeights.Num() == Samples.Num());
	check(InOutGapFlags == nullptr || InOutGapFlags->Num() == Samples.Num());
	check(InOutOverlapFlags == nullptr || InOutOverlapFlags->Num() == Samples.Num());
	check(InOutOverlapPlateIds == nullptr || InOutOverlapPlateIds->Num() == Samples.Num());
	check(InOutSubductionDistances == nullptr || InOutSubductionDistances->Num() == Samples.Num());
	check(InOutSubductionSpeeds == nullptr || InOutSubductionSpeeds->Num() == Samples.Num());
	check(InOutGapResolutionPaths == nullptr || InOutGapResolutionPaths->Num() == Samples.Num());
	if (OutLocalizedRestoreFlags != nullptr)
	{
		OutLocalizedRestoreFlags->Init(0, Samples.Num());
	}

	TSet<int32> ChildPlateIdSet;
	for (const int32 ChildPlateId : PendingRiftEvent.ChildPlateIds)
	{
		if (ChildPlateId != INDEX_NONE)
		{
			ChildPlateIdSet.Add(ChildPlateId);
		}
	}

	TMap<int32, int32> RestoredCountByPreviousPlateId;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const int32 PreviousPlateId = PreviousPlateAssignments[SampleIndex];
		if (ChildPlateIdSet.Contains(PreviousPlateId))
		{
			continue;
		}

		const bool bGapSample =
			InOutGapFlags != nullptr &&
			InOutGapFlags->IsValidIndex(SampleIndex) &&
			(*InOutGapFlags)[SampleIndex] != 0;
		const bool bOverlapSample =
			InOutOverlapFlags != nullptr &&
			InOutOverlapFlags->IsValidIndex(SampleIndex) &&
			(*InOutOverlapFlags)[SampleIndex] != 0;
		const bool bHasOverlapPlateIds =
			InOutOverlapPlateIds != nullptr &&
			InOutOverlapPlateIds->IsValidIndex(SampleIndex) &&
			!(*InOutOverlapPlateIds)[SampleIndex].IsEmpty();
		const bool bNeedsRestore =
			InOutNewPlateIds[SampleIndex] != PreviousPlateId ||
			bGapSample ||
			bOverlapSample ||
			bHasOverlapPlateIds;
		if (!bNeedsRestore)
		{
			continue;
		}

		InOutNewPlateIds[SampleIndex] = PreviousPlateId;
		if (InOutGapFlags != nullptr && InOutGapFlags->IsValidIndex(SampleIndex))
		{
			(*InOutGapFlags)[SampleIndex] = 0;
		}
		if (InOutOverlapFlags != nullptr && InOutOverlapFlags->IsValidIndex(SampleIndex))
		{
			(*InOutOverlapFlags)[SampleIndex] = 0;
		}
		if (InOutOverlapPlateIds != nullptr && InOutOverlapPlateIds->IsValidIndex(SampleIndex))
		{
			(*InOutOverlapPlateIds)[SampleIndex].Reset();
		}
		float RestoredSubductionDistanceKm = Samples[SampleIndex].SubductionDistanceKm;
		float RestoredSubductionSpeed = 0.0f;
		if (const FCarriedSample* PreviousCarriedSample =
				FindCarriedSampleForCanonicalVertex(*this, PreviousPlateId, SampleIndex))
		{
			RestoredSubductionDistanceKm = PreviousCarriedSample->SubductionDistanceKm;
			RestoredSubductionSpeed = PreviousCarriedSample->SubductionSpeed;
		}
		if (InOutSubductionDistances != nullptr && InOutSubductionDistances->IsValidIndex(SampleIndex))
		{
			(*InOutSubductionDistances)[SampleIndex] = RestoredSubductionDistanceKm;
		}
		if (InOutSubductionSpeeds != nullptr && InOutSubductionSpeeds->IsValidIndex(SampleIndex))
		{
			(*InOutSubductionSpeeds)[SampleIndex] = RestoredSubductionSpeed;
		}
			if (InOutGapResolutionPaths != nullptr && InOutGapResolutionPaths->IsValidIndex(SampleIndex))
			{
				(*InOutGapResolutionPaths)[SampleIndex] = EGapResolutionPath::None;
			}
			const float PreviousContinentalWeight =
				FMath::Clamp(PreviousContinentalWeights[SampleIndex], 0.0f, 1.0f);
			const float InterpolatedContinentalWeight = Samples[SampleIndex].ContinentalWeight;
			Samples[SampleIndex].ContinentalWeight = PreviousContinentalWeight;
			if (OutLocalizedRestoreFlags != nullptr && OutLocalizedRestoreFlags->IsValidIndex(SampleIndex))
			{
				(*OutLocalizedRestoreFlags)[SampleIndex] = 1;
			}
			if (InOutStats != nullptr)
			{
				++InOutStats->RiftLocalizedNonChildSampleRestoreCount;
				InOutStats->RiftLocalizedGapSampleRestoreCount += bGapSample ? 1 : 0;
				InOutStats->RiftLocalizedCWRestoredCount +=
					!FMath::IsNearlyEqual(
						InterpolatedContinentalWeight,
						PreviousContinentalWeight,
						1.0e-6f)
						? 1
						: 0;
				const bool bInterpolatedContinental = InterpolatedContinentalWeight >= 0.5f;
				const bool bFinalRestoredContinental = PreviousContinentalWeight >= 0.5f;
				InOutStats->RiftLocalizedCWPhantomPreventedCount +=
					(!bFinalRestoredContinental && bInterpolatedContinental) ? 1 : 0;
				InOutStats->RiftLocalizedCWContinentalPreventedCount +=
					(!bFinalRestoredContinental && bInterpolatedContinental) ? 1 : 0;
			}
			++RestoredCountByPreviousPlateId.FindOrAdd(PreviousPlateId);
		}

	if (InOutStats != nullptr)
	{
		InOutStats->RiftLocalizedRestoredPreviousPlateIds.Reset();
		InOutStats->RiftLocalizedRestoredPreviousPlateCounts.Reset();
		TArray<int32> SortedPreviousPlateIds;
		RestoredCountByPreviousPlateId.GenerateKeyArray(SortedPreviousPlateIds);
		SortedPreviousPlateIds.Sort();
		for (const int32 PreviousPlateId : SortedPreviousPlateIds)
		{
			InOutStats->RiftLocalizedRestoredPreviousPlateIds.Add(PreviousPlateId);
			InOutStats->RiftLocalizedRestoredPreviousPlateCounts.Add(
				RestoredCountByPreviousPlateId.FindRef(PreviousPlateId));
		}
	}
}

void FTectonicPlanet::ApplyRiftFollowupFinalOwnerContinentalWeightReconciliation(
	const TArray<int32>& InterpolationPlateIds,
	const TArray<int32>& FinalPlateIds,
	const TArray<int32>& PreviousPlateAssignments,
	const TArray<float>& PreviousContinentalWeights,
	const TArray<uint8>& LocalizedRestoreFlags,
	FResamplingStats* InOutStats)
{
	if (!PendingRiftEvent.bValid)
	{
		return;
	}

	check(InterpolationPlateIds.Num() == Samples.Num());
	check(FinalPlateIds.Num() == Samples.Num());
	check(PreviousPlateAssignments.Num() == Samples.Num());
	check(PreviousContinentalWeights.Num() == Samples.Num());
	check(LocalizedRestoreFlags.Num() == Samples.Num());

	TSet<int32> ChildPlateIdSet;
	for (const int32 ChildPlateId : PendingRiftEvent.ChildPlateIds)
	{
		if (ChildPlateId != INDEX_NONE)
		{
			ChildPlateIdSet.Add(ChildPlateId);
		}
	}

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const float PreviousContinentalWeight =
			FMath::Clamp(PreviousContinentalWeights[SampleIndex], 0.0f, 1.0f);
		const float PreReconciliationContinentalWeight = Samples[SampleIndex].ContinentalWeight;
		const int32 InterpolationPlateId = InterpolationPlateIds[SampleIndex];
		const int32 FinalPlateId = FinalPlateIds[SampleIndex];
			const bool bInterpolatedGain =
				PreviousContinentalWeight < 0.5f &&
				PreReconciliationContinentalWeight >= 0.5f;
			const bool bOwnerMismatch = InterpolationPlateId != FinalPlateId;
			const bool bOwnerMismatchGain = bOwnerMismatch && bInterpolatedGain;
			const bool bSameOwnerChildInterpolationGain =
				!bOwnerMismatch &&
				ChildPlateIdSet.Contains(FinalPlateId) &&
				bInterpolatedGain;

			if (InOutStats != nullptr)
			{
				InOutStats->RiftFinalOwnerMismatchGainCountBeforeReconciliation +=
					bOwnerMismatchGain ? 1 : 0;
				InOutStats->RiftSameOwnerChildInterpolationGainCountBeforeReconciliation +=
					bSameOwnerChildInterpolationGain ? 1 : 0;
			}

		if (LocalizedRestoreFlags[SampleIndex] != 0)
		{
			continue;
		}

			if (!bOwnerMismatchGain && !bSameOwnerChildInterpolationGain)
			{
				continue;
			}

		float ReconciledContinentalWeight = PreReconciliationContinentalWeight;
		bool bResolved = false;
		if (FinalPlateId == PreviousPlateAssignments[SampleIndex])
		{
			ReconciledContinentalWeight = PreviousContinentalWeight;
			bResolved = true;
		}
		else if (TryComputeAuthoritativeContinentalWeightForPlate(
					*this,
					SampleIndex,
					FinalPlateId,
					ReconciledContinentalWeight))
		{
			bResolved = true;
		}

		if (!bResolved)
		{
			continue;
		}

		if (InOutStats != nullptr)
		{
			InOutStats->RiftFinalOwnerCWReconciledCount +=
				!FMath::IsNearlyEqual(
					PreReconciliationContinentalWeight,
					ReconciledContinentalWeight,
					1.0e-6f)
					? 1
					: 0;
				InOutStats->RiftFinalOwnerMismatchContinentalPreventedCount +=
					(bOwnerMismatchGain && ReconciledContinentalWeight < 0.5f)
						? 1
						: 0;
			}

		Samples[SampleIndex].ContinentalWeight =
			FMath::Clamp(ReconciledContinentalWeight, 0.0f, 1.0f);
	}

	if (InOutStats == nullptr)
	{
		return;
	}

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const float PreviousContinentalWeight =
			FMath::Clamp(PreviousContinentalWeights[SampleIndex], 0.0f, 1.0f);
		const bool bFinalGain =
			PreviousContinentalWeight < 0.5f &&
			Samples[SampleIndex].ContinentalWeight >= 0.5f;
		if (!bFinalGain)
		{
			continue;
		}

		const int32 InterpolationPlateId = InterpolationPlateIds[SampleIndex];
		const int32 FinalPlateId = FinalPlateIds[SampleIndex];
		InOutStats->RiftFinalOwnerMismatchGainCountAfterReconciliation +=
			(InterpolationPlateId != FinalPlateId) ? 1 : 0;
		InOutStats->RiftSameOwnerChildInterpolationGainCountAfterReconciliation +=
			(InterpolationPlateId == FinalPlateId &&
				ChildPlateIdSet.Contains(FinalPlateId))
				? 1
				: 0;
		InOutStats->RiftFinalGainStartedBelow025Count += PreviousContinentalWeight < 0.25f ? 1 : 0;
		InOutStats->RiftFinalGainStartedBelow040Count += PreviousContinentalWeight < 0.40f ? 1 : 0;
		InOutStats->RiftFinalGainStartedBelow050Count += PreviousContinentalWeight < 0.50f ? 1 : 0;
	}
}

void FTectonicPlanet::ApplyRiftChildCoherenceProtection(
	TArray<int32>& InOutNewPlateIds,
	const TArray<int32>& PreviousPlateAssignments,
	TArray<uint8>* InOutGapFlags,
	TArray<uint8>* InOutOverlapFlags,
	TArray<TArray<int32>>* InOutOverlapPlateIds,
	TArray<EGapResolutionPath>* InOutGapResolutionPaths,
	FResamplingStats* InOutStats) const
{
	if (!PendingRiftEvent.bValid)
	{
		return;
	}

	check(InOutNewPlateIds.Num() == Samples.Num());
	check(PreviousPlateAssignments.Num() == Samples.Num());
	check(InOutGapFlags == nullptr || InOutGapFlags->Num() == Samples.Num());
	check(InOutOverlapFlags == nullptr || InOutOverlapFlags->Num() == Samples.Num());
	check(InOutOverlapPlateIds == nullptr || InOutOverlapPlateIds->Num() == Samples.Num());
	check(InOutGapResolutionPaths == nullptr || InOutGapResolutionPaths->Num() == Samples.Num());

	if (InOutStats != nullptr)
	{
		InOutStats->RiftChildAnchorSampleIndices.Reset();
		InOutStats->RiftChildComponentCountsBeforeSuppression.Reset();
		InOutStats->RiftChildComponentCountsAfterSuppression.Reset();
		InOutStats->RiftChildStrayFragmentDetectedCount = 0;
		InOutStats->RiftChildStrayFragmentReassignedCount = 0;
		InOutStats->RiftLargestStrayChildFragmentSize = 0;
		InOutStats->RiftStrayChildFragmentReassignedToSiblingCount = 0;
		InOutStats->RiftStrayChildFragmentReassignedToOtherNeighborCount = 0;
		InOutStats->RiftStrayFragmentReassignedByAdjacentSiblingCount = 0;
		InOutStats->RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount = 0;
		InOutStats->RiftStrayFragmentForcedNonChildAssignmentCount = 0;
		InOutStats->RiftStrayFragmentZeroSiblingAdjacencyCount = 0;
		InOutStats->RiftStrayFragmentPositiveSiblingAdjacencyCount = 0;
		InOutStats->RiftStrayFragmentRecipientCandidateConsideredCount = 0;
		InOutStats->RiftStrayFragmentRecipientIncoherenceRejectedCount = 0;
		InOutStats->RiftStrayFragmentIncoherentForcedAssignmentCount = 0;
		InOutStats->RiftLargestFragmentCausingRecipientGrowthSize = 0;
		InOutStats->RiftStrayFragmentRecipientPlateIds.Reset();
		InOutStats->RiftStrayFragmentRecipientTypeCodes.Reset();
		InOutStats->RiftStrayFragmentSiblingEdgeCounts.Reset();
		InOutStats->RiftStrayFragmentSizes.Reset();
		InOutStats->RiftStrayFragmentRecipientComponentsBefore.Reset();
		InOutStats->RiftStrayFragmentRecipientComponentsAfter.Reset();
	}

	for (int32 ChildIndex = 0; ChildIndex < PendingRiftEvent.ChildPlateIds.Num(); ++ChildIndex)
	{
		const int32 ChildPlateId = PendingRiftEvent.ChildPlateIds[ChildIndex];
		if (ChildPlateId == INDEX_NONE)
		{
			if (InOutStats != nullptr)
			{
				InOutStats->RiftChildAnchorSampleIndices.Add(INDEX_NONE);
				InOutStats->RiftChildComponentCountsBeforeSuppression.Add(0);
				InOutStats->RiftChildComponentCountsAfterSuppression.Add(0);
			}
			continue;
		}

		int32 AnchorSampleIndex =
			PendingRiftEvent.ChildAnchorSampleIndices.IsValidIndex(ChildIndex)
				? PendingRiftEvent.ChildAnchorSampleIndices[ChildIndex]
				: INDEX_NONE;
		if (!Samples.IsValidIndex(AnchorSampleIndex))
		{
			AnchorSampleIndex = INDEX_NONE;
			for (int32 SampleIndex = 0; SampleIndex < PreviousPlateAssignments.Num(); ++SampleIndex)
			{
				if (PreviousPlateAssignments[SampleIndex] == ChildPlateId)
				{
					AnchorSampleIndex = SampleIndex;
					break;
				}
			}
		}

		if (Samples.IsValidIndex(AnchorSampleIndex))
		{
			InOutNewPlateIds[AnchorSampleIndex] = ChildPlateId;
			if (InOutGapFlags != nullptr && InOutGapFlags->IsValidIndex(AnchorSampleIndex))
			{
				(*InOutGapFlags)[AnchorSampleIndex] = 0;
			}
			if (InOutOverlapFlags != nullptr && InOutOverlapFlags->IsValidIndex(AnchorSampleIndex))
			{
				(*InOutOverlapFlags)[AnchorSampleIndex] = 0;
			}
			if (InOutOverlapPlateIds != nullptr && InOutOverlapPlateIds->IsValidIndex(AnchorSampleIndex))
			{
				(*InOutOverlapPlateIds)[AnchorSampleIndex].Reset();
			}
			if (InOutGapResolutionPaths != nullptr && InOutGapResolutionPaths->IsValidIndex(AnchorSampleIndex))
			{
				(*InOutGapResolutionPaths)[AnchorSampleIndex] = EGapResolutionPath::None;
			}
		}

		TArray<FRiftPlateAssignmentComponent> ChildComponents;
		CollectPlateAssignmentComponents(*this, InOutNewPlateIds, ChildPlateId, ChildComponents);
		if (InOutStats != nullptr)
		{
			InOutStats->RiftChildAnchorSampleIndices.Add(AnchorSampleIndex);
			InOutStats->RiftChildComponentCountsBeforeSuppression.Add(ChildComponents.Num());
		}

		int32 AnchorComponentIndex = INDEX_NONE;
		for (int32 ComponentIndex = 0; ComponentIndex < ChildComponents.Num(); ++ComponentIndex)
		{
			if (ChildComponents[ComponentIndex].SampleIndices.Contains(AnchorSampleIndex))
			{
				AnchorComponentIndex = ComponentIndex;
				break;
			}
		}
		if (AnchorComponentIndex == INDEX_NONE && !ChildComponents.IsEmpty())
		{
			AnchorComponentIndex = 0;
		}

		for (int32 ComponentIndex = 0; ComponentIndex < ChildComponents.Num(); ++ComponentIndex)
		{
			if (ComponentIndex == AnchorComponentIndex)
			{
				continue;
			}

			const FRiftPlateAssignmentComponent& Component = ChildComponents[ComponentIndex];
			if (InOutStats != nullptr)
			{
				++InOutStats->RiftChildStrayFragmentDetectedCount;
				InOutStats->RiftLargestStrayChildFragmentSize = FMath::Max(
					InOutStats->RiftLargestStrayChildFragmentSize,
					Component.SampleIndices.Num());
			}

			const FRiftFragmentDestinationSelectionResult DestinationSelection =
				SelectRecipientAwareRiftFragmentDestination(
					*this,
					InOutNewPlateIds,
					ChildPlateId,
					Component);
			const int32 DestinationPlateId = DestinationSelection.DestinationPlateId;
			if (DestinationPlateId == INDEX_NONE)
			{
				continue;
			}

			const int32 RecipientComponentsBefore =
				CountPlateAssignmentComponents(*this, InOutNewPlateIds, DestinationPlateId);

			for (const int32 SampleIndex : Component.SampleIndices)
			{
				if (!InOutNewPlateIds.IsValidIndex(SampleIndex))
				{
					continue;
				}

				InOutNewPlateIds[SampleIndex] = DestinationPlateId;
				if (InOutGapFlags != nullptr && InOutGapFlags->IsValidIndex(SampleIndex))
				{
					(*InOutGapFlags)[SampleIndex] = 0;
				}
				if (InOutOverlapFlags != nullptr && InOutOverlapFlags->IsValidIndex(SampleIndex))
				{
					(*InOutOverlapFlags)[SampleIndex] = 0;
				}
				if (InOutOverlapPlateIds != nullptr && InOutOverlapPlateIds->IsValidIndex(SampleIndex))
				{
					(*InOutOverlapPlateIds)[SampleIndex].Reset();
				}
				if (InOutGapResolutionPaths != nullptr && InOutGapResolutionPaths->IsValidIndex(SampleIndex))
				{
					(*InOutGapResolutionPaths)[SampleIndex] = EGapResolutionPath::None;
				}
			}

			const int32 RecipientComponentsAfter =
				CountPlateAssignmentComponents(*this, InOutNewPlateIds, DestinationPlateId);

			if (InOutStats != nullptr)
			{
				++InOutStats->RiftChildStrayFragmentReassignedCount;
				if (DestinationSelection.bHadPositiveSiblingAdjacency)
				{
					++InOutStats->RiftStrayFragmentPositiveSiblingAdjacencyCount;
				}
				else
				{
					++InOutStats->RiftStrayFragmentZeroSiblingAdjacencyCount;
				}
				if (DestinationSelection.bUsedSiblingChild &&
					!DestinationSelection.bUsedZeroAdjacencySiblingFallback)
				{
					++InOutStats->RiftStrayFragmentReassignedByAdjacentSiblingCount;
				}
				if (DestinationSelection.bUsedZeroAdjacencySiblingFallback)
				{
					++InOutStats->RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount;
				}
				if (DestinationSelection.bUsedNonChildFallback)
				{
					++InOutStats->RiftStrayFragmentForcedNonChildAssignmentCount;
				}
				InOutStats->RiftStrayFragmentRecipientPlateIds.Add(DestinationPlateId);
				InOutStats->RiftStrayFragmentRecipientTypeCodes.Add(
					DestinationSelection.bUsedSiblingChild
						? static_cast<int32>(ERiftRecipientTypeCode::SiblingChild)
						: static_cast<int32>(ERiftRecipientTypeCode::OtherNeighbor));
				InOutStats->RiftStrayFragmentSiblingEdgeCounts.Add(
					DestinationSelection.BestSiblingEdgeCount);
				InOutStats->RiftStrayFragmentSizes.Add(Component.SampleIndices.Num());
				InOutStats->RiftStrayFragmentRecipientComponentsBefore.Add(RecipientComponentsBefore);
				InOutStats->RiftStrayFragmentRecipientComponentsAfter.Add(RecipientComponentsAfter);
				if (RecipientComponentsAfter > RecipientComponentsBefore)
				{
					InOutStats->RiftLargestFragmentCausingRecipientGrowthSize = FMath::Max(
						InOutStats->RiftLargestFragmentCausingRecipientGrowthSize,
						Component.SampleIndices.Num());
				}
				if (DestinationSelection.bUsedSiblingChild)
				{
					++InOutStats->RiftStrayChildFragmentReassignedToSiblingCount;
				}
				if (DestinationSelection.bUsedNonChildFallback)
				{
					++InOutStats->RiftStrayChildFragmentReassignedToOtherNeighborCount;
				}
			}
		}

		if (InOutStats != nullptr)
		{
			InOutStats->RiftChildComponentCountsAfterSuppression.Add(
				CountPlateAssignmentComponents(*this, InOutNewPlateIds, ChildPlateId));
		}
	}
}

void FTectonicPlanet::ResolveGaps(
	TArray<int32>& NewPlateIds,
	const TArray<uint8>& GapFlags,
	TArray<float>& InOutSubductionDistances,
	TArray<float>& InOutSubductionSpeeds,
	FResamplingStats* InOutStats,
	TArray<EGapResolutionPath>* OutGapResolutionPaths)
{
	check(NewPlateIds.Num() == Samples.Num());
	check(GapFlags.Num() == Samples.Num());
	check(InOutSubductionDistances.Num() == Samples.Num());
	check(InOutSubductionSpeeds.Num() == Samples.Num());

	if (OutGapResolutionPaths != nullptr)
	{
		OutGapResolutionPaths->Init(EGapResolutionPath::None, Samples.Num());
	}

	if (InOutStats != nullptr)
	{
		InOutStats->DivergentGapCount = 0;
		InOutStats->NonDivergentGapCount = 0;
		InOutStats->DivergentContinentalGapCount = 0;
		InOutStats->NonDivergentContinentalGapCount = 0;
		InOutStats->FormerContinentalDivergentGapCount = 0;
		InOutStats->FormerContinentalNonDivergentGapCount = 0;
		InOutStats->FormerContinentalNonDivergentGapProjectionResolvedCount = 0;
		InOutStats->FormerContinentalNonDivergentGapNearestCopyResolvedCount = 0;
		InOutStats->FormerContinentalNonDivergentFallbackOceanizedCount = 0;
		InOutStats->FormerContinentalDivergentOceanizedCount = 0;
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
		if (InOutStats != nullptr && Decision.bWasContinental)
		{
			if (Decision.Disposition == EGapDisposition::NonDivergent)
			{
				++InOutStats->FormerContinentalNonDivergentGapCount;
			}
			else if (Decision.Disposition == EGapDisposition::Divergent)
			{
				++InOutStats->FormerContinentalDivergentGapCount;
			}
		}

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
					if (OutGapResolutionPaths != nullptr)
					{
						(*OutGapResolutionPaths)[SampleIndex] = EGapResolutionPath::NonDivergentProjection;
					}
					if (InOutStats != nullptr)
					{
						++InOutStats->NonDivergentGapCount;
						InOutStats->NonDivergentContinentalGapCount += Decision.bWasContinental ? 1 : 0;
						++InOutStats->NonDivergentGapTriangleProjectionCount;
						InOutStats->FormerContinentalNonDivergentGapProjectionResolvedCount += Decision.bWasContinental ? 1 : 0;
					}
					break;
				}

				if (TryApplyNearestCarriedCopyFromPlate(*this, SampleIndex, CandidatePlateId, InOutSubductionDistances, InOutSubductionSpeeds))
				{
					NewPlateIds[SampleIndex] = CandidatePlateId;
					bResolved = true;
					if (OutGapResolutionPaths != nullptr)
					{
						(*OutGapResolutionPaths)[SampleIndex] = EGapResolutionPath::NonDivergentNearestCopy;
					}
					if (InOutStats != nullptr)
					{
						++InOutStats->NonDivergentGapCount;
						InOutStats->NonDivergentContinentalGapCount += Decision.bWasContinental ? 1 : 0;
						++InOutStats->NonDivergentGapNearestCopyCount;
						InOutStats->FormerContinentalNonDivergentGapNearestCopyResolvedCount += Decision.bWasContinental ? 1 : 0;
					}
					break;
				}
			}

			if (bResolved)
			{
				continue;
			}

			++NonDivergentFallbackOceanizedCount;
			if (OutGapResolutionPaths != nullptr)
			{
				(*OutGapResolutionPaths)[SampleIndex] = EGapResolutionPath::NonDivergentFallbackOceanized;
			}
			if (InOutStats != nullptr && Decision.bWasContinental)
			{
				++InOutStats->FormerContinentalNonDivergentFallbackOceanizedCount;
			}
		}
		else if (OutGapResolutionPaths != nullptr)
		{
			(*OutGapResolutionPaths)[SampleIndex] = EGapResolutionPath::DivergentOceanized;
		}

		ApplyDivergentGap(SampleIndex);
		if (InOutStats != nullptr)
		{
			++InOutStats->DivergentGapCount;
			InOutStats->DivergentContinentalGapCount += Decision.bWasContinental ? 1 : 0;
			InOutStats->FormerContinentalDivergentOceanizedCount +=
				(Decision.bWasContinental && Decision.Disposition == EGapDisposition::Divergent) ? 1 : 0;
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
	PruneEmptyPlates(*this);
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

bool FTectonicPlanet::DetectGeometricCollisionOverlapDiagnostics(
	const TArray<int32>& OwningPlateIds,
	FResamplingStats* InOutStats,
	FPendingGeometricCollisionEvent* OutPendingEvent)
{
	if (OutPendingEvent != nullptr)
	{
		*OutPendingEvent = FPendingGeometricCollisionEvent{};
	}
	if (InOutStats != nullptr)
	{
		InOutStats->GeometricCollisionCandidateCount = 0;
		InOutStats->GeometricCollisionQualifiedCount = 0;
		InOutStats->GeometricCollisionQualifiedButDonorAmbiguousCount = 0;
		InOutStats->GeometricCollisionQualifiedButDonorSeedEmptyCount = 0;
		InOutStats->GeometricCollisionQualifiedUsingDirectionalDonorCount = 0;
		InOutStats->GeometricCollisionQualifiedUsingFallbackDonorRuleCount = 0;
		InOutStats->GeometricCollisionRejectedByMassFilterCount = 0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthCount = 0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationCount = 0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthTotalOverlapSampleCount = 0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthMaxOverlapSampleCount = 0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthTotalTerraneEstimate = 0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthMaxTerraneEstimate = 0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalOverlapSampleCount = 0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxOverlapSampleCount = 0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalTerraneEstimate = 0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxTerraneEstimate = 0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalObservationCount = 0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxObservationCount = 0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthTotalMeanConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalMeanConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalAccumulatedPenetrationKm = 0.0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxAccumulatedPenetrationKm = 0.0;
		InOutStats->GeometricCollisionRejectedByEmptyTerraneCount = 0;
		InOutStats->GeometricCollisionRejectedByRoleResolutionCount = 0;
		InOutStats->GeometricCollisionRejectedBySeedDirectionCount = 0;
		InOutStats->GeometricCollisionQualifiedDirectionalCount = 0;
		InOutStats->GeometricCollisionDirectionalSeedCount = 0;
		InOutStats->GeometricCollisionDirectionalSeedCountOpposite = 0;
		InOutStats->GeometricCollisionSeedDirectionAuditCount = 0;
		InOutStats->GeometricCollisionSeedDirectionMismatchCount = 0;
		InOutStats->GeometricCollisionOnlyOverridingBucketPopulatedCount = 0;
		InOutStats->GeometricCollisionOnlySubductingBucketPopulatedCount = 0;
		InOutStats->GeometricCollisionBothDirectionalBucketsPopulatedCount = 0;
		InOutStats->GeometricCollisionNeitherDirectionalBucketPopulatedCount = 0;
		InOutStats->GeometricCollisionPolarityChosenFromDirectionalEvidenceCount = 0;
		InOutStats->GeometricCollisionPolarityChosenFromBidirectionalTieBreakCount = 0;
		InOutStats->GeometricCollisionFallbackUsedCount = 0;
		InOutStats->GeometricCollisionOverlapSampleCount = 0;
		InOutStats->GeometricCollisionPairCount = 0;
		InOutStats->GeometricCollisionQualifiedPairCount = 0;
		InOutStats->GeometricCollisionLargestTerraneEstimate = 0;
		InOutStats->GeometricCollisionBestPlateA = INDEX_NONE;
		InOutStats->GeometricCollisionBestPlateB = INDEX_NONE;
		InOutStats->GeometricCollisionBestOverlapSampleCount = 0;
		InOutStats->GeometricCollisionBestTerraneEstimate = 0;
		InOutStats->GeometricCollisionBestSubductingOverlapSampleCount = 0;
		InOutStats->GeometricCollisionBestOpposingSupportCount = 0;
		InOutStats->GeometricCollisionExecutedTerraneEstimate = 0;
		InOutStats->GeometricCollisionExecutedMeanConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionExecutedMaxConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionExecutedAccumulatedPenetrationKm = 0.0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthTotalDepthKm = 0.0;
		InOutStats->GeometricCollisionRejectedByOverlapDepthMaxDepthKm = 0.0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalDepthKm = 0.0;
		InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxDepthKm = 0.0;
		InOutStats->GeometricCollisionExecutedObservationCount = 0;
		InOutStats->GeometricCollisionRepeatedCandidatePlateA = INDEX_NONE;
		InOutStats->GeometricCollisionRepeatedCandidatePlateB = INDEX_NONE;
		InOutStats->GeometricCollisionRepeatedCandidateObservationCount = 0;
		InOutStats->GeometricCollisionRepeatedCandidateOverlapSampleCount = 0;
		InOutStats->GeometricCollisionRepeatedCandidateTerraneEstimate = 0;
		InOutStats->GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm = 0.0;
		InOutStats->GeometricCollisionRepeatedCandidateEffectiveConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionRepeatedCandidateMeanConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionRepeatedCandidateMaxConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionRepeatedCandidateMeanOverlapDepthKm = 0.0;
		InOutStats->GeometricCollisionRepeatedCandidateMaxOverlapDepthKm = 0.0;
		InOutStats->GeometricCollisionPersistentPenetrationThresholdKm =
			GeometricCollisionMinPersistentPenetrationKm;
		InOutStats->bGeometricCollisionBestPassedMassFilter = false;
	}

	if (!bEnableContinentalCollision ||
		OwningPlateIds.Num() != Samples.Num() ||
		Plates.IsEmpty())
	{
		return false;
	}

	TMap<uint64, FGeometricCollisionPairDiagnostics> PairDiagnosticsByKey;
	int32 TotalOverlapHits = 0;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		if (!Samples.IsValidIndex(SampleIndex) || Samples[SampleIndex].ContinentalWeight < 0.5f)
		{
			continue;
		}

		const int32 OwningPlateId = OwningPlateIds[SampleIndex];
		if (OwningPlateId == INDEX_NONE || FindPlateById(*this, OwningPlateId) == nullptr)
		{
			continue;
		}

		const FVector3d QueryPoint = Samples[SampleIndex].Position.GetSafeNormal();
		for (const FPlate& CandidatePlate : Plates)
		{
			if (CandidatePlate.Id == OwningPlateId ||
				!TryFindMeaningfulContainmentHitInPlate(*this, CandidatePlate, QueryPoint))
			{
				continue;
			}

			++TotalOverlapHits;

			const int32 PlateA = FMath::Min(OwningPlateId, CandidatePlate.Id);
			const int32 PlateB = FMath::Max(OwningPlateId, CandidatePlate.Id);
			const uint64 PairKey = MakeBoundaryContactPlatePairKey(PlateA, PlateB);
			FGeometricCollisionPairDiagnostics& PairDiagnostics =
				PairDiagnosticsByKey.FindOrAdd(PairKey);
			PairDiagnostics.PlateA = PlateA;
			PairDiagnostics.PlateB = PlateB;
			++PairDiagnostics.TotalOverlapSampleCount;
			PairDiagnostics.MinSampleIndex =
				PairDiagnostics.MinSampleIndex == INDEX_NONE
					? SampleIndex
					: FMath::Min(PairDiagnostics.MinSampleIndex, SampleIndex);

			const double ConvergenceMagnitudeKmPerMy =
				ComputeGeometricOverlapConvergenceMagnitudeKmPerMy(
					*this,
					OwningPlateId,
					CandidatePlate.Id,
					QueryPoint);
			PairDiagnostics.TotalConvergenceMagnitudeKmPerMy += ConvergenceMagnitudeKmPerMy;
			PairDiagnostics.MaxConvergenceMagnitudeKmPerMy = FMath::Max(
				PairDiagnostics.MaxConvergenceMagnitudeKmPerMy,
				ConvergenceMagnitudeKmPerMy);

			if (OwningPlateId == PlateA)
			{
				PairDiagnostics.SamplesFromAInsideB.AddUnique(SampleIndex);
				PairDiagnostics.TotalConvergenceMagnitudeAInsideBKmPerMy += ConvergenceMagnitudeKmPerMy;
			}
			else
			{
				PairDiagnostics.SamplesFromBInsideA.AddUnique(SampleIndex);
				PairDiagnostics.TotalConvergenceMagnitudeBInsideAKmPerMy += ConvergenceMagnitudeKmPerMy;
			}
		}
	}

	if (InOutStats != nullptr)
	{
		InOutStats->GeometricCollisionCandidateCount = PairDiagnosticsByKey.Num();
		InOutStats->GeometricCollisionOverlapSampleCount = TotalOverlapHits;
		InOutStats->GeometricCollisionPairCount = PairDiagnosticsByKey.Num();
	}

	if (PairDiagnosticsByKey.IsEmpty())
	{
		GeometricCollisionPairRecurrenceByKey.Reset();
		return false;
	}

	bool bHasBestPair = false;
	FGeometricCollisionPairDiagnostics BestPair;
	bool bHasBestExecutablePair = false;
	FGeometricCollisionPairDiagnostics BestExecutablePair;
	bool bHasBestRepeatedPair = false;
	FGeometricCollisionPairDiagnostics BestRepeatedPair;
	FGeometricCollisionPairRecurrenceState BestRepeatedPairState;
	TSet<uint64> ObservedPersistentPenetrationPairKeys;
	for (TPair<uint64, FGeometricCollisionPairDiagnostics>& PairEntry : PairDiagnosticsByKey)
	{
		FGeometricCollisionPairDiagnostics& PairDiagnostics = PairEntry.Value;
		const bool bLegacyPlateAOverriding = IsOverridingPlate(*this, PairDiagnostics.PlateA, PairDiagnostics.PlateB);
		const int32 LegacyOverridingPlateId = bLegacyPlateAOverriding ? PairDiagnostics.PlateA : PairDiagnostics.PlateB;
		const int32 LegacySubductingPlateId = bLegacyPlateAOverriding ? PairDiagnostics.PlateB : PairDiagnostics.PlateA;
		const TArray<int32>* LegacySubductingOverlapSeeds = nullptr;
		const TArray<int32>* LegacyOppositeDirectionalSeeds = nullptr;
		verify(AssignGeometricCollisionDirectionalBucketsForRoles(
			PairDiagnostics.PlateA,
			PairDiagnostics.PlateB,
			PairDiagnostics.SamplesFromAInsideB,
			PairDiagnostics.SamplesFromBInsideA,
			LegacyOverridingPlateId,
			LegacySubductingPlateId,
			LegacySubductingOverlapSeeds,
			LegacyOppositeDirectionalSeeds));

		const int32 LegacySubductingSeedCount = LegacySubductingOverlapSeeds != nullptr ? LegacySubductingOverlapSeeds->Num() : 0;
		const int32 OppositeDirectionalSeedCount = LegacyOppositeDirectionalSeeds != nullptr ? LegacyOppositeDirectionalSeeds->Num() : 0;
		const EGeometricDirectionalBucketPopulation BucketPopulation =
			ClassifyGeometricDirectionalBucketPopulation(
				LegacySubductingSeedCount,
				OppositeDirectionalSeedCount);
		if (InOutStats != nullptr)
		{
			++InOutStats->GeometricCollisionSeedDirectionAuditCount;
			switch (BucketPopulation)
			{
			case EGeometricDirectionalBucketPopulation::OnlyOverriding:
				++InOutStats->GeometricCollisionOnlyOverridingBucketPopulatedCount;
				++InOutStats->GeometricCollisionSeedDirectionMismatchCount;
				break;
			case EGeometricDirectionalBucketPopulation::OnlySubducting:
				++InOutStats->GeometricCollisionOnlySubductingBucketPopulatedCount;
				break;
			case EGeometricDirectionalBucketPopulation::Both:
				++InOutStats->GeometricCollisionBothDirectionalBucketsPopulatedCount;
				break;
			case EGeometricDirectionalBucketPopulation::Neither:
			default:
				++InOutStats->GeometricCollisionNeitherDirectionalBucketPopulatedCount;
				break;
			}
		}
		if (LegacySubductingSeedCount <= 0 && OppositeDirectionalSeedCount > 0)
		{
			const FPlate* OverridingPlate = FindPlateById(*this, LegacyOverridingPlateId);
			const FPlate* SubductingPlate = FindPlateById(*this, LegacySubductingPlateId);
			UE_LOG(
				LogTemp,
				Log,
				TEXT("[GeometricCollisionSeedDirectionAudit Step=%d] plate_pair=(%d,%d) overriding_plate=%d subducting_plate=%d a_inside_b=%d b_inside_a=%d chosen_subducting_bucket_count=%d opposite_overriding_bucket_count=%d populated_bucket=%s overriding_overlap_score=%d subducting_overlap_score=%d total_convergence_km_per_my=%.6f max_convergence_km_per_my=%.6f"),
				CurrentStep,
				PairDiagnostics.PlateA,
				PairDiagnostics.PlateB,
				LegacyOverridingPlateId,
				LegacySubductingPlateId,
				PairDiagnostics.SamplesFromAInsideB.Num(),
				PairDiagnostics.SamplesFromBInsideA.Num(),
				LegacySubductingSeedCount,
				OppositeDirectionalSeedCount,
				GetGeometricDirectionalBucketPopulationName(BucketPopulation),
				OverridingPlate != nullptr ? OverridingPlate->OverlapScore : 0,
				SubductingPlate != nullptr ? SubductingPlate->OverlapScore : 0,
				PairDiagnostics.TotalConvergenceMagnitudeKmPerMy,
				PairDiagnostics.MaxConvergenceMagnitudeKmPerMy);
		}

		TSet<int32> OpposingSupportSamples;
		const auto AccumulateOpposingSupport = [this, &OwningPlateIds, &OpposingSupportSamples](
			const TArray<int32>& OverlapSeeds,
			const int32 ReceiverPlateId)
		{
			for (const int32 SeedSampleIndex : OverlapSeeds)
			{
				if (!SampleAdjacency.IsValidIndex(SeedSampleIndex))
				{
					continue;
				}

				for (const int32 NeighborSampleIndex : SampleAdjacency[SeedSampleIndex])
				{
					if (!Samples.IsValidIndex(NeighborSampleIndex) ||
						Samples[NeighborSampleIndex].ContinentalWeight < 0.5f ||
						!OwningPlateIds.IsValidIndex(NeighborSampleIndex))
					{
						continue;
					}

					if (OwningPlateIds[NeighborSampleIndex] == ReceiverPlateId)
					{
						OpposingSupportSamples.Add(NeighborSampleIndex);
					}
				}
			}
		};
		AccumulateOpposingSupport(PairDiagnostics.SamplesFromAInsideB, PairDiagnostics.PlateB);
		AccumulateOpposingSupport(PairDiagnostics.SamplesFromBInsideA, PairDiagnostics.PlateA);

		PairDiagnostics.OpposingContinentalSupportCount = OpposingSupportSamples.Num();
		PairDiagnostics.bPassedMassFilter =
			PairDiagnostics.TotalOverlapSampleCount >= MinGeometricCollisionOverlapContinentalSamples &&
			PairDiagnostics.OpposingContinentalSupportCount >= MinGeometricCollisionOpposingContinentalSupport;
		if (!PairDiagnostics.bPassedMassFilter && PairDiagnostics.TotalOverlapSampleCount > 0 && InOutStats != nullptr)
		{
			++InOutStats->GeometricCollisionRejectedByMassFilterCount;
		}

		if (PairDiagnostics.bPassedMassFilter)
		{
			if (InOutStats != nullptr)
			{
				++InOutStats->GeometricCollisionQualifiedCount;
				++InOutStats->GeometricCollisionQualifiedPairCount;
			}

			int32 ReceiverPlateId = INDEX_NONE;
			int32 DonorPlateId = INDEX_NONE;
			const TArray<int32>* DonorDirectionalSeeds = nullptr;
			const TArray<int32>* OppositeDirectionalSeeds = nullptr;
			EGeometricCollisionDonorSelectionRule DonorRule = EGeometricCollisionDonorSelectionRule::None;
			if (!ResolveGeometricCollisionDonorReceiver(
					*this,
					PairDiagnostics,
					ReceiverPlateId,
					DonorPlateId,
					DonorDirectionalSeeds,
					OppositeDirectionalSeeds,
					DonorRule))
			{
				if (InOutStats != nullptr)
				{
					++InOutStats->GeometricCollisionQualifiedButDonorAmbiguousCount;
				}
			}
			else
			{
				PairDiagnostics.SubductingOverlapSampleCount = DonorDirectionalSeeds != nullptr ? DonorDirectionalSeeds->Num() : 0;
				if (InOutStats != nullptr)
				{
					if (IsDirectionalGeometricCollisionDonorSelectionRule(DonorRule))
					{
						++InOutStats->GeometricCollisionQualifiedUsingDirectionalDonorCount;
						++InOutStats->GeometricCollisionQualifiedDirectionalCount;
						++InOutStats->GeometricCollisionPolarityChosenFromDirectionalEvidenceCount;
					}
					else
					{
						++InOutStats->GeometricCollisionQualifiedUsingFallbackDonorRuleCount;
						++InOutStats->GeometricCollisionPolarityChosenFromBidirectionalTieBreakCount;
					}
				}

				if (DonorDirectionalSeeds == nullptr || DonorDirectionalSeeds->IsEmpty())
				{
					if (InOutStats != nullptr)
					{
						++InOutStats->GeometricCollisionQualifiedButDonorSeedEmptyCount;
					}
				}
				else
				{
					PairDiagnostics.TerraneEstimate = EstimateGeometricCollisionTerraneSize(
						*this,
						OwningPlateIds,
						DonorPlateId,
						*DonorDirectionalSeeds);
					if (InOutStats != nullptr)
					{
						InOutStats->GeometricCollisionLargestTerraneEstimate = FMath::Max(
							InOutStats->GeometricCollisionLargestTerraneEstimate,
							PairDiagnostics.TerraneEstimate);
						if (PairDiagnostics.TerraneEstimate <= 0)
						{
							++InOutStats->GeometricCollisionRejectedByEmptyTerraneCount;
						}
					}

					ComputeDirectionalOverlapDepthStatsKm(
						*this,
						OwningPlateIds,
						ReceiverPlateId,
						*DonorDirectionalSeeds,
						PairDiagnostics.MeanOverlapDepthKm,
						PairDiagnostics.MaxOverlapDepthKm);

					const double EffectiveConvergenceMagnitudeKmPerMy =
						GetGeometricCollisionDirectionalMeanConvergenceMagnitudeKmPerMy(
							PairDiagnostics,
							DonorPlateId);
					FGeometricCollisionPairRecurrenceState& RecurrenceState =
						GeometricCollisionPairRecurrenceByKey.FindOrAdd(PairEntry.Key);
					const bool bRolesChanged =
						RecurrenceState.LastReceiverPlateId != INDEX_NONE &&
						(RecurrenceState.LastReceiverPlateId != ReceiverPlateId ||
							RecurrenceState.LastDonorPlateId != DonorPlateId);
					const bool bSameObservation =
						RecurrenceState.LastObservedStep == CurrentStep;
					if (EffectiveConvergenceMagnitudeKmPerMy <= UE_DOUBLE_SMALL_NUMBER)
					{
						RecurrenceState = FGeometricCollisionPairRecurrenceState{};
					}
					else
					{
						if (bRolesChanged)
						{
							RecurrenceState = FGeometricCollisionPairRecurrenceState{};
						}
						if (!bSameObservation)
						{
							if (RecurrenceState.LastObservedStep != INDEX_NONE &&
								CurrentStep > RecurrenceState.LastObservedStep)
							{
								const double ElapsedTimeMy =
									static_cast<double>(CurrentStep - RecurrenceState.LastObservedStep) * DeltaTimeMyears;
								RecurrenceState.AccumulatedPenetrationKm +=
									EffectiveConvergenceMagnitudeKmPerMy * ElapsedTimeMy;
							}
							++RecurrenceState.ObservationCount;
						}
						else if (RecurrenceState.ObservationCount <= 0)
						{
							RecurrenceState.ObservationCount = 1;
						}

						RecurrenceState.LastObservedStep = CurrentStep;
						RecurrenceState.LastReceiverPlateId = ReceiverPlateId;
						RecurrenceState.LastDonorPlateId = DonorPlateId;
						RecurrenceState.LastOverlapSampleCount = PairDiagnostics.TotalOverlapSampleCount;
						RecurrenceState.LastTerraneEstimate = PairDiagnostics.TerraneEstimate;
						RecurrenceState.LastEffectiveConvergenceKmPerMy = EffectiveConvergenceMagnitudeKmPerMy;
						RecurrenceState.LastMeanConvergenceMagnitudeKmPerMy = EffectiveConvergenceMagnitudeKmPerMy;
						RecurrenceState.LastMaxConvergenceMagnitudeKmPerMy =
							PairDiagnostics.MaxConvergenceMagnitudeKmPerMy;
						RecurrenceState.LastMeanOverlapDepthKm = PairDiagnostics.MeanOverlapDepthKm;
						RecurrenceState.LastMaxOverlapDepthKm = PairDiagnostics.MaxOverlapDepthKm;
						ObservedPersistentPenetrationPairKeys.Add(PairEntry.Key);
					}

					if (RecurrenceState.ObservationCount > 1 &&
						(!bHasBestRepeatedPair ||
							IsRepeatedGeometricCollisionCandidateStronger(
								RecurrenceState,
								BestRepeatedPairState,
								PairDiagnostics,
								BestRepeatedPair)))
					{
						BestRepeatedPair = PairDiagnostics;
						BestRepeatedPairState = RecurrenceState;
						bHasBestRepeatedPair = true;
					}

					const uint64 PairKey = MakeBoundaryContactPlatePairKey(
						PairDiagnostics.PlateA,
						PairDiagnostics.PlateB);
					if (const int32* LastExecutionOrdinal = CollisionLastExecutionOrdinalByPair.Find(PairKey))
					{
						if (InOutStats != nullptr)
						{
							++InOutStats->CollisionRepeatedPairCount;
							if ((ResamplingExecutionOrdinal - *LastExecutionOrdinal) <= CollisionPairCooldownResamples)
							{
								++InOutStats->CollisionRepeatedPairWithinCooldownCount;
							}
						}
					}

					const bool bBelowLegacyDepthThreshold =
						PairDiagnostics.MaxOverlapDepthKm < GeometricCollisionMinOverlapDepthKm;
					if (bBelowLegacyDepthThreshold)
					{
						if (InOutStats != nullptr)
						{
							++InOutStats->GeometricCollisionRejectedByOverlapDepthCount;
							InOutStats->GeometricCollisionRejectedByOverlapDepthTotalOverlapSampleCount +=
								PairDiagnostics.TotalOverlapSampleCount;
							InOutStats->GeometricCollisionRejectedByOverlapDepthMaxOverlapSampleCount =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByOverlapDepthMaxOverlapSampleCount,
									PairDiagnostics.TotalOverlapSampleCount);
							InOutStats->GeometricCollisionRejectedByOverlapDepthTotalTerraneEstimate +=
								PairDiagnostics.TerraneEstimate;
							InOutStats->GeometricCollisionRejectedByOverlapDepthMaxTerraneEstimate =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByOverlapDepthMaxTerraneEstimate,
									PairDiagnostics.TerraneEstimate);
							InOutStats->GeometricCollisionRejectedByOverlapDepthTotalMeanConvergenceKmPerMy +=
								EffectiveConvergenceMagnitudeKmPerMy;
							InOutStats->GeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy,
									PairDiagnostics.MaxConvergenceMagnitudeKmPerMy);
							InOutStats->GeometricCollisionRejectedByOverlapDepthTotalDepthKm +=
								PairDiagnostics.MaxOverlapDepthKm;
							InOutStats->GeometricCollisionRejectedByOverlapDepthMaxDepthKm =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByOverlapDepthMaxDepthKm,
									PairDiagnostics.MaxOverlapDepthKm);
						}
						UE_LOG(
							LogTemp,
							Log,
							TEXT("[GeometricCollisionLegacyDepthGate Step=%d] plate_pair=(%d,%d) overlap_samples=%d terrane_estimate=%d mean_depth_km=%.3f max_depth_km=%.3f effective_convergence_km_per_my=%.6f max_convergence_km_per_my=%.6f observation_count=%d accumulated_penetration_km=%.3f"),
							CurrentStep,
							PairDiagnostics.PlateA,
							PairDiagnostics.PlateB,
							PairDiagnostics.TotalOverlapSampleCount,
							PairDiagnostics.TerraneEstimate,
							PairDiagnostics.MeanOverlapDepthKm,
							PairDiagnostics.MaxOverlapDepthKm,
							EffectiveConvergenceMagnitudeKmPerMy,
							PairDiagnostics.MaxConvergenceMagnitudeKmPerMy,
							RecurrenceState.ObservationCount,
							RecurrenceState.AccumulatedPenetrationKm);
					}

					const bool bPassedPersistentPenetrationGate =
						RecurrenceState.AccumulatedPenetrationKm >= GeometricCollisionMinPersistentPenetrationKm;
					if (!bPassedPersistentPenetrationGate)
					{
						if (InOutStats != nullptr)
						{
							++InOutStats->GeometricCollisionRejectedByPersistentPenetrationCount;
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalOverlapSampleCount +=
								PairDiagnostics.TotalOverlapSampleCount;
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxOverlapSampleCount =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxOverlapSampleCount,
									PairDiagnostics.TotalOverlapSampleCount);
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalTerraneEstimate +=
								PairDiagnostics.TerraneEstimate;
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxTerraneEstimate =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxTerraneEstimate,
									PairDiagnostics.TerraneEstimate);
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalObservationCount +=
								RecurrenceState.ObservationCount;
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxObservationCount =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxObservationCount,
									RecurrenceState.ObservationCount);
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalMeanConvergenceKmPerMy +=
								EffectiveConvergenceMagnitudeKmPerMy;
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy,
									PairDiagnostics.MaxConvergenceMagnitudeKmPerMy);
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalAccumulatedPenetrationKm +=
								RecurrenceState.AccumulatedPenetrationKm;
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxAccumulatedPenetrationKm =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxAccumulatedPenetrationKm,
									RecurrenceState.AccumulatedPenetrationKm);
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationTotalDepthKm +=
								PairDiagnostics.MaxOverlapDepthKm;
							InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxDepthKm =
								FMath::Max(
									InOutStats->GeometricCollisionRejectedByPersistentPenetrationMaxDepthKm,
									PairDiagnostics.MaxOverlapDepthKm);
						}
						UE_LOG(
							LogTemp,
							Log,
							TEXT("[GeometricCollisionPenetrationGate Step=%d] plate_pair=(%d,%d) overlap_samples=%d terrane_estimate=%d mean_depth_km=%.3f max_depth_km=%.3f effective_convergence_km_per_my=%.6f max_convergence_km_per_my=%.6f observation_count=%d accumulated_penetration_km=%.3f threshold_km=%.3f"),
							CurrentStep,
							PairDiagnostics.PlateA,
							PairDiagnostics.PlateB,
							PairDiagnostics.TotalOverlapSampleCount,
							PairDiagnostics.TerraneEstimate,
							PairDiagnostics.MeanOverlapDepthKm,
							PairDiagnostics.MaxOverlapDepthKm,
							EffectiveConvergenceMagnitudeKmPerMy,
							PairDiagnostics.MaxConvergenceMagnitudeKmPerMy,
							RecurrenceState.ObservationCount,
							RecurrenceState.AccumulatedPenetrationKm,
							GeometricCollisionMinPersistentPenetrationKm);
					}
					else
					{
						const int32* LastExecutionOrdinal = CollisionLastExecutionOrdinalByPair.Find(PairKey);
						const bool bWithinCooldown =
							LastExecutionOrdinal != nullptr &&
							(ResamplingExecutionOrdinal - *LastExecutionOrdinal) <= CollisionPairCooldownResamples;
						if (!bWithinCooldown &&
							PairDiagnostics.TerraneEstimate > 0 &&
							(!bHasBestExecutablePair ||
								IsGeometricCollisionPairStronger(PairDiagnostics, BestExecutablePair)))
						{
							BestExecutablePair = PairDiagnostics;
							bHasBestExecutablePair = true;
						}
					}
				}
			}
		}

		if (!bHasBestPair || IsGeometricCollisionPairStronger(PairDiagnostics, BestPair))
		{
			BestPair = PairDiagnostics;
			bHasBestPair = true;
		}
	}

	for (auto RecurrenceIt = GeometricCollisionPairRecurrenceByKey.CreateIterator(); RecurrenceIt; ++RecurrenceIt)
	{
		if (!ObservedPersistentPenetrationPairKeys.Contains(RecurrenceIt.Key()))
		{
			RecurrenceIt.RemoveCurrent();
		}
	}

	if (InOutStats != nullptr && bHasBestPair)
	{
		int32 BestReceiverPlateId = INDEX_NONE;
		int32 BestDonorPlateId = INDEX_NONE;
		const TArray<int32>* BestDirectionalSeeds = nullptr;
		const TArray<int32>* BestOppositeDirectionalSeeds = nullptr;
		EGeometricCollisionDonorSelectionRule BestDonorRule = EGeometricCollisionDonorSelectionRule::None;
		const bool bResolvedBestDirection = ResolveGeometricCollisionDonorReceiver(
			*this,
			BestPair,
			BestReceiverPlateId,
			BestDonorPlateId,
			BestDirectionalSeeds,
			BestOppositeDirectionalSeeds,
			BestDonorRule);
		(void)BestDonorRule;
		InOutStats->GeometricCollisionBestPlateA = BestPair.PlateA;
		InOutStats->GeometricCollisionBestPlateB = BestPair.PlateB;
		InOutStats->GeometricCollisionBestOverlapSampleCount = BestPair.TotalOverlapSampleCount;
		InOutStats->GeometricCollisionBestTerraneEstimate = BestPair.TerraneEstimate;
		InOutStats->GeometricCollisionBestSubductingOverlapSampleCount =
			BestPair.SubductingOverlapSampleCount;
		InOutStats->GeometricCollisionBestOpposingSupportCount =
			BestPair.OpposingContinentalSupportCount;
		InOutStats->bGeometricCollisionBestPassedMassFilter = BestPair.bPassedMassFilter;
		InOutStats->GeometricCollisionDirectionalSeedCount =
			(bResolvedBestDirection && BestDirectionalSeeds != nullptr) ? BestDirectionalSeeds->Num() : 0;
		InOutStats->GeometricCollisionDirectionalSeedCountOpposite =
			(bResolvedBestDirection && BestOppositeDirectionalSeeds != nullptr) ? BestOppositeDirectionalSeeds->Num() : 0;
	}
	if (InOutStats != nullptr && bHasBestRepeatedPair)
	{
		InOutStats->GeometricCollisionRepeatedCandidatePlateA = BestRepeatedPair.PlateA;
		InOutStats->GeometricCollisionRepeatedCandidatePlateB = BestRepeatedPair.PlateB;
		InOutStats->GeometricCollisionRepeatedCandidateObservationCount =
			BestRepeatedPairState.ObservationCount;
		InOutStats->GeometricCollisionRepeatedCandidateOverlapSampleCount =
			BestRepeatedPair.TotalOverlapSampleCount;
		InOutStats->GeometricCollisionRepeatedCandidateTerraneEstimate =
			BestRepeatedPair.TerraneEstimate;
		InOutStats->GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm =
			BestRepeatedPairState.AccumulatedPenetrationKm;
		InOutStats->GeometricCollisionRepeatedCandidateEffectiveConvergenceKmPerMy =
			BestRepeatedPairState.LastEffectiveConvergenceKmPerMy;
		InOutStats->GeometricCollisionRepeatedCandidateMeanConvergenceKmPerMy =
			BestRepeatedPairState.LastMeanConvergenceMagnitudeKmPerMy;
		InOutStats->GeometricCollisionRepeatedCandidateMaxConvergenceKmPerMy =
			BestRepeatedPairState.LastMaxConvergenceMagnitudeKmPerMy;
		InOutStats->GeometricCollisionRepeatedCandidateMeanOverlapDepthKm =
			BestRepeatedPairState.LastMeanOverlapDepthKm;
		InOutStats->GeometricCollisionRepeatedCandidateMaxOverlapDepthKm =
			BestRepeatedPairState.LastMaxOverlapDepthKm;
	}

	if (OutPendingEvent != nullptr && bHasBestExecutablePair && BestExecutablePair.bPassedMassFilter)
	{
		OutPendingEvent->bValid = true;
		OutPendingEvent->PlateA = BestExecutablePair.PlateA;
		OutPendingEvent->PlateB = BestExecutablePair.PlateB;
		OutPendingEvent->SamplesFromAInsideB = BestExecutablePair.SamplesFromAInsideB;
		OutPendingEvent->SamplesFromBInsideA = BestExecutablePair.SamplesFromBInsideA;
		OutPendingEvent->OverlapSampleIndices = BestExecutablePair.SamplesFromAInsideB;
		for (const int32 SampleIndex : BestExecutablePair.SamplesFromBInsideA)
		{
			OutPendingEvent->OverlapSampleIndices.AddUnique(SampleIndex);
		}
		OutPendingEvent->ContactCenter = ComputeSampleSetCentroidOnUnitSphere(*this, OutPendingEvent->OverlapSampleIndices);
		OutPendingEvent->OverlapSampleCount = BestExecutablePair.TotalOverlapSampleCount;
		OutPendingEvent->TerraneEstimate = BestExecutablePair.TerraneEstimate;
		OutPendingEvent->TotalConvergenceMagnitudeAInsideBKmPerMy = BestExecutablePair.TotalConvergenceMagnitudeAInsideBKmPerMy;
		OutPendingEvent->TotalConvergenceMagnitudeBInsideAKmPerMy = BestExecutablePair.TotalConvergenceMagnitudeBInsideAKmPerMy;
		OutPendingEvent->TotalConvergenceMagnitudeKmPerMy = BestExecutablePair.TotalConvergenceMagnitudeKmPerMy;
		OutPendingEvent->MaxConvergenceMagnitudeKmPerMy = BestExecutablePair.MaxConvergenceMagnitudeKmPerMy;
		OutPendingEvent->MeanOverlapDepthKm = BestExecutablePair.MeanOverlapDepthKm;
		OutPendingEvent->MaxOverlapDepthKm = BestExecutablePair.MaxOverlapDepthKm;
		const FGeometricCollisionPairRecurrenceState* BestExecutableRecurrenceState =
			GeometricCollisionPairRecurrenceByKey.Find(MakeBoundaryContactPlatePairKey(
				BestExecutablePair.PlateA,
				BestExecutablePair.PlateB));
		if (BestExecutableRecurrenceState != nullptr)
		{
			OutPendingEvent->EffectiveConvergenceKmPerMy =
				BestExecutableRecurrenceState->LastEffectiveConvergenceKmPerMy;
			OutPendingEvent->AccumulatedPenetrationKm =
				BestExecutableRecurrenceState->AccumulatedPenetrationKm;
			OutPendingEvent->ObservationCount =
				BestExecutableRecurrenceState->ObservationCount;
		}
		int32 ReceiverPlateId = INDEX_NONE;
		int32 DonorPlateId = INDEX_NONE;
		const TArray<int32>* DonorDirectionalSeeds = nullptr;
		const TArray<int32>* OppositeDirectionalSeeds = nullptr;
		EGeometricCollisionDonorSelectionRule DonorRule = EGeometricCollisionDonorSelectionRule::None;
		if (ResolveGeometricCollisionDonorReceiver(
				*this,
				BestExecutablePair,
				ReceiverPlateId,
				DonorPlateId,
				DonorDirectionalSeeds,
				OppositeDirectionalSeeds,
				DonorRule))
		{
			OutPendingEvent->OverridingPlateId = ReceiverPlateId;
			OutPendingEvent->SubductingPlateId = DonorPlateId;
			OutPendingEvent->PolaritySelectionRule = static_cast<uint8>(DonorRule);
			OutPendingEvent->bPolarityChosenFromDirectionalEvidence =
				IsDirectionalGeometricCollisionDonorSelectionRule(DonorRule);
		}
	}

	if (bHasBestPair)
	{
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[GeometricCollisionOverlap Step=%d] plate_pair=(%d,%d) overlap_samples=%d donor_overlap_samples=%d opposite_directional_samples=%d opposing_support=%d terrane_estimate=%d mean_overlap_depth_km=%.3f max_overlap_depth_km=%.3f passed_mass_filter=%d selected_for_execution=%d"),
			CurrentStep,
			BestPair.PlateA,
			BestPair.PlateB,
			BestPair.TotalOverlapSampleCount,
			BestPair.SubductingOverlapSampleCount,
			BestPair.TotalOverlapSampleCount - BestPair.SubductingOverlapSampleCount,
			BestPair.OpposingContinentalSupportCount,
			BestPair.TerraneEstimate,
			BestPair.MeanOverlapDepthKm,
			BestPair.MaxOverlapDepthKm,
			BestPair.bPassedMassFilter ? 1 : 0,
			(bHasBestExecutablePair &&
			 BestPair.PlateA == BestExecutablePair.PlateA &&
			 BestPair.PlateB == BestExecutablePair.PlateB &&
			 BestPair.MinSampleIndex == BestExecutablePair.MinSampleIndex) ? 1 : 0);
	}
	if (bHasBestRepeatedPair)
	{
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[GeometricCollisionTemporalAudit Step=%d] plate_pair=(%d,%d) observation_count=%d overlap_samples=%d terrane_estimate=%d mean_depth_km=%.3f max_depth_km=%.3f effective_convergence_km_per_my=%.6f max_convergence_km_per_my=%.6f accumulated_penetration_km=%.3f"),
			CurrentStep,
			BestRepeatedPair.PlateA,
			BestRepeatedPair.PlateB,
			BestRepeatedPairState.ObservationCount,
			BestRepeatedPair.TotalOverlapSampleCount,
			BestRepeatedPair.TerraneEstimate,
			BestRepeatedPairState.LastMeanOverlapDepthKm,
			BestRepeatedPairState.LastMaxOverlapDepthKm,
			BestRepeatedPairState.LastEffectiveConvergenceKmPerMy,
			BestRepeatedPairState.LastMaxConvergenceMagnitudeKmPerMy,
			BestRepeatedPairState.AccumulatedPenetrationKm);
	}

	return true;
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
		InOutStats->BoundaryContactBestPlateA = INDEX_NONE;
		InOutStats->BoundaryContactBestPlateB = INDEX_NONE;
		InOutStats->BoundaryContactBestZoneSize = 0;
		InOutStats->BoundaryContactBestPersistenceCount = 0;
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
		InOutStats->BoundaryContactBestPlateA = BestZone.PlateA;
		InOutStats->BoundaryContactBestPlateB = BestZone.PlateB;
		InOutStats->BoundaryContactBestZoneSize = BestZone.CandidateCount;
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
	InOutStats.BoundaryContactBestPersistenceCount = 0;
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
	InOutStats.BoundaryContactBestPersistenceCount = Persistence.ConsecutiveResamples;

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
	FResamplingStats* InOutStats)
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
		InOutStats->CollisionCWBoostedCount = 0;
		InOutStats->CollisionSurgeRadiusRad = 0.0;
		InOutStats->CollisionSurgeMeanElevationDelta = 0.0;
		InOutStats->bUsedCachedBoundaryContactCollision = false;
		InOutStats->bUsedGeometricCollisionExecution = false;
		InOutStats->CachedBoundaryContactSeedCount = 0;
		InOutStats->CachedBoundaryContactTerraneSeedCount = 0;
		InOutStats->CachedBoundaryContactTerraneRecoveredCount = 0;
		InOutStats->CachedBoundaryContactPlateA = INDEX_NONE;
		InOutStats->CachedBoundaryContactPlateB = INDEX_NONE;
		InOutStats->GeometricCollisionExecutedPlateA = INDEX_NONE;
		InOutStats->GeometricCollisionExecutedPlateB = INDEX_NONE;
		InOutStats->GeometricCollisionExecutedOverlapSampleCount = 0;
		InOutStats->GeometricCollisionExecutedTerraneEstimate = 0;
		InOutStats->GeometricCollisionExecutedTerraneRecoveredCount = 0;
		InOutStats->GeometricCollisionExecutedCollisionGainCount = 0;
		InOutStats->GeometricCollisionExecutedCWBoostedCount = 0;
		InOutStats->GeometricCollisionExecutedSurgeAffectedCount = 0;
		InOutStats->GeometricCollisionExecutedFromDirectionalPolarityCount = 0;
		InOutStats->BoundaryContactFallbackCollisionGainCount = 0;
		InOutStats->BoundaryContactFallbackTerraneRecoveredCount = 0;
		InOutStats->BoundaryContactFallbackCWBoostedCount = 0;
		InOutStats->BoundaryContactFallbackSurgeAffectedCount = 0;
		InOutStats->GeometricCollisionExecutedOverlapDepthKm = 0.0;
		InOutStats->GeometricCollisionExecutedMaxOverlapDepthKm = 0.0;
		InOutStats->GeometricCollisionExecutedMeanConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionExecutedMaxConvergenceKmPerMy = 0.0;
		InOutStats->GeometricCollisionExecutedAccumulatedPenetrationKm = 0.0;
		InOutStats->GeometricCollisionExecutedDonorTerraneLocalityLimitedCount = 0;
		InOutStats->GeometricCollisionExecutedObservationCount = 0;
		InOutStats->CollisionRepeatedPairCount = 0;
		InOutStats->CollisionRepeatedPairWithinCooldownCount = 0;
		InOutStats->GeometricCollisionQualifiedDirectionalCount = 0;
		InOutStats->GeometricCollisionDirectionalSeedCount = 0;
		InOutStats->GeometricCollisionDirectionalSeedCountOpposite = 0;
		InOutStats->GeometricCollisionFallbackUsedCount = 0;
		InOutStats->CollisionProposedGlobalMaxComponentsAfter = 0;
		InOutStats->CollisionProposedDonorComponentsBefore = 0;
		InOutStats->CollisionProposedDonorComponentsAfter = 0;
		InOutStats->CollisionProposedReceiverComponentsBefore = 0;
		InOutStats->CollisionProposedReceiverComponentsAfter = 0;
		InOutStats->CollisionProposedDonorLargestRemainingComponentSize = 0;
		InOutStats->CollisionProposedDonorNewFragmentCount = 0;
		InOutStats->CollisionProposedDonorSmallestNewFragmentSize = 0;
		InOutStats->CollisionProposedDonorLargestNewFragmentSize = 0;
		InOutStats->CollisionProposedDonorMeanNewFragmentSize = 0.0;
		InOutStats->CollisionProposedReceiverDisconnectedFragmentCount = 0;
		InOutStats->CollisionProposedReceiverLargestNewDisconnectedFragmentSize = 0;
		InOutStats->CollisionProposedTerraneSampleCount = 0;
		InOutStats->CollisionDonorComponentsBefore = 0;
		InOutStats->CollisionDonorComponentsAfter = 0;
		InOutStats->CollisionDonorLargestRemainingComponentSize = 0;
		InOutStats->CollisionDonorNewFragmentCount = 0;
		InOutStats->CollisionDonorSmallestNewFragmentSize = 0;
		InOutStats->CollisionDonorLargestNewFragmentSize = 0;
		InOutStats->CollisionDonorMeanNewFragmentSize = 0.0;
		InOutStats->CollisionAcceptedTerraneSampleCount = 0;
		InOutStats->CollisionReceiverComponentsBefore = 0;
		InOutStats->CollisionReceiverComponentsAfter = 0;
		InOutStats->CollisionReceiverDisconnectedFragmentCount = 0;
		InOutStats->CollisionReceiverLargestNewDisconnectedFragmentSize = 0;
		InOutStats->CollisionTrimmedByDonorProtectionCount = 0;
		InOutStats->CollisionRejectedByDonorProtectionCount = 0;
		InOutStats->CollisionTrimmedByDonorComponentCapCount = 0;
		InOutStats->CollisionTrimmedByDonorFragmentFloorCount = 0;
		InOutStats->CollisionRejectedByDonorComponentCapCount = 0;
		InOutStats->CollisionRejectedByDonorFragmentFloorCount = 0;
		InOutStats->CollisionTrimmedByReceiverProtectionCount = 0;
		InOutStats->CollisionRejectedByReceiverProtectionCount = 0;
		InOutStats->CollisionTransferTrimRatio = 0.0;
		InOutStats->bCollisionReceiverTransferContiguousWithExistingTerritory = false;
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

	bool bGeometricCollisionExecutionFailed = false;
	if (TriggerReason == EResampleTriggerReason::CollisionFollowup &&
		PendingGeometricCollisionEvent.bValid)
	{
		FGeometricCollisionPairDiagnostics PendingPairDiagnostics;
		PendingPairDiagnostics.PlateA = PendingGeometricCollisionEvent.PlateA;
		PendingPairDiagnostics.PlateB = PendingGeometricCollisionEvent.PlateB;
		PendingPairDiagnostics.SamplesFromAInsideB = PendingGeometricCollisionEvent.SamplesFromAInsideB;
		PendingPairDiagnostics.SamplesFromBInsideA = PendingGeometricCollisionEvent.SamplesFromBInsideA;
		PendingPairDiagnostics.TotalConvergenceMagnitudeAInsideBKmPerMy =
			PendingGeometricCollisionEvent.TotalConvergenceMagnitudeAInsideBKmPerMy;
		PendingPairDiagnostics.TotalConvergenceMagnitudeBInsideAKmPerMy =
			PendingGeometricCollisionEvent.TotalConvergenceMagnitudeBInsideAKmPerMy;
		PendingPairDiagnostics.TotalConvergenceMagnitudeKmPerMy =
			PendingGeometricCollisionEvent.TotalConvergenceMagnitudeKmPerMy;
		PendingPairDiagnostics.MaxConvergenceMagnitudeKmPerMy =
			PendingGeometricCollisionEvent.MaxConvergenceMagnitudeKmPerMy;

		int32 OverridingPlateId = INDEX_NONE;
		int32 SubductingPlateId = INDEX_NONE;
		const TArray<int32>* DirectionalSeedSamples = nullptr;
		const TArray<int32>* OppositeDirectionalSamples = nullptr;
		int32 TerraneSeedCount = 0;
		int32 TerraneRecoveredCount = 0;
		int32 LocalityLimitedCount = 0;
		const TCHAR* FailureReason = TEXT("none");
		FCollisionTransferProtectionDiagnostics ProtectionDiagnostics;
		EGeometricCollisionDonorSelectionRule DonorRule = EGeometricCollisionDonorSelectionRule::None;
		const bool bResolvedDirectionalBuckets = ResolveGeometricCollisionDonorReceiver(
			*this,
			PendingPairDiagnostics,
			OverridingPlateId,
			SubductingPlateId,
			DirectionalSeedSamples,
			OppositeDirectionalSamples,
			DonorRule);
		if (InOutStats != nullptr)
		{
			InOutStats->GeometricCollisionExecutedPlateA = PendingGeometricCollisionEvent.PlateA;
			InOutStats->GeometricCollisionExecutedPlateB = PendingGeometricCollisionEvent.PlateB;
			InOutStats->GeometricCollisionExecutedOverlapSampleCount =
				PendingGeometricCollisionEvent.OverlapSampleCount > 0
					? PendingGeometricCollisionEvent.OverlapSampleCount
					: PendingGeometricCollisionEvent.OverlapSampleIndices.Num();
			InOutStats->GeometricCollisionExecutedTerraneEstimate =
				PendingGeometricCollisionEvent.TerraneEstimate;
			InOutStats->GeometricCollisionDirectionalSeedCount =
				(bResolvedDirectionalBuckets && DirectionalSeedSamples != nullptr) ? DirectionalSeedSamples->Num() : 0;
			InOutStats->GeometricCollisionDirectionalSeedCountOpposite =
				(bResolvedDirectionalBuckets && OppositeDirectionalSamples != nullptr) ? OppositeDirectionalSamples->Num() : 0;
			InOutStats->GeometricCollisionExecutedFromDirectionalPolarityCount =
				(bResolvedDirectionalBuckets && IsDirectionalGeometricCollisionDonorSelectionRule(DonorRule)) ? 1 : 0;
			InOutStats->GeometricCollisionExecutedMeanConvergenceKmPerMy =
				PendingGeometricCollisionEvent.EffectiveConvergenceKmPerMy;
			InOutStats->GeometricCollisionExecutedMaxConvergenceKmPerMy =
				PendingGeometricCollisionEvent.MaxConvergenceMagnitudeKmPerMy;
			InOutStats->GeometricCollisionExecutedAccumulatedPenetrationKm =
				PendingGeometricCollisionEvent.AccumulatedPenetrationKm;
			InOutStats->GeometricCollisionExecutedObservationCount =
				PendingGeometricCollisionEvent.ObservationCount;
		}

		const uint64 CollisionPairKey = MakeBoundaryContactPlatePairKey(
			PendingGeometricCollisionEvent.PlateA,
			PendingGeometricCollisionEvent.PlateB);
		if (const int32* LastExecutionOrdinal = CollisionLastExecutionOrdinalByPair.Find(CollisionPairKey))
		{
			if (InOutStats != nullptr)
			{
				InOutStats->CollisionRepeatedPairCount = 1;
			}
			if ((ResamplingExecutionOrdinal - *LastExecutionOrdinal) <= CollisionPairCooldownResamples)
			{
				if (InOutStats != nullptr)
				{
					InOutStats->CollisionRepeatedPairWithinCooldownCount = 1;
				}
				FailureReason = TEXT("pair_cooldown");
			}
		}

		double MeanOverlapDepthKm = PendingGeometricCollisionEvent.MeanOverlapDepthKm;
		double MaxOverlapDepthKm = PendingGeometricCollisionEvent.MaxOverlapDepthKm;
		if (bResolvedDirectionalBuckets && DirectionalSeedSamples != nullptr && DirectionalSeedSamples->Num() > 0)
		{
			double RecomputedMeanOverlapDepthKm = 0.0;
			double RecomputedMaxOverlapDepthKm = 0.0;
			if (ComputeDirectionalOverlapDepthStatsKm(
					*this,
					PreviousPlateIds,
					OverridingPlateId,
					*DirectionalSeedSamples,
					RecomputedMeanOverlapDepthKm,
					RecomputedMaxOverlapDepthKm))
			{
				MeanOverlapDepthKm = RecomputedMeanOverlapDepthKm;
				MaxOverlapDepthKm = RecomputedMaxOverlapDepthKm;
			}
		}
		if (InOutStats != nullptr)
		{
			InOutStats->GeometricCollisionExecutedOverlapDepthKm = MeanOverlapDepthKm;
			InOutStats->GeometricCollisionExecutedMaxOverlapDepthKm = MaxOverlapDepthKm;
		}

		if (!bResolvedDirectionalBuckets)
		{
			FailureReason = TEXT("donor_ambiguous");
		}
		else if (DirectionalSeedSamples == nullptr || DirectionalSeedSamples->IsEmpty())
		{
			FailureReason = TEXT("donor_seed_empty");
		}
		else if (PendingGeometricCollisionEvent.AccumulatedPenetrationKm <
			GeometricCollisionMinPersistentPenetrationKm)
		{
			FailureReason = TEXT("persistent_penetration_below_threshold");
		}
		else if (TryBuildCollisionEventFromSeedSamples(
					*this,
					*DirectionalSeedSamples,
					OverridingPlateId,
					SubductingPlateId,
					PendingGeometricCollisionEvent.ContactCenter,
					PreviousPlateIds,
					PreviousContinentalWeights,
					PreviousTerraneAssignments,
					InOutNewPlateIds,
					OutEvent,
					TerraneSeedCount,
					TerraneRecoveredCount,
					LocalityLimitedCount,
					FailureReason,
					&ProtectionDiagnostics))
		{
			if (InOutStats != nullptr)
			{
				InOutStats->CollisionCount = 1;
				InOutStats->CollisionTerraneId = OutEvent.TerraneId;
				InOutStats->CollisionTerraneSampleCount = OutEvent.TerraneSampleIndices.Num();
				InOutStats->CollisionOverridingPlateId = OutEvent.OverridingPlateId;
				InOutStats->CollisionSubductingPlateId = OutEvent.SubductingPlateId;
				InOutStats->bUsedGeometricCollisionExecution = true;
				InOutStats->GeometricCollisionExecutedTerraneRecoveredCount = TerraneRecoveredCount;
				InOutStats->GeometricCollisionExecutedDonorTerraneLocalityLimitedCount = LocalityLimitedCount;
				InOutStats->CollisionProposedGlobalMaxComponentsAfter =
					ProtectionDiagnostics.Proposed.GlobalMaxComponentsAfter;
				InOutStats->CollisionProposedDonorComponentsBefore =
					ProtectionDiagnostics.Proposed.DonorComponentsBefore;
				InOutStats->CollisionProposedDonorComponentsAfter =
					ProtectionDiagnostics.Proposed.DonorComponentsAfter;
				InOutStats->CollisionProposedReceiverComponentsBefore =
					ProtectionDiagnostics.Proposed.ReceiverComponentsBefore;
				InOutStats->CollisionProposedReceiverComponentsAfter =
					ProtectionDiagnostics.Proposed.ReceiverComponentsAfter;
				InOutStats->CollisionProposedDonorLargestRemainingComponentSize =
					ProtectionDiagnostics.Proposed.DonorLargestRemainingComponentSize;
				InOutStats->CollisionProposedDonorNewFragmentCount =
					ProtectionDiagnostics.Proposed.DonorNewFragmentCount;
				InOutStats->CollisionProposedDonorSmallestNewFragmentSize =
					ProtectionDiagnostics.Proposed.DonorSmallestNewFragmentSize;
				InOutStats->CollisionProposedDonorLargestNewFragmentSize =
					ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize;
				InOutStats->CollisionProposedDonorMeanNewFragmentSize =
					ProtectionDiagnostics.Proposed.DonorMeanNewFragmentSize;
				InOutStats->CollisionProposedReceiverDisconnectedFragmentCount =
					ProtectionDiagnostics.Proposed.ReceiverDisconnectedFragmentCount;
				InOutStats->CollisionProposedReceiverLargestNewDisconnectedFragmentSize =
					ProtectionDiagnostics.Proposed.ReceiverLargestNewDisconnectedFragmentSize;
				InOutStats->CollisionProposedTerraneSampleCount =
					ProtectionDiagnostics.ProposedTerraneSampleCount;
				InOutStats->CollisionAcceptedTerraneSampleCount =
					ProtectionDiagnostics.AppliedTerraneSampleCount;
				InOutStats->CollisionTransferTrimRatio =
					ProtectionDiagnostics.AppliedTrimRatio;
				InOutStats->CollisionTrimmedByDonorProtectionCount =
					ProtectionDiagnostics.bTrimmedByDonorProtection ? 1 : 0;
				InOutStats->CollisionTrimmedByDonorComponentCapCount =
					ProtectionDiagnostics.bTrimmedByDonorComponentCap ? 1 : 0;
				InOutStats->CollisionTrimmedByDonorFragmentFloorCount =
					ProtectionDiagnostics.bTrimmedByDonorFragmentFloor ? 1 : 0;
				InOutStats->CollisionTrimmedByReceiverProtectionCount =
					ProtectionDiagnostics.bTrimmedByReceiverProtection ? 1 : 0;
			}

			UE_LOG(
				LogTemp,
				Log,
				TEXT("[GeometricCollisionExecution Step=%d] pair=(%d,%d) overlap_samples=%d directional_seed_count=%d opposite_directional_seed_count=%d terrane_seed_count=%d terrane_recovered=%d overlap_depth_km=%.3f max_overlap_depth_km=%.3f accumulated_penetration_km=%.3f observation_count=%d locality_limited_count=%d over_plate=%d sub_plate=%d donor_rule=%s proposed_terrane_size=%d accepted_terrane_size=%d trim_ratio=%.3f proposed_donor_delta=%d accepted_donor_delta=%d proposed_receiver_delta=%d proposed_donor_new_fragments=%d accepted_donor_new_fragments=%d proposed_donor_fragment_size_min=%d proposed_donor_fragment_size_max=%d proposed_donor_fragment_size_mean=%.3f trimmed_by_donor=%d trimmed_by_donor_component_cap=%d trimmed_by_donor_fragment_floor=%d trimmed_by_receiver=%d"),
				CurrentStep,
				PendingGeometricCollisionEvent.PlateA,
				PendingGeometricCollisionEvent.PlateB,
				PendingGeometricCollisionEvent.OverlapSampleCount > 0
					? PendingGeometricCollisionEvent.OverlapSampleCount
					: PendingGeometricCollisionEvent.OverlapSampleIndices.Num(),
				DirectionalSeedSamples != nullptr ? DirectionalSeedSamples->Num() : 0,
				OppositeDirectionalSamples != nullptr ? OppositeDirectionalSamples->Num() : 0,
				TerraneSeedCount,
				TerraneRecoveredCount,
				MeanOverlapDepthKm,
				MaxOverlapDepthKm,
				PendingGeometricCollisionEvent.AccumulatedPenetrationKm,
				PendingGeometricCollisionEvent.ObservationCount,
				LocalityLimitedCount,
				OutEvent.OverridingPlateId,
				OutEvent.SubductingPlateId,
				GetGeometricCollisionDonorSelectionRuleName(DonorRule),
				ProtectionDiagnostics.ProposedTerraneSampleCount,
				ProtectionDiagnostics.AppliedTerraneSampleCount,
				ProtectionDiagnostics.AppliedTrimRatio,
				ProtectionDiagnostics.Proposed.DonorComponentsAfter -
					ProtectionDiagnostics.Proposed.DonorComponentsBefore,
				ProtectionDiagnostics.Applied.DonorComponentsAfter -
					ProtectionDiagnostics.Applied.DonorComponentsBefore,
				ProtectionDiagnostics.Proposed.ReceiverComponentsAfter -
					ProtectionDiagnostics.Proposed.ReceiverComponentsBefore,
				ProtectionDiagnostics.Proposed.DonorNewFragmentCount,
				ProtectionDiagnostics.Applied.DonorNewFragmentCount,
				ProtectionDiagnostics.Proposed.DonorSmallestNewFragmentSize,
				ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize,
				ProtectionDiagnostics.Proposed.DonorMeanNewFragmentSize,
				ProtectionDiagnostics.bTrimmedByDonorProtection ? 1 : 0,
				ProtectionDiagnostics.bTrimmedByDonorComponentCap ? 1 : 0,
				ProtectionDiagnostics.bTrimmedByDonorFragmentFloor ? 1 : 0,
				ProtectionDiagnostics.bTrimmedByReceiverProtection ? 1 : 0);
			return true;
		}

		bGeometricCollisionExecutionFailed = true;
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[GeometricCollisionExecutionRejected Step=%d] pair=(%d,%d) overlap_samples=%d directional_seed_count=%d opposite_directional_seed_count=%d overlap_depth_km=%.3f max_overlap_depth_km=%.3f accumulated_penetration_km=%.3f observation_count=%d rejection_reason=%s donor_rule=%s proposed_terrane_size=%d accepted_terrane_size=%d trim_ratio=%.3f proposed_donor_delta=%d proposed_donor_new_fragments=%d proposed_donor_fragment_size_min=%d proposed_donor_fragment_size_max=%d proposed_donor_fragment_size_mean=%.3f rejected_by_donor_component_cap=%d rejected_by_donor_fragment_floor=%d"),
			CurrentStep,
			PendingGeometricCollisionEvent.PlateA,
			PendingGeometricCollisionEvent.PlateB,
			PendingGeometricCollisionEvent.OverlapSampleCount > 0
				? PendingGeometricCollisionEvent.OverlapSampleCount
				: PendingGeometricCollisionEvent.OverlapSampleIndices.Num(),
			DirectionalSeedSamples != nullptr ? DirectionalSeedSamples->Num() : 0,
			OppositeDirectionalSamples != nullptr ? OppositeDirectionalSamples->Num() : 0,
			MeanOverlapDepthKm,
			MaxOverlapDepthKm,
			PendingGeometricCollisionEvent.AccumulatedPenetrationKm,
			PendingGeometricCollisionEvent.ObservationCount,
			FailureReason,
			GetGeometricCollisionDonorSelectionRuleName(DonorRule),
			ProtectionDiagnostics.ProposedTerraneSampleCount,
			ProtectionDiagnostics.AppliedTerraneSampleCount,
			ProtectionDiagnostics.AppliedTrimRatio,
			ProtectionDiagnostics.Proposed.DonorComponentsAfter -
				ProtectionDiagnostics.Proposed.DonorComponentsBefore,
			ProtectionDiagnostics.Proposed.DonorNewFragmentCount,
			ProtectionDiagnostics.Proposed.DonorSmallestNewFragmentSize,
			ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize,
			ProtectionDiagnostics.Proposed.DonorMeanNewFragmentSize,
			ProtectionDiagnostics.bRejectedByDonorComponentCap ? 1 : 0,
			ProtectionDiagnostics.bRejectedByDonorFragmentFloor ? 1 : 0);
		if (InOutStats != nullptr)
		{
			InOutStats->CollisionProposedGlobalMaxComponentsAfter =
				ProtectionDiagnostics.Proposed.GlobalMaxComponentsAfter;
			InOutStats->CollisionProposedDonorComponentsBefore =
				ProtectionDiagnostics.Proposed.DonorComponentsBefore;
			InOutStats->CollisionProposedDonorComponentsAfter =
				ProtectionDiagnostics.Proposed.DonorComponentsAfter;
			InOutStats->CollisionProposedReceiverComponentsBefore =
				ProtectionDiagnostics.Proposed.ReceiverComponentsBefore;
			InOutStats->CollisionProposedReceiverComponentsAfter =
				ProtectionDiagnostics.Proposed.ReceiverComponentsAfter;
			InOutStats->CollisionProposedDonorLargestRemainingComponentSize =
				ProtectionDiagnostics.Proposed.DonorLargestRemainingComponentSize;
			InOutStats->CollisionProposedDonorNewFragmentCount =
				ProtectionDiagnostics.Proposed.DonorNewFragmentCount;
			InOutStats->CollisionProposedDonorSmallestNewFragmentSize =
				ProtectionDiagnostics.Proposed.DonorSmallestNewFragmentSize;
			InOutStats->CollisionProposedDonorLargestNewFragmentSize =
				ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize;
			InOutStats->CollisionProposedDonorMeanNewFragmentSize =
				ProtectionDiagnostics.Proposed.DonorMeanNewFragmentSize;
			InOutStats->CollisionProposedReceiverDisconnectedFragmentCount =
				ProtectionDiagnostics.Proposed.ReceiverDisconnectedFragmentCount;
			InOutStats->CollisionProposedReceiverLargestNewDisconnectedFragmentSize =
				ProtectionDiagnostics.Proposed.ReceiverLargestNewDisconnectedFragmentSize;
			InOutStats->CollisionProposedTerraneSampleCount =
				ProtectionDiagnostics.ProposedTerraneSampleCount;
			InOutStats->CollisionAcceptedTerraneSampleCount =
				ProtectionDiagnostics.AppliedTerraneSampleCount;
			InOutStats->CollisionTransferTrimRatio =
				ProtectionDiagnostics.AppliedTrimRatio;
			InOutStats->CollisionRejectedByDonorProtectionCount =
				ProtectionDiagnostics.bRejectedByDonorProtection ? 1 : 0;
			InOutStats->CollisionRejectedByDonorComponentCapCount =
				ProtectionDiagnostics.bRejectedByDonorComponentCap ? 1 : 0;
			InOutStats->CollisionRejectedByDonorFragmentFloorCount =
				ProtectionDiagnostics.bRejectedByDonorFragmentFloor ? 1 : 0;
			InOutStats->CollisionRejectedByReceiverProtectionCount =
				ProtectionDiagnostics.bRejectedByReceiverProtection ? 1 : 0;
			AccumulateGeometricCollisionFailureReason(*InOutStats, FailureReason);
		}
	}

	if (TriggerReason == EResampleTriggerReason::CollisionFollowup &&
		(!PendingGeometricCollisionEvent.bValid || bGeometricCollisionExecutionFailed) &&
		PendingBoundaryContactCollisionEvent.bValid)
	{
		int32 TerraneSeedCount = 0;
		int32 TerraneRecoveredCount = 0;
		int32 LocalityLimitedCount = 0;
		const TCHAR* FailureReason = TEXT("none");
		FCollisionTransferProtectionDiagnostics ProtectionDiagnostics;
		if (InOutStats != nullptr)
		{
			InOutStats->GeometricCollisionFallbackUsedCount = 1;
			InOutStats->CachedBoundaryContactSeedCount =
				PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices.Num();
			InOutStats->CachedBoundaryContactPlateA = PendingBoundaryContactCollisionEvent.PlateA;
			InOutStats->CachedBoundaryContactPlateB = PendingBoundaryContactCollisionEvent.PlateB;
		}

		const int32 OverridingPlateId =
			IsOverridingPlate(*this, PendingBoundaryContactCollisionEvent.PlateA, PendingBoundaryContactCollisionEvent.PlateB)
				? PendingBoundaryContactCollisionEvent.PlateA
				: PendingBoundaryContactCollisionEvent.PlateB;
		const int32 SubductingPlateId =
			OverridingPlateId == PendingBoundaryContactCollisionEvent.PlateA
				? PendingBoundaryContactCollisionEvent.PlateB
				: PendingBoundaryContactCollisionEvent.PlateA;

		const uint64 CollisionPairKey = MakeBoundaryContactPlatePairKey(
			PendingBoundaryContactCollisionEvent.PlateA,
			PendingBoundaryContactCollisionEvent.PlateB);
		if (const int32* LastExecutionOrdinal = CollisionLastExecutionOrdinalByPair.Find(CollisionPairKey))
		{
			if (InOutStats != nullptr)
			{
				InOutStats->CollisionRepeatedPairCount = 1;
			}
			if ((ResamplingExecutionOrdinal - *LastExecutionOrdinal) <= CollisionPairCooldownResamples)
			{
				if (InOutStats != nullptr)
				{
					InOutStats->CollisionRepeatedPairWithinCooldownCount = 1;
				}
				FailureReason = TEXT("pair_cooldown");
			}
		}

		if (FCString::Strcmp(FailureReason, TEXT("pair_cooldown")) != 0 &&
			TryBuildCollisionEventFromSeedSamples(
				*this,
				PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices,
				OverridingPlateId,
				SubductingPlateId,
				PendingBoundaryContactCollisionEvent.ContactCenter,
				PreviousPlateIds,
				PreviousContinentalWeights,
				PreviousTerraneAssignments,
				InOutNewPlateIds,
				OutEvent,
				TerraneSeedCount,
				TerraneRecoveredCount,
				LocalityLimitedCount,
				FailureReason,
				&ProtectionDiagnostics))
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
				InOutStats->CollisionProposedGlobalMaxComponentsAfter =
					ProtectionDiagnostics.Proposed.GlobalMaxComponentsAfter;
				InOutStats->CollisionProposedDonorComponentsBefore =
					ProtectionDiagnostics.Proposed.DonorComponentsBefore;
				InOutStats->CollisionProposedDonorComponentsAfter =
					ProtectionDiagnostics.Proposed.DonorComponentsAfter;
				InOutStats->CollisionProposedReceiverComponentsBefore =
					ProtectionDiagnostics.Proposed.ReceiverComponentsBefore;
				InOutStats->CollisionProposedReceiverComponentsAfter =
					ProtectionDiagnostics.Proposed.ReceiverComponentsAfter;
				InOutStats->CollisionProposedDonorLargestRemainingComponentSize =
					ProtectionDiagnostics.Proposed.DonorLargestRemainingComponentSize;
				InOutStats->CollisionProposedDonorNewFragmentCount =
					ProtectionDiagnostics.Proposed.DonorNewFragmentCount;
				InOutStats->CollisionProposedDonorSmallestNewFragmentSize =
					ProtectionDiagnostics.Proposed.DonorSmallestNewFragmentSize;
				InOutStats->CollisionProposedDonorLargestNewFragmentSize =
					ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize;
				InOutStats->CollisionProposedDonorMeanNewFragmentSize =
					ProtectionDiagnostics.Proposed.DonorMeanNewFragmentSize;
				InOutStats->CollisionProposedReceiverDisconnectedFragmentCount =
					ProtectionDiagnostics.Proposed.ReceiverDisconnectedFragmentCount;
				InOutStats->CollisionProposedReceiverLargestNewDisconnectedFragmentSize =
					ProtectionDiagnostics.Proposed.ReceiverLargestNewDisconnectedFragmentSize;
				InOutStats->CollisionProposedTerraneSampleCount =
					ProtectionDiagnostics.ProposedTerraneSampleCount;
				InOutStats->CollisionAcceptedTerraneSampleCount =
					ProtectionDiagnostics.AppliedTerraneSampleCount;
				InOutStats->CollisionTransferTrimRatio =
					ProtectionDiagnostics.AppliedTrimRatio;
				InOutStats->CollisionTrimmedByDonorProtectionCount =
					ProtectionDiagnostics.bTrimmedByDonorProtection ? 1 : 0;
				InOutStats->CollisionTrimmedByDonorComponentCapCount =
					ProtectionDiagnostics.bTrimmedByDonorComponentCap ? 1 : 0;
				InOutStats->CollisionTrimmedByDonorFragmentFloorCount =
					ProtectionDiagnostics.bTrimmedByDonorFragmentFloor ? 1 : 0;
				InOutStats->CollisionTrimmedByReceiverProtectionCount =
					ProtectionDiagnostics.bTrimmedByReceiverProtection ? 1 : 0;
			}

			UE_LOG(
				LogTemp,
				Log,
				TEXT("[CachedBoundaryContactCollision Step=%d] pair=(%d,%d) seed_count=%d terrane_seed_count=%d terrane_recovered=%d over_plate=%d sub_plate=%d proposed_terrane_size=%d accepted_terrane_size=%d trim_ratio=%.3f proposed_donor_delta=%d accepted_donor_delta=%d proposed_donor_new_fragments=%d proposed_donor_fragment_size_min=%d proposed_donor_fragment_size_max=%d proposed_donor_fragment_size_mean=%.3f trimmed_by_donor=%d trimmed_by_donor_component_cap=%d trimmed_by_donor_fragment_floor=%d trimmed_by_receiver=%d"),
				CurrentStep,
				PendingBoundaryContactCollisionEvent.PlateA,
				PendingBoundaryContactCollisionEvent.PlateB,
				PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices.Num(),
				TerraneSeedCount,
				TerraneRecoveredCount,
				OutEvent.OverridingPlateId,
				OutEvent.SubductingPlateId,
				ProtectionDiagnostics.ProposedTerraneSampleCount,
				ProtectionDiagnostics.AppliedTerraneSampleCount,
				ProtectionDiagnostics.AppliedTrimRatio,
				ProtectionDiagnostics.Proposed.DonorComponentsAfter -
					ProtectionDiagnostics.Proposed.DonorComponentsBefore,
				ProtectionDiagnostics.Applied.DonorComponentsAfter -
					ProtectionDiagnostics.Applied.DonorComponentsBefore,
				ProtectionDiagnostics.Proposed.DonorNewFragmentCount,
				ProtectionDiagnostics.Proposed.DonorSmallestNewFragmentSize,
				ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize,
				ProtectionDiagnostics.Proposed.DonorMeanNewFragmentSize,
				ProtectionDiagnostics.bTrimmedByDonorProtection ? 1 : 0,
				ProtectionDiagnostics.bTrimmedByDonorComponentCap ? 1 : 0,
				ProtectionDiagnostics.bTrimmedByDonorFragmentFloor ? 1 : 0,
				ProtectionDiagnostics.bTrimmedByReceiverProtection ? 1 : 0);
			return true;
		}

		if (InOutStats != nullptr)
		{
			InOutStats->CachedBoundaryContactTerraneSeedCount = TerraneSeedCount;
			InOutStats->CachedBoundaryContactTerraneRecoveredCount = TerraneRecoveredCount;
			InOutStats->CollisionProposedGlobalMaxComponentsAfter =
				ProtectionDiagnostics.Proposed.GlobalMaxComponentsAfter;
			InOutStats->CollisionProposedDonorComponentsBefore =
				ProtectionDiagnostics.Proposed.DonorComponentsBefore;
			InOutStats->CollisionProposedDonorComponentsAfter =
				ProtectionDiagnostics.Proposed.DonorComponentsAfter;
			InOutStats->CollisionProposedReceiverComponentsBefore =
				ProtectionDiagnostics.Proposed.ReceiverComponentsBefore;
			InOutStats->CollisionProposedReceiverComponentsAfter =
				ProtectionDiagnostics.Proposed.ReceiverComponentsAfter;
			InOutStats->CollisionProposedDonorLargestRemainingComponentSize =
				ProtectionDiagnostics.Proposed.DonorLargestRemainingComponentSize;
			InOutStats->CollisionProposedDonorNewFragmentCount =
				ProtectionDiagnostics.Proposed.DonorNewFragmentCount;
			InOutStats->CollisionProposedDonorSmallestNewFragmentSize =
				ProtectionDiagnostics.Proposed.DonorSmallestNewFragmentSize;
			InOutStats->CollisionProposedDonorLargestNewFragmentSize =
				ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize;
			InOutStats->CollisionProposedDonorMeanNewFragmentSize =
				ProtectionDiagnostics.Proposed.DonorMeanNewFragmentSize;
			InOutStats->CollisionProposedReceiverDisconnectedFragmentCount =
				ProtectionDiagnostics.Proposed.ReceiverDisconnectedFragmentCount;
			InOutStats->CollisionProposedReceiverLargestNewDisconnectedFragmentSize =
				ProtectionDiagnostics.Proposed.ReceiverLargestNewDisconnectedFragmentSize;
			InOutStats->CollisionProposedTerraneSampleCount =
				ProtectionDiagnostics.ProposedTerraneSampleCount;
			InOutStats->CollisionAcceptedTerraneSampleCount =
				ProtectionDiagnostics.AppliedTerraneSampleCount;
			InOutStats->CollisionTransferTrimRatio =
				ProtectionDiagnostics.AppliedTrimRatio;
			InOutStats->CollisionRejectedByDonorProtectionCount =
				ProtectionDiagnostics.bRejectedByDonorProtection ? 1 : 0;
			InOutStats->CollisionRejectedByDonorComponentCapCount =
				ProtectionDiagnostics.bRejectedByDonorComponentCap ? 1 : 0;
			InOutStats->CollisionRejectedByDonorFragmentFloorCount =
				ProtectionDiagnostics.bRejectedByDonorFragmentFloor ? 1 : 0;
			InOutStats->CollisionRejectedByReceiverProtectionCount =
				ProtectionDiagnostics.bRejectedByReceiverProtection ? 1 : 0;
		}

		UE_LOG(
			LogTemp,
			Log,
			TEXT("[CachedBoundaryContactCollisionRejected Step=%d] pair=(%d,%d) seed_count=%d rejection_reason=%s proposed_terrane_size=%d accepted_terrane_size=%d trim_ratio=%.3f proposed_donor_delta=%d proposed_donor_new_fragments=%d proposed_donor_fragment_size_min=%d proposed_donor_fragment_size_max=%d proposed_donor_fragment_size_mean=%.3f rejected_by_donor_component_cap=%d rejected_by_donor_fragment_floor=%d"),
			CurrentStep,
			PendingBoundaryContactCollisionEvent.PlateA,
			PendingBoundaryContactCollisionEvent.PlateB,
			PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices.Num(),
			FailureReason,
			ProtectionDiagnostics.ProposedTerraneSampleCount,
			ProtectionDiagnostics.AppliedTerraneSampleCount,
			ProtectionDiagnostics.AppliedTrimRatio,
			ProtectionDiagnostics.Proposed.DonorComponentsAfter -
				ProtectionDiagnostics.Proposed.DonorComponentsBefore,
			ProtectionDiagnostics.Proposed.DonorNewFragmentCount,
			ProtectionDiagnostics.Proposed.DonorSmallestNewFragmentSize,
			ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize,
			ProtectionDiagnostics.Proposed.DonorMeanNewFragmentSize,
			ProtectionDiagnostics.bRejectedByDonorComponentCap ? 1 : 0,
			ProtectionDiagnostics.bRejectedByDonorFragmentFloor ? 1 : 0);
		(void)LocalityLimitedCount;
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
	int32 LocalityLimitedCount = 0;
	const TCHAR* FailureReason = TEXT("none");
	FCollisionTransferProtectionDiagnostics ProtectionDiagnostics;
	if (!TryBuildCollisionEventFromSeedSamples(
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
			LocalityLimitedCount,
			FailureReason,
			&ProtectionDiagnostics))
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
	(void)LocalityLimitedCount;

	if (InOutStats != nullptr)
	{
		InOutStats->CollisionCount = 1;
		InOutStats->CollisionTerraneId = OutEvent.TerraneId;
		InOutStats->CollisionTerraneSampleCount = OutEvent.TerraneSampleIndices.Num();
		InOutStats->CollisionOverridingPlateId = OutEvent.OverridingPlateId;
		InOutStats->CollisionSubductingPlateId = OutEvent.SubductingPlateId;
		InOutStats->CollisionProposedGlobalMaxComponentsAfter =
			ProtectionDiagnostics.Proposed.GlobalMaxComponentsAfter;
		InOutStats->CollisionProposedDonorComponentsBefore =
			ProtectionDiagnostics.Proposed.DonorComponentsBefore;
		InOutStats->CollisionProposedDonorComponentsAfter =
			ProtectionDiagnostics.Proposed.DonorComponentsAfter;
		InOutStats->CollisionProposedReceiverComponentsBefore =
			ProtectionDiagnostics.Proposed.ReceiverComponentsBefore;
		InOutStats->CollisionProposedReceiverComponentsAfter =
			ProtectionDiagnostics.Proposed.ReceiverComponentsAfter;
		InOutStats->CollisionProposedDonorLargestRemainingComponentSize =
			ProtectionDiagnostics.Proposed.DonorLargestRemainingComponentSize;
		InOutStats->CollisionProposedDonorNewFragmentCount =
			ProtectionDiagnostics.Proposed.DonorNewFragmentCount;
		InOutStats->CollisionProposedDonorLargestNewFragmentSize =
			ProtectionDiagnostics.Proposed.DonorLargestNewFragmentSize;
		InOutStats->CollisionProposedReceiverDisconnectedFragmentCount =
			ProtectionDiagnostics.Proposed.ReceiverDisconnectedFragmentCount;
		InOutStats->CollisionProposedReceiverLargestNewDisconnectedFragmentSize =
			ProtectionDiagnostics.Proposed.ReceiverLargestNewDisconnectedFragmentSize;
		InOutStats->CollisionTrimmedByDonorProtectionCount =
			ProtectionDiagnostics.bTrimmedByDonorProtection ? 1 : 0;
		InOutStats->CollisionTrimmedByReceiverProtectionCount =
			ProtectionDiagnostics.bTrimmedByReceiverProtection ? 1 : 0;
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
	const bool bGeometricExecution = InOutStats != nullptr && InOutStats->bUsedGeometricCollisionExecution;
	double SurgeRadiusRad = FMath::Max(
		CollisionGlobalDistanceRad * FMath::Min(Xi, 1.0),
		MinCollisionRadiusRad);
	if (bGeometricExecution)
	{
		SurgeRadiusRad = FMath::Min(
			CollisionGlobalDistanceRad,
			SurgeRadiusRad * FMath::Max(GeometricCollisionInfluenceRadiusScale, 1.0));
	}

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
		InOutStats->CollisionCWBoostedCount = CWBoostedCount;
		InOutStats->CollisionSurgeRadiusRad = SurgeRadiusRad;
		InOutStats->CollisionSurgeMeanElevationDelta =
			SurgeAffectedCount > 0
				? TotalElevationDelta / static_cast<double>(SurgeAffectedCount)
				: 0.0;
		if (InOutStats->bUsedGeometricCollisionExecution)
		{
			InOutStats->GeometricCollisionExecutedCWBoostedCount = CWBoostedCount;
			InOutStats->GeometricCollisionExecutedSurgeAffectedCount = SurgeAffectedCount;
		}
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
	++ResamplingExecutionOrdinal;
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
	Stats.TriggerReason = TriggerReason;
	Stats.OwnershipMode = OwnershipMode;
	Stats.Step = CurrentStep;
	Stats.Interval = ResampleInterval;
	Stats.GeometricCollisionDonorLocalityClampKm = GeometricCollisionDonorLocalityClampKm;
	Stats.GeometricCollisionInfluenceRadiusScale = GeometricCollisionInfluenceRadiusScale;
	Stats.GeometricCollisionPersistentPenetrationThresholdKm =
		GeometricCollisionMinPersistentPenetrationKm;
	Stats.ContinentalSamplesBefore = 0;
	Stats.ContinentalSamplesAfter = 0;
	Stats.ContinentalSamplesLost = 0;
	Stats.FormerContinentalSamplesTurnedOceanic = 0;
	Stats.FormerContinentalDivergentGapCount = 0;
	Stats.FormerContinentalNonDivergentGapCount = 0;
	Stats.FormerContinentalNonDivergentGapProjectionResolvedCount = 0;
	Stats.FormerContinentalNonDivergentGapNearestCopyResolvedCount = 0;
	Stats.FormerContinentalNonDivergentFallbackOceanizedCount = 0;
	Stats.FormerContinentalDivergentOceanizedCount = 0;
	Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount = 0;
	Stats.FormerContinentalProjectionRecoveredSamePlateNonContinentalFinalCount = 0;
	Stats.FormerContinentalProjectionRecoveredChangedPlateNonContinentalFinalCount = 0;
	Stats.FormerContinentalProjectionRecoveredPreserveModeNonContinentalFinalCount = 0;
	Stats.FormerContinentalProjectionRecoveredFullResolutionNonContinentalFinalCount = 0;
	Stats.FormerContinentalNearestCopyRecoveredNonContinentalFinalCount = 0;
	Stats.FormerContinentalNonGapReclassifiedNonContinentalFinalCount = 0;
	Stats.FormerContinentalChangedPlateNonContinentalFinalCount = 0;
	Stats.FormerContinentalFullResolutionNonContinentalFinalCount = 0;
	Stats.FullResolutionSamePlateNonContinentalFinalCount = 0;
	Stats.FullResolutionChangedPlateNonContinentalFinalCount = 0;
	Stats.FullResolutionCollisionFollowupSamePlateNonContinentalCount = 0;
	Stats.FullResolutionCollisionFollowupChangedPlateNonContinentalCount = 0;
	Stats.FullResolutionRiftFollowupSamePlateNonContinentalCount = 0;
	Stats.FullResolutionRiftFollowupChangedPlateNonContinentalCount = 0;
	Stats.FullResolutionOtherTriggerSamePlateNonContinentalCount = 0;
	Stats.FullResolutionOtherTriggerChangedPlateNonContinentalCount = 0;
	Stats.FormerContinentalPreserveModeNonGapNonContinentalFinalCount = 0;
	Stats.FormerContinentalFallbackQueryNonContinentalFinalCount = 0;
	Stats.FullResolutionSamePlateCWRetainedCount = 0;
	Stats.FullResolutionSamePlateCWThresholdCrossingPreventedCount = 0;
	Stats.FullResolutionCollisionFollowupSamePlateCWRetainedCount = 0;
	Stats.FullResolutionRiftFollowupSamePlateCWRetainedCount = 0;
	Stats.FullResolutionOtherTriggerSamePlateCWRetainedCount = 0;
	Stats.AndeanContinentalGainCount = 0;
	Stats.CollisionContinentalGainCount = 0;
	Stats.NetContinentalSampleDelta = 0;
	Stats.PreserveOwnershipCWRetainedCount = 0;
	Stats.PreserveOwnershipCWThresholdCrossingPreventedCount = 0;
	Stats.PreserveOwnershipFallbackSamePlateRecontainedCount = 0;
	Stats.PreserveOwnershipFallbackSamePlateRetainedCount = 0;
	Stats.PreserveOwnershipFallbackChangedOwnerCount = 0;
	Stats.PreserveOwnershipFallbackGapCount = 0;
	Stats.PreserveOwnershipFallbackDivergentOceanizationCount = 0;
	Stats.PreserveOwnershipContinentalLossCountAfterFallback = 0;
	Stats.PreserveOwnershipPreviousOwnerHysteresisApplicationCount = 0;
	Stats.PreserveOwnershipStronglyContinentalBoundarySavedCount = 0;
	Stats.PreserveOwnershipFallbackChangedOwnerNonGapLossCount = 0;
	Stats.PreserveOwnershipFallbackDivergentLossCount = 0;
	Stats.PreserveOwnershipFallbackNonDivergentProjectionLossCount = 0;
	Stats.PreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount = 0;
	Stats.PreserveOwnershipFallbackStrongLossGE090Count = 0;
	Stats.PreserveOwnershipFallbackStrongLossGE075Count = 0;
	Stats.PreserveOwnershipFallbackStrongLossGE050Count = 0;
	Stats.PreserveOwnershipFallbackLossPairPreviousPlateIds.Reset();
	Stats.PreserveOwnershipFallbackLossPairFinalPlateIds.Reset();
	Stats.PreserveOwnershipFallbackLossPairCounts.Reset();
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
	Stats.RiftLocalizedNonChildSampleRestoreCount = 0;
	Stats.RiftLocalizedGapSampleRestoreCount = 0;
	Stats.RiftLocalizedCWRestoredCount = 0;
		Stats.RiftLocalizedCWPhantomPreventedCount = 0;
		Stats.RiftLocalizedCWContinentalPreventedCount = 0;
		Stats.RiftInterpolationCreatedGainCount = 0;
		Stats.RiftFinalOwnerMismatchGainCountAfterLocalization = 0;
		Stats.RiftFinalOwnerCWReconciledCount = 0;
		Stats.RiftFinalOwnerMismatchGainCountBeforeReconciliation = 0;
		Stats.RiftFinalOwnerMismatchGainCountAfterReconciliation = 0;
		Stats.RiftFinalOwnerMismatchContinentalPreventedCount = 0;
		Stats.RiftSameOwnerChildInterpolationGainCountBeforeReconciliation = 0;
		Stats.RiftSameOwnerChildInterpolationGainCountAfterReconciliation = 0;
		Stats.RiftFinalGainStartedBelow025Count = 0;
		Stats.RiftFinalGainStartedBelow040Count = 0;
		Stats.RiftFinalGainStartedBelow050Count = 0;
		Stats.RiftChildStrayFragmentDetectedCount = 0;
	Stats.RiftChildStrayFragmentReassignedCount = 0;
	Stats.RiftLargestStrayChildFragmentSize = 0;
	Stats.RiftStrayChildFragmentReassignedToSiblingCount = 0;
	Stats.RiftStrayChildFragmentReassignedToOtherNeighborCount = 0;
	Stats.RiftStrayFragmentReassignedByAdjacentSiblingCount = 0;
	Stats.RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount = 0;
	Stats.RiftStrayFragmentForcedNonChildAssignmentCount = 0;
	Stats.RiftStrayFragmentZeroSiblingAdjacencyCount = 0;
	Stats.RiftStrayFragmentPositiveSiblingAdjacencyCount = 0;
	Stats.RiftStrayFragmentRecipientCandidateConsideredCount = 0;
	Stats.RiftStrayFragmentRecipientIncoherenceRejectedCount = 0;
	Stats.RiftStrayFragmentIncoherentForcedAssignmentCount = 0;
	Stats.RiftLargestFragmentCausingRecipientGrowthSize = 0;
	Stats.RiftLocalizedRestoredPreviousPlateIds.Reset();
	Stats.RiftLocalizedRestoredPreviousPlateCounts.Reset();
	Stats.RiftChildAnchorSampleIndices.Reset();
	Stats.RiftChildComponentCountsBeforeSuppression.Reset();
	Stats.RiftChildComponentCountsAfterSuppression.Reset();
	Stats.RiftStrayFragmentRecipientPlateIds.Reset();
	Stats.RiftStrayFragmentRecipientTypeCodes.Reset();
	Stats.RiftStrayFragmentSiblingEdgeCounts.Reset();
	Stats.RiftStrayFragmentSizes.Reset();
	Stats.RiftStrayFragmentRecipientComponentsBefore.Reset();
	Stats.RiftStrayFragmentRecipientComponentsAfter.Reset();
	Stats.RiftPositiveGrowthPlateIds.Reset();
	Stats.RiftPositiveGrowthPlateTypeCodes.Reset();
	Stats.RiftPositiveGrowthPlateComponentsBefore.Reset();
	Stats.RiftPositiveGrowthPlateComponentsAfter.Reset();
	Stats.RiftPositiveGrowthPlateLargestFragmentSizes.Reset();
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
		Stats.RiftChildAnchorSampleIndices = PendingRiftEvent.ChildAnchorSampleIndices;
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
	const bool bTrackTopologyAttribution =
		TriggerReason == EResampleTriggerReason::CollisionFollowup ||
		TriggerReason == EResampleTriggerReason::RiftFollowup;
	const int32 PendingCollisionPlateAAtStart =
		TriggerReason == EResampleTriggerReason::CollisionFollowup
			? (PendingGeometricCollisionEvent.bValid
					? PendingGeometricCollisionEvent.PlateA
					: (PendingBoundaryContactCollisionEvent.bValid ? PendingBoundaryContactCollisionEvent.PlateA : INDEX_NONE))
			: INDEX_NONE;
	const int32 PendingCollisionPlateBAtStart =
		TriggerReason == EResampleTriggerReason::CollisionFollowup
			? (PendingGeometricCollisionEvent.bValid
					? PendingGeometricCollisionEvent.PlateB
					: (PendingBoundaryContactCollisionEvent.bValid ? PendingBoundaryContactCollisionEvent.PlateB : INDEX_NONE))
			: INDEX_NONE;
	FPlateComponentCountSummary TopologyBeforeSummary;
	if (bTrackTopologyAttribution)
	{
		ComputePlateComponentCountSummary(*this, nullptr, TopologyBeforeSummary);
		Stats.TopologyGlobalMaxComponentsBefore = TopologyBeforeSummary.GlobalMaxComponents;
	}
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
		Stats.ContinentalSamplesBefore += Sample.ContinentalWeight >= 0.5f ? 1 : 0;
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
	TArray<uint8> PreserveOwnershipCWRetainFlags;
	TArray<uint8> PreserveOwnershipFallbackQueryFlags;
	TArray<uint8> RiftLocalizedRestoreFlags;
	TArray<EGapResolutionPath> GapResolutionPaths;
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
		OwnershipMode,
		&PreserveOwnershipCWRetainFlags,
		&PreserveOwnershipFallbackQueryFlags);
	Stats.OwnershipQueryMs = (FPlatformTime::Seconds() - OwnershipStartTime) * 1000.0;
	TArray<int32> InterpolationPlateIds;
	if (TriggerReason == EResampleTriggerReason::RiftFollowup && PendingRiftEvent.bValid)
	{
		InterpolationPlateIds = NewPlateIds;
	}
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
	ApplyPreserveOwnershipContinentalWeightRetention(
		PreserveOwnershipCWRetainFlags,
		PreviousContinentalWeights,
		OwnershipMode,
		TriggerReason,
		&Stats);
	if (TriggerReason == EResampleTriggerReason::RiftFollowup && PendingRiftEvent.bValid)
	{
		for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
		{
			const bool bWasContinental = PreviousContinentalWeights[SampleIndex] >= 0.5f;
			const bool bInterpolatedContinental = Samples[SampleIndex].ContinentalWeight >= 0.5f;
			Stats.RiftInterpolationCreatedGainCount +=
				(!bWasContinental && bInterpolatedContinental) ? 1 : 0;
		}
	}
	Stats.InterpolationMs = (FPlatformTime::Seconds() - InterpolationStartTime) * 1000.0;

	const double GapResolutionStartTime = FPlatformTime::Seconds();
	ResolveGaps(
		NewPlateIds,
		GapFlags,
		InterpolatedSubductionDistances,
		InterpolatedSubductionSpeeds,
		&Stats,
		&GapResolutionPaths);
	ApplyPreserveOwnershipFallbackSamePlateRetention(
		NewPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		PreserveOwnershipFallbackQueryFlags,
		GapResolutionPaths,
		OwnershipMode,
		TriggerReason,
		&Stats);
	Stats.GapResolutionMs = (FPlatformTime::Seconds() - GapResolutionStartTime) * 1000.0;

	if (TriggerReason == EResampleTriggerReason::RiftFollowup && PendingRiftEvent.bValid)
	{
			ApplyRiftFollowupLocalizationOverride(
				NewPlateIds,
				PreviousPlateAssignments,
				PreviousContinentalWeights,
				&GapFlags,
				&OverlapFlags,
				&OverlapPlateIds,
				&InterpolatedSubductionDistances,
				&InterpolatedSubductionSpeeds,
				&GapResolutionPaths,
				&RiftLocalizedRestoreFlags,
				&Stats);
			ApplyRiftChildCoherenceProtection(
				NewPlateIds,
				PreviousPlateAssignments,
			&GapFlags,
			&OverlapFlags,
			&OverlapPlateIds,
				&GapResolutionPaths,
				&Stats);
			if (InterpolationPlateIds.Num() == NewPlateIds.Num())
			{
				for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
				{
					const bool bWasContinental = PreviousContinentalWeights[SampleIndex] >= 0.5f;
					const bool bIsContinental = Samples[SampleIndex].ContinentalWeight >= 0.5f;
					Stats.RiftFinalOwnerMismatchGainCountAfterLocalization +=
						(!bWasContinental &&
							bIsContinental &&
							InterpolationPlateIds[SampleIndex] != NewPlateIds[SampleIndex])
							? 1
							: 0;
				}
			}
			ApplyRiftFollowupFinalOwnerContinentalWeightReconciliation(
				InterpolationPlateIds,
				NewPlateIds,
				PreviousPlateAssignments,
				PreviousContinentalWeights,
				RiftLocalizedRestoreFlags,
				&Stats);
		}

	const double OverlapClassificationStartTime = FPlatformTime::Seconds();
	ClassifyOverlapsImpl(*this, OverlapFlags, NewPlateIds, &OverlapPlateIds, Stats);
	Stats.OverlapClassificationMs = (FPlatformTime::Seconds() - OverlapClassificationStartTime) * 1000.0;

	FCollisionEvent CollisionEvent;
	const double CollisionStartTime = FPlatformTime::Seconds();
	const bool bUsePreserveBoundaryContactTrigger = OwnershipMode == EResampleOwnershipMode::PreserveOwnership;
	FPendingGeometricCollisionEvent GeometricCollisionEvent;
	if (bUsePreserveBoundaryContactTrigger)
	{
		DetectGeometricCollisionOverlapDiagnostics(NewPlateIds, &Stats, &GeometricCollisionEvent);
	}
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

	ApplyFullResolutionSamePlateContinentalWeightRetention(
		NewPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		GapResolutionPaths,
		OwnershipMode,
		TriggerReason,
		&Stats);

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

	if (bTrackTopologyAttribution)
	{
		FPlateComponentCountSummary TopologyAfterSummary;
		ComputePlateComponentCountSummary(*this, nullptr, TopologyAfterSummary);
		Stats.TopologyGlobalMaxComponentsAfter = TopologyAfterSummary.GlobalMaxComponents;

		const auto SetAffectedPlateMetrics = [&Stats, &TopologyBeforeSummary, &TopologyAfterSummary](
			const int32 PrimaryPlateId,
			const int32 SecondaryPlateId)
		{
			Stats.TopologyPrimaryAffectedPlateId = PrimaryPlateId;
			Stats.TopologyPrimaryAffectedPlateComponentsBefore =
				TopologyBeforeSummary.ComponentsByPlate.FindRef(PrimaryPlateId);
			Stats.TopologyPrimaryAffectedPlateComponentsAfter =
				TopologyAfterSummary.ComponentsByPlate.FindRef(PrimaryPlateId);
			Stats.TopologySecondaryAffectedPlateId = SecondaryPlateId;
			Stats.TopologySecondaryAffectedPlateComponentsBefore =
				TopologyBeforeSummary.ComponentsByPlate.FindRef(SecondaryPlateId);
			Stats.TopologySecondaryAffectedPlateComponentsAfter =
				TopologyAfterSummary.ComponentsByPlate.FindRef(SecondaryPlateId);
		};

		if (TriggerReason == EResampleTriggerReason::CollisionFollowup)
		{
			int32 ReceiverPlateId = Stats.CollisionOverridingPlateId;
			int32 DonorPlateId = Stats.CollisionSubductingPlateId;
			if (ReceiverPlateId == INDEX_NONE || DonorPlateId == INDEX_NONE)
			{
				if (PendingGeometricCollisionEvent.bValid)
				{
					ReceiverPlateId = PendingGeometricCollisionEvent.OverridingPlateId;
					DonorPlateId = PendingGeometricCollisionEvent.SubductingPlateId;
				}
				else if (PendingCollisionPlateAAtStart != INDEX_NONE && PendingCollisionPlateBAtStart != INDEX_NONE)
				{
					ReceiverPlateId =
						IsOverridingPlate(*this, PendingCollisionPlateAAtStart, PendingCollisionPlateBAtStart)
							? PendingCollisionPlateAAtStart
							: PendingCollisionPlateBAtStart;
					DonorPlateId =
						ReceiverPlateId == PendingCollisionPlateAAtStart
							? PendingCollisionPlateBAtStart
							: PendingCollisionPlateAAtStart;
				}
			}

			SetAffectedPlateMetrics(ReceiverPlateId, DonorPlateId);
			Stats.CollisionDonorComponentsBefore =
				TopologyBeforeSummary.ComponentsByPlate.FindRef(DonorPlateId);
			Stats.CollisionDonorComponentsAfter =
				TopologyAfterSummary.ComponentsByPlate.FindRef(DonorPlateId);
			Stats.CollisionReceiverComponentsBefore =
				TopologyBeforeSummary.ComponentsByPlate.FindRef(ReceiverPlateId);
			Stats.CollisionReceiverComponentsAfter =
				TopologyAfterSummary.ComponentsByPlate.FindRef(ReceiverPlateId);
			if (CollisionEvent.bDetected)
			{
				TArray<int32> CanonicalPlateAssignments;
				CanonicalPlateAssignments.Reserve(Samples.Num());
				for (const FSample& Sample : Samples)
				{
					CanonicalPlateAssignments.Add(Sample.PlateId);
				}

				const FDonorFragmentationDiagnostics DonorDiagnostics =
					ComputeDonorFragmentationDiagnostics(
						*this,
						PreviousPlateAssignments,
						CanonicalPlateAssignments,
						DonorPlateId);
				Stats.CollisionDonorLargestRemainingComponentSize =
					DonorDiagnostics.LargestRemainingComponentSize;
				Stats.CollisionDonorNewFragmentCount = DonorDiagnostics.NewFragmentCount;
				Stats.CollisionDonorSmallestNewFragmentSize =
					DonorDiagnostics.SmallestNewFragmentSize;
				Stats.CollisionDonorLargestNewFragmentSize =
					DonorDiagnostics.LargestNewFragmentSize;
				Stats.CollisionDonorMeanNewFragmentSize =
					DonorDiagnostics.MeanNewFragmentSize;
				Stats.CollisionAcceptedTerraneSampleCount =
					CollisionEvent.TerraneSampleIndices.Num();

				const FCollisionReceiverContiguityDiagnostics ReceiverDiagnostics =
					ComputeCollisionReceiverContiguityDiagnostics(
						*this,
						PreviousPlateAssignments,
						&CanonicalPlateAssignments,
						ReceiverPlateId,
						CollisionEvent.TerraneSampleIndices,
						TopologyBeforeSummary,
						TopologyAfterSummary);
				Stats.CollisionReceiverComponentsBefore = ReceiverDiagnostics.ReceiverComponentsBefore;
				Stats.CollisionReceiverComponentsAfter = ReceiverDiagnostics.ReceiverComponentsAfter;
				Stats.bCollisionReceiverTransferContiguousWithExistingTerritory =
					ReceiverDiagnostics.bContiguousWithExistingTerritory;
				Stats.CollisionReceiverDisconnectedFragmentCount =
					ReceiverDiagnostics.DisconnectedFragmentCount;
				Stats.CollisionReceiverLargestNewDisconnectedFragmentSize =
					ReceiverDiagnostics.LargestNewDisconnectedFragmentSize;
			}
		}
		else if (TriggerReason == EResampleTriggerReason::RiftFollowup)
		{
			Stats.RiftParentComponentsBefore =
				TopologyBeforeSummary.ComponentsByPlate.FindRef(Stats.RiftParentPlateId);
			Stats.RiftParentComponentsAfter =
				TopologyAfterSummary.ComponentsByPlate.FindRef(Stats.RiftParentPlateId);
			Stats.RiftChildComponentCountsBefore.Reset();
			Stats.RiftChildComponentCountsAfter.Reset();
			Stats.RiftChildComponentCountsBefore.Reserve(Stats.RiftChildPlateIds.Num());
			Stats.RiftChildComponentCountsAfter.Reserve(Stats.RiftChildPlateIds.Num());
			TSet<int32> UniqueAffectedPlateIds;
			if (Stats.RiftParentPlateId != INDEX_NONE)
			{
				UniqueAffectedPlateIds.Add(Stats.RiftParentPlateId);
			}

			for (const int32 ChildPlateId : Stats.RiftChildPlateIds)
			{
				const int32 ChildBefore = TopologyBeforeSummary.ComponentsByPlate.FindRef(ChildPlateId);
				const int32 ChildAfter = TopologyAfterSummary.ComponentsByPlate.FindRef(ChildPlateId);
				Stats.RiftChildComponentCountsBefore.Add(ChildBefore);
				Stats.RiftChildComponentCountsAfter.Add(ChildAfter);
				UniqueAffectedPlateIds.Add(ChildPlateId);
			}

			TMap<int32, int32> ReassignmentTypeByPlateId;
			TMap<int32, int32> LargestReassignedFragmentByPlateId;
			for (int32 Index = 0; Index < Stats.RiftStrayFragmentRecipientPlateIds.Num(); ++Index)
			{
				const int32 RecipientPlateId = Stats.RiftStrayFragmentRecipientPlateIds[Index];
				const int32 RecipientTypeCode =
					Stats.RiftStrayFragmentRecipientTypeCodes.IsValidIndex(Index)
						? Stats.RiftStrayFragmentRecipientTypeCodes[Index]
						: static_cast<int32>(ERiftRecipientTypeCode::OtherNeighbor);
				const int32 FragmentSize =
					Stats.RiftStrayFragmentSizes.IsValidIndex(Index)
						? Stats.RiftStrayFragmentSizes[Index]
						: 0;
				ReassignmentTypeByPlateId.Add(RecipientPlateId, RecipientTypeCode);
				LargestReassignedFragmentByPlateId.FindOrAdd(RecipientPlateId) = FMath::Max(
					LargestReassignedFragmentByPlateId.FindRef(RecipientPlateId),
					FragmentSize);
			}

			Stats.RiftPositiveGrowthPlateIds.Reset();
			Stats.RiftPositiveGrowthPlateTypeCodes.Reset();
			Stats.RiftPositiveGrowthPlateComponentsBefore.Reset();
			Stats.RiftPositiveGrowthPlateComponentsAfter.Reset();
			Stats.RiftPositiveGrowthPlateLargestFragmentSizes.Reset();

			TSet<int32> CandidateGrowthPlateIds;
			for (const TPair<int32, int32>& Pair : TopologyBeforeSummary.ComponentsByPlate)
			{
				CandidateGrowthPlateIds.Add(Pair.Key);
			}
			for (const TPair<int32, int32>& Pair : TopologyAfterSummary.ComponentsByPlate)
			{
				CandidateGrowthPlateIds.Add(Pair.Key);
			}
			for (const int32 PlateId : Stats.RiftStrayFragmentRecipientPlateIds)
			{
				CandidateGrowthPlateIds.Add(PlateId);
			}

			struct FGrowthPlateRecord
			{
				int32 PlateId = INDEX_NONE;
				int32 ComponentsBefore = 0;
				int32 ComponentsAfter = 0;
				int32 TypeCode = static_cast<int32>(ERiftRecipientTypeCode::OtherAffectedPlate);
				int32 LargestFragmentSize = 0;
			};

			TArray<FGrowthPlateRecord> PositiveGrowthRecords;
			for (const int32 PlateId : CandidateGrowthPlateIds)
			{
				if (PlateId == INDEX_NONE)
				{
					continue;
				}

				const int32 ComponentsBefore = TopologyBeforeSummary.ComponentsByPlate.FindRef(PlateId);
				const int32 ComponentsAfter = TopologyAfterSummary.ComponentsByPlate.FindRef(PlateId);
				if (ComponentsAfter <= ComponentsBefore)
				{
					continue;
				}

				FGrowthPlateRecord& Record = PositiveGrowthRecords.Emplace_GetRef();
				Record.PlateId = PlateId;
				Record.ComponentsBefore = ComponentsBefore;
				Record.ComponentsAfter = ComponentsAfter;
				Record.TypeCode = ReassignmentTypeByPlateId.FindRef(PlateId);
				if (!ReassignmentTypeByPlateId.Contains(PlateId))
				{
					Record.TypeCode = static_cast<int32>(ERiftRecipientTypeCode::OtherAffectedPlate);
				}
				Record.LargestFragmentSize = LargestReassignedFragmentByPlateId.FindRef(PlateId);
			}

			PositiveGrowthRecords.Sort([](const FGrowthPlateRecord& A, const FGrowthPlateRecord& B)
			{
				const int32 DeltaA = A.ComponentsAfter - A.ComponentsBefore;
				const int32 DeltaB = B.ComponentsAfter - B.ComponentsBefore;
				return
					DeltaA > DeltaB ||
					(DeltaA == DeltaB && A.PlateId < B.PlateId);
			});

			for (const FGrowthPlateRecord& Record : PositiveGrowthRecords)
			{
				Stats.RiftPositiveGrowthPlateIds.Add(Record.PlateId);
				Stats.RiftPositiveGrowthPlateTypeCodes.Add(Record.TypeCode);
				Stats.RiftPositiveGrowthPlateComponentsBefore.Add(Record.ComponentsBefore);
				Stats.RiftPositiveGrowthPlateComponentsAfter.Add(Record.ComponentsAfter);
				Stats.RiftPositiveGrowthPlateLargestFragmentSizes.Add(Record.LargestFragmentSize);
			}

			if (!PositiveGrowthRecords.IsEmpty())
			{
				Stats.TopologyPrimaryAffectedPlateId = PositiveGrowthRecords[0].PlateId;
				Stats.TopologyPrimaryAffectedPlateComponentsBefore = PositiveGrowthRecords[0].ComponentsBefore;
				Stats.TopologyPrimaryAffectedPlateComponentsAfter = PositiveGrowthRecords[0].ComponentsAfter;
			}
			else
			{
				Stats.TopologyPrimaryAffectedPlateId = INDEX_NONE;
				Stats.TopologyPrimaryAffectedPlateComponentsBefore = 0;
				Stats.TopologyPrimaryAffectedPlateComponentsAfter = 0;
			}
			if (PositiveGrowthRecords.Num() > 1)
			{
				Stats.TopologySecondaryAffectedPlateId = PositiveGrowthRecords[1].PlateId;
				Stats.TopologySecondaryAffectedPlateComponentsBefore = PositiveGrowthRecords[1].ComponentsBefore;
				Stats.TopologySecondaryAffectedPlateComponentsAfter = PositiveGrowthRecords[1].ComponentsAfter;
			}
			else
			{
				Stats.TopologySecondaryAffectedPlateId = INDEX_NONE;
				Stats.TopologySecondaryAffectedPlateComponentsBefore = 0;
				Stats.TopologySecondaryAffectedPlateComponentsAfter = 0;
			}
			for (const int32 AffectedPlateId : UniqueAffectedPlateIds)
			{
				Stats.RiftAffectedPlateCountBefore +=
					TopologyBeforeSummary.ComponentsByPlate.FindRef(AffectedPlateId) > 0 ? 1 : 0;
				Stats.RiftAffectedPlateCountAfter +=
					TopologyAfterSummary.ComponentsByPlate.FindRef(AffectedPlateId) > 0 ? 1 : 0;
			}
		}
	}

	TArray<uint8> PostRepartitionContinentalMask;
	PostRepartitionContinentalMask.SetNumZeroed(Samples.Num());
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const bool bWasContinental = PreviousContinentalWeights[SampleIndex] >= 0.5f;
		const bool bIsContinentalAfterRepartition = Samples[SampleIndex].ContinentalWeight >= 0.5f;
		PostRepartitionContinentalMask[SampleIndex] = bIsContinentalAfterRepartition ? 1 : 0;
		Stats.AndeanContinentalGainCount += (!bWasContinental && bIsContinentalAfterRepartition) ? 1 : 0;
	}

	if (!bUsePreserveBoundaryContactTrigger)
	{
		ApplyCollisionElevationSurge(CollisionEvent, &Stats);
		if (Stats.CollisionCount > 0 &&
			Stats.CollisionOverridingPlateId != INDEX_NONE &&
			Stats.CollisionSubductingPlateId != INDEX_NONE)
		{
			const uint64 ExecutedPairKey = MakeBoundaryContactPlatePairKey(
				Stats.CollisionOverridingPlateId,
				Stats.CollisionSubductingPlateId);
			CollisionLastExecutionOrdinalByPair.Add(ExecutedPairKey, ResamplingExecutionOrdinal);
		}
	}

	TMap<uint64, int32> PreserveFallbackChangedOwnerLossPairCounts;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const bool bIsContinentalFinal = Samples[SampleIndex].ContinentalWeight >= 0.5f;
		Stats.CollisionContinentalGainCount +=
			(PostRepartitionContinentalMask[SampleIndex] == 0 && bIsContinentalFinal) ? 1 : 0;
		Stats.ContinentalSamplesAfter += bIsContinentalFinal ? 1 : 0;
		Stats.ContinentalSamplesLost +=
			(PreviousContinentalWeights[SampleIndex] >= 0.5f && !bIsContinentalFinal) ? 1 : 0;
		Stats.FormerContinentalSamplesTurnedOceanic +=
			(PreviousContinentalWeights[SampleIndex] >= 0.5f &&
				GapFlags.IsValidIndex(SampleIndex) &&
				GapFlags[SampleIndex] != 0 &&
				!bIsContinentalFinal)
				? 1
				: 0;
		if (PreviousContinentalWeights[SampleIndex] >= 0.5f && !bIsContinentalFinal)
		{
			const EGapResolutionPath GapResolutionPath =
				GapResolutionPaths.IsValidIndex(SampleIndex) ? GapResolutionPaths[SampleIndex] : EGapResolutionPath::None;
			const bool bSamePlate = Samples[SampleIndex].PlateId == PreviousPlateAssignments[SampleIndex];
			const bool bFallbackQueried =
				PreserveOwnershipFallbackQueryFlags.IsValidIndex(SampleIndex) &&
				PreserveOwnershipFallbackQueryFlags[SampleIndex] != 0;
			if (OwnershipMode == EResampleOwnershipMode::PreserveOwnership && bFallbackQueried)
			{
				++Stats.PreserveOwnershipFallbackStrongLossGE050Count;
				Stats.PreserveOwnershipFallbackStrongLossGE075Count +=
					PreviousContinentalWeights[SampleIndex] >= 0.75f ? 1 : 0;
				Stats.PreserveOwnershipFallbackStrongLossGE090Count +=
					PreviousContinentalWeights[SampleIndex] >= 0.90f ? 1 : 0;

				switch (GapResolutionPath)
				{
				case EGapResolutionPath::None:
					if (!bSamePlate)
					{
						++Stats.PreserveOwnershipFallbackChangedOwnerNonGapLossCount;
						const uint64 PairKey = MakeOwnerTransitionKey(
							PreviousPlateAssignments[SampleIndex],
							Samples[SampleIndex].PlateId);
						PreserveFallbackChangedOwnerLossPairCounts.FindOrAdd(PairKey)++;
					}
					break;
				case EGapResolutionPath::DivergentOceanized:
					++Stats.PreserveOwnershipFallbackDivergentLossCount;
					break;
				case EGapResolutionPath::NonDivergentProjection:
					++Stats.PreserveOwnershipFallbackNonDivergentProjectionLossCount;
					break;
				case EGapResolutionPath::NonDivergentFallbackOceanized:
					++Stats.PreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount;
					break;
				default:
					break;
				}
			}
			if (GapResolutionPath == EGapResolutionPath::NonDivergentProjection)
			{
				++Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount;
				Stats.FormerContinentalProjectionRecoveredSamePlateNonContinentalFinalCount += bSamePlate ? 1 : 0;
				Stats.FormerContinentalProjectionRecoveredChangedPlateNonContinentalFinalCount += bSamePlate ? 0 : 1;
				Stats.FormerContinentalProjectionRecoveredPreserveModeNonContinentalFinalCount +=
					(OwnershipMode == EResampleOwnershipMode::PreserveOwnership) ? 1 : 0;
				Stats.FormerContinentalProjectionRecoveredFullResolutionNonContinentalFinalCount +=
					(OwnershipMode == EResampleOwnershipMode::FullResolution) ? 1 : 0;
			}
			else if (GapResolutionPath == EGapResolutionPath::NonDivergentNearestCopy)
			{
				++Stats.FormerContinentalNearestCopyRecoveredNonContinentalFinalCount;
			}
			else if (GapResolutionPath == EGapResolutionPath::None)
			{
				++Stats.FormerContinentalNonGapReclassifiedNonContinentalFinalCount;
				Stats.FormerContinentalChangedPlateNonContinentalFinalCount += bSamePlate ? 0 : 1;
				Stats.FormerContinentalFullResolutionNonContinentalFinalCount +=
					(OwnershipMode == EResampleOwnershipMode::FullResolution) ? 1 : 0;
				if (OwnershipMode == EResampleOwnershipMode::FullResolution)
				{
					Stats.FullResolutionSamePlateNonContinentalFinalCount += bSamePlate ? 1 : 0;
					Stats.FullResolutionChangedPlateNonContinentalFinalCount += bSamePlate ? 0 : 1;
					switch (TriggerReason)
					{
					case EResampleTriggerReason::CollisionFollowup:
						Stats.FullResolutionCollisionFollowupSamePlateNonContinentalCount +=
							bSamePlate ? 1 : 0;
						Stats.FullResolutionCollisionFollowupChangedPlateNonContinentalCount +=
							bSamePlate ? 0 : 1;
						break;
					case EResampleTriggerReason::RiftFollowup:
						Stats.FullResolutionRiftFollowupSamePlateNonContinentalCount +=
							bSamePlate ? 1 : 0;
						Stats.FullResolutionRiftFollowupChangedPlateNonContinentalCount +=
							bSamePlate ? 0 : 1;
						break;
					default:
						Stats.FullResolutionOtherTriggerSamePlateNonContinentalCount +=
							bSamePlate ? 1 : 0;
						Stats.FullResolutionOtherTriggerChangedPlateNonContinentalCount +=
							bSamePlate ? 0 : 1;
						break;
					}
				}
				Stats.FormerContinentalPreserveModeNonGapNonContinentalFinalCount +=
					(OwnershipMode == EResampleOwnershipMode::PreserveOwnership) ? 1 : 0;
				Stats.FormerContinentalFallbackQueryNonContinentalFinalCount += bFallbackQueried ? 1 : 0;
			}
		}
	}
	if (!PreserveFallbackChangedOwnerLossPairCounts.IsEmpty())
	{
		TArray<TPair<uint64, int32>> SortedLossPairs;
		SortedLossPairs.Reserve(PreserveFallbackChangedOwnerLossPairCounts.Num());
		for (const TPair<uint64, int32>& Pair : PreserveFallbackChangedOwnerLossPairCounts)
		{
			SortedLossPairs.Add(Pair);
		}

		SortedLossPairs.Sort([](const TPair<uint64, int32>& A, const TPair<uint64, int32>& B)
		{
			return
				A.Value > B.Value ||
				(A.Value == B.Value && A.Key < B.Key);
		});

		const int32 PairLimit = FMath::Min(SortedLossPairs.Num(), 5);
		Stats.PreserveOwnershipFallbackLossPairPreviousPlateIds.Reserve(PairLimit);
		Stats.PreserveOwnershipFallbackLossPairFinalPlateIds.Reserve(PairLimit);
		Stats.PreserveOwnershipFallbackLossPairCounts.Reserve(PairLimit);
		for (int32 PairIndex = 0; PairIndex < PairLimit; ++PairIndex)
		{
			Stats.PreserveOwnershipFallbackLossPairPreviousPlateIds.Add(
				DecodeOwnerTransitionPreviousPlateId(SortedLossPairs[PairIndex].Key));
			Stats.PreserveOwnershipFallbackLossPairFinalPlateIds.Add(
				DecodeOwnerTransitionFinalPlateId(SortedLossPairs[PairIndex].Key));
			Stats.PreserveOwnershipFallbackLossPairCounts.Add(SortedLossPairs[PairIndex].Value);
		}
	}
	Stats.NetContinentalSampleDelta = Stats.ContinentalSamplesAfter - Stats.ContinentalSamplesBefore;
	if (Stats.CollisionCount > 0)
	{
		if (Stats.bUsedGeometricCollisionExecution)
		{
			Stats.GeometricCollisionExecutedCollisionGainCount = Stats.CollisionContinentalGainCount;
		}
		else if (Stats.bUsedCachedBoundaryContactCollision)
		{
			Stats.BoundaryContactFallbackCollisionGainCount = Stats.CollisionContinentalGainCount;
			Stats.BoundaryContactFallbackTerraneRecoveredCount = Stats.CollisionTerraneSampleCount;
			Stats.BoundaryContactFallbackCWBoostedCount = Stats.CollisionCWBoostedCount;
			Stats.BoundaryContactFallbackSurgeAffectedCount = Stats.CollisionSurgeAffectedCount;
		}
	}

	ComputePlateScores();
	if (bUsePreserveBoundaryContactTrigger)
	{
		FPendingBoundaryContactCollisionEvent BoundaryContactCollisionEvent;
		const bool bImmediateBoundaryContactTrigger =
			DetectBoundaryContactCollisionTrigger(&Stats, &BoundaryContactCollisionEvent);
		const bool bPersistenceBoundaryContactTrigger = UpdateBoundaryContactCollisionPersistence(Stats, ResampleInterval);
		if (GeometricCollisionEvent.bValid)
		{
			PendingGeometricCollisionEvent = GeometricCollisionEvent;
			PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
			bPendingFullResolutionResample = true;
			if (bPersistenceBoundaryContactTrigger)
			{
				bPendingBoundaryContactPersistenceReset = true;
			}
		}
		else if (bImmediateBoundaryContactTrigger || bPersistenceBoundaryContactTrigger)
		{
			PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};
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

	PopulateCanonicalSampleMetrics(*this, Stats);
	if (bTrackTopologyAttribution)
	{
		Stats.TopologyGlobalMaxComponentsAfter = Stats.MaxComponentsPerPlate;
	}

	for (FPlate& Plate : Plates)
	{
		Plate.CumulativeRotation = FQuat4d::Identity;
	}

	Stats.TotalMs = (FPlatformTime::Seconds() - TotalStartTime) * 1000.0;
	LastResamplingStats = Stats;
	ResamplingHistory.Add(Stats);
	LastResampleTriggerReason = TriggerReason;
	LastResampleOwnershipMode = OwnershipMode;
	const int32 ResampleIndex = ResamplingSteps.Num() + 1;
	ResamplingSteps.Add(CurrentStep);

	if (TriggerReason == EResampleTriggerReason::CollisionFollowup &&
		Stats.CollisionCount > 0 &&
		Stats.CollisionOverridingPlateId != INDEX_NONE &&
		Stats.CollisionSubductingPlateId != INDEX_NONE)
	{
		const uint64 ExecutedPairKey = MakeBoundaryContactPlatePairKey(
			FMath::Min(Stats.CollisionOverridingPlateId, Stats.CollisionSubductingPlateId),
			FMath::Max(Stats.CollisionOverridingPlateId, Stats.CollisionSubductingPlateId));
		GeometricCollisionPairRecurrenceByKey.Remove(ExecutedPairKey);
	}

	if (TriggerReason == EResampleTriggerReason::CollisionFollowup &&
		(bPendingBoundaryContactPersistenceReset || Stats.bUsedGeometricCollisionExecution))
	{
		ResetBoundaryContactCollisionPersistence();
		PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};
	}
	else if (TriggerReason == EResampleTriggerReason::CollisionFollowup)
	{
		PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
		PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};
	}
	if (TriggerReason == EResampleTriggerReason::RiftFollowup)
	{
		PendingRiftEvent = FPendingRiftEvent{};
	}

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[ResampleMetrics R=%d Step=%d] continental_area_fraction=%.6f continental_sample_count=%d continental_samples_before=%d continental_samples_after=%d continental_samples_lost=%d former_continental_turned_oceanic=%d former_continental_divergent_gap_count=%d former_continental_non_divergent_gap_count=%d former_continental_non_divergent_gap_projection_resolved_count=%d former_continental_non_divergent_gap_nearest_copy_resolved_count=%d former_continental_non_divergent_fallback_oceanized_count=%d former_continental_divergent_oceanized_count=%d former_continental_projection_recovered_non_continental_final_count=%d former_continental_projection_recovered_same_plate_non_continental_final_count=%d former_continental_projection_recovered_changed_plate_non_continental_final_count=%d former_continental_projection_recovered_preserve_mode_non_continental_final_count=%d former_continental_projection_recovered_full_resolution_non_continental_final_count=%d former_continental_nearest_copy_recovered_non_continental_final_count=%d former_continental_non_gap_reclassified_non_continental_final_count=%d former_continental_changed_plate_non_continental_final_count=%d former_continental_full_resolution_non_continental_final_count=%d former_continental_preserve_mode_non_gap_non_continental_final_count=%d former_continental_fallback_query_non_continental_final_count=%d andean_continental_gain_count=%d collision_continental_gain_count=%d net_continental_sample_delta=%d preserve_cw_retained_count=%d preserve_cw_threshold_crossing_prevented_count=%d overlap_count=%d overlap_rate=%.6f gap_count=%d gap_rate=%.6f divergent_gap_count=%d numerical_gap_count=%d plate_count=%d max_components_per_plate=%d boundary_sample_count=%d boundary_sample_fraction=%.6f elevation_min=%.6f elevation_max=%.6f elevation_mean=%.6f subduction_front_count=%d active_collision_tracks=%d boundary_contact_best_plate_a=%d boundary_contact_best_plate_b=%d boundary_contact_best_zone_size=%d boundary_contact_best_persistence_count=%d geometric_collision_candidate_count=%d geometric_collision_qualified_count=%d geometric_collision_qualified_but_donor_ambiguous_count=%d geometric_collision_qualified_but_donor_seed_empty_count=%d geometric_collision_qualified_using_directional_donor_count=%d geometric_collision_qualified_using_fallback_donor_rule_count=%d geometric_collision_rejected_by_mass_filter_count=%d geometric_collision_rejected_by_overlap_depth_count=%d geometric_collision_rejected_by_overlap_depth_total_overlap_sample_count=%d geometric_collision_rejected_by_overlap_depth_max_overlap_sample_count=%d geometric_collision_rejected_by_overlap_depth_total_terrane_estimate=%d geometric_collision_rejected_by_overlap_depth_max_terrane_estimate=%d geometric_collision_rejected_by_overlap_depth_total_mean_convergence_km_per_my=%.6f geometric_collision_rejected_by_overlap_depth_max_convergence_km_per_my=%.6f geometric_collision_rejected_by_overlap_depth_total_depth_km=%.6f geometric_collision_rejected_by_overlap_depth_max_depth_km=%.6f geometric_collision_rejected_by_persistent_penetration_count=%d geometric_collision_rejected_by_persistent_penetration_total_overlap_sample_count=%d geometric_collision_rejected_by_persistent_penetration_max_overlap_sample_count=%d geometric_collision_rejected_by_persistent_penetration_total_terrane_estimate=%d geometric_collision_rejected_by_persistent_penetration_max_terrane_estimate=%d geometric_collision_rejected_by_persistent_penetration_total_observation_count=%d geometric_collision_rejected_by_persistent_penetration_max_observation_count=%d geometric_collision_rejected_by_persistent_penetration_total_mean_convergence_km_per_my=%.6f geometric_collision_rejected_by_persistent_penetration_max_convergence_km_per_my=%.6f geometric_collision_rejected_by_persistent_penetration_total_accumulated_penetration_km=%.6f geometric_collision_rejected_by_persistent_penetration_max_accumulated_penetration_km=%.6f geometric_collision_rejected_by_persistent_penetration_total_depth_km=%.6f geometric_collision_rejected_by_persistent_penetration_max_depth_km=%.6f geometric_collision_rejected_by_empty_terrane_count=%d geometric_collision_rejected_by_role_resolution_count=%d geometric_collision_rejected_by_seed_direction_count=%d geometric_collision_qualified_directional_count=%d geometric_collision_directional_seed_count=%d geometric_collision_directional_seed_count_opposite=%d geometric_collision_seed_direction_audit_count=%d geometric_collision_seed_direction_mismatch_count=%d geometric_collision_only_overriding_bucket_populated_count=%d geometric_collision_only_subducting_bucket_populated_count=%d geometric_collision_both_directional_buckets_populated_count=%d geometric_collision_neither_directional_buckets_populated_count=%d geometric_collision_polarity_chosen_from_directional_evidence_count=%d geometric_collision_polarity_chosen_from_bidirectional_tie_break_count=%d geometric_collision_fallback_used_count=%d geometric_collision_overlap_sample_count=%d geometric_collision_pair_count=%d geometric_collision_qualified_pair_count=%d geometric_collision_largest_terrane_estimate=%d geometric_collision_best_plate_a=%d geometric_collision_best_plate_b=%d geometric_collision_best_overlap_sample_count=%d geometric_collision_best_terrane_estimate=%d geometric_collision_best_subducting_overlap_sample_count=%d geometric_collision_best_opposing_support_count=%d geometric_collision_best_passed_mass_filter=%d collision_cw_boosted_count=%d used_geometric_collision_execution=%d geometric_collision_executed_plate_a=%d geometric_collision_executed_plate_b=%d geometric_collision_executed_overlap_sample_count=%d geometric_collision_executed_terrane_estimate=%d geometric_collision_executed_terrane_recovered_count=%d geometric_collision_executed_collision_gain_count=%d geometric_collision_executed_cw_boosted_count=%d geometric_collision_executed_surge_affected_count=%d geometric_collision_executed_from_directional_polarity_count=%d boundary_contact_fallback_collision_gain_count=%d boundary_contact_fallback_terrane_recovered_count=%d boundary_contact_fallback_cw_boosted_count=%d boundary_contact_fallback_surge_affected_count=%d geometric_collision_executed_overlap_depth_km=%.6f geometric_collision_executed_max_overlap_depth_km=%.6f geometric_collision_executed_mean_convergence_km_per_my=%.6f geometric_collision_executed_max_convergence_km_per_my=%.6f geometric_collision_executed_accumulated_penetration_km=%.6f geometric_collision_executed_observation_count=%d geometric_collision_executed_donor_terrane_locality_limited_count=%d geometric_collision_repeated_candidate_plate_a=%d geometric_collision_repeated_candidate_plate_b=%d geometric_collision_repeated_candidate_observation_count=%d geometric_collision_repeated_candidate_overlap_sample_count=%d geometric_collision_repeated_candidate_terrane_estimate=%d geometric_collision_repeated_candidate_accumulated_penetration_km=%.6f geometric_collision_repeated_candidate_effective_convergence_km_per_my=%.6f geometric_collision_repeated_candidate_mean_convergence_km_per_my=%.6f geometric_collision_repeated_candidate_max_convergence_km_per_my=%.6f geometric_collision_repeated_candidate_mean_overlap_depth_km=%.6f geometric_collision_repeated_candidate_max_overlap_depth_km=%.6f geometric_collision_donor_locality_clamp_km=%.3f geometric_collision_influence_radius_scale=%.3f geometric_collision_persistent_penetration_threshold_km=%.3f collision_repeated_pair_count=%d collision_repeated_pair_within_cooldown_count=%d collision_event_count=%d rift_event_count=%d resampling_interval_R=%d trigger_reason=%s ownership_mode=%s"),
		ResampleIndex,
		CurrentStep,
		Stats.ContinentalAreaFraction,
		Stats.ContinentalSampleCount,
		Stats.ContinentalSamplesBefore,
		Stats.ContinentalSamplesAfter,
		Stats.ContinentalSamplesLost,
		Stats.FormerContinentalSamplesTurnedOceanic,
		Stats.FormerContinentalDivergentGapCount,
		Stats.FormerContinentalNonDivergentGapCount,
		Stats.FormerContinentalNonDivergentGapProjectionResolvedCount,
		Stats.FormerContinentalNonDivergentGapNearestCopyResolvedCount,
		Stats.FormerContinentalNonDivergentFallbackOceanizedCount,
		Stats.FormerContinentalDivergentOceanizedCount,
		Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredSamePlateNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredChangedPlateNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredPreserveModeNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredFullResolutionNonContinentalFinalCount,
		Stats.FormerContinentalNearestCopyRecoveredNonContinentalFinalCount,
		Stats.FormerContinentalNonGapReclassifiedNonContinentalFinalCount,
		Stats.FormerContinentalChangedPlateNonContinentalFinalCount,
		Stats.FormerContinentalFullResolutionNonContinentalFinalCount,
		Stats.FormerContinentalPreserveModeNonGapNonContinentalFinalCount,
		Stats.FormerContinentalFallbackQueryNonContinentalFinalCount,
		Stats.AndeanContinentalGainCount,
		Stats.CollisionContinentalGainCount,
		Stats.NetContinentalSampleDelta,
		Stats.PreserveOwnershipCWRetainedCount,
		Stats.PreserveOwnershipCWThresholdCrossingPreventedCount,
		Stats.OverlapCount,
		Stats.OverlapRate,
		Stats.GapCount,
		Stats.GapRate,
		Stats.DivergentGapCount,
		Stats.NonDivergentGapCount,
		Stats.PlateCount,
		Stats.MaxComponentsPerPlate,
		Stats.BoundarySampleCount,
		Stats.BoundarySampleFraction,
		Stats.ElevationMinKm,
		Stats.ElevationMaxKm,
		Stats.ElevationMeanKm,
		Stats.SubductionSeedSampleCount,
		Stats.ActiveCollisionTrackCount,
		Stats.BoundaryContactBestPlateA,
		Stats.BoundaryContactBestPlateB,
		Stats.BoundaryContactBestZoneSize,
		Stats.BoundaryContactBestPersistenceCount,
		Stats.GeometricCollisionCandidateCount,
		Stats.GeometricCollisionQualifiedCount,
		Stats.GeometricCollisionQualifiedButDonorAmbiguousCount,
		Stats.GeometricCollisionQualifiedButDonorSeedEmptyCount,
		Stats.GeometricCollisionQualifiedUsingDirectionalDonorCount,
		Stats.GeometricCollisionQualifiedUsingFallbackDonorRuleCount,
		Stats.GeometricCollisionRejectedByMassFilterCount,
		Stats.GeometricCollisionRejectedByOverlapDepthCount,
		Stats.GeometricCollisionRejectedByOverlapDepthTotalOverlapSampleCount,
		Stats.GeometricCollisionRejectedByOverlapDepthMaxOverlapSampleCount,
		Stats.GeometricCollisionRejectedByOverlapDepthTotalTerraneEstimate,
		Stats.GeometricCollisionRejectedByOverlapDepthMaxTerraneEstimate,
		Stats.GeometricCollisionRejectedByOverlapDepthTotalMeanConvergenceKmPerMy,
		Stats.GeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy,
		Stats.GeometricCollisionRejectedByOverlapDepthTotalDepthKm,
		Stats.GeometricCollisionRejectedByOverlapDepthMaxDepthKm,
		Stats.GeometricCollisionRejectedByPersistentPenetrationCount,
		Stats.GeometricCollisionRejectedByPersistentPenetrationTotalOverlapSampleCount,
		Stats.GeometricCollisionRejectedByPersistentPenetrationMaxOverlapSampleCount,
		Stats.GeometricCollisionRejectedByPersistentPenetrationTotalTerraneEstimate,
		Stats.GeometricCollisionRejectedByPersistentPenetrationMaxTerraneEstimate,
		Stats.GeometricCollisionRejectedByPersistentPenetrationTotalObservationCount,
		Stats.GeometricCollisionRejectedByPersistentPenetrationMaxObservationCount,
		Stats.GeometricCollisionRejectedByPersistentPenetrationTotalMeanConvergenceKmPerMy,
		Stats.GeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy,
		Stats.GeometricCollisionRejectedByPersistentPenetrationTotalAccumulatedPenetrationKm,
		Stats.GeometricCollisionRejectedByPersistentPenetrationMaxAccumulatedPenetrationKm,
		Stats.GeometricCollisionRejectedByPersistentPenetrationTotalDepthKm,
		Stats.GeometricCollisionRejectedByPersistentPenetrationMaxDepthKm,
		Stats.GeometricCollisionRejectedByEmptyTerraneCount,
		Stats.GeometricCollisionRejectedByRoleResolutionCount,
		Stats.GeometricCollisionRejectedBySeedDirectionCount,
		Stats.GeometricCollisionQualifiedDirectionalCount,
		Stats.GeometricCollisionDirectionalSeedCount,
		Stats.GeometricCollisionDirectionalSeedCountOpposite,
		Stats.GeometricCollisionSeedDirectionAuditCount,
		Stats.GeometricCollisionSeedDirectionMismatchCount,
		Stats.GeometricCollisionOnlyOverridingBucketPopulatedCount,
		Stats.GeometricCollisionOnlySubductingBucketPopulatedCount,
		Stats.GeometricCollisionBothDirectionalBucketsPopulatedCount,
		Stats.GeometricCollisionNeitherDirectionalBucketPopulatedCount,
		Stats.GeometricCollisionPolarityChosenFromDirectionalEvidenceCount,
		Stats.GeometricCollisionPolarityChosenFromBidirectionalTieBreakCount,
		Stats.GeometricCollisionFallbackUsedCount,
		Stats.GeometricCollisionOverlapSampleCount,
		Stats.GeometricCollisionPairCount,
		Stats.GeometricCollisionQualifiedPairCount,
		Stats.GeometricCollisionLargestTerraneEstimate,
		Stats.GeometricCollisionBestPlateA,
		Stats.GeometricCollisionBestPlateB,
		Stats.GeometricCollisionBestOverlapSampleCount,
		Stats.GeometricCollisionBestTerraneEstimate,
		Stats.GeometricCollisionBestSubductingOverlapSampleCount,
		Stats.GeometricCollisionBestOpposingSupportCount,
		Stats.bGeometricCollisionBestPassedMassFilter ? 1 : 0,
		Stats.CollisionCWBoostedCount,
		Stats.bUsedGeometricCollisionExecution ? 1 : 0,
		Stats.GeometricCollisionExecutedPlateA,
		Stats.GeometricCollisionExecutedPlateB,
			Stats.GeometricCollisionExecutedOverlapSampleCount,
			Stats.GeometricCollisionExecutedTerraneEstimate,
			Stats.GeometricCollisionExecutedTerraneRecoveredCount,
			Stats.GeometricCollisionExecutedCollisionGainCount,
			Stats.GeometricCollisionExecutedCWBoostedCount,
			Stats.GeometricCollisionExecutedSurgeAffectedCount,
			Stats.GeometricCollisionExecutedFromDirectionalPolarityCount,
			Stats.BoundaryContactFallbackCollisionGainCount,
			Stats.BoundaryContactFallbackTerraneRecoveredCount,
			Stats.BoundaryContactFallbackCWBoostedCount,
			Stats.BoundaryContactFallbackSurgeAffectedCount,
			Stats.GeometricCollisionExecutedOverlapDepthKm,
			Stats.GeometricCollisionExecutedMaxOverlapDepthKm,
		Stats.GeometricCollisionExecutedMeanConvergenceKmPerMy,
		Stats.GeometricCollisionExecutedMaxConvergenceKmPerMy,
		Stats.GeometricCollisionExecutedAccumulatedPenetrationKm,
		Stats.GeometricCollisionExecutedObservationCount,
		Stats.GeometricCollisionExecutedDonorTerraneLocalityLimitedCount,
		Stats.GeometricCollisionRepeatedCandidatePlateA,
		Stats.GeometricCollisionRepeatedCandidatePlateB,
		Stats.GeometricCollisionRepeatedCandidateObservationCount,
		Stats.GeometricCollisionRepeatedCandidateOverlapSampleCount,
		Stats.GeometricCollisionRepeatedCandidateTerraneEstimate,
		Stats.GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm,
		Stats.GeometricCollisionRepeatedCandidateEffectiveConvergenceKmPerMy,
		Stats.GeometricCollisionRepeatedCandidateMeanConvergenceKmPerMy,
		Stats.GeometricCollisionRepeatedCandidateMaxConvergenceKmPerMy,
		Stats.GeometricCollisionRepeatedCandidateMeanOverlapDepthKm,
		Stats.GeometricCollisionRepeatedCandidateMaxOverlapDepthKm,
		Stats.GeometricCollisionDonorLocalityClampKm,
		Stats.GeometricCollisionInfluenceRadiusScale,
		Stats.GeometricCollisionPersistentPenetrationThresholdKm,
		Stats.CollisionRepeatedPairCount,
		Stats.CollisionRepeatedPairWithinCooldownCount,
		Stats.CollisionCount,
		Stats.RiftCount,
		Stats.Interval,
		GetResampleTriggerReasonName(TriggerReason),
		GetResampleOwnershipModeName(OwnershipMode));
	UE_LOG(
		LogTemp,
		Log,
		TEXT("[ResampleFullResolutionLossMetrics R=%d Step=%d] former_continental_full_resolution_non_continental_final_count=%d full_resolution_same_plate_non_continental_final_count=%d full_resolution_changed_plate_non_continental_final_count=%d full_resolution_collision_followup_same_plate_non_continental_count=%d full_resolution_collision_followup_changed_plate_non_continental_count=%d full_resolution_rift_followup_same_plate_non_continental_count=%d full_resolution_rift_followup_changed_plate_non_continental_count=%d full_resolution_other_trigger_same_plate_non_continental_count=%d full_resolution_other_trigger_changed_plate_non_continental_count=%d full_resolution_same_plate_cw_retained_count=%d full_resolution_same_plate_cw_threshold_crossing_prevented_count=%d full_resolution_collision_followup_same_plate_cw_retained_count=%d full_resolution_rift_followup_same_plate_cw_retained_count=%d full_resolution_other_trigger_same_plate_cw_retained_count=%d trigger_reason=%s ownership_mode=%s"),
		ResampleIndex,
		CurrentStep,
		Stats.FormerContinentalFullResolutionNonContinentalFinalCount,
		Stats.FullResolutionSamePlateNonContinentalFinalCount,
		Stats.FullResolutionChangedPlateNonContinentalFinalCount,
		Stats.FullResolutionCollisionFollowupSamePlateNonContinentalCount,
		Stats.FullResolutionCollisionFollowupChangedPlateNonContinentalCount,
		Stats.FullResolutionRiftFollowupSamePlateNonContinentalCount,
		Stats.FullResolutionRiftFollowupChangedPlateNonContinentalCount,
		Stats.FullResolutionOtherTriggerSamePlateNonContinentalCount,
		Stats.FullResolutionOtherTriggerChangedPlateNonContinentalCount,
		Stats.FullResolutionSamePlateCWRetainedCount,
		Stats.FullResolutionSamePlateCWThresholdCrossingPreventedCount,
		Stats.FullResolutionCollisionFollowupSamePlateCWRetainedCount,
		Stats.FullResolutionRiftFollowupSamePlateCWRetainedCount,
		Stats.FullResolutionOtherTriggerSamePlateCWRetainedCount,
		GetResampleTriggerReasonName(TriggerReason),
		GetResampleOwnershipModeName(OwnershipMode));
	UE_LOG(
		LogTemp,
		Log,
		TEXT("[PreserveFallbackRetention R=%d Step=%d] fallback_same_plate_recontained_count=%d fallback_same_plate_retained_count=%d fallback_changed_owner_count=%d fallback_gap_count=%d fallback_divergent_oceanization_count=%d fallback_continental_loss_count_after_fallback=%d"),
		ResampleIndex,
		CurrentStep,
		Stats.PreserveOwnershipFallbackSamePlateRecontainedCount,
		Stats.PreserveOwnershipFallbackSamePlateRetainedCount,
		Stats.PreserveOwnershipFallbackChangedOwnerCount,
		Stats.PreserveOwnershipFallbackGapCount,
		Stats.PreserveOwnershipFallbackDivergentOceanizationCount,
		Stats.PreserveOwnershipContinentalLossCountAfterFallback);
	const FString PreserveFallbackLossPairsString = JoinOwnerTransitionCountsForLog(
		Stats.PreserveOwnershipFallbackLossPairPreviousPlateIds,
		Stats.PreserveOwnershipFallbackLossPairFinalPlateIds,
		Stats.PreserveOwnershipFallbackLossPairCounts);
	UE_LOG(
		LogTemp,
		Log,
		TEXT("[PreserveBoundaryPolicyAudit R=%d Step=%d] previous_owner_hysteresis_application_count=%d preserved_strongly_continental_boundary_sample_count=%d changed_owner_non_gap_loss_count=%d divergent_loss_count=%d non_divergent_projection_loss_count=%d non_divergent_fallback_oceanized_loss_count=%d fallback_loss_ge_090_count=%d fallback_loss_ge_075_count=%d fallback_loss_ge_050_count=%d top_changed_owner_loss_pairs=(%s)"),
		ResampleIndex,
		CurrentStep,
		Stats.PreserveOwnershipPreviousOwnerHysteresisApplicationCount,
		Stats.PreserveOwnershipStronglyContinentalBoundarySavedCount,
		Stats.PreserveOwnershipFallbackChangedOwnerNonGapLossCount,
		Stats.PreserveOwnershipFallbackDivergentLossCount,
		Stats.PreserveOwnershipFallbackNonDivergentProjectionLossCount,
		Stats.PreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount,
		Stats.PreserveOwnershipFallbackStrongLossGE090Count,
		Stats.PreserveOwnershipFallbackStrongLossGE075Count,
		Stats.PreserveOwnershipFallbackStrongLossGE050Count,
		*PreserveFallbackLossPairsString);
	UE_LOG(
		LogTemp,
		Log,
		TEXT("[ResampleTopologyAttribution R=%d Step=%d] trigger_reason=%s ownership_mode=%s global_max_components_before=%d global_max_components_after=%d primary_affected_plate_id=%d primary_components_before=%d primary_components_after=%d secondary_affected_plate_id=%d secondary_components_before=%d secondary_components_after=%d"),
		ResampleIndex,
		CurrentStep,
		GetResampleTriggerReasonName(TriggerReason),
		GetResampleOwnershipModeName(OwnershipMode),
		Stats.TopologyGlobalMaxComponentsBefore,
		Stats.TopologyGlobalMaxComponentsAfter,
		Stats.TopologyPrimaryAffectedPlateId,
		Stats.TopologyPrimaryAffectedPlateComponentsBefore,
		Stats.TopologyPrimaryAffectedPlateComponentsAfter,
		Stats.TopologySecondaryAffectedPlateId,
		Stats.TopologySecondaryAffectedPlateComponentsBefore,
		Stats.TopologySecondaryAffectedPlateComponentsAfter);
	if (TriggerReason == EResampleTriggerReason::RiftFollowup)
	{
		const FString RiftChildPlateIdsString = JoinIntArrayForLog(Stats.RiftChildPlateIds);
		const FString RiftChildAnchorSampleIndicesString =
			JoinIntArrayForLog(Stats.RiftChildAnchorSampleIndices);
		const FString RiftLocalizationRestoredPlateIdsString =
			JoinIntArrayForLog(Stats.RiftLocalizedRestoredPreviousPlateIds);
		const FString RiftLocalizationRestoredPlateCountsString =
			JoinIntArrayForLog(Stats.RiftLocalizedRestoredPreviousPlateCounts);
		const FString RiftChildComponentCountsBeforeSuppressionString =
			JoinIntArrayForLog(Stats.RiftChildComponentCountsBeforeSuppression);
		const FString RiftChildComponentCountsAfterSuppressionString =
			JoinIntArrayForLog(Stats.RiftChildComponentCountsAfterSuppression);
		const FString RiftChildComponentCountsBeforeString =
			JoinIntArrayForLog(Stats.RiftChildComponentCountsBefore);
		const FString RiftChildComponentCountsAfterString =
			JoinIntArrayForLog(Stats.RiftChildComponentCountsAfter);
		const FString RiftRecipientPlateIdsString =
			JoinIntArrayForLog(Stats.RiftStrayFragmentRecipientPlateIds);
		const FString RiftRecipientTypeCodesString =
			JoinRiftRecipientTypeCodesForLog(Stats.RiftStrayFragmentRecipientTypeCodes);
		const FString RiftSiblingEdgeCountsString =
			JoinIntArrayForLog(Stats.RiftStrayFragmentSiblingEdgeCounts);
		const FString RiftRecipientFragmentSizesString =
			JoinIntArrayForLog(Stats.RiftStrayFragmentSizes);
		const FString RiftRecipientComponentsBeforeString =
			JoinIntArrayForLog(Stats.RiftStrayFragmentRecipientComponentsBefore);
		const FString RiftRecipientComponentsAfterString =
			JoinIntArrayForLog(Stats.RiftStrayFragmentRecipientComponentsAfter);
		const FString RiftPositiveGrowthPlateIdsString =
			JoinIntArrayForLog(Stats.RiftPositiveGrowthPlateIds);
		const FString RiftPositiveGrowthTypeCodesString =
			JoinRiftRecipientTypeCodesForLog(Stats.RiftPositiveGrowthPlateTypeCodes);
		const FString RiftPositiveGrowthBeforeString =
			JoinIntArrayForLog(Stats.RiftPositiveGrowthPlateComponentsBefore);
		const FString RiftPositiveGrowthAfterString =
			JoinIntArrayForLog(Stats.RiftPositiveGrowthPlateComponentsAfter);
		const FString RiftPositiveGrowthLargestFragmentSizesString =
			JoinIntArrayForLog(Stats.RiftPositiveGrowthPlateLargestFragmentSizes);
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[RiftFollowupLocalization R=%d Step=%d] parent_plate_id=%d child_plate_ids=(%s) restored_non_child_samples=%d restored_gap_samples=%d restored_previous_plate_ids=(%s) restored_previous_plate_counts=(%s)"),
			ResampleIndex,
			CurrentStep,
			Stats.RiftParentPlateId,
			*RiftChildPlateIdsString,
			Stats.RiftLocalizedNonChildSampleRestoreCount,
			Stats.RiftLocalizedGapSampleRestoreCount,
			*RiftLocalizationRestoredPlateIdsString,
			*RiftLocalizationRestoredPlateCountsString);
			UE_LOG(
				LogTemp,
				Log,
				TEXT("[RiftFollowupCWRollback R=%d Step=%d] interpolation_created_gains=%d localized_cw_restored_count=%d localized_cw_phantom_prevented_count=%d localized_cw_continental_prevented_count=%d final_owner_mismatch_gain_count_after_localization=%d"),
				ResampleIndex,
			CurrentStep,
			Stats.RiftInterpolationCreatedGainCount,
			Stats.RiftLocalizedCWRestoredCount,
				Stats.RiftLocalizedCWPhantomPreventedCount,
				Stats.RiftLocalizedCWContinentalPreventedCount,
				Stats.RiftFinalOwnerMismatchGainCountAfterLocalization);
			UE_LOG(
				LogTemp,
				Log,
				TEXT("[RiftFollowupCWReconciliation R=%d Step=%d] final_owner_cw_reconciled_count=%d mismatch_gain_count_before_reconciliation=%d mismatch_gain_count_after_reconciliation=%d mismatch_continental_prevented_count=%d same_owner_child_interpolation_gain_count_before_reconciliation=%d same_owner_child_interpolation_gain_count_after_reconciliation=%d final_gain_started_below_025_count=%d final_gain_started_below_040_count=%d final_gain_started_below_050_count=%d"),
				ResampleIndex,
				CurrentStep,
				Stats.RiftFinalOwnerCWReconciledCount,
				Stats.RiftFinalOwnerMismatchGainCountBeforeReconciliation,
				Stats.RiftFinalOwnerMismatchGainCountAfterReconciliation,
				Stats.RiftFinalOwnerMismatchContinentalPreventedCount,
				Stats.RiftSameOwnerChildInterpolationGainCountBeforeReconciliation,
				Stats.RiftSameOwnerChildInterpolationGainCountAfterReconciliation,
				Stats.RiftFinalGainStartedBelow025Count,
				Stats.RiftFinalGainStartedBelow040Count,
				Stats.RiftFinalGainStartedBelow050Count);
				UE_LOG(
					LogTemp,
					Log,
				TEXT("[RiftChildCoherence R=%d Step=%d] child_plate_ids=(%s) child_anchor_samples=(%s) child_components_before_suppression=(%s) child_components_after_suppression=(%s) stray_fragments_detected=%d stray_fragments_reassigned=%d largest_stray_fragment_size=%d reassigned_to_sibling=%d reassigned_to_other_neighbor=%d"),
			ResampleIndex,
			CurrentStep,
			*RiftChildPlateIdsString,
			*RiftChildAnchorSampleIndicesString,
			*RiftChildComponentCountsBeforeSuppressionString,
			*RiftChildComponentCountsAfterSuppressionString,
			Stats.RiftChildStrayFragmentDetectedCount,
			Stats.RiftChildStrayFragmentReassignedCount,
			Stats.RiftLargestStrayChildFragmentSize,
			Stats.RiftStrayChildFragmentReassignedToSiblingCount,
			Stats.RiftStrayChildFragmentReassignedToOtherNeighborCount);
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[RiftRecipientCoherence R=%d Step=%d] recipient_plate_ids=(%s) recipient_types=(%s) sibling_edge_counts=(%s) fragment_sizes=(%s) recipient_components_before=(%s) recipient_components_after=(%s) reassigned_by_adjacent_sibling=%d reassigned_by_zero_adjacency_sibling_fallback=%d reassigned_to_non_child_fallback=%d zero_sibling_adjacency_count=%d positive_sibling_adjacency_count=%d candidate_destinations_considered=%d incoherence_rejections=%d incoherent_forced_assignments=%d largest_fragment_causing_recipient_growth=%d positive_growth_plate_ids=(%s) positive_growth_plate_types=(%s) positive_growth_components_before=(%s) positive_growth_components_after=(%s) positive_growth_largest_fragment_sizes=(%s)"),
			ResampleIndex,
			CurrentStep,
			*RiftRecipientPlateIdsString,
			*RiftRecipientTypeCodesString,
			*RiftSiblingEdgeCountsString,
			*RiftRecipientFragmentSizesString,
			*RiftRecipientComponentsBeforeString,
			*RiftRecipientComponentsAfterString,
			Stats.RiftStrayFragmentReassignedByAdjacentSiblingCount,
			Stats.RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount,
			Stats.RiftStrayFragmentForcedNonChildAssignmentCount,
			Stats.RiftStrayFragmentZeroSiblingAdjacencyCount,
			Stats.RiftStrayFragmentPositiveSiblingAdjacencyCount,
			Stats.RiftStrayFragmentRecipientCandidateConsideredCount,
			Stats.RiftStrayFragmentRecipientIncoherenceRejectedCount,
			Stats.RiftStrayFragmentIncoherentForcedAssignmentCount,
			Stats.RiftLargestFragmentCausingRecipientGrowthSize,
			*RiftPositiveGrowthPlateIdsString,
			*RiftPositiveGrowthTypeCodesString,
			*RiftPositiveGrowthBeforeString,
			*RiftPositiveGrowthAfterString,
			*RiftPositiveGrowthLargestFragmentSizesString);
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[RiftTopologyAttribution R=%d Step=%d] parent_plate_id=%d child_count=%d child_plate_ids=(%s) global_max_components_before=%d global_max_components_after=%d topology_component_delta=%d parent_components_before=%d parent_components_after=%d affected_plate_count_before=%d affected_plate_count_after=%d child_components_before=(%s) child_components_after=(%s) min_child_samples=%d max_child_samples=%d mean_child_samples=%.2f"),
			ResampleIndex,
			CurrentStep,
			Stats.RiftParentPlateId,
			Stats.RiftChildCount,
			*RiftChildPlateIdsString,
			Stats.TopologyGlobalMaxComponentsBefore,
			Stats.TopologyGlobalMaxComponentsAfter,
			FMath::Max(
				0,
				Stats.TopologyGlobalMaxComponentsAfter - Stats.TopologyGlobalMaxComponentsBefore),
			Stats.RiftParentComponentsBefore,
			Stats.RiftParentComponentsAfter,
			Stats.RiftAffectedPlateCountBefore,
			Stats.RiftAffectedPlateCountAfter,
			*RiftChildComponentCountsBeforeString,
			*RiftChildComponentCountsAfterString,
			Stats.RiftMinChildSampleCount,
			Stats.RiftMaxChildSampleCount,
			Stats.RiftMeanChildSampleCount);
	}
	if (Stats.CollisionCount > 0 && Stats.bUsedGeometricCollisionExecution)
	{
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[CollisionReceiverContiguity Step=%d] receiver_plate=%d donor_plate=%d transferred_terrane_size=%d receiver_components_before=%d receiver_components_after=%d contiguous_with_existing=%d disconnected_fragments_created=%d largest_new_disconnected_fragment_size=%d"),
			CurrentStep,
			Stats.CollisionOverridingPlateId,
			Stats.CollisionSubductingPlateId,
			Stats.CollisionTerraneSampleCount,
			Stats.CollisionReceiverComponentsBefore,
			Stats.CollisionReceiverComponentsAfter,
			Stats.bCollisionReceiverTransferContiguousWithExistingTerritory ? 1 : 0,
			Stats.CollisionReceiverDisconnectedFragmentCount,
			Stats.CollisionReceiverLargestNewDisconnectedFragmentSize);
	}
	UE_LOG(
		LogTemp,
		Log,
		TEXT("[ResamplePhaseTiming R=%d Step=%d] soup_build_ms=%.3f ownership_query_ms=%.3f interpolation_ms=%.3f gap_resolution_ms=%.3f overlap_classification_ms=%.3f repartition_ms=%.3f collision_ms=%.3f subduction_distance_field_ms=%.3f slab_pull_ms=%.3f terrane_detection_ms=%.3f rift_ms=%.3f total_ms=%.3f"),
		ResampleIndex,
		CurrentStep,
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
		Stats.RiftMs,
		Stats.TotalMs);

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
