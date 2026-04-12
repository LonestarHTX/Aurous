#include "TectonicPlanetV6.h"

#include "Algo/Sort.h"
#include "Async/ParallelFor.h"
#include "CompGeom/ConvexHull3.h"
#include "Distance/DistPoint3Triangle3.h"
#include "HAL/PlatformTime.h"
#include "ShewchukPredicates.h"
#include "VectorUtil.h"

namespace
{
	constexpr double BoundingCapMargin = 1.0e-6;
	constexpr double TriangleEpsilon = 1.0e-12;
	constexpr double DirectionDegeneracyThreshold = 1.0e-8;
	constexpr double BoundaryNormalVelocityThresholdKmPerMy = 0.5;
	constexpr double BoundaryNormalVelocityRatioThreshold = 0.15;
	constexpr int32 BoundaryStrongNeighborVoteCount = 2;
	constexpr double BoundaryRecoverySlackRadians = 0.01;
	constexpr double DestructiveConvergentThresholdRatio = -0.3;
	constexpr double TrenchElevationKm = -10.0;
	constexpr double ElevationCeilingKm = 10.0;
	constexpr double RidgeElevationKm = -1.0;
	constexpr double ContinentalThicknessKm = 35.0;
	constexpr double OceanicThicknessKm = 7.0;
	constexpr double DeltaTimeMyears = 2.0;
	constexpr double SubductionMaxDistanceRad = 0.283;
	constexpr double SubductionControlDistanceRad = 0.10;
	constexpr double MaxPlateSpeedKmPerMy = 100.0;
	constexpr int32 TinyComponentMaxSizeExclusive = 10;
	constexpr int32 MaxCoherencePasses = 4;
	constexpr double CopiedFrontierPreviousPlateRayTieToleranceKm = 25.0;
	constexpr double CopiedFrontierPreviousPlateFitTieTolerance = 0.05;
	constexpr double DestructiveTrackedConvergentThresholdRatio = -0.05;
	constexpr double DestructiveTrackedMinConvergenceSpeedKmPerMy = 0.15;
	constexpr double StructuredGapBlendFloorRadians = 1.0e-4;
	constexpr double DestructiveGapContinuityRecoveryDistanceKm = 150.0;
	constexpr double DestructiveGapContinuityDominanceSlackKm = 75.0;
	constexpr float ActiveBandFieldContinuityElevationDeltaThresholdKm = 1.0f;
	constexpr float ActiveBandSameOwnerTriangleContinuityBlend = 0.15f;
	constexpr float ActiveBandCrossOwnerTriangleContinuityBlend = 0.35f;
	constexpr float ActiveBandSameOwnerTriangleMaxElevationDeltaKm = 0.75f;
	constexpr float ActiveBandCrossOwnerTriangleMaxElevationDeltaKm = 1.0f;
	constexpr float ActiveBandSameOwnerTriangleMaxContinentalWeightDelta = 0.12f;
	constexpr float ActiveBandCrossOwnerTriangleMaxContinentalWeightDelta = 0.18f;
	constexpr float ActiveBandSameOwnerTriangleMaxThicknessDeltaKm = 2.5f;
	constexpr float ActiveBandCrossOwnerTriangleMaxThicknessDeltaKm = 4.0f;
	constexpr float ActiveBandSyntheticContinuityBlend = 0.10f;
	constexpr float ActiveBandSyntheticMaxElevationDeltaKm = 0.60f;
	constexpr float ActiveBandSyntheticMaxContinentalWeightDelta = 0.10f;
	constexpr float ActiveBandSyntheticMaxThicknessDeltaKm = 2.5f;
	constexpr int32 ActiveBandSyntheticLoopBreakMinNeighborSupport = 2;
	constexpr float ContinentalBreadthStrongInteriorPreserveFactor = 0.60f;
	constexpr float ContinentalBreadthModerateInteriorPreserveFactor = 0.32f;
	constexpr int32 ContinentalBreadthStrongInteriorMinCoastDepth = 2;
	constexpr int32 ContinentalBreadthModerateInteriorMinCoastDepth = 1;
	constexpr double ContinentalBreadthStrongNeighborhoodFraction = 0.70;
	constexpr double ContinentalBreadthModerateNeighborhoodFraction = 0.58;
	constexpr double ContinentalBreadthStrongSubaerialNeighborhoodFraction = 0.40;
	constexpr float ContinentalBreadthMinQuietElevationKm = -0.25f;
	constexpr float SubmergedContinentalFringeMaxDepthKm = -2.0f;
	constexpr float PaperSurrogateSelectiveElevationLowKm = 1.5f;
	constexpr float PaperSurrogateSelectiveElevationHighKm = 4.0f;
	constexpr float PaperSurrogateSelectiveElevationHighBlend = 0.60f;
	constexpr double PaperSurrogateDiagnosticScale = 1000000.0;

	enum class EPaperSurrogateElevationBand : uint8
	{
		Low,
		Moderate,
		High,
	};

	EPaperSurrogateElevationBand ClassifyPaperSurrogateElevationBand(const float PreSolveElevationKm)
	{
		if (PreSolveElevationKm <= PaperSurrogateSelectiveElevationLowKm)
		{
			return EPaperSurrogateElevationBand::Low;
		}
		if (PreSolveElevationKm <= PaperSurrogateSelectiveElevationHighKm)
		{
			return EPaperSurrogateElevationBand::Moderate;
		}
		return EPaperSurrogateElevationBand::High;
	}

	float ComputePaperSurrogateSelectiveElevationBlend(const float PreSolveElevationKm)
	{
		if (PreSolveElevationKm <= PaperSurrogateSelectiveElevationLowKm)
		{
			return 1.0f;
		}
		if (PreSolveElevationKm >= PaperSurrogateSelectiveElevationHighKm)
		{
			return PaperSurrogateSelectiveElevationHighBlend;
		}
		const float Alpha = (PreSolveElevationKm - PaperSurrogateSelectiveElevationLowKm) /
			(PaperSurrogateSelectiveElevationHighKm - PaperSurrogateSelectiveElevationLowKm);
		return FMath::Lerp(1.0f, PaperSurrogateSelectiveElevationHighBlend, Alpha);
	}

	double GetPhaseTimingSeconds()
	{
		return FPlatformTime::Seconds();
	}

	void AccumulatePhaseTimingMs(double& InOutPhaseTimingMs, const double PhaseStartTimeSeconds)
	{
		InOutPhaseTimingMs += (FPlatformTime::Seconds() - PhaseStartTimeSeconds) * 1000.0;
	}

	void AccumulatePhaseTimingMicroseconds(TAtomic<int64>& InOutMicroseconds, const double PhaseStartTimeSeconds)
	{
		InOutMicroseconds += static_cast<int64>(
			FMath::RoundToInt64((FPlatformTime::Seconds() - PhaseStartTimeSeconds) * 1000000.0));
	}

	double ConvertPhaseTimingMicrosecondsToMilliseconds(const TAtomic<int64>& Microseconds)
	{
		return static_cast<double>(Microseconds.Load()) / 1000.0;
	}

	struct FV6ContinentalBreadthPreservationSupport
	{
		TArray<uint8> StrongInteriorFlags;
		TArray<uint8> ModerateInteriorFlags;
	};

	FV6ContinentalBreadthPreservationSupport BuildContinentalBreadthPreservationSupport(
		const FTectonicPlanet& Planet,
		const TArray<float>& PreSolveContinentalWeights,
		const TArray<float>& PreSolveElevations)
	{
		FV6ContinentalBreadthPreservationSupport Result;
		const int32 SampleCount = Planet.Samples.Num();
		Result.StrongInteriorFlags.Init(0, SampleCount);
		Result.ModerateInteriorFlags.Init(0, SampleCount);

		if (PreSolveContinentalWeights.Num() != SampleCount)
		{
			return Result;
		}

		TArray<uint8> IsContinental;
		IsContinental.Init(0, SampleCount);
		TArray<int32> CoastDistance;
		CoastDistance.Init(INDEX_NONE, SampleCount);
		TArray<int32> Queue;
		Queue.Reserve(SampleCount);

		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			if (PreSolveContinentalWeights[SampleIndex] < 0.5f)
			{
				continue;
			}

			IsContinental[SampleIndex] = 1;
			bool bTouchesOceanicNeighbor = false;
			if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!PreSolveContinentalWeights.IsValidIndex(NeighborIndex) ||
						PreSolveContinentalWeights[NeighborIndex] < 0.5f)
					{
						bTouchesOceanicNeighbor = true;
						break;
					}
				}
			}

			if (bTouchesOceanicNeighbor)
			{
				CoastDistance[SampleIndex] = 0;
				Queue.Add(SampleIndex);
			}
		}

		for (int32 QueueIndex = 0; QueueIndex < Queue.Num(); ++QueueIndex)
		{
			const int32 SampleIndex = Queue[QueueIndex];
			const int32 NextDistance = CoastDistance[SampleIndex] + 1;
			if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!IsContinental.IsValidIndex(NeighborIndex) ||
					IsContinental[NeighborIndex] == 0 ||
					CoastDistance[NeighborIndex] != INDEX_NONE)
				{
					continue;
				}

				CoastDistance[NeighborIndex] = NextDistance;
				Queue.Add(NeighborIndex);
			}
		}

		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			if (IsContinental[SampleIndex] == 0)
			{
				continue;
			}

			const int32 CoastDepth = CoastDistance[SampleIndex];
			const float PreSolveElevationKm =
				PreSolveElevations.IsValidIndex(SampleIndex) ? PreSolveElevations[SampleIndex] : 0.0f;
			int32 NeighborhoodCount = 1;
			int32 ContinentalNeighborhoodCount = 1;
			int32 SubaerialNeighborhoodCount = PreSolveElevationKm > 0.0f ? 1 : 0;
			if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					++NeighborhoodCount;
					if (IsContinental.IsValidIndex(NeighborIndex) && IsContinental[NeighborIndex] != 0)
					{
						++ContinentalNeighborhoodCount;
						if (PreSolveElevations.IsValidIndex(NeighborIndex) &&
							PreSolveElevations[NeighborIndex] > 0.0f)
						{
							++SubaerialNeighborhoodCount;
						}
					}
				}
			}

			const double ContinentalNeighborhoodFraction =
				static_cast<double>(ContinentalNeighborhoodCount) /
				static_cast<double>(FMath::Max(NeighborhoodCount, 1));
			const double SubaerialNeighborhoodFraction =
				static_cast<double>(SubaerialNeighborhoodCount) /
				static_cast<double>(FMath::Max(NeighborhoodCount, 1));
			const bool bBroadInteriorQuietCandidate =
				PreSolveElevationKm > ContinentalBreadthMinQuietElevationKm;
			const bool bStrongInterior =
				CoastDepth >= ContinentalBreadthStrongInteriorMinCoastDepth &&
				ContinentalNeighborhoodFraction >= ContinentalBreadthStrongNeighborhoodFraction &&
				(PreSolveElevationKm > 0.0f ||
					SubaerialNeighborhoodFraction >= ContinentalBreadthStrongSubaerialNeighborhoodFraction);
			const bool bModerateInterior =
				CoastDepth >= ContinentalBreadthModerateInteriorMinCoastDepth &&
				ContinentalNeighborhoodFraction >= ContinentalBreadthModerateNeighborhoodFraction &&
				bBroadInteriorQuietCandidate;

			Result.StrongInteriorFlags[SampleIndex] = bStrongInterior ? 1 : 0;
			Result.ModerateInteriorFlags[SampleIndex] =
				(bStrongInterior || bModerateInterior) ? 1 : 0;
		}

		return Result;
	}

	FString JoinIntArrayForLog(const TArray<int32>& Values)
	{
		FString Joined;
		for (int32 Index = 0; Index < Values.Num(); ++Index)
		{
			if (Index > 0)
			{
				Joined += TEXT(",");
			}
			Joined += FString::FromInt(Values[Index]);
		}
		return Joined;
	}

	struct FV6PlateQueryGeometry
	{
		FV6PlateQueryGeometry()
			: SoupAdapter(&SoupData)
		{
		}

		FV6PlateQueryGeometry(const FV6PlateQueryGeometry& Other)
			: PlateId(Other.PlateId)
			, BoundingCap(Other.BoundingCap)
			, SoupData(Other.SoupData)
			, LocalTriangleCopiedFrontierFlags(Other.LocalTriangleCopiedFrontierFlags)
			, LocalTriangleDestructiveFlags(Other.LocalTriangleDestructiveFlags)
			, SoupAdapter(&SoupData)
		{
		}

		FV6PlateQueryGeometry(FV6PlateQueryGeometry&& Other) noexcept
			: PlateId(Other.PlateId)
			, BoundingCap(Other.BoundingCap)
			, SoupData(MoveTemp(Other.SoupData))
			, LocalTriangleCopiedFrontierFlags(MoveTemp(Other.LocalTriangleCopiedFrontierFlags))
			, LocalTriangleDestructiveFlags(MoveTemp(Other.LocalTriangleDestructiveFlags))
			, SoupAdapter(&SoupData)
		{
		}

		FV6PlateQueryGeometry& operator=(const FV6PlateQueryGeometry& Other)
		{
			if (this != &Other)
			{
				PlateId = Other.PlateId;
				BoundingCap = Other.BoundingCap;
				SoupData = Other.SoupData;
				LocalTriangleCopiedFrontierFlags = Other.LocalTriangleCopiedFrontierFlags;
				LocalTriangleDestructiveFlags = Other.LocalTriangleDestructiveFlags;
				SoupAdapter = FPlateTriangleSoupAdapter(&SoupData);
				SoupBVH = FPlateTriangleSoupBVH{};
			}

			return *this;
		}

		FV6PlateQueryGeometry& operator=(FV6PlateQueryGeometry&& Other) noexcept
		{
			if (this != &Other)
			{
				PlateId = Other.PlateId;
				BoundingCap = Other.BoundingCap;
				SoupData = MoveTemp(Other.SoupData);
				LocalTriangleCopiedFrontierFlags = MoveTemp(Other.LocalTriangleCopiedFrontierFlags);
				LocalTriangleDestructiveFlags = MoveTemp(Other.LocalTriangleDestructiveFlags);
				SoupAdapter = FPlateTriangleSoupAdapter(&SoupData);
				SoupBVH = FPlateTriangleSoupBVH{};
			}

			return *this;
		}

		int32 PlateId = INDEX_NONE;
		FSphericalBoundingCap BoundingCap;
		FPlateTriangleSoupData SoupData;
		TArray<uint8> LocalTriangleCopiedFrontierFlags;
		TArray<uint8> LocalTriangleDestructiveFlags;
		FPlateTriangleSoupAdapter SoupAdapter;
		FPlateTriangleSoupBVH SoupBVH;
	};

	struct FV6CopiedFrontierDestructiveFilterState
	{
		TArray<uint8> GlobalTriangleDestructiveFlags;
		TArray<uint8> GlobalTriangleDestructiveKinds;
		TArray<int32> GlobalTrianglePreferredContinuationPlateIds;
		int32 GeometryExcludedLocalTriangleCount = 0;
		bool bApplied = false;
	};

	struct FV6CopiedFrontierDestructiveTrackingUpdateStats
	{
		int32 TrackedTriangleCount = 0;
		int32 TrackedSubductionTriangleCount = 0;
		int32 TrackedCollisionTriangleCount = 0;
		int32 NewlySeededTriangleCount = 0;
		int32 PropagatedTriangleCount = 0;
		int32 ExpiredTriangleCount = 0;
		int32 ClearedTriangleCount = 0;
		FTectonicPlanetV6DestructiveTrackingLifecycleStats LifecycleStats;
	};

	struct FV6FrontierPoint
	{
		int32 PlateId = INDEX_NONE;
		int32 LocalVertexIndex = INDEX_NONE;
		FVector3d Position = FVector3d::ZeroVector;
		FCarriedSample CarriedSample;
	};

	struct FV6PlateFrontierPointSet
	{
		int32 PlateId = INDEX_NONE;
		TArray<FV6FrontierPoint> Points;
	};

	struct FV6CopiedFrontierGeometryCounts
	{
		int32 PlateLocalVertexCount = 0;
		int32 PlateLocalTriangleCount = 0;
		int32 CopiedFrontierVertexCount = 0;
		int32 CopiedFrontierTriangleCount = 0;
		int32 CopiedFrontierCarriedSampleCount = 0;
	};

	struct FV6CopiedFrontierQueryStageSetup
	{
		FV6CopiedFrontierGeometryCounts GeometryCounts;
		TArray<FV6PlateQueryGeometry> QueryGeometries;
		TArray<FV6PlateFrontierPointSet> FrontierPointSets;
		TArray<FV6PlateQueryGeometry> UnfilteredComparisonQueryGeometries;
		TMap<int32, int32> QueryGeometryIndexByPlateId;
		TMap<int32, int32> UnfilteredComparisonQueryGeometryIndexByPlateId;
	};

	struct FV6CopiedFrontierTrackingPlateOverlapData
	{
		int32 PlateId = INDEX_NONE;
		TArray<FVector3d> RotatedVertices;
		TArray<UE::Geometry::FIndex3i> LocalTriangles;
		TMultiMap<int32, int32> LocalTriangleIndicesByGlobalTriangle;
	};

	enum class EV6CopiedFrontierDestructiveKind : uint8
	{
		None,
		Subduction,
		Collision,
	};

	enum class EV6ThesisFrontierMeshBuildMode : uint8
	{
		WholeTriangleDuplication,
		PartitionedMixedTriangles,
		ExcludeMixed,
	};

	bool IsCopiedFrontierLikeSolveMode(const ETectonicPlanetV6PeriodicSolveMode SolveMode)
	{
		return SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike;
	}

	bool IsCopiedFrontierProcessSolveMode(const ETectonicPlanetV6PeriodicSolveMode SolveMode)
	{
		return SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike;
	}

	bool IsPartitionedFrontierSolveMode(const ETectonicPlanetV6PeriodicSolveMode SolveMode)
	{
		return SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike;
	}

	bool UsesIntervalDestructivePropagationForSolveMode(const ETectonicPlanetV6PeriodicSolveMode SolveMode)
	{
		return SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike;
	}

	bool UsesPartitionedStructuredGapFillForSolveMode(const ETectonicPlanetV6PeriodicSolveMode SolveMode)
	{
		return SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike ||
			SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike;
	}

	bool UsesStructuredGapFillForCopiedFrontierSolveConfiguration(
		const ETectonicPlanetV6PeriodicSolveMode SolveMode,
		const bool bForceWholeTriangleBoundaryDuplicationForTest,
		const bool bForceExcludeMixedTrianglesForTest = false)
	{
		return UsesPartitionedStructuredGapFillForSolveMode(SolveMode) ||
			bForceWholeTriangleBoundaryDuplicationForTest ||
			bForceExcludeMixedTrianglesForTest;
	}

	EV6ThesisFrontierMeshBuildMode GetThesisFrontierMeshBuildMode(
		const ETectonicPlanetV6PeriodicSolveMode SolveMode)
	{
		return IsPartitionedFrontierSolveMode(SolveMode)
			? EV6ThesisFrontierMeshBuildMode::PartitionedMixedTriangles
			: EV6ThesisFrontierMeshBuildMode::WholeTriangleDuplication;
	}

	EV6ThesisFrontierMeshBuildMode GetEffectiveThesisFrontierMeshBuildMode(
		const ETectonicPlanetV6PeriodicSolveMode SolveMode,
		const bool bForceWholeTriangleBoundaryDuplicationForTest,
		const bool bForceExcludeMixedTrianglesForTest = false)
	{
		if (bForceExcludeMixedTrianglesForTest)
		{
			return EV6ThesisFrontierMeshBuildMode::ExcludeMixed;
		}
		return bForceWholeTriangleBoundaryDuplicationForTest
			? EV6ThesisFrontierMeshBuildMode::WholeTriangleDuplication
			: GetThesisFrontierMeshBuildMode(SolveMode);
	}

	const TCHAR* GetCopiedFrontierModeLabel(const ETectonicPlanetV6PeriodicSolveMode SolveMode)
	{
		switch (SolveMode)
		{
		case ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike:
			return TEXT("ThesisPartitionedFrontierPropagationProcess");
		case ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike:
			return TEXT("ThesisPartitionedFrontierPropagation");
		case ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike:
			return TEXT("ThesisCopiedFrontierProcess");
		case ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike:
			return TEXT("ThesisPartitionedFrontierProcess");
		case ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike:
			return TEXT("ThesisPartitionedFrontier");
		default:
			return TEXT("ThesisCopiedFrontier");
		}
	}

	bool IsSyntheticTransferSourceKind(const ETectonicPlanetV6TransferSourceKind SourceKind)
	{
		return SourceKind == ETectonicPlanetV6TransferSourceKind::StructuredSynthetic ||
			SourceKind == ETectonicPlanetV6TransferSourceKind::OceanicCreation ||
			SourceKind == ETectonicPlanetV6TransferSourceKind::Defaulted;
	}

	void BuildBoundaryBandAndRingDistance(
		const FTectonicPlanet& Planet,
		TArray<uint8>& OutBoundaryBand,
		TArray<int32>& OutRingDistance)
	{
		const int32 SampleCount = Planet.Samples.Num();
		OutBoundaryBand.Init(0, SampleCount);
		OutRingDistance.Init(INDEX_NONE, SampleCount);

		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			const int32 PlateId = Planet.Samples[SampleIndex].PlateId;
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (Planet.Samples.IsValidIndex(NeighborIndex) &&
					Planet.Samples[NeighborIndex].PlateId != PlateId)
				{
					OutBoundaryBand[SampleIndex] = 1;
					OutRingDistance[SampleIndex] = 0;
					break;
				}
			}
		}

		TArray<int32> BfsQueue;
		BfsQueue.Reserve(SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			if (OutBoundaryBand[SampleIndex] != 0)
			{
				BfsQueue.Add(SampleIndex);
			}
		}

		int32 BfsHead = 0;
		while (BfsHead < BfsQueue.Num())
		{
			const int32 Current = BfsQueue[BfsHead++];
			const int32 CurrentDist = OutRingDistance[Current];
			for (const int32 NeighborIndex : Planet.SampleAdjacency[Current])
			{
				if (Planet.Samples.IsValidIndex(NeighborIndex) &&
					OutRingDistance[NeighborIndex] == INDEX_NONE)
				{
					OutRingDistance[NeighborIndex] = CurrentDist + 1;
					BfsQueue.Add(NeighborIndex);
				}
			}
		}
	}

	int32 CountDistinctRecoveryCandidatePlates(const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates)
	{
		TSet<int32> UniquePlateIds;
		for (const FTectonicPlanetV6RecoveryCandidate& RecoveryCandidate : RecoveryCandidates)
		{
			if (RecoveryCandidate.PlateId != INDEX_NONE)
			{
				UniquePlateIds.Add(RecoveryCandidate.PlateId);
			}
		}
		return UniquePlateIds.Num();
	}

	struct FV6CopiedFrontierActiveCollisionPair
	{
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		TSet<int32> SampleIndices;
	};

	void CollectBoundaryTrianglePlateIds(
		const int32 VertexPlateIdA,
		const int32 VertexPlateIdB,
		const int32 VertexPlateIdC,
		TArray<int32>& OutPlateIds);

	void BuildV6CopiedFrontierQueryGeometries(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		TArray<FV6PlateQueryGeometry>& OutQueryGeometries,
		const FV6CopiedFrontierDestructiveFilterState* DestructiveFilterState);

	const FTectonicPlanetV6RecoveryCandidate* FindBestRecoveryCandidateForPlate(
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const int32 PlateId);

	bool FindNearestMemberSample(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		const FVector3d& QueryPoint,
		int32& OutCanonicalSampleIndex,
		double& OutGeodesicDistanceRadians);

	struct FV6CoherenceComponent
	{
		int32 PlateId = INDEX_NONE;
		int32 MinSampleIndex = TNumericLimits<int32>::Max();
		TArray<int32> Samples;
		TMap<int32, int32> NeighborEdgeCounts;
	};

	struct FV6AssignmentComponent
	{
		int32 PlateId = INDEX_NONE;
		int32 MinSampleIndex = TNumericLimits<int32>::Max();
		TArray<int32> Samples;
	};

	struct FV6BoundaryNeighborSummary
	{
		int32 PlateId = INDEX_NONE;
		int32 VoteCount = 0;
		FVector3d TangentSum = FVector3d::ZeroVector;
	};

	enum class EV6BoundaryMotionClass : uint8
	{
		None,
		Weak,
		Divergent,
		Convergent,
	};

	uint64 MakeOrderedPlatePairKey(int32 PlateA, int32 PlateB);

	void BuildBoundaryMotionSamplesForPlateIds(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		int32 SampleIndex,
		const TArray<int32>& RelevantPlateIds,
		TArray<FTectonicPlanetV6BoundaryMotionSample>& OutMotionSamples);

	EV6BoundaryMotionClass ClassifyBoundaryMotion(
		const FVector3d& QueryPoint,
		int32 PrimaryPlateId,
		int32 SecondaryPlateId,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
		double& OutRelativeNormalVelocity);

	const FPlate* FindPlateById(const FTectonicPlanet& Planet, int32 PlateId);
	bool IsOverridingPlateLocal(const FTectonicPlanet& Planet, int32 CandidatePlateId, int32 OtherPlateId);
	double ComputeGeodesicDistance(const FVector3d& A, const FVector3d& B);

	struct FV9Phase1ActivePairContext
	{
		bool bValid = false;
		int32 PrimaryPlateId = INDEX_NONE;
		int32 SecondaryPlateId = INDEX_NONE;
		int32 PrimaryVoteCount = 0;
		int32 SecondaryVoteCount = 0;
		int32 ForeignPlateCount = 0;
		double AbsRelativeNormalVelocity = 0.0;
		ETectonicPlanetV6ActiveZoneCause Cause = ETectonicPlanetV6ActiveZoneCause::None;
	};

	struct FV9Phase1ActivePairAggregate
	{
		int32 SampleCount = 0;
		int32 DivergenceCount = 0;
		int32 ConvergentSubductionCount = 0;
		int32 CollisionContactCount = 0;
		int32 RiftCount = 0;
		int32 MaxSecondaryVoteCount = 0;
		double SumAbsRelativeNormalVelocity = 0.0;
	};

	struct FV9Phase1PersistentActivePairState
	{
		int32 RemainingSolveIntervals = 0;
		ETectonicPlanetV6ActiveZoneCause Cause = ETectonicPlanetV6ActiveZoneCause::None;
		bool bFreshSeed = false;
	};

	struct FV9CollisionShadowObservedPairAggregate
	{
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		int32 SupportSampleCount = 0;
		int32 BoundarySampleCount = 0;
		int32 SubductionSampleCount = 0;
		int32 CollisionSampleCount = 0;
		int32 ContinentalSupportPlateACount = 0;
		int32 ContinentalSupportPlateBCount = 0;
		int32 ContinentalQualifiedSampleCount = 0;
		double SumConvergenceKmPerMy = 0.0;
		double MaxConvergenceKmPerMy = 0.0;
		TSet<int32> SupportTriangles;
		TSet<int32> SubductionTriangles;
		TSet<int32> CollisionTriangles;
		TArray<int32> SampleIndices;
	};

	bool HasContinentalSupportForPlateAtSample(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		const TArray<float>& ContinentalWeights,
		const int32 SampleIndex,
		const int32 PlateId)
	{
		if (PlateId == INDEX_NONE ||
			!Planet.Samples.IsValidIndex(SampleIndex) ||
			!PlateIds.IsValidIndex(SampleIndex) ||
			!ContinentalWeights.IsValidIndex(SampleIndex))
		{
			return false;
		}

		if (PlateIds[SampleIndex] == PlateId && ContinentalWeights[SampleIndex] >= 0.5f)
		{
			return true;
		}

		if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			return false;
		}

		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!PlateIds.IsValidIndex(NeighborIndex) ||
				!ContinentalWeights.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			if (PlateIds[NeighborIndex] == PlateId && ContinentalWeights[NeighborIndex] >= 0.5f)
			{
				return true;
			}
		}

		return false;
	}

	int32 CountFlaggedConnectedComponents(
		const FTectonicPlanet& Planet,
		const TArray<uint8>& Flags)
	{
		if (Flags.Num() != Planet.Samples.Num())
		{
			return 0;
		}

		TArray<uint8> Visited;
		Visited.Init(0, Flags.Num());
		int32 ComponentCount = 0;
		TArray<int32, TInlineAllocator<256>> Stack;
		for (int32 SampleIndex = 0; SampleIndex < Flags.Num(); ++SampleIndex)
		{
			if (Flags[SampleIndex] == 0 || Visited[SampleIndex] != 0)
			{
				continue;
			}

			++ComponentCount;
			Stack.Reset();
			Stack.Add(SampleIndex);
			Visited[SampleIndex] = 1;
			while (!Stack.IsEmpty())
			{
				const int32 Current = Stack.Pop(EAllowShrinking::No);
				if (!Planet.SampleAdjacency.IsValidIndex(Current))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[Current])
				{
					if (!Flags.IsValidIndex(NeighborIndex) ||
						Flags[NeighborIndex] == 0 ||
						Visited[NeighborIndex] != 0)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}
		}

		return ComponentCount;
	}

	void UpdateV9CollisionShadowState(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		const TArray<float>& ContinentalWeights,
		const TArray<TArray<int32>>& SampleToAdjacentTriangles,
		const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples,
		TMap<uint64, FGeometricCollisionPairRecurrenceState>& InOutPairRecurrenceByKey,
		TArray<uint8>& OutTrackedFlags,
		TArray<uint8>& OutQualifiedFlags,
		TArray<uint8>& OutPersistenceMask,
		FTectonicPlanetV6CollisionShadowDiagnostic& OutDiagnostic)
	{
		constexpr int32 MinCollisionCandidateSupportSamples = 3;
		constexpr int32 MinCollisionContinentalSupportSamples = 3;
		constexpr int32 MinCollisionObservationCount = 2;
		constexpr int32 MaxTopPairs = 5;

		OutDiagnostic = FTectonicPlanetV6CollisionShadowDiagnostic{};
		OutTrackedFlags.Init(0, Planet.Samples.Num());
		OutQualifiedFlags.Init(0, Planet.Samples.Num());
		OutPersistenceMask.Init(0, Planet.Samples.Num());

		if (ResolvedSamples.Num() != Planet.Samples.Num() ||
			PlateIds.Num() != Planet.Samples.Num() ||
			ContinentalWeights.Num() != Planet.Samples.Num())
		{
			InOutPairRecurrenceByKey.Reset();
			return;
		}

		TMap<uint64, FV9CollisionShadowObservedPairAggregate> ObservedPairs;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const FTectonicPlanetV6ResolvedSample& Resolved = ResolvedSamples[SampleIndex];
			if (!Resolved.bActiveZoneSample ||
				(Resolved.ActiveZoneCause != ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction &&
					Resolved.ActiveZoneCause != ETectonicPlanetV6ActiveZoneCause::CollisionContact) ||
				Resolved.ActiveZonePrimaryPlateId == INDEX_NONE ||
				Resolved.ActiveZoneSecondaryPlateId == INDEX_NONE)
			{
				continue;
			}

			const uint64 PairKey = MakeOrderedPlatePairKey(
				Resolved.ActiveZonePrimaryPlateId,
				Resolved.ActiveZoneSecondaryPlateId);
			FV9CollisionShadowObservedPairAggregate& Aggregate = ObservedPairs.FindOrAdd(PairKey);
			Aggregate.PlateA = FMath::Min(Resolved.ActiveZonePrimaryPlateId, Resolved.ActiveZoneSecondaryPlateId);
			Aggregate.PlateB = FMath::Max(Resolved.ActiveZonePrimaryPlateId, Resolved.ActiveZoneSecondaryPlateId);
			++Aggregate.SupportSampleCount;
			Aggregate.BoundarySampleCount += Planet.Samples[SampleIndex].bIsBoundary ? 1 : 0;
			if (Resolved.ActiveZoneCause == ETectonicPlanetV6ActiveZoneCause::CollisionContact)
			{
				++Aggregate.CollisionSampleCount;
			}
			else
			{
				++Aggregate.SubductionSampleCount;
			}
			Aggregate.SampleIndices.Add(SampleIndex);

			TArray<int32> RelevantPlateIds;
			RelevantPlateIds.Add(Resolved.ActiveZonePrimaryPlateId);
			RelevantPlateIds.Add(Resolved.ActiveZoneSecondaryPlateId);
			TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
			BuildBoundaryMotionSamplesForPlateIds(
				Planet,
				PlateIds,
				SampleIndex,
				RelevantPlateIds,
				MotionSamples);
			double RelativeNormalVelocity = 0.0;
			const EV6BoundaryMotionClass MotionClass = ClassifyBoundaryMotion(
				Planet.Samples[SampleIndex].Position,
				Resolved.ActiveZonePrimaryPlateId,
				Resolved.ActiveZoneSecondaryPlateId,
				MotionSamples,
				RelativeNormalVelocity);
			if (MotionClass == EV6BoundaryMotionClass::Convergent)
			{
				const double ConvergenceKmPerMy = FMath::Abs(RelativeNormalVelocity);
				Aggregate.SumConvergenceKmPerMy += ConvergenceKmPerMy;
				Aggregate.MaxConvergenceKmPerMy =
					FMath::Max(Aggregate.MaxConvergenceKmPerMy, ConvergenceKmPerMy);
			}

			const bool bPlateAContinental =
				HasContinentalSupportForPlateAtSample(
					Planet,
					PlateIds,
					ContinentalWeights,
					SampleIndex,
					Aggregate.PlateA);
			const bool bPlateBContinental =
				HasContinentalSupportForPlateAtSample(
					Planet,
					PlateIds,
					ContinentalWeights,
					SampleIndex,
					Aggregate.PlateB);
			Aggregate.ContinentalSupportPlateACount += bPlateAContinental ? 1 : 0;
			Aggregate.ContinentalSupportPlateBCount += bPlateBContinental ? 1 : 0;
			Aggregate.ContinentalQualifiedSampleCount +=
				(bPlateAContinental && bPlateBContinental) ? 1 : 0;

			if (SampleToAdjacentTriangles.IsValidIndex(SampleIndex))
			{
				for (const int32 TriangleIndex : SampleToAdjacentTriangles[SampleIndex])
				{
					if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
					{
						continue;
					}

					const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
					if (!PlateIds.IsValidIndex(Triangle.X) ||
						!PlateIds.IsValidIndex(Triangle.Y) ||
						!PlateIds.IsValidIndex(Triangle.Z))
					{
						continue;
					}

					const int32 TrianglePlateIds[3] = {
						PlateIds[Triangle.X],
						PlateIds[Triangle.Y],
						PlateIds[Triangle.Z]
					};
					bool bPairLocalTriangle = true;
					for (const int32 TrianglePlateId : TrianglePlateIds)
					{
						if (TrianglePlateId != Aggregate.PlateA && TrianglePlateId != Aggregate.PlateB)
						{
							bPairLocalTriangle = false;
							break;
						}
					}
					if (!bPairLocalTriangle)
					{
						continue;
					}

					Aggregate.SupportTriangles.Add(TriangleIndex);
					const auto IsVertexPairMatch = [&](const int32 VertexSampleIndex)
					{
						return ResolvedSamples.IsValidIndex(VertexSampleIndex) &&
							MakeOrderedPlatePairKey(
								ResolvedSamples[VertexSampleIndex].ActiveZonePrimaryPlateId,
								ResolvedSamples[VertexSampleIndex].ActiveZoneSecondaryPlateId) == PairKey;
					};
					const auto IsVertexCollision = [&](const int32 VertexSampleIndex)
					{
						return IsVertexPairMatch(VertexSampleIndex) &&
							ResolvedSamples[VertexSampleIndex].ActiveZoneCause ==
								ETectonicPlanetV6ActiveZoneCause::CollisionContact;
					};

					if (IsVertexCollision(Triangle.X) ||
						IsVertexCollision(Triangle.Y) ||
						IsVertexCollision(Triangle.Z))
					{
						Aggregate.CollisionTriangles.Add(TriangleIndex);
					}
					else
					{
						Aggregate.SubductionTriangles.Add(TriangleIndex);
					}
				}
			}
		}

		TSet<uint64> ObservedPairKeys;
		TArray<FTectonicPlanetV6CollisionShadowTopPair> TopPairs;
		TopPairs.Reserve(ObservedPairs.Num());

		for (const TPair<uint64, FV9CollisionShadowObservedPairAggregate>& PairEntry : ObservedPairs)
		{
			const FV9CollisionShadowObservedPairAggregate& Aggregate = PairEntry.Value;
			FGeometricCollisionPairRecurrenceState& Recurrence =
				InOutPairRecurrenceByKey.FindOrAdd(PairEntry.Key);
			if (Recurrence.LastObservedStep != Planet.CurrentStep)
			{
				if (Recurrence.LastObservedStep != INDEX_NONE &&
					Planet.CurrentStep > Recurrence.LastObservedStep &&
					Aggregate.SupportSampleCount > 0)
				{
					const double MeanConvergenceKmPerMy =
						Aggregate.SumConvergenceKmPerMy /
						static_cast<double>(FMath::Max(1, Aggregate.SupportSampleCount));
					const double ElapsedTimeMyears =
						static_cast<double>(Planet.CurrentStep - Recurrence.LastObservedStep) * DeltaTimeMyears;
					Recurrence.AccumulatedPenetrationKm += MeanConvergenceKmPerMy * ElapsedTimeMyears;
				}
				++Recurrence.ObservationCount;
			}

			const double MeanConvergenceKmPerMy =
				Aggregate.SupportSampleCount > 0
					? Aggregate.SumConvergenceKmPerMy /
						static_cast<double>(Aggregate.SupportSampleCount)
					: 0.0;
			Recurrence.LastObservedStep = Planet.CurrentStep;
			Recurrence.LastReceiverPlateId = Aggregate.PlateA;
			Recurrence.LastDonorPlateId = Aggregate.PlateB;
			Recurrence.LastOverlapSampleCount = Aggregate.SupportSampleCount;
			Recurrence.LastTerraneEstimate = Aggregate.CollisionTriangles.Num();
			Recurrence.LastEffectiveConvergenceKmPerMy = MeanConvergenceKmPerMy;
			Recurrence.LastMeanConvergenceMagnitudeKmPerMy = MeanConvergenceKmPerMy;
			Recurrence.LastMaxConvergenceMagnitudeKmPerMy = Aggregate.MaxConvergenceKmPerMy;
			Recurrence.LastMeanOverlapDepthKm = 0.0;
			Recurrence.LastMaxOverlapDepthKm = 0.0;
			ObservedPairKeys.Add(PairEntry.Key);

			OutDiagnostic.TrackedSubductionSampleCount += Aggregate.SubductionSampleCount;
			OutDiagnostic.TrackedSubductionTriangleCount += Aggregate.SubductionTriangles.Num();
			OutDiagnostic.TrackedCollisionSampleCount += Aggregate.CollisionSampleCount;
			OutDiagnostic.TrackedCollisionTriangleCount += Aggregate.CollisionTriangles.Num();

			for (const int32 SampleIndex : Aggregate.SampleIndices)
			{
				OutTrackedFlags[SampleIndex] = 1;
				OutPersistenceMask[SampleIndex] = static_cast<uint8>(FMath::Clamp(Recurrence.ObservationCount, 0, 255));
			}

			const bool bCandidateSupportQualified =
				Aggregate.SupportSampleCount >= MinCollisionCandidateSupportSamples;
			const bool bContinentalQualified =
				Aggregate.ContinentalSupportPlateACount >= MinCollisionContinentalSupportSamples &&
				Aggregate.ContinentalSupportPlateBCount >= MinCollisionContinentalSupportSamples;
			const bool bPersistenceQualified =
				Recurrence.ObservationCount >= MinCollisionObservationCount;
			const bool bThresholdQualified =
				Recurrence.AccumulatedPenetrationKm >= Planet.GeometricCollisionMinPersistentPenetrationKm;

			if (!bCandidateSupportQualified)
			{
				++OutDiagnostic.CandidateRejectedBySupportCount;
				continue;
			}

			++OutDiagnostic.CollisionShadowCandidateCount;
			FTectonicPlanetV6CollisionShadowTopPair& TopPair = TopPairs.AddDefaulted_GetRef();
			TopPair.PlateA = Aggregate.PlateA;
			TopPair.PlateB = Aggregate.PlateB;
			TopPair.ObservationCount = Recurrence.ObservationCount;
			TopPair.AccumulatedPenetrationKm = Recurrence.AccumulatedPenetrationKm;
			TopPair.MeanConvergenceKmPerMy = MeanConvergenceKmPerMy;
			TopPair.MaxConvergenceKmPerMy = Aggregate.MaxConvergenceKmPerMy;
			TopPair.SupportSampleCount = Aggregate.SupportSampleCount;
			TopPair.SupportTriangleCount = Aggregate.SupportTriangles.Num();
			TopPair.ContinentalSupportPlateACount = Aggregate.ContinentalSupportPlateACount;
			TopPair.ContinentalSupportPlateBCount = Aggregate.ContinentalSupportPlateBCount;
			TopPair.ContinentalQualifiedSampleCount = Aggregate.ContinentalQualifiedSampleCount;
			TopPair.SubductionSampleCount = Aggregate.SubductionSampleCount;
			TopPair.CollisionSampleCount = Aggregate.CollisionSampleCount;
			OutDiagnostic.PersistentObservedPairCount += bPersistenceQualified ? 1 : 0;
			OutDiagnostic.ContinentalQualifiedCandidateCount += bContinentalQualified ? 1 : 0;

			if (!bContinentalQualified)
			{
				++OutDiagnostic.CandidateRejectedByContinentalityCount;
			}
			else if (!bPersistenceQualified)
			{
				++OutDiagnostic.CandidateRejectedByPersistenceCount;
			}
			else if (!bThresholdQualified)
			{
				++OutDiagnostic.CandidateRejectedByThresholdCount;
			}
			else
			{
				++OutDiagnostic.CollisionShadowQualifiedCount;
				TopPair.bQualified = true;
				for (const int32 SampleIndex : Aggregate.SampleIndices)
				{
					OutQualifiedFlags[SampleIndex] = 1;
				}
			}
		}

		for (auto It = InOutPairRecurrenceByKey.CreateIterator(); It; ++It)
		{
			if (!ObservedPairKeys.Contains(It.Key()))
			{
				It.RemoveCurrent();
			}
		}

		OutDiagnostic.TrackedConvergentPairCount = ObservedPairs.Num();
		OutDiagnostic.TrackedConvergentRegionCount =
			CountFlaggedConnectedComponents(Planet, OutTrackedFlags);

		TopPairs.Sort([](
			const FTectonicPlanetV6CollisionShadowTopPair& Left,
			const FTectonicPlanetV6CollisionShadowTopPair& Right)
		{
			if (Left.bQualified != Right.bQualified)
			{
				return Left.bQualified;
			}
			if (Left.ObservationCount != Right.ObservationCount)
			{
				return Left.ObservationCount > Right.ObservationCount;
			}
			if (!FMath::IsNearlyEqual(
					Left.AccumulatedPenetrationKm,
					Right.AccumulatedPenetrationKm,
					TriangleEpsilon))
			{
				return Left.AccumulatedPenetrationKm > Right.AccumulatedPenetrationKm;
			}
			if (Left.SupportSampleCount != Right.SupportSampleCount)
			{
				return Left.SupportSampleCount > Right.SupportSampleCount;
			}
			if (!FMath::IsNearlyEqual(
					Left.MaxConvergenceKmPerMy,
					Right.MaxConvergenceKmPerMy,
					TriangleEpsilon))
			{
				return Left.MaxConvergenceKmPerMy > Right.MaxConvergenceKmPerMy;
			}
			if (Left.PlateA != Right.PlateA)
			{
				return Left.PlateA < Right.PlateA;
			}
			return Left.PlateB < Right.PlateB;
		});

		if (TopPairs.Num() > MaxTopPairs)
		{
			TopPairs.SetNum(MaxTopPairs, EAllowShrinking::No);
		}
		OutDiagnostic.TopPairs = MoveTemp(TopPairs);
		if (!OutDiagnostic.TopPairs.IsEmpty())
		{
			const FTectonicPlanetV6CollisionShadowTopPair& BestPair = OutDiagnostic.TopPairs[0];
			OutDiagnostic.BestCandidatePlateA = BestPair.PlateA;
			OutDiagnostic.BestCandidatePlateB = BestPair.PlateB;
			OutDiagnostic.BestCandidateObservationCount = BestPair.ObservationCount;
			OutDiagnostic.BestCandidateAccumulatedPenetrationKm = BestPair.AccumulatedPenetrationKm;
			OutDiagnostic.BestCandidateMeanConvergenceKmPerMy = BestPair.MeanConvergenceKmPerMy;
			OutDiagnostic.BestCandidateMaxConvergenceKmPerMy = BestPair.MaxConvergenceKmPerMy;
			OutDiagnostic.BestCandidateSupportSampleCount = BestPair.SupportSampleCount;
			OutDiagnostic.BestCandidateSupportTriangleCount = BestPair.SupportTriangleCount;
			OutDiagnostic.BestCandidateContinentalQualifiedSampleCount =
				BestPair.ContinentalQualifiedSampleCount;
		}
	}

	void SyncV6CarriedSamplesFromCanonicalIndices(
		FTectonicPlanet& Planet,
		const TArray<int32>& CanonicalSampleIndices);

	void RemoveV6CarriedSampleForCanonicalVertex(
		FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 CanonicalSampleIndex);

	void UpsertV6CarriedSampleForCanonicalVertex(
		FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 CanonicalSampleIndex);

	void RebuildV6MembershipFromCanonical(FTectonicPlanet& Planet);

	void RecomputeV6BoundaryFlagsFromCanonical(FTectonicPlanet& Planet);

	void ResetV9CollisionExecutionLegacyStats(FTectonicPlanet& Planet)
	{
		FResamplingStats& Stats = Planet.LastResamplingStats;
		Stats.CollisionCount = 0;
		Stats.CollisionDeferredCount = 0;
		Stats.CollisionTerraneId = INDEX_NONE;
		Stats.CollisionTerraneSampleCount = 0;
		Stats.CollisionOverridingPlateId = INDEX_NONE;
		Stats.CollisionSubductingPlateId = INDEX_NONE;
		Stats.CollisionSurgeAffectedCount = 0;
		Stats.CollisionCWBoostedCount = 0;
		Stats.CollisionContinentalGainCount = 0;
		Stats.CollisionSurgeRadiusRad = 0.0;
		Stats.CollisionSurgeMeanElevationDelta = 0.0;
	}

	void ExecuteV9CollisionShadowEvent(
		FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples,
		const FTectonicPlanetV6CollisionShadowDiagnostic& ShadowDiagnostic,
		TMap<uint64, int32>& InOutLastExecutedSolveIndexByKey,
		const int32 CurrentSolveIndex,
		TArray<uint8>& OutExecutionMask,
		TArray<uint8>& OutTransferMask,
		TArray<uint8>& InOutCumulativeExecutionMask,
		TArray<uint8>& InOutCumulativeTransferMask,
		TArray<float>& InOutCumulativeElevationDeltaMaskKm,
		TArray<float>& InOutCumulativeContinentalGainMask,
		FTectonicPlanetV6CollisionExecutionDiagnostic& OutDiagnostic,
		int32& InOutCumulativeExecutionCount,
		int32& InOutCumulativeAffectedSampleVisits,
		int32& InOutCumulativeContinentalGainCount,
		int32& InOutCumulativeOwnershipChangeCount,
		int32& InOutCumulativeTransferredSampleVisits,
		int32& InOutCumulativeTransferredContinentalSampleCount,
		double& InOutCumulativeElevationDeltaKm,
		double& InOutCumulativeMaxElevationDeltaKm,
		const bool bEnableEnhancedConsequences,
		const bool bEnableStructuralTransfer,
		const bool bEnableRefinedStructuralTransfer)
	{
		constexpr int32 CollisionExecutionPairCooldownSolves = 3;
		constexpr int32 MinStructuralTransferSamples = 8;
		constexpr int32 MaxStructuralTransferSamples = 40;
		constexpr int32 RefinedStructuralTransferTargetFloor = 12;

		OutDiagnostic = FTectonicPlanetV6CollisionExecutionDiagnostic{};
		OutExecutionMask.Init(0, Planet.Samples.Num());
		OutTransferMask.Init(0, Planet.Samples.Num());
		OutDiagnostic.CumulativeExecutedCollisionCount = InOutCumulativeExecutionCount;
		if (InOutCumulativeExecutionMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeExecutionMask.Init(0, Planet.Samples.Num());
		}
		if (InOutCumulativeTransferMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeTransferMask.Init(0, Planet.Samples.Num());
		}
		if (InOutCumulativeElevationDeltaMaskKm.Num() != Planet.Samples.Num())
		{
			InOutCumulativeElevationDeltaMaskKm.Init(0.0f, Planet.Samples.Num());
		}
		if (InOutCumulativeContinentalGainMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeContinentalGainMask.Init(0.0f, Planet.Samples.Num());
		}
		OutDiagnostic.CumulativeCollisionAffectedSampleVisits = InOutCumulativeAffectedSampleVisits;
		OutDiagnostic.CumulativeCollisionDrivenContinentalGainCount =
			InOutCumulativeContinentalGainCount;
		OutDiagnostic.CumulativeCollisionDrivenOwnershipChangeCount =
			InOutCumulativeOwnershipChangeCount;
		OutDiagnostic.CumulativeCollisionTransferredSampleVisits =
			InOutCumulativeTransferredSampleVisits;
		OutDiagnostic.CumulativeCollisionTransferredContinentalSampleCount =
			InOutCumulativeTransferredContinentalSampleCount;
		OutDiagnostic.CumulativeMeanElevationDeltaKm =
			InOutCumulativeAffectedSampleVisits > 0
				? InOutCumulativeElevationDeltaKm /
					static_cast<double>(InOutCumulativeAffectedSampleVisits)
				: 0.0;
		OutDiagnostic.CumulativeMaxElevationDeltaKm = InOutCumulativeMaxElevationDeltaKm;
		for (const uint8 Value : InOutCumulativeExecutionMask)
		{
			OutDiagnostic.CumulativeCollisionAffectedSampleCount += Value != 0 ? 1 : 0;
		}
		for (const uint8 Value : InOutCumulativeTransferMask)
		{
			OutDiagnostic.CumulativeCollisionTransferredSampleCount += Value != 0 ? 1 : 0;
		}
		ResetV9CollisionExecutionLegacyStats(Planet);

		if (Planet.Samples.IsEmpty() || ResolvedSamples.Num() != Planet.Samples.Num())
		{
			return;
		}

		TArray<FTectonicPlanetV6CollisionShadowTopPair> ExecutionPairs = ShadowDiagnostic.TopPairs;
		ExecutionPairs.Sort([](
			const FTectonicPlanetV6CollisionShadowTopPair& Left,
			const FTectonicPlanetV6CollisionShadowTopPair& Right)
		{
			if (Left.bQualified != Right.bQualified)
			{
				return Left.bQualified;
			}
			if (Left.CollisionSampleCount != Right.CollisionSampleCount)
			{
				return Left.CollisionSampleCount > Right.CollisionSampleCount;
			}
			if (Left.ObservationCount != Right.ObservationCount)
			{
				return Left.ObservationCount > Right.ObservationCount;
			}
			if (!FMath::IsNearlyEqual(
					Left.AccumulatedPenetrationKm,
					Right.AccumulatedPenetrationKm,
					TriangleEpsilon))
			{
				return Left.AccumulatedPenetrationKm > Right.AccumulatedPenetrationKm;
			}
			if (Left.ContinentalQualifiedSampleCount != Right.ContinentalQualifiedSampleCount)
			{
				return Left.ContinentalQualifiedSampleCount > Right.ContinentalQualifiedSampleCount;
			}
			if (Left.SupportSampleCount != Right.SupportSampleCount)
			{
				return Left.SupportSampleCount > Right.SupportSampleCount;
			}
			return Left.PlateA < Right.PlateA ||
				(Left.PlateA == Right.PlateA && Left.PlateB < Right.PlateB);
		});

		int32 QualifiedCount = 0;
		for (const FTectonicPlanetV6CollisionShadowTopPair& Pair : ExecutionPairs)
		{
			QualifiedCount += Pair.bQualified ? 1 : 0;
		}

		const FTectonicPlanetV6CollisionShadowTopPair* SelectedPair = nullptr;
		for (const FTectonicPlanetV6CollisionShadowTopPair& Pair : ExecutionPairs)
		{
			if (!Pair.bQualified)
			{
				continue;
			}

			const uint64 PairKey = MakeOrderedPlatePairKey(Pair.PlateA, Pair.PlateB);
			if (const int32* LastExecutedSolveIndex = InOutLastExecutedSolveIndexByKey.Find(PairKey))
			{
				if ((CurrentSolveIndex - *LastExecutedSolveIndex) <= CollisionExecutionPairCooldownSolves)
				{
					++OutDiagnostic.CooldownSuppressedQualifiedCount;
					continue;
				}
			}

			SelectedPair = &Pair;
			break;
		}

		OutDiagnostic.QualifiedButUnexecutedCount =
			FMath::Max(0, QualifiedCount - (SelectedPair != nullptr ? 1 : 0));
		if (SelectedPair == nullptr)
		{
			return;
		}

		const int32 OverridingPlateId =
			IsOverridingPlateLocal(Planet, SelectedPair->PlateA, SelectedPair->PlateB)
				? SelectedPair->PlateA
				: SelectedPair->PlateB;
		const int32 SubductingPlateId =
			OverridingPlateId == SelectedPair->PlateA ? SelectedPair->PlateB : SelectedPair->PlateA;

		TArray<int32> SeedSampleIndices;
		TArray<int32> CollisionSeedSampleIndices;
		SeedSampleIndices.Reserve(SelectedPair->SupportSampleCount);
		CollisionSeedSampleIndices.Reserve(SelectedPair->CollisionSampleCount);
		FVector3d CenterSum = FVector3d::ZeroVector;
		const uint64 SelectedPairKey = MakeOrderedPlatePairKey(SelectedPair->PlateA, SelectedPair->PlateB);
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!ResolvedSamples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FTectonicPlanetV6ResolvedSample& Resolved = ResolvedSamples[SampleIndex];
			if (!Resolved.bActiveZoneSample ||
				Resolved.ActiveZonePrimaryPlateId == INDEX_NONE ||
				Resolved.ActiveZoneSecondaryPlateId == INDEX_NONE ||
				(Resolved.ActiveZoneCause != ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction &&
					Resolved.ActiveZoneCause != ETectonicPlanetV6ActiveZoneCause::CollisionContact))
			{
				continue;
			}

			if (MakeOrderedPlatePairKey(
					Resolved.ActiveZonePrimaryPlateId,
					Resolved.ActiveZoneSecondaryPlateId) != SelectedPairKey)
			{
				continue;
			}

			SeedSampleIndices.Add(SampleIndex);
			CenterSum += Planet.Samples[SampleIndex].Position;
			if (Resolved.ActiveZoneCause == ETectonicPlanetV6ActiveZoneCause::CollisionContact)
			{
				CollisionSeedSampleIndices.Add(SampleIndex);
			}
		}

		if (SeedSampleIndices.IsEmpty())
		{
			return;
		}

		FCollisionEvent CollisionEvent;
		CollisionEvent.bDetected = true;
		CollisionEvent.OverridingPlateId = OverridingPlateId;
		CollisionEvent.SubductingPlateId = SubductingPlateId;
		CollisionEvent.CollisionCenter = CenterSum.GetSafeNormal();
		CollisionEvent.CollisionSampleIndices =
			!CollisionSeedSampleIndices.IsEmpty() ? CollisionSeedSampleIndices : SeedSampleIndices;
		CollisionEvent.TerraneSampleIndices =
			!CollisionSeedSampleIndices.IsEmpty() ? CollisionSeedSampleIndices : SeedSampleIndices;

		if (CollisionEvent.CollisionCenter.IsNearlyZero())
		{
			CollisionEvent.CollisionCenter =
				Planet.Samples[CollisionEvent.CollisionSampleIndices[0]].Position.GetSafeNormal();
		}

		FVector3d CollisionCenter = CollisionEvent.CollisionCenter.GetSafeNormal();
		const FPlate* OverridingPlate = FindPlateById(Planet, CollisionEvent.OverridingPlateId);
		const FPlate* SubductingPlate = FindPlateById(Planet, CollisionEvent.SubductingPlateId);
		if (OverridingPlate == nullptr ||
			SubductingPlate == nullptr ||
			CollisionCenter.IsNearlyZero())
		{
			return;
		}

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
			((OverridingVelocity - SubductingVelocity).Length() * Planet.PlanetRadiusKm) /
			FMath::Max(DeltaTimeMyears, UE_DOUBLE_SMALL_NUMBER);
		const double AverageSampleAreaSr = (4.0 * PI) / static_cast<double>(Planet.Samples.Num());
		const double Xi =
			FMath::Sqrt(FMath::Max(RelativeSpeedKmPerMy, 0.0) /
				FMath::Max(MaxPlateSpeedKmPerMy, UE_DOUBLE_SMALL_NUMBER)) *
			static_cast<double>(CollisionEvent.TerraneSampleIndices.Num()) *
			AverageSampleAreaSr;
		const double CollisionGlobalDistanceRad = 0.659;
		const double MinCollisionRadiusRad = 0.05;
		const double CollisionCoefficient = 0.013;
		const double ObservationStrength = FMath::Clamp(
			(static_cast<double>(SelectedPair->ObservationCount) - 3.0) / 9.0,
			0.0,
			1.0);
		const double PenetrationStrength = FMath::Clamp(
			(SelectedPair->AccumulatedPenetrationKm - 3000.0) / 9000.0,
			0.0,
			1.0);
		const double ContinentalStrength = FMath::Clamp(
			static_cast<double>(SelectedPair->ContinentalQualifiedSampleCount) / 24.0,
			0.0,
			1.0);
		const double CollisionStrength01 = bEnableEnhancedConsequences
			? FMath::Clamp(
				(0.45 * ObservationStrength) +
				(0.35 * PenetrationStrength) +
				(0.20 * ContinentalStrength),
				0.0,
				1.0)
			: 0.0;
		const int32 EffectiveMassSampleCount = bEnableEnhancedConsequences
			? FMath::Max(
				CollisionEvent.TerraneSampleIndices.Num(),
				SelectedPair->ContinentalQualifiedSampleCount)
			: CollisionEvent.TerraneSampleIndices.Num();
		const double CollisionRadiusScale = bEnableEnhancedConsequences
			? (1.0 + (0.20 * CollisionStrength01))
			: 1.0;
		const double CollisionCoefficientScale = bEnableEnhancedConsequences
			? (1.0 + (0.85 * CollisionStrength01))
			: 1.0;
		const double SurgeRadiusRad = FMath::Min(
			CollisionGlobalDistanceRad,
			FMath::Max(
				(CollisionGlobalDistanceRad * FMath::Min(Xi, 1.0)) * CollisionRadiusScale,
				MinCollisionRadiusRad));

		TArray<int32> AffectedSampleIndices;
		AffectedSampleIndices.Reserve(Planet.Samples.Num() / 16);
		int32 ContinentalGainCount = 0;
		int32 CWBoostedCount = 0;
		double TotalElevationDeltaKm = 0.0;
		double MaxElevationDeltaKm = 0.0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			const double DistanceRad = ComputeGeodesicDistance(
				Sample.Position.GetSafeNormal(),
				CollisionCenter);
			if (DistanceRad >= SurgeRadiusRad)
			{
				continue;
			}

			const double Kernel = FTectonicPlanet::CollisionBiweightKernel(DistanceRad, SurgeRadiusRad);
			if (Kernel <= 0.0)
			{
				continue;
			}

			const float PreviousContinentalWeight = Sample.ContinentalWeight;
			const float PreviousElevation = Sample.Elevation;
			const double ElevationDeltaKm =
				CollisionCoefficient *
				CollisionCoefficientScale *
				(static_cast<double>(EffectiveMassSampleCount) /
					static_cast<double>(Planet.Samples.Num())) *
				Kernel *
				Planet.PlanetRadiusKm;

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
			Sample.Thickness = FMath::Max(Sample.Thickness, static_cast<float>(ContinentalThicknessKm));
			const bool bContinentalGain =
				PreviousContinentalWeight < 0.5f && Sample.ContinentalWeight >= 0.5f;
			ContinentalGainCount += bContinentalGain ? 1 : 0;
			CWBoostedCount += Sample.ContinentalWeight > PreviousContinentalWeight ? 1 : 0;
			const double AppliedElevationDeltaKm = static_cast<double>(Sample.Elevation - PreviousElevation);
			TotalElevationDeltaKm += AppliedElevationDeltaKm;
			MaxElevationDeltaKm = FMath::Max(MaxElevationDeltaKm, AppliedElevationDeltaKm);
			AffectedSampleIndices.Add(SampleIndex);
			OutExecutionMask[SampleIndex] = 1;
			InOutCumulativeExecutionMask[SampleIndex] = 1;
			InOutCumulativeElevationDeltaMaskKm[SampleIndex] += static_cast<float>(AppliedElevationDeltaKm);
			if (bContinentalGain)
			{
				InOutCumulativeContinentalGainMask[SampleIndex] += 1.0f;
			}
		}

		TArray<int32> TransferredSampleIndices;
		int32 TransferRejectedByLocalityCount = 0;
		int32 TransferRejectedByContinentalityCount = 0;
		int32 TransferRejectedByCapCount = 0;
		int32 TransferredContinentalSampleCount = 0;
		int32 TransferBoundaryLocalSampleCount = 0;
		int32 TransferCandidateSupportCount = 0;
		int32 TransferAnchorSeedCount = 0;
		double TransferInfluenceRadiusRad = 0.0;
		double DonorPlateShareBefore = 0.0;
		double DonorPlateShareAfter = 0.0;
		double RecipientPlateShareBefore = 0.0;
		double RecipientPlateShareAfter = 0.0;

		if (bEnableStructuralTransfer)
		{
			const auto IsRecipientAdjacent =
				[&Planet, OverridingPlateId](const int32 SampleIndex) -> bool
			{
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					return false;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex))
					{
						continue;
					}

					if (Planet.Samples[NeighborIndex].PlateId == OverridingPlateId)
					{
						return true;
					}
				}

				return false;
			};

			TArray<int32> DonorSeedSampleIndices;
			DonorSeedSampleIndices.Reserve(SeedSampleIndices.Num());
			TArray<int32> BoundaryAnchorSeedSampleIndices;
			BoundaryAnchorSeedSampleIndices.Reserve(SeedSampleIndices.Num());
			TArray<uint8> RefinedBoundaryLocalFlags;
			if (bEnableRefinedStructuralTransfer)
			{
				RefinedBoundaryLocalFlags.Init(0, Planet.Samples.Num());
			}
			const auto TryAddDonorSeed =
				[&Planet,
				 SubductingPlateId,
				 &DonorSeedSampleIndices,
				 &BoundaryAnchorSeedSampleIndices,
				 &RefinedBoundaryLocalFlags,
				 &TransferRejectedByContinentalityCount,
				 &IsRecipientAdjacent,
				 bEnableRefinedStructuralTransfer](
					const int32 SampleIndex)
			{
				if (!Planet.Samples.IsValidIndex(SampleIndex))
				{
					return;
				}

				const FSample& Sample = Planet.Samples[SampleIndex];
				if (Sample.PlateId == SubductingPlateId && Sample.ContinentalWeight >= 0.5f)
				{
					DonorSeedSampleIndices.AddUnique(SampleIndex);
					if (bEnableRefinedStructuralTransfer &&
						IsRecipientAdjacent(SampleIndex))
					{
						BoundaryAnchorSeedSampleIndices.AddUnique(SampleIndex);
						RefinedBoundaryLocalFlags[SampleIndex] = 1;
					}
				}
				else
				{
					++TransferRejectedByContinentalityCount;
				}
			};

			for (const int32 SampleIndex : CollisionEvent.CollisionSampleIndices)
			{
				TryAddDonorSeed(SampleIndex);
			}
			for (const int32 SampleIndex : SeedSampleIndices)
			{
				TryAddDonorSeed(SampleIndex);
			}

			if (DonorSeedSampleIndices.IsEmpty())
			{
				for (const int32 SeedSampleIndex : SeedSampleIndices)
				{
					if (!Planet.SampleAdjacency.IsValidIndex(SeedSampleIndex))
					{
						continue;
					}

					for (const int32 NeighborIndex : Planet.SampleAdjacency[SeedSampleIndex])
					{
						TryAddDonorSeed(NeighborIndex);
					}
				}
			}

			if (bEnableRefinedStructuralTransfer)
			{
				for (const int32 SeedSampleIndex : SeedSampleIndices)
				{
					if (!Planet.SampleAdjacency.IsValidIndex(SeedSampleIndex))
					{
						continue;
					}

					for (const int32 NeighborIndex : Planet.SampleAdjacency[SeedSampleIndex])
					{
						if (!Planet.Samples.IsValidIndex(NeighborIndex))
						{
							continue;
						}

						const FSample& NeighborSample = Planet.Samples[NeighborIndex];
						if (NeighborSample.PlateId == SubductingPlateId &&
							NeighborSample.ContinentalWeight >= 0.5f &&
							IsRecipientAdjacent(NeighborIndex))
						{
							BoundaryAnchorSeedSampleIndices.AddUnique(NeighborIndex);
							RefinedBoundaryLocalFlags[NeighborIndex] = 1;
						}
					}
				}

				for (const int32 AnchorSampleIndex : BoundaryAnchorSeedSampleIndices)
				{
					if (!Planet.SampleAdjacency.IsValidIndex(AnchorSampleIndex))
					{
						continue;
					}

					for (const int32 NeighborIndex : Planet.SampleAdjacency[AnchorSampleIndex])
					{
						if (!Planet.Samples.IsValidIndex(NeighborIndex))
						{
							continue;
						}

						const FSample& NeighborSample = Planet.Samples[NeighborIndex];
						if (NeighborSample.PlateId == SubductingPlateId &&
							NeighborSample.ContinentalWeight >= 0.5f)
						{
							RefinedBoundaryLocalFlags[NeighborIndex] = 1;
						}
					}
				}
			}

			const TArray<int32>& TransferSeedSampleIndices =
				(bEnableRefinedStructuralTransfer && !BoundaryAnchorSeedSampleIndices.IsEmpty())
					? BoundaryAnchorSeedSampleIndices
					: DonorSeedSampleIndices;
			TransferAnchorSeedCount = TransferSeedSampleIndices.Num();

			if (!TransferSeedSampleIndices.IsEmpty())
			{
				const double PatchSupportStrength =
					bEnableRefinedStructuralTransfer
						? FMath::Clamp(
							static_cast<double>(TransferAnchorSeedCount) / 16.0,
							0.0,
							1.0)
						: 0.0;
				const double TransferRadiusScale =
					bEnableRefinedStructuralTransfer
						? 1.10 +
							(0.35 * PenetrationStrength) +
							(0.25 * ContinentalStrength) +
							(0.20 * PatchSupportStrength)
						: 1.15 + (0.50 * PenetrationStrength) + (0.25 * ContinentalStrength);
				TransferInfluenceRadiusRad = FMath::Min(
					CollisionGlobalDistanceRad,
					FMath::Max(
						MinCollisionRadiusRad * (bEnableRefinedStructuralTransfer ? 1.6 : 1.5),
						SurgeRadiusRad * TransferRadiusScale));
				const double MaxTransferDistanceKm = TransferInfluenceRadiusRad * Planet.PlanetRadiusKm;

				TArray<double> BestDistanceKm;
				BestDistanceKm.Init(TNumericLimits<double>::Max(), Planet.Samples.Num());
				TArray<int32> Frontier = TransferSeedSampleIndices;
				for (const int32 SeedSampleIndex : TransferSeedSampleIndices)
				{
					if (BestDistanceKm.IsValidIndex(SeedSampleIndex))
					{
						BestDistanceKm[SeedSampleIndex] = 0.0;
					}
				}

				TArray<uint8> AddedToTransferCandidates;
				AddedToTransferCandidates.Init(0, Planet.Samples.Num());
				struct FTransferCandidate
				{
					int32 SampleIndex = INDEX_NONE;
					double DistanceKm = TNumericLimits<double>::Max();
					bool bBoundaryLocal = false;
				};
				TArray<FTransferCandidate> TransferCandidates;
				TransferCandidates.Reserve(TransferSeedSampleIndices.Num() * 4);

				while (!Frontier.IsEmpty())
				{
					int32 BestFrontierIndex = 0;
					double BestFrontierDistanceKm = BestDistanceKm[Frontier[0]];
					for (int32 FrontierIndex = 1; FrontierIndex < Frontier.Num(); ++FrontierIndex)
					{
						const double CandidateDistanceKm = BestDistanceKm[Frontier[FrontierIndex]];
						if (CandidateDistanceKm < BestFrontierDistanceKm)
						{
							BestFrontierIndex = FrontierIndex;
							BestFrontierDistanceKm = CandidateDistanceKm;
						}
					}

					const int32 SampleIndex = Frontier[BestFrontierIndex];
					Frontier.RemoveAtSwap(BestFrontierIndex, 1, EAllowShrinking::No);
					if (!Planet.Samples.IsValidIndex(SampleIndex) ||
						AddedToTransferCandidates[SampleIndex] != 0)
					{
						continue;
					}

					FSample& Sample = Planet.Samples[SampleIndex];
					if (Sample.PlateId != SubductingPlateId || Sample.ContinentalWeight < 0.5f)
					{
						++TransferRejectedByContinentalityCount;
						continue;
					}

					const double CenterDistanceKm =
						ComputeGeodesicDistance(
							Sample.Position.GetSafeNormal(),
							CollisionCenter) * Planet.PlanetRadiusKm;
					if (CenterDistanceKm > MaxTransferDistanceKm + UE_DOUBLE_SMALL_NUMBER ||
						BestFrontierDistanceKm > MaxTransferDistanceKm + UE_DOUBLE_SMALL_NUMBER)
					{
						++TransferRejectedByLocalityCount;
						continue;
					}

					const bool bDirectBoundaryLocal =
						CollisionEvent.CollisionSampleIndices.Contains(SampleIndex) ||
						IsRecipientAdjacent(SampleIndex);
					const bool bBoundaryLocal =
						bDirectBoundaryLocal ||
						(bEnableRefinedStructuralTransfer &&
							RefinedBoundaryLocalFlags.IsValidIndex(SampleIndex) &&
							RefinedBoundaryLocalFlags[SampleIndex] != 0);
					if (bEnableRefinedStructuralTransfer && !bBoundaryLocal)
					{
						++TransferRejectedByLocalityCount;
						continue;
					}

					AddedToTransferCandidates[SampleIndex] = 1;
					TransferCandidates.Add({ SampleIndex, BestFrontierDistanceKm, bBoundaryLocal });

					if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
					{
						continue;
					}

					const FVector3d SamplePosition = Sample.Position.GetSafeNormal();
					for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
					{
						if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
							Planet.Samples[NeighborIndex].PlateId != SubductingPlateId ||
							Planet.Samples[NeighborIndex].ContinentalWeight < 0.5f)
						{
							continue;
						}
						if (bEnableRefinedStructuralTransfer &&
							(!RefinedBoundaryLocalFlags.IsValidIndex(NeighborIndex) ||
								RefinedBoundaryLocalFlags[NeighborIndex] == 0))
						{
							continue;
						}

						const double EdgeDistanceKm =
							ComputeGeodesicDistance(
								SamplePosition,
								Planet.Samples[NeighborIndex].Position.GetSafeNormal()) * Planet.PlanetRadiusKm;
						const double CandidateDistanceKm = BestFrontierDistanceKm + EdgeDistanceKm;
						if (CandidateDistanceKm + UE_DOUBLE_SMALL_NUMBER < BestDistanceKm[NeighborIndex] &&
							CandidateDistanceKm <= MaxTransferDistanceKm + UE_DOUBLE_SMALL_NUMBER)
						{
							BestDistanceKm[NeighborIndex] = CandidateDistanceKm;
							Frontier.Add(NeighborIndex);
						}
					}
				}

				TransferCandidateSupportCount = TransferCandidates.Num();
				const int32 MaxTransferSamples =
					bEnableRefinedStructuralTransfer
						? FMath::Clamp(
							FMath::Max3(
								RefinedStructuralTransferTargetFloor,
								FMath::Min(
									TransferCandidateSupportCount,
									SelectedPair->ContinentalQualifiedSampleCount +
										FMath::Max(4, TransferAnchorSeedCount / 2)),
								FMath::Min(
									TransferCandidateSupportCount,
									FMath::Max(
										SelectedPair->SupportSampleCount / 3,
										TransferAnchorSeedCount * 2))),
							MinStructuralTransferSamples,
							MaxStructuralTransferSamples)
						: FMath::Clamp(
							FMath::Max(
								SelectedPair->ContinentalQualifiedSampleCount,
								CollisionSeedSampleIndices.Num()),
							MinStructuralTransferSamples,
							MaxStructuralTransferSamples);

				TransferCandidates.Sort([](const FTransferCandidate& Left, const FTransferCandidate& Right)
				{
					if (Left.bBoundaryLocal != Right.bBoundaryLocal)
					{
						return Left.bBoundaryLocal;
					}
					if (!FMath::IsNearlyEqual(Left.DistanceKm, Right.DistanceKm, TriangleEpsilon))
					{
						return Left.DistanceKm < Right.DistanceKm;
					}
					return Left.SampleIndex < Right.SampleIndex;
				});

				const auto CountSamplesForPlate =
					[&Planet](const int32 PlateId)
				{
					int32 Count = 0;
					for (const FSample& Sample : Planet.Samples)
					{
						Count += Sample.PlateId == PlateId ? 1 : 0;
					}
					return Count;
				};

				const int32 DonorSamplesBefore = CountSamplesForPlate(SubductingPlateId);
				const int32 RecipientSamplesBefore = CountSamplesForPlate(OverridingPlateId);
				const double InverseSampleCount = 1.0 / static_cast<double>(Planet.Samples.Num());

				for (const FTransferCandidate& Candidate : TransferCandidates)
				{
					if (TransferredSampleIndices.Num() >= MaxTransferSamples)
					{
						++TransferRejectedByCapCount;
						continue;
					}

					if (!Planet.Samples.IsValidIndex(Candidate.SampleIndex))
					{
						continue;
					}

					FSample& Sample = Planet.Samples[Candidate.SampleIndex];
					if (Sample.PlateId != SubductingPlateId || Sample.ContinentalWeight < 0.5f)
					{
						continue;
					}

					const float PreviousElevation = Sample.Elevation;
					const double TransferKernel = FTectonicPlanet::CollisionBiweightKernel(
						ComputeGeodesicDistance(
							Sample.Position.GetSafeNormal(),
							CollisionCenter),
						TransferInfluenceRadiusRad);
					const double StructuralElevationDeltaKm =
						0.22 *
						(0.75 + (0.25 * CollisionCoefficientScale)) *
						FMath::Max(TransferKernel, 0.25);

					RemoveV6CarriedSampleForCanonicalVertex(Planet, SubductingPlateId, Candidate.SampleIndex);
					Sample.PlateId = OverridingPlateId;
					Sample.ContinentalWeight = 1.0f;
					Sample.Thickness = FMath::Max(Sample.Thickness, static_cast<float>(ContinentalThicknessKm));
					Sample.Elevation = FMath::Clamp(
						static_cast<float>(static_cast<double>(Sample.Elevation) + StructuralElevationDeltaKm),
						0.0f,
						static_cast<float>(ElevationCeilingKm));
					Sample.OrogenyType = EOrogenyType::Himalayan;
					UpsertV6CarriedSampleForCanonicalVertex(Planet, OverridingPlateId, Candidate.SampleIndex);

					const double AppliedElevationDeltaKm = static_cast<double>(Sample.Elevation - PreviousElevation);
					TotalElevationDeltaKm += AppliedElevationDeltaKm;
					MaxElevationDeltaKm = FMath::Max(MaxElevationDeltaKm, AppliedElevationDeltaKm);
					++TransferredContinentalSampleCount;
					TransferBoundaryLocalSampleCount += Candidate.bBoundaryLocal ? 1 : 0;
					TransferredSampleIndices.Add(Candidate.SampleIndex);
					OutTransferMask[Candidate.SampleIndex] = 1;
					InOutCumulativeTransferMask[Candidate.SampleIndex] = 1;
					if (OutExecutionMask[Candidate.SampleIndex] == 0)
					{
						OutExecutionMask[Candidate.SampleIndex] = 1;
						InOutCumulativeExecutionMask[Candidate.SampleIndex] = 1;
						AffectedSampleIndices.Add(Candidate.SampleIndex);
					}
					InOutCumulativeElevationDeltaMaskKm[Candidate.SampleIndex] +=
						static_cast<float>(AppliedElevationDeltaKm);
				}

				RebuildV6MembershipFromCanonical(Planet);
				RecomputeV6BoundaryFlagsFromCanonical(Planet);

				const int32 DonorSamplesAfter = CountSamplesForPlate(SubductingPlateId);
				const int32 RecipientSamplesAfter = CountSamplesForPlate(OverridingPlateId);
				DonorPlateShareBefore = static_cast<double>(DonorSamplesBefore) * InverseSampleCount;
				DonorPlateShareAfter = static_cast<double>(DonorSamplesAfter) * InverseSampleCount;
				RecipientPlateShareBefore =
					static_cast<double>(RecipientSamplesBefore) * InverseSampleCount;
				RecipientPlateShareAfter =
					static_cast<double>(RecipientSamplesAfter) * InverseSampleCount;
			}
		}

		SyncV6CarriedSamplesFromCanonicalIndices(Planet, AffectedSampleIndices);
		Planet.ComputePlateScores();

		FResamplingStats& LegacyStats = Planet.LastResamplingStats;
		LegacyStats.CollisionCount = 1;
		LegacyStats.CollisionTerraneId = INDEX_NONE;
		LegacyStats.CollisionTerraneSampleCount =
			TransferredSampleIndices.IsEmpty()
				? CollisionEvent.TerraneSampleIndices.Num()
				: TransferredSampleIndices.Num();
		LegacyStats.CollisionOverridingPlateId = CollisionEvent.OverridingPlateId;
		LegacyStats.CollisionSubductingPlateId = CollisionEvent.SubductingPlateId;
		LegacyStats.CollisionSurgeAffectedCount = AffectedSampleIndices.Num();
		LegacyStats.CollisionCWBoostedCount = CWBoostedCount;
		LegacyStats.CollisionContinentalGainCount = ContinentalGainCount;
		LegacyStats.CollisionSurgeRadiusRad = SurgeRadiusRad;
		LegacyStats.CollisionSurgeMeanElevationDelta =
			AffectedSampleIndices.IsEmpty()
				? 0.0
				: TotalElevationDeltaKm / static_cast<double>(AffectedSampleIndices.Num());

		InOutLastExecutedSolveIndexByKey.Add(SelectedPairKey, CurrentSolveIndex);
		++InOutCumulativeExecutionCount;
		InOutCumulativeAffectedSampleVisits += AffectedSampleIndices.Num();
		InOutCumulativeContinentalGainCount += ContinentalGainCount;
		InOutCumulativeOwnershipChangeCount += TransferredSampleIndices.Num();
		InOutCumulativeTransferredSampleVisits += TransferredSampleIndices.Num();
		InOutCumulativeTransferredContinentalSampleCount += TransferredContinentalSampleCount;
		InOutCumulativeElevationDeltaKm += TotalElevationDeltaKm;
		InOutCumulativeMaxElevationDeltaKm =
			FMath::Max(InOutCumulativeMaxElevationDeltaKm, MaxElevationDeltaKm);

		OutDiagnostic.ExecutedCollisionCount = 1;
		OutDiagnostic.CumulativeExecutedCollisionCount = InOutCumulativeExecutionCount;
		OutDiagnostic.CumulativeCollisionAffectedSampleVisits = InOutCumulativeAffectedSampleVisits;
		OutDiagnostic.CumulativeCollisionDrivenContinentalGainCount =
			InOutCumulativeContinentalGainCount;
		OutDiagnostic.CumulativeCollisionDrivenOwnershipChangeCount =
			InOutCumulativeOwnershipChangeCount;
		OutDiagnostic.CumulativeCollisionTransferredSampleVisits =
			InOutCumulativeTransferredSampleVisits;
		OutDiagnostic.CumulativeCollisionTransferredContinentalSampleCount =
			InOutCumulativeTransferredContinentalSampleCount;
		OutDiagnostic.ExecutedPlateA = SelectedPair->PlateA;
		OutDiagnostic.ExecutedPlateB = SelectedPair->PlateB;
		OutDiagnostic.ExecutedOverridingPlateId = OverridingPlateId;
		OutDiagnostic.ExecutedSubductingPlateId = SubductingPlateId;
		OutDiagnostic.ExecutedObservationCount = SelectedPair->ObservationCount;
		OutDiagnostic.ExecutedAccumulatedPenetrationKm = SelectedPair->AccumulatedPenetrationKm;
		OutDiagnostic.ExecutedMeanConvergenceKmPerMy = SelectedPair->MeanConvergenceKmPerMy;
		OutDiagnostic.ExecutedMaxConvergenceKmPerMy = SelectedPair->MaxConvergenceKmPerMy;
		OutDiagnostic.ExecutedSupportSampleCount = SelectedPair->SupportSampleCount;
		OutDiagnostic.ExecutedSupportTriangleCount = SelectedPair->SupportTriangleCount;
		OutDiagnostic.ExecutedContinentalSupportPlateACount =
			SelectedPair->ContinentalSupportPlateACount;
		OutDiagnostic.ExecutedContinentalSupportPlateBCount =
			SelectedPair->ContinentalSupportPlateBCount;
		OutDiagnostic.ExecutedContinentalQualifiedSampleCount =
			SelectedPair->ContinentalQualifiedSampleCount;
		OutDiagnostic.ExecutedSeedSampleCount = SeedSampleIndices.Num();
		OutDiagnostic.ExecutedCollisionSeedSampleCount = CollisionSeedSampleIndices.Num();
		OutDiagnostic.ExecutedEffectiveMassSampleCount = EffectiveMassSampleCount;
		OutDiagnostic.CollisionAffectedSampleCount = AffectedSampleIndices.Num();
		OutDiagnostic.CollisionDrivenContinentalGainCount = ContinentalGainCount;
		OutDiagnostic.CollisionDrivenOwnershipChangeCount = TransferredSampleIndices.Num();
		OutDiagnostic.CollisionTransferredSampleCount = TransferredSampleIndices.Num();
		OutDiagnostic.CollisionTransferredContinentalSampleCount = TransferredContinentalSampleCount;
		OutDiagnostic.ExecutedTransferRejectedByLocalityCount = TransferRejectedByLocalityCount;
		OutDiagnostic.ExecutedTransferRejectedByContinentalityCount =
			TransferRejectedByContinentalityCount;
		OutDiagnostic.ExecutedTransferRejectedByCapCount = TransferRejectedByCapCount;
		OutDiagnostic.ExecutedTransferBoundaryLocalSampleCount = TransferBoundaryLocalSampleCount;
		OutDiagnostic.ExecutedTransferCandidateSupportCount = TransferCandidateSupportCount;
		OutDiagnostic.ExecutedTransferAnchorSeedCount = TransferAnchorSeedCount;
		OutDiagnostic.ExecutedInfluenceRadiusRad = SurgeRadiusRad;
		OutDiagnostic.ExecutedTransferInfluenceRadiusRad = TransferInfluenceRadiusRad;
		OutDiagnostic.ExecutedMeanElevationDeltaKm =
			AffectedSampleIndices.IsEmpty()
				? 0.0
				: TotalElevationDeltaKm / static_cast<double>(AffectedSampleIndices.Num());
		OutDiagnostic.ExecutedMaxElevationDeltaKm = MaxElevationDeltaKm;
		OutDiagnostic.ExecutedStrengthScale = CollisionCoefficientScale;
		OutDiagnostic.ExecutedDonorPlateShareBefore = DonorPlateShareBefore;
		OutDiagnostic.ExecutedDonorPlateShareAfter = DonorPlateShareAfter;
		OutDiagnostic.ExecutedDonorPlateShareDelta = DonorPlateShareAfter - DonorPlateShareBefore;
		OutDiagnostic.ExecutedRecipientPlateShareBefore = RecipientPlateShareBefore;
		OutDiagnostic.ExecutedRecipientPlateShareAfter = RecipientPlateShareAfter;
		OutDiagnostic.ExecutedRecipientPlateShareDelta =
			RecipientPlateShareAfter - RecipientPlateShareBefore;
		OutDiagnostic.CumulativeMeanElevationDeltaKm =
			InOutCumulativeAffectedSampleVisits > 0
				? InOutCumulativeElevationDeltaKm /
					static_cast<double>(InOutCumulativeAffectedSampleVisits)
				: 0.0;
		OutDiagnostic.CumulativeMaxElevationDeltaKm = InOutCumulativeMaxElevationDeltaKm;
		OutDiagnostic.bExecutedFromShadowQualifiedState = true;
		OutDiagnostic.CumulativeCollisionAffectedSampleCount = 0;
		for (const uint8 Value : InOutCumulativeExecutionMask)
		{
			OutDiagnostic.CumulativeCollisionAffectedSampleCount += Value != 0 ? 1 : 0;
		}
		OutDiagnostic.CumulativeCollisionTransferredSampleCount = 0;
		for (const uint8 Value : InOutCumulativeTransferMask)
		{
			OutDiagnostic.CumulativeCollisionTransferredSampleCount += Value != 0 ? 1 : 0;
		}

		UE_LOG(
			LogTemp,
			Log,
			TEXT("[V9CollisionExecution Step=%d] pair=(%d,%d) over_plate=%d sub_plate=%d obs=%d accumulated_penetration_km=%.1f mean_convergence_km_per_my=%.3f max_convergence_km_per_my=%.3f support=%d triangles=%d continental_support=%d/%d qualified_samples=%d seed_samples=%d collision_seed_samples=%d effective_mass=%d affected=%d cumulative_affected=%d cumulative_affected_visits=%d continental_gain=%d cumulative_continental_gain=%d ownership_change=%d cumulative_ownership_change=%d transfer_count=%d cumulative_transfer=%d transfer_continental=%d cumulative_transfer_continental=%d transfer_boundary_local=%d transfer_patch_support=%d transfer_anchor_seeds=%d transfer_reject_locality=%d transfer_reject_continentality=%d transfer_reject_cap=%d donor_share=%.6f->%.6f recipient_share=%.6f->%.6f cooldown_suppressed=%d qualified_unexecuted=%d influence_radius_rad=%.6f transfer_radius_rad=%.6f mean_elev_delta_km=%.6f max_elev_delta_km=%.6f cumulative_mean_elev_delta_km=%.6f cumulative_max_elev_delta_km=%.6f strength_scale=%.6f"),
			Planet.CurrentStep,
			SelectedPair->PlateA,
			SelectedPair->PlateB,
			OverridingPlateId,
			SubductingPlateId,
			SelectedPair->ObservationCount,
			SelectedPair->AccumulatedPenetrationKm,
			SelectedPair->MeanConvergenceKmPerMy,
			SelectedPair->MaxConvergenceKmPerMy,
			SelectedPair->SupportSampleCount,
			SelectedPair->SupportTriangleCount,
			SelectedPair->ContinentalSupportPlateACount,
			SelectedPair->ContinentalSupportPlateBCount,
			SelectedPair->ContinentalQualifiedSampleCount,
			SeedSampleIndices.Num(),
			CollisionSeedSampleIndices.Num(),
			EffectiveMassSampleCount,
			AffectedSampleIndices.Num(),
			OutDiagnostic.CumulativeCollisionAffectedSampleCount,
			InOutCumulativeAffectedSampleVisits,
			ContinentalGainCount,
			InOutCumulativeContinentalGainCount,
			TransferredSampleIndices.Num(),
			InOutCumulativeOwnershipChangeCount,
			TransferredSampleIndices.Num(),
			OutDiagnostic.CumulativeCollisionTransferredSampleCount,
			TransferredContinentalSampleCount,
			InOutCumulativeTransferredContinentalSampleCount,
			TransferBoundaryLocalSampleCount,
			TransferCandidateSupportCount,
			TransferAnchorSeedCount,
			TransferRejectedByLocalityCount,
			TransferRejectedByContinentalityCount,
			TransferRejectedByCapCount,
			DonorPlateShareBefore,
			DonorPlateShareAfter,
			RecipientPlateShareBefore,
			RecipientPlateShareAfter,
			OutDiagnostic.CooldownSuppressedQualifiedCount,
			OutDiagnostic.QualifiedButUnexecutedCount,
			SurgeRadiusRad,
			TransferInfluenceRadiusRad,
			OutDiagnostic.ExecutedMeanElevationDeltaKm,
			OutDiagnostic.ExecutedMaxElevationDeltaKm,
			OutDiagnostic.CumulativeMeanElevationDeltaKm,
			OutDiagnostic.CumulativeMaxElevationDeltaKm,
			OutDiagnostic.ExecutedStrengthScale);
	}

	void ExecuteV9ThesisShapedCollisionEvent(
		FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples,
		const FTectonicPlanetV6CollisionShadowDiagnostic& ShadowDiagnostic,
		TMap<uint64, int32>& InOutLastExecutedSolveIndexByKey,
		const int32 CurrentSolveIndex,
		TArray<uint8>& OutExecutionMask,
		TArray<uint8>& OutTransferMask,
		TArray<uint8>& OutTerraneComponentMask,
		TArray<uint8>& InOutCumulativeExecutionMask,
		TArray<uint8>& InOutCumulativeTransferMask,
		TArray<float>& InOutCumulativeElevationDeltaMaskKm,
		TArray<float>& InOutCumulativeContinentalGainMask,
		FTectonicPlanetV6CollisionExecutionDiagnostic& OutDiagnostic,
		int32& InOutCumulativeExecutionCount,
		int32& InOutCumulativeAffectedSampleVisits,
		int32& InOutCumulativeContinentalGainCount,
		int32& InOutCumulativeOwnershipChangeCount,
		int32& InOutCumulativeTransferredSampleVisits,
		int32& InOutCumulativeTransferredContinentalSampleCount,
		double& InOutCumulativeElevationDeltaKm,
		double& InOutCumulativeMaxElevationDeltaKm)
	{
		constexpr int32 CollisionExecutionPairCooldownSolves = 3;

		OutDiagnostic = FTectonicPlanetV6CollisionExecutionDiagnostic{};
		OutExecutionMask.Init(0, Planet.Samples.Num());
		OutTransferMask.Init(0, Planet.Samples.Num());
		OutTerraneComponentMask.Init(0, Planet.Samples.Num());
		OutDiagnostic.CumulativeExecutedCollisionCount = InOutCumulativeExecutionCount;
		if (InOutCumulativeExecutionMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeExecutionMask.Init(0, Planet.Samples.Num());
		}
		if (InOutCumulativeTransferMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeTransferMask.Init(0, Planet.Samples.Num());
		}
		if (InOutCumulativeElevationDeltaMaskKm.Num() != Planet.Samples.Num())
		{
			InOutCumulativeElevationDeltaMaskKm.Init(0.0f, Planet.Samples.Num());
		}
		if (InOutCumulativeContinentalGainMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeContinentalGainMask.Init(0.0f, Planet.Samples.Num());
		}
		OutDiagnostic.CumulativeCollisionAffectedSampleVisits = InOutCumulativeAffectedSampleVisits;
		OutDiagnostic.CumulativeCollisionDrivenContinentalGainCount = InOutCumulativeContinentalGainCount;
		OutDiagnostic.CumulativeCollisionDrivenOwnershipChangeCount = InOutCumulativeOwnershipChangeCount;
		OutDiagnostic.CumulativeCollisionTransferredSampleVisits = InOutCumulativeTransferredSampleVisits;
		OutDiagnostic.CumulativeCollisionTransferredContinentalSampleCount =
			InOutCumulativeTransferredContinentalSampleCount;
		OutDiagnostic.CumulativeMeanElevationDeltaKm =
			InOutCumulativeAffectedSampleVisits > 0
				? InOutCumulativeElevationDeltaKm /
					static_cast<double>(InOutCumulativeAffectedSampleVisits)
				: 0.0;
		OutDiagnostic.CumulativeMaxElevationDeltaKm = InOutCumulativeMaxElevationDeltaKm;
		for (const uint8 Value : InOutCumulativeExecutionMask)
		{
			OutDiagnostic.CumulativeCollisionAffectedSampleCount += Value != 0 ? 1 : 0;
		}
		for (const uint8 Value : InOutCumulativeTransferMask)
		{
			OutDiagnostic.CumulativeCollisionTransferredSampleCount += Value != 0 ? 1 : 0;
		}
		ResetV9CollisionExecutionLegacyStats(Planet);

		if (Planet.Samples.IsEmpty() || ResolvedSamples.Num() != Planet.Samples.Num())
		{
			return;
		}

		// --- Phase 1: Pair selection with per-pair cooldown ---
		TArray<FTectonicPlanetV6CollisionShadowTopPair> ExecutionPairs = ShadowDiagnostic.TopPairs;
		ExecutionPairs.Sort([](
			const FTectonicPlanetV6CollisionShadowTopPair& Left,
			const FTectonicPlanetV6CollisionShadowTopPair& Right)
		{
			if (Left.bQualified != Right.bQualified) { return Left.bQualified; }
			if (Left.CollisionSampleCount != Right.CollisionSampleCount)
			{
				return Left.CollisionSampleCount > Right.CollisionSampleCount;
			}
			if (Left.ObservationCount != Right.ObservationCount)
			{
				return Left.ObservationCount > Right.ObservationCount;
			}
			if (!FMath::IsNearlyEqual(Left.AccumulatedPenetrationKm, Right.AccumulatedPenetrationKm, TriangleEpsilon))
			{
				return Left.AccumulatedPenetrationKm > Right.AccumulatedPenetrationKm;
			}
			if (Left.ContinentalQualifiedSampleCount != Right.ContinentalQualifiedSampleCount)
			{
				return Left.ContinentalQualifiedSampleCount > Right.ContinentalQualifiedSampleCount;
			}
			if (Left.SupportSampleCount != Right.SupportSampleCount)
			{
				return Left.SupportSampleCount > Right.SupportSampleCount;
			}
			return Left.PlateA < Right.PlateA ||
				(Left.PlateA == Right.PlateA && Left.PlateB < Right.PlateB);
		});

		int32 QualifiedCount = 0;
		for (const FTectonicPlanetV6CollisionShadowTopPair& Pair : ExecutionPairs)
		{
			QualifiedCount += Pair.bQualified ? 1 : 0;
		}

		const FTectonicPlanetV6CollisionShadowTopPair* SelectedPair = nullptr;
		for (const FTectonicPlanetV6CollisionShadowTopPair& Pair : ExecutionPairs)
		{
			if (!Pair.bQualified) { continue; }
			const uint64 PairKey = MakeOrderedPlatePairKey(Pair.PlateA, Pair.PlateB);
			if (const int32* LastExecutedSolveIndex = InOutLastExecutedSolveIndexByKey.Find(PairKey))
			{
				if ((CurrentSolveIndex - *LastExecutedSolveIndex) <= CollisionExecutionPairCooldownSolves)
				{
					++OutDiagnostic.CooldownSuppressedQualifiedCount;
					continue;
				}
			}
			SelectedPair = &Pair;
			break;
		}

		OutDiagnostic.QualifiedButUnexecutedCount =
			FMath::Max(0, QualifiedCount - (SelectedPair != nullptr ? 1 : 0));
		if (SelectedPair == nullptr)
		{
			return;
		}

		// --- Phase 2: Determine polarity and identify seed samples ---
		const int32 OverridingPlateId =
			IsOverridingPlateLocal(Planet, SelectedPair->PlateA, SelectedPair->PlateB)
				? SelectedPair->PlateA
				: SelectedPair->PlateB;
		const int32 SubductingPlateId =
			OverridingPlateId == SelectedPair->PlateA ? SelectedPair->PlateB : SelectedPair->PlateA;

		TArray<int32> SeedSampleIndices;
		TArray<int32> CollisionSeedSampleIndices;
		SeedSampleIndices.Reserve(SelectedPair->SupportSampleCount);
		CollisionSeedSampleIndices.Reserve(SelectedPair->CollisionSampleCount);
		FVector3d CenterSum = FVector3d::ZeroVector;
		const uint64 SelectedPairKey =
			MakeOrderedPlatePairKey(SelectedPair->PlateA, SelectedPair->PlateB);
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!ResolvedSamples.IsValidIndex(SampleIndex)) { continue; }
			const FTectonicPlanetV6ResolvedSample& Resolved = ResolvedSamples[SampleIndex];
			if (!Resolved.bActiveZoneSample ||
				Resolved.ActiveZonePrimaryPlateId == INDEX_NONE ||
				Resolved.ActiveZoneSecondaryPlateId == INDEX_NONE ||
				(Resolved.ActiveZoneCause != ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction &&
					Resolved.ActiveZoneCause != ETectonicPlanetV6ActiveZoneCause::CollisionContact))
			{
				continue;
			}
			if (MakeOrderedPlatePairKey(
					Resolved.ActiveZonePrimaryPlateId,
					Resolved.ActiveZoneSecondaryPlateId) != SelectedPairKey)
			{
				continue;
			}
			SeedSampleIndices.Add(SampleIndex);
			CenterSum += Planet.Samples[SampleIndex].Position;
			if (Resolved.ActiveZoneCause == ETectonicPlanetV6ActiveZoneCause::CollisionContact)
			{
				CollisionSeedSampleIndices.Add(SampleIndex);
			}
		}

		if (SeedSampleIndices.IsEmpty())
		{
			return;
		}

		FVector3d CollisionCenter = CenterSum.GetSafeNormal();
		if (CollisionCenter.IsNearlyZero())
		{
			CollisionCenter = Planet.Samples[SeedSampleIndices[0]].Position.GetSafeNormal();
		}

		const FPlate* OverridingPlate = FindPlateById(Planet, OverridingPlateId);
		const FPlate* SubductingPlate = FindPlateById(Planet, SubductingPlateId);
		if (OverridingPlate == nullptr || SubductingPlate == nullptr || CollisionCenter.IsNearlyZero())
		{
			return;
		}

		// --- Phase 3: Identify donor-side continental seed samples ---
		const auto IsRecipientAdjacent =
			[&Planet, OverridingPlateId](const int32 SampleIndex) -> bool
		{
			if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex)) { return false; }
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (Planet.Samples.IsValidIndex(NeighborIndex) &&
					Planet.Samples[NeighborIndex].PlateId == OverridingPlateId)
				{
					return true;
				}
			}
			return false;
		};

		TSet<int32> DonorSeedSet;
		int32 TransferRejectedByContinentalityCount = 0;
		for (const int32 SampleIndex : CollisionSeedSampleIndices)
		{
			if (Planet.Samples.IsValidIndex(SampleIndex) &&
				Planet.Samples[SampleIndex].PlateId == SubductingPlateId &&
				Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f)
			{
				DonorSeedSet.Add(SampleIndex);
			}
		}
		for (const int32 SampleIndex : SeedSampleIndices)
		{
			if (Planet.Samples.IsValidIndex(SampleIndex) &&
				Planet.Samples[SampleIndex].PlateId == SubductingPlateId &&
				Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f)
			{
				DonorSeedSet.Add(SampleIndex);
			}
		}
		if (DonorSeedSet.IsEmpty())
		{
			for (const int32 SeedSampleIndex : SeedSampleIndices)
			{
				if (!Planet.SampleAdjacency.IsValidIndex(SeedSampleIndex)) { continue; }
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SeedSampleIndex])
				{
					if (Planet.Samples.IsValidIndex(NeighborIndex) &&
						Planet.Samples[NeighborIndex].PlateId == SubductingPlateId &&
						Planet.Samples[NeighborIndex].ContinentalWeight >= 0.5f)
					{
						DonorSeedSet.Add(NeighborIndex);
					}
					else if (Planet.Samples.IsValidIndex(NeighborIndex) &&
						Planet.Samples[NeighborIndex].PlateId == SubductingPlateId)
					{
						++TransferRejectedByContinentalityCount;
					}
				}
			}
		}

		if (DonorSeedSet.IsEmpty())
		{
			OutDiagnostic.ExecutedTransferRejectedByContinentalityCount = TransferRejectedByContinentalityCount;
			return;
		}

		// --- Phase 4: BFS flood-fill to detect coherent donor terrane component ---
		TArray<double> BFSDistanceKm;
		BFSDistanceKm.Init(TNumericLimits<double>::Max(), Planet.Samples.Num());
		TArray<uint8> InTerraneComponent;
		InTerraneComponent.Init(0, Planet.Samples.Num());
		TArray<int32> TerraneComponentIndices;
		TerraneComponentIndices.Reserve(512);

		TArray<int32> BFSFrontier;
		BFSFrontier.Reserve(256);
		for (const int32 SeedIndex : DonorSeedSet)
		{
			BFSDistanceKm[SeedIndex] = 0.0;
			BFSFrontier.Add(SeedIndex);
		}

		const double MaxBFSDistanceKm = 0.659 * Planet.PlanetRadiusKm;

		while (!BFSFrontier.IsEmpty())
		{
			int32 BestFrontierIndex = 0;
			double BestFrontierDistKm = BFSDistanceKm[BFSFrontier[0]];
			for (int32 I = 1; I < BFSFrontier.Num(); ++I)
			{
				const double D = BFSDistanceKm[BFSFrontier[I]];
				if (D < BestFrontierDistKm)
				{
					BestFrontierIndex = I;
					BestFrontierDistKm = D;
				}
			}

			const int32 Current = BFSFrontier[BestFrontierIndex];
			BFSFrontier.RemoveAtSwap(BestFrontierIndex, 1, EAllowShrinking::No);

			if (InTerraneComponent[Current] != 0) { continue; }
			if (!Planet.Samples.IsValidIndex(Current)) { continue; }

			const FSample& CurrentSample = Planet.Samples[Current];
			if (CurrentSample.PlateId != SubductingPlateId || CurrentSample.ContinentalWeight < 0.5f)
			{
				continue;
			}

			InTerraneComponent[Current] = 1;
			TerraneComponentIndices.Add(Current);
			OutTerraneComponentMask[Current] = 1;

			if (!Planet.SampleAdjacency.IsValidIndex(Current)) { continue; }

			const FVector3d CurrentPos = CurrentSample.Position.GetSafeNormal();
			for (const int32 NeighborIndex : Planet.SampleAdjacency[Current])
			{
				if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
					InTerraneComponent[NeighborIndex] != 0)
				{
					continue;
				}
				const FSample& NeighborSample = Planet.Samples[NeighborIndex];
				if (NeighborSample.PlateId != SubductingPlateId ||
					NeighborSample.ContinentalWeight < 0.5f)
				{
					continue;
				}
				const double EdgeDistKm =
					ComputeGeodesicDistance(CurrentPos, NeighborSample.Position.GetSafeNormal()) *
					Planet.PlanetRadiusKm;
				const double CandidateDistKm = BestFrontierDistKm + EdgeDistKm;
				if (CandidateDistKm < BFSDistanceKm[NeighborIndex] &&
					CandidateDistKm <= MaxBFSDistanceKm)
				{
					BFSDistanceKm[NeighborIndex] = CandidateDistKm;
					BFSFrontier.Add(NeighborIndex);
				}
			}
		}

		const int32 DonorTerraneComponentSize = TerraneComponentIndices.Num();
		if (DonorTerraneComponentSize == 0)
		{
			OutDiagnostic.ExecutedDonorTerraneComponentSize = 0;
			return;
		}

		// --- Phase 5: Compute thesis-style event parameters ---
		const FVector3d OverridingAxis = OverridingPlate->RotationAxis.IsNearlyZero()
			? FVector3d::ZeroVector : OverridingPlate->RotationAxis.GetSafeNormal();
		const FVector3d SubductingAxis = SubductingPlate->RotationAxis.IsNearlyZero()
			? FVector3d::ZeroVector : SubductingPlate->RotationAxis.GetSafeNormal();
		const FVector3d OverridingVelocity =
			FVector3d::CrossProduct(OverridingAxis * OverridingPlate->AngularSpeed, CollisionCenter);
		const FVector3d SubductingVelocity =
			FVector3d::CrossProduct(SubductingAxis * SubductingPlate->AngularSpeed, CollisionCenter);
		const double RelativeSpeedKmPerMy =
			((OverridingVelocity - SubductingVelocity).Length() * Planet.PlanetRadiusKm) /
			FMath::Max(DeltaTimeMyears, UE_DOUBLE_SMALL_NUMBER);

		const double TypicalPlateSize =
			static_cast<double>(Planet.Samples.Num()) /
			static_cast<double>(FMath::Max(Planet.Plates.Num(), 1));
		const double TerraneAreaFraction =
			static_cast<double>(DonorTerraneComponentSize) / FMath::Max(TypicalPlateSize, 1.0);
		const double SpeedFraction =
			FMath::Max(RelativeSpeedKmPerMy, 0.0) / FMath::Max(MaxPlateSpeedKmPerMy, UE_DOUBLE_SMALL_NUMBER);
		const double Xi = FMath::Sqrt(FMath::Clamp(SpeedFraction * TerraneAreaFraction, 0.0, 4.0));

		const double CollisionGlobalDistanceRad = 0.659;
		const double MinCollisionRadiusRad = 0.05;
		const double CollisionCoefficient = 0.013;
		const double SurgeRadiusRad = FMath::Min(
			CollisionGlobalDistanceRad,
			FMath::Max(CollisionGlobalDistanceRad * FMath::Min(Xi, 1.0), MinCollisionRadiusRad));
		const double TransferBoundRadiusKm = SurgeRadiusRad * Planet.PlanetRadiusKm;
		const double AverageSampleAreaSr = (4.0 * PI) / static_cast<double>(Planet.Samples.Num());
		const double TerraneAreaSr = static_cast<double>(DonorTerraneComponentSize) * AverageSampleAreaSr;
		const int32 EffectiveMassSampleCount = DonorTerraneComponentSize;

		// --- Phase 6: Locality-bounded transfer ---
		struct FThesisTransferCandidate
		{
			int32 SampleIndex = INDEX_NONE;
			double DistanceFromFrontKm = TNumericLimits<double>::Max();
			bool bBoundaryLocal = false;
		};
		TArray<FThesisTransferCandidate> TransferCandidates;
		TransferCandidates.Reserve(DonorTerraneComponentSize);

		for (const int32 SampleIndex : TerraneComponentIndices)
		{
			const double DistFromFront = BFSDistanceKm[SampleIndex];
			if (DistFromFront > TransferBoundRadiusKm + UE_DOUBLE_SMALL_NUMBER)
			{
				continue;
			}
			const double DistFromCenter =
				ComputeGeodesicDistance(
					Planet.Samples[SampleIndex].Position.GetSafeNormal(),
					CollisionCenter) * Planet.PlanetRadiusKm;
			if (DistFromCenter > TransferBoundRadiusKm * 1.5 + UE_DOUBLE_SMALL_NUMBER)
			{
				continue;
			}
			const bool bBL = DonorSeedSet.Contains(SampleIndex) || IsRecipientAdjacent(SampleIndex);
			TransferCandidates.Add({ SampleIndex, DistFromFront, bBL });
		}

		TransferCandidates.Sort([](const FThesisTransferCandidate& Left, const FThesisTransferCandidate& Right)
		{
			if (Left.bBoundaryLocal != Right.bBoundaryLocal) { return Left.bBoundaryLocal; }
			if (!FMath::IsNearlyEqual(Left.DistanceFromFrontKm, Right.DistanceFromFrontKm, TriangleEpsilon))
			{
				return Left.DistanceFromFrontKm < Right.DistanceFromFrontKm;
			}
			return Left.SampleIndex < Right.SampleIndex;
		});

		const int32 SafetyCapSamples =
			FMath::Max(50, Planet.Samples.Num() / 50);
		const int32 MaxTransferSamples = FMath::Min(TransferCandidates.Num(), SafetyCapSamples);

		const auto CountSamplesForPlate =
			[&Planet](const int32 PlateId)
		{
			int32 Count = 0;
			for (const FSample& Sample : Planet.Samples)
			{
				Count += Sample.PlateId == PlateId ? 1 : 0;
			}
			return Count;
		};

		const int32 DonorSamplesBefore = CountSamplesForPlate(SubductingPlateId);
		const int32 RecipientSamplesBefore = CountSamplesForPlate(OverridingPlateId);
		const double InverseSampleCount = 1.0 / static_cast<double>(Planet.Samples.Num());

		TArray<int32> TransferredSampleIndices;
		TransferredSampleIndices.Reserve(MaxTransferSamples);
		int32 TransferredContinentalSampleCount = 0;
		int32 TransferBoundaryLocalSampleCount = 0;
		int32 TransferRejectedByLocalityCount = 0;
		int32 TransferRejectedByCapCount = 0;
		int32 TransferRejectedByConnectivityCount = 0;
		double TotalElevationDeltaKm = 0.0;
		double MaxElevationDeltaKm = 0.0;

		const double ObservationStrength = FMath::Clamp(
			(static_cast<double>(SelectedPair->ObservationCount) - 3.0) / 9.0, 0.0, 1.0);
		const double PenetrationStrength = FMath::Clamp(
			(SelectedPair->AccumulatedPenetrationKm - 3000.0) / 9000.0, 0.0, 1.0);
		const double ContinentalStrength = FMath::Clamp(
			static_cast<double>(SelectedPair->ContinentalQualifiedSampleCount) / 24.0, 0.0, 1.0);
		const double CollisionStrength01 = FMath::Clamp(
			(0.45 * ObservationStrength) + (0.35 * PenetrationStrength) + (0.20 * ContinentalStrength),
			0.0, 1.0);
		const double CollisionCoefficientScale = 1.0 + (0.85 * CollisionStrength01);

		for (const FThesisTransferCandidate& Candidate : TransferCandidates)
		{
			if (TransferredSampleIndices.Num() >= MaxTransferSamples)
			{
				++TransferRejectedByCapCount;
				continue;
			}
			if (!Planet.Samples.IsValidIndex(Candidate.SampleIndex)) { continue; }

			FSample& Sample = Planet.Samples[Candidate.SampleIndex];
			if (Sample.PlateId != SubductingPlateId || Sample.ContinentalWeight < 0.5f)
			{
				continue;
			}

			const float PreviousElevation = Sample.Elevation;
			const double TransferKernel = FTectonicPlanet::CollisionBiweightKernel(
				ComputeGeodesicDistance(Sample.Position.GetSafeNormal(), CollisionCenter),
				FMath::Max(SurgeRadiusRad, MinCollisionRadiusRad));
			const double StructuralElevationDeltaKm =
				0.22 * (0.75 + (0.25 * CollisionCoefficientScale)) *
				FMath::Max(TransferKernel, 0.25);

			RemoveV6CarriedSampleForCanonicalVertex(Planet, SubductingPlateId, Candidate.SampleIndex);
			Sample.PlateId = OverridingPlateId;
			Sample.ContinentalWeight = 1.0f;
			Sample.Thickness = FMath::Max(Sample.Thickness, static_cast<float>(ContinentalThicknessKm));
			Sample.Elevation = FMath::Clamp(
				static_cast<float>(static_cast<double>(Sample.Elevation) + StructuralElevationDeltaKm),
				0.0f, static_cast<float>(ElevationCeilingKm));
			Sample.OrogenyType = EOrogenyType::Himalayan;
			UpsertV6CarriedSampleForCanonicalVertex(Planet, OverridingPlateId, Candidate.SampleIndex);

			const double AppliedElevationDeltaKm = static_cast<double>(Sample.Elevation - PreviousElevation);
			TotalElevationDeltaKm += AppliedElevationDeltaKm;
			MaxElevationDeltaKm = FMath::Max(MaxElevationDeltaKm, AppliedElevationDeltaKm);
			++TransferredContinentalSampleCount;
			TransferBoundaryLocalSampleCount += Candidate.bBoundaryLocal ? 1 : 0;
			TransferredSampleIndices.Add(Candidate.SampleIndex);
			OutTransferMask[Candidate.SampleIndex] = 1;
			InOutCumulativeTransferMask[Candidate.SampleIndex] = 1;
			OutExecutionMask[Candidate.SampleIndex] = 1;
			InOutCumulativeExecutionMask[Candidate.SampleIndex] = 1;
			InOutCumulativeElevationDeltaMaskKm[Candidate.SampleIndex] +=
				static_cast<float>(AppliedElevationDeltaKm);
		}

		if (!TransferredSampleIndices.IsEmpty())
		{
			RebuildV6MembershipFromCanonical(Planet);
			RecomputeV6BoundaryFlagsFromCanonical(Planet);
		}

		// --- Phase 7: Thesis-shaped collision uplift across influence zone ---
		TArray<int32> AffectedSampleIndices;
		AffectedSampleIndices.Reserve(Planet.Samples.Num() / 16);
		int32 ContinentalGainCount = 0;

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			const double DistanceRad = ComputeGeodesicDistance(
				Sample.Position.GetSafeNormal(), CollisionCenter);
			if (DistanceRad >= SurgeRadiusRad) { continue; }

			const double Kernel = FTectonicPlanet::CollisionBiweightKernel(DistanceRad, SurgeRadiusRad);
			if (Kernel <= 0.0) { continue; }

			const float PreviousContinentalWeight = Sample.ContinentalWeight;
			const float PreviousElevation = Sample.Elevation;
			const double ElevationDeltaKm =
				CollisionCoefficient *
				CollisionCoefficientScale *
				(static_cast<double>(EffectiveMassSampleCount) /
					static_cast<double>(Planet.Samples.Num())) *
				Kernel *
				Planet.PlanetRadiusKm;

			Sample.Elevation = FMath::Clamp(
				static_cast<float>(static_cast<double>(Sample.Elevation) + ElevationDeltaKm),
				static_cast<float>(TrenchElevationKm),
				static_cast<float>(ElevationCeilingKm));
			if (Sample.Elevation < 0.0f) { Sample.Elevation = 0.0f; }
			if (Sample.Elevation > 0.0f) { Sample.OrogenyType = EOrogenyType::Himalayan; }
			Sample.ContinentalWeight = 1.0f;
			Sample.Thickness = FMath::Max(Sample.Thickness, static_cast<float>(ContinentalThicknessKm));

			const bool bContinentalGain =
				PreviousContinentalWeight < 0.5f && Sample.ContinentalWeight >= 0.5f;
			ContinentalGainCount += bContinentalGain ? 1 : 0;
			const double AppliedElevationDeltaKm = static_cast<double>(Sample.Elevation - PreviousElevation);
			TotalElevationDeltaKm += AppliedElevationDeltaKm;
			MaxElevationDeltaKm = FMath::Max(MaxElevationDeltaKm, AppliedElevationDeltaKm);
			AffectedSampleIndices.Add(SampleIndex);
			if (OutExecutionMask[SampleIndex] == 0)
			{
				OutExecutionMask[SampleIndex] = 1;
				InOutCumulativeExecutionMask[SampleIndex] = 1;
			}
			InOutCumulativeElevationDeltaMaskKm[SampleIndex] += static_cast<float>(AppliedElevationDeltaKm);
			if (bContinentalGain)
			{
				InOutCumulativeContinentalGainMask[SampleIndex] += 1.0f;
			}
		}

		// --- Phase 8: Bookkeeping ---
		SyncV6CarriedSamplesFromCanonicalIndices(Planet, AffectedSampleIndices);
		Planet.ComputePlateScores();

		FResamplingStats& LegacyStats = Planet.LastResamplingStats;
		LegacyStats.CollisionCount = 1;
		LegacyStats.CollisionTerraneId = INDEX_NONE;
		LegacyStats.CollisionTerraneSampleCount = TransferredSampleIndices.Num();
		LegacyStats.CollisionOverridingPlateId = OverridingPlateId;
		LegacyStats.CollisionSubductingPlateId = SubductingPlateId;
		LegacyStats.CollisionSurgeAffectedCount = AffectedSampleIndices.Num();
		LegacyStats.CollisionContinentalGainCount = ContinentalGainCount;
		LegacyStats.CollisionSurgeRadiusRad = SurgeRadiusRad;
		LegacyStats.CollisionSurgeMeanElevationDelta =
			AffectedSampleIndices.IsEmpty()
				? 0.0
				: TotalElevationDeltaKm / static_cast<double>(AffectedSampleIndices.Num());

		const int32 DonorSamplesAfter = CountSamplesForPlate(SubductingPlateId);
		const int32 RecipientSamplesAfter = CountSamplesForPlate(OverridingPlateId);
		const double DonorPlateShareBefore = static_cast<double>(DonorSamplesBefore) * InverseSampleCount;
		const double DonorPlateShareAfter = static_cast<double>(DonorSamplesAfter) * InverseSampleCount;
		const double RecipientPlateShareBefore = static_cast<double>(RecipientSamplesBefore) * InverseSampleCount;
		const double RecipientPlateShareAfter = static_cast<double>(RecipientSamplesAfter) * InverseSampleCount;

		InOutLastExecutedSolveIndexByKey.Add(SelectedPairKey, CurrentSolveIndex);
		++InOutCumulativeExecutionCount;
		InOutCumulativeAffectedSampleVisits += AffectedSampleIndices.Num();
		InOutCumulativeContinentalGainCount += ContinentalGainCount;
		InOutCumulativeOwnershipChangeCount += TransferredSampleIndices.Num();
		InOutCumulativeTransferredSampleVisits += TransferredSampleIndices.Num();
		InOutCumulativeTransferredContinentalSampleCount += TransferredContinentalSampleCount;
		InOutCumulativeElevationDeltaKm += TotalElevationDeltaKm;
		InOutCumulativeMaxElevationDeltaKm =
			FMath::Max(InOutCumulativeMaxElevationDeltaKm, MaxElevationDeltaKm);

		// --- Populate diagnostics ---
		OutDiagnostic.ExecutedCollisionCount = 1;
		OutDiagnostic.CumulativeExecutedCollisionCount = InOutCumulativeExecutionCount;
		OutDiagnostic.CumulativeCollisionAffectedSampleVisits = InOutCumulativeAffectedSampleVisits;
		OutDiagnostic.CumulativeCollisionDrivenContinentalGainCount = InOutCumulativeContinentalGainCount;
		OutDiagnostic.CumulativeCollisionDrivenOwnershipChangeCount = InOutCumulativeOwnershipChangeCount;
		OutDiagnostic.CumulativeCollisionTransferredSampleVisits = InOutCumulativeTransferredSampleVisits;
		OutDiagnostic.CumulativeCollisionTransferredContinentalSampleCount =
			InOutCumulativeTransferredContinentalSampleCount;
		OutDiagnostic.ExecutedPlateA = SelectedPair->PlateA;
		OutDiagnostic.ExecutedPlateB = SelectedPair->PlateB;
		OutDiagnostic.ExecutedOverridingPlateId = OverridingPlateId;
		OutDiagnostic.ExecutedSubductingPlateId = SubductingPlateId;
		OutDiagnostic.ExecutedObservationCount = SelectedPair->ObservationCount;
		OutDiagnostic.ExecutedAccumulatedPenetrationKm = SelectedPair->AccumulatedPenetrationKm;
		OutDiagnostic.ExecutedMeanConvergenceKmPerMy = SelectedPair->MeanConvergenceKmPerMy;
		OutDiagnostic.ExecutedMaxConvergenceKmPerMy = SelectedPair->MaxConvergenceKmPerMy;
		OutDiagnostic.ExecutedSupportSampleCount = SelectedPair->SupportSampleCount;
		OutDiagnostic.ExecutedSupportTriangleCount = SelectedPair->SupportTriangleCount;
		OutDiagnostic.ExecutedContinentalSupportPlateACount = SelectedPair->ContinentalSupportPlateACount;
		OutDiagnostic.ExecutedContinentalSupportPlateBCount = SelectedPair->ContinentalSupportPlateBCount;
		OutDiagnostic.ExecutedContinentalQualifiedSampleCount = SelectedPair->ContinentalQualifiedSampleCount;
		OutDiagnostic.ExecutedSeedSampleCount = SeedSampleIndices.Num();
		OutDiagnostic.ExecutedCollisionSeedSampleCount = CollisionSeedSampleIndices.Num();
		OutDiagnostic.ExecutedEffectiveMassSampleCount = EffectiveMassSampleCount;
		OutDiagnostic.CollisionAffectedSampleCount = AffectedSampleIndices.Num();
		OutDiagnostic.CollisionDrivenContinentalGainCount = ContinentalGainCount;
		OutDiagnostic.CollisionDrivenOwnershipChangeCount = TransferredSampleIndices.Num();
		OutDiagnostic.CollisionTransferredSampleCount = TransferredSampleIndices.Num();
		OutDiagnostic.CollisionTransferredContinentalSampleCount = TransferredContinentalSampleCount;
		OutDiagnostic.ExecutedTransferRejectedByLocalityCount = TransferRejectedByLocalityCount;
		OutDiagnostic.ExecutedTransferRejectedByContinentalityCount = TransferRejectedByContinentalityCount;
		OutDiagnostic.ExecutedTransferRejectedByCapCount = TransferRejectedByCapCount;
		OutDiagnostic.ExecutedTransferRejectedByConnectivityCount = TransferRejectedByConnectivityCount;
		OutDiagnostic.ExecutedTransferBoundaryLocalSampleCount = TransferBoundaryLocalSampleCount;
		OutDiagnostic.ExecutedTransferCandidateSupportCount = TransferCandidates.Num();
		OutDiagnostic.ExecutedTransferAnchorSeedCount = DonorSeedSet.Num();
		OutDiagnostic.ExecutedDonorTerraneComponentSize = DonorTerraneComponentSize;
		OutDiagnostic.ExecutedTransferredComponentSize = TransferredSampleIndices.Num();
		OutDiagnostic.ExecutedTerraneAreaSr = TerraneAreaSr;
		OutDiagnostic.ExecutedXi = Xi;
		OutDiagnostic.ExecutedRelativeSpeedKmPerMy = RelativeSpeedKmPerMy;
		OutDiagnostic.ExecutedInfluenceRadiusRad = SurgeRadiusRad;
		OutDiagnostic.ExecutedTransferInfluenceRadiusRad = SurgeRadiusRad;
		OutDiagnostic.ExecutedMeanElevationDeltaKm =
			AffectedSampleIndices.IsEmpty()
				? 0.0
				: TotalElevationDeltaKm / static_cast<double>(AffectedSampleIndices.Num());
		OutDiagnostic.ExecutedMaxElevationDeltaKm = MaxElevationDeltaKm;
		OutDiagnostic.ExecutedStrengthScale = CollisionCoefficientScale;
		OutDiagnostic.ExecutedDonorPlateShareBefore = DonorPlateShareBefore;
		OutDiagnostic.ExecutedDonorPlateShareAfter = DonorPlateShareAfter;
		OutDiagnostic.ExecutedDonorPlateShareDelta = DonorPlateShareAfter - DonorPlateShareBefore;
		OutDiagnostic.ExecutedRecipientPlateShareBefore = RecipientPlateShareBefore;
		OutDiagnostic.ExecutedRecipientPlateShareAfter = RecipientPlateShareAfter;
		OutDiagnostic.ExecutedRecipientPlateShareDelta = RecipientPlateShareAfter - RecipientPlateShareBefore;
		OutDiagnostic.CumulativeMeanElevationDeltaKm =
			InOutCumulativeAffectedSampleVisits > 0
				? InOutCumulativeElevationDeltaKm /
					static_cast<double>(InOutCumulativeAffectedSampleVisits)
				: 0.0;
		OutDiagnostic.CumulativeMaxElevationDeltaKm = InOutCumulativeMaxElevationDeltaKm;
		OutDiagnostic.bExecutedFromShadowQualifiedState = true;
		OutDiagnostic.CumulativeCollisionAffectedSampleCount = 0;
		for (const uint8 Value : InOutCumulativeExecutionMask)
		{
			OutDiagnostic.CumulativeCollisionAffectedSampleCount += Value != 0 ? 1 : 0;
		}
		OutDiagnostic.CumulativeCollisionTransferredSampleCount = 0;
		for (const uint8 Value : InOutCumulativeTransferMask)
		{
			OutDiagnostic.CumulativeCollisionTransferredSampleCount += Value != 0 ? 1 : 0;
		}

		UE_LOG(
			LogTemp,
			Log,
			TEXT("[V9ThesisCollision Step=%d] pair=(%d,%d) over_plate=%d sub_plate=%d obs=%d accumulated_penetration_km=%.1f mean_convergence_km_per_my=%.3f max_convergence_km_per_my=%.3f support=%d continental_qualified=%d seed_samples=%d collision_seeds=%d terrane_component=%d terrane_area_sr=%.6f xi=%.6f rel_speed_km_per_my=%.3f effective_mass=%d surge_radius_rad=%.6f transfer_bound_km=%.1f transferred=%d transferred_continental=%d boundary_local=%d reject_locality=%d reject_continentality=%d reject_cap=%d reject_connectivity=%d affected=%d continental_gain=%d mean_elev_delta_km=%.6f max_elev_delta_km=%.6f donor_share=%.6f->%.6f recipient_share=%.6f->%.6f cumulative_exec=%d cumulative_affected=%d cumulative_transfer=%d cumulative_continental_gain=%d"),
			Planet.CurrentStep,
			SelectedPair->PlateA,
			SelectedPair->PlateB,
			OverridingPlateId,
			SubductingPlateId,
			SelectedPair->ObservationCount,
			SelectedPair->AccumulatedPenetrationKm,
			SelectedPair->MeanConvergenceKmPerMy,
			SelectedPair->MaxConvergenceKmPerMy,
			SelectedPair->SupportSampleCount,
			SelectedPair->ContinentalQualifiedSampleCount,
			SeedSampleIndices.Num(),
			CollisionSeedSampleIndices.Num(),
			DonorTerraneComponentSize,
			TerraneAreaSr,
			Xi,
			RelativeSpeedKmPerMy,
			EffectiveMassSampleCount,
			SurgeRadiusRad,
			TransferBoundRadiusKm,
			TransferredSampleIndices.Num(),
			TransferredContinentalSampleCount,
			TransferBoundaryLocalSampleCount,
			TransferRejectedByLocalityCount,
			TransferRejectedByContinentalityCount,
			TransferRejectedByCapCount,
			TransferRejectedByConnectivityCount,
			AffectedSampleIndices.Num(),
			ContinentalGainCount,
			OutDiagnostic.ExecutedMeanElevationDeltaKm,
			OutDiagnostic.ExecutedMaxElevationDeltaKm,
			DonorPlateShareBefore,
			DonorPlateShareAfter,
			RecipientPlateShareBefore,
			RecipientPlateShareAfter,
			InOutCumulativeExecutionCount,
			OutDiagnostic.CumulativeCollisionAffectedSampleCount,
			OutDiagnostic.CumulativeCollisionTransferredSampleCount,
			InOutCumulativeContinentalGainCount);
	}

	void ExecuteV9ThesisShapedCollisionEventRedesign(
		FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples,
		const FTectonicPlanetV6CollisionShadowDiagnostic& ShadowDiagnostic,
		const bool bUseRidgeConcentratedSurge,
		TMap<uint64, int32>& InOutLastExecutedSolveIndexByKey,
		const int32 CurrentSolveIndex,
		TArray<uint8>& OutExecutionMask,
		TArray<uint8>& OutTransferMask,
		TArray<uint8>& OutTerraneComponentMask,
		TArray<uint8>& InOutCumulativeExecutionMask,
		TArray<uint8>& InOutCumulativeTransferMask,
		TArray<float>& InOutCumulativeElevationDeltaMaskKm,
		TArray<float>& InOutCumulativeContinentalGainMask,
		FTectonicPlanetV6CollisionExecutionDiagnostic& OutDiagnostic,
		int32& InOutCumulativeExecutionCount,
		int32& InOutCumulativeAffectedSampleVisits,
		int32& InOutCumulativeContinentalGainCount,
		int32& InOutCumulativeOwnershipChangeCount,
		int32& InOutCumulativeTransferredSampleVisits,
		int32& InOutCumulativeTransferredContinentalSampleCount,
		double& InOutCumulativeElevationDeltaKm,
		double& InOutCumulativeMaxElevationDeltaKm)
	{
		constexpr int32 CollisionExecutionPairCooldownSolves = 3;
		constexpr int32 MaxCollisionExecutionsPerSolve = 3;
		constexpr double CollisionGlobalDistanceRad = 0.659;
		constexpr double MinCollisionRadiusRad = 0.05;
		constexpr double MinTerraneDetectionRadiusRad = 0.08;
		constexpr double MaxAllowedEventOverlapFraction = 0.65;
		constexpr int32 MinThesisTransferSamples = 18;
		constexpr int32 MaxThesisTransferSamples = 96;
		constexpr double MaxPerSampleSurgeDeltaKm = 3.25;
		constexpr double MaxPerSampleTransferDeltaKm = 0.85;
		constexpr double CollisionCoefficient = 0.013;

		struct FThesisTransferCandidate
		{
			int32 SampleIndex = INDEX_NONE;
			double DistanceFromFrontKm = TNumericLimits<double>::Max();
			bool bBoundaryLocal = false;
		};

		struct FPlannedThesisCollisionEvent
		{
			const FTectonicPlanetV6CollisionShadowTopPair* Pair = nullptr;
			uint64 PairKey = 0;
			int32 OverridingPlateId = INDEX_NONE;
			int32 SubductingPlateId = INDEX_NONE;
			FVector3d CollisionCenter = FVector3d::ZeroVector;
			TArray<int32> SeedSampleIndices;
			TArray<int32> CollisionSeedSampleIndices;
			TArray<int32> TerraneComponentIndices;
			TArray<FThesisTransferCandidate> TransferCandidates;
			int32 TransferRejectedByLocalityCount = 0;
			int32 TransferRejectedByContinentalityCount = 0;
			int32 TransferRejectedByCapCount = 0;
			int32 TransferRejectedByConnectivityCount = 0;
			int32 TransferCandidateSupportCount = 0;
			int32 TransferAnchorSeedCount = 0;
			int32 DonorTerraneComponentSize = 0;
			int32 MaxTransferSamples = 0;
			int32 EffectiveMassSampleCount = 0;
			double RelativeSpeedKmPerMy = 0.0;
			double TerraneAreaSr = 0.0;
			double Xi = 0.0;
			double SurgeRadiusRad = 0.0;
			double TransferInfluenceRadiusRad = 0.0;
			double CollisionStrength01 = 0.0;
			double CollisionCoefficientScale = 1.0;
			FVector3d SutureFallbackAxis = FVector3d::ZeroVector;
		};

		struct FReservedInfluenceZone
		{
			FVector3d Center = FVector3d::ZeroVector;
			double RadiusRad = 0.0;
		};

		OutDiagnostic = FTectonicPlanetV6CollisionExecutionDiagnostic{};
		OutExecutionMask.Init(0, Planet.Samples.Num());
		OutTransferMask.Init(0, Planet.Samples.Num());
		OutTerraneComponentMask.Init(0, Planet.Samples.Num());
		OutDiagnostic.CumulativeExecutedCollisionCount = InOutCumulativeExecutionCount;
		if (InOutCumulativeExecutionMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeExecutionMask.Init(0, Planet.Samples.Num());
		}
		if (InOutCumulativeTransferMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeTransferMask.Init(0, Planet.Samples.Num());
		}
		if (InOutCumulativeElevationDeltaMaskKm.Num() != Planet.Samples.Num())
		{
			InOutCumulativeElevationDeltaMaskKm.Init(0.0f, Planet.Samples.Num());
		}
		if (InOutCumulativeContinentalGainMask.Num() != Planet.Samples.Num())
		{
			InOutCumulativeContinentalGainMask.Init(0.0f, Planet.Samples.Num());
		}
		OutDiagnostic.CumulativeCollisionAffectedSampleVisits = InOutCumulativeAffectedSampleVisits;
		OutDiagnostic.CumulativeCollisionDrivenContinentalGainCount = InOutCumulativeContinentalGainCount;
		OutDiagnostic.CumulativeCollisionDrivenOwnershipChangeCount = InOutCumulativeOwnershipChangeCount;
		OutDiagnostic.CumulativeCollisionTransferredSampleVisits = InOutCumulativeTransferredSampleVisits;
		OutDiagnostic.CumulativeCollisionTransferredContinentalSampleCount =
			InOutCumulativeTransferredContinentalSampleCount;
		OutDiagnostic.CumulativeMeanElevationDeltaKm =
			InOutCumulativeAffectedSampleVisits > 0
				? InOutCumulativeElevationDeltaKm /
					static_cast<double>(InOutCumulativeAffectedSampleVisits)
				: 0.0;
		OutDiagnostic.CumulativeMaxElevationDeltaKm = InOutCumulativeMaxElevationDeltaKm;
		for (const uint8 Value : InOutCumulativeExecutionMask)
		{
			OutDiagnostic.CumulativeCollisionAffectedSampleCount += Value != 0 ? 1 : 0;
		}
		for (const uint8 Value : InOutCumulativeTransferMask)
		{
			OutDiagnostic.CumulativeCollisionTransferredSampleCount += Value != 0 ? 1 : 0;
		}
		ResetV9CollisionExecutionLegacyStats(Planet);

		if (Planet.Samples.IsEmpty() || ResolvedSamples.Num() != Planet.Samples.Num())
		{
			return;
		}

		const double AverageSampleAreaSr = (4.0 * PI) / static_cast<double>(Planet.Samples.Num());
		const double TypicalPlateSize =
			static_cast<double>(Planet.Samples.Num()) /
			static_cast<double>(FMath::Max(Planet.Plates.Num(), 1));

		const auto CountSamplesForPlate =
			[&Planet](const int32 PlateId)
		{
			int32 Count = 0;
			for (const FSample& Sample : Planet.Samples)
			{
				Count += Sample.PlateId == PlateId ? 1 : 0;
			}
			return Count;
		};

		const auto BuildCollisionSutureFrame =
			[&Planet](
				const FVector3d& CollisionCenter,
				const TArray<int32>& PreferredSampleIndices,
				const TArray<int32>& SecondarySampleIndices,
				const FVector3d& FallbackAxis,
				FVector3d& OutSutureAxis,
				FVector3d& OutPerpendicularAxis,
				double& OutBeltLengthEstimateRad) -> bool
		{
			OutSutureAxis = FVector3d::ZeroVector;
			OutPerpendicularAxis = FVector3d::ZeroVector;
			OutBeltLengthEstimateRad = 0.0;

			const FVector3d CenterNormal = CollisionCenter.GetSafeNormal();
			if (CenterNormal.IsNearlyZero())
			{
				return false;
			}

			auto BuildBasisFromSamples =
				[&Planet, &CenterNormal](
					const TArray<int32>& SampleIndices,
					FVector3d& OutBasisX,
					FVector3d& OutBasisY) -> bool
			{
				for (const int32 SampleIndex : SampleIndices)
				{
					if (!Planet.Samples.IsValidIndex(SampleIndex))
					{
						continue;
					}
					const FVector3d SampleNormal =
						Planet.Samples[SampleIndex].Position.GetSafeNormal();
					const FVector3d TangentProjection =
						SampleNormal - (CenterNormal * FVector3d::DotProduct(SampleNormal, CenterNormal));
					if (TangentProjection.SquaredLength() <= UE_DOUBLE_SMALL_NUMBER)
					{
						continue;
					}
					OutBasisX = TangentProjection.GetSafeNormal();
					OutBasisY = FVector3d::CrossProduct(CenterNormal, OutBasisX).GetSafeNormal();
					return !OutBasisY.IsNearlyZero();
				}
				return false;
			};

			FVector3d BasisX = FVector3d::ZeroVector;
			FVector3d BasisY = FVector3d::ZeroVector;
			if (!BuildBasisFromSamples(PreferredSampleIndices, BasisX, BasisY) &&
				!BuildBasisFromSamples(SecondarySampleIndices, BasisX, BasisY))
			{
				const FVector3d FallbackTangent =
					FallbackAxis - (CenterNormal * FVector3d::DotProduct(FallbackAxis, CenterNormal));
				if (FallbackTangent.SquaredLength() <= UE_DOUBLE_SMALL_NUMBER)
				{
					return false;
				}
				BasisX = FallbackTangent.GetSafeNormal();
				BasisY = FVector3d::CrossProduct(CenterNormal, BasisX).GetSafeNormal();
				if (BasisY.IsNearlyZero())
				{
					return false;
				}
			}

			TArray<FVector2d> ProjectedPoints;
			ProjectedPoints.Reserve(PreferredSampleIndices.Num() + SecondarySampleIndices.Num());
			auto AppendProjectedPoints =
				[&Planet, &CenterNormal, &BasisX, &BasisY, &ProjectedPoints](
					const TArray<int32>& SampleIndices)
			{
				for (const int32 SampleIndex : SampleIndices)
				{
					if (!Planet.Samples.IsValidIndex(SampleIndex))
					{
						continue;
					}
					const FVector3d SampleNormal =
						Planet.Samples[SampleIndex].Position.GetSafeNormal();
					const double AngularDistance =
						ComputeGeodesicDistance(CenterNormal, SampleNormal);
					if (AngularDistance <= UE_DOUBLE_SMALL_NUMBER)
					{
						continue;
					}
					const FVector3d TangentProjection =
						SampleNormal - (CenterNormal * FVector3d::DotProduct(SampleNormal, CenterNormal));
					if (TangentProjection.SquaredLength() <= UE_DOUBLE_SMALL_NUMBER)
					{
						continue;
					}
					const FVector3d TangentDirection = TangentProjection.GetSafeNormal();
					ProjectedPoints.Add(
						FVector2d(
							FVector3d::DotProduct(TangentDirection, BasisX) * AngularDistance,
							FVector3d::DotProduct(TangentDirection, BasisY) * AngularDistance));
				}
			};

			AppendProjectedPoints(PreferredSampleIndices);
			AppendProjectedPoints(SecondarySampleIndices);

			if (ProjectedPoints.Num() < 2)
			{
				OutSutureAxis = BasisX;
				OutPerpendicularAxis = BasisY;
				OutBeltLengthEstimateRad = 0.0;
				return true;
			}

			int32 BestI = 0;
			int32 BestJ = 1;
			double BestDistanceSquared = 0.0;
			for (int32 I = 0; I < ProjectedPoints.Num(); ++I)
			{
				for (int32 J = I + 1; J < ProjectedPoints.Num(); ++J)
				{
					const double DistanceSquared =
						(ProjectedPoints[J] - ProjectedPoints[I]).SquaredLength();
					if (DistanceSquared > BestDistanceSquared)
					{
						BestDistanceSquared = DistanceSquared;
						BestI = I;
						BestJ = J;
					}
				}
			}

			const FVector2d Axis2D =
				(ProjectedPoints[BestJ] - ProjectedPoints[BestI]).GetSafeNormal();
			if (!Axis2D.IsNearlyZero())
			{
				OutSutureAxis =
					((BasisX * Axis2D.X) + (BasisY * Axis2D.Y)).GetSafeNormal();
			}
			if (OutSutureAxis.IsNearlyZero())
			{
				OutSutureAxis = BasisX;
			}
			OutPerpendicularAxis =
				FVector3d::CrossProduct(CenterNormal, OutSutureAxis).GetSafeNormal();
			if (OutPerpendicularAxis.IsNearlyZero())
			{
				return false;
			}

			double MaxAbsAlongRad = 0.0;
			for (const FVector2d& Point : ProjectedPoints)
			{
				const double AlongRad = FMath::Abs((Point.X * Axis2D.X) + (Point.Y * Axis2D.Y));
				MaxAbsAlongRad = FMath::Max(MaxAbsAlongRad, AlongRad);
			}
			OutBeltLengthEstimateRad = MaxAbsAlongRad * 2.0;
			return true;
		};

		const auto ProjectOntoCollisionSuture =
			[](const FVector3d& CollisionCenter,
				const FVector3d& SutureAxis,
				const FVector3d& PerpendicularAxis,
				const FVector3d& SamplePosition,
				double& OutAlongRad,
				double& OutPerpendicularRad)
		{
			OutAlongRad = 0.0;
			OutPerpendicularRad = 0.0;
			const FVector3d CenterNormal = CollisionCenter.GetSafeNormal();
			const FVector3d SampleNormal = SamplePosition.GetSafeNormal();
			const double AngularDistance =
				ComputeGeodesicDistance(CenterNormal, SampleNormal);
			if (AngularDistance <= UE_DOUBLE_SMALL_NUMBER)
			{
				return;
			}
			const FVector3d TangentProjection =
				SampleNormal - (CenterNormal * FVector3d::DotProduct(SampleNormal, CenterNormal));
			if (TangentProjection.SquaredLength() <= UE_DOUBLE_SMALL_NUMBER)
			{
				return;
			}
			const FVector3d TangentDirection = TangentProjection.GetSafeNormal();
			OutAlongRad =
				FVector3d::DotProduct(TangentDirection, SutureAxis) * AngularDistance;
			OutPerpendicularRad =
				FVector3d::DotProduct(TangentDirection, PerpendicularAxis) * AngularDistance;
		};

		TArray<FTectonicPlanetV6CollisionShadowTopPair> ExecutionPairs = ShadowDiagnostic.TopPairs;
		ExecutionPairs.Sort([](
			const FTectonicPlanetV6CollisionShadowTopPair& Left,
			const FTectonicPlanetV6CollisionShadowTopPair& Right)
		{
			if (Left.bQualified != Right.bQualified) { return Left.bQualified; }
			if (Left.CollisionSampleCount != Right.CollisionSampleCount)
			{
				return Left.CollisionSampleCount > Right.CollisionSampleCount;
			}
			if (Left.ObservationCount != Right.ObservationCount)
			{
				return Left.ObservationCount > Right.ObservationCount;
			}
			if (!FMath::IsNearlyEqual(
					Left.AccumulatedPenetrationKm,
					Right.AccumulatedPenetrationKm,
					TriangleEpsilon))
			{
				return Left.AccumulatedPenetrationKm > Right.AccumulatedPenetrationKm;
			}
			if (Left.ContinentalQualifiedSampleCount != Right.ContinentalQualifiedSampleCount)
			{
				return Left.ContinentalQualifiedSampleCount > Right.ContinentalQualifiedSampleCount;
			}
			if (Left.SupportSampleCount != Right.SupportSampleCount)
			{
				return Left.SupportSampleCount > Right.SupportSampleCount;
			}
			return Left.PlateA < Right.PlateA ||
				(Left.PlateA == Right.PlateA && Left.PlateB < Right.PlateB);
		});

		int32 QualifiedCount = 0;
		for (const FTectonicPlanetV6CollisionShadowTopPair& Pair : ExecutionPairs)
		{
			QualifiedCount += Pair.bQualified ? 1 : 0;
		}

		const auto TryBuildPlan =
			[&Planet,
			 &ResolvedSamples,
			 AverageSampleAreaSr,
			 TypicalPlateSize,
			 CollisionGlobalDistanceRad,
			 MinTerraneDetectionRadiusRad,
			 MinCollisionRadiusRad,
			 MinThesisTransferSamples,
			 MaxThesisTransferSamples](
				const FTectonicPlanetV6CollisionShadowTopPair& Pair,
				FPlannedThesisCollisionEvent& OutPlan) -> bool
		{
			OutPlan = FPlannedThesisCollisionEvent{};
			OutPlan.Pair = &Pair;
			OutPlan.PairKey = MakeOrderedPlatePairKey(Pair.PlateA, Pair.PlateB);
			OutPlan.OverridingPlateId =
				IsOverridingPlateLocal(Planet, Pair.PlateA, Pair.PlateB)
					? Pair.PlateA
					: Pair.PlateB;
			OutPlan.SubductingPlateId =
				OutPlan.OverridingPlateId == Pair.PlateA ? Pair.PlateB : Pair.PlateA;

			FVector3d CenterSum = FVector3d::ZeroVector;
			for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
			{
				if (!ResolvedSamples.IsValidIndex(SampleIndex))
				{
					continue;
				}
				const FTectonicPlanetV6ResolvedSample& Resolved = ResolvedSamples[SampleIndex];
				if (!Resolved.bActiveZoneSample ||
					Resolved.ActiveZonePrimaryPlateId == INDEX_NONE ||
					Resolved.ActiveZoneSecondaryPlateId == INDEX_NONE ||
					(Resolved.ActiveZoneCause != ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction &&
						Resolved.ActiveZoneCause != ETectonicPlanetV6ActiveZoneCause::CollisionContact))
				{
					continue;
				}
				if (MakeOrderedPlatePairKey(
						Resolved.ActiveZonePrimaryPlateId,
						Resolved.ActiveZoneSecondaryPlateId) != OutPlan.PairKey)
				{
					continue;
				}

				OutPlan.SeedSampleIndices.Add(SampleIndex);
				CenterSum += Planet.Samples[SampleIndex].Position;
				if (Resolved.ActiveZoneCause == ETectonicPlanetV6ActiveZoneCause::CollisionContact)
				{
					OutPlan.CollisionSeedSampleIndices.Add(SampleIndex);
				}
			}

			if (OutPlan.SeedSampleIndices.IsEmpty())
			{
				return false;
			}

			OutPlan.CollisionCenter = CenterSum.GetSafeNormal();
			if (OutPlan.CollisionCenter.IsNearlyZero())
			{
				OutPlan.CollisionCenter =
					Planet.Samples[OutPlan.SeedSampleIndices[0]].Position.GetSafeNormal();
			}
			if (OutPlan.CollisionCenter.IsNearlyZero())
			{
				return false;
			}

			const FPlate* OverridingPlate = FindPlateById(Planet, OutPlan.OverridingPlateId);
			const FPlate* SubductingPlate = FindPlateById(Planet, OutPlan.SubductingPlateId);
			if (OverridingPlate == nullptr || SubductingPlate == nullptr)
			{
				return false;
			}

			const auto IsRecipientAdjacent =
				[&Planet, &OutPlan](const int32 SampleIndex) -> bool
			{
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					return false;
				}
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (Planet.Samples.IsValidIndex(NeighborIndex) &&
						Planet.Samples[NeighborIndex].PlateId == OutPlan.OverridingPlateId)
					{
						return true;
					}
				}
				return false;
			};

			TArray<int32> DonorSeedSampleIndices;
			DonorSeedSampleIndices.Reserve(Pair.SupportSampleCount);
			TArray<int32> BoundaryAnchorSeedSampleIndices;
			BoundaryAnchorSeedSampleIndices.Reserve(Pair.SupportSampleCount);
			TArray<uint8> BoundaryLocalFlags;
			BoundaryLocalFlags.Init(0, Planet.Samples.Num());

			const auto TryAddDonorSeed =
				[&Planet,
				 &OutPlan,
				 &DonorSeedSampleIndices,
				 &BoundaryAnchorSeedSampleIndices,
				 &BoundaryLocalFlags,
				 &IsRecipientAdjacent](const int32 SampleIndex, int32& InOutRejectedByContinentality)
			{
				if (!Planet.Samples.IsValidIndex(SampleIndex))
				{
					return;
				}

				const FSample& Sample = Planet.Samples[SampleIndex];
				if (Sample.PlateId == OutPlan.SubductingPlateId &&
					Sample.ContinentalWeight >= 0.5f)
				{
					DonorSeedSampleIndices.AddUnique(SampleIndex);
					if (IsRecipientAdjacent(SampleIndex))
					{
						BoundaryAnchorSeedSampleIndices.AddUnique(SampleIndex);
						BoundaryLocalFlags[SampleIndex] = 1;
					}
				}
				else if (Sample.PlateId == OutPlan.SubductingPlateId)
				{
					++InOutRejectedByContinentality;
				}
			};

			for (const int32 SampleIndex : OutPlan.CollisionSeedSampleIndices)
			{
				TryAddDonorSeed(SampleIndex, OutPlan.TransferRejectedByContinentalityCount);
			}
			for (const int32 SampleIndex : OutPlan.SeedSampleIndices)
			{
				TryAddDonorSeed(SampleIndex, OutPlan.TransferRejectedByContinentalityCount);
			}
			if (DonorSeedSampleIndices.IsEmpty())
			{
				for (const int32 SeedSampleIndex : OutPlan.SeedSampleIndices)
				{
					if (!Planet.SampleAdjacency.IsValidIndex(SeedSampleIndex))
					{
						continue;
					}
					for (const int32 NeighborIndex : Planet.SampleAdjacency[SeedSampleIndex])
					{
						TryAddDonorSeed(NeighborIndex, OutPlan.TransferRejectedByContinentalityCount);
					}
				}
			}
			if (DonorSeedSampleIndices.IsEmpty())
			{
				return false;
			}

			for (const int32 AnchorSampleIndex : BoundaryAnchorSeedSampleIndices)
			{
				if (!Planet.SampleAdjacency.IsValidIndex(AnchorSampleIndex))
				{
					continue;
				}
				for (const int32 NeighborIndex : Planet.SampleAdjacency[AnchorSampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex))
					{
						continue;
					}
					const FSample& NeighborSample = Planet.Samples[NeighborIndex];
					if (NeighborSample.PlateId == OutPlan.SubductingPlateId &&
						NeighborSample.ContinentalWeight >= 0.5f)
					{
						BoundaryLocalFlags[NeighborIndex] = 1;
					}
				}
			}

			const FVector3d OverridingAxis = OverridingPlate->RotationAxis.IsNearlyZero()
				? FVector3d::ZeroVector
				: OverridingPlate->RotationAxis.GetSafeNormal();
			const FVector3d SubductingAxis = SubductingPlate->RotationAxis.IsNearlyZero()
				? FVector3d::ZeroVector
				: SubductingPlate->RotationAxis.GetSafeNormal();
			const FVector3d OverridingVelocity =
				FVector3d::CrossProduct(
					OverridingAxis * OverridingPlate->AngularSpeed,
					OutPlan.CollisionCenter);
			const FVector3d SubductingVelocity =
				FVector3d::CrossProduct(
					SubductingAxis * SubductingPlate->AngularSpeed,
					OutPlan.CollisionCenter);
			const FVector3d RelativeVelocity =
				OverridingVelocity - SubductingVelocity;
			OutPlan.RelativeSpeedKmPerMy =
				(RelativeVelocity.Length() * Planet.PlanetRadiusKm) /
				FMath::Max(DeltaTimeMyears, UE_DOUBLE_SMALL_NUMBER);
			OutPlan.SutureFallbackAxis =
				FVector3d::CrossProduct(
					OutPlan.CollisionCenter,
					RelativeVelocity).GetSafeNormal();

			const double SpeedFraction = FMath::Clamp(
				FMath::Max(OutPlan.RelativeSpeedKmPerMy, 0.0) /
					FMath::Max(MaxPlateSpeedKmPerMy, UE_DOUBLE_SMALL_NUMBER),
				0.0,
				4.0);
			const double SupportAreaFraction =
				static_cast<double>(FMath::Max3(
					Pair.ContinentalQualifiedSampleCount,
					FMath::Max(Pair.SupportSampleCount / 2, 1),
					FMath::Max(OutPlan.SeedSampleIndices.Num(), 1))) /
				FMath::Max(TypicalPlateSize, 1.0);
			const double PreliminaryXi =
				FMath::Sqrt(FMath::Clamp(SpeedFraction * SupportAreaFraction, 0.0, 4.0));
			const double TerraneDetectionRadiusRad = FMath::Min(
				CollisionGlobalDistanceRad,
				FMath::Max(
					CollisionGlobalDistanceRad * FMath::Max(PreliminaryXi, 0.22),
					MinTerraneDetectionRadiusRad));
			const double MaxTerraneDetectionDistanceKm =
				TerraneDetectionRadiusRad * Planet.PlanetRadiusKm;

			TArray<double> BestTerraneDistanceKm;
			BestTerraneDistanceKm.Init(TNumericLimits<double>::Max(), Planet.Samples.Num());
			TArray<uint8> TerraneFlags;
			TerraneFlags.Init(0, Planet.Samples.Num());
			TArray<int32> TerraneFrontier =
				!BoundaryAnchorSeedSampleIndices.IsEmpty()
					? BoundaryAnchorSeedSampleIndices
					: DonorSeedSampleIndices;
			for (const int32 SeedIndex : TerraneFrontier)
			{
				if (BestTerraneDistanceKm.IsValidIndex(SeedIndex))
				{
					BestTerraneDistanceKm[SeedIndex] = 0.0;
				}
			}

			while (!TerraneFrontier.IsEmpty())
			{
				int32 BestFrontierIndex = 0;
				double BestFrontierDistanceKm = BestTerraneDistanceKm[TerraneFrontier[0]];
				for (int32 FrontierIndex = 1; FrontierIndex < TerraneFrontier.Num(); ++FrontierIndex)
				{
					const double CandidateDistanceKm = BestTerraneDistanceKm[TerraneFrontier[FrontierIndex]];
					if (CandidateDistanceKm < BestFrontierDistanceKm)
					{
						BestFrontierIndex = FrontierIndex;
						BestFrontierDistanceKm = CandidateDistanceKm;
					}
				}

				const int32 SampleIndex = TerraneFrontier[BestFrontierIndex];
				TerraneFrontier.RemoveAtSwap(BestFrontierIndex, 1, EAllowShrinking::No);
				if (!Planet.Samples.IsValidIndex(SampleIndex) || TerraneFlags[SampleIndex] != 0)
				{
					continue;
				}

				const FSample& Sample = Planet.Samples[SampleIndex];
				if (Sample.PlateId != OutPlan.SubductingPlateId ||
					Sample.ContinentalWeight < 0.5f)
				{
					continue;
				}

				const double CenterDistanceKm =
					ComputeGeodesicDistance(
						Sample.Position.GetSafeNormal(),
						OutPlan.CollisionCenter) * Planet.PlanetRadiusKm;
				if (CenterDistanceKm > MaxTerraneDetectionDistanceKm + UE_DOUBLE_SMALL_NUMBER ||
					BestFrontierDistanceKm > MaxTerraneDetectionDistanceKm + UE_DOUBLE_SMALL_NUMBER)
				{
					continue;
				}

				TerraneFlags[SampleIndex] = 1;
				OutPlan.TerraneComponentIndices.Add(SampleIndex);
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				const FVector3d SamplePosition = Sample.Position.GetSafeNormal();
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) || TerraneFlags[NeighborIndex] != 0)
					{
						continue;
					}
					const FSample& NeighborSample = Planet.Samples[NeighborIndex];
					if (NeighborSample.PlateId != OutPlan.SubductingPlateId ||
						NeighborSample.ContinentalWeight < 0.5f)
					{
						continue;
					}

					const double EdgeDistanceKm =
						ComputeGeodesicDistance(
							SamplePosition,
							NeighborSample.Position.GetSafeNormal()) * Planet.PlanetRadiusKm;
					const double CandidateDistanceKm = BestFrontierDistanceKm + EdgeDistanceKm;
					if (CandidateDistanceKm + UE_DOUBLE_SMALL_NUMBER < BestTerraneDistanceKm[NeighborIndex] &&
						CandidateDistanceKm <= MaxTerraneDetectionDistanceKm + UE_DOUBLE_SMALL_NUMBER)
					{
						BestTerraneDistanceKm[NeighborIndex] = CandidateDistanceKm;
						TerraneFrontier.Add(NeighborIndex);
					}
				}
			}

			OutPlan.DonorTerraneComponentSize = OutPlan.TerraneComponentIndices.Num();
			if (OutPlan.DonorTerraneComponentSize == 0)
			{
				return false;
			}

			OutPlan.TerraneAreaSr =
				static_cast<double>(OutPlan.DonorTerraneComponentSize) * AverageSampleAreaSr;
			const double A0Sr =
				(4.0 * PI) / static_cast<double>(FMath::Max(Planet.Plates.Num(), 1));
			const double TerraneAreaFraction =
				OutPlan.TerraneAreaSr / FMath::Max(A0Sr, UE_DOUBLE_SMALL_NUMBER);
			OutPlan.Xi =
				FMath::Sqrt(FMath::Clamp(SpeedFraction * TerraneAreaFraction, 0.0, 4.0));

			const double ObservationStrength = FMath::Clamp(
				(static_cast<double>(Pair.ObservationCount) - 3.0) / 9.0,
				0.0,
				1.0);
			const double PenetrationStrength = FMath::Clamp(
				(Pair.AccumulatedPenetrationKm - 3000.0) / 9000.0,
				0.0,
				1.0);
			const double ContinentalStrength = FMath::Clamp(
				static_cast<double>(Pair.ContinentalQualifiedSampleCount) / 24.0,
				0.0,
				1.0);
			OutPlan.CollisionStrength01 = FMath::Clamp(
				(0.45 * ObservationStrength) +
				(0.35 * PenetrationStrength) +
				(0.20 * ContinentalStrength),
				0.0,
				1.0);
			OutPlan.CollisionCoefficientScale = 1.0 + (0.85 * OutPlan.CollisionStrength01);
			OutPlan.SurgeRadiusRad = FMath::Min(
				CollisionGlobalDistanceRad,
				FMath::Max(
					CollisionGlobalDistanceRad * FMath::Min(OutPlan.Xi, 1.0),
					MinCollisionRadiusRad));

			const double PatchSupportStrength = FMath::Clamp(
				static_cast<double>(FMath::Max(BoundaryAnchorSeedSampleIndices.Num(), 1)) / 18.0,
				0.0,
				1.0);
			OutPlan.TransferInfluenceRadiusRad = FMath::Min(
				CollisionGlobalDistanceRad,
				FMath::Max(
					MinCollisionRadiusRad * 1.6,
					OutPlan.SurgeRadiusRad *
						(1.15 + (0.35 * PenetrationStrength) + (0.20 * ContinentalStrength) +
							(0.15 * PatchSupportStrength))));

			const double MaxTransferDistanceKm =
				OutPlan.TransferInfluenceRadiusRad * Planet.PlanetRadiusKm;
			TArray<double> BestTransferDistanceKm;
			BestTransferDistanceKm.Init(TNumericLimits<double>::Max(), Planet.Samples.Num());
			TArray<uint8> AddedTransferCandidateFlags;
			AddedTransferCandidateFlags.Init(0, Planet.Samples.Num());
			TArray<int32> TransferFrontier =
				!BoundaryAnchorSeedSampleIndices.IsEmpty()
					? BoundaryAnchorSeedSampleIndices
					: DonorSeedSampleIndices;
			for (const int32 SeedIndex : TransferFrontier)
			{
				if (BestTransferDistanceKm.IsValidIndex(SeedIndex))
				{
					BestTransferDistanceKm[SeedIndex] = 0.0;
				}
			}

			while (!TransferFrontier.IsEmpty())
			{
				int32 BestFrontierIndex = 0;
				double BestFrontierDistanceKm = BestTransferDistanceKm[TransferFrontier[0]];
				for (int32 FrontierIndex = 1; FrontierIndex < TransferFrontier.Num(); ++FrontierIndex)
				{
					const double CandidateDistanceKm = BestTransferDistanceKm[TransferFrontier[FrontierIndex]];
					if (CandidateDistanceKm < BestFrontierDistanceKm)
					{
						BestFrontierIndex = FrontierIndex;
						BestFrontierDistanceKm = CandidateDistanceKm;
					}
				}

				const int32 SampleIndex = TransferFrontier[BestFrontierIndex];
				TransferFrontier.RemoveAtSwap(BestFrontierIndex, 1, EAllowShrinking::No);
				if (!Planet.Samples.IsValidIndex(SampleIndex) ||
					AddedTransferCandidateFlags[SampleIndex] != 0)
				{
					continue;
				}

				const FSample& Sample = Planet.Samples[SampleIndex];
				if (Sample.PlateId != OutPlan.SubductingPlateId ||
					Sample.ContinentalWeight < 0.5f)
				{
					++OutPlan.TransferRejectedByContinentalityCount;
					continue;
				}
				if (TerraneFlags[SampleIndex] == 0)
				{
					++OutPlan.TransferRejectedByConnectivityCount;
					continue;
				}

				const double CenterDistanceKm =
					ComputeGeodesicDistance(
						Sample.Position.GetSafeNormal(),
						OutPlan.CollisionCenter) * Planet.PlanetRadiusKm;
				if (CenterDistanceKm > (MaxTransferDistanceKm * 1.20) + UE_DOUBLE_SMALL_NUMBER ||
					BestFrontierDistanceKm > MaxTransferDistanceKm + UE_DOUBLE_SMALL_NUMBER)
				{
					++OutPlan.TransferRejectedByLocalityCount;
					continue;
				}

				const bool bDirectBoundaryLocal =
					OutPlan.CollisionSeedSampleIndices.Contains(SampleIndex) ||
					IsRecipientAdjacent(SampleIndex);
				const bool bBoundaryLocal =
					bDirectBoundaryLocal ||
					(BoundaryLocalFlags.IsValidIndex(SampleIndex) &&
						BoundaryLocalFlags[SampleIndex] != 0);
				if (!bBoundaryLocal &&
					BestFrontierDistanceKm > (MaxTransferDistanceKm * 0.55) + UE_DOUBLE_SMALL_NUMBER)
				{
					++OutPlan.TransferRejectedByLocalityCount;
					continue;
				}

				AddedTransferCandidateFlags[SampleIndex] = 1;
				OutPlan.TransferCandidates.Add(
					{ SampleIndex, BestFrontierDistanceKm, bBoundaryLocal });

				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				const FVector3d SamplePosition = Sample.Position.GetSafeNormal();
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						AddedTransferCandidateFlags[NeighborIndex] != 0 ||
						TerraneFlags[NeighborIndex] == 0)
					{
						continue;
					}
					const FSample& NeighborSample = Planet.Samples[NeighborIndex];
					if (NeighborSample.PlateId != OutPlan.SubductingPlateId ||
						NeighborSample.ContinentalWeight < 0.5f)
					{
						continue;
					}

					const double EdgeDistanceKm =
						ComputeGeodesicDistance(
							SamplePosition,
							NeighborSample.Position.GetSafeNormal()) * Planet.PlanetRadiusKm;
					const double CandidateDistanceKm = BestFrontierDistanceKm + EdgeDistanceKm;
					if (CandidateDistanceKm + UE_DOUBLE_SMALL_NUMBER < BestTransferDistanceKm[NeighborIndex] &&
						CandidateDistanceKm <= MaxTransferDistanceKm + UE_DOUBLE_SMALL_NUMBER)
					{
						BestTransferDistanceKm[NeighborIndex] = CandidateDistanceKm;
						TransferFrontier.Add(NeighborIndex);
					}
				}
			}

			OutPlan.TransferCandidateSupportCount = OutPlan.TransferCandidates.Num();
			OutPlan.TransferAnchorSeedCount =
				!BoundaryAnchorSeedSampleIndices.IsEmpty()
					? BoundaryAnchorSeedSampleIndices.Num()
					: DonorSeedSampleIndices.Num();
			if (OutPlan.TransferCandidates.IsEmpty())
			{
				return false;
			}

			OutPlan.TransferCandidates.Sort([](
				const FThesisTransferCandidate& Left,
				const FThesisTransferCandidate& Right)
			{
				if (Left.bBoundaryLocal != Right.bBoundaryLocal)
				{
					return Left.bBoundaryLocal;
				}
				if (!FMath::IsNearlyEqual(
						Left.DistanceFromFrontKm,
						Right.DistanceFromFrontKm,
						TriangleEpsilon))
				{
					return Left.DistanceFromFrontKm < Right.DistanceFromFrontKm;
				}
				return Left.SampleIndex < Right.SampleIndex;
			});

			OutPlan.MaxTransferSamples = FMath::Clamp(
				FMath::Max3(
					MinThesisTransferSamples,
					FMath::Min(
						OutPlan.TransferCandidateSupportCount,
						FMath::Max(
							Pair.ContinentalQualifiedSampleCount * 2,
							FMath::Max(12, OutPlan.TransferAnchorSeedCount * 3))),
					FMath::Min(
						OutPlan.TransferCandidateSupportCount,
						FMath::Max(
							OutPlan.DonorTerraneComponentSize / 3,
							FMath::Max(Pair.SupportSampleCount / 2, 18)))),
				MinThesisTransferSamples,
				MaxThesisTransferSamples);
			OutPlan.EffectiveMassSampleCount = FMath::Max3(
				OutPlan.DonorTerraneComponentSize,
				FMath::Max(Pair.SupportSampleCount, 1),
				FMath::Max(Pair.ContinentalQualifiedSampleCount * 2, 1));
			return true;
		};

		TSet<int32> UsedPlateIds;
		TArray<FReservedInfluenceZone> ReservedZones;
		ReservedZones.Reserve(MaxCollisionExecutionsPerSolve);
		int32 ExecutedCollisionCount = 0;
		int32 CurrentSolveContinentalGainCount = 0;
		int32 CurrentSolveOwnershipChangeCount = 0;
		int32 CurrentSolveTransferredSampleCount = 0;
		int32 CurrentSolveTransferredContinentalSampleCount = 0;
		double CurrentSolveTotalElevationDeltaKm = 0.0;
		double CurrentSolveMaxElevationDeltaKm = 0.0;
		bool bRepresentativeEventSet = false;

		for (const FTectonicPlanetV6CollisionShadowTopPair& Pair : ExecutionPairs)
		{
			if (!Pair.bQualified)
			{
				continue;
			}

			const uint64 PairKey = MakeOrderedPlatePairKey(Pair.PlateA, Pair.PlateB);
			if (const int32* LastExecutedSolveIndex = InOutLastExecutedSolveIndexByKey.Find(PairKey))
			{
				if ((CurrentSolveIndex - *LastExecutedSolveIndex) <= CollisionExecutionPairCooldownSolves)
				{
					++OutDiagnostic.CooldownSuppressedQualifiedCount;
					continue;
				}
			}

			if (UsedPlateIds.Contains(Pair.PlateA) || UsedPlateIds.Contains(Pair.PlateB))
			{
				++OutDiagnostic.PlateConflictSuppressedQualifiedCount;
				continue;
			}

			FPlannedThesisCollisionEvent Plan;
			if (!TryBuildPlan(Pair, Plan))
			{
				continue;
			}

			bool bOverlapsReservedZone = false;
			for (const FReservedInfluenceZone& Zone : ReservedZones)
			{
				const double CenterDistanceRad =
					ComputeGeodesicDistance(Plan.CollisionCenter, Zone.Center);
				if (CenterDistanceRad <
					(Plan.SurgeRadiusRad + Zone.RadiusRad) * MaxAllowedEventOverlapFraction)
				{
					bOverlapsReservedZone = true;
					break;
				}
			}
			if (bOverlapsReservedZone)
			{
				++OutDiagnostic.OverlapSuppressedQualifiedCount;
				continue;
			}

			for (const int32 SampleIndex : Plan.TerraneComponentIndices)
			{
				OutTerraneComponentMask[SampleIndex] = 1;
			}

			const int32 DonorSamplesBefore = CountSamplesForPlate(Plan.SubductingPlateId);
			const int32 RecipientSamplesBefore = CountSamplesForPlate(Plan.OverridingPlateId);
			const double InverseSampleCount = 1.0 / static_cast<double>(Planet.Samples.Num());

			TArray<int32> TransferredSampleIndices;
			TransferredSampleIndices.Reserve(Plan.MaxTransferSamples);
			int32 EventTransferredContinentalCount = 0;
			int32 EventTransferBoundaryLocalCount = 0;
			double EventTotalElevationDeltaKm = 0.0;
			double EventMaxElevationDeltaKm = 0.0;

			for (const FThesisTransferCandidate& Candidate : Plan.TransferCandidates)
			{
				if (TransferredSampleIndices.Num() >= Plan.MaxTransferSamples)
				{
					++Plan.TransferRejectedByCapCount;
					continue;
				}
				if (!Planet.Samples.IsValidIndex(Candidate.SampleIndex))
				{
					continue;
				}

				FSample& Sample = Planet.Samples[Candidate.SampleIndex];
				if (Sample.PlateId != Plan.SubductingPlateId ||
					Sample.ContinentalWeight < 0.5f)
				{
					continue;
				}

				const float PreviousElevation = Sample.Elevation;
				const double TransferKernel = FTectonicPlanet::CollisionBiweightKernel(
					ComputeGeodesicDistance(
						Sample.Position.GetSafeNormal(),
						Plan.CollisionCenter),
					FMath::Max(Plan.TransferInfluenceRadiusRad, MinCollisionRadiusRad));
				const double StructuralElevationDeltaKm = FMath::Min(
					MaxPerSampleTransferDeltaKm,
					0.35 *
						(0.95 + (0.45 * Plan.CollisionStrength01)) *
						FMath::Max(TransferKernel, 0.30));

				RemoveV6CarriedSampleForCanonicalVertex(
					Planet, Plan.SubductingPlateId, Candidate.SampleIndex);
				Sample.PlateId = Plan.OverridingPlateId;
				Sample.ContinentalWeight = 1.0f;
				Sample.Thickness = FMath::Max(
					Sample.Thickness,
					static_cast<float>(ContinentalThicknessKm));
				Sample.Elevation = FMath::Clamp(
					static_cast<float>(
						static_cast<double>(Sample.Elevation) + StructuralElevationDeltaKm),
					0.0f,
					static_cast<float>(ElevationCeilingKm));
				Sample.OrogenyType = EOrogenyType::Himalayan;
				UpsertV6CarriedSampleForCanonicalVertex(
					Planet, Plan.OverridingPlateId, Candidate.SampleIndex);

				const double AppliedElevationDeltaKm =
					static_cast<double>(Sample.Elevation - PreviousElevation);
				EventTotalElevationDeltaKm += AppliedElevationDeltaKm;
				EventMaxElevationDeltaKm =
					FMath::Max(EventMaxElevationDeltaKm, AppliedElevationDeltaKm);
				++EventTransferredContinentalCount;
				EventTransferBoundaryLocalCount += Candidate.bBoundaryLocal ? 1 : 0;
				TransferredSampleIndices.Add(Candidate.SampleIndex);
				OutTransferMask[Candidate.SampleIndex] = 1;
				InOutCumulativeTransferMask[Candidate.SampleIndex] = 1;
				OutExecutionMask[Candidate.SampleIndex] = 1;
				InOutCumulativeExecutionMask[Candidate.SampleIndex] = 1;
				InOutCumulativeElevationDeltaMaskKm[Candidate.SampleIndex] +=
					static_cast<float>(AppliedElevationDeltaKm);
			}

			if (TransferredSampleIndices.IsEmpty())
			{
				continue;
			}

			RebuildV6MembershipFromCanonical(Planet);
			RecomputeV6BoundaryFlagsFromCanonical(Planet);

			TArray<int32> EventAffectedSampleIndices;
			EventAffectedSampleIndices.Reserve(Planet.Samples.Num() / 24);
			int32 EventContinentalGainCount = 0;
			FVector3d EventSutureAxis = FVector3d::ZeroVector;
			FVector3d EventPerpendicularAxis = FVector3d::ZeroVector;
			double EventSutureHalfWidthRad = 0.0;
			double EventBeltLengthEstimateRad = 0.0;
			double EventMeanAbsAlongSutureRad = 0.0;
			double EventMeanAbsPerpendicularRad = 0.0;
			int32 EventRidgeCoreAffectedCount = 0;
			int32 EventRidgeFlankAffectedCount = 0;
			const double TerraneAreaFraction =
				static_cast<double>(Plan.DonorTerraneComponentSize) /
				FMath::Max(TypicalPlateSize, 1.0);
			const double SurgeMassFactor = FMath::Clamp(
				(TerraneAreaFraction * 0.20) +
					(static_cast<double>(Plan.Pair->ContinentalQualifiedSampleCount) /
						FMath::Max(TypicalPlateSize, 1.0) * 0.08),
				0.012,
				0.085);
			if (bUseRidgeConcentratedSurge)
			{
				BuildCollisionSutureFrame(
					Plan.CollisionCenter,
					TransferredSampleIndices,
					Plan.CollisionSeedSampleIndices,
					Plan.SutureFallbackAxis,
					EventSutureAxis,
					EventPerpendicularAxis,
					EventBeltLengthEstimateRad);
				EventSutureHalfWidthRad = FMath::Clamp(
					Plan.SurgeRadiusRad * (0.24 + (0.04 * Plan.CollisionStrength01)),
					0.015,
					Plan.SurgeRadiusRad * 0.32);
				EventBeltLengthEstimateRad = FMath::Max(
					EventBeltLengthEstimateRad,
					Plan.SurgeRadiusRad * 2.0);
			}
			const double RidgeEnergyCompensation =
				(bUseRidgeConcentratedSurge && EventSutureHalfWidthRad > UE_DOUBLE_SMALL_NUMBER)
					? FMath::Clamp(Plan.SurgeRadiusRad / EventSutureHalfWidthRad, 1.0, 4.0)
					: 1.0;

			for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
			{
				FSample& Sample = Planet.Samples[SampleIndex];
				const FVector3d SamplePosition = Sample.Position.GetSafeNormal();
				double Kernel = 0.0;
				double AlongSutureRad = 0.0;
				double PerpendicularRad = 0.0;
				if (bUseRidgeConcentratedSurge &&
					!EventSutureAxis.IsNearlyZero() &&
					!EventPerpendicularAxis.IsNearlyZero())
				{
					ProjectOntoCollisionSuture(
						Plan.CollisionCenter,
						EventSutureAxis,
						EventPerpendicularAxis,
						SamplePosition,
						AlongSutureRad,
						PerpendicularRad);
					if (FMath::Abs(AlongSutureRad) >= Plan.SurgeRadiusRad ||
						FMath::Abs(PerpendicularRad) >= EventSutureHalfWidthRad)
					{
						continue;
					}
					const double AlongKernel =
						FTectonicPlanet::CollisionBiweightKernel(
							FMath::Abs(AlongSutureRad),
							Plan.SurgeRadiusRad);
					const double PerpendicularKernel =
						FTectonicPlanet::CollisionBiweightKernel(
							FMath::Abs(PerpendicularRad),
							EventSutureHalfWidthRad);
					Kernel = AlongKernel * PerpendicularKernel * RidgeEnergyCompensation;
				}
				else
				{
					const double DistanceRad = ComputeGeodesicDistance(
						SamplePosition,
						Plan.CollisionCenter);
					if (DistanceRad >= Plan.SurgeRadiusRad)
					{
						continue;
					}
					Kernel =
						FTectonicPlanet::CollisionBiweightKernel(
							DistanceRad,
							Plan.SurgeRadiusRad);
				}
				if (Kernel <= 0.0)
				{
					continue;
				}

				const float PreviousContinentalWeight = Sample.ContinentalWeight;
				const float PreviousElevation = Sample.Elevation;
				const double RawElevationDeltaKm =
					CollisionCoefficient *
					Plan.CollisionCoefficientScale *
					SurgeMassFactor *
					Kernel *
					Planet.PlanetRadiusKm;
				const double ElevationDeltaKm =
					FMath::Min(
						RawElevationDeltaKm,
						MaxPerSampleSurgeDeltaKm * FMath::Max(Kernel, 0.25));

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
				Sample.Thickness = FMath::Max(
					Sample.Thickness,
					static_cast<float>(ContinentalThicknessKm));

				const bool bContinentalGain =
					PreviousContinentalWeight < 0.5f &&
					Sample.ContinentalWeight >= 0.5f;
				EventContinentalGainCount += bContinentalGain ? 1 : 0;
				const double AppliedElevationDeltaKm =
					static_cast<double>(Sample.Elevation - PreviousElevation);
				EventTotalElevationDeltaKm += AppliedElevationDeltaKm;
				EventMaxElevationDeltaKm =
					FMath::Max(EventMaxElevationDeltaKm, AppliedElevationDeltaKm);
				EventAffectedSampleIndices.Add(SampleIndex);
				if (bUseRidgeConcentratedSurge)
				{
					EventMeanAbsAlongSutureRad += FMath::Abs(AlongSutureRad);
					EventMeanAbsPerpendicularRad += FMath::Abs(PerpendicularRad);
					if (FMath::Abs(PerpendicularRad) <= (EventSutureHalfWidthRad * 0.5))
					{
						++EventRidgeCoreAffectedCount;
					}
					else
					{
						++EventRidgeFlankAffectedCount;
					}
				}
				if (OutExecutionMask[SampleIndex] == 0)
				{
					OutExecutionMask[SampleIndex] = 1;
					InOutCumulativeExecutionMask[SampleIndex] = 1;
				}
				InOutCumulativeElevationDeltaMaskKm[SampleIndex] +=
					static_cast<float>(AppliedElevationDeltaKm);
				if (bContinentalGain)
				{
					InOutCumulativeContinentalGainMask[SampleIndex] += 1.0f;
				}
			}

			if (bUseRidgeConcentratedSurge && !EventAffectedSampleIndices.IsEmpty())
			{
				EventMeanAbsAlongSutureRad /=
					static_cast<double>(EventAffectedSampleIndices.Num());
				EventMeanAbsPerpendicularRad /=
					static_cast<double>(EventAffectedSampleIndices.Num());
			}

			SyncV6CarriedSamplesFromCanonicalIndices(Planet, EventAffectedSampleIndices);
			Planet.ComputePlateScores();

			const int32 DonorSamplesAfter = CountSamplesForPlate(Plan.SubductingPlateId);
			const int32 RecipientSamplesAfter = CountSamplesForPlate(Plan.OverridingPlateId);
			const double DonorPlateShareBefore =
				static_cast<double>(DonorSamplesBefore) * InverseSampleCount;
			const double DonorPlateShareAfter =
				static_cast<double>(DonorSamplesAfter) * InverseSampleCount;
			const double RecipientPlateShareBefore =
				static_cast<double>(RecipientSamplesBefore) * InverseSampleCount;
			const double RecipientPlateShareAfter =
				static_cast<double>(RecipientSamplesAfter) * InverseSampleCount;

			InOutLastExecutedSolveIndexByKey.Add(Plan.PairKey, CurrentSolveIndex);
			++ExecutedCollisionCount;
			++InOutCumulativeExecutionCount;
			InOutCumulativeAffectedSampleVisits += EventAffectedSampleIndices.Num();
			InOutCumulativeContinentalGainCount += EventContinentalGainCount;
			InOutCumulativeOwnershipChangeCount += TransferredSampleIndices.Num();
			InOutCumulativeTransferredSampleVisits += TransferredSampleIndices.Num();
			InOutCumulativeTransferredContinentalSampleCount += EventTransferredContinentalCount;
			InOutCumulativeElevationDeltaKm += EventTotalElevationDeltaKm;
			InOutCumulativeMaxElevationDeltaKm =
				FMath::Max(InOutCumulativeMaxElevationDeltaKm, EventMaxElevationDeltaKm);

			CurrentSolveContinentalGainCount += EventContinentalGainCount;
			CurrentSolveOwnershipChangeCount += TransferredSampleIndices.Num();
			CurrentSolveTransferredSampleCount += TransferredSampleIndices.Num();
			CurrentSolveTransferredContinentalSampleCount += EventTransferredContinentalCount;
			CurrentSolveTotalElevationDeltaKm += EventTotalElevationDeltaKm;
			CurrentSolveMaxElevationDeltaKm =
				FMath::Max(CurrentSolveMaxElevationDeltaKm, EventMaxElevationDeltaKm);

			if (!bRepresentativeEventSet)
			{
				bRepresentativeEventSet = true;
				OutDiagnostic.ExecutedPlateA = Plan.Pair->PlateA;
				OutDiagnostic.ExecutedPlateB = Plan.Pair->PlateB;
				OutDiagnostic.ExecutedOverridingPlateId = Plan.OverridingPlateId;
				OutDiagnostic.ExecutedSubductingPlateId = Plan.SubductingPlateId;
				OutDiagnostic.ExecutedObservationCount = Plan.Pair->ObservationCount;
				OutDiagnostic.ExecutedAccumulatedPenetrationKm = Plan.Pair->AccumulatedPenetrationKm;
				OutDiagnostic.ExecutedMeanConvergenceKmPerMy = Plan.Pair->MeanConvergenceKmPerMy;
				OutDiagnostic.ExecutedMaxConvergenceKmPerMy = Plan.Pair->MaxConvergenceKmPerMy;
				OutDiagnostic.ExecutedSupportSampleCount = Plan.Pair->SupportSampleCount;
				OutDiagnostic.ExecutedSupportTriangleCount = Plan.Pair->SupportTriangleCount;
				OutDiagnostic.ExecutedContinentalSupportPlateACount =
					Plan.Pair->ContinentalSupportPlateACount;
				OutDiagnostic.ExecutedContinentalSupportPlateBCount =
					Plan.Pair->ContinentalSupportPlateBCount;
				OutDiagnostic.ExecutedContinentalQualifiedSampleCount =
					Plan.Pair->ContinentalQualifiedSampleCount;
				OutDiagnostic.ExecutedSeedSampleCount = Plan.SeedSampleIndices.Num();
				OutDiagnostic.ExecutedCollisionSeedSampleCount = Plan.CollisionSeedSampleIndices.Num();
				OutDiagnostic.ExecutedEffectiveMassSampleCount = Plan.EffectiveMassSampleCount;
				OutDiagnostic.ExecutedTransferRejectedByLocalityCount =
					Plan.TransferRejectedByLocalityCount;
				OutDiagnostic.ExecutedTransferRejectedByContinentalityCount =
					Plan.TransferRejectedByContinentalityCount;
				OutDiagnostic.ExecutedTransferRejectedByCapCount =
					Plan.TransferRejectedByCapCount;
				OutDiagnostic.ExecutedTransferRejectedByConnectivityCount =
					Plan.TransferRejectedByConnectivityCount;
				OutDiagnostic.ExecutedTransferBoundaryLocalSampleCount =
					EventTransferBoundaryLocalCount;
				OutDiagnostic.ExecutedTransferCandidateSupportCount =
					Plan.TransferCandidateSupportCount;
				OutDiagnostic.ExecutedTransferAnchorSeedCount =
					Plan.TransferAnchorSeedCount;
				OutDiagnostic.ExecutedDonorTerraneComponentSize =
					Plan.DonorTerraneComponentSize;
				OutDiagnostic.ExecutedTransferredComponentSize =
					TransferredSampleIndices.Num();
				OutDiagnostic.ExecutedTerraneAreaSr = Plan.TerraneAreaSr;
				OutDiagnostic.ExecutedXi = Plan.Xi;
				OutDiagnostic.ExecutedRelativeSpeedKmPerMy = Plan.RelativeSpeedKmPerMy;
				OutDiagnostic.ExecutedSutureAxis = EventSutureAxis;
				OutDiagnostic.ExecutedSutureHalfWidthRad = EventSutureHalfWidthRad;
				OutDiagnostic.ExecutedSutureBeltLengthEstimateRad = EventBeltLengthEstimateRad;
				OutDiagnostic.ExecutedMeanAbsAlongSutureRad = EventMeanAbsAlongSutureRad;
				OutDiagnostic.ExecutedMeanAbsPerpendicularRad = EventMeanAbsPerpendicularRad;
				OutDiagnostic.ExecutedRidgeCoreAffectedSampleCount = EventRidgeCoreAffectedCount;
				OutDiagnostic.ExecutedRidgeFlankAffectedSampleCount = EventRidgeFlankAffectedCount;
				OutDiagnostic.ExecutedInfluenceRadiusRad = Plan.SurgeRadiusRad;
				OutDiagnostic.ExecutedTransferInfluenceRadiusRad =
					Plan.TransferInfluenceRadiusRad;
				OutDiagnostic.ExecutedMeanElevationDeltaKm =
					EventAffectedSampleIndices.IsEmpty()
						? 0.0
						: EventTotalElevationDeltaKm /
							static_cast<double>(EventAffectedSampleIndices.Num());
				OutDiagnostic.ExecutedMaxElevationDeltaKm = EventMaxElevationDeltaKm;
				OutDiagnostic.ExecutedStrengthScale = Plan.CollisionCoefficientScale;
				OutDiagnostic.ExecutedDonorPlateShareBefore = DonorPlateShareBefore;
				OutDiagnostic.ExecutedDonorPlateShareAfter = DonorPlateShareAfter;
				OutDiagnostic.ExecutedDonorPlateShareDelta =
					DonorPlateShareAfter - DonorPlateShareBefore;
				OutDiagnostic.ExecutedRecipientPlateShareBefore = RecipientPlateShareBefore;
				OutDiagnostic.ExecutedRecipientPlateShareAfter = RecipientPlateShareAfter;
				OutDiagnostic.ExecutedRecipientPlateShareDelta =
					RecipientPlateShareAfter - RecipientPlateShareBefore;
			}

			ReservedZones.Add({ Plan.CollisionCenter, Plan.SurgeRadiusRad });
			UsedPlateIds.Add(Plan.Pair->PlateA);
			UsedPlateIds.Add(Plan.Pair->PlateB);

			UE_LOG(
				LogTemp,
				Log,
				TEXT("[V9ThesisCollisionEvent Step=%d idx=%d] pair=(%d,%d) over_plate=%d sub_plate=%d obs=%d accumulated_penetration_km=%.1f mean_convergence_km_per_my=%.3f max_convergence_km_per_my=%.3f support=%d collision_support=%d continental_qualified=%d seed_samples=%d collision_seeds=%d terrane_component=%d terrane_area_sr=%.6f xi=%.6f rel_speed_km_per_my=%.3f effective_mass=%d surge_radius_rad=%.6f transfer_radius_rad=%.6f suture_axis=(%.4f,%.4f,%.4f) ridge_half_width_rad=%.6f belt_length_rad=%.6f mean_abs_along_rad=%.6f mean_abs_perp_rad=%.6f ridge_core=%d ridge_flank=%d transfer_candidates=%d transfer_anchor_seeds=%d transferred=%d transferred_continental=%d boundary_local=%d reject_locality=%d reject_continentality=%d reject_cap=%d reject_connectivity=%d affected=%d continental_gain=%d mean_elev_delta_km=%.6f max_elev_delta_km=%.6f donor_share=%.6f->%.6f recipient_share=%.6f->%.6f"),
				Planet.CurrentStep,
				ExecutedCollisionCount,
				Plan.Pair->PlateA,
				Plan.Pair->PlateB,
				Plan.OverridingPlateId,
				Plan.SubductingPlateId,
				Plan.Pair->ObservationCount,
				Plan.Pair->AccumulatedPenetrationKm,
				Plan.Pair->MeanConvergenceKmPerMy,
				Plan.Pair->MaxConvergenceKmPerMy,
				Plan.Pair->SupportSampleCount,
				Plan.Pair->CollisionSampleCount,
				Plan.Pair->ContinentalQualifiedSampleCount,
				Plan.SeedSampleIndices.Num(),
				Plan.CollisionSeedSampleIndices.Num(),
				Plan.DonorTerraneComponentSize,
				Plan.TerraneAreaSr,
				Plan.Xi,
				Plan.RelativeSpeedKmPerMy,
				Plan.EffectiveMassSampleCount,
				Plan.SurgeRadiusRad,
				Plan.TransferInfluenceRadiusRad,
				EventSutureAxis.X,
				EventSutureAxis.Y,
				EventSutureAxis.Z,
				EventSutureHalfWidthRad,
				EventBeltLengthEstimateRad,
				EventMeanAbsAlongSutureRad,
				EventMeanAbsPerpendicularRad,
				EventRidgeCoreAffectedCount,
				EventRidgeFlankAffectedCount,
				Plan.TransferCandidateSupportCount,
				Plan.TransferAnchorSeedCount,
				TransferredSampleIndices.Num(),
				EventTransferredContinentalCount,
				EventTransferBoundaryLocalCount,
				Plan.TransferRejectedByLocalityCount,
				Plan.TransferRejectedByContinentalityCount,
				Plan.TransferRejectedByCapCount,
				Plan.TransferRejectedByConnectivityCount,
				EventAffectedSampleIndices.Num(),
				EventContinentalGainCount,
				EventAffectedSampleIndices.IsEmpty()
					? 0.0
					: EventTotalElevationDeltaKm /
						static_cast<double>(EventAffectedSampleIndices.Num()),
				EventMaxElevationDeltaKm,
				DonorPlateShareBefore,
				DonorPlateShareAfter,
				RecipientPlateShareBefore,
				RecipientPlateShareAfter);

			if (ExecutedCollisionCount >= MaxCollisionExecutionsPerSolve)
			{
				break;
			}
		}

		OutDiagnostic.ExecutedCollisionCount = ExecutedCollisionCount;
		OutDiagnostic.QualifiedButUnexecutedCount =
			FMath::Max(0, QualifiedCount - ExecutedCollisionCount);
		OutDiagnostic.CumulativeExecutedCollisionCount = InOutCumulativeExecutionCount;
		OutDiagnostic.CumulativeCollisionAffectedSampleVisits = InOutCumulativeAffectedSampleVisits;
		OutDiagnostic.CumulativeCollisionDrivenContinentalGainCount =
			InOutCumulativeContinentalGainCount;
		OutDiagnostic.CumulativeCollisionDrivenOwnershipChangeCount =
			InOutCumulativeOwnershipChangeCount;
		OutDiagnostic.CumulativeCollisionTransferredSampleVisits =
			InOutCumulativeTransferredSampleVisits;
		OutDiagnostic.CumulativeCollisionTransferredContinentalSampleCount =
			InOutCumulativeTransferredContinentalSampleCount;
		OutDiagnostic.CollisionDrivenContinentalGainCount = CurrentSolveContinentalGainCount;
		OutDiagnostic.CollisionDrivenOwnershipChangeCount = CurrentSolveOwnershipChangeCount;
		OutDiagnostic.CollisionTransferredSampleCount = CurrentSolveTransferredSampleCount;
		OutDiagnostic.CollisionTransferredContinentalSampleCount =
			CurrentSolveTransferredContinentalSampleCount;
		OutDiagnostic.bExecutedFromShadowQualifiedState = ExecutedCollisionCount > 0;

		int32 CurrentSolveAffectedSampleCount = 0;
		for (const uint8 Value : OutExecutionMask)
		{
			CurrentSolveAffectedSampleCount += Value != 0 ? 1 : 0;
		}
		OutDiagnostic.CollisionAffectedSampleCount = CurrentSolveAffectedSampleCount;
		OutDiagnostic.CumulativeCollisionAffectedSampleCount = 0;
		for (const uint8 Value : InOutCumulativeExecutionMask)
		{
			OutDiagnostic.CumulativeCollisionAffectedSampleCount += Value != 0 ? 1 : 0;
		}
		OutDiagnostic.CumulativeCollisionTransferredSampleCount = 0;
		for (const uint8 Value : InOutCumulativeTransferMask)
		{
			OutDiagnostic.CumulativeCollisionTransferredSampleCount += Value != 0 ? 1 : 0;
		}
		OutDiagnostic.CumulativeMeanElevationDeltaKm =
			InOutCumulativeAffectedSampleVisits > 0
				? InOutCumulativeElevationDeltaKm /
					static_cast<double>(InOutCumulativeAffectedSampleVisits)
				: 0.0;
		OutDiagnostic.CumulativeMaxElevationDeltaKm = InOutCumulativeMaxElevationDeltaKm;

		if (ExecutedCollisionCount == 0)
		{
			return;
		}

		FResamplingStats& LegacyStats = Planet.LastResamplingStats;
		LegacyStats.CollisionCount = ExecutedCollisionCount;
		LegacyStats.CollisionTerraneId = INDEX_NONE;
		LegacyStats.CollisionTerraneSampleCount = CurrentSolveTransferredSampleCount;
		LegacyStats.CollisionOverridingPlateId = OutDiagnostic.ExecutedOverridingPlateId;
		LegacyStats.CollisionSubductingPlateId = OutDiagnostic.ExecutedSubductingPlateId;
		LegacyStats.CollisionSurgeAffectedCount = CurrentSolveAffectedSampleCount;
		LegacyStats.CollisionContinentalGainCount = CurrentSolveContinentalGainCount;
		LegacyStats.CollisionSurgeRadiusRad = OutDiagnostic.ExecutedInfluenceRadiusRad;
		LegacyStats.CollisionSurgeMeanElevationDelta =
			CurrentSolveAffectedSampleCount > 0
				? CurrentSolveTotalElevationDeltaKm /
					static_cast<double>(CurrentSolveAffectedSampleCount)
				: 0.0;

		UE_LOG(
			LogTemp,
			Log,
			TEXT("[V9ThesisCollision Step=%d] executed=%d cooldown_suppressed=%d plate_conflict_suppressed=%d overlap_suppressed=%d qualified_unexecuted=%d cumulative_exec=%d current_affected=%d cumulative_affected=%d current_transfer=%d cumulative_transfer=%d current_ownership_change=%d cumulative_ownership_change=%d current_continental_gain=%d cumulative_continental_gain=%d current_mean_elev_delta_km=%.6f current_max_elev_delta_km=%.6f cumulative_mean_elev_delta_km=%.6f cumulative_max_elev_delta_km=%.6f representative_pair=(%d,%d) representative_terrane=%d representative_transfer=%d representative_xi=%.6f representative_rel_speed_km_per_my=%.3f representative_radius_rad=%.6f"),
			Planet.CurrentStep,
			ExecutedCollisionCount,
			OutDiagnostic.CooldownSuppressedQualifiedCount,
			OutDiagnostic.PlateConflictSuppressedQualifiedCount,
			OutDiagnostic.OverlapSuppressedQualifiedCount,
			OutDiagnostic.QualifiedButUnexecutedCount,
			InOutCumulativeExecutionCount,
			CurrentSolveAffectedSampleCount,
			OutDiagnostic.CumulativeCollisionAffectedSampleCount,
			CurrentSolveTransferredSampleCount,
			OutDiagnostic.CumulativeCollisionTransferredSampleCount,
			CurrentSolveOwnershipChangeCount,
			InOutCumulativeOwnershipChangeCount,
			CurrentSolveContinentalGainCount,
			InOutCumulativeContinentalGainCount,
			CurrentSolveAffectedSampleCount > 0
				? CurrentSolveTotalElevationDeltaKm /
					static_cast<double>(CurrentSolveAffectedSampleCount)
				: 0.0,
			CurrentSolveMaxElevationDeltaKm,
			OutDiagnostic.CumulativeMeanElevationDeltaKm,
			OutDiagnostic.CumulativeMaxElevationDeltaKm,
			OutDiagnostic.ExecutedPlateA,
			OutDiagnostic.ExecutedPlateB,
			OutDiagnostic.ExecutedDonorTerraneComponentSize,
			OutDiagnostic.ExecutedTransferredComponentSize,
			OutDiagnostic.ExecutedXi,
			OutDiagnostic.ExecutedRelativeSpeedKmPerMy,
			OutDiagnostic.ExecutedInfluenceRadiusRad);
	}

	const FPlate* FindPlateById(const FTectonicPlanet& Planet, const int32 PlateId)
	{
		for (const FPlate& Plate : Planet.Plates)
		{
			if (Plate.Id == PlateId)
			{
				return &Plate;
			}
		}

		return nullptr;
	}

	bool IsOverridingPlateLocal(const FTectonicPlanet& Planet, const int32 CandidatePlateId, const int32 OtherPlateId)
	{
		const FPlate* CandidatePlate = FindPlateById(Planet, CandidatePlateId);
		const FPlate* OtherPlate = FindPlateById(Planet, OtherPlateId);
		if (CandidatePlate == nullptr || OtherPlate == nullptr)
		{
			return false;
		}

		return
			CandidatePlate->OverlapScore > OtherPlate->OverlapScore ||
			(CandidatePlate->OverlapScore == OtherPlate->OverlapScore &&
				CandidatePlateId < OtherPlateId);
	}

	FVector3d ComputePlateSurfaceVelocity(const FPlate& Plate, const FVector3d& QueryPoint, const double PlanetRadius)
	{
		if (Plate.RotationAxis.IsNearlyZero() || Plate.AngularSpeed <= 0.0)
		{
			return FVector3d::ZeroVector;
		}

		return FVector3d::CrossProduct(Plate.RotationAxis.GetSafeNormal(), QueryPoint) * (Plate.AngularSpeed * PlanetRadius);
	}

	void ResetV6QueryGeometry(FV6PlateQueryGeometry& Geometry)
	{
		Geometry.PlateId = INDEX_NONE;
		Geometry.BoundingCap = FSphericalBoundingCap{};
		Geometry.SoupData = FPlateTriangleSoupData{};
		Geometry.LocalTriangleCopiedFrontierFlags.Reset();
		Geometry.LocalTriangleDestructiveFlags.Reset();
		Geometry.SoupAdapter = FPlateTriangleSoupAdapter(&Geometry.SoupData);
		Geometry.SoupBVH = FPlateTriangleSoupBVH{};
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

	template <int32 InlineSize>
	void AppendQueryGeometryIndexForPlateId(
		const TMap<int32, int32>& QueryGeometryIndexByPlateId,
		const int32 PlateId,
		TArray<int32, TInlineAllocator<InlineSize>>& OutQueryGeometryIndices)
	{
		if (PlateId == INDEX_NONE)
		{
			return;
		}

		if (const int32* QueryGeometryIndexPtr = QueryGeometryIndexByPlateId.Find(PlateId))
		{
			OutQueryGeometryIndices.AddUnique(*QueryGeometryIndexPtr);
		}
	}

	double ComputeGeodesicDistance(const FVector3d& A, const FVector3d& B)
	{
		return FMath::Acos(FMath::Clamp(A.Dot(B), -1.0, 1.0));
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

	FVector3d ProjectOntoTangent(const FVector3d& Vector, const FVector3d& Normal)
	{
		return Vector - (Vector.Dot(Normal) * Normal);
	}

	void GatherBoundaryNeighborSummaries(
		const FTectonicPlanet& Planet,
		const int32 SampleIndex,
		TArray<FV6BoundaryNeighborSummary, TInlineAllocator<8>>& OutSummaries)
	{
		OutSummaries.Reset();
		if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			return;
		}

		TMap<int32, int32> PlateToSummaryIndex;
		const FVector3d QueryPoint = Planet.Samples.IsValidIndex(SampleIndex)
			? Planet.Samples[SampleIndex].Position.GetSafeNormal()
			: FVector3d::ZeroVector;

		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!Planet.Samples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 PlateId = Planet.Samples[NeighborIndex].PlateId;
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

			FV6BoundaryNeighborSummary& Summary = OutSummaries[SummaryIndex];
			++Summary.VoteCount;
			const FVector3d NeighborDirection =
				ProjectOntoTangent(Planet.Samples[NeighborIndex].Position - QueryPoint, QueryPoint);
			if (!NeighborDirection.IsNearlyZero())
			{
				Summary.TangentSum += NeighborDirection.GetSafeNormal();
			}
		}

		Algo::Sort(OutSummaries, [](const FV6BoundaryNeighborSummary& Left, const FV6BoundaryNeighborSummary& Right)
		{
			if (Left.VoteCount != Right.VoteCount)
			{
				return Left.VoteCount > Right.VoteCount;
			}
			return Left.PlateId < Right.PlateId;
		});
	}

	void GatherBoundaryNeighborSummariesForPlateIds(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		const int32 SampleIndex,
		TArray<FV6BoundaryNeighborSummary, TInlineAllocator<8>>& OutSummaries)
	{
		OutSummaries.Reset();
		if (!PlateIds.IsValidIndex(SampleIndex) || !Planet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			return;
		}

		TMap<int32, int32> PlateToSummaryIndex;
		const FVector3d QueryPoint = Planet.Samples.IsValidIndex(SampleIndex)
			? Planet.Samples[SampleIndex].Position.GetSafeNormal()
			: FVector3d::ZeroVector;

		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!Planet.Samples.IsValidIndex(NeighborIndex) || !PlateIds.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 PlateId = PlateIds[NeighborIndex];
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

			FV6BoundaryNeighborSummary& Summary = OutSummaries[SummaryIndex];
			++Summary.VoteCount;
			const FVector3d NeighborDirection =
				ProjectOntoTangent(Planet.Samples[NeighborIndex].Position - QueryPoint, QueryPoint);
			if (!NeighborDirection.IsNearlyZero())
			{
				Summary.TangentSum += NeighborDirection.GetSafeNormal();
			}
		}

		Algo::Sort(OutSummaries, [](const FV6BoundaryNeighborSummary& Left, const FV6BoundaryNeighborSummary& Right)
		{
			if (Left.VoteCount != Right.VoteCount)
			{
				return Left.VoteCount > Right.VoteCount;
			}
			return Left.PlateId < Right.PlateId;
		});
	}

	const FV6BoundaryNeighborSummary* FindBoundaryNeighborSummary(
		const TArray<FV6BoundaryNeighborSummary, TInlineAllocator<8>>& Summaries,
		const int32 PlateId)
	{
		for (const FV6BoundaryNeighborSummary& Summary : Summaries)
		{
			if (Summary.PlateId == PlateId)
			{
				return &Summary;
			}
		}

		return nullptr;
	}

	void BuildBoundaryMotionSamplesForPlateIds(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		const int32 SampleIndex,
		const TArray<int32>& RelevantPlateIds,
		TArray<FTectonicPlanetV6BoundaryMotionSample>& OutMotionSamples)
	{
		OutMotionSamples.Reset();
		if (!Planet.Samples.IsValidIndex(SampleIndex))
		{
			return;
		}

		TArray<FV6BoundaryNeighborSummary, TInlineAllocator<8>> NeighborSummaries;
		GatherBoundaryNeighborSummariesForPlateIds(Planet, PlateIds, SampleIndex, NeighborSummaries);

		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		for (const int32 PlateId : RelevantPlateIds)
		{
			if (PlateId == INDEX_NONE)
			{
				continue;
			}

			const FPlate* Plate = FindPlateById(Planet, PlateId);
			if (Plate == nullptr)
			{
				continue;
			}

			FTectonicPlanetV6BoundaryMotionSample& MotionSample = OutMotionSamples.AddDefaulted_GetRef();
			MotionSample.PlateId = PlateId;
			MotionSample.SurfaceVelocity = ComputePlateSurfaceVelocity(*Plate, QueryPoint, Planet.PlanetRadiusKm);
			if (const FV6BoundaryNeighborSummary* Summary = FindBoundaryNeighborSummary(NeighborSummaries, PlateId))
			{
				MotionSample.NeighborVoteCount = Summary->VoteCount;
				MotionSample.NeighborTangent =
					Summary->TangentSum.IsNearlyZero() ? FVector3d::ZeroVector : Summary->TangentSum.GetSafeNormal();
			}
		}
	}

	ETectonicPlanetV6CopiedFrontierMotionClass ToCopiedFrontierMotionClass(const EV6BoundaryMotionClass MotionClass)
	{
		switch (MotionClass)
		{
		case EV6BoundaryMotionClass::Weak:
			return ETectonicPlanetV6CopiedFrontierMotionClass::Weak;
		case EV6BoundaryMotionClass::Divergent:
			return ETectonicPlanetV6CopiedFrontierMotionClass::Divergent;
		case EV6BoundaryMotionClass::Convergent:
			return ETectonicPlanetV6CopiedFrontierMotionClass::Convergent;
		case EV6BoundaryMotionClass::None:
		default:
			return ETectonicPlanetV6CopiedFrontierMotionClass::None;
		}
	}

	void BuildSampleToAdjacentTriangles(const FTectonicPlanet& Planet, TArray<TArray<int32>>& OutSampleToAdjacentTriangles)
	{
		OutSampleToAdjacentTriangles.Reset();
		OutSampleToAdjacentTriangles.SetNum(Planet.Samples.Num());
		for (int32 GlobalTriangleIndex = 0; GlobalTriangleIndex < Planet.TriangleIndices.Num(); ++GlobalTriangleIndex)
		{
			if (!Planet.TriangleIndices.IsValidIndex(GlobalTriangleIndex))
			{
				continue;
			}

			const FIntVector& Triangle = Planet.TriangleIndices[GlobalTriangleIndex];
			if (OutSampleToAdjacentTriangles.IsValidIndex(Triangle.X))
			{
				OutSampleToAdjacentTriangles[Triangle.X].Add(GlobalTriangleIndex);
			}
			if (OutSampleToAdjacentTriangles.IsValidIndex(Triangle.Y))
			{
				OutSampleToAdjacentTriangles[Triangle.Y].Add(GlobalTriangleIndex);
			}
			if (OutSampleToAdjacentTriangles.IsValidIndex(Triangle.Z))
			{
				OutSampleToAdjacentTriangles[Triangle.Z].Add(GlobalTriangleIndex);
			}
		}
	}

	enum class EV6CopiedFrontierMultiHitPlateMix : uint8
	{
		None,
		SameOnly,
		CrossOnly,
		Mixed,
	};

	enum class EV6CopiedFrontierMultiHitGeometryMix : uint8
	{
		None,
		CopiedFrontierOnly,
		InteriorOnly,
		Mixed,
	};

	bool IsTrackedDestructiveTriangle(
		const TArray<uint8>& TrackedDestructiveKinds,
		const int32 GlobalTriangleIndex)
	{
		return TrackedDestructiveKinds.IsValidIndex(GlobalTriangleIndex) &&
			TrackedDestructiveKinds[GlobalTriangleIndex] != 0;
	}

	void ComputeTrackedDestructiveTriangleProximity(
		const FTectonicPlanet& Planet,
		const TArray<TArray<int32>>& SampleToAdjacentTriangles,
		const TArray<uint8>& TrackedDestructiveKinds,
		const int32 SampleIndex,
		bool& bOutTouchesTrackedTriangle,
		bool& bOutWithinTrackedOneRing,
		bool& bOutWithinTrackedTwoRing)
	{
		bOutTouchesTrackedTriangle = false;
		bOutWithinTrackedOneRing = false;
		bOutWithinTrackedTwoRing = false;

		if (!SampleToAdjacentTriangles.IsValidIndex(SampleIndex))
		{
			return;
		}

		TArray<int32> Frontier = SampleToAdjacentTriangles[SampleIndex];
		if (Frontier.IsEmpty())
		{
			return;
		}

		TSet<int32> VisitedTriangles;
		for (const int32 TriangleIndex : Frontier)
		{
			VisitedTriangles.Add(TriangleIndex);
			if (IsTrackedDestructiveTriangle(TrackedDestructiveKinds, TriangleIndex))
			{
				bOutTouchesTrackedTriangle = true;
				bOutWithinTrackedOneRing = true;
				bOutWithinTrackedTwoRing = true;
			}
		}

		if (bOutWithinTrackedTwoRing)
		{
			return;
		}

		TArray<int32> NextFrontier;
		for (int32 RingDepth = 1; RingDepth <= 2; ++RingDepth)
		{
			NextFrontier.Reset();
			for (const int32 TriangleIndex : Frontier)
			{
				if (!Planet.TriangleAdjacency.IsValidIndex(TriangleIndex))
				{
					continue;
				}

				for (const int32 NeighborTriangleIndex : Planet.TriangleAdjacency[TriangleIndex])
				{
					if (VisitedTriangles.Contains(NeighborTriangleIndex))
					{
						continue;
					}

					VisitedTriangles.Add(NeighborTriangleIndex);
					if (IsTrackedDestructiveTriangle(TrackedDestructiveKinds, NeighborTriangleIndex))
					{
						if (RingDepth <= 1)
						{
							bOutWithinTrackedOneRing = true;
						}
						bOutWithinTrackedTwoRing = true;
					}

					NextFrontier.Add(NeighborTriangleIndex);
				}
			}

			if (bOutWithinTrackedTwoRing)
			{
				if (RingDepth <= 1)
				{
					bOutWithinTrackedOneRing = true;
				}
				return;
			}

			Frontier = NextFrontier;
			if (Frontier.IsEmpty())
			{
				return;
			}
		}
	}

	const TCHAR* GetCopiedFrontierMultiHitPlateMixName(const EV6CopiedFrontierMultiHitPlateMix PlateMix)
	{
		switch (PlateMix)
		{
		case EV6CopiedFrontierMultiHitPlateMix::SameOnly:
			return TEXT("same_only");
		case EV6CopiedFrontierMultiHitPlateMix::CrossOnly:
			return TEXT("cross_only");
		case EV6CopiedFrontierMultiHitPlateMix::Mixed:
			return TEXT("mixed");
		case EV6CopiedFrontierMultiHitPlateMix::None:
		default:
			return TEXT("none");
		}
	}

	const TCHAR* GetCopiedFrontierMultiHitGeometryMixName(const EV6CopiedFrontierMultiHitGeometryMix GeometryMix)
	{
		switch (GeometryMix)
		{
		case EV6CopiedFrontierMultiHitGeometryMix::CopiedFrontierOnly:
			return TEXT("frontier_only");
		case EV6CopiedFrontierMultiHitGeometryMix::InteriorOnly:
			return TEXT("interior_only");
		case EV6CopiedFrontierMultiHitGeometryMix::Mixed:
			return TEXT("mixed");
		case EV6CopiedFrontierMultiHitGeometryMix::None:
		default:
			return TEXT("none");
		}
	}

	FString BuildCopiedFrontierMultiHitPatternKey(
		const bool bHasTrackedCandidate,
		const bool bWithinTrackedOneRing,
		const bool bWithinTrackedTwoRing,
		const EV6CopiedFrontierMultiHitPlateMix PlateMix,
		const EV6CopiedFrontierMultiHitGeometryMix GeometryMix,
		const bool bSamePlateWinner,
		const bool bAdjacentToDestructiveGap,
		const bool bAdjacentToContinuityHandledDestructiveGap)
	{
		const TCHAR* TrackedBand = TEXT("deep");
		if (bWithinTrackedOneRing)
		{
			TrackedBand = TEXT("ring1");
		}
		else if (bWithinTrackedTwoRing)
		{
			TrackedBand = TEXT("ring2");
		}

		return FString::Printf(
			TEXT("tracked_candidate=%d tracked_band=%s plate_mix=%s geometry_mix=%s winner=%s destructive_gap_adj=%d continuity_gap_adj=%d"),
			bHasTrackedCandidate ? 1 : 0,
			TrackedBand,
			GetCopiedFrontierMultiHitPlateMixName(PlateMix),
			GetCopiedFrontierMultiHitGeometryMixName(GeometryMix),
			bSamePlateWinner ? TEXT("same") : TEXT("cross"),
			bAdjacentToDestructiveGap ? 1 : 0,
			bAdjacentToContinuityHandledDestructiveGap ? 1 : 0);
	}

	bool IsSampleAdjacentToCopiedFrontierGeometry(
		const TArray<TArray<int32>>& SampleToAdjacentTriangles,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		const int32 SampleIndex)
	{
		if (!SampleToAdjacentTriangles.IsValidIndex(SampleIndex))
		{
			return false;
		}

		for (const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh : PlateMeshes)
		{
			if (const int32* LocalVertexIndex = Mesh.CanonicalToLocalVertex.Find(SampleIndex))
			{
				if (Mesh.LocalVertexCopiedFrontierFlags.IsValidIndex(*LocalVertexIndex) &&
					Mesh.LocalVertexCopiedFrontierFlags[*LocalVertexIndex] != 0)
				{
					return true;
				}
			}
		}

		for (const int32 GlobalTriangleIndex : SampleToAdjacentTriangles[SampleIndex])
		{
			for (const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh : PlateMeshes)
			{
				const int32* LocalTriangleIndexPtr = Mesh.GlobalToLocalTriangle.Find(GlobalTriangleIndex);
				if (LocalTriangleIndexPtr != nullptr &&
					Mesh.LocalTriangleCopiedFrontierFlags.IsValidIndex(*LocalTriangleIndexPtr) &&
					Mesh.LocalTriangleCopiedFrontierFlags[*LocalTriangleIndexPtr] != 0)
				{
					return true;
				}
			}
		}

		return false;
	}

	double ComputeNearestCopiedFrontierTriangleDistanceKm(
		const FTectonicPlanet& Planet,
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const FVector3d& QueryPoint)
	{
		double BestDistanceSqr = TNumericLimits<double>::Max();
		for (const FV6PlateQueryGeometry& QueryGeometry : QueryGeometries)
		{
			if (QueryGeometry.SoupData.LocalTriangles.IsEmpty())
			{
				continue;
			}

			for (int32 LocalTriangleIndex = 0; LocalTriangleIndex < QueryGeometry.SoupData.LocalTriangles.Num(); ++LocalTriangleIndex)
			{
				if (!QueryGeometry.LocalTriangleCopiedFrontierFlags.IsValidIndex(LocalTriangleIndex) ||
					QueryGeometry.LocalTriangleCopiedFrontierFlags[LocalTriangleIndex] == 0)
				{
					continue;
				}

				FVector3d A;
				FVector3d B;
				FVector3d C;
				QueryGeometry.SoupAdapter.GetTriVertices(LocalTriangleIndex, A, B, C);
				const UE::Geometry::TTriangle3<double> Triangle(A, B, C);
				UE::Geometry::FDistPoint3Triangle3d DistanceQuery(QueryPoint, Triangle);
				DistanceQuery.ComputeResult();
				BestDistanceSqr = FMath::Min(BestDistanceSqr, DistanceQuery.GetSquared());
			}
		}

		if (!FMath::IsFinite(BestDistanceSqr) || BestDistanceSqr == TNumericLimits<double>::Max())
		{
			return -1.0;
		}

		return FMath::Sqrt(FMath::Max(BestDistanceSqr, 0.0)) * Planet.PlanetRadiusKm;
	}

	double ComputeQueryTriangleDistanceKm(
		const FTectonicPlanet& Planet,
		const FV6PlateQueryGeometry& QueryGeometry,
		const FVector3d& QueryPoint,
		const int32 LocalTriangleIndex)
	{
		if (!QueryGeometry.SoupData.LocalTriangles.IsValidIndex(LocalTriangleIndex))
		{
			return -1.0;
		}

		FVector3d A;
		FVector3d B;
		FVector3d C;
		QueryGeometry.SoupAdapter.GetTriVertices(LocalTriangleIndex, A, B, C);
		const UE::Geometry::TTriangle3<double> Triangle(A, B, C);
		UE::Geometry::FDistPoint3Triangle3d DistanceQuery(QueryPoint, Triangle);
		DistanceQuery.ComputeResult();
		return FMath::Sqrt(FMath::Max(DistanceQuery.GetSquared(), 0.0)) * Planet.PlanetRadiusKm;
	}

	void FindPrimaryAndSecondaryRecoveryPlatesKm(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		int32& OutPrimaryPlateId,
		double& OutPrimaryDistanceKm,
		int32& OutSecondaryPlateId,
		double& OutSecondaryDistanceKm)
	{
		OutPrimaryPlateId = INDEX_NONE;
		OutPrimaryDistanceKm = -1.0;
		OutSecondaryPlateId = INDEX_NONE;
		OutSecondaryDistanceKm = -1.0;

		TMap<int32, double> BestDistanceByPlateKm;
		for (const FTectonicPlanetV6RecoveryCandidate& Candidate : RecoveryCandidates)
		{
			if (Candidate.PlateId == INDEX_NONE)
			{
				continue;
			}

			const double DistanceKm = Candidate.DistanceRadians * Planet.PlanetRadiusKm;
			double& BestDistanceKm = BestDistanceByPlateKm.FindOrAdd(Candidate.PlateId, DistanceKm);
			BestDistanceKm = FMath::Min(BestDistanceKm, DistanceKm);
		}

		for (const TPair<int32, double>& Entry : BestDistanceByPlateKm)
		{
			const bool bBeatsPrimary =
				OutPrimaryPlateId == INDEX_NONE ||
				Entry.Value + TriangleEpsilon < OutPrimaryDistanceKm ||
				(FMath::IsNearlyEqual(Entry.Value, OutPrimaryDistanceKm, TriangleEpsilon) && Entry.Key < OutPrimaryPlateId);
			if (bBeatsPrimary)
			{
				OutSecondaryPlateId = OutPrimaryPlateId;
				OutSecondaryDistanceKm = OutPrimaryDistanceKm;
				OutPrimaryPlateId = Entry.Key;
				OutPrimaryDistanceKm = Entry.Value;
				continue;
			}

			if (Entry.Key == OutPrimaryPlateId)
			{
				continue;
			}

			const bool bBeatsSecondary =
				OutSecondaryPlateId == INDEX_NONE ||
				Entry.Value + TriangleEpsilon < OutSecondaryDistanceKm ||
				(FMath::IsNearlyEqual(Entry.Value, OutSecondaryDistanceKm, TriangleEpsilon) && Entry.Key < OutSecondaryPlateId);
			if (bBeatsSecondary)
			{
				OutSecondaryPlateId = Entry.Key;
				OutSecondaryDistanceKm = Entry.Value;
			}
		}
	}

		void IncrementFragmentSizeBucket(
			FTectonicPlanetV6CopiedFrontierFragmentSizeBuckets& Buckets,
			const int32 Size)
		{
		if (Size <= 1)
		{
			++Buckets.Size1;
		}
		else if (Size <= 4)
		{
			++Buckets.Size2To4;
		}
		else if (Size <= 16)
		{
			++Buckets.Size5To16;
		}
		else
		{
			++Buckets.Size17Plus;
			}
		}

		void IncrementPlateCount(TMap<int32, int32>& InOutCounts, const int32 PlateId)
		{
			if (PlateId == INDEX_NONE)
			{
				return;
			}

			++InOutCounts.FindOrAdd(PlateId);
		}

		void AppendSortedPlateCounts(
			const TMap<int32, int32>& CountsByPlate,
			TArray<FTectonicPlanetV6PlateCount>& OutCounts)
		{
			OutCounts.Reset();
			OutCounts.Reserve(CountsByPlate.Num());
			for (const TPair<int32, int32>& Entry : CountsByPlate)
			{
				FTectonicPlanetV6PlateCount& Count = OutCounts.AddDefaulted_GetRef();
				Count.PlateId = Entry.Key;
				Count.Count = Entry.Value;
			}

			OutCounts.Sort([](const FTectonicPlanetV6PlateCount& Left, const FTectonicPlanetV6PlateCount& Right)
			{
				return Left.Count != Right.Count ? Left.Count > Right.Count : Left.PlateId < Right.PlateId;
			});
		}

		void AppendTopPatternCounts(
			const TMap<FString, int32>& PatternCounts,
			TArray<FTectonicPlanetV6CopiedFrontierHitLossPatternCount>& OutCounts,
			const int32 MaxCount)
		{
			OutCounts.Reset();
			OutCounts.Reserve(PatternCounts.Num());
			for (const TPair<FString, int32>& Entry : PatternCounts)
			{
				FTectonicPlanetV6CopiedFrontierHitLossPatternCount& Count = OutCounts.AddDefaulted_GetRef();
				Count.Pattern = Entry.Key;
				Count.Count = Entry.Value;
			}

			OutCounts.Sort([](
				const FTectonicPlanetV6CopiedFrontierHitLossPatternCount& Left,
				const FTectonicPlanetV6CopiedFrontierHitLossPatternCount& Right)
			{
				return Left.Count != Right.Count ? Left.Count > Right.Count : Left.Pattern < Right.Pattern;
			});

			if (OutCounts.Num() > MaxCount)
			{
				OutCounts.SetNum(MaxCount, EAllowShrinking::No);
			}
		}

		FString BuildSingleHitLossPatternKey(
			const bool bSamePlateWinner,
			const bool bCopiedFrontierWinningTriangle,
			const bool bNearFrontier,
			const bool bDominantSourceWasContinental,
			const bool bPreviousPlateHadNoCapAlternative)
		{
			return FString::Printf(
				TEXT("%s|%s|%s|%s|%s"),
				bSamePlateWinner ? TEXT("same_plate") : TEXT("cross_plate"),
				bCopiedFrontierWinningTriangle ? TEXT("frontier_hit") : TEXT("interior_hit"),
				bNearFrontier ? TEXT("near_frontier") : TEXT("interior_region"),
				bDominantSourceWasContinental ? TEXT("dominant_continental") : TEXT("dominant_subcontinental"),
				bPreviousPlateHadNoCapAlternative ? TEXT("prev_plate_alt_nocap") : TEXT("no_prev_plate_alt_nocap"));
		}

		void BuildAssignmentComponents(
			const FTectonicPlanet& Planet,
			const TArray<int32>& PlateIds,
		TArray<FV6AssignmentComponent>& OutComponents)
	{
		OutComponents.Reset();
		if (PlateIds.Num() != Planet.Samples.Num())
		{
			return;
		}

		TArray<uint8> Visited;
		Visited.Init(0, PlateIds.Num());
		for (int32 SampleIndex = 0; SampleIndex < PlateIds.Num(); ++SampleIndex)
		{
			const int32 PlateId = PlateIds[SampleIndex];
			if (PlateId == INDEX_NONE || Visited[SampleIndex] != 0)
			{
				continue;
			}

			FV6AssignmentComponent& Component = OutComponents.AddDefaulted_GetRef();
			Component.PlateId = PlateId;
			Component.MinSampleIndex = SampleIndex;
			TArray<int32, TInlineAllocator<256>> Stack;
			Stack.Add(SampleIndex);
			Visited[SampleIndex] = 1;

			while (!Stack.IsEmpty())
			{
				const int32 CurrentSampleIndex = Stack.Pop(EAllowShrinking::No);
				Component.Samples.Add(CurrentSampleIndex);
				Component.MinSampleIndex = FMath::Min(Component.MinSampleIndex, CurrentSampleIndex);
				if (!Planet.SampleAdjacency.IsValidIndex(CurrentSampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[CurrentSampleIndex])
				{
					if (!PlateIds.IsValidIndex(NeighborIndex) ||
						Visited[NeighborIndex] != 0 ||
						PlateIds[NeighborIndex] != PlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}
		}
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

	EOrogenyType ChooseTransferredOrogenyType(
		const EOrogenyType A,
		const EOrogenyType B,
		const EOrogenyType C,
		const FVector3d& Barycentric,
		ETectonicPlanetV6CategoricalTransferKind& OutTransferKind)
	{
		if (A == B || A == C)
		{
			OutTransferKind = ETectonicPlanetV6CategoricalTransferKind::MajorityVote;
			return A;
		}
		if (B == C)
		{
			OutTransferKind = ETectonicPlanetV6CategoricalTransferKind::MajorityVote;
			return B;
		}

		OutTransferKind = ETectonicPlanetV6CategoricalTransferKind::DominantSource;
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

	int32 ChooseTransferredTerraneId(
		const int32 A,
		const int32 B,
		const int32 C,
		const FVector3d& Barycentric,
		ETectonicPlanetV6CategoricalTransferKind& OutTransferKind)
	{
		if (A == B || A == C)
		{
			OutTransferKind = ETectonicPlanetV6CategoricalTransferKind::MajorityVote;
			return A;
		}
		if (B == C)
		{
			OutTransferKind = ETectonicPlanetV6CategoricalTransferKind::MajorityVote;
			return B;
		}

		OutTransferKind = ETectonicPlanetV6CategoricalTransferKind::DominantSource;
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

	FVector3d TransferDirectionalField(
		const FVector3d& A,
		const FVector3d& B,
		const FVector3d& C,
		const FVector3d& Barycentric,
		const FVector3d& SurfaceNormal,
		ETectonicPlanetV6DirectionalTransferKind& OutTransferKind)
	{
		const FVector3d Blended =
			(Barycentric.X * A) +
			(Barycentric.Y * B) +
			(Barycentric.Z * C);
		const FVector3d Projected = ProjectOntoTangent(Blended, SurfaceNormal);
		if (Projected.SquaredLength() >= DirectionDegeneracyThreshold * DirectionDegeneracyThreshold)
		{
			OutTransferKind = ETectonicPlanetV6DirectionalTransferKind::WeightedBlend;
			return Projected.GetSafeNormal();
		}

		const int32 DominantIndex = PickDominantBarycentricIndex(Barycentric);
		const FVector3d Dominant =
			DominantIndex == 1 ? B : (DominantIndex == 2 ? C : A);
		const FVector3d Fallback = ProjectOntoTangent(Dominant, SurfaceNormal);
		if (Fallback.SquaredLength() > DirectionDegeneracyThreshold * DirectionDegeneracyThreshold)
		{
			OutTransferKind = ETectonicPlanetV6DirectionalTransferKind::DominantSourceFallback;
			return Fallback.GetSafeNormal();
		}

		OutTransferKind = ETectonicPlanetV6DirectionalTransferKind::ZeroVectorFallback;
		return FVector3d::ZeroVector;
	}

	FVector3d ProjectDirectionToSurface(const FVector3d& Direction, const FVector3d& SurfaceNormal)
	{
		const FVector3d Projected = ProjectOntoTangent(Direction, SurfaceNormal);
		return Projected.SquaredLength() > DirectionDegeneracyThreshold * DirectionDegeneracyThreshold
			? Projected.GetSafeNormal()
			: FVector3d::ZeroVector;
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

	const FCarriedSample* FindCarriedSampleForCanonicalVertex(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 CanonicalSampleIndex)
	{
		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr)
		{
			return nullptr;
		}

		const int32* CarriedIndexPtr = Plate->CanonicalToCarriedIndex.Find(CanonicalSampleIndex);
		if (CarriedIndexPtr == nullptr || !Plate->CarriedSamples.IsValidIndex(*CarriedIndexPtr))
		{
			return nullptr;
		}

		const FCarriedSample& CarriedSample = Plate->CarriedSamples[*CarriedIndexPtr];
		return CarriedSample.CanonicalSampleIndex == CanonicalSampleIndex ? &CarriedSample : nullptr;
	}

	const FCarriedSample* FindCarriedSampleForPreviousOwnerCanonicalVertex(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PreviousCanonicalPlateIds,
		const int32 CanonicalSampleIndex)
	{
		if (!PreviousCanonicalPlateIds.IsValidIndex(CanonicalSampleIndex))
		{
			return nullptr;
		}

		const int32 PreviousOwnerPlateId = PreviousCanonicalPlateIds[CanonicalSampleIndex];
		if (PreviousOwnerPlateId == INDEX_NONE)
		{
			return nullptr;
		}

		return FindCarriedSampleForCanonicalVertex(Planet, PreviousOwnerPlateId, CanonicalSampleIndex);
	}

	void PopulateV6CarriedSampleFromCanonical(
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

	void SyncV6CarriedSamplesFromCanonicalIndices(
		FTectonicPlanet& Planet,
		const TArray<int32>& CanonicalSampleIndices)
	{
		for (const int32 CanonicalSampleIndex : CanonicalSampleIndices)
		{
			if (!Planet.Samples.IsValidIndex(CanonicalSampleIndex))
			{
				continue;
			}

			const int32 PlateArrayIndex =
				Planet.FindPlateArrayIndexById(Planet.Samples[CanonicalSampleIndex].PlateId);
			if (!Planet.Plates.IsValidIndex(PlateArrayIndex))
			{
				continue;
			}

			FPlate& Plate = Planet.Plates[PlateArrayIndex];
			const int32* CarriedIndexPtr = Plate.CanonicalToCarriedIndex.Find(CanonicalSampleIndex);
			if (CarriedIndexPtr == nullptr || !Plate.CarriedSamples.IsValidIndex(*CarriedIndexPtr))
			{
				continue;
			}

			PopulateV6CarriedSampleFromCanonical(
				Planet,
				CanonicalSampleIndex,
				Plate.CarriedSamples[*CarriedIndexPtr]);
		}
	}

	void RemoveV6CarriedSampleForCanonicalVertex(
		FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 CanonicalSampleIndex)
	{
		const int32 PlateArrayIndex = Planet.FindPlateArrayIndexById(PlateId);
		if (!Planet.Plates.IsValidIndex(PlateArrayIndex))
		{
			return;
		}

		FPlate& Plate = Planet.Plates[PlateArrayIndex];
		const int32* CarriedIndexPtr = Plate.CanonicalToCarriedIndex.Find(CanonicalSampleIndex);
		if (CarriedIndexPtr == nullptr || !Plate.CarriedSamples.IsValidIndex(*CarriedIndexPtr))
		{
			return;
		}

		const int32 RemoveIndex = *CarriedIndexPtr;
		const int32 LastIndex = Plate.CarriedSamples.Num() - 1;
		if (RemoveIndex != LastIndex && Plate.CarriedSamples.IsValidIndex(LastIndex))
		{
			const FCarriedSample MovedSample = Plate.CarriedSamples[LastIndex];
			Plate.CarriedSamples[RemoveIndex] = MovedSample;
			Plate.CanonicalToCarriedIndex.Add(MovedSample.CanonicalSampleIndex, RemoveIndex);
		}

		Plate.CarriedSamples.RemoveAt(LastIndex, 1, EAllowShrinking::No);
		Plate.CanonicalToCarriedIndex.Remove(CanonicalSampleIndex);
	}

	void UpsertV6CarriedSampleForCanonicalVertex(
		FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 CanonicalSampleIndex)
	{
		const int32 PlateArrayIndex = Planet.FindPlateArrayIndexById(PlateId);
		if (!Planet.Plates.IsValidIndex(PlateArrayIndex))
		{
			return;
		}

		FPlate& Plate = Planet.Plates[PlateArrayIndex];
		if (const int32* CarriedIndexPtr = Plate.CanonicalToCarriedIndex.Find(CanonicalSampleIndex))
		{
			if (Plate.CarriedSamples.IsValidIndex(*CarriedIndexPtr))
			{
				PopulateV6CarriedSampleFromCanonical(
					Planet,
					CanonicalSampleIndex,
					Plate.CarriedSamples[*CarriedIndexPtr]);
				return;
			}
		}

		FCarriedSample NewCarriedSample;
		PopulateV6CarriedSampleFromCanonical(Planet, CanonicalSampleIndex, NewCarriedSample);
		const int32 NewIndex = Plate.CarriedSamples.Add(NewCarriedSample);
		Plate.CanonicalToCarriedIndex.Add(CanonicalSampleIndex, NewIndex);
	}

	void RebuildV6MembershipFromCanonical(FTectonicPlanet& Planet)
	{
		for (FPlate& Plate : Planet.Plates)
		{
			Plate.MemberSamples.Reset();
		}

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const int32 PlateArrayIndex = Planet.FindPlateArrayIndexById(Planet.Samples[SampleIndex].PlateId);
			if (Planet.Plates.IsValidIndex(PlateArrayIndex))
			{
				Planet.Plates[PlateArrayIndex].MemberSamples.Add(SampleIndex);
			}
		}
	}

	void RecomputeV6BoundaryFlagsFromCanonical(FTectonicPlanet& Planet)
	{
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			FSample& Sample = Planet.Samples[SampleIndex];
			bool bBoundary = (Sample.PlateId == INDEX_NONE);
			if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex))
					{
						continue;
					}

					if (Planet.Samples[NeighborIndex].PlateId != Sample.PlateId)
					{
						bBoundary = true;
						break;
					}
				}
			}

			Sample.bIsBoundary = bBoundary;
		}
	}

	uint64 MakeOrderedSamplePairKey(const int32 SampleIndexA, const int32 SampleIndexB)
	{
		const uint32 MinSampleIndex = static_cast<uint32>(FMath::Min(SampleIndexA, SampleIndexB));
		const uint32 MaxSampleIndex = static_cast<uint32>(FMath::Max(SampleIndexA, SampleIndexB));
		return (static_cast<uint64>(MinSampleIndex) << 32) | static_cast<uint64>(MaxSampleIndex);
	}

	uint64 MakeOrderedPlatePairKey(const int32 PlateA, const int32 PlateB)
	{
		const uint32 MinPlateId = static_cast<uint32>(FMath::Min(PlateA, PlateB));
		const uint32 MaxPlateId = static_cast<uint32>(FMath::Max(PlateA, PlateB));
		return (static_cast<uint64>(MinPlateId) << 32) | static_cast<uint64>(MaxPlateId);
	}

	void BuildCopiedFrontierConvergentEdgeSet(
		const FTectonicPlanet& Planet,
		const double MaxConvergentRatio,
		const double MinConvergenceSpeedKmPerMy,
		TSet<uint64>& OutStrongConvergentEdgeKeys)
	{
		OutStrongConvergentEdgeKeys.Reset();
		if (Planet.SampleAdjacency.Num() != Planet.Samples.Num())
		{
			return;
		}

		for (int32 SampleIndexA = 0; SampleIndexA < Planet.SampleAdjacency.Num(); ++SampleIndexA)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndexA))
			{
				continue;
			}

			const int32 PlateAId = Planet.Samples[SampleIndexA].PlateId;
			const FPlate* PlateA = FindPlateById(Planet, PlateAId);
			if (PlateA == nullptr)
			{
				continue;
			}

			for (const int32 SampleIndexB : Planet.SampleAdjacency[SampleIndexA])
			{
				if (SampleIndexB <= SampleIndexA || !Planet.Samples.IsValidIndex(SampleIndexB))
				{
					continue;
				}

				const int32 PlateBId = Planet.Samples[SampleIndexB].PlateId;
				const FPlate* PlateB = FindPlateById(Planet, PlateBId);
				if (PlateB == nullptr || PlateAId == PlateBId)
				{
					continue;
				}

				const FVector3d PosA = Planet.Samples[SampleIndexA].Position.GetSafeNormal();
				const FVector3d PosB = Planet.Samples[SampleIndexB].Position.GetSafeNormal();
				FVector3d Midpoint = (PosA + PosB).GetSafeNormal();
				if (Midpoint.IsNearlyZero())
				{
					Midpoint = PosA;
				}

				const FVector3d SurfaceVelocityA = ComputePlateSurfaceVelocity(*PlateA, Midpoint, Planet.PlanetRadiusKm);
				const FVector3d SurfaceVelocityB = ComputePlateSurfaceVelocity(*PlateB, Midpoint, Planet.PlanetRadiusKm);
				const FVector3d RelativeVelocity = SurfaceVelocityB - SurfaceVelocityA;
				const double RelativeSpeed = RelativeVelocity.Length();
				if (RelativeSpeed <= UE_DOUBLE_SMALL_NUMBER)
				{
					continue;
				}

				const FVector3d Separation = (PosB - PosA).GetSafeNormal();
				if (Separation.IsNearlyZero())
				{
					continue;
				}

				const double NormalComponent = RelativeVelocity.Dot(Separation);
				const double Ratio = NormalComponent / RelativeSpeed;
				const double ConvergenceSpeedKmPerMy = FMath::Abs(NormalComponent);
				if (Ratio >= MaxConvergentRatio ||
					ConvergenceSpeedKmPerMy + UE_KINDA_SMALL_NUMBER < MinConvergenceSpeedKmPerMy)
				{
					continue;
				}

				OutStrongConvergentEdgeKeys.Add(MakeOrderedSamplePairKey(SampleIndexA, SampleIndexB));
			}
		}
	}

	void BuildCopiedFrontierStrongConvergentEdgeSet(
		const FTectonicPlanet& Planet,
		TSet<uint64>& OutStrongConvergentEdgeKeys)
	{
		BuildCopiedFrontierConvergentEdgeSet(
			Planet,
			DestructiveConvergentThresholdRatio,
			BoundaryNormalVelocityThresholdKmPerMy,
			OutStrongConvergentEdgeKeys);
	}

	void BuildCopiedFrontierTrackedContextConvergentEdgeSet(
		const FTectonicPlanet& Planet,
		TSet<uint64>& OutTrackedContextConvergentEdgeKeys)
	{
		BuildCopiedFrontierConvergentEdgeSet(
			Planet,
			DestructiveTrackedConvergentThresholdRatio,
			DestructiveTrackedMinConvergenceSpeedKmPerMy,
			OutTrackedContextConvergentEdgeKeys);
	}

	void AddCopiedFrontierActiveCollisionPair(
		const int32 PlateA,
		const int32 PlateB,
		const TArray<int32>& SampleIndices,
		TArray<FV6CopiedFrontierActiveCollisionPair>& InOutPairs,
		TMap<uint64, int32>& InOutPairIndexByKey)
	{
		if (PlateA == INDEX_NONE || PlateB == INDEX_NONE || PlateA == PlateB || SampleIndices.IsEmpty())
		{
			return;
		}

		const uint64 PairKey = MakeOrderedPlatePairKey(PlateA, PlateB);
		int32 PairIndex = INDEX_NONE;
		if (const int32* ExistingIndexPtr = InOutPairIndexByKey.Find(PairKey))
		{
			PairIndex = *ExistingIndexPtr;
		}
		else
		{
			PairIndex = InOutPairs.AddDefaulted();
			FV6CopiedFrontierActiveCollisionPair& Pair = InOutPairs[PairIndex];
			Pair.PlateA = FMath::Min(PlateA, PlateB);
			Pair.PlateB = FMath::Max(PlateA, PlateB);
			InOutPairIndexByKey.Add(PairKey, PairIndex);
		}

		FV6CopiedFrontierActiveCollisionPair& Pair = InOutPairs[PairIndex];
		for (const int32 SampleIndex : SampleIndices)
		{
			Pair.SampleIndices.Add(SampleIndex);
		}
	}

	void BuildCopiedFrontierActiveCollisionPairs(
		const FTectonicPlanet& Planet,
		TArray<FV6CopiedFrontierActiveCollisionPair>& OutPairs)
	{
		OutPairs.Reset();
		TMap<uint64, int32> PairIndexByKey;

		if (Planet.PendingGeometricCollisionEvent.bValid)
		{
			TArray<int32> CollisionSampleIndices = Planet.PendingGeometricCollisionEvent.OverlapSampleIndices;
			for (const int32 SampleIndex : Planet.PendingGeometricCollisionEvent.SamplesFromAInsideB)
			{
				CollisionSampleIndices.AddUnique(SampleIndex);
			}
			for (const int32 SampleIndex : Planet.PendingGeometricCollisionEvent.SamplesFromBInsideA)
			{
				CollisionSampleIndices.AddUnique(SampleIndex);
			}

			AddCopiedFrontierActiveCollisionPair(
				Planet.PendingGeometricCollisionEvent.PlateA,
				Planet.PendingGeometricCollisionEvent.PlateB,
				CollisionSampleIndices,
				OutPairs,
				PairIndexByKey);
		}

		if (Planet.PendingBoundaryContactCollisionEvent.bValid)
		{
			AddCopiedFrontierActiveCollisionPair(
				Planet.PendingBoundaryContactCollisionEvent.PlateA,
				Planet.PendingBoundaryContactCollisionEvent.PlateB,
				Planet.PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices,
				OutPairs,
				PairIndexByKey);
		}
	}

	bool TriangleContainsStrongConvergentEdge(
		const FIntVector& Triangle,
		const TSet<uint64>& StrongConvergentEdgeKeys)
	{
		return
			StrongConvergentEdgeKeys.Contains(MakeOrderedSamplePairKey(Triangle.X, Triangle.Y)) ||
			StrongConvergentEdgeKeys.Contains(MakeOrderedSamplePairKey(Triangle.Y, Triangle.Z)) ||
			StrongConvergentEdgeKeys.Contains(MakeOrderedSamplePairKey(Triangle.Z, Triangle.X));
	}

	bool TriangleMatchesActiveCollisionPair(
		const FIntVector& Triangle,
		const TArray<int32>& InvolvedPlateIds,
		const FV6CopiedFrontierActiveCollisionPair& Pair)
	{
		if (!InvolvedPlateIds.Contains(Pair.PlateA) || !InvolvedPlateIds.Contains(Pair.PlateB))
		{
			return false;
		}

		int32 CollisionVertexCount = 0;
		CollisionVertexCount += Pair.SampleIndices.Contains(Triangle.X) ? 1 : 0;
		CollisionVertexCount += Pair.SampleIndices.Contains(Triangle.Y) ? 1 : 0;
		CollisionVertexCount += Pair.SampleIndices.Contains(Triangle.Z) ? 1 : 0;
		return CollisionVertexCount >= 2;
	}

	int32 CountTriangleVerticesMatchingOrogeny(
		const FTectonicPlanet& Planet,
		const FIntVector& Triangle,
		const EOrogenyType OrogenyType)
	{
		int32 MatchingCount = 0;
		MatchingCount +=
			Planet.Samples.IsValidIndex(Triangle.X) && Planet.Samples[Triangle.X].OrogenyType == OrogenyType
				? 1
				: 0;
		MatchingCount +=
			Planet.Samples.IsValidIndex(Triangle.Y) && Planet.Samples[Triangle.Y].OrogenyType == OrogenyType
				? 1
				: 0;
		MatchingCount +=
			Planet.Samples.IsValidIndex(Triangle.Z) && Planet.Samples[Triangle.Z].OrogenyType == OrogenyType
				? 1
				: 0;
		return MatchingCount;
	}

	bool TriangleHasTrackedSubductionContext(
		const FTectonicPlanet& Planet,
		const FIntVector& Triangle,
		const TSet<uint64>& TrackedContextConvergentEdgeKeys)
	{
		const bool bHasTrackedContextConvergentEdge =
			!TrackedContextConvergentEdgeKeys.IsEmpty() &&
			TriangleContainsStrongConvergentEdge(Triangle, TrackedContextConvergentEdgeKeys);
		if (!bHasTrackedContextConvergentEdge)
		{
			return false;
		}

		return
			(Planet.Samples.IsValidIndex(Triangle.X) && Planet.Samples[Triangle.X].SubductionDistanceKm >= 0.0f) ||
			(Planet.Samples.IsValidIndex(Triangle.Y) && Planet.Samples[Triangle.Y].SubductionDistanceKm >= 0.0f) ||
			(Planet.Samples.IsValidIndex(Triangle.Z) && Planet.Samples[Triangle.Z].SubductionDistanceKm >= 0.0f);
	}

	bool TriangleHasTrackedCollisionContext(
		const FTectonicPlanet& Planet,
		const FIntVector& Triangle,
		const TArray<int32>& InvolvedPlateIds,
		const TArray<FV6CopiedFrontierActiveCollisionPair>& ActiveCollisionPairs,
		const TSet<uint64>& TrackedContextConvergentEdgeKeys)
	{
		for (const FV6CopiedFrontierActiveCollisionPair& Pair : ActiveCollisionPairs)
		{
			if (TriangleMatchesActiveCollisionPair(Triangle, InvolvedPlateIds, Pair))
			{
				return true;
			}
		}

		return
			TriangleContainsStrongConvergentEdge(Triangle, TrackedContextConvergentEdgeKeys) &&
			CountTriangleVerticesMatchingOrogeny(Planet, Triangle, EOrogenyType::Himalayan) >= 2;
	}

	int32 ChooseHighestOverlapScorePlateId(
		const FTectonicPlanet& Planet,
		const TArray<int32>& CandidatePlateIds)
	{
		int32 BestPlateId = INDEX_NONE;
		int32 BestOverlapScore = TNumericLimits<int32>::Lowest();
		for (const int32 PlateId : CandidatePlateIds)
		{
			const FPlate* Plate = FindPlateById(Planet, PlateId);
			if (Plate == nullptr)
			{
				continue;
			}

			if (Plate->OverlapScore > BestOverlapScore ||
				(Plate->OverlapScore == BestOverlapScore &&
					(BestPlateId == INDEX_NONE || PlateId < BestPlateId)))
			{
				BestPlateId = PlateId;
				BestOverlapScore = Plate->OverlapScore;
			}
		}

		return BestPlateId;
	}

	int32 ChooseCopiedFrontierPreferredContinuationPlateId(
		const FTectonicPlanet& Planet,
		const TArray<int32>& InvolvedPlateIds,
		const bool bCollisionTriangle)
	{
		if (bCollisionTriangle)
		{
			if (Planet.PendingGeometricCollisionEvent.bValid)
			{
				const int32 PlateA = Planet.PendingGeometricCollisionEvent.PlateA;
				const int32 PlateB = Planet.PendingGeometricCollisionEvent.PlateB;
				const int32 OverridingPlateId = Planet.PendingGeometricCollisionEvent.OverridingPlateId;
				if (InvolvedPlateIds.Contains(PlateA) &&
					InvolvedPlateIds.Contains(PlateB) &&
					OverridingPlateId != INDEX_NONE)
				{
					return OverridingPlateId;
				}
			}

			if (Planet.PendingBoundaryContactCollisionEvent.bValid)
			{
				const int32 PlateA = Planet.PendingBoundaryContactCollisionEvent.PlateA;
				const int32 PlateB = Planet.PendingBoundaryContactCollisionEvent.PlateB;
				if (InvolvedPlateIds.Contains(PlateA) && InvolvedPlateIds.Contains(PlateB))
				{
					return IsOverridingPlateLocal(Planet, PlateA, PlateB) ? PlateA : PlateB;
				}
			}
		}

		return ChooseHighestOverlapScorePlateId(Planet, InvolvedPlateIds);
	}

	void ResetCopiedFrontierDestructiveTrackingState(
		const FTectonicPlanet& Planet,
		TArray<uint8>& OutTrackedKinds,
		TArray<int32>& OutTrackedPreferredContinuationPlateIds,
		TArray<int32>& OutTrackedSourcePlateIds,
		TArray<float>& OutTrackedDistancesKm,
		TArray<uint8>& OutTrackedSeedOriginFlags,
		TArray<uint8>& OutTrackedSeedSurvivalLoggedFlags,
		TArray<uint8>& OutTrackedTopologyNeighborOriginFlags)
	{
		const int32 TriangleCount = Planet.TriangleIndices.Num();
		OutTrackedKinds.Init(0, TriangleCount);
		OutTrackedPreferredContinuationPlateIds.Init(INDEX_NONE, TriangleCount);
		OutTrackedSourcePlateIds.Init(INDEX_NONE, TriangleCount);
		OutTrackedDistancesKm.Init(0.0f, TriangleCount);
		OutTrackedSeedOriginFlags.Init(0, TriangleCount);
		OutTrackedSeedSurvivalLoggedFlags.Init(0, TriangleCount);
		OutTrackedTopologyNeighborOriginFlags.Init(0, TriangleCount);
	}

	void AccumulateCopiedFrontierTrackedDestructiveCounts(
		const TArray<uint8>& TrackedKinds,
		FV6CopiedFrontierDestructiveTrackingUpdateStats& InOutStats)
	{
		for (const uint8 TrackedKind : TrackedKinds)
		{
			switch (static_cast<EV6CopiedFrontierDestructiveKind>(TrackedKind))
			{
			case EV6CopiedFrontierDestructiveKind::Subduction:
				++InOutStats.TrackedSubductionTriangleCount;
				break;
			case EV6CopiedFrontierDestructiveKind::Collision:
				++InOutStats.TrackedCollisionTriangleCount;
				break;
			case EV6CopiedFrontierDestructiveKind::None:
			default:
				break;
			}
		}

		InOutStats.TrackedTriangleCount =
			InOutStats.TrackedSubductionTriangleCount +
			InOutStats.TrackedCollisionTriangleCount;
	}

	int32 CountTrackedSeedOriginTriangles(
		const TArray<uint8>& TrackedKinds,
		const TArray<uint8>& TrackedSeedOriginFlags)
	{
		const int32 TriangleCount = FMath::Min(TrackedKinds.Num(), TrackedSeedOriginFlags.Num());
		int32 SeedTrackedCount = 0;
		for (int32 TriangleIndex = 0; TriangleIndex < TriangleCount; ++TriangleIndex)
		{
			if (TrackedKinds[TriangleIndex] != static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None) &&
				TrackedSeedOriginFlags[TriangleIndex] != 0)
			{
				++SeedTrackedCount;
			}
		}

		return SeedTrackedCount;
	}

	int32 CountTrackedTopologyNeighborOriginTriangles(
		const TArray<uint8>& TrackedKinds,
		const TArray<uint8>& TrackedTopologyNeighborOriginFlags)
	{
		const int32 TriangleCount = FMath::Min(TrackedKinds.Num(), TrackedTopologyNeighborOriginFlags.Num());
		int32 TopologyTrackedCount = 0;
		for (int32 TriangleIndex = 0; TriangleIndex < TriangleCount; ++TriangleIndex)
		{
			if (TrackedKinds[TriangleIndex] != static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None) &&
				TrackedTopologyNeighborOriginFlags[TriangleIndex] != 0)
			{
				++TopologyTrackedCount;
			}
		}

		return TopologyTrackedCount;
	}

	void AccumulateTrackingLifecycleStats(
		FTectonicPlanetV6DestructiveTrackingLifecycleStats& InOutTotals,
		const FTectonicPlanetV6DestructiveTrackingLifecycleStats& Delta)
	{
		InOutTotals.EntryCandidateCount += Delta.EntryCandidateCount;
		InOutTotals.EntrySeedConvergentEdgeCandidateCount += Delta.EntrySeedConvergentEdgeCandidateCount;
		InOutTotals.EntryOverlapTestedCount += Delta.EntryOverlapTestedCount;
		InOutTotals.EntryAdmittedCount += Delta.EntryAdmittedCount;
		InOutTotals.EntrySeedConvergentEdgeAdmittedCount += Delta.EntrySeedConvergentEdgeAdmittedCount;
		InOutTotals.EntrySeedConvergentEdgeSurvivedOneTimestepCount += Delta.EntrySeedConvergentEdgeSurvivedOneTimestepCount;
		InOutTotals.EntrySeedConvergentEdgeSurvivedToRemeshCount += Delta.EntrySeedConvergentEdgeSurvivedToRemeshCount;
		InOutTotals.EntryRejectedNoOverlapCount += Delta.EntryRejectedNoOverlapCount;
		InOutTotals.EntryRejectedAuthorizationCount += Delta.EntryRejectedAuthorizationCount;
		InOutTotals.EntryRejectedAlreadyTrackedCount += Delta.EntryRejectedAlreadyTrackedCount;
		InOutTotals.EntryAdmittedSubductionCount += Delta.EntryAdmittedSubductionCount;
		InOutTotals.EntryAdmittedCollisionCount += Delta.EntryAdmittedCollisionCount;
		InOutTotals.ActiveAdvanceCount += Delta.ActiveAdvanceCount;
		InOutTotals.ActiveOverlapConfirmedCount += Delta.ActiveOverlapConfirmedCount;
		InOutTotals.ActiveOverlapRejectedCount += Delta.ActiveOverlapRejectedCount;
		InOutTotals.NeighborCandidateGeneratedCount += Delta.NeighborCandidateGeneratedCount;
		InOutTotals.NeighborCandidateTestedCount += Delta.NeighborCandidateTestedCount;
		InOutTotals.NeighborAdmittedCount += Delta.NeighborAdmittedCount;
		InOutTotals.NeighborRejectedNoOverlapCount += Delta.NeighborRejectedNoOverlapCount;
		InOutTotals.NeighborRejectedAuthorizationCount += Delta.NeighborRejectedAuthorizationCount;
		InOutTotals.NeighborRejectedAlreadyTrackedCount += Delta.NeighborRejectedAlreadyTrackedCount;
		InOutTotals.NeighborAdmittedSubductionCount += Delta.NeighborAdmittedSubductionCount;
		InOutTotals.NeighborAdmittedCollisionCount += Delta.NeighborAdmittedCollisionCount;
		InOutTotals.TopologyNeighborCandidateGeneratedCount += Delta.TopologyNeighborCandidateGeneratedCount;
		InOutTotals.TopologyNeighborAdmittedCount += Delta.TopologyNeighborAdmittedCount;
		InOutTotals.TopologyNeighborExpiredBeforeRemeshCount += Delta.TopologyNeighborExpiredBeforeRemeshCount;
		InOutTotals.DirectionalNeighborCandidateConsideredCount += Delta.DirectionalNeighborCandidateConsideredCount;
		InOutTotals.DirectionalNeighborAdmittedCount += Delta.DirectionalNeighborAdmittedCount;
		InOutTotals.DirectionalNeighborRejectedNotInwardCount += Delta.DirectionalNeighborRejectedNotInwardCount;
		InOutTotals.DirectionalNeighborSurvivedToRemeshCount += Delta.DirectionalNeighborSurvivedToRemeshCount;
		InOutTotals.ExpiredSubductionCount += Delta.ExpiredSubductionCount;
		InOutTotals.ExpiredCollisionCount += Delta.ExpiredCollisionCount;
	}

	void BuildCopiedFrontierTrackingPlateOverlapData(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		TArray<FV6CopiedFrontierTrackingPlateOverlapData>& OutOverlapData)
	{
		OutOverlapData.SetNum(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			FV6CopiedFrontierTrackingPlateOverlapData& PlateOverlapData = OutOverlapData[PlateIndex];
			PlateOverlapData = FV6CopiedFrontierTrackingPlateOverlapData{};
			PlateOverlapData.PlateId = Planet.Plates[PlateIndex].Id;
			if (!PlateMeshes.IsValidIndex(PlateIndex))
			{
				continue;
			}

			const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = PlateMeshes[PlateIndex];
			PlateOverlapData.LocalTriangles = Mesh.LocalTriangles;
			PlateOverlapData.RotatedVertices.Reserve(Mesh.BaseVertices.Num());
			for (const FVector3d& BaseVertex : Mesh.BaseVertices)
			{
				PlateOverlapData.RotatedVertices.Add(
					Planet.Plates[PlateIndex].CumulativeRotation.RotateVector(BaseVertex).GetSafeNormal());
			}

			for (int32 LocalTriangleIndex = 0; LocalTriangleIndex < Mesh.GlobalTriangleIndices.Num(); ++LocalTriangleIndex)
			{
				PlateOverlapData.LocalTriangleIndicesByGlobalTriangle.Add(
					Mesh.GlobalTriangleIndices[LocalTriangleIndex],
					LocalTriangleIndex);
			}
		}
	}

	void CollectTrianglePlateIds(
		const FTectonicPlanet& Planet,
		const int32 TriangleIndex,
		TArray<int32>& OutPlateIds)
	{
		OutPlateIds.Reset();
		if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
		{
			return;
		}

		const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
		if (!Planet.Samples.IsValidIndex(Triangle.X) ||
			!Planet.Samples.IsValidIndex(Triangle.Y) ||
			!Planet.Samples.IsValidIndex(Triangle.Z))
		{
			return;
		}

		CollectBoundaryTrianglePlateIds(
			Planet.Samples[Triangle.X].PlateId,
			Planet.Samples[Triangle.Y].PlateId,
			Planet.Samples[Triangle.Z].PlateId,
			OutPlateIds);
	}

	bool TriangleContainsPlateId(
		const FTectonicPlanet& Planet,
		const int32 TriangleIndex,
		const int32 PlateId)
	{
		if (PlateId == INDEX_NONE)
		{
			return false;
		}

		TArray<int32> InvolvedPlateIds;
		CollectTrianglePlateIds(Planet, TriangleIndex, InvolvedPlateIds);
		return InvolvedPlateIds.Contains(PlateId);
	}

	bool ComputeTrackedTriangleSourcePlateSubductionDistanceKm(
		const FTectonicPlanet& Planet,
		const int32 TriangleIndex,
		const int32 SourcePlateId,
		double& OutDistanceKm)
	{
		OutDistanceKm = -1.0;
		if (SourcePlateId == INDEX_NONE || !Planet.TriangleIndices.IsValidIndex(TriangleIndex))
		{
			return false;
		}

		const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
		const int32 TriangleSampleIndices[3] = { Triangle.X, Triangle.Y, Triangle.Z };
		double DistanceSumKm = 0.0;
		int32 DistanceCount = 0;
		for (const int32 SampleIndex : TriangleSampleIndices)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FSample& Sample = Planet.Samples[SampleIndex];
			if (Sample.PlateId != SourcePlateId || Sample.SubductionDistanceKm < 0.0f)
			{
				continue;
			}

			DistanceSumKm += static_cast<double>(Sample.SubductionDistanceKm);
			++DistanceCount;
		}

		if (DistanceCount <= 0)
		{
			return false;
		}

		OutDistanceKm = DistanceSumKm / static_cast<double>(DistanceCount);
		return true;
	}

	int32 ChooseTrackedTriangleSourcePlateId(
		const FTectonicPlanet& Planet,
		const TArray<int32>& InvolvedPlateIds,
		const int32 PreferredContinuationPlateId,
		const int32 FallbackSourcePlateId = INDEX_NONE)
	{
		if (Planet.PendingGeometricCollisionEvent.bValid &&
			PreferredContinuationPlateId == Planet.PendingGeometricCollisionEvent.OverridingPlateId &&
			InvolvedPlateIds.Contains(Planet.PendingGeometricCollisionEvent.SubductingPlateId))
		{
			return Planet.PendingGeometricCollisionEvent.SubductingPlateId;
		}

		if (FallbackSourcePlateId != INDEX_NONE &&
			FallbackSourcePlateId != PreferredContinuationPlateId &&
			InvolvedPlateIds.Contains(FallbackSourcePlateId))
		{
			return FallbackSourcePlateId;
		}

		if (InvolvedPlateIds.Num() == 1 && InvolvedPlateIds[0] != PreferredContinuationPlateId)
		{
			return InvolvedPlateIds[0];
		}

		int32 LowestOverlapPlateId = INDEX_NONE;
		int32 LowestOverlapScore = TNumericLimits<int32>::Max();
		for (const int32 PlateId : InvolvedPlateIds)
		{
			if (PlateId == PreferredContinuationPlateId)
			{
				continue;
			}

			const FPlate* Plate = FindPlateById(Planet, PlateId);
			const int32 OverlapScore = Plate != nullptr ? Plate->OverlapScore : TNumericLimits<int32>::Max();
			if (LowestOverlapPlateId == INDEX_NONE ||
				OverlapScore < LowestOverlapScore ||
				(OverlapScore == LowestOverlapScore && PlateId < LowestOverlapPlateId))
			{
				LowestOverlapPlateId = PlateId;
				LowestOverlapScore = OverlapScore;
			}
		}

		if (LowestOverlapPlateId != INDEX_NONE)
		{
			return LowestOverlapPlateId;
		}

		return FallbackSourcePlateId != INDEX_NONE
			? FallbackSourcePlateId
			: (InvolvedPlateIds.IsEmpty() ? INDEX_NONE : InvolvedPlateIds[0]);
	}

	FVector3d ComputeTrackedTriangleBarycenterForPlate(
		const FTectonicPlanet& Planet,
		const int32 TriangleIndex,
		const int32 SourcePlateId)
	{
		if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
		{
			return FVector3d::ZeroVector;
		}

		const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
		if (!Planet.Samples.IsValidIndex(Triangle.X) ||
			!Planet.Samples.IsValidIndex(Triangle.Y) ||
			!Planet.Samples.IsValidIndex(Triangle.Z))
		{
			return FVector3d::ZeroVector;
		}

		const FVector3d CanonicalBarycenter =
			(Planet.Samples[Triangle.X].Position +
				Planet.Samples[Triangle.Y].Position +
				Planet.Samples[Triangle.Z].Position).GetSafeNormal();
		if (CanonicalBarycenter.IsNearlyZero())
		{
			return FVector3d::ZeroVector;
		}

		const FPlate* SourcePlate = FindPlateById(Planet, SourcePlateId);
		return SourcePlate != nullptr
			? SourcePlate->CumulativeRotation.RotateVector(CanonicalBarycenter).GetSafeNormal()
			: CanonicalBarycenter;
	}

	void CollectTrackedTriangleCandidateNeighborhood(
		const FTectonicPlanet& Planet,
		const int32 TriangleIndex,
		TArray<int32, TInlineAllocator<8>>& OutGlobalTriangleIndices)
	{
		OutGlobalTriangleIndices.Reset();
		OutGlobalTriangleIndices.Add(TriangleIndex);
		if (!Planet.TriangleAdjacency.IsValidIndex(TriangleIndex))
		{
			return;
		}

		for (const int32 NeighborTriangleIndex : Planet.TriangleAdjacency[TriangleIndex])
		{
			OutGlobalTriangleIndices.AddUnique(NeighborTriangleIndex);
		}
	}

	bool DoesTrackedTriangleBarycenterOverlapPreferredPlate(
		const FTectonicPlanet& Planet,
		const TArray<FV6CopiedFrontierTrackingPlateOverlapData>& PlateOverlapData,
		const int32 TriangleIndex,
		const int32 SourcePlateId,
		const int32 PreferredContinuationPlateId)
	{
		if (PreferredContinuationPlateId == INDEX_NONE || PreferredContinuationPlateId == SourcePlateId)
		{
			return false;
		}

		const int32 PreferredPlateIndex = Planet.FindPlateArrayIndexById(PreferredContinuationPlateId);
		if (!PlateOverlapData.IsValidIndex(PreferredPlateIndex))
		{
			return false;
		}

		const FV6CopiedFrontierTrackingPlateOverlapData& PreferredPlateOverlapData =
			PlateOverlapData[PreferredPlateIndex];
		if (PreferredPlateOverlapData.LocalTriangles.IsEmpty())
		{
			return false;
		}

		const FVector3d QueryPoint =
			ComputeTrackedTriangleBarycenterForPlate(Planet, TriangleIndex, SourcePlateId);
		if (QueryPoint.IsNearlyZero())
		{
			return false;
		}

		TArray<int32, TInlineAllocator<8>> CandidateNeighborhood;
		CollectTrackedTriangleCandidateNeighborhood(Planet, TriangleIndex, CandidateNeighborhood);
		TArray<int32, TInlineAllocator<16>> LocalTriangleIndices;
		for (const int32 GlobalTriangleIndex : CandidateNeighborhood)
		{
			LocalTriangleIndices.Reset();
			PreferredPlateOverlapData.LocalTriangleIndicesByGlobalTriangle.MultiFind(GlobalTriangleIndex, LocalTriangleIndices);
			for (const int32 LocalTriangleIndex : LocalTriangleIndices)
			{
				if (!PreferredPlateOverlapData.LocalTriangles.IsValidIndex(LocalTriangleIndex))
				{
					continue;
				}

				const UE::Geometry::FIndex3i& LocalTriangle =
					PreferredPlateOverlapData.LocalTriangles[LocalTriangleIndex];
				if (!PreferredPlateOverlapData.RotatedVertices.IsValidIndex(LocalTriangle.A) ||
					!PreferredPlateOverlapData.RotatedVertices.IsValidIndex(LocalTriangle.B) ||
					!PreferredPlateOverlapData.RotatedVertices.IsValidIndex(LocalTriangle.C))
				{
					continue;
				}

				if (IsPointInsideSphericalTriangleRobust(
						PreferredPlateOverlapData.RotatedVertices[LocalTriangle.A],
						PreferredPlateOverlapData.RotatedVertices[LocalTriangle.B],
						PreferredPlateOverlapData.RotatedVertices[LocalTriangle.C],
						QueryPoint))
				{
					return true;
				}
			}
		}

		return false;
	}

	bool DetermineSeedTrackedTriangleContext(
		const FTectonicPlanet& Planet,
		const TSet<uint64>& TrackedContextConvergentEdgeKeys,
		const TArray<FV6CopiedFrontierActiveCollisionPair>& ActiveCollisionPairs,
		const int32 TriangleIndex,
		EV6CopiedFrontierDestructiveKind& OutTrackedKind,
		int32& OutPreferredContinuationPlateId,
		int32& OutSourcePlateId)
	{
		OutTrackedKind = EV6CopiedFrontierDestructiveKind::None;
		OutPreferredContinuationPlateId = INDEX_NONE;
		OutSourcePlateId = INDEX_NONE;

		if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
		{
			return false;
		}

		const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
		TArray<int32> InvolvedPlateIds;
		CollectTrianglePlateIds(Planet, TriangleIndex, InvolvedPlateIds);
		if (InvolvedPlateIds.Num() <= 1)
		{
			return false;
		}

		const bool bCollisionTriangle =
			TriangleHasTrackedCollisionContext(
				Planet,
				Triangle,
				InvolvedPlateIds,
				ActiveCollisionPairs,
				TrackedContextConvergentEdgeKeys);
		const bool bSubductionTriangle =
			!bCollisionTriangle &&
			TriangleHasTrackedSubductionContext(
				Planet,
				Triangle,
				TrackedContextConvergentEdgeKeys);
		if (!bCollisionTriangle && !bSubductionTriangle)
		{
			return false;
		}

		OutTrackedKind =
			bCollisionTriangle
				? EV6CopiedFrontierDestructiveKind::Collision
				: EV6CopiedFrontierDestructiveKind::Subduction;
		OutPreferredContinuationPlateId =
			ChooseCopiedFrontierPreferredContinuationPlateId(
				Planet,
				InvolvedPlateIds,
				bCollisionTriangle);
		OutSourcePlateId =
			ChooseTrackedTriangleSourcePlateId(
				Planet,
				InvolvedPlateIds,
				OutPreferredContinuationPlateId);
		return
			OutTrackedKind != EV6CopiedFrontierDestructiveKind::None &&
			OutPreferredContinuationPlateId != INDEX_NONE &&
			OutSourcePlateId != INDEX_NONE &&
			OutSourcePlateId != OutPreferredContinuationPlateId;
	}

	bool DetermineNeighborTrackedTriangleContext(
		const FTectonicPlanet& Planet,
		const int32 TriangleIndex,
		const TArray<uint8>& TrackedKinds,
		const TArray<int32>& TrackedPreferredContinuationPlateIds,
		const TArray<int32>& TrackedSourcePlateIds,
		EV6CopiedFrontierDestructiveKind& OutTrackedKind,
		int32& OutPreferredContinuationPlateId,
		int32& OutSourcePlateId)
	{
		OutTrackedKind = EV6CopiedFrontierDestructiveKind::None;
		OutPreferredContinuationPlateId = INDEX_NONE;
		OutSourcePlateId = INDEX_NONE;
		if (!Planet.TriangleAdjacency.IsValidIndex(TriangleIndex))
		{
			return false;
		}

		int32 SubductionVoteCount = 0;
		int32 CollisionVoteCount = 0;
		TMap<int32, int32> ContinuationVoteCounts;
		TMap<int32, int32> SourceVoteCounts;
		for (const int32 NeighborTriangleIndex : Planet.TriangleAdjacency[TriangleIndex])
		{
			if (!TrackedKinds.IsValidIndex(NeighborTriangleIndex))
			{
				continue;
			}

			const EV6CopiedFrontierDestructiveKind NeighborTrackedKind =
				static_cast<EV6CopiedFrontierDestructiveKind>(TrackedKinds[NeighborTriangleIndex]);
			if (NeighborTrackedKind == EV6CopiedFrontierDestructiveKind::None)
			{
				continue;
			}

			if (NeighborTrackedKind == EV6CopiedFrontierDestructiveKind::Collision)
			{
				++CollisionVoteCount;
			}
			else
			{
				++SubductionVoteCount;
			}

			if (TrackedPreferredContinuationPlateIds.IsValidIndex(NeighborTriangleIndex) &&
				TrackedPreferredContinuationPlateIds[NeighborTriangleIndex] != INDEX_NONE)
			{
				++ContinuationVoteCounts.FindOrAdd(
					TrackedPreferredContinuationPlateIds[NeighborTriangleIndex]);
			}
			if (TrackedSourcePlateIds.IsValidIndex(NeighborTriangleIndex) &&
				TrackedSourcePlateIds[NeighborTriangleIndex] != INDEX_NONE)
			{
				++SourceVoteCounts.FindOrAdd(
					TrackedSourcePlateIds[NeighborTriangleIndex]);
			}
		}

		if (SubductionVoteCount + CollisionVoteCount == 0)
		{
			return false;
		}

		OutTrackedKind =
			CollisionVoteCount > SubductionVoteCount
				? EV6CopiedFrontierDestructiveKind::Collision
				: EV6CopiedFrontierDestructiveKind::Subduction;

		int32 BestContinuationVoteCount = TNumericLimits<int32>::Lowest();
		for (const TPair<int32, int32>& Entry : ContinuationVoteCounts)
		{
			if (Entry.Value > BestContinuationVoteCount ||
				(Entry.Value == BestContinuationVoteCount &&
					(OutPreferredContinuationPlateId == INDEX_NONE || Entry.Key < OutPreferredContinuationPlateId)))
			{
				OutPreferredContinuationPlateId = Entry.Key;
				BestContinuationVoteCount = Entry.Value;
			}
		}

		int32 BestSourceVoteCount = TNumericLimits<int32>::Lowest();
		for (const TPair<int32, int32>& Entry : SourceVoteCounts)
		{
			if (Entry.Value > BestSourceVoteCount ||
				(Entry.Value == BestSourceVoteCount &&
					(OutSourcePlateId == INDEX_NONE || Entry.Key < OutSourcePlateId)))
			{
				OutSourcePlateId = Entry.Key;
				BestSourceVoteCount = Entry.Value;
			}
		}

		TArray<int32> InvolvedPlateIds;
		CollectTrianglePlateIds(Planet, TriangleIndex, InvolvedPlateIds);
		OutPreferredContinuationPlateId =
			OutPreferredContinuationPlateId != INDEX_NONE
				? OutPreferredContinuationPlateId
				: ChooseCopiedFrontierPreferredContinuationPlateId(
					Planet,
					InvolvedPlateIds,
					OutTrackedKind == EV6CopiedFrontierDestructiveKind::Collision);
		OutSourcePlateId =
			ChooseTrackedTriangleSourcePlateId(
				Planet,
				InvolvedPlateIds,
				OutPreferredContinuationPlateId,
				OutSourcePlateId);
		return
			OutPreferredContinuationPlateId != INDEX_NONE &&
			OutSourcePlateId != INDEX_NONE &&
			OutSourcePlateId != OutPreferredContinuationPlateId;
	}

	double ComputeTrackedTriangleAdvanceSpeedKmPerMy(
		const FTectonicPlanet& Planet,
		const int32 TriangleIndex,
		const int32 SourcePlateId)
	{
		const FVector3d QueryPoint =
			ComputeTrackedTriangleBarycenterForPlate(Planet, TriangleIndex, SourcePlateId);
		if (QueryPoint.IsNearlyZero())
		{
			return 0.0;
		}

		const FPlate* SourcePlate = FindPlateById(Planet, SourcePlateId);
		return SourcePlate != nullptr
			? ComputePlateSurfaceVelocity(*SourcePlate, QueryPoint, Planet.PlanetRadiusKm).Length()
			: 0.0;
	}

	void SeedCopiedFrontierIntervalDestructiveTracking(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		TArray<uint8>& OutTrackedKinds,
		TArray<int32>& OutTrackedPreferredContinuationPlateIds,
		TArray<int32>& OutTrackedSourcePlateIds,
		TArray<float>& OutTrackedDistancesKm,
		TArray<uint8>& OutTrackedSeedOriginFlags,
		TArray<uint8>& OutTrackedSeedSurvivalLoggedFlags,
		TArray<uint8>& OutTrackedTopologyNeighborOriginFlags,
		FV6CopiedFrontierDestructiveTrackingUpdateStats& OutStats)
	{
		OutStats = FV6CopiedFrontierDestructiveTrackingUpdateStats{};
		(void)PlateMeshes;
		ResetCopiedFrontierDestructiveTrackingState(
			Planet,
			OutTrackedKinds,
			OutTrackedPreferredContinuationPlateIds,
			OutTrackedSourcePlateIds,
			OutTrackedDistancesKm,
			OutTrackedSeedOriginFlags,
			OutTrackedSeedSurvivalLoggedFlags,
			OutTrackedTopologyNeighborOriginFlags);
		TSet<uint64> TrackedContextConvergentEdgeKeys;
		BuildCopiedFrontierTrackedContextConvergentEdgeSet(Planet, TrackedContextConvergentEdgeKeys);
		TArray<FV6CopiedFrontierActiveCollisionPair> ActiveCollisionPairs;
		BuildCopiedFrontierActiveCollisionPairs(Planet, ActiveCollisionPairs);

		for (int32 GlobalTriangleIndex = 0; GlobalTriangleIndex < Planet.TriangleIndices.Num(); ++GlobalTriangleIndex)
		{
			TArray<int32> InvolvedPlateIds;
			CollectTrianglePlateIds(Planet, GlobalTriangleIndex, InvolvedPlateIds);
			if (InvolvedPlateIds.Num() <= 1)
			{
				continue;
			}

			++OutStats.LifecycleStats.EntryCandidateCount;
			const bool bConvergentEdgeSeedCandidate =
				TriangleContainsStrongConvergentEdge(
					Planet.TriangleIndices[GlobalTriangleIndex],
					TrackedContextConvergentEdgeKeys);
			if (bConvergentEdgeSeedCandidate)
			{
				++OutStats.LifecycleStats.EntrySeedConvergentEdgeCandidateCount;
			}
			EV6CopiedFrontierDestructiveKind TrackedKind = EV6CopiedFrontierDestructiveKind::None;
			int32 PreferredContinuationPlateId = INDEX_NONE;
			int32 SourcePlateId = INDEX_NONE;
			if (!DetermineSeedTrackedTriangleContext(
					Planet,
					TrackedContextConvergentEdgeKeys,
					ActiveCollisionPairs,
					GlobalTriangleIndex,
					TrackedKind,
					PreferredContinuationPlateId,
					SourcePlateId))
			{
				++OutStats.LifecycleStats.EntryRejectedAuthorizationCount;
				continue;
			}

			if (OutTrackedKinds[GlobalTriangleIndex] != static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None))
			{
				++OutStats.LifecycleStats.EntryRejectedAlreadyTrackedCount;
				continue;
			}

			OutTrackedKinds[GlobalTriangleIndex] = static_cast<uint8>(TrackedKind);
			OutTrackedPreferredContinuationPlateIds[GlobalTriangleIndex] = PreferredContinuationPlateId;
			OutTrackedSourcePlateIds[GlobalTriangleIndex] = SourcePlateId;
			OutTrackedDistancesKm[GlobalTriangleIndex] = 0.0f;
			OutTrackedSeedOriginFlags[GlobalTriangleIndex] = bConvergentEdgeSeedCandidate ? 1 : 0;
			OutTrackedSeedSurvivalLoggedFlags[GlobalTriangleIndex] = 0;
			OutTrackedTopologyNeighborOriginFlags[GlobalTriangleIndex] = 0;
			++OutStats.NewlySeededTriangleCount;
			++OutStats.LifecycleStats.EntryAdmittedCount;
			if (bConvergentEdgeSeedCandidate)
			{
				++OutStats.LifecycleStats.EntrySeedConvergentEdgeAdmittedCount;
			}
			if (TrackedKind == EV6CopiedFrontierDestructiveKind::Collision)
			{
				++OutStats.LifecycleStats.EntryAdmittedCollisionCount;
			}
			else if (TrackedKind == EV6CopiedFrontierDestructiveKind::Subduction)
			{
				++OutStats.LifecycleStats.EntryAdmittedSubductionCount;
			}
		}

		AccumulateCopiedFrontierTrackedDestructiveCounts(OutTrackedKinds, OutStats);
	}

	int32 AdvanceCopiedFrontierIntervalDestructiveTracking(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		TArray<uint8>& InOutTrackedKinds,
		TArray<int32>& InOutTrackedPreferredContinuationPlateIds,
		TArray<int32>& InOutTrackedSourcePlateIds,
		TArray<float>& InOutTrackedDistancesKm,
		TArray<uint8>& InOutTrackedSeedOriginFlags,
		TArray<uint8>& InOutTrackedSeedSurvivalLoggedFlags,
		TArray<uint8>& InOutTrackedTopologyNeighborOriginFlags,
		int32& OutExpiredTriangleCount,
		FV6CopiedFrontierDestructiveTrackingUpdateStats& OutWaveStats)
	{
		OutWaveStats = FV6CopiedFrontierDestructiveTrackingUpdateStats{};
		OutExpiredTriangleCount = 0;
		if (Planet.TriangleAdjacency.IsEmpty() || InOutTrackedKinds.IsEmpty())
		{
			return 0;
		}

		TArray<FV6CopiedFrontierTrackingPlateOverlapData> PlateOverlapData;
		BuildCopiedFrontierTrackingPlateOverlapData(Planet, PlateMeshes, PlateOverlapData);

		TArray<uint8> UpdatedTrackedKinds = InOutTrackedKinds;
		TArray<int32> UpdatedTrackedPreferredContinuationPlateIds = InOutTrackedPreferredContinuationPlateIds;
		TArray<int32> UpdatedTrackedSourcePlateIds = InOutTrackedSourcePlateIds;
		TArray<float> UpdatedTrackedDistancesKm = InOutTrackedDistancesKm;
		TArray<uint8> UpdatedTrackedSeedOriginFlags = InOutTrackedSeedOriginFlags;
		TArray<uint8> UpdatedTrackedSeedSurvivalLoggedFlags = InOutTrackedSeedSurvivalLoggedFlags;
		TArray<uint8> UpdatedTrackedTopologyNeighborOriginFlags = InOutTrackedTopologyNeighborOriginFlags;
		const double MaxTrackedDistanceKm = SubductionMaxDistanceRad * Planet.PlanetRadiusKm;
		const double DirectionalInwardDistanceEpsilonKm = 0.5;
		TMap<int32, TMap<uint64, int32>> TopologyNeighborCandidateVoteCounts;
		const auto MakeTopologyNeighborContextKey = [](const int32 SourcePlateId, const int32 PreferredContinuationPlateId) -> uint64
		{
			return
				(static_cast<uint64>(static_cast<uint32>(SourcePlateId)) << 32) |
				static_cast<uint64>(static_cast<uint32>(PreferredContinuationPlateId));
		};

		for (int32 TriangleIndex = 0; TriangleIndex < InOutTrackedKinds.Num(); ++TriangleIndex)
		{
			const EV6CopiedFrontierDestructiveKind TrackedKind =
				static_cast<EV6CopiedFrontierDestructiveKind>(InOutTrackedKinds[TriangleIndex]);
			if (TrackedKind == EV6CopiedFrontierDestructiveKind::None)
			{
				continue;
			}
			++OutWaveStats.LifecycleStats.ActiveAdvanceCount;
			if (InOutTrackedSeedOriginFlags.IsValidIndex(TriangleIndex) &&
				InOutTrackedSeedOriginFlags[TriangleIndex] != 0 &&
				InOutTrackedSeedSurvivalLoggedFlags.IsValidIndex(TriangleIndex) &&
				InOutTrackedSeedSurvivalLoggedFlags[TriangleIndex] == 0)
			{
				++OutWaveStats.LifecycleStats.EntrySeedConvergentEdgeSurvivedOneTimestepCount;
				UpdatedTrackedSeedSurvivalLoggedFlags[TriangleIndex] = 1;
			}

			const int32 PreferredContinuationPlateId =
				InOutTrackedPreferredContinuationPlateIds.IsValidIndex(TriangleIndex)
					? InOutTrackedPreferredContinuationPlateIds[TriangleIndex]
					: INDEX_NONE;
			const int32 SourcePlateId =
				InOutTrackedSourcePlateIds.IsValidIndex(TriangleIndex)
					? InOutTrackedSourcePlateIds[TriangleIndex]
					: INDEX_NONE;
			if (PreferredContinuationPlateId == INDEX_NONE ||
				SourcePlateId == INDEX_NONE ||
				!Planet.TriangleAdjacency.IsValidIndex(TriangleIndex))
			{
				continue;
			}

			const double UpdatedDistanceKm =
				(UpdatedTrackedDistancesKm.IsValidIndex(TriangleIndex)
					? static_cast<double>(UpdatedTrackedDistancesKm[TriangleIndex])
					: 0.0) +
				(ComputeTrackedTriangleAdvanceSpeedKmPerMy(Planet, TriangleIndex, SourcePlateId) * DeltaTimeMyears);
			UpdatedTrackedDistancesKm[TriangleIndex] = static_cast<float>(UpdatedDistanceKm);
			if (UpdatedDistanceKm > MaxTrackedDistanceKm)
			{
				UpdatedTrackedKinds[TriangleIndex] = static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None);
				UpdatedTrackedPreferredContinuationPlateIds[TriangleIndex] = INDEX_NONE;
				UpdatedTrackedSourcePlateIds[TriangleIndex] = INDEX_NONE;
				UpdatedTrackedDistancesKm[TriangleIndex] = 0.0f;
				if (UpdatedTrackedSeedOriginFlags.IsValidIndex(TriangleIndex))
				{
					UpdatedTrackedSeedOriginFlags[TriangleIndex] = 0;
				}
				if (UpdatedTrackedSeedSurvivalLoggedFlags.IsValidIndex(TriangleIndex))
				{
					UpdatedTrackedSeedSurvivalLoggedFlags[TriangleIndex] = 0;
				}
				if (UpdatedTrackedTopologyNeighborOriginFlags.IsValidIndex(TriangleIndex))
				{
					if (UpdatedTrackedTopologyNeighborOriginFlags[TriangleIndex] != 0)
					{
						++OutWaveStats.LifecycleStats.TopologyNeighborExpiredBeforeRemeshCount;
					}
					UpdatedTrackedTopologyNeighborOriginFlags[TriangleIndex] = 0;
				}
				++OutExpiredTriangleCount;
				if (TrackedKind == EV6CopiedFrontierDestructiveKind::Collision)
				{
					++OutWaveStats.LifecycleStats.ExpiredCollisionCount;
				}
				else if (TrackedKind == EV6CopiedFrontierDestructiveKind::Subduction)
				{
					++OutWaveStats.LifecycleStats.ExpiredSubductionCount;
				}
				continue;
			}

			if (!DoesTrackedTriangleBarycenterOverlapPreferredPlate(
					Planet,
					PlateOverlapData,
					TriangleIndex,
					SourcePlateId,
					PreferredContinuationPlateId))
			{
				++OutWaveStats.LifecycleStats.ActiveOverlapRejectedCount;
			}
			else
			{
				++OutWaveStats.LifecycleStats.ActiveOverlapConfirmedCount;
			}

			if (TrackedKind != EV6CopiedFrontierDestructiveKind::Subduction)
			{
				continue;
			}

			double ParentSourcePlateSubductionDistanceKm = -1.0;
			if (!ComputeTrackedTriangleSourcePlateSubductionDistanceKm(
					Planet,
					TriangleIndex,
					SourcePlateId,
					ParentSourcePlateSubductionDistanceKm))
			{
				continue;
			}

			for (const int32 NeighborTriangleIndex : Planet.TriangleAdjacency[TriangleIndex])
			{
				if (!UpdatedTrackedKinds.IsValidIndex(NeighborTriangleIndex))
				{
					continue;
				}

				if (UpdatedTrackedKinds[NeighborTriangleIndex] == 0)
				{
					if (!TriangleContainsPlateId(Planet, NeighborTriangleIndex, SourcePlateId))
					{
						continue;
					}

					++OutWaveStats.LifecycleStats.DirectionalNeighborCandidateConsideredCount;
					double NeighborSourcePlateSubductionDistanceKm = -1.0;
					if (!ComputeTrackedTriangleSourcePlateSubductionDistanceKm(
							Planet,
							NeighborTriangleIndex,
							SourcePlateId,
							NeighborSourcePlateSubductionDistanceKm) ||
						NeighborSourcePlateSubductionDistanceKm <=
							(ParentSourcePlateSubductionDistanceKm + DirectionalInwardDistanceEpsilonKm))
					{
						++OutWaveStats.LifecycleStats.DirectionalNeighborRejectedNotInwardCount;
						continue;
					}

					TMap<uint64, int32>& ContextVoteCounts =
						TopologyNeighborCandidateVoteCounts.FindOrAdd(NeighborTriangleIndex);
					if (ContextVoteCounts.IsEmpty())
					{
						++OutWaveStats.LifecycleStats.NeighborCandidateGeneratedCount;
						++OutWaveStats.LifecycleStats.TopologyNeighborCandidateGeneratedCount;
					}
					++ContextVoteCounts.FindOrAdd(
						MakeTopologyNeighborContextKey(SourcePlateId, PreferredContinuationPlateId));
				}
				else
				{
					++OutWaveStats.LifecycleStats.NeighborRejectedAlreadyTrackedCount;
				}
			}
		}

		int32 PropagatedTriangleCount = 0;
		for (const TPair<int32, TMap<uint64, int32>>& CandidateEntry : TopologyNeighborCandidateVoteCounts)
		{
			const int32 NeighborTriangleIndex = CandidateEntry.Key;
			++OutWaveStats.LifecycleStats.NeighborCandidateTestedCount;
			if (UpdatedTrackedKinds[NeighborTriangleIndex] != static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None))
			{
				++OutWaveStats.LifecycleStats.NeighborRejectedAlreadyTrackedCount;
				continue;
			}

			uint64 BestContextKey = 0;
			int32 BestVoteCount = TNumericLimits<int32>::Lowest();
			for (const TPair<uint64, int32>& VoteEntry : CandidateEntry.Value)
			{
				if (VoteEntry.Value > BestVoteCount ||
					(VoteEntry.Value == BestVoteCount && VoteEntry.Key < BestContextKey))
				{
					BestContextKey = VoteEntry.Key;
					BestVoteCount = VoteEntry.Value;
				}
			}

			const int32 SourcePlateId = static_cast<int32>(static_cast<uint32>(BestContextKey >> 32));
			const int32 PreferredContinuationPlateId =
				static_cast<int32>(static_cast<uint32>(BestContextKey & 0xffffffffull));
			const EV6CopiedFrontierDestructiveKind NeighborTrackedKind =
				EV6CopiedFrontierDestructiveKind::Subduction;

			UpdatedTrackedKinds[NeighborTriangleIndex] = static_cast<uint8>(NeighborTrackedKind);
			UpdatedTrackedPreferredContinuationPlateIds[NeighborTriangleIndex] = PreferredContinuationPlateId;
			UpdatedTrackedSourcePlateIds[NeighborTriangleIndex] = SourcePlateId;
			UpdatedTrackedDistancesKm[NeighborTriangleIndex] = 0.0f;
			if (UpdatedTrackedSeedOriginFlags.IsValidIndex(NeighborTriangleIndex))
			{
				UpdatedTrackedSeedOriginFlags[NeighborTriangleIndex] = 0;
			}
			if (UpdatedTrackedSeedSurvivalLoggedFlags.IsValidIndex(NeighborTriangleIndex))
			{
				UpdatedTrackedSeedSurvivalLoggedFlags[NeighborTriangleIndex] = 0;
			}
			if (UpdatedTrackedTopologyNeighborOriginFlags.IsValidIndex(NeighborTriangleIndex))
			{
				UpdatedTrackedTopologyNeighborOriginFlags[NeighborTriangleIndex] = 1;
			}
			++PropagatedTriangleCount;
			++OutWaveStats.LifecycleStats.NeighborAdmittedCount;
			++OutWaveStats.LifecycleStats.TopologyNeighborAdmittedCount;
			++OutWaveStats.LifecycleStats.DirectionalNeighborAdmittedCount;
			++OutWaveStats.LifecycleStats.NeighborAdmittedSubductionCount;
		}

		InOutTrackedKinds = MoveTemp(UpdatedTrackedKinds);
		InOutTrackedPreferredContinuationPlateIds = MoveTemp(UpdatedTrackedPreferredContinuationPlateIds);
		InOutTrackedSourcePlateIds = MoveTemp(UpdatedTrackedSourcePlateIds);
		InOutTrackedDistancesKm = MoveTemp(UpdatedTrackedDistancesKm);
		InOutTrackedSeedOriginFlags = MoveTemp(UpdatedTrackedSeedOriginFlags);
		InOutTrackedSeedSurvivalLoggedFlags = MoveTemp(UpdatedTrackedSeedSurvivalLoggedFlags);
		InOutTrackedTopologyNeighborOriginFlags = MoveTemp(UpdatedTrackedTopologyNeighborOriginFlags);
		return PropagatedTriangleCount;
	}

	void BuildCopiedFrontierDestructiveFilterState(
		const FTectonicPlanet& Planet,
		const TArray<uint8>& TrackedKinds,
		const TArray<int32>& TrackedPreferredContinuationPlateIds,
		FV6CopiedFrontierDestructiveFilterState& OutFilterState)
	{
		OutFilterState = FV6CopiedFrontierDestructiveFilterState{};
		OutFilterState.GlobalTriangleDestructiveFlags.Init(0, Planet.TriangleIndices.Num());
		OutFilterState.GlobalTriangleDestructiveKinds.Init(
			static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None),
			Planet.TriangleIndices.Num());
		OutFilterState.GlobalTrianglePreferredContinuationPlateIds.Init(INDEX_NONE, Planet.TriangleIndices.Num());
		if (Planet.TriangleIndices.IsEmpty())
		{
			return;
		}

		TSet<uint64> StrongConvergentEdgeKeys;
		BuildCopiedFrontierStrongConvergentEdgeSet(Planet, StrongConvergentEdgeKeys);
		TArray<FV6CopiedFrontierActiveCollisionPair> ActiveCollisionPairs;
		BuildCopiedFrontierActiveCollisionPairs(Planet, ActiveCollisionPairs);

		for (int32 GlobalTriangleIndex = 0; GlobalTriangleIndex < Planet.TriangleIndices.Num(); ++GlobalTriangleIndex)
		{
			if (!Planet.TriangleIndices.IsValidIndex(GlobalTriangleIndex))
			{
				continue;
			}

			const FIntVector& Triangle = Planet.TriangleIndices[GlobalTriangleIndex];
			if (!Planet.Samples.IsValidIndex(Triangle.X) ||
				!Planet.Samples.IsValidIndex(Triangle.Y) ||
				!Planet.Samples.IsValidIndex(Triangle.Z))
			{
				continue;
			}

			TArray<int32> InvolvedPlateIds;
			CollectBoundaryTrianglePlateIds(
				Planet.Samples[Triangle.X].PlateId,
				Planet.Samples[Triangle.Y].PlateId,
				Planet.Samples[Triangle.Z].PlateId,
				InvolvedPlateIds);
			const bool bCopiedFrontierTriangle = InvolvedPlateIds.Num() > 1;

			const uint8 TrackedKind =
				TrackedKinds.IsValidIndex(GlobalTriangleIndex)
					? TrackedKinds[GlobalTriangleIndex]
					: static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None);
			uint8 DestructiveKind = static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None);
			if (TrackedKind != static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None))
			{
				DestructiveKind = TrackedKind;
			}
			else
			{
				if (!bCopiedFrontierTriangle)
				{
					continue;
				}

				bool bCollisionTriangle = false;
				for (const FV6CopiedFrontierActiveCollisionPair& Pair : ActiveCollisionPairs)
				{
					if (TriangleMatchesActiveCollisionPair(Triangle, InvolvedPlateIds, Pair))
					{
						bCollisionTriangle = true;
						break;
					}
				}
				const bool bSubductionTriangle =
					!StrongConvergentEdgeKeys.IsEmpty() &&
					TriangleContainsStrongConvergentEdge(Triangle, StrongConvergentEdgeKeys);
				if (bCollisionTriangle)
				{
					DestructiveKind = static_cast<uint8>(EV6CopiedFrontierDestructiveKind::Collision);
				}
				else if (bSubductionTriangle)
				{
					DestructiveKind = static_cast<uint8>(EV6CopiedFrontierDestructiveKind::Subduction);
				}
			}

			if (DestructiveKind == static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None))
			{
				continue;
			}

			const bool bUseTrackedContinuation =
				TrackedKind != static_cast<uint8>(EV6CopiedFrontierDestructiveKind::None) &&
				TrackedPreferredContinuationPlateIds.IsValidIndex(GlobalTriangleIndex) &&
				TrackedPreferredContinuationPlateIds[GlobalTriangleIndex] != INDEX_NONE;

			OutFilterState.GlobalTriangleDestructiveFlags[GlobalTriangleIndex] = 1;
			OutFilterState.GlobalTriangleDestructiveKinds[GlobalTriangleIndex] = DestructiveKind;
			OutFilterState.GlobalTrianglePreferredContinuationPlateIds[GlobalTriangleIndex] =
				bUseTrackedContinuation
					? TrackedPreferredContinuationPlateIds[GlobalTriangleIndex]
					: ChooseCopiedFrontierPreferredContinuationPlateId(
						Planet,
						InvolvedPlateIds,
						DestructiveKind == static_cast<uint8>(EV6CopiedFrontierDestructiveKind::Collision));
			OutFilterState.bApplied = true;
		}
	}

	bool IsCopiedFrontierTriangleDestructive(
		const FV6CopiedFrontierDestructiveFilterState* FilterState,
		const int32 GlobalTriangleIndex)
	{
		return
			FilterState != nullptr &&
			FilterState->GlobalTriangleDestructiveFlags.IsValidIndex(GlobalTriangleIndex) &&
			FilterState->GlobalTriangleDestructiveFlags[GlobalTriangleIndex] != 0;
	}

	int32 GetCopiedFrontierTrianglePreferredContinuationPlateId(
		const FV6CopiedFrontierDestructiveFilterState* FilterState,
		const int32 GlobalTriangleIndex)
	{
		return
			FilterState != nullptr &&
			FilterState->GlobalTrianglePreferredContinuationPlateIds.IsValidIndex(GlobalTriangleIndex)
				? FilterState->GlobalTrianglePreferredContinuationPlateIds[GlobalTriangleIndex]
				: INDEX_NONE;
	}

	struct FV6CopiedFrontierTectonicMaintenanceStats
	{
		int32 AppliedCount = 0;
		int32 ContinentalRecoveredCount = 0;
		int32 ContinentalGainCount = 0;
		int32 SamePlateRecoveredCount = 0;
		int32 CrossPlateRecoveredCount = 0;
		int32 AndeanTaggedCount = 0;
		int32 ElevationBoostCount = 0;
		int32 ThicknessBoostCount = 0;
	};

	bool IsConvergentMaintenanceActiveZoneCause(const ETectonicPlanetV6ActiveZoneCause Cause)
	{
		return
			Cause == ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction ||
			Cause == ETectonicPlanetV6ActiveZoneCause::CollisionContact;
	}

	enum class EV6ActiveBandFieldContinuityKind : uint8
	{
		None,
		Triangle,
		Synthetic,
		Oceanic,
	};

	void BuildCopiedFrontierConvergentMaintenanceLocality(
		const FTectonicPlanet& Planet,
		const TArray<uint8>& ActiveZoneFlags,
		const TArray<uint8>& ActiveZoneCauseValues,
		TArray<uint8>& OutConvergentActiveFlags,
		TArray<uint8>& OutAdjacentToConvergentActiveFlags)
	{
		const int32 SampleCount = Planet.Samples.Num();
		OutConvergentActiveFlags.Init(0, SampleCount);
		OutAdjacentToConvergentActiveFlags.Init(0, SampleCount);
		if (ActiveZoneFlags.Num() != SampleCount || ActiveZoneCauseValues.Num() != SampleCount)
		{
			return;
		}

		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			if (ActiveZoneFlags[SampleIndex] == 0)
			{
				continue;
			}

			const ETectonicPlanetV6ActiveZoneCause Cause =
				static_cast<ETectonicPlanetV6ActiveZoneCause>(ActiveZoneCauseValues[SampleIndex]);
			if (!IsConvergentMaintenanceActiveZoneCause(Cause))
			{
				continue;
			}

			OutConvergentActiveFlags[SampleIndex] = 1;
			if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!OutAdjacentToConvergentActiveFlags.IsValidIndex(NeighborIndex) ||
					NeighborIndex == SampleIndex)
				{
					continue;
				}

				OutAdjacentToConvergentActiveFlags[NeighborIndex] = 1;
			}
		}
	}

	float BlendAndClampFieldContinuityValue(
		const float PreviousValue,
		const float CurrentValue,
		const float PreserveBlend,
		const float MaxDelta)
	{
		const float BlendedValue = FMath::Lerp(PreviousValue, CurrentValue, PreserveBlend);
		return FMath::Clamp(BlendedValue, PreviousValue - MaxDelta, PreviousValue + MaxDelta);
	}

	bool IsCopiedFrontierConvergentFieldContinuityLocality(
		const TArray<uint8>& ConvergentActiveFlags,
		const TArray<uint8>& AdjacentToConvergentActiveFlags,
		const int32 SampleIndex)
	{
		return
			(ConvergentActiveFlags.IsValidIndex(SampleIndex) &&
				ConvergentActiveFlags[SampleIndex] != 0) ||
			(AdjacentToConvergentActiveFlags.IsValidIndex(SampleIndex) &&
				AdjacentToConvergentActiveFlags[SampleIndex] != 0);
	}

	bool HasCopiedFrontierPreviousOwnerFieldRecoveryContext(
		const FTectonicPlanetV6ResolvedSample& Resolved,
		const TArray<uint8>& PreviousOwnerRecoveryFlags,
		const TArray<uint8>& PreviousOwnerCanonicalVertexInMeshFlags,
		const TArray<uint8>& PreviousOwnerAdjacentTriangleInMeshFlags,
		const int32 SampleIndex)
	{
		if (Resolved.PreviousPlateId == INDEX_NONE)
		{
			return false;
		}

		return
			(PreviousOwnerRecoveryFlags.IsValidIndex(SampleIndex) &&
				PreviousOwnerRecoveryFlags[SampleIndex] != 0) ||
			(PreviousOwnerCanonicalVertexInMeshFlags.IsValidIndex(SampleIndex) &&
				PreviousOwnerCanonicalVertexInMeshFlags[SampleIndex] != 0) ||
			(PreviousOwnerAdjacentTriangleInMeshFlags.IsValidIndex(SampleIndex) &&
				PreviousOwnerAdjacentTriangleInMeshFlags[SampleIndex] != 0);
	}

	bool IsCopiedFrontierRemeshMissResolutionKind(const ETectonicPlanetV6ResolutionKind ResolutionKind)
	{
		return
			ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
			ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion ||
			ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous;
	}

	bool IsStablePreviousDirectHitTransfer(
		const uint8 PreviousTransferSourceKindValue,
		const uint8 PreviousResolutionKindValue,
		const bool bPreviousSynthetic)
	{
		if (bPreviousSynthetic)
		{
			return false;
		}

		const ETectonicPlanetV6TransferSourceKind PreviousSourceKind =
			static_cast<ETectonicPlanetV6TransferSourceKind>(PreviousTransferSourceKindValue);
		const ETectonicPlanetV6ResolutionKind PreviousResolutionKind =
			static_cast<ETectonicPlanetV6ResolutionKind>(PreviousResolutionKindValue);
		return
			PreviousSourceKind == ETectonicPlanetV6TransferSourceKind::Triangle &&
			PreviousResolutionKind != ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery &&
			!IsCopiedFrontierRemeshMissResolutionKind(PreviousResolutionKind) &&
			PreviousResolutionKind != ETectonicPlanetV6ResolutionKind::ThesisRemeshRetainedSyntheticCoverage;
	}

	bool TryChooseActiveBandSyntheticLoopBreakSource(
		const FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const int32 PreviousPlateId,
		const TArray<int32>& PreSolvePlateIds,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const TArray<uint8>& PreviousSyntheticFlags,
		const TArray<uint8>& PreviousTransferSourceKindValues,
		const TArray<uint8>& PreviousResolutionKindValues,
		int32& OutPlateId,
		int32& OutSourceCanonicalSampleIndex,
		int32& OutSupportCount)
	{
		OutPlateId = INDEX_NONE;
		OutSourceCanonicalSampleIndex = INDEX_NONE;
		OutSupportCount = 0;
		if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			return false;
		}

		struct FNeighborSupport
		{
			int32 Count = 0;
			int32 RepresentativeSampleIndex = INDEX_NONE;
			bool bHasRecoveryCandidate = false;
		};

		TMap<int32, FNeighborSupport> SupportByPlate;
		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!PreSolvePlateIds.IsValidIndex(NeighborIndex) ||
				!PreviousSyntheticFlags.IsValidIndex(NeighborIndex) ||
				!PreviousTransferSourceKindValues.IsValidIndex(NeighborIndex) ||
				!PreviousResolutionKindValues.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = PreSolvePlateIds[NeighborIndex];
			const bool bNeighborPreviousSynthetic = PreviousSyntheticFlags[NeighborIndex] != 0;
			if (NeighborPlateId == INDEX_NONE ||
				!IsStablePreviousDirectHitTransfer(
					PreviousTransferSourceKindValues[NeighborIndex],
					PreviousResolutionKindValues[NeighborIndex],
					bNeighborPreviousSynthetic) ||
				FindCarriedSampleForCanonicalVertex(Planet, NeighborPlateId, NeighborIndex) == nullptr)
			{
				continue;
			}

			const FTectonicPlanetV6RecoveryCandidate* NeighborRecoveryCandidate =
				FindBestRecoveryCandidateForPlate(RecoveryCandidates, NeighborPlateId);
			if (NeighborRecoveryCandidate == nullptr &&
				NeighborPlateId != PreviousPlateId)
			{
				continue;
			}

			FNeighborSupport& Support = SupportByPlate.FindOrAdd(NeighborPlateId);
			++Support.Count;
			if (Support.RepresentativeSampleIndex == INDEX_NONE)
			{
				Support.RepresentativeSampleIndex = NeighborIndex;
			}
			Support.bHasRecoveryCandidate |= NeighborRecoveryCandidate != nullptr;
		}

		int32 BestCount = 0;
		int32 SecondBestCount = 0;
		for (const TPair<int32, FNeighborSupport>& Pair : SupportByPlate)
		{
			const int32 PlateId = Pair.Key;
			const FNeighborSupport& Support = Pair.Value;
			if (Support.Count > BestCount ||
				(Support.Count == BestCount &&
					PlateId == PreviousPlateId &&
					OutPlateId != PreviousPlateId) ||
				(Support.Count == BestCount &&
					Support.bHasRecoveryCandidate &&
					OutPlateId != PreviousPlateId &&
					(OutPlateId == INDEX_NONE || !SupportByPlate.FindChecked(OutPlateId).bHasRecoveryCandidate)) ||
				(Support.Count == BestCount &&
					PlateId != PreviousPlateId &&
					PlateId < OutPlateId))
			{
				SecondBestCount = BestCount;
				BestCount = Support.Count;
				OutPlateId = PlateId;
				OutSourceCanonicalSampleIndex = Support.RepresentativeSampleIndex;
				OutSupportCount = Support.Count;
			}
			else if (Support.Count > SecondBestCount)
			{
				SecondBestCount = Support.Count;
			}
		}

		if (OutPlateId == INDEX_NONE || OutSourceCanonicalSampleIndex == INDEX_NONE)
		{
			return false;
		}

		const int32 MinSupport =
			OutPlateId == PreviousPlateId ? 1 : ActiveBandSyntheticLoopBreakMinNeighborSupport;
		if (OutSupportCount < MinSupport)
		{
			return false;
		}
		if (OutPlateId != PreviousPlateId && OutSupportCount <= SecondBestCount)
		{
			return false;
		}
		return true;
	}

	EV6ActiveBandFieldContinuityKind ApplyCopiedFrontierActiveBandFieldContinuity(
		FSample& Sample,
		const FTectonicPlanetV6ResolvedSample& Resolved,
		const int32 SampleIndex,
		const TArray<float>& PreSolveContinentalWeights,
		const TArray<float>& PreSolveElevations,
		const TArray<float>& PreSolveThicknesses,
		const TArray<uint8>& ConvergentActiveFlags,
		const TArray<uint8>& AdjacentToConvergentActiveFlags,
		const TArray<uint8>& PreviousOwnerRecoveryFlags,
		const TArray<uint8>& PreviousOwnerCanonicalVertexInMeshFlags,
		const TArray<uint8>& PreviousOwnerAdjacentTriangleInMeshFlags)
	{
		if (!IsCopiedFrontierConvergentFieldContinuityLocality(
				ConvergentActiveFlags,
				AdjacentToConvergentActiveFlags,
				SampleIndex) ||
			!PreSolveContinentalWeights.IsValidIndex(SampleIndex) ||
			!PreSolveElevations.IsValidIndex(SampleIndex) ||
			!PreSolveThicknesses.IsValidIndex(SampleIndex))
		{
			return EV6ActiveBandFieldContinuityKind::None;
		}

		const float PreviousElevation = PreSolveElevations[SampleIndex];
		if (FMath::Abs(Sample.Elevation - PreviousElevation) <=
			ActiveBandFieldContinuityElevationDeltaThresholdKm)
		{
			return EV6ActiveBandFieldContinuityKind::None;
		}

		const bool bSameOwner =
			Resolved.PreviousPlateId != INDEX_NONE &&
			Resolved.PreviousPlateId == Resolved.FinalPlateId;
		const bool bTriangleTransfer =
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle;
		const bool bSyntheticTransfer =
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::StructuredSynthetic ||
			Resolved.bRetainedSyntheticCoverage;
		const bool bOceanicRecoveryTransfer =
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::OceanicCreation;
		const bool bNeedsPreviousOwnerPlausibility =
			bSyntheticTransfer || bOceanicRecoveryTransfer;
		if (bNeedsPreviousOwnerPlausibility &&
			!HasCopiedFrontierPreviousOwnerFieldRecoveryContext(
				Resolved,
				PreviousOwnerRecoveryFlags,
				PreviousOwnerCanonicalVertexInMeshFlags,
				PreviousOwnerAdjacentTriangleInMeshFlags,
				SampleIndex))
		{
			return EV6ActiveBandFieldContinuityKind::None;
		}

		if (!bTriangleTransfer && !bSyntheticTransfer && !bOceanicRecoveryTransfer)
		{
			return EV6ActiveBandFieldContinuityKind::None;
		}

		const float PreviousContinentalWeight = FMath::Clamp(
			PreSolveContinentalWeights[SampleIndex],
			0.0f,
			1.0f);
		const float PreviousThickness = FMath::Max(0.0f, PreSolveThicknesses[SampleIndex]);
		if (bSameOwner)
		{
			Sample.ContinentalWeight = PreviousContinentalWeight;
			Sample.Elevation = FMath::Clamp(
				PreviousElevation,
				static_cast<float>(TrenchElevationKm),
				static_cast<float>(ElevationCeilingKm));
			Sample.Thickness = PreviousThickness;
			if (bTriangleTransfer)
			{
				return EV6ActiveBandFieldContinuityKind::Triangle;
			}
			return bOceanicRecoveryTransfer
				? EV6ActiveBandFieldContinuityKind::Oceanic
				: EV6ActiveBandFieldContinuityKind::Synthetic;
		}

		const float PreserveBlend = bTriangleTransfer
			? (bSameOwner
				? ActiveBandSameOwnerTriangleContinuityBlend
				: ActiveBandCrossOwnerTriangleContinuityBlend)
			: ActiveBandSyntheticContinuityBlend;
		const float MaxElevationDeltaKm = bTriangleTransfer
			? (bSameOwner
				? ActiveBandSameOwnerTriangleMaxElevationDeltaKm
				: ActiveBandCrossOwnerTriangleMaxElevationDeltaKm)
			: ActiveBandSyntheticMaxElevationDeltaKm;
		const float MaxContinentalWeightDelta = bTriangleTransfer
			? (bSameOwner
				? ActiveBandSameOwnerTriangleMaxContinentalWeightDelta
				: ActiveBandCrossOwnerTriangleMaxContinentalWeightDelta)
			: ActiveBandSyntheticMaxContinentalWeightDelta;
		const float MaxThicknessDeltaKm = bTriangleTransfer
			? (bSameOwner
				? ActiveBandSameOwnerTriangleMaxThicknessDeltaKm
				: ActiveBandCrossOwnerTriangleMaxThicknessDeltaKm)
			: ActiveBandSyntheticMaxThicknessDeltaKm;

		Sample.ContinentalWeight = FMath::Clamp(
			BlendAndClampFieldContinuityValue(
				PreviousContinentalWeight,
				FMath::Clamp(Sample.ContinentalWeight, 0.0f, 1.0f),
				PreserveBlend,
				MaxContinentalWeightDelta),
			0.0f,
			1.0f);
		Sample.Elevation = FMath::Clamp(
			BlendAndClampFieldContinuityValue(
				PreviousElevation,
				Sample.Elevation,
				PreserveBlend,
				MaxElevationDeltaKm),
			static_cast<float>(TrenchElevationKm),
			static_cast<float>(ElevationCeilingKm));
		Sample.Thickness = FMath::Max(
			0.0f,
			BlendAndClampFieldContinuityValue(
				PreviousThickness,
				FMath::Max(0.0f, Sample.Thickness),
				PreserveBlend,
				MaxThicknessDeltaKm));

		if (bTriangleTransfer)
		{
			return EV6ActiveBandFieldContinuityKind::Triangle;
		}
		return bOceanicRecoveryTransfer
			? EV6ActiveBandFieldContinuityKind::Oceanic
			: EV6ActiveBandFieldContinuityKind::Synthetic;
	}

	FV6CopiedFrontierTectonicMaintenanceStats ApplyCopiedFrontierConvergentMaintenance(
		FTectonicPlanet& Planet,
		const TArray<int32>& PreSolvePlateIds,
		const TArray<uint8>& PreSolveContinentalFlags,
		const TArray<float>& PreSolveContinentalWeights,
		const TArray<float>& PreSolveElevations,
		const TArray<float>& PreSolveThicknesses,
		const TArray<uint8>& ActiveZoneFlags,
		const TArray<uint8>& ActiveZoneCauseValues,
		const int32 IntervalSteps,
		const bool bUseLinearSpeedFactor,
		const bool bUseLinearInfluence)
	{
		FV6CopiedFrontierTectonicMaintenanceStats Stats;
		if (Planet.Samples.IsEmpty() || IntervalSteps <= 0)
		{
			return Stats;
		}

		const double IntervalMyears = static_cast<double>(IntervalSteps) * DeltaTimeMyears;
		constexpr double AdjacentConvergentMaintenanceInfluenceThreshold = 0.20;
		constexpr double StrongConvergentMaintenanceInfluenceThreshold = 0.65;
		constexpr double AndeanLocalityInfluenceThreshold = 0.45;
		constexpr double StrongAndeanInfluenceThreshold = 0.80;
		constexpr double AndeanSpeedFactorThreshold = 0.15;
		TArray<uint8> ConvergentActiveFlags;
		TArray<uint8> AdjacentToConvergentActiveFlags;
		BuildCopiedFrontierConvergentMaintenanceLocality(
			Planet,
			ActiveZoneFlags,
			ActiveZoneCauseValues,
			ConvergentActiveFlags,
			AdjacentToConvergentActiveFlags);

		TArray<int32> AffectedSampleIndices;
		AffectedSampleIndices.Reserve(Planet.Samples.Num() / 16);

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			FSample& Sample = Planet.Samples[SampleIndex];
			if (Sample.SubductionDistanceKm < 0.0f)
			{
				continue;
			}

			double SampleSubductionSpeed = 0.0;
			const int32 PlateArrayIndex = Planet.FindPlateArrayIndexById(Sample.PlateId);
			if (Planet.Plates.IsValidIndex(PlateArrayIndex))
			{
				const FPlate& Plate = Planet.Plates[PlateArrayIndex];
				if (const int32* CarriedIndexPtr = Plate.CanonicalToCarriedIndex.Find(SampleIndex))
				{
					if (Plate.CarriedSamples.IsValidIndex(*CarriedIndexPtr))
					{
						SampleSubductionSpeed = static_cast<double>(Plate.CarriedSamples[*CarriedIndexPtr].SubductionSpeed);
					}
				}
			}
			if (SampleSubductionSpeed <= 0.0)
			{
				continue;
			}

			const double DistanceRad =
				static_cast<double>(Sample.SubductionDistanceKm) /
				FMath::Max(Planet.PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);
			const double Influence = FTectonicPlanet::SubductionDistanceTransfer(
				DistanceRad,
				SubductionControlDistanceRad,
				SubductionMaxDistanceRad);
			if (Influence <= UE_DOUBLE_SMALL_NUMBER)
			{
				continue;
			}

			const bool bActiveConvergentSample =
				ConvergentActiveFlags.IsValidIndex(SampleIndex) &&
				ConvergentActiveFlags[SampleIndex] != 0;
			const bool bAdjacentToActiveConvergentSample =
				AdjacentToConvergentActiveFlags.IsValidIndex(SampleIndex) &&
				AdjacentToConvergentActiveFlags[SampleIndex] != 0;
			const bool bStrongFrontProximity =
				Influence >= StrongConvergentMaintenanceInfluenceThreshold;
			if (!bActiveConvergentSample &&
				!(bAdjacentToActiveConvergentSample &&
					Influence >= AdjacentConvergentMaintenanceInfluenceThreshold) &&
				!bStrongFrontProximity)
			{
				continue;
			}

			const double InfluenceStrength = FMath::Clamp(Influence, 0.0, 1.0);
			const double SpeedRatio = FMath::Clamp(
				SampleSubductionSpeed / MaxPlateSpeedKmPerMy,
				0.0,
				1.0);
			const double SpeedFactor = bUseLinearSpeedFactor
				? SpeedRatio
				: FMath::Sqrt(SpeedRatio);
			if (SpeedFactor <= UE_DOUBLE_SMALL_NUMBER)
			{
				continue;
			}

			const bool bWasContinental =
				PreSolveContinentalFlags.IsValidIndex(SampleIndex) &&
				PreSolveContinentalFlags[SampleIndex] != 0;
			const bool bWasContinentalBeforeProcess = Sample.ContinentalWeight >= 0.5f;
			const float PreviousContinentalWeight =
				PreSolveContinentalWeights.IsValidIndex(SampleIndex)
					? FMath::Clamp(PreSolveContinentalWeights[SampleIndex], 0.0f, 1.0f)
					: 0.0f;
			const float PreviousElevation =
				PreSolveElevations.IsValidIndex(SampleIndex) ? PreSolveElevations[SampleIndex] : 0.0f;
			const float PreviousThickness =
				PreSolveThicknesses.IsValidIndex(SampleIndex) ? PreSolveThicknesses[SampleIndex] : 0.0f;
			const float CurrentContinentalWeight = FMath::Clamp(Sample.ContinentalWeight, 0.0f, 1.0f);
			const bool bStrongAndeanLocality =
				(bActiveConvergentSample && SpeedFactor >= AndeanSpeedFactorThreshold) ||
				(bAdjacentToActiveConvergentSample &&
					Influence >= AndeanLocalityInfluenceThreshold &&
					SpeedFactor >= AndeanSpeedFactorThreshold) ||
				(Influence >= StrongAndeanInfluenceThreshold &&
					SpeedFactor >= AndeanSpeedFactorThreshold);

			double TargetContinentalWeight = CurrentContinentalWeight;
			if (Planet.bEnableAndeanContinentalConversion && bStrongAndeanLocality)
			{
				TargetContinentalWeight = FMath::Max(
					TargetContinentalWeight,
					static_cast<double>(CurrentContinentalWeight) +
						(Planet.AndeanContinentalConversionRatePerMy * IntervalMyears * Influence * SpeedFactor));
			}
			if (bWasContinental)
			{
				TargetContinentalWeight = FMath::Max(
					TargetContinentalWeight,
					static_cast<double>(PreviousContinentalWeight) * InfluenceStrength);
			}

			TargetContinentalWeight = FMath::Clamp(TargetContinentalWeight, 0.0, 1.0);
			const float NewContinentalWeight = static_cast<float>(TargetContinentalWeight);
			const bool bBoostedContinentalWeight =
				NewContinentalWeight > CurrentContinentalWeight + UE_KINDA_SMALL_NUMBER;
			if (!bBoostedContinentalWeight)
			{
				continue;
			}

			Sample.ContinentalWeight = NewContinentalWeight;
			const double OrogenicStrength = bUseLinearInfluence
				? InfluenceStrength
				: FMath::Sqrt(InfluenceStrength);
			const float TargetElevation =
				bWasContinental
					? FMath::Lerp(Sample.Elevation, FMath::Max(PreviousElevation, 0.0f), static_cast<float>(OrogenicStrength))
					: FMath::Lerp(Sample.Elevation, 0.25f, static_cast<float>(OrogenicStrength));
			const float TargetThickness =
				bWasContinental
					? FMath::Lerp(Sample.Thickness, FMath::Max(PreviousThickness, static_cast<float>(OceanicThicknessKm)), static_cast<float>(OrogenicStrength))
					: FMath::Lerp(
						Sample.Thickness,
						static_cast<float>(OceanicThicknessKm + ((ContinentalThicknessKm - OceanicThicknessKm) * OrogenicStrength)),
						static_cast<float>(OrogenicStrength));
			const float PreviousAppliedElevation = Sample.Elevation;
			const float PreviousAppliedThickness = Sample.Thickness;
			Sample.Elevation = FMath::Clamp(
				FMath::Max(Sample.Elevation, TargetElevation),
				static_cast<float>(TrenchElevationKm),
				static_cast<float>(ElevationCeilingKm));
			Sample.Thickness = FMath::Max(Sample.Thickness, TargetThickness);
			if (Sample.OrogenyType == EOrogenyType::None && bStrongAndeanLocality)
			{
				Sample.OrogenyType = EOrogenyType::Andean;
				++Stats.AndeanTaggedCount;
			}

			const bool bIsContinentalAfterProcess = Sample.ContinentalWeight >= 0.5f;
			++Stats.AppliedCount;
			Stats.ContinentalRecoveredCount +=
				(bWasContinental && !bWasContinentalBeforeProcess && bIsContinentalAfterProcess) ? 1 : 0;
			Stats.ContinentalGainCount +=
				(!bWasContinental && !bWasContinentalBeforeProcess && bIsContinentalAfterProcess) ? 1 : 0;
			Stats.SamePlateRecoveredCount +=
				(bWasContinental &&
					!bWasContinentalBeforeProcess &&
					bIsContinentalAfterProcess &&
					PreSolvePlateIds.IsValidIndex(SampleIndex) &&
					PreSolvePlateIds[SampleIndex] == Sample.PlateId)
					? 1
					: 0;
			Stats.CrossPlateRecoveredCount +=
				(bWasContinental &&
					!bWasContinentalBeforeProcess &&
					bIsContinentalAfterProcess &&
					PreSolvePlateIds.IsValidIndex(SampleIndex) &&
					PreSolvePlateIds[SampleIndex] != INDEX_NONE &&
					PreSolvePlateIds[SampleIndex] != Sample.PlateId)
					? 1
					: 0;
			Stats.ElevationBoostCount += Sample.Elevation > PreviousAppliedElevation + UE_KINDA_SMALL_NUMBER ? 1 : 0;
			Stats.ThicknessBoostCount += Sample.Thickness > PreviousAppliedThickness + UE_KINDA_SMALL_NUMBER ? 1 : 0;
			AffectedSampleIndices.Add(SampleIndex);
		}

		SyncV6CarriedSamplesFromCanonicalIndices(Planet, AffectedSampleIndices);
		return Stats;
	}

	FCarriedSample BuildRotatedLocalCarriedSample(const FPlate& Plate, const FCarriedSample& LocalSample)
	{
		FCarriedSample RotatedSample = LocalSample;
		RotatedSample.RidgeDirection = Plate.CumulativeRotation.RotateVector(LocalSample.RidgeDirection);
		RotatedSample.FoldDirection = Plate.CumulativeRotation.RotateVector(LocalSample.FoldDirection);
		return RotatedSample;
	}

	uint64 MakeUndirectedLocalEdgeKey(const int32 VertexA, const int32 VertexB)
	{
		const uint32 MinVertex = static_cast<uint32>(FMath::Min(VertexA, VertexB));
		const uint32 MaxVertex = static_cast<uint32>(FMath::Max(VertexA, VertexB));
		return (static_cast<uint64>(MinVertex) << 32) | static_cast<uint64>(MaxVertex);
	}

	void BuildV6CopiedFrontierFrontierPointSets(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		TArray<FV6PlateFrontierPointSet>& OutFrontierPointSets)
	{
		OutFrontierPointSets.SetNum(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			FV6PlateFrontierPointSet& FrontierPointSet = OutFrontierPointSets[PlateIndex];
			FrontierPointSet = FV6PlateFrontierPointSet{};
			FrontierPointSet.PlateId = Planet.Plates[PlateIndex].Id;

			if (!PlateMeshes.IsValidIndex(PlateIndex) || !QueryGeometries.IsValidIndex(PlateIndex))
			{
				continue;
			}

			const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = PlateMeshes[PlateIndex];
			const FV6PlateQueryGeometry& QueryGeometry = QueryGeometries[PlateIndex];
			if (QueryGeometry.SoupData.LocalTriangles.IsEmpty())
			{
				continue;
			}

			TMap<uint64, int32> EdgeUseCounts;
			EdgeUseCounts.Reserve(QueryGeometry.SoupData.LocalTriangles.Num() * 3);
			for (const UE::Geometry::FIndex3i& LocalTriangle : QueryGeometry.SoupData.LocalTriangles)
			{
				++EdgeUseCounts.FindOrAdd(MakeUndirectedLocalEdgeKey(LocalTriangle.A, LocalTriangle.B));
				++EdgeUseCounts.FindOrAdd(MakeUndirectedLocalEdgeKey(LocalTriangle.B, LocalTriangle.C));
				++EdgeUseCounts.FindOrAdd(MakeUndirectedLocalEdgeKey(LocalTriangle.C, LocalTriangle.A));
			}

			TArray<uint8> FrontierVertexFlags;
			FrontierVertexFlags.Init(0, QueryGeometry.SoupData.RotatedVertices.Num());
			for (const TPair<uint64, int32>& EdgeUsePair : EdgeUseCounts)
			{
				if (EdgeUsePair.Value != 1)
				{
					continue;
				}

				const int32 VertexA = static_cast<int32>(EdgeUsePair.Key >> 32);
				const int32 VertexB = static_cast<int32>(EdgeUsePair.Key & 0xffffffff);
				if (FrontierVertexFlags.IsValidIndex(VertexA))
				{
					FrontierVertexFlags[VertexA] = 1;
				}
				if (FrontierVertexFlags.IsValidIndex(VertexB))
				{
					FrontierVertexFlags[VertexB] = 1;
				}
			}

			FrontierPointSet.Points.Reserve(FrontierVertexFlags.Num());
			for (int32 LocalVertexIndex = 0; LocalVertexIndex < FrontierVertexFlags.Num(); ++LocalVertexIndex)
			{
				if (FrontierVertexFlags[LocalVertexIndex] == 0 ||
					!QueryGeometry.SoupData.RotatedVertices.IsValidIndex(LocalVertexIndex) ||
					!Mesh.LocalCarriedSamples.IsValidIndex(LocalVertexIndex))
				{
					continue;
				}

				FV6FrontierPoint& FrontierPoint = FrontierPointSet.Points.AddDefaulted_GetRef();
				FrontierPoint.PlateId = QueryGeometry.PlateId;
				FrontierPoint.LocalVertexIndex = LocalVertexIndex;
				FrontierPoint.Position = QueryGeometry.SoupData.RotatedVertices[LocalVertexIndex];
				FrontierPoint.CarriedSample =
					BuildRotatedLocalCarriedSample(Planet.Plates[PlateIndex], Mesh.LocalCarriedSamples[LocalVertexIndex]);
			}
		}
	}

	void BuildV6CopiedFrontierQueryStageSetup(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& SolveCopiedFrontierMeshes,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>* UnfilteredComparisonMeshes,
		const FV6CopiedFrontierDestructiveFilterState* DestructiveFilterState,
		const bool bApplyDestructiveFilter,
		const bool bEnableStructuredGapFill,
		const bool bRecordPhaseTiming,
		FTectonicPlanetV6PhaseTiming& InOutPhaseTiming,
		FV6CopiedFrontierQueryStageSetup& OutSetup)
	{
		// Runtime resolve inputs and their geometry-facing diagnostics travel together here so
		// the copied-frontier solve can treat query/frontier assembly as one explicit setup stage.
		OutSetup = FV6CopiedFrontierQueryStageSetup{};
		for (const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh : SolveCopiedFrontierMeshes)
		{
			OutSetup.GeometryCounts.PlateLocalVertexCount += Mesh.BaseVertices.Num();
			OutSetup.GeometryCounts.PlateLocalTriangleCount += Mesh.LocalTriangles.Num();
			OutSetup.GeometryCounts.CopiedFrontierVertexCount += Mesh.CopiedFrontierVertexCount;
			OutSetup.GeometryCounts.CopiedFrontierTriangleCount += Mesh.CopiedFrontierTriangleCount;
			OutSetup.GeometryCounts.CopiedFrontierCarriedSampleCount += Mesh.CopiedFrontierCarriedSampleCount;
		}

		const double QueryGeometryBuildStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		BuildV6CopiedFrontierQueryGeometries(
			Planet,
			SolveCopiedFrontierMeshes,
			OutSetup.QueryGeometries,
			bApplyDestructiveFilter ? DestructiveFilterState : nullptr);
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMs(InOutPhaseTiming.QueryGeometryBuildMs, QueryGeometryBuildStartTime);
		}

		if (bEnableStructuredGapFill)
		{
			const double FrontierPointSetBuildStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
			BuildV6CopiedFrontierFrontierPointSets(
				Planet,
				SolveCopiedFrontierMeshes,
				OutSetup.QueryGeometries,
				OutSetup.FrontierPointSets);
			if (bRecordPhaseTiming)
			{
				AccumulatePhaseTimingMs(InOutPhaseTiming.FrontierPointSetBuildMs, FrontierPointSetBuildStartTime);
			}
		}

		if (bApplyDestructiveFilter && UnfilteredComparisonMeshes != nullptr)
		{
			const double UnfilteredQueryGeometryBuildStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
			BuildV6CopiedFrontierQueryGeometries(
				Planet,
				*UnfilteredComparisonMeshes,
				OutSetup.UnfilteredComparisonQueryGeometries,
				nullptr);
			if (bRecordPhaseTiming)
			{
				AccumulatePhaseTimingMs(
					InOutPhaseTiming.QueryGeometryBuildMs,
					UnfilteredQueryGeometryBuildStartTime);
			}
		}

		OutSetup.QueryGeometryIndexByPlateId.Reserve(OutSetup.QueryGeometries.Num());
		for (int32 QueryGeometryIndex = 0; QueryGeometryIndex < OutSetup.QueryGeometries.Num(); ++QueryGeometryIndex)
		{
			OutSetup.QueryGeometryIndexByPlateId.Add(
				OutSetup.QueryGeometries[QueryGeometryIndex].PlateId,
				QueryGeometryIndex);
		}

		if (bApplyDestructiveFilter)
		{
			OutSetup.UnfilteredComparisonQueryGeometryIndexByPlateId.Reserve(
				OutSetup.UnfilteredComparisonQueryGeometries.Num());
			for (int32 QueryGeometryIndex = 0;
				QueryGeometryIndex < OutSetup.UnfilteredComparisonQueryGeometries.Num();
				++QueryGeometryIndex)
			{
				OutSetup.UnfilteredComparisonQueryGeometryIndexByPlateId.Add(
					OutSetup.UnfilteredComparisonQueryGeometries[QueryGeometryIndex].PlateId,
					QueryGeometryIndex);
			}
		}
	}

	struct FRecoveredContainmentHit
	{
		int32 PlateId = INDEX_NONE;
		int32 LocalTriangleIndex = INDEX_NONE;
		int32 GlobalTriangleIndex = INDEX_NONE;
		FVector3d Barycentric = FVector3d(-1.0, -1.0, -1.0);
		double Distance = TNumericLimits<double>::Max();
	};

	struct FV6ThesisRemeshRayHit
	{
		int32 PlateId = INDEX_NONE;
		int32 LocalTriangleIndex = INDEX_NONE;
		int32 GlobalTriangleIndex = INDEX_NONE;
		FVector3d Barycentric = FVector3d(-1.0, -1.0, -1.0);
		double RayParameter = TNumericLimits<double>::Max();
		double FitScore = -TNumericLimits<double>::Max();
		bool bCopiedFrontierTriangle = false;
		bool bDestructiveTriangle = false;
	};

	int32 CountDistinctHitCandidatePlates(const TArray<FV6ThesisRemeshRayHit>& HitCandidates)
	{
		TSet<int32> UniquePlateIds;
		for (const FV6ThesisRemeshRayHit& HitCandidate : HitCandidates)
		{
			if (HitCandidate.PlateId != INDEX_NONE)
			{
				UniquePlateIds.Add(HitCandidate.PlateId);
			}
		}
		return UniquePlateIds.Num();
	}

	bool HasHitCandidateForPlate(
		const TArray<FV6ThesisRemeshRayHit>& HitCandidates,
		const int32 PlateId)
	{
		if (PlateId == INDEX_NONE)
		{
			return false;
		}

		for (const FV6ThesisRemeshRayHit& HitCandidate : HitCandidates)
		{
			if (HitCandidate.PlateId == PlateId)
			{
				return true;
			}
		}

		return false;
	}

	const FV6ThesisRemeshRayHit* FindBestHitCandidateForPlate(
		const TArray<FV6ThesisRemeshRayHit>& HitCandidates,
		const int32 PlateId)
	{
		if (PlateId == INDEX_NONE)
		{
			return nullptr;
		}

		const FV6ThesisRemeshRayHit* BestHit = nullptr;
		for (const FV6ThesisRemeshRayHit& HitCandidate : HitCandidates)
		{
			if (HitCandidate.PlateId != PlateId)
			{
				continue;
			}

			const bool bBetterThanBest =
				BestHit == nullptr ||
				(HitCandidate.RayParameter + TriangleEpsilon < BestHit->RayParameter) ||
				(FMath::IsNearlyEqual(HitCandidate.RayParameter, BestHit->RayParameter, TriangleEpsilon) &&
					(HitCandidate.FitScore > BestHit->FitScore + TriangleEpsilon ||
						(FMath::IsNearlyEqual(HitCandidate.FitScore, BestHit->FitScore, TriangleEpsilon) &&
							HitCandidate.PlateId < BestHit->PlateId)));
			if (bBetterThanBest)
			{
				BestHit = &HitCandidate;
			}
		}

		return BestHit;
	}

	bool DoesQueryPointPassBoundingCap(const FV6PlateQueryGeometry& QueryGeometry, const FVector3d& QueryDirection)
	{
		return QueryGeometry.BoundingCap.Center.IsNearlyZero() ||
			QueryDirection.Dot(QueryGeometry.BoundingCap.Center) >= QueryGeometry.BoundingCap.CosAngle;
	}

	double ComputeTriangleRayParameter(
		const FVector3d& A,
		const FVector3d& B,
		const FVector3d& C,
		const FVector3d& RayDirection)
	{
		const FVector3d Normal = UE::Geometry::VectorUtil::Normal(A, B, C);
		const double RayDot = Normal.Dot(RayDirection);
		if (FMath::Abs(RayDot) <= UE_DOUBLE_SMALL_NUMBER)
		{
			return RayDirection.Length();
		}

		const double RayParameter = Normal.Dot(A) / RayDot;
		return FMath::IsFinite(RayParameter) && RayParameter > 0.0
			? RayParameter
			: RayDirection.Length();
	}

	void PopulateThesisRemeshRayHitFromLocalTriangle(
		const FV6PlateQueryGeometry& QueryGeometry,
		const int32 LocalTriangleIndex,
		const FVector3d& Barycentric,
		const double RayParameter,
		FV6ThesisRemeshRayHit& OutHit)
	{
		OutHit.PlateId = QueryGeometry.PlateId;
		OutHit.LocalTriangleIndex = LocalTriangleIndex;
		OutHit.GlobalTriangleIndex = QueryGeometry.SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleIndex)
			? QueryGeometry.SoupData.GlobalTriangleIndices[LocalTriangleIndex]
			: INDEX_NONE;
		OutHit.Barycentric = Barycentric;
		OutHit.RayParameter = RayParameter;
		OutHit.FitScore = ComputeContainmentScore(Barycentric);
		OutHit.bCopiedFrontierTriangle = QueryGeometry.LocalTriangleCopiedFrontierFlags.IsValidIndex(LocalTriangleIndex) &&
			QueryGeometry.LocalTriangleCopiedFrontierFlags[LocalTriangleIndex] != 0;
		OutHit.bDestructiveTriangle = QueryGeometry.LocalTriangleDestructiveFlags.IsValidIndex(LocalTriangleIndex) &&
			QueryGeometry.LocalTriangleDestructiveFlags[LocalTriangleIndex] != 0;
	}

	void ClassifyCopiedFrontierMultiHitCandidateMix(
		const TArray<FV6ThesisRemeshRayHit>& HitCandidates,
		const int32 PreviousPlateId,
		const TArray<uint8>& TrackedDestructiveKinds,
		bool& bOutHasTrackedCandidate,
		EV6CopiedFrontierMultiHitPlateMix& OutPlateMix,
		EV6CopiedFrontierMultiHitGeometryMix& OutGeometryMix)
	{
		bOutHasTrackedCandidate = false;
		bool bHasSamePlateCandidate = false;
		bool bHasCrossPlateCandidate = false;
		bool bHasCopiedFrontierCandidate = false;
		bool bHasInteriorCandidate = false;

		for (const FV6ThesisRemeshRayHit& HitCandidate : HitCandidates)
		{
			bOutHasTrackedCandidate |=
				IsTrackedDestructiveTriangle(TrackedDestructiveKinds, HitCandidate.GlobalTriangleIndex);
			bHasSamePlateCandidate |= HitCandidate.PlateId == PreviousPlateId;
			bHasCrossPlateCandidate |= HitCandidate.PlateId != PreviousPlateId;
			bHasCopiedFrontierCandidate |= HitCandidate.bCopiedFrontierTriangle;
			bHasInteriorCandidate |= !HitCandidate.bCopiedFrontierTriangle;
		}

		if (bHasSamePlateCandidate && bHasCrossPlateCandidate)
		{
			OutPlateMix = EV6CopiedFrontierMultiHitPlateMix::Mixed;
		}
		else if (bHasSamePlateCandidate)
		{
			OutPlateMix = EV6CopiedFrontierMultiHitPlateMix::SameOnly;
		}
		else if (bHasCrossPlateCandidate)
		{
			OutPlateMix = EV6CopiedFrontierMultiHitPlateMix::CrossOnly;
		}
		else
		{
			OutPlateMix = EV6CopiedFrontierMultiHitPlateMix::None;
		}

		if (bHasCopiedFrontierCandidate && bHasInteriorCandidate)
		{
			OutGeometryMix = EV6CopiedFrontierMultiHitGeometryMix::Mixed;
		}
		else if (bHasCopiedFrontierCandidate)
		{
			OutGeometryMix = EV6CopiedFrontierMultiHitGeometryMix::CopiedFrontierOnly;
		}
		else if (bHasInteriorCandidate)
		{
			OutGeometryMix = EV6CopiedFrontierMultiHitGeometryMix::InteriorOnly;
		}
		else
		{
			OutGeometryMix = EV6CopiedFrontierMultiHitGeometryMix::None;
		}
	}

	bool TryFindThesisRemeshRawRayHitNoCap(
		const FV6PlateQueryGeometry& QueryGeometry,
		const FVector3d& RayDirection,
		FV6ThesisRemeshRayHit& OutHit)
	{
		OutHit = FV6ThesisRemeshRayHit{};
		if (QueryGeometry.SoupData.LocalTriangles.IsEmpty() || RayDirection.IsNearlyZero())
		{
			return false;
		}

		double RayParameter = TNumericLimits<double>::Max();
		int32 LocalTriangleIndex = INDEX_NONE;
		FVector3d RawBarycentric = FVector3d(-1.0, -1.0, -1.0);
		const FRay3d Ray(FVector3d::ZeroVector, RayDirection);
		if (!QueryGeometry.SoupBVH.FindNearestHitTriangle(Ray, RayParameter, LocalTriangleIndex, RawBarycentric) ||
			!QueryGeometry.SoupData.LocalTriangles.IsValidIndex(LocalTriangleIndex))
		{
			return false;
		}

		PopulateThesisRemeshRayHitFromLocalTriangle(
			QueryGeometry,
			LocalTriangleIndex,
			NormalizeBarycentric(RawBarycentric),
			RayParameter,
			OutHit);
		return true;
	}

	bool TryFindThesisRemeshContainingHitNoCap(
		const FV6PlateQueryGeometry& QueryGeometry,
		const FVector3d& QueryPoint,
		FV6ThesisRemeshRayHit& OutHit)
	{
		OutHit = FV6ThesisRemeshRayHit{};
		if (QueryGeometry.SoupData.LocalTriangles.IsEmpty())
		{
			return false;
		}

		int32 LocalTriangleIndex = INDEX_NONE;
		FVector3d A;
		FVector3d B;
		FVector3d C;
		if (!FindContainingTriangleInBVH(QueryGeometry.SoupBVH, QueryGeometry.SoupAdapter, QueryPoint, LocalTriangleIndex, A, B, C) ||
			!QueryGeometry.SoupData.LocalTriangles.IsValidIndex(LocalTriangleIndex))
		{
			return false;
		}

		const FVector3d Barycentric = NormalizeBarycentric(ComputePlanarBarycentric(A, B, C, QueryPoint));
		PopulateThesisRemeshRayHitFromLocalTriangle(
			QueryGeometry,
			LocalTriangleIndex,
			Barycentric,
			ComputeTriangleRayParameter(A, B, C, QueryPoint.GetSafeNormal()),
			OutHit);
		return true;
	}

	bool TryFindNearestTriangleRecoveryHit(
		const FV6PlateQueryGeometry& QueryGeometry,
		const FVector3d& QueryPoint,
		FRecoveredContainmentHit& OutHit)
	{
		OutHit = FRecoveredContainmentHit{};
		if (QueryGeometry.SoupBVH.RootIndex < 0 || QueryGeometry.SoupData.LocalTriangles.IsEmpty())
		{
			return false;
		}

		double DistanceSqr = TNumericLimits<double>::Max();
		const int32 LocalTriangleId = QueryGeometry.SoupBVH.FindNearestTriangle(QueryPoint, DistanceSqr);
		if (!QueryGeometry.SoupData.LocalTriangles.IsValidIndex(LocalTriangleId) ||
			LocalTriangleId == INDEX_NONE)
		{
			return false;
		}

		FVector3d A;
		FVector3d B;
		FVector3d C;
		QueryGeometry.SoupAdapter.GetTriVertices(LocalTriangleId, A, B, C);
		const UE::Geometry::TTriangle3<double> Triangle(A, B, C);
		UE::Geometry::FDistPoint3Triangle3d DistanceQuery(QueryPoint, Triangle);
		DistanceQuery.ComputeResult();

		OutHit.PlateId = QueryGeometry.PlateId;
		OutHit.LocalTriangleIndex = LocalTriangleId;
		OutHit.GlobalTriangleIndex = QueryGeometry.SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId)
			? QueryGeometry.SoupData.GlobalTriangleIndices[LocalTriangleId]
			: INDEX_NONE;
		OutHit.Barycentric = ClampAndNormalizeBarycentric(FVector3d(
			DistanceQuery.TriangleBaryCoords.X,
			DistanceQuery.TriangleBaryCoords.Y,
			DistanceQuery.TriangleBaryCoords.Z));
		OutHit.Distance = FMath::Sqrt(FMath::Max(DistanceSqr, 0.0));
		return true;
	}

	bool TryFindThesisRemeshRayHit(
		const FV6PlateQueryGeometry& QueryGeometry,
		const FVector3d& QueryPoint,
		FV6ThesisRemeshRayHit& OutHit,
		int32* OutRejectedDestructiveHitCount = nullptr);

	bool TryFindThesisRemeshRayHitForPlate(
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const TMap<int32, int32>& QueryGeometryIndexByPlateId,
		const int32 PlateId,
		const FVector3d& QueryPoint,
		FV6ThesisRemeshRayHit& OutHit,
		int32* OutRejectedDestructiveHitCount = nullptr)
	{
		OutHit = FV6ThesisRemeshRayHit{};
		const int32* QueryGeometryIndexPtr = QueryGeometryIndexByPlateId.Find(PlateId);
		if (QueryGeometryIndexPtr == nullptr || !QueryGeometries.IsValidIndex(*QueryGeometryIndexPtr))
		{
			return false;
		}

		return TryFindThesisRemeshRayHit(
			QueryGeometries[*QueryGeometryIndexPtr],
			QueryPoint,
			OutHit,
			OutRejectedDestructiveHitCount);
	}

	bool TryFindThesisRemeshRecoveryCandidateForPlate(
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const TMap<int32, int32>& QueryGeometryIndexByPlateId,
		const int32 PlateId,
		const FVector3d& QueryPoint,
		FTectonicPlanetV6RecoveryCandidate& OutCandidate)
	{
		OutCandidate = FTectonicPlanetV6RecoveryCandidate{};
		const int32* QueryGeometryIndexPtr = QueryGeometryIndexByPlateId.Find(PlateId);
		if (QueryGeometryIndexPtr == nullptr || !QueryGeometries.IsValidIndex(*QueryGeometryIndexPtr))
		{
			return false;
		}

		FRecoveredContainmentHit RecoveryHit;
		if (!TryFindNearestTriangleRecoveryHit(QueryGeometries[*QueryGeometryIndexPtr], QueryPoint, RecoveryHit))
		{
			return false;
		}

		OutCandidate.PlateId = RecoveryHit.PlateId;
		OutCandidate.TriangleIndex = RecoveryHit.GlobalTriangleIndex;
		OutCandidate.LocalTriangleIndex = RecoveryHit.LocalTriangleIndex;
		OutCandidate.Barycentric = RecoveryHit.Barycentric;
		OutCandidate.DistanceRadians = RecoveryHit.Distance;
		return true;
	}

	template <typename CandidateAllocatorType>
	void CollectThesisRemeshRayHitCandidates(
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const TArray<int32, CandidateAllocatorType>* CandidateQueryGeometryIndices,
		const FVector3d& QueryPoint,
		TArray<FV6ThesisRemeshRayHit>& OutHitCandidates,
		int32* OutRejectedDestructiveHitCount = nullptr,
		int32* OutTestedPlateCount = nullptr)
	{
		OutHitCandidates.Reset();
		const int32 ReserveCount =
			CandidateQueryGeometryIndices != nullptr
				? CandidateQueryGeometryIndices->Num()
				: QueryGeometries.Num();
		OutHitCandidates.Reserve(ReserveCount);

		int32 LocalRejectedDestructiveHitCount = 0;
		int32 LocalTestedPlateCount = 0;
		if (CandidateQueryGeometryIndices != nullptr)
		{
			for (const int32 QueryGeometryIndex : *CandidateQueryGeometryIndices)
			{
				if (!QueryGeometries.IsValidIndex(QueryGeometryIndex))
				{
					continue;
				}

				++LocalTestedPlateCount;
				FV6ThesisRemeshRayHit Hit;
				if (TryFindThesisRemeshRayHit(
						QueryGeometries[QueryGeometryIndex],
						QueryPoint,
						Hit,
						&LocalRejectedDestructiveHitCount))
				{
					OutHitCandidates.Add(Hit);
				}
			}
		}
		else
		{
			for (const FV6PlateQueryGeometry& QueryGeometry : QueryGeometries)
			{
				++LocalTestedPlateCount;
				FV6ThesisRemeshRayHit Hit;
				if (TryFindThesisRemeshRayHit(QueryGeometry, QueryPoint, Hit, &LocalRejectedDestructiveHitCount))
				{
					OutHitCandidates.Add(Hit);
				}
			}
		}

		if (OutRejectedDestructiveHitCount != nullptr)
		{
			*OutRejectedDestructiveHitCount += LocalRejectedDestructiveHitCount;
		}
		if (OutTestedPlateCount != nullptr)
		{
			*OutTestedPlateCount = LocalTestedPlateCount;
		}
	}

		bool TryFindThesisRemeshRayHit(
			const FV6PlateQueryGeometry& QueryGeometry,
			const FVector3d& QueryPoint,
			FV6ThesisRemeshRayHit& OutHit,
			int32* OutRejectedDestructiveHitCount)
	{
		OutHit = FV6ThesisRemeshRayHit{};
		if (QueryGeometry.SoupData.LocalTriangles.IsEmpty())
		{
			return false;
		}

		const FVector3d RayDirection = QueryPoint.GetSafeNormal();
		if (RayDirection.IsNearlyZero() || !DoesQueryPointPassBoundingCap(QueryGeometry, RayDirection))
		{
			return false;
		}

		const bool bFoundHit =
			TryFindThesisRemeshRawRayHitNoCap(QueryGeometry, RayDirection, OutHit) ||
			TryFindThesisRemeshContainingHitNoCap(QueryGeometry, QueryPoint, OutHit);
		if (bFoundHit && OutHit.bDestructiveTriangle)
		{
			if (OutRejectedDestructiveHitCount != nullptr)
			{
				++(*OutRejectedDestructiveHitCount);
			}
			OutHit = FV6ThesisRemeshRayHit{};
			return false;
		}

		return bFoundHit;
		}

		bool TryFindThesisRemeshHitIgnoringBoundingCap(
			const FV6PlateQueryGeometry& QueryGeometry,
			const FVector3d& QueryPoint,
			FV6ThesisRemeshRayHit& OutHit,
			int32* OutRejectedDestructiveHitCount = nullptr)
		{
			OutHit = FV6ThesisRemeshRayHit{};
			if (QueryGeometry.SoupData.LocalTriangles.IsEmpty())
			{
				return false;
			}

			const FVector3d RayDirection = QueryPoint.GetSafeNormal();
			if (RayDirection.IsNearlyZero())
			{
				return false;
			}

			const bool bFoundHit =
				TryFindThesisRemeshRawRayHitNoCap(QueryGeometry, RayDirection, OutHit) ||
				TryFindThesisRemeshContainingHitNoCap(QueryGeometry, QueryPoint, OutHit);
			if (bFoundHit && OutHit.bDestructiveTriangle)
			{
				if (OutRejectedDestructiveHitCount != nullptr)
				{
					++(*OutRejectedDestructiveHitCount);
				}
				OutHit = FV6ThesisRemeshRayHit{};
				return false;
			}

			return bFoundHit;
		}

	void CollectThesisRemeshRayHitCandidates(
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const FVector3d& QueryPoint,
		TArray<FV6ThesisRemeshRayHit>& OutHitCandidates,
		int32* OutRejectedDestructiveHitCount = nullptr)
	{
		CollectThesisRemeshRayHitCandidates(
			QueryGeometries,
			static_cast<const TArray<int32>*>(nullptr),
			QueryPoint,
			OutHitCandidates,
			OutRejectedDestructiveHitCount);
	}

	bool IsBetterThesisRemeshHit(const FV6ThesisRemeshRayHit& Candidate, const FV6ThesisRemeshRayHit& Best)
	{
		const bool bCloser = Candidate.RayParameter + TriangleEpsilon < Best.RayParameter;
		if (bCloser)
		{
			return true;
		}

		if (!FMath::IsNearlyEqual(Candidate.RayParameter, Best.RayParameter, TriangleEpsilon))
		{
			return false;
		}

		const bool bHigherFit = Candidate.FitScore > Best.FitScore + TriangleEpsilon;
		if (bHigherFit)
		{
			return true;
		}

		if (!FMath::IsNearlyEqual(Candidate.FitScore, Best.FitScore, TriangleEpsilon))
		{
			return false;
		}

		return Candidate.PlateId < Best.PlateId;
	}

	int32 CountThesisRemeshHitTrianglePlateSupport(
		const FTectonicPlanet& Planet,
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const TArray<int32>& PreviousPlateIds,
		const FV6ThesisRemeshRayHit& Hit,
		const int32 SupportedPlateId)
	{
		if (SupportedPlateId == INDEX_NONE)
		{
			return 0;
		}

		const int32 PlateIndex = Planet.FindPlateArrayIndexById(Hit.PlateId);
		if (!QueryGeometries.IsValidIndex(PlateIndex))
		{
			return 0;
		}

		const FV6PlateQueryGeometry& QueryGeometry = QueryGeometries[PlateIndex];
		if (!QueryGeometry.SoupData.LocalTriangles.IsValidIndex(Hit.LocalTriangleIndex))
		{
			return 0;
		}

		const UE::Geometry::FIndex3i& LocalTriangle = QueryGeometry.SoupData.LocalTriangles[Hit.LocalTriangleIndex];
		const int32 LocalVertices[3] = { LocalTriangle.A, LocalTriangle.B, LocalTriangle.C };

		int32 SupportCount = 0;
		for (const int32 LocalVertexIndex : LocalVertices)
		{
			if (!QueryGeometry.SoupData.LocalToCanonicalVertex.IsValidIndex(LocalVertexIndex))
			{
				continue;
			}

			const int32 CanonicalSampleIndex = QueryGeometry.SoupData.LocalToCanonicalVertex[LocalVertexIndex];
			if (PreviousPlateIds.IsValidIndex(CanonicalSampleIndex) &&
				PreviousPlateIds[CanonicalSampleIndex] == SupportedPlateId)
			{
				++SupportCount;
			}
		}

		return SupportCount;
	}

	void ApplyCopiedFrontierOverlapCoherenceFiltering(
		const FTectonicPlanet& Planet,
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const TArray<int32>& PreviousPlateIds,
		const int32 PreviousPlateId,
		TArray<FV6ThesisRemeshRayHit>& InOutHitCandidates,
		int32& OutSuppressedCandidateCount,
		bool& bOutUsedSupportPruning,
		bool& bOutUsedPreviousPlateStabilization)
	{
		OutSuppressedCandidateCount = 0;
		bOutUsedSupportPruning = false;
		bOutUsedPreviousPlateStabilization = false;
		if (InOutHitCandidates.Num() <= 1)
		{
			return;
		}

		TArray<int32, TInlineAllocator<8>> CandidateSupportCounts;
		CandidateSupportCounts.Reserve(InOutHitCandidates.Num());
		int32 BestSupportCount = 0;
		for (const FV6ThesisRemeshRayHit& Candidate : InOutHitCandidates)
		{
			const int32 SupportCount = CountThesisRemeshHitTrianglePlateSupport(
				Planet,
				QueryGeometries,
				PreviousPlateIds,
				Candidate,
				Candidate.PlateId);
			CandidateSupportCounts.Add(SupportCount);
			BestSupportCount = FMath::Max(BestSupportCount, SupportCount);
		}

		if (BestSupportCount >= 2)
		{
			TArray<FV6ThesisRemeshRayHit, TInlineAllocator<8>> PrunedCandidates;
			PrunedCandidates.Reserve(InOutHitCandidates.Num());
			for (int32 CandidateIndex = 0; CandidateIndex < InOutHitCandidates.Num(); ++CandidateIndex)
			{
				if (CandidateSupportCounts[CandidateIndex] == BestSupportCount)
				{
					PrunedCandidates.Add(InOutHitCandidates[CandidateIndex]);
				}
			}

			if (PrunedCandidates.Num() < InOutHitCandidates.Num())
			{
				OutSuppressedCandidateCount += InOutHitCandidates.Num() - PrunedCandidates.Num();
				bOutUsedSupportPruning = true;
				InOutHitCandidates.Reset(PrunedCandidates.Num());
				for (const FV6ThesisRemeshRayHit& Candidate : PrunedCandidates)
				{
					InOutHitCandidates.Add(Candidate);
				}

				CandidateSupportCounts.Reset();
				CandidateSupportCounts.Reserve(InOutHitCandidates.Num());
				for (int32 CandidateIndex = 0; CandidateIndex < InOutHitCandidates.Num(); ++CandidateIndex)
				{
					CandidateSupportCounts.Add(BestSupportCount);
				}
			}
		}

		if (InOutHitCandidates.Num() <= 1 || PreviousPlateId == INDEX_NONE)
		{
			return;
		}

		int32 BestCandidateIndex = 0;
		for (int32 CandidateIndex = 1; CandidateIndex < InOutHitCandidates.Num(); ++CandidateIndex)
		{
			if (IsBetterThesisRemeshHit(InOutHitCandidates[CandidateIndex], InOutHitCandidates[BestCandidateIndex]))
			{
				BestCandidateIndex = CandidateIndex;
			}
		}

		int32 PreviousPlateCandidateIndex = INDEX_NONE;
		for (int32 CandidateIndex = 0; CandidateIndex < InOutHitCandidates.Num(); ++CandidateIndex)
		{
			if (InOutHitCandidates[CandidateIndex].PlateId == PreviousPlateId)
			{
				PreviousPlateCandidateIndex = CandidateIndex;
				break;
			}
		}

		if (PreviousPlateCandidateIndex == INDEX_NONE || PreviousPlateCandidateIndex == BestCandidateIndex)
		{
			return;
		}

		const FV6ThesisRemeshRayHit& BestCandidate = InOutHitCandidates[BestCandidateIndex];
		const FV6ThesisRemeshRayHit& PreviousPlateCandidate = InOutHitCandidates[PreviousPlateCandidateIndex];
		const int32 BestCandidateSupportCount =
			CandidateSupportCounts.IsValidIndex(BestCandidateIndex) ? CandidateSupportCounts[BestCandidateIndex] : 0;
		const int32 PreviousPlateSupportCount =
			CandidateSupportCounts.IsValidIndex(PreviousPlateCandidateIndex) ? CandidateSupportCounts[PreviousPlateCandidateIndex] : 0;
		const bool bSupportCompetitive = PreviousPlateSupportCount >= BestCandidateSupportCount;
		const bool bRayCompetitive =
			PreviousPlateCandidate.RayParameter <= BestCandidate.RayParameter + CopiedFrontierPreviousPlateRayTieToleranceKm;
		const bool bFitCompetitive =
			PreviousPlateCandidate.FitScore + CopiedFrontierPreviousPlateFitTieTolerance >= BestCandidate.FitScore;
		if (!bSupportCompetitive || !bRayCompetitive || !bFitCompetitive)
		{
			return;
		}

		const FV6ThesisRemeshRayHit StableCandidate = PreviousPlateCandidate;
		OutSuppressedCandidateCount += InOutHitCandidates.Num() - 1;
		bOutUsedPreviousPlateStabilization = true;
		InOutHitCandidates.Reset(1);
		InOutHitCandidates.Add(StableCandidate);
	}

	template <typename CandidateAllocatorType>
	void CollectThesisRemeshMissRecoveryCandidates(
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const TArray<int32, CandidateAllocatorType>* CandidateQueryGeometryIndices,
		const FVector3d& QueryPoint,
		TArray<FTectonicPlanetV6RecoveryCandidate>& OutRecoveryCandidates,
		int32* OutTestedPlateCount = nullptr)
	{
		OutRecoveryCandidates.Reset();
		const int32 ReserveCount =
			CandidateQueryGeometryIndices != nullptr
				? CandidateQueryGeometryIndices->Num()
				: QueryGeometries.Num();
		OutRecoveryCandidates.Reserve(ReserveCount);

		int32 LocalTestedPlateCount = 0;
		if (CandidateQueryGeometryIndices != nullptr)
		{
			for (const int32 QueryGeometryIndex : *CandidateQueryGeometryIndices)
			{
				if (!QueryGeometries.IsValidIndex(QueryGeometryIndex))
				{
					continue;
				}

				++LocalTestedPlateCount;
				FRecoveredContainmentHit RecoveryHit;
				if (!TryFindNearestTriangleRecoveryHit(QueryGeometries[QueryGeometryIndex], QueryPoint, RecoveryHit))
				{
					continue;
				}

				FTectonicPlanetV6RecoveryCandidate& Candidate = OutRecoveryCandidates.AddDefaulted_GetRef();
				Candidate.PlateId = RecoveryHit.PlateId;
				Candidate.TriangleIndex = RecoveryHit.GlobalTriangleIndex;
				Candidate.LocalTriangleIndex = RecoveryHit.LocalTriangleIndex;
				Candidate.Barycentric = RecoveryHit.Barycentric;
				Candidate.DistanceRadians = RecoveryHit.Distance;
			}
		}
		else
		{
			for (const FV6PlateQueryGeometry& QueryGeometry : QueryGeometries)
			{
				++LocalTestedPlateCount;
				FRecoveredContainmentHit RecoveryHit;
				if (!TryFindNearestTriangleRecoveryHit(QueryGeometry, QueryPoint, RecoveryHit))
				{
					continue;
				}

				FTectonicPlanetV6RecoveryCandidate& Candidate = OutRecoveryCandidates.AddDefaulted_GetRef();
				Candidate.PlateId = RecoveryHit.PlateId;
				Candidate.TriangleIndex = RecoveryHit.GlobalTriangleIndex;
				Candidate.LocalTriangleIndex = RecoveryHit.LocalTriangleIndex;
				Candidate.Barycentric = RecoveryHit.Barycentric;
				Candidate.DistanceRadians = RecoveryHit.Distance;
			}
		}

		if (OutTestedPlateCount != nullptr)
		{
			*OutTestedPlateCount = LocalTestedPlateCount;
		}
	}

	void CollectThesisRemeshMissRecoveryCandidates(
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const FVector3d& QueryPoint,
		TArray<FTectonicPlanetV6RecoveryCandidate>& OutRecoveryCandidates)
	{
		CollectThesisRemeshMissRecoveryCandidates(
			QueryGeometries,
			static_cast<const TArray<int32>*>(nullptr),
			QueryPoint,
			OutRecoveryCandidates);
	}

	void FilterThesisRemeshHitCandidatesToAllowedPlates(
		TArray<FV6ThesisRemeshRayHit>& InOutHitCandidates,
		const int32 AllowedPlateIdA,
		const int32 AllowedPlateIdB)
	{
		if (AllowedPlateIdA == INDEX_NONE && AllowedPlateIdB == INDEX_NONE)
		{
			return;
		}

		InOutHitCandidates.RemoveAll([AllowedPlateIdA, AllowedPlateIdB](const FV6ThesisRemeshRayHit& Hit)
		{
			return Hit.PlateId != AllowedPlateIdA && Hit.PlateId != AllowedPlateIdB;
		});
	}

	void FilterThesisRemeshRecoveryCandidatesToAllowedPlates(
		TArray<FTectonicPlanetV6RecoveryCandidate>& InOutRecoveryCandidates,
		const int32 AllowedPlateIdA,
		const int32 AllowedPlateIdB)
	{
		if (AllowedPlateIdA == INDEX_NONE && AllowedPlateIdB == INDEX_NONE)
		{
			return;
		}

		InOutRecoveryCandidates.RemoveAll([AllowedPlateIdA, AllowedPlateIdB](const FTectonicPlanetV6RecoveryCandidate& Candidate)
		{
			return Candidate.PlateId != AllowedPlateIdA && Candidate.PlateId != AllowedPlateIdB;
		});
	}

	void ChooseThesisRemeshMissOwnerPlates(
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const int32 PreviousPlateId,
		const int32 ExplicitFallbackPlateId,
		int32& OutPrimaryPlateId,
		int32& OutSecondaryPlateId,
		double& OutPrimaryDistance)
	{
		OutPrimaryPlateId = INDEX_NONE;
		OutSecondaryPlateId = INDEX_NONE;
		OutPrimaryDistance = -1.0;
		double SecondaryDistance = TNumericLimits<double>::Max();

		for (const FTectonicPlanetV6RecoveryCandidate& Candidate : RecoveryCandidates)
		{
			if (Candidate.PlateId == INDEX_NONE)
			{
				continue;
			}

			const bool bBeatsPrimary =
				OutPrimaryPlateId == INDEX_NONE ||
				Candidate.DistanceRadians + TriangleEpsilon < OutPrimaryDistance ||
				(FMath::IsNearlyEqual(Candidate.DistanceRadians, OutPrimaryDistance, TriangleEpsilon) &&
					Candidate.PlateId < OutPrimaryPlateId);
			if (bBeatsPrimary)
			{
				if (OutPrimaryPlateId != INDEX_NONE && OutPrimaryPlateId != Candidate.PlateId)
				{
					OutSecondaryPlateId = OutPrimaryPlateId;
					SecondaryDistance = OutPrimaryDistance;
				}

				OutPrimaryPlateId = Candidate.PlateId;
				OutPrimaryDistance = Candidate.DistanceRadians;
				continue;
			}

			if (Candidate.PlateId == OutPrimaryPlateId)
			{
				continue;
			}

			const bool bBeatsSecondary =
				OutSecondaryPlateId == INDEX_NONE ||
				Candidate.DistanceRadians + TriangleEpsilon < SecondaryDistance ||
				(FMath::IsNearlyEqual(Candidate.DistanceRadians, SecondaryDistance, TriangleEpsilon) &&
					Candidate.PlateId < OutSecondaryPlateId);
			if (bBeatsSecondary)
			{
				OutSecondaryPlateId = Candidate.PlateId;
				SecondaryDistance = Candidate.DistanceRadians;
			}
		}

		if (OutPrimaryPlateId == INDEX_NONE)
		{
			OutPrimaryPlateId = PreviousPlateId != INDEX_NONE ? PreviousPlateId : ExplicitFallbackPlateId;
			OutPrimaryDistance = -1.0;
		}

		if (OutSecondaryPlateId == INDEX_NONE &&
			PreviousPlateId != INDEX_NONE &&
			PreviousPlateId != OutPrimaryPlateId)
		{
			OutSecondaryPlateId = PreviousPlateId;
		}
	}

	int32 ChooseDestructiveExclusionContinuationPlateId(
		const FTectonicPlanet& Planet,
		const TArray<FV6ThesisRemeshRayHit>& UnfilteredHitCandidates,
		const FV6CopiedFrontierDestructiveFilterState& DestructiveFilterState,
		const int32 PreviousPlateId,
		const int32 RecoveryPrimaryPlateId)
	{
		TMap<int32, int32> VotesByPlateId;
		TMap<int32, double> BestFitScoreByPlateId;

		for (const FV6ThesisRemeshRayHit& Hit : UnfilteredHitCandidates)
		{
			if (!IsCopiedFrontierTriangleDestructive(&DestructiveFilterState, Hit.GlobalTriangleIndex))
			{
				continue;
			}

			int32 PreferredPlateId = GetCopiedFrontierTrianglePreferredContinuationPlateId(
				&DestructiveFilterState,
				Hit.GlobalTriangleIndex);
			if (PreferredPlateId == INDEX_NONE)
			{
				PreferredPlateId = Hit.PlateId;
			}
			if (PreferredPlateId == INDEX_NONE)
			{
				continue;
			}

			++VotesByPlateId.FindOrAdd(PreferredPlateId);
			double& BestFitScore = BestFitScoreByPlateId.FindOrAdd(PreferredPlateId);
			BestFitScore = FMath::Max(BestFitScore, Hit.FitScore);
		}

		int32 BestPlateId = INDEX_NONE;
		int32 BestVoteCount = -1;
		double BestFitScore = -TNumericLimits<double>::Max();
		for (const TPair<int32, int32>& VotePair : VotesByPlateId)
		{
			const double CandidateBestFit = BestFitScoreByPlateId.FindRef(VotePair.Key);
			if (VotePair.Value > BestVoteCount ||
				(VotePair.Value == BestVoteCount && CandidateBestFit > BestFitScore + TriangleEpsilon) ||
				(VotePair.Value == BestVoteCount &&
					FMath::IsNearlyEqual(CandidateBestFit, BestFitScore, TriangleEpsilon) &&
					VotePair.Key == PreviousPlateId) ||
				(VotePair.Value == BestVoteCount &&
					FMath::IsNearlyEqual(CandidateBestFit, BestFitScore, TriangleEpsilon) &&
					BestPlateId != PreviousPlateId &&
					(BestPlateId == INDEX_NONE || VotePair.Key < BestPlateId)))
			{
				BestPlateId = VotePair.Key;
				BestVoteCount = VotePair.Value;
				BestFitScore = CandidateBestFit;
			}
		}

		if (BestPlateId != INDEX_NONE)
		{
			return BestPlateId;
		}
		if (PreviousPlateId != INDEX_NONE && FindPlateById(Planet, PreviousPlateId) != nullptr)
		{
			return PreviousPlateId;
		}
		if (RecoveryPrimaryPlateId != INDEX_NONE && FindPlateById(Planet, RecoveryPrimaryPlateId) != nullptr)
		{
			return RecoveryPrimaryPlateId;
		}
		return INDEX_NONE;
	}

	void SeedStructuredDestructiveGapSource(
		const FTectonicPlanet& Planet,
		const int32 PreferredPlateId,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		FTectonicPlanetV6ResolvedSample& InOutResolved,
		const FVector3d& QueryPoint)
	{
		InOutResolved.FinalPlateId = PreferredPlateId;
		InOutResolved.PreCoherencePlateId = PreferredPlateId;
		if (PreferredPlateId == INDEX_NONE)
		{
			return;
		}

		if (const FTectonicPlanetV6RecoveryCandidate* RecoveryCandidate =
				FindBestRecoveryCandidateForPlate(RecoveryCandidates, PreferredPlateId))
		{
			InOutResolved.SourceTriangleIndex = RecoveryCandidate->TriangleIndex;
			InOutResolved.SourceLocalTriangleIndex = RecoveryCandidate->LocalTriangleIndex;
			InOutResolved.SourceBarycentric = RecoveryCandidate->Barycentric;
			InOutResolved.RecoveryDistanceRadians = RecoveryCandidate->DistanceRadians;
			return;
		}

		double NearestMemberDistanceRadians = TNumericLimits<double>::Max();
		if (FindNearestMemberSample(
				Planet,
				PreferredPlateId,
				QueryPoint,
				InOutResolved.SourceCanonicalSampleIndex,
				NearestMemberDistanceRadians))
		{
			InOutResolved.RecoveryDistanceRadians = NearestMemberDistanceRadians;
		}
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

	bool FindGlobalNearestMemberRecovery(
		const FTectonicPlanet& Planet,
		const FVector3d& QueryPoint,
		int32& OutPlateId,
		int32& OutCanonicalSampleIndex,
		double& OutDistanceRadians)
	{
		OutPlateId = INDEX_NONE;
		OutCanonicalSampleIndex = INDEX_NONE;
		OutDistanceRadians = TNumericLimits<double>::Max();

		bool bFound = false;
		for (const FPlate& Plate : Planet.Plates)
		{
			int32 CandidateSampleIndex = INDEX_NONE;
			double CandidateDistance = TNumericLimits<double>::Max();
			if (!FindNearestMemberSample(Planet, Plate.Id, QueryPoint, CandidateSampleIndex, CandidateDistance))
			{
				continue;
			}

			const bool bCloser = CandidateDistance + TriangleEpsilon < OutDistanceRadians;
			const bool bTie = FMath::IsNearlyEqual(CandidateDistance, OutDistanceRadians, TriangleEpsilon);
			if (!bFound || bCloser || (bTie && Plate.Id < OutPlateId))
			{
				OutPlateId = Plate.Id;
				OutCanonicalSampleIndex = CandidateSampleIndex;
				OutDistanceRadians = CandidateDistance;
				bFound = true;
			}
		}

		return bFound;
	}

	void CollectBoundaryTrianglePlateIds(
		const int32 VertexPlateIdA,
		const int32 VertexPlateIdB,
		const int32 VertexPlateIdC,
		TArray<int32>& OutPlateIds)
	{
		OutPlateIds.Reset();
		if (VertexPlateIdA != INDEX_NONE)
		{
			OutPlateIds.AddUnique(VertexPlateIdA);
		}
		if (VertexPlateIdB != INDEX_NONE)
		{
			OutPlateIds.AddUnique(VertexPlateIdB);
		}
		if (VertexPlateIdC != INDEX_NONE)
		{
			OutPlateIds.AddUnique(VertexPlateIdC);
		}
		Algo::Sort(OutPlateIds);
	}

	void AppendTrianglesToV6QueryGeometry(
		const FTectonicPlanet& Planet,
		const FPlate& Plate,
		const TArray<int32>& TriangleIndices,
		FV6PlateQueryGeometry& OutQueryGeometry)
	{
		const FQuat4d& PlateRotation = Plate.CumulativeRotation;
		for (const int32 GlobalTriangleIndex : TriangleIndices)
		{
			if (!Planet.TriangleIndices.IsValidIndex(GlobalTriangleIndex))
			{
				continue;
			}

			const FIntVector& GlobalTriangle = Planet.TriangleIndices[GlobalTriangleIndex];
			const int32 CanonicalVertices[3] = { GlobalTriangle.X, GlobalTriangle.Y, GlobalTriangle.Z };
			int32 LocalVertices[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };

			for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
			{
				const int32 CanonicalVertexIndex = CanonicalVertices[CornerIndex];
				int32* ExistingLocalIndex = OutQueryGeometry.SoupData.CanonicalToLocalVertex.Find(CanonicalVertexIndex);
				if (ExistingLocalIndex != nullptr)
				{
					LocalVertices[CornerIndex] = *ExistingLocalIndex;
					continue;
				}

				if (!Planet.Samples.IsValidIndex(CanonicalVertexIndex))
				{
					continue;
				}

				const int32 NewLocalVertexIndex = OutQueryGeometry.SoupData.RotatedVertices.Num();
				const FVector3d RotatedPosition =
					PlateRotation.RotateVector(Planet.Samples[CanonicalVertexIndex].Position).GetSafeNormal();
				OutQueryGeometry.SoupData.CanonicalToLocalVertex.Add(CanonicalVertexIndex, NewLocalVertexIndex);
				OutQueryGeometry.SoupData.LocalToCanonicalVertex.Add(CanonicalVertexIndex);
				OutQueryGeometry.SoupData.RotatedVertices.Add(RotatedPosition);
				LocalVertices[CornerIndex] = NewLocalVertexIndex;
			}

			if (LocalVertices[0] == INDEX_NONE || LocalVertices[1] == INDEX_NONE || LocalVertices[2] == INDEX_NONE)
			{
				continue;
			}

			const int32 LocalTriangleIndex = OutQueryGeometry.SoupData.LocalTriangles.Num();
			OutQueryGeometry.SoupData.GlobalTriangleIndices.Add(GlobalTriangleIndex);
			OutQueryGeometry.SoupData.LocalTriangles.Add(
				UE::Geometry::FIndex3i(LocalVertices[0], LocalVertices[1], LocalVertices[2]));
			OutQueryGeometry.SoupData.GlobalToLocalTriangle.Add(GlobalTriangleIndex, LocalTriangleIndex);
			OutQueryGeometry.LocalTriangleCopiedFrontierFlags.Add(false);
		}
	}

	int32 FindOrAddCopiedFrontierMeshVertex(
		const FTectonicPlanet& Planet,
		FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh,
		const int32 CanonicalVertexIndex)
	{
		if (const int32* ExistingLocalIndex = Mesh.CanonicalToLocalVertex.Find(CanonicalVertexIndex))
		{
			return *ExistingLocalIndex;
		}

		if (!Planet.Samples.IsValidIndex(CanonicalVertexIndex))
		{
			return INDEX_NONE;
		}

		const int32 NewLocalVertexIndex = Mesh.BaseVertices.Num();
		Mesh.BaseVertices.Add(Planet.Samples[CanonicalVertexIndex].Position);
		Mesh.LocalToCanonicalVertex.Add(CanonicalVertexIndex);
		Mesh.CanonicalToLocalVertex.Add(CanonicalVertexIndex, NewLocalVertexIndex);
		FCarriedSample& LocalCarriedSample = Mesh.LocalCarriedSamples.AddDefaulted_GetRef();
		PopulateV6CarriedSampleFromCanonical(Planet, CanonicalVertexIndex, LocalCarriedSample);
		Mesh.LocalVertexSourceTriangleIndices.Add(INDEX_NONE);
		Mesh.LocalVertexSourceBarycentrics.Add(FVector3d::ZeroVector);
		Mesh.LocalVertexCopiedFrontierFlags.Add(false);
		return NewLocalVertexIndex;
	}

	void MarkCopiedFrontierMeshVertex(
		FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh,
		const int32 LocalVertexIndex)
	{
		if (!Mesh.LocalVertexCopiedFrontierFlags.IsValidIndex(LocalVertexIndex) ||
			Mesh.LocalVertexCopiedFrontierFlags[LocalVertexIndex] != 0)
		{
			return;
		}

		Mesh.LocalVertexCopiedFrontierFlags[LocalVertexIndex] = true;
		++Mesh.CopiedFrontierVertexCount;
		++Mesh.CopiedFrontierCarriedSampleCount;
	}

	struct FV6PartitionedCopiedFrontierVertexDesc
	{
		bool bCanonical = false;
		int32 CanonicalVertexIndex = INDEX_NONE;
		FVector3d Barycentric = FVector3d::ZeroVector;
	};

	FCarriedSample BuildSyntheticCopiedFrontierCarriedSample(
		const FTectonicPlanet& Planet,
		const FIntVector& Triangle,
		const FVector3d& Barycentric,
		const FVector3d& SurfaceNormal)
	{
		FCarriedSample SyntheticSample;
		if (!Planet.Samples.IsValidIndex(Triangle.X) ||
			!Planet.Samples.IsValidIndex(Triangle.Y) ||
			!Planet.Samples.IsValidIndex(Triangle.Z))
		{
			return SyntheticSample;
		}

		const FSample& SampleA = Planet.Samples[Triangle.X];
		const FSample& SampleB = Planet.Samples[Triangle.Y];
		const FSample& SampleC = Planet.Samples[Triangle.Z];
		const int32 DominantIndex = PickDominantBarycentricIndex(Barycentric);
		SyntheticSample.CanonicalSampleIndex =
			DominantIndex == 1 ? Triangle.Y : (DominantIndex == 2 ? Triangle.Z : Triangle.X);
		SyntheticSample.ContinentalWeight = static_cast<float>(
			(Barycentric.X * static_cast<double>(SampleA.ContinentalWeight)) +
			(Barycentric.Y * static_cast<double>(SampleB.ContinentalWeight)) +
			(Barycentric.Z * static_cast<double>(SampleC.ContinentalWeight)));
		SyntheticSample.Elevation = static_cast<float>(
			(Barycentric.X * static_cast<double>(SampleA.Elevation)) +
			(Barycentric.Y * static_cast<double>(SampleB.Elevation)) +
			(Barycentric.Z * static_cast<double>(SampleC.Elevation)));
		SyntheticSample.Thickness = static_cast<float>(
			(Barycentric.X * static_cast<double>(SampleA.Thickness)) +
			(Barycentric.Y * static_cast<double>(SampleB.Thickness)) +
			(Barycentric.Z * static_cast<double>(SampleC.Thickness)));
		SyntheticSample.Age = static_cast<float>(
			(Barycentric.X * static_cast<double>(SampleA.Age)) +
			(Barycentric.Y * static_cast<double>(SampleB.Age)) +
			(Barycentric.Z * static_cast<double>(SampleC.Age)));
		SyntheticSample.OrogenyType =
			DominantIndex == 1 ? SampleB.OrogenyType : (DominantIndex == 2 ? SampleC.OrogenyType : SampleA.OrogenyType);
		SyntheticSample.TerraneId =
			DominantIndex == 1 ? SampleB.TerraneId : (DominantIndex == 2 ? SampleC.TerraneId : SampleA.TerraneId);
		ETectonicPlanetV6DirectionalTransferKind RidgeTransferKind = ETectonicPlanetV6DirectionalTransferKind::None;
		SyntheticSample.RidgeDirection = TransferDirectionalField(
			SampleA.RidgeDirection,
			SampleB.RidgeDirection,
			SampleC.RidgeDirection,
			Barycentric,
			SurfaceNormal,
			RidgeTransferKind);
		ETectonicPlanetV6DirectionalTransferKind FoldTransferKind = ETectonicPlanetV6DirectionalTransferKind::None;
		SyntheticSample.FoldDirection = TransferDirectionalField(
			SampleA.FoldDirection,
			SampleB.FoldDirection,
			SampleC.FoldDirection,
			Barycentric,
			SurfaceNormal,
			FoldTransferKind);
		SyntheticSample.SubductionDistanceKm = static_cast<float>(
			(Barycentric.X * static_cast<double>(SampleA.SubductionDistanceKm)) +
			(Barycentric.Y * static_cast<double>(SampleB.SubductionDistanceKm)) +
			(Barycentric.Z * static_cast<double>(SampleC.SubductionDistanceKm)));
		SyntheticSample.SubductionSpeed = 0.0f;
		return SyntheticSample;
	}

	int32 AddCopiedFrontierSyntheticMeshVertex(
		const FTectonicPlanet& Planet,
		FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh,
		const FIntVector& Triangle,
		const int32 GlobalTriangleIndex,
		const FVector3d& Barycentric,
		const bool bMarkCopiedFrontier)
	{
		const FVector3d SurfacePosition =
			((Planet.Samples[Triangle.X].Position * Barycentric.X) +
				(Planet.Samples[Triangle.Y].Position * Barycentric.Y) +
				(Planet.Samples[Triangle.Z].Position * Barycentric.Z)).GetSafeNormal();
		if (SurfacePosition.IsNearlyZero())
		{
			return INDEX_NONE;
		}

		const int32 LocalVertexIndex = Mesh.BaseVertices.Num();
		Mesh.BaseVertices.Add(SurfacePosition);
		Mesh.LocalToCanonicalVertex.Add(INDEX_NONE);
		FCarriedSample& LocalCarriedSample = Mesh.LocalCarriedSamples.AddDefaulted_GetRef();
		LocalCarriedSample = BuildSyntheticCopiedFrontierCarriedSample(
			Planet,
			Triangle,
			Barycentric,
			SurfacePosition);
		Mesh.LocalVertexSourceTriangleIndices.Add(GlobalTriangleIndex);
		Mesh.LocalVertexSourceBarycentrics.Add(Barycentric);
		Mesh.LocalVertexCopiedFrontierFlags.Add(0);
		if (bMarkCopiedFrontier)
		{
			MarkCopiedFrontierMeshVertex(Mesh, LocalVertexIndex);
		}
		return LocalVertexIndex;
	}

	int32 ResolveCopiedFrontierPartitionVertex(
		const FTectonicPlanet& Planet,
		FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh,
		const FIntVector& Triangle,
		const int32 GlobalTriangleIndex,
		const FV6PartitionedCopiedFrontierVertexDesc& VertexDesc,
		const bool bMarkCopiedFrontier)
	{
		if (VertexDesc.bCanonical)
		{
			const int32 LocalVertexIndex =
				FindOrAddCopiedFrontierMeshVertex(Planet, Mesh, VertexDesc.CanonicalVertexIndex);
			if (LocalVertexIndex != INDEX_NONE && bMarkCopiedFrontier)
			{
				MarkCopiedFrontierMeshVertex(Mesh, LocalVertexIndex);
			}
			return LocalVertexIndex;
		}

		return AddCopiedFrontierSyntheticMeshVertex(
			Planet,
			Mesh,
			Triangle,
			GlobalTriangleIndex,
			VertexDesc.Barycentric,
			bMarkCopiedFrontier);
	}

	void RefreshCopiedFrontierMeshCarriedSamples(
		const FTectonicPlanet& Planet,
		TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& Meshes,
		int32& OutCanonicalRefreshCount,
		int32& OutSyntheticRefreshCount)
	{
		OutCanonicalRefreshCount = 0;
		OutSyntheticRefreshCount = 0;

		for (FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh : Meshes)
		{
			const int32 LocalVertexCount = Mesh.BaseVertices.Num();
			if (Mesh.LocalCarriedSamples.Num() != LocalVertexCount)
			{
				Mesh.LocalCarriedSamples.SetNum(LocalVertexCount);
			}

			for (int32 LocalVertexIndex = 0; LocalVertexIndex < LocalVertexCount; ++LocalVertexIndex)
			{
				const int32 CanonicalVertexIndex =
					Mesh.LocalToCanonicalVertex.IsValidIndex(LocalVertexIndex)
						? Mesh.LocalToCanonicalVertex[LocalVertexIndex]
						: INDEX_NONE;
				if (CanonicalVertexIndex != INDEX_NONE)
				{
					PopulateV6CarriedSampleFromCanonical(
						Planet,
						CanonicalVertexIndex,
						Mesh.LocalCarriedSamples[LocalVertexIndex]);
					++OutCanonicalRefreshCount;
					continue;
				}

				const int32 SourceTriangleIndex =
					Mesh.LocalVertexSourceTriangleIndices.IsValidIndex(LocalVertexIndex)
						? Mesh.LocalVertexSourceTriangleIndices[LocalVertexIndex]
						: INDEX_NONE;
				if (SourceTriangleIndex == INDEX_NONE ||
					!Planet.TriangleIndices.IsValidIndex(SourceTriangleIndex) ||
					!Mesh.LocalVertexSourceBarycentrics.IsValidIndex(LocalVertexIndex) ||
					!Mesh.BaseVertices.IsValidIndex(LocalVertexIndex))
				{
					continue;
				}

				Mesh.LocalCarriedSamples[LocalVertexIndex] =
					BuildSyntheticCopiedFrontierCarriedSample(
						Planet,
						Planet.TriangleIndices[SourceTriangleIndex],
						Mesh.LocalVertexSourceBarycentrics[LocalVertexIndex],
						Mesh.BaseVertices[LocalVertexIndex]);
				++OutSyntheticRefreshCount;
			}
		}
	}

	void AddCopiedFrontierMeshTriangle(
		FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh,
		const int32 GlobalTriangleIndex,
		const int32 LocalVertexA,
		const int32 LocalVertexB,
		const int32 LocalVertexC,
		const bool bCopiedFrontierTriangle)
	{
		if (LocalVertexA == INDEX_NONE || LocalVertexB == INDEX_NONE || LocalVertexC == INDEX_NONE ||
			LocalVertexA == LocalVertexB || LocalVertexB == LocalVertexC || LocalVertexC == LocalVertexA)
		{
			return;
		}

		const int32 LocalTriangleIndex = Mesh.LocalTriangles.Num();
		Mesh.GlobalTriangleIndices.Add(GlobalTriangleIndex);
		Mesh.LocalTriangles.Add(UE::Geometry::FIndex3i(LocalVertexA, LocalVertexB, LocalVertexC));
		if (!Mesh.GlobalToLocalTriangle.Contains(GlobalTriangleIndex))
		{
			Mesh.GlobalToLocalTriangle.Add(GlobalTriangleIndex, LocalTriangleIndex);
		}
		Mesh.LocalTriangleCopiedFrontierFlags.Add(bCopiedFrontierTriangle);
		if (bCopiedFrontierTriangle)
		{
			++Mesh.CopiedFrontierTriangleCount;
			MarkCopiedFrontierMeshVertex(Mesh, LocalVertexA);
			MarkCopiedFrontierMeshVertex(Mesh, LocalVertexB);
			MarkCopiedFrontierMeshVertex(Mesh, LocalVertexC);
		}
	}

	int32 ComputeCopiedFrontierExcludedLocalTriangleCount(
		const int32 InvolvedPlateCount,
		const EV6ThesisFrontierMeshBuildMode BuildMode)
	{
		if (BuildMode == EV6ThesisFrontierMeshBuildMode::PartitionedMixedTriangles)
		{
			if (InvolvedPlateCount <= 1)
			{
				return 1;
			}
			if (InvolvedPlateCount == 2)
			{
				return 3;
			}
			return 6;
		}

		return FMath::Max(1, InvolvedPlateCount);
	}

	int32 CountPlateOccurrencesInTriangle(
		const int32 TrianglePlateIds[3],
		const int32 PlateId)
	{
		int32 Count = 0;
		for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
		{
			Count += TrianglePlateIds[CornerIndex] == PlateId ? 1 : 0;
		}
		return Count;
	}

	void AddWholeTriangleCoverageSupportToCopiedFrontierMesh(
		const FTectonicPlanet& Planet,
		const FIntVector& Triangle,
		const int32 GlobalTriangleIndex,
		const int32 PlateId,
		TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& OutMeshes)
	{
		const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
		if (!OutMeshes.IsValidIndex(PlateIndex))
		{
			return;
		}

		FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = OutMeshes[PlateIndex];
		const int32 LocalVertices[3] = {
			FindOrAddCopiedFrontierMeshVertex(Planet, Mesh, Triangle.X),
			FindOrAddCopiedFrontierMeshVertex(Planet, Mesh, Triangle.Y),
			FindOrAddCopiedFrontierMeshVertex(Planet, Mesh, Triangle.Z) };
		AddCopiedFrontierMeshTriangle(
			Mesh,
			GlobalTriangleIndex,
			LocalVertices[0],
			LocalVertices[1],
			LocalVertices[2],
			true);
	}

	void AddPartitionedMixedTriangleToCopiedFrontierMeshes(
		const FTectonicPlanet& Planet,
		const FIntVector& Triangle,
		const int32 GlobalTriangleIndex,
		const int32 TrianglePlateIds[3],
		const TArray<int32>& InvolvedPlateIds,
		TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& OutMeshes)
	{
		const FVector3d EdgeMidpointWeights[3] = {
			FVector3d(0.5, 0.5, 0.0),
			FVector3d(0.0, 0.5, 0.5),
			FVector3d(0.5, 0.0, 0.5)
		};
		const FVector3d CentroidWeights(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);

		for (const int32 PlateId : InvolvedPlateIds)
		{
			TArray<FV6PartitionedCopiedFrontierVertexDesc, TInlineAllocator<4>> PolygonVertices;
			for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
			{
				const int32 NextCornerIndex = (CornerIndex + 1) % 3;
				if (TrianglePlateIds[CornerIndex] == PlateId)
				{
					FV6PartitionedCopiedFrontierVertexDesc& VertexDesc = PolygonVertices.AddDefaulted_GetRef();
					VertexDesc.bCanonical = true;
					VertexDesc.CanonicalVertexIndex =
						CornerIndex == 0 ? Triangle.X : (CornerIndex == 1 ? Triangle.Y : Triangle.Z);
				}

				const bool bCrossesPlateBoundary =
					(TrianglePlateIds[CornerIndex] == PlateId) != (TrianglePlateIds[NextCornerIndex] == PlateId);
				if (bCrossesPlateBoundary)
				{
					FV6PartitionedCopiedFrontierVertexDesc& VertexDesc = PolygonVertices.AddDefaulted_GetRef();
					VertexDesc.Barycentric = EdgeMidpointWeights[CornerIndex];
				}
			}

			if (InvolvedPlateIds.Num() == 3 && PolygonVertices.Num() == 3)
			{
				const int32 InsertIndex = PolygonVertices.Num() - 1;
				PolygonVertices.InsertDefaulted(InsertIndex, 1);
				FV6PartitionedCopiedFrontierVertexDesc& VertexDesc = PolygonVertices[InsertIndex];
				VertexDesc.Barycentric = CentroidWeights;
			}

			if (PolygonVertices.Num() < 3)
			{
				continue;
			}

			const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
			if (!OutMeshes.IsValidIndex(PlateIndex))
			{
				continue;
			}

			FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = OutMeshes[PlateIndex];
			TArray<int32, TInlineAllocator<4>> LocalVertices;
			LocalVertices.Reserve(PolygonVertices.Num());
			for (const FV6PartitionedCopiedFrontierVertexDesc& VertexDesc : PolygonVertices)
			{
				LocalVertices.Add(ResolveCopiedFrontierPartitionVertex(
					Planet,
					Mesh,
					Triangle,
					GlobalTriangleIndex,
					VertexDesc,
					true));
			}

			for (int32 PolygonVertexIndex = 1; PolygonVertexIndex + 1 < LocalVertices.Num(); ++PolygonVertexIndex)
			{
				AddCopiedFrontierMeshTriangle(
					Mesh,
					GlobalTriangleIndex,
					LocalVertices[0],
					LocalVertices[PolygonVertexIndex],
					LocalVertices[PolygonVertexIndex + 1],
					true);
			}
		}
	}

	void BuildV6CopiedFrontierPlateMeshes(
		const FTectonicPlanet& Planet,
		TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& OutMeshes,
		FV6CopiedFrontierDestructiveFilterState* DestructiveFilterState = nullptr,
		const EV6ThesisFrontierMeshBuildMode BuildMode = EV6ThesisFrontierMeshBuildMode::WholeTriangleDuplication,
		const TArray<uint8>* PreviousSyntheticVertexFlags = nullptr)
	{
		OutMeshes.SetNum(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = OutMeshes[PlateIndex];
			Mesh = FTectonicPlanetV6CopiedFrontierPlateMesh{};
			Mesh.PlateId = Planet.Plates[PlateIndex].Id;
		}

		for (int32 GlobalTriangleIndex = 0; GlobalTriangleIndex < Planet.TriangleIndices.Num(); ++GlobalTriangleIndex)
		{
			if (!Planet.TriangleIndices.IsValidIndex(GlobalTriangleIndex))
			{
				continue;
			}

			const FIntVector& Triangle = Planet.TriangleIndices[GlobalTriangleIndex];
			if (!Planet.Samples.IsValidIndex(Triangle.X) ||
				!Planet.Samples.IsValidIndex(Triangle.Y) ||
				!Planet.Samples.IsValidIndex(Triangle.Z))
			{
				continue;
			}

			const int32 TrianglePlateIds[3] = {
				Planet.Samples[Triangle.X].PlateId,
				Planet.Samples[Triangle.Y].PlateId,
				Planet.Samples[Triangle.Z].PlateId
			};
			TArray<int32> InvolvedPlateIds;
			CollectBoundaryTrianglePlateIds(
				TrianglePlateIds[0],
				TrianglePlateIds[1],
				TrianglePlateIds[2],
				InvolvedPlateIds);
			if (InvolvedPlateIds.IsEmpty())
			{
				continue;
			}

			const bool bCopiedFrontierTriangle = InvolvedPlateIds.Num() > 1;
			if (bCopiedFrontierTriangle && BuildMode == EV6ThesisFrontierMeshBuildMode::ExcludeMixed)
			{
				continue;
			}
			if (IsCopiedFrontierTriangleDestructive(DestructiveFilterState, GlobalTriangleIndex))
			{
				if (DestructiveFilterState != nullptr)
				{
					DestructiveFilterState->GeometryExcludedLocalTriangleCount +=
						ComputeCopiedFrontierExcludedLocalTriangleCount(InvolvedPlateIds.Num(), BuildMode);
				}
				continue;
			}

				if (bCopiedFrontierTriangle &&
					BuildMode == EV6ThesisFrontierMeshBuildMode::PartitionedMixedTriangles)
				{
					AddPartitionedMixedTriangleToCopiedFrontierMeshes(
						Planet,
					Triangle,
					GlobalTriangleIndex,
					TrianglePlateIds,
						InvolvedPlateIds,
						OutMeshes);

					if (PreviousSyntheticVertexFlags != nullptr &&
						PreviousSyntheticVertexFlags->Num() == Planet.Samples.Num())
					{
						TArray<int32, TInlineAllocator<3>> SupportPlateIds;
						const int32 CanonicalVertices[3] = { Triangle.X, Triangle.Y, Triangle.Z };
						for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
						{
							const int32 CanonicalSampleIndex = CanonicalVertices[CornerIndex];
								const int32 SupportPlateId = TrianglePlateIds[CornerIndex];
								if (!PreviousSyntheticVertexFlags->IsValidIndex(CanonicalSampleIndex) ||
									(*PreviousSyntheticVertexFlags)[CanonicalSampleIndex] == 0 ||
									FindPlateById(Planet, SupportPlateId) == nullptr)
								{
									continue;
								}

							SupportPlateIds.AddUnique(SupportPlateId);
						}

						for (const int32 SupportPlateId : SupportPlateIds)
						{
							AddWholeTriangleCoverageSupportToCopiedFrontierMesh(
								Planet,
								Triangle,
								GlobalTriangleIndex,
								SupportPlateId,
								OutMeshes);
						}
					}
					continue;
				}

			for (const int32 PlateId : InvolvedPlateIds)
			{
				const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
				if (!OutMeshes.IsValidIndex(PlateIndex))
				{
					continue;
				}

				FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = OutMeshes[PlateIndex];
				const int32 LocalVertices[3] = {
					FindOrAddCopiedFrontierMeshVertex(Planet, Mesh, Triangle.X),
					FindOrAddCopiedFrontierMeshVertex(Planet, Mesh, Triangle.Y),
					FindOrAddCopiedFrontierMeshVertex(Planet, Mesh, Triangle.Z) };
				AddCopiedFrontierMeshTriangle(
					Mesh,
					GlobalTriangleIndex,
					LocalVertices[0],
					LocalVertices[1],
					LocalVertices[2],
					bCopiedFrontierTriangle);
			}
		}
	}

	void BuildV6CopiedFrontierQueryGeometries(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		TArray<FV6PlateQueryGeometry>& OutQueryGeometries,
		const FV6CopiedFrontierDestructiveFilterState* DestructiveFilterState = nullptr)
	{
		OutQueryGeometries.SetNum(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			const FPlate& Plate = Planet.Plates[PlateIndex];
			const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = PlateMeshes[PlateIndex];
			FV6PlateQueryGeometry& QueryGeometry = OutQueryGeometries[PlateIndex];
			ResetV6QueryGeometry(QueryGeometry);
			QueryGeometry.PlateId = Plate.Id;
			QueryGeometry.SoupData.PlateId = Plate.Id;
			QueryGeometry.SoupData.ChangeStamp = 1;

			if (Mesh.LocalTriangles.IsEmpty())
			{
				continue;
			}

			QueryGeometry.SoupData.GlobalTriangleIndices = Mesh.GlobalTriangleIndices;
			QueryGeometry.SoupData.GlobalToLocalTriangle = Mesh.GlobalToLocalTriangle;
			QueryGeometry.SoupData.LocalTriangles = Mesh.LocalTriangles;
			QueryGeometry.SoupData.LocalToCanonicalVertex = Mesh.LocalToCanonicalVertex;
			QueryGeometry.SoupData.CanonicalToLocalVertex = Mesh.CanonicalToLocalVertex;
			QueryGeometry.LocalTriangleCopiedFrontierFlags = Mesh.LocalTriangleCopiedFrontierFlags;
			QueryGeometry.LocalTriangleDestructiveFlags.Init(0, Mesh.LocalTriangles.Num());
			for (int32 LocalTriangleIndex = 0; LocalTriangleIndex < Mesh.LocalTriangles.Num(); ++LocalTriangleIndex)
			{
				const int32 GlobalTriangleIndex =
					Mesh.GlobalTriangleIndices.IsValidIndex(LocalTriangleIndex)
						? Mesh.GlobalTriangleIndices[LocalTriangleIndex]
						: INDEX_NONE;
				QueryGeometry.LocalTriangleDestructiveFlags[LocalTriangleIndex] =
					IsCopiedFrontierTriangleDestructive(DestructiveFilterState, GlobalTriangleIndex) ? 1 : 0;
			}
			QueryGeometry.SoupData.RotatedVertices.Reserve(Mesh.BaseVertices.Num());
			for (const FVector3d& BaseVertex : Mesh.BaseVertices)
			{
				QueryGeometry.SoupData.RotatedVertices.Add(Plate.CumulativeRotation.RotateVector(BaseVertex).GetSafeNormal());
			}

			QueryGeometry.SoupData.ChangeStamp++;
			QueryGeometry.SoupAdapter = FPlateTriangleSoupAdapter(&QueryGeometry.SoupData);
			QueryGeometry.SoupBVH.SetMesh(&QueryGeometry.SoupAdapter, true);
			QueryGeometry.BoundingCap = BuildBoundingCapFromVertices(QueryGeometry.SoupData.RotatedVertices);
		}
	}

	int32 FindOrAddPlateSubmeshVertex(
		const FTectonicPlanet& Planet,
		FTectonicPlanetV6PlateSubmesh& Mesh,
		const int32 CanonicalVertexIndex)
	{
		if (const int32* ExistingLocalIndex = Mesh.CanonicalToLocalVertex.Find(CanonicalVertexIndex))
		{
			return *ExistingLocalIndex;
		}

		if (!Planet.Samples.IsValidIndex(CanonicalVertexIndex))
		{
			return INDEX_NONE;
		}

		const int32 NewLocalVertexIndex = Mesh.BaseVertices.Num();
		Mesh.BaseVertices.Add(Planet.Samples[CanonicalVertexIndex].Position);
		Mesh.LocalToCanonicalVertex.Add(CanonicalVertexIndex);
		Mesh.CanonicalToLocalVertex.Add(CanonicalVertexIndex, NewLocalVertexIndex);
		FCarriedSample& LocalCarriedSample = Mesh.LocalCarriedSamples.AddDefaulted_GetRef();
		PopulateV6CarriedSampleFromCanonical(Planet, CanonicalVertexIndex, LocalCarriedSample);
		Mesh.LocalVertexFrontierFlags.Add(false);
		return NewLocalVertexIndex;
	}

	void MarkPlateSubmeshVertexFrontier(
		FTectonicPlanetV6PlateSubmesh& Mesh,
		const int32 LocalVertexIndex)
	{
		if (!Mesh.LocalVertexFrontierFlags.IsValidIndex(LocalVertexIndex) ||
			Mesh.LocalVertexFrontierFlags[LocalVertexIndex] != 0)
		{
			return;
		}

		Mesh.LocalVertexFrontierFlags[LocalVertexIndex] = true;
		++Mesh.FrontierVertexCount;
		++Mesh.FrontierCarriedSampleCount;
	}

	void AddPlateSubmeshTriangle(
		FTectonicPlanetV6PlateSubmesh& Mesh,
		const int32 LocalVertexA,
		const int32 LocalVertexB,
		const int32 LocalVertexC)
	{
		if (LocalVertexA == INDEX_NONE || LocalVertexB == INDEX_NONE || LocalVertexC == INDEX_NONE ||
			LocalVertexA == LocalVertexB || LocalVertexB == LocalVertexC || LocalVertexC == LocalVertexA)
		{
			return;
		}

		const bool bFrontierTriangle =
			(Mesh.LocalVertexFrontierFlags.IsValidIndex(LocalVertexA) && Mesh.LocalVertexFrontierFlags[LocalVertexA] != 0) ||
			(Mesh.LocalVertexFrontierFlags.IsValidIndex(LocalVertexB) && Mesh.LocalVertexFrontierFlags[LocalVertexB] != 0) ||
			(Mesh.LocalVertexFrontierFlags.IsValidIndex(LocalVertexC) && Mesh.LocalVertexFrontierFlags[LocalVertexC] != 0);

		Mesh.LocalTriangles.Add(UE::Geometry::FIndex3i(LocalVertexA, LocalVertexB, LocalVertexC));
		Mesh.LocalTriangleFrontierFlags.Add(bFrontierTriangle);
		++Mesh.RetriangulatedTriangleCount;
		if (bFrontierTriangle)
		{
			++Mesh.FrontierTriangleCount;
		}
	}

	void BuildV6PlateSubmeshMeshes(
		const FTectonicPlanet& Planet,
		TArray<FTectonicPlanetV6PlateSubmesh>& OutMeshes)
	{
		OutMeshes.SetNum(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			FTectonicPlanetV6PlateSubmesh& Mesh = OutMeshes[PlateIndex];
			Mesh = FTectonicPlanetV6PlateSubmesh{};
			Mesh.PlateId = Planet.Plates[PlateIndex].Id;
		}

		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			const FPlate& Plate = Planet.Plates[PlateIndex];
			FTectonicPlanetV6PlateSubmesh& Mesh = OutMeshes[PlateIndex];
			TSet<int32> VisitedSamples;

			for (const int32 SeedSampleIndex : Plate.MemberSamples)
			{
				if (!Planet.Samples.IsValidIndex(SeedSampleIndex) ||
					Planet.Samples[SeedSampleIndex].PlateId != Plate.Id ||
					VisitedSamples.Contains(SeedSampleIndex))
				{
					continue;
				}

				TArray<int32, TInlineAllocator<128>> Stack;
				TArray<int32> ComponentSampleIndices;
				Stack.Add(SeedSampleIndex);
				VisitedSamples.Add(SeedSampleIndex);

				while (!Stack.IsEmpty())
				{
					const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
					ComponentSampleIndices.Add(SampleIndex);
					if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
					{
						continue;
					}

					for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
					{
						if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
							Planet.Samples[NeighborIndex].PlateId != Plate.Id ||
							VisitedSamples.Contains(NeighborIndex))
						{
							continue;
						}

						VisitedSamples.Add(NeighborIndex);
						Stack.Add(NeighborIndex);
					}
				}

				if (ComponentSampleIndices.IsEmpty())
				{
					continue;
				}

				++Mesh.ComponentCount;

				TArray<FVector3d> ComponentVertices;
				TArray<int32> ComponentLocalVertices;
				ComponentVertices.Reserve(ComponentSampleIndices.Num());
				ComponentLocalVertices.Reserve(ComponentSampleIndices.Num());

				for (const int32 CanonicalSampleIndex : ComponentSampleIndices)
				{
					const int32 LocalVertexIndex = FindOrAddPlateSubmeshVertex(Planet, Mesh, CanonicalSampleIndex);
					if (LocalVertexIndex == INDEX_NONE)
					{
						continue;
					}

					ComponentVertices.Add(Planet.Samples[CanonicalSampleIndex].Position);
					ComponentLocalVertices.Add(LocalVertexIndex);
					if (Planet.Samples[CanonicalSampleIndex].bIsBoundary)
					{
						MarkPlateSubmeshVertexFrontier(Mesh, LocalVertexIndex);
					}
				}

				if (ComponentLocalVertices.Num() < 3)
				{
					continue;
				}

				if (ComponentLocalVertices.Num() == 3)
				{
					AddPlateSubmeshTriangle(
						Mesh,
						ComponentLocalVertices[0],
						ComponentLocalVertices[1],
						ComponentLocalVertices[2]);
					continue;
				}

				UE::Geometry::TConvexHull3<double> Hull;
				if (!Hull.Solve(TArrayView<const FVector3d>(ComponentVertices.GetData(), ComponentVertices.Num())))
				{
					continue;
				}

				for (const UE::Geometry::FIndex3i& HullTriangle : Hull.GetTriangles())
				{
					if (!ComponentLocalVertices.IsValidIndex(HullTriangle.A) ||
						!ComponentLocalVertices.IsValidIndex(HullTriangle.B) ||
						!ComponentLocalVertices.IsValidIndex(HullTriangle.C))
					{
						continue;
					}

					AddPlateSubmeshTriangle(
						Mesh,
						ComponentLocalVertices[HullTriangle.A],
						ComponentLocalVertices[HullTriangle.B],
						ComponentLocalVertices[HullTriangle.C]);
				}
			}
		}
	}

	void BuildV6PlateSubmeshQueryGeometries(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6PlateSubmesh>& PlateMeshes,
		TArray<FV6PlateQueryGeometry>& OutQueryGeometries)
	{
		OutQueryGeometries.SetNum(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			const FPlate& Plate = Planet.Plates[PlateIndex];
			const FTectonicPlanetV6PlateSubmesh& Mesh = PlateMeshes[PlateIndex];
			FV6PlateQueryGeometry& QueryGeometry = OutQueryGeometries[PlateIndex];
			ResetV6QueryGeometry(QueryGeometry);
			QueryGeometry.PlateId = Plate.Id;
			QueryGeometry.SoupData.PlateId = Plate.Id;
			QueryGeometry.SoupData.ChangeStamp = 1;

			if (Mesh.LocalTriangles.IsEmpty())
			{
				continue;
			}

			QueryGeometry.SoupData.LocalTriangles = Mesh.LocalTriangles;
			QueryGeometry.SoupData.GlobalTriangleIndices.Init(INDEX_NONE, Mesh.LocalTriangles.Num());
			QueryGeometry.SoupData.LocalToCanonicalVertex = Mesh.LocalToCanonicalVertex;
			QueryGeometry.SoupData.CanonicalToLocalVertex = Mesh.CanonicalToLocalVertex;
			QueryGeometry.LocalTriangleCopiedFrontierFlags = Mesh.LocalTriangleFrontierFlags;
			QueryGeometry.SoupData.RotatedVertices.Reserve(Mesh.BaseVertices.Num());
			for (const FVector3d& BaseVertex : Mesh.BaseVertices)
			{
				QueryGeometry.SoupData.RotatedVertices.Add(Plate.CumulativeRotation.RotateVector(BaseVertex).GetSafeNormal());
			}

			QueryGeometry.SoupData.ChangeStamp++;
			QueryGeometry.SoupAdapter = FPlateTriangleSoupAdapter(&QueryGeometry.SoupData);
			QueryGeometry.SoupBVH.SetMesh(&QueryGeometry.SoupAdapter, true);
			QueryGeometry.BoundingCap = BuildBoundingCapFromVertices(QueryGeometry.SoupData.RotatedVertices);
		}
	}

	void BuildV6QueryGeometries(const FTectonicPlanet& Planet, TArray<FV6PlateQueryGeometry>& OutQueryGeometries)
	{
		OutQueryGeometries.SetNum(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			const FPlate& Plate = Planet.Plates[PlateIndex];
			FV6PlateQueryGeometry& QueryGeometry = OutQueryGeometries[PlateIndex];
			ResetV6QueryGeometry(QueryGeometry);
			QueryGeometry.PlateId = Plate.Id;
			QueryGeometry.SoupData.PlateId = Plate.Id;
			QueryGeometry.SoupData.ChangeStamp = 1;

			const int32 QueryTriangleCount = Plate.InteriorTriangles.Num() + Plate.BoundaryTriangles.Num();
			if (QueryTriangleCount == 0)
			{
				continue;
			}

			QueryGeometry.SoupData.GlobalTriangleIndices.Reserve(QueryTriangleCount);
			QueryGeometry.SoupData.LocalTriangles.Reserve(QueryTriangleCount);
			QueryGeometry.SoupData.RotatedVertices.Reserve(QueryTriangleCount * 3);

			// v6 query geometry duplicates unsplit boundary triangles onto every incident plate.
			AppendTrianglesToV6QueryGeometry(Planet, Plate, Plate.InteriorTriangles, QueryGeometry);
			AppendTrianglesToV6QueryGeometry(Planet, Plate, Plate.BoundaryTriangles, QueryGeometry);
			if (QueryGeometry.SoupData.LocalTriangles.IsEmpty())
			{
				continue;
			}

			QueryGeometry.SoupData.ChangeStamp++;
			QueryGeometry.SoupAdapter = FPlateTriangleSoupAdapter(&QueryGeometry.SoupData);
			QueryGeometry.SoupBVH.SetMesh(&QueryGeometry.SoupAdapter, true);
			QueryGeometry.BoundingCap = BuildBoundingCapFromVertices(QueryGeometry.SoupData.RotatedVertices);
		}
	}

	void BuildV6ThesisRemeshQueryGeometries(const FTectonicPlanet& Planet, TArray<FV6PlateQueryGeometry>& OutQueryGeometries)
	{
		OutQueryGeometries.SetNum(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			const FPlate& Plate = Planet.Plates[PlateIndex];
			FV6PlateQueryGeometry& QueryGeometry = OutQueryGeometries[PlateIndex];
			ResetV6QueryGeometry(QueryGeometry);
			QueryGeometry.PlateId = Plate.Id;
			QueryGeometry.SoupData.PlateId = Plate.Id;
			QueryGeometry.SoupData.ChangeStamp = 1;

			if (Plate.SoupTriangles.IsEmpty())
			{
				continue;
			}

			QueryGeometry.SoupData.GlobalTriangleIndices.Reserve(Plate.SoupTriangles.Num());
			QueryGeometry.SoupData.LocalTriangles.Reserve(Plate.SoupTriangles.Num());
			QueryGeometry.SoupData.RotatedVertices.Reserve(Plate.SoupTriangles.Num() * 3);

			// Thesis-spike query geometry stays aligned with the same per-plate soup that owns the
			// carried crust state. This avoids v6's duplicated boundary triangles where query hits
			// and transfer sources can disagree by construction.
			AppendTrianglesToV6QueryGeometry(Planet, Plate, Plate.SoupTriangles, QueryGeometry);
			if (QueryGeometry.SoupData.LocalTriangles.IsEmpty())
			{
				continue;
			}

			QueryGeometry.SoupData.ChangeStamp++;
			QueryGeometry.SoupAdapter = FPlateTriangleSoupAdapter(&QueryGeometry.SoupData);
			QueryGeometry.SoupBVH.SetMesh(&QueryGeometry.SoupAdapter, true);
			QueryGeometry.BoundingCap = BuildBoundingCapFromVertices(QueryGeometry.SoupData.RotatedVertices);
		}
	}

	int32 PickTransferFallbackCanonicalVertexForTriangle(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 TriangleIndex,
		const FVector3d& Barycentric)
	{
		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr || !Planet.TriangleIndices.IsValidIndex(TriangleIndex))
		{
			return INDEX_NONE;
		}

		const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
		const int32 CanonicalVertices[3] = { Triangle.X, Triangle.Y, Triangle.Z };
		TArray<int32, TInlineAllocator<3>> VertexOrder = { 0, 1, 2 };
		VertexOrder.Sort([&Barycentric, &CanonicalVertices](const int32 Lhs, const int32 Rhs)
		{
			const double LhsWeight = Lhs == 0 ? Barycentric.X : (Lhs == 1 ? Barycentric.Y : Barycentric.Z);
			const double RhsWeight = Rhs == 0 ? Barycentric.X : (Rhs == 1 ? Barycentric.Y : Barycentric.Z);
			if (!FMath::IsNearlyEqual(LhsWeight, RhsWeight, TriangleEpsilon))
			{
				return LhsWeight > RhsWeight;
			}

			return CanonicalVertices[Lhs] < CanonicalVertices[Rhs];
		});

		for (const int32 VertexIndex : VertexOrder)
		{
			const int32 CanonicalSampleIndex = CanonicalVertices[VertexIndex];
			if (Plate->CanonicalToCarriedIndex.Contains(CanonicalSampleIndex))
			{
				return CanonicalSampleIndex;
			}
		}

		return INDEX_NONE;
	}

	int32 PickTransferFallbackCanonicalVertexForLocalTriangle(
		const TArray<int32>& LocalToCanonicalVertex,
		const UE::Geometry::FIndex3i& LocalTriangle,
		const FVector3d& Barycentric)
	{
		if (!LocalToCanonicalVertex.IsValidIndex(LocalTriangle.A) ||
			!LocalToCanonicalVertex.IsValidIndex(LocalTriangle.B) ||
			!LocalToCanonicalVertex.IsValidIndex(LocalTriangle.C))
		{
			return INDEX_NONE;
		}

		const int32 CanonicalVertices[3] = {
			LocalToCanonicalVertex[LocalTriangle.A],
			LocalToCanonicalVertex[LocalTriangle.B],
			LocalToCanonicalVertex[LocalTriangle.C]
		};
		TArray<int32, TInlineAllocator<3>> VertexOrder = { 0, 1, 2 };
		VertexOrder.Sort([&Barycentric, &CanonicalVertices](const int32 Lhs, const int32 Rhs)
		{
			const double LhsWeight = Lhs == 0 ? Barycentric.X : (Lhs == 1 ? Barycentric.Y : Barycentric.Z);
			const double RhsWeight = Rhs == 0 ? Barycentric.X : (Rhs == 1 ? Barycentric.Y : Barycentric.Z);
			if (!FMath::IsNearlyEqual(LhsWeight, RhsWeight, TriangleEpsilon))
			{
				return LhsWeight > RhsWeight;
			}

			return CanonicalVertices[Lhs] < CanonicalVertices[Rhs];
		});

		for (const int32 VertexIndex : VertexOrder)
		{
			if (CanonicalVertices[VertexIndex] != INDEX_NONE)
			{
				return CanonicalVertices[VertexIndex];
			}
		}

		return INDEX_NONE;
	}

	bool FindPlateQueryGeometryIndex(
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const int32 PlateId,
		int32& OutPlateQueryGeometryIndex)
	{
		for (int32 PlateIndex = 0; PlateIndex < QueryGeometries.Num(); ++PlateIndex)
		{
			if (QueryGeometries[PlateIndex].PlateId == PlateId)
			{
				OutPlateQueryGeometryIndex = PlateIndex;
				return true;
			}
		}

		OutPlateQueryGeometryIndex = INDEX_NONE;
		return false;
	}

	bool ChooseExplicitFallbackForPlate(
		const FTectonicPlanet& Planet,
		const int32 PlateId,
		int32& OutSourceSampleIndex)
	{
		OutSourceSampleIndex = INDEX_NONE;
		const FPlate* Plate = FindPlateById(Planet, PlateId);
		if (Plate == nullptr)
		{
			return false;
		}

		for (const int32 MemberSampleIndex : Plate->MemberSamples)
		{
			OutSourceSampleIndex =
				OutSourceSampleIndex == INDEX_NONE ? MemberSampleIndex : FMath::Min(OutSourceSampleIndex, MemberSampleIndex);
		}

		return OutSourceSampleIndex != INDEX_NONE;
	}

	void RefreshResolvedSampleTransferSourceForFinalOwner(
		const FTectonicPlanet& Planet,
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const int32 SampleIndex,
		FTectonicPlanetV6ResolvedSample& InOutResolvedSample)
	{
		InOutResolvedSample.SourceTriangleIndex = INDEX_NONE;
		InOutResolvedSample.SourceLocalTriangleIndex = INDEX_NONE;
		InOutResolvedSample.SourceCanonicalSampleIndex = INDEX_NONE;
		InOutResolvedSample.SourceBarycentric = FVector3d(-1.0, -1.0, -1.0);
		InOutResolvedSample.WinningFitScore = -TNumericLimits<double>::Max();
		InOutResolvedSample.RecoveryDistanceRadians = -1.0;

		if (!Planet.Samples.IsValidIndex(SampleIndex) ||
			InOutResolvedSample.FinalPlateId == INDEX_NONE ||
			InOutResolvedSample.BoundaryOutcome == ETectonicPlanetV6BoundaryOutcome::DivergentOceanic)
		{
			return;
		}

		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		int32 QueryGeometryIndex = INDEX_NONE;
		if (FindPlateQueryGeometryIndex(QueryGeometries, InOutResolvedSample.FinalPlateId, QueryGeometryIndex))
		{
			const FV6PlateQueryGeometry& QueryGeometry = QueryGeometries[QueryGeometryIndex];
			if (QueryGeometry.BoundingCap.Center.IsNearlyZero() ||
				QueryPoint.Dot(QueryGeometry.BoundingCap.Center) >= QueryGeometry.BoundingCap.CosAngle)
			{
				int32 LocalTriangleId = INDEX_NONE;
				FVector3d A;
				FVector3d B;
				FVector3d C;
				if (FindContainingTriangleInBVH(QueryGeometry.SoupBVH, QueryGeometry.SoupAdapter, QueryPoint, LocalTriangleId, A, B, C))
				{
					InOutResolvedSample.SourceTriangleIndex =
						QueryGeometry.SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId)
							? QueryGeometry.SoupData.GlobalTriangleIndices[LocalTriangleId]
							: INDEX_NONE;
					InOutResolvedSample.SourceLocalTriangleIndex = LocalTriangleId;
					InOutResolvedSample.SourceBarycentric = NormalizeBarycentric(ComputePlanarBarycentric(A, B, C, QueryPoint));
					InOutResolvedSample.WinningFitScore = ComputeContainmentScore(InOutResolvedSample.SourceBarycentric);
					InOutResolvedSample.SourceCanonicalSampleIndex = PickTransferFallbackCanonicalVertexForTriangle(
						Planet,
						InOutResolvedSample.FinalPlateId,
						InOutResolvedSample.SourceTriangleIndex,
						InOutResolvedSample.SourceBarycentric);
					if (InOutResolvedSample.SourceCanonicalSampleIndex != INDEX_NONE)
					{
						return;
					}
				}
			}

			FRecoveredContainmentHit RecoveryHit;
			if (TryFindNearestTriangleRecoveryHit(QueryGeometry, QueryPoint, RecoveryHit))
			{
				InOutResolvedSample.SourceTriangleIndex = RecoveryHit.GlobalTriangleIndex;
				InOutResolvedSample.SourceLocalTriangleIndex = RecoveryHit.LocalTriangleIndex;
				InOutResolvedSample.SourceBarycentric = RecoveryHit.Barycentric;
				InOutResolvedSample.RecoveryDistanceRadians = RecoveryHit.Distance;
				InOutResolvedSample.SourceCanonicalSampleIndex = PickTransferFallbackCanonicalVertexForTriangle(
					Planet,
					InOutResolvedSample.FinalPlateId,
					RecoveryHit.GlobalTriangleIndex,
					RecoveryHit.Barycentric);
				if (InOutResolvedSample.SourceCanonicalSampleIndex != INDEX_NONE)
				{
					return;
				}
			}
		}

		double NearestMemberDistanceRadians = TNumericLimits<double>::Max();
		if (FindNearestMemberSample(
				Planet,
				InOutResolvedSample.FinalPlateId,
				QueryPoint,
				InOutResolvedSample.SourceCanonicalSampleIndex,
				NearestMemberDistanceRadians))
		{
			InOutResolvedSample.RecoveryDistanceRadians = NearestMemberDistanceRadians;
			return;
		}

		ChooseExplicitFallbackForPlate(Planet, InOutResolvedSample.FinalPlateId, InOutResolvedSample.SourceCanonicalSampleIndex);
	}

	ETectonicPlanetV6DirectionalTransferKind ProjectSingleSourceDirection(
		const FVector3d& Direction,
		const FVector3d& SurfaceNormal,
		FVector3d& OutProjectedDirection)
	{
		OutProjectedDirection = ProjectDirectionToSurface(Direction, SurfaceNormal);
		return OutProjectedDirection.SquaredLength() > 0.0
			? ETectonicPlanetV6DirectionalTransferKind::SingleSourceProjection
			: ETectonicPlanetV6DirectionalTransferKind::ZeroVectorFallback;
	}

	void ApplyTransferredAttributesFromTriangle(
		FSample& Sample,
		const FCarriedSample& V0,
		const FCarriedSample& V1,
		const FCarriedSample& V2,
		const FVector3d& RawBarycentric,
		float& OutSubductionDistanceKm,
		float& OutSubductionSpeed,
		FTectonicPlanetV6TransferDebugInfo& OutTransferDebug)
	{
		const FVector3d SurfaceNormal = Sample.Position.GetSafeNormal();
		const FVector3d Barycentric = NormalizeBarycentric(RawBarycentric);
		const int32 DominantIndex = PickDominantBarycentricIndex(Barycentric);
		const FCarriedSample& DominantSource =
			DominantIndex == 1 ? V1 : (DominantIndex == 2 ? V2 : V0);
		const double BarycentricContinentalWeight =
			(Barycentric.X * static_cast<double>(V0.ContinentalWeight)) +
			(Barycentric.Y * static_cast<double>(V1.ContinentalWeight)) +
			(Barycentric.Z * static_cast<double>(V2.ContinentalWeight));

		OutTransferDebug = FTectonicPlanetV6TransferDebugInfo{};
		OutTransferDebug.SourceKind = ETectonicPlanetV6TransferSourceKind::Triangle;
		OutTransferDebug.DominantSourceTriangleCorner = DominantIndex;
		OutTransferDebug.DominantSourceCanonicalSampleIndex = DominantSource.CanonicalSampleIndex;
		OutTransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend =
			(BarycentricContinentalWeight >= 0.5) != (static_cast<double>(DominantSource.ContinentalWeight) >= 0.5);
		Sample.ContinentalWeight = FMath::Clamp(DominantSource.ContinentalWeight, 0.0f, 1.0f);
		Sample.Elevation = FMath::Clamp(
			static_cast<float>(
				(Barycentric.X * V0.Elevation) +
				(Barycentric.Y * V1.Elevation) +
				(Barycentric.Z * V2.Elevation)),
			static_cast<float>(TrenchElevationKm),
			static_cast<float>(ElevationCeilingKm));
		Sample.Thickness = FMath::Max(
			0.0f,
			static_cast<float>(
				(Barycentric.X * V0.Thickness) +
				(Barycentric.Y * V1.Thickness) +
				(Barycentric.Z * V2.Thickness)));
		Sample.Age = FMath::Max(
			0.0f,
			static_cast<float>(
				(Barycentric.X * V0.Age) +
				(Barycentric.Y * V1.Age) +
				(Barycentric.Z * V2.Age)));
		Sample.OrogenyType = ChooseTransferredOrogenyType(
			V0.OrogenyType,
			V1.OrogenyType,
			V2.OrogenyType,
			Barycentric,
			OutTransferDebug.OrogenyTransferKind);
		Sample.TerraneId = ChooseTransferredTerraneId(
			V0.TerraneId,
			V1.TerraneId,
			V2.TerraneId,
			Barycentric,
			OutTransferDebug.TerraneTransferKind);
		Sample.RidgeDirection = TransferDirectionalField(
			V0.RidgeDirection,
			V1.RidgeDirection,
			V2.RidgeDirection,
			Barycentric,
			SurfaceNormal,
			OutTransferDebug.RidgeDirectionTransferKind);
		Sample.FoldDirection = TransferDirectionalField(
			V0.FoldDirection,
			V1.FoldDirection,
			V2.FoldDirection,
			Barycentric,
			SurfaceNormal,
			OutTransferDebug.FoldDirectionTransferKind);
		OutSubductionDistanceKm = static_cast<float>(
			(Barycentric.X * V0.SubductionDistanceKm) +
			(Barycentric.Y * V1.SubductionDistanceKm) +
			(Barycentric.Z * V2.SubductionDistanceKm));
		OutSubductionSpeed = static_cast<float>(
			(Barycentric.X * V0.SubductionSpeed) +
			(Barycentric.Y * V1.SubductionSpeed) +
			(Barycentric.Z * V2.SubductionSpeed));
	}

	void ApplyTransferredAttributesFromSingleSource(
		FSample& Sample,
		const FCarriedSample& Source,
		float& OutSubductionDistanceKm,
		float& OutSubductionSpeed,
		FTectonicPlanetV6TransferDebugInfo& OutTransferDebug)
	{
		const FVector3d SurfaceNormal = Sample.Position.GetSafeNormal();
		OutTransferDebug = FTectonicPlanetV6TransferDebugInfo{};
		OutTransferDebug.SourceKind = ETectonicPlanetV6TransferSourceKind::SingleSource;
		OutTransferDebug.DominantSourceCanonicalSampleIndex = Source.CanonicalSampleIndex;
		OutTransferDebug.TerraneTransferKind = ETectonicPlanetV6CategoricalTransferKind::DominantSource;
		OutTransferDebug.OrogenyTransferKind = ETectonicPlanetV6CategoricalTransferKind::DominantSource;
		Sample.ContinentalWeight = FMath::Clamp(Source.ContinentalWeight, 0.0f, 1.0f);
		Sample.Elevation = FMath::Clamp(Source.Elevation, static_cast<float>(TrenchElevationKm), static_cast<float>(ElevationCeilingKm));
		Sample.Thickness = FMath::Max(0.0f, Source.Thickness);
		Sample.Age = FMath::Max(0.0f, Source.Age);
		Sample.OrogenyType = Source.OrogenyType;
		Sample.TerraneId = Source.TerraneId;
		OutTransferDebug.RidgeDirectionTransferKind = ProjectSingleSourceDirection(
			Source.RidgeDirection,
			SurfaceNormal,
			Sample.RidgeDirection);
		OutTransferDebug.FoldDirectionTransferKind = ProjectSingleSourceDirection(
			Source.FoldDirection,
			SurfaceNormal,
			Sample.FoldDirection);
		OutSubductionDistanceKm = Source.SubductionDistanceKm;
		OutSubductionSpeed = Source.SubductionSpeed;
	}

	void ApplyOceanicBoundaryCreation(
		FSample& Sample,
		const FTectonicPlanet& Planet,
		const FTectonicPlanetV6ResolvedSample& Resolved,
		float& OutSubductionDistanceKm,
		float& OutSubductionSpeed,
		FTectonicPlanetV6TransferDebugInfo& OutTransferDebug)
	{
		OutTransferDebug = FTectonicPlanetV6TransferDebugInfo{};
		OutTransferDebug.SourceKind = ETectonicPlanetV6TransferSourceKind::OceanicCreation;
		Sample.ContinentalWeight = 0.0f;
		Sample.Elevation = static_cast<float>(RidgeElevationKm);
		Sample.Thickness = static_cast<float>(OceanicThicknessKm);
		Sample.Age = 0.0f;
		Sample.OrogenyType = EOrogenyType::None;
		Sample.TerraneId = INDEX_NONE;
		Sample.FoldDirection = FVector3d::ZeroVector;
		OutSubductionDistanceKm = -1.0f;
		OutSubductionSpeed = 0.0f;

		const FPlate* WinningPlate = FindPlateById(Planet, Resolved.FinalPlateId);
		const FPlate* OtherPlate = FindPlateById(Planet, Resolved.BoundaryOtherPlateId);
		if (WinningPlate != nullptr && OtherPlate != nullptr)
		{
			const FVector3d RelativeVelocity =
				ComputePlateSurfaceVelocity(*WinningPlate, Sample.Position, Planet.PlanetRadiusKm) -
				ComputePlateSurfaceVelocity(*OtherPlate, Sample.Position, Planet.PlanetRadiusKm);
			Sample.RidgeDirection = ProjectDirectionToSurface(RelativeVelocity, Sample.Position.GetSafeNormal());
			return;
		}

		Sample.RidgeDirection = FVector3d::ZeroVector;
	}

	FCarriedSample BuildInterpolatedTransferredCarriedSample(
		const FCarriedSample& V0,
		const FCarriedSample& V1,
		const FCarriedSample& V2,
		const FVector3d& RawBarycentric,
		const FVector3d& SurfacePoint)
	{
		FCarriedSample Result;
		const FVector3d SurfaceNormal = SurfacePoint.GetSafeNormal();
		const FVector3d Barycentric = NormalizeBarycentric(RawBarycentric);
		const int32 DominantIndex = PickDominantBarycentricIndex(Barycentric);
		const FCarriedSample& DominantSource =
			DominantIndex == 1 ? V1 : (DominantIndex == 2 ? V2 : V0);
		Result.CanonicalSampleIndex = DominantSource.CanonicalSampleIndex;
		Result.ContinentalWeight = FMath::Clamp(DominantSource.ContinentalWeight, 0.0f, 1.0f);
		Result.Elevation = FMath::Clamp(
			static_cast<float>(
				(Barycentric.X * V0.Elevation) +
				(Barycentric.Y * V1.Elevation) +
				(Barycentric.Z * V2.Elevation)),
			static_cast<float>(TrenchElevationKm),
			static_cast<float>(ElevationCeilingKm));
		Result.Thickness = FMath::Max(
			0.0f,
			static_cast<float>(
				(Barycentric.X * V0.Thickness) +
				(Barycentric.Y * V1.Thickness) +
				(Barycentric.Z * V2.Thickness)));
		Result.Age = FMath::Max(
			0.0f,
			static_cast<float>(
				(Barycentric.X * V0.Age) +
				(Barycentric.Y * V1.Age) +
				(Barycentric.Z * V2.Age)));
		ETectonicPlanetV6CategoricalTransferKind OrogenyTransferKind = ETectonicPlanetV6CategoricalTransferKind::None;
		Result.OrogenyType = ChooseTransferredOrogenyType(
			V0.OrogenyType,
			V1.OrogenyType,
			V2.OrogenyType,
			Barycentric,
			OrogenyTransferKind);
		ETectonicPlanetV6CategoricalTransferKind TerraneTransferKind = ETectonicPlanetV6CategoricalTransferKind::None;
		Result.TerraneId = ChooseTransferredTerraneId(
			V0.TerraneId,
			V1.TerraneId,
			V2.TerraneId,
			Barycentric,
			TerraneTransferKind);
		ETectonicPlanetV6DirectionalTransferKind RidgeTransferKind = ETectonicPlanetV6DirectionalTransferKind::None;
		Result.RidgeDirection = TransferDirectionalField(
			V0.RidgeDirection,
			V1.RidgeDirection,
			V2.RidgeDirection,
			Barycentric,
			SurfaceNormal,
			RidgeTransferKind);
		ETectonicPlanetV6DirectionalTransferKind FoldTransferKind = ETectonicPlanetV6DirectionalTransferKind::None;
		Result.FoldDirection = TransferDirectionalField(
			V0.FoldDirection,
			V1.FoldDirection,
			V2.FoldDirection,
			Barycentric,
			SurfaceNormal,
			FoldTransferKind);
		Result.SubductionDistanceKm = static_cast<float>(
			(Barycentric.X * V0.SubductionDistanceKm) +
			(Barycentric.Y * V1.SubductionDistanceKm) +
			(Barycentric.Z * V2.SubductionDistanceKm));
		Result.SubductionSpeed = static_cast<float>(
			(Barycentric.X * V0.SubductionSpeed) +
			(Barycentric.Y * V1.SubductionSpeed) +
			(Barycentric.Z * V2.SubductionSpeed));
		return Result;
	}

	bool TryBuildRecoveryCandidateInterpolatedSample(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const FTectonicPlanetV6RecoveryCandidate& RecoveryCandidate,
		FCarriedSample& OutSample,
		FVector3d& OutSurfacePoint)
	{
		OutSample = FCarriedSample{};
		OutSurfacePoint = FVector3d::ZeroVector;
		const int32 PlateIndex = Planet.FindPlateArrayIndexById(RecoveryCandidate.PlateId);
		if (!PlateMeshes.IsValidIndex(PlateIndex) ||
			!QueryGeometries.IsValidIndex(PlateIndex) ||
			!Planet.Plates.IsValidIndex(PlateIndex))
		{
			return false;
		}

		const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = PlateMeshes[PlateIndex];
		const FV6PlateQueryGeometry& QueryGeometry = QueryGeometries[PlateIndex];
		int32 LocalTriangleIndex = RecoveryCandidate.LocalTriangleIndex;
		if (!Mesh.LocalTriangles.IsValidIndex(LocalTriangleIndex) &&
			RecoveryCandidate.TriangleIndex != INDEX_NONE)
		{
			if (const int32* LocalTriangleIndexPtr = Mesh.GlobalToLocalTriangle.Find(RecoveryCandidate.TriangleIndex))
			{
				LocalTriangleIndex = *LocalTriangleIndexPtr;
			}
		}
		if (!Mesh.LocalTriangles.IsValidIndex(LocalTriangleIndex) ||
			!QueryGeometry.SoupData.LocalTriangles.IsValidIndex(LocalTriangleIndex))
		{
			return false;
		}

		const UE::Geometry::FIndex3i& LocalTriangle = Mesh.LocalTriangles[LocalTriangleIndex];
		if (!Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.A) ||
			!Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.B) ||
			!Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.C) ||
			!QueryGeometry.SoupData.RotatedVertices.IsValidIndex(LocalTriangle.A) ||
			!QueryGeometry.SoupData.RotatedVertices.IsValidIndex(LocalTriangle.B) ||
			!QueryGeometry.SoupData.RotatedVertices.IsValidIndex(LocalTriangle.C))
		{
			return false;
		}

		const FVector3d Barycentric = NormalizeBarycentric(RecoveryCandidate.Barycentric);
		const FVector3d& QueryA = QueryGeometry.SoupData.RotatedVertices[LocalTriangle.A];
		const FVector3d& QueryB = QueryGeometry.SoupData.RotatedVertices[LocalTriangle.B];
		const FVector3d& QueryC = QueryGeometry.SoupData.RotatedVertices[LocalTriangle.C];
		OutSurfacePoint =
			((QueryA * Barycentric.X) +
				(QueryB * Barycentric.Y) +
				(QueryC * Barycentric.Z)).GetSafeNormal();
		if (OutSurfacePoint.IsNearlyZero())
		{
			return false;
		}

		const FPlate& Plate = Planet.Plates[PlateIndex];
		const FCarriedSample V0 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.A]);
		const FCarriedSample V1 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.B]);
		const FCarriedSample V2 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.C]);
		OutSample = BuildInterpolatedTransferredCarriedSample(V0, V1, V2, Barycentric, OutSurfacePoint);
		return true;
	}

	const FTectonicPlanetV6RecoveryCandidate* FindBestRecoveryCandidateForOtherPlate(
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const int32 ExcludedPlateId)
	{
		const FTectonicPlanetV6RecoveryCandidate* BestCandidate = nullptr;
		for (const FTectonicPlanetV6RecoveryCandidate& Candidate : RecoveryCandidates)
		{
			if (Candidate.PlateId == INDEX_NONE || Candidate.PlateId == ExcludedPlateId)
			{
				continue;
			}

			if (BestCandidate == nullptr ||
				Candidate.DistanceRadians + TriangleEpsilon < BestCandidate->DistanceRadians ||
				(FMath::IsNearlyEqual(Candidate.DistanceRadians, BestCandidate->DistanceRadians, TriangleEpsilon) &&
					Candidate.PlateId < BestCandidate->PlateId))
			{
				BestCandidate = &Candidate;
			}
		}
		return BestCandidate;
	}

	bool ShouldRetainPreviousSyntheticCoverageForMiss(
		const FTectonicPlanet& Planet,
		const int32 SampleIndex,
		const int32 PreviousPlateId,
		const bool bPreviousSynthetic,
		const bool bPreviousRetainedSyntheticCoverage,
		const bool bPreviousOwnerAdjacentTriangleInMeshPresent,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates)
	{
		if (!bPreviousSynthetic ||
			bPreviousRetainedSyntheticCoverage ||
			PreviousPlateId == INDEX_NONE ||
			!bPreviousOwnerAdjacentTriangleInMeshPresent)
		{
			return false;
		}

		if (FindCarriedSampleForCanonicalVertex(Planet, PreviousPlateId, SampleIndex) == nullptr)
		{
			return false;
		}

		const FTectonicPlanetV6RecoveryCandidate* PreviousRecovery =
			FindBestRecoveryCandidateForPlate(RecoveryCandidates, PreviousPlateId);
		const FTectonicPlanetV6RecoveryCandidate* OtherRecovery =
			FindBestRecoveryCandidateForOtherPlate(RecoveryCandidates, PreviousPlateId);
		if (OtherRecovery == nullptr)
		{
			return true;
		}

		return PreviousRecovery != nullptr &&
			PreviousRecovery->DistanceRadians <= OtherRecovery->DistanceRadians + BoundaryRecoverySlackRadians;
	}

	const FV6FrontierPoint* FindNearestFrontierPointForPlateSet(
		const FV6PlateFrontierPointSet& FrontierPointSet,
		const FVector3d& QueryPoint,
		double& OutDistanceRadians)
	{
		OutDistanceRadians = TNumericLimits<double>::Max();
		const FV6FrontierPoint* BestPoint = nullptr;
		double BestDot = -2.0;

		for (const FV6FrontierPoint& FrontierPoint : FrontierPointSet.Points)
		{
			const double CandidateDot = QueryPoint.Dot(FrontierPoint.Position);
			if (BestPoint == nullptr ||
				CandidateDot > BestDot + TriangleEpsilon ||
				(FMath::IsNearlyEqual(CandidateDot, BestDot, TriangleEpsilon) &&
					FrontierPoint.LocalVertexIndex < BestPoint->LocalVertexIndex))
			{
				BestPoint = &FrontierPoint;
				BestDot = CandidateDot;
			}
		}

		if (BestPoint != nullptr)
		{
			OutDistanceRadians = ComputeGeodesicDistance(QueryPoint, BestPoint->Position);
		}
		return BestPoint;
	}

	bool TryFindNearestStructuredFrontierPair(
		const TArray<FV6PlateFrontierPointSet>& FrontierPointSets,
		const FVector3d& QueryPoint,
		const int32 PreferredPrimaryPlateId,
		const int32 PreferredSecondaryPlateId,
		const FV6FrontierPoint*& OutPrimaryPoint,
		const FV6FrontierPoint*& OutSecondaryPoint,
		double& OutPrimaryDistanceRadians,
		double& OutSecondaryDistanceRadians)
	{
		OutPrimaryPoint = nullptr;
		OutSecondaryPoint = nullptr;
		OutPrimaryDistanceRadians = TNumericLimits<double>::Max();
		OutSecondaryDistanceRadians = TNumericLimits<double>::Max();

		for (const FV6PlateFrontierPointSet& FrontierPointSet : FrontierPointSets)
		{
			double CandidateDistanceRadians = TNumericLimits<double>::Max();
			const FV6FrontierPoint* CandidatePoint =
				FindNearestFrontierPointForPlateSet(FrontierPointSet, QueryPoint, CandidateDistanceRadians);
			if (CandidatePoint == nullptr)
			{
				continue;
			}

			const bool bBeatsPrimary =
				OutPrimaryPoint == nullptr ||
				CandidateDistanceRadians + TriangleEpsilon < OutPrimaryDistanceRadians ||
				(FMath::IsNearlyEqual(CandidateDistanceRadians, OutPrimaryDistanceRadians, TriangleEpsilon) &&
					(PreferredPrimaryPlateId != INDEX_NONE && CandidatePoint->PlateId == PreferredPrimaryPlateId &&
						OutPrimaryPoint->PlateId != PreferredPrimaryPlateId)) ||
				(FMath::IsNearlyEqual(CandidateDistanceRadians, OutPrimaryDistanceRadians, TriangleEpsilon) &&
					CandidatePoint->PlateId < OutPrimaryPoint->PlateId);
			if (bBeatsPrimary)
			{
				if (OutPrimaryPoint != nullptr && OutPrimaryPoint->PlateId != CandidatePoint->PlateId)
				{
					OutSecondaryPoint = OutPrimaryPoint;
					OutSecondaryDistanceRadians = OutPrimaryDistanceRadians;
				}

				OutPrimaryPoint = CandidatePoint;
				OutPrimaryDistanceRadians = CandidateDistanceRadians;
				continue;
			}

			if (OutPrimaryPoint != nullptr && CandidatePoint->PlateId == OutPrimaryPoint->PlateId)
			{
				continue;
			}

			const bool bBeatsSecondary =
				OutSecondaryPoint == nullptr ||
				CandidateDistanceRadians + TriangleEpsilon < OutSecondaryDistanceRadians ||
				(FMath::IsNearlyEqual(CandidateDistanceRadians, OutSecondaryDistanceRadians, TriangleEpsilon) &&
					(PreferredSecondaryPlateId != INDEX_NONE && CandidatePoint->PlateId == PreferredSecondaryPlateId &&
						OutSecondaryPoint->PlateId != PreferredSecondaryPlateId)) ||
				(FMath::IsNearlyEqual(CandidateDistanceRadians, OutSecondaryDistanceRadians, TriangleEpsilon) &&
					CandidatePoint->PlateId < OutSecondaryPoint->PlateId);
			if (bBeatsSecondary)
			{
				OutSecondaryPoint = CandidatePoint;
				OutSecondaryDistanceRadians = CandidateDistanceRadians;
			}
		}

		return OutPrimaryPoint != nullptr &&
			OutSecondaryPoint != nullptr &&
			OutPrimaryPoint->PlateId != OutSecondaryPoint->PlateId;
	}

	double ComputeStructuredGapCenterBlend(
		const FVector3d& QueryPoint,
		const FVector3d& PointA,
		const FVector3d& PointB)
	{
		const FVector3d Midpoint = (PointA + PointB).GetSafeNormal();
		if (Midpoint.IsNearlyZero())
		{
			return 0.0;
		}

		const double DistanceToMidpoint = ComputeGeodesicDistance(QueryPoint, Midpoint);
		const double DistanceToA = ComputeGeodesicDistance(QueryPoint, PointA);
		const double DistanceToB = ComputeGeodesicDistance(QueryPoint, PointB);
		const double Scale = FMath::Max(0.5 * (DistanceToA + DistanceToB), StructuredGapBlendFloorRadians);
		return FMath::Clamp(1.0 - (DistanceToMidpoint / Scale), 0.0, 1.0);
	}

	FVector3d ComputeStructuredGapAxisDirection(
		const FVector3d& SurfaceNormal,
		const FVector3d& PointA,
		const FVector3d& PointB,
		const FVector3d& FallbackDirection)
	{
		const FVector3d AxisDirection = ProjectDirectionToSurface(PointB - PointA, SurfaceNormal);
		if (AxisDirection.SquaredLength() > 0.0)
		{
			return AxisDirection;
		}

		const FVector3d ProjectedFallback = ProjectDirectionToSurface(FallbackDirection, SurfaceNormal);
		return ProjectedFallback.SquaredLength() > 0.0 ? ProjectedFallback : FVector3d::ZeroVector;
	}

	double ComputeStructuredFrontierPairRidgeBlend(
		const FVector3d& QueryPoint,
		const FVector3d& FrontierPointA,
		const FVector3d& FrontierPointB,
		const FVector3d& RidgePoint)
	{
		const double DistanceToRidgeRadians = ComputeGeodesicDistance(QueryPoint, RidgePoint);
		const double DistanceToFrontierRadians = FMath::Min(
			ComputeGeodesicDistance(QueryPoint, FrontierPointA),
			ComputeGeodesicDistance(QueryPoint, FrontierPointB));
		const double Denominator =
			FMath::Max(DistanceToRidgeRadians + DistanceToFrontierRadians, StructuredGapBlendFloorRadians);
		const double RidgeBlend = FMath::Clamp(DistanceToFrontierRadians / Denominator, 0.0, 1.0);
		return RidgeBlend * RidgeBlend * (3.0 - (2.0 * RidgeBlend));
	}

	bool TryBuildStructuredFrontierPairFill(
		const TArray<FV6PlateFrontierPointSet>& FrontierPointSets,
		const FVector3d& QueryPoint,
		const int32 PreferredPrimaryPlateId,
		const int32 PreferredSecondaryPlateId,
		FCarriedSample& OutSyntheticSample,
		int32& OutPrimaryPlateId,
		int32& OutSecondaryPlateId)
	{
		OutSyntheticSample = FCarriedSample{};
		OutPrimaryPlateId = INDEX_NONE;
		OutSecondaryPlateId = INDEX_NONE;

		const FV6FrontierPoint* PrimaryPoint = nullptr;
		const FV6FrontierPoint* SecondaryPoint = nullptr;
		double PrimaryDistanceRadians = TNumericLimits<double>::Max();
		double SecondaryDistanceRadians = TNumericLimits<double>::Max();
		if (!TryFindNearestStructuredFrontierPair(
				FrontierPointSets,
				QueryPoint,
				PreferredPrimaryPlateId,
				PreferredSecondaryPlateId,
				PrimaryPoint,
				SecondaryPoint,
				PrimaryDistanceRadians,
				SecondaryDistanceRadians) ||
			PrimaryPoint == nullptr ||
			SecondaryPoint == nullptr)
		{
			return false;
		}

		const FVector3d RidgePoint = (PrimaryPoint->Position + SecondaryPoint->Position).GetSafeNormal();
		if (RidgePoint.IsNearlyZero())
		{
			return false;
		}

		const FVector3d SurfaceNormal = QueryPoint.GetSafeNormal();
		const double RidgeBlend = ComputeStructuredFrontierPairRidgeBlend(
			QueryPoint,
			PrimaryPoint->Position,
			SecondaryPoint->Position,
			RidgePoint);
		const float BorderElevation = static_cast<float>(
			0.5 * (PrimaryPoint->CarriedSample.Elevation + SecondaryPoint->CarriedSample.Elevation));
		const FVector3d FallbackDirection =
			PrimaryPoint->CarriedSample.RidgeDirection + SecondaryPoint->CarriedSample.RidgeDirection;

		OutSyntheticSample.CanonicalSampleIndex =
			PrimaryPoint->CarriedSample.CanonicalSampleIndex != INDEX_NONE
				? PrimaryPoint->CarriedSample.CanonicalSampleIndex
				: SecondaryPoint->CarriedSample.CanonicalSampleIndex;
		OutSyntheticSample.ContinentalWeight = 0.0f;
		OutSyntheticSample.Elevation = FMath::Clamp(
			FMath::Lerp(BorderElevation, static_cast<float>(RidgeElevationKm), static_cast<float>(RidgeBlend)),
			static_cast<float>(TrenchElevationKm),
			static_cast<float>(ElevationCeilingKm));
		OutSyntheticSample.Thickness = static_cast<float>(OceanicThicknessKm);
		OutSyntheticSample.Age = 0.0f;
		OutSyntheticSample.OrogenyType = EOrogenyType::None;
		OutSyntheticSample.TerraneId = INDEX_NONE;
		OutSyntheticSample.RidgeDirection = ComputeStructuredGapAxisDirection(
			SurfaceNormal,
			PrimaryPoint->Position,
			SecondaryPoint->Position,
			FallbackDirection);
		OutSyntheticSample.FoldDirection = FVector3d::ZeroVector;
		OutSyntheticSample.SubductionDistanceKm = -1.0f;
		OutSyntheticSample.SubductionSpeed = 0.0f;

		OutPrimaryPlateId = PrimaryPoint->PlateId;
		OutSecondaryPlateId = SecondaryPoint->PlateId;
		return true;
	}

	bool TryBuildStructuredTrueDivergenceFill(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const FVector3d& QueryPoint,
		const FTectonicPlanetV6RecoveryCandidate& PrimaryRecoveryCandidate,
		const FTectonicPlanetV6RecoveryCandidate& SecondaryRecoveryCandidate,
		FCarriedSample& OutSyntheticSample)
	{
		FCarriedSample PrimarySample;
		FCarriedSample SecondarySample;
		FVector3d PrimaryPoint;
		FVector3d SecondaryPoint;
		if (!TryBuildRecoveryCandidateInterpolatedSample(
				Planet,
				PlateMeshes,
				QueryGeometries,
				PrimaryRecoveryCandidate,
				PrimarySample,
				PrimaryPoint) ||
			!TryBuildRecoveryCandidateInterpolatedSample(
				Planet,
				PlateMeshes,
				QueryGeometries,
				SecondaryRecoveryCandidate,
				SecondarySample,
				SecondaryPoint))
		{
			return false;
		}

		const FVector3d SurfaceNormal = QueryPoint.GetSafeNormal();
		const double CenterBlend = ComputeStructuredGapCenterBlend(QueryPoint, PrimaryPoint, SecondaryPoint);
		const float BorderElevation = static_cast<float>(0.5 * (PrimarySample.Elevation + SecondarySample.Elevation));
		OutSyntheticSample = FCarriedSample{};
		OutSyntheticSample.CanonicalSampleIndex = PrimarySample.CanonicalSampleIndex;
		OutSyntheticSample.ContinentalWeight = 0.0f;
		OutSyntheticSample.Elevation = FMath::Clamp(
			FMath::Lerp(BorderElevation, static_cast<float>(RidgeElevationKm), static_cast<float>(CenterBlend)),
			static_cast<float>(TrenchElevationKm),
			static_cast<float>(ElevationCeilingKm));
		OutSyntheticSample.Thickness = static_cast<float>(OceanicThicknessKm);
		OutSyntheticSample.Age = 0.0f;
		OutSyntheticSample.OrogenyType = EOrogenyType::None;
		OutSyntheticSample.TerraneId = INDEX_NONE;
		OutSyntheticSample.RidgeDirection = ComputeStructuredGapAxisDirection(
			SurfaceNormal,
			PrimaryPoint,
			SecondaryPoint,
			PrimarySample.RidgeDirection + SecondarySample.RidgeDirection);
		OutSyntheticSample.FoldDirection = FVector3d::ZeroVector;
		OutSyntheticSample.SubductionDistanceKm = -1.0f;
		OutSyntheticSample.SubductionSpeed = 0.0f;
		return true;
	}

	bool ShouldUseDestructiveGapContinuity(
		const FTectonicPlanet& Planet,
		const int32 PreferredPlateId,
		const int32 PreviousPlateId,
		const FTectonicPlanetV6RecoveryCandidate* PreferredRecoveryCandidate,
		const FTectonicPlanetV6RecoveryCandidate* SecondaryRecoveryCandidate)
	{
		if (PreferredPlateId == INDEX_NONE ||
			PreferredPlateId != PreviousPlateId ||
			PreferredRecoveryCandidate == nullptr)
		{
			return false;
		}

		const double PreferredDistanceKm = PreferredRecoveryCandidate->DistanceRadians * Planet.PlanetRadiusKm;
		if (PreferredDistanceKm > DestructiveGapContinuityRecoveryDistanceKm)
		{
			return false;
		}

		if (SecondaryRecoveryCandidate == nullptr)
		{
			return true;
		}

		const double SecondaryDistanceKm = SecondaryRecoveryCandidate->DistanceRadians * Planet.PlanetRadiusKm;
		return PreferredDistanceKm + DestructiveGapContinuityDominanceSlackKm < SecondaryDistanceKm;
	}

	bool TryBuildStructuredDestructiveSyntheticFill(
		const FTectonicPlanet& Planet,
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& PlateMeshes,
		const TArray<FV6PlateQueryGeometry>& QueryGeometries,
		const FVector3d& QueryPoint,
		const FTectonicPlanetV6RecoveryCandidate& PreferredRecoveryCandidate,
		const FTectonicPlanetV6RecoveryCandidate* SecondaryRecoveryCandidate,
		FCarriedSample& OutSyntheticSample)
	{
		FCarriedSample PreferredSample;
		FVector3d PreferredPoint;
		if (!TryBuildRecoveryCandidateInterpolatedSample(
				Planet,
				PlateMeshes,
				QueryGeometries,
				PreferredRecoveryCandidate,
				PreferredSample,
				PreferredPoint))
		{
			return false;
		}

		FCarriedSample SecondarySample = PreferredSample;
		FVector3d SecondaryPoint = PreferredPoint;
		const bool bHasSecondary =
			SecondaryRecoveryCandidate != nullptr &&
			TryBuildRecoveryCandidateInterpolatedSample(
				Planet,
				PlateMeshes,
				QueryGeometries,
				*SecondaryRecoveryCandidate,
				SecondarySample,
				SecondaryPoint);
		const FVector3d SurfaceNormal = QueryPoint.GetSafeNormal();
		const double CenterBlend =
			bHasSecondary
				? ComputeStructuredGapCenterBlend(QueryPoint, PreferredPoint, SecondaryPoint)
				: 0.5;
		const float BorderElevation = bHasSecondary
			? static_cast<float>(0.5 * (PreferredSample.Elevation + SecondarySample.Elevation))
			: PreferredSample.Elevation;
		const float BorderThickness = bHasSecondary
			? static_cast<float>(0.5 * (PreferredSample.Thickness + SecondarySample.Thickness))
			: PreferredSample.Thickness;
		const float BorderAge = bHasSecondary
			? static_cast<float>(0.5 * (PreferredSample.Age + SecondarySample.Age))
			: PreferredSample.Age;
		const float BorderContinentalWeight = bHasSecondary
			? static_cast<float>(0.5 * (PreferredSample.ContinentalWeight + SecondarySample.ContinentalWeight))
			: PreferredSample.ContinentalWeight;
		const float PreservedContinentalWeight = FMath::Clamp(
			FMath::Max(PreferredSample.ContinentalWeight, BorderContinentalWeight) *
				(1.0f - (0.85f * static_cast<float>(CenterBlend))),
			0.0f,
			1.0f);

		OutSyntheticSample = FCarriedSample{};
		OutSyntheticSample.CanonicalSampleIndex = PreferredSample.CanonicalSampleIndex;
		OutSyntheticSample.ContinentalWeight = PreservedContinentalWeight;
		OutSyntheticSample.Elevation = FMath::Clamp(
			FMath::Lerp(
				BorderElevation,
				static_cast<float>(TrenchElevationKm),
				static_cast<float>(0.35 + (0.65 * CenterBlend))),
			static_cast<float>(TrenchElevationKm),
			static_cast<float>(ElevationCeilingKm));
		OutSyntheticSample.Thickness = FMath::Max(
			0.0f,
			FMath::Lerp(
				BorderThickness,
				static_cast<float>(OceanicThicknessKm),
				static_cast<float>(0.2 + (0.6 * CenterBlend))));
		OutSyntheticSample.Age = FMath::Max(
			0.0f,
			FMath::Lerp(BorderAge, 0.0f, static_cast<float>(0.5 * CenterBlend)));
		OutSyntheticSample.OrogenyType =
			PreservedContinentalWeight >= 0.5f ? PreferredSample.OrogenyType : EOrogenyType::None;
		OutSyntheticSample.TerraneId =
			PreservedContinentalWeight >= 0.5f ? PreferredSample.TerraneId : INDEX_NONE;
		OutSyntheticSample.RidgeDirection = ComputeStructuredGapAxisDirection(
			SurfaceNormal,
			PreferredPoint,
			SecondaryPoint,
			PreferredSample.RidgeDirection);
		OutSyntheticSample.FoldDirection =
			PreservedContinentalWeight >= 0.5f
				? ProjectDirectionToSurface(PreferredSample.FoldDirection, SurfaceNormal)
				: FVector3d::ZeroVector;
		OutSyntheticSample.SubductionDistanceKm = PreferredSample.SubductionDistanceKm;
		OutSyntheticSample.SubductionSpeed = PreferredSample.SubductionSpeed;
		return true;
	}

	void ApplyStructuredSyntheticGapFill(
		FSample& Sample,
		const FCarriedSample& SyntheticSample,
		float& OutSubductionDistanceKm,
		float& OutSubductionSpeed,
		FTectonicPlanetV6TransferDebugInfo& OutTransferDebug)
	{
		ApplyTransferredAttributesFromSingleSource(
			Sample,
			SyntheticSample,
			OutSubductionDistanceKm,
			OutSubductionSpeed,
			OutTransferDebug);
		OutTransferDebug.SourceKind = ETectonicPlanetV6TransferSourceKind::StructuredSynthetic;
	}

	void IncrementTransferResolutionCounts(
		FTectonicPlanetV6TransferResolutionCounts& InOutCounts,
		const FTectonicPlanetV6ResolvedSample& ResolvedSample)
	{
		if (ResolvedSample.bCoherenceReassigned)
		{
			++InOutCounts.CoherenceReassigned;
			return;
		}

		switch (ResolvedSample.ResolutionKind)
		{
		case ETectonicPlanetV6ResolutionKind::SingleCandidate:
			++InOutCounts.ExactSingleHit;
			return;
		case ETectonicPlanetV6ResolutionKind::OverlapWinner:
			++InOutCounts.OverlapWinner;
			return;
		case ETectonicPlanetV6ResolutionKind::BoundaryRetained:
			++InOutCounts.BoundaryRetained;
			return;
		case ETectonicPlanetV6ResolutionKind::BoundaryReassigned:
			++InOutCounts.BoundaryReassigned;
			return;
		case ETectonicPlanetV6ResolutionKind::BoundaryOceanic:
			++InOutCounts.BoundaryOceanic;
			return;
		case ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery:
			++InOutCounts.TriangleRecovery;
			return;
		case ETectonicPlanetV6ResolutionKind::NearestMemberRecovery:
		case ETectonicPlanetV6ResolutionKind::ExplicitFallback:
		case ETectonicPlanetV6ResolutionKind::None:
		default:
			++InOutCounts.Other;
			return;
		}
	}

	bool UsesCategoricalMajorityTransfer(const FTectonicPlanetV6TransferDebugInfo& TransferDebug)
	{
		return TransferDebug.TerraneTransferKind == ETectonicPlanetV6CategoricalTransferKind::MajorityVote ||
			TransferDebug.OrogenyTransferKind == ETectonicPlanetV6CategoricalTransferKind::MajorityVote;
	}

	bool UsesDirectionalFallback(const FTectonicPlanetV6TransferDebugInfo& TransferDebug)
	{
		auto UsesFallback = [](const ETectonicPlanetV6DirectionalTransferKind Kind)
		{
			return Kind == ETectonicPlanetV6DirectionalTransferKind::DominantSourceFallback ||
				Kind == ETectonicPlanetV6DirectionalTransferKind::ZeroVectorFallback;
		};

		return UsesFallback(TransferDebug.RidgeDirectionTransferKind) ||
			UsesFallback(TransferDebug.FoldDirectionTransferKind);
	}

	bool ChooseExplicitFallback(
		const FTectonicPlanet& Planet,
		int32& OutPlateId,
		int32& OutSourceSampleIndex);

	struct FV6CopiedFrontierZeroHitResolvedDecisionStageContext
	{
		const FTectonicPlanet* Planet = nullptr;
		const TArray<FV6PlateFrontierPointSet>* FrontierPointSets = nullptr;
		const FV6CopiedFrontierDestructiveFilterState* DestructiveFilterState = nullptr;
		const TArray<int32>* PreSolvePlateIds = nullptr;
		const TArray<uint8>* MissLineageCounts = nullptr;
		const TArray<uint8>* CurrentSolvePreviousSyntheticFlags = nullptr;
		const TArray<uint8>* CurrentSolvePreviousRetainedSyntheticCoverageFlags = nullptr;
		TArray<uint8>* CurrentSolveRetainedSyntheticCoverageFlags = nullptr;
		const TArray<uint8>* CurrentSolvePreviousOwnerRecoveryFlags = nullptr;
		const TArray<uint8>* CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags = nullptr;
		const TArray<uint8>* CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags = nullptr;
		const TArray<uint8>* CurrentSolvePreviousTransferSourceKindValues = nullptr;
		const TArray<uint8>* CurrentSolvePreviousResolutionKindValues = nullptr;
		const TArray<uint8>* ConvergentActiveFieldContinuityFlags = nullptr;
		const TArray<uint8>* AdjacentToConvergentActiveFieldContinuityFlags = nullptr;
		TArray<uint8>* CurrentSolveLocalParticipationPlateCounts = nullptr;
		TArray<uint8>* RecoveryMissPlateCandidateCounts = nullptr;
		const TArray<uint8>* RecoveryCandidatePlateCandidateCounts = nullptr;
		const TArray<int32>* UnfilteredExactCandidateCounts = nullptr;
		const TArray<int32>* UnfilteredDestructiveCandidateCounts = nullptr;
		TAtomic<int32>* ActiveBandPreviousOwnerCompatibleRecoveryCount = nullptr;
		TAtomic<int32>* ActiveBandSyntheticLoopBreakCount = nullptr;
		TAtomic<int32>* MissCount = nullptr;
		bool bEnableSyntheticCoverageRetentionForTest = false;
		bool bEnableStructuredGapFill = false;
		bool bApplyDestructiveFilter = false;
	};

	void ApplyCopiedFrontierZeroHitResolvedDecisionStage(
		const FV6CopiedFrontierZeroHitResolvedDecisionStageContext& Context,
		const int32 SampleIndex,
		const FVector3d& QueryPoint,
		const int32 PreviousPlateId,
		const TArray<FV6ThesisRemeshRayHit>& UnfilteredHitCandidates,
		const FTectonicPlanetV6RecoveryCandidate* PreviousOwnerRecoveryCandidate,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		FTectonicPlanetV6ResolvedSample& Resolved)
	{
		(*Context.CurrentSolveLocalParticipationPlateCounts)[SampleIndex] = static_cast<uint8>(FMath::Clamp(
			CountDistinctRecoveryCandidatePlates(RecoveryCandidates),
			0,
			255));
		(*Context.RecoveryMissPlateCandidateCounts)[SampleIndex] =
			(*Context.RecoveryCandidatePlateCandidateCounts)[SampleIndex];

		const bool bPreviousSynthetic = (*Context.CurrentSolvePreviousSyntheticFlags)[SampleIndex] != 0;
		const bool bPreviousRetainedSyntheticCoverage =
			Context.CurrentSolvePreviousRetainedSyntheticCoverageFlags->IsValidIndex(SampleIndex) &&
			(*Context.CurrentSolvePreviousRetainedSyntheticCoverageFlags)[SampleIndex] != 0;
		const bool bPreviousOwnerAdjacentTriangleSupportPresent =
			Context.CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags->IsValidIndex(SampleIndex) &&
			(*Context.CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags)[SampleIndex] != 0;
		const bool bConvergentActiveFieldContinuityLocality =
			IsCopiedFrontierConvergentFieldContinuityLocality(
				*Context.ConvergentActiveFieldContinuityFlags,
				*Context.AdjacentToConvergentActiveFieldContinuityFlags,
				SampleIndex);
		const bool bRetainPreviousSyntheticCoverage =
			Context.bEnableSyntheticCoverageRetentionForTest &&
			ShouldRetainPreviousSyntheticCoverageForMiss(
				*Context.Planet,
				SampleIndex,
				PreviousPlateId,
				bPreviousSynthetic,
				bPreviousRetainedSyntheticCoverage,
				bPreviousOwnerAdjacentTriangleSupportPresent,
				RecoveryCandidates);
		if (bRetainPreviousSyntheticCoverage)
		{
			Resolved.FinalPlateId = PreviousPlateId;
			Resolved.PreCoherencePlateId = PreviousPlateId;
			Resolved.SourceCanonicalSampleIndex = SampleIndex;
			Resolved.RecoveryDistanceRadians =
				PreviousOwnerRecoveryCandidate != nullptr
					? PreviousOwnerRecoveryCandidate->DistanceRadians
					: -1.0;
			if (const FTectonicPlanetV6RecoveryCandidate* OtherRecoveryCandidate =
					FindBestRecoveryCandidateForOtherPlate(RecoveryCandidates, PreviousPlateId))
			{
				Resolved.BoundaryOtherPlateId = OtherRecoveryCandidate->PlateId;
			}
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshRetainedSyntheticCoverage;
			Resolved.bRetainedSyntheticCoverage = true;
			(*Context.CurrentSolveRetainedSyntheticCoverageFlags)[SampleIndex] = 1;
			return;
		}

		int32 ExplicitFallbackPlateId = INDEX_NONE;
		int32 ExplicitFallbackSampleIndex = INDEX_NONE;
		ChooseExplicitFallback(*Context.Planet, ExplicitFallbackPlateId, ExplicitFallbackSampleIndex);

		int32 PrimaryPlateId = INDEX_NONE;
		int32 SecondaryPlateId = INDEX_NONE;
		double PrimaryDistance = -1.0;
		ChooseThesisRemeshMissOwnerPlates(
			RecoveryCandidates,
			PreviousPlateId,
			ExplicitFallbackPlateId,
			PrimaryPlateId,
			SecondaryPlateId,
			PrimaryDistance);

		const int32 UnfilteredCandidateCount =
			(Context.bApplyDestructiveFilter && Context.UnfilteredExactCandidateCounts->IsValidIndex(SampleIndex))
				? (*Context.UnfilteredExactCandidateCounts)[SampleIndex]
				: 0;
		const int32 UnfilteredDestructiveCandidateCount =
			(Context.bApplyDestructiveFilter && Context.UnfilteredDestructiveCandidateCounts->IsValidIndex(SampleIndex))
				? (*Context.UnfilteredDestructiveCandidateCounts)[SampleIndex]
				: 0;

		Resolved.BoundaryOtherPlateId = SecondaryPlateId;
		Resolved.RecoveryDistanceRadians = PrimaryDistance;
		int32 FillPreferredPrimaryPlateId = PrimaryPlateId;
		int32 FillPreferredSecondaryPlateId = SecondaryPlateId;
		if (Context.bApplyDestructiveFilter &&
			UnfilteredCandidateCount > 0 &&
			UnfilteredDestructiveCandidateCount == UnfilteredCandidateCount)
		{
			const int32 PreferredPlateId = ChooseDestructiveExclusionContinuationPlateId(
				*Context.Planet,
				UnfilteredHitCandidates,
				*Context.DestructiveFilterState,
				PreviousPlateId,
				PrimaryPlateId);
			const FTectonicPlanetV6RecoveryCandidate* PreferredOtherRecoveryCandidate =
				FindBestRecoveryCandidateForOtherPlate(RecoveryCandidates, PreferredPlateId);
			FillPreferredPrimaryPlateId = PreferredPlateId != INDEX_NONE ? PreferredPlateId : PrimaryPlateId;
			FillPreferredSecondaryPlateId =
				PreferredOtherRecoveryCandidate != nullptr
					? PreferredOtherRecoveryCandidate->PlateId
					: SecondaryPlateId;
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion;
		}
		else if (Context.bApplyDestructiveFilter && UnfilteredCandidateCount > 0)
		{
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous;
		}
		else
		{
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic;
		}

		bool bUseActiveBandCompatibleRecovery = false;
		bool bUsedSyntheticLoopBreak = false;
		int32 ActiveBandPreferredSourceCanonicalSampleIndex = INDEX_NONE;
		if (bConvergentActiveFieldContinuityLocality &&
			PreviousPlateId != INDEX_NONE &&
			FindPlateById(*Context.Planet, PreviousPlateId) != nullptr &&
			HasCopiedFrontierPreviousOwnerFieldRecoveryContext(
				Resolved,
				*Context.CurrentSolvePreviousOwnerRecoveryFlags,
				*Context.CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags,
				*Context.CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags,
				SampleIndex))
		{
			FillPreferredPrimaryPlateId = PreviousPlateId;
			if (const FTectonicPlanetV6RecoveryCandidate* OtherRecoveryCandidate =
					FindBestRecoveryCandidateForOtherPlate(RecoveryCandidates, PreviousPlateId))
			{
				FillPreferredSecondaryPlateId = OtherRecoveryCandidate->PlateId;
			}
			if (FindCarriedSampleForCanonicalVertex(*Context.Planet, PreviousPlateId, SampleIndex) != nullptr)
			{
				ActiveBandPreferredSourceCanonicalSampleIndex = SampleIndex;
			}
			bUseActiveBandCompatibleRecovery = true;
		}
		else if (bConvergentActiveFieldContinuityLocality &&
			bPreviousSynthetic &&
			Context.MissLineageCounts->IsValidIndex(SampleIndex) &&
			(*Context.MissLineageCounts)[SampleIndex] >= 2)
		{
			int32 LoopBreakPlateId = INDEX_NONE;
			int32 LoopBreakSourceCanonicalSampleIndex = INDEX_NONE;
			int32 LoopBreakSupportCount = 0;
			if (TryChooseActiveBandSyntheticLoopBreakSource(
					*Context.Planet,
					SampleIndex,
					PreviousPlateId,
					*Context.PreSolvePlateIds,
					RecoveryCandidates,
					*Context.CurrentSolvePreviousSyntheticFlags,
					*Context.CurrentSolvePreviousTransferSourceKindValues,
					*Context.CurrentSolvePreviousResolutionKindValues,
					LoopBreakPlateId,
					LoopBreakSourceCanonicalSampleIndex,
					LoopBreakSupportCount))
			{
				FillPreferredPrimaryPlateId = LoopBreakPlateId;
				if (const FTectonicPlanetV6RecoveryCandidate* OtherRecoveryCandidate =
						FindBestRecoveryCandidateForOtherPlate(RecoveryCandidates, LoopBreakPlateId))
				{
					FillPreferredSecondaryPlateId = OtherRecoveryCandidate->PlateId;
				}
				ActiveBandPreferredSourceCanonicalSampleIndex = LoopBreakSourceCanonicalSampleIndex;
				bUseActiveBandCompatibleRecovery = true;
				bUsedSyntheticLoopBreak = true;
			}
		}

		Resolved.FinalPlateId = FillPreferredPrimaryPlateId;
		Resolved.PreCoherencePlateId = FillPreferredPrimaryPlateId;
		Resolved.BoundaryOtherPlateId = FillPreferredSecondaryPlateId;
		if (bUseActiveBandCompatibleRecovery)
		{
			SeedStructuredDestructiveGapSource(
				*Context.Planet,
				FillPreferredPrimaryPlateId,
				RecoveryCandidates,
				Resolved,
				QueryPoint);
			if (ActiveBandPreferredSourceCanonicalSampleIndex != INDEX_NONE)
			{
				Resolved.SourceCanonicalSampleIndex = ActiveBandPreferredSourceCanonicalSampleIndex;
			}
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshTransferFallback;
			if (!bUsedSyntheticLoopBreak)
			{
				++(*Context.ActiveBandPreviousOwnerCompatibleRecoveryCount);
			}
			if (bUsedSyntheticLoopBreak)
			{
				++(*Context.ActiveBandSyntheticLoopBreakCount);
			}
			return;
		}

		++(*Context.MissCount);
		if (Context.bEnableStructuredGapFill)
		{
			int32 FrontierPrimaryPlateId = INDEX_NONE;
			int32 FrontierSecondaryPlateId = INDEX_NONE;
			if (TryBuildStructuredFrontierPairFill(
					*Context.FrontierPointSets,
					QueryPoint,
					FillPreferredPrimaryPlateId,
					FillPreferredSecondaryPlateId,
					Resolved.StructuredSyntheticFillSample,
					FrontierPrimaryPlateId,
					FrontierSecondaryPlateId))
			{
				Resolved.bHasStructuredSyntheticFill = true;
				if (FrontierPrimaryPlateId != INDEX_NONE)
				{
					Resolved.FinalPlateId = FrontierPrimaryPlateId;
					Resolved.PreCoherencePlateId = FrontierPrimaryPlateId;
				}
				Resolved.BoundaryOtherPlateId = FrontierSecondaryPlateId;
			}
		}
	}

	struct FV6CopiedFrontierRecoveryPreparationStageContext
	{
		const FTectonicPlanet* Planet = nullptr;
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>* SolveCopiedFrontierMeshes = nullptr;
		const TArray<FV6PlateQueryGeometry>* QueryGeometries = nullptr;
		const TMap<int32, int32>* QueryGeometryIndexByPlateId = nullptr;
		const TArray<FV6PlateQueryGeometry>* UnfilteredComparisonQueryGeometries = nullptr;
		const TMap<int32, int32>* UnfilteredComparisonQueryGeometryIndexByPlateId = nullptr;
		const TArray<TArray<int32>>* SampleToAdjacentTriangles = nullptr;
		TArray<uint8>* CurrentSolvePreviousOwnerFilteredHitFlags = nullptr;
		TArray<uint8>* CurrentSolvePreviousOwnerUnfilteredHitFlags = nullptr;
		TArray<uint8>* CurrentSolvePreviousOwnerIgnoringBoundingCapHitFlags = nullptr;
		TArray<uint8>* CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags = nullptr;
		TArray<uint8>* CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags = nullptr;
		TArray<float>* CurrentSolvePreviousOwnerAdjacentTriangleNearestDistanceKm = nullptr;
		TArray<uint16>* CurrentSolvePreviousOwnerAdjacentTriangleSupportCounts = nullptr;
		TArray<uint16>* CurrentSolvePreviousOwnerAdjacentTriangleMixedSupportCounts = nullptr;
		TArray<uint16>* CurrentSolvePreviousOwnerAdjacentTriangleMinoritySupportCounts = nullptr;
		TArray<uint8>* CurrentSolveLocalParticipationPlateCounts = nullptr;
		TArray<uint8>* RecoveryCandidatePlateCandidateCounts = nullptr;
		TArray<uint8>* RecoveryCandidateGatherSampleFlags = nullptr;
		TArray<uint8>* RecoveryCandidatePrunedSampleFlags = nullptr;
		TArray<uint8>* CurrentSolvePreviousOwnerRecoveryFlags = nullptr;
		TAtomic<int64>* ZeroHitRecoveryMicroseconds = nullptr;
		bool bApplyDestructiveFilter = false;
		bool bRecordPhaseTiming = false;
	};

	struct FV6CopiedFrontierRecoveryPreparationStageOutput
	{
		FV6ThesisRemeshRayHit PreviousOwnerFilteredHit;
		bool bPreviousOwnerFilteredHitPresent = false;
		TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
		FTectonicPlanetV6RecoveryCandidate PreviousOwnerRecoveryCandidate;
		bool bHasPreviousOwnerRecoveryCandidate = false;
	};

	void PopulateCopiedFrontierRecoveryPreparationCandidates(
		const FV6CopiedFrontierRecoveryPreparationStageContext& Context,
		const int32 SampleIndex,
		const int32 PreviousPlateId,
		const FVector3d& QueryPoint,
		const TArray<int32, TInlineAllocator<4>>* HitSearchCandidateQueryGeometryIndices,
		const bool bGatherRecoveryCandidates,
		const bool bUseActivePairFiltering,
		const int32 ActiveZonePrimaryPlateId,
		const int32 ActiveZoneSecondaryPlateId,
		const bool bAccumulateGatherTiming,
		FV6CopiedFrontierRecoveryPreparationStageOutput& InOut)
	{
		int32 RecoveryCandidateTestedPlateCount = 0;
		if (bGatherRecoveryCandidates)
		{
			(*Context.RecoveryCandidateGatherSampleFlags)[SampleIndex] = 1;
			const double RecoveryCandidateGatherStartTime =
				(Context.bRecordPhaseTiming && bAccumulateGatherTiming) ? GetPhaseTimingSeconds() : 0.0;
			CollectThesisRemeshMissRecoveryCandidates(
				*Context.QueryGeometries,
				HitSearchCandidateQueryGeometryIndices,
				QueryPoint,
				InOut.RecoveryCandidates,
				&RecoveryCandidateTestedPlateCount);
			if (Context.bRecordPhaseTiming && bAccumulateGatherTiming)
			{
				AccumulatePhaseTimingMicroseconds(
					*Context.ZeroHitRecoveryMicroseconds,
					RecoveryCandidateGatherStartTime);
			}
		}

		FTectonicPlanetV6RecoveryCandidate PreviousOwnerRecoveryCandidateValue;
		const bool bPreviousOwnerRecoveryCandidatePresent =
			TryFindThesisRemeshRecoveryCandidateForPlate(
				*Context.QueryGeometries,
				*Context.QueryGeometryIndexByPlateId,
				PreviousPlateId,
				QueryPoint,
				PreviousOwnerRecoveryCandidateValue);
		const FTectonicPlanetV6RecoveryCandidate* PreviousOwnerRecoveryCandidate =
			bPreviousOwnerRecoveryCandidatePresent
				? &PreviousOwnerRecoveryCandidateValue
				: FindBestRecoveryCandidateForPlate(InOut.RecoveryCandidates, PreviousPlateId);
		(*Context.CurrentSolvePreviousOwnerRecoveryFlags)[SampleIndex] =
			PreviousOwnerRecoveryCandidate != nullptr ? 1 : 0;
		(*Context.RecoveryCandidatePlateCandidateCounts)[SampleIndex] = static_cast<uint8>(FMath::Clamp(
			RecoveryCandidateTestedPlateCount,
			0,
			255));
		(*Context.RecoveryCandidatePrunedSampleFlags)[SampleIndex] =
			(bGatherRecoveryCandidates &&
				HitSearchCandidateQueryGeometryIndices != nullptr &&
				HitSearchCandidateQueryGeometryIndices->Num() < Context.QueryGeometries->Num())
				? 1
				: 0;
		if (bGatherRecoveryCandidates)
		{
			(*Context.CurrentSolveLocalParticipationPlateCounts)[SampleIndex] = static_cast<uint8>(FMath::Clamp(
				CountDistinctRecoveryCandidatePlates(InOut.RecoveryCandidates),
				0,
				255));
		}

		if (bUseActivePairFiltering && bGatherRecoveryCandidates)
		{
			FilterThesisRemeshRecoveryCandidatesToAllowedPlates(
				InOut.RecoveryCandidates,
				ActiveZonePrimaryPlateId,
				ActiveZoneSecondaryPlateId);
			PreviousOwnerRecoveryCandidate =
				bPreviousOwnerRecoveryCandidatePresent
					? &PreviousOwnerRecoveryCandidateValue
					: FindBestRecoveryCandidateForPlate(InOut.RecoveryCandidates, PreviousPlateId);
		}

		if (PreviousOwnerRecoveryCandidate != nullptr)
		{
			InOut.PreviousOwnerRecoveryCandidate = *PreviousOwnerRecoveryCandidate;
			InOut.bHasPreviousOwnerRecoveryCandidate = true;
		}
		else
		{
			InOut.bHasPreviousOwnerRecoveryCandidate = false;
		}
	}

	void BuildCopiedFrontierRecoveryPreparationStage(
		const FV6CopiedFrontierRecoveryPreparationStageContext& Context,
		const int32 SampleIndex,
		const int32 PreviousPlateId,
		const FVector3d& QueryPoint,
		const TArray<int32, TInlineAllocator<4>>* HitSearchCandidateQueryGeometryIndices,
		const bool bNeedsRecoveryCandidates,
		const bool bUseActivePairFiltering,
		const int32 ActiveZonePrimaryPlateId,
		const int32 ActiveZoneSecondaryPlateId,
		TArray<FV6ThesisRemeshRayHit>& InOutHitCandidates,
		FV6CopiedFrontierRecoveryPreparationStageOutput& Out)
	{
		Out = FV6CopiedFrontierRecoveryPreparationStageOutput{};
		Out.bPreviousOwnerFilteredHitPresent =
			TryFindThesisRemeshRayHitForPlate(
				*Context.QueryGeometries,
				*Context.QueryGeometryIndexByPlateId,
				PreviousPlateId,
				QueryPoint,
				Out.PreviousOwnerFilteredHit);

		FV6ThesisRemeshRayHit PreviousOwnerUnfilteredHit;
		const bool bPreviousOwnerUnfilteredHitPresent =
			Context.bApplyDestructiveFilter
				? TryFindThesisRemeshRayHitForPlate(
					*Context.UnfilteredComparisonQueryGeometries,
					*Context.UnfilteredComparisonQueryGeometryIndexByPlateId,
					PreviousPlateId,
					QueryPoint,
					PreviousOwnerUnfilteredHit)
				: Out.bPreviousOwnerFilteredHitPresent;
		bool bPreviousOwnerIgnoringBoundingCapHitPresent = false;
		bool bPreviousOwnerCanonicalVertexInMeshPresent = false;
		bool bPreviousOwnerAdjacentTriangleInMeshPresent = false;
		double PreviousOwnerAdjacentTriangleNearestDistanceKm = -1.0;
		int32 PreviousOwnerAdjacentTriangleSupportCount = 0;
		int32 PreviousOwnerAdjacentTriangleMixedSupportCount = 0;
		int32 PreviousOwnerAdjacentTriangleMinoritySupportCount = 0;

		const int32 PreviousPlateIndex = Context.Planet->FindPlateArrayIndexById(PreviousPlateId);
		if (PreviousPlateIndex != INDEX_NONE &&
			Context.SolveCopiedFrontierMeshes->IsValidIndex(PreviousPlateIndex) &&
			Context.QueryGeometries->IsValidIndex(PreviousPlateIndex))
		{
			const FTectonicPlanetV6CopiedFrontierPlateMesh& PreviousOwnerMesh =
				(*Context.SolveCopiedFrontierMeshes)[PreviousPlateIndex];
			bPreviousOwnerCanonicalVertexInMeshPresent =
				PreviousOwnerMesh.CanonicalToLocalVertex.Contains(SampleIndex);
			if (Context.SampleToAdjacentTriangles->IsValidIndex(SampleIndex))
			{
				for (const int32 AdjacentTriangleIndex : (*Context.SampleToAdjacentTriangles)[SampleIndex])
				{
					const int32* LocalTriangleIndexPtr =
						PreviousOwnerMesh.GlobalToLocalTriangle.Find(AdjacentTriangleIndex);
					if (LocalTriangleIndexPtr == nullptr)
					{
						continue;
					}

					bPreviousOwnerAdjacentTriangleInMeshPresent = true;
					++PreviousOwnerAdjacentTriangleSupportCount;
					const FIntVector& AdjacentTriangle = Context.Planet->TriangleIndices[AdjacentTriangleIndex];
					const int32 AdjacentTrianglePlateIds[3] = {
						Context.Planet->Samples[AdjacentTriangle.X].PlateId,
						Context.Planet->Samples[AdjacentTriangle.Y].PlateId,
						Context.Planet->Samples[AdjacentTriangle.Z].PlateId
					};
					TArray<int32> AdjacentInvolvedPlateIds;
					CollectBoundaryTrianglePlateIds(
						AdjacentTrianglePlateIds[0],
						AdjacentTrianglePlateIds[1],
						AdjacentTrianglePlateIds[2],
						AdjacentInvolvedPlateIds);
					if (AdjacentInvolvedPlateIds.Num() > 1)
					{
						++PreviousOwnerAdjacentTriangleMixedSupportCount;
					}
					if (CountPlateOccurrencesInTriangle(AdjacentTrianglePlateIds, PreviousPlateId) == 1)
					{
						++PreviousOwnerAdjacentTriangleMinoritySupportCount;
					}

					const double CandidateDistanceKm = ComputeQueryTriangleDistanceKm(
						*Context.Planet,
						(*Context.QueryGeometries)[PreviousPlateIndex],
						QueryPoint,
						*LocalTriangleIndexPtr);
					if (CandidateDistanceKm >= 0.0 &&
						(PreviousOwnerAdjacentTriangleNearestDistanceKm < 0.0 ||
							CandidateDistanceKm < PreviousOwnerAdjacentTriangleNearestDistanceKm))
					{
						PreviousOwnerAdjacentTriangleNearestDistanceKm = CandidateDistanceKm;
					}
				}
			}

			FV6ThesisRemeshRayHit IgnoringCapHit;
			bPreviousOwnerIgnoringBoundingCapHitPresent =
				TryFindThesisRemeshHitIgnoringBoundingCap(
					(*Context.QueryGeometries)[PreviousPlateIndex],
					QueryPoint,
					IgnoringCapHit);
		}

		(*Context.CurrentSolvePreviousOwnerFilteredHitFlags)[SampleIndex] =
			Out.bPreviousOwnerFilteredHitPresent ? 1 : 0;
		(*Context.CurrentSolvePreviousOwnerUnfilteredHitFlags)[SampleIndex] =
			bPreviousOwnerUnfilteredHitPresent ? 1 : 0;
		(*Context.CurrentSolvePreviousOwnerIgnoringBoundingCapHitFlags)[SampleIndex] =
			bPreviousOwnerIgnoringBoundingCapHitPresent ? 1 : 0;
		(*Context.CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags)[SampleIndex] =
			bPreviousOwnerCanonicalVertexInMeshPresent ? 1 : 0;
		(*Context.CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags)[SampleIndex] =
			bPreviousOwnerAdjacentTriangleInMeshPresent ? 1 : 0;
		(*Context.CurrentSolvePreviousOwnerAdjacentTriangleNearestDistanceKm)[SampleIndex] =
			static_cast<float>(PreviousOwnerAdjacentTriangleNearestDistanceKm);
		(*Context.CurrentSolvePreviousOwnerAdjacentTriangleSupportCounts)[SampleIndex] =
			static_cast<uint16>(FMath::Clamp(PreviousOwnerAdjacentTriangleSupportCount, 0, 65535));
		(*Context.CurrentSolvePreviousOwnerAdjacentTriangleMixedSupportCounts)[SampleIndex] =
			static_cast<uint16>(FMath::Clamp(PreviousOwnerAdjacentTriangleMixedSupportCount, 0, 65535));
		(*Context.CurrentSolvePreviousOwnerAdjacentTriangleMinoritySupportCounts)[SampleIndex] =
			static_cast<uint16>(FMath::Clamp(PreviousOwnerAdjacentTriangleMinoritySupportCount, 0, 65535));
		(*Context.CurrentSolveLocalParticipationPlateCounts)[SampleIndex] = static_cast<uint8>(FMath::Clamp(
			CountDistinctHitCandidatePlates(InOutHitCandidates),
			0,
			255));

		PopulateCopiedFrontierRecoveryPreparationCandidates(
			Context,
			SampleIndex,
			PreviousPlateId,
			QueryPoint,
			HitSearchCandidateQueryGeometryIndices,
			bNeedsRecoveryCandidates,
			bUseActivePairFiltering,
			ActiveZonePrimaryPlateId,
			ActiveZoneSecondaryPlateId,
			true,
			Out);

		if (bUseActivePairFiltering)
		{
			FilterThesisRemeshHitCandidatesToAllowedPlates(
				InOutHitCandidates,
				ActiveZonePrimaryPlateId,
				ActiveZoneSecondaryPlateId);
		}
	}

	struct FV6CopiedFrontierTransferStageContext
	{
		const FTectonicPlanet* Planet = nullptr;
		const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>* SolveCopiedFrontierMeshes = nullptr;
		const TArray<uint8>* ConvergentActiveFieldContinuityFlags = nullptr;
		const TArray<uint8>* AdjacentToConvergentActiveFieldContinuityFlags = nullptr;
		TAtomic<int32>* DirectHitTriangleTransferCount = nullptr;
		TAtomic<int32>* TransferFallbackCount = nullptr;
		TAtomic<int32>* NearestMemberFallbackTransferCount = nullptr;
		TAtomic<int32>* ExplicitFallbackTransferCount = nullptr;
		TAtomic<int32>* ActiveBandSyntheticSingleSourceRecoveryCount = nullptr;
		TAtomic<int64>* DirectHitTransferMicroseconds = nullptr;
		TAtomic<int64>* FallbackTransferMicroseconds = nullptr;
		bool bRecordPhaseTiming = false;
	};

	bool ApplyCopiedFrontierResolvedTransferStage(
		const FV6CopiedFrontierTransferStageContext& Context,
		const int32 SampleIndex,
		FSample& Sample,
		FTectonicPlanetV6ResolvedSample& Resolved,
		float& OutSubductionDistanceKm,
		float& OutSubductionSpeed)
	{
		Resolved.TransferDebug = FTectonicPlanetV6TransferDebugInfo{};
		bool bTransferred = false;
		double FallbackTransferStartTime = 0.0;
		bool bFallbackTransferTimingActive = false;
		const auto BeginFallbackTransferTiming = [&]()
		{
			if (Context.bRecordPhaseTiming && !bFallbackTransferTimingActive)
			{
				FallbackTransferStartTime = GetPhaseTimingSeconds();
				bFallbackTransferTimingActive = true;
			}
		};
		const auto FinishFallbackTransferTiming = [&]()
		{
			if (Context.bRecordPhaseTiming && bFallbackTransferTimingActive)
			{
				AccumulatePhaseTimingMicroseconds(
					*Context.FallbackTransferMicroseconds,
					FallbackTransferStartTime);
				bFallbackTransferTimingActive = false;
			}
		};
		const auto TryApplyResolvedTriangleTransfer = [&](const bool bCountDirectHitTransfer)
		{
			if (Resolved.FinalPlateId == INDEX_NONE)
			{
				return false;
			}

			const int32 PlateIndex = Context.Planet->FindPlateArrayIndexById(Resolved.FinalPlateId);
			if (!Context.SolveCopiedFrontierMeshes->IsValidIndex(PlateIndex) ||
				!Context.Planet->Plates.IsValidIndex(PlateIndex))
			{
				return false;
			}

			const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = (*Context.SolveCopiedFrontierMeshes)[PlateIndex];
			int32 LocalTriangleIndex = Resolved.SourceLocalTriangleIndex;
			if (!Mesh.LocalTriangles.IsValidIndex(LocalTriangleIndex) &&
				Resolved.SourceTriangleIndex != INDEX_NONE)
			{
				if (const int32* LocalTriangleIndexPtr = Mesh.GlobalToLocalTriangle.Find(Resolved.SourceTriangleIndex))
				{
					LocalTriangleIndex = *LocalTriangleIndexPtr;
				}
			}
			if (!Mesh.LocalTriangles.IsValidIndex(LocalTriangleIndex))
			{
				return false;
			}

			const UE::Geometry::FIndex3i& LocalTriangle = Mesh.LocalTriangles[LocalTriangleIndex];
			if (!Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.A) ||
				!Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.B) ||
				!Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.C))
			{
				return false;
			}

			const FPlate& Plate = Context.Planet->Plates[PlateIndex];
			const FCarriedSample V0 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.A]);
			const FCarriedSample V1 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.B]);
			const FCarriedSample V2 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.C]);
			const double DirectHitTransferStartTime =
				Context.bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
			ApplyTransferredAttributesFromTriangle(
				Sample,
				V0,
				V1,
				V2,
				Resolved.SourceBarycentric,
				OutSubductionDistanceKm,
				OutSubductionSpeed,
				Resolved.TransferDebug);
			Resolved.SourceCanonicalSampleIndex = Resolved.TransferDebug.DominantSourceCanonicalSampleIndex;
			Resolved.TransferDebug.bUsedCopiedFrontierTriangleTransfer =
				Mesh.LocalTriangleCopiedFrontierFlags.IsValidIndex(LocalTriangleIndex) &&
				Mesh.LocalTriangleCopiedFrontierFlags[LocalTriangleIndex] != 0;
			if (bCountDirectHitTransfer)
			{
				++(*Context.DirectHitTriangleTransferCount);
			}
			if (Context.bRecordPhaseTiming)
			{
				AccumulatePhaseTimingMicroseconds(
					*Context.DirectHitTransferMicroseconds,
					DirectHitTransferStartTime);
			}
			return true;
		};
		const auto TryApplyCanonicalSingleSourceTransfer = [&]()
		{
			if (Resolved.FinalPlateId == INDEX_NONE || Resolved.SourceCanonicalSampleIndex == INDEX_NONE)
			{
				return false;
			}

			if (const FCarriedSample* Source =
					FindCarriedSampleForCanonicalVertex(
						*Context.Planet,
						Resolved.FinalPlateId,
						Resolved.SourceCanonicalSampleIndex))
			{
				ApplyTransferredAttributesFromSingleSource(
					Sample,
					*Source,
					OutSubductionDistanceKm,
					OutSubductionSpeed,
					Resolved.TransferDebug);
				return true;
			}

			return false;
		};

		if ((Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshHit ||
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshRetainedOutsideActiveZone) &&
			TryApplyResolvedTriangleTransfer(true))
		{
			bTransferred = true;
		}

		if (!bTransferred &&
			Resolved.bAuthorityRetainedOutsideActiveZone &&
			Resolved.FinalPlateId != INDEX_NONE &&
			Resolved.SourceCanonicalSampleIndex != INDEX_NONE)
		{
			BeginFallbackTransferTiming();
			bTransferred = TryApplyCanonicalSingleSourceTransfer();
		}

		if (!bTransferred &&
			Resolved.bRetainedSyntheticCoverage &&
			Resolved.FinalPlateId != INDEX_NONE &&
			Resolved.SourceCanonicalSampleIndex != INDEX_NONE)
		{
			BeginFallbackTransferTiming();
			bTransferred = TryApplyCanonicalSingleSourceTransfer();
		}

		if (!bTransferred &&
			Resolved.bHasStructuredSyntheticFill)
		{
			BeginFallbackTransferTiming();
			ApplyStructuredSyntheticGapFill(
				Sample,
				Resolved.StructuredSyntheticFillSample,
				OutSubductionDistanceKm,
				OutSubductionSpeed,
				Resolved.TransferDebug);
			Resolved.SourceCanonicalSampleIndex = Resolved.StructuredSyntheticFillSample.CanonicalSampleIndex;
			bTransferred = true;
		}

		if (!bTransferred &&
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion &&
			Resolved.FinalPlateId != INDEX_NONE)
		{
			bTransferred = TryApplyResolvedTriangleTransfer(false);
			if (!bTransferred && Resolved.SourceCanonicalSampleIndex != INDEX_NONE)
			{
				BeginFallbackTransferTiming();
				bTransferred = TryApplyCanonicalSingleSourceTransfer();
			}
		}

		if (!bTransferred &&
			(Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion ||
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous))
		{
			BeginFallbackTransferTiming();
			ApplyOceanicBoundaryCreation(
				Sample,
				*Context.Planet,
				Resolved,
				OutSubductionDistanceKm,
				OutSubductionSpeed,
				Resolved.TransferDebug);
			bTransferred = true;
		}

		if (!bTransferred && Resolved.FinalPlateId != INDEX_NONE)
		{
			BeginFallbackTransferTiming();
			int32 TransferSourceCanonicalSampleIndex = Resolved.SourceCanonicalSampleIndex;
			if (TransferSourceCanonicalSampleIndex == INDEX_NONE &&
				Resolved.SourceTriangleIndex != INDEX_NONE)
			{
				TransferSourceCanonicalSampleIndex =
					PickTransferFallbackCanonicalVertexForTriangle(
						*Context.Planet,
						Resolved.FinalPlateId,
						Resolved.SourceTriangleIndex,
						Resolved.SourceBarycentric);
			}
			bool bUsedNearestMemberFallback = false;
			bool bUsedExplicitFallback = false;
			const FCarriedSample* Source =
				TransferSourceCanonicalSampleIndex != INDEX_NONE
					? FindCarriedSampleForCanonicalVertex(
						*Context.Planet,
						Resolved.FinalPlateId,
						TransferSourceCanonicalSampleIndex)
					: nullptr;
			if (Source == nullptr)
			{
				double NearestMemberDistanceRadians = TNumericLimits<double>::Max();
				if (FindNearestMemberSample(
						*Context.Planet,
						Resolved.FinalPlateId,
						Sample.Position,
						TransferSourceCanonicalSampleIndex,
						NearestMemberDistanceRadians))
				{
					Source = FindCarriedSampleForCanonicalVertex(
						*Context.Planet,
						Resolved.FinalPlateId,
						TransferSourceCanonicalSampleIndex);
					bUsedNearestMemberFallback = Source != nullptr;
				}
			}
			if (Source == nullptr)
			{
				ChooseExplicitFallbackForPlate(
					*Context.Planet,
					Resolved.FinalPlateId,
					TransferSourceCanonicalSampleIndex);
				if (TransferSourceCanonicalSampleIndex != INDEX_NONE)
				{
					Source = FindCarriedSampleForCanonicalVertex(
						*Context.Planet,
						Resolved.FinalPlateId,
						TransferSourceCanonicalSampleIndex);
					bUsedExplicitFallback = Source != nullptr;
				}
			}
			if (Source != nullptr)
			{
				Resolved.SourceCanonicalSampleIndex = TransferSourceCanonicalSampleIndex;
				ApplyTransferredAttributesFromSingleSource(
					Sample,
					*Source,
					OutSubductionDistanceKm,
					OutSubductionSpeed,
					Resolved.TransferDebug);
				Resolved.TransferDebug.bUsedNearestMemberFallback = bUsedNearestMemberFallback;
				Resolved.TransferDebug.bUsedExplicitFallback = bUsedExplicitFallback;
				Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshTransferFallback;
				if (bUsedNearestMemberFallback)
				{
					++(*Context.NearestMemberFallbackTransferCount);
				}
				if (bUsedExplicitFallback)
				{
					++(*Context.ExplicitFallbackTransferCount);
				}
				if (IsCopiedFrontierConvergentFieldContinuityLocality(
						*Context.ConvergentActiveFieldContinuityFlags,
						*Context.AdjacentToConvergentActiveFieldContinuityFlags,
						SampleIndex) &&
					Resolved.ExactCandidateCount == 0 &&
					Resolved.SourceCanonicalSampleIndex != INDEX_NONE)
				{
					++(*Context.ActiveBandSyntheticSingleSourceRecoveryCount);
				}
				++(*Context.TransferFallbackCount);
				bTransferred = true;
			}
		}

		FinishFallbackTransferTiming();
		return bTransferred;
	}

	FTectonicPlanetV6BoundaryOutcomeTransferStats* FindBoundaryOutcomeTransferStats(
		FTectonicPlanetV6PeriodicSolveStats& Stats,
		const ETectonicPlanetV6BoundaryOutcome Outcome)
	{
		switch (Outcome)
		{
		case ETectonicPlanetV6BoundaryOutcome::RetainedOwner:
			return &Stats.BoundaryRetainedTransferStats;
		case ETectonicPlanetV6BoundaryOutcome::ReassignedOwner:
			return &Stats.BoundaryReassignedTransferStats;
		case ETectonicPlanetV6BoundaryOutcome::DivergentOceanic:
			return &Stats.BoundaryOceanicTransferStats;
		case ETectonicPlanetV6BoundaryOutcome::None:
		default:
			return nullptr;
		}
	}

	void AccumulateBoundaryOutcomeTransferStats(
		FTectonicPlanetV6BoundaryOutcomeTransferStats& InOutStats,
		const FTectonicPlanetV6ResolvedSample& Resolved)
	{
		++InOutStats.SampleCount;

		switch (Resolved.TransferDebug.SourceKind)
		{
		case ETectonicPlanetV6TransferSourceKind::Triangle:
			++InOutStats.TriangleTransferCount;
			if (Resolved.TransferDebug.bUsedTriangleRecovery)
			{
				++InOutStats.TriangleRecoveryTransferCount;
			}
			break;
		case ETectonicPlanetV6TransferSourceKind::SingleSource:
			++InOutStats.SingleSourceTransferCount;
			if (Resolved.TransferDebug.bUsedNearestMemberFallback)
			{
				++InOutStats.NearestMemberSingleSourceTransferCount;
			}
			if (Resolved.TransferDebug.bUsedExplicitFallback)
			{
				++InOutStats.ExplicitFallbackSingleSourceTransferCount;
			}
			break;
		case ETectonicPlanetV6TransferSourceKind::OceanicCreation:
			++InOutStats.OceanicCreationCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::Defaulted:
			++InOutStats.DefaultTransferCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::None:
		default:
			break;
		}

		if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
		{
			++InOutStats.ContinentalWeightThresholdMismatchCount;
		}
	}

	const TCHAR* GetBoundaryOutcomeName(const ETectonicPlanetV6BoundaryOutcome Outcome)
	{
		switch (Outcome)
		{
		case ETectonicPlanetV6BoundaryOutcome::RetainedOwner:
			return TEXT("retained");
		case ETectonicPlanetV6BoundaryOutcome::ReassignedOwner:
			return TEXT("reassigned");
		case ETectonicPlanetV6BoundaryOutcome::DivergentOceanic:
			return TEXT("oceanic");
		case ETectonicPlanetV6BoundaryOutcome::None:
		default:
			return TEXT("none");
		}
	}

	int32 CountBoundarySamples(const FTectonicPlanet& Planet)
	{
		int32 BoundarySampleCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			BoundarySampleCount += Sample.bIsBoundary ? 1 : 0;
		}
		return BoundarySampleCount;
	}

	int32 CountPlatePairContactEdges(
		const FTectonicPlanet& Planet,
		const int32 PlateA,
		const int32 PlateB)
	{
		if (PlateA == INDEX_NONE || PlateB == INDEX_NONE || PlateA == PlateB)
		{
			return 0;
		}

		int32 ContactEdgeCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 SamplePlateId = Planet.Samples[SampleIndex].PlateId;
			if (SamplePlateId != PlateA && SamplePlateId != PlateB)
			{
				continue;
			}

			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (NeighborIndex <= SampleIndex || !Planet.Samples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				const int32 NeighborPlateId = Planet.Samples[NeighborIndex].PlateId;
				if ((SamplePlateId == PlateA && NeighborPlateId == PlateB) ||
					(SamplePlateId == PlateB && NeighborPlateId == PlateA))
				{
					++ContactEdgeCount;
				}
			}
		}

		return ContactEdgeCount;
	}

	FTectonicPlanetV6PlatePairBoundaryMotionDiagnostic ComputePlatePairBoundaryMotionDiagnostic(
		const FTectonicPlanet& Planet,
		const int32 PlateA,
		const int32 PlateB)
	{
		FTectonicPlanetV6PlatePairBoundaryMotionDiagnostic Diagnostic;
		if (PlateA == INDEX_NONE || PlateB == INDEX_NONE || PlateA == PlateB)
		{
			return Diagnostic;
		}

		TArray<int32> CurrentPlateIds;
		CurrentPlateIds.Reserve(Planet.Samples.Num());
		for (const FSample& Sample : Planet.Samples)
		{
			CurrentPlateIds.Add(Sample.PlateId);
		}

		const TArray<int32> RelevantPlateIds{ PlateA, PlateB };
		double SumRelativeNormalVelocity = 0.0;
		int32 ClassifiedEdgeCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex) ||
				!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 SamplePlateId = Planet.Samples[SampleIndex].PlateId;
			if (SamplePlateId != PlateA && SamplePlateId != PlateB)
			{
				continue;
			}

			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (NeighborIndex <= SampleIndex || !Planet.Samples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				const int32 NeighborPlateId = Planet.Samples[NeighborIndex].PlateId;
				if (!((SamplePlateId == PlateA && NeighborPlateId == PlateB) ||
					  (SamplePlateId == PlateB && NeighborPlateId == PlateA)))
				{
					continue;
				}

				++Diagnostic.ContactEdgeCount;

				TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
				BuildBoundaryMotionSamplesForPlateIds(
					Planet,
					CurrentPlateIds,
					SampleIndex,
					RelevantPlateIds,
					MotionSamples);

				double RelativeNormalVelocity = 0.0;
				const EV6BoundaryMotionClass MotionClass = ClassifyBoundaryMotion(
					Planet.Samples[SampleIndex].Position,
					PlateA,
					PlateB,
					MotionSamples,
					RelativeNormalVelocity);

				if (MotionClass != EV6BoundaryMotionClass::Divergent &&
					MotionClass != EV6BoundaryMotionClass::Convergent)
				{
					continue;
				}

				SumRelativeNormalVelocity += RelativeNormalVelocity;
				++ClassifiedEdgeCount;
				Diagnostic.MaxAbsRelativeNormalVelocityKmPerMy = FMath::Max(
					Diagnostic.MaxAbsRelativeNormalVelocityKmPerMy,
					FMath::Abs(RelativeNormalVelocity));

				if (MotionClass == EV6BoundaryMotionClass::Divergent)
				{
					++Diagnostic.DivergentEdgeCount;
				}
				else
				{
					++Diagnostic.ConvergentEdgeCount;
				}
			}
		}

		Diagnostic.MeanRelativeNormalVelocityKmPerMy =
			ClassifiedEdgeCount > 0 ? SumRelativeNormalVelocity / static_cast<double>(ClassifiedEdgeCount) : 0.0;
		Diagnostic.bPairIsDivergent =
			Diagnostic.DivergentEdgeCount > 0 &&
			Diagnostic.DivergentEdgeCount >= Diagnostic.ConvergentEdgeCount &&
			Diagnostic.MeanRelativeNormalVelocityKmPerMy > 0.0;
		return Diagnostic;
	}

	void CountActivePairCauseSamples(
		const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples,
		const int32 PlateA,
		const int32 PlateB,
		int32& OutRiftSampleCount,
		int32& OutDivergenceSampleCount)
	{
		OutRiftSampleCount = 0;
		OutDivergenceSampleCount = 0;
		if (PlateA == INDEX_NONE || PlateB == INDEX_NONE || PlateA == PlateB)
		{
			return;
		}

		const uint64 PairKey = MakeOrderedPlatePairKey(PlateA, PlateB);
		for (const FTectonicPlanetV6ResolvedSample& Resolved : ResolvedSamples)
		{
			if (!Resolved.bActiveZoneSample ||
				Resolved.ActiveZonePrimaryPlateId == INDEX_NONE ||
				Resolved.ActiveZoneSecondaryPlateId == INDEX_NONE ||
				MakeOrderedPlatePairKey(
					Resolved.ActiveZonePrimaryPlateId,
					Resolved.ActiveZoneSecondaryPlateId) != PairKey)
			{
				continue;
			}

			switch (Resolved.ActiveZoneCause)
			{
			case ETectonicPlanetV6ActiveZoneCause::Rift:
				++OutRiftSampleCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::DivergenceFill:
				++OutDivergenceSampleCount;
				break;
			default:
				break;
			}
		}
	}

	double ComputeContinentalAreaFraction(const FTectonicPlanet& Planet)
	{
		if (Planet.Samples.IsEmpty())
		{
			return 0.0;
		}

		int32 ContinentalCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			ContinentalCount += Sample.ContinentalWeight >= 0.5f ? 1 : 0;
		}

		return static_cast<double>(ContinentalCount) / static_cast<double>(Planet.Samples.Num());
	}

	int32 ComputeMaxComponentsPerPlateFromAssignments(const FTectonicPlanet& Planet, const TArray<int32>& PlateIds)
	{
		check(PlateIds.Num() == Planet.Samples.Num());

		int32 MaxComponents = 0;
		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		TMap<int32, int32> ComponentCountsByPlate;

		for (int32 SampleIndex = 0; SampleIndex < PlateIds.Num(); ++SampleIndex)
		{
			const int32 PlateId = PlateIds[SampleIndex];
			if (PlateId == INDEX_NONE || Visited[SampleIndex] != 0)
			{
				continue;
			}

			Visited[SampleIndex] = 1;
			TArray<int32, TInlineAllocator<256>> Stack;
			Stack.Add(SampleIndex);

			while (!Stack.IsEmpty())
			{
				const int32 CurrentSampleIndex = Stack.Pop(EAllowShrinking::No);
				if (!Planet.SampleAdjacency.IsValidIndex(CurrentSampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[CurrentSampleIndex])
				{
					if (!PlateIds.IsValidIndex(NeighborIndex) ||
						Visited[NeighborIndex] != 0 ||
						PlateIds[NeighborIndex] != PlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			int32& ComponentCount = ComponentCountsByPlate.FindOrAdd(PlateId);
			++ComponentCount;
			MaxComponents = FMath::Max(MaxComponents, ComponentCount);
		}

		return MaxComponents;
	}

	int32 ComputeMaxComponentsPerPlate(const FTectonicPlanet& Planet)
	{
		TArray<int32> PlateIds;
		PlateIds.Reserve(Planet.Samples.Num());
		for (const FSample& Sample : Planet.Samples)
		{
			PlateIds.Add(Sample.PlateId);
		}
		return ComputeMaxComponentsPerPlateFromAssignments(Planet, PlateIds);
	}

	void ApplyCoherencePass(
		const FTectonicPlanet& Planet,
		TArray<int32>& InOutPlateIds,
		TArray<FTectonicPlanetV6ResolvedSample>& InOutResolvedSamples,
		const int32 MaxComponentSize,
		int32& OutReassignedSampleCount,
		int32& OutRemovedComponentCount,
		int32& OutLargestRemovedComponentSize,
		int32& OutFinalMaxComponentsPerPlate)
	{
		check(InOutPlateIds.Num() == Planet.Samples.Num());
		check(InOutResolvedSamples.Num() == Planet.Samples.Num());

		OutReassignedSampleCount = 0;
		OutRemovedComponentCount = 0;
		OutLargestRemovedComponentSize = 0;

		for (int32 PassIndex = 0; PassIndex < MaxCoherencePasses; ++PassIndex)
		{
			TArray<int32> PlateSampleCounts;
			PlateSampleCounts.Init(0, Planet.Plates.Num());
			TMap<int32, int32> PlateIdToIndex;
			for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
			{
				PlateIdToIndex.Add(Planet.Plates[PlateIndex].Id, PlateIndex);
			}
			for (const int32 PlateId : InOutPlateIds)
			{
				const int32* PlateIndexPtr = PlateIdToIndex.Find(PlateId);
				if (PlateIndexPtr != nullptr)
				{
					++PlateSampleCounts[*PlateIndexPtr];
				}
			}

			TArray<FV6CoherenceComponent> Components;
			TArray<uint8> Visited;
			Visited.Init(0, InOutPlateIds.Num());
			for (int32 SampleIndex = 0; SampleIndex < InOutPlateIds.Num(); ++SampleIndex)
			{
				const int32 PlateId = InOutPlateIds[SampleIndex];
				if (PlateId == INDEX_NONE || Visited[SampleIndex] != 0)
				{
					continue;
				}

				FV6CoherenceComponent& Component = Components.AddDefaulted_GetRef();
				Component.PlateId = PlateId;
				Component.MinSampleIndex = SampleIndex;
				TArray<int32, TInlineAllocator<256>> Stack;
				Stack.Add(SampleIndex);
				Visited[SampleIndex] = 1;

				while (!Stack.IsEmpty())
				{
					const int32 CurrentSampleIndex = Stack.Pop(EAllowShrinking::No);
					Component.Samples.Add(CurrentSampleIndex);
					Component.MinSampleIndex = FMath::Min(Component.MinSampleIndex, CurrentSampleIndex);
					if (!Planet.SampleAdjacency.IsValidIndex(CurrentSampleIndex))
					{
						continue;
					}

					for (const int32 NeighborIndex : Planet.SampleAdjacency[CurrentSampleIndex])
					{
						if (!InOutPlateIds.IsValidIndex(NeighborIndex))
						{
							continue;
						}

						const int32 NeighborPlateId = InOutPlateIds[NeighborIndex];
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
							int32& NeighborEdgeCount = Component.NeighborEdgeCounts.FindOrAdd(NeighborPlateId);
							++NeighborEdgeCount;
						}
					}
				}
			}

			Components.Sort([](const FV6CoherenceComponent& Lhs, const FV6CoherenceComponent& Rhs)
			{
				return Lhs.MinSampleIndex < Rhs.MinSampleIndex;
			});

			TArray<int32> ProposedPlateIds = InOutPlateIds;
			int32 PassReassignedSampleCount = 0;
			int32 PassRemovedComponentCount = 0;
			int32 PassLargestRemovedComponentSize = 0;

			for (const FV6CoherenceComponent& Component : Components)
			{
				const int32 ComponentSize = Component.Samples.Num();
				if (ComponentSize <= 0 || ComponentSize >= MaxComponentSize || Component.NeighborEdgeCounts.IsEmpty())
				{
					continue;
				}

				int32 BestNeighborPlateId = INDEX_NONE;
				int32 BestNeighborEdgeCount = -1;
				int32 BestNeighborSampleCount = -1;
				for (const TPair<int32, int32>& NeighborEdgeCount : Component.NeighborEdgeCounts)
				{
					const int32* NeighborPlateIndexPtr = PlateIdToIndex.Find(NeighborEdgeCount.Key);
					if (NeighborPlateIndexPtr == nullptr)
					{
						continue;
					}

					const int32 NeighborSampleCount = PlateSampleCounts[*NeighborPlateIndexPtr];
					// Reassign only into a strictly larger current-solve plate to avoid tiny-fragment swaps.
					if (NeighborSampleCount <= ComponentSize)
					{
						continue;
					}

					if (NeighborEdgeCount.Value > BestNeighborEdgeCount ||
						(NeighborEdgeCount.Value == BestNeighborEdgeCount && NeighborSampleCount > BestNeighborSampleCount) ||
						(NeighborEdgeCount.Value == BestNeighborEdgeCount &&
							NeighborSampleCount == BestNeighborSampleCount &&
							(BestNeighborPlateId == INDEX_NONE || NeighborEdgeCount.Key < BestNeighborPlateId)))
					{
						BestNeighborPlateId = NeighborEdgeCount.Key;
						BestNeighborEdgeCount = NeighborEdgeCount.Value;
						BestNeighborSampleCount = NeighborSampleCount;
					}
				}

				if (BestNeighborPlateId == INDEX_NONE || BestNeighborPlateId == Component.PlateId)
				{
					continue;
				}

				for (const int32 SampleIndex : Component.Samples)
				{
					ProposedPlateIds[SampleIndex] = BestNeighborPlateId;
					FTectonicPlanetV6ResolvedSample& ResolvedSample = InOutResolvedSamples[SampleIndex];
					ResolvedSample.PreCoherencePlateId =
						ResolvedSample.PreCoherencePlateId == INDEX_NONE ? ResolvedSample.FinalPlateId : ResolvedSample.PreCoherencePlateId;
					ResolvedSample.FinalPlateId = BestNeighborPlateId;
					ResolvedSample.bCoherenceReassigned = true;
				}

				PassReassignedSampleCount += ComponentSize;
				++PassRemovedComponentCount;
				PassLargestRemovedComponentSize = FMath::Max(PassLargestRemovedComponentSize, ComponentSize);
			}

			if (PassReassignedSampleCount == 0)
			{
				break;
			}

			InOutPlateIds = MoveTemp(ProposedPlateIds);
			OutReassignedSampleCount += PassReassignedSampleCount;
			OutRemovedComponentCount += PassRemovedComponentCount;
			OutLargestRemovedComponentSize = FMath::Max(OutLargestRemovedComponentSize, PassLargestRemovedComponentSize);
		}

		OutFinalMaxComponentsPerPlate = ComputeMaxComponentsPerPlateFromAssignments(Planet, InOutPlateIds);
	}

	bool ChooseExplicitFallback(
		const FTectonicPlanet& Planet,
		int32& OutPlateId,
		int32& OutSourceSampleIndex)
	{
		OutPlateId = INDEX_NONE;
		OutSourceSampleIndex = INDEX_NONE;
		int32 BestPlateId = TNumericLimits<int32>::Max();
		int32 BestSampleIndex = TNumericLimits<int32>::Max();

		for (const FPlate& Plate : Planet.Plates)
		{
			if (Plate.MemberSamples.IsEmpty())
			{
				continue;
			}

			int32 LocalBestSampleIndex = TNumericLimits<int32>::Max();
			for (const int32 MemberSampleIndex : Plate.MemberSamples)
			{
				LocalBestSampleIndex = FMath::Min(LocalBestSampleIndex, MemberSampleIndex);
			}

			if (Plate.Id < BestPlateId ||
				(Plate.Id == BestPlateId && LocalBestSampleIndex < BestSampleIndex))
			{
				BestPlateId = Plate.Id;
				BestSampleIndex = LocalBestSampleIndex;
			}
		}

		if (BestPlateId == TNumericLimits<int32>::Max() || BestSampleIndex == TNumericLimits<int32>::Max())
		{
			return false;
		}

		OutPlateId = BestPlateId;
		OutSourceSampleIndex = BestSampleIndex;
		return true;
	}

	const FTectonicPlanetV6OwnerCandidate* FindBestOwnerCandidateForPlate(
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const int32 PlateId)
	{
		const FTectonicPlanetV6OwnerCandidate* BestCandidate = nullptr;
		for (const FTectonicPlanetV6OwnerCandidate& Candidate : OwnerCandidates)
		{
			if (Candidate.PlateId != PlateId)
			{
				continue;
			}

			if (BestCandidate == nullptr ||
				Candidate.FitScore > BestCandidate->FitScore + TriangleEpsilon ||
				(FMath::IsNearlyEqual(Candidate.FitScore, BestCandidate->FitScore, TriangleEpsilon) &&
					Candidate.TriangleIndex < BestCandidate->TriangleIndex))
			{
				BestCandidate = &Candidate;
			}
		}

		return BestCandidate;
	}

	const FTectonicPlanetV6RecoveryCandidate* FindBestRecoveryCandidateForPlate(
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const int32 PlateId)
	{
		const FTectonicPlanetV6RecoveryCandidate* BestCandidate = nullptr;
		for (const FTectonicPlanetV6RecoveryCandidate& Candidate : RecoveryCandidates)
		{
			if (Candidate.PlateId != PlateId)
			{
				continue;
			}

			if (BestCandidate == nullptr ||
				Candidate.DistanceRadians + TriangleEpsilon < BestCandidate->DistanceRadians ||
				(FMath::IsNearlyEqual(Candidate.DistanceRadians, BestCandidate->DistanceRadians, TriangleEpsilon) &&
					Candidate.TriangleIndex < BestCandidate->TriangleIndex))
			{
				BestCandidate = &Candidate;
			}
		}

		return BestCandidate;
	}

	const FTectonicPlanetV6BoundaryMotionSample* FindBoundaryMotionSampleByPlate(
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
		const int32 PlateId)
	{
		for (const FTectonicPlanetV6BoundaryMotionSample& MotionSample : MotionSamples)
		{
			if (MotionSample.PlateId == PlateId)
			{
				return &MotionSample;
			}
		}

		return nullptr;
	}

	double ComputeBoundarySupportScore(
		const int32 PlateId,
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples)
	{
		if (PlateId == INDEX_NONE)
		{
			return -TNumericLimits<double>::Max();
		}

		double Score = 0.0;
		if (const FTectonicPlanetV6OwnerCandidate* ExactCandidate = FindBestOwnerCandidateForPlate(OwnerCandidates, PlateId))
		{
			Score += 1000.0 + (ExactCandidate->FitScore * 100.0);
		}

		if (const FTectonicPlanetV6RecoveryCandidate* RecoveryCandidate = FindBestRecoveryCandidateForPlate(RecoveryCandidates, PlateId))
		{
			Score += 100.0 - (FMath::Clamp(RecoveryCandidate->DistanceRadians, 0.0, PI) * 100.0);
		}

		if (const FTectonicPlanetV6BoundaryMotionSample* MotionSample = FindBoundaryMotionSampleByPlate(MotionSamples, PlateId))
		{
			Score += static_cast<double>(MotionSample->NeighborVoteCount * 10);
		}

		return Score;
	}

	int32 ChooseBestSupportedPlateId(
		const TArray<int32>& CandidatePlateIds,
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
		const int32 ExcludedPlateId = INDEX_NONE)
	{
		int32 BestPlateId = INDEX_NONE;
		double BestScore = -TNumericLimits<double>::Max();
		for (const int32 PlateId : CandidatePlateIds)
		{
			if (PlateId == INDEX_NONE || PlateId == ExcludedPlateId)
			{
				continue;
			}

			const double Score = ComputeBoundarySupportScore(PlateId, OwnerCandidates, RecoveryCandidates, MotionSamples);
			if (Score > BestScore + TriangleEpsilon ||
				(FMath::IsNearlyEqual(Score, BestScore, TriangleEpsilon) &&
					(BestPlateId == INDEX_NONE || PlateId < BestPlateId)))
			{
				BestPlateId = PlateId;
				BestScore = Score;
			}
		}

		return BestPlateId;
	}

	void CollectRelevantBoundaryPlateIds(
		const int32 PreviousPlateId,
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
		TArray<int32>& OutPlateIds)
	{
		OutPlateIds.Reset();
		if (PreviousPlateId != INDEX_NONE)
		{
			OutPlateIds.Add(PreviousPlateId);
		}

		for (const FTectonicPlanetV6OwnerCandidate& Candidate : OwnerCandidates)
		{
			if (Candidate.PlateId != INDEX_NONE)
			{
				OutPlateIds.AddUnique(Candidate.PlateId);
			}
		}

		for (const FTectonicPlanetV6RecoveryCandidate& Candidate : RecoveryCandidates)
		{
			if (Candidate.PlateId != INDEX_NONE)
			{
				OutPlateIds.AddUnique(Candidate.PlateId);
			}
		}

		for (const FTectonicPlanetV6BoundaryMotionSample& MotionSample : MotionSamples)
		{
			if (MotionSample.PlateId != INDEX_NONE)
			{
				OutPlateIds.AddUnique(MotionSample.PlateId);
			}
		}
	}

	bool TryComputeBoundaryNormal(
		const FVector3d& QueryPoint,
		const int32 PrimaryPlateId,
		const int32 SecondaryPlateId,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
		FVector3d& OutBoundaryNormal)
	{
		OutBoundaryNormal = FVector3d::ZeroVector;
		const FTectonicPlanetV6BoundaryMotionSample* PrimaryMotion = FindBoundaryMotionSampleByPlate(MotionSamples, PrimaryPlateId);
		const FTectonicPlanetV6BoundaryMotionSample* SecondaryMotion = FindBoundaryMotionSampleByPlate(MotionSamples, SecondaryPlateId);

		FVector3d BoundaryNormal = FVector3d::ZeroVector;
		if (PrimaryMotion != nullptr && SecondaryMotion != nullptr)
		{
			BoundaryNormal = ProjectOntoTangent(
				SecondaryMotion->NeighborTangent - PrimaryMotion->NeighborTangent,
				QueryPoint);
		}
		if (BoundaryNormal.SquaredLength() <= DirectionDegeneracyThreshold * DirectionDegeneracyThreshold &&
			SecondaryMotion != nullptr)
		{
			BoundaryNormal = ProjectOntoTangent(SecondaryMotion->NeighborTangent, QueryPoint);
		}
		if (BoundaryNormal.SquaredLength() <= DirectionDegeneracyThreshold * DirectionDegeneracyThreshold &&
			PrimaryMotion != nullptr)
		{
			BoundaryNormal = ProjectOntoTangent(-PrimaryMotion->NeighborTangent, QueryPoint);
		}
		if (BoundaryNormal.SquaredLength() <= DirectionDegeneracyThreshold * DirectionDegeneracyThreshold)
		{
			return false;
		}

		OutBoundaryNormal = BoundaryNormal.GetSafeNormal();
		return !OutBoundaryNormal.IsNearlyZero();
	}

	EV6BoundaryMotionClass ClassifyBoundaryMotion(
		const FVector3d& QueryPoint,
		const int32 PrimaryPlateId,
		const int32 SecondaryPlateId,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
		double& OutRelativeNormalVelocity)
	{
		OutRelativeNormalVelocity = 0.0;
		const FTectonicPlanetV6BoundaryMotionSample* PrimaryMotion = FindBoundaryMotionSampleByPlate(MotionSamples, PrimaryPlateId);
		const FTectonicPlanetV6BoundaryMotionSample* SecondaryMotion = FindBoundaryMotionSampleByPlate(MotionSamples, SecondaryPlateId);
		if (PrimaryMotion == nullptr || SecondaryMotion == nullptr)
		{
			return EV6BoundaryMotionClass::None;
		}

		FVector3d BoundaryNormal = FVector3d::ZeroVector;
		if (!TryComputeBoundaryNormal(QueryPoint, PrimaryPlateId, SecondaryPlateId, MotionSamples, BoundaryNormal))
		{
			return EV6BoundaryMotionClass::None;
		}

		const FVector3d RelativeVelocity = SecondaryMotion->SurfaceVelocity - PrimaryMotion->SurfaceVelocity;
		const double RelativeSpeed = RelativeVelocity.Length();
		if (RelativeSpeed <= UE_DOUBLE_SMALL_NUMBER)
		{
			return EV6BoundaryMotionClass::Weak;
		}

		OutRelativeNormalVelocity = RelativeVelocity.Dot(BoundaryNormal);
		const double NormalRatio = FMath::Abs(OutRelativeNormalVelocity) / RelativeSpeed;
		if (FMath::Abs(OutRelativeNormalVelocity) < BoundaryNormalVelocityThresholdKmPerMy ||
			NormalRatio < BoundaryNormalVelocityRatioThreshold)
		{
			return EV6BoundaryMotionClass::Weak;
		}

		return OutRelativeNormalVelocity > 0.0
			? EV6BoundaryMotionClass::Divergent
			: EV6BoundaryMotionClass::Convergent;
	}

	int32 GetActiveZoneCausePriority(const ETectonicPlanetV6ActiveZoneCause Cause)
	{
		switch (Cause)
		{
		case ETectonicPlanetV6ActiveZoneCause::CollisionContact:
			return 4;
		case ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction:
			return 3;
		case ETectonicPlanetV6ActiveZoneCause::Rift:
			return 2;
		case ETectonicPlanetV6ActiveZoneCause::DivergenceFill:
			return 1;
		case ETectonicPlanetV6ActiveZoneCause::GenericQueryCompetition:
			return 0;
		case ETectonicPlanetV6ActiveZoneCause::UnknownOther:
			return -1;
		case ETectonicPlanetV6ActiveZoneCause::None:
		default:
			return -2;
		}
	}

	ETectonicPlanetV6ActiveZoneCause ChooseDominantActivePairCause(
		const FV9Phase1ActivePairAggregate& Aggregate,
		const ETectonicPlanetV6ActiveZoneCause FallbackCause)
	{
		ETectonicPlanetV6ActiveZoneCause BestCause =
			FallbackCause != ETectonicPlanetV6ActiveZoneCause::None
				? FallbackCause
				: ETectonicPlanetV6ActiveZoneCause::UnknownOther;
		int32 BestCount = -1;

		const auto ConsiderCause = [&](const ETectonicPlanetV6ActiveZoneCause Cause, const int32 Count)
		{
			if (Count > BestCount ||
				(Count == BestCount && GetActiveZoneCausePriority(Cause) > GetActiveZoneCausePriority(BestCause)))
			{
				BestCount = Count;
				BestCause = Cause;
			}
		};

		ConsiderCause(ETectonicPlanetV6ActiveZoneCause::DivergenceFill, Aggregate.DivergenceCount);
		ConsiderCause(
			ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction,
			Aggregate.ConvergentSubductionCount);
		ConsiderCause(
			ETectonicPlanetV6ActiveZoneCause::CollisionContact,
			Aggregate.CollisionContactCount);
		ConsiderCause(ETectonicPlanetV6ActiveZoneCause::Rift, Aggregate.RiftCount);
		return BestCause;
	}

	bool HasContinentalNeighborForPlate(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		const int32 SampleIndex,
		const int32 PlateId)
	{
		if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex) || PlateId == INDEX_NONE)
		{
			return false;
		}

		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!Planet.Samples.IsValidIndex(NeighborIndex) || !PlateIds.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			if (PlateIds[NeighborIndex] == PlateId &&
				Planet.Samples[NeighborIndex].ContinentalWeight >= 0.5f)
			{
				return true;
			}
		}

		return false;
	}

	ETectonicPlanetV6ActiveZoneCause ClassifyActiveZoneCauseForPlatePair(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		const int32 SampleIndex,
		const int32 PrimaryPlateId,
		const int32 SecondaryPlateId,
		const EV6BoundaryMotionClass MotionClass)
	{
		if (MotionClass == EV6BoundaryMotionClass::Divergent)
		{
			return ETectonicPlanetV6ActiveZoneCause::DivergenceFill;
		}
		if (MotionClass != EV6BoundaryMotionClass::Convergent)
		{
			return ETectonicPlanetV6ActiveZoneCause::None;
		}

		const bool bPrimaryContinental = Planet.Samples.IsValidIndex(SampleIndex) &&
			Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f;
		const bool bSecondaryContinental =
			HasContinentalNeighborForPlate(Planet, PlateIds, SampleIndex, SecondaryPlateId);
		return
			(bPrimaryContinental && bSecondaryContinental)
				? ETectonicPlanetV6ActiveZoneCause::CollisionContact
				: ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction;
	}

	bool TryBuildDominantActivePairContextForBoundarySample(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		const int32 SampleIndex,
		FV9Phase1ActivePairContext& OutContext)
	{
		OutContext = FV9Phase1ActivePairContext{};
		if (!Planet.Samples.IsValidIndex(SampleIndex) ||
			!PlateIds.IsValidIndex(SampleIndex) ||
			!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			return false;
		}

		const int32 PrimaryPlateId = PlateIds[SampleIndex];
		if (PrimaryPlateId == INDEX_NONE)
		{
			return false;
		}

		TArray<FV6BoundaryNeighborSummary, TInlineAllocator<8>> NeighborSummaries;
		GatherBoundaryNeighborSummariesForPlateIds(Planet, PlateIds, SampleIndex, NeighborSummaries);
		const FV6BoundaryNeighborSummary* PrimarySummary =
			FindBoundaryNeighborSummary(NeighborSummaries, PrimaryPlateId);

		const FV6BoundaryNeighborSummary* DominantForeignSummary = nullptr;
		int32 TotalForeignVotes = 0;
		int32 RunnerUpForeignVotes = 0;
		int32 ForeignPlateCount = 0;
		for (const FV6BoundaryNeighborSummary& Summary : NeighborSummaries)
		{
			if (Summary.PlateId == PrimaryPlateId)
			{
				continue;
			}

			++ForeignPlateCount;
			TotalForeignVotes += Summary.VoteCount;
			if (DominantForeignSummary == nullptr ||
				Summary.VoteCount > DominantForeignSummary->VoteCount ||
				(Summary.VoteCount == DominantForeignSummary->VoteCount &&
					Summary.PlateId < DominantForeignSummary->PlateId))
			{
				RunnerUpForeignVotes =
					DominantForeignSummary != nullptr ? DominantForeignSummary->VoteCount : RunnerUpForeignVotes;
				DominantForeignSummary = &Summary;
			}
			else
			{
				RunnerUpForeignVotes = FMath::Max(RunnerUpForeignVotes, Summary.VoteCount);
			}
		}

		constexpr int32 NarrowMinLocalSecondaryVotes = 2;
		constexpr double NarrowStrongVelocityThresholdKmPerMy =
			BoundaryNormalVelocityThresholdKmPerMy * 1.5;
		if (DominantForeignSummary == nullptr ||
			DominantForeignSummary->VoteCount < NarrowMinLocalSecondaryVotes ||
			DominantForeignSummary->VoteCount <= RunnerUpForeignVotes ||
			DominantForeignSummary->VoteCount <= TotalForeignVotes - DominantForeignSummary->VoteCount)
		{
			return false;
		}

		TArray<int32> RelevantPlateIds;
		RelevantPlateIds.Add(PrimaryPlateId);
		RelevantPlateIds.Add(DominantForeignSummary->PlateId);
		TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
		BuildBoundaryMotionSamplesForPlateIds(
			Planet,
			PlateIds,
			SampleIndex,
			RelevantPlateIds,
			MotionSamples);

		double RelativeNormalVelocity = 0.0;
		const EV6BoundaryMotionClass MotionClass =
			ClassifyBoundaryMotion(
				Planet.Samples[SampleIndex].Position,
				PrimaryPlateId,
				DominantForeignSummary->PlateId,
				MotionSamples,
				RelativeNormalVelocity);
		if ((MotionClass != EV6BoundaryMotionClass::Divergent &&
				MotionClass != EV6BoundaryMotionClass::Convergent) ||
			FMath::Abs(RelativeNormalVelocity) < NarrowStrongVelocityThresholdKmPerMy)
		{
			return false;
		}

		const ETectonicPlanetV6ActiveZoneCause Cause =
			ClassifyActiveZoneCauseForPlatePair(
				Planet,
				PlateIds,
				SampleIndex,
				PrimaryPlateId,
				DominantForeignSummary->PlateId,
				MotionClass);
		if (Cause == ETectonicPlanetV6ActiveZoneCause::None)
		{
			return false;
		}

		OutContext.bValid = true;
		OutContext.PrimaryPlateId = PrimaryPlateId;
		OutContext.SecondaryPlateId = DominantForeignSummary->PlateId;
		OutContext.PrimaryVoteCount = PrimarySummary != nullptr ? PrimarySummary->VoteCount : 0;
		OutContext.SecondaryVoteCount = DominantForeignSummary->VoteCount;
		OutContext.ForeignPlateCount = ForeignPlateCount;
		OutContext.AbsRelativeNormalVelocity = FMath::Abs(RelativeNormalVelocity);
		OutContext.Cause = Cause;
		return true;
	}

	void BuildPhase1ActiveZoneMask(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateIds,
		const ETectonicPlanetV6ActiveZoneClassifierMode ClassifierMode,
		const int32 ActiveBoundaryRingCount,
		const int32 PersistentActivePairHorizon,
		TMap<uint64, int32>& InOutPersistentPairRemainingSolveIntervals,
		TMap<uint64, uint8>& InOutPersistentPairCauseValues,
		TArray<uint8>& OutActiveFlags,
		TArray<uint8>& OutSeedFlags,
		TArray<uint8>& OutCarryoverFlags,
		TArray<uint8>& OutFreshExpandedFlags,
		TArray<uint8>& OutCarryoverExpandedFlags,
		TArray<uint8>& OutCauseValues,
		TArray<int32>& OutPrimaryPlateIds,
		TArray<int32>& OutSecondaryPlateIds,
		int32& OutActivePairCount,
		int32& OutFreshSeedActivePairCount,
		int32& OutPersistentCarryoverActivePairCount,
		int32& OutFreshPairCandidateCount,
		int32& OutFreshPairAdmittedCount,
		int32& OutFreshPairRejectedSupportCount,
		int32& OutFreshPairRejectedVelocityCount,
		int32& OutFreshPairRejectedDominanceCount,
		int32& OutFreshPairAdmittedDivergenceCount,
		int32& OutFreshPairAdmittedConvergentSubductionCount,
		int32& OutFreshPairAdmittedCollisionContactCount,
		int32& OutFreshPairAdmittedRiftCount)
	{
		const int32 SampleCount = Planet.Samples.Num();
		OutActiveFlags.Init(0, SampleCount);
		OutSeedFlags.Init(0, SampleCount);
		OutCarryoverFlags.Init(0, SampleCount);
		OutFreshExpandedFlags.Init(0, SampleCount);
		OutCarryoverExpandedFlags.Init(0, SampleCount);
		OutCauseValues.Init(static_cast<uint8>(ETectonicPlanetV6ActiveZoneCause::None), SampleCount);
		OutPrimaryPlateIds.Init(INDEX_NONE, SampleCount);
		OutSecondaryPlateIds.Init(INDEX_NONE, SampleCount);
		OutActivePairCount = 0;
		OutFreshSeedActivePairCount = 0;
		OutPersistentCarryoverActivePairCount = 0;
		OutFreshPairCandidateCount = 0;
		OutFreshPairAdmittedCount = 0;
		OutFreshPairRejectedSupportCount = 0;
		OutFreshPairRejectedVelocityCount = 0;
		OutFreshPairRejectedDominanceCount = 0;
		OutFreshPairAdmittedDivergenceCount = 0;
		OutFreshPairAdmittedConvergentSubductionCount = 0;
		OutFreshPairAdmittedCollisionContactCount = 0;
		OutFreshPairAdmittedRiftCount = 0;

		if (SampleCount == 0 || PlateIds.Num() != SampleCount)
		{
			InOutPersistentPairRemainingSolveIntervals.Reset();
			InOutPersistentPairCauseValues.Reset();
			return;
		}

		if (ClassifierMode == ETectonicPlanetV6ActiveZoneClassifierMode::BroadBoundaryBand)
		{
			InOutPersistentPairRemainingSolveIntervals.Reset();
			InOutPersistentPairCauseValues.Reset();
			TArray<int32> Frontier;
			Frontier.Reserve(SampleCount / 8);
			for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
			{
				FV9Phase1ActivePairContext Context;
				if (!TryBuildDominantActivePairContextForBoundarySample(Planet, PlateIds, SampleIndex, Context))
				{
					continue;
				}

				OutActiveFlags[SampleIndex] = 1;
				OutSeedFlags[SampleIndex] = 1;
				OutCauseValues[SampleIndex] = static_cast<uint8>(Context.Cause);
				OutPrimaryPlateIds[SampleIndex] = Context.PrimaryPlateId;
				OutSecondaryPlateIds[SampleIndex] = Context.SecondaryPlateId;
				Frontier.Add(SampleIndex);
			}

			if (ActiveBoundaryRingCount <= 0)
			{
				return;
			}

			for (int32 Ring = 0; Ring < ActiveBoundaryRingCount; ++Ring)
			{
				TArray<int32> NextFrontier;
				for (const int32 SampleIndex : Frontier)
				{
					if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
					{
						continue;
					}

					const ETectonicPlanetV6ActiveZoneCause PropagatedCause =
						static_cast<ETectonicPlanetV6ActiveZoneCause>(OutCauseValues[SampleIndex]);
					for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
					{
						if (!Planet.Samples.IsValidIndex(NeighborIndex))
						{
							continue;
						}

						if (OutActiveFlags[NeighborIndex] == 0)
						{
							OutActiveFlags[NeighborIndex] = 1;
							OutCauseValues[NeighborIndex] = static_cast<uint8>(PropagatedCause);
							OutPrimaryPlateIds[NeighborIndex] = OutPrimaryPlateIds[SampleIndex];
							OutSecondaryPlateIds[NeighborIndex] = OutSecondaryPlateIds[SampleIndex];
							NextFrontier.Add(NeighborIndex);
							continue;
						}

						const ETectonicPlanetV6ActiveZoneCause ExistingCause =
							static_cast<ETectonicPlanetV6ActiveZoneCause>(OutCauseValues[NeighborIndex]);
						if (GetActiveZoneCausePriority(PropagatedCause) >
							GetActiveZoneCausePriority(ExistingCause))
						{
							OutCauseValues[NeighborIndex] = static_cast<uint8>(PropagatedCause);
							OutPrimaryPlateIds[NeighborIndex] = OutPrimaryPlateIds[SampleIndex];
							OutSecondaryPlateIds[NeighborIndex] = OutSecondaryPlateIds[SampleIndex];
						}
					}
				}

				Frontier = MoveTemp(NextFrontier);
				if (Frontier.IsEmpty())
				{
					break;
				}
			}
			return;
		}

		if (ClassifierMode == ETectonicPlanetV6ActiveZoneClassifierMode::NarrowTectonicPairs)
		{
			InOutPersistentPairRemainingSolveIntervals.Reset();
			InOutPersistentPairCauseValues.Reset();
		}

		TArray<FV9Phase1ActivePairContext> LocalContexts;
		LocalContexts.SetNum(SampleCount);
		TMap<uint64, FV9Phase1ActivePairAggregate> PairAggregates;

		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			FV9Phase1ActivePairContext& Context = LocalContexts[SampleIndex];
			if (!TryBuildDominantActivePairContextForBoundarySample(Planet, PlateIds, SampleIndex, Context))
			{
				continue;
			}

			const uint64 PairKey =
				MakeOrderedPlatePairKey(Context.PrimaryPlateId, Context.SecondaryPlateId);
			FV9Phase1ActivePairAggregate& Aggregate = PairAggregates.FindOrAdd(PairKey);
			++Aggregate.SampleCount;
			Aggregate.MaxSecondaryVoteCount = FMath::Max(Aggregate.MaxSecondaryVoteCount, Context.SecondaryVoteCount);
			Aggregate.SumAbsRelativeNormalVelocity += Context.AbsRelativeNormalVelocity;
			switch (Context.Cause)
			{
			case ETectonicPlanetV6ActiveZoneCause::DivergenceFill:
				++Aggregate.DivergenceCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction:
				++Aggregate.ConvergentSubductionCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::CollisionContact:
				++Aggregate.CollisionContactCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::Rift:
				++Aggregate.RiftCount;
				break;
			default:
				break;
			}
		}

		if (ClassifierMode == ETectonicPlanetV6ActiveZoneClassifierMode::NarrowTectonicPairs)
		{
			TSet<uint64> ActivePairs;
			constexpr int32 NarrowMinPairSupportSamples = 96;
			constexpr double NarrowMeanVelocityThresholdKmPerMy =
				BoundaryNormalVelocityThresholdKmPerMy * 1.75;
			for (const TPair<uint64, FV9Phase1ActivePairAggregate>& PairEntry : PairAggregates)
			{
				const FV9Phase1ActivePairAggregate& Aggregate = PairEntry.Value;
				const double MeanAbsVelocity =
					Aggregate.SampleCount > 0
						? Aggregate.SumAbsRelativeNormalVelocity / static_cast<double>(Aggregate.SampleCount)
						: 0.0;
				if (Aggregate.SampleCount >= NarrowMinPairSupportSamples &&
					Aggregate.MaxSecondaryVoteCount >= 2 &&
					MeanAbsVelocity >= NarrowMeanVelocityThresholdKmPerMy)
				{
					ActivePairs.Add(PairEntry.Key);
				}
			}

			TArray<int32> Frontier;
			Frontier.Reserve(SampleCount / 16);
			for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
			{
				const FV9Phase1ActivePairContext& Context = LocalContexts[SampleIndex];
				if (!Context.bValid)
				{
					continue;
				}

				const uint64 PairKey = MakeOrderedPlatePairKey(Context.PrimaryPlateId, Context.SecondaryPlateId);
				if (!ActivePairs.Contains(PairKey))
				{
					continue;
				}

				OutActiveFlags[SampleIndex] = 1;
				OutSeedFlags[SampleIndex] = 1;
				OutCauseValues[SampleIndex] = static_cast<uint8>(Context.Cause);
				OutPrimaryPlateIds[SampleIndex] = Context.PrimaryPlateId;
				OutSecondaryPlateIds[SampleIndex] = Context.SecondaryPlateId;
				Frontier.Add(SampleIndex);
			}

			OutActivePairCount = ActivePairs.Num();
			OutFreshSeedActivePairCount = ActivePairs.Num();
			if (ActiveBoundaryRingCount <= 0)
			{
				return;
			}

			for (int32 Ring = 0; Ring < ActiveBoundaryRingCount; ++Ring)
			{
				TArray<int32> NextFrontier;
				for (const int32 SampleIndex : Frontier)
				{
					if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
					{
						continue;
					}

					const uint64 PairKey =
						MakeOrderedPlatePairKey(
							OutPrimaryPlateIds[SampleIndex],
							OutSecondaryPlateIds[SampleIndex]);
					for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
					{
						if (!Planet.Samples.IsValidIndex(NeighborIndex) || OutActiveFlags[NeighborIndex] != 0)
						{
							continue;
						}

						const FV9Phase1ActivePairContext& NeighborContext = LocalContexts[NeighborIndex];
						if (!NeighborContext.bValid ||
							MakeOrderedPlatePairKey(
								NeighborContext.PrimaryPlateId,
								NeighborContext.SecondaryPlateId) != PairKey)
						{
							continue;
						}

						OutActiveFlags[NeighborIndex] = 1;
						OutCauseValues[NeighborIndex] = static_cast<uint8>(NeighborContext.Cause);
						OutPrimaryPlateIds[NeighborIndex] = NeighborContext.PrimaryPlateId;
						OutSecondaryPlateIds[NeighborIndex] = NeighborContext.SecondaryPlateId;
						NextFrontier.Add(NeighborIndex);
					}
				}

				Frontier = MoveTemp(NextFrontier);
				if (Frontier.IsEmpty())
				{
					break;
				}
			}
			return;
		}

		const bool bTightFreshAdmission =
			ClassifierMode == ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission;
		TMap<uint64, FV9Phase1PersistentActivePairState> ActivePairs;
		const int32 PersistentFreshMinPairSupportSamples = bTightFreshAdmission ? 40 : 24;
		const int32 PersistentFreshMinSecondaryVoteCount = bTightFreshAdmission ? 3 : 2;
		const double PersistentFreshMeanVelocityThresholdKmPerMy =
			BoundaryNormalVelocityThresholdKmPerMy * (bTightFreshAdmission ? 1.35 : 1.15);
		const double PersistentFreshDominanceFractionThreshold = bTightFreshAdmission ? 0.55 : 0.0;
		const int32 PersistentFreshDominanceMinimumCount = bTightFreshAdmission ? 10 : 0;
		constexpr int32 PersistentCarryoverMinPairSupportSamples = 6;
		constexpr double PersistentCarryoverMeanVelocityThresholdKmPerMy =
			BoundaryNormalVelocityThresholdKmPerMy * 0.75;
		const auto CountFreshAdmissionCause =
			[&](
				const ETectonicPlanetV6ActiveZoneCause Cause)
		{
			switch (Cause)
			{
			case ETectonicPlanetV6ActiveZoneCause::DivergenceFill:
				++OutFreshPairAdmittedDivergenceCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction:
				++OutFreshPairAdmittedConvergentSubductionCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::CollisionContact:
				++OutFreshPairAdmittedCollisionContactCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::Rift:
				++OutFreshPairAdmittedRiftCount;
				break;
			default:
				break;
			}
		};

		for (const TPair<uint64, FV9Phase1ActivePairAggregate>& PairEntry : PairAggregates)
		{
			++OutFreshPairCandidateCount;
			const FV9Phase1ActivePairAggregate& Aggregate = PairEntry.Value;
			const double MeanAbsVelocity =
				Aggregate.SampleCount > 0
					? Aggregate.SumAbsRelativeNormalVelocity / static_cast<double>(Aggregate.SampleCount)
					: 0.0;
			if (Aggregate.SampleCount < PersistentFreshMinPairSupportSamples ||
				Aggregate.MaxSecondaryVoteCount < PersistentFreshMinSecondaryVoteCount)
			{
				++OutFreshPairRejectedSupportCount;
				continue;
			}

			if (MeanAbsVelocity < PersistentFreshMeanVelocityThresholdKmPerMy)
			{
				++OutFreshPairRejectedVelocityCount;
				continue;
			}

			const ETectonicPlanetV6ActiveZoneCause DominantCause =
				ChooseDominantActivePairCause(
					Aggregate,
					ETectonicPlanetV6ActiveZoneCause::UnknownOther);
			const int32 DominantSupportCount =
				FMath::Max(
					FMath::Max(Aggregate.DivergenceCount, Aggregate.ConvergentSubductionCount),
					FMath::Max(Aggregate.CollisionContactCount, Aggregate.RiftCount));
			const int32 RequiredDominantSupportCount =
				FMath::Max(
					PersistentFreshDominanceMinimumCount,
					FMath::CeilToInt(static_cast<double>(Aggregate.SampleCount) *
						PersistentFreshDominanceFractionThreshold));
			if (bTightFreshAdmission &&
				DominantSupportCount < RequiredDominantSupportCount)
			{
				++OutFreshPairRejectedDominanceCount;
				continue;
			}

			FV9Phase1PersistentActivePairState& PairState = ActivePairs.FindOrAdd(PairEntry.Key);
			PairState.RemainingSolveIntervals = PersistentActivePairHorizon;
			PairState.Cause = DominantCause;
			PairState.bFreshSeed = true;
			++OutFreshPairAdmittedCount;
			CountFreshAdmissionCause(DominantCause);
		}

		TMap<uint64, int32> NextPersistentPairRemainingSolveIntervals;
		TMap<uint64, uint8> NextPersistentPairCauseValues;
		for (const TPair<uint64, FV9Phase1PersistentActivePairState>& PairEntry : ActivePairs)
		{
			NextPersistentPairRemainingSolveIntervals.Add(
				PairEntry.Key,
				PairEntry.Value.RemainingSolveIntervals);
			NextPersistentPairCauseValues.Add(
				PairEntry.Key,
				static_cast<uint8>(PairEntry.Value.Cause));
		}

		for (const TPair<uint64, int32>& PairEntry : InOutPersistentPairRemainingSolveIntervals)
		{
			if (ActivePairs.Contains(PairEntry.Key))
			{
				continue;
			}

			const int32 RemainingSolveIntervals = PairEntry.Value - 1;
			if (RemainingSolveIntervals <= 0)
			{
				continue;
			}

			const FV9Phase1ActivePairAggregate* Aggregate = PairAggregates.Find(PairEntry.Key);
			if (Aggregate == nullptr)
			{
				continue;
			}

			const double MeanAbsVelocity =
				Aggregate->SampleCount > 0
					? Aggregate->SumAbsRelativeNormalVelocity / static_cast<double>(Aggregate->SampleCount)
					: 0.0;
			if (Aggregate->SampleCount < PersistentCarryoverMinPairSupportSamples ||
				Aggregate->MaxSecondaryVoteCount < 1 ||
				MeanAbsVelocity < PersistentCarryoverMeanVelocityThresholdKmPerMy)
			{
				continue;
			}

			FV9Phase1PersistentActivePairState& PairState = ActivePairs.FindOrAdd(PairEntry.Key);
			PairState.RemainingSolveIntervals = RemainingSolveIntervals;
			PairState.Cause =
				ChooseDominantActivePairCause(
					*Aggregate,
					static_cast<ETectonicPlanetV6ActiveZoneCause>(
						InOutPersistentPairCauseValues.FindRef(PairEntry.Key)));
			PairState.bFreshSeed = false;

			NextPersistentPairRemainingSolveIntervals.Add(PairEntry.Key, RemainingSolveIntervals);
			NextPersistentPairCauseValues.Add(
				PairEntry.Key,
				static_cast<uint8>(PairState.Cause));
		}

		InOutPersistentPairRemainingSolveIntervals = MoveTemp(NextPersistentPairRemainingSolveIntervals);
		InOutPersistentPairCauseValues = MoveTemp(NextPersistentPairCauseValues);

		OutActivePairCount = ActivePairs.Num();
		for (const TPair<uint64, FV9Phase1PersistentActivePairState>& PairEntry : ActivePairs)
		{
			if (PairEntry.Value.bFreshSeed)
			{
				++OutFreshSeedActivePairCount;
			}
			else
			{
				++OutPersistentCarryoverActivePairCount;
			}
		}

		TArray<int32> Frontier;
		Frontier.Reserve(SampleCount / 16);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			const FV9Phase1ActivePairContext& Context = LocalContexts[SampleIndex];
			if (!Context.bValid)
			{
				continue;
			}

			const uint64 PairKey = MakeOrderedPlatePairKey(Context.PrimaryPlateId, Context.SecondaryPlateId);
			const FV9Phase1PersistentActivePairState* PairState = ActivePairs.Find(PairKey);
			if (PairState == nullptr)
			{
				continue;
			}

			OutActiveFlags[SampleIndex] = 1;
			OutSeedFlags[SampleIndex] = PairState->bFreshSeed ? 1 : 0;
			OutCarryoverFlags[SampleIndex] = PairState->bFreshSeed ? 0 : 1;
			OutCauseValues[SampleIndex] = static_cast<uint8>(Context.Cause);
			OutPrimaryPlateIds[SampleIndex] = Context.PrimaryPlateId;
			OutSecondaryPlateIds[SampleIndex] = Context.SecondaryPlateId;
			Frontier.Add(SampleIndex);
		}

		if (ActiveBoundaryRingCount <= 0)
		{
			return;
		}

		for (int32 Ring = 0; Ring < ActiveBoundaryRingCount; ++Ring)
		{
			TArray<int32> NextFrontier;
			for (const int32 SampleIndex : Frontier)
			{
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				const int32 PrimaryPlateId = OutPrimaryPlateIds[SampleIndex];
				const int32 SecondaryPlateId = OutSecondaryPlateIds[SampleIndex];
				const uint64 PairKey = MakeOrderedPlatePairKey(PrimaryPlateId, SecondaryPlateId);
				const FV9Phase1PersistentActivePairState* PairState = ActivePairs.Find(PairKey);
				if (PairState == nullptr)
				{
					continue;
				}
				if (bTightFreshAdmission && PairState->bFreshSeed)
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) || OutActiveFlags[NeighborIndex] != 0)
					{
						continue;
					}

					if (!PlateIds.IsValidIndex(NeighborIndex))
					{
						continue;
					}

					const int32 NeighborPlateId = PlateIds[NeighborIndex];
					if (NeighborPlateId != PrimaryPlateId && NeighborPlateId != SecondaryPlateId)
					{
						continue;
					}

					const FV9Phase1ActivePairContext& NeighborContext = LocalContexts[NeighborIndex];
					if (NeighborContext.bValid &&
						MakeOrderedPlatePairKey(
							NeighborContext.PrimaryPlateId,
							NeighborContext.SecondaryPlateId) != PairKey)
					{
						continue;
					}

					OutActiveFlags[NeighborIndex] = 1;
					OutCarryoverFlags[NeighborIndex] = PairState->bFreshSeed ? 0 : 1;
					OutFreshExpandedFlags[NeighborIndex] = PairState->bFreshSeed ? 1 : 0;
					OutCarryoverExpandedFlags[NeighborIndex] = PairState->bFreshSeed ? 0 : 1;
					OutCauseValues[NeighborIndex] =
						NeighborContext.bValid
							? static_cast<uint8>(NeighborContext.Cause)
							: static_cast<uint8>(PairState->Cause);
					OutPrimaryPlateIds[NeighborIndex] = PrimaryPlateId;
					OutSecondaryPlateIds[NeighborIndex] = SecondaryPlateId;
					NextFrontier.Add(NeighborIndex);
				}
			}

			Frontier = MoveTemp(NextFrontier);
				if (Frontier.IsEmpty())
				{
					break;
				}
			}
			return;
		}

	int32 ChooseAdvancingPlateId(
		const FVector3d& QueryPoint,
		const int32 PrimaryPlateId,
		const int32 SecondaryPlateId,
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples)
	{
		FVector3d BoundaryNormal = FVector3d::ZeroVector;
		if (!TryComputeBoundaryNormal(QueryPoint, PrimaryPlateId, SecondaryPlateId, MotionSamples, BoundaryNormal))
		{
			const TArray<int32> CandidateIds = { PrimaryPlateId, SecondaryPlateId };
			return ChooseBestSupportedPlateId(CandidateIds, OwnerCandidates, RecoveryCandidates, MotionSamples);
		}

		const FTectonicPlanetV6BoundaryMotionSample* PrimaryMotion = FindBoundaryMotionSampleByPlate(MotionSamples, PrimaryPlateId);
		const FTectonicPlanetV6BoundaryMotionSample* SecondaryMotion = FindBoundaryMotionSampleByPlate(MotionSamples, SecondaryPlateId);
		const double PrimaryAdvance = PrimaryMotion != nullptr
			? PrimaryMotion->SurfaceVelocity.Dot(BoundaryNormal)
			: -TNumericLimits<double>::Max();
		const double SecondaryAdvance = SecondaryMotion != nullptr
			? -SecondaryMotion->SurfaceVelocity.Dot(BoundaryNormal)
			: -TNumericLimits<double>::Max();
		if (SecondaryAdvance > PrimaryAdvance + TriangleEpsilon)
		{
			return SecondaryPlateId;
		}
		if (PrimaryAdvance > SecondaryAdvance + TriangleEpsilon)
		{
			return PrimaryPlateId;
		}

		const TArray<int32> CandidateIds = { PrimaryPlateId, SecondaryPlateId };
		return ChooseBestSupportedPlateId(CandidateIds, OwnerCandidates, RecoveryCandidates, MotionSamples);
	}

	bool HasStrongPreviousOwnerEvidence(
		const int32 PreviousPlateId,
		const int32 CompetitorPlateId,
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples)
	{
		if (PreviousPlateId == INDEX_NONE)
		{
			return false;
		}

		if (FindBestOwnerCandidateForPlate(OwnerCandidates, PreviousPlateId) != nullptr)
		{
			return true;
		}

		const FTectonicPlanetV6RecoveryCandidate* PreviousRecovery =
			FindBestRecoveryCandidateForPlate(RecoveryCandidates, PreviousPlateId);
		const FTectonicPlanetV6RecoveryCandidate* CompetitorRecovery =
			FindBestRecoveryCandidateForPlate(RecoveryCandidates, CompetitorPlateId);
		if (PreviousRecovery != nullptr &&
			(CompetitorRecovery == nullptr ||
				PreviousRecovery->DistanceRadians <= CompetitorRecovery->DistanceRadians + BoundaryRecoverySlackRadians))
		{
			return true;
		}

		const FTectonicPlanetV6BoundaryMotionSample* PreviousMotion =
			FindBoundaryMotionSampleByPlate(MotionSamples, PreviousPlateId);
		const FTectonicPlanetV6BoundaryMotionSample* CompetitorMotion =
			FindBoundaryMotionSampleByPlate(MotionSamples, CompetitorPlateId);
		return PreviousMotion != nullptr &&
			PreviousMotion->NeighborVoteCount >= BoundaryStrongNeighborVoteCount &&
			PreviousMotion->NeighborVoteCount >= (CompetitorMotion != nullptr ? CompetitorMotion->NeighborVoteCount : 0);
	}

	void AssignResolvedSourceForPlate(
		FTectonicPlanetV6ResolvedSample& InOutResolved,
		const int32 WinningPlateId,
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const int32 NearestMemberFallbackPlateId,
		const int32 NearestMemberFallbackSampleIndex,
		const double NearestMemberFallbackDistance)
	{
		InOutResolved.FinalPlateId = WinningPlateId;
		if (const FTectonicPlanetV6OwnerCandidate* ExactCandidate = FindBestOwnerCandidateForPlate(OwnerCandidates, WinningPlateId))
		{
			InOutResolved.SourceTriangleIndex = ExactCandidate->TriangleIndex;
			InOutResolved.SourceBarycentric = ExactCandidate->Barycentric;
			InOutResolved.WinningFitScore = ExactCandidate->FitScore;
			return;
		}

		if (const FTectonicPlanetV6RecoveryCandidate* RecoveryCandidate = FindBestRecoveryCandidateForPlate(RecoveryCandidates, WinningPlateId))
		{
			InOutResolved.SourceTriangleIndex = RecoveryCandidate->TriangleIndex;
			InOutResolved.SourceBarycentric = RecoveryCandidate->Barycentric;
			InOutResolved.RecoveryDistanceRadians = RecoveryCandidate->DistanceRadians;
			return;
		}

		if (WinningPlateId == NearestMemberFallbackPlateId && NearestMemberFallbackSampleIndex != INDEX_NONE)
		{
			InOutResolved.SourceCanonicalSampleIndex = NearestMemberFallbackSampleIndex;
			InOutResolved.RecoveryDistanceRadians = NearestMemberFallbackDistance;
		}
	}

	int32 ChooseOceanicOwnerPlateId(
		const int32 PreviousPlateId,
		const int32 CompetitorPlateId,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples)
	{
		const FTectonicPlanetV6RecoveryCandidate* PreviousRecovery =
			FindBestRecoveryCandidateForPlate(RecoveryCandidates, PreviousPlateId);
		const FTectonicPlanetV6RecoveryCandidate* CompetitorRecovery =
			FindBestRecoveryCandidateForPlate(RecoveryCandidates, CompetitorPlateId);
		if (PreviousRecovery != nullptr || CompetitorRecovery != nullptr)
		{
			if (PreviousRecovery == nullptr)
			{
				return CompetitorPlateId;
			}
			if (CompetitorRecovery == nullptr)
			{
				return PreviousPlateId;
			}

			if (PreviousRecovery->DistanceRadians < CompetitorRecovery->DistanceRadians - TriangleEpsilon)
			{
				return PreviousPlateId;
			}
			if (CompetitorRecovery->DistanceRadians < PreviousRecovery->DistanceRadians - TriangleEpsilon)
			{
				return CompetitorPlateId;
			}
		}

		const FTectonicPlanetV6BoundaryMotionSample* PreviousMotion =
			FindBoundaryMotionSampleByPlate(MotionSamples, PreviousPlateId);
		const FTectonicPlanetV6BoundaryMotionSample* CompetitorMotion =
			FindBoundaryMotionSampleByPlate(MotionSamples, CompetitorPlateId);
		if (PreviousMotion != nullptr || CompetitorMotion != nullptr)
		{
			const int32 PreviousVotes = PreviousMotion != nullptr ? PreviousMotion->NeighborVoteCount : -1;
			const int32 CompetitorVotes = CompetitorMotion != nullptr ? CompetitorMotion->NeighborVoteCount : -1;
			if (PreviousVotes > CompetitorVotes)
			{
				return PreviousPlateId;
			}
			if (CompetitorVotes > PreviousVotes)
			{
				return CompetitorPlateId;
			}
		}

		if (PreviousPlateId != INDEX_NONE)
		{
			return PreviousPlateId;
		}
		if (CompetitorPlateId != INDEX_NONE)
		{
			return CompetitorPlateId;
		}
		return INDEX_NONE;
	}

	void FinalizeBoundaryResolvedSample(
		FTectonicPlanetV6ResolvedSample& InOutResolved,
		const int32 WinningPlateId,
		const int32 OtherPlateId,
		const ETectonicPlanetV6ResolutionKind ResolutionKind,
		const ETectonicPlanetV6BoundaryOutcome BoundaryOutcome,
		const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
		const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
		const int32 NearestMemberFallbackPlateId,
		const int32 NearestMemberFallbackSampleIndex,
		const double NearestMemberFallbackDistance)
	{
		InOutResolved.ResolutionKind = ResolutionKind;
		InOutResolved.BoundaryOutcome = BoundaryOutcome;
		InOutResolved.BoundaryOtherPlateId = OtherPlateId;
		InOutResolved.bBoundaryDecision = true;
		AssignResolvedSourceForPlate(
			InOutResolved,
			WinningPlateId,
			OwnerCandidates,
			RecoveryCandidates,
			NearestMemberFallbackPlateId,
			NearestMemberFallbackSampleIndex,
			NearestMemberFallbackDistance);
	}

		FTectonicPlanetV6ResolvedSample ResolveBoundaryState(
			const int32 PreviousPlateId,
			const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
			const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
			const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
			const FVector3d& QueryPoint,
			const bool bPreviouslyContinental,
			const int32 NearestMemberFallbackPlateId,
			const int32 NearestMemberFallbackSampleIndex,
			const double NearestMemberFallbackDistance,
			const int32 ExplicitFallbackPlateId,
		const int32 ExplicitFallbackSampleIndex)
	{
		FTectonicPlanetV6ResolvedSample Resolved;
		Resolved.PreviousPlateId = PreviousPlateId;
		Resolved.ExactCandidateCount = OwnerCandidates.Num();

		TArray<int32> RelevantPlateIds;
		CollectRelevantBoundaryPlateIds(PreviousPlateId, OwnerCandidates, RecoveryCandidates, MotionSamples, RelevantPlateIds);
		const int32 PrimaryPlateId = PreviousPlateId != INDEX_NONE
			? PreviousPlateId
			: ChooseBestSupportedPlateId(RelevantPlateIds, OwnerCandidates, RecoveryCandidates, MotionSamples);
		const int32 CompetitorPlateId = ChooseBestSupportedPlateId(
			RelevantPlateIds,
			OwnerCandidates,
			RecoveryCandidates,
			MotionSamples,
			PrimaryPlateId);
		Resolved.BoundaryOtherPlateId = CompetitorPlateId;

		double RelativeNormalVelocity = 0.0;
		const EV6BoundaryMotionClass MotionClass =
			(PrimaryPlateId != INDEX_NONE && CompetitorPlateId != INDEX_NONE)
				? ClassifyBoundaryMotion(QueryPoint, PrimaryPlateId, CompetitorPlateId, MotionSamples, RelativeNormalVelocity)
				: EV6BoundaryMotionClass::None;
		Resolved.BoundaryRelativeNormalVelocity = RelativeNormalVelocity;

		const bool bPreviousHasStrongEvidence = HasStrongPreviousOwnerEvidence(
			PreviousPlateId,
			CompetitorPlateId,
			OwnerCandidates,
			RecoveryCandidates,
			MotionSamples);
		const bool bPreviousHasExactCandidate =
			PreviousPlateId != INDEX_NONE && FindBestOwnerCandidateForPlate(OwnerCandidates, PreviousPlateId) != nullptr;

		if (OwnerCandidates.Num() > 1)
		{
			if (PreviousPlateId != INDEX_NONE && bPreviousHasExactCandidate)
			{
				if (MotionClass == EV6BoundaryMotionClass::Convergent)
				{
					const int32 AdvancingPlateId = ChooseAdvancingPlateId(
						QueryPoint,
						PreviousPlateId,
						CompetitorPlateId,
						OwnerCandidates,
						RecoveryCandidates,
						MotionSamples);
					if (AdvancingPlateId != INDEX_NONE && AdvancingPlateId != PreviousPlateId)
					{
						FinalizeBoundaryResolvedSample(
							Resolved,
							AdvancingPlateId,
							PreviousPlateId,
							ETectonicPlanetV6ResolutionKind::BoundaryReassigned,
							ETectonicPlanetV6BoundaryOutcome::ReassignedOwner,
							OwnerCandidates,
							RecoveryCandidates,
							NearestMemberFallbackPlateId,
							NearestMemberFallbackSampleIndex,
							NearestMemberFallbackDistance);
						return Resolved;
					}
				}

				FinalizeBoundaryResolvedSample(
					Resolved,
					PreviousPlateId,
					CompetitorPlateId,
					ETectonicPlanetV6ResolutionKind::BoundaryRetained,
					ETectonicPlanetV6BoundaryOutcome::RetainedOwner,
					OwnerCandidates,
					RecoveryCandidates,
					NearestMemberFallbackPlateId,
					NearestMemberFallbackSampleIndex,
					NearestMemberFallbackDistance);
				return Resolved;
			}

			int32 WinningPlateId = INDEX_NONE;
			if (MotionClass == EV6BoundaryMotionClass::Convergent)
			{
				WinningPlateId = ChooseAdvancingPlateId(
					QueryPoint,
					PrimaryPlateId,
					CompetitorPlateId,
					OwnerCandidates,
					RecoveryCandidates,
					MotionSamples);
			}
			if (WinningPlateId == INDEX_NONE)
			{
				WinningPlateId = ChooseBestSupportedPlateId(
					RelevantPlateIds,
					OwnerCandidates,
					RecoveryCandidates,
					MotionSamples);
			}

			FinalizeBoundaryResolvedSample(
				Resolved,
				WinningPlateId,
				WinningPlateId == PreviousPlateId ? CompetitorPlateId : PreviousPlateId,
				WinningPlateId == PreviousPlateId
					? ETectonicPlanetV6ResolutionKind::BoundaryRetained
					: ETectonicPlanetV6ResolutionKind::BoundaryReassigned,
				WinningPlateId == PreviousPlateId
					? ETectonicPlanetV6BoundaryOutcome::RetainedOwner
					: ETectonicPlanetV6BoundaryOutcome::ReassignedOwner,
				OwnerCandidates,
				RecoveryCandidates,
				NearestMemberFallbackPlateId,
				NearestMemberFallbackSampleIndex,
				NearestMemberFallbackDistance);
			return Resolved;
		}

		if (OwnerCandidates.IsEmpty())
		{
				if (PreviousPlateId != INDEX_NONE &&
					bPreviousHasStrongEvidence &&
					(MotionClass == EV6BoundaryMotionClass::None || MotionClass == EV6BoundaryMotionClass::Weak))
				{
					FinalizeBoundaryResolvedSample(
						Resolved,
						PreviousPlateId,
						CompetitorPlateId,
						ETectonicPlanetV6ResolutionKind::BoundaryRetained,
						ETectonicPlanetV6BoundaryOutcome::RetainedOwner,
						OwnerCandidates,
						RecoveryCandidates,
						NearestMemberFallbackPlateId,
						NearestMemberFallbackSampleIndex,
						NearestMemberFallbackDistance);
					return Resolved;
				}

				if (PreviousPlateId != INDEX_NONE &&
					bPreviouslyContinental &&
					MotionClass == EV6BoundaryMotionClass::Divergent)
				{
					FinalizeBoundaryResolvedSample(
						Resolved,
						PreviousPlateId,
						CompetitorPlateId,
						ETectonicPlanetV6ResolutionKind::BoundaryRetained,
						ETectonicPlanetV6BoundaryOutcome::RetainedOwner,
						OwnerCandidates,
						RecoveryCandidates,
						NearestMemberFallbackPlateId,
						NearestMemberFallbackSampleIndex,
						NearestMemberFallbackDistance);
					return Resolved;
				}

				if (MotionClass == EV6BoundaryMotionClass::Divergent && CompetitorPlateId != INDEX_NONE)
				{
					const int32 OceanicOwnerPlateId = ChooseOceanicOwnerPlateId(
						PreviousPlateId,
						CompetitorPlateId,
						RecoveryCandidates,
						MotionSamples);
					FinalizeBoundaryResolvedSample(
						Resolved,
						OceanicOwnerPlateId,
						OceanicOwnerPlateId == PreviousPlateId ? CompetitorPlateId : PreviousPlateId,
						ETectonicPlanetV6ResolutionKind::BoundaryOceanic,
						ETectonicPlanetV6BoundaryOutcome::DivergentOceanic,
						OwnerCandidates,
						RecoveryCandidates,
						NearestMemberFallbackPlateId,
						NearestMemberFallbackSampleIndex,
						NearestMemberFallbackDistance);
					Resolved.SourceTriangleIndex = INDEX_NONE;
					Resolved.SourceCanonicalSampleIndex = INDEX_NONE;
					Resolved.SourceBarycentric = FVector3d(-1.0, -1.0, -1.0);
					return Resolved;
				}

				if (MotionClass == EV6BoundaryMotionClass::Convergent && CompetitorPlateId != INDEX_NONE)
				{
					const int32 AdvancingPlateId = ChooseAdvancingPlateId(
						QueryPoint,
						PrimaryPlateId,
						CompetitorPlateId,
						OwnerCandidates,
						RecoveryCandidates,
						MotionSamples);
					FinalizeBoundaryResolvedSample(
						Resolved,
						AdvancingPlateId,
						AdvancingPlateId == PreviousPlateId ? CompetitorPlateId : PreviousPlateId,
						AdvancingPlateId == PreviousPlateId
							? ETectonicPlanetV6ResolutionKind::BoundaryRetained
							: ETectonicPlanetV6ResolutionKind::BoundaryReassigned,
						AdvancingPlateId == PreviousPlateId
							? ETectonicPlanetV6BoundaryOutcome::RetainedOwner
							: ETectonicPlanetV6BoundaryOutcome::ReassignedOwner,
						OwnerCandidates,
						RecoveryCandidates,
						NearestMemberFallbackPlateId,
						NearestMemberFallbackSampleIndex,
						NearestMemberFallbackDistance);
					return Resolved;
				}

			if (PreviousPlateId != INDEX_NONE && bPreviousHasStrongEvidence)
			{
				FinalizeBoundaryResolvedSample(
					Resolved,
					PreviousPlateId,
					CompetitorPlateId,
					ETectonicPlanetV6ResolutionKind::BoundaryRetained,
					ETectonicPlanetV6BoundaryOutcome::RetainedOwner,
					OwnerCandidates,
					RecoveryCandidates,
					NearestMemberFallbackPlateId,
					NearestMemberFallbackSampleIndex,
					NearestMemberFallbackDistance);
				return Resolved;
			}
		}

		Resolved = FTectonicPlanetV6::ResolvePhase1OwnershipForTest(
			OwnerCandidates,
			RecoveryCandidates,
			NearestMemberFallbackPlateId,
			NearestMemberFallbackSampleIndex,
			ExplicitFallbackPlateId,
			ExplicitFallbackSampleIndex);
		Resolved.PreviousPlateId = PreviousPlateId;
		Resolved.ExactCandidateCount = OwnerCandidates.Num();
		Resolved.BoundaryOutcome =
			Resolved.FinalPlateId == PreviousPlateId
				? ETectonicPlanetV6BoundaryOutcome::RetainedOwner
				: ETectonicPlanetV6BoundaryOutcome::ReassignedOwner;
		Resolved.BoundaryOtherPlateId = CompetitorPlateId;
		Resolved.BoundaryRelativeNormalVelocity = RelativeNormalVelocity;
		Resolved.bBoundaryDecision = OwnerCandidates.Num() != 1;
		return Resolved;
	}
}

void FTectonicPlanetV6::Initialize(
	const int32 InSampleCount,
	const int32 InPlateCount,
	const int32 InRandomSeed,
	const float InBoundaryWarpAmplitude,
	const float InContinentalFraction,
	const double InPlanetRadiusKm)
{
	Planet = FTectonicPlanet{};
	Planet.Initialize(InSampleCount, InPlanetRadiusKm);
	Planet.InitializePlates(InPlateCount, InRandomSeed, InBoundaryWarpAmplitude, InContinentalFraction);

	FTectonicPlanetRuntimeConfig RuntimeConfig = GetM6BaselineRuntimeConfig();
	RuntimeConfig.ResamplingPolicy = EResamplingPolicy::EventDrivenOnly;
	RuntimeConfig.bEnableAutomaticRifting = bEnableAutomaticRiftingForTest;
	ApplyTectonicPlanetRuntimeConfig(Planet, RuntimeConfig);
	Planet.bDeferRiftFollowupResamplingToV6 = bEnableAutomaticRiftingForTest;

	Planet.ResamplingSteps.Reset();
	Planet.ResamplingHistory.Reset();
	Planet.LastResamplingStats = FResamplingStats{};
	Planet.LastResampleTriggerReason = EResampleTriggerReason::None;
	Planet.LastResampleOwnershipMode = EResampleOwnershipMode::FullResolution;
	Planet.MaxStepsWithoutResampling = INDEX_NONE;
	Planet.bPendingFullResolutionResample = false;
	Planet.PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	Planet.PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};
	Planet.PendingRiftEvent = FPendingRiftEvent{};

	PeriodicSolveCount = 0;
	PeriodicSolveSteps.Reset();
	LastResolvedSamples.Reset();
	LastSolveStats = FTectonicPlanetV6PeriodicSolveStats{};
	PeriodicSolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative;
	ThesisRemeshFixedIntervalSteps = 25;
	ThesisCopiedFrontierMeshes.Reset();
	ThesisPlateSubmeshMeshes.Reset();
	CopiedFrontierTrackedDestructiveKinds.Reset();
	CopiedFrontierTrackedPreferredContinuationPlateIds.Reset();
	CopiedFrontierTrackedDestructiveSourcePlateIds.Reset();
	CopiedFrontierTrackedDestructiveDistancesKm.Reset();
	CopiedFrontierTrackedDestructiveSeedOriginFlags.Reset();
	CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags.Reset();
	CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags.Reset();
	CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats = FTectonicPlanetV6DestructiveTrackingLifecycleStats{};
	CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount = 0;
	CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount = 0;
	CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount = 0;
	CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
	CopiedFrontierIntervalPropagationWaveCapForTest = INDEX_NONE;
	MissLineageCounts.Reset();
	PreviousSolveSyntheticFlags.Reset();
	PreviousSolveRetainedSyntheticCoverageFlags.Reset();
	CurrentSolvePreviousSyntheticFlags.Reset();
	CurrentSolvePreviousRetainedSyntheticCoverageFlags.Reset();
	CurrentSolveRetainedSyntheticCoverageFlags.Reset();
	CurrentSolvePreviousOwnerFilteredHitFlags.Reset();
	CurrentSolvePreviousOwnerUnfilteredHitFlags.Reset();
	CurrentSolvePreviousOwnerRecoveryFlags.Reset();
	CurrentSolvePreviousOwnerIgnoringBoundingCapHitFlags.Reset();
	CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags.Reset();
	CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags.Reset();
	CurrentSolvePreviousOwnerAdjacentTriangleNearestDistanceKm.Reset();
	CurrentSolvePreviousOwnerAdjacentTriangleSupportCounts.Reset();
	CurrentSolvePreviousOwnerAdjacentTriangleMixedSupportCounts.Reset();
	CurrentSolvePreviousOwnerAdjacentTriangleMinoritySupportCounts.Reset();
	CurrentSolveLocalParticipationPlateCounts.Reset();
	CurrentSolveCollisionShadowTrackedFlags.Reset();
	CurrentSolveCollisionShadowQualifiedFlags.Reset();
	CurrentSolveCollisionShadowPersistenceMask.Reset();
	CurrentSolveCollisionShadowDiagnostic = FTectonicPlanetV6CollisionShadowDiagnostic{};
	CurrentSolveCollisionExecutionMask.Reset();
	CurrentSolveCollisionTransferMask.Reset();
	CumulativeCollisionExecutionMask.Reset();
	CumulativeCollisionTransferMask.Reset();
	CumulativeCollisionElevationDeltaMaskKm.Reset();
	CumulativeCollisionContinentalGainMask.Reset();
	CurrentSolveCollisionExecutionDiagnostic = FTectonicPlanetV6CollisionExecutionDiagnostic{};
	CurrentSolveRiftDiagnostic = FTectonicPlanetV6RiftDiagnostic{};
	V9CollisionShadowPairRecurrenceByKey.Reset();
	V9CollisionExecutionLastSolveIndexByKey.Reset();
	V9CollisionExecutionCumulativeCount = 0;
	V9CollisionExecutionCumulativeAffectedSampleVisits = 0;
	V9CollisionExecutionCumulativeContinentalGainCount = 0;
	V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
	V9CollisionExecutionCumulativeElevationDeltaKm = 0.0;
	V9CollisionExecutionCumulativeMaxElevationDeltaKm = 0.0;
	V9RiftCumulativeCount = 0;
	bEnableSyntheticCoverageRetentionForTest = false;
	bEnableV9CollisionShadowForTest = false;
	bEnableV9CollisionExecutionForTest = false;
	bEnableV9CollisionExecutionEnhancedConsequencesForTest = false;
	bEnableV9CollisionExecutionStructuralTransferForTest = false;
	bEnableV9CollisionExecutionRefinedStructuralTransferForTest = false;
	bEnableV9ThesisShapedCollisionRidgeSurgeForTest = false;
	PreviousIntervalMeanSubductionDistanceKm = -1.0;
	PreviousIntervalSubductionSampleCount = 0;
	PreviousIntervalTrackedTriangleCount = 0;
	RebuildThesisCopiedFrontierMeshes();
	RebuildThesisPlateSubmeshMeshes();
}

void FTectonicPlanetV6::AdvanceStep()
{
	Planet.AdvanceStep();
	if (bForcePerTimestepContainmentSoupRebuildForTest)
	{
		Planet.BuildContainmentSoups();
	}

	if (HandlePendingAutomaticRiftAfterAdvance())
	{
		return;
	}

	ApplySubmergedContinentalFringeRelaxationAfterStep();

	if (UsesIntervalDestructivePropagationForSolveMode(PeriodicSolveMode) &&
		Planet.CurrentStep > 0)
	{
		const bool bAllowPropagationWave =
			CopiedFrontierIntervalPropagationWaveCapForTest == INDEX_NONE ||
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount <
				CopiedFrontierIntervalPropagationWaveCapForTest;
		if (bAllowPropagationWave)
		{
			int32 ExpiredTriangleCount = 0;
			FV6CopiedFrontierDestructiveTrackingUpdateStats WaveStats;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount +=
				AdvanceCopiedFrontierIntervalDestructiveTracking(
					Planet,
					ThesisCopiedFrontierMeshes,
					CopiedFrontierTrackedDestructiveKinds,
					CopiedFrontierTrackedPreferredContinuationPlateIds,
					CopiedFrontierTrackedDestructiveSourcePlateIds,
					CopiedFrontierTrackedDestructiveDistancesKm,
					CopiedFrontierTrackedDestructiveSeedOriginFlags,
					CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags,
					CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags,
					ExpiredTriangleCount,
					WaveStats);
			CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount += ExpiredTriangleCount;
			AccumulateTrackingLifecycleStats(
				CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats,
				WaveStats.LifecycleStats);
			++CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount;
		}
	}

	const int32 Interval = ComputePeriodicSolveInterval();
	if (Planet.CurrentStep > 0 && Interval > 0 && (Planet.CurrentStep % Interval) == 0)
	{
		PerformAuthoritativePeriodicSolve(ETectonicPlanetV6SolveTrigger::Periodic);
	}
}

void FTectonicPlanetV6::ApplySubmergedContinentalFringeRelaxationAfterStep()
{
	if (!bEnableV9SubmergedContinentalFringeRelaxationForTest)
	{
		return;
	}

	const int32 SampleCount = Planet.Samples.Num();
	if (CurrentSolveSubmergedContinentalFringeFlags.Num() != SampleCount ||
		CurrentSolveSubmergedContinentalFringeBoundaryOrActiveFlags.Num() != SampleCount ||
		CurrentSolveSubmergedContinentalFringeOceanicNeighborFractions.Num() != SampleCount)
	{
		return;
	}

	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		if (CurrentSolveSubmergedContinentalFringeFlags[SampleIndex] == 0)
		{
			continue;
		}

		FSample& Sample = Planet.Samples[SampleIndex];
		if (Sample.ContinentalWeight < 0.5f ||
			Sample.Elevation > 0.0f ||
			Sample.Elevation <= SubmergedContinentalFringeMaxDepthKm ||
			Sample.OrogenyType != EOrogenyType::None)
		{
			continue;
		}

		const double OceanicNeighborFraction = FMath::Clamp(
			static_cast<double>(CurrentSolveSubmergedContinentalFringeOceanicNeighborFractions[SampleIndex]),
			0.0,
			1.0);
		double RelaxationRate =
			V9SubmergedContinentalFringeRelaxationRatePerStepForTest *
			FMath::Lerp(0.5, 1.0, OceanicNeighborFraction);
		if (CurrentSolveSubmergedContinentalFringeBoundaryOrActiveFlags[SampleIndex] != 0)
		{
			RelaxationRate += V9SubmergedContinentalFringeBoundaryOrActiveBonusRatePerStepForTest;
		}

		Sample.ContinentalWeight = FMath::Max(
			0.0f,
			Sample.ContinentalWeight - static_cast<float>(RelaxationRate));
	}
}

void FTectonicPlanetV6::AdvanceSteps(const int32 StepCount)
{
	for (int32 StepIndex = 0; StepIndex < StepCount; ++StepIndex)
	{
		AdvanceStep();
	}
}

bool FTectonicPlanetV6::HandlePendingAutomaticRiftAfterAdvance()
{
	if (!bEnableAutomaticRiftingForTest || !Planet.PendingRiftEvent.bValid)
	{
		return false;
	}

	const FPendingRiftEvent PendingRift = Planet.PendingRiftEvent;
	++V9RiftCumulativeCount;

	CurrentSolveRiftDiagnostic = FTectonicPlanetV6RiftDiagnostic{};
	CurrentSolveRiftDiagnostic.bTriggeredThisSolve = true;
	CurrentSolveRiftDiagnostic.bAutomatic = PendingRift.bAutomatic;
	CurrentSolveRiftDiagnostic.bForcedByTest = PendingRift.bForcedByTest;
	CurrentSolveRiftDiagnostic.bOwnershipAppliedDirectlyByEvent = true;
	CurrentSolveRiftDiagnostic.bCopiedFrontierRebuiltBeforeSolve = true;
	CurrentSolveRiftDiagnostic.bPlateSubmeshRebuiltBeforeSolve = true;
	CurrentSolveRiftDiagnostic.Step = Planet.CurrentStep;
	CurrentSolveRiftDiagnostic.CumulativeRiftCount = V9RiftCumulativeCount;
	CurrentSolveRiftDiagnostic.ParentPlateId = PendingRift.ParentPlateId;
	CurrentSolveRiftDiagnostic.ChildPlateA = PendingRift.ChildPlateA;
	CurrentSolveRiftDiagnostic.ChildPlateB = PendingRift.ChildPlateB;
	CurrentSolveRiftDiagnostic.ParentSampleCount = PendingRift.ParentSampleCount;
	CurrentSolveRiftDiagnostic.ParentContinentalSampleCount = PendingRift.ParentContinentalSampleCount;
	CurrentSolveRiftDiagnostic.ChildSampleCountA = PendingRift.ChildSampleCountA;
	CurrentSolveRiftDiagnostic.ChildSampleCountB = PendingRift.ChildSampleCountB;
	CurrentSolveRiftDiagnostic.PostRiftPlateCount = Planet.Plates.Num();
	CurrentSolveRiftDiagnostic.ParentContinentalFraction = PendingRift.ParentContinentalFraction;
	CurrentSolveRiftDiagnostic.TriggerProbability = PendingRift.TriggerProbability;
	CurrentSolveRiftDiagnostic.TriggerDraw = PendingRift.TriggerDraw;
	CurrentSolveRiftDiagnostic.RiftMilliseconds = PendingRift.RiftMs;
	CurrentSolveRiftDiagnostic.ChildPlateIds = PendingRift.ChildPlateIds;
	CurrentSolveRiftDiagnostic.ChildSampleCounts = PendingRift.ChildSampleCounts;

	RebuildThesisCopiedFrontierMeshes();
	RebuildThesisPlateSubmeshMeshes();

	if (IsCopiedFrontierLikeSolveMode(PeriodicSolveMode))
	{
		if (UsesIntervalDestructivePropagationForSolveMode(PeriodicSolveMode))
		{
			RefreshCopiedFrontierDestructiveTrackingForTest();
		}
		else
		{
			CopiedFrontierTrackedDestructiveKinds.Reset();
			CopiedFrontierTrackedPreferredContinuationPlateIds.Reset();
			CopiedFrontierTrackedDestructiveSourcePlateIds.Reset();
			CopiedFrontierTrackedDestructiveDistancesKm.Reset();
			CopiedFrontierTrackedDestructiveSeedOriginFlags.Reset();
			CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags.Reset();
			CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags.Reset();
			CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats =
				FTectonicPlanetV6DestructiveTrackingLifecycleStats{};
			CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
		}
	}

	PerformAuthoritativePeriodicSolve(ETectonicPlanetV6SolveTrigger::Rift);

	CurrentSolveRiftDiagnostic.bPostRiftSolveRan =
		LastSolveStats.Trigger == ETectonicPlanetV6SolveTrigger::Rift &&
		LastSolveStats.Step == Planet.CurrentStep;
	CurrentSolveRiftDiagnostic.PostRiftPlateCount = Planet.Plates.Num();
	CurrentSolveRiftDiagnostic.ChildBoundaryContactEdgeCount =
		CountPlatePairContactEdges(
			Planet,
			CurrentSolveRiftDiagnostic.ChildPlateA,
			CurrentSolveRiftDiagnostic.ChildPlateB);
	CountActivePairCauseSamples(
		LastResolvedSamples,
		CurrentSolveRiftDiagnostic.ChildPlateA,
		CurrentSolveRiftDiagnostic.ChildPlateB,
		CurrentSolveRiftDiagnostic.ChildBoundaryRiftActiveSampleCount,
		CurrentSolveRiftDiagnostic.ChildBoundaryDivergenceActiveSampleCount);
	CurrentSolveRiftDiagnostic.bChildBoundaryClassifiedDivergent =
		CurrentSolveRiftDiagnostic.ChildBoundaryRiftActiveSampleCount > 0 ||
		CurrentSolveRiftDiagnostic.ChildBoundaryDivergenceActiveSampleCount > 0;
	CurrentSolveRiftDiagnostic.ChildBoundaryDivergentEdgeCount =
		CurrentSolveRiftDiagnostic.bChildBoundaryClassifiedDivergent
			? CurrentSolveRiftDiagnostic.ChildBoundaryContactEdgeCount
			: 0;
	const FTectonicPlanetV6PlatePairBoundaryMotionDiagnostic BoundaryMotion =
		ComputePlatePairBoundaryMotionDiagnostic(
			Planet,
			CurrentSolveRiftDiagnostic.ChildPlateA,
			CurrentSolveRiftDiagnostic.ChildPlateB);
	CurrentSolveRiftDiagnostic.ChildBoundaryConvergentEdgeCount = BoundaryMotion.ConvergentEdgeCount;
	CurrentSolveRiftDiagnostic.ChildBoundaryMeanRelativeNormalVelocityKmPerMy =
		BoundaryMotion.MeanRelativeNormalVelocityKmPerMy;
	CurrentSolveRiftDiagnostic.ChildBoundaryMaxAbsRelativeNormalVelocityKmPerMy =
		BoundaryMotion.MaxAbsRelativeNormalVelocityKmPerMy;
	CurrentSolveRiftDiagnostic.bChildBoundaryClassifiedDivergent =
		CurrentSolveRiftDiagnostic.bChildBoundaryClassifiedDivergent ||
		BoundaryMotion.bPairIsDivergent;
	if (BoundaryMotion.DivergentEdgeCount > 0)
	{
		CurrentSolveRiftDiagnostic.ChildBoundaryDivergentEdgeCount = BoundaryMotion.DivergentEdgeCount;
	}

	const FString ChildPlateIdsString = JoinIntArrayForLog(CurrentSolveRiftDiagnostic.ChildPlateIds);
	const FString ChildSampleCountsString = JoinIntArrayForLog(CurrentSolveRiftDiagnostic.ChildSampleCounts);
	UE_LOG(
		LogTemp,
		Log,
		TEXT("[TectonicPlanetV6 Rift Step=%d] automatic=%d forced=%d parent=%d child_ids=(%s) parent_samples=%d child_samples=(%s) parent_continental_fraction=%.4f trigger_probability=%.6f trigger_draw=%.6f post_rift_plate_count=%d remesh_ran=%d ownership_applied_directly=%d child_boundary_contact_edges=%d child_boundary_divergent_edges=%d child_boundary_convergent_edges=%d child_boundary_mean_rel_normal_velocity=%.4f child_boundary_max_abs_rel_normal_velocity=%.4f child_boundary_rift_active_samples=%d child_boundary_divergence_samples=%d"),
		CurrentSolveRiftDiagnostic.Step,
		CurrentSolveRiftDiagnostic.bAutomatic ? 1 : 0,
		CurrentSolveRiftDiagnostic.bForcedByTest ? 1 : 0,
		CurrentSolveRiftDiagnostic.ParentPlateId,
		*ChildPlateIdsString,
		CurrentSolveRiftDiagnostic.ParentSampleCount,
		*ChildSampleCountsString,
		CurrentSolveRiftDiagnostic.ParentContinentalFraction,
		CurrentSolveRiftDiagnostic.TriggerProbability,
		CurrentSolveRiftDiagnostic.TriggerDraw,
		CurrentSolveRiftDiagnostic.PostRiftPlateCount,
		CurrentSolveRiftDiagnostic.bPostRiftSolveRan ? 1 : 0,
		CurrentSolveRiftDiagnostic.bOwnershipAppliedDirectlyByEvent ? 1 : 0,
		CurrentSolveRiftDiagnostic.ChildBoundaryContactEdgeCount,
		CurrentSolveRiftDiagnostic.ChildBoundaryDivergentEdgeCount,
		CurrentSolveRiftDiagnostic.ChildBoundaryConvergentEdgeCount,
		CurrentSolveRiftDiagnostic.ChildBoundaryMeanRelativeNormalVelocityKmPerMy,
		CurrentSolveRiftDiagnostic.ChildBoundaryMaxAbsRelativeNormalVelocityKmPerMy,
		CurrentSolveRiftDiagnostic.ChildBoundaryRiftActiveSampleCount,
		CurrentSolveRiftDiagnostic.ChildBoundaryDivergenceActiveSampleCount);

	Planet.PendingRiftEvent = FPendingRiftEvent{};
	return true;
}

FTectonicPlanetV6ResolvedSample FTectonicPlanetV6::ResolvePhase1OwnershipForTest(
	const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
	const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
	const int32 NearestMemberFallbackPlateId,
	const int32 NearestMemberFallbackSampleIndex,
	const int32 ExplicitFallbackPlateId,
	const int32 ExplicitFallbackSampleIndex)
{
	FTectonicPlanetV6ResolvedSample Resolved;
	Resolved.ExactCandidateCount = OwnerCandidates.Num();

	// Temporary Phase 1/1b ownership-only rule:
	// 1. One exact owner candidate wins immediately.
	// 2. Multiple exact owner candidates resolve to the best fit score, then lowest plate id.
	// 3. Zero exact candidates recover the nearest owner, first by nearest triangle, then nearest member.
	// 4. Only if recovery fails entirely do we use an explicit logged fallback owner.
	if (OwnerCandidates.Num() == 1)
	{
		const FTectonicPlanetV6OwnerCandidate& Candidate = OwnerCandidates[0];
		Resolved.FinalPlateId = Candidate.PlateId;
		Resolved.SourceTriangleIndex = Candidate.TriangleIndex;
		Resolved.SourceBarycentric = Candidate.Barycentric;
		Resolved.WinningFitScore = Candidate.FitScore;
		Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::SingleCandidate;
		return Resolved;
	}

	if (OwnerCandidates.Num() > 1)
	{
		const FTectonicPlanetV6OwnerCandidate* BestCandidate = &OwnerCandidates[0];
		for (int32 CandidateIndex = 1; CandidateIndex < OwnerCandidates.Num(); ++CandidateIndex)
		{
			const FTectonicPlanetV6OwnerCandidate& Candidate = OwnerCandidates[CandidateIndex];
			const bool bHigherScore = Candidate.FitScore > BestCandidate->FitScore + TriangleEpsilon;
			const bool bTie = FMath::IsNearlyEqual(Candidate.FitScore, BestCandidate->FitScore, TriangleEpsilon);
			if (bHigherScore || (bTie && Candidate.PlateId < BestCandidate->PlateId))
			{
				BestCandidate = &Candidate;
			}
		}

		Resolved.FinalPlateId = BestCandidate->PlateId;
		Resolved.SourceTriangleIndex = BestCandidate->TriangleIndex;
		Resolved.SourceBarycentric = BestCandidate->Barycentric;
		Resolved.WinningFitScore = BestCandidate->FitScore;
		Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::OverlapWinner;
		return Resolved;
	}

	if (!RecoveryCandidates.IsEmpty())
	{
		const FTectonicPlanetV6RecoveryCandidate* BestRecovery = &RecoveryCandidates[0];
		for (int32 RecoveryIndex = 1; RecoveryIndex < RecoveryCandidates.Num(); ++RecoveryIndex)
		{
			const FTectonicPlanetV6RecoveryCandidate& Candidate = RecoveryCandidates[RecoveryIndex];
			const bool bCloser = Candidate.DistanceRadians + TriangleEpsilon < BestRecovery->DistanceRadians;
			const bool bTie = FMath::IsNearlyEqual(Candidate.DistanceRadians, BestRecovery->DistanceRadians, TriangleEpsilon);
			if (bCloser || (bTie && Candidate.PlateId < BestRecovery->PlateId))
			{
				BestRecovery = &Candidate;
			}
		}

		Resolved.FinalPlateId = BestRecovery->PlateId;
		Resolved.SourceTriangleIndex = BestRecovery->TriangleIndex;
		Resolved.SourceBarycentric = BestRecovery->Barycentric;
		Resolved.RecoveryDistanceRadians = BestRecovery->DistanceRadians;
		Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery;
		return Resolved;
	}

	if (NearestMemberFallbackPlateId != INDEX_NONE && NearestMemberFallbackSampleIndex != INDEX_NONE)
	{
		Resolved.FinalPlateId = NearestMemberFallbackPlateId;
		Resolved.SourceCanonicalSampleIndex = NearestMemberFallbackSampleIndex;
		Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::NearestMemberRecovery;
		return Resolved;
	}

	Resolved.FinalPlateId = ExplicitFallbackPlateId;
	Resolved.SourceCanonicalSampleIndex = ExplicitFallbackSampleIndex;
	Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ExplicitFallback;
	return Resolved;
}

FTectonicPlanetV6ResolvedSample FTectonicPlanetV6::ResolveBoundaryStateForTest(
	const int32 PreviousPlateId,
	const TArray<FTectonicPlanetV6OwnerCandidate>& OwnerCandidates,
	const TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates,
	const TArray<FTectonicPlanetV6BoundaryMotionSample>& MotionSamples,
	const bool bPreviouslyContinental,
	const FVector3d& QueryPoint)
{
	return ResolveBoundaryState(
		PreviousPlateId,
		OwnerCandidates,
		RecoveryCandidates,
		MotionSamples,
		QueryPoint.GetSafeNormal(),
		bPreviouslyContinental,
		INDEX_NONE,
		INDEX_NONE,
		TNumericLimits<double>::Max(),
		INDEX_NONE,
		INDEX_NONE);
}

void FTectonicPlanetV6::CollectBoundaryTrianglePlateIdsForTest(
	const int32 VertexPlateIdA,
	const int32 VertexPlateIdB,
	const int32 VertexPlateIdC,
	TArray<int32>& OutPlateIds)
{
	CollectBoundaryTrianglePlateIds(VertexPlateIdA, VertexPlateIdB, VertexPlateIdC, OutPlateIds);
}

void FTectonicPlanetV6::ApplyCoherencePassForTest(
	const TArray<TArray<int32>>& SampleAdjacency,
	TArray<int32>& InOutPlateIds,
	const int32 MaxComponentSize,
	int32& OutReassignedSampleCount,
	int32& OutRemovedComponentCount,
	int32& OutLargestRemovedComponentSize,
	int32& OutFinalMaxComponentsPerPlate)
{
	FTectonicPlanet TestPlanet;
	TestPlanet.Samples.SetNum(InOutPlateIds.Num());
	TestPlanet.SampleAdjacency = SampleAdjacency;
	for (int32 SampleIndex = 0; SampleIndex < InOutPlateIds.Num(); ++SampleIndex)
	{
		TestPlanet.Samples[SampleIndex].PlateId = InOutPlateIds[SampleIndex];
	}

	TArray<int32> UniquePlateIds;
	for (const int32 PlateId : InOutPlateIds)
	{
		if (PlateId != INDEX_NONE)
		{
			UniquePlateIds.AddUnique(PlateId);
		}
	}
	Algo::Sort(UniquePlateIds);
	for (const int32 PlateId : UniquePlateIds)
	{
		FPlate& Plate = TestPlanet.Plates.AddDefaulted_GetRef();
		Plate.Id = PlateId;
	}

	TArray<FTectonicPlanetV6ResolvedSample> ResolvedSamples;
	ResolvedSamples.SetNum(InOutPlateIds.Num());
	for (int32 SampleIndex = 0; SampleIndex < InOutPlateIds.Num(); ++SampleIndex)
	{
		ResolvedSamples[SampleIndex].FinalPlateId = InOutPlateIds[SampleIndex];
		ResolvedSamples[SampleIndex].PreCoherencePlateId = InOutPlateIds[SampleIndex];
	}

	ApplyCoherencePass(
		TestPlanet,
		InOutPlateIds,
		ResolvedSamples,
		MaxComponentSize,
		OutReassignedSampleCount,
		OutRemovedComponentCount,
		OutLargestRemovedComponentSize,
		OutFinalMaxComponentsPerPlate);
}

void FTectonicPlanetV6::ApplyTriangleTransferForTest(
	FSample& InOutSample,
	const FCarriedSample& V0,
	const FCarriedSample& V1,
	const FCarriedSample& V2,
	const FVector3d& RawBarycentric,
	float& OutSubductionDistanceKm,
	float& OutSubductionSpeed,
	FTectonicPlanetV6TransferDebugInfo& OutTransferDebug)
{
	ApplyTransferredAttributesFromTriangle(
		InOutSample,
		V0,
		V1,
		V2,
		RawBarycentric,
		OutSubductionDistanceKm,
		OutSubductionSpeed,
		OutTransferDebug);
}

void FTectonicPlanetV6::RebuildThesisCopiedFrontierMeshes()
{
	const TArray<uint8>* SyntheticCoverageSupportFlags =
		UsesStructuredGapFillForCopiedFrontierSolveConfiguration(
			PeriodicSolveMode,
			bForceWholeTriangleBoundaryDuplicationForTest,
			bForceExcludeMixedTrianglesForTest) &&
		PreviousSolveSyntheticFlags.Num() == Planet.Samples.Num()
			? &PreviousSolveSyntheticFlags
			: nullptr;
	BuildV6CopiedFrontierPlateMeshes(
		Planet,
		ThesisCopiedFrontierMeshes,
		nullptr,
		GetEffectiveThesisFrontierMeshBuildMode(
			PeriodicSolveMode,
			bForceWholeTriangleBoundaryDuplicationForTest,
			bForceExcludeMixedTrianglesForTest),
		SyntheticCoverageSupportFlags);
}

void FTectonicPlanetV6::RebuildThesisPlateSubmeshMeshes()
{
	BuildV6PlateSubmeshMeshes(Planet, ThesisPlateSubmeshMeshes);
}

void FTectonicPlanetV6::PerformThesisPlateSubmeshSpikeSolve(const ETectonicPlanetV6SolveTrigger Trigger)
{
	const double SolveStartTime = FPlatformTime::Seconds();
	const TArray<FTerrane> PreviousTerranes = Planet.Terranes;
	const int32 Interval = ComputePeriodicSolveInterval();
	if (ThesisPlateSubmeshMeshes.Num() != Planet.Plates.Num())
	{
		RebuildThesisPlateSubmeshMeshes();
	}

	int32 PlateLocalVertexCount = 0;
	int32 PlateLocalTriangleCount = 0;
	int32 PlateSubmeshFrontierVertexCount = 0;
	int32 PlateSubmeshFrontierTriangleCount = 0;
	int32 PlateSubmeshFrontierCarriedSampleCount = 0;
	int32 PlateSubmeshRetriangulatedTriangleCount = 0;
	int32 PlateSubmeshComponentCount = 0;
	for (const FTectonicPlanetV6PlateSubmesh& Mesh : ThesisPlateSubmeshMeshes)
	{
		PlateLocalVertexCount += Mesh.BaseVertices.Num();
		PlateLocalTriangleCount += Mesh.LocalTriangles.Num();
		PlateSubmeshFrontierVertexCount += Mesh.FrontierVertexCount;
		PlateSubmeshFrontierTriangleCount += Mesh.FrontierTriangleCount;
		PlateSubmeshFrontierCarriedSampleCount += Mesh.FrontierCarriedSampleCount;
		PlateSubmeshRetriangulatedTriangleCount += Mesh.RetriangulatedTriangleCount;
		PlateSubmeshComponentCount += Mesh.ComponentCount;
	}

	TArray<FV6PlateQueryGeometry> QueryGeometries;
	BuildV6PlateSubmeshQueryGeometries(Planet, ThesisPlateSubmeshMeshes, QueryGeometries);

	LastResolvedSamples.SetNum(Planet.Samples.Num());
	TArray<int32> NewPlateIds;
	NewPlateIds.Init(INDEX_NONE, Planet.Samples.Num());
	TArray<float> SubductionDistances;
	SubductionDistances.Init(-1.0f, Planet.Samples.Num());
	TArray<float> SubductionSpeeds;
	SubductionSpeeds.Init(0.0f, Planet.Samples.Num());

	TAtomic<int32> HitCount(0);
	TAtomic<int32> MissCount(0);
	TAtomic<int32> MultiHitCount(0);
	TAtomic<int32> DirectHitTriangleTransferCount(0);
	TAtomic<int32> TransferFallbackCount(0);
	TAtomic<int32> NearestMemberFallbackTransferCount(0);
	TAtomic<int32> ExplicitFallbackTransferCount(0);
	TAtomic<int32> PlateSubmeshFrontierHitCount(0);
	TAtomic<int32> InteriorHitCount(0);

	ParallelFor(Planet.Samples.Num(), [this, &QueryGeometries, &NewPlateIds, &HitCount, &MissCount, &MultiHitCount, &PlateSubmeshFrontierHitCount, &InteriorHitCount](const int32 SampleIndex)
	{
		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		const int32 PreviousPlateId = Planet.Samples[SampleIndex].PlateId;

		TArray<FV6ThesisRemeshRayHit> HitCandidates;
		HitCandidates.Reserve(QueryGeometries.Num());
		for (const FV6PlateQueryGeometry& QueryGeometry : QueryGeometries)
		{
			FV6ThesisRemeshRayHit Hit;
			if (TryFindThesisRemeshRayHit(QueryGeometry, QueryPoint, Hit))
			{
				HitCandidates.Add(Hit);
			}
		}

		FTectonicPlanetV6ResolvedSample Resolved;
		Resolved.PreviousPlateId = PreviousPlateId;
		Resolved.ExactCandidateCount = HitCandidates.Num();

		if (!HitCandidates.IsEmpty())
		{
			++HitCount;
			if (HitCandidates.Num() > 1)
			{
				++MultiHitCount;
			}

			const FV6ThesisRemeshRayHit* BestHit = &HitCandidates[0];
			for (int32 HitIndex = 1; HitIndex < HitCandidates.Num(); ++HitIndex)
			{
				if (IsBetterThesisRemeshHit(HitCandidates[HitIndex], *BestHit))
				{
					BestHit = &HitCandidates[HitIndex];
				}
			}

			Resolved.FinalPlateId = BestHit->PlateId;
			Resolved.PreCoherencePlateId = BestHit->PlateId;
			Resolved.SourceLocalTriangleIndex = BestHit->LocalTriangleIndex;
			Resolved.SourceTriangleIndex = BestHit->GlobalTriangleIndex;
			Resolved.SourceBarycentric = BestHit->Barycentric;
			Resolved.WinningFitScore = BestHit->FitScore;
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshHit;
			if (BestHit->bCopiedFrontierTriangle)
			{
				++PlateSubmeshFrontierHitCount;
			}
			else
			{
				++InteriorHitCount;
			}
		}
		else
		{
			++MissCount;

			TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
			CollectThesisRemeshMissRecoveryCandidates(QueryGeometries, QueryPoint, RecoveryCandidates);

			int32 ExplicitFallbackPlateId = INDEX_NONE;
			int32 ExplicitFallbackSampleIndex = INDEX_NONE;
			ChooseExplicitFallback(Planet, ExplicitFallbackPlateId, ExplicitFallbackSampleIndex);

			int32 PrimaryPlateId = INDEX_NONE;
			int32 SecondaryPlateId = INDEX_NONE;
			double PrimaryDistance = -1.0;
			ChooseThesisRemeshMissOwnerPlates(
				RecoveryCandidates,
				PreviousPlateId,
				ExplicitFallbackPlateId,
				PrimaryPlateId,
				SecondaryPlateId,
				PrimaryDistance);

			Resolved.FinalPlateId = PrimaryPlateId;
			Resolved.PreCoherencePlateId = PrimaryPlateId;
			Resolved.BoundaryOtherPlateId = SecondaryPlateId;
			Resolved.RecoveryDistanceRadians = PrimaryDistance;
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic;
		}

		LastResolvedSamples[SampleIndex] = Resolved;
		NewPlateIds[SampleIndex] = Resolved.FinalPlateId;
	});

	const int32 MaxComponentsBeforeRepartition = ComputeMaxComponentsPerPlateFromAssignments(Planet, NewPlateIds);

	ParallelFor(Planet.Samples.Num(), [this, &SubductionDistances, &SubductionSpeeds, &DirectHitTriangleTransferCount, &TransferFallbackCount](const int32 SampleIndex)
	{
		FSample& Sample = Planet.Samples[SampleIndex];
		FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		Sample.PlateId = Resolved.FinalPlateId;
		Resolved.TransferDebug = FTectonicPlanetV6TransferDebugInfo{};

		bool bTransferred = false;
		if (Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshHit &&
			Resolved.FinalPlateId != INDEX_NONE &&
			Resolved.SourceLocalTriangleIndex != INDEX_NONE)
		{
			const int32 PlateIndex = Planet.FindPlateArrayIndexById(Resolved.FinalPlateId);
			if (ThesisPlateSubmeshMeshes.IsValidIndex(PlateIndex) && Planet.Plates.IsValidIndex(PlateIndex))
			{
				const FTectonicPlanetV6PlateSubmesh& Mesh = ThesisPlateSubmeshMeshes[PlateIndex];
				if (Mesh.LocalTriangles.IsValidIndex(Resolved.SourceLocalTriangleIndex))
				{
					const UE::Geometry::FIndex3i& LocalTriangle = Mesh.LocalTriangles[Resolved.SourceLocalTriangleIndex];
					if (Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.A) &&
						Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.B) &&
						Mesh.LocalCarriedSamples.IsValidIndex(LocalTriangle.C))
					{
						const FPlate& Plate = Planet.Plates[PlateIndex];
						const FCarriedSample V0 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.A]);
						const FCarriedSample V1 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.B]);
						const FCarriedSample V2 = BuildRotatedLocalCarriedSample(Plate, Mesh.LocalCarriedSamples[LocalTriangle.C]);
						ApplyTransferredAttributesFromTriangle(
							Sample,
							V0,
							V1,
							V2,
							Resolved.SourceBarycentric,
							SubductionDistances[SampleIndex],
							SubductionSpeeds[SampleIndex],
							Resolved.TransferDebug);
						Resolved.SourceCanonicalSampleIndex = Resolved.TransferDebug.DominantSourceCanonicalSampleIndex;
						Resolved.TransferDebug.bUsedPlateSubmeshTriangleTransfer =
							Mesh.LocalTriangleFrontierFlags.IsValidIndex(Resolved.SourceLocalTriangleIndex) &&
							Mesh.LocalTriangleFrontierFlags[Resolved.SourceLocalTriangleIndex] != 0;
						++DirectHitTriangleTransferCount;
						bTransferred = true;
					}
				}
			}
		}

		if (!bTransferred &&
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic)
		{
			ApplyOceanicBoundaryCreation(
				Sample,
				Planet,
				Resolved,
				SubductionDistances[SampleIndex],
				SubductionSpeeds[SampleIndex],
				Resolved.TransferDebug);
			bTransferred = true;
		}

		if (!bTransferred && Resolved.FinalPlateId != INDEX_NONE)
		{
			int32 TransferSourceCanonicalSampleIndex = INDEX_NONE;
			const int32 PlateIndex = Planet.FindPlateArrayIndexById(Resolved.FinalPlateId);
			if (ThesisPlateSubmeshMeshes.IsValidIndex(PlateIndex) &&
				ThesisPlateSubmeshMeshes[PlateIndex].LocalTriangles.IsValidIndex(Resolved.SourceLocalTriangleIndex))
			{
				const FTectonicPlanetV6PlateSubmesh& Mesh = ThesisPlateSubmeshMeshes[PlateIndex];
				TransferSourceCanonicalSampleIndex = PickTransferFallbackCanonicalVertexForLocalTriangle(
					Mesh.LocalToCanonicalVertex,
					Mesh.LocalTriangles[Resolved.SourceLocalTriangleIndex],
					Resolved.SourceBarycentric);
			}
			else if (Resolved.SourceTriangleIndex != INDEX_NONE)
			{
				TransferSourceCanonicalSampleIndex = PickTransferFallbackCanonicalVertexForTriangle(
					Planet,
					Resolved.FinalPlateId,
					Resolved.SourceTriangleIndex,
					Resolved.SourceBarycentric);
			}

			bool bUsedNearestMemberFallback = false;
			bool bUsedExplicitFallback = false;
			const FCarriedSample* Source =
				TransferSourceCanonicalSampleIndex != INDEX_NONE
					? FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex)
					: nullptr;
			if (Source == nullptr)
			{
				double NearestMemberDistanceRadians = TNumericLimits<double>::Max();
				if (FindNearestMemberSample(
						Planet,
						Resolved.FinalPlateId,
						Sample.Position,
						TransferSourceCanonicalSampleIndex,
						NearestMemberDistanceRadians))
				{
					Source =
						FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
					bUsedNearestMemberFallback = Source != nullptr;
				}
			}
			if (Source == nullptr)
			{
				ChooseExplicitFallbackForPlate(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
				if (TransferSourceCanonicalSampleIndex != INDEX_NONE)
				{
					Source =
						FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
					bUsedExplicitFallback = Source != nullptr;
				}
			}
			if (Source != nullptr)
			{
				Resolved.SourceCanonicalSampleIndex = TransferSourceCanonicalSampleIndex;
				ApplyTransferredAttributesFromSingleSource(
					Sample,
					*Source,
					SubductionDistances[SampleIndex],
					SubductionSpeeds[SampleIndex],
					Resolved.TransferDebug);
				Resolved.TransferDebug.bUsedNearestMemberFallback = bUsedNearestMemberFallback;
				Resolved.TransferDebug.bUsedExplicitFallback = bUsedExplicitFallback;
				Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshTransferFallback;
				++TransferFallbackCount;
				bTransferred = true;
			}
		}

		if (!bTransferred)
		{
			Sample.ContinentalWeight = 0.0f;
			Sample.Elevation = -6.0f;
			Sample.Thickness = 7.0f;
			Sample.Age = 0.0f;
			Sample.OrogenyType = EOrogenyType::None;
			Sample.TerraneId = INDEX_NONE;
			Sample.RidgeDirection = FVector3d::ZeroVector;
			Sample.FoldDirection = FVector3d::ZeroVector;
			SubductionDistances[SampleIndex] = -1.0f;
			SubductionSpeeds[SampleIndex] = 0.0f;
			Resolved.TransferDebug = FTectonicPlanetV6TransferDebugInfo{};
			Resolved.TransferDebug.SourceKind = ETectonicPlanetV6TransferSourceKind::Defaulted;
			++TransferFallbackCount;
		}
	});

	FTectonicPlanetV6PeriodicSolveStats TransferStats;
	for (const FTectonicPlanetV6ResolvedSample& Resolved : LastResolvedSamples)
	{
		switch (Resolved.TransferDebug.SourceKind)
		{
		case ETectonicPlanetV6TransferSourceKind::Triangle:
			++TransferStats.TriangleTransferCount;
			IncrementTransferResolutionCounts(TransferStats.TriangleTransferCountsByResolution, Resolved);
			break;
		case ETectonicPlanetV6TransferSourceKind::SingleSource:
			++TransferStats.SingleSourceTransferCount;
			IncrementTransferResolutionCounts(TransferStats.SingleSourceTransferCountsByResolution, Resolved);
			break;
		case ETectonicPlanetV6TransferSourceKind::OceanicCreation:
			++TransferStats.OceanicCreationCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::Defaulted:
			++TransferStats.DefaultTransferCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::None:
		default:
			break;
		}

		if (UsesCategoricalMajorityTransfer(Resolved.TransferDebug))
		{
			++TransferStats.CategoricalMajorityTransferCount;
		}
		else if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle ||
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::SingleSource)
		{
			++TransferStats.CategoricalDominantTransferCount;
		}

		if (UsesDirectionalFallback(Resolved.TransferDebug))
		{
			++TransferStats.DirectionalFallbackCount;
		}

		if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
		{
			++TransferStats.ContinentalWeightThresholdMismatchCount;
			IncrementTransferResolutionCounts(
				TransferStats.ContinentalWeightThresholdMismatchCountsByResolution,
				Resolved);
		}
	}

	Planet.RepartitionMembership(NewPlateIds, &SubductionDistances, &SubductionSpeeds);
	Planet.ComputeSubductionDistanceField();
	Planet.ComputeSlabPullCorrections();
	Planet.DetectTerranes(PreviousTerranes);
	RebuildThesisPlateSubmeshMeshes();

	++PeriodicSolveCount;
	PeriodicSolveSteps.Add(Planet.CurrentStep);

	LastSolveStats = FTectonicPlanetV6PeriodicSolveStats{};
	LastSolveStats.Trigger = Trigger;
	LastSolveStats.SolveMode = ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike;
	LastSolveStats.Step = Planet.CurrentStep;
	LastSolveStats.Interval = Interval;
	LastSolveStats.SolveIndex = PeriodicSolveCount;
	LastSolveStats.PlateCount = Planet.Plates.Num();
	LastSolveStats.HitCount = HitCount.Load();
	LastSolveStats.MissCount = MissCount.Load();
	LastSolveStats.MultiHitCount = MultiHitCount.Load();
	LastSolveStats.SingleCandidateWinnerCount = LastSolveStats.HitCount - LastSolveStats.MultiHitCount;
	LastSolveStats.MultiCandidateWinnerCount = LastSolveStats.MultiHitCount;
	LastSolveStats.ZeroCandidateCount = LastSolveStats.MissCount;
	LastSolveStats.GapCount = LastSolveStats.MissCount;
	LastSolveStats.OverlapCount = LastSolveStats.MultiHitCount;
	LastSolveStats.TriangleTransferCount = TransferStats.TriangleTransferCount;
	LastSolveStats.DirectHitTriangleTransferCount = DirectHitTriangleTransferCount.Load();
	LastSolveStats.SingleSourceTransferCount = TransferStats.SingleSourceTransferCount;
	LastSolveStats.OceanicCreationCount = TransferStats.OceanicCreationCount;
	LastSolveStats.DefaultTransferCount = TransferStats.DefaultTransferCount;
	LastSolveStats.TransferFallbackCount = TransferFallbackCount.Load();
	LastSolveStats.PlateLocalVertexCount = PlateLocalVertexCount;
	LastSolveStats.PlateLocalTriangleCount = PlateLocalTriangleCount;
	LastSolveStats.CopiedFrontierVertexCount = 0;
	LastSolveStats.CopiedFrontierTriangleCount = 0;
	LastSolveStats.CopiedFrontierCarriedSampleCount = 0;
	LastSolveStats.CopiedFrontierHitCount = 0;
	LastSolveStats.PlateSubmeshFrontierVertexCount = PlateSubmeshFrontierVertexCount;
	LastSolveStats.PlateSubmeshFrontierTriangleCount = PlateSubmeshFrontierTriangleCount;
	LastSolveStats.PlateSubmeshFrontierCarriedSampleCount = PlateSubmeshFrontierCarriedSampleCount;
	LastSolveStats.PlateSubmeshFrontierHitCount = PlateSubmeshFrontierHitCount.Load();
	LastSolveStats.PlateSubmeshRetriangulatedTriangleCount = PlateSubmeshRetriangulatedTriangleCount;
	LastSolveStats.PlateSubmeshComponentCount = PlateSubmeshComponentCount;
	LastSolveStats.PlateSubmeshWholeMixedTriangleDuplicationCount = 0;
	LastSolveStats.InteriorHitCount = InteriorHitCount.Load();
	LastSolveStats.DestructiveTriangleRejectedCount = 0;
	LastSolveStats.CategoricalMajorityTransferCount = TransferStats.CategoricalMajorityTransferCount;
	LastSolveStats.CategoricalDominantTransferCount = TransferStats.CategoricalDominantTransferCount;
	LastSolveStats.DirectionalFallbackCount = TransferStats.DirectionalFallbackCount;
	LastSolveStats.ContinentalWeightThresholdMismatchCount = TransferStats.ContinentalWeightThresholdMismatchCount;
	LastSolveStats.bDestructiveTriangleExclusionApplied = false;
	LastSolveStats.TriangleTransferCountsByResolution = TransferStats.TriangleTransferCountsByResolution;
	LastSolveStats.SingleSourceTransferCountsByResolution = TransferStats.SingleSourceTransferCountsByResolution;
	LastSolveStats.ContinentalWeightThresholdMismatchCountsByResolution =
		TransferStats.ContinentalWeightThresholdMismatchCountsByResolution;
	LastSolveStats.BoundarySampleCount = CountBoundarySamples(Planet);
	LastSolveStats.ContinentalAreaFraction = ComputeContinentalAreaFraction(Planet);
	LastSolveStats.MaxComponentsBeforeCoherence = MaxComponentsBeforeRepartition;
	LastSolveStats.MaxComponentsPerPlate = ComputeMaxComponentsPerPlate(Planet);

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[TectonicPlanetV6 ThesisPlateSubmesh Step=%d] solve_ms=%.3f cadence_steps=%d plate_local_vertices=%d plate_local_triangles=%d submesh_frontier_vertices=%d submesh_frontier_triangles=%d submesh_frontier_carried=%d submesh_frontier_hits=%d interior_hits=%d submesh_retriangulated_triangles=%d submesh_components=%d mixed_triangle_duplication=%d hits=%d misses=%d multi_hits=%d direct_hit_triangle_transfer=%d single_source_transfer=%d oceanic_creation=%d default_transfer=%d transfer_fallback=%d destructive_filter_applied=%d destructive_filter_rejected=%d cw_threshold_mismatch=%d continental_area_fraction=%.6f max_components_after=%d"),
		Planet.CurrentStep,
		(FPlatformTime::Seconds() - SolveStartTime) * 1000.0,
		LastSolveStats.Interval,
		LastSolveStats.PlateLocalVertexCount,
		LastSolveStats.PlateLocalTriangleCount,
		LastSolveStats.PlateSubmeshFrontierVertexCount,
		LastSolveStats.PlateSubmeshFrontierTriangleCount,
		LastSolveStats.PlateSubmeshFrontierCarriedSampleCount,
		LastSolveStats.PlateSubmeshFrontierHitCount,
		LastSolveStats.InteriorHitCount,
		LastSolveStats.PlateSubmeshRetriangulatedTriangleCount,
		LastSolveStats.PlateSubmeshComponentCount,
		LastSolveStats.PlateSubmeshWholeMixedTriangleDuplicationCount,
		LastSolveStats.HitCount,
		LastSolveStats.MissCount,
		LastSolveStats.MultiHitCount,
		LastSolveStats.DirectHitTriangleTransferCount,
		LastSolveStats.SingleSourceTransferCount,
		LastSolveStats.OceanicCreationCount,
		LastSolveStats.DefaultTransferCount,
		LastSolveStats.TransferFallbackCount,
		LastSolveStats.bDestructiveTriangleExclusionApplied ? 1 : 0,
		LastSolveStats.DestructiveTriangleRejectedCount,
		LastSolveStats.ContinentalWeightThresholdMismatchCount,
		LastSolveStats.ContinentalAreaFraction,
		LastSolveStats.MaxComponentsPerPlate);
}

void FTectonicPlanetV6::PerformThesisCopiedFrontierSpikeSolve(const ETectonicPlanetV6SolveTrigger Trigger)
{
	const double SolveStartTime = FPlatformTime::Seconds();
	const bool bRecordPhaseTiming = bEnablePhaseTimingForTest;
	FTectonicPlanetV6PhaseTiming PhaseTiming;
	const TArray<FTerrane> PreviousTerranes = Planet.Terranes;
	const int32 Interval = ComputePeriodicSolveInterval();
	const bool bEnableTectonicMaintenance =
		IsCopiedFrontierProcessSolveMode(PeriodicSolveMode) &&
		Trigger == ETectonicPlanetV6SolveTrigger::Periodic &&
		Planet.CurrentStep >= Interval;
	const bool bEnableIntervalDestructivePropagation =
		UsesIntervalDestructivePropagationForSolveMode(PeriodicSolveMode);
	const bool bEnableStructuredGapFill =
		UsesStructuredGapFillForCopiedFrontierSolveConfiguration(
			PeriodicSolveMode,
			bForceWholeTriangleBoundaryDuplicationForTest,
			bForceExcludeMixedTrianglesForTest);
	const bool bEnableV9Phase1Authority =
		bEnableV9Phase1AuthorityForTest &&
		IsCopiedFrontierProcessSolveMode(PeriodicSolveMode);
	const EV6ThesisFrontierMeshBuildMode FrontierMeshBuildMode =
		GetEffectiveThesisFrontierMeshBuildMode(
			PeriodicSolveMode,
			bForceWholeTriangleBoundaryDuplicationForTest,
			bForceExcludeMixedTrianglesForTest);
	const TArray<uint8>* SyntheticCoverageSupportFlags =
		bEnableStructuredGapFill &&
		PreviousSolveSyntheticFlags.Num() == Planet.Samples.Num()
			? &PreviousSolveSyntheticFlags
			: nullptr;
	LastCopiedFrontierSolveAttribution = FTectonicPlanetV6CopiedFrontierSolveAttribution{};
	if (ThesisCopiedFrontierMeshes.Num() != Planet.Plates.Num())
	{
		const double RebuildCopiedFrontierMeshesStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		RebuildThesisCopiedFrontierMeshes();
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMs(
				PhaseTiming.RebuildCopiedFrontierMeshesMs,
				RebuildCopiedFrontierMeshesStartTime);
		}
	}

	const bool bApplyDestructiveFilter =
		Trigger == ETectonicPlanetV6SolveTrigger::Periodic &&
		Planet.CurrentStep > 0;
	FV6CopiedFrontierDestructiveFilterState DestructiveFilterState;
	FV6CopiedFrontierDestructiveTrackingUpdateStats DestructiveTrackingStats;
	double CopiedFrontierUnfilteredMeshPrepareMs = 0.0;
	double CopiedFrontierFilteredMeshBuildMs = 0.0;
	int32 CopiedFrontierRefreshedCanonicalVertexCount = 0;
	int32 CopiedFrontierRefreshedSyntheticVertexCount = 0;
	TArray<FTectonicPlanetV6CopiedFrontierPlateMesh> FilteredCopiedFrontierMeshes;
	TArray<FTectonicPlanetV6CopiedFrontierPlateMesh> UnfilteredComparisonCopiedFrontierMeshes;
	const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>* ActiveCopiedFrontierMeshes =
		&ThesisCopiedFrontierMeshes;
	const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>* UnfilteredComparisonMeshes =
		&ThesisCopiedFrontierMeshes;
	if (bApplyDestructiveFilter)
	{
		if (bEnableIntervalDestructivePropagation &&
			(CopiedFrontierTrackedDestructiveKinds.Num() != Planet.TriangleIndices.Num() ||
				CopiedFrontierTrackedPreferredContinuationPlateIds.Num() != Planet.TriangleIndices.Num() ||
				CopiedFrontierTrackedDestructiveSourcePlateIds.Num() != Planet.TriangleIndices.Num() ||
				CopiedFrontierTrackedDestructiveDistancesKm.Num() != Planet.TriangleIndices.Num() ||
				CopiedFrontierTrackedDestructiveSeedOriginFlags.Num() != Planet.TriangleIndices.Num() ||
				CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags.Num() != Planet.TriangleIndices.Num() ||
				CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags.Num() != Planet.TriangleIndices.Num()))
		{
			SeedCopiedFrontierIntervalDestructiveTracking(
				Planet,
				ThesisCopiedFrontierMeshes,
				CopiedFrontierTrackedDestructiveKinds,
				CopiedFrontierTrackedPreferredContinuationPlateIds,
				CopiedFrontierTrackedDestructiveSourcePlateIds,
				CopiedFrontierTrackedDestructiveDistancesKm,
				CopiedFrontierTrackedDestructiveSeedOriginFlags,
				CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags,
				CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags,
				DestructiveTrackingStats);
			CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount =
				DestructiveTrackingStats.NewlySeededTriangleCount;
			CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats =
				DestructiveTrackingStats.LifecycleStats;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
		}
		if (bEnableIntervalDestructivePropagation)
		{
			AccumulateCopiedFrontierTrackedDestructiveCounts(
				CopiedFrontierTrackedDestructiveKinds,
				DestructiveTrackingStats);
			DestructiveTrackingStats.NewlySeededTriangleCount =
				CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount;
			DestructiveTrackingStats.PropagatedTriangleCount =
				CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount;
			DestructiveTrackingStats.ExpiredTriangleCount =
				CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount;
			DestructiveTrackingStats.LifecycleStats =
				CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats;
		}
		else
		{
			DestructiveTrackingStats = FV6CopiedFrontierDestructiveTrackingUpdateStats{};
		}
		const double CopiedFrontierMeshBuildStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		const double UnfilteredMeshPrepareStartTime =
			bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		if (bUseCopiedFrontierUnfilteredMeshReuseForTest)
		{
			RefreshCopiedFrontierMeshCarriedSamples(
				Planet,
				ThesisCopiedFrontierMeshes,
				CopiedFrontierRefreshedCanonicalVertexCount,
				CopiedFrontierRefreshedSyntheticVertexCount);
			UnfilteredComparisonMeshes = &ThesisCopiedFrontierMeshes;
		}
		else
		{
			BuildV6CopiedFrontierPlateMeshes(
				Planet,
				UnfilteredComparisonCopiedFrontierMeshes,
				nullptr,
				FrontierMeshBuildMode,
				SyntheticCoverageSupportFlags);
			UnfilteredComparisonMeshes = &UnfilteredComparisonCopiedFrontierMeshes;
		}
		if (bRecordPhaseTiming)
		{
			CopiedFrontierUnfilteredMeshPrepareMs =
				(FPlatformTime::Seconds() - UnfilteredMeshPrepareStartTime) * 1000.0;
		}
		BuildCopiedFrontierDestructiveFilterState(
			Planet,
			CopiedFrontierTrackedDestructiveKinds,
			CopiedFrontierTrackedPreferredContinuationPlateIds,
			DestructiveFilterState);
		const double FilteredMeshBuildStartTime =
			bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
			BuildV6CopiedFrontierPlateMeshes(
				Planet,
				FilteredCopiedFrontierMeshes,
				&DestructiveFilterState,
				FrontierMeshBuildMode,
				SyntheticCoverageSupportFlags);
		if (bRecordPhaseTiming)
		{
			CopiedFrontierFilteredMeshBuildMs =
				(FPlatformTime::Seconds() - FilteredMeshBuildStartTime) * 1000.0;
		}
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMs(PhaseTiming.CopiedFrontierMeshBuildMs, CopiedFrontierMeshBuildStartTime);
		}
		ActiveCopiedFrontierMeshes = &FilteredCopiedFrontierMeshes;
	}
	const TArray<FTectonicPlanetV6CopiedFrontierPlateMesh>& SolveCopiedFrontierMeshes =
		*ActiveCopiedFrontierMeshes;

	const double PreSolveCaptureStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
	TArray<int32> PreSolvePlateIds;
	TArray<uint8> PreSolveContinentalFlags;
	TArray<float> PreSolveContinentalWeights;
	TArray<float> PreSolveElevations;
	TArray<float> PreSolveThicknesses;
	CaptureCopiedFrontierPreSolveState(
		PreSolvePlateIds,
		PreSolveContinentalFlags,
		PreSolveContinentalWeights,
		PreSolveElevations,
		PreSolveThicknesses);
	ResetCopiedFrontierSolveTransientState(
		PreSolvePlateIds,
		PreSolveContinentalWeights,
		PreSolveElevations);
	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(PhaseTiming.PreSolveCaptureMs, PreSolveCaptureStartTime);
	}

	TArray<TArray<int32>> SampleToAdjacentTriangles;
	const double SampleAdjacencyBuildStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
	BuildSampleToAdjacentTriangles(Planet, SampleToAdjacentTriangles);
	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(PhaseTiming.SampleAdjacencyBuildMs, SampleAdjacencyBuildStartTime);
	}
	if (bEnableV9Phase1Authority)
	{
		const double ActiveZoneMaskStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		BuildPhase1ActiveZoneMask(
			Planet,
			PreSolvePlateIds,
			V9Phase1ActiveZoneClassifierModeForTest,
			FMath::Max(0, V9Phase1ActiveBoundaryRingCountForTest),
			FMath::Clamp(V9Phase1PersistentActivePairHorizonForTest, 1, 3),
			PersistentV9ActivePairRemainingSolveIntervals,
			PersistentV9ActivePairCauseValues,
			CurrentSolveActiveZoneFlags,
			CurrentSolveActiveZoneSeedFlags,
			CurrentSolveActiveZoneCarryoverFlags,
			CurrentSolveActiveZoneFreshExpandedFlags,
			CurrentSolveActiveZoneCarryoverExpandedFlags,
			CurrentSolveActiveZoneCauseValues,
			CurrentSolveActiveZonePrimaryPlateIds,
			CurrentSolveActiveZoneSecondaryPlateIds,
			CurrentSolveActivePairCount,
			CurrentSolveFreshSeedActivePairCount,
			CurrentSolvePersistentCarryoverActivePairCount,
			CurrentSolveFreshPairCandidateCount,
			CurrentSolveFreshPairAdmittedCount,
			CurrentSolveFreshPairRejectedSupportCount,
			CurrentSolveFreshPairRejectedVelocityCount,
			CurrentSolveFreshPairRejectedDominanceCount,
			CurrentSolveFreshPairAdmittedDivergenceCount,
			CurrentSolveFreshPairAdmittedConvergentSubductionCount,
			CurrentSolveFreshPairAdmittedCollisionContactCount,
			CurrentSolveFreshPairAdmittedRiftCount);
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMs(PhaseTiming.ActiveZoneMaskMs, ActiveZoneMaskStartTime);
		}
	}

	FV6CopiedFrontierQueryStageSetup QueryStageSetup;
	BuildV6CopiedFrontierQueryStageSetup(
		Planet,
		SolveCopiedFrontierMeshes,
		UnfilteredComparisonMeshes,
		&DestructiveFilterState,
		bApplyDestructiveFilter,
		bEnableStructuredGapFill,
		bRecordPhaseTiming,
		PhaseTiming,
		QueryStageSetup);
	const FV6CopiedFrontierGeometryCounts& GeometryCounts = QueryStageSetup.GeometryCounts;
	const TArray<FV6PlateQueryGeometry>& QueryGeometries = QueryStageSetup.QueryGeometries;
	const TArray<FV6PlateFrontierPointSet>& FrontierPointSets = QueryStageSetup.FrontierPointSets;
	const TArray<FV6PlateQueryGeometry>& UnfilteredComparisonQueryGeometries =
		QueryStageSetup.UnfilteredComparisonQueryGeometries;
	const TMap<int32, int32>& QueryGeometryIndexByPlateId = QueryStageSetup.QueryGeometryIndexByPlateId;
	const TMap<int32, int32>& UnfilteredComparisonQueryGeometryIndexByPlateId =
		QueryStageSetup.UnfilteredComparisonQueryGeometryIndexByPlateId;

	LastResolvedSamples.SetNum(Planet.Samples.Num());
	TArray<int32> NewPlateIds;
	NewPlateIds.Init(INDEX_NONE, Planet.Samples.Num());
	TArray<float> SubductionDistances;
	SubductionDistances.Init(-1.0f, Planet.Samples.Num());
	TArray<float> SubductionSpeeds;
	SubductionSpeeds.Init(0.0f, Planet.Samples.Num());

	TAtomic<int32> HitCount(0);
	TAtomic<int32> MissCount(0);
	TAtomic<int32> MultiHitCount(0);
	TAtomic<int32> DirectHitTriangleTransferCount(0);
	TAtomic<int32> TransferFallbackCount(0);
	TAtomic<int32> NearestMemberFallbackTransferCount(0);
	TAtomic<int32> ExplicitFallbackTransferCount(0);
	TAtomic<int32> CopiedFrontierHitCount(0);
	TAtomic<int32> InteriorHitCount(0);
	TAtomic<int32> DestructiveTriangleRejectedCount(0);
	TAtomic<int32> OverlapCoherenceSupportPrunedSampleCount(0);
	TAtomic<int32> OverlapCoherencePreviousPlateStabilizedSampleCount(0);
	TAtomic<int32> OverlapCoherenceSuppressedCandidateCount(0);
	TArray<int32> UnfilteredExactCandidateCounts;
	TArray<int32> UnfilteredDestructiveCandidateCounts;
	TArray<uint8> MultiHitTrackedCandidateFlags;
	TArray<uint8> MultiHitPlateMixValues;
	TArray<uint8> MultiHitGeometryMixValues;
	if (bApplyDestructiveFilter)
	{
		UnfilteredExactCandidateCounts.Init(0, Planet.Samples.Num());
		UnfilteredDestructiveCandidateCounts.Init(0, Planet.Samples.Num());
	}
	MultiHitTrackedCandidateFlags.Init(0, Planet.Samples.Num());
	MultiHitPlateMixValues.Init(static_cast<uint8>(EV6CopiedFrontierMultiHitPlateMix::None), Planet.Samples.Num());
	MultiHitGeometryMixValues.Init(
		static_cast<uint8>(EV6CopiedFrontierMultiHitGeometryMix::None),
		Planet.Samples.Num());
	TArray<uint8> ConvergentActiveFieldContinuityFlags;
	TArray<uint8> AdjacentToConvergentActiveFieldContinuityFlags;
	BuildCopiedFrontierConvergentMaintenanceLocality(
		Planet,
		CurrentSolveActiveZoneFlags,
		CurrentSolveActiveZoneCauseValues,
		ConvergentActiveFieldContinuityFlags,
		AdjacentToConvergentActiveFieldContinuityFlags);
	TAtomic<int32> ActiveBandPreviousOwnerCompatibleRecoveryCount(0);
	TAtomic<int32> ActiveBandSyntheticLoopBreakCount(0);
	TAtomic<int32> ActiveBandSyntheticSingleSourceRecoveryCount(0);
	TAtomic<int64> HitSearchMicroseconds(0);
	TAtomic<int64> ZeroHitRecoveryMicroseconds(0);
	TAtomic<int64> DirectHitTransferMicroseconds(0);
	TAtomic<int64> FallbackTransferMicroseconds(0);
	TAtomic<int64> QuietInteriorPreservationMicroseconds(0);
	TArray<uint8> HitSearchPlateCandidateCounts;
	HitSearchPlateCandidateCounts.Init(0, Planet.Samples.Num());
	TArray<uint8> RecoveryCandidatePlateCandidateCounts;
	RecoveryCandidatePlateCandidateCounts.Init(0, Planet.Samples.Num());
	TArray<uint8> RecoveryMissPlateCandidateCounts;
	RecoveryMissPlateCandidateCounts.Init(0, Planet.Samples.Num());
	TArray<uint8> RecoveryMissSampleFlags;
	RecoveryMissSampleFlags.Init(0, Planet.Samples.Num());
	TArray<uint8> RecoveryCandidateGatherSampleFlags;
	RecoveryCandidateGatherSampleFlags.Init(0, Planet.Samples.Num());
	TArray<uint8> HitSearchPrunedSampleFlags;
	HitSearchPrunedSampleFlags.Init(0, Planet.Samples.Num());
	TArray<uint8> RecoveryCandidatePrunedSampleFlags;
	RecoveryCandidatePrunedSampleFlags.Init(0, Planet.Samples.Num());
	const FV6CopiedFrontierRecoveryPreparationStageContext RecoveryPreparationStageContext{
		&Planet,
		&SolveCopiedFrontierMeshes,
		&QueryGeometries,
		&QueryGeometryIndexByPlateId,
		&UnfilteredComparisonQueryGeometries,
		&UnfilteredComparisonQueryGeometryIndexByPlateId,
		&SampleToAdjacentTriangles,
		&CurrentSolvePreviousOwnerFilteredHitFlags,
		&CurrentSolvePreviousOwnerUnfilteredHitFlags,
		&CurrentSolvePreviousOwnerIgnoringBoundingCapHitFlags,
		&CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags,
		&CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags,
		&CurrentSolvePreviousOwnerAdjacentTriangleNearestDistanceKm,
		&CurrentSolvePreviousOwnerAdjacentTriangleSupportCounts,
		&CurrentSolvePreviousOwnerAdjacentTriangleMixedSupportCounts,
		&CurrentSolvePreviousOwnerAdjacentTriangleMinoritySupportCounts,
		&CurrentSolveLocalParticipationPlateCounts,
		&RecoveryCandidatePlateCandidateCounts,
		&RecoveryCandidateGatherSampleFlags,
		&RecoveryCandidatePrunedSampleFlags,
		&CurrentSolvePreviousOwnerRecoveryFlags,
		&ZeroHitRecoveryMicroseconds,
		bApplyDestructiveFilter,
		bRecordPhaseTiming};
	const FV6CopiedFrontierZeroHitResolvedDecisionStageContext ZeroHitResolvedDecisionStageContext{
		&Planet,
		&FrontierPointSets,
		&DestructiveFilterState,
		&PreSolvePlateIds,
		&MissLineageCounts,
		&CurrentSolvePreviousSyntheticFlags,
		&CurrentSolvePreviousRetainedSyntheticCoverageFlags,
		&CurrentSolveRetainedSyntheticCoverageFlags,
		&CurrentSolvePreviousOwnerRecoveryFlags,
		&CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags,
		&CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags,
		&CurrentSolvePreviousTransferSourceKindValues,
		&CurrentSolvePreviousResolutionKindValues,
		&ConvergentActiveFieldContinuityFlags,
		&AdjacentToConvergentActiveFieldContinuityFlags,
		&CurrentSolveLocalParticipationPlateCounts,
		&RecoveryMissPlateCandidateCounts,
		&RecoveryCandidatePlateCandidateCounts,
		&UnfilteredExactCandidateCounts,
		&UnfilteredDestructiveCandidateCounts,
		&ActiveBandPreviousOwnerCompatibleRecoveryCount,
		&ActiveBandSyntheticLoopBreakCount,
		&MissCount,
		bEnableSyntheticCoverageRetentionForTest,
		bEnableStructuredGapFill,
		bApplyDestructiveFilter};

		const double ResolveTransferLoopStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		ParallelFor(Planet.Samples.Num(), [this, &SolveCopiedFrontierMeshes, &QueryGeometries, &UnfilteredComparisonQueryGeometries, &QueryGeometryIndexByPlateId, &UnfilteredComparisonQueryGeometryIndexByPlateId, &DestructiveFilterState, &PreSolvePlateIds, &NewPlateIds, &UnfilteredExactCandidateCounts, &UnfilteredDestructiveCandidateCounts, &MultiHitTrackedCandidateFlags, &MultiHitPlateMixValues, &MultiHitGeometryMixValues, &ConvergentActiveFieldContinuityFlags, &AdjacentToConvergentActiveFieldContinuityFlags, &HitSearchPlateCandidateCounts, &RecoveryCandidatePlateCandidateCounts, &RecoveryMissPlateCandidateCounts, &RecoveryMissSampleFlags, &HitSearchPrunedSampleFlags, &HitCount, &MissCount, &MultiHitCount, &CopiedFrontierHitCount, &InteriorHitCount, &DestructiveTriangleRejectedCount, &OverlapCoherenceSupportPrunedSampleCount, &OverlapCoherencePreviousPlateStabilizedSampleCount, &OverlapCoherenceSuppressedCandidateCount, &HitSearchMicroseconds, &ZeroHitRecoveryMicroseconds, &DirectHitTransferMicroseconds, &FallbackTransferMicroseconds, &QuietInteriorPreservationMicroseconds, &RecoveryPreparationStageContext, &ZeroHitResolvedDecisionStageContext, bApplyDestructiveFilter, bEnableV9Phase1Authority, bRecordPhaseTiming](const int32 SampleIndex)
		{
			const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
			const int32 PreviousPlateId = Planet.Samples[SampleIndex].PlateId;
			const bool bActiveZoneSample =
				!bEnableV9Phase1Authority ||
				(CurrentSolveActiveZoneFlags.IsValidIndex(SampleIndex) &&
					CurrentSolveActiveZoneFlags[SampleIndex] != 0);
			const int32 ActiveZonePrimaryPlateId =
				CurrentSolveActiveZonePrimaryPlateIds.IsValidIndex(SampleIndex)
					? CurrentSolveActiveZonePrimaryPlateIds[SampleIndex]
					: INDEX_NONE;
			const int32 ActiveZoneSecondaryPlateId =
				CurrentSolveActiveZoneSecondaryPlateIds.IsValidIndex(SampleIndex)
					? CurrentSolveActiveZoneSecondaryPlateIds[SampleIndex]
					: INDEX_NONE;
			const bool bUseActivePairFiltering =
				bEnableV9Phase1Authority &&
				(V9Phase1ActiveZoneClassifierModeForTest == ETectonicPlanetV6ActiveZoneClassifierMode::NarrowTectonicPairs ||
					V9Phase1ActiveZoneClassifierModeForTest == ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocal) &&
				bActiveZoneSample &&
				ActiveZonePrimaryPlateId != INDEX_NONE &&
				ActiveZoneSecondaryPlateId != INDEX_NONE;
			const bool bUseOutsideActiveZoneAuthority =
				bEnableV9Phase1Authority &&
				!bActiveZoneSample &&
				PreviousPlateId != INDEX_NONE;
			TArray<int32, TInlineAllocator<4>> PrunedQueryGeometryIndices;
			TArray<int32, TInlineAllocator<4>> PrunedUnfilteredQueryGeometryIndices;
			const bool bUseSemanticPlateCandidatePruning =
				bUsePlateCandidatePruningForTest &&
				(bUseOutsideActiveZoneAuthority || bUseActivePairFiltering);
			if (bUseSemanticPlateCandidatePruning)
			{
				if (bUseOutsideActiveZoneAuthority)
				{
					AppendQueryGeometryIndexForPlateId(QueryGeometryIndexByPlateId, PreviousPlateId, PrunedQueryGeometryIndices);
					if (bApplyDestructiveFilter)
					{
						AppendQueryGeometryIndexForPlateId(
							UnfilteredComparisonQueryGeometryIndexByPlateId,
							PreviousPlateId,
							PrunedUnfilteredQueryGeometryIndices);
					}
				}
				else
				{
					AppendQueryGeometryIndexForPlateId(
						QueryGeometryIndexByPlateId,
						ActiveZonePrimaryPlateId,
						PrunedQueryGeometryIndices);
					AppendQueryGeometryIndexForPlateId(
						QueryGeometryIndexByPlateId,
						ActiveZoneSecondaryPlateId,
						PrunedQueryGeometryIndices);
					if (bApplyDestructiveFilter)
					{
						AppendQueryGeometryIndexForPlateId(
							UnfilteredComparisonQueryGeometryIndexByPlateId,
							ActiveZonePrimaryPlateId,
							PrunedUnfilteredQueryGeometryIndices);
						AppendQueryGeometryIndexForPlateId(
							UnfilteredComparisonQueryGeometryIndexByPlateId,
							ActiveZoneSecondaryPlateId,
							PrunedUnfilteredQueryGeometryIndices);
					}
				}
			}
			const TArray<int32, TInlineAllocator<4>>* HitSearchCandidateQueryGeometryIndices =
				PrunedQueryGeometryIndices.IsEmpty() ? nullptr : &PrunedQueryGeometryIndices;
			const TArray<int32, TInlineAllocator<4>>* UnfilteredHitSearchCandidateQueryGeometryIndices =
				PrunedUnfilteredQueryGeometryIndices.IsEmpty() ? nullptr : &PrunedUnfilteredQueryGeometryIndices;

		TArray<FV6ThesisRemeshRayHit> HitCandidates;
		TArray<FV6ThesisRemeshRayHit> UnfilteredHitCandidates;
		int32 LocalRejectedDestructiveHitCount = 0;
		int32 HitSearchTestedPlateCount = 0;
		const double HitSearchStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		CollectThesisRemeshRayHitCandidates(
			QueryGeometries,
			HitSearchCandidateQueryGeometryIndices,
			QueryPoint,
			HitCandidates,
			&LocalRejectedDestructiveHitCount,
			&HitSearchTestedPlateCount);
		if (bApplyDestructiveFilter)
		{
			CollectThesisRemeshRayHitCandidates(
				UnfilteredComparisonQueryGeometries,
				UnfilteredHitSearchCandidateQueryGeometryIndices,
				QueryPoint,
				UnfilteredHitCandidates);
			UnfilteredExactCandidateCounts[SampleIndex] = UnfilteredHitCandidates.Num();
			int32 LocalDestructiveCandidateCount = 0;
			for (const FV6ThesisRemeshRayHit& Hit : UnfilteredHitCandidates)
			{
				LocalDestructiveCandidateCount +=
					IsCopiedFrontierTriangleDestructive(&DestructiveFilterState, Hit.GlobalTriangleIndex) ? 1 : 0;
			}
			UnfilteredDestructiveCandidateCounts[SampleIndex] = LocalDestructiveCandidateCount;
		}
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMicroseconds(HitSearchMicroseconds, HitSearchStartTime);
		}
		HitSearchPlateCandidateCounts[SampleIndex] =
			static_cast<uint8>(FMath::Clamp(HitSearchTestedPlateCount, 0, 255));
		HitSearchPrunedSampleFlags[SampleIndex] =
			(HitSearchCandidateQueryGeometryIndices != nullptr &&
				HitSearchCandidateQueryGeometryIndices->Num() < QueryGeometries.Num())
				? 1
				: 0;
		if (LocalRejectedDestructiveHitCount > 0)
		{
			DestructiveTriangleRejectedCount += LocalRejectedDestructiveHitCount;
		}

		FTectonicPlanetV6ResolvedSample Resolved;
		Resolved.PreviousPlateId = PreviousPlateId;
		Resolved.bActiveZoneSample = bActiveZoneSample;
		Resolved.ActiveZoneCause =
			CurrentSolveActiveZoneCauseValues.IsValidIndex(SampleIndex)
				? static_cast<ETectonicPlanetV6ActiveZoneCause>(CurrentSolveActiveZoneCauseValues[SampleIndex])
				: ETectonicPlanetV6ActiveZoneCause::None;
		Resolved.ActiveZonePrimaryPlateId = ActiveZonePrimaryPlateId;
		Resolved.ActiveZoneSecondaryPlateId = ActiveZoneSecondaryPlateId;
		const bool bNeedsRecoveryCandidates =
			(bEnableV9Phase1Authority || HitCandidates.IsEmpty()) &&
			(!bUseOutsideActiveZoneAuthority || !bUsePlateCandidatePruningForTest);
		FV6CopiedFrontierRecoveryPreparationStageOutput RecoveryPreparationStageOutput;
		BuildCopiedFrontierRecoveryPreparationStage(
			RecoveryPreparationStageContext,
			SampleIndex,
			PreviousPlateId,
			QueryPoint,
			HitSearchCandidateQueryGeometryIndices,
			bNeedsRecoveryCandidates,
			bUseActivePairFiltering,
			Resolved.ActiveZonePrimaryPlateId,
			Resolved.ActiveZoneSecondaryPlateId,
			HitCandidates,
			RecoveryPreparationStageOutput);
		const FTectonicPlanetV6RecoveryCandidate* PreviousOwnerRecoveryCandidate =
			RecoveryPreparationStageOutput.bHasPreviousOwnerRecoveryCandidate
				? &RecoveryPreparationStageOutput.PreviousOwnerRecoveryCandidate
				: nullptr;
		TArray<FTectonicPlanetV6RecoveryCandidate>& RecoveryCandidates =
			RecoveryPreparationStageOutput.RecoveryCandidates;

		if (bUseOutsideActiveZoneAuthority)
		{
			const FV6ThesisRemeshRayHit* PreviousOwnerExactHit =
				RecoveryPreparationStageOutput.bPreviousOwnerFilteredHitPresent
					? &RecoveryPreparationStageOutput.PreviousOwnerFilteredHit
					: nullptr;
			const bool bOutsideZoneQueryMiss = HitCandidates.IsEmpty();
			const bool bOutsideZoneCoverageDeficit = PreviousOwnerExactHit == nullptr;
			CurrentSolveOutsideActiveZoneQueryMissFlags[SampleIndex] =
				bOutsideZoneQueryMiss ? 1 : 0;
			CurrentSolveOutsideActiveZoneCoverageDeficitFlags[SampleIndex] =
				bOutsideZoneCoverageDeficit ? 1 : 0;

			Resolved.FinalPlateId = PreviousPlateId;
			Resolved.PreCoherencePlateId = PreviousPlateId;
			Resolved.ResolutionKind =
				ETectonicPlanetV6ResolutionKind::ThesisRemeshRetainedOutsideActiveZone;
			Resolved.bOutsideActiveZoneQueryMiss = bOutsideZoneQueryMiss;
			Resolved.bOutsideActiveZoneCoverageDeficit = bOutsideZoneCoverageDeficit;
			Resolved.bAuthorityRetainedOutsideActiveZone = true;
			Resolved.SourceCanonicalSampleIndex = SampleIndex;
			if (PreviousOwnerExactHit != nullptr)
			{
				Resolved.ExactCandidateCount = 1;
				Resolved.SourceLocalTriangleIndex = PreviousOwnerExactHit->LocalTriangleIndex;
				Resolved.SourceTriangleIndex = PreviousOwnerExactHit->GlobalTriangleIndex;
				Resolved.SourceBarycentric = PreviousOwnerExactHit->Barycentric;
				Resolved.WinningFitScore = PreviousOwnerExactHit->FitScore;
				++HitCount;
				if (PreviousOwnerExactHit->bCopiedFrontierTriangle)
				{
					++CopiedFrontierHitCount;
				}
				else
				{
					++InteriorHitCount;
				}
			}
			else if (PreviousOwnerRecoveryCandidate != nullptr)
			{
				Resolved.ExactCandidateCount = 0;
				Resolved.SourceLocalTriangleIndex = PreviousOwnerRecoveryCandidate->LocalTriangleIndex;
				Resolved.SourceTriangleIndex = PreviousOwnerRecoveryCandidate->TriangleIndex;
				Resolved.SourceBarycentric = PreviousOwnerRecoveryCandidate->Barycentric;
				Resolved.RecoveryDistanceRadians = PreviousOwnerRecoveryCandidate->DistanceRadians;
			}
			else
			{
				Resolved.ExactCandidateCount = 0;
			}

			LastResolvedSamples[SampleIndex] = Resolved;
			NewPlateIds[SampleIndex] = Resolved.FinalPlateId;
			return;
		}

		if (HitCandidates.Num() > 1)
		{
			int32 LocalSuppressedCandidateCount = 0;
			bool bUsedSupportPruning = false;
			bool bUsedPreviousPlateStabilization = false;
			ApplyCopiedFrontierOverlapCoherenceFiltering(
				Planet,
				QueryGeometries,
				PreSolvePlateIds,
				PreviousPlateId,
				HitCandidates,
				LocalSuppressedCandidateCount,
				bUsedSupportPruning,
				bUsedPreviousPlateStabilization);
			if (bUsedSupportPruning)
			{
				++OverlapCoherenceSupportPrunedSampleCount;
			}
			if (bUsedPreviousPlateStabilization)
			{
				++OverlapCoherencePreviousPlateStabilizedSampleCount;
			}
			if (LocalSuppressedCandidateCount > 0)
			{
				OverlapCoherenceSuppressedCandidateCount += LocalSuppressedCandidateCount;
			}

			bool bHasTrackedCandidate = false;
			EV6CopiedFrontierMultiHitPlateMix PlateMix = EV6CopiedFrontierMultiHitPlateMix::None;
			EV6CopiedFrontierMultiHitGeometryMix GeometryMix = EV6CopiedFrontierMultiHitGeometryMix::None;
			ClassifyCopiedFrontierMultiHitCandidateMix(
				HitCandidates,
				PreviousPlateId,
				CopiedFrontierTrackedDestructiveKinds,
				bHasTrackedCandidate,
				PlateMix,
				GeometryMix);
			MultiHitTrackedCandidateFlags[SampleIndex] = bHasTrackedCandidate ? 1 : 0;
			MultiHitPlateMixValues[SampleIndex] = static_cast<uint8>(PlateMix);
			MultiHitGeometryMixValues[SampleIndex] = static_cast<uint8>(GeometryMix);
		}
		Resolved.ExactCandidateCount = HitCandidates.Num();

		if (!HitCandidates.IsEmpty())
		{
			++HitCount;
			if (HitCandidates.Num() > 1)
			{
				++MultiHitCount;
			}

			const FV6ThesisRemeshRayHit* BestHit = &HitCandidates[0];
			for (int32 HitIndex = 1; HitIndex < HitCandidates.Num(); ++HitIndex)
			{
				if (IsBetterThesisRemeshHit(HitCandidates[HitIndex], *BestHit))
				{
					BestHit = &HitCandidates[HitIndex];
				}
			}

			Resolved.FinalPlateId = BestHit->PlateId;
			Resolved.PreCoherencePlateId = BestHit->PlateId;
			Resolved.SourceLocalTriangleIndex = BestHit->LocalTriangleIndex;
			Resolved.SourceTriangleIndex = BestHit->GlobalTriangleIndex;
			Resolved.SourceBarycentric = BestHit->Barycentric;
			Resolved.WinningFitScore = BestHit->FitScore;
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshHit;
			if (BestHit->bCopiedFrontierTriangle)
			{
				++CopiedFrontierHitCount;
			}
			else
			{
				++InteriorHitCount;
			}
		}
		else
		{
			RecoveryMissSampleFlags[SampleIndex] = 1;
			const double ZeroHitRecoveryStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
			if (!bNeedsRecoveryCandidates)
			{
				PopulateCopiedFrontierRecoveryPreparationCandidates(
					RecoveryPreparationStageContext,
					SampleIndex,
					PreviousPlateId,
					QueryPoint,
					HitSearchCandidateQueryGeometryIndices,
					true,
					bUseActivePairFiltering,
					Resolved.ActiveZonePrimaryPlateId,
					Resolved.ActiveZoneSecondaryPlateId,
					false,
					RecoveryPreparationStageOutput);
				PreviousOwnerRecoveryCandidate =
					RecoveryPreparationStageOutput.bHasPreviousOwnerRecoveryCandidate
						? &RecoveryPreparationStageOutput.PreviousOwnerRecoveryCandidate
						: nullptr;
			}
			ApplyCopiedFrontierZeroHitResolvedDecisionStage(
				ZeroHitResolvedDecisionStageContext,
				SampleIndex,
				QueryPoint,
				PreviousPlateId,
				UnfilteredHitCandidates,
				PreviousOwnerRecoveryCandidate,
				RecoveryCandidates,
				Resolved);
			if (bRecordPhaseTiming)
			{
				AccumulatePhaseTimingMicroseconds(ZeroHitRecoveryMicroseconds, ZeroHitRecoveryStartTime);
			}
		}

		LastResolvedSamples[SampleIndex] = Resolved;
		NewPlateIds[SampleIndex] = Resolved.FinalPlateId;
	});

	const int32 MaxComponentsBeforeRepartition = ComputeMaxComponentsPerPlateFromAssignments(Planet, NewPlateIds);
	TAtomic<int32> ActiveBandTriangleFieldContinuityClampCount(0);
	TAtomic<int32> ActiveBandSyntheticFieldPreserveCount(0);
	TAtomic<int32> ActiveBandOceanicFieldPreserveCount(0);
	TAtomic<int32> QuietInteriorContinentalRetentionCount(0);
	TAtomic<int32> QuietInteriorContinentalRetentionTriangleCount(0);
	TAtomic<int32> QuietInteriorContinentalRetentionSingleSourceCount(0);
	TAtomic<int32> ContinentalBreadthPreservationCount(0);
	TAtomic<int32> ContinentalBreadthPreservationStrongInteriorCount(0);
	TAtomic<int32> ContinentalBreadthPreservationModerateInteriorCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideSubaerialCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideSubmergedCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideTriangleCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideSingleSourceCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideStrongInteriorCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideModerateInteriorCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideLowElevationBandCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideModerateElevationBandCount(0);
	TAtomic<int32> PaperSurrogateOwnershipOverrideHighElevationBandCount(0);
	TAtomic<int32> PaperSurrogateQuietInteriorDirectHitEligibleCount(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightScaledSum(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightScaledSum(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightScaledSum(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitPreSolveElevationScaledSum(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitPostTransferElevationScaledSum(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitFinalElevationScaledSum(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitPreSolveThicknessScaledSum(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitPostTransferThicknessScaledSum(0);
	TAtomic<int64> PaperSurrogateQuietInteriorDirectHitFinalThicknessScaledSum(0);
	const FV6ContinentalBreadthPreservationSupport ContinentalBreadthSupport =
		(bEnableV9ContinentalBreadthPreservationForTest || bEnableV9PaperSurrogateOwnershipForTest)
			? BuildContinentalBreadthPreservationSupport(
				Planet,
				PreSolveContinentalWeights,
				PreSolveElevations)
			: FV6ContinentalBreadthPreservationSupport{};
	const FV6CopiedFrontierTransferStageContext TransferStageContext{
		&Planet,
		&SolveCopiedFrontierMeshes,
		&ConvergentActiveFieldContinuityFlags,
		&AdjacentToConvergentActiveFieldContinuityFlags,
		&DirectHitTriangleTransferCount,
		&TransferFallbackCount,
		&NearestMemberFallbackTransferCount,
		&ExplicitFallbackTransferCount,
		&ActiveBandSyntheticSingleSourceRecoveryCount,
		&DirectHitTransferMicroseconds,
		&FallbackTransferMicroseconds,
		bRecordPhaseTiming};

	ParallelFor(Planet.Samples.Num(), [this, &PreSolveContinentalWeights, &PreSolveElevations, &PreSolveThicknesses, &SubductionDistances, &SubductionSpeeds, &TransferStageContext, &ActiveBandTriangleFieldContinuityClampCount, &ActiveBandSyntheticFieldPreserveCount, &ActiveBandOceanicFieldPreserveCount, &QuietInteriorContinentalRetentionCount, &QuietInteriorContinentalRetentionTriangleCount, &QuietInteriorContinentalRetentionSingleSourceCount, &ContinentalBreadthSupport, &ContinentalBreadthPreservationCount, &ContinentalBreadthPreservationStrongInteriorCount, &ContinentalBreadthPreservationModerateInteriorCount, &PaperSurrogateOwnershipOverrideCount, &PaperSurrogateOwnershipOverrideSubaerialCount, &PaperSurrogateOwnershipOverrideSubmergedCount, &PaperSurrogateOwnershipOverrideTriangleCount, &PaperSurrogateOwnershipOverrideSingleSourceCount, &PaperSurrogateOwnershipOverrideStrongInteriorCount, &PaperSurrogateOwnershipOverrideModerateInteriorCount, &PaperSurrogateOwnershipOverrideLowElevationBandCount, &PaperSurrogateOwnershipOverrideModerateElevationBandCount, &PaperSurrogateOwnershipOverrideHighElevationBandCount, &PaperSurrogateQuietInteriorDirectHitEligibleCount, &PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightScaledSum, &PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightScaledSum, &PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightScaledSum, &PaperSurrogateQuietInteriorDirectHitPreSolveElevationScaledSum, &PaperSurrogateQuietInteriorDirectHitPostTransferElevationScaledSum, &PaperSurrogateQuietInteriorDirectHitFinalElevationScaledSum, &PaperSurrogateQuietInteriorDirectHitPreSolveThicknessScaledSum, &PaperSurrogateQuietInteriorDirectHitPostTransferThicknessScaledSum, &PaperSurrogateQuietInteriorDirectHitFinalThicknessScaledSum, &QuietInteriorPreservationMicroseconds, bRecordPhaseTiming](const int32 SampleIndex)
	{
		FSample& Sample = Planet.Samples[SampleIndex];
		const FSample PreviousSample = Sample;
		FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		Sample.PlateId = Resolved.FinalPlateId;
		const bool bTransferred = ApplyCopiedFrontierResolvedTransferStage(
			TransferStageContext,
			SampleIndex,
			Sample,
			Resolved,
			SubductionDistances[SampleIndex],
			SubductionSpeeds[SampleIndex]);

		const double QuietInteriorPreservationStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		const bool bQuietInteriorPaperSurrogateEligibility =
			bTransferred &&
			Resolved.bAuthorityRetainedOutsideActiveZone &&
			!Resolved.bActiveZoneSample &&
			Resolved.PreviousPlateId != INDEX_NONE &&
			Resolved.PreviousPlateId == Resolved.FinalPlateId &&
			PreSolveContinentalWeights.IsValidIndex(SampleIndex) &&
			PreSolveContinentalWeights[SampleIndex] >= 0.5f;
		const bool bQuietInteriorStrongInterior =
			bQuietInteriorPaperSurrogateEligibility &&
			ContinentalBreadthSupport.StrongInteriorFlags.IsValidIndex(SampleIndex) &&
			ContinentalBreadthSupport.StrongInteriorFlags[SampleIndex] != 0;
		const bool bQuietInteriorModerateInterior =
			bQuietInteriorPaperSurrogateEligibility &&
			ContinentalBreadthSupport.ModerateInteriorFlags.IsValidIndex(SampleIndex) &&
			ContinentalBreadthSupport.ModerateInteriorFlags[SampleIndex] != 0;
		const bool bQuietInteriorSupportedInterior =
			bQuietInteriorStrongInterior || bQuietInteriorModerateInterior;

		// Quiet-interior continental retention: prevent retained-owner outside-active-zone
		// samples from flipping below CW 0.5 when there is no tectonic cause.
		// Only protect samples that are genuinely elevated continental crust (pre-solve
		// elevation above sea level). Clamp to 0.5 floor rather than restoring pre-solve
		// CW to avoid positive feedback / runaway continental growth.
		if (bTransferred &&
			bEnableV9QuietInteriorContinentalRetentionForTest &&
			Resolved.bAuthorityRetainedOutsideActiveZone &&
			!Resolved.bActiveZoneSample &&
			Resolved.PreviousPlateId != INDEX_NONE &&
			Resolved.PreviousPlateId == Resolved.FinalPlateId &&
			PreSolveContinentalWeights.IsValidIndex(SampleIndex) &&
			PreSolveContinentalWeights[SampleIndex] >= 0.5f &&
			PreSolveElevations.IsValidIndex(SampleIndex) &&
			PreSolveElevations[SampleIndex] > 0.0f &&
			Sample.ContinentalWeight < 0.5f)
		{
			Sample.ContinentalWeight = 0.5f;
			++QuietInteriorContinentalRetentionCount;
			if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle)
			{
				++QuietInteriorContinentalRetentionTriangleCount;
			}
			else
			{
				++QuietInteriorContinentalRetentionSingleSourceCount;
			}
		}

		if (bTransferred &&
			bEnableV9ContinentalBreadthPreservationForTest &&
			bQuietInteriorPaperSurrogateEligibility &&
			PreSolveElevations.IsValidIndex(SampleIndex) &&
			PreSolveElevations[SampleIndex] > ContinentalBreadthMinQuietElevationKm)
		{
			if (bQuietInteriorSupportedInterior)
			{
				const float PreSolveContinentalWeight = PreSolveContinentalWeights[SampleIndex];
				const float PreserveFactor =
					bQuietInteriorStrongInterior
						? ContinentalBreadthStrongInteriorPreserveFactor
						: ContinentalBreadthModerateInteriorPreserveFactor;
				const float TargetContinentalWeight = FMath::Lerp(
					0.5f,
					PreSolveContinentalWeight,
					PreserveFactor);
				if (Sample.ContinentalWeight < TargetContinentalWeight)
				{
					Sample.ContinentalWeight = TargetContinentalWeight;
					++ContinentalBreadthPreservationCount;
					if (bQuietInteriorStrongInterior)
					{
						++ContinentalBreadthPreservationStrongInteriorCount;
					}
					else
					{
						++ContinentalBreadthPreservationModerateInteriorCount;
					}
				}
			}
		}

		const bool bPaperSurrogateDirectHitQuietInterior =
			bQuietInteriorSupportedInterior &&
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle;
		if (bPaperSurrogateDirectHitQuietInterior)
		{
			++PaperSurrogateQuietInteriorDirectHitEligibleCount;
			const int64 PreSolveCWScaled = FMath::RoundToInt64(
				static_cast<double>(PreSolveContinentalWeights[SampleIndex]) *
				PaperSurrogateDiagnosticScale);
			const int64 PostTransferCWScaled = FMath::RoundToInt64(
				static_cast<double>(Sample.ContinentalWeight) *
				PaperSurrogateDiagnosticScale);
			const int64 PreSolveElevationScaled = FMath::RoundToInt64(
				static_cast<double>(PreviousSample.Elevation) *
				PaperSurrogateDiagnosticScale);
			const int64 PostTransferElevationScaled = FMath::RoundToInt64(
				static_cast<double>(Sample.Elevation) *
				PaperSurrogateDiagnosticScale);
			const int64 PreSolveThicknessScaled = FMath::RoundToInt64(
				static_cast<double>(PreviousSample.Thickness) *
				PaperSurrogateDiagnosticScale);
			const int64 PostTransferThicknessScaled = FMath::RoundToInt64(
				static_cast<double>(Sample.Thickness) *
				PaperSurrogateDiagnosticScale);
			PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightScaledSum += PreSolveCWScaled;
			PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightScaledSum += PostTransferCWScaled;
			PaperSurrogateQuietInteriorDirectHitPreSolveElevationScaledSum += PreSolveElevationScaled;
			PaperSurrogateQuietInteriorDirectHitPostTransferElevationScaledSum += PostTransferElevationScaled;
			PaperSurrogateQuietInteriorDirectHitPreSolveThicknessScaledSum += PreSolveThicknessScaled;
			PaperSurrogateQuietInteriorDirectHitPostTransferThicknessScaledSum += PostTransferThicknessScaled;
		}

		// Paper-surrogate quiet-interior ownership test: if a sample is a broad-interior,
		// retained-owner, outside-active-zone direct hit on its own plate, skip the entire
		// triangle field transfer result and keep the previous crust state for this cycle.
		// This probes whether repeated quiet-interior interpolation is diffusing away
		// continental identity and morphology even when ownership stays stable.
		if (bTransferred &&
			bEnableV9PaperSurrogateOwnershipForTest &&
			bQuietInteriorPaperSurrogateEligibility &&
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle)
		{
			if (bQuietInteriorSupportedInterior)
			{
				const EPaperSurrogateElevationBand ElevationBand =
					ClassifyPaperSurrogateElevationBand(PreviousSample.Elevation);
				switch (V9PaperSurrogateFieldModeForTest)
				{
				case ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightElevationThickness:
					Sample.ContinentalWeight = PreviousSample.ContinentalWeight;
					Sample.Elevation = PreviousSample.Elevation;
					Sample.Thickness = PreviousSample.Thickness;
					break;
				case ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightThickness:
					Sample.ContinentalWeight = PreviousSample.ContinentalWeight;
					Sample.Thickness = PreviousSample.Thickness;
					break;
				case ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightThicknessSelectiveElevation:
				{
					Sample.ContinentalWeight = PreviousSample.ContinentalWeight;
					Sample.Thickness = PreviousSample.Thickness;
					const float ElevationBlend =
						ComputePaperSurrogateSelectiveElevationBlend(PreviousSample.Elevation);
					Sample.Elevation = FMath::Lerp(Sample.Elevation, PreviousSample.Elevation, ElevationBlend);
					break;
				}
				case ETectonicPlanetV6PaperSurrogateFieldMode::FullState:
				default:
					Sample = PreviousSample;
					break;
				}
				Sample.PlateId = Resolved.FinalPlateId;
				++PaperSurrogateOwnershipOverrideCount;
				if (Sample.Elevation > 0.0f)
				{
					++PaperSurrogateOwnershipOverrideSubaerialCount;
				}
				else
				{
					++PaperSurrogateOwnershipOverrideSubmergedCount;
				}
				++PaperSurrogateOwnershipOverrideTriangleCount;
				if (bQuietInteriorStrongInterior)
				{
					++PaperSurrogateOwnershipOverrideStrongInteriorCount;
				}
				else
				{
					++PaperSurrogateOwnershipOverrideModerateInteriorCount;
				}
				switch (ElevationBand)
				{
				case EPaperSurrogateElevationBand::Low:
					++PaperSurrogateOwnershipOverrideLowElevationBandCount;
					break;
				case EPaperSurrogateElevationBand::Moderate:
					++PaperSurrogateOwnershipOverrideModerateElevationBandCount;
					break;
				case EPaperSurrogateElevationBand::High:
				default:
					++PaperSurrogateOwnershipOverrideHighElevationBandCount;
					break;
				}
			}
		}

		if (bTransferred)
		{
			switch (ApplyCopiedFrontierActiveBandFieldContinuity(
				Sample,
				Resolved,
				SampleIndex,
				PreSolveContinentalWeights,
				PreSolveElevations,
				PreSolveThicknesses,
				*TransferStageContext.ConvergentActiveFieldContinuityFlags,
				*TransferStageContext.AdjacentToConvergentActiveFieldContinuityFlags,
				CurrentSolvePreviousOwnerRecoveryFlags,
				CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags,
				CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags))
			{
			case EV6ActiveBandFieldContinuityKind::Triangle:
				++ActiveBandTriangleFieldContinuityClampCount;
				break;
			case EV6ActiveBandFieldContinuityKind::Synthetic:
				++ActiveBandSyntheticFieldPreserveCount;
				break;
			case EV6ActiveBandFieldContinuityKind::Oceanic:
				++ActiveBandOceanicFieldPreserveCount;
				break;
			case EV6ActiveBandFieldContinuityKind::None:
			default:
				break;
			}
		}

		if (!bTransferred)
		{
			Sample.ContinentalWeight = 0.0f;
			Sample.Elevation = -6.0f;
			Sample.Thickness = 7.0f;
			Sample.Age = 0.0f;
			Sample.OrogenyType = EOrogenyType::None;
			Sample.TerraneId = INDEX_NONE;
			Sample.RidgeDirection = FVector3d::ZeroVector;
			Sample.FoldDirection = FVector3d::ZeroVector;
			SubductionDistances[SampleIndex] = -1.0f;
			SubductionSpeeds[SampleIndex] = 0.0f;
			Resolved.TransferDebug = FTectonicPlanetV6TransferDebugInfo{};
			Resolved.TransferDebug.SourceKind = ETectonicPlanetV6TransferSourceKind::Defaulted;
			++(*TransferStageContext.TransferFallbackCount);
		}
		else if (bPaperSurrogateDirectHitQuietInterior)
		{
			const int64 FinalCWScaled = FMath::RoundToInt64(
				static_cast<double>(Sample.ContinentalWeight) *
				PaperSurrogateDiagnosticScale);
			const int64 FinalElevationScaled = FMath::RoundToInt64(
				static_cast<double>(Sample.Elevation) *
				PaperSurrogateDiagnosticScale);
			const int64 FinalThicknessScaled = FMath::RoundToInt64(
				static_cast<double>(Sample.Thickness) *
				PaperSurrogateDiagnosticScale);
			PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightScaledSum += FinalCWScaled;
			PaperSurrogateQuietInteriorDirectHitFinalElevationScaledSum += FinalElevationScaled;
			PaperSurrogateQuietInteriorDirectHitFinalThicknessScaledSum += FinalThicknessScaled;
		}
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMicroseconds(
				QuietInteriorPreservationMicroseconds,
				QuietInteriorPreservationStartTime);
		}
	});

	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(PhaseTiming.ResolveTransferLoopMs, ResolveTransferLoopStartTime);
	}
	const double AttributionStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
	FTectonicPlanetV6PeriodicSolveStats TransferStats;
	for (const FTectonicPlanetV6ResolvedSample& Resolved : LastResolvedSamples)
	{
		switch (Resolved.TransferDebug.SourceKind)
		{
		case ETectonicPlanetV6TransferSourceKind::Triangle:
			++TransferStats.TriangleTransferCount;
			IncrementTransferResolutionCounts(TransferStats.TriangleTransferCountsByResolution, Resolved);
			break;
		case ETectonicPlanetV6TransferSourceKind::SingleSource:
			++TransferStats.SingleSourceTransferCount;
			IncrementTransferResolutionCounts(TransferStats.SingleSourceTransferCountsByResolution, Resolved);
			break;
		case ETectonicPlanetV6TransferSourceKind::StructuredSynthetic:
			++TransferStats.SingleSourceTransferCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::OceanicCreation:
			++TransferStats.OceanicCreationCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::Defaulted:
			++TransferStats.DefaultTransferCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::None:
		default:
			break;
		}

		if (UsesCategoricalMajorityTransfer(Resolved.TransferDebug))
		{
			++TransferStats.CategoricalMajorityTransferCount;
		}
		else if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle ||
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::SingleSource)
		{
			++TransferStats.CategoricalDominantTransferCount;
		}

		if (UsesDirectionalFallback(Resolved.TransferDebug))
		{
			++TransferStats.DirectionalFallbackCount;
		}

		if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
		{
			++TransferStats.ContinentalWeightThresholdMismatchCount;
			IncrementTransferResolutionCounts(
				TransferStats.ContinentalWeightThresholdMismatchCountsByResolution,
				Resolved);
		}
	}

		FTectonicPlanetV6CopiedFrontierSolveAttribution CopiedFrontierAttribution;
		CopiedFrontierAttribution.SolveStep = Planet.CurrentStep;
		CopiedFrontierAttribution.Interval = Interval;
		CopiedFrontierAttribution.HitCount = HitCount.Load();
		CopiedFrontierAttribution.MissCount = MissCount.Load();
		CopiedFrontierAttribution.OceanicCreationCount = TransferStats.OceanicCreationCount;
		CopiedFrontierAttribution.OverlapCoherenceSupportPrunedSampleCount =
			OverlapCoherenceSupportPrunedSampleCount.Load();
		CopiedFrontierAttribution.OverlapCoherencePreviousPlateStabilizedSampleCount =
			OverlapCoherencePreviousPlateStabilizedSampleCount.Load();
		CopiedFrontierAttribution.OverlapCoherenceSuppressedCandidateCount =
			OverlapCoherenceSuppressedCandidateCount.Load();
		const bool bBuildDetailedCopiedFrontierAttribution =
			bEnableDetailedCopiedFrontierAttributionForTest;
		TArray<uint8> GapAttributionBySample;
		GapAttributionBySample.Init(
			static_cast<uint8>(ETectonicPlanetV6CopiedFrontierGapAttribution::None),
			Planet.Samples.Num());
		TArray<uint8> DestructiveExclusionGapMask;
		DestructiveExclusionGapMask.Init(0, Planet.Samples.Num());
		TArray<uint8> DestructiveExclusionContinuityMask;
		DestructiveExclusionContinuityMask.Init(0, Planet.Samples.Num());

		TArray<FTectonicPlanetV6CopiedFrontierRepresentativeMiss> RepresentativeMissCandidates;
		if (bBuildDetailedCopiedFrontierAttribution)
		{
			RepresentativeMissCandidates.Reserve(CopiedFrontierAttribution.MissCount);
		}
		TArray<FTectonicPlanetV6CopiedFrontierRepresentativeHitLoss> RepresentativeSingleHitLossCandidates;
		TMap<int32, int32> SingleHitLossCountsByPreviousPlate;
		TMap<int32, int32> SingleHitLossCountsByFinalPlate;
		TMap<FString, int32> SingleHitLossPatternCounts;
		TMap<FString, int32> MultiHitPatternCounts;

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!LastResolvedSamples.IsValidIndex(SampleIndex) || LastResolvedSamples[SampleIndex].ExactCandidateCount != 0)
			{
				continue;
			}

			const int32 UnfilteredCandidateCount =
				(bApplyDestructiveFilter && UnfilteredExactCandidateCounts.IsValidIndex(SampleIndex))
					? UnfilteredExactCandidateCounts[SampleIndex]
					: 0;
			const int32 UnfilteredDestructiveCandidateCount =
				(bApplyDestructiveFilter && UnfilteredDestructiveCandidateCounts.IsValidIndex(SampleIndex))
					? UnfilteredDestructiveCandidateCounts[SampleIndex]
					: 0;

			ETectonicPlanetV6CopiedFrontierGapAttribution GapAttribution =
				ETectonicPlanetV6CopiedFrontierGapAttribution::None;
			if (bApplyDestructiveFilter)
			{
				if (UnfilteredCandidateCount == 0)
				{
					GapAttribution = ETectonicPlanetV6CopiedFrontierGapAttribution::TrueDivergence;
				}
				else if (UnfilteredDestructiveCandidateCount == UnfilteredCandidateCount)
				{
					GapAttribution = ETectonicPlanetV6CopiedFrontierGapAttribution::DestructiveExclusion;
				}
				else
				{
					GapAttribution = ETectonicPlanetV6CopiedFrontierGapAttribution::Ambiguous;
				}
			}

			GapAttributionBySample[SampleIndex] = static_cast<uint8>(GapAttribution);
			if (GapAttribution == ETectonicPlanetV6CopiedFrontierGapAttribution::DestructiveExclusion)
			{
				DestructiveExclusionGapMask[SampleIndex] = 1;
				if (LastResolvedSamples[SampleIndex].TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle ||
					LastResolvedSamples[SampleIndex].TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::SingleSource)
				{
					DestructiveExclusionContinuityMask[SampleIndex] = 1;
				}
			}
		}

		if (!bBuildDetailedCopiedFrontierAttribution)
		{
			for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
			{
				if (!Planet.Samples.IsValidIndex(SampleIndex) || !LastResolvedSamples.IsValidIndex(SampleIndex))
				{
					continue;
				}

				const FSample& Sample = Planet.Samples[SampleIndex];
				const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
				const bool bWasContinental =
					PreSolveContinentalFlags.IsValidIndex(SampleIndex) &&
					PreSolveContinentalFlags[SampleIndex] != 0;
				const bool bIsContinental = Sample.ContinentalWeight >= 0.5f;

				CopiedFrontierAttribution.ContinentalSamplesBefore += bWasContinental ? 1 : 0;
				CopiedFrontierAttribution.ContinentalSamplesAfter += bIsContinental ? 1 : 0;
				CopiedFrontierAttribution.ContinentalSamplesGained += (!bWasContinental && bIsContinental) ? 1 : 0;

				if (Resolved.ExactCandidateCount > 0)
				{
					const bool bCopiedFrontierHit =
						Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle &&
						Resolved.TransferDebug.bUsedCopiedFrontierTriangleTransfer;
					const bool bSamePlateWinner = Resolved.FinalPlateId == Resolved.PreviousPlateId;

					if (Resolved.ExactCandidateCount > 1)
					{
						++CopiedFrontierAttribution.MultiHitWinnerCount;
						CopiedFrontierAttribution.MultiHitWinnerSamePlateCount += bSamePlateWinner ? 1 : 0;
						CopiedFrontierAttribution.MultiHitWinnerCrossPlateCount += !bSamePlateWinner ? 1 : 0;
						CopiedFrontierAttribution.MultiHitWinnerCopiedFrontierHitCount += bCopiedFrontierHit ? 1 : 0;
						CopiedFrontierAttribution.MultiHitWinnerInteriorHitCount += !bCopiedFrontierHit ? 1 : 0;
					}
					else
					{
						++CopiedFrontierAttribution.SingleHitWinnerCount;
					}

					if (bCopiedFrontierHit)
					{
						++CopiedFrontierAttribution.CopiedFrontierHitCount;
					}
					else
					{
						++CopiedFrontierAttribution.InteriorHitCount;
					}

					if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
					{
						++CopiedFrontierAttribution.HitCwThresholdMismatchCount;
						if (bCopiedFrontierHit)
						{
							++CopiedFrontierAttribution.CopiedFrontierHitCwThresholdMismatchCount;
						}
						else
						{
							++CopiedFrontierAttribution.InteriorHitCwThresholdMismatchCount;
						}
					}

					if (bApplyDestructiveFilter &&
						UnfilteredExactCandidateCounts.IsValidIndex(SampleIndex) &&
						UnfilteredDestructiveCandidateCounts.IsValidIndex(SampleIndex) &&
						UnfilteredExactCandidateCounts[SampleIndex] > Resolved.ExactCandidateCount &&
						UnfilteredDestructiveCandidateCounts[SampleIndex] > 0)
					{
						++CopiedFrontierAttribution.SamplesWithPartialDestructiveCandidateRemovalCount;
					}
				}

				if (bWasContinental && !bIsContinental)
				{
					++CopiedFrontierAttribution.ContinentalSamplesLost;
					switch (Resolved.TransferDebug.SourceKind)
					{
					case ETectonicPlanetV6TransferSourceKind::OceanicCreation:
						++CopiedFrontierAttribution.ContinentalLossFromOceanicCreationCount;
						break;
					case ETectonicPlanetV6TransferSourceKind::Triangle:
						++CopiedFrontierAttribution.ContinentalLossFromHitCount;
						if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
						{
							++CopiedFrontierAttribution.ContinentalLossFromHitCwThresholdMismatchCount;
						}
						if (Resolved.ExactCandidateCount > 1)
						{
							++CopiedFrontierAttribution.ContinentalLossFromHitMultiWinnerCount;
						}
						else if (Resolved.ExactCandidateCount == 1)
						{
							++CopiedFrontierAttribution.ContinentalLossFromHitSingleWinnerCount;
						}
						break;
					default:
						break;
					}

					if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle)
					{
						const bool bCopiedFrontierHit =
							Resolved.TransferDebug.bUsedCopiedFrontierTriangleTransfer;
						const bool bSamePlateWinner = Resolved.FinalPlateId == Resolved.PreviousPlateId;

						if (Resolved.ExactCandidateCount == 1)
						{
							CopiedFrontierAttribution.SingleHitContinentalLossSamePlateCount += bSamePlateWinner ? 1 : 0;
							CopiedFrontierAttribution.SingleHitContinentalLossCrossPlateCount += !bSamePlateWinner ? 1 : 0;
							CopiedFrontierAttribution.SingleHitContinentalLossCopiedFrontierHitCount += bCopiedFrontierHit ? 1 : 0;
							CopiedFrontierAttribution.SingleHitContinentalLossInteriorHitCount += !bCopiedFrontierHit ? 1 : 0;
						}
						else if (Resolved.ExactCandidateCount > 1)
						{
							CopiedFrontierAttribution.MultiHitContinentalLossSamePlateCount += bSamePlateWinner ? 1 : 0;
							CopiedFrontierAttribution.MultiHitContinentalLossCrossPlateCount += !bSamePlateWinner ? 1 : 0;
							CopiedFrontierAttribution.MultiHitContinentalLossCopiedFrontierHitCount += bCopiedFrontierHit ? 1 : 0;
							CopiedFrontierAttribution.MultiHitContinentalLossInteriorHitCount += !bCopiedFrontierHit ? 1 : 0;
						}
					}
				}

				if (Resolved.ExactCandidateCount != 0)
				{
					continue;
				}

				const ETectonicPlanetV6CopiedFrontierGapAttribution GapAttribution =
					GapAttributionBySample.IsValidIndex(SampleIndex)
						? static_cast<ETectonicPlanetV6CopiedFrontierGapAttribution>(GapAttributionBySample[SampleIndex])
						: ETectonicPlanetV6CopiedFrontierGapAttribution::None;
				const bool bUsedStructuredSyntheticFill =
					Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::StructuredSynthetic;

				if (bUsedStructuredSyntheticFill)
				{
					++CopiedFrontierAttribution.MissFrontierPairStructuredSyntheticCount;
					++CopiedFrontierAttribution.MissFrontierPairVertexApproximationCount;
				}

				switch (GapAttribution)
				{
				case ETectonicPlanetV6CopiedFrontierGapAttribution::TrueDivergence:
					++CopiedFrontierAttribution.MissTrueDivergenceGapCount;
					if (bUsedStructuredSyntheticFill)
					{
						++CopiedFrontierAttribution.TrueDivergenceStructuredSyntheticCount;
					}
					break;
				case ETectonicPlanetV6CopiedFrontierGapAttribution::DestructiveExclusion:
					++CopiedFrontierAttribution.MissDestructiveExclusionGapCount;
					break;
				case ETectonicPlanetV6CopiedFrontierGapAttribution::Ambiguous:
					++CopiedFrontierAttribution.MissAmbiguousGapCount;
					if (bUsedStructuredSyntheticFill)
					{
						++CopiedFrontierAttribution.AmbiguousStructuredSyntheticCount;
					}
					break;
				case ETectonicPlanetV6CopiedFrontierGapAttribution::None:
				default:
					break;
				}

				if (GapAttribution == ETectonicPlanetV6CopiedFrontierGapAttribution::DestructiveExclusion)
				{
					if (bUsedStructuredSyntheticFill)
					{
						++CopiedFrontierAttribution.DestructiveExclusionStructuredSyntheticCount;
					}
					else if (DestructiveExclusionContinuityMask.IsValidIndex(SampleIndex) &&
						DestructiveExclusionContinuityMask[SampleIndex] != 0)
					{
						++CopiedFrontierAttribution.DestructiveExclusionOverrideContinuityCount;
					}
					else
					{
						++CopiedFrontierAttribution.DestructiveExclusionFallbackOceanicCount;
					}
				}
			}
		}
		else
		{
			for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
			{
		if (!Planet.Samples.IsValidIndex(SampleIndex) || !LastResolvedSamples.IsValidIndex(SampleIndex))
		{
			continue;
		}

		const FSample& Sample = Planet.Samples[SampleIndex];
		const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		const bool bWasContinental = PreSolveContinentalFlags.IsValidIndex(SampleIndex) && PreSolveContinentalFlags[SampleIndex] != 0;
		const bool bIsContinental = Sample.ContinentalWeight >= 0.5f;

		CopiedFrontierAttribution.ContinentalSamplesBefore += bWasContinental ? 1 : 0;
		CopiedFrontierAttribution.ContinentalSamplesAfter += bIsContinental ? 1 : 0;
		CopiedFrontierAttribution.ContinentalSamplesGained += (!bWasContinental && bIsContinental) ? 1 : 0;

		if (Resolved.ExactCandidateCount > 0)
		{
			if (Resolved.ExactCandidateCount > 1)
			{
				++CopiedFrontierAttribution.MultiHitWinnerCount;
			}
			else
			{
				++CopiedFrontierAttribution.SingleHitWinnerCount;
			}

			const bool bCopiedFrontierHit =
				Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle &&
				Resolved.TransferDebug.bUsedCopiedFrontierTriangleTransfer;
			if (bCopiedFrontierHit)
			{
				++CopiedFrontierAttribution.CopiedFrontierHitCount;
			}
			else
			{
				++CopiedFrontierAttribution.InteriorHitCount;
			}

			if (Resolved.ExactCandidateCount > 1)
			{
				const bool bSamePlateWinner = Resolved.FinalPlateId == Resolved.PreviousPlateId;
				const bool bAdjacentToCopiedFrontierGeometry =
					IsSampleAdjacentToCopiedFrontierGeometry(
						SampleToAdjacentTriangles,
						SolveCopiedFrontierMeshes,
						SampleIndex);
				const double NearestCopiedFrontierTriangleDistanceKm =
					ComputeNearestCopiedFrontierTriangleDistanceKm(Planet, QueryGeometries, Sample.Position);
				const bool bNearFrontier =
					bAdjacentToCopiedFrontierGeometry ||
					(NearestCopiedFrontierTriangleDistanceKm >= 0.0 &&
						NearestCopiedFrontierTriangleDistanceKm <= 500.0);

				CopiedFrontierAttribution.MultiHitWinnerSamePlateCount += bSamePlateWinner ? 1 : 0;
				CopiedFrontierAttribution.MultiHitWinnerCrossPlateCount += !bSamePlateWinner ? 1 : 0;
				CopiedFrontierAttribution.MultiHitWinnerCopiedFrontierHitCount += bCopiedFrontierHit ? 1 : 0;
				CopiedFrontierAttribution.MultiHitWinnerInteriorHitCount += !bCopiedFrontierHit ? 1 : 0;
				CopiedFrontierAttribution.MultiHitWinnerNearFrontierCount += bNearFrontier ? 1 : 0;

				const bool bHasTrackedCandidate =
					MultiHitTrackedCandidateFlags.IsValidIndex(SampleIndex) &&
					MultiHitTrackedCandidateFlags[SampleIndex] != 0;
				bool bTouchesTrackedTriangle = false;
				bool bWithinTrackedOneRing = false;
				bool bWithinTrackedTwoRing = false;
				ComputeTrackedDestructiveTriangleProximity(
					Planet,
					SampleToAdjacentTriangles,
					CopiedFrontierTrackedDestructiveKinds,
					SampleIndex,
					bTouchesTrackedTriangle,
					bWithinTrackedOneRing,
					bWithinTrackedTwoRing);

				CopiedFrontierAttribution.MultiHitSamplesWithTrackedCandidateCount += bHasTrackedCandidate ? 1 : 0;
				CopiedFrontierAttribution.MultiHitSamplesWithinTrackedTriangleCount += bTouchesTrackedTriangle ? 1 : 0;
				CopiedFrontierAttribution.MultiHitSamplesWithinTrackedOneRingCount += bWithinTrackedOneRing ? 1 : 0;
				CopiedFrontierAttribution.MultiHitSamplesWithinTrackedTwoRingCount += bWithinTrackedTwoRing ? 1 : 0;

				const EV6CopiedFrontierMultiHitPlateMix PlateMix =
					MultiHitPlateMixValues.IsValidIndex(SampleIndex)
						? static_cast<EV6CopiedFrontierMultiHitPlateMix>(MultiHitPlateMixValues[SampleIndex])
						: EV6CopiedFrontierMultiHitPlateMix::None;
				const EV6CopiedFrontierMultiHitGeometryMix GeometryMix =
					MultiHitGeometryMixValues.IsValidIndex(SampleIndex)
						? static_cast<EV6CopiedFrontierMultiHitGeometryMix>(MultiHitGeometryMixValues[SampleIndex])
						: EV6CopiedFrontierMultiHitGeometryMix::None;

				switch (PlateMix)
				{
				case EV6CopiedFrontierMultiHitPlateMix::SameOnly:
					++CopiedFrontierAttribution.MultiHitCandidateSamePlateOnlyCount;
					break;
				case EV6CopiedFrontierMultiHitPlateMix::CrossOnly:
					++CopiedFrontierAttribution.MultiHitCandidateCrossPlateOnlyCount;
					break;
				case EV6CopiedFrontierMultiHitPlateMix::Mixed:
					++CopiedFrontierAttribution.MultiHitCandidateMixedPlateCount;
					break;
				case EV6CopiedFrontierMultiHitPlateMix::None:
				default:
					break;
				}

				switch (GeometryMix)
				{
				case EV6CopiedFrontierMultiHitGeometryMix::CopiedFrontierOnly:
					++CopiedFrontierAttribution.MultiHitCandidateCopiedFrontierOnlyCount;
					break;
				case EV6CopiedFrontierMultiHitGeometryMix::InteriorOnly:
					++CopiedFrontierAttribution.MultiHitCandidateInteriorOnlyCount;
					break;
				case EV6CopiedFrontierMultiHitGeometryMix::Mixed:
					++CopiedFrontierAttribution.MultiHitCandidateMixedGeometryCount;
					break;
				case EV6CopiedFrontierMultiHitGeometryMix::None:
				default:
					break;
				}

				bool bAdjacentToDestructiveGap = false;
				bool bAdjacentToContinuityHandledDestructiveGap = false;
				if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
					{
						bAdjacentToDestructiveGap |=
							DestructiveExclusionGapMask.IsValidIndex(NeighborIndex) &&
							DestructiveExclusionGapMask[NeighborIndex] != 0;
						bAdjacentToContinuityHandledDestructiveGap |=
							DestructiveExclusionContinuityMask.IsValidIndex(NeighborIndex) &&
							DestructiveExclusionContinuityMask[NeighborIndex] != 0;
						if (bAdjacentToDestructiveGap && bAdjacentToContinuityHandledDestructiveGap)
						{
							break;
						}
					}
				}

				CopiedFrontierAttribution.MultiHitAdjacentToDestructiveGapCount += bAdjacentToDestructiveGap ? 1 : 0;
				CopiedFrontierAttribution.MultiHitAdjacentToContinuityHandledDestructiveGapCount +=
					bAdjacentToContinuityHandledDestructiveGap ? 1 : 0;

				++MultiHitPatternCounts.FindOrAdd(
					BuildCopiedFrontierMultiHitPatternKey(
						bHasTrackedCandidate,
						bWithinTrackedOneRing,
						bWithinTrackedTwoRing,
						PlateMix,
						GeometryMix,
						bSamePlateWinner,
						bAdjacentToDestructiveGap,
						bAdjacentToContinuityHandledDestructiveGap));
			}

			if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
			{
				++CopiedFrontierAttribution.HitCwThresholdMismatchCount;
				if (bCopiedFrontierHit)
				{
					++CopiedFrontierAttribution.CopiedFrontierHitCwThresholdMismatchCount;
				}
				else
				{
					++CopiedFrontierAttribution.InteriorHitCwThresholdMismatchCount;
				}
			}

			if (bApplyDestructiveFilter &&
				UnfilteredExactCandidateCounts.IsValidIndex(SampleIndex) &&
				UnfilteredDestructiveCandidateCounts.IsValidIndex(SampleIndex) &&
				UnfilteredExactCandidateCounts[SampleIndex] > Resolved.ExactCandidateCount &&
				UnfilteredDestructiveCandidateCounts[SampleIndex] > 0)
			{
				++CopiedFrontierAttribution.SamplesWithPartialDestructiveCandidateRemovalCount;
			}
		}

			if (bWasContinental && !bIsContinental)
			{
				++CopiedFrontierAttribution.ContinentalSamplesLost;
				switch (Resolved.TransferDebug.SourceKind)
				{
			case ETectonicPlanetV6TransferSourceKind::OceanicCreation:
				++CopiedFrontierAttribution.ContinentalLossFromOceanicCreationCount;
				break;
				case ETectonicPlanetV6TransferSourceKind::Triangle:
				++CopiedFrontierAttribution.ContinentalLossFromHitCount;
					if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
					{
						++CopiedFrontierAttribution.ContinentalLossFromHitCwThresholdMismatchCount;
				}
				if (Resolved.ExactCandidateCount > 1)
				{
					++CopiedFrontierAttribution.ContinentalLossFromHitMultiWinnerCount;
				}
				else if (Resolved.ExactCandidateCount == 1)
				{
					++CopiedFrontierAttribution.ContinentalLossFromHitSingleWinnerCount;
				}
				break;
				default:
					break;
				}

				if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle)
				{
					const bool bCopiedFrontierHit =
						Resolved.TransferDebug.bUsedCopiedFrontierTriangleTransfer;
					const bool bSamePlateWinner = Resolved.FinalPlateId == Resolved.PreviousPlateId;
					const bool bAdjacentToCopiedFrontierGeometry =
						IsSampleAdjacentToCopiedFrontierGeometry(
							SampleToAdjacentTriangles,
							SolveCopiedFrontierMeshes,
							SampleIndex);
					const double NearestCopiedFrontierTriangleDistanceKm =
						ComputeNearestCopiedFrontierTriangleDistanceKm(Planet, QueryGeometries, Sample.Position);
					const bool bNearFrontier =
						bAdjacentToCopiedFrontierGeometry ||
						(NearestCopiedFrontierTriangleDistanceKm >= 0.0 &&
							NearestCopiedFrontierTriangleDistanceKm <= 500.0);

					int32 DominantSourcePreviousPlateId = INDEX_NONE;
					float DominantSourceContinentalWeight = 0.0f;
					bool bDominantSourceWasContinental = false;
					if (Planet.Samples.IsValidIndex(Resolved.TransferDebug.DominantSourceCanonicalSampleIndex))
					{
						const int32 DominantSourceCanonicalSampleIndex =
							Resolved.TransferDebug.DominantSourceCanonicalSampleIndex;
						DominantSourcePreviousPlateId =
							PreSolvePlateIds.IsValidIndex(DominantSourceCanonicalSampleIndex)
								? PreSolvePlateIds[DominantSourceCanonicalSampleIndex]
								: INDEX_NONE;
						DominantSourceContinentalWeight =
							PreSolveContinentalWeights.IsValidIndex(DominantSourceCanonicalSampleIndex)
								? PreSolveContinentalWeights[DominantSourceCanonicalSampleIndex]
								: 0.0f;
						bDominantSourceWasContinental =
							PreSolveContinentalFlags.IsValidIndex(DominantSourceCanonicalSampleIndex) &&
							PreSolveContinentalFlags[DominantSourceCanonicalSampleIndex] != 0;
					}

					int32 SourceTriangleUniquePreviousPlateCount = 0;
					int32 SourceTriangleContinentalCornerCount = 0;
					const int32 PlateIndex = Planet.FindPlateArrayIndexById(Resolved.FinalPlateId);
					if (SolveCopiedFrontierMeshes.IsValidIndex(PlateIndex))
					{
						const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = SolveCopiedFrontierMeshes[PlateIndex];
						int32 LocalTriangleIndex = Resolved.SourceLocalTriangleIndex;
						if (!Mesh.LocalTriangles.IsValidIndex(LocalTriangleIndex))
						{
							if (const int32* LocalTriangleIndexPtr = Mesh.GlobalToLocalTriangle.Find(Resolved.SourceTriangleIndex))
							{
								LocalTriangleIndex = *LocalTriangleIndexPtr;
							}
						}

						if (Mesh.LocalTriangles.IsValidIndex(LocalTriangleIndex))
						{
							const UE::Geometry::FIndex3i& LocalTriangle = Mesh.LocalTriangles[LocalTriangleIndex];
							TSet<int32> UniquePreviousPlateIds;
							const int32 LocalVertices[3] = { LocalTriangle.A, LocalTriangle.B, LocalTriangle.C };
							for (const int32 LocalVertexIndex : LocalVertices)
							{
								if (!Mesh.LocalToCanonicalVertex.IsValidIndex(LocalVertexIndex))
								{
									continue;
								}

								const int32 CanonicalSampleIndex = Mesh.LocalToCanonicalVertex[LocalVertexIndex];
								if (PreSolvePlateIds.IsValidIndex(CanonicalSampleIndex))
								{
									UniquePreviousPlateIds.Add(PreSolvePlateIds[CanonicalSampleIndex]);
								}
								SourceTriangleContinentalCornerCount +=
									(PreSolveContinentalFlags.IsValidIndex(CanonicalSampleIndex) &&
										PreSolveContinentalFlags[CanonicalSampleIndex] != 0)
										? 1
										: 0;
							}
							SourceTriangleUniquePreviousPlateCount = UniquePreviousPlateIds.Num();
						}
					}

					int32 NoCapHitCount = 0;
					int32 PreviousPlateNoCapHitCount = 0;
					int32 OtherPlateNoCapHitCount = 0;
					for (const FV6PlateQueryGeometry& QueryGeometry : QueryGeometries)
					{
						FV6ThesisRemeshRayHit NoCapHit;
						if (!TryFindThesisRemeshHitIgnoringBoundingCap(QueryGeometry, Sample.Position, NoCapHit))
						{
							continue;
						}

						++NoCapHitCount;
						PreviousPlateNoCapHitCount += QueryGeometry.PlateId == Resolved.PreviousPlateId ? 1 : 0;
						OtherPlateNoCapHitCount +=
							(QueryGeometry.PlateId != Resolved.FinalPlateId && QueryGeometry.PlateId != Resolved.PreviousPlateId)
								? 1
								: 0;
					}

					if (Resolved.ExactCandidateCount == 1)
					{
						CopiedFrontierAttribution.SingleHitContinentalLossSamePlateCount += bSamePlateWinner ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossCrossPlateCount += !bSamePlateWinner ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossCopiedFrontierHitCount += bCopiedFrontierHit ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossInteriorHitCount += !bCopiedFrontierHit ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossNearFrontierCount += bNearFrontier ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossInteriorRegionCount += !bNearFrontier ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossDominantSourceContinentalCount +=
							bDominantSourceWasContinental ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossDominantSourceSubcontinentalCount +=
							!bDominantSourceWasContinental ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossAlternativeNoCapHitCount += NoCapHitCount > 1 ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossPreviousPlateNoCapHitCount +=
							(PreviousPlateNoCapHitCount > 0 && Resolved.PreviousPlateId != Resolved.FinalPlateId) ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossOtherPlateNoCapHitCount +=
							OtherPlateNoCapHitCount > 0 ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossSourceTriangleMixedPreviousPlateCount +=
							SourceTriangleUniquePreviousPlateCount > 1 ? 1 : 0;
						CopiedFrontierAttribution.SingleHitContinentalLossSourceTriangleMixedContinentalCount +=
							SourceTriangleContinentalCornerCount > 0 && SourceTriangleContinentalCornerCount < 3 ? 1 : 0;

						IncrementPlateCount(SingleHitLossCountsByPreviousPlate, Resolved.PreviousPlateId);
						IncrementPlateCount(SingleHitLossCountsByFinalPlate, Resolved.FinalPlateId);
						++SingleHitLossPatternCounts.FindOrAdd(
							BuildSingleHitLossPatternKey(
								bSamePlateWinner,
								bCopiedFrontierHit,
								bNearFrontier,
								bDominantSourceWasContinental,
								PreviousPlateNoCapHitCount > 0 && Resolved.PreviousPlateId != Resolved.FinalPlateId));

						FTectonicPlanetV6CopiedFrontierRepresentativeHitLoss& Representative =
							RepresentativeSingleHitLossCandidates.AddDefaulted_GetRef();
						Representative.SampleIndex = SampleIndex;
						Representative.PreviousPlateId = Resolved.PreviousPlateId;
						Representative.FinalPlateId = Resolved.FinalPlateId;
						Representative.DominantSourceCanonicalSampleIndex =
							Resolved.TransferDebug.DominantSourceCanonicalSampleIndex;
						Representative.DominantSourcePreviousPlateId = DominantSourcePreviousPlateId;
						Representative.NoCapHitCount = NoCapHitCount;
						Representative.PreviousPlateNoCapHitCount = PreviousPlateNoCapHitCount;
						Representative.OtherPlateNoCapHitCount = OtherPlateNoCapHitCount;
						Representative.SourceTriangleUniquePreviousPlateCount = SourceTriangleUniquePreviousPlateCount;
						Representative.SourceTriangleContinentalCornerCount = SourceTriangleContinentalCornerCount;
						Representative.DominantSourceContinentalWeight = DominantSourceContinentalWeight;
						Representative.TransferredContinentalWeight = Sample.ContinentalWeight;
						Representative.WinningFitScore = Resolved.WinningFitScore;
						Representative.NearestCopiedFrontierTriangleDistance = NearestCopiedFrontierTriangleDistanceKm;
						Representative.bSingleHit = true;
						Representative.bSamePlateWinner = bSamePlateWinner;
						Representative.bCopiedFrontierWinningTriangle = bCopiedFrontierHit;
						Representative.bNearFrontier = bNearFrontier;
						Representative.bAdjacentToCopiedFrontierGeometry = bAdjacentToCopiedFrontierGeometry;
						Representative.bDominantSourceWasContinental = bDominantSourceWasContinental;
					}
					else if (Resolved.ExactCandidateCount > 1)
					{
						CopiedFrontierAttribution.MultiHitContinentalLossSamePlateCount += bSamePlateWinner ? 1 : 0;
						CopiedFrontierAttribution.MultiHitContinentalLossCrossPlateCount += !bSamePlateWinner ? 1 : 0;
						CopiedFrontierAttribution.MultiHitContinentalLossCopiedFrontierHitCount += bCopiedFrontierHit ? 1 : 0;
						CopiedFrontierAttribution.MultiHitContinentalLossInteriorHitCount += !bCopiedFrontierHit ? 1 : 0;
						CopiedFrontierAttribution.MultiHitContinentalLossNearFrontierCount += bNearFrontier ? 1 : 0;
					}
				}
			}

			if (Resolved.ExactCandidateCount != 0)
			{
			continue;
		}

		const bool bAdjacentToCopiedFrontierGeometry =
			IsSampleAdjacentToCopiedFrontierGeometry(SampleToAdjacentTriangles, SolveCopiedFrontierMeshes, SampleIndex);
		CopiedFrontierAttribution.MissAdjacentToCopiedFrontierGeometryCount += bAdjacentToCopiedFrontierGeometry ? 1 : 0;

		bool bNearMultiHitRegion = false;
		if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (LastResolvedSamples.IsValidIndex(NeighborIndex) &&
					LastResolvedSamples[NeighborIndex].ExactCandidateCount > 1)
				{
					bNearMultiHitRegion = true;
					break;
				}
			}
		}
		CopiedFrontierAttribution.MissNearMultiHitRegionCount += bNearMultiHitRegion ? 1 : 0;
		CopiedFrontierAttribution.MissAdjacentFrontierAndNearMultiHitCount +=
			(bAdjacentToCopiedFrontierGeometry && bNearMultiHitRegion) ? 1 : 0;

		TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
		CollectThesisRemeshMissRecoveryCandidates(QueryGeometries, Sample.Position, RecoveryCandidates);
		int32 PrimaryNearestPlateId = INDEX_NONE;
		double PrimaryNearestDistanceKm = -1.0;
		int32 SecondaryNearestPlateId = INDEX_NONE;
		double SecondaryNearestDistanceKm = -1.0;
		FindPrimaryAndSecondaryRecoveryPlatesKm(
			Planet,
			RecoveryCandidates,
			PrimaryNearestPlateId,
			PrimaryNearestDistanceKm,
			SecondaryNearestPlateId,
			SecondaryNearestDistanceKm);

		TArray<int32> RelevantPlateIds;
		if (PreSolvePlateIds.IsValidIndex(SampleIndex) && PreSolvePlateIds[SampleIndex] != INDEX_NONE)
		{
			RelevantPlateIds.AddUnique(PreSolvePlateIds[SampleIndex]);
		}
		if (PrimaryNearestPlateId != INDEX_NONE)
		{
			RelevantPlateIds.AddUnique(PrimaryNearestPlateId);
		}
		if (SecondaryNearestPlateId != INDEX_NONE)
		{
			RelevantPlateIds.AddUnique(SecondaryNearestPlateId);
		}

		TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
		BuildBoundaryMotionSamplesForPlateIds(Planet, PreSolvePlateIds, SampleIndex, RelevantPlateIds, MotionSamples);

		double RelativeNormalVelocity = 0.0;
		const EV6BoundaryMotionClass MotionClass =
			(PrimaryNearestPlateId != INDEX_NONE && SecondaryNearestPlateId != INDEX_NONE)
				? ClassifyBoundaryMotion(
					Sample.Position,
					PrimaryNearestPlateId,
					SecondaryNearestPlateId,
					MotionSamples,
					RelativeNormalVelocity)
				: EV6BoundaryMotionClass::None;

		switch (MotionClass)
		{
		case EV6BoundaryMotionClass::Divergent:
			++CopiedFrontierAttribution.MissDivergentMotionCount;
			CopiedFrontierAttribution.MissAdjacentFrontierAndDivergentCount += bAdjacentToCopiedFrontierGeometry ? 1 : 0;
			break;
		case EV6BoundaryMotionClass::Convergent:
			++CopiedFrontierAttribution.MissConvergentMotionCount;
			CopiedFrontierAttribution.MissAdjacentFrontierAndConvergentCount += bAdjacentToCopiedFrontierGeometry ? 1 : 0;
			break;
		case EV6BoundaryMotionClass::Weak:
		case EV6BoundaryMotionClass::None:
		default:
			++CopiedFrontierAttribution.MissWeakOrNoMotionCount;
			CopiedFrontierAttribution.MissAdjacentFrontierAndWeakCount += bAdjacentToCopiedFrontierGeometry ? 1 : 0;
			break;
		}

		const double NearestCopiedFrontierTriangleDistanceKm =
			ComputeNearestCopiedFrontierTriangleDistanceKm(Planet, QueryGeometries, Sample.Position);
		if (NearestCopiedFrontierTriangleDistanceKm >= 0.0)
		{
			if (NearestCopiedFrontierTriangleDistanceKm <= 100.0)
			{
				++CopiedFrontierAttribution.MissNearestCopiedFrontierDistanceLe100KmCount;
			}
			if (NearestCopiedFrontierTriangleDistanceKm <= 500.0)
			{
				++CopiedFrontierAttribution.MissNearestCopiedFrontierDistanceLe500KmCount;
			}
			if (NearestCopiedFrontierTriangleDistanceKm <= 1000.0)
			{
				++CopiedFrontierAttribution.MissNearestCopiedFrontierDistanceLe1000KmCount;
			}
			if (NearestCopiedFrontierTriangleDistanceKm <= 4000.0)
			{
				++CopiedFrontierAttribution.MissNearestCopiedFrontierDistanceLe4000KmCount;
			}
			else
			{
				++CopiedFrontierAttribution.MissNearestCopiedFrontierDistanceGt4000KmCount;
			}
		}

		const bool bLikelyCoverageFailure =
			bAdjacentToCopiedFrontierGeometry ||
			(NearestCopiedFrontierTriangleDistanceKm >= 0.0 && NearestCopiedFrontierTriangleDistanceKm <= 100.0);
		const bool bLikelyTrueDivergenceGap =
			MotionClass == EV6BoundaryMotionClass::Divergent &&
			!bAdjacentToCopiedFrontierGeometry &&
			!bNearMultiHitRegion &&
			NearestCopiedFrontierTriangleDistanceKm > 500.0;
		CopiedFrontierAttribution.MissLikelyCoverageFailureCount += bLikelyCoverageFailure ? 1 : 0;
		CopiedFrontierAttribution.MissLikelyTrueDivergenceGapCount += bLikelyTrueDivergenceGap ? 1 : 0;

		const int32 UnfilteredCandidateCount =
			(bApplyDestructiveFilter && UnfilteredExactCandidateCounts.IsValidIndex(SampleIndex))
				? UnfilteredExactCandidateCounts[SampleIndex]
				: 0;
		const int32 UnfilteredDestructiveCandidateCount =
			(bApplyDestructiveFilter && UnfilteredDestructiveCandidateCounts.IsValidIndex(SampleIndex))
				? UnfilteredDestructiveCandidateCounts[SampleIndex]
				: 0;
		const ETectonicPlanetV6CopiedFrontierGapAttribution GapAttribution =
			GapAttributionBySample.IsValidIndex(SampleIndex)
				? static_cast<ETectonicPlanetV6CopiedFrontierGapAttribution>(GapAttributionBySample[SampleIndex])
				: ETectonicPlanetV6CopiedFrontierGapAttribution::None;
		const bool bUsedStructuredSyntheticFill =
			LastResolvedSamples[SampleIndex].TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::StructuredSynthetic;

		if (bUsedStructuredSyntheticFill)
		{
			++CopiedFrontierAttribution.MissFrontierPairStructuredSyntheticCount;
			++CopiedFrontierAttribution.MissFrontierPairVertexApproximationCount;
		}

		switch (GapAttribution)
		{
		case ETectonicPlanetV6CopiedFrontierGapAttribution::TrueDivergence:
			++CopiedFrontierAttribution.MissTrueDivergenceGapCount;
			if (bUsedStructuredSyntheticFill)
			{
				++CopiedFrontierAttribution.TrueDivergenceStructuredSyntheticCount;
			}
			break;
		case ETectonicPlanetV6CopiedFrontierGapAttribution::DestructiveExclusion:
			++CopiedFrontierAttribution.MissDestructiveExclusionGapCount;
			break;
		case ETectonicPlanetV6CopiedFrontierGapAttribution::Ambiguous:
			++CopiedFrontierAttribution.MissAmbiguousGapCount;
			if (bUsedStructuredSyntheticFill)
			{
				++CopiedFrontierAttribution.AmbiguousStructuredSyntheticCount;
			}
			break;
		case ETectonicPlanetV6CopiedFrontierGapAttribution::None:
		default:
			break;
		}

		if (GapAttribution == ETectonicPlanetV6CopiedFrontierGapAttribution::DestructiveExclusion)
		{
			if (bUsedStructuredSyntheticFill)
			{
				++CopiedFrontierAttribution.DestructiveExclusionStructuredSyntheticCount;
			}
			else if (DestructiveExclusionContinuityMask.IsValidIndex(SampleIndex) &&
				DestructiveExclusionContinuityMask[SampleIndex] != 0)
			{
				++CopiedFrontierAttribution.DestructiveExclusionOverrideContinuityCount;
			}
			else
			{
				++CopiedFrontierAttribution.DestructiveExclusionFallbackOceanicCount;
			}
		}

		FTectonicPlanetV6CopiedFrontierRepresentativeMiss& RepresentativeMiss =
			RepresentativeMissCandidates.AddDefaulted_GetRef();
		RepresentativeMiss.SampleIndex = SampleIndex;
		RepresentativeMiss.PreviousPlateId = PreSolvePlateIds.IsValidIndex(SampleIndex) ? PreSolvePlateIds[SampleIndex] : INDEX_NONE;
		RepresentativeMiss.PrimaryNearestPlateId = PrimaryNearestPlateId;
		RepresentativeMiss.SecondaryNearestPlateId = SecondaryNearestPlateId;
		RepresentativeMiss.PrimaryNearestTriangleDistance = PrimaryNearestDistanceKm;
		RepresentativeMiss.SecondaryNearestTriangleDistance = SecondaryNearestDistanceKm;
		RepresentativeMiss.NearestCopiedFrontierTriangleDistance = NearestCopiedFrontierTriangleDistanceKm;
		RepresentativeMiss.RelativeNormalVelocity = RelativeNormalVelocity;
		RepresentativeMiss.UnfilteredCandidateCount = UnfilteredCandidateCount;
		RepresentativeMiss.UnfilteredDestructiveCandidateCount = UnfilteredDestructiveCandidateCount;
		RepresentativeMiss.bAdjacentToCopiedFrontierGeometry = bAdjacentToCopiedFrontierGeometry;
		RepresentativeMiss.bNearMultiHitRegion = bNearMultiHitRegion;
		RepresentativeMiss.MotionClass = ToCopiedFrontierMotionClass(MotionClass);
		RepresentativeMiss.GapAttribution = GapAttribution;
	}
		}
	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(PhaseTiming.AttributionMs, AttributionStartTime);
	}

	const double RepartitionStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
	Planet.RepartitionMembership(NewPlateIds, &SubductionDistances, &SubductionSpeeds);
	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(PhaseTiming.RepartitionMembershipMs, RepartitionStartTime);
	}
	FSubductionComputationDiagnostics SubductionFieldDiagnostics;
	FSubductionComputationDiagnostics SlabPullDiagnostics;
	const double SubductionDistanceFieldStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
	Planet.ComputeSubductionDistanceField();
	SubductionFieldDiagnostics = Planet.LastSubductionDiagnostics;
	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(
			PhaseTiming.SubductionDistanceFieldMs,
			SubductionDistanceFieldStartTime);
	}
	FV6CopiedFrontierTectonicMaintenanceStats TectonicMaintenanceStats;
	if (bEnableTectonicMaintenance)
	{
		TectonicMaintenanceStats = ApplyCopiedFrontierConvergentMaintenance(
			Planet,
			PreSolvePlateIds,
			PreSolveContinentalFlags,
			PreSolveContinentalWeights,
			PreSolveElevations,
			PreSolveThicknesses,
			CurrentSolveActiveZoneFlags,
			CurrentSolveActiveZoneCauseValues,
			Interval,
			bUseLinearConvergentMaintenanceSpeedFactorForTest,
			bUseLinearConvergentMaintenanceInfluenceForTest);
		const double PlateScoresStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		Planet.ComputePlateScores();
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMs(PhaseTiming.PlateScoresMs, PlateScoresStartTime);
		}
	}
	const double SlabPullStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
	Planet.ComputeSlabPullCorrections();
	SlabPullDiagnostics = Planet.LastSubductionDiagnostics;
	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(PhaseTiming.SlabPullMs, SlabPullStartTime);
	}
	const double TerraneDetectionStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
	Planet.DetectTerranes(PreviousTerranes);
	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(PhaseTiming.TerraneDetectionMs, TerraneDetectionStartTime);
	}

	const double ComponentAuditStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
	TArray<FV6AssignmentComponent> FinalComponents;
	BuildAssignmentComponents(Planet, NewPlateIds, FinalComponents);
	TMap<int32, int32> LargestComponentIndexByPlate;
	TMap<int32, int32> LargestComponentSizeByPlate;
	for (int32 ComponentIndex = 0; ComponentIndex < FinalComponents.Num(); ++ComponentIndex)
	{
		const FV6AssignmentComponent& Component = FinalComponents[ComponentIndex];
		const int32 ComponentSize = Component.Samples.Num();
		int32* ExistingIndexPtr = LargestComponentIndexByPlate.Find(Component.PlateId);
		int32* ExistingSizePtr = LargestComponentSizeByPlate.Find(Component.PlateId);
		if (ExistingIndexPtr == nullptr || ExistingSizePtr == nullptr ||
			ComponentSize > *ExistingSizePtr ||
			(ComponentSize == *ExistingSizePtr &&
				FinalComponents[*ExistingIndexPtr].MinSampleIndex > Component.MinSampleIndex))
		{
			LargestComponentIndexByPlate.Add(Component.PlateId, ComponentIndex);
			LargestComponentSizeByPlate.Add(Component.PlateId, ComponentSize);
		}
	}

	for (int32 ComponentIndex = 0; ComponentIndex < FinalComponents.Num(); ++ComponentIndex)
	{
		const FV6AssignmentComponent& Component = FinalComponents[ComponentIndex];
		const int32* LargestComponentIndexPtr = LargestComponentIndexByPlate.Find(Component.PlateId);
		if (LargestComponentIndexPtr != nullptr && *LargestComponentIndexPtr == ComponentIndex)
		{
			continue;
		}

		++CopiedFrontierAttribution.FragmentedComponentCount;
		IncrementFragmentSizeBucket(CopiedFrontierAttribution.FragmentSizeBuckets, Component.Samples.Num());

		int32 OceanicCreatedCount = 0;
		int32 CopiedFrontierHitSampleCount = 0;
		int32 MultiHitWinnerSampleCount = 0;
		int32 CopiedFrontierHitCwMismatchSampleCount = 0;
		bool bAdjacentToDestructiveExclusionGap = false;
		for (const int32 SampleIndex : Component.Samples)
		{
			if (!LastResolvedSamples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
			const bool bCopiedFrontierHit =
				Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle &&
				Resolved.TransferDebug.bUsedCopiedFrontierTriangleTransfer;
			OceanicCreatedCount += Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::OceanicCreation ? 1 : 0;
			CopiedFrontierHitSampleCount += bCopiedFrontierHit ? 1 : 0;
			MultiHitWinnerSampleCount += Resolved.ExactCandidateCount > 1 ? 1 : 0;
			CopiedFrontierHitCwMismatchSampleCount +=
				(bCopiedFrontierHit && Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend) ? 1 : 0;

			if (!bAdjacentToDestructiveExclusionGap)
			{
				bAdjacentToDestructiveExclusionGap =
					DestructiveExclusionGapMask.IsValidIndex(SampleIndex) &&
					DestructiveExclusionGapMask[SampleIndex] != 0;
			}
			if (!bAdjacentToDestructiveExclusionGap && Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (DestructiveExclusionGapMask.IsValidIndex(NeighborIndex) &&
						DestructiveExclusionGapMask[NeighborIndex] != 0)
					{
						bAdjacentToDestructiveExclusionGap = true;
						break;
					}
				}
			}
		}

		CopiedFrontierAttribution.FragmentedComponentsTouchingOceanicCreationCount += OceanicCreatedCount > 0 ? 1 : 0;
		CopiedFrontierAttribution.FragmentedComponentsDominatedByOceanicCreationCount +=
			(OceanicCreatedCount * 2 > Component.Samples.Num()) ? 1 : 0;
		CopiedFrontierAttribution.FragmentedComponentsTouchingCopiedFrontierHitCount +=
			CopiedFrontierHitSampleCount > 0 ? 1 : 0;
		CopiedFrontierAttribution.FragmentedComponentsDominatedByCopiedFrontierHitCount +=
			(CopiedFrontierHitSampleCount * 2 > Component.Samples.Num()) ? 1 : 0;
		CopiedFrontierAttribution.FragmentedComponentsTouchingMultiHitWinnerCount +=
			MultiHitWinnerSampleCount > 0 ? 1 : 0;
		CopiedFrontierAttribution.FragmentedComponentsStronglyAssociatedWithMultiHitWinnerCount +=
			(MultiHitWinnerSampleCount * 2 >= Component.Samples.Num()) ? 1 : 0;
		CopiedFrontierAttribution.FragmentedComponentsMajorityCopiedFrontierHitWithCwMismatchCount +=
			(CopiedFrontierHitCwMismatchSampleCount * 2 > Component.Samples.Num()) ? 1 : 0;
		CopiedFrontierAttribution.FragmentedComponentsAdjacentToDestructiveExclusionGapCount +=
			bAdjacentToDestructiveExclusionGap ? 1 : 0;
	}
	if (bRecordPhaseTiming)
	{
		AccumulatePhaseTimingMs(PhaseTiming.ComponentAuditMs, ComponentAuditStartTime);
	}

	if (bBuildDetailedCopiedFrontierAttribution)
	{
		RepresentativeMissCandidates.Sort([](
			const FTectonicPlanetV6CopiedFrontierRepresentativeMiss& Left,
			const FTectonicPlanetV6CopiedFrontierRepresentativeMiss& Right)
		{
			if (Left.bAdjacentToCopiedFrontierGeometry != Right.bAdjacentToCopiedFrontierGeometry)
			{
				return Left.bAdjacentToCopiedFrontierGeometry;
			}
			const bool bLeftHasDistance = Left.NearestCopiedFrontierTriangleDistance >= 0.0;
			const bool bRightHasDistance = Right.NearestCopiedFrontierTriangleDistance >= 0.0;
			if (bLeftHasDistance != bRightHasDistance)
			{
				return bLeftHasDistance;
			}
			if (bLeftHasDistance &&
				!FMath::IsNearlyEqual(
					Left.NearestCopiedFrontierTriangleDistance,
					Right.NearestCopiedFrontierTriangleDistance,
					TriangleEpsilon))
			{
				return Left.NearestCopiedFrontierTriangleDistance < Right.NearestCopiedFrontierTriangleDistance;
			}
			if (Left.bNearMultiHitRegion != Right.bNearMultiHitRegion)
			{
				return Left.bNearMultiHitRegion;
			}
			return Left.SampleIndex < Right.SampleIndex;
		});

		const int32 RepresentativeMissLimit = FMath::Min(RepresentativeMissCandidates.Num(), 8);
		for (int32 MissIndex = 0; MissIndex < RepresentativeMissLimit; ++MissIndex)
		{
			CopiedFrontierAttribution.RepresentativeMisses.Add(RepresentativeMissCandidates[MissIndex]);
		}

		AppendSortedPlateCounts(
			SingleHitLossCountsByPreviousPlate,
			CopiedFrontierAttribution.SingleHitLossCountsByPreviousPlate);
		AppendSortedPlateCounts(
			SingleHitLossCountsByFinalPlate,
			CopiedFrontierAttribution.SingleHitLossCountsByFinalPlate);
		AppendTopPatternCounts(
			SingleHitLossPatternCounts,
			CopiedFrontierAttribution.TopSingleHitLossPatterns,
			5);
		AppendTopPatternCounts(
			MultiHitPatternCounts,
			CopiedFrontierAttribution.TopMultiHitPatterns,
			3);

		RepresentativeSingleHitLossCandidates.Sort([](
			const FTectonicPlanetV6CopiedFrontierRepresentativeHitLoss& Left,
			const FTectonicPlanetV6CopiedFrontierRepresentativeHitLoss& Right)
		{
			const bool bLeftCrossPlateWithPrevAlt =
				!Left.bSamePlateWinner && Left.PreviousPlateNoCapHitCount > 0;
			const bool bRightCrossPlateWithPrevAlt =
				!Right.bSamePlateWinner && Right.PreviousPlateNoCapHitCount > 0;
			if (bLeftCrossPlateWithPrevAlt != bRightCrossPlateWithPrevAlt)
			{
				return bLeftCrossPlateWithPrevAlt;
			}
			if (Left.bSamePlateWinner != Right.bSamePlateWinner)
			{
				return !Left.bSamePlateWinner;
			}
			if (Left.bNearFrontier != Right.bNearFrontier)
			{
				return Left.bNearFrontier;
			}
			const bool bLeftHasDistance = Left.NearestCopiedFrontierTriangleDistance >= 0.0;
			const bool bRightHasDistance = Right.NearestCopiedFrontierTriangleDistance >= 0.0;
			if (bLeftHasDistance != bRightHasDistance)
			{
				return bLeftHasDistance;
			}
			if (bLeftHasDistance &&
				!FMath::IsNearlyEqual(
					Left.NearestCopiedFrontierTriangleDistance,
					Right.NearestCopiedFrontierTriangleDistance,
					TriangleEpsilon))
			{
				return Left.NearestCopiedFrontierTriangleDistance < Right.NearestCopiedFrontierTriangleDistance;
			}
			return Left.SampleIndex < Right.SampleIndex;
		});

		const int32 RepresentativeSingleHitLossLimit =
			FMath::Min(RepresentativeSingleHitLossCandidates.Num(), 8);
		for (int32 LossIndex = 0; LossIndex < RepresentativeSingleHitLossLimit; ++LossIndex)
		{
			CopiedFrontierAttribution.RepresentativeSingleHitLosses.Add(
				RepresentativeSingleHitLossCandidates[LossIndex]);
		}
	}

		CopiedFrontierAttribution.TectonicMaintenanceAppliedCount = TectonicMaintenanceStats.AppliedCount;
		CopiedFrontierAttribution.TectonicMaintenanceContinentalRecoveredCount =
			TectonicMaintenanceStats.ContinentalRecoveredCount;
		CopiedFrontierAttribution.TectonicMaintenanceContinentalGainCount =
			TectonicMaintenanceStats.ContinentalGainCount;
		CopiedFrontierAttribution.TectonicMaintenanceSamePlateRecoveredCount =
			TectonicMaintenanceStats.SamePlateRecoveredCount;
		CopiedFrontierAttribution.TectonicMaintenanceCrossPlateRecoveredCount =
			TectonicMaintenanceStats.CrossPlateRecoveredCount;
		CopiedFrontierAttribution.TectonicMaintenanceAndeanTaggedCount =
			TectonicMaintenanceStats.AndeanTaggedCount;
		CopiedFrontierAttribution.TectonicMaintenanceElevationBoostCount =
			TectonicMaintenanceStats.ElevationBoostCount;
		CopiedFrontierAttribution.TectonicMaintenanceThicknessBoostCount =
			TectonicMaintenanceStats.ThicknessBoostCount;
		CopiedFrontierAttribution.ActiveBandTriangleFieldContinuityClampCount =
			ActiveBandTriangleFieldContinuityClampCount.Load();
		CopiedFrontierAttribution.ActiveBandSyntheticFieldPreserveCount =
			ActiveBandSyntheticFieldPreserveCount.Load();
		CopiedFrontierAttribution.ActiveBandOceanicFieldPreserveCount =
			ActiveBandOceanicFieldPreserveCount.Load();
		CopiedFrontierAttribution.ActiveBandPreviousOwnerCompatibleRecoveryCount =
			ActiveBandPreviousOwnerCompatibleRecoveryCount.Load();
		CopiedFrontierAttribution.ActiveBandSyntheticLoopBreakCount =
			ActiveBandSyntheticLoopBreakCount.Load();
		CopiedFrontierAttribution.ActiveBandSyntheticSingleSourceRecoveryCount =
			ActiveBandSyntheticSingleSourceRecoveryCount.Load();
		CopiedFrontierAttribution.ContinentalSamplesAfter +=
			TectonicMaintenanceStats.ContinentalRecoveredCount +
			TectonicMaintenanceStats.ContinentalGainCount;
		CopiedFrontierAttribution.ContinentalSamplesLost = FMath::Max(
			0,
			CopiedFrontierAttribution.ContinentalSamplesLost -
				TectonicMaintenanceStats.ContinentalRecoveredCount);
		CopiedFrontierAttribution.ContinentalSamplesGained +=
			TectonicMaintenanceStats.ContinentalGainCount;

		LastCopiedFrontierSolveAttribution = MoveTemp(CopiedFrontierAttribution);

		if (bApplyDestructiveFilter && bEnableIntervalDestructivePropagation)
		{
			DestructiveTrackingStats.LifecycleStats.EntrySeedConvergentEdgeSurvivedToRemeshCount =
				CountTrackedSeedOriginTriangles(
					CopiedFrontierTrackedDestructiveKinds,
					CopiedFrontierTrackedDestructiveSeedOriginFlags);
			DestructiveTrackingStats.LifecycleStats.DirectionalNeighborSurvivedToRemeshCount =
				CountTrackedTopologyNeighborOriginTriangles(
					CopiedFrontierTrackedDestructiveKinds,
					CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags);
			DestructiveTrackingStats.ClearedTriangleCount = DestructiveTrackingStats.TrackedTriangleCount;
			DestructiveTrackingStats.ExpiredTriangleCount =
				CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount;
		}

		RebuildThesisCopiedFrontierMeshes();

		if (bApplyDestructiveFilter && bEnableIntervalDestructivePropagation)
		{
			FV6CopiedFrontierDestructiveTrackingUpdateStats PostRemeshSeedStats;
			SeedCopiedFrontierIntervalDestructiveTracking(
				Planet,
				ThesisCopiedFrontierMeshes,
				CopiedFrontierTrackedDestructiveKinds,
				CopiedFrontierTrackedPreferredContinuationPlateIds,
				CopiedFrontierTrackedDestructiveSourcePlateIds,
				CopiedFrontierTrackedDestructiveDistancesKm,
				CopiedFrontierTrackedDestructiveSeedOriginFlags,
				CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags,
				CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags,
				PostRemeshSeedStats);
			CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount =
				PostRemeshSeedStats.NewlySeededTriangleCount;
			CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats =
				PostRemeshSeedStats.LifecycleStats;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
		}

	++PeriodicSolveCount;
	PeriodicSolveSteps.Add(Planet.CurrentStep);

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
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		const int32 HitSearchCandidateCount = HitSearchPlateCandidateCounts[SampleIndex];
		HitSearchPlateCandidateCountTotal += HitSearchCandidateCount;
		HitSearchPlateCandidateCountMax =
			FMath::Max(HitSearchPlateCandidateCountMax, HitSearchCandidateCount);
		HitSearchPrunedSampleCount += HitSearchPrunedSampleFlags[SampleIndex] != 0 ? 1 : 0;

		const int32 RecoveryCandidateCount = RecoveryCandidatePlateCandidateCounts[SampleIndex];
		RecoveryCandidatePlateCandidateCountTotal += RecoveryCandidateCount;
		RecoveryCandidatePlateCandidateCountMax =
			FMath::Max(RecoveryCandidatePlateCandidateCountMax, RecoveryCandidateCount);
		RecoveryCandidateGatherSampleCount +=
			RecoveryCandidateGatherSampleFlags[SampleIndex] != 0 ? 1 : 0;
		RecoveryCandidatePrunedSampleCount +=
			RecoveryCandidatePrunedSampleFlags[SampleIndex] != 0 ? 1 : 0;

		const int32 RecoveryMissCandidateCount = RecoveryMissPlateCandidateCounts[SampleIndex];
		RecoveryMissPlateCandidateCountTotal += RecoveryMissCandidateCount;
		RecoveryMissPlateCandidateCountMax =
			FMath::Max(RecoveryMissPlateCandidateCountMax, RecoveryMissCandidateCount);
		RecoveryMissSampleCount += RecoveryMissSampleFlags[SampleIndex] != 0 ? 1 : 0;
	}

	LastSolveStats = FTectonicPlanetV6PeriodicSolveStats{};
	LastSolveStats.Trigger = Trigger;
	LastSolveStats.SolveMode = PeriodicSolveMode;
	LastSolveStats.Step = Planet.CurrentStep;
	LastSolveStats.Interval = Interval;
	LastSolveStats.SolveIndex = PeriodicSolveCount;
	LastSolveStats.PlateCount = Planet.Plates.Num();
	LastSolveStats.SolveMilliseconds = (FPlatformTime::Seconds() - SolveStartTime) * 1000.0;
	LastSolveStats.HitCount = HitCount.Load();
	LastSolveStats.MissCount = MissCount.Load();
	LastSolveStats.MultiHitCount = MultiHitCount.Load();
	LastSolveStats.SingleCandidateWinnerCount = LastSolveStats.HitCount - LastSolveStats.MultiHitCount;
	LastSolveStats.MultiCandidateWinnerCount = LastSolveStats.MultiHitCount;
	LastSolveStats.ZeroCandidateCount = LastSolveStats.MissCount;
	LastSolveStats.GapCount = LastSolveStats.MissCount;
	LastSolveStats.OverlapCount = LastSolveStats.MultiHitCount;
	LastSolveStats.TriangleTransferCount = TransferStats.TriangleTransferCount;
	LastSolveStats.DirectHitTriangleTransferCount = DirectHitTriangleTransferCount.Load();
	LastSolveStats.SingleSourceTransferCount = TransferStats.SingleSourceTransferCount;
	LastSolveStats.OceanicCreationCount = TransferStats.OceanicCreationCount;
	LastSolveStats.DefaultTransferCount = TransferStats.DefaultTransferCount;
	LastSolveStats.TransferFallbackCount = TransferFallbackCount.Load();
	LastSolveStats.NearestMemberFallbackTransferCount = NearestMemberFallbackTransferCount.Load();
	LastSolveStats.ExplicitFallbackTransferCount = ExplicitFallbackTransferCount.Load();
	LastSolveStats.PlateLocalVertexCount = GeometryCounts.PlateLocalVertexCount;
	LastSolveStats.PlateLocalTriangleCount = GeometryCounts.PlateLocalTriangleCount;
	LastSolveStats.CopiedFrontierVertexCount = GeometryCounts.CopiedFrontierVertexCount;
	LastSolveStats.CopiedFrontierTriangleCount = GeometryCounts.CopiedFrontierTriangleCount;
	LastSolveStats.CopiedFrontierCarriedSampleCount = GeometryCounts.CopiedFrontierCarriedSampleCount;
	LastSolveStats.CopiedFrontierHitCount = CopiedFrontierHitCount.Load();
	LastSolveStats.InteriorHitCount = InteriorHitCount.Load();
	LastSolveStats.CopiedFrontierRefreshedCanonicalVertexCount =
		CopiedFrontierRefreshedCanonicalVertexCount;
	LastSolveStats.CopiedFrontierRefreshedSyntheticVertexCount =
		CopiedFrontierRefreshedSyntheticVertexCount;
	LastSolveStats.SubductionFieldComputeCount = SubductionFieldDiagnostics.SubductionFieldComputeCount;
	LastSolveStats.SlabPullComputeCount = SlabPullDiagnostics.SlabPullComputeCount;
	LastSolveStats.ConvergentEdgeBuildCount =
		SubductionFieldDiagnostics.ConvergentEdgeBuildCount +
		SlabPullDiagnostics.ConvergentEdgeBuildCount;
	LastSolveStats.ReusedConvergentEdgeSetCount =
		SubductionFieldDiagnostics.ReusedConvergentEdgeSetCount +
		SlabPullDiagnostics.ReusedConvergentEdgeSetCount;
	LastSolveStats.SubductionConvergentEdgeCount = SubductionFieldDiagnostics.ConvergentEdgeCount;
	LastSolveStats.SlabPullConvergentEdgeCount = SlabPullDiagnostics.ConvergentEdgeCount;
	LastSolveStats.SubductionSeedSampleCount = SubductionFieldDiagnostics.SeedSampleCount;
	LastSolveStats.SubductionInfluencedCount = SubductionFieldDiagnostics.InfluencedSampleCount;
	LastSolveStats.SlabPullFrontSampleCount = SlabPullDiagnostics.SlabPullTotalFrontSamples;
	LastSolveStats.CachedAdjacencyEdgeDistanceCount =
		FMath::Max(
			SubductionFieldDiagnostics.CachedAdjacencyEdgeDistanceCount,
			SlabPullDiagnostics.CachedAdjacencyEdgeDistanceCount);
	LastSolveStats.CachedAdjacencyEdgeLookupCount = SubductionFieldDiagnostics.CachedAdjacencyEdgeLookupCount;
	LastSolveStats.SubductionQueuePushCount = SubductionFieldDiagnostics.SubductionQueuePushCount;
	LastSolveStats.SubductionQueuePopCount = SubductionFieldDiagnostics.SubductionQueuePopCount;
	LastSolveStats.SubductionRelaxationCount = SubductionFieldDiagnostics.SubductionRelaxationCount;
	LastSolveStats.SubductionConvergentEdgeBuildMs =
		SubductionFieldDiagnostics.SubductionConvergentEdgeBuildMs;
	LastSolveStats.SubductionSeedInitializationMs =
		SubductionFieldDiagnostics.SubductionSeedInitializationMs;
	LastSolveStats.SubductionPropagationMs = SubductionFieldDiagnostics.SubductionPropagationMs;
	LastSolveStats.SubductionFinalizeMs = SubductionFieldDiagnostics.SubductionFinalizeMs;
	LastSolveStats.CopiedFrontierUnfilteredMeshPrepareMs =
		CopiedFrontierUnfilteredMeshPrepareMs;
	LastSolveStats.CopiedFrontierFilteredMeshBuildMs =
		CopiedFrontierFilteredMeshBuildMs;
	LastSolveStats.SlabPullConvergentEdgeBuildMs =
		SlabPullDiagnostics.SlabPullConvergentEdgeBuildMs;
	LastSolveStats.SlabPullFrontierBuildMs = SlabPullDiagnostics.SlabPullFrontierBuildMs;
	LastSolveStats.SlabPullApplyMs = SlabPullDiagnostics.SlabPullApplyMs;
	LastSolveStats.HitSearchPlateCandidateCountTotal = HitSearchPlateCandidateCountTotal;
	LastSolveStats.HitSearchPlateCandidateCountMax = HitSearchPlateCandidateCountMax;
	LastSolveStats.HitSearchPrunedSampleCount = HitSearchPrunedSampleCount;
	LastSolveStats.RecoveryCandidatePlateCandidateCountTotal = RecoveryCandidatePlateCandidateCountTotal;
	LastSolveStats.RecoveryCandidatePlateCandidateCountMax = RecoveryCandidatePlateCandidateCountMax;
	LastSolveStats.RecoveryCandidateGatherSampleCount = RecoveryCandidateGatherSampleCount;
	LastSolveStats.RecoveryCandidatePrunedSampleCount = RecoveryCandidatePrunedSampleCount;
	LastSolveStats.RecoveryMissPlateCandidateCountTotal = RecoveryMissPlateCandidateCountTotal;
	LastSolveStats.RecoveryMissPlateCandidateCountMax = RecoveryMissPlateCandidateCountMax;
	LastSolveStats.RecoveryMissSampleCount = RecoveryMissSampleCount;
	LastSolveStats.DestructiveTriangleGeometryExcludedCount = DestructiveFilterState.GeometryExcludedLocalTriangleCount;
	LastSolveStats.DestructiveTriangleRejectedCount = DestructiveTriangleRejectedCount.Load();
	LastSolveStats.TrackedDestructiveTriangleCount = DestructiveTrackingStats.TrackedTriangleCount;
	LastSolveStats.TrackedSubductionTriangleCount = DestructiveTrackingStats.TrackedSubductionTriangleCount;
	LastSolveStats.TrackedCollisionTriangleCount = DestructiveTrackingStats.TrackedCollisionTriangleCount;
	LastSolveStats.TrackedDestructiveTriangleNewlySeededCount = DestructiveTrackingStats.NewlySeededTriangleCount;
	LastSolveStats.TrackedDestructiveTrianglePropagatedCount = DestructiveTrackingStats.PropagatedTriangleCount;
	LastSolveStats.TrackedDestructiveTriangleExpiredCount = DestructiveTrackingStats.ExpiredTriangleCount;
	LastSolveStats.TrackedDestructiveTriangleClearedCount = DestructiveTrackingStats.ClearedTriangleCount;
	LastSolveStats.TrackingLifecycleStats = DestructiveTrackingStats.LifecycleStats;
	LastSolveStats.MissTrueDivergenceGapCount = LastCopiedFrontierSolveAttribution.MissTrueDivergenceGapCount;
	LastSolveStats.MissDestructiveExclusionGapCount =
		LastCopiedFrontierSolveAttribution.MissDestructiveExclusionGapCount;
	LastSolveStats.MissAmbiguousGapCount = LastCopiedFrontierSolveAttribution.MissAmbiguousGapCount;
	LastSolveStats.SamplesWithPartialDestructiveCandidateRemovalCount =
		LastCopiedFrontierSolveAttribution.SamplesWithPartialDestructiveCandidateRemovalCount;
	LastSolveStats.FragmentedComponentsAdjacentToDestructiveExclusionGapCount =
		LastCopiedFrontierSolveAttribution.FragmentedComponentsAdjacentToDestructiveExclusionGapCount;
	LastSolveStats.CategoricalMajorityTransferCount = TransferStats.CategoricalMajorityTransferCount;
	LastSolveStats.CategoricalDominantTransferCount = TransferStats.CategoricalDominantTransferCount;
	LastSolveStats.DirectionalFallbackCount = TransferStats.DirectionalFallbackCount;
	LastSolveStats.ContinentalWeightThresholdMismatchCount = TransferStats.ContinentalWeightThresholdMismatchCount;
	LastSolveStats.bDestructiveTriangleExclusionApplied =
		bApplyDestructiveFilter &&
		(LastSolveStats.DestructiveTriangleGeometryExcludedCount > 0 ||
			LastSolveStats.DestructiveTriangleRejectedCount > 0);
	LastSolveStats.TriangleTransferCountsByResolution = TransferStats.TriangleTransferCountsByResolution;
	LastSolveStats.SingleSourceTransferCountsByResolution = TransferStats.SingleSourceTransferCountsByResolution;
	LastSolveStats.ContinentalWeightThresholdMismatchCountsByResolution =
		TransferStats.ContinentalWeightThresholdMismatchCountsByResolution;
	LastSolveStats.BoundarySampleCount = CountBoundarySamples(Planet);
	LastSolveStats.ContinentalAreaFraction = ComputeContinentalAreaFraction(Planet);
	LastSolveStats.QuietInteriorContinentalRetentionCount = QuietInteriorContinentalRetentionCount.Load();
	LastSolveStats.QuietInteriorContinentalRetentionTriangleCount = QuietInteriorContinentalRetentionTriangleCount.Load();
	LastSolveStats.QuietInteriorContinentalRetentionSingleSourceCount = QuietInteriorContinentalRetentionSingleSourceCount.Load();
	LastSolveStats.ContinentalBreadthPreservationCount = ContinentalBreadthPreservationCount.Load();
	LastSolveStats.ContinentalBreadthPreservationStrongInteriorCount =
		ContinentalBreadthPreservationStrongInteriorCount.Load();
	LastSolveStats.ContinentalBreadthPreservationModerateInteriorCount =
		ContinentalBreadthPreservationModerateInteriorCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideCount = PaperSurrogateOwnershipOverrideCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideSubaerialCount = PaperSurrogateOwnershipOverrideSubaerialCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideSubmergedCount = PaperSurrogateOwnershipOverrideSubmergedCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideTriangleCount = PaperSurrogateOwnershipOverrideTriangleCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideSingleSourceCount = PaperSurrogateOwnershipOverrideSingleSourceCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideStrongInteriorCount =
		PaperSurrogateOwnershipOverrideStrongInteriorCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideModerateInteriorCount =
		PaperSurrogateOwnershipOverrideModerateInteriorCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideLowElevationBandCount =
		PaperSurrogateOwnershipOverrideLowElevationBandCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideModerateElevationBandCount =
		PaperSurrogateOwnershipOverrideModerateElevationBandCount.Load();
	LastSolveStats.PaperSurrogateOwnershipOverrideHighElevationBandCount =
		PaperSurrogateOwnershipOverrideHighElevationBandCount.Load();
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount =
		PaperSurrogateQuietInteriorDirectHitEligibleCount.Load();
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveElevationSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitPreSolveElevationScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferElevationSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitPostTransferElevationScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitFinalElevationSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitFinalElevationScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveThicknessSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitPreSolveThicknessScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferThicknessSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitPostTransferThicknessScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.PaperSurrogateQuietInteriorDirectHitFinalThicknessSum =
		static_cast<double>(PaperSurrogateQuietInteriorDirectHitFinalThicknessScaledSum.Load()) /
		PaperSurrogateDiagnosticScale;
	LastSolveStats.MaxComponentsBeforeCoherence = MaxComponentsBeforeRepartition;
	LastSolveStats.MaxComponentsPerPlate = ComputeMaxComponentsPerPlate(Planet);
	if (bEnableV9CollisionShadowForTest && bEnableV9Phase1Authority)
	{
		const double CollisionShadowStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		UpdateV9CollisionShadowState(
			Planet,
			PreSolvePlateIds,
			PreSolveContinentalWeights,
			SampleToAdjacentTriangles,
			LastResolvedSamples,
			V9CollisionShadowPairRecurrenceByKey,
			CurrentSolveCollisionShadowTrackedFlags,
			CurrentSolveCollisionShadowQualifiedFlags,
			CurrentSolveCollisionShadowPersistenceMask,
			CurrentSolveCollisionShadowDiagnostic);
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMs(PhaseTiming.CollisionShadowMs, CollisionShadowStartTime);
		}
	}
	else
	{
		CurrentSolveCollisionShadowTrackedFlags.Init(0, Planet.Samples.Num());
		CurrentSolveCollisionShadowQualifiedFlags.Init(0, Planet.Samples.Num());
		CurrentSolveCollisionShadowPersistenceMask.Init(0, Planet.Samples.Num());
		CurrentSolveCollisionShadowDiagnostic = FTectonicPlanetV6CollisionShadowDiagnostic{};
		V9CollisionShadowPairRecurrenceByKey.Reset();
	}
	if (bEnableV9CollisionExecutionForTest &&
		bEnableV9CollisionShadowForTest &&
		bEnableV9Phase1Authority)
	{
		const double CollisionExecutionStartTime = bRecordPhaseTiming ? GetPhaseTimingSeconds() : 0.0;
		if (bEnableV9ThesisShapedCollisionExecutionForTest)
		{
			ExecuteV9ThesisShapedCollisionEventRedesign(
				Planet,
				LastResolvedSamples,
				CurrentSolveCollisionShadowDiagnostic,
				bEnableV9ThesisShapedCollisionRidgeSurgeForTest,
				V9CollisionExecutionLastSolveIndexByKey,
				PeriodicSolveCount + 1,
				CurrentSolveCollisionExecutionMask,
				CurrentSolveCollisionTransferMask,
				CurrentSolveThesisCollisionTerraneComponentMask,
				CumulativeCollisionExecutionMask,
				CumulativeCollisionTransferMask,
				CumulativeCollisionElevationDeltaMaskKm,
				CumulativeCollisionContinentalGainMask,
				CurrentSolveCollisionExecutionDiagnostic,
				V9CollisionExecutionCumulativeCount,
				V9CollisionExecutionCumulativeAffectedSampleVisits,
				V9CollisionExecutionCumulativeContinentalGainCount,
				V9CollisionExecutionCumulativeOwnershipChangeCount,
				V9CollisionExecutionCumulativeTransferredSampleVisits,
				V9CollisionExecutionCumulativeTransferredContinentalSampleCount,
				V9CollisionExecutionCumulativeElevationDeltaKm,
				V9CollisionExecutionCumulativeMaxElevationDeltaKm);
		}
		else
		{
			ExecuteV9CollisionShadowEvent(
				Planet,
				LastResolvedSamples,
				CurrentSolveCollisionShadowDiagnostic,
				V9CollisionExecutionLastSolveIndexByKey,
				PeriodicSolveCount + 1,
				CurrentSolveCollisionExecutionMask,
				CurrentSolveCollisionTransferMask,
				CumulativeCollisionExecutionMask,
				CumulativeCollisionTransferMask,
				CumulativeCollisionElevationDeltaMaskKm,
				CumulativeCollisionContinentalGainMask,
				CurrentSolveCollisionExecutionDiagnostic,
				V9CollisionExecutionCumulativeCount,
				V9CollisionExecutionCumulativeAffectedSampleVisits,
				V9CollisionExecutionCumulativeContinentalGainCount,
				V9CollisionExecutionCumulativeOwnershipChangeCount,
				V9CollisionExecutionCumulativeTransferredSampleVisits,
				V9CollisionExecutionCumulativeTransferredContinentalSampleCount,
				V9CollisionExecutionCumulativeElevationDeltaKm,
				V9CollisionExecutionCumulativeMaxElevationDeltaKm,
				bEnableV9CollisionExecutionEnhancedConsequencesForTest,
				bEnableV9CollisionExecutionStructuralTransferForTest,
				bEnableV9CollisionExecutionRefinedStructuralTransferForTest);
		}
		if (bRecordPhaseTiming)
		{
			AccumulatePhaseTimingMs(
				PhaseTiming.CollisionExecutionMs,
				CollisionExecutionStartTime);
		}
	}
	else
	{
		CurrentSolveCollisionExecutionMask.Init(0, Planet.Samples.Num());
		CurrentSolveCollisionTransferMask.Init(0, Planet.Samples.Num());
		CurrentSolveCollisionExecutionDiagnostic = FTectonicPlanetV6CollisionExecutionDiagnostic{};
		CurrentSolveCollisionExecutionDiagnostic.CumulativeExecutedCollisionCount =
			V9CollisionExecutionCumulativeCount;
		CurrentSolveCollisionExecutionDiagnostic.CumulativeCollisionAffectedSampleVisits =
			V9CollisionExecutionCumulativeAffectedSampleVisits;
		CurrentSolveCollisionExecutionDiagnostic.CumulativeCollisionDrivenContinentalGainCount =
			V9CollisionExecutionCumulativeContinentalGainCount;
		CurrentSolveCollisionExecutionDiagnostic.CumulativeCollisionDrivenOwnershipChangeCount =
			V9CollisionExecutionCumulativeOwnershipChangeCount;
		CurrentSolveCollisionExecutionDiagnostic.CumulativeCollisionTransferredSampleVisits =
			V9CollisionExecutionCumulativeTransferredSampleVisits;
		CurrentSolveCollisionExecutionDiagnostic.CumulativeCollisionTransferredContinentalSampleCount =
			V9CollisionExecutionCumulativeTransferredContinentalSampleCount;
		CurrentSolveCollisionExecutionDiagnostic.CumulativeMeanElevationDeltaKm =
			V9CollisionExecutionCumulativeAffectedSampleVisits > 0
				? V9CollisionExecutionCumulativeElevationDeltaKm /
					static_cast<double>(V9CollisionExecutionCumulativeAffectedSampleVisits)
				: 0.0;
		CurrentSolveCollisionExecutionDiagnostic.CumulativeMaxElevationDeltaKm =
			V9CollisionExecutionCumulativeMaxElevationDeltaKm;
		if (CumulativeCollisionExecutionMask.Num() != Planet.Samples.Num())
		{
			CumulativeCollisionExecutionMask.Init(0, Planet.Samples.Num());
		}
		if (CumulativeCollisionTransferMask.Num() != Planet.Samples.Num())
		{
			CumulativeCollisionTransferMask.Init(0, Planet.Samples.Num());
		}
		if (CumulativeCollisionElevationDeltaMaskKm.Num() != Planet.Samples.Num())
		{
			CumulativeCollisionElevationDeltaMaskKm.Init(0.0f, Planet.Samples.Num());
		}
		if (CumulativeCollisionContinentalGainMask.Num() != Planet.Samples.Num())
		{
			CumulativeCollisionContinentalGainMask.Init(0.0f, Planet.Samples.Num());
		}
		for (const uint8 Value : CumulativeCollisionExecutionMask)
		{
			CurrentSolveCollisionExecutionDiagnostic.CumulativeCollisionAffectedSampleCount +=
				Value != 0 ? 1 : 0;
		}
		for (const uint8 Value : CumulativeCollisionTransferMask)
		{
			CurrentSolveCollisionExecutionDiagnostic.CumulativeCollisionTransferredSampleCount +=
				Value != 0 ? 1 : 0;
		}
		ResetV9CollisionExecutionLegacyStats(Planet);
	}
	PhaseTiming.HitSearchMs = ConvertPhaseTimingMicrosecondsToMilliseconds(HitSearchMicroseconds);
	PhaseTiming.ZeroHitRecoveryMs = ConvertPhaseTimingMicrosecondsToMilliseconds(ZeroHitRecoveryMicroseconds);
	PhaseTiming.DirectHitTransferMs = ConvertPhaseTimingMicrosecondsToMilliseconds(DirectHitTransferMicroseconds);
	PhaseTiming.FallbackTransferMs = ConvertPhaseTimingMicrosecondsToMilliseconds(FallbackTransferMicroseconds);
	PhaseTiming.QuietInteriorPreservationMs =
		ConvertPhaseTimingMicrosecondsToMilliseconds(QuietInteriorPreservationMicroseconds);
	LastSolveStats.PhaseTiming = PhaseTiming;
	LastSolveStats.SolveMilliseconds = (FPlatformTime::Seconds() - SolveStartTime) * 1000.0;
	const TCHAR* CopiedFrontierModeLabel = GetCopiedFrontierModeLabel(PeriodicSolveMode);

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[TectonicPlanetV6 %s Step=%d] solve_ms=%.3f cadence_steps=%d plate_local_vertices=%d plate_local_triangles=%d copied_frontier_vertices=%d copied_frontier_triangles=%d copied_frontier_carried=%d hits=%d copied_frontier_hits=%d interior_hits=%d misses=%d multi_hits=%d direct_hit_triangle_transfer=%d single_source_transfer=%d oceanic_creation=%d default_transfer=%d transfer_fallback=%d tectonic_maintenance_applied=%d tectonic_maintenance_recovered=%d tectonic_maintenance_gain=%d destructive_filter_applied=%d destructive_filter_geometry_excluded=%d destructive_filter_rejected=%d tracked_destructive_triangles=%d tracked_subduction_triangles=%d tracked_collision_triangles=%d tracked_newly_seeded=%d tracked_propagated=%d tracked_expired=%d tracked_cleared=%d tracked_entry_candidates=%d tracked_seed_edge_candidates=%d tracked_entry_overlap_tested=%d tracked_entry_admitted=%d tracked_seed_edge_admitted=%d tracked_seed_edge_survived_one_timestep=%d tracked_seed_edge_survived_to_remesh=%d tracked_entry_reject_no_overlap=%d tracked_entry_reject_authorization=%d tracked_entry_reject_already_tracked=%d tracked_entry_subduction=%d tracked_entry_collision=%d tracked_active_advanced=%d tracked_active_overlap_confirmed=%d tracked_active_overlap_rejected=%d tracked_neighbor_generated=%d tracked_neighbor_tested=%d tracked_neighbor_admitted=%d tracked_neighbor_reject_no_overlap=%d tracked_neighbor_reject_authorization=%d tracked_neighbor_reject_already_tracked=%d tracked_neighbor_subduction=%d tracked_neighbor_collision=%d tracked_topology_neighbor_generated=%d tracked_topology_neighbor_admitted=%d tracked_topology_neighbor_expired=%d tracked_directional_neighbor_considered=%d tracked_directional_neighbor_admitted=%d tracked_directional_neighbor_rejected_not_inward=%d tracked_directional_neighbor_survived_to_remesh=%d tracked_expired_subduction=%d tracked_expired_collision=%d miss_true_divergence_gap=%d miss_destructive_exclusion_gap=%d miss_ambiguous_gap=%d partial_destructive_candidate_removal=%d fragmented_components_adjacent_to_destructive_gap=%d cw_threshold_mismatch=%d continental_area_fraction=%.6f max_components_after=%d"),
		CopiedFrontierModeLabel,
		Planet.CurrentStep,
		LastSolveStats.SolveMilliseconds,
		LastSolveStats.Interval,
		LastSolveStats.PlateLocalVertexCount,
		LastSolveStats.PlateLocalTriangleCount,
		LastSolveStats.CopiedFrontierVertexCount,
		LastSolveStats.CopiedFrontierTriangleCount,
		LastSolveStats.CopiedFrontierCarriedSampleCount,
		LastSolveStats.HitCount,
		LastSolveStats.CopiedFrontierHitCount,
		LastSolveStats.InteriorHitCount,
		LastSolveStats.MissCount,
		LastSolveStats.MultiHitCount,
		LastSolveStats.DirectHitTriangleTransferCount,
		LastSolveStats.SingleSourceTransferCount,
		LastSolveStats.OceanicCreationCount,
		LastSolveStats.DefaultTransferCount,
		LastSolveStats.TransferFallbackCount,
		TectonicMaintenanceStats.AppliedCount,
		TectonicMaintenanceStats.ContinentalRecoveredCount,
		TectonicMaintenanceStats.ContinentalGainCount,
		LastSolveStats.bDestructiveTriangleExclusionApplied ? 1 : 0,
		LastSolveStats.DestructiveTriangleGeometryExcludedCount,
		LastSolveStats.DestructiveTriangleRejectedCount,
		LastSolveStats.TrackedDestructiveTriangleCount,
		LastSolveStats.TrackedSubductionTriangleCount,
		LastSolveStats.TrackedCollisionTriangleCount,
		LastSolveStats.TrackedDestructiveTriangleNewlySeededCount,
		LastSolveStats.TrackedDestructiveTrianglePropagatedCount,
		LastSolveStats.TrackedDestructiveTriangleExpiredCount,
		LastSolveStats.TrackedDestructiveTriangleClearedCount,
		LastSolveStats.TrackingLifecycleStats.EntryCandidateCount,
		LastSolveStats.TrackingLifecycleStats.EntrySeedConvergentEdgeCandidateCount,
		LastSolveStats.TrackingLifecycleStats.EntryOverlapTestedCount,
		LastSolveStats.TrackingLifecycleStats.EntryAdmittedCount,
		LastSolveStats.TrackingLifecycleStats.EntrySeedConvergentEdgeAdmittedCount,
		LastSolveStats.TrackingLifecycleStats.EntrySeedConvergentEdgeSurvivedOneTimestepCount,
		LastSolveStats.TrackingLifecycleStats.EntrySeedConvergentEdgeSurvivedToRemeshCount,
		LastSolveStats.TrackingLifecycleStats.EntryRejectedNoOverlapCount,
		LastSolveStats.TrackingLifecycleStats.EntryRejectedAuthorizationCount,
		LastSolveStats.TrackingLifecycleStats.EntryRejectedAlreadyTrackedCount,
		LastSolveStats.TrackingLifecycleStats.EntryAdmittedSubductionCount,
		LastSolveStats.TrackingLifecycleStats.EntryAdmittedCollisionCount,
		LastSolveStats.TrackingLifecycleStats.ActiveAdvanceCount,
		LastSolveStats.TrackingLifecycleStats.ActiveOverlapConfirmedCount,
		LastSolveStats.TrackingLifecycleStats.ActiveOverlapRejectedCount,
		LastSolveStats.TrackingLifecycleStats.NeighborCandidateGeneratedCount,
		LastSolveStats.TrackingLifecycleStats.NeighborCandidateTestedCount,
		LastSolveStats.TrackingLifecycleStats.NeighborAdmittedCount,
		LastSolveStats.TrackingLifecycleStats.NeighborRejectedNoOverlapCount,
		LastSolveStats.TrackingLifecycleStats.NeighborRejectedAuthorizationCount,
		LastSolveStats.TrackingLifecycleStats.NeighborRejectedAlreadyTrackedCount,
		LastSolveStats.TrackingLifecycleStats.NeighborAdmittedSubductionCount,
		LastSolveStats.TrackingLifecycleStats.NeighborAdmittedCollisionCount,
		LastSolveStats.TrackingLifecycleStats.TopologyNeighborCandidateGeneratedCount,
		LastSolveStats.TrackingLifecycleStats.TopologyNeighborAdmittedCount,
		LastSolveStats.TrackingLifecycleStats.TopologyNeighborExpiredBeforeRemeshCount,
		LastSolveStats.TrackingLifecycleStats.DirectionalNeighborCandidateConsideredCount,
		LastSolveStats.TrackingLifecycleStats.DirectionalNeighborAdmittedCount,
		LastSolveStats.TrackingLifecycleStats.DirectionalNeighborRejectedNotInwardCount,
		LastSolveStats.TrackingLifecycleStats.DirectionalNeighborSurvivedToRemeshCount,
		LastSolveStats.TrackingLifecycleStats.ExpiredSubductionCount,
		LastSolveStats.TrackingLifecycleStats.ExpiredCollisionCount,
		LastSolveStats.MissTrueDivergenceGapCount,
		LastSolveStats.MissDestructiveExclusionGapCount,
		LastSolveStats.MissAmbiguousGapCount,
		LastSolveStats.SamplesWithPartialDestructiveCandidateRemovalCount,
		LastSolveStats.FragmentedComponentsAdjacentToDestructiveExclusionGapCount,
		LastSolveStats.ContinentalWeightThresholdMismatchCount,
		LastSolveStats.ContinentalAreaFraction,
		LastSolveStats.MaxComponentsPerPlate);
	if (bRecordPhaseTiming)
	{
		const FTectonicPlanetV6PhaseTiming& SolvePhaseTiming = LastSolveStats.PhaseTiming;
		UE_LOG(
			LogTemp,
			Log,
			TEXT("[V6PhaseTiming %s Step=%d] total_ms=%.3f pre_solve_ms=%.3f sample_adjacency_ms=%.3f active_zone_ms=%.3f copied_frontier_mesh_ms=%.3f query_geometry_ms=%.3f frontier_point_sets_ms=%.3f resolve_transfer_loop_ms=%.3f hit_search_ms=%.3f zero_hit_recovery_ms=%.3f direct_hit_transfer_ms=%.3f fallback_transfer_ms=%.3f quiet_interior_preserve_ms=%.3f attribution_ms=%.3f repartition_ms=%.3f subduction_field_ms=%.3f plate_scores_ms=%.3f slab_pull_ms=%.3f terrane_ms=%.3f component_audit_ms=%.3f rebuild_meshes_ms=%.3f collision_shadow_ms=%.3f collision_exec_ms=%.3f"),
			CopiedFrontierModeLabel,
			Planet.CurrentStep,
			LastSolveStats.SolveMilliseconds,
			SolvePhaseTiming.PreSolveCaptureMs,
			SolvePhaseTiming.SampleAdjacencyBuildMs,
			SolvePhaseTiming.ActiveZoneMaskMs,
			SolvePhaseTiming.CopiedFrontierMeshBuildMs,
			SolvePhaseTiming.QueryGeometryBuildMs,
			SolvePhaseTiming.FrontierPointSetBuildMs,
			SolvePhaseTiming.ResolveTransferLoopMs,
			SolvePhaseTiming.HitSearchMs,
			SolvePhaseTiming.ZeroHitRecoveryMs,
			SolvePhaseTiming.DirectHitTransferMs,
			SolvePhaseTiming.FallbackTransferMs,
			SolvePhaseTiming.QuietInteriorPreservationMs,
			SolvePhaseTiming.AttributionMs,
			SolvePhaseTiming.RepartitionMembershipMs,
			SolvePhaseTiming.SubductionDistanceFieldMs,
			SolvePhaseTiming.PlateScoresMs,
			SolvePhaseTiming.SlabPullMs,
			SolvePhaseTiming.TerraneDetectionMs,
			SolvePhaseTiming.ComponentAuditMs,
			SolvePhaseTiming.RebuildCopiedFrontierMeshesMs,
			SolvePhaseTiming.CollisionShadowMs,
			SolvePhaseTiming.CollisionExecutionMs);
	}

	// --- Update persistent diagnostic state ---
	// Miss lineage counters
	if (MissLineageCounts.Num() != Planet.Samples.Num())
	{
		MissLineageCounts.Init(0, Planet.Samples.Num());
	}
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		if (LastResolvedSamples.IsValidIndex(SampleIndex))
		{
			const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
			const bool bMiss =
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion ||
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous;
			if (bMiss)
			{
				MissLineageCounts[SampleIndex] = FMath::Min(
					static_cast<int32>(MissLineageCounts[SampleIndex]) + 1, 255);
			}
			else
			{
				MissLineageCounts[SampleIndex] = 0;
			}
		}
	}

	// Persist per-sample synthetic state for next interval's stability diagnostics.
	PreviousSolveSyntheticFlags.Init(0, Planet.Samples.Num());
	PreviousSolveRetainedSyntheticCoverageFlags.Init(0, Planet.Samples.Num());
	PreviousSolveTransferSourceKindValues.Init(
		static_cast<uint8>(ETectonicPlanetV6TransferSourceKind::None),
		Planet.Samples.Num());
	PreviousSolveResolutionKindValues.Init(
		static_cast<uint8>(ETectonicPlanetV6ResolutionKind::None),
		Planet.Samples.Num());
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		if (!LastResolvedSamples.IsValidIndex(SampleIndex))
		{
			continue;
		}

		const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		PreviousSolveSyntheticFlags[SampleIndex] =
			(IsSyntheticTransferSourceKind(Resolved.TransferDebug.SourceKind) ||
				Resolved.bRetainedSyntheticCoverage) ? 1 : 0;
		PreviousSolveRetainedSyntheticCoverageFlags[SampleIndex] =
			Resolved.bRetainedSyntheticCoverage ? 1 : 0;
		PreviousSolveTransferSourceKindValues[SampleIndex] =
			static_cast<uint8>(Resolved.TransferDebug.SourceKind);
		PreviousSolveResolutionKindValues[SampleIndex] =
			static_cast<uint8>(Resolved.ResolutionKind);
	}

	// Front retreat: snapshot current subduction state for next interval's delta
	{
		double SubductionSum = 0.0;
		int32 SubductionCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			if (Sample.SubductionDistanceKm >= 0.0f)
			{
				SubductionSum += static_cast<double>(Sample.SubductionDistanceKm);
				++SubductionCount;
			}
		}
		PreviousIntervalMeanSubductionDistanceKm =
			SubductionCount > 0 ? SubductionSum / static_cast<double>(SubductionCount) : -1.0;
		PreviousIntervalSubductionSampleCount = SubductionCount;
		PreviousIntervalTrackedTriangleCount = LastSolveStats.TrackedDestructiveTriangleCount;
	}
}

void FTectonicPlanetV6::PerformThesisRemeshSpikeSolve(const ETectonicPlanetV6SolveTrigger Trigger)
{
	const double SolveStartTime = FPlatformTime::Seconds();
	const TArray<FTerrane> PreviousTerranes = Planet.Terranes;
	const int32 Interval = ComputePeriodicSolveInterval();

	TArray<FV6PlateQueryGeometry> QueryGeometries;
	BuildV6ThesisRemeshQueryGeometries(Planet, QueryGeometries);

	LastResolvedSamples.SetNum(Planet.Samples.Num());
	TArray<int32> NewPlateIds;
	NewPlateIds.Init(INDEX_NONE, Planet.Samples.Num());
	TArray<float> SubductionDistances;
	SubductionDistances.Init(-1.0f, Planet.Samples.Num());
	TArray<float> SubductionSpeeds;
	SubductionSpeeds.Init(0.0f, Planet.Samples.Num());

	TAtomic<int32> HitCount(0);
	TAtomic<int32> MissCount(0);
	TAtomic<int32> MultiHitCount(0);
	TAtomic<int32> DirectHitTriangleTransferCount(0);
	TAtomic<int32> TransferFallbackCount(0);

	ParallelFor(Planet.Samples.Num(), [this, &QueryGeometries, &NewPlateIds, &HitCount, &MissCount, &MultiHitCount](const int32 SampleIndex)
	{
		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		const int32 PreviousPlateId = Planet.Samples[SampleIndex].PlateId;

		TArray<FV6ThesisRemeshRayHit> HitCandidates;
		HitCandidates.Reserve(QueryGeometries.Num());
		for (const FV6PlateQueryGeometry& QueryGeometry : QueryGeometries)
		{
			FV6ThesisRemeshRayHit Hit;
			if (TryFindThesisRemeshRayHit(QueryGeometry, QueryPoint, Hit))
			{
				HitCandidates.Add(Hit);
			}
		}

		FTectonicPlanetV6ResolvedSample Resolved;
		Resolved.PreviousPlateId = PreviousPlateId;
		Resolved.ExactCandidateCount = HitCandidates.Num();

		if (!HitCandidates.IsEmpty())
		{
			++HitCount;
			if (HitCandidates.Num() > 1)
			{
				++MultiHitCount;
			}

			const FV6ThesisRemeshRayHit* BestHit = &HitCandidates[0];
			for (int32 HitIndex = 1; HitIndex < HitCandidates.Num(); ++HitIndex)
			{
				if (IsBetterThesisRemeshHit(HitCandidates[HitIndex], *BestHit))
				{
					BestHit = &HitCandidates[HitIndex];
				}
			}

			Resolved.FinalPlateId = BestHit->PlateId;
			Resolved.PreCoherencePlateId = BestHit->PlateId;
			Resolved.SourceLocalTriangleIndex = BestHit->LocalTriangleIndex;
			Resolved.SourceTriangleIndex = BestHit->GlobalTriangleIndex;
			Resolved.SourceBarycentric = BestHit->Barycentric;
			Resolved.WinningFitScore = BestHit->FitScore;
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshHit;
		}
		else
		{
			++MissCount;

			TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
			CollectThesisRemeshMissRecoveryCandidates(QueryGeometries, QueryPoint, RecoveryCandidates);

			int32 ExplicitFallbackPlateId = INDEX_NONE;
			int32 ExplicitFallbackSampleIndex = INDEX_NONE;
			ChooseExplicitFallback(Planet, ExplicitFallbackPlateId, ExplicitFallbackSampleIndex);

			int32 PrimaryPlateId = INDEX_NONE;
			int32 SecondaryPlateId = INDEX_NONE;
			double PrimaryDistance = -1.0;
			ChooseThesisRemeshMissOwnerPlates(
				RecoveryCandidates,
				PreviousPlateId,
				ExplicitFallbackPlateId,
				PrimaryPlateId,
				SecondaryPlateId,
				PrimaryDistance);

			Resolved.FinalPlateId = PrimaryPlateId;
			Resolved.PreCoherencePlateId = PrimaryPlateId;
			Resolved.BoundaryOtherPlateId = SecondaryPlateId;
			Resolved.RecoveryDistanceRadians = PrimaryDistance;
			Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic;
		}

		LastResolvedSamples[SampleIndex] = Resolved;
		NewPlateIds[SampleIndex] = Resolved.FinalPlateId;
	});

	const int32 MaxComponentsBeforeRepartition = ComputeMaxComponentsPerPlateFromAssignments(Planet, NewPlateIds);

	ParallelFor(Planet.Samples.Num(), [this, &SubductionDistances, &SubductionSpeeds, &DirectHitTriangleTransferCount, &TransferFallbackCount](const int32 SampleIndex)
	{
		FSample& Sample = Planet.Samples[SampleIndex];
		FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		Sample.PlateId = Resolved.FinalPlateId;
		Resolved.TransferDebug = FTectonicPlanetV6TransferDebugInfo{};

		bool bTransferred = false;
		if (Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshHit &&
			Resolved.SourceTriangleIndex != INDEX_NONE &&
			Planet.TriangleIndices.IsValidIndex(Resolved.SourceTriangleIndex))
		{
			const FIntVector& Triangle = Planet.TriangleIndices[Resolved.SourceTriangleIndex];
			const FCarriedSample* V0 = FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, Triangle.X);
			const FCarriedSample* V1 = FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, Triangle.Y);
			const FCarriedSample* V2 = FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, Triangle.Z);
			if (V0 != nullptr && V1 != nullptr && V2 != nullptr)
			{
				ApplyTransferredAttributesFromTriangle(
					Sample,
					*V0,
					*V1,
					*V2,
					Resolved.SourceBarycentric,
					SubductionDistances[SampleIndex],
					SubductionSpeeds[SampleIndex],
					Resolved.TransferDebug);
				Resolved.SourceCanonicalSampleIndex = Resolved.TransferDebug.DominantSourceCanonicalSampleIndex;
				++DirectHitTriangleTransferCount;
				bTransferred = true;
			}
		}

		if (!bTransferred &&
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic)
		{
			ApplyOceanicBoundaryCreation(
				Sample,
				Planet,
				Resolved,
				SubductionDistances[SampleIndex],
				SubductionSpeeds[SampleIndex],
				Resolved.TransferDebug);
			bTransferred = true;
		}

		if (!bTransferred && Resolved.FinalPlateId != INDEX_NONE)
		{
			int32 TransferSourceCanonicalSampleIndex =
				Resolved.SourceTriangleIndex != INDEX_NONE
					? PickTransferFallbackCanonicalVertexForTriangle(
						Planet,
						Resolved.FinalPlateId,
						Resolved.SourceTriangleIndex,
						Resolved.SourceBarycentric)
					: INDEX_NONE;
			bool bUsedNearestMemberFallback = false;
			bool bUsedExplicitFallback = false;
			const FCarriedSample* Source =
				TransferSourceCanonicalSampleIndex != INDEX_NONE
					? FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex)
					: nullptr;
			if (Source == nullptr)
			{
				double NearestMemberDistanceRadians = TNumericLimits<double>::Max();
				if (FindNearestMemberSample(
						Planet,
						Resolved.FinalPlateId,
						Sample.Position,
						TransferSourceCanonicalSampleIndex,
						NearestMemberDistanceRadians))
				{
					Source =
						FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
					bUsedNearestMemberFallback = Source != nullptr;
				}
			}
			if (Source == nullptr)
			{
				ChooseExplicitFallbackForPlate(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
				if (TransferSourceCanonicalSampleIndex != INDEX_NONE)
				{
					Source =
						FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
					bUsedExplicitFallback = Source != nullptr;
				}
			}
			if (Source != nullptr)
			{
				Resolved.SourceCanonicalSampleIndex = TransferSourceCanonicalSampleIndex;
				ApplyTransferredAttributesFromSingleSource(
					Sample,
					*Source,
					SubductionDistances[SampleIndex],
					SubductionSpeeds[SampleIndex],
					Resolved.TransferDebug);
				Resolved.TransferDebug.bUsedNearestMemberFallback = bUsedNearestMemberFallback;
				Resolved.TransferDebug.bUsedExplicitFallback = bUsedExplicitFallback;
				Resolved.ResolutionKind = ETectonicPlanetV6ResolutionKind::ThesisRemeshTransferFallback;
				++TransferFallbackCount;
				bTransferred = true;
			}
		}

		if (!bTransferred)
		{
			Sample.ContinentalWeight = 0.0f;
			Sample.Elevation = -6.0f;
			Sample.Thickness = 7.0f;
			Sample.Age = 0.0f;
			Sample.OrogenyType = EOrogenyType::None;
			Sample.TerraneId = INDEX_NONE;
			Sample.RidgeDirection = FVector3d::ZeroVector;
			Sample.FoldDirection = FVector3d::ZeroVector;
			SubductionDistances[SampleIndex] = -1.0f;
			SubductionSpeeds[SampleIndex] = 0.0f;
			Resolved.TransferDebug = FTectonicPlanetV6TransferDebugInfo{};
			Resolved.TransferDebug.SourceKind = ETectonicPlanetV6TransferSourceKind::Defaulted;
			++TransferFallbackCount;
		}
	});

	FTectonicPlanetV6PeriodicSolveStats TransferStats;
	for (const FTectonicPlanetV6ResolvedSample& Resolved : LastResolvedSamples)
	{
		switch (Resolved.TransferDebug.SourceKind)
		{
		case ETectonicPlanetV6TransferSourceKind::Triangle:
			++TransferStats.TriangleTransferCount;
			IncrementTransferResolutionCounts(TransferStats.TriangleTransferCountsByResolution, Resolved);
			break;
		case ETectonicPlanetV6TransferSourceKind::SingleSource:
			++TransferStats.SingleSourceTransferCount;
			IncrementTransferResolutionCounts(TransferStats.SingleSourceTransferCountsByResolution, Resolved);
			break;
		case ETectonicPlanetV6TransferSourceKind::OceanicCreation:
			++TransferStats.OceanicCreationCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::Defaulted:
			++TransferStats.DefaultTransferCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::None:
		default:
			break;
		}

		if (UsesCategoricalMajorityTransfer(Resolved.TransferDebug))
		{
			++TransferStats.CategoricalMajorityTransferCount;
		}
		else if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle ||
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::SingleSource)
		{
			++TransferStats.CategoricalDominantTransferCount;
		}

		if (UsesDirectionalFallback(Resolved.TransferDebug))
		{
			++TransferStats.DirectionalFallbackCount;
		}

		if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
		{
			++TransferStats.ContinentalWeightThresholdMismatchCount;
			IncrementTransferResolutionCounts(
				TransferStats.ContinentalWeightThresholdMismatchCountsByResolution,
				Resolved);
		}
	}

	Planet.RepartitionMembership(NewPlateIds, &SubductionDistances, &SubductionSpeeds);
	Planet.ComputeSubductionDistanceField();
	Planet.ComputeSlabPullCorrections();
	Planet.DetectTerranes(PreviousTerranes);

	++PeriodicSolveCount;
	PeriodicSolveSteps.Add(Planet.CurrentStep);

	LastSolveStats = FTectonicPlanetV6PeriodicSolveStats{};
	LastSolveStats.Trigger = Trigger;
	LastSolveStats.SolveMode = ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike;
	LastSolveStats.Step = Planet.CurrentStep;
	LastSolveStats.Interval = Interval;
	LastSolveStats.SolveIndex = PeriodicSolveCount;
	LastSolveStats.PlateCount = Planet.Plates.Num();
	LastSolveStats.HitCount = HitCount.Load();
	LastSolveStats.MissCount = MissCount.Load();
	LastSolveStats.MultiHitCount = MultiHitCount.Load();
	LastSolveStats.SingleCandidateWinnerCount = LastSolveStats.HitCount - LastSolveStats.MultiHitCount;
	LastSolveStats.MultiCandidateWinnerCount = LastSolveStats.MultiHitCount;
	LastSolveStats.ZeroCandidateCount = LastSolveStats.MissCount;
	LastSolveStats.GapCount = LastSolveStats.MissCount;
	LastSolveStats.OverlapCount = LastSolveStats.MultiHitCount;
	LastSolveStats.TriangleTransferCount = TransferStats.TriangleTransferCount;
	LastSolveStats.DirectHitTriangleTransferCount = DirectHitTriangleTransferCount.Load();
	LastSolveStats.SingleSourceTransferCount = TransferStats.SingleSourceTransferCount;
	LastSolveStats.OceanicCreationCount = TransferStats.OceanicCreationCount;
	LastSolveStats.DefaultTransferCount = TransferStats.DefaultTransferCount;
	LastSolveStats.TransferFallbackCount = TransferFallbackCount.Load();
	LastSolveStats.DestructiveTriangleRejectedCount = 0;
	LastSolveStats.CategoricalMajorityTransferCount = TransferStats.CategoricalMajorityTransferCount;
	LastSolveStats.CategoricalDominantTransferCount = TransferStats.CategoricalDominantTransferCount;
	LastSolveStats.DirectionalFallbackCount = TransferStats.DirectionalFallbackCount;
	LastSolveStats.ContinentalWeightThresholdMismatchCount = TransferStats.ContinentalWeightThresholdMismatchCount;
	LastSolveStats.bDestructiveTriangleExclusionApplied = false;
	LastSolveStats.TriangleTransferCountsByResolution = TransferStats.TriangleTransferCountsByResolution;
	LastSolveStats.SingleSourceTransferCountsByResolution = TransferStats.SingleSourceTransferCountsByResolution;
	LastSolveStats.ContinentalWeightThresholdMismatchCountsByResolution =
		TransferStats.ContinentalWeightThresholdMismatchCountsByResolution;
	LastSolveStats.BoundarySampleCount = CountBoundarySamples(Planet);
	LastSolveStats.ContinentalAreaFraction = ComputeContinentalAreaFraction(Planet);
	LastSolveStats.MaxComponentsBeforeCoherence = MaxComponentsBeforeRepartition;
	LastSolveStats.MaxComponentsPerPlate = ComputeMaxComponentsPerPlate(Planet);

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[TectonicPlanetV6 ThesisRemesh Step=%d] solve_ms=%.3f cadence_steps=%d hits=%d misses=%d multi_hits=%d direct_hit_triangle_transfer=%d single_source_transfer=%d oceanic_creation=%d default_transfer=%d transfer_fallback=%d destructive_filter_applied=%d destructive_filter_rejected=%d cw_threshold_mismatch=%d continental_area_fraction=%.6f max_components_after=%d"),
		Planet.CurrentStep,
		(FPlatformTime::Seconds() - SolveStartTime) * 1000.0,
		LastSolveStats.Interval,
		LastSolveStats.HitCount,
		LastSolveStats.MissCount,
		LastSolveStats.MultiHitCount,
		LastSolveStats.DirectHitTriangleTransferCount,
		LastSolveStats.SingleSourceTransferCount,
		LastSolveStats.OceanicCreationCount,
		LastSolveStats.DefaultTransferCount,
		LastSolveStats.TransferFallbackCount,
		LastSolveStats.bDestructiveTriangleExclusionApplied ? 1 : 0,
		LastSolveStats.DestructiveTriangleRejectedCount,
		LastSolveStats.ContinentalWeightThresholdMismatchCount,
		LastSolveStats.ContinentalAreaFraction,
		LastSolveStats.MaxComponentsPerPlate);
}

void FTectonicPlanetV6::PerformAuthoritativePeriodicSolve(const ETectonicPlanetV6SolveTrigger Trigger)
{
	if (PeriodicSolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike)
	{
		PerformThesisPlateSubmeshSpikeSolve(Trigger);
		RebuildSubmergedContinentalFringeRelaxationMask();
		return;
	}

	if (IsCopiedFrontierLikeSolveMode(PeriodicSolveMode))
	{
		PerformThesisCopiedFrontierSpikeSolve(Trigger);
		RebuildSubmergedContinentalFringeRelaxationMask();
		return;
	}

	if (PeriodicSolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike)
	{
		PerformThesisRemeshSpikeSolve(Trigger);
		RebuildSubmergedContinentalFringeRelaxationMask();
		return;
	}

	const double SolveStartTime = FPlatformTime::Seconds();
	const TArray<FTerrane> PreviousTerranes = Planet.Terranes;
	const int32 Interval = ComputePeriodicSolveInterval();
	TArray<FV6PlateQueryGeometry> QueryGeometries;
	BuildV6QueryGeometries(Planet, QueryGeometries);

	LastResolvedSamples.SetNum(Planet.Samples.Num());
	TArray<int32> NewPlateIds;
	NewPlateIds.Init(INDEX_NONE, Planet.Samples.Num());
	TArray<float> SubductionDistances;
	SubductionDistances.Init(-1.0f, Planet.Samples.Num());
	TArray<float> SubductionSpeeds;
	SubductionSpeeds.Init(0.0f, Planet.Samples.Num());

	TAtomic<int32> SingleCandidateWinnerCount(0);
	TAtomic<int32> MultiCandidateWinnerCount(0);
	TAtomic<int32> ZeroCandidateCount(0);
	TAtomic<int32> ZeroCandidateTriangleRecoveryCount(0);
	TAtomic<int32> ZeroCandidateMemberRecoveryCount(0);
	TAtomic<int32> ExplicitFallbackCount(0);
	TAtomic<int32> GapCount(0);
	TAtomic<int32> OverlapCount(0);

	ParallelFor(Planet.Samples.Num(), [this, &QueryGeometries, &NewPlateIds, &SingleCandidateWinnerCount, &MultiCandidateWinnerCount, &ZeroCandidateCount, &ZeroCandidateTriangleRecoveryCount, &ZeroCandidateMemberRecoveryCount, &ExplicitFallbackCount, &GapCount, &OverlapCount](const int32 SampleIndex)
	{
		const FVector3d QueryPoint = Planet.Samples[SampleIndex].Position;
		const int32 PreviousPlateId = Planet.Samples[SampleIndex].PlateId;

		TArray<FTectonicPlanetV6OwnerCandidate> OwnerCandidates;
		OwnerCandidates.Reserve(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			const FPlate& Plate = Planet.Plates[PlateIndex];
			const FV6PlateQueryGeometry& QueryGeometry = QueryGeometries[PlateIndex];
			if (QueryGeometry.SoupData.LocalTriangles.IsEmpty())
			{
				continue;
			}

			if (!QueryGeometry.BoundingCap.Center.IsNearlyZero() &&
				QueryPoint.Dot(QueryGeometry.BoundingCap.Center) < QueryGeometry.BoundingCap.CosAngle)
			{
				continue;
			}

			int32 LocalTriangleId = INDEX_NONE;
			FVector3d A;
			FVector3d B;
			FVector3d C;
			if (!FindContainingTriangleInBVH(QueryGeometry.SoupBVH, QueryGeometry.SoupAdapter, QueryPoint, LocalTriangleId, A, B, C))
			{
				continue;
			}

			FTectonicPlanetV6OwnerCandidate& Candidate = OwnerCandidates.AddDefaulted_GetRef();
			Candidate.PlateId = Plate.Id;
			Candidate.TriangleIndex =
				QueryGeometry.SoupData.GlobalTriangleIndices.IsValidIndex(LocalTriangleId)
					? QueryGeometry.SoupData.GlobalTriangleIndices[LocalTriangleId]
					: INDEX_NONE;
			Candidate.Barycentric = NormalizeBarycentric(ComputePlanarBarycentric(A, B, C, QueryPoint));
			Candidate.FitScore = ComputeContainmentScore(Candidate.Barycentric);
		}

		if (OwnerCandidates.Num() == 0)
		{
			++GapCount;
		}
		else if (OwnerCandidates.Num() > 1)
		{
			++OverlapCount;
		}

		TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
		RecoveryCandidates.Reserve(Planet.Plates.Num());
		if (OwnerCandidates.IsEmpty())
		{
			++ZeroCandidateCount;
			for (const FV6PlateQueryGeometry& QueryGeometry : QueryGeometries)
			{
				FRecoveredContainmentHit RecoveryHit;
				if (!TryFindNearestTriangleRecoveryHit(QueryGeometry, QueryPoint, RecoveryHit))
				{
					continue;
				}

				FTectonicPlanetV6RecoveryCandidate& Candidate = RecoveryCandidates.AddDefaulted_GetRef();
				Candidate.PlateId = RecoveryHit.PlateId;
				Candidate.TriangleIndex = RecoveryHit.GlobalTriangleIndex;
				Candidate.Barycentric = RecoveryHit.Barycentric;
				Candidate.DistanceRadians = RecoveryHit.Distance;
			}
		}

		TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
		if (OwnerCandidates.Num() != 1)
		{
			TArray<FV6BoundaryNeighborSummary, TInlineAllocator<8>> NeighborSummaries;
			GatherBoundaryNeighborSummaries(Planet, SampleIndex, NeighborSummaries);

			TArray<int32> RelevantPlateIds;
			CollectRelevantBoundaryPlateIds(PreviousPlateId, OwnerCandidates, RecoveryCandidates, MotionSamples, RelevantPlateIds);
			for (const FV6BoundaryNeighborSummary& Summary : NeighborSummaries)
			{
				if (Summary.PlateId != INDEX_NONE)
				{
					RelevantPlateIds.AddUnique(Summary.PlateId);
				}
			}

			MotionSamples.Reserve(RelevantPlateIds.Num());
			for (const int32 PlateId : RelevantPlateIds)
			{
				const FPlate* Plate = FindPlateById(Planet, PlateId);
				if (Plate == nullptr)
				{
					continue;
				}

				FTectonicPlanetV6BoundaryMotionSample& MotionSample = MotionSamples.AddDefaulted_GetRef();
				MotionSample.PlateId = PlateId;
				MotionSample.SurfaceVelocity = ComputePlateSurfaceVelocity(*Plate, QueryPoint, Planet.PlanetRadiusKm);
				if (const FV6BoundaryNeighborSummary* Summary = FindBoundaryNeighborSummary(NeighborSummaries, PlateId))
				{
					MotionSample.NeighborVoteCount = Summary->VoteCount;
					MotionSample.NeighborTangent = Summary->TangentSum;
				}
			}
		}

		int32 NearestMemberFallbackPlateId = INDEX_NONE;
		int32 NearestMemberFallbackSampleIndex = INDEX_NONE;
		double NearestMemberFallbackDistance = TNumericLimits<double>::Max();
		if (OwnerCandidates.IsEmpty() && RecoveryCandidates.IsEmpty())
		{
			FindGlobalNearestMemberRecovery(
				Planet,
				QueryPoint,
				NearestMemberFallbackPlateId,
				NearestMemberFallbackSampleIndex,
				NearestMemberFallbackDistance);
		}

		int32 ExplicitFallbackPlateId = INDEX_NONE;
		int32 ExplicitFallbackSampleIndex = INDEX_NONE;
		ChooseExplicitFallback(Planet, ExplicitFallbackPlateId, ExplicitFallbackSampleIndex);

		FTectonicPlanetV6ResolvedSample Resolved =
			OwnerCandidates.Num() == 1
				? ResolvePhase1OwnershipForTest(
					OwnerCandidates,
					RecoveryCandidates,
					NearestMemberFallbackPlateId,
					NearestMemberFallbackSampleIndex,
					ExplicitFallbackPlateId,
					ExplicitFallbackSampleIndex)
				: ResolveBoundaryState(
					PreviousPlateId,
					OwnerCandidates,
					RecoveryCandidates,
					MotionSamples,
					QueryPoint,
					Planet.Samples[SampleIndex].ContinentalWeight > 0.5f,
					NearestMemberFallbackPlateId,
					NearestMemberFallbackSampleIndex,
					NearestMemberFallbackDistance,
					ExplicitFallbackPlateId,
					ExplicitFallbackSampleIndex);
		FTectonicPlanetV6ResolvedSample ResolvedWithTransferFallback = Resolved;
		ResolvedWithTransferFallback.PreviousPlateId = PreviousPlateId;
		if (ResolvedWithTransferFallback.BoundaryOutcome == ETectonicPlanetV6BoundaryOutcome::None &&
			ResolvedWithTransferFallback.FinalPlateId != INDEX_NONE &&
			PreviousPlateId != INDEX_NONE)
		{
			ResolvedWithTransferFallback.BoundaryOutcome =
				ResolvedWithTransferFallback.FinalPlateId == PreviousPlateId
					? ETectonicPlanetV6BoundaryOutcome::RetainedOwner
					: ETectonicPlanetV6BoundaryOutcome::ReassignedOwner;
		}
		ResolvedWithTransferFallback.PreCoherencePlateId = ResolvedWithTransferFallback.FinalPlateId;
		if (ResolvedWithTransferFallback.SourceTriangleIndex != INDEX_NONE &&
			ResolvedWithTransferFallback.SourceCanonicalSampleIndex == INDEX_NONE)
		{
			ResolvedWithTransferFallback.SourceCanonicalSampleIndex = PickTransferFallbackCanonicalVertexForTriangle(
				Planet,
				ResolvedWithTransferFallback.FinalPlateId,
				ResolvedWithTransferFallback.SourceTriangleIndex,
				ResolvedWithTransferFallback.SourceBarycentric);
		}

		LastResolvedSamples[SampleIndex] = ResolvedWithTransferFallback;
		NewPlateIds[SampleIndex] = ResolvedWithTransferFallback.FinalPlateId;

		if (OwnerCandidates.Num() == 1)
		{
			++SingleCandidateWinnerCount;
		}
		else if (OwnerCandidates.Num() > 1)
		{
			++MultiCandidateWinnerCount;
		}

		switch (ResolvedWithTransferFallback.ResolutionKind)
		{
		case ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery:
			++ZeroCandidateTriangleRecoveryCount;
			break;
		case ETectonicPlanetV6ResolutionKind::NearestMemberRecovery:
			++ZeroCandidateMemberRecoveryCount;
			LastResolvedSamples[SampleIndex].RecoveryDistanceRadians = NearestMemberFallbackDistance;
			break;
		case ETectonicPlanetV6ResolutionKind::ExplicitFallback:
			++ExplicitFallbackCount;
			break;
		case ETectonicPlanetV6ResolutionKind::None:
		default:
			break;
		}
	});

	const int32 MaxComponentsBeforeCoherence = ComputeMaxComponentsPerPlateFromAssignments(Planet, NewPlateIds);
	int32 CoherenceReassignedSampleCount = 0;
	int32 CoherenceRemovedComponentCount = 0;
	int32 CoherenceLargestRemovedComponentSize = 0;
	int32 FinalMaxComponentsPerPlate = 0;
	ApplyCoherencePass(
		Planet,
		NewPlateIds,
		LastResolvedSamples,
		TinyComponentMaxSizeExclusive,
		CoherenceReassignedSampleCount,
		CoherenceRemovedComponentCount,
		CoherenceLargestRemovedComponentSize,
		FinalMaxComponentsPerPlate);

	for (int32 SampleIndex = 0; SampleIndex < LastResolvedSamples.Num(); ++SampleIndex)
	{
		if (LastResolvedSamples[SampleIndex].bCoherenceReassigned)
		{
			RefreshResolvedSampleTransferSourceForFinalOwner(Planet, QueryGeometries, SampleIndex, LastResolvedSamples[SampleIndex]);
		}
	}

	TArray<int32> PreviousCanonicalPlateIds;
	PreviousCanonicalPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& ExistingSample : Planet.Samples)
	{
		PreviousCanonicalPlateIds.Add(ExistingSample.PlateId);
	}

	ParallelFor(Planet.Samples.Num(), [this, &NewPlateIds, &SubductionDistances, &SubductionSpeeds, &PreviousCanonicalPlateIds](const int32 SampleIndex)
	{
		FSample& Sample = Planet.Samples[SampleIndex];
		FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		Sample.PlateId = Resolved.FinalPlateId;
		Resolved.TransferDebug = FTectonicPlanetV6TransferDebugInfo{};

		bool bTransferred = false;
		if (Resolved.BoundaryOutcome == ETectonicPlanetV6BoundaryOutcome::DivergentOceanic)
		{
			ApplyOceanicBoundaryCreation(
				Sample,
				Planet,
				Resolved,
				SubductionDistances[SampleIndex],
				SubductionSpeeds[SampleIndex],
				Resolved.TransferDebug);
			bTransferred = true;
		}
		else if (Resolved.SourceTriangleIndex != INDEX_NONE && Planet.TriangleIndices.IsValidIndex(Resolved.SourceTriangleIndex))
		{
			const bool bTriangleTransferUsedRecovery = Resolved.RecoveryDistanceRadians >= 0.0;
			const bool bBoundarySymmetricTriangleCandidate =
				Resolved.bBoundaryDecision &&
				(Resolved.BoundaryOutcome == ETectonicPlanetV6BoundaryOutcome::RetainedOwner ||
					Resolved.BoundaryOutcome == ETectonicPlanetV6BoundaryOutcome::ReassignedOwner);
			const FIntVector& Triangle = Planet.TriangleIndices[Resolved.SourceTriangleIndex];
			const FCarriedSample* V0 = FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, Triangle.X);
			const FCarriedSample* V1 = FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, Triangle.Y);
			const FCarriedSample* V2 = FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, Triangle.Z);
			if (V0 != nullptr && V1 != nullptr && V2 != nullptr)
			{
				ApplyTransferredAttributesFromTriangle(
					Sample,
					*V0,
					*V1,
					*V2,
					Resolved.SourceBarycentric,
					SubductionDistances[SampleIndex],
					SubductionSpeeds[SampleIndex],
					Resolved.TransferDebug);
				Resolved.TransferDebug.bUsedTriangleRecovery = bTriangleTransferUsedRecovery;
				Resolved.SourceCanonicalSampleIndex = Resolved.TransferDebug.DominantSourceCanonicalSampleIndex;
				bTransferred = true;
			}
			else if (bBoundarySymmetricTriangleCandidate)
			{
				// Query geometry already duplicates unsplit boundary triangles into the winning plate's frame.
				// For symmetric transfer, keep the same triangle geometry but source each corner's material from
				// the plate that owned that canonical vertex before this solve.
				const FCarriedSample* BoundaryV0 =
					FindCarriedSampleForPreviousOwnerCanonicalVertex(Planet, PreviousCanonicalPlateIds, Triangle.X);
				const FCarriedSample* BoundaryV1 =
					FindCarriedSampleForPreviousOwnerCanonicalVertex(Planet, PreviousCanonicalPlateIds, Triangle.Y);
				const FCarriedSample* BoundaryV2 =
					FindCarriedSampleForPreviousOwnerCanonicalVertex(Planet, PreviousCanonicalPlateIds, Triangle.Z);
				if (BoundaryV0 != nullptr && BoundaryV1 != nullptr && BoundaryV2 != nullptr)
				{
					ApplyTransferredAttributesFromTriangle(
						Sample,
						*BoundaryV0,
						*BoundaryV1,
						*BoundaryV2,
						Resolved.SourceBarycentric,
						SubductionDistances[SampleIndex],
						SubductionSpeeds[SampleIndex],
						Resolved.TransferDebug);
					Resolved.TransferDebug.bUsedTriangleRecovery = bTriangleTransferUsedRecovery;
					Resolved.TransferDebug.bUsedBoundarySymmetricTriangleTransfer = true;
					Resolved.SourceCanonicalSampleIndex = Resolved.TransferDebug.DominantSourceCanonicalSampleIndex;
					bTransferred = true;
				}
			}
		}

		if (!bTransferred && Resolved.FinalPlateId != INDEX_NONE)
		{
			int32 TransferSourceCanonicalSampleIndex = Resolved.SourceCanonicalSampleIndex;
			bool bUsedNearestMemberFallback = false;
			bool bUsedExplicitFallback = false;
			const FCarriedSample* Source =
				TransferSourceCanonicalSampleIndex != INDEX_NONE
					? FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex)
					: nullptr;
			if (Source == nullptr)
			{
				double NearestMemberDistanceRadians = TNumericLimits<double>::Max();
				if (FindNearestMemberSample(
						Planet,
						Resolved.FinalPlateId,
						Sample.Position,
						TransferSourceCanonicalSampleIndex,
						NearestMemberDistanceRadians))
				{
					Source =
						FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
					bUsedNearestMemberFallback = Source != nullptr;
				}
			}
			if (Source == nullptr)
			{
				ChooseExplicitFallbackForPlate(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
				if (TransferSourceCanonicalSampleIndex != INDEX_NONE)
				{
					Source =
						FindCarriedSampleForCanonicalVertex(Planet, Resolved.FinalPlateId, TransferSourceCanonicalSampleIndex);
					bUsedExplicitFallback = Source != nullptr;
				}
			}
			if (Source != nullptr)
			{
				Resolved.SourceCanonicalSampleIndex = TransferSourceCanonicalSampleIndex;
				ApplyTransferredAttributesFromSingleSource(
					Sample,
					*Source,
					SubductionDistances[SampleIndex],
					SubductionSpeeds[SampleIndex],
					Resolved.TransferDebug);
				Resolved.TransferDebug.bUsedNearestMemberFallback = bUsedNearestMemberFallback;
				Resolved.TransferDebug.bUsedExplicitFallback = bUsedExplicitFallback;
				bTransferred = true;
			}
		}

		if (!bTransferred)
		{
			Sample.ContinentalWeight = 0.0f;
			Sample.Elevation = -6.0f;
			Sample.Thickness = 7.0f;
			Sample.Age = 0.0f;
			Sample.OrogenyType = EOrogenyType::None;
			Sample.TerraneId = INDEX_NONE;
			Sample.RidgeDirection = FVector3d::ZeroVector;
			Sample.FoldDirection = FVector3d::ZeroVector;
			SubductionDistances[SampleIndex] = -1.0f;
			SubductionSpeeds[SampleIndex] = 0.0f;
			Resolved.TransferDebug = FTectonicPlanetV6TransferDebugInfo{};
			Resolved.TransferDebug.SourceKind = ETectonicPlanetV6TransferSourceKind::Defaulted;
		}
	});

	FTectonicPlanetV6PeriodicSolveStats TransferStats;
	int32 BoundaryDecisionSampleCount = 0;
	int32 BoundaryRetainedCount = 0;
	int32 BoundaryReassignedCount = 0;
	int32 BoundaryOceanicCount = 0;
	int32 BoundaryLimitedFallbackCount = 0;
	for (const FTectonicPlanetV6ResolvedSample& Resolved : LastResolvedSamples)
	{
		if (Resolved.bBoundaryDecision)
		{
			++BoundaryDecisionSampleCount;
			switch (Resolved.BoundaryOutcome)
			{
			case ETectonicPlanetV6BoundaryOutcome::RetainedOwner:
				++BoundaryRetainedCount;
				break;
			case ETectonicPlanetV6BoundaryOutcome::ReassignedOwner:
				++BoundaryReassignedCount;
				break;
			case ETectonicPlanetV6BoundaryOutcome::DivergentOceanic:
				++BoundaryOceanicCount;
				break;
			case ETectonicPlanetV6BoundaryOutcome::None:
			default:
				break;
			}

			if (Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery ||
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::NearestMemberRecovery ||
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ExplicitFallback)
			{
				++BoundaryLimitedFallbackCount;
			}

			if (FTectonicPlanetV6BoundaryOutcomeTransferStats* OutcomeStats =
					FindBoundaryOutcomeTransferStats(TransferStats, Resolved.BoundaryOutcome))
			{
				AccumulateBoundaryOutcomeTransferStats(*OutcomeStats, Resolved);
			}
		}

		switch (Resolved.TransferDebug.SourceKind)
		{
		case ETectonicPlanetV6TransferSourceKind::Triangle:
			++TransferStats.TriangleTransferCount;
			IncrementTransferResolutionCounts(TransferStats.TriangleTransferCountsByResolution, Resolved);
			break;
		case ETectonicPlanetV6TransferSourceKind::SingleSource:
			++TransferStats.SingleSourceTransferCount;
			IncrementTransferResolutionCounts(TransferStats.SingleSourceTransferCountsByResolution, Resolved);
			break;
		case ETectonicPlanetV6TransferSourceKind::OceanicCreation:
			++TransferStats.OceanicCreationCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::Defaulted:
			++TransferStats.DefaultTransferCount;
			break;
		case ETectonicPlanetV6TransferSourceKind::None:
		default:
			break;
		}

		if (UsesCategoricalMajorityTransfer(Resolved.TransferDebug))
		{
			++TransferStats.CategoricalMajorityTransferCount;
		}
		else if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle ||
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::SingleSource)
		{
			++TransferStats.CategoricalDominantTransferCount;
		}

		if (UsesDirectionalFallback(Resolved.TransferDebug))
		{
			++TransferStats.DirectionalFallbackCount;
		}

		if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
		{
			++TransferStats.ContinentalWeightThresholdMismatchCount;
			IncrementTransferResolutionCounts(
				TransferStats.ContinentalWeightThresholdMismatchCountsByResolution,
				Resolved);
		}
	}

	Planet.RepartitionMembership(NewPlateIds, &SubductionDistances, &SubductionSpeeds);
	Planet.ComputeSubductionDistanceField();
	Planet.ComputeSlabPullCorrections();
	Planet.DetectTerranes(PreviousTerranes);

	++PeriodicSolveCount;
	PeriodicSolveSteps.Add(Planet.CurrentStep);

	LastSolveStats = FTectonicPlanetV6PeriodicSolveStats{};
	LastSolveStats.Trigger = Trigger;
	LastSolveStats.SolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative;
	LastSolveStats.Step = Planet.CurrentStep;
	LastSolveStats.Interval = Interval;
	LastSolveStats.SolveIndex = PeriodicSolveCount;
	LastSolveStats.PlateCount = Planet.Plates.Num();
	LastSolveStats.SingleCandidateWinnerCount = SingleCandidateWinnerCount.Load();
	LastSolveStats.MultiCandidateWinnerCount = MultiCandidateWinnerCount.Load();
	LastSolveStats.ZeroCandidateCount = ZeroCandidateCount.Load();
	LastSolveStats.ZeroCandidateTriangleRecoveryCount = ZeroCandidateTriangleRecoveryCount.Load();
	LastSolveStats.ZeroCandidateMemberRecoveryCount = ZeroCandidateMemberRecoveryCount.Load();
	LastSolveStats.ExplicitFallbackCount = ExplicitFallbackCount.Load();
	LastSolveStats.CoherenceReassignedSampleCount = CoherenceReassignedSampleCount;
	LastSolveStats.CoherenceRemovedComponentCount = CoherenceRemovedComponentCount;
	LastSolveStats.CoherenceLargestRemovedComponentSize = CoherenceLargestRemovedComponentSize;
	LastSolveStats.GapCount = GapCount.Load();
	LastSolveStats.OverlapCount = OverlapCount.Load();
	LastSolveStats.BoundaryDecisionSampleCount = BoundaryDecisionSampleCount;
	LastSolveStats.BoundaryRetainedCount = BoundaryRetainedCount;
	LastSolveStats.BoundaryReassignedCount = BoundaryReassignedCount;
	LastSolveStats.BoundaryOceanicCount = BoundaryOceanicCount;
	LastSolveStats.BoundaryLimitedFallbackCount = BoundaryLimitedFallbackCount;
	LastSolveStats.TriangleTransferCount = TransferStats.TriangleTransferCount;
	LastSolveStats.SingleSourceTransferCount = TransferStats.SingleSourceTransferCount;
	LastSolveStats.OceanicCreationCount = TransferStats.OceanicCreationCount;
	LastSolveStats.DefaultTransferCount = TransferStats.DefaultTransferCount;
	LastSolveStats.CategoricalMajorityTransferCount = TransferStats.CategoricalMajorityTransferCount;
	LastSolveStats.CategoricalDominantTransferCount = TransferStats.CategoricalDominantTransferCount;
	LastSolveStats.DirectionalFallbackCount = TransferStats.DirectionalFallbackCount;
	LastSolveStats.ContinentalWeightThresholdMismatchCount = TransferStats.ContinentalWeightThresholdMismatchCount;
	LastSolveStats.TriangleTransferCountsByResolution = TransferStats.TriangleTransferCountsByResolution;
	LastSolveStats.SingleSourceTransferCountsByResolution = TransferStats.SingleSourceTransferCountsByResolution;
	LastSolveStats.ContinentalWeightThresholdMismatchCountsByResolution = TransferStats.ContinentalWeightThresholdMismatchCountsByResolution;
	LastSolveStats.BoundaryRetainedTransferStats = TransferStats.BoundaryRetainedTransferStats;
	LastSolveStats.BoundaryReassignedTransferStats = TransferStats.BoundaryReassignedTransferStats;
	LastSolveStats.BoundaryOceanicTransferStats = TransferStats.BoundaryOceanicTransferStats;
	LastSolveStats.BoundarySampleCount = CountBoundarySamples(Planet);
	LastSolveStats.ContinentalAreaFraction = ComputeContinentalAreaFraction(Planet);
	LastSolveStats.MaxComponentsBeforeCoherence = MaxComponentsBeforeCoherence;
	LastSolveStats.MaxComponentsPerPlate = FinalMaxComponentsPerPlate;
	LastSolveStats.MaxComponentsPerPlate = ComputeMaxComponentsPerPlate(Planet);
	if (LastSolveStats.ExplicitFallbackCount > 0)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("[TectonicPlanetV6 Phase3Fallback Step=%d] explicit_fallback_count=%d"),
			Planet.CurrentStep,
			LastSolveStats.ExplicitFallbackCount);
	}

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[TectonicPlanetV6 Phase3 Step=%d] solve_ms=%.3f single=%d overlap=%d zero=%d triangle_recovery=%d member_recovery=%d explicit_fallback=%d boundary_decisions=%d retained=%d reassigned=%d oceanic=%d limited_fallback=%d coherence_reassigned=%d coherence_removed_components=%d largest_removed_component=%d triangle_transfer=%d single_source_transfer=%d oceanic_creation=%d default_transfer=%d categorical_majority=%d categorical_dominant=%d directional_fallback=%d cw_threshold_mismatch=%d continental_area_fraction=%.6f max_components_before=%d max_components_after=%d"),
		Planet.CurrentStep,
		(FPlatformTime::Seconds() - SolveStartTime) * 1000.0,
		LastSolveStats.SingleCandidateWinnerCount,
		LastSolveStats.MultiCandidateWinnerCount,
		LastSolveStats.ZeroCandidateCount,
		LastSolveStats.ZeroCandidateTriangleRecoveryCount,
		LastSolveStats.ZeroCandidateMemberRecoveryCount,
		LastSolveStats.ExplicitFallbackCount,
		LastSolveStats.BoundaryDecisionSampleCount,
		LastSolveStats.BoundaryRetainedCount,
		LastSolveStats.BoundaryReassignedCount,
		LastSolveStats.BoundaryOceanicCount,
		LastSolveStats.BoundaryLimitedFallbackCount,
		LastSolveStats.CoherenceReassignedSampleCount,
		LastSolveStats.CoherenceRemovedComponentCount,
		LastSolveStats.CoherenceLargestRemovedComponentSize,
		LastSolveStats.TriangleTransferCount,
		LastSolveStats.SingleSourceTransferCount,
		LastSolveStats.OceanicCreationCount,
		LastSolveStats.DefaultTransferCount,
		LastSolveStats.CategoricalMajorityTransferCount,
		LastSolveStats.CategoricalDominantTransferCount,
		LastSolveStats.DirectionalFallbackCount,
		LastSolveStats.ContinentalWeightThresholdMismatchCount,
		LastSolveStats.ContinentalAreaFraction,
		MaxComponentsBeforeCoherence,
		LastSolveStats.MaxComponentsPerPlate);

	const ETectonicPlanetV6BoundaryOutcome LoggedOutcomes[3] = {
		ETectonicPlanetV6BoundaryOutcome::RetainedOwner,
		ETectonicPlanetV6BoundaryOutcome::ReassignedOwner,
		ETectonicPlanetV6BoundaryOutcome::DivergentOceanic };
	for (const ETectonicPlanetV6BoundaryOutcome Outcome : LoggedOutcomes)
	{
		const FTectonicPlanetV6BoundaryOutcomeTransferStats* OutcomeStats =
			FindBoundaryOutcomeTransferStats(LastSolveStats, Outcome);
		if (OutcomeStats == nullptr)
		{
			continue;
		}

		UE_LOG(
			LogTemp,
			Log,
			TEXT("[TectonicPlanetV6 Phase3BoundaryTransfer Step=%d Outcome=%s] samples=%d triangle=%d triangle_recovery=%d single_source=%d nearest_member_single_source=%d explicit_fallback_single_source=%d oceanic_creation=%d default_transfer=%d cw_threshold_mismatch=%d"),
			Planet.CurrentStep,
			GetBoundaryOutcomeName(Outcome),
			OutcomeStats->SampleCount,
			OutcomeStats->TriangleTransferCount,
			OutcomeStats->TriangleRecoveryTransferCount,
			OutcomeStats->SingleSourceTransferCount,
			OutcomeStats->NearestMemberSingleSourceTransferCount,
			OutcomeStats->ExplicitFallbackSingleSourceTransferCount,
			OutcomeStats->OceanicCreationCount,
			OutcomeStats->DefaultTransferCount,
			OutcomeStats->ContinentalWeightThresholdMismatchCount);
	}

	RebuildSubmergedContinentalFringeRelaxationMask();
}

void FTectonicPlanetV6::RebuildSubmergedContinentalFringeRelaxationMask()
{
	const int32 SampleCount = Planet.Samples.Num();
	CurrentSolveSubmergedContinentalFringeFlags.Init(0, SampleCount);
	CurrentSolveSubmergedContinentalFringeBoundaryOrActiveFlags.Init(0, SampleCount);
	CurrentSolveSubmergedContinentalFringeOceanicNeighborFractions.Init(0.0f, SampleCount);

	if (!bEnableV9SubmergedContinentalFringeRelaxationForTest)
	{
		return;
	}

	const bool bHasActiveZoneFlags = CurrentSolveActiveZoneFlags.Num() == SampleCount;
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const FSample& Sample = Planet.Samples[SampleIndex];
		if (Sample.ContinentalWeight < 0.5f)
		{
			continue;
		}

		if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex) || Planet.SampleAdjacency[SampleIndex].IsEmpty())
		{
			continue;
		}

		int32 ValidNeighborCount = 0;
		int32 OceanicNeighborCount = 0;
		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (!Planet.Samples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			++ValidNeighborCount;
			if (Planet.Samples[NeighborIndex].ContinentalWeight < 0.5f)
			{
				++OceanicNeighborCount;
			}
		}

		if (ValidNeighborCount == 0 || OceanicNeighborCount == 0)
		{
			continue;
		}

		CurrentSolveSubmergedContinentalFringeFlags[SampleIndex] = 1;
		CurrentSolveSubmergedContinentalFringeOceanicNeighborFractions[SampleIndex] =
			static_cast<float>(OceanicNeighborCount) / static_cast<float>(ValidNeighborCount);
		const bool bBoundaryOrActive =
			Sample.bIsBoundary ||
			(bHasActiveZoneFlags && CurrentSolveActiveZoneFlags[SampleIndex] != 0);
		CurrentSolveSubmergedContinentalFringeBoundaryOrActiveFlags[SampleIndex] =
			bBoundaryOrActive ? 1 : 0;
	}
}

int32 FTectonicPlanetV6::ComputePeriodicSolveInterval() const
{
	if (PeriodicSolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike ||
		IsCopiedFrontierLikeSolveMode(PeriodicSolveMode) ||
		PeriodicSolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike)
	{
		return FMath::Max(1, ThesisRemeshFixedIntervalSteps);
	}

	return Planet.ComputeResampleInterval();
}

int32 FTectonicPlanetV6::GetPeriodicSolveCount() const
{
	return PeriodicSolveCount;
}

bool FTectonicPlanetV6::IsPhase1AuthoritativePeriodicPath() const
{
	return true;
}

ETectonicPlanetV6PeriodicSolveMode FTectonicPlanetV6::GetPeriodicSolveMode() const
{
	return PeriodicSolveMode;
}

FTectonicPlanetV6PeriodicSolveStats FTectonicPlanetV6::BuildCurrentDiagnosticSnapshotForTest() const
{
	FTectonicPlanetV6PeriodicSolveStats Snapshot;
	Snapshot.SolveMode = PeriodicSolveMode;
	Snapshot.Step = Planet.CurrentStep;
	Snapshot.Interval = ComputePeriodicSolveInterval();
	Snapshot.SolveIndex = PeriodicSolveCount;
	Snapshot.PlateCount = Planet.Plates.Num();
	Snapshot.SolveMilliseconds = 0.0;
	Snapshot.BoundarySampleCount = CountBoundarySamples(Planet);
	Snapshot.ContinentalAreaFraction = ComputeContinentalAreaFraction(Planet);
	Snapshot.MaxComponentsPerPlate = ComputeMaxComponentsPerPlate(Planet);
	for (const uint8 TrackedKind : CopiedFrontierTrackedDestructiveKinds)
	{
		switch (static_cast<EV6CopiedFrontierDestructiveKind>(TrackedKind))
		{
		case EV6CopiedFrontierDestructiveKind::Subduction:
			++Snapshot.TrackedSubductionTriangleCount;
			break;
		case EV6CopiedFrontierDestructiveKind::Collision:
			++Snapshot.TrackedCollisionTriangleCount;
			break;
		case EV6CopiedFrontierDestructiveKind::None:
		default:
			break;
		}
	}
	Snapshot.TrackedDestructiveTriangleCount =
		Snapshot.TrackedSubductionTriangleCount +
		Snapshot.TrackedCollisionTriangleCount;
	Snapshot.TrackedDestructiveTriangleNewlySeededCount =
		CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount;
	Snapshot.TrackedDestructiveTrianglePropagatedCount =
		CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount;
	Snapshot.TrackedDestructiveTriangleExpiredCount =
		CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount;
	Snapshot.TrackingLifecycleStats =
		CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats;

	if (IsCopiedFrontierLikeSolveMode(PeriodicSolveMode))
	{
		for (const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh : ThesisCopiedFrontierMeshes)
		{
			Snapshot.PlateLocalVertexCount += Mesh.BaseVertices.Num();
			Snapshot.PlateLocalTriangleCount += Mesh.LocalTriangles.Num();
			Snapshot.CopiedFrontierVertexCount += Mesh.CopiedFrontierVertexCount;
			Snapshot.CopiedFrontierTriangleCount += Mesh.CopiedFrontierTriangleCount;
			Snapshot.CopiedFrontierCarriedSampleCount += Mesh.CopiedFrontierCarriedSampleCount;
		}
	}
	else if (PeriodicSolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike)
	{
		for (const FTectonicPlanetV6PlateSubmesh& Mesh : ThesisPlateSubmeshMeshes)
		{
			Snapshot.PlateLocalVertexCount += Mesh.BaseVertices.Num();
			Snapshot.PlateLocalTriangleCount += Mesh.LocalTriangles.Num();
			Snapshot.PlateSubmeshFrontierVertexCount += Mesh.FrontierVertexCount;
			Snapshot.PlateSubmeshFrontierTriangleCount += Mesh.FrontierTriangleCount;
			Snapshot.PlateSubmeshFrontierCarriedSampleCount += Mesh.FrontierCarriedSampleCount;
			Snapshot.PlateSubmeshRetriangulatedTriangleCount += Mesh.RetriangulatedTriangleCount;
			Snapshot.PlateSubmeshComponentCount += Mesh.ComponentCount;
			Snapshot.PlateSubmeshWholeMixedTriangleDuplicationCount += Mesh.WholeMixedTriangleDuplicationCount;
		}
	}

	return Snapshot;
}

FTectonicPlanetV6CopiedFrontierRayDiagnostics FTectonicPlanetV6::BuildCopiedFrontierRayDiagnosticsForTest(
	const int32 MaxMissSamples) const
{
	FTectonicPlanetV6CopiedFrontierRayDiagnostics Diagnostics;
	Diagnostics.SampleCount = Planet.Samples.Num();

	TArray<FTectonicPlanetV6CopiedFrontierPlateMesh> PlateMeshes;
	if (ThesisCopiedFrontierMeshes.Num() == Planet.Plates.Num())
	{
		PlateMeshes = ThesisCopiedFrontierMeshes;
	}
	else
	{
		BuildV6CopiedFrontierPlateMeshes(
			Planet,
			PlateMeshes,
			nullptr,
			GetEffectiveThesisFrontierMeshBuildMode(
				PeriodicSolveMode,
				bForceWholeTriangleBoundaryDuplicationForTest,
				bForceExcludeMixedTrianglesForTest),
			UsesStructuredGapFillForCopiedFrontierSolveConfiguration(
				PeriodicSolveMode,
				bForceWholeTriangleBoundaryDuplicationForTest,
				bForceExcludeMixedTrianglesForTest) &&
				PreviousSolveSyntheticFlags.Num() == Planet.Samples.Num()
				? &PreviousSolveSyntheticFlags
				: nullptr);
	}

	TArray<FV6PlateQueryGeometry> QueryGeometries;
	BuildV6CopiedFrontierQueryGeometries(Planet, PlateMeshes, QueryGeometries);

	TArray<TArray<int32>> SampleToAdjacentTriangles;
	SampleToAdjacentTriangles.SetNum(Planet.Samples.Num());
	for (int32 GlobalTriangleIndex = 0; GlobalTriangleIndex < Planet.TriangleIndices.Num(); ++GlobalTriangleIndex)
	{
		if (!Planet.TriangleIndices.IsValidIndex(GlobalTriangleIndex))
		{
			continue;
		}

		const FIntVector& Triangle = Planet.TriangleIndices[GlobalTriangleIndex];
		if (SampleToAdjacentTriangles.IsValidIndex(Triangle.X))
		{
			SampleToAdjacentTriangles[Triangle.X].Add(GlobalTriangleIndex);
		}
		if (SampleToAdjacentTriangles.IsValidIndex(Triangle.Y))
		{
			SampleToAdjacentTriangles[Triangle.Y].Add(GlobalTriangleIndex);
		}
		if (SampleToAdjacentTriangles.IsValidIndex(Triangle.Z))
		{
			SampleToAdjacentTriangles[Triangle.Z].Add(GlobalTriangleIndex);
		}
	}

	constexpr double VertexBarycentricDiagnosticEpsilon = 1.0e-10;
	constexpr double EdgeBarycentricDiagnosticEpsilon = 1.0e-8;
	constexpr double DegenerateAreaDiagnosticEpsilon = 1.0e-14;

	auto AccumulateBarycentricFlags = [](
		const FVector3d& A,
		const FVector3d& B,
		const FVector3d& C,
		const FVector3d& Barycentric,
		bool& bInOutNearVertex,
		bool& bInOutNearEdge,
		bool& bInOutNearDegenerate)
	{
		const double MaxWeight = FMath::Max3(Barycentric.X, Barycentric.Y, Barycentric.Z);
		const double MinWeight = FMath::Min3(Barycentric.X, Barycentric.Y, Barycentric.Z);
		bInOutNearVertex = bInOutNearVertex || MaxWeight >= (1.0 - VertexBarycentricDiagnosticEpsilon);
		bInOutNearEdge = bInOutNearEdge || MinWeight <= EdgeBarycentricDiagnosticEpsilon;
		bInOutNearDegenerate =
			bInOutNearDegenerate ||
			UE::Geometry::VectorUtil::Normal(A, B, C).SquaredLength() <= DegenerateAreaDiagnosticEpsilon;
	};

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		if (!Planet.Samples.IsValidIndex(SampleIndex))
		{
			continue;
		}

		const FSample& Sample = Planet.Samples[SampleIndex];
		const FVector3d QueryPoint = Sample.Position;
		const FVector3d RayDirection = QueryPoint.GetSafeNormal();
		if (RayDirection.IsNearlyZero())
		{
			continue;
		}

		bool bLegacyRawRayHit = false;
		for (const FV6PlateQueryGeometry& QueryGeometry : QueryGeometries)
		{
			FV6ThesisRemeshRayHit RawHit;
			if (DoesQueryPointPassBoundingCap(QueryGeometry, RayDirection) &&
				TryFindThesisRemeshRawRayHitNoCap(QueryGeometry, RayDirection, RawHit))
			{
				bLegacyRawRayHit = true;
				break;
			}
		}
		if (bLegacyRawRayHit)
		{
			continue;
		}

		++Diagnostics.RawRayMissCount;
		Diagnostics.BoundaryRawRayMissCount += Sample.bIsBoundary ? 1 : 0;

		FTectonicPlanetV6CopiedFrontierMissDiagnostic Miss;
		Miss.SampleIndex = SampleIndex;
		Miss.Position = QueryPoint;
		Miss.PreviousPlateId = Sample.PlateId;
		Miss.bBoundarySample = Sample.bIsBoundary;
		if (SampleToAdjacentTriangles.IsValidIndex(SampleIndex))
		{
			Miss.AdjacentGlobalTriangleIndices = SampleToAdjacentTriangles[SampleIndex];
		}
		Miss.AdjacentGlobalTriangleCount = Miss.AdjacentGlobalTriangleIndices.Num();

		for (const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh : PlateMeshes)
		{
			const int32* LocalVertexIndex = Mesh.CanonicalToLocalVertex.Find(SampleIndex);
			if (LocalVertexIndex != nullptr &&
				Mesh.LocalVertexCopiedFrontierFlags.IsValidIndex(*LocalVertexIndex) &&
				Mesh.LocalVertexCopiedFrontierFlags[*LocalVertexIndex] != 0)
			{
				Miss.bAtCopiedFrontierVertex = true;
				break;
			}
		}

		bool bAnyAdjacentTriangleCopy = false;
		bool bAnyBoundingCapRejectedRawRayHit = false;
		bool bAnyContainingTriangle = false;
		bool bAnyAdjacentContainingTriangle = false;
		bool bAnyBoundingCapRejectedContainingTriangle = false;

		for (int32 PlateIndex = 0; PlateIndex < PlateMeshes.Num() && PlateIndex < QueryGeometries.Num(); ++PlateIndex)
		{
			const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh = PlateMeshes[PlateIndex];
			const FV6PlateQueryGeometry& QueryGeometry = QueryGeometries[PlateIndex];
			FTectonicPlanetV6CopiedFrontierMissPlateDiagnostic PlateDiagnostic;
			PlateDiagnostic.PlateId = Mesh.PlateId;
			PlateDiagnostic.bBoundingCapRejected = !DoesQueryPointPassBoundingCap(QueryGeometry, RayDirection);

			for (const int32 GlobalTriangleIndex : Miss.AdjacentGlobalTriangleIndices)
			{
				if (Mesh.GlobalToLocalTriangle.Contains(GlobalTriangleIndex))
				{
					++PlateDiagnostic.AdjacentTriangleCopyCount;
				}
			}
			PlateDiagnostic.bHasAdjacentTriangleCopy = PlateDiagnostic.AdjacentTriangleCopyCount > 0;
			if (PlateDiagnostic.bHasAdjacentTriangleCopy)
			{
				bAnyAdjacentTriangleCopy = true;
				++Miss.PlateMeshCountWithAdjacentTriangleCopies;
			}

			FV6ThesisRemeshRayHit RawHitIgnoringCap;
			PlateDiagnostic.bRawRayHitIgnoringCap = TryFindThesisRemeshRawRayHitNoCap(
				QueryGeometry,
				RayDirection,
				RawHitIgnoringCap);
			PlateDiagnostic.bRawRayHit =
				!PlateDiagnostic.bBoundingCapRejected && PlateDiagnostic.bRawRayHitIgnoringCap;
			Miss.RawRayHitPlateCount += PlateDiagnostic.bRawRayHit ? 1 : 0;
			bAnyBoundingCapRejectedRawRayHit =
				bAnyBoundingCapRejectedRawRayHit ||
				(PlateDiagnostic.bBoundingCapRejected && PlateDiagnostic.bRawRayHitIgnoringCap);

			FV6ThesisRemeshRayHit ContainingHit;
			PlateDiagnostic.bContainingTriangleHit = TryFindThesisRemeshContainingHitNoCap(
				QueryGeometry,
				QueryPoint,
				ContainingHit);
			if (PlateDiagnostic.bContainingTriangleHit)
			{
				bool bAdjacentContainingTriangle = false;
				for (const int32 GlobalTriangleIndex : Miss.AdjacentGlobalTriangleIndices)
				{
					if (ContainingHit.GlobalTriangleIndex == GlobalTriangleIndex)
					{
						bAdjacentContainingTriangle = true;
						break;
					}
				}

				PlateDiagnostic.bContainingAdjacentTriangleHit = bAdjacentContainingTriangle;
				if (PlateDiagnostic.bContainingAdjacentTriangleHit)
				{
					++Miss.AdjacentContainingTrianglePlateCount;
				}
				++Miss.ContainingTrianglePlateCount;

				if (QueryGeometry.SoupData.LocalTriangles.IsValidIndex(ContainingHit.LocalTriangleIndex))
				{
					FVector3d A;
					FVector3d B;
					FVector3d C;
					QueryGeometry.SoupAdapter.GetTriVertices(ContainingHit.LocalTriangleIndex, A, B, C);
					AccumulateBarycentricFlags(
						A,
						B,
						C,
						ContainingHit.Barycentric,
						PlateDiagnostic.bNearVertex,
						PlateDiagnostic.bNearEdge,
						PlateDiagnostic.bNearDegenerate);
				}
			}

			bAnyContainingTriangle = bAnyContainingTriangle || PlateDiagnostic.bContainingTriangleHit;
			bAnyAdjacentContainingTriangle = bAnyAdjacentContainingTriangle || PlateDiagnostic.bContainingAdjacentTriangleHit;
			bAnyBoundingCapRejectedContainingTriangle =
				bAnyBoundingCapRejectedContainingTriangle ||
				(PlateDiagnostic.bBoundingCapRejected && PlateDiagnostic.bContainingTriangleHit);
			Miss.BoundingCapRejectedPlateCount += PlateDiagnostic.bBoundingCapRejected ? 1 : 0;
			Miss.bNearVertex = Miss.bNearVertex || PlateDiagnostic.bNearVertex;
			Miss.bNearEdge = Miss.bNearEdge || PlateDiagnostic.bNearEdge;
			Miss.bNearDegenerate = Miss.bNearDegenerate || PlateDiagnostic.bNearDegenerate;

			if (PlateDiagnostic.bHasAdjacentTriangleCopy ||
				PlateDiagnostic.bBoundingCapRejected ||
				PlateDiagnostic.bContainingTriangleHit ||
				PlateDiagnostic.bRawRayHitIgnoringCap)
			{
				Miss.PlateDiagnostics.Add(MoveTemp(PlateDiagnostic));
			}
		}

		Miss.bCoverageGap = !bAnyAdjacentTriangleCopy;
		Miss.bRawRayMissRecoveredByContainingTriangle = bAnyContainingTriangle;
		Miss.bBoundingCapRejectedContainingTriangle = bAnyBoundingCapRejectedContainingTriangle;

		for (const int32 GlobalTriangleIndex : Miss.AdjacentGlobalTriangleIndices)
		{
			FTectonicPlanetV6CopiedFrontierAdjacentTriangleDiagnostic& TriangleDiagnostic =
				Miss.AdjacentTriangleDiagnostics.AddDefaulted_GetRef();
			TriangleDiagnostic.GlobalTriangleIndex = GlobalTriangleIndex;

			for (const FTectonicPlanetV6CopiedFrontierPlateMesh& Mesh : PlateMeshes)
			{
				if (Mesh.GlobalToLocalTriangle.Contains(GlobalTriangleIndex))
				{
					TriangleDiagnostic.PlateIdsWithCopies.Add(Mesh.PlateId);
				}
			}
		}

		Diagnostics.CoverageGapMissCount += Miss.bCoverageGap ? 1 : 0;
		Diagnostics.BoundingCapRejectedRawRayHitMissCount += bAnyBoundingCapRejectedRawRayHit ? 1 : 0;
		Diagnostics.BoundingCapRejectedContainingTriangleMissCount +=
			Miss.bBoundingCapRejectedContainingTriangle ? 1 : 0;
		Diagnostics.ContainingTriangleRecoveredMissCount +=
			Miss.bRawRayMissRecoveredByContainingTriangle ? 1 : 0;
		Diagnostics.AdjacentContainingTriangleRecoveredMissCount +=
			bAnyAdjacentContainingTriangle ? 1 : 0;
		Diagnostics.NearVertexMissCount += Miss.bNearVertex ? 1 : 0;
		Diagnostics.NearEdgeMissCount += Miss.bNearEdge ? 1 : 0;
		Diagnostics.NearDegenerateMissCount += Miss.bNearDegenerate ? 1 : 0;

		if (Diagnostics.MissDiagnostics.Num() < MaxMissSamples)
		{
			Diagnostics.MissDiagnostics.Add(MoveTemp(Miss));
		}
	}

	return Diagnostics;
}

const FTectonicPlanetV6CopiedFrontierSolveAttribution& FTectonicPlanetV6::GetLastCopiedFrontierSolveAttributionForTest() const
{
	return LastCopiedFrontierSolveAttribution;
}

const TArray<uint8>& FTectonicPlanetV6::GetTrackedCopiedFrontierDestructiveKindsForTest() const
{
	return CopiedFrontierTrackedDestructiveKinds;
}

void FTectonicPlanetV6::RefreshCopiedFrontierDestructiveTrackingForTest()
{
	RebuildThesisCopiedFrontierMeshes();
	FV6CopiedFrontierDestructiveTrackingUpdateStats TrackingStats;
	SeedCopiedFrontierIntervalDestructiveTracking(
		Planet,
		ThesisCopiedFrontierMeshes,
		CopiedFrontierTrackedDestructiveKinds,
		CopiedFrontierTrackedPreferredContinuationPlateIds,
		CopiedFrontierTrackedDestructiveSourcePlateIds,
		CopiedFrontierTrackedDestructiveDistancesKm,
		CopiedFrontierTrackedDestructiveSeedOriginFlags,
		CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags,
		CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags,
		TrackingStats);
	CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount =
		TrackingStats.NewlySeededTriangleCount;
	CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats =
		TrackingStats.LifecycleStats;
	CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount = 0;
	CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount = 0;
	CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
}

void FTectonicPlanetV6::SetCopiedFrontierIntervalPropagationWaveCapForTest(const int32 InMaxWaves)
{
	CopiedFrontierIntervalPropagationWaveCapForTest = InMaxWaves < 0 ? INDEX_NONE : InMaxWaves;
	CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
}

FTectonicPlanetV6BoundaryCoherenceDiagnostic FTectonicPlanetV6::ComputeBoundaryCoherenceDiagnosticForTest() const
{
	FTectonicPlanetV6BoundaryCoherenceDiagnostic Diag;
	const int32 SampleCount = Planet.Samples.Num();
	if (SampleCount == 0 || Planet.SampleAdjacency.Num() != SampleCount)
	{
		return Diag;
	}

	// --- Step 1: Identify boundary band from current plate assignments ---
	TArray<uint8> IsBoundaryBand;
	IsBoundaryBand.Init(0, SampleCount);
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const int32 PlateId = Planet.Samples[SampleIndex].PlateId;
		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (Planet.Samples.IsValidIndex(NeighborIndex) &&
				Planet.Samples[NeighborIndex].PlateId != PlateId)
			{
				IsBoundaryBand[SampleIndex] = 1;
				break;
			}
		}
	}

	for (const uint8 Flag : IsBoundaryBand)
	{
		Diag.BoundaryBandSampleCount += Flag;
	}

	// --- Step 2: Connected components within boundary band ---
	TArray<uint8> BoundaryVisited;
	BoundaryVisited.Init(0, SampleCount);
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		if (IsBoundaryBand[SampleIndex] == 0 || BoundaryVisited[SampleIndex] != 0)
		{
			continue;
		}

		int32 ComponentSize = 0;
		TArray<int32, TInlineAllocator<256>> Stack;
		Stack.Add(SampleIndex);
		BoundaryVisited[SampleIndex] = 1;

		while (!Stack.IsEmpty())
		{
			const int32 Current = Stack.Pop(EAllowShrinking::No);
			++ComponentSize;

			for (const int32 NeighborIndex : Planet.SampleAdjacency[Current])
			{
				if (Planet.Samples.IsValidIndex(NeighborIndex) &&
					IsBoundaryBand[NeighborIndex] != 0 &&
					BoundaryVisited[NeighborIndex] == 0)
				{
					BoundaryVisited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}
		}

		++Diag.BoundaryBandComponentCount;
		Diag.LargestBoundaryBandComponentSize = FMath::Max(
			Diag.LargestBoundaryBandComponentSize, ComponentSize);
		if (Diag.SmallestBoundaryBandComponentSize == 0)
		{
			Diag.SmallestBoundaryBandComponentSize = ComponentSize;
		}
		else
		{
			Diag.SmallestBoundaryBandComponentSize = FMath::Min(
				Diag.SmallestBoundaryBandComponentSize, ComponentSize);
		}
		if (ComponentSize == 1)
		{
			++Diag.SingletonBoundaryBandComponentCount;
		}
	}

	// --- Step 3: BFS from boundary band to compute ring distance for every sample ---
	TArray<int32> RingDistance;
	RingDistance.Init(INDEX_NONE, SampleCount);
	TArray<int32> BfsQueue;
	BfsQueue.Reserve(Diag.BoundaryBandSampleCount);
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		if (IsBoundaryBand[SampleIndex] != 0)
		{
			RingDistance[SampleIndex] = 0;
			BfsQueue.Add(SampleIndex);
		}
	}

	int32 BfsHead = 0;
	while (BfsHead < BfsQueue.Num())
	{
		const int32 Current = BfsQueue[BfsHead++];
		const int32 CurrentDist = RingDistance[Current];
		for (const int32 NeighborIndex : Planet.SampleAdjacency[Current])
		{
			if (Planet.Samples.IsValidIndex(NeighborIndex) &&
				RingDistance[NeighborIndex] == INDEX_NONE)
			{
				RingDistance[NeighborIndex] = CurrentDist + 1;
				BfsQueue.Add(NeighborIndex);
			}
		}
	}

	// --- Step 4: Classify conflict samples from resolved samples ---
	const bool bHasResolvedSamples = LastResolvedSamples.Num() == SampleCount;
	if (bHasResolvedSamples)
	{
		double RingDistanceSum = 0.0;
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
			const bool bMultiHit = Resolved.ExactCandidateCount > 1;
			const bool bMiss =
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous;
			const bool bDestructiveExclusion =
				Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion;

			const bool bConflict = bMultiHit || bMiss || bDestructiveExclusion;
			if (!bConflict)
			{
				continue;
			}

			const bool bOnBoundary = IsBoundaryBand[SampleIndex] != 0;
			const int32 Dist = RingDistance[SampleIndex];

			++Diag.TotalConflictSamples;
			if (Dist == 0)
			{
				++Diag.ConflictSamplesAtRing0;
			}
			else if (Dist == 1)
			{
				++Diag.ConflictSamplesAtRing1;
			}
			else
			{
				++Diag.ConflictSamplesAtRing2Plus;
			}
			RingDistanceSum += (Dist != INDEX_NONE) ? static_cast<double>(Dist) : 0.0;

			if (bMultiHit)
			{
				if (bOnBoundary)
				{
					++Diag.MultiHitOnBoundaryCount;
				}
				else
				{
					++Diag.MultiHitInteriorCount;
				}
			}

			if (bMiss)
			{
				if (bOnBoundary)
				{
					++Diag.MissOnBoundaryCount;
				}
				else
				{
					++Diag.MissInteriorCount;
				}
			}

			if (bDestructiveExclusion)
			{
				if (bOnBoundary)
				{
					++Diag.DestructiveExclusionOnBoundaryCount;
				}
				else
				{
					++Diag.DestructiveExclusionInteriorCount;
				}
			}

			if (!bOnBoundary)
			{
				++Diag.TotalInteriorConflictCount;
			}
		}

		if (Diag.TotalConflictSamples > 0)
		{
			Diag.MeanConflictRingDistance = RingDistanceSum / static_cast<double>(Diag.TotalConflictSamples);
			Diag.InteriorLeakageFraction =
				static_cast<double>(Diag.TotalInteriorConflictCount) /
				static_cast<double>(Diag.TotalConflictSamples);
		}
	}

	// --- Step 5: Composite score ---
	// Sub-score 1: Continuity — ratio of largest component to total boundary band
	const double ContinuityScore = (Diag.BoundaryBandSampleCount > 0)
		? static_cast<double>(Diag.LargestBoundaryBandComponentSize) /
			static_cast<double>(Diag.BoundaryBandSampleCount)
		: 0.0;

	// Sub-score 2: Narrowness — fraction of conflict at ring 0-1
	const double NarrownessScore = (Diag.TotalConflictSamples > 0)
		? static_cast<double>(Diag.ConflictSamplesAtRing0 + Diag.ConflictSamplesAtRing1) /
			static_cast<double>(Diag.TotalConflictSamples)
		: 1.0;

	// Sub-score 3: Interior quiet — 1 minus leakage fraction
	const double InteriorQuietScore = 1.0 - Diag.InteriorLeakageFraction;

	Diag.BoundaryCoherenceScore = (ContinuityScore + NarrownessScore + InteriorQuietScore) / 3.0;

	return Diag;
}

FTectonicPlanetV6OwnershipChurnDiagnostic FTectonicPlanetV6::ComputeOwnershipChurnDiagnosticForTest() const
{
	FTectonicPlanetV6OwnershipChurnDiagnostic Diag;
	const int32 SampleCount = Planet.Samples.Num();
	if (SampleCount == 0 || LastResolvedSamples.Num() != SampleCount ||
		Planet.SampleAdjacency.Num() != SampleCount)
	{
		return Diag;
	}

	// Build boundary band + ring distance (same approach as boundary coherence)
	TArray<uint8> IsBoundaryBand;
	IsBoundaryBand.Init(0, SampleCount);
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const int32 PlateId = Planet.Samples[SampleIndex].PlateId;
		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (Planet.Samples.IsValidIndex(NeighborIndex) &&
				Planet.Samples[NeighborIndex].PlateId != PlateId)
			{
				IsBoundaryBand[SampleIndex] = 1;
				break;
			}
		}
	}

	TArray<int32> RingDistance;
	RingDistance.Init(INDEX_NONE, SampleCount);
	TArray<int32> BfsQueue;
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		if (IsBoundaryBand[SampleIndex] != 0)
		{
			RingDistance[SampleIndex] = 0;
			BfsQueue.Add(SampleIndex);
		}
	}
	int32 BfsHead = 0;
	while (BfsHead < BfsQueue.Num())
	{
		const int32 Current = BfsQueue[BfsHead++];
		const int32 CurrentDist = RingDistance[Current];
		for (const int32 NeighborIndex : Planet.SampleAdjacency[Current])
		{
			if (Planet.Samples.IsValidIndex(NeighborIndex) && RingDistance[NeighborIndex] == INDEX_NONE)
			{
				RingDistance[NeighborIndex] = CurrentDist + 1;
				BfsQueue.Add(NeighborIndex);
			}
		}
	}

	// Count churn and collect plate-pair flows
	TMap<int64, int32> PlatePairFlowCounts;
	double RingDistanceSum = 0.0;
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		if (Resolved.PreviousPlateId == INDEX_NONE || Resolved.FinalPlateId == INDEX_NONE)
		{
			continue;
		}
		if (Resolved.FinalPlateId == Resolved.PreviousPlateId)
		{
			continue;
		}

		++Diag.TotalChurnCount;
		const int32 Dist = RingDistance[SampleIndex];
		if (IsBoundaryBand[SampleIndex] != 0)
		{
			++Diag.BoundaryBandChurnCount;
		}
		else
		{
			++Diag.InteriorChurnCount;
		}
		if (Dist == 0)
		{
			++Diag.ChurnAtRing0;
		}
		else if (Dist == 1)
		{
			++Diag.ChurnAtRing1;
		}
		else
		{
			++Diag.ChurnAtRing2Plus;
		}
		RingDistanceSum += (Dist != INDEX_NONE) ? static_cast<double>(Dist) : 0.0;

		const int64 PairKey = (static_cast<int64>(Resolved.PreviousPlateId) << 32) |
			static_cast<int64>(static_cast<uint32>(Resolved.FinalPlateId));
		++PlatePairFlowCounts.FindOrAdd(PairKey);
	}

	if (SampleCount > 0)
	{
		Diag.ChurnFraction = static_cast<double>(Diag.TotalChurnCount) / static_cast<double>(SampleCount);
	}
	if (Diag.TotalChurnCount > 0)
	{
		Diag.MeanChurnRingDistance = RingDistanceSum / static_cast<double>(Diag.TotalChurnCount);
	}

	// Top 3 flows
	TArray<TPair<int64, int32>> FlowArray;
	for (const TPair<int64, int32>& Pair : PlatePairFlowCounts)
	{
		FlowArray.Add(Pair);
	}
	FlowArray.Sort([](const TPair<int64, int32>& A, const TPair<int64, int32>& B)
	{
		return A.Value > B.Value;
	});
	for (int32 I = 0; I < FMath::Min(3, FlowArray.Num()); ++I)
	{
		Diag.TopFlows[I].FromPlateId = static_cast<int32>(FlowArray[I].Key >> 32);
		Diag.TopFlows[I].ToPlateId = static_cast<int32>(FlowArray[I].Key & 0xFFFFFFFF);
		Diag.TopFlows[I].Count = FlowArray[I].Value;
	}

	return Diag;
}

FTectonicPlanetV6CompetitiveParticipationDiagnostic FTectonicPlanetV6::ComputeCompetitiveParticipationDiagnosticForTest() const
{
	FTectonicPlanetV6CompetitiveParticipationDiagnostic Diag;
	const int32 SampleCount = Planet.Samples.Num();
	if (SampleCount == 0 || LastResolvedSamples.Num() != SampleCount)
	{
		return Diag;
	}

	Diag.TotalPlateCount = Planet.Plates.Num();

	// Count hits per plate
	TMap<int32, int32> HitCountsByPlate;
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		if (Resolved.ExactCandidateCount > 0 && Resolved.FinalPlateId != INDEX_NONE)
		{
			++HitCountsByPlate.FindOrAdd(Resolved.FinalPlateId);
			++Diag.TotalHitSamples;
		}
	}

	Diag.PlatesWithAnyHit = HitCountsByPlate.Num();

	for (const TPair<int32, int32>& Pair : HitCountsByPlate)
	{
		if (Pair.Value > Diag.LargestPlateHitCount)
		{
			Diag.LargestPlateHitCount = Pair.Value;
			Diag.LargestPlateId = Pair.Key;
		}
	}

	if (Diag.TotalHitSamples > 0)
	{
		Diag.LargestPlateHitShare =
			static_cast<double>(Diag.LargestPlateHitCount) / static_cast<double>(Diag.TotalHitSamples);

		const double OnePercentThreshold = static_cast<double>(Diag.TotalHitSamples) * 0.01;
		const double FivePercentThreshold = static_cast<double>(Diag.TotalHitSamples) * 0.05;
		for (const TPair<int32, int32>& Pair : HitCountsByPlate)
		{
			if (static_cast<double>(Pair.Value) >= OnePercentThreshold)
			{
				++Diag.PlatesAboveOnePercentThreshold;
			}
			if (static_cast<double>(Pair.Value) >= FivePercentThreshold)
			{
				++Diag.PlatesAboveFivePercentThreshold;
			}
		}
	}

	return Diag;
}

FTectonicPlanetV6MissLineageDiagnostic FTectonicPlanetV6::ComputeMissLineageDiagnosticForTest() const
{
	FTectonicPlanetV6MissLineageDiagnostic Diag;
	const int32 SampleCount = Planet.Samples.Num();
	if (SampleCount == 0 || LastResolvedSamples.Num() != SampleCount ||
		MissLineageCounts.Num() != SampleCount ||
		Planet.SampleAdjacency.Num() != SampleCount)
	{
		return Diag;
	}

	// Build boundary band for splitting
	TArray<uint8> IsBoundaryBand;
	IsBoundaryBand.Init(0, SampleCount);
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const int32 PlateId = Planet.Samples[SampleIndex].PlateId;
		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (Planet.Samples.IsValidIndex(NeighborIndex) &&
				Planet.Samples[NeighborIndex].PlateId != PlateId)
			{
				IsBoundaryBand[SampleIndex] = 1;
				break;
			}
		}
	}

	double DepthSum = 0.0;
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		const bool bMiss =
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous;
		if (!bMiss)
		{
			continue;
		}

		++Diag.MissesThisCycle;
		const uint8 Depth = MissLineageCounts[SampleIndex];
		DepthSum += static_cast<double>(Depth);
		const bool bOnBoundary = IsBoundaryBand[SampleIndex] != 0;

		if (Depth <= 1)
		{
			++Diag.FirstTimeSynthetic;
			if (bOnBoundary) { ++Diag.BoundaryMissFirstTime; }
			else { ++Diag.InteriorMissFirstTime; }
		}
		else if (Depth == 2)
		{
			++Diag.SecondTimeSynthetic;
			if (bOnBoundary) { ++Diag.BoundaryMissRepeat; }
			else { ++Diag.InteriorMissRepeat; }
		}
		else
		{
			++Diag.RepeatedSynthetic3Plus;
			if (bOnBoundary) { ++Diag.BoundaryMissRepeat; }
			else { ++Diag.InteriorMissRepeat; }
		}
	}

	if (Diag.MissesThisCycle > 0)
	{
		Diag.MeanResynthesisDepth = DepthSum / static_cast<double>(Diag.MissesThisCycle);
	}

	return Diag;
}

FTectonicPlanetV6InteriorInstabilityDiagnostic FTectonicPlanetV6::ComputeInteriorInstabilityDiagnosticForTest() const
{
	FTectonicPlanetV6InteriorInstabilityDiagnostic Diag;
	const int32 SampleCount = Planet.Samples.Num();
	if (SampleCount == 0 ||
		LastResolvedSamples.Num() != SampleCount ||
		Planet.SampleAdjacency.Num() != SampleCount ||
		MissLineageCounts.Num() != SampleCount ||
		CurrentSolvePreviousSyntheticFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerFilteredHitFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerUnfilteredHitFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerRecoveryFlags.Num() != SampleCount ||
		CurrentSolveLocalParticipationPlateCounts.Num() != SampleCount)
	{
		return Diag;
	}

	TArray<uint8> IsBoundaryBand;
	TArray<int32> RingDistance;
	BuildBoundaryBandAndRingDistance(Planet, IsBoundaryBand, RingDistance);

	const FTectonicPlanetV6CompetitiveParticipationDiagnostic Participation =
		ComputeCompetitiveParticipationDiagnosticForTest();
	TMap<int64, int32> InteriorFlowCounts;

	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const int32 Dist = RingDistance[SampleIndex];
		if (Dist != INDEX_NONE && Dist < Diag.InteriorRingThreshold)
		{
			continue;
		}

		const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		const bool bMiss =
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous;
		const bool bCurrentSynthetic =
			IsSyntheticTransferSourceKind(Resolved.TransferDebug.SourceKind);
		const bool bPreviousSynthetic = CurrentSolvePreviousSyntheticFlags[SampleIndex] != 0;
		const bool bPreviousOwnerFilteredHit =
			CurrentSolvePreviousOwnerFilteredHitFlags[SampleIndex] != 0;
		const bool bPreviousOwnerUnfilteredHit =
			CurrentSolvePreviousOwnerUnfilteredHitFlags[SampleIndex] != 0;
		const bool bPreviousOwnerRecovery =
			CurrentSolvePreviousOwnerRecoveryFlags[SampleIndex] != 0;
		const int32 LocalParticipationCount =
			static_cast<int32>(CurrentSolveLocalParticipationPlateCounts[SampleIndex]);

		bool bPreviousOwnerNeighborHit = false;
		bool bOtherPlateNeighborHit = false;
		bool bNearRepeatResynthesis = MissLineageCounts[SampleIndex] >= 2;
		if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!LastResolvedSamples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				const FTectonicPlanetV6ResolvedSample& NeighborResolved = LastResolvedSamples[NeighborIndex];
				if (NeighborResolved.ExactCandidateCount > 0 && NeighborResolved.FinalPlateId != INDEX_NONE)
				{
					if (NeighborResolved.FinalPlateId == Resolved.PreviousPlateId)
					{
						bPreviousOwnerNeighborHit = true;
					}
					else
					{
						bOtherPlateNeighborHit = true;
					}
				}

				if (MissLineageCounts.IsValidIndex(NeighborIndex) &&
					MissLineageCounts[NeighborIndex] >= 2)
				{
					bNearRepeatResynthesis = true;
				}
			}
		}

		if (bPreviousSynthetic)
		{
			++Diag.InteriorPreviousSyntheticSampleCount;
			if (bMiss)
			{
				++Diag.InteriorPreviousSyntheticMissedAgainCount;
			}
			else if (Resolved.FinalPlateId == Resolved.PreviousPlateId)
			{
				++Diag.InteriorPreviousSyntheticSurvivedUnchangedCount;
			}
			else
			{
				++Diag.InteriorPreviousSyntheticReassignedCount;
			}
		}

		if (bMiss)
		{
			++Diag.InteriorMissCount;

			if (!bPreviousOwnerNeighborHit &&
				!bPreviousOwnerRecovery &&
				!bPreviousOwnerUnfilteredHit)
			{
				++Diag.InteriorMissPreviousOwnerNoLocalCoverageApproxCount;
			}
			if (!bPreviousOwnerNeighborHit && bOtherPlateNeighborHit)
			{
				++Diag.InteriorMissOtherPlateValidHitNearbyApproxCount;
			}
			if (bPreviousSynthetic)
			{
				++Diag.InteriorMissPreviousSyntheticRepeatCount;
			}
			else
			{
				++Diag.InteriorMissPreviousNaturalCount;
			}

			if (bPreviousOwnerUnfilteredHit && !bPreviousOwnerFilteredHit)
			{
				++Diag.InteriorMissPreviousOwnerGeometryRejectedCount;
			}
			else if (bPreviousOwnerRecovery)
			{
				++Diag.InteriorMissPreviousOwnerGeometryNearbyRecoveryCount;
			}
			else
			{
				++Diag.InteriorMissPreviousOwnerGeometryAbsentCount;
			}

			if (bPreviousOwnerNeighborHit)
			{
				++Diag.InteriorMissPreviousOwnerNeighborHitCount;
			}
			if (LocalParticipationCount <= 1)
			{
				++Diag.InteriorMissSinglePlateParticipationCount;
			}
			else
			{
				++Diag.InteriorMissMultiPlateParticipationCount;
			}
			if (Resolved.FinalPlateId != INDEX_NONE &&
				Participation.LargestPlateId != INDEX_NONE &&
				Resolved.FinalPlateId == Participation.LargestPlateId)
			{
				++Diag.InteriorMissAssignedToLargestParticipationPlateCount;
			}
		}

		if (Resolved.PreviousPlateId == INDEX_NONE ||
			Resolved.FinalPlateId == INDEX_NONE ||
			Resolved.FinalPlateId == Resolved.PreviousPlateId)
		{
			continue;
		}

		++Diag.InteriorChurnCount;
		if (bMiss)
		{
			++Diag.InteriorChurnPreviousOwnerNowMissingCount;
		}
		else
		{
			++Diag.InteriorChurnValidOwnerToValidOwnerCount;
		}

		if (bPreviousOwnerFilteredHit)
		{
			++Diag.InteriorChurnPreviousOwnerStillHadValidHitCount;
		}
		else if (!bMiss)
		{
			++Diag.InteriorChurnValidOwnerWithoutPreviousOwnerHitCount;
		}

		if (bPreviousSynthetic)
		{
			++Diag.InteriorChurnPreviousSyntheticCount;
		}
		if (bCurrentSynthetic)
		{
			++Diag.InteriorChurnCurrentSyntheticCount;
		}
		if (bPreviousSynthetic || bCurrentSynthetic)
		{
			++Diag.InteriorChurnEitherSyntheticCount;
		}
		if (bNearRepeatResynthesis)
		{
			++Diag.InteriorChurnNearRepeatResynthesisCount;
		}
		if (bPreviousOwnerNeighborHit)
		{
			++Diag.InteriorChurnPreviousOwnerNeighborHitCount;
		}
		if (LocalParticipationCount <= 1)
		{
			++Diag.InteriorChurnSinglePlateParticipationCount;
		}
		else
		{
			++Diag.InteriorChurnMultiPlateParticipationCount;
		}
		if (Participation.LargestPlateId != INDEX_NONE &&
			Resolved.FinalPlateId == Participation.LargestPlateId)
		{
			++Diag.InteriorChurnAbsorbedByLargestParticipationPlateCount;
		}

		const int64 PairKey =
			(static_cast<int64>(Resolved.PreviousPlateId) << 32) |
			static_cast<int64>(static_cast<uint32>(Resolved.FinalPlateId));
		++InteriorFlowCounts.FindOrAdd(PairKey);
	}

	TArray<TPair<int64, int32>> FlowArray;
	for (const TPair<int64, int32>& Pair : InteriorFlowCounts)
	{
		FlowArray.Add(Pair);
	}
	FlowArray.Sort([](const TPair<int64, int32>& A, const TPair<int64, int32>& B)
	{
		return A.Value > B.Value;
	});
	for (int32 Index = 0; Index < FMath::Min(3, FlowArray.Num()); ++Index)
	{
		Diag.TopInteriorFlows[Index].FromPlateId = static_cast<int32>(FlowArray[Index].Key >> 32);
		Diag.TopInteriorFlows[Index].ToPlateId = static_cast<int32>(FlowArray[Index].Key & 0xFFFFFFFF);
		Diag.TopInteriorFlows[Index].Count = FlowArray[Index].Value;
	}

	return Diag;
}

FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic
FTectonicPlanetV6::ComputeSyntheticCoveragePersistenceDiagnosticForTest() const
{
	FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic Diag;
	const int32 SampleCount = Planet.Samples.Num();
	if (SampleCount == 0 ||
		LastResolvedSamples.Num() != SampleCount ||
		CurrentSolvePreviousSyntheticFlags.Num() != SampleCount ||
		CurrentSolvePreviousRetainedSyntheticCoverageFlags.Num() != SampleCount ||
		CurrentSolveRetainedSyntheticCoverageFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerFilteredHitFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerUnfilteredHitFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerIgnoringBoundingCapHitFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerRecoveryFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags.Num() != SampleCount ||
		CurrentSolvePreviousOwnerAdjacentTriangleNearestDistanceKm.Num() != SampleCount ||
		CurrentSolvePreviousOwnerAdjacentTriangleSupportCounts.Num() != SampleCount ||
		CurrentSolvePreviousOwnerAdjacentTriangleMixedSupportCounts.Num() != SampleCount ||
		CurrentSolvePreviousOwnerAdjacentTriangleMinoritySupportCounts.Num() != SampleCount)
	{
		return Diag;
	}

	TArray<uint8> IsBoundaryBand;
	TArray<int32> RingDistance;
	BuildBoundaryBandAndRingDistance(Planet, IsBoundaryBand, RingDistance);
	const FTectonicPlanetV6CompetitiveParticipationDiagnostic Participation =
		ComputeCompetitiveParticipationDiagnosticForTest();

	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		if (CurrentSolvePreviousSyntheticFlags[SampleIndex] == 0)
		{
			continue;
		}

		++Diag.PreviousSyntheticSampleCount;
		if (!RingDistance.IsValidIndex(SampleIndex) ||
			RingDistance[SampleIndex] == INDEX_NONE ||
			RingDistance[SampleIndex] >= 2)
		{
			++Diag.PreviousSyntheticInteriorSampleCount;
		}

		const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		const bool bMiss =
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous;
		const bool bPreviousRetainedSyntheticCoverage =
			CurrentSolvePreviousRetainedSyntheticCoverageFlags[SampleIndex] != 0;
		const bool bCurrentRetainedSyntheticCoverage =
			CurrentSolveRetainedSyntheticCoverageFlags[SampleIndex] != 0;
		const bool bPreviousOwnerExactHit =
			CurrentSolvePreviousOwnerFilteredHitFlags[SampleIndex] != 0;
		const bool bPreviousOwnerIgnoringCapHit =
			CurrentSolvePreviousOwnerIgnoringBoundingCapHitFlags[SampleIndex] != 0;
		const bool bPreviousOwnerRecovery =
			CurrentSolvePreviousOwnerRecoveryFlags[SampleIndex] != 0;
			const bool bPreviousOwnerCanonicalVertexInMesh =
				CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags[SampleIndex] != 0;
			const bool bPreviousOwnerAdjacentTriangleInMesh =
				CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags[SampleIndex] != 0;
			const double PreviousOwnerAdjacentTriangleNearestDistanceKm =
				static_cast<double>(CurrentSolvePreviousOwnerAdjacentTriangleNearestDistanceKm[SampleIndex]);
			const int32 PreviousOwnerAdjacentTriangleSupportCount =
				static_cast<int32>(CurrentSolvePreviousOwnerAdjacentTriangleSupportCounts[SampleIndex]);
			const int32 PreviousOwnerAdjacentTriangleMixedSupportCount =
				static_cast<int32>(CurrentSolvePreviousOwnerAdjacentTriangleMixedSupportCounts[SampleIndex]);
			const int32 PreviousOwnerAdjacentTriangleMinoritySupportCount =
				static_cast<int32>(CurrentSolvePreviousOwnerAdjacentTriangleMinoritySupportCounts[SampleIndex]);
			const bool bPreviousOwnerGeometryNearby =
				bPreviousOwnerExactHit ||
				bPreviousOwnerIgnoringCapHit ||
			bPreviousOwnerRecovery ||
			bPreviousOwnerAdjacentTriangleInMesh ||
			bPreviousOwnerCanonicalVertexInMesh;

		if (bCurrentRetainedSyntheticCoverage)
		{
			++Diag.CurrentRetainedSyntheticCoverageCount;
		}
		if (bPreviousRetainedSyntheticCoverage)
		{
			++Diag.PreviousRetainedSyntheticCoverageSampleCount;
		}

		if (bPreviousOwnerExactHit)
		{
			++Diag.PreviousOwnerExactHitCount;
		}
		if (bPreviousOwnerIgnoringCapHit)
		{
			++Diag.PreviousOwnerIgnoringBoundingCapHitCount;
		}
		if (bPreviousOwnerRecovery)
		{
			++Diag.PreviousOwnerRecoveryCoverageCount;
		}
		if (bPreviousOwnerCanonicalVertexInMesh)
		{
			++Diag.PreviousOwnerCanonicalVertexInMeshCount;
		}
			if (bPreviousOwnerAdjacentTriangleInMesh)
			{
				++Diag.PreviousOwnerAdjacentTriangleInMeshCount;
				Diag.PreviousOwnerAdjacentTriangleTotalSupportCount += PreviousOwnerAdjacentTriangleSupportCount;
				Diag.PreviousOwnerAdjacentTriangleTotalMixedSupportCount += PreviousOwnerAdjacentTriangleMixedSupportCount;
				Diag.PreviousOwnerAdjacentTriangleTotalMinoritySupportCount += PreviousOwnerAdjacentTriangleMinoritySupportCount;
				if (PreviousOwnerAdjacentTriangleMixedSupportCount > 0)
				{
					++Diag.PreviousOwnerAdjacentTriangleAnyMixedSupportSampleCount;
				}
				if (PreviousOwnerAdjacentTriangleSupportCount > 0 &&
					PreviousOwnerAdjacentTriangleMixedSupportCount == PreviousOwnerAdjacentTriangleSupportCount)
				{
					++Diag.PreviousOwnerAdjacentTriangleAllMixedSupportSampleCount;
				}
				if (PreviousOwnerAdjacentTriangleSupportCount > 0 &&
					PreviousOwnerAdjacentTriangleMinoritySupportCount == PreviousOwnerAdjacentTriangleSupportCount)
				{
					++Diag.PreviousOwnerAdjacentTriangleAllMinoritySupportSampleCount;
				}
			}
			if (bPreviousOwnerCanonicalVertexInMesh && !bPreviousOwnerAdjacentTriangleInMesh)
			{
				++Diag.PreviousOwnerVertexButNoAdjacentTriangleCount;
			}
			if (bPreviousOwnerAdjacentTriangleInMesh && !bPreviousOwnerExactHit)
			{
				++Diag.PreviousOwnerAdjacentTriangleButNoExactHitCount;
				if (PreviousOwnerAdjacentTriangleMixedSupportCount > 0)
				{
					++Diag.PreviousOwnerAdjacentTriangleNoExactHitAnyMixedSupportSampleCount;
				}
				if (PreviousOwnerAdjacentTriangleSupportCount > 0 &&
					PreviousOwnerAdjacentTriangleMixedSupportCount == PreviousOwnerAdjacentTriangleSupportCount)
				{
					++Diag.PreviousOwnerAdjacentTriangleNoExactHitAllMixedSupportSampleCount;
				}
				if (PreviousOwnerAdjacentTriangleSupportCount > 0 &&
					PreviousOwnerAdjacentTriangleMinoritySupportCount == PreviousOwnerAdjacentTriangleSupportCount)
				{
					++Diag.PreviousOwnerAdjacentTriangleNoExactHitAllMinoritySupportSampleCount;
				}

				if (PreviousOwnerAdjacentTriangleNearestDistanceKm < 0.0)
				{
					++Diag.PreviousOwnerAdjacentTriangleNoDistanceCount;
				}
				else if (PreviousOwnerAdjacentTriangleNearestDistanceKm <=
					FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic::VeryNearCoverageThresholdKm)
				{
					++Diag.PreviousOwnerAdjacentTriangleVeryNearMissCount;
					if (PreviousOwnerAdjacentTriangleSupportCount > 0 &&
						PreviousOwnerAdjacentTriangleMinoritySupportCount == PreviousOwnerAdjacentTriangleSupportCount)
					{
						++Diag.PreviousOwnerAdjacentTriangleVeryNearMissAllMinoritySupportCount;
					}
				}
				else if (PreviousOwnerAdjacentTriangleNearestDistanceKm <=
					FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic::ModerateCoverageThresholdKm)
				{
					++Diag.PreviousOwnerAdjacentTriangleModerateMissCount;
				}
				else
				{
					++Diag.PreviousOwnerAdjacentTriangleTrueLocalHoleCount;
					if (PreviousOwnerAdjacentTriangleSupportCount > 0 &&
						PreviousOwnerAdjacentTriangleMinoritySupportCount == PreviousOwnerAdjacentTriangleSupportCount)
					{
						++Diag.PreviousOwnerAdjacentTriangleTrueLocalHoleAllMinoritySupportCount;
					}
				}
			}
		if (bPreviousOwnerAdjacentTriangleInMesh &&
			bPreviousOwnerIgnoringCapHit &&
			!bPreviousOwnerExactHit)
		{
			++Diag.PreviousOwnerAdjacentTriangleButIgnoringCapHitCount;
		}

		if (bMiss)
		{
			++Diag.MissCount;
			if (bPreviousRetainedSyntheticCoverage)
			{
				++Diag.PreviousRetainedSyntheticCoverageMissCount;
			}
			if (bPreviousOwnerGeometryNearby)
			{
				++Diag.MissWithPreviousOwnerGeometryNearbyCount;
			}
			else
			{
				++Diag.MissWithNoPreviousOwnerGeometryNearbyCount;
			}

			if (bPreviousOwnerAdjacentTriangleInMesh)
			{
				++Diag.MissWithPreviousOwnerAdjacentTrianglePresentCount;
			}
			else
			{
				++Diag.MissWithPreviousOwnerAdjacentTriangleAbsentCount;
			}
			continue;
		}

		if (Resolved.FinalPlateId == Resolved.PreviousPlateId)
		{
			++Diag.SameOwnerValidHitCount;
			if (bPreviousRetainedSyntheticCoverage)
			{
				++Diag.PreviousRetainedSyntheticCoverageStabilizedSameOwnerCount;
			}
		}
		else
		{
			++Diag.DifferentOwnerValidHitCount;
			if (bPreviousRetainedSyntheticCoverage)
			{
				++Diag.PreviousRetainedSyntheticCoverageReassignedCount;
			}
			if (bPreviousOwnerExactHit)
			{
				++Diag.DifferentOwnerValidHitWhilePreviousOwnerStillHadExactHitCount;
			}
			else if (bPreviousOwnerRecovery)
			{
				++Diag.DifferentOwnerValidHitWhilePreviousOwnerOnlyHadRecoveryCount;
			}

			if (Participation.LargestPlateId != INDEX_NONE &&
				Resolved.FinalPlateId == Participation.LargestPlateId)
			{
				++Diag.DifferentOwnerValidHitAbsorbedByLargestParticipationPlateCount;
			}
		}
	}

	return Diag;
}

FTectonicPlanetV6FrontRetreatDiagnostic FTectonicPlanetV6::ComputeFrontRetreatDiagnosticForTest() const
{
	FTectonicPlanetV6FrontRetreatDiagnostic Diag;

	// Current subduction state
	double SubductionSum = 0.0;
	int32 SubductionCount = 0;
	for (const FSample& Sample : Planet.Samples)
	{
		if (Sample.SubductionDistanceKm >= 0.0f)
		{
			SubductionSum += static_cast<double>(Sample.SubductionDistanceKm);
			++SubductionCount;
		}
	}

	Diag.CurrentSubductionSampleCount = SubductionCount;
	Diag.CurrentMeanSubductionDistanceKm =
		SubductionCount > 0 ? SubductionSum / static_cast<double>(SubductionCount) : 0.0;

	// Previous state from persistent snapshot
	Diag.PreviousMeanSubductionDistanceKm = PreviousIntervalMeanSubductionDistanceKm;
	Diag.PreviousSubductionSampleCount = PreviousIntervalSubductionSampleCount;

	if (PreviousIntervalMeanSubductionDistanceKm >= 0.0 && SubductionCount > 0)
	{
		Diag.DeltaMeanSubductionDistanceKm =
			Diag.CurrentMeanSubductionDistanceKm - PreviousIntervalMeanSubductionDistanceKm;
	}

	// Tracked triangle count delta
	Diag.TrackedTriangleCount = LastSolveStats.TrackedDestructiveTriangleCount;
	Diag.PreviousTrackedTriangleCount = PreviousIntervalTrackedTriangleCount;
	Diag.DeltaTrackedTriangleCount =
		Diag.TrackedTriangleCount - PreviousIntervalTrackedTriangleCount;

	return Diag;
}

FTectonicPlanetV6OwnerCoverageAudit FTectonicPlanetV6::ComputeCurrentOwnerCoverageAuditForTest() const
{
	FTectonicPlanetV6OwnerCoverageAudit Audit;
	TArray<FV6PlateQueryGeometry> QueryGeometries;

	if (IsCopiedFrontierLikeSolveMode(PeriodicSolveMode))
	{
		const bool bEnableStructuredGapFill =
			UsesStructuredGapFillForCopiedFrontierSolveConfiguration(
				PeriodicSolveMode,
				bForceWholeTriangleBoundaryDuplicationForTest,
				bForceExcludeMixedTrianglesForTest);
		const EV6ThesisFrontierMeshBuildMode FrontierMeshBuildMode =
			GetEffectiveThesisFrontierMeshBuildMode(
				PeriodicSolveMode,
				bForceWholeTriangleBoundaryDuplicationForTest,
				bForceExcludeMixedTrianglesForTest);
		const TArray<uint8>* SyntheticCoverageSupportFlags =
			bEnableStructuredGapFill &&
			PreviousSolveSyntheticFlags.Num() == Planet.Samples.Num()
				? &PreviousSolveSyntheticFlags
				: nullptr;

		TArray<FTectonicPlanetV6CopiedFrontierPlateMesh> PlateMeshes;
		BuildV6CopiedFrontierPlateMeshes(
			Planet,
			PlateMeshes,
			nullptr,
			FrontierMeshBuildMode,
			SyntheticCoverageSupportFlags);
		BuildV6CopiedFrontierQueryGeometries(Planet, PlateMeshes, QueryGeometries, nullptr);
	}
	else if (PeriodicSolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike)
	{
		TArray<FTectonicPlanetV6PlateSubmesh> PlateMeshes;
		BuildV6PlateSubmeshMeshes(Planet, PlateMeshes);
		BuildV6PlateSubmeshQueryGeometries(Planet, PlateMeshes, QueryGeometries);
	}
	else if (PeriodicSolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike)
	{
		BuildV6ThesisRemeshQueryGeometries(Planet, QueryGeometries);
	}
	else
	{
		BuildV6QueryGeometries(Planet, QueryGeometries);
	}

	TMap<int32, int32> QueryGeometryIndexByPlateId;
	for (int32 PlateIndex = 0; PlateIndex < QueryGeometries.Num(); ++PlateIndex)
	{
		if (QueryGeometries[PlateIndex].PlateId != INDEX_NONE)
		{
			QueryGeometryIndexByPlateId.Add(QueryGeometries[PlateIndex].PlateId, PlateIndex);
		}
	}

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		if (!Planet.Samples.IsValidIndex(SampleIndex))
		{
			continue;
		}

		const FSample& Sample = Planet.Samples[SampleIndex];
		if (Sample.PlateId == INDEX_NONE)
		{
			continue;
		}

		++Audit.AssignedSampleCount;
		if (!Sample.bIsBoundary)
		{
			++Audit.AssignedInteriorSampleCount;
		}

		bool bOwnerQueryHit = false;
		bool bOwnerQueryIgnoringCapHit = false;
		if (const int32* QueryGeometryIndexPtr = QueryGeometryIndexByPlateId.Find(Sample.PlateId))
		{
			const FV6PlateQueryGeometry& QueryGeometry = QueryGeometries[*QueryGeometryIndexPtr];
			FV6ThesisRemeshRayHit Hit;
			bOwnerQueryHit = TryFindThesisRemeshRayHit(QueryGeometry, Sample.Position, Hit);
			bOwnerQueryIgnoringCapHit =
				TryFindThesisRemeshHitIgnoringBoundingCap(QueryGeometry, Sample.Position, Hit);
		}

		if (bOwnerQueryHit)
		{
			++Audit.OwnerQueryExactHitCount;
		}
		else
		{
			++Audit.OwnerQueryMissCount;
			if (!Sample.bIsBoundary && Audit.ExampleInteriorQueryMissSampleIndices.Num() < 8)
			{
				Audit.ExampleInteriorQueryMissSampleIndices.Add(SampleIndex);
			}
		}

		if (bOwnerQueryIgnoringCapHit)
		{
			++Audit.OwnerQueryIgnoringCapHitCount;
		}
		else
		{
			++Audit.OwnerQueryMissIgnoringCapCount;
		}

		bool bOwnerSoupContains = false;
		if (const FPlate* OwnerPlate = FindPlateById(Planet, Sample.PlateId))
		{
			int32 LocalTriangleId = INDEX_NONE;
			FVector3d A;
			FVector3d B;
			FVector3d C;
			bOwnerSoupContains =
				FindContainingTriangleInBVH(
					OwnerPlate->SoupBVH,
					OwnerPlate->SoupAdapter,
					Sample.Position,
					LocalTriangleId,
					A,
					B,
					C);
		}

		if (bOwnerSoupContains)
		{
			++Audit.OwnerSoupContainmentCount;
		}
		else
		{
			++Audit.OwnerSoupMissCount;
		}

		if (bOwnerSoupContains && !bOwnerQueryHit)
		{
			++Audit.OwnerSoupContainmentButQueryMissCount;
			if (!Sample.bIsBoundary &&
				Audit.ExampleInteriorSoupContainedButQueryMissSampleIndices.Num() < 8)
			{
				Audit.ExampleInteriorSoupContainedButQueryMissSampleIndices.Add(SampleIndex);
			}
		}

		if (bOwnerSoupContains && !bOwnerQueryIgnoringCapHit)
		{
			++Audit.OwnerSoupContainmentButQueryMissIgnoringCapCount;
		}
	}

	return Audit;
}

FTectonicPlanetV6ActiveZoneDiagnostic FTectonicPlanetV6::ComputeActiveZoneDiagnosticForTest() const
{
	FTectonicPlanetV6ActiveZoneDiagnostic Diag;
	const int32 SampleCount = Planet.Samples.Num();
	if (SampleCount == 0 ||
		LastResolvedSamples.Num() != SampleCount ||
		CurrentSolveActiveZoneFlags.Num() != SampleCount ||
		CurrentSolveActiveZoneSeedFlags.Num() != SampleCount ||
		CurrentSolveActiveZoneCarryoverFlags.Num() != SampleCount ||
		CurrentSolveActiveZoneFreshExpandedFlags.Num() != SampleCount ||
		CurrentSolveActiveZoneCarryoverExpandedFlags.Num() != SampleCount ||
		CurrentSolveActiveZoneCauseValues.Num() != SampleCount ||
		CurrentSolveOutsideActiveZoneQueryMissFlags.Num() != SampleCount ||
		CurrentSolveOutsideActiveZoneCoverageDeficitFlags.Num() != SampleCount)
	{
		return Diag;
	}

	Diag.ActivePairCount = CurrentSolveActivePairCount;
	Diag.FreshSeedActivePairCount = CurrentSolveFreshSeedActivePairCount;
	Diag.PersistentCarryoverActivePairCount = CurrentSolvePersistentCarryoverActivePairCount;
	Diag.FreshPairCandidateCount = CurrentSolveFreshPairCandidateCount;
	Diag.FreshPairAdmittedCount = CurrentSolveFreshPairAdmittedCount;
	Diag.FreshPairRejectedSupportCount = CurrentSolveFreshPairRejectedSupportCount;
	Diag.FreshPairRejectedVelocityCount = CurrentSolveFreshPairRejectedVelocityCount;
	Diag.FreshPairRejectedDominanceCount = CurrentSolveFreshPairRejectedDominanceCount;
	Diag.FreshPairAdmittedDivergenceCount = CurrentSolveFreshPairAdmittedDivergenceCount;
	Diag.FreshPairAdmittedConvergentSubductionCount =
		CurrentSolveFreshPairAdmittedConvergentSubductionCount;
	Diag.FreshPairAdmittedCollisionContactCount =
		CurrentSolveFreshPairAdmittedCollisionContactCount;
	Diag.FreshPairAdmittedRiftCount = CurrentSolveFreshPairAdmittedRiftCount;

	TArray<uint8> IsBoundaryBand;
	TArray<int32> RingDistance;
	BuildBoundaryBandAndRingDistance(Planet, IsBoundaryBand, RingDistance);

	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const bool bActive = CurrentSolveActiveZoneFlags[SampleIndex] != 0;
		const bool bSeed = CurrentSolveActiveZoneSeedFlags[SampleIndex] != 0;
		const ETectonicPlanetV6ActiveZoneCause ActiveCause =
			static_cast<ETectonicPlanetV6ActiveZoneCause>(CurrentSolveActiveZoneCauseValues[SampleIndex]);
		const bool bOutsideZoneQueryMiss = CurrentSolveOutsideActiveZoneQueryMissFlags[SampleIndex] != 0;
		const bool bOutsideZoneCoverageDeficit =
			CurrentSolveOutsideActiveZoneCoverageDeficitFlags[SampleIndex] != 0;

		if (bActive)
		{
			++Diag.ActiveSampleCount;
			if (IsBoundaryBand.IsValidIndex(SampleIndex) && IsBoundaryBand[SampleIndex] != 0)
			{
				++Diag.ActiveBoundaryBandCount;
			}
			else
			{
				++Diag.ActiveInteriorCount;
			}
			switch (ActiveCause)
			{
			case ETectonicPlanetV6ActiveZoneCause::DivergenceFill:
				++Diag.ActiveDivergenceSampleCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction:
				++Diag.ActiveConvergentSubductionSampleCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::CollisionContact:
				++Diag.ActiveCollisionContactSampleCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::Rift:
				++Diag.ActiveRiftSampleCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::GenericQueryCompetition:
				++Diag.ActiveGenericQueryCompetitionSampleCount;
				break;
			case ETectonicPlanetV6ActiveZoneCause::UnknownOther:
			case ETectonicPlanetV6ActiveZoneCause::None:
			default:
				++Diag.ActiveUnknownOtherSampleCount;
				break;
			}
		}
		else
		{
			++Diag.InactiveSampleCount;
		}

		if (bSeed)
		{
			++Diag.ActiveSeedSampleCount;
		}
		if (CurrentSolveActiveZoneCarryoverFlags[SampleIndex] != 0)
		{
			++Diag.ActiveCarryoverSampleCount;
		}
		if (CurrentSolveActiveZoneFreshExpandedFlags[SampleIndex] != 0)
		{
			++Diag.FreshExpandedSampleCount;
		}
		if (CurrentSolveActiveZoneCarryoverExpandedFlags[SampleIndex] != 0)
		{
			++Diag.CarryoverExpandedSampleCount;
		}
		if (bOutsideZoneQueryMiss)
		{
			++Diag.OutsideZoneQueryMissCount;
		}
		if (bOutsideZoneCoverageDeficit)
		{
			++Diag.OutsideZoneCoverageDeficitCount;
		}

		const FTectonicPlanetV6ResolvedSample& Resolved = LastResolvedSamples[SampleIndex];
		if (Resolved.PreviousPlateId == INDEX_NONE ||
			Resolved.FinalPlateId == INDEX_NONE ||
			Resolved.FinalPlateId == Resolved.PreviousPlateId)
		{
			continue;
		}

		if (bActive)
		{
			++Diag.OwnershipChangesInsideActiveZone;
		}
		else
		{
			++Diag.OwnershipChangesOutsideActiveZone;
		}

		ETectonicPlanetV6ActiveZoneCause ChangeCause = Resolved.ActiveZoneCause;
		const bool bPairScopedChange =
			(Resolved.ActiveZonePrimaryPlateId != INDEX_NONE || Resolved.ActiveZoneSecondaryPlateId != INDEX_NONE) &&
			(Resolved.PreviousPlateId == INDEX_NONE ||
				Resolved.PreviousPlateId == Resolved.ActiveZonePrimaryPlateId ||
				Resolved.PreviousPlateId == Resolved.ActiveZoneSecondaryPlateId) &&
			(Resolved.FinalPlateId == Resolved.ActiveZonePrimaryPlateId ||
				Resolved.FinalPlateId == Resolved.ActiveZoneSecondaryPlateId);
		const bool bExplicitDivergentFill =
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
			Resolved.bHasStructuredSyntheticFill ||
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::StructuredSynthetic ||
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::OceanicCreation;
		const bool bGenericCompetitionChange =
			Resolved.ExactCandidateCount > 0 &&
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshHit;

		if (bExplicitDivergentFill)
		{
			ChangeCause = ETectonicPlanetV6ActiveZoneCause::DivergenceFill;
		}
		else if (bPairScopedChange &&
			(ChangeCause == ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction ||
				ChangeCause == ETectonicPlanetV6ActiveZoneCause::CollisionContact ||
				ChangeCause == ETectonicPlanetV6ActiveZoneCause::Rift ||
				ChangeCause == ETectonicPlanetV6ActiveZoneCause::DivergenceFill))
		{
			// Keep explicit tectonic attribution when the reassignment stayed within the active plate pair.
		}
		else if (bGenericCompetitionChange)
		{
			ChangeCause = ETectonicPlanetV6ActiveZoneCause::GenericQueryCompetition;
		}
		else if (ChangeCause == ETectonicPlanetV6ActiveZoneCause::None)
		{
			ChangeCause = ETectonicPlanetV6ActiveZoneCause::UnknownOther;
		}

		switch (ChangeCause)
		{
		case ETectonicPlanetV6ActiveZoneCause::DivergenceFill:
			++Diag.OwnershipChangeDivergenceFillCount;
			break;
		case ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction:
			++Diag.OwnershipChangeConvergentSubductionCount;
			break;
		case ETectonicPlanetV6ActiveZoneCause::CollisionContact:
			++Diag.OwnershipChangeCollisionContactCount;
			break;
		case ETectonicPlanetV6ActiveZoneCause::Rift:
			++Diag.OwnershipChangeRiftCount;
			break;
		case ETectonicPlanetV6ActiveZoneCause::GenericQueryCompetition:
			++Diag.OwnershipChangeGenericQueryCompetitionCount;
			break;
		case ETectonicPlanetV6ActiveZoneCause::UnknownOther:
		case ETectonicPlanetV6ActiveZoneCause::None:
		default:
			++Diag.OwnershipChangeUnknownOtherCount;
			break;
		}
	}

	if (SampleCount > 0)
	{
		Diag.ActiveFraction = static_cast<double>(Diag.ActiveSampleCount) /
			static_cast<double>(SampleCount);
	}

	return Diag;
}

FTectonicPlanetV6CollisionShadowDiagnostic FTectonicPlanetV6::ComputeCollisionShadowDiagnosticForTest() const
{
	return CurrentSolveCollisionShadowDiagnostic;
}

FTectonicPlanetV6CollisionExecutionDiagnostic FTectonicPlanetV6::ComputeCollisionExecutionDiagnosticForTest() const
{
	return CurrentSolveCollisionExecutionDiagnostic;
}

FTectonicPlanetV6RiftDiagnostic FTectonicPlanetV6::ComputeRiftDiagnosticForTest() const
{
	FTectonicPlanetV6RiftDiagnostic Diag = CurrentSolveRiftDiagnostic;
	Diag.CumulativeRiftCount = V9RiftCumulativeCount;
	Diag.bTriggeredThisSolve =
		LastSolveStats.Trigger == ETectonicPlanetV6SolveTrigger::Rift &&
		LastSolveStats.Step == Diag.Step;
	Diag.PostRiftPlateCount = Planet.Plates.Num();
	Diag.bParentPlateStillPresent =
		Diag.ParentPlateId != INDEX_NONE &&
		Planet.FindPlateArrayIndexById(Diag.ParentPlateId) != INDEX_NONE;

	const int32 ChildPlateAIndex =
		Diag.ChildPlateA != INDEX_NONE ? Planet.FindPlateArrayIndexById(Diag.ChildPlateA) : INDEX_NONE;
	const int32 ChildPlateBIndex =
		Diag.ChildPlateB != INDEX_NONE ? Planet.FindPlateArrayIndexById(Diag.ChildPlateB) : INDEX_NONE;
	if (Planet.Plates.IsValidIndex(ChildPlateAIndex))
	{
		Diag.bChildPlateAAlive = true;
		Diag.CurrentChildSampleCountA = Planet.Plates[ChildPlateAIndex].MemberSamples.Num();
	}
	if (Planet.Plates.IsValidIndex(ChildPlateBIndex))
	{
		Diag.bChildPlateBAlive = true;
		Diag.CurrentChildSampleCountB = Planet.Plates[ChildPlateBIndex].MemberSamples.Num();
	}

	if (Diag.bChildPlateAAlive && Diag.bChildPlateBAlive)
	{
		Diag.ChildBoundaryContactEdgeCount =
			CountPlatePairContactEdges(Planet, Diag.ChildPlateA, Diag.ChildPlateB);
		CountActivePairCauseSamples(
			LastResolvedSamples,
			Diag.ChildPlateA,
			Diag.ChildPlateB,
			Diag.ChildBoundaryRiftActiveSampleCount,
			Diag.ChildBoundaryDivergenceActiveSampleCount);

		const FTectonicPlanetV6PlatePairBoundaryMotionDiagnostic MotionDiag =
			ComputePlatePairBoundaryMotionDiagnostic(Planet, Diag.ChildPlateA, Diag.ChildPlateB);
		Diag.ChildBoundaryDivergentEdgeCount = MotionDiag.DivergentEdgeCount;
		Diag.ChildBoundaryConvergentEdgeCount = MotionDiag.ConvergentEdgeCount;
		Diag.ChildBoundaryMeanRelativeNormalVelocityKmPerMy = MotionDiag.MeanRelativeNormalVelocityKmPerMy;
		Diag.ChildBoundaryMaxAbsRelativeNormalVelocityKmPerMy = MotionDiag.MaxAbsRelativeNormalVelocityKmPerMy;
		Diag.bChildBoundaryClassifiedDivergent =
			Diag.ChildBoundaryRiftActiveSampleCount > 0 ||
			Diag.ChildBoundaryDivergenceActiveSampleCount > 0 ||
			MotionDiag.bPairIsDivergent;
	}
	return Diag;
}

FTectonicPlanetV6PlatePairBoundaryMotionDiagnostic
FTectonicPlanetV6::ComputePlatePairBoundaryMotionDiagnosticForTest(
	const int32 PlateA,
	const int32 PlateB) const
{
	return ComputePlatePairBoundaryMotionDiagnostic(Planet, PlateA, PlateB);
}

const FTectonicPlanet& FTectonicPlanetV6::GetPlanet() const
{
	return Planet;
}

FTectonicPlanet& FTectonicPlanetV6::GetPlanetMutable()
{
	return Planet;
}

const FTectonicPlanetV6PeriodicSolveStats& FTectonicPlanetV6::GetLastSolveStats() const
{
	return LastSolveStats;
}

const TArray<FTectonicPlanetV6ResolvedSample>& FTectonicPlanetV6::GetLastResolvedSamples() const
{
	return LastResolvedSamples;
}

const TArray<int32>& FTectonicPlanetV6::GetPeriodicSolveSteps() const
{
	return PeriodicSolveSteps;
}

ETectonicPlanetV6PeriodicSolveMode FTectonicPlanetV6::GetKeptV6PeriodicSolveMode()
{
	return ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike;
}

int32 FTectonicPlanetV6::GetKeptV6FixedIntervalSteps()
{
	return 16;
}

FString FTectonicPlanetV6::DescribeKeptV6RuntimeProfile(
	const bool bEnableAutomaticRifting,
	const bool bEnableSubmergedContinentalFringeRelaxation)
{
	return FString::Printf(
		TEXT("Solve=ThesisPartitionedFrontierProcessSpike cadence=%d retention=ON surrogate=CWThicknessSelectiveElev submerged_relaxation=ON shoulder_fix=%s rifting=%s"),
		GetKeptV6FixedIntervalSteps(),
		bEnableSubmergedContinentalFringeRelaxation ? TEXT("ON") : TEXT("OFF"),
		bEnableAutomaticRifting ? TEXT("ON") : TEXT("OFF"));
}

void FTectonicPlanetV6::ApplyKeptV6RuntimeProfile(
	const FTectonicPlanetV6KeptRuntimeProfileOptions& Options)
{
	SetPeriodicSolveModeForTest(GetKeptV6PeriodicSolveMode(), GetKeptV6FixedIntervalSteps());
	SetSyntheticCoverageRetentionForTest(false);
	SetWholeTriangleBoundaryDuplicationForTest(false);
	SetExcludeMixedTrianglesForTest(false);
	SetV9Phase1AuthorityForTest(true, 1);
	SetV9Phase1ActiveZoneClassifierModeForTest(
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
	SetV9Phase1PersistentActivePairHorizonForTest(2);
	SetV9CollisionShadowForTest(true);
	SetV9CollisionExecutionForTest(true);
	SetV9CollisionExecutionEnhancedConsequencesForTest(true);
	SetV9CollisionExecutionStructuralTransferForTest(true);
	SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
	SetV9ThesisShapedCollisionExecutionForTest(true);
	SetV9QuietInteriorContinentalRetentionForTest(true);
	SetV9ContinentalBreadthPreservationForTest(false);
	SetV9PaperSurrogateOwnershipForTest(true);
	SetV9PaperSurrogateFieldModeForTest(
		ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightThicknessSelectiveElevation);
	SetSubmergedContinentalRelaxationForTest(true, Options.SubmergedContinentalRelaxationRatePerStep);
	SetV9SubmergedContinentalFringeRelaxationForTest(
		Options.bEnableSubmergedContinentalFringeRelaxation,
		Options.SubmergedContinentalFringeRelaxationRatePerStep,
		Options.SubmergedContinentalFringeBoundaryOrActiveBonusRatePerStep);
	SetAutomaticRiftingForTest(Options.bEnableAutomaticRifting);
	SetPlateCandidatePruningForTest(Options.bEnablePlateCandidatePruning);
	SetCopiedFrontierUnfilteredMeshReuseForTest(Options.bEnableCopiedFrontierUnfilteredMeshReuse);
	Planet.bUseCachedSubductionAdjacencyEdgeDistancesForTest =
		Options.bUseCachedSubductionAdjacencyEdgeDistances;
	Planet.bUseSubductionPerformanceOptimizationsForTest =
		Options.bUseSubductionPerformanceOptimizations;
}

void FTectonicPlanetV6::ApplyKeptV6DiagnosticsProfile(
	const FTectonicPlanetV6KeptDiagnosticsOptions& Options)
{
	SetPhaseTimingForTest(Options.bEnablePhaseTiming);
	SetDetailedCopiedFrontierAttributionForTest(Options.bEnableDetailedCopiedFrontierAttribution);
}

void FTectonicPlanetV6::SetPeriodicSolveModeForTest(
	const ETectonicPlanetV6PeriodicSolveMode InMode,
	const int32 InFixedIntervalSteps)
{
	PeriodicSolveMode = InMode;
	if (InMode == ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike ||
		IsCopiedFrontierLikeSolveMode(InMode) ||
		InMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike)
	{
		ThesisRemeshFixedIntervalSteps = FMath::Max(1, InFixedIntervalSteps);
	}

	if (IsCopiedFrontierLikeSolveMode(InMode))
	{
		RebuildThesisCopiedFrontierMeshes();
		if (UsesIntervalDestructivePropagationForSolveMode(InMode))
		{
			RefreshCopiedFrontierDestructiveTrackingForTest();
		}
		else
		{
			CopiedFrontierTrackedDestructiveKinds.Reset();
			CopiedFrontierTrackedPreferredContinuationPlateIds.Reset();
			CopiedFrontierTrackedDestructiveSourcePlateIds.Reset();
			CopiedFrontierTrackedDestructiveDistancesKm.Reset();
			CopiedFrontierTrackedDestructiveSeedOriginFlags.Reset();
			CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags.Reset();
			CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags.Reset();
			CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats = FTectonicPlanetV6DestructiveTrackingLifecycleStats{};
			CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount = 0;
			CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
		}
	}
	else
	{
		CopiedFrontierTrackedDestructiveKinds.Reset();
		CopiedFrontierTrackedPreferredContinuationPlateIds.Reset();
		CopiedFrontierTrackedDestructiveSourcePlateIds.Reset();
		CopiedFrontierTrackedDestructiveDistancesKm.Reset();
		CopiedFrontierTrackedDestructiveSeedOriginFlags.Reset();
		CopiedFrontierTrackedDestructiveSeedSurvivalLoggedFlags.Reset();
		CopiedFrontierTrackedDestructiveTopologyNeighborOriginFlags.Reset();
		CopiedFrontierTrackedDestructiveCurrentIntervalLifecycleStats = FTectonicPlanetV6DestructiveTrackingLifecycleStats{};
		CopiedFrontierTrackedDestructiveCurrentIntervalSeedCount = 0;
		CopiedFrontierTrackedDestructiveCurrentIntervalPropagationCount = 0;
		CopiedFrontierTrackedDestructiveCurrentIntervalExpiredCount = 0;
		CopiedFrontierTrackedDestructiveCurrentIntervalPropagationWaveCount = 0;
	}

	if (InMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike)
	{
		RebuildThesisPlateSubmeshMeshes();
	}
}

void FTectonicPlanetV6::SetSyntheticCoverageRetentionForTest(const bool bEnable)
{
	bEnableSyntheticCoverageRetentionForTest = bEnable;
}

void FTectonicPlanetV6::SetWholeTriangleBoundaryDuplicationForTest(const bool bEnable)
{
	bForceWholeTriangleBoundaryDuplicationForTest = bEnable;
	if (IsCopiedFrontierLikeSolveMode(PeriodicSolveMode))
	{
		RebuildThesisCopiedFrontierMeshes();
	}
}

void FTectonicPlanetV6::SetExcludeMixedTrianglesForTest(const bool bEnable)
{
	bForceExcludeMixedTrianglesForTest = bEnable;
	if (bEnable)
	{
		bForceWholeTriangleBoundaryDuplicationForTest = false;
	}
	if (IsCopiedFrontierLikeSolveMode(PeriodicSolveMode))
	{
		RebuildThesisCopiedFrontierMeshes();
	}
}

void FTectonicPlanetV6::SetPerTimestepContainmentSoupRebuildForTest(const bool bEnable)
{
	bForcePerTimestepContainmentSoupRebuildForTest = bEnable;
}

void FTectonicPlanetV6::SetPhaseTimingForTest(const bool bEnable)
{
	bEnablePhaseTimingForTest = bEnable;
}

void FTectonicPlanetV6::SetDetailedCopiedFrontierAttributionForTest(const bool bEnable)
{
	bEnableDetailedCopiedFrontierAttributionForTest = bEnable;
}

void FTectonicPlanetV6::SetPlateCandidatePruningForTest(const bool bEnable)
{
	bUsePlateCandidatePruningForTest = bEnable;
}

void FTectonicPlanetV6::SetCopiedFrontierUnfilteredMeshReuseForTest(const bool bEnable)
{
	bUseCopiedFrontierUnfilteredMeshReuseForTest = bEnable;
}

void FTectonicPlanetV6::CaptureCopiedFrontierPreSolveState(
	TArray<int32>& OutPreSolvePlateIds,
	TArray<uint8>& OutPreSolveContinentalFlags,
	TArray<float>& OutPreSolveContinentalWeights,
	TArray<float>& OutPreSolveElevations,
	TArray<float>& OutPreSolveThicknesses)
{
	OutPreSolvePlateIds.Reset();
	OutPreSolvePlateIds.Reserve(Planet.Samples.Num());
	OutPreSolveContinentalFlags.Reset();
	OutPreSolveContinentalFlags.Reserve(Planet.Samples.Num());
	OutPreSolveContinentalWeights.Reset();
	OutPreSolveContinentalWeights.Reserve(Planet.Samples.Num());
	OutPreSolveElevations.Reset();
	OutPreSolveElevations.Reserve(Planet.Samples.Num());
	OutPreSolveThicknesses.Reset();
	OutPreSolveThicknesses.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OutPreSolvePlateIds.Add(Sample.PlateId);
		OutPreSolveContinentalFlags.Add(Sample.ContinentalWeight >= 0.5f ? 1 : 0);
		OutPreSolveContinentalWeights.Add(Sample.ContinentalWeight);
		OutPreSolveElevations.Add(Sample.Elevation);
		OutPreSolveThicknesses.Add(Sample.Thickness);
	}
}

void FTectonicPlanetV6::ResetCopiedFrontierSolveTransientState(
	const TArray<int32>& PreSolvePlateIds,
	const TArray<float>& PreSolveContinentalWeights,
	const TArray<float>& PreSolveElevations)
{
	CurrentSolvePreSolvePlateIds = PreSolvePlateIds;
	CurrentSolvePreSolveContinentalWeights = PreSolveContinentalWeights;
	CurrentSolvePreSolveElevations = PreSolveElevations;

	CurrentSolvePreviousSyntheticFlags.Init(0, Planet.Samples.Num());
	if (PreviousSolveSyntheticFlags.Num() == Planet.Samples.Num())
	{
		CurrentSolvePreviousSyntheticFlags = PreviousSolveSyntheticFlags;
	}
	CurrentSolvePreviousRetainedSyntheticCoverageFlags.Init(0, Planet.Samples.Num());
	if (PreviousSolveRetainedSyntheticCoverageFlags.Num() == Planet.Samples.Num())
	{
		CurrentSolvePreviousRetainedSyntheticCoverageFlags = PreviousSolveRetainedSyntheticCoverageFlags;
	}
	CurrentSolvePreviousTransferSourceKindValues.Init(
		static_cast<uint8>(ETectonicPlanetV6TransferSourceKind::None),
		Planet.Samples.Num());
	if (PreviousSolveTransferSourceKindValues.Num() == Planet.Samples.Num())
	{
		CurrentSolvePreviousTransferSourceKindValues = PreviousSolveTransferSourceKindValues;
	}
	CurrentSolvePreviousResolutionKindValues.Init(
		static_cast<uint8>(ETectonicPlanetV6ResolutionKind::None),
		Planet.Samples.Num());
	if (PreviousSolveResolutionKindValues.Num() == Planet.Samples.Num())
	{
		CurrentSolvePreviousResolutionKindValues = PreviousSolveResolutionKindValues;
	}
	CurrentSolveRetainedSyntheticCoverageFlags.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerFilteredHitFlags.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerUnfilteredHitFlags.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerIgnoringBoundingCapHitFlags.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerRecoveryFlags.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerCanonicalVertexInMeshFlags.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerAdjacentTriangleInMeshFlags.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerAdjacentTriangleNearestDistanceKm.Init(-1.0f, Planet.Samples.Num());
	CurrentSolvePreviousOwnerAdjacentTriangleSupportCounts.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerAdjacentTriangleMixedSupportCounts.Init(0, Planet.Samples.Num());
	CurrentSolvePreviousOwnerAdjacentTriangleMinoritySupportCounts.Init(0, Planet.Samples.Num());
	CurrentSolveLocalParticipationPlateCounts.Init(0, Planet.Samples.Num());
	CurrentSolveActiveZoneFlags.Init(0, Planet.Samples.Num());
	CurrentSolveActiveZoneSeedFlags.Init(0, Planet.Samples.Num());
	CurrentSolveActiveZoneCarryoverFlags.Init(0, Planet.Samples.Num());
	CurrentSolveActiveZoneFreshExpandedFlags.Init(0, Planet.Samples.Num());
	CurrentSolveActiveZoneCarryoverExpandedFlags.Init(0, Planet.Samples.Num());
	CurrentSolveActiveZoneCauseValues.Init(
		static_cast<uint8>(ETectonicPlanetV6ActiveZoneCause::None),
		Planet.Samples.Num());
	CurrentSolveActiveZonePrimaryPlateIds.Init(INDEX_NONE, Planet.Samples.Num());
	CurrentSolveActiveZoneSecondaryPlateIds.Init(INDEX_NONE, Planet.Samples.Num());
	CurrentSolveActivePairCount = 0;
	CurrentSolveFreshSeedActivePairCount = 0;
	CurrentSolvePersistentCarryoverActivePairCount = 0;
	CurrentSolveFreshPairCandidateCount = 0;
	CurrentSolveFreshPairAdmittedCount = 0;
	CurrentSolveFreshPairRejectedSupportCount = 0;
	CurrentSolveFreshPairRejectedVelocityCount = 0;
	CurrentSolveFreshPairRejectedDominanceCount = 0;
	CurrentSolveFreshPairAdmittedDivergenceCount = 0;
	CurrentSolveFreshPairAdmittedConvergentSubductionCount = 0;
	CurrentSolveFreshPairAdmittedCollisionContactCount = 0;
	CurrentSolveFreshPairAdmittedRiftCount = 0;
	CurrentSolveOutsideActiveZoneQueryMissFlags.Init(0, Planet.Samples.Num());
	CurrentSolveOutsideActiveZoneCoverageDeficitFlags.Init(0, Planet.Samples.Num());
}

void FTectonicPlanetV6::SetV9Phase1AuthorityForTest(const bool bEnable, const int32 InActiveBoundaryRingCount)
{
	bEnableV9Phase1AuthorityForTest = bEnable;
	V9Phase1ActiveBoundaryRingCountForTest = FMath::Max(0, InActiveBoundaryRingCount);
	PersistentV9ActivePairRemainingSolveIntervals.Reset();
	PersistentV9ActivePairCauseValues.Reset();
	V9CollisionShadowPairRecurrenceByKey.Reset();
	V9CollisionExecutionLastSolveIndexByKey.Reset();
	V9CollisionExecutionCumulativeCount = 0;
	V9CollisionExecutionCumulativeAffectedSampleVisits = 0;
	V9CollisionExecutionCumulativeContinentalGainCount = 0;
	V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
	V9CollisionExecutionCumulativeElevationDeltaKm = 0.0;
	V9CollisionExecutionCumulativeMaxElevationDeltaKm = 0.0;
	CumulativeCollisionExecutionMask.Reset();
	CumulativeCollisionTransferMask.Reset();
	CumulativeCollisionElevationDeltaMaskKm.Reset();
	CumulativeCollisionContinentalGainMask.Reset();
}

void FTectonicPlanetV6::SetV9Phase1ActiveZoneClassifierModeForTest(
	const ETectonicPlanetV6ActiveZoneClassifierMode InMode)
{
	V9Phase1ActiveZoneClassifierModeForTest = InMode;
	PersistentV9ActivePairRemainingSolveIntervals.Reset();
	PersistentV9ActivePairCauseValues.Reset();
	V9CollisionShadowPairRecurrenceByKey.Reset();
	V9CollisionExecutionLastSolveIndexByKey.Reset();
	V9CollisionExecutionCumulativeCount = 0;
	V9CollisionExecutionCumulativeAffectedSampleVisits = 0;
	V9CollisionExecutionCumulativeContinentalGainCount = 0;
	V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
	V9CollisionExecutionCumulativeElevationDeltaKm = 0.0;
	V9CollisionExecutionCumulativeMaxElevationDeltaKm = 0.0;
	CumulativeCollisionExecutionMask.Reset();
	CumulativeCollisionTransferMask.Reset();
	CumulativeCollisionElevationDeltaMaskKm.Reset();
	CumulativeCollisionContinentalGainMask.Reset();
}

void FTectonicPlanetV6::SetV9Phase1PersistentActivePairHorizonForTest(const int32 InPersistenceHorizon)
{
	V9Phase1PersistentActivePairHorizonForTest = FMath::Clamp(InPersistenceHorizon, 1, 3);
	PersistentV9ActivePairRemainingSolveIntervals.Reset();
	PersistentV9ActivePairCauseValues.Reset();
	V9CollisionShadowPairRecurrenceByKey.Reset();
	V9CollisionExecutionLastSolveIndexByKey.Reset();
	V9CollisionExecutionCumulativeCount = 0;
	V9CollisionExecutionCumulativeAffectedSampleVisits = 0;
	V9CollisionExecutionCumulativeContinentalGainCount = 0;
	V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
	V9CollisionExecutionCumulativeElevationDeltaKm = 0.0;
	V9CollisionExecutionCumulativeMaxElevationDeltaKm = 0.0;
	CumulativeCollisionExecutionMask.Reset();
	CumulativeCollisionTransferMask.Reset();
	CumulativeCollisionElevationDeltaMaskKm.Reset();
	CumulativeCollisionContinentalGainMask.Reset();
}

void FTectonicPlanetV6::SetV9CollisionShadowForTest(const bool bEnable)
{
	bEnableV9CollisionShadowForTest = bEnable;
	CurrentSolveCollisionShadowTrackedFlags.Reset();
	CurrentSolveCollisionShadowQualifiedFlags.Reset();
	CurrentSolveCollisionShadowPersistenceMask.Reset();
	CurrentSolveCollisionShadowDiagnostic = FTectonicPlanetV6CollisionShadowDiagnostic{};
	V9CollisionShadowPairRecurrenceByKey.Reset();
}

void FTectonicPlanetV6::SetV9CollisionExecutionForTest(const bool bEnable)
{
	bEnableV9CollisionExecutionForTest = bEnable;
	bEnableV9CollisionExecutionRefinedStructuralTransferForTest = false;
	bEnableV9ThesisShapedCollisionRidgeSurgeForTest = false;
	CurrentSolveCollisionExecutionMask.Reset();
	CurrentSolveCollisionTransferMask.Reset();
	CumulativeCollisionExecutionMask.Reset();
	CumulativeCollisionTransferMask.Reset();
	CumulativeCollisionElevationDeltaMaskKm.Reset();
	CumulativeCollisionContinentalGainMask.Reset();
	CurrentSolveCollisionExecutionDiagnostic = FTectonicPlanetV6CollisionExecutionDiagnostic{};
	V9CollisionExecutionLastSolveIndexByKey.Reset();
	V9CollisionExecutionCumulativeCount = 0;
	V9CollisionExecutionCumulativeAffectedSampleVisits = 0;
	V9CollisionExecutionCumulativeContinentalGainCount = 0;
	V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
	V9CollisionExecutionCumulativeElevationDeltaKm = 0.0;
	V9CollisionExecutionCumulativeMaxElevationDeltaKm = 0.0;
}

void FTectonicPlanetV6::SetV9CollisionExecutionEnhancedConsequencesForTest(const bool bEnable)
{
	bEnableV9CollisionExecutionEnhancedConsequencesForTest = bEnable;
}

void FTectonicPlanetV6::SetV9CollisionExecutionStructuralTransferForTest(const bool bEnable)
{
	bEnableV9CollisionExecutionStructuralTransferForTest = bEnable;
	CurrentSolveCollisionTransferMask.Reset();
	CumulativeCollisionTransferMask.Reset();
	V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
}

void FTectonicPlanetV6::SetV9CollisionExecutionRefinedStructuralTransferForTest(const bool bEnable)
{
	bEnableV9CollisionExecutionRefinedStructuralTransferForTest = bEnable;
	CurrentSolveCollisionTransferMask.Reset();
	CumulativeCollisionTransferMask.Reset();
	V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
}

void FTectonicPlanetV6::SetV9ThesisShapedCollisionExecutionForTest(const bool bEnable)
{
	bEnableV9ThesisShapedCollisionExecutionForTest = bEnable;
	bEnableV9ThesisShapedCollisionRidgeSurgeForTest = false;
	CurrentSolveThesisCollisionTerraneComponentMask.Reset();
	CurrentSolveCollisionTransferMask.Reset();
	CumulativeCollisionTransferMask.Reset();
	V9CollisionExecutionCumulativeOwnershipChangeCount = 0;
	V9CollisionExecutionCumulativeTransferredSampleVisits = 0;
	V9CollisionExecutionCumulativeTransferredContinentalSampleCount = 0;
}

void FTectonicPlanetV6::SetV9ThesisShapedCollisionRidgeSurgeForTest(const bool bEnable)
{
	bEnableV9ThesisShapedCollisionRidgeSurgeForTest = bEnable;
}

void FTectonicPlanetV6::SetV9QuietInteriorContinentalRetentionForTest(const bool bEnable)
{
	bEnableV9QuietInteriorContinentalRetentionForTest = bEnable;
}

void FTectonicPlanetV6::SetV9ContinentalBreadthPreservationForTest(const bool bEnable)
{
	bEnableV9ContinentalBreadthPreservationForTest = bEnable;
}

void FTectonicPlanetV6::SetV9PaperSurrogateOwnershipForTest(const bool bEnable)
{
	bEnableV9PaperSurrogateOwnershipForTest = bEnable;
}

void FTectonicPlanetV6::SetV9PaperSurrogateFieldModeForTest(
	const ETectonicPlanetV6PaperSurrogateFieldMode InMode)
{
	V9PaperSurrogateFieldModeForTest = InMode;
}

void FTectonicPlanetV6::SetSubmergedContinentalRelaxationForTest(const bool bEnable, const double RatePerStep)
{
	Planet.bEnableSubmergedContinentalRelaxation = bEnable;
	Planet.SubmergedContinentalRelaxationRatePerStep = RatePerStep;
}

void FTectonicPlanetV6::SetV9SubmergedContinentalFringeRelaxationForTest(
	const bool bEnable,
	const double RatePerStep,
	const double BoundaryOrActiveBonusRatePerStep)
{
	bEnableV9SubmergedContinentalFringeRelaxationForTest = bEnable;
	V9SubmergedContinentalFringeRelaxationRatePerStepForTest = FMath::Max(0.0, RatePerStep);
	V9SubmergedContinentalFringeBoundaryOrActiveBonusRatePerStepForTest =
		FMath::Max(0.0, BoundaryOrActiveBonusRatePerStep);
	RebuildSubmergedContinentalFringeRelaxationMask();
}

void FTectonicPlanetV6::SetAutomaticRiftingForTest(const bool bEnable)
{
	bEnableAutomaticRiftingForTest = bEnable;
	Planet.bEnableAutomaticRifting = bEnable;
	Planet.bDeferRiftFollowupResamplingToV6 = bEnable;
}

bool FTectonicPlanetV6::ForceLargestEligibleAutomaticRiftForTest(const int32 ChildCount, const int32 Seed)
{
	if (Planet.PendingRiftEvent.bValid)
	{
		return HandlePendingAutomaticRiftAfterAdvance();
	}

	int32 ParentContinentalSampleCount = 0;
	double ParentContinentalFraction = 0.0;
	const int32 ParentPlateId = Planet.FindLargestEligibleAutomaticRiftParentId(
		&ParentContinentalSampleCount,
		&ParentContinentalFraction);
	if (ParentPlateId == INDEX_NONE)
	{
		return false;
	}

	const int32 ParentPlateIndex = Planet.FindPlateArrayIndexById(ParentPlateId);
	if (!Planet.Plates.IsValidIndex(ParentPlateIndex))
	{
		return false;
	}

	const int32 ParentSampleCount = Planet.Plates[ParentPlateIndex].MemberSamples.Num();
	const double TriggerProbability = Planet.ComputeAutomaticRiftProbabilityForSampleCount(
		ParentSampleCount,
		ParentContinentalFraction);

	const bool bPreviousAutomaticRiftingForTest = bEnableAutomaticRiftingForTest;
	const bool bPreviousAutomaticRifting = Planet.bEnableAutomaticRifting;
	const bool bPreviousDeferRiftFollowup = Planet.bDeferRiftFollowupResamplingToV6;

	bEnableAutomaticRiftingForTest = true;
	Planet.bEnableAutomaticRifting = true;
	Planet.bDeferRiftFollowupResamplingToV6 = true;

	const bool bTriggered = Planet.TriggerForcedRift(ParentPlateId, ChildCount, Seed);
	if (!bTriggered || !Planet.PendingRiftEvent.bValid)
	{
		bEnableAutomaticRiftingForTest = bPreviousAutomaticRiftingForTest;
		Planet.bEnableAutomaticRifting = bPreviousAutomaticRifting;
		Planet.bDeferRiftFollowupResamplingToV6 = bPreviousDeferRiftFollowup;
		return false;
	}

	Planet.PendingRiftEvent.bAutomatic = false;
	Planet.PendingRiftEvent.bForcedByTest = true;
	Planet.PendingRiftEvent.TriggerProbability = TriggerProbability;
	Planet.PendingRiftEvent.TriggerDraw = -1.0;

	const bool bHandled = HandlePendingAutomaticRiftAfterAdvance();

	bEnableAutomaticRiftingForTest = bPreviousAutomaticRiftingForTest;
	Planet.bEnableAutomaticRifting = bPreviousAutomaticRifting;
	Planet.bDeferRiftFollowupResamplingToV6 = bPreviousDeferRiftFollowup;
	return bHandled;
}

void FTectonicPlanetV6::SetUseLinearConvergentMaintenanceSpeedFactorForTest(
	const bool bEnableLinear)
{
	bUseLinearConvergentMaintenanceSpeedFactorForTest = bEnableLinear;
}

void FTectonicPlanetV6::SetUseLinearConvergentMaintenanceInfluenceForTest(
	const bool bEnableLinear)
{
	bUseLinearConvergentMaintenanceInfluenceForTest = bEnableLinear;
}
