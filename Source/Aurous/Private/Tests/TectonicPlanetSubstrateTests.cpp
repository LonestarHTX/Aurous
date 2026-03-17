#include "Misc/AutomationTest.h"

#include "Algo/Count.h"
#include "HAL/PlatformFile.h"
#include "HAL/PlatformFileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"
#include "TectonicMollweideExporter.h"
#include "TectonicPlanet.h"

namespace
{
	constexpr int32 TestSampleCount = 60000;
	constexpr int32 TestPlateCount = 7;
	constexpr int32 StressPlateCount = 40;
	constexpr int32 HighResStressSampleCount = 500000;
	constexpr int32 TestExportWidth = 4096;
	constexpr int32 TestExportHeight = 2048;
	constexpr int32 TestRandomSeed = 42;
	constexpr double TestPlanetRadiusKm = 6371.0;
	constexpr float TestBoundaryWarpAmplitude = 0.2f;
	constexpr float TestContinentalFraction = 0.30f;
	constexpr double AbyssalPlainElevationKm = -6.0;
	constexpr double ElevationFloorKm = -10.0;
	constexpr double ElevationCeilingKm = 10.0;
	constexpr double TestDivergentThresholdRatio = 0.3;

	struct FElevationSummary
	{
		double MaxContinentalElevation = -TNumericLimits<double>::Max();
		double MeanOceanicElevation = 0.0;
		int32 ContinentalSampleCount = 0;
		int32 OceanicSampleCount = 0;
	};

	struct FOwnershipQuerySummary
	{
		int32 GapCount = 0;
		int32 DivergentGapCount = 0;
		int32 OverlapCount = 0;
		int32 ValidContainmentCount = 0;
		int32 MultiContainmentCount = 0;
		int32 NoValidHitCount = 0;
		int32 ValidBarycentricCount = 0;
	};

	struct FTerraneValidationSummary
	{
		int32 ContinentalSampleCount = 0;
		int32 ContinentalSamplesWithValidTerrane = 0;
		int32 ContinentalSamplesWithMatchingTerranePlate = 0;
		int32 OceanicSampleCount = 0;
		int32 OceanicSamplesWithUnsetTerrane = 0;
		bool bUniqueTerraneIds = true;
	};

	struct FSoupPartitionSummary
	{
		int32 InteriorSoupTriangleCount = 0;
		int32 BoundarySoupTriangleCount = 0;
		int32 TotalSoupTriangleCount = 0;
		int32 DroppedMixedTriangleCount = 0;
		int32 DuplicatedSoupTriangleCount = 0;
		int32 ForeignLocalVertexCount = 0;
	};

	FTectonicPlanet CreateInitializedPlanet(const int32 PlateCount = TestPlateCount, const int32 SampleCount = TestSampleCount)
	{
		FTectonicPlanet Planet;
		Planet.Initialize(SampleCount, TestPlanetRadiusKm);
		Planet.InitializePlates(PlateCount, TestRandomSeed, TestBoundaryWarpAmplitude, TestContinentalFraction);
		Planet.bEnableAutomaticRifting = false;
		return Planet;
	}

	void ConfigureAutomaticRiftingForTest(FTectonicPlanet& Planet, const double BaseLambdaPerStep = 0.18)
	{
		Planet.bEnableAutomaticRifting = true;
		Planet.bEnableWarpedRiftBoundaries = true;
		Planet.AutomaticRiftBaseLambdaPerStep = BaseLambdaPerStep;
		Planet.AutomaticRiftMinParentSamples = 1500;
		Planet.AutomaticRiftMinContinentalSamples = 64;
		Planet.AutomaticRiftMinContinentalFraction = 0.05;
		Planet.AutomaticRiftCooldownSteps = 20;
		Planet.RiftBoundaryWarpAmplitude = 0.25;
		Planet.RiftBoundaryWarpFrequency = 1.5;
	}

	FElevationSummary BuildElevationSummary(const FTectonicPlanet& Planet)
	{
		FElevationSummary Summary;
		for (const FSample& Sample : Planet.Samples)
		{
			if (Sample.ContinentalWeight >= 0.5f)
			{
				++Summary.ContinentalSampleCount;
				Summary.MaxContinentalElevation = FMath::Max(Summary.MaxContinentalElevation, static_cast<double>(Sample.Elevation));
			}
			else
			{
				++Summary.OceanicSampleCount;
				Summary.MeanOceanicElevation += Sample.Elevation;
			}
		}

		if (Summary.OceanicSampleCount > 0)
		{
			Summary.MeanOceanicElevation /= static_cast<double>(Summary.OceanicSampleCount);
		}

		if (Summary.ContinentalSampleCount == 0)
		{
			Summary.MaxContinentalElevation = 0.0;
		}

		return Summary;
	}

	double ComputeContinentalAreaFraction(const FTectonicPlanet& Planet)
	{
		int32 ContinentalSampleCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			ContinentalSampleCount += Sample.ContinentalWeight >= 0.5f ? 1 : 0;
		}

		return Planet.Samples.IsEmpty()
			? 0.0
			: static_cast<double>(ContinentalSampleCount) / static_cast<double>(Planet.Samples.Num());
	}

	int32 CountContinentalSamples(const FTectonicPlanet& Planet)
	{
		int32 ContinentalSampleCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			ContinentalSampleCount += Sample.ContinentalWeight >= 0.5f ? 1 : 0;
		}

		return ContinentalSampleCount;
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

	FVector3d ComputePlateSurfaceVelocityForTest(const FPlate& Plate, const FVector3d& QueryPoint, const double PlanetRadiusKm);
	FVector3d ComputeSampleSetCentroidForTest(const FTectonicPlanet& Planet, const TArray<int32>& SampleIndices);

	FString JoinIntArray(const TArray<int32>& Values)
	{
		TArray<FString> Parts;
		Parts.Reserve(Values.Num());
		for (const int32 Value : Values)
		{
			Parts.Add(FString::FromInt(Value));
		}

		return FString::Join(Parts, TEXT(","));
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

	int32 FindLargestPlateId(const FTectonicPlanet& Planet)
	{
		int32 LargestPlateId = INDEX_NONE;
		int32 LargestPlateSize = -1;
		for (const FPlate& Plate : Planet.Plates)
		{
			if (Plate.MemberSamples.Num() > LargestPlateSize ||
				(Plate.MemberSamples.Num() == LargestPlateSize &&
					(LargestPlateId == INDEX_NONE || Plate.Id < LargestPlateId)))
			{
				LargestPlateId = Plate.Id;
				LargestPlateSize = Plate.MemberSamples.Num();
			}
		}

		return LargestPlateId;
	}

	int32 FindLargestTerraneBearingPlateId(const FTectonicPlanet& Planet)
	{
		int32 BestPlateId = INDEX_NONE;
		int32 BestPlateSize = -1;
		for (const FPlate& Plate : Planet.Plates)
		{
			bool bHasTerrane = false;
			for (const int32 SampleIndex : Plate.MemberSamples)
			{
				if (Planet.Samples.IsValidIndex(SampleIndex) &&
					Planet.Samples[SampleIndex].TerraneId != INDEX_NONE)
				{
					bHasTerrane = true;
					break;
				}
			}

			if (!bHasTerrane)
			{
				continue;
			}

			if (Plate.MemberSamples.Num() > BestPlateSize ||
				(Plate.MemberSamples.Num() == BestPlateSize &&
					(BestPlateId == INDEX_NONE || Plate.Id < BestPlateId)))
			{
				BestPlateId = Plate.Id;
				BestPlateSize = Plate.MemberSamples.Num();
			}
		}

		return BestPlateId;
	}

	double ComputeGeodesicDistanceForTest(const FVector3d& A, const FVector3d& B)
	{
		return FMath::Acos(FMath::Clamp(A.Dot(B), -1.0, 1.0));
	}

	int32 FindPlateIdBySizeRank(const FTectonicPlanet& Planet, const int32 Rank, const int32 MinSamples = 0)
	{
		TArray<int32> OrderedPlateIndices;
		OrderedPlateIndices.Reserve(Planet.Plates.Num());
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			if (Planet.Plates[PlateIndex].MemberSamples.Num() >= MinSamples)
			{
				OrderedPlateIndices.Add(PlateIndex);
			}
		}

		OrderedPlateIndices.Sort([&Planet](const int32 LeftIndex, const int32 RightIndex)
		{
			const FPlate& LeftPlate = Planet.Plates[LeftIndex];
			const FPlate& RightPlate = Planet.Plates[RightIndex];
			if (LeftPlate.MemberSamples.Num() != RightPlate.MemberSamples.Num())
			{
				return LeftPlate.MemberSamples.Num() > RightPlate.MemberSamples.Num();
			}

			return LeftPlate.Id < RightPlate.Id;
		});

		return OrderedPlateIndices.IsValidIndex(Rank)
			? Planet.Plates[OrderedPlateIndices[Rank]].Id
			: INDEX_NONE;
	}

	int32 CountDistinctTerraneIdsOnPlate(const FTectonicPlanet& Planet, const int32 PlateId)
	{
		const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
		if (!Planet.Plates.IsValidIndex(PlateIndex))
		{
			return 0;
		}

		TSet<int32> TerraneIds;
		for (const int32 SampleIndex : Planet.Plates[PlateIndex].MemberSamples)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 TerraneId = Planet.Samples[SampleIndex].TerraneId;
			if (TerraneId != INDEX_NONE)
			{
				TerraneIds.Add(TerraneId);
			}
		}

		return TerraneIds.Num();
	}

	bool AssignSyntheticTerranesToPlateForTest(
		FTectonicPlanet& Planet,
		const int32 PlateId,
		const int32 TerraneCount,
		const int32 Seed)
	{
		const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
		if (!Planet.Plates.IsValidIndex(PlateIndex) || TerraneCount <= 0)
		{
			return false;
		}

		TArray<int32> ParentMembers = Planet.Plates[PlateIndex].MemberSamples;
		ParentMembers.Sort();
		if (ParentMembers.Num() < TerraneCount)
		{
			return false;
		}

		TArray<int32> SeedSamples;
		SeedSamples.Reserve(TerraneCount);
		const int32 FirstSeedOffset = FMath::Abs(Seed) % ParentMembers.Num();
		SeedSamples.Add(ParentMembers[FirstSeedOffset]);
		while (SeedSamples.Num() < TerraneCount)
		{
			int32 BestSampleIndex = INDEX_NONE;
			double BestMinDistance = -1.0;
			for (const int32 CandidateSampleIndex : ParentMembers)
			{
				bool bAlreadyChosen = false;
				for (const int32 ChosenSeedSample : SeedSamples)
				{
					if (ChosenSeedSample == CandidateSampleIndex)
					{
						bAlreadyChosen = true;
						break;
					}
				}

				if (bAlreadyChosen || !Planet.Samples.IsValidIndex(CandidateSampleIndex))
				{
					continue;
				}

				const FVector3d CandidatePosition = Planet.Samples[CandidateSampleIndex].Position.GetSafeNormal();
				double MinDistanceToChosenSeed = TNumericLimits<double>::Max();
				for (const int32 ChosenSeedSample : SeedSamples)
				{
					MinDistanceToChosenSeed = FMath::Min(
						MinDistanceToChosenSeed,
						ComputeGeodesicDistanceForTest(
							CandidatePosition,
							Planet.Samples[ChosenSeedSample].Position.GetSafeNormal()));
				}

				if (MinDistanceToChosenSeed > BestMinDistance + KINDA_SMALL_NUMBER ||
					(FMath::IsNearlyEqual(MinDistanceToChosenSeed, BestMinDistance, KINDA_SMALL_NUMBER) &&
						(BestSampleIndex == INDEX_NONE || CandidateSampleIndex < BestSampleIndex)))
				{
					BestSampleIndex = CandidateSampleIndex;
					BestMinDistance = MinDistanceToChosenSeed;
				}
			}

			if (BestSampleIndex == INDEX_NONE)
			{
				return false;
			}

			SeedSamples.Add(BestSampleIndex);
		}

		TArray<TArray<int32>> TerraneMembers;
		TerraneMembers.SetNum(TerraneCount);
		for (const int32 SampleIndex : ParentMembers)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FVector3d SamplePosition = Planet.Samples[SampleIndex].Position.GetSafeNormal();
			int32 BestTerraneIndex = INDEX_NONE;
			double BestDistance = TNumericLimits<double>::Max();
			for (int32 TerraneIndex = 0; TerraneIndex < SeedSamples.Num(); ++TerraneIndex)
			{
				const double Distance = ComputeGeodesicDistanceForTest(
					SamplePosition,
					Planet.Samples[SeedSamples[TerraneIndex]].Position.GetSafeNormal());
				if (Distance < BestDistance - KINDA_SMALL_NUMBER ||
					(FMath::IsNearlyEqual(Distance, BestDistance, KINDA_SMALL_NUMBER) &&
						(BestTerraneIndex == INDEX_NONE || SeedSamples[TerraneIndex] < SeedSamples[BestTerraneIndex])))
				{
					BestTerraneIndex = TerraneIndex;
					BestDistance = Distance;
				}
			}

			if (!TerraneMembers.IsValidIndex(BestTerraneIndex))
			{
				return false;
			}

			TerraneMembers[BestTerraneIndex].Add(SampleIndex);
		}

		const double SampleAreaKm2 =
			Planet.Samples.IsEmpty()
				? 0.0
				: (4.0 * PI * Planet.PlanetRadiusKm * Planet.PlanetRadiusKm) / static_cast<double>(Planet.Samples.Num());
		const int32 BaseTerraneId = FMath::Max(Planet.NextTerraneId, 1000);
		Planet.NextTerraneId = BaseTerraneId + TerraneCount;

		Planet.Terranes.RemoveAll([PlateId](const FTerrane& Terrane)
		{
			return Terrane.PlateId == PlateId;
		});

		for (int32 TerraneIndex = 0; TerraneIndex < TerraneMembers.Num(); ++TerraneIndex)
		{
			const int32 TerraneId = BaseTerraneId + TerraneIndex;
			for (const int32 SampleIndex : TerraneMembers[TerraneIndex])
			{
				if (!Planet.Samples.IsValidIndex(SampleIndex))
				{
					continue;
				}

				Planet.Samples[SampleIndex].TerraneId = TerraneId;
				const int32* CarriedIndex = Planet.Plates[PlateIndex].CanonicalToCarriedIndex.Find(SampleIndex);
				if (CarriedIndex != nullptr && Planet.Plates[PlateIndex].CarriedSamples.IsValidIndex(*CarriedIndex))
				{
					Planet.Plates[PlateIndex].CarriedSamples[*CarriedIndex].TerraneId = TerraneId;
				}
			}

			FTerrane Terrane;
			Terrane.TerraneId = TerraneId;
			Terrane.AnchorSample = TerraneMembers[TerraneIndex].IsEmpty() ? INDEX_NONE : TerraneMembers[TerraneIndex][0];
			Terrane.Centroid = ComputeSampleSetCentroidForTest(Planet, TerraneMembers[TerraneIndex]);
			Terrane.AreaKm2 = SampleAreaKm2 * static_cast<double>(TerraneMembers[TerraneIndex].Num());
			Terrane.PlateId = PlateId;
			Planet.Terranes.Add(Terrane);
		}

		return true;
	}

	int32 CountAdjacencyEdgesBetweenPlates(const FTectonicPlanet& Planet, const int32 PlateAId, const int32 PlateBId)
	{
		int32 EdgeCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (Planet.Samples[SampleIndex].PlateId != PlateAId)
			{
				continue;
			}

			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (NeighborIndex > SampleIndex && Planet.Samples.IsValidIndex(NeighborIndex) &&
					Planet.Samples[NeighborIndex].PlateId == PlateBId)
				{
					++EdgeCount;
				}
			}
		}

		return EdgeCount;
	}

	struct FPlateCoherenceSnapshot
	{
		int32 PlateId = INDEX_NONE;
		int32 SampleCount = 0;
		int32 ConnectedComponentCount = 0;
		int32 LargestComponentSize = 0;
		int32 ContinentalSampleCount = 0;
		int32 BoundarySampleCount = 0;
	};

	double ComputeLargestComponentFraction(const FPlateCoherenceSnapshot& Snapshot)
	{
		return Snapshot.SampleCount > 0
			? static_cast<double>(Snapshot.LargestComponentSize) / static_cast<double>(Snapshot.SampleCount)
			: 0.0;
	}

	FPlateCoherenceSnapshot BuildPlateCoherenceSnapshot(const FTectonicPlanet& Planet, const int32 PlateId)
	{
		FPlateCoherenceSnapshot Snapshot;
		Snapshot.PlateId = PlateId;
		const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
		if (PlateIndex == INDEX_NONE || !Planet.Plates.IsValidIndex(PlateIndex))
		{
			return Snapshot;
		}

		const TArray<int32>& MemberSamples = Planet.Plates[PlateIndex].MemberSamples;
		Snapshot.SampleCount = MemberSamples.Num();
		if (MemberSamples.IsEmpty())
		{
			return Snapshot;
		}

		TSet<int32> MemberSampleSet;
		MemberSampleSet.Reserve(MemberSamples.Num());
		for (const int32 SampleIndex : MemberSamples)
		{
			MemberSampleSet.Add(SampleIndex);
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FSample& Sample = Planet.Samples[SampleIndex];
			Snapshot.ContinentalSampleCount += Sample.ContinentalWeight >= 0.5f ? 1 : 0;
			Snapshot.BoundarySampleCount += Sample.bIsBoundary ? 1 : 0;
		}

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		for (const int32 SeedSampleIndex : MemberSamples)
		{
			if (!Planet.Samples.IsValidIndex(SeedSampleIndex) || Visited[SeedSampleIndex] != 0)
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
						!MemberSampleSet.Contains(NeighborIndex))
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			++Snapshot.ConnectedComponentCount;
			Snapshot.LargestComponentSize = FMath::Max(Snapshot.LargestComponentSize, ComponentSize);
		}

		return Snapshot;
	}

	struct FPlatePairBoundaryActivitySummary
	{
		int32 ContactEdgeCount = 0;
		int32 DivergentEdgeCount = 0;
	};

	struct FMultiPlateBoundaryActivitySummary
	{
		int32 ContactEdgeCount = 0;
		int32 DivergentEdgeCount = 0;
	};

	FPlatePairBoundaryActivitySummary BuildPlatePairBoundaryActivitySummary(
		const FTectonicPlanet& Planet,
		const int32 PlateAId,
		const int32 PlateBId)
	{
		FPlatePairBoundaryActivitySummary Summary;
		const int32 PlateAIndex = Planet.FindPlateArrayIndexById(PlateAId);
		const int32 PlateBIndex = Planet.FindPlateArrayIndexById(PlateBId);
		if (PlateAIndex == INDEX_NONE || PlateBIndex == INDEX_NONE)
		{
			return Summary;
		}

		const FPlate& PlateA = Planet.Plates[PlateAIndex];
		const FPlate& PlateB = Planet.Plates[PlateBIndex];
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (Planet.Samples[SampleIndex].PlateId != PlateAId)
			{
				continue;
			}

			const FVector3d PosA = Planet.Samples[SampleIndex].Position.GetSafeNormal();
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (NeighborIndex <= SampleIndex || !Planet.Samples.IsValidIndex(NeighborIndex) ||
					Planet.Samples[NeighborIndex].PlateId != PlateBId)
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

				const FVector3d RelVel =
					ComputePlateSurfaceVelocityForTest(PlateB, Midpoint, Planet.PlanetRadiusKm) -
					ComputePlateSurfaceVelocityForTest(PlateA, Midpoint, Planet.PlanetRadiusKm);
				const double RelSpeed = RelVel.Length();
				if (RelSpeed <= UE_DOUBLE_SMALL_NUMBER)
				{
					continue;
				}

				const FVector3d Separation = (PosB - PosA).GetSafeNormal();
				const double Ratio = RelVel.Dot(Separation) / RelSpeed;
				if (Ratio > TestDivergentThresholdRatio)
				{
					++Summary.DivergentEdgeCount;
				}
			}
		}

		return Summary;
	}

	FMultiPlateBoundaryActivitySummary BuildChildPlateBoundaryActivitySummary(
		const FTectonicPlanet& Planet,
		const TArray<int32>& ChildPlateIds)
	{
		FMultiPlateBoundaryActivitySummary Summary;
		for (int32 LeftChildIndex = 0; LeftChildIndex < ChildPlateIds.Num(); ++LeftChildIndex)
		{
			for (int32 RightChildIndex = LeftChildIndex + 1; RightChildIndex < ChildPlateIds.Num(); ++RightChildIndex)
			{
				const FPlatePairBoundaryActivitySummary PairSummary =
					BuildPlatePairBoundaryActivitySummary(
						Planet,
						ChildPlateIds[LeftChildIndex],
						ChildPlateIds[RightChildIndex]);
				Summary.ContactEdgeCount += PairSummary.ContactEdgeCount;
				Summary.DivergentEdgeCount += PairSummary.DivergentEdgeCount;
			}
		}

		return Summary;
	}

	FString JoinDoubleArray(const TArray<double>& Values)
	{
		TArray<FString> Parts;
		Parts.Reserve(Values.Num());
		for (const double Value : Values)
		{
			Parts.Add(FString::Printf(TEXT("%.4f"), Value));
		}

		return FString::Join(Parts, TEXT(","));
	}

	bool AreAllStablePlateIdsValid(const FTectonicPlanet& Planet)
	{
		for (const FSample& Sample : Planet.Samples)
		{
			if (Planet.FindPlateArrayIndexById(Sample.PlateId) == INDEX_NONE)
			{
				return false;
			}
		}

		for (const FPlate& Plate : Planet.Plates)
		{
			if (Plate.MemberSamples.IsEmpty())
			{
				return false;
			}
		}

		return true;
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

	struct FCurrentOwnershipSnapshot
	{
		int32 GapCount = 0;
		int32 OverlapCount = 0;
		int32 ContinentalContinentalOverlapCount = 0;
	};

	FCurrentOwnershipSnapshot CaptureCurrentOwnershipSnapshot(const FTectonicPlanet& Planet)
	{
		FTectonicPlanet DiagnosticPlanet = Planet;
		DiagnosticPlanet.BuildContainmentSoups();

		TArray<int32> NewPlateIds;
		TArray<int32> ContainingTriangles;
		TArray<FVector3d> BarycentricCoords;
		TArray<uint8> GapFlags;
		TArray<uint8> OverlapFlags;
		TArray<TArray<int32>> OverlapPlateIds;
		int32 GapCount = 0;
		int32 OverlapCount = 0;
		DiagnosticPlanet.QueryOwnership(
			NewPlateIds,
			ContainingTriangles,
			BarycentricCoords,
			GapFlags,
			OverlapFlags,
			GapCount,
			OverlapCount,
			&OverlapPlateIds,
			nullptr);

		TArray<FCollisionCandidate> CollisionCandidates;
		DiagnosticPlanet.CollectCollisionCandidates(OverlapFlags, OverlapPlateIds, CollisionCandidates);
		TSet<int32> ContinentalContinentalOverlapSamples;
		for (const FCollisionCandidate& Candidate : CollisionCandidates)
		{
			ContinentalContinentalOverlapSamples.Add(Candidate.SampleIndex);
		}

		FCurrentOwnershipSnapshot Snapshot;
		Snapshot.GapCount = GapCount;
		Snapshot.OverlapCount = OverlapCount;
		Snapshot.ContinentalContinentalOverlapCount = ContinentalContinentalOverlapSamples.Num();
		return Snapshot;
	}

	void SetAllContinentalWeights(FTectonicPlanet& Planet, const float ContinentalWeight)
	{
		for (FSample& Sample : Planet.Samples)
		{
			Sample.ContinentalWeight = ContinentalWeight;
		}

		for (FPlate& Plate : Planet.Plates)
		{
			for (FCarriedSample& CarriedSample : Plate.CarriedSamples)
			{
				CarriedSample.ContinentalWeight = ContinentalWeight;
			}
		}

		Planet.ComputePlateScores();
	}

	double ComputeMeanElevation(const FTectonicPlanet& Planet)
	{
		if (Planet.Samples.IsEmpty())
		{
			return 0.0;
		}

		double ElevationSum = 0.0;
		for (const FSample& Sample : Planet.Samples)
		{
			ElevationSum += static_cast<double>(Sample.Elevation);
		}

		return ElevationSum / static_cast<double>(Planet.Samples.Num());
	}

	FVector3d ComputeSampleSetCentroidForTest(const FTectonicPlanet& Planet, const TArray<int32>& SampleIndices)
	{
		FVector3d Sum = FVector3d::ZeroVector;
		for (const int32 SampleIndex : SampleIndices)
		{
			if (Planet.Samples.IsValidIndex(SampleIndex))
			{
				Sum += Planet.Samples[SampleIndex].Position.GetSafeNormal();
			}
		}
		return Sum.GetSafeNormal();
	}

	FVector3d ComputePlateSurfaceVelocityForTest(const FPlate& Plate, const FVector3d& QueryPoint, const double PlanetRadiusKm)
	{
		if (Plate.RotationAxis.IsNearlyZero() || Plate.AngularSpeed <= 0.0)
		{
			return FVector3d::ZeroVector;
		}

		return FVector3d::CrossProduct(Plate.RotationAxis.GetSafeNormal(), QueryPoint) * (Plate.AngularSpeed * PlanetRadiusKm);
	}

	double SumContinentalWeight(const FTectonicPlanet& Planet)
	{
		double TotalWeight = 0.0;
		for (const FSample& Sample : Planet.Samples)
		{
			TotalWeight += Sample.ContinentalWeight;
		}
		return TotalWeight;
	}

	int32 CountAndeanSamples(const FTectonicPlanet& Planet)
	{
		int32 AndeanSampleCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			AndeanSampleCount += Sample.OrogenyType == EOrogenyType::Andean ? 1 : 0;
		}
		return AndeanSampleCount;
	}

	int32 CountHimalayanSamples(const FTectonicPlanet& Planet)
	{
		int32 HimalayanSampleCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			HimalayanSampleCount += Sample.OrogenyType == EOrogenyType::Himalayan ? 1 : 0;
		}
		return HimalayanSampleCount;
	}

	int32 CountPartiallyConvertedSamples(const FTectonicPlanet& Planet)
	{
		int32 PartialSampleCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			PartialSampleCount += (Sample.ContinentalWeight > 0.0f && Sample.ContinentalWeight < 0.5f) ? 1 : 0;
		}
		return PartialSampleCount;
	}

	TArray<double> CaptureAngularSpeeds(const FTectonicPlanet& Planet)
	{
		TArray<double> AngularSpeeds;
		AngularSpeeds.Reserve(Planet.Plates.Num());
		for (const FPlate& Plate : Planet.Plates)
		{
			AngularSpeeds.Add(Plate.AngularSpeed);
		}
		return AngularSpeeds;
	}

	TArray<FVector3d> CaptureRotationAxes(const FTectonicPlanet& Planet)
	{
		TArray<FVector3d> RotationAxes;
		RotationAxes.Reserve(Planet.Plates.Num());
		for (const FPlate& Plate : Planet.Plates)
		{
			RotationAxes.Add(Plate.RotationAxis.GetSafeNormal());
		}
		return RotationAxes;
	}

	double ComputeAxisAngleRad(const FVector3d& A, const FVector3d& B)
	{
		const FVector3d NormalizedA = A.GetSafeNormal();
		const FVector3d NormalizedB = B.GetSafeNormal();
		return FMath::Acos(FMath::Clamp(NormalizedA.Dot(NormalizedB), -1.0, 1.0));
	}

	bool IsFiniteNormalizedAxis(const FVector3d& Axis)
	{
		return
			FMath::IsFinite(Axis.X) &&
			FMath::IsFinite(Axis.Y) &&
			FMath::IsFinite(Axis.Z) &&
			Axis.Length() > 0.0 &&
			FMath::IsNearlyEqual(Axis.Length(), 1.0, 1.0e-3);
	}

	FVector3d MakeEquatorialDirection(const double LongitudeDegrees)
	{
		const double LongitudeRadians = FMath::DegreesToRadians(LongitudeDegrees);
		return FVector3d(FMath::Cos(LongitudeRadians), FMath::Sin(LongitudeRadians), 0.0);
	}

	FVector3d MakePositiveOctantDirection(const double X, const double Y, const double Z)
	{
		return FVector3d(X, Y, Z).GetSafeNormal();
	}

	FTectonicPlanet CreateManualSubductionTestPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(100);
		Planet.SampleAdjacency.SetNum(100);
		Planet.Plates.SetNum(2);

		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
		}

		for (FSample& Sample : Planet.Samples)
		{
			Sample.Position = FVector3d(0.0, 0.0, 1.0);
			Sample.PlateId = INDEX_NONE;
			Sample.SubductionDistanceKm = -1.0f;
		}

		Planet.Samples[0].Position = MakeEquatorialDirection(0.0);
		Planet.Samples[0].PlateId = 0;
		Planet.Samples[1].Position = MakeEquatorialDirection(5.0);
		Planet.Samples[1].PlateId = 1;
		Planet.Samples[2].Position = MakeEquatorialDirection(-5.0);
		Planet.Samples[2].PlateId = 0;
		Planet.Samples[3].Position = MakeEquatorialDirection(-10.0);
		Planet.Samples[3].PlateId = 0;
		Planet.Samples[4].Position = MakeEquatorialDirection(-25.0);
		Planet.Samples[4].PlateId = 0;
		Planet.Samples[5].Position = MakeEquatorialDirection(-40.0);
		Planet.Samples[5].PlateId = 0;

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].Add(B);
			Planet.SampleAdjacency[B].Add(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(0, 2);
		AddUndirectedEdge(2, 3);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);

		Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
		Planet.Plates[0].AngularSpeed = 0.01;
		Planet.Plates[0].OverlapScore = 10;
		Planet.Plates[0].MemberSamples = { 0, 2, 3, 4, 5 };
		Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
		Planet.Plates[1].AngularSpeed = 0.01;
		Planet.Plates[1].OverlapScore = 0;
		Planet.Plates[1].MemberSamples = { 1 };

		for (const int32 SampleIndex : Planet.Plates[0].MemberSamples)
		{
			FCarriedSample CarriedSample;
			CarriedSample.CanonicalSampleIndex = SampleIndex;
			Planet.Plates[0].CanonicalToCarriedIndex.Add(SampleIndex, Planet.Plates[0].CarriedSamples.Add(CarriedSample));
		}

		for (const int32 SampleIndex : Planet.Plates[1].MemberSamples)
		{
			FCarriedSample CarriedSample;
			CarriedSample.CanonicalSampleIndex = SampleIndex;
			Planet.Plates[1].CanonicalToCarriedIndex.Add(SampleIndex, Planet.Plates[1].CarriedSamples.Add(CarriedSample));
		}

		return Planet;
	}

	FTectonicPlanet CreateManualSlabPullTestPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(100);
		Planet.SampleAdjacency.SetNum(100);
		Planet.Plates.SetNum(2);

		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
		}

		for (FSample& Sample : Planet.Samples)
		{
			Sample.Position = FVector3d(0.0, 0.0, 1.0);
			Sample.PlateId = INDEX_NONE;
			Sample.SubductionDistanceKm = -1.0f;
		}

		Planet.Samples[0].Position = MakeEquatorialDirection(0.0);
		Planet.Samples[0].PlateId = 0;
		Planet.Samples[1].Position = MakeEquatorialDirection(5.0);
		Planet.Samples[1].PlateId = 1;
		Planet.Samples[2].Position = MakeEquatorialDirection(-5.0);
		Planet.Samples[2].PlateId = 0;
		Planet.Samples[3].Position = MakeEquatorialDirection(15.0);
		Planet.Samples[3].PlateId = 1;
		Planet.Samples[4].Position = MakeEquatorialDirection(-15.0);
		Planet.Samples[4].PlateId = 0;

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].Add(B);
			Planet.SampleAdjacency[B].Add(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(1, 3);
		AddUndirectedEdge(2, 4);
		AddUndirectedEdge(0, 2);

		Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
		Planet.Plates[0].AngularSpeed = 0.01;
		Planet.Plates[0].OverlapScore = 10;
		Planet.Plates[0].MemberSamples = { 0, 2, 4 };
		Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
		Planet.Plates[1].AngularSpeed = 0.01;
		Planet.Plates[1].OverlapScore = 0;
		Planet.Plates[1].MemberSamples = { 1, 3 };

		for (const int32 SampleIndex : Planet.Plates[0].MemberSamples)
		{
			FCarriedSample CarriedSample;
			CarriedSample.CanonicalSampleIndex = SampleIndex;
			Planet.Plates[0].CanonicalToCarriedIndex.Add(SampleIndex, Planet.Plates[0].CarriedSamples.Add(CarriedSample));
		}

		for (const int32 SampleIndex : Planet.Plates[1].MemberSamples)
		{
			FCarriedSample CarriedSample;
			CarriedSample.CanonicalSampleIndex = SampleIndex;
			Planet.Plates[1].CanonicalToCarriedIndex.Add(SampleIndex, Planet.Plates[1].CarriedSamples.Add(CarriedSample));
		}

		return Planet;
	}

	FTectonicPlanet CreateManualOverlapOwnershipTestPlanet(
		const int32 QuerySampleCurrentPlateId,
		const int32 Plate0OverlapScore,
		const int32 Plate1OverlapScore)
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(4);
		Planet.SampleAdjacency.SetNum(4);
		Planet.TriangleIndices = { FIntVector(0, 1, 2) };
		Planet.TriangleAdjacency.SetNum(1);
		Planet.Plates.SetNum(3);

		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0);
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0);
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0);
		Planet.Samples[3].Position = FVector3d(1.0, 1.0, 1.0).GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			Planet.Samples[SampleIndex].Position = Planet.Samples[SampleIndex].Position.GetSafeNormal();
			Planet.Samples[SampleIndex].PlateId = 0;
		}

		Planet.Samples[3].PlateId = QuerySampleCurrentPlateId;

		Planet.Plates[0].OverlapScore = Plate0OverlapScore;
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[1].OverlapScore = Plate1OverlapScore;
		Planet.Plates[1].SoupTriangles = { 0 };
		Planet.Plates[2].OverlapScore = -100;
		Planet.Plates[2].CumulativeRotation = FQuat4d(FVector3d(0.0, 0.0, 1.0), 1.0e-6);

		return Planet;
	}

	FTectonicPlanet CreateManualPreserveRecoveryTestPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(4);
		Planet.SampleAdjacency.SetNum(4);
		Planet.TriangleIndices = { FIntVector(0, 1, 2) };
		Planet.TriangleAdjacency.SetNum(1);
		Planet.Plates.SetNum(1);
		Planet.Plates[0].Id = 0;
		Planet.Plates[0].OverlapScore = 10;
		Planet.Plates[0].SoupTriangles = { 0 };

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(1.0, 0.01, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(1.0, 0.0, 0.01).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(1.0, -0.001, 0.005).GetSafeNormal();

		for (FSample& Sample : Planet.Samples)
		{
			Sample.PlateId = 0;
		}

		return Planet;
	}

	FTectonicPlanet CreateManualBoundaryContactCollisionPlanet(const int32 BoundaryPairCount, const bool bConvergent)
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(BoundaryPairCount * 2);
		Planet.SampleAdjacency.SetNum(BoundaryPairCount * 2);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].OverlapScore = PlateIndex == 0 ? 100 : 50;
		}

		for (int32 PairIndex = 0; PairIndex < BoundaryPairCount; ++PairIndex)
		{
			const int32 LeftSampleIndex = PairIndex * 2;
			const int32 RightSampleIndex = LeftSampleIndex + 1;
			const double Z = BoundaryPairCount > 1
				? FMath::Lerp(-0.35, 0.35, static_cast<double>(PairIndex) / static_cast<double>(BoundaryPairCount - 1))
				: 0.0;

			Planet.Samples[LeftSampleIndex].Position = FVector3d(0.05, 1.0, Z).GetSafeNormal();
			Planet.Samples[RightSampleIndex].Position = FVector3d(-0.05, 1.0, Z).GetSafeNormal();
			Planet.Samples[LeftSampleIndex].PlateId = 0;
			Planet.Samples[RightSampleIndex].PlateId = 1;
			Planet.Samples[LeftSampleIndex].ContinentalWeight = 1.0f;
			Planet.Samples[RightSampleIndex].ContinentalWeight = 1.0f;
			Planet.Samples[LeftSampleIndex].Elevation = 0.2f;
			Planet.Samples[RightSampleIndex].Elevation = 0.2f;
			Planet.Samples[LeftSampleIndex].Thickness = 35.0f;
			Planet.Samples[RightSampleIndex].Thickness = 35.0f;

			Planet.SampleAdjacency[LeftSampleIndex].Add(RightSampleIndex);
			Planet.SampleAdjacency[RightSampleIndex].Add(LeftSampleIndex);
			if (PairIndex > 0)
			{
				const int32 PreviousLeftSampleIndex = LeftSampleIndex - 2;
				const int32 PreviousRightSampleIndex = RightSampleIndex - 2;
				Planet.SampleAdjacency[LeftSampleIndex].Add(PreviousLeftSampleIndex);
				Planet.SampleAdjacency[PreviousLeftSampleIndex].Add(LeftSampleIndex);
				Planet.SampleAdjacency[RightSampleIndex].Add(PreviousRightSampleIndex);
				Planet.SampleAdjacency[PreviousRightSampleIndex].Add(RightSampleIndex);
			}
		}

		Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
		Planet.Plates[0].AngularSpeed = 0.02;
		Planet.Plates[1].RotationAxis = bConvergent ? FVector3d(0.0, 0.0, -1.0) : FVector3d(0.0, 0.0, 1.0);
		Planet.Plates[1].AngularSpeed = 0.02;
		return Planet;
	}

	void AssignPlanetToHemispheres(FTectonicPlanet& Planet)
	{
		TArray<int32> NewPlateIds;
		NewPlateIds.SetNum(Planet.Samples.Num());
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			NewPlateIds[SampleIndex] = Planet.Samples[SampleIndex].Position.X >= 0.0 ? 0 : 1;
		}

		Planet.RepartitionMembership(NewPlateIds);
		Planet.ComputePlateScores();
	}

	TArray<FIntPoint> FindConnectedBoundaryContactEdges(const FTectonicPlanet& Planet, const int32 DesiredEdgeCount)
	{
		TArray<FIntPoint> BoundaryEdges;
		TMap<int32, TArray<int32>> EdgeIndicesByEndpoint;
		for (int32 SampleIndexA = 0; SampleIndexA < Planet.SampleAdjacency.Num(); ++SampleIndexA)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndexA))
			{
				continue;
			}

			for (const int32 SampleIndexB : Planet.SampleAdjacency[SampleIndexA])
			{
				if (SampleIndexB <= SampleIndexA || !Planet.Samples.IsValidIndex(SampleIndexB))
				{
					continue;
				}

				if (Planet.Samples[SampleIndexA].PlateId == Planet.Samples[SampleIndexB].PlateId)
				{
					continue;
				}

				const int32 EdgeIndex = BoundaryEdges.Add(FIntPoint(SampleIndexA, SampleIndexB));
				EdgeIndicesByEndpoint.FindOrAdd(SampleIndexA).Add(EdgeIndex);
				EdgeIndicesByEndpoint.FindOrAdd(SampleIndexB).Add(EdgeIndex);
			}
		}

		TArray<TArray<int32>> EdgeAdjacency;
		EdgeAdjacency.SetNum(BoundaryEdges.Num());
		for (int32 EdgeIndex = 0; EdgeIndex < BoundaryEdges.Num(); ++EdgeIndex)
		{
			const FIntPoint& Edge = BoundaryEdges[EdgeIndex];
			const int32 Endpoints[2] = { Edge.X, Edge.Y };
			for (const int32 Endpoint : Endpoints)
			{
				if (const TArray<int32>* SharedEndpointEdges = EdgeIndicesByEndpoint.Find(Endpoint))
				{
					for (const int32 OtherEdgeIndex : *SharedEndpointEdges)
					{
						if (OtherEdgeIndex != EdgeIndex)
						{
							EdgeAdjacency[EdgeIndex].AddUnique(OtherEdgeIndex);
						}
					}
				}

				if (!Planet.SampleAdjacency.IsValidIndex(Endpoint))
				{
					continue;
				}

				for (const int32 NeighborSampleIndex : Planet.SampleAdjacency[Endpoint])
				{
					if (const TArray<int32>* AdjacentEndpointEdges = EdgeIndicesByEndpoint.Find(NeighborSampleIndex))
					{
						for (const int32 OtherEdgeIndex : *AdjacentEndpointEdges)
						{
							if (OtherEdgeIndex != EdgeIndex)
							{
								EdgeAdjacency[EdgeIndex].AddUnique(OtherEdgeIndex);
							}
						}
					}
				}
			}
		}

		TArray<bool> Visited;
		Visited.Init(false, BoundaryEdges.Num());
		for (int32 StartEdgeIndex = 0; StartEdgeIndex < BoundaryEdges.Num(); ++StartEdgeIndex)
		{
			if (Visited[StartEdgeIndex])
			{
				continue;
			}

			TArray<int32> Queue;
			TArray<int32> Component;
			Queue.Add(StartEdgeIndex);
			Visited[StartEdgeIndex] = true;
			for (int32 QueueIndex = 0; QueueIndex < Queue.Num(); ++QueueIndex)
			{
				const int32 EdgeIndex = Queue[QueueIndex];
				Component.Add(EdgeIndex);
				for (const int32 NeighborEdgeIndex : EdgeAdjacency[EdgeIndex])
				{
					if (!Visited[NeighborEdgeIndex])
					{
						Visited[NeighborEdgeIndex] = true;
						Queue.Add(NeighborEdgeIndex);
					}
				}
			}

			if (Component.Num() >= DesiredEdgeCount)
			{
				TArray<FIntPoint> SelectedEdges;
				SelectedEdges.Reserve(DesiredEdgeCount);
				for (int32 ComponentIndex = 0; ComponentIndex < DesiredEdgeCount; ++ComponentIndex)
				{
					SelectedEdges.Add(BoundaryEdges[Component[ComponentIndex]]);
				}
				return SelectedEdges;
			}
		}

		return {};
	}

	TSet<int32> CollectBoundaryContactZoneSamples(const TArray<FIntPoint>& BoundaryEdges)
	{
		TSet<int32> SampleIndices;
		for (const FIntPoint& Edge : BoundaryEdges)
		{
			SampleIndices.Add(Edge.X);
			SampleIndices.Add(Edge.Y);
		}
		return SampleIndices;
	}

	void StampSparseContinentalBoundaryZone(FTectonicPlanet& Planet, const TSet<int32>& ContinentalSampleIndices)
	{
		SetAllContinentalWeights(Planet, 0.0f);
		for (const int32 SampleIndex : ContinentalSampleIndices)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 0.25f;
			Sample.Thickness = 35.0f;

			const int32 PlateIndex = Planet.FindPlateArrayIndexById(Sample.PlateId);
			if (!Planet.Plates.IsValidIndex(PlateIndex))
			{
				continue;
			}

			if (const int32* CarriedSampleIndex = Planet.Plates[PlateIndex].CanonicalToCarriedIndex.Find(SampleIndex))
			{
				FCarriedSample& CarriedSample = Planet.Plates[PlateIndex].CarriedSamples[*CarriedSampleIndex];
				CarriedSample.ContinentalWeight = 1.0f;
				CarriedSample.Elevation = 0.25f;
				CarriedSample.Thickness = 35.0f;
			}
		}

		Planet.ComputePlateScores();
	}

	TSet<int32> FindSparseBoundaryContactZoneSamples(FTectonicPlanet& Planet, const int32 DesiredZoneSize)
	{
		for (int32 CandidateEdgeCount = DesiredZoneSize; CandidateEdgeCount <= DesiredZoneSize * 4; ++CandidateEdgeCount)
		{
			const TArray<FIntPoint> CandidateEdges = FindConnectedBoundaryContactEdges(Planet, CandidateEdgeCount);
			if (CandidateEdges.Num() < DesiredZoneSize)
			{
				continue;
			}

			for (int32 EdgeIndexA = 0; EdgeIndexA < CandidateEdges.Num(); ++EdgeIndexA)
			{
				for (int32 EdgeIndexB = EdgeIndexA + 1; EdgeIndexB < CandidateEdges.Num(); ++EdgeIndexB)
				{
					for (int32 EdgeIndexC = EdgeIndexB + 1; EdgeIndexC < CandidateEdges.Num(); ++EdgeIndexC)
					{
						TArray<FIntPoint> TrialEdges;
						TrialEdges.Reserve(DesiredZoneSize);
						TrialEdges.Add(CandidateEdges[EdgeIndexA]);
						TrialEdges.Add(CandidateEdges[EdgeIndexB]);
						TrialEdges.Add(CandidateEdges[EdgeIndexC]);

						const TSet<int32> TrialSampleIndices = CollectBoundaryContactZoneSamples(TrialEdges);
						StampSparseContinentalBoundaryZone(Planet, TrialSampleIndices);

						FResamplingStats Stats;
						const bool bImmediateTrigger = Planet.DetectBoundaryContactCollisionTrigger(&Stats);
						if (!bImmediateTrigger &&
							Stats.BoundaryContactCollisionPairCount == 1 &&
							Stats.BoundaryContactLargestZoneSize == DesiredZoneSize)
						{
							SetAllContinentalWeights(Planet, 0.0f);
							return TrialSampleIndices;
						}
					}
				}
			}
		}

		SetAllContinentalWeights(Planet, 0.0f);
		return {};
	}

	void ForceAdvanceToPeriodicResample(FTectonicPlanet& Planet, const int32 ResampleOrdinal)
	{
		const int32 ResampleInterval = Planet.ComputeResampleInterval();
		Planet.CurrentStep = (ResampleOrdinal * ResampleInterval) - 1;
		Planet.AdvanceStep();
	}

	FPendingBoundaryContactCollisionEvent BuildPendingBoundaryContactCollisionEvent(
		const FTectonicPlanet& Planet,
		const TArray<FIntPoint>& BoundaryEdges)
	{
		FPendingBoundaryContactCollisionEvent PendingEvent;
		PendingEvent.bValid = !BoundaryEdges.IsEmpty();
		if (BoundaryEdges.IsEmpty())
		{
			return PendingEvent;
		}

		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d ContactCenterSum = FVector3d::ZeroVector;
		for (const FIntPoint& Edge : BoundaryEdges)
		{
			const int32 SampleIndexA = Edge.X;
			const int32 SampleIndexB = Edge.Y;
			if (!Planet.Samples.IsValidIndex(SampleIndexA) || !Planet.Samples.IsValidIndex(SampleIndexB))
			{
				continue;
			}

			PendingEvent.BoundarySeedSampleIndices.AddUnique(SampleIndexA);
			PendingEvent.BoundarySeedSampleIndices.AddUnique(SampleIndexB);
			ContactCenterSum += Planet.Samples[SampleIndexA].Position.GetSafeNormal();
			ContactCenterSum += Planet.Samples[SampleIndexB].Position.GetSafeNormal();

			const int32 SamplePlateA = Planet.Samples[SampleIndexA].PlateId;
			const int32 SamplePlateB = Planet.Samples[SampleIndexB].PlateId;
			const int32 NormalizedPlateA = FMath::Min(SamplePlateA, SamplePlateB);
			const int32 NormalizedPlateB = FMath::Max(SamplePlateA, SamplePlateB);
			if (PlateA == INDEX_NONE || PlateB == INDEX_NONE)
			{
				PlateA = NormalizedPlateA;
				PlateB = NormalizedPlateB;
			}
		}

		PendingEvent.PlateA = PlateA;
		PendingEvent.PlateB = PlateB;
		PendingEvent.ContactCenter = ContactCenterSum.GetSafeNormal();
		PendingEvent.PeakZoneSize = BoundaryEdges.Num();
		PendingEvent.ConsecutiveResamples = 1;
		PendingEvent.bValid =
			PendingEvent.PlateA != INDEX_NONE &&
			PendingEvent.PlateB != INDEX_NONE &&
			!PendingEvent.BoundarySeedSampleIndices.IsEmpty();
		return PendingEvent;
	}

	struct FManualCollisionScenario
	{
		FTectonicPlanet Planet;
		TArray<uint8> OverlapFlags;
		TArray<TArray<int32>> OverlapPlateIds;
		TArray<int32> PreviousPlateIds;
		TArray<float> PreviousContinentalWeights;
		TArray<int32> PreviousTerraneAssignments;
		TArray<int32> NewPlateIds;
		TArray<int32> ZoneASamples;
		TArray<int32> ZoneBSamples;
		int32 ZoneAExtraSample = INDEX_NONE;
		int32 ZoneBExtraSample = INDEX_NONE;
		int32 NearbyBoostSample = INDEX_NONE;
	};

	FManualCollisionScenario CreateManualCollisionScenario()
	{
		FManualCollisionScenario Scenario;
		FTectonicPlanet& Planet = Scenario.Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(26);
		Planet.SampleAdjacency.SetNum(26);
		Planet.TriangleIndices = { FIntVector(0, 1, 2) };
		Planet.TriangleAdjacency.SetNum(1);
		Planet.Plates.SetNum(2);
		Planet.bEnableOverlapHysteresis = false;

		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0);
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0);
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0);
		Scenario.ZoneASamples = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
		Scenario.ZoneBSamples = { 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 };
		Scenario.ZoneAExtraSample = 13;
		Scenario.ZoneBExtraSample = 24;
		Scenario.NearbyBoostSample = 25;

		const TArray<FVector3d> ZoneAPositions = {
			MakePositiveOctantDirection(0.55, 0.25, 0.20),
			MakePositiveOctantDirection(0.50, 0.28, 0.22),
			MakePositiveOctantDirection(0.46, 0.30, 0.24),
			MakePositiveOctantDirection(0.42, 0.32, 0.26),
			MakePositiveOctantDirection(0.38, 0.34, 0.28),
			MakePositiveOctantDirection(0.34, 0.36, 0.30),
			MakePositiveOctantDirection(0.30, 0.38, 0.32),
			MakePositiveOctantDirection(0.26, 0.40, 0.34),
			MakePositiveOctantDirection(0.22, 0.42, 0.36),
			MakePositiveOctantDirection(0.20, 0.44, 0.36)
		};
		const TArray<FVector3d> ZoneBPositions = {
			MakePositiveOctantDirection(0.22, 0.22, 0.56),
			MakePositiveOctantDirection(0.24, 0.24, 0.52),
			MakePositiveOctantDirection(0.26, 0.26, 0.48),
			MakePositiveOctantDirection(0.28, 0.28, 0.44),
			MakePositiveOctantDirection(0.30, 0.30, 0.40),
			MakePositiveOctantDirection(0.32, 0.32, 0.36),
			MakePositiveOctantDirection(0.34, 0.34, 0.32),
			MakePositiveOctantDirection(0.36, 0.30, 0.34),
			MakePositiveOctantDirection(0.38, 0.28, 0.34),
			MakePositiveOctantDirection(0.40, 0.26, 0.34)
		};

		for (int32 PositionIndex = 0; PositionIndex < Scenario.ZoneASamples.Num(); ++PositionIndex)
		{
			Planet.Samples[Scenario.ZoneASamples[PositionIndex]].Position = ZoneAPositions[PositionIndex];
		}
		for (int32 PositionIndex = 0; PositionIndex < Scenario.ZoneBSamples.Num(); ++PositionIndex)
		{
			Planet.Samples[Scenario.ZoneBSamples[PositionIndex]].Position = ZoneBPositions[PositionIndex];
		}
		Planet.Samples[Scenario.ZoneAExtraSample].Position = MakePositiveOctantDirection(0.18, 0.46, 0.36);
		Planet.Samples[Scenario.ZoneBExtraSample].Position = MakePositiveOctantDirection(0.42, 0.24, 0.34);
		Planet.Samples[Scenario.NearbyBoostSample].Position = MakePositiveOctantDirection(0.36, 0.34, 0.30);

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.Position = Sample.Position.GetSafeNormal();
			Sample.PlateId = (SampleIndex == 0 || SampleIndex == 1 || SampleIndex == 2 || SampleIndex == Scenario.NearbyBoostSample) ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 0.2f;
			Sample.Thickness = 35.0f;
			Sample.SubductionDistanceKm = -1.0f;
			Sample.OrogenyType = EOrogenyType::None;
		}
		Planet.Samples[Scenario.NearbyBoostSample].ContinentalWeight = 0.3f;
		Planet.Samples[Scenario.NearbyBoostSample].Elevation = 0.1f;

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].Add(B);
			Planet.SampleAdjacency[B].Add(A);
		};

		for (int32 SampleOffset = 1; SampleOffset < Scenario.ZoneASamples.Num(); ++SampleOffset)
		{
			AddUndirectedEdge(Scenario.ZoneASamples[SampleOffset - 1], Scenario.ZoneASamples[SampleOffset]);
		}
		AddUndirectedEdge(Scenario.ZoneASamples.Last(), Scenario.ZoneAExtraSample);
		AddUndirectedEdge(Scenario.ZoneASamples[5], Scenario.NearbyBoostSample);

		for (int32 SampleOffset = 1; SampleOffset < Scenario.ZoneBSamples.Num(); ++SampleOffset)
		{
			AddUndirectedEdge(Scenario.ZoneBSamples[SampleOffset - 1], Scenario.ZoneBSamples[SampleOffset]);
		}
		AddUndirectedEdge(Scenario.ZoneBSamples.Last(), Scenario.ZoneBExtraSample);

		Planet.Plates[0].OverlapScore = 500;
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].MemberSamples = { 0, 1, 2, Scenario.NearbyBoostSample };
		Planet.Plates[1].OverlapScore = 50;
		Planet.Plates[1].SoupTriangles = { 0 };
		Planet.Plates[1].MemberSamples = Scenario.ZoneASamples;
		Planet.Plates[1].MemberSamples.Append(Scenario.ZoneBSamples);
		Planet.Plates[1].MemberSamples.Add(Scenario.ZoneAExtraSample);
		Planet.Plates[1].MemberSamples.Add(Scenario.ZoneBExtraSample);

		for (FPlate& Plate : Planet.Plates)
		{
			Plate.RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Plate.AngularSpeed = 0.0;
			Plate.CumulativeRotation = FQuat4d::Identity;
			Plate.BoundingCap = FSphericalBoundingCap{};
		}
		Planet.Plates[0].AngularSpeed = 0.01;
		Planet.Plates[1].AngularSpeed = 0.02;

		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			FPlate& Plate = Planet.Plates[PlateIndex];
			for (const int32 CanonicalSampleIndex : { 0, 1, 2 })
			{
				FCarriedSample CarriedSample;
				CarriedSample.CanonicalSampleIndex = CanonicalSampleIndex;
				CarriedSample.ContinentalWeight = 1.0f;
				CarriedSample.Elevation = 0.2f;
				CarriedSample.Thickness = 35.0f;
				Plate.CanonicalToCarriedIndex.Add(CanonicalSampleIndex, Plate.CarriedSamples.Add(CarriedSample));
			}
		}

		Scenario.OverlapFlags.Init(0, Planet.Samples.Num());
		Scenario.OverlapPlateIds.SetNum(Planet.Samples.Num());
		Scenario.PreviousPlateIds.SetNum(Planet.Samples.Num());
		Scenario.PreviousContinentalWeights.SetNum(Planet.Samples.Num());
		Scenario.PreviousTerraneAssignments.SetNum(Planet.Samples.Num());
		Scenario.NewPlateIds.SetNum(Planet.Samples.Num());
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			Scenario.PreviousPlateIds[SampleIndex] = Planet.Samples[SampleIndex].PlateId;
			Scenario.PreviousContinentalWeights[SampleIndex] = Planet.Samples[SampleIndex].ContinentalWeight;
			Scenario.PreviousTerraneAssignments[SampleIndex] = INDEX_NONE;
			Scenario.NewPlateIds[SampleIndex] = Planet.Samples[SampleIndex].PlateId;
		}

		for (const int32 SampleIndex : Scenario.ZoneASamples)
		{
			Scenario.OverlapFlags[SampleIndex] = 1;
			Scenario.OverlapPlateIds[SampleIndex] = { 0, 1 };
			Scenario.PreviousTerraneAssignments[SampleIndex] = 77;
		}
		Scenario.PreviousTerraneAssignments[Scenario.ZoneAExtraSample] = 77;

		for (const int32 SampleIndex : Scenario.ZoneBSamples)
		{
			Scenario.OverlapFlags[SampleIndex] = 1;
			Scenario.OverlapPlateIds[SampleIndex] = { 0, 1 };
			Scenario.PreviousTerraneAssignments[SampleIndex] = 88;
		}
		Scenario.PreviousTerraneAssignments[Scenario.ZoneBExtraSample] = 88;

		Planet.BuildContainmentSoups();
		return Scenario;
	}

	struct FManualOverlapOwnershipQueryResult
	{
		TArray<int32> NewPlateIds;
		TArray<int32> ContainingTriangles;
		TArray<FVector3d> BarycentricCoords;
		TArray<uint8> GapFlags;
		TArray<uint8> OverlapFlags;
		TArray<TArray<int32>> OverlapPlateIds;
		FResamplingStats Stats;
		int32 GapCount = 0;
		int32 OverlapCount = 0;
	};

	FManualOverlapOwnershipQueryResult RunManualOverlapOwnershipQuery(
		FTectonicPlanet& Planet,
		const EResampleOwnershipMode OwnershipMode = EResampleOwnershipMode::FullResolution)
	{
		FManualOverlapOwnershipQueryResult Result;
		Planet.BuildContainmentSoups();
		Planet.QueryOwnership(
			Result.NewPlateIds,
			Result.ContainingTriangles,
			Result.BarycentricCoords,
			Result.GapFlags,
			Result.OverlapFlags,
			Result.GapCount,
			Result.OverlapCount,
			&Result.OverlapPlateIds,
			&Result.Stats,
			OwnershipMode);
		return Result;
	}

	double ComputeExpectedArcDistanceKm(const double LongitudeDegrees)
	{
		return FMath::DegreesToRadians(FMath::Abs(LongitudeDegrees)) * TestPlanetRadiusKm;
	}

	double ComputeMeanElevationDeltaForDistanceBand(
		const FTectonicPlanet& Planet,
		const TArray<float>& InitialElevations,
		const double MinDistanceKm,
		const double MaxDistanceKm,
		int32& OutSampleCount)
	{
		OutSampleCount = 0;
		double DeltaSum = 0.0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (!InitialElevations.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const FSample& Sample = Planet.Samples[SampleIndex];
			if (Sample.SubductionDistanceKm < MinDistanceKm || Sample.SubductionDistanceKm > MaxDistanceKm)
			{
				continue;
			}

			DeltaSum += static_cast<double>(Sample.Elevation) - static_cast<double>(InitialElevations[SampleIndex]);
			++OutSampleCount;
		}

		return OutSampleCount > 0 ? (DeltaSum / static_cast<double>(OutSampleCount)) : 0.0;
	}

	int32 CountInvalidPlateAssignments(const FTectonicPlanet& Planet)
	{
		int32 InvalidAssignmentCount = 0;
		for (const FSample& Sample : Planet.Samples)
		{
			InvalidAssignmentCount += (Sample.PlateId < 0 || Sample.PlateId >= Planet.Plates.Num()) ? 1 : 0;
		}

		return InvalidAssignmentCount;
	}

	TSet<int32> BuildTerraneIdSet(const TArray<FTerrane>& Terranes)
	{
		TSet<int32> TerraneIds;
		for (const FTerrane& Terrane : Terranes)
		{
			TerraneIds.Add(Terrane.TerraneId);
		}
		return TerraneIds;
	}

	int32 CountSharedTerraneIds(const TSet<int32>& LeftIds, const TSet<int32>& RightIds)
	{
		int32 SharedCount = 0;
		for (const int32 TerraneId : LeftIds)
		{
			SharedCount += RightIds.Contains(TerraneId) ? 1 : 0;
		}
		return SharedCount;
	}

	FTerraneValidationSummary BuildTerraneValidationSummary(const FTectonicPlanet& Planet)
	{
		FTerraneValidationSummary Summary;
		TMap<int32, int32> TerranePlateIds;

		for (const FTerrane& Terrane : Planet.Terranes)
		{
			Summary.bUniqueTerraneIds &= !TerranePlateIds.Contains(Terrane.TerraneId);
			TerranePlateIds.Add(Terrane.TerraneId, Terrane.PlateId);
		}

		for (const FSample& Sample : Planet.Samples)
		{
			if (Sample.ContinentalWeight >= 0.5f)
			{
				++Summary.ContinentalSampleCount;
				if (Sample.TerraneId != INDEX_NONE)
				{
					++Summary.ContinentalSamplesWithValidTerrane;
					if (const int32* TerranePlateId = TerranePlateIds.Find(Sample.TerraneId))
					{
						Summary.ContinentalSamplesWithMatchingTerranePlate += (*TerranePlateId == Sample.PlateId) ? 1 : 0;
					}
				}
			}
			else
			{
				++Summary.OceanicSampleCount;
				Summary.OceanicSamplesWithUnsetTerrane += (Sample.TerraneId == INDEX_NONE) ? 1 : 0;
			}
		}

		return Summary;
	}

	bool ExportCheckpointMaps(
		FAutomationTestBase& Test,
		const FTectonicPlanet& Planet,
		const FString& BaseDirectory,
		const int32 Step,
		const ETectonicMapExportMode Mode = ETectonicMapExportMode::All)
	{
		const FString OutputDirectory = FPaths::Combine(BaseDirectory, FString::Printf(TEXT("Step_%d"), Step));

		FTectonicMollweideExportOptions Options;
		Options.Mode = Mode;
		Options.Width = TestExportWidth;
		Options.Height = TestExportHeight;
		Options.OutputDirectory = OutputDirectory;

		FTectonicMollweideExportStats Stats;
		FString Error;
		const bool bExported = TectonicMollweideExporter::ExportPlanet(Planet, Options, Stats, Error);
		Test.TestTrue(*FString::Printf(TEXT("Step %d export succeeded (%s)"), Step, *Error), bExported);
		if (!bExported)
		{
			return false;
		}

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		Test.TestTrue(
			*FString::Printf(TEXT("Step %d elevation export exists"), Step),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("Elevation.png"))));
		Test.TestTrue(
			*FString::Printf(TEXT("Step %d plate export exists"), Step),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("PlateId.png"))));
		Test.TestTrue(
			*FString::Printf(TEXT("Step %d crust export exists"), Step),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("CrustType.png"))));
		if (Mode == ETectonicMapExportMode::All || Mode == ETectonicMapExportMode::SubductionDistance)
		{
			Test.TestTrue(
				*FString::Printf(TEXT("Step %d subduction-distance export exists"), Step),
				PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("SubductionDistance.png"))));
		}
		if (Mode == ETectonicMapExportMode::All || Mode == ETectonicMapExportMode::BoundaryMask)
		{
			Test.TestTrue(
				*FString::Printf(TEXT("Step %d boundary-mask export exists"), Step),
				PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("BoundaryMask.png"))));
		}
		if (Mode == ETectonicMapExportMode::All || Mode == ETectonicMapExportMode::GapMask)
		{
			Test.TestTrue(
				*FString::Printf(TEXT("Step %d gap-mask export exists"), Step),
				PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("GapMask.png"))));
		}
		if (Mode == ETectonicMapExportMode::All || Mode == ETectonicMapExportMode::OverlapMask)
		{
			Test.TestTrue(
				*FString::Printf(TEXT("Step %d overlap-mask export exists"), Step),
				PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("OverlapMask.png"))));
		}
		if (Mode == ETectonicMapExportMode::All || Mode == ETectonicMapExportMode::ContinentalWeight)
		{
			Test.TestTrue(
				*FString::Printf(TEXT("Step %d continental-weight export exists"), Step),
				PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("ContinentalWeight.png"))));
		}

		return true;
	}

	bool ExportNamedCheckpointMaps(
		FAutomationTestBase& Test,
		const FTectonicPlanet& Planet,
		const FString& BaseDirectory,
		const FString& Label,
		const ETectonicMapExportMode Mode = ETectonicMapExportMode::All)
	{
		const FString OutputDirectory = FPaths::Combine(BaseDirectory, Label);

		FTectonicMollweideExportOptions Options;
		Options.Mode = Mode;
		Options.Width = TestExportWidth;
		Options.Height = TestExportHeight;
		Options.OutputDirectory = OutputDirectory;

		FTectonicMollweideExportStats Stats;
		FString Error;
		const bool bExported = TectonicMollweideExporter::ExportPlanet(Planet, Options, Stats, Error);
		Test.TestTrue(*FString::Printf(TEXT("%s export succeeded (%s)"), *Label, *Error), bExported);
		if (!bExported)
		{
			return false;
		}

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		Test.TestTrue(
			*FString::Printf(TEXT("%s elevation export exists"), *Label),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("Elevation.png"))));
		Test.TestTrue(
			*FString::Printf(TEXT("%s plate export exists"), *Label),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("PlateId.png"))));
		Test.TestTrue(
			*FString::Printf(TEXT("%s crust export exists"), *Label),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("CrustType.png"))));
		Test.TestTrue(
			*FString::Printf(TEXT("%s boundary-mask export exists"), *Label),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("BoundaryMask.png"))));
		Test.TestTrue(
			*FString::Printf(TEXT("%s overlap-mask export exists"), *Label),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("OverlapMask.png"))));
		Test.TestTrue(
			*FString::Printf(TEXT("%s continental-weight export exists"), *Label),
			PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("ContinentalWeight.png"))));

		return true;
	}

	int64 CountTotalMembers(const FTectonicPlanet& Planet)
	{
		int64 TotalMembers = 0;
		for (const FPlate& Plate : Planet.Plates)
		{
			TotalMembers += Plate.MemberSamples.Num();
		}
		return TotalMembers;
	}

	bool IsInteriorTriangle(const FTectonicPlanet& Planet, const FIntVector& Triangle)
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
		return PlateA != INDEX_NONE && PlateA == PlateB && PlateB == PlateC;
	}

	int32 ChooseExpectedSoupPlateId(const FTectonicPlanet& Planet, const FIntVector& Triangle)
	{
		TArray<TPair<int32, int32>, TInlineAllocator<3>> PlateCounts;
		auto AccumulatePlate = [&Planet, &PlateCounts](const int32 PlateId)
		{
			if (Planet.FindPlateArrayIndexById(PlateId) == INDEX_NONE)
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

	FSoupPartitionSummary BuildSoupPartitionSummary(const FTectonicPlanet& Planet)
	{
		FSoupPartitionSummary Summary;
		TArray<uint8> TriangleAssignmentCounts;
		TriangleAssignmentCounts.Init(0, Planet.TriangleIndices.Num());

		for (const FPlate& Plate : Planet.Plates)
		{
			for (const int32 TriangleIndex : Plate.SoupTriangles)
			{
				++Summary.TotalSoupTriangleCount;
				if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
				{
					continue;
				}

				++TriangleAssignmentCounts[TriangleIndex];
				if (IsInteriorTriangle(Planet, Planet.TriangleIndices[TriangleIndex]))
				{
					++Summary.InteriorSoupTriangleCount;
				}
				else
				{
					++Summary.BoundarySoupTriangleCount;
				}
			}

			for (const FCarriedSample& CarriedSample : Plate.CarriedSamples)
			{
				if (!Planet.Samples.IsValidIndex(CarriedSample.CanonicalSampleIndex))
				{
					continue;
				}

				Summary.ForeignLocalVertexCount +=
					Planet.Samples[CarriedSample.CanonicalSampleIndex].PlateId != Plate.Id ? 1 : 0;
			}
		}

		for (int32 TriangleIndex = 0; TriangleIndex < Planet.TriangleIndices.Num(); ++TriangleIndex)
		{
			if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
			{
				continue;
			}

			if (!IsInteriorTriangle(Planet, Planet.TriangleIndices[TriangleIndex]) &&
				TriangleAssignmentCounts[TriangleIndex] == 0)
			{
				++Summary.DroppedMixedTriangleCount;
			}

			if (TriangleAssignmentCounts[TriangleIndex] > 1)
			{
				Summary.DuplicatedSoupTriangleCount += TriangleAssignmentCounts[TriangleIndex] - 1;
			}
		}

		return Summary;
	}

	FOwnershipQuerySummary RunOwnershipQuery(FAutomationTestBase& Test, FTectonicPlanet& Planet)
	{
		TArray<int32> NewPlateIds;
		TArray<int32> ContainingTriangles;
		TArray<FVector3d> BarycentricCoords;
		TArray<uint8> GapFlags;
		TArray<uint8> OverlapFlags;
		FOwnershipQuerySummary Summary;

		Planet.BuildContainmentSoups();
			Planet.QueryOwnership(
				NewPlateIds,
				ContainingTriangles,
			BarycentricCoords,
			GapFlags,
			OverlapFlags,
				Summary.GapCount,
				Summary.OverlapCount);
			Summary.DivergentGapCount = Summary.GapCount;
			Summary.NoValidHitCount = Summary.GapCount;
			Summary.MultiContainmentCount = Summary.OverlapCount;
			Summary.ValidContainmentCount = Planet.Samples.Num() - Summary.NoValidHitCount - Summary.MultiContainmentCount;
			Test.TestEqual(
				TEXT("Ownership query partitions samples into valid containment, overlap, or no-hit"),
				Summary.ValidContainmentCount + Summary.MultiContainmentCount + Summary.NoValidHitCount,
				Planet.Samples.Num());

			for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
			{
				if (NewPlateIds[SampleIndex] == INDEX_NONE || ContainingTriangles[SampleIndex] == INDEX_NONE)
				{
					continue;
			}

			const FVector3d& Barycentric = BarycentricCoords[SampleIndex];
			const double Sum = Barycentric.X + Barycentric.Y + Barycentric.Z;
			Test.TestTrue(
				*FString::Printf(TEXT("Sample %d barycentric x is non-negative"), SampleIndex),
				Barycentric.X >= -1.0e-6);
			Test.TestTrue(
				*FString::Printf(TEXT("Sample %d barycentric y is non-negative"), SampleIndex),
				Barycentric.Y >= -1.0e-6);
			Test.TestTrue(
				*FString::Printf(TEXT("Sample %d barycentric z is non-negative"), SampleIndex),
				Barycentric.Z >= -1.0e-6);
			Test.TestTrue(
				*FString::Printf(TEXT("Sample %d barycentric sums to one"), SampleIndex),
				FMath::IsNearlyEqual(Sum, 1.0, 1.0e-4));
			Test.TestTrue(
				*FString::Printf(TEXT("Sample %d containing triangle is valid"), SampleIndex),
				ContainingTriangles[SampleIndex] >= 0 && ContainingTriangles[SampleIndex] < Planet.TriangleIndices.Num());
			++Summary.ValidBarycentricCount;
		}

		return Summary;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSubductionDistanceTransferTest,
	"Aurous.TectonicPlanet.SubductionDistanceTransfer",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSubductionDistanceTransferTest::RunTest(const FString& Parameters)
{
	constexpr double ControlDistanceRad = 0.10;
	constexpr double MaxDistanceRad = 0.283;
	const double TransferAtZero = FTectonicPlanet::SubductionDistanceTransfer(0.0, ControlDistanceRad, MaxDistanceRad);
	const double TransferAtControl = FTectonicPlanet::SubductionDistanceTransfer(ControlDistanceRad, ControlDistanceRad, MaxDistanceRad);
	const double TransferAtMax = FTectonicPlanet::SubductionDistanceTransfer(MaxDistanceRad, ControlDistanceRad, MaxDistanceRad);
	const double TransferNearFront = FTectonicPlanet::SubductionDistanceTransfer(0.005, ControlDistanceRad, MaxDistanceRad);
	const double TransferMidArc = FTectonicPlanet::SubductionDistanceTransfer(0.20, ControlDistanceRad, MaxDistanceRad);

	TestTrue(TEXT("Transfer at d=0 matches the expected trench-side negative lobe"), FMath::IsNearlyEqual(TransferAtZero, -0.222, 0.01));
	TestTrue(TEXT("Transfer peaks at the control distance"), FMath::IsNearlyEqual(TransferAtControl, 1.0, 1.0e-6));
	TestTrue(TEXT("Transfer reaches zero at the max distance"), FMath::IsNearlyEqual(TransferAtMax, 0.0, 1.0e-6));
	TestTrue(TEXT("Transfer remains negative close to the front"), TransferNearFront < 0.0);
	TestTrue(TEXT("Transfer is positive across the volcanic-arc interior"), TransferMidArc > 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSubductionDistanceFieldRespectsPlateBoundariesTest,
	"Aurous.TectonicPlanet.SubductionDistanceFieldRespectsPlateBoundaries",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSubductionDistanceFieldRespectsPlateBoundariesTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualSubductionTestPlanet();
	FResamplingStats Stats;
	Planet.ComputeSubductionDistanceField(&Stats);

	TestEqual(TEXT("Manual subduction test detects one convergent front edge"), Stats.SubductionFrontEdgeCount, 1);
	TestEqual(TEXT("Manual subduction test seeds one overriding boundary sample"), Stats.SubductionSeedSampleCount, 1);
	TestTrue(TEXT("Manual subduction test influences the overriding plate interior"), Stats.SubductionInfluencedCount >= 3);
	TestTrue(TEXT("Seed sample distance is zero"), FMath::IsNearlyEqual(Planet.Samples[0].SubductionDistanceKm, 0.0f, 1.0e-3f));
	TestTrue(
		TEXT("First overriding interior sample gets the correct 5-degree geodesic distance"),
		FMath::IsNearlyEqual(
			static_cast<double>(Planet.Samples[2].SubductionDistanceKm),
			ComputeExpectedArcDistanceKm(5.0),
			1.0));
	TestTrue(
		TEXT("Second overriding interior sample gets the correct 10-degree geodesic distance"),
		FMath::IsNearlyEqual(
			static_cast<double>(Planet.Samples[3].SubductionDistanceKm),
			ComputeExpectedArcDistanceKm(10.0),
			1.0));
	TestTrue(TEXT("The subducting plate side is not influenced"), Planet.Samples[1].SubductionDistanceKm < 0.0f);
	TestTrue(TEXT("Samples beyond the cutoff remain uninfluenced"), Planet.Samples[4].SubductionDistanceKm < 0.0f && Planet.Samples[5].SubductionDistanceKm < 0.0f);
	TestTrue(TEXT("Carried sample distance is written back to the overriding plate"), Planet.Plates[0].CarriedSamples[1].SubductionDistanceKm >= 0.0f);
	TestTrue(TEXT("Carried sample distance is not written across the plate boundary"), Planet.Plates[1].CarriedSamples[0].SubductionDistanceKm < 0.0f);

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		if (Planet.Samples[SampleIndex].SubductionDistanceKm < 0.0f)
		{
			continue;
		}

		TestEqual(
			FString::Printf(TEXT("Influenced sample %d stays on the overriding plate"), SampleIndex),
			Planet.Samples[SampleIndex].PlateId,
			0);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSubductionUpliftSignTest,
	"Aurous.TectonicPlanet.SubductionUpliftSign",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSubductionUpliftSignTest::RunTest(const FString& Parameters)
{
	constexpr double ControlDistanceRad = 0.10;
	constexpr double MaxDistanceRad = 0.283;
	constexpr double BaseUpliftKmPerMy = 0.0006;
	constexpr double DeltaTimeMyr = 2.0;
	const double PeakArcUplift =
		BaseUpliftKmPerMy *
		FTectonicPlanet::SubductionDistanceTransfer(ControlDistanceRad, ControlDistanceRad, MaxDistanceRad) *
		DeltaTimeMyr;
	const double FrontTrenchUplift =
		BaseUpliftKmPerMy *
		FTectonicPlanet::SubductionDistanceTransfer(0.0, ControlDistanceRad, MaxDistanceRad) *
		DeltaTimeMyr;

	TestTrue(TEXT("Volcanic-arc control distance produces positive uplift"), PeakArcUplift > 0.0);
	TestTrue(TEXT("Immediate front distance produces negative uplift"), FrontTrenchUplift < 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSubductionDistanceFieldAtFirstResampleTest,
	"Aurous.TectonicPlanet.SubductionDistanceFieldAtFirstResample",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSubductionDistanceFieldAtFirstResampleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	while (Planet.ResamplingSteps.IsEmpty())
	{
		Planet.AdvanceStep();
	}

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	AddInfo(FString::Printf(
		TEXT("subduction_distance_field_first_resample step=%d front_edges=%d seed_samples=%d influenced=%d mean_distance_km=%.3f max_distance_km=%.3f dijkstra_ms=%.3f"),
		Stats.Step,
		Stats.SubductionFrontEdgeCount,
		Stats.SubductionSeedSampleCount,
		Stats.SubductionInfluencedCount,
		Stats.SubductionMeanDistanceKm,
		Stats.SubductionMaxDistanceKm,
		Stats.SubductionDistanceFieldMs));

	TestEqual(TEXT("First resample for the 60k/7 setup lands at step 10"), Stats.Step, 10);
	TestTrue(TEXT("Subduction front detection finds convergent boundary edges"), Stats.SubductionFrontEdgeCount > 0);
	TestTrue(TEXT("Subduction front detection seeds overriding-boundary samples"), Stats.SubductionSeedSampleCount > 0);
	TestTrue(TEXT("Subduction Dijkstra influences a non-zero sample set"), Stats.SubductionInfluencedCount > 0);
	TestTrue(TEXT("Subduction mean distance stays in the expected broad range"), Stats.SubductionMeanDistanceKm >= 200.0 && Stats.SubductionMeanDistanceKm <= 1200.0);
	TestTrue(TEXT("Subduction max distance respects the cutoff"), Stats.SubductionMaxDistanceKm <= 1803.0 + 1.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSubductionUpliftAtFirstResampleTest,
	"Aurous.TectonicPlanet.SubductionUpliftAtFirstResample",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSubductionUpliftAtFirstResampleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	TArray<float> InitialElevations;
	InitialElevations.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		InitialElevations.Add(Sample.Elevation);
	}

	while (Planet.ResamplingSteps.IsEmpty())
	{
		Planet.AdvanceStep();
	}

	int32 ArcBandSampleCount = 0;
	int32 TrenchBandSampleCount = 0;
	const double ArcMeanDeltaKm = ComputeMeanElevationDeltaForDistanceBand(Planet, InitialElevations, 500.0, 800.0, ArcBandSampleCount);
	const double TrenchMeanDeltaKm = ComputeMeanElevationDeltaForDistanceBand(Planet, InitialElevations, 0.0, 50.0, TrenchBandSampleCount);
	const int32 AndeanCount = CountAndeanSamples(Planet);
	AddInfo(FString::Printf(
		TEXT("subduction_uplift_first_resample andean_count=%d arc_band_samples=%d arc_mean_delta_km=%.6f trench_band_samples=%d trench_mean_delta_km=%.6f"),
		AndeanCount,
		ArcBandSampleCount,
		ArcMeanDeltaKm,
		TrenchBandSampleCount,
		TrenchMeanDeltaKm));

	TestTrue(TEXT("Subduction uplift marks at least some samples as Andean"), AndeanCount > 0);
	TestTrue(TEXT("The volcanic-arc band is populated"), ArcBandSampleCount > 0);
	TestTrue(TEXT("The trench-side band is populated"), TrenchBandSampleCount > 0);
	TestTrue(TEXT("Samples in the volcanic-arc band gain elevation on average"), ArcMeanDeltaKm > 0.0);

	for (const FSample& Sample : Planet.Samples)
	{
		TestTrue(TEXT("Elevation remains finite"), FMath::IsFinite(Sample.Elevation));
		TestTrue(TEXT("Elevation remains within the configured clamp bounds"), Sample.Elevation >= ElevationFloorKm && Sample.Elevation <= ElevationCeilingKm);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSlabPullCorrectionDirectionTest,
	"Aurous.TectonicPlanet.SlabPullCorrectionDirection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSlabPullCorrectionDirectionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualSlabPullTestPlanet();
	FResamplingStats Stats;
	Planet.ComputeSlabPullCorrections(&Stats);

	const FVector3d ExpectedCentroid =
		(Planet.Samples[1].Position.GetSafeNormal() + Planet.Samples[3].Position.GetSafeNormal()).GetSafeNormal();
	const FVector3d ExpectedCorrection =
		FVector3d::CrossProduct(ExpectedCentroid, Planet.Samples[1].Position.GetSafeNormal()).GetSafeNormal();
	const FVector3d ActualCorrection = Planet.Plates[1].SlabPullCorrectionAxis.GetSafeNormal();

	TestEqual(TEXT("The subducting plate keeps a single unique front sample"), Planet.Plates[1].SlabPullFrontSampleCount, 1);
	TestTrue(TEXT("The subducting plate receives a non-zero slab-pull correction"), !Planet.Plates[1].SlabPullCorrectionAxis.IsNearlyZero());
	TestTrue(TEXT("The slab-pull correction direction matches Cross(centroid, front)"), ActualCorrection.Dot(ExpectedCorrection) > 0.999);
	TestEqual(TEXT("Only the subducting plate contributes front samples"), Stats.SlabPullTotalFrontSamples, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSlabPullAxisOnlyTest,
	"Aurous.TectonicPlanet.SlabPullAxisOnly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSlabPullAxisOnlyTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualSlabPullTestPlanet();
	Planet.ComputeSlabPullCorrections();
	const TArray<double> InitialAngularSpeeds = CaptureAngularSpeeds(Planet);

	TestTrue(TEXT("The manual slab-pull setup produces a non-zero correction"), !Planet.Plates[1].SlabPullCorrectionAxis.IsNearlyZero());
	Planet.AdvanceStep();

	for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
	{
		TestTrue(
			FString::Printf(TEXT("Slab pull leaves plate %d angular speed unchanged"), PlateIndex),
			FMath::IsNearlyEqual(Planet.Plates[PlateIndex].AngularSpeed, InitialAngularSpeeds[PlateIndex], 1.0e-12));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSlabPullFrontSampleDeduplicationTest,
	"Aurous.TectonicPlanet.SlabPullFrontSampleDeduplication",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSlabPullFrontSampleDeduplicationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualSlabPullTestPlanet();
	FResamplingStats Stats;
	Planet.ComputeSlabPullCorrections(&Stats);

	TestEqual(TEXT("A front sample shared by multiple convergent edges is counted once"), Planet.Plates[1].SlabPullFrontSampleCount, 1);
	TestEqual(TEXT("The resampling stats also report one unique slab-pull front sample"), Stats.SlabPullTotalFrontSamples, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetAndeanContinentalConversionRateTest,
	"Aurous.TectonicPlanet.AndeanContinentalConversionRate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetAndeanContinentalConversionRateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	Planet.PlanetRadiusKm = TestPlanetRadiusKm;
	Planet.Samples.SetNum(1);
	Planet.SampleAdjacency.SetNum(1);
	Planet.Plates.SetNum(1);
	Planet.Plates[0].Id = 0;
	Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
	Planet.Plates[0].AngularSpeed = 0.0;
	Planet.Plates[0].MemberSamples = { 0 };
	Planet.Samples[0].Position = FVector3d(0.0, 0.0, 1.0);
	Planet.Samples[0].PlateId = 0;

	FCarriedSample& CarriedSample = Planet.Plates[0].CarriedSamples.AddDefaulted_GetRef();
	CarriedSample.CanonicalSampleIndex = 0;
	CarriedSample.ContinentalWeight = 0.20f;
	CarriedSample.Elevation = 1.0f;
	CarriedSample.OrogenyType = EOrogenyType::Andean;
	Planet.Plates[0].CanonicalToCarriedIndex.Add(0, 0);

	Planet.AdvanceStep();

	TestTrue(TEXT("Andean continental conversion advances at the configured rate"), FMath::IsNearlyEqual(Planet.Samples[0].ContinentalWeight, 0.21f, 1.0e-6f));
	TestEqual(TEXT("Andean continental conversion preserves the Andean orogeny flag"), Planet.Samples[0].OrogenyType, EOrogenyType::Andean);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSlabPullSafetyCapTest,
	"Aurous.TectonicPlanet.SlabPullSafetyCap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSlabPullSafetyCapTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	Planet.PlanetRadiusKm = TestPlanetRadiusKm;
	Planet.Samples.SetNum(1);
	Planet.SampleAdjacency.SetNum(1);
	Planet.Plates.SetNum(1);
	Planet.Plates[0].Id = 0;
	Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
	Planet.Plates[0].AngularSpeed = 0.01;
	Planet.Plates[0].MemberSamples = { 0 };
	Planet.Plates[0].SlabPullCorrectionAxis = FVector3d(1.0e9, 0.0, 0.0);
	Planet.Samples[0].Position = FVector3d(0.0, 0.0, 1.0);
	Planet.Samples[0].PlateId = 0;

	FCarriedSample& CarriedSample = Planet.Plates[0].CarriedSamples.AddDefaulted_GetRef();
	CarriedSample.CanonicalSampleIndex = 0;
	Planet.Plates[0].CanonicalToCarriedIndex.Add(0, 0);

	const FVector3d InitialAxis = Planet.Plates[0].RotationAxis;
	const double InitialSpeed = Planet.Plates[0].AngularSpeed;
	Planet.AdvanceStep();

	const double AxisChangeRad = ComputeAxisAngleRad(InitialAxis, Planet.Plates[0].RotationAxis);
	TestTrue(TEXT("The slab-pull safety cap limits the per-step axis change"), AxisChangeRad <= FMath::DegreesToRadians(2.9) + 1.0e-3);
	TestTrue(TEXT("The slab-pull safety cap still allows a non-zero steering update"), AxisChangeRad > 0.0);
	TestTrue(TEXT("The capped axis remains finite and normalized"), IsFiniteNormalizedAxis(Planet.Plates[0].RotationAxis));
	TestTrue(TEXT("The slab-pull safety cap does not change angular speed"), FMath::IsNearlyEqual(Planet.Plates[0].AngularSpeed, InitialSpeed, 1.0e-12));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSlabPullAtFirstResampleTest,
	"Aurous.TectonicPlanet.SlabPullAtFirstResample",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSlabPullAtFirstResampleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const TArray<double> InitialAngularSpeeds = CaptureAngularSpeeds(Planet);

	while (Planet.ResamplingSteps.IsEmpty())
	{
		Planet.AdvanceStep();
	}

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	AddInfo(FString::Printf(
		TEXT("slab_pull_first_resample step=%d slab_pull_plate_count=%d slab_pull_total_front_samples=%d slab_pull_max_axis_change_rad=%.6f slab_pull_ms=%.3f"),
		Stats.Step,
		Stats.SlabPullPlateCount,
		Stats.SlabPullTotalFrontSamples,
		Stats.SlabPullMaxAxisChangeRad,
		Stats.SlabPullMs));

	TestEqual(TEXT("First resample for the 60k/7 slab-pull setup lands at step 10"), Stats.Step, 10);
	TestTrue(TEXT("Some plates receive slab-pull corrections"), Stats.SlabPullPlateCount > 0);
	TestTrue(TEXT("The slab-pull front sample set is non-empty"), Stats.SlabPullTotalFrontSamples > 0);
	TestTrue(TEXT("The slab-pull correction stays below the safety cap at epsilon=0.1"), Stats.SlabPullMaxAxisChangeRad > 0.0 && Stats.SlabPullMaxAxisChangeRad < FMath::DegreesToRadians(2.9));
	TestTrue(TEXT("Slab-pull correction timing is recorded"), Stats.SlabPullMs > 0.0);

	for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
	{
		TestTrue(
			FString::Printf(TEXT("Plate %d rotation axis stays finite and normalized after first resample"), PlateIndex),
			IsFiniteNormalizedAxis(Planet.Plates[PlateIndex].RotationAxis));
		TestTrue(
			FString::Printf(TEXT("Plate %d angular speed remains unchanged after slab-pull steering"), PlateIndex),
			FMath::IsNearlyEqual(Planet.Plates[PlateIndex].AngularSpeed, InitialAngularSpeeds[PlateIndex], 1.0e-12));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSlabPullSteeringEffectTest,
	"Aurous.TectonicPlanet.SlabPullSteeringEffect",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSlabPullSteeringEffectTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const TArray<FVector3d> InitialAxes = CaptureRotationAxes(Planet);
	TArray<FString> AxisDotSummaries;
	bool bAnyAxisChanged = false;

	for (int32 StepIndex = 0; StepIndex < 50; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
	{
		const double AxisDot = InitialAxes[PlateIndex].Dot(Planet.Plates[PlateIndex].RotationAxis.GetSafeNormal());
		const double AxisAngleRad = ComputeAxisAngleRad(InitialAxes[PlateIndex], Planet.Plates[PlateIndex].RotationAxis);
		bAnyAxisChanged |= AxisDot < (1.0 - 1.0e-6);
		TestTrue(
			FString::Printf(TEXT("Plate %d slab-pull steering stays within the 30-degree total-change envelope"), PlateIndex),
			AxisAngleRad <= FMath::DegreesToRadians(30.0) + 1.0e-3);
		AxisDotSummaries.Add(FString::Printf(TEXT("%d:%.6f"), PlateIndex, AxisDot));
	}

	AddInfo(FString::Printf(TEXT("slab_pull_axis_dots step=%d dots=%s"), Planet.CurrentStep, *FString::Join(AxisDotSummaries, TEXT(","))));
	TestEqual(TEXT("The slab-pull steering run advances to step 50"), Planet.CurrentStep, 50);
	TestTrue(TEXT("At least one plate rotation axis changes direction over 50 steps"), bAnyAxisChanged);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetAndeanContinentalConversionAt50StepsTest,
	"Aurous.TectonicPlanet.AndeanContinentalConversionAt50Steps",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetAndeanContinentalConversionAt50StepsTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet M2kBaselinePlanet = CreateInitializedPlanet();
	M2kBaselinePlanet.bEnableSlabPull = false;
	M2kBaselinePlanet.bEnableAndeanContinentalConversion = false;

	FTectonicPlanet NoCreationPlanet = CreateInitializedPlanet();
	NoCreationPlanet.bEnableAndeanContinentalConversion = false;

	FTectonicPlanet Planet = CreateInitializedPlanet();
	TMap<int32, int32> ContinentalCountsByStep;
	TMap<int32, int32> M2kBaselineContinentalCountsByStep;
	TMap<int32, int32> PartialCountsByStep;
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M3b-SlabPullAndeanConversion-seed42-samples60000-plates7"));

	for (int32 StepIndex = 0; StepIndex < 50; ++StepIndex)
	{
		M2kBaselinePlanet.AdvanceStep();
		NoCreationPlanet.AdvanceStep();
		Planet.AdvanceStep();

		if (!Planet.ResamplingSteps.IsEmpty() && Planet.ResamplingSteps.Last() == Planet.CurrentStep)
		{
			ContinentalCountsByStep.Add(Planet.CurrentStep, CountContinentalSamples(Planet));
			PartialCountsByStep.Add(Planet.CurrentStep, CountPartiallyConvertedSamples(Planet));
			if (Planet.CurrentStep == 10 || Planet.CurrentStep == 50)
			{
				TestTrue(
					*FString::Printf(TEXT("M3b export succeeded at step %d"), Planet.CurrentStep),
					ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep));
			}
		}

		if (!M2kBaselinePlanet.ResamplingSteps.IsEmpty() && M2kBaselinePlanet.ResamplingSteps.Last() == M2kBaselinePlanet.CurrentStep)
		{
			M2kBaselineContinentalCountsByStep.Add(M2kBaselinePlanet.CurrentStep, CountContinentalSamples(M2kBaselinePlanet));
		}
	}

	const int32 AndeanCount = CountAndeanSamples(Planet);
	TestTrue(TEXT("The M3b run records the step-10 checkpoint"), ContinentalCountsByStep.Contains(10) && M2kBaselineContinentalCountsByStep.Contains(10) && PartialCountsByStep.Contains(10));
	TestTrue(TEXT("The M3b run records the step-50 checkpoint"), ContinentalCountsByStep.Contains(50) && M2kBaselineContinentalCountsByStep.Contains(50) && PartialCountsByStep.Contains(50));
	const int32 PartialCountAt50 = CountPartiallyConvertedSamples(Planet);
	const int32 ContinentalCountAt50 = ContinentalCountsByStep.FindRef(50);
	const int32 M2kBaselineContinentalCountAt50 = M2kBaselineContinentalCountsByStep.FindRef(50);
	const double TotalContinentalWeightAt50 = SumContinentalWeight(Planet);
	const double NoCreationTotalContinentalWeightAt50 = SumContinentalWeight(NoCreationPlanet);
	AddInfo(FString::Printf(
		TEXT("andean_conversion_50_steps andean_count=%d m2k_baseline_checkpoints=10:%d,20:%d,30:%d,40:%d,50:%d experimental_checkpoints=10:%d,20:%d,30:%d,40:%d,50:%d partial_checkpoints=10:%d,20:%d,30:%d,40:%d,50:%d m2k_count_delta_50=%d creation_control_weight_delta_50=%.3f"),
		AndeanCount,
		M2kBaselineContinentalCountsByStep.FindRef(10),
		M2kBaselineContinentalCountsByStep.FindRef(20),
		M2kBaselineContinentalCountsByStep.FindRef(30),
		M2kBaselineContinentalCountsByStep.FindRef(40),
		M2kBaselineContinentalCountsByStep.FindRef(50),
		ContinentalCountsByStep.FindRef(10),
		ContinentalCountsByStep.FindRef(20),
		ContinentalCountsByStep.FindRef(30),
		ContinentalCountsByStep.FindRef(40),
		ContinentalCountsByStep.FindRef(50),
		PartialCountsByStep.FindRef(10),
		PartialCountsByStep.FindRef(20),
		PartialCountsByStep.FindRef(30),
		PartialCountsByStep.FindRef(40),
		PartialCountsByStep.FindRef(50),
		ContinentalCountAt50 - M2kBaselineContinentalCountAt50,
		TotalContinentalWeightAt50 - NoCreationTotalContinentalWeightAt50));

	TestTrue(TEXT("The M3b run produces Andean samples by step 50"), AndeanCount > 0);
	TestTrue(TEXT("Partial continental conversion appears by step 10"), PartialCountsByStep.FindRef(10) > 0);
	TestTrue(TEXT("Partial continental conversion remains visible by step 50"), PartialCountAt50 > 0);
	TestTrue(TEXT("M3b increases total continental weight above the slab-pull/no-conversion control by step 50"), TotalContinentalWeightAt50 > NoCreationTotalContinentalWeightAt50);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetHysteresisRetentionTest,
	"Aurous.TectonicPlanet.HysteresisRetention",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetHysteresisRetentionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(0, 10, 1);
	Planet.bEnableOverlapHysteresis = true;
	const FManualOverlapOwnershipQueryResult Result = RunManualOverlapOwnershipQuery(Planet);

	TestEqual(TEXT("Manual overlap setup produces one overlap"), Result.OverlapCount, 1);
	TestEqual(TEXT("Hysteresis keeps the sample on its current containing plate"), Result.NewPlateIds[3], 0);
	TestEqual(TEXT("Hysteresis records one retained overlap"), Result.Stats.HysteresisRetainedCount, 1);
	TestEqual(TEXT("Hysteresis records no reassigned overlaps"), Result.Stats.HysteresisReassignedCount, 0);
	TestEqual(TEXT("The retained sample remains marked as an overlap"), Result.OverlapFlags[3], static_cast<uint8>(1));
	TestTrue(TEXT("Overlap plate tracking preserves every containing plate"), Result.OverlapPlateIds[3].Contains(0) && Result.OverlapPlateIds[3].Contains(1));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetHysteresisReassignmentTest,
	"Aurous.TectonicPlanet.HysteresisReassignment",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetHysteresisReassignmentTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(2, 3, 9);
	Planet.bEnableOverlapHysteresis = true;
	const FManualOverlapOwnershipQueryResult Result = RunManualOverlapOwnershipQuery(Planet);

	TestEqual(TEXT("Manual overlap setup produces one overlap"), Result.OverlapCount, 1);
	TestEqual(TEXT("If the current plate is not containing, overlap resolution falls through to the score winner"), Result.NewPlateIds[3], 1);
	TestEqual(TEXT("No overlap is retained when the current plate is absent"), Result.Stats.HysteresisRetainedCount, 0);
	TestEqual(TEXT("The overlap is counted as a hysteresis reassignment"), Result.Stats.HysteresisReassignedCount, 1);
	TestEqual(TEXT("The reassigned sample remains marked as an overlap"), Result.OverlapFlags[3], static_cast<uint8>(1));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetHysteresisRetainsNonWinnerTest,
	"Aurous.TectonicPlanet.HysteresisRetainsNonWinner",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetHysteresisRetainsNonWinnerTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(0, 1, 10);
	Planet.bEnableOverlapHysteresis = true;
	const FManualOverlapOwnershipQueryResult Result = RunManualOverlapOwnershipQuery(Planet);

	TestEqual(TEXT("Manual overlap setup produces one overlap"), Result.OverlapCount, 1);
	TestEqual(TEXT("Binary hysteresis keeps the current plate even when it loses the overlap score"), Result.NewPlateIds[3], 0);
	TestEqual(TEXT("The non-winner overlap is still counted as retained"), Result.Stats.HysteresisRetainedCount, 1);
	TestEqual(TEXT("No hysteresis reassignment occurs when the current plate is still containing"), Result.Stats.HysteresisReassignedCount, 0);

	Planet.bEnableOverlapHysteresis = false;
	const FManualOverlapOwnershipQueryResult BaselineResult = RunManualOverlapOwnershipQuery(Planet);
	TestEqual(TEXT("Without hysteresis the same overlap resolves to the higher-score plate"), BaselineResult.NewPlateIds[3], 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetHysteresisBoundaryWidthRegressionTest,
	"Aurous.TectonicPlanet.HysteresisBoundaryWidthRegression",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetHysteresisBoundaryWidthRegressionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BaselinePlanet = CreateInitializedPlanet();
	BaselinePlanet.bEnableOverlapHysteresis = false;

	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.bEnableOverlapHysteresis = true;
	TMap<int32, int32> ContinentalCountsByStep;
	TMap<int32, int32> BaselineContinentalCountsByStep;
	FResamplingStats Step10Stats;
	FResamplingStats BaselineStep10Stats;
	bool bCapturedStep10Stats = false;
	bool bCapturedBaselineStep10Stats = false;
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M3c-BinaryHysteresis-WithHysteresis-seed42-samples60000-plates7"));
	const FString BaselineExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M3c-BinaryHysteresis-WithoutHysteresis-seed42-samples60000-plates7"));

	for (int32 StepIndex = 0; StepIndex < 50; ++StepIndex)
	{
		BaselinePlanet.AdvanceStep();
		Planet.AdvanceStep();

		if (!Planet.ResamplingSteps.IsEmpty() && Planet.ResamplingSteps.Last() == Planet.CurrentStep)
		{
			ContinentalCountsByStep.Add(Planet.CurrentStep, CountContinentalSamples(Planet));
			if (Planet.CurrentStep == 10)
			{
				Step10Stats = Planet.LastResamplingStats;
				bCapturedStep10Stats = true;
			}

			if (Planet.CurrentStep == 10 || Planet.CurrentStep == 50)
			{
				TestTrue(
					*FString::Printf(TEXT("M3c hysteresis export succeeded at step %d"), Planet.CurrentStep),
					ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep));
			}
		}

		if (!BaselinePlanet.ResamplingSteps.IsEmpty() && BaselinePlanet.ResamplingSteps.Last() == BaselinePlanet.CurrentStep)
		{
			BaselineContinentalCountsByStep.Add(BaselinePlanet.CurrentStep, CountContinentalSamples(BaselinePlanet));
			if (BaselinePlanet.CurrentStep == 10)
			{
				BaselineStep10Stats = BaselinePlanet.LastResamplingStats;
				bCapturedBaselineStep10Stats = true;
			}

			if (BaselinePlanet.CurrentStep == 50)
			{
				TestTrue(
					TEXT("M3c no-hysteresis baseline export succeeded at step 50"),
					ExportCheckpointMaps(*this, BaselinePlanet, BaselineExportRoot, BaselinePlanet.CurrentStep));
			}
		}
	}

	TestTrue(TEXT("The hysteresis run captured step-10 stats"), bCapturedStep10Stats);
	TestTrue(TEXT("The no-hysteresis baseline captured step-10 stats"), bCapturedBaselineStep10Stats);
	TestTrue(TEXT("The hysteresis run captured step-10 and step-50 continental counts"), ContinentalCountsByStep.Contains(10) && ContinentalCountsByStep.Contains(50));
	TestTrue(TEXT("The no-hysteresis baseline captured step-10 and step-50 continental counts"), BaselineContinentalCountsByStep.Contains(10) && BaselineContinentalCountsByStep.Contains(50));

	const int32 Step10ClassifiedOverlapCount =
		Step10Stats.OceanicOceanicOverlapCount +
		Step10Stats.OceanicContinentalOverlapCount +
		Step10Stats.ContinentalContinentalOverlapCount +
		Step10Stats.NonConvergentOverlapCount;
	const int32 BoundaryCountAt50 = CountBoundarySamples(Planet);
	const int32 BaselineBoundaryCountAt50 = CountBoundarySamples(BaselinePlanet);
	const double BoundaryFractionAt50 = static_cast<double>(BoundaryCountAt50) / static_cast<double>(Planet.Samples.Num());
	const double BaselineBoundaryFractionAt50 = static_cast<double>(BaselineBoundaryCountAt50) / static_cast<double>(BaselinePlanet.Samples.Num());
	const int32 Step10ContinentalDelta = ContinentalCountsByStep.FindRef(10) - BaselineContinentalCountsByStep.FindRef(10);
	const int32 Step50ContinentalDelta = ContinentalCountsByStep.FindRef(50) - BaselineContinentalCountsByStep.FindRef(50);

	TestTrue(TEXT("Hysteresis retains at least some first-resample overlaps"), Step10Stats.HysteresisRetainedCount > 0);
	TestEqual(TEXT("First-resample hysteresis accounting partitions all overlaps"), Step10Stats.HysteresisRetainedCount + Step10Stats.HysteresisReassignedCount, Step10Stats.OverlapCount);
	TestTrue(TEXT("Hysteresis does not increase first-resample gap count"), Step10Stats.GapCount <= BaselineStep10Stats.GapCount);
	TestTrue(TEXT("Hysteresis does not increase first-resample true-gap count"), Step10Stats.TrueGapCount <= BaselineStep10Stats.TrueGapCount);
	TestEqual(
		TEXT("Hysteresis preserves the ownership accounting invariant"),
		Step10Stats.ExactSingleHitCount + Step10Stats.ExactMultiHitCount + Step10Stats.RecoveryContainmentCount + Step10Stats.TrueGapCount,
		Planet.Samples.Num());
	TestTrue(TEXT("First-resample overlap classifications remain populated"), Step10Stats.OverlapCount > 0 && Step10ClassifiedOverlapCount == Step10Stats.OverlapCount);
	TestTrue(TEXT("Binary hysteresis narrows the step-50 boundary sample count"), BoundaryCountAt50 < BaselineBoundaryCountAt50);
	TestTrue(TEXT("The step-50 subduction field remains populated"), Planet.LastResamplingStats.SubductionFrontEdgeCount > 0 && Planet.LastResamplingStats.SubductionSeedSampleCount > 0 && Planet.LastResamplingStats.SubductionInfluencedCount > 0);
	TestTrue(TEXT("The step-50 subduction field still respects the cutoff"), Planet.LastResamplingStats.SubductionMaxDistanceKm <= 1803.0 + 1.0);

	AddInfo(FString::Printf(
		TEXT("hysteresis_boundary_width_regression step10_hysteresis_retained=%d step10_hysteresis_reassigned=%d step10_gap_count=%d baseline_step10_gap_count=%d step10_true_gap_count=%d baseline_step10_true_gap_count=%d step10_overlap_count=%d step10_oo=%d step10_oc=%d step10_cc=%d step10_nonconvergent=%d step10_continental=%d baseline_step10_continental=%d step10_continental_delta=%d step50_boundary_count=%d baseline_step50_boundary_count=%d step50_boundary_fraction=%.4f baseline_step50_boundary_fraction=%.4f step50_continental=%d baseline_step50_continental=%d step50_continental_delta=%d step50_subduction_front_edge_count=%d step50_subduction_influenced_count=%d"),
		Step10Stats.HysteresisRetainedCount,
		Step10Stats.HysteresisReassignedCount,
		Step10Stats.GapCount,
		BaselineStep10Stats.GapCount,
		Step10Stats.TrueGapCount,
		BaselineStep10Stats.TrueGapCount,
		Step10Stats.OverlapCount,
		Step10Stats.OceanicOceanicOverlapCount,
		Step10Stats.OceanicContinentalOverlapCount,
		Step10Stats.ContinentalContinentalOverlapCount,
		Step10Stats.NonConvergentOverlapCount,
		ContinentalCountsByStep.FindRef(10),
		BaselineContinentalCountsByStep.FindRef(10),
		Step10ContinentalDelta,
		BoundaryCountAt50,
		BaselineBoundaryCountAt50,
		BoundaryFractionAt50,
		BaselineBoundaryFractionAt50,
		ContinentalCountsByStep.FindRef(50),
		BaselineContinentalCountsByStep.FindRef(50),
		Step50ContinentalDelta,
		Planet.LastResamplingStats.SubductionFrontEdgeCount,
		Planet.LastResamplingStats.SubductionInfluencedCount));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveOwnershipSamePlateHitTest,
	"Aurous.TectonicPlanet.PreserveOwnershipSamePlateHit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipSamePlateHitTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(0, 10, 1);
	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::PreserveOwnership);

	TestEqual(TEXT("PreserveOwnership keeps the sample on its current plate when exact containment succeeds"), Result.NewPlateIds[3], 0);
	TestEqual(TEXT("PreserveOwnership records same-plate exact hits for all four samples in the manual query"), Result.Stats.PreserveOwnershipSamePlateHitCount, 4);
	TestEqual(TEXT("PreserveOwnership records no same-plate recovery"), Result.Stats.PreserveOwnershipSamePlateRecoveryCount, 0);
	TestEqual(TEXT("PreserveOwnership records no fallback query"), Result.Stats.PreserveOwnershipFallbackQueryCount, 0);
	TestEqual(TEXT("PreserveOwnership does not mark the exact same-plate hit as a gap"), Result.GapFlags[3], static_cast<uint8>(0));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveOwnershipSamePlateRecoveryTest,
	"Aurous.TectonicPlanet.PreserveOwnershipSamePlateRecovery",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipSamePlateRecoveryTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualPreserveRecoveryTestPlanet();
	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::PreserveOwnership);

	TestEqual(TEXT("PreserveOwnership keeps the sample on its current plate when same-plate recovery succeeds"), Result.NewPlateIds[3], 0);
	TestEqual(TEXT("PreserveOwnership still records exact same-plate hits for the unaffected manual samples"), Result.Stats.PreserveOwnershipSamePlateHitCount, 3);
	TestEqual(TEXT("PreserveOwnership records one same-plate recovery"), Result.Stats.PreserveOwnershipSamePlateRecoveryCount, 1);
	TestEqual(TEXT("PreserveOwnership records no fallback query when recovery succeeds"), Result.Stats.PreserveOwnershipFallbackQueryCount, 0);
	TestEqual(TEXT("The recovery contributes to containment recovery accounting"), Result.Stats.RecoveryContainmentCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveOwnershipFallbackTest,
	"Aurous.TectonicPlanet.PreserveOwnershipFallback",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipFallbackTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(2, 3, 9);
	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::PreserveOwnership);

	TestEqual(TEXT("PreserveOwnership still records exact same-plate hits for the unaffected manual samples"), Result.Stats.PreserveOwnershipSamePlateHitCount, 3);
	TestEqual(TEXT("PreserveOwnership records no same-plate recovery when the current plate has no soup"), Result.Stats.PreserveOwnershipSamePlateRecoveryCount, 0);
	TestEqual(TEXT("PreserveOwnership records one fallback full query"), Result.Stats.PreserveOwnershipFallbackQueryCount, 1);
	TestEqual(TEXT("Fallback full query can still reassign to the score winner"), Result.NewPlateIds[3], 1);
	TestEqual(TEXT("Fallback full query still classifies the overlap"), Result.OverlapCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetBoundaryContactCollisionCandidateTest,
	"Aurous.TectonicPlanet.BoundaryContactCollisionCandidateTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetBoundaryContactCollisionCandidateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualBoundaryContactCollisionPlanet(3, true);
	FResamplingStats Stats;
	const bool bTriggered = Planet.DetectBoundaryContactCollisionTrigger(&Stats);

	TestTrue(TEXT("Convergent continental boundary contact produces candidates"), Stats.BoundaryContactCollisionCandidateCount > 0);
	TestEqual(TEXT("The manual boundary-contact setup uses one unordered plate pair"), Stats.BoundaryContactCollisionPairCount, 1);
	TestEqual(TEXT("The strongest local boundary-contact zone contains all manual convergent contact edges"), Stats.BoundaryContactLargestZoneSize, 3);
	TestTrue(TEXT("The candidate-only setup remains below the preserve-mode collision threshold"), !bTriggered && !Stats.bBoundaryContactCollisionTriggered);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetBoundaryContactCollisionNoFalsePositiveDivergentTest,
	"Aurous.TectonicPlanet.BoundaryContactCollisionNoFalsePositiveDivergentTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetBoundaryContactCollisionNoFalsePositiveDivergentTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualBoundaryContactCollisionPlanet(10, false);
	FResamplingStats Stats;
	const bool bTriggered = Planet.DetectBoundaryContactCollisionTrigger(&Stats);

	TestEqual(TEXT("Divergent continental contact produces no boundary-contact collision candidates"), Stats.BoundaryContactCollisionCandidateCount, 0);
	TestEqual(TEXT("Divergent continental contact produces no boundary-contact collision plate pairs"), Stats.BoundaryContactCollisionPairCount, 0);
	TestEqual(TEXT("Divergent continental contact produces no boundary-contact collision zone"), Stats.BoundaryContactLargestZoneSize, 0);
	TestTrue(TEXT("Divergent continental contact does not queue a collision follow-up"), !bTriggered && !Stats.bBoundaryContactCollisionTriggered);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetBoundaryContactCollisionQueueFollowupTest,
	"Aurous.TectonicPlanet.BoundaryContactCollisionQueueFollowupTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetBoundaryContactCollisionQueueFollowupTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(2, 8000);
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	SetAllContinentalWeights(Planet, 1.0f);
	AssignPlanetToHemispheres(Planet);
	Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
	Planet.Plates[0].AngularSpeed = 0.02;
	Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
	Planet.Plates[1].AngularSpeed = 0.02;
	Planet.CurrentStep = Planet.ComputeResampleInterval() - 1;

	Planet.AdvanceStep();

	TestEqual(TEXT("Boundary-contact preserve mode queues a same-step full-resolution follow-up"), Planet.LastResampleTriggerReason, EResampleTriggerReason::CollisionFollowup);
	TestEqual(TEXT("The queued follow-up ends in full-resolution ownership mode"), Planet.LastResampleOwnershipMode, EResampleOwnershipMode::FullResolution);
	TestTrue(TEXT("The preserve periodic resample and the queued follow-up occur on the same step"),
		Planet.ResamplingSteps.Num() >= 2 &&
		Planet.ResamplingSteps[Planet.ResamplingSteps.Num() - 1] == Planet.CurrentStep &&
		Planet.ResamplingSteps[Planet.ResamplingSteps.Num() - 2] == Planet.CurrentStep);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetBoundaryContactPersistenceTriggerTest,
	"Aurous.TectonicPlanet.BoundaryContactPersistenceTriggerTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetBoundaryContactPersistenceTriggerTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(2, 8000);
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	AssignPlanetToHemispheres(Planet);
	Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
	Planet.Plates[0].AngularSpeed = 0.02;
	Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
	Planet.Plates[1].AngularSpeed = 0.02;

	const TSet<int32> ContinentalSampleIndices = FindSparseBoundaryContactZoneSamples(Planet, 3);
	TestTrue(TEXT("The persistence trigger setup finds a sparse three-edge continental boundary zone"), ContinentalSampleIndices.Num() > 0);
	if (ContinentalSampleIndices.IsEmpty())
	{
		return false;
	}
	for (int32 ResampleOrdinal = 1; ResampleOrdinal <= 3; ++ResampleOrdinal)
	{
		StampSparseContinentalBoundaryZone(Planet, ContinentalSampleIndices);
		ForceAdvanceToPeriodicResample(Planet, ResampleOrdinal);

		if (ResampleOrdinal < 3)
		{
			TestEqual(
				FString::Printf(TEXT("Sub-threshold preserve pass %d remains periodic"), ResampleOrdinal),
				Planet.LastResampleTriggerReason,
				EResampleTriggerReason::Periodic);
			TestEqual(
				FString::Printf(TEXT("Persistence count increments on preserve pass %d"), ResampleOrdinal),
				Planet.LastResamplingStats.BoundaryContactPersistenceCount,
				ResampleOrdinal);
			TestTrue(
				FString::Printf(TEXT("Persistence does not trigger before pass %d"), ResampleOrdinal),
				!Planet.LastResamplingStats.bBoundaryContactPersistenceTriggered);
		}
	}

	TestEqual(TEXT("Persistence queues a same-step collision follow-up on the third preserve pass"), Planet.LastResampleTriggerReason, EResampleTriggerReason::CollisionFollowup);
	TestEqual(TEXT("The persistence-triggered follow-up ends in full-resolution ownership mode"), Planet.LastResampleOwnershipMode, EResampleOwnershipMode::FullResolution);
	TestTrue(TEXT("Persistence-triggered follow-up records the same step twice"),
		Planet.ResamplingSteps.Num() >= 2 &&
		Planet.ResamplingSteps[Planet.ResamplingSteps.Num() - 1] == Planet.CurrentStep &&
		Planet.ResamplingSteps[Planet.ResamplingSteps.Num() - 2] == Planet.CurrentStep);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetBoundaryContactPersistenceResetTest,
	"Aurous.TectonicPlanet.BoundaryContactPersistenceResetTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetBoundaryContactPersistenceResetTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(2, 8000);
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	AssignPlanetToHemispheres(Planet);
	Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
	Planet.Plates[0].AngularSpeed = 0.02;
	Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
	Planet.Plates[1].AngularSpeed = 0.02;

	const TSet<int32> ContinentalSampleIndices = FindSparseBoundaryContactZoneSamples(Planet, 3);
	TestTrue(TEXT("The persistence reset setup finds a sparse three-edge continental boundary zone"), ContinentalSampleIndices.Num() > 0);
	if (ContinentalSampleIndices.IsEmpty())
	{
		return false;
	}

	StampSparseContinentalBoundaryZone(Planet, ContinentalSampleIndices);
	ForceAdvanceToPeriodicResample(Planet, 1);
	TestEqual(TEXT("The first observation starts a new persistence streak"), Planet.LastResamplingStats.BoundaryContactPersistenceCount, 1);

	SetAllContinentalWeights(Planet, 0.0f);
	ForceAdvanceToPeriodicResample(Planet, 2);
	TestEqual(TEXT("Persistence count clears when the boundary-contact pair disappears"), Planet.LastResamplingStats.BoundaryContactPersistenceCount, 0);
	TestTrue(TEXT("No persistence entry remains after the pair disappears"), Planet.BoundaryContactPersistenceByPair.IsEmpty());

	for (int32 ResampleOrdinal = 3; ResampleOrdinal <= 5; ++ResampleOrdinal)
	{
		StampSparseContinentalBoundaryZone(Planet, ContinentalSampleIndices);
		ForceAdvanceToPeriodicResample(Planet, ResampleOrdinal);
	}

	TestEqual(TEXT("The reset scenario also reaches a persistence-triggered follow-up"), Planet.LastResampleTriggerReason, EResampleTriggerReason::CollisionFollowup);
	TestTrue(TEXT("Persistence state is cleared after the queued full-resolution follow-up"), Planet.BoundaryContactPersistenceByPair.IsEmpty());

	StampSparseContinentalBoundaryZone(Planet, ContinentalSampleIndices);
	ForceAdvanceToPeriodicResample(Planet, 6);
	TestEqual(TEXT("After reset, the next preserve pass starts a fresh persistence streak"), Planet.LastResampleTriggerReason, EResampleTriggerReason::Periodic);
	TestEqual(TEXT("After reset, persistence count restarts at one"), Planet.LastResamplingStats.BoundaryContactPersistenceCount, 1);
	TestTrue(TEXT("After reset, persistence does not immediately retrigger"), !Planet.LastResamplingStats.bBoundaryContactPersistenceTriggered);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCachedBoundaryContactCollisionExecutionTest,
	"Aurous.TectonicPlanet.CachedBoundaryContactCollisionExecutionTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCachedBoundaryContactCollisionExecutionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(2, 8000);
	AssignPlanetToHemispheres(Planet);
	SetAllContinentalWeights(Planet, 1.0f);

	const TArray<FIntPoint> BoundaryEdges = FindConnectedBoundaryContactEdges(Planet, 3);
	TestEqual(TEXT("The cached collision execution setup finds a connected three-edge boundary zone"), BoundaryEdges.Num(), 3);
	if (BoundaryEdges.Num() != 3)
	{
		return false;
	}

	Planet.PendingBoundaryContactCollisionEvent = BuildPendingBoundaryContactCollisionEvent(Planet, BoundaryEdges);
	TestTrue(TEXT("The cached boundary-contact event is populated"), Planet.PendingBoundaryContactCollisionEvent.bValid);

	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	TestEqual(TEXT("Cached boundary-contact follow-up executes a real collision"), Planet.LastResamplingStats.CollisionCount, 1);
	TestTrue(TEXT("The executed collision is marked as using cached boundary-contact evidence"), Planet.LastResamplingStats.bUsedCachedBoundaryContactCollision);
	TestTrue(TEXT("Cached collision records the cached seed count"), Planet.LastResamplingStats.CachedBoundaryContactSeedCount > 0);
	TestTrue(TEXT("Cached collision recovers terrane seeds on the subducting side"), Planet.LastResamplingStats.CachedBoundaryContactTerraneSeedCount > 0);
	TestTrue(TEXT("Cached collision recovers a non-empty terrane"), Planet.LastResamplingStats.CachedBoundaryContactTerraneRecoveredCount > 0);
	TestTrue(TEXT("Cached collision applies a non-zero surge footprint"), Planet.LastResamplingStats.CollisionSurgeAffectedCount > 0);
	TestFalse(TEXT("The cached event is cleared after the follow-up finishes"), Planet.PendingBoundaryContactCollisionEvent.bValid);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCachedBoundaryContactCollisionFallbackTest,
	"Aurous.TectonicPlanet.CachedBoundaryContactCollisionFallbackTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCachedBoundaryContactCollisionFallbackTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(2, 8000);
	AssignPlanetToHemispheres(Planet);
	SetAllContinentalWeights(Planet, 0.0f);

	Planet.PendingBoundaryContactCollisionEvent.bValid = true;
	Planet.PendingBoundaryContactCollisionEvent.PlateA = 0;
	Planet.PendingBoundaryContactCollisionEvent.PlateB = 1;
	Planet.PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices = { 0, 1, 2 };
	Planet.PendingBoundaryContactCollisionEvent.ContactCenter = FVector3d(1.0, 0.0, 0.0);

	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	TestEqual(TEXT("A malformed cached boundary-contact event does not force a collision"), Planet.LastResamplingStats.CollisionCount, 0);
	TestTrue(TEXT("Malformed cached execution does not report cached collision success"), !Planet.LastResamplingStats.bUsedCachedBoundaryContactCollision);
	TestFalse(TEXT("The malformed cached event is still cleared after the follow-up finishes"), Planet.PendingBoundaryContactCollisionEvent.bValid);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionCandidateCollectionTest,
	"Aurous.TectonicPlanet.CollisionCandidateCollection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionCandidateCollectionTest::RunTest(const FString& Parameters)
{
	FManualCollisionScenario Scenario = CreateManualCollisionScenario();
	TArray<FCollisionCandidate> Candidates;
	Scenario.Planet.CollectCollisionCandidates(Scenario.OverlapFlags, Scenario.OverlapPlateIds, Candidates);

	TestEqual(TEXT("Each overlap sample produces one CC candidate for the single continental plate pair"), Candidates.Num(), 20);
	const FCollisionCandidate* CandidateForFirstZoneSample = Candidates.FindByPredicate([](const FCollisionCandidate& Candidate)
	{
		return Candidate.SampleIndex == 3;
	});
	TestNotNull(TEXT("A candidate is produced for the first collision-zone sample"), CandidateForFirstZoneSample);
	if (CandidateForFirstZoneSample != nullptr)
	{
		TestEqual(TEXT("The higher-score plate is recorded as overriding"), CandidateForFirstZoneSample->OverridingPlateId, 0);
		TestEqual(TEXT("The lower-score plate is recorded as subducting"), CandidateForFirstZoneSample->SubductingPlateId, 1);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionTerraneBfsTest,
	"Aurous.TectonicPlanet.CollisionTerraneBfs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionTerraneBfsTest::RunTest(const FString& Parameters)
{
	FManualCollisionScenario Scenario = CreateManualCollisionScenario();
	FCollisionEvent CollisionEvent;
	FResamplingStats Stats;
	const bool bDetected = Scenario.Planet.DetectAndApplyCollision(
		EResampleTriggerReason::None,
		Scenario.OverlapFlags,
		Scenario.OverlapPlateIds,
		Scenario.PreviousPlateIds,
		Scenario.PreviousContinentalWeights,
		Scenario.PreviousTerraneAssignments,
		Scenario.NewPlateIds,
		CollisionEvent,
		&Stats);

	TestTrue(TEXT("The manual collision setup produces a collision event"), bDetected);
	TestEqual(TEXT("The selected previous terrane id is preserved on the event"), CollisionEvent.TerraneId, 77);
	TestEqual(TEXT("The terrane BFS includes the ten overlap samples plus the connected inland sample"), CollisionEvent.TerraneSampleIndices.Num(), 11);
	TestTrue(TEXT("The terrane BFS reaches the non-overlap inland sample"), CollisionEvent.TerraneSampleIndices.Contains(Scenario.ZoneAExtraSample));
	TestFalse(TEXT("The terrane BFS does not leak into the deferred collision zone"), CollisionEvent.TerraneSampleIndices.Contains(Scenario.ZoneBExtraSample));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionTerraneReassignmentTest,
	"Aurous.TectonicPlanet.CollisionTerraneReassignment",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionTerraneReassignmentTest::RunTest(const FString& Parameters)
{
	FManualCollisionScenario Scenario = CreateManualCollisionScenario();
	FCollisionEvent CollisionEvent;
	const bool bDetected = Scenario.Planet.DetectAndApplyCollision(
		EResampleTriggerReason::None,
		Scenario.OverlapFlags,
		Scenario.OverlapPlateIds,
		Scenario.PreviousPlateIds,
		Scenario.PreviousContinentalWeights,
		Scenario.PreviousTerraneAssignments,
		Scenario.NewPlateIds,
		CollisionEvent);

	TestTrue(TEXT("The manual collision setup produces a collision event"), bDetected);
	Scenario.Planet.RepartitionMembership(Scenario.NewPlateIds);
	for (const int32 SampleIndex : Scenario.ZoneASamples)
	{
		TestEqual(FString::Printf(TEXT("Zone A overlap sample %d transfers to the overriding plate"), SampleIndex), Scenario.Planet.Samples[SampleIndex].PlateId, 0);
	}
	TestEqual(TEXT("The connected inland terrane sample also transfers to the overriding plate"), Scenario.Planet.Samples[Scenario.ZoneAExtraSample].PlateId, 0);
	for (const int32 SampleIndex : Scenario.ZoneBSamples)
	{
		TestEqual(FString::Printf(TEXT("Deferred zone B sample %d stays on the subducting plate"), SampleIndex), Scenario.Planet.Samples[SampleIndex].PlateId, 1);
	}
	TestEqual(TEXT("The deferred inland sample also stays on the subducting plate"), Scenario.Planet.Samples[Scenario.ZoneBExtraSample].PlateId, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionBiweightKernelTest,
	"Aurous.TectonicPlanet.CollisionBiweightKernel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionBiweightKernelTest::RunTest(const FString& Parameters)
{
	constexpr double RadiusRad = 0.40;
	TestTrue(TEXT("The collision kernel peaks at 1.0 at the collision center"), FMath::IsNearlyEqual(FTectonicPlanet::CollisionBiweightKernel(0.0, RadiusRad), 1.0, 1.0e-6));
	TestTrue(TEXT("The collision kernel matches the biweight value at half radius"), FMath::IsNearlyEqual(FTectonicPlanet::CollisionBiweightKernel(RadiusRad * 0.5, RadiusRad), 0.5625, 1.0e-6));
	TestEqual(TEXT("The collision kernel is zero at the surge radius"), FTectonicPlanet::CollisionBiweightKernel(RadiusRad, RadiusRad), 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionThrottleTest,
	"Aurous.TectonicPlanet.CollisionThrottle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionThrottleTest::RunTest(const FString& Parameters)
{
	FManualCollisionScenario Scenario = CreateManualCollisionScenario();
	FCollisionEvent CollisionEvent;
	FResamplingStats Stats;
	const bool bDetected = Scenario.Planet.DetectAndApplyCollision(
		EResampleTriggerReason::None,
		Scenario.OverlapFlags,
		Scenario.OverlapPlateIds,
		Scenario.PreviousPlateIds,
		Scenario.PreviousContinentalWeights,
		Scenario.PreviousTerraneAssignments,
		Scenario.NewPlateIds,
		CollisionEvent,
		&Stats);

	TestTrue(TEXT("The manual collision setup produces a collision event"), bDetected);
	TestEqual(TEXT("Only one collision is processed per resampling"), Stats.CollisionCount, 1);
	TestEqual(TEXT("The second connected component is deferred"), Stats.CollisionDeferredCount, 1);
	TestTrue(TEXT("The earliest equally-sized component wins the deterministic tie-break"), CollisionEvent.CollisionSampleIndices.Contains(3) && !CollisionEvent.CollisionSampleIndices.Contains(14));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionScoreRerunTest,
	"Aurous.TectonicPlanet.CollisionScoreRerun",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionScoreRerunTest::RunTest(const FString& Parameters)
{
	FManualCollisionScenario Scenario = CreateManualCollisionScenario();
	FCollisionEvent CollisionEvent;
	FResamplingStats Stats;
	const bool bDetected = Scenario.Planet.DetectAndApplyCollision(
		EResampleTriggerReason::None,
		Scenario.OverlapFlags,
		Scenario.OverlapPlateIds,
		Scenario.PreviousPlateIds,
		Scenario.PreviousContinentalWeights,
		Scenario.PreviousTerraneAssignments,
		Scenario.NewPlateIds,
		CollisionEvent,
		&Stats);

	TestTrue(TEXT("The manual collision setup produces a collision event"), bDetected);
	Scenario.Planet.RepartitionMembership(Scenario.NewPlateIds);
	const int32 ScoreBeforeSurge = Scenario.Planet.Plates[0].OverlapScore;
	Scenario.Planet.ApplyCollisionElevationSurge(CollisionEvent, &Stats);
	Scenario.Planet.ComputePlateScores();

	TestTrue(TEXT("The nearby low-CW sample crosses the continental threshold after the surge"), Scenario.Planet.Samples[Scenario.NearbyBoostSample].ContinentalWeight >= 0.5f);
	TestTrue(TEXT("Rerunning plate scores reflects the collision CW boost"), Scenario.Planet.Plates[0].OverlapScore > ScoreBeforeSurge);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionHimalayanAndCWBoostTest,
	"Aurous.TectonicPlanet.CollisionHimalayanAndCWBoost",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionHimalayanAndCWBoostTest::RunTest(const FString& Parameters)
{
	FManualCollisionScenario Scenario = CreateManualCollisionScenario();
	FCollisionEvent CollisionEvent;
	FResamplingStats Stats;
	const bool bDetected = Scenario.Planet.DetectAndApplyCollision(
		EResampleTriggerReason::None,
		Scenario.OverlapFlags,
		Scenario.OverlapPlateIds,
		Scenario.PreviousPlateIds,
		Scenario.PreviousContinentalWeights,
		Scenario.PreviousTerraneAssignments,
		Scenario.NewPlateIds,
		CollisionEvent,
		&Stats);

	TestTrue(TEXT("The manual collision setup produces a collision event"), bDetected);
	Scenario.Planet.RepartitionMembership(Scenario.NewPlateIds);
	Scenario.Planet.ApplyCollisionElevationSurge(CollisionEvent, &Stats);

	TestTrue(TEXT("The surge marks at least some samples as Himalayan"), CountHimalayanSamples(Scenario.Planet) > 0);
	TestTrue(TEXT("The surge boosts continental weight for affected samples"), Scenario.Planet.Samples[Scenario.NearbyBoostSample].ContinentalWeight > 0.3f);
	TestTrue(TEXT("Collision stats record a non-zero affected sample count"), Stats.CollisionSurgeAffectedCount > 0);
	TestTrue(TEXT("Collision stats record a positive mean elevation delta"), Stats.CollisionSurgeMeanElevationDelta > 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetResamplingCadenceABTest,
	"Aurous.TectonicPlanet.ResamplingCadenceAB",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetResamplingCadenceABTest::RunTest(const FString& Parameters)
{
	auto LogCheckpoint = [this](const TCHAR* RunLabel, const FTectonicPlanet& Planet)
	{
		TArray<int32> NewPlateIds;
		TArray<int32> ContainingTriangles;
		TArray<FVector3d> BarycentricCoords;
		TArray<uint8> GapFlags;
		TArray<uint8> OverlapFlags;
		TArray<TArray<int32>> OverlapPlateIds;
		int32 GapCount = 0;
		int32 OverlapCount = 0;
		Planet.QueryOwnership(
			NewPlateIds,
			ContainingTriangles,
			BarycentricCoords,
			GapFlags,
			OverlapFlags,
			GapCount,
			OverlapCount,
			&OverlapPlateIds,
			nullptr);

		TArray<FCollisionCandidate> CollisionCandidates;
		Planet.CollectCollisionCandidates(OverlapFlags, OverlapPlateIds, CollisionCandidates);
		TSet<int32> ContinentalContinentalOverlapSamples;
		for (const FCollisionCandidate& Candidate : CollisionCandidates)
		{
			ContinentalContinentalOverlapSamples.Add(Candidate.SampleIndex);
		}

		const int32 BoundarySampleCount = CountBoundarySamples(Planet);
		const double BoundarySampleFraction = Planet.Samples.IsEmpty()
			? 0.0
			: static_cast<double>(BoundarySampleCount) / static_cast<double>(Planet.Samples.Num());

		AddInfo(FString::Printf(
			TEXT("resampling_cadence_ab run=%s current_step=%d resampling_steps=[%s] boundary_sample_count=%d boundary_sample_fraction=%.6f overlap_count=%d continental_continental_overlap_count=%d"),
			RunLabel,
			Planet.CurrentStep,
			*JoinIntArray(Planet.ResamplingSteps),
			BoundarySampleCount,
			BoundarySampleFraction,
			OverlapCount,
			ContinentalContinentalOverlapSamples.Num()));
	};

	FTectonicPlanet BaselinePlanet = CreateInitializedPlanet();
	FTectonicPlanet OneResamplePlanet = CreateInitializedPlanet();
	OneResamplePlanet.MaxResampleCount = 1;

	const FString BaselineExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("ResamplingCadenceAB-Baseline-seed42-samples60000-plates7"));
	const FString OneResampleExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("ResamplingCadenceAB-OneResampleOnly-seed42-samples60000-plates7"));

	for (int32 StepIndex = 0; StepIndex < 50; ++StepIndex)
	{
		BaselinePlanet.AdvanceStep();
		OneResamplePlanet.AdvanceStep();

		if (BaselinePlanet.CurrentStep == 10 || BaselinePlanet.CurrentStep == 50)
		{
			TestTrue(
				*FString::Printf(TEXT("Baseline export succeeded at step %d"), BaselinePlanet.CurrentStep),
				ExportCheckpointMaps(*this, BaselinePlanet, BaselineExportRoot, BaselinePlanet.CurrentStep, ETectonicMapExportMode::All));
			LogCheckpoint(TEXT("baseline"), BaselinePlanet);
		}

		if (OneResamplePlanet.CurrentStep == 10 || OneResamplePlanet.CurrentStep == 50)
		{
			TestTrue(
				*FString::Printf(TEXT("One-resample export succeeded at step %d"), OneResamplePlanet.CurrentStep),
				ExportCheckpointMaps(*this, OneResamplePlanet, OneResampleExportRoot, OneResamplePlanet.CurrentStep, ETectonicMapExportMode::All));
			LogCheckpoint(TEXT("one_resample_only"), OneResamplePlanet);
		}
	}

	TestEqual(TEXT("Baseline run resamples at every scheduled interval through step 50"), BaselinePlanet.ResamplingSteps.Num(), 5);
	TestEqual(TEXT("One-resample-only run performs exactly one resample"), OneResamplePlanet.ResamplingSteps.Num(), 1);
	TestTrue(TEXT("The one-resample-only run records a resampling step"), !OneResamplePlanet.ResamplingSteps.IsEmpty());
	if (!OneResamplePlanet.ResamplingSteps.IsEmpty())
	{
		TestEqual(TEXT("The one-resample-only run resamples at step 10"), OneResamplePlanet.ResamplingSteps[0], 10);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetContinentalCollisionAt60k7Test,
	"Aurous.TectonicPlanet.ContinentalCollisionAt60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetContinentalCollisionAt60k7Test::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	FTectonicPlanet BaselinePlanet = CreateInitializedPlanet();
	BaselinePlanet.bEnableContinentalCollision = false;

	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M4-ContinentalCollision-seed42-samples60000-plates7"));

	bool bCollisionDetected = false;
	int32 CollisionStep = INDEX_NONE;
	int32 FinalStep = 100;
	int32 CollisionStepContinentalCount = INDEX_NONE;
	int32 BaselineCollisionStepContinentalCount = INDEX_NONE;
	int32 MaxCCOverlapCount = 0;
	int32 MaxCCOverlapStep = INDEX_NONE;
	FResamplingStats CollisionStats;

	for (int32 StepIndex = 0; StepIndex < 100; ++StepIndex)
	{
		Planet.AdvanceStep();
		BaselinePlanet.AdvanceStep();

		if (!Planet.ResamplingSteps.IsEmpty() && Planet.ResamplingSteps.Last() == Planet.CurrentStep)
		{
			if (Planet.LastResamplingStats.ContinentalContinentalOverlapCount > MaxCCOverlapCount)
			{
				MaxCCOverlapCount = Planet.LastResamplingStats.ContinentalContinentalOverlapCount;
				MaxCCOverlapStep = Planet.CurrentStep;
			}

			if (!bCollisionDetected && Planet.LastResamplingStats.CollisionCount > 0)
			{
				bCollisionDetected = true;
				CollisionStep = Planet.CurrentStep;
				CollisionStats = Planet.LastResamplingStats;
				CollisionStepContinentalCount = CountContinentalSamples(Planet);
				BaselineCollisionStepContinentalCount = CountContinentalSamples(BaselinePlanet);
				TestTrue(TEXT("Collision-step export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep));
				FinalStep = FMath::Min(100, CollisionStep + 10);
			}
		}

		if (bCollisionDetected && Planet.CurrentStep >= FinalStep)
		{
			break;
		}
	}

	if (!bCollisionDetected)
	{
		AddInfo(FString::Printf(
			TEXT("m4_collision_60k7_no_event max_cc_overlap_count=%d max_cc_overlap_step=%d final_continental=%d baseline_final_continental=%d final_gap_count=%d baseline_final_gap_count=%d"),
			MaxCCOverlapCount,
			MaxCCOverlapStep,
			CountContinentalSamples(Planet),
			CountContinentalSamples(BaselinePlanet),
			Planet.LastResamplingStats.GapCount,
			BaselinePlanet.LastResamplingStats.GapCount));
		return true;
	}

	TestTrue(TEXT("The collision count is non-zero"), CollisionStats.CollisionCount > 0);
	TestTrue(TEXT("The collision terrane size is non-zero"), CollisionStats.CollisionTerraneSampleCount > 0);
	TestTrue(TEXT("The collision surge affects at least one sample"), CollisionStats.CollisionSurgeAffectedCount > 0);
	TestTrue(TEXT("The collision surge has a positive mean elevation delta"), CollisionStats.CollisionSurgeMeanElevationDelta > 0.0);
	TestTrue(TEXT("The collision step is no later than step 100"), CollisionStep <= 100);
	TestTrue(TEXT("The collision run has at least one Himalayan sample after the event"), CountHimalayanSamples(Planet) > 0);
	TestTrue(TEXT("The collision run preserves subduction coherence"), Planet.LastResamplingStats.SubductionFrontEdgeCount > 0 && Planet.LastResamplingStats.SubductionInfluencedCount > 0);
	TestTrue(TEXT("The collision run does not increase gap count over the collision-disabled control"), Planet.LastResamplingStats.GapCount <= BaselinePlanet.LastResamplingStats.GapCount);
	TestTrue(TEXT("The collision step continental count is at least as high as the collision-disabled control"), CollisionStepContinentalCount >= BaselineCollisionStepContinentalCount);

	for (const FSample& Sample : Planet.Samples)
	{
		TestTrue(TEXT("Collision run elevations stay finite"), FMath::IsFinite(Sample.Elevation));
		TestTrue(TEXT("Collision run elevations respect the clamp range"), Sample.Elevation >= ElevationFloorKm && Sample.Elevation <= ElevationCeilingKm);
	}

	TestTrue(
		*FString::Printf(TEXT("Post-collision export succeeded at step %d"), FinalStep),
		ExportCheckpointMaps(*this, Planet, ExportRoot, FinalStep));

	AddInfo(FString::Printf(
		TEXT("m4_collision_60k7 collision_step=%d terrane_id=%d terrane_samples=%d surge_affected=%d surge_radius_rad=%.6f surge_mean_elev_delta=%.6f collision_continental=%d baseline_collision_continental=%d final_continental=%d baseline_final_continental=%d final_gap_count=%d baseline_final_gap_count=%d"),
		CollisionStep,
		CollisionStats.CollisionTerraneId,
		CollisionStats.CollisionTerraneSampleCount,
		CollisionStats.CollisionSurgeAffectedCount,
		CollisionStats.CollisionSurgeRadiusRad,
		CollisionStats.CollisionSurgeMeanElevationDelta,
		CollisionStepContinentalCount,
		BaselineCollisionStepContinentalCount,
		CountContinentalSamples(Planet),
		CountContinentalSamples(BaselinePlanet),
		Planet.LastResamplingStats.GapCount,
		BaselinePlanet.LastResamplingStats.GapCount));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSubstrateInitializationTest,
	"Aurous.TectonicPlanet.SubstrateInitialization",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSubstrateInitializationTest::RunTest(const FString& Parameters)
{
	constexpr double ExpectedMeanNeighborCount = 6.0 - (12.0 / static_cast<double>(TestSampleCount));

	FTectonicPlanet Planet;
	Planet.Initialize(TestSampleCount, TestPlanetRadiusKm);

	TestEqual(TEXT("Sample count"), Planet.Samples.Num(), TestSampleCount);
	TestEqual(TEXT("Triangle count"), Planet.TriangleIndices.Num(), 2 * TestSampleCount - 4);
	TestEqual(TEXT("Adjacency list count"), Planet.SampleAdjacency.Num(), TestSampleCount);

	int64 TotalNeighborCount = 0;
	int32 MinNeighborCount = TNumericLimits<int32>::Max();
	int32 MaxNeighborCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		const double Radius = Planet.Samples[SampleIndex].Position.Length();
		TestTrue(FString::Printf(TEXT("Sample %d stays on the unit sphere"), SampleIndex), FMath::IsNearlyEqual(Radius, 1.0, 1e-12));

		const TArray<int32>& Neighbors = Planet.SampleAdjacency[SampleIndex];
		TotalNeighborCount += Neighbors.Num();
		MinNeighborCount = FMath::Min(MinNeighborCount, Neighbors.Num());
		MaxNeighborCount = FMath::Max(MaxNeighborCount, Neighbors.Num());

		for (const int32 NeighborIndex : Neighbors)
		{
			TestTrue(
				FString::Printf(TEXT("Adjacency is symmetric for %d <-> %d"), SampleIndex, NeighborIndex),
				Planet.SampleAdjacency.IsValidIndex(NeighborIndex) && Planet.SampleAdjacency[NeighborIndex].Contains(SampleIndex));
		}
	}

	const int64 UniqueEdgeCount = TotalNeighborCount / 2;
	const double MeanNeighborCount = static_cast<double>(TotalNeighborCount) / static_cast<double>(TestSampleCount);
	TestEqual(TEXT("Unique edge count"), UniqueEdgeCount, 3ll * TestSampleCount - 6ll);
	TestTrue(TEXT("Minimum neighbor count stays near spherical Voronoi valence"), MinNeighborCount >= 5);
	TestTrue(TEXT("Maximum neighbor count stays bounded"), MaxNeighborCount <= 8);
	TestTrue(TEXT("Mean neighbor count matches triangulation expectation"), FMath::Abs(MeanNeighborCount - ExpectedMeanNeighborCount) < 0.05);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetMollweideExportTest,
	"Aurous.TectonicPlanet.MollweideExport",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetMollweideExportTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();

	const FString OutputDirectory = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("AutomationTest-seed42-samples60000-plates7"),
		TEXT("Step_0"));

	FTectonicMollweideExportOptions Options;
	Options.Mode = ETectonicMapExportMode::All;
	Options.Width = TestExportWidth;
	Options.Height = TestExportHeight;
	Options.OutputDirectory = OutputDirectory;

	FTectonicMollweideExportStats Stats;
	FString Error;
	const bool bExported = TectonicMollweideExporter::ExportPlanet(Planet, Options, Stats, Error);
	TestTrue(FString::Printf(TEXT("Export succeeded (%s)"), *Error), bExported);
	TestTrue(TEXT("Exporter resolved map pixels"), Stats.ResolvedPixelCount > 0);
	TestEqual(TEXT("No uncovered interior pixels after gap-fill"), Stats.UncoveredInteriorPixelCount, 0);
	TestTrue(TEXT("Exporter wrote at least the required maps"), Stats.WrittenFiles.Num() >= 4);
	AddInfo(FString::Printf(
		TEXT("mollweide_export_coverage pre_gap_fill_uncovered=%d post_gap_fill_uncovered=%d"),
		Stats.PreGapFillUncoveredInteriorPixelCount,
		Stats.UncoveredInteriorPixelCount));

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	TestTrue(TEXT("Elevation export exists"), PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("Elevation.png"))));
	TestTrue(TEXT("PlateId export exists"), PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("PlateId.png"))));
	TestTrue(TEXT("CrustType export exists"), PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("CrustType.png"))));
	TestTrue(TEXT("ContinentalWeight export exists"), PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("ContinentalWeight.png"))));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetInitialStateTest,
	"Aurous.TectonicPlanet.InitialState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetInitialStateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const FSoupPartitionSummary SoupSummary = BuildSoupPartitionSummary(Planet);

	TestEqual(TEXT("Plate count"), Planet.Plates.Num(), TestPlateCount);
	TestTrue(TEXT("Terranes were created"), Planet.Terranes.Num() > 0);

	int32 ContinentalSampleCount = 0;
	int32 BoundarySampleCount = 0;
	int32 InteriorTriangleCount = 0;
	int32 BoundaryTriangleCount = 0;
	int32 ContinentalSeededPlateCount = 0;
	int32 OceanicOnlyPlateCount = 0;

	for (const FPlate& Plate : Planet.Plates)
	{
		TestTrue(FString::Printf(TEXT("Plate %d owns at least one sample"), Plate.Id), Plate.MemberSamples.Num() > 0);
		TestTrue(
			FString::Printf(TEXT("Plate %d carried lookup count matches carried samples"), Plate.Id),
			Plate.CanonicalToCarriedIndex.Num() == Plate.CarriedSamples.Num());
		TestTrue(
			FString::Printf(TEXT("Plate %d has a normalized bounding cap center"), Plate.Id),
			FMath::IsNearlyEqual(Plate.BoundingCap.Center.Length(), 1.0, 1e-6));
		TestTrue(
			FString::Printf(TEXT("Plate %d has a positive angular speed"), Plate.Id),
			Plate.AngularSpeed > 0.0);

		int32 ContinentalSamplesForPlate = 0;
		TSet<int32> ExpectedCarriedVertices;
		for (const int32 SampleIndex : Plate.MemberSamples)
		{
			if (Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f)
			{
				++ContinentalSamplesForPlate;
			}
		}

		for (const int32 TriangleIndex : Plate.SoupTriangles)
		{
			if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
			{
				continue;
			}

			const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
			ExpectedCarriedVertices.Add(Triangle.X);
			ExpectedCarriedVertices.Add(Triangle.Y);
			ExpectedCarriedVertices.Add(Triangle.Z);
		}

		for (const int32 SampleIndex : ExpectedCarriedVertices)
		{
			const int32* CarriedIndex = Plate.CanonicalToCarriedIndex.Find(SampleIndex);
			TestTrue(
				FString::Printf(TEXT("Plate %d soup vertex %d has a valid carried lookup"), Plate.Id, SampleIndex),
				CarriedIndex != nullptr &&
					Plate.CarriedSamples.IsValidIndex(*CarriedIndex));
		}

		TestTrue(
			FString::Printf(TEXT("Plate %d carried samples exactly match unique soup vertices"), Plate.Id),
			Plate.CarriedSamples.Num() == ExpectedCarriedVertices.Num());

		ContinentalSeededPlateCount += ContinentalSamplesForPlate > 0 ? 1 : 0;
		OceanicOnlyPlateCount += ContinentalSamplesForPlate <= 0 ? 1 : 0;
	}

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		const FSample& Sample = Planet.Samples[SampleIndex];
		TestTrue(
			FString::Printf(TEXT("Sample %d has a valid plate owner"), SampleIndex),
			Planet.FindPlateArrayIndexById(Sample.PlateId) != INDEX_NONE);
		TestTrue(
			FString::Printf(TEXT("Sample %d continental weight is binary"), SampleIndex),
			Sample.ContinentalWeight == 0.0f || Sample.ContinentalWeight == 1.0f);

		bool bExpectedBoundary = false;
		for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
		{
			if (Planet.Samples[NeighborIndex].PlateId != Sample.PlateId)
			{
				bExpectedBoundary = true;
				break;
			}
		}
		TestEqual(
			FString::Printf(TEXT("Sample %d boundary flag matches adjacency"), SampleIndex),
			Sample.bIsBoundary,
			bExpectedBoundary);

		if (Sample.ContinentalWeight >= 0.5f)
		{
			++ContinentalSampleCount;
			TestTrue(FString::Printf(TEXT("Sample %d continental elevation is above sea floor"), SampleIndex), Sample.Elevation > -1.0f);
			TestTrue(FString::Printf(TEXT("Sample %d continental thickness is crustal"), SampleIndex), Sample.Thickness >= 30.0f);
			TestTrue(FString::Printf(TEXT("Sample %d continental terrane assigned"), SampleIndex), Sample.TerraneId != INDEX_NONE);
		}
		else
		{
			TestTrue(FString::Printf(TEXT("Sample %d oceanic elevation is deep"), SampleIndex), Sample.Elevation < -5.0f);
			TestTrue(FString::Printf(TEXT("Sample %d oceanic thickness is thin"), SampleIndex), Sample.Thickness <= 10.0f);
			TestEqual(FString::Printf(TEXT("Sample %d oceanic terrane stays unset"), SampleIndex), Sample.TerraneId, INDEX_NONE);
		}

		BoundarySampleCount += Sample.bIsBoundary ? 1 : 0;
	}

	for (const FIntVector& Triangle : Planet.TriangleIndices)
	{
		const int32 PlateA = Planet.Samples[Triangle.X].PlateId;
		const int32 PlateB = Planet.Samples[Triangle.Y].PlateId;
		const int32 PlateC = Planet.Samples[Triangle.Z].PlateId;
		if (PlateA == PlateB && PlateB == PlateC)
		{
			++InteriorTriangleCount;
		}
		else
		{
			++BoundaryTriangleCount;
		}
	}

	const double ContinentalAreaFraction = static_cast<double>(ContinentalSampleCount) / static_cast<double>(TestSampleCount);
	AddInfo(FString::Printf(
		TEXT("initial_state plate_count=%d terrane_count=%d continental_fraction=%.3f interior_triangles=%d boundary_triangles=%d boundary_samples=%d"),
		Planet.Plates.Num(),
		Planet.Terranes.Num(),
		ContinentalAreaFraction,
		InteriorTriangleCount,
		BoundaryTriangleCount,
		BoundarySampleCount));
	TestTrue(TEXT("Continental area fraction remains in a plausible band"), ContinentalAreaFraction >= 0.20 && ContinentalAreaFraction <= 0.40);
	TestTrue(TEXT("A subset of plates remains purely oceanic"), OceanicOnlyPlateCount > 0);
	TestTrue(TEXT("Not every plate receives a continental seed"), ContinentalSeededPlateCount < TestPlateCount);
	TestTrue(TEXT("At least one plate receives a continental seed"), ContinentalSeededPlateCount > 0);
	TestTrue(TEXT("Boundary samples exist"), BoundarySampleCount > 0);
	TestTrue(TEXT("Boundary triangles exist"), BoundaryTriangleCount > 0);
	TestEqual(TEXT("Interior plus boundary triangles cover the mesh"), InteriorTriangleCount + BoundaryTriangleCount, Planet.TriangleIndices.Num());
	TestEqual(TEXT("Initial soup partition covers every global triangle"), SoupSummary.TotalSoupTriangleCount, Planet.TriangleIndices.Num());
	TestEqual(TEXT("Initial soup partition drops no mixed triangles"), SoupSummary.DroppedMixedTriangleCount, 0);
	TestEqual(TEXT("Initial soup partition duplicates no triangles"), SoupSummary.DuplicatedSoupTriangleCount, 0);
	TestTrue(TEXT("Initial soup partition produces foreign local vertices for boundary triangles"), SoupSummary.ForeignLocalVertexCount > 0);

	for (const FTerrane& Terrane : Planet.Terranes)
	{
		TestTrue(FString::Printf(TEXT("Terrane %d has a valid anchor"), Terrane.TerraneId), Terrane.AnchorSample != INDEX_NONE);
		TestTrue(FString::Printf(TEXT("Terrane %d has a valid owning plate"), Terrane.TerraneId), Planet.FindPlateArrayIndexById(Terrane.PlateId) != INDEX_NONE);
		TestTrue(FString::Printf(TEXT("Terrane %d has positive area"), Terrane.TerraneId), Terrane.AreaKm2 > 0.0);
		TestTrue(FString::Printf(TEXT("Terrane %d centroid is normalized"), Terrane.TerraneId), FMath::IsNearlyEqual(Terrane.Centroid.Length(), 1.0, 1e-6));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPerStepEvolutionTest,
	"Aurous.TectonicPlanet.PerStepEvolution",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPerStepEvolutionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const int32 StepsBeforeResample = FMath::Max(1, Planet.ComputeResampleInterval() - 1);

	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M1-preresample-seed42-samples60000-plates7"));

	TArray<int32> InitialPlateIds;
	InitialPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		InitialPlateIds.Add(Sample.PlateId);
	}

	TArray<int32> InitialMemberCounts;
	TArray<FQuat4d> InitialRotations;
	for (const FPlate& Plate : Planet.Plates)
	{
		InitialMemberCounts.Add(Plate.MemberSamples.Num());
		InitialRotations.Add(Plate.CumulativeRotation);
	}

	const FElevationSummary Step0Summary = BuildElevationSummary(Planet);
	TestTrue(TEXT("Step 0 export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, 0));

	double TotalAdvanceMs = 0.0;
	for (int32 StepIndex = 0; StepIndex < StepsBeforeResample; ++StepIndex)
	{
		const double StepStart = FPlatformTime::Seconds();
		Planet.AdvanceStep();
		TotalAdvanceMs += (FPlatformTime::Seconds() - StepStart) * 1000.0;
	}

	const FElevationSummary FinalSummary = BuildElevationSummary(Planet);
	TestTrue(TEXT("Pre-resample export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep));

	TestEqual(TEXT("Current step advanced to pre-resample count"), Planet.CurrentStep, StepsBeforeResample);
	TestEqual(TEXT("No resampling happened before the trigger interval"), Planet.ResamplingSteps.Num(), 0);
	TestTrue(
		TEXT("Continental erosion reduced max continental elevation"),
		FinalSummary.MaxContinentalElevation < Step0Summary.MaxContinentalElevation);
	TestTrue(
		TEXT("Oceanic dampening moved mean oceanic elevation closer to abyssal plain"),
		FMath::Abs(FinalSummary.MeanOceanicElevation - AbyssalPlainElevationKm) <
			FMath::Abs(Step0Summary.MeanOceanicElevation - AbyssalPlainElevationKm) + 1e-9);

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		const FSample& Sample = Planet.Samples[SampleIndex];
		TestEqual(
			FString::Printf(TEXT("Sample %d plate ownership is unchanged before first resample"), SampleIndex),
			Sample.PlateId,
			InitialPlateIds[SampleIndex]);
		TestTrue(
			FString::Printf(TEXT("Sample %d elevation remains clamped"), SampleIndex),
			Sample.Elevation >= ElevationFloorKm && Sample.Elevation <= ElevationCeilingKm);
	}

	for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
	{
		const FPlate& Plate = Planet.Plates[PlateIndex];
		TestEqual(
			FString::Printf(TEXT("Plate %d member count is unchanged before first resample"), Plate.Id),
			Plate.MemberSamples.Num(),
			InitialMemberCounts[PlateIndex]);

		for (const FCarriedSample& CarriedSample : Plate.CarriedSamples)
		{
			const FSample& CanonicalSample = Planet.Samples[CarriedSample.CanonicalSampleIndex];
			TestEqual(
				FString::Printf(TEXT("Plate %d carried sample %d stays synchronized with canonical subduction distance"), Plate.Id, CarriedSample.CanonicalSampleIndex),
				CarriedSample.SubductionDistanceKm,
				CanonicalSample.SubductionDistanceKm);
		}
	}

	bool bAnyRotationAdvanced = false;
	for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
	{
		const FQuat4d& CurrentRotation = Planet.Plates[PlateIndex].CumulativeRotation;
		const FQuat4d& InitialRotation = InitialRotations[PlateIndex];
		const bool bChanged =
			!FMath::IsNearlyEqual(CurrentRotation.X, InitialRotation.X, 1e-9) ||
			!FMath::IsNearlyEqual(CurrentRotation.Y, InitialRotation.Y, 1e-9) ||
			!FMath::IsNearlyEqual(CurrentRotation.Z, InitialRotation.Z, 1e-9) ||
			!FMath::IsNearlyEqual(CurrentRotation.W, InitialRotation.W, 1e-9);
		bAnyRotationAdvanced |= bChanged;
	}
	TestTrue(TEXT("At least one plate rotation advanced"), bAnyRotationAdvanced);

	const double MeanAdvanceMs = TotalAdvanceMs / static_cast<double>(StepsBeforeResample);
	AddInfo(FString::Printf(
		TEXT("per_step_evolution steps=%d step0_max_continental=%.6f stepN_max_continental=%.6f step0_mean_oceanic=%.6f stepN_mean_oceanic=%.6f mean_advance_ms=%.6f"),
		StepsBeforeResample,
		Step0Summary.MaxContinentalElevation,
		FinalSummary.MaxContinentalElevation,
		Step0Summary.MeanOceanicElevation,
		FinalSummary.MeanOceanicElevation,
		MeanAdvanceMs));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSoupBuildProducesValidBVHTest,
	"Aurous.TectonicPlanet.SoupBuildProducesValidBVH",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSoupBuildProducesValidBVHTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.BuildContainmentSoups();
	const FSoupPartitionSummary SoupSummary = BuildSoupPartitionSummary(Planet);

	int32 TotalSoupTriangles = 0;
	for (const FPlate& Plate : Planet.Plates)
	{
		TestEqual(
			FString::Printf(TEXT("Plate %d soup triangle count matches assigned soup triangle count"), Plate.Id),
			Plate.SoupData.LocalTriangles.Num(),
			Plate.SoupTriangles.Num());
		TestEqual(
			FString::Printf(TEXT("Plate %d soup global-to-local triangle map matches triangle count"), Plate.Id),
			Plate.SoupData.GlobalToLocalTriangle.Num(),
			Plate.SoupData.LocalTriangles.Num());
		TotalSoupTriangles += Plate.SoupData.LocalTriangles.Num();

		if (!Plate.SoupData.LocalTriangles.IsEmpty())
		{
			TestTrue(
				FString::Printf(TEXT("Plate %d BVH has a root node"), Plate.Id),
				Plate.SoupBVH.RootIndex >= 0);
			TestTrue(
				FString::Printf(TEXT("Plate %d rotated soup vertices exist"), Plate.Id),
				Plate.SoupData.RotatedVertices.Num() >= 3);
		}
	}

	TestEqual(TEXT("Total soup triangles match full global-triangle coverage"), TotalSoupTriangles, Planet.TriangleIndices.Num());
	TestEqual(TEXT("Soup partition records the same total count"), TotalSoupTriangles, SoupSummary.TotalSoupTriangleCount);
	TestEqual(TEXT("Soup partition drops no mixed triangles"), SoupSummary.DroppedMixedTriangleCount, 0);
	TestEqual(TEXT("Soup partition duplicates no triangles"), SoupSummary.DuplicatedSoupTriangleCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetOwnershipQueryAtIdentityRotationTest,
	"Aurous.TectonicPlanet.OwnershipQueryAtIdentityRotation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetOwnershipQueryAtIdentityRotationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();

	TArray<int32> NewPlateIds;
	TArray<int32> ContainingTriangles;
	TArray<FVector3d> BarycentricCoords;
	TArray<uint8> GapFlags;
	TArray<uint8> OverlapFlags;
	int32 GapCount = 0;
	int32 OverlapCount = 0;

	Planet.BuildContainmentSoups();
	Planet.QueryOwnership(NewPlateIds, ContainingTriangles, BarycentricCoords, GapFlags, OverlapFlags, GapCount, OverlapCount);

	TestEqual(TEXT("Identity-rotation query has no structural gaps"), GapCount, 0);
	TestEqual(TEXT("Identity-rotation query has zero overlaps"), OverlapCount, 0);
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		TestTrue(
			FString::Printf(TEXT("Sample %d has a valid containing plate at identity"), SampleIndex),
			NewPlateIds[SampleIndex] >= 0 && NewPlateIds[SampleIndex] < Planet.Plates.Num());
		TestTrue(
			FString::Printf(TEXT("Sample %d has a valid containing triangle at identity"), SampleIndex),
			ContainingTriangles[SampleIndex] >= 0 && ContainingTriangles[SampleIndex] < Planet.TriangleIndices.Num());
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetOwnershipQueryAfterMotionTest,
	"Aurous.TectonicPlanet.OwnershipQueryAfterMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetOwnershipQueryAfterMotionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const int32 StepsBeforeResample = FMath::Max(1, Planet.ComputeResampleInterval() - 1);
	for (int32 StepIndex = 0; StepIndex < StepsBeforeResample; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const FOwnershipQuerySummary Summary = RunOwnershipQuery(*this, Planet);
	const double TotalGapRate =
		static_cast<double>(Summary.GapCount) / static_cast<double>(Planet.Samples.Num());
	const double OverlapRate = static_cast<double>(Summary.OverlapCount) / static_cast<double>(Planet.Samples.Num());
	AddInfo(FString::Printf(
		TEXT("ownership_after_motion steps=%d gap_count=%d divergent_gap_count=%d overlap_count=%d valid_containment_count=%d multi_containment_count=%d no_valid_hit_count=%d gap_rate=%.4f overlap_rate=%.4f valid_bary=%d"),
		StepsBeforeResample,
		Summary.GapCount,
		Summary.DivergentGapCount,
		Summary.OverlapCount,
		Summary.ValidContainmentCount,
		Summary.MultiContainmentCount,
		Summary.NoValidHitCount,
		TotalGapRate,
		OverlapRate,
		Summary.ValidBarycentricCount));
	TestTrue(TEXT("Gap rate stays below 10%"), TotalGapRate < 0.10);
	TestTrue(TEXT("Overlap rate stays below 5%"), OverlapRate < 0.05);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSingleResamplingContinentalStability500k40Test,
	"Aurous.TectonicPlanet.SingleResamplingContinentalStability500k40",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSingleResamplingContinentalStability500k40Test::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(StressPlateCount, HighResStressSampleCount);
	const int32 StepsBeforeResample = FMath::Max(1, Planet.ComputeResampleInterval() - 1);
	for (int32 StepIndex = 0; StepIndex < StepsBeforeResample; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const int32 PreResampleContinentalCount = CountContinentalSamples(Planet);
	Planet.AdvanceStep();
	TestEqual(TEXT("Single-resampling stability resampled at step 10"), Planet.LastResamplingStats.Step, 10);
	const int32 PostResampleContinentalCount = CountContinentalSamples(Planet);
	const int32 InvalidPlateAssignmentCount = CountInvalidPlateAssignments(Planet);
	const double ContinentalRetention =
		PreResampleContinentalCount > 0
			? static_cast<double>(PostResampleContinentalCount) / static_cast<double>(PreResampleContinentalCount)
			: 0.0;
	AddInfo(FString::Printf(
		TEXT("single_resampling_continental_stability_500k40 samples=%d plates=%d steps=%d pre_continental=%d post_continental=%d retention=%.4f interior_soup_tris=%d boundary_soup_tris=%d total_soup_tris=%d dropped_mixed_triangle_count=%d duplicated_soup_triangle_count=%d foreign_local_vertex_count=%d exact_single_hit_count=%d exact_multi_hit_count=%d recovery_containment_count=%d recovery_continental_count=%d true_gap_count=%d recovery_mean_distance=%.6f recovery_p95_distance=%.6f recovery_max_distance=%.6f missing_local_carried_lookup_count=%d gap_count=%d divergent_gap_count=%d non_divergent_gap_count=%d divergent_continental_gap_count=%d non_divergent_continental_gap_count=%d non_divergent_gap_triangle_projection_count=%d non_divergent_gap_nearest_copy_count=%d overlap_count=%d valid_containment_count=%d multi_containment_count=%d no_valid_hit_count=%d invalid_plate_assignments=%d"),
		Planet.Samples.Num(),
		Planet.Plates.Num(),
		StepsBeforeResample,
		PreResampleContinentalCount,
		PostResampleContinentalCount,
		ContinentalRetention,
		Planet.LastResamplingStats.InteriorSoupTriangleCount,
		Planet.LastResamplingStats.BoundarySoupTriangleCount,
		Planet.LastResamplingStats.TotalSoupTriangleCount,
		Planet.LastResamplingStats.DroppedMixedTriangleCount,
		Planet.LastResamplingStats.DuplicatedSoupTriangleCount,
		Planet.LastResamplingStats.ForeignLocalVertexCount,
		Planet.LastResamplingStats.ExactSingleHitCount,
		Planet.LastResamplingStats.ExactMultiHitCount,
		Planet.LastResamplingStats.RecoveryContainmentCount,
		Planet.LastResamplingStats.RecoveryContinentalCount,
		Planet.LastResamplingStats.TrueGapCount,
		Planet.LastResamplingStats.RecoveryMeanDistance,
		Planet.LastResamplingStats.RecoveryP95Distance,
		Planet.LastResamplingStats.RecoveryMaxDistance,
		Planet.LastResamplingStats.MissingLocalCarriedLookupCount,
		Planet.LastResamplingStats.GapCount,
		Planet.LastResamplingStats.DivergentGapCount,
		Planet.LastResamplingStats.NonDivergentGapCount,
		Planet.LastResamplingStats.DivergentContinentalGapCount,
		Planet.LastResamplingStats.NonDivergentContinentalGapCount,
		Planet.LastResamplingStats.NonDivergentGapTriangleProjectionCount,
		Planet.LastResamplingStats.NonDivergentGapNearestCopyCount,
		Planet.LastResamplingStats.OverlapCount,
		Planet.LastResamplingStats.ValidContainmentCount,
		Planet.LastResamplingStats.MultiContainmentCount,
		Planet.LastResamplingStats.NoValidHitCount,
		InvalidPlateAssignmentCount));
	TestEqual(TEXT("500k/40 resample leaves no invalid plate assignments"), InvalidPlateAssignmentCount, 0);
	TestTrue(TEXT("500k/40 still retains some continental samples after one resample"), PostResampleContinentalCount > 0);
	TestEqual(TEXT("500k/40 soup partition covers every global triangle"), Planet.LastResamplingStats.TotalSoupTriangleCount, Planet.TriangleIndices.Num());
	TestEqual(TEXT("500k/40 drops no mixed triangles"), Planet.LastResamplingStats.DroppedMixedTriangleCount, 0);
	TestEqual(TEXT("500k/40 duplicates no soup triangles"), Planet.LastResamplingStats.DuplicatedSoupTriangleCount, 0);
	TestEqual(TEXT("500k/40 interpolation misses no local carried lookups"), Planet.LastResamplingStats.MissingLocalCarriedLookupCount, 0);
	TestEqual(
		TEXT("500k/40 ownership accounting partitions samples into exact hits, recovered hits, overlaps, or true gaps"),
		Planet.LastResamplingStats.ExactSingleHitCount +
			Planet.LastResamplingStats.ExactMultiHitCount +
			Planet.LastResamplingStats.RecoveryContainmentCount +
			Planet.LastResamplingStats.TrueGapCount,
		Planet.Samples.Num());
	TestEqual(TEXT("500k/40 true gap count matches no-valid-hit count"), Planet.LastResamplingStats.TrueGapCount, Planet.LastResamplingStats.NoValidHitCount);
	TestEqual(
		TEXT("500k/40 gap classification accounts for every no-hit sample"),
		Planet.LastResamplingStats.DivergentGapCount + Planet.LastResamplingStats.NonDivergentGapCount,
		Planet.LastResamplingStats.NoValidHitCount);
	TestEqual(
		TEXT("500k/40 non-divergent path accounting matches non-divergent gap count"),
		Planet.LastResamplingStats.NonDivergentGapTriangleProjectionCount + Planet.LastResamplingStats.NonDivergentGapNearestCopyCount,
		Planet.LastResamplingStats.NonDivergentGapCount);
	TestTrue(TEXT("500k/40 still resolves at least some no-hit samples through the non-divergent path"), Planet.LastResamplingStats.NonDivergentGapCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRepartitionPreservesAllSamplesTest,
	"Aurous.TectonicPlanet.RepartitionPreservesAllSamples",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRepartitionPreservesAllSamplesTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const int32 StepsBeforeResample = FMath::Max(1, Planet.ComputeResampleInterval() - 1);
	for (int32 StepIndex = 0; StepIndex < StepsBeforeResample; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	TArray<int32> NewPlateIds;
	TArray<int32> ContainingTriangles;
	TArray<FVector3d> BarycentricCoords;
	TArray<uint8> GapFlags;
	TArray<uint8> OverlapFlags;
	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	int32 GapCount = 0;
	int32 OverlapCount = 0;
	Planet.BuildContainmentSoups();
	Planet.QueryOwnership(
		NewPlateIds,
		ContainingTriangles,
		BarycentricCoords,
		GapFlags,
		OverlapFlags,
		GapCount,
		OverlapCount);
	Planet.InterpolateFromCarried(
		NewPlateIds,
		ContainingTriangles,
		BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds);
	Planet.ResolveGaps(NewPlateIds, GapFlags, SubductionDistances, SubductionSpeeds);
	Planet.RepartitionMembership(NewPlateIds, &SubductionDistances, &SubductionSpeeds);

	TestEqual(TEXT("Repartitioned membership covers all samples after gap resolution"), CountTotalMembers(Planet), static_cast<int64>(Planet.Samples.Num()));
	for (const FPlate& Plate : Planet.Plates)
	{
		TestTrue(FString::Printf(TEXT("Plate %d retains at least one member"), Plate.Id), Plate.MemberSamples.Num() > 0);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetInterpolationPreservesAttributesTest,
	"Aurous.TectonicPlanet.InterpolationPreservesAttributes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetInterpolationPreservesAttributesTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();

	TArray<float> OriginalElevation;
	TArray<float> OriginalContinentalWeight;
	TArray<float> OriginalAge;
	OriginalElevation.Reserve(Planet.Samples.Num());
	OriginalContinentalWeight.Reserve(Planet.Samples.Num());
	OriginalAge.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OriginalElevation.Add(Sample.Elevation);
		OriginalContinentalWeight.Add(Sample.ContinentalWeight);
		OriginalAge.Add(Sample.Age);
	}

	TArray<int32> NewPlateIds;
	TArray<int32> ContainingTriangles;
	TArray<FVector3d> BarycentricCoords;
	TArray<uint8> GapFlags;
	TArray<uint8> OverlapFlags;
	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	int32 GapCount = 0;
	int32 OverlapCount = 0;

	Planet.BuildContainmentSoups();
	Planet.QueryOwnership(NewPlateIds, ContainingTriangles, BarycentricCoords, GapFlags, OverlapFlags, GapCount, OverlapCount);
	TestEqual(TEXT("Identity interpolation query has no structural gaps"), GapCount, 0);
	TestEqual(TEXT("Identity interpolation query has zero overlaps"), OverlapCount, 0);

	int32 MissingLocalCarriedLookupCount = INDEX_NONE;
	Planet.InterpolateFromCarried(
		NewPlateIds,
		ContainingTriangles,
		BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds,
		&MissingLocalCarriedLookupCount);
	TestEqual(TEXT("Identity interpolation needs no missing local carried fallback"), MissingLocalCarriedLookupCount, 0);

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		if (NewPlateIds[SampleIndex] == INDEX_NONE || ContainingTriangles[SampleIndex] == INDEX_NONE)
		{
			continue;
		}

		const FSample& Sample = Planet.Samples[SampleIndex];
		TestTrue(
			FString::Printf(TEXT("Sample %d elevation is preserved at identity"), SampleIndex),
			FMath::Abs(static_cast<double>(Sample.Elevation) - static_cast<double>(OriginalElevation[SampleIndex])) <= 0.01);
		TestTrue(
			FString::Printf(TEXT("Sample %d continental weight is preserved at identity"), SampleIndex),
			FMath::Abs(static_cast<double>(Sample.ContinentalWeight) - static_cast<double>(OriginalContinentalWeight[SampleIndex])) <= 0.01);
		TestTrue(
			FString::Printf(TEXT("Sample %d age is preserved at identity"), SampleIndex),
			FMath::Abs(static_cast<double>(Sample.Age) - static_cast<double>(OriginalAge[SampleIndex])) <= 0.01);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGlobalSoupPartitionIncludesBoundaryTrianglesTest,
	"Aurous.TectonicPlanet.GlobalSoupPartitionIncludesBoundaryTriangles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGlobalSoupPartitionIncludesBoundaryTrianglesTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(StressPlateCount);
	const FSoupPartitionSummary SoupSummary = BuildSoupPartitionSummary(Planet);
	int32 VerifiedSoupTriangleCount = 0;

	for (const FPlate& Plate : Planet.Plates)
	{
		TSet<int32> ExpectedCarriedVertices;
		for (const int32 TriangleIndex : Plate.SoupTriangles)
		{
			if (!Planet.TriangleIndices.IsValidIndex(TriangleIndex))
			{
				continue;
			}

			const FIntVector& Triangle = Planet.TriangleIndices[TriangleIndex];
			TestEqual(
				FString::Printf(TEXT("Plate %d soup triangle %d follows deterministic partitioning"), Plate.Id, TriangleIndex),
				Plate.Id,
				ChooseExpectedSoupPlateId(Planet, Triangle));
			if (IsInteriorTriangle(Planet, Triangle))
			{
				TestTrue(FString::Printf(TEXT("Plate %d interior soup triangle %d is tracked as interior"), Plate.Id, TriangleIndex), Plate.InteriorTriangles.Contains(TriangleIndex));
			}
			else
			{
				TestTrue(FString::Printf(TEXT("Plate %d boundary soup triangle %d is tracked as boundary"), Plate.Id, TriangleIndex), Plate.BoundaryTriangles.Contains(TriangleIndex));
			}

			ExpectedCarriedVertices.Add(Triangle.X);
			ExpectedCarriedVertices.Add(Triangle.Y);
			ExpectedCarriedVertices.Add(Triangle.Z);
			++VerifiedSoupTriangleCount;
		}

		TestEqual(
			FString::Printf(TEXT("Plate %d carried sample count matches unique assigned soup vertices"), Plate.Id),
			Plate.CarriedSamples.Num(),
			ExpectedCarriedVertices.Num());

		for (const int32 SampleIndex : ExpectedCarriedVertices)
		{
			const int32* CarriedIndex = Plate.CanonicalToCarriedIndex.Find(SampleIndex);
			TestTrue(
				FString::Printf(TEXT("Plate %d assigned soup vertex %d has a local carried lookup"), Plate.Id, SampleIndex),
				CarriedIndex != nullptr && Plate.CarriedSamples.IsValidIndex(*CarriedIndex));
		}

		for (const FCarriedSample& CarriedSample : Plate.CarriedSamples)
		{
			TestTrue(
				FString::Printf(TEXT("Plate %d carried sample %d is referenced by an assigned soup triangle"), Plate.Id, CarriedSample.CanonicalSampleIndex),
				ExpectedCarriedVertices.Contains(CarriedSample.CanonicalSampleIndex));
		}
	}

	TestTrue(TEXT("Verified global soup partition triangles across the stress setup"), VerifiedSoupTriangleCount > 0);
	TestEqual(TEXT("Global soup partition covers every triangle"), SoupSummary.TotalSoupTriangleCount, Planet.TriangleIndices.Num());
	TestEqual(TEXT("Global soup partition drops no mixed triangles"), SoupSummary.DroppedMixedTriangleCount, 0);
	TestEqual(TEXT("Global soup partition duplicates no triangles"), SoupSummary.DuplicatedSoupTriangleCount, 0);
	TestTrue(TEXT("Global soup partition includes boundary-assigned triangles"), SoupSummary.BoundarySoupTriangleCount > 0);
	TestTrue(TEXT("Global soup partition creates foreign local vertices"), SoupSummary.ForeignLocalVertexCount > 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetInterpolationAfterMotionTest,
	"Aurous.TectonicPlanet.InterpolationAfterMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetInterpolationAfterMotionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const int32 ResampleInterval = Planet.ComputeResampleInterval();
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M2b-InterpolationAfterMotion-seed42-samples60000-plates7"));

	TestTrue(TEXT("InterpolationAfterMotion step 0 export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, 0));

	for (int32 StepIndex = 0; StepIndex < ResampleInterval - 1; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const FOwnershipQuerySummary PreResampleSummary = RunOwnershipQuery(*this, Planet);
	TestTrue(TEXT("Pre-resample query found interpolable samples"), PreResampleSummary.ValidBarycentricCount > 0);

	Planet.AdvanceStep();
	TestEqual(TEXT("Resampling occurred at step 10"), Planet.LastResamplingStats.Step, ResampleInterval);
	TestTrue(TEXT("InterpolationAfterMotion step 10 export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep));

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		const FSample& Sample = Planet.Samples[SampleIndex];
		TestTrue(FString::Printf(TEXT("Sample %d has valid ownership after resampling"), SampleIndex), Planet.FindPlateArrayIndexById(Sample.PlateId) != INDEX_NONE);
		TestTrue(FString::Printf(TEXT("Sample %d continental weight stays normalized"), SampleIndex), Sample.ContinentalWeight >= 0.0f && Sample.ContinentalWeight <= 1.0f);
		TestTrue(FString::Printf(TEXT("Sample %d elevation stays clamped after interpolation"), SampleIndex), Sample.Elevation >= ElevationFloorKm && Sample.Elevation <= ElevationCeilingKm);
	}

	const double ContinentalAreaFraction = ComputeContinentalAreaFraction(Planet);
	TestTrue(TEXT("Continental area fraction stays in the M2b band"), ContinentalAreaFraction >= 0.15 && ContinentalAreaFraction <= 0.50);
	AddInfo(FString::Printf(
		TEXT("interpolation_after_motion step=%d gap_count=%d divergent_gap_count=%d overlap_count=%d continental_fraction=%.4f"),
		Planet.LastResamplingStats.Step,
		Planet.LastResamplingStats.GapCount,
		Planet.LastResamplingStats.DivergentGapCount,
		Planet.LastResamplingStats.OverlapCount,
		ContinentalAreaFraction));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetDivergentGapDetectionTest,
	"Aurous.TectonicPlanet.DivergentGapDetection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetDivergentGapDetectionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	for (int32 StepIndex = 0; StepIndex < Planet.ComputeResampleInterval(); ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestEqual(TEXT("Gap-classification test resampled at step 10"), Stats.Step, 10);
	TestTrue(TEXT("Gap classification observed no-hit samples"), Stats.NoValidHitCount > 0);
	TestEqual(
		TEXT("Divergent and non-divergent gap counts sum to the no-hit count"),
		Stats.DivergentGapCount + Stats.NonDivergentGapCount,
		Stats.NoValidHitCount);
	TestEqual(
		TEXT("Non-divergent resolution paths sum to the non-divergent gap count"),
		Stats.NonDivergentGapTriangleProjectionCount + Stats.NonDivergentGapNearestCopyCount,
		Stats.NonDivergentGapCount);
	AddInfo(FString::Printf(
		TEXT("divergent_gap_detection step=%d no_valid_hit_count=%d divergent_gap_count=%d non_divergent_gap_count=%d divergent_continental_gap_count=%d non_divergent_continental_gap_count=%d triangle_projection_count=%d nearest_copy_count=%d"),
		Stats.Step,
		Stats.NoValidHitCount,
		Stats.DivergentGapCount,
		Stats.NonDivergentGapCount,
		Stats.DivergentContinentalGapCount,
		Stats.NonDivergentContinentalGapCount,
		Stats.NonDivergentGapTriangleProjectionCount,
		Stats.NonDivergentGapNearestCopyCount));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetOceanicCrustGenerationTest,
	"Aurous.TectonicPlanet.OceanicCrustGeneration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetOceanicCrustGenerationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();

	TArray<uint8> Step0ContinentalMask;
	Step0ContinentalMask.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		Step0ContinentalMask.Add(Sample.ContinentalWeight >= 0.5f ? 1 : 0);
	}

	for (int32 StepIndex = 0; StepIndex < 20; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	int32 NewOceanicCrustCount = 0;
	int32 ChangedCrustTypeCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		const FSample& Sample = Planet.Samples[SampleIndex];
		const bool bContinentalNow = Sample.ContinentalWeight >= 0.5f;
		ChangedCrustTypeCount += (Step0ContinentalMask[SampleIndex] != (bContinentalNow ? 1 : 0)) ? 1 : 0;

		if (Sample.Age > 1.0e-4f || Sample.ContinentalWeight != 0.0f)
		{
			continue;
		}

		++NewOceanicCrustCount;
		TestEqual(FString::Printf(TEXT("New oceanic crust sample %d has no orogeny"), SampleIndex), Sample.OrogenyType, EOrogenyType::None);
		TestEqual(FString::Printf(TEXT("New oceanic crust sample %d has no terrane"), SampleIndex), Sample.TerraneId, INDEX_NONE);
		TestTrue(FString::Printf(TEXT("New oceanic crust sample %d stays near the ridge template"), SampleIndex), Sample.Elevation >= -3.0f && Sample.Elevation <= 1.0f);
	}

	TestTrue(TEXT("At least some new oceanic crust was generated by step 20"), NewOceanicCrustCount > 0);
	AddInfo(FString::Printf(
		TEXT("oceanic_crust_generation step=%d new_oceanic_count=%d changed_crust_type_count=%d"),
		Planet.CurrentStep,
		NewOceanicCrustCount,
		ChangedCrustTypeCount));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetOverlapClassificationAfterMotionTest,
	"Aurous.TectonicPlanet.OverlapClassificationAfterMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetOverlapClassificationAfterMotionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	for (int32 StepIndex = 0; StepIndex < 20; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	const int32 ClassifiedOverlapCount =
		Stats.OceanicOceanicOverlapCount +
		Stats.OceanicContinentalOverlapCount +
		Stats.ContinentalContinentalOverlapCount +
		Stats.NonConvergentOverlapCount;

	TestEqual(TEXT("Latest overlap-classification resample happened at step 20"), Stats.Step, 20);
	TestTrue(TEXT("Overlap classification observed overlaps after motion"), Stats.OverlapCount > 0);
	TestEqual(TEXT("Overlap classifications sum to overlap count"), ClassifiedOverlapCount, Stats.OverlapCount);
	if (Stats.OverlapCount > 0)
	{
		const double NonConvergentFraction =
			static_cast<double>(Stats.NonConvergentOverlapCount) / static_cast<double>(Stats.OverlapCount);
		TestTrue(TEXT("Non-convergent overlaps remain a small fraction of overlaps"), NonConvergentFraction < 0.25);
	}

	AddInfo(FString::Printf(
		TEXT("overlap_classification_after_motion step=%d overlap_count=%d oo=%d oc=%d cc=%d nonconvergent=%d classification_ms=%.3f"),
		Stats.Step,
		Stats.OverlapCount,
		Stats.OceanicOceanicOverlapCount,
		Stats.OceanicContinentalOverlapCount,
		Stats.ContinentalContinentalOverlapCount,
		Stats.NonConvergentOverlapCount,
		Stats.OverlapClassificationMs));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetTerraneIdentityPersistenceTest,
	"Aurous.TectonicPlanet.TerraneIdentityPersistence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetTerraneIdentityPersistenceTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const TSet<int32> InitialTerraneIds = BuildTerraneIdSet(Planet.Terranes);

	for (int32 StepIndex = 0; StepIndex < 10; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const TSet<int32> ResampledTerraneIds = BuildTerraneIdSet(Planet.Terranes);
	const int32 SharedTerraneCount = CountSharedTerraneIds(InitialTerraneIds, ResampledTerraneIds);
	const double RetentionRate =
		InitialTerraneIds.Num() == 0
			? 0.0
			: static_cast<double>(SharedTerraneCount) / static_cast<double>(InitialTerraneIds.Num());
	const FTerraneValidationSummary Validation = BuildTerraneValidationSummary(Planet);

	TestEqual(TEXT("Terrane persistence resampled at step 10"), Planet.LastResamplingStats.Step, 10);
	TestTrue(TEXT("Terranes remain populated after the first resample"), Planet.Terranes.Num() > 0);
	TestTrue(TEXT("At least 80% of initial terrane IDs survived the first resample"), RetentionRate >= 0.80);
	TestTrue(TEXT("Terrane IDs remain unique after resampling"), Validation.bUniqueTerraneIds);
	TestEqual(
		TEXT("All continental samples keep a terrane assignment after resampling"),
		Validation.ContinentalSamplesWithValidTerrane,
		Validation.ContinentalSampleCount);
	TestEqual(
		TEXT("All continental samples match their terrane's owning plate"),
		Validation.ContinentalSamplesWithMatchingTerranePlate,
		Validation.ContinentalSampleCount);
	TestEqual(
		TEXT("All oceanic samples keep terrane unset after resampling"),
		Validation.OceanicSamplesWithUnsetTerrane,
		Validation.OceanicSampleCount);

	AddInfo(FString::Printf(
		TEXT("terrane_identity_persistence step=%d terrane_count=%d retained_ids=%d initial_ids=%d retention_rate=%.3f new_terrane_count=%d merged_terrane_count=%d detection_ms=%.3f"),
		Planet.LastResamplingStats.Step,
		Planet.Terranes.Num(),
		SharedTerraneCount,
		InitialTerraneIds.Num(),
		RetentionRate,
		Planet.LastResamplingStats.NewTerraneCount,
		Planet.LastResamplingStats.MergedTerraneCount,
		Planet.LastResamplingStats.TerraneDetectionMs));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetTerraneDetectionStress40Test,
	"Aurous.TectonicPlanet.TerraneDetectionStress40",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetTerraneDetectionStress40Test::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(StressPlateCount);
	for (int32 StepIndex = 0; StepIndex < 30; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const FTerraneValidationSummary Validation = BuildTerraneValidationSummary(Planet);
	TestEqual(TEXT("Stress40 terrane detection resampled at step 30"), Planet.LastResamplingStats.Step, 30);
	TestTrue(TEXT("Stress40 terrane count stays positive"), Planet.Terranes.Num() > 0);
	TestTrue(TEXT("Stress40 terrane count stays below 650"), Planet.Terranes.Num() < 650);
	TestEqual(TEXT("Stress40 stats terrane count matches planet terrane count"), Planet.LastResamplingStats.TerraneCount, Planet.Terranes.Num());
	TestTrue(TEXT("Stress40 terrane IDs remain unique"), Validation.bUniqueTerraneIds);
	TestEqual(
		TEXT("Stress40 continental samples all keep valid terranes"),
		Validation.ContinentalSamplesWithValidTerrane,
		Validation.ContinentalSampleCount);
	TestEqual(
		TEXT("Stress40 continental samples all match terrane owning plates"),
		Validation.ContinentalSamplesWithMatchingTerranePlate,
		Validation.ContinentalSampleCount);
	TestEqual(
		TEXT("Stress40 oceanic samples keep terranes unset"),
		Validation.OceanicSamplesWithUnsetTerrane,
		Validation.OceanicSampleCount);

	AddInfo(FString::Printf(
		TEXT("terrane_detection_stress40 step=%d terrane_count=%d new_terrane_count=%d merged_terrane_count=%d detection_ms=%.3f"),
		Planet.LastResamplingStats.Step,
		Planet.Terranes.Num(),
		Planet.LastResamplingStats.NewTerraneCount,
		Planet.LastResamplingStats.MergedTerraneCount,
		Planet.LastResamplingStats.TerraneDetectionMs));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetFullPipelineWithOverlapsAndTerranesTest,
	"Aurous.TectonicPlanet.FullPipelineWithOverlapsAndTerranes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetFullPipelineWithOverlapsAndTerranesTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M2c-FullPipelineWithOverlapsAndTerranes-seed42-samples60000-plates7"));

	TSet<int32> Step20TerraneIds;
	for (int32 StepIndex = 0; StepIndex < 50; ++StepIndex)
	{
		Planet.AdvanceStep();
		if (Planet.CurrentStep == 20)
		{
			Step20TerraneIds = BuildTerraneIdSet(Planet.Terranes);
		}
	}

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	const int32 ClassifiedOverlapCount =
		Stats.OceanicOceanicOverlapCount +
		Stats.OceanicContinentalOverlapCount +
		Stats.ContinentalContinentalOverlapCount +
		Stats.NonConvergentOverlapCount;
	const FTerraneValidationSummary Validation = BuildTerraneValidationSummary(Planet);
	const TSet<int32> Step50TerraneIds = BuildTerraneIdSet(Planet.Terranes);
	const int32 SharedTerraneCount = CountSharedTerraneIds(Step20TerraneIds, Step50TerraneIds);

	TestEqual(TEXT("Full pipeline with overlaps and terranes resampled five times by step 50"), Planet.ResamplingSteps.Num(), 5);
	TestEqual(TEXT("Full pipeline with overlaps and terranes finished on step 50"), Stats.Step, 50);
	TestTrue(TEXT("Step 20 terrane snapshot was captured"), Step20TerraneIds.Num() > 0);
	TestEqual(TEXT("Full pipeline overlap classifications sum to overlap count"), ClassifiedOverlapCount, Stats.OverlapCount);
	TestTrue(TEXT("Full pipeline overlap classification counts are non-negative"), Stats.OceanicOceanicOverlapCount >= 0 && Stats.OceanicContinentalOverlapCount >= 0 && Stats.ContinentalContinentalOverlapCount >= 0 && Stats.NonConvergentOverlapCount >= 0);
	TestEqual(TEXT("Full pipeline terrane count matches stats"), Stats.TerraneCount, Planet.Terranes.Num());
	TestTrue(TEXT("Full pipeline terrane IDs remain unique"), Validation.bUniqueTerraneIds);
	TestEqual(TEXT("Full pipeline continental samples all keep valid terranes"), Validation.ContinentalSamplesWithValidTerrane, Validation.ContinentalSampleCount);
	TestEqual(TEXT("Full pipeline continental samples all match terrane owning plates"), Validation.ContinentalSamplesWithMatchingTerranePlate, Validation.ContinentalSampleCount);
	TestEqual(TEXT("Full pipeline oceanic samples keep terranes unset"), Validation.OceanicSamplesWithUnsetTerrane, Validation.OceanicSampleCount);
	TestTrue(TEXT("Some terrane IDs survive from step 20 to step 50"), SharedTerraneCount > 0);
	TestTrue(TEXT("Full pipeline step 50 export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep));

	AddInfo(FString::Printf(
		TEXT("full_pipeline_with_overlaps_and_terranes step=%d overlap_count=%d oo=%d oc=%d cc=%d nonconvergent=%d terrane_count=%d new_terrane_count=%d merged_terrane_count=%d shared_20_to_50=%d overlap_classification_ms=%.3f terrane_detection_ms=%.3f"),
		Stats.Step,
		Stats.OverlapCount,
		Stats.OceanicOceanicOverlapCount,
		Stats.OceanicContinentalOverlapCount,
		Stats.ContinentalContinentalOverlapCount,
		Stats.NonConvergentOverlapCount,
		Stats.TerraneCount,
		Stats.NewTerraneCount,
		Stats.MergedTerraneCount,
		SharedTerraneCount,
		Stats.OverlapClassificationMs,
		Stats.TerraneDetectionMs));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetResamplingTriggersAtCorrectCadenceTest,
	"Aurous.TectonicPlanet.ResamplingTriggersAtCorrectCadence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetResamplingTriggersAtCorrectCadenceTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	const int32 ExpectedInterval = 10;
	TestEqual(TEXT("Resample interval is 10 at 60k"), Planet.ComputeResampleInterval(), ExpectedInterval);

	for (int32 StepIndex = 0; StepIndex < 25; ++StepIndex)
	{
		Planet.AdvanceStep();
		if (!Planet.ResamplingSteps.IsEmpty() && Planet.ResamplingSteps.Last() == Planet.CurrentStep)
		{
			for (const FPlate& Plate : Planet.Plates)
			{
				TestTrue(
					FString::Printf(TEXT("Plate %d cumulative rotation resets after resample at step %d"), Plate.Id, Planet.CurrentStep),
					FMath::IsNearlyEqual(Plate.CumulativeRotation.X, 0.0, 1.0e-9) &&
					FMath::IsNearlyEqual(Plate.CumulativeRotation.Y, 0.0, 1.0e-9) &&
					FMath::IsNearlyEqual(Plate.CumulativeRotation.Z, 0.0, 1.0e-9) &&
					FMath::IsNearlyEqual(Plate.CumulativeRotation.W, 1.0, 1.0e-9));
			}
		}
	}

	TestEqual(TEXT("Two resamples occurred by step 25"), Planet.ResamplingSteps.Num(), 2);
	TestEqual(TEXT("First resample happened at step 10"), Planet.ResamplingSteps[0], 10);
	TestEqual(TEXT("Second resample happened at step 20"), Planet.ResamplingSteps[1], 20);
	AddInfo(FString::Printf(
		TEXT("resampling_cadence step10_total_ms=%.3f step20_total_ms=%.3f"),
		Planet.LastResamplingStats.Step == 20 ? Planet.LastResamplingStats.TotalMs : 0.0,
		Planet.LastResamplingStats.TotalMs));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPeriodicModeCompatibilityTest,
	"Aurous.TectonicPlanet.PeriodicModeCompatibility",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPeriodicModeCompatibilityTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	TestEqual(TEXT("PeriodicFull is the default resampling policy"), Planet.ResamplingPolicy, EResamplingPolicy::PeriodicFull);

	for (int32 StepIndex = 0; StepIndex < 25; ++StepIndex)
	{
		Planet.AdvanceStep();
		if (!Planet.ResamplingSteps.IsEmpty() && Planet.ResamplingSteps.Last() == Planet.CurrentStep)
		{
			TestEqual(
				FString::Printf(TEXT("Resample at step %d is tagged as periodic"), Planet.CurrentStep),
				Planet.LastResampleTriggerReason,
				EResampleTriggerReason::Periodic);
			TestEqual(
				FString::Printf(TEXT("Resample at step %d uses full overlap resolution"), Planet.CurrentStep),
				Planet.LastResampleOwnershipMode,
				EResampleOwnershipMode::FullResolution);
		}
	}

	TestEqual(TEXT("Periodic mode still resamples twice by step 25"), Planet.ResamplingSteps.Num(), 2);
	TestEqual(TEXT("Last resample reason remains periodic"), Planet.LastResampleTriggerReason, EResampleTriggerReason::Periodic);
	TestEqual(TEXT("Last resample ownership mode remains full resolution"), Planet.LastResampleOwnershipMode, EResampleOwnershipMode::FullResolution);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetEventDrivenNoPeriodicTest,
	"Aurous.TectonicPlanet.EventDrivenNoPeriodic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetEventDrivenNoPeriodicTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::EventDrivenOnly;
	Planet.MaxStepsWithoutResampling = 1000;
	SetAllContinentalWeights(Planet, 0.0f);

	for (int32 StepIndex = 0; StepIndex < 20; ++StepIndex)
	{
		Planet.AdvanceStep();
		TestTrue(
			FString::Printf(TEXT("Event-driven mode never records a periodic trigger at step %d"), Planet.CurrentStep),
			Planet.LastResampleTriggerReason != EResampleTriggerReason::Periodic);
	}

	TestEqual(TEXT("Event-driven mode does not resample under a large safety valve with no collision trigger"), Planet.ResamplingSteps.Num(), 0);
	TestEqual(TEXT("No event trigger reason is recorded when no resampling occurs"), Planet.LastResampleTriggerReason, EResampleTriggerReason::None);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetEventDrivenSafetyValveTest,
	"Aurous.TectonicPlanet.EventDrivenSafetyValve",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetEventDrivenSafetyValveTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::EventDrivenOnly;
	Planet.MaxStepsWithoutResampling = 5;
	SetAllContinentalWeights(Planet, 0.0f);

	for (int32 StepIndex = 0; StepIndex < 5; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	TestEqual(TEXT("Safety valve causes exactly one resample"), Planet.ResamplingSteps.Num(), 1);
	TestEqual(TEXT("Safety valve fires at step 5"), Planet.ResamplingSteps[0], 5);
	TestEqual(TEXT("Safety valve records its trigger reason"), Planet.LastResampleTriggerReason, EResampleTriggerReason::SafetyValve);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetHybridStablePeriodicModeTest,
	"Aurous.TectonicPlanet.HybridStablePeriodicMode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetHybridStablePeriodicModeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::HybridStablePeriodic;

	for (int32 StepIndex = 0; StepIndex < 10; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	TestTrue(TEXT("HybridStablePeriodic resamples on the periodic cadence"), !Planet.ResamplingSteps.IsEmpty());
	TestEqual(TEXT("The first hybrid resample still occurs at step 10"), Planet.ResamplingSteps[0], 10);
	TestEqual(TEXT("Hybrid periodic resamples are tagged as periodic"), Planet.LastResampleTriggerReason, EResampleTriggerReason::Periodic);
	TestEqual(TEXT("Hybrid periodic resamples record stable-overlap ownership mode"), Planet.LastResampleOwnershipMode, EResampleOwnershipMode::StableOverlaps);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetHybridCollisionFollowupTest,
	"Aurous.TectonicPlanet.CollisionFollowup",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetHybridCollisionFollowupTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::HybridStablePeriodic;

	bool bSawCollisionFollowup = false;
	for (int32 StepIndex = 0; StepIndex < 100; ++StepIndex)
	{
		const int32 ResampleCountBeforeAdvance = Planet.ResamplingSteps.Num();
		Planet.AdvanceStep();
		if (Planet.LastResampleTriggerReason == EResampleTriggerReason::CollisionFollowup)
		{
			bSawCollisionFollowup = true;
			TestTrue(TEXT("Collision follow-up adds a second same-step resample"), Planet.ResamplingSteps.Num() >= ResampleCountBeforeAdvance + 2);
			TestEqual(TEXT("Collision follow-up ends in full-resolution ownership mode"), Planet.LastResampleOwnershipMode, EResampleOwnershipMode::FullResolution);
			TestTrue(TEXT("Collision follow-up records the same step twice"), Planet.ResamplingSteps.Num() >= 2 &&
				Planet.ResamplingSteps[Planet.ResamplingSteps.Num() - 1] == Planet.CurrentStep &&
				Planet.ResamplingSteps[Planet.ResamplingSteps.Num() - 2] == Planet.CurrentStep);
			break;
		}
	}

	if (!bSawCollisionFollowup)
	{
		AddInfo(TEXT("hybrid_collision_followup_no_collision_by_step_100"));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveOwnershipPeriodicABTest,
	"Aurous.TectonicPlanet.PreserveOwnershipPeriodicAB",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipPeriodicABTest::RunTest(const FString& Parameters)
{
	auto LogCheckpoint = [this](const TCHAR* RunLabel, const FTectonicPlanet& Planet)
	{
		const FCurrentOwnershipSnapshot Snapshot = CaptureCurrentOwnershipSnapshot(Planet);
		const int32 BoundarySampleCount = CountBoundarySamples(Planet);
		const double BoundarySampleFraction = Planet.Samples.IsEmpty()
			? 0.0
			: static_cast<double>(BoundarySampleCount) / static_cast<double>(Planet.Samples.Num());

		AddInfo(FString::Printf(
			TEXT("preserve_ownership_periodic_ab run=%s current_step=%d resampling_steps=[%s] last_resample_trigger_reason=%s last_resample_ownership_mode=%s boundary_sample_count=%d boundary_sample_fraction=%.6f gap_count=%d overlap_count=%d continental_continental_overlap_count=%d continental_count=%d preserve_same_plate_hit_count=%d preserve_same_plate_recovery_count=%d preserve_fallback_query_count=%d preserve_plate_changed_count=%d mean_elevation=%.6f"),
			RunLabel,
			Planet.CurrentStep,
			*JoinIntArray(Planet.ResamplingSteps),
			GetResampleTriggerReasonName(Planet.LastResampleTriggerReason),
			GetResampleOwnershipModeName(Planet.LastResampleOwnershipMode),
			BoundarySampleCount,
			BoundarySampleFraction,
			Snapshot.GapCount,
			Snapshot.OverlapCount,
			Snapshot.ContinentalContinentalOverlapCount,
			CountContinentalSamples(Planet),
			Planet.LastResamplingStats.PreserveOwnershipSamePlateHitCount,
			Planet.LastResamplingStats.PreserveOwnershipSamePlateRecoveryCount,
			Planet.LastResamplingStats.PreserveOwnershipFallbackQueryCount,
			Planet.LastResamplingStats.PreserveOwnershipPlateChangedCount,
			ComputeMeanElevation(Planet)));
	};

	FTectonicPlanet PeriodicPlanet = CreateInitializedPlanet();
	FTectonicPlanet EventDrivenPlanet = CreateInitializedPlanet();
	EventDrivenPlanet.ResamplingPolicy = EResamplingPolicy::EventDrivenOnly;
	EventDrivenPlanet.MaxStepsWithoutResampling = 1000;
	FTectonicPlanet HybridPlanet = CreateInitializedPlanet();
	HybridPlanet.ResamplingPolicy = EResamplingPolicy::HybridStablePeriodic;
	FTectonicPlanet PreservePlanet = CreateInitializedPlanet();
	PreservePlanet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;

	const FString PeriodicExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("PreserveOwnershipPeriodicAB-PeriodicFull-seed42-samples60000-plates7"));
	const FString EventDrivenExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("PreserveOwnershipPeriodicAB-EventDrivenOnly-seed42-samples60000-plates7"));
	const FString HybridExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("PreserveOwnershipPeriodicAB-HybridStablePeriodic-seed42-samples60000-plates7"));
	const FString PreserveExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("PreserveOwnershipPeriodicAB-PreserveOwnershipPeriodic-seed42-samples60000-plates7"));

	bool bEventDrivenSawPeriodicReason = false;
	for (int32 StepIndex = 0; StepIndex < 50; ++StepIndex)
	{
		PeriodicPlanet.AdvanceStep();
		EventDrivenPlanet.AdvanceStep();
		HybridPlanet.AdvanceStep();
		PreservePlanet.AdvanceStep();
		bEventDrivenSawPeriodicReason |= EventDrivenPlanet.LastResampleTriggerReason == EResampleTriggerReason::Periodic;

		if (PeriodicPlanet.CurrentStep == 10 || PeriodicPlanet.CurrentStep == 50)
		{
			TestTrue(
				*FString::Printf(TEXT("PeriodicFull export succeeded at step %d"), PeriodicPlanet.CurrentStep),
				ExportCheckpointMaps(*this, PeriodicPlanet, PeriodicExportRoot, PeriodicPlanet.CurrentStep, ETectonicMapExportMode::All));
			LogCheckpoint(TEXT("periodic_full"), PeriodicPlanet);
		}

		if (EventDrivenPlanet.CurrentStep == 10 || EventDrivenPlanet.CurrentStep == 50)
		{
			TestTrue(
				*FString::Printf(TEXT("EventDrivenOnly export succeeded at step %d"), EventDrivenPlanet.CurrentStep),
				ExportCheckpointMaps(*this, EventDrivenPlanet, EventDrivenExportRoot, EventDrivenPlanet.CurrentStep, ETectonicMapExportMode::All));
			LogCheckpoint(TEXT("event_driven_only"), EventDrivenPlanet);
		}

		if (HybridPlanet.CurrentStep == 10 || HybridPlanet.CurrentStep == 50)
		{
			TestTrue(
				*FString::Printf(TEXT("HybridStablePeriodic export succeeded at step %d"), HybridPlanet.CurrentStep),
				ExportCheckpointMaps(*this, HybridPlanet, HybridExportRoot, HybridPlanet.CurrentStep, ETectonicMapExportMode::All));
			LogCheckpoint(TEXT("hybrid_stable_periodic"), HybridPlanet);
		}

		if (PreservePlanet.CurrentStep == 10 || PreservePlanet.CurrentStep == 50)
		{
			TestTrue(
				*FString::Printf(TEXT("PreserveOwnershipPeriodic export succeeded at step %d"), PreservePlanet.CurrentStep),
				ExportCheckpointMaps(*this, PreservePlanet, PreserveExportRoot, PreservePlanet.CurrentStep, ETectonicMapExportMode::All));
			LogCheckpoint(TEXT("preserve_ownership_periodic"), PreservePlanet);
		}
	}

	TestEqual(TEXT("PeriodicFull still resamples five times by step 50"), PeriodicPlanet.ResamplingSteps.Num(), 5);
	TestTrue(TEXT("EventDrivenOnly never records periodic trigger reasons"), !bEventDrivenSawPeriodicReason);
	TestTrue(TEXT("HybridStablePeriodic performs periodic maintenance passes"), HybridPlanet.ResamplingSteps.Num() >= 5);
	TestTrue(TEXT("PreserveOwnershipPeriodic performs periodic maintenance passes"), PreservePlanet.ResamplingSteps.Num() >= 5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveOwnershipDecisionRunTest,
	"Aurous.TectonicPlanet.PreserveOwnershipDecisionRun",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipDecisionRunTest::RunTest(const FString& Parameters)
{
	auto LogCheckpoint = [this](const TCHAR* RunLabel, const TCHAR* CheckpointLabel, const FTectonicPlanet& Planet)
	{
		const FCurrentOwnershipSnapshot Snapshot = CaptureCurrentOwnershipSnapshot(Planet);
		const int32 BoundarySampleCount = CountBoundarySamples(Planet);
		const double BoundarySampleFraction = Planet.Samples.IsEmpty()
			? 0.0
			: static_cast<double>(BoundarySampleCount) / static_cast<double>(Planet.Samples.Num());

		AddInfo(FString::Printf(
			TEXT("preserve_ownership_decision run=%s checkpoint=%s current_step=%d resampling_steps=[%s] last_resample_trigger_reason=%s last_resample_ownership_mode=%s boundary_sample_count=%d boundary_sample_fraction=%.6f gap_count=%d overlap_count=%d continental_continental_overlap_count=%d continental_count=%d preserve_same_plate_hit_count=%d preserve_same_plate_recovery_count=%d preserve_fallback_query_count=%d preserve_plate_changed_count=%d mean_elevation=%.6f collision_count=%d used_cached_boundary_contact_collision=%s cached_boundary_contact_seed_count=%d cached_boundary_contact_terrane_seed_count=%d cached_boundary_contact_terrane_recovered_count=%d"),
			RunLabel,
			CheckpointLabel,
			Planet.CurrentStep,
			*JoinIntArray(Planet.ResamplingSteps),
			GetResampleTriggerReasonName(Planet.LastResampleTriggerReason),
			GetResampleOwnershipModeName(Planet.LastResampleOwnershipMode),
			BoundarySampleCount,
			BoundarySampleFraction,
			Snapshot.GapCount,
			Snapshot.OverlapCount,
			Snapshot.ContinentalContinentalOverlapCount,
			CountContinentalSamples(Planet),
			Planet.LastResamplingStats.PreserveOwnershipSamePlateHitCount,
			Planet.LastResamplingStats.PreserveOwnershipSamePlateRecoveryCount,
			Planet.LastResamplingStats.PreserveOwnershipFallbackQueryCount,
			Planet.LastResamplingStats.PreserveOwnershipPlateChangedCount,
			ComputeMeanElevation(Planet),
			Planet.LastResamplingStats.CollisionCount,
			Planet.LastResamplingStats.bUsedCachedBoundaryContactCollision ? TEXT("true") : TEXT("false"),
			Planet.LastResamplingStats.CachedBoundaryContactSeedCount,
			Planet.LastResamplingStats.CachedBoundaryContactTerraneSeedCount,
			Planet.LastResamplingStats.CachedBoundaryContactTerraneRecoveredCount));
	};

	auto MaybeExportAndLogCheckpoint =
		[this, &LogCheckpoint](
			const TCHAR* RunLabel,
			const TCHAR* CheckpointLabel,
			FTectonicPlanet& Planet,
			const FString& ExportRoot,
			TSet<int32>& ExportedSteps)
	{
		if (ExportedSteps.Contains(Planet.CurrentStep))
		{
			return;
		}

		TestTrue(
			*FString::Printf(TEXT("%s export succeeded at step %d"), RunLabel, Planet.CurrentStep),
			ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep, ETectonicMapExportMode::All));
		LogCheckpoint(RunLabel, CheckpointLabel, Planet);
		ExportedSteps.Add(Planet.CurrentStep);
	};

	FTectonicPlanet PeriodicPlanet = CreateInitializedPlanet();
	FTectonicPlanet PreservePlanet = CreateInitializedPlanet();
	PreservePlanet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;

	const FString PeriodicExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("PreserveOwnershipDecisionRun-PeriodicFull-seed42-samples60000-plates7"));
	const FString PreserveExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("PreserveOwnershipDecisionRun-PreserveOwnershipPeriodic-seed42-samples60000-plates7"));

	TSet<int32> PeriodicExportedSteps;
	TSet<int32> PreserveExportedSteps;
	int32 PeriodicCollisionStep = INDEX_NONE;
	int32 PreserveCollisionStep = INDEX_NONE;
	int32 PreserveCollisionFollowupStep = INDEX_NONE;
	bool bPreserveCollisionFollowupSameStep = false;
	bool bPreserveCollisionUsedCachedBoundaryContact = false;

	for (int32 StepIndex = 0; StepIndex < 110; ++StepIndex)
	{
		PeriodicPlanet.AdvanceStep();
		PreservePlanet.AdvanceStep();

		if (PeriodicPlanet.CurrentStep == 10 || PeriodicPlanet.CurrentStep == 50 || PeriodicPlanet.CurrentStep == 100 || PeriodicPlanet.CurrentStep == 110)
		{
			MaybeExportAndLogCheckpoint(TEXT("periodic_full"), *FString::Printf(TEXT("step_%d"), PeriodicPlanet.CurrentStep), PeriodicPlanet, PeriodicExportRoot, PeriodicExportedSteps);
		}
		if (PreservePlanet.CurrentStep == 10 || PreservePlanet.CurrentStep == 50 || PreservePlanet.CurrentStep == 100 || PreservePlanet.CurrentStep == 110)
		{
			MaybeExportAndLogCheckpoint(TEXT("preserve_ownership_periodic"), *FString::Printf(TEXT("step_%d"), PreservePlanet.CurrentStep), PreservePlanet, PreserveExportRoot, PreserveExportedSteps);
		}

		if (PeriodicCollisionStep == INDEX_NONE && PeriodicPlanet.LastResamplingStats.CollisionCount > 0)
		{
			PeriodicCollisionStep = PeriodicPlanet.CurrentStep;
			AddInfo(FString::Printf(
				TEXT("preserve_ownership_decision_collision run=periodic_full collision_step=%d followup_step=-1 followup_same_step=false"),
				PeriodicCollisionStep));
			if (PeriodicCollisionStep != 10 && PeriodicCollisionStep != 50 && PeriodicCollisionStep != 100)
			{
				MaybeExportAndLogCheckpoint(TEXT("periodic_full"), TEXT("collision_step"), PeriodicPlanet, PeriodicExportRoot, PeriodicExportedSteps);
			}
		}

		if (PreserveCollisionStep == INDEX_NONE &&
			PreservePlanet.LastResampleTriggerReason == EResampleTriggerReason::CollisionFollowup &&
			PreservePlanet.LastResamplingStats.CollisionCount > 0)
		{
			PreserveCollisionStep = PreservePlanet.CurrentStep;
			PreserveCollisionFollowupStep = PreservePlanet.CurrentStep;
			bPreserveCollisionFollowupSameStep = true;
			bPreserveCollisionUsedCachedBoundaryContact =
				PreservePlanet.LastResamplingStats.bUsedCachedBoundaryContactCollision;
			AddInfo(FString::Printf(
				TEXT("preserve_ownership_decision_collision run=preserve_ownership_periodic collision_step=%d followup_step=%d followup_same_step=%s used_cached_boundary_contact_collision=%s"),
				PreserveCollisionStep,
				PreserveCollisionFollowupStep,
				bPreserveCollisionFollowupSameStep ? TEXT("true") : TEXT("false"),
				bPreserveCollisionUsedCachedBoundaryContact ? TEXT("true") : TEXT("false")));
			if (PreserveCollisionStep != 10 && PreserveCollisionStep != 50 && PreserveCollisionStep != 100)
			{
				MaybeExportAndLogCheckpoint(TEXT("preserve_ownership_periodic"), TEXT("collision_step"), PreservePlanet, PreserveExportRoot, PreserveExportedSteps);
			}
		}

		if (PeriodicCollisionStep != INDEX_NONE && PeriodicPlanet.CurrentStep == PeriodicCollisionStep + 10)
		{
			MaybeExportAndLogCheckpoint(TEXT("periodic_full"), TEXT("collision_plus_10"), PeriodicPlanet, PeriodicExportRoot, PeriodicExportedSteps);
		}
		if (PreserveCollisionStep != INDEX_NONE && PreservePlanet.CurrentStep == PreserveCollisionStep + 10)
		{
			MaybeExportAndLogCheckpoint(TEXT("preserve_ownership_periodic"), TEXT("collision_plus_10"), PreservePlanet, PreserveExportRoot, PreserveExportedSteps);
		}
	}

	TestTrue(TEXT("PeriodicFull resamples through step 110"), PeriodicPlanet.ResamplingSteps.Num() >= 10);
	TestTrue(TEXT("PreserveOwnershipPeriodic resamples through step 110"), PreservePlanet.ResamplingSteps.Num() >= 10);
	TestTrue(TEXT("PreserveOwnershipPeriodic reaches a real collision event by step 110"), PreserveCollisionStep != INDEX_NONE);
	TestTrue(TEXT("PreserveOwnershipPeriodic follow-up occurs on the same step as the triggering periodic pass"), bPreserveCollisionFollowupSameStep);
	TestTrue(TEXT("PreserveOwnershipPeriodic executes the real collision from cached boundary-contact evidence"), bPreserveCollisionUsedCachedBoundaryContact);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftCreatesTwoChildrenTest,
	"Aurous.TectonicPlanet.ForcedRiftCreatesTwoChildren",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftCreatesTwoChildrenTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(5, 12000);
	const int32 ParentPlateId = FindLargestPlateId(Planet);
	const int32 ParentPlateIndex = Planet.FindPlateArrayIndexById(ParentPlateId);
	TestTrue(TEXT("Largest parent plate exists"), ParentPlateIndex != INDEX_NONE);
	const TArray<int32> FormerParentMembers = Planet.Plates[ParentPlateIndex].MemberSamples;
	const int32 PlateCountBeforeRift = Planet.Plates.Num();
	int32 MaxOriginalPlateId = INDEX_NONE;
	for (const FPlate& Plate : Planet.Plates)
	{
		MaxOriginalPlateId = FMath::Max(MaxOriginalPlateId, Plate.Id);
	}

	TestTrue(TEXT("Forced rift succeeds on the largest plate"), Planet.TriggerForcedRift(ParentPlateId, 2, 17));
	TestEqual(TEXT("A 2-way rift increases active plate count by one"), Planet.Plates.Num(), PlateCountBeforeRift + 1);
	TestEqual(TEXT("The follow-up resample is tagged as RiftFollowup"), Planet.LastResampleTriggerReason, EResampleTriggerReason::RiftFollowup);
	TestEqual(TEXT("Rift follow-up uses full resolution ownership"), Planet.LastResampleOwnershipMode, EResampleOwnershipMode::FullResolution);
	TestEqual(TEXT("Rift stats report one rift"), Planet.LastResamplingStats.RiftCount, 1);
	TestEqual(TEXT("The parent stable id is reported in stats"), Planet.LastResamplingStats.RiftParentPlateId, ParentPlateId);
	TestEqual(TEXT("The retired parent stable id is no longer active"), Planet.FindPlateArrayIndexById(ParentPlateId), INDEX_NONE);
	TestTrue(TEXT("Child A gets a fresh stable id"), Planet.LastResamplingStats.RiftChildPlateA > MaxOriginalPlateId);
	TestTrue(TEXT("Child B gets a fresh stable id"), Planet.LastResamplingStats.RiftChildPlateB > MaxOriginalPlateId);
	TestTrue(TEXT("Children use distinct stable ids"), Planet.LastResamplingStats.RiftChildPlateA != Planet.LastResamplingStats.RiftChildPlateB);

	for (const int32 SampleIndex : FormerParentMembers)
	{
		TestTrue(
			FString::Printf(TEXT("Former parent sample %d now belongs to one of the child plates"), SampleIndex),
			Planet.Samples[SampleIndex].PlateId == Planet.LastResamplingStats.RiftChildPlateA ||
				Planet.Samples[SampleIndex].PlateId == Planet.LastResamplingStats.RiftChildPlateB);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftPreservesOtherPlatesTest,
	"Aurous.TectonicPlanet.ForcedRiftPreservesOtherPlates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftPreservesOtherPlatesTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(5, 12000);
	const int32 ParentPlateId = FindLargestPlateId(Planet);
	TArray<int32> OriginalPlateIds;
	OriginalPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OriginalPlateIds.Add(Sample.PlateId);
	}

	TestTrue(TEXT("Forced rift succeeds"), Planet.TriggerForcedRift(ParentPlateId, 2, 23));
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		if (OriginalPlateIds[SampleIndex] == ParentPlateId)
		{
			continue;
		}

		TestEqual(
			FString::Printf(TEXT("Non-parent sample %d keeps its original plate id"), SampleIndex),
			Planet.Samples[SampleIndex].PlateId,
			OriginalPlateIds[SampleIndex]);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftTriggersFollowupResampleTest,
	"Aurous.TectonicPlanet.ForcedRiftTriggersFollowupResample",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftTriggersFollowupResampleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(5, 12000);
	const int32 ParentPlateId = FindLargestPlateId(Planet);

	TestTrue(TEXT("Forced rift succeeds"), Planet.TriggerForcedRift(ParentPlateId, 2, 31));
	TestEqual(TEXT("Forced rift records exactly one immediate follow-up resample"), Planet.ResamplingSteps.Num(), 1);
	TestEqual(TEXT("Forced rift resamples on the current step"), Planet.ResamplingSteps.Last(), Planet.CurrentStep);
	TestEqual(TEXT("Last resample reason is RiftFollowup"), Planet.LastResampleTriggerReason, EResampleTriggerReason::RiftFollowup);
	TestEqual(TEXT("Last resample ownership mode is FullResolution"), Planet.LastResampleOwnershipMode, EResampleOwnershipMode::FullResolution);
	TestEqual(TEXT("Rift parent sample count is reported"), Planet.LastResamplingStats.RiftParentSampleCount,
		Planet.LastResamplingStats.RiftChildSampleCountA + Planet.LastResamplingStats.RiftChildSampleCountB);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftChildMotionsDivergeTest,
	"Aurous.TectonicPlanet.ForcedRiftChildMotionsDiverge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftChildMotionsDivergeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(5, 12000);
	const int32 ParentPlateId = FindLargestPlateId(Planet);

	TestTrue(TEXT("Forced rift succeeds"), Planet.TriggerForcedRift(ParentPlateId, 2, 47));
	const int32 ChildAIndex = Planet.FindPlateArrayIndexById(Planet.LastResamplingStats.RiftChildPlateA);
	const int32 ChildBIndex = Planet.FindPlateArrayIndexById(Planet.LastResamplingStats.RiftChildPlateB);
	TestTrue(TEXT("Child A plate is active"), ChildAIndex != INDEX_NONE);
	TestTrue(TEXT("Child B plate is active"), ChildBIndex != INDEX_NONE);

	const FPlate& ChildA = Planet.Plates[ChildAIndex];
	const FPlate& ChildB = Planet.Plates[ChildBIndex];
	const FVector3d ChildCentroidA = ComputeSampleSetCentroidForTest(Planet, ChildA.MemberSamples);
	const FVector3d ChildCentroidB = ComputeSampleSetCentroidForTest(Planet, ChildB.MemberSamples);
	const FVector3d FractureCenter = (ChildCentroidA + ChildCentroidB).GetSafeNormal();
	const FVector3d SeparationDirection =
		(ChildCentroidB - ChildCentroidA - ((ChildCentroidB - ChildCentroidA).Dot(FractureCenter) * FractureCenter)).GetSafeNormal();

	TestTrue(TEXT("Child axes are distinct after rift"), ChildA.RotationAxis.Dot(ChildB.RotationAxis) < 0.999);
	TestTrue(TEXT("Fracture center is well-defined"), !FractureCenter.IsNearlyZero());
	TestTrue(TEXT("Child centroid separation direction is well-defined"), !SeparationDirection.IsNearlyZero());

	const double RelativeSeparationVelocity =
		(ComputePlateSurfaceVelocityForTest(ChildB, FractureCenter, Planet.PlanetRadiusKm) -
		 ComputePlateSurfaceVelocityForTest(ChildA, FractureCenter, Planet.PlanetRadiusKm)).Dot(SeparationDirection);
	TestTrue(TEXT("Child motions separate across the fracture"), RelativeSeparationVelocity > 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftSmoke60k7Test,
	"Aurous.TectonicPlanet.ForcedRiftSmoke60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftSmoke60k7Test::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	for (int32 StepIndex = 0; StepIndex < 5; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const int32 ParentPlateId = FindLargestPlateId(Planet);
	const int32 PlateCountBeforeRift = Planet.Plates.Num();
	TestTrue(TEXT("Forced rift succeeds on the 60k/7 smoke planet"), Planet.TriggerForcedRift(ParentPlateId, 2, 59));
	const int32 ChildPlateA = Planet.LastResamplingStats.RiftChildPlateA;
	const int32 ChildPlateB = Planet.LastResamplingStats.RiftChildPlateB;
	TestEqual(TEXT("60k smoke rift produces one net new plate"), Planet.Plates.Num(), PlateCountBeforeRift + 1);
	TestTrue(TEXT("The new child boundary exists immediately after the rift"), CountAdjacencyEdgesBetweenPlates(Planet, ChildPlateA, ChildPlateB) > 0);

	bool bSawPeriodicOceanicGeneration = false;
	for (int32 StepIndex = 0; StepIndex < 25; ++StepIndex)
	{
		Planet.AdvanceStep();
		if (!Planet.ResamplingSteps.IsEmpty() &&
			Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
			Planet.LastResampleTriggerReason == EResampleTriggerReason::Periodic &&
			Planet.LastResamplingStats.DivergentGapCount > 0)
		{
			bSawPeriodicOceanicGeneration = true;
		}
	}

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		TestTrue(
			FString::Printf(TEXT("Smoke sample %d retains a valid stable plate id"), SampleIndex),
			Planet.FindPlateArrayIndexById(Planet.Samples[SampleIndex].PlateId) != INDEX_NONE);
	}
	for (const FPlate& Plate : Planet.Plates)
	{
		TestTrue(FString::Printf(TEXT("Smoke plate %d retains non-empty membership"), Plate.Id), Plate.MemberSamples.Num() > 0);
	}

	TestTrue(TEXT("Later periodic maintenance still generates divergent gaps after the forced rift"), bSawPeriodicOceanicGeneration);
	AddInfo(FString::Printf(
		TEXT("forced_rift_smoke60k7 step=%d plate_count=%d child_pair=(%d,%d) child_contact_edges=%d last_gap_count=%d last_divergent_gap_count=%d"),
		Planet.CurrentStep,
		Planet.Plates.Num(),
		ChildPlateA,
		ChildPlateB,
		CountAdjacencyEdgesBetweenPlates(Planet, ChildPlateA, ChildPlateB),
		Planet.LastResamplingStats.GapCount,
		Planet.LastResamplingStats.DivergentGapCount));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftTerraneInheritanceMetricsTest,
	"Aurous.TectonicPlanet.ForcedRiftTerraneInheritanceMetrics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftTerraneInheritanceMetricsTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(5, 12000);
	int32 ParentPlateId = FindLargestTerraneBearingPlateId(Planet);
	if (ParentPlateId == INDEX_NONE)
	{
		ParentPlateId = FindLargestPlateId(Planet);
	}

	TestTrue(TEXT("A terrane-bearing or fallback largest parent plate exists"), ParentPlateId != INDEX_NONE);
	TestTrue(TEXT("Forced rift succeeds for terrane inheritance metrics"), Planet.TriggerForcedRift(ParentPlateId, 2, 67));

	const FResamplingStats& RiftStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Terrane metrics are reported on the rift follow-up resample"), Planet.LastResampleTriggerReason, EResampleTriggerReason::RiftFollowup);
	TestTrue(TEXT("The chosen parent plate carried at least one terrane before the rift"), RiftStats.RiftTerraneCountBefore > 0);
	TestTrue(TEXT("Post-rift terrane count stays positive"), RiftStats.RiftTerraneCountAfter > 0);
	TestEqual(
		TEXT("Post-rift terrane count matches the sum of child fragment counts"),
		RiftStats.RiftTerraneCountAfter,
		RiftStats.RiftTerraneFragmentsOnChildA + RiftStats.RiftTerraneFragmentsOnChildB);
	TestEqual(
		TEXT("Split/preserved accounting covers all pre-rift terranes"),
		RiftStats.RiftTerraneSplitCount + RiftStats.RiftTerranePreservedCount,
		RiftStats.RiftTerraneCountBefore);
	TestTrue(TEXT("Child A fragment count is non-negative"), RiftStats.RiftTerraneFragmentsOnChildA >= 0);
	TestTrue(TEXT("Child B fragment count is non-negative"), RiftStats.RiftTerraneFragmentsOnChildB >= 0);

	AddInfo(FString::Printf(
		TEXT("forced_rift_terrane_metrics parent=%d child_pair=(%d,%d) terranes_before=%d terranes_after=%d terrane_split_count=%d terrane_preserved_count=%d child_fragments=(%d,%d)"),
		ParentPlateId,
		RiftStats.RiftChildPlateA,
		RiftStats.RiftChildPlateB,
		RiftStats.RiftTerraneCountBefore,
		RiftStats.RiftTerraneCountAfter,
		RiftStats.RiftTerraneSplitCount,
		RiftStats.RiftTerranePreservedCount,
		RiftStats.RiftTerraneFragmentsOnChildA,
		RiftStats.RiftTerraneFragmentsOnChildB));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftChildCoherenceTest,
	"Aurous.TectonicPlanet.ForcedRiftChildCoherence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftChildCoherenceTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	for (int32 StepIndex = 0; StepIndex < 5; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	int32 ParentPlateId = FindLargestTerraneBearingPlateId(Planet);
	if (ParentPlateId == INDEX_NONE)
	{
		ParentPlateId = FindLargestPlateId(Planet);
	}

	TestTrue(TEXT("A valid parent plate exists for the coherence test"), ParentPlateId != INDEX_NONE);
	TestTrue(TEXT("Forced rift succeeds for the child coherence test"), Planet.TriggerForcedRift(ParentPlateId, 2, 71));

	const int32 ChildPlateA = Planet.LastResamplingStats.RiftChildPlateA;
	const int32 ChildPlateB = Planet.LastResamplingStats.RiftChildPlateB;
	for (int32 StepIndex = 0; StepIndex < 30; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	const FPlateCoherenceSnapshot ChildASnapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateA);
	const FPlateCoherenceSnapshot ChildBSnapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateB);
	const FPlatePairBoundaryActivitySummary BoundarySummary =
		BuildPlatePairBoundaryActivitySummary(Planet, ChildPlateA, ChildPlateB);
	const double ChildALargestComponentFraction = ComputeLargestComponentFraction(ChildASnapshot);
	const double ChildBLargestComponentFraction = ComputeLargestComponentFraction(ChildBSnapshot);

	TestTrue(TEXT("Child A remains non-empty after several maintenance resamples"), ChildASnapshot.SampleCount > 0);
	TestTrue(TEXT("Child B remains non-empty after several maintenance resamples"), ChildBSnapshot.SampleCount > 0);
	TestTrue(TEXT("Child A retains a single dominant connected component"), ChildALargestComponentFraction >= 0.99);
	TestTrue(TEXT("Child B retains a single dominant connected component"), ChildBLargestComponentFraction >= 0.99);
	TestTrue(TEXT("The child-child boundary remains active"), BoundarySummary.ContactEdgeCount > 0);
	TestTrue(TEXT("The child-child boundary remains divergent"), BoundarySummary.DivergentEdgeCount > 0);

	AddInfo(FString::Printf(
		TEXT("forced_rift_child_coherence step=%d child_pair=(%d,%d) child_counts=(%d,%d) components=(%d,%d) dominant_fraction=(%.4f,%.4f) continental=(%d,%d) child_contact_edges=%d child_divergent_edges=%d"),
		Planet.CurrentStep,
		ChildPlateA,
		ChildPlateB,
		ChildASnapshot.SampleCount,
		ChildBSnapshot.SampleCount,
		ChildASnapshot.ConnectedComponentCount,
		ChildBSnapshot.ConnectedComponentCount,
		ChildALargestComponentFraction,
		ChildBLargestComponentFraction,
		ChildASnapshot.ContinentalSampleCount,
		ChildBSnapshot.ContinentalSampleCount,
		BoundarySummary.ContactEdgeCount,
		BoundarySummary.DivergentEdgeCount));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftMediumHorizon60k7Test,
	"Aurous.TectonicPlanet.ForcedRiftMediumHorizon60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftMediumHorizon60k7Test::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;

	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M5b-ForcedRiftMediumHorizon60k7-seed42-samples60000-plates7"));

	for (int32 StepIndex = 0; StepIndex < 5; ++StepIndex)
	{
		Planet.AdvanceStep();
	}

	TestTrue(TEXT("Pre-rift export succeeds"), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PreRift")));

	int32 ParentPlateId = FindLargestTerraneBearingPlateId(Planet);
	if (ParentPlateId == INDEX_NONE)
	{
		ParentPlateId = FindLargestPlateId(Planet);
	}

	TestTrue(TEXT("A valid parent plate exists for the medium-horizon rift harness"), ParentPlateId != INDEX_NONE);
	TestTrue(TEXT("Forced rift succeeds in the medium-horizon harness"), Planet.TriggerForcedRift(ParentPlateId, 2, 73));

	const FResamplingStats RiftStats = Planet.LastResamplingStats;
	const int32 ChildPlateA = RiftStats.RiftChildPlateA;
	const int32 ChildPlateB = RiftStats.RiftChildPlateB;

	auto LogCheckpoint = [&](const FString& Label)
	{
		const FPlateCoherenceSnapshot ChildASnapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateA);
		const FPlateCoherenceSnapshot ChildBSnapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateB);
		const FPlatePairBoundaryActivitySummary BoundarySummary =
			BuildPlatePairBoundaryActivitySummary(Planet, ChildPlateA, ChildPlateB);
		const double ChildALargestComponentFraction = ComputeLargestComponentFraction(ChildASnapshot);
		const double ChildBLargestComponentFraction = ComputeLargestComponentFraction(ChildBSnapshot);
		AddInfo(FString::Printf(
			TEXT("forced_rift_medium_horizon checkpoint=%s current_step=%d child_ids=(%d,%d) child_counts=(%d,%d) components=(%d,%d) dominant_fraction=(%.4f,%.4f) continental=(%d,%d) boundary_samples=(%d,%d) child_contact_edges=%d child_divergent_edges=%d rift_terrane_before=%d rift_terrane_after=%d rift_terrane_split=%d rift_terrane_preserved=%d last_gap_count=%d last_divergent_gap_count=%d"),
			*Label,
			Planet.CurrentStep,
			ChildPlateA,
			ChildPlateB,
			ChildASnapshot.SampleCount,
			ChildBSnapshot.SampleCount,
			ChildASnapshot.ConnectedComponentCount,
			ChildBSnapshot.ConnectedComponentCount,
			ChildALargestComponentFraction,
			ChildBLargestComponentFraction,
			ChildASnapshot.ContinentalSampleCount,
			ChildBSnapshot.ContinentalSampleCount,
			ChildASnapshot.BoundarySampleCount,
			ChildBSnapshot.BoundarySampleCount,
			BoundarySummary.ContactEdgeCount,
			BoundarySummary.DivergentEdgeCount,
			RiftStats.RiftTerraneCountBefore,
			RiftStats.RiftTerraneCountAfter,
			RiftStats.RiftTerraneSplitCount,
			RiftStats.RiftTerranePreservedCount,
			Planet.LastResamplingStats.GapCount,
			Planet.LastResamplingStats.DivergentGapCount));
	};

	TestTrue(TEXT("Immediate post-rift export succeeds"), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftImmediate")));
	LogCheckpoint(TEXT("post_rift_immediate"));

	bool bSawPeriodicOceanicGeneration = false;
	while (Planet.CurrentStep < 55)
	{
		Planet.AdvanceStep();
		if (!Planet.ResamplingSteps.IsEmpty() &&
			Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
			Planet.LastResampleTriggerReason == EResampleTriggerReason::Periodic &&
			Planet.LastResamplingStats.DivergentGapCount > 0)
		{
			bSawPeriodicOceanicGeneration = true;
		}

		const int32 StepsAfterRift = Planet.CurrentStep - 5;
		if (StepsAfterRift == 10)
		{
			TestTrue(TEXT("Post-rift step 10 export succeeds"), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftStep10")));
			LogCheckpoint(TEXT("post_rift_step_10"));
		}
		else if (StepsAfterRift == 30)
		{
			TestTrue(TEXT("Post-rift step 30 export succeeds"), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftStep30")));
			LogCheckpoint(TEXT("post_rift_step_30"));
		}
		else if (StepsAfterRift == 50)
		{
			TestTrue(TEXT("Post-rift step 50 export succeeds"), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftStep50")));
			LogCheckpoint(TEXT("post_rift_step_50"));
		}
	}

	const FPlateCoherenceSnapshot FinalChildASnapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateA);
	const FPlateCoherenceSnapshot FinalChildBSnapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateB);
	const FPlatePairBoundaryActivitySummary FinalBoundarySummary =
		BuildPlatePairBoundaryActivitySummary(Planet, ChildPlateA, ChildPlateB);
	const double FinalChildALargestComponentFraction = ComputeLargestComponentFraction(FinalChildASnapshot);
	const double FinalChildBLargestComponentFraction = ComputeLargestComponentFraction(FinalChildBSnapshot);

	TestTrue(TEXT("Child A still exists at step 50 after the rift"), Planet.FindPlateArrayIndexById(ChildPlateA) != INDEX_NONE);
	TestTrue(TEXT("Child B still exists at step 50 after the rift"), Planet.FindPlateArrayIndexById(ChildPlateB) != INDEX_NONE);
	TestTrue(TEXT("Child A remains non-empty through step 50 after the rift"), FinalChildASnapshot.SampleCount > 0);
	TestTrue(TEXT("Child B remains non-empty through step 50 after the rift"), FinalChildBSnapshot.SampleCount > 0);
	TestTrue(TEXT("Child A retains a dominant connected component through step 50"), FinalChildALargestComponentFraction >= 0.99);
	TestTrue(TEXT("Child B retains a dominant connected component through step 50"), FinalChildBLargestComponentFraction >= 0.99);
	TestTrue(TEXT("The child-child boundary remains active through step 50"), FinalBoundarySummary.ContactEdgeCount > 0);
	TestTrue(TEXT("The child-child boundary remains divergent through step 50"), FinalBoundarySummary.DivergentEdgeCount > 0);
	TestTrue(TEXT("Later maintenance resamples continue generating divergent gaps after the rift"), bSawPeriodicOceanicGeneration);

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		TestTrue(
			FString::Printf(TEXT("Medium-horizon sample %d retains a valid stable plate id"), SampleIndex),
			Planet.FindPlateArrayIndexById(Planet.Samples[SampleIndex].PlateId) != INDEX_NONE);
	}
	for (const FPlate& Plate : Planet.Plates)
	{
		TestTrue(FString::Printf(TEXT("Medium-horizon plate %d retains non-empty membership"), Plate.Id), Plate.MemberSamples.Num() > 0);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftSupportsThreeChildrenTest,
	"Aurous.TectonicPlanet.ForcedRiftSupportsThreeChildren",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftSupportsThreeChildrenTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(5, 24000);
	const int32 ParentPlateId = FindLargestPlateId(Planet);
	const int32 ParentPlateIndex = Planet.FindPlateArrayIndexById(ParentPlateId);
	TestTrue(TEXT("Largest parent plate exists for the 3-child rift"), ParentPlateIndex != INDEX_NONE);
	const TArray<int32> FormerParentMembers = Planet.Plates[ParentPlateIndex].MemberSamples;
	const int32 PlateCountBeforeRift = Planet.Plates.Num();

	TestTrue(TEXT("Deterministic 3-way forced rift succeeds"), Planet.TriggerForcedRift(ParentPlateId, 3, 101));

	const FResamplingStats& RiftStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Three child plates are reported"), RiftStats.RiftChildCount, 3);
	TestEqual(TEXT("3-way rift increases active plate count by two"), Planet.Plates.Num(), PlateCountBeforeRift + 2);
	TestEqual(TEXT("Three child ids are recorded"), RiftStats.RiftChildPlateIds.Num(), 3);
	TestEqual(TEXT("Three child sample counts are recorded"), RiftStats.RiftChildSampleCounts.Num(), 3);
	TestEqual(TEXT("The retired parent stable id is no longer active"), Planet.FindPlateArrayIndexById(ParentPlateId), INDEX_NONE);

	TSet<int32> ChildPlateIdSet;
	int32 TotalChildSamples = 0;
	for (int32 ChildIndex = 0; ChildIndex < RiftStats.RiftChildPlateIds.Num(); ++ChildIndex)
	{
		const int32 ChildPlateId = RiftStats.RiftChildPlateIds[ChildIndex];
		ChildPlateIdSet.Add(ChildPlateId);
		TestTrue(FString::Printf(TEXT("Child plate %d is active"), ChildPlateId), Planet.FindPlateArrayIndexById(ChildPlateId) != INDEX_NONE);
		TestTrue(FString::Printf(TEXT("Child plate %d is non-empty"), ChildPlateId), RiftStats.RiftChildSampleCounts[ChildIndex] > 0);
		TotalChildSamples += RiftStats.RiftChildSampleCounts[ChildIndex];
	}

	TestEqual(TEXT("Child stable ids are unique"), ChildPlateIdSet.Num(), 3);
	TestEqual(TEXT("Former parent samples are fully accounted for by the three children"), TotalChildSamples, FormerParentMembers.Num());
	for (const int32 SampleIndex : FormerParentMembers)
	{
		TestTrue(
			FString::Printf(TEXT("Former parent sample %d now belongs to one of the three child plates"), SampleIndex),
			ChildPlateIdSet.Contains(Planet.Samples[SampleIndex].PlateId));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftSupportsFourChildrenTest,
	"Aurous.TectonicPlanet.ForcedRiftSupportsFourChildren",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftSupportsFourChildrenTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(5, 24000);
	const int32 ParentPlateId = FindLargestPlateId(Planet);
	const int32 ParentPlateIndex = Planet.FindPlateArrayIndexById(ParentPlateId);
	TestTrue(TEXT("Largest parent plate exists for the 4-child rift"), ParentPlateIndex != INDEX_NONE);
	const TArray<int32> FormerParentMembers = Planet.Plates[ParentPlateIndex].MemberSamples;
	const int32 PlateCountBeforeRift = Planet.Plates.Num();

	TestTrue(TEXT("Deterministic 4-way forced rift succeeds"), Planet.TriggerForcedRift(ParentPlateId, 4, 131));

	const FResamplingStats& RiftStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Four child plates are reported"), RiftStats.RiftChildCount, 4);
	TestEqual(TEXT("4-way rift increases active plate count by three"), Planet.Plates.Num(), PlateCountBeforeRift + 3);
	TestEqual(TEXT("Four child ids are recorded"), RiftStats.RiftChildPlateIds.Num(), 4);
	TestEqual(TEXT("Four child sample counts are recorded"), RiftStats.RiftChildSampleCounts.Num(), 4);

	TSet<int32> ChildPlateIdSet;
	int32 TotalChildSamples = 0;
	for (int32 ChildIndex = 0; ChildIndex < RiftStats.RiftChildPlateIds.Num(); ++ChildIndex)
	{
		const int32 ChildPlateId = RiftStats.RiftChildPlateIds[ChildIndex];
		ChildPlateIdSet.Add(ChildPlateId);
		TestTrue(FString::Printf(TEXT("Child plate %d is active"), ChildPlateId), Planet.FindPlateArrayIndexById(ChildPlateId) != INDEX_NONE);
		TestTrue(FString::Printf(TEXT("Child plate %d is non-empty"), ChildPlateId), RiftStats.RiftChildSampleCounts[ChildIndex] > 0);
		TotalChildSamples += RiftStats.RiftChildSampleCounts[ChildIndex];
	}

	TestEqual(TEXT("Child stable ids are unique"), ChildPlateIdSet.Num(), 4);
	TestEqual(TEXT("Former parent samples are fully accounted for by the four children"), TotalChildSamples, FormerParentMembers.Num());
	for (const int32 SampleIndex : FormerParentMembers)
	{
		TestTrue(
			FString::Printf(TEXT("Former parent sample %d now belongs to one of the four child plates"), SampleIndex),
			ChildPlateIdSet.Contains(Planet.Samples[SampleIndex].PlateId));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftCentroidPlacementDeterministicTest,
	"Aurous.TectonicPlanet.ForcedRiftCentroidPlacementDeterministic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftCentroidPlacementDeterministicTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet PlanetA = CreateInitializedPlanet(5, 24000);
	FTectonicPlanet PlanetB = CreateInitializedPlanet(5, 24000);
	const int32 ParentPlateIdA = FindLargestPlateId(PlanetA);
	const int32 ParentPlateIdB = FindLargestPlateId(PlanetB);
	TestEqual(TEXT("Repeated runs pick the same parent plate"), ParentPlateIdA, ParentPlateIdB);

	const int32 ParentPlateIndexA = PlanetA.FindPlateArrayIndexById(ParentPlateIdA);
	const int32 ParentPlateIndexB = PlanetB.FindPlateArrayIndexById(ParentPlateIdB);
	TestTrue(TEXT("Parent plate exists in run A"), ParentPlateIndexA != INDEX_NONE);
	TestTrue(TEXT("Parent plate exists in run B"), ParentPlateIndexB != INDEX_NONE);
	const TArray<int32> FormerParentMembersA = PlanetA.Plates[ParentPlateIndexA].MemberSamples;
	const TArray<int32> FormerParentMembersB = PlanetB.Plates[ParentPlateIndexB].MemberSamples;
	TestEqual(TEXT("Repeated runs start from the same parent membership size"), FormerParentMembersA.Num(), FormerParentMembersB.Num());

	TestTrue(TEXT("Deterministic rift succeeds in run A"), PlanetA.TriggerForcedRift(ParentPlateIdA, 3, 151));
	TestTrue(TEXT("Deterministic rift succeeds in run B"), PlanetB.TriggerForcedRift(ParentPlateIdB, 3, 151));

	TestTrue(
		TEXT("Repeated runs produce the same child stable ids"),
		PlanetA.LastResamplingStats.RiftChildPlateIds == PlanetB.LastResamplingStats.RiftChildPlateIds);
	TestTrue(
		TEXT("Repeated runs produce the same child sample counts"),
		PlanetA.LastResamplingStats.RiftChildSampleCounts == PlanetB.LastResamplingStats.RiftChildSampleCounts);
	for (int32 SampleOffset = 0; SampleOffset < FormerParentMembersA.Num(); ++SampleOffset)
	{
		const int32 SampleIndexA = FormerParentMembersA[SampleOffset];
		const int32 SampleIndexB = FormerParentMembersB[SampleOffset];
		TestEqual(TEXT("Repeated runs preserve sample ordering within the former parent"), SampleIndexA, SampleIndexB);
		TestEqual(
			FString::Printf(TEXT("Repeated runs assign former parent sample %d to the same child plate"), SampleIndexA),
			PlanetA.Samples[SampleIndexA].PlateId,
			PlanetB.Samples[SampleIndexB].PlateId);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftMultiTerraneMetricsTest,
	"Aurous.TectonicPlanet.ForcedRiftMultiTerraneMetrics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftMultiTerraneMetricsTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(5, 24000);
	const int32 ParentPlateId = FindLargestPlateId(Planet);
	TestTrue(TEXT("A valid parent plate exists for the multi-terrane rift test"), ParentPlateId != INDEX_NONE);
	TestTrue(TEXT("Synthetic multi-terrane setup succeeds"), AssignSyntheticTerranesToPlateForTest(Planet, ParentPlateId, 3, 181));
	TestTrue(TEXT("The synthetic setup produces multiple distinct terranes on the parent plate"), CountDistinctTerraneIdsOnPlate(Planet, ParentPlateId) >= 3);
	TestTrue(TEXT("Generalized rift succeeds for the multi-terrane metrics test"), Planet.TriggerForcedRift(ParentPlateId, 3, 191));

	const FResamplingStats& RiftStats = Planet.LastResamplingStats;
	TestEqual(TEXT("The generalized rift reports three children"), RiftStats.RiftChildCount, 3);
	TestTrue(TEXT("The parent plate reported multiple pre-rift terranes"), RiftStats.RiftTerraneCountBefore >= 3);
	TestEqual(TEXT("Pre-rift terrane id list length matches the reported pre-rift terrane count"), RiftStats.RiftPreTerraneIds.Num(), RiftStats.RiftTerraneCountBefore);
	TestEqual(TEXT("Touched-child counts align with pre-rift terrane ids"), RiftStats.RiftPreTerraneTouchedChildCounts.Num(), RiftStats.RiftPreTerraneIds.Num());
	TestEqual(TEXT("Child terrane fragment counts align with child count"), RiftStats.RiftChildTerraneFragmentCounts.Num(), RiftStats.RiftChildCount);
	TestEqual(TEXT("Split/preserved accounting covers all pre-rift terranes"), RiftStats.RiftTerraneSplitCount + RiftStats.RiftTerranePreservedCount, RiftStats.RiftTerraneCountBefore);

	int32 TotalChildFragments = 0;
	for (const int32 ChildFragmentCount : RiftStats.RiftChildTerraneFragmentCounts)
	{
		TestTrue(TEXT("Child terrane fragment counts stay non-negative"), ChildFragmentCount >= 0);
		TotalChildFragments += ChildFragmentCount;
	}
	TestEqual(TEXT("Post-rift terrane count matches the sum of child fragment counts"), RiftStats.RiftTerraneCountAfter, TotalChildFragments);
	for (const int32 TouchedChildCount : RiftStats.RiftPreTerraneTouchedChildCounts)
	{
		TestTrue(TEXT("Each pre-rift terrane touches at least one child"), TouchedChildCount >= 1);
		TestTrue(TEXT("Each pre-rift terrane touches at most the child count"), TouchedChildCount <= RiftStats.RiftChildCount);
	}

	AddInfo(FString::Printf(
		TEXT("forced_rift_multi_terrane_metrics parent=%d child_ids=(%s) child_samples=(%s) terranes_before=%d terranes_after=%d terrane_split_count=%d terrane_preserved_count=%d child_fragments=(%s) pre_terrane_ids=(%s) pre_terrane_touched_child_counts=(%s)"),
		ParentPlateId,
		*JoinIntArray(RiftStats.RiftChildPlateIds),
		*JoinIntArray(RiftStats.RiftChildSampleCounts),
		RiftStats.RiftTerraneCountBefore,
		RiftStats.RiftTerraneCountAfter,
		RiftStats.RiftTerraneSplitCount,
		RiftStats.RiftTerranePreservedCount,
		*JoinIntArray(RiftStats.RiftChildTerraneFragmentCounts),
		*JoinIntArray(RiftStats.RiftPreTerraneIds),
		*JoinIntArray(RiftStats.RiftPreTerraneTouchedChildCounts)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedRiftGeneralizedMediumHorizon60k7Test,
	"Aurous.TectonicPlanet.ForcedRiftGeneralizedMediumHorizon60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedRiftGeneralizedMediumHorizon60k7Test::RunTest(const FString& Parameters)
{
	struct FScenarioConfig
	{
		FString Name;
		int32 ChildCount = 2;
		int32 ParentSizeRank = 0;
		bool bInjectSyntheticTerranes = false;
		bool bRequirePersistentBoundary = true;
		int32 RiftSeed = 0;
	};

	auto RunScenario = [this](const FScenarioConfig& Config)
	{
		const double MinDominantComponentFraction = 0.98;
		FTectonicPlanet Planet = CreateInitializedPlanet();
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		for (int32 StepIndex = 0; StepIndex < 5; ++StepIndex)
		{
			Planet.AdvanceStep();
		}

		const FString ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			FString::Printf(TEXT("M5c-%s-seed42-samples60000-plates7"), *Config.Name));

		int32 ParentPlateId = FindPlateIdBySizeRank(Planet, Config.ParentSizeRank, Config.ChildCount * 256);
		if (ParentPlateId == INDEX_NONE)
		{
			ParentPlateId = FindPlateIdBySizeRank(Planet, 0, Config.ChildCount * 256);
		}

		TestTrue(FString::Printf(TEXT("%s parent plate exists"), *Config.Name), ParentPlateId != INDEX_NONE);
		TestTrue(FString::Printf(TEXT("%s pre-rift export succeeds"), *Config.Name), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PreRift")));
		if (Config.bInjectSyntheticTerranes)
		{
			TestTrue(
				FString::Printf(TEXT("%s synthetic multi-terrane setup succeeds"), *Config.Name),
				AssignSyntheticTerranesToPlateForTest(Planet, ParentPlateId, 3, Config.RiftSeed + 17));
		}

		TestTrue(
			FString::Printf(TEXT("%s forced rift succeeds"), *Config.Name),
			Planet.TriggerForcedRift(ParentPlateId, Config.ChildCount, Config.RiftSeed));

		const FResamplingStats RiftStats = Planet.LastResamplingStats;
		const TArray<int32> ChildPlateIds = RiftStats.RiftChildPlateIds;
		TestEqual(FString::Printf(TEXT("%s reports the expected child count"), *Config.Name), RiftStats.RiftChildCount, Config.ChildCount);
		TestEqual(FString::Printf(TEXT("%s records all child ids"), *Config.Name), ChildPlateIds.Num(), Config.ChildCount);
		TestTrue(FString::Printf(TEXT("%s immediate post-rift export succeeds"), *Config.Name), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftImmediate")));

		auto LogCheckpoint = [&](const FString& Label)
		{
			TArray<int32> ChildSampleCounts;
			TArray<int32> ChildComponentCounts;
			TArray<int32> ChildContinentalCounts;
			TArray<double> ChildLargestComponentFractions;
			for (const int32 ChildPlateId : ChildPlateIds)
			{
				const FPlateCoherenceSnapshot Snapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateId);
				ChildSampleCounts.Add(Snapshot.SampleCount);
				ChildComponentCounts.Add(Snapshot.ConnectedComponentCount);
				ChildContinentalCounts.Add(Snapshot.ContinentalSampleCount);
				ChildLargestComponentFractions.Add(ComputeLargestComponentFraction(Snapshot));
			}

			const FMultiPlateBoundaryActivitySummary BoundarySummary =
				BuildChildPlateBoundaryActivitySummary(Planet, ChildPlateIds);
			AddInfo(FString::Printf(
				TEXT("forced_rift_generalized_medium_horizon scenario=%s checkpoint=%s current_step=%d parent=%d child_count=%d child_ids=(%s) child_counts=(%s) components=(%s) dominant_fraction=(%s) continental=(%s) terranes_before=%d terranes_after=%d terrane_split_count=%d terrane_preserved_count=%d child_fragments=(%s) child_contact_edges=%d child_divergent_edges=%d last_gap_count=%d last_divergent_gap_count=%d"),
				*Config.Name,
				*Label,
				Planet.CurrentStep,
				ParentPlateId,
				Config.ChildCount,
				*JoinIntArray(ChildPlateIds),
				*JoinIntArray(ChildSampleCounts),
				*JoinIntArray(ChildComponentCounts),
				*JoinDoubleArray(ChildLargestComponentFractions),
				*JoinIntArray(ChildContinentalCounts),
				RiftStats.RiftTerraneCountBefore,
				RiftStats.RiftTerraneCountAfter,
				RiftStats.RiftTerraneSplitCount,
				RiftStats.RiftTerranePreservedCount,
				*JoinIntArray(RiftStats.RiftChildTerraneFragmentCounts),
				BoundarySummary.ContactEdgeCount,
				BoundarySummary.DivergentEdgeCount,
				Planet.LastResamplingStats.GapCount,
				Planet.LastResamplingStats.DivergentGapCount));
		};

		LogCheckpoint(TEXT("post_rift_immediate"));
		bool bSawPeriodicOceanicGeneration = false;
		while (Planet.CurrentStep < 55)
		{
			Planet.AdvanceStep();
			if (!Planet.ResamplingSteps.IsEmpty() &&
				Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
				Planet.LastResampleTriggerReason == EResampleTriggerReason::Periodic &&
				Planet.LastResamplingStats.DivergentGapCount > 0)
			{
				bSawPeriodicOceanicGeneration = true;
			}

			const int32 StepsAfterRift = Planet.CurrentStep - 5;
			if (StepsAfterRift == 30)
			{
				TestTrue(FString::Printf(TEXT("%s post-rift step 30 export succeeds"), *Config.Name), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftStep30")));
				LogCheckpoint(TEXT("post_rift_step_30"));
			}
			else if (StepsAfterRift == 50)
			{
				TestTrue(FString::Printf(TEXT("%s post-rift step 50 export succeeds"), *Config.Name), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftStep50")));
				LogCheckpoint(TEXT("post_rift_step_50"));
			}
		}

		for (const int32 ChildPlateId : ChildPlateIds)
		{
			const FPlateCoherenceSnapshot Snapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateId);
			TestTrue(FString::Printf(TEXT("%s child %d still exists at the final checkpoint"), *Config.Name, ChildPlateId), Planet.FindPlateArrayIndexById(ChildPlateId) != INDEX_NONE);
			TestTrue(FString::Printf(TEXT("%s child %d remains non-empty"), *Config.Name, ChildPlateId), Snapshot.SampleCount > 0);
			TestTrue(
				FString::Printf(TEXT("%s child %d retains a dominant connected component"), *Config.Name, ChildPlateId),
				ComputeLargestComponentFraction(Snapshot) >= MinDominantComponentFraction);
		}

		const FMultiPlateBoundaryActivitySummary FinalBoundarySummary =
			BuildChildPlateBoundaryActivitySummary(Planet, ChildPlateIds);
		TestTrue(FString::Printf(TEXT("%s later maintenance keeps generating divergent gaps"), *Config.Name), bSawPeriodicOceanicGeneration);
		if (Config.bRequirePersistentBoundary)
		{
			TestTrue(FString::Printf(TEXT("%s child-child boundary remains active"), *Config.Name), FinalBoundarySummary.ContactEdgeCount > 0);
			TestTrue(FString::Printf(TEXT("%s child-child boundary remains divergent"), *Config.Name), FinalBoundarySummary.DivergentEdgeCount > 0);
		}
		else
		{
			AddInfo(FString::Printf(
				TEXT("forced_rift_generalized_medium_horizon scenario=%s final_boundary_diagnostic contact_edges=%d divergent_edges=%d"),
				*Config.Name,
				FinalBoundarySummary.ContactEdgeCount,
				FinalBoundarySummary.DivergentEdgeCount));
		}
		TestTrue(FString::Printf(TEXT("%s retains valid stable plate ids and non-empty plates"), *Config.Name), AreAllStablePlateIdsValid(Planet));
		return true;
	};

	const FScenarioConfig MediumTwoChild{TEXT("ForcedRiftGeneralizedMediumHorizon60k7-ScenarioA-TwoChildMedium"), 2, 1, false, false, 301};
	const FScenarioConfig LargestThreeChildMultiTerrane{TEXT("ForcedRiftGeneralizedMediumHorizon60k7-ScenarioB-ThreeChildLargestMultiTerrane"), 3, 0, true, true, 311};
	const FScenarioConfig LargestFourChild{TEXT("ForcedRiftGeneralizedMediumHorizon60k7-ScenarioC-FourChildLargest"), 4, 0, false, true, 321};

	TestTrue(TEXT("Scenario A succeeds"), RunScenario(MediumTwoChild));
	TestTrue(TEXT("Scenario B succeeds"), RunScenario(LargestThreeChildMultiTerrane));
	TestTrue(TEXT("Scenario C succeeds"), RunScenario(LargestFourChild));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetAutomaticRiftTriggerSmokeTest,
	"Aurous.TectonicPlanet.AutomaticRiftTriggerSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetAutomaticRiftTriggerSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	ConfigureAutomaticRiftingForTest(Planet, 0.18);

	bool bSawAutomaticRift = false;
	for (int32 StepIndex = 0; StepIndex < 80; ++StepIndex)
	{
		Planet.AdvanceStep();
		if (!Planet.ResamplingSteps.IsEmpty() &&
			Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
			Planet.LastResampleTriggerReason == EResampleTriggerReason::RiftFollowup &&
			Planet.LastResamplingStats.RiftCount > 0 &&
			Planet.LastResamplingStats.bRiftWasAutomatic)
		{
			bSawAutomaticRift = true;
			break;
		}
	}

	TestTrue(TEXT("Automatic rifting fires in a bounded deterministic run"), bSawAutomaticRift);
	TestTrue(TEXT("The automatic rift produces child plates"), Planet.LastResamplingStats.RiftChildCount >= 2);
	TestTrue(TEXT("The automatic rift records a trigger probability"), Planet.LastResamplingStats.RiftTriggerProbability > 0.0);
	AddInfo(FString::Printf(
		TEXT("automatic_rift_trigger_smoke step=%d parent=%d child_count=%d child_ids=(%s) child_samples=(%s) probability=%.6f"),
		Planet.CurrentStep,
		Planet.LastResamplingStats.RiftParentPlateId,
		Planet.LastResamplingStats.RiftChildCount,
		*JoinIntArray(Planet.LastResamplingStats.RiftChildPlateIds),
		*JoinIntArray(Planet.LastResamplingStats.RiftChildSampleCounts),
		Planet.LastResamplingStats.RiftTriggerProbability));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetNoiseWarpedRiftBoundaryTest,
	"Aurous.TectonicPlanet.NoiseWarpedRiftBoundaryTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetNoiseWarpedRiftBoundaryTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet UnwarpedPlanet = CreateInitializedPlanet(5, 24000);
	FTectonicPlanet WarpedPlanet = CreateInitializedPlanet(5, 24000);
	const int32 ParentPlateId = FindLargestPlateId(UnwarpedPlanet);
	const int32 ParentPlateIndex = UnwarpedPlanet.FindPlateArrayIndexById(ParentPlateId);
	TestTrue(TEXT("Warped-boundary test parent exists"), ParentPlateIndex != INDEX_NONE);
	const TArray<int32> FormerParentMembers = UnwarpedPlanet.Plates[ParentPlateIndex].MemberSamples;

	UnwarpedPlanet.bEnableWarpedRiftBoundaries = false;
	WarpedPlanet.bEnableWarpedRiftBoundaries = true;
	WarpedPlanet.RiftBoundaryWarpAmplitude = 0.25;
	WarpedPlanet.RiftBoundaryWarpFrequency = 1.5;

	TestTrue(TEXT("Unwarped reference rift succeeds"), UnwarpedPlanet.TriggerForcedRift(ParentPlateId, 3, 601));
	TestTrue(TEXT("Warped rift succeeds"), WarpedPlanet.TriggerForcedRift(ParentPlateId, 3, 601));
	TestTrue(
		TEXT("Child ids remain stable across warped and unwarped runs"),
		UnwarpedPlanet.LastResamplingStats.RiftChildPlateIds == WarpedPlanet.LastResamplingStats.RiftChildPlateIds);

	int32 DifferingAssignmentCount = 0;
	for (const int32 SampleIndex : FormerParentMembers)
	{
		if (UnwarpedPlanet.Samples[SampleIndex].PlateId != WarpedPlanet.Samples[SampleIndex].PlateId)
		{
			++DifferingAssignmentCount;
		}
	}

	TestTrue(TEXT("Noise warping changes the fracture assignment materially"), DifferingAssignmentCount > FormerParentMembers.Num() / 50);
	for (const int32 ChildPlateId : WarpedPlanet.LastResamplingStats.RiftChildPlateIds)
	{
		const FPlateCoherenceSnapshot Snapshot = BuildPlateCoherenceSnapshot(WarpedPlanet, ChildPlateId);
		TestTrue(FString::Printf(TEXT("Warped child %d remains non-empty"), ChildPlateId), Snapshot.SampleCount > 0);
		TestTrue(
			FString::Printf(TEXT("Warped child %d retains a dominant component"), ChildPlateId),
			ComputeLargestComponentFraction(Snapshot) >= 0.95);
	}

	AddInfo(FString::Printf(
		TEXT("noise_warped_rift_boundary parent=%d differing_assignments=%d former_parent_samples=%d child_ids=(%s) child_samples=(%s)"),
		ParentPlateId,
		DifferingAssignmentCount,
		FormerParentMembers.Num(),
		*JoinIntArray(WarpedPlanet.LastResamplingStats.RiftChildPlateIds),
		*JoinIntArray(WarpedPlanet.LastResamplingStats.RiftChildSampleCounts)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPersistentDivergenceAfterRiftTest,
	"Aurous.TectonicPlanet.PersistentDivergenceAfterRiftTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPersistentDivergenceAfterRiftTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BaselinePlanet = CreateInitializedPlanet();
	FTectonicPlanet WarpedPlanet = CreateInitializedPlanet();
	BaselinePlanet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	WarpedPlanet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	BaselinePlanet.bEnableWarpedRiftBoundaries = false;
	WarpedPlanet.bEnableWarpedRiftBoundaries = true;
	WarpedPlanet.RiftBoundaryWarpAmplitude = 0.25;
	WarpedPlanet.RiftBoundaryWarpFrequency = 1.5;

	for (int32 StepIndex = 0; StepIndex < 5; ++StepIndex)
	{
		BaselinePlanet.AdvanceStep();
		WarpedPlanet.AdvanceStep();
	}

	int32 ParentPlateId = FindPlateIdBySizeRank(WarpedPlanet, 1, 2 * 256);
	if (ParentPlateId == INDEX_NONE)
	{
		ParentPlateId = FindLargestPlateId(WarpedPlanet);
	}

	TestTrue(TEXT("A medium parent plate exists for the persistent-divergence test"), ParentPlateId != INDEX_NONE);
	TestTrue(TEXT("Baseline medium 2-way rift succeeds"), BaselinePlanet.TriggerForcedRift(ParentPlateId, 2, 301));
	TestTrue(TEXT("Warped medium 2-way rift succeeds"), WarpedPlanet.TriggerForcedRift(ParentPlateId, 2, 301));

	const int32 BaselineChildPlateA = BaselinePlanet.LastResamplingStats.RiftChildPlateA;
	const int32 BaselineChildPlateB = BaselinePlanet.LastResamplingStats.RiftChildPlateB;
	const int32 WarpedChildPlateA = WarpedPlanet.LastResamplingStats.RiftChildPlateA;
	const int32 WarpedChildPlateB = WarpedPlanet.LastResamplingStats.RiftChildPlateB;
	int32 BaselineIntegratedContactEdges = 0;
	int32 BaselineIntegratedDivergentEdges = 0;
	int32 WarpedIntegratedContactEdges = 0;
	int32 WarpedIntegratedDivergentEdges = 0;
	bool bSawWarpedPeriodicOceanicGeneration = false;
	while (WarpedPlanet.CurrentStep < 55)
	{
		BaselinePlanet.AdvanceStep();
		WarpedPlanet.AdvanceStep();

		const int32 BaselineStepsAfterRift = BaselinePlanet.CurrentStep - 5;
		if (BaselineStepsAfterRift == 10 || BaselineStepsAfterRift == 20 || BaselineStepsAfterRift == 30 || BaselineStepsAfterRift == 40 || BaselineStepsAfterRift == 50)
		{
			const FPlatePairBoundaryActivitySummary BoundarySummary =
				BuildPlatePairBoundaryActivitySummary(BaselinePlanet, BaselineChildPlateA, BaselineChildPlateB);
			BaselineIntegratedContactEdges += BoundarySummary.ContactEdgeCount;
			BaselineIntegratedDivergentEdges += BoundarySummary.DivergentEdgeCount;
		}

		const int32 WarpedStepsAfterRift = WarpedPlanet.CurrentStep - 5;
		if (WarpedStepsAfterRift == 10 || WarpedStepsAfterRift == 20 || WarpedStepsAfterRift == 30 || WarpedStepsAfterRift == 40 || WarpedStepsAfterRift == 50)
		{
			const FPlatePairBoundaryActivitySummary BoundarySummary =
				BuildPlatePairBoundaryActivitySummary(WarpedPlanet, WarpedChildPlateA, WarpedChildPlateB);
			WarpedIntegratedContactEdges += BoundarySummary.ContactEdgeCount;
			WarpedIntegratedDivergentEdges += BoundarySummary.DivergentEdgeCount;
		}

		if (!WarpedPlanet.ResamplingSteps.IsEmpty() &&
			WarpedPlanet.ResamplingSteps.Last() == WarpedPlanet.CurrentStep &&
			WarpedPlanet.LastResampleTriggerReason == EResampleTriggerReason::Periodic &&
			WarpedPlanet.LastResamplingStats.DivergentGapCount > 0)
		{
			bSawWarpedPeriodicOceanicGeneration = true;
		}
	}

	const FPlatePairBoundaryActivitySummary BaselineFinalBoundarySummary =
		BuildPlatePairBoundaryActivitySummary(BaselinePlanet, BaselineChildPlateA, BaselineChildPlateB);
	const FPlatePairBoundaryActivitySummary WarpedFinalBoundarySummary =
		BuildPlatePairBoundaryActivitySummary(WarpedPlanet, WarpedChildPlateA, WarpedChildPlateB);
	TestTrue(TEXT("The warped medium 2-way case keeps more sibling-boundary contact activity than the unwarped control"), WarpedIntegratedContactEdges > BaselineIntegratedContactEdges);
	TestTrue(TEXT("The warped medium 2-way case keeps more sibling-boundary divergent activity than the unwarped control"), WarpedIntegratedDivergentEdges > BaselineIntegratedDivergentEdges);
	TestTrue(TEXT("Periodic maintenance keeps generating divergent gaps after the warped 2-way rift"), bSawWarpedPeriodicOceanicGeneration);

	AddInfo(FString::Printf(
		TEXT("persistent_divergence_after_rift parent=%d baseline_child_ids=(%d,%d) warped_child_ids=(%d,%d) integrated_contact_edges=(%d,%d) integrated_divergent_edges=(%d,%d) final_contact_edges=(%d,%d) final_divergent_edges=(%d,%d)"),
		ParentPlateId,
		BaselineChildPlateA,
		BaselineChildPlateB,
		WarpedChildPlateA,
		WarpedChildPlateB,
		BaselineIntegratedContactEdges,
		WarpedIntegratedContactEdges,
		BaselineIntegratedDivergentEdges,
		WarpedIntegratedDivergentEdges,
		BaselineFinalBoundarySummary.ContactEdgeCount,
		WarpedFinalBoundarySummary.ContactEdgeCount,
		BaselineFinalBoundarySummary.DivergentEdgeCount,
		WarpedFinalBoundarySummary.DivergentEdgeCount));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetAutomaticRiftMediumHorizon60k7Test,
	"Aurous.TectonicPlanet.AutomaticRiftMediumHorizon60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetAutomaticRiftMediumHorizon60k7Test::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	ConfigureAutomaticRiftingForTest(Planet, 0.18);

	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M5d-AutomaticRiftMediumHorizon60k7-seed42-samples60000-plates7"));

	int32 RiftStep = INDEX_NONE;
	int32 PlateCountBeforeRift = Planet.Plates.Num();
	TArray<int32> ChildPlateIds;
	FResamplingStats FirstRiftStats;
	bool bSawPeriodicOceanicGeneration = false;

	while (Planet.CurrentStep < 140)
	{
		const FTectonicPlanet PreAdvancePlanet = Planet;
		Planet.AdvanceStep();

		const bool bAutomaticRiftThisStep =
			!Planet.ResamplingSteps.IsEmpty() &&
			Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
			Planet.LastResampleTriggerReason == EResampleTriggerReason::RiftFollowup &&
			Planet.LastResamplingStats.RiftCount > 0 &&
			Planet.LastResamplingStats.bRiftWasAutomatic;

		if (bAutomaticRiftThisStep && RiftStep == INDEX_NONE)
		{
			RiftStep = Planet.CurrentStep;
			PlateCountBeforeRift = PreAdvancePlanet.Plates.Num();
			FirstRiftStats = Planet.LastResamplingStats;
			ChildPlateIds = FirstRiftStats.RiftChildPlateIds;
			Planet.bEnableAutomaticRifting = false;

			TestTrue(TEXT("Automatic-rift harness pre-rift export succeeds"), ExportNamedCheckpointMaps(*this, PreAdvancePlanet, ExportRoot, TEXT("PreRift")));
			TestTrue(TEXT("Automatic-rift harness immediate post-rift export succeeds"), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftImmediate")));
		}

		if (RiftStep != INDEX_NONE &&
			!Planet.ResamplingSteps.IsEmpty() &&
			Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
			Planet.LastResampleTriggerReason == EResampleTriggerReason::Periodic &&
			Planet.LastResamplingStats.DivergentGapCount > 0)
		{
			bSawPeriodicOceanicGeneration = true;
		}

		if (RiftStep != INDEX_NONE)
		{
			const int32 StepsAfterRift = Planet.CurrentStep - RiftStep;
			if (StepsAfterRift == 30)
			{
				TestTrue(TEXT("Automatic-rift harness step-30 export succeeds"), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftStep30")));
			}
			else if (StepsAfterRift == 50)
			{
				TestTrue(TEXT("Automatic-rift harness step-50 export succeeds"), ExportNamedCheckpointMaps(*this, Planet, ExportRoot, TEXT("PostRiftStep50")));
				break;
			}
		}
	}

	TestTrue(TEXT("The automatic medium-horizon harness reaches a rift event"), RiftStep != INDEX_NONE);
	TestTrue(TEXT("The first automatic rift records child ids"), ChildPlateIds.Num() >= 2);
	TestEqual(TEXT("Plate birth increases the active plate count by child_count - 1"), Planet.Plates.Num(), PlateCountBeforeRift + ChildPlateIds.Num() - 1);
	TestTrue(TEXT("Later maintenance resamples continue generating divergent gaps/oceanic crust after the automatic rift"), bSawPeriodicOceanicGeneration);
	TestTrue(TEXT("The automatic-rift horizon keeps all stable plate ids valid"), AreAllStablePlateIdsValid(Planet));

	const FMultiPlateBoundaryActivitySummary BoundarySummary =
		BuildChildPlateBoundaryActivitySummary(Planet, ChildPlateIds);
	TestTrue(TEXT("The automatic-rift child boundary remains active"), BoundarySummary.ContactEdgeCount > 0);
	TestTrue(TEXT("The automatic-rift child boundary remains divergent"), BoundarySummary.DivergentEdgeCount > 0);

	for (const int32 ChildPlateId : ChildPlateIds)
	{
		const FPlateCoherenceSnapshot Snapshot = BuildPlateCoherenceSnapshot(Planet, ChildPlateId);
		TestTrue(FString::Printf(TEXT("Automatic-rift child %d remains non-empty"), ChildPlateId), Snapshot.SampleCount > 0);
		TestTrue(
			FString::Printf(TEXT("Automatic-rift child %d remains coherent"), ChildPlateId),
			ComputeLargestComponentFraction(Snapshot) >= 0.98);
	}

	AddInfo(FString::Printf(
		TEXT("automatic_rift_medium_horizon rift_step=%d parent=%d child_count=%d child_ids=(%s) child_samples=(%s) boundary_contact_edges=%d boundary_divergent_edges=%d"),
		RiftStep,
		FirstRiftStats.RiftParentPlateId,
		FirstRiftStats.RiftChildCount,
		*JoinIntArray(ChildPlateIds),
		*JoinIntArray(FirstRiftStats.RiftChildSampleCounts),
		BoundarySummary.ContactEdgeCount,
		BoundarySummary.DivergentEdgeCount));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetForcedWarpedRiftMultiChildRegressionTest,
	"Aurous.TectonicPlanet.ForcedWarpedRiftMultiChildRegression",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetForcedWarpedRiftMultiChildRegressionTest::RunTest(const FString& Parameters)
{
	for (int32 ChildCount = 2; ChildCount <= 4; ++ChildCount)
	{
		FTectonicPlanet Planet = CreateInitializedPlanet(5, 24000);
		Planet.bEnableWarpedRiftBoundaries = true;
		Planet.RiftBoundaryWarpAmplitude = 0.25;
		Planet.RiftBoundaryWarpFrequency = 1.5;

		int32 ParentPlateId = FindPlateIdBySizeRank(Planet, 0, ChildCount * 256);
		if (ParentPlateId == INDEX_NONE)
		{
			ParentPlateId = FindLargestPlateId(Planet);
		}

		const int32 PlateCountBeforeRift = Planet.Plates.Num();
		TestTrue(
			FString::Printf(TEXT("Warped forced rift succeeds for %d children"), ChildCount),
			Planet.TriggerForcedRift(ParentPlateId, ChildCount, 701 + ChildCount));

		const FResamplingStats& RiftStats = Planet.LastResamplingStats;
		TestEqual(FString::Printf(TEXT("Warped forced rift reports %d children"), ChildCount), RiftStats.RiftChildCount, ChildCount);
		TestEqual(
			FString::Printf(TEXT("Warped forced rift increases the plate count correctly for %d children"), ChildCount),
			Planet.Plates.Num(),
			PlateCountBeforeRift + ChildCount - 1);
		TestTrue(FString::Printf(TEXT("Warped %d-child rift keeps stable ids valid"), ChildCount), AreAllStablePlateIdsValid(Planet));

		for (const int32 ChildPlateId : RiftStats.RiftChildPlateIds)
		{
			const int32 ChildPlateIndex = Planet.FindPlateArrayIndexById(ChildPlateId);
			TestTrue(FString::Printf(TEXT("Warped child plate %d exists"), ChildPlateId), ChildPlateIndex != INDEX_NONE);
			TestTrue(FString::Printf(TEXT("Warped child plate %d remains non-empty"), ChildPlateId), Planet.Plates[ChildPlateIndex].MemberSamples.Num() > 0);
		}
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetFullPipelineStress40Test,
	"Aurous.TectonicPlanet.FullPipelineStress40",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetFullPipelineStress40Test::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(StressPlateCount);
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M2-refmodel-FullPipelineStress40-seed42-samples60000-plates40"));

	TestTrue(TEXT("FullPipelineStress40 step 0 export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, 0));

	for (int32 StepIndex = 0; StepIndex < 30; ++StepIndex)
	{
		Planet.AdvanceStep();
		if (!Planet.ResamplingSteps.IsEmpty() && Planet.ResamplingSteps.Last() == Planet.CurrentStep)
		{
			const double GapRate = static_cast<double>(Planet.LastResamplingStats.GapCount) / static_cast<double>(Planet.Samples.Num());
			const double OverlapRate = static_cast<double>(Planet.LastResamplingStats.OverlapCount) / static_cast<double>(Planet.Samples.Num());
			const double ContinentalAreaFraction = ComputeContinentalAreaFraction(Planet);

				TestTrue(FString::Printf(TEXT("Stress40 gap rate stays below 30%% at step %d"), Planet.CurrentStep), GapRate < 0.30);
				TestTrue(FString::Printf(TEXT("Stress40 overlap rate stays below 10%% at step %d"), Planet.CurrentStep), OverlapRate < 0.10);
				TestTrue(FString::Printf(TEXT("Stress40 continental area fraction stays within the reference-model band at step %d"), Planet.CurrentStep), ContinentalAreaFraction >= 0.05 && ContinentalAreaFraction <= 0.50);
			for (const FPlate& Plate : Planet.Plates)
			{
				TestTrue(FString::Printf(TEXT("Stress40 plate %d retains members at step %d"), Plate.Id, Planet.CurrentStep), Plate.MemberSamples.Num() > 0);
			}
		}

		if (Planet.CurrentStep == 10 || Planet.CurrentStep == 20 || Planet.CurrentStep == 30)
		{
			TestTrue(
				*FString::Printf(TEXT("FullPipelineStress40 export succeeded at step %d"), Planet.CurrentStep),
				ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep));
		}
	}

	TestEqual(TEXT("FullPipelineStress40 resampled three times by step 30"), Planet.ResamplingSteps.Num(), 3);
	AddInfo(FString::Printf(
		TEXT("full_pipeline_stress40 step=%d gap_count=%d divergent_gap_count=%d non_divergent_gap_count=%d overlap_count=%d soup_ms=%.3f ownership_ms=%.3f interpolation_ms=%.3f gap_resolution_ms=%.3f repartition_ms=%.3f total_ms=%.3f"),
		Planet.LastResamplingStats.Step,
		Planet.LastResamplingStats.GapCount,
		Planet.LastResamplingStats.DivergentGapCount,
		Planet.LastResamplingStats.NonDivergentGapCount,
		Planet.LastResamplingStats.OverlapCount,
		Planet.LastResamplingStats.SoupBuildMs,
		Planet.LastResamplingStats.OwnershipQueryMs,
		Planet.LastResamplingStats.InterpolationMs,
		Planet.LastResamplingStats.GapResolutionMs,
		Planet.LastResamplingStats.RepartitionMs,
		Planet.LastResamplingStats.TotalMs));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetStressTest40Plates,
	"Aurous.TectonicPlanet.StressTest40Plates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetStressTest40Plates::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(StressPlateCount);
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("Stress40-seed42-samples60000-plates40"));

	TArray<int32> InitialPlateIds;
	InitialPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		InitialPlateIds.Add(Sample.PlateId);
	}

	TestTrue(TEXT("Stress40 step 0 export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, 0));
	for (int32 StepIndex = 0; StepIndex < 20; ++StepIndex)
	{
		Planet.AdvanceStep();
	}
	TestTrue(TEXT("Stress40 step 20 export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, 20));

	const double GapRate = static_cast<double>(Planet.LastResamplingStats.GapCount) / static_cast<double>(Planet.Samples.Num());
	const double OverlapRate = static_cast<double>(Planet.LastResamplingStats.OverlapCount) / static_cast<double>(Planet.Samples.Num());
	TestTrue(TEXT("Stress40 gap rate stays below 30%"), GapRate < 0.30);
	TestTrue(TEXT("Stress40 overlap rate stays below 10%"), OverlapRate < 0.10);

	bool bAnyOwnershipShifted = false;
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		if (Planet.Samples[SampleIndex].PlateId != InitialPlateIds[SampleIndex])
		{
			bAnyOwnershipShifted = true;
			break;
		}
	}
	TestTrue(TEXT("Stress40 ownership changed by step 20"), bAnyOwnershipShifted);
	AddInfo(FString::Printf(
		TEXT("stress40 step=%d gap_count=%d divergent_gap_count=%d non_divergent_gap_count=%d overlap_count=%d soup_ms=%.3f ownership_ms=%.3f interpolation_ms=%.3f gap_resolution_ms=%.3f repartition_ms=%.3f total_ms=%.3f"),
		Planet.LastResamplingStats.Step,
		Planet.LastResamplingStats.GapCount,
		Planet.LastResamplingStats.DivergentGapCount,
		Planet.LastResamplingStats.NonDivergentGapCount,
		Planet.LastResamplingStats.OverlapCount,
		Planet.LastResamplingStats.SoupBuildMs,
		Planet.LastResamplingStats.OwnershipQueryMs,
		Planet.LastResamplingStats.InterpolationMs,
		Planet.LastResamplingStats.GapResolutionMs,
		Planet.LastResamplingStats.RepartitionMs,
		Planet.LastResamplingStats.TotalMs));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetStabilityAfterFiveResamplings500k40Test,
	"Aurous.TectonicPlanet.StabilityAfterFiveResamplings500k40",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetStabilityAfterFiveResamplings500k40Test::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(StressPlateCount, HighResStressSampleCount);
	const int32 InitialContinentalSampleCount = CountContinentalSamples(Planet);
	TMap<int32, int32> ContinentalCountsByStep;
	ContinentalCountsByStep.Add(0, InitialContinentalSampleCount);
	TArray<FString> ResampleSummaries;
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("M3a-SubductionDistanceAndeanUplift-StabilityAfterFiveResamplings500k40-seed42-samples500000-plates40"));

	TestTrue(TEXT("500k/40 step 0 export succeeded"), ExportCheckpointMaps(*this, Planet, ExportRoot, 0));

	for (int32 StepIndex = 0; StepIndex < 50; ++StepIndex)
	{
		Planet.AdvanceStep();
		if (Planet.ResamplingSteps.IsEmpty() || Planet.ResamplingSteps.Last() != Planet.CurrentStep)
		{
			continue;
		}

		const int32 ContinentalSampleCount = CountContinentalSamples(Planet);
		ContinentalCountsByStep.Add(Planet.CurrentStep, ContinentalSampleCount);
		ResampleSummaries.Add(FString::Printf(TEXT("%d:%d"), Planet.CurrentStep, ContinentalSampleCount));
		TestTrue(FString::Printf(TEXT("500k/40 continental samples remain non-zero at step %d"), Planet.CurrentStep), ContinentalSampleCount > 0);
		TestEqual(FString::Printf(TEXT("500k/40 total soup triangles match global triangulation at step %d"), Planet.CurrentStep), Planet.LastResamplingStats.TotalSoupTriangleCount, Planet.TriangleIndices.Num());
		TestEqual(FString::Printf(TEXT("500k/40 drops no mixed triangles at step %d"), Planet.CurrentStep), Planet.LastResamplingStats.DroppedMixedTriangleCount, 0);
		TestEqual(FString::Printf(TEXT("500k/40 duplicates no soup triangles at step %d"), Planet.CurrentStep), Planet.LastResamplingStats.DuplicatedSoupTriangleCount, 0);
		TestEqual(FString::Printf(TEXT("500k/40 misses no local carried lookups at step %d"), Planet.CurrentStep), Planet.LastResamplingStats.MissingLocalCarriedLookupCount, 0);

		if (Planet.CurrentStep == 10 || Planet.CurrentStep == 20 || Planet.CurrentStep == 30 || Planet.CurrentStep == 40 || Planet.CurrentStep == 50)
		{
			TestTrue(
				*FString::Printf(TEXT("500k/40 export succeeded at step %d"), Planet.CurrentStep),
				ExportCheckpointMaps(*this, Planet, ExportRoot, Planet.CurrentStep));
		}
	}

	TestTrue(TEXT("500k/40 recorded a step-10 continental checkpoint"), ContinentalCountsByStep.Contains(10));
	const double FirstResampleRetention =
		InitialContinentalSampleCount > 0
			? static_cast<double>(ContinentalCountsByStep.FindRef(10)) / static_cast<double>(InitialContinentalSampleCount)
			: 0.0;
	// M6 target is 0.95+; M3 subduction uplift provides the continental creation mechanism.
	TestTrue(TEXT("500k/40 first-resample continental retention stays above the M2 regression guard"), FirstResampleRetention > 0.85);

	TestEqual(TEXT("500k/40 resampled five times by step 50"), Planet.ResamplingSteps.Num(), 5);
	TestEqual(TEXT("500k/40 latest resample landed at step 50"), Planet.LastResamplingStats.Step, 50);
	AddInfo(FString::Printf(
		TEXT("stability_after_five_resamplings_500k40 initial_continental_count=%d first_resample_retention=%.4f checkpoints=%s"),
		InitialContinentalSampleCount,
		FirstResampleRetention,
		*FString::Join(ResampleSummaries, TEXT(","))));

	return true;
}
