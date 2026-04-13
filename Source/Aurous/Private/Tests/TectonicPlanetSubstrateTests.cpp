#include "Misc/AutomationTest.h"

#include "Algo/Count.h"
#include "HAL/PlatformFile.h"
#include "HAL/PlatformFileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"
#include "TectonicMollweideExporter.h"
#include "TectonicPlanet.h"
#include "TectonicPlanetActor.h"

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

	FTectonicPlanet CreateInitializedPlanet(
		const int32 PlateCount = TestPlateCount,
		const int32 SampleCount = TestSampleCount,
		const int32 RandomSeed = TestRandomSeed)
	{
		FTectonicPlanet Planet;
		Planet.Initialize(SampleCount, TestPlanetRadiusKm);
		Planet.InitializePlates(PlateCount, RandomSeed, TestBoundaryWarpAmplitude, TestContinentalFraction);
		Planet.bEnableAutomaticRifting = false;
		return Planet;
	}

	void ConfigureAutomaticRiftingForTest(FTectonicPlanet& Planet, const double BaseRatePerMy = 0.18)
	{
		FTectonicPlanetRuntimeConfig RuntimeConfig = GetM6BaselineRuntimeConfig();
		RuntimeConfig.AutomaticRiftBaseRatePerMy = BaseRatePerMy;
		Planet.bEnableAutomaticRifting = RuntimeConfig.bEnableAutomaticRifting;
		Planet.bEnableWarpedRiftBoundaries = RuntimeConfig.bEnableWarpedRiftBoundaries;
		Planet.AutomaticRiftBaseRatePerMy = RuntimeConfig.AutomaticRiftBaseRatePerMy;
		Planet.AutomaticRiftMinParentSamples = RuntimeConfig.AutomaticRiftMinParentSamples;
		Planet.AutomaticRiftMinContinentalSamples = RuntimeConfig.AutomaticRiftMinContinentalSamples;
		Planet.AutomaticRiftMinContinentalFraction = RuntimeConfig.AutomaticRiftMinContinentalFraction;
		Planet.AutomaticRiftCooldownSteps = RuntimeConfig.AutomaticRiftCooldownSteps;
		Planet.RiftBoundaryWarpAmplitude = RuntimeConfig.RiftBoundaryWarpAmplitude;
		Planet.RiftBoundaryWarpFrequency = RuntimeConfig.RiftBoundaryWarpFrequency;
	}

	void TestEqualRuntimeConfig(
		FAutomationTestBase& Test,
		const TCHAR* Prefix,
		const FTectonicPlanetRuntimeConfig& Actual,
		const FTectonicPlanetRuntimeConfig& Expected)
	{
		Test.TestEqual(*FString::Printf(TEXT("%s resampling policy"), Prefix), Actual.ResamplingPolicy, Expected.ResamplingPolicy);
		Test.TestEqual(*FString::Printf(TEXT("%s automatic rifting enabled"), Prefix), Actual.bEnableAutomaticRifting, Expected.bEnableAutomaticRifting);
		Test.TestEqual(*FString::Printf(TEXT("%s warped rift boundaries enabled"), Prefix), Actual.bEnableWarpedRiftBoundaries, Expected.bEnableWarpedRiftBoundaries);
		Test.TestEqual(*FString::Printf(TEXT("%s automatic rift base rate"), Prefix), Actual.AutomaticRiftBaseRatePerMy, Expected.AutomaticRiftBaseRatePerMy);
		Test.TestEqual(*FString::Printf(TEXT("%s automatic rift min parent samples"), Prefix), Actual.AutomaticRiftMinParentSamples, Expected.AutomaticRiftMinParentSamples);
		Test.TestEqual(*FString::Printf(TEXT("%s automatic rift min continental samples"), Prefix), Actual.AutomaticRiftMinContinentalSamples, Expected.AutomaticRiftMinContinentalSamples);
		Test.TestEqual(*FString::Printf(TEXT("%s automatic rift min continental fraction"), Prefix), Actual.AutomaticRiftMinContinentalFraction, Expected.AutomaticRiftMinContinentalFraction);
		Test.TestEqual(*FString::Printf(TEXT("%s automatic rift cooldown steps"), Prefix), Actual.AutomaticRiftCooldownSteps, Expected.AutomaticRiftCooldownSteps);
		Test.TestEqual(*FString::Printf(TEXT("%s rift boundary warp amplitude"), Prefix), Actual.RiftBoundaryWarpAmplitude, Expected.RiftBoundaryWarpAmplitude);
		Test.TestEqual(*FString::Printf(TEXT("%s rift boundary warp frequency"), Prefix), Actual.RiftBoundaryWarpFrequency, Expected.RiftBoundaryWarpFrequency);
		Test.TestEqual(*FString::Printf(TEXT("%s Andean rate"), Prefix), Actual.AndeanContinentalConversionRatePerMy, Expected.AndeanContinentalConversionRatePerMy);
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

	const TCHAR* GetRiftRecipientTypeNameForTest(const int32 TypeCode)
	{
		switch (TypeCode)
		{
		case 1:
			return TEXT("SiblingChild");
		case 2:
			return TEXT("OtherNeighbor");
		case 0:
		default:
			return TEXT("OtherAffectedPlate");
		}
	}

	FString JoinRiftRecipientTypeCodesForTest(const TArray<int32>& TypeCodes)
	{
		TArray<FString> Parts;
		Parts.Reserve(TypeCodes.Num());
		for (const int32 TypeCode : TypeCodes)
		{
			Parts.Add(GetRiftRecipientTypeNameForTest(TypeCode));
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

	int32 PickDominantBarycentricIndexForTest(const FVector3d& Barycentric)
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
		TSet<int32> ActivePlateIds;
		for (const FSample& Sample : Planet.Samples)
		{
			if (Planet.FindPlateArrayIndexById(Sample.PlateId) == INDEX_NONE)
			{
				return false;
			}
		}

		for (const FPlate& Plate : Planet.Plates)
		{
			if (Plate.Id == INDEX_NONE || ActivePlateIds.Contains(Plate.Id))
			{
				return false;
			}
			ActivePlateIds.Add(Plate.Id);

			if (Plate.MemberSamples.IsEmpty())
			{
				return false;
			}

			for (const int32 SampleIndex : Plate.MemberSamples)
			{
				if (!Planet.Samples.IsValidIndex(SampleIndex) ||
					Planet.Samples[SampleIndex].PlateId != Plate.Id)
				{
					return false;
				}
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

	FTectonicPlanet CreateManualPreserveBoundaryHysteresisTestPlanet(
		const double PreviousOwnerRotationRadians,
		const bool bFavorPreviousOwnerNeighbors,
		const float PreviousSampleContinentalWeight = 1.0f,
		const double RecoveryTolerance = 0.0015)
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.ContainmentRecoveryTolerance = RecoveryTolerance;
		Planet.Samples.SetNum(4);
		Planet.SampleAdjacency.SetNum(4);
		Planet.TriangleIndices = { FIntVector(0, 1, 2) };
		Planet.TriangleAdjacency.SetNum(1);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].SoupTriangles = { 0 };
		}

		Planet.Samples[0].Position = FVector3d(1.0, -0.002, -0.002).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(1.0, 0.002, -0.002).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(1.0, 0.0, 0.002).GetSafeNormal();
		Planet.Samples[3].Position =
			(Planet.Samples[0].Position + Planet.Samples[1].Position + Planet.Samples[2].Position).GetSafeNormal();

		Planet.Samples[0].PlateId = 0;
		Planet.Samples[1].PlateId = bFavorPreviousOwnerNeighbors ? 0 : 1;
		Planet.Samples[2].PlateId = 1;
		Planet.Samples[3].PlateId = 0;
		Planet.Samples[3].ContinentalWeight = PreviousSampleContinentalWeight;

		Planet.SampleAdjacency[3] = { 0, 1, 2 };

		Planet.Plates[0].OverlapScore = 5;
		Planet.Plates[1].OverlapScore = 25;
		Planet.Plates[0].CumulativeRotation =
			FQuat4d(FVector3d(0.0, 0.0, 1.0), PreviousOwnerRotationRadians);
		Planet.Plates[1].CumulativeRotation = FQuat4d::Identity;

		for (int32 PlateId = 0; PlateId < 2; ++PlateId)
		{
			FPlate& Plate = Planet.Plates[PlateId];
			Plate.CarriedSamples.Reset();
			Plate.CanonicalToCarriedIndex.Reset();
			for (int32 VertexIndex = 0; VertexIndex < 3; ++VertexIndex)
			{
				FCarriedSample CarriedSample;
				CarriedSample.CanonicalSampleIndex = VertexIndex;
				CarriedSample.ContinentalWeight = PlateId == 0 ? 1.0f : 0.2f;
				CarriedSample.Elevation = PlateId == 0 ? 2.0f : 0.5f;
				CarriedSample.Thickness = PlateId == 0 ? 30.0f : 15.0f;
				CarriedSample.Age = 5.0f;
				CarriedSample.SubductionDistanceKm = -1.0f;
				CarriedSample.SubductionSpeed = 0.0f;
				Plate.CanonicalToCarriedIndex.Add(
					VertexIndex,
					Plate.CarriedSamples.Add(CarriedSample));
			}
		}
		Planet.ComputePlateScores();
		return Planet;
	}

	void ConfigureManualTriangleCarriedSamples(
		FTectonicPlanet& Planet,
		const int32 PlateId,
		const float ContinentalWeight,
		const float ElevationBase = 1.0f)
	{
		const TArray<int32> TriangleVertices = { 0, 1, 2 };
		const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
		check(Planet.Plates.IsValidIndex(PlateIndex));

		FPlate& Plate = Planet.Plates[PlateIndex];
		Plate.CarriedSamples.Reset();
		Plate.CanonicalToCarriedIndex.Reset();
		for (int32 VertexOffset = 0; VertexOffset < TriangleVertices.Num(); ++VertexOffset)
		{
			FCarriedSample CarriedSample;
			CarriedSample.CanonicalSampleIndex = TriangleVertices[VertexOffset];
			CarriedSample.ContinentalWeight = ContinentalWeight;
			CarriedSample.Elevation = ElevationBase + static_cast<float>(VertexOffset);
			CarriedSample.Thickness = 20.0f + static_cast<float>(VertexOffset);
			CarriedSample.Age = 5.0f + static_cast<float>(VertexOffset);
			CarriedSample.SubductionDistanceKm = -1.0f;
			CarriedSample.SubductionSpeed = 0.0f;
			Plate.CanonicalToCarriedIndex.Add(TriangleVertices[VertexOffset], Plate.CarriedSamples.Add(CarriedSample));
		}
	}

	void ConfigureManualTriangleCarriedSamplesExplicit(
		FTectonicPlanet& Planet,
		const int32 PlateId,
		const TArray<float>& ContinentalWeights,
		const TArray<float>& Elevations,
		const TArray<float>& Thicknesses,
		const TArray<float>& Ages)
	{
		check(ContinentalWeights.Num() == 3);
		check(Elevations.Num() == 3);
		check(Thicknesses.Num() == 3);
		check(Ages.Num() == 3);

		const TArray<int32> TriangleVertices = { 0, 1, 2 };
		const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
		check(Planet.Plates.IsValidIndex(PlateIndex));

		FPlate& Plate = Planet.Plates[PlateIndex];
		Plate.CarriedSamples.Reset();
		Plate.CanonicalToCarriedIndex.Reset();
		for (int32 VertexOffset = 0; VertexOffset < TriangleVertices.Num(); ++VertexOffset)
		{
			FCarriedSample CarriedSample;
			CarriedSample.CanonicalSampleIndex = TriangleVertices[VertexOffset];
			CarriedSample.ContinentalWeight = ContinentalWeights[VertexOffset];
			CarriedSample.Elevation = Elevations[VertexOffset];
			CarriedSample.Thickness = Thicknesses[VertexOffset];
			CarriedSample.Age = Ages[VertexOffset];
			CarriedSample.SubductionDistanceKm = -1.0f;
			CarriedSample.SubductionSpeed = 0.0f;
			Plate.CanonicalToCarriedIndex.Add(
				TriangleVertices[VertexOffset],
				Plate.CarriedSamples.Add(CarriedSample));
		}
	}

	FTectonicPlanet CreateManualContinentalWeightDominantVertexTransferTestPlanet()
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
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(0.15, 0.70, 0.15).GetSafeNormal();

		for (FSample& Sample : Planet.Samples)
		{
			Sample.PlateId = 0;
		}

		return Planet;
	}

	void ConfigureManualCarriedSamplesForVertices(
		FTectonicPlanet& Planet,
		const int32 PlateId,
		const TArray<int32>& VertexSampleIndices,
		const float ContinentalWeight,
		const float ElevationBase = 1.0f)
	{
		const int32 PlateIndex = Planet.FindPlateArrayIndexById(PlateId);
		check(Planet.Plates.IsValidIndex(PlateIndex));

		FPlate& Plate = Planet.Plates[PlateIndex];
		Plate.CarriedSamples.Reset();
		Plate.CanonicalToCarriedIndex.Reset();
		for (int32 VertexOffset = 0; VertexOffset < VertexSampleIndices.Num(); ++VertexOffset)
		{
			FCarriedSample CarriedSample;
			CarriedSample.CanonicalSampleIndex = VertexSampleIndices[VertexOffset];
			CarriedSample.ContinentalWeight = ContinentalWeight;
			CarriedSample.Elevation = ElevationBase + static_cast<float>(VertexOffset);
			CarriedSample.Thickness = 20.0f + static_cast<float>(VertexOffset);
			CarriedSample.Age = 5.0f + static_cast<float>(VertexOffset);
			CarriedSample.SubductionDistanceKm = -1.0f;
			CarriedSample.SubductionSpeed = 0.0f;
			Plate.CanonicalToCarriedIndex.Add(VertexSampleIndices[VertexOffset], Plate.CarriedSamples.Add(CarriedSample));
		}
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

	FTectonicPlanet CreateManualFormerContinentalDivergentGapPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(3);
		Planet.SampleAdjacency.SetNum(3);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
		}

		Planet.Samples[0].Position = FVector3d(-0.05, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.05, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();

		Planet.Samples[0].PlateId = 0;
		Planet.Samples[1].PlateId = 1;
		Planet.Samples[2].PlateId = INDEX_NONE;

		Planet.Samples[0].ContinentalWeight = 1.0f;
		Planet.Samples[1].ContinentalWeight = 1.0f;
		Planet.Samples[2].ContinentalWeight = 1.0f;
		Planet.Samples[0].Elevation = 0.3f;
		Planet.Samples[1].Elevation = 0.4f;
		Planet.Samples[2].Elevation = 1.5f;
		Planet.Samples[0].Thickness = 35.0f;
		Planet.Samples[1].Thickness = 35.0f;
		Planet.Samples[2].Thickness = 35.0f;
		Planet.Samples[2].Age = 12.0f;
		Planet.Samples[2].OrogenyType = EOrogenyType::Andean;

		Planet.SampleAdjacency[2] = { 0, 1 };
		Planet.SampleAdjacency[0] = { 2 };
		Planet.SampleAdjacency[1] = { 2 };

		Planet.Plates[0].MemberSamples = { 0 };
		Planet.Plates[1].MemberSamples = { 1 };
		Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
		Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
		Planet.Plates[0].AngularSpeed = 0.02;
		Planet.Plates[1].AngularSpeed = 0.02;
		return Planet;
	}

	FTectonicPlanet CreateManualProjectionGapRecoveryPlanet(const int32 PreviousPlateId)
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(5);
		Planet.SampleAdjacency.SetNum(5);
		Planet.TriangleIndices = { FIntVector(0, 1, 2) };
		Planet.TriangleAdjacency.SetNum(1);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
			Planet.Plates[PlateIndex].RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Planet.Plates[PlateIndex].AngularSpeed = 0.0;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(1.0, 1.0, 1.0).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(-1.0, 0.0, 0.0).GetSafeNormal();

		Planet.Samples[0].PlateId = 0;
		Planet.Samples[1].PlateId = 0;
		Planet.Samples[2].PlateId = 0;
		Planet.Samples[3].PlateId = PreviousPlateId;
		Planet.Samples[4].PlateId = 1;

		for (FSample& Sample : Planet.Samples)
		{
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		Planet.SampleAdjacency[0] = { 3 };
		Planet.SampleAdjacency[1] = { 3 };
		Planet.SampleAdjacency[2] = { 3 };
		Planet.SampleAdjacency[3] = { 0, 1, 2, 4 };
		Planet.SampleAdjacency[4] = { 3 };

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[1].MemberSamples = { 4 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[1].SoupTriangles.Reset();
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 0.2f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualGeometricCollisionOverlapPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(7);
		Planet.SampleAdjacency.SetNum(7);
		Planet.TriangleIndices = { FIntVector(0, 1, 2) };
		Planet.TriangleAdjacency.SetNum(1);
		Planet.Plates.SetNum(2);
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(1.0, 1.0, 1.0).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.8, 1.0, 1.0).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(1.0, 0.8, 1.0).GetSafeNormal();
		Planet.Samples[6].Position = FVector3d(-1.0, 1.0, 1.0).GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = SampleIndex < 3 ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		for (const int32 BoundarySampleIndex : { 3, 4, 5 })
		{
			AddUndirectedEdge(0, BoundarySampleIndex);
			AddUndirectedEdge(1, BoundarySampleIndex);
			AddUndirectedEdge(2, BoundarySampleIndex);
		}
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 6);
		AddUndirectedEdge(3, 6);

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
		Planet.Plates[0].AngularSpeed = 0.02;

		Planet.Plates[1].MemberSamples = { 3, 4, 5, 6 };
		Planet.Plates[1].SoupTriangles.Reset();
		Planet.Plates[1].OverlapScore = 50;
		Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
		Planet.Plates[1].AngularSpeed = 0.02;
		Planet.GeometricCollisionMinPersistentPenetrationKm = 0.0;

		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualGeometricCollisionExecutionPlanet()
	{
		FTectonicPlanet Planet = CreateManualGeometricCollisionOverlapPlanet();
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(2);
		Planet.Samples[3].Position = FVector3d(0.60, 0.20, 0.20).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.20, 0.60, 0.20).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.20, 0.20, 0.60).GetSafeNormal();
		Planet.Samples[6].Position = FVector3d(0.34, 0.33, 0.33).GetSafeNormal();
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[1].OverlapScore = 50;
		Planet.Plates[1].SoupTriangles = { 1 };
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualShallowGeometricCollisionExecutionPlanet()
	{
		FTectonicPlanet Planet = CreateManualGeometricCollisionExecutionPlanet();
		Planet.Samples[3].Position = FVector3d(0.49, 0.49, 0.02).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.49, 0.02, 0.49).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.02, 0.49, 0.49).GetSafeNormal();
		Planet.Samples[6].Position = FVector3d(0.44, 0.44, 0.12).GetSafeNormal();
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualGeometricCollisionLocalityClampPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		Planet.Samples.SetNum(15);
		Planet.SampleAdjacency.SetNum(15);
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(2);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
			Planet.Plates[PlateIndex].RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Planet.Plates[PlateIndex].AngularSpeed = 0.02;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(0.55, 0.25, 0.20).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.25, 0.55, 0.20).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.20, 0.25, 0.55).GetSafeNormal();

		const FVector3d ChainEnd = FVector3d(-1.0, -0.2, 0.1).GetSafeNormal();
		for (int32 ChainIndex = 0; ChainIndex < 9; ++ChainIndex)
		{
			const double T = static_cast<double>(ChainIndex + 1) / 10.0;
			Planet.Samples[6 + ChainIndex].Position =
				FMath::Lerp(Planet.Samples[3].Position, ChainEnd, T).GetSafeNormal();
		}

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = SampleIndex < 3 ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		for (const int32 DonorSeedIndex : { 3, 4, 5 })
		{
			AddUndirectedEdge(0, DonorSeedIndex);
			AddUndirectedEdge(1, DonorSeedIndex);
			AddUndirectedEdge(2, DonorSeedIndex);
			AddUndirectedEdge(DonorSeedIndex, 6);
		}
		for (int32 ChainSampleIndex = 6; ChainSampleIndex < 14; ++ChainSampleIndex)
		{
			AddUndirectedEdge(ChainSampleIndex, ChainSampleIndex + 1);
		}

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[1].MemberSamples = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
		Planet.Plates[1].SoupTriangles = { 1 };
		Planet.Plates[1].OverlapScore = 50;
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualCollisionDonorFragmentationPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		Planet.GeometricCollisionDonorLocalityClampKm = 400.0;
		Planet.Samples.SetNum(14);
		Planet.SampleAdjacency.SetNum(14);
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(2);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
			Planet.Plates[PlateIndex].RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Planet.Plates[PlateIndex].AngularSpeed = 0.02;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(0.56, 0.24, 0.20).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.24, 0.56, 0.20).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.20, 0.24, 0.56).GetSafeNormal();
		Planet.Samples[6].Position = FVector3d(0.72, 0.18, 0.18).GetSafeNormal();
		Planet.Samples[7].Position = FVector3d(0.88, 0.10, 0.10).GetSafeNormal();
		Planet.Samples[8].Position = FVector3d(0.18, 0.72, 0.18).GetSafeNormal();
		Planet.Samples[9].Position = FVector3d(0.10, 0.88, 0.10).GetSafeNormal();
		Planet.Samples[10].Position = FVector3d(0.18, 0.18, 0.72).GetSafeNormal();
		Planet.Samples[11].Position = FVector3d(0.10, 0.10, 0.88).GetSafeNormal();
		Planet.Samples[12].Position = FVector3d(0.80, 0.22, 0.18).GetSafeNormal();
		Planet.Samples[13].Position = FVector3d(0.94, 0.18, 0.14).GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = SampleIndex < 3 ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		for (const int32 DonorSeedIndex : { 3, 4, 5 })
		{
			AddUndirectedEdge(0, DonorSeedIndex);
			AddUndirectedEdge(1, DonorSeedIndex);
			AddUndirectedEdge(2, DonorSeedIndex);
		}
		AddUndirectedEdge(3, 6);
		AddUndirectedEdge(6, 7);
		AddUndirectedEdge(4, 8);
		AddUndirectedEdge(8, 9);
		AddUndirectedEdge(5, 10);
		AddUndirectedEdge(10, 11);
		AddUndirectedEdge(3, 12);
		AddUndirectedEdge(12, 13);

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[1].MemberSamples = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
		Planet.Plates[1].SoupTriangles = { 1 };
		Planet.Plates[1].OverlapScore = 50;
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualCollisionDonorMeaningfulFragmentPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		Planet.GeometricCollisionDonorLocalityClampKm = 4000.0;
		Planet.Samples.SetNum(18);
		Planet.SampleAdjacency.SetNum(18);
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(2);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
			Planet.Plates[PlateIndex].RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Planet.Plates[PlateIndex].AngularSpeed = 0.02;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(0.56, 0.24, 0.20).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.24, 0.56, 0.20).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.20, 0.24, 0.56).GetSafeNormal();
		Planet.Samples[6].Position = FVector3d(0.70, 0.20, 0.18).GetSafeNormal();
		Planet.Samples[7].Position = FVector3d(0.82, 0.18, 0.16).GetSafeNormal();
		Planet.Samples[8].Position = FVector3d(0.90, 0.16, 0.14).GetSafeNormal();
		Planet.Samples[9].Position = FVector3d(0.18, 0.70, 0.18).GetSafeNormal();
		Planet.Samples[10].Position = FVector3d(0.16, 0.82, 0.16).GetSafeNormal();
		Planet.Samples[11].Position = FVector3d(0.14, 0.90, 0.14).GetSafeNormal();
		Planet.Samples[12].Position = FVector3d(0.18, 0.18, 0.70).GetSafeNormal();
		Planet.Samples[13].Position = FVector3d(0.16, 0.16, 0.82).GetSafeNormal();
		Planet.Samples[14].Position = FVector3d(0.14, 0.14, 0.90).GetSafeNormal();
		Planet.Samples[15].Position = FVector3d(0.74, 0.28, 0.18).GetSafeNormal();
		Planet.Samples[16].Position = FVector3d(0.86, 0.30, 0.16).GetSafeNormal();
		Planet.Samples[17].Position = FVector3d(0.94, 0.32, 0.14).GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = SampleIndex < 3 ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		for (const int32 DonorSeedIndex : { 3, 4, 5 })
		{
			AddUndirectedEdge(0, DonorSeedIndex);
			AddUndirectedEdge(1, DonorSeedIndex);
			AddUndirectedEdge(2, DonorSeedIndex);
		}
		AddUndirectedEdge(3, 6);
		AddUndirectedEdge(6, 7);
		AddUndirectedEdge(7, 8);
		AddUndirectedEdge(4, 9);
		AddUndirectedEdge(9, 10);
		AddUndirectedEdge(10, 11);
		AddUndirectedEdge(5, 12);
		AddUndirectedEdge(12, 13);
		AddUndirectedEdge(13, 14);
		AddUndirectedEdge(3, 15);
		AddUndirectedEdge(15, 16);
		AddUndirectedEdge(16, 17);

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[1].MemberSamples = {
			3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17
		};
		Planet.Plates[1].SoupTriangles = { 1 };
		Planet.Plates[1].OverlapScore = 50;
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualCollisionDonorCapRelaxationPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		Planet.GeometricCollisionDonorLocalityClampKm = 400.0;
		Planet.Samples.SetNum(20);
		Planet.SampleAdjacency.SetNum(20);
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(2);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
			Planet.Plates[PlateIndex].RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Planet.Plates[PlateIndex].AngularSpeed = 0.02;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(0.56, 0.24, 0.20).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.24, 0.56, 0.20).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.20, 0.24, 0.56).GetSafeNormal();
		Planet.Samples[6].Position = FVector3d(0.78, 0.16, 0.16).GetSafeNormal();
		Planet.Samples[7].Position = FVector3d(0.90, 0.10, 0.10).GetSafeNormal();
		Planet.Samples[8].Position = FVector3d(0.70, 0.32, 0.16).GetSafeNormal();
		Planet.Samples[9].Position = FVector3d(0.82, 0.38, 0.12).GetSafeNormal();
		Planet.Samples[10].Position = FVector3d(0.62, 0.44, 0.18).GetSafeNormal();
		Planet.Samples[11].Position = FVector3d(0.70, 0.54, 0.14).GetSafeNormal();
		Planet.Samples[12].Position = FVector3d(0.16, 0.78, 0.16).GetSafeNormal();
		Planet.Samples[13].Position = FVector3d(0.10, 0.90, 0.10).GetSafeNormal();
		Planet.Samples[14].Position = FVector3d(0.32, 0.70, 0.16).GetSafeNormal();
		Planet.Samples[15].Position = FVector3d(0.38, 0.82, 0.12).GetSafeNormal();
		Planet.Samples[16].Position = FVector3d(0.16, 0.16, 0.78).GetSafeNormal();
		Planet.Samples[17].Position = FVector3d(0.10, 0.10, 0.90).GetSafeNormal();
		Planet.Samples[18].Position = FVector3d(0.32, 0.16, 0.70).GetSafeNormal();
		Planet.Samples[19].Position = FVector3d(0.38, 0.12, 0.82).GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = SampleIndex < 3 ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		for (const int32 DonorSeedIndex : { 3, 4, 5 })
		{
			AddUndirectedEdge(0, DonorSeedIndex);
			AddUndirectedEdge(1, DonorSeedIndex);
			AddUndirectedEdge(2, DonorSeedIndex);
		}

		AddUndirectedEdge(3, 6);
		AddUndirectedEdge(6, 7);
		AddUndirectedEdge(3, 8);
		AddUndirectedEdge(8, 9);
		AddUndirectedEdge(3, 10);
		AddUndirectedEdge(10, 11);
		AddUndirectedEdge(4, 12);
		AddUndirectedEdge(12, 13);
		AddUndirectedEdge(4, 14);
		AddUndirectedEdge(14, 15);
		AddUndirectedEdge(5, 16);
		AddUndirectedEdge(16, 17);
		AddUndirectedEdge(5, 18);
		AddUndirectedEdge(18, 19);

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[1].MemberSamples = {
			3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
		};
		Planet.Plates[1].SoupTriangles = { 1 };
		Planet.Plates[1].OverlapScore = 50;
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualCollisionDonorComponentCapPathologyPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		Planet.GeometricCollisionDonorLocalityClampKm = 400.0;
		Planet.Samples.SetNum(18);
		Planet.SampleAdjacency.SetNum(18);
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(2);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
			Planet.Plates[PlateIndex].RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Planet.Plates[PlateIndex].AngularSpeed = 0.02;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(0.56, 0.24, 0.20).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.24, 0.56, 0.20).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.20, 0.24, 0.56).GetSafeNormal();
		Planet.Samples[6].Position = FVector3d(0.78, 0.16, 0.16).GetSafeNormal();
		Planet.Samples[7].Position = FVector3d(0.88, 0.10, 0.10).GetSafeNormal();
		Planet.Samples[8].Position = FVector3d(0.70, 0.32, 0.16).GetSafeNormal();
		Planet.Samples[9].Position = FVector3d(0.62, 0.44, 0.18).GetSafeNormal();
		Planet.Samples[10].Position = FVector3d(0.16, 0.78, 0.16).GetSafeNormal();
		Planet.Samples[11].Position = FVector3d(0.10, 0.88, 0.10).GetSafeNormal();
		Planet.Samples[12].Position = FVector3d(0.32, 0.70, 0.16).GetSafeNormal();
		Planet.Samples[13].Position = FVector3d(0.44, 0.62, 0.18).GetSafeNormal();
		Planet.Samples[14].Position = FVector3d(0.16, 0.16, 0.78).GetSafeNormal();
		Planet.Samples[15].Position = FVector3d(0.10, 0.10, 0.88).GetSafeNormal();
		Planet.Samples[16].Position = FVector3d(0.32, 0.16, 0.70).GetSafeNormal();
		Planet.Samples[17].Position = FVector3d(0.18, 0.44, 0.62).GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = SampleIndex < 3 ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		for (const int32 DonorSeedIndex : { 3, 4, 5 })
		{
			AddUndirectedEdge(0, DonorSeedIndex);
			AddUndirectedEdge(1, DonorSeedIndex);
			AddUndirectedEdge(2, DonorSeedIndex);
		}

		AddUndirectedEdge(3, 6);
		AddUndirectedEdge(3, 7);
		AddUndirectedEdge(3, 8);
		AddUndirectedEdge(3, 9);
		AddUndirectedEdge(4, 10);
		AddUndirectedEdge(4, 11);
		AddUndirectedEdge(4, 12);
		AddUndirectedEdge(4, 13);
		AddUndirectedEdge(5, 14);
		AddUndirectedEdge(5, 15);
		AddUndirectedEdge(5, 16);
		AddUndirectedEdge(5, 17);

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[1].MemberSamples = {
			3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17
		};
		Planet.Plates[1].SoupTriangles = { 1 };
		Planet.Plates[1].OverlapScore = 50;
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	int32 CountPlateComponentsForAssignmentsForTest(
		const FTectonicPlanet& Planet,
		const TArray<int32>& PlateAssignments,
		const int32 PlateId)
	{
		if (PlateId == INDEX_NONE || PlateAssignments.Num() != Planet.Samples.Num())
		{
			return 0;
		}

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		int32 ComponentCount = 0;
		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (Visited[SeedSampleIndex] != 0 || PlateAssignments[SeedSampleIndex] != PlateId)
			{
				continue;
			}

			++ComponentCount;
			TArray<int32, TInlineAllocator<32>> Stack;
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
					if (!PlateAssignments.IsValidIndex(NeighborIndex) ||
						Visited[NeighborIndex] != 0 ||
						PlateAssignments[NeighborIndex] != PlateId)
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

		FTectonicPlanet CreateManualRiftLocalizationDiagnosticPlanet()
		{
			FTectonicPlanet Planet;
		Planet.Samples.SetNum(5);
		Planet.SampleAdjacency.SetNum(5);
		Planet.Plates.SetNum(5);
		const TArray<int32> PlateIds = { 3, 6, 7, 10, 11 };
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			FPlate& Plate = Planet.Plates[PlateIndex];
			Plate.Id = PlateIds[PlateIndex];
		}
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			Planet.Samples[SampleIndex].PlateId = SampleIndex == 1 ? 10 : (SampleIndex == 2 ? 11 : (SampleIndex + 3));
			Planet.Samples[SampleIndex].SubductionDistanceKm = 100.0f + static_cast<float>(SampleIndex);
			FPlate* Plate = nullptr;
			for (FPlate& CandidatePlate : Planet.Plates)
			{
				if (CandidatePlate.Id == Planet.Samples[SampleIndex].PlateId)
				{
					Plate = &CandidatePlate;
					break;
				}
			}
			if (Plate != nullptr)
			{
				const int32 CarriedIndex = Plate->CarriedSamples.Num();
				FCarriedSample& CarriedSample = Plate->CarriedSamples.AddDefaulted_GetRef();
				CarriedSample.CanonicalSampleIndex = SampleIndex;
				CarriedSample.SubductionDistanceKm = 100.0f + static_cast<float>(SampleIndex);
				CarriedSample.SubductionSpeed = 10.0f + static_cast<float>(SampleIndex);
				Plate->CanonicalToCarriedIndex.Add(SampleIndex, CarriedIndex);
			}
		}

		Planet.PendingRiftEvent = FPendingRiftEvent{};
		Planet.PendingRiftEvent.bValid = true;
		Planet.PendingRiftEvent.ChildCount = 2;
		Planet.PendingRiftEvent.ChildPlateA = 10;
		Planet.PendingRiftEvent.ChildPlateB = 11;
		Planet.PendingRiftEvent.ChildPlateIds = { 10, 11 };
			Planet.PendingRiftEvent.ChildAnchorSampleIndices = { 1, 2 };
			return Planet;
		}

		FTectonicPlanet CreateManualRiftFinalOwnerCWDiagnosticPlanet()
		{
			FTectonicPlanet Planet;
			Planet.Samples.SetNum(4);
			Planet.SampleAdjacency.SetNum(4);
			Planet.Plates.SetNum(4);

			const TArray<int32> PlateIds = { 10, 11, 20, 21 };
			for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
			{
				FPlate& Plate = Planet.Plates[PlateIndex];
				Plate.Id = PlateIds[PlateIndex];
				Plate.CumulativeRotation = FQuat4d::Identity;
				Plate.MemberSamples.Reset();
			}

			Planet.Samples[0].Position = FVector3d(0.90, 0.10, 0.42).GetSafeNormal();
			Planet.Samples[1].Position = FVector3d(0.88, 0.16, 0.44).GetSafeNormal();
			Planet.Samples[2].Position = FVector3d(0.84, 0.23, 0.49).GetSafeNormal();
			Planet.Samples[3].Position = FVector3d(0.93, 0.08, 0.36).GetSafeNormal();

			const TArray<int32> SamplePlateIds = { 21, 10, 11, 20 };
			const TArray<float> SampleContinentalWeights = { 0.15f, 0.90f, 0.20f, 0.10f };
			for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
			{
				FSample& Sample = Planet.Samples[SampleIndex];
				Sample.PlateId = SamplePlateIds[SampleIndex];
				Sample.ContinentalWeight = SampleContinentalWeights[SampleIndex];
				Sample.SubductionDistanceKm = 10.0f + static_cast<float>(SampleIndex);

				FPlate* Plate = nullptr;
				for (FPlate& CandidatePlate : Planet.Plates)
				{
					if (CandidatePlate.Id == Sample.PlateId)
					{
						Plate = &CandidatePlate;
						break;
					}
				}
				if (Plate == nullptr)
				{
					continue;
				}

				Plate->MemberSamples.Add(SampleIndex);
				const int32 CarriedIndex = Plate->CarriedSamples.Num();
				FCarriedSample& CarriedSample = Plate->CarriedSamples.AddDefaulted_GetRef();
				CarriedSample.CanonicalSampleIndex = SampleIndex;
				CarriedSample.ContinentalWeight = SampleContinentalWeights[SampleIndex];
				CarriedSample.SubductionDistanceKm = 10.0f + static_cast<float>(SampleIndex);
				CarriedSample.SubductionSpeed = 1.0f + static_cast<float>(SampleIndex);
				Plate->CanonicalToCarriedIndex.Add(SampleIndex, CarriedIndex);
			}

			Planet.PendingRiftEvent = FPendingRiftEvent{};
			Planet.PendingRiftEvent.bValid = true;
			Planet.PendingRiftEvent.ChildCount = 2;
			Planet.PendingRiftEvent.ChildPlateA = 10;
			Planet.PendingRiftEvent.ChildPlateB = 11;
			Planet.PendingRiftEvent.ChildPlateIds = { 10, 11 };
			Planet.PendingRiftEvent.ChildAnchorSampleIndices = { 1, 2 };
			return Planet;
		}

		FTectonicPlanet CreateManualRiftFinalOwnerPulseDiagnosticPlanet()
		{
			FTectonicPlanet Planet;
			Planet.Samples.SetNum(5);
			Planet.SampleAdjacency.SetNum(5);
			Planet.Plates.SetNum(4);

			const TArray<int32> PlateIds = { 10, 11, 20, 21 };
			for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
			{
				FPlate& Plate = Planet.Plates[PlateIndex];
				Plate.Id = PlateIds[PlateIndex];
				Plate.CumulativeRotation = FQuat4d::Identity;
				Plate.MemberSamples.Reset();
			}

			Planet.Samples[0].Position = FVector3d(0.90, 0.10, 0.42).GetSafeNormal();
			Planet.Samples[1].Position = FVector3d(0.88, 0.16, 0.44).GetSafeNormal();
			Planet.Samples[2].Position = FVector3d(0.84, 0.23, 0.49).GetSafeNormal();
			Planet.Samples[3].Position = FVector3d(0.93, 0.08, 0.36).GetSafeNormal();
			Planet.Samples[4].Position = FVector3d(0.89, 0.15, 0.43).GetSafeNormal();

			const TArray<int32> SamplePlateIds = { 21, 10, 11, 20, 21 };
			const TArray<float> SampleContinentalWeights = { 0.15f, 0.90f, 0.20f, 0.10f, 0.20f };
			for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
			{
				FSample& Sample = Planet.Samples[SampleIndex];
				Sample.PlateId = SamplePlateIds[SampleIndex];
				Sample.ContinentalWeight = SampleContinentalWeights[SampleIndex];
				Sample.SubductionDistanceKm = 20.0f + static_cast<float>(SampleIndex);

				FPlate* Plate = nullptr;
				for (FPlate& CandidatePlate : Planet.Plates)
				{
					if (CandidatePlate.Id == Sample.PlateId)
					{
						Plate = &CandidatePlate;
						break;
					}
				}
				if (Plate == nullptr)
				{
					continue;
				}

				Plate->MemberSamples.Add(SampleIndex);
				const int32 CarriedIndex = Plate->CarriedSamples.Num();
				FCarriedSample& CarriedSample = Plate->CarriedSamples.AddDefaulted_GetRef();
				CarriedSample.CanonicalSampleIndex = SampleIndex;
				CarriedSample.ContinentalWeight = SampleContinentalWeights[SampleIndex];
				CarriedSample.SubductionDistanceKm = 20.0f + static_cast<float>(SampleIndex);
				CarriedSample.SubductionSpeed = 2.0f + static_cast<float>(SampleIndex);
				Plate->CanonicalToCarriedIndex.Add(SampleIndex, CarriedIndex);
			}

			Planet.PendingRiftEvent = FPendingRiftEvent{};
			Planet.PendingRiftEvent.bValid = true;
			Planet.PendingRiftEvent.ChildCount = 2;
			Planet.PendingRiftEvent.ChildPlateA = 10;
			Planet.PendingRiftEvent.ChildPlateB = 11;
			Planet.PendingRiftEvent.ChildPlateIds = { 10, 11 };
			Planet.PendingRiftEvent.ChildAnchorSampleIndices = { 1, 2 };
			return Planet;
		}

		FTectonicPlanet CreateManualRiftChildFragmentPlanet()
		{
		FTectonicPlanet Planet;
		Planet.Samples.SetNum(8);
		Planet.SampleAdjacency.SetNum(8);

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(2, 4);
		AddUndirectedEdge(2, 6);
		AddUndirectedEdge(3, 5);
		AddUndirectedEdge(3, 7);

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			Planet.Samples[SampleIndex].PlateId =
				SampleIndex <= 3 ? 10 : (SampleIndex <= 5 ? 11 : (SampleIndex == 6 ? 20 : 21));
		}

		Planet.PendingRiftEvent = FPendingRiftEvent{};
		Planet.PendingRiftEvent.bValid = true;
		Planet.PendingRiftEvent.ChildCount = 2;
		Planet.PendingRiftEvent.ChildPlateA = 10;
		Planet.PendingRiftEvent.ChildPlateB = 11;
		Planet.PendingRiftEvent.ChildPlateIds = { 10, 11 };
		Planet.PendingRiftEvent.ChildAnchorSampleIndices = { 0, 4 };
		return Planet;
	}

	FTectonicPlanet CreateManualRiftFragmentRegressionPlanet()
	{
		FTectonicPlanet Planet;
		Planet.Samples.SetNum(12);
		Planet.SampleAdjacency.SetNum(12);

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(5, 6);
		AddUndirectedEdge(2, 5);
		AddUndirectedEdge(2, 9);
		AddUndirectedEdge(3, 6);
		AddUndirectedEdge(3, 10);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(4, 11);
		AddUndirectedEdge(7, 0);
		AddUndirectedEdge(7, 9);
		AddUndirectedEdge(8, 1);
		AddUndirectedEdge(8, 10);

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			Planet.Samples[SampleIndex].PlateId =
				SampleIndex <= 4 ? 10 : (SampleIndex <= 8 ? 11 : (10 + SampleIndex));
		}

		Planet.PendingRiftEvent = FPendingRiftEvent{};
		Planet.PendingRiftEvent.bValid = true;
		Planet.PendingRiftEvent.ChildCount = 2;
		Planet.PendingRiftEvent.ChildPlateA = 10;
		Planet.PendingRiftEvent.ChildPlateB = 11;
		Planet.PendingRiftEvent.ChildPlateIds = { 10, 11 };
		Planet.PendingRiftEvent.ChildAnchorSampleIndices = { 0, 5 };
		return Planet;
	}

	FTectonicPlanet CreateManualRiftZeroAdjacencySiblingFallbackPlanet()
	{
		FTectonicPlanet Planet;
		Planet.Samples.SetNum(4);
		Planet.SampleAdjacency.SetNum(4);

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(1, 3);

		const TArray<int32> PlateIds = { 10, 11, 11, 20 };
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			Planet.Samples[SampleIndex].PlateId = PlateIds[SampleIndex];
		}

		Planet.PendingRiftEvent = FPendingRiftEvent{};
		Planet.PendingRiftEvent.bValid = true;
		Planet.PendingRiftEvent.ChildCount = 2;
		Planet.PendingRiftEvent.ChildPlateA = 10;
		Planet.PendingRiftEvent.ChildPlateB = 11;
		Planet.PendingRiftEvent.ChildPlateIds = { 10, 11 };
		Planet.PendingRiftEvent.ChildAnchorSampleIndices = { 0, 2 };
		return Planet;
	}

	FTectonicPlanet CreateManualRiftStep38StylePlanet()
	{
		FTectonicPlanet Planet;
		Planet.Samples.SetNum(10);
		Planet.SampleAdjacency.SetNum(10);

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(1, 6);
		AddUndirectedEdge(2, 7);
		AddUndirectedEdge(3, 8);
		AddUndirectedEdge(4, 9);

		const TArray<int32> PlateIds = { 10, 11, 11, 11, 11, 11, 20, 21, 22, 23 };
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			Planet.Samples[SampleIndex].PlateId = PlateIds[SampleIndex];
		}

		Planet.PendingRiftEvent = FPendingRiftEvent{};
		Planet.PendingRiftEvent.bValid = true;
		Planet.PendingRiftEvent.ChildCount = 2;
		Planet.PendingRiftEvent.ChildPlateA = 10;
		Planet.PendingRiftEvent.ChildPlateB = 11;
		Planet.PendingRiftEvent.ChildPlateIds = { 10, 11 };
		Planet.PendingRiftEvent.ChildAnchorSampleIndices = { 0, 5 };
		return Planet;
	}

	FTectonicPlanet CreateManualCollisionReceiverFragmentPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		Planet.GeometricCollisionDonorLocalityClampKm = 400.0;
		Planet.Samples.SetNum(9);
		Planet.SampleAdjacency.SetNum(9);
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5), FIntVector(6, 7, 8) };
		Planet.TriangleAdjacency.SetNum(3);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
			Planet.Plates[PlateIndex].RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Planet.Plates[PlateIndex].AngularSpeed = 0.02;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(0.58, 0.22, 0.20).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.22, 0.58, 0.20).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.20, 0.22, 0.58).GetSafeNormal();
		Planet.Samples[6].Position = FVector3d(0.63, 0.27, 0.14).GetSafeNormal();
		Planet.Samples[7].Position = FVector3d(0.27, 0.63, 0.14).GetSafeNormal();
		Planet.Samples[8].Position = FVector3d(0.14, 0.27, 0.63).GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = SampleIndex < 3 ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		AddUndirectedEdge(6, 7);
		AddUndirectedEdge(7, 8);
		AddUndirectedEdge(8, 6);
		for (const int32 DonorSeedIndex : { 3, 4, 5 })
		{
			AddUndirectedEdge(0, DonorSeedIndex);
			AddUndirectedEdge(1, DonorSeedIndex);
			AddUndirectedEdge(2, DonorSeedIndex);
		}

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[1].MemberSamples = { 3, 4, 5, 6, 7, 8 };
		Planet.Plates[1].SoupTriangles = { 1, 2 };
		Planet.Plates[1].OverlapScore = 50;
		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5, 6, 7, 8 }, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualGeometricCollisionInfluenceGainPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		Planet.Samples.SetNum(12);
		Planet.SampleAdjacency.SetNum(12);
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(2);
		Planet.Plates.SetNum(2);
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
			Planet.Plates[PlateIndex].RotationAxis = FVector3d(0.0, 0.0, 1.0);
			Planet.Plates[PlateIndex].AngularSpeed = 0.02;
		}

		Planet.Samples[0].Position = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		Planet.Samples[1].Position = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		Planet.Samples[2].Position = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		Planet.Samples[3].Position = FVector3d(0.60, 0.20, 0.20).GetSafeNormal();
		Planet.Samples[4].Position = FVector3d(0.20, 0.60, 0.20).GetSafeNormal();
		Planet.Samples[5].Position = FVector3d(0.20, 0.20, 0.60).GetSafeNormal();

		const FVector3d CollisionCenter =
			(Planet.Samples[3].Position + Planet.Samples[4].Position + Planet.Samples[5].Position).GetSafeNormal();
		FVector3d TangentX = FVector3d::CrossProduct(CollisionCenter, FVector3d(0.0, 0.0, 1.0));
		if (TangentX.IsNearlyZero())
		{
			TangentX = FVector3d::CrossProduct(CollisionCenter, FVector3d(0.0, 1.0, 0.0));
		}
		TangentX = TangentX.GetSafeNormal();
		const FVector3d TangentY = FVector3d::CrossProduct(CollisionCenter, TangentX).GetSafeNormal();
		const auto MakeOffsetPoint = [&CollisionCenter, &TangentX, &TangentY](const double DistanceRad, const double AzimuthRad)
		{
			const FVector3d TangentDirection =
				((TangentX * FMath::Cos(AzimuthRad)) + (TangentY * FMath::Sin(AzimuthRad))).GetSafeNormal();
			return ((CollisionCenter * FMath::Cos(DistanceRad)) + (TangentDirection * FMath::Sin(DistanceRad))).GetSafeNormal();
		};

		Planet.Samples[6].Position = MakeOffsetPoint(0.074, 0.0);
		Planet.Samples[7].Position = MakeOffsetPoint(0.076, PI * 0.5);
		Planet.Samples[8].Position = MakeOffsetPoint(0.079, PI);
		Planet.Samples[9].Position = MakeOffsetPoint(0.082, PI * 1.5);
		Planet.Samples[10].Position = MakeOffsetPoint(0.085, PI * 0.25);
		Planet.Samples[11].Position = MakeOffsetPoint(0.088, PI * 1.25);

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = (SampleIndex < 3 || SampleIndex >= 6) ? 0 : 1;
			Sample.ContinentalWeight = SampleIndex >= 6 ? 0.6f : 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		for (int32 HaloSampleIndex = 6; HaloSampleIndex < 12; ++HaloSampleIndex)
		{
			AddUndirectedEdge(0, HaloSampleIndex);
			AddUndirectedEdge(1, HaloSampleIndex);
			AddUndirectedEdge(2, HaloSampleIndex);
			if (HaloSampleIndex > 6)
			{
				AddUndirectedEdge(HaloSampleIndex - 1, HaloSampleIndex);
			}
		}
		for (const int32 DonorSeedIndex : { 3, 4, 5 })
		{
			AddUndirectedEdge(0, DonorSeedIndex);
			AddUndirectedEdge(1, DonorSeedIndex);
			AddUndirectedEdge(2, DonorSeedIndex);
		}

		Planet.Plates[0].MemberSamples = { 0, 1, 2, 6, 7, 8, 9, 10, 11 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[1].MemberSamples = { 3, 4, 5 };
		Planet.Plates[1].SoupTriangles = { 1 };
		Planet.Plates[1].OverlapScore = 50;

		Planet.BuildContainmentSoups();
		ConfigureManualTriangleCarriedSamples(Planet, 0, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	FPendingGeometricCollisionEvent MakeManualPendingGeometricCollisionEvent(
		const FTectonicPlanet& Planet,
		const int32 PlateA,
		const int32 PlateB,
		const TArray<int32>& SamplesFromAInsideB,
		const TArray<int32>& SamplesFromBInsideA)
	{
		FPendingGeometricCollisionEvent Event;
		Event.bValid = true;
		Event.PlateA = PlateA;
		Event.PlateB = PlateB;
		Event.SamplesFromAInsideB = SamplesFromAInsideB;
		Event.SamplesFromBInsideA = SamplesFromBInsideA;
		Event.OverlapSampleIndices = SamplesFromAInsideB;
		for (const int32 SampleIndex : SamplesFromBInsideA)
		{
			Event.OverlapSampleIndices.AddUnique(SampleIndex);
		}
		Event.OverlapSampleCount = Event.OverlapSampleIndices.Num();
		FVector3d ContactCenterSum = FVector3d::ZeroVector;
		for (const int32 SampleIndex : Event.OverlapSampleIndices)
		{
			if (Planet.Samples.IsValidIndex(SampleIndex))
			{
				ContactCenterSum += Planet.Samples[SampleIndex].Position.GetSafeNormal();
			}
		}
		Event.ContactCenter = ContactCenterSum.GetSafeNormal();
		Event.ObservationCount = 2;
		Event.EffectiveConvergenceKmPerMy = 100.0;
		Event.AccumulatedPenetrationKm = Planet.GeometricCollisionMinPersistentPenetrationKm + 100.0;
		return Event;
	}

	FTectonicPlanet CreateManualDirectionalGeometricCollisionOverlapPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(12);
		Planet.SampleAdjacency.SetNum(12);
		Planet.TriangleIndices = { FIntVector(0, 1, 2), FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(2);
		Planet.Plates.SetNum(2);
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
		}

		const FVector3d A0 = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		const FVector3d A1 = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		const FVector3d A2 = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		const FVector3d B0 = FVector3d(1.0, 1.0, 0.2).GetSafeNormal();
		const FVector3d B1 = FVector3d(1.0, 0.2, 1.0).GetSafeNormal();
		const FVector3d B2 = FVector3d(0.2, 1.0, 1.0).GetSafeNormal();
		const auto MakeInteriorPoint = [](const FVector3d& V0, const FVector3d& V1, const FVector3d& V2, const double W0, const double W1, const double W2)
		{
			return ((V0 * W0) + (V1 * W1) + (V2 * W2)).GetSafeNormal();
		};

		Planet.Samples[0].Position = A0;
		Planet.Samples[1].Position = A1;
		Planet.Samples[2].Position = A2;
		Planet.Samples[3].Position = B0;
		Planet.Samples[4].Position = B1;
		Planet.Samples[5].Position = B2;
		Planet.Samples[6].Position = MakeInteriorPoint(B0, B1, B2, 0.6, 0.2, 0.2);
		Planet.Samples[7].Position = MakeInteriorPoint(B0, B1, B2, 0.2, 0.6, 0.2);
		Planet.Samples[8].Position = MakeInteriorPoint(B0, B1, B2, 0.2, 0.2, 0.6);
		Planet.Samples[9].Position = MakeInteriorPoint(A0, A1, A2, 0.6, 0.2, 0.2);
		Planet.Samples[10].Position = MakeInteriorPoint(A0, A1, A2, 0.2, 0.6, 0.2);
		Planet.Samples[11].Position = MakeInteriorPoint(A0, A1, A2, 0.2, 0.2, 0.6);

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = (SampleIndex < 3 || (SampleIndex >= 6 && SampleIndex < 9)) ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		AddUndirectedEdge(6, 7);
		AddUndirectedEdge(7, 8);
		AddUndirectedEdge(8, 6);
		AddUndirectedEdge(9, 10);
		AddUndirectedEdge(10, 11);
		AddUndirectedEdge(11, 9);
		for (int32 Plate0SeedIndex = 6; Plate0SeedIndex < 9; ++Plate0SeedIndex)
		{
			AddUndirectedEdge(Plate0SeedIndex, 0);
			AddUndirectedEdge(Plate0SeedIndex, 1);
			AddUndirectedEdge(Plate0SeedIndex, 2);
		}
		for (int32 Plate1SeedIndex = 9; Plate1SeedIndex < 12; ++Plate1SeedIndex)
		{
			AddUndirectedEdge(Plate1SeedIndex, 3);
			AddUndirectedEdge(Plate1SeedIndex, 4);
			AddUndirectedEdge(Plate1SeedIndex, 5);
		}

		for (int32 Plate0SampleIndex = 0; Plate0SampleIndex < 9; ++Plate0SampleIndex)
		{
			for (int32 Plate1SampleIndex = 9; Plate1SampleIndex < 12; ++Plate1SampleIndex)
			{
				AddUndirectedEdge(Plate0SampleIndex, Plate1SampleIndex);
			}
		}

		for (int32 Plate0SeedIndex = 6; Plate0SeedIndex < 9; ++Plate0SeedIndex)
		{
			AddUndirectedEdge(Plate0SeedIndex, 3);
			AddUndirectedEdge(Plate0SeedIndex, 4);
			AddUndirectedEdge(Plate0SeedIndex, 5);
		}
		for (int32 Plate1SeedIndex = 9; Plate1SeedIndex < 12; ++Plate1SeedIndex)
		{
			AddUndirectedEdge(Plate1SeedIndex, 0);
			AddUndirectedEdge(Plate1SeedIndex, 1);
			AddUndirectedEdge(Plate1SeedIndex, 2);
		}

		Planet.Plates[0].MemberSamples = { 0, 1, 2, 6, 7, 8 };
		Planet.Plates[0].SoupTriangles = { 0 };
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
		Planet.Plates[0].AngularSpeed = 0.02;

		Planet.Plates[1].MemberSamples = { 3, 4, 5, 9, 10, 11 };
		Planet.Plates[1].SoupTriangles = { 1 };
		Planet.Plates[1].OverlapScore = 50;
		Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
		Planet.Plates[1].AngularSpeed = 0.02;

		Planet.BuildContainmentSoups();
		ConfigureManualCarriedSamplesForVertices(Planet, 0, { 0, 1, 2 }, 1.0f, 1.0f);
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
		return Planet;
	}

	FTectonicPlanet CreateManualOppositeBucketOnlyGeometricCollisionOverlapPlanet()
	{
		FTectonicPlanet Planet;
		Planet.PlanetRadiusKm = TestPlanetRadiusKm;
		Planet.Samples.SetNum(7);
		Planet.SampleAdjacency.SetNum(7);
		Planet.TriangleIndices = { FIntVector(3, 4, 5) };
		Planet.TriangleAdjacency.SetNum(1);
		Planet.Plates.SetNum(2);
		Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
		for (int32 PlateIndex = 0; PlateIndex < Planet.Plates.Num(); ++PlateIndex)
		{
			Planet.Plates[PlateIndex].Id = PlateIndex;
			Planet.Plates[PlateIndex].CumulativeRotation = FQuat4d::Identity;
		}

		const FVector3d B0 = FVector3d(1.0, 0.0, 0.0).GetSafeNormal();
		const FVector3d B1 = FVector3d(0.0, 1.0, 0.0).GetSafeNormal();
		const FVector3d B2 = FVector3d(0.0, 0.0, 1.0).GetSafeNormal();
		const auto MakeInteriorPoint = [](const FVector3d& V0, const FVector3d& V1, const FVector3d& V2, const double W0, const double W1, const double W2)
		{
			return ((V0 * W0) + (V1 * W1) + (V2 * W2)).GetSafeNormal();
		};

		Planet.Samples[0].Position = MakeInteriorPoint(B0, B1, B2, 0.6, 0.2, 0.2);
		Planet.Samples[1].Position = MakeInteriorPoint(B0, B1, B2, 0.2, 0.6, 0.2);
		Planet.Samples[2].Position = MakeInteriorPoint(B0, B1, B2, 0.2, 0.2, 0.6);
		Planet.Samples[3].Position = B0;
		Planet.Samples[4].Position = B1;
		Planet.Samples[5].Position = B2;
		Planet.Samples[6].Position = FVector3d(-1.0, 1.0, 1.0).GetSafeNormal();

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			FSample& Sample = Planet.Samples[SampleIndex];
			Sample.PlateId = SampleIndex < 3 ? 0 : 1;
			Sample.ContinentalWeight = 1.0f;
			Sample.Elevation = 1.0f;
			Sample.Thickness = 35.0f;
		}

		auto AddUndirectedEdge = [&Planet](const int32 A, const int32 B)
		{
			Planet.SampleAdjacency[A].AddUnique(B);
			Planet.SampleAdjacency[B].AddUnique(A);
		};

		AddUndirectedEdge(0, 1);
		AddUndirectedEdge(1, 2);
		AddUndirectedEdge(2, 0);
		AddUndirectedEdge(3, 4);
		AddUndirectedEdge(4, 5);
		AddUndirectedEdge(5, 3);
		AddUndirectedEdge(3, 6);
		AddUndirectedEdge(4, 6);
		AddUndirectedEdge(5, 6);
		for (int32 Plate0SampleIndex = 0; Plate0SampleIndex < 3; ++Plate0SampleIndex)
		{
			for (int32 Plate1SampleIndex = 3; Plate1SampleIndex < 6; ++Plate1SampleIndex)
			{
				AddUndirectedEdge(Plate0SampleIndex, Plate1SampleIndex);
			}
		}

		Planet.Plates[0].MemberSamples = { 0, 1, 2 };
		Planet.Plates[0].SoupTriangles.Reset();
		Planet.Plates[0].OverlapScore = 100;
		Planet.Plates[0].RotationAxis = FVector3d(0.0, 0.0, 1.0);
		Planet.Plates[0].AngularSpeed = 0.02;

		Planet.Plates[1].MemberSamples = { 3, 4, 5, 6 };
		Planet.Plates[1].SoupTriangles = { 0 };
		Planet.Plates[1].OverlapScore = 50;
		Planet.Plates[1].RotationAxis = FVector3d(0.0, 0.0, -1.0);
		Planet.Plates[1].AngularSpeed = 0.02;

		Planet.BuildContainmentSoups();
		ConfigureManualCarriedSamplesForVertices(Planet, 1, { 3, 4, 5 }, 1.0f, 1.0f);
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
		TArray<uint8> PreserveOwnershipCWRetainFlags;
		TArray<uint8> PreserveOwnershipFallbackQueryFlags;
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
			OwnershipMode,
			&Result.PreserveOwnershipCWRetainFlags,
			&Result.PreserveOwnershipFallbackQueryFlags);
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
		if (Mode == ETectonicMapExportMode::All || Mode == ETectonicMapExportMode::Elevation)
		{
			Test.TestTrue(
				*FString::Printf(TEXT("Step %d enhanced elevation export exists"), Step),
				PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("ElevationEnhanced.png"))));
		}
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
		if (Mode == ETectonicMapExportMode::All || Mode == ETectonicMapExportMode::Elevation)
		{
			Test.TestTrue(
				*FString::Printf(TEXT("%s enhanced elevation export exists"), *Label),
				PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("ElevationEnhanced.png"))));
		}
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

	bool ExportM6BaselineCheckpointMaps(
		FAutomationTestBase& Test,
		const FTectonicPlanet& Planet,
		const FString& BaseDirectory,
		const int32 Step)
	{
		const FString OutputDirectory = FPaths::Combine(BaseDirectory, FString::Printf(TEXT("Step_%d"), Step));
		const ETectonicMapExportMode Modes[] = {
			ETectonicMapExportMode::Elevation,
			ETectonicMapExportMode::PlateId,
			ETectonicMapExportMode::CrustType,
			ETectonicMapExportMode::ContinentalWeight,
			ETectonicMapExportMode::BoundaryMask
		};

		for (const ETectonicMapExportMode Mode : Modes)
		{
			FTectonicMollweideExportOptions Options;
			Options.Mode = Mode;
			Options.Width = TestExportWidth;
			Options.Height = TestExportHeight;
			Options.OutputDirectory = OutputDirectory;

			FTectonicMollweideExportStats Stats;
			FString Error;
			const bool bExported = TectonicMollweideExporter::ExportPlanet(Planet, Options, Stats, Error);
			Test.TestTrue(
				*FString::Printf(TEXT("M6 step %d %s export succeeded (%s)"), Step, TectonicPlanetVisualization::GetExportModeName(Mode), *Error),
				bExported);
			if (!bExported)
			{
				return false;
			}
		}

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		const TCHAR* ExpectedFiles[] = {
			TEXT("Elevation.png"),
			TEXT("PlateId.png"),
			TEXT("CrustType.png"),
			TEXT("ContinentalWeight.png"),
			TEXT("BoundaryMask.png")
		};
		for (const TCHAR* ExpectedFile : ExpectedFiles)
		{
			Test.TestTrue(
				*FString::Printf(TEXT("M6 step %d export exists: %s"), Step, ExpectedFile),
				PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, ExpectedFile)));
		}

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

	struct FM6CheckpointSnapshot
	{
		int32 Step = 0;
		TArray<int32> ResamplingSteps;
		EResampleTriggerReason LastTriggerReason = EResampleTriggerReason::None;
		EResampleOwnershipMode LastOwnershipMode = EResampleOwnershipMode::FullResolution;
		int32 PlateCount = 0;
		int32 ContinentalCount = 0;
		int32 BoundarySampleCount = 0;
		int32 GapCount = 0;
		int32 OverlapCount = 0;
		int32 ContinentalContinentalOverlapCount = 0;
		int32 MaxComponentsPerPlate = 0;
		int32 PreserveSamePlateHitCount = 0;
		int32 PreserveSamePlateRecoveryCount = 0;
		int32 PreserveFallbackQueryCount = 0;
		int32 PreservePlateChangedCount = 0;
		double ContinentalAreaFraction = 0.0;
		double BoundarySampleFraction = 0.0;
		double GapRate = 0.0;
		double OverlapRate = 0.0;
		double ElevationMinKm = 0.0;
		double ElevationMaxKm = 0.0;
		double ElevationMeanKm = 0.0;
	};

	FM6CheckpointSnapshot BuildM6CheckpointSnapshot(const FTectonicPlanet& Planet)
	{
		FM6CheckpointSnapshot Snapshot;
		Snapshot.Step = Planet.CurrentStep;
		Snapshot.ResamplingSteps = Planet.ResamplingSteps;
		Snapshot.LastTriggerReason = Planet.LastResampleTriggerReason;
		Snapshot.LastOwnershipMode = Planet.LastResampleOwnershipMode;
		Snapshot.PlateCount = Planet.Plates.Num();
		Snapshot.ContinentalCount = CountContinentalSamples(Planet);
		Snapshot.BoundarySampleCount = CountBoundarySamples(Planet);

		if (!Planet.Samples.IsEmpty())
		{
			double ElevationSum = 0.0;
			Snapshot.ElevationMinKm = TNumericLimits<double>::Max();
			Snapshot.ElevationMaxKm = -TNumericLimits<double>::Max();
			for (const FSample& Sample : Planet.Samples)
			{
				const double ElevationKm = static_cast<double>(Sample.Elevation);
				Snapshot.ElevationMinKm = FMath::Min(Snapshot.ElevationMinKm, ElevationKm);
				Snapshot.ElevationMaxKm = FMath::Max(Snapshot.ElevationMaxKm, ElevationKm);
				ElevationSum += ElevationKm;
			}

			const double SampleCount = static_cast<double>(Planet.Samples.Num());
			Snapshot.ContinentalAreaFraction = static_cast<double>(Snapshot.ContinentalCount) / SampleCount;
			Snapshot.BoundarySampleFraction = static_cast<double>(Snapshot.BoundarySampleCount) / SampleCount;
			Snapshot.ElevationMeanKm = ElevationSum / SampleCount;
		}

		for (const FPlate& Plate : Planet.Plates)
		{
			Snapshot.MaxComponentsPerPlate = FMath::Max(
				Snapshot.MaxComponentsPerPlate,
				BuildPlateCoherenceSnapshot(Planet, Plate.Id).ConnectedComponentCount);
		}

		if (!Planet.ResamplingHistory.IsEmpty())
		{
			const FResamplingStats& Stats = Planet.LastResamplingStats;
			Snapshot.GapCount = Stats.GapCount;
			Snapshot.OverlapCount = Stats.OverlapCount;
			Snapshot.ContinentalContinentalOverlapCount = Stats.ContinentalContinentalOverlapCount;
			Snapshot.PreserveSamePlateHitCount = Stats.PreserveOwnershipSamePlateHitCount;
			Snapshot.PreserveSamePlateRecoveryCount = Stats.PreserveOwnershipSamePlateRecoveryCount;
			Snapshot.PreserveFallbackQueryCount = Stats.PreserveOwnershipFallbackQueryCount;
			Snapshot.PreservePlateChangedCount = Stats.PreserveOwnershipPlateChangedCount;
			if (!Planet.Samples.IsEmpty())
			{
				const double SampleCount = static_cast<double>(Planet.Samples.Num());
				Snapshot.GapRate = static_cast<double>(Snapshot.GapCount) / SampleCount;
				Snapshot.OverlapRate = static_cast<double>(Snapshot.OverlapCount) / SampleCount;
			}
		}

		return Snapshot;
	}

	FString FormatStepList(const TArray<int32>& Steps)
	{
		TArray<FString> StepStrings;
		StepStrings.Reserve(Steps.Num());
		for (const int32 Step : Steps)
		{
			StepStrings.Add(FString::FromInt(Step));
		}

		return FString::Printf(TEXT("[%s]"), *FString::Join(StepStrings, TEXT(",")));
	}

	struct FM6BaselineRunConfig
	{
		int32 Seed = TestRandomSeed;
		int32 PlateCount = TestPlateCount;
		int32 SampleCount = TestSampleCount;
		int32 MaxStep = 200;
		FTectonicPlanetRuntimeConfig RuntimeConfig = GetM6BaselineRuntimeConfig();
		FString PresetLabel = TEXT("M6Baseline");
		double AutomaticRiftBaseRatePerMy = GetM6BaselineRuntimeConfig().AutomaticRiftBaseRatePerMy;
		double AndeanContinentalConversionRatePerMy = GetM6BaselineRuntimeConfig().AndeanContinentalConversionRatePerMy;
		FString RunId;
		FString ExportRoot;
		bool bExportCheckpoints = false;
		bool bLogCheckpoints = false;
		bool bLogSummary = true;
	};

	struct FM6BaselineRunSummary
	{
		FString RunId;
		int32 Seed = 0;
		double AndeanContinentalConversionRatePerMy = 0.0;
		FM6CheckpointSnapshot InitialSnapshot;
		FM6CheckpointSnapshot Step100Snapshot;
		FM6CheckpointSnapshot Step200Snapshot;
		FM6CheckpointSnapshot Step300Snapshot;
		FM6CheckpointSnapshot Step400Snapshot;
		FM6CheckpointSnapshot FinalSnapshot;
		FResamplingStats Step229RiftFollowupStats;
		bool bHasStep100Snapshot = false;
		bool bHasStep200Snapshot = false;
		bool bHasStep300Snapshot = false;
		bool bHasStep400Snapshot = false;
		bool bHasStep229RiftFollowupStats = false;
		bool bStablePlateIdsValid = false;
		int32 TotalResamples = 0;
		int32 TotalCollisionEvents = 0;
		int32 TotalRiftEvents = 0;
		int32 FirstCollisionStep = INDEX_NONE;
		int32 FirstRiftStep = INDEX_NONE;
		int32 MinPlateCount = 0;
		int32 MaxPlateCount = 0;
		int32 FinalPlateCount = 0;
		int32 MaxComponentsPerPlateObserved = 0;
		int32 OwnershipChangesOutsideResamplingCount = 0;
		int32 StepsWithOwnershipChangesOutsideResampling = 0;
		double MinContinentalAreaFraction = 0.0;
		double MaxContinentalAreaFraction = 0.0;
		double FinalContinentalAreaFraction = 0.0;
		double MinBoundarySampleFraction = 0.0;
		double MaxBoundarySampleFraction = 0.0;
		double FinalBoundarySampleFraction = 0.0;
		double MinGapRate = 0.0;
		double MaxGapRate = 0.0;
		double FinalGapRate = 0.0;
		double MinOverlapRate = 0.0;
		double MaxOverlapRate = 0.0;
		double FinalOverlapRate = 0.0;
		double MinMeanElevation = 0.0;
		double MaxMeanElevation = 0.0;
		double FinalMeanElevation = 0.0;
		int64 TotalContinentalSamplesLost = 0;
		int64 TotalFormerContinentalSamplesTurnedOceanic = 0;
		int64 TotalFormerContinentalDivergentGaps = 0;
		int64 TotalFormerContinentalNonDivergentGaps = 0;
		int64 TotalFormerContinentalNonDivergentGapProjectionResolved = 0;
		int64 TotalFormerContinentalNonDivergentGapNearestCopyResolved = 0;
		int64 TotalFormerContinentalNonDivergentFallbackOceanized = 0;
		int64 TotalFormerContinentalDivergentOceanized = 0;
		int64 TotalFormerContinentalProjectionRecoveredNonContinentalFinal = 0;
		int64 TotalFormerContinentalProjectionRecoveredSamePlateNonContinentalFinal = 0;
		int64 TotalFormerContinentalProjectionRecoveredChangedPlateNonContinentalFinal = 0;
		int64 TotalFormerContinentalProjectionRecoveredPreserveModeNonContinentalFinal = 0;
		int64 TotalFormerContinentalProjectionRecoveredFullResolutionNonContinentalFinal = 0;
		int64 TotalFormerContinentalNearestCopyRecoveredNonContinentalFinal = 0;
		int64 TotalFormerContinentalNonGapReclassifiedNonContinentalFinal = 0;
		int64 TotalFormerContinentalChangedPlateNonContinentalFinal = 0;
		int64 TotalFormerContinentalFullResolutionNonContinentalFinal = 0;
		int64 TotalFullResolutionSamePlateNonContinentalFinal = 0;
		int64 TotalFullResolutionChangedPlateNonContinentalFinal = 0;
		int64 TotalFullResolutionCollisionFollowupSamePlateNonContinental = 0;
		int64 TotalFullResolutionCollisionFollowupChangedPlateNonContinental = 0;
		int64 TotalFullResolutionRiftFollowupSamePlateNonContinental = 0;
		int64 TotalFullResolutionRiftFollowupChangedPlateNonContinental = 0;
		int64 TotalFullResolutionOtherTriggerSamePlateNonContinental = 0;
		int64 TotalFullResolutionOtherTriggerChangedPlateNonContinental = 0;
		int64 TotalFormerContinentalPreserveModeNonGapNonContinentalFinal = 0;
		int64 TotalFormerContinentalFallbackQueryNonContinentalFinal = 0;
		int64 TotalPreserveOwnershipFallbackSamePlateRecontainedCount = 0;
		int64 TotalPreserveOwnershipFallbackSamePlateRetainedCount = 0;
		int64 TotalPreserveOwnershipFallbackChangedOwnerCount = 0;
		int64 TotalPreserveOwnershipFallbackGapCount = 0;
		int64 TotalPreserveOwnershipFallbackDivergentOceanizationCount = 0;
		int64 TotalPreserveOwnershipContinentalLossCountAfterFallback = 0;
		int64 TotalPreserveOwnershipPreviousOwnerHysteresisApplicationCount = 0;
		int64 TotalPreserveOwnershipStronglyContinentalBoundarySavedCount = 0;
		int64 TotalPreserveOwnershipFallbackChangedOwnerNonGapLossCount = 0;
		int64 TotalPreserveOwnershipFallbackDivergentLossCount = 0;
		int64 TotalPreserveOwnershipFallbackNonDivergentProjectionLossCount = 0;
		int64 TotalPreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount = 0;
		int64 TotalPreserveOwnershipFallbackStrongLossGE090Count = 0;
		int64 TotalPreserveOwnershipFallbackStrongLossGE075Count = 0;
		int64 TotalPreserveOwnershipFallbackStrongLossGE050Count = 0;
		int64 TotalFullResolutionSamePlateCWRetained = 0;
		int64 TotalFullResolutionSamePlateCWThresholdCrossingPrevented = 0;
		int64 TotalFullResolutionCollisionFollowupSamePlateCWRetained = 0;
		int64 TotalFullResolutionRiftFollowupSamePlateCWRetained = 0;
		int64 TotalFullResolutionOtherTriggerSamePlateCWRetained = 0;
		int64 TotalAndeanContinentalGains = 0;
		int64 TotalCollisionContinentalGains = 0;
		int64 TotalNetContinentalSampleDelta = 0;
		int64 FirstHalfContinentalSamplesLost = 0;
		int64 SecondHalfContinentalSamplesLost = 0;
		int64 FirstHalfAndeanContinentalGains = 0;
		int64 SecondHalfAndeanContinentalGains = 0;
		int64 FirstHalfCollisionContinentalGains = 0;
		int64 SecondHalfCollisionContinentalGains = 0;
		int64 FirstHalfNetContinentalSampleDelta = 0;
		int64 SecondHalfNetContinentalSampleDelta = 0;
		int64 TotalCollisionFollowupComponentIncrease = 0;
		int64 TotalRiftFollowupComponentIncrease = 0;
		int64 CollisionFollowupComponentIncrease0To100 = 0;
		int64 CollisionFollowupComponentIncrease100To200 = 0;
		int64 CollisionFollowupComponentIncrease200To400 = 0;
		int64 RiftFollowupComponentIncrease0To100 = 0;
		int64 RiftFollowupComponentIncrease100To200 = 0;
		int64 RiftFollowupComponentIncrease200To400 = 0;
		int64 TotalRiftLocalizedNonChildSampleRestoreCount = 0;
		int64 TotalRiftLocalizedGapSampleRestoreCount = 0;
		int64 TotalRiftLocalizedCWRestoredCount = 0;
			int64 TotalRiftLocalizedCWPhantomPreventedCount = 0;
			int64 TotalRiftLocalizedCWContinentalPreventedCount = 0;
			int64 TotalRiftInterpolationCreatedGainCount = 0;
			int64 TotalRiftFinalOwnerMismatchGainCountAfterLocalization = 0;
			int64 TotalRiftFinalOwnerCWReconciledCount = 0;
			int64 TotalRiftFinalOwnerMismatchGainCountBeforeReconciliation = 0;
			int64 TotalRiftFinalOwnerMismatchGainCountAfterReconciliation = 0;
			int64 TotalRiftFinalOwnerMismatchContinentalPreventedCount = 0;
			int64 TotalRiftSameOwnerChildInterpolationGainCountBeforeReconciliation = 0;
			int64 TotalRiftSameOwnerChildInterpolationGainCountAfterReconciliation = 0;
			int64 TotalRiftFinalGainStartedBelow025Count = 0;
			int64 TotalRiftFinalGainStartedBelow040Count = 0;
			int64 TotalRiftFinalGainStartedBelow050Count = 0;
			int64 TotalRiftChildStrayFragmentDetectedCount = 0;
		int64 TotalRiftChildStrayFragmentReassignedCount = 0;
		int32 MaxRiftLargestStrayChildFragmentSize = 0;
		int32 TotalRiftStrayFragmentReassignedToSiblingCount = 0;
		int32 TotalRiftStrayFragmentReassignedToOtherNeighborCount = 0;
		int32 TotalRiftStrayFragmentReassignedByAdjacentSiblingCount = 0;
		int32 TotalRiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount = 0;
		int32 TotalRiftStrayFragmentForcedNonChildAssignmentCount = 0;
		int32 TotalRiftStrayFragmentZeroSiblingAdjacencyCount = 0;
		int32 TotalRiftStrayFragmentPositiveSiblingAdjacencyCount = 0;
		int32 TotalRiftStrayFragmentRecipientCandidateConsideredCount = 0;
		int32 TotalRiftStrayFragmentRecipientIncoherenceRejectedCount = 0;
		int32 TotalRiftStrayFragmentIncoherentForcedAssignmentCount = 0;
		int32 MaxRiftLargestFragmentCausingRecipientGrowthSize = 0;
		int32 RiftFollowupCount0To100 = 0;
		int32 RiftFollowupCount100To200 = 0;
		int32 RiftFollowupCount200To400 = 0;
		int32 CollisionFollowupConsolidationEventCount = 0;
		int32 RiftFollowupConsolidationEventCount = 0;
		int64 CollisionFollowupAffectedPlateComponentDecrease = 0;
		int64 RiftFollowupAffectedPlateComponentDecrease = 0;
		int64 TotalCollisionProposedDonorComponentIncrease = 0;
		int64 TotalCollisionProposedReceiverComponentIncrease = 0;
		int64 TotalCollisionAppliedDonorComponentIncrease = 0;
		int64 TotalCollisionAppliedReceiverComponentIncrease = 0;
		int32 TotalCollisionTransferDiagnosticsCount = 0;
		int64 TotalCollisionProposedTerraneSampleCount = 0;
		int32 MaxCollisionProposedTerraneSampleCount = 0;
		int64 TotalCollisionAcceptedTerraneSampleCount = 0;
		int32 MaxCollisionAcceptedTerraneSampleCount = 0;
		double TotalCollisionTransferTrimRatio = 0.0;
		double MaxCollisionTransferTrimRatio = 0.0;
		int64 TotalCollisionTransferProposedDonorComponentIncrease = 0;
		int32 MaxCollisionTransferProposedDonorComponentIncrease = 0;
		int64 TotalCollisionTransferAcceptedDonorComponentIncrease = 0;
		int32 MaxCollisionTransferAcceptedDonorComponentIncrease = 0;
		int32 MaxCollisionProposedDonorComponentIncrease = 0;
		int32 MaxCollisionProposedReceiverComponentIncrease = 0;
		int32 MaxCollisionAppliedDonorComponentIncrease = 0;
		int32 MaxCollisionAppliedReceiverComponentIncrease = 0;
		int32 TotalCollisionTrimmedByDonorProtectionCount = 0;
		int32 TotalCollisionRejectedByDonorProtectionCount = 0;
		int32 TotalCollisionTrimmedByDonorComponentCapCount = 0;
		int32 TotalCollisionTrimmedByDonorFragmentFloorCount = 0;
		int32 TotalCollisionRejectedByDonorComponentCapCount = 0;
		int32 TotalCollisionRejectedByDonorFragmentFloorCount = 0;
		int32 TotalCollisionTrimmedByReceiverProtectionCount = 0;
		int32 TotalCollisionRejectedByReceiverProtectionCount = 0;
		int32 TotalGeometricCollisionReceiverContiguousCount = 0;
		int32 TotalGeometricCollisionReceiverDisconnectedEventCount = 0;
		int64 TotalGeometricCollisionReceiverDisconnectedFragmentCount = 0;
		int32 MaxGeometricCollisionReceiverDisconnectedFragmentCount = 0;
		int32 MaxGeometricCollisionReceiverLargestNewDisconnectedFragmentSize = 0;
		int64 TotalPreserveOwnershipCWRetained = 0;
		int64 TotalPreserveOwnershipCWThresholdCrossingPrevented = 0;
		int64 TotalGeometricCollisionOverlapSamples = 0;
		int64 TotalGeometricCollisionPairCount = 0;
		int64 TotalGeometricCollisionQualifiedPairCount = 0;
		int64 TotalGeometricCollisionCandidateCount = 0;
		int64 TotalGeometricCollisionQualifiedCount = 0;
		int64 TotalGeometricCollisionQualifiedButDonorAmbiguousCount = 0;
		int64 TotalGeometricCollisionQualifiedButDonorSeedEmptyCount = 0;
		int64 TotalGeometricCollisionQualifiedUsingDirectionalDonorCount = 0;
		int64 TotalGeometricCollisionQualifiedUsingFallbackDonorRuleCount = 0;
		int64 TotalGeometricCollisionRejectedByMassFilterCount = 0;
		int64 TotalGeometricCollisionRejectedByOverlapDepthCount = 0;
		int64 TotalGeometricCollisionRejectedByOverlapDepthOverlapSampleCount = 0;
		int32 MaxGeometricCollisionRejectedByOverlapDepthOverlapSampleCount = 0;
		int64 TotalGeometricCollisionRejectedByOverlapDepthTerraneEstimate = 0;
		int32 MaxGeometricCollisionRejectedByOverlapDepthTerraneEstimate = 0;
		double TotalGeometricCollisionRejectedByOverlapDepthMeanConvergenceKmPerMy = 0.0;
		double MaxGeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy = 0.0;
		double TotalGeometricCollisionRejectedByOverlapDepthKm = 0.0;
		double MaxGeometricCollisionRejectedByOverlapDepthKm = 0.0;
		int64 TotalGeometricCollisionRejectedByPersistentPenetrationCount = 0;
		int64 TotalGeometricCollisionRejectedByPersistentPenetrationOverlapSampleCount = 0;
		int32 MaxGeometricCollisionRejectedByPersistentPenetrationOverlapSampleCount = 0;
		int64 TotalGeometricCollisionRejectedByPersistentPenetrationTerraneEstimate = 0;
		int32 MaxGeometricCollisionRejectedByPersistentPenetrationTerraneEstimate = 0;
		int64 TotalGeometricCollisionRejectedByPersistentPenetrationObservationCount = 0;
		int32 MaxGeometricCollisionRejectedByPersistentPenetrationObservationCount = 0;
		double TotalGeometricCollisionRejectedByPersistentPenetrationMeanConvergenceKmPerMy = 0.0;
		double MaxGeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy = 0.0;
		double TotalGeometricCollisionRejectedByPersistentPenetrationAccumulatedPenetrationKm = 0.0;
		double MaxGeometricCollisionRejectedByPersistentPenetrationAccumulatedPenetrationKm = 0.0;
		double TotalGeometricCollisionRejectedByPersistentPenetrationDepthKm = 0.0;
		double MaxGeometricCollisionRejectedByPersistentPenetrationDepthKm = 0.0;
		int64 TotalGeometricCollisionRejectedByEmptyTerraneCount = 0;
		int64 TotalGeometricCollisionRejectedByRoleResolutionCount = 0;
		int64 TotalGeometricCollisionRejectedBySeedDirectionCount = 0;
		int64 TotalGeometricCollisionQualifiedDirectionalCount = 0;
		int64 TotalGeometricCollisionDirectionalSeedCount = 0;
		int64 TotalGeometricCollisionDirectionalSeedCountOpposite = 0;
		int64 TotalGeometricCollisionSeedDirectionAuditCount = 0;
		int64 TotalGeometricCollisionSeedDirectionMismatchCount = 0;
		int64 TotalGeometricCollisionOnlyOverridingBucketPopulatedCount = 0;
		int64 TotalGeometricCollisionOnlySubductingBucketPopulatedCount = 0;
		int64 TotalGeometricCollisionBothDirectionalBucketsPopulatedCount = 0;
		int64 TotalGeometricCollisionNeitherDirectionalBucketsPopulatedCount = 0;
		int32 TotalPreserveResamplesWithGeometricCandidates = 0;
		int32 TotalPreserveResamplesWithQualifiedGeometricCandidates = 0;
		int32 TotalGeometricCollisionExecutions = 0;
		int32 TotalBoundaryContactFallbackExecutions = 0;
		int32 TotalGeometricCollisionFallbackUsed = 0;
		int32 TotalGeometricCollisionExecutedFromDirectionalPolarityCount = 0;
		int64 TotalCollisionTerraneRecovered = 0;
		int32 MaxCollisionTerraneRecovered = 0;
		int64 TotalCollisionCWBoosted = 0;
		int32 MaxCollisionCWBoosted = 0;
		int64 TotalCollisionSurgeAffected = 0;
		int32 MaxCollisionSurgeAffected = 0;
		int64 TotalGeometricCollisionExecutionCollisionGains = 0;
		int64 TotalGeometricCollisionExecutionTerraneRecovered = 0;
		int32 MaxGeometricCollisionExecutionTerraneRecovered = 0;
		int64 TotalGeometricCollisionExecutionCWBoosted = 0;
		int32 MaxGeometricCollisionExecutionCWBoosted = 0;
		int64 TotalGeometricCollisionExecutionSurgeAffected = 0;
		int32 MaxGeometricCollisionExecutionSurgeAffected = 0;
		int64 TotalBoundaryContactFallbackCollisionGains = 0;
		int64 TotalBoundaryContactFallbackTerraneRecovered = 0;
		int32 MaxBoundaryContactFallbackTerraneRecovered = 0;
		int64 TotalBoundaryContactFallbackCWBoosted = 0;
		int32 MaxBoundaryContactFallbackCWBoosted = 0;
		int64 TotalBoundaryContactFallbackSurgeAffected = 0;
		int32 MaxBoundaryContactFallbackSurgeAffected = 0;
		int64 TotalGeometricCollisionExecutedOverlapSampleCount = 0;
		int32 MaxGeometricCollisionExecutedOverlapSampleCount = 0;
		int64 TotalGeometricCollisionExecutedTerraneEstimate = 0;
		int32 MaxGeometricCollisionExecutedTerraneEstimate = 0;
		double TotalGeometricCollisionExecutedMeanConvergenceKmPerMy = 0.0;
		double MaxGeometricCollisionExecutedMaxConvergenceKmPerMy = 0.0;
		double TotalGeometricCollisionExecutedOverlapDepthKm = 0.0;
		double MaxGeometricCollisionExecutedOverlapDepthKm = 0.0;
		double TotalGeometricCollisionExecutedAccumulatedPenetrationKm = 0.0;
		double MaxGeometricCollisionExecutedAccumulatedPenetrationKm = 0.0;
		int64 TotalGeometricCollisionExecutedObservationCount = 0;
		int32 MaxGeometricCollisionExecutedObservationCount = 0;
		int32 TotalGeometricCollisionExecutedAfterMultipleObservationsCount = 0;
		int64 TotalGeometricCollisionExecutedDonorTerraneLocalityLimitedCount = 0;
		int32 TotalCollisionRepeatedPairCount = 0;
		int32 TotalCollisionRepeatedPairWithinCooldownCount = 0;
		int32 LargestGeometricTerraneEstimateObserved = 0;
		int32 MatchingBestPairResampleCount = 0;
		double GeometricCollisionDonorLocalityClampKm = 0.0;
		double GeometricCollisionInfluenceRadiusScale = 1.0;
		double GeometricCollisionPersistentPenetrationThresholdKm = 0.0;
		int32 GeometricCollisionDonorMaxComponentIncrease = 0;
		int32 GeometricCollisionDonorMinNewFragmentSampleCount = 0;
		FString DominantContinentalSink = TEXT("none");
		int64 DominantContinentalSinkMagnitude = 0;
		FString DominantDestructionPath = TEXT("none");
		int64 DominantDestructionPathMagnitude = 0;
		FString DominantNonGapDestructionSubpath = TEXT("none");
		int64 DominantNonGapDestructionSubpathMagnitude = 0;
		FString DominantFullResolutionLossBucket = TEXT("none");
		int64 DominantFullResolutionLossBucketMagnitude = 0;
		FString DominantGapLossMode = TEXT("none");
		int64 DominantGapLossMagnitude = 0;
		FString DominantGeometricRejectionReason = TEXT("none");
		int64 DominantGeometricRejectionMagnitude = 0;
		TArray<int32> MatchingBestPairSteps;
		bool bHasStrongestGeometricCandidate = false;
		FResamplingStats StrongestGeometricCandidateStats;
		bool bHasStrongestBoundaryCandidate = false;
		FResamplingStats StrongestBoundaryCandidateStats;
		bool bHasStrongestRepeatedGeometricCandidate = false;
		FResamplingStats StrongestRepeatedGeometricCandidateStats;
		bool bHasStrongestExecutedCollision = false;
		FResamplingStats StrongestExecutedCollisionStats;
		bool bHasLargestTopologyIncreaseEvent = false;
		FResamplingStats LargestTopologyIncreaseEventStats;
		bool bHasLargestRiftTopologyIncreaseEvent = false;
		FResamplingStats LargestRiftTopologyIncreaseEventStats;
		bool bHasPeakTopologySpikeEvent = false;
		FResamplingStats PeakTopologySpikeEventStats;
	};

	FString FormatM6BaselineBudgetReassessmentSummary(const FM6BaselineRunSummary& Summary)
	{
		const TCHAR* DominantCollisionProductivityPath =
			Summary.TotalGeometricCollisionExecutionCollisionGains >= Summary.TotalBoundaryContactFallbackCollisionGains
				? TEXT("geometric")
				: TEXT("boundary_fallback");
		return FString::Printf(
			TEXT("m6_baseline_budget_reassessment_summary run_id=%s seed=%d total_continental_samples_lost=%lld total_net_continental_sample_delta=%lld total_former_continental_turned_oceanic=%lld total_former_continental_divergent_oceanized=%lld total_former_continental_non_divergent_fallback_oceanized=%lld total_former_continental_non_gap_reclassified_non_continental=%lld total_former_continental_changed_plate_non_continental_final=%lld total_former_continental_full_resolution_non_continental_final=%lld total_full_resolution_same_plate_non_continental_final=%lld total_full_resolution_changed_plate_non_continental_final=%lld total_former_continental_projection_recovered_non_continental_final=%lld total_former_continental_preserve_mode_non_gap_non_continental_final=%lld total_former_continental_fallback_query_non_continental_final=%lld total_andean_continental_gains=%lld total_collision_continental_gains=%lld geometric_collision_gain_total=%lld boundary_fallback_collision_gain_total=%lld final_continental_area_fraction=%.6f dominant_destruction_path=%s dominant_destruction_path_magnitude=%lld dominant_non_gap_destruction_subpath=%s dominant_non_gap_destruction_subpath_magnitude=%lld dominant_full_resolution_loss_bucket=%s dominant_full_resolution_loss_bucket_magnitude=%lld dominant_collision_productivity_path=%s"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalContinentalSamplesLost,
			Summary.TotalNetContinentalSampleDelta,
			Summary.TotalFormerContinentalSamplesTurnedOceanic,
			Summary.TotalFormerContinentalDivergentOceanized,
			Summary.TotalFormerContinentalNonDivergentFallbackOceanized,
			Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal,
			Summary.TotalFormerContinentalChangedPlateNonContinentalFinal,
			Summary.TotalFormerContinentalFullResolutionNonContinentalFinal,
			Summary.TotalFullResolutionSamePlateNonContinentalFinal,
			Summary.TotalFullResolutionChangedPlateNonContinentalFinal,
			Summary.TotalFormerContinentalProjectionRecoveredNonContinentalFinal,
			Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal,
			Summary.TotalFormerContinentalFallbackQueryNonContinentalFinal,
			Summary.TotalAndeanContinentalGains,
			Summary.TotalCollisionContinentalGains,
			Summary.TotalGeometricCollisionExecutionCollisionGains,
			Summary.TotalBoundaryContactFallbackCollisionGains,
			Summary.FinalContinentalAreaFraction,
			*Summary.DominantDestructionPath,
			Summary.DominantDestructionPathMagnitude,
			*Summary.DominantNonGapDestructionSubpath,
			Summary.DominantNonGapDestructionSubpathMagnitude,
			*Summary.DominantFullResolutionLossBucket,
			Summary.DominantFullResolutionLossBucketMagnitude,
			DominantCollisionProductivityPath);
	}

	FString FormatM6BaselineFullResolutionLossSplitSummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_full_resolution_loss_split_summary run_id=%s seed=%d total_former_continental_full_resolution_non_continental_final=%lld total_full_resolution_same_plate_non_continental_final=%lld total_full_resolution_changed_plate_non_continental_final=%lld total_full_resolution_collision_followup_same_plate_non_continental=%lld total_full_resolution_collision_followup_changed_plate_non_continental=%lld total_full_resolution_rift_followup_same_plate_non_continental=%lld total_full_resolution_rift_followup_changed_plate_non_continental=%lld total_full_resolution_other_trigger_same_plate_non_continental=%lld total_full_resolution_other_trigger_changed_plate_non_continental=%lld total_full_resolution_same_plate_cw_retained=%lld total_full_resolution_same_plate_cw_threshold_crossing_prevented=%lld total_full_resolution_collision_followup_same_plate_cw_retained=%lld total_full_resolution_rift_followup_same_plate_cw_retained=%lld total_full_resolution_other_trigger_same_plate_cw_retained=%lld dominant_full_resolution_loss_bucket=%s dominant_full_resolution_loss_bucket_magnitude=%lld"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalFormerContinentalFullResolutionNonContinentalFinal,
			Summary.TotalFullResolutionSamePlateNonContinentalFinal,
			Summary.TotalFullResolutionChangedPlateNonContinentalFinal,
			Summary.TotalFullResolutionCollisionFollowupSamePlateNonContinental,
			Summary.TotalFullResolutionCollisionFollowupChangedPlateNonContinental,
			Summary.TotalFullResolutionRiftFollowupSamePlateNonContinental,
			Summary.TotalFullResolutionRiftFollowupChangedPlateNonContinental,
			Summary.TotalFullResolutionOtherTriggerSamePlateNonContinental,
			Summary.TotalFullResolutionOtherTriggerChangedPlateNonContinental,
			Summary.TotalFullResolutionSamePlateCWRetained,
			Summary.TotalFullResolutionSamePlateCWThresholdCrossingPrevented,
			Summary.TotalFullResolutionCollisionFollowupSamePlateCWRetained,
			Summary.TotalFullResolutionRiftFollowupSamePlateCWRetained,
			Summary.TotalFullResolutionOtherTriggerSamePlateCWRetained,
			*Summary.DominantFullResolutionLossBucket,
			Summary.DominantFullResolutionLossBucketMagnitude);
	}

	FString FormatM6Baseline400StepStabilitySummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_400_step_stability_summary run_id=%s seed=%d checkpoint_200_continental_area_fraction=%.6f checkpoint_300_continental_area_fraction=%.6f checkpoint_400_continental_area_fraction=%.6f first_half_net_continental_sample_delta=%lld second_half_net_continental_sample_delta=%lld first_half_continental_samples_lost=%lld second_half_continental_samples_lost=%lld first_half_andean_continental_gains=%lld second_half_andean_continental_gains=%lld first_half_collision_continental_gains=%lld second_half_collision_continental_gains=%lld max_components_per_plate_observed=%d final_max_components_per_plate=%d"),
			*Summary.RunId,
			Summary.Seed,
			Summary.bHasStep200Snapshot ? Summary.Step200Snapshot.ContinentalAreaFraction : -1.0,
			Summary.bHasStep300Snapshot ? Summary.Step300Snapshot.ContinentalAreaFraction : -1.0,
			Summary.bHasStep400Snapshot ? Summary.Step400Snapshot.ContinentalAreaFraction : Summary.FinalContinentalAreaFraction,
			Summary.FirstHalfNetContinentalSampleDelta,
			Summary.SecondHalfNetContinentalSampleDelta,
			Summary.FirstHalfContinentalSamplesLost,
			Summary.SecondHalfContinentalSamplesLost,
			Summary.FirstHalfAndeanContinentalGains,
			Summary.SecondHalfAndeanContinentalGains,
			Summary.FirstHalfCollisionContinentalGains,
			Summary.SecondHalfCollisionContinentalGains,
			Summary.MaxComponentsPerPlateObserved,
			Summary.bHasStep400Snapshot ? Summary.Step400Snapshot.MaxComponentsPerPlate : Summary.FinalSnapshot.MaxComponentsPerPlate);
	}

	FString FormatM6BaselineSecondHalfBudgetSplitSummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_second_half_budget_split_summary run_id=%s seed=%d first_half_continental_samples_lost=%lld second_half_continental_samples_lost=%lld first_half_andean_continental_gains=%lld second_half_andean_continental_gains=%lld first_half_collision_continental_gains=%lld second_half_collision_continental_gains=%lld first_half_net_continental_sample_delta=%lld second_half_net_continental_sample_delta=%lld"),
			*Summary.RunId,
			Summary.Seed,
			Summary.FirstHalfContinentalSamplesLost,
			Summary.SecondHalfContinentalSamplesLost,
			Summary.FirstHalfAndeanContinentalGains,
			Summary.SecondHalfAndeanContinentalGains,
			Summary.FirstHalfCollisionContinentalGains,
			Summary.SecondHalfCollisionContinentalGains,
			Summary.FirstHalfNetContinentalSampleDelta,
			Summary.SecondHalfNetContinentalSampleDelta);
	}

	FString FormatM6BaselineTopologyDriftAttributionSummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_topology_drift_attribution_summary run_id=%s seed=%d collision_followup_component_increase_total=%lld rift_followup_component_increase_total=%lld collision_followup_component_increase_0_100=%lld collision_followup_component_increase_100_200=%lld collision_followup_component_increase_200_400=%lld rift_followup_component_increase_0_100=%lld rift_followup_component_increase_100_200=%lld rift_followup_component_increase_200_400=%lld collision_followup_consolidation_event_count=%d rift_followup_consolidation_event_count=%d collision_followup_component_decrease=%lld rift_followup_component_decrease=%lld largest_single_event_component_increase=%d largest_single_event_type=%s largest_single_event_step=%d largest_single_event_plate_a=%d largest_single_event_plate_b=%d largest_single_event_global_before=%d largest_single_event_global_after=%d checkpoint_100_max_components_per_plate=%d checkpoint_200_max_components_per_plate=%d checkpoint_300_max_components_per_plate=%d checkpoint_400_max_components_per_plate=%d"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalCollisionFollowupComponentIncrease,
			Summary.TotalRiftFollowupComponentIncrease,
			Summary.CollisionFollowupComponentIncrease0To100,
			Summary.CollisionFollowupComponentIncrease100To200,
			Summary.CollisionFollowupComponentIncrease200To400,
			Summary.RiftFollowupComponentIncrease0To100,
			Summary.RiftFollowupComponentIncrease100To200,
			Summary.RiftFollowupComponentIncrease200To400,
			Summary.CollisionFollowupConsolidationEventCount,
			Summary.RiftFollowupConsolidationEventCount,
			Summary.CollisionFollowupAffectedPlateComponentDecrease,
			Summary.RiftFollowupAffectedPlateComponentDecrease,
			Summary.bHasLargestTopologyIncreaseEvent
				? FMath::Max(
					0,
					Summary.LargestTopologyIncreaseEventStats.TopologyGlobalMaxComponentsAfter -
						Summary.LargestTopologyIncreaseEventStats.TopologyGlobalMaxComponentsBefore)
				: 0,
			Summary.bHasLargestTopologyIncreaseEvent
				? GetResampleTriggerReasonName(Summary.LargestTopologyIncreaseEventStats.TriggerReason)
				: TEXT("None"),
			Summary.bHasLargestTopologyIncreaseEvent ? Summary.LargestTopologyIncreaseEventStats.Step : INDEX_NONE,
			Summary.bHasLargestTopologyIncreaseEvent ? Summary.LargestTopologyIncreaseEventStats.TopologyPrimaryAffectedPlateId : INDEX_NONE,
			Summary.bHasLargestTopologyIncreaseEvent ? Summary.LargestTopologyIncreaseEventStats.TopologySecondaryAffectedPlateId : INDEX_NONE,
			Summary.bHasLargestTopologyIncreaseEvent ? Summary.LargestTopologyIncreaseEventStats.TopologyGlobalMaxComponentsBefore : 0,
			Summary.bHasLargestTopologyIncreaseEvent ? Summary.LargestTopologyIncreaseEventStats.TopologyGlobalMaxComponentsAfter : 0,
			Summary.bHasStep100Snapshot ? Summary.Step100Snapshot.MaxComponentsPerPlate : -1,
			Summary.bHasStep200Snapshot ? Summary.Step200Snapshot.MaxComponentsPerPlate : -1,
			Summary.bHasStep300Snapshot ? Summary.Step300Snapshot.MaxComponentsPerPlate : -1,
			Summary.bHasStep400Snapshot ? Summary.Step400Snapshot.MaxComponentsPerPlate : Summary.FinalSnapshot.MaxComponentsPerPlate);
	}

	FString FormatM6BaselineRiftTopologyAttributionSummary(const FM6BaselineRunSummary& Summary)
	{
		const auto ComputeMeanComponentDelta = [](const int64 TotalIncrease, const int32 EventCount)
		{
			return EventCount > 0
				? static_cast<double>(TotalIncrease) / static_cast<double>(EventCount)
				: 0.0;
		};

		return FString::Printf(
			TEXT("m6_baseline_rift_topology_attribution_summary run_id=%s seed=%d total_rift_followup_count=%d rift_followup_component_increase_total=%lld rift_followup_count_0_100=%d rift_followup_count_100_200=%d rift_followup_count_200_400=%d rift_followup_component_increase_0_100=%lld rift_followup_component_increase_100_200=%lld rift_followup_component_increase_200_400=%lld rift_followup_mean_component_delta_0_100=%.6f rift_followup_mean_component_delta_100_200=%.6f rift_followup_mean_component_delta_200_400=%.6f rift_followup_consolidation_event_count=%d rift_followup_component_decrease=%lld"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalRiftEvents,
			Summary.TotalRiftFollowupComponentIncrease,
			Summary.RiftFollowupCount0To100,
			Summary.RiftFollowupCount100To200,
			Summary.RiftFollowupCount200To400,
			Summary.RiftFollowupComponentIncrease0To100,
			Summary.RiftFollowupComponentIncrease100To200,
			Summary.RiftFollowupComponentIncrease200To400,
			ComputeMeanComponentDelta(
				Summary.RiftFollowupComponentIncrease0To100,
				Summary.RiftFollowupCount0To100),
			ComputeMeanComponentDelta(
				Summary.RiftFollowupComponentIncrease100To200,
				Summary.RiftFollowupCount100To200),
			ComputeMeanComponentDelta(
				Summary.RiftFollowupComponentIncrease200To400,
				Summary.RiftFollowupCount200To400),
			Summary.RiftFollowupConsolidationEventCount,
			Summary.RiftFollowupAffectedPlateComponentDecrease);
	}

	FString FormatM6BaselineWorstRiftSpikeSummary(const FM6BaselineRunSummary& Summary)
	{
		const FString ChildPlateIds = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(Summary.LargestRiftTopologyIncreaseEventStats.RiftChildPlateIds))
			: TEXT("[]");
		const FString ChildComponentsBefore = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(Summary.LargestRiftTopologyIncreaseEventStats.RiftChildComponentCountsBefore))
			: TEXT("[]");
		const FString ChildComponentsAfter = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(Summary.LargestRiftTopologyIncreaseEventStats.RiftChildComponentCountsAfter))
			: TEXT("[]");
		const FString PositiveGrowthPlateIds = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(Summary.LargestRiftTopologyIncreaseEventStats.RiftPositiveGrowthPlateIds))
			: TEXT("[]");
		const FString PositiveGrowthPlateTypes = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinRiftRecipientTypeCodesForTest(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftPositiveGrowthPlateTypeCodes))
			: TEXT("[]");
		const FString PositiveGrowthComponentsBefore = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftPositiveGrowthPlateComponentsBefore))
			: TEXT("[]");
		const FString PositiveGrowthComponentsAfter = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftPositiveGrowthPlateComponentsAfter))
			: TEXT("[]");
		const FString PositiveGrowthLargestFragmentSizes = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftPositiveGrowthPlateLargestFragmentSizes))
			: TEXT("[]");
		const FString RecipientPlateIds = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftStrayFragmentRecipientPlateIds))
			: TEXT("[]");
		const FString RecipientTypes = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinRiftRecipientTypeCodesForTest(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftStrayFragmentRecipientTypeCodes))
			: TEXT("[]");
		const FString RecipientComponentsBefore = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftStrayFragmentRecipientComponentsBefore))
			: TEXT("[]");
		const FString RecipientComponentsAfter = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftStrayFragmentRecipientComponentsAfter))
			: TEXT("[]");
		const FString RecipientFragmentSizes = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(Summary.LargestRiftTopologyIncreaseEventStats.RiftStrayFragmentSizes))
			: TEXT("[]");
		const FString RecipientSiblingEdgeCounts = Summary.bHasLargestRiftTopologyIncreaseEvent
			? FString::Printf(
				TEXT("[%s]"),
				*JoinIntArray(
					Summary.LargestRiftTopologyIncreaseEventStats.RiftStrayFragmentSiblingEdgeCounts))
			: TEXT("[]");
		return FString::Printf(
			TEXT("m6_baseline_worst_rift_spike_summary run_id=%s seed=%d has_worst_rift_spike=%d worst_rift_step=%d worst_rift_global_before=%d worst_rift_global_after=%d worst_rift_component_increase=%d worst_rift_parent_plate_id=%d worst_rift_parent_components_before=%d worst_rift_parent_components_after=%d worst_rift_child_count=%d worst_rift_child_plate_ids=%s worst_rift_child_components_before=%s worst_rift_child_components_after=%s worst_rift_positive_growth_plate_ids=%s worst_rift_positive_growth_plate_types=%s worst_rift_positive_growth_components_before=%s worst_rift_positive_growth_components_after=%s worst_rift_positive_growth_largest_fragment_sizes=%s worst_rift_recipient_plate_ids=%s worst_rift_recipient_types=%s worst_rift_recipient_components_before=%s worst_rift_recipient_components_after=%s worst_rift_recipient_fragment_sizes=%s worst_rift_recipient_sibling_edge_counts=%s worst_rift_affected_plate_count_before=%d worst_rift_affected_plate_count_after=%d worst_rift_min_child_sample_count=%d worst_rift_max_child_sample_count=%d worst_rift_mean_child_sample_count=%.6f"),
			*Summary.RunId,
			Summary.Seed,
			Summary.bHasLargestRiftTopologyIncreaseEvent ? 1 : 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.Step
				: INDEX_NONE,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.TopologyGlobalMaxComponentsBefore
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.TopologyGlobalMaxComponentsAfter
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? FMath::Max(
					0,
					Summary.LargestRiftTopologyIncreaseEventStats.TopologyGlobalMaxComponentsAfter -
						Summary.LargestRiftTopologyIncreaseEventStats.TopologyGlobalMaxComponentsBefore)
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftParentPlateId
				: INDEX_NONE,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftParentComponentsBefore
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftParentComponentsAfter
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftChildCount
				: 0,
			*ChildPlateIds,
			*ChildComponentsBefore,
			*ChildComponentsAfter,
			*PositiveGrowthPlateIds,
			*PositiveGrowthPlateTypes,
			*PositiveGrowthComponentsBefore,
			*PositiveGrowthComponentsAfter,
			*PositiveGrowthLargestFragmentSizes,
			*RecipientPlateIds,
			*RecipientTypes,
			*RecipientComponentsBefore,
			*RecipientComponentsAfter,
			*RecipientFragmentSizes,
			*RecipientSiblingEdgeCounts,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftAffectedPlateCountBefore
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftAffectedPlateCountAfter
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftMinChildSampleCount
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftMaxChildSampleCount
				: 0,
			Summary.bHasLargestRiftTopologyIncreaseEvent
				? Summary.LargestRiftTopologyIncreaseEventStats.RiftMeanChildSampleCount
				: 0.0);
	}

	FString FormatM6BaselineRiftLocalizationSummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_rift_localization_summary run_id=%s seed=%d total_restored_non_child_samples=%lld total_restored_gap_samples=%lld total_localized_cw_restored_count=%lld total_localized_cw_phantom_prevented_count=%lld total_localized_cw_continental_prevented_count=%lld total_rift_interpolation_created_gains=%lld total_rift_final_owner_mismatch_gain_count_after_localization=%lld total_rift_final_owner_cw_reconciled_count=%lld total_rift_final_owner_mismatch_gain_count_before_reconciliation=%lld total_rift_final_owner_mismatch_gain_count_after_reconciliation=%lld total_rift_final_owner_mismatch_continental_prevented_count=%lld total_rift_same_owner_child_interpolation_gain_count_before_reconciliation=%lld total_rift_same_owner_child_interpolation_gain_count_after_reconciliation=%lld total_rift_final_gain_started_below_025_count=%lld total_rift_final_gain_started_below_040_count=%lld total_rift_final_gain_started_below_050_count=%lld total_child_stray_fragments_detected=%lld total_child_stray_fragments_reassigned=%lld max_largest_stray_child_fragment_size=%d reassigned_to_sibling_count=%d reassigned_to_other_neighbor_count=%d reassigned_by_adjacent_sibling_count=%d reassigned_by_zero_adjacency_sibling_fallback_count=%d reassigned_to_non_child_fallback_count=%d zero_sibling_adjacency_count=%d positive_sibling_adjacency_count=%d max_largest_fragment_causing_recipient_growth_size=%d"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalRiftLocalizedNonChildSampleRestoreCount,
			Summary.TotalRiftLocalizedGapSampleRestoreCount,
			Summary.TotalRiftLocalizedCWRestoredCount,
			Summary.TotalRiftLocalizedCWPhantomPreventedCount,
			Summary.TotalRiftLocalizedCWContinentalPreventedCount,
			Summary.TotalRiftInterpolationCreatedGainCount,
			Summary.TotalRiftFinalOwnerMismatchGainCountAfterLocalization,
			Summary.TotalRiftFinalOwnerCWReconciledCount,
			Summary.TotalRiftFinalOwnerMismatchGainCountBeforeReconciliation,
			Summary.TotalRiftFinalOwnerMismatchGainCountAfterReconciliation,
			Summary.TotalRiftFinalOwnerMismatchContinentalPreventedCount,
			Summary.TotalRiftSameOwnerChildInterpolationGainCountBeforeReconciliation,
			Summary.TotalRiftSameOwnerChildInterpolationGainCountAfterReconciliation,
			Summary.TotalRiftFinalGainStartedBelow025Count,
			Summary.TotalRiftFinalGainStartedBelow040Count,
			Summary.TotalRiftFinalGainStartedBelow050Count,
			Summary.TotalRiftChildStrayFragmentDetectedCount,
			Summary.TotalRiftChildStrayFragmentReassignedCount,
			Summary.MaxRiftLargestStrayChildFragmentSize,
			Summary.TotalRiftStrayFragmentReassignedToSiblingCount,
			Summary.TotalRiftStrayFragmentReassignedToOtherNeighborCount,
			Summary.TotalRiftStrayFragmentReassignedByAdjacentSiblingCount,
			Summary.TotalRiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount,
			Summary.TotalRiftStrayFragmentForcedNonChildAssignmentCount,
			Summary.TotalRiftStrayFragmentZeroSiblingAdjacencyCount,
			Summary.TotalRiftStrayFragmentPositiveSiblingAdjacencyCount,
			Summary.MaxRiftLargestFragmentCausingRecipientGrowthSize);
	}

	FString FormatM6BaselinePreserveFallbackSummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_preserve_fallback_summary run_id=%s seed=%d total_fallback_same_plate_recontained_count=%lld total_fallback_same_plate_retained_count=%lld total_fallback_changed_owner_count=%lld total_fallback_gap_count=%lld total_fallback_divergent_oceanization_count=%lld total_continental_loss_count_after_fallback=%lld"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalPreserveOwnershipFallbackSamePlateRecontainedCount,
			Summary.TotalPreserveOwnershipFallbackSamePlateRetainedCount,
			Summary.TotalPreserveOwnershipFallbackChangedOwnerCount,
			Summary.TotalPreserveOwnershipFallbackGapCount,
			Summary.TotalPreserveOwnershipFallbackDivergentOceanizationCount,
			Summary.TotalPreserveOwnershipContinentalLossCountAfterFallback);
	}

	FString FormatPreserveFallbackLossPairs(const FResamplingStats& Stats)
	{
		if (Stats.PreserveOwnershipFallbackLossPairPreviousPlateIds.Num() !=
				Stats.PreserveOwnershipFallbackLossPairFinalPlateIds.Num() ||
			Stats.PreserveOwnershipFallbackLossPairPreviousPlateIds.Num() !=
				Stats.PreserveOwnershipFallbackLossPairCounts.Num() ||
			Stats.PreserveOwnershipFallbackLossPairCounts.IsEmpty())
		{
			return TEXT("none");
		}

		TArray<FString> Parts;
		Parts.Reserve(Stats.PreserveOwnershipFallbackLossPairCounts.Num());
		for (int32 Index = 0; Index < Stats.PreserveOwnershipFallbackLossPairCounts.Num(); ++Index)
		{
			Parts.Add(FString::Printf(
				TEXT("%d->%d:%d"),
				Stats.PreserveOwnershipFallbackLossPairPreviousPlateIds[Index],
				Stats.PreserveOwnershipFallbackLossPairFinalPlateIds[Index],
				Stats.PreserveOwnershipFallbackLossPairCounts[Index]));
		}

		return FString::Join(Parts, TEXT(","));
	}

	FString FormatM6BaselinePreserveBoundaryPolicySummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_preserve_boundary_policy_summary run_id=%s seed=%d total_previous_owner_hysteresis_application_count=%lld total_preserved_strongly_continental_boundary_sample_count=%lld total_changed_owner_non_gap_loss_count=%lld total_divergent_loss_count=%lld total_non_divergent_projection_loss_count=%lld total_non_divergent_fallback_oceanized_loss_count=%lld total_fallback_loss_ge_090_count=%lld total_fallback_loss_ge_075_count=%lld total_fallback_loss_ge_050_count=%lld"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalPreserveOwnershipPreviousOwnerHysteresisApplicationCount,
			Summary.TotalPreserveOwnershipStronglyContinentalBoundarySavedCount,
			Summary.TotalPreserveOwnershipFallbackChangedOwnerNonGapLossCount,
			Summary.TotalPreserveOwnershipFallbackDivergentLossCount,
			Summary.TotalPreserveOwnershipFallbackNonDivergentProjectionLossCount,
			Summary.TotalPreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount,
			Summary.TotalPreserveOwnershipFallbackStrongLossGE090Count,
			Summary.TotalPreserveOwnershipFallbackStrongLossGE075Count,
			Summary.TotalPreserveOwnershipFallbackStrongLossGE050Count);
	}

	FString FormatM6BaselineStep229RiftPulseSummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_step229_rift_pulse_summary run_id=%s seed=%d has_step229_rift_followup=%d gross_gain_total=%d gross_loss_total=%d net_gain=%d interpolation_created_gains=%d localized_cw_restored_count=%d localized_cw_phantom_prevented_count=%d localized_cw_continental_prevented_count=%d final_owner_mismatch_gain_count_after_localization=%d final_owner_cw_reconciled_count=%d mismatch_gain_count_before_reconciliation=%d mismatch_gain_count_after_reconciliation=%d mismatch_continental_prevented_count=%d same_owner_child_interpolation_gain_count_before_reconciliation=%d same_owner_child_interpolation_gain_count_after_reconciliation=%d final_gain_started_below_025_count=%d final_gain_started_below_040_count=%d final_gain_started_below_050_count=%d"),
			*Summary.RunId,
			Summary.Seed,
			Summary.bHasStep229RiftFollowupStats ? 1 : 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.AndeanContinentalGainCount
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.ContinentalSamplesLost
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.NetContinentalSampleDelta
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftInterpolationCreatedGainCount
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftLocalizedCWRestoredCount
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftLocalizedCWPhantomPreventedCount
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftLocalizedCWContinentalPreventedCount
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftFinalOwnerMismatchGainCountAfterLocalization
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftFinalOwnerCWReconciledCount
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftFinalOwnerMismatchGainCountBeforeReconciliation
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftFinalOwnerMismatchGainCountAfterReconciliation
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftFinalOwnerMismatchContinentalPreventedCount
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftSameOwnerChildInterpolationGainCountBeforeReconciliation
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftSameOwnerChildInterpolationGainCountAfterReconciliation
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftFinalGainStartedBelow025Count
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftFinalGainStartedBelow040Count
				: 0,
			Summary.bHasStep229RiftFollowupStats
				? Summary.Step229RiftFollowupStats.RiftFinalGainStartedBelow050Count
				: 0);
	}

		FString FormatM6BaselineCollisionFragmentationSplitSummary(const FM6BaselineRunSummary& Summary)
		{
		const double MeanAppliedDonorDelta =
			Summary.TotalCollisionEvents > 0
				? static_cast<double>(Summary.TotalCollisionAppliedDonorComponentIncrease) /
					static_cast<double>(Summary.TotalCollisionEvents)
				: 0.0;
		const double MeanAppliedReceiverDelta =
			Summary.TotalCollisionEvents > 0
				? static_cast<double>(Summary.TotalCollisionAppliedReceiverComponentIncrease) /
					static_cast<double>(Summary.TotalCollisionEvents)
				: 0.0;
		const FString DominantProposedFragmentationSide =
			Summary.TotalCollisionProposedDonorComponentIncrease >= Summary.TotalCollisionProposedReceiverComponentIncrease
				? TEXT("donor")
				: TEXT("receiver");
		return FString::Printf(
			TEXT("m6_baseline_collision_fragmentation_split_summary run_id=%s seed=%d proposed_donor_component_increase_total=%lld proposed_receiver_component_increase_total=%lld applied_donor_component_increase_total=%lld applied_receiver_component_increase_total=%lld mean_applied_donor_component_delta=%.6f max_applied_donor_component_delta=%d mean_applied_receiver_component_delta=%.6f max_applied_receiver_component_delta=%d trimmed_by_donor_protection_count=%d rejected_by_donor_protection_count=%d trimmed_by_receiver_protection_count=%d rejected_by_receiver_protection_count=%d dominant_proposed_fragmentation_side=%s"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalCollisionProposedDonorComponentIncrease,
			Summary.TotalCollisionProposedReceiverComponentIncrease,
			Summary.TotalCollisionAppliedDonorComponentIncrease,
			Summary.TotalCollisionAppliedReceiverComponentIncrease,
			MeanAppliedDonorDelta,
			Summary.MaxCollisionAppliedDonorComponentIncrease,
			MeanAppliedReceiverDelta,
			Summary.MaxCollisionAppliedReceiverComponentIncrease,
			Summary.TotalCollisionTrimmedByDonorProtectionCount,
			Summary.TotalCollisionRejectedByDonorProtectionCount,
			Summary.TotalCollisionTrimmedByReceiverProtectionCount,
			Summary.TotalCollisionRejectedByReceiverProtectionCount,
			*DominantProposedFragmentationSide);
	}

	FString FormatM6BaselineCollisionDonorProtectionTrimSummary(const FM6BaselineRunSummary& Summary)
	{
		const double MeanProposedTerraneSampleCount =
			Summary.TotalCollisionTransferDiagnosticsCount > 0
				? static_cast<double>(Summary.TotalCollisionProposedTerraneSampleCount) /
					static_cast<double>(Summary.TotalCollisionTransferDiagnosticsCount)
				: 0.0;
		const double MeanAcceptedTerraneSampleCount =
			Summary.TotalCollisionTransferDiagnosticsCount > 0
				? static_cast<double>(Summary.TotalCollisionAcceptedTerraneSampleCount) /
					static_cast<double>(Summary.TotalCollisionTransferDiagnosticsCount)
				: 0.0;
		const double MeanTrimRatio =
			Summary.TotalCollisionTransferDiagnosticsCount > 0
				? Summary.TotalCollisionTransferTrimRatio /
					static_cast<double>(Summary.TotalCollisionTransferDiagnosticsCount)
				: 0.0;
		const double MeanProposedDonorDelta =
			Summary.TotalCollisionTransferDiagnosticsCount > 0
				? static_cast<double>(Summary.TotalCollisionTransferProposedDonorComponentIncrease) /
					static_cast<double>(Summary.TotalCollisionTransferDiagnosticsCount)
				: 0.0;
		const double MeanAcceptedDonorDelta =
			Summary.TotalCollisionTransferDiagnosticsCount > 0
				? static_cast<double>(Summary.TotalCollisionTransferAcceptedDonorComponentIncrease) /
					static_cast<double>(Summary.TotalCollisionTransferDiagnosticsCount)
				: 0.0;
		return FString::Printf(
			TEXT("m6_baseline_collision_donor_protection_trim_summary run_id=%s seed=%d donor_max_component_increase=%d donor_min_new_fragment_sample_count=%d transfer_diagnostics_count=%d mean_proposed_terrane_size=%.6f max_proposed_terrane_size=%d mean_accepted_terrane_size=%.6f max_accepted_terrane_size=%d mean_trim_ratio=%.6f max_trim_ratio=%.6f mean_proposed_donor_component_delta=%.6f max_proposed_donor_component_delta=%d mean_accepted_donor_component_delta=%.6f max_accepted_donor_component_delta=%d trimmed_by_donor_component_cap_count=%d trimmed_by_donor_fragment_floor_count=%d rejected_by_donor_component_cap_count=%d rejected_by_donor_fragment_floor_count=%d"),
			*Summary.RunId,
			Summary.Seed,
			Summary.GeometricCollisionDonorMaxComponentIncrease,
			Summary.GeometricCollisionDonorMinNewFragmentSampleCount,
			Summary.TotalCollisionTransferDiagnosticsCount,
			MeanProposedTerraneSampleCount,
			Summary.MaxCollisionProposedTerraneSampleCount,
			MeanAcceptedTerraneSampleCount,
			Summary.MaxCollisionAcceptedTerraneSampleCount,
			MeanTrimRatio,
			Summary.MaxCollisionTransferTrimRatio,
			MeanProposedDonorDelta,
			Summary.MaxCollisionTransferProposedDonorComponentIncrease,
			MeanAcceptedDonorDelta,
			Summary.MaxCollisionTransferAcceptedDonorComponentIncrease,
			Summary.TotalCollisionTrimmedByDonorComponentCapCount,
			Summary.TotalCollisionTrimmedByDonorFragmentFloorCount,
			Summary.TotalCollisionRejectedByDonorComponentCapCount,
			Summary.TotalCollisionRejectedByDonorFragmentFloorCount);
	}

	FString FormatM6BaselineCollisionReceiverContiguitySummary(const FM6BaselineRunSummary& Summary)
	{
		const double MeanDisconnectedFragmentCount =
			Summary.TotalGeometricCollisionReceiverDisconnectedEventCount > 0
				? static_cast<double>(Summary.TotalGeometricCollisionReceiverDisconnectedFragmentCount) /
					static_cast<double>(Summary.TotalGeometricCollisionReceiverDisconnectedEventCount)
				: 0.0;
		return FString::Printf(
			TEXT("m6_baseline_collision_receiver_contiguity_summary run_id=%s seed=%d total_geometric_collision_executions=%d contiguous_receiver_collision_count=%d disconnected_receiver_fragment_event_count=%d mean_disconnected_receiver_fragment_count=%.6f max_disconnected_receiver_fragment_count=%d max_largest_new_disconnected_fragment_size=%d"),
			*Summary.RunId,
			Summary.Seed,
			Summary.TotalGeometricCollisionExecutions,
			Summary.TotalGeometricCollisionReceiverContiguousCount,
			Summary.TotalGeometricCollisionReceiverDisconnectedEventCount,
			MeanDisconnectedFragmentCount,
			Summary.MaxGeometricCollisionReceiverDisconnectedFragmentCount,
			Summary.MaxGeometricCollisionReceiverLargestNewDisconnectedFragmentSize);
	}

	FString FormatM6BaselineTopologySpikeSummary(const FM6BaselineRunSummary& Summary)
	{
		return FString::Printf(
			TEXT("m6_baseline_topology_spike_summary run_id=%s seed=%d peak_max_components_per_plate=%d peak_event_type=%s peak_event_step=%d peak_event_plate_a=%d peak_event_plate_b=%d peak_event_global_before=%d peak_event_global_after=%d peak_event_collision_terrane_recovered=%d peak_event_collision_donor_components_before=%d peak_event_collision_donor_components_after=%d peak_event_collision_donor_delta=%d peak_event_collision_donor_largest_remaining_component_size=%d peak_event_collision_donor_new_fragment_count=%d peak_event_collision_donor_largest_new_fragment_size=%d peak_event_collision_receiver_components_before=%d peak_event_collision_receiver_components_after=%d peak_event_collision_receiver_delta=%d peak_event_collision_receiver_contiguous=%d peak_event_collision_disconnected_fragment_count=%d peak_event_collision_largest_new_disconnected_fragment_size=%d peak_event_collision_trimmed_by_donor=%d peak_event_collision_trimmed_by_receiver=%d peak_event_rift_child_count=%d"),
			*Summary.RunId,
			Summary.Seed,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.TopologyGlobalMaxComponentsAfter : 0,
			Summary.bHasPeakTopologySpikeEvent
				? GetResampleTriggerReasonName(Summary.PeakTopologySpikeEventStats.TriggerReason)
				: TEXT("None"),
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.Step : INDEX_NONE,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.TopologyPrimaryAffectedPlateId : INDEX_NONE,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.TopologySecondaryAffectedPlateId : INDEX_NONE,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.TopologyGlobalMaxComponentsBefore : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.TopologyGlobalMaxComponentsAfter : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.GeometricCollisionExecutedTerraneRecoveredCount : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionDonorComponentsBefore : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionDonorComponentsAfter : 0,
			Summary.bHasPeakTopologySpikeEvent
				? (Summary.PeakTopologySpikeEventStats.CollisionDonorComponentsAfter -
					Summary.PeakTopologySpikeEventStats.CollisionDonorComponentsBefore)
				: 0,
			Summary.bHasPeakTopologySpikeEvent
				? Summary.PeakTopologySpikeEventStats.CollisionDonorLargestRemainingComponentSize
				: 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionDonorNewFragmentCount : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionDonorLargestNewFragmentSize : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionReceiverComponentsBefore : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionReceiverComponentsAfter : 0,
			Summary.bHasPeakTopologySpikeEvent
				? (Summary.PeakTopologySpikeEventStats.CollisionReceiverComponentsAfter -
					Summary.PeakTopologySpikeEventStats.CollisionReceiverComponentsBefore)
				: 0,
			(Summary.bHasPeakTopologySpikeEvent && Summary.PeakTopologySpikeEventStats.bCollisionReceiverTransferContiguousWithExistingTerritory) ? 1 : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionReceiverDisconnectedFragmentCount : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionReceiverLargestNewDisconnectedFragmentSize : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionTrimmedByDonorProtectionCount : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.CollisionTrimmedByReceiverProtectionCount : 0,
			Summary.bHasPeakTopologySpikeEvent ? Summary.PeakTopologySpikeEventStats.RiftChildCount : 0);
	}

	FM6BaselineRunSummary RunM6BaselineScenario(
		FAutomationTestBase& Test,
		const FM6BaselineRunConfig& Config)
	{
		FM6BaselineRunSummary Summary;
		Summary.RunId = Config.RunId;
		Summary.Seed = Config.Seed;

		FTectonicPlanet Planet = CreateInitializedPlanet(Config.PlateCount, Config.SampleCount, Config.Seed);
		FTectonicPlanetRuntimeConfig RuntimeConfig = Config.RuntimeConfig;
		RuntimeConfig.AutomaticRiftBaseRatePerMy = Config.AutomaticRiftBaseRatePerMy;
		RuntimeConfig.AndeanContinentalConversionRatePerMy = Config.AndeanContinentalConversionRatePerMy;
		Summary.AndeanContinentalConversionRatePerMy = RuntimeConfig.AndeanContinentalConversionRatePerMy;
		ApplyTectonicPlanetRuntimeConfig(Planet, RuntimeConfig);
		Test.AddInfo(FString::Printf(
			TEXT("m6_baseline_runtime_preset run_id=%s preset=%s %s"),
			*Summary.RunId,
			*Config.PresetLabel,
			*DescribeTectonicPlanetRuntimeConfig(CaptureTectonicPlanetRuntimeConfig(Planet))));
		Summary.GeometricCollisionDonorLocalityClampKm = Planet.GeometricCollisionDonorLocalityClampKm;
		Summary.GeometricCollisionInfluenceRadiusScale = Planet.GeometricCollisionInfluenceRadiusScale;
		Summary.GeometricCollisionPersistentPenetrationThresholdKm =
			Planet.GeometricCollisionMinPersistentPenetrationKm;
		Summary.GeometricCollisionDonorMaxComponentIncrease =
			Planet.GeometricCollisionDonorMaxComponentIncrease;
		Summary.GeometricCollisionDonorMinNewFragmentSampleCount =
			Planet.GeometricCollisionDonorMinNewFragmentSampleCount;

		TArray<int32> PreviousPlateIds;
		PreviousPlateIds.Reserve(Planet.Samples.Num());
		for (const FSample& Sample : Planet.Samples)
		{
			PreviousPlateIds.Add(Sample.PlateId);
		}

		Summary.InitialSnapshot = BuildM6CheckpointSnapshot(Planet);
		if (Config.bExportCheckpoints)
		{
			Test.TestTrue(
				TEXT("M6 baseline step 0 export succeeded"),
				ExportM6BaselineCheckpointMaps(Test, Planet, Config.ExportRoot, 0));
		}
		if (Config.bLogCheckpoints)
		{
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_checkpoint run_id=%s step=%d resampling_steps=%s last_trigger_reason=%s last_ownership_mode=%s boundary_sample_count=%d boundary_sample_fraction=%.6f gap_count=%d overlap_count=%d continental_continental_overlap_count=%d continental_count=%d mean_elevation=%.6f preserve_same_plate_hit_count=%d preserve_same_plate_recovery_count=%d preserve_fallback_query_count=%d preserve_plate_changed_count=%d"),
				*Summary.RunId,
				Summary.InitialSnapshot.Step,
				*FormatStepList(Summary.InitialSnapshot.ResamplingSteps),
				GetResampleTriggerReasonName(Summary.InitialSnapshot.LastTriggerReason),
				GetResampleOwnershipModeName(Summary.InitialSnapshot.LastOwnershipMode),
				Summary.InitialSnapshot.BoundarySampleCount,
				Summary.InitialSnapshot.BoundarySampleFraction,
				Summary.InitialSnapshot.GapCount,
				Summary.InitialSnapshot.OverlapCount,
				Summary.InitialSnapshot.ContinentalContinentalOverlapCount,
				Summary.InitialSnapshot.ContinentalCount,
				Summary.InitialSnapshot.ElevationMeanKm,
				Summary.InitialSnapshot.PreserveSamePlateHitCount,
				Summary.InitialSnapshot.PreserveSamePlateRecoveryCount,
				Summary.InitialSnapshot.PreserveFallbackQueryCount,
				Summary.InitialSnapshot.PreservePlateChangedCount));
		}

		TSet<int32> CheckpointSteps;
		for (const int32 CheckpointStep : { 50, 100, 150, 200, 300, Config.MaxStep })
		{
			if (CheckpointStep > 0 && CheckpointStep <= Config.MaxStep)
			{
				CheckpointSteps.Add(CheckpointStep);
			}
		}

		TMap<int32, FM6CheckpointSnapshot> Checkpoints;
		Checkpoints.Add(0, Summary.InitialSnapshot);
		while (Planet.CurrentStep < Config.MaxStep)
		{
			const int32 ResampleHistoryCountBeforeStep = Planet.ResamplingHistory.Num();
			Planet.AdvanceStep();
			const bool bResampledThisStep = Planet.ResamplingHistory.Num() > ResampleHistoryCountBeforeStep;

			int32 OwnershipChangesThisStep = 0;
			for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
			{
				if (Planet.Samples[SampleIndex].PlateId != PreviousPlateIds[SampleIndex])
				{
					++OwnershipChangesThisStep;
					PreviousPlateIds[SampleIndex] = Planet.Samples[SampleIndex].PlateId;
				}
			}

			if (!bResampledThisStep && OwnershipChangesThisStep > 0)
			{
				Summary.OwnershipChangesOutsideResamplingCount += OwnershipChangesThisStep;
				++Summary.StepsWithOwnershipChangesOutsideResampling;
			}

			if (CheckpointSteps.Contains(Planet.CurrentStep))
			{
				const FM6CheckpointSnapshot Snapshot = BuildM6CheckpointSnapshot(Planet);
				Checkpoints.Add(Planet.CurrentStep, Snapshot);
				if (Planet.CurrentStep == 100)
				{
					Summary.Step100Snapshot = Snapshot;
					Summary.bHasStep100Snapshot = true;
				}
				else if (Planet.CurrentStep == 200)
				{
					Summary.Step200Snapshot = Snapshot;
					Summary.bHasStep200Snapshot = true;
				}
				else if (Planet.CurrentStep == 300)
				{
					Summary.Step300Snapshot = Snapshot;
					Summary.bHasStep300Snapshot = true;
				}
				else if (Planet.CurrentStep == 400)
				{
					Summary.Step400Snapshot = Snapshot;
					Summary.bHasStep400Snapshot = true;
				}
				if (Config.bExportCheckpoints)
				{
					Test.TestTrue(
						*FString::Printf(TEXT("M6 baseline export succeeded at step %d"), Planet.CurrentStep),
						ExportM6BaselineCheckpointMaps(Test, Planet, Config.ExportRoot, Planet.CurrentStep));
				}
				if (Config.bLogCheckpoints)
				{
					Test.AddInfo(FString::Printf(
						TEXT("m6_baseline_checkpoint run_id=%s step=%d resampling_steps=%s last_trigger_reason=%s last_ownership_mode=%s boundary_sample_count=%d boundary_sample_fraction=%.6f gap_count=%d overlap_count=%d continental_continental_overlap_count=%d continental_count=%d mean_elevation=%.6f preserve_same_plate_hit_count=%d preserve_same_plate_recovery_count=%d preserve_fallback_query_count=%d preserve_plate_changed_count=%d"),
						*Summary.RunId,
						Snapshot.Step,
						*FormatStepList(Snapshot.ResamplingSteps),
						GetResampleTriggerReasonName(Snapshot.LastTriggerReason),
						GetResampleOwnershipModeName(Snapshot.LastOwnershipMode),
						Snapshot.BoundarySampleCount,
						Snapshot.BoundarySampleFraction,
						Snapshot.GapCount,
						Snapshot.OverlapCount,
						Snapshot.ContinentalContinentalOverlapCount,
						Snapshot.ContinentalCount,
						Snapshot.ElevationMeanKm,
						Snapshot.PreserveSamePlateHitCount,
						Snapshot.PreserveSamePlateRecoveryCount,
						Snapshot.PreserveFallbackQueryCount,
						Snapshot.PreservePlateChangedCount));
				}
			}
		}

		Summary.FinalSnapshot = BuildM6CheckpointSnapshot(Planet);
		Checkpoints.Add(Config.MaxStep, Summary.FinalSnapshot);
		if (Config.MaxStep == 200)
		{
			Summary.Step200Snapshot = Summary.FinalSnapshot;
			Summary.bHasStep200Snapshot = true;
		}
		else if (Config.MaxStep == 100)
		{
			Summary.Step100Snapshot = Summary.FinalSnapshot;
			Summary.bHasStep100Snapshot = true;
		}
		else if (Config.MaxStep == 300)
		{
			Summary.Step300Snapshot = Summary.FinalSnapshot;
			Summary.bHasStep300Snapshot = true;
		}
		else if (Config.MaxStep == 400)
		{
			Summary.Step400Snapshot = Summary.FinalSnapshot;
			Summary.bHasStep400Snapshot = true;
		}
		Test.TestEqual(TEXT("M6 baseline keeps ownership changes outside resampling at zero"), Summary.OwnershipChangesOutsideResamplingCount, 0);

		Summary.TotalResamples = Planet.ResamplingHistory.Num();
		Summary.MinPlateCount = Summary.InitialSnapshot.PlateCount;
		Summary.MaxPlateCount = Summary.InitialSnapshot.PlateCount;
		Summary.MaxComponentsPerPlateObserved = Summary.InitialSnapshot.MaxComponentsPerPlate;
		Summary.MinContinentalAreaFraction = Summary.InitialSnapshot.ContinentalAreaFraction;
		Summary.MaxContinentalAreaFraction = Summary.InitialSnapshot.ContinentalAreaFraction;
		Summary.MinBoundarySampleFraction = Summary.InitialSnapshot.BoundarySampleFraction;
		Summary.MaxBoundarySampleFraction = Summary.InitialSnapshot.BoundarySampleFraction;
		Summary.MinGapRate = Summary.InitialSnapshot.GapRate;
		Summary.MaxGapRate = Summary.InitialSnapshot.GapRate;
		Summary.MinOverlapRate = Summary.InitialSnapshot.OverlapRate;
		Summary.MaxOverlapRate = Summary.InitialSnapshot.OverlapRate;
		Summary.MinMeanElevation = Summary.InitialSnapshot.ElevationMeanKm;
		Summary.MaxMeanElevation = Summary.InitialSnapshot.ElevationMeanKm;

		const auto IsGeometricSummaryStronger = [](const FResamplingStats& Left, const FResamplingStats& Right)
		{
			if (Left.bGeometricCollisionBestPassedMassFilter != Right.bGeometricCollisionBestPassedMassFilter)
			{
				return Left.bGeometricCollisionBestPassedMassFilter;
			}
			if (Left.GeometricCollisionBestOverlapSampleCount != Right.GeometricCollisionBestOverlapSampleCount)
			{
				return Left.GeometricCollisionBestOverlapSampleCount > Right.GeometricCollisionBestOverlapSampleCount;
			}
			if (Left.GeometricCollisionLargestTerraneEstimate != Right.GeometricCollisionLargestTerraneEstimate)
			{
				return Left.GeometricCollisionLargestTerraneEstimate > Right.GeometricCollisionLargestTerraneEstimate;
			}
			if (Left.GeometricCollisionBestPlateA != Right.GeometricCollisionBestPlateA)
			{
				return Left.GeometricCollisionBestPlateA < Right.GeometricCollisionBestPlateA;
			}
			if (Left.GeometricCollisionBestPlateB != Right.GeometricCollisionBestPlateB)
			{
				return Left.GeometricCollisionBestPlateB < Right.GeometricCollisionBestPlateB;
			}
			return Left.Step < Right.Step;
		};

		const auto IsBoundarySummaryStronger = [](const FResamplingStats& Left, const FResamplingStats& Right)
		{
			if (Left.BoundaryContactBestZoneSize != Right.BoundaryContactBestZoneSize)
			{
				return Left.BoundaryContactBestZoneSize > Right.BoundaryContactBestZoneSize;
			}
			if (Left.BoundaryContactBestPersistenceCount != Right.BoundaryContactBestPersistenceCount)
			{
				return Left.BoundaryContactBestPersistenceCount > Right.BoundaryContactBestPersistenceCount;
			}
			if (Left.BoundaryContactBestPlateA != Right.BoundaryContactBestPlateA)
			{
				return Left.BoundaryContactBestPlateA < Right.BoundaryContactBestPlateA;
			}
			if (Left.BoundaryContactBestPlateB != Right.BoundaryContactBestPlateB)
			{
				return Left.BoundaryContactBestPlateB < Right.BoundaryContactBestPlateB;
			}
			return Left.Step < Right.Step;
		};

		const auto IsExecutedCollisionStronger = [](const FResamplingStats& Left, const FResamplingStats& Right)
		{
			if (Left.CollisionTerraneSampleCount != Right.CollisionTerraneSampleCount)
			{
				return Left.CollisionTerraneSampleCount > Right.CollisionTerraneSampleCount;
			}
			if (Left.CollisionCWBoostedCount != Right.CollisionCWBoostedCount)
			{
				return Left.CollisionCWBoostedCount > Right.CollisionCWBoostedCount;
			}
			if (Left.CollisionSurgeAffectedCount != Right.CollisionSurgeAffectedCount)
			{
				return Left.CollisionSurgeAffectedCount > Right.CollisionSurgeAffectedCount;
			}
			if (Left.CollisionOverridingPlateId != Right.CollisionOverridingPlateId)
			{
				return Left.CollisionOverridingPlateId < Right.CollisionOverridingPlateId;
			}
			if (Left.CollisionSubductingPlateId != Right.CollisionSubductingPlateId)
			{
				return Left.CollisionSubductingPlateId < Right.CollisionSubductingPlateId;
			}
			return Left.Step < Right.Step;
		};

		const auto IsRepeatedGeometricCandidateStronger = [](const FResamplingStats& Left, const FResamplingStats& Right)
		{
			if (Left.GeometricCollisionRepeatedCandidateObservationCount != Right.GeometricCollisionRepeatedCandidateObservationCount)
			{
				return Left.GeometricCollisionRepeatedCandidateObservationCount >
					Right.GeometricCollisionRepeatedCandidateObservationCount;
			}
			if (!FMath::IsNearlyEqual(
					Left.GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm,
					Right.GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm,
					UE_DOUBLE_SMALL_NUMBER))
			{
				return Left.GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm >
					Right.GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm;
			}
			if (Left.GeometricCollisionRepeatedCandidatePlateA != Right.GeometricCollisionRepeatedCandidatePlateA)
			{
				return Left.GeometricCollisionRepeatedCandidatePlateA <
					Right.GeometricCollisionRepeatedCandidatePlateA;
			}
			if (Left.GeometricCollisionRepeatedCandidatePlateB != Right.GeometricCollisionRepeatedCandidatePlateB)
			{
				return Left.GeometricCollisionRepeatedCandidatePlateB <
					Right.GeometricCollisionRepeatedCandidatePlateB;
			}
			return Left.Step < Right.Step;
		};

		const auto GetPositiveTopologyComponentIncrease = [](const FResamplingStats& Stats)
		{
			return FMath::Max(0, Stats.TopologyGlobalMaxComponentsAfter - Stats.TopologyGlobalMaxComponentsBefore);
		};

		const auto IsTopologyIncreaseStronger = [&GetPositiveTopologyComponentIncrease](
			const FResamplingStats& Left,
			const FResamplingStats& Right)
		{
			const int32 LeftIncrease = GetPositiveTopologyComponentIncrease(Left);
			const int32 RightIncrease = GetPositiveTopologyComponentIncrease(Right);
			if (LeftIncrease != RightIncrease)
			{
				return LeftIncrease > RightIncrease;
			}
			if (Left.TopologyGlobalMaxComponentsAfter != Right.TopologyGlobalMaxComponentsAfter)
			{
				return Left.TopologyGlobalMaxComponentsAfter > Right.TopologyGlobalMaxComponentsAfter;
			}
			return Left.Step < Right.Step;
		};

		const auto IsTopologySpikeStronger = [](const FResamplingStats& Left, const FResamplingStats& Right)
		{
			if (Left.TopologyGlobalMaxComponentsAfter != Right.TopologyGlobalMaxComponentsAfter)
			{
				return Left.TopologyGlobalMaxComponentsAfter > Right.TopologyGlobalMaxComponentsAfter;
			}
			const int32 LeftIncrease =
				FMath::Max(0, Left.TopologyGlobalMaxComponentsAfter - Left.TopologyGlobalMaxComponentsBefore);
			const int32 RightIncrease =
				FMath::Max(0, Right.TopologyGlobalMaxComponentsAfter - Right.TopologyGlobalMaxComponentsBefore);
			if (LeftIncrease != RightIncrease)
			{
				return LeftIncrease > RightIncrease;
			}
			return Left.Step < Right.Step;
		};

		for (const FResamplingStats& Stats : Planet.ResamplingHistory)
		{
			Summary.TotalCollisionEvents += Stats.CollisionCount;
			Summary.TotalRiftEvents += Stats.RiftCount;
			if (Summary.FirstCollisionStep == INDEX_NONE && Stats.CollisionCount > 0)
			{
				Summary.FirstCollisionStep = Stats.Step;
			}
			if (Summary.FirstRiftStep == INDEX_NONE && Stats.RiftCount > 0)
			{
				Summary.FirstRiftStep = Stats.Step;
			}

			Summary.MinPlateCount = FMath::Min(Summary.MinPlateCount, Stats.PlateCount);
			Summary.MaxPlateCount = FMath::Max(Summary.MaxPlateCount, Stats.PlateCount);
			Summary.MaxComponentsPerPlateObserved = FMath::Max(Summary.MaxComponentsPerPlateObserved, Stats.MaxComponentsPerPlate);
			Summary.MinContinentalAreaFraction = FMath::Min(Summary.MinContinentalAreaFraction, Stats.ContinentalAreaFraction);
			Summary.MaxContinentalAreaFraction = FMath::Max(Summary.MaxContinentalAreaFraction, Stats.ContinentalAreaFraction);
			Summary.MinBoundarySampleFraction = FMath::Min(Summary.MinBoundarySampleFraction, Stats.BoundarySampleFraction);
			Summary.MaxBoundarySampleFraction = FMath::Max(Summary.MaxBoundarySampleFraction, Stats.BoundarySampleFraction);
			Summary.MinGapRate = FMath::Min(Summary.MinGapRate, Stats.GapRate);
			Summary.MaxGapRate = FMath::Max(Summary.MaxGapRate, Stats.GapRate);
			Summary.MinOverlapRate = FMath::Min(Summary.MinOverlapRate, Stats.OverlapRate);
			Summary.MaxOverlapRate = FMath::Max(Summary.MaxOverlapRate, Stats.OverlapRate);
			Summary.MinMeanElevation = FMath::Min(Summary.MinMeanElevation, Stats.ElevationMeanKm);
			Summary.MaxMeanElevation = FMath::Max(Summary.MaxMeanElevation, Stats.ElevationMeanKm);
			Summary.TotalContinentalSamplesLost += Stats.ContinentalSamplesLost;
			Summary.TotalFormerContinentalSamplesTurnedOceanic += Stats.FormerContinentalSamplesTurnedOceanic;
			Summary.TotalFormerContinentalDivergentGaps += Stats.FormerContinentalDivergentGapCount;
			Summary.TotalFormerContinentalNonDivergentGaps += Stats.FormerContinentalNonDivergentGapCount;
			Summary.TotalFormerContinentalNonDivergentGapProjectionResolved += Stats.FormerContinentalNonDivergentGapProjectionResolvedCount;
			Summary.TotalFormerContinentalNonDivergentGapNearestCopyResolved += Stats.FormerContinentalNonDivergentGapNearestCopyResolvedCount;
			Summary.TotalFormerContinentalNonDivergentFallbackOceanized += Stats.FormerContinentalNonDivergentFallbackOceanizedCount;
			Summary.TotalFormerContinentalDivergentOceanized += Stats.FormerContinentalDivergentOceanizedCount;
			Summary.TotalFormerContinentalProjectionRecoveredNonContinentalFinal += Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount;
			Summary.TotalFormerContinentalProjectionRecoveredSamePlateNonContinentalFinal += Stats.FormerContinentalProjectionRecoveredSamePlateNonContinentalFinalCount;
			Summary.TotalFormerContinentalProjectionRecoveredChangedPlateNonContinentalFinal += Stats.FormerContinentalProjectionRecoveredChangedPlateNonContinentalFinalCount;
			Summary.TotalFormerContinentalProjectionRecoveredPreserveModeNonContinentalFinal += Stats.FormerContinentalProjectionRecoveredPreserveModeNonContinentalFinalCount;
			Summary.TotalFormerContinentalProjectionRecoveredFullResolutionNonContinentalFinal += Stats.FormerContinentalProjectionRecoveredFullResolutionNonContinentalFinalCount;
			Summary.TotalFormerContinentalNearestCopyRecoveredNonContinentalFinal += Stats.FormerContinentalNearestCopyRecoveredNonContinentalFinalCount;
			Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal += Stats.FormerContinentalNonGapReclassifiedNonContinentalFinalCount;
			Summary.TotalFormerContinentalChangedPlateNonContinentalFinal += Stats.FormerContinentalChangedPlateNonContinentalFinalCount;
			Summary.TotalFormerContinentalFullResolutionNonContinentalFinal += Stats.FormerContinentalFullResolutionNonContinentalFinalCount;
			Summary.TotalFullResolutionSamePlateNonContinentalFinal += Stats.FullResolutionSamePlateNonContinentalFinalCount;
			Summary.TotalFullResolutionChangedPlateNonContinentalFinal += Stats.FullResolutionChangedPlateNonContinentalFinalCount;
			Summary.TotalFullResolutionCollisionFollowupSamePlateNonContinental +=
				Stats.FullResolutionCollisionFollowupSamePlateNonContinentalCount;
			Summary.TotalFullResolutionCollisionFollowupChangedPlateNonContinental +=
				Stats.FullResolutionCollisionFollowupChangedPlateNonContinentalCount;
			Summary.TotalFullResolutionRiftFollowupSamePlateNonContinental +=
				Stats.FullResolutionRiftFollowupSamePlateNonContinentalCount;
			Summary.TotalFullResolutionRiftFollowupChangedPlateNonContinental +=
				Stats.FullResolutionRiftFollowupChangedPlateNonContinentalCount;
			Summary.TotalFullResolutionOtherTriggerSamePlateNonContinental +=
				Stats.FullResolutionOtherTriggerSamePlateNonContinentalCount;
			Summary.TotalFullResolutionOtherTriggerChangedPlateNonContinental +=
				Stats.FullResolutionOtherTriggerChangedPlateNonContinentalCount;
			Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal += Stats.FormerContinentalPreserveModeNonGapNonContinentalFinalCount;
			Summary.TotalFormerContinentalFallbackQueryNonContinentalFinal += Stats.FormerContinentalFallbackQueryNonContinentalFinalCount;
			Summary.TotalPreserveOwnershipFallbackSamePlateRecontainedCount +=
				Stats.PreserveOwnershipFallbackSamePlateRecontainedCount;
			Summary.TotalPreserveOwnershipFallbackSamePlateRetainedCount +=
				Stats.PreserveOwnershipFallbackSamePlateRetainedCount;
			Summary.TotalPreserveOwnershipFallbackChangedOwnerCount +=
				Stats.PreserveOwnershipFallbackChangedOwnerCount;
			Summary.TotalPreserveOwnershipFallbackGapCount +=
				Stats.PreserveOwnershipFallbackGapCount;
			Summary.TotalPreserveOwnershipFallbackDivergentOceanizationCount +=
				Stats.PreserveOwnershipFallbackDivergentOceanizationCount;
			Summary.TotalPreserveOwnershipContinentalLossCountAfterFallback +=
				Stats.PreserveOwnershipContinentalLossCountAfterFallback;
			Summary.TotalPreserveOwnershipPreviousOwnerHysteresisApplicationCount +=
				Stats.PreserveOwnershipPreviousOwnerHysteresisApplicationCount;
			Summary.TotalPreserveOwnershipStronglyContinentalBoundarySavedCount +=
				Stats.PreserveOwnershipStronglyContinentalBoundarySavedCount;
			Summary.TotalPreserveOwnershipFallbackChangedOwnerNonGapLossCount +=
				Stats.PreserveOwnershipFallbackChangedOwnerNonGapLossCount;
			Summary.TotalPreserveOwnershipFallbackDivergentLossCount +=
				Stats.PreserveOwnershipFallbackDivergentLossCount;
			Summary.TotalPreserveOwnershipFallbackNonDivergentProjectionLossCount +=
				Stats.PreserveOwnershipFallbackNonDivergentProjectionLossCount;
			Summary.TotalPreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount +=
				Stats.PreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount;
			Summary.TotalPreserveOwnershipFallbackStrongLossGE090Count +=
				Stats.PreserveOwnershipFallbackStrongLossGE090Count;
			Summary.TotalPreserveOwnershipFallbackStrongLossGE075Count +=
				Stats.PreserveOwnershipFallbackStrongLossGE075Count;
			Summary.TotalPreserveOwnershipFallbackStrongLossGE050Count +=
				Stats.PreserveOwnershipFallbackStrongLossGE050Count;
			Summary.TotalFullResolutionSamePlateCWRetained += Stats.FullResolutionSamePlateCWRetainedCount;
			Summary.TotalFullResolutionSamePlateCWThresholdCrossingPrevented +=
				Stats.FullResolutionSamePlateCWThresholdCrossingPreventedCount;
			Summary.TotalFullResolutionCollisionFollowupSamePlateCWRetained +=
				Stats.FullResolutionCollisionFollowupSamePlateCWRetainedCount;
			Summary.TotalFullResolutionRiftFollowupSamePlateCWRetained +=
				Stats.FullResolutionRiftFollowupSamePlateCWRetainedCount;
			Summary.TotalFullResolutionOtherTriggerSamePlateCWRetained +=
				Stats.FullResolutionOtherTriggerSamePlateCWRetainedCount;
			Summary.TotalAndeanContinentalGains += Stats.AndeanContinentalGainCount;
			Summary.TotalCollisionContinentalGains += Stats.CollisionContinentalGainCount;
			Summary.TotalNetContinentalSampleDelta += Stats.NetContinentalSampleDelta;
			if (Stats.Step <= 200)
			{
				Summary.FirstHalfContinentalSamplesLost += Stats.ContinentalSamplesLost;
				Summary.FirstHalfAndeanContinentalGains += Stats.AndeanContinentalGainCount;
				Summary.FirstHalfCollisionContinentalGains += Stats.CollisionContinentalGainCount;
				Summary.FirstHalfNetContinentalSampleDelta += Stats.NetContinentalSampleDelta;
			}
			else
			{
				Summary.SecondHalfContinentalSamplesLost += Stats.ContinentalSamplesLost;
				Summary.SecondHalfAndeanContinentalGains += Stats.AndeanContinentalGainCount;
				Summary.SecondHalfCollisionContinentalGains += Stats.CollisionContinentalGainCount;
				Summary.SecondHalfNetContinentalSampleDelta += Stats.NetContinentalSampleDelta;
			}

			const int32 TopologyIncrease = GetPositiveTopologyComponentIncrease(Stats);
			const int32 PrimaryDelta =
				Stats.TopologyPrimaryAffectedPlateComponentsAfter - Stats.TopologyPrimaryAffectedPlateComponentsBefore;
			const int32 SecondaryDelta =
				Stats.TopologySecondaryAffectedPlateComponentsAfter - Stats.TopologySecondaryAffectedPlateComponentsBefore;
			const int64 ConsolidationMagnitude =
				static_cast<int64>(FMath::Max(0, -PrimaryDelta)) +
				static_cast<int64>(FMath::Max(0, -SecondaryDelta));
			if (Stats.TriggerReason == EResampleTriggerReason::CollisionFollowup)
			{
				Summary.TotalCollisionFollowupComponentIncrease += TopologyIncrease;
				if (Stats.Step <= 100)
				{
					Summary.CollisionFollowupComponentIncrease0To100 += TopologyIncrease;
				}
				else if (Stats.Step <= 200)
				{
					Summary.CollisionFollowupComponentIncrease100To200 += TopologyIncrease;
				}
				else
				{
					Summary.CollisionFollowupComponentIncrease200To400 += TopologyIncrease;
				}
				if (ConsolidationMagnitude > 0)
				{
					++Summary.CollisionFollowupConsolidationEventCount;
					Summary.CollisionFollowupAffectedPlateComponentDecrease += ConsolidationMagnitude;
				}
				Summary.TotalCollisionTrimmedByDonorProtectionCount +=
					Stats.CollisionTrimmedByDonorProtectionCount;
				Summary.TotalCollisionRejectedByDonorProtectionCount +=
					Stats.CollisionRejectedByDonorProtectionCount;
				Summary.TotalCollisionTrimmedByDonorComponentCapCount +=
					Stats.CollisionTrimmedByDonorComponentCapCount;
				Summary.TotalCollisionTrimmedByDonorFragmentFloorCount +=
					Stats.CollisionTrimmedByDonorFragmentFloorCount;
				Summary.TotalCollisionRejectedByDonorComponentCapCount +=
					Stats.CollisionRejectedByDonorComponentCapCount;
				Summary.TotalCollisionRejectedByDonorFragmentFloorCount +=
					Stats.CollisionRejectedByDonorFragmentFloorCount;
				Summary.TotalCollisionTrimmedByReceiverProtectionCount +=
					Stats.CollisionTrimmedByReceiverProtectionCount;
				Summary.TotalCollisionRejectedByReceiverProtectionCount +=
					Stats.CollisionRejectedByReceiverProtectionCount;
				if (Stats.CollisionProposedTerraneSampleCount > 0)
				{
					const int32 ProposedDonorDelta = FMath::Max(
						0,
						Stats.CollisionProposedDonorComponentsAfter -
							Stats.CollisionProposedDonorComponentsBefore);
					const int32 AcceptedDonorDelta = Stats.CollisionCount > 0
						? FMath::Max(
							0,
							Stats.CollisionDonorComponentsAfter -
								Stats.CollisionDonorComponentsBefore)
						: 0;
					++Summary.TotalCollisionTransferDiagnosticsCount;
					Summary.TotalCollisionProposedTerraneSampleCount +=
						Stats.CollisionProposedTerraneSampleCount;
					Summary.MaxCollisionProposedTerraneSampleCount = FMath::Max(
						Summary.MaxCollisionProposedTerraneSampleCount,
						Stats.CollisionProposedTerraneSampleCount);
					Summary.TotalCollisionAcceptedTerraneSampleCount +=
						Stats.CollisionAcceptedTerraneSampleCount;
					Summary.MaxCollisionAcceptedTerraneSampleCount = FMath::Max(
						Summary.MaxCollisionAcceptedTerraneSampleCount,
						Stats.CollisionAcceptedTerraneSampleCount);
					Summary.TotalCollisionTransferTrimRatio +=
						Stats.CollisionTransferTrimRatio;
					Summary.MaxCollisionTransferTrimRatio = FMath::Max(
						Summary.MaxCollisionTransferTrimRatio,
						Stats.CollisionTransferTrimRatio);
					Summary.TotalCollisionTransferProposedDonorComponentIncrease +=
						ProposedDonorDelta;
					Summary.MaxCollisionTransferProposedDonorComponentIncrease = FMath::Max(
						Summary.MaxCollisionTransferProposedDonorComponentIncrease,
						ProposedDonorDelta);
					Summary.TotalCollisionTransferAcceptedDonorComponentIncrease +=
						AcceptedDonorDelta;
					Summary.MaxCollisionTransferAcceptedDonorComponentIncrease = FMath::Max(
						Summary.MaxCollisionTransferAcceptedDonorComponentIncrease,
						AcceptedDonorDelta);
				}
				if (Stats.CollisionCount > 0)
				{
					const int32 ProposedDonorDelta = FMath::Max(
						0,
						Stats.CollisionProposedDonorComponentsAfter -
							Stats.CollisionProposedDonorComponentsBefore);
					const int32 ProposedReceiverDelta = FMath::Max(
						0,
						Stats.CollisionProposedReceiverComponentsAfter -
							Stats.CollisionProposedReceiverComponentsBefore);
					const int32 AppliedDonorDelta = FMath::Max(
						0,
						Stats.CollisionDonorComponentsAfter -
							Stats.CollisionDonorComponentsBefore);
					const int32 AppliedReceiverDelta = FMath::Max(
						0,
						Stats.CollisionReceiverComponentsAfter -
							Stats.CollisionReceiverComponentsBefore);
					Summary.TotalCollisionProposedDonorComponentIncrease += ProposedDonorDelta;
					Summary.TotalCollisionProposedReceiverComponentIncrease += ProposedReceiverDelta;
					Summary.TotalCollisionAppliedDonorComponentIncrease += AppliedDonorDelta;
					Summary.TotalCollisionAppliedReceiverComponentIncrease += AppliedReceiverDelta;
					Summary.MaxCollisionProposedDonorComponentIncrease = FMath::Max(
						Summary.MaxCollisionProposedDonorComponentIncrease,
						ProposedDonorDelta);
					Summary.MaxCollisionProposedReceiverComponentIncrease = FMath::Max(
						Summary.MaxCollisionProposedReceiverComponentIncrease,
						ProposedReceiverDelta);
					Summary.MaxCollisionAppliedDonorComponentIncrease = FMath::Max(
						Summary.MaxCollisionAppliedDonorComponentIncrease,
						AppliedDonorDelta);
					Summary.MaxCollisionAppliedReceiverComponentIncrease = FMath::Max(
						Summary.MaxCollisionAppliedReceiverComponentIncrease,
						AppliedReceiverDelta);
				}
				if (Stats.bUsedGeometricCollisionExecution)
				{
					Summary.TotalGeometricCollisionReceiverContiguousCount +=
						Stats.bCollisionReceiverTransferContiguousWithExistingTerritory ? 1 : 0;
					if (Stats.CollisionReceiverDisconnectedFragmentCount > 0)
					{
						++Summary.TotalGeometricCollisionReceiverDisconnectedEventCount;
					}
					Summary.TotalGeometricCollisionReceiverDisconnectedFragmentCount +=
						Stats.CollisionReceiverDisconnectedFragmentCount;
					Summary.MaxGeometricCollisionReceiverDisconnectedFragmentCount = FMath::Max(
						Summary.MaxGeometricCollisionReceiverDisconnectedFragmentCount,
						Stats.CollisionReceiverDisconnectedFragmentCount);
					Summary.MaxGeometricCollisionReceiverLargestNewDisconnectedFragmentSize = FMath::Max(
						Summary.MaxGeometricCollisionReceiverLargestNewDisconnectedFragmentSize,
						Stats.CollisionReceiverLargestNewDisconnectedFragmentSize);
				}
			}
				else if (Stats.TriggerReason == EResampleTriggerReason::RiftFollowup)
				{
					Summary.TotalRiftFollowupComponentIncrease += TopologyIncrease;
					Summary.TotalRiftLocalizedNonChildSampleRestoreCount +=
						Stats.RiftLocalizedNonChildSampleRestoreCount;
					Summary.TotalRiftLocalizedGapSampleRestoreCount +=
						Stats.RiftLocalizedGapSampleRestoreCount;
					Summary.TotalRiftLocalizedCWRestoredCount +=
						Stats.RiftLocalizedCWRestoredCount;
					Summary.TotalRiftLocalizedCWPhantomPreventedCount +=
						Stats.RiftLocalizedCWPhantomPreventedCount;
					Summary.TotalRiftLocalizedCWContinentalPreventedCount +=
						Stats.RiftLocalizedCWContinentalPreventedCount;
					Summary.TotalRiftInterpolationCreatedGainCount +=
						Stats.RiftInterpolationCreatedGainCount;
					Summary.TotalRiftFinalOwnerMismatchGainCountAfterLocalization +=
						Stats.RiftFinalOwnerMismatchGainCountAfterLocalization;
					Summary.TotalRiftFinalOwnerCWReconciledCount +=
						Stats.RiftFinalOwnerCWReconciledCount;
					Summary.TotalRiftFinalOwnerMismatchGainCountBeforeReconciliation +=
						Stats.RiftFinalOwnerMismatchGainCountBeforeReconciliation;
					Summary.TotalRiftFinalOwnerMismatchGainCountAfterReconciliation +=
						Stats.RiftFinalOwnerMismatchGainCountAfterReconciliation;
					Summary.TotalRiftFinalOwnerMismatchContinentalPreventedCount +=
						Stats.RiftFinalOwnerMismatchContinentalPreventedCount;
					Summary.TotalRiftSameOwnerChildInterpolationGainCountBeforeReconciliation +=
						Stats.RiftSameOwnerChildInterpolationGainCountBeforeReconciliation;
					Summary.TotalRiftSameOwnerChildInterpolationGainCountAfterReconciliation +=
						Stats.RiftSameOwnerChildInterpolationGainCountAfterReconciliation;
					Summary.TotalRiftFinalGainStartedBelow025Count +=
						Stats.RiftFinalGainStartedBelow025Count;
					Summary.TotalRiftFinalGainStartedBelow040Count +=
						Stats.RiftFinalGainStartedBelow040Count;
					Summary.TotalRiftFinalGainStartedBelow050Count +=
						Stats.RiftFinalGainStartedBelow050Count;
					Summary.TotalRiftChildStrayFragmentDetectedCount +=
						Stats.RiftChildStrayFragmentDetectedCount;
					Summary.TotalRiftChildStrayFragmentReassignedCount +=
						Stats.RiftChildStrayFragmentReassignedCount;
				Summary.MaxRiftLargestStrayChildFragmentSize = FMath::Max(
					Summary.MaxRiftLargestStrayChildFragmentSize,
					Stats.RiftLargestStrayChildFragmentSize);
				Summary.TotalRiftStrayFragmentReassignedToSiblingCount +=
					Stats.RiftStrayChildFragmentReassignedToSiblingCount;
				Summary.TotalRiftStrayFragmentReassignedToOtherNeighborCount +=
					Stats.RiftStrayChildFragmentReassignedToOtherNeighborCount;
				Summary.TotalRiftStrayFragmentReassignedByAdjacentSiblingCount +=
					Stats.RiftStrayFragmentReassignedByAdjacentSiblingCount;
				Summary.TotalRiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount +=
					Stats.RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount;
				Summary.TotalRiftStrayFragmentForcedNonChildAssignmentCount +=
					Stats.RiftStrayFragmentForcedNonChildAssignmentCount;
				Summary.TotalRiftStrayFragmentZeroSiblingAdjacencyCount +=
					Stats.RiftStrayFragmentZeroSiblingAdjacencyCount;
				Summary.TotalRiftStrayFragmentPositiveSiblingAdjacencyCount +=
					Stats.RiftStrayFragmentPositiveSiblingAdjacencyCount;
				Summary.TotalRiftStrayFragmentRecipientCandidateConsideredCount +=
					Stats.RiftStrayFragmentRecipientCandidateConsideredCount;
				Summary.TotalRiftStrayFragmentRecipientIncoherenceRejectedCount +=
					Stats.RiftStrayFragmentRecipientIncoherenceRejectedCount;
				Summary.TotalRiftStrayFragmentIncoherentForcedAssignmentCount +=
					Stats.RiftStrayFragmentIncoherentForcedAssignmentCount;
					Summary.MaxRiftLargestFragmentCausingRecipientGrowthSize = FMath::Max(
						Summary.MaxRiftLargestFragmentCausingRecipientGrowthSize,
						Stats.RiftLargestFragmentCausingRecipientGrowthSize);
					if (Stats.Step == 229)
					{
						Summary.Step229RiftFollowupStats = Stats;
						Summary.bHasStep229RiftFollowupStats = true;
					}
					if (Stats.Step <= 100)
					{
						++Summary.RiftFollowupCount0To100;
					Summary.RiftFollowupComponentIncrease0To100 += TopologyIncrease;
				}
				else if (Stats.Step <= 200)
				{
					++Summary.RiftFollowupCount100To200;
					Summary.RiftFollowupComponentIncrease100To200 += TopologyIncrease;
				}
				else
				{
					++Summary.RiftFollowupCount200To400;
					Summary.RiftFollowupComponentIncrease200To400 += TopologyIncrease;
				}
				if (ConsolidationMagnitude > 0)
				{
					++Summary.RiftFollowupConsolidationEventCount;
					Summary.RiftFollowupAffectedPlateComponentDecrease += ConsolidationMagnitude;
				}
				if (TopologyIncrease > 0 &&
					(!Summary.bHasLargestRiftTopologyIncreaseEvent ||
						IsTopologyIncreaseStronger(Stats, Summary.LargestRiftTopologyIncreaseEventStats)))
				{
					Summary.LargestRiftTopologyIncreaseEventStats = Stats;
					Summary.bHasLargestRiftTopologyIncreaseEvent = true;
				}
			}

			const bool bIsTopologyFollowup =
				Stats.TriggerReason == EResampleTriggerReason::CollisionFollowup ||
				Stats.TriggerReason == EResampleTriggerReason::RiftFollowup;
			if (bIsTopologyFollowup &&
				TopologyIncrease > 0 &&
				(!Summary.bHasLargestTopologyIncreaseEvent ||
					IsTopologyIncreaseStronger(Stats, Summary.LargestTopologyIncreaseEventStats)))
			{
				Summary.LargestTopologyIncreaseEventStats = Stats;
				Summary.bHasLargestTopologyIncreaseEvent = true;
			}
			if (bIsTopologyFollowup &&
				(!Summary.bHasPeakTopologySpikeEvent ||
					IsTopologySpikeStronger(Stats, Summary.PeakTopologySpikeEventStats)))
			{
				Summary.PeakTopologySpikeEventStats = Stats;
				Summary.bHasPeakTopologySpikeEvent = true;
			}

			Summary.TotalPreserveOwnershipCWRetained += Stats.PreserveOwnershipCWRetainedCount;
			Summary.TotalPreserveOwnershipCWThresholdCrossingPrevented += Stats.PreserveOwnershipCWThresholdCrossingPreventedCount;
			Summary.TotalGeometricCollisionOverlapSamples += Stats.GeometricCollisionOverlapSampleCount;
			Summary.TotalGeometricCollisionPairCount += Stats.GeometricCollisionPairCount;
			Summary.TotalGeometricCollisionQualifiedPairCount += Stats.GeometricCollisionQualifiedPairCount;
			Summary.TotalGeometricCollisionCandidateCount += Stats.GeometricCollisionCandidateCount;
			Summary.TotalGeometricCollisionQualifiedCount += Stats.GeometricCollisionQualifiedCount;
			Summary.TotalGeometricCollisionQualifiedButDonorAmbiguousCount += Stats.GeometricCollisionQualifiedButDonorAmbiguousCount;
			Summary.TotalGeometricCollisionQualifiedButDonorSeedEmptyCount += Stats.GeometricCollisionQualifiedButDonorSeedEmptyCount;
			Summary.TotalGeometricCollisionQualifiedUsingDirectionalDonorCount += Stats.GeometricCollisionQualifiedUsingDirectionalDonorCount;
			Summary.TotalGeometricCollisionQualifiedUsingFallbackDonorRuleCount += Stats.GeometricCollisionQualifiedUsingFallbackDonorRuleCount;
			Summary.TotalGeometricCollisionRejectedByMassFilterCount += Stats.GeometricCollisionRejectedByMassFilterCount;
			Summary.TotalGeometricCollisionRejectedByOverlapDepthCount += Stats.GeometricCollisionRejectedByOverlapDepthCount;
			Summary.TotalGeometricCollisionRejectedByOverlapDepthOverlapSampleCount +=
				Stats.GeometricCollisionRejectedByOverlapDepthTotalOverlapSampleCount;
			Summary.MaxGeometricCollisionRejectedByOverlapDepthOverlapSampleCount = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByOverlapDepthOverlapSampleCount,
				Stats.GeometricCollisionRejectedByOverlapDepthMaxOverlapSampleCount);
			Summary.TotalGeometricCollisionRejectedByOverlapDepthTerraneEstimate +=
				Stats.GeometricCollisionRejectedByOverlapDepthTotalTerraneEstimate;
			Summary.MaxGeometricCollisionRejectedByOverlapDepthTerraneEstimate = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByOverlapDepthTerraneEstimate,
				Stats.GeometricCollisionRejectedByOverlapDepthMaxTerraneEstimate);
			Summary.TotalGeometricCollisionRejectedByOverlapDepthMeanConvergenceKmPerMy +=
				Stats.GeometricCollisionRejectedByOverlapDepthTotalMeanConvergenceKmPerMy;
			Summary.MaxGeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy,
				Stats.GeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy);
			Summary.TotalGeometricCollisionRejectedByOverlapDepthKm +=
				Stats.GeometricCollisionRejectedByOverlapDepthTotalDepthKm;
			Summary.MaxGeometricCollisionRejectedByOverlapDepthKm = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByOverlapDepthKm,
				Stats.GeometricCollisionRejectedByOverlapDepthMaxDepthKm);
			Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount +=
				Stats.GeometricCollisionRejectedByPersistentPenetrationCount;
			Summary.TotalGeometricCollisionRejectedByPersistentPenetrationOverlapSampleCount +=
				Stats.GeometricCollisionRejectedByPersistentPenetrationTotalOverlapSampleCount;
			Summary.MaxGeometricCollisionRejectedByPersistentPenetrationOverlapSampleCount = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationOverlapSampleCount,
				Stats.GeometricCollisionRejectedByPersistentPenetrationMaxOverlapSampleCount);
			Summary.TotalGeometricCollisionRejectedByPersistentPenetrationTerraneEstimate +=
				Stats.GeometricCollisionRejectedByPersistentPenetrationTotalTerraneEstimate;
			Summary.MaxGeometricCollisionRejectedByPersistentPenetrationTerraneEstimate = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationTerraneEstimate,
				Stats.GeometricCollisionRejectedByPersistentPenetrationMaxTerraneEstimate);
			Summary.TotalGeometricCollisionRejectedByPersistentPenetrationObservationCount +=
				Stats.GeometricCollisionRejectedByPersistentPenetrationTotalObservationCount;
			Summary.MaxGeometricCollisionRejectedByPersistentPenetrationObservationCount = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationObservationCount,
				Stats.GeometricCollisionRejectedByPersistentPenetrationMaxObservationCount);
			Summary.TotalGeometricCollisionRejectedByPersistentPenetrationMeanConvergenceKmPerMy +=
				Stats.GeometricCollisionRejectedByPersistentPenetrationTotalMeanConvergenceKmPerMy;
			Summary.MaxGeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy,
				Stats.GeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy);
			Summary.TotalGeometricCollisionRejectedByPersistentPenetrationAccumulatedPenetrationKm +=
				Stats.GeometricCollisionRejectedByPersistentPenetrationTotalAccumulatedPenetrationKm;
			Summary.MaxGeometricCollisionRejectedByPersistentPenetrationAccumulatedPenetrationKm = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationAccumulatedPenetrationKm,
				Stats.GeometricCollisionRejectedByPersistentPenetrationMaxAccumulatedPenetrationKm);
			Summary.TotalGeometricCollisionRejectedByPersistentPenetrationDepthKm +=
				Stats.GeometricCollisionRejectedByPersistentPenetrationTotalDepthKm;
			Summary.MaxGeometricCollisionRejectedByPersistentPenetrationDepthKm = FMath::Max(
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationDepthKm,
				Stats.GeometricCollisionRejectedByPersistentPenetrationMaxDepthKm);
			Summary.TotalGeometricCollisionRejectedByEmptyTerraneCount += Stats.GeometricCollisionRejectedByEmptyTerraneCount;
			Summary.TotalGeometricCollisionRejectedByRoleResolutionCount += Stats.GeometricCollisionRejectedByRoleResolutionCount;
			Summary.TotalGeometricCollisionRejectedBySeedDirectionCount += Stats.GeometricCollisionRejectedBySeedDirectionCount;
			Summary.TotalGeometricCollisionQualifiedDirectionalCount += Stats.GeometricCollisionQualifiedDirectionalCount;
			Summary.TotalGeometricCollisionDirectionalSeedCount += Stats.GeometricCollisionDirectionalSeedCount;
			Summary.TotalGeometricCollisionDirectionalSeedCountOpposite += Stats.GeometricCollisionDirectionalSeedCountOpposite;
			Summary.TotalGeometricCollisionSeedDirectionAuditCount += Stats.GeometricCollisionSeedDirectionAuditCount;
			Summary.TotalGeometricCollisionSeedDirectionMismatchCount += Stats.GeometricCollisionSeedDirectionMismatchCount;
			Summary.TotalGeometricCollisionOnlyOverridingBucketPopulatedCount += Stats.GeometricCollisionOnlyOverridingBucketPopulatedCount;
			Summary.TotalGeometricCollisionOnlySubductingBucketPopulatedCount += Stats.GeometricCollisionOnlySubductingBucketPopulatedCount;
			Summary.TotalGeometricCollisionBothDirectionalBucketsPopulatedCount += Stats.GeometricCollisionBothDirectionalBucketsPopulatedCount;
			Summary.TotalGeometricCollisionNeitherDirectionalBucketsPopulatedCount += Stats.GeometricCollisionNeitherDirectionalBucketPopulatedCount;
			Summary.TotalGeometricCollisionFallbackUsed += Stats.GeometricCollisionFallbackUsedCount;
			Summary.TotalPreserveResamplesWithGeometricCandidates += Stats.GeometricCollisionCandidateCount > 0 ? 1 : 0;
			Summary.TotalPreserveResamplesWithQualifiedGeometricCandidates += Stats.GeometricCollisionQualifiedCount > 0 ? 1 : 0;
			Summary.LargestGeometricTerraneEstimateObserved = FMath::Max(
				Summary.LargestGeometricTerraneEstimateObserved,
				Stats.GeometricCollisionLargestTerraneEstimate);

			if (Stats.CollisionCount > 0)
			{
				Summary.TotalCollisionTerraneRecovered += Stats.CollisionTerraneSampleCount;
				Summary.MaxCollisionTerraneRecovered = FMath::Max(Summary.MaxCollisionTerraneRecovered, Stats.CollisionTerraneSampleCount);
				Summary.TotalCollisionCWBoosted += Stats.CollisionCWBoostedCount;
				Summary.MaxCollisionCWBoosted = FMath::Max(Summary.MaxCollisionCWBoosted, Stats.CollisionCWBoostedCount);
				Summary.TotalCollisionSurgeAffected += Stats.CollisionSurgeAffectedCount;
				Summary.MaxCollisionSurgeAffected = FMath::Max(Summary.MaxCollisionSurgeAffected, Stats.CollisionSurgeAffectedCount);
				if (Stats.bUsedGeometricCollisionExecution)
				{
					Summary.TotalGeometricCollisionExecutionCollisionGains +=
						Stats.GeometricCollisionExecutedCollisionGainCount;
					Summary.TotalGeometricCollisionExecutionTerraneRecovered +=
						Stats.GeometricCollisionExecutedTerraneRecoveredCount;
					Summary.MaxGeometricCollisionExecutionTerraneRecovered = FMath::Max(
						Summary.MaxGeometricCollisionExecutionTerraneRecovered,
						Stats.GeometricCollisionExecutedTerraneRecoveredCount);
					Summary.TotalGeometricCollisionExecutionCWBoosted +=
						Stats.GeometricCollisionExecutedCWBoostedCount;
					Summary.MaxGeometricCollisionExecutionCWBoosted = FMath::Max(
						Summary.MaxGeometricCollisionExecutionCWBoosted,
						Stats.GeometricCollisionExecutedCWBoostedCount);
					Summary.TotalGeometricCollisionExecutionSurgeAffected +=
						Stats.GeometricCollisionExecutedSurgeAffectedCount;
					Summary.MaxGeometricCollisionExecutionSurgeAffected = FMath::Max(
						Summary.MaxGeometricCollisionExecutionSurgeAffected,
						Stats.GeometricCollisionExecutedSurgeAffectedCount);
				}
				if (Stats.bUsedCachedBoundaryContactCollision)
				{
					Summary.TotalBoundaryContactFallbackCollisionGains +=
						Stats.BoundaryContactFallbackCollisionGainCount;
					Summary.TotalBoundaryContactFallbackTerraneRecovered +=
						Stats.BoundaryContactFallbackTerraneRecoveredCount;
					Summary.MaxBoundaryContactFallbackTerraneRecovered = FMath::Max(
						Summary.MaxBoundaryContactFallbackTerraneRecovered,
						Stats.BoundaryContactFallbackTerraneRecoveredCount);
					Summary.TotalBoundaryContactFallbackCWBoosted +=
						Stats.BoundaryContactFallbackCWBoostedCount;
					Summary.MaxBoundaryContactFallbackCWBoosted = FMath::Max(
						Summary.MaxBoundaryContactFallbackCWBoosted,
						Stats.BoundaryContactFallbackCWBoostedCount);
					Summary.TotalBoundaryContactFallbackSurgeAffected +=
						Stats.BoundaryContactFallbackSurgeAffectedCount;
					Summary.MaxBoundaryContactFallbackSurgeAffected = FMath::Max(
						Summary.MaxBoundaryContactFallbackSurgeAffected,
						Stats.BoundaryContactFallbackSurgeAffectedCount);
				}
				Summary.TotalGeometricCollisionExecutedOverlapSampleCount +=
					Stats.GeometricCollisionExecutedOverlapSampleCount;
				Summary.MaxGeometricCollisionExecutedOverlapSampleCount = FMath::Max(
					Summary.MaxGeometricCollisionExecutedOverlapSampleCount,
					Stats.GeometricCollisionExecutedOverlapSampleCount);
				Summary.TotalGeometricCollisionExecutedTerraneEstimate +=
					Stats.GeometricCollisionExecutedTerraneEstimate;
				Summary.MaxGeometricCollisionExecutedTerraneEstimate = FMath::Max(
					Summary.MaxGeometricCollisionExecutedTerraneEstimate,
					Stats.GeometricCollisionExecutedTerraneEstimate);
				Summary.TotalGeometricCollisionExecutedMeanConvergenceKmPerMy +=
					Stats.GeometricCollisionExecutedMeanConvergenceKmPerMy;
				Summary.MaxGeometricCollisionExecutedMaxConvergenceKmPerMy = FMath::Max(
					Summary.MaxGeometricCollisionExecutedMaxConvergenceKmPerMy,
					Stats.GeometricCollisionExecutedMaxConvergenceKmPerMy);
				Summary.TotalGeometricCollisionExecutions += Stats.bUsedGeometricCollisionExecution ? 1 : 0;
				Summary.TotalBoundaryContactFallbackExecutions += Stats.bUsedCachedBoundaryContactCollision ? 1 : 0;
				Summary.TotalGeometricCollisionExecutedFromDirectionalPolarityCount +=
					Stats.GeometricCollisionExecutedFromDirectionalPolarityCount;
				Summary.TotalGeometricCollisionExecutedOverlapDepthKm += Stats.GeometricCollisionExecutedOverlapDepthKm;
				Summary.MaxGeometricCollisionExecutedOverlapDepthKm = FMath::Max(
					Summary.MaxGeometricCollisionExecutedOverlapDepthKm,
					Stats.GeometricCollisionExecutedMaxOverlapDepthKm);
				Summary.TotalGeometricCollisionExecutedAccumulatedPenetrationKm +=
					Stats.GeometricCollisionExecutedAccumulatedPenetrationKm;
				Summary.MaxGeometricCollisionExecutedAccumulatedPenetrationKm = FMath::Max(
					Summary.MaxGeometricCollisionExecutedAccumulatedPenetrationKm,
					Stats.GeometricCollisionExecutedAccumulatedPenetrationKm);
				Summary.TotalGeometricCollisionExecutedObservationCount +=
					Stats.GeometricCollisionExecutedObservationCount;
				Summary.MaxGeometricCollisionExecutedObservationCount = FMath::Max(
					Summary.MaxGeometricCollisionExecutedObservationCount,
					Stats.GeometricCollisionExecutedObservationCount);
				Summary.TotalGeometricCollisionExecutedAfterMultipleObservationsCount +=
					(Stats.bUsedGeometricCollisionExecution &&
						Stats.GeometricCollisionExecutedObservationCount > 1) ? 1 : 0;
				Summary.TotalGeometricCollisionExecutedDonorTerraneLocalityLimitedCount +=
					Stats.GeometricCollisionExecutedDonorTerraneLocalityLimitedCount;
				Summary.TotalCollisionRepeatedPairCount += Stats.CollisionRepeatedPairCount;
				Summary.TotalCollisionRepeatedPairWithinCooldownCount +=
					Stats.CollisionRepeatedPairWithinCooldownCount;
				if (!Summary.bHasStrongestExecutedCollision || IsExecutedCollisionStronger(Stats, Summary.StrongestExecutedCollisionStats))
				{
					Summary.StrongestExecutedCollisionStats = Stats;
					Summary.bHasStrongestExecutedCollision = true;
				}
			}

			if (Stats.GeometricCollisionBestPlateA != INDEX_NONE &&
				(!Summary.bHasStrongestGeometricCandidate || IsGeometricSummaryStronger(Stats, Summary.StrongestGeometricCandidateStats)))
			{
				Summary.StrongestGeometricCandidateStats = Stats;
				Summary.bHasStrongestGeometricCandidate = true;
			}
			if (Stats.BoundaryContactBestPlateA != INDEX_NONE &&
				(!Summary.bHasStrongestBoundaryCandidate || IsBoundarySummaryStronger(Stats, Summary.StrongestBoundaryCandidateStats)))
			{
				Summary.StrongestBoundaryCandidateStats = Stats;
				Summary.bHasStrongestBoundaryCandidate = true;
			}
			if (Stats.GeometricCollisionRepeatedCandidateObservationCount > 1 &&
				(!Summary.bHasStrongestRepeatedGeometricCandidate ||
					IsRepeatedGeometricCandidateStronger(Stats, Summary.StrongestRepeatedGeometricCandidateStats)))
			{
				Summary.StrongestRepeatedGeometricCandidateStats = Stats;
				Summary.bHasStrongestRepeatedGeometricCandidate = true;
			}
			if (Stats.GeometricCollisionBestPlateA != INDEX_NONE &&
				Stats.BoundaryContactBestPlateA != INDEX_NONE &&
				Stats.GeometricCollisionBestPlateA == Stats.BoundaryContactBestPlateA &&
				Stats.GeometricCollisionBestPlateB == Stats.BoundaryContactBestPlateB)
			{
				++Summary.MatchingBestPairResampleCount;
				Summary.MatchingBestPairSteps.Add(Stats.Step);
			}
		}

		Summary.FinalPlateCount = Summary.FinalSnapshot.PlateCount;
		Summary.FinalContinentalAreaFraction = Summary.FinalSnapshot.ContinentalAreaFraction;
		Summary.FinalBoundarySampleFraction = Summary.FinalSnapshot.BoundarySampleFraction;
		Summary.FinalGapRate = Summary.FinalSnapshot.GapRate;
		Summary.FinalOverlapRate = Summary.FinalSnapshot.OverlapRate;
		Summary.FinalMeanElevation = Summary.FinalSnapshot.ElevationMeanKm;
		Summary.bStablePlateIdsValid = AreAllStablePlateIdsValid(Planet);
		Test.TestTrue(TEXT("M6 baseline keeps stable plate ids valid"), Summary.bStablePlateIdsValid);

		const int64 ResidualContinentalLossCount = FMath::Max<int64>(
			0,
			Summary.TotalContinentalSamplesLost - Summary.TotalFormerContinentalSamplesTurnedOceanic);
		Summary.DominantContinentalSink = TEXT("generic_interpolation_erosion_dilution");
		Summary.DominantContinentalSinkMagnitude = ResidualContinentalLossCount;
		if (Summary.TotalFormerContinentalDivergentGaps > Summary.DominantContinentalSinkMagnitude)
		{
			Summary.DominantContinentalSink = TEXT("divergent_gaps");
			Summary.DominantContinentalSinkMagnitude = Summary.TotalFormerContinentalDivergentGaps;
		}
		if (Summary.TotalFormerContinentalNonDivergentGaps > Summary.DominantContinentalSinkMagnitude)
		{
			Summary.DominantContinentalSink = TEXT("non_divergent_numerical_gaps");
			Summary.DominantContinentalSinkMagnitude = Summary.TotalFormerContinentalNonDivergentGaps;
		}

		Summary.DominantDestructionPath = TEXT("oceanization");
		Summary.DominantDestructionPathMagnitude = Summary.TotalFormerContinentalSamplesTurnedOceanic;
		const auto ConsiderDominantDestructionPath = [&Summary](const TCHAR* Path, const int64 Count)
		{
			if (Count > Summary.DominantDestructionPathMagnitude)
			{
				Summary.DominantDestructionPath = Path;
				Summary.DominantDestructionPathMagnitude = Count;
			}
		};
		ConsiderDominantDestructionPath(
			TEXT("non_gap_reclassification"),
			Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal);
		ConsiderDominantDestructionPath(
			TEXT("projection_recovery_non_continental"),
			Summary.TotalFormerContinentalProjectionRecoveredNonContinentalFinal);
		ConsiderDominantDestructionPath(
			TEXT("nearest_copy_recovery_non_continental"),
			Summary.TotalFormerContinentalNearestCopyRecoveredNonContinentalFinal);

		Summary.DominantNonGapDestructionSubpath = TEXT("changed_plate_non_continental");
		Summary.DominantNonGapDestructionSubpathMagnitude =
			Summary.TotalFormerContinentalChangedPlateNonContinentalFinal;
		const auto ConsiderDominantNonGapSubpath = [&Summary](const TCHAR* Path, const int64 Count)
		{
			if (Count > Summary.DominantNonGapDestructionSubpathMagnitude)
			{
				Summary.DominantNonGapDestructionSubpath = Path;
				Summary.DominantNonGapDestructionSubpathMagnitude = Count;
			}
		};
		ConsiderDominantNonGapSubpath(
			TEXT("full_resolution_non_continental"),
			Summary.TotalFormerContinentalFullResolutionNonContinentalFinal);
		ConsiderDominantNonGapSubpath(
			TEXT("preserve_mode_non_gap_non_continental"),
			Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal);
		ConsiderDominantNonGapSubpath(
			TEXT("fallback_query_non_continental"),
			Summary.TotalFormerContinentalFallbackQueryNonContinentalFinal);

		Summary.DominantFullResolutionLossBucket = TEXT("same_plate_non_continental");
		Summary.DominantFullResolutionLossBucketMagnitude =
			Summary.TotalFullResolutionSamePlateNonContinentalFinal;
		const auto ConsiderDominantFullResolutionLossBucket = [&Summary](const TCHAR* Path, const int64 Count)
		{
			if (Count > Summary.DominantFullResolutionLossBucketMagnitude)
			{
				Summary.DominantFullResolutionLossBucket = Path;
				Summary.DominantFullResolutionLossBucketMagnitude = Count;
			}
		};
		ConsiderDominantFullResolutionLossBucket(
			TEXT("changed_plate_non_continental"),
			Summary.TotalFullResolutionChangedPlateNonContinentalFinal);
		ConsiderDominantFullResolutionLossBucket(
			TEXT("collision_followup_same_plate_non_continental"),
			Summary.TotalFullResolutionCollisionFollowupSamePlateNonContinental);
		ConsiderDominantFullResolutionLossBucket(
			TEXT("collision_followup_changed_plate_non_continental"),
			Summary.TotalFullResolutionCollisionFollowupChangedPlateNonContinental);
		ConsiderDominantFullResolutionLossBucket(
			TEXT("rift_followup_same_plate_non_continental"),
			Summary.TotalFullResolutionRiftFollowupSamePlateNonContinental);
		ConsiderDominantFullResolutionLossBucket(
			TEXT("rift_followup_changed_plate_non_continental"),
			Summary.TotalFullResolutionRiftFollowupChangedPlateNonContinental);
		ConsiderDominantFullResolutionLossBucket(
			TEXT("other_trigger_same_plate_non_continental"),
			Summary.TotalFullResolutionOtherTriggerSamePlateNonContinental);
		ConsiderDominantFullResolutionLossBucket(
			TEXT("other_trigger_changed_plate_non_continental"),
			Summary.TotalFullResolutionOtherTriggerChangedPlateNonContinental);

		Summary.DominantGapLossMode = TEXT("none");
		Summary.DominantGapLossMagnitude = 0;
		if (Summary.TotalFormerContinentalDivergentOceanized > Summary.DominantGapLossMagnitude)
		{
			Summary.DominantGapLossMode = TEXT("divergent_oceanization");
			Summary.DominantGapLossMagnitude = Summary.TotalFormerContinentalDivergentOceanized;
		}
		if (Summary.TotalFormerContinentalNonDivergentFallbackOceanized > Summary.DominantGapLossMagnitude)
		{
			Summary.DominantGapLossMode = TEXT("non_divergent_fallback_oceanization");
			Summary.DominantGapLossMagnitude = Summary.TotalFormerContinentalNonDivergentFallbackOceanized;
		}

		Summary.DominantGeometricRejectionReason = TEXT("none");
		Summary.DominantGeometricRejectionMagnitude = 0;
		const auto ConsiderGeometricRejection = [&Summary](const TCHAR* Reason, const int64 Count)
		{
			if (Count > Summary.DominantGeometricRejectionMagnitude)
			{
				Summary.DominantGeometricRejectionReason = Reason;
				Summary.DominantGeometricRejectionMagnitude = Count;
			}
		};
		ConsiderGeometricRejection(TEXT("mass_filter"), Summary.TotalGeometricCollisionRejectedByMassFilterCount);
		ConsiderGeometricRejection(TEXT("overlap_depth"), Summary.TotalGeometricCollisionRejectedByOverlapDepthCount);
		ConsiderGeometricRejection(TEXT("persistent_penetration"), Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount);
		ConsiderGeometricRejection(TEXT("empty_terrane"), Summary.TotalGeometricCollisionRejectedByEmptyTerraneCount);
		ConsiderGeometricRejection(TEXT("role_resolution"), Summary.TotalGeometricCollisionRejectedByRoleResolutionCount);
		ConsiderGeometricRejection(TEXT("seed_direction"), Summary.TotalGeometricCollisionRejectedBySeedDirectionCount);

		if (Config.bLogSummary)
		{
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_guard run_id=%s stable_plate_ids_valid=%d ownership_changes_outside_resampling_count=%d steps_with_ownership_changes_outside_resampling=%d"),
				*Summary.RunId,
				Summary.bStablePlateIdsValid ? 1 : 0,
				Summary.OwnershipChangesOutsideResamplingCount,
				Summary.StepsWithOwnershipChangesOutsideResampling));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_summary run_id=%s seed=%d andean_continental_conversion_rate_per_my=%.6f total_resamples=%d total_collision_events=%d total_rift_events=%d first_collision_step=%d first_rift_step=%d min_continental_area_fraction=%.6f max_continental_area_fraction=%.6f final_continental_area_fraction=%.6f min_boundary_sample_fraction=%.6f max_boundary_sample_fraction=%.6f final_boundary_sample_fraction=%.6f min_gap_rate=%.6f max_gap_rate=%.6f final_gap_rate=%.6f min_overlap_rate=%.6f max_overlap_rate=%.6f final_overlap_rate=%.6f min_plate_count=%d max_plate_count=%d final_plate_count=%d max_components_per_plate_observed=%d min_mean_elevation=%.6f max_mean_elevation=%.6f final_mean_elevation=%.6f ownership_changes_outside_resampling_count=%d steps_with_ownership_changes_outside_resampling=%d"),
				*Summary.RunId,
				Summary.Seed,
				Summary.AndeanContinentalConversionRatePerMy,
				Summary.TotalResamples,
				Summary.TotalCollisionEvents,
				Summary.TotalRiftEvents,
				Summary.FirstCollisionStep,
				Summary.FirstRiftStep,
				Summary.MinContinentalAreaFraction,
				Summary.MaxContinentalAreaFraction,
				Summary.FinalContinentalAreaFraction,
				Summary.MinBoundarySampleFraction,
				Summary.MaxBoundarySampleFraction,
				Summary.FinalBoundarySampleFraction,
				Summary.MinGapRate,
				Summary.MaxGapRate,
				Summary.FinalGapRate,
				Summary.MinOverlapRate,
				Summary.MaxOverlapRate,
				Summary.FinalOverlapRate,
				Summary.MinPlateCount,
				Summary.MaxPlateCount,
				Summary.FinalPlateCount,
				Summary.MaxComponentsPerPlateObserved,
				Summary.MinMeanElevation,
				Summary.MaxMeanElevation,
				Summary.FinalMeanElevation,
				Summary.OwnershipChangesOutsideResamplingCount,
				Summary.StepsWithOwnershipChangesOutsideResampling));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_continental_budget run_id=%s seed=%d total_continental_samples_lost=%lld total_net_continental_sample_delta=%lld total_former_continental_turned_oceanic=%lld total_former_continental_divergent_gaps=%lld total_former_continental_non_divergent_gaps=%lld total_former_continental_non_divergent_gap_projection_resolved=%lld total_former_continental_non_divergent_gap_nearest_copy_resolved=%lld total_former_continental_non_divergent_fallback_oceanized=%lld total_former_continental_divergent_oceanized=%lld total_andean_continental_gains=%lld total_collision_continental_gains=%lld total_preserve_cw_retained=%lld total_preserve_cw_threshold_crossing_prevented=%lld dominant_continental_sink=%s dominant_continental_sink_magnitude=%lld"),
				*Summary.RunId,
				Summary.Seed,
				Summary.TotalContinentalSamplesLost,
				Summary.TotalNetContinentalSampleDelta,
				Summary.TotalFormerContinentalSamplesTurnedOceanic,
				Summary.TotalFormerContinentalDivergentGaps,
				Summary.TotalFormerContinentalNonDivergentGaps,
				Summary.TotalFormerContinentalNonDivergentGapProjectionResolved,
				Summary.TotalFormerContinentalNonDivergentGapNearestCopyResolved,
				Summary.TotalFormerContinentalNonDivergentFallbackOceanized,
				Summary.TotalFormerContinentalDivergentOceanized,
				Summary.TotalAndeanContinentalGains,
				Summary.TotalCollisionContinentalGains,
				Summary.TotalPreserveOwnershipCWRetained,
				Summary.TotalPreserveOwnershipCWThresholdCrossingPrevented,
				*Summary.DominantContinentalSink,
				Summary.DominantContinentalSinkMagnitude));
			Test.AddInfo(FormatM6BaselineBudgetReassessmentSummary(Summary));
			Test.AddInfo(FormatM6BaselineFullResolutionLossSplitSummary(Summary));
			Test.AddInfo(FormatM6Baseline400StepStabilitySummary(Summary));
			Test.AddInfo(FormatM6BaselineSecondHalfBudgetSplitSummary(Summary));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_gap_loss_breakdown run_id=%s seed=%d total_former_continental_divergent_oceanized=%lld total_former_continental_non_divergent_fallback_oceanized=%lld total_former_continental_non_divergent_projection_recoveries=%lld total_former_continental_non_divergent_nearest_copy_recoveries=%lld dominant_gap_loss_mode=%s dominant_gap_loss_magnitude=%lld"),
				*Summary.RunId,
				Summary.Seed,
				Summary.TotalFormerContinentalDivergentOceanized,
				Summary.TotalFormerContinentalNonDivergentFallbackOceanized,
				Summary.TotalFormerContinentalNonDivergentGapProjectionResolved,
				Summary.TotalFormerContinentalNonDivergentGapNearestCopyResolved,
				*Summary.DominantGapLossMode,
				Summary.DominantGapLossMagnitude));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_projection_recovery_loss_breakdown run_id=%s seed=%d total_projection_recovered_non_continental_final=%lld total_projection_recovered_same_plate_non_continental_final=%lld total_projection_recovered_changed_plate_non_continental_final=%lld total_projection_recovered_preserve_mode_non_continental_final=%lld total_projection_recovered_full_resolution_non_continental_final=%lld total_nearest_copy_recovered_non_continental_final=%lld"),
				*Summary.RunId,
				Summary.Seed,
				Summary.TotalFormerContinentalProjectionRecoveredNonContinentalFinal,
				Summary.TotalFormerContinentalProjectionRecoveredSamePlateNonContinentalFinal,
				Summary.TotalFormerContinentalProjectionRecoveredChangedPlateNonContinentalFinal,
				Summary.TotalFormerContinentalProjectionRecoveredPreserveModeNonContinentalFinal,
				Summary.TotalFormerContinentalProjectionRecoveredFullResolutionNonContinentalFinal,
				Summary.TotalFormerContinentalNearestCopyRecoveredNonContinentalFinal));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_collision_signal_summary run_id=%s seed=%d total_geometric_overlap_samples=%lld total_geometric_pair_count=%lld total_geometric_qualified_pairs=%lld largest_geometric_terrane_estimate_observed=%d strongest_geometric_step=%d strongest_geometric_plate_a=%d strongest_geometric_plate_b=%d strongest_geometric_overlap_sample_count=%d strongest_geometric_terrane_estimate=%d strongest_geometric_passed_mass_filter=%d strongest_boundary_step=%d strongest_boundary_plate_a=%d strongest_boundary_plate_b=%d strongest_boundary_zone_size=%d strongest_boundary_persistence_count=%d strongest_boundary_would_trigger_or_mature=%d matching_best_pair_resample_count=%d matching_best_pair_steps=%s"),
				*Summary.RunId,
				Summary.Seed,
				Summary.TotalGeometricCollisionOverlapSamples,
				Summary.TotalGeometricCollisionPairCount,
				Summary.TotalGeometricCollisionQualifiedPairCount,
				Summary.LargestGeometricTerraneEstimateObserved,
				Summary.bHasStrongestGeometricCandidate ? Summary.StrongestGeometricCandidateStats.Step : INDEX_NONE,
				Summary.bHasStrongestGeometricCandidate ? Summary.StrongestGeometricCandidateStats.GeometricCollisionBestPlateA : INDEX_NONE,
				Summary.bHasStrongestGeometricCandidate ? Summary.StrongestGeometricCandidateStats.GeometricCollisionBestPlateB : INDEX_NONE,
				Summary.bHasStrongestGeometricCandidate ? Summary.StrongestGeometricCandidateStats.GeometricCollisionBestOverlapSampleCount : 0,
				Summary.bHasStrongestGeometricCandidate ? Summary.StrongestGeometricCandidateStats.GeometricCollisionBestTerraneEstimate : 0,
				(Summary.bHasStrongestGeometricCandidate && Summary.StrongestGeometricCandidateStats.bGeometricCollisionBestPassedMassFilter) ? 1 : 0,
				Summary.bHasStrongestBoundaryCandidate ? Summary.StrongestBoundaryCandidateStats.Step : INDEX_NONE,
				Summary.bHasStrongestBoundaryCandidate ? Summary.StrongestBoundaryCandidateStats.BoundaryContactBestPlateA : INDEX_NONE,
				Summary.bHasStrongestBoundaryCandidate ? Summary.StrongestBoundaryCandidateStats.BoundaryContactBestPlateB : INDEX_NONE,
				Summary.bHasStrongestBoundaryCandidate ? Summary.StrongestBoundaryCandidateStats.BoundaryContactBestZoneSize : 0,
				Summary.bHasStrongestBoundaryCandidate ? Summary.StrongestBoundaryCandidateStats.BoundaryContactBestPersistenceCount : 0,
				(Summary.bHasStrongestBoundaryCandidate &&
					(Summary.StrongestBoundaryCandidateStats.bBoundaryContactCollisionTriggered ||
						Summary.StrongestBoundaryCandidateStats.bBoundaryContactPersistenceTriggered)) ? 1 : 0,
				Summary.MatchingBestPairResampleCount,
				*FormatStepList(Summary.MatchingBestPairSteps)));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_geometric_collision_coverage_summary run_id=%s seed=%d total_preserve_resamples_with_geometric_candidates=%d total_preserve_resamples_with_qualified_geometric_candidates=%d total_geometric_collision_candidate_count=%lld total_geometric_collision_qualified_count=%lld total_geometric_collision_qualified_but_donor_ambiguous_count=%lld total_geometric_collision_qualified_but_donor_seed_empty_count=%lld total_geometric_collision_qualified_using_directional_donor_count=%lld total_geometric_collision_qualified_using_fallback_donor_rule_count=%lld total_geometric_collision_qualified_directional_count=%lld total_geometric_collision_rejected_by_mass_filter=%lld total_geometric_collision_rejected_by_overlap_depth=%lld total_geometric_collision_rejected_by_persistent_penetration=%lld total_geometric_collision_rejected_by_empty_terrane=%lld total_geometric_collision_rejected_by_role_resolution=%lld total_geometric_collision_rejected_by_seed_direction=%lld total_geometric_collision_directional_seed_count=%lld total_geometric_collision_directional_seed_count_opposite=%lld total_geometric_collision_fallback_used=%d total_geometric_collision_executions=%d total_boundary_contact_fallback_executions=%d total_geometric_collision_executed_from_directional_polarity_count=%d dominant_geometric_rejection_reason=%s dominant_geometric_rejection_magnitude=%lld"),
				*Summary.RunId,
				Summary.Seed,
				Summary.TotalPreserveResamplesWithGeometricCandidates,
				Summary.TotalPreserveResamplesWithQualifiedGeometricCandidates,
				Summary.TotalGeometricCollisionCandidateCount,
				Summary.TotalGeometricCollisionQualifiedCount,
				Summary.TotalGeometricCollisionQualifiedButDonorAmbiguousCount,
				Summary.TotalGeometricCollisionQualifiedButDonorSeedEmptyCount,
				Summary.TotalGeometricCollisionQualifiedUsingDirectionalDonorCount,
				Summary.TotalGeometricCollisionQualifiedUsingFallbackDonorRuleCount,
				Summary.TotalGeometricCollisionQualifiedDirectionalCount,
				Summary.TotalGeometricCollisionRejectedByMassFilterCount,
				Summary.TotalGeometricCollisionRejectedByOverlapDepthCount,
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount,
				Summary.TotalGeometricCollisionRejectedByEmptyTerraneCount,
				Summary.TotalGeometricCollisionRejectedByRoleResolutionCount,
				Summary.TotalGeometricCollisionRejectedBySeedDirectionCount,
				Summary.TotalGeometricCollisionDirectionalSeedCount,
				Summary.TotalGeometricCollisionDirectionalSeedCountOpposite,
				Summary.TotalGeometricCollisionFallbackUsed,
				Summary.TotalGeometricCollisionExecutions,
				Summary.TotalBoundaryContactFallbackExecutions,
				Summary.TotalGeometricCollisionExecutedFromDirectionalPolarityCount,
				*Summary.DominantGeometricRejectionReason,
				Summary.DominantGeometricRejectionMagnitude));
			const double SeedDirectionMismatchPercent =
				Summary.TotalGeometricCollisionRejectedBySeedDirectionCount > 0
					? (100.0 * static_cast<double>(Summary.TotalGeometricCollisionSeedDirectionMismatchCount) /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedBySeedDirectionCount))
					: 0.0;
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_geometric_collision_polarity_direction_summary run_id=%s seed=%d total_seed_direction_audits=%lld total_seed_direction_mismatches=%lld total_only_overriding_bucket_populated=%lld total_only_subducting_bucket_populated=%lld total_both_directional_buckets_populated=%lld total_neither_directional_buckets_populated=%lld seed_direction_rejection_mismatch_percent=%.6f"),
				*Summary.RunId,
				Summary.Seed,
				Summary.TotalGeometricCollisionSeedDirectionAuditCount,
				Summary.TotalGeometricCollisionSeedDirectionMismatchCount,
				Summary.TotalGeometricCollisionOnlyOverridingBucketPopulatedCount,
				Summary.TotalGeometricCollisionOnlySubductingBucketPopulatedCount,
				Summary.TotalGeometricCollisionBothDirectionalBucketsPopulatedCount,
				Summary.TotalGeometricCollisionNeitherDirectionalBucketsPopulatedCount,
				SeedDirectionMismatchPercent));
			const double MeanCollisionTerraneRecovered =
				Summary.TotalCollisionEvents > 0
					? static_cast<double>(Summary.TotalCollisionTerraneRecovered) / static_cast<double>(Summary.TotalCollisionEvents)
					: 0.0;
			const double MeanCollisionCWBoosted =
				Summary.TotalCollisionEvents > 0
					? static_cast<double>(Summary.TotalCollisionCWBoosted) / static_cast<double>(Summary.TotalCollisionEvents)
					: 0.0;
			const double MeanCollisionSurgeAffected =
				Summary.TotalCollisionEvents > 0
					? static_cast<double>(Summary.TotalCollisionSurgeAffected) / static_cast<double>(Summary.TotalCollisionEvents)
					: 0.0;
			const double MeanGeometricExecutionTerraneRecovered =
				Summary.TotalGeometricCollisionExecutions > 0
					? static_cast<double>(Summary.TotalGeometricCollisionExecutionTerraneRecovered) /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanGeometricExecutionCWBoosted =
				Summary.TotalGeometricCollisionExecutions > 0
					? static_cast<double>(Summary.TotalGeometricCollisionExecutionCWBoosted) /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanBoundaryFallbackTerraneRecovered =
				Summary.TotalBoundaryContactFallbackExecutions > 0
					? static_cast<double>(Summary.TotalBoundaryContactFallbackTerraneRecovered) /
						static_cast<double>(Summary.TotalBoundaryContactFallbackExecutions)
					: 0.0;
			const double MeanBoundaryFallbackCWBoosted =
				Summary.TotalBoundaryContactFallbackExecutions > 0
					? static_cast<double>(Summary.TotalBoundaryContactFallbackCWBoosted) /
						static_cast<double>(Summary.TotalBoundaryContactFallbackExecutions)
					: 0.0;
			const double MeanGeometricExecutionSurgeAffected =
				Summary.TotalGeometricCollisionExecutions > 0
					? static_cast<double>(Summary.TotalGeometricCollisionExecutionSurgeAffected) /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanBoundaryFallbackSurgeAffected =
				Summary.TotalBoundaryContactFallbackExecutions > 0
					? static_cast<double>(Summary.TotalBoundaryContactFallbackSurgeAffected) /
						static_cast<double>(Summary.TotalBoundaryContactFallbackExecutions)
					: 0.0;
			const double MeanExecutedOverlapDepthKm =
				Summary.TotalGeometricCollisionExecutions > 0
					? Summary.TotalGeometricCollisionExecutedOverlapDepthKm /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanDepthGatedCandidateDepthKm =
				Summary.TotalGeometricCollisionRejectedByOverlapDepthCount > 0
					? Summary.TotalGeometricCollisionRejectedByOverlapDepthKm /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByOverlapDepthCount)
					: 0.0;
			const double MeanDepthGatedCandidateOverlapSampleCount =
				Summary.TotalGeometricCollisionRejectedByOverlapDepthCount > 0
					? static_cast<double>(Summary.TotalGeometricCollisionRejectedByOverlapDepthOverlapSampleCount) /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByOverlapDepthCount)
					: 0.0;
			const double MeanDepthGatedCandidateTerraneEstimate =
				Summary.TotalGeometricCollisionRejectedByOverlapDepthCount > 0
					? static_cast<double>(Summary.TotalGeometricCollisionRejectedByOverlapDepthTerraneEstimate) /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByOverlapDepthCount)
					: 0.0;
			const double MeanDepthGatedCandidateMeanConvergenceKmPerMy =
				Summary.TotalGeometricCollisionRejectedByOverlapDepthCount > 0
					? Summary.TotalGeometricCollisionRejectedByOverlapDepthMeanConvergenceKmPerMy /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByOverlapDepthCount)
					: 0.0;
			const double MeanExecutedOverlapSampleCount =
				Summary.TotalGeometricCollisionExecutions > 0
					? static_cast<double>(Summary.TotalGeometricCollisionExecutedOverlapSampleCount) /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanExecutedTerraneEstimate =
				Summary.TotalGeometricCollisionExecutions > 0
					? static_cast<double>(Summary.TotalGeometricCollisionExecutedTerraneEstimate) /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanExecutedMeanConvergenceKmPerMy =
				Summary.TotalGeometricCollisionExecutions > 0
					? Summary.TotalGeometricCollisionExecutedMeanConvergenceKmPerMy /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanExecutedAccumulatedPenetrationKm =
				Summary.TotalGeometricCollisionExecutions > 0
					? Summary.TotalGeometricCollisionExecutedAccumulatedPenetrationKm /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanExecutedObservationCount =
				Summary.TotalGeometricCollisionExecutions > 0
					? static_cast<double>(Summary.TotalGeometricCollisionExecutedObservationCount) /
						static_cast<double>(Summary.TotalGeometricCollisionExecutions)
					: 0.0;
			const double MeanPersistentPenetrationRejectedDepthKm =
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount > 0
					? Summary.TotalGeometricCollisionRejectedByPersistentPenetrationDepthKm /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount)
					: 0.0;
			const double MeanPersistentPenetrationRejectedOverlapSampleCount =
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount > 0
					? static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationOverlapSampleCount) /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount)
					: 0.0;
			const double MeanPersistentPenetrationRejectedTerraneEstimate =
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount > 0
					? static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationTerraneEstimate) /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount)
					: 0.0;
			const double MeanPersistentPenetrationRejectedMeanConvergenceKmPerMy =
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount > 0
					? Summary.TotalGeometricCollisionRejectedByPersistentPenetrationMeanConvergenceKmPerMy /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount)
					: 0.0;
			const double MeanPersistentPenetrationRejectedAccumulatedPenetrationKm =
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount > 0
					? Summary.TotalGeometricCollisionRejectedByPersistentPenetrationAccumulatedPenetrationKm /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount)
					: 0.0;
			const double MeanPersistentPenetrationRejectedObservationCount =
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount > 0
					? static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationObservationCount) /
						static_cast<double>(Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount)
					: 0.0;
			const bool bDonorClampChangedFromM6q =
				!FMath::IsNearlyEqual(Summary.GeometricCollisionDonorLocalityClampKm, 2250.0, KINDA_SMALL_NUMBER);
			const bool bInfluenceChangedFromM6q =
				!FMath::IsNearlyEqual(Summary.GeometricCollisionInfluenceRadiusScale, 1.8, KINDA_SMALL_NUMBER);
			const TCHAR* DominantCollisionProductivityPath =
				Summary.TotalGeometricCollisionExecutionCollisionGains >= Summary.TotalBoundaryContactFallbackCollisionGains
					? TEXT("geometric")
					: TEXT("boundary_fallback");
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_depth_gate_summary run_id=%s seed=%d geometric_collision_donor_locality_clamp_km=%.3f geometric_collision_influence_radius_scale=%.3f total_geometric_collision_rejected_by_overlap_depth=%lld mean_depth_gated_candidate_depth_km=%.6f max_depth_gated_candidate_depth_km=%.6f mean_depth_gated_candidate_overlap_sample_count=%.6f max_depth_gated_candidate_overlap_sample_count=%d mean_depth_gated_candidate_terrane_estimate=%.6f max_depth_gated_candidate_terrane_estimate=%d mean_depth_gated_candidate_mean_convergence_km_per_my=%.6f max_depth_gated_candidate_max_convergence_km_per_my=%.6f"),
				*Summary.RunId,
				Summary.Seed,
				Summary.GeometricCollisionDonorLocalityClampKm,
				Summary.GeometricCollisionInfluenceRadiusScale,
				Summary.TotalGeometricCollisionRejectedByOverlapDepthCount,
				MeanDepthGatedCandidateDepthKm,
				Summary.MaxGeometricCollisionRejectedByOverlapDepthKm,
				MeanDepthGatedCandidateOverlapSampleCount,
				Summary.MaxGeometricCollisionRejectedByOverlapDepthOverlapSampleCount,
				MeanDepthGatedCandidateTerraneEstimate,
				Summary.MaxGeometricCollisionRejectedByOverlapDepthTerraneEstimate,
				MeanDepthGatedCandidateMeanConvergenceKmPerMy,
				Summary.MaxGeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_depth_proxy_comparison_summary run_id=%s seed=%d executed_collision_count=%d depth_gated_candidate_count=%lld executed_mean_depth_km=%.6f executed_max_depth_km=%.6f depth_gated_mean_depth_km=%.6f depth_gated_max_depth_km=%.6f executed_mean_overlap_sample_count=%.6f executed_max_overlap_sample_count=%d depth_gated_mean_overlap_sample_count=%.6f depth_gated_max_overlap_sample_count=%d executed_mean_terrane_estimate=%.6f executed_max_terrane_estimate=%d depth_gated_mean_terrane_estimate=%.6f depth_gated_max_terrane_estimate=%d executed_mean_convergence_km_per_my=%.6f executed_max_convergence_km_per_my=%.6f depth_gated_mean_convergence_km_per_my=%.6f depth_gated_max_convergence_km_per_my=%.6f"),
				*Summary.RunId,
				Summary.Seed,
				Summary.TotalGeometricCollisionExecutions,
				Summary.TotalGeometricCollisionRejectedByOverlapDepthCount,
				MeanExecutedOverlapDepthKm,
				Summary.MaxGeometricCollisionExecutedOverlapDepthKm,
				MeanDepthGatedCandidateDepthKm,
				Summary.MaxGeometricCollisionRejectedByOverlapDepthKm,
				MeanExecutedOverlapSampleCount,
				Summary.MaxGeometricCollisionExecutedOverlapSampleCount,
				MeanDepthGatedCandidateOverlapSampleCount,
				Summary.MaxGeometricCollisionRejectedByOverlapDepthOverlapSampleCount,
				MeanExecutedTerraneEstimate,
				Summary.MaxGeometricCollisionExecutedTerraneEstimate,
				MeanDepthGatedCandidateTerraneEstimate,
				Summary.MaxGeometricCollisionRejectedByOverlapDepthTerraneEstimate,
				MeanExecutedMeanConvergenceKmPerMy,
				Summary.MaxGeometricCollisionExecutedMaxConvergenceKmPerMy,
				MeanDepthGatedCandidateMeanConvergenceKmPerMy,
				Summary.MaxGeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_persistent_penetration_summary run_id=%s seed=%d geometric_collision_persistent_penetration_threshold_km=%.6f executed_collision_count=%d penetration_gated_candidate_count=%lld executed_mean_accumulated_penetration_km=%.6f executed_max_accumulated_penetration_km=%.6f penetration_gated_mean_accumulated_penetration_km=%.6f penetration_gated_max_accumulated_penetration_km=%.6f executed_mean_observation_count=%.6f executed_max_observation_count=%d penetration_gated_mean_observation_count=%.6f penetration_gated_max_observation_count=%d executed_after_multiple_observations_count=%d"),
				*Summary.RunId,
				Summary.Seed,
				Summary.GeometricCollisionPersistentPenetrationThresholdKm,
				Summary.TotalGeometricCollisionExecutions,
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount,
				MeanExecutedAccumulatedPenetrationKm,
				Summary.MaxGeometricCollisionExecutedAccumulatedPenetrationKm,
				MeanPersistentPenetrationRejectedAccumulatedPenetrationKm,
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationAccumulatedPenetrationKm,
				MeanExecutedObservationCount,
				Summary.MaxGeometricCollisionExecutedObservationCount,
				MeanPersistentPenetrationRejectedObservationCount,
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationObservationCount,
				Summary.TotalGeometricCollisionExecutedAfterMultipleObservationsCount));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_penetration_gate_spatial_comparison_summary run_id=%s seed=%d executed_collision_count=%d penetration_gated_candidate_count=%lld executed_mean_depth_km=%.6f executed_max_depth_km=%.6f penetration_gated_mean_depth_km=%.6f penetration_gated_max_depth_km=%.6f executed_mean_overlap_sample_count=%.6f executed_max_overlap_sample_count=%d penetration_gated_mean_overlap_sample_count=%.6f penetration_gated_max_overlap_sample_count=%d executed_mean_terrane_estimate=%.6f executed_max_terrane_estimate=%d penetration_gated_mean_terrane_estimate=%.6f penetration_gated_max_terrane_estimate=%d executed_mean_convergence_km_per_my=%.6f executed_max_convergence_km_per_my=%.6f penetration_gated_mean_convergence_km_per_my=%.6f penetration_gated_max_convergence_km_per_my=%.6f"),
				*Summary.RunId,
				Summary.Seed,
				Summary.TotalGeometricCollisionExecutions,
				Summary.TotalGeometricCollisionRejectedByPersistentPenetrationCount,
				MeanExecutedOverlapDepthKm,
				Summary.MaxGeometricCollisionExecutedOverlapDepthKm,
				MeanPersistentPenetrationRejectedDepthKm,
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationDepthKm,
				MeanExecutedOverlapSampleCount,
				Summary.MaxGeometricCollisionExecutedOverlapSampleCount,
				MeanPersistentPenetrationRejectedOverlapSampleCount,
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationOverlapSampleCount,
				MeanExecutedTerraneEstimate,
				Summary.MaxGeometricCollisionExecutedTerraneEstimate,
				MeanPersistentPenetrationRejectedTerraneEstimate,
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationTerraneEstimate,
				MeanExecutedMeanConvergenceKmPerMy,
				Summary.MaxGeometricCollisionExecutedMaxConvergenceKmPerMy,
				MeanPersistentPenetrationRejectedMeanConvergenceKmPerMy,
				Summary.MaxGeometricCollisionRejectedByPersistentPenetrationMaxConvergenceKmPerMy));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_temporal_penetration_summary run_id=%s seed=%d repeated_pair_plate_a=%d repeated_pair_plate_b=%d repeated_pair_observation_count=%d repeated_pair_overlap_sample_count=%d repeated_pair_terrane_estimate=%d repeated_pair_mean_depth_km=%.6f repeated_pair_max_depth_km=%.6f repeated_pair_effective_convergence_km_per_my=%.6f repeated_pair_max_convergence_km_per_my=%.6f repeated_pair_accumulated_penetration_km=%.6f"),
				*Summary.RunId,
				Summary.Seed,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidatePlateA : INDEX_NONE,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidatePlateB : INDEX_NONE,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidateObservationCount : 0,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidateOverlapSampleCount : 0,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidateTerraneEstimate : 0,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidateMeanOverlapDepthKm : 0.0,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidateMaxOverlapDepthKm : 0.0,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidateEffectiveConvergenceKmPerMy : 0.0,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidateMaxConvergenceKmPerMy : 0.0,
				Summary.bHasStrongestRepeatedGeometricCandidate ? Summary.StrongestRepeatedGeometricCandidateStats.GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm : 0.0));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_collision_execution_summary run_id=%s seed=%d geometric_collision_donor_locality_clamp_km=%.3f geometric_collision_influence_radius_scale=%.3f geometric_collision_persistent_penetration_threshold_km=%.3f total_collision_events=%d total_geometric_collision_executions=%d total_boundary_contact_fallback_executions=%d total_collision_repeated_pair_count=%d total_collision_repeated_pair_within_cooldown_count=%d mean_collision_terrane_recovered=%.6f max_collision_terrane_recovered=%d mean_collision_cw_boosted=%.6f max_collision_cw_boosted=%d mean_collision_surge_affected=%.6f max_collision_surge_affected=%d mean_geometric_collision_overlap_depth_km=%.6f max_geometric_collision_overlap_depth_km=%.6f mean_executed_accumulated_penetration_km=%.6f max_executed_accumulated_penetration_km=%.6f total_geometric_collision_donor_terrane_locality_limited_count=%lld strongest_executed_step=%d strongest_executed_over_plate=%d strongest_executed_sub_plate=%d strongest_executed_terrane_recovered=%d strongest_executed_cw_boosted=%d strongest_executed_surge_affected=%d strongest_executed_used_geometric=%d"),
				*Summary.RunId,
				Summary.Seed,
				Summary.GeometricCollisionDonorLocalityClampKm,
				Summary.GeometricCollisionInfluenceRadiusScale,
				Summary.GeometricCollisionPersistentPenetrationThresholdKm,
				Summary.TotalCollisionEvents,
				Summary.TotalGeometricCollisionExecutions,
				Summary.TotalBoundaryContactFallbackExecutions,
				Summary.TotalCollisionRepeatedPairCount,
				Summary.TotalCollisionRepeatedPairWithinCooldownCount,
				MeanCollisionTerraneRecovered,
				Summary.MaxCollisionTerraneRecovered,
				MeanCollisionCWBoosted,
				Summary.MaxCollisionCWBoosted,
				MeanCollisionSurgeAffected,
				Summary.MaxCollisionSurgeAffected,
				MeanExecutedOverlapDepthKm,
				Summary.MaxGeometricCollisionExecutedOverlapDepthKm,
				MeanExecutedAccumulatedPenetrationKm,
				Summary.MaxGeometricCollisionExecutedAccumulatedPenetrationKm,
				Summary.TotalGeometricCollisionExecutedDonorTerraneLocalityLimitedCount,
				Summary.bHasStrongestExecutedCollision ? Summary.StrongestExecutedCollisionStats.Step : INDEX_NONE,
				Summary.bHasStrongestExecutedCollision ? Summary.StrongestExecutedCollisionStats.CollisionOverridingPlateId : INDEX_NONE,
				Summary.bHasStrongestExecutedCollision ? Summary.StrongestExecutedCollisionStats.CollisionSubductingPlateId : INDEX_NONE,
				Summary.bHasStrongestExecutedCollision ? Summary.StrongestExecutedCollisionStats.CollisionTerraneSampleCount : 0,
				Summary.bHasStrongestExecutedCollision ? Summary.StrongestExecutedCollisionStats.CollisionCWBoostedCount : 0,
				Summary.bHasStrongestExecutedCollision ? Summary.StrongestExecutedCollisionStats.CollisionSurgeAffectedCount : 0,
				(Summary.bHasStrongestExecutedCollision && Summary.StrongestExecutedCollisionStats.bUsedGeometricCollisionExecution) ? 1 : 0));
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_collision_productivity_split_summary run_id=%s seed=%d geometric_collision_donor_locality_clamp_km=%.3f geometric_collision_influence_radius_scale=%.3f donor_clamp_changed_from_m6q=%d influence_radius_scale_changed_from_m6q=%d geometric_execution_count=%d boundary_fallback_execution_count=%d geometric_collision_gain_total=%lld boundary_fallback_collision_gain_total=%lld geometric_mean_terrane_recovered=%.6f geometric_max_terrane_recovered=%d boundary_fallback_mean_terrane_recovered=%.6f boundary_fallback_max_terrane_recovered=%d geometric_mean_cw_boosted=%.6f geometric_max_cw_boosted=%d boundary_fallback_mean_cw_boosted=%.6f boundary_fallback_max_cw_boosted=%d geometric_mean_surge_affected=%.6f geometric_max_surge_affected=%d boundary_fallback_mean_surge_affected=%.6f boundary_fallback_max_surge_affected=%d dominant_collision_productivity_path=%s"),
				*Summary.RunId,
				Summary.Seed,
				Summary.GeometricCollisionDonorLocalityClampKm,
				Summary.GeometricCollisionInfluenceRadiusScale,
				bDonorClampChangedFromM6q ? 1 : 0,
				bInfluenceChangedFromM6q ? 1 : 0,
				Summary.TotalGeometricCollisionExecutions,
				Summary.TotalBoundaryContactFallbackExecutions,
				Summary.TotalGeometricCollisionExecutionCollisionGains,
				Summary.TotalBoundaryContactFallbackCollisionGains,
				MeanGeometricExecutionTerraneRecovered,
				Summary.MaxGeometricCollisionExecutionTerraneRecovered,
				MeanBoundaryFallbackTerraneRecovered,
				Summary.MaxBoundaryContactFallbackTerraneRecovered,
				MeanGeometricExecutionCWBoosted,
				Summary.MaxGeometricCollisionExecutionCWBoosted,
				MeanBoundaryFallbackCWBoosted,
				Summary.MaxBoundaryContactFallbackCWBoosted,
				MeanGeometricExecutionSurgeAffected,
				Summary.MaxGeometricCollisionExecutionSurgeAffected,
				MeanBoundaryFallbackSurgeAffected,
				Summary.MaxBoundaryContactFallbackSurgeAffected,
				DominantCollisionProductivityPath));
			const bool bRecommendDonorResolutionRefinement =
				Summary.TotalGeometricCollisionQualifiedButDonorAmbiguousCount > 0 ||
				Summary.TotalGeometricCollisionQualifiedButDonorSeedEmptyCount > 0 ||
				Summary.TotalBoundaryContactFallbackExecutions >= Summary.TotalGeometricCollisionExecutions;
			const bool bRecommendCollisionLocalityTuning =
				!bRecommendDonorResolutionRefinement &&
				Summary.TotalGeometricCollisionExecutions > Summary.TotalBoundaryContactFallbackExecutions &&
				(Summary.MaxComponentsPerPlateObserved > 100 ||
				 MeanCollisionTerraneRecovered > 1000.0 ||
				 Summary.TotalCollisionRepeatedPairWithinCooldownCount > 0);
			const bool bRecommendCollisionFormulaTuning =
				!bRecommendDonorResolutionRefinement &&
				!bRecommendCollisionLocalityTuning &&
				Summary.TotalGeometricCollisionExecutions > Summary.TotalBoundaryContactFallbackExecutions;
			Test.AddInfo(FString::Printf(
				TEXT("m6_baseline_recommendation run_id=%s seed=%d recommend=%s"),
				*Summary.RunId,
				Summary.Seed,
				bRecommendDonorResolutionRefinement
					? TEXT("donor_resolution_refinement")
					: (bRecommendCollisionLocalityTuning
						? TEXT("collision_locality_tuning")
						: (bRecommendCollisionFormulaTuning
							? TEXT("collision_formula_tuning")
							: TEXT("broader_balance_reassessment")))));
				Test.AddInfo(FormatM6BaselineTopologyDriftAttributionSummary(Summary));
				Test.AddInfo(FormatM6BaselineRiftTopologyAttributionSummary(Summary));
				Test.AddInfo(FormatM6BaselineWorstRiftSpikeSummary(Summary));
				Test.AddInfo(FormatM6BaselineRiftLocalizationSummary(Summary));
				Test.AddInfo(FormatM6BaselinePreserveFallbackSummary(Summary));
				Test.AddInfo(FormatM6BaselinePreserveBoundaryPolicySummary(Summary));
				Test.AddInfo(FormatM6BaselineStep229RiftPulseSummary(Summary));
				Test.AddInfo(FormatM6BaselineCollisionFragmentationSplitSummary(Summary));
				Test.AddInfo(FormatM6BaselineCollisionDonorProtectionTrimSummary(Summary));
			Test.AddInfo(FormatM6BaselineCollisionReceiverContiguitySummary(Summary));
			Test.AddInfo(FormatM6BaselineTopologySpikeSummary(Summary));
		}

		return Summary;
	}

	constexpr double AuditTriangleEpsilon = 1.0e-12;
	constexpr double AuditMaxDistanceAdvanceKmPerStep = 200.0;
	constexpr int32 AuditMinRiftChildSamples = 256;
	constexpr int32 M6OwnershipAuditStep30 = 30;
	constexpr int32 M6OwnershipAuditStep229 = 229;
	constexpr int32 M6OwnershipAuditStep229ParentPlateId = 15;
	constexpr int32 M6OwnershipAuditStep229ChildCount = 2;
	constexpr int32 M6OwnershipAuditStep229AutoTriggerSeed = 1624092517;
	constexpr int32 M6OwnershipAuditStep229ParentContinentalSamples = 3710;
	constexpr double M6OwnershipAuditStep229ParentContinentalFraction = 0.2186;
	constexpr double M6OwnershipAuditStep229TriggerProbability = 0.056536;

	FTectonicPlanet CreateM6BaselinePlanetForAudit()
	{
		FTectonicPlanet Planet = CreateInitializedPlanet(TestPlateCount, TestSampleCount, TestRandomSeed);
		ApplyTectonicPlanetRuntimeConfig(Planet, GetM6BaselineRuntimeConfig());
		return Planet;
	}

	FTectonicPlanet AdvanceOneStepWithoutResamplingForAudit(const FTectonicPlanet& SourcePlanet)
	{
		FTectonicPlanet Planet = SourcePlanet;
		const int32 ResampleCountBefore = Planet.ResamplingSteps.Num();
		Planet.MaxResampleCount = ResampleCountBefore;
		Planet.AdvanceStep();
		Planet.MaxResampleCount = INDEX_NONE;
		return Planet;
	}

	FTectonicPlanet AdvanceOneStepAllowSingleResampleForAudit(const FTectonicPlanet& SourcePlanet)
	{
		FTectonicPlanet Planet = SourcePlanet;
		const int32 ResampleCountBefore = Planet.ResamplingSteps.Num();
		Planet.MaxResampleCount = ResampleCountBefore + 1;
		Planet.AdvanceStep();
		Planet.MaxResampleCount = INDEX_NONE;
		return Planet;
	}

	uint32 MixBitsForAudit(uint32 Value)
	{
		Value ^= Value >> 16;
		Value *= 0x7feb352dU;
		Value ^= Value >> 15;
		Value *= 0x846ca68bU;
		Value ^= Value >> 16;
		return Value;
	}

	int32 MakeDeterministicSeedForAudit(const int32 A, const int32 B, const int32 C = 0)
	{
		uint32 Seed = MixBitsForAudit(static_cast<uint32>(A));
		Seed ^= MixBitsForAudit(static_cast<uint32>(B) + 0x9e3779b9U);
		Seed ^= MixBitsForAudit(static_cast<uint32>(C) + 0x85ebca6bU);
		return static_cast<int32>(Seed & 0x7fffffffU);
	}

	FVector3d MakeNoiseOffsetForAudit(const int32 SeedA, const int32 SeedB, const int32 Salt)
	{
		FRandomStream Random(MakeDeterministicSeedForAudit(SeedA, SeedB, Salt));
		return FVector3d(
			Random.FRandRange(-10.0f, 10.0f),
			Random.FRandRange(-10.0f, 10.0f),
			Random.FRandRange(-10.0f, 10.0f));
	}

	double ComputeNoiseValueForAudit(const FVector3d& Position, const FVector3d& Offset, const double Frequency)
	{
		return static_cast<double>(FMath::PerlinNoise3D(FVector((Position * Frequency) + Offset)));
	}

	double ComputeWarpedDistanceForAudit(
		const FVector3d& Position,
		const FVector3d& Centroid,
		const int32 GlobalSeed,
		const int32 PlateId,
		const int32 Salt,
		const double Amplitude,
		const double Frequency)
	{
		const double BaseDistance = ComputeGeodesicDistanceForTest(Position, Centroid);
		if (Amplitude <= 0.0)
		{
			return BaseDistance;
		}

		const FVector3d Offset = MakeNoiseOffsetForAudit(GlobalSeed, PlateId, Salt);
		const double Noise = ComputeNoiseValueForAudit(Position, Offset, Frequency);
		const double WarpMultiplier = FMath::Max(0.05, 1.0 + (Amplitude * Noise));
		return BaseDistance * WarpMultiplier;
	}

	bool IsFiniteVectorForAudit(const FVector3d& Vector)
	{
		return
			FMath::IsFinite(Vector.X) &&
			FMath::IsFinite(Vector.Y) &&
			FMath::IsFinite(Vector.Z);
	}

	bool ChooseForcedRiftCentroidSamplesForAudit(
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

		const int32 FirstMemberOffset = FMath::Abs(Seed) % ParentMembers.Num();
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
						ComputeGeodesicDistanceForTest(
							Planet.Samples[CandidateSampleIndex].Position.GetSafeNormal(),
							Planet.Samples[ExistingCentroidSampleIndex].Position.GetSafeNormal()));
				}

				if (MinDistanceToExistingCentroids > BestMinDistance + AuditTriangleEpsilon ||
					(FMath::IsNearlyEqual(MinDistanceToExistingCentroids, BestMinDistance, AuditTriangleEpsilon) &&
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

	bool PartitionParentMembersByCentroidsForAudit(
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
					? ComputeWarpedDistanceForAudit(
						SamplePosition,
						CentroidPositions[ChildIndex],
						EventSeed,
						CentroidSampleIndices[ChildIndex],
						303,
						WarpAmplitude,
						WarpFrequency)
					: ComputeGeodesicDistanceForTest(SamplePosition, CentroidPositions[ChildIndex]);
				if (DistanceToCentroid < BestDistance - AuditTriangleEpsilon ||
					(FMath::IsNearlyEqual(DistanceToCentroid, BestDistance, AuditTriangleEpsilon) &&
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
			if (ChildMembers.Num() < AuditMinRiftChildSamples)
			{
				return false;
			}
		}

		return true;
	}

	bool TryComputeBinaryForcedRiftChildAxesForAudit(
		const FVector3d& ParentCentroid,
		const FVector3d& ParentRotationAxis,
		const TArray<FVector3d>& ChildCentroids,
		TArray<FVector3d>& OutChildAxes)
	{
		OutChildAxes.Reset();
		if (ChildCentroids.Num() != 2)
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
			!IsFiniteVectorForAudit(DivergentAxisA) ||
			!IsFiniteVectorForAudit(DivergentAxisB))
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
				if (!BlendedAxis.IsNearlyZero() && IsFiniteVectorForAudit(BlendedAxis))
				{
					ChildAxis = BlendedAxis;
				}
			}
		}

		return true;
	}

	bool TriggerBinaryRiftWithoutFollowupForAudit(
		FTectonicPlanet& Planet,
		const int32 ParentPlateId,
		const int32 Seed,
		const bool bAutomatic,
		const double TriggerProbability,
		const int32 ParentContinentalSampleCount,
		const double ParentContinentalFraction)
	{
		const int32 ParentPlateIndex = Planet.FindPlateArrayIndexById(ParentPlateId);
		if (!Planet.Plates.IsValidIndex(ParentPlateIndex))
		{
			return false;
		}

		const FPlate ParentPlate = Planet.Plates[ParentPlateIndex];
		if (ParentPlate.MemberSamples.Num() < (M6OwnershipAuditStep229ChildCount * AuditMinRiftChildSamples))
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
				Planet.Samples.IsValidIndex(SampleIndex)
					? Planet.Samples[SampleIndex].TerraneId
					: INDEX_NONE);
		}

		const int32 RiftEventSeed = MakeDeterministicSeedForAudit(Planet.SimulationSeed, ParentPlateId, Seed);
		TArray<int32> CentroidSampleIndices;
		if (!ChooseForcedRiftCentroidSamplesForAudit(
				Planet,
				ParentMembers,
				M6OwnershipAuditStep229ChildCount,
				Seed,
				CentroidSampleIndices))
		{
			return false;
		}

		TArray<TArray<int32>> ChildMemberSamples;
		if (!PartitionParentMembersByCentroidsForAudit(
				Planet,
				ParentMembers,
				CentroidSampleIndices,
				RiftEventSeed,
				Planet.bEnableWarpedRiftBoundaries,
				Planet.RiftBoundaryWarpAmplitude,
				Planet.RiftBoundaryWarpFrequency,
				ChildMemberSamples))
		{
			return false;
		}

		TArray<FVector3d> ChildCentroids;
		ChildCentroids.Reserve(ChildMemberSamples.Num());
		for (const TArray<int32>& ChildMembers : ChildMemberSamples)
		{
			const FVector3d ChildCentroid = ComputeSampleSetCentroidForTest(Planet, ChildMembers);
			if (ChildCentroid.IsNearlyZero())
			{
				return false;
			}

			ChildCentroids.Add(ChildCentroid);
		}

		const FVector3d ParentCentroid = ComputeSampleSetCentroidForTest(Planet, ParentMembers);
		TArray<FVector3d> ChildAxes;
		if (!TryComputeBinaryForcedRiftChildAxesForAudit(
				ParentCentroid,
				ParentPlate.RotationAxis,
				ChildCentroids,
				ChildAxes))
		{
			return false;
		}

		TArray<int32> ChildPlateIds;
		ChildPlateIds.Reserve(M6OwnershipAuditStep229ChildCount);
		TArray<int32> ChildSampleCounts;
		ChildSampleCounts.Reserve(M6OwnershipAuditStep229ChildCount);
		TArray<FPlate> NewChildPlates;
		NewChildPlates.Reserve(M6OwnershipAuditStep229ChildCount);
		const double MaxRiftAngularSpeed =
			AuditMaxDistanceAdvanceKmPerStep / FMath::Max(Planet.PlanetRadiusKm, UE_DOUBLE_SMALL_NUMBER);
		const double ChildAngularSpeedScale = 1.35;
		for (int32 ChildIndex = 0; ChildIndex < M6OwnershipAuditStep229ChildCount; ++ChildIndex)
		{
			FPlate ChildPlate;
			ChildPlate.Id = Planet.NextPlateId++;
			ChildPlate.RotationAxis = ChildAxes[ChildIndex];
			ChildPlate.AngularSpeed = FMath::Min(ParentPlate.AngularSpeed * ChildAngularSpeedScale, MaxRiftAngularSpeed);
			ChildPlate.LastRiftStep = Planet.CurrentStep;
			ChildPlate.CumulativeRotation = ParentPlate.CumulativeRotation;
			ChildPlate.MemberSamples = ChildMemberSamples[ChildIndex];
			ChildPlateIds.Add(ChildPlate.Id);
			ChildSampleCounts.Add(ChildPlate.MemberSamples.Num());

			for (const int32 SampleIndex : ChildPlate.MemberSamples)
			{
				if (Planet.Samples.IsValidIndex(SampleIndex))
				{
					Planet.Samples[SampleIndex].PlateId = ChildPlate.Id;
				}
			}

			NewChildPlates.Add(MoveTemp(ChildPlate));
		}

		Planet.Plates.RemoveAt(ParentPlateIndex);
		for (FPlate& ChildPlate : NewChildPlates)
		{
			Planet.Plates.Add(MoveTemp(ChildPlate));
		}
		Planet.ResetBoundaryContactCollisionPersistence();

		TArray<int32> PostRiftPlateIds;
		PostRiftPlateIds.Reserve(Planet.Samples.Num());
		for (const FSample& Sample : Planet.Samples)
		{
			PostRiftPlateIds.Add(Sample.PlateId);
		}

		Planet.RepartitionMembership(PostRiftPlateIds);
		Planet.PendingRiftEvent = FPendingRiftEvent{};
		Planet.PendingRiftEvent.bValid = true;
		Planet.PendingRiftEvent.bAutomatic = bAutomatic;
		Planet.PendingRiftEvent.ParentPlateId = ParentPlateId;
		Planet.PendingRiftEvent.ChildCount = M6OwnershipAuditStep229ChildCount;
		Planet.PendingRiftEvent.ChildPlateA = ChildPlateIds.IsValidIndex(0) ? ChildPlateIds[0] : INDEX_NONE;
		Planet.PendingRiftEvent.ChildPlateB = ChildPlateIds.IsValidIndex(1) ? ChildPlateIds[1] : INDEX_NONE;
		Planet.PendingRiftEvent.ParentSampleCount = ParentMembers.Num();
		Planet.PendingRiftEvent.ParentContinentalSampleCount = ParentContinentalSampleCount;
		Planet.PendingRiftEvent.ChildSampleCountA = ChildSampleCounts.IsValidIndex(0) ? ChildSampleCounts[0] : 0;
		Planet.PendingRiftEvent.ChildSampleCountB = ChildSampleCounts.IsValidIndex(1) ? ChildSampleCounts[1] : 0;
		Planet.PendingRiftEvent.EventSeed = RiftEventSeed;
		Planet.PendingRiftEvent.ChildPlateIds = ChildPlateIds;
		Planet.PendingRiftEvent.ChildAnchorSampleIndices = CentroidSampleIndices;
		Planet.PendingRiftEvent.ChildSampleCounts = ChildSampleCounts;
		Planet.PendingRiftEvent.FormerParentSampleIndices = ParentMembers;
		Planet.PendingRiftEvent.FormerParentTerraneIds = MoveTemp(FormerParentTerraneIds);
		Planet.PendingRiftEvent.ParentContinentalFraction = ParentContinentalFraction;
		Planet.PendingRiftEvent.TriggerProbability = TriggerProbability;
		return true;
	}

	struct FManualResampleAuditExecutionResult
	{
		FTectonicPlanet FinalPlanet;
		TArray<int32> PreviousPlateAssignments;
		TArray<float> PreviousContinentalWeights;
		TArray<EOrogenyType> PreviousOrogenyTypes;
		TArray<int32> NewPlateIdsAfterQuery;
		TArray<int32> NewPlateIdsAfterGapResolution;
		TArray<int32> NewPlateIdsBeforeRepartition;
		TArray<uint8> GapFlags;
		TArray<uint8> OverlapFlags;
		TArray<uint8> LocalizedRestoreFlags;
		TArray<uint8> PreserveOwnershipCWRetainFlags;
		TArray<uint8> PreserveOwnershipFallbackQueryFlags;
		TArray<EGapResolutionPath> GapResolutionPaths;
		TArray<float> ContinentalWeightsAfterInterpolation;
		TArray<float> ContinentalWeightsAfterGapResolution;
		TArray<float> ContinentalWeightsAfterFinalOwnerReconciliation;
	};

	TArray<float> CaptureContinentalWeightsForAudit(const FTectonicPlanet& Planet)
	{
		TArray<float> Weights;
		Weights.Reserve(Planet.Samples.Num());
		for (const FSample& Sample : Planet.Samples)
		{
			Weights.Add(Sample.ContinentalWeight);
		}
		return Weights;
	}

	FManualResampleAuditExecutionResult ReplayResampleForAudit(
		const FTectonicPlanet& SourcePlanet,
		const EResampleOwnershipMode OwnershipMode,
		const EResampleTriggerReason TriggerReason)
	{
		FManualResampleAuditExecutionResult Result;
		Result.FinalPlanet = SourcePlanet;
		FTectonicPlanet& Planet = Result.FinalPlanet;
		Planet.ComputePlateScores();

		Result.PreviousPlateAssignments.Reserve(Planet.Samples.Num());
		Result.PreviousContinentalWeights.Reserve(Planet.Samples.Num());
		Result.PreviousOrogenyTypes.Reserve(Planet.Samples.Num());
		for (const FSample& Sample : Planet.Samples)
		{
			Result.PreviousPlateAssignments.Add(Sample.PlateId);
			Result.PreviousContinentalWeights.Add(Sample.ContinentalWeight);
			Result.PreviousOrogenyTypes.Add(Sample.OrogenyType);
		}

		Planet.BuildContainmentSoups();
		TArray<int32> NewPlateIds;
		TArray<int32> ContainingTriangles;
		TArray<FVector3d> BarycentricCoords;
		TArray<float> InterpolatedSubductionDistances;
		TArray<float> InterpolatedSubductionSpeeds;
		int32 GapCount = 0;
		int32 OverlapCount = 0;
		Planet.QueryOwnership(
			NewPlateIds,
			ContainingTriangles,
			BarycentricCoords,
			Result.GapFlags,
			Result.OverlapFlags,
			GapCount,
			OverlapCount,
			nullptr,
			nullptr,
			OwnershipMode,
			&Result.PreserveOwnershipCWRetainFlags,
			&Result.PreserveOwnershipFallbackQueryFlags);
		Result.NewPlateIdsAfterQuery = NewPlateIds;

		Planet.InterpolateFromCarried(
			NewPlateIds,
			ContainingTriangles,
			BarycentricCoords,
			InterpolatedSubductionDistances,
			InterpolatedSubductionSpeeds);
		Result.ContinentalWeightsAfterInterpolation = CaptureContinentalWeightsForAudit(Planet);

		Planet.ApplyPreserveOwnershipContinentalWeightRetention(
			Result.PreserveOwnershipCWRetainFlags,
			Result.PreviousContinentalWeights,
			OwnershipMode,
			TriggerReason,
			nullptr);

		Planet.ResolveGaps(
			NewPlateIds,
			Result.GapFlags,
			InterpolatedSubductionDistances,
			InterpolatedSubductionSpeeds,
			nullptr,
			&Result.GapResolutionPaths);
		Result.NewPlateIdsAfterGapResolution = NewPlateIds;
		Result.ContinentalWeightsAfterGapResolution = CaptureContinentalWeightsForAudit(Planet);
		Planet.ApplyPreserveOwnershipFallbackSamePlateRetention(
			NewPlateIds,
			Result.PreviousPlateAssignments,
			Result.PreviousContinentalWeights,
			Result.PreserveOwnershipFallbackQueryFlags,
			Result.GapResolutionPaths,
			OwnershipMode,
			TriggerReason,
			nullptr);

		if (TriggerReason == EResampleTriggerReason::RiftFollowup && Planet.PendingRiftEvent.bValid)
		{
			Planet.ApplyRiftFollowupLocalizationOverride(
				NewPlateIds,
				Result.PreviousPlateAssignments,
				Result.PreviousContinentalWeights,
				&Result.GapFlags,
				&Result.OverlapFlags,
				nullptr,
				&InterpolatedSubductionDistances,
				&InterpolatedSubductionSpeeds,
				&Result.GapResolutionPaths,
				&Result.LocalizedRestoreFlags,
				nullptr);
			Planet.ApplyRiftChildCoherenceProtection(
				NewPlateIds,
				Result.PreviousPlateAssignments,
				&Result.GapFlags,
				&Result.OverlapFlags,
				nullptr,
					&Result.GapResolutionPaths,
					nullptr);
				Planet.ApplyRiftFollowupFinalOwnerContinentalWeightReconciliation(
					Result.NewPlateIdsAfterQuery,
					NewPlateIds,
					Result.PreviousPlateAssignments,
					Result.PreviousContinentalWeights,
					Result.LocalizedRestoreFlags,
					nullptr);
			}
			Result.ContinentalWeightsAfterFinalOwnerReconciliation = CaptureContinentalWeightsForAudit(Planet);
			Result.NewPlateIdsBeforeRepartition = NewPlateIds;

		Planet.ApplyFullResolutionSamePlateContinentalWeightRetention(
			NewPlateIds,
			Result.PreviousPlateAssignments,
			Result.PreviousContinentalWeights,
			Result.GapResolutionPaths,
			OwnershipMode,
			TriggerReason,
			nullptr);
		Planet.RepartitionMembership(NewPlateIds, &InterpolatedSubductionDistances, &InterpolatedSubductionSpeeds);
		return Result;
	}

	bool CompareSamplePlateAndCWStateForAudit(
		FAutomationTestBase& Test,
		const TCHAR* Label,
		const FTectonicPlanet& Expected,
		const FTectonicPlanet& Actual)
	{
		Test.TestEqual(*FString::Printf(TEXT("%s sample count matches"), Label), Actual.Samples.Num(), Expected.Samples.Num());
		if (Actual.Samples.Num() != Expected.Samples.Num())
		{
			return false;
		}

		int32 PlateMismatchCount = 0;
		int32 CWMismatchCount = 0;
		int32 FirstPlateMismatchSample = INDEX_NONE;
		int32 FirstCWMismatchSample = INDEX_NONE;
		for (int32 SampleIndex = 0; SampleIndex < Expected.Samples.Num(); ++SampleIndex)
		{
			if (Expected.Samples[SampleIndex].PlateId != Actual.Samples[SampleIndex].PlateId)
			{
				++PlateMismatchCount;
				if (FirstPlateMismatchSample == INDEX_NONE)
				{
					FirstPlateMismatchSample = SampleIndex;
				}
			}
			if (!FMath::IsNearlyEqual(
					Expected.Samples[SampleIndex].ContinentalWeight,
					Actual.Samples[SampleIndex].ContinentalWeight,
					1.0e-6f))
			{
				++CWMismatchCount;
				if (FirstCWMismatchSample == INDEX_NONE)
				{
					FirstCWMismatchSample = SampleIndex;
				}
			}
		}

		if (PlateMismatchCount > 0)
		{
			Test.AddInfo(FString::Printf(
				TEXT("m6_ownership_audit_parity plate_label=%s mismatch_count=%d first_sample=%d expected=%d actual=%d"),
				Label,
				PlateMismatchCount,
				FirstPlateMismatchSample,
				Expected.Samples.IsValidIndex(FirstPlateMismatchSample) ? Expected.Samples[FirstPlateMismatchSample].PlateId : INDEX_NONE,
				Actual.Samples.IsValidIndex(FirstPlateMismatchSample) ? Actual.Samples[FirstPlateMismatchSample].PlateId : INDEX_NONE));
		}
		if (CWMismatchCount > 0)
		{
			Test.AddInfo(FString::Printf(
				TEXT("m6_ownership_audit_parity cw_label=%s mismatch_count=%d first_sample=%d expected=%.6f actual=%.6f"),
				Label,
				CWMismatchCount,
				FirstCWMismatchSample,
				Expected.Samples.IsValidIndex(FirstCWMismatchSample) ? Expected.Samples[FirstCWMismatchSample].ContinentalWeight : -1.0f,
				Actual.Samples.IsValidIndex(FirstCWMismatchSample) ? Actual.Samples[FirstCWMismatchSample].ContinentalWeight : -1.0f));
		}

		Test.TestEqual(*FString::Printf(TEXT("%s plate ids match"), Label), PlateMismatchCount, 0);
		Test.TestEqual(*FString::Printf(TEXT("%s continental weights match"), Label), CWMismatchCount, 0);
		return PlateMismatchCount == 0 && CWMismatchCount == 0;
	}

	void AddOwnerPairCountForAudit(
		TMap<FString, int32>& CountsByPair,
		const int32 OldPlateId,
		const int32 NewPlateId)
	{
		const FString Key = FString::Printf(TEXT("%d->%d"), OldPlateId, NewPlateId);
		++CountsByPair.FindOrAdd(Key);
	}

	FString FormatSortedCountMapForAudit(const TMap<FString, int32>& CountsByKey)
	{
		TArray<FString> Keys;
		CountsByKey.GenerateKeyArray(Keys);
		Keys.Sort([&CountsByKey](const FString& A, const FString& B)
		{
			const int32 CountA = CountsByKey.FindRef(A);
			const int32 CountB = CountsByKey.FindRef(B);
			return
				CountA > CountB ||
				(CountA == CountB && A.Compare(B) < 0);
		});

		TArray<FString> Parts;
		Parts.Reserve(Keys.Num());
		for (const FString& Key : Keys)
		{
			Parts.Add(FString::Printf(TEXT("%s:%d"), *Key, CountsByKey.FindRef(Key)));
		}
		return FString::Join(Parts, TEXT(","));
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
	FTectonicPlanetPreserveOwnershipSamePlateCWRetentionTest,
	"Aurous.TectonicPlanet.PreserveOwnershipSamePlateCWRetentionTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipSamePlateCWRetentionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(0, 10, 1);
	Planet.Samples[3].ContinentalWeight = 1.0f;
	Planet.Samples[3].Elevation = -2.0f;
	ConfigureManualTriangleCarriedSamples(Planet, 0, 0.2f, 1.0f);

	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::PreserveOwnership);

	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	Planet.InterpolateFromCarried(
		Result.NewPlateIds,
		Result.ContainingTriangles,
		Result.BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds);
	const float DilutedContinentalWeight = Planet.Samples[3].ContinentalWeight;
	TestTrue(TEXT("Same-plate preserve interpolation would otherwise dilute continental weight below threshold"), DilutedContinentalWeight < 0.5f);
	const float InterpolatedElevation = Planet.Samples[3].Elevation;

	TArray<float> PreviousContinentalWeights;
	PreviousContinentalWeights.Init(0.0f, Planet.Samples.Num());
	PreviousContinentalWeights[3] = 1.0f;
	FResamplingStats Stats;
	Planet.ApplyPreserveOwnershipContinentalWeightRetention(
		Result.PreserveOwnershipCWRetainFlags,
		PreviousContinentalWeights,
		EResampleOwnershipMode::PreserveOwnership,
		EResampleTriggerReason::Periodic,
		&Stats);

	TestEqual(TEXT("Same-plate preserve CW retention restores the previous continental weight"), Planet.Samples[3].ContinentalWeight, 1.0f);
	TestEqual(TEXT("Same-plate preserve CW retention still records the interpolated elevation"), Planet.Samples[3].Elevation, InterpolatedElevation);
	TestTrue(TEXT("Same-plate preserve CW retention records retained samples"), Stats.PreserveOwnershipCWRetainedCount > 0);
	TestEqual(TEXT("Same-plate preserve CW retention records one prevented continental threshold crossing"), Stats.PreserveOwnershipCWThresholdCrossingPreventedCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveOwnershipFallbackStillInterpolatesCWTest,
	"Aurous.TectonicPlanet.PreserveOwnershipFallbackStillInterpolatesCWTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipFallbackStillInterpolatesCWTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(2, 3, 9);
	Planet.Samples[3].ContinentalWeight = 1.0f;
	ConfigureManualTriangleCarriedSamples(Planet, 1, 0.2f, 1.0f);

	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::PreserveOwnership);

	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	Planet.InterpolateFromCarried(
		Result.NewPlateIds,
		Result.ContainingTriangles,
		Result.BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds);
	const float InterpolatedContinentalWeight = Planet.Samples[3].ContinentalWeight;

	TArray<float> PreviousContinentalWeights;
	PreviousContinentalWeights.Init(0.0f, Planet.Samples.Num());
	PreviousContinentalWeights[3] = 1.0f;
	FResamplingStats Stats;
	Planet.ApplyPreserveOwnershipContinentalWeightRetention(
		Result.PreserveOwnershipCWRetainFlags,
		PreviousContinentalWeights,
		EResampleOwnershipMode::PreserveOwnership,
		EResampleTriggerReason::Periodic,
		&Stats);

	TestEqual(TEXT("Fallback preserve query does not mark the fallback sample for same-plate CW retention"), Result.PreserveOwnershipCWRetainFlags[3], static_cast<uint8>(0));
	TestEqual(TEXT("Fallback preserve query still interpolates continental weight normally"), Planet.Samples[3].ContinentalWeight, InterpolatedContinentalWeight);
	TestEqual(TEXT("Fallback preserve query does not record prevented threshold crossings"), Stats.PreserveOwnershipCWThresholdCrossingPreventedCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetFullResolutionStillInterpolatesCWTest,
	"Aurous.TectonicPlanet.FullResolutionStillInterpolatesCWTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetFullResolutionStillInterpolatesCWTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(2, 3, 9);
	Planet.Samples[3].ContinentalWeight = 1.0f;
	ConfigureManualTriangleCarriedSamples(Planet, 1, 0.2f, 1.0f);

	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::FullResolution);

	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	Planet.InterpolateFromCarried(
		Result.NewPlateIds,
		Result.ContainingTriangles,
		Result.BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds);
	const float InterpolatedContinentalWeight = Planet.Samples[3].ContinentalWeight;
	const int32 PreviousPlateId = Planet.Samples[3].PlateId;

	TArray<int32> PreviousPlateAssignments;
	PreviousPlateAssignments.Init(INDEX_NONE, Planet.Samples.Num());
	PreviousPlateAssignments[3] = PreviousPlateId;
	TArray<float> PreviousContinentalWeights;
	PreviousContinentalWeights.Init(0.0f, Planet.Samples.Num());
	PreviousContinentalWeights[3] = 1.0f;
	TArray<EGapResolutionPath> GapResolutionPaths;
	GapResolutionPaths.Init(EGapResolutionPath::None, Planet.Samples.Num());
	FResamplingStats Stats;
	Planet.ApplyFullResolutionSamePlateContinentalWeightRetention(
		Result.NewPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		GapResolutionPaths,
		EResampleOwnershipMode::FullResolution,
		EResampleTriggerReason::CollisionFollowup,
		&Stats);

	TestTrue(TEXT("FullResolution recomputes continental weight normally"), InterpolatedContinentalWeight < 0.5f);
	TestTrue(TEXT("FullResolution changed-plate scenario actually changes plate ownership"), Result.NewPlateIds[3] != PreviousPlateId);
	TestEqual(TEXT("FullResolution changed-plate path still interpolates continental weight normally"), Planet.Samples[3].ContinentalWeight, InterpolatedContinentalWeight);
	TestEqual(TEXT("FullResolution changed-plate path does not record retained CW samples"), Stats.FullResolutionSamePlateCWRetainedCount, 0);
	TestEqual(TEXT("FullResolution changed-plate path does not record prevented threshold crossings"), Stats.FullResolutionSamePlateCWThresholdCrossingPreventedCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetFullResolutionSamePlateCWRetentionPreservesFormerContinentalTest,
	"Aurous.TectonicPlanet.FullResolutionSamePlateCWRetentionPreservesFormerContinentalTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetFullResolutionSamePlateCWRetentionPreservesFormerContinentalTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(0, 10, 1);
	Planet.Samples[3].ContinentalWeight = 1.0f;
	Planet.Samples[3].Elevation = -2.0f;
	ConfigureManualTriangleCarriedSamples(Planet, 0, 0.2f, 1.0f);

	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::FullResolution);

	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	Planet.InterpolateFromCarried(
		Result.NewPlateIds,
		Result.ContainingTriangles,
		Result.BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds);
	const float DilutedContinentalWeight = Planet.Samples[3].ContinentalWeight;
	TestTrue(TEXT("FullResolution same-plate interpolation would otherwise dilute continental weight below threshold"), DilutedContinentalWeight < 0.5f);
	const float InterpolatedElevation = Planet.Samples[3].Elevation;

	TArray<int32> PreviousPlateAssignments;
	PreviousPlateAssignments.Init(INDEX_NONE, Planet.Samples.Num());
	PreviousPlateAssignments[3] = 0;
	TArray<float> PreviousContinentalWeights;
	PreviousContinentalWeights.Init(0.0f, Planet.Samples.Num());
	PreviousContinentalWeights[3] = 1.0f;
	TArray<EGapResolutionPath> GapResolutionPaths;
	GapResolutionPaths.Init(EGapResolutionPath::None, Planet.Samples.Num());
	FResamplingStats Stats;
	Planet.ApplyFullResolutionSamePlateContinentalWeightRetention(
		Result.NewPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		GapResolutionPaths,
		EResampleOwnershipMode::FullResolution,
		EResampleTriggerReason::RiftFollowup,
		&Stats);

	TestEqual(TEXT("FullResolution same-plate retention restores the previous continental weight"), Planet.Samples[3].ContinentalWeight, 1.0f);
	TestEqual(TEXT("FullResolution same-plate retention keeps interpolated elevation"), Planet.Samples[3].Elevation, InterpolatedElevation);
	TestEqual(TEXT("FullResolution same-plate retention records one retained sample"), Stats.FullResolutionSamePlateCWRetainedCount, 1);
	TestEqual(TEXT("FullResolution same-plate retention records one prevented threshold crossing"), Stats.FullResolutionSamePlateCWThresholdCrossingPreventedCount, 1);
	TestEqual(TEXT("FullResolution same-plate retention records a rift-followup retained sample"), Stats.FullResolutionRiftFollowupSamePlateCWRetainedCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetFullResolutionChangedPlateStillAllowsReclassificationTest,
	"Aurous.TectonicPlanet.FullResolutionChangedPlateStillAllowsReclassificationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetFullResolutionChangedPlateStillAllowsReclassificationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOverlapOwnershipTestPlanet(2, 3, 9);
	Planet.Samples[3].ContinentalWeight = 1.0f;
	ConfigureManualTriangleCarriedSamples(Planet, 1, 0.2f, 1.0f);

	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::FullResolution);

	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	Planet.InterpolateFromCarried(
		Result.NewPlateIds,
		Result.ContainingTriangles,
		Result.BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds);
	const float InterpolatedContinentalWeight = Planet.Samples[3].ContinentalWeight;

	TArray<int32> PreviousPlateAssignments;
	PreviousPlateAssignments.Init(INDEX_NONE, Planet.Samples.Num());
	PreviousPlateAssignments[3] = 2;
	TArray<float> PreviousContinentalWeights;
	PreviousContinentalWeights.Init(0.0f, Planet.Samples.Num());
	PreviousContinentalWeights[3] = 1.0f;
	TArray<EGapResolutionPath> GapResolutionPaths;
	GapResolutionPaths.Init(EGapResolutionPath::None, Planet.Samples.Num());
	FResamplingStats Stats;
	Planet.ApplyFullResolutionSamePlateContinentalWeightRetention(
		Result.NewPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		GapResolutionPaths,
		EResampleOwnershipMode::FullResolution,
		EResampleTriggerReason::CollisionFollowup,
		&Stats);

	TestTrue(TEXT("FullResolution changed-plate retention test actually changes plate ownership"), Result.NewPlateIds[3] != PreviousPlateAssignments[3]);
	TestEqual(TEXT("FullResolution changed-plate path still allows reclassification"), Planet.Samples[3].ContinentalWeight, InterpolatedContinentalWeight);
	TestEqual(TEXT("FullResolution changed-plate path records no retained samples"), Stats.FullResolutionSamePlateCWRetainedCount, 0);
	TestEqual(TEXT("FullResolution changed-plate path records no prevented threshold crossings"), Stats.FullResolutionSamePlateCWThresholdCrossingPreventedCount, 0);
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
	TestTrue(TEXT("Enhanced elevation export exists"), PlatformFile.FileExists(*FPaths::Combine(OutputDirectory, TEXT("ElevationEnhanced.png"))));
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
	FTectonicPlanetContinentalWeightDominantVertexTransferTest,
	"Aurous.TectonicPlanet.ContinentalWeightDominantVertexTransferTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetContinentalWeightDominantVertexTransferTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualContinentalWeightDominantVertexTransferTestPlanet();
	const TArray<float> ContinentalWeights = { 0.15f, 0.85f, 0.35f };
	const TArray<float> Elevations = { 0.5f, 2.0f, 7.0f };
	const TArray<float> Thicknesses = { 12.0f, 80.0f, 21.0f };
	const TArray<float> Ages = { 3.0f, 40.0f, 9.0f };
	ConfigureManualTriangleCarriedSamplesExplicit(
		Planet,
		0,
		ContinentalWeights,
		Elevations,
		Thicknesses,
		Ages);

	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::FullResolution);
	TestEqual(TEXT("Dominant-vertex CW transfer query keeps the manual sample on the only plate"), Result.NewPlateIds[3], 0);
	TestTrue(TEXT("Dominant-vertex CW transfer query finds a containing triangle"), Result.ContainingTriangles[3] != INDEX_NONE);

	const FVector3d Barycentric = Result.BarycentricCoords[3];
	const int32 DominantIndex = PickDominantBarycentricIndexForTest(Barycentric);
	TestEqual(TEXT("Manual CW transfer test biases the query toward vertex 1"), DominantIndex, 1);

	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	Planet.InterpolateFromCarried(
		Result.NewPlateIds,
		Result.ContainingTriangles,
		Result.BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds);

	const float ExpectedDominantContinentalWeight = ContinentalWeights[DominantIndex];
	const float BlendedContinentalWeight = static_cast<float>(
		(Barycentric.X * ContinentalWeights[0]) +
		(Barycentric.Y * ContinentalWeights[1]) +
		(Barycentric.Z * ContinentalWeights[2]));
	TestEqual(
		TEXT("ContinentalWeight now comes from the dominant barycentric source"),
		Planet.Samples[3].ContinentalWeight,
		ExpectedDominantContinentalWeight);
	TestTrue(
		TEXT("Dominant-vertex CW transfer no longer leaves the barycentric-blended CW on the sample"),
		!FMath::IsNearlyEqual(Planet.Samples[3].ContinentalWeight, BlendedContinentalWeight, 1.0e-4f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetContinentalWeightDominantVertexDoesNotAffectOtherFieldInterpolationTest,
	"Aurous.TectonicPlanet.ContinentalWeightDominantVertexDoesNotAffectOtherFieldInterpolationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetContinentalWeightDominantVertexDoesNotAffectOtherFieldInterpolationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualContinentalWeightDominantVertexTransferTestPlanet();
	const TArray<float> ContinentalWeights = { 0.15f, 0.85f, 0.35f };
	const TArray<float> Elevations = { 0.5f, 2.0f, 7.0f };
	const TArray<float> Thicknesses = { 12.0f, 80.0f, 21.0f };
	const TArray<float> Ages = { 3.0f, 40.0f, 9.0f };
	ConfigureManualTriangleCarriedSamplesExplicit(
		Planet,
		0,
		ContinentalWeights,
		Elevations,
		Thicknesses,
		Ages);

	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::FullResolution);
	const FVector3d Barycentric = Result.BarycentricCoords[3];
	const int32 DominantIndex = PickDominantBarycentricIndexForTest(Barycentric);

	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	Planet.InterpolateFromCarried(
		Result.NewPlateIds,
		Result.ContainingTriangles,
		Result.BarycentricCoords,
		SubductionDistances,
		SubductionSpeeds);

	const float ExpectedElevation = static_cast<float>(
		(Barycentric.X * Elevations[0]) +
		(Barycentric.Y * Elevations[1]) +
		(Barycentric.Z * Elevations[2]));
	const float ExpectedThickness = static_cast<float>(
		(Barycentric.X * Thicknesses[0]) +
		(Barycentric.Y * Thicknesses[1]) +
		(Barycentric.Z * Thicknesses[2]));
	const float ExpectedAge = static_cast<float>(
		(Barycentric.X * Ages[0]) +
		(Barycentric.Y * Ages[1]) +
		(Barycentric.Z * Ages[2]));
	TestTrue(
		TEXT("Elevation remains barycentrically interpolated"),
		FMath::IsNearlyEqual(Planet.Samples[3].Elevation, ExpectedElevation, 1.0e-4f));
	TestTrue(
		TEXT("Thickness remains barycentrically interpolated"),
		FMath::IsNearlyEqual(Planet.Samples[3].Thickness, ExpectedThickness, 1.0e-4f));
	TestTrue(
		TEXT("Age remains barycentrically interpolated"),
		FMath::IsNearlyEqual(Planet.Samples[3].Age, ExpectedAge, 1.0e-4f));
	TestTrue(
		TEXT("Elevation is not snapped to the dominant CW vertex"),
		!FMath::IsNearlyEqual(Planet.Samples[3].Elevation, Elevations[DominantIndex], 1.0e-4f));
	TestTrue(
		TEXT("Thickness is not snapped to the dominant CW vertex"),
		!FMath::IsNearlyEqual(Planet.Samples[3].Thickness, Thicknesses[DominantIndex], 1.0e-4f));
	TestTrue(
		TEXT("Age is not snapped to the dominant CW vertex"),
		!FMath::IsNearlyEqual(Planet.Samples[3].Age, Ages[DominantIndex], 1.0e-4f));
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
	for (int32 StepIndex = 0; StepIndex < 60; ++StepIndex)
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
	FTectonicPlanetAutomaticRiftUsesLargestEligibleParentTest,
	"Aurous.TectonicPlanet.AutomaticRiftUsesLargestEligibleParent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetAutomaticRiftUsesLargestEligibleParentTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	ConfigureAutomaticRiftingForTest(Planet, 100.0);

	bool bSawAutomaticRift = false;
	for (int32 StepIndex = 0; StepIndex < 40 && !bSawAutomaticRift; ++StepIndex)
	{
		int32 ExpectedContinentalSampleCount = 0;
		double ExpectedContinentalFraction = 0.0;
		const int32 ExpectedParentPlateId =
			Planet.FindLargestEligibleAutomaticRiftParentId(&ExpectedContinentalSampleCount, &ExpectedContinentalFraction);

		Planet.AdvanceStep();
		const bool bAutomaticRiftThisStep =
			!Planet.ResamplingSteps.IsEmpty() &&
			Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
			Planet.LastResampleTriggerReason == EResampleTriggerReason::RiftFollowup &&
			Planet.LastResamplingStats.RiftCount > 0 &&
			Planet.LastResamplingStats.bRiftWasAutomatic;
		if (!bAutomaticRiftThisStep)
		{
			continue;
		}

		bSawAutomaticRift = true;
		TestEqual(TEXT("Automatic rifting evaluates and uses the largest eligible parent"), Planet.LastResamplingStats.RiftParentPlateId, ExpectedParentPlateId);
		TestTrue(TEXT("The retired automatic-rift parent stable id is no longer active"), Planet.FindPlateArrayIndexById(ExpectedParentPlateId) == INDEX_NONE);
		TestEqual(TEXT("Automatic rift reports the parent continental sample count used for eligibility"), Planet.LastResamplingStats.RiftParentContinentalSampleCount, ExpectedContinentalSampleCount);
		TestTrue(TEXT("Automatic rift reports the parent continental fraction used for eligibility"), FMath::IsNearlyEqual(Planet.LastResamplingStats.RiftParentContinentalFraction, ExpectedContinentalFraction, 1.0e-9));
	}

	TestTrue(TEXT("Largest-parent selection test sees an automatic rift"), bSawAutomaticRift);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetAutomaticRiftProbabilityMonotonicWithParentSizeTest,
	"Aurous.TectonicPlanet.AutomaticRiftProbabilityMonotonicWithParentSize",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetAutomaticRiftProbabilityMonotonicWithParentSizeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	ConfigureAutomaticRiftingForTest(Planet, 0.18);

	const double ContinentalFraction = 0.20;
	const double SmallProbability = Planet.ComputeAutomaticRiftProbabilityForSampleCount(1500, ContinentalFraction);
	const double MediumProbability = Planet.ComputeAutomaticRiftProbabilityForSampleCount(3000, ContinentalFraction);
	const double LargeProbability = Planet.ComputeAutomaticRiftProbabilityForSampleCount(6000, ContinentalFraction);

	TestTrue(TEXT("Automatic-rift probability stays non-negative"), SmallProbability >= 0.0);
	TestTrue(TEXT("Automatic-rift probability is monotonic from small to medium parents"), MediumProbability >= SmallProbability);
	TestTrue(TEXT("Automatic-rift probability is monotonic from medium to large parents"), LargeProbability >= MediumProbability);
	AddInfo(FString::Printf(
		TEXT("automatic_rift_probability_monotonic small=%.6f medium=%.6f large=%.6f"),
		SmallProbability,
		MediumProbability,
		LargeProbability));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetAutomaticRiftUsesBinarySplitInAutomaticModeTest,
	"Aurous.TectonicPlanet.AutomaticRiftUsesBinarySplitInAutomaticMode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetAutomaticRiftUsesBinarySplitInAutomaticModeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	ConfigureAutomaticRiftingForTest(Planet, 100.0);

	bool bSawAutomaticRift = false;
	for (int32 StepIndex = 0; StepIndex < 40 && !bSawAutomaticRift; ++StepIndex)
	{
		Planet.AdvanceStep();
		bSawAutomaticRift =
			!Planet.ResamplingSteps.IsEmpty() &&
			Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
			Planet.LastResampleTriggerReason == EResampleTriggerReason::RiftFollowup &&
			Planet.LastResamplingStats.RiftCount > 0 &&
			Planet.LastResamplingStats.bRiftWasAutomatic;
	}

	TestTrue(TEXT("Binary automatic-rift test sees an automatic rift"), bSawAutomaticRift);
	TestEqual(TEXT("Automatic rifts are clamped to binary splits in M6b"), Planet.LastResamplingStats.RiftChildCount, 2);
	TestEqual(TEXT("Automatic rift reports exactly two child plate ids"), Planet.LastResamplingStats.RiftChildPlateIds.Num(), 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetStablePlateIdsRemainValidThroughAutomaticRiftTest,
	"Aurous.TectonicPlanet.StablePlateIdsRemainValidThroughAutomaticRift",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetStablePlateIdsRemainValidThroughAutomaticRiftTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	ConfigureAutomaticRiftingForTest(Planet, 100.0);

	bool bSawAutomaticRift = false;
	for (int32 StepIndex = 0; StepIndex < 30; ++StepIndex)
	{
		Planet.AdvanceStep();
		TestTrue(
			FString::Printf(TEXT("Stable plate ids remain valid at step %d"), Planet.CurrentStep),
			AreAllStablePlateIdsValid(Planet));

		if (!Planet.ResamplingSteps.IsEmpty() &&
			Planet.ResamplingSteps.Last() == Planet.CurrentStep &&
			Planet.LastResampleTriggerReason == EResampleTriggerReason::RiftFollowup &&
			Planet.LastResamplingStats.RiftCount > 0 &&
			Planet.LastResamplingStats.bRiftWasAutomatic)
		{
			bSawAutomaticRift = true;
		}
	}

	TestTrue(TEXT("The stable-id test experiences an automatic rift"), bSawAutomaticRift);
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
	TestEqual(TEXT("The automatic rift uses a binary split in M6b"), Planet.LastResamplingStats.RiftChildCount, 2);
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
	FTectonicPlanetM6MetricAuditSmokeTest,
	"Aurous.TectonicPlanet.M6MetricAuditSmoke",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetM6MetricAuditSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;

	while (Planet.CurrentStep < 10)
	{
		Planet.AdvanceStep();
	}

	TestEqual(TEXT("Metric audit smoke recorded exactly one resample"), Planet.ResamplingHistory.Num(), 1);
	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestEqual(TEXT("Metric audit smoke resample landed at step 10"), Stats.Step, 10);
	TestEqual(TEXT("Metric audit smoke tracks the active plate count"), Stats.PlateCount, Planet.Plates.Num());
	TestEqual(TEXT("Metric audit smoke tracks the continental sample count"), Stats.ContinentalSampleCount, CountContinentalSamples(Planet));
	TestEqual(TEXT("Metric audit smoke tracks the boundary sample count"), Stats.BoundarySampleCount, CountBoundarySamples(Planet));
	TestTrue(TEXT("Metric audit smoke computes a positive component count"), Stats.MaxComponentsPerPlate > 0);
	TestTrue(TEXT("Metric audit smoke continental fraction is bounded"), Stats.ContinentalAreaFraction >= 0.0 && Stats.ContinentalAreaFraction <= 1.0);
	TestTrue(TEXT("Metric audit smoke boundary fraction is bounded"), Stats.BoundarySampleFraction >= 0.0 && Stats.BoundarySampleFraction <= 1.0);
	TestTrue(TEXT("Metric audit smoke gap rate is bounded"), Stats.GapRate >= 0.0 && Stats.GapRate <= 1.0);
	TestTrue(TEXT("Metric audit smoke overlap rate is bounded"), Stats.OverlapRate >= 0.0 && Stats.OverlapRate <= 1.0);
	TestTrue(TEXT("Metric audit smoke elevation minimum is finite"), FMath::IsFinite(Stats.ElevationMinKm));
	TestTrue(TEXT("Metric audit smoke elevation maximum is finite"), FMath::IsFinite(Stats.ElevationMaxKm));
	TestTrue(TEXT("Metric audit smoke elevation mean is finite"), FMath::IsFinite(Stats.ElevationMeanKm));
	AddInfo(FString::Printf(
		TEXT("m6_metric_audit_smoke step=%d plate_count=%d continental_area_fraction=%.6f boundary_sample_fraction=%.6f gap_rate=%.6f overlap_rate=%.6f max_components_per_plate=%d elevation_min=%.6f elevation_max=%.6f elevation_mean=%.6f"),
		Stats.Step,
		Stats.PlateCount,
		Stats.ContinentalAreaFraction,
		Stats.BoundarySampleFraction,
		Stats.GapRate,
		Stats.OverlapRate,
		Stats.MaxComponentsPerPlate,
		Stats.ElevationMinKm,
		Stats.ElevationMaxKm,
		Stats.ElevationMeanKm));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetContinentalBudgetMetricsSmokeTest,
	"Aurous.TectonicPlanet.ContinentalBudgetMetricsSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetContinentalBudgetMetricsSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;

	while (Planet.CurrentStep < 10)
	{
		Planet.AdvanceStep();
	}

	TestEqual(TEXT("Continental budget smoke recorded exactly one resample"), Planet.ResamplingHistory.Num(), 1);
	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestEqual(TEXT("Continental budget smoke resample landed at step 10"), Stats.Step, 10);
	TestEqual(
		TEXT("Continental budget smoke net delta matches after minus before"),
		Stats.ContinentalSamplesAfter - Stats.ContinentalSamplesBefore,
		Stats.NetContinentalSampleDelta);
	TestEqual(
		TEXT("Continental budget smoke final continental sample count matches the canonical count"),
		Stats.ContinentalSamplesAfter,
		Stats.ContinentalSampleCount);
	TestTrue(TEXT("Continental budget smoke losses are non-negative"), Stats.ContinentalSamplesLost >= 0);
	TestTrue(TEXT("Continental budget smoke gains are non-negative"), Stats.AndeanContinentalGainCount >= 0 && Stats.CollisionContinentalGainCount >= 0);
	TestTrue(
		TEXT("Continental budget smoke former-continental gap oceanization stays within total losses"),
		Stats.FormerContinentalSamplesTurnedOceanic <= Stats.ContinentalSamplesLost);
	TestTrue(
		TEXT("Continental budget smoke former-continental oceanization is bounded by former-continental gap classifications"),
		Stats.FormerContinentalSamplesTurnedOceanic <=
			Stats.FormerContinentalDivergentGapCount + Stats.FormerContinentalNonDivergentGapCount);
	AddInfo(FString::Printf(
		TEXT("continental_budget_smoke step=%d continental_samples_before=%d continental_samples_after=%d continental_samples_lost=%d former_continental_turned_oceanic=%d former_continental_divergent_gap_count=%d former_continental_non_divergent_gap_count=%d andean_continental_gain_count=%d collision_continental_gain_count=%d net_continental_sample_delta=%d"),
		Stats.Step,
		Stats.ContinentalSamplesBefore,
		Stats.ContinentalSamplesAfter,
		Stats.ContinentalSamplesLost,
		Stats.FormerContinentalSamplesTurnedOceanic,
		Stats.FormerContinentalDivergentGapCount,
		Stats.FormerContinentalNonDivergentGapCount,
		Stats.AndeanContinentalGainCount,
		Stats.CollisionContinentalGainCount,
		Stats.NetContinentalSampleDelta));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetContinentalDestructionPathBreakdownSmokeTest,
	"Aurous.TectonicPlanet.ContinentalDestructionPathBreakdownSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetContinentalDestructionPathBreakdownSmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 5;
	Config.SampleCount = 12000;
	Config.MaxStep = 40;
	Config.RunId = TEXT("M6t-DestructionPathBreakdownSmoke");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestTrue(TEXT("Destruction-path smoke produces at least one resample"), Summary.TotalResamples > 0);
	TestTrue(TEXT("Destruction-path smoke keeps stable plate ids valid"), Summary.bStablePlateIdsValid);
	TestTrue(TEXT("Destruction-path smoke oceanized loss count is non-negative"), Summary.TotalFormerContinentalSamplesTurnedOceanic >= 0);
	TestTrue(TEXT("Destruction-path smoke divergent oceanized loss count is non-negative"), Summary.TotalFormerContinentalDivergentOceanized >= 0);
	TestTrue(TEXT("Destruction-path smoke non-divergent fallback oceanized loss count is non-negative"), Summary.TotalFormerContinentalNonDivergentFallbackOceanized >= 0);
	TestTrue(TEXT("Destruction-path smoke non-gap reclassification loss count is non-negative"), Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal >= 0);
	TestTrue(TEXT("Destruction-path smoke changed-plate loss count is non-negative"), Summary.TotalFormerContinentalChangedPlateNonContinentalFinal >= 0);
	TestTrue(TEXT("Destruction-path smoke full-resolution loss count is non-negative"), Summary.TotalFormerContinentalFullResolutionNonContinentalFinal >= 0);
	TestTrue(TEXT("Destruction-path smoke preserve-mode non-gap loss count is non-negative"), Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal >= 0);
	TestTrue(TEXT("Destruction-path smoke fallback-query loss count is non-negative"), Summary.TotalFormerContinentalFallbackQueryNonContinentalFinal >= 0);
	TestTrue(TEXT("Destruction-path smoke projection-recovery loss count is non-negative"), Summary.TotalFormerContinentalProjectionRecoveredNonContinentalFinal >= 0);
	TestTrue(TEXT("Destruction-path smoke nearest-copy recovery loss count is non-negative"), Summary.TotalFormerContinentalNearestCopyRecoveredNonContinentalFinal >= 0);
	TestTrue(TEXT("Destruction-path smoke changed-plate losses stay within the non-gap bucket"), Summary.TotalFormerContinentalChangedPlateNonContinentalFinal <= Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal);
	TestTrue(TEXT("Destruction-path smoke full-resolution losses stay within the non-gap bucket"), Summary.TotalFormerContinentalFullResolutionNonContinentalFinal <= Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal);
	TestTrue(TEXT("Destruction-path smoke preserve-mode losses stay within the non-gap bucket"), Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal <= Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal);
	TestTrue(TEXT("Destruction-path smoke fallback-query losses stay within preserve-mode non-gap losses"), Summary.TotalFormerContinentalFallbackQueryNonContinentalFinal <= Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal);
	AddInfo(FString::Printf(
		TEXT("continental_destruction_path_breakdown_smoke run_id=%s total_continental_samples_lost=%lld total_former_continental_turned_oceanic=%lld total_former_continental_divergent_oceanized=%lld total_former_continental_non_divergent_fallback_oceanized=%lld total_former_continental_non_gap_reclassified_non_continental=%lld total_former_continental_changed_plate_non_continental_final=%lld total_former_continental_full_resolution_non_continental_final=%lld total_former_continental_projection_recovered_non_continental_final=%lld total_former_continental_preserve_mode_non_gap_non_continental_final=%lld total_former_continental_fallback_query_non_continental_final=%lld"),
		*Summary.RunId,
		Summary.TotalContinentalSamplesLost,
		Summary.TotalFormerContinentalSamplesTurnedOceanic,
		Summary.TotalFormerContinentalDivergentOceanized,
		Summary.TotalFormerContinentalNonDivergentFallbackOceanized,
		Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal,
		Summary.TotalFormerContinentalChangedPlateNonContinentalFinal,
		Summary.TotalFormerContinentalFullResolutionNonContinentalFinal,
		Summary.TotalFormerContinentalProjectionRecoveredNonContinentalFinal,
		Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal,
		Summary.TotalFormerContinentalFallbackQueryNonContinentalFinal));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetContinentalLossCategoriesSumCoherentlyTest,
	"Aurous.TectonicPlanet.ContinentalLossCategoriesSumCoherentlyTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetContinentalLossCategoriesSumCoherentlyTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 5;
	Config.SampleCount = 12000;
	Config.MaxStep = 40;
	Config.RunId = TEXT("M6t-LossCategoriesSumCoherently");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	const int64 TopLevelLossCategoryTotal =
		Summary.TotalFormerContinentalSamplesTurnedOceanic +
		Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal;
	TestEqual(
		TEXT("Top-level loss categories reconcile exactly to total continental loss"),
		TopLevelLossCategoryTotal,
		Summary.TotalContinentalSamplesLost);
	TestTrue(
		TEXT("Projection-recovery non-continental losses are a subset of oceanized losses"),
		Summary.TotalFormerContinentalProjectionRecoveredNonContinentalFinal <=
			Summary.TotalFormerContinentalSamplesTurnedOceanic);
	TestTrue(
		TEXT("Nearest-copy recovery non-continental losses are a subset of oceanized losses"),
		Summary.TotalFormerContinentalNearestCopyRecoveredNonContinentalFinal <=
			Summary.TotalFormerContinentalSamplesTurnedOceanic);
	TestTrue(
		TEXT("Direct divergent oceanization is a subset of oceanized losses"),
		Summary.TotalFormerContinentalDivergentOceanized <=
			Summary.TotalFormerContinentalSamplesTurnedOceanic);
	TestTrue(
		TEXT("Direct non-divergent fallback oceanization is a subset of oceanized losses"),
		Summary.TotalFormerContinentalNonDivergentFallbackOceanized <=
			Summary.TotalFormerContinentalSamplesTurnedOceanic);
	TestEqual(TEXT("Non-gap losses partition by ownership mode"), Summary.TotalFormerContinentalFullResolutionNonContinentalFinal + Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal, Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal);
	TestTrue(TEXT("Fallback-query losses are a subset of preserve-mode non-gap losses"), Summary.TotalFormerContinentalFallbackQueryNonContinentalFinal <= Summary.TotalFormerContinentalPreserveModeNonGapNonContinentalFinal);
	AddInfo(FString::Printf(
		TEXT("continental_loss_categories_sum_coherently run_id=%s total_continental_samples_lost=%lld top_level_loss_category_total=%lld non_gap_reclassified=%lld oceanized=%lld projection_recovered_non_continental=%lld nearest_copy_recovered_non_continental=%lld"),
		*Summary.RunId,
		Summary.TotalContinentalSamplesLost,
		TopLevelLossCategoryTotal,
		Summary.TotalFormerContinentalNonGapReclassifiedNonContinentalFinal,
		Summary.TotalFormerContinentalSamplesTurnedOceanic,
		Summary.TotalFormerContinentalProjectionRecoveredNonContinentalFinal,
		Summary.TotalFormerContinentalNearestCopyRecoveredNonContinentalFinal));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetFullResolutionLossSplitSamePlateVsChangedPlateSmokeTest,
	"Aurous.TectonicPlanet.FullResolutionLossSplitSamePlateVsChangedPlateSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetFullResolutionLossSplitSamePlateVsChangedPlateSmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 5;
	Config.SampleCount = 12000;
	Config.MaxStep = 40;
	Config.RunId = TEXT("M6u-FullResolutionLossSplitSameVsChanged");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestTrue(TEXT("Full-resolution split smoke keeps stable plate ids valid"), Summary.bStablePlateIdsValid);
	TestEqual(
		TEXT("Full-resolution same-plate and changed-plate losses sum to the full-resolution loss bucket"),
		Summary.TotalFullResolutionSamePlateNonContinentalFinal +
			Summary.TotalFullResolutionChangedPlateNonContinentalFinal,
		Summary.TotalFormerContinentalFullResolutionNonContinentalFinal);
	AddInfo(FString::Printf(
		TEXT("full_resolution_loss_split_same_vs_changed_smoke run_id=%s total_full_resolution_non_continental=%lld same_plate=%lld changed_plate=%lld"),
		*Summary.RunId,
		Summary.TotalFormerContinentalFullResolutionNonContinentalFinal,
		Summary.TotalFullResolutionSamePlateNonContinentalFinal,
		Summary.TotalFullResolutionChangedPlateNonContinentalFinal));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetFullResolutionLossSplitByTriggerTypeSmokeTest,
	"Aurous.TectonicPlanet.FullResolutionLossSplitByTriggerTypeSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetFullResolutionLossSplitByTriggerTypeSmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 5;
	Config.SampleCount = 12000;
	Config.MaxStep = 40;
	Config.RunId = TEXT("M6u-FullResolutionLossSplitByTrigger");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	const int64 TriggerSplitTotal =
		Summary.TotalFullResolutionCollisionFollowupSamePlateNonContinental +
		Summary.TotalFullResolutionCollisionFollowupChangedPlateNonContinental +
		Summary.TotalFullResolutionRiftFollowupSamePlateNonContinental +
		Summary.TotalFullResolutionRiftFollowupChangedPlateNonContinental +
		Summary.TotalFullResolutionOtherTriggerSamePlateNonContinental +
		Summary.TotalFullResolutionOtherTriggerChangedPlateNonContinental;
	TestEqual(
		TEXT("Full-resolution trigger buckets sum to the full-resolution loss bucket"),
		TriggerSplitTotal,
		Summary.TotalFormerContinentalFullResolutionNonContinentalFinal);
	AddInfo(FString::Printf(
		TEXT("full_resolution_loss_split_by_trigger_smoke run_id=%s total_full_resolution_non_continental=%lld collision_same=%lld collision_changed=%lld rift_same=%lld rift_changed=%lld other_same=%lld other_changed=%lld"),
		*Summary.RunId,
		Summary.TotalFormerContinentalFullResolutionNonContinentalFinal,
		Summary.TotalFullResolutionCollisionFollowupSamePlateNonContinental,
		Summary.TotalFullResolutionCollisionFollowupChangedPlateNonContinental,
		Summary.TotalFullResolutionRiftFollowupSamePlateNonContinental,
		Summary.TotalFullResolutionRiftFollowupChangedPlateNonContinental,
		Summary.TotalFullResolutionOtherTriggerSamePlateNonContinental,
		Summary.TotalFullResolutionOtherTriggerChangedPlateNonContinental));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetM6vBaseline400StepSummarySmokeTest,
	"Aurous.TectonicPlanet.M6vBaseline400StepSummarySmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetM6vBaseline400StepSummarySmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 5;
	Config.SampleCount = 12000;
	Config.MaxStep = 400;
	Config.RunId = TEXT("M6v-Baseline400StepSummarySmoke");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestTrue(TEXT("M6v summary smoke keeps stable plate ids valid"), Summary.bStablePlateIdsValid);
	TestTrue(TEXT("M6v summary smoke records step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestTrue(TEXT("M6v summary smoke records step-300 checkpoint"), Summary.bHasStep300Snapshot);
	TestTrue(TEXT("M6v summary smoke records step-400 checkpoint"), Summary.bHasStep400Snapshot);
	TestEqual(TEXT("M6v summary smoke final snapshot lands at step 400"), Summary.FinalSnapshot.Step, 400);
	AddInfo(FString::Printf(
		TEXT("m6v_baseline_400_step_summary_smoke run_id=%s step200_continental_area_fraction=%.6f step300_continental_area_fraction=%.6f step400_continental_area_fraction=%.6f max_components_per_plate_observed=%d final_max_components_per_plate=%d"),
		*Summary.RunId,
		Summary.Step200Snapshot.ContinentalAreaFraction,
		Summary.Step300Snapshot.ContinentalAreaFraction,
		Summary.Step400Snapshot.ContinentalAreaFraction,
		Summary.MaxComponentsPerPlateObserved,
		Summary.Step400Snapshot.MaxComponentsPerPlate));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetM6vSecondHalfBudgetSplitSmokeTest,
	"Aurous.TectonicPlanet.M6vSecondHalfBudgetSplitSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetM6vSecondHalfBudgetSplitSmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 5;
	Config.SampleCount = 12000;
	Config.MaxStep = 400;
	Config.RunId = TEXT("M6v-SecondHalfBudgetSplitSmoke");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestEqual(
		TEXT("M6v second-half smoke net delta halves sum to total"),
		Summary.FirstHalfNetContinentalSampleDelta + Summary.SecondHalfNetContinentalSampleDelta,
		Summary.TotalNetContinentalSampleDelta);
	TestEqual(
		TEXT("M6v second-half smoke loss halves sum to total"),
		Summary.FirstHalfContinentalSamplesLost + Summary.SecondHalfContinentalSamplesLost,
		Summary.TotalContinentalSamplesLost);
	TestEqual(
		TEXT("M6v second-half smoke Andean gain halves sum to total"),
		Summary.FirstHalfAndeanContinentalGains + Summary.SecondHalfAndeanContinentalGains,
		Summary.TotalAndeanContinentalGains);
	TestEqual(
		TEXT("M6v second-half smoke collision gain halves sum to total"),
		Summary.FirstHalfCollisionContinentalGains + Summary.SecondHalfCollisionContinentalGains,
		Summary.TotalCollisionContinentalGains);
	AddInfo(FString::Printf(
		TEXT("m6v_second_half_budget_split_smoke run_id=%s first_half_net_delta=%lld second_half_net_delta=%lld first_half_loss=%lld second_half_loss=%lld first_half_andean_gains=%lld second_half_andean_gains=%lld first_half_collision_gains=%lld second_half_collision_gains=%lld"),
		*Summary.RunId,
		Summary.FirstHalfNetContinentalSampleDelta,
		Summary.SecondHalfNetContinentalSampleDelta,
		Summary.FirstHalfContinentalSamplesLost,
		Summary.SecondHalfContinentalSamplesLost,
		Summary.FirstHalfAndeanContinentalGains,
		Summary.SecondHalfAndeanContinentalGains,
		Summary.FirstHalfCollisionContinentalGains,
		Summary.SecondHalfCollisionContinentalGains));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetM6BaselinePresetMatchesHarnessAndActorConfigTest,
	"Aurous.TectonicPlanet.M6BaselinePresetMatchesHarnessAndActorConfigTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetM6BaselinePresetUsesPreserveOwnershipPeriodicTest,
	"Aurous.TectonicPlanet.M6BaselinePresetUsesPreserveOwnershipPeriodicTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetM6BaselinePresetFirstPeriodicResampleParitySmokeTest,
	"Aurous.TectonicPlanet.M6BaselinePresetFirstPeriodicResampleParitySmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetArchitectureSpikeAPresetUsesAuthoritativePeriodicTest,
	"Aurous.TectonicPlanet.ArchitectureSpikeAPresetUsesAuthoritativePeriodicTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetArchitectureSpikeAFirstPeriodicResampleUsesFullResolutionSmokeTest,
	"Aurous.TectonicPlanet.ArchitectureSpikeAFirstPeriodicResampleUsesFullResolutionSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetArchitectureSpikeA60k7Test,
	"Aurous.TectonicPlanet.ArchitectureSpikeA60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetM6Baseline60k7Test,
	"Aurous.TectonicPlanet.M6Baseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

	IMPLEMENT_SIMPLE_AUTOMATION_TEST(
		FTectonicPlanetGeometricCollisionOverlapCandidateTest,
		"Aurous.TectonicPlanet.GeometricCollisionOverlapCandidateTest",
		EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionOverlapCandidateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats);

	TestTrue(TEXT("Geometric overlap diagnostic detects read-only overlap candidates"), bDetected);
	TestEqual(TEXT("Geometric overlap diagnostic reports one plate pair"), Stats.GeometricCollisionPairCount, 1);
	TestEqual(TEXT("Geometric overlap diagnostic reports plate A"), Stats.GeometricCollisionBestPlateA, 0);
	TestEqual(TEXT("Geometric overlap diagnostic reports plate B"), Stats.GeometricCollisionBestPlateB, 1);
	TestEqual(TEXT("Geometric overlap diagnostic reports three overlap hits"), Stats.GeometricCollisionBestOverlapSampleCount, 3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionOverlapDoesNotRewriteOwnershipTest,
	"Aurous.TectonicPlanet.GeometricCollisionOverlapDoesNotRewriteOwnershipTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionOverlapDoesNotRewriteOwnershipTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionOverlapPlanet();
	TArray<int32> OriginalPlateIds;
	TArray<int32> OwningPlateIds;
	OriginalPlateIds.Reserve(Planet.Samples.Num());
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OriginalPlateIds.Add(Sample.PlateId);
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats);

	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		TestEqual(
			*FString::Printf(TEXT("Geometric overlap diagnostic keeps canonical plate id unchanged for sample %d"), SampleIndex),
			Planet.Samples[SampleIndex].PlateId,
			OriginalPlateIds[SampleIndex]);
		TestEqual(
			*FString::Printf(TEXT("Geometric overlap diagnostic keeps proposed owning plate id unchanged for sample %d"), SampleIndex),
			OwningPlateIds[SampleIndex],
			OriginalPlateIds[SampleIndex]);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionTerraneEstimateTest,
	"Aurous.TectonicPlanet.GeometricCollisionTerraneEstimateTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionTerraneEstimateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats);

	TestEqual(TEXT("Geometric terrane estimate qualifies one pair"), Stats.GeometricCollisionQualifiedPairCount, 1);
	TestTrue(TEXT("Geometric terrane estimate is non-trivial"), Stats.GeometricCollisionLargestTerraneEstimate >= 4);
	TestTrue(TEXT("Geometric terrane estimate best candidate passes the mass filter"), Stats.bGeometricCollisionBestPassedMassFilter);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionExecutionQueuesFollowupTest,
	"Aurous.TectonicPlanet.GeometricCollisionExecutionQueuesFollowupTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionExecutionQueuesFollowupTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);
	Planet.PendingGeometricCollisionEvent = PendingEvent;
	Planet.bPendingFullResolutionResample = PendingEvent.bValid;

	TestTrue(TEXT("Preserve-mode geometric collision execution detects a geometric candidate"), bDetected);
	TestTrue(TEXT("Preserve-mode geometric collision execution caches a pending event"), Planet.PendingGeometricCollisionEvent.bValid);
	TestTrue(TEXT("Preserve-mode geometric collision execution queues a same-step follow-up"), Planet.bPendingFullResolutionResample);
	TestTrue(TEXT("Preserve-mode geometric collision execution records a qualified geometric pair"), Stats.GeometricCollisionQualifiedPairCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionExecutionUsesQualifiedCandidateTest,
	"Aurous.TectonicPlanet.GeometricCollisionExecutionUsesQualifiedCandidateTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionExecutionUsesQualifiedCandidateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);
	TestTrue(TEXT("Qualified-candidate test produced a geometric preserve candidate"), PendingEvent.bValid);
	Planet.PendingGeometricCollisionEvent = PendingEvent;

	const TArray<FIntPoint> BoundaryEdges = FindConnectedBoundaryContactEdges(Planet, 3);
	TestTrue(TEXT("Qualified-candidate priority test also finds boundary-contact noise"), BoundaryEdges.Num() == 3);
	if (BoundaryEdges.Num() == 3)
	{
		Planet.PendingBoundaryContactCollisionEvent = BuildPendingBoundaryContactCollisionEvent(Planet, BoundaryEdges);
	}

	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	TestFalse(TEXT("Qualified geometric candidate does not fall back to cached boundary-contact execution"), Planet.LastResamplingStats.bUsedCachedBoundaryContactCollision);
	TestEqual(TEXT("Qualified geometric candidate is attempted first for plate A"), Planet.LastResamplingStats.GeometricCollisionExecutedPlateA, PendingEvent.PlateA);
	TestEqual(TEXT("Qualified geometric candidate is attempted first for plate B"), Planet.LastResamplingStats.GeometricCollisionExecutedPlateB, PendingEvent.PlateB);
	TestEqual(TEXT("Qualified geometric candidate carries the cached overlap sample count into follow-up"), Planet.LastResamplingStats.GeometricCollisionExecutedOverlapSampleCount, PendingEvent.OverlapSampleIndices.Num());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionExecutionFallbackToBoundaryContactTest,
	"Aurous.TectonicPlanet.GeometricCollisionExecutionFallbackToBoundaryContactTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionExecutionFallbackToBoundaryContactTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet(2, 8000);
	AssignPlanetToHemispheres(Planet);
	SetAllContinentalWeights(Planet, 1.0f);

	const TArray<FIntPoint> BoundaryEdges = FindConnectedBoundaryContactEdges(Planet, 3);
	TestEqual(TEXT("Boundary-contact fallback test finds a connected three-edge boundary zone"), BoundaryEdges.Num(), 3);
	if (BoundaryEdges.Num() != 3)
	{
		return false;
	}

	Planet.PendingBoundaryContactCollisionEvent = BuildPendingBoundaryContactCollisionEvent(Planet, BoundaryEdges);
	Planet.PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};

	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	TestEqual(TEXT("Boundary-contact fallback still executes a real collision"), Planet.LastResamplingStats.CollisionCount, 1);
	TestTrue(TEXT("Boundary-contact fallback marks cached boundary-contact execution"), Planet.LastResamplingStats.bUsedCachedBoundaryContactCollision);
	TestFalse(TEXT("Boundary-contact fallback does not claim geometric execution"), Planet.LastResamplingStats.bUsedGeometricCollisionExecution);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionExecutionProducesRealCollisionEventTest,
	"Aurous.TectonicPlanet.GeometricCollisionExecutionProducesRealCollisionEventTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionExecutionProducesRealCollisionEventTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;
	ConfigureAutomaticRiftingForTest(Planet, 0.18);

	bool bSawRealGeometricCollision = false;
	while (Planet.CurrentStep < 200)
	{
		Planet.AdvanceStep();
		if (Planet.LastResampleTriggerReason == EResampleTriggerReason::CollisionFollowup &&
			Planet.LastResamplingStats.bUsedGeometricCollisionExecution &&
			Planet.LastResamplingStats.CollisionCount > 0)
		{
			bSawRealGeometricCollision = true;
			break;
		}
	}

	TestTrue(TEXT("Geometric execution produces a real collision event in the preserve-mode simulation"), bSawRealGeometricCollision);
	if (!bSawRealGeometricCollision)
	{
		return false;
	}

	TestTrue(TEXT("Geometric execution recovers a non-empty terrane"), Planet.LastResamplingStats.GeometricCollisionExecutedTerraneRecoveredCount > 0);
	TestTrue(TEXT("Geometric execution applies a non-zero CW boost"), Planet.LastResamplingStats.GeometricCollisionExecutedCWBoostedCount > 0);
	TestTrue(TEXT("Geometric execution applies a non-zero surge footprint"), Planet.LastResamplingStats.GeometricCollisionExecutedSurgeAffectedCount > 0);
	TestFalse(TEXT("Geometric execution clears the cached geometric event after the follow-up"), Planet.PendingGeometricCollisionEvent.bValid);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetDirectionalGeometricOverlapBucketsPreservedTest,
	"Aurous.TectonicPlanet.DirectionalGeometricOverlapBucketsPreservedTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetDirectionalGeometricOverlapBucketsPreservedTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualDirectionalGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);

	TestTrue(TEXT("Directional overlap buckets test detects a qualified geometric candidate"), bDetected);
	TestTrue(TEXT("Directional overlap buckets test caches a pending geometric event"), PendingEvent.bValid);
	TestEqual(TEXT("Directional overlap buckets preserve plate-A-inside-B samples"), PendingEvent.SamplesFromAInsideB.Num(), 3);
	TestEqual(TEXT("Directional overlap buckets preserve plate-B-inside-A samples"), PendingEvent.SamplesFromBInsideA.Num(), 3);
	TestEqual(TEXT("Directional overlap buckets preserve the aggregate overlap sample set"), PendingEvent.OverlapSampleIndices.Num(), 6);
	TestEqual(TEXT("Directional overlap buckets report one directionally qualified pair"), Stats.GeometricCollisionQualifiedDirectionalCount, 1);
	TestEqual(TEXT("Directional overlap buckets report the chosen directional seed count"), Stats.GeometricCollisionDirectionalSeedCount, 3);
	TestEqual(TEXT("Directional overlap buckets report the opposite directional seed count"), Stats.GeometricCollisionDirectionalSeedCountOpposite, 3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetDirectionalGeometricCollisionUsesSubductingSeedBucketTest,
	"Aurous.TectonicPlanet.DirectionalGeometricCollisionUsesSubductingSeedBucketTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetDirectionalGeometricCollisionUsesSubductingSeedBucketTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualDirectionalGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);
	TestTrue(TEXT("Directional seed-bucket execution test detects a pending geometric event"), bDetected && PendingEvent.bValid);
	if (!PendingEvent.bValid)
	{
		return false;
	}

	// Simulate the legacy merged-event weakness by leaving the aggregate overlap set
	// pointed at the opposite direction only. The M6j execution path must still choose
	// the subducting-side directional bucket after polarity selection.
	PendingEvent.OverlapSampleIndices = PendingEvent.SamplesFromAInsideB;
	Planet.PendingGeometricCollisionEvent = PendingEvent;
	Planet.PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};

	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& FollowupStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Directional seed-bucket execution produces one collision"), FollowupStats.CollisionCount, 1);
	TestTrue(TEXT("Directional seed-bucket execution uses geometric collision"), FollowupStats.bUsedGeometricCollisionExecution);
	TestEqual(TEXT("Directional seed-bucket execution resolves the overriding plate"), FollowupStats.CollisionOverridingPlateId, 0);
	TestEqual(TEXT("Directional seed-bucket execution resolves the subducting plate"), FollowupStats.CollisionSubductingPlateId, 1);
	TestEqual(TEXT("Directional seed-bucket execution records the chosen directional seed count"), FollowupStats.GeometricCollisionDirectionalSeedCount, PendingEvent.SamplesFromBInsideA.Num());
	TestEqual(TEXT("Directional seed-bucket execution records the opposite directional seed count"), FollowupStats.GeometricCollisionDirectionalSeedCountOpposite, PendingEvent.SamplesFromAInsideB.Num());
	TestTrue(TEXT("Directional seed-bucket execution recovers a non-trivial terrane"), FollowupStats.GeometricCollisionExecutedTerraneRecoveredCount >= PendingEvent.SamplesFromBInsideA.Num());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetDirectionalGeometricCollisionReducesSeedDirectionRejectionsTest,
	"Aurous.TectonicPlanet.DirectionalGeometricCollisionReducesSeedDirectionRejectionsTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetDirectionalGeometricCollisionReducesSeedDirectionRejectionsTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualDirectionalGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);
	TestTrue(TEXT("Directional rejection-reduction test caches a pending geometric event"), PendingEvent.bValid);
	if (!PendingEvent.bValid)
	{
		return false;
	}

	PendingEvent.OverlapSampleIndices = PendingEvent.SamplesFromAInsideB;
	int32 LegacySubductingSeedCount = 0;
	for (const int32 SampleIndex : PendingEvent.OverlapSampleIndices)
	{
		LegacySubductingSeedCount += (Planet.Samples.IsValidIndex(SampleIndex) && Planet.Samples[SampleIndex].PlateId == 1) ? 1 : 0;
	}
	TestEqual(TEXT("Legacy merged-bucket filtering would see no subducting-side seeds in this scenario"), LegacySubductingSeedCount, 0);

	Planet.PendingGeometricCollisionEvent = PendingEvent;
	Planet.PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& FollowupStats = Planet.LastResamplingStats;
	TestTrue(TEXT("Directional-bucket execution now succeeds where the legacy merged seed list would reject"), FollowupStats.bUsedGeometricCollisionExecution);
	TestEqual(TEXT("Directional-bucket execution avoids a seed-direction rejection"), FollowupStats.GeometricCollisionRejectedBySeedDirectionCount, 0);
	TestEqual(TEXT("Directional-bucket execution keeps boundary-contact fallback unused in this scenario"), FollowupStats.GeometricCollisionFallbackUsedCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionSeedDirectionMismatchMetricsSmokeTest,
	"Aurous.TectonicPlanet.GeometricCollisionSeedDirectionMismatchMetricsSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionSeedDirectionMismatchMetricsSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats);

	TestTrue(TEXT("Seed-direction mismatch smoke detects geometric overlap candidates"), bDetected);
	TestTrue(TEXT("Seed-direction mismatch smoke audits at least one candidate"), Stats.GeometricCollisionSeedDirectionAuditCount > 0);
	TestEqual(
		TEXT("Seed-direction mismatch smoke partitions audited candidates into exactly one directional-bucket state"),
		Stats.GeometricCollisionOnlyOverridingBucketPopulatedCount +
			Stats.GeometricCollisionOnlySubductingBucketPopulatedCount +
			Stats.GeometricCollisionBothDirectionalBucketsPopulatedCount +
			Stats.GeometricCollisionNeitherDirectionalBucketPopulatedCount,
		Stats.GeometricCollisionSeedDirectionAuditCount);
	TestTrue(TEXT("Seed-direction mismatch smoke keeps mismatch count within seed-direction rejections"), Stats.GeometricCollisionSeedDirectionMismatchCount <= Stats.GeometricCollisionRejectedBySeedDirectionCount);
	TestEqual(TEXT("Seed-direction mismatch smoke identifies the one-sided manual overlap as subducting-side populated"), Stats.GeometricCollisionOnlySubductingBucketPopulatedCount, 1);
	TestEqual(TEXT("Seed-direction mismatch smoke keeps the overriding-side-only count at zero"), Stats.GeometricCollisionOnlyOverridingBucketPopulatedCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionOnlyOppositeBucketPopulatedDiagnosticTest,
	"Aurous.TectonicPlanet.GeometricCollisionOnlyOppositeBucketPopulatedDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionOnlyOppositeBucketPopulatedDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOppositeBucketOnlyGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);

	TestTrue(TEXT("Only-opposite-bucket diagnostic detects a geometric candidate"), bDetected);
	TestFalse(TEXT("Only-opposite-bucket diagnostic does not qualify a pending event"), PendingEvent.bValid);
	TestEqual(TEXT("Only-opposite-bucket diagnostic audits one candidate"), Stats.GeometricCollisionSeedDirectionAuditCount, 1);
	TestEqual(TEXT("Only-opposite-bucket diagnostic records one mismatch"), Stats.GeometricCollisionSeedDirectionMismatchCount, 1);
	TestEqual(TEXT("Only-opposite-bucket diagnostic records one overriding-side-only case"), Stats.GeometricCollisionOnlyOverridingBucketPopulatedCount, 1);
	TestEqual(TEXT("Only-opposite-bucket diagnostic records no subducting-side-only cases"), Stats.GeometricCollisionOnlySubductingBucketPopulatedCount, 0);
	TestEqual(TEXT("Only-opposite-bucket diagnostic records no bilateral cases"), Stats.GeometricCollisionBothDirectionalBucketsPopulatedCount, 0);
	TestEqual(TEXT("Only-opposite-bucket diagnostic records no empty-bucket cases"), Stats.GeometricCollisionNeitherDirectionalBucketPopulatedCount, 0);
	TestEqual(TEXT("Only-opposite-bucket diagnostic records one seed-direction rejection"), Stats.GeometricCollisionRejectedBySeedDirectionCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionBothBucketsPopulatedDiagnosticTest,
	"Aurous.TectonicPlanet.GeometricCollisionBothBucketsPopulatedDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionBothBucketsPopulatedDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualDirectionalGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);

	TestTrue(TEXT("Both-buckets diagnostic detects a qualified geometric candidate"), bDetected);
	TestTrue(TEXT("Both-buckets diagnostic qualifies a pending event"), PendingEvent.bValid);
	TestEqual(TEXT("Both-buckets diagnostic audits one candidate"), Stats.GeometricCollisionSeedDirectionAuditCount, 1);
	TestEqual(TEXT("Both-buckets diagnostic records no mismatches"), Stats.GeometricCollisionSeedDirectionMismatchCount, 0);
	TestEqual(TEXT("Both-buckets diagnostic records one bilateral case"), Stats.GeometricCollisionBothDirectionalBucketsPopulatedCount, 1);
	TestEqual(TEXT("Both-buckets diagnostic records no overriding-only cases"), Stats.GeometricCollisionOnlyOverridingBucketPopulatedCount, 0);
	TestEqual(TEXT("Both-buckets diagnostic records no subducting-only cases"), Stats.GeometricCollisionOnlySubductingBucketPopulatedCount, 0);
	TestEqual(TEXT("Both-buckets diagnostic records no empty-bucket cases"), Stats.GeometricCollisionNeitherDirectionalBucketPopulatedCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionQualificationIgnoresSeedDirectionGateTest,
	"Aurous.TectonicPlanet.GeometricCollisionQualificationIgnoresSeedDirectionGateTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionQualificationIgnoresSeedDirectionGateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOppositeBucketOnlyGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);

	TestTrue(TEXT("Qualification ignores seed-direction gate and still detects the collision pair"), bDetected);
	TestEqual(TEXT("Qualification ignores seed-direction gate keeps one candidate"), Stats.GeometricCollisionCandidateCount, 1);
	TestEqual(TEXT("Qualification ignores seed-direction gate keeps one qualified pair"), Stats.GeometricCollisionQualifiedCount, 1);
	TestEqual(TEXT("Qualification ignores seed-direction gate no longer rejects by seed direction"), Stats.GeometricCollisionRejectedBySeedDirectionCount, 0);
	TestEqual(TEXT("Qualification ignores seed-direction gate keeps donor ambiguity at zero"), Stats.GeometricCollisionQualifiedButDonorAmbiguousCount, 0);
	TestEqual(TEXT("Qualification ignores seed-direction gate keeps donor-seed-empty at zero"), Stats.GeometricCollisionQualifiedButDonorSeedEmptyCount, 0);
	TestTrue(TEXT("Qualification ignores seed-direction gate caches a pending event"), PendingEvent.bValid);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionDonorResolutionUsesDirectionalEvidenceTest,
	"Aurous.TectonicPlanet.GeometricCollisionDonorResolutionUsesDirectionalEvidenceTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionDonorResolutionUsesDirectionalEvidenceTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualOppositeBucketOnlyGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);
	TestTrue(TEXT("Directional donor-resolution test qualifies a pending geometric event"), PendingEvent.bValid);
	if (!PendingEvent.bValid)
	{
		return false;
	}

	Planet.PendingGeometricCollisionEvent = PendingEvent;
	Planet.PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& FollowupStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Directional donor-resolution execution produces one collision"), FollowupStats.CollisionCount, 1);
	TestTrue(TEXT("Directional donor-resolution execution uses geometric collision"), FollowupStats.bUsedGeometricCollisionExecution);
	TestEqual(TEXT("Directional donor-resolution marks plate 1 as receiver"), FollowupStats.CollisionOverridingPlateId, 1);
	TestEqual(TEXT("Directional donor-resolution marks plate 0 as donor"), FollowupStats.CollisionSubductingPlateId, 0);
	TestEqual(TEXT("Directional donor-resolution records directional donor execution"), FollowupStats.GeometricCollisionExecutedFromDirectionalPolarityCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionQualifiedPairFallbackDonorRuleTest,
	"Aurous.TectonicPlanet.GeometricCollisionQualifiedPairFallbackDonorRuleTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionQualifiedPairFallbackDonorRuleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualDirectionalGeometricCollisionOverlapPlanet();
	Planet.Plates[0].AngularSpeed = 0.0;
	Planet.Plates[1].AngularSpeed = 0.0;

	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	FPendingGeometricCollisionEvent PendingEvent;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats, &PendingEvent);

	TestTrue(TEXT("Fallback donor-rule test qualifies a pending geometric event"), PendingEvent.bValid);
	if (!PendingEvent.bValid)
	{
		return false;
	}

	TestEqual(TEXT("Fallback donor-rule test uses the fallback donor path once"), Stats.GeometricCollisionQualifiedUsingFallbackDonorRuleCount, 1);
	TestEqual(TEXT("Fallback donor-rule test does not need directional donor resolution"), Stats.GeometricCollisionQualifiedUsingDirectionalDonorCount, 0);
	TestFalse(TEXT("Fallback donor-rule test records a non-directional donor selection"), PendingEvent.bPolarityChosenFromDirectionalEvidence);
	TestEqual(TEXT("Fallback donor-rule test chooses plate 0 as receiver from higher plate score"), PendingEvent.OverridingPlateId, 0);
	TestEqual(TEXT("Fallback donor-rule test chooses plate 1 as donor from lower plate score"), PendingEvent.SubductingPlateId, 1);

	Planet.PendingGeometricCollisionEvent = PendingEvent;
	Planet.PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& FollowupStats = Planet.LastResamplingStats;
	TestTrue(TEXT("Fallback donor-rule execution uses geometric collision"), FollowupStats.bUsedGeometricCollisionExecution);
	TestEqual(TEXT("Fallback donor-rule execution does not mark directional donor execution"), FollowupStats.GeometricCollisionExecutedFromDirectionalPolarityCount, 0);
	TestEqual(TEXT("Fallback donor-rule execution keeps plate 0 as receiver"), FollowupStats.CollisionOverridingPlateId, 0);
	TestEqual(TEXT("Fallback donor-rule execution keeps plate 1 as donor"), FollowupStats.CollisionSubductingPlateId, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionRespectsSingleCollisionPerResampleTest,
	"Aurous.TectonicPlanet.GeometricCollisionRespectsSingleCollisionPerResampleTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionRespectsSingleCollisionPerResampleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionExecutionPlanet();
	TArray<int32> OwningPlateIds;
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats PreserveStats;
	FPendingGeometricCollisionEvent PendingEvent;
	TestTrue(
		TEXT("Single-collision-per-resample test detects a geometric candidate"),
		Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &PreserveStats, &PendingEvent));
	TestTrue(TEXT("Single-collision-per-resample test gets a pending geometric event"), PendingEvent.bValid);
	if (!PendingEvent.bValid)
	{
		return false;
	}

	Planet.PendingGeometricCollisionEvent = PendingEvent;
	Planet.PendingBoundaryContactCollisionEvent.bValid = true;
	Planet.PendingBoundaryContactCollisionEvent.PlateA = 0;
	Planet.PendingBoundaryContactCollisionEvent.PlateB = 1;
	Planet.PendingBoundaryContactCollisionEvent.BoundarySeedSampleIndices = { 3, 4, 5 };
	Planet.PendingBoundaryContactCollisionEvent.ContactCenter = PendingEvent.ContactCenter;
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& FollowupStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Single-collision-per-resample executes exactly one collision"), FollowupStats.CollisionCount, 1);
	TestTrue(TEXT("Single-collision-per-resample uses the geometric path"), FollowupStats.bUsedGeometricCollisionExecution);
	TestFalse(TEXT("Single-collision-per-resample does not also use boundary fallback"), FollowupStats.bUsedCachedBoundaryContactCollision);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionOverlapDepthGateTest,
	"Aurous.TectonicPlanet.GeometricCollisionOverlapDepthGateTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionOverlapDepthGateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet ShallowPlanet = CreateManualGeometricCollisionExecutionPlanet();
	ShallowPlanet.GeometricCollisionMinOverlapDepthKm = 6500.0;
	TArray<int32> ShallowOwningPlateIds;
	for (const FSample& Sample : ShallowPlanet.Samples)
	{
		ShallowOwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats ShallowStats;
	FPendingGeometricCollisionEvent ShallowPendingEvent;
	ShallowPlanet.DetectGeometricCollisionOverlapDiagnostics(
		ShallowOwningPlateIds,
		&ShallowStats,
		&ShallowPendingEvent);
	TestTrue(
		TEXT("Overlap-depth diagnostic records shallow legacy-threshold misses"),
		ShallowStats.GeometricCollisionRejectedByOverlapDepthCount > 0);
	TestTrue(
		TEXT("Overlap-depth diagnostic keeps reporting positive shallow depths"),
		ShallowStats.GeometricCollisionRejectedByOverlapDepthTotalDepthKm > 0.0);

	FTectonicPlanet DeepPlanet = CreateManualGeometricCollisionExecutionPlanet();
	TArray<int32> DeepOwningPlateIds;
	for (const FSample& Sample : DeepPlanet.Samples)
	{
		DeepOwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats DeepStats;
	FPendingGeometricCollisionEvent DeepPendingEvent;
	DeepPlanet.DetectGeometricCollisionOverlapDiagnostics(DeepOwningPlateIds, &DeepStats, &DeepPendingEvent);
	TestTrue(TEXT("Overlap-depth gate still queues a deep candidate"), DeepPendingEvent.bValid);
	if (!DeepPendingEvent.bValid)
	{
		return false;
	}

	DeepPlanet.PendingGeometricCollisionEvent = DeepPendingEvent;
	DeepPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats& DeepFollowupStats = DeepPlanet.LastResamplingStats;
	TestEqual(TEXT("Overlap-depth gate still allows deep collision execution"), DeepFollowupStats.CollisionCount, 1);
	TestTrue(TEXT("Overlap-depth gate records positive overlap depth"), DeepFollowupStats.GeometricCollisionExecutedMaxOverlapDepthKm >= DeepPlanet.GeometricCollisionMinOverlapDepthKm);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionTerraneLocalityClampTest,
	"Aurous.TectonicPlanet.GeometricCollisionTerraneLocalityClampTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionTerraneLocalityClampTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionLocalityClampPlanet();
	const int32 DonorPlateSampleCount = Planet.Plates[1].MemberSamples.Num();
	TArray<int32> SamplesFromAInsideB;
	TArray<int32> SamplesFromBInsideA = { 3, 4, 5 };
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		SamplesFromAInsideB,
		SamplesFromBInsideA);
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& FollowupStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Locality clamp still executes one collision"), FollowupStats.CollisionCount, 1);
	TestTrue(TEXT("Locality clamp uses geometric collision execution"), FollowupStats.bUsedGeometricCollisionExecution);
	TestTrue(
		TEXT("Locality clamp reduces recovered terrane below full donor component"),
		FollowupStats.GeometricCollisionExecutedTerraneRecoveredCount < DonorPlateSampleCount);
	TestTrue(
		TEXT("Locality clamp records that donor terrane growth was limited"),
		FollowupStats.GeometricCollisionExecutedDonorTerraneLocalityLimitedCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionLocalityRelaxationIncreasesRecoveredTerraneTest,
	"Aurous.TectonicPlanet.GeometricCollisionLocalityRelaxationIncreasesRecoveredTerraneTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionLocalityRelaxationIncreasesRecoveredTerraneTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet StrictPlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	StrictPlanet.GeometricCollisionDonorLocalityClampKm = 1500.0;
	StrictPlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		StrictPlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	StrictPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats StrictStats = StrictPlanet.LastResamplingStats;

	FTectonicPlanet RelaxedPlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	const double RelaxedClampKm = RelaxedPlanet.GeometricCollisionDonorLocalityClampKm;
	RelaxedPlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		RelaxedPlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	RelaxedPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats RelaxedStats = RelaxedPlanet.LastResamplingStats;

	TestTrue(TEXT("Strict locality control still executes"), StrictStats.CollisionCount == 1);
	TestTrue(TEXT("Relaxed locality control still executes"), RelaxedStats.CollisionCount == 1);
	TestTrue(TEXT("Relaxed locality clamp is actually looser than the M6m control"), RelaxedClampKm > StrictPlanet.GeometricCollisionDonorLocalityClampKm);
	TestTrue(
		TEXT("Relaxed locality increases recovered terrane size"),
		RelaxedStats.GeometricCollisionExecutedTerraneRecoveredCount >
			StrictStats.GeometricCollisionExecutedTerraneRecoveredCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionLocalityRelaxationPreservesClampTest,
	"Aurous.TectonicPlanet.GeometricCollisionLocalityRelaxationPreservesClampTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionLocalityRelaxationPreservesClampTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionLocalityClampPlanet();
	const int32 DonorPlateSampleCount = Planet.Plates[1].MemberSamples.Num();
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& FollowupStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Relaxed locality clamp still executes one collision"), FollowupStats.CollisionCount, 1);
	TestTrue(
		TEXT("Relaxed locality clamp still prevents whole-donor capture"),
		FollowupStats.GeometricCollisionExecutedTerraneRecoveredCount < DonorPlateSampleCount);
	TestTrue(
		TEXT("Relaxed locality clamp still records limited donor growth"),
		FollowupStats.GeometricCollisionExecutedDonorTerraneLocalityLimitedCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionInfluenceFootprintCanExceedDonorExtentTest,
	"Aurous.TectonicPlanet.GeometricCollisionInfluenceFootprintCanExceedDonorExtentTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionInfluenceFootprintCanExceedDonorExtentTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BasePlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	BasePlanet.GeometricCollisionInfluenceRadiusScale = 1.0;
	BasePlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		BasePlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	BasePlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats BaseStats = BasePlanet.LastResamplingStats;

	FTectonicPlanet WiderPlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	const double WiderInfluenceScale = WiderPlanet.GeometricCollisionInfluenceRadiusScale;
	WiderPlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		WiderPlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	WiderPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats WiderStats = WiderPlanet.LastResamplingStats;

	TestEqual(
		TEXT("Influence radius decoupling keeps donor terrane size unchanged"),
		WiderStats.GeometricCollisionExecutedTerraneRecoveredCount,
		BaseStats.GeometricCollisionExecutedTerraneRecoveredCount);
	TestTrue(TEXT("Influence radius decoupling actually broadens the scale"), WiderInfluenceScale > BasePlanet.GeometricCollisionInfluenceRadiusScale);
	TestTrue(
		TEXT("Influence radius decoupling increases surge radius"),
		WiderStats.CollisionSurgeRadiusRad > BaseStats.CollisionSurgeRadiusRad);
	TestTrue(
		TEXT("Influence radius decoupling keeps donor transfer clamped"),
		WiderStats.GeometricCollisionExecutedDonorTerraneLocalityLimitedCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionInfluenceRelaxationIncreasesCWGainTest,
	"Aurous.TectonicPlanet.GeometricCollisionInfluenceRelaxationIncreasesCWGainTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionInfluenceRelaxationIncreasesCWGainTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BasePlanet = CreateManualGeometricCollisionInfluenceGainPlanet();
	BasePlanet.GeometricCollisionInfluenceRadiusScale = 1.4;
	FCollisionEvent BaseEvent;
	BaseEvent.bDetected = true;
	BaseEvent.OverridingPlateId = 0;
	BaseEvent.SubductingPlateId = 1;
	BaseEvent.CollisionSampleIndices = { 3, 4, 5 };
	BaseEvent.TerraneSampleIndices = { 3, 4, 5 };
	BaseEvent.CollisionCenter =
		(BasePlanet.Samples[3].Position + BasePlanet.Samples[4].Position + BasePlanet.Samples[5].Position).GetSafeNormal();
	FResamplingStats BaseStats;
	BaseStats.bUsedGeometricCollisionExecution = true;
	BasePlanet.ApplyCollisionElevationSurge(BaseEvent, &BaseStats);

	FTectonicPlanet WiderPlanet = CreateManualGeometricCollisionInfluenceGainPlanet();
	const double WiderScale = WiderPlanet.GeometricCollisionInfluenceRadiusScale;
	FCollisionEvent WiderEvent = BaseEvent;
	FResamplingStats WiderStats;
	WiderStats.bUsedGeometricCollisionExecution = true;
	WiderPlanet.ApplyCollisionElevationSurge(WiderEvent, &WiderStats);

	TestEqual(
		TEXT("Influence relaxation keeps donor recovery unchanged"),
		WiderEvent.TerraneSampleIndices.Num(),
		BaseEvent.TerraneSampleIndices.Num());
	TestTrue(TEXT("Influence relaxation broadens the configured radius scale"), WiderScale > BasePlanet.GeometricCollisionInfluenceRadiusScale);
	TestTrue(
		TEXT("Influence relaxation increases surge footprint"),
		WiderStats.CollisionSurgeAffectedCount >
			BaseStats.CollisionSurgeAffectedCount);
	TestTrue(
		TEXT("Influence relaxation increases CW gain"),
		WiderStats.CollisionCWBoostedCount >
			BaseStats.CollisionCWBoostedCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionInfluenceRelaxationPreservesDonorClampTest,
	"Aurous.TectonicPlanet.GeometricCollisionInfluenceRelaxationPreservesDonorClampTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionInfluenceRelaxationPreservesDonorClampTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BasePlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	BasePlanet.GeometricCollisionInfluenceRadiusScale = 1.4;
	const int32 DonorPlateSampleCount = BasePlanet.Plates[1].MemberSamples.Num();
	BasePlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		BasePlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	BasePlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats BaseStats = BasePlanet.LastResamplingStats;

	FTectonicPlanet WiderPlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	WiderPlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		WiderPlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	WiderPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats WiderStats = WiderPlanet.LastResamplingStats;

	TestEqual(
		TEXT("Influence relaxation does not expand donor terrane transfer"),
		WiderStats.GeometricCollisionExecutedTerraneRecoveredCount,
		BaseStats.GeometricCollisionExecutedTerraneRecoveredCount);
	TestTrue(
		TEXT("Influence relaxation still prevents whole-donor capture"),
		WiderStats.GeometricCollisionExecutedTerraneRecoveredCount < DonorPlateSampleCount);
	TestTrue(
		TEXT("Influence relaxation still records locality limiting"),
		WiderStats.GeometricCollisionExecutedDonorTerraneLocalityLimitedCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionInfluencePullbackRemainsBetweenM6qAndM6rTest,
	"Aurous.TectonicPlanet.GeometricCollisionInfluencePullbackRemainsBetweenM6qAndM6rTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionInfluencePullbackRemainsBetweenM6qAndM6rTest::RunTest(const FString& Parameters)
{
	const auto BuildStatsAtScale = [](const double InfluenceScale)
	{
		FTectonicPlanet Planet = CreateManualGeometricCollisionInfluenceGainPlanet();
		Planet.GeometricCollisionInfluenceRadiusScale = InfluenceScale;
		FCollisionEvent Event;
		Event.bDetected = true;
		Event.OverridingPlateId = 0;
		Event.SubductingPlateId = 1;
		Event.CollisionSampleIndices = { 3, 4, 5 };
		Event.TerraneSampleIndices = { 3, 4, 5 };
		Event.CollisionCenter =
			(Planet.Samples[3].Position + Planet.Samples[4].Position + Planet.Samples[5].Position).GetSafeNormal();
		FResamplingStats Stats;
		Stats.bUsedGeometricCollisionExecution = true;
		Planet.ApplyCollisionElevationSurge(Event, &Stats);
		return Stats;
	};

	constexpr double M6qInfluenceScale = 1.8;
	const double M6sInfluenceScale = FTectonicPlanet().GeometricCollisionInfluenceRadiusScale;
	constexpr double M6rInfluenceScale = 2.1;

	const FResamplingStats M6qStats = BuildStatsAtScale(M6qInfluenceScale);
	const FResamplingStats M6sStats = BuildStatsAtScale(M6sInfluenceScale);
	const FResamplingStats M6rStats = BuildStatsAtScale(M6rInfluenceScale);

	TestTrue(TEXT("M6s influence scale is above the M6q value"), M6sInfluenceScale > M6qInfluenceScale);
	TestTrue(TEXT("M6s influence scale is below the M6r value"), M6sInfluenceScale < M6rInfluenceScale);
	TestTrue(TEXT("M6s surge radius stays between the M6q and M6r regimes"), M6sStats.CollisionSurgeRadiusRad > M6qStats.CollisionSurgeRadiusRad && M6sStats.CollisionSurgeRadiusRad < M6rStats.CollisionSurgeRadiusRad);
	TestTrue(TEXT("M6s mean uplift stays between the M6q and M6r regimes"), M6sStats.CollisionSurgeMeanElevationDelta > M6qStats.CollisionSurgeMeanElevationDelta && M6sStats.CollisionSurgeMeanElevationDelta < M6rStats.CollisionSurgeMeanElevationDelta);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetContinentalBudgetSummaryIncludesKeyDrainsAndGainsTest,
	"Aurous.TectonicPlanet.ContinentalBudgetSummaryIncludesKeyDrainsAndGainsTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetContinentalBudgetSummaryIncludesKeyDrainsAndGainsTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunSummary Summary;
	Summary.RunId = TEXT("M6s-BudgetSummary-Test");
	Summary.Seed = 42;
	Summary.TotalContinentalSamplesLost = 11;
	Summary.TotalNetContinentalSampleDelta = -7;
	Summary.TotalFormerContinentalSamplesTurnedOceanic = 5;
	Summary.TotalFormerContinentalDivergentOceanized = 2;
	Summary.TotalAndeanContinentalGains = 13;
	Summary.TotalCollisionContinentalGains = 17;
	Summary.TotalGeometricCollisionExecutionCollisionGains = 19;
	Summary.TotalBoundaryContactFallbackCollisionGains = 3;
	Summary.FinalContinentalAreaFraction = 0.125;

	const FString SummaryLine = FormatM6BaselineBudgetReassessmentSummary(Summary);
	TestTrue(TEXT("Budget reassessment summary includes total continental loss"), SummaryLine.Contains(TEXT("total_continental_samples_lost=11")));
	TestTrue(TEXT("Budget reassessment summary includes net continental delta"), SummaryLine.Contains(TEXT("total_net_continental_sample_delta=-7")));
	TestTrue(TEXT("Budget reassessment summary includes former continental turned oceanic"), SummaryLine.Contains(TEXT("total_former_continental_turned_oceanic=5")));
	TestTrue(TEXT("Budget reassessment summary includes former continental divergent oceanized"), SummaryLine.Contains(TEXT("total_former_continental_divergent_oceanized=2")));
	TestTrue(TEXT("Budget reassessment summary includes Andean gains"), SummaryLine.Contains(TEXT("total_andean_continental_gains=13")));
	TestTrue(TEXT("Budget reassessment summary includes total collision gains"), SummaryLine.Contains(TEXT("total_collision_continental_gains=17")));
	TestTrue(TEXT("Budget reassessment summary includes geometric collision gains"), SummaryLine.Contains(TEXT("geometric_collision_gain_total=19")));
	TestTrue(TEXT("Budget reassessment summary includes fallback collision gains"), SummaryLine.Contains(TEXT("boundary_fallback_collision_gain_total=3")));
	TestTrue(TEXT("Budget reassessment summary includes final continental area fraction"), SummaryLine.Contains(TEXT("final_continental_area_fraction=0.125000")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionPostGateProductivityIncreaseTest,
	"Aurous.TectonicPlanet.GeometricCollisionPostGateProductivityIncreaseTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionPostGateProductivityIncreaseTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BasePlanet = CreateManualGeometricCollisionInfluenceGainPlanet();
	BasePlanet.GeometricCollisionInfluenceRadiusScale = 1.8;
	FCollisionEvent BaseEvent;
	BaseEvent.bDetected = true;
	BaseEvent.OverridingPlateId = 0;
	BaseEvent.SubductingPlateId = 1;
	BaseEvent.CollisionSampleIndices = { 3, 4, 5 };
	BaseEvent.TerraneSampleIndices = { 3, 4, 5 };
	BaseEvent.CollisionCenter =
		(BasePlanet.Samples[3].Position + BasePlanet.Samples[4].Position + BasePlanet.Samples[5].Position).GetSafeNormal();
	FResamplingStats BaseStats;
	BaseStats.bUsedGeometricCollisionExecution = true;
	BasePlanet.ApplyCollisionElevationSurge(BaseEvent, &BaseStats);

	FTectonicPlanet TunedPlanet = CreateManualGeometricCollisionInfluenceGainPlanet();
	const double TunedInfluenceScale = TunedPlanet.GeometricCollisionInfluenceRadiusScale;
	FCollisionEvent TunedEvent = BaseEvent;
	FResamplingStats TunedStats;
	TunedStats.bUsedGeometricCollisionExecution = true;
	TunedPlanet.ApplyCollisionElevationSurge(TunedEvent, &TunedStats);

	TestTrue(TEXT("Post-gate productivity tuning increases the configured influence scale above the M6q baseline"), TunedInfluenceScale > BasePlanet.GeometricCollisionInfluenceRadiusScale);
	TestTrue(
		TEXT("Post-gate productivity tuning increases the geometric surge radius"),
		TunedStats.CollisionSurgeRadiusRad > BaseStats.CollisionSurgeRadiusRad);
	TestTrue(
		TEXT("Post-gate productivity tuning increases geometric mean elevation uplift"),
		TunedStats.CollisionSurgeMeanElevationDelta > BaseStats.CollisionSurgeMeanElevationDelta);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionPostGateTuningPreservesCoherenceClampTest,
	"Aurous.TectonicPlanet.GeometricCollisionPostGateTuningPreservesCoherenceClampTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionPostGateTuningPreservesCoherenceClampTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BasePlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	BasePlanet.GeometricCollisionInfluenceRadiusScale = 1.8;
	const int32 DonorPlateSampleCount = BasePlanet.Plates[1].MemberSamples.Num();
	BasePlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		BasePlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	BasePlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats BaseStats = BasePlanet.LastResamplingStats;

	FTectonicPlanet TunedPlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	TunedPlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		TunedPlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	TunedPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats TunedStats = TunedPlanet.LastResamplingStats;

	TestEqual(TEXT("Post-gate tuning keeps donor transfer extent tied to the same locality clamp"), TunedStats.GeometricCollisionExecutedTerraneRecoveredCount, BaseStats.GeometricCollisionExecutedTerraneRecoveredCount);
	TestTrue(
		TEXT("Post-gate tuning still prevents whole-donor capture"),
		TunedStats.GeometricCollisionExecutedTerraneRecoveredCount < DonorPlateSampleCount);
	TestTrue(
		TEXT("Post-gate tuning still records locality limiting"),
		TunedStats.GeometricCollisionExecutedDonorTerraneLocalityLimitedCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricVsFallbackProductivityBreakdownSmokeTest,
	"Aurous.TectonicPlanet.GeometricVsFallbackProductivityBreakdownSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricVsFallbackProductivityBreakdownSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet GeometricPlanet = CreateManualGeometricCollisionLocalityClampPlanet();
	GeometricPlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		GeometricPlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	GeometricPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats& GeometricStats = GeometricPlanet.LastResamplingStats;

	TestTrue(TEXT("Productivity breakdown smoke records geometric execution"), GeometricStats.bUsedGeometricCollisionExecution);
	TestEqual(TEXT("Productivity breakdown smoke maps geometric collision gains onto the geometric path"), GeometricStats.GeometricCollisionExecutedCollisionGainCount, GeometricStats.CollisionContinentalGainCount);
	TestEqual(TEXT("Productivity breakdown smoke keeps fallback gain count zero for geometric execution"), GeometricStats.BoundaryContactFallbackCollisionGainCount, 0);
	TestEqual(TEXT("Productivity breakdown smoke keeps fallback CW boost count zero for geometric execution"), GeometricStats.BoundaryContactFallbackCWBoostedCount, 0);

	FTectonicPlanet FallbackPlanet = CreateInitializedPlanet(2, 8000);
	AssignPlanetToHemispheres(FallbackPlanet);
	SetAllContinentalWeights(FallbackPlanet, 1.0f);
	const TArray<FIntPoint> BoundaryEdges = FindConnectedBoundaryContactEdges(FallbackPlanet, 3);
	TestEqual(TEXT("Productivity breakdown smoke finds a valid fallback boundary zone"), BoundaryEdges.Num(), 3);
	if (BoundaryEdges.Num() != 3)
	{
		return false;
	}

	FallbackPlanet.PendingBoundaryContactCollisionEvent = BuildPendingBoundaryContactCollisionEvent(FallbackPlanet, BoundaryEdges);
	FallbackPlanet.PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};
	FallbackPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats& FallbackStats = FallbackPlanet.LastResamplingStats;

	TestTrue(TEXT("Productivity breakdown smoke records boundary fallback execution"), FallbackStats.bUsedCachedBoundaryContactCollision);
	TestEqual(TEXT("Productivity breakdown smoke maps fallback collision gains onto the fallback path"), FallbackStats.BoundaryContactFallbackCollisionGainCount, FallbackStats.CollisionContinentalGainCount);
	TestEqual(TEXT("Productivity breakdown smoke maps fallback CW boost onto the fallback path"), FallbackStats.BoundaryContactFallbackCWBoostedCount, FallbackStats.CollisionCWBoostedCount);
	TestEqual(TEXT("Productivity breakdown smoke keeps geometric gain count zero for fallback execution"), FallbackStats.GeometricCollisionExecutedCollisionGainCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGapLossBreakdownMetricsSmokeTest,
	"Aurous.TectonicPlanet.GapLossBreakdownMetricsSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGapLossBreakdownMetricsSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;

	while (Planet.CurrentStep < 10)
	{
		Planet.AdvanceStep();
	}

	TestEqual(TEXT("Gap-loss breakdown smoke recorded exactly one resample"), Planet.ResamplingHistory.Num(), 1);
	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestEqual(TEXT("Gap-loss breakdown smoke resample landed at step 10"), Stats.Step, 10);
	TestTrue(TEXT("Gap-loss breakdown smoke projection recoveries are non-negative"), Stats.FormerContinentalNonDivergentGapProjectionResolvedCount >= 0);
	TestTrue(TEXT("Gap-loss breakdown smoke nearest-copy recoveries are non-negative"), Stats.FormerContinentalNonDivergentGapNearestCopyResolvedCount >= 0);
	TestTrue(TEXT("Gap-loss breakdown smoke non-divergent fallback oceanizations are non-negative"), Stats.FormerContinentalNonDivergentFallbackOceanizedCount >= 0);
	TestTrue(TEXT("Gap-loss breakdown smoke divergent oceanizations are non-negative"), Stats.FormerContinentalDivergentOceanizedCount >= 0);
	TestTrue(
		TEXT("Gap-loss breakdown smoke non-divergent former-continental outcomes stay within the non-divergent classification count"),
		Stats.FormerContinentalNonDivergentGapProjectionResolvedCount +
				Stats.FormerContinentalNonDivergentGapNearestCopyResolvedCount +
				Stats.FormerContinentalNonDivergentFallbackOceanizedCount <=
			Stats.FormerContinentalNonDivergentGapCount);
	TestTrue(
		TEXT("Gap-loss breakdown smoke divergent former-continental oceanizations stay within the divergent classification count"),
		Stats.FormerContinentalDivergentOceanizedCount <= Stats.FormerContinentalDivergentGapCount);
	AddInfo(FString::Printf(
		TEXT("gap_loss_breakdown_smoke step=%d former_continental_divergent_gap_count=%d former_continental_non_divergent_gap_count=%d former_continental_non_divergent_gap_projection_resolved_count=%d former_continental_non_divergent_gap_nearest_copy_resolved_count=%d former_continental_non_divergent_fallback_oceanized_count=%d former_continental_divergent_oceanized_count=%d"),
		Stats.Step,
		Stats.FormerContinentalDivergentGapCount,
		Stats.FormerContinentalNonDivergentGapCount,
		Stats.FormerContinentalNonDivergentGapProjectionResolvedCount,
		Stats.FormerContinentalNonDivergentGapNearestCopyResolvedCount,
		Stats.FormerContinentalNonDivergentFallbackOceanizedCount,
		Stats.FormerContinentalDivergentOceanizedCount));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetResidualProjectionRecoveryCWLossMetricsSmokeTest,
	"Aurous.TectonicPlanet.ResidualProjectionRecoveryCWLossMetricsSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetResidualProjectionRecoveryCWLossMetricsSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateInitializedPlanet();
	Planet.ResamplingPolicy = EResamplingPolicy::PreserveOwnershipPeriodic;

	while (Planet.CurrentStep < 10)
	{
		Planet.AdvanceStep();
	}

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestEqual(TEXT("Residual projection-recovery smoke resample landed at step 10"), Stats.Step, 10);
	TestTrue(TEXT("Residual projection-recovery smoke projection-loss count is non-negative"), Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount >= 0);
	TestTrue(TEXT("Residual projection-recovery smoke same-plate subset is non-negative"), Stats.FormerContinentalProjectionRecoveredSamePlateNonContinentalFinalCount >= 0);
	TestTrue(TEXT("Residual projection-recovery smoke changed-plate subset is non-negative"), Stats.FormerContinentalProjectionRecoveredChangedPlateNonContinentalFinalCount >= 0);
	TestTrue(TEXT("Residual projection-recovery smoke preserve-mode subset is non-negative"), Stats.FormerContinentalProjectionRecoveredPreserveModeNonContinentalFinalCount >= 0);
	TestTrue(TEXT("Residual projection-recovery smoke full-resolution subset is non-negative"), Stats.FormerContinentalProjectionRecoveredFullResolutionNonContinentalFinalCount >= 0);
	TestTrue(TEXT("Residual projection-recovery smoke nearest-copy loss count is non-negative"), Stats.FormerContinentalNearestCopyRecoveredNonContinentalFinalCount >= 0);
	TestEqual(
		TEXT("Residual projection-recovery smoke same/change subsets partition the projection-loss total"),
		Stats.FormerContinentalProjectionRecoveredSamePlateNonContinentalFinalCount +
			Stats.FormerContinentalProjectionRecoveredChangedPlateNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount);
	TestEqual(
		TEXT("Residual projection-recovery smoke preserve/full subsets partition the projection-loss total in current modes"),
		Stats.FormerContinentalProjectionRecoveredPreserveModeNonContinentalFinalCount +
			Stats.FormerContinentalProjectionRecoveredFullResolutionNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount);
	TestTrue(
		TEXT("Residual projection-recovery smoke projection losses stay within former-continental projection recoveries"),
		Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount <=
			Stats.FormerContinentalNonDivergentGapProjectionResolvedCount);
	AddInfo(FString::Printf(
		TEXT("residual_projection_recovery_cw_loss_smoke step=%d projection_recovered_non_continental_final=%d same_plate=%d changed_plate=%d preserve_mode=%d full_resolution=%d nearest_copy=%d"),
		Stats.Step,
		Stats.FormerContinentalProjectionRecoveredNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredSamePlateNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredChangedPlateNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredPreserveModeNonContinentalFinalCount,
		Stats.FormerContinentalProjectionRecoveredFullResolutionNonContinentalFinalCount,
		Stats.FormerContinentalNearestCopyRecoveredNonContinentalFinalCount));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetDivergentGapStillOceanizesNormallyTest,
	"Aurous.TectonicPlanet.DivergentGapStillOceanizesNormallyTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetChangedPlateProjectionRecoveryStillInterpolatesCWTest,
	"Aurous.TectonicPlanet.ChangedPlateProjectionRecoveryStillInterpolatesCWTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetChangedPlateProjectionRecoveryStillInterpolatesCWTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualProjectionGapRecoveryPlanet(/*PreviousPlateId=*/1);
	TArray<int32> NewPlateIds = { 0, 0, 0, INDEX_NONE, 1 };
	TArray<uint8> GapFlags = { 0, 0, 0, 1, 0 };
	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	SubductionDistances.Init(-1.0f, Planet.Samples.Num());
	SubductionSpeeds.Init(0.0f, Planet.Samples.Num());
	FResamplingStats Stats;

	Planet.ResolveGaps(NewPlateIds, GapFlags, SubductionDistances, SubductionSpeeds, &Stats);

	TestEqual(TEXT("Changed-plate projection recovery records one former-continental non-divergent projection resolution"), Stats.FormerContinentalNonDivergentGapProjectionResolvedCount, 1);
	TestEqual(TEXT("Changed-plate projection recovery reassigns the gap sample to the projected plate"), NewPlateIds[3], 0);
	TestTrue(TEXT("Changed-plate projection recovery still interpolates continental weight normally"), Planet.Samples[3].ContinentalWeight < 0.5f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetFullResolutionProjectionRecoveryStillInterpolatesCWTest,
	"Aurous.TectonicPlanet.FullResolutionProjectionRecoveryStillInterpolatesCWTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetFullResolutionProjectionRecoveryStillInterpolatesCWTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualProjectionGapRecoveryPlanet(/*PreviousPlateId=*/0);
	TArray<int32> NewPlateIds = { 0, 0, 0, INDEX_NONE, 1 };
	TArray<uint8> GapFlags = { 0, 0, 0, 1, 0 };
	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	SubductionDistances.Init(-1.0f, Planet.Samples.Num());
	SubductionSpeeds.Init(0.0f, Planet.Samples.Num());
	FResamplingStats Stats;

	Planet.ResolveGaps(NewPlateIds, GapFlags, SubductionDistances, SubductionSpeeds, &Stats);
	const float InterpolatedContinentalWeight = Planet.Samples[3].ContinentalWeight;

	TArray<uint8> FakeRetainFlags;
	FakeRetainFlags.Init(static_cast<uint8>(0), Planet.Samples.Num());
	FakeRetainFlags[3] = 1;
	TArray<float> PreviousContinentalWeights;
	PreviousContinentalWeights.Init(0.0f, Planet.Samples.Num());
	PreviousContinentalWeights[3] = 1.0f;
	FResamplingStats RetentionStats;
	Planet.ApplyPreserveOwnershipContinentalWeightRetention(
		FakeRetainFlags,
		PreviousContinentalWeights,
		EResampleOwnershipMode::FullResolution,
		EResampleTriggerReason::Periodic,
		&RetentionStats);

	TestEqual(TEXT("FullResolution projection recovery records one former-continental non-divergent projection resolution"), Stats.FormerContinentalNonDivergentGapProjectionResolvedCount, 1);
	TestTrue(TEXT("FullResolution projection recovery still interpolates continental weight below threshold"), InterpolatedContinentalWeight < 0.5f);
	TestEqual(TEXT("FullResolution projection recovery ignores preserve-mode CW retention"), Planet.Samples[3].ContinentalWeight, InterpolatedContinentalWeight);
	TestEqual(TEXT("FullResolution projection recovery does not record retained samples"), RetentionStats.PreserveOwnershipCWRetainedCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveOwnershipFallbackSamePlateRetainsCWTest,
	"Aurous.TectonicPlanet.PreserveOwnershipFallbackSamePlateRetainsCWTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipFallbackSamePlateRetainsCWTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	Planet.Samples.SetNum(1);
	Planet.Samples[0].PlateId = 3;
	Planet.Samples[0].ContinentalWeight = 0.05f;

	const TArray<int32> NewPlateIds = { 3 };
	const TArray<int32> PreviousPlateAssignments = { 3 };
	const TArray<float> PreviousContinentalWeights = { 1.0f };
	const TArray<uint8> PreserveOwnershipFallbackQueryFlags = { 1 };
	const TArray<EGapResolutionPath> GapResolutionPaths = { EGapResolutionPath::None };
	FResamplingStats Stats;

	Planet.ApplyPreserveOwnershipFallbackSamePlateRetention(
		NewPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		PreserveOwnershipFallbackQueryFlags,
		GapResolutionPaths,
		EResampleOwnershipMode::PreserveOwnership,
		EResampleTriggerReason::Periodic,
		&Stats);

	TestTrue(TEXT("Preserve fallback same-plate retain restores the previous continental weight"), FMath::IsNearlyEqual(Planet.Samples[0].ContinentalWeight, 1.0f, 1.0e-6f));
	TestEqual(TEXT("Preserve fallback same-plate retain records one recontained sample"), Stats.PreserveOwnershipFallbackSamePlateRecontainedCount, 1);
	TestEqual(TEXT("Preserve fallback same-plate retain records one retained sample"), Stats.PreserveOwnershipFallbackSamePlateRetainedCount, 1);
	TestEqual(TEXT("Preserve fallback same-plate retain records no changed-owner samples"), Stats.PreserveOwnershipFallbackChangedOwnerCount, 0);
	TestEqual(TEXT("Preserve fallback same-plate retain records no gap samples"), Stats.PreserveOwnershipFallbackGapCount, 0);
	TestEqual(TEXT("Preserve fallback same-plate retain clears continental loss after fallback"), Stats.PreserveOwnershipContinentalLossCountAfterFallback, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveOwnershipFallbackChangedOwnerStillAllowedTest,
	"Aurous.TectonicPlanet.PreserveOwnershipFallbackChangedOwnerStillAllowedTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveOwnershipFallbackChangedOwnerStillAllowedTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	Planet.Samples.SetNum(1);
	Planet.Samples[0].PlateId = 3;
	Planet.Samples[0].ContinentalWeight = 0.20f;

	const TArray<int32> NewPlateIds = { 7 };
	const TArray<int32> PreviousPlateAssignments = { 3 };
	const TArray<float> PreviousContinentalWeights = { 1.0f };
	const TArray<uint8> PreserveOwnershipFallbackQueryFlags = { 1 };
	const TArray<EGapResolutionPath> GapResolutionPaths = { EGapResolutionPath::None };
	FResamplingStats Stats;

	Planet.ApplyPreserveOwnershipFallbackSamePlateRetention(
		NewPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		PreserveOwnershipFallbackQueryFlags,
		GapResolutionPaths,
		EResampleOwnershipMode::PreserveOwnership,
		EResampleTriggerReason::Periodic,
		&Stats);

	TestTrue(TEXT("Preserve fallback changed-owner path leaves interpolated CW untouched"), FMath::IsNearlyEqual(Planet.Samples[0].ContinentalWeight, 0.20f, 1.0e-6f));
	TestEqual(TEXT("Preserve fallback changed-owner path records no same-plate retains"), Stats.PreserveOwnershipFallbackSamePlateRetainedCount, 0);
	TestEqual(TEXT("Preserve fallback changed-owner path records one changed-owner sample"), Stats.PreserveOwnershipFallbackChangedOwnerCount, 1);
	TestEqual(TEXT("Preserve fallback changed-owner path leaves one continental loss after fallback"), Stats.PreserveOwnershipContinentalLossCountAfterFallback, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveBoundaryStronglyContinentalPreviousOwnerHysteresisTest,
	"Aurous.TectonicPlanet.PreserveBoundaryStronglyContinentalPreviousOwnerHysteresisTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveBoundaryStronglyContinentalPreviousOwnerHysteresisTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualPreserveBoundaryHysteresisTestPlanet(0.0025, true);
	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::PreserveOwnership);
	AddInfo(FString::Printf(
		TEXT("preserve_boundary_hysteresis_probe new_plate=%d fallback_queries=%d retain_flag=%d gap_count=%d overlap_count=%d hysteresis_count=%d strong_saved_count=%d"),
		Result.NewPlateIds[3],
		Result.Stats.PreserveOwnershipFallbackQueryCount,
		Result.PreserveOwnershipCWRetainFlags[3],
		Result.GapCount,
		Result.OverlapCount,
		Result.Stats.PreserveOwnershipPreviousOwnerHysteresisApplicationCount,
		Result.Stats.PreserveOwnershipStronglyContinentalBoundarySavedCount));

	TestEqual(TEXT("Strongly continental preserve boundary hysteresis keeps the sample on the previous owner"), Result.NewPlateIds[3], 0);
	TestEqual(TEXT("Strongly continental preserve boundary hysteresis records one fallback query"), Result.Stats.PreserveOwnershipFallbackQueryCount, 1);
	TestEqual(TEXT("Strongly continental preserve boundary hysteresis marks the sample for preserve CW retention"), Result.PreserveOwnershipCWRetainFlags[3], static_cast<uint8>(1));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveBoundaryGenuineReassignmentStillAllowedTest,
	"Aurous.TectonicPlanet.PreserveBoundaryGenuineReassignmentStillAllowedTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveBoundaryGenuineReassignmentStillAllowedTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualPreserveBoundaryHysteresisTestPlanet(0.0045, false);
	const FManualOverlapOwnershipQueryResult Result =
		RunManualOverlapOwnershipQuery(Planet, EResampleOwnershipMode::PreserveOwnership);

	TestEqual(TEXT("Preserve boundary policy still allows genuine reassignment when the previous owner is no longer viable"), Result.NewPlateIds[3], 1);
	TestTrue(TEXT("Preserve boundary policy still reaches the fallback query path for the reassigned sample"), Result.Stats.PreserveOwnershipFallbackQueryCount >= 1);
	TestEqual(TEXT("Preserve boundary policy records no previous-owner hysteresis when reassignment is genuine"), Result.Stats.PreserveOwnershipPreviousOwnerHysteresisApplicationCount, 0);
	TestEqual(TEXT("Preserve boundary policy leaves the reassigned sample off the preserve CW-retain path"), Result.PreserveOwnershipCWRetainFlags[3], static_cast<uint8>(0));
	return true;
}

bool FTectonicPlanetDivergentGapStillOceanizesNormallyTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualFormerContinentalDivergentGapPlanet();
	TArray<int32> NewPlateIds = { 0, 1, INDEX_NONE };
	TArray<uint8> GapFlags = { 0, 0, 1 };
	TArray<float> SubductionDistances;
	TArray<float> SubductionSpeeds;
	SubductionDistances.Init(-1.0f, Planet.Samples.Num());
	SubductionSpeeds.Init(0.0f, Planet.Samples.Num());
	FResamplingStats Stats;

	Planet.ResolveGaps(NewPlateIds, GapFlags, SubductionDistances, SubductionSpeeds, &Stats);

	const FSample& GapSample = Planet.Samples[2];
	TestEqual(TEXT("Divergent gap test records one divergent gap"), Stats.DivergentGapCount, 1);
	TestEqual(TEXT("Divergent gap test records one former-continental divergent classification"), Stats.FormerContinentalDivergentGapCount, 1);
	TestEqual(TEXT("Divergent gap test records one former-continental divergent oceanization"), Stats.FormerContinentalDivergentOceanizedCount, 1);
	TestEqual(TEXT("Divergent gap test records no former-continental non-divergent fallback oceanizations"), Stats.FormerContinentalNonDivergentFallbackOceanizedCount, 0);
	TestEqual(TEXT("Divergent gap oceanization clears continental weight"), GapSample.ContinentalWeight, 0.0f);
	TestEqual(TEXT("Divergent gap oceanization resets age"), GapSample.Age, 0.0f);
	TestEqual(TEXT("Divergent gap oceanization resets thickness"), GapSample.Thickness, 7.0f);
	TestEqual(TEXT("Divergent gap oceanization clears orogeny"), GapSample.OrogenyType, EOrogenyType::None);
	TestTrue(TEXT("Divergent gap oceanization assigns an active plate"), Planet.FindPlateArrayIndexById(NewPlateIds[2]) != INDEX_NONE);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionCoverageMetricsSmokeTest,
	"Aurous.TectonicPlanet.GeometricCollisionCoverageMetricsSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionCoverageMetricsSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionOverlapPlanet();
	TArray<int32> OwningPlateIds;
	OwningPlateIds.Reserve(Planet.Samples.Num());
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats Stats;
	const bool bDetected = Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &Stats);

	TestTrue(TEXT("Geometric coverage smoke detects a geometric collision candidate"), bDetected);
	TestEqual(TEXT("Geometric coverage smoke candidate count matches pair count"), Stats.GeometricCollisionCandidateCount, Stats.GeometricCollisionPairCount);
	TestEqual(TEXT("Geometric coverage smoke qualified count matches qualified pair count"), Stats.GeometricCollisionQualifiedCount, Stats.GeometricCollisionQualifiedPairCount);
	TestTrue(TEXT("Geometric coverage smoke has at least one candidate"), Stats.GeometricCollisionCandidateCount > 0);
	TestTrue(TEXT("Geometric coverage smoke keeps qualified count within candidate count"), Stats.GeometricCollisionQualifiedCount <= Stats.GeometricCollisionCandidateCount);
	TestEqual(
		TEXT("Geometric coverage smoke partitions candidates across qualified and rejection buckets"),
		Stats.GeometricCollisionQualifiedCount +
			Stats.GeometricCollisionRejectedByMassFilterCount +
			Stats.GeometricCollisionRejectedByRoleResolutionCount +
			Stats.GeometricCollisionRejectedBySeedDirectionCount,
		Stats.GeometricCollisionCandidateCount);
	TestTrue(TEXT("Geometric coverage smoke keeps empty-terrane rejections within the qualified bucket"), Stats.GeometricCollisionRejectedByEmptyTerraneCount <= Stats.GeometricCollisionQualifiedCount);
	TestEqual(TEXT("Geometric coverage smoke does not mark boundary fallback usage"), Stats.GeometricCollisionFallbackUsedCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionDepthProxyMetricsSmokeTest,
	"Aurous.TectonicPlanet.GeometricCollisionDepthProxyMetricsSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionDepthProxyMetricsSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet ShallowPlanet = CreateManualGeometricCollisionExecutionPlanet();
	ShallowPlanet.GeometricCollisionMinOverlapDepthKm = 6500.0;
	TArray<int32> ShallowOwningPlateIds;
	for (const FSample& Sample : ShallowPlanet.Samples)
	{
		ShallowOwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats ShallowStats;
	FPendingGeometricCollisionEvent ShallowPendingEvent;
	ShallowPlanet.DetectGeometricCollisionOverlapDiagnostics(
		ShallowOwningPlateIds,
		&ShallowStats,
		&ShallowPendingEvent);
	TestTrue(TEXT("Depth-proxy smoke records at least one depth-gated candidate"), ShallowStats.GeometricCollisionRejectedByOverlapDepthCount > 0);
	TestTrue(TEXT("Depth-proxy smoke records positive total depth for depth-gated candidates"), ShallowStats.GeometricCollisionRejectedByOverlapDepthTotalDepthKm > 0.0);
	TestTrue(TEXT("Depth-proxy smoke records positive overlap samples for depth-gated candidates"), ShallowStats.GeometricCollisionRejectedByOverlapDepthTotalOverlapSampleCount > 0);
	TestTrue(TEXT("Depth-proxy smoke records positive terrane estimate for depth-gated candidates"), ShallowStats.GeometricCollisionRejectedByOverlapDepthTotalTerraneEstimate > 0);
	TestTrue(TEXT("Depth-proxy smoke records non-negative mean convergence for depth-gated candidates"), ShallowStats.GeometricCollisionRejectedByOverlapDepthTotalMeanConvergenceKmPerMy >= 0.0);
	TestTrue(TEXT("Depth-proxy smoke records non-negative max convergence for depth-gated candidates"), ShallowStats.GeometricCollisionRejectedByOverlapDepthMaxConvergenceKmPerMy >= 0.0);

	FTectonicPlanet DeepPlanet = CreateManualGeometricCollisionExecutionPlanet();
	TArray<int32> DeepOwningPlateIds;
	for (const FSample& Sample : DeepPlanet.Samples)
	{
		DeepOwningPlateIds.Add(Sample.PlateId);
	}

	FResamplingStats DeepPreserveStats;
	FPendingGeometricCollisionEvent DeepPendingEvent;
	TestTrue(
		TEXT("Depth-proxy smoke queues a deep geometric candidate"),
		DeepPlanet.DetectGeometricCollisionOverlapDiagnostics(DeepOwningPlateIds, &DeepPreserveStats, &DeepPendingEvent));
	TestTrue(TEXT("Depth-proxy smoke gets a valid pending event"), DeepPendingEvent.bValid);
	if (!DeepPendingEvent.bValid)
	{
		return false;
	}

	DeepPlanet.PendingGeometricCollisionEvent = DeepPendingEvent;
	DeepPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats& DeepFollowupStats = DeepPlanet.LastResamplingStats;
	TestEqual(TEXT("Depth-proxy smoke executes one collision"), DeepFollowupStats.CollisionCount, 1);
	TestTrue(TEXT("Depth-proxy smoke records executed overlap sample count"), DeepFollowupStats.GeometricCollisionExecutedOverlapSampleCount > 0);
	TestTrue(TEXT("Depth-proxy smoke records executed terrane estimate"), DeepFollowupStats.GeometricCollisionExecutedTerraneEstimate > 0);
	TestTrue(TEXT("Depth-proxy smoke records executed overlap depth"), DeepFollowupStats.GeometricCollisionExecutedMaxOverlapDepthKm > 0.0);
	TestTrue(TEXT("Depth-proxy smoke records non-negative executed mean convergence"), DeepFollowupStats.GeometricCollisionExecutedMeanConvergenceKmPerMy >= 0.0);
	TestTrue(TEXT("Depth-proxy smoke records executed max convergence at least mean convergence"), DeepFollowupStats.GeometricCollisionExecutedMaxConvergenceKmPerMy >= DeepFollowupStats.GeometricCollisionExecutedMeanConvergenceKmPerMy);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionDepthProxyTemporalEstimateTest,
	"Aurous.TectonicPlanet.GeometricCollisionDepthProxyTemporalEstimateTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionDepthProxyTemporalEstimateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionExecutionPlanet();
	TArray<int32> OwningPlateIds;
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	Planet.CurrentStep = 10;
	FResamplingStats FirstStats;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &FirstStats);
	Planet.CurrentStep = 20;
	FResamplingStats SecondStats;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &SecondStats);

	TestTrue(TEXT("Depth-proxy temporal test records a repeated pair"), SecondStats.GeometricCollisionRepeatedCandidateObservationCount >= 2);
	TestTrue(TEXT("Depth-proxy temporal test records repeated pair ids"), SecondStats.GeometricCollisionRepeatedCandidatePlateA != INDEX_NONE && SecondStats.GeometricCollisionRepeatedCandidatePlateB != INDEX_NONE);
	TestTrue(TEXT("Depth-proxy temporal test records repeated pair overlap samples"), SecondStats.GeometricCollisionRepeatedCandidateOverlapSampleCount > 0);
	TestTrue(TEXT("Depth-proxy temporal test records repeated pair terrane estimate"), SecondStats.GeometricCollisionRepeatedCandidateTerraneEstimate > 0);
	TestTrue(TEXT("Depth-proxy temporal test records repeated pair accumulated penetration"), SecondStats.GeometricCollisionRepeatedCandidateAccumulatedPenetrationKm > 0.0);
	TestTrue(TEXT("Depth-proxy temporal test records repeated pair depth"), SecondStats.GeometricCollisionRepeatedCandidateMaxOverlapDepthKm > 0.0);
	TestTrue(TEXT("Depth-proxy temporal test records non-negative repeated pair mean convergence"), SecondStats.GeometricCollisionRepeatedCandidateMeanConvergenceKmPerMy >= 0.0);
	TestTrue(TEXT("Depth-proxy temporal test records repeated pair max convergence at least mean convergence"), SecondStats.GeometricCollisionRepeatedCandidateMaxConvergenceKmPerMy >= SecondStats.GeometricCollisionRepeatedCandidateMeanConvergenceKmPerMy);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionPersistentPenetrationAccumulatesAcrossResamplesTest,
	"Aurous.TectonicPlanet.GeometricCollisionPersistentPenetrationAccumulatesAcrossResamplesTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionPersistentPenetrationAccumulatesAcrossResamplesTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionExecutionPlanet();
	Planet.GeometricCollisionMinPersistentPenetrationKm = 300.0;
	TArray<int32> OwningPlateIds;
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	const uint64 PairKey = (static_cast<uint64>(0) << 32) | static_cast<uint32>(1);
	Planet.CurrentStep = 10;
	FResamplingStats FirstStats;
	FPendingGeometricCollisionEvent FirstPendingEvent;
	TestTrue(
		TEXT("Persistent penetration accumulation test detects the candidate on first observation"),
		Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &FirstStats, &FirstPendingEvent));
	TestFalse(TEXT("Persistent penetration accumulation test does not execute on the first observation"), FirstPendingEvent.bValid);
	const FGeometricCollisionPairRecurrenceState* FirstState = Planet.GeometricCollisionPairRecurrenceByKey.Find(PairKey);
	TestNotNull(TEXT("Persistent penetration accumulation test records recurrence state after first observation"), FirstState);
	if (FirstState == nullptr)
	{
		return false;
	}
	TestEqual(TEXT("Persistent penetration accumulation test starts with one observation"), FirstState->ObservationCount, 1);
	TestEqual(TEXT("Persistent penetration accumulation test starts with zero accumulated penetration"), FirstState->AccumulatedPenetrationKm, 0.0);

	Planet.CurrentStep = 20;
	FResamplingStats SecondStats;
	FPendingGeometricCollisionEvent SecondPendingEvent;
	TestTrue(
		TEXT("Persistent penetration accumulation test detects the candidate on the second observation"),
		Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &SecondStats, &SecondPendingEvent));
	const FGeometricCollisionPairRecurrenceState* SecondState = Planet.GeometricCollisionPairRecurrenceByKey.Find(PairKey);
	TestNotNull(TEXT("Persistent penetration accumulation test keeps recurrence state on repeated observation"), SecondState);
	if (SecondState == nullptr)
	{
		return false;
	}
	TestTrue(TEXT("Persistent penetration accumulation test increments observation count"), SecondState->ObservationCount >= 2);
	TestTrue(TEXT("Persistent penetration accumulation test accumulates positive penetration"), SecondState->AccumulatedPenetrationKm > 0.0);
	TestTrue(TEXT("Persistent penetration accumulation test records positive effective convergence"), SecondState->LastEffectiveConvergenceKmPerMy > 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionPersistentPenetrationResetsWhenPairDisappearsTest,
	"Aurous.TectonicPlanet.GeometricCollisionPersistentPenetrationResetsWhenPairDisappearsTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionPersistentPenetrationResetsWhenPairDisappearsTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionExecutionPlanet();
	Planet.GeometricCollisionMinPersistentPenetrationKm = 300.0;
	TArray<int32> OwningPlateIds;
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	const uint64 PairKey = (static_cast<uint64>(0) << 32) | static_cast<uint32>(1);
	Planet.CurrentStep = 10;
	FResamplingStats FirstStats;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &FirstStats);
	Planet.CurrentStep = 20;
	FResamplingStats SecondStats;
	Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &SecondStats);
	TestTrue(TEXT("Persistent penetration reset test builds recurrence state before disappearance"), Planet.GeometricCollisionPairRecurrenceByKey.Contains(PairKey));

	Planet.Plates[0].SoupTriangles.Reset();
	Planet.Plates[1].SoupTriangles.Reset();
	Planet.BuildContainmentSoups();
	Planet.CurrentStep = 30;
	FResamplingStats ThirdStats;
	const bool bDetectedAfterDisappearance =
		Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &ThirdStats);
	TestFalse(TEXT("Persistent penetration reset test no longer detects the pair after disappearance"), bDetectedAfterDisappearance);
	TestFalse(TEXT("Persistent penetration reset test clears recurrence state when the pair disappears"), Planet.GeometricCollisionPairRecurrenceByKey.Contains(PairKey));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetGeometricCollisionPersistentPenetrationGateTest,
	"Aurous.TectonicPlanet.GeometricCollisionPersistentPenetrationGateTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetGeometricCollisionPersistentPenetrationGateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionExecutionPlanet();
	Planet.GeometricCollisionMinPersistentPenetrationKm = 300.0;
	TArray<int32> OwningPlateIds;
	for (const FSample& Sample : Planet.Samples)
	{
		OwningPlateIds.Add(Sample.PlateId);
	}

	Planet.CurrentStep = 10;
	FResamplingStats FirstStats;
	FPendingGeometricCollisionEvent FirstPendingEvent;
	TestTrue(
		TEXT("Persistent penetration gate test detects the initial candidate"),
		Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &FirstStats, &FirstPendingEvent));
	TestFalse(TEXT("Persistent penetration gate test blocks execution below the threshold"), FirstPendingEvent.bValid);
	TestTrue(
		TEXT("Persistent penetration gate test records a penetration-gated rejection"),
		FirstStats.GeometricCollisionRejectedByPersistentPenetrationCount > 0);

	Planet.CurrentStep = 20;
	FResamplingStats SecondStats;
	FPendingGeometricCollisionEvent SecondPendingEvent;
	TestTrue(
		TEXT("Persistent penetration gate test detects the repeated candidate"),
		Planet.DetectGeometricCollisionOverlapDiagnostics(OwningPlateIds, &SecondStats, &SecondPendingEvent));
	TestTrue(TEXT("Persistent penetration gate test queues execution after enough penetration accumulates"), SecondPendingEvent.bValid);
	TestTrue(TEXT("Persistent penetration gate test stores accumulated penetration above threshold"), SecondPendingEvent.AccumulatedPenetrationKm >= Planet.GeometricCollisionMinPersistentPenetrationKm);
	TestTrue(TEXT("Persistent penetration gate test records multiple observations before execution"), SecondPendingEvent.ObservationCount >= 2);

	Planet.PendingGeometricCollisionEvent = SecondPendingEvent;
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);
	const FResamplingStats& FollowupStats = Planet.LastResamplingStats;
	TestEqual(TEXT("Persistent penetration gate test executes one collision"), FollowupStats.CollisionCount, 1);
	TestTrue(TEXT("Persistent penetration gate test uses the geometric collision path"), FollowupStats.bUsedGeometricCollisionExecution);
	TestTrue(TEXT("Persistent penetration gate test reports accumulated penetration on execution"), FollowupStats.GeometricCollisionExecutedAccumulatedPenetrationKm >= Planet.GeometricCollisionMinPersistentPenetrationKm);
	TestTrue(TEXT("Persistent penetration gate test reports repeated observations on execution"), FollowupStats.GeometricCollisionExecutedObservationCount >= 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetAndeanSensitivityHarnessSmokeTest,
	"Aurous.TectonicPlanet.AndeanSensitivityHarnessSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetAndeanSensitivityHarnessSmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig BaselineConfig;
	BaselineConfig.Seed = TestRandomSeed;
	BaselineConfig.PlateCount = 5;
	BaselineConfig.SampleCount = 12000;
	BaselineConfig.MaxStep = 40;
	BaselineConfig.RunId = TEXT("M6i-AndeanSensitivityHarnessSmoke-Baseline");
	BaselineConfig.bExportCheckpoints = false;
	BaselineConfig.bLogCheckpoints = false;
	BaselineConfig.bLogSummary = false;

	const FM6BaselineRunSummary BaselineRunA = RunM6BaselineScenario(*this, BaselineConfig);
	const FM6BaselineRunSummary BaselineRunB = RunM6BaselineScenario(*this, BaselineConfig);

	TestEqual(TEXT("Andean sensitivity smoke baseline run is deterministic for rift count"), BaselineRunA.TotalRiftEvents, BaselineRunB.TotalRiftEvents);
	TestEqual(TEXT("Andean sensitivity smoke baseline run is deterministic for collision count"), BaselineRunA.TotalCollisionEvents, BaselineRunB.TotalCollisionEvents);
	TestEqual(TEXT("Andean sensitivity smoke baseline run is deterministic for Andean gains"), BaselineRunA.TotalAndeanContinentalGains, BaselineRunB.TotalAndeanContinentalGains);
	TestEqual(TEXT("Andean sensitivity smoke baseline run is deterministic for net continental delta"), BaselineRunA.TotalNetContinentalSampleDelta, BaselineRunB.TotalNetContinentalSampleDelta);
	TestTrue(TEXT("Andean sensitivity smoke baseline run keeps stable ids valid"), BaselineRunA.bStablePlateIdsValid);

	FM6BaselineRunConfig DoubledConfig = BaselineConfig;
	DoubledConfig.RunId = TEXT("M6i-AndeanSensitivityHarnessSmoke-Doubled");
	DoubledConfig.AndeanContinentalConversionRatePerMy = BaselineConfig.AndeanContinentalConversionRatePerMy * 2.0;

	const FM6BaselineRunSummary DoubledRun = RunM6BaselineScenario(*this, DoubledConfig);
	TestTrue(TEXT("Andean sensitivity smoke doubled run produces at least one resample"), DoubledRun.TotalResamples > 0);
	TestEqual(TEXT("Andean sensitivity smoke uses the doubled Andean rate"), DoubledRun.AndeanContinentalConversionRatePerMy, BaselineConfig.AndeanContinentalConversionRatePerMy * 2.0);
	TestTrue(TEXT("Andean sensitivity smoke doubled run keeps stable ids valid"), DoubledRun.bStablePlateIdsValid);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftTopologyAttributionSmokeTest,
	"Aurous.TectonicPlanet.RiftTopologyAttributionSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftTopologyAttributionSmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 7;
	Config.SampleCount = 16000;
	Config.MaxStep = 200;
	Config.RunId = TEXT("M6ab-RiftTopologyAttributionSmoke");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestTrue(TEXT("Rift topology attribution smoke records at least one resample"), Summary.TotalResamples > 0);
	TestTrue(TEXT("Rift topology attribution smoke records at least one rift followup"), Summary.TotalRiftEvents > 0);
	TestTrue(TEXT("Rift topology attribution smoke keeps rift component increase non-negative"), Summary.TotalRiftFollowupComponentIncrease >= 0);
	TestEqual(
		TEXT("Rift topology attribution smoke rift window counts sum to total rifts"),
		Summary.RiftFollowupCount0To100 +
			Summary.RiftFollowupCount100To200 +
			Summary.RiftFollowupCount200To400,
		Summary.TotalRiftEvents);
	if (Summary.bHasLargestRiftTopologyIncreaseEvent)
	{
		TestEqual(
			TEXT("Rift topology attribution smoke largest rift event is tagged as a rift followup"),
			Summary.LargestRiftTopologyIncreaseEventStats.TriggerReason,
			EResampleTriggerReason::RiftFollowup);
		TestTrue(
			TEXT("Rift topology attribution smoke records parent component counts"),
			Summary.LargestRiftTopologyIncreaseEventStats.RiftParentComponentsBefore >= 0 &&
				Summary.LargestRiftTopologyIncreaseEventStats.RiftParentComponentsAfter >= 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftTopologyWindowedGrowthSummarySmokeTest,
	"Aurous.TectonicPlanet.RiftTopologyWindowedGrowthSummarySmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftTopologyWindowedGrowthSummarySmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 7;
	Config.SampleCount = 16000;
	Config.MaxStep = 200;
	Config.RunId = TEXT("M6ab-RiftTopologyWindowedGrowthSmoke");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestEqual(
		TEXT("Rift topology windowed smoke component increases sum coherently"),
		Summary.RiftFollowupComponentIncrease0To100 +
			Summary.RiftFollowupComponentIncrease100To200 +
			Summary.RiftFollowupComponentIncrease200To400,
		Summary.TotalRiftFollowupComponentIncrease);
	TestEqual(
		TEXT("Rift topology windowed smoke counts sum coherently"),
		Summary.RiftFollowupCount0To100 +
			Summary.RiftFollowupCount100To200 +
			Summary.RiftFollowupCount200To400,
		Summary.TotalRiftEvents);
	AddInfo(FormatM6BaselineRiftTopologyAttributionSummary(Summary));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftWorstSpikeSummarySmokeTest,
	"Aurous.TectonicPlanet.RiftWorstSpikeSummarySmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftWorstSpikeSummarySmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 7;
	Config.SampleCount = 16000;
	Config.MaxStep = 200;
	Config.RunId = TEXT("M6ab-RiftWorstSpikeSmoke");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestTrue(TEXT("Rift worst-spike smoke records a rift spike event"), Summary.bHasLargestRiftTopologyIncreaseEvent);
	if (!Summary.bHasLargestRiftTopologyIncreaseEvent)
	{
		return false;
	}

	TestEqual(
		TEXT("Rift worst-spike smoke largest rift event is tagged as a rift followup"),
		Summary.LargestRiftTopologyIncreaseEventStats.TriggerReason,
		EResampleTriggerReason::RiftFollowup);
	TestTrue(
		TEXT("Rift worst-spike smoke records a valid step"),
		Summary.LargestRiftTopologyIncreaseEventStats.Step != INDEX_NONE);
	TestTrue(
		TEXT("Rift worst-spike smoke stays within run-wide max components"),
		Summary.LargestRiftTopologyIncreaseEventStats.TopologyGlobalMaxComponentsAfter <=
			Summary.MaxComponentsPerPlateObserved);
	TestEqual(
		TEXT("Rift worst-spike smoke keeps child plate/component list sizes aligned"),
		Summary.LargestRiftTopologyIncreaseEventStats.RiftChildPlateIds.Num(),
		Summary.LargestRiftTopologyIncreaseEventStats.RiftChildComponentCountsAfter.Num());
	AddInfo(FormatM6BaselineWorstRiftSpikeSummary(Summary));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetTopologyDriftAttributionByEventTypeSmokeTest,
	"Aurous.TectonicPlanet.TopologyDriftAttributionByEventTypeSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetTopologyDriftAttributionByEventTypeSmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 7;
	Config.SampleCount = 16000;
	Config.MaxStep = 200;
	Config.RunId = TEXT("M6w-TopologyDriftAttributionSmoke");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestTrue(TEXT("Topology drift attribution smoke records at least one resample"), Summary.TotalResamples > 0);
	TestTrue(TEXT("Topology drift attribution smoke records a peak topology spike event"), Summary.bHasPeakTopologySpikeEvent);
	TestTrue(TEXT("Topology drift attribution smoke keeps collision component increase non-negative"), Summary.TotalCollisionFollowupComponentIncrease >= 0);
	TestTrue(TEXT("Topology drift attribution smoke keeps rift component increase non-negative"), Summary.TotalRiftFollowupComponentIncrease >= 0);
	TestEqual(
		TEXT("Topology drift attribution smoke collision window totals sum coherently"),
		Summary.CollisionFollowupComponentIncrease0To100 +
			Summary.CollisionFollowupComponentIncrease100To200 +
			Summary.CollisionFollowupComponentIncrease200To400,
		Summary.TotalCollisionFollowupComponentIncrease);
	TestEqual(
		TEXT("Topology drift attribution smoke rift window totals sum coherently"),
		Summary.RiftFollowupComponentIncrease0To100 +
			Summary.RiftFollowupComponentIncrease100To200 +
			Summary.RiftFollowupComponentIncrease200To400,
		Summary.TotalRiftFollowupComponentIncrease);
	if (Summary.bHasPeakTopologySpikeEvent)
	{
		TestTrue(
			TEXT("Topology drift attribution smoke peak followup event does not exceed the run-wide observed max"),
			Summary.PeakTopologySpikeEventStats.TopologyGlobalMaxComponentsAfter <=
				Summary.MaxComponentsPerPlateObserved);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionReceiverContiguityDiagnosticSmokeTest,
	"Aurous.TectonicPlanet.CollisionReceiverContiguityDiagnosticSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionReceiverContiguityDiagnosticSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionLocalityClampPlanet();
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestEqual(TEXT("Collision receiver contiguity smoke executes one collision"), Stats.CollisionCount, 1);
	TestTrue(TEXT("Collision receiver contiguity smoke uses geometric execution"), Stats.bUsedGeometricCollisionExecution);
	TestTrue(TEXT("Collision receiver contiguity smoke records receiver components before"), Stats.CollisionReceiverComponentsBefore > 0);
	TestTrue(TEXT("Collision receiver contiguity smoke records receiver components after"), Stats.CollisionReceiverComponentsAfter > 0);
	TestTrue(TEXT("Collision receiver contiguity smoke keeps disconnected fragment count non-negative"), Stats.CollisionReceiverDisconnectedFragmentCount >= 0);
	TestTrue(TEXT("Collision receiver contiguity smoke keeps largest disconnected fragment size non-negative"), Stats.CollisionReceiverLargestNewDisconnectedFragmentSize >= 0);
	if (Stats.CollisionReceiverDisconnectedFragmentCount == 0)
	{
		TestEqual(
			TEXT("Collision receiver contiguity smoke reports zero largest disconnected fragment when none are created"),
			Stats.CollisionReceiverLargestNewDisconnectedFragmentSize,
			0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetTopologySpikeEventSummarySmokeTest,
	"Aurous.TectonicPlanet.TopologySpikeEventSummarySmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetTopologySpikeEventSummarySmokeTest::RunTest(const FString& Parameters)
{
	FM6BaselineRunConfig Config;
	Config.Seed = TestRandomSeed;
	Config.PlateCount = 7;
	Config.SampleCount = 16000;
	Config.MaxStep = 200;
	Config.RunId = TEXT("M6w-TopologySpikeSummarySmoke");
	Config.bExportCheckpoints = false;
	Config.bLogCheckpoints = false;
	Config.bLogSummary = false;

	const FM6BaselineRunSummary Summary = RunM6BaselineScenario(*this, Config);
	TestTrue(TEXT("Topology spike summary smoke records a peak event"), Summary.bHasPeakTopologySpikeEvent);
	if (!Summary.bHasPeakTopologySpikeEvent)
	{
		return false;
	}

	TestTrue(TEXT("Topology spike summary smoke records a valid step"), Summary.PeakTopologySpikeEventStats.Step != INDEX_NONE);
	TestTrue(
		TEXT("Topology spike summary smoke peak followup event does not exceed the run-wide observed max"),
		Summary.PeakTopologySpikeEventStats.TopologyGlobalMaxComponentsAfter <=
			Summary.MaxComponentsPerPlateObserved);
	TestTrue(
		TEXT("Topology spike summary smoke peak event is a topology-changing followup"),
		Summary.PeakTopologySpikeEventStats.TriggerReason == EResampleTriggerReason::CollisionFollowup ||
			Summary.PeakTopologySpikeEventStats.TriggerReason == EResampleTriggerReason::RiftFollowup);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionDonorTrimCauseSplitMetricsSmokeTest,
	"Aurous.TectonicPlanet.CollisionDonorTrimCauseSplitMetricsSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionDonorTrimCauseSplitMetricsSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualCollisionDonorComponentCapPathologyPlanet();
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestTrue(TEXT("Donor trim cause smoke records proposed terrane size"), Stats.CollisionProposedTerraneSampleCount > 0);
	TestTrue(TEXT("Donor trim cause smoke keeps accepted terrane size non-negative"), Stats.CollisionAcceptedTerraneSampleCount >= 0);
	TestTrue(TEXT("Donor trim cause smoke keeps accepted terrane size bounded by proposed size"), Stats.CollisionAcceptedTerraneSampleCount <= Stats.CollisionProposedTerraneSampleCount);
	TestTrue(TEXT("Donor trim cause smoke records trim ratio in range"), Stats.CollisionTransferTrimRatio >= 0.0 && Stats.CollisionTransferTrimRatio <= 1.0);
	TestTrue(TEXT("Donor trim cause smoke records cap-driven donor protection"), Stats.CollisionTrimmedByDonorComponentCapCount > 0 || Stats.CollisionRejectedByDonorComponentCapCount > 0);
	TestEqual(TEXT("Donor trim cause smoke keeps fragment-floor trim count at zero"), Stats.CollisionTrimmedByDonorFragmentFloorCount, 0);
	TestEqual(TEXT("Donor trim cause smoke keeps fragment-floor reject count at zero"), Stats.CollisionRejectedByDonorFragmentFloorCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionDonorComponentCapRelaxationImprovesTransferTest,
	"Aurous.TectonicPlanet.CollisionDonorComponentCapRelaxationImprovesTransferTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionDonorComponentCapRelaxationImprovesTransferTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet StrictPlanet = CreateManualCollisionDonorCapRelaxationPlanet();
	StrictPlanet.GeometricCollisionDonorMaxComponentIncrease = 5;
	StrictPlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		StrictPlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	StrictPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	FTectonicPlanet RelaxedPlanet = CreateManualCollisionDonorCapRelaxationPlanet();
	RelaxedPlanet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		RelaxedPlanet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	RelaxedPlanet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& StrictStats = StrictPlanet.LastResamplingStats;
	const FResamplingStats& RelaxedStats = RelaxedPlanet.LastResamplingStats;
	TestEqual(TEXT("Strict donor cap still executes the moderate donor transfer"), StrictStats.CollisionCount, 1);
	TestEqual(TEXT("Relaxed donor cap still executes the moderate donor transfer"), RelaxedStats.CollisionCount, 1);
	TestTrue(
		TEXT("Relaxed donor cap accepts at least as much terrane as the strict cap"),
		RelaxedStats.CollisionAcceptedTerraneSampleCount >= StrictStats.CollisionAcceptedTerraneSampleCount);
	TestTrue(
		TEXT("Relaxed donor cap trims no more aggressively than the strict cap"),
		RelaxedStats.CollisionTransferTrimRatio <= StrictStats.CollisionTransferTrimRatio);
	TestTrue(
		TEXT("Strict donor cap actually trims or rejects the moderate donor case"),
		StrictStats.CollisionTrimmedByDonorProtectionCount > 0 || StrictStats.CollisionRejectedByDonorProtectionCount > 0);
	TestEqual(TEXT("Relaxed donor cap allows the moderate donor case without donor trim"), RelaxedStats.CollisionTrimmedByDonorProtectionCount, 0);
	TestEqual(TEXT("Relaxed donor cap allows the moderate donor case without donor rejection"), RelaxedStats.CollisionRejectedByDonorProtectionCount, 0);
	TestEqual(TEXT("Relaxed donor cap keeps the fragment floor disabled"), RelaxedStats.CollisionTrimmedByDonorFragmentFloorCount + RelaxedStats.CollisionRejectedByDonorFragmentFloorCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionDonorComponentCapRelaxationStillBlocksPathologyTest,
	"Aurous.TectonicPlanet.CollisionDonorComponentCapRelaxationStillBlocksPathologyTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionDonorComponentCapRelaxationStillBlocksPathologyTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualCollisionDonorComponentCapPathologyPlanet();
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestTrue(
		TEXT("Component cap still triggers donor protection on pathological shattering"),
		Stats.CollisionRejectedByDonorProtectionCount > 0 || Stats.CollisionTrimmedByDonorProtectionCount > 0);
	TestTrue(
		TEXT("Relaxed donor cap still sees a donor delta above the allowed cap"),
		Stats.CollisionProposedDonorComponentsAfter > Stats.CollisionProposedDonorComponentsBefore + Planet.GeometricCollisionDonorMaxComponentIncrease);
	TestTrue(
		TEXT("Relaxed donor cap still reduces accepted terrane size materially"),
		Stats.CollisionAcceptedTerraneSampleCount < Stats.CollisionProposedTerraneSampleCount);
	TestTrue(
		TEXT("Relaxed donor cap still keeps applied donor delta below the rejected proposal"),
		(Stats.CollisionDonorComponentsAfter - Stats.CollisionDonorComponentsBefore) <
			(Stats.CollisionProposedDonorComponentsAfter - Stats.CollisionProposedDonorComponentsBefore));
	TestTrue(
		TEXT("Relaxed donor cap is the explicit donor reject or trim cause"),
		Stats.CollisionTrimmedByDonorComponentCapCount > 0 || Stats.CollisionRejectedByDonorComponentCapCount > 0);
	TestEqual(TEXT("Relaxed donor cap pathological case does not need the fragment floor"), Stats.CollisionTrimmedByDonorFragmentFloorCount + Stats.CollisionRejectedByDonorFragmentFloorCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionFragmentationSplitDonorVsReceiverSmokeTest,
	"Aurous.TectonicPlanet.CollisionFragmentationSplitDonorVsReceiverSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionFragmentationSplitDonorVsReceiverSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualGeometricCollisionLocalityClampPlanet();
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestEqual(TEXT("Collision fragmentation split smoke executes one collision"), Stats.CollisionCount, 1);
	TestTrue(TEXT("Collision fragmentation split smoke records proposed donor components before"), Stats.CollisionProposedDonorComponentsBefore >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke records proposed donor components after"), Stats.CollisionProposedDonorComponentsAfter >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke records proposed receiver components before"), Stats.CollisionProposedReceiverComponentsBefore >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke records proposed receiver components after"), Stats.CollisionProposedReceiverComponentsAfter >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke records actual donor components before"), Stats.CollisionDonorComponentsBefore >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke records actual donor components after"), Stats.CollisionDonorComponentsAfter >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke records actual receiver components before"), Stats.CollisionReceiverComponentsBefore >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke records actual receiver components after"), Stats.CollisionReceiverComponentsAfter >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke keeps donor new fragment count non-negative"), Stats.CollisionDonorNewFragmentCount >= 0);
	TestTrue(TEXT("Collision fragmentation split smoke keeps receiver disconnected fragment count non-negative"), Stats.CollisionReceiverDisconnectedFragmentCount >= 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionDonorRemnantConnectivityProtectionTest,
	"Aurous.TectonicPlanet.CollisionDonorRemnantConnectivityProtectionTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionDonorRemnantConnectivityProtectionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualCollisionDonorComponentCapPathologyPlanet();
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestTrue(
		TEXT("Collision donor protection trims or rejects the shattering transfer"),
		Stats.CollisionRejectedByDonorProtectionCount > 0 || Stats.CollisionTrimmedByDonorProtectionCount > 0);
	TestEqual(TEXT("Collision donor protection does not need receiver rejection here"), Stats.CollisionRejectedByReceiverProtectionCount, 0);
	TestTrue(
		TEXT("Collision donor protection sees donor fragmentation beyond the allowed increase"),
		Stats.CollisionProposedDonorComponentsAfter >
			Stats.CollisionProposedDonorComponentsBefore + Planet.GeometricCollisionDonorMaxComponentIncrease);
	TestTrue(
		TEXT("Collision donor protection keeps donor fragmentation below the rejected proposal"),
		Stats.CollisionDonorComponentsAfter < Stats.CollisionProposedDonorComponentsAfter);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionReceiverFragmentSuppressionTest,
	"Aurous.TectonicPlanet.CollisionReceiverFragmentSuppressionTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionReceiverFragmentSuppressionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualCollisionReceiverFragmentPlanet();
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		{},
		{ 3, 4, 5, 6, 7, 8 });
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestEqual(TEXT("Collision receiver protection leaves receiver-side fragmentation diagnostic-only"), Stats.CollisionRejectedByReceiverProtectionCount, 0);
	TestEqual(TEXT("Collision receiver protection does not need donor rejection here"), Stats.CollisionRejectedByDonorProtectionCount, 0);
	TestTrue(
		TEXT("Collision receiver protection still measures proposed disconnected receiver fragments"),
		Stats.CollisionProposedReceiverDisconnectedFragmentCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetCollisionFragmentationRegressionStep370StyleTest,
	"Aurous.TectonicPlanet.CollisionFragmentationRegressionStep370StyleTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetCollisionFragmentationRegressionStep370StyleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualCollisionDonorComponentCapPathologyPlanet();
	Planet.PendingGeometricCollisionEvent = MakeManualPendingGeometricCollisionEvent(
		Planet,
		0,
		1,
		{},
		{ 3, 4, 5 });
	Planet.PerformResampling(EResampleOwnershipMode::FullResolution, EResampleTriggerReason::CollisionFollowup);

	const FResamplingStats& Stats = Planet.LastResamplingStats;
	TestTrue(
		TEXT("Collision fragmentation regression sees a pathological proposed spike"),
		Stats.CollisionProposedGlobalMaxComponentsAfter > Stats.TopologyGlobalMaxComponentsBefore + 1);
	TestTrue(
		TEXT("Collision fragmentation regression triggers donor-side trim or rejection on the pathological transfer"),
		Stats.CollisionRejectedByDonorProtectionCount > 0 || Stats.CollisionTrimmedByDonorProtectionCount > 0);
	TestTrue(
		TEXT("Collision fragmentation regression leaves donor fragmentation smaller than the rejected proposal"),
		(Stats.CollisionDonorComponentsAfter - Stats.CollisionDonorComponentsBefore) <
			(Stats.CollisionProposedDonorComponentsAfter - Stats.CollisionProposedDonorComponentsBefore));
	TestTrue(
		TEXT("Collision fragmentation regression is still blocked by the donor component cap"),
		Stats.CollisionTrimmedByDonorComponentCapCount > 0 || Stats.CollisionRejectedByDonorComponentCapCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftStrayFragmentZeroAdjacencySiblingFallbackTest,
	"Aurous.TectonicPlanet.RiftStrayFragmentZeroAdjacencySiblingFallbackTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftStrayFragmentZeroAdjacencySiblingFallbackTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftZeroAdjacencySiblingFallbackPlanet();
	const TArray<int32> PreviousPlateAssignments = { 10, 11, 11, 20 };
	TArray<int32> NewPlateIds = PreviousPlateAssignments;
	FResamplingStats Stats;

	Planet.ApplyRiftChildCoherenceProtection(NewPlateIds, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &Stats);

	TestEqual(TEXT("Zero-adjacency sibling fallback keeps the stray fragment inside the rift family"), NewPlateIds[1], 10);
	TestEqual(TEXT("Zero-adjacency sibling fallback leaves the non-child plate untouched"), NewPlateIds[3], 20);
	TestEqual(TEXT("Zero-adjacency sibling fallback records one detected stray fragment"), Stats.RiftChildStrayFragmentDetectedCount, 1);
	TestEqual(TEXT("Zero-adjacency sibling fallback records one reassigned stray fragment"), Stats.RiftChildStrayFragmentReassignedCount, 1);
	TestEqual(TEXT("Zero-adjacency sibling fallback counts sibling reassignment"), Stats.RiftStrayChildFragmentReassignedToSiblingCount, 1);
	TestEqual(TEXT("Zero-adjacency sibling fallback avoids other-neighbor reassignment"), Stats.RiftStrayChildFragmentReassignedToOtherNeighborCount, 0);
	TestEqual(TEXT("Zero-adjacency sibling fallback records the dedicated fallback path"), Stats.RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount, 1);
	TestEqual(TEXT("Zero-adjacency sibling fallback does not count an adjacent sibling choice"), Stats.RiftStrayFragmentReassignedByAdjacentSiblingCount, 0);
	TestEqual(TEXT("Zero-adjacency sibling fallback avoids forced non-child assignment"), Stats.RiftStrayFragmentForcedNonChildAssignmentCount, 0);
	TestEqual(TEXT("Zero-adjacency sibling fallback records zero sibling adjacency"), Stats.RiftStrayFragmentZeroSiblingAdjacencyCount, 1);
	TestEqual(TEXT("Zero-adjacency sibling fallback records no positive sibling adjacency"), Stats.RiftStrayFragmentPositiveSiblingAdjacencyCount, 0);
	TestTrue(TEXT("Zero-adjacency sibling fallback records sibling edge counts"), Stats.RiftStrayFragmentSiblingEdgeCounts.Num() == 1);
	if (Stats.RiftStrayFragmentSiblingEdgeCounts.Num() == 1)
	{
		TestEqual(TEXT("Zero-adjacency sibling fallback records zero sibling edges"), Stats.RiftStrayFragmentSiblingEdgeCounts[0], 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftStrayFragmentNonChildFallbackOnlyWhenSiblingUnavailableTest,
	"Aurous.TectonicPlanet.RiftStrayFragmentNonChildFallbackOnlyWhenSiblingUnavailableTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftStrayFragmentNonChildFallbackOnlyWhenSiblingUnavailableTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftZeroAdjacencySiblingFallbackPlanet();
	Planet.PendingRiftEvent.ChildCount = 1;
	Planet.PendingRiftEvent.ChildPlateB = INDEX_NONE;
	Planet.PendingRiftEvent.ChildPlateIds = { 11 };
	Planet.PendingRiftEvent.ChildAnchorSampleIndices = { 2 };
	const TArray<int32> PreviousPlateAssignments = { 10, 11, 11, 20 };
	TArray<int32> NewPlateIds = PreviousPlateAssignments;
	FResamplingStats Stats;

	Planet.ApplyRiftChildCoherenceProtection(NewPlateIds, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &Stats);

	TestEqual(TEXT("Non-child fallback only when sibling unavailable reassigns to the neighboring non-child plate"), NewPlateIds[1], 20);
	TestEqual(TEXT("Non-child fallback only when sibling unavailable records other-neighbor reassignment"), Stats.RiftStrayChildFragmentReassignedToOtherNeighborCount, 1);
	TestEqual(TEXT("Non-child fallback only when sibling unavailable records no sibling reassignment"), Stats.RiftStrayChildFragmentReassignedToSiblingCount, 0);
	TestEqual(TEXT("Non-child fallback only when sibling unavailable records one forced non-child assignment"), Stats.RiftStrayFragmentForcedNonChildAssignmentCount, 1);
	TestEqual(TEXT("Non-child fallback only when sibling unavailable records no zero-adjacency sibling fallback"), Stats.RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftFollowupLocalizationRestoresNonChildSamplesTest,
	"Aurous.TectonicPlanet.RiftFollowupLocalizationRestoresNonChildSamplesTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftFollowupLocalizationRestoresNonChildSamplesTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftLocalizationDiagnosticPlanet();
	const TArray<int32> PreviousPlateAssignments = { 3, 10, 11, 6, 7 };
	const TArray<float> PreviousContinentalWeights = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f };
	TArray<int32> NewPlateIds = { 10, 10, 11, 11, 11 };
	TArray<uint8> GapFlags = { 1, 0, 0, 1, 0 };
	TArray<uint8> OverlapFlags = { 1, 0, 0, 1, 1 };
	TArray<TArray<int32>> OverlapPlateIds;
	OverlapPlateIds.SetNum(5);
	OverlapPlateIds[0] = { 10 };
	OverlapPlateIds[3] = { 11 };
	OverlapPlateIds[4] = { 11 };
	TArray<float> SubductionDistances = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
	TArray<float> SubductionSpeeds = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	TArray<EGapResolutionPath> GapResolutionPaths;
	GapResolutionPaths.Init(EGapResolutionPath::None, 5);
	GapResolutionPaths[0] = EGapResolutionPath::NonDivergentProjection;
	GapResolutionPaths[3] = EGapResolutionPath::NonDivergentProjection;
	FResamplingStats Stats;

		Planet.ApplyRiftFollowupLocalizationOverride(
			NewPlateIds,
			PreviousPlateAssignments,
			PreviousContinentalWeights,
			&GapFlags,
			&OverlapFlags,
			&OverlapPlateIds,
			&SubductionDistances,
			&SubductionSpeeds,
			&GapResolutionPaths,
			nullptr,
			&Stats);

	TestEqual(TEXT("Rift localization restores bystander sample 0"), NewPlateIds[0], 3);
	TestEqual(TEXT("Rift localization keeps child A sample"), NewPlateIds[1], 10);
	TestEqual(TEXT("Rift localization keeps child B sample"), NewPlateIds[2], 11);
	TestEqual(TEXT("Rift localization restores bystander sample 3"), NewPlateIds[3], 6);
	TestEqual(TEXT("Rift localization restores bystander sample 4"), NewPlateIds[4], 7);
	TestEqual(TEXT("Rift localization counts restored non-child samples"), Stats.RiftLocalizedNonChildSampleRestoreCount, 3);
	TestEqual(TEXT("Rift localization counts restored gap samples"), Stats.RiftLocalizedGapSampleRestoreCount, 2);
	TestEqual(TEXT("Rift localization clears restored gap flag"), GapFlags[0], static_cast<uint8>(0));
	TestEqual(TEXT("Rift localization clears restored overlap flag"), OverlapFlags[3], static_cast<uint8>(0));
	TestTrue(TEXT("Rift localization clears restored overlap ids"), OverlapPlateIds[4].IsEmpty());
	TestEqual(TEXT("Rift localization resets restored gap path"), GapResolutionPaths[3], EGapResolutionPath::None);
	TestEqual(TEXT("Rift localization restores subduction distance from prior sample"), SubductionDistances[0], Planet.Samples[0].SubductionDistanceKm);
	TestEqual(TEXT("Rift localization restores subduction speed from prior carried sample"), SubductionSpeeds[3], 13.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftFollowupLocalizationRestoresPreviousCWTest,
	"Aurous.TectonicPlanet.RiftFollowupLocalizationRestoresPreviousCWTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftFollowupLocalizationRestoresPreviousCWTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftLocalizationDiagnosticPlanet();
	const TArray<int32> PreviousPlateAssignments = { 3, 10, 11, 6, 7 };
	const TArray<float> PreviousContinentalWeights = { 0.25f, 0.0f, 0.0f, 0.0f, 0.0f };
	Planet.Samples[0].ContinentalWeight = 0.85f;
	Planet.Samples[1].ContinentalWeight = 0.75f;
	TArray<int32> NewPlateIds = { 10, 10, 11, 11, 11 };
	TArray<uint8> GapFlags = { 1, 0, 0, 0, 0 };
	TArray<uint8> OverlapFlags = { 1, 0, 0, 0, 0 };
	TArray<TArray<int32>> OverlapPlateIds;
	OverlapPlateIds.SetNum(5);
	OverlapPlateIds[0] = { 10 };
	TArray<float> SubductionDistances = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
	TArray<float> SubductionSpeeds = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	TArray<EGapResolutionPath> GapResolutionPaths;
	GapResolutionPaths.Init(EGapResolutionPath::None, 5);
	GapResolutionPaths[0] = EGapResolutionPath::NonDivergentProjection;
	FResamplingStats Stats;

		Planet.ApplyRiftFollowupLocalizationOverride(
			NewPlateIds,
			PreviousPlateAssignments,
			PreviousContinentalWeights,
			&GapFlags,
			&OverlapFlags,
			&OverlapPlateIds,
			&SubductionDistances,
			&SubductionSpeeds,
			&GapResolutionPaths,
			nullptr,
			&Stats);

	TestEqual(TEXT("Rift localization CW rollback restores the bystander owner"), NewPlateIds[0], 3);
	TestTrue(TEXT("Rift localization CW rollback restores the previous continental weight"), FMath::IsNearlyEqual(Planet.Samples[0].ContinentalWeight, 0.25f, 1.0e-6f));
	TestTrue(TEXT("Rift localization CW rollback leaves child samples untouched"), FMath::IsNearlyEqual(Planet.Samples[1].ContinentalWeight, 0.75f, 1.0e-6f));
	TestEqual(TEXT("Rift localization CW rollback records one restored CW"), Stats.RiftLocalizedCWRestoredCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftFollowupLocalizationPreventsPhantomContinentalGainTest,
	"Aurous.TectonicPlanet.RiftFollowupLocalizationPreventsPhantomContinentalGainTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftFollowupLocalizationPreventsPhantomContinentalGainTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftLocalizationDiagnosticPlanet();
	const TArray<int32> PreviousPlateAssignments = { 3, 10, 11, 6, 7 };
	const TArray<float> PreviousContinentalWeights = { 0.25f, 0.0f, 0.0f, 0.0f, 0.0f };
	Planet.Samples[0].ContinentalWeight = 0.85f;
	TArray<int32> NewPlateIds = { 10, 10, 11, 11, 11 };
	TArray<uint8> GapFlags = { 1, 0, 0, 0, 0 };
	TArray<uint8> OverlapFlags = { 1, 0, 0, 0, 0 };
	TArray<TArray<int32>> OverlapPlateIds;
	OverlapPlateIds.SetNum(5);
	OverlapPlateIds[0] = { 10 };
	TArray<float> SubductionDistances = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
	TArray<float> SubductionSpeeds = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	TArray<EGapResolutionPath> GapResolutionPaths;
	GapResolutionPaths.Init(EGapResolutionPath::None, 5);
	GapResolutionPaths[0] = EGapResolutionPath::NonDivergentProjection;
	FResamplingStats Stats;

		Planet.ApplyRiftFollowupLocalizationOverride(
			NewPlateIds,
			PreviousPlateAssignments,
			PreviousContinentalWeights,
			&GapFlags,
			&OverlapFlags,
			&OverlapPlateIds,
			&SubductionDistances,
			&SubductionSpeeds,
			&GapResolutionPaths,
			nullptr,
			&Stats);

	TestTrue(TEXT("Rift localization phantom-gain rollback leaves the bystander non-continental"), Planet.Samples[0].ContinentalWeight < 0.5f);
	TestEqual(TEXT("Rift localization phantom-gain rollback records one phantom prevented sample"), Stats.RiftLocalizedCWPhantomPreventedCount, 1);
	TestEqual(TEXT("Rift localization phantom-gain rollback records one prevented continental sample"), Stats.RiftLocalizedCWContinentalPreventedCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftFollowupFinalOwnerMismatchCWReconciledTest,
	"Aurous.TectonicPlanet.RiftFollowupFinalOwnerMismatchCWReconciledTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftFollowupFinalOwnerMismatchCWReconciledTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftFinalOwnerCWDiagnosticPlanet();
	const TArray<int32> PreviousPlateAssignments = { 21, 10, 11, 20 };
	const TArray<float> PreviousContinentalWeights = { 0.15f, 0.90f, 0.20f, 0.10f };
	const TArray<int32> InterpolationPlateIds = { 10, 10, 11, 20 };
	const TArray<int32> FinalPlateIds = { 20, 10, 11, 20 };
	TArray<uint8> LocalizedRestoreFlags;
	LocalizedRestoreFlags.Init(0, Planet.Samples.Num());
	FResamplingStats Stats;

	Planet.Samples[0].ContinentalWeight = 0.85f;
	Planet.ApplyRiftFollowupFinalOwnerContinentalWeightReconciliation(
		InterpolationPlateIds,
		FinalPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		LocalizedRestoreFlags,
		&Stats);

	TestTrue(
		TEXT("Final-owner CW reconciliation replaces provisional mismatch CW with the final owner's CW"),
		FMath::IsNearlyEqual(Planet.Samples[0].ContinentalWeight, 0.10f, 1.0e-6f));
	TestEqual(TEXT("Final-owner CW reconciliation records one CW change"), Stats.RiftFinalOwnerCWReconciledCount, 1);
	TestEqual(TEXT("Final-owner CW reconciliation sees one mismatch gain before reconciliation"), Stats.RiftFinalOwnerMismatchGainCountBeforeReconciliation, 1);
	TestEqual(TEXT("Final-owner CW reconciliation clears mismatch gains after reconciliation"), Stats.RiftFinalOwnerMismatchGainCountAfterReconciliation, 0);
	TestEqual(TEXT("Final-owner CW reconciliation records one prevented mismatch continental gain"), Stats.RiftFinalOwnerMismatchContinentalPreventedCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftFollowupChildCoherenceRewriteReconcilesCWTest,
	"Aurous.TectonicPlanet.RiftFollowupChildCoherenceRewriteReconcilesCWTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftFollowupChildCoherenceRewriteReconcilesCWTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftFinalOwnerCWDiagnosticPlanet();
	const TArray<int32> PreviousPlateAssignments = { 21, 10, 11, 20 };
	const TArray<float> PreviousContinentalWeights = { 0.15f, 0.90f, 0.20f, 0.10f };
	const TArray<int32> InterpolationPlateIds = { 10, 10, 11, 20 };
	const TArray<int32> FinalPlateIds = { 11, 10, 11, 20 };
	TArray<uint8> LocalizedRestoreFlags;
	LocalizedRestoreFlags.Init(0, Planet.Samples.Num());
	FResamplingStats Stats;

	Planet.Samples[0].ContinentalWeight = 0.85f;
	Planet.ApplyRiftFollowupFinalOwnerContinentalWeightReconciliation(
		InterpolationPlateIds,
		FinalPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		LocalizedRestoreFlags,
		&Stats);

	TestTrue(
		TEXT("Child-coherence CW reconciliation replaces provisional child-side CW with the surviving child owner's CW"),
		FMath::IsNearlyEqual(Planet.Samples[0].ContinentalWeight, 0.20f, 1.0e-6f));
	TestEqual(TEXT("Child-coherence CW reconciliation records one CW change"), Stats.RiftFinalOwnerCWReconciledCount, 1);
	TestEqual(TEXT("Child-coherence CW reconciliation counts the mismatch gain before reconciliation"), Stats.RiftFinalOwnerMismatchGainCountBeforeReconciliation, 1);
	TestEqual(TEXT("Child-coherence CW reconciliation removes the mismatch gain after reconciliation"), Stats.RiftFinalOwnerMismatchGainCountAfterReconciliation, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftChildFragmentSuppressionSmokeTest,
	"Aurous.TectonicPlanet.RiftChildFragmentSuppressionSmokeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftChildFragmentSuppressionSmokeTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftChildFragmentPlanet();
	const TArray<int32> PreviousPlateAssignments = { 10, 10, 10, 10, 11, 11, 20, 21 };
	TArray<int32> NewPlateIds = PreviousPlateAssignments;
	FResamplingStats Stats;

	Planet.ApplyRiftChildCoherenceProtection(NewPlateIds, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &Stats);

	TestEqual(TEXT("Rift child suppression detects both stray fragments"), Stats.RiftChildStrayFragmentDetectedCount, 2);
	TestEqual(TEXT("Rift child suppression reassigns both stray fragments"), Stats.RiftChildStrayFragmentReassignedCount, 2);
	TestEqual(TEXT("Rift child suppression prefers sibling child reassignment"), Stats.RiftStrayChildFragmentReassignedToSiblingCount, 2);
	TestEqual(TEXT("Rift child suppression does not need other-neighbor reassignment here"), Stats.RiftStrayChildFragmentReassignedToOtherNeighborCount, 0);
	TestEqual(TEXT("Rift child suppression records adjacent sibling reassignments"), Stats.RiftStrayFragmentReassignedByAdjacentSiblingCount, 2);
	TestEqual(TEXT("Rift child suppression records no zero-adjacency sibling fallbacks"), Stats.RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount, 0);
	TestEqual(TEXT("Rift child suppression leaves child A contiguous"), CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 10), 1);
	TestEqual(TEXT("Rift child suppression leaves child B contiguous"), CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 11), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftChildAnchorComponentPreservedTest,
	"Aurous.TectonicPlanet.RiftChildAnchorComponentPreservedTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftChildAnchorComponentPreservedTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftChildFragmentPlanet();
	const TArray<int32> PreviousPlateAssignments = { 10, 10, 10, 10, 11, 11, 20, 21 };
	TArray<int32> NewPlateIds = PreviousPlateAssignments;
	NewPlateIds[0] = 20;
	FResamplingStats Stats;

	Planet.ApplyRiftChildCoherenceProtection(NewPlateIds, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &Stats);

	TestEqual(TEXT("Rift child anchor sample is restored to child A"), NewPlateIds[0], 10);
	TestEqual(TEXT("Rift child anchor-connected sample remains on child A"), NewPlateIds[1], 10);
	TestTrue(TEXT("Rift child anchor diagnostics are recorded"), Stats.RiftChildAnchorSampleIndices.Num() >= 2);
	TestEqual(TEXT("Rift child anchor diagnostic matches child A anchor"), Stats.RiftChildAnchorSampleIndices[0], 0);
	TestEqual(TEXT("Rift child anchor component remains contiguous"), CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 10), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftChildFragmentReassignmentDeterministicTest,
	"Aurous.TectonicPlanet.RiftChildFragmentReassignmentDeterministicTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftChildFragmentReassignmentDeterministicTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet PlanetA = CreateManualRiftChildFragmentPlanet();
	FTectonicPlanet PlanetB = CreateManualRiftChildFragmentPlanet();
	const TArray<int32> PreviousPlateAssignments = { 10, 10, 10, 10, 11, 11, 20, 21 };
	TArray<int32> NewPlateIdsA = PreviousPlateAssignments;
	TArray<int32> NewPlateIdsB = PreviousPlateAssignments;
	FResamplingStats StatsA;
	FResamplingStats StatsB;

	PlanetA.ApplyRiftChildCoherenceProtection(NewPlateIdsA, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &StatsA);
	PlanetB.ApplyRiftChildCoherenceProtection(NewPlateIdsB, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &StatsB);

	TestTrue(TEXT("Rift child fragment reassignment is deterministic"), NewPlateIdsA == NewPlateIdsB);
	TestEqual(TEXT("Rift child fragment sample 2 reassigns to sibling child"), NewPlateIdsA[2], 11);
	TestEqual(TEXT("Rift child fragment sample 3 reassigns to sibling child"), NewPlateIdsA[3], 11);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftTopologyRegressionStep38StyleTest,
	"Aurous.TectonicPlanet.RiftTopologyRegressionStep38StyleTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftTopologyRegressionStep38StyleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftStep38StylePlanet();
	const TArray<int32> PreviousPlateAssignments = { 10, 11, 11, 11, 11, 11, 20, 21, 22, 23 };
	TArray<int32> NewPlateIds = PreviousPlateAssignments;
	FResamplingStats Stats;

	Planet.ApplyRiftChildCoherenceProtection(NewPlateIds, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &Stats);

	TestEqual(TEXT("Step-38-style regression detects four stray fragments"), Stats.RiftChildStrayFragmentDetectedCount, 4);
	TestEqual(TEXT("Step-38-style regression reassigns all stray fragments"), Stats.RiftChildStrayFragmentReassignedCount, 4);
	TestEqual(TEXT("Step-38-style regression keeps all stray fragments inside the rift family"), Stats.RiftStrayChildFragmentReassignedToSiblingCount, 4);
	TestEqual(TEXT("Step-38-style regression avoids other-neighbor reassignment"), Stats.RiftStrayChildFragmentReassignedToOtherNeighborCount, 0);
	TestEqual(TEXT("Step-38-style regression uses zero-adjacency sibling fallback for all fragments"), Stats.RiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount, 4);
	TestEqual(TEXT("Step-38-style regression uses no forced non-child assignment"), Stats.RiftStrayFragmentForcedNonChildAssignmentCount, 0);
	bool bAllRecipientsStayedInFamily = true;
	for (const int32 PlateId : Stats.RiftStrayFragmentRecipientPlateIds)
	{
		if (PlateId != 10)
		{
			bAllRecipientsStayedInFamily = false;
			break;
		}
	}
	TestTrue(TEXT("Step-38-style regression records only sibling recipients"), bAllRecipientsStayedInFamily);
	TestEqual(TEXT("Step-38-style regression leaves non-child neighbor 20 untouched"), NewPlateIds[6], 20);
	TestEqual(TEXT("Step-38-style regression leaves non-child neighbor 21 untouched"), NewPlateIds[7], 21);
	TestEqual(TEXT("Step-38-style regression leaves non-child neighbor 22 untouched"), NewPlateIds[8], 22);
	TestEqual(TEXT("Step-38-style regression leaves non-child neighbor 23 untouched"), NewPlateIds[9], 23);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftTopologyRegressionStep332StyleTest,
	"Aurous.TectonicPlanet.RiftTopologyRegressionStep332StyleTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftTopologyRegressionStep332StyleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftFragmentRegressionPlanet();
	const TArray<int32> PreviousPlateAssignments = { 10, 10, 10, 10, 10, 11, 11, 11, 11, 19, 20, 21 };
	TArray<int32> NewPlateIds = PreviousPlateAssignments;
	const int32 ChildABefore = CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 10);
	const int32 ChildBBefore = CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 11);
	const int32 TotalBefore = ChildABefore + ChildBBefore;
	FResamplingStats Stats;

	Planet.ApplyRiftChildCoherenceProtection(NewPlateIds, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &Stats);

	const int32 ChildAAfter = CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 10);
	const int32 ChildBAfter = CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 11);
	const int32 TotalAfter = ChildAAfter + ChildBAfter;

	TestTrue(TEXT("Rift regression starts with highly fragmented child A"), ChildABefore >= 4);
	TestTrue(TEXT("Rift regression starts with highly fragmented child B"), ChildBBefore >= 3);
	TestEqual(TEXT("Rift regression collapses child A to one primary component"), ChildAAfter, 1);
	TestEqual(TEXT("Rift regression collapses child B to one primary component"), ChildBAfter, 1);
	TestTrue(TEXT("Rift regression reduces total child component count"), TotalAfter < TotalBefore);
	TestTrue(TEXT("Rift regression reassigns multiple stray fragments"), Stats.RiftChildStrayFragmentReassignedCount >= 4);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftTopologyRegressionStep332StillProtectedTest,
	"Aurous.TectonicPlanet.RiftTopologyRegressionStep332StillProtectedTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftTopologyRegressionStep332StillProtectedTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftFragmentRegressionPlanet();
	const TArray<int32> PreviousPlateAssignments = { 10, 10, 10, 10, 10, 11, 11, 11, 11, 19, 20, 21 };
	TArray<int32> NewPlateIds = PreviousPlateAssignments;
	FResamplingStats Stats;

	Planet.ApplyRiftChildCoherenceProtection(NewPlateIds, PreviousPlateAssignments, nullptr, nullptr, nullptr, nullptr, &Stats);

	TestEqual(TEXT("Step-332 protection keeps child A contiguous"), CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 10), 1);
	TestEqual(TEXT("Step-332 protection keeps child B contiguous"), CountPlateComponentsForAssignmentsForTest(Planet, NewPlateIds, 11), 1);
	TestTrue(TEXT("Step-332 protection still reassigns multiple stray fragments"), Stats.RiftChildStrayFragmentReassignedCount >= 4);
	TestTrue(TEXT("Step-332 protection keeps all stray fragments inside the rift family"), Stats.RiftStrayChildFragmentReassignedToOtherNeighborCount == 0);
	return true;
}

bool FTectonicPlanetM6BaselinePresetMatchesHarnessAndActorConfigTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanetRuntimeConfig ExpectedRuntimeConfig = GetM6BaselineRuntimeConfig();

	FTectonicPlanet HarnessPlanet = CreateInitializedPlanet();
	ApplyTectonicPlanetRuntimeConfig(HarnessPlanet, ExpectedRuntimeConfig);
	TestEqualRuntimeConfig(*this, TEXT("Harness M6 baseline preset"), CaptureTectonicPlanetRuntimeConfig(HarnessPlanet), ExpectedRuntimeConfig);

	TStrongObjectPtr<ATectonicPlanetActor> Actor(NewObject<ATectonicPlanetActor>(GetTransientPackage()));
	TestNotNull(TEXT("Actor for runtime parity exists"), Actor.Get());
	if (!Actor.IsValid())
	{
		return false;
	}

	Actor->GeneratePlanet(TestSampleCount, TestPlateCount, TestRandomSeed, TestBoundaryWarpAmplitude, TestContinentalFraction);
	TestTrue(TEXT("Actor path enables the M6 baseline runtime preset"), Actor->GetUseM6BaselineRuntimePreset());
	TestEqual(TEXT("Actor exposes the M6 baseline preset label"), Actor->GetActiveRuntimePresetLabel(), FString(TEXT("M6Baseline")));
	TestEqualRuntimeConfig(*this, TEXT("Actor M6 baseline preset"), CaptureTectonicPlanetRuntimeConfig(Actor->GetPlanet()), ExpectedRuntimeConfig);
	return true;
}

bool FTectonicPlanetM6BaselinePresetUsesPreserveOwnershipPeriodicTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanetRuntimeConfig RuntimeConfig = GetM6BaselineRuntimeConfig();
	TestEqual(TEXT("M6 preset uses preserve-ownership periodic resampling"), RuntimeConfig.ResamplingPolicy, EResamplingPolicy::PreserveOwnershipPeriodic);
	TestTrue(TEXT("M6 preset keeps automatic rifting enabled"), RuntimeConfig.bEnableAutomaticRifting);
	TestEqual(TEXT("M6 preset uses the baseline automatic rift base rate"), RuntimeConfig.AutomaticRiftBaseRatePerMy, 0.18);
	TestEqual(TEXT("M6 preset uses the baseline minimum parent sample threshold"), RuntimeConfig.AutomaticRiftMinParentSamples, 1500);
	TestEqual(TEXT("M6 preset uses the baseline minimum continental sample threshold"), RuntimeConfig.AutomaticRiftMinContinentalSamples, 64);
	TestEqual(TEXT("M6 preset uses the baseline minimum continental fraction threshold"), RuntimeConfig.AutomaticRiftMinContinentalFraction, 0.05);
	TestEqual(TEXT("M6 preset uses the baseline rift boundary warp amplitude"), RuntimeConfig.RiftBoundaryWarpAmplitude, 0.25);
	TestEqual(TEXT("M6 preset uses the baseline rift boundary warp frequency"), RuntimeConfig.RiftBoundaryWarpFrequency, 1.5);

	FTectonicPlanet Planet = CreateInitializedPlanet();
	ApplyTectonicPlanetRuntimeConfig(Planet, RuntimeConfig);
	TestEqualRuntimeConfig(*this, TEXT("Applied M6 runtime config"), CaptureTectonicPlanetRuntimeConfig(Planet), RuntimeConfig);
	return true;
}

bool FTectonicPlanetM6BaselinePresetFirstPeriodicResampleParitySmokeTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanetRuntimeConfig RuntimeConfig = GetM6BaselineRuntimeConfig();

	FTectonicPlanet HarnessPlanet = CreateInitializedPlanet();
	ApplyTectonicPlanetRuntimeConfig(HarnessPlanet, RuntimeConfig);

	TStrongObjectPtr<ATectonicPlanetActor> Actor(NewObject<ATectonicPlanetActor>(GetTransientPackage()));
	TestNotNull(TEXT("Actor for first-resample parity exists"), Actor.Get());
	if (!Actor.IsValid())
	{
		return false;
	}

	Actor->GeneratePlanet(TestSampleCount, TestPlateCount, TestRandomSeed, TestBoundaryWarpAmplitude, TestContinentalFraction);

	const int32 HarnessResampleInterval = HarnessPlanet.ComputeResampleInterval();
	TestEqual(TEXT("Harness M6 preset resample interval is 10 at 60k"), HarnessResampleInterval, 10);
	TestEqual(TEXT("Actor M6 preset resample interval matches harness"), Actor->GetPlanet().ComputeResampleInterval(), HarnessResampleInterval);

	while (HarnessPlanet.CurrentStep < HarnessResampleInterval)
	{
		HarnessPlanet.AdvanceStep();
	}
	Actor->AdvancePlanetSteps(HarnessResampleInterval);

	const FM6CheckpointSnapshot HarnessSnapshot = BuildM6CheckpointSnapshot(HarnessPlanet);
	const FM6CheckpointSnapshot ActorSnapshot = BuildM6CheckpointSnapshot(Actor->GetPlanet());
	TestEqual(TEXT("Harness first periodic resample uses preserve ownership"), HarnessSnapshot.LastOwnershipMode, EResampleOwnershipMode::PreserveOwnership);
	TestEqual(TEXT("Actor first periodic resample uses preserve ownership"), ActorSnapshot.LastOwnershipMode, EResampleOwnershipMode::PreserveOwnership);
	TestEqual(TEXT("Actor and harness first periodic resample land on the same step"), ActorSnapshot.Step, HarnessSnapshot.Step);
	TestEqual(TEXT("Actor and harness first periodic resample have identical boundary sample counts"), ActorSnapshot.BoundarySampleCount, HarnessSnapshot.BoundarySampleCount);
	TestEqual(TEXT("Actor and harness first periodic resample have identical gap counts"), ActorSnapshot.GapCount, HarnessSnapshot.GapCount);
	TestEqual(TEXT("Actor and harness first periodic resample have identical overlap counts"), ActorSnapshot.OverlapCount, HarnessSnapshot.OverlapCount);
	TestEqual(TEXT("Actor and harness first periodic resample have identical continental counts"), ActorSnapshot.ContinentalCount, HarnessSnapshot.ContinentalCount);
	return true;
}

bool FTectonicPlanetArchitectureSpikeAPresetUsesAuthoritativePeriodicTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanetRuntimeConfig RuntimeConfig = GetArchitectureSpikeARuntimeConfig();
	const FTectonicPlanetRuntimeConfig M6BaselineRuntimeConfig = GetM6BaselineRuntimeConfig();

	TestEqual(
		TEXT("Architecture Spike A uses the dedicated authoritative periodic policy"),
		RuntimeConfig.ResamplingPolicy,
		EResamplingPolicy::PeriodicGlobalAuthoritativeSpike);
	TestTrue(TEXT("Architecture Spike A keeps automatic rifting enabled"), RuntimeConfig.bEnableAutomaticRifting);
	TestEqual(
		TEXT("Architecture Spike A keeps the M6 automatic rift base rate"),
		RuntimeConfig.AutomaticRiftBaseRatePerMy,
		M6BaselineRuntimeConfig.AutomaticRiftBaseRatePerMy);
	TestEqual(
		TEXT("Architecture Spike A keeps the M6 Andean conversion rate"),
		RuntimeConfig.AndeanContinentalConversionRatePerMy,
		M6BaselineRuntimeConfig.AndeanContinentalConversionRatePerMy);

	FTectonicPlanet Planet = CreateInitializedPlanet();
	ApplyTectonicPlanetRuntimeConfig(Planet, RuntimeConfig);
	TestEqualRuntimeConfig(
		*this,
		TEXT("Applied Architecture Spike A runtime config"),
		CaptureTectonicPlanetRuntimeConfig(Planet),
		RuntimeConfig);
	return true;
}

bool FTectonicPlanetArchitectureSpikeAFirstPeriodicResampleUsesFullResolutionSmokeTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanetRuntimeConfig RuntimeConfig = GetArchitectureSpikeARuntimeConfig();

	FTectonicPlanet Planet = CreateInitializedPlanet();
	ApplyTectonicPlanetRuntimeConfig(Planet, RuntimeConfig);

	const int32 ResampleInterval = Planet.ComputeResampleInterval();
	TestEqual(TEXT("Architecture Spike A resample interval is 10 at 60k"), ResampleInterval, 10);

	while (Planet.CurrentStep < ResampleInterval)
	{
		Planet.AdvanceStep();
	}

	const FM6CheckpointSnapshot Snapshot = BuildM6CheckpointSnapshot(Planet);
	TestEqual(TEXT("Architecture Spike A first periodic resample uses FullResolution"), Snapshot.LastOwnershipMode, EResampleOwnershipMode::FullResolution);
	TestEqual(TEXT("Architecture Spike A first periodic resample is periodic"), Snapshot.LastTriggerReason, EResampleTriggerReason::Periodic);
	TestEqual(TEXT("Architecture Spike A first periodic resample does not use preserve same-plate hits"), Snapshot.PreserveSamePlateHitCount, 0);
	TestEqual(TEXT("Architecture Spike A first periodic resample does not use preserve fallback queries"), Snapshot.PreserveFallbackQueryCount, 0);
	return true;
}

bool FTectonicPlanetArchitectureSpikeA60k7Test::RunTest(const FString& Parameters)
{
	const int32 SpikeSeed = Parameters.IsEmpty() ? TestRandomSeed : FCString::Atoi(*Parameters);
	FM6BaselineRunConfig SpikeConfig;
	SpikeConfig.Seed = SpikeSeed;
	SpikeConfig.MaxStep = 400;
	SpikeConfig.PresetLabel = TEXT("ArchitectureSpikeA");
	SpikeConfig.RuntimeConfig = GetArchitectureSpikeARuntimeConfig();
	SpikeConfig.AutomaticRiftBaseRatePerMy = SpikeConfig.RuntimeConfig.AutomaticRiftBaseRatePerMy;
	SpikeConfig.AndeanContinentalConversionRatePerMy = SpikeConfig.RuntimeConfig.AndeanContinentalConversionRatePerMy;
	SpikeConfig.RunId = FString::Printf(
		TEXT("ArchitectureSpikeA60k7-seed%d-samples%d-plates%d"),
		SpikeSeed,
		TestSampleCount,
		TestPlateCount);
	SpikeConfig.ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		SpikeConfig.RunId);
	SpikeConfig.bExportCheckpoints = true;
	SpikeConfig.bLogCheckpoints = true;
	SpikeConfig.bLogSummary = true;

	const FM6BaselineRunSummary SpikeSummary = RunM6BaselineScenario(*this, SpikeConfig);
	TestTrue(TEXT("Architecture Spike A final continental area fraction remains finite"), FMath::IsFinite(SpikeSummary.FinalContinentalAreaFraction));
	TestTrue(TEXT("Architecture Spike A keeps stable plate ids valid"), SpikeSummary.bStablePlateIdsValid);
	TestTrue(TEXT("Architecture Spike A records the step-100 checkpoint"), SpikeSummary.bHasStep100Snapshot);
	TestTrue(TEXT("Architecture Spike A records the step-200 checkpoint"), SpikeSummary.bHasStep200Snapshot);
	TestTrue(TEXT("Architecture Spike A records the step-300 checkpoint"), SpikeSummary.bHasStep300Snapshot);
	TestTrue(TEXT("Architecture Spike A records the step-400 checkpoint"), SpikeSummary.bHasStep400Snapshot);
	TestEqual(TEXT("Architecture Spike A final snapshot lands at step 400"), SpikeSummary.FinalSnapshot.Step, 400);
	TestEqual(TEXT("Architecture Spike A keeps preserve CW retention inert"), SpikeSummary.TotalPreserveOwnershipCWRetained, static_cast<int64>(0));
	TestEqual(TEXT("Architecture Spike A keeps preserve fallback same-plate retention inert"), SpikeSummary.TotalPreserveOwnershipFallbackSamePlateRetainedCount, static_cast<int64>(0));
	TestEqual(TEXT("Architecture Spike A keeps preserve previous-owner hysteresis inert"), SpikeSummary.TotalPreserveOwnershipPreviousOwnerHysteresisApplicationCount, static_cast<int64>(0));
	return true;
}

bool FTectonicPlanetM6Baseline60k7Test::RunTest(const FString& Parameters)
{
	const int32 BaselineSeed = Parameters.IsEmpty() ? TestRandomSeed : FCString::Atoi(*Parameters);
	FM6BaselineRunConfig BaselineConfig;
	BaselineConfig.Seed = BaselineSeed;
	BaselineConfig.MaxStep = 400;
	BaselineConfig.RunId = FString::Printf(
		TEXT("M6Baseline60k7-seed%d-samples%d-plates%d"),
		BaselineSeed,
		TestSampleCount,
		TestPlateCount);
	BaselineConfig.ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		BaselineConfig.RunId);
	BaselineConfig.bExportCheckpoints = true;
	BaselineConfig.bLogCheckpoints = true;
	BaselineConfig.bLogSummary = true;

	const FM6BaselineRunSummary BaselineSummary = RunM6BaselineScenario(*this, BaselineConfig);
	TestEqual(TEXT("M6ad baseline keeps donor component cap frozen at 10"), BaselineSummary.GeometricCollisionDonorMaxComponentIncrease, 10);
	TestEqual(TEXT("M6ad baseline keeps donor fragment floor disabled"), BaselineSummary.GeometricCollisionDonorMinNewFragmentSampleCount, 0);
	TestTrue(TEXT("M6ad baseline remains deterministic with stable ids"), BaselineSummary.bStablePlateIdsValid);
	TestTrue(TEXT("M6ad baseline final continental area fraction remains finite"), FMath::IsFinite(BaselineSummary.FinalContinentalAreaFraction));
	TestTrue(TEXT("M6ad baseline records the step-100 checkpoint"), BaselineSummary.bHasStep100Snapshot);
	TestTrue(TEXT("M6ad baseline records the step-200 checkpoint"), BaselineSummary.bHasStep200Snapshot);
	TestTrue(TEXT("M6ad baseline records the step-300 checkpoint"), BaselineSummary.bHasStep300Snapshot);
	TestTrue(TEXT("M6ad baseline records the step-400 checkpoint"), BaselineSummary.bHasStep400Snapshot);
	TestEqual(TEXT("M6ad baseline final snapshot lands at step 400"), BaselineSummary.FinalSnapshot.Step, 400);
	TestEqual(TEXT("M6ad baseline net delta halves sum to total"), BaselineSummary.FirstHalfNetContinentalSampleDelta + BaselineSummary.SecondHalfNetContinentalSampleDelta, BaselineSummary.TotalNetContinentalSampleDelta);
	TestEqual(TEXT("M6ad baseline loss halves sum to total"), BaselineSummary.FirstHalfContinentalSamplesLost + BaselineSummary.SecondHalfContinentalSamplesLost, BaselineSummary.TotalContinentalSamplesLost);
	TestEqual(TEXT("M6ad baseline Andean gain halves sum to total"), BaselineSummary.FirstHalfAndeanContinentalGains + BaselineSummary.SecondHalfAndeanContinentalGains, BaselineSummary.TotalAndeanContinentalGains);
	TestEqual(TEXT("M6ad baseline collision gain halves sum to total"), BaselineSummary.FirstHalfCollisionContinentalGains + BaselineSummary.SecondHalfCollisionContinentalGains, BaselineSummary.TotalCollisionContinentalGains);
	TestTrue(TEXT("M6ad baseline records a peak topology spike event"), BaselineSummary.bHasPeakTopologySpikeEvent);
	if (BaselineSummary.bHasPeakTopologySpikeEvent)
	{
		TestTrue(
			TEXT("M6ad baseline peak topology followup does not exceed the run-wide observed max"),
			BaselineSummary.PeakTopologySpikeEventStats.TopologyGlobalMaxComponentsAfter <=
				BaselineSummary.MaxComponentsPerPlateObserved);
	}
	TestEqual(
		TEXT("M6ad baseline collision attribution windows sum to total"),
		BaselineSummary.CollisionFollowupComponentIncrease0To100 +
			BaselineSummary.CollisionFollowupComponentIncrease100To200 +
			BaselineSummary.CollisionFollowupComponentIncrease200To400,
		BaselineSummary.TotalCollisionFollowupComponentIncrease);
	TestEqual(
		TEXT("M6ad baseline rift attribution windows sum to total"),
		BaselineSummary.RiftFollowupComponentIncrease0To100 +
			BaselineSummary.RiftFollowupComponentIncrease100To200 +
			BaselineSummary.RiftFollowupComponentIncrease200To400,
		BaselineSummary.TotalRiftFollowupComponentIncrease);
	TestEqual(
		TEXT("M6ad baseline rift attribution counts sum to total rifts"),
		BaselineSummary.RiftFollowupCount0To100 +
			BaselineSummary.RiftFollowupCount100To200 +
			BaselineSummary.RiftFollowupCount200To400,
		BaselineSummary.TotalRiftEvents);
	TestTrue(TEXT("M6ad baseline records a worst rift spike event"), BaselineSummary.bHasLargestRiftTopologyIncreaseEvent);
	if (BaselineSummary.bHasLargestRiftTopologyIncreaseEvent)
	{
		TestEqual(
			TEXT("M6ad baseline worst rift spike is tagged as a rift followup"),
			BaselineSummary.LargestRiftTopologyIncreaseEventStats.TriggerReason,
			EResampleTriggerReason::RiftFollowup);
	}
	TestTrue(
		TEXT("M6x baseline records collision donor fragmentation totals"),
		BaselineSummary.TotalCollisionAppliedDonorComponentIncrease >= 0 &&
			BaselineSummary.TotalCollisionProposedDonorComponentIncrease >= 0);
	TestTrue(
		TEXT("M6x baseline records collision receiver fragmentation totals"),
		BaselineSummary.TotalCollisionAppliedReceiverComponentIncrease >= 0 &&
			BaselineSummary.TotalCollisionProposedReceiverComponentIncrease >= 0);
	TestTrue(
		TEXT("M6x baseline records non-negative connectivity protection counts"),
		BaselineSummary.TotalCollisionTrimmedByDonorProtectionCount >= 0 &&
			BaselineSummary.TotalCollisionRejectedByDonorProtectionCount >= 0 &&
			BaselineSummary.TotalCollisionTrimmedByReceiverProtectionCount >= 0 &&
			BaselineSummary.TotalCollisionRejectedByReceiverProtectionCount >= 0);
	TestTrue(
		TEXT("M6aa baseline records coherent donor trim diagnostics"),
		BaselineSummary.TotalCollisionTransferDiagnosticsCount >= 0 &&
			BaselineSummary.TotalCollisionAcceptedTerraneSampleCount <=
				BaselineSummary.TotalCollisionProposedTerraneSampleCount &&
			BaselineSummary.MaxCollisionAcceptedTerraneSampleCount <=
				BaselineSummary.MaxCollisionProposedTerraneSampleCount &&
			BaselineSummary.MaxCollisionTransferTrimRatio >= 0.0 &&
			BaselineSummary.MaxCollisionTransferTrimRatio <= 1.0);
	TestTrue(
		TEXT("M6aa baseline records non-negative donor trim cause totals"),
		BaselineSummary.TotalCollisionTrimmedByDonorComponentCapCount >= 0 &&
			BaselineSummary.TotalCollisionRejectedByDonorComponentCapCount >= 0 &&
			BaselineSummary.TotalCollisionTrimmedByDonorFragmentFloorCount >= 0 &&
			BaselineSummary.TotalCollisionRejectedByDonorFragmentFloorCount >= 0);
	TestTrue(
		TEXT("M6ad baseline records non-negative rift localization totals"),
		BaselineSummary.TotalRiftLocalizedNonChildSampleRestoreCount >= 0 &&
			BaselineSummary.TotalRiftLocalizedGapSampleRestoreCount >= 0 &&
			BaselineSummary.TotalRiftLocalizedCWRestoredCount >= 0 &&
			BaselineSummary.TotalRiftLocalizedCWPhantomPreventedCount >= 0 &&
			BaselineSummary.TotalRiftLocalizedCWContinentalPreventedCount >= 0 &&
			BaselineSummary.TotalRiftFinalOwnerMismatchGainCountAfterLocalization >= 0 &&
			BaselineSummary.TotalRiftFinalOwnerCWReconciledCount >= 0 &&
			BaselineSummary.TotalRiftFinalOwnerMismatchGainCountBeforeReconciliation >= 0 &&
			BaselineSummary.TotalRiftFinalOwnerMismatchGainCountAfterReconciliation >= 0 &&
			BaselineSummary.TotalRiftFinalOwnerMismatchContinentalPreventedCount >= 0 &&
			BaselineSummary.TotalRiftSameOwnerChildInterpolationGainCountBeforeReconciliation >= 0 &&
			BaselineSummary.TotalRiftSameOwnerChildInterpolationGainCountAfterReconciliation >= 0 &&
			BaselineSummary.TotalRiftFinalGainStartedBelow025Count >= 0 &&
			BaselineSummary.TotalRiftFinalGainStartedBelow040Count >= 0 &&
			BaselineSummary.TotalRiftFinalGainStartedBelow050Count >= 0);
	TestTrue(
		TEXT("M6ah baseline records non-negative preserve fallback totals"),
		BaselineSummary.TotalPreserveOwnershipFallbackSamePlateRecontainedCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackSamePlateRetainedCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackChangedOwnerCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackGapCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackDivergentOceanizationCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipContinentalLossCountAfterFallback >= 0 &&
			BaselineSummary.TotalPreserveOwnershipPreviousOwnerHysteresisApplicationCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipStronglyContinentalBoundarySavedCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackChangedOwnerNonGapLossCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackDivergentLossCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackNonDivergentProjectionLossCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackNonDivergentFallbackOceanizedLossCount >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackStrongLossGE090Count >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackStrongLossGE075Count >= 0 &&
			BaselineSummary.TotalPreserveOwnershipFallbackStrongLossGE050Count >= 0);
	if (BaselineSummary.bHasStep229RiftFollowupStats)
	{
		TestEqual(
			TEXT("M6af baseline step-229 summary records a rift followup when present"),
			BaselineSummary.Step229RiftFollowupStats.TriggerReason,
			EResampleTriggerReason::RiftFollowup);
		TestTrue(
			TEXT("M6af baseline step-229 interpolation-created gain count is non-negative"),
			BaselineSummary.Step229RiftFollowupStats.RiftInterpolationCreatedGainCount >= 0);
		TestTrue(
			TEXT("M6ag baseline step-229 reconciliation counters are non-negative"),
			BaselineSummary.Step229RiftFollowupStats.RiftFinalOwnerCWReconciledCount >= 0 &&
				BaselineSummary.Step229RiftFollowupStats.RiftFinalOwnerMismatchGainCountBeforeReconciliation >= 0 &&
				BaselineSummary.Step229RiftFollowupStats.RiftFinalOwnerMismatchGainCountAfterReconciliation >= 0 &&
				BaselineSummary.Step229RiftFollowupStats.RiftSameOwnerChildInterpolationGainCountBeforeReconciliation >= 0 &&
				BaselineSummary.Step229RiftFollowupStats.RiftSameOwnerChildInterpolationGainCountAfterReconciliation >= 0);
	}
	else
	{
		TestEqual(
			TEXT("M6af baseline step-229 summary reports zero gross gains when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.AndeanContinentalGainCount,
			0);
		TestEqual(
			TEXT("M6af baseline step-229 summary reports zero gross losses when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.ContinentalSamplesLost,
			0);
		TestEqual(
			TEXT("M6af baseline step-229 summary reports zero interpolation-created gains when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.RiftInterpolationCreatedGainCount,
			0);
		TestEqual(
			TEXT("M6af baseline step-229 summary reports zero localized CW restores when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.RiftLocalizedCWRestoredCount,
			0);
		TestEqual(
			TEXT("M6af baseline step-229 summary reports zero owner-mismatch gains when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.RiftFinalOwnerMismatchGainCountAfterLocalization,
			0);
		TestEqual(
			TEXT("M6ag baseline step-229 summary reports zero final-owner CW reconciliations when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.RiftFinalOwnerCWReconciledCount,
			0);
		TestEqual(
			TEXT("M6ag baseline step-229 summary reports zero mismatch gains before reconciliation when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.RiftFinalOwnerMismatchGainCountBeforeReconciliation,
			0);
		TestEqual(
			TEXT("M6ag baseline step-229 summary reports zero mismatch gains after reconciliation when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.RiftFinalOwnerMismatchGainCountAfterReconciliation,
			0);
		TestEqual(
			TEXT("M6ag baseline step-229 summary reports zero same-owner child gains when no rift followup is present"),
			BaselineSummary.Step229RiftFollowupStats.RiftSameOwnerChildInterpolationGainCountAfterReconciliation,
			0);
	}
	TestTrue(
		TEXT("M6ad baseline records coherent rift child fragment suppression totals"),
		BaselineSummary.TotalRiftChildStrayFragmentDetectedCount >= 0 &&
			BaselineSummary.TotalRiftChildStrayFragmentReassignedCount >= 0 &&
			BaselineSummary.MaxRiftLargestStrayChildFragmentSize >= 0 &&
			BaselineSummary.TotalRiftChildStrayFragmentReassignedCount <=
				BaselineSummary.TotalRiftChildStrayFragmentDetectedCount);
	TestTrue(
		TEXT("M6ad baseline records non-negative rift child fragment destination totals"),
		BaselineSummary.TotalRiftStrayFragmentReassignedToSiblingCount >= 0 &&
			BaselineSummary.TotalRiftStrayFragmentReassignedToOtherNeighborCount >= 0);
	TestEqual(
		TEXT("M6ad baseline keeps rift stray fragment destination buckets coherent"),
		static_cast<int64>(BaselineSummary.TotalRiftStrayFragmentReassignedByAdjacentSiblingCount) +
			static_cast<int64>(BaselineSummary.TotalRiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount) +
			static_cast<int64>(BaselineSummary.TotalRiftStrayFragmentForcedNonChildAssignmentCount),
		BaselineSummary.TotalRiftChildStrayFragmentReassignedCount);
	TestEqual(
		TEXT("M6ad baseline keeps sibling-adjacency buckets coherent"),
		static_cast<int64>(BaselineSummary.TotalRiftStrayFragmentZeroSiblingAdjacencyCount) +
			static_cast<int64>(BaselineSummary.TotalRiftStrayFragmentPositiveSiblingAdjacencyCount),
		BaselineSummary.TotalRiftChildStrayFragmentReassignedCount);
	TestEqual(
		TEXT("M6ad baseline sibling destination totals match the detailed split"),
		BaselineSummary.TotalRiftStrayFragmentReassignedToSiblingCount,
		BaselineSummary.TotalRiftStrayFragmentReassignedByAdjacentSiblingCount +
			BaselineSummary.TotalRiftStrayFragmentReassignedByZeroAdjacencySiblingFallbackCount);
	TestEqual(
		TEXT("M6ad baseline non-child destination totals match the detailed split"),
		BaselineSummary.TotalRiftStrayFragmentReassignedToOtherNeighborCount,
		BaselineSummary.TotalRiftStrayFragmentForcedNonChildAssignmentCount);
	if (BaselineSummary.bHasPeakTopologySpikeEvent)
	{
		TestTrue(
			TEXT("M6ad baseline peak topology event records donor diagnostics"),
			BaselineSummary.PeakTopologySpikeEventStats.CollisionDonorComponentsBefore >= 0 &&
				BaselineSummary.PeakTopologySpikeEventStats.CollisionDonorComponentsAfter >= 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetM6OwnershipProvenanceAuditTest,
	"Aurous.TectonicPlanet.M6OwnershipProvenanceAuditStep30And229",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetM6OwnershipProvenanceAuditTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BasePlanet = CreateM6BaselinePlanetForAudit();
	while (BasePlanet.CurrentStep < (M6OwnershipAuditStep30 - 1))
	{
		BasePlanet.AdvanceStep();
	}
	TestEqual(TEXT("Ownership provenance audit reaches step 29 before the preserve pass"), BasePlanet.CurrentStep, M6OwnershipAuditStep30 - 1);

	const FTectonicPlanet Step29Planet = BasePlanet;
	const FTectonicPlanet Step30PrePeriodicPlanet = AdvanceOneStepWithoutResamplingForAudit(Step29Planet);
	const FTectonicPlanet Step30PeriodicActualPlanet = AdvanceOneStepAllowSingleResampleForAudit(Step29Planet);
	TestEqual(TEXT("Ownership provenance audit isolates the step-30 pre-periodic state"), Step30PrePeriodicPlanet.CurrentStep, M6OwnershipAuditStep30);
	TestEqual(TEXT("Ownership provenance audit preserves the real step-30 periodic trigger reason"), Step30PeriodicActualPlanet.LastResampleTriggerReason, EResampleTriggerReason::Periodic);
	TestEqual(TEXT("Ownership provenance audit preserves the real step-30 periodic ownership mode"), Step30PeriodicActualPlanet.LastResampleOwnershipMode, EResampleOwnershipMode::PreserveOwnership);

	const FManualResampleAuditExecutionResult Step30Replay =
		ReplayResampleForAudit(
			Step30PrePeriodicPlanet,
			EResampleOwnershipMode::PreserveOwnership,
			EResampleTriggerReason::Periodic);
	CompareSamplePlateAndCWStateForAudit(
		*this,
		TEXT("step30_periodic"),
		Step30PeriodicActualPlanet,
		Step30Replay.FinalPlanet);

	TestEqual(TEXT("Ownership provenance audit matches the logged step-30 loss count"), Step30PeriodicActualPlanet.LastResamplingStats.ContinentalSamplesLost, 439);
	TestEqual(TEXT("Ownership provenance audit matches the logged step-30 changed-owner non-gap loss count"), Step30PeriodicActualPlanet.LastResamplingStats.FormerContinentalChangedPlateNonContinentalFinalCount, 214);
	TestEqual(TEXT("Ownership provenance audit matches the logged step-30 fallback-query non-continental loss count"), Step30PeriodicActualPlanet.LastResamplingStats.FormerContinentalFallbackQueryNonContinentalFinalCount, 214);

	int32 Step30LossTotal = 0;
	int32 Step30GainTotal = 0;
	int32 Step30NoGapChangedOwnerLossCount = 0;
	int32 Step30NoGapSameOwnerLossCount = 0;
	int32 Step30GapFlagNoResolutionChangedOwnerLossCount = 0;
	int32 Step30GapFlagNoResolutionSameOwnerLossCount = 0;
	int32 Step30ProjectionChangedOwnerLossCount = 0;
	int32 Step30ProjectionSameOwnerLossCount = 0;
	int32 Step30NearestCopyChangedOwnerLossCount = 0;
	int32 Step30NearestCopySameOwnerLossCount = 0;
	int32 Step30NonDivergentFallbackOceanizedLossCount = 0;
	int32 Step30DivergentOceanizedLossCount = 0;
	int32 Step30FinalChangedOwnerLossCount = 0;
	int32 Step30QueryChangedOwnerLossCount = 0;
	int32 Step30FallbackQueryLossCount = 0;
	int32 Step30FallbackQueryChangedOwnerLossCount = 0;
	int32 Step30FallbackQuerySameOwnerLossCount = 0;
	int32 Step30PreserveRetainFlagLossCount = 0;
	int32 Step30OldCWGe090LossCount = 0;
	int32 Step30OldCWGe075Lt090LossCount = 0;
	int32 Step30OldCWGe050Lt075LossCount = 0;
	int32 Step30FinalCWZeroLossCount = 0;
	int32 Step30FinalCWGtZeroLt025LossCount = 0;
	int32 Step30FinalCWGe025Lt050LossCount = 0;
	TMap<FString, int32> Step30FinalOwnerPairs;
	TMap<FString, int32> Step30QueryOwnerPairs;
	for (int32 SampleIndex = 0; SampleIndex < Step30Replay.FinalPlanet.Samples.Num(); ++SampleIndex)
	{
		const float PreviousContinentalWeight = Step30Replay.PreviousContinentalWeights[SampleIndex];
		const float FinalContinentalWeight = Step30Replay.FinalPlanet.Samples[SampleIndex].ContinentalWeight;
		const bool bWasContinental = PreviousContinentalWeight >= 0.5f;
		const bool bIsContinental = FinalContinentalWeight >= 0.5f;
		Step30GainTotal += (!bWasContinental && bIsContinental) ? 1 : 0;
		if (!bWasContinental || bIsContinental)
		{
			continue;
		}

		++Step30LossTotal;
		const int32 PreviousPlateId = Step30Replay.PreviousPlateAssignments[SampleIndex];
		const int32 QueryPlateId =
			Step30Replay.NewPlateIdsAfterQuery.IsValidIndex(SampleIndex)
				? Step30Replay.NewPlateIdsAfterQuery[SampleIndex]
				: INDEX_NONE;
		const int32 FinalPlateId = Step30Replay.FinalPlanet.Samples[SampleIndex].PlateId;
		const bool bGapFlag = Step30Replay.GapFlags.IsValidIndex(SampleIndex) && Step30Replay.GapFlags[SampleIndex] != 0;
		const bool bFinalOwnerChanged = FinalPlateId != PreviousPlateId;
		const bool bQueryOwnerChanged = QueryPlateId != PreviousPlateId;
		const bool bFallbackQuery =
			Step30Replay.PreserveOwnershipFallbackQueryFlags.IsValidIndex(SampleIndex) &&
			Step30Replay.PreserveOwnershipFallbackQueryFlags[SampleIndex] != 0;
		const bool bRetainFlag =
			Step30Replay.PreserveOwnershipCWRetainFlags.IsValidIndex(SampleIndex) &&
			Step30Replay.PreserveOwnershipCWRetainFlags[SampleIndex] != 0;
		const EGapResolutionPath GapResolutionPath =
			Step30Replay.GapResolutionPaths.IsValidIndex(SampleIndex)
				? Step30Replay.GapResolutionPaths[SampleIndex]
				: EGapResolutionPath::None;

		Step30FinalChangedOwnerLossCount += bFinalOwnerChanged ? 1 : 0;
		Step30QueryChangedOwnerLossCount += bQueryOwnerChanged ? 1 : 0;
		Step30FallbackQueryLossCount += bFallbackQuery ? 1 : 0;
		Step30FallbackQueryChangedOwnerLossCount += (bFallbackQuery && bFinalOwnerChanged) ? 1 : 0;
		Step30FallbackQuerySameOwnerLossCount += (bFallbackQuery && !bFinalOwnerChanged) ? 1 : 0;
		Step30PreserveRetainFlagLossCount += bRetainFlag ? 1 : 0;
		AddOwnerPairCountForAudit(Step30FinalOwnerPairs, PreviousPlateId, FinalPlateId);
		AddOwnerPairCountForAudit(Step30QueryOwnerPairs, PreviousPlateId, QueryPlateId);

		if (PreviousContinentalWeight >= 0.90f)
		{
			++Step30OldCWGe090LossCount;
		}
		else if (PreviousContinentalWeight >= 0.75f)
		{
			++Step30OldCWGe075Lt090LossCount;
		}
		else
		{
			++Step30OldCWGe050Lt075LossCount;
		}

		if (FinalContinentalWeight <= KINDA_SMALL_NUMBER)
		{
			++Step30FinalCWZeroLossCount;
		}
		else if (FinalContinentalWeight < 0.25f)
		{
			++Step30FinalCWGtZeroLt025LossCount;
		}
		else
		{
			++Step30FinalCWGe025Lt050LossCount;
		}

		switch (GapResolutionPath)
		{
		case EGapResolutionPath::NonDivergentProjection:
			if (bFinalOwnerChanged)
			{
				++Step30ProjectionChangedOwnerLossCount;
			}
			else
			{
				++Step30ProjectionSameOwnerLossCount;
			}
			break;
		case EGapResolutionPath::NonDivergentNearestCopy:
			if (bFinalOwnerChanged)
			{
				++Step30NearestCopyChangedOwnerLossCount;
			}
			else
			{
				++Step30NearestCopySameOwnerLossCount;
			}
			break;
		case EGapResolutionPath::NonDivergentFallbackOceanized:
			++Step30NonDivergentFallbackOceanizedLossCount;
			break;
		case EGapResolutionPath::DivergentOceanized:
			++Step30DivergentOceanizedLossCount;
			break;
		case EGapResolutionPath::None:
		default:
			if (bGapFlag)
			{
				if (bFinalOwnerChanged)
				{
					++Step30GapFlagNoResolutionChangedOwnerLossCount;
				}
				else
				{
					++Step30GapFlagNoResolutionSameOwnerLossCount;
				}
			}
			else if (bFinalOwnerChanged)
			{
				++Step30NoGapChangedOwnerLossCount;
			}
			else
			{
				++Step30NoGapSameOwnerLossCount;
			}
			break;
		}
	}

	TestEqual(TEXT("Ownership provenance audit recomputes the step-30 gross loss count"), Step30LossTotal, Step30PeriodicActualPlanet.LastResamplingStats.ContinentalSamplesLost);
	TestEqual(TEXT("Ownership provenance audit recomputes the step-30 gross gain count"), Step30GainTotal, 7);
	AddInfo(FString::Printf(
		TEXT("m6_ownership_audit_step30_summary loss_total=%d gain_total=%d no_gap_changed_owner=%d no_gap_same_owner=%d gapflag_no_resolution_changed_owner=%d gapflag_no_resolution_same_owner=%d projection_changed_owner=%d projection_same_owner=%d nearest_copy_changed_owner=%d nearest_copy_same_owner=%d nondive_fallback_oceanized=%d divergent_oceanized=%d final_changed_owner_losses=%d query_changed_owner_losses=%d fallback_query_losses=%d fallback_query_changed_owner_losses=%d fallback_query_same_owner_losses=%d preserve_retain_flag_losses=%d"),
		Step30LossTotal,
		Step30GainTotal,
		Step30NoGapChangedOwnerLossCount,
		Step30NoGapSameOwnerLossCount,
		Step30GapFlagNoResolutionChangedOwnerLossCount,
		Step30GapFlagNoResolutionSameOwnerLossCount,
		Step30ProjectionChangedOwnerLossCount,
		Step30ProjectionSameOwnerLossCount,
		Step30NearestCopyChangedOwnerLossCount,
		Step30NearestCopySameOwnerLossCount,
		Step30NonDivergentFallbackOceanizedLossCount,
		Step30DivergentOceanizedLossCount,
		Step30FinalChangedOwnerLossCount,
		Step30QueryChangedOwnerLossCount,
		Step30FallbackQueryLossCount,
		Step30FallbackQueryChangedOwnerLossCount,
		Step30FallbackQuerySameOwnerLossCount,
		Step30PreserveRetainFlagLossCount));
	AddInfo(FString::Printf(
		TEXT("m6_ownership_audit_step30_cw old_cw_ge_090=%d old_cw_075_090=%d old_cw_050_075=%d final_cw_zero=%d final_cw_000_025=%d final_cw_025_050=%d"),
		Step30OldCWGe090LossCount,
		Step30OldCWGe075Lt090LossCount,
		Step30OldCWGe050Lt075LossCount,
		Step30FinalCWZeroLossCount,
		Step30FinalCWGtZeroLt025LossCount,
		Step30FinalCWGe025Lt050LossCount));
	AddInfo(FString::Printf(
		TEXT("m6_ownership_audit_step30_owner_pairs final_pairs=%s query_pairs=%s"),
		*FormatSortedCountMapForAudit(Step30FinalOwnerPairs),
		*FormatSortedCountMapForAudit(Step30QueryOwnerPairs)));

	while (BasePlanet.CurrentStep < (M6OwnershipAuditStep229 - 1))
	{
		BasePlanet.AdvanceStep();
	}
	TestEqual(TEXT("Ownership provenance audit reaches step 228 before the rift"), BasePlanet.CurrentStep, M6OwnershipAuditStep229 - 1);

	const FTectonicPlanet Step228Planet = BasePlanet;
	const FTectonicPlanet Step229PreRiftPlanet = AdvanceOneStepWithoutResamplingForAudit(Step228Planet);
	TestEqual(TEXT("Ownership provenance audit isolates the step-229 pre-rift state"), Step229PreRiftPlanet.CurrentStep, M6OwnershipAuditStep229);

	FTectonicPlanet Step229ActualPlanet = Step229PreRiftPlanet;
	TestTrue(
		TEXT("Ownership provenance audit replays the logged step-229 automatic rift"),
		Step229ActualPlanet.TriggerForcedRiftInternal(
			M6OwnershipAuditStep229ParentPlateId,
			M6OwnershipAuditStep229ChildCount,
			M6OwnershipAuditStep229AutoTriggerSeed,
			true,
			M6OwnershipAuditStep229TriggerProbability,
			M6OwnershipAuditStep229ParentContinentalSamples,
			M6OwnershipAuditStep229ParentContinentalFraction));
	TestEqual(TEXT("Ownership provenance audit matches the logged step-229 collision count"), Step229ActualPlanet.LastResamplingStats.CollisionCount, 0);

	FTectonicPlanet Step229PreFollowupSplitPlanet = Step229PreRiftPlanet;
	TestTrue(
		TEXT("Ownership provenance audit reproduces the step-229 pre-followup split"),
		TriggerBinaryRiftWithoutFollowupForAudit(
			Step229PreFollowupSplitPlanet,
			M6OwnershipAuditStep229ParentPlateId,
			M6OwnershipAuditStep229AutoTriggerSeed,
			true,
			M6OwnershipAuditStep229TriggerProbability,
			M6OwnershipAuditStep229ParentContinentalSamples,
			M6OwnershipAuditStep229ParentContinentalFraction));
	TestEqual(TEXT("Ownership provenance audit reproduces the logged step-229 child A id"), Step229PreFollowupSplitPlanet.PendingRiftEvent.ChildPlateA, Step229ActualPlanet.LastResamplingStats.RiftChildPlateA);
	TestEqual(TEXT("Ownership provenance audit reproduces the logged step-229 child B id"), Step229PreFollowupSplitPlanet.PendingRiftEvent.ChildPlateB, Step229ActualPlanet.LastResamplingStats.RiftChildPlateB);
	TestEqual(TEXT("Ownership provenance audit reproduces the logged step-229 rift event seed"), Step229PreFollowupSplitPlanet.PendingRiftEvent.EventSeed, Step229ActualPlanet.LastResamplingStats.RiftEventSeed);

	const FManualResampleAuditExecutionResult Step229Replay =
		ReplayResampleForAudit(
			Step229PreFollowupSplitPlanet,
			EResampleOwnershipMode::FullResolution,
			EResampleTriggerReason::RiftFollowup);
	CompareSamplePlateAndCWStateForAudit(
		*this,
		TEXT("step229_rift_followup"),
		Step229ActualPlanet,
		Step229Replay.FinalPlanet);

	TSet<int32> Step229ChildPlateIds;
	for (const int32 ChildPlateId : Step229ActualPlanet.LastResamplingStats.RiftChildPlateIds)
	{
		if (ChildPlateId != INDEX_NONE)
		{
			Step229ChildPlateIds.Add(ChildPlateId);
		}
	}

	int32 Step229GrossGainTotal = 0;
	int32 Step229GrossLossTotal = 0;
	int32 Step229FinalOwnerChangedGainCount = 0;
	int32 Step229SameOwnerFinalGainCount = 0;
	int32 Step229QueryOwnerChangedGainCount = 0;
	int32 Step229QueryOwnerSameGainCount = 0;
	int32 Step229GapOwnerChangedGainCount = 0;
	int32 Step229ReassignedAfterGapGainCount = 0;
	int32 Step229ReassignedAlreadyHighCWGainCount = 0;
	int32 Step229QueryChangedThenFinalRestoredGainCount = 0;
	int32 Step229CreatedAtInterpolationGainCount = 0;
	int32 Step229CreatedAtGapResolutionGainCount = 0;
	int32 Step229CreatedOnlyAfterLaterStageGainCount = 0;
	int32 Step229ProjectionGainCount = 0;
	int32 Step229NearestCopyGainCount = 0;
	int32 Step229NonDivergentFallbackGainCount = 0;
	int32 Step229DivergentOceanizedGainCount = 0;
	int32 Step229GapFlagNoResolutionGainCount = 0;
	int32 Step229QueryChildOwnerGainCount = 0;
	int32 Step229FinalChildOwnerGainCount = 0;
	int32 Step229ParentToChildQueryGainCount = 0;
	int32 Step229ParentToChildFinalGainCount = 0;
	int32 Step229OldAndeanGainCount = 0;
	int32 Step229FinalAndeanGainCount = 0;
	int32 Step229OldCWGe045Lt050GainCount = 0;
	int32 Step229OldCWGe040Lt045GainCount = 0;
	int32 Step229OldCWGe025Lt040GainCount = 0;
	int32 Step229OldCWBlt025GainCount = 0;
	TMap<FString, int32> Step229FinalOwnerPairs;
	TMap<FString, int32> Step229QueryOwnerPairs;
	for (int32 SampleIndex = 0; SampleIndex < Step229Replay.FinalPlanet.Samples.Num(); ++SampleIndex)
	{
		const float PreviousContinentalWeight = Step229Replay.PreviousContinentalWeights[SampleIndex];
		const float InterpolatedContinentalWeight = Step229Replay.ContinentalWeightsAfterInterpolation[SampleIndex];
		const float GapStageContinentalWeight = Step229Replay.ContinentalWeightsAfterGapResolution[SampleIndex];
		const float FinalContinentalWeight = Step229Replay.FinalPlanet.Samples[SampleIndex].ContinentalWeight;
		const bool bWasContinental = PreviousContinentalWeight >= 0.5f;
		const bool bIsContinental = FinalContinentalWeight >= 0.5f;
		Step229GrossLossTotal += (bWasContinental && !bIsContinental) ? 1 : 0;
		if (bWasContinental || !bIsContinental)
		{
			continue;
		}

		++Step229GrossGainTotal;
		const int32 PreviousPlateId = Step229Replay.PreviousPlateAssignments[SampleIndex];
		const int32 QueryPlateId =
			Step229Replay.NewPlateIdsAfterQuery.IsValidIndex(SampleIndex)
				? Step229Replay.NewPlateIdsAfterQuery[SampleIndex]
				: INDEX_NONE;
		const int32 GapPlateId =
			Step229Replay.NewPlateIdsAfterGapResolution.IsValidIndex(SampleIndex)
				? Step229Replay.NewPlateIdsAfterGapResolution[SampleIndex]
				: INDEX_NONE;
		const int32 FinalAssignedPlateId =
			Step229Replay.NewPlateIdsBeforeRepartition.IsValidIndex(SampleIndex)
				? Step229Replay.NewPlateIdsBeforeRepartition[SampleIndex]
				: INDEX_NONE;
		const int32 FinalPlateId = Step229Replay.FinalPlanet.Samples[SampleIndex].PlateId;
		const bool bFinalOwnerChanged = FinalPlateId != PreviousPlateId;
		const bool bQueryOwnerChanged = QueryPlateId != PreviousPlateId;
		const bool bGapOwnerChanged = GapPlateId != PreviousPlateId;
		const bool bReassignedAfterGap = FinalAssignedPlateId != GapPlateId;
		const bool bQueryPlateIsChild = Step229ChildPlateIds.Contains(QueryPlateId);
		const bool bFinalPlateIsChild = Step229ChildPlateIds.Contains(FinalPlateId);
		const bool bGapFlag = Step229Replay.GapFlags.IsValidIndex(SampleIndex) && Step229Replay.GapFlags[SampleIndex] != 0;
		const EGapResolutionPath GapResolutionPath =
			Step229Replay.GapResolutionPaths.IsValidIndex(SampleIndex)
				? Step229Replay.GapResolutionPaths[SampleIndex]
				: EGapResolutionPath::None;

		Step229FinalOwnerChangedGainCount += bFinalOwnerChanged ? 1 : 0;
		Step229SameOwnerFinalGainCount += bFinalOwnerChanged ? 0 : 1;
		Step229QueryOwnerChangedGainCount += bQueryOwnerChanged ? 1 : 0;
		Step229QueryOwnerSameGainCount += bQueryOwnerChanged ? 0 : 1;
		Step229GapOwnerChangedGainCount += bGapOwnerChanged ? 1 : 0;
		Step229ReassignedAfterGapGainCount += bReassignedAfterGap ? 1 : 0;
		Step229ReassignedAlreadyHighCWGainCount += (bReassignedAfterGap && GapStageContinentalWeight >= 0.5f) ? 1 : 0;
		Step229QueryChangedThenFinalRestoredGainCount +=
			(bQueryOwnerChanged && FinalPlateId == PreviousPlateId && GapStageContinentalWeight >= 0.5f) ? 1 : 0;
		Step229CreatedAtInterpolationGainCount += (InterpolatedContinentalWeight >= 0.5f) ? 1 : 0;
		Step229CreatedAtGapResolutionGainCount +=
			(InterpolatedContinentalWeight < 0.5f && GapStageContinentalWeight >= 0.5f) ? 1 : 0;
		Step229CreatedOnlyAfterLaterStageGainCount +=
			(InterpolatedContinentalWeight < 0.5f && GapStageContinentalWeight < 0.5f) ? 1 : 0;
		Step229QueryChildOwnerGainCount += bQueryPlateIsChild ? 1 : 0;
		Step229FinalChildOwnerGainCount += bFinalPlateIsChild ? 1 : 0;
		Step229ParentToChildQueryGainCount +=
			(PreviousPlateId == M6OwnershipAuditStep229ParentPlateId && bQueryPlateIsChild) ? 1 : 0;
		Step229ParentToChildFinalGainCount +=
			(PreviousPlateId == M6OwnershipAuditStep229ParentPlateId && bFinalPlateIsChild) ? 1 : 0;
		Step229OldAndeanGainCount +=
			Step229Replay.PreviousOrogenyTypes[SampleIndex] == EOrogenyType::Andean ? 1 : 0;
		Step229FinalAndeanGainCount +=
			Step229ActualPlanet.Samples[SampleIndex].OrogenyType == EOrogenyType::Andean ? 1 : 0;
		AddOwnerPairCountForAudit(Step229FinalOwnerPairs, PreviousPlateId, FinalPlateId);
		AddOwnerPairCountForAudit(Step229QueryOwnerPairs, PreviousPlateId, QueryPlateId);

		if (PreviousContinentalWeight >= 0.45f)
		{
			++Step229OldCWGe045Lt050GainCount;
		}
		else if (PreviousContinentalWeight >= 0.40f)
		{
			++Step229OldCWGe040Lt045GainCount;
		}
		else if (PreviousContinentalWeight >= 0.25f)
		{
			++Step229OldCWGe025Lt040GainCount;
		}
		else
		{
			++Step229OldCWBlt025GainCount;
		}

		switch (GapResolutionPath)
		{
		case EGapResolutionPath::NonDivergentProjection:
			++Step229ProjectionGainCount;
			break;
		case EGapResolutionPath::NonDivergentNearestCopy:
			++Step229NearestCopyGainCount;
			break;
		case EGapResolutionPath::NonDivergentFallbackOceanized:
			++Step229NonDivergentFallbackGainCount;
			break;
		case EGapResolutionPath::DivergentOceanized:
			++Step229DivergentOceanizedGainCount;
			break;
		case EGapResolutionPath::None:
		default:
			Step229GapFlagNoResolutionGainCount += bGapFlag ? 1 : 0;
			break;
		}
	}

	TestEqual(TEXT("Ownership provenance audit recomputes the step-229 gross gain count"), Step229GrossGainTotal, Step229ActualPlanet.LastResamplingStats.AndeanContinentalGainCount);
	TestEqual(TEXT("Ownership provenance audit recomputes the step-229 gross loss count"), Step229GrossLossTotal, Step229ActualPlanet.LastResamplingStats.ContinentalSamplesLost);
	TestEqual(TEXT("Ownership provenance audit recomputes the step-229 net continental delta"), Step229GrossGainTotal - Step229GrossLossTotal, Step229ActualPlanet.LastResamplingStats.NetContinentalSampleDelta);
	AddInfo(FString::Printf(
		TEXT("m6_ownership_audit_step229_summary gross_gain_total=%d gross_loss_total=%d net_gain=%d final_owner_changed_gains=%d same_owner_final_gains=%d query_owner_changed_gains=%d query_owner_same_gains=%d gap_owner_changed_gains=%d reassigned_after_gap_gains=%d reassigned_already_high_cw_gains=%d query_changed_then_final_restored_gains=%d interpolation_created_gains=%d gap_created_gains=%d created_only_after_later_stage_gains=%d projection_gains=%d nearest_copy_gains=%d nondive_fallback_gains=%d divergent_oceanized_gains=%d gapflag_no_resolution_gains=%d"),
		Step229GrossGainTotal,
		Step229GrossLossTotal,
		Step229GrossGainTotal - Step229GrossLossTotal,
		Step229FinalOwnerChangedGainCount,
		Step229SameOwnerFinalGainCount,
		Step229QueryOwnerChangedGainCount,
		Step229QueryOwnerSameGainCount,
		Step229GapOwnerChangedGainCount,
		Step229ReassignedAfterGapGainCount,
		Step229ReassignedAlreadyHighCWGainCount,
		Step229QueryChangedThenFinalRestoredGainCount,
		Step229CreatedAtInterpolationGainCount,
		Step229CreatedAtGapResolutionGainCount,
		Step229CreatedOnlyAfterLaterStageGainCount,
		Step229ProjectionGainCount,
		Step229NearestCopyGainCount,
		Step229NonDivergentFallbackGainCount,
		Step229DivergentOceanizedGainCount,
		Step229GapFlagNoResolutionGainCount));
	AddInfo(FString::Printf(
		TEXT("m6_ownership_audit_step229_stage_breakdown query_child_owner_gains=%d final_child_owner_gains=%d parent_to_child_query_gains=%d parent_to_child_final_gains=%d old_andean_gains=%d final_andean_gains=%d old_cw_045_050=%d old_cw_040_045=%d old_cw_025_040=%d old_cw_below_025=%d"),
		Step229QueryChildOwnerGainCount,
		Step229FinalChildOwnerGainCount,
		Step229ParentToChildQueryGainCount,
		Step229ParentToChildFinalGainCount,
		Step229OldAndeanGainCount,
		Step229FinalAndeanGainCount,
		Step229OldCWGe045Lt050GainCount,
		Step229OldCWGe040Lt045GainCount,
		Step229OldCWGe025Lt040GainCount,
		Step229OldCWBlt025GainCount));
	AddInfo(FString::Printf(
		TEXT("m6_ownership_audit_step229_owner_pairs final_pairs=%s query_pairs=%s"),
		*FormatSortedCountMapForAudit(Step229FinalOwnerPairs),
		*FormatSortedCountMapForAudit(Step229QueryOwnerPairs)));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetPreserveBoundaryRegressionStep30ResidualLossReducedTest,
	"Aurous.TectonicPlanet.PreserveBoundaryRegressionStep30ResidualLossReducedTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetPreserveBoundaryRegressionStep30ResidualLossReducedTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BasePlanet = CreateM6BaselinePlanetForAudit();
	while (BasePlanet.CurrentStep < (M6OwnershipAuditStep30 - 1))
	{
		BasePlanet.AdvanceStep();
	}
	TestEqual(TEXT("Step-30-style preserve regression reaches step 29 before the periodic pass"), BasePlanet.CurrentStep, M6OwnershipAuditStep30 - 1);

	const FTectonicPlanet Step29Planet = BasePlanet;
	const FTectonicPlanet Step30PrePeriodicPlanet = AdvanceOneStepWithoutResamplingForAudit(Step29Planet);
	const FTectonicPlanet Step30PeriodicActualPlanet = AdvanceOneStepAllowSingleResampleForAudit(Step29Planet);
	TestEqual(TEXT("Step-30-style preserve regression isolates the pre-periodic state"), Step30PrePeriodicPlanet.CurrentStep, M6OwnershipAuditStep30);
	TestEqual(TEXT("Step-30-style preserve regression preserves the real periodic ownership mode"), Step30PeriodicActualPlanet.LastResampleOwnershipMode, EResampleOwnershipMode::PreserveOwnership);

	const FManualResampleAuditExecutionResult Step30Replay =
		ReplayResampleForAudit(
			Step30PrePeriodicPlanet,
			EResampleOwnershipMode::PreserveOwnership,
			EResampleTriggerReason::Periodic);
	CompareSamplePlateAndCWStateForAudit(
		*this,
		TEXT("step30_periodic_m6ah"),
		Step30PeriodicActualPlanet,
		Step30Replay.FinalPlanet);

	int32 LossTotal = 0;
	int32 ChangedOwnerLossCount = 0;
	int32 NonDivergentProjectionLossCount = 0;
	int32 NonDivergentFallbackOceanizedLossCount = 0;
	int32 DivergentOceanizationLossCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < Step30Replay.FinalPlanet.Samples.Num(); ++SampleIndex)
	{
		const bool bWasContinental = Step30Replay.PreviousContinentalWeights[SampleIndex] >= 0.5f;
		const bool bIsContinental = Step30Replay.FinalPlanet.Samples[SampleIndex].ContinentalWeight >= 0.5f;
		if (!bWasContinental || bIsContinental)
		{
			continue;
		}

		++LossTotal;
		const bool bChangedOwner =
			Step30Replay.FinalPlanet.Samples[SampleIndex].PlateId !=
			Step30Replay.PreviousPlateAssignments[SampleIndex];
		ChangedOwnerLossCount += bChangedOwner ? 1 : 0;

		const EGapResolutionPath GapResolutionPath =
			Step30Replay.GapResolutionPaths.IsValidIndex(SampleIndex)
				? Step30Replay.GapResolutionPaths[SampleIndex]
				: EGapResolutionPath::None;
		NonDivergentProjectionLossCount +=
			(GapResolutionPath == EGapResolutionPath::NonDivergentProjection) ? 1 : 0;
		NonDivergentFallbackOceanizedLossCount +=
			(GapResolutionPath == EGapResolutionPath::NonDivergentFallbackOceanized) ? 1 : 0;
		DivergentOceanizationLossCount +=
			(GapResolutionPath == EGapResolutionPath::DivergentOceanized) ? 1 : 0;
	}

	TestTrue(TEXT("Step-30 residual preserve regression reduces continental losses beyond the M6ah result"), LossTotal < 371);
	TestTrue(TEXT("Step-30 residual preserve regression reduces changed-owner losses beyond the M6ah result"), ChangedOwnerLossCount < 281);
	TestTrue(TEXT("Step-30 residual preserve regression applies previous-owner hysteresis at least once"), Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipPreviousOwnerHysteresisApplicationCount > 0);
	TestTrue(TEXT("Step-30 residual preserve regression still allows genuine changed-owner fallback outcomes"), Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackChangedOwnerCount > 0);
	AddInfo(FString::Printf(
		TEXT("m6_preserve_step30_residual_loss_reduced continental_losses=%d changed_owner_losses=%d non_divergent_projection_losses=%d non_divergent_fallback_oceanized_losses=%d divergent_oceanizations=%d previous_owner_hysteresis_application_count=%d preserved_strongly_continental_boundary_sample_count=%d fallback_same_plate_recontained_count=%d fallback_same_plate_retained_count=%d fallback_changed_owner_count=%d fallback_gap_count=%d fallback_divergent_oceanization_count=%d fallback_continental_loss_count_after_fallback=%d changed_owner_non_gap_loss_count=%d divergent_loss_count=%d strong_loss_ge_090_count=%d top_changed_owner_loss_pairs=%s"),
		LossTotal,
		ChangedOwnerLossCount,
		NonDivergentProjectionLossCount,
		NonDivergentFallbackOceanizedLossCount,
		DivergentOceanizationLossCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipPreviousOwnerHysteresisApplicationCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipStronglyContinentalBoundarySavedCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackSamePlateRecontainedCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackSamePlateRetainedCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackChangedOwnerCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackGapCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackDivergentOceanizationCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipContinentalLossCountAfterFallback,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackChangedOwnerNonGapLossCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackDivergentLossCount,
		Step30PeriodicActualPlanet.LastResamplingStats.PreserveOwnershipFallbackStrongLossGE090Count,
		*FormatPreserveFallbackLossPairs(Step30PeriodicActualPlanet.LastResamplingStats)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftTopologyRegressionStep229StylePulseReducedTest,
	"Aurous.TectonicPlanet.RiftTopologyRegressionStep229StylePulseReducedTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftTopologyRegressionStep229StylePulseReducedTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet BasePlanet = CreateM6BaselinePlanetForAudit();
	while (BasePlanet.CurrentStep < (M6OwnershipAuditStep229 - 1))
	{
		BasePlanet.AdvanceStep();
	}
	TestEqual(TEXT("Step-229-style pulse regression reaches step 228 before the rift"), BasePlanet.CurrentStep, M6OwnershipAuditStep229 - 1);

	const FTectonicPlanet Step228Planet = BasePlanet;
	const FTectonicPlanet Step229PreRiftPlanet = AdvanceOneStepWithoutResamplingForAudit(Step228Planet);
	TestEqual(TEXT("Step-229-style pulse regression isolates the pre-rift state"), Step229PreRiftPlanet.CurrentStep, M6OwnershipAuditStep229);

	FTectonicPlanet Step229ActualPlanet = Step229PreRiftPlanet;
	TestTrue(
		TEXT("Step-229-style pulse regression replays the automatic rift"),
		Step229ActualPlanet.TriggerForcedRiftInternal(
			M6OwnershipAuditStep229ParentPlateId,
			M6OwnershipAuditStep229ChildCount,
			M6OwnershipAuditStep229AutoTriggerSeed,
			true,
			M6OwnershipAuditStep229TriggerProbability,
			M6OwnershipAuditStep229ParentContinentalSamples,
			M6OwnershipAuditStep229ParentContinentalFraction));

	FTectonicPlanet Step229PreFollowupSplitPlanet = Step229PreRiftPlanet;
	TestTrue(
		TEXT("Step-229-style pulse regression reproduces the pre-followup split"),
		TriggerBinaryRiftWithoutFollowupForAudit(
			Step229PreFollowupSplitPlanet,
			M6OwnershipAuditStep229ParentPlateId,
			M6OwnershipAuditStep229AutoTriggerSeed,
			true,
			M6OwnershipAuditStep229TriggerProbability,
			M6OwnershipAuditStep229ParentContinentalSamples,
			M6OwnershipAuditStep229ParentContinentalFraction));

	const FManualResampleAuditExecutionResult Step229Replay =
		ReplayResampleForAudit(
			Step229PreFollowupSplitPlanet,
			EResampleOwnershipMode::FullResolution,
			EResampleTriggerReason::RiftFollowup);
	CompareSamplePlateAndCWStateForAudit(
		*this,
		TEXT("step229_rift_followup_post_patch"),
		Step229ActualPlanet,
		Step229Replay.FinalPlanet);

	int32 GrossGainTotal = 0;
	int32 GrossLossTotal = 0;
	int32 InterpolationCreatedGains = 0;
	for (int32 SampleIndex = 0; SampleIndex < Step229Replay.FinalPlanet.Samples.Num(); ++SampleIndex)
	{
		const bool bWasContinental = Step229Replay.PreviousContinentalWeights[SampleIndex] >= 0.5f;
		const bool bInterpolatedContinental = Step229Replay.ContinentalWeightsAfterInterpolation[SampleIndex] >= 0.5f;
		const bool bIsContinental = Step229Replay.FinalPlanet.Samples[SampleIndex].ContinentalWeight >= 0.5f;
		GrossGainTotal += (!bWasContinental && bIsContinental) ? 1 : 0;
		GrossLossTotal += (bWasContinental && !bIsContinental) ? 1 : 0;
		InterpolationCreatedGains += (!bWasContinental && bInterpolatedContinental) ? 1 : 0;
	}

	TestEqual(TEXT("Step-229-style pulse regression recomputes the gross gain count"), GrossGainTotal, Step229ActualPlanet.LastResamplingStats.AndeanContinentalGainCount);
	TestEqual(TEXT("Step-229-style pulse regression recomputes the gross loss count"), GrossLossTotal, Step229ActualPlanet.LastResamplingStats.ContinentalSamplesLost);
	TestEqual(TEXT("Step-229-style pulse regression recomputes interpolation-created gains"), InterpolationCreatedGains, Step229ActualPlanet.LastResamplingStats.RiftInterpolationCreatedGainCount);
	TestTrue(TEXT("Step-229-style pulse regression reduces the gross gain below the M6ae baseline"), GrossGainTotal < 2301);
	TestTrue(TEXT("Step-229-style pulse regression reduces the net gain below the M6ae baseline"), (GrossGainTotal - GrossLossTotal) < 2072);
	TestTrue(TEXT("Step-229-style pulse regression prevents at least one localized phantom continental gain"), Step229ActualPlanet.LastResamplingStats.RiftLocalizedCWPhantomPreventedCount > 0);
	AddInfo(FString::Printf(
		TEXT("m6_rift_step229_style_pulse_reduced gross_gain_total=%d gross_loss_total=%d net_gain=%d interpolation_created_gains=%d localized_cw_restored_count=%d localized_cw_phantom_prevented_count=%d localized_cw_continental_prevented_count=%d final_owner_mismatch_gain_count_after_localization=%d"),
		GrossGainTotal,
		GrossLossTotal,
		GrossGainTotal - GrossLossTotal,
		InterpolationCreatedGains,
		Step229ActualPlanet.LastResamplingStats.RiftLocalizedCWRestoredCount,
		Step229ActualPlanet.LastResamplingStats.RiftLocalizedCWPhantomPreventedCount,
		Step229ActualPlanet.LastResamplingStats.RiftLocalizedCWContinentalPreventedCount,
		Step229ActualPlanet.LastResamplingStats.RiftFinalOwnerMismatchGainCountAfterLocalization));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetRiftTopologyRegressionStep229StylePulseFurtherReducedTest,
	"Aurous.TectonicPlanet.RiftTopologyRegressionStep229StylePulseFurtherReducedTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetRiftTopologyRegressionStep229StylePulseFurtherReducedTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = CreateManualRiftFinalOwnerPulseDiagnosticPlanet();
	const TArray<int32> PreviousPlateAssignments = { 21, 10, 11, 20, 21 };
	const TArray<float> PreviousContinentalWeights = { 0.15f, 0.90f, 0.20f, 0.10f, 0.20f };
	const TArray<int32> InterpolationPlateIds = { 10, 10, 11, 20, 10 };
	const TArray<int32> FinalPlateIds = { 20, 10, 11, 20, 10 };
	TArray<uint8> LocalizedRestoreFlags;
	LocalizedRestoreFlags.Init(0, Planet.Samples.Num());

	Planet.Samples[0].ContinentalWeight = 0.85f;
	Planet.Samples[1].ContinentalWeight = 0.90f;
	Planet.Samples[2].ContinentalWeight = 0.80f;
	Planet.Samples[3].ContinentalWeight = 0.10f;
	Planet.Samples[4].ContinentalWeight = 0.88f;

	int32 GrossGainBeforeReconciliation = 0;
	int32 GrossLossBeforeReconciliation = 0;
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		const bool bWasContinental = PreviousContinentalWeights[SampleIndex] >= 0.5f;
		const bool bIsProvisionallyContinental = Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f;
		GrossGainBeforeReconciliation += (!bWasContinental && bIsProvisionallyContinental) ? 1 : 0;
		GrossLossBeforeReconciliation += (bWasContinental && !bIsProvisionallyContinental) ? 1 : 0;
	}

	FResamplingStats Stats;
	Planet.ApplyRiftFollowupFinalOwnerContinentalWeightReconciliation(
		InterpolationPlateIds,
		FinalPlateIds,
		PreviousPlateAssignments,
		PreviousContinentalWeights,
		LocalizedRestoreFlags,
		&Stats);

	int32 GrossGainTotal = 0;
	int32 GrossLossTotal = 0;
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		const bool bWasContinental = PreviousContinentalWeights[SampleIndex] >= 0.5f;
		const bool bIsContinental = Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f;
		GrossGainTotal += (!bWasContinental && bIsContinental) ? 1 : 0;
		GrossLossTotal += (bWasContinental && !bIsContinental) ? 1 : 0;
	}

	TestEqual(TEXT("Step-229-style pulse further-reduced regression creates three provisional pulse gains before reconciliation"), GrossGainBeforeReconciliation, 3);
	TestEqual(TEXT("Step-229-style pulse further-reduced regression leaves one legitimate gain after reconciliation"), GrossGainTotal, 1);
	TestEqual(TEXT("Step-229-style pulse further-reduced regression keeps gross losses at zero in the modeled scenario"), GrossLossTotal, 0);
	TestTrue(TEXT("Step-229-style pulse further-reduced regression reduces the modeled gross gain"), GrossGainTotal < GrossGainBeforeReconciliation);
	TestTrue(
		TEXT("Step-229-style pulse further-reduced regression sharply reduces mismatch gains after reconciliation"),
		Stats.RiftFinalOwnerMismatchGainCountAfterReconciliation <
			Stats.RiftFinalOwnerMismatchGainCountBeforeReconciliation);
	TestTrue(
		TEXT("Step-229-style pulse further-reduced regression reduces same-owner child interpolation gains after reconciliation"),
		Stats.RiftSameOwnerChildInterpolationGainCountAfterReconciliation <
			Stats.RiftSameOwnerChildInterpolationGainCountBeforeReconciliation);
	TestEqual(TEXT("Step-229-style pulse further-reduced regression prevents one mismatch continental gain"), Stats.RiftFinalOwnerMismatchContinentalPreventedCount, 1);
	TestEqual(TEXT("Step-229-style pulse further-reduced regression reconciles three provisional CW values"), Stats.RiftFinalOwnerCWReconciledCount, 3);
	TestTrue(
		TEXT("Step-229-style pulse further-reduced regression preserves one legitimate final-owner child gain"),
		FMath::IsNearlyEqual(Planet.Samples[4].ContinentalWeight, 0.90f, 1.0e-6f));
	AddInfo(FString::Printf(
		TEXT("m6_rift_step229_style_pulse_further_reduced gross_gain_total=%d gross_loss_total=%d net_gain=%d interpolation_created_gains=%d gross_gain_before_reconciliation=%d gross_loss_before_reconciliation=%d final_owner_cw_reconciled_count=%d mismatch_gain_count_before_reconciliation=%d mismatch_gain_count_after_reconciliation=%d same_owner_child_interpolation_gain_count_before_reconciliation=%d same_owner_child_interpolation_gain_count_after_reconciliation=%d final_gain_started_below_025_count=%d final_gain_started_below_040_count=%d final_gain_started_below_050_count=%d"),
		GrossGainTotal,
		GrossLossTotal,
		GrossGainTotal - GrossLossTotal,
		GrossGainBeforeReconciliation,
		GrossGainBeforeReconciliation,
		GrossLossBeforeReconciliation,
		Stats.RiftFinalOwnerCWReconciledCount,
		Stats.RiftFinalOwnerMismatchGainCountBeforeReconciliation,
		Stats.RiftFinalOwnerMismatchGainCountAfterReconciliation,
		Stats.RiftSameOwnerChildInterpolationGainCountBeforeReconciliation,
		Stats.RiftSameOwnerChildInterpolationGainCountAfterReconciliation,
		Stats.RiftFinalGainStartedBelow025Count,
		Stats.RiftFinalGainStartedBelow040Count,
		Stats.RiftFinalGainStartedBelow050Count));
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

// ---------------------------------------------------------------------------
// Vertex-Level Soup Inclusion Spike: Reconstruction-Model Falsification
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetVertexLevelSoupInclusionSpikeTest,
	"Aurous.TectonicPlanet.VertexLevelSoupInclusionSpike",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetVertexLevelSoupInclusionSpikeTest::RunTest(const FString& Parameters)
{
	// ---- Spike-local diagnostic structs ----

	struct FSpikeOwnerCoverageAudit
	{
		int32 AssignedSampleCount = 0;
		int32 OwnerQueryExactHitCount = 0;
		int32 OwnerSoupContainedCount = 0;
	};

	struct FSpikeCheckpointMetrics
	{
		int32 Step = 0;
		double ContinentalAreaFraction = 0.0;
		int32 MultiHitCount = 0;
		int32 MaxComponentsAfter = 0;
		int32 MissCount = 0;
		double OwnershipChurnFraction = 0.0;
		int32 InteriorLeakageCount = 0;
		double MeanConflictRingDist = 0.0;
		double CoherenceScore = 0.0;
		int32 OwnerQueryExactHit = 0;
		int32 OwnerSoupContained = 0;
		int32 CollisionCount = 0;
		int32 PlatesAbove5PctHits = 0;
		double LargestPlateShare = 0.0;
		int32 GapCount = 0;
		int32 OverlapCount = 0;
		int32 TotalSoupTriangleCount = 0;
		int32 DuplicatedSoupTriangleCount = 0;
		int32 ForeignLocalVertexCount = 0;
	};

	// ---- Lambdas for spike-local metric computation ----

	auto ComputeSpikeOwnerCoverageAudit = [](FTectonicPlanet& Planet) -> FSpikeOwnerCoverageAudit
	{
		FSpikeOwnerCoverageAudit Audit;

		// owner_soup_contained: sample is in its assigned plate's CanonicalToCarriedIndex
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const FSample& Sample = Planet.Samples[SampleIndex];
			if (Sample.PlateId == INDEX_NONE)
			{
				continue;
			}

			++Audit.AssignedSampleCount;

			const FPlate* PlatePtr = nullptr;
			for (const FPlate& Plate : Planet.Plates)
			{
				if (Plate.Id == Sample.PlateId)
				{
					PlatePtr = &Plate;
					break;
				}
			}

			if (PlatePtr != nullptr && PlatePtr->CanonicalToCarriedIndex.Contains(SampleIndex))
			{
				++Audit.OwnerSoupContainedCount;
			}
		}

		// owner_query_exact_hit: run a full QueryOwnership and count samples where
		// the query returns the same plate the sample is currently assigned to.
		TArray<int32> QueryPlateIds;
		TArray<int32> QueryContainingTriangles;
		TArray<FVector3d> QueryBarycentricCoords;
		TArray<uint8> QueryGapFlags;
		TArray<uint8> QueryOverlapFlags;
		int32 QueryGapCount = 0;
		int32 QueryOverlapCount = 0;
		FResamplingStats QueryStats;
		Planet.QueryOwnership(
			QueryPlateIds,
			QueryContainingTriangles,
			QueryBarycentricCoords,
			QueryGapFlags,
			QueryOverlapFlags,
			QueryGapCount,
			QueryOverlapCount,
			nullptr,
			&QueryStats);

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (Planet.Samples[SampleIndex].PlateId != INDEX_NONE &&
				QueryPlateIds.IsValidIndex(SampleIndex) &&
				QueryPlateIds[SampleIndex] == Planet.Samples[SampleIndex].PlateId)
			{
				++Audit.OwnerQueryExactHitCount;
			}
		}

		return Audit;
	};

	auto ComputeMaxComponents = [](const FTectonicPlanet& Planet) -> int32
	{
		int32 MaxComponents = 0;
		for (const FPlate& Plate : Planet.Plates)
		{
			if (Plate.MemberSamples.IsEmpty())
			{
				continue;
			}

			TSet<int32> MemberSet;
			MemberSet.Reserve(Plate.MemberSamples.Num());
			for (const int32 SampleIndex : Plate.MemberSamples)
			{
				MemberSet.Add(SampleIndex);
			}

			TArray<uint8> Visited;
			Visited.Init(0, Planet.Samples.Num());
			int32 ComponentCount = 0;
			for (const int32 SeedIndex : Plate.MemberSamples)
			{
				if (!Planet.Samples.IsValidIndex(SeedIndex) || Visited[SeedIndex] != 0)
				{
					continue;
				}

				TArray<int32, TInlineAllocator<128>> Stack;
				Stack.Add(SeedIndex);
				Visited[SeedIndex] = 1;
				while (!Stack.IsEmpty())
				{
					const int32 Current = Stack.Pop(EAllowShrinking::No);
					if (Planet.SampleAdjacency.IsValidIndex(Current))
					{
						for (const int32 Neighbor : Planet.SampleAdjacency[Current])
						{
							if (Planet.Samples.IsValidIndex(Neighbor) &&
								Visited[Neighbor] == 0 &&
								MemberSet.Contains(Neighbor))
							{
								Visited[Neighbor] = 1;
								Stack.Add(Neighbor);
							}
						}
					}
				}
				++ComponentCount;
			}

			MaxComponents = FMath::Max(MaxComponents, ComponentCount);
		}

		return MaxComponents;
	};

	auto ComputeOwnershipChurn = [](const TArray<int32>& PreviousPlateIds, const FTectonicPlanet& Planet) -> double
	{
		if (Planet.Samples.IsEmpty())
		{
			return 0.0;
		}

		int32 ChangedCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (PreviousPlateIds.IsValidIndex(SampleIndex) &&
				Planet.Samples[SampleIndex].PlateId != PreviousPlateIds[SampleIndex])
			{
				++ChangedCount;
			}
		}

		return static_cast<double>(ChangedCount) / static_cast<double>(Planet.Samples.Num());
	};

	auto ComputeMultiHitCount = [](FTectonicPlanet& Planet) -> int32
	{
		// Run a full ownership query to get accurate multi-hit counts from the BVH.
		TArray<int32> QueryPlateIds;
		TArray<int32> QueryContainingTriangles;
		TArray<FVector3d> QueryBarycentricCoords;
		TArray<uint8> QueryGapFlags;
		TArray<uint8> QueryOverlapFlags;
		int32 QueryGapCount = 0;
		int32 QueryOverlapCount = 0;
		FResamplingStats QueryStats;
		Planet.QueryOwnership(
			QueryPlateIds,
			QueryContainingTriangles,
			QueryBarycentricCoords,
			QueryGapFlags,
			QueryOverlapFlags,
			QueryGapCount,
			QueryOverlapCount,
			nullptr,
			&QueryStats);

		return QueryStats.ExactMultiHitCount;
	};

	auto ComputePlatesAbove5PctHits = [](const FTectonicPlanet& Planet) -> int32
	{
		const double Threshold = 0.05 * static_cast<double>(Planet.Samples.Num());
		int32 Count = 0;
		for (const FPlate& Plate : Planet.Plates)
		{
			if (static_cast<double>(Plate.MemberSamples.Num()) >= Threshold)
			{
				++Count;
			}
		}

		return Count;
	};

	auto ComputeLargestPlateShare = [](const FTectonicPlanet& Planet) -> double
	{
		int32 LargestPlateSize = 0;
		for (const FPlate& Plate : Planet.Plates)
		{
			LargestPlateSize = FMath::Max(LargestPlateSize, Plate.MemberSamples.Num());
		}

		return Planet.Samples.IsEmpty()
			? 0.0
			: static_cast<double>(LargestPlateSize) / static_cast<double>(Planet.Samples.Num());
	};

	auto SnapshotPlateIds = [](const FTectonicPlanet& Planet) -> TArray<int32>
	{
		TArray<int32> PlateIds;
		PlateIds.Reserve(Planet.Samples.Num());
		for (const FSample& Sample : Planet.Samples)
		{
			PlateIds.Add(Sample.PlateId);
		}

		return PlateIds;
	};

	auto BuildCheckpointMetrics = [&](FTectonicPlanet& Planet, const TArray<int32>& PreviousPlateIds, const int32 Step) -> FSpikeCheckpointMetrics
	{
		// Ensure soups and BVHs are fresh for the current plate state before querying.
		Planet.BuildContainmentSoups();

		FSpikeCheckpointMetrics M;
		M.Step = Step;
		M.ContinentalAreaFraction = ComputeContinentalAreaFraction(Planet);
		M.MultiHitCount = ComputeMultiHitCount(Planet);
		M.MaxComponentsAfter = ComputeMaxComponents(Planet);
		M.MissCount = Planet.LastResamplingStats.TrueGapCount;
		M.OwnershipChurnFraction = ComputeOwnershipChurn(PreviousPlateIds, Planet);
		M.CollisionCount = Planet.LastResamplingStats.CollisionCount;
		M.PlatesAbove5PctHits = ComputePlatesAbove5PctHits(Planet);
		M.LargestPlateShare = ComputeLargestPlateShare(Planet);
		M.GapCount = Planet.LastResamplingStats.GapCount;
		M.OverlapCount = Planet.LastResamplingStats.OverlapCount;
		M.TotalSoupTriangleCount = Planet.LastResamplingStats.TotalSoupTriangleCount;
		M.DuplicatedSoupTriangleCount = Planet.LastResamplingStats.DuplicatedSoupTriangleCount;
		M.ForeignLocalVertexCount = Planet.LastResamplingStats.ForeignLocalVertexCount;

		// Interior leakage: samples owned by plate P that are interior (not boundary)
		// but NOT in plate P's carried sample map.
		int32 InteriorLeakage = 0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const FSample& Sample = Planet.Samples[SampleIndex];
			if (Sample.PlateId == INDEX_NONE || Sample.bIsBoundary)
			{
				continue;
			}

			const FPlate* PlatePtr = nullptr;
			for (const FPlate& Plate : Planet.Plates)
			{
				if (Plate.Id == Sample.PlateId)
				{
					PlatePtr = &Plate;
					break;
				}
			}

			if (PlatePtr != nullptr && !PlatePtr->CanonicalToCarriedIndex.Contains(SampleIndex))
			{
				++InteriorLeakage;
			}
		}
		M.InteriorLeakageCount = InteriorLeakage;

		// Mean conflict ring distance: for boundary samples, mean graph distance to nearest
		// sample owned by a different plate. Approximated as 0 for boundary, 1 for 1-hop neighbor.
		double ConflictRingDistSum = 0.0;
		int32 BoundaryCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const FSample& Sample = Planet.Samples[SampleIndex];
			if (!Sample.bIsBoundary || Sample.PlateId == INDEX_NONE)
			{
				continue;
			}

			++BoundaryCount;
			int32 MinRing = TNumericLimits<int32>::Max();
			if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 Neighbor : Planet.SampleAdjacency[SampleIndex])
				{
					if (Planet.Samples.IsValidIndex(Neighbor) &&
						Planet.Samples[Neighbor].PlateId != Sample.PlateId)
					{
						MinRing = 0;
						break;
					}
				}

				if (MinRing > 0)
				{
					MinRing = 1;
				}
			}

			ConflictRingDistSum += static_cast<double>(MinRing);
		}
		M.MeanConflictRingDist = BoundaryCount > 0 ? ConflictRingDistSum / static_cast<double>(BoundaryCount) : 0.0;

		// Coherence score: fraction of non-boundary samples that are surrounded
		// exclusively by same-plate neighbors.
		int32 InteriorSampleCount = 0;
		int32 CoherentInteriorCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const FSample& Sample = Planet.Samples[SampleIndex];
			if (Sample.bIsBoundary || Sample.PlateId == INDEX_NONE)
			{
				continue;
			}

			++InteriorSampleCount;
			bool bCoherent = true;
			if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 Neighbor : Planet.SampleAdjacency[SampleIndex])
				{
					if (Planet.Samples.IsValidIndex(Neighbor) &&
						Planet.Samples[Neighbor].PlateId != Sample.PlateId)
					{
						bCoherent = false;
						break;
					}
				}
			}

			if (bCoherent)
			{
				++CoherentInteriorCount;
			}
		}
		M.CoherenceScore = InteriorSampleCount > 0
			? static_cast<double>(CoherentInteriorCount) / static_cast<double>(InteriorSampleCount)
			: 0.0;

		const FSpikeOwnerCoverageAudit Audit = ComputeSpikeOwnerCoverageAudit(Planet);
		M.OwnerQueryExactHit = Audit.OwnerQueryExactHitCount;
		M.OwnerSoupContained = Audit.OwnerSoupContainedCount;

		return M;
	};

	auto LogCheckpoint = [this](const TCHAR* VariantLabel, const FSpikeCheckpointMetrics& M)
	{
		const FString Line = FString::Printf(
			TEXT("[VertexLevelSoupSpike %s step=%d] continental_area_fraction=%.4f multi_hit_count=%d max_components_after=%d miss_count=%d ownership_churn_fraction=%.4f interior_leakage=%d mean_conflict_ring_dist=%.3f coherence_score=%.4f owner_query_exact_hit=%d owner_soup_contained=%d collision_count=%d plates_above_5pct_hits=%d largest_plate_share=%.4f gap_count=%d overlap_count=%d total_soup_triangle_count=%d duplicated_soup_triangle_count=%d foreign_local_vertex_count=%d"),
			VariantLabel,
			M.Step,
			M.ContinentalAreaFraction,
			M.MultiHitCount,
			M.MaxComponentsAfter,
			M.MissCount,
			M.OwnershipChurnFraction,
			M.InteriorLeakageCount,
			M.MeanConflictRingDist,
			M.CoherenceScore,
			M.OwnerQueryExactHit,
			M.OwnerSoupContained,
			M.CollisionCount,
			M.PlatesAbove5PctHits,
			M.LargestPlateShare,
			M.GapCount,
			M.OverlapCount,
			M.TotalSoupTriangleCount,
			M.DuplicatedSoupTriangleCount,
			M.ForeignLocalVertexCount);
		AddInfo(Line);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Line);
	};

	// ---- Run both variants ----

	struct FVariant
	{
		FString Label;
		bool bVertexLevelInclusion = false;
		FTectonicPlanet Planet;
		TArray<int32> PreStep25PlateIds;
		TArray<int32> PreStep100PlateIds;
		TArray<int32> PreStep200PlateIds;
		FSpikeCheckpointMetrics Step25Metrics;
		FSpikeCheckpointMetrics Step100Metrics;
		FSpikeCheckpointMetrics Step200Metrics;
		bool bRanStep200 = false;
	};

	TArray<FVariant> Variants;
	Variants.Reserve(2);
	{
		FVariant Baseline;
		Baseline.Label = TEXT("Baseline");
		Baseline.bVertexLevelInclusion = false;
		Variants.Add(MoveTemp(Baseline));
	}
	{
		FVariant Spike;
		Spike.Label = TEXT("VertexLevelInclusion");
		Spike.bVertexLevelInclusion = true;
		Variants.Add(MoveTemp(Spike));
	}

	const FString ExportBaseRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("VertexLevelSoupInclusionSpike"));

	for (FVariant& Variant : Variants)
	{
		FTectonicPlanet& Planet = Variant.Planet;
		Planet.Initialize(TestSampleCount, TestPlanetRadiusKm);
		// Set the spike flag BEFORE InitializePlates so ClassifyPlateTriangles in init
		// uses vertex-level inclusion from the start.
		Planet.bUseVertexLevelSoupInclusionForTest = Variant.bVertexLevelInclusion;
		Planet.InitializePlates(StressPlateCount, TestRandomSeed, TestBoundaryWarpAmplitude, TestContinentalFraction);
		Planet.bEnableAutomaticRifting = false;
		// Cadence ~25: ResamplingPolicy PeriodicFull with 60k/40 gives interval ~10.
		// We run to step 25, then 100, and optionally 200.
		// Retention OFF, propagation OFF: these are v5-only concepts, the default PeriodicFull path
		// does not use them.
		// Frontier-pair fill ON: this is handled by ResolveGaps in the standard pipeline.

		const FString VariantExportRoot = FPaths::Combine(ExportBaseRoot, Variant.Label);

		// Step 0 initial coverage audit
		Planet.BuildContainmentSoups();
		{
			const FSpikeOwnerCoverageAudit InitAudit = ComputeSpikeOwnerCoverageAudit(Planet);
			AddInfo(FString::Printf(
				TEXT("[VertexLevelSoupSpike %s step=0] assigned=%d owner_query_exact_hit=%d owner_soup_contained=%d coverage_pct=%.2f"),
				*Variant.Label,
				InitAudit.AssignedSampleCount,
				InitAudit.OwnerQueryExactHitCount,
				InitAudit.OwnerSoupContainedCount,
				InitAudit.AssignedSampleCount > 0
					? 100.0 * static_cast<double>(InitAudit.OwnerSoupContainedCount) / static_cast<double>(InitAudit.AssignedSampleCount)
					: 0.0));
		}

		// Run to step 25 — snapshot initial state for churn measurement
		Variant.PreStep25PlateIds = SnapshotPlateIds(Planet);
		for (int32 StepIndex = 0; StepIndex < 25; ++StepIndex)
		{
			Planet.AdvanceStep();
		}
		Variant.Step25Metrics = BuildCheckpointMetrics(Planet, Variant.PreStep25PlateIds, 25);
		LogCheckpoint(*Variant.Label, Variant.Step25Metrics);

		// Run to step 100 — snapshot state at step 25 for churn measurement
		Variant.PreStep100PlateIds = SnapshotPlateIds(Planet);
		for (int32 StepIndex = 25; StepIndex < 100; ++StepIndex)
		{
			Planet.AdvanceStep();
		}
		Variant.Step100Metrics = BuildCheckpointMetrics(Planet, Variant.PreStep100PlateIds, 100);
		LogCheckpoint(*Variant.Label, Variant.Step100Metrics);

		// Export visual maps for the spike variant at step 100
		if (Variant.bVertexLevelInclusion)
		{
			ExportCheckpointMaps(*this, Planet, VariantExportRoot, 100);
		}
	}

	// ---- Decide whether to run step 200 ----
	const FSpikeCheckpointMetrics& SpikeStep100 = Variants[1].Step100Metrics;
	const double SpikeStep100CoveragePct = SpikeStep100.OwnerSoupContained > 0 && Variants[1].Planet.Samples.Num() > 0
		? 100.0 * static_cast<double>(SpikeStep100.OwnerSoupContained) / static_cast<double>(Variants[1].Planet.Samples.Num())
		: 0.0;
	const bool bSpikeShowsPromise = SpikeStep100CoveragePct > 10.0 && SpikeStep100.OwnershipChurnFraction < 0.50;

	if (bSpikeShowsPromise)
	{
		AddInfo(TEXT("[VertexLevelSoupSpike] Step 100 looks promising, extending to step 200"));
		for (FVariant& Variant : Variants)
		{
			Variant.PreStep200PlateIds = SnapshotPlateIds(Variant.Planet);
			for (int32 StepIndex = 100; StepIndex < 200; ++StepIndex)
			{
				Variant.Planet.AdvanceStep();
			}
			Variant.Step200Metrics = BuildCheckpointMetrics(Variant.Planet, Variant.PreStep200PlateIds, 200);
			Variant.bRanStep200 = true;
			LogCheckpoint(*Variant.Label, Variant.Step200Metrics);

			if (Variant.bVertexLevelInclusion)
			{
				const FString VariantExportRoot = FPaths::Combine(ExportBaseRoot, Variant.Label);
				ExportCheckpointMaps(*this, Variant.Planet, VariantExportRoot, 200);
			}
		}
	}

	// ---- Final comparative summary ----
	const FSpikeCheckpointMetrics& BaselineStep100 = Variants[0].Step100Metrics;

	const double BaselineCovPct = Variants[0].Planet.Samples.Num() > 0
		? 100.0 * static_cast<double>(BaselineStep100.OwnerSoupContained) / static_cast<double>(Variants[0].Planet.Samples.Num())
		: 0.0;

	AddInfo(FString::Printf(
		TEXT("[VertexLevelSoupSpike Compare step=100] coverage_pct=%.2f->%.2f churn=%.4f->%.4f multi_hit=%d->%d max_components=%d->%d miss=%d->%d coherence=%.4f->%.4f interior_leakage=%d->%d collision=%d->%d gap=%d->%d overlap=%d->%d duplicated_soup=%d->%d"),
		BaselineCovPct,
		SpikeStep100CoveragePct,
		BaselineStep100.OwnershipChurnFraction,
		SpikeStep100.OwnershipChurnFraction,
		BaselineStep100.MultiHitCount,
		SpikeStep100.MultiHitCount,
		BaselineStep100.MaxComponentsAfter,
		SpikeStep100.MaxComponentsAfter,
		BaselineStep100.MissCount,
		SpikeStep100.MissCount,
		BaselineStep100.CoherenceScore,
		SpikeStep100.CoherenceScore,
		BaselineStep100.InteriorLeakageCount,
		SpikeStep100.InteriorLeakageCount,
		BaselineStep100.CollisionCount,
		SpikeStep100.CollisionCount,
		BaselineStep100.GapCount,
		SpikeStep100.GapCount,
		BaselineStep100.OverlapCount,
		SpikeStep100.OverlapCount,
		BaselineStep100.DuplicatedSoupTriangleCount,
		SpikeStep100.DuplicatedSoupTriangleCount));
	UE_LOG(LogTemp, Log,
		TEXT("[VertexLevelSoupSpike Compare step=100] coverage_pct=%.2f->%.2f churn=%.4f->%.4f multi_hit=%d->%d max_components=%d->%d miss=%d->%d coherence=%.4f->%.4f interior_leakage=%d->%d"),
		BaselineCovPct,
		SpikeStep100CoveragePct,
		BaselineStep100.OwnershipChurnFraction,
		SpikeStep100.OwnershipChurnFraction,
		BaselineStep100.MultiHitCount,
		SpikeStep100.MultiHitCount,
		BaselineStep100.MaxComponentsAfter,
		SpikeStep100.MaxComponentsAfter,
		BaselineStep100.MissCount,
		SpikeStep100.MissCount,
		BaselineStep100.CoherenceScore,
		SpikeStep100.CoherenceScore,
		BaselineStep100.InteriorLeakageCount,
		SpikeStep100.InteriorLeakageCount);

	// ---- Hard stop verdict ----
	const bool bCoverageRoseDramatically = SpikeStep100CoveragePct > 50.0;
	const bool bChurnDroppedBelow50 = SpikeStep100.OwnershipChurnFraction < 0.50;
	const bool bSpikeSucceeded = bCoverageRoseDramatically && bChurnDroppedBelow50;

	if (bSpikeSucceeded)
	{
		AddInfo(TEXT("[VertexLevelSoupSpike VERDICT] SPIKE SUCCEEDED: coverage >50% and churn <50%. Vertex-level soup inclusion is the hidden fix."));
	}
	else
	{
		AddInfo(FString::Printf(
			TEXT("[VertexLevelSoupSpike VERDICT] SPIKE FAILED: coverage=%.2f%% (need >50%%), churn=%.4f (need <0.50). Recommend ownership-authority rewrite."),
			SpikeStep100CoveragePct,
			SpikeStep100.OwnershipChurnFraction));
	}

	// We do not assert pass/fail on the spike outcome — the test is observational.
	// The test always passes; the verdict is in the logged metrics.
	return true;
}
