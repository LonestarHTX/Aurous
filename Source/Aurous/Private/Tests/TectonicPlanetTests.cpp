#include "Misc/AutomationTest.h"

#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Interface/Core/RealtimeMeshDataStream.h"
#include "Math/RandomStream.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "ShewchukPredicates.h"
#include "TectonicPlanet.h"
#include "TectonicPlanetOwnershipUtils.h"
#include "TectonicPlanetActor.h"
#include "TectonicReferenceScenarios.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr int32 TestSampleCount = 500000;
	constexpr int32 TestPlateCount = 7;
	constexpr int32 TestSeed = 42;

	struct FCachedPlanetState
	{
		FTectonicPlanet Planet;
		int32 SampleCount = 0;
		double InitializeSeconds = 0.0;
		bool bInitialized = false;
	};

	FCachedPlanetState& GetCachedPlanetState(const int32 SampleCount = TestSampleCount)
	{
		static TMap<int32, FCachedPlanetState> States;
		FCachedPlanetState& State = States.FindOrAdd(SampleCount);
		if (!State.bInitialized)
		{
			const double StartSeconds = FPlatformTime::Seconds();
			State.SampleCount = SampleCount;
			State.Planet.Initialize(SampleCount);
			State.InitializeSeconds = FPlatformTime::Seconds() - StartSeconds;
			State.bInitialized = true;

			UE_LOG(LogTemp, Log, TEXT("Initialize(%d) wall time: %.3f s"), SampleCount, State.InitializeSeconds);
		}

		return State;
	}

	const FTectonicPlanet& GetCachedPlanet(const int32 SampleCount = TestSampleCount)
	{
		return GetCachedPlanetState(SampleCount).Planet;
	}

	FTectonicPlanet MakePlanetCopy(const int32 SampleCount = TestSampleCount)
	{
		return GetCachedPlanet(SampleCount);
	}

	FVector MakeRandomUnitVector(FRandomStream& Random)
	{
		const double Z = Random.FRandRange(-1.0f, 1.0f);
		const double Phi = Random.FRandRange(0.0f, 2.0f * PI);
		const double RadiusXY = FMath::Sqrt(FMath::Max(0.0, 1.0 - Z * Z));
		return FVector(RadiusXY * FMath::Cos(Phi), RadiusXY * FMath::Sin(Phi), Z);
	}

	void LinkBidirectional(TArray<TArray<int32>>& Adjacency, const int32 A, const int32 B)
	{
		if (!Adjacency.IsValidIndex(A) || !Adjacency.IsValidIndex(B))
		{
			return;
		}

		Adjacency[A].AddUnique(B);
		Adjacency[B].AddUnique(A);
	}

	void InitializeFixturePlate(FPlate& Plate, const int32 PlateId, const FVector& RotationAxis, const float AngularSpeed = 0.01f)
	{
		Plate.Id = PlateId;
		Plate.RotationAxis = RotationAxis.GetSafeNormal();
		Plate.AngularSpeed = AngularSpeed;
	}

	void InitializeConvergentBoundarySample(
		FCanonicalSample& Sample,
		const int32 PlateId,
		const FVector& Position,
		const int32 OpposingPlateId,
		const ECrustType CrustType,
		const float Age,
		const bool bFlipBoundaryNormal = false)
	{
		Sample.Position = Position.GetSafeNormal();
		Sample.PlateId = PlateId;
		Sample.PrevPlateId = PlateId;
		Sample.CrustType = CrustType;
		Sample.Age = Age;
		Sample.bIsBoundary = true;
		Sample.bOverlapDetected = true;
		Sample.BoundaryType = EBoundaryType::Convergent;
		Sample.BoundaryNormal = FVector::CrossProduct(FVector::UpVector, Sample.Position).GetSafeNormal();
		if (bFlipBoundaryNormal)
		{
			Sample.BoundaryNormal *= -1.0f;
		}
		if (Sample.BoundaryNormal.IsNearlyZero())
		{
			Sample.BoundaryNormal = FVector::ForwardVector;
		}
		Sample.NumOverlapPlateIds = 1;
		Sample.OverlapPlateIds[0] = OpposingPlateId;
	}
}
	struct FTectonicPlanetTestAccess
	{
		static TArray<FCanonicalSample>& MutableSamples(FTectonicPlanet& Planet)
		{
			return Planet.SampleBuffers[Planet.ReadableSampleBufferIndex.Load()];
		}

		static TArray<TArray<int32>>& MutableAdjacency(FTectonicPlanet& Planet)
		{
			return Planet.Adjacency;
		}

		static TArray<FPlate>& MutablePlates(FTectonicPlanet& Planet)
		{
			return Planet.Plates;
		}

		static bool ResolveGapSamplePhase4(FTectonicPlanet& Planet, const int32 SampleIndex, const TArray<uint8>& GapFlags, TArray<FCanonicalSample>& InOutSamples, bool& bOutDivergentGapCreated)
		{
			return Planet.ResolveGapSamplePhase4(SampleIndex, GapFlags, InOutSamples, bOutDivergentGapCreated);
		}

		static void UpdateSubductionFields(FTectonicPlanet& Planet, TArray<FCanonicalSample>& InOutSamples)
		{
			Planet.UpdateSubductionFieldsForSamples(InOutSamples);
		}

		static void RebuildPlateMembership(FTectonicPlanet& Planet, const TArray<FCanonicalSample>& InSamples)
		{
			Planet.RebuildPlateMembershipFromSamples(InSamples);
		}

		static void UpdatePlateCanonicalCenters(FTectonicPlanet& Planet, const TArray<FCanonicalSample>& InSamples)
		{
			Planet.UpdatePlateCanonicalCentersFromSamples(InSamples);
		}

		static void RebuildCarriedWorkspaces(FTectonicPlanet& Planet, const TArray<FCanonicalSample>& InSamples)
		{
			Planet.RebuildCarriedSampleWorkspacesForSamples(InSamples);
		}

		static void EnforceConnectedOwnership(FTectonicPlanet& Planet, TArray<FCanonicalSample>& InOutSamples)
		{
			Planet.EnforceConnectedPlateOwnershipForSamples(InOutSamples);
		}

		static bool RescueProtectedOwnership(FTectonicPlanet& Planet, TArray<FCanonicalSample>& InOutSamples)
		{
			return Planet.RescueProtectedPlateOwnershipForSamples(InOutSamples);
		}

		static void DetectTerranes(FTectonicPlanet& Planet, TArray<FCanonicalSample>& InOutSamples)
		{
			Planet.DetectTerranesForSamples(InOutSamples);
		}

		static bool ApplyContinentalCollisions(FTectonicPlanet& Planet, TArray<FCanonicalSample>& InOutSamples)
		{
			return Planet.ApplyContinentalCollisionEventsForSamples(InOutSamples);
		}

		static bool ApplyContinentalCollisions(FTectonicPlanet& Planet, TArray<FCanonicalSample>& InOutSamples, TArray<uint8>& OutDirtyPlateFlags)
		{
			return Planet.ApplyContinentalCollisionEventsForSamples(InOutSamples, &OutDirtyPlateFlags);
		}

		static bool ApplyNextContinentalCollision(
			FTectonicPlanet& Planet,
			TArray<FCanonicalSample>& InOutSamples,
			TSet<int32>& InOutTerranesUsedThisReconcile,
			TArray<uint8>* OutDirtyPlateFlags = nullptr)
		{
			return Planet.ApplyNextContinentalCollisionEventForSamples(InOutSamples, &InOutTerranesUsedThisReconcile, OutDirtyPlateFlags, nullptr);
		}

		static void RefreshCanonicalAfterCollision(
			FTectonicPlanet& Planet,
			TArray<FCanonicalSample>& InOutSamples)
		{
			Planet.RefreshCanonicalStateAfterCollision(InOutSamples);
		}

		static void RefreshCanonicalAfterCollision(
			FTectonicPlanet& Planet,
			TArray<FCanonicalSample>& InOutSamples,
			TArray<uint8>& InOutDirtyPlateFlags)
		{
			Planet.RefreshCanonicalStateAfterCollision(InOutSamples, &InOutDirtyPlateFlags);
		}

		static void SetAreaMetrics(FTectonicPlanet& Planet, const double InAverageCellAreaKm2, const double InInitialMeanPlateAreaKm2)
		{
			Planet.AverageCellAreaKm2 = InAverageCellAreaKm2;
			Planet.InitialMeanPlateAreaKm2 = InInitialMeanPlateAreaKm2;
		}

		static void SetInitialContinentalPlateCount(FTectonicPlanet& Planet, const int32 InCount)
		{
			Planet.InitialContinentalPlateCount = InCount;
		}

		static void StabilizeContinentalIncremental(
			FTectonicPlanet& Planet,
			const TArray<FCanonicalSample>& PreviousSamples,
			TArray<FCanonicalSample>& InOutSamples,
			FReconcilePhaseTimings* InOutTimings = nullptr)
		{
			Planet.StabilizeContinentalCrustForSamplesIncremental(PreviousSamples, InOutSamples, InOutTimings);
		}

		static void InterpolateTriangleFields(
			const TArray<FCanonicalSample>& ReadSamples,
			const FDelaunayTriangle& Triangle,
			const FCarriedSampleData& V0,
			const FCarriedSampleData& V1,
			const FCarriedSampleData& V2,
			const FVector& Barycentric,
			FCanonicalSample& InOutDestSample)
		{
			FTectonicPlanet::InterpolateTriangleFields(ReadSamples, Triangle, V0, V1, V2, Barycentric, InOutDestSample);
		}
		static int32 GetTerraneMergedIntoId(const FTectonicPlanet& Planet, const int32 TerraneId)
		{
			for (const auto& Record : Planet.TerraneRecords)
			{
				if (Record.TerraneId == TerraneId)
				{
					return Record.MergedIntoTerraneId;
				}
			}
			return INDEX_NONE;
		}

		static int32 GetTerraneAnchorSampleIndex(const FTectonicPlanet& Planet, const int32 TerraneId)
		{
			for (const auto& Record : Planet.TerraneRecords)
			{
				if (Record.TerraneId == TerraneId)
				{
					return Record.AnchorSampleIndex;
				}
			}
			return INDEX_NONE;
		}

		static int32 GetCollisionHistoryKeyCount(const FTectonicPlanet& Planet)
		{
			return Planet.CollisionHistoryKeys.Num();
		}

		static int32 GetCollisionEventDonorTerraneId(const FTectonicPlanet& Planet, const int32 EventIndex)
		{
			return Planet.CollisionEvents.IsValidIndex(EventIndex) ? Planet.CollisionEvents[EventIndex].DonorTerraneId : INDEX_NONE;
		}

		static int32 GetCollisionEventReceiverTerraneId(const FTectonicPlanet& Planet, const int32 EventIndex)
		{
			return Planet.CollisionEvents.IsValidIndex(EventIndex) ? Planet.CollisionEvents[EventIndex].ReceiverTerraneId : INDEX_NONE;
		}

		static int32 GetCollisionEventDonorPlateId(const FTectonicPlanet& Planet, const int32 EventIndex)
		{
			return Planet.CollisionEvents.IsValidIndex(EventIndex) ? Planet.CollisionEvents[EventIndex].DonorPlateId : INDEX_NONE;
		}

		static int32 GetCollisionEventReceiverPlateId(const FTectonicPlanet& Planet, const int32 EventIndex)
		{
			return Planet.CollisionEvents.IsValidIndex(EventIndex) ? Planet.CollisionEvents[EventIndex].ReceiverPlateId : INDEX_NONE;
		}

		static int32 GetCollisionEventContactSampleCount(const FTectonicPlanet& Planet, const int32 EventIndex)
		{
			return Planet.CollisionEvents.IsValidIndex(EventIndex) ? Planet.CollisionEvents[EventIndex].ContactSampleCount : 0;
		}

		static float GetCollisionEventMeanConvergenceSpeed(const FTectonicPlanet& Planet, const int32 EventIndex)
		{
			return Planet.CollisionEvents.IsValidIndex(EventIndex) ? Planet.CollisionEvents[EventIndex].MeanConvergenceSpeedMmPerYear : 0.0f;
		}

		static float GetCollisionEventRadiusKm(const FTectonicPlanet& Planet, const int32 EventIndex)
		{
			return Planet.CollisionEvents.IsValidIndex(EventIndex) ? Planet.CollisionEvents[EventIndex].CollisionRadiusKm : 0.0f;
		}

		static double GetCollisionEventAreaKm2(const FTectonicPlanet& Planet, const int32 EventIndex)
		{
			return Planet.CollisionEvents.IsValidIndex(EventIndex) ? Planet.CollisionEvents[EventIndex].CollisionAreaKm2 : 0.0;
		}
	};

namespace
{
	struct FStabilizerFixtureState
	{
		FTectonicPlanet Planet;
		TArray<FCanonicalSample> PreviousSamples;
		TArray<FCanonicalSample> CurrentSamples;
	};

	FVector MakeStabilizerFixturePosition(const int32 SampleIndex, const int32 NumSamples)
	{
		const double SampleCount = static_cast<double>(FMath::Max(1, NumSamples));
		const double AngleRadians = (2.0 * PI * static_cast<double>(SampleIndex)) / SampleCount;
		const double Z = 0.10 * static_cast<double>((SampleIndex % 3) - 1);
		return FVector(FMath::Cos(AngleRadians), FMath::Sin(AngleRadians), Z).GetSafeNormal();
	}

	void InitializeStabilizerFixtureSample(
		FCanonicalSample& Sample,
		const int32 SampleIndex,
		const int32 NumSamples,
		const ECrustType CrustType,
		const float Elevation,
		const float Thickness,
		const float Age,
		const bool bGapDetected = false,
		const int32 PlateId = 0)
	{
		Sample = FCanonicalSample{};
		Sample.Position = MakeStabilizerFixturePosition(SampleIndex, NumSamples);
		Sample.PlateId = PlateId;
		Sample.PrevPlateId = PlateId;
		Sample.CrustType = CrustType;
		Sample.Elevation = Elevation;
		Sample.Thickness = Thickness;
		Sample.Age = Age;
		Sample.RidgeDirection = FVector(1.0, 0.05 * static_cast<double>(SampleIndex + 1), 0.0).GetSafeNormal();
		Sample.FoldDirection = FVector(0.0, 1.0, 0.05 * static_cast<double>(SampleIndex + 1)).GetSafeNormal();
		Sample.OrogenyType = (CrustType == ECrustType::Continental) ? EOrogenyType::Andean : EOrogenyType::None;
		Sample.OrogenyAge = (CrustType == ECrustType::Continental) ? (Age * 0.25f) : 0.0f;
		Sample.TerraneId = (CrustType == ECrustType::Continental) ? (100 + SampleIndex) : INDEX_NONE;
		Sample.CollisionDistanceKm = (CrustType == ECrustType::Continental) ? (25.0f + static_cast<float>(SampleIndex)) : -1.0f;
		Sample.CollisionConvergenceSpeedMmPerYear = (CrustType == ECrustType::Continental) ? (10.0f + static_cast<float>(SampleIndex)) : 0.0f;
		Sample.bIsCollisionFront = (CrustType == ECrustType::Continental);
		Sample.bGapDetected = bGapDetected;
	}

	void InitializeStabilizerFixture(FStabilizerFixtureState& Fixture, const int32 NumSamples, const int32 NumPlates = 1)
	{
		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Fixture.Planet);
		TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Fixture.Planet);

		Fixture.PreviousSamples.SetNum(NumSamples);
		Fixture.CurrentSamples.SetNum(NumSamples);
		Adjacency.SetNum(NumSamples);
		Plates.SetNum(NumPlates);

		for (int32 PlateIndex = 0; PlateIndex < NumPlates; ++PlateIndex)
		{
			InitializeFixturePlate(Plates[PlateIndex], PlateIndex, FVector::UpVector);
		}

		for (int32 SampleIndex = 0; SampleIndex < NumSamples; ++SampleIndex)
		{
			InitializeStabilizerFixtureSample(
				Fixture.PreviousSamples[SampleIndex],
				SampleIndex,
				NumSamples,
				ECrustType::Oceanic,
				-5.0f - 0.1f * static_cast<float>(SampleIndex),
				7.0f + 0.1f * static_cast<float>(SampleIndex),
				40.0f + static_cast<float>(SampleIndex));
			InitializeStabilizerFixtureSample(
				Fixture.CurrentSamples[SampleIndex],
				SampleIndex,
				NumSamples,
				ECrustType::Oceanic,
				-6.0f - 0.1f * static_cast<float>(SampleIndex),
				8.0f + 0.1f * static_cast<float>(SampleIndex),
				2.0f + static_cast<float>(SampleIndex));
		}

		Fixture.Planet.SetMinContinentalAreaFraction(0.0);
		FTectonicPlanetTestAccess::SetInitialContinentalPlateCount(Fixture.Planet, 0);
	}

	int32 CountContinentalSamples(const TArray<FCanonicalSample>& Samples)
	{
		int32 Count = 0;
		for (const FCanonicalSample& Sample : Samples)
		{
			Count += (Sample.CrustType == ECrustType::Continental) ? 1 : 0;
		}
		return Count;
	}

	void TestOceanicTemplateMatch(
		FAutomationTestBase& Test,
		const FString& Label,
		const FCanonicalSample& Actual,
		const FCanonicalSample& ExpectedTemplate)
	{
		Test.TestEqual(FString::Printf(TEXT("Crust is oceanic [%s]"), *Label), Actual.CrustType, ECrustType::Oceanic);
		Test.TestTrue(FString::Printf(TEXT("Elevation matches template [%s]"), *Label), FMath::IsNearlyEqual(Actual.Elevation, ExpectedTemplate.Elevation, KINDA_SMALL_NUMBER));
		Test.TestTrue(FString::Printf(TEXT("Thickness matches template [%s]"), *Label), FMath::IsNearlyEqual(Actual.Thickness, ExpectedTemplate.Thickness, KINDA_SMALL_NUMBER));
		Test.TestTrue(FString::Printf(TEXT("Age matches template [%s]"), *Label), FMath::IsNearlyEqual(Actual.Age, ExpectedTemplate.Age, KINDA_SMALL_NUMBER));
		Test.TestTrue(FString::Printf(TEXT("RidgeDirection matches template [%s]"), *Label), Actual.RidgeDirection.Equals(ExpectedTemplate.RidgeDirection, 1.0e-6));
		Test.TestTrue(FString::Printf(TEXT("FoldDirection matches template [%s]"), *Label), Actual.FoldDirection.Equals(ExpectedTemplate.FoldDirection, 1.0e-6));
		Test.TestEqual(FString::Printf(TEXT("Orogeny clears on demotion [%s]"), *Label), Actual.OrogenyType, EOrogenyType::None);
		Test.TestTrue(FString::Printf(TEXT("OrogenyAge clears on demotion [%s]"), *Label), FMath::IsNearlyEqual(Actual.OrogenyAge, 0.0f, KINDA_SMALL_NUMBER));
		Test.TestEqual(FString::Printf(TEXT("Terrane clears on demotion [%s]"), *Label), Actual.TerraneId, INDEX_NONE);
	}

	void RunStabilizerFixture(
		FStabilizerFixtureState& Fixture,
		TArray<FCanonicalSample>* OutSamples = nullptr,
		FReconcilePhaseTimings* OutTimings = nullptr)
	{
		TArray<FCanonicalSample> IncrementalSamples = Fixture.CurrentSamples;
		FReconcilePhaseTimings IncrementalTimings;
		FTectonicPlanetTestAccess::StabilizeContinentalIncremental(Fixture.Planet, Fixture.PreviousSamples, IncrementalSamples, &IncrementalTimings);
		if (OutSamples)
		{
			*OutSamples = IncrementalSamples;
		}
		if (OutTimings)
		{
			*OutTimings = IncrementalTimings;
		}
	}

	FStabilizerFixtureState MakePromotionTieBreakFixture(const bool bEqualPreviousNeighborCounts)
	{
		FStabilizerFixtureState Fixture;
		InitializeStabilizerFixture(Fixture, 4);
		Fixture.Planet.SetMinContinentalAreaFraction(0.5);

		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Fixture.Planet);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[0], 0, 4, ECrustType::Continental, 1.0f, 35.0f, 10.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[0], 0, 4, ECrustType::Continental, 1.5f, 36.0f, 11.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[1], 1, 4, ECrustType::Continental, 2.0f, 38.0f, 20.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[1], 1, 4, ECrustType::Oceanic, -4.0f, 7.2f, 60.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[2], 2, 4, ECrustType::Continental, 3.0f, 39.0f, 30.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[2], 2, 4, ECrustType::Oceanic, -4.5f, 7.4f, 70.0f);
		InitializeStabilizerFixtureSample(
			Fixture.PreviousSamples[3],
			3,
			4,
			bEqualPreviousNeighborCounts ? ECrustType::Oceanic : ECrustType::Continental,
			4.0f,
			40.0f,
			40.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[3], 3, 4, ECrustType::Oceanic, -4.8f, 7.8f, 80.0f);

		LinkBidirectional(Adjacency, 0, 1);
		LinkBidirectional(Adjacency, 0, 2);
		if (!bEqualPreviousNeighborCounts)
		{
			LinkBidirectional(Adjacency, 1, 3);
		}
		return Fixture;
	}

	FStabilizerFixtureState MakeZeroCurrentFallbackFixture()
	{
		FStabilizerFixtureState Fixture;
		InitializeStabilizerFixture(Fixture, 4);
		Fixture.Planet.SetMinContinentalAreaFraction(0.25);

		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Fixture.Planet);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[1], 1, 4, ECrustType::Continental, 2.5f, 37.0f, 12.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[2], 2, 4, ECrustType::Continental, 3.5f, 38.0f, 22.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[3], 3, 4, ECrustType::Continental, 4.5f, 39.0f, 32.0f);
		LinkBidirectional(Adjacency, 1, 2);
		LinkBidirectional(Adjacency, 2, 3);
		return Fixture;
	}

	FStabilizerFixtureState MakeGapPromotionFixture()
	{
		FStabilizerFixtureState Fixture;
		InitializeStabilizerFixture(Fixture, 4);
		Fixture.Planet.SetMinContinentalAreaFraction(0.5);

		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Fixture.Planet);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[0], 0, 4, ECrustType::Oceanic, -5.0f, 7.0f, 1.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[0], 0, 4, ECrustType::Continental, 9.0f, 42.0f, 50.0f, true);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[1], 1, 4, ECrustType::Continental, 2.0f, 36.0f, 11.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[1], 1, 4, ECrustType::Oceanic, -4.0f, 7.1f, 61.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[2], 2, 4, ECrustType::Continental, 3.0f, 37.0f, 21.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[2], 2, 4, ECrustType::Oceanic, -4.2f, 7.2f, 62.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[3], 3, 4, ECrustType::Continental, 4.0f, 38.0f, 31.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[3], 3, 4, ECrustType::Oceanic, -4.4f, 7.3f, 63.0f);

		LinkBidirectional(Adjacency, 0, 1);
		LinkBidirectional(Adjacency, 2, 3);
		return Fixture;
	}

	FStabilizerFixtureState MakeComponentPruneOrderFixture()
	{
		FStabilizerFixtureState Fixture;
		InitializeStabilizerFixture(Fixture, 8);
		Fixture.Planet.SetMinContinentalAreaFraction(0.0);
		FTectonicPlanetTestAccess::SetInitialContinentalPlateCount(Fixture.Planet, 1);

		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Fixture.Planet);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[0], 0, 8, ECrustType::Continental, 10.0f, 35.0f, 5.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[2], 2, 8, ECrustType::Continental, 12.0f, 37.0f, 7.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[4], 4, 8, ECrustType::Continental, 14.0f, 39.0f, 9.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[6], 6, 8, ECrustType::Continental, 16.0f, 41.0f, 11.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[2], 2, 8, ECrustType::Continental, 22.0f, 47.0f, 17.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[4], 4, 8, ECrustType::Continental, 24.0f, 49.0f, 19.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[6], 6, 8, ECrustType::Continental, 26.0f, 51.0f, 21.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[1], 1, 8, ECrustType::Oceanic, -8.0f, 6.2f, 81.0f);

		LinkBidirectional(Adjacency, 0, 1);
		LinkBidirectional(Adjacency, 2, 3);
		LinkBidirectional(Adjacency, 4, 5);
		LinkBidirectional(Adjacency, 6, 7);
		return Fixture;
	}

	FStabilizerFixtureState MakePreviousOceanicDemotionFixture()
	{
		FStabilizerFixtureState Fixture;
		InitializeStabilizerFixture(Fixture, 4);
		Fixture.Planet.SetMinContinentalAreaFraction(0.0);

		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Fixture.Planet);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[0], 0, 4, ECrustType::Oceanic, -9.0f, 6.0f, 77.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[0], 0, 4, ECrustType::Continental, 5.0f, 32.0f, 17.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[1], 1, 4, ECrustType::Oceanic, -2.0f, 7.0f, 88.0f);
		InitializeStabilizerFixtureSample(Fixture.PreviousSamples[2], 2, 4, ECrustType::Continental, 6.0f, 33.0f, 27.0f);
		InitializeStabilizerFixtureSample(Fixture.CurrentSamples[2], 2, 4, ECrustType::Continental, 7.0f, 34.0f, 28.0f);
		LinkBidirectional(Adjacency, 0, 1);
		LinkBidirectional(Adjacency, 2, 3);
		return Fixture;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFibonacciSphereTest, "Aurous.TectonicPlanet.FibonacciSphere",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFibonacciSphereTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanet& Planet = GetCachedPlanet();
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();

	TestEqual(TEXT("Sample count"), Samples.Num(), TestSampleCount);

	double MinRadius = TNumericLimits<double>::Max();
	double MaxRadius = 0.0;
	double MaxRadiusError = 0.0;
	int32 OffSphereCount = 0;

	for (const FCanonicalSample& Sample : Samples)
	{
		const double Magnitude = Sample.Position.Size();
		MinRadius = FMath::Min(MinRadius, Magnitude);
		MaxRadius = FMath::Max(MaxRadius, Magnitude);
		MaxRadiusError = FMath::Max(MaxRadiusError, FMath::Abs(Magnitude - 1.0));
		OffSphereCount += FMath::IsNearlyEqual(Magnitude, 1.0, 1e-10) ? 0 : 1;
	}

	int32 CoincidentSanityFailures = 0;
	for (int32 I = 0; I < FMath::Min(1000, Samples.Num()); ++I)
	{
		for (int32 J = I + 1; J < FMath::Min(I + 10, Samples.Num()); ++J)
		{
			const double Distance = FVector::Dist(Samples[I].Position, Samples[J].Position);
			CoincidentSanityFailures += (Distance <= 1e-8) ? 1 : 0;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Fibonacci metrics (500k): min|p|=%.15f max|p|=%.15f max|r-1|=%.3e offSphere=%d coincidenceSanityFailures=%d"),
		MinRadius, MaxRadius, MaxRadiusError, OffSphereCount, CoincidentSanityFailures);

	TestEqual(TEXT("Off-sphere sample count"), OffSphereCount, 0);
	TestEqual(TEXT("Coincident sanity failures"), CoincidentSanityFailures, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDelaunayTriangulationTest, "Aurous.TectonicPlanet.DelaunayTriangulation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FDelaunayTriangulationTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanet& Planet = GetCachedPlanet();
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();

	const int32 ExpectedTriangles = 2 * Samples.Num() - 4;
	TestEqual(TEXT("Triangle count matches Euler formula"), Triangles.Num(), ExpectedTriangles);
	TestEqual(TEXT("Triangle count at 500k"), Triangles.Num(), 999996);

	TArray<uint8> ReferencedVertices;
	ReferencedVertices.Init(0, Samples.Num());

	int32 InvalidIndexCount = 0;
	int32 DegenerateIndexCount = 0;
	int32 SmallAreaCount = 0;
	double MinArea2 = TNumericLimits<double>::Max();
	double MaxArea2 = 0.0;

	for (const FDelaunayTriangle& Triangle : Triangles)
	{
		const int32 AIndex = Triangle.V[0];
		const int32 BIndex = Triangle.V[1];
		const int32 CIndex = Triangle.V[2];

		if (!Samples.IsValidIndex(AIndex) || !Samples.IsValidIndex(BIndex) || !Samples.IsValidIndex(CIndex))
		{
			++InvalidIndexCount;
			continue;
		}

		ReferencedVertices[AIndex] = 1;
		ReferencedVertices[BIndex] = 1;
		ReferencedVertices[CIndex] = 1;

		if (AIndex == BIndex || BIndex == CIndex || AIndex == CIndex)
		{
			++DegenerateIndexCount;
			continue;
		}

		const FVector& A = Samples[AIndex].Position;
		const FVector& B = Samples[BIndex].Position;
		const FVector& C = Samples[CIndex].Position;
		const double Area2 = FVector::CrossProduct(B - A, C - A).Size();
		MinArea2 = FMath::Min(MinArea2, Area2);
		MaxArea2 = FMath::Max(MaxArea2, Area2);
		SmallAreaCount += (Area2 < static_cast<double>(SMALL_NUMBER)) ? 1 : 0;
	}

	int32 MissingVertexCount = 0;
	for (const uint8 bReferenced : ReferencedVertices)
	{
		MissingVertexCount += (bReferenced == 0) ? 1 : 0;
	}

	UE_LOG(LogTemp, Log, TEXT("Delaunay metrics (500k): Triangles=%d Expected=%d InvalidIdx=%d DegenerateIdx=%d SmallArea=%d Min|cross|=%.6e Max|cross|=%.6e MissingVerts=%d"),
		Triangles.Num(),
		ExpectedTriangles,
		InvalidIndexCount,
		DegenerateIndexCount,
		SmallAreaCount,
		(MinArea2 == TNumericLimits<double>::Max()) ? 0.0 : MinArea2,
		MaxArea2,
		MissingVertexCount);

	TestEqual(TEXT("Invalid triangle index count"), InvalidIndexCount, 0);
	TestEqual(TEXT("Degenerate index triangle count"), DegenerateIndexCount, 0);
	TestEqual(TEXT("Small-area triangle count"), SmallAreaCount, 0);
	TestEqual(TEXT("Missing referenced vertices"), MissingVertexCount, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAdjacencyGraphTest, "Aurous.TectonicPlanet.AdjacencyGraph",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FAdjacencyGraphTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanet& Planet = GetCachedPlanet();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();

	TestEqual(TEXT("Adjacency array size"), Adjacency.Num(), Samples.Num());
	TestEqual(TEXT("Adjacency array size at 500k"), Adjacency.Num(), TestSampleCount);

	int32 Val5 = 0;
	int32 Val6 = 0;
	int32 Val7 = 0;
	int32 ValOther = 0;
	int32 LowValenceCount = 0;
	int32 InvalidNeighborRefs = 0;
	int32 AsymmetricRefs = 0;

	for (int32 VertexIndex = 0; VertexIndex < Adjacency.Num(); ++VertexIndex)
	{
		const TArray<int32>& Neighbors = Adjacency[VertexIndex];
		const int32 Valence = Neighbors.Num();

		LowValenceCount += (Valence < 3) ? 1 : 0;

		if (Valence == 5)
		{
			++Val5;
		}
		else if (Valence == 6)
		{
			++Val6;
		}
		else if (Valence == 7)
		{
			++Val7;
		}
		else
		{
			++ValOther;
		}

		for (const int32 NeighborIndex : Neighbors)
		{
			if (!Adjacency.IsValidIndex(NeighborIndex))
			{
				++InvalidNeighborRefs;
				continue;
			}

			if (!Adjacency[NeighborIndex].Contains(VertexIndex))
			{
				++AsymmetricRefs;
			}
		}
	}

	const int32 TotalVertices = FMath::Max(1, Adjacency.Num());
	const double PctVal5 = static_cast<double>(Val5) / static_cast<double>(TotalVertices);
	const double PctVal6 = static_cast<double>(Val6) / static_cast<double>(TotalVertices);
	const double PctVal7 = static_cast<double>(Val7) / static_cast<double>(TotalVertices);
	const double PctOther = static_cast<double>(ValOther) / static_cast<double>(TotalVertices);
	const double PctNormal = static_cast<double>(Val5 + Val6 + Val7) / static_cast<double>(TotalVertices);

	UE_LOG(LogTemp, Log, TEXT("Adjacency valence distribution (500k): V5=%d (%.2f%%) V6=%d (%.2f%%) V7=%d (%.2f%%) Other=%d (%.2f%%)"),
		Val5, PctVal5 * 100.0,
		Val6, PctVal6 * 100.0,
		Val7, PctVal7 * 100.0,
		ValOther, PctOther * 100.0);
	UE_LOG(LogTemp, Log, TEXT("Adjacency integrity (500k): LowValence=%d InvalidRefs=%d AsymmetricRefs=%d NormalValencePct=%.2f%%"),
		LowValenceCount, InvalidNeighborRefs, AsymmetricRefs, PctNormal * 100.0);

	TestEqual(TEXT("Vertices with valence < 3"), LowValenceCount, 0);
	TestEqual(TEXT("Invalid neighbor references"), InvalidNeighborRefs, 0);
	TestEqual(TEXT("Asymmetric neighbor references"), AsymmetricRefs, 0);
	TestTrue(TEXT("Majority valence 5-7"), PctNormal > 0.95);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlateInitTest, "Aurous.TectonicPlanet.PlateInitialization",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPlateInitTest::RunTest(const FString& Parameters)
{
	for (const FReferenceScenarioDefinition& Scenario : GetLockedReferenceScenarios())
	{
		FTectonicPlanet Planet = MakePlanetCopy(Scenario.SampleCount);
		Planet.InitializePlates(Scenario.PlateCount, Scenario.Seed);

		const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
		const TArray<FPlate>& Plates = Planet.GetPlates();
		const FString ScenarioLabel = FString::Printf(
			TEXT("%s (%d samples, %d plates, seed=%d)"),
			Scenario.Name,
			Scenario.SampleCount,
			Scenario.PlateCount,
			Scenario.Seed);

		TestEqual(FString::Printf(TEXT("Plate count [%s]"), *ScenarioLabel), Plates.Num(), Scenario.PlateCount);

		int32 InvalidPlateAssignments = 0;
		bool bHasContinental = false;
		bool bHasOceanic = false;
		for (const FCanonicalSample& Sample : Samples)
		{
			InvalidPlateAssignments += (Sample.PlateId < 0 || Sample.PlateId >= Scenario.PlateCount) ? 1 : 0;
			bHasContinental |= (Sample.CrustType == ECrustType::Continental);
			bHasOceanic |= (Sample.CrustType == ECrustType::Oceanic);
		}

		const int32 InitialPlateFloorSamples = ComputeExpectedInitialPlateFloorSamples(Samples.Num(), Plates.Num());
		int32 TotalAssigned = 0;
		int32 EmptyPlates = 0;
		int32 InvalidAxes = 0;
		int32 InvalidAngularSpeeds = 0;
		int32 MinPlateSamples = TNumericLimits<int32>::Max();
		int32 MaxPlateSamples = 0;
		TArray<int32> PlateSizes;
		PlateSizes.Reserve(Plates.Num());
		for (const FPlate& Plate : Plates)
		{
			EmptyPlates += (Plate.SampleIndices.Num() == 0) ? 1 : 0;
			TotalAssigned += Plate.SampleIndices.Num();
			InvalidAxes += !FMath::IsNearlyEqual(Plate.RotationAxis.Size(), 1.0, 1e-8) ? 1 : 0;
			InvalidAngularSpeeds += (Plate.AngularSpeed < (0.5f * 3.14e-2f) || Plate.AngularSpeed > (1.5f * 3.14e-2f)) ? 1 : 0;
			MinPlateSamples = FMath::Min(MinPlateSamples, Plate.SampleIndices.Num());
			MaxPlateSamples = FMath::Max(MaxPlateSamples, Plate.SampleIndices.Num());
			PlateSizes.Add(Plate.SampleIndices.Num());
			TestEqual(
				FString::Printf(TEXT("Initial sample count is recorded for plate %d [%s]"), Plate.Id, *ScenarioLabel),
				Plate.InitialSampleCount,
				Plate.SampleIndices.Num());
			TestTrue(
				FString::Printf(TEXT("Plate %d meets initialization floor [%s]"), Plate.Id, *ScenarioLabel),
				Plate.SampleIndices.Num() >= InitialPlateFloorSamples);
		}
		PlateSizes.Sort();
		const int32 MedianPlateSamples = PlateSizes.Num() > 0 ? PlateSizes[PlateSizes.Num() / 2] : 0;

		UE_LOG(LogTemp, Log,
			TEXT("Reference init [%s]: Plates=%d ContinentalPlates=%d ContinentalArea=%.3f%% TotalAssigned=%d EmptyPlates=%d InvalidAssignments=%d InvalidAxes=%d InvalidSpeeds=%d Min/Median/Max=%d/%d/%d Floor=%d"),
			*ScenarioLabel,
			Plates.Num(),
			Planet.GetContinentalPlateCount(),
			Planet.GetContinentalAreaFraction() * 100.0,
			TotalAssigned,
			EmptyPlates,
			InvalidPlateAssignments,
			InvalidAxes,
			InvalidAngularSpeeds,
			MinPlateSamples,
			MedianPlateSamples,
			MaxPlateSamples,
			InitialPlateFloorSamples);

		TestEqual(FString::Printf(TEXT("Invalid plate assignments [%s]"), *ScenarioLabel), InvalidPlateAssignments, 0);
		TestEqual(FString::Printf(TEXT("All samples assigned to plates [%s]"), *ScenarioLabel), TotalAssigned, Samples.Num());
		TestEqual(FString::Printf(TEXT("Empty plate count [%s]"), *ScenarioLabel), EmptyPlates, 0);
		TestEqual(FString::Printf(TEXT("Invalid rotation axis count [%s]"), *ScenarioLabel), InvalidAxes, 0);
		TestEqual(FString::Printf(TEXT("Invalid angular speed count [%s]"), *ScenarioLabel), InvalidAngularSpeeds, 0);
		TestTrue(FString::Printf(TEXT("Has continental crust [%s]"), *ScenarioLabel), bHasContinental);
		TestTrue(FString::Printf(TEXT("Has oceanic crust [%s]"), *ScenarioLabel), bHasOceanic);
		TestEqual(
			FString::Printf(TEXT("Initial continental plate count [%s]"), *ScenarioLabel),
			Planet.GetContinentalPlateCount(),
			Scenario.Bounds.ExpectedInitialContinentalPlateCount);
		TestTrue(
			FString::Printf(TEXT("Initial continental area fraction within band [%s]"), *ScenarioLabel),
			Planet.GetContinentalAreaFraction() + UE_DOUBLE_SMALL_NUMBER >= Scenario.Bounds.InitialContinentalAreaFractionMin &&
			Planet.GetContinentalAreaFraction() - UE_DOUBLE_SMALL_NUMBER <= Scenario.Bounds.InitialContinentalAreaFractionMax);

		FTectonicPlanet Planet2 = MakePlanetCopy(Scenario.SampleCount);
		Planet2.InitializePlates(Scenario.PlateCount, Scenario.Seed);

		int32 DeterminismMismatches = 0;
		const TArray<FCanonicalSample>& Samples2 = Planet2.GetSamples();
		for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
		{
			DeterminismMismatches +=
				(Samples2[SampleIndex].PlateId != Samples[SampleIndex].PlateId ||
				Samples2[SampleIndex].CrustType != Samples[SampleIndex].CrustType) ? 1 : 0;
		}

		UE_LOG(LogTemp, Log, TEXT("Reference init determinism [%s]: mismatches=%d"), *ScenarioLabel, DeterminismMismatches);
		TestEqual(FString::Printf(TEXT("Determinism mismatches [%s]"), *ScenarioLabel), DeterminismMismatches, 0);
		TestEqual(
			FString::Printf(TEXT("Deterministic continental plate count [%s]"), *ScenarioLabel),
			Planet2.GetContinentalPlateCount(),
			Planet.GetContinentalPlateCount());
		TestTrue(
			FString::Printf(TEXT("Deterministic continental area fraction [%s]"), *ScenarioLabel),
			FMath::IsNearlyEqual(Planet2.GetContinentalAreaFraction(), Planet.GetContinentalAreaFraction(), UE_DOUBLE_SMALL_NUMBER));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInterpolationCategoricalMajorityVoteTest, "Aurous.TectonicPlanet.InterpolationCategoricalMajorityVote",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FInterpolationCategoricalMajorityVoteTest::RunTest(const FString& Parameters)
{
	TArray<FCanonicalSample> ReadSamples;
	ReadSamples.SetNum(3);
	ReadSamples[0].Position = FVector(1.0, 0.0, 0.0);
	ReadSamples[1].Position = FVector(0.0, 1.0, 0.0);
	ReadSamples[2].Position = FVector(0.0, 0.0, 1.0);
	ReadSamples[0].TerraneId = 101;
	ReadSamples[1].TerraneId = 202;
	ReadSamples[2].TerraneId = 303;

	FDelaunayTriangle Triangle;
	Triangle.V[0] = 0;
	Triangle.V[1] = 1;
	Triangle.V[2] = 2;

	auto MakeCarriedSample = [](
		const int32 SampleIndex,
		const float Elevation,
		const float Thickness,
		const float Age,
		const float OrogenyAge,
		const ECrustType CrustType,
		const EOrogenyType OrogenyType,
		const float CollisionDistanceKm,
		const float CollisionSpeedMmPerYear,
		const int32 CollisionOpposingPlateId,
		const float CollisionInfluenceRadiusKm,
		const bool bIsCollisionFront) -> FCarriedSampleData
	{
		FCarriedSampleData Sample;
		Sample.CanonicalSampleIndex = SampleIndex;
		Sample.Elevation = Elevation;
		Sample.Thickness = Thickness;
		Sample.Age = Age;
		Sample.OrogenyAge = OrogenyAge;
		Sample.CrustType = CrustType;
		Sample.OrogenyType = OrogenyType;
		Sample.RidgeDirection = FVector(1.0, 0.0, 0.0);
		Sample.FoldDirection = FVector(0.0, 1.0, 0.0);
		Sample.CollisionDistanceKm = CollisionDistanceKm;
		Sample.CollisionConvergenceSpeedMmPerYear = CollisionSpeedMmPerYear;
		Sample.CollisionOpposingPlateId = CollisionOpposingPlateId;
		Sample.CollisionInfluenceRadiusKm = CollisionInfluenceRadiusKm;
		Sample.bIsCollisionFront = bIsCollisionFront;
		return Sample;
	};

	{
		const FVector Barycentric(0.60f, 0.25f, 0.15f);
		const FCarriedSampleData V0 = MakeCarriedSample(0, 10.0f, 30.0f, 100.0f, 1.0f, ECrustType::Oceanic, EOrogenyType::None, 11.0f, 12.0f, 7, 13.0f, true);
		const FCarriedSampleData V1 = MakeCarriedSample(1, 20.0f, 40.0f, 200.0f, 2.0f, ECrustType::Continental, EOrogenyType::Andean, 21.0f, 22.0f, 8, 23.0f, false);
		const FCarriedSampleData V2 = MakeCarriedSample(2, 30.0f, 50.0f, 300.0f, 3.0f, ECrustType::Continental, EOrogenyType::Andean, 31.0f, 32.0f, 9, 33.0f, false);
		FCanonicalSample DestSample;
		DestSample.Position = FVector(0.2, 0.3, 0.93).GetSafeNormal();

		FTectonicPlanetTestAccess::InterpolateTriangleFields(ReadSamples, Triangle, V0, V1, V2, Barycentric, DestSample);

		TestTrue(TEXT("Continuous elevation stays barycentric when categorical majority differs from highest weight"), FMath::IsNearlyEqual(DestSample.Elevation, 15.5f, KINDA_SMALL_NUMBER));
		TestTrue(TEXT("Continuous thickness stays barycentric when categorical majority differs from highest weight"), FMath::IsNearlyEqual(DestSample.Thickness, 35.5f, KINDA_SMALL_NUMBER));
		TestEqual(TEXT("Crust type uses majority vote when highest weight disagrees"), DestSample.CrustType, ECrustType::Continental);
		TestEqual(TEXT("Orogeny type uses majority vote when highest weight disagrees"), DestSample.OrogenyType, EOrogenyType::Andean);
		TestEqual(TEXT("Terrane selection remains highest-weight driven"), DestSample.TerraneId, 101);
		TestTrue(TEXT("Collision distance remains highest-weight driven"), FMath::IsNearlyEqual(DestSample.CollisionDistanceKm, 11.0f, KINDA_SMALL_NUMBER));
		TestEqual(TEXT("Collision opposing plate id remains highest-weight driven"), DestSample.CollisionOpposingPlateId, 7);
		TestTrue(TEXT("Collision influence radius remains highest-weight driven"), FMath::IsNearlyEqual(DestSample.CollisionInfluenceRadiusKm, 13.0f, KINDA_SMALL_NUMBER));
	}

	{
		const FVector Barycentric(0.55f, 0.30f, 0.15f);
		const FCarriedSampleData V0 = MakeCarriedSample(0, 40.0f, 60.0f, 400.0f, 4.0f, ECrustType::Oceanic, EOrogenyType::None, 41.0f, 42.0f, 10, 43.0f, true);
		const FCarriedSampleData V1 = MakeCarriedSample(1, 50.0f, 70.0f, 500.0f, 5.0f, ECrustType::Oceanic, EOrogenyType::None, 51.0f, 52.0f, 11, 53.0f, false);
		const FCarriedSampleData V2 = MakeCarriedSample(2, 60.0f, 80.0f, 600.0f, 6.0f, ECrustType::Continental, EOrogenyType::Himalayan, 61.0f, 62.0f, 12, 63.0f, false);
		FCanonicalSample DestSample;
		DestSample.Position = FVector(0.4, 0.2, 0.89).GetSafeNormal();

		FTectonicPlanetTestAccess::InterpolateTriangleFields(ReadSamples, Triangle, V0, V1, V2, Barycentric, DestSample);

		TestTrue(TEXT("Continuous age stays barycentric when highest weight is also the majority"), FMath::IsNearlyEqual(DestSample.Age, 460.0f, KINDA_SMALL_NUMBER));
		TestTrue(TEXT("Continuous orogeny age stays barycentric when highest weight is also the majority"), FMath::IsNearlyEqual(DestSample.OrogenyAge, 4.6f, KINDA_SMALL_NUMBER));
		TestEqual(TEXT("Crust type remains the majority value when highest weight agrees"), DestSample.CrustType, ECrustType::Oceanic);
		TestEqual(TEXT("Orogeny type remains the majority value when highest weight agrees"), DestSample.OrogenyType, EOrogenyType::None);
		TestEqual(TEXT("Dominant-source collision opposing plate id follows highest weight"), DestSample.CollisionOpposingPlateId, 10);
		TestTrue(TEXT("Dominant-source collision radius follows highest weight"), FMath::IsNearlyEqual(DestSample.CollisionInfluenceRadiusKm, 43.0f, KINDA_SMALL_NUMBER));
	}

	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTriangleClassificationTest, "Aurous.TectonicPlanet.TriangleClassification",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FTriangleClassificationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();
	const TArray<FPlate>& Plates = Planet.GetPlates();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();

	int32 TotalInterior = 0;
	int32 InvalidInteriorTriangles = 0;
	int32 InteriorClassificationMismatches = 0;

	TArray<uint8> BoundaryTriangleMask;
	BoundaryTriangleMask.Init(0, Triangles.Num());
	int32 UniqueBoundaryTriangleCount = 0;
	int32 InvalidBoundaryTriangleRefs = 0;

	for (const FPlate& Plate : Plates)
	{
		TotalInterior += Plate.InteriorTriangles.Num();

		for (const int32 TriangleIndex : Plate.InteriorTriangles)
		{
			if (!Triangles.IsValidIndex(TriangleIndex))
			{
				++InvalidInteriorTriangles;
				continue;
			}

			const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
			const bool bInteriorMatches =
				Samples[Triangle.V[0]].PlateId == Plate.Id &&
				Samples[Triangle.V[1]].PlateId == Plate.Id &&
				Samples[Triangle.V[2]].PlateId == Plate.Id;
			InteriorClassificationMismatches += bInteriorMatches ? 0 : 1;
		}

		for (const int32 TriangleIndex : Plate.BoundaryTriangles)
		{
			if (!Triangles.IsValidIndex(TriangleIndex))
			{
				++InvalidBoundaryTriangleRefs;
				continue;
			}

			if (BoundaryTriangleMask[TriangleIndex] == 0)
			{
				BoundaryTriangleMask[TriangleIndex] = 1;
				++UniqueBoundaryTriangleCount;
			}
		}
	}

	int32 BoundaryClassificationMismatches = 0;
	for (int32 TriangleIndex = 0; TriangleIndex < BoundaryTriangleMask.Num(); ++TriangleIndex)
	{
		if (BoundaryTriangleMask[TriangleIndex] == 0)
		{
			continue;
		}

		const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
		const int32 P0 = Samples[Triangle.V[0]].PlateId;
		const int32 P1 = Samples[Triangle.V[1]].PlateId;
		const int32 P2 = Samples[Triangle.V[2]].PlateId;
		const bool bSpansPlates = (P0 != P1) || (P1 != P2) || (P0 != P2);
		BoundaryClassificationMismatches += bSpansPlates ? 0 : 1;
	}

	int32 BoundaryFlagMismatches = 0;
	int32 BoundarySampleCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		bool bHasDifferentNeighbor = false;
		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (Samples[NeighborIndex].PlateId != Samples[SampleIndex].PlateId)
			{
				bHasDifferentNeighbor = true;
				break;
			}
		}

		BoundaryFlagMismatches += (Samples[SampleIndex].bIsBoundary == bHasDifferentNeighbor) ? 0 : 1;
		BoundarySampleCount += Samples[SampleIndex].bIsBoundary ? 1 : 0;
	}

	const int32 TotalTriangles = Triangles.Num();
	const double InteriorTriangleFraction = (TotalTriangles > 0) ? static_cast<double>(TotalInterior) / static_cast<double>(TotalTriangles) : 0.0;
	const double BoundarySampleFraction = (Samples.Num() > 0) ? static_cast<double>(BoundarySampleCount) / static_cast<double>(Samples.Num()) : 0.0;

	UE_LOG(LogTemp, Log, TEXT("Triangle classification metrics (500k): Interior=%d/%d (%.2f%%), BoundaryUnique=%d, BoundarySamples=%d/%d (%.2f%%)"),
		TotalInterior,
		TotalTriangles,
		InteriorTriangleFraction * 100.0,
		UniqueBoundaryTriangleCount,
		BoundarySampleCount,
		Samples.Num(),
		BoundarySampleFraction * 100.0);
	UE_LOG(LogTemp, Log, TEXT("Triangle classification integrity (500k): InvalidInteriorRefs=%d InvalidBoundaryRefs=%d InteriorMismatch=%d BoundaryMismatch=%d BoundaryFlagMismatch=%d"),
		InvalidInteriorTriangles,
		InvalidBoundaryTriangleRefs,
		InteriorClassificationMismatches,
		BoundaryClassificationMismatches,
		BoundaryFlagMismatches);

	TestEqual(TEXT("Interior + boundary = total triangles"), TotalInterior + UniqueBoundaryTriangleCount, TotalTriangles);
	TestEqual(TEXT("Invalid interior triangle refs"), InvalidInteriorTriangles, 0);
	TestEqual(TEXT("Invalid boundary triangle refs"), InvalidBoundaryTriangleRefs, 0);
	TestEqual(TEXT("Interior classification mismatches"), InteriorClassificationMismatches, 0);
	TestEqual(TEXT("Boundary classification mismatches"), BoundaryClassificationMismatches, 0);
	TestEqual(TEXT("Boundary flag mismatches"), BoundaryFlagMismatches, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundaryClassificationTest, "Aurous.TectonicPlanet.BoundaryClassification",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FBoundaryClassificationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();

	int32 BoundarySampleCount = 0;
	int32 MissingBoundaryNormal = 0;
	int32 NonTangentBoundaryNormal = 0;
	int32 CrossDirectionMismatch = 0;
	int32 InteriorDirectionMismatch = 0;
	int32 BoundaryTypeNoneCount = 0;
	int32 BoundaryTypeMismatch = 0;
	int32 ConvergentCount = 0;
	int32 DivergentCount = 0;
	int32 TransformCount = 0;

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = Samples[SampleIndex];
		if (!Sample.bIsBoundary)
		{
			continue;
		}

		++BoundarySampleCount;
		const FVector Position = Sample.Position.GetSafeNormal();
		if (Sample.BoundaryNormal.IsNearlyZero())
		{
			++MissingBoundaryNormal;
			continue;
		}

		const FVector BoundaryNormal = Sample.BoundaryNormal.GetSafeNormal();
		const double TangencyAbsDot = FMath::Abs(FVector::DotProduct(BoundaryNormal, Position));
		NonTangentBoundaryNormal += (TangencyAbsDot > 1e-4) ? 1 : 0;

		FVector MeanCrossDirection = FVector::ZeroVector;
		FVector MeanInteriorDirection = FVector::ZeroVector;
		int32 CrossPlateIds[16];
		int32 CrossPlateCounts[16];
		int32 NumCrossPlates = 0;
		for (int32 SlotIndex = 0; SlotIndex < 16; ++SlotIndex)
		{
			CrossPlateIds[SlotIndex] = INDEX_NONE;
			CrossPlateCounts[SlotIndex] = 0;
		}

		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!Samples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const FCanonicalSample& Neighbor = Samples[NeighborIndex];
			const FVector Direction = (Neighbor.Position - Sample.Position).GetSafeNormal();
			if (Direction.IsNearlyZero())
			{
				continue;
			}

			if (Neighbor.PlateId != Sample.PlateId)
			{
				MeanCrossDirection += Direction;
				int32 ExistingSlot = INDEX_NONE;
				for (int32 SlotIndex = 0; SlotIndex < NumCrossPlates; ++SlotIndex)
				{
					if (CrossPlateIds[SlotIndex] == Neighbor.PlateId)
					{
						ExistingSlot = SlotIndex;
						break;
					}
				}

				if (ExistingSlot != INDEX_NONE)
				{
					++CrossPlateCounts[ExistingSlot];
				}
				else if (NumCrossPlates < 16)
				{
					CrossPlateIds[NumCrossPlates] = Neighbor.PlateId;
					CrossPlateCounts[NumCrossPlates] = 1;
					++NumCrossPlates;
				}
			}
			else
			{
				MeanInteriorDirection += Direction;
			}
		}

		const FVector CrossTangent = MeanCrossDirection - FVector::DotProduct(MeanCrossDirection, Position) * Position;
		if (!CrossTangent.IsNearlyZero())
		{
			const double Alignment = FVector::DotProduct(BoundaryNormal, CrossTangent.GetSafeNormal());
			CrossDirectionMismatch += (Alignment < 0.5) ? 1 : 0;
		}

		const FVector InteriorTangent = MeanInteriorDirection - FVector::DotProduct(MeanInteriorDirection, Position) * Position;
		if (!InteriorTangent.IsNearlyZero())
		{
			const double InteriorAlignment = FVector::DotProduct(BoundaryNormal, InteriorTangent.GetSafeNormal());
			InteriorDirectionMismatch += (InteriorAlignment > 0.0) ? 1 : 0;
		}

		switch (Sample.BoundaryType)
		{
		case EBoundaryType::Convergent:
			++ConvergentCount;
			break;
		case EBoundaryType::Divergent:
			++DivergentCount;
			break;
		case EBoundaryType::Transform:
			++TransformCount;
			break;
		case EBoundaryType::None:
		default:
			++BoundaryTypeNoneCount;
			break;
		}

		if (NumCrossPlates > 0)
		{
			int32 DominantCrossPlate = CrossPlateIds[0];
			int32 DominantCount = CrossPlateCounts[0];
			for (int32 SlotIndex = 1; SlotIndex < NumCrossPlates; ++SlotIndex)
			{
				if (CrossPlateCounts[SlotIndex] > DominantCount ||
					(CrossPlateCounts[SlotIndex] == DominantCount && CrossPlateIds[SlotIndex] < DominantCrossPlate))
				{
					DominantCrossPlate = CrossPlateIds[SlotIndex];
					DominantCount = CrossPlateCounts[SlotIndex];
				}
			}

			const FVector RelativeVelocity = Planet.ComputeSurfaceVelocity(Sample.PlateId, Position) - Planet.ComputeSurfaceVelocity(DominantCrossPlate, Position);
			const double RelativeSpeed = RelativeVelocity.Size();
			const double NormalComponent = FVector::DotProduct(RelativeVelocity, BoundaryNormal);
			EBoundaryType ExpectedType = EBoundaryType::Transform;
			if (RelativeSpeed > 1e-12)
			{
				if (FMath::Abs(NormalComponent) < 0.3 * RelativeSpeed)
				{
					ExpectedType = EBoundaryType::Transform;
				}
				else
				{
					ExpectedType = (NormalComponent < 0.0) ? EBoundaryType::Convergent : EBoundaryType::Divergent;
				}
			}

			BoundaryTypeMismatch += (Sample.BoundaryType == ExpectedType) ? 0 : 1;
		}
	}

	Planet.StepSimulation();
	const TArray<FCanonicalSample>& ReconciledSamples = Planet.GetSamples();
	int32 GapSampleCount = 0;
	int32 MissingFlankingA = 0;
	int32 InvalidFlankingPair = 0;
	for (int32 SampleIndex = 0; SampleIndex < ReconciledSamples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = ReconciledSamples[SampleIndex];
		if (!Sample.bGapDetected)
		{
			continue;
		}

		++GapSampleCount;
		if (Sample.FlankingPlateIdA < 0 || Sample.FlankingPlateIdA >= TestPlateCount)
		{
			++MissingFlankingA;
			continue;
		}

		int32 UniqueNeighborPlateCount = 0;
		int32 UniqueNeighborPlates[16];
		for (int32 SlotIndex = 0; SlotIndex < 16; ++SlotIndex)
		{
			UniqueNeighborPlates[SlotIndex] = INDEX_NONE;
		}
		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!ReconciledSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = ReconciledSamples[NeighborIndex].PlateId;
			if (NeighborPlateId < 0 || NeighborPlateId >= TestPlateCount)
			{
				continue;
			}

			bool bAlreadyAdded = false;
			for (int32 SlotIndex = 0; SlotIndex < UniqueNeighborPlateCount; ++SlotIndex)
			{
				if (UniqueNeighborPlates[SlotIndex] == NeighborPlateId)
				{
					bAlreadyAdded = true;
					break;
				}
			}
			if (!bAlreadyAdded && UniqueNeighborPlateCount < 16)
			{
				UniqueNeighborPlates[UniqueNeighborPlateCount++] = NeighborPlateId;
			}
		}

		if (UniqueNeighborPlateCount >= 2)
		{
			const bool bValidPair =
				Sample.FlankingPlateIdB >= 0 &&
				Sample.FlankingPlateIdB < TestPlateCount &&
				Sample.FlankingPlateIdA != Sample.FlankingPlateIdB;
			InvalidFlankingPair += bValidPair ? 0 : 1;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Boundary classification metrics (500k): Boundary=%d MissingNormal=%d NonTangent=%d CrossMismatch=%d InteriorMismatch=%d TypeNone=%d TypeMismatch=%d Cvg=%d Div=%d Trn=%d"),
		BoundarySampleCount,
		MissingBoundaryNormal,
		NonTangentBoundaryNormal,
		CrossDirectionMismatch,
		InteriorDirectionMismatch,
		BoundaryTypeNoneCount,
		BoundaryTypeMismatch,
		ConvergentCount,
		DivergentCount,
		TransformCount);
	UE_LOG(LogTemp, Log, TEXT("Gap flanking metrics (500k): Gap=%d MissingA=%d InvalidPair=%d"),
		GapSampleCount,
		MissingFlankingA,
		InvalidFlankingPair);

	TestTrue(TEXT("Boundary samples exist"), BoundarySampleCount > 0);
	TestEqual(TEXT("Missing boundary normals"), MissingBoundaryNormal, 0);
	TestEqual(TEXT("Boundary normals tangent to surface"), NonTangentBoundaryNormal, 0);
	TestTrue(TEXT("Boundary normals align with cross-plate direction"), CrossDirectionMismatch <= FMath::Max(1, BoundarySampleCount / 100));
	TestTrue(TEXT("Boundary normals point away from plate interior"), InteriorDirectionMismatch <= FMath::Max(1, BoundarySampleCount / 100));
	TestEqual(TEXT("Boundary type none count for boundary samples"), BoundaryTypeNoneCount, 0);
	TestEqual(TEXT("Boundary type matches velocity rule"), BoundaryTypeMismatch, 0);
	TestTrue(TEXT("At least one boundary type populated"), (ConvergentCount + DivergentCount + TransformCount) > 0);
	TestTrue(TEXT("Gap samples exist after reconcile"), GapSampleCount > 0);
	TestEqual(TEXT("Gap samples have primary flanking plate"), MissingFlankingA, 0);
	TestEqual(TEXT("Gap samples with 2+ neighboring plates have valid flanking pair"), InvalidFlankingPair, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundaryClassificationKnownPairTest, "Aurous.TectonicPlanet.BoundaryClassificationKnownPair",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FBoundaryClassificationKnownPairTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& InitialSamples = Planet.GetSamples();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();
	int32 TargetSampleIndex = INDEX_NONE;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	for (int32 SampleIndex = 0; SampleIndex < InitialSamples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = InitialSamples[SampleIndex];
		if (!Sample.bIsBoundary || Sample.PlateId < 0 || Sample.PlateId >= TestPlateCount)
		{
			continue;
		}

		int32 UniqueCrossPlates[8];
		int32 NumUniqueCrossPlates = 0;
		for (int32 SlotIndex = 0; SlotIndex < 8; ++SlotIndex)
		{
			UniqueCrossPlates[SlotIndex] = INDEX_NONE;
		}

		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InitialSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = InitialSamples[NeighborIndex].PlateId;
			if (NeighborPlateId < 0 || NeighborPlateId >= TestPlateCount || NeighborPlateId == Sample.PlateId)
			{
				continue;
			}

			bool bAlreadyAdded = false;
			for (int32 SlotIndex = 0; SlotIndex < NumUniqueCrossPlates; ++SlotIndex)
			{
				if (UniqueCrossPlates[SlotIndex] == NeighborPlateId)
				{
					bAlreadyAdded = true;
					break;
				}
			}
			if (!bAlreadyAdded && NumUniqueCrossPlates < 8)
			{
				UniqueCrossPlates[NumUniqueCrossPlates++] = NeighborPlateId;
			}
		}

		if (NumUniqueCrossPlates == 1)
		{
			TargetSampleIndex = SampleIndex;
			PlateA = Sample.PlateId;
			PlateB = UniqueCrossPlates[0];
			break;
		}
	}

	TestTrue(TEXT("Found boundary sample with single opposing plate"), TargetSampleIndex != INDEX_NONE);
	if (TargetSampleIndex == INDEX_NONE)
	{
		return false;
	}

	const FVector P = InitialSamples[TargetSampleIndex].Position.GetSafeNormal();
	FVector N = InitialSamples[TargetSampleIndex].BoundaryNormal.GetSafeNormal();
	if (N.IsNearlyZero())
	{
		for (const int32 NeighborIndex : Adjacency[TargetSampleIndex])
		{
			if (!InitialSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}
			if (InitialSamples[NeighborIndex].PlateId != PlateA)
			{
				N += (InitialSamples[NeighborIndex].Position - InitialSamples[TargetSampleIndex].Position).GetSafeNormal();
			}
		}
		N = (N - FVector::DotProduct(N, P) * P).GetSafeNormal();
	}
	TestFalse(TEXT("Boundary normal valid"), N.IsNearlyZero());
	if (N.IsNearlyZero())
	{
		return false;
	}

	FVector Axis = FVector::CrossProduct(P, N).GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		Axis = FVector::CrossProduct(P, FVector::UpVector).GetSafeNormal();
	}
	TestFalse(TEXT("Constructed rotation axis valid"), Axis.IsNearlyZero());
	if (Axis.IsNearlyZero())
	{
		return false;
	}

	TArray<FPlate>& MutablePlates = Planet.GetPlates();
	for (FPlate& Plate : MutablePlates)
	{
		Plate.AngularSpeed = 0.0f;
	}
	MutablePlates[PlateA].RotationAxis = Axis;
	MutablePlates[PlateA].AngularSpeed = 0.05f;
	MutablePlates[PlateB].RotationAxis = -Axis;
	MutablePlates[PlateB].AngularSpeed = 0.05f;

	Planet.ClassifyTriangles();
	const FCanonicalSample& UpdatedSample = Planet.GetSamples()[TargetSampleIndex];
	const FVector RelativeVelocity = Planet.ComputeSurfaceVelocity(PlateA, P) - Planet.ComputeSurfaceVelocity(PlateB, P);
	const double RelativeAlongBoundary = FVector::DotProduct(RelativeVelocity, UpdatedSample.BoundaryNormal.GetSafeNormal());

	UE_LOG(LogTemp, Log, TEXT("Boundary known-pair metrics (500k): sample=%d plateA=%d plateB=%d relAlongNormal=%.6e type=%d"),
		TargetSampleIndex,
		PlateA,
		PlateB,
		RelativeAlongBoundary,
		static_cast<int32>(UpdatedSample.BoundaryType));

	TestTrue(TEXT("Relative velocity points along boundary normal for separating pair"), RelativeAlongBoundary > 0.0);
	TestEqual(TEXT("Known separating pair classified as divergent"), UpdatedSample.BoundaryType, EBoundaryType::Divergent);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGapOceanicGenerationTest, "Aurous.TectonicPlanet.GapOceanicGeneration",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundaryClassificationKnownConvergentPairTest, "Aurous.TectonicPlanet.BoundaryClassificationKnownConvergentPair",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FBoundaryClassificationKnownConvergentPairTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& InitialSamples = Planet.GetSamples();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();
	int32 TargetSampleIndex = INDEX_NONE;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	for (int32 SampleIndex = 0; SampleIndex < InitialSamples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = InitialSamples[SampleIndex];
		if (!Sample.bIsBoundary || Sample.PlateId < 0 || Sample.PlateId >= TestPlateCount)
		{
			continue;
		}

		int32 UniqueCrossPlates[8];
		int32 NumUniqueCrossPlates = 0;
		for (int32 SlotIndex = 0; SlotIndex < 8; ++SlotIndex)
		{
			UniqueCrossPlates[SlotIndex] = INDEX_NONE;
		}

		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InitialSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = InitialSamples[NeighborIndex].PlateId;
			if (NeighborPlateId < 0 || NeighborPlateId >= TestPlateCount || NeighborPlateId == Sample.PlateId)
			{
				continue;
			}

			bool bAlreadyAdded = false;
			for (int32 SlotIndex = 0; SlotIndex < NumUniqueCrossPlates; ++SlotIndex)
			{
				if (UniqueCrossPlates[SlotIndex] == NeighborPlateId)
				{
					bAlreadyAdded = true;
					break;
				}
			}
			if (!bAlreadyAdded && NumUniqueCrossPlates < 8)
			{
				UniqueCrossPlates[NumUniqueCrossPlates++] = NeighborPlateId;
			}
		}

		if (NumUniqueCrossPlates == 1)
		{
			TargetSampleIndex = SampleIndex;
			PlateA = Sample.PlateId;
			PlateB = UniqueCrossPlates[0];
			break;
		}
	}

	TestTrue(TEXT("Found boundary sample with single opposing plate"), TargetSampleIndex != INDEX_NONE);
	if (TargetSampleIndex == INDEX_NONE)
	{
		return false;
	}

	const FVector P = InitialSamples[TargetSampleIndex].Position.GetSafeNormal();
	FVector N = InitialSamples[TargetSampleIndex].BoundaryNormal.GetSafeNormal();
	if (N.IsNearlyZero())
	{
		for (const int32 NeighborIndex : Adjacency[TargetSampleIndex])
		{
			if (!InitialSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}
			if (InitialSamples[NeighborIndex].PlateId != PlateA)
			{
				N += (InitialSamples[NeighborIndex].Position - InitialSamples[TargetSampleIndex].Position).GetSafeNormal();
			}
		}
		N = (N - FVector::DotProduct(N, P) * P).GetSafeNormal();
	}
	TestFalse(TEXT("Boundary normal valid"), N.IsNearlyZero());
	if (N.IsNearlyZero())
	{
		return false;
	}

	FVector Axis = FVector::CrossProduct(P, N).GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		Axis = FVector::CrossProduct(P, FVector::UpVector).GetSafeNormal();
	}
	TestFalse(TEXT("Constructed rotation axis valid"), Axis.IsNearlyZero());
	if (Axis.IsNearlyZero())
	{
		return false;
	}

	TArray<FPlate>& MutablePlates = Planet.GetPlates();
	for (FPlate& Plate : MutablePlates)
	{
		Plate.AngularSpeed = 0.0f;
	}
	MutablePlates[PlateA].RotationAxis = -Axis;
	MutablePlates[PlateA].AngularSpeed = 0.05f;
	MutablePlates[PlateB].RotationAxis = Axis;
	MutablePlates[PlateB].AngularSpeed = 0.05f;

	Planet.ClassifyTriangles();
	const FCanonicalSample& UpdatedSample = Planet.GetSamples()[TargetSampleIndex];
	const FVector RelativeVelocity = Planet.ComputeSurfaceVelocity(PlateA, P) - Planet.ComputeSurfaceVelocity(PlateB, P);
	const double RelativeAlongBoundary = FVector::DotProduct(RelativeVelocity, UpdatedSample.BoundaryNormal.GetSafeNormal());

	TestTrue(TEXT("Relative velocity points against boundary normal for converging pair"), RelativeAlongBoundary < 0.0);
	TestEqual(TEXT("Known converging pair classified as convergent"), UpdatedSample.BoundaryType, EBoundaryType::Convergent);

	return true;
}

bool FGapOceanicGenerationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& InitialSamples = Planet.GetSamples();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();

	int32 TargetSampleIndex = INDEX_NONE;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	for (int32 SampleIndex = 0; SampleIndex < InitialSamples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = InitialSamples[SampleIndex];
		if (!Sample.bIsBoundary || Sample.PlateId < 0 || Sample.PlateId >= TestPlateCount)
		{
			continue;
		}

		int32 UniqueCrossPlates[8];
		int32 NumUniqueCrossPlates = 0;
		for (int32 SlotIndex = 0; SlotIndex < 8; ++SlotIndex)
		{
			UniqueCrossPlates[SlotIndex] = INDEX_NONE;
		}

		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InitialSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = InitialSamples[NeighborIndex].PlateId;
			if (NeighborPlateId < 0 || NeighborPlateId >= TestPlateCount || NeighborPlateId == Sample.PlateId)
			{
				continue;
			}

			bool bAlreadyAdded = false;
			for (int32 SlotIndex = 0; SlotIndex < NumUniqueCrossPlates; ++SlotIndex)
			{
				if (UniqueCrossPlates[SlotIndex] == NeighborPlateId)
				{
					bAlreadyAdded = true;
					break;
				}
			}
			if (!bAlreadyAdded && NumUniqueCrossPlates < 8)
			{
				UniqueCrossPlates[NumUniqueCrossPlates++] = NeighborPlateId;
			}
		}

		if (NumUniqueCrossPlates == 1)
		{
			TargetSampleIndex = SampleIndex;
			PlateA = Sample.PlateId;
			PlateB = UniqueCrossPlates[0];
			break;
		}
	}

	TestTrue(TEXT("Found boundary sample for divergent gap setup"), TargetSampleIndex != INDEX_NONE);
	if (TargetSampleIndex == INDEX_NONE)
	{
		return false;
	}

	const FVector P = InitialSamples[TargetSampleIndex].Position.GetSafeNormal();
	FVector N = InitialSamples[TargetSampleIndex].BoundaryNormal.GetSafeNormal();
	if (N.IsNearlyZero())
	{
		for (const int32 NeighborIndex : Adjacency[TargetSampleIndex])
		{
			if (!InitialSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}
			if (InitialSamples[NeighborIndex].PlateId != PlateA)
			{
				N += (InitialSamples[NeighborIndex].Position - InitialSamples[TargetSampleIndex].Position).GetSafeNormal();
			}
		}
		N = (N - FVector::DotProduct(N, P) * P).GetSafeNormal();
	}
	TestFalse(TEXT("Boundary normal valid for divergent setup"), N.IsNearlyZero());
	if (N.IsNearlyZero())
	{
		return false;
	}

	FVector Axis = FVector::CrossProduct(P, N).GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		Axis = FVector::CrossProduct(P, FVector::UpVector).GetSafeNormal();
	}
	TestFalse(TEXT("Constructed divergent axis valid"), Axis.IsNearlyZero());
	if (Axis.IsNearlyZero())
	{
		return false;
	}

	TArray<FPlate>& MutablePlates = Planet.GetPlates();
	for (FPlate& Plate : MutablePlates)
	{
		Plate.AngularSpeed = 0.0f;
	}
	MutablePlates[PlateA].RotationAxis = Axis;
	MutablePlates[PlateA].AngularSpeed = 0.05f;
	MutablePlates[PlateB].RotationAxis = -Axis;
	MutablePlates[PlateB].AngularSpeed = 0.05f;

	Planet.ClassifyTriangles();
	for (int32 StepIndex = 0; StepIndex < 3; ++StepIndex)
	{
		Planet.StepSimulation();
	}

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	constexpr float RidgeTemplateElevationKm = -1.0f;

	auto BuildLocalNeighborhood = [&Adjacency, &Samples](const int32 CenterIndex, TArray<int32, TInlineAllocator<96>>& OutNeighborhood)
	{
		OutNeighborhood.Reset();
		for (const int32 NeighborIndex : Adjacency[CenterIndex])
		{
			if (Samples.IsValidIndex(NeighborIndex))
			{
				OutNeighborhood.AddUnique(NeighborIndex);
			}
		}

		const int32 FirstRingCount = OutNeighborhood.Num();
		for (int32 RingIndex = 0; RingIndex < FirstRingCount; ++RingIndex)
		{
			const int32 NeighborIndex = OutNeighborhood[RingIndex];
			if (!Adjacency.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			for (const int32 SecondRingIndex : Adjacency[NeighborIndex])
			{
				if (SecondRingIndex != CenterIndex && Samples.IsValidIndex(SecondRingIndex))
				{
					OutNeighborhood.AddUnique(SecondRingIndex);
				}
			}
		}
	};

	auto FindNearestFlankBorder = [&Samples](const TArray<int32, TInlineAllocator<96>>& Neighborhood, const FVector& Position, const int32 TargetPlateId, const bool bRequireBoundary, int32& OutSampleIndex, double& OutDistSq)
	{
		OutSampleIndex = INDEX_NONE;
		OutDistSq = TNumericLimits<double>::Max();
		if (TargetPlateId == INDEX_NONE)
		{
			return;
		}

		for (const int32 CandidateIndex : Neighborhood)
		{
			if (!Samples.IsValidIndex(CandidateIndex))
			{
				continue;
			}

			const FCanonicalSample& Candidate = Samples[CandidateIndex];
			if (Candidate.PlateId != TargetPlateId || Candidate.bGapDetected)
			{
				continue;
			}
			if (bRequireBoundary && !Candidate.bIsBoundary)
			{
				continue;
			}

			const double DistSq = FVector::DistSquared(Position, Candidate.Position);
			if (DistSq < OutDistSq)
			{
				OutDistSq = DistSq;
				OutSampleIndex = CandidateIndex;
			}
		}
	};

	int32 GapSampleCount = 0;
	int32 OceanicMismatchCount = 0;
	int32 AgeMismatchCount = 0;
	int32 ThicknessMismatchCount = 0;
	int32 FormulaCheckedCount = 0;
	int32 ElevationFormulaMismatchCount = 0;
	int32 ElevationBlendBoundViolationCount = 0;
	int32 AssignedPlateMismatchCount = 0;
	int32 AssignedPlateOutsideFlanksCount = 0;
	int32 RidgeZeroCount = 0;
	int32 RidgeTangentMismatchCount = 0;
	int32 RidgePerpendicularMismatchCount = 0;

	TArray<int32, TInlineAllocator<96>> Neighborhood;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = Samples[SampleIndex];
		if (!Sample.bGapDetected)
		{
			continue;
		}

		++GapSampleCount;
		OceanicMismatchCount += (Sample.CrustType == ECrustType::Oceanic) ? 0 : 1;
		AgeMismatchCount += FMath::IsNearlyEqual(Sample.Age, 0.0f, 1e-6f) ? 0 : 1;
		ThicknessMismatchCount += FMath::IsNearlyEqual(Sample.Thickness, 7.0f, 1e-6f) ? 0 : 1;

		const int32 FlankA = Sample.FlankingPlateIdA;
		const int32 FlankB = Sample.FlankingPlateIdB;
		if (FlankA < 0 || FlankA >= TestPlateCount || FlankB < 0 || FlankB >= TestPlateCount || FlankA == FlankB)
		{
			continue;
		}

		const FVector Position = Sample.Position.GetSafeNormal();
		if (Position.IsNearlyZero())
		{
			continue;
		}

		BuildLocalNeighborhood(SampleIndex, Neighborhood);
		int32 BorderA = INDEX_NONE;
		int32 BorderB = INDEX_NONE;
		double DistSqA = TNumericLimits<double>::Max();
		double DistSqB = TNumericLimits<double>::Max();
		FindNearestFlankBorder(Neighborhood, Position, FlankA, true, BorderA, DistSqA);
		FindNearestFlankBorder(Neighborhood, Position, FlankB, true, BorderB, DistSqB);
		if (!Samples.IsValidIndex(BorderA))
		{
			FindNearestFlankBorder(Neighborhood, Position, FlankA, false, BorderA, DistSqA);
		}
		if (!Samples.IsValidIndex(BorderB))
		{
			FindNearestFlankBorder(Neighborhood, Position, FlankB, false, BorderB, DistSqB);
		}
		if (!Samples.IsValidIndex(BorderA) || !Samples.IsValidIndex(BorderB))
		{
			continue;
		}

		++FormulaCheckedCount;
		const double DistA = FMath::Sqrt(FMath::Max(0.0, DistSqA));
		const double DistB = FMath::Sqrt(FMath::Max(0.0, DistSqB));
		const double WeightA = 1.0 / FMath::Max(1e-6, DistA);
		const double WeightB = 1.0 / FMath::Max(1e-6, DistB);
		const double WeightSum = WeightA + WeightB;
		const float BorderElevation = (WeightSum > UE_DOUBLE_SMALL_NUMBER)
			? static_cast<float>((WeightA * Samples[BorderA].Elevation + WeightB * Samples[BorderB].Elevation) / WeightSum)
			: 0.5f * (Samples[BorderA].Elevation + Samples[BorderB].Elevation);
		const double DistanceToRidge = 0.5 * FMath::Abs(DistA - DistB);
		const double DistanceToBorder = FMath::Min(DistA, DistB);
		const double AlphaDenominator = DistanceToRidge + DistanceToBorder;
		const double Alpha = (AlphaDenominator > UE_DOUBLE_SMALL_NUMBER)
			? FMath::Clamp(DistanceToRidge / AlphaDenominator, 0.0, 1.0)
			: 0.0;
		const float ExpectedElevation = static_cast<float>(Alpha * static_cast<double>(BorderElevation) + (1.0 - Alpha) * static_cast<double>(RidgeTemplateElevationKm));
		ElevationFormulaMismatchCount += FMath::IsNearlyEqual(Sample.Elevation, ExpectedElevation, 1e-3f) ? 0 : 1;
		const float ElevationMin = FMath::Min(BorderElevation, RidgeTemplateElevationKm) - 0.25f;
		const float ElevationMax = FMath::Max(BorderElevation, RidgeTemplateElevationKm) + 0.25f;
		ElevationBlendBoundViolationCount += (Sample.Elevation >= ElevationMin && Sample.Elevation <= ElevationMax) ? 0 : 1;

		const int32 ExpectedPlate = (DistSqA <= DistSqB) ? FlankA : FlankB;
		AssignedPlateMismatchCount += (Sample.PlateId == ExpectedPlate) ? 0 : 1;
		AssignedPlateOutsideFlanksCount += (Sample.PlateId == FlankA || Sample.PlateId == FlankB) ? 0 : 1;

		const FVector RidgeDirection = Sample.RidgeDirection.GetSafeNormal();
		if (RidgeDirection.IsNearlyZero())
		{
			++RidgeZeroCount;
			continue;
		}

		const double Tangency = FMath::Abs(FVector::DotProduct(RidgeDirection, Position));
		RidgeTangentMismatchCount += (Tangency > 1e-4) ? 1 : 0;

		const FVector RelativeVelocity = Planet.ComputeSurfaceVelocity(FlankA, Position) - Planet.ComputeSurfaceVelocity(FlankB, Position);
		const double RelativeSpeed = RelativeVelocity.Size();
		if (RelativeSpeed > 1e-8)
		{
			const double Alignment = FMath::Abs(FVector::DotProduct(RidgeDirection, RelativeVelocity / RelativeSpeed));
			RidgePerpendicularMismatchCount += (Alignment > 0.25) ? 1 : 0;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Gap oceanic generation metrics (500k): Gap=%d OceanicMismatch=%d AgeMismatch=%d ThicknessMismatch=%d FormulaChecked=%d ElevMismatch=%d ElevBlendBoundViolation=%d PlateMismatch=%d PlateOutsideFlanks=%d RidgeZero=%d RidgeTangentMismatch=%d RidgePerpMismatch=%d"),
		GapSampleCount,
		OceanicMismatchCount,
		AgeMismatchCount,
		ThicknessMismatchCount,
		FormulaCheckedCount,
		ElevationFormulaMismatchCount,
		ElevationBlendBoundViolationCount,
		AssignedPlateMismatchCount,
		AssignedPlateOutsideFlanksCount,
		RidgeZeroCount,
		RidgeTangentMismatchCount,
		RidgePerpendicularMismatchCount);

	TestTrue(TEXT("Gap samples exist after divergent stepping"), GapSampleCount > 0);
	TestEqual(TEXT("Gap samples initialized as oceanic crust"), OceanicMismatchCount, 0);
	TestEqual(TEXT("Gap sample age reset to zero"), AgeMismatchCount, 0);
	TestEqual(TEXT("Gap sample thickness initialized to oceanic thickness"), ThicknessMismatchCount, 0);
	TestTrue(TEXT("Gap elevation formula evaluated on at least one sample"), FormulaCheckedCount > 0);
	TestEqual(TEXT("Gap elevations stay within ridge/border blend bounds"), ElevationBlendBoundViolationCount, 0);
	TestTrue(TEXT("At least one gap sample assigns to the nearer flanking plate"),
		(FormulaCheckedCount - AssignedPlateMismatchCount) > 0);
	TestEqual(TEXT("Ridge direction is tangent"), RidgeTangentMismatchCount, 0);
	TestTrue(TEXT("Ridge direction mostly perpendicular to relative motion"), RidgePerpendicularMismatchCount <= FMath::Max(1, FormulaCheckedCount / 50));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPhase4ArtifactGapResolutionTest, "Aurous.TectonicPlanet.Phase4ArtifactGapResolution",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPhase4ArtifactGapResolutionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(4);
	Adjacency.SetNum(4);
	Plates.SetNum(2);
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		Plates[PlateIndex].Id = PlateIndex;
	}

	Samples[0].Position = FVector(1.0, 0.0, 0.0).GetSafeNormal();
	Samples[0].PlateId = 1;
	Samples[0].PrevPlateId = 0;
	Samples[0].bGapDetected = true;
	Samples[0].CrustType = ECrustType::Oceanic;
	Samples[0].Elevation = -6.0f;
	Samples[0].Thickness = 7.0f;
	Samples[0].Age = 10.0f;

	Samples[1].Position = FVector(1.0, 0.02, 0.0).GetSafeNormal();
	Samples[1].PlateId = 0;
	Samples[1].PrevPlateId = 0;
	Samples[1].CrustType = ECrustType::Continental;
	Samples[1].Elevation = 1.5f;
	Samples[1].Thickness = 33.0f;
	Samples[1].Age = 120.0f;
	Samples[1].OrogenyType = EOrogenyType::Himalayan;

	Samples[2].Position = FVector(1.0, -0.08, 0.0).GetSafeNormal();
	Samples[2].PlateId = 0;
	Samples[2].PrevPlateId = 0;
	Samples[2].CrustType = ECrustType::Continental;
	Samples[2].Elevation = 0.5f;
	Samples[2].Thickness = 30.0f;
	Samples[2].Age = 80.0f;

	Samples[3].Position = FVector(0.99, 0.0, 0.12).GetSafeNormal();
	Samples[3].PlateId = 0;
	Samples[3].PrevPlateId = 0;
	Samples[3].CrustType = ECrustType::Continental;
	Samples[3].Elevation = 0.25f;
	Samples[3].Thickness = 28.0f;
	Samples[3].Age = 60.0f;

	Adjacency[0] = { 1, 2, 3 };
	Adjacency[1] = { 0, 2 };
	Adjacency[2] = { 0, 1, 3 };
	Adjacency[3] = { 0, 2 };

	TArray<uint8> GapFlags;
	GapFlags.Init(0, Samples.Num());
	GapFlags[0] = 1;

	bool bDivergentGapCreated = false;
	const bool bResolved = FTectonicPlanetTestAccess::ResolveGapSamplePhase4(Planet, 0, GapFlags, Samples, bDivergentGapCreated);
	TestTrue(TEXT("Artifact gap resolves successfully"), bResolved);
	TestFalse(TEXT("Artifact gap does not create divergent oceanic crust"), bDivergentGapCreated);
	TestEqual(TEXT("Artifact gap assigned to surrounding plate"), Samples[0].PlateId, 0);
	TestFalse(TEXT("Artifact gap flag cleared"), Samples[0].bGapDetected);
	TestEqual(TEXT("Artifact gap copies crust type from nearest donor"), Samples[0].CrustType, Samples[1].CrustType);
	TestTrue(TEXT("Artifact gap copies nearest donor elevation"), FMath::IsNearlyEqual(Samples[0].Elevation, Samples[1].Elevation, 1e-6f));
	TestTrue(TEXT("Artifact gap copies nearest donor thickness"), FMath::IsNearlyEqual(Samples[0].Thickness, Samples[1].Thickness, 1e-6f));
	TestTrue(TEXT("Artifact gap copies nearest donor age"), FMath::IsNearlyEqual(Samples[0].Age, Samples[1].Age, 1e-6f));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPhase4DivergentGapResolutionTest, "Aurous.TectonicPlanet.Phase4DivergentGapResolution",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPhase4DivergentGapResolutionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(3);
	Adjacency.SetNum(3);
	Plates.SetNum(2);
	Plates[0].Id = 0;
	Plates[0].RotationAxis = FVector::UpVector;
	Plates[0].AngularSpeed = 1.0f;
	Plates[1].Id = 1;
	Plates[1].RotationAxis = -FVector::UpVector;
	Plates[1].AngularSpeed = 1.0f;

	Samples[0].Position = FVector(1.0, 0.0, 0.0).GetSafeNormal();
	Samples[0].PlateId = 0;
	Samples[0].PrevPlateId = 0;
	Samples[0].bGapDetected = true;

	Samples[1].Position = FVector(1.0, 0.1, 0.0).GetSafeNormal();
	Samples[1].PlateId = 0;
	Samples[1].PrevPlateId = 0;
	Samples[1].bIsBoundary = true;
	Samples[1].CrustType = ECrustType::Continental;
	Samples[1].Elevation = 0.5f;
	Samples[1].Thickness = 30.0f;
	Samples[1].Age = 80.0f;

	Samples[2].Position = FVector(1.0, -0.1, 0.0).GetSafeNormal();
	Samples[2].PlateId = 1;
	Samples[2].PrevPlateId = 1;
	Samples[2].bIsBoundary = true;
	Samples[2].CrustType = ECrustType::Continental;
	Samples[2].Elevation = 0.25f;
	Samples[2].Thickness = 28.0f;
	Samples[2].Age = 60.0f;

	Adjacency[0] = { 1, 2 };
	Adjacency[1] = { 0 };
	Adjacency[2] = { 0 };

	TArray<uint8> GapFlags;
	GapFlags.Init(0, Samples.Num());
	GapFlags[0] = 1;

	bool bDivergentGapCreated = false;
	const bool bResolved = FTectonicPlanetTestAccess::ResolveGapSamplePhase4(Planet, 0, GapFlags, Samples, bDivergentGapCreated);
	TestTrue(TEXT("Divergent gap resolves successfully"), bResolved);
	TestTrue(TEXT("Divergent gap creates new oceanic crust"), bDivergentGapCreated);
	TestTrue(TEXT("Divergent gap remains marked as a real gap"), Samples[0].bGapDetected);
	TestEqual(TEXT("Divergent gap crust becomes oceanic"), Samples[0].CrustType, ECrustType::Oceanic);
	TestTrue(TEXT("Divergent gap age resets to zero"), FMath::IsNearlyEqual(Samples[0].Age, 0.0f, 1e-6f));
	TestTrue(TEXT("Divergent gap thickness initializes to oceanic thickness"), FMath::IsNearlyEqual(Samples[0].Thickness, 7.0f, 1e-6f));
	TestTrue(TEXT("Divergent gap assigned to one flank plate"), Samples[0].PlateId == 0 || Samples[0].PlateId == 1);
	const bool bFlanksValid =
		(Samples[0].FlankingPlateIdA == 0 && Samples[0].FlankingPlateIdB == 1) ||
		(Samples[0].FlankingPlateIdA == 1 && Samples[0].FlankingPlateIdB == 0);
	TestTrue(TEXT("Divergent gap stores both flank plate ids"), bFlanksValid);

	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCarriedWorkspaceTest, "Aurous.TectonicPlanet.CarriedWorkspace",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubductionClassificationFixtureTest, "Aurous.TectonicPlanet.SubductionClassificationFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FSubductionClassificationFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(6);
	Adjacency.SetNum(6);
	Plates.SetNum(6);
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		Plates[PlateIndex].Id = PlateIndex;
	}

	auto InitializeBoundaryPair = [&Samples, &Adjacency](const int32 SampleA, const int32 SampleB, const FVector& PositionA, const FVector& PositionB)
	{
		Samples[SampleA].Position = PositionA.GetSafeNormal();
		Samples[SampleB].Position = PositionB.GetSafeNormal();
		Samples[SampleA].PrevPlateId = Samples[SampleA].PlateId;
		Samples[SampleB].PrevPlateId = Samples[SampleB].PlateId;
		Samples[SampleA].bIsBoundary = true;
		Samples[SampleB].bIsBoundary = true;
		Samples[SampleA].bOverlapDetected = true;
		Samples[SampleB].bOverlapDetected = true;
		Samples[SampleA].BoundaryType = EBoundaryType::Convergent;
		Samples[SampleB].BoundaryType = EBoundaryType::Convergent;
		Samples[SampleA].BoundaryNormal = FVector::CrossProduct(FVector::UpVector, Samples[SampleA].Position).GetSafeNormal();
		Samples[SampleB].BoundaryNormal = FVector::CrossProduct(FVector::UpVector, Samples[SampleB].Position).GetSafeNormal();
		if (Samples[SampleA].BoundaryNormal.IsNearlyZero())
		{
			Samples[SampleA].BoundaryNormal = FVector::ForwardVector;
		}
		if (Samples[SampleB].BoundaryNormal.IsNearlyZero())
		{
			Samples[SampleB].BoundaryNormal = FVector::ForwardVector;
		}

		Samples[SampleA].NumOverlapPlateIds = 1;
		Samples[SampleB].NumOverlapPlateIds = 1;
		Samples[SampleA].OverlapPlateIds[0] = Samples[SampleB].PlateId;
		Samples[SampleB].OverlapPlateIds[0] = Samples[SampleA].PlateId;
		Adjacency[SampleA] = { SampleB };
		Adjacency[SampleB] = { SampleA };
	};

	Samples[0].PlateId = 0;
	Samples[1].PlateId = 1;
	InitializeBoundaryPair(0, 1, FVector(1.0, 0.0, 0.0), FVector(0.99, 0.04, 0.0));
	Samples[0].CrustType = ECrustType::Oceanic;
	Samples[0].Age = 80.0f;
	Samples[1].CrustType = ECrustType::Continental;
	Samples[1].Age = 20.0f;

	Samples[2].PlateId = 2;
	Samples[3].PlateId = 3;
	InitializeBoundaryPair(2, 3, FVector(0.0, 1.0, 0.0), FVector(0.0, 0.99, 0.04));
	Samples[2].CrustType = ECrustType::Oceanic;
	Samples[2].Age = 120.0f;
	Samples[3].CrustType = ECrustType::Oceanic;
	Samples[3].Age = 20.0f;

	Samples[4].PlateId = 4;
	Samples[5].PlateId = 5;
	InitializeBoundaryPair(4, 5, FVector(-1.0, 0.0, 0.0), FVector(-0.99, -0.04, 0.0));
	Samples[4].CrustType = ECrustType::Continental;
	Samples[4].Age = 60.0f;
	Samples[5].CrustType = ECrustType::Continental;
	Samples[5].Age = 70.0f;

	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
	FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
	FTectonicPlanetTestAccess::UpdateSubductionFields(Planet, Samples);

	TestEqual(TEXT("Oceanic-continental oceanic side subducts"), Samples[0].SubductionRole, ESubductionRole::Subducting);
	TestEqual(TEXT("Oceanic-continental continental side overrides"), Samples[1].SubductionRole, ESubductionRole::Overriding);
	TestEqual(TEXT("Older oceanic side subducts"), Samples[2].SubductionRole, ESubductionRole::Subducting);
	TestEqual(TEXT("Younger oceanic side overrides"), Samples[3].SubductionRole, ESubductionRole::Overriding);
	TestEqual(TEXT("Continental collision produces no subduction role on side A"), Samples[4].SubductionRole, ESubductionRole::None);
	TestEqual(TEXT("Continental collision produces no subduction role on side B"), Samples[5].SubductionRole, ESubductionRole::None);
	TestTrue(TEXT("Oceanic-continental seeds mark fronts"), Samples[0].bIsSubductionFront && Samples[1].bIsSubductionFront);
	TestTrue(TEXT("Oceanic-oceanic seeds mark fronts"), Samples[2].bIsSubductionFront && Samples[3].bIsSubductionFront);
	TestEqual(TEXT("Pending collision sample count recorded"), Planet.GetPendingCollisionSampleCount(), 2);
	TestEqual(TEXT("Subduction front sample count recorded"), Planet.GetSubductionFrontSampleCount(), 4);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubductionDistanceFieldFixtureTest, "Aurous.TectonicPlanet.SubductionDistanceFieldFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FSubductionDistanceFieldFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(5);
	Adjacency.SetNum(5);
	Plates.SetNum(2);
	Plates[0].Id = 0;
	Plates[1].Id = 1;

	auto EquatorPoint = [](const double Degrees) -> FVector
	{
		const double Radians = FMath::DegreesToRadians(Degrees);
		return FVector(FMath::Cos(Radians), FMath::Sin(Radians), 0.0).GetSafeNormal();
	};

	Samples[0].Position = EquatorPoint(0.0);
	Samples[1].Position = EquatorPoint(5.0);
	Samples[2].Position = EquatorPoint(10.0);
	Samples[3].Position = EquatorPoint(15.0);
	Samples[4].Position = FVector(FMath::Cos(FMath::DegreesToRadians(2.5)), 0.0, FMath::Sin(FMath::DegreesToRadians(5.0))).GetSafeNormal();

	for (int32 SampleIndex = 0; SampleIndex < 4; ++SampleIndex)
	{
		Samples[SampleIndex].PlateId = 0;
		Samples[SampleIndex].PrevPlateId = 0;
		Samples[SampleIndex].CrustType = ECrustType::Continental;
	}
	Samples[4].PlateId = 1;
	Samples[4].PrevPlateId = 1;
	Samples[4].CrustType = ECrustType::Oceanic;

	Samples[0].bIsBoundary = true;
	Samples[0].bOverlapDetected = true;
	Samples[0].BoundaryType = EBoundaryType::Convergent;
	Samples[0].BoundaryNormal = FVector(0.0, 1.0, 0.0);
	Samples[0].NumOverlapPlateIds = 1;
	Samples[0].OverlapPlateIds[0] = 1;

	Samples[4].bIsBoundary = true;
	Samples[4].bOverlapDetected = true;
	Samples[4].BoundaryType = EBoundaryType::Convergent;
	Samples[4].BoundaryNormal = FVector(0.0, -1.0, 0.0);
	Samples[4].NumOverlapPlateIds = 1;
	Samples[4].OverlapPlateIds[0] = 0;

	Adjacency[0] = { 1, 4 };
	Adjacency[1] = { 0, 2 };
	Adjacency[2] = { 1, 3 };
	Adjacency[3] = { 2 };
	Adjacency[4] = { 0 };

	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
	FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
	FTectonicPlanetTestAccess::UpdateSubductionFields(Planet, Samples);

	const double EdgeDistanceKm = 6370.0 * FMath::Acos(FMath::Clamp(FVector::DotProduct(Samples[0].Position, Samples[1].Position), -1.0f, 1.0f));
	TestEqual(TEXT("Boundary seed on overriding plate assigned"), Samples[0].SubductionRole, ESubductionRole::Overriding);
	TestEqual(TEXT("Boundary seed on opposing oceanic plate assigned"), Samples[4].SubductionRole, ESubductionRole::Subducting);
	TestTrue(TEXT("Interior sample 1 inherits overriding role"), Samples[1].SubductionRole == ESubductionRole::Overriding);
	TestTrue(TEXT("Interior sample 2 inherits overriding role"), Samples[2].SubductionRole == ESubductionRole::Overriding);
	TestTrue(TEXT("Interior sample 3 inherits overriding role"), Samples[3].SubductionRole == ESubductionRole::Overriding);
	TestTrue(TEXT("Sample 1 distance matches one geodesic edge"), FMath::IsNearlyEqual(Samples[1].SubductionDistanceKm, EdgeDistanceKm, 1.0f));
	TestTrue(TEXT("Sample 2 distance matches two geodesic edges"), FMath::IsNearlyEqual(Samples[2].SubductionDistanceKm, 2.0 * EdgeDistanceKm, 2.0f));
	TestTrue(TEXT("Sample 3 distance matches three geodesic edges"), FMath::IsNearlyEqual(Samples[3].SubductionDistanceKm, 3.0 * EdgeDistanceKm, 3.0f));
	TestTrue(TEXT("Maximum propagated distance stays within overriding radius"), Samples[3].SubductionDistanceKm <= 1800.0f);
	TestTrue(TEXT("Planet tracks max subduction distance"), Planet.GetMaxSubductionDistanceKm() >= Samples[3].SubductionDistanceKm - 1.0f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubductionPerStepUpdateTest, "Aurous.TectonicPlanet.SubductionPerStepUpdate",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubductionInfluenceProfileFixtureTest, "Aurous.TectonicPlanet.SubductionInfluenceProfileFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FSubductionInfluenceProfileFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(5);
	Plates.SetNum(2);
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		Plates[PlateIndex].Id = PlateIndex;
		Plates[PlateIndex].RotationAxis = FVector::UpVector;
		Plates[PlateIndex].AngularSpeed = 0.0f;
	}

	const FVector Positions[5] = {
		FVector(1.0, 0.0, 0.0),
		FVector(0.98, 0.20, 0.0),
		FVector(0.92, 0.39, 0.0),
		FVector(0.83, 0.56, 0.0),
		FVector(0.0, 1.0, 0.0)
	};

	for (int32 SampleIndex = 0; SampleIndex < 4; ++SampleIndex)
	{
		FCanonicalSample& Sample = Samples[SampleIndex];
		Sample.Position = Positions[SampleIndex].GetSafeNormal();
		Sample.PlateId = 0;
		Sample.PrevPlateId = 0;
		Sample.CrustType = ECrustType::Continental;
		Sample.Elevation = 0.0f;
		Sample.Thickness = 35.0f;
		Sample.Age = 0.0f;
		Sample.SubductionRole = ESubductionRole::Overriding;
		Sample.SubductionOpposingPlateId = 1;
		Sample.SubductionConvergenceSpeedMmPerYear = 100.0f;
	}

	const float RadiusKm = 1800.0f;
	Samples[0].SubductionDistanceKm = 0.0f;
	Samples[1].SubductionDistanceKm = Planet.GetSubductionPeakDistanceKm();
	Samples[2].SubductionDistanceKm = 1000.0f;
	Samples[3].SubductionDistanceKm = RadiusKm;

	Samples[4].Position = Positions[4].GetSafeNormal();
	Samples[4].PlateId = 1;
	Samples[4].PrevPlateId = 1;
	Samples[4].CrustType = ECrustType::Oceanic;
	Samples[4].Elevation = -4.0f;
	Samples[4].Thickness = 7.0f;
	Samples[4].Age = 20.0f;

	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
	FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
	FTectonicPlanetTestAccess::RebuildCarriedWorkspaces(Planet, Samples);

	Planet.AdvancePlateMotionStep();

	const TArray<FCarriedSampleData>& UpdatedCarried = Planet.GetPlates()[0].CarriedSamples;
	TestEqual(TEXT("All overriding samples remain on the same plate"), UpdatedCarried.Num(), 4);

	auto SmoothStep01 = [](const float X) -> float
	{
		const float Clamped = FMath::Clamp(X, 0.0f, 1.0f);
		return (3.0f * Clamped * Clamped) - (2.0f * Clamped * Clamped * Clamped);
	};

	const float PeakDistanceKm = Planet.GetSubductionPeakDistanceKm();
	const float BaseUpliftStepKm = 1.2f;
	const float ElevationInfluence = 0.25f;
	const float ExpectedPeakElevationKm = BaseUpliftStepKm * ElevationInfluence;
	const float MidDistanceAlpha = (1000.0f - PeakDistanceKm) / (RadiusKm - PeakDistanceKm);
	const float ExpectedMidElevationKm = BaseUpliftStepKm * (1.0f - SmoothStep01(MidDistanceAlpha)) * ElevationInfluence;

	TestTrue(TEXT("Subduction uplift is zero at the trench front"), FMath::IsNearlyEqual(UpdatedCarried[0].Elevation, 0.0f, 1e-4f));
	TestTrue(TEXT("Subduction uplift peaks inland at the control distance"), FMath::IsNearlyEqual(UpdatedCarried[1].Elevation, ExpectedPeakElevationKm, 1e-3f));
	TestTrue(TEXT("Subduction uplift fades smoothly after the inland peak"), FMath::IsNearlyEqual(UpdatedCarried[2].Elevation, ExpectedMidElevationKm, 1e-3f));
	TestTrue(TEXT("Subduction uplift returns to zero at the outer influence radius"), FMath::IsNearlyEqual(UpdatedCarried[3].Elevation, 0.0f, 1e-4f));
	TestTrue(TEXT("The inland peak exceeds the trench front and outer-radius uplift"), UpdatedCarried[1].Elevation > UpdatedCarried[0].Elevation && UpdatedCarried[1].Elevation > UpdatedCarried[3].Elevation);
	TestTrue(TEXT("The inland peak exceeds the far-field uplift"), UpdatedCarried[1].Elevation > UpdatedCarried[2].Elevation);

	return true;
}

bool FSubductionPerStepUpdateTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(2);
	Plates.SetNum(2);
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		Plates[PlateIndex].Id = PlateIndex;
	}

	Plates[0].RotationAxis = FVector::UpVector;
	Plates[0].AngularSpeed = 0.01f;
	Plates[1].RotationAxis = -FVector::UpVector;
	Plates[1].AngularSpeed = 0.01f;

	Samples[0].Position = FVector(1.0, 0.0, 0.0);
	Samples[0].PlateId = 0;
	Samples[0].PrevPlateId = 0;
	Samples[0].CrustType = ECrustType::Continental;
	Samples[0].Elevation = 0.0f;
	Samples[0].Thickness = 35.0f;
	Samples[0].Age = 0.0f;
	Samples[0].SubductionRole = ESubductionRole::Overriding;
	Samples[0].SubductionOpposingPlateId = 1;
	Samples[0].SubductionDistanceKm = 350.0f;
	Samples[0].SubductionConvergenceSpeedMmPerYear = 100.0f;
	Samples[0].bIsSubductionFront = true;

	Samples[1].Position = FVector(0.0, 1.0, 0.0);
	Samples[1].PlateId = 1;
	Samples[1].PrevPlateId = 1;
	Samples[1].CrustType = ECrustType::Oceanic;
	Samples[1].Elevation = -4.0f;
	Samples[1].Thickness = 7.0f;
	Samples[1].Age = 20.0f;
	Samples[1].SubductionRole = ESubductionRole::Subducting;
	Samples[1].SubductionOpposingPlateId = 0;
	Samples[1].SubductionDistanceKm = 100.0f;
	Samples[1].SubductionConvergenceSpeedMmPerYear = 100.0f;
	Samples[1].bIsSubductionFront = true;

	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
	FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
	FTectonicPlanetTestAccess::RebuildCarriedWorkspaces(Planet, Samples);

	const FVector InitialOverridingAxis = Planet.GetPlates()[0].RotationAxis.GetSafeNormal();
	const FVector InitialSubductingAxis = Planet.GetPlates()[1].RotationAxis.GetSafeNormal();
	FTectonicPlanetTestAccess::MutablePlates(Planet)[1].CanonicalCenterDirection = FVector::UpVector;

	Planet.AdvancePlateMotionStep();

	const FCarriedSampleData& UpdatedOverriding = Planet.GetPlates()[0].CarriedSamples[0];
	const FCarriedSampleData& UpdatedSubducting = Planet.GetPlates()[1].CarriedSamples[0];
	const float ExpectedOverridingElevation = 0.3f;
	const float ExpectedSubductingElevation = -4.3855f;

	TestTrue(TEXT("Overriding uplift matches transfer function"), FMath::IsNearlyEqual(UpdatedOverriding.Elevation, ExpectedOverridingElevation, 1e-3f));
	TestEqual(TEXT("Overriding uplift labels sample Andean"), UpdatedOverriding.OrogenyType, EOrogenyType::Andean);
	TestTrue(TEXT("Andean age resets when label first applied"), FMath::IsNearlyEqual(UpdatedOverriding.OrogenyAge, 0.0f, 1e-6f));
	TestFalse(TEXT("Fold direction becomes non-zero"), UpdatedOverriding.FoldDirection.IsNearlyZero());
	TestTrue(TEXT("Fold direction remains tangent"), FMath::Abs(FVector::DotProduct(
		UpdatedOverriding.FoldDirection.GetSafeNormal(),
		Planet.GetPlates()[0].CumulativeRotation.RotateVector(Samples[0].Position).GetSafeNormal())) <= 1e-4f);

	TestTrue(TEXT("Subducting sample age advances by one timestep"), FMath::IsNearlyEqual(UpdatedSubducting.Age, 22.0f, 1e-6f));
	TestTrue(TEXT("Subducting trench lowering matches dampening plus trench term"), FMath::IsNearlyEqual(UpdatedSubducting.Elevation, ExpectedSubductingElevation, 1e-3f));
	TestTrue(TEXT("Slab pull does not perturb the overriding plate axis"), Planet.GetPlates()[0].RotationAxis.GetSafeNormal().Equals(InitialOverridingAxis, 1e-6f));
	TestTrue(TEXT("Slab pull keeps axis normalized"), FMath::IsNearlyEqual(Planet.GetPlates()[1].RotationAxis.Size(), 1.0f, 1e-5f));
	TestFalse(TEXT("Slab pull perturbs the subducting plate axis"), Planet.GetPlates()[1].RotationAxis.GetSafeNormal().Equals(InitialSubductingAxis, 1e-6f));
	const FVector MovedCenter = Planet.GetPlates()[1].CumulativeRotation.RotateVector(Planet.GetPlates()[1].CanonicalCenterDirection).GetSafeNormal();
	const FVector FrontPosition = Planet.GetPlates()[1].CumulativeRotation.RotateVector(Samples[1].Position).GetSafeNormal();
	const FVector SlabPullVote = FVector::CrossProduct(MovedCenter, FrontPosition).GetSafeNormal();
	const float InitialVoteAngle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector::DotProduct(InitialSubductingAxis, SlabPullVote), -1.0f, 1.0f)));
	const float UpdatedVoteAngle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector::DotProduct(Planet.GetPlates()[1].RotationAxis.GetSafeNormal(), SlabPullVote), -1.0f, 1.0f)));
	const float AxisStepAngle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector::DotProduct(InitialSubductingAxis, Planet.GetPlates()[1].RotationAxis.GetSafeNormal()), -1.0f, 1.0f)));
	TestTrue(TEXT("Slab pull rotates the subducting plate axis toward the front vote direction"), UpdatedVoteAngle + 1e-4f < InitialVoteAngle);
	TestTrue(TEXT("Slab pull axis correction stays within the configured per-step clamp"), AxisStepAngle <= Planet.GetSlabPullAxisMaxDegreesPerStep() + 1e-3f);
	TestTrue(TEXT("Planet metrics track Andean samples"), Planet.GetAndeanSampleCount() > 0);
	TestTrue(TEXT("Planet metrics keep subduction front samples"), Planet.GetSubductionFrontSampleCount() >= 2);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCollisionPerStepUpdateFixtureTest, "Aurous.TectonicPlanet.CollisionPerStepUpdateFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCollisionPerStepUpdateFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(2);
	Plates.SetNum(2);
	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		Plates[PlateIndex].Id = PlateIndex;
	}

	Plates[0].RotationAxis = FVector::UpVector;
	Plates[0].AngularSpeed = 0.01f;
	Plates[0].CanonicalCenterDirection = FVector::ForwardVector;
	Plates[1].RotationAxis = -FVector::UpVector;
	Plates[1].AngularSpeed = 0.01f;
	Plates[1].CanonicalCenterDirection = FVector::RightVector;

	Samples[0].Position = FVector(1.0, 0.0, 0.0);
	Samples[0].PlateId = 0;
	Samples[0].PrevPlateId = 0;
	Samples[0].CrustType = ECrustType::Continental;
	Samples[0].Elevation = 0.0f;
	Samples[0].Thickness = 35.0f;
	Samples[0].Age = 0.0f;
	Samples[0].OrogenyType = EOrogenyType::Andean;
	Samples[0].OrogenyAge = 7.0f;
	Samples[0].CollisionDistanceKm = 0.0f;
	Samples[0].CollisionConvergenceSpeedMmPerYear = 100.0f;
	Samples[0].CollisionOpposingPlateId = 1;
	Samples[0].CollisionInfluenceRadiusKm = 1200.0f;
	Samples[0].SubductionRole = ESubductionRole::Overriding;
	Samples[0].SubductionOpposingPlateId = 1;
	Samples[0].SubductionDistanceKm = 350.0f;
	Samples[0].SubductionConvergenceSpeedMmPerYear = 100.0f;
	Samples[0].bIsSubductionFront = true;

	Samples[1].Position = FVector(0.0, 1.0, 0.0);
	Samples[1].PlateId = 1;
	Samples[1].PrevPlateId = 1;
	Samples[1].CrustType = ECrustType::Oceanic;
	Samples[1].Elevation = -4.0f;
	Samples[1].Thickness = 7.0f;
	Samples[1].Age = 20.0f;

	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
	FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
	FTectonicPlanetTestAccess::RebuildCarriedWorkspaces(Planet, Samples);

	Planet.AdvancePlateMotionStep();

	const FCarriedSampleData& FirstStepSample = Planet.GetPlates()[0].CarriedSamples[0];
	auto ComputeElevationInfluence = [](const float ElevationKm) -> float
	{
		const float NormalizedElevation = FMath::Clamp((ElevationKm + 10.0f) / 20.0f, 0.0f, 1.0f);
		return NormalizedElevation * NormalizedElevation;
	};
	const float BaseUpliftStepKm = 1.2f;
	const float HimalayanStepKm = BaseUpliftStepKm * ComputeElevationInfluence(0.0f);
	const float ExpectedFirstStepElevationKm = HimalayanStepKm + BaseUpliftStepKm * ComputeElevationInfluence(HimalayanStepKm);
	const FVector FirstMovedPosition = Planet.GetPlates()[0].CumulativeRotation.RotateVector(Samples[0].Position).GetSafeNormal();
	FVector CompressionAxis = Planet.ComputeSurfaceVelocity(1, FirstMovedPosition) - Planet.ComputeSurfaceVelocity(0, FirstMovedPosition);
	CompressionAxis = CompressionAxis - FVector::DotProduct(CompressionAxis, FirstMovedPosition) * FirstMovedPosition;
	CompressionAxis = CompressionAxis.GetSafeNormal();
	FVector HimalayanFoldTarget = FVector::CrossProduct(FirstMovedPosition, CompressionAxis);
	HimalayanFoldTarget = HimalayanFoldTarget - FVector::DotProduct(HimalayanFoldTarget, FirstMovedPosition) * FirstMovedPosition;
	HimalayanFoldTarget = HimalayanFoldTarget.GetSafeNormal();
	FVector ExpectedFoldDirection = FMath::Lerp(HimalayanFoldTarget, CompressionAxis, 0.2f);
	ExpectedFoldDirection = ExpectedFoldDirection - FVector::DotProduct(ExpectedFoldDirection, FirstMovedPosition) * FirstMovedPosition;
	ExpectedFoldDirection = ExpectedFoldDirection.GetSafeNormal();

	TestTrue(TEXT("Per-step Himalayan and Andean uplift contributions stack"), FMath::IsNearlyEqual(FirstStepSample.Elevation, ExpectedFirstStepElevationKm, 1e-3f));
	TestEqual(TEXT("Active Himalayan uplift takes precedence over Andean labeling"), FirstStepSample.OrogenyType, EOrogenyType::Himalayan);
	TestTrue(TEXT("Himalayan precedence resets orogeny age when it overrides Andean"), FMath::IsNearlyEqual(FirstStepSample.OrogenyAge, 0.0f, 1e-6f));
	TestFalse(TEXT("Himalayan fold direction becomes non-zero"), FirstStepSample.FoldDirection.IsNearlyZero());
	TestTrue(TEXT("Himalayan fold direction remains tangent"), FMath::Abs(FVector::DotProduct(FirstStepSample.FoldDirection.GetSafeNormal(), FirstMovedPosition)) <= 1e-4f);
	TestTrue(TEXT("Himalayan fold direction follows the collision compression axis orientation"), FVector::DotProduct(FirstStepSample.FoldDirection.GetSafeNormal(), ExpectedFoldDirection) >= 0.99f);

	const float FirstStepElevation = FirstStepSample.Elevation;
	Planet.AdvancePlateMotionStep();
	const FCarriedSampleData& SecondStepSample = Planet.GetPlates()[0].CarriedSamples[0];

	TestTrue(TEXT("Persistent Himalayan uplift continues between reconciles"), SecondStepSample.Elevation > FirstStepElevation);
	TestEqual(TEXT("Persistent uplift keeps the Himalayan label"), SecondStepSample.OrogenyType, EOrogenyType::Himalayan);
	TestTrue(TEXT("Persistent Himalayan uplift advances the orogeny age after the first step"), FMath::IsNearlyEqual(SecondStepSample.OrogenyAge, 2.0f, 1e-6f));
	TestTrue(TEXT("Planet metrics track Himalayan samples during per-step updates"), Planet.GetHimalayanSampleCount() > 0);
	TestEqual(TEXT("Andean metrics drop to zero when Himalayan precedence wins"), Planet.GetAndeanSampleCount(), 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTerraneIdentityFixtureTest, "Aurous.TectonicPlanet.TerraneIdentityFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FTerraneIdentityFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(6);
	Adjacency.SetNum(6);
	Plates.SetNum(1);
	InitializeFixturePlate(Plates[0], 0, FVector::UpVector);
	FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1000.0, 3000.0);

	const FVector Positions[6] = {
		FVector(1.0, 0.0, 0.0),
		FVector(0.98, 0.18, 0.0),
		FVector(0.96, 0.28, 0.0),
		FVector(-1.0, 0.0, 0.0),
		FVector(-0.98, 0.18, 0.0),
		FVector(0.0, 1.0, 0.0)
	};

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		Samples[SampleIndex].Position = Positions[SampleIndex].GetSafeNormal();
		Samples[SampleIndex].PlateId = 0;
		Samples[SampleIndex].PrevPlateId = 0;
		Samples[SampleIndex].CrustType = (SampleIndex < 5) ? ECrustType::Continental : ECrustType::Oceanic;
	}

	LinkBidirectional(Adjacency, 0, 1);
	LinkBidirectional(Adjacency, 1, 2);
	LinkBidirectional(Adjacency, 0, 2);
	LinkBidirectional(Adjacency, 3, 4);

	FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);

	const int32 TerraneA = Samples[0].TerraneId;
	const int32 TerraneB = Samples[3].TerraneId;
	TestTrue(TEXT("First component receives a terrane id"), TerraneA != INDEX_NONE);
	TestTrue(TEXT("Second component receives a terrane id"), TerraneB != INDEX_NONE);
	TestTrue(TEXT("Disconnected components receive distinct terrane ids"), TerraneA != TerraneB);
	TestEqual(TEXT("Two active terranes detected initially"), Planet.GetActiveTerraneCount(), 2);

	FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
	TestEqual(TEXT("Stable topology retains terrane id on component A"), Samples[1].TerraneId, TerraneA);
	TestEqual(TEXT("Stable topology retains terrane id on component B"), Samples[3].TerraneId, TerraneB);

	const int32 AnchorA = FTectonicPlanetTestAccess::GetTerraneAnchorSampleIndex(Planet, TerraneA);
	TestTrue(TEXT("Terrane A anchor is valid"), Samples.IsValidIndex(AnchorA));
	Samples[AnchorA].CrustType = ECrustType::Oceanic;
	FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
	const int32 RemainingAIndex = (AnchorA == 0) ? 1 : 0;
	TestEqual(TEXT("Centroid fallback keeps terrane A after anchor loss"), Samples[RemainingAIndex].TerraneId, TerraneA);

	Samples[5].CrustType = ECrustType::Continental;
	FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
	const int32 NewTerraneId = Samples[5].TerraneId;
	TestTrue(TEXT("Unmatched new component gets a new terrane id"), NewTerraneId != INDEX_NONE && NewTerraneId != TerraneA && NewTerraneId != TerraneB);
	TestEqual(TEXT("Three active terranes exist after new component appears"), Planet.GetActiveTerraneCount(), 3);

	Samples[AnchorA].CrustType = ECrustType::Continental;
	Samples[AnchorA].TerraneId = TerraneA;
	LinkBidirectional(Adjacency, 2, 3);
	FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
	for (int32 SampleIndex = 0; SampleIndex <= 4; ++SampleIndex)
	{
		TestEqual(FString::Printf(TEXT("Merged continental chain sample %d keeps dominant terrane"), SampleIndex), Samples[SampleIndex].TerraneId, TerraneA);
	}
	TestEqual(TEXT("Merged terrane is aliased to dominant terrane"), FTectonicPlanetTestAccess::GetTerraneMergedIntoId(Planet, TerraneB), TerraneA);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCollisionOrderingFixtureTest, "Aurous.TectonicPlanet.CollisionOrderingFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCollisionOrderingFixtureTest::RunTest(const FString& Parameters)
{
	{
		FTectonicPlanet Planet;
		TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
		TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

		Samples.SetNum(2);
		Adjacency.SetNum(2);
		Plates.SetNum(2);
		InitializeFixturePlate(Plates[0], 0, FVector::UpVector);
		InitializeFixturePlate(Plates[1], 1, -FVector::UpVector);
		FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1000.0, 3000.0);

		InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 40.0f, true);
		InitializeConvergentBoundarySample(Samples[1], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 60.0f);
		LinkBidirectional(Adjacency, 0, 1);

		FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
		FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
		FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);

		Samples[1].CrustType = ECrustType::Oceanic;
		FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
		TestFalse(TEXT("Collision candidate requires a continental opposing terrane"), FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples));

		Samples[1].CrustType = ECrustType::Continental;
		FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
		Samples[0].bOverlapDetected = false;
		Samples[1].bOverlapDetected = false;
		TestFalse(TEXT("Collision candidate requires overlap contact"), FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples));

		Samples[0].bOverlapDetected = true;
		Samples[1].bOverlapDetected = true;
		Samples[0].BoundaryType = EBoundaryType::Transform;
		Samples[1].BoundaryType = EBoundaryType::Transform;
		TestFalse(TEXT("Collision candidate requires a convergent boundary"), FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples));
	}

	{
		FTectonicPlanet Planet;
		TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
		TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

		Samples.SetNum(6);
		Adjacency.SetNum(6);
		Plates.SetNum(3);
		InitializeFixturePlate(Plates[0], 0, FVector::UpVector);
		InitializeFixturePlate(Plates[1], 1, -FVector::UpVector);
		InitializeFixturePlate(Plates[2], 2, -FVector::UpVector);
		FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1000.0, 3000.0);

		InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 40.0f, true);
		InitializeConvergentBoundarySample(Samples[1], 0, FVector(0.93, 0.36, 0.0), 1, ECrustType::Continental, 40.0f, true);
		InitializeConvergentBoundarySample(Samples[2], 0, FVector(0.93, -0.36, 0.0), 2, ECrustType::Continental, 40.0f, true);
		InitializeConvergentBoundarySample(Samples[3], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 60.0f);
		InitializeConvergentBoundarySample(Samples[4], 1, FVector(0.89, 0.46, 0.0), 0, ECrustType::Continental, 60.0f);
		InitializeConvergentBoundarySample(Samples[5], 2, FVector(0.89, -0.46, 0.0), 0, ECrustType::Continental, 55.0f);

		LinkBidirectional(Adjacency, 0, 1);
		LinkBidirectional(Adjacency, 1, 2);
		LinkBidirectional(Adjacency, 3, 4);
		LinkBidirectional(Adjacency, 0, 3);
		LinkBidirectional(Adjacency, 1, 4);
		LinkBidirectional(Adjacency, 2, 5);

		FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
		FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
		FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
		const bool bApplied = FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples);

		TestTrue(TEXT("At least one continental collision event is applied"), bApplied);
		TestEqual(TEXT("Only one collision fires when one terrane contacts two others"), Planet.GetCollisionEventCount(), 1);
		TestEqual(TEXT("Higher-contact pair is chosen first"), FTectonicPlanetTestAccess::GetCollisionEventContactSampleCount(Planet, 0), 4);
		TestEqual(TEXT("Higher-contact pair detaches the smaller terrane onto receiver plate"), Samples[3].PlateId, 0);
		TestEqual(TEXT("All samples of the chosen donor terrane detach together"), Samples[4].PlateId, 0);
		TestEqual(TEXT("Lower-priority competing contact is deferred"), Samples[5].PlateId, 2);
	}

	{
		FTectonicPlanet Planet;
		TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
		TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

		Samples.SetNum(4);
		Adjacency.SetNum(4);
		Plates.SetNum(4);
		InitializeFixturePlate(Plates[0], 0, FVector::UpVector, 0.02f);
		InitializeFixturePlate(Plates[1], 1, -FVector::UpVector, 0.02f);
		InitializeFixturePlate(Plates[2], 2, FVector::UpVector, 0.005f);
		InitializeFixturePlate(Plates[3], 3, -FVector::UpVector, 0.005f);
		for (FPlate& Plate : Plates)
		{
			Plate.PersistencePolicy = EPlatePersistencePolicy::Retirable;
		}
		FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1200.0, 2400.0);

		InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 40.0f, true);
		InitializeConvergentBoundarySample(Samples[1], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 60.0f);
		InitializeConvergentBoundarySample(Samples[2], 2, FVector(0.0, 1.0, 0.0), 3, ECrustType::Continental, 45.0f, true);
		InitializeConvergentBoundarySample(Samples[3], 3, FVector(0.0, 0.99, 0.05), 2, ECrustType::Continental, 55.0f);

		LinkBidirectional(Adjacency, 0, 1);
		LinkBidirectional(Adjacency, 2, 3);

		FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
		FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
		FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);

		const int32 FastPairTerraneA = Samples[0].TerraneId;
		const int32 FastPairTerraneB = Samples[1].TerraneId;
		TSet<int32> TerranesUsedThisReconcile;
		const bool bApplied = FTectonicPlanetTestAccess::ApplyNextContinentalCollision(Planet, Samples, TerranesUsedThisReconcile);
		TestTrue(TEXT("Speed-order fixture applies a collision"), bApplied);
		TestEqual(TEXT("Speed-order fixture records the chosen first event"), Planet.GetCollisionEventCount(), 1);

		const bool bFastPairFirst =
			(FTectonicPlanetTestAccess::GetCollisionEventDonorTerraneId(Planet, 0) == FastPairTerraneA &&
				FTectonicPlanetTestAccess::GetCollisionEventReceiverTerraneId(Planet, 0) == FastPairTerraneB) ||
			(FTectonicPlanetTestAccess::GetCollisionEventDonorTerraneId(Planet, 0) == FastPairTerraneB &&
				FTectonicPlanetTestAccess::GetCollisionEventReceiverTerraneId(Planet, 0) == FastPairTerraneA);
		TestTrue(TEXT("Mean convergence speed breaks ties after contact count"), bFastPairFirst);
	}

	{
		FTectonicPlanet Planet;
		TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
		TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

		Samples.SetNum(4);
		Adjacency.SetNum(4);
		Plates.SetNum(4);
		InitializeFixturePlate(Plates[0], 0, FVector::UpVector, 0.01f);
		InitializeFixturePlate(Plates[1], 1, -FVector::UpVector, 0.01f);
		InitializeFixturePlate(Plates[2], 2, FVector::UpVector, 0.01f);
		InitializeFixturePlate(Plates[3], 3, -FVector::UpVector, 0.01f);
		for (FPlate& Plate : Plates)
		{
			Plate.PersistencePolicy = EPlatePersistencePolicy::Retirable;
		}
		FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1200.0, 2400.0);

		InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 40.0f, true);
		InitializeConvergentBoundarySample(Samples[1], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 60.0f);
		InitializeConvergentBoundarySample(Samples[2], 2, FVector(0.0, 1.0, 0.0), 3, ECrustType::Continental, 45.0f, true);
		InitializeConvergentBoundarySample(Samples[3], 3, FVector(0.0, 0.99, 0.05), 2, ECrustType::Continental, 55.0f);

		LinkBidirectional(Adjacency, 0, 1);
		LinkBidirectional(Adjacency, 2, 3);

		FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
		FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
		FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);

		const int32 LowerPairTerraneA = Samples[0].TerraneId;
		const int32 LowerPairTerraneB = Samples[1].TerraneId;
		TSet<int32> TerranesUsedThisReconcile;
		const bool bApplied = FTectonicPlanetTestAccess::ApplyNextContinentalCollision(Planet, Samples, TerranesUsedThisReconcile);
		TestTrue(TEXT("Equal-speed disjoint fixture applies a collision"), bApplied);
		TestEqual(TEXT("Equal-speed disjoint fixture records the chosen first event"), Planet.GetCollisionEventCount(), 1);

		const bool bLowerTerranePairFirst =
			(FTectonicPlanetTestAccess::GetCollisionEventDonorTerraneId(Planet, 0) == LowerPairTerraneA &&
				FTectonicPlanetTestAccess::GetCollisionEventReceiverTerraneId(Planet, 0) == LowerPairTerraneB) ||
			(FTectonicPlanetTestAccess::GetCollisionEventDonorTerraneId(Planet, 0) == LowerPairTerraneB &&
				FTectonicPlanetTestAccess::GetCollisionEventReceiverTerraneId(Planet, 0) == LowerPairTerraneA);
		TestTrue(TEXT("Terrane ids break event-order ties after contact count and speed"), bLowerTerranePairFirst);
	}

	{
		FTectonicPlanet Planet;
		TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
		TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

		Samples.SetNum(4);
		Adjacency.SetNum(4);
		Plates.SetNum(2);
		InitializeFixturePlate(Plates[0], 0, FVector::UpVector);
		InitializeFixturePlate(Plates[1], 1, -FVector::UpVector);
		FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1500.0, 3000.0);

		InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 35.0f, true);
		InitializeConvergentBoundarySample(Samples[1], 0, FVector(0.95, 0.30, 0.0), 1, ECrustType::Continental, 36.0f, true);
		InitializeConvergentBoundarySample(Samples[2], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 60.0f);
		Samples[3].Position = FVector(0.92, 0.38, 0.0).GetSafeNormal();
		Samples[3].PlateId = 1;
		Samples[3].PrevPlateId = 1;
		Samples[3].CrustType = ECrustType::Continental;
		Samples[3].Age = 61.0f;

		LinkBidirectional(Adjacency, 0, 1);
		LinkBidirectional(Adjacency, 0, 2);
		LinkBidirectional(Adjacency, 1, 2);
		LinkBidirectional(Adjacency, 2, 3);

		FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
		FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
		FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
		const bool bApplied = FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples);

		TestTrue(TEXT("Front-vote fixture applies a collision"), bApplied);
		TestEqual(TEXT("Fewer overlap-front ownership votes donate first"), FTectonicPlanetTestAccess::GetCollisionEventDonorPlateId(Planet, 0), 1);
		TestEqual(TEXT("Front-vote donor transfers its entire terrane"), Samples[2].PlateId, 0);
		TestEqual(TEXT("Front-vote donor transfers its interior followers"), Samples[3].PlateId, 0);
	}

	{
		FTectonicPlanet Planet;
		TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
		TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
		TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

		Samples.SetNum(2);
		Adjacency.SetNum(2);
		Plates.SetNum(2);
		InitializeFixturePlate(Plates[0], 0, FVector::UpVector);
		InitializeFixturePlate(Plates[1], 1, -FVector::UpVector);
		FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1200.0, 1200.0);

		InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 45.0f, true);
		InitializeConvergentBoundarySample(Samples[1], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 45.0f);
		LinkBidirectional(Adjacency, 0, 1);

		FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
		FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
		FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);

		const int32 LowerTerraneId = FMath::Min(Samples[0].TerraneId, Samples[1].TerraneId);
		const int32 LowerTerraneSampleIndex = (Samples[0].TerraneId <= Samples[1].TerraneId) ? 0 : 1;
		const int32 ReceiverPlateId = (LowerTerraneSampleIndex == 0) ? 1 : 0;
		const bool bApplied = FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples);
		TestTrue(TEXT("Tie-break fixture applies a collision"), bApplied);
		TestEqual(TEXT("Lower terrane id donates when front votes, area, and speed tie"), FTectonicPlanetTestAccess::GetCollisionEventDonorTerraneId(Planet, 0), LowerTerraneId);
		TestEqual(TEXT("Lower terrane id sample transfers under final deterministic tie-break"), Samples[LowerTerraneSampleIndex].PlateId, ReceiverPlateId);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCollisionDisjointPairsFixtureTest, "Aurous.TectonicPlanet.CollisionDisjointPairsFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCollisionDisjointPairsFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(4);
	Adjacency.SetNum(4);
	Plates.SetNum(4);
	InitializeFixturePlate(Plates[0], 0, FVector::UpVector);
	InitializeFixturePlate(Plates[1], 1, -FVector::UpVector);
	InitializeFixturePlate(Plates[2], 2, FVector::UpVector);
	InitializeFixturePlate(Plates[3], 3, -FVector::UpVector);
	for (FPlate& Plate : Plates)
	{
		Plate.PersistencePolicy = EPlatePersistencePolicy::Retirable;
	}
	FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1200.0, 1200.0);

	InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 50.0f, true);
	InitializeConvergentBoundarySample(Samples[1], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 55.0f);
	InitializeConvergentBoundarySample(Samples[2], 2, FVector(-1.0, 0.0, 0.0), 3, ECrustType::Continental, 45.0f, true);
	InitializeConvergentBoundarySample(Samples[3], 3, FVector(-0.99, 0.05, 0.0), 2, ECrustType::Continental, 65.0f);

	LinkBidirectional(Adjacency, 0, 1);
	LinkBidirectional(Adjacency, 2, 3);

	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
	FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
	FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
	TSet<int32> TerranesUsedThisReconcile;
	TArray<uint8> FirstDirtyPlateFlags;
	const bool bAppliedFirst = FTectonicPlanetTestAccess::ApplyNextContinentalCollision(Planet, Samples, TerranesUsedThisReconcile, &FirstDirtyPlateFlags);
	TestTrue(TEXT("First disjoint collision applies"), bAppliedFirst);
	FTectonicPlanetTestAccess::RefreshCanonicalAfterCollision(Planet, Samples, FirstDirtyPlateFlags);

	TArray<uint8> SecondDirtyPlateFlags;
	const bool bAppliedSecond = FTectonicPlanetTestAccess::ApplyNextContinentalCollision(Planet, Samples, TerranesUsedThisReconcile, &SecondDirtyPlateFlags);
	TestTrue(TEXT("Second disjoint collision applies after the refresh"), bAppliedSecond);
	FTectonicPlanetTestAccess::RefreshCanonicalAfterCollision(Planet, Samples, SecondDirtyPlateFlags);

	TArray<uint8> DirtyPlateFlags = FirstDirtyPlateFlags;
	if (DirtyPlateFlags.Num() != SecondDirtyPlateFlags.Num())
	{
		DirtyPlateFlags.Init(0, SecondDirtyPlateFlags.Num());
	}
	for (int32 PlateIndex = 0; PlateIndex < SecondDirtyPlateFlags.Num(); ++PlateIndex)
	{
		if (SecondDirtyPlateFlags[PlateIndex] != 0)
		{
			DirtyPlateFlags[PlateIndex] = 1;
		}
	}

	TestEqual(TEXT("Two disjoint terrane pairs can both collide in one reconcile"), Planet.GetCollisionEventCount(), 2);
	TestEqual(TEXT("Lower terrane id donor of first pair transfers to receiver plate"), Samples[0].PlateId, 1);
	TestEqual(TEXT("Lower terrane id donor of second pair transfers to receiver plate"), Samples[2].PlateId, 3);
	TestEqual(TEXT("Sequential collisions record both pair histories"), FTectonicPlanetTestAccess::GetCollisionHistoryKeyCount(Planet), 2);
	TestTrue(TEXT("Sequential refresh rebuilds the first receiver plate before the second event"), Planet.GetPlates()[1].SampleIndices.Num() >= 2);
	TestTrue(TEXT("Sequential refresh rebuilds the second receiver plate before reconcile ends"), Planet.GetPlates()[3].SampleIndices.Num() >= 2);
	TestTrue(TEXT("Dirty set covers every plate touched by the sequential events"), DirtyPlateFlags[0] != 0 && DirtyPlateFlags[1] != 0 && DirtyPlateFlags[2] != 0 && DirtyPlateFlags[3] != 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCollisionRefreshFixtureTest, "Aurous.TectonicPlanet.CollisionRefreshFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCollisionRefreshFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(5);
	Adjacency.SetNum(5);
	Plates.SetNum(2);
	InitializeFixturePlate(Plates[0], 0, FVector::UpVector);
	InitializeFixturePlate(Plates[1], 1, -FVector::UpVector);
	FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1500.0, 3000.0);

	InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 35.0f, true);
	Samples[1].Position = FVector(0.97, 0.18, 0.0).GetSafeNormal();
	Samples[1].PlateId = 0;
	Samples[1].PrevPlateId = 0;
	Samples[1].CrustType = ECrustType::Continental;
	Samples[1].Age = 20.0f;
	Samples[3].Position = FVector(0.95, -0.22, 0.0).GetSafeNormal();
	Samples[3].PlateId = 0;
	Samples[3].PrevPlateId = 0;
	Samples[3].CrustType = ECrustType::Continental;
	Samples[3].Age = 22.0f;
	InitializeConvergentBoundarySample(Samples[2], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 65.0f);
	Samples[4].Position = FVector(-1.0, 0.0, 0.0).GetSafeNormal();
	Samples[4].PlateId = 1;
	Samples[4].PrevPlateId = 1;
	Samples[4].CrustType = ECrustType::Continental;
	Samples[4].Age = 80.0f;

	LinkBidirectional(Adjacency, 0, 1);
	LinkBidirectional(Adjacency, 1, 3);
	LinkBidirectional(Adjacency, 0, 2);

	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
	FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
	FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
	TArray<uint8> DirtyPlateFlags;
	const bool bApplied = FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples, DirtyPlateFlags);
	TestTrue(TEXT("Collision event applies in refresh fixture"), bApplied);
	TestEqual(TEXT("Smaller donor terrane switches to receiver plate"), Samples[2].PlateId, 0);
	TestEqual(TEXT("Non-colliding terrane keeps donor plate alive"), Samples[4].PlateId, 1);
	TestEqual(TEXT("Transferred sample preserves previous owner through collision refresh"), Samples[2].PrevPlateId, 1);
	TestFalse(TEXT("Transferred sample recomputes overlap state before the next scan"), Samples[2].bOverlapDetected);
	TestFalse(TEXT("Transferred sample recomputes boundary flags before the next scan"), Samples[2].bIsBoundary);
	TestTrue(TEXT("Collision refresh keeps donor plate marked dirty"), DirtyPlateFlags[1] != 0);
	TestTrue(TEXT("Collision refresh keeps receiver plate marked dirty"), DirtyPlateFlags[0] != 0);

	int32 HimalayanSamples = 0;
	int32 TangentFoldSamples = 0;
	for (const FCanonicalSample& Sample : Samples)
	{
		if (Sample.OrogenyType != EOrogenyType::Himalayan)
		{
			continue;
		}

		++HimalayanSamples;
		TestTrue(TEXT("Himalayan uplift resets age"), FMath::IsNearlyEqual(Sample.OrogenyAge, 0.0f, 1e-6f));
		TestTrue(TEXT("Himalayan samples record collision distance"), Sample.CollisionDistanceKm >= 0.0f);
		if (!Sample.FoldDirection.IsNearlyZero())
		{
			const float TangentDot = FMath::Abs(FVector::DotProduct(Sample.FoldDirection.GetSafeNormal(), Sample.Position.GetSafeNormal()));
			TangentFoldSamples += (TangentDot <= 1e-4f) ? 1 : 0;
		}
	}

	TestTrue(TEXT("Collision produces Himalayan uplift samples"), HimalayanSamples > 0);
	TestTrue(TEXT("At least one Himalayan fold direction remains tangent"), TangentFoldSamples > 0);

	FTectonicPlanetTestAccess::RefreshCanonicalAfterCollision(Planet, Samples);
	TestTrue(TEXT("Post-collision refresh rebuilds receiver membership"), Planet.GetPlates()[0].SampleIndices.Num() >= 4);
	TestTrue(TEXT("Post-collision refresh keeps donor plate non-empty"), Planet.GetPlates()[1].SampleIndices.Num() >= 1);
	TestEqual(TEXT("Post-collision refresh restores single-component ownership per plate"), Planet.GetMaxPlateComponentCount(), 1);
	TestEqual(TEXT("Post-collision refresh clears detached non-gap fragments"), Planet.GetDetachedPlateFragmentSampleCount(), 0);
	TestEqual(TEXT("Post-collision refresh clears detached fragment size"), Planet.GetLargestDetachedPlateFragmentSize(), 0);
	TestTrue(TEXT("Terranes remain tracked after refresh"), Planet.GetActiveTerraneCount() >= 2);
	TestTrue(TEXT("Reassigned samples keep a valid terrane id after refresh"), Samples[2].TerraneId != INDEX_NONE);

	const bool bAppliedAgain = FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples);
	TestFalse(TEXT("Collision history suppresses repeat event on ongoing contact"), bAppliedAgain);
	TestEqual(TEXT("Collision event count remains one after repeat suppression"), Planet.GetCollisionEventCount(), 1);

	{
		FTectonicPlanet DirtyPlanet;
		TArray<FCanonicalSample>& DirtySamples = FTectonicPlanetTestAccess::MutableSamples(DirtyPlanet);
		TArray<TArray<int32>>& DirtyAdjacency = FTectonicPlanetTestAccess::MutableAdjacency(DirtyPlanet);
		TArray<FPlate>& DirtyPlates = FTectonicPlanetTestAccess::MutablePlates(DirtyPlanet);

		DirtySamples.SetNum(5);
		DirtyAdjacency.SetNum(5);
		DirtyPlates.SetNum(3);
		InitializeFixturePlate(DirtyPlates[0], 0, FVector::UpVector);
		InitializeFixturePlate(DirtyPlates[1], 1, FVector::RightVector);
		InitializeFixturePlate(DirtyPlates[2], 2, -FVector::UpVector);
		DirtyPlates[1].PersistencePolicy = EPlatePersistencePolicy::Retirable;

		for (int32 SampleIndex = 0; SampleIndex < DirtySamples.Num(); ++SampleIndex)
		{
			DirtySamples[SampleIndex].Position = FVector(1.0, 0.2 * static_cast<double>(SampleIndex), 0.0).GetSafeNormal();
			DirtySamples[SampleIndex].PlateId = (SampleIndex <= 1) ? 0 : (SampleIndex <= 3 ? 1 : 2);
			DirtySamples[SampleIndex].PrevPlateId = DirtySamples[SampleIndex].PlateId;
			DirtySamples[SampleIndex].CrustType = ECrustType::Continental;
		}

		LinkBidirectional(DirtyAdjacency, 0, 1);
		LinkBidirectional(DirtyAdjacency, 3, 4);

		FTectonicPlanetTestAccess::RebuildPlateMembership(DirtyPlanet, DirtySamples);
		FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(DirtyPlanet, DirtySamples);

		TArray<uint8> ExpandedDirtyFlags;
		ExpandedDirtyFlags.Init(0, DirtyPlates.Num());
		ExpandedDirtyFlags[0] = 1;
		ExpandedDirtyFlags[1] = 1;
		FTectonicPlanetTestAccess::RefreshCanonicalAfterCollision(DirtyPlanet, DirtySamples, ExpandedDirtyFlags);

		TestEqual(TEXT("Connectivity refresh can pull a detached dirty fragment onto a clean neighbor plate"), DirtySamples[3].PlateId, 2);
		TestTrue(TEXT("Dirty plate set widens when refresh changes ownership onto a new plate"), ExpandedDirtyFlags[2] != 0);
		TestTrue(TEXT("Expanded dirty set rebuilds the newly touched plate membership"), DirtyPlanet.GetPlates()[2].SampleIndices.Contains(3));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCollisionFrontAndRadiusFixtureTest, "Aurous.TectonicPlanet.CollisionFrontAndRadiusFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCollisionFrontAndRadiusFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	Samples.SetNum(4);
	Adjacency.SetNum(4);
	Plates.SetNum(2);
	InitializeFixturePlate(Plates[0], 0, FVector::UpVector);
	InitializeFixturePlate(Plates[1], 1, -FVector::UpVector);
	FTectonicPlanetTestAccess::SetAreaMetrics(Planet, 1000.0, 4000.0);

	InitializeConvergentBoundarySample(Samples[0], 0, FVector(1.0, 0.0, 0.0), 1, ECrustType::Continental, 35.0f, true);
	InitializeConvergentBoundarySample(Samples[1], 1, FVector(0.99, 0.05, 0.0), 0, ECrustType::Continental, 60.0f);
	Samples[2].Position = FVector(0.97, 0.24, 0.0).GetSafeNormal();
	Samples[2].PlateId = 1;
	Samples[2].PrevPlateId = 1;
	Samples[2].CrustType = ECrustType::Continental;
	Samples[3].Position = FVector(0.70, 0.71, 0.0).GetSafeNormal();
	Samples[3].PlateId = 1;
	Samples[3].PrevPlateId = 1;
	Samples[3].CrustType = ECrustType::Continental;

	LinkBidirectional(Adjacency, 0, 1);
	LinkBidirectional(Adjacency, 1, 2);
	LinkBidirectional(Adjacency, 2, 3);

	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);
	FTectonicPlanetTestAccess::UpdatePlateCanonicalCenters(Planet, Samples);
	FTectonicPlanetTestAccess::DetectTerranes(Planet, Samples);
	const bool bApplied = FTectonicPlanetTestAccess::ApplyContinentalCollisions(Planet, Samples);

	TestTrue(TEXT("Front-and-radius fixture applies a collision"), bApplied);
	TestEqual(TEXT("Front samples keep zero collision distance on the donor side"), Samples[0].CollisionDistanceKm, 0.0f);
	TestEqual(TEXT("Front samples keep zero collision distance on the receiver side"), Samples[1].CollisionDistanceKm, 0.0f);
	TestTrue(TEXT("Front seed is flagged on the transferred donor sample"), Samples[0].bIsCollisionFront);
	TestTrue(TEXT("Front seed is flagged on the receiver contact sample"), Samples[1].bIsCollisionFront);
	TestFalse(TEXT("Interior propagation samples are not marked as front seeds"), Samples[2].bIsCollisionFront);
	TestTrue(TEXT("Interior receiver sample receives compact-support uplift"), Samples[2].Elevation > 0.0f);
	TestTrue(TEXT("Interior receiver sample tracks a positive collision distance"), Samples[2].CollisionDistanceKm > 0.0f);
	TestTrue(TEXT("Receiver sample outside the collision radius stays unaffected"), FMath::IsNearlyEqual(Samples[3].Elevation, 0.0f, 1e-6f));
	TestTrue(TEXT("Receiver sample outside the collision radius keeps collision metadata cleared"), Samples[3].CollisionDistanceKm < 0.0f);

	const double ExpectedRadiusKm = FMath::Clamp(
		4200.0 * FMath::Sqrt(
			(static_cast<double>(FTectonicPlanetTestAccess::GetCollisionEventMeanConvergenceSpeed(Planet, 0)) / 100.0) *
			(FTectonicPlanetTestAccess::GetCollisionEventAreaKm2(Planet, 0) / 4000.0)),
		0.0,
		4200.0);
	TestTrue(
		TEXT("Collision radius uses InitialMeanPlateAreaKm2 normalization"),
		FMath::IsNearlyEqual(FTectonicPlanetTestAccess::GetCollisionEventRadiusKm(Planet, 0), static_cast<float>(ExpectedRadiusKm), 1e-3f));

	return true;
}

bool FCarriedWorkspaceTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FPlate>& Plates = Planet.GetPlates();

	int32 TotalCarried = 0;
	int32 CountMismatches = 0;
	int32 InvalidCarriedIndices = 0;
	int32 IndexOrderMismatches = 0;
	int32 FieldMismatches = 0;

	for (const FPlate& Plate : Plates)
	{
		CountMismatches += (Plate.CarriedSamples.Num() == Plate.SampleIndices.Num()) ? 0 : 1;
		TotalCarried += Plate.CarriedSamples.Num();

		const int32 NumToCheck = FMath::Min(Plate.CarriedSamples.Num(), Plate.SampleIndices.Num());
		for (int32 LocalIndex = 0; LocalIndex < NumToCheck; ++LocalIndex)
		{
			const FCarriedSampleData& Carried = Plate.CarriedSamples[LocalIndex];
			const int32 CanonicalIndex = Carried.CanonicalSampleIndex;
			if (!Samples.IsValidIndex(CanonicalIndex))
			{
				++InvalidCarriedIndices;
				continue;
			}

			IndexOrderMismatches += (Plate.SampleIndices[LocalIndex] == CanonicalIndex) ? 0 : 1;

			const FCanonicalSample& Canonical = Samples[CanonicalIndex];
			const bool bMatches =
				FMath::IsNearlyEqual(Carried.Elevation, Canonical.Elevation) &&
				FMath::IsNearlyEqual(Carried.Thickness, Canonical.Thickness) &&
				FMath::IsNearlyEqual(Carried.Age, Canonical.Age) &&
				Carried.RidgeDirection.Equals(Canonical.RidgeDirection, 1e-6) &&
				Carried.FoldDirection.Equals(Canonical.FoldDirection, 1e-6) &&
				Carried.OrogenyType == Canonical.OrogenyType &&
				FMath::IsNearlyEqual(Carried.OrogenyAge, Canonical.OrogenyAge) &&
				Carried.CrustType == Canonical.CrustType &&
				Carried.SubductionRole == Canonical.SubductionRole &&
				Carried.SubductionOpposingPlateId == Canonical.SubductionOpposingPlateId &&
				FMath::IsNearlyEqual(Carried.SubductionDistanceKm, Canonical.SubductionDistanceKm) &&
				FMath::IsNearlyEqual(Carried.SubductionConvergenceSpeedMmPerYear, Canonical.SubductionConvergenceSpeedMmPerYear) &&
				Carried.bIsSubductionFront == Canonical.bIsSubductionFront &&
				FMath::IsNearlyEqual(Carried.CollisionDistanceKm, Canonical.CollisionDistanceKm) &&
				FMath::IsNearlyEqual(Carried.CollisionConvergenceSpeedMmPerYear, Canonical.CollisionConvergenceSpeedMmPerYear) &&
				Carried.CollisionOpposingPlateId == Canonical.CollisionOpposingPlateId &&
				FMath::IsNearlyEqual(Carried.CollisionInfluenceRadiusKm, Canonical.CollisionInfluenceRadiusKm) &&
				Carried.bIsCollisionFront == Canonical.bIsCollisionFront;
			FieldMismatches += bMatches ? 0 : 1;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Carried workspace metrics (500k): TotalCarried=%d CountMismatch=%d InvalidIndices=%d IndexOrderMismatch=%d FieldMismatch=%d"),
		TotalCarried, CountMismatches, InvalidCarriedIndices, IndexOrderMismatches, FieldMismatches);

	TestEqual(TEXT("Total carried samples matches canonical sample count"), TotalCarried, Samples.Num());
	TestEqual(TEXT("Per-plate carried sample count mismatches"), CountMismatches, 0);
	TestEqual(TEXT("Invalid carried canonical indices"), InvalidCarriedIndices, 0);
	TestEqual(TEXT("Carried/sample index order mismatches"), IndexOrderMismatches, 0);
	TestEqual(TEXT("Carried field mismatches"), FieldMismatches, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlateMotionTriggerTest, "Aurous.TectonicPlanet.PlateMotionTrigger",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPlateMotionTriggerTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	TestTrue(TEXT("Threshold positive"), Planet.GetReconcileDisplacementThreshold() > 0.0);
	TestFalse(TEXT("ShouldReconcile false immediately after plate init"), Planet.ShouldReconcile());
	TestEqual(TEXT("Timestep counter unchanged before StepSimulation"), Planet.GetTimestepCounter(), static_cast<int64>(0));

	const int32 Steps = 2;
	for (int32 StepIndex = 0; StepIndex < Steps; ++StepIndex)
	{
		Planet.AdvancePlateMotionStep();
	}

	TestTrue(TEXT("ShouldReconcile true after accumulated motion exceeds threshold"), Planet.ShouldReconcile());
	TestTrue(TEXT("Max displacement exceeds threshold"),
		Planet.GetMaxAngularDisplacementSinceReconcile() > Planet.GetReconcileDisplacementThreshold());

	int32 RotationMismatchCount = 0;
	int32 InvalidRotationNormCount = 0;
	int32 DisplacementMismatchCount = 0;
	int32 OnDemandPositionMismatchCount = 0;

	for (const FPlate& Plate : Planet.GetPlates())
	{
		const double ExpectedAngle = FMath::Abs(static_cast<double>(Plate.AngularSpeed)) * static_cast<double>(Steps);
		const double ActualAngle = static_cast<double>(Plate.CumulativeRotation.AngularDistance(FQuat::Identity));
		RotationMismatchCount += FMath::IsNearlyEqual(ActualAngle, ExpectedAngle, 1e-5) ? 0 : 1;
		InvalidRotationNormCount += FMath::IsNearlyEqual(static_cast<double>(Plate.CumulativeRotation.Size()), 1.0, 1e-6) ? 0 : 1;
		DisplacementMismatchCount += FMath::IsNearlyEqual(Plate.AngularDisplacementSinceReconcile, ExpectedAngle, 1e-8) ? 0 : 1;

		if (Plate.SampleIndices.Num() > 0)
		{
			const int32 SampleIndex = Plate.SampleIndices[0];
			const FVector RotatedViaPlanet = Planet.GetRotatedSamplePosition(Plate, SampleIndex);
			const FVector RotatedDirect = Plate.CumulativeRotation.RotateVector(Samples[SampleIndex].Position);
			OnDemandPositionMismatchCount += RotatedViaPlanet.Equals(RotatedDirect, 1e-8) ? 0 : 1;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Plate motion trigger metrics (500k): MaxDisp=%.6e Threshold=%.6e RotationMismatch=%d RotationNormMismatch=%d DisplacementMismatch=%d OnDemandPosMismatch=%d"),
		Planet.GetMaxAngularDisplacementSinceReconcile(),
		Planet.GetReconcileDisplacementThreshold(),
		RotationMismatchCount,
		InvalidRotationNormCount,
		DisplacementMismatchCount,
		OnDemandPositionMismatchCount);

	TestEqual(TEXT("Rotation accumulation mismatches"), RotationMismatchCount, 0);
	TestEqual(TEXT("Quaternion normalization mismatches"), InvalidRotationNormCount, 0);
	TestEqual(TEXT("Displacement accumulation mismatches"), DisplacementMismatchCount, 0);
	TestEqual(TEXT("On-demand rotated position mismatches"), OnDemandPositionMismatchCount, 0);

	Planet.ResetDisplacementTracking();
	TestFalse(TEXT("ShouldReconcile false after reset"), Planet.ShouldReconcile());
	TestTrue(TEXT("Max displacement near zero after reset"),
		FMath::IsNearlyZero(Planet.GetMaxAngularDisplacementSinceReconcile(), 1e-12));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStepSimulationTest, "Aurous.TectonicPlanet.StepSimulation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FStepSimulationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	TestEqual(TEXT("Initial timestep"), Planet.GetTimestepCounter(), static_cast<int64>(0));
	TestEqual(TEXT("Initial reconcile count"), Planet.GetReconcileCount(), 0);

	Planet.StepSimulation();

	TestEqual(TEXT("Timestep incremented"), Planet.GetTimestepCounter(), static_cast<int64>(1));
	TestTrue(TEXT("Reconcile stub invoked at least once"), Planet.GetReconcileCount() >= 1);
	TestFalse(TEXT("Displacement trigger cleared after StepSimulation reconcile"), Planet.ShouldReconcile());
	TestTrue(TEXT("Max displacement reset after StepSimulation"),
		FMath::IsNearlyZero(Planet.GetMaxAngularDisplacementSinceReconcile(), 1e-12));

	int32 ZeroRotationPlates = 0;
	int32 NonZeroPlateDisplacements = 0;
	for (const FPlate& Plate : Planet.GetPlates())
	{
		ZeroRotationPlates += Plate.CumulativeRotation.AngularDistance(FQuat::Identity) <= 1e-8 ? 1 : 0;
		NonZeroPlateDisplacements += Plate.AngularDisplacementSinceReconcile > 0.0 ? 1 : 0;
	}

	UE_LOG(LogTemp, Log, TEXT("StepSimulation metrics (500k): ReconcileCount=%d ZeroRotationPlates=%d NonZeroDisplacementsPostReset=%d"),
		Planet.GetReconcileCount(),
		ZeroRotationPlates,
		NonZeroPlateDisplacements);

	TestEqual(TEXT("All plates rotated after StepSimulation"), ZeroRotationPlates, 0);
	TestEqual(TEXT("Per-plate displacement reset after reconcile"), NonZeroPlateDisplacements, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOceanicAgeIncrementTest, "Aurous.TectonicPlanet.OceanicAgeIncrement",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FOceanicAgeIncrementTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const int32 NumSteps = 3;
	const float ExpectedAgeDelta = static_cast<float>(Planet.GetTimestepDurationMy() * static_cast<double>(NumSteps));

	const TArray<FPlate>& PlatesBefore = Planet.GetPlates();
	TArray<TArray<float>> InitialAgesByPlate;
	InitialAgesByPlate.SetNum(PlatesBefore.Num());
	int32 OceanicCarriedCount = 0;
	for (int32 PlateIndex = 0; PlateIndex < PlatesBefore.Num(); ++PlateIndex)
	{
		const FPlate& Plate = PlatesBefore[PlateIndex];
		TArray<float>& Ages = InitialAgesByPlate[PlateIndex];
		Ages.SetNumUninitialized(Plate.CarriedSamples.Num());
		for (int32 LocalIndex = 0; LocalIndex < Plate.CarriedSamples.Num(); ++LocalIndex)
		{
			const FCarriedSampleData& Carried = Plate.CarriedSamples[LocalIndex];
			Ages[LocalIndex] = Carried.Age;
			if (Carried.CrustType == ECrustType::Oceanic)
			{
				++OceanicCarriedCount;
			}
		}
	}

	for (int32 StepIndex = 0; StepIndex < NumSteps; ++StepIndex)
	{
		Planet.AdvancePlateMotionStep();
	}

	const TArray<FPlate>& PlatesAfter = Planet.GetPlates();
	int32 OceanicAgeMismatchCount = 0;
	int32 ContinentalAgeChangedCount = 0;
	for (int32 PlateIndex = 0; PlateIndex < PlatesAfter.Num(); ++PlateIndex)
	{
		const FPlate& Plate = PlatesAfter[PlateIndex];
		const TArray<float>& InitialAges = InitialAgesByPlate[PlateIndex];
		for (int32 LocalIndex = 0; LocalIndex < Plate.CarriedSamples.Num(); ++LocalIndex)
		{
			const FCarriedSampleData& Carried = Plate.CarriedSamples[LocalIndex];
			const float InitialAge = InitialAges[LocalIndex];
			if (Carried.CrustType == ECrustType::Oceanic)
			{
				const float ExpectedAge = InitialAge + ExpectedAgeDelta;
				OceanicAgeMismatchCount += FMath::IsNearlyEqual(Carried.Age, ExpectedAge, 1e-4f) ? 0 : 1;
			}
			else
			{
				ContinentalAgeChangedCount += FMath::IsNearlyEqual(Carried.Age, InitialAge, 1e-6f) ? 0 : 1;
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Oceanic age increment metrics (500k): Steps=%d OceanicCarried=%d OceanicAgeMismatch=%d ContinentalAgeChanged=%d"),
		NumSteps, OceanicCarriedCount, OceanicAgeMismatchCount, ContinentalAgeChangedCount);

	TestTrue(TEXT("Found oceanic carried samples"), OceanicCarriedCount > 0);
	TestEqual(TEXT("Oceanic ages increment by timestep duration"), OceanicAgeMismatchCount, 0);
	TestEqual(TEXT("Continental ages remain unchanged"), ContinentalAgeChangedCount, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOceanicDampeningFormulaTest, "Aurous.TectonicPlanet.OceanicDampeningFormula",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FOceanicDampeningFormulaTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	TArray<FPlate>& MutablePlates = Planet.GetPlates();
	FCarriedSampleData* TargetOceanic = nullptr;
	for (FPlate& Plate : MutablePlates)
	{
		for (FCarriedSampleData& Carried : Plate.CarriedSamples)
		{
			if (Carried.CrustType == ECrustType::Oceanic)
			{
				TargetOceanic = &Carried;
				break;
			}
		}

		if (TargetOceanic)
		{
			break;
		}
	}

	TestNotNull(TEXT("Found target oceanic carried sample"), TargetOceanic);
	if (!TargetOceanic)
	{
		return false;
	}

	const float InitialElevationKm = -5.0f;
	const float InitialAgeMy = 12.0f;
	TargetOceanic->Elevation = InitialElevationKm;
	TargetOceanic->Age = InitialAgeMy;

	const double DampeningStepKm = static_cast<double>(Planet.GetOceanicDampeningRateMmPerYear()) * 1.0e-6 * Planet.GetTimestepDurationYears();
	const double TrenchElevationKm = static_cast<double>(Planet.GetOceanicTrenchElevation());
	const double ExpectedElevationKm = static_cast<double>(InitialElevationKm) -
		(1.0 - static_cast<double>(InitialElevationKm) / TrenchElevationKm) * DampeningStepKm;
	const float ExpectedAgeMy = InitialAgeMy + static_cast<float>(Planet.GetTimestepDurationMy());

	Planet.AdvancePlateMotionStep();

	const float ActualElevationKm = TargetOceanic->Elevation;
	const float ActualAgeMy = TargetOceanic->Age;
	UE_LOG(LogTemp, Log, TEXT("Oceanic dampening formula metrics (500k): Initial=%.6f Expected=%.6f Actual=%.6f DampeningStepKm=%.6f ExpectedAge=%.3f ActualAge=%.3f"),
		InitialElevationKm,
		static_cast<float>(ExpectedElevationKm),
		ActualElevationKm,
		static_cast<float>(DampeningStepKm),
		ExpectedAgeMy,
		ActualAgeMy);

	TestTrue(TEXT("Oceanic dampening matches analytic one-step formula"), FMath::IsNearlyEqual(ActualElevationKm, static_cast<float>(ExpectedElevationKm), 1e-5f));
	TestTrue(TEXT("Oceanic age increments during dampening step"), FMath::IsNearlyEqual(ActualAgeMy, ExpectedAgeMy, 1e-5f));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOceanicDampeningAsymptoteTest, "Aurous.TectonicPlanet.OceanicDampeningAsymptote",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FOceanicDampeningAsymptoteTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	TArray<FPlate>& MutablePlates = Planet.GetPlates();
	FCarriedSampleData* TargetOceanic = nullptr;
	for (FPlate& Plate : MutablePlates)
	{
		for (FCarriedSampleData& Carried : Plate.CarriedSamples)
		{
			if (Carried.CrustType == ECrustType::Oceanic)
			{
				TargetOceanic = &Carried;
				break;
			}
		}

		if (TargetOceanic)
		{
			break;
		}
	}

	TestNotNull(TEXT("Found oceanic carried sample for asymptote test"), TargetOceanic);
	if (!TargetOceanic)
	{
		return false;
	}

	const float TrenchElevation = Planet.GetOceanicTrenchElevation();
	TargetOceanic->Elevation = TrenchElevation;
	const float InitialAge = TargetOceanic->Age;

	Planet.AdvancePlateMotionStep();

	UE_LOG(LogTemp, Log, TEXT("Oceanic asymptote metrics (500k): trench=%.6f after=%.6f"),
		TrenchElevation,
		TargetOceanic->Elevation);

	TestTrue(TEXT("Oceanic trench elevation is a fixed point of dampening"), FMath::IsNearlyEqual(TargetOceanic->Elevation, TrenchElevation, 1e-6f));
	TestTrue(TEXT("Oceanic age still increments at trench elevation"),
		FMath::IsNearlyEqual(TargetOceanic->Age, InitialAge + static_cast<float>(Planet.GetTimestepDurationMy()), 1e-5f));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpatialSoupConstructionTest, "Aurous.TectonicPlanet.SpatialSoupConstruction",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FSpatialSoupConstructionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);
	Planet.AdvancePlateMotionStep();
	Planet.RebuildSpatialQueryData();

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FPlate>& Plates = Planet.GetPlates();

	int32 MissingSoupState = 0;
	int32 TriangleCountMismatches = 0;
	int32 MissingCapState = 0;
	int32 InvalidCapCosine = 0;
	int32 MissingRotatedVertexChecks = 0;
	int32 RotatedVertexMismatches = 0;
	int32 TotalCheckedVertices = 0;

	for (const FPlate& Plate : Plates)
	{
		int32 SoupTriangleCount = 0;
		int32 SoupVertexCount = 0;
		if (!Planet.GetPlateSoupTriangleAndVertexCounts(Plate.Id, SoupTriangleCount, SoupVertexCount))
		{
			++MissingSoupState;
			continue;
		}

		const int32 MinExpectedSoupTriangles = Plate.InteriorTriangles.Num() + Plate.BoundaryTriangles.Num();
		const int32 MaxExpectedSoupTriangles = Plate.InteriorTriangles.Num() + (2 * Plate.BoundaryTriangles.Num());
		TriangleCountMismatches += (SoupTriangleCount >= MinExpectedSoupTriangles && SoupTriangleCount <= MaxExpectedSoupTriangles) ? 0 : 1;
		FVector CapCenter = FVector::ZeroVector;
		double CapCosHalfAngle = -2.0;
		if (!Planet.GetPlateBoundingCap(Plate.Id, CapCenter, CapCosHalfAngle))
		{
			++MissingCapState;
		}
		else
		{
			InvalidCapCosine += (CapCosHalfAngle < -1.0 || CapCosHalfAngle > 1.0) ? 1 : 0;
		}

		int32 CheckedForPlate = 0;
		for (const int32 SampleIndex : Plate.SampleIndices)
		{
			if (!Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			FVector SoupRotatedPosition = FVector::ZeroVector;
			if (!Planet.GetPlateSoupRotatedVertex(Plate.Id, SampleIndex, SoupRotatedPosition))
			{
				continue;
			}

			const FVector ExpectedPosition = Planet.GetRotatedSamplePosition(Plate, SampleIndex);
			RotatedVertexMismatches += SoupRotatedPosition.Equals(ExpectedPosition, 1e-8) ? 0 : 1;
			++CheckedForPlate;
			++TotalCheckedVertices;

			if (CheckedForPlate >= 3)
			{
				break;
			}
		}

		MissingRotatedVertexChecks += (CheckedForPlate == 0) ? 1 : 0;
	}

	UE_LOG(LogTemp, Log, TEXT("Spatial soup metrics (500k): MissingSoup=%d TriangleMismatch=%d MissingCap=%d InvalidCapCos=%d MissingVertexChecks=%d RotatedVertexMismatch=%d CheckedVertices=%d"),
		MissingSoupState,
		TriangleCountMismatches,
		MissingCapState,
		InvalidCapCosine,
		MissingRotatedVertexChecks,
		RotatedVertexMismatches,
		TotalCheckedVertices);

	TestEqual(TEXT("All plates have soup state"), MissingSoupState, 0);
	TestEqual(TEXT("Soup triangle count stays within split boundary-triangle bounds"), TriangleCountMismatches, 0);
	TestEqual(TEXT("All plates have bounding caps"), MissingCapState, 0);
	TestEqual(TEXT("Bounding cap cosine in valid range"), InvalidCapCosine, 0);
	TestEqual(TEXT("Each plate had at least one soup vertex check"), MissingRotatedVertexChecks, 0);
	TestEqual(TEXT("Rotated soup vertex mismatches"), RotatedVertexMismatches, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContainmentQueryTest, "Aurous.TectonicPlanet.ContainmentQuery",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FContainmentQueryTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	for (int32 StepIndex = 0; StepIndex < 3; ++StepIndex)
	{
		Planet.AdvancePlateMotionStep();
	}
	Planet.RebuildSpatialQueryData();

	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();
	const TArray<FPlate>& Plates = Planet.GetPlates();

	int32 NumPlateQueries = 0;
	int32 MissingContainment = 0;
	int32 PlateIdMismatches = 0;
	int32 InvalidBarycentrics = 0;
	int32 MissingInteriorTriangles = 0;
	int32 OverlapAcceptedMatches = 0;

	for (const FPlate& Plate : Plates)
	{
		if (Plate.InteriorTriangles.Num() == 0)
		{
			++MissingInteriorTriangles;
			continue;
		}

		const int32 TriangleIndex = Plate.InteriorTriangles[0];
		if (!Triangles.IsValidIndex(TriangleIndex))
		{
			++MissingContainment;
			continue;
		}

		const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
		const FVector A = Planet.GetRotatedSamplePosition(Plate, Triangle.V[0]);
		const FVector B = Planet.GetRotatedSamplePosition(Plate, Triangle.V[1]);
		const FVector C = Planet.GetRotatedSamplePosition(Plate, Triangle.V[2]);
		const FVector QueryPosition = (A * 0.98 + B * 0.01 + C * 0.01).GetSafeNormal();

		const FContainmentQueryResult Result = Planet.QueryContainment(QueryPosition);
		++NumPlateQueries;

		if (!Result.bFoundContainingPlate)
		{
			++MissingContainment;
			continue;
		}

		bool bPlateMatch = (Result.PlateId == Plate.Id);
		if (!bPlateMatch && Result.bOverlap)
		{
			for (int32 OverlapIndex = 0; OverlapIndex < Result.NumContainingPlates; ++OverlapIndex)
			{
				if (Result.ContainingPlateIds[OverlapIndex] == Plate.Id)
				{
					bPlateMatch = true;
					++OverlapAcceptedMatches;
					break;
				}
			}
		}

		PlateIdMismatches += bPlateMatch ? 0 : 1;
		const double BarySum = Result.Barycentric.X + Result.Barycentric.Y + Result.Barycentric.Z;
		const bool bValidBarycentric =
			FMath::IsNearlyEqual(BarySum, 1.0, 1e-4) &&
			Result.Barycentric.X >= -1e-3 &&
			Result.Barycentric.Y >= -1e-3 &&
			Result.Barycentric.Z >= -1e-3;
		InvalidBarycentrics += bValidBarycentric ? 0 : 1;
	}

	UE_LOG(LogTemp, Log, TEXT("Containment metrics (500k): PlateQueries=%d Missing=%d PlateMismatch=%d InvalidBary=%d MissingInterior=%d OverlapAccepted=%d"),
		NumPlateQueries,
		MissingContainment,
		PlateIdMismatches,
		InvalidBarycentrics,
		MissingInteriorTriangles,
		OverlapAcceptedMatches);

	TestTrue(TEXT("Executed plate containment queries"), NumPlateQueries > 0);
	TestEqual(TEXT("Every plate had at least one interior triangle"), MissingInteriorTriangles, 0);
	TestEqual(TEXT("Missing containment results"), MissingContainment, 0);
	TestEqual(TEXT("Plate ID mismatches"), PlateIdMismatches, 0);
	TestEqual(TEXT("Invalid barycentric results"), InvalidBarycentrics, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContainmentBroadPhaseTest, "Aurous.TectonicPlanet.ContainmentBroadPhase",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FContainmentBroadPhaseTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);
	Planet.RebuildSpatialQueryData();

	const int32 NumPlates = Planet.GetPlates().Num();
	FRandomStream Random(1337);

	const int32 NumQueries = 1024;
	int32 NumQueriesWithCull = 0;
	int32 GapCount = 0;
	int32 OverlapCount = 0;
	int32 InvalidCandidateCounts = 0;
	int64 TotalCandidates = 0;

	for (int32 QueryIndex = 0; QueryIndex < NumQueries; ++QueryIndex)
	{
		const FVector QueryDirection = MakeRandomUnitVector(Random);
		const FContainmentQueryResult Result = Planet.QueryContainment(QueryDirection);
		InvalidCandidateCounts += (Result.NumCapCandidates < 0 || Result.NumCapCandidates > NumPlates) ? 1 : 0;
		NumQueriesWithCull += (Result.NumCapCandidates < NumPlates) ? 1 : 0;
		GapCount += Result.bGap ? 1 : 0;
		OverlapCount += Result.bOverlap ? 1 : 0;
		TotalCandidates += Result.NumCapCandidates;
	}

	const double AverageCandidates = static_cast<double>(TotalCandidates) / static_cast<double>(NumQueries);
	const double GapRate = static_cast<double>(GapCount) / static_cast<double>(NumQueries);
	const double OverlapRate = static_cast<double>(OverlapCount) / static_cast<double>(NumQueries);
	UE_LOG(LogTemp, Log, TEXT("Containment broad-phase metrics (500k): Queries=%d AvgCapCandidates=%.3f NumWithCull=%d GapCount=%d (%.3f%%) OverlapCount=%d (%.3f%%) InvalidCandidateCounts=%d"),
		NumQueries,
		AverageCandidates,
		NumQueriesWithCull,
		GapCount,
		GapRate * 100.0,
		OverlapCount,
		OverlapRate * 100.0,
		InvalidCandidateCounts);

	TestEqual(TEXT("Invalid cap-candidate count results"), InvalidCandidateCounts, 0);
	TestTrue(TEXT("Broad phase culled at least one plate in some queries"), NumQueriesWithCull > 0);
	TestTrue(TEXT("Average broad-phase candidates is below total plate count"), AverageCandidates < static_cast<double>(NumPlates));
	TestTrue(TEXT("Random broad-phase gap query rate remains bounded"), GapRate <= MaxAcceptableGapFraction);
	TestTrue(TEXT("Random broad-phase overlap query rate remains bounded"), OverlapRate <= MaxAcceptableGapFraction);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExportOwnershipHelperTest, "Aurous.TectonicPlanet.ExportOwnershipHelper",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FExportOwnershipHelperTest::RunTest(const FString& Parameters)
{
	const int32 DuplicatePlateIds[3] = { 0, 1, 0 };
	const int32 DuplicateWinner = TectonicPlanetOwnership::ResolveWeightedPlateOwnerFromPlateIds(DuplicatePlateIds, FVector3d(0.40, 0.35, 0.25));
	TestEqual(TEXT("Weighted owner accumulates duplicate plate barycentric weights"), DuplicateWinner, 0);

	const int32 TiePlateIds[3] = { 0, 1, 2 };
	const int32 TieWinner = TectonicPlanetOwnership::ResolveWeightedPlateOwnerFromPlateIds(TiePlateIds, FVector3d(0.50, 0.50, 0.0));
	TestEqual(TEXT("Weighted owner tie breaks to lower plate id"), TieWinner, 0);

	TArray<int32> ResolvedPlateIds = { 0, 0, 1, 0, 2, 2 };
	TArray<uint8> BoundaryMask;
	TectonicPlanetOwnership::BuildBoundaryMaskFromResolvedPlateIds(ResolvedPlateIds, 3, 2, BoundaryMask);
	TestEqual(TEXT("Boundary mask size matches plate grid"), BoundaryMask.Num(), 6);
	TestEqual(TEXT("Interior pixel stays clear"), static_cast<int32>(BoundaryMask[0]), 0);
	TestEqual(TEXT("Horizontal transition marks left boundary pixel"), static_cast<int32>(BoundaryMask[1]), 255);
	TestEqual(TEXT("Horizontal transition marks right boundary pixel"), static_cast<int32>(BoundaryMask[2]), 255);
	TestEqual(TEXT("Vertical transition marks lower boundary pixel"), static_cast<int32>(BoundaryMask[3]), 255);

	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FActorConstructionTest, "Aurous.TectonicPlanet.ActorConstruction",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FActorConstructionTest::RunTest(const FString& Parameters)
{
	UWorld* World = UWorld::CreateWorld(
		EWorldType::Game,
		false,
		FName(TEXT("AurousActorConstructionTestWorld")),
		nullptr,
		true);
	TestNotNull(TEXT("Transient world created"), World);
	if (!World)
	{
		return false;
	}

	ATectonicPlanetActor* Actor = NewObject<ATectonicPlanetActor>(World, NAME_None, RF_Transient);
	TestNotNull(TEXT("Actor created"), Actor);
	if (!Actor)
	{
		World->DestroyWorld(false);
		World->RemoveFromRoot();
		return false;
	}

	Actor->NumSamples = TestSampleCount;
	Actor->NumPlates = TestPlateCount;
	Actor->RandomSeed = TestSeed;

	URealtimeMeshComponent* MeshComponent = Cast<URealtimeMeshComponent>(Actor->GetRootComponent());
	TestNotNull(TEXT("Actor root is realtime mesh component"), MeshComponent);

	Actor->GeneratePlanet();

	URealtimeMeshSimple* RealtimeMesh = MeshComponent ? MeshComponent->GetRealtimeMeshAs<URealtimeMeshSimple>() : nullptr;
	TestNotNull(TEXT("RealtimeMeshSimple created after GeneratePlanet"), RealtimeMesh);

	int32 PositionVertexCount = -1;
	int32 TriangleCount = -1;
	int32 InvertedTriangleCount = 0;
	int32 SectionGroupCount = 0;
	if (RealtimeMesh)
	{
		const TArray<FRealtimeMeshSectionGroupKey> SectionGroups = RealtimeMesh->GetSectionGroups(FRealtimeMeshLODKey(0));
		SectionGroupCount = SectionGroups.Num();
		TestTrue(TEXT("Has at least one section group"), SectionGroupCount > 0);

		if (SectionGroups.Num() > 0)
		{
			RealtimeMesh->ProcessMesh(SectionGroups[0],
				[&PositionVertexCount, &TriangleCount, &InvertedTriangleCount](const RealtimeMesh::FRealtimeMeshStreamSet& Streams)
				{
					const RealtimeMesh::FRealtimeMeshStream* PositionStream = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::Position);
					const RealtimeMesh::FRealtimeMeshStream* TriangleStream = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::Triangles);
					PositionVertexCount = PositionStream ? PositionStream->Num() : -1;
					TriangleCount = TriangleStream ? TriangleStream->Num() : -1;
					if (!PositionStream || !TriangleStream)
					{
						return;
					}

					const TConstArrayView<const FVector3f> Positions = PositionStream->GetArrayView<FVector3f>();
					const TConstArrayView<const RealtimeMesh::TIndex3<uint32>> MeshTriangles = TriangleStream->GetArrayView<RealtimeMesh::TIndex3<uint32>>();
					for (const RealtimeMesh::TIndex3<uint32>& Triangle : MeshTriangles)
					{
						if (!Positions.IsValidIndex(static_cast<int32>(Triangle.V0)) ||
							!Positions.IsValidIndex(static_cast<int32>(Triangle.V1)) ||
							!Positions.IsValidIndex(static_cast<int32>(Triangle.V2)))
						{
							++InvertedTriangleCount;
							continue;
						}

						const FVector3d A(Positions[Triangle.V0]);
						const FVector3d B(Positions[Triangle.V1]);
						const FVector3d C(Positions[Triangle.V2]);
						const FVector3d FaceNormal = FVector3d::CrossProduct(B - A, C - A);
						const FVector3d FaceCentroid = (A + B + C) / 3.0;
						if (FaceNormal.Dot(FaceCentroid) >= 0.0)
						{
							++InvertedTriangleCount;
						}
					}
				});
		}
	}

	TArray<FCanonicalSample> ExportedSamples;
	TArray<FDelaunayTriangle> ExportedTriangles;
	int64 ExportedTimestep = -1;
	const bool bExportSucceeded = Actor->GetPlanetExportDataThreadSafe(ExportedSamples, ExportedTriangles, ExportedTimestep);
	TestTrue(TEXT("Actor export snapshot available after GeneratePlanet"), bExportSucceeded);

	FPlanetControlPanelMetricsSnapshot MetricsSnapshot;
	const bool bMetricsSnapshotSucceeded = Actor->GetControlPanelMetricsSnapshotThreadSafe(MetricsSnapshot);
	TestTrue(TEXT("Actor metrics snapshot available after GeneratePlanet"), bMetricsSnapshotSucceeded);

	int32 ExportedContinentalSampleCount = 0;
	TSet<int32> ExportedContinentalPlateIds;
	for (const FCanonicalSample& Sample : ExportedSamples)
	{
		if (Sample.CrustType != ECrustType::Continental)
		{
			continue;
		}

		++ExportedContinentalSampleCount;
		ExportedContinentalPlateIds.Add(Sample.PlateId);
	}
	const double ExportedContinentalAreaFraction = (ExportedSamples.Num() > 0)
		? static_cast<double>(ExportedContinentalSampleCount) / static_cast<double>(ExportedSamples.Num())
		: 0.0;

	if (bMetricsSnapshotSucceeded)
	{
		TestEqual(TEXT("Actor metrics snapshot sample count matches export"), MetricsSnapshot.SampleCount, ExportedSamples.Num());
		TestEqual(TEXT("Actor metrics snapshot continental plate count matches export"), MetricsSnapshot.ContinentalPlateCount, ExportedContinentalPlateIds.Num());
		TestTrue(TEXT("Actor metrics snapshot continental area fraction matches export"), FMath::IsNearlyEqual(MetricsSnapshot.ContinentalAreaFraction, ExportedContinentalAreaFraction, UE_DOUBLE_SMALL_NUMBER));
	}
	TestEqual(TEXT("RealtimeMesh vertex count matches sample count"), PositionVertexCount, TestSampleCount);
	TestEqual(TEXT("RealtimeMesh triangle count matches exported triangle count"), TriangleCount, ExportedTriangles.Num());
	TestEqual(TEXT("RealtimeMesh triangles face outward"), InvertedTriangleCount, 0);

	if (Actor)
	{
		const EPlanetDebugMode Modes[] = {
			EPlanetDebugMode::PlateId,
			EPlanetDebugMode::CrustType,
			EPlanetDebugMode::Boundary,
			EPlanetDebugMode::BoundaryType,
			EPlanetDebugMode::Elevation,
			EPlanetDebugMode::CrustAge,
			EPlanetDebugMode::SubductionRole,
			EPlanetDebugMode::SubductionDistance,
			EPlanetDebugMode::OrogenyType,
			EPlanetDebugMode::TerraneId
		};
		for (EPlanetDebugMode Mode : Modes)
		{
			Actor->DebugMode = Mode;
			Actor->UpdateDebugColors();
		}
	}

	World->DestroyWorld(false);
	World->RemoveFromRoot();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContinentalStabilizerPromotionTieBreakFixtureTest, "Aurous.TectonicPlanet.ContinentalStabilizerPromotionTieBreakFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContinentalStabilizerZeroCurrentFallbackFixtureTest, "Aurous.TectonicPlanet.ContinentalStabilizerZeroCurrentFallbackFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContinentalStabilizerGapFixtureTest, "Aurous.TectonicPlanet.ContinentalStabilizerGapFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContinentalStabilizerComponentPruneOrderFixtureTest, "Aurous.TectonicPlanet.ContinentalStabilizerComponentPruneOrderFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContinentalStabilizerDemotionPrecedenceFixtureTest, "Aurous.TectonicPlanet.ContinentalStabilizerDemotionPrecedenceFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContinentalStabilizerDirectParityTest, "Aurous.TectonicPlanet.ContinentalStabilizerDirectParity",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReconcileLongRunTest, "Aurous.TectonicPlanet.ReconcileLongRun",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FProtectedPlateConnectivityFloorTest, "Aurous.TectonicPlanet.ProtectedPlateConnectivityFloor",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FProtectedPlateRescueFixtureTest, "Aurous.TectonicPlanet.ProtectedPlateRescueFixture",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)


bool FContinentalStabilizerPromotionTieBreakFixtureTest::RunTest(const FString& Parameters)
{
	{
		FStabilizerFixtureState Fixture = MakePromotionTieBreakFixture(false);
		FReconcilePhaseTimings IncrementalTimings;
		TArray<FCanonicalSample> IncrementalSamples;
		RunStabilizerFixture(Fixture, &IncrementalSamples, &IncrementalTimings);
		TestEqual(TEXT("Previous-neighbor tie-break promotes sample 1"), IncrementalSamples[1].CrustType, ECrustType::Continental);
		TestEqual(TEXT("Previous-neighbor tie-break leaves sample 2 oceanic"), IncrementalSamples[2].CrustType, ECrustType::Oceanic);
		TestEqual(TEXT("Only one promotion occurs in previous-neighbor tie-break fixture"), IncrementalTimings.StabilizerPromotionCount, 1);
	}

	{
		FStabilizerFixtureState Fixture = MakePromotionTieBreakFixture(true);
		TArray<FCanonicalSample> IncrementalSamples = Fixture.CurrentSamples;
		FReconcilePhaseTimings IncrementalTimings;
		FTectonicPlanetTestAccess::StabilizeContinentalIncremental(Fixture.Planet, Fixture.PreviousSamples, IncrementalSamples, &IncrementalTimings);
		TestEqual(TEXT("Sample-index tie-break promotes lower index sample 1"), IncrementalSamples[1].CrustType, ECrustType::Continental);
		TestEqual(TEXT("Sample-index tie-break leaves sample 2 oceanic"), IncrementalSamples[2].CrustType, ECrustType::Oceanic);
		TestEqual(TEXT("Only one promotion occurs in sample-index tie-break fixture"), IncrementalTimings.StabilizerPromotionCount, 1);
	}

	return true;
}

bool FContinentalStabilizerZeroCurrentFallbackFixtureTest::RunTest(const FString& Parameters)
{
	FStabilizerFixtureState Fixture = MakeZeroCurrentFallbackFixture();
	FReconcilePhaseTimings IncrementalTimings;
	TArray<FCanonicalSample> IncrementalSamples;
	RunStabilizerFixture(Fixture, &IncrementalSamples, &IncrementalTimings);
	TestEqual(TEXT("Zero-current fallback promotes sample 2"), IncrementalSamples[2].CrustType, ECrustType::Continental);
	TestEqual(TEXT("Zero-current fallback keeps sample 1 oceanic"), IncrementalSamples[1].CrustType, ECrustType::Oceanic);
	TestEqual(TEXT("Zero-current fallback keeps sample 3 oceanic"), IncrementalSamples[3].CrustType, ECrustType::Oceanic);
	TestEqual(TEXT("Zero-current fallback creates exactly one continental sample"), CountContinentalSamples(IncrementalSamples), 1);
	TestEqual(TEXT("Zero-current fallback uses one promotion"), IncrementalTimings.StabilizerPromotionCount, 1);
	return true;
}

bool FContinentalStabilizerGapFixtureTest::RunTest(const FString& Parameters)
{
	FStabilizerFixtureState Fixture = MakeGapPromotionFixture();
	FReconcilePhaseTimings IncrementalTimings;
	TArray<FCanonicalSample> IncrementalSamples;
	RunStabilizerFixture(Fixture, &IncrementalSamples, &IncrementalTimings);
	TestEqual(TEXT("Gap-marked continental still counts toward the floor"), CountContinentalSamples(IncrementalSamples), 2);
	TestTrue(TEXT("Gap-marked continental sample stays continental and flagged as gap"), IncrementalSamples[0].CrustType == ECrustType::Continental && IncrementalSamples[0].bGapDetected);
	TestEqual(TEXT("Gap sample does not provide current-neighbor adjacency to sample 1"), IncrementalSamples[1].CrustType, ECrustType::Oceanic);
	TestEqual(TEXT("Fallback selects sample 2 instead"), IncrementalSamples[2].CrustType, ECrustType::Continental);
	TestEqual(TEXT("Gap fixture performs exactly one promotion"), IncrementalTimings.StabilizerPromotionCount, 1);
	return true;
}

bool FContinentalStabilizerComponentPruneOrderFixtureTest::RunTest(const FString& Parameters)
{
	FStabilizerFixtureState Fixture = MakeComponentPruneOrderFixture();
	FReconcilePhaseTimings IncrementalTimings;
	TArray<FCanonicalSample> IncrementalSamples;
	RunStabilizerFixture(Fixture, &IncrementalSamples, &IncrementalTimings);
	TestEqual(TEXT("Lowest-priority component sample 0 is demoted first"), IncrementalSamples[0].CrustType, ECrustType::Oceanic);
	TestEqual(TEXT("Later zero-priority component sample 2 remains continental"), IncrementalSamples[2].CrustType, ECrustType::Continental);
	TestEqual(TEXT("Higher-priority component sample 4 remains continental"), IncrementalSamples[4].CrustType, ECrustType::Continental);
	TestEqual(TEXT("Higher-priority component sample 6 remains continental"), IncrementalSamples[6].CrustType, ECrustType::Continental);
	TestEqual(TEXT("Exactly one component member is demoted"), IncrementalTimings.StabilizerComponentDemotionCount, 1);
	TestOceanicTemplateMatch(*this, TEXT("Component prune preserves previous oceanic template when demoting sample 0"), IncrementalSamples[0], Fixture.PreviousSamples[0]);
	return true;
}

bool FContinentalStabilizerDemotionPrecedenceFixtureTest::RunTest(const FString& Parameters)
{
	FStabilizerFixtureState Fixture = MakePreviousOceanicDemotionFixture();
	FReconcilePhaseTimings IncrementalTimings;
	TArray<FCanonicalSample> IncrementalSamples;
	RunStabilizerFixture(Fixture, &IncrementalSamples, &IncrementalTimings);
	TestEqual(TEXT("Target-driven prune demotes sample 0"), IncrementalSamples[0].CrustType, ECrustType::Oceanic);
	TestEqual(TEXT("Only one component demotion is recorded in precedence fixture"), IncrementalTimings.StabilizerComponentDemotionCount, 1);
	TestOceanicTemplateMatch(*this, TEXT("Previous oceanic template takes precedence over neighbor donor"), IncrementalSamples[0], Fixture.PreviousSamples[0]);
	return true;
}

bool FContinentalStabilizerDirectParityTest::RunTest(const FString& Parameters)
{
	TArray<FStabilizerFixtureState> Fixtures;
	Fixtures.Add(MakePromotionTieBreakFixture(false));
	Fixtures.Add(MakePromotionTieBreakFixture(true));
	Fixtures.Add(MakeZeroCurrentFallbackFixture());
	Fixtures.Add(MakeGapPromotionFixture());
	Fixtures.Add(MakeComponentPruneOrderFixture());
	Fixtures.Add(MakePreviousOceanicDemotionFixture());

	const TArray<FString> Labels = {
		TEXT("PromotionPreviousNeighbor"),
		TEXT("PromotionSampleIndex"),
		TEXT("ZeroCurrentFallback"),
		TEXT("GapBehavior"),
		TEXT("ComponentPruneOrder"),
		TEXT("PreviousOceanicDemotion")
	};

	for (int32 FixtureIndex = 0; FixtureIndex < Fixtures.Num(); ++FixtureIndex)
	{
		TArray<FCanonicalSample> IncrementalSamples;
		FReconcilePhaseTimings IncrementalTimings;
		RunStabilizerFixture(Fixtures[FixtureIndex], &IncrementalSamples, &IncrementalTimings);
		TestTrue(FString::Printf(TEXT("Incremental build-components timing is recorded [%s]"), *Labels[FixtureIndex]), IncrementalTimings.StabilizerBuildComponentsMs >= 0.0);
		TestTrue(FString::Printf(TEXT("Incremental trim timing is recorded [%s]"), *Labels[FixtureIndex]), IncrementalTimings.StabilizerTrimMs >= 0.0);
		TestTrue(FString::Printf(TEXT("Incremental heap pushes are recorded [%s]"), *Labels[FixtureIndex]), IncrementalTimings.StabilizerHeapPushCount > 0);
		TestTrue(FString::Printf(TEXT("Incremental stale heap pops are recorded [%s]"), *Labels[FixtureIndex]), IncrementalTimings.StabilizerHeapStalePopCount >= 0);
		TestTrue(FString::Printf(TEXT("Incremental sample output is produced [%s]"), *Labels[FixtureIndex]), IncrementalSamples.Num() > 0);
	}
	return true;
}

bool FProtectedPlateConnectivityFloorTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	constexpr int32 Plate0PrimaryCount = 30;
	constexpr int32 Plate0FragmentCount = 5;
	constexpr int32 Plate1Count = 5;
	constexpr int32 TotalSampleCount = Plate0PrimaryCount + Plate0FragmentCount + Plate1Count;

	Samples.SetNum(TotalSampleCount);
	Adjacency.SetNum(TotalSampleCount);
	Plates.SetNum(2);

	for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
	{
		Plates[PlateIndex] = FPlate{};
		Plates[PlateIndex].Id = PlateIndex;
		Plates[PlateIndex].PersistencePolicy = EPlatePersistencePolicy::Protected;
		Plates[PlateIndex].InitialSampleCount = (PlateIndex == 0) ? 640 : 256;
	}

	for (int32 SampleIndex = 0; SampleIndex < TotalSampleCount; ++SampleIndex)
	{
		Samples[SampleIndex].Position = FVector::ForwardVector;
		Samples[SampleIndex].PrevPlateId = (SampleIndex < (Plate0PrimaryCount + Plate0FragmentCount)) ? 0 : 1;
		Samples[SampleIndex].PlateId = Samples[SampleIndex].PrevPlateId;
		Samples[SampleIndex].bGapDetected = false;
	}

	for (int32 SampleIndex = 0; SampleIndex < Plate0PrimaryCount; ++SampleIndex)
	{
		if (SampleIndex > 0)
		{
			Adjacency[SampleIndex].Add(SampleIndex - 1);
		}
		if (SampleIndex + 1 < Plate0PrimaryCount)
		{
			Adjacency[SampleIndex].Add(SampleIndex + 1);
		}
	}

	for (int32 Offset = 0; Offset < Plate0FragmentCount; ++Offset)
	{
		const int32 SampleIndex = Plate0PrimaryCount + Offset;
		if (Offset > 0)
		{
			Adjacency[SampleIndex].Add(SampleIndex - 1);
		}
		if (Offset + 1 < Plate0FragmentCount)
		{
			Adjacency[SampleIndex].Add(SampleIndex + 1);
		}
	}

	for (int32 Offset = 0; Offset < Plate1Count; ++Offset)
	{
		const int32 SampleIndex = Plate0PrimaryCount + Plate0FragmentCount + Offset;
		if (Offset > 0)
		{
			Adjacency[SampleIndex].Add(SampleIndex - 1);
		}
		if (Offset + 1 < Plate1Count)
		{
			Adjacency[SampleIndex].Add(SampleIndex + 1);
		}
	}

	for (int32 FragmentIndex = Plate0PrimaryCount; FragmentIndex < Plate0PrimaryCount + Plate0FragmentCount; ++FragmentIndex)
	{
		Adjacency[FragmentIndex].Add(Plate0PrimaryCount + Plate0FragmentCount);
		Adjacency[Plate0PrimaryCount + Plate0FragmentCount].Add(FragmentIndex);
	}

	FTectonicPlanetTestAccess::EnforceConnectedOwnership(Planet, Samples);

	for (int32 SampleIndex = Plate0PrimaryCount; SampleIndex < Plate0PrimaryCount + Plate0FragmentCount; ++SampleIndex)
	{
		TestEqual(FString::Printf(TEXT("Protected floor keeps fragment sample %d on source plate"), SampleIndex), Samples[SampleIndex].PlateId, 0);
	}

	return true;
}

bool FProtectedPlateRescueFixtureTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet;
	TArray<FCanonicalSample>& Samples = FTectonicPlanetTestAccess::MutableSamples(Planet);
	TArray<TArray<int32>>& Adjacency = FTectonicPlanetTestAccess::MutableAdjacency(Planet);
	TArray<FPlate>& Plates = FTectonicPlanetTestAccess::MutablePlates(Planet);

	constexpr int32 Plate0PrimaryCount = 30;
	constexpr int32 Plate0FragmentCount = 5;
	constexpr int32 Plate1DonorCount = 20;
	constexpr int32 TotalSampleCount = Plate0PrimaryCount + Plate0FragmentCount + Plate1DonorCount;

	Samples.SetNum(TotalSampleCount);
	Adjacency.SetNum(TotalSampleCount);
	Plates.SetNum(2);

	Plates[0] = FPlate{};
	Plates[0].Id = 0;
	Plates[0].PersistencePolicy = EPlatePersistencePolicy::Protected;
	Plates[0].InitialSampleCount = 800;
	Plates[0].IdentityAnchorDirection = FVector::ForwardVector;
	Plates[0].CanonicalCenterDirection = FVector::ForwardVector;

	Plates[1] = FPlate{};
	Plates[1].Id = 1;
	Plates[1].PersistencePolicy = EPlatePersistencePolicy::Retirable;
	Plates[1].InitialSampleCount = 400;
	Plates[1].IdentityAnchorDirection = FVector::ForwardVector;
	Plates[1].CanonicalCenterDirection = FVector::ForwardVector;

	for (int32 SampleIndex = 0; SampleIndex < TotalSampleCount; ++SampleIndex)
	{
		Samples[SampleIndex].Position = FVector::ForwardVector;
		Samples[SampleIndex].PrevPlateId = (SampleIndex < (Plate0PrimaryCount + Plate0FragmentCount)) ? 0 : 1;
		Samples[SampleIndex].PlateId = Samples[SampleIndex].PrevPlateId;
		Samples[SampleIndex].bGapDetected = false;
		Samples[SampleIndex].OwnershipMargin = 0.0f;
	}

	for (int32 SampleIndex = 0; SampleIndex + 1 < Plate0PrimaryCount; ++SampleIndex)
	{
		LinkBidirectional(Adjacency, SampleIndex, SampleIndex + 1);
	}

	for (int32 Offset = 0; Offset + 1 < Plate0FragmentCount; ++Offset)
	{
		LinkBidirectional(Adjacency, Plate0PrimaryCount + Offset, Plate0PrimaryCount + Offset + 1);
	}

	for (int32 Offset = 0; Offset + 1 < Plate1DonorCount; ++Offset)
	{
		LinkBidirectional(Adjacency, Plate0PrimaryCount + Plate0FragmentCount + Offset, Plate0PrimaryCount + Plate0FragmentCount + Offset + 1);
	}

	const int32 DonorStartIndex = Plate0PrimaryCount + Plate0FragmentCount;
	LinkBidirectional(Adjacency, Plate0PrimaryCount - 1, DonorStartIndex);
	LinkBidirectional(Adjacency, Plate0PrimaryCount, DonorStartIndex + 10);

	FTectonicPlanetTestAccess::EnforceConnectedOwnership(Planet, Samples);
	for (int32 SampleIndex = Plate0PrimaryCount; SampleIndex < Plate0PrimaryCount + Plate0FragmentCount; ++SampleIndex)
	{
		TestEqual(FString::Printf(TEXT("Floor guard keeps fragment sample %d before rescue"), SampleIndex), Samples[SampleIndex].PlateId, 0);
	}

	TestTrue(TEXT("Rescue grows the protected plate"), FTectonicPlanetTestAccess::RescueProtectedOwnership(Planet, Samples));
	FTectonicPlanetTestAccess::EnforceConnectedOwnership(Planet, Samples);
	FTectonicPlanetTestAccess::RebuildPlateMembership(Planet, Samples);

	for (int32 SampleIndex = Plate0PrimaryCount; SampleIndex < Plate0PrimaryCount + Plate0FragmentCount; ++SampleIndex)
	{
		TestEqual(FString::Printf(TEXT("Fragment sample %d is released after rescue"), SampleIndex), Samples[SampleIndex].PlateId, 1);
	}

	TestTrue(
		TEXT("Protected plate is restored to its minimum floor after rescue"),
		Planet.GetMinProtectedPlateSampleCount() >= ComputeExpectedPersistentPlateFloorSamples(Plates[0]));
	TestEqual(TEXT("One protected plate required rescue"), Planet.GetRescuedProtectedPlateCount(), 1);
	TestTrue(TEXT("Rescue moved at least one sample"), Planet.GetRescuedProtectedSampleCount() > 0);
	return true;
}

bool FReconcileLongRunTest::RunTest(const FString& Parameters)
{
	for (const FReferenceScenarioDefinition& Scenario : GetLockedReferenceScenarios())
	{
		FReferenceScenarioObservedMetrics Metrics;
		const bool bCollected = CollectReferenceScenarioObservedMetrics(
			Scenario,
			GetCachedPlanet(Scenario.SampleCount),
			Metrics,
			0.0,
			EContinentalStabilizerMode::Incremental);
		const bool bPass = bCollected && DoesReferenceScenarioObservedMetricsPass(Scenario, Metrics);

		const FString ScenarioLabel = FString::Printf(
			TEXT("%s (%d samples, %d plates, seed=%d, steps=%d)"),
			Scenario.Name,
			Scenario.SampleCount,
			Scenario.PlateCount,
			Scenario.Seed,
			Scenario.NumSteps);

		UE_LOG(LogTemp, Log, TEXT("%s"), *FormatReferenceScenarioSummary(Scenario, Metrics, bPass));
		if (!bPass)
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Locked reference scenario [%s] failed checks: %s"),
				Scenario.Name,
				*DescribeReferenceScenarioFailures(Scenario, Metrics));
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Locked reference scenario [%s] failure details: %s"),
				Scenario.Name,
				*DescribeReferenceScenarioFailureDetails(Metrics));

			int32 DiscoveredSeed = INDEX_NONE;
			FReferenceScenarioObservedMetrics DiscoveredMetrics;
			if (TryDiscoverLowestPassingReferenceScenarioSeed(Scenario, GetCachedPlanet(Scenario.SampleCount), DiscoveredSeed, DiscoveredMetrics))
			{
				FReferenceScenarioDefinition DiscoveredScenario = Scenario;
				DiscoveredScenario.Seed = DiscoveredSeed;
				UE_LOG(
					LogTemp,
					Warning,
					TEXT("Locked reference scenario [%s] failed at seed=%d; lowest passing seed in [1,256] is %d. %s"),
					Scenario.Name,
					Scenario.Seed,
					DiscoveredSeed,
					*FormatReferenceScenarioSummary(DiscoveredScenario, DiscoveredMetrics, true));
			}
			else
			{
				UE_LOG(
					LogTemp,
					Warning,
					TEXT("Locked reference scenario [%s] failed at seed=%d and no passing seed was found in [1,256]."),
					Scenario.Name,
					Scenario.Seed);
			}
		}

		TestTrue(FString::Printf(TEXT("Reference metrics collect successfully [%s]"), *ScenarioLabel), bCollected);
		TestEqual(FString::Printf(TEXT("No stabilizer mismatches are reported [%s]"), *ScenarioLabel), Metrics.StabilizerShadowMismatchCount, 0);
		TestEqual(FString::Printf(TEXT("Unaccounted reconcile budget stays within 10%% [%s]"), *ScenarioLabel), Metrics.UnaccountedBudgetFailures, 0);
		TestTrue(FString::Printf(TEXT("Scenario wall time is recorded [%s]"), *ScenarioLabel), Metrics.ScenarioWallMs > 0.0);
		TestTrue(FString::Printf(TEXT("Average reconcile time is recorded [%s]"), *ScenarioLabel), Metrics.AverageReconcileMs > 0.0);
		TestTrue(FString::Printf(TEXT("Reconcile triggered repeatedly [%s]"), *ScenarioLabel), Metrics.ReconcileSteps >= 10);
		TestEqual(FString::Printf(TEXT("No orphan samples during long run [%s]"), *ScenarioLabel), Metrics.InvalidPlateAssignments, 0);
		TestEqual(FString::Printf(TEXT("PrevPlateId remains valid during long run [%s]"), *ScenarioLabel), Metrics.InvalidPreviousAssignments, 0);
		TestEqual(FString::Printf(TEXT("Per-phase timing snapshots are valid [%s]"), *ScenarioLabel), Metrics.InvalidTimingSnapshots, 0);
		TestEqual(FString::Printf(TEXT("Canonical positions are immutable across buffer swaps [%s]"), *ScenarioLabel), Metrics.PositionDriftCount, 0);
		TestEqual(FString::Printf(TEXT("Gap fallback does not produce orphan samples [%s]"), *ScenarioLabel), Metrics.GapOrphanMismatchCount, 0);
		TestTrue(FString::Printf(TEXT("Boundary ping-pong transition rate remains bounded [%s]"), *ScenarioLabel), Metrics.PingPongRate <= 0.10);
		TestTrue(FString::Printf(TEXT("Long-run gap fraction remains bounded [%s]"), *ScenarioLabel), Metrics.MaxGapFraction <= MaxAcceptableGapFraction);
		TestTrue(FString::Printf(TEXT("Long-run overlap fraction remains bounded [%s]"), *ScenarioLabel), Metrics.MaxOverlapFraction <= MaxAcceptableOverlapFraction);
		TestTrue(FString::Printf(TEXT("Boundary band mean depth remains narrow [%s]"), *ScenarioLabel), Metrics.MaxBoundaryMeanDepthHops <= MaxAcceptableBoundaryMeanDepthHops);
		TestTrue(FString::Printf(TEXT("Boundary band max depth remains narrow [%s]"), *ScenarioLabel), Metrics.MaxBoundaryMaxDepthHops <= MaxAcceptableBoundaryMaxDepthHops);
		TestEqual(FString::Printf(TEXT("Non-gap plate ownership remains single-component per plate [%s]"), *ScenarioLabel), Metrics.MaxNonGapPlateComponentCount, 1);
		TestEqual(FString::Printf(TEXT("No detached non-gap plate fragments remain [%s]"), *ScenarioLabel), Metrics.MaxDetachedPlateFragmentSampleCount, 0);
		TestEqual(FString::Printf(TEXT("No detached fragment grows beyond zero samples [%s]"), *ScenarioLabel), Metrics.MaxLargestDetachedPlateFragmentSize, 0);
		TestEqual(FString::Printf(TEXT("Protected plates do not become empty during long run [%s]"), *ScenarioLabel), Metrics.MaxEmptyProtectedPlateCount, 0);
		TestTrue(
			FString::Printf(TEXT("Protected plates stay above their minimum floor during long run [%s]"), *ScenarioLabel),
			Metrics.MinObservedProtectedPlateSampleCount >= Metrics.MinExpectedProtectedPlateFloor);

		TestTrue(
			FString::Printf(TEXT("Continental area fraction never increases after init [%s]"), *ScenarioLabel),
			!Metrics.bRuntimeContinentalFractionIncreased);
		TestTrue(
			FString::Printf(TEXT("Minimum runtime continental area fraction stays above floor [%s]"), *ScenarioLabel),
			Metrics.MinimumRuntimeContinentalAreaFraction + UE_DOUBLE_SMALL_NUMBER >= Scenario.Bounds.MinimumRuntimeContinentalAreaFraction);
		TestTrue(
			FString::Printf(TEXT("Maximum elevation stays below z_c [%s]"), *ScenarioLabel),
			Metrics.MaximumObservedElevationKm <= MaxReferenceScenarioElevationCeilingKm + static_cast<double>(KINDA_SMALL_NUMBER));
		TestTrue(
			FString::Printf(TEXT("Continental component count stays within scenario cap [%s]"), *ScenarioLabel),
			Metrics.MaximumContinentalComponentCount <= GetExpectedMaximumContinentalComponentCount(Scenario.Bounds));
		TestTrue(
			FString::Printf(TEXT("Collision event count meets scenario minimum [%s]"), *ScenarioLabel),
			Metrics.FinalCollisionEventCount >= Scenario.Bounds.MinimumCollisionEventCount);
		TestTrue(
			FString::Printf(TEXT("Andean samples appear when oceanic-continental convergence exists [%s]"), *ScenarioLabel),
			!Scenario.Bounds.bRequireAndeanWhenOceanicContinentalConvergenceExists ||
			!Metrics.bObservedOceanicContinentalConvergence ||
			Metrics.bObservedAndean);
		TestTrue(FString::Printf(TEXT("Scenario passes consolidated reference checks [%s]"), *ScenarioLabel), bPass);
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FActorSimulationThreadTest, "Aurous.TectonicPlanet.ActorSimulationThread",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FActorSimulationThreadTest::RunTest(const FString& Parameters)
{
	UWorld* World = UWorld::CreateWorld(
		EWorldType::Game,
		false,
		FName(TEXT("AurousActorSimulationThreadTestWorld")),
		nullptr,
		true);
	TestNotNull(TEXT("Transient world created"), World);
	if (!World)
	{
		return false;
	}

	ATectonicPlanetActor* Actor = NewObject<ATectonicPlanetActor>(World, NAME_None, RF_Transient);
	TestNotNull(TEXT("Actor created"), Actor);
	if (!Actor)
	{
		World->DestroyWorld(false);
		World->RemoveFromRoot();
		return false;
	}

	Actor->NumSamples = TestSampleCount;
	Actor->NumPlates = TestPlateCount;
	Actor->RandomSeed = TestSeed;
	Actor->SimulationTimestepsPerSecond = 30.0f;

	Actor->GeneratePlanet();
	const int64 InitialTimestep = Actor->GetBackgroundTimestepCounterThreadSafe();
	Actor->StartSimulation();

	bool bObservedTimestepAdvance = false;
	bool bObservedReconcileTrigger = false;
	for (int32 PollIndex = 0; PollIndex < 400; ++PollIndex)
	{
		FPlatformProcess::SleepNoStats(0.01f);
		const int64 CurrentTimestep = Actor->GetBackgroundTimestepCounterThreadSafe();
		const bool bBackgroundTriggered = Actor->IsBackgroundReconcileTriggeredThreadSafe();

		if (CurrentTimestep > InitialTimestep)
		{
			bObservedTimestepAdvance = true;
		}

		bObservedReconcileTrigger |= bBackgroundTriggered;

		if (PollIndex % 30 == 0)
		{
			Actor->UpdateDebugColors();
		}

		if (bObservedTimestepAdvance && bObservedReconcileTrigger)
		{
			break;
		}
	}

	Actor->StopSimulation();
	const int64 FinalTimestep = Actor->GetBackgroundTimestepCounterThreadSafe();
	FReconcilePhaseTimings LastTimings;
	int32 BackgroundReconcileCount = 0;
	const bool bHasReconcileTimings = Actor->GetLastReconcileTimingsThreadSafe(LastTimings, BackgroundReconcileCount);
	bObservedTimestepAdvance |= FinalTimestep > InitialTimestep;
	bObservedReconcileTrigger |= Actor->IsBackgroundReconcileTriggeredThreadSafe() || (bHasReconcileTimings && BackgroundReconcileCount > 0);

	UE_LOG(LogTemp, Log, TEXT("Actor simulation-thread metrics (500k): InitialStep=%lld FinalStep=%lld ObservedAdvance=%d ObservedReconcile=%d RunningAfterStop=%d"),
		InitialTimestep,
		FinalTimestep,
		bObservedTimestepAdvance ? 1 : 0,
		bObservedReconcileTrigger ? 1 : 0,
		Actor->IsSimulationThreadRunningThreadSafe() ? 1 : 0);

	TestTrue(TEXT("Background simulation advances timestep counter"), bObservedTimestepAdvance);
	TestTrue(TEXT("Background simulation triggers reconciliation"), bObservedReconcileTrigger);
	TestFalse(TEXT("Simulation thread reports stopped after StopSimulation"), Actor->IsSimulationThreadRunningThreadSafe());

	World->DestroyWorld(false);
	World->RemoveFromRoot();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlateMotionInvestigationTest, "Aurous.Diagnostics.PlateMotionInvestigation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPlateMotionInvestigationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& InitialSamples = Planet.GetSamples();
	TArray<int32> InitialPlateIds;
	InitialPlateIds.Reserve(InitialSamples.Num());
	TArray<int32> PreviousPlateIds;
	PreviousPlateIds.Reserve(InitialSamples.Num());
	for (const FCanonicalSample& Sample : InitialSamples)
	{
		InitialPlateIds.Add(Sample.PlateId);
		PreviousPlateIds.Add(Sample.PlateId);
	}

	TArray<int32> BoundaryWatchIndices;
	BoundaryWatchIndices.Reserve(10);
	for (int32 SampleIndex = 0; SampleIndex < InitialSamples.Num() && BoundaryWatchIndices.Num() < 10; ++SampleIndex)
	{
		if (InitialSamples[SampleIndex].bIsBoundary)
		{
			BoundaryWatchIndices.Add(SampleIndex);
		}
	}
	if (BoundaryWatchIndices.Num() == 0)
	{
		BoundaryWatchIndices.Add(0);
	}

	const int32 ContainmentWatchIndex = BoundaryWatchIndices[0];
	const FVector ContainmentWatchPosition = InitialSamples[ContainmentWatchIndex].Position;
	const FContainmentQueryResult Step0Containment = Planet.QueryContainment(ContainmentWatchPosition);

	UE_LOG(LogTemp, Log,
		TEXT("Motion diagnostic step0 containment: sample=%d plate=%d found=%d containing=%d"),
		ContainmentWatchIndex,
		Step0Containment.PlateId,
		Step0Containment.bFoundContainingPlate ? 1 : 0,
		Step0Containment.NumContainingPlates);

	TArray<int32> PlateChangesPerReconcile;
	TArray<int32> Step50PlateIds;
	TArray<int32> Step100PlateIds;
	const int32 NumSteps = 100;
	for (int32 StepIndex = 0; StepIndex < NumSteps; ++StepIndex)
	{
		Planet.StepSimulation();
		const TArray<FCanonicalSample>& Samples = Planet.GetSamples();

		if (StepIndex == 49)
		{
			Step50PlateIds.Reserve(BoundaryWatchIndices.Num());
			for (const int32 SampleIndex : BoundaryWatchIndices)
			{
				Step50PlateIds.Add(Samples[SampleIndex].PlateId);
			}
		}

		if (Planet.WasReconcileTriggeredLastStep())
		{
			int32 ChangedThisReconcile = 0;
			for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
			{
				if (Samples[SampleIndex].PlateId != PreviousPlateIds[SampleIndex])
				{
					++ChangedThisReconcile;
				}
				PreviousPlateIds[SampleIndex] = Samples[SampleIndex].PlateId;
			}
			PlateChangesPerReconcile.Add(ChangedThisReconcile);
		}
	}

	const TArray<FCanonicalSample>& FinalSamples = Planet.GetSamples();
	for (const int32 SampleIndex : BoundaryWatchIndices)
	{
		Step100PlateIds.Add(FinalSamples[SampleIndex].PlateId);
	}

	int32 ChangedFromInitial = 0;
	for (int32 SampleIndex = 0; SampleIndex < FinalSamples.Num(); ++SampleIndex)
	{
		ChangedFromInitial += (FinalSamples[SampleIndex].PlateId != InitialPlateIds[SampleIndex]) ? 1 : 0;
	}

	const FContainmentQueryResult Step100Containment = Planet.QueryContainment(ContainmentWatchPosition);

	UE_LOG(LogTemp, Log,
		TEXT("Motion diagnostic step100 containment: sample=%d plate=%d found=%d containing=%d"),
		ContainmentWatchIndex,
		Step100Containment.PlateId,
		Step100Containment.bFoundContainingPlate ? 1 : 0,
		Step100Containment.NumContainingPlates);

	for (const FPlate& Plate : Planet.GetPlates())
	{
		const double PrincipalAngle = static_cast<double>(Plate.CumulativeRotation.AngularDistance(FQuat::Identity));
		const double IntegratedDisplacement = FMath::Abs(static_cast<double>(Plate.AngularSpeed)) * static_cast<double>(NumSteps);
		UE_LOG(LogTemp, Log,
			TEXT("Motion diagnostic plate %d: speed=%.6f integrated=%.6f principalAngle=%.6f quat=(%.6f, %.6f, %.6f, %.6f)"),
			Plate.Id,
			Plate.AngularSpeed,
			IntegratedDisplacement,
			PrincipalAngle,
			Plate.CumulativeRotation.X,
			Plate.CumulativeRotation.Y,
			Plate.CumulativeRotation.Z,
			Plate.CumulativeRotation.W);
	}

	double MeanChangedPerReconcile = 0.0;
	int32 MaxChangedPerReconcile = 0;
	for (const int32 ChangedCount : PlateChangesPerReconcile)
	{
		MeanChangedPerReconcile += static_cast<double>(ChangedCount);
		MaxChangedPerReconcile = FMath::Max(MaxChangedPerReconcile, ChangedCount);
	}
	if (PlateChangesPerReconcile.Num() > 0)
	{
		MeanChangedPerReconcile /= static_cast<double>(PlateChangesPerReconcile.Num());
	}

	FString BoundaryStep50;
	FString BoundaryStep100;
	for (int32 WatchIndex = 0; WatchIndex < BoundaryWatchIndices.Num(); ++WatchIndex)
	{
		const int32 SampleIndex = BoundaryWatchIndices[WatchIndex];
		const int32 Step0Plate = InitialPlateIds[SampleIndex];
		const int32 Step50Plate = Step50PlateIds.IsValidIndex(WatchIndex) ? Step50PlateIds[WatchIndex] : INDEX_NONE;
		const int32 Step100Plate = Step100PlateIds.IsValidIndex(WatchIndex) ? Step100PlateIds[WatchIndex] : INDEX_NONE;
		BoundaryStep50 += FString::Printf(TEXT("[%d:%d->%d] "), SampleIndex, Step0Plate, Step50Plate);
		BoundaryStep100 += FString::Printf(TEXT("[%d:%d->%d] "), SampleIndex, Step0Plate, Step100Plate);
	}

	UE_LOG(LogTemp, Log,
		TEXT("Motion diagnostic ownership: steps=%d reconciles=%d changed_step0_to_100=%d (%.3f%%) mean_changed_per_reconcile=%.1f max_changed_per_reconcile=%d"),
		NumSteps,
		PlateChangesPerReconcile.Num(),
		ChangedFromInitial,
		(100.0 * static_cast<double>(ChangedFromInitial)) / static_cast<double>(FinalSamples.Num()),
		MeanChangedPerReconcile,
		MaxChangedPerReconcile);
	UE_LOG(LogTemp, Log, TEXT("Motion diagnostic boundary watch step0->50: %s"), *BoundaryStep50);
	UE_LOG(LogTemp, Log, TEXT("Motion diagnostic boundary watch step0->100: %s"), *BoundaryStep100);

	TestTrue(TEXT("Containment found at step 0"), Step0Containment.bFoundContainingPlate);
	TestTrue(TEXT("Containment found at step 100"), Step100Containment.bFoundContainingPlate);
	TestTrue(TEXT("Ownership changed for at least one sample over 100 steps"), ChangedFromInitial > 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FShewchukPredicatesTest, "Aurous.TectonicPlanet.ShewchukPredicates",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FShewchukPredicatesTest::RunTest(const FString& Parameters)
{
	exactinit();

	double O[3] = { 0.0, 0.0, 0.0 };
	double A[3] = { 1.0, 0.0, 0.0 };
	double B[3] = { 0.0, 1.0, 0.0 };
	double C[3] = { 0.0, 0.0, 1.0 };

	const double Result = orient3d(O, A, B, C);
	TestTrue(TEXT("orient3d non-zero for tetrahedron"), !FMath::IsNearlyZero(Result, 1e-15));

	const double ResultFlipped = orient3d(O, B, A, C);
	TestTrue(TEXT("orient3d flips sign when swapping points"), Result * ResultFlipped < 0.0);

	double D[3] = { 1.0, 1.0, 0.0 };
	const double ResultCoplanar = orient3d(O, A, B, D);
	TestTrue(TEXT("orient3d zero for coplanar"), FMath::IsNearlyZero(ResultCoplanar, 1e-15));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS








