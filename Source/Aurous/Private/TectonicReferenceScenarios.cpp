#include "TectonicReferenceScenarios.h"

#include "HAL/PlatformTime.h"

namespace
{
	void FinalizeReferenceScenarioMetrics(
		FReferenceScenarioObservedMetrics& Metrics,
		const double ScenarioStartSeconds)
	{
		Metrics.ScenarioWallMs = (FPlatformTime::Seconds() - ScenarioStartSeconds) * 1000.0;
		if (Metrics.ReconcileSteps > 0)
		{
			const double ReconcileCount = static_cast<double>(Metrics.ReconcileSteps);
			Metrics.AverageReconcileMs /= ReconcileCount;
			Metrics.AverageUnaccountedPct /= ReconcileCount;
		}
	}
}

int32 ComputeExpectedInitialPlateFloorSamples(const int32 TotalSampleCount, const int32 PlateCount)
{
	if (TotalSampleCount <= 0 || PlateCount <= 0)
	{
		return 0;
	}

	return FMath::Max(64, FMath::FloorToInt(0.30 * static_cast<double>(TotalSampleCount) / static_cast<double>(PlateCount)));
}

int32 ComputeExpectedPersistentPlateFloorSamples(const FPlate& Plate)
{
	if (Plate.PersistencePolicy != EPlatePersistencePolicy::Protected)
	{
		return 0;
	}

	return FMath::Max(32, FMath::CeilToInt(0.05 * static_cast<double>(FMath::Max(0, Plate.InitialSampleCount))));
}

const TArray<FReferenceScenarioDefinition>& GetLockedReferenceScenarios()
{
	static const TArray<FReferenceScenarioDefinition> Scenarios = {
		{ TEXT("Smoke7"), 100000, 7, 1, 40, { 2, 0.25, 0.40, 0.25, 6, 0, true } },
		{ TEXT("Nominal20"), 200000, 20, 1, 60, { 6, 0.25, 0.40, 0.25, 18, 1, true } },
		{ TEXT("Stress40"), 500000, 40, 1, 60, { 12, 0.25, 0.40, 0.25, 36, 1, true } }
	};
	return Scenarios;
}

const FReferenceScenarioDefinition* FindLockedReferenceScenarioByName(const FString& ScenarioName)
{
	for (const FReferenceScenarioDefinition& Scenario : GetLockedReferenceScenarios())
	{
		if (ScenarioName.Equals(Scenario.Name, ESearchCase::IgnoreCase))
		{
			return &Scenario;
		}
	}

	return nullptr;
}

bool CollectReferenceScenarioObservedMetrics(
	const FReferenceScenarioDefinition& Scenario,
	const FTectonicPlanet& BasePlanet,
	FReferenceScenarioObservedMetrics& OutMetrics,
	double StartupToMainMs,
	EContinentalStabilizerMode StabilizerMode)
{
	OutMetrics = FReferenceScenarioObservedMetrics{};
	OutMetrics.StartupToMainMs = StartupToMainMs;

	const double ScenarioStartSeconds = FPlatformTime::Seconds();
	FTectonicPlanet Planet = BasePlanet;
	Planet.SetContinentalStabilizerMode(StabilizerMode);
	Planet.InitializePlates(Scenario.PlateCount, Scenario.Seed);

	const TArray<FPlate>& InitialPlates = Planet.GetPlates();
	OutMetrics.MinExpectedProtectedPlateFloor = TNumericLimits<int32>::Max();
	for (const FPlate& Plate : InitialPlates)
	{
		if (Plate.PersistencePolicy == EPlatePersistencePolicy::Protected)
		{
			OutMetrics.MinExpectedProtectedPlateFloor = FMath::Min(
				OutMetrics.MinExpectedProtectedPlateFloor,
				ComputeExpectedPersistentPlateFloorSamples(Plate));
		}
	}
	if (OutMetrics.MinExpectedProtectedPlateFloor == TNumericLimits<int32>::Max())
	{
		OutMetrics.MinExpectedProtectedPlateFloor = 0;
	}

	OutMetrics.InitialContinentalPlateCount = Planet.GetContinentalPlateCount();
	OutMetrics.InitialContinentalAreaFraction = Planet.GetContinentalAreaFraction();
	OutMetrics.MinimumRuntimeContinentalAreaFraction = OutMetrics.InitialContinentalAreaFraction;

	const int32 SampleStride = FMath::Max(1, Scenario.SampleCount / 2048);
	TArray<int32> WatchSampleIndices;
	for (int32 SampleIndex = 0; SampleIndex < Scenario.SampleCount; SampleIndex += SampleStride)
	{
		WatchSampleIndices.Add(SampleIndex);
	}

	const TArray<FCanonicalSample>& InitialSamples = Planet.GetSamples();
	TArray<FVector> WatchInitialPositions;
	TArray<int32> WatchPreviousPlateIds;
	TArray<int32> WatchCurrentPlateIds;
	WatchInitialPositions.Reserve(WatchSampleIndices.Num());
	WatchPreviousPlateIds.Reserve(WatchSampleIndices.Num());
	WatchCurrentPlateIds.Reserve(WatchSampleIndices.Num());
	for (const int32 SampleIndex : WatchSampleIndices)
	{
		WatchInitialPositions.Add(InitialSamples[SampleIndex].Position);
		WatchPreviousPlateIds.Add(INDEX_NONE);
		WatchCurrentPlateIds.Add(InitialSamples[SampleIndex].PlateId);
	}

	double PreviousContinentalAreaFraction = OutMetrics.InitialContinentalAreaFraction;
	for (int32 StepIndex = 0; StepIndex < Scenario.NumSteps; ++StepIndex)
	{
		Planet.StepSimulation();
		const TArray<FCanonicalSample>& Samples = Planet.GetSamples();

		for (const FCanonicalSample& Sample : Samples)
		{
			if (Sample.PlateId < 0 || Sample.PlateId >= Scenario.PlateCount)
			{
				++OutMetrics.InvalidPlateAssignments;
			}

			if (Sample.PrevPlateId < 0 || Sample.PrevPlateId >= Scenario.PlateCount)
			{
				++OutMetrics.InvalidPreviousAssignments;
			}

			if (Sample.bGapDetected && (Sample.PlateId < 0 || Sample.PlateId >= Scenario.PlateCount))
			{
				++OutMetrics.GapOrphanMismatchCount;
			}
		}

		for (int32 WatchIndex = 0; WatchIndex < WatchSampleIndices.Num(); ++WatchIndex)
		{
			const int32 SampleIndex = WatchSampleIndices[WatchIndex];
			const FCanonicalSample& Sample = Samples[SampleIndex];

			OutMetrics.PositionDriftCount += Sample.Position.Equals(WatchInitialPositions[WatchIndex], 1e-10) ? 0 : 1;

			if (Planet.WasReconcileTriggeredLastStep())
			{
				const int32 CurrentPlateId = Sample.PlateId;
				const int32 PreviousPlateIdForWatch = WatchCurrentPlateIds[WatchIndex];
				const int32 OldPlateId = WatchPreviousPlateIds[WatchIndex];
				if (OldPlateId != INDEX_NONE && OldPlateId == CurrentPlateId && PreviousPlateIdForWatch != CurrentPlateId)
				{
					++OutMetrics.PingPongTransitions;
				}

				WatchPreviousPlateIds[WatchIndex] = PreviousPlateIdForWatch;
				WatchCurrentPlateIds[WatchIndex] = CurrentPlateId;
			}
		}

		if (!Planet.WasReconcileTriggeredLastStep())
		{
			continue;
		}

		++OutMetrics.ReconcileSteps;
		const FReconcilePhaseTimings& Timings = Planet.GetLastReconcileTimings();
		const bool bTimingsValid =
			Timings.SampleBufferCopyMs >= 0.0 &&
			Timings.Phase1BuildSpatialMs >= 0.0 &&
			Timings.Phase2OwnershipMs >= 0.0 &&
			Timings.Phase3InterpolationMs >= 0.0 &&
			Timings.Phase4GapMs >= 0.0 &&
			Timings.ContinentalStabilizerMs >= 0.0 &&
			Timings.Phase5OverlapMs >= 0.0 &&
			Timings.Phase6MembershipMs >= 0.0 &&
			Timings.Phase6PersistenceMs >= 0.0 &&
			Timings.Phase7TerraneMs >= 0.0 &&
			Timings.Phase8CollisionMs >= 0.0 &&
			Timings.Phase8PostCollisionRefreshMs >= 0.0 &&
			Timings.PrevPlateWritebackMs >= 0.0 &&
			Timings.Phase9SubductionMs >= 0.0 &&
			Timings.Phase7SubductionMs >= 0.0 &&
			Timings.PhaseSumMs >= 0.0 &&
			Timings.UnaccountedMs >= 0.0 &&
			Timings.UnaccountedPct >= 0.0 &&
			Timings.TotalMs > 0.0;
		OutMetrics.InvalidTimingSnapshots += bTimingsValid ? 0 : 1;
		OutMetrics.AverageReconcileMs += Timings.TotalMs;
		OutMetrics.MaxReconcileMs = FMath::Max(OutMetrics.MaxReconcileMs, Timings.TotalMs);
		OutMetrics.AverageUnaccountedPct += Timings.UnaccountedPct;
		OutMetrics.MaxUnaccountedPct = FMath::Max(OutMetrics.MaxUnaccountedPct, Timings.UnaccountedPct);
		if (Timings.UnaccountedPct > MaxAcceptableUnaccountedPct)
		{
			++OutMetrics.UnaccountedBudgetFailures;
		}

		const double SampleCountDouble = static_cast<double>(FMath::Max(1, Samples.Num()));
		OutMetrics.MaxGapFraction = FMath::Max(OutMetrics.MaxGapFraction, static_cast<double>(Planet.GetLastGapSampleCount()) / SampleCountDouble);
		OutMetrics.MaxOverlapFraction = FMath::Max(OutMetrics.MaxOverlapFraction, static_cast<double>(Planet.GetLastOverlapSampleCount()) / SampleCountDouble);
		OutMetrics.MaxBoundaryMeanDepthHops = FMath::Max(OutMetrics.MaxBoundaryMeanDepthHops, Planet.GetBoundaryMeanDepthHops());
		OutMetrics.MaxBoundaryMaxDepthHops = FMath::Max(OutMetrics.MaxBoundaryMaxDepthHops, Planet.GetBoundaryMaxDepthHops());
		OutMetrics.MaxNonGapPlateComponentCount = FMath::Max(OutMetrics.MaxNonGapPlateComponentCount, Planet.GetMaxPlateComponentCount());
		OutMetrics.MaxDetachedPlateFragmentSampleCount = FMath::Max(OutMetrics.MaxDetachedPlateFragmentSampleCount, Planet.GetDetachedPlateFragmentSampleCount());
		OutMetrics.MaxLargestDetachedPlateFragmentSize = FMath::Max(OutMetrics.MaxLargestDetachedPlateFragmentSize, Planet.GetLargestDetachedPlateFragmentSize());
		OutMetrics.MaximumContinentalComponentCount = FMath::Max(OutMetrics.MaximumContinentalComponentCount, Planet.GetContinentalComponentCount());
		OutMetrics.MinObservedProtectedPlateSampleCount = FMath::Min(OutMetrics.MinObservedProtectedPlateSampleCount, Planet.GetMinProtectedPlateSampleCount());
		OutMetrics.MaxEmptyProtectedPlateCount = FMath::Max(OutMetrics.MaxEmptyProtectedPlateCount, Planet.GetEmptyProtectedPlateCount());
		OutMetrics.FinalCollisionEventCount = Planet.GetCollisionEventCount();

		const double CurrentContinentalAreaFraction = Planet.GetContinentalAreaFraction();
		OutMetrics.MinimumRuntimeContinentalAreaFraction = FMath::Min(OutMetrics.MinimumRuntimeContinentalAreaFraction, CurrentContinentalAreaFraction);
		OutMetrics.bRuntimeContinentalFractionIncreased |= (CurrentContinentalAreaFraction > PreviousContinentalAreaFraction + MaxContinentalFractionIncreaseEpsilon);
		PreviousContinentalAreaFraction = CurrentContinentalAreaFraction;

		const int32 ContinentalSampleCount = Planet.GetContinentalSampleCount();
		if (ContinentalSampleCount > 0)
		{
			const double LargestContinentalShare = static_cast<double>(Planet.GetLargestContinentalComponentSize()) / static_cast<double>(ContinentalSampleCount);
			OutMetrics.MinLargestContinentalShare = FMath::Min(OutMetrics.MinLargestContinentalShare, LargestContinentalShare);
		}

		bool bObservedContinentalOverridingFront = false;
		for (const FCanonicalSample& Sample : Samples)
		{
			if (Sample.bIsSubductionFront &&
				Sample.CrustType == ECrustType::Continental &&
				Sample.SubductionRole == ESubductionRole::Overriding)
			{
				bObservedContinentalOverridingFront = true;
				break;
			}
		}

		OutMetrics.bObservedOceanicContinentalConvergence |=
			bObservedContinentalOverridingFront && (Planet.GetSubductionFrontSampleCount() > 0);
		OutMetrics.bObservedAndean |= (Planet.GetAndeanSampleCount() > 0);
	}

	const double WatchCount = static_cast<double>(FMath::Max(1, WatchSampleIndices.Num() * FMath::Max(1, OutMetrics.ReconcileSteps)));
	OutMetrics.PingPongRate = static_cast<double>(OutMetrics.PingPongTransitions) / WatchCount;
	if (OutMetrics.MinObservedProtectedPlateSampleCount == TNumericLimits<int32>::Max())
	{
		OutMetrics.MinObservedProtectedPlateSampleCount = 0;
	}

	FinalizeReferenceScenarioMetrics(OutMetrics, ScenarioStartSeconds);
	return true;
}

FString FormatReferenceScenarioSummary(
	const FReferenceScenarioDefinition& Scenario,
	const FReferenceScenarioObservedMetrics& Metrics,
	bool bPass)
{
	return FString::Printf(
		TEXT("Reference scenario [%s]: seed=%d steps=%d startup_to_main=%.1fms wall=%.1fms avg_reconcile=%.1fms max_reconcile=%.1fms avg_unaccounted=%.2f%% max_unaccounted=%.2f%% stabilizer_mismatches=%d unaccounted_budget_failures=%d init_continental_plate_count=%d init_continental_area_fraction=%.3f minimum_runtime_continental_fraction=%.3f maximum_continental_component_count=%d final_collision_event_count=%d pass=%s"),
		Scenario.Name,
		Scenario.Seed,
		Scenario.NumSteps,
		Metrics.StartupToMainMs,
		Metrics.ScenarioWallMs,
		Metrics.AverageReconcileMs,
		Metrics.MaxReconcileMs,
		Metrics.AverageUnaccountedPct,
		Metrics.MaxUnaccountedPct,
		Metrics.StabilizerShadowMismatchCount,
		Metrics.UnaccountedBudgetFailures,
		Metrics.InitialContinentalPlateCount,
		Metrics.InitialContinentalAreaFraction,
		Metrics.MinimumRuntimeContinentalAreaFraction,
		Metrics.MaximumContinentalComponentCount,
		Metrics.FinalCollisionEventCount,
		bPass ? TEXT("true") : TEXT("false"));
}

bool DoesReferenceScenarioObservedMetricsPass(
	const FReferenceScenarioDefinition& Scenario,
	const FReferenceScenarioObservedMetrics& Metrics)
{
	const FReferenceScenarioBounds& Bounds = Scenario.Bounds;
	const bool bBaseInvariantsPass =
		Metrics.ReconcileSteps >= 10 &&
		Metrics.InvalidPlateAssignments == 0 &&
		Metrics.InvalidPreviousAssignments == 0 &&
		Metrics.InvalidTimingSnapshots == 0 &&
		Metrics.StabilizerShadowMismatchCount == 0 &&
		Metrics.UnaccountedBudgetFailures == 0 &&
		Metrics.PositionDriftCount == 0 &&
		Metrics.GapOrphanMismatchCount == 0 &&
		Metrics.PingPongRate <= 0.10 &&
		Metrics.MaxGapFraction <= MaxAcceptableGapFraction &&
		Metrics.MaxOverlapFraction <= MaxAcceptableOverlapFraction &&
		Metrics.MaxBoundaryMeanDepthHops <= MaxAcceptableBoundaryMeanDepthHops &&
		Metrics.MaxBoundaryMaxDepthHops <= MaxAcceptableBoundaryMaxDepthHops &&
		Metrics.MaxNonGapPlateComponentCount == 1 &&
		Metrics.MaxDetachedPlateFragmentSampleCount == 0 &&
		Metrics.MaxLargestDetachedPlateFragmentSize == 0 &&
		Metrics.MaxEmptyProtectedPlateCount == 0 &&
		Metrics.MinObservedProtectedPlateSampleCount >= Metrics.MinExpectedProtectedPlateFloor;

	const bool bStructuralGeologyPass =
		Metrics.InitialContinentalPlateCount == Bounds.ExpectedInitialContinentalPlateCount &&
		Metrics.InitialContinentalAreaFraction + UE_DOUBLE_SMALL_NUMBER >= Bounds.InitialContinentalAreaFractionMin &&
		Metrics.InitialContinentalAreaFraction - UE_DOUBLE_SMALL_NUMBER <= Bounds.InitialContinentalAreaFractionMax &&
		!Metrics.bRuntimeContinentalFractionIncreased &&
		Metrics.MinimumRuntimeContinentalAreaFraction + UE_DOUBLE_SMALL_NUMBER >= Bounds.MinimumRuntimeContinentalAreaFraction &&
		Metrics.MaximumContinentalComponentCount <= Bounds.MaximumContinentalComponentCount &&
		Metrics.FinalCollisionEventCount >= Bounds.MinimumCollisionEventCount &&
		(!Bounds.bRequireAndeanWhenOceanicContinentalConvergenceExists ||
			!Metrics.bObservedOceanicContinentalConvergence ||
			Metrics.bObservedAndean);

	return bBaseInvariantsPass && bStructuralGeologyPass;
}

bool TryDiscoverLowestPassingReferenceScenarioSeed(
	const FReferenceScenarioDefinition& ScenarioTemplate,
	const FTectonicPlanet& BasePlanet,
	int32& OutSeed,
	FReferenceScenarioObservedMetrics& OutMetrics)
{
	for (int32 CandidateSeed = 1; CandidateSeed <= 256; ++CandidateSeed)
	{
		FReferenceScenarioDefinition CandidateScenario = ScenarioTemplate;
		CandidateScenario.Seed = CandidateSeed;

		FReferenceScenarioObservedMetrics CandidateMetrics;
		if (!CollectReferenceScenarioObservedMetrics(CandidateScenario, BasePlanet, CandidateMetrics, 0.0, EContinentalStabilizerMode::Incremental))
		{
			continue;
		}
		if (DoesReferenceScenarioObservedMetricsPass(CandidateScenario, CandidateMetrics))
		{
			OutSeed = CandidateSeed;
			OutMetrics = CandidateMetrics;
			return true;
		}
	}

	return false;
}
