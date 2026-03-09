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

	bool HasFailureObservation(const FReferenceScenarioObservedMetrics& Metrics, const TCHAR* Label)
	{
		return Metrics.FailureObservations.ContainsByPredicate(
			[Label](const FReferenceScenarioFailureObservation& Observation)
			{
				return Observation.Label.Equals(Label, ESearchCase::CaseSensitive);
			});
	}

	void RecordFailureObservation(
		FReferenceScenarioObservedMetrics& Metrics,
		const TCHAR* Label,
		const int32 StepIndex,
		const FString& Detail)
	{
		if (HasFailureObservation(Metrics, Label))
		{
			return;
		}

		FReferenceScenarioFailureObservation Observation;
		Observation.Label = Label;
		Observation.StepIndex = StepIndex;
		Observation.Detail = Detail;
		Metrics.FailureObservations.Add(MoveTemp(Observation));
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
	// Breakup7 explicitly exercises M5 plate rifting by requiring a rift event and temporary active-plate growth.
	static const TArray<FReferenceScenarioDefinition> Scenarios = {
		{ TEXT("Smoke7"), 100000, 7, 1, 40, { 2, 0.25, 0.40, 0.25, 0, 0, 0, true, true } },
		{ TEXT("Nominal20"), 200000, 20, 6, 60, { 6, 0.25, 0.40, 0.25, 1, 0, 0, true, true } },
		{ TEXT("Stress40"), 500000, 40, 4, 60, { 12, 0.25, 0.40, 0.25, 1, 0, 0, true, true } },
		{ TEXT("Breakup7"), 100000, 7, 1, 80, { 2, 0.25, 0.40, 0.25, 0, 1, 8, true, true } }
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
	const FReferenceScenarioBounds& Bounds = Scenario.Bounds;
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
	OutMetrics.PeakActivePlateCount = Planet.GetActivePlateCount();
	OutMetrics.PeakAllocatedPlateCount = Planet.GetPlates().Num();
	OutMetrics.FinalRiftEventCount = Planet.GetRiftEventCount();
	if (OutMetrics.InitialContinentalPlateCount != Bounds.ExpectedInitialContinentalPlateCount)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("initial_continental_plate_count"),
			0,
			FString::Printf(
				TEXT("value=%d expected=%d"),
				OutMetrics.InitialContinentalPlateCount,
				Bounds.ExpectedInitialContinentalPlateCount));
	}
	if (OutMetrics.InitialContinentalAreaFraction + UE_DOUBLE_SMALL_NUMBER < Bounds.InitialContinentalAreaFractionMin ||
		OutMetrics.InitialContinentalAreaFraction - UE_DOUBLE_SMALL_NUMBER > Bounds.InitialContinentalAreaFractionMax)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("initial_continental_area_fraction"),
			0,
			FString::Printf(
				TEXT("value=%.6f bounds=[%.6f, %.6f]"),
				OutMetrics.InitialContinentalAreaFraction,
				Bounds.InitialContinentalAreaFractionMin,
				Bounds.InitialContinentalAreaFractionMax));
	}

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
		const int32 CurrentAllocatedPlateCount = Planet.GetPlates().Num();
		const int32 StepNumber = StepIndex + 1;
		double StepMaximumElevationKm = 0.0;
		bool bStepInvalidPlateAssignments = false;
		bool bStepInvalidPreviousAssignments = false;
		bool bStepGapOrphanMismatch = false;
		bool bStepPositionDrift = false;

		for (const FCanonicalSample& Sample : Samples)
		{
			StepMaximumElevationKm = FMath::Max(StepMaximumElevationKm, static_cast<double>(Sample.Elevation));
			OutMetrics.MaximumObservedElevationKm = FMath::Max(
				OutMetrics.MaximumObservedElevationKm,
				static_cast<double>(Sample.Elevation));
			if (Sample.CrustType == ECrustType::Continental)
			{
				OutMetrics.MaximumObservedContinentalElevationKm = FMath::Max(
					OutMetrics.MaximumObservedContinentalElevationKm,
					static_cast<double>(Sample.Elevation));
			}

			if (Sample.PlateId < 0 || Sample.PlateId >= CurrentAllocatedPlateCount)
			{
				bStepInvalidPlateAssignments = true;
				++OutMetrics.InvalidPlateAssignments;
			}

			if (Sample.PrevPlateId < 0 || Sample.PrevPlateId >= CurrentAllocatedPlateCount)
			{
				bStepInvalidPreviousAssignments = true;
				++OutMetrics.InvalidPreviousAssignments;
			}

			if (Sample.bGapDetected && (Sample.PlateId < 0 || Sample.PlateId >= CurrentAllocatedPlateCount))
			{
				bStepGapOrphanMismatch = true;
				++OutMetrics.GapOrphanMismatchCount;
			}
		}
		if (StepMaximumElevationKm > MaxReferenceScenarioElevationCeilingKm + static_cast<double>(KINDA_SMALL_NUMBER))
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("max_elevation"),
				StepNumber,
				FString::Printf(
					TEXT("value=%.3f ceiling=%.3f"),
					StepMaximumElevationKm,
					MaxReferenceScenarioElevationCeilingKm));
		}
		if (bStepInvalidPlateAssignments)
		{
			RecordFailureObservation(OutMetrics, TEXT("invalid_plate_assignments"), StepNumber, TEXT("observed invalid PlateId"));
		}
		if (bStepInvalidPreviousAssignments)
		{
			RecordFailureObservation(OutMetrics, TEXT("invalid_prev_plate_assignments"), StepNumber, TEXT("observed invalid PrevPlateId"));
		}
		if (bStepGapOrphanMismatch)
		{
			RecordFailureObservation(OutMetrics, TEXT("gap_orphan_mismatch"), StepNumber, TEXT("gap sample had invalid owner"));
		}

		for (int32 WatchIndex = 0; WatchIndex < WatchSampleIndices.Num(); ++WatchIndex)
		{
			const int32 SampleIndex = WatchSampleIndices[WatchIndex];
			const FCanonicalSample& Sample = Samples[SampleIndex];

			const bool bPositionDrifted = !Sample.Position.Equals(WatchInitialPositions[WatchIndex], 1e-10);
			bStepPositionDrift |= bPositionDrifted;
			OutMetrics.PositionDriftCount += bPositionDrifted ? 1 : 0;

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
		if (bStepPositionDrift)
		{
			RecordFailureObservation(OutMetrics, TEXT("position_drift"), StepNumber, TEXT("watch sample position changed"));
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
			Timings.Phase10RiftingMs >= 0.0 &&
			Timings.Phase10PostRiftRefreshMs >= 0.0 &&
			Timings.PrevPlateWritebackMs >= 0.0 &&
			Timings.Phase9SubductionMs >= 0.0 &&
			Timings.Phase7SubductionMs >= 0.0 &&
			Timings.PhaseSumMs >= 0.0 &&
			Timings.UnaccountedMs >= 0.0 &&
			Timings.UnaccountedPct >= 0.0 &&
			Timings.TotalMs > 0.0;
		OutMetrics.InvalidTimingSnapshots += bTimingsValid ? 0 : 1;
		if (!bTimingsValid)
		{
			RecordFailureObservation(OutMetrics, TEXT("invalid_timing_snapshots"), StepNumber, TEXT("negative or zero reconcile timing field"));
		}
		OutMetrics.AverageReconcileMs += Timings.TotalMs;
		OutMetrics.MaxReconcileMs = FMath::Max(OutMetrics.MaxReconcileMs, Timings.TotalMs);
		OutMetrics.AverageUnaccountedPct += Timings.UnaccountedPct;
		OutMetrics.MaxUnaccountedPct = FMath::Max(OutMetrics.MaxUnaccountedPct, Timings.UnaccountedPct);
		if (Timings.UnaccountedPct > MaxAcceptableUnaccountedPct)
		{
			++OutMetrics.UnaccountedBudgetFailures;
			RecordFailureObservation(
				OutMetrics,
				TEXT("unaccounted_budget"),
				StepNumber,
				FString::Printf(
					TEXT("value=%.2f%% limit=%.2f%%"),
					Timings.UnaccountedPct,
					MaxAcceptableUnaccountedPct));
		}

		const double SampleCountDouble = static_cast<double>(FMath::Max(1, Samples.Num()));
		const double CurrentGapFraction = static_cast<double>(Planet.GetLastGapSampleCount()) / SampleCountDouble;
		const double CurrentOverlapFraction = static_cast<double>(Planet.GetLastOverlapSampleCount()) / SampleCountDouble;
		const double CurrentBoundaryMeanDepthHops = Planet.GetBoundaryMeanDepthHops();
		const int32 CurrentBoundaryMaxDepthHops = Planet.GetBoundaryMaxDepthHops();
		const int32 CurrentPlateComponentCount = Planet.GetMaxPlateComponentCount();
		const int32 CurrentDetachedPlateFragmentSampleCount = Planet.GetDetachedPlateFragmentSampleCount();
		const int32 CurrentLargestDetachedPlateFragmentSize = Planet.GetLargestDetachedPlateFragmentSize();
		const int32 CurrentContinentalComponentCount = Planet.GetContinentalComponentCount();
		const int32 CurrentProtectedPlateSampleCount = Planet.GetMinProtectedPlateSampleCount();
		const int32 CurrentEmptyProtectedPlateCount = Planet.GetEmptyProtectedPlateCount();
		OutMetrics.MaxGapFraction = FMath::Max(OutMetrics.MaxGapFraction, CurrentGapFraction);
		OutMetrics.MaxOverlapFraction = FMath::Max(OutMetrics.MaxOverlapFraction, CurrentOverlapFraction);
		OutMetrics.MaxBoundaryMeanDepthHops = FMath::Max(OutMetrics.MaxBoundaryMeanDepthHops, CurrentBoundaryMeanDepthHops);
		OutMetrics.MaxBoundaryMaxDepthHops = FMath::Max(OutMetrics.MaxBoundaryMaxDepthHops, CurrentBoundaryMaxDepthHops);
		OutMetrics.MaxNonGapPlateComponentCount = FMath::Max(OutMetrics.MaxNonGapPlateComponentCount, CurrentPlateComponentCount);
		OutMetrics.MaxDetachedPlateFragmentSampleCount = FMath::Max(OutMetrics.MaxDetachedPlateFragmentSampleCount, CurrentDetachedPlateFragmentSampleCount);
		OutMetrics.MaxLargestDetachedPlateFragmentSize = FMath::Max(OutMetrics.MaxLargestDetachedPlateFragmentSize, CurrentLargestDetachedPlateFragmentSize);
		OutMetrics.MaximumContinentalComponentCount = FMath::Max(OutMetrics.MaximumContinentalComponentCount, CurrentContinentalComponentCount);
		OutMetrics.MinObservedProtectedPlateSampleCount = FMath::Min(OutMetrics.MinObservedProtectedPlateSampleCount, CurrentProtectedPlateSampleCount);
		OutMetrics.MaxEmptyProtectedPlateCount = FMath::Max(OutMetrics.MaxEmptyProtectedPlateCount, CurrentEmptyProtectedPlateCount);
		OutMetrics.FinalCollisionEventCount = Planet.GetCollisionEventCount();
		OutMetrics.FinalRiftEventCount = Planet.GetRiftEventCount();
		OutMetrics.PeakActivePlateCount = FMath::Max(OutMetrics.PeakActivePlateCount, Planet.GetActivePlateCount());
		OutMetrics.PeakAllocatedPlateCount = FMath::Max(OutMetrics.PeakAllocatedPlateCount, CurrentAllocatedPlateCount);
		if (CurrentGapFraction > MaxAcceptableGapFraction)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("gap_fraction"),
				StepNumber,
				FString::Printf(TEXT("value=%.6f limit=%.6f"), CurrentGapFraction, MaxAcceptableGapFraction));
		}
		if (CurrentOverlapFraction > MaxAcceptableOverlapFraction)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("overlap_fraction"),
				StepNumber,
				FString::Printf(TEXT("value=%.6f limit=%.6f"), CurrentOverlapFraction, MaxAcceptableOverlapFraction));
		}
		if (CurrentBoundaryMeanDepthHops > MaxAcceptableBoundaryMeanDepthHops)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("boundary_mean_depth"),
				StepNumber,
				FString::Printf(
					TEXT("value=%.3f limit=%.3f"),
					CurrentBoundaryMeanDepthHops,
					MaxAcceptableBoundaryMeanDepthHops));
		}
		if (CurrentBoundaryMaxDepthHops > MaxAcceptableBoundaryMaxDepthHops)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("boundary_max_depth"),
				StepNumber,
				FString::Printf(TEXT("value=%d limit=%d"), CurrentBoundaryMaxDepthHops, MaxAcceptableBoundaryMaxDepthHops));
		}
		if (CurrentPlateComponentCount != 1)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("plate_connectivity"),
				StepNumber,
				FString::Printf(TEXT("value=%d expected=1"), CurrentPlateComponentCount));
		}
		if (CurrentDetachedPlateFragmentSampleCount != 0)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("detached_fragment_samples"),
				StepNumber,
				FString::Printf(TEXT("value=%d expected=0"), CurrentDetachedPlateFragmentSampleCount));
		}
		if (CurrentLargestDetachedPlateFragmentSize != 0)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("detached_fragment_size"),
				StepNumber,
				FString::Printf(TEXT("value=%d expected=0"), CurrentLargestDetachedPlateFragmentSize));
		}
		if (CurrentEmptyProtectedPlateCount != 0)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("empty_protected_plate"),
				StepNumber,
				FString::Printf(TEXT("value=%d expected=0"), CurrentEmptyProtectedPlateCount));
		}
		if (CurrentProtectedPlateSampleCount < OutMetrics.MinExpectedProtectedPlateFloor)
		{
			RecordFailureObservation(
				OutMetrics,
				TEXT("protected_plate_floor"),
				StepNumber,
				FString::Printf(
					TEXT("value=%d floor=%d"),
					CurrentProtectedPlateSampleCount,
					OutMetrics.MinExpectedProtectedPlateFloor));
		}

		const double CurrentContinentalAreaFraction = Planet.GetContinentalAreaFraction();
		// With M5 rifting active, continental area can legitimately rebound and fragment during breakup.
		// Keep these metrics for visibility, but do not treat them as structural failures.
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
		OutMetrics.bObservedHimalayan |= (Planet.GetHimalayanSampleCount() > 0);
	}

	const double WatchCount = static_cast<double>(FMath::Max(1, WatchSampleIndices.Num() * FMath::Max(1, OutMetrics.ReconcileSteps)));
	OutMetrics.PingPongRate = static_cast<double>(OutMetrics.PingPongTransitions) / WatchCount;
	if (OutMetrics.MinObservedProtectedPlateSampleCount == TNumericLimits<int32>::Max())
	{
		OutMetrics.MinObservedProtectedPlateSampleCount = 0;
	}
	if (OutMetrics.MaximumObservedElevationKm == -TNumericLimits<double>::Max())
	{
		OutMetrics.MaximumObservedElevationKm = 0.0;
	}
	if (OutMetrics.MaximumObservedContinentalElevationKm == -TNumericLimits<double>::Max())
	{
		OutMetrics.MaximumObservedContinentalElevationKm = 0.0;
	}

	FinalizeReferenceScenarioMetrics(OutMetrics, ScenarioStartSeconds);
	if (OutMetrics.ReconcileSteps < 10)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("reconcile_steps"),
			Scenario.NumSteps,
			FString::Printf(TEXT("value=%d minimum=10"), OutMetrics.ReconcileSteps));
	}
	if (OutMetrics.PingPongRate > 0.10)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("ping_pong_rate"),
			Scenario.NumSteps,
			FString::Printf(TEXT("value=%.6f limit=0.100000"), OutMetrics.PingPongRate));
	}
	if (OutMetrics.FinalCollisionEventCount < Bounds.MinimumCollisionEventCount)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("collision_event_count"),
			Scenario.NumSteps,
			FString::Printf(
				TEXT("value=%d minimum=%d"),
				OutMetrics.FinalCollisionEventCount,
				Bounds.MinimumCollisionEventCount));
	}
	if (OutMetrics.FinalRiftEventCount < Bounds.MinimumRiftEventCount)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("rift_event_count"),
			Scenario.NumSteps,
			FString::Printf(
				TEXT("value=%d minimum=%d"),
				OutMetrics.FinalRiftEventCount,
				Bounds.MinimumRiftEventCount));
	}
	if (OutMetrics.PeakActivePlateCount < Bounds.MinimumPeakActivePlateCount)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("peak_active_plate_count"),
			Scenario.NumSteps,
			FString::Printf(
				TEXT("value=%d minimum=%d"),
				OutMetrics.PeakActivePlateCount,
				Bounds.MinimumPeakActivePlateCount));
	}
	if (Bounds.bRequireAndeanWhenOceanicContinentalConvergenceExists &&
		OutMetrics.bObservedOceanicContinentalConvergence &&
		!OutMetrics.bObservedAndean)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("andean_observation"),
			Scenario.NumSteps,
			TEXT("oceanic-continental convergence observed without Andean samples"));
	}
	if (Bounds.bRequireHimalayanWhenCollisionOccurs &&
		OutMetrics.FinalCollisionEventCount > 0 &&
		!OutMetrics.bObservedHimalayan)
	{
		RecordFailureObservation(
			OutMetrics,
			TEXT("himalayan_observation"),
			Scenario.NumSteps,
			TEXT("collision events observed without Himalayan samples"));
	}

	return true;
}

FString FormatReferenceScenarioSummary(
	const FReferenceScenarioDefinition& Scenario,
	const FReferenceScenarioObservedMetrics& Metrics,
	bool bPass)
{
	return FString::Printf(
		TEXT("Reference scenario [%s]: samples=%d seed=%d steps=%d startup_to_main=%.1fms wall=%.1fms avg_reconcile=%.1fms max_reconcile=%.1fms avg_unaccounted=%.2f%% max_unaccounted=%.2f%% stabilizer_mismatches=%d unaccounted_budget_failures=%d init_continental_plate_count=%d init_continental_area_fraction=%.6f minimum_runtime_continental_fraction=%.6f maximum_continental_component_count=%d final_collision_event_count=%d final_rift_event_count=%d peak_active_plate_count=%d peak_allocated_plate_count=%d max_elevation_km=%.3f max_continental_elevation_km=%.3f observed_himalayan=%s pass=%s"),
		Scenario.Name,
		Scenario.SampleCount,
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
		Metrics.FinalRiftEventCount,
		Metrics.PeakActivePlateCount,
		Metrics.PeakAllocatedPlateCount,
		Metrics.MaximumObservedElevationKm,
		Metrics.MaximumObservedContinentalElevationKm,
		Metrics.bObservedHimalayan ? TEXT("true") : TEXT("false"),
		bPass ? TEXT("true") : TEXT("false"));
}

FString DescribeReferenceScenarioFailureDetails(const FReferenceScenarioObservedMetrics& Metrics)
{
	if (Metrics.FailureObservations.Num() == 0)
	{
		return TEXT("none");
	}

	TArray<FString> Parts;
	Parts.Reserve(Metrics.FailureObservations.Num());
	for (const FReferenceScenarioFailureObservation& Observation : Metrics.FailureObservations)
	{
		if (Observation.Detail.IsEmpty())
		{
			Parts.Add(FString::Printf(TEXT("%s@step=%d"), *Observation.Label, Observation.StepIndex));
			continue;
		}

		Parts.Add(FString::Printf(TEXT("%s@step=%d(%s)"), *Observation.Label, Observation.StepIndex, *Observation.Detail));
	}

	return FString::Join(Parts, TEXT("; "));
}

FString DescribeReferenceScenarioFailures(
	const FReferenceScenarioDefinition& Scenario,
	const FReferenceScenarioObservedMetrics& Metrics)
{
	const FReferenceScenarioBounds& Bounds = Scenario.Bounds;
	TArray<FString> Failures;
	auto AddFailure = [&Failures](const TCHAR* Label)
	{
		Failures.Add(Label);
	};

	if (Metrics.ReconcileSteps < 10)
	{
		AddFailure(TEXT("reconcile_steps"));
	}
	if (Metrics.InvalidPlateAssignments != 0)
	{
		AddFailure(TEXT("invalid_plate_assignments"));
	}
	if (Metrics.InvalidPreviousAssignments != 0)
	{
		AddFailure(TEXT("invalid_prev_plate_assignments"));
	}
	if (Metrics.InvalidTimingSnapshots != 0)
	{
		AddFailure(TEXT("invalid_timing_snapshots"));
	}
	if (Metrics.StabilizerShadowMismatchCount != 0)
	{
		AddFailure(TEXT("stabilizer_shadow_mismatch"));
	}
	if (Metrics.UnaccountedBudgetFailures != 0)
	{
		AddFailure(TEXT("unaccounted_budget"));
	}
	if (Metrics.PositionDriftCount != 0)
	{
		AddFailure(TEXT("position_drift"));
	}
	if (Metrics.GapOrphanMismatchCount != 0)
	{
		AddFailure(TEXT("gap_orphan_mismatch"));
	}
	if (Metrics.PingPongRate > 0.10)
	{
		AddFailure(TEXT("ping_pong_rate"));
	}
	if (Metrics.MaxGapFraction > MaxAcceptableGapFraction)
	{
		AddFailure(TEXT("gap_fraction"));
	}
	if (Metrics.MaxOverlapFraction > MaxAcceptableOverlapFraction)
	{
		AddFailure(TEXT("overlap_fraction"));
	}
	if (Metrics.MaxBoundaryMeanDepthHops > MaxAcceptableBoundaryMeanDepthHops)
	{
		AddFailure(TEXT("boundary_mean_depth"));
	}
	if (Metrics.MaxBoundaryMaxDepthHops > MaxAcceptableBoundaryMaxDepthHops)
	{
		AddFailure(TEXT("boundary_max_depth"));
	}
	if (Metrics.MaxNonGapPlateComponentCount != 1)
	{
		AddFailure(TEXT("plate_connectivity"));
	}
	if (Metrics.MaxDetachedPlateFragmentSampleCount != 0)
	{
		AddFailure(TEXT("detached_fragment_samples"));
	}
	if (Metrics.MaxLargestDetachedPlateFragmentSize != 0)
	{
		AddFailure(TEXT("detached_fragment_size"));
	}
	if (Metrics.MaxEmptyProtectedPlateCount != 0)
	{
		AddFailure(TEXT("empty_protected_plate"));
	}
	if (Metrics.MinObservedProtectedPlateSampleCount < Metrics.MinExpectedProtectedPlateFloor)
	{
		AddFailure(TEXT("protected_plate_floor"));
	}
	if (Metrics.InitialContinentalPlateCount != Bounds.ExpectedInitialContinentalPlateCount)
	{
		AddFailure(TEXT("initial_continental_plate_count"));
	}
	if (Metrics.InitialContinentalAreaFraction + UE_DOUBLE_SMALL_NUMBER < Bounds.InitialContinentalAreaFractionMin ||
		Metrics.InitialContinentalAreaFraction - UE_DOUBLE_SMALL_NUMBER > Bounds.InitialContinentalAreaFractionMax)
	{
		AddFailure(TEXT("initial_continental_area_fraction"));
	}
	if (Metrics.MaximumObservedElevationKm > MaxReferenceScenarioElevationCeilingKm + static_cast<double>(KINDA_SMALL_NUMBER))
	{
		AddFailure(TEXT("max_elevation"));
	}
	if (Metrics.FinalCollisionEventCount < Bounds.MinimumCollisionEventCount)
	{
		AddFailure(TEXT("collision_event_count"));
	}
	if (Metrics.FinalRiftEventCount < Bounds.MinimumRiftEventCount)
	{
		AddFailure(TEXT("rift_event_count"));
	}
	if (Metrics.PeakActivePlateCount < Bounds.MinimumPeakActivePlateCount)
	{
		AddFailure(TEXT("peak_active_plate_count"));
	}
	if (Bounds.bRequireAndeanWhenOceanicContinentalConvergenceExists &&
		Metrics.bObservedOceanicContinentalConvergence &&
		!Metrics.bObservedAndean)
	{
		AddFailure(TEXT("andean_observation"));
	}
	if (Bounds.bRequireHimalayanWhenCollisionOccurs &&
		Metrics.FinalCollisionEventCount > 0 &&
		!Metrics.bObservedHimalayan)
	{
		AddFailure(TEXT("himalayan_observation"));
	}

	return Failures.Num() > 0 ? FString::Join(Failures, TEXT(", ")) : TEXT("none");
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
		Metrics.MaximumObservedElevationKm <= MaxReferenceScenarioElevationCeilingKm + static_cast<double>(KINDA_SMALL_NUMBER) &&
		Metrics.FinalCollisionEventCount >= Bounds.MinimumCollisionEventCount &&
		Metrics.FinalRiftEventCount >= Bounds.MinimumRiftEventCount &&
		Metrics.PeakActivePlateCount >= Bounds.MinimumPeakActivePlateCount &&
		(!Bounds.bRequireAndeanWhenOceanicContinentalConvergenceExists ||
			!Metrics.bObservedOceanicContinentalConvergence ||
			Metrics.bObservedAndean) &&
		(!Bounds.bRequireHimalayanWhenCollisionOccurs ||
			Metrics.FinalCollisionEventCount <= 0 ||
			Metrics.bObservedHimalayan);

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
		if (!CollectReferenceScenarioObservedMetrics(CandidateScenario, BasePlanet, CandidateMetrics, 0.0, EContinentalStabilizerMode::Disabled))
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



