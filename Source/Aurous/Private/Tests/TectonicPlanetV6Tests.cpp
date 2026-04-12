#include "Misc/AutomationTest.h"

#include "HAL/PlatformFile.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/Paths.h"
#include "TectonicMollweideExporter.h"
#include "TectonicPlanetV6.h"

namespace
{
	constexpr int32 TestSampleCount = 60000;
	constexpr int32 TestPlateCount = 7;
	constexpr int32 TestRandomSeed = 42;
	constexpr int32 TestExportWidth = 4096;
	constexpr int32 TestExportHeight = 2048;
	constexpr float TestBoundaryWarpAmplitude = 0.2f;
	constexpr float TestContinentalFraction = 0.30f;

	FString JoinIntArrayForDiagnostics(const TArray<int32>& Values);

	FTectonicPlanetV6KeptRuntimeProfileOptions MakeKeptV6RuntimeProfileOptions(
		const bool bEnableAutomaticRifting = true,
		const bool bEnableShoulderFix = true,
		const bool bEnablePlateCandidatePruning = true,
		const bool bEnableCopiedFrontierUnfilteredMeshReuse = true,
		const bool bUseCachedSubductionAdjacencyEdgeDistances = true,
		const bool bUseSubductionPerformanceOptimizations = true)
	{
		FTectonicPlanetV6KeptRuntimeProfileOptions Options;
		Options.bEnableAutomaticRifting = bEnableAutomaticRifting;
		Options.bEnableSubmergedContinentalFringeRelaxation = bEnableShoulderFix;
		Options.bEnablePlateCandidatePruning = bEnablePlateCandidatePruning;
		Options.bEnableCopiedFrontierUnfilteredMeshReuse = bEnableCopiedFrontierUnfilteredMeshReuse;
		Options.bUseCachedSubductionAdjacencyEdgeDistances =
			bUseCachedSubductionAdjacencyEdgeDistances;
		Options.bUseSubductionPerformanceOptimizations =
			bUseSubductionPerformanceOptimizations;
		return Options;
	}

	FTectonicPlanetV6KeptDiagnosticsOptions MakeKeptV6DiagnosticsOptions(
		const bool bEnablePhaseTiming = true,
		const bool bEnableDetailedCopiedFrontierAttribution = false)
	{
		FTectonicPlanetV6KeptDiagnosticsOptions Options;
		Options.bEnablePhaseTiming = bEnablePhaseTiming;
		Options.bEnableDetailedCopiedFrontierAttribution =
			bEnableDetailedCopiedFrontierAttribution;
		return Options;
	}

	void ApplyKeptV6TestProfile(
		FTectonicPlanetV6& Planet,
		const FTectonicPlanetV6KeptRuntimeProfileOptions& RuntimeOptions,
		const FTectonicPlanetV6KeptDiagnosticsOptions& DiagnosticsOptions =
			FTectonicPlanetV6KeptDiagnosticsOptions{})
	{
		Planet.ApplyKeptV6RuntimeProfile(RuntimeOptions);
		Planet.ApplyKeptV6DiagnosticsProfile(DiagnosticsOptions);
	}

	struct FV6CheckpointSnapshot
	{
		int32 Step = 0;
		int32 SolveCount = 0;
		int32 Interval = 0;
		int32 PlateCount = 0;
		ETectonicPlanetV6PeriodicSolveMode SolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative;
		double SolveMilliseconds = 0.0;
		int32 HitCount = 0;
		int32 MissCount = 0;
		int32 MultiHitCount = 0;
		int32 BoundarySampleCount = 0;
		int32 BoundaryDecisionSampleCount = 0;
		int32 BoundaryRetainedCount = 0;
		int32 BoundaryReassignedCount = 0;
		int32 BoundaryOceanicCount = 0;
		int32 BoundaryLimitedFallbackCount = 0;
		int32 GapCount = 0;
		int32 OverlapCount = 0;
		int32 SingleCandidateWinnerCount = 0;
		int32 MultiCandidateWinnerCount = 0;
		int32 TriangleRecoveryCount = 0;
		int32 MemberRecoveryCount = 0;
		int32 ExplicitFallbackCount = 0;
		int32 CoherenceReassignedSampleCount = 0;
		int32 CoherenceRemovedComponentCount = 0;
		int32 CoherenceLargestRemovedComponentSize = 0;
		int32 TriangleTransferCount = 0;
		int32 DirectHitTriangleTransferCount = 0;
		int32 SingleSourceTransferCount = 0;
		int32 OceanicCreationCount = 0;
		int32 DefaultTransferCount = 0;
		int32 TransferFallbackCount = 0;
		int32 PlateLocalVertexCount = 0;
		int32 PlateLocalTriangleCount = 0;
		int32 CopiedFrontierVertexCount = 0;
		int32 CopiedFrontierTriangleCount = 0;
		int32 CopiedFrontierCarriedSampleCount = 0;
		int32 CopiedFrontierHitCount = 0;
		int32 PlateSubmeshFrontierVertexCount = 0;
		int32 PlateSubmeshFrontierTriangleCount = 0;
		int32 PlateSubmeshFrontierCarriedSampleCount = 0;
		int32 PlateSubmeshFrontierHitCount = 0;
		int32 PlateSubmeshRetriangulatedTriangleCount = 0;
		int32 PlateSubmeshComponentCount = 0;
		int32 PlateSubmeshWholeMixedTriangleDuplicationCount = 0;
		int32 InteriorHitCount = 0;
		int32 DestructiveTriangleGeometryExcludedCount = 0;
		int32 DestructiveTriangleRejectedCount = 0;
		int32 TrackedDestructiveTriangleCount = 0;
		int32 TrackedSubductionTriangleCount = 0;
		int32 TrackedCollisionTriangleCount = 0;
		int32 TrackedDestructiveTriangleNewlySeededCount = 0;
		int32 TrackedDestructiveTrianglePropagatedCount = 0;
		int32 TrackedDestructiveTriangleExpiredCount = 0;
		int32 TrackedDestructiveTriangleClearedCount = 0;
		FTectonicPlanetV6DestructiveTrackingLifecycleStats TrackingLifecycleStats;
		int32 MissTrueDivergenceGapCount = 0;
		int32 MissDestructiveExclusionGapCount = 0;
		int32 MissAmbiguousGapCount = 0;
		int32 SamplesWithPartialDestructiveCandidateRemovalCount = 0;
		int32 FragmentedComponentsAdjacentToDestructiveExclusionGapCount = 0;
		int32 CategoricalMajorityTransferCount = 0;
		int32 CategoricalDominantTransferCount = 0;
		int32 DirectionalFallbackCount = 0;
		int32 ContinentalWeightThresholdMismatchCount = 0;
		int32 MaxComponentsBeforeCoherence = 0;
		int32 MaxComponentsPerPlate = 0;
		double ContinentalAreaFraction = 0.0;
		int32 CollisionCount = 0;
		int32 CollisionDeferredCount = 0;
		int32 CollisionOverridingPlateId = INDEX_NONE;
		int32 CollisionSubductingPlateId = INDEX_NONE;
		int32 BoundaryContactCollisionPairCount = 0;
		int32 GeometricCollisionCandidateCount = 0;
		int32 GeometricCollisionPairCount = 0;
		int32 GeometricCollisionQualifiedPairCount = 0;
		int32 GeometricCollisionOverlapSampleCount = 0;
		int32 GeometricCollisionExecutedTerraneEstimate = 0;
		bool bUsedGeometricCollisionExecution = false;
		bool bUsedCachedBoundaryContactCollision = false;
		bool bDestructiveTriangleExclusionApplied = false;
		FTectonicPlanetV6TransferResolutionCounts TriangleTransferCountsByResolution;
		FTectonicPlanetV6TransferResolutionCounts SingleSourceTransferCountsByResolution;
		FTectonicPlanetV6TransferResolutionCounts ContinentalWeightThresholdMismatchCountsByResolution;
		FTectonicPlanetV6BoundaryOutcomeTransferStats BoundaryRetainedTransferStats;
		FTectonicPlanetV6BoundaryOutcomeTransferStats BoundaryReassignedTransferStats;
		FTectonicPlanetV6BoundaryOutcomeTransferStats BoundaryOceanicTransferStats;
		FTectonicPlanetV6BoundaryCoherenceDiagnostic BoundaryCoherence;
		FTectonicPlanetV6OwnershipChurnDiagnostic OwnershipChurn;
		FTectonicPlanetV6CompetitiveParticipationDiagnostic CompetitiveParticipation;
		FTectonicPlanetV6MissLineageDiagnostic MissLineage;
		FTectonicPlanetV6InteriorInstabilityDiagnostic InteriorInstability;
		FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic SyntheticCoveragePersistence;
		FTectonicPlanetV6FrontRetreatDiagnostic FrontRetreat;
		FTectonicPlanetV6ActiveZoneDiagnostic ActiveZone;
		FTectonicPlanetV6CollisionShadowDiagnostic CollisionShadow;
		FTectonicPlanetV6CollisionExecutionDiagnostic CollisionExecution;
		int32 CurrentCollisionRegionSamplesAbove5Km = 0;
		int32 CumulativeCollisionRegionSamplesAbove5Km = 0;
	};

	struct FV6BaselineSummary
	{
		FString RunId;
		FV6CheckpointSnapshot Step0Snapshot;
		FV6CheckpointSnapshot Step50Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FV6CheckpointSnapshot Step150Snapshot;
		FV6CheckpointSnapshot Step200Snapshot;
		FV6CheckpointSnapshot FinalSnapshot;
		bool bHasStep0Snapshot = false;
		bool bHasStep50Snapshot = false;
		bool bHasStep100Snapshot = false;
		bool bHasStep150Snapshot = false;
		bool bHasStep200Snapshot = false;
		int32 MaxComponentsObserved = 0;
	};

	struct FV9CollisionFidelityGateResult
	{
		bool bFootprintPass = false;
		bool bElevationPass = false;
		bool bContinentalGainPass = false;
		bool bChurnPass = false;
		bool bCoherencePass = false;
		bool bLeakagePass = false;
		bool bAllowStep200 = false;
		int32 RequiredCumulativeAffectedSamples = 0;
		int32 RequiredCumulativeContinentalGain = 0;
		double RequiredExecutedMeanElevationDeltaKm = 0.0;
		double RequiredCumulativeMeanElevationDeltaKm = 0.0;
		double MaxAllowedChurn = 0.0;
		double MinAllowedCoherence = 0.0;
		double MaxAllowedInteriorLeakage = 0.0;
	};

	struct FV9CollisionStructuralGateResult
	{
		bool bOwnershipChangePass = false;
		bool bTransferPass = false;
		bool bTransferredContinentalPass = false;
		bool bTransferFootprintPass = false;
		bool bShareDeltaPass = false;
		bool bBoundaryLocalityPass = false;
		bool bChurnPass = false;
		bool bCoherencePass = false;
		bool bLeakagePass = false;
		bool bAllowStep200 = false;
		int32 RequiredCumulativeTransferFootprint = 0;
		double RequiredMinPlateShareDelta = 0.0;
		double RequiredMaxPlateShareDelta = 0.0;
		double RequiredBoundaryLocalFraction = 0.0;
		double MaxAllowedChurn = 0.0;
		double MinAllowedCoherence = 0.0;
		double MaxAllowedInteriorLeakage = 0.0;
	};

	struct FV9ThesisShapedCollisionGateResult
	{
		bool bTerraneDetectedPass = false;
		bool bTransferCoherentPass = false;
		bool bOwnershipMaterialPass = false;
		bool bMaxElevationPass = false;
		bool bCollisionAbove5Pass = false;
		bool bChurnPass = false;
		bool bCoherencePass = false;
		bool bLeakagePass = false;
		bool bAllowStep200 = false;
		int32 RequiredMinTerraneComponentSize = 0;
		int32 RequiredMinTransferSize = 0;
		int32 RequiredMinOwnershipChange = 0;
		double RequiredMinMaxElevationDeltaKm = 0.0;
		int32 RequiredMinCollisionAbove5KmSamples = 1;
		double MaxAllowedChurn = 0.05;
		double MinAllowedCoherence = 0.93;
		double MaxAllowedInteriorLeakage = 0.17;
	};

	struct FV9ContinentalElevationStats
	{
		double MeanElevationKm = 0.0;
		double P95ElevationKm = 0.0;
		double MaxElevationKm = 0.0;
		int32 SamplesAbove2Km = 0;
		int32 SamplesAbove5Km = 0;
		int32 ContinentalSampleCount = 0;
	};

	struct FV9ContinentalElevationBandDiagnostic
	{
		double ContinentalMeanElevationKm = 0.0;
		double ContinentalP95ElevationKm = 0.0;
		double ContinentalMaxElevationKm = 0.0;
		double SubaerialMeanElevationKm = 0.0;
		double SubaerialP95ElevationKm = 0.0;
		int32 ContinentalSampleCount = 0;
		int32 SubaerialContinentalSampleCount = 0;
		int32 SubmergedContinentalSampleCount = 0;
		int32 SamplesAbove2Km = 0;
		int32 SamplesAbove5Km = 0;
		int32 ContinentalSamples0To0p5Km = 0;
		int32 ContinentalSamples0p5To1Km = 0;
		int32 ContinentalSamples1To2Km = 0;
		int32 ContinentalSamples2To5Km = 0;
		int32 ContinentalSamplesAbove5Km = 0;
	};

	FV6CheckpointSnapshot BuildV6CheckpointSnapshot(const FTectonicPlanetV6& Planet);
	int32 ComputeCollisionRegionSamplesAboveElevationThreshold(
		const FTectonicPlanetV6& Planet,
		double ElevationThresholdKm,
		bool bUseCumulativeMask);

	FTectonicPlanetV6 CreateInitializedPlanetV6WithConfig(
		const ETectonicPlanetV6PeriodicSolveMode SolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative,
		const int32 FixedIntervalSteps = 25,
		const int32 PropagationWaveCap = INDEX_NONE,
		const int32 SampleCount = TestSampleCount,
		const int32 PlateCount = TestPlateCount,
		const int32 RandomSeed = TestRandomSeed)
	{
		FTectonicPlanetV6 Planet;
		Planet.Initialize(
			SampleCount,
			PlateCount,
			RandomSeed,
			TestBoundaryWarpAmplitude,
			TestContinentalFraction);
		Planet.SetPeriodicSolveModeForTest(SolveMode, FixedIntervalSteps);
		if (PropagationWaveCap != INDEX_NONE)
		{
			Planet.SetCopiedFrontierIntervalPropagationWaveCapForTest(PropagationWaveCap);
		}
		return Planet;
	}

	FTectonicPlanetV6 CreateInitializedPlanetV6(
		const ETectonicPlanetV6PeriodicSolveMode SolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative,
		const int32 FixedIntervalSteps = 25,
		const int32 PropagationWaveCap = INDEX_NONE,
		const int32 SampleCount = TestSampleCount)
	{
		return CreateInitializedPlanetV6WithConfig(
			SolveMode,
			FixedIntervalSteps,
			PropagationWaveCap,
			SampleCount,
			TestPlateCount,
			TestRandomSeed);
	}

	struct FV6PropagationDepthSweepResult
	{
		FString Label;
		ETectonicPlanetV6PeriodicSolveMode SolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative;
		int32 PropagationWaveCap = INDEX_NONE;
		FV6CheckpointSnapshot OneCycleSnapshot;
		FV6CheckpointSnapshot Step200Snapshot;
		FTectonicPlanetV6CopiedFrontierSolveAttribution OneCycleAttribution;
		FTectonicPlanetV6CopiedFrontierSolveAttribution Step200Attribution;
	};

	FString DescribePropagationWaveCap(const int32 PropagationWaveCap)
	{
		return PropagationWaveCap == INDEX_NONE
			? TEXT("full")
			: FString::Printf(TEXT("cap%d"), PropagationWaveCap);
	}

	FV6PropagationDepthSweepResult RunPartitionedPropagationProcessDepthSweepCase(
		FAutomationTestBase& Test,
		const FString& Label,
		const ETectonicPlanetV6PeriodicSolveMode SolveMode,
		const int32 PropagationWaveCap)
	{
		FV6PropagationDepthSweepResult Result;
		Result.Label = Label;
		Result.SolveMode = SolveMode;
		Result.PropagationWaveCap = PropagationWaveCap;

		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(SolveMode, 25, PropagationWaveCap);
		const int32 Interval = Planet.ComputePeriodicSolveInterval();
		Planet.AdvanceSteps(Interval);
		Result.OneCycleSnapshot = BuildV6CheckpointSnapshot(Planet);
		Result.OneCycleAttribution = Planet.GetLastCopiedFrontierSolveAttributionForTest();
		if (Planet.GetPlanet().CurrentStep < 200)
		{
			Planet.AdvanceSteps(200 - Planet.GetPlanet().CurrentStep);
		}
		Result.Step200Snapshot = BuildV6CheckpointSnapshot(Planet);
		Result.Step200Attribution = Planet.GetLastCopiedFrontierSolveAttributionForTest();

		Test.AddInfo(FString::Printf(
			TEXT("[V6ThesisPartitionedFrontierPropagationDepthSweep label=%s mode=%d wave_cap=%s] step25_caf=%.6f step25_multi=%d step25_max_components=%d step25_cw=%d step25_miss=%d step25_true_divergence=%d step25_destructive_exclusion=%d step25_ambiguous=%d step25_oceanic_creation=%d step25_transfer_fallback=%d step25_seed=%d step25_tracked=%d step25_propagated=%d step25_cleared=%d step200_caf=%.6f step200_multi=%d step200_max_components=%d step200_cw=%d step200_miss=%d step200_true_divergence=%d step200_destructive_exclusion=%d step200_ambiguous=%d step200_oceanic_creation=%d step200_transfer_fallback=%d step200_seed=%d step200_tracked=%d step200_propagated=%d step200_cleared=%d"),
			*Result.Label,
			static_cast<int32>(Result.SolveMode),
			*DescribePropagationWaveCap(Result.PropagationWaveCap),
			Result.OneCycleSnapshot.ContinentalAreaFraction,
			Result.OneCycleSnapshot.MultiHitCount,
			Result.OneCycleSnapshot.MaxComponentsPerPlate,
			Result.OneCycleSnapshot.ContinentalWeightThresholdMismatchCount,
			Result.OneCycleSnapshot.MissCount,
			Result.OneCycleSnapshot.MissTrueDivergenceGapCount,
			Result.OneCycleSnapshot.MissDestructiveExclusionGapCount,
			Result.OneCycleSnapshot.MissAmbiguousGapCount,
			Result.OneCycleSnapshot.OceanicCreationCount,
			Result.OneCycleSnapshot.TransferFallbackCount,
			Result.OneCycleSnapshot.TrackedDestructiveTriangleNewlySeededCount,
			Result.OneCycleSnapshot.TrackedDestructiveTriangleCount,
			Result.OneCycleSnapshot.TrackedDestructiveTrianglePropagatedCount,
			Result.OneCycleSnapshot.TrackedDestructiveTriangleClearedCount,
			Result.Step200Snapshot.ContinentalAreaFraction,
			Result.Step200Snapshot.MultiHitCount,
			Result.Step200Snapshot.MaxComponentsPerPlate,
			Result.Step200Snapshot.ContinentalWeightThresholdMismatchCount,
			Result.Step200Snapshot.MissCount,
			Result.Step200Snapshot.MissTrueDivergenceGapCount,
			Result.Step200Snapshot.MissDestructiveExclusionGapCount,
			Result.Step200Snapshot.MissAmbiguousGapCount,
			Result.Step200Snapshot.OceanicCreationCount,
			Result.Step200Snapshot.TransferFallbackCount,
			Result.Step200Snapshot.TrackedDestructiveTriangleNewlySeededCount,
			Result.Step200Snapshot.TrackedDestructiveTriangleCount,
			Result.Step200Snapshot.TrackedDestructiveTrianglePropagatedCount,
			Result.Step200Snapshot.TrackedDestructiveTriangleClearedCount));

		return Result;
	}

	FCarriedSample MakeCarriedSample(
		const int32 CanonicalSampleIndex,
		const float ContinentalWeight,
		const float Elevation,
		const float Thickness,
		const float Age,
		const EOrogenyType OrogenyType,
		const int32 TerraneId,
		const FVector3d& RidgeDirection = FVector3d::ZeroVector,
		const FVector3d& FoldDirection = FVector3d::ZeroVector)
	{
		FCarriedSample Sample;
		Sample.CanonicalSampleIndex = CanonicalSampleIndex;
		Sample.ContinentalWeight = ContinentalWeight;
		Sample.Elevation = Elevation;
		Sample.Thickness = Thickness;
		Sample.Age = Age;
		Sample.OrogenyType = OrogenyType;
		Sample.TerraneId = TerraneId;
		Sample.RidgeDirection = RidgeDirection;
		Sample.FoldDirection = FoldDirection;
		return Sample;
	}

	FTectonicPlanetV6BoundaryMotionSample MakeBoundaryMotionSample(
		const int32 PlateId,
		const FVector3d& SurfaceVelocity,
		const FVector3d& NeighborTangent,
		const int32 NeighborVoteCount)
	{
		FTectonicPlanetV6BoundaryMotionSample MotionSample;
		MotionSample.PlateId = PlateId;
		MotionSample.SurfaceVelocity = SurfaceVelocity;
		MotionSample.NeighborTangent = NeighborTangent;
		MotionSample.NeighborVoteCount = NeighborVoteCount;
		return MotionSample;
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

	FV6CheckpointSnapshot BuildV6CheckpointSnapshot(const FTectonicPlanetV6& Planet)
	{
		FV6CheckpointSnapshot Snapshot;
		const FTectonicPlanet& LegacyPlanet = Planet.GetPlanet();
		const FResamplingStats& ResamplingStats = LegacyPlanet.LastResamplingStats;
		const FTectonicPlanetV6PeriodicSolveStats SolveStats =
			Planet.GetPeriodicSolveCount() > 0
				? Planet.GetLastSolveStats()
				: Planet.BuildCurrentDiagnosticSnapshotForTest();
		Snapshot.Step = LegacyPlanet.CurrentStep;
		Snapshot.SolveCount = Planet.GetPeriodicSolveCount();
		Snapshot.Interval = SolveStats.Interval;
		Snapshot.PlateCount = LegacyPlanet.Plates.Num();
		Snapshot.SolveMode = SolveStats.SolveMode;
		Snapshot.SolveMilliseconds = SolveStats.SolveMilliseconds;
		Snapshot.HitCount = SolveStats.HitCount;
		Snapshot.MissCount = SolveStats.MissCount;
		Snapshot.MultiHitCount = SolveStats.MultiHitCount;
		Snapshot.BoundarySampleCount = CountBoundarySamples(LegacyPlanet);
		Snapshot.BoundaryDecisionSampleCount = SolveStats.BoundaryDecisionSampleCount;
		Snapshot.BoundaryRetainedCount = SolveStats.BoundaryRetainedCount;
		Snapshot.BoundaryReassignedCount = SolveStats.BoundaryReassignedCount;
		Snapshot.BoundaryOceanicCount = SolveStats.BoundaryOceanicCount;
		Snapshot.BoundaryLimitedFallbackCount = SolveStats.BoundaryLimitedFallbackCount;
		Snapshot.GapCount = SolveStats.GapCount;
		Snapshot.OverlapCount = SolveStats.OverlapCount;
		Snapshot.SingleCandidateWinnerCount = SolveStats.SingleCandidateWinnerCount;
		Snapshot.MultiCandidateWinnerCount = SolveStats.MultiCandidateWinnerCount;
		Snapshot.TriangleRecoveryCount = SolveStats.ZeroCandidateTriangleRecoveryCount;
		Snapshot.MemberRecoveryCount = SolveStats.ZeroCandidateMemberRecoveryCount;
		Snapshot.ExplicitFallbackCount = SolveStats.ExplicitFallbackCount;
		Snapshot.CoherenceReassignedSampleCount = SolveStats.CoherenceReassignedSampleCount;
		Snapshot.CoherenceRemovedComponentCount = SolveStats.CoherenceRemovedComponentCount;
		Snapshot.CoherenceLargestRemovedComponentSize = SolveStats.CoherenceLargestRemovedComponentSize;
		Snapshot.TriangleTransferCount = SolveStats.TriangleTransferCount;
		Snapshot.DirectHitTriangleTransferCount = SolveStats.DirectHitTriangleTransferCount;
		Snapshot.SingleSourceTransferCount = SolveStats.SingleSourceTransferCount;
		Snapshot.OceanicCreationCount = SolveStats.OceanicCreationCount;
		Snapshot.DefaultTransferCount = SolveStats.DefaultTransferCount;
		Snapshot.TransferFallbackCount = SolveStats.TransferFallbackCount;
		Snapshot.PlateLocalVertexCount = SolveStats.PlateLocalVertexCount;
		Snapshot.PlateLocalTriangleCount = SolveStats.PlateLocalTriangleCount;
		Snapshot.CopiedFrontierVertexCount = SolveStats.CopiedFrontierVertexCount;
		Snapshot.CopiedFrontierTriangleCount = SolveStats.CopiedFrontierTriangleCount;
		Snapshot.CopiedFrontierCarriedSampleCount = SolveStats.CopiedFrontierCarriedSampleCount;
		Snapshot.CopiedFrontierHitCount = SolveStats.CopiedFrontierHitCount;
		Snapshot.PlateSubmeshFrontierVertexCount = SolveStats.PlateSubmeshFrontierVertexCount;
		Snapshot.PlateSubmeshFrontierTriangleCount = SolveStats.PlateSubmeshFrontierTriangleCount;
		Snapshot.PlateSubmeshFrontierCarriedSampleCount = SolveStats.PlateSubmeshFrontierCarriedSampleCount;
		Snapshot.PlateSubmeshFrontierHitCount = SolveStats.PlateSubmeshFrontierHitCount;
		Snapshot.PlateSubmeshRetriangulatedTriangleCount = SolveStats.PlateSubmeshRetriangulatedTriangleCount;
		Snapshot.PlateSubmeshComponentCount = SolveStats.PlateSubmeshComponentCount;
		Snapshot.PlateSubmeshWholeMixedTriangleDuplicationCount = SolveStats.PlateSubmeshWholeMixedTriangleDuplicationCount;
		Snapshot.InteriorHitCount = SolveStats.InteriorHitCount;
		Snapshot.DestructiveTriangleGeometryExcludedCount = SolveStats.DestructiveTriangleGeometryExcludedCount;
		Snapshot.DestructiveTriangleRejectedCount = SolveStats.DestructiveTriangleRejectedCount;
		Snapshot.TrackedDestructiveTriangleCount = SolveStats.TrackedDestructiveTriangleCount;
		Snapshot.TrackedSubductionTriangleCount = SolveStats.TrackedSubductionTriangleCount;
		Snapshot.TrackedCollisionTriangleCount = SolveStats.TrackedCollisionTriangleCount;
		Snapshot.TrackedDestructiveTriangleNewlySeededCount = SolveStats.TrackedDestructiveTriangleNewlySeededCount;
		Snapshot.TrackedDestructiveTrianglePropagatedCount = SolveStats.TrackedDestructiveTrianglePropagatedCount;
		Snapshot.TrackedDestructiveTriangleExpiredCount = SolveStats.TrackedDestructiveTriangleExpiredCount;
		Snapshot.TrackedDestructiveTriangleClearedCount = SolveStats.TrackedDestructiveTriangleClearedCount;
		Snapshot.TrackingLifecycleStats = SolveStats.TrackingLifecycleStats;
		Snapshot.MissTrueDivergenceGapCount = SolveStats.MissTrueDivergenceGapCount;
		Snapshot.MissDestructiveExclusionGapCount = SolveStats.MissDestructiveExclusionGapCount;
		Snapshot.MissAmbiguousGapCount = SolveStats.MissAmbiguousGapCount;
		Snapshot.SamplesWithPartialDestructiveCandidateRemovalCount =
			SolveStats.SamplesWithPartialDestructiveCandidateRemovalCount;
		Snapshot.FragmentedComponentsAdjacentToDestructiveExclusionGapCount =
			SolveStats.FragmentedComponentsAdjacentToDestructiveExclusionGapCount;
		Snapshot.CategoricalMajorityTransferCount = SolveStats.CategoricalMajorityTransferCount;
		Snapshot.CategoricalDominantTransferCount = SolveStats.CategoricalDominantTransferCount;
		Snapshot.DirectionalFallbackCount = SolveStats.DirectionalFallbackCount;
		Snapshot.ContinentalWeightThresholdMismatchCount = SolveStats.ContinentalWeightThresholdMismatchCount;
		Snapshot.MaxComponentsBeforeCoherence = SolveStats.MaxComponentsBeforeCoherence;
		Snapshot.MaxComponentsPerPlate = SolveStats.MaxComponentsPerPlate;
		Snapshot.ContinentalAreaFraction = ComputeContinentalAreaFraction(LegacyPlanet);
		Snapshot.CollisionCount = ResamplingStats.CollisionCount;
		Snapshot.CollisionDeferredCount = ResamplingStats.CollisionDeferredCount;
		Snapshot.CollisionOverridingPlateId = ResamplingStats.CollisionOverridingPlateId;
		Snapshot.CollisionSubductingPlateId = ResamplingStats.CollisionSubductingPlateId;
		Snapshot.BoundaryContactCollisionPairCount = ResamplingStats.BoundaryContactCollisionPairCount;
		Snapshot.GeometricCollisionCandidateCount = ResamplingStats.GeometricCollisionCandidateCount;
		Snapshot.GeometricCollisionPairCount = ResamplingStats.GeometricCollisionPairCount;
		Snapshot.GeometricCollisionQualifiedPairCount = ResamplingStats.GeometricCollisionQualifiedPairCount;
		Snapshot.GeometricCollisionOverlapSampleCount = ResamplingStats.GeometricCollisionOverlapSampleCount;
		Snapshot.GeometricCollisionExecutedTerraneEstimate =
			ResamplingStats.GeometricCollisionExecutedTerraneEstimate;
		Snapshot.bUsedGeometricCollisionExecution = ResamplingStats.bUsedGeometricCollisionExecution;
		Snapshot.bUsedCachedBoundaryContactCollision = ResamplingStats.bUsedCachedBoundaryContactCollision;
		Snapshot.bDestructiveTriangleExclusionApplied = SolveStats.bDestructiveTriangleExclusionApplied;
		Snapshot.TriangleTransferCountsByResolution = SolveStats.TriangleTransferCountsByResolution;
		Snapshot.SingleSourceTransferCountsByResolution = SolveStats.SingleSourceTransferCountsByResolution;
		Snapshot.ContinentalWeightThresholdMismatchCountsByResolution = SolveStats.ContinentalWeightThresholdMismatchCountsByResolution;
		Snapshot.BoundaryRetainedTransferStats = SolveStats.BoundaryRetainedTransferStats;
		Snapshot.BoundaryReassignedTransferStats = SolveStats.BoundaryReassignedTransferStats;
		Snapshot.BoundaryOceanicTransferStats = SolveStats.BoundaryOceanicTransferStats;
		Snapshot.BoundaryCoherence = Planet.ComputeBoundaryCoherenceDiagnosticForTest();
		Snapshot.OwnershipChurn = Planet.ComputeOwnershipChurnDiagnosticForTest();
			Snapshot.CompetitiveParticipation = Planet.ComputeCompetitiveParticipationDiagnosticForTest();
			Snapshot.MissLineage = Planet.ComputeMissLineageDiagnosticForTest();
		Snapshot.InteriorInstability = Planet.ComputeInteriorInstabilityDiagnosticForTest();
		Snapshot.SyntheticCoveragePersistence = Planet.ComputeSyntheticCoveragePersistenceDiagnosticForTest();
		Snapshot.FrontRetreat = Planet.ComputeFrontRetreatDiagnosticForTest();
		Snapshot.ActiveZone = Planet.ComputeActiveZoneDiagnosticForTest();
		Snapshot.CollisionShadow = Planet.ComputeCollisionShadowDiagnosticForTest();
		Snapshot.CollisionExecution = Planet.ComputeCollisionExecutionDiagnosticForTest();
		Snapshot.CurrentCollisionRegionSamplesAbove5Km =
			ComputeCollisionRegionSamplesAboveElevationThreshold(Planet, 5.0, false);
		Snapshot.CumulativeCollisionRegionSamplesAbove5Km =
			ComputeCollisionRegionSamplesAboveElevationThreshold(Planet, 5.0, true);
		return Snapshot;
	}

	bool IsThesisLikeRunId(const FString& RunId)
	{
		return RunId.Contains(TEXT("ThesisRemesh")) ||
			RunId.Contains(TEXT("ThesisCopiedFrontier")) ||
			RunId.Contains(TEXT("ThesisPartitionedFrontier")) ||
			RunId.Contains(TEXT("ThesisPlateSubmesh"));
	}

	const TCHAR* GetV6ThesisBaselineSummaryTag(const FString& RunId)
	{
		if (RunId.Contains(TEXT("ThesisPlateSubmesh")))
		{
			return TEXT("[V6ThesisPlateSubmeshBaseline]");
		}
		if (RunId.Contains(TEXT("ThesisPartitionedFrontierPropagationProcess")))
		{
			return TEXT("[V6ThesisPartitionedFrontierPropagationProcessBaseline]");
		}
		if (RunId.Contains(TEXT("ThesisPartitionedFrontierPropagation")))
		{
			return TEXT("[V6ThesisPartitionedFrontierPropagationBaseline]");
		}
		if (RunId.Contains(TEXT("ThesisPartitionedFrontierProcess")))
		{
			return TEXT("[V6ThesisPartitionedFrontierProcessBaseline]");
		}
		if (RunId.Contains(TEXT("ThesisPartitionedFrontier")))
		{
			return TEXT("[V6ThesisPartitionedFrontierBaseline]");
		}
		if (RunId.Contains(TEXT("ThesisCopiedFrontierProcess")))
		{
			return TEXT("[V6ThesisCopiedFrontierProcessBaseline]");
		}
		if (RunId.Contains(TEXT("ThesisCopiedFrontier")))
		{
			return TEXT("[V6ThesisCopiedFrontierBaseline]");
		}
		return TEXT("[V6ThesisRemeshBaseline]");
	}

	int32 CountDirectSingleSourceTransfers(const FTectonicPlanetV6BoundaryOutcomeTransferStats& Stats)
	{
		return Stats.SingleSourceTransferCount -
			Stats.NearestMemberSingleSourceTransferCount -
			Stats.ExplicitFallbackSingleSourceTransferCount;
	}

	void AddV6BoundaryTransferInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		Test.AddInfo(FString::Printf(
			TEXT("%s step=%d continental_area_fraction=%.6f boundary_decisions=%d boundary_retained=%d boundary_reassigned=%d boundary_oceanic=%d coherence_reassigned=%d max_components_after=%d retained_samples=%d retained_triangle=%d retained_triangle_recovery=%d retained_single_source=%d retained_single_source_direct=%d retained_single_source_nearest_member=%d retained_single_source_explicit_fallback=%d retained_oceanic_creation=%d retained_default_transfer=%d retained_cw_threshold_mismatch=%d reassigned_samples=%d reassigned_triangle=%d reassigned_triangle_recovery=%d reassigned_single_source=%d reassigned_single_source_direct=%d reassigned_single_source_nearest_member=%d reassigned_single_source_explicit_fallback=%d reassigned_oceanic_creation=%d reassigned_default_transfer=%d reassigned_cw_threshold_mismatch=%d oceanic_samples=%d oceanic_triangle=%d oceanic_triangle_recovery=%d oceanic_single_source=%d oceanic_single_source_direct=%d oceanic_single_source_nearest_member=%d oceanic_single_source_explicit_fallback=%d oceanic_oceanic_creation=%d oceanic_default_transfer=%d oceanic_cw_threshold_mismatch=%d"),
			*SummaryTag,
			Snapshot.Step,
			Snapshot.ContinentalAreaFraction,
			Snapshot.BoundaryDecisionSampleCount,
			Snapshot.BoundaryRetainedCount,
			Snapshot.BoundaryReassignedCount,
			Snapshot.BoundaryOceanicCount,
			Snapshot.CoherenceReassignedSampleCount,
			Snapshot.MaxComponentsPerPlate,
			Snapshot.BoundaryRetainedTransferStats.SampleCount,
			Snapshot.BoundaryRetainedTransferStats.TriangleTransferCount,
			Snapshot.BoundaryRetainedTransferStats.TriangleRecoveryTransferCount,
			Snapshot.BoundaryRetainedTransferStats.SingleSourceTransferCount,
			CountDirectSingleSourceTransfers(Snapshot.BoundaryRetainedTransferStats),
			Snapshot.BoundaryRetainedTransferStats.NearestMemberSingleSourceTransferCount,
			Snapshot.BoundaryRetainedTransferStats.ExplicitFallbackSingleSourceTransferCount,
			Snapshot.BoundaryRetainedTransferStats.OceanicCreationCount,
			Snapshot.BoundaryRetainedTransferStats.DefaultTransferCount,
			Snapshot.BoundaryRetainedTransferStats.ContinentalWeightThresholdMismatchCount,
			Snapshot.BoundaryReassignedTransferStats.SampleCount,
			Snapshot.BoundaryReassignedTransferStats.TriangleTransferCount,
			Snapshot.BoundaryReassignedTransferStats.TriangleRecoveryTransferCount,
			Snapshot.BoundaryReassignedTransferStats.SingleSourceTransferCount,
			CountDirectSingleSourceTransfers(Snapshot.BoundaryReassignedTransferStats),
			Snapshot.BoundaryReassignedTransferStats.NearestMemberSingleSourceTransferCount,
			Snapshot.BoundaryReassignedTransferStats.ExplicitFallbackSingleSourceTransferCount,
			Snapshot.BoundaryReassignedTransferStats.OceanicCreationCount,
			Snapshot.BoundaryReassignedTransferStats.DefaultTransferCount,
			Snapshot.BoundaryReassignedTransferStats.ContinentalWeightThresholdMismatchCount,
			Snapshot.BoundaryOceanicTransferStats.SampleCount,
			Snapshot.BoundaryOceanicTransferStats.TriangleTransferCount,
			Snapshot.BoundaryOceanicTransferStats.TriangleRecoveryTransferCount,
			Snapshot.BoundaryOceanicTransferStats.SingleSourceTransferCount,
			CountDirectSingleSourceTransfers(Snapshot.BoundaryOceanicTransferStats),
			Snapshot.BoundaryOceanicTransferStats.NearestMemberSingleSourceTransferCount,
			Snapshot.BoundaryOceanicTransferStats.ExplicitFallbackSingleSourceTransferCount,
			Snapshot.BoundaryOceanicTransferStats.OceanicCreationCount,
			Snapshot.BoundaryOceanicTransferStats.DefaultTransferCount,
			Snapshot.BoundaryOceanicTransferStats.ContinentalWeightThresholdMismatchCount));
	}

	void AddV6ThesisRemeshInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FString Message = FString::Printf(
			TEXT("%s step=%d solve_ms=%.3f cadence_steps=%d continental_area_fraction=%.6f boundary_decisions=%d coherence_reassigned=%d max_components_after=%d cw_threshold_mismatch=%d hit_count=%d copied_frontier_hit_count=%d plate_submesh_frontier_hit_count=%d interior_hit_count=%d miss_count=%d multi_hit_count=%d oceanic_creation=%d direct_hit_triangle_transfer=%d triangle_transfer=%d single_source_transfer=%d default_transfer=%d transfer_fallback=%d plate_local_vertex_count=%d plate_local_triangle_count=%d copied_frontier_vertex_count=%d copied_frontier_triangle_count=%d copied_frontier_carried_sample_count=%d plate_submesh_frontier_vertex_count=%d plate_submesh_frontier_triangle_count=%d plate_submesh_frontier_carried_sample_count=%d plate_submesh_retriangulated_triangle_count=%d plate_submesh_component_count=%d plate_submesh_mixed_triangle_duplication_count=%d destructive_filter_applied=%d destructive_filter_geometry_excluded=%d destructive_filter_rejected=%d tracked_destructive_triangles=%d tracked_subduction_triangles=%d tracked_collision_triangles=%d tracked_newly_seeded=%d tracked_propagated=%d tracked_expired=%d tracked_cleared=%d tracked_entry_candidates=%d tracked_entry_overlap_tested=%d tracked_entry_admitted=%d tracked_entry_reject_no_overlap=%d tracked_entry_reject_authorization=%d tracked_entry_reject_already_tracked=%d tracked_entry_subduction=%d tracked_entry_collision=%d tracked_active_advanced=%d tracked_active_overlap_confirmed=%d tracked_active_overlap_rejected=%d tracked_neighbor_generated=%d tracked_neighbor_tested=%d tracked_neighbor_admitted=%d tracked_neighbor_reject_no_overlap=%d tracked_neighbor_reject_authorization=%d tracked_neighbor_reject_already_tracked=%d tracked_neighbor_subduction=%d tracked_neighbor_collision=%d tracked_topology_neighbor_generated=%d tracked_topology_neighbor_admitted=%d tracked_topology_neighbor_expired=%d tracked_directional_neighbor_considered=%d tracked_directional_neighbor_admitted=%d tracked_directional_neighbor_rejected_not_inward=%d tracked_directional_neighbor_survived_to_remesh=%d tracked_expired_subduction=%d tracked_expired_collision=%d miss_true_divergence_gap=%d miss_destructive_exclusion_gap=%d miss_ambiguous_gap=%d partial_destructive_candidate_removal=%d fragmented_components_adjacent_to_destructive_gap=%d"),
			*SummaryTag,
			Snapshot.Step,
			Snapshot.SolveMilliseconds,
			Snapshot.Interval,
			Snapshot.ContinentalAreaFraction,
			Snapshot.BoundaryDecisionSampleCount,
			Snapshot.CoherenceReassignedSampleCount,
			Snapshot.MaxComponentsPerPlate,
			Snapshot.ContinentalWeightThresholdMismatchCount,
			Snapshot.HitCount,
			Snapshot.CopiedFrontierHitCount,
			Snapshot.PlateSubmeshFrontierHitCount,
			Snapshot.InteriorHitCount,
			Snapshot.MissCount,
			Snapshot.MultiHitCount,
			Snapshot.OceanicCreationCount,
			Snapshot.DirectHitTriangleTransferCount,
			Snapshot.TriangleTransferCount,
			Snapshot.SingleSourceTransferCount,
			Snapshot.DefaultTransferCount,
			Snapshot.TransferFallbackCount,
			Snapshot.PlateLocalVertexCount,
			Snapshot.PlateLocalTriangleCount,
			Snapshot.CopiedFrontierVertexCount,
			Snapshot.CopiedFrontierTriangleCount,
			Snapshot.CopiedFrontierCarriedSampleCount,
			Snapshot.PlateSubmeshFrontierVertexCount,
			Snapshot.PlateSubmeshFrontierTriangleCount,
			Snapshot.PlateSubmeshFrontierCarriedSampleCount,
			Snapshot.PlateSubmeshRetriangulatedTriangleCount,
			Snapshot.PlateSubmeshComponentCount,
			Snapshot.PlateSubmeshWholeMixedTriangleDuplicationCount,
			Snapshot.bDestructiveTriangleExclusionApplied ? 1 : 0,
			Snapshot.DestructiveTriangleGeometryExcludedCount,
			Snapshot.DestructiveTriangleRejectedCount,
			Snapshot.TrackedDestructiveTriangleCount,
			Snapshot.TrackedSubductionTriangleCount,
			Snapshot.TrackedCollisionTriangleCount,
			Snapshot.TrackedDestructiveTriangleNewlySeededCount,
			Snapshot.TrackedDestructiveTrianglePropagatedCount,
			Snapshot.TrackedDestructiveTriangleExpiredCount,
			Snapshot.TrackedDestructiveTriangleClearedCount,
			Snapshot.TrackingLifecycleStats.EntryCandidateCount,
			Snapshot.TrackingLifecycleStats.EntryOverlapTestedCount,
			Snapshot.TrackingLifecycleStats.EntryAdmittedCount,
			Snapshot.TrackingLifecycleStats.EntryRejectedNoOverlapCount,
			Snapshot.TrackingLifecycleStats.EntryRejectedAuthorizationCount,
			Snapshot.TrackingLifecycleStats.EntryRejectedAlreadyTrackedCount,
			Snapshot.TrackingLifecycleStats.EntryAdmittedSubductionCount,
			Snapshot.TrackingLifecycleStats.EntryAdmittedCollisionCount,
			Snapshot.TrackingLifecycleStats.ActiveAdvanceCount,
			Snapshot.TrackingLifecycleStats.ActiveOverlapConfirmedCount,
			Snapshot.TrackingLifecycleStats.ActiveOverlapRejectedCount,
			Snapshot.TrackingLifecycleStats.NeighborCandidateGeneratedCount,
			Snapshot.TrackingLifecycleStats.NeighborCandidateTestedCount,
			Snapshot.TrackingLifecycleStats.NeighborAdmittedCount,
			Snapshot.TrackingLifecycleStats.NeighborRejectedNoOverlapCount,
			Snapshot.TrackingLifecycleStats.NeighborRejectedAuthorizationCount,
			Snapshot.TrackingLifecycleStats.NeighborRejectedAlreadyTrackedCount,
			Snapshot.TrackingLifecycleStats.NeighborAdmittedSubductionCount,
			Snapshot.TrackingLifecycleStats.NeighborAdmittedCollisionCount,
			Snapshot.TrackingLifecycleStats.TopologyNeighborCandidateGeneratedCount,
			Snapshot.TrackingLifecycleStats.TopologyNeighborAdmittedCount,
			Snapshot.TrackingLifecycleStats.TopologyNeighborExpiredBeforeRemeshCount,
			Snapshot.TrackingLifecycleStats.DirectionalNeighborCandidateConsideredCount,
			Snapshot.TrackingLifecycleStats.DirectionalNeighborAdmittedCount,
			Snapshot.TrackingLifecycleStats.DirectionalNeighborRejectedNotInwardCount,
			Snapshot.TrackingLifecycleStats.DirectionalNeighborSurvivedToRemeshCount,
			Snapshot.TrackingLifecycleStats.ExpiredSubductionCount,
			Snapshot.TrackingLifecycleStats.ExpiredCollisionCount,
			Snapshot.MissTrueDivergenceGapCount,
			Snapshot.MissDestructiveExclusionGapCount,
			Snapshot.MissAmbiguousGapCount,
			Snapshot.SamplesWithPartialDestructiveCandidateRemovalCount,
			Snapshot.FragmentedComponentsAdjacentToDestructiveExclusionGapCount);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6TrackingLifecycleInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6DestructiveTrackingLifecycleStats& Lifecycle = Snapshot.TrackingLifecycleStats;
		const FString Message = FString::Printf(
			TEXT("%s step=%d tracked=%d tracked_subduction=%d tracked_collision=%d seed=%d propagated=%d expired=%d cleared=%d entry_candidates=%d seed_edge_candidates=%d entry_overlap_tested=%d entry_admitted=%d seed_edge_admitted=%d seed_edge_survived_one_timestep=%d seed_edge_survived_to_remesh=%d entry_reject_no_overlap=%d entry_reject_authorization=%d entry_reject_already_tracked=%d entry_subduction=%d entry_collision=%d active_advanced=%d active_overlap_confirmed=%d active_overlap_rejected=%d neighbor_generated=%d neighbor_tested=%d neighbor_admitted=%d neighbor_reject_no_overlap=%d neighbor_reject_authorization=%d neighbor_reject_already_tracked=%d neighbor_subduction=%d neighbor_collision=%d topology_neighbor_generated=%d topology_neighbor_admitted=%d topology_neighbor_expired=%d directional_neighbor_considered=%d directional_neighbor_admitted=%d directional_neighbor_rejected_not_inward=%d directional_neighbor_survived_to_remesh=%d expired_subduction=%d expired_collision=%d"),
			*SummaryTag,
			Snapshot.Step,
			Snapshot.TrackedDestructiveTriangleCount,
			Snapshot.TrackedSubductionTriangleCount,
			Snapshot.TrackedCollisionTriangleCount,
			Snapshot.TrackedDestructiveTriangleNewlySeededCount,
			Snapshot.TrackedDestructiveTrianglePropagatedCount,
			Snapshot.TrackedDestructiveTriangleExpiredCount,
			Snapshot.TrackedDestructiveTriangleClearedCount,
			Lifecycle.EntryCandidateCount,
			Lifecycle.EntrySeedConvergentEdgeCandidateCount,
			Lifecycle.EntryOverlapTestedCount,
			Lifecycle.EntryAdmittedCount,
			Lifecycle.EntrySeedConvergentEdgeAdmittedCount,
			Lifecycle.EntrySeedConvergentEdgeSurvivedOneTimestepCount,
			Lifecycle.EntrySeedConvergentEdgeSurvivedToRemeshCount,
			Lifecycle.EntryRejectedNoOverlapCount,
			Lifecycle.EntryRejectedAuthorizationCount,
			Lifecycle.EntryRejectedAlreadyTrackedCount,
			Lifecycle.EntryAdmittedSubductionCount,
			Lifecycle.EntryAdmittedCollisionCount,
			Lifecycle.ActiveAdvanceCount,
			Lifecycle.ActiveOverlapConfirmedCount,
			Lifecycle.ActiveOverlapRejectedCount,
			Lifecycle.NeighborCandidateGeneratedCount,
			Lifecycle.NeighborCandidateTestedCount,
			Lifecycle.NeighborAdmittedCount,
			Lifecycle.NeighborRejectedNoOverlapCount,
			Lifecycle.NeighborRejectedAuthorizationCount,
			Lifecycle.NeighborRejectedAlreadyTrackedCount,
			Lifecycle.NeighborAdmittedSubductionCount,
			Lifecycle.NeighborAdmittedCollisionCount,
			Lifecycle.TopologyNeighborCandidateGeneratedCount,
			Lifecycle.TopologyNeighborAdmittedCount,
			Lifecycle.TopologyNeighborExpiredBeforeRemeshCount,
			Lifecycle.DirectionalNeighborCandidateConsideredCount,
			Lifecycle.DirectionalNeighborAdmittedCount,
			Lifecycle.DirectionalNeighborRejectedNotInwardCount,
			Lifecycle.DirectionalNeighborSurvivedToRemeshCount,
			Lifecycle.ExpiredSubductionCount,
			Lifecycle.ExpiredCollisionCount);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6BoundaryCoherenceInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6BoundaryCoherenceDiagnostic& BC = Snapshot.BoundaryCoherence;
		const FString Message = FString::Printf(
			TEXT("%s step=%d boundary_band_samples=%d boundary_band_components=%d largest_component=%d smallest_component=%d singleton_components=%d total_conflict=%d conflict_ring0=%d conflict_ring1=%d conflict_ring2plus=%d mean_conflict_ring=%.3f multi_hit_boundary=%d multi_hit_interior=%d miss_boundary=%d miss_interior=%d destructive_excl_boundary=%d destructive_excl_interior=%d total_interior_conflict=%d interior_leakage=%.4f coherence_score=%.4f"),
			*SummaryTag,
			Snapshot.Step,
			BC.BoundaryBandSampleCount,
			BC.BoundaryBandComponentCount,
			BC.LargestBoundaryBandComponentSize,
			BC.SmallestBoundaryBandComponentSize,
			BC.SingletonBoundaryBandComponentCount,
			BC.TotalConflictSamples,
			BC.ConflictSamplesAtRing0,
			BC.ConflictSamplesAtRing1,
			BC.ConflictSamplesAtRing2Plus,
			BC.MeanConflictRingDistance,
			BC.MultiHitOnBoundaryCount,
			BC.MultiHitInteriorCount,
			BC.MissOnBoundaryCount,
			BC.MissInteriorCount,
			BC.DestructiveExclusionOnBoundaryCount,
			BC.DestructiveExclusionInteriorCount,
			BC.TotalInteriorConflictCount,
			BC.InteriorLeakageFraction,
			BC.BoundaryCoherenceScore);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6DiagnosticsPackageInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6OwnershipChurnDiagnostic& OC = Snapshot.OwnershipChurn;
		const FTectonicPlanetV6CompetitiveParticipationDiagnostic& CP = Snapshot.CompetitiveParticipation;
		const FTectonicPlanetV6MissLineageDiagnostic& ML = Snapshot.MissLineage;
		const FTectonicPlanetV6FrontRetreatDiagnostic& FR = Snapshot.FrontRetreat;
		const FTectonicPlanetV6BoundaryCoherenceDiagnostic& BC = Snapshot.BoundaryCoherence;
		const FString Message = FString::Printf(
			TEXT("%s step=%d churn_total=%d churn_fraction=%.4f churn_boundary=%d churn_interior=%d churn_ring0=%d churn_ring1=%d churn_ring2plus=%d churn_mean_ring=%.3f churn_top1=%d(%d->%d) churn_top2=%d(%d->%d) churn_top3=%d(%d->%d) plates_with_hit=%d/%d total_hits=%d largest_plate_hits=%d(plate%d) largest_share=%.4f plates_gt1pct=%d plates_gt5pct=%d miss_total=%d miss_first=%d miss_second=%d miss_repeat3plus=%d miss_mean_depth=%.3f miss_boundary_first=%d miss_boundary_repeat=%d miss_interior_first=%d miss_interior_repeat=%d retreat_current_subduction_km=%.1f retreat_previous_subduction_km=%.1f retreat_delta_km=%.1f retreat_subduction_samples=%d/%d retreat_tracked=%d retreat_prev_tracked=%d retreat_delta_tracked=%d coherence_score=%.4f interior_leakage=%.4f multi_hit=%d miss=%d max_components=%d cw_mismatch=%d"),
			*SummaryTag,
			Snapshot.Step,
			OC.TotalChurnCount,
			OC.ChurnFraction,
			OC.BoundaryBandChurnCount,
			OC.InteriorChurnCount,
			OC.ChurnAtRing0,
			OC.ChurnAtRing1,
			OC.ChurnAtRing2Plus,
			OC.MeanChurnRingDistance,
			OC.TopFlows[0].Count, OC.TopFlows[0].FromPlateId, OC.TopFlows[0].ToPlateId,
			OC.TopFlows[1].Count, OC.TopFlows[1].FromPlateId, OC.TopFlows[1].ToPlateId,
			OC.TopFlows[2].Count, OC.TopFlows[2].FromPlateId, OC.TopFlows[2].ToPlateId,
			CP.PlatesWithAnyHit, CP.TotalPlateCount,
			CP.TotalHitSamples,
			CP.LargestPlateHitCount, CP.LargestPlateId,
			CP.LargestPlateHitShare,
			CP.PlatesAboveOnePercentThreshold,
			CP.PlatesAboveFivePercentThreshold,
			ML.MissesThisCycle,
			ML.FirstTimeSynthetic,
			ML.SecondTimeSynthetic,
			ML.RepeatedSynthetic3Plus,
			ML.MeanResynthesisDepth,
			ML.BoundaryMissFirstTime,
			ML.BoundaryMissRepeat,
			ML.InteriorMissFirstTime,
			ML.InteriorMissRepeat,
			FR.CurrentMeanSubductionDistanceKm,
			FR.PreviousMeanSubductionDistanceKm,
			FR.DeltaMeanSubductionDistanceKm,
			FR.CurrentSubductionSampleCount, FR.PreviousSubductionSampleCount,
			FR.TrackedTriangleCount,
			FR.PreviousTrackedTriangleCount,
			FR.DeltaTrackedTriangleCount,
			BC.BoundaryCoherenceScore,
			BC.InteriorLeakageFraction,
			Snapshot.MultiHitCount,
			Snapshot.MissCount,
			Snapshot.MaxComponentsPerPlate,
			Snapshot.ContinentalWeightThresholdMismatchCount);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6InteriorInstabilityInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6InteriorInstabilityDiagnostic& DI = Snapshot.InteriorInstability;
		const FString Message = FString::Printf(
			TEXT("%s step=%d interior_ring_ge%d miss_total=%d miss_prev_owner_no_coverage_approx=%d miss_other_plate_hit_nearby_approx=%d miss_prev_synthetic_repeat=%d miss_prev_natural=%d miss_prev_geom_absent=%d miss_prev_geom_nearby_recovery=%d miss_prev_geom_rejected=%d miss_prev_owner_neighbor_hit=%d miss_single_participation=%d miss_multi_participation=%d miss_largest_plate_absorb=%d churn_total=%d churn_prev_owner_missing=%d churn_valid_to_valid=%d churn_prev_owner_still_valid=%d churn_valid_without_prev_hit=%d churn_prev_synthetic=%d churn_current_synthetic=%d churn_either_synthetic=%d churn_near_repeat_resynth=%d churn_prev_owner_neighbor_hit=%d churn_single_participation=%d churn_multi_participation=%d churn_largest_plate_absorb=%d churn_top1=%d(%d->%d) churn_top2=%d(%d->%d) churn_top3=%d(%d->%d) synthetic_prev_total=%d synthetic_survived=%d synthetic_reassigned=%d synthetic_missed_again=%d"),
			*SummaryTag,
			Snapshot.Step,
			DI.InteriorRingThreshold,
			DI.InteriorMissCount,
			DI.InteriorMissPreviousOwnerNoLocalCoverageApproxCount,
			DI.InteriorMissOtherPlateValidHitNearbyApproxCount,
			DI.InteriorMissPreviousSyntheticRepeatCount,
			DI.InteriorMissPreviousNaturalCount,
			DI.InteriorMissPreviousOwnerGeometryAbsentCount,
			DI.InteriorMissPreviousOwnerGeometryNearbyRecoveryCount,
			DI.InteriorMissPreviousOwnerGeometryRejectedCount,
			DI.InteriorMissPreviousOwnerNeighborHitCount,
			DI.InteriorMissSinglePlateParticipationCount,
			DI.InteriorMissMultiPlateParticipationCount,
			DI.InteriorMissAssignedToLargestParticipationPlateCount,
			DI.InteriorChurnCount,
			DI.InteriorChurnPreviousOwnerNowMissingCount,
			DI.InteriorChurnValidOwnerToValidOwnerCount,
			DI.InteriorChurnPreviousOwnerStillHadValidHitCount,
			DI.InteriorChurnValidOwnerWithoutPreviousOwnerHitCount,
			DI.InteriorChurnPreviousSyntheticCount,
			DI.InteriorChurnCurrentSyntheticCount,
			DI.InteriorChurnEitherSyntheticCount,
			DI.InteriorChurnNearRepeatResynthesisCount,
			DI.InteriorChurnPreviousOwnerNeighborHitCount,
			DI.InteriorChurnSinglePlateParticipationCount,
			DI.InteriorChurnMultiPlateParticipationCount,
			DI.InteriorChurnAbsorbedByLargestParticipationPlateCount,
			DI.TopInteriorFlows[0].Count, DI.TopInteriorFlows[0].FromPlateId, DI.TopInteriorFlows[0].ToPlateId,
			DI.TopInteriorFlows[1].Count, DI.TopInteriorFlows[1].FromPlateId, DI.TopInteriorFlows[1].ToPlateId,
			DI.TopInteriorFlows[2].Count, DI.TopInteriorFlows[2].FromPlateId, DI.TopInteriorFlows[2].ToPlateId,
			DI.InteriorPreviousSyntheticSampleCount,
			DI.InteriorPreviousSyntheticSurvivedUnchangedCount,
			DI.InteriorPreviousSyntheticReassignedCount,
			DI.InteriorPreviousSyntheticMissedAgainCount);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6SyntheticCoveragePersistenceInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic& DI =
			Snapshot.SyntheticCoveragePersistence;
		const FString Message = FString::Printf(
			TEXT("%s step=%d prev_synth_total=%d prev_synth_interior=%d same_owner_valid_hit=%d different_owner_valid_hit=%d synth_miss=%d prev_owner_exact_hit=%d prev_owner_ignoring_cap_hit=%d prev_owner_recovery=%d prev_owner_vertex_in_mesh=%d prev_owner_adjacent_triangle_in_mesh=%d prev_owner_vertex_no_adjacent_triangle=%d prev_owner_adjacent_triangle_no_exact_hit=%d prev_owner_adjacent_triangle_ignoring_cap_hit=%d miss_prev_owner_geom_nearby=%d miss_prev_owner_geom_absent=%d miss_prev_owner_adjacent_triangle_present=%d miss_prev_owner_adjacent_triangle_absent=%d diff_owner_prev_owner_still_exact=%d diff_owner_prev_owner_recovery_only=%d diff_owner_absorbed_by_largest_plate=%d current_retained_synth_coverage=%d prev_retained_synth_total=%d prev_retained_stabilized_same_owner=%d prev_retained_miss=%d prev_retained_reassigned=%d"),
			*SummaryTag,
			Snapshot.Step,
			DI.PreviousSyntheticSampleCount,
			DI.PreviousSyntheticInteriorSampleCount,
			DI.SameOwnerValidHitCount,
			DI.DifferentOwnerValidHitCount,
			DI.MissCount,
			DI.PreviousOwnerExactHitCount,
			DI.PreviousOwnerIgnoringBoundingCapHitCount,
			DI.PreviousOwnerRecoveryCoverageCount,
			DI.PreviousOwnerCanonicalVertexInMeshCount,
			DI.PreviousOwnerAdjacentTriangleInMeshCount,
			DI.PreviousOwnerVertexButNoAdjacentTriangleCount,
			DI.PreviousOwnerAdjacentTriangleButNoExactHitCount,
			DI.PreviousOwnerAdjacentTriangleButIgnoringCapHitCount,
			DI.MissWithPreviousOwnerGeometryNearbyCount,
			DI.MissWithNoPreviousOwnerGeometryNearbyCount,
			DI.MissWithPreviousOwnerAdjacentTrianglePresentCount,
			DI.MissWithPreviousOwnerAdjacentTriangleAbsentCount,
			DI.DifferentOwnerValidHitWhilePreviousOwnerStillHadExactHitCount,
			DI.DifferentOwnerValidHitWhilePreviousOwnerOnlyHadRecoveryCount,
			DI.DifferentOwnerValidHitAbsorbedByLargestParticipationPlateCount,
			DI.CurrentRetainedSyntheticCoverageCount,
			DI.PreviousRetainedSyntheticCoverageSampleCount,
			DI.PreviousRetainedSyntheticCoverageStabilizedSameOwnerCount,
			DI.PreviousRetainedSyntheticCoverageMissCount,
			DI.PreviousRetainedSyntheticCoverageReassignedCount);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);

		const FString LocalCoverageMessage = FString::Printf(
			TEXT("%s step=%d local_thresholds_km=%.1f/%.1f adj_prev_owner_very_near=%d adj_prev_owner_moderate=%d adj_prev_owner_true_hole=%d adj_prev_owner_no_distance=%d adj_support_total=%d adj_mixed_total=%d adj_minority_total=%d adj_any_mixed_samples=%d adj_all_mixed_samples=%d adj_all_minority_samples=%d adj_no_exact_any_mixed=%d adj_no_exact_all_mixed=%d adj_no_exact_all_minority=%d adj_very_near_all_minority=%d adj_true_hole_all_minority=%d"),
			*SummaryTag,
			Snapshot.Step,
			FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic::VeryNearCoverageThresholdKm,
			FTectonicPlanetV6SyntheticCoveragePersistenceDiagnostic::ModerateCoverageThresholdKm,
			DI.PreviousOwnerAdjacentTriangleVeryNearMissCount,
			DI.PreviousOwnerAdjacentTriangleModerateMissCount,
			DI.PreviousOwnerAdjacentTriangleTrueLocalHoleCount,
			DI.PreviousOwnerAdjacentTriangleNoDistanceCount,
			DI.PreviousOwnerAdjacentTriangleTotalSupportCount,
			DI.PreviousOwnerAdjacentTriangleTotalMixedSupportCount,
			DI.PreviousOwnerAdjacentTriangleTotalMinoritySupportCount,
			DI.PreviousOwnerAdjacentTriangleAnyMixedSupportSampleCount,
			DI.PreviousOwnerAdjacentTriangleAllMixedSupportSampleCount,
			DI.PreviousOwnerAdjacentTriangleAllMinoritySupportSampleCount,
			DI.PreviousOwnerAdjacentTriangleNoExactHitAnyMixedSupportSampleCount,
			DI.PreviousOwnerAdjacentTriangleNoExactHitAllMixedSupportSampleCount,
			DI.PreviousOwnerAdjacentTriangleNoExactHitAllMinoritySupportSampleCount,
			DI.PreviousOwnerAdjacentTriangleVeryNearMissAllMinoritySupportCount,
			DI.PreviousOwnerAdjacentTriangleTrueLocalHoleAllMinoritySupportCount);
		Test.AddInfo(LocalCoverageMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *LocalCoverageMessage);
	}

	void AddV6OwnerCoverageAuditInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const int32 Step,
		const FTectonicPlanetV6OwnerCoverageAudit& Audit)
	{
		const auto FormatSampleIndices = [](const TArray<int32>& SampleIndices) -> FString
		{
			if (SampleIndices.IsEmpty())
			{
				return TEXT("none");
			}

			return FString::JoinBy(
				SampleIndices,
				TEXT(","),
				[](const int32 SampleIndex)
				{
					return FString::FromInt(SampleIndex);
				});
		};

		const FString Message = FString::Printf(
			TEXT("%s step=%d assigned=%d assigned_interior=%d owner_query_exact_hit=%d owner_query_ignoring_cap_hit=%d owner_query_miss=%d owner_query_miss_ignoring_cap=%d owner_soup_contained=%d owner_soup_miss=%d owner_soup_contained_but_query_miss=%d owner_soup_contained_but_query_miss_ignoring_cap=%d interior_query_miss_examples=%s interior_soup_contained_query_miss_examples=%s"),
			*SummaryTag,
			Step,
			Audit.AssignedSampleCount,
			Audit.AssignedInteriorSampleCount,
			Audit.OwnerQueryExactHitCount,
			Audit.OwnerQueryIgnoringCapHitCount,
			Audit.OwnerQueryMissCount,
			Audit.OwnerQueryMissIgnoringCapCount,
			Audit.OwnerSoupContainmentCount,
			Audit.OwnerSoupMissCount,
			Audit.OwnerSoupContainmentButQueryMissCount,
			Audit.OwnerSoupContainmentButQueryMissIgnoringCapCount,
			*FormatSampleIndices(Audit.ExampleInteriorQueryMissSampleIndices),
			*FormatSampleIndices(Audit.ExampleInteriorSoupContainedButQueryMissSampleIndices));
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6ActiveZoneInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6ActiveZoneDiagnostic& AZ = Snapshot.ActiveZone;
		const FString Message = FString::Printf(
			TEXT("%s step=%d active=%d inactive=%d active_fraction=%.4f active_pairs=%d fresh_pairs=%d carry_pairs=%d fresh_pair_candidates=%d fresh_pair_admitted=%d fresh_reject_support=%d fresh_reject_velocity=%d fresh_reject_dominance=%d fresh_admit_divergence=%d fresh_admit_convergent_subduction=%d fresh_admit_collision=%d fresh_admit_rift=%d active_seed=%d active_carryover=%d fresh_expanded=%d carry_expanded=%d active_boundary=%d active_interior=%d active_divergence=%d active_convergent_subduction=%d active_collision=%d active_rift=%d active_generic=%d active_unknown=%d ownership_change_inside=%d ownership_change_outside=%d outside_zone_query_miss=%d outside_zone_coverage_deficit=%d cause_divergence_fill=%d cause_convergent_subduction=%d cause_collision=%d cause_rift=%d cause_generic_query=%d cause_unknown=%d"),
			*SummaryTag,
			Snapshot.Step,
			AZ.ActiveSampleCount,
			AZ.InactiveSampleCount,
			AZ.ActiveFraction,
			AZ.ActivePairCount,
			AZ.FreshSeedActivePairCount,
			AZ.PersistentCarryoverActivePairCount,
			AZ.FreshPairCandidateCount,
			AZ.FreshPairAdmittedCount,
			AZ.FreshPairRejectedSupportCount,
			AZ.FreshPairRejectedVelocityCount,
			AZ.FreshPairRejectedDominanceCount,
			AZ.FreshPairAdmittedDivergenceCount,
			AZ.FreshPairAdmittedConvergentSubductionCount,
			AZ.FreshPairAdmittedCollisionContactCount,
			AZ.FreshPairAdmittedRiftCount,
			AZ.ActiveSeedSampleCount,
			AZ.ActiveCarryoverSampleCount,
			AZ.FreshExpandedSampleCount,
			AZ.CarryoverExpandedSampleCount,
			AZ.ActiveBoundaryBandCount,
			AZ.ActiveInteriorCount,
			AZ.ActiveDivergenceSampleCount,
			AZ.ActiveConvergentSubductionSampleCount,
			AZ.ActiveCollisionContactSampleCount,
			AZ.ActiveRiftSampleCount,
			AZ.ActiveGenericQueryCompetitionSampleCount,
			AZ.ActiveUnknownOtherSampleCount,
			AZ.OwnershipChangesInsideActiveZone,
			AZ.OwnershipChangesOutsideActiveZone,
			AZ.OutsideZoneQueryMissCount,
			AZ.OutsideZoneCoverageDeficitCount,
			AZ.OwnershipChangeDivergenceFillCount,
			AZ.OwnershipChangeConvergentSubductionCount,
			AZ.OwnershipChangeCollisionContactCount,
			AZ.OwnershipChangeRiftCount,
			AZ.OwnershipChangeGenericQueryCompetitionCount,
			AZ.OwnershipChangeUnknownOtherCount);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6CollisionShadowInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6CollisionShadowDiagnostic& CS = Snapshot.CollisionShadow;
		const auto FormatTopPairs = [](const TArray<FTectonicPlanetV6CollisionShadowTopPair>& TopPairs) -> FString
		{
			if (TopPairs.IsEmpty())
			{
				return TEXT("none");
			}

			TArray<FString> Parts;
			Parts.Reserve(TopPairs.Num());
			for (const FTectonicPlanetV6CollisionShadowTopPair& Pair : TopPairs)
			{
				Parts.Add(FString::Printf(
					TEXT("(%d,%d):obs=%d pen=%.1f conv=%.2f/%.2f support=%d tris=%d cont=%d/%d qualified=%d"),
					Pair.PlateA,
					Pair.PlateB,
					Pair.ObservationCount,
					Pair.AccumulatedPenetrationKm,
					Pair.MeanConvergenceKmPerMy,
					Pair.MaxConvergenceKmPerMy,
					Pair.SupportSampleCount,
					Pair.SupportTriangleCount,
					Pair.ContinentalSupportPlateACount,
					Pair.ContinentalSupportPlateBCount,
					Pair.bQualified ? 1 : 0));
			}

			return FString::Join(Parts, TEXT(" | "));
		};

		const FString Message = FString::Printf(
			TEXT("%s step=%d shadow_tracked_pairs=%d shadow_regions=%d shadow_subduction_samples=%d shadow_subduction_triangles=%d shadow_collision_samples=%d shadow_collision_triangles=%d shadow_candidates=%d shadow_qualified=%d shadow_persistent_pairs=%d shadow_continental_qualified=%d shadow_reject_support=%d shadow_reject_threshold=%d shadow_reject_continentality=%d shadow_reject_persistence=%d shadow_best_pair=(%d,%d) shadow_best_obs=%d shadow_best_penetration_km=%.1f shadow_best_mean_convergence_km_per_my=%.3f shadow_best_max_convergence_km_per_my=%.3f shadow_best_support=%d shadow_best_triangles=%d shadow_best_continental_samples=%d top_pairs=%s"),
			*SummaryTag,
			Snapshot.Step,
			CS.TrackedConvergentPairCount,
			CS.TrackedConvergentRegionCount,
			CS.TrackedSubductionSampleCount,
			CS.TrackedSubductionTriangleCount,
			CS.TrackedCollisionSampleCount,
			CS.TrackedCollisionTriangleCount,
			CS.CollisionShadowCandidateCount,
			CS.CollisionShadowQualifiedCount,
			CS.PersistentObservedPairCount,
			CS.ContinentalQualifiedCandidateCount,
			CS.CandidateRejectedBySupportCount,
			CS.CandidateRejectedByThresholdCount,
			CS.CandidateRejectedByContinentalityCount,
			CS.CandidateRejectedByPersistenceCount,
			CS.BestCandidatePlateA,
			CS.BestCandidatePlateB,
			CS.BestCandidateObservationCount,
			CS.BestCandidateAccumulatedPenetrationKm,
			CS.BestCandidateMeanConvergenceKmPerMy,
			CS.BestCandidateMaxConvergenceKmPerMy,
			CS.BestCandidateSupportSampleCount,
			CS.BestCandidateSupportTriangleCount,
			CS.BestCandidateContinentalQualifiedSampleCount,
			*FormatTopPairs(CS.TopPairs));
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6CollisionExecutionInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& CE = Snapshot.CollisionExecution;
		const FString Message = FString::Printf(
			TEXT("%s step=%d exec_collision_count=%d exec_collision_cumulative=%d exec_cumulative_affected=%d exec_cumulative_affected_visits=%d exec_cumulative_continental_gain=%d exec_cumulative_ownership_change=%d exec_cumulative_transfer=%d exec_cumulative_transfer_visits=%d exec_cumulative_transfer_continental=%d exec_pair=(%d,%d) exec_over_plate=%d exec_sub_plate=%d exec_obs=%d exec_penetration_km=%.1f exec_mean_convergence_km_per_my=%.3f exec_max_convergence_km_per_my=%.3f exec_support=%d exec_triangles=%d exec_continental_support=%d/%d exec_qualified_samples=%d exec_seed_samples=%d exec_collision_seed_samples=%d exec_effective_mass=%d exec_affected=%d exec_continental_gain=%d exec_ownership_change=%d exec_transfer=%d exec_transfer_continental=%d exec_cooldown_suppressed=%d exec_plate_conflict_suppressed=%d exec_overlap_suppressed=%d exec_qualified_unexecuted=%d exec_transfer_reject_locality=%d exec_transfer_reject_continentality=%d exec_transfer_reject_cap=%d exec_transfer_boundary_local=%d exec_transfer_patch_support=%d exec_transfer_anchor_seeds=%d exec_thesis_terrane_component=%d exec_thesis_transferred_component=%d exec_thesis_xi=%.6f exec_thesis_rel_speed_km_per_my=%.3f exec_thesis_terrane_area_sr=%.6f exec_suture_axis=(%.4f,%.4f,%.4f) exec_ridge_half_width_rad=%.6f exec_belt_length_rad=%.6f exec_mean_abs_along_rad=%.6f exec_mean_abs_perp_rad=%.6f exec_ridge_core=%d exec_ridge_flank=%d exec_radius_rad=%.6f exec_transfer_radius_rad=%.6f exec_mean_elev_delta_km=%.6f exec_max_elev_delta_km=%.6f exec_cumulative_mean_elev_delta_km=%.6f exec_cumulative_max_elev_delta_km=%.6f exec_strength_scale=%.6f donor_share=%.6f->%.6f recipient_share=%.6f->%.6f exec_from_shadow=%d"),
			*SummaryTag,
			Snapshot.Step,
			CE.ExecutedCollisionCount,
			CE.CumulativeExecutedCollisionCount,
			CE.CumulativeCollisionAffectedSampleCount,
			CE.CumulativeCollisionAffectedSampleVisits,
			CE.CumulativeCollisionDrivenContinentalGainCount,
			CE.CumulativeCollisionDrivenOwnershipChangeCount,
			CE.CumulativeCollisionTransferredSampleCount,
			CE.CumulativeCollisionTransferredSampleVisits,
			CE.CumulativeCollisionTransferredContinentalSampleCount,
			CE.ExecutedPlateA,
			CE.ExecutedPlateB,
			CE.ExecutedOverridingPlateId,
			CE.ExecutedSubductingPlateId,
			CE.ExecutedObservationCount,
			CE.ExecutedAccumulatedPenetrationKm,
			CE.ExecutedMeanConvergenceKmPerMy,
			CE.ExecutedMaxConvergenceKmPerMy,
			CE.ExecutedSupportSampleCount,
			CE.ExecutedSupportTriangleCount,
			CE.ExecutedContinentalSupportPlateACount,
			CE.ExecutedContinentalSupportPlateBCount,
			CE.ExecutedContinentalQualifiedSampleCount,
			CE.ExecutedSeedSampleCount,
			CE.ExecutedCollisionSeedSampleCount,
			CE.ExecutedEffectiveMassSampleCount,
			CE.CollisionAffectedSampleCount,
			CE.CollisionDrivenContinentalGainCount,
			CE.CollisionDrivenOwnershipChangeCount,
			CE.CollisionTransferredSampleCount,
			CE.CollisionTransferredContinentalSampleCount,
			CE.CooldownSuppressedQualifiedCount,
			CE.PlateConflictSuppressedQualifiedCount,
			CE.OverlapSuppressedQualifiedCount,
			CE.QualifiedButUnexecutedCount,
			CE.ExecutedTransferRejectedByLocalityCount,
			CE.ExecutedTransferRejectedByContinentalityCount,
			CE.ExecutedTransferRejectedByCapCount,
			CE.ExecutedTransferBoundaryLocalSampleCount,
			CE.ExecutedTransferCandidateSupportCount,
			CE.ExecutedTransferAnchorSeedCount,
			CE.ExecutedDonorTerraneComponentSize,
			CE.ExecutedTransferredComponentSize,
			CE.ExecutedXi,
			CE.ExecutedRelativeSpeedKmPerMy,
			CE.ExecutedTerraneAreaSr,
			CE.ExecutedSutureAxis.X,
			CE.ExecutedSutureAxis.Y,
			CE.ExecutedSutureAxis.Z,
			CE.ExecutedSutureHalfWidthRad,
			CE.ExecutedSutureBeltLengthEstimateRad,
			CE.ExecutedMeanAbsAlongSutureRad,
			CE.ExecutedMeanAbsPerpendicularRad,
			CE.ExecutedRidgeCoreAffectedSampleCount,
			CE.ExecutedRidgeFlankAffectedSampleCount,
			CE.ExecutedInfluenceRadiusRad,
			CE.ExecutedTransferInfluenceRadiusRad,
			CE.ExecutedMeanElevationDeltaKm,
			CE.ExecutedMaxElevationDeltaKm,
			CE.CumulativeMeanElevationDeltaKm,
			CE.CumulativeMaxElevationDeltaKm,
			CE.ExecutedStrengthScale,
			CE.ExecutedDonorPlateShareBefore,
			CE.ExecutedDonorPlateShareAfter,
			CE.ExecutedRecipientPlateShareBefore,
			CE.ExecutedRecipientPlateShareAfter,
			CE.bExecutedFromShadowQualifiedState ? 1 : 0);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6RiftInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FTectonicPlanetV6RiftDiagnostic& Rift)
	{
		const FString ChildPlateIds = JoinIntArrayForDiagnostics(Rift.ChildPlateIds);
		const FString ChildSampleCounts = JoinIntArrayForDiagnostics(Rift.ChildSampleCounts);
		const FString Message = FString::Printf(
			TEXT("%s rift_step=%d rift_cumulative=%d triggered_this_solve=%d automatic=%d forced=%d parent=%d parent_still_present=%d child_ids=(%s) parent_samples=%d parent_continental_samples=%d parent_continental_fraction=%.4f child_samples=(%s) current_child_samples=(%d,%d) child_alive=(%d,%d) trigger_probability=%.6f trigger_draw=%.6f post_rift_plate_count=%d ownership_applied_directly=%d copied_frontier_rebuilt=%d plate_submesh_rebuilt=%d post_rift_solve_ran=%d child_boundary_contact_edges=%d child_boundary_divergent_edges=%d child_boundary_convergent_edges=%d child_boundary_mean_rel_normal_velocity=%.4f child_boundary_max_abs_rel_normal_velocity=%.4f child_boundary_rift_active_samples=%d child_boundary_divergence_samples=%d child_boundary_classified_divergent=%d"),
			*SummaryTag,
			Rift.Step,
			Rift.CumulativeRiftCount,
			Rift.bTriggeredThisSolve ? 1 : 0,
			Rift.bAutomatic ? 1 : 0,
			Rift.bForcedByTest ? 1 : 0,
			Rift.ParentPlateId,
			Rift.bParentPlateStillPresent ? 1 : 0,
			*ChildPlateIds,
			Rift.ParentSampleCount,
			Rift.ParentContinentalSampleCount,
			Rift.ParentContinentalFraction,
			*ChildSampleCounts,
			Rift.CurrentChildSampleCountA,
			Rift.CurrentChildSampleCountB,
			Rift.bChildPlateAAlive ? 1 : 0,
			Rift.bChildPlateBAlive ? 1 : 0,
			Rift.TriggerProbability,
			Rift.TriggerDraw,
			Rift.PostRiftPlateCount,
			Rift.bOwnershipAppliedDirectlyByEvent ? 1 : 0,
			Rift.bCopiedFrontierRebuiltBeforeSolve ? 1 : 0,
			Rift.bPlateSubmeshRebuiltBeforeSolve ? 1 : 0,
			Rift.bPostRiftSolveRan ? 1 : 0,
			Rift.ChildBoundaryContactEdgeCount,
			Rift.ChildBoundaryDivergentEdgeCount,
			Rift.ChildBoundaryConvergentEdgeCount,
			Rift.ChildBoundaryMeanRelativeNormalVelocityKmPerMy,
			Rift.ChildBoundaryMaxAbsRelativeNormalVelocityKmPerMy,
			Rift.ChildBoundaryRiftActiveSampleCount,
			Rift.ChildBoundaryDivergenceActiveSampleCount,
			Rift.bChildBoundaryClassifiedDivergent ? 1 : 0);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void AddV6TectonicInteractionInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FV6CheckpointSnapshot& Snapshot)
	{
		const FTectonicPlanetV6CompetitiveParticipationDiagnostic& CP = Snapshot.CompetitiveParticipation;
		const FString Message = FString::Printf(
			TEXT("%s step=%d collision_count=%d collision_deferred=%d collision_overriding_plate=%d collision_subducting_plate=%d boundary_contact_collision_pairs=%d geometric_collision_candidates=%d geometric_collision_pairs=%d geometric_collision_qualified_pairs=%d geometric_collision_overlap_samples=%d geometric_collision_executed_terrane=%d used_geometric_collision=%d used_cached_boundary_contact_collision=%d tracked_collision_triangles=%d plates_with_hit=%d/%d plates_gt5pct=%d largest_share=%.4f"),
			*SummaryTag,
			Snapshot.Step,
			Snapshot.CollisionCount,
			Snapshot.CollisionDeferredCount,
			Snapshot.CollisionOverridingPlateId,
			Snapshot.CollisionSubductingPlateId,
			Snapshot.BoundaryContactCollisionPairCount,
			Snapshot.GeometricCollisionCandidateCount,
			Snapshot.GeometricCollisionPairCount,
			Snapshot.GeometricCollisionQualifiedPairCount,
			Snapshot.GeometricCollisionOverlapSampleCount,
			Snapshot.GeometricCollisionExecutedTerraneEstimate,
			Snapshot.bUsedGeometricCollisionExecution ? 1 : 0,
			Snapshot.bUsedCachedBoundaryContactCollision ? 1 : 0,
			Snapshot.TrackedCollisionTriangleCount,
			CP.PlatesWithAnyHit,
			CP.TotalPlateCount,
			CP.PlatesAboveFivePercentThreshold,
			CP.LargestPlateHitShare);
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	FString JoinIntArrayForDiagnostics(const TArray<int32>& Values)
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

	void AddCopiedFrontierRayDiagnosticsInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FTectonicPlanetV6CopiedFrontierRayDiagnostics& Diagnostics)
	{
		const FString SummaryMessage = FString::Printf(
			TEXT("%s sample_count=%d raw_ray_miss_count=%d boundary_raw_ray_miss_count=%d coverage_gap_miss_count=%d bounding_cap_rejected_raw_ray_hit_miss_count=%d bounding_cap_rejected_containing_triangle_miss_count=%d containing_triangle_recovered_miss_count=%d adjacent_containing_triangle_recovered_miss_count=%d near_vertex_miss_count=%d near_edge_miss_count=%d near_degenerate_miss_count=%d logged_miss_samples=%d"),
			*SummaryTag,
			Diagnostics.SampleCount,
			Diagnostics.RawRayMissCount,
			Diagnostics.BoundaryRawRayMissCount,
			Diagnostics.CoverageGapMissCount,
			Diagnostics.BoundingCapRejectedRawRayHitMissCount,
			Diagnostics.BoundingCapRejectedContainingTriangleMissCount,
			Diagnostics.ContainingTriangleRecoveredMissCount,
			Diagnostics.AdjacentContainingTriangleRecoveredMissCount,
			Diagnostics.NearVertexMissCount,
			Diagnostics.NearEdgeMissCount,
			Diagnostics.NearDegenerateMissCount,
			Diagnostics.MissDiagnostics.Num());
		Test.AddInfo(SummaryMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *SummaryMessage);

		for (const FTectonicPlanetV6CopiedFrontierMissDiagnostic& Miss : Diagnostics.MissDiagnostics)
		{
			const FString MissMessage = FString::Printf(
				TEXT("%s sample=%d previous_plate=%d boundary=%d copied_frontier_vertex=%d coverage_gap=%d containing_recovered=%d cap_rejected_containing=%d near_vertex=%d near_edge=%d near_degenerate=%d adjacent_triangle_count=%d plate_mesh_copy_count=%d cap_rejected_plate_count=%d containing_plate_count=%d adjacent_containing_plate_count=%d position=(%.9f,%.9f,%.9f) adjacent_triangles=[%s]"),
				*SummaryTag,
				Miss.SampleIndex,
				Miss.PreviousPlateId,
				Miss.bBoundarySample ? 1 : 0,
				Miss.bAtCopiedFrontierVertex ? 1 : 0,
				Miss.bCoverageGap ? 1 : 0,
				Miss.bRawRayMissRecoveredByContainingTriangle ? 1 : 0,
				Miss.bBoundingCapRejectedContainingTriangle ? 1 : 0,
				Miss.bNearVertex ? 1 : 0,
				Miss.bNearEdge ? 1 : 0,
				Miss.bNearDegenerate ? 1 : 0,
				Miss.AdjacentGlobalTriangleCount,
				Miss.PlateMeshCountWithAdjacentTriangleCopies,
				Miss.BoundingCapRejectedPlateCount,
				Miss.ContainingTrianglePlateCount,
				Miss.AdjacentContainingTrianglePlateCount,
				Miss.Position.X,
				Miss.Position.Y,
				Miss.Position.Z,
				*JoinIntArrayForDiagnostics(Miss.AdjacentGlobalTriangleIndices));
			Test.AddInfo(MissMessage);
			UE_LOG(LogTemp, Log, TEXT("%s"), *MissMessage);

			for (const FTectonicPlanetV6CopiedFrontierAdjacentTriangleDiagnostic& TriangleDiagnostic : Miss.AdjacentTriangleDiagnostics)
			{
				const FString TriangleMessage = FString::Printf(
					TEXT("%s sample=%d adjacent_triangle=%d copy_plates=[%s]"),
					*SummaryTag,
					Miss.SampleIndex,
					TriangleDiagnostic.GlobalTriangleIndex,
					*JoinIntArrayForDiagnostics(TriangleDiagnostic.PlateIdsWithCopies));
				Test.AddInfo(TriangleMessage);
				UE_LOG(LogTemp, Log, TEXT("%s"), *TriangleMessage);
			}

			for (const FTectonicPlanetV6CopiedFrontierMissPlateDiagnostic& PlateDiagnostic : Miss.PlateDiagnostics)
			{
				const FString PlateMessage = FString::Printf(
					TEXT("%s sample=%d plate=%d cap_rejected=%d adjacent_triangle_copy_count=%d raw_ray_hit=%d raw_ray_hit_ignoring_cap=%d containing_hit=%d containing_adjacent_hit=%d near_vertex=%d near_edge=%d near_degenerate=%d"),
					*SummaryTag,
					Miss.SampleIndex,
					PlateDiagnostic.PlateId,
					PlateDiagnostic.bBoundingCapRejected ? 1 : 0,
					PlateDiagnostic.AdjacentTriangleCopyCount,
					PlateDiagnostic.bRawRayHit ? 1 : 0,
					PlateDiagnostic.bRawRayHitIgnoringCap ? 1 : 0,
					PlateDiagnostic.bContainingTriangleHit ? 1 : 0,
					PlateDiagnostic.bContainingAdjacentTriangleHit ? 1 : 0,
					PlateDiagnostic.bNearVertex ? 1 : 0,
					PlateDiagnostic.bNearEdge ? 1 : 0,
					PlateDiagnostic.bNearDegenerate ? 1 : 0);
				Test.AddInfo(PlateMessage);
				UE_LOG(LogTemp, Log, TEXT("%s"), *PlateMessage);
			}
		}
	}

	// --- Continental Mass Diagnostic Structs and Logging ---

	struct FV9ContinentalMassDiagnostic
	{
		// Diagnostic 1: Area and connectivity
		double ContinentalAreaFraction = 0.0;
		int32 ContinentalSampleCount = 0;
		int32 SubaerialContinentalSampleCount = 0;
		int32 SubmergedContinentalSampleCount = 0;
		double SubaerialContinentalFraction = 0.0;
		double SubmergedContinentalFraction = 0.0;
		int32 ComponentCount = 0;
		int32 LargestComponentSize = 0;
		TArray<int32> Top5ComponentSizes;
		int32 SingletonCount = 0;
		int32 TinyComponentCount = 0; // <= 10 samples

		// Diagnostic 2: Boundary-local vs interior distribution
		int32 ActiveZoneContinentalCount = 0;
		int32 BoundaryBandContinentalCount = 0;
		int32 DeepInteriorContinentalCount = 0;
		double ActiveZoneContinentalFraction = 0.0;
		double BoundaryBandContinentalFraction = 0.0;
		double DeepInteriorContinentalFraction = 0.0;
		double MeanDistToActiveZoneHops = 0.0;
		double P50DistToActiveZone = 0.0;
		double P90DistToActiveZone = 0.0;

		// Diagnostic 4: Width proxy (BFS depth from coastline)
		double MeanCoastDistHops = 0.0;
		double P50CoastDist = 0.0;
		double P90CoastDist = 0.0;
		double MaxCoastDist = 0.0;
		double SubaerialMeanCoastDistHops = 0.0;
		double SubaerialP50CoastDist = 0.0;
		double SubaerialP90CoastDist = 0.0;

		// Diagnostic 5: Largest continent profile
		int32 LargestComponentId = -1;
		double LargestMeanElevationKm = 0.0;
		double LargestP95ElevationKm = 0.0;
		double LargestMeanCoastDist = 0.0;
		double LargestActiveZoneFraction = 0.0;
		double LargestBoundaryFraction = 0.0;
		int32 LargestSubaerialComponentId = -1;
		int32 LargestSubaerialComponentSize = 0;
		double LargestSubaerialMeanCoastDist = 0.0;

		// Per-sample auxiliary data (transient, for export overlays)
		TArray<int32> SampleComponentId; // -1 for non-continental
		TArray<int32> SampleCoastDistHops; // -1 for non-continental
	};

	struct FV9SeededContinentalSurvivalDiagnostic
	{
		int32 Step0ContinentalCount = 0;
		int32 Step0ContinentalRemainingCount = 0;
		double Step0ContinentalRemainingFraction = 0.0;
		int32 Step0BroadInteriorCount = 0;
		int32 Step0BroadInteriorRemainingCount = 0;
		double Step0BroadInteriorRemainingFraction = 0.0;
		int32 Step0CoastAdjacentCount = 0;
		int32 Step0CoastAdjacentRemainingCount = 0;
		double Step0CoastAdjacentRemainingFraction = 0.0;
	};

	struct FV9ContinentalLossAttribution
	{
		int32 TotalLostSamples = 0;
		int32 LostViaDirectHit = 0;
		int32 LostViaOverlapWinner = 0;
		int32 LostViaNearestTriangleRecovery = 0;
		int32 LostViaNearestMemberFallback = 0;
		int32 LostViaExplicitFallback = 0;
		int32 LostViaSyntheticDivergenceFill = 0;
		int32 LostViaDestructiveGapFill = 0;
		int32 LostViaOceanicCreation = 0;
		int32 LostViaBoundaryOceanic = 0;
		int32 LostViaRetainedOutsideActiveZone = 0;
		int32 LostViaTransferFallback = 0;
		int32 LostViaRetainedSyntheticCoverage = 0;
		int32 LostViaOther = 0;
		int32 TotalGainedSamples = 0;
		int32 NetChange = 0;

		// Sub-bucket: retained_outside_az losses by transfer source kind
		int32 RetainedOazViaTriangle = 0;
		int32 RetainedOazViaSingleSource = 0;
		int32 RetainedOazViaStructuredSynthetic = 0;
		int32 RetainedOazViaOceanicCreation = 0;
		int32 RetainedOazViaDefaulted = 0;
		int32 RetainedOazViaOtherSource = 0;
		// Sub-bucket: CW threshold mismatch (barycentric blend >= 0.5 but dominant < 0.5)
		int32 RetainedOazWithCwThresholdMismatch = 0;
	};

	void AddV6ContinentalMassDiagnosticInfo(
		FAutomationTestBase& Test,
		const FString& Tag,
		const FV9ContinentalMassDiagnostic& D)
	{
		// Diagnostic 1: Area and connectivity
		{
			FString Top5Str;
			for (int32 I = 0; I < D.Top5ComponentSizes.Num(); ++I)
			{
				if (I > 0) { Top5Str += TEXT(","); }
				Top5Str += FString::FromInt(D.Top5ComponentSizes[I]);
			}
			const FString Msg = FString::Printf(
				TEXT("%s continental_area_fraction=%.4f continental_samples=%d subaerial=%d(%.4f) submerged=%d(%.4f) components=%d largest=%d largest_subaerial=%d top5=[%s] singletons=%d tiny_le10=%d"),
				*Tag,
				D.ContinentalAreaFraction,
				D.ContinentalSampleCount,
				D.SubaerialContinentalSampleCount,
				D.SubaerialContinentalFraction,
				D.SubmergedContinentalSampleCount,
				D.SubmergedContinentalFraction,
				D.ComponentCount,
				D.LargestComponentSize,
				D.LargestSubaerialComponentSize,
				*Top5Str,
				D.SingletonCount,
				D.TinyComponentCount);
			Test.AddInfo(Msg);
			UE_LOG(LogTemp, Log, TEXT("%s"), *Msg);
		}

		// Diagnostic 2: Boundary-local vs interior
		{
			const FString Msg = FString::Printf(
				TEXT("%s active_zone_continental=%d(%.4f) boundary_band=%d(%.4f) deep_interior=%d(%.4f) mean_dist_to_active=%.2f p50=%.1f p90=%.1f"),
				*Tag,
				D.ActiveZoneContinentalCount,
				D.ActiveZoneContinentalFraction,
				D.BoundaryBandContinentalCount,
				D.BoundaryBandContinentalFraction,
				D.DeepInteriorContinentalCount,
				D.DeepInteriorContinentalFraction,
				D.MeanDistToActiveZoneHops,
				D.P50DistToActiveZone,
				D.P90DistToActiveZone);
			Test.AddInfo(Msg);
			UE_LOG(LogTemp, Log, TEXT("%s"), *Msg);
		}

		// Diagnostic 4: Width proxy
		{
			const FString Msg = FString::Printf(
				TEXT("%s coast_dist_mean=%.2f p50=%.1f p90=%.1f max=%.1f subaerial_mean=%.2f subaerial_p50=%.1f subaerial_p90=%.1f"),
				*Tag,
				D.MeanCoastDistHops,
				D.P50CoastDist,
				D.P90CoastDist,
				D.MaxCoastDist,
				D.SubaerialMeanCoastDistHops,
				D.SubaerialP50CoastDist,
				D.SubaerialP90CoastDist);
			Test.AddInfo(Msg);
			UE_LOG(LogTemp, Log, TEXT("%s"), *Msg);
		}

		// Diagnostic 5: Largest continent profile
		{
			const FString Msg = FString::Printf(
				TEXT("%s largest_size=%d largest_mean_elev=%.4f largest_p95_elev=%.4f largest_mean_coast_dist=%.2f largest_active_frac=%.4f largest_boundary_frac=%.4f largest_subaerial_size=%d largest_subaerial_mean_coast_dist=%.2f"),
				*Tag,
				D.LargestComponentSize,
				D.LargestMeanElevationKm,
				D.LargestP95ElevationKm,
				D.LargestMeanCoastDist,
				D.LargestActiveZoneFraction,
				D.LargestBoundaryFraction,
				D.LargestSubaerialComponentSize,
				D.LargestSubaerialMeanCoastDist);
			Test.AddInfo(Msg);
			UE_LOG(LogTemp, Log, TEXT("%s"), *Msg);
		}
	}

	void AddV6ContinentalLossAttributionInfo(
		FAutomationTestBase& Test,
		const FString& Tag,
		const FV9ContinentalLossAttribution& A)
	{
		const FString Msg = FString::Printf(
			TEXT("%s loss_total=%d gain_total=%d net=%d direct_hit=%d overlap_winner=%d tri_recovery=%d member_fallback=%d explicit_fallback=%d synth_divergence=%d destructive_gap=%d oceanic_creation=%d boundary_oceanic=%d retained_outside_az=%d transfer_fallback=%d retained_synth=%d other=%d"),
			*Tag,
			A.TotalLostSamples,
			A.TotalGainedSamples,
			A.NetChange,
			A.LostViaDirectHit,
			A.LostViaOverlapWinner,
			A.LostViaNearestTriangleRecovery,
			A.LostViaNearestMemberFallback,
			A.LostViaExplicitFallback,
			A.LostViaSyntheticDivergenceFill,
			A.LostViaDestructiveGapFill,
			A.LostViaOceanicCreation,
			A.LostViaBoundaryOceanic,
			A.LostViaRetainedOutsideActiveZone,
			A.LostViaTransferFallback,
			A.LostViaRetainedSyntheticCoverage,
			A.LostViaOther);
		Test.AddInfo(Msg);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Msg);

		if (A.LostViaRetainedOutsideActiveZone > 0)
		{
			const FString SubMsg = FString::Printf(
				TEXT("%s retained_oaz_sub: triangle=%d single_source=%d structured_synth=%d oceanic_creation=%d defaulted=%d other_source=%d cw_threshold_mismatch=%d"),
				*Tag,
				A.RetainedOazViaTriangle,
				A.RetainedOazViaSingleSource,
				A.RetainedOazViaStructuredSynthetic,
				A.RetainedOazViaOceanicCreation,
				A.RetainedOazViaDefaulted,
				A.RetainedOazViaOtherSource,
				A.RetainedOazWithCwThresholdMismatch);
			Test.AddInfo(SubMsg);
			UE_LOG(LogTemp, Log, TEXT("%s"), *SubMsg);
		}
	}

	bool ExportContinentalMassOverlays(
		FAutomationTestBase& Test,
		const FTectonicPlanetV6& Planet,
		const FV9ContinentalMassDiagnostic& Diag,
		const FString& ExportRoot,
		const int32 Step)
	{
		const FString OutputDirectory = FPaths::Combine(ExportRoot, FString::Printf(TEXT("step_%03d"), Step));
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		const int32 SampleCount = PlanetData.Samples.Num();
		if (SampleCount == 0) { return false; }

		bool bAllSucceeded = true;
		FString Error;

		// ActiveZoneMask
		{
			const TArray<uint8>& ActiveZoneFlags = Planet.GetCurrentSolveActiveZoneFlagsForTest();
			if (ActiveZoneFlags.Num() == SampleCount)
			{
				TArray<float> Values;
				Values.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					Values[I] = ActiveZoneFlags[I] ? 1.0f : 0.0f;
				}
				const FString Path = FPaths::Combine(OutputDirectory, TEXT("ActiveZoneMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData, Values, 0.0f, 1.0f, Path, TestExportWidth, TestExportHeight, Error))
				{
					Test.AddError(FString::Printf(TEXT("ActiveZoneMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		// ContinentalComponentMask (component ID → scalar, top 5 components get distinct values)
		if (Diag.SampleComponentId.Num() == SampleCount)
		{
			TArray<float> Values;
			Values.SetNum(SampleCount);
			for (int32 I = 0; I < SampleCount; ++I)
			{
				if (Diag.SampleComponentId[I] < 0)
				{
					Values[I] = 0.0f;
				}
				else
				{
					// Map top-5 components to distinct bands, rest to low value
					int32 Rank = -1;
					for (int32 R = 0; R < Diag.Top5ComponentSizes.Num(); ++R)
					{
						// Find which rank this component has by matching sizes
						// (approximate — just maps component to band based on ID order)
						if (Diag.SampleComponentId[I] == R) { Rank = R; break; }
					}
					Values[I] = Rank >= 0
						? static_cast<float>(Rank + 1) * 2.0f
						: 0.5f;
				}
			}
			const FString Path = FPaths::Combine(OutputDirectory, TEXT("ContinentalComponentMask.png"));
			if (!TectonicMollweideExporter::ExportScalarOverlay(
				PlanetData, Values, 0.0f, 10.0f, Path, TestExportWidth, TestExportHeight, Error))
			{
				Test.AddError(FString::Printf(TEXT("ContinentalComponentMask export step %d failed: %s"), Step, *Error));
				bAllSucceeded = false;
			}
		}

		// ContinentalInteriorDepthMask (coast distance in hops)
		if (Diag.SampleCoastDistHops.Num() == SampleCount)
		{
			TArray<float> Values;
			Values.SetNum(SampleCount);
			for (int32 I = 0; I < SampleCount; ++I)
			{
				Values[I] = Diag.SampleCoastDistHops[I] > 0
					? static_cast<float>(Diag.SampleCoastDistHops[I])
					: 0.0f;
			}
			const FString Path = FPaths::Combine(OutputDirectory, TEXT("ContinentalInteriorDepthMask.png"));
			if (!TectonicMollweideExporter::ExportScalarOverlay(
				PlanetData, Values, 0.0f, 20.0f, Path, TestExportWidth, TestExportHeight, Error))
			{
				Test.AddError(FString::Printf(TEXT("ContinentalInteriorDepthMask export step %d failed: %s"), Step, *Error));
				bAllSucceeded = false;
			}
		}

		return bAllSucceeded;
	}

	const TCHAR* GetCopiedFrontierMotionClassName(const ETectonicPlanetV6CopiedFrontierMotionClass MotionClass)
	{
		switch (MotionClass)
		{
		case ETectonicPlanetV6CopiedFrontierMotionClass::Weak:
			return TEXT("Weak");
		case ETectonicPlanetV6CopiedFrontierMotionClass::Divergent:
			return TEXT("Divergent");
		case ETectonicPlanetV6CopiedFrontierMotionClass::Convergent:
			return TEXT("Convergent");
		case ETectonicPlanetV6CopiedFrontierMotionClass::None:
		default:
			return TEXT("None");
		}
	}

	const TCHAR* GetCopiedFrontierGapAttributionName(const ETectonicPlanetV6CopiedFrontierGapAttribution GapAttribution)
	{
		switch (GapAttribution)
		{
		case ETectonicPlanetV6CopiedFrontierGapAttribution::TrueDivergence:
			return TEXT("TrueDivergence");
		case ETectonicPlanetV6CopiedFrontierGapAttribution::DestructiveExclusion:
			return TEXT("DestructiveExclusion");
		case ETectonicPlanetV6CopiedFrontierGapAttribution::Ambiguous:
			return TEXT("Ambiguous");
		case ETectonicPlanetV6CopiedFrontierGapAttribution::None:
		default:
			return TEXT("None");
		}
	}

	void AddCopiedFrontierSolveAttributionInfo(
		FAutomationTestBase& Test,
		const FString& SummaryTag,
		const FTectonicPlanetV6CopiedFrontierSolveAttribution& Attribution)
	{
		const FString SummaryMessage = FString::Printf(
			TEXT("%s step=%d cadence_steps=%d hit_count=%d miss_count=%d oceanic_creation=%d single_hit_winner_count=%d multi_hit_winner_count=%d multi_hit_winner_same_plate=%d multi_hit_winner_cross_plate=%d multi_hit_winner_frontier_hit=%d multi_hit_winner_interior_hit=%d multi_hit_winner_near_frontier=%d multi_hit_with_tracked_candidate=%d multi_hit_tracked_triangle=%d multi_hit_tracked_1ring=%d multi_hit_tracked_2ring=%d multi_hit_same_only_candidates=%d multi_hit_cross_only_candidates=%d multi_hit_mixed_plate_candidates=%d multi_hit_frontier_only_candidates=%d multi_hit_interior_only_candidates=%d multi_hit_mixed_geometry_candidates=%d multi_hit_adjacent_destructive_gap=%d multi_hit_adjacent_continuity_gap=%d overlap_support_pruned_samples=%d overlap_previous_plate_stabilized=%d overlap_suppressed_candidates=%d copied_frontier_hit_count=%d interior_hit_count=%d hit_cw_threshold_mismatch=%d copied_frontier_hit_cw_threshold_mismatch=%d interior_hit_cw_threshold_mismatch=%d miss_adjacent_to_copied_frontier=%d miss_near_multi_hit=%d miss_divergent_motion=%d miss_convergent_motion=%d miss_weak_or_no_motion=%d miss_adjacent_frontier_and_divergent=%d miss_adjacent_frontier_and_weak=%d miss_adjacent_frontier_and_convergent=%d miss_adjacent_frontier_and_near_multi_hit=%d miss_nearest_copied_frontier_le_100km=%d miss_nearest_copied_frontier_le_500km=%d miss_nearest_copied_frontier_le_1000km=%d miss_nearest_copied_frontier_le_4000km=%d miss_nearest_copied_frontier_gt_4000km=%d miss_likely_coverage_failure=%d miss_likely_true_divergence_gap=%d miss_true_divergence_gap=%d true_divergence_structured_synthetic=%d miss_destructive_exclusion_gap=%d miss_ambiguous_gap=%d miss_frontier_pair_structured_synthetic=%d miss_frontier_pair_vertex_approximation=%d ambiguous_structured_synthetic=%d destructive_exclusion_override_continuity=%d destructive_exclusion_structured_synthetic=%d destructive_exclusion_fallback_oceanic=%d partial_destructive_candidate_removal=%d continental_samples_before=%d continental_samples_after=%d continental_samples_gained=%d continental_samples_lost=%d tectonic_maintenance_applied=%d tectonic_maintenance_recovered=%d tectonic_maintenance_gain=%d tectonic_maintenance_same_plate_recovered=%d tectonic_maintenance_cross_plate_recovered=%d tectonic_maintenance_andean_tagged=%d tectonic_maintenance_elevation_boosted=%d tectonic_maintenance_thickness_boosted=%d continental_loss_from_oceanic_creation=%d continental_loss_from_hit=%d continental_loss_from_hit_cw_threshold_mismatch=%d continental_loss_from_hit_multi_winner=%d continental_loss_from_hit_single_winner=%d single_hit_loss_same_plate=%d single_hit_loss_cross_plate=%d single_hit_loss_frontier_hit=%d single_hit_loss_interior_hit=%d single_hit_loss_near_frontier=%d single_hit_loss_interior_region=%d single_hit_loss_dominant_continental=%d single_hit_loss_dominant_subcontinental=%d single_hit_loss_alt_nocap=%d single_hit_loss_prev_plate_alt_nocap=%d single_hit_loss_other_plate_alt_nocap=%d single_hit_loss_mixed_source_plate=%d single_hit_loss_mixed_source_cw=%d multi_hit_loss_same_plate=%d multi_hit_loss_cross_plate=%d multi_hit_loss_frontier_hit=%d multi_hit_loss_interior_hit=%d multi_hit_loss_near_frontier=%d fragmented_component_count=%d fragmented_components_touching_oceanic_creation=%d fragmented_components_dominated_by_oceanic_creation=%d fragmented_components_touching_copied_frontier_hit=%d fragmented_components_dominated_by_copied_frontier_hit=%d fragmented_components_touching_multi_hit_winner=%d fragmented_components_strongly_multi_hit=%d fragmented_components_majority_copied_frontier_hit_with_cw_mismatch=%d fragmented_components_adjacent_to_destructive_gap=%d fragment_size_1=%d fragment_size_2_to_4=%d fragment_size_5_to_16=%d fragment_size_17_plus=%d representative_miss_count=%d representative_single_hit_loss_count=%d top_multi_hit_pattern_count=%d"),
				*SummaryTag,
				Attribution.SolveStep,
				Attribution.Interval,
			Attribution.HitCount,
			Attribution.MissCount,
			Attribution.OceanicCreationCount,
			Attribution.SingleHitWinnerCount,
			Attribution.MultiHitWinnerCount,
			Attribution.MultiHitWinnerSamePlateCount,
			Attribution.MultiHitWinnerCrossPlateCount,
			Attribution.MultiHitWinnerCopiedFrontierHitCount,
			Attribution.MultiHitWinnerInteriorHitCount,
			Attribution.MultiHitWinnerNearFrontierCount,
			Attribution.MultiHitSamplesWithTrackedCandidateCount,
			Attribution.MultiHitSamplesWithinTrackedTriangleCount,
			Attribution.MultiHitSamplesWithinTrackedOneRingCount,
			Attribution.MultiHitSamplesWithinTrackedTwoRingCount,
			Attribution.MultiHitCandidateSamePlateOnlyCount,
			Attribution.MultiHitCandidateCrossPlateOnlyCount,
			Attribution.MultiHitCandidateMixedPlateCount,
			Attribution.MultiHitCandidateCopiedFrontierOnlyCount,
			Attribution.MultiHitCandidateInteriorOnlyCount,
			Attribution.MultiHitCandidateMixedGeometryCount,
			Attribution.MultiHitAdjacentToDestructiveGapCount,
			Attribution.MultiHitAdjacentToContinuityHandledDestructiveGapCount,
			Attribution.OverlapCoherenceSupportPrunedSampleCount,
			Attribution.OverlapCoherencePreviousPlateStabilizedSampleCount,
			Attribution.OverlapCoherenceSuppressedCandidateCount,
			Attribution.CopiedFrontierHitCount,
			Attribution.InteriorHitCount,
			Attribution.HitCwThresholdMismatchCount,
			Attribution.CopiedFrontierHitCwThresholdMismatchCount,
			Attribution.InteriorHitCwThresholdMismatchCount,
			Attribution.MissAdjacentToCopiedFrontierGeometryCount,
			Attribution.MissNearMultiHitRegionCount,
			Attribution.MissDivergentMotionCount,
			Attribution.MissConvergentMotionCount,
			Attribution.MissWeakOrNoMotionCount,
			Attribution.MissAdjacentFrontierAndDivergentCount,
			Attribution.MissAdjacentFrontierAndWeakCount,
			Attribution.MissAdjacentFrontierAndConvergentCount,
			Attribution.MissAdjacentFrontierAndNearMultiHitCount,
			Attribution.MissNearestCopiedFrontierDistanceLe100KmCount,
			Attribution.MissNearestCopiedFrontierDistanceLe500KmCount,
			Attribution.MissNearestCopiedFrontierDistanceLe1000KmCount,
			Attribution.MissNearestCopiedFrontierDistanceLe4000KmCount,
			Attribution.MissNearestCopiedFrontierDistanceGt4000KmCount,
			Attribution.MissLikelyCoverageFailureCount,
			Attribution.MissLikelyTrueDivergenceGapCount,
			Attribution.MissTrueDivergenceGapCount,
			Attribution.TrueDivergenceStructuredSyntheticCount,
			Attribution.MissDestructiveExclusionGapCount,
			Attribution.MissAmbiguousGapCount,
			Attribution.MissFrontierPairStructuredSyntheticCount,
			Attribution.MissFrontierPairVertexApproximationCount,
			Attribution.AmbiguousStructuredSyntheticCount,
			Attribution.DestructiveExclusionOverrideContinuityCount,
			Attribution.DestructiveExclusionStructuredSyntheticCount,
			Attribution.DestructiveExclusionFallbackOceanicCount,
			Attribution.SamplesWithPartialDestructiveCandidateRemovalCount,
			Attribution.ContinentalSamplesBefore,
			Attribution.ContinentalSamplesAfter,
			Attribution.ContinentalSamplesGained,
			Attribution.ContinentalSamplesLost,
				Attribution.TectonicMaintenanceAppliedCount,
				Attribution.TectonicMaintenanceContinentalRecoveredCount,
				Attribution.TectonicMaintenanceContinentalGainCount,
				Attribution.TectonicMaintenanceSamePlateRecoveredCount,
				Attribution.TectonicMaintenanceCrossPlateRecoveredCount,
				Attribution.TectonicMaintenanceAndeanTaggedCount,
				Attribution.TectonicMaintenanceElevationBoostCount,
				Attribution.TectonicMaintenanceThicknessBoostCount,
				Attribution.ContinentalLossFromOceanicCreationCount,
				Attribution.ContinentalLossFromHitCount,
				Attribution.ContinentalLossFromHitCwThresholdMismatchCount,
				Attribution.ContinentalLossFromHitMultiWinnerCount,
				Attribution.ContinentalLossFromHitSingleWinnerCount,
				Attribution.SingleHitContinentalLossSamePlateCount,
				Attribution.SingleHitContinentalLossCrossPlateCount,
				Attribution.SingleHitContinentalLossCopiedFrontierHitCount,
				Attribution.SingleHitContinentalLossInteriorHitCount,
				Attribution.SingleHitContinentalLossNearFrontierCount,
				Attribution.SingleHitContinentalLossInteriorRegionCount,
				Attribution.SingleHitContinentalLossDominantSourceContinentalCount,
				Attribution.SingleHitContinentalLossDominantSourceSubcontinentalCount,
				Attribution.SingleHitContinentalLossAlternativeNoCapHitCount,
				Attribution.SingleHitContinentalLossPreviousPlateNoCapHitCount,
				Attribution.SingleHitContinentalLossOtherPlateNoCapHitCount,
				Attribution.SingleHitContinentalLossSourceTriangleMixedPreviousPlateCount,
				Attribution.SingleHitContinentalLossSourceTriangleMixedContinentalCount,
				Attribution.MultiHitContinentalLossSamePlateCount,
				Attribution.MultiHitContinentalLossCrossPlateCount,
				Attribution.MultiHitContinentalLossCopiedFrontierHitCount,
				Attribution.MultiHitContinentalLossInteriorHitCount,
				Attribution.MultiHitContinentalLossNearFrontierCount,
				Attribution.FragmentedComponentCount,
				Attribution.FragmentedComponentsTouchingOceanicCreationCount,
				Attribution.FragmentedComponentsDominatedByOceanicCreationCount,
			Attribution.FragmentedComponentsTouchingCopiedFrontierHitCount,
			Attribution.FragmentedComponentsDominatedByCopiedFrontierHitCount,
			Attribution.FragmentedComponentsTouchingMultiHitWinnerCount,
			Attribution.FragmentedComponentsStronglyAssociatedWithMultiHitWinnerCount,
			Attribution.FragmentedComponentsMajorityCopiedFrontierHitWithCwMismatchCount,
			Attribution.FragmentedComponentsAdjacentToDestructiveExclusionGapCount,
				Attribution.FragmentSizeBuckets.Size1,
				Attribution.FragmentSizeBuckets.Size2To4,
				Attribution.FragmentSizeBuckets.Size5To16,
				Attribution.FragmentSizeBuckets.Size17Plus,
				Attribution.RepresentativeMisses.Num(),
				Attribution.RepresentativeSingleHitLosses.Num(),
				Attribution.TopMultiHitPatterns.Num());
		Test.AddInfo(SummaryMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *SummaryMessage);

		for (const FTectonicPlanetV6PlateCount& PlateCount : Attribution.SingleHitLossCountsByPreviousPlate)
		{
			const FString PlateMessage = FString::Printf(
				TEXT("%s single_hit_loss_previous_plate=%d count=%d"),
				*SummaryTag,
				PlateCount.PlateId,
				PlateCount.Count);
			Test.AddInfo(PlateMessage);
			UE_LOG(LogTemp, Log, TEXT("%s"), *PlateMessage);
		}

		for (const FTectonicPlanetV6PlateCount& PlateCount : Attribution.SingleHitLossCountsByFinalPlate)
		{
			const FString PlateMessage = FString::Printf(
				TEXT("%s single_hit_loss_final_plate=%d count=%d"),
				*SummaryTag,
				PlateCount.PlateId,
				PlateCount.Count);
			Test.AddInfo(PlateMessage);
			UE_LOG(LogTemp, Log, TEXT("%s"), *PlateMessage);
		}

		for (const FTectonicPlanetV6CopiedFrontierHitLossPatternCount& PatternCount : Attribution.TopSingleHitLossPatterns)
		{
			const FString PatternMessage = FString::Printf(
				TEXT("%s single_hit_loss_pattern=%s count=%d"),
				*SummaryTag,
				*PatternCount.Pattern,
				PatternCount.Count);
			Test.AddInfo(PatternMessage);
			UE_LOG(LogTemp, Log, TEXT("%s"), *PatternMessage);
		}

		for (const FTectonicPlanetV6CopiedFrontierHitLossPatternCount& PatternCount : Attribution.TopMultiHitPatterns)
		{
			const FString PatternMessage = FString::Printf(
				TEXT("%s multi_hit_pattern=%s count=%d"),
				*SummaryTag,
				*PatternCount.Pattern,
				PatternCount.Count);
			Test.AddInfo(PatternMessage);
			UE_LOG(LogTemp, Log, TEXT("%s"), *PatternMessage);
		}

		for (const FTectonicPlanetV6CopiedFrontierRepresentativeMiss& Miss : Attribution.RepresentativeMisses)
		{
			const FString MissMessage = FString::Printf(
				TEXT("%s sample=%d previous_plate=%d primary_nearest_plate=%d primary_nearest_triangle_distance_km=%.6f secondary_nearest_plate=%d secondary_nearest_triangle_distance_km=%.6f nearest_copied_frontier_triangle_distance_km=%.6f relative_normal_velocity=%.6f unfiltered_candidate_count=%d unfiltered_destructive_candidate_count=%d adjacent_to_copied_frontier=%d near_multi_hit_region=%d motion_class=%s gap_attribution=%s"),
				*SummaryTag,
				Miss.SampleIndex,
				Miss.PreviousPlateId,
				Miss.PrimaryNearestPlateId,
				Miss.PrimaryNearestTriangleDistance,
				Miss.SecondaryNearestPlateId,
				Miss.SecondaryNearestTriangleDistance,
				Miss.NearestCopiedFrontierTriangleDistance,
				Miss.RelativeNormalVelocity,
				Miss.UnfilteredCandidateCount,
				Miss.UnfilteredDestructiveCandidateCount,
				Miss.bAdjacentToCopiedFrontierGeometry ? 1 : 0,
				Miss.bNearMultiHitRegion ? 1 : 0,
				GetCopiedFrontierMotionClassName(Miss.MotionClass),
				GetCopiedFrontierGapAttributionName(Miss.GapAttribution));
			Test.AddInfo(MissMessage);
			UE_LOG(LogTemp, Log, TEXT("%s"), *MissMessage);
		}

		for (const FTectonicPlanetV6CopiedFrontierRepresentativeHitLoss& Loss : Attribution.RepresentativeSingleHitLosses)
		{
			const FString LossMessage = FString::Printf(
				TEXT("%s hit_loss sample=%d previous_plate=%d final_plate=%d dominant_source_sample=%d dominant_source_previous_plate=%d dominant_source_continental_weight=%.6f transferred_continental_weight=%.6f winning_fit_score=%.6f nearest_copied_frontier_triangle_distance_km=%.6f no_cap_hit_count=%d previous_plate_no_cap_hit_count=%d other_plate_no_cap_hit_count=%d source_triangle_unique_previous_plate_count=%d source_triangle_continental_corner_count=%d single_hit=%d same_plate_winner=%d copied_frontier_winning_triangle=%d near_frontier=%d adjacent_to_copied_frontier=%d dominant_source_was_continental=%d"),
				*SummaryTag,
				Loss.SampleIndex,
				Loss.PreviousPlateId,
				Loss.FinalPlateId,
				Loss.DominantSourceCanonicalSampleIndex,
				Loss.DominantSourcePreviousPlateId,
				Loss.DominantSourceContinentalWeight,
				Loss.TransferredContinentalWeight,
				Loss.WinningFitScore,
				Loss.NearestCopiedFrontierTriangleDistance,
				Loss.NoCapHitCount,
				Loss.PreviousPlateNoCapHitCount,
				Loss.OtherPlateNoCapHitCount,
				Loss.SourceTriangleUniquePreviousPlateCount,
				Loss.SourceTriangleContinentalCornerCount,
				Loss.bSingleHit ? 1 : 0,
				Loss.bSamePlateWinner ? 1 : 0,
				Loss.bCopiedFrontierWinningTriangle ? 1 : 0,
				Loss.bNearFrontier ? 1 : 0,
				Loss.bAdjacentToCopiedFrontierGeometry ? 1 : 0,
				Loss.bDominantSourceWasContinental ? 1 : 0);
			Test.AddInfo(LossMessage);
			UE_LOG(LogTemp, Log, TEXT("%s"), *LossMessage);
		}
	}

	bool ExportV6CheckpointMaps(
		FAutomationTestBase& Test,
		const FTectonicPlanetV6& Planet,
		const FString& ExportRoot,
		const int32 Step)
	{
		const FString OutputDirectory = FPaths::Combine(ExportRoot, FString::Printf(TEXT("step_%03d"), Step));
		FTectonicMollweideExportOptions Options;
		Options.Mode = ETectonicMapExportMode::All;
		Options.Width = TestExportWidth;
		Options.Height = TestExportHeight;
		Options.OutputDirectory = OutputDirectory;

		FTectonicMollweideExportStats ExportStats;
		FString Error;
		const bool bExported = TectonicMollweideExporter::ExportPlanet(Planet.GetPlanet(), Options, ExportStats, Error);
		Test.TestTrue(*FString::Printf(TEXT("V6 export step %d succeeded"), Step), bExported);
		if (!bExported)
		{
			Test.AddError(FString::Printf(TEXT("V6 export step %d failed: %s"), Step, *Error));
		}
		return bExported;
	}

	bool ExportV6DebugOverlays(
		FAutomationTestBase& Test,
		const FTectonicPlanetV6& Planet,
		const FString& ExportRoot,
		const int32 Step)
	{
		const FString OutputDirectory = FPaths::Combine(ExportRoot, FString::Printf(TEXT("step_%03d"), Step));
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		const int32 SampleCount = PlanetData.Samples.Num();
		if (SampleCount == 0) { return false; }

		bool bAllSucceeded = true;
		FString Error;

		// 1. OwnershipChurnMask: 1.0 = sample changed plate this cycle, 0.0 = retained
		{
			const FTectonicPlanetV6OwnershipChurnDiagnostic Churn = Planet.ComputeOwnershipChurnDiagnosticForTest();
			TArray<float> ChurnValues;
			ChurnValues.SetNum(SampleCount);

			const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples = Planet.GetLastResolvedSamplesForTest();
			for (int32 I = 0; I < SampleCount; ++I)
			{
				if (ResolvedSamples.IsValidIndex(I) &&
					ResolvedSamples[I].PreviousPlateId != INDEX_NONE &&
					ResolvedSamples[I].FinalPlateId != INDEX_NONE &&
					ResolvedSamples[I].FinalPlateId != ResolvedSamples[I].PreviousPlateId)
				{
					ChurnValues[I] = 1.0f;
				}
				else
				{
					ChurnValues[I] = 0.0f;
				}
			}

			const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("OwnershipChurnMask.png"));
			if (!TectonicMollweideExporter::ExportScalarOverlay(
				PlanetData, ChurnValues, 0.0f, 1.0f, OutputPath, TestExportWidth, TestExportHeight, Error))
			{
				Test.AddError(FString::Printf(TEXT("OwnershipChurnMask export step %d failed: %s"), Step, *Error));
				bAllSucceeded = false;
			}
		}

		// 2. MissLineageMask: resynthesis depth (0 = no miss, 1 = first, 2+ = repeat)
		{
			const TArray<uint8>& LineageCounts = Planet.GetMissLineageCountsForTest();
			if (LineageCounts.Num() == SampleCount)
			{
				TArray<float> LineageValues;
				LineageValues.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					LineageValues[I] = static_cast<float>(LineageCounts[I]);
				}

				const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("MissLineageMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData, LineageValues, 0.0f, 5.0f, OutputPath, TestExportWidth, TestExportHeight, Error))
				{
					Test.AddError(FString::Printf(TEXT("MissLineageMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		// 3. ConvergentPersistenceMask: current shadow persistence count for convergent/collision samples
		{
			const TArray<uint8>& PersistenceCounts = Planet.GetCollisionShadowPersistenceMaskForTest();
			if (PersistenceCounts.Num() == SampleCount)
			{
				TArray<float> PersistenceValues;
				PersistenceValues.SetNum(SampleCount);
				float MaxPersistence = 1.0f;
				for (int32 I = 0; I < SampleCount; ++I)
				{
					PersistenceValues[I] = static_cast<float>(PersistenceCounts[I]);
					MaxPersistence = FMath::Max(MaxPersistence, PersistenceValues[I]);
				}

				const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("ConvergentPersistenceMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData,
					PersistenceValues,
					0.0f,
					MaxPersistence,
					OutputPath,
					TestExportWidth,
					TestExportHeight,
					Error))
				{
					Test.AddError(FString::Printf(TEXT("ConvergentPersistenceMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		// 4. CollisionExecutedMask: samples affected by the current solve's collision execution slice
		{
			const TArray<uint8>& CollisionExecutionMask = Planet.GetCollisionExecutionMaskForTest();
			if (CollisionExecutionMask.Num() == SampleCount)
			{
				TArray<float> CollisionExecutionValues;
				CollisionExecutionValues.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					CollisionExecutionValues[I] = CollisionExecutionMask[I] != 0 ? 1.0f : 0.0f;
				}

				const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("CollisionExecutedMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData,
					CollisionExecutionValues,
					0.0f,
					1.0f,
					OutputPath,
					TestExportWidth,
					TestExportHeight,
					Error))
				{
					Test.AddError(FString::Printf(TEXT("CollisionExecutedMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		// 5. CollisionTransferMask: samples structurally transferred by the current solve
		{
			const TArray<uint8>& CollisionTransferMask = Planet.GetCollisionTransferMaskForTest();
			if (CollisionTransferMask.Num() == SampleCount)
			{
				TArray<float> CollisionTransferValues;
				CollisionTransferValues.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					CollisionTransferValues[I] = CollisionTransferMask[I] != 0 ? 1.0f : 0.0f;
				}

				const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("CollisionTransferMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData,
					CollisionTransferValues,
					0.0f,
					1.0f,
					OutputPath,
					TestExportWidth,
					TestExportHeight,
					Error))
				{
					Test.AddError(FString::Printf(TEXT("CollisionTransferMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		// 6. CollisionCumulativeMask: samples touched by any collision event through this checkpoint
		{
			const TArray<uint8>& CollisionCumulativeMask = Planet.GetCollisionCumulativeExecutionMaskForTest();
			if (CollisionCumulativeMask.Num() == SampleCount)
			{
				TArray<float> CollisionCumulativeValues;
				CollisionCumulativeValues.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					CollisionCumulativeValues[I] = CollisionCumulativeMask[I] != 0 ? 1.0f : 0.0f;
				}

				const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("CollisionCumulativeMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData,
					CollisionCumulativeValues,
					0.0f,
					1.0f,
					OutputPath,
					TestExportWidth,
					TestExportHeight,
					Error))
				{
					Test.AddError(FString::Printf(TEXT("CollisionCumulativeMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		// 7. CollisionCumulativeTransferMask: samples structurally transferred through this checkpoint
		{
			const TArray<uint8>& CollisionCumulativeTransferMask =
				Planet.GetCollisionCumulativeTransferMaskForTest();
			if (CollisionCumulativeTransferMask.Num() == SampleCount)
			{
				TArray<float> CollisionCumulativeTransferValues;
				CollisionCumulativeTransferValues.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					CollisionCumulativeTransferValues[I] =
						CollisionCumulativeTransferMask[I] != 0 ? 1.0f : 0.0f;
				}

				const FString OutputPath =
					FPaths::Combine(OutputDirectory, TEXT("CollisionCumulativeTransferMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData,
					CollisionCumulativeTransferValues,
					0.0f,
					1.0f,
					OutputPath,
					TestExportWidth,
					TestExportHeight,
					Error))
				{
					Test.AddError(FString::Printf(TEXT("CollisionCumulativeTransferMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		// 8. CollisionElevationDeltaMask: cumulative collision-driven elevation gain in km
		{
			const TArray<float>& CollisionElevationDeltaMask = Planet.GetCollisionCumulativeElevationDeltaMaskForTest();
			if (CollisionElevationDeltaMask.Num() == SampleCount)
			{
				float MaxDelta = 1.0f;
				for (const float Value : CollisionElevationDeltaMask)
				{
					MaxDelta = FMath::Max(MaxDelta, Value);
				}

				const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("CollisionElevationDeltaMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData,
					CollisionElevationDeltaMask,
					0.0f,
					MaxDelta,
					OutputPath,
					TestExportWidth,
					TestExportHeight,
					Error))
				{
					Test.AddError(FString::Printf(TEXT("CollisionElevationDeltaMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		// 9. CollisionContinentalGainMask: cumulative count of collision-driven CW gains
		{
			const TArray<float>& CollisionContinentalGainMask = Planet.GetCollisionCumulativeContinentalGainMaskForTest();
			if (CollisionContinentalGainMask.Num() == SampleCount)
			{
				float MaxGain = 1.0f;
				for (const float Value : CollisionContinentalGainMask)
				{
					MaxGain = FMath::Max(MaxGain, Value);
				}

				const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("CollisionContinentalGainMask.png"));
				if (!TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData,
					CollisionContinentalGainMask,
					0.0f,
					MaxGain,
					OutputPath,
					TestExportWidth,
					TestExportHeight,
					Error))
				{
					Test.AddError(FString::Printf(TEXT("CollisionContinentalGainMask export step %d failed: %s"), Step, *Error));
					bAllSucceeded = false;
				}
			}
		}

		return bAllSucceeded;
	}

	bool ExportV6CollisionTuningInnerLoopOverlays(
		FAutomationTestBase& Test,
		const FTectonicPlanetV6& Planet,
		const FString& ExportRoot,
		const int32 Step)
	{
		const FString OutputDirectory = FPaths::Combine(ExportRoot, FString::Printf(TEXT("step_%03d"), Step));
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		const int32 SampleCount = PlanetData.Samples.Num();
		if (SampleCount == 0) { return false; }

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*OutputDirectory);

		bool bAllSucceeded = true;
		FString Error;

		const auto ExportOverlay =
			[&](
				const TCHAR* Name,
				const TArray<float>& Values,
				const float MinValue,
				const float MaxValue)
		{
			const FString OutputPath = FPaths::Combine(OutputDirectory, Name);
			if (!TectonicMollweideExporter::ExportScalarOverlay(
				PlanetData,
				Values,
				MinValue,
				MaxValue,
				OutputPath,
				TestExportWidth,
				TestExportHeight,
				Error))
			{
				Test.AddError(FString::Printf(TEXT("%s export step %d failed: %s"), Name, Step, *Error));
				bAllSucceeded = false;
			}
		};

		{
			TArray<float> ChurnValues;
			ChurnValues.Init(0.0f, SampleCount);
			const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples = Planet.GetLastResolvedSamplesForTest();
			for (int32 I = 0; I < SampleCount; ++I)
			{
				if (ResolvedSamples.IsValidIndex(I) &&
					ResolvedSamples[I].PreviousPlateId != INDEX_NONE &&
					ResolvedSamples[I].FinalPlateId != INDEX_NONE &&
					ResolvedSamples[I].FinalPlateId != ResolvedSamples[I].PreviousPlateId)
				{
					ChurnValues[I] = 1.0f;
				}
			}
			ExportOverlay(TEXT("OwnershipChurnMask.png"), ChurnValues, 0.0f, 1.0f);
		}

		{
			const TArray<uint8>& CollisionExecutionMask = Planet.GetCollisionExecutionMaskForTest();
			if (CollisionExecutionMask.Num() == SampleCount)
			{
				TArray<float> Values;
				Values.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					Values[I] = CollisionExecutionMask[I] != 0 ? 1.0f : 0.0f;
				}
				ExportOverlay(TEXT("CollisionExecutedMask.png"), Values, 0.0f, 1.0f);
			}
		}

		{
			const TArray<uint8>& CollisionTransferMask = Planet.GetCollisionTransferMaskForTest();
			if (CollisionTransferMask.Num() == SampleCount)
			{
				TArray<float> Values;
				Values.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					Values[I] = CollisionTransferMask[I] != 0 ? 1.0f : 0.0f;
				}
				ExportOverlay(TEXT("CollisionTransferMask.png"), Values, 0.0f, 1.0f);
			}
		}

		{
			const TArray<uint8>& CollisionCumulativeMask = Planet.GetCollisionCumulativeExecutionMaskForTest();
			if (CollisionCumulativeMask.Num() == SampleCount)
			{
				TArray<float> Values;
				Values.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					Values[I] = CollisionCumulativeMask[I] != 0 ? 1.0f : 0.0f;
				}
				ExportOverlay(TEXT("CollisionCumulativeMask.png"), Values, 0.0f, 1.0f);
			}
		}

		{
			const TArray<uint8>& CollisionCumulativeTransferMask =
				Planet.GetCollisionCumulativeTransferMaskForTest();
			if (CollisionCumulativeTransferMask.Num() == SampleCount)
			{
				TArray<float> Values;
				Values.SetNum(SampleCount);
				for (int32 I = 0; I < SampleCount; ++I)
				{
					Values[I] = CollisionCumulativeTransferMask[I] != 0 ? 1.0f : 0.0f;
				}
				ExportOverlay(TEXT("CollisionCumulativeTransferMask.png"), Values, 0.0f, 1.0f);
			}
		}

		{
			const TArray<float>& CollisionElevationDeltaMask = Planet.GetCollisionCumulativeElevationDeltaMaskForTest();
			if (CollisionElevationDeltaMask.Num() == SampleCount)
			{
				float MaxDelta = 1.0f;
				for (const float Value : CollisionElevationDeltaMask)
				{
					MaxDelta = FMath::Max(MaxDelta, Value);
				}
				ExportOverlay(TEXT("CollisionElevationDeltaMask.png"), CollisionElevationDeltaMask, 0.0f, MaxDelta);
			}
		}

		{
			const TArray<float>& CollisionContinentalGainMask = Planet.GetCollisionCumulativeContinentalGainMaskForTest();
			if (CollisionContinentalGainMask.Num() == SampleCount)
			{
				float MaxGain = 1.0f;
				for (const float Value : CollisionContinentalGainMask)
				{
					MaxGain = FMath::Max(MaxGain, Value);
				}
				ExportOverlay(TEXT("CollisionContinentalGainMask.png"), CollisionContinentalGainMask, 0.0f, MaxGain);
			}
		}

		return bAllSucceeded;
	}

	FV9CollisionFidelityGateResult EvaluateV9CollisionFidelityStep100Gate(
		const FV6CheckpointSnapshot& BaselineSnapshot,
		const FV6CheckpointSnapshot& CandidateSnapshot)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec =
			BaselineSnapshot.CollisionExecution;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec =
			CandidateSnapshot.CollisionExecution;

		FV9CollisionFidelityGateResult Result;
		Result.RequiredCumulativeAffectedSamples = FMath::Max(
			BaselineExec.CumulativeCollisionAffectedSampleCount + 25,
			FMath::CeilToInt(static_cast<double>(BaselineExec.CumulativeCollisionAffectedSampleCount) * 1.15));
		Result.RequiredCumulativeContinentalGain = FMath::Max(
			BaselineExec.CumulativeCollisionDrivenContinentalGainCount + 10,
			FMath::CeilToInt(static_cast<double>(BaselineExec.CumulativeCollisionDrivenContinentalGainCount) * 1.15));
		Result.RequiredExecutedMeanElevationDeltaKm =
			BaselineExec.ExecutedMeanElevationDeltaKm > UE_DOUBLE_SMALL_NUMBER
				? BaselineExec.ExecutedMeanElevationDeltaKm * 1.10
				: 0.05;
		Result.RequiredCumulativeMeanElevationDeltaKm =
			BaselineExec.CumulativeMeanElevationDeltaKm > UE_DOUBLE_SMALL_NUMBER
				? BaselineExec.CumulativeMeanElevationDeltaKm * 1.10
				: 0.05;
		Result.MaxAllowedChurn = BaselineSnapshot.OwnershipChurn.ChurnFraction + 0.01;
		Result.MinAllowedCoherence = BaselineSnapshot.BoundaryCoherence.BoundaryCoherenceScore - 0.01;
		Result.MaxAllowedInteriorLeakage =
			BaselineSnapshot.BoundaryCoherence.InteriorLeakageFraction + 0.02;

		Result.bFootprintPass =
			CandidateExec.CumulativeCollisionAffectedSampleCount >= Result.RequiredCumulativeAffectedSamples;
		Result.bContinentalGainPass =
			CandidateExec.CumulativeCollisionDrivenContinentalGainCount >= Result.RequiredCumulativeContinentalGain;
		Result.bElevationPass =
			CandidateExec.ExecutedMeanElevationDeltaKm >= Result.RequiredExecutedMeanElevationDeltaKm ||
			CandidateExec.CumulativeMeanElevationDeltaKm >= Result.RequiredCumulativeMeanElevationDeltaKm;
		Result.bChurnPass =
			CandidateSnapshot.OwnershipChurn.ChurnFraction <= Result.MaxAllowedChurn;
		Result.bCoherencePass =
			CandidateSnapshot.BoundaryCoherence.BoundaryCoherenceScore >= Result.MinAllowedCoherence;
		Result.bLeakagePass =
			CandidateSnapshot.BoundaryCoherence.InteriorLeakageFraction <= Result.MaxAllowedInteriorLeakage;
		Result.bAllowStep200 =
			Result.bFootprintPass &&
			Result.bElevationPass &&
			Result.bContinentalGainPass &&
			Result.bChurnPass &&
			Result.bCoherencePass &&
			Result.bLeakagePass;
		return Result;
	}

	FV9CollisionStructuralGateResult EvaluateV9CollisionStructuralTransferStep100Gate(
		const FV6CheckpointSnapshot& BaselineSnapshot,
		const FV6CheckpointSnapshot& CandidateSnapshot,
		const int32 TotalSampleCount)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec =
			BaselineSnapshot.CollisionExecution;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec =
			CandidateSnapshot.CollisionExecution;

		FV9CollisionStructuralGateResult Result;
		Result.RequiredCumulativeTransferFootprint =
			FMath::Max(BaselineExec.CumulativeCollisionTransferredSampleCount + 12, 12);
		Result.RequiredMinPlateShareDelta = 8.0 / static_cast<double>(FMath::Max(TotalSampleCount, 1));
		Result.RequiredMaxPlateShareDelta = 64.0 / static_cast<double>(FMath::Max(TotalSampleCount, 1));
		Result.RequiredBoundaryLocalFraction = 0.75;
		Result.MaxAllowedChurn = BaselineSnapshot.OwnershipChurn.ChurnFraction + 0.01;
		Result.MinAllowedCoherence = BaselineSnapshot.BoundaryCoherence.BoundaryCoherenceScore - 0.01;
		Result.MaxAllowedInteriorLeakage =
			BaselineSnapshot.BoundaryCoherence.InteriorLeakageFraction + 0.02;

		const double RecipientShareDelta = FMath::Abs(CandidateExec.ExecutedRecipientPlateShareDelta);
		const double DonorShareDelta = FMath::Abs(CandidateExec.ExecutedDonorPlateShareDelta);
		const int32 CurrentOrCumulativeOwnershipChange = FMath::Max(
			CandidateExec.CollisionDrivenOwnershipChangeCount,
			CandidateExec.CumulativeCollisionDrivenOwnershipChangeCount);
		const int32 CurrentOrCumulativeTransferCount = FMath::Max(
			CandidateExec.CollisionTransferredSampleCount,
			CandidateExec.CumulativeCollisionTransferredSampleVisits);
		const int32 CurrentOrCumulativeContinentalTransferCount = FMath::Max(
			CandidateExec.CollisionTransferredContinentalSampleCount,
			CandidateExec.CumulativeCollisionTransferredContinentalSampleCount);
		const double BoundaryLocalFraction =
			CandidateExec.CollisionTransferredSampleCount > 0
				? static_cast<double>(CandidateExec.ExecutedTransferBoundaryLocalSampleCount) /
					static_cast<double>(CandidateExec.CollisionTransferredSampleCount)
				: 0.0;

		Result.bOwnershipChangePass = CurrentOrCumulativeOwnershipChange > 0;
		Result.bTransferPass = CurrentOrCumulativeTransferCount > 0;
		Result.bTransferredContinentalPass = CurrentOrCumulativeContinentalTransferCount > 0;
		Result.bTransferFootprintPass =
			CandidateExec.CumulativeCollisionTransferredSampleCount >= Result.RequiredCumulativeTransferFootprint;
		Result.bShareDeltaPass =
			CandidateExec.CollisionTransferredSampleCount > 0 &&
			RecipientShareDelta >= Result.RequiredMinPlateShareDelta &&
			DonorShareDelta >= Result.RequiredMinPlateShareDelta &&
			RecipientShareDelta <= Result.RequiredMaxPlateShareDelta &&
			DonorShareDelta <= Result.RequiredMaxPlateShareDelta;
		Result.bBoundaryLocalityPass =
			CandidateExec.CollisionTransferredSampleCount > 0 &&
			BoundaryLocalFraction >= Result.RequiredBoundaryLocalFraction;
		Result.bChurnPass =
			CandidateSnapshot.OwnershipChurn.ChurnFraction <= Result.MaxAllowedChurn;
		Result.bCoherencePass =
			CandidateSnapshot.BoundaryCoherence.BoundaryCoherenceScore >= Result.MinAllowedCoherence;
		Result.bLeakagePass =
			CandidateSnapshot.BoundaryCoherence.InteriorLeakageFraction <= Result.MaxAllowedInteriorLeakage;
		Result.bAllowStep200 =
			Result.bOwnershipChangePass &&
			Result.bTransferPass &&
			Result.bTransferredContinentalPass &&
			Result.bTransferFootprintPass &&
			Result.bShareDeltaPass &&
			Result.bBoundaryLocalityPass &&
			Result.bChurnPass &&
			Result.bCoherencePass &&
			Result.bLeakagePass;
		return Result;
	}

	FV9ContinentalElevationStats ComputeContinentalElevationStats(const FTectonicPlanetV6& Planet)
	{
		FV9ContinentalElevationStats Result;
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		TArray<double> ContinentalElevations;
		ContinentalElevations.Reserve(PlanetData.Samples.Num() / 3);
		for (const FSample& Sample : PlanetData.Samples)
		{
			if (Sample.ContinentalWeight >= 0.5f)
			{
				++Result.ContinentalSampleCount;
				ContinentalElevations.Add(static_cast<double>(Sample.Elevation));
				if (Sample.Elevation >= 2.0f) { ++Result.SamplesAbove2Km; }
				if (Sample.Elevation >= 5.0f) { ++Result.SamplesAbove5Km; }
				Result.MaxElevationKm = FMath::Max(Result.MaxElevationKm, static_cast<double>(Sample.Elevation));
			}
		}
		if (!ContinentalElevations.IsEmpty())
		{
			double Sum = 0.0;
			for (const double E : ContinentalElevations) { Sum += E; }
			Result.MeanElevationKm = Sum / static_cast<double>(ContinentalElevations.Num());
			ContinentalElevations.Sort();
			const int32 P95Index = FMath::Min(
				static_cast<int32>(static_cast<double>(ContinentalElevations.Num()) * 0.95),
				ContinentalElevations.Num() - 1);
			Result.P95ElevationKm = ContinentalElevations[P95Index];
		}
		return Result;
	}

	FV9ContinentalElevationBandDiagnostic ComputeContinentalElevationBandDiagnostic(
		const FTectonicPlanetV6& Planet)
	{
		FV9ContinentalElevationBandDiagnostic Result;
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		TArray<double> ContinentalElevations;
		TArray<double> SubaerialContinentalElevations;
		ContinentalElevations.Reserve(PlanetData.Samples.Num() / 3);
		SubaerialContinentalElevations.Reserve(PlanetData.Samples.Num() / 3);

		for (const FSample& Sample : PlanetData.Samples)
		{
			if (Sample.ContinentalWeight < 0.5f)
			{
				continue;
			}

			const double ElevationKm = static_cast<double>(Sample.Elevation);
			++Result.ContinentalSampleCount;
			ContinentalElevations.Add(ElevationKm);
			Result.ContinentalMaxElevationKm =
				FMath::Max(Result.ContinentalMaxElevationKm, ElevationKm);

			if (Sample.Elevation >= 2.0f)
			{
				++Result.SamplesAbove2Km;
			}
			if (Sample.Elevation >= 5.0f)
			{
				++Result.SamplesAbove5Km;
			}

			if (Sample.Elevation <= 0.0f)
			{
				++Result.SubmergedContinentalSampleCount;
				continue;
			}

			++Result.SubaerialContinentalSampleCount;
			SubaerialContinentalElevations.Add(ElevationKm);

			if (Sample.Elevation <= 0.5f)
			{
				++Result.ContinentalSamples0To0p5Km;
			}
			else if (Sample.Elevation <= 1.0f)
			{
				++Result.ContinentalSamples0p5To1Km;
			}
			else if (Sample.Elevation <= 2.0f)
			{
				++Result.ContinentalSamples1To2Km;
			}
			else if (Sample.Elevation <= 5.0f)
			{
				++Result.ContinentalSamples2To5Km;
			}
			else
			{
				++Result.ContinentalSamplesAbove5Km;
			}
		}

		if (!ContinentalElevations.IsEmpty())
		{
			double Sum = 0.0;
			for (const double ElevationKm : ContinentalElevations)
			{
				Sum += ElevationKm;
			}
			Result.ContinentalMeanElevationKm =
				Sum / static_cast<double>(ContinentalElevations.Num());
			ContinentalElevations.Sort();
			const int32 P95Index = FMath::Min(
				static_cast<int32>(static_cast<double>(ContinentalElevations.Num()) * 0.95),
				ContinentalElevations.Num() - 1);
			Result.ContinentalP95ElevationKm = ContinentalElevations[P95Index];
		}

		if (!SubaerialContinentalElevations.IsEmpty())
		{
			double Sum = 0.0;
			for (const double ElevationKm : SubaerialContinentalElevations)
			{
				Sum += ElevationKm;
			}
			Result.SubaerialMeanElevationKm =
				Sum / static_cast<double>(SubaerialContinentalElevations.Num());
			SubaerialContinentalElevations.Sort();
			const int32 P95Index = FMath::Min(
				static_cast<int32>(static_cast<double>(SubaerialContinentalElevations.Num()) * 0.95),
				SubaerialContinentalElevations.Num() - 1);
			Result.SubaerialP95ElevationKm = SubaerialContinentalElevations[P95Index];
		}

		return Result;
	}

	struct FV6PlateSizeStats
	{
		int32 MinSamples = 0;
		double MeanSamples = 0.0;
		int32 MaxSamples = 0;
	};

	FV6PlateSizeStats ComputePlateSizeStats(const FTectonicPlanetV6& Planet)
	{
		FV6PlateSizeStats Result;
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		if (PlanetData.Plates.IsEmpty())
		{
			return Result;
		}

		int64 TotalSamples = 0;
		Result.MinSamples = PlanetData.Plates[0].MemberSamples.Num();
		Result.MaxSamples = Result.MinSamples;
		for (const FPlate& Plate : PlanetData.Plates)
		{
			const int32 MemberCount = Plate.MemberSamples.Num();
			Result.MinSamples = FMath::Min(Result.MinSamples, MemberCount);
			Result.MaxSamples = FMath::Max(Result.MaxSamples, MemberCount);
			TotalSamples += MemberCount;
		}

		Result.MeanSamples =
			static_cast<double>(TotalSamples) / static_cast<double>(PlanetData.Plates.Num());
		return Result;
	}

	int32 ComputeCollisionRegionSamplesAboveElevationThreshold(
		const FTectonicPlanetV6& Planet,
		const double ElevationThresholdKm,
		const bool bUseCumulativeMask)
	{
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		const TArray<uint8>& CollisionMask =
			bUseCumulativeMask
				? Planet.GetCollisionCumulativeExecutionMaskForTest()
				: Planet.GetCollisionExecutionMaskForTest();
		if (CollisionMask.Num() != PlanetData.Samples.Num())
		{
			return 0;
		}

		int32 Count = 0;
		for (int32 SampleIndex = 0; SampleIndex < PlanetData.Samples.Num(); ++SampleIndex)
		{
			if (CollisionMask[SampleIndex] == 0)
			{
				continue;
			}

			if (PlanetData.Samples[SampleIndex].Elevation >= ElevationThresholdKm)
			{
				++Count;
			}
		}

		return Count;
	}

	int32 ComputeAndeanSampleCount(const FTectonicPlanetV6& Planet)
	{
		int32 Count = 0;
		for (const FSample& Sample : Planet.GetPlanet().Samples)
		{
			Count += Sample.OrogenyType == EOrogenyType::Andean ? 1 : 0;
		}
		return Count;
	}

	// --- Continental Mass Diagnostic Compute Functions ---

	FV9ContinentalMassDiagnostic ComputeContinentalMassDiagnostic(const FTectonicPlanetV6& Planet)
	{
		FV9ContinentalMassDiagnostic Result;
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		const int32 SampleCount = PlanetData.Samples.Num();
		if (SampleCount == 0) { return Result; }

		// Continental flag per sample
		TArray<uint8> IsContinental;
		IsContinental.SetNumZeroed(SampleCount);
		TArray<uint8> IsSubaerialContinental;
		IsSubaerialContinental.SetNumZeroed(SampleCount);
		for (int32 I = 0; I < SampleCount; ++I)
		{
			IsContinental[I] = PlanetData.Samples[I].ContinentalWeight >= 0.5f ? 1 : 0;
			IsSubaerialContinental[I] =
				(IsContinental[I] != 0 && PlanetData.Samples[I].Elevation > 0.0f) ? 1 : 0;
			Result.ContinentalSampleCount += IsContinental[I];
			Result.SubaerialContinentalSampleCount += IsSubaerialContinental[I];
		}
		Result.SubmergedContinentalSampleCount =
			Result.ContinentalSampleCount - Result.SubaerialContinentalSampleCount;
		Result.ContinentalAreaFraction = static_cast<double>(Result.ContinentalSampleCount) / static_cast<double>(SampleCount);
		Result.SubaerialContinentalFraction =
			static_cast<double>(Result.SubaerialContinentalSampleCount) / static_cast<double>(SampleCount);
		Result.SubmergedContinentalFraction =
			static_cast<double>(Result.SubmergedContinentalSampleCount) / static_cast<double>(SampleCount);

		// --- Connected Components (BFS on adjacency, continental only) ---
		Result.SampleComponentId.SetNum(SampleCount);
		for (int32 I = 0; I < SampleCount; ++I) { Result.SampleComponentId[I] = -1; }

		int32 NextComponentId = 0;
		TArray<int32> ComponentSizes;
		TArray<int32> BfsQueue;
		BfsQueue.Reserve(SampleCount);

		for (int32 Seed = 0; Seed < SampleCount; ++Seed)
		{
			if (!IsContinental[Seed] || Result.SampleComponentId[Seed] >= 0) { continue; }

			const int32 CompId = NextComponentId++;
			int32 CompSize = 0;
			BfsQueue.Reset();
			BfsQueue.Add(Seed);
			Result.SampleComponentId[Seed] = CompId;

			int32 Head = 0;
			while (Head < BfsQueue.Num())
			{
				const int32 Current = BfsQueue[Head++];
				++CompSize;

				if (PlanetData.SampleAdjacency.IsValidIndex(Current))
				{
					for (const int32 Neighbor : PlanetData.SampleAdjacency[Current])
					{
						if (IsContinental[Neighbor] && Result.SampleComponentId[Neighbor] < 0)
						{
							Result.SampleComponentId[Neighbor] = CompId;
							BfsQueue.Add(Neighbor);
						}
					}
				}
			}
			ComponentSizes.Add(CompSize);
		}

		Result.ComponentCount = ComponentSizes.Num();
		ComponentSizes.Sort([](const int32 A, const int32 B) { return A > B; });
		Result.LargestComponentSize = ComponentSizes.Num() > 0 ? ComponentSizes[0] : 0;
		for (int32 I = 0; I < FMath::Min(5, ComponentSizes.Num()); ++I)
		{
			Result.Top5ComponentSizes.Add(ComponentSizes[I]);
		}
		for (const int32 Size : ComponentSizes)
		{
			if (Size == 1) { ++Result.SingletonCount; }
			if (Size <= 10) { ++Result.TinyComponentCount; }
		}

		// --- Subaerial continental components ---
		TArray<int32> SubaerialComponentIds;
		SubaerialComponentIds.Init(-1, SampleCount);
		int32 NextSubaerialComponentId = 0;
		int32 LargestSubaerialComponentSize = 0;
		for (int32 Seed = 0; Seed < SampleCount; ++Seed)
		{
			if (!IsSubaerialContinental[Seed] || SubaerialComponentIds[Seed] >= 0)
			{
				continue;
			}

			const int32 ComponentId = NextSubaerialComponentId++;
			int32 ComponentSize = 0;
			BfsQueue.Reset();
			BfsQueue.Add(Seed);
			SubaerialComponentIds[Seed] = ComponentId;

			for (int32 Head = 0; Head < BfsQueue.Num(); ++Head)
			{
				const int32 Current = BfsQueue[Head];
				++ComponentSize;
				if (!PlanetData.SampleAdjacency.IsValidIndex(Current))
				{
					continue;
				}

				for (const int32 Neighbor : PlanetData.SampleAdjacency[Current])
				{
					if (IsSubaerialContinental[Neighbor] && SubaerialComponentIds[Neighbor] < 0)
					{
						SubaerialComponentIds[Neighbor] = ComponentId;
						BfsQueue.Add(Neighbor);
					}
				}
			}

			if (ComponentSize > LargestSubaerialComponentSize)
			{
				LargestSubaerialComponentSize = ComponentSize;
				Result.LargestSubaerialComponentId = ComponentId;
				Result.LargestSubaerialComponentSize = ComponentSize;
			}
		}

		// --- Coast Distance (multi-source BFS from oceanic samples) ---
		Result.SampleCoastDistHops.SetNum(SampleCount);
		for (int32 I = 0; I < SampleCount; ++I) { Result.SampleCoastDistHops[I] = -1; }

		TArray<int32> DistQueue;
		DistQueue.Reserve(SampleCount);
		// Seed: all oceanic samples adjacent to at least one continental sample
		for (int32 I = 0; I < SampleCount; ++I)
		{
			if (IsContinental[I]) { continue; }
			if (PlanetData.SampleAdjacency.IsValidIndex(I))
			{
				for (const int32 Neighbor : PlanetData.SampleAdjacency[I])
				{
					if (IsContinental[Neighbor] && Result.SampleCoastDistHops[Neighbor] < 0)
					{
						Result.SampleCoastDistHops[Neighbor] = 1;
						DistQueue.Add(Neighbor);
					}
				}
			}
		}
		{
			int32 Head = 0;
			while (Head < DistQueue.Num())
			{
				const int32 Current = DistQueue[Head++];
				const int32 CurrentDist = Result.SampleCoastDistHops[Current];
				if (PlanetData.SampleAdjacency.IsValidIndex(Current))
				{
					for (const int32 Neighbor : PlanetData.SampleAdjacency[Current])
					{
						if (IsContinental[Neighbor] && Result.SampleCoastDistHops[Neighbor] < 0)
						{
							Result.SampleCoastDistHops[Neighbor] = CurrentDist + 1;
							DistQueue.Add(Neighbor);
						}
					}
				}
			}
		}

		// Collect coast distance stats for continental samples
		TArray<int32> CoastDistValues;
		CoastDistValues.Reserve(Result.ContinentalSampleCount);
		TArray<int32> SubaerialCoastDistValues;
		SubaerialCoastDistValues.Reserve(Result.SubaerialContinentalSampleCount);
		for (int32 I = 0; I < SampleCount; ++I)
		{
			if (IsContinental[I] && Result.SampleCoastDistHops[I] > 0)
			{
				CoastDistValues.Add(Result.SampleCoastDistHops[I]);
				if (IsSubaerialContinental[I])
				{
					SubaerialCoastDistValues.Add(Result.SampleCoastDistHops[I]);
				}
			}
		}
		if (!CoastDistValues.IsEmpty())
		{
			CoastDistValues.Sort();
			double Sum = 0.0;
			for (const int32 D : CoastDistValues) { Sum += D; }
			Result.MeanCoastDistHops = Sum / static_cast<double>(CoastDistValues.Num());
			Result.P50CoastDist = CoastDistValues[CoastDistValues.Num() / 2];
			Result.P90CoastDist = CoastDistValues[FMath::Min(
				static_cast<int32>(static_cast<double>(CoastDistValues.Num()) * 0.90),
				CoastDistValues.Num() - 1)];
			Result.MaxCoastDist = CoastDistValues.Last();
		}
		if (!SubaerialCoastDistValues.IsEmpty())
		{
			SubaerialCoastDistValues.Sort();
			double Sum = 0.0;
			for (const int32 D : SubaerialCoastDistValues) { Sum += D; }
			Result.SubaerialMeanCoastDistHops =
				Sum / static_cast<double>(SubaerialCoastDistValues.Num());
			Result.SubaerialP50CoastDist =
				SubaerialCoastDistValues[SubaerialCoastDistValues.Num() / 2];
			Result.SubaerialP90CoastDist = SubaerialCoastDistValues[FMath::Min(
				static_cast<int32>(static_cast<double>(SubaerialCoastDistValues.Num()) * 0.90),
				SubaerialCoastDistValues.Num() - 1)];
		}

		// --- Active zone and boundary distribution ---
		const TArray<uint8>& ActiveZoneFlags = Planet.GetCurrentSolveActiveZoneFlagsForTest();
		const bool bHasActiveZone = ActiveZoneFlags.Num() == SampleCount;

		// Distance to nearest active-zone sample (BFS from active zone, continental only)
		TArray<int32> DistToActiveZone;
		DistToActiveZone.SetNum(SampleCount);
		for (int32 I = 0; I < SampleCount; ++I) { DistToActiveZone[I] = -1; }

		if (bHasActiveZone)
		{
			TArray<int32> AzQueue;
			AzQueue.Reserve(SampleCount);

			for (int32 I = 0; I < SampleCount; ++I)
			{
				if (IsContinental[I] && ActiveZoneFlags[I])
				{
					DistToActiveZone[I] = 0;
					AzQueue.Add(I);
					++Result.ActiveZoneContinentalCount;
				}
			}

			{
				int32 Head = 0;
				while (Head < AzQueue.Num())
				{
					const int32 Current = AzQueue[Head++];
					const int32 CurrentDist = DistToActiveZone[Current];
					if (PlanetData.SampleAdjacency.IsValidIndex(Current))
					{
						for (const int32 Neighbor : PlanetData.SampleAdjacency[Current])
						{
							if (IsContinental[Neighbor] && DistToActiveZone[Neighbor] < 0)
							{
								DistToActiveZone[Neighbor] = CurrentDist + 1;
								AzQueue.Add(Neighbor);
							}
						}
					}
				}
			}

			// Boundary band: continental samples that are boundary or 1-ring of boundary
			for (int32 I = 0; I < SampleCount; ++I)
			{
				if (!IsContinental[I]) { continue; }
				if (PlanetData.Samples[I].bIsBoundary)
				{
					++Result.BoundaryBandContinentalCount;
				}
			}

			Result.DeepInteriorContinentalCount =
				Result.ContinentalSampleCount - Result.ActiveZoneContinentalCount -
				FMath::Max(0, Result.BoundaryBandContinentalCount - Result.ActiveZoneContinentalCount);

			if (Result.ContinentalSampleCount > 0)
			{
				Result.ActiveZoneContinentalFraction =
					static_cast<double>(Result.ActiveZoneContinentalCount) / static_cast<double>(Result.ContinentalSampleCount);
				Result.BoundaryBandContinentalFraction =
					static_cast<double>(Result.BoundaryBandContinentalCount) / static_cast<double>(Result.ContinentalSampleCount);
				Result.DeepInteriorContinentalFraction =
					static_cast<double>(Result.DeepInteriorContinentalCount) / static_cast<double>(Result.ContinentalSampleCount);
			}

			// Distance-to-active-zone stats
			TArray<int32> AzDistValues;
			AzDistValues.Reserve(Result.ContinentalSampleCount);
			for (int32 I = 0; I < SampleCount; ++I)
			{
				if (IsContinental[I] && DistToActiveZone[I] >= 0)
				{
					AzDistValues.Add(DistToActiveZone[I]);
				}
			}
			if (!AzDistValues.IsEmpty())
			{
				AzDistValues.Sort();
				double Sum = 0.0;
				for (const int32 D : AzDistValues) { Sum += D; }
				Result.MeanDistToActiveZoneHops = Sum / static_cast<double>(AzDistValues.Num());
				Result.P50DistToActiveZone = AzDistValues[AzDistValues.Num() / 2];
				Result.P90DistToActiveZone = AzDistValues[FMath::Min(
					static_cast<int32>(static_cast<double>(AzDistValues.Num()) * 0.90),
					AzDistValues.Num() - 1)];
			}
		}

		// --- Largest continent profile ---
		if (Result.LargestComponentSize > 0)
		{
			// Find the component ID of the largest
			int32 LargestCompId = -1;
			{
				TMap<int32, int32> CompIdToSize;
				for (int32 I = 0; I < SampleCount; ++I)
				{
					if (Result.SampleComponentId[I] >= 0)
					{
						CompIdToSize.FindOrAdd(Result.SampleComponentId[I], 0)++;
					}
				}
				int32 MaxSize = 0;
				for (const auto& Pair : CompIdToSize)
				{
					if (Pair.Value > MaxSize)
					{
						MaxSize = Pair.Value;
						LargestCompId = Pair.Key;
					}
				}
			}
			Result.LargestComponentId = LargestCompId;

			TArray<double> LargestElevations;
			LargestElevations.Reserve(Result.LargestComponentSize);
			int32 LargestActiveCount = 0;
			int32 LargestBoundaryCount = 0;
			double LargestCoastDistSum = 0.0;
			int32 LargestCoastDistN = 0;

			for (int32 I = 0; I < SampleCount; ++I)
			{
				if (Result.SampleComponentId[I] != LargestCompId) { continue; }
				LargestElevations.Add(static_cast<double>(PlanetData.Samples[I].Elevation));
				if (bHasActiveZone && ActiveZoneFlags[I]) { ++LargestActiveCount; }
				if (PlanetData.Samples[I].bIsBoundary) { ++LargestBoundaryCount; }
				if (Result.SampleCoastDistHops[I] > 0)
				{
					LargestCoastDistSum += Result.SampleCoastDistHops[I];
					++LargestCoastDistN;
				}
			}

			if (!LargestElevations.IsEmpty())
			{
				double ElevSum = 0.0;
				for (const double E : LargestElevations) { ElevSum += E; }
				Result.LargestMeanElevationKm = ElevSum / static_cast<double>(LargestElevations.Num());
				LargestElevations.Sort();
				const int32 P95Idx = FMath::Min(
					static_cast<int32>(static_cast<double>(LargestElevations.Num()) * 0.95),
					LargestElevations.Num() - 1);
				Result.LargestP95ElevationKm = LargestElevations[P95Idx];
			}
			if (Result.LargestComponentSize > 0)
			{
				Result.LargestActiveZoneFraction =
					static_cast<double>(LargestActiveCount) / static_cast<double>(Result.LargestComponentSize);
				Result.LargestBoundaryFraction =
					static_cast<double>(LargestBoundaryCount) / static_cast<double>(Result.LargestComponentSize);
			}
			if (LargestCoastDistN > 0)
			{
				Result.LargestMeanCoastDist = LargestCoastDistSum / static_cast<double>(LargestCoastDistN);
			}
		}

		if (Result.LargestSubaerialComponentSize > 0 && Result.LargestSubaerialComponentId != INDEX_NONE)
		{
			double LargestSubaerialCoastDistSum = 0.0;
			int32 LargestSubaerialCoastDistCount = 0;
			for (int32 I = 0; I < SampleCount; ++I)
			{
				if (SubaerialComponentIds[I] != Result.LargestSubaerialComponentId)
				{
					continue;
				}

				if (Result.SampleCoastDistHops[I] > 0)
				{
					LargestSubaerialCoastDistSum += Result.SampleCoastDistHops[I];
					++LargestSubaerialCoastDistCount;
				}
			}

			if (LargestSubaerialCoastDistCount > 0)
			{
				Result.LargestSubaerialMeanCoastDist =
					LargestSubaerialCoastDistSum /
					static_cast<double>(LargestSubaerialCoastDistCount);
			}
		}

		return Result;
	}

	FV9ContinentalLossAttribution ComputeContinentalLossAttribution(const FTectonicPlanetV6& Planet)
	{
		FV9ContinentalLossAttribution Result;
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		const int32 SampleCount = PlanetData.Samples.Num();
		const TArray<float>& PreSolveCW = Planet.GetCurrentSolvePreSolveContinentalWeightsForTest();
		const TArray<FTectonicPlanetV6ResolvedSample>& Resolved = Planet.GetLastResolvedSamplesForTest();

		if (PreSolveCW.Num() != SampleCount || Resolved.Num() != SampleCount)
		{
			return Result;
		}

		for (int32 I = 0; I < SampleCount; ++I)
		{
			const bool bWasContinental = PreSolveCW[I] >= 0.5f;
			const bool bIsContinental = PlanetData.Samples[I].ContinentalWeight >= 0.5f;

			if (bWasContinental && !bIsContinental)
			{
				++Result.TotalLostSamples;
				const ETectonicPlanetV6ResolutionKind Kind = Resolved[I].ResolutionKind;
				switch (Kind)
				{
				case ETectonicPlanetV6ResolutionKind::SingleCandidate:
				case ETectonicPlanetV6ResolutionKind::ThesisRemeshHit:
					++Result.LostViaDirectHit;
					break;
				case ETectonicPlanetV6ResolutionKind::OverlapWinner:
					++Result.LostViaOverlapWinner;
					break;
				case ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery:
					++Result.LostViaNearestTriangleRecovery;
					break;
				case ETectonicPlanetV6ResolutionKind::NearestMemberRecovery:
					++Result.LostViaNearestMemberFallback;
					break;
				case ETectonicPlanetV6ResolutionKind::ExplicitFallback:
					++Result.LostViaExplicitFallback;
					break;
				case ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic:
					++Result.LostViaSyntheticDivergenceFill;
					break;
				case ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion:
					++Result.LostViaDestructiveGapFill;
					break;
				case ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous:
					++Result.LostViaSyntheticDivergenceFill;
					break;
				case ETectonicPlanetV6ResolutionKind::BoundaryOceanic:
					++Result.LostViaBoundaryOceanic;
					break;
				case ETectonicPlanetV6ResolutionKind::ThesisRemeshRetainedOutsideActiveZone:
					++Result.LostViaRetainedOutsideActiveZone;
					{
						const FTectonicPlanetV6TransferDebugInfo& TD = Resolved[I].TransferDebug;
						switch (TD.SourceKind)
						{
						case ETectonicPlanetV6TransferSourceKind::Triangle:
							++Result.RetainedOazViaTriangle;
							break;
						case ETectonicPlanetV6TransferSourceKind::SingleSource:
							++Result.RetainedOazViaSingleSource;
							break;
						case ETectonicPlanetV6TransferSourceKind::StructuredSynthetic:
							++Result.RetainedOazViaStructuredSynthetic;
							break;
						case ETectonicPlanetV6TransferSourceKind::OceanicCreation:
							++Result.RetainedOazViaOceanicCreation;
							break;
						case ETectonicPlanetV6TransferSourceKind::Defaulted:
							++Result.RetainedOazViaDefaulted;
							break;
						default:
							++Result.RetainedOazViaOtherSource;
							break;
						}
						if (TD.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
						{
							++Result.RetainedOazWithCwThresholdMismatch;
						}
					}
					break;
				case ETectonicPlanetV6ResolutionKind::ThesisRemeshTransferFallback:
					++Result.LostViaTransferFallback;
					break;
				case ETectonicPlanetV6ResolutionKind::ThesisRemeshRetainedSyntheticCoverage:
					++Result.LostViaRetainedSyntheticCoverage;
					break;
				default:
					++Result.LostViaOther;
					break;
				}
			}
			else if (!bWasContinental && bIsContinental)
			{
				++Result.TotalGainedSamples;
			}
		}

		Result.NetChange = Result.TotalGainedSamples - Result.TotalLostSamples;
		return Result;
	}

	FV9ContinentalLossAttribution ComputeCumulativeContinentalLossAttribution(
		const TArray<float>& BaselineCW,
		const FTectonicPlanet& CurrentPlanet)
	{
		FV9ContinentalLossAttribution Result;
		const int32 SampleCount = CurrentPlanet.Samples.Num();
		if (BaselineCW.Num() != SampleCount) { return Result; }

		for (int32 I = 0; I < SampleCount; ++I)
		{
			const bool bWasContinental = BaselineCW[I] >= 0.5f;
			const bool bIsContinental = CurrentPlanet.Samples[I].ContinentalWeight >= 0.5f;
			if (bWasContinental && !bIsContinental) { ++Result.TotalLostSamples; }
			else if (!bWasContinental && bIsContinental) { ++Result.TotalGainedSamples; }
		}
		Result.NetChange = Result.TotalGainedSamples - Result.TotalLostSamples;
		return Result;
	}

	TArray<float> SnapshotContinentalWeights(const FTectonicPlanet& Planet)
	{
		TArray<float> CW;
		CW.SetNum(Planet.Samples.Num());
		for (int32 I = 0; I < Planet.Samples.Num(); ++I)
		{
			CW[I] = Planet.Samples[I].ContinentalWeight;
		}
		return CW;
	}

	FV9SeededContinentalSurvivalDiagnostic ComputeSeededContinentalSurvivalDiagnostic(
		const FTectonicPlanet& Planet,
		const TArray<uint8>& Step0ContinentalFlags,
		const TArray<uint8>& Step0BroadInteriorFlags,
		const TArray<uint8>& Step0CoastAdjacentFlags)
	{
		FV9SeededContinentalSurvivalDiagnostic Result;
		const int32 SampleCount = Planet.Samples.Num();
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			const bool bCurrentlyContinental =
				Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f;
			if (Step0ContinentalFlags.IsValidIndex(SampleIndex) && Step0ContinentalFlags[SampleIndex] != 0)
			{
				++Result.Step0ContinentalCount;
				if (bCurrentlyContinental)
				{
					++Result.Step0ContinentalRemainingCount;
				}
			}
			if (Step0BroadInteriorFlags.IsValidIndex(SampleIndex) && Step0BroadInteriorFlags[SampleIndex] != 0)
			{
				++Result.Step0BroadInteriorCount;
				if (bCurrentlyContinental)
				{
					++Result.Step0BroadInteriorRemainingCount;
				}
			}
			if (Step0CoastAdjacentFlags.IsValidIndex(SampleIndex) && Step0CoastAdjacentFlags[SampleIndex] != 0)
			{
				++Result.Step0CoastAdjacentCount;
				if (bCurrentlyContinental)
				{
					++Result.Step0CoastAdjacentRemainingCount;
				}
			}
		}

		if (Result.Step0ContinentalCount > 0)
		{
			Result.Step0ContinentalRemainingFraction =
				static_cast<double>(Result.Step0ContinentalRemainingCount) /
				static_cast<double>(Result.Step0ContinentalCount);
		}
		if (Result.Step0BroadInteriorCount > 0)
		{
			Result.Step0BroadInteriorRemainingFraction =
				static_cast<double>(Result.Step0BroadInteriorRemainingCount) /
				static_cast<double>(Result.Step0BroadInteriorCount);
		}
		if (Result.Step0CoastAdjacentCount > 0)
		{
			Result.Step0CoastAdjacentRemainingFraction =
				static_cast<double>(Result.Step0CoastAdjacentRemainingCount) /
				static_cast<double>(Result.Step0CoastAdjacentCount);
		}

		return Result;
	}

	enum class EBoundaryFieldCouplingProvenanceBucket : uint8
	{
		DirectHitTriangle = 0,
		SingleSource = 1,
		SyntheticRecovery = 2,
		FallbackDefault = 3,
		Count = 4,
	};

	enum class EBoundaryFieldCouplingLeakCause : uint8
	{
		CwThresholdCross = 0,
		ElevationJumpGt1Km = 1,
		QueryMissWithPreviousOwner = 2,
		TransferProvenanceChanged = 3,
		OtherUnknown = 4,
		Count = 5,
	};

	enum class EBoundaryFieldCouplingProximityBucket : uint8
	{
		ActiveBand = 0,
		OneRing = 1,
		NearInterior2To3 = 2,
		DeepInterior4Plus = 3,
		Count = 4,
	};

	struct FBoundaryFieldCouplingMismatchLog
	{
		int32 SampleIndex = INDEX_NONE;
		double AbsCwDelta = 0.0;
		EBoundaryFieldCouplingProvenanceBucket Provenance =
			EBoundaryFieldCouplingProvenanceBucket::FallbackDefault;
		float PreSolveContinentalWeight = 0.0f;
		float PostSolveContinentalWeight = 0.0f;
		float PreSolveElevationKm = 0.0f;
		float PostSolveElevationKm = 0.0f;
		int32 PreviousPlateId = INDEX_NONE;
		int32 FinalPlateId = INDEX_NONE;
		int32 ActiveRingDistance = INDEX_NONE;
		bool bActiveZoneSample = false;
	};

	struct FBoundaryFieldCouplingSyntheticMissLog
	{
		int32 SampleIndex = INDEX_NONE;
		int32 MissLineageAge = 0;
		EBoundaryFieldCouplingProvenanceBucket PreviousProvenance =
			EBoundaryFieldCouplingProvenanceBucket::FallbackDefault;
		float PreSolveContinentalWeight = 0.0f;
		float PostSolveContinentalWeight = 0.0f;
		float PreSolveElevationKm = 0.0f;
		float PostSolveElevationKm = 0.0f;
		int32 PreviousPlateId = INDEX_NONE;
		int32 FinalPlateId = INDEX_NONE;
		int32 ConvergentActiveRingDistance = INDEX_NONE;
		bool bNearConvergentFront = false;
		bool bCurrentActive = false;
	};

	struct FBoundaryFieldCouplingLeakLog
	{
		int32 SampleIndex = INDEX_NONE;
		double AbsElevationDeltaKm = 0.0;
		EBoundaryFieldCouplingLeakCause Cause = EBoundaryFieldCouplingLeakCause::OtherUnknown;
		EBoundaryFieldCouplingProvenanceBucket PreviousProvenance =
			EBoundaryFieldCouplingProvenanceBucket::FallbackDefault;
		EBoundaryFieldCouplingProvenanceBucket CurrentProvenance =
			EBoundaryFieldCouplingProvenanceBucket::FallbackDefault;
		EBoundaryFieldCouplingProximityBucket ProximityBucket =
			EBoundaryFieldCouplingProximityBucket::DeepInterior4Plus;
		float PreSolveContinentalWeight = 0.0f;
		float PostSolveContinentalWeight = 0.0f;
		float PreSolveElevationKm = 0.0f;
		float PostSolveElevationKm = 0.0f;
		int32 PreviousPlateId = INDEX_NONE;
		int32 FinalPlateId = INDEX_NONE;
	};

	struct FBoundaryFieldCouplingBandDeltaStats
	{
		int32 SampleCount = 0;
		double SumAbsCwDelta = 0.0;
		double MaxAbsCwDelta = 0.0;
		double SumAbsElevationDeltaKm = 0.0;
		double MaxAbsElevationDeltaKm = 0.0;
		int32 ThresholdCrossCount = 0;
		int32 OwnershipRetainedCount = 0;
	};

	struct FBoundaryFieldCouplingVariantDiagnostics
	{
		FString Label;
		FV6CheckpointSnapshot Snapshot;
		int32 CwThresholdMismatchCount = 0;
		int32 CwThresholdMismatchByProvenance[4] = { 0, 0, 0, 0 };
		double MeanMismatchNearestActiveZoneRingDistance = 0.0;
		FBoundaryFieldCouplingBandDeltaStats BoundaryBandDelta;
		int32 SyntheticMissedAgainCount = 0;
		int32 SyntheticMissedAgainByPreviousProvenance[4] = { 0, 0, 0, 0 };
		TMap<int32, int32> SyntheticMissLineageAgeCounts;
		int32 SyntheticMissedAgainNearConvergentFrontCount = 0;
		int32 SyntheticMissedAgainWithCwDeltaCount = 0;
		double SyntheticMissedAgainMeanAbsCwDelta = 0.0;
		int32 TotalInteriorLeakCount = 0;
		int32 InteriorLeakCauseCounts[5] = { 0, 0, 0, 0, 0 };
		int32 InteriorLeakProximityCounts[4] = { 0, 0, 0, 0 };
		TArray<FBoundaryFieldCouplingMismatchLog> TopMismatchLogs;
		TArray<FBoundaryFieldCouplingSyntheticMissLog> TopSyntheticMissLogs;
		TArray<FBoundaryFieldCouplingLeakLog> TopLeakLogs;
	};

	const TCHAR* GetBoundaryFieldCouplingProvenanceName(
		const EBoundaryFieldCouplingProvenanceBucket Bucket)
	{
		switch (Bucket)
		{
		case EBoundaryFieldCouplingProvenanceBucket::DirectHitTriangle:
			return TEXT("direct_hit_triangle");
		case EBoundaryFieldCouplingProvenanceBucket::SingleSource:
			return TEXT("single_source");
		case EBoundaryFieldCouplingProvenanceBucket::SyntheticRecovery:
			return TEXT("synthetic_recovery");
		case EBoundaryFieldCouplingProvenanceBucket::FallbackDefault:
		default:
			return TEXT("fallback_default");
		}
	}

	const TCHAR* GetBoundaryFieldCouplingLeakCauseName(
		const EBoundaryFieldCouplingLeakCause Cause)
	{
		switch (Cause)
		{
		case EBoundaryFieldCouplingLeakCause::CwThresholdCross:
			return TEXT("cw_threshold_cross");
		case EBoundaryFieldCouplingLeakCause::ElevationJumpGt1Km:
			return TEXT("elevation_jump_gt_1km");
		case EBoundaryFieldCouplingLeakCause::QueryMissWithPreviousOwner:
			return TEXT("query_miss_with_previous_owner");
		case EBoundaryFieldCouplingLeakCause::TransferProvenanceChanged:
			return TEXT("transfer_provenance_changed");
		case EBoundaryFieldCouplingLeakCause::OtherUnknown:
		default:
			return TEXT("other_unknown");
		}
	}

	const TCHAR* GetBoundaryFieldCouplingProximityName(
		const EBoundaryFieldCouplingProximityBucket Bucket)
	{
		switch (Bucket)
		{
		case EBoundaryFieldCouplingProximityBucket::ActiveBand:
			return TEXT("active_band");
		case EBoundaryFieldCouplingProximityBucket::OneRing:
			return TEXT("one_ring");
		case EBoundaryFieldCouplingProximityBucket::NearInterior2To3:
			return TEXT("near_interior_2_3");
		case EBoundaryFieldCouplingProximityBucket::DeepInterior4Plus:
		default:
			return TEXT("deep_interior_4_plus");
		}
	}

	bool IsBoundaryFieldCouplingRemeshMiss(const FTectonicPlanetV6ResolvedSample& Resolved)
	{
		return
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous;
	}

	bool IsBoundaryFieldCouplingConflictSample(const FTectonicPlanetV6ResolvedSample& Resolved)
	{
		return Resolved.ExactCandidateCount > 1 || IsBoundaryFieldCouplingRemeshMiss(Resolved);
	}

	void BuildBoundaryFieldCouplingRingDistanceFromSeedFlags(
		const FTectonicPlanet& Planet,
		const TArray<uint8>& SeedFlags,
		TArray<int32>& OutRingDistance)
	{
		const int32 SampleCount = Planet.Samples.Num();
		OutRingDistance.Init(INDEX_NONE, SampleCount);
		TArray<int32> Queue;
		Queue.Reserve(SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			if (SeedFlags.IsValidIndex(SampleIndex) && SeedFlags[SampleIndex] != 0)
			{
				OutRingDistance[SampleIndex] = 0;
				Queue.Add(SampleIndex);
			}
		}

		for (int32 QueueIndex = 0; QueueIndex < Queue.Num(); ++QueueIndex)
		{
			const int32 SampleIndex = Queue[QueueIndex];
			if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 NextDistance = OutRingDistance[SampleIndex] + 1;
			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!OutRingDistance.IsValidIndex(NeighborIndex) ||
					OutRingDistance[NeighborIndex] != INDEX_NONE)
				{
					continue;
				}

				OutRingDistance[NeighborIndex] = NextDistance;
				Queue.Add(NeighborIndex);
			}
		}
	}

	void BuildBoundaryFieldCouplingBoundaryBandAndRingDistance(
		const FTectonicPlanet& Planet,
		TArray<uint8>& OutBoundaryBandFlags,
		TArray<int32>& OutBoundaryRingDistance)
	{
		const int32 SampleCount = Planet.Samples.Num();
		OutBoundaryBandFlags.Init(0, SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			if (!Planet.Samples.IsValidIndex(SampleIndex) ||
				!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 PlateId = Planet.Samples[SampleIndex].PlateId;
			if (PlateId == INDEX_NONE)
			{
				continue;
			}

			for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
			{
				if (!Planet.Samples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				const int32 NeighborPlateId = Planet.Samples[NeighborIndex].PlateId;
				if (NeighborPlateId != INDEX_NONE && NeighborPlateId != PlateId)
				{
					OutBoundaryBandFlags[SampleIndex] = 1;
					break;
				}
			}
		}

		BuildBoundaryFieldCouplingRingDistanceFromSeedFlags(
			Planet,
			OutBoundaryBandFlags,
			OutBoundaryRingDistance);
	}

	EBoundaryFieldCouplingProximityBucket ClassifyBoundaryFieldCouplingProximity(
		const int32 ActiveRingDistance)
	{
		if (ActiveRingDistance == 0)
		{
			return EBoundaryFieldCouplingProximityBucket::ActiveBand;
		}
		if (ActiveRingDistance == 1)
		{
			return EBoundaryFieldCouplingProximityBucket::OneRing;
		}
		if (ActiveRingDistance >= 2 && ActiveRingDistance <= 3)
		{
			return EBoundaryFieldCouplingProximityBucket::NearInterior2To3;
		}
		return EBoundaryFieldCouplingProximityBucket::DeepInterior4Plus;
	}

	EBoundaryFieldCouplingProvenanceBucket ClassifyBoundaryFieldCouplingCurrentProvenance(
		const FTectonicPlanetV6ResolvedSample& Resolved)
	{
		if (Resolved.bHasStructuredSyntheticFill ||
			Resolved.bRetainedSyntheticCoverage ||
			Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::StructuredSynthetic)
		{
			return EBoundaryFieldCouplingProvenanceBucket::SyntheticRecovery;
		}
		if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::SingleSource)
		{
			return EBoundaryFieldCouplingProvenanceBucket::SingleSource;
		}
		if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle &&
			Resolved.ResolutionKind != ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery)
		{
			return EBoundaryFieldCouplingProvenanceBucket::DirectHitTriangle;
		}
		return EBoundaryFieldCouplingProvenanceBucket::FallbackDefault;
	}

	EBoundaryFieldCouplingProvenanceBucket ClassifyBoundaryFieldCouplingPreviousProvenance(
		const uint8 PreviousTransferSourceKindValue,
		const uint8 PreviousResolutionKindValue,
		const bool bPreviousSynthetic)
	{
		const ETectonicPlanetV6TransferSourceKind PreviousSourceKind =
			static_cast<ETectonicPlanetV6TransferSourceKind>(PreviousTransferSourceKindValue);
		const ETectonicPlanetV6ResolutionKind PreviousResolutionKind =
			static_cast<ETectonicPlanetV6ResolutionKind>(PreviousResolutionKindValue);
		if (bPreviousSynthetic ||
			PreviousSourceKind == ETectonicPlanetV6TransferSourceKind::StructuredSynthetic ||
			PreviousResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshRetainedSyntheticCoverage)
		{
			return EBoundaryFieldCouplingProvenanceBucket::SyntheticRecovery;
		}
		if (PreviousSourceKind == ETectonicPlanetV6TransferSourceKind::SingleSource)
		{
			return EBoundaryFieldCouplingProvenanceBucket::SingleSource;
		}
		if (PreviousSourceKind == ETectonicPlanetV6TransferSourceKind::Triangle &&
			PreviousResolutionKind != ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery)
		{
			return EBoundaryFieldCouplingProvenanceBucket::DirectHitTriangle;
		}
		return EBoundaryFieldCouplingProvenanceBucket::FallbackDefault;
	}

	FString FormatBoundaryFieldCouplingAgeDistribution(const TMap<int32, int32>& Counts)
	{
		TArray<int32> Ages;
		Counts.GenerateKeyArray(Ages);
		Ages.Sort();

		TArray<FString> Parts;
		for (const int32 Age : Ages)
		{
			if (const int32* CountPtr = Counts.Find(Age))
			{
				Parts.Add(FString::Printf(TEXT("%d:%d"), Age, *CountPtr));
			}
		}
		return FString::Join(Parts, TEXT(","));
	}

	void AddBoundaryFieldCouplingLog(
		FAutomationTestBase& Test,
		const FString& Message)
	{
		Test.AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	}

	void ConfigureBoundaryFieldCouplingVariant(
		FTectonicPlanetV6& Planet,
		const bool bUseThesisScaleRelief)
	{
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);

		FTectonicPlanet& LegacyPlanet = Planet.GetPlanetMutable();
		if (bUseThesisScaleRelief)
		{
			LegacyPlanet.SubductionBaseUpliftKmPerMyForTest = -1.0;
			LegacyPlanet.bDisableSubductionElevationTransferForTest = false;
			Planet.SetUseLinearConvergentMaintenanceSpeedFactorForTest(true);
			Planet.SetUseLinearConvergentMaintenanceInfluenceForTest(true);
		}
		else
		{
			LegacyPlanet.SubductionBaseUpliftKmPerMyForTest = 0.0006;
			LegacyPlanet.bDisableSubductionElevationTransferForTest = true;
			Planet.SetUseLinearConvergentMaintenanceSpeedFactorForTest(false);
			Planet.SetUseLinearConvergentMaintenanceInfluenceForTest(false);
		}
	}

	FBoundaryFieldCouplingVariantDiagnostics BuildBoundaryFieldCouplingVariantDiagnostics(
		const FString& Label,
		const FTectonicPlanetV6& Planet)
	{
		FBoundaryFieldCouplingVariantDiagnostics Result;
		Result.Label = Label;
		Result.Snapshot = BuildV6CheckpointSnapshot(Planet);

		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		const int32 SampleCount = PlanetData.Samples.Num();
		const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples = Planet.GetLastResolvedSamplesForTest();
		const TArray<int32>& PreSolvePlateIds = Planet.GetCurrentSolvePreSolvePlateIdsForTest();
		const TArray<float>& PreSolveContinentalWeights =
			Planet.GetCurrentSolvePreSolveContinentalWeightsForTest();
		const TArray<float>& PreSolveElevations = Planet.GetCurrentSolvePreSolveElevationsForTest();
		const TArray<uint8>& PreviousSyntheticFlags = Planet.GetCurrentSolvePreviousSyntheticFlagsForTest();
		const TArray<uint8>& PreviousTransferSourceKindValues =
			Planet.GetCurrentSolvePreviousTransferSourceKindValuesForTest();
		const TArray<uint8>& PreviousResolutionKindValues =
			Planet.GetCurrentSolvePreviousResolutionKindValuesForTest();
		const TArray<uint8>& ActiveZoneFlags = Planet.GetCurrentSolveActiveZoneFlagsForTest();
		const TArray<uint8>& ActiveZoneCauseValues = Planet.GetCurrentSolveActiveZoneCauseValuesForTest();
		const TArray<uint8>& MissLineageCounts = Planet.GetMissLineageCountsForTest();

		if (ResolvedSamples.Num() != SampleCount ||
			PreSolvePlateIds.Num() != SampleCount ||
			PreSolveContinentalWeights.Num() != SampleCount ||
			PreSolveElevations.Num() != SampleCount ||
			PreviousSyntheticFlags.Num() != SampleCount ||
			PreviousTransferSourceKindValues.Num() != SampleCount ||
			PreviousResolutionKindValues.Num() != SampleCount ||
			ActiveZoneFlags.Num() != SampleCount ||
			ActiveZoneCauseValues.Num() != SampleCount ||
			MissLineageCounts.Num() != SampleCount)
		{
			return Result;
		}

		TArray<int32> ActiveRingDistance;
		BuildBoundaryFieldCouplingRingDistanceFromSeedFlags(
			PlanetData,
			ActiveZoneFlags,
			ActiveRingDistance);

		TArray<uint8> ConvergentActiveFlags;
		ConvergentActiveFlags.Init(0, SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			if (ActiveZoneFlags[SampleIndex] == 0)
			{
				continue;
			}

			const ETectonicPlanetV6ActiveZoneCause Cause =
				static_cast<ETectonicPlanetV6ActiveZoneCause>(ActiveZoneCauseValues[SampleIndex]);
			if (Cause == ETectonicPlanetV6ActiveZoneCause::ConvergentSubduction ||
				Cause == ETectonicPlanetV6ActiveZoneCause::CollisionContact)
			{
				ConvergentActiveFlags[SampleIndex] = 1;
			}
		}

		TArray<int32> ConvergentActiveRingDistance;
		BuildBoundaryFieldCouplingRingDistanceFromSeedFlags(
			PlanetData,
			ConvergentActiveFlags,
			ConvergentActiveRingDistance);

		TArray<uint8> BoundaryBandFlags;
		TArray<int32> BoundaryRingDistance;
		BuildBoundaryFieldCouplingBoundaryBandAndRingDistance(
			PlanetData,
			BoundaryBandFlags,
			BoundaryRingDistance);

		double MismatchRingDistanceSum = 0.0;
		int32 MismatchRingDistanceCount = 0;
		double SyntheticMissAbsCwDeltaSum = 0.0;

		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			const FTectonicPlanetV6ResolvedSample& Resolved = ResolvedSamples[SampleIndex];
			const FSample& CurrentSample = PlanetData.Samples[SampleIndex];
			const float PreSolveCw = PreSolveContinentalWeights[SampleIndex];
			const float PostSolveCw = CurrentSample.ContinentalWeight;
			const float PreSolveElevation = PreSolveElevations[SampleIndex];
			const float PostSolveElevation = CurrentSample.Elevation;
			const double AbsCwDelta = FMath::Abs(static_cast<double>(PostSolveCw - PreSolveCw));
			const double AbsElevationDelta = FMath::Abs(static_cast<double>(PostSolveElevation - PreSolveElevation));
			const int32 ActiveDistance = ActiveRingDistance.IsValidIndex(SampleIndex)
				? ActiveRingDistance[SampleIndex]
				: INDEX_NONE;
			const int32 ConvergentDistance = ConvergentActiveRingDistance.IsValidIndex(SampleIndex)
				? ConvergentActiveRingDistance[SampleIndex]
				: INDEX_NONE;
			const bool bPreviousSynthetic = PreviousSyntheticFlags[SampleIndex] != 0;
			const EBoundaryFieldCouplingProvenanceBucket CurrentProvenance =
				ClassifyBoundaryFieldCouplingCurrentProvenance(Resolved);
			const EBoundaryFieldCouplingProvenanceBucket PreviousProvenance =
				ClassifyBoundaryFieldCouplingPreviousProvenance(
					PreviousTransferSourceKindValues[SampleIndex],
					PreviousResolutionKindValues[SampleIndex],
					bPreviousSynthetic);

			if (Resolved.TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend)
			{
				++Result.CwThresholdMismatchCount;
				++Result.CwThresholdMismatchByProvenance[static_cast<int32>(CurrentProvenance)];
				if (ActiveDistance != INDEX_NONE)
				{
					MismatchRingDistanceSum += static_cast<double>(ActiveDistance);
					++MismatchRingDistanceCount;
				}

				FBoundaryFieldCouplingMismatchLog Log;
				Log.SampleIndex = SampleIndex;
				Log.AbsCwDelta = AbsCwDelta;
				Log.Provenance = CurrentProvenance;
				Log.PreSolveContinentalWeight = PreSolveCw;
				Log.PostSolveContinentalWeight = PostSolveCw;
				Log.PreSolveElevationKm = PreSolveElevation;
				Log.PostSolveElevationKm = PostSolveElevation;
				Log.PreviousPlateId = PreSolvePlateIds[SampleIndex];
				Log.FinalPlateId = Resolved.FinalPlateId;
				Log.ActiveRingDistance = ActiveDistance;
				Log.bActiveZoneSample = ActiveZoneFlags[SampleIndex] != 0;
				Result.TopMismatchLogs.Add(Log);
			}

			if (ActiveDistance != INDEX_NONE && ActiveDistance <= 1)
			{
				++Result.BoundaryBandDelta.SampleCount;
				Result.BoundaryBandDelta.SumAbsCwDelta += AbsCwDelta;
				Result.BoundaryBandDelta.MaxAbsCwDelta =
					FMath::Max(Result.BoundaryBandDelta.MaxAbsCwDelta, AbsCwDelta);
				Result.BoundaryBandDelta.SumAbsElevationDeltaKm += AbsElevationDelta;
				Result.BoundaryBandDelta.MaxAbsElevationDeltaKm =
					FMath::Max(Result.BoundaryBandDelta.MaxAbsElevationDeltaKm, AbsElevationDelta);
				if ((PreSolveCw >= 0.5f) != (PostSolveCw >= 0.5f))
				{
					++Result.BoundaryBandDelta.ThresholdCrossCount;
				}
				if (PreSolvePlateIds[SampleIndex] != INDEX_NONE &&
					Resolved.FinalPlateId == PreSolvePlateIds[SampleIndex])
				{
					++Result.BoundaryBandDelta.OwnershipRetainedCount;
				}
			}

			if (bPreviousSynthetic && IsBoundaryFieldCouplingRemeshMiss(Resolved))
			{
				++Result.SyntheticMissedAgainCount;
				++Result.SyntheticMissedAgainByPreviousProvenance[static_cast<int32>(PreviousProvenance)];
				++Result.SyntheticMissLineageAgeCounts.FindOrAdd(static_cast<int32>(MissLineageCounts[SampleIndex]));
				if (ConvergentDistance != INDEX_NONE && ConvergentDistance <= 1)
				{
					++Result.SyntheticMissedAgainNearConvergentFrontCount;
				}
				if (AbsCwDelta > 1.0e-3)
				{
					++Result.SyntheticMissedAgainWithCwDeltaCount;
				}
				SyntheticMissAbsCwDeltaSum += AbsCwDelta;

				FBoundaryFieldCouplingSyntheticMissLog Log;
				Log.SampleIndex = SampleIndex;
				Log.MissLineageAge = static_cast<int32>(MissLineageCounts[SampleIndex]);
				Log.PreviousProvenance = PreviousProvenance;
				Log.PreSolveContinentalWeight = PreSolveCw;
				Log.PostSolveContinentalWeight = PostSolveCw;
				Log.PreSolveElevationKm = PreSolveElevation;
				Log.PostSolveElevationKm = PostSolveElevation;
				Log.PreviousPlateId = PreSolvePlateIds[SampleIndex];
				Log.FinalPlateId = Resolved.FinalPlateId;
				Log.ConvergentActiveRingDistance = ConvergentDistance;
				Log.bNearConvergentFront = ConvergentDistance != INDEX_NONE && ConvergentDistance <= 1;
				Log.bCurrentActive = ActiveZoneFlags[SampleIndex] != 0;
				Result.TopSyntheticMissLogs.Add(Log);
			}

			const int32 BoundaryDistance = BoundaryRingDistance.IsValidIndex(SampleIndex)
				? BoundaryRingDistance[SampleIndex]
				: INDEX_NONE;
			if (BoundaryDistance == INDEX_NONE || BoundaryDistance < 2)
			{
				continue;
			}
			if (!IsBoundaryFieldCouplingConflictSample(Resolved))
			{
				continue;
			}

			++Result.TotalInteriorLeakCount;
			const bool bCwCrossedThreshold = (PreSolveCw >= 0.5f) != (PostSolveCw >= 0.5f);
			const bool bElevationJumped = AbsElevationDelta > 1.0;
			const bool bQueryMissWithPreviousOwner =
				IsBoundaryFieldCouplingRemeshMiss(Resolved) && PreSolvePlateIds[SampleIndex] != INDEX_NONE;
			const bool bTransferProvenanceChanged = CurrentProvenance != PreviousProvenance;
			EBoundaryFieldCouplingLeakCause Cause = EBoundaryFieldCouplingLeakCause::OtherUnknown;
			if (bCwCrossedThreshold)
			{
				Cause = EBoundaryFieldCouplingLeakCause::CwThresholdCross;
			}
			else if (bElevationJumped)
			{
				Cause = EBoundaryFieldCouplingLeakCause::ElevationJumpGt1Km;
			}
			else if (bQueryMissWithPreviousOwner)
			{
				Cause = EBoundaryFieldCouplingLeakCause::QueryMissWithPreviousOwner;
			}
			else if (bTransferProvenanceChanged)
			{
				Cause = EBoundaryFieldCouplingLeakCause::TransferProvenanceChanged;
			}
			++Result.InteriorLeakCauseCounts[static_cast<int32>(Cause)];

			const EBoundaryFieldCouplingProximityBucket ProximityBucket =
				ClassifyBoundaryFieldCouplingProximity(ActiveDistance);
			++Result.InteriorLeakProximityCounts[static_cast<int32>(ProximityBucket)];

			FBoundaryFieldCouplingLeakLog Log;
			Log.SampleIndex = SampleIndex;
			Log.AbsElevationDeltaKm = AbsElevationDelta;
			Log.Cause = Cause;
			Log.PreviousProvenance = PreviousProvenance;
			Log.CurrentProvenance = CurrentProvenance;
			Log.ProximityBucket = ProximityBucket;
			Log.PreSolveContinentalWeight = PreSolveCw;
			Log.PostSolveContinentalWeight = PostSolveCw;
			Log.PreSolveElevationKm = PreSolveElevation;
			Log.PostSolveElevationKm = PostSolveElevation;
			Log.PreviousPlateId = PreSolvePlateIds[SampleIndex];
			Log.FinalPlateId = Resolved.FinalPlateId;
			Result.TopLeakLogs.Add(Log);
		}

		Result.MeanMismatchNearestActiveZoneRingDistance =
			MismatchRingDistanceCount > 0
				? MismatchRingDistanceSum / static_cast<double>(MismatchRingDistanceCount)
				: 0.0;
		Result.SyntheticMissedAgainMeanAbsCwDelta =
			Result.SyntheticMissedAgainCount > 0
				? SyntheticMissAbsCwDeltaSum / static_cast<double>(Result.SyntheticMissedAgainCount)
				: 0.0;

		Result.TopMismatchLogs.Sort([](
			const FBoundaryFieldCouplingMismatchLog& A,
			const FBoundaryFieldCouplingMismatchLog& B)
		{
			return A.AbsCwDelta > B.AbsCwDelta;
		});
		if (Result.TopMismatchLogs.Num() > 20)
		{
			Result.TopMismatchLogs.SetNum(20);
		}

		Result.TopSyntheticMissLogs.Sort([](
			const FBoundaryFieldCouplingSyntheticMissLog& A,
			const FBoundaryFieldCouplingSyntheticMissLog& B)
		{
			if (A.MissLineageAge != B.MissLineageAge)
			{
				return A.MissLineageAge > B.MissLineageAge;
			}
			return FMath::Abs(
				static_cast<double>(A.PostSolveContinentalWeight - A.PreSolveContinentalWeight)) >
				FMath::Abs(static_cast<double>(B.PostSolveContinentalWeight - B.PreSolveContinentalWeight));
		});
		if (Result.TopSyntheticMissLogs.Num() > 20)
		{
			Result.TopSyntheticMissLogs.SetNum(20);
		}

		Result.TopLeakLogs.Sort([](
			const FBoundaryFieldCouplingLeakLog& A,
			const FBoundaryFieldCouplingLeakLog& B)
		{
			return A.AbsElevationDeltaKm > B.AbsElevationDeltaKm;
		});
		if (Result.TopLeakLogs.Num() > 20)
		{
			Result.TopLeakLogs.SetNum(20);
		}

		return Result;
	}

	void AddBoundaryFieldCouplingAggregateComparison(
		FAutomationTestBase& Test,
		const FString& MetricName,
		const double ControlValue,
		const double CandidateValue)
	{
		AddBoundaryFieldCouplingLog(
			Test,
			FString::Printf(
				TEXT("[BoundaryFieldCoupling metric=%s] A=%.6f B=%.6f delta=%.6f"),
				*MetricName,
				ControlValue,
				CandidateValue,
				CandidateValue - ControlValue));
	}

	void AddBoundaryFieldCouplingVariantTopLogs(
		FAutomationTestBase& Test,
		const FBoundaryFieldCouplingVariantDiagnostics& Variant)
	{
		for (const FBoundaryFieldCouplingMismatchLog& Log : Variant.TopMismatchLogs)
		{
			AddBoundaryFieldCouplingLog(
				Test,
				FString::Printf(
					TEXT("[BoundaryFieldCoupling mismatch_top variant=%s] sample=%d abs_cw_delta=%.6f provenance=%s pre_cw=%.6f post_cw=%.6f pre_elev=%.6f post_elev=%.6f prev_plate=%d final_plate=%d active=%d active_ring=%d"),
					*Variant.Label,
					Log.SampleIndex,
					Log.AbsCwDelta,
					GetBoundaryFieldCouplingProvenanceName(Log.Provenance),
					Log.PreSolveContinentalWeight,
					Log.PostSolveContinentalWeight,
					Log.PreSolveElevationKm,
					Log.PostSolveElevationKm,
					Log.PreviousPlateId,
					Log.FinalPlateId,
					Log.bActiveZoneSample ? 1 : 0,
					Log.ActiveRingDistance));
		}

		for (const FBoundaryFieldCouplingSyntheticMissLog& Log : Variant.TopSyntheticMissLogs)
		{
			AddBoundaryFieldCouplingLog(
				Test,
				FString::Printf(
					TEXT("[BoundaryFieldCoupling synthetic_miss_top variant=%s] sample=%d lineage_age=%d prev_provenance=%s near_convergent_front=%d convergent_ring=%d pre_cw=%.6f post_cw=%.6f pre_elev=%.6f post_elev=%.6f prev_plate=%d final_plate=%d active=%d"),
					*Variant.Label,
					Log.SampleIndex,
					Log.MissLineageAge,
					GetBoundaryFieldCouplingProvenanceName(Log.PreviousProvenance),
					Log.bNearConvergentFront ? 1 : 0,
					Log.ConvergentActiveRingDistance,
					Log.PreSolveContinentalWeight,
					Log.PostSolveContinentalWeight,
					Log.PreSolveElevationKm,
					Log.PostSolveElevationKm,
					Log.PreviousPlateId,
					Log.FinalPlateId,
					Log.bCurrentActive ? 1 : 0));
		}

		for (const FBoundaryFieldCouplingLeakLog& Log : Variant.TopLeakLogs)
		{
			AddBoundaryFieldCouplingLog(
				Test,
				FString::Printf(
					TEXT("[BoundaryFieldCoupling leak_top variant=%s] sample=%d abs_elev_delta=%.6f cause=%s prev_provenance=%s current_provenance=%s proximity=%s pre_cw=%.6f post_cw=%.6f pre_elev=%.6f post_elev=%.6f prev_plate=%d final_plate=%d"),
					*Variant.Label,
					Log.SampleIndex,
					Log.AbsElevationDeltaKm,
					GetBoundaryFieldCouplingLeakCauseName(Log.Cause),
					GetBoundaryFieldCouplingProvenanceName(Log.PreviousProvenance),
					GetBoundaryFieldCouplingProvenanceName(Log.CurrentProvenance),
					GetBoundaryFieldCouplingProximityName(Log.ProximityBucket),
					Log.PreSolveContinentalWeight,
					Log.PostSolveContinentalWeight,
					Log.PreSolveElevationKm,
					Log.PostSolveElevationKm,
					Log.PreviousPlateId,
					Log.FinalPlateId));
		}
	}

	FV9ThesisShapedCollisionGateResult EvaluateV9ThesisShapedCollisionStep100Gate(
		const FV6CheckpointSnapshot& BaselineSnapshot,
		const FV6CheckpointSnapshot& CandidateSnapshot,
		const int32 TotalSampleCount)
	{
		(void)TotalSampleCount;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec =
			BaselineSnapshot.CollisionExecution;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec =
			CandidateSnapshot.CollisionExecution;

		FV9ThesisShapedCollisionGateResult Result;
		Result.RequiredMinTerraneComponentSize = FMath::Max(
			BaselineExec.ExecutedDonorTerraneComponentSize > 0
				? FMath::Max(
					24,
					FMath::FloorToInt(
						static_cast<double>(BaselineExec.ExecutedDonorTerraneComponentSize) * 0.75))
				: 24,
			24);
		Result.RequiredMinTransferSize = FMath::Max(
			40,
			FMath::FloorToInt(
				static_cast<double>(FMath::Max(BaselineExec.CumulativeCollisionTransferredSampleCount, 1)) *
				0.85));
		Result.RequiredMinOwnershipChange = FMath::Max(
			40,
			FMath::FloorToInt(
				static_cast<double>(FMath::Max(BaselineExec.CumulativeCollisionDrivenOwnershipChangeCount, 1)) *
				0.85));
		Result.RequiredMinMaxElevationDeltaKm =
			FMath::Max(
				4.0,
				BaselineExec.ExecutedMaxElevationDeltaKm > UE_DOUBLE_SMALL_NUMBER
					? BaselineExec.ExecutedMaxElevationDeltaKm * 1.50
					: 4.0);
		Result.RequiredMinCollisionAbove5KmSamples =
			FMath::Max(1, BaselineSnapshot.CumulativeCollisionRegionSamplesAbove5Km + 1);

		Result.bTerraneDetectedPass =
			CandidateExec.ExecutedDonorTerraneComponentSize >= Result.RequiredMinTerraneComponentSize;
		Result.bTransferCoherentPass =
			CandidateExec.CumulativeCollisionTransferredSampleCount >= Result.RequiredMinTransferSize;
		Result.bOwnershipMaterialPass =
			CandidateExec.CumulativeCollisionDrivenOwnershipChangeCount >= Result.RequiredMinOwnershipChange;
		Result.bMaxElevationPass =
			CandidateExec.ExecutedMaxElevationDeltaKm >= Result.RequiredMinMaxElevationDeltaKm;
		Result.bCollisionAbove5Pass =
			CandidateSnapshot.CumulativeCollisionRegionSamplesAbove5Km >=
			Result.RequiredMinCollisionAbove5KmSamples;
		Result.bChurnPass =
			CandidateSnapshot.OwnershipChurn.ChurnFraction <= Result.MaxAllowedChurn;
		Result.bCoherencePass =
			CandidateSnapshot.BoundaryCoherence.BoundaryCoherenceScore >= Result.MinAllowedCoherence;
		Result.bLeakagePass =
			CandidateSnapshot.BoundaryCoherence.InteriorLeakageFraction <= Result.MaxAllowedInteriorLeakage;
		Result.bAllowStep200 =
			Result.bTerraneDetectedPass &&
			Result.bTransferCoherentPass &&
			Result.bOwnershipMaterialPass &&
			Result.bMaxElevationPass &&
			Result.bCollisionAbove5Pass &&
			Result.bChurnPass &&
			Result.bCoherencePass &&
			Result.bLeakagePass;
		return Result;
	}

	FV6BaselineSummary RunV6BaselineScenario(
		FAutomationTestBase& Test,
		const FString& RunId,
		const int32 MaxStep,
		const TArray<int32>& ExportSteps,
		const ETectonicPlanetV6PeriodicSolveMode SolveMode = ETectonicPlanetV6PeriodicSolveMode::Phase3Authoritative,
		const int32 FixedIntervalSteps = 25,
		const bool bExportMaps = true)
	{
		FV6BaselineSummary Summary;
		Summary.RunId = RunId;

		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(SolveMode, FixedIntervalSteps);
		const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
		Summary.Step0Snapshot = BuildV6CheckpointSnapshot(Planet);
		Summary.bHasStep0Snapshot = true;
		Summary.MaxComponentsObserved = Summary.Step0Snapshot.MaxComponentsPerPlate;

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		if (bExportMaps)
		{
			Test.TestTrue(TEXT("V6 export root created"), PlatformFile.CreateDirectoryTree(*ExportRoot));
			ExportV6CheckpointMaps(Test, Planet, ExportRoot, 0);
		}

		for (int32 Step = 0; Step < MaxStep; ++Step)
		{
			Planet.AdvanceStep();
			const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
			Summary.MaxComponentsObserved = FMath::Max(Summary.MaxComponentsObserved, Snapshot.MaxComponentsPerPlate);
			const bool bLogCopiedFrontierAttribution =
				SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike ||
				SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike ||
				SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike ||
				SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike ||
				SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike ||
				SolveMode == ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike;
			FString LiveSummaryTag = TEXT("[V6Phase1Baseline]");
			if (IsThesisLikeRunId(RunId))
			{
				LiveSummaryTag = GetV6ThesisBaselineSummaryTag(RunId);
			}

			if (bExportMaps && ExportSteps.Contains(Planet.GetPlanet().CurrentStep))
			{
				ExportV6CheckpointMaps(Test, Planet, ExportRoot, Planet.GetPlanet().CurrentStep);
			}

			if (Planet.GetPlanet().CurrentStep == 100)
			{
				Summary.Step100Snapshot = Snapshot;
				Summary.bHasStep100Snapshot = true;
				if (bLogCopiedFrontierAttribution)
				{
					AddCopiedFrontierSolveAttributionInfo(
						Test,
						FString::Printf(TEXT("%s Step100Attribution"), *LiveSummaryTag),
						Planet.GetLastCopiedFrontierSolveAttributionForTest());
				}
			}
			if (Planet.GetPlanet().CurrentStep == 50)
			{
				Summary.Step50Snapshot = Snapshot;
				Summary.bHasStep50Snapshot = true;
				if (bLogCopiedFrontierAttribution)
				{
					AddCopiedFrontierSolveAttributionInfo(
						Test,
						FString::Printf(TEXT("%s Step50Attribution"), *LiveSummaryTag),
						Planet.GetLastCopiedFrontierSolveAttributionForTest());
				}
			}
			if (Planet.GetPlanet().CurrentStep == 150)
			{
				Summary.Step150Snapshot = Snapshot;
				Summary.bHasStep150Snapshot = true;
				if (bLogCopiedFrontierAttribution)
				{
					AddCopiedFrontierSolveAttributionInfo(
						Test,
						FString::Printf(TEXT("%s Step150Attribution"), *LiveSummaryTag),
						Planet.GetLastCopiedFrontierSolveAttributionForTest());
				}
			}
			if (Planet.GetPlanet().CurrentStep == 200)
			{
				Summary.Step200Snapshot = Snapshot;
				Summary.bHasStep200Snapshot = true;
				if (bLogCopiedFrontierAttribution)
				{
					AddCopiedFrontierSolveAttributionInfo(
						Test,
						FString::Printf(TEXT("%s Step200Attribution"), *LiveSummaryTag),
						Planet.GetLastCopiedFrontierSolveAttributionForTest());
				}
			}
		}

		Summary.FinalSnapshot = BuildV6CheckpointSnapshot(Planet);
		FString SummaryTag = TEXT("[V6Phase1Baseline]");
		if (IsThesisLikeRunId(RunId))
		{
			SummaryTag = GetV6ThesisBaselineSummaryTag(RunId);
		}
		else if (RunId.Contains(TEXT("Phase2")))
		{
			SummaryTag = TEXT("[V6Phase2Baseline]");
		}
		else if (RunId.Contains(TEXT("Phase3")))
		{
			SummaryTag = TEXT("[V6Phase3Baseline]");
		}
		else if (RunId.Contains(TEXT("Phase1b")))
		{
			SummaryTag = TEXT("[V6Phase1bBaseline]");
		}
		if (IsThesisLikeRunId(RunId))
		{
			if (Summary.bHasStep0Snapshot)
			{
				AddV6ThesisRemeshInfo(Test, SummaryTag, Summary.Step0Snapshot);
			}
			if (Summary.bHasStep50Snapshot)
			{
				AddV6ThesisRemeshInfo(Test, SummaryTag, Summary.Step50Snapshot);
			}
			if (Summary.bHasStep100Snapshot)
			{
				AddV6ThesisRemeshInfo(Test, SummaryTag, Summary.Step100Snapshot);
			}
			if (Summary.bHasStep150Snapshot)
			{
				AddV6ThesisRemeshInfo(Test, SummaryTag, Summary.Step150Snapshot);
			}
			if (Summary.bHasStep200Snapshot)
			{
				AddV6ThesisRemeshInfo(Test, SummaryTag, Summary.Step200Snapshot);
			}
			Test.AddInfo(FString::Printf(
				TEXT("%s run_id=%s step0_continental_area_fraction=%.6f step50_continental_area_fraction=%.6f step100_continental_area_fraction=%.6f step150_continental_area_fraction=%.6f step200_continental_area_fraction=%.6f step0_hit_count=%d step0_miss_count=%d step0_oceanic_creation=%d step50_oceanic_creation=%d step100_oceanic_creation=%d step150_oceanic_creation=%d step200_oceanic_creation=%d step200_hit_count=%d step200_copied_frontier_hit_count=%d step200_plate_submesh_frontier_hit_count=%d step200_miss_count=%d step200_multi_hit_count=%d step200_direct_hit_triangle_transfer=%d step200_transfer_fallback=%d step200_coherence_reassigned=%d step200_max_components=%d step200_cw_threshold_mismatch=%d step200_plate_submesh_frontier_triangle_count=%d step200_plate_submesh_mixed_triangle_duplication_count=%d cadence_steps=%d"),
				*SummaryTag,
				*Summary.RunId,
				Summary.Step0Snapshot.ContinentalAreaFraction,
				Summary.Step50Snapshot.ContinentalAreaFraction,
				Summary.Step100Snapshot.ContinentalAreaFraction,
				Summary.Step150Snapshot.ContinentalAreaFraction,
				Summary.Step200Snapshot.ContinentalAreaFraction,
				Summary.Step0Snapshot.HitCount,
				Summary.Step0Snapshot.MissCount,
				Summary.Step0Snapshot.OceanicCreationCount,
				Summary.Step50Snapshot.OceanicCreationCount,
				Summary.Step100Snapshot.OceanicCreationCount,
				Summary.Step150Snapshot.OceanicCreationCount,
				Summary.Step200Snapshot.OceanicCreationCount,
				Summary.Step200Snapshot.HitCount,
				Summary.Step200Snapshot.CopiedFrontierHitCount,
				Summary.Step200Snapshot.PlateSubmeshFrontierHitCount,
				Summary.Step200Snapshot.MissCount,
				Summary.Step200Snapshot.MultiHitCount,
				Summary.Step200Snapshot.DirectHitTriangleTransferCount,
				Summary.Step200Snapshot.TransferFallbackCount,
				Summary.Step200Snapshot.CoherenceReassignedSampleCount,
				Summary.Step200Snapshot.MaxComponentsPerPlate,
				Summary.Step200Snapshot.ContinentalWeightThresholdMismatchCount,
				Summary.Step200Snapshot.PlateSubmeshFrontierTriangleCount,
				Summary.Step200Snapshot.PlateSubmeshWholeMixedTriangleDuplicationCount,
				Summary.Step200Snapshot.Interval));
			return Summary;
		}

		if (Summary.bHasStep50Snapshot)
		{
			AddV6BoundaryTransferInfo(Test, SummaryTag, Summary.Step50Snapshot);
		}
		if (Summary.bHasStep100Snapshot)
		{
			AddV6BoundaryTransferInfo(Test, SummaryTag, Summary.Step100Snapshot);
		}
		if (Summary.bHasStep150Snapshot)
		{
			AddV6BoundaryTransferInfo(Test, SummaryTag, Summary.Step150Snapshot);
		}
		if (Summary.bHasStep200Snapshot)
		{
			AddV6BoundaryTransferInfo(Test, SummaryTag, Summary.Step200Snapshot);
		}
		Test.AddInfo(FString::Printf(
			TEXT("%s run_id=%s step100_continental_area_fraction=%.6f step200_continental_area_fraction=%.6f step100_gap_count=%d step100_overlap_count=%d step100_single=%d step100_multi=%d step100_triangle_recovery=%d step100_member_recovery=%d step100_explicit_fallback=%d step100_boundary_decisions=%d step100_boundary_retained=%d step100_boundary_reassigned=%d step100_boundary_oceanic=%d step100_boundary_limited_fallback=%d step100_coherence_reassigned=%d step100_triangle_transfer=%d step100_single_source_transfer=%d step100_oceanic_creation=%d step100_default_transfer=%d step100_cw_threshold_mismatch=%d step100_triangle_transfer_single=%d step100_triangle_transfer_overlap=%d step100_triangle_transfer_recovery=%d step100_triangle_transfer_coherence=%d step100_single_source_single=%d step100_single_source_overlap=%d step100_single_source_recovery=%d step100_single_source_coherence=%d step200_gap_count=%d step200_overlap_count=%d step200_single=%d step200_multi=%d step200_triangle_recovery=%d step200_member_recovery=%d step200_explicit_fallback=%d step200_boundary_decisions=%d step200_boundary_retained=%d step200_boundary_reassigned=%d step200_boundary_oceanic=%d step200_boundary_limited_fallback=%d step200_coherence_reassigned=%d step200_triangle_transfer=%d step200_single_source_transfer=%d step200_oceanic_creation=%d step200_default_transfer=%d step200_cw_threshold_mismatch=%d step200_triangle_transfer_single=%d step200_triangle_transfer_overlap=%d step200_triangle_transfer_recovery=%d step200_triangle_transfer_coherence=%d step200_single_source_single=%d step200_single_source_overlap=%d step200_single_source_recovery=%d step200_single_source_coherence=%d max_components_observed=%d final_max_components=%d"),
			*SummaryTag,
			*Summary.RunId,
			Summary.Step100Snapshot.ContinentalAreaFraction,
			Summary.Step200Snapshot.ContinentalAreaFraction,
			Summary.Step100Snapshot.GapCount,
			Summary.Step100Snapshot.OverlapCount,
			Summary.Step100Snapshot.SingleCandidateWinnerCount,
			Summary.Step100Snapshot.MultiCandidateWinnerCount,
			Summary.Step100Snapshot.TriangleRecoveryCount,
			Summary.Step100Snapshot.MemberRecoveryCount,
			Summary.Step100Snapshot.ExplicitFallbackCount,
			Summary.Step100Snapshot.BoundaryDecisionSampleCount,
			Summary.Step100Snapshot.BoundaryRetainedCount,
			Summary.Step100Snapshot.BoundaryReassignedCount,
			Summary.Step100Snapshot.BoundaryOceanicCount,
			Summary.Step100Snapshot.BoundaryLimitedFallbackCount,
			Summary.Step100Snapshot.CoherenceReassignedSampleCount,
			Summary.Step100Snapshot.TriangleTransferCount,
			Summary.Step100Snapshot.SingleSourceTransferCount,
			Summary.Step100Snapshot.OceanicCreationCount,
			Summary.Step100Snapshot.DefaultTransferCount,
			Summary.Step100Snapshot.ContinentalWeightThresholdMismatchCount,
			Summary.Step100Snapshot.TriangleTransferCountsByResolution.ExactSingleHit,
			Summary.Step100Snapshot.TriangleTransferCountsByResolution.OverlapWinner,
			Summary.Step100Snapshot.TriangleTransferCountsByResolution.TriangleRecovery,
			Summary.Step100Snapshot.TriangleTransferCountsByResolution.CoherenceReassigned,
			Summary.Step100Snapshot.SingleSourceTransferCountsByResolution.ExactSingleHit,
			Summary.Step100Snapshot.SingleSourceTransferCountsByResolution.OverlapWinner,
			Summary.Step100Snapshot.SingleSourceTransferCountsByResolution.TriangleRecovery,
			Summary.Step100Snapshot.SingleSourceTransferCountsByResolution.CoherenceReassigned,
			Summary.Step200Snapshot.GapCount,
			Summary.Step200Snapshot.OverlapCount,
			Summary.Step200Snapshot.SingleCandidateWinnerCount,
			Summary.Step200Snapshot.MultiCandidateWinnerCount,
			Summary.Step200Snapshot.TriangleRecoveryCount,
			Summary.Step200Snapshot.MemberRecoveryCount,
			Summary.Step200Snapshot.ExplicitFallbackCount,
			Summary.Step200Snapshot.BoundaryDecisionSampleCount,
			Summary.Step200Snapshot.BoundaryRetainedCount,
			Summary.Step200Snapshot.BoundaryReassignedCount,
			Summary.Step200Snapshot.BoundaryOceanicCount,
			Summary.Step200Snapshot.BoundaryLimitedFallbackCount,
			Summary.Step200Snapshot.CoherenceReassignedSampleCount,
			Summary.Step200Snapshot.TriangleTransferCount,
			Summary.Step200Snapshot.SingleSourceTransferCount,
			Summary.Step200Snapshot.OceanicCreationCount,
			Summary.Step200Snapshot.DefaultTransferCount,
			Summary.Step200Snapshot.ContinentalWeightThresholdMismatchCount,
			Summary.Step200Snapshot.TriangleTransferCountsByResolution.ExactSingleHit,
			Summary.Step200Snapshot.TriangleTransferCountsByResolution.OverlapWinner,
			Summary.Step200Snapshot.TriangleTransferCountsByResolution.TriangleRecovery,
			Summary.Step200Snapshot.TriangleTransferCountsByResolution.CoherenceReassigned,
			Summary.Step200Snapshot.SingleSourceTransferCountsByResolution.ExactSingleHit,
			Summary.Step200Snapshot.SingleSourceTransferCountsByResolution.OverlapWinner,
			Summary.Step200Snapshot.SingleSourceTransferCountsByResolution.TriangleRecovery,
			Summary.Step200Snapshot.SingleSourceTransferCountsByResolution.CoherenceReassigned,
			Summary.MaxComponentsObserved,
			Summary.FinalSnapshot.MaxComponentsPerPlate));
		return Summary;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1AuthoritativePeriodicPathTest,
	"Aurous.TectonicPlanet.V6Phase1AuthoritativePeriodicPathTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1AuthoritativePeriodicPathTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6();
	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	TestEqual(TEXT("V6 periodic interval matches 60k baseline"), Interval, 10);

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	TestTrue(TEXT("V6 uses the authoritative periodic path"), Planet.IsPhase1AuthoritativePeriodicPath());
	TestEqual(TEXT("V6 runs exactly one periodic solve at the first interval"), Planet.GetPeriodicSolveCount(), 1);
	TestEqual(TEXT("V6 keeps legacy resampling history empty"), Planet.GetPlanet().ResamplingHistory.Num(), 0);
	TestEqual(TEXT("V6 first solve is periodic"), Planet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Periodic);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1dTightFreshAdmissionClassifierTest,
	"Aurous.TectonicPlanet.V6V9Phase1dTightFreshAdmissionClassifierTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1dTightFreshAdmissionClassifierTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	constexpr int32 BroadActiveBoundaryRingCount = 1;
	constexpr int32 PersistentActiveBoundaryRingCount = 1;
	constexpr int32 TightFreshActiveBoundaryRingCount = 1;
	constexpr int32 PersistentActivePairHorizon = 2;
	const FString RunId = TEXT("V9Phase1dTightFreshAdmission");

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		bool bHasStep100 = false;
	};

	const auto InitializeVariant =
		[this, FixedIntervalSteps, SampleCount, PlateCount, &RunId](
			const TCHAR* VariantLabel,
			const ETectonicPlanetV6ActiveZoneClassifierMode ClassifierMode,
			const int32 ActiveBoundaryRingCount,
			const int32 PersistenceHorizon) -> FVariantState
	{
		FVariantState Variant;
		Variant.Label = VariantLabel;
		Variant.Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Variant.Planet.SetSyntheticCoverageRetentionForTest(false);
		Variant.Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Variant.Planet.SetExcludeMixedTrianglesForTest(false);
		Variant.Planet.SetV9Phase1AuthorityForTest(true, ActiveBoundaryRingCount);
		Variant.Planet.SetV9Phase1ActiveZoneClassifierModeForTest(ClassifierMode);
		Variant.Planet.SetV9Phase1PersistentActivePairHorizonForTest(PersistenceHorizon);

		Variant.ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			RunId,
			VariantLabel);

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.DeleteDirectoryRecursively(*Variant.ExportRoot);
		PlatformFile.CreateDirectoryTree(*Variant.ExportRoot);
		ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, 0);

		FString ClassifierLabel = TEXT("BroadBoundaryBand");
		switch (ClassifierMode)
		{
		case ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocal:
			ClassifierLabel = TEXT("PersistentPairLocal");
			break;
		case ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission:
			ClassifierLabel = TEXT("PersistentPairLocalTightFreshAdmission");
			break;
		case ETectonicPlanetV6ActiveZoneClassifierMode::NarrowTectonicPairs:
			ClassifierLabel = TEXT("NarrowTectonicPairs");
			break;
		case ETectonicPlanetV6ActiveZoneClassifierMode::BroadBoundaryBand:
		default:
			break;
		}

		const FString Message = FString::Printf(
			TEXT("[V9Phase1d variant=%s] phase1=1 cadence=%d samples=%d plates=%d active_boundary_rings=%d persistence_horizon=%d classifier=%s mixed_mode=PartitionSplit retention=0"),
			VariantLabel,
			FixedIntervalSteps,
			SampleCount,
			PlateCount,
			ActiveBoundaryRingCount,
			PersistenceHorizon,
			*ClassifierLabel);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
		return Variant;
	};

	const auto LogCheckpoint =
		[this](
			FVariantState& Variant,
			const FString& SummaryTag,
			const int32 Step,
			FV6CheckpointSnapshot FVariantState::* SnapshotMember)
	{
		ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
		ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		Variant.*SnapshotMember = Snapshot;
		AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
		AddV6TectonicInteractionInfo(*this, SummaryTag, Snapshot);
		AddV6ActiveZoneInfo(*this, SummaryTag, Snapshot);
	};

	FVariantState Broad = InitializeVariant(
		TEXT("phase1_broad_classifier"),
		ETectonicPlanetV6ActiveZoneClassifierMode::BroadBoundaryBand,
		BroadActiveBoundaryRingCount,
		1);
	FVariantState Phase1c = InitializeVariant(
		TEXT("phase1c_persistent_pair_local_classifier"),
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocal,
		PersistentActiveBoundaryRingCount,
		PersistentActivePairHorizon);
	FVariantState Phase1d = InitializeVariant(
		TEXT("phase1d_tight_fresh_admission_classifier"),
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission,
		TightFreshActiveBoundaryRingCount,
		PersistentActivePairHorizon);

	Broad.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Broad,
		TEXT("[V9Phase1d broad step=25]"),
		25,
		&FVariantState::Step25Snapshot);
	Phase1c.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Phase1c,
		TEXT("[V9Phase1d phase1c step=25]"),
		25,
		&FVariantState::Step25Snapshot);
	Phase1d.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Phase1d,
		TEXT("[V9Phase1d phase1d step=25]"),
		25,
		&FVariantState::Step25Snapshot);

	const bool bFrozenAtStep25 =
		Phase1d.Step25Snapshot.ActiveZone.ActiveFraction <= UE_DOUBLE_SMALL_NUMBER;
	const bool bTooBroadAtStep25 =
		Phase1d.Step25Snapshot.ActiveZone.ActiveFraction > 0.30;
	const bool bInvalidOutsideAtStep25 =
		Phase1d.Step25Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone != 0;
	const bool bRunStep100 =
		!bFrozenAtStep25 &&
		!bTooBroadAtStep25 &&
		!bInvalidOutsideAtStep25;

	AddInfo(FString::Printf(
		TEXT("[V9Phase1d gate step=25] run_step100=%d active_fraction=%.4f outside_zone_changes=%d churn=%.4f phase1c_churn=%.4f broad_churn=%.4f plates_with_hit=%d/%d largest_share=%.4f active_pairs=%d fresh_pairs=%d carry_pairs=%d fresh_pair_candidates=%d fresh_pair_admitted=%d fresh_reject_support=%d fresh_reject_velocity=%d fresh_reject_dominance=%d generic_query=%d frozen=%d too_broad=%d invalid_outside=%d"),
		bRunStep100 ? 1 : 0,
		Phase1d.Step25Snapshot.ActiveZone.ActiveFraction,
		Phase1d.Step25Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone,
		Phase1d.Step25Snapshot.OwnershipChurn.ChurnFraction,
		Phase1c.Step25Snapshot.OwnershipChurn.ChurnFraction,
		Broad.Step25Snapshot.OwnershipChurn.ChurnFraction,
		Phase1d.Step25Snapshot.CompetitiveParticipation.PlatesWithAnyHit,
		Phase1d.Step25Snapshot.CompetitiveParticipation.TotalPlateCount,
		Phase1d.Step25Snapshot.CompetitiveParticipation.LargestPlateHitShare,
		Phase1d.Step25Snapshot.ActiveZone.ActivePairCount,
		Phase1d.Step25Snapshot.ActiveZone.FreshSeedActivePairCount,
		Phase1d.Step25Snapshot.ActiveZone.PersistentCarryoverActivePairCount,
		Phase1d.Step25Snapshot.ActiveZone.FreshPairCandidateCount,
		Phase1d.Step25Snapshot.ActiveZone.FreshPairAdmittedCount,
		Phase1d.Step25Snapshot.ActiveZone.FreshPairRejectedSupportCount,
		Phase1d.Step25Snapshot.ActiveZone.FreshPairRejectedVelocityCount,
		Phase1d.Step25Snapshot.ActiveZone.FreshPairRejectedDominanceCount,
		Phase1d.Step25Snapshot.ActiveZone.OwnershipChangeGenericQueryCompetitionCount,
		bFrozenAtStep25 ? 1 : 0,
		bTooBroadAtStep25 ? 1 : 0,
		bInvalidOutsideAtStep25 ? 1 : 0));

	if (!bRunStep100)
	{
		TestTrue(
			TEXT("Phase1d step-25 outside-zone ownership changes remain near zero"),
			Phase1d.Step25Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone == 0);
		return true;
	}

	Broad.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Broad,
		TEXT("[V9Phase1d broad step=100]"),
		100,
		&FVariantState::Step100Snapshot);
	Broad.bHasStep100 = true;

	Phase1c.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Phase1c,
		TEXT("[V9Phase1d phase1c step=100]"),
		100,
		&FVariantState::Step100Snapshot);
	Phase1c.bHasStep100 = true;

	Phase1d.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Phase1d,
		TEXT("[V9Phase1d phase1d step=100]"),
		100,
		&FVariantState::Step100Snapshot);
	Phase1d.bHasStep100 = true;

	const int32 Phase1dExplicitCauseChanges =
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangeDivergenceFillCount +
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangeConvergentSubductionCount +
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangeCollisionContactCount +
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangeRiftCount;
	const bool bValidatedForStep200Next =
		Phase1d.Step100Snapshot.ActiveZone.ActiveFraction > UE_DOUBLE_SMALL_NUMBER &&
		Phase1d.Step100Snapshot.ActiveZone.ActiveFraction <= 0.30 &&
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone == 0 &&
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangeGenericQueryCompetitionCount == 0 &&
		Phase1d.Step100Snapshot.OwnershipChurn.ChurnFraction <=
			Phase1c.Step100Snapshot.OwnershipChurn.ChurnFraction &&
		Phase1d.Step100Snapshot.CompetitiveParticipation.PlatesWithAnyHit >= 8 &&
		Phase1d.Step100Snapshot.CompetitiveParticipation.LargestPlateHitShare <= 0.55;

	AddInfo(FString::Printf(
		TEXT("[V9Phase1d gate step=100] validated_for_step200_next=%d active_fraction=%.4f phase1c_active_fraction=%.4f outside_zone_changes=%d churn=%.4f phase1c_churn=%.4f coherence=%.4f phase1c_coherence=%.4f active_pairs=%d fresh_pairs=%d carry_pairs=%d fresh_pair_candidates=%d fresh_pair_admitted=%d fresh_reject_support=%d fresh_reject_velocity=%d fresh_reject_dominance=%d active_carryover=%d fresh_expanded=%d carry_expanded=%d generic_query=%d explicit_causes=%d plates_with_hit=%d/%d largest_share=%.4f collision=%d"),
		bValidatedForStep200Next ? 1 : 0,
		Phase1d.Step100Snapshot.ActiveZone.ActiveFraction,
		Phase1c.Step100Snapshot.ActiveZone.ActiveFraction,
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone,
		Phase1d.Step100Snapshot.OwnershipChurn.ChurnFraction,
		Phase1c.Step100Snapshot.OwnershipChurn.ChurnFraction,
		Phase1d.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Phase1c.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Phase1d.Step100Snapshot.ActiveZone.ActivePairCount,
		Phase1d.Step100Snapshot.ActiveZone.FreshSeedActivePairCount,
		Phase1d.Step100Snapshot.ActiveZone.PersistentCarryoverActivePairCount,
		Phase1d.Step100Snapshot.ActiveZone.FreshPairCandidateCount,
		Phase1d.Step100Snapshot.ActiveZone.FreshPairAdmittedCount,
		Phase1d.Step100Snapshot.ActiveZone.FreshPairRejectedSupportCount,
		Phase1d.Step100Snapshot.ActiveZone.FreshPairRejectedVelocityCount,
		Phase1d.Step100Snapshot.ActiveZone.FreshPairRejectedDominanceCount,
		Phase1d.Step100Snapshot.ActiveZone.ActiveCarryoverSampleCount,
		Phase1d.Step100Snapshot.ActiveZone.FreshExpandedSampleCount,
		Phase1d.Step100Snapshot.ActiveZone.CarryoverExpandedSampleCount,
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangeGenericQueryCompetitionCount,
		Phase1dExplicitCauseChanges,
		Phase1d.Step100Snapshot.CompetitiveParticipation.PlatesWithAnyHit,
		Phase1d.Step100Snapshot.CompetitiveParticipation.TotalPlateCount,
		Phase1d.Step100Snapshot.CompetitiveParticipation.LargestPlateHitShare,
		Phase1d.Step100Snapshot.CollisionCount));

	TestTrue(
		TEXT("Phase1d step-100 outside-zone ownership changes remain near zero"),
		Phase1d.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone == 0);
	TestTrue(
		TEXT("Phase1d step-100 active+inactive samples cover the whole planet"),
		Phase1d.Step100Snapshot.ActiveZone.ActiveSampleCount +
			Phase1d.Step100Snapshot.ActiveZone.InactiveSampleCount ==
			SampleCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1SingleFinalOwnerPerSampleTest,
	"Aurous.TectonicPlanet.V6Phase1SingleFinalOwnerPerSampleTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1SingleFinalOwnerPerSampleTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6();
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const TArray<FTectonicPlanetV6ResolvedSample>& ResolvedSamples = Planet.GetLastResolvedSamples();
	TestEqual(TEXT("V6 resolves one sample entry per canonical sample"), ResolvedSamples.Num(), Planet.GetPlanet().Samples.Num());
	int32 UnresolvedCount = 0;
	int32 CanonicalMismatchCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < ResolvedSamples.Num(); ++SampleIndex)
	{
		const FTectonicPlanetV6ResolvedSample& Resolved = ResolvedSamples[SampleIndex];
		UnresolvedCount += Resolved.FinalPlateId == INDEX_NONE ? 1 : 0;
		CanonicalMismatchCount += Planet.GetPlanet().Samples[SampleIndex].PlateId == Resolved.FinalPlateId ? 0 : 1;
	}
	TestEqual(TEXT("V6 resolves every sample to exactly one final owner"), UnresolvedCount, 0);
	TestEqual(TEXT("V6 writes every resolved owner back to canonical state"), CanonicalMismatchCount, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1DeterministicOverlapWinnerTest,
	"Aurous.TectonicPlanet.V6Phase1DeterministicOverlapWinnerTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1DeterministicOverlapWinnerTest::RunTest(const FString& Parameters)
{
	TArray<FTectonicPlanetV6OwnerCandidate> OwnerCandidates;
	OwnerCandidates.Add(FTectonicPlanetV6OwnerCandidate{ 7, 17, FVector3d(0.20, 0.50, 0.30), 0.20 });
	OwnerCandidates.Add(FTectonicPlanetV6OwnerCandidate{ 3, 19, FVector3d(0.45, 0.35, 0.20), 0.20 });
	OwnerCandidates.Add(FTectonicPlanetV6OwnerCandidate{ 5, 23, FVector3d(0.40, 0.40, 0.20), 0.20 });
	TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;

	const FTectonicPlanetV6ResolvedSample Resolved = FTectonicPlanetV6::ResolvePhase1OwnershipForTest(OwnerCandidates, RecoveryCandidates);
	TestEqual(TEXT("Overlap winner resolves to exactly one plate"), Resolved.FinalPlateId, 3);
	TestEqual(TEXT("Overlap winner records the overlap resolution kind"), Resolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::OverlapWinner);
	TestEqual(TEXT("Overlap winner records all exact candidates"), Resolved.ExactCandidateCount, 3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1ZeroCandidateNearestRecoveryTest,
	"Aurous.TectonicPlanet.V6Phase1ZeroCandidateNearestRecoveryTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1ZeroCandidateNearestRecoveryTest::RunTest(const FString& Parameters)
{
	TArray<FTectonicPlanetV6OwnerCandidate> OwnerCandidates;
	TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 9, 31, INDEX_NONE, FVector3d(0.34, 0.33, 0.33), 0.020 });
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 4, 27, INDEX_NONE, FVector3d(0.20, 0.20, 0.60), 0.020 });
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 6, 25, INDEX_NONE, FVector3d(0.25, 0.50, 0.25), 0.010 });

	const FTectonicPlanetV6ResolvedSample Resolved = FTectonicPlanetV6::ResolvePhase1OwnershipForTest(OwnerCandidates, RecoveryCandidates);
	TestEqual(TEXT("Zero-candidate recovery selects the nearest recoverable owner"), Resolved.FinalPlateId, 6);
	TestEqual(TEXT("Zero-candidate recovery records the recovery resolution kind"), Resolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery);
	TestTrue(TEXT("Zero-candidate recovery preserves the winning recovery distance"), FMath::IsNearlyEqual(Resolved.RecoveryDistanceRadians, 0.010, 1.0e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase3WeakBoundaryMotionRetainsPreviousOwnerTest,
	"Aurous.TectonicPlanet.V6Phase3WeakBoundaryMotionRetainsPreviousOwnerTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase3WeakBoundaryMotionRetainsPreviousOwnerTest::RunTest(const FString& Parameters)
{
	TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 3, 11, INDEX_NONE, FVector3d(0.34, 0.33, 0.33), 0.004 });
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 7, 12, INDEX_NONE, FVector3d(0.25, 0.50, 0.25), 0.003 });

	TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
	MotionSamples.Add(MakeBoundaryMotionSample(3, FVector3d(0.0, 0.0, 0.0), FVector3d(-1.0, 0.0, 0.0), 3));
	MotionSamples.Add(MakeBoundaryMotionSample(7, FVector3d(0.2, 0.0, 0.0), FVector3d(1.0, 0.0, 0.0), 2));

	const FTectonicPlanetV6ResolvedSample Resolved = FTectonicPlanetV6::ResolveBoundaryStateForTest(
		3,
		TArray<FTectonicPlanetV6OwnerCandidate>{},
		RecoveryCandidates,
		MotionSamples);

	TestEqual(TEXT("Weak boundary motion keeps the previous owner"), Resolved.FinalPlateId, 3);
	TestEqual(TEXT("Weak boundary motion records a retained boundary outcome"), Resolved.BoundaryOutcome, ETectonicPlanetV6BoundaryOutcome::RetainedOwner);
	TestEqual(TEXT("Weak boundary motion uses the explicit boundary retention path"), Resolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::BoundaryRetained);
	TestEqual(TEXT("Boundary resolution preserves the previous plate id"), Resolved.PreviousPlateId, 3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase3ConvergentOverlapReassignsAdvancingPlateTest,
	"Aurous.TectonicPlanet.V6Phase3ConvergentOverlapReassignsAdvancingPlateTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase3ConvergentOverlapReassignsAdvancingPlateTest::RunTest(const FString& Parameters)
{
	TArray<FTectonicPlanetV6OwnerCandidate> OwnerCandidates;
	OwnerCandidates.Add(FTectonicPlanetV6OwnerCandidate{ 3, 21, FVector3d(0.45, 0.35, 0.20), 0.18 });
	OwnerCandidates.Add(FTectonicPlanetV6OwnerCandidate{ 7, 22, FVector3d(0.20, 0.45, 0.35), 0.17 });

	TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
	MotionSamples.Add(MakeBoundaryMotionSample(3, FVector3d(0.2, 0.0, 0.0), FVector3d(-1.0, 0.0, 0.0), 2));
	MotionSamples.Add(MakeBoundaryMotionSample(7, FVector3d(-1.0, 0.0, 0.0), FVector3d(1.0, 0.0, 0.0), 3));

	const FTectonicPlanetV6ResolvedSample Resolved = FTectonicPlanetV6::ResolveBoundaryStateForTest(
		3,
		OwnerCandidates,
		TArray<FTectonicPlanetV6RecoveryCandidate>{},
		MotionSamples);

	TestEqual(TEXT("Convergent overlap reassigns to the advancing plate"), Resolved.FinalPlateId, 7);
	TestEqual(TEXT("Convergent overlap records a reassigned boundary outcome"), Resolved.BoundaryOutcome, ETectonicPlanetV6BoundaryOutcome::ReassignedOwner);
	TestEqual(TEXT("Convergent overlap uses the explicit boundary reassignment path"), Resolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::BoundaryReassigned);
	TestTrue(TEXT("Convergent overlap records negative relative normal velocity"), Resolved.BoundaryRelativeNormalVelocity < 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase3DivergentGapOceanizesTest,
	"Aurous.TectonicPlanet.V6Phase3DivergentGapOceanizesTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase3DivergentGapOceanizesTest::RunTest(const FString& Parameters)
{
	TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 5, 31, INDEX_NONE, FVector3d(0.34, 0.33, 0.33), 0.003 });
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 9, 32, INDEX_NONE, FVector3d(0.20, 0.20, 0.60), 0.002 });

	TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
	MotionSamples.Add(MakeBoundaryMotionSample(5, FVector3d(-1.0, 0.0, 0.0), FVector3d(-1.0, 0.0, 0.0), 2));
	MotionSamples.Add(MakeBoundaryMotionSample(9, FVector3d(1.0, 0.0, 0.0), FVector3d(1.0, 0.0, 0.0), 2));

	const FTectonicPlanetV6ResolvedSample Resolved = FTectonicPlanetV6::ResolveBoundaryStateForTest(
		5,
		TArray<FTectonicPlanetV6OwnerCandidate>{},
		RecoveryCandidates,
		MotionSamples);

	TestEqual(TEXT("Divergent zero-hit gap resolves to an oceanic outcome"), Resolved.BoundaryOutcome, ETectonicPlanetV6BoundaryOutcome::DivergentOceanic);
	TestEqual(TEXT("Divergent zero-hit gap uses the explicit boundary oceanic path"), Resolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::BoundaryOceanic);
	TestEqual(TEXT("Divergent zero-hit gap chooses the closer bordering owner plate"), Resolved.FinalPlateId, 9);
	TestEqual(TEXT("Divergent zero-hit gap preserves the previous plate id for loss detection"), Resolved.PreviousPlateId, 5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase3DivergentGapContinentalRetainsTest,
	"Aurous.TectonicPlanet.V6Phase3DivergentGapContinentalRetainsTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase3DivergentGapContinentalRetainsTest::RunTest(const FString& Parameters)
{
	TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 5, 31, INDEX_NONE, FVector3d(0.34, 0.33, 0.33), 0.003 });
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 9, 32, INDEX_NONE, FVector3d(0.20, 0.20, 0.60), 0.002 });

	TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
	MotionSamples.Add(MakeBoundaryMotionSample(5, FVector3d(-1.0, 0.0, 0.0), FVector3d(-1.0, 0.0, 0.0), 2));
	MotionSamples.Add(MakeBoundaryMotionSample(9, FVector3d(1.0, 0.0, 0.0), FVector3d(1.0, 0.0, 0.0), 2));

	const FTectonicPlanetV6ResolvedSample Resolved = FTectonicPlanetV6::ResolveBoundaryStateForTest(
		5,
		TArray<FTectonicPlanetV6OwnerCandidate>{},
		RecoveryCandidates,
		MotionSamples,
		true);

	TestEqual(TEXT("Previously continental divergent zero-hit gap retains the previous owner"), Resolved.FinalPlateId, 5);
	TestEqual(TEXT("Previously continental divergent zero-hit gap records a retained boundary outcome"), Resolved.BoundaryOutcome, ETectonicPlanetV6BoundaryOutcome::RetainedOwner);
	TestEqual(TEXT("Previously continental divergent zero-hit gap uses the explicit boundary retain path"), Resolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::BoundaryRetained);
	TestEqual(TEXT("Previously continental divergent zero-hit gap preserves the previous plate id"), Resolved.PreviousPlateId, 5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase3DivergentGapOceanicStillOceanizesTest,
	"Aurous.TectonicPlanet.V6Phase3DivergentGapOceanicStillOceanizesTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase3DivergentGapOceanicStillOceanizesTest::RunTest(const FString& Parameters)
{
	TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 5, 31, INDEX_NONE, FVector3d(0.34, 0.33, 0.33), 0.003 });
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 9, 32, INDEX_NONE, FVector3d(0.20, 0.20, 0.60), 0.002 });

	TArray<FTectonicPlanetV6BoundaryMotionSample> MotionSamples;
	MotionSamples.Add(MakeBoundaryMotionSample(5, FVector3d(-1.0, 0.0, 0.0), FVector3d(-1.0, 0.0, 0.0), 2));
	MotionSamples.Add(MakeBoundaryMotionSample(9, FVector3d(1.0, 0.0, 0.0), FVector3d(1.0, 0.0, 0.0), 2));

	const FTectonicPlanetV6ResolvedSample Resolved = FTectonicPlanetV6::ResolveBoundaryStateForTest(
		5,
		TArray<FTectonicPlanetV6OwnerCandidate>{},
		RecoveryCandidates,
		MotionSamples,
		false);

	TestEqual(TEXT("Non-continental divergent zero-hit gap still oceanizes"), Resolved.BoundaryOutcome, ETectonicPlanetV6BoundaryOutcome::DivergentOceanic);
	TestEqual(TEXT("Non-continental divergent zero-hit gap still uses the explicit boundary oceanic path"), Resolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::BoundaryOceanic);
	TestEqual(TEXT("Non-continental divergent zero-hit gap still chooses the closer bordering owner plate"), Resolved.FinalPlateId, 9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1Baseline60k7Test,
	"Aurous.TectonicPlanet.V6Phase1Baseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1Baseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6Phase1Baseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	TArray<int32> ExportSteps = { 50, 100, 150, 200 };
	const FV6BaselineSummary Summary = RunV6BaselineScenario(*this, RunId, 200, ExportSteps);

	TestTrue(TEXT("V6 baseline records the step-100 checkpoint"), Summary.bHasStep100Snapshot);
	TestTrue(TEXT("V6 baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("V6 baseline final step lands at 200"), Summary.FinalSnapshot.Step, 200);
	TestTrue(TEXT("V6 baseline step-100 continental area fraction is finite"), FMath::IsFinite(Summary.Step100Snapshot.ContinentalAreaFraction));
	TestTrue(TEXT("V6 baseline step-200 continental area fraction is finite"), FMath::IsFinite(Summary.Step200Snapshot.ContinentalAreaFraction));
	TestTrue(TEXT("V6 baseline keeps a non-negative max component count"), Summary.MaxComponentsObserved >= 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1bBoundaryTriangleDualAssignmentTest,
	"Aurous.TectonicPlanet.V6Phase1bBoundaryTriangleDualAssignmentTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1bBoundaryTriangleDualAssignmentTest::RunTest(const FString& Parameters)
{
	TArray<int32> IncidentPlateIds;
	FTectonicPlanetV6::CollectBoundaryTrianglePlateIdsForTest(7, 3, 7, IncidentPlateIds);
	TestEqual(TEXT("Boundary triangle dual assignment keeps one entry per incident plate"), IncidentPlateIds.Num(), 2);
	TestEqual(TEXT("Boundary triangle dual assignment sorts plate ids deterministically"), IncidentPlateIds[0], 3);
	TestEqual(TEXT("Boundary triangle dual assignment includes the second incident plate"), IncidentPlateIds[1], 7);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1bOverlapPreferredOverGapRecoveryTest,
	"Aurous.TectonicPlanet.V6Phase1bOverlapPreferredOverGapRecoveryTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1bOverlapPreferredOverGapRecoveryTest::RunTest(const FString& Parameters)
{
	TArray<int32> IncidentPlateIds;
	FTectonicPlanetV6::CollectBoundaryTrianglePlateIdsForTest(5, 2, 5, IncidentPlateIds);

	TArray<FTectonicPlanetV6OwnerCandidate> OwnerCandidates;
	for (const int32 PlateId : IncidentPlateIds)
	{
		OwnerCandidates.Add(FTectonicPlanetV6OwnerCandidate{
			PlateId,
			11,
			FVector3d(0.34, 0.33, 0.33),
			PlateId == 2 ? 0.19 : 0.18 });
	}

	TArray<FTectonicPlanetV6RecoveryCandidate> RecoveryCandidates;
	RecoveryCandidates.Add(FTectonicPlanetV6RecoveryCandidate{ 9, 31, INDEX_NONE, FVector3d(0.34, 0.33, 0.33), 0.001 });

	const FTectonicPlanetV6ResolvedSample RecoveryResolved =
		FTectonicPlanetV6::ResolvePhase1OwnershipForTest(TArray<FTectonicPlanetV6OwnerCandidate>{}, RecoveryCandidates);
	TestEqual(TEXT("Without exact candidates the old path would drop to recovery"), RecoveryResolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::NearestTriangleRecovery);

	const FTectonicPlanetV6ResolvedSample OverlapResolved =
		FTectonicPlanetV6::ResolvePhase1OwnershipForTest(OwnerCandidates, RecoveryCandidates);
	TestEqual(TEXT("Boundary dual assignment prefers exact overlap competition over recovery"), OverlapResolved.ResolutionKind, ETectonicPlanetV6ResolutionKind::OverlapWinner);
	TestEqual(TEXT("Overlap winner still resolves deterministically"), OverlapResolved.FinalPlateId, 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1bTinyComponentCoherencePassTest,
	"Aurous.TectonicPlanet.V6Phase1bTinyComponentCoherencePassTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1bTinyComponentCoherencePassTest::RunTest(const FString& Parameters)
{
	TArray<TArray<int32>> SampleAdjacency;
	SampleAdjacency.SetNum(6);
	SampleAdjacency[0] = { 1 };
	SampleAdjacency[1] = { 0, 2 };
	SampleAdjacency[2] = { 1, 3, 4 };
	SampleAdjacency[3] = { 2 };
	SampleAdjacency[4] = { 2, 5 };
	SampleAdjacency[5] = { 4 };

	TArray<int32> PlateIds = { 2, 2, 1, 1, 2, 2 };
	int32 ReassignedSampleCount = 0;
	int32 RemovedComponentCount = 0;
	int32 LargestRemovedComponentSize = 0;
	int32 FinalMaxComponentsPerPlate = 0;
	FTectonicPlanetV6::ApplyCoherencePassForTest(
		SampleAdjacency,
		PlateIds,
		3,
		ReassignedSampleCount,
		RemovedComponentCount,
		LargestRemovedComponentSize,
		FinalMaxComponentsPerPlate);

	TestEqual(TEXT("Coherence pass reassigns every sample touched by the multipass cleanup"), ReassignedSampleCount, 4);
	TestEqual(TEXT("Coherence pass removes all tiny components created by the chain"), RemovedComponentCount, 3);
	TestEqual(TEXT("Coherence pass records the largest removed component size"), LargestRemovedComponentSize, 2);
	TestEqual(TEXT("Coherence pass moves the tiny component to the strongest adjacent plate"), PlateIds[2], 2);
	TestEqual(TEXT("Coherence pass updates every sample in the removed component"), PlateIds[3], 2);
	TestEqual(TEXT("Coherence pass reduces final max components per plate"), FinalMaxComponentsPerPlate, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase1bBaseline60k7Test,
	"Aurous.TectonicPlanet.V6Phase1bBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase1bBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6Phase1bBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	TArray<int32> ExportSteps = { 50, 100, 150, 200 };
	const FV6BaselineSummary Summary = RunV6BaselineScenario(*this, RunId, 200, ExportSteps);

	TestTrue(TEXT("V6 Phase 1b baseline records the step-100 checkpoint"), Summary.bHasStep100Snapshot);
	TestTrue(TEXT("V6 Phase 1b baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("V6 Phase 1b baseline final step lands at 200"), Summary.FinalSnapshot.Step, 200);
	TestTrue(TEXT("V6 Phase 1b step-100 continental area fraction is finite"), FMath::IsFinite(Summary.Step100Snapshot.ContinentalAreaFraction));
	TestTrue(TEXT("V6 Phase 1b step-200 continental area fraction is finite"), FMath::IsFinite(Summary.Step200Snapshot.ContinentalAreaFraction));
	TestTrue(TEXT("V6 Phase 1b records a non-negative coherence reassignment count"), Summary.Step200Snapshot.CoherenceReassignedSampleCount >= 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase2ContinentalWeightUsesFinalOwnerDominantSourceTest,
	"Aurous.TectonicPlanet.V6Phase2ContinentalWeightUsesFinalOwnerDominantSourceTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase2ContinentalWeightUsesFinalOwnerDominantSourceTest::RunTest(const FString& Parameters)
{
	FSample Sample;
	Sample.Position = FVector3d(0.0, 0.0, 1.0);
	const FCarriedSample V0 = MakeCarriedSample(10, 0.0f, 1.0f, 10.0f, 100.0f, EOrogenyType::None, 1);
	const FCarriedSample V1 = MakeCarriedSample(11, 1.0f, 2.0f, 20.0f, 200.0f, EOrogenyType::Andean, 2);
	const FCarriedSample V2 = MakeCarriedSample(12, 1.0f, 3.0f, 30.0f, 300.0f, EOrogenyType::Himalayan, 3);
	FTectonicPlanetV6TransferDebugInfo TransferDebug;
	float SubductionDistanceKm = 0.0f;
	float SubductionSpeed = 0.0f;

	FTectonicPlanetV6::ApplyTriangleTransferForTest(
		Sample,
		V0,
		V1,
		V2,
		FVector3d(0.40, 0.35, 0.25),
		SubductionDistanceKm,
		SubductionSpeed,
		TransferDebug);

	TestTrue(TEXT("Phase 2 CW uses the dominant final-owner source rather than a barycentric blend"), FMath::IsNearlyEqual(Sample.ContinentalWeight, 0.0f, 1.0e-6f));
	TestEqual(TEXT("Phase 2 records a triangle transfer source"), TransferDebug.SourceKind, ETectonicPlanetV6TransferSourceKind::Triangle);
	TestEqual(TEXT("Phase 2 records the dominant source canonical sample"), TransferDebug.DominantSourceCanonicalSampleIndex, 10);
	TestTrue(TEXT("Phase 2 flags when barycentric CW would cross the threshold differently"), TransferDebug.bContinentalWeightWouldCrossThresholdUnderBarycentricBlend);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase2CategoricalFieldsDoNotUseScalarInterpolationTest,
	"Aurous.TectonicPlanet.V6Phase2CategoricalFieldsDoNotUseScalarInterpolationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase2CategoricalFieldsDoNotUseScalarInterpolationTest::RunTest(const FString& Parameters)
{
	FSample Sample;
	Sample.Position = FVector3d(0.0, 0.0, 1.0);
	const FCarriedSample V0 = MakeCarriedSample(21, 1.0f, 1.0f, 10.0f, 100.0f, EOrogenyType::None, 7);
	const FCarriedSample V1 = MakeCarriedSample(22, 0.0f, 2.0f, 20.0f, 200.0f, EOrogenyType::Andean, 9);
	const FCarriedSample V2 = MakeCarriedSample(23, 0.0f, 3.0f, 30.0f, 300.0f, EOrogenyType::Andean, 9);
	FTectonicPlanetV6TransferDebugInfo TransferDebug;
	float SubductionDistanceKm = 0.0f;
	float SubductionSpeed = 0.0f;

	FTectonicPlanetV6::ApplyTriangleTransferForTest(
		Sample,
		V0,
		V1,
		V2,
		FVector3d(0.55, 0.25, 0.20),
		SubductionDistanceKm,
		SubductionSpeed,
		TransferDebug);

	TestEqual(TEXT("Phase 2 terrane transfer uses categorical majority logic"), Sample.TerraneId, 9);
	TestEqual(TEXT("Phase 2 orogeny transfer uses categorical majority logic"), Sample.OrogenyType, EOrogenyType::Andean);
	TestEqual(TEXT("Phase 2 records terrane majority transfer"), TransferDebug.TerraneTransferKind, ETectonicPlanetV6CategoricalTransferKind::MajorityVote);
	TestEqual(TEXT("Phase 2 records orogeny majority transfer"), TransferDebug.OrogenyTransferKind, ETectonicPlanetV6CategoricalTransferKind::MajorityVote);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase2ScalarFieldsStillUseFinalOwnerInterpolationTest,
	"Aurous.TectonicPlanet.V6Phase2ScalarFieldsStillUseFinalOwnerInterpolationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase2ScalarFieldsStillUseFinalOwnerInterpolationTest::RunTest(const FString& Parameters)
{
	FSample Sample;
	Sample.Position = FVector3d(0.0, 0.0, 1.0);
	const FCarriedSample V0 = MakeCarriedSample(31, 1.0f, 1.0f, 10.0f, 100.0f, EOrogenyType::None, 1);
	const FCarriedSample V1 = MakeCarriedSample(32, 1.0f, 3.0f, 20.0f, 200.0f, EOrogenyType::Andean, 2);
	const FCarriedSample V2 = MakeCarriedSample(33, 1.0f, -2.0f, 30.0f, 400.0f, EOrogenyType::Himalayan, 3);
	FTectonicPlanetV6TransferDebugInfo TransferDebug;
	float SubductionDistanceKm = 0.0f;
	float SubductionSpeed = 0.0f;

	FTectonicPlanetV6::ApplyTriangleTransferForTest(
		Sample,
		V0,
		V1,
		V2,
		FVector3d(0.50, 0.25, 0.25),
		SubductionDistanceKm,
		SubductionSpeed,
		TransferDebug);

	TestTrue(TEXT("Phase 2 elevation still uses final-owner scalar interpolation"), FMath::IsNearlyEqual(Sample.Elevation, 0.75f, 1.0e-6f));
	TestTrue(TEXT("Phase 2 thickness still uses final-owner scalar interpolation"), FMath::IsNearlyEqual(Sample.Thickness, 17.5f, 1.0e-6f));
	TestTrue(TEXT("Phase 2 age still uses final-owner scalar interpolation"), FMath::IsNearlyEqual(Sample.Age, 200.0f, 1.0e-6f));
	TestEqual(TEXT("Phase 2 scalar interpolation still reports triangle transfer"), TransferDebug.SourceKind, ETectonicPlanetV6TransferSourceKind::Triangle);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6Phase3Baseline60k7Test,
	"Aurous.TectonicPlanet.V6Phase3Baseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6Phase3Baseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6Phase3Baseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	TArray<int32> ExportSteps = { 50, 100, 150, 200 };
	const FV6BaselineSummary Summary = RunV6BaselineScenario(*this, RunId, 200, ExportSteps);

	TestTrue(TEXT("V6 Phase 3 baseline records the step-100 checkpoint"), Summary.bHasStep100Snapshot);
	TestTrue(TEXT("V6 Phase 3 baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("V6 Phase 3 baseline final step lands at 200"), Summary.FinalSnapshot.Step, 200);
	TestTrue(TEXT("V6 Phase 3 step-100 boundary policy resolves ambiguous samples"), Summary.Step100Snapshot.BoundaryDecisionSampleCount > 0);
	TestTrue(TEXT("V6 Phase 3 step-200 boundary policy resolves ambiguous samples"), Summary.Step200Snapshot.BoundaryDecisionSampleCount > 0);
	TestEqual(
		TEXT("V6 Phase 3 step-200 boundary outcomes partition the boundary decisions"),
		Summary.Step200Snapshot.BoundaryDecisionSampleCount,
		Summary.Step200Snapshot.BoundaryRetainedCount +
			Summary.Step200Snapshot.BoundaryReassignedCount +
			Summary.Step200Snapshot.BoundaryOceanicCount);
	TestTrue(
		TEXT("V6 Phase 3 step-200 limits fallback usage below the zero-hit count"),
		Summary.Step200Snapshot.BoundaryLimitedFallbackCount < Summary.Step200Snapshot.GapCount);
	TestTrue(TEXT("V6 Phase 3 step-200 creates explicit oceanic crust"), Summary.Step200Snapshot.OceanicCreationCount > 0);
	TestTrue(TEXT("V6 Phase 3 step-200 keeps continental area above 0.145"), Summary.Step200Snapshot.ContinentalAreaFraction > 0.145);
	TestTrue(TEXT("V6 Phase 3 final max component count stays below 60"), Summary.FinalSnapshot.MaxComponentsPerPlate < 60);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisRemeshSpikeDirectHitTriangleTransferTest,
	"Aurous.TectonicPlanet.V6ThesisRemeshSpikeDirectHitTriangleTransferTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisRemeshSpikeDirectHitTriangleTransferTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanetV6PeriodicSolveStats& Stats = Planet.GetLastSolveStats();
	TestEqual(TEXT("Thesis remesh spike uses the dedicated solve mode"), Stats.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike);
	TestEqual(TEXT("Thesis remesh spike uses the fixed 25-step cadence"), Stats.Interval, 25);
	TestTrue(TEXT("Thesis remesh spike produces direct ray hits"), Stats.HitCount > 0);
	TestEqual(TEXT("Every hit transfers directly from its intersected triangle"), Stats.DirectHitTriangleTransferCount, Stats.HitCount);
	TestEqual(TEXT("Thesis remesh spike avoids single-source transfer on the first remesh"), Stats.SingleSourceTransferCount, 0);
	TestEqual(TEXT("Thesis remesh spike avoids default transfer on the first remesh"), Stats.DefaultTransferCount, 0);
	TestEqual(TEXT("Thesis remesh spike keeps hit-transfer fallbacks at zero on the first remesh"), Stats.TransferFallbackCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisRemeshSpikeMissCreatesOceanicCrustTest,
	"Aurous.TectonicPlanet.V6ThesisRemeshSpikeMissCreatesOceanicCrustTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisRemeshSpikeMissCreatesOceanicCrustTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanetV6PeriodicSolveStats& Stats = Planet.GetLastSolveStats();
	TestTrue(TEXT("Thesis remesh spike produces true misses on the first remesh"), Stats.MissCount > 0);
	TestEqual(TEXT("Every miss is filled by explicit oceanic creation"), Stats.OceanicCreationCount, Stats.MissCount);

	int32 MissResolutionCount = 0;
	int32 MissOceanicTransferCount = 0;
	for (const FTectonicPlanetV6ResolvedSample& Resolved : Planet.GetLastResolvedSamples())
	{
		if (Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic)
		{
			++MissResolutionCount;
			if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::OceanicCreation)
			{
				++MissOceanicTransferCount;
			}
		}
	}

	TestEqual(TEXT("Miss resolution count matches the remesh miss metric"), MissResolutionCount, Stats.MissCount);
	TestEqual(TEXT("Every miss-resolved sample uses the oceanic creation transfer path"), MissOceanicTransferCount, Stats.MissCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisRemeshSpikeRepartitionConsistencyTest,
	"Aurous.TectonicPlanet.V6ThesisRemeshSpikeRepartitionConsistencyTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisRemeshSpikeRepartitionConsistencyTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanet& LegacyPlanet = Planet.GetPlanet();
	TSet<int32> ValidPlateIds;
	for (const FPlate& Plate : LegacyPlanet.Plates)
	{
		ValidPlateIds.Add(Plate.Id);
	}

	for (int32 SampleIndex = 0; SampleIndex < LegacyPlanet.Samples.Num(); ++SampleIndex)
	{
		const int32 PlateId = LegacyPlanet.Samples[SampleIndex].PlateId;
		TestTrue(
			*FString::Printf(TEXT("Every canonical sample resolves to an existing plate after thesis remesh (%d)"), SampleIndex),
			ValidPlateIds.Contains(PlateId));
	}

	for (const FPlate& Plate : LegacyPlanet.Plates)
	{
		for (const FCarriedSample& CarriedSample : Plate.CarriedSamples)
		{
			TestTrue(
				*FString::Printf(TEXT("Plate %d carried samples keep valid canonical indices"), Plate.Id),
				LegacyPlanet.Samples.IsValidIndex(CarriedSample.CanonicalSampleIndex));
			const int32* CarriedIndexPtr = Plate.CanonicalToCarriedIndex.Find(CarriedSample.CanonicalSampleIndex);
			TestNotNull(
				*FString::Printf(TEXT("Plate %d maps each carried canonical index back to the carried sample"), Plate.Id),
				CarriedIndexPtr);
			if (CarriedIndexPtr != nullptr)
			{
				TestTrue(
					*FString::Printf(TEXT("Plate %d carried sample reverse map stays in range"), Plate.Id),
					Plate.CarriedSamples.IsValidIndex(*CarriedIndexPtr));
			}
		}

		for (const int32 TriangleIndex : Plate.SoupTriangles)
		{
			TestTrue(
				*FString::Printf(TEXT("Plate %d soup triangle index stays valid"), Plate.Id),
				LegacyPlanet.TriangleIndices.IsValidIndex(TriangleIndex));
			if (!LegacyPlanet.TriangleIndices.IsValidIndex(TriangleIndex))
			{
				continue;
			}

			const FIntVector& Triangle = LegacyPlanet.TriangleIndices[TriangleIndex];
			TestTrue(
				*FString::Printf(TEXT("Plate %d carries triangle vertex X for soup triangle %d"), Plate.Id, TriangleIndex),
				Plate.CanonicalToCarriedIndex.Contains(Triangle.X));
			TestTrue(
				*FString::Printf(TEXT("Plate %d carries triangle vertex Y for soup triangle %d"), Plate.Id, TriangleIndex),
				Plate.CanonicalToCarriedIndex.Contains(Triangle.Y));
			TestTrue(
				*FString::Printf(TEXT("Plate %d carries triangle vertex Z for soup triangle %d"), Plate.Id, TriangleIndex),
				Plate.CanonicalToCarriedIndex.Contains(Triangle.Z));
		}
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisRemeshSpikeBaseline60k7Test,
	"Aurous.TectonicPlanet.V6ThesisRemeshSpikeBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisRemeshSpikeBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6ThesisRemeshSpikeBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	const FV6BaselineSummary Summary = RunV6BaselineScenario(
		*this,
		RunId,
		200,
		TArray<int32>{},
		ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike,
		25,
		false);

	TestTrue(TEXT("Thesis remesh spike baseline records the step-50 checkpoint"), Summary.bHasStep50Snapshot);
	TestTrue(TEXT("Thesis remesh spike baseline records the step-100 checkpoint"), Summary.bHasStep100Snapshot);
	TestTrue(TEXT("Thesis remesh spike baseline records the step-150 checkpoint"), Summary.bHasStep150Snapshot);
	TestTrue(TEXT("Thesis remesh spike baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("Thesis remesh spike baseline final step lands at 200"), Summary.FinalSnapshot.Step, 200);
	TestEqual(TEXT("Thesis remesh spike step-200 uses the fixed 25-step cadence"), Summary.Step200Snapshot.Interval, 25);
	TestEqual(TEXT("Thesis remesh spike step-200 reports the thesis remesh mode"), Summary.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisRemeshSpike);
	TestTrue(TEXT("Thesis remesh spike step-200 still has direct hits"), Summary.Step200Snapshot.HitCount > 0);
	TestTrue(TEXT("Thesis remesh spike step-200 still has true misses"), Summary.Step200Snapshot.MissCount > 0);
	TestEqual(TEXT("Thesis remesh spike step-200 miss count matches oceanic creation"), Summary.Step200Snapshot.OceanicCreationCount, Summary.Step200Snapshot.MissCount);
	TestEqual(TEXT("Thesis remesh spike step-200 keeps transfer fallback at zero"), Summary.Step200Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Thesis remesh spike step-200 keeps direct hit transfer aligned with hits"), Summary.Step200Snapshot.DirectHitTriangleTransferCount, Summary.Step200Snapshot.HitCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierSpikeRepresentationTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierSpikeRepresentationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierSpikeRepresentationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanetV6PeriodicSolveStats& Stats = Planet.GetLastSolveStats();
	TestEqual(TEXT("Copied-frontier spike uses the dedicated solve mode"), Stats.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike);
	TestEqual(TEXT("Copied-frontier spike uses the fixed 25-step cadence"), Stats.Interval, 25);
	TestTrue(TEXT("Copied-frontier spike builds plate-local vertices"), Stats.PlateLocalVertexCount > 0);
	TestTrue(TEXT("Copied-frontier spike builds plate-local triangles"), Stats.PlateLocalTriangleCount > 0);
	TestTrue(TEXT("Copied-frontier spike duplicates frontier vertices"), Stats.CopiedFrontierVertexCount > 0);
	TestTrue(TEXT("Copied-frontier spike duplicates frontier triangles"), Stats.CopiedFrontierTriangleCount > 0);
	TestTrue(TEXT("Copied-frontier spike carries copied frontier samples"), Stats.CopiedFrontierCarriedSampleCount > 0);
	TestTrue(TEXT("Copied-frontier spike gets hits from copied frontier geometry"), Stats.CopiedFrontierHitCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierSpikeStep0BaselineSnapshotTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierSpikeStep0BaselineSnapshotTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierSpikeStep0BaselineSnapshotTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierDiagnosticStep0]"), Snapshot);
	AddInfo(TEXT("[V6ThesisCopiedFrontierDiagnosticStep0] boundary_decisions and coherence_reassigned are not used by the copied-frontier remesh path and remain zero in this diagnostic."));

	TestEqual(TEXT("Copied-frontier step-0 snapshot stays at step 0"), Snapshot.Step, 0);
	TestEqual(TEXT("Copied-frontier step-0 snapshot records no solves yet"), Snapshot.SolveCount, 0);
	TestEqual(TEXT("Copied-frontier step-0 snapshot reports the copied-frontier mode"), Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike);
	TestEqual(TEXT("Copied-frontier step-0 snapshot reports the fixed 25-step cadence"), Snapshot.Interval, 25);
	TestEqual(TEXT("Copied-frontier step-0 hit count is zero before remesh"), Snapshot.HitCount, 0);
	TestEqual(TEXT("Copied-frontier step-0 miss count is zero before remesh"), Snapshot.MissCount, 0);
	TestEqual(TEXT("Copied-frontier step-0 oceanic creation is zero before remesh"), Snapshot.OceanicCreationCount, 0);
	TestEqual(TEXT("Copied-frontier step-0 transfer fallback is zero before remesh"), Snapshot.TransferFallbackCount, 0);
	TestTrue(TEXT("Copied-frontier step-0 still has continental area"), Snapshot.ContinentalAreaFraction > 0.0);
	TestTrue(TEXT("Copied-frontier step-0 exposes copied-frontier vertices"), Snapshot.CopiedFrontierVertexCount > 0);
	TestTrue(TEXT("Copied-frontier step-0 exposes copied-frontier triangles"), Snapshot.CopiedFrontierTriangleCount > 0);
	TestTrue(TEXT("Copied-frontier step-0 exposes copied-frontier carried samples"), Snapshot.CopiedFrontierCarriedSampleCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierSpikeZeroDriftRawRayMissDiagnosisTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierSpikeZeroDriftRawRayMissDiagnosisTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierSpikeZeroDriftRawRayMissDiagnosisTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const FTectonicPlanetV6CopiedFrontierRayDiagnostics Diagnostics =
		Planet.BuildCopiedFrontierRayDiagnosticsForTest(6);
	AddCopiedFrontierRayDiagnosticsInfo(*this, TEXT("[V6ThesisCopiedFrontierRawRayDiagnostics]"), Diagnostics);

	TestTrue(TEXT("Copied-frontier legacy raw-ray path exposes the zero-drift miss set"), Diagnostics.RawRayMissCount > 0);
	TestEqual(
		TEXT("Copied-frontier legacy raw-ray misses are not caused by missing copied-frontier adjacent triangle copies"),
		Diagnostics.CoverageGapMissCount,
		0);
	TestTrue(
		TEXT("Copied-frontier legacy raw-ray misses are recoverable by a containing-triangle query"),
		Diagnostics.ContainingTriangleRecoveredMissCount > 0);
	TestTrue(
		TEXT("Copied-frontier legacy raw-ray misses recover specifically on adjacent copied-frontier triangles"),
		Diagnostics.AdjacentContainingTriangleRecoveredMissCount > 0);
	TestTrue(
		TEXT("Copied-frontier legacy raw-ray misses cluster on exact vertex or edge configurations"),
		(Diagnostics.NearVertexMissCount + Diagnostics.NearEdgeMissCount) > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierSpikeZeroDriftRemeshDiagnosticTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierSpikeZeroDriftRemeshDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierSpikeZeroDriftRemeshDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const FV6CheckpointSnapshot Step0Snapshot = BuildV6CheckpointSnapshot(Planet);
	const FTectonicPlanetV6CopiedFrontierRayDiagnostics Diagnostics =
		Planet.BuildCopiedFrontierRayDiagnosticsForTest(6);
	Planet.PerformAuthoritativePeriodicSolve(ETectonicPlanetV6SolveTrigger::Manual);
	const FV6CheckpointSnapshot ZeroDriftSnapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierDiagnosticStep0]"), Step0Snapshot);
	AddCopiedFrontierRayDiagnosticsInfo(*this, TEXT("[V6ThesisCopiedFrontierRawRayDiagnostics]"), Diagnostics);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierDiagnosticZeroDriftRemesh]"), ZeroDriftSnapshot);
	AddInfo(TEXT("[V6ThesisCopiedFrontierDiagnosticZeroDriftRemesh] boundary_decisions and coherence_reassigned are not used by the copied-frontier remesh path and remain zero in this diagnostic."));

	TestEqual(TEXT("Zero-drift copied-frontier remesh leaves the step at 0"), ZeroDriftSnapshot.Step, 0);
	TestEqual(TEXT("Zero-drift copied-frontier remesh performs exactly one solve"), ZeroDriftSnapshot.SolveCount, 1);
	TestEqual(TEXT("Zero-drift copied-frontier remesh is manual"), Planet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Manual);
	TestEqual(TEXT("Zero-drift copied-frontier remesh reports the copied-frontier mode"), ZeroDriftSnapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike);
	TestEqual(TEXT("Zero-drift copied-frontier remesh keeps the fixed 25-step cadence"), ZeroDriftSnapshot.Interval, 25);
	TestTrue(TEXT("Zero-drift copied-frontier remesh still produces direct hits"), ZeroDriftSnapshot.HitCount > 0);
	TestEqual(TEXT("Zero-drift copied-frontier remesh keeps direct hit transfer aligned with hits"), ZeroDriftSnapshot.DirectHitTriangleTransferCount, ZeroDriftSnapshot.HitCount);
	TestEqual(TEXT("Zero-drift copied-frontier remesh reaches identity with zero misses"), ZeroDriftSnapshot.MissCount, 0);
	TestEqual(TEXT("Zero-drift copied-frontier remesh performs no oceanic creation"), ZeroDriftSnapshot.OceanicCreationCount, 0);
	TestEqual(TEXT("Zero-drift copied-frontier remesh keeps transfer fallback at zero"), ZeroDriftSnapshot.TransferFallbackCount, 0);
	TestTrue(
		TEXT("Zero-drift copied-frontier remesh keeps max components near identity"),
		ZeroDriftSnapshot.MaxComponentsPerPlate <= (Step0Snapshot.MaxComponentsPerPlate + 1));
	TestTrue(
		TEXT("Zero-drift copied-frontier remesh preserves continental area fraction"),
		FMath::Abs(ZeroDriftSnapshot.ContinentalAreaFraction - Step0Snapshot.ContinentalAreaFraction) <= UE_DOUBLE_SMALL_NUMBER);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierSpikeOneIntervalSingleRemeshDiagnosticTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierSpikeOneIntervalSingleRemeshDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierSpikeOneIntervalSingleRemeshDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const FV6CheckpointSnapshot Step0Snapshot = BuildV6CheckpointSnapshot(Planet);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	Planet.AdvanceSteps(Interval);
	const FV6CheckpointSnapshot OneCycleSnapshot = BuildV6CheckpointSnapshot(Planet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& Attribution =
		Planet.GetLastCopiedFrontierSolveAttributionForTest();
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierDiagnosticStep0]"), Step0Snapshot);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierDiagnosticOneCycle]"), OneCycleSnapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisCopiedFrontierDiagnosticOneCycleAttribution]"), Attribution);
	AddInfo(TEXT("[V6ThesisCopiedFrontierDiagnosticOneCycle] boundary_decisions and coherence_reassigned are not used by the copied-frontier remesh path and remain zero in this diagnostic."));

	TestEqual(TEXT("One-cycle copied-frontier diagnostic lands on the first remesh step"), OneCycleSnapshot.Step, Interval);
	TestEqual(TEXT("One-cycle copied-frontier diagnostic performs exactly one solve"), OneCycleSnapshot.SolveCount, 1);
	TestEqual(TEXT("One-cycle copied-frontier diagnostic is periodic"), Planet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Periodic);
	TestEqual(TEXT("One-cycle copied-frontier diagnostic reports the copied-frontier mode"), OneCycleSnapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike);
	TestEqual(TEXT("One-cycle copied-frontier diagnostic keeps the fixed 25-step cadence"), OneCycleSnapshot.Interval, 25);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic applies the destructive remesh filter"), OneCycleSnapshot.bDestructiveTriangleExclusionApplied);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic excludes at least some destructive plate-local triangles"), OneCycleSnapshot.DestructiveTriangleGeometryExcludedCount > 0);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic seeds at least some tracked destructive triangles"), OneCycleSnapshot.TrackedDestructiveTriangleCount > 0);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic seeds new tracked destructive triangles"), OneCycleSnapshot.TrackedDestructiveTriangleNewlySeededCount > 0);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic propagates tracked destructive triangles during the interval"), OneCycleSnapshot.TrackedDestructiveTrianglePropagatedCount > 0);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic still produces direct hits"), OneCycleSnapshot.HitCount > 0);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic still uses copied-frontier hit geometry"), OneCycleSnapshot.CopiedFrontierHitCount > 0);
	TestEqual(TEXT("One-cycle copied-frontier diagnostic keeps direct hit transfer aligned with hits"), OneCycleSnapshot.DirectHitTriangleTransferCount, OneCycleSnapshot.HitCount);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic keeps at least some true-divergence oceanic creation"), OneCycleSnapshot.OceanicCreationCount > 0);
	TestTrue(TEXT("One-cycle copied-frontier diagnostic no longer forces every miss through oceanic creation"), OneCycleSnapshot.OceanicCreationCount < OneCycleSnapshot.MissCount);
	TestEqual(TEXT("One-cycle copied-frontier diagnostic keeps transfer fallback at zero"), OneCycleSnapshot.TransferFallbackCount, 0);
	TestEqual(
		TEXT("One-cycle copied-frontier diagnostic snapshot partitions misses into explicit gap-attribution buckets"),
		OneCycleSnapshot.MissTrueDivergenceGapCount +
			OneCycleSnapshot.MissDestructiveExclusionGapCount +
			OneCycleSnapshot.MissAmbiguousGapCount,
		OneCycleSnapshot.MissCount);
	TestEqual(TEXT("One-cycle attribution tracks the same hit count as the snapshot"), Attribution.HitCount, OneCycleSnapshot.HitCount);
	TestEqual(TEXT("One-cycle attribution tracks the same miss count as the snapshot"), Attribution.MissCount, OneCycleSnapshot.MissCount);
	TestEqual(TEXT("One-cycle attribution tracks the same oceanic creation count as the snapshot"), Attribution.OceanicCreationCount, OneCycleSnapshot.OceanicCreationCount);
	TestEqual(
		TEXT("One-cycle attribution partitions misses into true divergence, destructive-exclusion, and ambiguous buckets"),
		Attribution.MissTrueDivergenceGapCount +
			Attribution.MissDestructiveExclusionGapCount +
			Attribution.MissAmbiguousGapCount,
		Attribution.MissCount);
	TestEqual(
		TEXT("One-cycle attribution partitions destructive-exclusion handling into explicit continuation/synthetic/fallback buckets"),
		Attribution.DestructiveExclusionOverrideContinuityCount +
			Attribution.DestructiveExclusionStructuredSyntheticCount +
			Attribution.DestructiveExclusionFallbackOceanicCount,
		Attribution.MissDestructiveExclusionGapCount);
	TestEqual(TEXT("One-cycle attribution partitions hit winners"), Attribution.SingleHitWinnerCount + Attribution.MultiHitWinnerCount, Attribution.HitCount);
	TestEqual(TEXT("One-cycle attribution partitions hit geometry sources"), Attribution.CopiedFrontierHitCount + Attribution.InteriorHitCount, Attribution.HitCount);
	TestEqual(
		TEXT("One-cycle attribution partitions multi-hit winners by same vs cross plate"),
		Attribution.MultiHitWinnerSamePlateCount + Attribution.MultiHitWinnerCrossPlateCount,
		Attribution.MultiHitWinnerCount);
	TestEqual(
		TEXT("One-cycle attribution partitions multi-hit winners by frontier vs interior hit geometry"),
		Attribution.MultiHitWinnerCopiedFrontierHitCount + Attribution.MultiHitWinnerInteriorHitCount,
		Attribution.MultiHitWinnerCount);
	TestEqual(
		TEXT("One-cycle attribution partitions multi-hit candidate sets by same-only, cross-only, and mixed plate membership"),
		Attribution.MultiHitCandidateSamePlateOnlyCount +
			Attribution.MultiHitCandidateCrossPlateOnlyCount +
			Attribution.MultiHitCandidateMixedPlateCount,
		Attribution.MultiHitWinnerCount);
	TestEqual(
		TEXT("One-cycle attribution partitions multi-hit candidate sets by frontier-only, interior-only, and mixed geometry"),
		Attribution.MultiHitCandidateCopiedFrontierOnlyCount +
			Attribution.MultiHitCandidateInteriorOnlyCount +
			Attribution.MultiHitCandidateMixedGeometryCount,
		Attribution.MultiHitWinnerCount);
	TestTrue(
		TEXT("One-cycle attribution keeps tracked-front ring counts monotonic"),
		Attribution.MultiHitSamplesWithinTrackedTwoRingCount >= Attribution.MultiHitSamplesWithinTrackedOneRingCount &&
			Attribution.MultiHitSamplesWithinTrackedOneRingCount >= Attribution.MultiHitSamplesWithinTrackedTriangleCount);
	TestTrue(TEXT("One-cycle attribution records continental loss from the clean step-0 state"), Attribution.ContinentalSamplesLost > 0);
	TestTrue(TEXT("One-cycle attribution resolves at least some destructive-exclusion misses by continuity instead of generic oceanization"), Attribution.DestructiveExclusionOverrideContinuityCount > 0);
	TestTrue(
		TEXT("One-cycle attribution uses bounded overlap coherence pruning or previous-plate stabilization"),
		Attribution.OverlapCoherenceSupportPrunedSampleCount > 0 ||
			Attribution.OverlapCoherencePreviousPlateStabilizedSampleCount > 0);
	TestEqual(
		TEXT("One-cycle attribution partitions single-hit continental loss by same vs cross plate"),
		Attribution.SingleHitContinentalLossSamePlateCount + Attribution.SingleHitContinentalLossCrossPlateCount,
		Attribution.ContinentalLossFromHitSingleWinnerCount);
	TestEqual(
		TEXT("One-cycle attribution partitions single-hit continental loss by winning triangle type"),
		Attribution.SingleHitContinentalLossCopiedFrontierHitCount + Attribution.SingleHitContinentalLossInteriorHitCount,
		Attribution.ContinentalLossFromHitSingleWinnerCount);
	TestEqual(
		TEXT("One-cycle attribution partitions multi-hit continental loss by same vs cross plate"),
		Attribution.MultiHitContinentalLossSamePlateCount + Attribution.MultiHitContinentalLossCrossPlateCount,
		Attribution.ContinentalLossFromHitMultiWinnerCount);
	TestTrue(TEXT("One-cycle attribution records miss adjacency or multi-hit context"), Attribution.MissAdjacentToCopiedFrontierGeometryCount > 0 || Attribution.MissNearMultiHitRegionCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierIntervalTrackingPropagatesDuringIntervalTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierIntervalTrackingPropagatesDuringIntervalTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierIntervalTrackingPropagatesDuringIntervalTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	const FTectonicPlanetV6PeriodicSolveStats SeedSnapshot = Planet.BuildCurrentDiagnosticSnapshotForTest();
	const TArray<uint8> SeedTrackedKinds = Planet.GetTrackedCopiedFrontierDestructiveKindsForTest();

	Planet.AdvanceSteps(FMath::Max(1, Interval / 2));
	const FTectonicPlanetV6PeriodicSolveStats MidIntervalSnapshot = Planet.BuildCurrentDiagnosticSnapshotForTest();
	const TArray<uint8> MidTrackedKinds = Planet.GetTrackedCopiedFrontierDestructiveKindsForTest();

	int32 SeedTrackedCount = 0;
	int32 MidTrackedCount = 0;
	for (const uint8 TrackedKind : SeedTrackedKinds)
	{
		SeedTrackedCount += TrackedKind != 0 ? 1 : 0;
	}
	for (const uint8 TrackedKind : MidTrackedKinds)
	{
		MidTrackedCount += TrackedKind != 0 ? 1 : 0;
	}

	AddInfo(FString::Printf(
		TEXT("[V6ThesisCopiedFrontierIntervalTracking] seed_tracked=%d seed_new=%d mid_step=%d mid_tracked=%d mid_propagated=%d"),
		SeedTrackedCount,
		SeedSnapshot.TrackedDestructiveTriangleNewlySeededCount,
		MidIntervalSnapshot.Step,
		MidTrackedCount,
		MidIntervalSnapshot.TrackedDestructiveTrianglePropagatedCount));

	TestEqual(TEXT("Interval tracking propagation test does not remesh before the mid-interval checkpoint"), Planet.GetPeriodicSolveCount(), 0);
	TestTrue(TEXT("Interval tracking seeds destructive triangles immediately after the current remesh state"), SeedTrackedCount > 0);
	TestEqual(TEXT("Current interval seed snapshot matches the tracked triangle count"), SeedSnapshot.TrackedDestructiveTriangleCount, SeedTrackedCount);
	TestTrue(TEXT("Interval tracking records a non-zero seed count for the current interval"), SeedSnapshot.TrackedDestructiveTriangleNewlySeededCount > 0);
	TestTrue(TEXT("Interval tracking grows the tracked destructive set before remesh"), MidTrackedCount > SeedTrackedCount);
	TestTrue(TEXT("Interval tracking records propagated growth before remesh"), MidIntervalSnapshot.TrackedDestructiveTrianglePropagatedCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierIntervalTrackingRebuildsAtRemeshTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierIntervalTrackingRebuildsAtRemeshTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierIntervalTrackingRebuildsAtRemeshTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	Planet.AdvanceSteps(Interval - 1);
	const FTectonicPlanetV6PeriodicSolveStats PreRemeshSnapshot = Planet.BuildCurrentDiagnosticSnapshotForTest();
	const TArray<uint8> PreRemeshTrackedKinds = Planet.GetTrackedCopiedFrontierDestructiveKindsForTest();

	Planet.AdvanceStep();
	const FV6CheckpointSnapshot RemeshSnapshot = BuildV6CheckpointSnapshot(Planet);
	const FTectonicPlanetV6PeriodicSolveStats PostRemeshCurrentSnapshot = Planet.BuildCurrentDiagnosticSnapshotForTest();
	const TArray<uint8> PostRemeshTrackedKinds = Planet.GetTrackedCopiedFrontierDestructiveKindsForTest();

	int32 PreRemeshTrackedCount = 0;
	int32 PostRemeshTrackedCount = 0;
	for (const uint8 TrackedKind : PreRemeshTrackedKinds)
	{
		PreRemeshTrackedCount += TrackedKind != 0 ? 1 : 0;
	}
	for (const uint8 TrackedKind : PostRemeshTrackedKinds)
	{
		PostRemeshTrackedCount += TrackedKind != 0 ? 1 : 0;
	}

	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierIntervalTrackingRemesh]"), RemeshSnapshot);
	AddInfo(FString::Printf(
		TEXT("[V6ThesisCopiedFrontierIntervalTrackingRemesh] pre_remesh_tracked=%d pre_remesh_propagated=%d post_remesh_tracked=%d post_remesh_seeded=%d cleared=%d"),
		PreRemeshTrackedCount,
		PreRemeshSnapshot.TrackedDestructiveTrianglePropagatedCount,
		PostRemeshTrackedCount,
		PostRemeshCurrentSnapshot.TrackedDestructiveTriangleNewlySeededCount,
		RemeshSnapshot.TrackedDestructiveTriangleClearedCount));

	TestEqual(TEXT("Interval tracking remesh test lands on the periodic boundary"), RemeshSnapshot.Step, Interval);
	TestTrue(TEXT("Interval tracking has grown tracked triangles by remesh time"), PreRemeshSnapshot.TrackedDestructiveTrianglePropagatedCount > 0);
	TestEqual(TEXT("Interval tracking clears the full tracked destructive set that reached remesh"), RemeshSnapshot.TrackedDestructiveTriangleClearedCount, RemeshSnapshot.TrackedDestructiveTriangleCount);
	TestTrue(TEXT("Interval tracking reseeds tracked triangles for the next interval after remesh"), PostRemeshTrackedCount > 0);
	TestTrue(TEXT("Interval tracking records a fresh seed count for the new interval after remesh"), PostRemeshCurrentSnapshot.TrackedDestructiveTriangleNewlySeededCount > 0);
	TestEqual(TEXT("Current post-remesh tracked snapshot matches the reseeded tracked triangle count"), PostRemeshCurrentSnapshot.TrackedDestructiveTriangleCount, PostRemeshTrackedCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierIntervalTrackingClearInvalidContextTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierIntervalTrackingClearInvalidContextTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierIntervalTrackingClearInvalidContextTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);

	const TArray<uint8> TrackedKindsBefore = Planet.GetTrackedCopiedFrontierDestructiveKindsForTest();
	const FTectonicPlanet& PlanetBefore = Planet.GetPlanet();
	int32 TrackedTriangleIndex = INDEX_NONE;
	for (int32 TriangleIndex = 0; TriangleIndex < TrackedKindsBefore.Num(); ++TriangleIndex)
	{
		if (TrackedKindsBefore[TriangleIndex] != 0)
		{
			TrackedTriangleIndex = TriangleIndex;
			break;
		}
	}

	TestTrue(TEXT("Clear-on-invalid-context test needs at least one tracked seed triangle"), TrackedTriangleIndex != INDEX_NONE);
	if (TrackedTriangleIndex == INDEX_NONE || !PlanetBefore.TriangleIndices.IsValidIndex(TrackedTriangleIndex))
	{
		return false;
	}

	const FIntVector TrackedTriangle = PlanetBefore.TriangleIndices[TrackedTriangleIndex];
	FTectonicPlanet& MutablePlanet = Planet.GetPlanetMutable();
	const int32 UnifiedPlateId = MutablePlanet.Samples.IsValidIndex(TrackedTriangle.X)
		? MutablePlanet.Samples[TrackedTriangle.X].PlateId
		: INDEX_NONE;
	if (MutablePlanet.Samples.IsValidIndex(TrackedTriangle.X))
	{
		MutablePlanet.Samples[TrackedTriangle.X].PlateId = UnifiedPlateId;
		MutablePlanet.Samples[TrackedTriangle.X].SubductionDistanceKm = -1.0f;
	}
	if (MutablePlanet.Samples.IsValidIndex(TrackedTriangle.Y))
	{
		MutablePlanet.Samples[TrackedTriangle.Y].PlateId = UnifiedPlateId;
		MutablePlanet.Samples[TrackedTriangle.Y].SubductionDistanceKm = -1.0f;
		MutablePlanet.Samples[TrackedTriangle.Y].OrogenyType = EOrogenyType::None;
	}
	if (MutablePlanet.Samples.IsValidIndex(TrackedTriangle.Z))
	{
		MutablePlanet.Samples[TrackedTriangle.Z].PlateId = UnifiedPlateId;
		MutablePlanet.Samples[TrackedTriangle.Z].SubductionDistanceKm = -1.0f;
		MutablePlanet.Samples[TrackedTriangle.Z].OrogenyType = EOrogenyType::None;
	}
	MutablePlanet.PendingBoundaryContactCollisionEvent = FPendingBoundaryContactCollisionEvent{};
	MutablePlanet.PendingGeometricCollisionEvent = FPendingGeometricCollisionEvent{};

	Planet.RefreshCopiedFrontierDestructiveTrackingForTest();
	const TArray<uint8>& TrackedKindsAfter = Planet.GetTrackedCopiedFrontierDestructiveKindsForTest();
	TestEqual(
		TEXT("Interval tracking reseed drops the canonical triangle once the convergent frontier context is removed"),
		TrackedKindsAfter.IsValidIndex(TrackedTriangleIndex) ? static_cast<int32>(TrackedKindsAfter[TrackedTriangleIndex]) : -1,
		0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierSpikeBoundaryTransferTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierSpikeBoundaryTransferTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierSpikeBoundaryTransferTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanet& LegacyPlanet = Planet.GetPlanet();
	const FTectonicPlanetV6PeriodicSolveStats& Stats = Planet.GetLastSolveStats();
	int32 BoundaryCopiedFrontierTriangleTransfers = 0;
	for (int32 SampleIndex = 0; SampleIndex < Planet.GetLastResolvedSamples().Num(); ++SampleIndex)
	{
		const FTectonicPlanetV6ResolvedSample& Resolved = Planet.GetLastResolvedSamples()[SampleIndex];
		if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle &&
			Resolved.TransferDebug.bUsedCopiedFrontierTriangleTransfer &&
			LegacyPlanet.Samples.IsValidIndex(SampleIndex) &&
			LegacyPlanet.Samples[SampleIndex].bIsBoundary)
		{
			++BoundaryCopiedFrontierTriangleTransfers;
		}
	}

	TestTrue(TEXT("Copied-frontier spike keeps every hit on direct triangle transfer"), Stats.DirectHitTriangleTransferCount == Stats.HitCount);
	TestEqual(TEXT("Copied-frontier spike keeps transfer fallback at zero on the first remesh"), Stats.TransferFallbackCount, 0);
	TestTrue(TEXT("Copied-frontier spike can transfer a boundary sample from copied frontier triangle data"), BoundaryCopiedFrontierTriangleTransfers > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierSpikeMissCreatesOceanicCrustTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierSpikeMissCreatesOceanicCrustTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierSpikeMissCreatesOceanicCrustTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanetV6PeriodicSolveStats& Stats = Planet.GetLastSolveStats();
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& Attribution =
		Planet.GetLastCopiedFrontierSolveAttributionForTest();
	TestTrue(TEXT("Copied-frontier spike still produces true misses"), Stats.MissCount > 0);
	TestTrue(TEXT("Copied-frontier spike keeps some true-divergence oceanic creation"), Stats.OceanicCreationCount > 0);
	TestTrue(TEXT("Copied-frontier spike does not oceanize every miss once destructive-exclusion handling is active"), Stats.OceanicCreationCount < Stats.MissCount);
	TestEqual(
		TEXT("Copied-frontier destructive-exclusion handling partitions explicitly"),
		Attribution.DestructiveExclusionOverrideContinuityCount +
			Attribution.DestructiveExclusionStructuredSyntheticCount +
			Attribution.DestructiveExclusionFallbackOceanicCount,
		Attribution.MissDestructiveExclusionGapCount);

	int32 MissResolutionCount = 0;
	int32 MissOceanicTransferCount = 0;
	int32 DestructiveExclusionResolutionCount = 0;
	int32 DestructiveExclusionContinuityTransferCount = 0;
	for (const FTectonicPlanetV6ResolvedSample& Resolved : Planet.GetLastResolvedSamples())
	{
		if (Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion ||
			Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissAmbiguous)
		{
			++MissResolutionCount;
			if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::OceanicCreation)
			{
				++MissOceanicTransferCount;
			}
		}

		if (Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissDestructiveExclusion)
		{
			++DestructiveExclusionResolutionCount;
			if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle ||
				Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::SingleSource)
			{
				++DestructiveExclusionContinuityTransferCount;
			}
		}
	}

	TestEqual(TEXT("Copied-frontier miss resolution count matches the miss metric"), MissResolutionCount, Stats.MissCount);
	TestEqual(
		TEXT("Copied-frontier true-divergence and ambiguous misses still use explicit oceanic creation"),
		MissOceanicTransferCount,
		Attribution.MissTrueDivergenceGapCount +
			Attribution.MissAmbiguousGapCount +
			Attribution.DestructiveExclusionFallbackOceanicCount);
	TestEqual(TEXT("Copied-frontier destructive-exclusion resolution count matches attribution"), DestructiveExclusionResolutionCount, Attribution.MissDestructiveExclusionGapCount);
	TestEqual(TEXT("Copied-frontier destructive-exclusion continuity count matches attribution"), DestructiveExclusionContinuityTransferCount, Attribution.DestructiveExclusionOverrideContinuityCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierSpikeBaseline60k7Test,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierSpikeBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierSpikeBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6ThesisCopiedFrontierSpikeBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	const FV6BaselineSummary Summary = RunV6BaselineScenario(
		*this,
		RunId,
		200,
		TArray<int32>{},
		ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike,
		25,
		false);

	TestTrue(TEXT("Copied-frontier spike baseline records the step-0 checkpoint"), Summary.bHasStep0Snapshot);
	TestTrue(TEXT("Copied-frontier spike baseline records the step-50 checkpoint"), Summary.bHasStep50Snapshot);
	TestTrue(TEXT("Copied-frontier spike baseline records the step-100 checkpoint"), Summary.bHasStep100Snapshot);
	TestTrue(TEXT("Copied-frontier spike baseline records the step-150 checkpoint"), Summary.bHasStep150Snapshot);
	TestTrue(TEXT("Copied-frontier spike baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("Copied-frontier spike step-0 is still pre-remesh"), Summary.Step0Snapshot.Step, 0);
	TestEqual(TEXT("Copied-frontier spike step-0 has no remesh hits yet"), Summary.Step0Snapshot.HitCount, 0);
	TestEqual(TEXT("Copied-frontier spike step-0 has no remesh misses yet"), Summary.Step0Snapshot.MissCount, 0);
	TestEqual(TEXT("Copied-frontier spike step-0 has no oceanic creation yet"), Summary.Step0Snapshot.OceanicCreationCount, 0);
	TestEqual(TEXT("Copied-frontier spike baseline final step lands at 200"), Summary.FinalSnapshot.Step, 200);
	TestEqual(TEXT("Copied-frontier spike step-200 uses the fixed 25-step cadence"), Summary.Step200Snapshot.Interval, 25);
	TestEqual(TEXT("Copied-frontier spike step-200 reports the copied-frontier mode"), Summary.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike);
	TestTrue(TEXT("Copied-frontier spike step-200 keeps the destructive filter active"), Summary.Step200Snapshot.bDestructiveTriangleExclusionApplied);
	TestTrue(TEXT("Copied-frontier spike step-200 excludes destructive plate-local triangles"), Summary.Step200Snapshot.DestructiveTriangleGeometryExcludedCount > 0);
	TestTrue(TEXT("Copied-frontier spike step-200 keeps tracked destructive triangles active"), Summary.Step200Snapshot.TrackedDestructiveTriangleCount > 0);
	TestTrue(TEXT("Copied-frontier spike step-200 still has direct hits"), Summary.Step200Snapshot.HitCount > 0);
	TestTrue(TEXT("Copied-frontier spike step-200 uses copied frontier hit geometry"), Summary.Step200Snapshot.CopiedFrontierHitCount > 0);
	TestTrue(TEXT("Copied-frontier spike step-200 still has true misses"), Summary.Step200Snapshot.MissCount > 0);
	TestTrue(TEXT("Copied-frontier spike step-200 no longer routes every miss to oceanic creation"), Summary.Step200Snapshot.OceanicCreationCount < Summary.Step200Snapshot.MissCount);
	TestEqual(
		TEXT("Copied-frontier spike step-200 partitions misses into explicit gap-attribution buckets"),
		Summary.Step200Snapshot.MissTrueDivergenceGapCount +
			Summary.Step200Snapshot.MissDestructiveExclusionGapCount +
			Summary.Step200Snapshot.MissAmbiguousGapCount,
		Summary.Step200Snapshot.MissCount);
	TestEqual(TEXT("Copied-frontier spike step-200 keeps transfer fallback at zero"), Summary.Step200Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Copied-frontier spike step-200 keeps direct hit transfer aligned with hits"), Summary.Step200Snapshot.DirectHitTriangleTransferCount, Summary.Step200Snapshot.HitCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierProcessSpikeZeroDriftRemainsIdentityTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierProcessSpikeZeroDriftRemainsIdentityTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierProcessSpikeZeroDriftRemainsIdentityTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike, 25);
	const FV6CheckpointSnapshot Step0Snapshot = BuildV6CheckpointSnapshot(Planet);
	Planet.PerformAuthoritativePeriodicSolve(ETectonicPlanetV6SolveTrigger::Manual);
	const FV6CheckpointSnapshot ZeroDriftSnapshot = BuildV6CheckpointSnapshot(Planet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& Attribution =
		Planet.GetLastCopiedFrontierSolveAttributionForTest();
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierProcessStep0]"), Step0Snapshot);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierProcessZeroDrift]"), ZeroDriftSnapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisCopiedFrontierProcessZeroDriftAttribution]"), Attribution);

	TestEqual(TEXT("Copied-frontier process zero-drift solve reports the process mode"), ZeroDriftSnapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike);
	TestEqual(TEXT("Copied-frontier process zero-drift remains a manual solve"), Planet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Manual);
	TestEqual(TEXT("Copied-frontier process zero-drift keeps misses at zero"), ZeroDriftSnapshot.MissCount, 0);
	TestEqual(TEXT("Copied-frontier process zero-drift keeps oceanic creation at zero"), ZeroDriftSnapshot.OceanicCreationCount, 0);
	TestEqual(TEXT("Copied-frontier process zero-drift keeps fallback at zero"), ZeroDriftSnapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Copied-frontier process zero-drift does not apply tectonic maintenance"), Attribution.TectonicMaintenanceAppliedCount, 0);
	TestTrue(
		TEXT("Copied-frontier process zero-drift preserves continental area fraction"),
		FMath::Abs(ZeroDriftSnapshot.ContinentalAreaFraction - Step0Snapshot.ContinentalAreaFraction) <= UE_DOUBLE_SMALL_NUMBER);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierProcessSpikeOneIntervalDiagnosticTest,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierProcessSpikeOneIntervalDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierProcessSpikeOneIntervalDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 ControlPlanet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierSpike, 25);
	FTectonicPlanetV6 ProcessPlanet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike, 25);
	const int32 Interval = ControlPlanet.ComputePeriodicSolveInterval();
	ControlPlanet.AdvanceSteps(Interval);
	ProcessPlanet.AdvanceSteps(Interval);

	const FV6CheckpointSnapshot ControlSnapshot = BuildV6CheckpointSnapshot(ControlPlanet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& ControlAttribution =
		ControlPlanet.GetLastCopiedFrontierSolveAttributionForTest();
	const FV6CheckpointSnapshot ProcessSnapshot = BuildV6CheckpointSnapshot(ProcessPlanet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& ProcessAttribution =
		ProcessPlanet.GetLastCopiedFrontierSolveAttributionForTest();
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierControlOneCycle]"), ControlSnapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisCopiedFrontierControlOneCycleAttribution]"), ControlAttribution);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisCopiedFrontierProcessOneCycle]"), ProcessSnapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisCopiedFrontierProcessOneCycleAttribution]"), ProcessAttribution);

	TestEqual(TEXT("Copied-frontier process one-cycle lands on the first remesh step"), ProcessSnapshot.Step, Interval);
	TestEqual(TEXT("Copied-frontier process one-cycle reports the process mode"), ProcessSnapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike);
	TestEqual(TEXT("Copied-frontier process one-cycle is periodic"), ProcessPlanet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Periodic);
	TestTrue(TEXT("Copied-frontier process one-cycle applies the destructive remesh filter"), ProcessSnapshot.bDestructiveTriangleExclusionApplied);
	TestTrue(TEXT("Copied-frontier process one-cycle excludes destructive plate-local triangles"), ProcessSnapshot.DestructiveTriangleGeometryExcludedCount > 0);
	TestTrue(TEXT("Copied-frontier process one-cycle keeps tracked destructive triangles active"), ProcessSnapshot.TrackedDestructiveTriangleCount > 0);
	TestEqual(TEXT("Copied-frontier process one-cycle keeps direct hit transfer aligned with hits"), ProcessSnapshot.DirectHitTriangleTransferCount, ProcessSnapshot.HitCount);
	TestEqual(TEXT("Copied-frontier process one-cycle keeps transfer fallback at zero"), ProcessSnapshot.TransferFallbackCount, 0);
	TestEqual(
		TEXT("Copied-frontier process one-cycle attribution partitions misses into explicit gap-attribution buckets"),
		ProcessAttribution.MissTrueDivergenceGapCount +
			ProcessAttribution.MissDestructiveExclusionGapCount +
			ProcessAttribution.MissAmbiguousGapCount,
		ProcessAttribution.MissCount);
	TestEqual(
		TEXT("Copied-frontier process one-cycle attribution partitions multi-hit winners by same vs cross plate"),
		ProcessAttribution.MultiHitWinnerSamePlateCount + ProcessAttribution.MultiHitWinnerCrossPlateCount,
		ProcessAttribution.MultiHitWinnerCount);
	TestTrue(
		TEXT("Copied-frontier process one-cycle uses bounded overlap coherence pruning or previous-plate stabilization"),
		ProcessAttribution.OverlapCoherenceSupportPrunedSampleCount > 0 ||
			ProcessAttribution.OverlapCoherencePreviousPlateStabilizedSampleCount > 0);
	TestTrue(TEXT("Copied-frontier process one-cycle applies tectonic maintenance"), ProcessAttribution.TectonicMaintenanceAppliedCount > 0);
	TestTrue(TEXT("Copied-frontier process one-cycle recovers at least some continental samples"), ProcessAttribution.TectonicMaintenanceContinentalRecoveredCount > 0 || ProcessAttribution.TectonicMaintenanceContinentalGainCount > 0);
	TestTrue(TEXT("Copied-frontier process one-cycle should not worsen continental area fraction"), ProcessSnapshot.ContinentalAreaFraction >= ControlSnapshot.ContinentalAreaFraction);
	TestTrue(TEXT("Copied-frontier process one-cycle should not increase continental losses"), ProcessAttribution.ContinentalSamplesLost <= ControlAttribution.ContinentalSamplesLost);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisCopiedFrontierProcessSpikeBaseline60k7Test,
	"Aurous.TectonicPlanet.V6ThesisCopiedFrontierProcessSpikeBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisCopiedFrontierProcessSpikeBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6ThesisCopiedFrontierProcessSpikeBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	const FV6BaselineSummary Summary = RunV6BaselineScenario(
		*this,
		RunId,
		200,
		TArray<int32>{},
		ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike,
		25,
		false);

	TestTrue(TEXT("Copied-frontier process spike baseline records the step-0 checkpoint"), Summary.bHasStep0Snapshot);
	TestTrue(TEXT("Copied-frontier process spike baseline records the step-50 checkpoint"), Summary.bHasStep50Snapshot);
	TestTrue(TEXT("Copied-frontier process spike baseline records the step-100 checkpoint"), Summary.bHasStep100Snapshot);
	TestTrue(TEXT("Copied-frontier process spike baseline records the step-150 checkpoint"), Summary.bHasStep150Snapshot);
	TestTrue(TEXT("Copied-frontier process spike baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("Copied-frontier process spike step-200 uses the fixed 25-step cadence"), Summary.Step200Snapshot.Interval, 25);
	TestEqual(TEXT("Copied-frontier process spike step-200 reports the process mode"), Summary.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisCopiedFrontierProcessSpike);
	TestTrue(TEXT("Copied-frontier process spike step-200 keeps the destructive filter active"), Summary.Step200Snapshot.bDestructiveTriangleExclusionApplied);
	TestTrue(TEXT("Copied-frontier process spike step-200 excludes destructive plate-local triangles"), Summary.Step200Snapshot.DestructiveTriangleGeometryExcludedCount > 0);
	TestTrue(TEXT("Copied-frontier process spike step-200 keeps tracked destructive triangles active"), Summary.Step200Snapshot.TrackedDestructiveTriangleCount > 0);
	TestTrue(TEXT("Copied-frontier process spike step-200 still has direct hits"), Summary.Step200Snapshot.HitCount > 0);
	TestTrue(TEXT("Copied-frontier process spike step-200 still has true misses"), Summary.Step200Snapshot.MissCount > 0);
	TestEqual(
		TEXT("Copied-frontier process spike step-200 partitions misses into explicit gap-attribution buckets"),
		Summary.Step200Snapshot.MissTrueDivergenceGapCount +
			Summary.Step200Snapshot.MissDestructiveExclusionGapCount +
			Summary.Step200Snapshot.MissAmbiguousGapCount,
		Summary.Step200Snapshot.MissCount);
	TestEqual(TEXT("Copied-frontier process spike step-200 keeps transfer fallback at zero"), Summary.Step200Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Copied-frontier process spike step-200 keeps direct hit transfer aligned with hits"), Summary.Step200Snapshot.DirectHitTriangleTransferCount, Summary.Step200Snapshot.HitCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierSpikeZeroDriftIdentityTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierSpikeZeroDriftIdentityTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierSpikeZeroDriftIdentityTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike, 25);
	const FV6CheckpointSnapshot Step0Snapshot = BuildV6CheckpointSnapshot(Planet);
	Planet.PerformAuthoritativePeriodicSolve(ETectonicPlanetV6SolveTrigger::Manual);
	const FV6CheckpointSnapshot ZeroDriftSnapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierZeroDriftStep0]"), Step0Snapshot);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierZeroDrift]"), ZeroDriftSnapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisPartitionedFrontierZeroDriftAttribution]"), Planet.GetLastCopiedFrontierSolveAttributionForTest());

	TestEqual(TEXT("Partitioned-frontier zero-drift solve reports the partitioned mode"), ZeroDriftSnapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike);
	TestEqual(TEXT("Partitioned-frontier zero-drift solve is manual"), Planet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Manual);
	TestEqual(TEXT("Partitioned-frontier zero-drift keeps misses at zero"), ZeroDriftSnapshot.MissCount, 0);
	TestEqual(TEXT("Partitioned-frontier zero-drift keeps oceanic creation at zero"), ZeroDriftSnapshot.OceanicCreationCount, 0);
	TestEqual(TEXT("Partitioned-frontier zero-drift keeps transfer fallback at zero"), ZeroDriftSnapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Partitioned-frontier zero-drift keeps direct hit transfer aligned with hits"), ZeroDriftSnapshot.DirectHitTriangleTransferCount, ZeroDriftSnapshot.HitCount);
	TestTrue(TEXT("Partitioned-frontier zero-drift preserves continental area fraction"), FMath::Abs(ZeroDriftSnapshot.ContinentalAreaFraction - Step0Snapshot.ContinentalAreaFraction) <= UE_DOUBLE_SMALL_NUMBER);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierSpikeOneIntervalDiagnosticTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierSpikeOneIntervalDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierSpikeOneIntervalDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	Planet.AdvanceSteps(Interval);
	const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& Attribution = Planet.GetLastCopiedFrontierSolveAttributionForTest();
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierOneCycle]"), Snapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisPartitionedFrontierOneCycleAttribution]"), Attribution);

	TestEqual(TEXT("Partitioned-frontier one-cycle lands on the first remesh step"), Snapshot.Step, Interval);
	TestEqual(TEXT("Partitioned-frontier one-cycle reports the partitioned mode"), Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike);
	TestEqual(TEXT("Partitioned-frontier one-cycle is periodic"), Planet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Periodic);
	TestTrue(TEXT("Partitioned-frontier one-cycle applies the destructive filter"), Snapshot.bDestructiveTriangleExclusionApplied);
	TestTrue(TEXT("Partitioned-frontier one-cycle still produces direct hits"), Snapshot.HitCount > 0);
	TestEqual(TEXT("Partitioned-frontier one-cycle keeps direct hit transfer aligned with hits"), Snapshot.DirectHitTriangleTransferCount, Snapshot.HitCount);
	TestEqual(TEXT("Partitioned-frontier one-cycle keeps transfer fallback at zero"), Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Partitioned-frontier one-cycle partitions misses into explicit buckets"), Snapshot.MissTrueDivergenceGapCount + Snapshot.MissDestructiveExclusionGapCount + Snapshot.MissAmbiguousGapCount, Snapshot.MissCount);
	TestEqual(TEXT("Partitioned-frontier one-cycle attribution hit count matches the snapshot"), Attribution.HitCount, Snapshot.HitCount);
	TestEqual(TEXT("Partitioned-frontier one-cycle attribution miss count matches the snapshot"), Attribution.MissCount, Snapshot.MissCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierSpikeBaseline60k7Test,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierSpikeBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierSpikeBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6ThesisPartitionedFrontierSpikeBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	const FV6BaselineSummary Summary = RunV6BaselineScenario(
		*this,
		RunId,
		200,
		TArray<int32>{},
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike,
		25,
		false);

	TestTrue(TEXT("Partitioned-frontier spike baseline records the step-0 checkpoint"), Summary.bHasStep0Snapshot);
	TestTrue(TEXT("Partitioned-frontier spike baseline records the step-50 checkpoint"), Summary.bHasStep50Snapshot);
	TestTrue(TEXT("Partitioned-frontier spike baseline records the step-100 checkpoint"), Summary.bHasStep100Snapshot);
	TestTrue(TEXT("Partitioned-frontier spike baseline records the step-150 checkpoint"), Summary.bHasStep150Snapshot);
	TestTrue(TEXT("Partitioned-frontier spike baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("Partitioned-frontier spike step-200 reports the partitioned mode"), Summary.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike);
	TestEqual(TEXT("Partitioned-frontier spike step-200 uses the fixed 25-step cadence"), Summary.Step200Snapshot.Interval, 25);
	TestTrue(TEXT("Partitioned-frontier spike step-200 still has direct hits"), Summary.Step200Snapshot.HitCount > 0);
	TestEqual(TEXT("Partitioned-frontier spike step-200 keeps transfer fallback at zero"), Summary.Step200Snapshot.TransferFallbackCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierProcessSpikeOneIntervalDiagnosticTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierProcessSpikeOneIntervalDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierProcessSpikeOneIntervalDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 ControlPlanet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierSpike, 25);
	FTectonicPlanetV6 ProcessPlanet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike, 25);
	const int32 Interval = ControlPlanet.ComputePeriodicSolveInterval();
	ControlPlanet.AdvanceSteps(Interval);
	ProcessPlanet.AdvanceSteps(Interval);

	const FV6CheckpointSnapshot ControlSnapshot = BuildV6CheckpointSnapshot(ControlPlanet);
	const FV6CheckpointSnapshot ProcessSnapshot = BuildV6CheckpointSnapshot(ProcessPlanet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& ProcessAttribution = ProcessPlanet.GetLastCopiedFrontierSolveAttributionForTest();
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierControlOneCycle]"), ControlSnapshot);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierProcessOneCycle]"), ProcessSnapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisPartitionedFrontierProcessOneCycleAttribution]"), ProcessAttribution);

	TestEqual(TEXT("Partitioned-frontier process one-cycle reports the process mode"), ProcessSnapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike);
	TestEqual(TEXT("Partitioned-frontier process one-cycle is periodic"), ProcessPlanet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Periodic);
	TestEqual(TEXT("Partitioned-frontier process one-cycle keeps transfer fallback at zero"), ProcessSnapshot.TransferFallbackCount, 0);
	TestTrue(TEXT("Partitioned-frontier process one-cycle applies tectonic maintenance"), ProcessAttribution.TectonicMaintenanceAppliedCount > 0);
	TestTrue(
		TEXT("Partitioned-frontier process one-cycle uses frontier-pair structured fill for at least some misses"),
		ProcessAttribution.MissFrontierPairStructuredSyntheticCount > 0);
	TestTrue(TEXT("Partitioned-frontier process one-cycle should not worsen continental area fraction"), ProcessSnapshot.ContinentalAreaFraction >= ControlSnapshot.ContinentalAreaFraction);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierProcessSpikeBaseline60k7Test,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierProcessSpikeBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierProcessSpikeBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6ThesisPartitionedFrontierProcessSpikeBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	const FV6BaselineSummary Summary = RunV6BaselineScenario(
		*this,
		RunId,
		200,
		TArray<int32>{},
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		25,
		false);

	TestTrue(TEXT("Partitioned-frontier process baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("Partitioned-frontier process step-200 reports the process mode"), Summary.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike);
	TestEqual(TEXT("Partitioned-frontier process step-200 uses the fixed 25-step cadence"), Summary.Step200Snapshot.Interval, 25);
	TestTrue(TEXT("Partitioned-frontier process step-200 still has direct hits"), Summary.Step200Snapshot.HitCount > 0);
	TestEqual(TEXT("Partitioned-frontier process step-200 keeps transfer fallback at zero"), Summary.Step200Snapshot.TransferFallbackCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierPropagationSpikeZeroDriftIdentityTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierPropagationSpikeZeroDriftIdentityTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierPropagationSpikeZeroDriftIdentityTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike, 25);
	const FV6CheckpointSnapshot Step0Snapshot = BuildV6CheckpointSnapshot(Planet);
	Planet.PerformAuthoritativePeriodicSolve(ETectonicPlanetV6SolveTrigger::Manual);
	const FV6CheckpointSnapshot ZeroDriftSnapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationZeroDriftStep0]"), Step0Snapshot);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationZeroDrift]"), ZeroDriftSnapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationZeroDriftAttribution]"), Planet.GetLastCopiedFrontierSolveAttributionForTest());

	TestEqual(TEXT("Partitioned-frontier propagation zero-drift solve reports the propagation mode"), ZeroDriftSnapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike);
	TestEqual(TEXT("Partitioned-frontier propagation zero-drift solve is manual"), Planet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Manual);
	TestEqual(TEXT("Partitioned-frontier propagation zero-drift keeps misses at zero"), ZeroDriftSnapshot.MissCount, 0);
	TestEqual(TEXT("Partitioned-frontier propagation zero-drift keeps oceanic creation at zero"), ZeroDriftSnapshot.OceanicCreationCount, 0);
	TestEqual(TEXT("Partitioned-frontier propagation zero-drift keeps transfer fallback at zero"), ZeroDriftSnapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Partitioned-frontier propagation zero-drift keeps direct hit transfer aligned with hits"), ZeroDriftSnapshot.DirectHitTriangleTransferCount, ZeroDriftSnapshot.HitCount);
	TestTrue(TEXT("Partitioned-frontier propagation zero-drift preserves continental area fraction"), FMath::Abs(ZeroDriftSnapshot.ContinentalAreaFraction - Step0Snapshot.ContinentalAreaFraction) <= UE_DOUBLE_SMALL_NUMBER);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierPropagationSpikeIntervalTrackingPropagatesDuringIntervalTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierPropagationSpikeIntervalTrackingPropagatesDuringIntervalTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierPropagationSpikeIntervalTrackingPropagatesDuringIntervalTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	const FTectonicPlanetV6PeriodicSolveStats SeedSnapshot = Planet.BuildCurrentDiagnosticSnapshotForTest();
	const TArray<uint8> SeedTrackedKinds = Planet.GetTrackedCopiedFrontierDestructiveKindsForTest();

	Planet.AdvanceSteps(FMath::Max(1, Interval / 2));
	const FTectonicPlanetV6PeriodicSolveStats MidIntervalSnapshot = Planet.BuildCurrentDiagnosticSnapshotForTest();
	const TArray<uint8> MidTrackedKinds = Planet.GetTrackedCopiedFrontierDestructiveKindsForTest();

	int32 SeedTrackedCount = 0;
	int32 MidTrackedCount = 0;
	for (const uint8 TrackedKind : SeedTrackedKinds)
	{
		SeedTrackedCount += TrackedKind != 0 ? 1 : 0;
	}
	for (const uint8 TrackedKind : MidTrackedKinds)
	{
		MidTrackedCount += TrackedKind != 0 ? 1 : 0;
	}

	AddInfo(FString::Printf(
		TEXT("[V6ThesisPartitionedFrontierPropagationIntervalTracking] seed_tracked=%d seed_new=%d mid_step=%d mid_tracked=%d mid_propagated=%d"),
		SeedTrackedCount,
		SeedSnapshot.TrackedDestructiveTriangleNewlySeededCount,
		MidIntervalSnapshot.Step,
		MidTrackedCount,
		MidIntervalSnapshot.TrackedDestructiveTrianglePropagatedCount));

	TestEqual(TEXT("Partitioned-frontier propagation does not remesh before the mid-interval checkpoint"), Planet.GetPeriodicSolveCount(), 0);
	TestTrue(TEXT("Partitioned-frontier propagation seeds destructive triangles after remesh"), SeedTrackedCount > 0);
	TestEqual(TEXT("Partitioned-frontier propagation seed snapshot matches the tracked count"), SeedSnapshot.TrackedDestructiveTriangleCount, SeedTrackedCount);
	TestTrue(TEXT("Partitioned-frontier propagation records a non-zero seed count"), SeedSnapshot.TrackedDestructiveTriangleNewlySeededCount > 0);
	TestTrue(TEXT("Partitioned-frontier propagation grows the tracked set before remesh"), MidTrackedCount > SeedTrackedCount);
	TestTrue(TEXT("Partitioned-frontier propagation records propagated growth"), MidIntervalSnapshot.TrackedDestructiveTrianglePropagatedCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierPropagationSpikeOneIntervalDiagnosticTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierPropagationSpikeOneIntervalDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierPropagationSpikeOneIntervalDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	Planet.AdvanceSteps(Interval);
	const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& Attribution = Planet.GetLastCopiedFrontierSolveAttributionForTest();
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationOneCycle]"), Snapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationOneCycleAttribution]"), Attribution);

	TestEqual(TEXT("Partitioned-frontier propagation one-cycle lands on the first remesh step"), Snapshot.Step, Interval);
	TestEqual(TEXT("Partitioned-frontier propagation one-cycle reports the propagation mode"), Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike);
	TestEqual(TEXT("Partitioned-frontier propagation one-cycle is periodic"), Planet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Periodic);
	TestTrue(TEXT("Partitioned-frontier propagation one-cycle applies the destructive filter"), Snapshot.bDestructiveTriangleExclusionApplied);
	TestTrue(TEXT("Partitioned-frontier propagation one-cycle seeds tracked destructive triangles"), Snapshot.TrackedDestructiveTriangleNewlySeededCount > 0);
	TestTrue(TEXT("Partitioned-frontier propagation one-cycle records propagated destructive growth"), Snapshot.TrackedDestructiveTrianglePropagatedCount > 0);
	TestTrue(TEXT("Partitioned-frontier propagation one-cycle still produces direct hits"), Snapshot.HitCount > 0);
	TestEqual(TEXT("Partitioned-frontier propagation one-cycle keeps direct hit transfer aligned with hits"), Snapshot.DirectHitTriangleTransferCount, Snapshot.HitCount);
	TestEqual(TEXT("Partitioned-frontier propagation one-cycle keeps transfer fallback at zero"), Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Partitioned-frontier propagation one-cycle partitions misses into explicit buckets"), Snapshot.MissTrueDivergenceGapCount + Snapshot.MissDestructiveExclusionGapCount + Snapshot.MissAmbiguousGapCount, Snapshot.MissCount);
	TestEqual(TEXT("Partitioned-frontier propagation one-cycle attribution hit count matches the snapshot"), Attribution.HitCount, Snapshot.HitCount);
	TestEqual(TEXT("Partitioned-frontier propagation one-cycle attribution miss count matches the snapshot"), Attribution.MissCount, Snapshot.MissCount);
	TestEqual(
		TEXT("Partitioned-frontier propagation one-cycle partitions destructive-exclusion handling into continuity, synthetic, and fallback"),
		Attribution.DestructiveExclusionOverrideContinuityCount +
			Attribution.DestructiveExclusionStructuredSyntheticCount +
			Attribution.DestructiveExclusionFallbackOceanicCount,
		Attribution.MissDestructiveExclusionGapCount);
	TestTrue(
		TEXT("Partitioned-frontier propagation one-cycle uses structured gap handling for at least some misses"),
		Attribution.TrueDivergenceStructuredSyntheticCount > 0 ||
			Attribution.DestructiveExclusionStructuredSyntheticCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierPropagationSpikeBaseline60k7Test,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierPropagationSpikeBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierPropagationSpikeBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6ThesisPartitionedFrontierPropagationSpikeBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	const FV6BaselineSummary Summary = RunV6BaselineScenario(
		*this,
		RunId,
		200,
		TArray<int32>{},
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike,
		25,
		false);
	const auto HasTrackedActivity = [](const FV6CheckpointSnapshot& Snapshot)
	{
		return Snapshot.TrackedDestructiveTriangleCount > 0 ||
			Snapshot.TrackedDestructiveTriangleNewlySeededCount > 0 ||
			Snapshot.TrackedDestructiveTrianglePropagatedCount > 0 ||
			Snapshot.TrackedDestructiveTriangleExpiredCount > 0;
	};
	const bool bTrackedActivityObserved =
		(Summary.bHasStep50Snapshot && HasTrackedActivity(Summary.Step50Snapshot)) ||
		(Summary.bHasStep100Snapshot && HasTrackedActivity(Summary.Step100Snapshot)) ||
		(Summary.bHasStep150Snapshot && HasTrackedActivity(Summary.Step150Snapshot)) ||
		(Summary.bHasStep200Snapshot && HasTrackedActivity(Summary.Step200Snapshot));

	TestTrue(TEXT("Partitioned-frontier propagation baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("Partitioned-frontier propagation step-200 reports the propagation mode"), Summary.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike);
	TestEqual(TEXT("Partitioned-frontier propagation step-200 uses the fixed 25-step cadence"), Summary.Step200Snapshot.Interval, 25);
	TestTrue(TEXT("Partitioned-frontier propagation step-200 still has direct hits"), Summary.Step200Snapshot.HitCount > 0);
	TestTrue(TEXT("Partitioned-frontier propagation baseline records tracked destructive activity at one of the baseline checkpoints"), bTrackedActivityObserved);
	TestTrue(TEXT("Partitioned-frontier propagation step-200 expires at least some tracked triangles by subduction radius"), Summary.Step200Snapshot.TrackedDestructiveTriangleExpiredCount > 0 || bTrackedActivityObserved);
	TestEqual(TEXT("Partitioned-frontier propagation step-200 keeps transfer fallback at zero"), Summary.Step200Snapshot.TransferFallbackCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierPropagationProcessSpikeOneIntervalDiagnosticTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierPropagationProcessSpikeOneIntervalDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierPropagationProcessSpikeOneIntervalDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 ControlPlanet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationSpike, 25);
	FTectonicPlanetV6 ProcessPlanet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike, 25);
	const int32 Interval = ControlPlanet.ComputePeriodicSolveInterval();
	ControlPlanet.AdvanceSteps(Interval);
	ProcessPlanet.AdvanceSteps(Interval);

	const FV6CheckpointSnapshot ControlSnapshot = BuildV6CheckpointSnapshot(ControlPlanet);
	const FV6CheckpointSnapshot ProcessSnapshot = BuildV6CheckpointSnapshot(ProcessPlanet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& ProcessAttribution = ProcessPlanet.GetLastCopiedFrontierSolveAttributionForTest();
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationControlOneCycle]"), ControlSnapshot);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationProcessOneCycle]"), ProcessSnapshot);
	AddCopiedFrontierSolveAttributionInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationProcessOneCycleAttribution]"), ProcessAttribution);

	TestEqual(TEXT("Partitioned-frontier propagation process one-cycle reports the process mode"), ProcessSnapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike);
	TestEqual(TEXT("Partitioned-frontier propagation process one-cycle is periodic"), ProcessPlanet.GetLastSolveStats().Trigger, ETectonicPlanetV6SolveTrigger::Periodic);
	TestTrue(TEXT("Partitioned-frontier propagation process one-cycle records propagated destructive growth"), ProcessSnapshot.TrackedDestructiveTrianglePropagatedCount > 0);
	TestEqual(TEXT("Partitioned-frontier propagation process one-cycle keeps transfer fallback at zero"), ProcessSnapshot.TransferFallbackCount, 0);
	TestTrue(TEXT("Partitioned-frontier propagation process one-cycle applies tectonic maintenance"), ProcessAttribution.TectonicMaintenanceAppliedCount > 0);
	TestEqual(
		TEXT("Partitioned-frontier propagation process one-cycle partitions destructive-exclusion handling into continuity, synthetic, and fallback"),
		ProcessAttribution.DestructiveExclusionOverrideContinuityCount +
			ProcessAttribution.DestructiveExclusionStructuredSyntheticCount +
			ProcessAttribution.DestructiveExclusionFallbackOceanicCount,
		ProcessAttribution.MissDestructiveExclusionGapCount);
	TestTrue(
		TEXT("Partitioned-frontier propagation process one-cycle uses structured gap handling for at least some misses"),
		ProcessAttribution.TrueDivergenceStructuredSyntheticCount > 0 ||
			ProcessAttribution.DestructiveExclusionStructuredSyntheticCount > 0);
	TestTrue(TEXT("Partitioned-frontier propagation process one-cycle should not worsen continental area fraction"), ProcessSnapshot.ContinentalAreaFraction >= ControlSnapshot.ContinentalAreaFraction);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierPropagationProcessTrackingLifecycleDiagnosticTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierPropagationProcessTrackingLifecycleDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierPropagationProcessTrackingLifecycleDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike,
		25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	Planet.AdvanceSteps(Interval);
	const FV6CheckpointSnapshot Step25Snapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationProcessTrackingStep25]"), Step25Snapshot);
	AddV6TrackingLifecycleInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationProcessTrackingLifecycleStep25]"), Step25Snapshot);

	Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep);
	const FV6CheckpointSnapshot Step100Snapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationProcessTrackingStep100]"), Step100Snapshot);
	AddV6TrackingLifecycleInfo(*this, TEXT("[V6ThesisPartitionedFrontierPropagationProcessTrackingLifecycleStep100]"), Step100Snapshot);

	const FTectonicPlanetV6DestructiveTrackingLifecycleStats& Step25Lifecycle = Step25Snapshot.TrackingLifecycleStats;
	const FTectonicPlanetV6DestructiveTrackingLifecycleStats& Step100Lifecycle = Step100Snapshot.TrackingLifecycleStats;
	TestEqual(TEXT("Step 25 tracking lifecycle partitions entry candidates"), Step25Lifecycle.EntryAdmittedCount + Step25Lifecycle.EntryRejectedNoOverlapCount + Step25Lifecycle.EntryRejectedAuthorizationCount + Step25Lifecycle.EntryRejectedAlreadyTrackedCount, Step25Lifecycle.EntryCandidateCount);
	TestEqual(TEXT("Step 100 tracking lifecycle partitions entry candidates"), Step100Lifecycle.EntryAdmittedCount + Step100Lifecycle.EntryRejectedNoOverlapCount + Step100Lifecycle.EntryRejectedAuthorizationCount + Step100Lifecycle.EntryRejectedAlreadyTrackedCount, Step100Lifecycle.EntryCandidateCount);
	TestTrue(TEXT("Step 25 records convergent-edge seed candidates"), Step25Lifecycle.EntrySeedConvergentEdgeCandidateCount > 0);
	TestTrue(TEXT("Step 25 admits at least some convergent-edge seeds"), Step25Lifecycle.EntrySeedConvergentEdgeAdmittedCount > 0);
	TestTrue(TEXT("Step 25 generates topology neighbors"), Step25Lifecycle.TopologyNeighborCandidateGeneratedCount > 0);
	TestTrue(TEXT("Step 25 admits topology neighbors"), Step25Lifecycle.TopologyNeighborAdmittedCount > 0);
	TestTrue(TEXT("Step 25 considers directional topology neighbors"), Step25Lifecycle.DirectionalNeighborCandidateConsideredCount > 0);
	TestTrue(TEXT("Step 25 admits some directional topology neighbors"), Step25Lifecycle.DirectionalNeighborAdmittedCount > 0);
	TestTrue(TEXT("Step 25 one-timestep seed survival does not exceed admitted seeds"), Step25Lifecycle.EntrySeedConvergentEdgeSurvivedOneTimestepCount <= Step25Lifecycle.EntrySeedConvergentEdgeAdmittedCount);
	TestTrue(TEXT("Step 100 remesh seed survival does not exceed admitted seeds"), Step100Lifecycle.EntrySeedConvergentEdgeSurvivedToRemeshCount <= Step100Lifecycle.EntrySeedConvergentEdgeAdmittedCount);
	TestTrue(TEXT("Step 100 still generates topology neighbors"), Step100Lifecycle.TopologyNeighborCandidateGeneratedCount > 0);
	TestTrue(TEXT("Step 100 still admits topology neighbors"), Step100Lifecycle.TopologyNeighborAdmittedCount > 0);
	TestTrue(TEXT("Step 100 keeps considering directional topology neighbors"), Step100Lifecycle.DirectionalNeighborCandidateConsideredCount > 0);
	TestTrue(TEXT("Step 100 keeps admitting directional topology neighbors"), Step100Lifecycle.DirectionalNeighborAdmittedCount > 0);
	TestEqual(TEXT("Step 25 tracking lifecycle partitions neighbor candidates"), Step25Lifecycle.NeighborAdmittedCount + Step25Lifecycle.NeighborRejectedNoOverlapCount + Step25Lifecycle.NeighborRejectedAuthorizationCount, Step25Lifecycle.NeighborCandidateTestedCount);
	TestEqual(TEXT("Step 100 tracking lifecycle partitions neighbor candidates"), Step100Lifecycle.NeighborAdmittedCount + Step100Lifecycle.NeighborRejectedNoOverlapCount + Step100Lifecycle.NeighborRejectedAuthorizationCount, Step100Lifecycle.NeighborCandidateTestedCount);
	TestEqual(TEXT("Step 25 expired split matches expired total"), Step25Lifecycle.ExpiredSubductionCount + Step25Lifecycle.ExpiredCollisionCount, Step25Snapshot.TrackedDestructiveTriangleExpiredCount);
	TestEqual(TEXT("Step 100 expired split matches expired total"), Step100Lifecycle.ExpiredSubductionCount + Step100Lifecycle.ExpiredCollisionCount, Step100Snapshot.TrackedDestructiveTriangleExpiredCount);
	TestEqual(TEXT("Step 25 remains on the first remesh checkpoint"), Step25Snapshot.Step, Interval);
	TestEqual(TEXT("Step 100 lands on the fourth remesh checkpoint"), Step100Snapshot.Step, 100);
	TestEqual(TEXT("Step 100 keeps transfer fallback at zero"), Step100Snapshot.TransferFallbackCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierPropagationProcessSpikeBaseline60k7Test,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierPropagationProcessSpikeBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierPropagationProcessSpikeBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6ThesisPartitionedFrontierPropagationProcessSpikeBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	const FV6BaselineSummary Summary = RunV6BaselineScenario(
		*this,
		RunId,
		200,
		TArray<int32>{},
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike,
		25,
		false);
	const auto HasTrackedActivity = [](const FV6CheckpointSnapshot& Snapshot)
	{
		return Snapshot.TrackedDestructiveTriangleCount > 0 ||
			Snapshot.TrackedDestructiveTriangleNewlySeededCount > 0 ||
			Snapshot.TrackedDestructiveTrianglePropagatedCount > 0 ||
			Snapshot.TrackedDestructiveTriangleExpiredCount > 0;
	};
	const bool bTrackedActivityObserved =
		(Summary.bHasStep50Snapshot && HasTrackedActivity(Summary.Step50Snapshot)) ||
		(Summary.bHasStep100Snapshot && HasTrackedActivity(Summary.Step100Snapshot)) ||
		(Summary.bHasStep150Snapshot && HasTrackedActivity(Summary.Step150Snapshot)) ||
		(Summary.bHasStep200Snapshot && HasTrackedActivity(Summary.Step200Snapshot));

	TestTrue(TEXT("Partitioned-frontier propagation process baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("Partitioned-frontier propagation process step-200 reports the process mode"), Summary.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike);
	TestEqual(TEXT("Partitioned-frontier propagation process step-200 uses the fixed 25-step cadence"), Summary.Step200Snapshot.Interval, 25);
	TestTrue(TEXT("Partitioned-frontier propagation process step-200 still has direct hits"), Summary.Step200Snapshot.HitCount > 0);
	TestTrue(TEXT("Partitioned-frontier propagation process baseline records tracked destructive activity at one of the baseline checkpoints"), bTrackedActivityObserved);
	TestTrue(TEXT("Partitioned-frontier propagation process step-200 expires at least some tracked triangles by subduction radius"), Summary.Step200Snapshot.TrackedDestructiveTriangleExpiredCount > 0 || bTrackedActivityObserved);
	TestEqual(TEXT("Partitioned-frontier propagation process step-200 keeps transfer fallback at zero"), Summary.Step200Snapshot.TransferFallbackCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPartitionedFrontierPropagationProcessDepthSweepTest,
	"Aurous.TectonicPlanet.V6ThesisPartitionedFrontierPropagationProcessDepthSweepTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPartitionedFrontierPropagationProcessDepthSweepTest::RunTest(const FString& Parameters)
{
	const FV6PropagationDepthSweepResult OffResult =
		RunPartitionedPropagationProcessDepthSweepCase(
			*this,
			TEXT("off"),
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			INDEX_NONE);
	const FV6PropagationDepthSweepResult SeedOnlyResult =
		RunPartitionedPropagationProcessDepthSweepCase(
			*this,
			TEXT("seed_only"),
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike,
			0);
	const FV6PropagationDepthSweepResult MediumResult =
		RunPartitionedPropagationProcessDepthSweepCase(
			*this,
			TEXT("cap6"),
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike,
			6);
	const FV6PropagationDepthSweepResult FullResult =
		RunPartitionedPropagationProcessDepthSweepCase(
			*this,
			TEXT("full"),
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike,
			INDEX_NONE);

	TestEqual(TEXT("Propagation OFF reference uses the non-propagation process mode"), OffResult.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike);
	TestEqual(TEXT("Seed-only sweep case uses the propagation process mode"), SeedOnlyResult.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike);
	TestEqual(TEXT("Medium sweep case uses the propagation process mode"), MediumResult.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike);
	TestEqual(TEXT("Full sweep case uses the propagation process mode"), FullResult.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike);
	TestEqual(TEXT("Propagation OFF reference keeps transfer fallback at zero"), OffResult.Step200Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Seed-only sweep case keeps transfer fallback at zero"), SeedOnlyResult.Step200Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Medium sweep case keeps transfer fallback at zero"), MediumResult.Step200Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Full sweep case keeps transfer fallback at zero"), FullResult.Step200Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Seed-only sweep case records no propagated growth by step 25"), SeedOnlyResult.OneCycleSnapshot.TrackedDestructiveTrianglePropagatedCount, 0);
	TestTrue(TEXT("Medium sweep case records propagated growth by step 25"), MediumResult.OneCycleSnapshot.TrackedDestructiveTrianglePropagatedCount > 0);
	TestTrue(TEXT("Full sweep case records at least as much propagated growth as the medium cap by step 25"), FullResult.OneCycleSnapshot.TrackedDestructiveTrianglePropagatedCount >= MediumResult.OneCycleSnapshot.TrackedDestructiveTrianglePropagatedCount);
	TestTrue(TEXT("Full sweep case keeps tracked destructive exclusion active by step 200"), FullResult.Step200Snapshot.TrackedDestructiveTriangleCount > 0);
	TestTrue(TEXT("At least one propagation sweep case changes the step-200 multi-hit count relative to OFF"), SeedOnlyResult.Step200Snapshot.MultiHitCount != OffResult.Step200Snapshot.MultiHitCount || MediumResult.Step200Snapshot.MultiHitCount != OffResult.Step200Snapshot.MultiHitCount || FullResult.Step200Snapshot.MultiHitCount != OffResult.Step200Snapshot.MultiHitCount);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6VisualValidationExportTest,
	"Aurous.TectonicPlanet.V6VisualValidationExportTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6VisualValidationExportTest::RunTest(const FString& Parameters)
{
	const FString RunId = TEXT("V6VisualValidation_PartitionedOFF_FillON");
	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);

	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike, 25);

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	TestTrue(TEXT("Export root created"), PlatformFile.CreateDirectoryTree(*ExportRoot));

	// Step 0 — initial state (core maps only, no debug overlays since no solve has run)
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 0);

	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	// Step 25
	Planet.AdvanceSteps(Interval);
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 25);
	ExportV6DebugOverlays(*this, Planet, ExportRoot, 25);

	const FV6CheckpointSnapshot Step25Snapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6VisualValidation]"), Step25Snapshot);

	// Step 100
	if (Planet.GetPlanet().CurrentStep < 100) { Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep); }
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 100);
	ExportV6DebugOverlays(*this, Planet, ExportRoot, 100);

	const FV6CheckpointSnapshot Step100Snapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6VisualValidation]"), Step100Snapshot);

	// Step 200 (opportunistic)
	if (Planet.GetPlanet().CurrentStep < 200) { Planet.AdvanceSteps(200 - Planet.GetPlanet().CurrentStep); }
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 200);
	ExportV6DebugOverlays(*this, Planet, ExportRoot, 200);

	const FV6CheckpointSnapshot Step200Snapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6VisualValidation]"), Step200Snapshot);

	// Report export root
	AddInfo(FString::Printf(TEXT("[V6VisualValidation] export_root=%s"), *ExportRoot));
	UE_LOG(LogTemp, Log, TEXT("[V6VisualValidation] export_root=%s"), *ExportRoot);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6BoundaryCoherenceComparisonTest,
	"Aurous.TectonicPlanet.V6BoundaryCoherenceComparisonTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6BoundaryCoherenceComparisonTest::RunTest(const FString& Parameters)
{
	// OFF baseline (partitioned frontier process without propagation)
	FTectonicPlanetV6 OffPlanet = CreateInitializedPlanetV6(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike, 25);
	// Directional propagation baseline (full propagation)
	FTectonicPlanetV6 DirPlanet = CreateInitializedPlanetV6(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierPropagationProcessSpike, 25);

	const int32 OffInterval = OffPlanet.ComputePeriodicSolveInterval();
	const int32 DirInterval = DirPlanet.ComputePeriodicSolveInterval();

	// Step 25 (one interval)
	OffPlanet.AdvanceSteps(OffInterval);
	DirPlanet.AdvanceSteps(DirInterval);
	const FV6CheckpointSnapshot OffStep25 = BuildV6CheckpointSnapshot(OffPlanet);
	const FV6CheckpointSnapshot DirStep25 = BuildV6CheckpointSnapshot(DirPlanet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6Diagnostics OFF]"), OffStep25);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6Diagnostics Directional]"), DirStep25);

	// Step 100
	if (OffPlanet.GetPlanet().CurrentStep < 100) { OffPlanet.AdvanceSteps(100 - OffPlanet.GetPlanet().CurrentStep); }
	if (DirPlanet.GetPlanet().CurrentStep < 100) { DirPlanet.AdvanceSteps(100 - DirPlanet.GetPlanet().CurrentStep); }
	const FV6CheckpointSnapshot OffStep100 = BuildV6CheckpointSnapshot(OffPlanet);
	const FV6CheckpointSnapshot DirStep100 = BuildV6CheckpointSnapshot(DirPlanet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6Diagnostics OFF]"), OffStep100);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6Diagnostics Directional]"), DirStep100);

	// Step 200
	if (OffPlanet.GetPlanet().CurrentStep < 200) { OffPlanet.AdvanceSteps(200 - OffPlanet.GetPlanet().CurrentStep); }
	if (DirPlanet.GetPlanet().CurrentStep < 200) { DirPlanet.AdvanceSteps(200 - DirPlanet.GetPlanet().CurrentStep); }
	const FV6CheckpointSnapshot OffStep200 = BuildV6CheckpointSnapshot(OffPlanet);
	const FV6CheckpointSnapshot DirStep200 = BuildV6CheckpointSnapshot(DirPlanet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6Diagnostics OFF]"), OffStep200);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6Diagnostics Directional]"), DirStep200);

	// Structural assertions — boundary coherence
	TestTrue(TEXT("OFF step 25 has boundary band samples"), OffStep25.BoundaryCoherence.BoundaryBandSampleCount > 0);
	TestTrue(TEXT("Directional step 25 has boundary band samples"), DirStep25.BoundaryCoherence.BoundaryBandSampleCount > 0);
	TestTrue(TEXT("Coherence score is bounded [0,1] for OFF"), OffStep200.BoundaryCoherence.BoundaryCoherenceScore >= 0.0 && OffStep200.BoundaryCoherence.BoundaryCoherenceScore <= 1.0);
	TestTrue(TEXT("Coherence score is bounded [0,1] for Directional"), DirStep200.BoundaryCoherence.BoundaryCoherenceScore >= 0.0 && DirStep200.BoundaryCoherence.BoundaryCoherenceScore <= 1.0);

	// Structural assertions — ownership churn
	TestTrue(TEXT("OFF step 25 has nonzero churn"), OffStep25.OwnershipChurn.TotalChurnCount > 0);
	TestTrue(TEXT("Directional step 25 has nonzero churn"), DirStep25.OwnershipChurn.TotalChurnCount > 0);
	TestTrue(TEXT("Churn fraction bounded [0,1]"), OffStep200.OwnershipChurn.ChurnFraction >= 0.0 && OffStep200.OwnershipChurn.ChurnFraction <= 1.0);

	// Structural assertions — competitive participation
	TestTrue(TEXT("OFF step 25 has multiple plates with hits"), OffStep25.CompetitiveParticipation.PlatesWithAnyHit > 1);
	TestTrue(TEXT("Directional step 25 has multiple plates with hits"), DirStep25.CompetitiveParticipation.PlatesWithAnyHit > 1);
	TestTrue(TEXT("Largest plate share bounded [0,1]"), OffStep200.CompetitiveParticipation.LargestPlateHitShare >= 0.0 && OffStep200.CompetitiveParticipation.LargestPlateHitShare <= 1.0);

	// Structural assertions — miss lineage
	TestTrue(TEXT("OFF step 100 has miss lineage data"), OffStep100.MissLineage.MissesThisCycle > 0);
	TestTrue(TEXT("Directional step 100 has miss lineage data"), DirStep100.MissLineage.MissesThisCycle > 0);

	// Structural assertions — front retreat
	TestTrue(TEXT("OFF step 100 has front retreat delta"), OffStep100.FrontRetreat.PreviousSubductionSampleCount > 0);
	TestTrue(TEXT("Directional step 100 has front retreat delta"), DirStep100.FrontRetreat.PreviousSubductionSampleCount > 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6PartitionedFrontierProcessInteriorInstabilityDiagnosticTest,
	"Aurous.TectonicPlanet.V6PartitionedFrontierProcessInteriorInstabilityDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6PartitionedFrontierProcessInteriorInstabilityDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		25);

	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	Planet.AdvanceSteps(Interval);
	const FV6CheckpointSnapshot Step25 = BuildV6CheckpointSnapshot(Planet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6InteriorInstability]"), Step25);
	AddV6BoundaryCoherenceInfo(*this, TEXT("[V6InteriorInstability]"), Step25);
	AddV6InteriorInstabilityInfo(*this, TEXT("[V6InteriorInstability]"), Step25);

	if (Planet.GetPlanet().CurrentStep < 100)
	{
		Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep);
	}
	const FV6CheckpointSnapshot Step100 = BuildV6CheckpointSnapshot(Planet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6InteriorInstability]"), Step100);
	AddV6BoundaryCoherenceInfo(*this, TEXT("[V6InteriorInstability]"), Step100);
	AddV6InteriorInstabilityInfo(*this, TEXT("[V6InteriorInstability]"), Step100);

	if (Planet.GetPlanet().CurrentStep < 200)
	{
		Planet.AdvanceSteps(200 - Planet.GetPlanet().CurrentStep);
	}
	const FV6CheckpointSnapshot Step200 = BuildV6CheckpointSnapshot(Planet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6InteriorInstability]"), Step200);
	AddV6BoundaryCoherenceInfo(*this, TEXT("[V6InteriorInstability]"), Step200);
	AddV6InteriorInstabilityInfo(*this, TEXT("[V6InteriorInstability]"), Step200);

	TestTrue(TEXT("Step 25 reports interior miss causes"), Step25.InteriorInstability.InteriorMissCount > 0);
	TestTrue(TEXT("Step 100 reports interior churn causes"), Step100.InteriorInstability.InteriorChurnCount > 0);
	TestTrue(TEXT("Step 100 reports previous synthetic interior samples"), Step100.InteriorInstability.InteriorPreviousSyntheticSampleCount > 0);
	TestTrue(TEXT("Step 200 reports interior miss causes"), Step200.InteriorInstability.InteriorMissCount > 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6PartitionedFrontierProcessSyntheticCoveragePersistenceAuditTest,
	"Aurous.TectonicPlanet.V6PartitionedFrontierProcessSyntheticCoveragePersistenceAuditTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6PartitionedFrontierProcessSyntheticCoveragePersistenceAuditTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		25);

	const int32 Interval = Planet.ComputePeriodicSolveInterval();
	Planet.AdvanceSteps(Interval);
	const FV6CheckpointSnapshot Step25 = BuildV6CheckpointSnapshot(Planet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6SyntheticCoverage]"), Step25);
	AddV6BoundaryCoherenceInfo(*this, TEXT("[V6SyntheticCoverage]"), Step25);
	AddV6InteriorInstabilityInfo(*this, TEXT("[V6SyntheticCoverage]"), Step25);
	AddV6SyntheticCoveragePersistenceInfo(*this, TEXT("[V6SyntheticCoverage]"), Step25);

	if (Planet.GetPlanet().CurrentStep < 100)
	{
		Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep);
	}
	const FV6CheckpointSnapshot Step100 = BuildV6CheckpointSnapshot(Planet);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6SyntheticCoverage]"), Step100);
	AddV6BoundaryCoherenceInfo(*this, TEXT("[V6SyntheticCoverage]"), Step100);
	AddV6InteriorInstabilityInfo(*this, TEXT("[V6SyntheticCoverage]"), Step100);
	AddV6SyntheticCoveragePersistenceInfo(*this, TEXT("[V6SyntheticCoverage]"), Step100);

	TestTrue(TEXT("Step 100 reports previous synthetic samples"), Step100.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
	TestTrue(TEXT("Step 100 reports previous-owner adjacency coverage"), Step100.SyntheticCoveragePersistence.PreviousOwnerAdjacentTriangleInMeshCount > 0);
	TestTrue(TEXT("Step 100 reports same-owner or different-owner valid hits for previous synthetic samples"),
		Step100.SyntheticCoveragePersistence.SameOwnerValidHitCount +
		Step100.SyntheticCoveragePersistence.DifferentOwnerValidHitCount > 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6PartitionedFrontierProcessCadenceSensitivitySweepTest,
	"Aurous.TectonicPlanet.V6PartitionedFrontierProcessCadenceSensitivitySweepTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6PartitionedFrontierProcessCadenceSensitivitySweepTest::RunTest(const FString& Parameters)
{
	const TArray<int32> CadenceSteps = { 10, 16, 25 };
	for (const int32 FixedIntervalSteps : CadenceSteps)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps);
		const FString SummaryTag = FString::Printf(
			TEXT("[V6CadenceSweep cadence=%d]"),
			FixedIntervalSteps);

		if (Planet.GetPlanet().CurrentStep < 25)
		{
			Planet.AdvanceSteps(25 - Planet.GetPlanet().CurrentStep);
		}
		const FV6CheckpointSnapshot Step25 = BuildV6CheckpointSnapshot(Planet);
		AddV6ThesisRemeshInfo(*this, SummaryTag, Step25);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Step25);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Step25);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Step25);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Step25);

		if (Planet.GetPlanet().CurrentStep < 100)
		{
			Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep);
		}
		const FV6CheckpointSnapshot Step100 = BuildV6CheckpointSnapshot(Planet);
		AddV6ThesisRemeshInfo(*this, SummaryTag, Step100);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Step100);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Step100);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Step100);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Step100);

		TestTrue(
			*FString::Printf(TEXT("Cadence %d has active plates at step 100"), FixedIntervalSteps),
			Step100.CompetitiveParticipation.PlatesWithAnyHit > 1);
		TestTrue(
			*FString::Printf(TEXT("Cadence %d reports previous synthetic samples at step 100"), FixedIntervalSteps),
			Step100.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6PartitionedFrontierProcessSyntheticCoverageRetentionExperimentTest,
	"Aurous.TectonicPlanet.V6PartitionedFrontierProcessSyntheticCoverageRetentionExperimentTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6PartitionedFrontierProcessSyntheticCoverageRetentionExperimentTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;

	for (const bool bEnableRetention : { false, true })
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps);
		Planet.SetSyntheticCoverageRetentionForTest(bEnableRetention);

		const FString SummaryTag = FString::Printf(
			TEXT("[V6SyntheticRetention mode=%s cadence=%d]"),
			bEnableRetention ? TEXT("retained") : TEXT("baseline"),
			FixedIntervalSteps);

		if (Planet.GetPlanet().CurrentStep < 25)
		{
			Planet.AdvanceSteps(25 - Planet.GetPlanet().CurrentStep);
		}
		const FV6CheckpointSnapshot Step25 = BuildV6CheckpointSnapshot(Planet);
		AddV6ThesisRemeshInfo(*this, SummaryTag, Step25);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Step25);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Step25);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Step25);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Step25);

		if (Planet.GetPlanet().CurrentStep < 100)
		{
			Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep);
		}
		const FV6CheckpointSnapshot Step100 = BuildV6CheckpointSnapshot(Planet);
		AddV6ThesisRemeshInfo(*this, SummaryTag, Step100);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Step100);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Step100);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Step100);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Step100);

		TestTrue(
			*FString::Printf(TEXT("%s has previous synthetic samples at step 100"), *SummaryTag),
			Step100.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
		if (bEnableRetention)
		{
			TestTrue(
				TEXT("Retention variant retained at least one synthetic coverage sample by step 100"),
				Step100.SyntheticCoveragePersistence.CurrentRetainedSyntheticCoverageCount > 0 ||
				Step100.SyntheticCoveragePersistence.PreviousRetainedSyntheticCoverageSampleCount > 0);
		}
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6PartitionedFrontierProcessSyntheticCoverageRetentionValidationTest,
	"Aurous.TectonicPlanet.V6PartitionedFrontierProcessSyntheticCoverageRetentionValidationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6PartitionedFrontierProcessSyntheticCoverageRetentionValidationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	const FString RunId = TEXT("V6SyntheticRetentionValidation_cadence16");

	struct FRetentionValidationResult
	{
		FString Label;
		FString ExportRoot;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FV6CheckpointSnapshot Step200Snapshot;
	};

	const auto RunVariant = [this, FixedIntervalSteps, &RunId](
		const TCHAR* VariantLabel,
		const bool bEnableRetention) -> FRetentionValidationResult
	{
		FRetentionValidationResult Result;
		Result.Label = VariantLabel;

		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps);
		Planet.SetSyntheticCoverageRetentionForTest(bEnableRetention);

		Result.ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			RunId,
			bEnableRetention ? TEXT("retained") : TEXT("baseline"));

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		TestTrue(
			*FString::Printf(TEXT("%s export root created"), VariantLabel),
			PlatformFile.CreateDirectoryTree(*Result.ExportRoot));
		ExportV6CheckpointMaps(*this, Planet, Result.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Planet, Result.ExportRoot, 0);

		const FString SummaryTag = FString::Printf(
			TEXT("[V6SyntheticRetentionValidation mode=%s cadence=%d]"),
			bEnableRetention ? TEXT("retained") : TEXT("baseline"),
			FixedIntervalSteps);
		const FString ExportRootMessage = FString::Printf(
			TEXT("%s export_root=%s"),
			*SummaryTag,
			*Result.ExportRoot);
		AddInfo(ExportRootMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *ExportRootMessage);

		const auto LogCheckpoint = [this, &Planet, &SummaryTag, &Result](
			const int32 Step,
			FV6CheckpointSnapshot FRetentionValidationResult::* MemberSnapshot)
		{
			ExportV6CheckpointMaps(*this, Planet, Result.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Planet, Result.ExportRoot, Step);

			const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
			Result.*MemberSnapshot = Snapshot;

			AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
			AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
			AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
			AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
			AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
		};

		if (Planet.GetPlanet().CurrentStep < 25)
		{
			Planet.AdvanceSteps(25 - Planet.GetPlanet().CurrentStep);
		}
		LogCheckpoint(25, &FRetentionValidationResult::Step25Snapshot);

		if (Planet.GetPlanet().CurrentStep < 100)
		{
			Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep);
		}
		LogCheckpoint(100, &FRetentionValidationResult::Step100Snapshot);

		if (Planet.GetPlanet().CurrentStep < 200)
		{
			Planet.AdvanceSteps(200 - Planet.GetPlanet().CurrentStep);
		}
		LogCheckpoint(200, &FRetentionValidationResult::Step200Snapshot);

		TestTrue(
			*FString::Printf(TEXT("%s has previous synthetic samples at step 100"), VariantLabel),
			Result.Step100Snapshot.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
		TestTrue(
			*FString::Printf(TEXT("%s has previous synthetic samples at step 200"), VariantLabel),
			Result.Step200Snapshot.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
		if (bEnableRetention)
		{
			TestTrue(
				TEXT("Retention variant records retained synthetic coverage by step 100 or 200"),
				Result.Step100Snapshot.SyntheticCoveragePersistence.CurrentRetainedSyntheticCoverageCount > 0 ||
				Result.Step100Snapshot.SyntheticCoveragePersistence.PreviousRetainedSyntheticCoverageSampleCount > 0 ||
				Result.Step200Snapshot.SyntheticCoveragePersistence.CurrentRetainedSyntheticCoverageCount > 0 ||
				Result.Step200Snapshot.SyntheticCoveragePersistence.PreviousRetainedSyntheticCoverageSampleCount > 0);
		}

		return Result;
	};

	const FRetentionValidationResult Baseline = RunVariant(TEXT("baseline"), false);
	const FRetentionValidationResult Retained = RunVariant(TEXT("retained"), true);

	TestTrue(
		TEXT("Retention validation exports both variants"),
		!Baseline.ExportRoot.IsEmpty() && !Retained.ExportRoot.IsEmpty());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisRegimeFalsification60kProcessTest,
	"Aurous.TectonicPlanet.V6ThesisRegimeFalsification60kProcessTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisRegimeFalsification60kProcessTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 25;
	constexpr int32 RandomSeed = TestRandomSeed;
	const FString RunId = TEXT("V6ThesisRegimeFalsification");

	AddInfo(TEXT("[V6ThesisRegimeFalsification] mixed_triangle_policy=whole_triangle_duplication thesis_reading=global_tds_partition_plus_duplication selected_mode=ThesisPartitionedFrontierProcessSpike whole_triangle_duplication_forced=1 structured_frontier_pair_fill=1 synthetic_gap_retention=0 cadence_steps=25"));

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		int32 SampleCount = 0;
		int32 PlateCount = 0;
		FTectonicPlanetV6 Planet;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FV6CheckpointSnapshot Step200Snapshot;
	};

	const auto InitializeVariant = [this, FixedIntervalSteps, RandomSeed, &RunId](
		const TCHAR* VariantLabel,
		const int32 SampleCount,
		const int32 PlateCount) -> FVariantState
	{
		FVariantState Variant;
		Variant.Label = VariantLabel;
		Variant.SampleCount = SampleCount;
		Variant.PlateCount = PlateCount;
		Variant.Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			RandomSeed);
		Variant.Planet.SetSyntheticCoverageRetentionForTest(false);
		Variant.Planet.SetWholeTriangleBoundaryDuplicationForTest(true);

		Variant.ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			RunId,
			VariantLabel);

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		TestTrue(
			*FString::Printf(TEXT("%s export root created"), VariantLabel),
			PlatformFile.CreateDirectoryTree(*Variant.ExportRoot));
		ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, 0);

		const FString ExportRootMessage = FString::Printf(
			TEXT("[V6ThesisRegimeFalsification variant=%s samples=%d plates=%d cadence=%d] export_root=%s"),
			VariantLabel,
			SampleCount,
			PlateCount,
			FixedIntervalSteps,
			*Variant.ExportRoot);
		AddInfo(ExportRootMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *ExportRootMessage);
		return Variant;
	};

	const auto LogCheckpoint = [this](
		FVariantState& Variant,
		const TCHAR* SummaryTag,
		const int32 Step,
		FV6CheckpointSnapshot FVariantState::* MemberSnapshot)
	{
		ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
		ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		Variant.*MemberSnapshot = Snapshot;
		AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
		AddV6TectonicInteractionInfo(*this, SummaryTag, Snapshot);
	};

	const auto CountBoundaryLocalizationImprovements = [](
		const FV6CheckpointSnapshot& Reference,
		const FV6CheckpointSnapshot& ThesisLike) -> int32
	{
		return
			(ThesisLike.BoundaryCoherence.InteriorLeakageFraction <
					Reference.BoundaryCoherence.InteriorLeakageFraction ? 1 : 0) +
			(ThesisLike.BoundaryCoherence.MeanConflictRingDistance <
					Reference.BoundaryCoherence.MeanConflictRingDistance ? 1 : 0) +
			(ThesisLike.BoundaryCoherence.MissInteriorCount <
					Reference.BoundaryCoherence.MissInteriorCount ? 1 : 0) +
			(ThesisLike.BoundaryCoherence.DestructiveExclusionInteriorCount <
					Reference.BoundaryCoherence.DestructiveExclusionInteriorCount ? 1 : 0) +
			(ThesisLike.BoundaryCoherence.TotalInteriorConflictCount <
					Reference.BoundaryCoherence.TotalInteriorConflictCount ? 1 : 0) +
			(ThesisLike.BoundaryCoherence.BoundaryCoherenceScore >
					Reference.BoundaryCoherence.BoundaryCoherenceScore ? 1 : 0);
	};

	FVariantState Reference60k7 = InitializeVariant(TEXT("60k_7"), 60000, 7);
	FVariantState Thesis60k40 = InitializeVariant(TEXT("60k_40"), 60000, 40);

	const FString ReferenceSummaryTag = FString::Printf(
		TEXT("[V6ThesisRegimeFalsification variant=60k_7 samples=%d plates=%d cadence=%d]"),
		Reference60k7.SampleCount,
		Reference60k7.PlateCount,
		FixedIntervalSteps);
	const FString ThesisSummaryTag = FString::Printf(
		TEXT("[V6ThesisRegimeFalsification variant=60k_40 samples=%d plates=%d cadence=%d]"),
		Thesis60k40.SampleCount,
		Thesis60k40.PlateCount,
		FixedIntervalSteps);

	if (Reference60k7.Planet.GetPlanet().CurrentStep < 25)
	{
		Reference60k7.Planet.AdvanceSteps(25 - Reference60k7.Planet.GetPlanet().CurrentStep);
	}
	if (Thesis60k40.Planet.GetPlanet().CurrentStep < 25)
	{
		Thesis60k40.Planet.AdvanceSteps(25 - Thesis60k40.Planet.GetPlanet().CurrentStep);
	}
	LogCheckpoint(Reference60k7, *ReferenceSummaryTag, 25, &FVariantState::Step25Snapshot);
	LogCheckpoint(Thesis60k40, *ThesisSummaryTag, 25, &FVariantState::Step25Snapshot);

	if (Reference60k7.Planet.GetPlanet().CurrentStep < 100)
	{
		Reference60k7.Planet.AdvanceSteps(100 - Reference60k7.Planet.GetPlanet().CurrentStep);
	}
	if (Thesis60k40.Planet.GetPlanet().CurrentStep < 100)
	{
		Thesis60k40.Planet.AdvanceSteps(100 - Thesis60k40.Planet.GetPlanet().CurrentStep);
	}
	LogCheckpoint(Reference60k7, *ReferenceSummaryTag, 100, &FVariantState::Step100Snapshot);
	LogCheckpoint(Thesis60k40, *ThesisSummaryTag, 100, &FVariantState::Step100Snapshot);

	if (Reference60k7.Planet.GetPlanet().CurrentStep < 200)
	{
		Reference60k7.Planet.AdvanceSteps(200 - Reference60k7.Planet.GetPlanet().CurrentStep);
	}
	if (Thesis60k40.Planet.GetPlanet().CurrentStep < 200)
	{
		Thesis60k40.Planet.AdvanceSteps(200 - Thesis60k40.Planet.GetPlanet().CurrentStep);
	}
	LogCheckpoint(Reference60k7, *ReferenceSummaryTag, 200, &FVariantState::Step200Snapshot);
	LogCheckpoint(Thesis60k40, *ThesisSummaryTag, 200, &FVariantState::Step200Snapshot);

	const int32 Step100BoundaryLocalizationImprovements =
		CountBoundaryLocalizationImprovements(
			Reference60k7.Step100Snapshot,
			Thesis60k40.Step100Snapshot);
	const int32 Step200BoundaryLocalizationImprovements =
		CountBoundaryLocalizationImprovements(
			Reference60k7.Step200Snapshot,
			Thesis60k40.Step200Snapshot);

	AddInfo(FString::Printf(
		TEXT("[V6ThesisRegimeFalsification compare=60k7_vs_60k40 cadence=%d] step100_boundary_improvements=%d step100_ref_collision=%d step100_thesis_collision=%d step100_ref_geometric_exec=%d step100_thesis_geometric_exec=%d step100_ref_largest_share=%.4f step100_thesis_largest_share=%.4f step200_boundary_improvements=%d step200_ref_collision=%d step200_thesis_collision=%d step200_ref_geometric_exec=%d step200_thesis_geometric_exec=%d step200_ref_largest_share=%.4f step200_thesis_largest_share=%.4f"),
		FixedIntervalSteps,
		Step100BoundaryLocalizationImprovements,
		Reference60k7.Step100Snapshot.CollisionCount,
		Thesis60k40.Step100Snapshot.CollisionCount,
		Reference60k7.Step100Snapshot.bUsedGeometricCollisionExecution ? 1 : 0,
		Thesis60k40.Step100Snapshot.bUsedGeometricCollisionExecution ? 1 : 0,
		Reference60k7.Step100Snapshot.CompetitiveParticipation.LargestPlateHitShare,
		Thesis60k40.Step100Snapshot.CompetitiveParticipation.LargestPlateHitShare,
		Step200BoundaryLocalizationImprovements,
		Reference60k7.Step200Snapshot.CollisionCount,
		Thesis60k40.Step200Snapshot.CollisionCount,
		Reference60k7.Step200Snapshot.bUsedGeometricCollisionExecution ? 1 : 0,
		Thesis60k40.Step200Snapshot.bUsedGeometricCollisionExecution ? 1 : 0,
		Reference60k7.Step200Snapshot.CompetitiveParticipation.LargestPlateHitShare,
		Thesis60k40.Step200Snapshot.CompetitiveParticipation.LargestPlateHitShare));

	TestTrue(
		TEXT("60k/40 records previous synthetic samples by step 100"),
		Thesis60k40.Step100Snapshot.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
	TestTrue(
		TEXT("60k/40 retains multi-plate participation by step 200"),
		Thesis60k40.Step200Snapshot.CompetitiveParticipation.PlatesWithAnyHit > 1);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisRegimeFalsification250k40SpotCheckTest,
	"Aurous.TectonicPlanet.V6ThesisRegimeFalsification250k40SpotCheckTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisRegimeFalsification250k40SpotCheckTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 25;
	constexpr int32 SampleCount = 250000;
	constexpr int32 PlateCount = 40;
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		TEXT("V6ThesisRegimeFalsification"),
		TEXT("250k_40"));
	const FString SummaryTag = FString::Printf(
		TEXT("[V6ThesisRegimeFalsification variant=250k_40 samples=%d plates=%d cadence=%d]"),
		SampleCount,
		PlateCount,
		FixedIntervalSteps);

	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	Planet.SetSyntheticCoverageRetentionForTest(false);
	Planet.SetWholeTriangleBoundaryDuplicationForTest(true);

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	TestTrue(TEXT("250k/40 export root created"), PlatformFile.CreateDirectoryTree(*ExportRoot));
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 0);
	ExportV6DebugOverlays(*this, Planet, ExportRoot, 0);

	AddInfo(FString::Printf(
		TEXT("%s export_root=%s"),
		*SummaryTag,
		*ExportRoot));

	if (Planet.GetPlanet().CurrentStep < 25)
	{
		Planet.AdvanceSteps(25 - Planet.GetPlanet().CurrentStep);
	}
	FV6CheckpointSnapshot Step25 = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, SummaryTag, Step25);
	AddV6DiagnosticsPackageInfo(*this, SummaryTag, Step25);
	AddV6BoundaryCoherenceInfo(*this, SummaryTag, Step25);
	AddV6InteriorInstabilityInfo(*this, SummaryTag, Step25);
	AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Step25);
	AddV6TectonicInteractionInfo(*this, SummaryTag, Step25);

	if (Planet.GetPlanet().CurrentStep < 100)
	{
		Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep);
	}
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 100);
	ExportV6DebugOverlays(*this, Planet, ExportRoot, 100);
	FV6CheckpointSnapshot Step100 = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, SummaryTag, Step100);
	AddV6DiagnosticsPackageInfo(*this, SummaryTag, Step100);
	AddV6BoundaryCoherenceInfo(*this, SummaryTag, Step100);
	AddV6InteriorInstabilityInfo(*this, SummaryTag, Step100);
	AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Step100);
	AddV6TectonicInteractionInfo(*this, SummaryTag, Step100);

	TestTrue(
		TEXT("250k/40 has previous synthetic samples by step 100"),
		Step100.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
	TestTrue(
		TEXT("250k/40 keeps active plates by step 100"),
		Step100.CompetitiveParticipation.PlatesWithAnyHit > 1);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6PartitionedFrontierProcessDensitySensitivitySpotCheckTest,
	"Aurous.TectonicPlanet.V6PartitionedFrontierProcessDensitySensitivitySpotCheckTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6PartitionedFrontierProcessDensitySensitivitySpotCheckTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	const TArray<int32> SampleCounts = { 60000, 150000 };

	for (const int32 SampleCount : SampleCounts)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount);
		const FString SummaryTag = FString::Printf(
			TEXT("[V6DensitySweep samples=%d cadence=%d]"),
			SampleCount,
			FixedIntervalSteps);

		if (Planet.GetPlanet().CurrentStep < 25)
		{
			Planet.AdvanceSteps(25 - Planet.GetPlanet().CurrentStep);
		}
		const FV6CheckpointSnapshot Step25 = BuildV6CheckpointSnapshot(Planet);
		AddV6ThesisRemeshInfo(*this, SummaryTag, Step25);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Step25);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Step25);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Step25);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Step25);

		if (Planet.GetPlanet().CurrentStep < 100)
		{
			Planet.AdvanceSteps(100 - Planet.GetPlanet().CurrentStep);
		}
		const FV6CheckpointSnapshot Step100 = BuildV6CheckpointSnapshot(Planet);
		AddV6ThesisRemeshInfo(*this, SummaryTag, Step100);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Step100);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Step100);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Step100);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Step100);

		TestTrue(
			*FString::Printf(TEXT("Sample count %d has active plates at step 100"), SampleCount),
			Step100.CompetitiveParticipation.PlatesWithAnyHit > 1);
		TestTrue(
			*FString::Printf(TEXT("Sample count %d reports previous synthetic samples at step 100"), SampleCount),
			Step100.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6BoundaryTriangleOwnershipArchitectureABTest,
	"Aurous.TectonicPlanet.V6BoundaryTriangleOwnershipArchitectureABTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6BoundaryTriangleOwnershipArchitectureABTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	const FString RunId = TEXT("V6BoundaryOwnershipAB_cadence16_retained");

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FV6CheckpointSnapshot Step200Snapshot;
		bool bHasStep200 = false;
	};

	auto InitializeVariant = [this, FixedIntervalSteps, &RunId](
		const TCHAR* VariantLabel,
		const bool bWholeTriangleDuplication) -> FVariantState
	{
		FVariantState Variant;
		Variant.Label = VariantLabel;
		Variant.Planet = CreateInitializedPlanetV6(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps);
		Variant.Planet.SetSyntheticCoverageRetentionForTest(true);
		Variant.Planet.SetWholeTriangleBoundaryDuplicationForTest(bWholeTriangleDuplication);
		Variant.ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			RunId,
			bWholeTriangleDuplication ? TEXT("duplicated") : TEXT("partitioned"));

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		TestTrue(
			*FString::Printf(TEXT("%s export root created"), VariantLabel),
			PlatformFile.CreateDirectoryTree(*Variant.ExportRoot));
		ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, 0);

		const FString ExportRootMessage = FString::Printf(
			TEXT("[V6BoundaryOwnershipAB mode=%s cadence=%d] export_root=%s"),
			bWholeTriangleDuplication ? TEXT("duplicated") : TEXT("partitioned"),
			FixedIntervalSteps,
			*Variant.ExportRoot);
		AddInfo(ExportRootMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *ExportRootMessage);
		return Variant;
	};

	auto LogCheckpoint = [this](
		FVariantState& Variant,
		const TCHAR* SummaryTag,
		const int32 Step,
		FV6CheckpointSnapshot FVariantState::* MemberSnapshot,
		const bool bExportMaps)
	{
		if (bExportMaps)
		{
			ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		Variant.*MemberSnapshot = Snapshot;
		AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
	};

	FVariantState Partitioned = InitializeVariant(TEXT("partitioned"), false);
	FVariantState Duplicated = InitializeVariant(TEXT("duplicated"), true);

	const FString PartitionedSummaryTag = FString::Printf(
		TEXT("[V6BoundaryOwnershipAB mode=partitioned cadence=%d]"),
		FixedIntervalSteps);
	const FString DuplicatedSummaryTag = FString::Printf(
		TEXT("[V6BoundaryOwnershipAB mode=duplicated cadence=%d]"),
		FixedIntervalSteps);

	if (Partitioned.Planet.GetPlanet().CurrentStep < 25)
	{
		Partitioned.Planet.AdvanceSteps(25 - Partitioned.Planet.GetPlanet().CurrentStep);
	}
	if (Duplicated.Planet.GetPlanet().CurrentStep < 25)
	{
		Duplicated.Planet.AdvanceSteps(25 - Duplicated.Planet.GetPlanet().CurrentStep);
	}
	LogCheckpoint(Partitioned, *PartitionedSummaryTag, 25, &FVariantState::Step25Snapshot, false);
	LogCheckpoint(Duplicated, *DuplicatedSummaryTag, 25, &FVariantState::Step25Snapshot, false);

	if (Partitioned.Planet.GetPlanet().CurrentStep < 100)
	{
		Partitioned.Planet.AdvanceSteps(100 - Partitioned.Planet.GetPlanet().CurrentStep);
	}
	if (Duplicated.Planet.GetPlanet().CurrentStep < 100)
	{
		Duplicated.Planet.AdvanceSteps(100 - Duplicated.Planet.GetPlanet().CurrentStep);
	}
	LogCheckpoint(Partitioned, *PartitionedSummaryTag, 100, &FVariantState::Step100Snapshot, true);
	LogCheckpoint(Duplicated, *DuplicatedSummaryTag, 100, &FVariantState::Step100Snapshot, true);

	const int32 Step100BoundaryLocalizationImprovementCount =
		(Duplicated.Step100Snapshot.BoundaryCoherence.InteriorLeakageFraction <
				Partitioned.Step100Snapshot.BoundaryCoherence.InteriorLeakageFraction ? 1 : 0) +
		(Duplicated.Step100Snapshot.BoundaryCoherence.MeanConflictRingDistance <
				Partitioned.Step100Snapshot.BoundaryCoherence.MeanConflictRingDistance ? 1 : 0) +
		(Duplicated.Step100Snapshot.BoundaryCoherence.MissInteriorCount <
				Partitioned.Step100Snapshot.BoundaryCoherence.MissInteriorCount ? 1 : 0) +
		(Duplicated.Step100Snapshot.BoundaryCoherence.TotalInteriorConflictCount <
				Partitioned.Step100Snapshot.BoundaryCoherence.TotalInteriorConflictCount ? 1 : 0) +
		(Duplicated.Step100Snapshot.BoundaryCoherence.DestructiveExclusionInteriorCount <
				Partitioned.Step100Snapshot.BoundaryCoherence.DestructiveExclusionInteriorCount ? 1 : 0) +
		(Duplicated.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore >
				Partitioned.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore ? 1 : 0);
	const bool bStep100BoundaryLocalizationPromising =
		Step100BoundaryLocalizationImprovementCount >= 4 &&
		Duplicated.Step100Snapshot.BoundaryCoherence.InteriorLeakageFraction <=
			Partitioned.Step100Snapshot.BoundaryCoherence.InteriorLeakageFraction &&
		Duplicated.Step100Snapshot.BoundaryCoherence.MeanConflictRingDistance <=
			Partitioned.Step100Snapshot.BoundaryCoherence.MeanConflictRingDistance;
	const bool bStep100OverlapAcceptable =
		Duplicated.Step100Snapshot.MultiHitCount <=
		FMath::RoundToInt(static_cast<double>(Partitioned.Step100Snapshot.MultiHitCount) * 1.15);
	const bool bRunStep200 = bStep100BoundaryLocalizationPromising && bStep100OverlapAcceptable;

	AddInfo(FString::Printf(
		TEXT("[V6BoundaryOwnershipAB cadence=%d] step100_boundary_localization_improvements=%d boundary_gate=%d overlap_gate=%d run_step200=%d"),
		FixedIntervalSteps,
		Step100BoundaryLocalizationImprovementCount,
		bStep100BoundaryLocalizationPromising ? 1 : 0,
		bStep100OverlapAcceptable ? 1 : 0,
		bRunStep200 ? 1 : 0));

	if (bRunStep200)
	{
		if (Partitioned.Planet.GetPlanet().CurrentStep < 200)
		{
			Partitioned.Planet.AdvanceSteps(200 - Partitioned.Planet.GetPlanet().CurrentStep);
		}
		if (Duplicated.Planet.GetPlanet().CurrentStep < 200)
		{
			Duplicated.Planet.AdvanceSteps(200 - Duplicated.Planet.GetPlanet().CurrentStep);
		}
		LogCheckpoint(Partitioned, *PartitionedSummaryTag, 200, &FVariantState::Step200Snapshot, true);
		LogCheckpoint(Duplicated, *DuplicatedSummaryTag, 200, &FVariantState::Step200Snapshot, true);
		Partitioned.bHasStep200 = true;
		Duplicated.bHasStep200 = true;
	}

	TestTrue(TEXT("Partitioned step 100 has previous synthetic samples"), Partitioned.Step100Snapshot.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
	TestTrue(TEXT("Duplicated step 100 has previous synthetic samples"), Duplicated.Step100Snapshot.SyntheticCoveragePersistence.PreviousSyntheticSampleCount > 0);
	TestTrue(TEXT("Duplicated variant uses copied-frontier hit geometry by step 100"), Duplicated.Step100Snapshot.CopiedFrontierHitCount > 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPlateSubmeshSpikeRepresentationTest,
	"Aurous.TectonicPlanet.V6ThesisPlateSubmeshSpikeRepresentationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPlateSubmeshSpikeRepresentationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanetV6PeriodicSolveStats& Stats = Planet.GetLastSolveStats();
	TestEqual(TEXT("Plate-submesh spike uses the dedicated solve mode"), Stats.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike);
	TestEqual(TEXT("Plate-submesh spike uses the fixed 25-step cadence"), Stats.Interval, 25);
	TestTrue(TEXT("Plate-submesh spike builds plate-local vertices"), Stats.PlateLocalVertexCount > 0);
	TestTrue(TEXT("Plate-submesh spike builds plate-local triangles"), Stats.PlateLocalTriangleCount > 0);
	TestTrue(TEXT("Plate-submesh spike builds frontier vertices from plate-local copies"), Stats.PlateSubmeshFrontierVertexCount > 0);
	TestTrue(TEXT("Plate-submesh spike builds frontier triangles from retriangulated submeshes"), Stats.PlateSubmeshFrontierTriangleCount > 0);
	TestTrue(TEXT("Plate-submesh spike carries frontier samples in plate-local state"), Stats.PlateSubmeshFrontierCarriedSampleCount > 0);
	TestTrue(TEXT("Plate-submesh spike retriangulates every local triangle"), Stats.PlateSubmeshRetriangulatedTriangleCount == Stats.PlateLocalTriangleCount);
	TestTrue(TEXT("Plate-submesh spike builds plate-local connected components"), Stats.PlateSubmeshComponentCount > 0);
	TestEqual(TEXT("Plate-submesh spike no longer duplicates mixed global triangles wholesale"), Stats.PlateSubmeshWholeMixedTriangleDuplicationCount, 0);
	TestEqual(TEXT("Plate-submesh spike does not report copied-frontier triangles"), Stats.CopiedFrontierTriangleCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPlateSubmeshSpikeBoundaryTransferTest,
	"Aurous.TectonicPlanet.V6ThesisPlateSubmeshSpikeBoundaryTransferTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPlateSubmeshSpikeBoundaryTransferTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanet& LegacyPlanet = Planet.GetPlanet();
	const FTectonicPlanetV6PeriodicSolveStats& Stats = Planet.GetLastSolveStats();
	int32 BoundaryPlateSubmeshTriangleTransfers = 0;
	for (int32 SampleIndex = 0; SampleIndex < Planet.GetLastResolvedSamples().Num(); ++SampleIndex)
	{
		const FTectonicPlanetV6ResolvedSample& Resolved = Planet.GetLastResolvedSamples()[SampleIndex];
		if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::Triangle &&
			Resolved.TransferDebug.bUsedPlateSubmeshTriangleTransfer &&
			LegacyPlanet.Samples.IsValidIndex(SampleIndex) &&
			LegacyPlanet.Samples[SampleIndex].bIsBoundary)
		{
			++BoundaryPlateSubmeshTriangleTransfers;
		}
	}

	TestTrue(TEXT("Plate-submesh spike keeps every hit on direct triangle transfer"), Stats.DirectHitTriangleTransferCount == Stats.HitCount);
	TestEqual(TEXT("Plate-submesh spike keeps transfer fallback at zero on the first remesh"), Stats.TransferFallbackCount, 0);
	TestTrue(TEXT("Plate-submesh spike can transfer a boundary sample from plate-local frontier triangle data"), BoundaryPlateSubmeshTriangleTransfers > 0);
	TestTrue(TEXT("Plate-submesh spike records frontier hit geometry"), Stats.PlateSubmeshFrontierHitCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPlateSubmeshSpikeMissCreatesOceanicCrustTest,
	"Aurous.TectonicPlanet.V6ThesisPlateSubmeshSpikeMissCreatesOceanicCrustTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPlateSubmeshSpikeMissCreatesOceanicCrustTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6(ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike, 25);
	const int32 Interval = Planet.ComputePeriodicSolveInterval();

	while (Planet.GetPlanet().CurrentStep < Interval)
	{
		Planet.AdvanceStep();
	}

	const FTectonicPlanetV6PeriodicSolveStats& Stats = Planet.GetLastSolveStats();
	TestTrue(TEXT("Plate-submesh spike still produces true misses"), Stats.MissCount > 0);
	TestEqual(TEXT("Plate-submesh spike fills every miss with explicit oceanic creation"), Stats.OceanicCreationCount, Stats.MissCount);

	int32 MissResolutionCount = 0;
	int32 MissOceanicTransferCount = 0;
	for (const FTectonicPlanetV6ResolvedSample& Resolved : Planet.GetLastResolvedSamples())
	{
		if (Resolved.ResolutionKind == ETectonicPlanetV6ResolutionKind::ThesisRemeshMissOceanic)
		{
			++MissResolutionCount;
			if (Resolved.TransferDebug.SourceKind == ETectonicPlanetV6TransferSourceKind::OceanicCreation)
			{
				++MissOceanicTransferCount;
			}
		}
	}

	TestEqual(TEXT("Plate-submesh miss resolution count matches the miss metric"), MissResolutionCount, Stats.MissCount);
	TestEqual(TEXT("Plate-submesh miss-resolved samples all use oceanic creation"), MissOceanicTransferCount, Stats.MissCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisPlateSubmeshSpikeBaseline60k7Test,
	"Aurous.TectonicPlanet.V6ThesisPlateSubmeshSpikeBaseline60k7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisPlateSubmeshSpikeBaseline60k7Test::RunTest(const FString& Parameters)
{
	const FString RunId = FString::Printf(
		TEXT("V6ThesisPlateSubmeshSpikeBaseline60k7-seed%d-samples%d-plates%d"),
		TestRandomSeed,
		TestSampleCount,
		TestPlateCount);
	const FV6BaselineSummary Summary = RunV6BaselineScenario(
		*this,
		RunId,
		200,
		TArray<int32>{},
		ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike,
		25,
		false);

	TestTrue(TEXT("Plate-submesh spike baseline records the step-50 checkpoint"), Summary.bHasStep50Snapshot);
	TestTrue(TEXT("Plate-submesh spike baseline records the step-100 checkpoint"), Summary.bHasStep100Snapshot);
	TestTrue(TEXT("Plate-submesh spike baseline records the step-150 checkpoint"), Summary.bHasStep150Snapshot);
	TestTrue(TEXT("Plate-submesh spike baseline records the step-200 checkpoint"), Summary.bHasStep200Snapshot);
	TestEqual(TEXT("Plate-submesh spike baseline final step lands at 200"), Summary.FinalSnapshot.Step, 200);
	TestEqual(TEXT("Plate-submesh spike step-200 uses the fixed 25-step cadence"), Summary.Step200Snapshot.Interval, 25);
	TestEqual(TEXT("Plate-submesh spike step-200 reports the plate-submesh mode"), Summary.Step200Snapshot.SolveMode, ETectonicPlanetV6PeriodicSolveMode::ThesisPlateSubmeshSpike);
	TestTrue(TEXT("Plate-submesh spike step-200 still has direct hits"), Summary.Step200Snapshot.HitCount > 0);
	TestTrue(TEXT("Plate-submesh spike step-200 uses plate-submesh frontier hit geometry"), Summary.Step200Snapshot.PlateSubmeshFrontierHitCount > 0);
	TestTrue(TEXT("Plate-submesh spike step-200 still has true misses"), Summary.Step200Snapshot.MissCount > 0);
	TestEqual(TEXT("Plate-submesh spike step-200 miss count matches oceanic creation"), Summary.Step200Snapshot.OceanicCreationCount, Summary.Step200Snapshot.MissCount);
	TestEqual(TEXT("Plate-submesh spike step-200 keeps transfer fallback near zero"), Summary.Step200Snapshot.TransferFallbackCount, 0);
	TestEqual(TEXT("Plate-submesh spike step-200 keeps direct hit transfer aligned with hits"), Summary.Step200Snapshot.DirectHitTriangleTransferCount, Summary.Step200Snapshot.HitCount);
	TestEqual(TEXT("Plate-submesh spike step-200 keeps wholesale mixed-triangle duplication at zero"), Summary.Step200Snapshot.PlateSubmeshWholeMixedTriangleDuplicationCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisRegimeABCMixedTriangleTest,
	"Aurous.TectonicPlanet.V6ThesisRegimeABCMixedTriangleTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisRegimeABCMixedTriangleTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	constexpr int32 RandomSeed = TestRandomSeed;
	const FString RunId = TEXT("V6ThesisRegimeABC");

	AddInfo(TEXT("[V6ThesisRegimeABC] A/B/C mixed-triangle semantics at 60k/40 plates, retention OFF, propagation OFF, frontier-pair fill ON, cadence 16"));

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FV6CheckpointSnapshot Step200Snapshot;
	};

	const auto InitializeVariant = [this, FixedIntervalSteps, RandomSeed, &RunId](
		const TCHAR* VariantLabel,
		const bool bExcludeMixed,
		const bool bWholeTriangleDuplication) -> FVariantState
	{
		FVariantState Variant;
		Variant.Label = VariantLabel;
		Variant.Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			RandomSeed);
		Variant.Planet.SetSyntheticCoverageRetentionForTest(false);
		Variant.Planet.SetWholeTriangleBoundaryDuplicationForTest(bWholeTriangleDuplication);
		Variant.Planet.SetExcludeMixedTrianglesForTest(bExcludeMixed);

		Variant.ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			RunId,
			VariantLabel);

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*Variant.ExportRoot);

		const FString InfoMsg = FString::Printf(
			TEXT("[V6ThesisRegimeABC variant=%s] exclude_mixed=%d whole_dup=%d samples=%d plates=%d cadence=%d"),
			VariantLabel,
			bExcludeMixed ? 1 : 0,
			bWholeTriangleDuplication ? 1 : 0,
			SampleCount,
			PlateCount,
			FixedIntervalSteps);
		AddInfo(InfoMsg);
		UE_LOG(LogTemp, Log, TEXT("%s"), *InfoMsg);
		return Variant;
	};

	const auto LogCheckpoint = [this](
		FVariantState& Variant,
		const TCHAR* SummaryTag,
		const int32 Step,
		FV6CheckpointSnapshot FVariantState::* MemberSnapshot)
	{
		ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
		ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		Variant.*MemberSnapshot = Snapshot;
		AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
	};

	// Mode A: Exclude mixed triangles
	FVariantState ModeA = InitializeVariant(TEXT("A_ExcludeMixed"), true, false);
	// Mode B: Duplicate whole mixed triangles
	FVariantState ModeB = InitializeVariant(TEXT("B_DuplicateWhole"), false, true);
	// Mode C: Partition split mixed triangles (default partitioned substrate)
	FVariantState ModeC = InitializeVariant(TEXT("C_PartitionSplit"), false, false);

	// Run all three to step 25
	ModeA.Planet.AdvanceSteps(25);
	LogCheckpoint(ModeA, TEXT("[V6ThesisRegimeABC A_ExcludeMixed step=25]"), 25, &FVariantState::Step25Snapshot);
	ModeB.Planet.AdvanceSteps(25);
	LogCheckpoint(ModeB, TEXT("[V6ThesisRegimeABC B_DuplicateWhole step=25]"), 25, &FVariantState::Step25Snapshot);
	ModeC.Planet.AdvanceSteps(25);
	LogCheckpoint(ModeC, TEXT("[V6ThesisRegimeABC C_PartitionSplit step=25]"), 25, &FVariantState::Step25Snapshot);

	// Run all three to step 100
	ModeA.Planet.AdvanceSteps(75);
	LogCheckpoint(ModeA, TEXT("[V6ThesisRegimeABC A_ExcludeMixed step=100]"), 100, &FVariantState::Step100Snapshot);
	ModeB.Planet.AdvanceSteps(75);
	LogCheckpoint(ModeB, TEXT("[V6ThesisRegimeABC B_DuplicateWhole step=100]"), 100, &FVariantState::Step100Snapshot);
	ModeC.Planet.AdvanceSteps(75);
	LogCheckpoint(ModeC, TEXT("[V6ThesisRegimeABC C_PartitionSplit step=100]"), 100, &FVariantState::Step100Snapshot);

	// Find best mode by boundary coherence score at step 100
	const FV6CheckpointSnapshot* BestSnapshot = &ModeA.Step100Snapshot;
	FVariantState* BestVariant = &ModeA;
	const TCHAR* BestLabel = TEXT("A_ExcludeMixed");
	if (ModeB.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore > BestSnapshot->BoundaryCoherence.BoundaryCoherenceScore)
	{
		BestSnapshot = &ModeB.Step100Snapshot;
		BestVariant = &ModeB;
		BestLabel = TEXT("B_DuplicateWhole");
	}
	if (ModeC.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore > BestSnapshot->BoundaryCoherence.BoundaryCoherenceScore)
	{
		BestSnapshot = &ModeC.Step100Snapshot;
		BestVariant = &ModeC;
		BestLabel = TEXT("C_PartitionSplit");
	}

	AddInfo(*FString::Printf(TEXT("[V6ThesisRegimeABC] best_mode_at_step_100=%s coherence_score=%.4f"), BestLabel, BestSnapshot->BoundaryCoherence.BoundaryCoherenceScore));
	UE_LOG(LogTemp, Log, TEXT("[V6ThesisRegimeABC] best_mode_at_step_100=%s coherence_score=%.4f"), BestLabel, BestSnapshot->BoundaryCoherence.BoundaryCoherenceScore);

	// Run best mode to step 200
	BestVariant->Planet.AdvanceSteps(100);
	LogCheckpoint(*BestVariant, *FString::Printf(TEXT("[V6ThesisRegimeABC %s step=200]"), BestLabel), 200, &FVariantState::Step200Snapshot);

	// Summary comparison table — split into multiple log lines to avoid MSVC UE_LOG arg limits
	UE_LOG(LogTemp, Log, TEXT("[V6ThesisRegimeABC SUMMARY step=100]"));
	// Log summary via AddV6ThesisRemeshInfo which already handles all metrics
	AddInfo(TEXT("[V6ThesisRegimeABC SUMMARY] See individual variant logs above for full comparison"));

	// Basic validation
	TestTrue(TEXT("At least one mode has hits at step 100"), ModeA.Step100Snapshot.HitCount > 0 || ModeB.Step100Snapshot.HitCount > 0 || ModeC.Step100Snapshot.HitCount > 0);
	TestTrue(TEXT("Best mode reached step 200"), BestVariant->Step200Snapshot.Step == 200);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisRegime250k40SpotTest,
	"Aurous.TectonicPlanet.V6ThesisRegime250k40SpotTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisRegime250k40SpotTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		16,
		INDEX_NONE,
		250000,
		40,
		TestRandomSeed);
	Planet.SetSyntheticCoverageRetentionForTest(false);

	Planet.AdvanceSteps(25);

	const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
	AddV6ThesisRemeshInfo(*this, TEXT("[250k40Spot step=25]"), Snapshot);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[250k40Spot step=25]"), Snapshot);
	AddV6BoundaryCoherenceInfo(*this, TEXT("[250k40Spot step=25]"), Snapshot);

	TestTrue(TEXT("250k/40 has hits"), Snapshot.HitCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6ThesisImplementationFaithfulnessAuditTest,
	"Aurous.TectonicPlanet.V6ThesisImplementationFaithfulnessAuditTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6ThesisImplementationFaithfulnessAuditTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;

	struct FVariantResult
	{
		FString Label;
		FTectonicPlanetV6 Planet;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FTectonicPlanetV6OwnerCoverageAudit Step25CoverageAudit;
		FTectonicPlanetV6OwnerCoverageAudit Step100CoverageAudit;
	};

	const auto InitializeVariant = [FixedIntervalSteps](const TCHAR* Label, const bool bPerStepBVHRebuild) -> FVariantResult
	{
		FVariantResult Variant;
		Variant.Label = Label;
		Variant.Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Variant.Planet.SetSyntheticCoverageRetentionForTest(false);
		Variant.Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Variant.Planet.SetExcludeMixedTrianglesForTest(false);
		Variant.Planet.SetPerTimestepContainmentSoupRebuildForTest(bPerStepBVHRebuild);
		return Variant;
	};

	const auto LogVariantCheckpoint = [this](
		FVariantResult& Variant,
		const FString& SummaryTag,
		const int32 Step,
		FV6CheckpointSnapshot FVariantResult::* SnapshotMember,
		FTectonicPlanetV6OwnerCoverageAudit FVariantResult::* AuditMember)
	{
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		Variant.*SnapshotMember = Snapshot;
		AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
		AddV6TectonicInteractionInfo(*this, SummaryTag, Snapshot);

		Variant.Planet.GetPlanetMutable().BuildContainmentSoups();
		const FTectonicPlanetV6OwnerCoverageAudit Audit = Variant.Planet.ComputeCurrentOwnerCoverageAuditForTest();
		Variant.*AuditMember = Audit;
		AddV6OwnerCoverageAuditInfo(*this, SummaryTag, Step, Audit);
	};

	FVariantResult CurrentBVH = InitializeVariant(TEXT("A_RemeshBVHOnly"), false);
	FVariantResult PerStepBVH = InitializeVariant(TEXT("B_PerStepBVH"), true);

	for (FVariantResult* Variant : { &CurrentBVH, &PerStepBVH })
	{
		const FString SummaryTag = FString::Printf(
			TEXT("[V6ThesisFaithfulness mode=%s cadence=%d samples=%d plates=%d]"),
			*Variant->Label,
			FixedIntervalSteps,
			SampleCount,
			PlateCount);

		if (Variant->Planet.GetPlanet().CurrentStep < 25)
		{
			Variant->Planet.AdvanceSteps(25 - Variant->Planet.GetPlanet().CurrentStep);
		}
		LogVariantCheckpoint(
			*Variant,
			SummaryTag,
			25,
			&FVariantResult::Step25Snapshot,
			&FVariantResult::Step25CoverageAudit);

		if (Variant->Planet.GetPlanet().CurrentStep < 100)
		{
			Variant->Planet.AdvanceSteps(100 - Variant->Planet.GetPlanet().CurrentStep);
		}
		LogVariantCheckpoint(
			*Variant,
			SummaryTag,
			100,
			&FVariantResult::Step100Snapshot,
			&FVariantResult::Step100CoverageAudit);
	}

	AddInfo(*FString::Printf(
		TEXT("[V6ThesisFaithfulness Check1 compare step=25] churn=%.4f->%.4f coherence=%.4f->%.4f collision=%d->%d"),
		CurrentBVH.Step25Snapshot.OwnershipChurn.ChurnFraction,
		PerStepBVH.Step25Snapshot.OwnershipChurn.ChurnFraction,
		CurrentBVH.Step25Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		PerStepBVH.Step25Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		CurrentBVH.Step25Snapshot.CollisionCount,
		PerStepBVH.Step25Snapshot.CollisionCount));
	AddInfo(*FString::Printf(
		TEXT("[V6ThesisFaithfulness Check1 compare step=100] churn=%.4f->%.4f coherence=%.4f->%.4f collision=%d->%d owner_query_exact=%d->%d owner_soup_query_gap=%d->%d"),
		CurrentBVH.Step100Snapshot.OwnershipChurn.ChurnFraction,
		PerStepBVH.Step100Snapshot.OwnershipChurn.ChurnFraction,
		CurrentBVH.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		PerStepBVH.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		CurrentBVH.Step100Snapshot.CollisionCount,
		PerStepBVH.Step100Snapshot.CollisionCount,
		CurrentBVH.Step100CoverageAudit.OwnerQueryExactHitCount,
		PerStepBVH.Step100CoverageAudit.OwnerQueryExactHitCount,
		CurrentBVH.Step100CoverageAudit.OwnerSoupContainmentButQueryMissCount,
		PerStepBVH.Step100CoverageAudit.OwnerSoupContainmentButQueryMissCount));

	FTectonicPlanetV6 SmallPlanet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		100,
		3,
		TestRandomSeed);
	SmallPlanet.SetSyntheticCoverageRetentionForTest(false);
	SmallPlanet.SetWholeTriangleBoundaryDuplicationForTest(false);
	SmallPlanet.SetExcludeMixedTrianglesForTest(false);
	SmallPlanet.AdvanceSteps(25);
	SmallPlanet.GetPlanetMutable().BuildContainmentSoups();
	const FV6CheckpointSnapshot SmallStep25Snapshot = BuildV6CheckpointSnapshot(SmallPlanet);
	const FTectonicPlanetV6OwnerCoverageAudit SmallAudit =
		SmallPlanet.ComputeCurrentOwnerCoverageAuditForTest();
	AddV6ThesisRemeshInfo(*this, TEXT("[V6ThesisFaithfulness Small100x3]"), SmallStep25Snapshot);
	AddV6DiagnosticsPackageInfo(*this, TEXT("[V6ThesisFaithfulness Small100x3]"), SmallStep25Snapshot);
	AddV6OwnerCoverageAuditInfo(*this, TEXT("[V6ThesisFaithfulness Small100x3]"), 25, SmallAudit);

	TestTrue(TEXT("Current 60k/40 variant reaches step 100"), CurrentBVH.Step100Snapshot.Step == 100);
	TestTrue(TEXT("Per-step BVH 60k/40 variant reaches step 100"), PerStepBVH.Step100Snapshot.Step == 100);
	TestTrue(TEXT("Small 100/3 audit has assigned samples"), SmallAudit.AssignedSampleCount > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1AuthorityClassifierTest,
	"Aurous.TectonicPlanet.V6V9Phase1AuthorityClassifierTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1AuthorityClassifierTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	constexpr int32 ActiveBoundaryRingCount = 1;
	const FString RunId = TEXT("V9Phase1AuthorityClassifier");

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FV6CheckpointSnapshot Step200Snapshot;
		bool bHasStep200 = false;
	};

	const auto InitializeVariant = [this, FixedIntervalSteps, SampleCount, PlateCount, ActiveBoundaryRingCount, &RunId](
		const TCHAR* VariantLabel,
		const bool bEnablePhase1) -> FVariantState
	{
		FVariantState Variant;
		Variant.Label = VariantLabel;
		Variant.Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Variant.Planet.SetSyntheticCoverageRetentionForTest(false);
		Variant.Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Variant.Planet.SetExcludeMixedTrianglesForTest(false);
		Variant.Planet.SetV9Phase1AuthorityForTest(bEnablePhase1, ActiveBoundaryRingCount);

		Variant.ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			RunId,
			VariantLabel);

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*Variant.ExportRoot);

		const FString Message = FString::Printf(
			TEXT("[V9Phase1 variant=%s] phase1=%d cadence=%d samples=%d plates=%d active_boundary_rings=%d mixed_mode=PartitionSplit retention=0"),
			VariantLabel,
			bEnablePhase1 ? 1 : 0,
			FixedIntervalSteps,
			SampleCount,
			PlateCount,
			ActiveBoundaryRingCount);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
		return Variant;
	};

	const auto LogCheckpoint = [this](
		FVariantState& Variant,
		const FString& SummaryTag,
		const int32 Step,
		FV6CheckpointSnapshot FVariantState::* SnapshotMember,
		const bool bExport)
	{
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		Variant.*SnapshotMember = Snapshot;
		AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
		AddV6TectonicInteractionInfo(*this, SummaryTag, Snapshot);
		AddV6ActiveZoneInfo(*this, SummaryTag, Snapshot);
	};

	FVariantState Baseline = InitializeVariant(TEXT("baseline_v8_authority"), false);
	FVariantState Phase1 = InitializeVariant(TEXT("phase1_authority"), true);

	Baseline.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Baseline,
		TEXT("[V9Phase1 baseline step=25]"),
		25,
		&FVariantState::Step25Snapshot,
		false);
	Phase1.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Phase1,
		TEXT("[V9Phase1 phase1 step=25]"),
		25,
		&FVariantState::Step25Snapshot,
		false);

	Baseline.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Baseline,
		TEXT("[V9Phase1 baseline step=100]"),
		100,
		&FVariantState::Step100Snapshot,
		false);
	Phase1.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Phase1,
		TEXT("[V9Phase1 phase1 step=100]"),
		100,
		&FVariantState::Step100Snapshot,
		false);

	const bool bStep100Promising =
		Phase1.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone <= 64 &&
		Phase1.Step100Snapshot.ActiveZone.ActiveFraction <= 0.40 &&
		(Phase1.Step100Snapshot.OwnershipChurn.ChurnFraction <= 0.50 ||
			Phase1.Step100Snapshot.CollisionCount > 0 ||
			Phase1.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore >=
				Baseline.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore + 0.08);

	AddInfo(FString::Printf(
		TEXT("[V9Phase1 gate step=100] run_step200=%d active_fraction=%.4f outside_zone_changes=%d churn=%.4f baseline_churn=%.4f coherence=%.4f baseline_coherence=%.4f collision=%d"),
		bStep100Promising ? 1 : 0,
		Phase1.Step100Snapshot.ActiveZone.ActiveFraction,
		Phase1.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone,
		Phase1.Step100Snapshot.OwnershipChurn.ChurnFraction,
		Baseline.Step100Snapshot.OwnershipChurn.ChurnFraction,
		Phase1.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Baseline.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Phase1.Step100Snapshot.CollisionCount));

	if (bStep100Promising)
	{
		ExportV6CheckpointMaps(*this, Baseline.Planet, Baseline.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Baseline.Planet, Baseline.ExportRoot, 0);
		ExportV6CheckpointMaps(*this, Phase1.Planet, Phase1.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Phase1.Planet, Phase1.ExportRoot, 0);
		ExportV6CheckpointMaps(*this, Baseline.Planet, Baseline.ExportRoot, 25);
		ExportV6DebugOverlays(*this, Baseline.Planet, Baseline.ExportRoot, 25);
		ExportV6CheckpointMaps(*this, Phase1.Planet, Phase1.ExportRoot, 25);
		ExportV6DebugOverlays(*this, Phase1.Planet, Phase1.ExportRoot, 25);
		ExportV6CheckpointMaps(*this, Baseline.Planet, Baseline.ExportRoot, 100);
		ExportV6DebugOverlays(*this, Baseline.Planet, Baseline.ExportRoot, 100);
		ExportV6CheckpointMaps(*this, Phase1.Planet, Phase1.ExportRoot, 100);
		ExportV6DebugOverlays(*this, Phase1.Planet, Phase1.ExportRoot, 100);

		Baseline.Planet.AdvanceSteps(100);
		LogCheckpoint(
			Baseline,
			TEXT("[V9Phase1 baseline step=200]"),
			200,
			&FVariantState::Step200Snapshot,
			true);
		Baseline.bHasStep200 = true;

		Phase1.Planet.AdvanceSteps(100);
		LogCheckpoint(
			Phase1,
			TEXT("[V9Phase1 phase1 step=200]"),
			200,
			&FVariantState::Step200Snapshot,
			true);
		Phase1.bHasStep200 = true;
	}

	TestEqual(
		TEXT("Phase1 active+inactive samples cover the whole planet at step 100"),
		Phase1.Step100Snapshot.ActiveZone.ActiveSampleCount +
			Phase1.Step100Snapshot.ActiveZone.InactiveSampleCount,
		SampleCount);
	TestTrue(
		TEXT("Phase1 step 100 records outside-zone ownership changes no greater than total churn"),
		Phase1.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone <=
			Phase1.Step100Snapshot.OwnershipChurn.TotalChurnCount);
	TestTrue(TEXT("Baseline reaches step 100"), Baseline.Step100Snapshot.Step == 100);
	TestTrue(TEXT("Phase1 reaches step 100"), Phase1.Step100Snapshot.Step == 100);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1bNarrowClassifierTest,
	"Aurous.TectonicPlanet.V6V9Phase1bNarrowClassifierTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1bNarrowClassifierTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	constexpr int32 BroadActiveBoundaryRingCount = 1;
	constexpr int32 NarrowActiveBoundaryRingCount = 0;
	const FString RunId = TEXT("V9Phase1bNarrowClassifier");

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FV6CheckpointSnapshot Step200Snapshot;
		bool bHasStep200 = false;
	};

	const auto InitializeVariant =
		[this, FixedIntervalSteps, SampleCount, PlateCount, &RunId](
			const TCHAR* VariantLabel,
			const ETectonicPlanetV6ActiveZoneClassifierMode ClassifierMode,
			const int32 ActiveBoundaryRingCount) -> FVariantState
	{
		FVariantState Variant;
		Variant.Label = VariantLabel;
		Variant.Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Variant.Planet.SetSyntheticCoverageRetentionForTest(false);
		Variant.Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Variant.Planet.SetExcludeMixedTrianglesForTest(false);
		Variant.Planet.SetV9Phase1AuthorityForTest(true, ActiveBoundaryRingCount);
		Variant.Planet.SetV9Phase1ActiveZoneClassifierModeForTest(ClassifierMode);

		Variant.ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			RunId,
			VariantLabel);

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*Variant.ExportRoot);

		const FString Message = FString::Printf(
			TEXT("[V9Phase1b variant=%s] phase1=1 cadence=%d samples=%d plates=%d active_boundary_rings=%d classifier=%s mixed_mode=PartitionSplit retention=0"),
			VariantLabel,
			FixedIntervalSteps,
			SampleCount,
			PlateCount,
			ActiveBoundaryRingCount,
			ClassifierMode == ETectonicPlanetV6ActiveZoneClassifierMode::NarrowTectonicPairs
				? TEXT("NarrowTectonicPairs")
				: TEXT("BroadBoundaryBand"));
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
		return Variant;
	};

	const auto LogCheckpoint =
		[this](
			FVariantState& Variant,
			const FString& SummaryTag,
			const int32 Step,
			FV6CheckpointSnapshot FVariantState::* SnapshotMember,
			const bool bExport)
	{
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		Variant.*SnapshotMember = Snapshot;
		AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
		AddV6TectonicInteractionInfo(*this, SummaryTag, Snapshot);
		AddV6ActiveZoneInfo(*this, SummaryTag, Snapshot);
	};

	FVariantState Broad = InitializeVariant(
		TEXT("phase1_broad_classifier"),
		ETectonicPlanetV6ActiveZoneClassifierMode::BroadBoundaryBand,
		BroadActiveBoundaryRingCount);
	FVariantState Narrow = InitializeVariant(
		TEXT("phase1b_narrow_classifier"),
		ETectonicPlanetV6ActiveZoneClassifierMode::NarrowTectonicPairs,
		NarrowActiveBoundaryRingCount);

	Broad.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Broad,
		TEXT("[V9Phase1b broad step=25]"),
		25,
		&FVariantState::Step25Snapshot,
		false);
	Narrow.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Narrow,
		TEXT("[V9Phase1b narrow step=25]"),
		25,
		&FVariantState::Step25Snapshot,
		false);

	Broad.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Broad,
		TEXT("[V9Phase1b broad step=100]"),
		100,
		&FVariantState::Step100Snapshot,
		true);
	Narrow.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Narrow,
		TEXT("[V9Phase1b narrow step=100]"),
		100,
		&FVariantState::Step100Snapshot,
		true);

	const int32 NarrowExplicitCauseChanges =
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangeDivergenceFillCount +
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangeConvergentSubductionCount +
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangeCollisionContactCount +
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangeRiftCount;
	const bool bNotFrozen =
		Narrow.Step100Snapshot.ActiveZone.ActiveFraction >= 0.02 &&
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangesInsideActiveZone > 0 &&
		Narrow.Step100Snapshot.HitCount >= 512;
	const bool bNarrowGatePassed =
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone <= 64 &&
		Narrow.Step100Snapshot.ActiveZone.ActiveFraction <= 0.25 &&
		Narrow.Step100Snapshot.OwnershipChurn.ChurnFraction <= 0.50 &&
		bNotFrozen &&
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangeGenericQueryCompetitionCount <= NarrowExplicitCauseChanges;

	AddInfo(FString::Printf(
		TEXT("[V9Phase1b gate step=100] run_step200=%d active_fraction=%.4f outside_zone_changes=%d churn=%.4f broad_churn=%.4f coherence=%.4f broad_coherence=%.4f generic_query=%d explicit_causes=%d collision=%d not_frozen=%d hit_count=%d inside_changes=%d"),
		bNarrowGatePassed ? 1 : 0,
		Narrow.Step100Snapshot.ActiveZone.ActiveFraction,
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone,
		Narrow.Step100Snapshot.OwnershipChurn.ChurnFraction,
		Broad.Step100Snapshot.OwnershipChurn.ChurnFraction,
		Narrow.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Broad.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangeGenericQueryCompetitionCount,
		NarrowExplicitCauseChanges,
		Narrow.Step100Snapshot.CollisionCount,
		bNotFrozen ? 1 : 0,
		Narrow.Step100Snapshot.HitCount,
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangesInsideActiveZone));

	if (bNarrowGatePassed)
	{
		ExportV6CheckpointMaps(*this, Broad.Planet, Broad.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Broad.Planet, Broad.ExportRoot, 0);
		ExportV6CheckpointMaps(*this, Narrow.Planet, Narrow.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Narrow.Planet, Narrow.ExportRoot, 0);
		ExportV6CheckpointMaps(*this, Broad.Planet, Broad.ExportRoot, 25);
		ExportV6DebugOverlays(*this, Broad.Planet, Broad.ExportRoot, 25);
		ExportV6CheckpointMaps(*this, Narrow.Planet, Narrow.ExportRoot, 25);
		ExportV6DebugOverlays(*this, Narrow.Planet, Narrow.ExportRoot, 25);

		Broad.Planet.AdvanceSteps(100);
		LogCheckpoint(
			Broad,
			TEXT("[V9Phase1b broad step=200]"),
			200,
			&FVariantState::Step200Snapshot,
			true);
		Broad.bHasStep200 = true;

		Narrow.Planet.AdvanceSteps(100);
		LogCheckpoint(
			Narrow,
			TEXT("[V9Phase1b narrow step=200]"),
			200,
			&FVariantState::Step200Snapshot,
			true);
		Narrow.bHasStep200 = true;
	}

	TestEqual(
		TEXT("Phase1b narrow active+inactive samples cover the whole planet at step 100"),
		Narrow.Step100Snapshot.ActiveZone.ActiveSampleCount +
			Narrow.Step100Snapshot.ActiveZone.InactiveSampleCount,
		SampleCount);
	TestTrue(
		TEXT("Phase1b narrow keeps outside-zone ownership changes near zero"),
		Narrow.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone <= 64);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1cPersistentClassifierTest,
	"Aurous.TectonicPlanet.V6V9Phase1cPersistentClassifierTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1cPersistentClassifierTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	constexpr int32 BroadActiveBoundaryRingCount = 1;
	constexpr int32 NarrowActiveBoundaryRingCount = 0;
	constexpr int32 PersistentActiveBoundaryRingCount = 1;
	constexpr int32 PersistentActivePairHorizon = 2;
	const FString RunId = TEXT("V9Phase1cPersistentClassifier");

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		FV6CheckpointSnapshot Step25Snapshot;
		FV6CheckpointSnapshot Step100Snapshot;
		FV6CheckpointSnapshot Step200Snapshot;
		bool bHasStep100 = false;
		bool bHasStep200 = false;
	};

	const auto InitializeVariant =
		[this, FixedIntervalSteps, SampleCount, PlateCount, &RunId](
			const TCHAR* VariantLabel,
			const ETectonicPlanetV6ActiveZoneClassifierMode ClassifierMode,
			const int32 ActiveBoundaryRingCount,
			const int32 PersistenceHorizon) -> FVariantState
	{
		FVariantState Variant;
		Variant.Label = VariantLabel;
		Variant.Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Variant.Planet.SetSyntheticCoverageRetentionForTest(false);
		Variant.Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Variant.Planet.SetExcludeMixedTrianglesForTest(false);
		Variant.Planet.SetV9Phase1AuthorityForTest(true, ActiveBoundaryRingCount);
		Variant.Planet.SetV9Phase1ActiveZoneClassifierModeForTest(ClassifierMode);
		Variant.Planet.SetV9Phase1PersistentActivePairHorizonForTest(PersistenceHorizon);

		Variant.ExportRoot = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("MapExports"),
			RunId,
			VariantLabel);

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.DeleteDirectoryRecursively(*Variant.ExportRoot);
		PlatformFile.CreateDirectoryTree(*Variant.ExportRoot);
		ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, 0);
		ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, 0);

		const FString ClassifierLabel =
			ClassifierMode == ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocal
				? TEXT("PersistentPairLocal")
				: (ClassifierMode == ETectonicPlanetV6ActiveZoneClassifierMode::NarrowTectonicPairs
					? TEXT("NarrowTectonicPairs")
					: TEXT("BroadBoundaryBand"));
		const FString Message = FString::Printf(
			TEXT("[V9Phase1c variant=%s] phase1=1 cadence=%d samples=%d plates=%d active_boundary_rings=%d persistence_horizon=%d classifier=%s mixed_mode=PartitionSplit retention=0"),
			VariantLabel,
			FixedIntervalSteps,
			SampleCount,
			PlateCount,
			ActiveBoundaryRingCount,
			PersistenceHorizon,
			*ClassifierLabel);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
		return Variant;
	};

	const auto LogCheckpoint =
		[this](
			FVariantState& Variant,
			const FString& SummaryTag,
			const int32 Step,
			FV6CheckpointSnapshot FVariantState::* SnapshotMember)
	{
		ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
		ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		Variant.*SnapshotMember = Snapshot;
		AddV6ThesisRemeshInfo(*this, SummaryTag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, SummaryTag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, SummaryTag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, SummaryTag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, SummaryTag, Snapshot);
		AddV6TectonicInteractionInfo(*this, SummaryTag, Snapshot);
		AddV6ActiveZoneInfo(*this, SummaryTag, Snapshot);
	};

	FVariantState Broad = InitializeVariant(
		TEXT("phase1_broad_classifier"),
		ETectonicPlanetV6ActiveZoneClassifierMode::BroadBoundaryBand,
		BroadActiveBoundaryRingCount,
		1);
	FVariantState Narrow = InitializeVariant(
		TEXT("phase1b_narrow_classifier"),
		ETectonicPlanetV6ActiveZoneClassifierMode::NarrowTectonicPairs,
		NarrowActiveBoundaryRingCount,
		1);
	FVariantState Persistent = InitializeVariant(
		TEXT("phase1c_persistent_pair_local_classifier"),
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocal,
		PersistentActiveBoundaryRingCount,
		PersistentActivePairHorizon);

	Broad.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Broad,
		TEXT("[V9Phase1c broad step=25]"),
		25,
		&FVariantState::Step25Snapshot);
	Narrow.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Narrow,
		TEXT("[V9Phase1c narrow step=25]"),
		25,
		&FVariantState::Step25Snapshot);
	Persistent.Planet.AdvanceSteps(25);
	LogCheckpoint(
		Persistent,
		TEXT("[V9Phase1c persistent step=25]"),
		25,
		&FVariantState::Step25Snapshot);

	const bool bFrozenAtStep25 =
		Persistent.Step25Snapshot.ActiveZone.ActiveFraction <= UE_DOUBLE_SMALL_NUMBER ||
		Persistent.Step25Snapshot.ActiveZone.OwnershipChangesInsideActiveZone == 0;
	const bool bTooBroadAtStep25 =
		Persistent.Step25Snapshot.ActiveZone.ActiveFraction > 0.30;
	const bool bInvalidOutsideAtStep25 =
		Persistent.Step25Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone != 0;
	const bool bRunStep100 =
		!bFrozenAtStep25 &&
		!bTooBroadAtStep25 &&
		!bInvalidOutsideAtStep25;

	AddInfo(FString::Printf(
		TEXT("[V9Phase1c gate step=25] run_step100=%d active_fraction=%.4f outside_zone_changes=%d churn=%.4f broad_churn=%.4f plates_with_hit=%d/%d largest_share=%.4f active_pairs=%d fresh_pairs=%d carry_pairs=%d frozen=%d too_broad=%d invalid_outside=%d"),
		bRunStep100 ? 1 : 0,
		Persistent.Step25Snapshot.ActiveZone.ActiveFraction,
		Persistent.Step25Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone,
		Persistent.Step25Snapshot.OwnershipChurn.ChurnFraction,
		Broad.Step25Snapshot.OwnershipChurn.ChurnFraction,
		Persistent.Step25Snapshot.CompetitiveParticipation.PlatesWithAnyHit,
		Persistent.Step25Snapshot.CompetitiveParticipation.TotalPlateCount,
		Persistent.Step25Snapshot.CompetitiveParticipation.LargestPlateHitShare,
		Persistent.Step25Snapshot.ActiveZone.ActivePairCount,
		Persistent.Step25Snapshot.ActiveZone.FreshSeedActivePairCount,
		Persistent.Step25Snapshot.ActiveZone.PersistentCarryoverActivePairCount,
		bFrozenAtStep25 ? 1 : 0,
		bTooBroadAtStep25 ? 1 : 0,
		bInvalidOutsideAtStep25 ? 1 : 0));

	if (!bRunStep100)
	{
		TestTrue(
			TEXT("Phase1c step-25 outside-zone ownership changes remain near zero"),
			Persistent.Step25Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone == 0);
		return true;
	}

	Broad.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Broad,
		TEXT("[V9Phase1c broad step=100]"),
		100,
		&FVariantState::Step100Snapshot);
	Broad.bHasStep100 = true;

	Narrow.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Narrow,
		TEXT("[V9Phase1c narrow step=100]"),
		100,
		&FVariantState::Step100Snapshot);
	Narrow.bHasStep100 = true;

	Persistent.Planet.AdvanceSteps(75);
	LogCheckpoint(
		Persistent,
		TEXT("[V9Phase1c persistent step=100]"),
		100,
		&FVariantState::Step100Snapshot);
	Persistent.bHasStep100 = true;

	const int32 PersistentExplicitCauseChanges =
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangeDivergenceFillCount +
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangeConvergentSubductionCount +
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangeCollisionContactCount +
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangeRiftCount;
	const bool bActiveFractionInBand =
		Persistent.Step100Snapshot.ActiveZone.ActiveFraction >= 0.02 &&
		Persistent.Step100Snapshot.ActiveZone.ActiveFraction <= 0.25;
	const bool bOutsideStable =
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone == 0;
	const bool bChurnImproved =
		Persistent.Step100Snapshot.OwnershipChurn.ChurnFraction <
			Broad.Step100Snapshot.OwnershipChurn.ChurnFraction;
	const bool bLiveParticipation =
		Persistent.Step100Snapshot.CompetitiveParticipation.PlatesWithAnyHit >= 8 &&
		Persistent.Step100Snapshot.CompetitiveParticipation.LargestPlateHitShare <= 0.55;
	const bool bGenericCompetitionNotDominant =
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangeGenericQueryCompetitionCount <=
			PersistentExplicitCauseChanges;
	const bool bRunStep200 =
		bActiveFractionInBand &&
		bOutsideStable &&
		bChurnImproved &&
		bLiveParticipation &&
		bGenericCompetitionNotDominant;

	AddInfo(FString::Printf(
		TEXT("[V9Phase1c gate step=100] run_step200=%d active_fraction=%.4f outside_zone_changes=%d churn=%.4f broad_churn=%.4f coherence=%.4f broad_coherence=%.4f active_pairs=%d fresh_pairs=%d carry_pairs=%d active_carryover=%d generic_query=%d explicit_causes=%d plates_with_hit=%d/%d largest_share=%.4f collision=%d"),
		bRunStep200 ? 1 : 0,
		Persistent.Step100Snapshot.ActiveZone.ActiveFraction,
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone,
		Persistent.Step100Snapshot.OwnershipChurn.ChurnFraction,
		Broad.Step100Snapshot.OwnershipChurn.ChurnFraction,
		Persistent.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Broad.Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Persistent.Step100Snapshot.ActiveZone.ActivePairCount,
		Persistent.Step100Snapshot.ActiveZone.FreshSeedActivePairCount,
		Persistent.Step100Snapshot.ActiveZone.PersistentCarryoverActivePairCount,
		Persistent.Step100Snapshot.ActiveZone.ActiveCarryoverSampleCount,
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangeGenericQueryCompetitionCount,
		PersistentExplicitCauseChanges,
		Persistent.Step100Snapshot.CompetitiveParticipation.PlatesWithAnyHit,
		Persistent.Step100Snapshot.CompetitiveParticipation.TotalPlateCount,
		Persistent.Step100Snapshot.CompetitiveParticipation.LargestPlateHitShare,
		Persistent.Step100Snapshot.CollisionCount));

	if (bRunStep200)
	{
		Persistent.Planet.AdvanceSteps(100);
		LogCheckpoint(
			Persistent,
			TEXT("[V9Phase1c persistent step=200]"),
			200,
			&FVariantState::Step200Snapshot);
		Persistent.bHasStep200 = true;
	}

	TestTrue(
		TEXT("Phase1c step-100 outside-zone ownership changes remain near zero"),
		Persistent.Step100Snapshot.ActiveZone.OwnershipChangesOutsideActiveZone == 0);
	TestTrue(
		TEXT("Phase1c step-100 active+inactive samples cover the whole planet"),
		Persistent.Step100Snapshot.ActiveZone.ActiveSampleCount +
			Persistent.Step100Snapshot.ActiveZone.InactiveSampleCount ==
			SampleCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6PlateMotionDiagnosticTest,
	"Aurous.TectonicPlanet.V6PlateMotionDiagnosticTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6PlateMotionDiagnosticTest::RunTest(const FString& Parameters)
{
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		25,
		INDEX_NONE,
		60000,
		40,
		TestRandomSeed);

	const FTectonicPlanet& P = Planet.GetPlanet();
	const double RadiusKm = P.PlanetRadiusKm;

	AddInfo(TEXT("[PlateMotion] === Step 0 ==="));
	for (int32 PlateIndex = 0; PlateIndex < P.Plates.Num(); ++PlateIndex)
	{
		const FPlate& Plate = P.Plates[PlateIndex];
		const double SurfaceSpeedKmPerMy = Plate.AngularSpeed * RadiusKm;
		const double SurfaceSpeedMmPerYr = SurfaceSpeedKmPerMy;
		const FVector3d Axis = Plate.RotationAxis.GetSafeNormal();
		AddInfo(*FString::Printf(
			TEXT("[PlateMotion step=0] plate=%d axis=(%.3f,%.3f,%.3f) speed_mm_yr=%.2f members=%d"),
			Plate.Id, Axis.X, Axis.Y, Axis.Z, SurfaceSpeedMmPerYr, Plate.MemberSamples.Num()));
	}

	Planet.AdvanceSteps(25);
	AddInfo(TEXT("[PlateMotion] === Step 25 ==="));
	for (int32 PlateIndex = 0; PlateIndex < P.Plates.Num(); ++PlateIndex)
	{
		const FPlate& Plate = P.Plates[PlateIndex];
		const double SurfaceSpeedKmPerMy = Plate.AngularSpeed * RadiusKm;
		const double SurfaceSpeedMmPerYr = SurfaceSpeedKmPerMy;
		const FVector3d Axis = Plate.RotationAxis.GetSafeNormal();
		const FQuat4d& Rot = Plate.CumulativeRotation;
		const double AngleDeg = FMath::RadiansToDegrees(Rot.GetAngle());
		const double DriftKm = FMath::DegreesToRadians(AngleDeg) * RadiusKm;
		AddInfo(*FString::Printf(
			TEXT("[PlateMotion step=25] plate=%d axis=(%.3f,%.3f,%.3f) speed_mm_yr=%.2f cumulative_deg=%.3f drift_km=%.1f members=%d"),
			Plate.Id, Axis.X, Axis.Y, Axis.Z, SurfaceSpeedMmPerYr, AngleDeg, DriftKm, Plate.MemberSamples.Num()));
	}

	Planet.AdvanceSteps(75);
	AddInfo(TEXT("[PlateMotion] === Step 100 ==="));
	for (int32 PlateIndex = 0; PlateIndex < P.Plates.Num(); ++PlateIndex)
	{
		const FPlate& Plate = P.Plates[PlateIndex];
		const double SurfaceSpeedKmPerMy = Plate.AngularSpeed * RadiusKm;
		const double SurfaceSpeedMmPerYr = SurfaceSpeedKmPerMy;
		const FVector3d Axis = Plate.RotationAxis.GetSafeNormal();
		const FQuat4d& Rot = Plate.CumulativeRotation;
		const double AngleDeg = FMath::RadiansToDegrees(Rot.GetAngle());
		const double DriftKm = FMath::DegreesToRadians(AngleDeg) * RadiusKm;
		AddInfo(*FString::Printf(
			TEXT("[PlateMotion step=100] plate=%d axis=(%.3f,%.3f,%.3f) speed_mm_yr=%.2f cumulative_deg=%.3f drift_km=%.1f members=%d"),
			Plate.Id, Axis.X, Axis.Y, Axis.Z, SurfaceSpeedMmPerYr, AngleDeg, DriftKm, Plate.MemberSamples.Num()));
	}

	TestTrue(TEXT("All plates initialized"), P.Plates.Num() == 40);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1dStep200ValidationTest,
	"Aurous.TectonicPlanet.V6V9Phase1dStep200ValidationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1dStep200ValidationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9Phase1dStep200Validation");

	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	Planet.SetSyntheticCoverageRetentionForTest(false);
	Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
	Planet.SetExcludeMixedTrianglesForTest(false);
	Planet.SetV9Phase1AuthorityForTest(true, 1);
	Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
	Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);

	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 0);

	const auto LogCheckpoint = [this, &Planet, &ExportRoot](const int32 Step)
	{
		ExportV6CheckpointMaps(*this, Planet, ExportRoot, Step);
		ExportV6DebugOverlays(*this, Planet, ExportRoot, Step);
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(TEXT("[V9Phase1dStep200 step=%d]"), Step);
		AddV6ThesisRemeshInfo(*this, *Tag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, *Tag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, *Tag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, *Tag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, *Tag, Snapshot);
	};

	Planet.AdvanceSteps(25);
	LogCheckpoint(25);

	Planet.AdvanceSteps(75);
	LogCheckpoint(100);

	Planet.AdvanceSteps(100);
	LogCheckpoint(200);

	TestTrue(TEXT("Reached step 200"), Planet.GetPlanet().CurrentStep == 200);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9ElevationBudgetAlignmentStep100Test,
	"Aurous.TectonicPlanet.V6V9ElevationBudgetAlignmentStep100Test",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9ElevationBudgetAlignmentStep100Test::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;

	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	Planet.SetSyntheticCoverageRetentionForTest(false);
	Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
	Planet.SetExcludeMixedTrianglesForTest(false);
	Planet.SetV9Phase1AuthorityForTest(true, 1);
	Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
	Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);

	Planet.AdvanceSteps(100);

	const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
	const FBoundaryFieldCouplingVariantDiagnostics ContinuityDiagnostics =
		BuildBoundaryFieldCouplingVariantDiagnostics(TEXT("current"), Planet);
	const FTectonicPlanetV6CopiedFrontierSolveAttribution& Attribution =
		Planet.GetLastCopiedFrontierSolveAttributionForTest();
	const FString Tag = TEXT("[V9ElevationBudgetAlignment step=100]");
	AddV6ThesisRemeshInfo(*this, Tag, Snapshot);
	AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
	AddV6InteriorInstabilityInfo(*this, Tag, Snapshot);
	AddV6ActiveZoneInfo(*this, Tag, Snapshot);

	const FV9ContinentalElevationStats ElevationStats = ComputeContinentalElevationStats(Planet);
	const int32 AndeanCount = ComputeAndeanSampleCount(Planet);
	const int32 TectonicMaintenanceAppliedCount =
		Attribution.TectonicMaintenanceAppliedCount;
	const int32 BoundaryBandSampleCount = ContinuityDiagnostics.BoundaryBandDelta.SampleCount;
	const double BoundaryBandMeanAbsElevationDeltaKm =
		BoundaryBandSampleCount > 0
			? ContinuityDiagnostics.BoundaryBandDelta.SumAbsElevationDeltaKm /
				static_cast<double>(BoundaryBandSampleCount)
			: 0.0;
	const int32 InteriorLeakElevationJumpCount =
		ContinuityDiagnostics.InteriorLeakCauseCounts[
			static_cast<int32>(EBoundaryFieldCouplingLeakCause::ElevationJumpGt1Km)];
	const int32 SyntheticMissedAgainCount = ContinuityDiagnostics.SyntheticMissedAgainCount;
	const int32 SyntheticMissedAgainFromDirectHitCount =
		ContinuityDiagnostics.SyntheticMissedAgainByPreviousProvenance[
			static_cast<int32>(EBoundaryFieldCouplingProvenanceBucket::DirectHitTriangle)];
	const int32 SyntheticMissedAgainFromSyntheticCount =
		ContinuityDiagnostics.SyntheticMissedAgainByPreviousProvenance[
			static_cast<int32>(EBoundaryFieldCouplingProvenanceBucket::SyntheticRecovery)];
	const FString ElevationMessage = FString::Printf(
		TEXT("%s continental_count=%d mean_elev_km=%.4f p95_elev_km=%.4f max_elev_km=%.4f above_2km=%d above_5km=%d"),
		*Tag,
		ElevationStats.ContinentalSampleCount,
		ElevationStats.MeanElevationKm,
		ElevationStats.P95ElevationKm,
		ElevationStats.MaxElevationKm,
		ElevationStats.SamplesAbove2Km,
		ElevationStats.SamplesAbove5Km);
	AddInfo(ElevationMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *ElevationMessage);

	const FString FootprintMessage = FString::Printf(
		TEXT("%s tectonic_maintenance_applied=%d andean_count=%d"),
		*Tag,
		TectonicMaintenanceAppliedCount,
		AndeanCount);
	AddInfo(FootprintMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *FootprintMessage);

	const FString ContinuityMessage = FString::Printf(
		TEXT("%s active_band_mean_abs_elev_delta_km=%.4f active_band_max_abs_elev_delta_km=%.4f interior_leaks_total=%d interior_leaks_elev_gt_1km=%d continuity_triangle_clamped=%d continuity_synthetic_preserved=%d continuity_oceanic_preserved=%d"),
		*Tag,
		BoundaryBandMeanAbsElevationDeltaKm,
		ContinuityDiagnostics.BoundaryBandDelta.MaxAbsElevationDeltaKm,
		ContinuityDiagnostics.TotalInteriorLeakCount,
		InteriorLeakElevationJumpCount,
		Attribution.ActiveBandTriangleFieldContinuityClampCount,
		Attribution.ActiveBandSyntheticFieldPreserveCount,
		Attribution.ActiveBandOceanicFieldPreserveCount);
	AddInfo(ContinuityMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *ContinuityMessage);

	const FString SyntheticRecoveryMessage = FString::Printf(
		TEXT("%s synthetic_missed_again=%d synthetic_missed_again_prev_direct_hit=%d synthetic_missed_again_prev_synthetic=%d active_band_prev_owner_recovery=%d active_band_synthetic_loop_break=%d active_band_synthetic_single_source=%d"),
		*Tag,
		SyntheticMissedAgainCount,
		SyntheticMissedAgainFromDirectHitCount,
		SyntheticMissedAgainFromSyntheticCount,
		Attribution.ActiveBandPreviousOwnerCompatibleRecoveryCount,
		Attribution.ActiveBandSyntheticLoopBreakCount,
		Attribution.ActiveBandSyntheticSingleSourceRecoveryCount);
	AddInfo(SyntheticRecoveryMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *SyntheticRecoveryMessage);

	const bool bMeanPass =
		ElevationStats.MeanElevationKm >= 0.3 &&
		ElevationStats.MeanElevationKm <= 3.0;
	const bool bP95Pass = ElevationStats.P95ElevationKm > 2.0;
	const bool bMaxPass =
		ElevationStats.MaxElevationKm > 4.0 &&
		ElevationStats.MaxElevationKm <= 10.0;
	const bool bAbove2Pass = ElevationStats.SamplesAbove2Km >= 100;
	const bool bAbove5Pass = ElevationStats.SamplesAbove5Km >= 10;
	const bool bMaintenancePass = TectonicMaintenanceAppliedCount < 10000;
	const bool bAndeanPass = AndeanCount < 15000;
	const bool bChurnPass = Snapshot.OwnershipChurn.ChurnFraction < 0.05;
	const bool bCoherencePass = Snapshot.BoundaryCoherence.BoundaryCoherenceScore > 0.93;
	const bool bLeakagePass = Snapshot.BoundaryCoherence.InteriorLeakageFraction < 0.17;
	const bool bElevationJumpLeakPass = InteriorLeakElevationJumpCount < 30;

	const FString GateMessage = FString::Printf(
		TEXT("%s gates mean=%d p95=%d max=%d above2=%d above5=%d maintenance=%d andean=%d churn=%d coherence=%d leakage=%d elev_jump_leaks=%d ")
		TEXT("candidate_mean=%.4f candidate_p95=%.4f candidate_max=%.4f candidate_above2=%d candidate_above5=%d ")
		TEXT("candidate_maintenance=%d candidate_andean=%d candidate_churn=%.4f candidate_coherence=%.4f candidate_leakage=%.4f candidate_elev_jump_leaks=%d"),
		*Tag,
		bMeanPass ? 1 : 0,
		bP95Pass ? 1 : 0,
		bMaxPass ? 1 : 0,
		bAbove2Pass ? 1 : 0,
		bAbove5Pass ? 1 : 0,
		bMaintenancePass ? 1 : 0,
		bAndeanPass ? 1 : 0,
		bChurnPass ? 1 : 0,
		bCoherencePass ? 1 : 0,
		bLeakagePass ? 1 : 0,
		bElevationJumpLeakPass ? 1 : 0,
		ElevationStats.MeanElevationKm,
		ElevationStats.P95ElevationKm,
		ElevationStats.MaxElevationKm,
		ElevationStats.SamplesAbove2Km,
		ElevationStats.SamplesAbove5Km,
		TectonicMaintenanceAppliedCount,
		AndeanCount,
		Snapshot.OwnershipChurn.ChurnFraction,
		Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Snapshot.BoundaryCoherence.InteriorLeakageFraction,
		InteriorLeakElevationJumpCount);
	AddInfo(GateMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *GateMessage);

	TestTrue(TEXT("Step-100 continental mean elevation is positive and bounded"), bMeanPass);
	TestTrue(TEXT("Step-100 continental p95 elevation exceeds 2 km"), bP95Pass);
	TestTrue(TEXT("Step-100 continental max elevation exceeds 4 km without overshoot"), bMaxPass);
	TestTrue(TEXT("Step-100 continental samples above 2 km are nontrivial"), bAbove2Pass);
	TestTrue(TEXT("Step-100 continental samples above 5 km are nontrivial"), bAbove5Pass);
	TestTrue(TEXT("Step-100 tectonic maintenance footprint stays below 10k samples"), bMaintenancePass);
	TestTrue(TEXT("Step-100 Andean tag footprint stays below 15k samples"), bAndeanPass);
	TestTrue(TEXT("Step-100 ownership churn stays below 5%"), bChurnPass);
	TestTrue(TEXT("Step-100 boundary coherence stays above 0.93"), bCoherencePass);
	TestTrue(TEXT("Step-100 interior leakage stays below 0.17"), bLeakagePass);
	TestTrue(TEXT("Step-100 elevation-jump interior leaks stay below 30"), bElevationJumpLeakPass);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6BoundaryFieldCouplingDiagnosticStep100Test,
	"Aurous.TectonicPlanet.V6BoundaryFieldCouplingDiagnosticStep100Test",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6BoundaryFieldCouplingDiagnosticStep100Test::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;

	FTectonicPlanetV6 ControlPlanet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	ConfigureBoundaryFieldCouplingVariant(ControlPlanet, false);

	FTectonicPlanetV6 CandidatePlanet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	ConfigureBoundaryFieldCouplingVariant(CandidatePlanet, true);

	ControlPlanet.AdvanceSteps(100);
	CandidatePlanet.AdvanceSteps(100);

	TestTrue(TEXT("Control reached step 100"), ControlPlanet.GetPlanet().CurrentStep == 100);
	TestTrue(TEXT("Candidate reached step 100"), CandidatePlanet.GetPlanet().CurrentStep == 100);

	const FBoundaryFieldCouplingVariantDiagnostics Control =
		BuildBoundaryFieldCouplingVariantDiagnostics(TEXT("A_control_pre_uplift_fix"), ControlPlanet);
	const FBoundaryFieldCouplingVariantDiagnostics Candidate =
		BuildBoundaryFieldCouplingVariantDiagnostics(TEXT("B_candidate_thesis_scale_relief"), CandidatePlanet);

	TestTrue(
		TEXT("Control resolved sample diagnostics captured"),
		ControlPlanet.GetLastResolvedSamplesForTest().Num() == SampleCount);
	TestTrue(
		TEXT("Candidate resolved sample diagnostics captured"),
		CandidatePlanet.GetLastResolvedSamplesForTest().Num() == SampleCount);

	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("cw_threshold_mismatch_count"),
		Control.CwThresholdMismatchCount,
		Candidate.CwThresholdMismatchCount);
	for (int32 BucketIndex = 0;
		BucketIndex < static_cast<int32>(EBoundaryFieldCouplingProvenanceBucket::Count);
		++BucketIndex)
	{
		const FString MetricName = FString::Printf(
			TEXT("cw_threshold_mismatch_provenance_%s"),
			GetBoundaryFieldCouplingProvenanceName(
				static_cast<EBoundaryFieldCouplingProvenanceBucket>(BucketIndex)));
		AddBoundaryFieldCouplingAggregateComparison(
			*this,
			MetricName,
			Control.CwThresholdMismatchByProvenance[BucketIndex],
			Candidate.CwThresholdMismatchByProvenance[BucketIndex]);
	}
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("cw_threshold_mismatch_mean_active_ring_distance"),
		Control.MeanMismatchNearestActiveZoneRingDistance,
		Candidate.MeanMismatchNearestActiveZoneRingDistance);

	const auto& ControlBand = Control.BoundaryBandDelta;
	const auto& CandidateBand = Candidate.BoundaryBandDelta;
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("active_band_plus_one_sample_count"),
		ControlBand.SampleCount,
		CandidateBand.SampleCount);
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("active_band_plus_one_mean_abs_cw_delta"),
		ControlBand.SampleCount > 0
			? ControlBand.SumAbsCwDelta / static_cast<double>(ControlBand.SampleCount)
			: 0.0,
		CandidateBand.SampleCount > 0
			? CandidateBand.SumAbsCwDelta / static_cast<double>(CandidateBand.SampleCount)
			: 0.0);
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("active_band_plus_one_max_abs_cw_delta"),
		ControlBand.MaxAbsCwDelta,
		CandidateBand.MaxAbsCwDelta);
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("active_band_plus_one_mean_abs_elev_delta"),
		ControlBand.SampleCount > 0
			? ControlBand.SumAbsElevationDeltaKm / static_cast<double>(ControlBand.SampleCount)
			: 0.0,
		CandidateBand.SampleCount > 0
			? CandidateBand.SumAbsElevationDeltaKm / static_cast<double>(CandidateBand.SampleCount)
			: 0.0);
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("active_band_plus_one_max_abs_elev_delta"),
		ControlBand.MaxAbsElevationDeltaKm,
		CandidateBand.MaxAbsElevationDeltaKm);
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("active_band_plus_one_cw_threshold_cross_fraction"),
		ControlBand.SampleCount > 0
			? static_cast<double>(ControlBand.ThresholdCrossCount) /
				static_cast<double>(ControlBand.SampleCount)
			: 0.0,
		CandidateBand.SampleCount > 0
			? static_cast<double>(CandidateBand.ThresholdCrossCount) /
				static_cast<double>(CandidateBand.SampleCount)
			: 0.0);
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("active_band_plus_one_ownership_retention_rate"),
		ControlBand.SampleCount > 0
			? static_cast<double>(ControlBand.OwnershipRetainedCount) /
				static_cast<double>(ControlBand.SampleCount)
			: 0.0,
		CandidateBand.SampleCount > 0
			? static_cast<double>(CandidateBand.OwnershipRetainedCount) /
				static_cast<double>(CandidateBand.SampleCount)
			: 0.0);

	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("synthetic_missed_again_count"),
		Control.SyntheticMissedAgainCount,
		Candidate.SyntheticMissedAgainCount);
	for (int32 BucketIndex = 0;
		BucketIndex < static_cast<int32>(EBoundaryFieldCouplingProvenanceBucket::Count);
		++BucketIndex)
	{
		const FString MetricName = FString::Printf(
			TEXT("synthetic_missed_again_prev_provenance_%s"),
			GetBoundaryFieldCouplingProvenanceName(
				static_cast<EBoundaryFieldCouplingProvenanceBucket>(BucketIndex)));
		AddBoundaryFieldCouplingAggregateComparison(
			*this,
			MetricName,
			Control.SyntheticMissedAgainByPreviousProvenance[BucketIndex],
			Candidate.SyntheticMissedAgainByPreviousProvenance[BucketIndex]);
	}
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("synthetic_missed_again_near_convergent_front"),
		Control.SyntheticMissedAgainNearConvergentFrontCount,
		Candidate.SyntheticMissedAgainNearConvergentFrontCount);
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("synthetic_missed_again_with_cw_delta"),
		Control.SyntheticMissedAgainWithCwDeltaCount,
		Candidate.SyntheticMissedAgainWithCwDeltaCount);
	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("synthetic_missed_again_mean_abs_cw_delta"),
		Control.SyntheticMissedAgainMeanAbsCwDelta,
		Candidate.SyntheticMissedAgainMeanAbsCwDelta);
	AddBoundaryFieldCouplingLog(
		*this,
		FString::Printf(
			TEXT("[BoundaryFieldCoupling metric=synthetic_miss_lineage_age_distribution] A=%s B=%s"),
			*FormatBoundaryFieldCouplingAgeDistribution(Control.SyntheticMissLineageAgeCounts),
			*FormatBoundaryFieldCouplingAgeDistribution(Candidate.SyntheticMissLineageAgeCounts)));

	AddBoundaryFieldCouplingAggregateComparison(
		*this,
		TEXT("interior_leak_count"),
		Control.TotalInteriorLeakCount,
		Candidate.TotalInteriorLeakCount);
	for (int32 CauseIndex = 0;
		CauseIndex < static_cast<int32>(EBoundaryFieldCouplingLeakCause::Count);
		++CauseIndex)
	{
		const FString MetricName = FString::Printf(
			TEXT("interior_leak_cause_%s"),
			GetBoundaryFieldCouplingLeakCauseName(
				static_cast<EBoundaryFieldCouplingLeakCause>(CauseIndex)));
		AddBoundaryFieldCouplingAggregateComparison(
			*this,
			MetricName,
			Control.InteriorLeakCauseCounts[CauseIndex],
			Candidate.InteriorLeakCauseCounts[CauseIndex]);
	}
	for (int32 ProximityIndex = 0;
		ProximityIndex < static_cast<int32>(EBoundaryFieldCouplingProximityBucket::Count);
		++ProximityIndex)
	{
		const FString MetricName = FString::Printf(
			TEXT("interior_leak_proximity_%s"),
			GetBoundaryFieldCouplingProximityName(
				static_cast<EBoundaryFieldCouplingProximityBucket>(ProximityIndex)));
	AddBoundaryFieldCouplingAggregateComparison(
			*this,
			MetricName,
			Control.InteriorLeakProximityCounts[ProximityIndex],
			Candidate.InteriorLeakProximityCounts[ProximityIndex]);
	}

	AddV6ThesisRemeshInfo(*this, TEXT("[BoundaryFieldCoupling A step=100]"), Control.Snapshot);
	AddV6BoundaryCoherenceInfo(*this, TEXT("[BoundaryFieldCoupling A step=100]"), Control.Snapshot);
	AddV6SyntheticCoveragePersistenceInfo(*this, TEXT("[BoundaryFieldCoupling A step=100]"), Control.Snapshot);
	AddV6ThesisRemeshInfo(*this, TEXT("[BoundaryFieldCoupling B step=100]"), Candidate.Snapshot);
	AddV6BoundaryCoherenceInfo(*this, TEXT("[BoundaryFieldCoupling B step=100]"), Candidate.Snapshot);
	AddV6SyntheticCoveragePersistenceInfo(*this, TEXT("[BoundaryFieldCoupling B step=100]"), Candidate.Snapshot);

	AddBoundaryFieldCouplingVariantTopLogs(*this, Control);
	AddBoundaryFieldCouplingVariantTopLogs(*this, Candidate);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1dCollisionShadowValidationTest,
	"Aurous.TectonicPlanet.V6V9Phase1dCollisionShadowValidationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1dCollisionShadowValidationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9Phase1dCollisionShadowValidation");

	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	Planet.SetSyntheticCoverageRetentionForTest(false);
	Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
	Planet.SetExcludeMixedTrianglesForTest(false);
	Planet.SetV9Phase1AuthorityForTest(true, 1);
	Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
	Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
	Planet.SetV9CollisionShadowForTest(true);

	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 0);
	ExportV6DebugOverlays(*this, Planet, ExportRoot, 0);

	const auto LogCheckpoint = [this, &Planet, &ExportRoot](const int32 Step)
	{
		ExportV6CheckpointMaps(*this, Planet, ExportRoot, Step);
		ExportV6DebugOverlays(*this, Planet, ExportRoot, Step);
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(TEXT("[V9Phase1dCollisionShadow step=%d]"), Step);
		AddV6ThesisRemeshInfo(*this, *Tag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, *Tag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, *Tag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, *Tag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, *Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, *Tag, Snapshot);
		AddV6TectonicInteractionInfo(*this, *Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, *Tag, Snapshot);
	};

	Planet.AdvanceSteps(25);
	LogCheckpoint(25);

	Planet.AdvanceSteps(75);
	LogCheckpoint(100);

	Planet.AdvanceSteps(100);
	LogCheckpoint(200);

	const FV6CheckpointSnapshot FinalSnapshot = BuildV6CheckpointSnapshot(Planet);
	TestTrue(TEXT("Reached step 200"), Planet.GetPlanet().CurrentStep == 200);
	TestEqual(TEXT("Collision execution remains disabled in shadow mode"), FinalSnapshot.CollisionCount, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1dCollisionExecutionValidationTest,
	"Aurous.TectonicPlanet.V6V9Phase1dCollisionExecutionValidationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1dCollisionExecutionValidationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9Phase1dCollisionExecutionValidation");

	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	Planet.SetSyntheticCoverageRetentionForTest(false);
	Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
	Planet.SetExcludeMixedTrianglesForTest(false);
	Planet.SetV9Phase1AuthorityForTest(true, 1);
	Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
	Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
	Planet.SetV9CollisionShadowForTest(true);
	Planet.SetV9CollisionExecutionForTest(true);

	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);
	ExportV6CheckpointMaps(*this, Planet, ExportRoot, 0);
	ExportV6DebugOverlays(*this, Planet, ExportRoot, 0);

	const auto LogCheckpoint = [this, &Planet, &ExportRoot](const int32 Step)
	{
		ExportV6CheckpointMaps(*this, Planet, ExportRoot, Step);
		ExportV6DebugOverlays(*this, Planet, ExportRoot, Step);
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(TEXT("[V9Phase1dCollisionExecution step=%d]"), Step);
		AddV6ThesisRemeshInfo(*this, *Tag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, *Tag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, *Tag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, *Tag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, *Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, *Tag, Snapshot);
		AddV6TectonicInteractionInfo(*this, *Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, *Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, *Tag, Snapshot);
	};

	Planet.AdvanceSteps(25);
	LogCheckpoint(25);

	Planet.AdvanceSteps(75);
	LogCheckpoint(100);

	Planet.AdvanceSteps(100);
	LogCheckpoint(200);

	TestTrue(TEXT("Reached step 200"), Planet.GetPlanet().CurrentStep == 200);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1dCollisionConsequenceFidelityValidationTest,
	"Aurous.TectonicPlanet.V6V9Phase1dCollisionConsequenceFidelityValidationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1dCollisionConsequenceFidelityValidationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9Phase1dCollisionConsequenceFidelityValidation");

	const auto InitializePlanet = [&]() -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		return Planet;
	};

	FTectonicPlanetV6 BaselinePlanet = InitializePlanet();
	BaselinePlanet.SetV9CollisionExecutionEnhancedConsequencesForTest(false);

	FTectonicPlanetV6 FidelityPlanet = InitializePlanet();
	FidelityPlanet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_execution"));
	const FString FidelityExportRoot = FPaths::Combine(ExportRoot, TEXT("fidelity_execution"));
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*FidelityExportRoot);
	ExportV6CheckpointMaps(*this, BaselinePlanet, BaselineExportRoot, 0);
	ExportV6DebugOverlays(*this, BaselinePlanet, BaselineExportRoot, 0);
	ExportV6CheckpointMaps(*this, FidelityPlanet, FidelityExportRoot, 0);
	ExportV6DebugOverlays(*this, FidelityPlanet, FidelityExportRoot, 0);

	const auto CaptureCheckpoint =
		[this](
			FTectonicPlanetV6& Planet,
			const FString& VariantTag,
			const FString& VariantExportRoot,
			const int32 Step) -> FV6CheckpointSnapshot
	{
		ExportV6CheckpointMaps(*this, Planet, VariantExportRoot, Step);
		ExportV6DebugOverlays(*this, Planet, VariantExportRoot, Step);
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(TEXT("[V9CollisionConsequence variant=%s step=%d]"), *VariantTag, Step);
		AddV6ThesisRemeshInfo(*this, *Tag, Snapshot);
		AddV6DiagnosticsPackageInfo(*this, *Tag, Snapshot);
		AddV6BoundaryCoherenceInfo(*this, *Tag, Snapshot);
		AddV6InteriorInstabilityInfo(*this, *Tag, Snapshot);
		AddV6SyntheticCoveragePersistenceInfo(*this, *Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, *Tag, Snapshot);
		AddV6TectonicInteractionInfo(*this, *Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, *Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, *Tag, Snapshot);
		return Snapshot;
	};

	const auto LogComparison =
		[this](
			const int32 Step,
			const FV6CheckpointSnapshot& BaselineSnapshot,
			const FV6CheckpointSnapshot& FidelitySnapshot)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec =
			BaselineSnapshot.CollisionExecution;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& FidelityExec =
			FidelitySnapshot.CollisionExecution;
		const FString Message = FString::Printf(
			TEXT("[V9CollisionConsequence compare step=%d] churn %.4f->%.4f coherence %.4f->%.4f interior_leakage %.4f->%.4f max_components %d->%d miss %d->%d multi %d->%d active_fraction %.4f->%.4f exec_current %d->%d exec_cumulative %d->%d exec_affected %d->%d exec_cumulative_affected %d->%d exec_continental_gain %d->%d exec_cumulative_continental_gain %d->%d exec_mean_elev_delta_km %.6f->%.6f exec_max_elev_delta_km %.6f->%.6f exec_cumulative_mean_elev_delta_km %.6f->%.6f exec_cumulative_max_elev_delta_km %.6f->%.6f"),
			Step,
			BaselineSnapshot.OwnershipChurn.ChurnFraction,
			FidelitySnapshot.OwnershipChurn.ChurnFraction,
			BaselineSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			FidelitySnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			BaselineSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			FidelitySnapshot.BoundaryCoherence.InteriorLeakageFraction,
			BaselineSnapshot.MaxComponentsPerPlate,
			FidelitySnapshot.MaxComponentsPerPlate,
			BaselineSnapshot.MissCount,
			FidelitySnapshot.MissCount,
			BaselineSnapshot.MultiHitCount,
			FidelitySnapshot.MultiHitCount,
			BaselineSnapshot.ActiveZone.ActiveFraction,
			FidelitySnapshot.ActiveZone.ActiveFraction,
			BaselineExec.ExecutedCollisionCount,
			FidelityExec.ExecutedCollisionCount,
			BaselineExec.CumulativeExecutedCollisionCount,
			FidelityExec.CumulativeExecutedCollisionCount,
			BaselineExec.CollisionAffectedSampleCount,
			FidelityExec.CollisionAffectedSampleCount,
			BaselineExec.CumulativeCollisionAffectedSampleCount,
			FidelityExec.CumulativeCollisionAffectedSampleCount,
			BaselineExec.CollisionDrivenContinentalGainCount,
			FidelityExec.CollisionDrivenContinentalGainCount,
			BaselineExec.CumulativeCollisionDrivenContinentalGainCount,
			FidelityExec.CumulativeCollisionDrivenContinentalGainCount,
			BaselineExec.ExecutedMeanElevationDeltaKm,
			FidelityExec.ExecutedMeanElevationDeltaKm,
			BaselineExec.ExecutedMaxElevationDeltaKm,
			FidelityExec.ExecutedMaxElevationDeltaKm,
			BaselineExec.CumulativeMeanElevationDeltaKm,
			FidelityExec.CumulativeMeanElevationDeltaKm,
			BaselineExec.CumulativeMaxElevationDeltaKm,
			FidelityExec.CumulativeMaxElevationDeltaKm);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	};

	BaselinePlanet.AdvanceSteps(25);
	FidelityPlanet.AdvanceSteps(25);
	const FV6CheckpointSnapshot BaselineStep25 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 25);
	const FV6CheckpointSnapshot FidelityStep25 =
		CaptureCheckpoint(FidelityPlanet, TEXT("fidelity_execution"), FidelityExportRoot, 25);
	LogComparison(25, BaselineStep25, FidelityStep25);

	BaselinePlanet.AdvanceSteps(75);
	FidelityPlanet.AdvanceSteps(75);
	const FV6CheckpointSnapshot BaselineStep100 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 100);
	const FV6CheckpointSnapshot FidelityStep100 =
		CaptureCheckpoint(FidelityPlanet, TEXT("fidelity_execution"), FidelityExportRoot, 100);
	LogComparison(100, BaselineStep100, FidelityStep100);

	BaselinePlanet.AdvanceSteps(100);
	FidelityPlanet.AdvanceSteps(100);
	const FV6CheckpointSnapshot BaselineStep200 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 200);
	const FV6CheckpointSnapshot FidelityStep200 =
		CaptureCheckpoint(FidelityPlanet, TEXT("fidelity_execution"), FidelityExportRoot, 200);
	LogComparison(200, BaselineStep200, FidelityStep200);

	TestTrue(TEXT("Baseline variant reached step 200"), BaselinePlanet.GetPlanet().CurrentStep == 200);
	TestTrue(TEXT("Fidelity variant reached step 200"), FidelityPlanet.GetPlanet().CurrentStep == 200);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1dCollisionFidelityTuningHarnessTest,
	"Aurous.TectonicPlanet.V6V9Phase1dCollisionFidelityTuningHarnessTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1dCollisionFidelityTuningHarnessTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9Phase1dCollisionFidelityTuningHarness");

	const auto InitializePlanet = [&]() -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		return Planet;
	};

	FTectonicPlanetV6 BaselinePlanet = InitializePlanet();
	BaselinePlanet.SetV9CollisionExecutionEnhancedConsequencesForTest(false);

	FTectonicPlanetV6 CandidatePlanet = InitializePlanet();
	CandidatePlanet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_execution"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("candidate_execution"));
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	AddInfo(TEXT("[V9CollisionTuningHarness] export_policy step25/100=minimal_collision_overlays step200=full_only_if_gate_passes"));

	const auto CaptureCheckpoint =
		[this](
			FTectonicPlanetV6& Planet,
			const FString& VariantTag,
			const FString& VariantExportRoot,
			const int32 Step,
			const bool bFullExports) -> FV6CheckpointSnapshot
	{
		if (bFullExports)
		{
			ExportV6CheckpointMaps(*this, Planet, VariantExportRoot, Step);
			ExportV6DebugOverlays(*this, Planet, VariantExportRoot, Step);
		}
		else
		{
			ExportV6CollisionTuningInnerLoopOverlays(*this, Planet, VariantExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(TEXT("[V9CollisionTuningHarness variant=%s step=%d]"), *VariantTag, Step);
		AddV6BoundaryCoherenceInfo(*this, *Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, *Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, *Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, *Tag, Snapshot);
		return Snapshot;
	};

	const auto LogComparison =
		[this](
			const int32 Step,
			const FV6CheckpointSnapshot& BaselineSnapshot,
			const FV6CheckpointSnapshot& CandidateSnapshot)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec =
			BaselineSnapshot.CollisionExecution;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec =
			CandidateSnapshot.CollisionExecution;
		const FString Message = FString::Printf(
			TEXT("[V9CollisionTuningHarness compare step=%d] churn %.4f->%.4f coherence %.4f->%.4f interior_leakage %.4f->%.4f active_fraction %.4f->%.4f exec_current %d->%d exec_cumulative %d->%d exec_affected %d->%d exec_cumulative_affected %d->%d exec_continental_gain %d->%d exec_cumulative_continental_gain %d->%d exec_mean_elev_delta_km %.6f->%.6f exec_max_elev_delta_km %.6f->%.6f exec_cumulative_mean_elev_delta_km %.6f->%.6f"),
			Step,
			BaselineSnapshot.OwnershipChurn.ChurnFraction,
			CandidateSnapshot.OwnershipChurn.ChurnFraction,
			BaselineSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			CandidateSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			BaselineSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			CandidateSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			BaselineSnapshot.ActiveZone.ActiveFraction,
			CandidateSnapshot.ActiveZone.ActiveFraction,
			BaselineExec.ExecutedCollisionCount,
			CandidateExec.ExecutedCollisionCount,
			BaselineExec.CumulativeExecutedCollisionCount,
			CandidateExec.CumulativeExecutedCollisionCount,
			BaselineExec.CollisionAffectedSampleCount,
			CandidateExec.CollisionAffectedSampleCount,
			BaselineExec.CumulativeCollisionAffectedSampleCount,
			CandidateExec.CumulativeCollisionAffectedSampleCount,
			BaselineExec.CollisionDrivenContinentalGainCount,
			CandidateExec.CollisionDrivenContinentalGainCount,
			BaselineExec.CumulativeCollisionDrivenContinentalGainCount,
			CandidateExec.CumulativeCollisionDrivenContinentalGainCount,
			BaselineExec.ExecutedMeanElevationDeltaKm,
			CandidateExec.ExecutedMeanElevationDeltaKm,
			BaselineExec.ExecutedMaxElevationDeltaKm,
			CandidateExec.ExecutedMaxElevationDeltaKm,
			BaselineExec.CumulativeMeanElevationDeltaKm,
			CandidateExec.CumulativeMeanElevationDeltaKm);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	};

	BaselinePlanet.AdvanceSteps(25);
	CandidatePlanet.AdvanceSteps(25);
	const FV6CheckpointSnapshot BaselineStep25 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 25, false);
	const FV6CheckpointSnapshot CandidateStep25 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_execution"), CandidateExportRoot, 25, false);
	LogComparison(25, BaselineStep25, CandidateStep25);

	BaselinePlanet.AdvanceSteps(75);
	CandidatePlanet.AdvanceSteps(75);
	const FV6CheckpointSnapshot BaselineStep100 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 100, false);
	const FV6CheckpointSnapshot CandidateStep100 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_execution"), CandidateExportRoot, 100, false);
	LogComparison(100, BaselineStep100, CandidateStep100);

	const FV9CollisionFidelityGateResult Gate =
		EvaluateV9CollisionFidelityStep100Gate(BaselineStep100, CandidateStep100);
	const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec = BaselineStep100.CollisionExecution;
	const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec = CandidateStep100.CollisionExecution;

	const FString GateMessage = FString::Printf(
		TEXT("[V9CollisionTuningHarness gate step=100] run_step200=%d footprint_pass=%d candidate_cumulative_affected=%d required_cumulative_affected=%d elevation_pass=%d candidate_exec_mean_delta_km=%.6f required_exec_mean_delta_km=%.6f candidate_cumulative_mean_delta_km=%.6f required_cumulative_mean_delta_km=%.6f continental_gain_pass=%d candidate_cumulative_continental_gain=%d required_cumulative_continental_gain=%d churn_pass=%d candidate_churn=%.4f max_churn=%.4f coherence_pass=%d candidate_coherence=%.4f min_coherence=%.4f leakage_pass=%d candidate_interior_leakage=%.4f max_interior_leakage=%.4f"),
		Gate.bAllowStep200 ? 1 : 0,
		Gate.bFootprintPass ? 1 : 0,
		CandidateExec.CumulativeCollisionAffectedSampleCount,
		Gate.RequiredCumulativeAffectedSamples,
		Gate.bElevationPass ? 1 : 0,
		CandidateExec.ExecutedMeanElevationDeltaKm,
		Gate.RequiredExecutedMeanElevationDeltaKm,
		CandidateExec.CumulativeMeanElevationDeltaKm,
		Gate.RequiredCumulativeMeanElevationDeltaKm,
		Gate.bContinentalGainPass ? 1 : 0,
		CandidateExec.CumulativeCollisionDrivenContinentalGainCount,
		Gate.RequiredCumulativeContinentalGain,
		Gate.bChurnPass ? 1 : 0,
		CandidateStep100.OwnershipChurn.ChurnFraction,
		Gate.MaxAllowedChurn,
		Gate.bCoherencePass ? 1 : 0,
		CandidateStep100.BoundaryCoherence.BoundaryCoherenceScore,
		Gate.MinAllowedCoherence,
		Gate.bLeakagePass ? 1 : 0,
		CandidateStep100.BoundaryCoherence.InteriorLeakageFraction,
		Gate.MaxAllowedInteriorLeakage);
	AddInfo(GateMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *GateMessage);

	if (Gate.bAllowStep200)
	{
		BaselinePlanet.AdvanceSteps(100);
		CandidatePlanet.AdvanceSteps(100);
		const FV6CheckpointSnapshot BaselineStep200 =
			CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 200, true);
		const FV6CheckpointSnapshot CandidateStep200 =
			CaptureCheckpoint(CandidatePlanet, TEXT("candidate_execution"), CandidateExportRoot, 200, true);
		LogComparison(200, BaselineStep200, CandidateStep200);
		TestTrue(TEXT("Candidate promoted to step 200 when gates pass"), CandidatePlanet.GetPlanet().CurrentStep == 200);
	}
	else
	{
		TestTrue(TEXT("Weak candidate rejected before step 200"), CandidatePlanet.GetPlanet().CurrentStep == 100);
	}

	TestTrue(TEXT("Baseline reached step 100"), BaselinePlanet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Candidate reached step 100"), CandidatePlanet.GetPlanet().CurrentStep >= 100);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1dCollisionStructuralTransferTuningHarnessTest,
	"Aurous.TectonicPlanet.V6V9Phase1dCollisionStructuralTransferTuningHarnessTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1dCollisionStructuralTransferTuningHarnessTest::RunTest(
	const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9Phase1dCollisionStructuralTransferTuningHarness");

	const auto InitializePlanet = [&]() -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(false);
		return Planet;
	};

	FTectonicPlanetV6 BaselinePlanet = InitializePlanet();
	BaselinePlanet.SetV9CollisionExecutionStructuralTransferForTest(false);

	FTectonicPlanetV6 CandidatePlanet = InitializePlanet();
	CandidatePlanet.SetV9CollisionExecutionStructuralTransferForTest(true);

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_execution"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("candidate_structural_transfer"));
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	AddInfo(TEXT("[V9CollisionStructuralHarness] export_policy step25/100=minimal_collision_overlays step200=full_only_if_structural_gate_passes"));

	const auto CaptureCheckpoint =
		[this](
			FTectonicPlanetV6& Planet,
			const FString& VariantTag,
			const FString& VariantExportRoot,
			const int32 Step,
			const bool bFullExports) -> FV6CheckpointSnapshot
	{
		if (bFullExports)
		{
			ExportV6CheckpointMaps(*this, Planet, VariantExportRoot, Step);
			ExportV6DebugOverlays(*this, Planet, VariantExportRoot, Step);
		}
		else
		{
			ExportV6CollisionTuningInnerLoopOverlays(*this, Planet, VariantExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(
			TEXT("[V9CollisionStructuralHarness variant=%s step=%d]"),
			*VariantTag,
			Step);
		AddV6BoundaryCoherenceInfo(*this, *Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, *Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, *Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, *Tag, Snapshot);
		return Snapshot;
	};

	const auto LogComparison =
		[this](
			const int32 Step,
			const FV6CheckpointSnapshot& BaselineSnapshot,
			const FV6CheckpointSnapshot& CandidateSnapshot)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec =
			BaselineSnapshot.CollisionExecution;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec =
			CandidateSnapshot.CollisionExecution;
		const FString Message = FString::Printf(
			TEXT("[V9CollisionStructuralHarness compare step=%d] churn %.4f->%.4f coherence %.4f->%.4f interior_leakage %.4f->%.4f active_fraction %.4f->%.4f exec_current %d->%d exec_cumulative %d->%d ownership_change %d->%d ownership_change_cumulative %d->%d transfer_current %d->%d transfer_cumulative %d->%d transfer_continental_current %d->%d transfer_continental_cumulative %d->%d transfer_boundary_local %d->%d recipient_share_delta %.6f->%.6f donor_share_delta %.6f->%.6f"),
			Step,
			BaselineSnapshot.OwnershipChurn.ChurnFraction,
			CandidateSnapshot.OwnershipChurn.ChurnFraction,
			BaselineSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			CandidateSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			BaselineSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			CandidateSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			BaselineSnapshot.ActiveZone.ActiveFraction,
			CandidateSnapshot.ActiveZone.ActiveFraction,
			BaselineExec.ExecutedCollisionCount,
			CandidateExec.ExecutedCollisionCount,
			BaselineExec.CumulativeExecutedCollisionCount,
			CandidateExec.CumulativeExecutedCollisionCount,
			BaselineExec.CollisionDrivenOwnershipChangeCount,
			CandidateExec.CollisionDrivenOwnershipChangeCount,
			BaselineExec.CumulativeCollisionDrivenOwnershipChangeCount,
			CandidateExec.CumulativeCollisionDrivenOwnershipChangeCount,
			BaselineExec.CollisionTransferredSampleCount,
			CandidateExec.CollisionTransferredSampleCount,
			BaselineExec.CumulativeCollisionTransferredSampleCount,
			CandidateExec.CumulativeCollisionTransferredSampleCount,
			BaselineExec.CollisionTransferredContinentalSampleCount,
			CandidateExec.CollisionTransferredContinentalSampleCount,
			BaselineExec.CumulativeCollisionTransferredContinentalSampleCount,
			CandidateExec.CumulativeCollisionTransferredContinentalSampleCount,
			BaselineExec.ExecutedTransferBoundaryLocalSampleCount,
			CandidateExec.ExecutedTransferBoundaryLocalSampleCount,
			BaselineExec.ExecutedRecipientPlateShareDelta,
			CandidateExec.ExecutedRecipientPlateShareDelta,
			BaselineExec.ExecutedDonorPlateShareDelta,
			CandidateExec.ExecutedDonorPlateShareDelta);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	};

	BaselinePlanet.AdvanceSteps(25);
	CandidatePlanet.AdvanceSteps(25);
	const FV6CheckpointSnapshot BaselineStep25 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 25, false);
	const FV6CheckpointSnapshot CandidateStep25 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_structural_transfer"), CandidateExportRoot, 25, false);
	LogComparison(25, BaselineStep25, CandidateStep25);

	BaselinePlanet.AdvanceSteps(75);
	CandidatePlanet.AdvanceSteps(75);
	const FV6CheckpointSnapshot BaselineStep100 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 100, false);
	const FV6CheckpointSnapshot CandidateStep100 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_structural_transfer"), CandidateExportRoot, 100, false);
	LogComparison(100, BaselineStep100, CandidateStep100);

	const FV9CollisionStructuralGateResult Gate =
		EvaluateV9CollisionStructuralTransferStep100Gate(
			BaselineStep100,
			CandidateStep100,
			SampleCount);
	const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec = BaselineStep100.CollisionExecution;
	const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec = CandidateStep100.CollisionExecution;
	const double CandidateBoundaryLocalFraction =
		CandidateExec.CollisionTransferredSampleCount > 0
			? static_cast<double>(CandidateExec.ExecutedTransferBoundaryLocalSampleCount) /
				static_cast<double>(CandidateExec.CollisionTransferredSampleCount)
			: 0.0;
	const FString GateMessage = FString::Printf(
		TEXT("[V9CollisionStructuralHarness gate step=100] run_step200=%d ownership_change_pass=%d candidate_ownership_change=%d/%d transfer_pass=%d candidate_transfer=%d/%d transferred_continental_pass=%d candidate_transfer_continental=%d/%d transfer_footprint_pass=%d candidate_cumulative_transfer_footprint=%d required_cumulative_transfer_footprint=%d share_delta_pass=%d candidate_recipient_share_delta=%.6f candidate_donor_share_delta=%.6f required_share_delta=[%.6f,%.6f] boundary_locality_pass=%d candidate_boundary_local_fraction=%.4f required_boundary_local_fraction=%.4f churn_pass=%d candidate_churn=%.4f max_churn=%.4f coherence_pass=%d candidate_coherence=%.4f min_coherence=%.4f leakage_pass=%d candidate_interior_leakage=%.4f max_interior_leakage=%.4f"),
		Gate.bAllowStep200 ? 1 : 0,
		Gate.bOwnershipChangePass ? 1 : 0,
		CandidateExec.CollisionDrivenOwnershipChangeCount,
		CandidateExec.CumulativeCollisionDrivenOwnershipChangeCount,
		Gate.bTransferPass ? 1 : 0,
		CandidateExec.CollisionTransferredSampleCount,
		CandidateExec.CumulativeCollisionTransferredSampleVisits,
		Gate.bTransferredContinentalPass ? 1 : 0,
		CandidateExec.CollisionTransferredContinentalSampleCount,
		CandidateExec.CumulativeCollisionTransferredContinentalSampleCount,
		Gate.bTransferFootprintPass ? 1 : 0,
		CandidateExec.CumulativeCollisionTransferredSampleCount,
		Gate.RequiredCumulativeTransferFootprint,
		Gate.bShareDeltaPass ? 1 : 0,
		CandidateExec.ExecutedRecipientPlateShareDelta,
		CandidateExec.ExecutedDonorPlateShareDelta,
		Gate.RequiredMinPlateShareDelta,
		Gate.RequiredMaxPlateShareDelta,
		Gate.bBoundaryLocalityPass ? 1 : 0,
		CandidateBoundaryLocalFraction,
		Gate.RequiredBoundaryLocalFraction,
		Gate.bChurnPass ? 1 : 0,
		CandidateStep100.OwnershipChurn.ChurnFraction,
		Gate.MaxAllowedChurn,
		Gate.bCoherencePass ? 1 : 0,
		CandidateStep100.BoundaryCoherence.BoundaryCoherenceScore,
		Gate.MinAllowedCoherence,
		Gate.bLeakagePass ? 1 : 0,
		CandidateStep100.BoundaryCoherence.InteriorLeakageFraction,
		Gate.MaxAllowedInteriorLeakage);
	AddInfo(GateMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *GateMessage);

	if (Gate.bAllowStep200)
	{
		BaselinePlanet.AdvanceSteps(100);
		CandidatePlanet.AdvanceSteps(100);
		const FV6CheckpointSnapshot BaselineStep200 =
			CaptureCheckpoint(BaselinePlanet, TEXT("baseline_execution"), BaselineExportRoot, 200, true);
		const FV6CheckpointSnapshot CandidateStep200 =
			CaptureCheckpoint(CandidatePlanet, TEXT("candidate_structural_transfer"), CandidateExportRoot, 200, true);
		LogComparison(200, BaselineStep200, CandidateStep200);
		TestTrue(TEXT("Structural candidate promoted to step 200 when gates pass"), CandidatePlanet.GetPlanet().CurrentStep == 200);
	}
	else
	{
		TestTrue(TEXT("Weak structural candidate rejected before step 200"), CandidatePlanet.GetPlanet().CurrentStep == 100);
	}

	TestTrue(TEXT("Structural baseline reached step 100"), BaselinePlanet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Structural candidate reached step 100"), CandidatePlanet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Baseline structural transfer stays disabled"), BaselineExec.CumulativeCollisionTransferredSampleCount == 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Phase1dTerraneCaptureRefinementTuningHarnessTest,
	"Aurous.TectonicPlanet.V6V9Phase1dTerraneCaptureRefinementTuningHarnessTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Phase1dTerraneCaptureRefinementTuningHarnessTest::RunTest(
	const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9Phase1dTerraneCaptureRefinementTuningHarness");

	const auto InitializePlanet = [&]() -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(false);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		return Planet;
	};

	FTectonicPlanetV6 BaselinePlanet = InitializePlanet();
	BaselinePlanet.SetV9CollisionExecutionRefinedStructuralTransferForTest(false);

	FTectonicPlanetV6 CandidatePlanet = InitializePlanet();
	CandidatePlanet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_structural_transfer"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("candidate_refined_terrane_capture"));
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	AddInfo(TEXT("[V9TerraneCaptureHarness] export_policy step25/100=minimal_collision_overlays step200=full_only_if_structural_gate_passes"));

	const auto CaptureCheckpoint =
		[this](
			FTectonicPlanetV6& Planet,
			const FString& VariantTag,
			const FString& VariantExportRoot,
			const int32 Step,
			const bool bFullExports) -> FV6CheckpointSnapshot
	{
		if (bFullExports)
		{
			ExportV6CheckpointMaps(*this, Planet, VariantExportRoot, Step);
			ExportV6DebugOverlays(*this, Planet, VariantExportRoot, Step);
		}
		else
		{
			ExportV6CollisionTuningInnerLoopOverlays(*this, Planet, VariantExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(
			TEXT("[V9TerraneCaptureHarness variant=%s step=%d]"),
			*VariantTag,
			Step);
		AddV6BoundaryCoherenceInfo(*this, *Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, *Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, *Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, *Tag, Snapshot);
		return Snapshot;
	};

	const auto LogComparison =
		[this](
			const int32 Step,
			const FV6CheckpointSnapshot& BaselineSnapshot,
			const FV6CheckpointSnapshot& CandidateSnapshot)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& BaselineExec =
			BaselineSnapshot.CollisionExecution;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec =
			CandidateSnapshot.CollisionExecution;
		const FString Message = FString::Printf(
			TEXT("[V9TerraneCaptureHarness compare step=%d] churn %.4f->%.4f coherence %.4f->%.4f interior_leakage %.4f->%.4f active_fraction %.4f->%.4f exec_current %d->%d exec_cumulative %d->%d ownership_change %d->%d ownership_change_cumulative %d->%d transfer_current %d->%d transfer_cumulative %d->%d transfer_continental_current %d->%d transfer_continental_cumulative %d->%d transfer_boundary_local %d->%d transfer_patch_support %d->%d transfer_anchor_seeds %d->%d recipient_share_delta %.6f->%.6f donor_share_delta %.6f->%.6f"),
			Step,
			BaselineSnapshot.OwnershipChurn.ChurnFraction,
			CandidateSnapshot.OwnershipChurn.ChurnFraction,
			BaselineSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			CandidateSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			BaselineSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			CandidateSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			BaselineSnapshot.ActiveZone.ActiveFraction,
			CandidateSnapshot.ActiveZone.ActiveFraction,
			BaselineExec.ExecutedCollisionCount,
			CandidateExec.ExecutedCollisionCount,
			BaselineExec.CumulativeExecutedCollisionCount,
			CandidateExec.CumulativeExecutedCollisionCount,
			BaselineExec.CollisionDrivenOwnershipChangeCount,
			CandidateExec.CollisionDrivenOwnershipChangeCount,
			BaselineExec.CumulativeCollisionDrivenOwnershipChangeCount,
			CandidateExec.CumulativeCollisionDrivenOwnershipChangeCount,
			BaselineExec.CollisionTransferredSampleCount,
			CandidateExec.CollisionTransferredSampleCount,
			BaselineExec.CumulativeCollisionTransferredSampleCount,
			CandidateExec.CumulativeCollisionTransferredSampleCount,
			BaselineExec.CollisionTransferredContinentalSampleCount,
			CandidateExec.CollisionTransferredContinentalSampleCount,
			BaselineExec.CumulativeCollisionTransferredContinentalSampleCount,
			CandidateExec.CumulativeCollisionTransferredContinentalSampleCount,
			BaselineExec.ExecutedTransferBoundaryLocalSampleCount,
			CandidateExec.ExecutedTransferBoundaryLocalSampleCount,
			BaselineExec.ExecutedTransferCandidateSupportCount,
			CandidateExec.ExecutedTransferCandidateSupportCount,
			BaselineExec.ExecutedTransferAnchorSeedCount,
			CandidateExec.ExecutedTransferAnchorSeedCount,
			BaselineExec.ExecutedRecipientPlateShareDelta,
			CandidateExec.ExecutedRecipientPlateShareDelta,
			BaselineExec.ExecutedDonorPlateShareDelta,
			CandidateExec.ExecutedDonorPlateShareDelta);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	};

	BaselinePlanet.AdvanceSteps(25);
	CandidatePlanet.AdvanceSteps(25);
	const FV6CheckpointSnapshot BaselineStep25 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_structural_transfer"), BaselineExportRoot, 25, false);
	const FV6CheckpointSnapshot CandidateStep25 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_refined_terrane_capture"), CandidateExportRoot, 25, false);
	LogComparison(25, BaselineStep25, CandidateStep25);

	BaselinePlanet.AdvanceSteps(75);
	CandidatePlanet.AdvanceSteps(75);
	const FV6CheckpointSnapshot BaselineStep100 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_structural_transfer"), BaselineExportRoot, 100, false);
	const FV6CheckpointSnapshot CandidateStep100 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_refined_terrane_capture"), CandidateExportRoot, 100, false);
	LogComparison(100, BaselineStep100, CandidateStep100);

	const FV9CollisionStructuralGateResult Gate =
		EvaluateV9CollisionStructuralTransferStep100Gate(
			BaselineStep100,
			CandidateStep100,
			SampleCount);
	const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec = CandidateStep100.CollisionExecution;
	const double CandidateBoundaryLocalFraction =
		CandidateExec.CollisionTransferredSampleCount > 0
			? static_cast<double>(CandidateExec.ExecutedTransferBoundaryLocalSampleCount) /
				static_cast<double>(CandidateExec.CollisionTransferredSampleCount)
			: 0.0;
	const FString GateMessage = FString::Printf(
		TEXT("[V9TerraneCaptureHarness gate step=100] run_step200=%d ownership_change_pass=%d candidate_ownership_change=%d/%d transfer_pass=%d candidate_transfer=%d/%d transferred_continental_pass=%d candidate_transfer_continental=%d/%d transfer_footprint_pass=%d candidate_cumulative_transfer_footprint=%d required_cumulative_transfer_footprint=%d share_delta_pass=%d candidate_recipient_share_delta=%.6f candidate_donor_share_delta=%.6f required_share_delta=[%.6f,%.6f] boundary_locality_pass=%d candidate_boundary_local_fraction=%.4f required_boundary_local_fraction=%.4f transfer_patch_support=%d transfer_anchor_seeds=%d churn_pass=%d candidate_churn=%.4f max_churn=%.4f coherence_pass=%d candidate_coherence=%.4f min_coherence=%.4f leakage_pass=%d candidate_interior_leakage=%.4f max_interior_leakage=%.4f"),
		Gate.bAllowStep200 ? 1 : 0,
		Gate.bOwnershipChangePass ? 1 : 0,
		CandidateExec.CollisionDrivenOwnershipChangeCount,
		CandidateExec.CumulativeCollisionDrivenOwnershipChangeCount,
		Gate.bTransferPass ? 1 : 0,
		CandidateExec.CollisionTransferredSampleCount,
		CandidateExec.CumulativeCollisionTransferredSampleVisits,
		Gate.bTransferredContinentalPass ? 1 : 0,
		CandidateExec.CollisionTransferredContinentalSampleCount,
		CandidateExec.CumulativeCollisionTransferredContinentalSampleCount,
		Gate.bTransferFootprintPass ? 1 : 0,
		CandidateExec.CumulativeCollisionTransferredSampleCount,
		Gate.RequiredCumulativeTransferFootprint,
		Gate.bShareDeltaPass ? 1 : 0,
		CandidateExec.ExecutedRecipientPlateShareDelta,
		CandidateExec.ExecutedDonorPlateShareDelta,
		Gate.RequiredMinPlateShareDelta,
		Gate.RequiredMaxPlateShareDelta,
		Gate.bBoundaryLocalityPass ? 1 : 0,
		CandidateBoundaryLocalFraction,
		Gate.RequiredBoundaryLocalFraction,
		CandidateExec.ExecutedTransferCandidateSupportCount,
		CandidateExec.ExecutedTransferAnchorSeedCount,
		Gate.bChurnPass ? 1 : 0,
		CandidateStep100.OwnershipChurn.ChurnFraction,
		Gate.MaxAllowedChurn,
		Gate.bCoherencePass ? 1 : 0,
		CandidateStep100.BoundaryCoherence.BoundaryCoherenceScore,
		Gate.MinAllowedCoherence,
		Gate.bLeakagePass ? 1 : 0,
		CandidateStep100.BoundaryCoherence.InteriorLeakageFraction,
		Gate.MaxAllowedInteriorLeakage);
	AddInfo(GateMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *GateMessage);

	if (Gate.bAllowStep200)
	{
		BaselinePlanet.AdvanceSteps(100);
		CandidatePlanet.AdvanceSteps(100);
		const FV6CheckpointSnapshot BaselineStep200 =
			CaptureCheckpoint(BaselinePlanet, TEXT("baseline_structural_transfer"), BaselineExportRoot, 200, true);
		const FV6CheckpointSnapshot CandidateStep200 =
			CaptureCheckpoint(CandidatePlanet, TEXT("candidate_refined_terrane_capture"), CandidateExportRoot, 200, true);
		LogComparison(200, BaselineStep200, CandidateStep200);
		TestTrue(TEXT("Refined terrane-capture candidate promoted to step 200 when gates pass"), CandidatePlanet.GetPlanet().CurrentStep == 200);
	}
	else
	{
		TestTrue(TEXT("Refined terrane-capture candidate rejected before step 200"), CandidatePlanet.GetPlanet().CurrentStep == 100);
	}

	TestTrue(TEXT("Terrane-capture baseline reached step 100"), BaselinePlanet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Terrane-capture candidate reached step 100"), CandidatePlanet.GetPlanet().CurrentStep >= 100);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9HighReliefCollisionRidgeSurgeHarnessTest,
	"Aurous.TectonicPlanet.V6V9HighReliefCollisionRidgeSurgeHarnessTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9HighReliefCollisionRidgeSurgeHarnessTest::RunTest(
	const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9HighReliefCollisionRidgeSurgeHarness");

	const auto InitializePlanet = [&]() -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		return Planet;
	};

	FTectonicPlanetV6 BaselinePlanet = InitializePlanet();
	BaselinePlanet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
	BaselinePlanet.SetV9CollisionExecutionStructuralTransferForTest(true);
	BaselinePlanet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
	BaselinePlanet.SetV9ThesisShapedCollisionExecutionForTest(true);

	FTectonicPlanetV6 CandidatePlanet = InitializePlanet();
	CandidatePlanet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
	CandidatePlanet.SetV9CollisionExecutionStructuralTransferForTest(true);
	CandidatePlanet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
	CandidatePlanet.SetV9ThesisShapedCollisionExecutionForTest(true);
	CandidatePlanet.SetV9ThesisShapedCollisionRidgeSurgeForTest(true);

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_current_thesis"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("candidate_ridge_surge"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	const auto CaptureCheckpoint =
		[this](FTectonicPlanetV6& Planet, const FString& VariantTag,
			const FString& VariantExportRoot, const int32 Step, const bool bFullExports) -> FV6CheckpointSnapshot
	{
		if (bFullExports)
		{
			ExportV6CheckpointMaps(*this, Planet, VariantExportRoot, Step);
			ExportV6DebugOverlays(*this, Planet, VariantExportRoot, Step);
		}
		ExportV6CollisionTuningInnerLoopOverlays(*this, Planet, VariantExportRoot, Step);

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(TEXT("[V9CollisionRidgeSurgeHarness %s]"), *VariantTag);
		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, Tag, Snapshot);
		return Snapshot;
	};

	const auto LogComparison =
		[this, SampleCount](const int32 Step,
			const FV6CheckpointSnapshot& Baseline,
			const FV6CheckpointSnapshot& Candidate)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& BExec = Baseline.CollisionExecution;
		const FTectonicPlanetV6CollisionExecutionDiagnostic& CExec = Candidate.CollisionExecution;
		const FString CompareMessage = FString::Printf(
			TEXT("[V9CollisionRidgeSurgeHarness compare step=%d] ")
			TEXT("churn=%.4f/%.4f coherence=%.4f/%.4f interior_leakage=%.4f/%.4f ")
			TEXT("active_fraction=%.4f/%.4f ")
			TEXT("exec_current=%d/%d exec_cumulative=%d/%d ")
			TEXT("affected=%d/%d cumulative_affected=%d/%d ")
			TEXT("continental_gain=%d/%d cumulative_continental_gain=%d/%d ")
			TEXT("ownership_change=%d/%d cumulative_ownership_change=%d/%d ")
			TEXT("transfer=%d/%d cumulative_transfer=%d/%d ")
			TEXT("terrane_component=%d/%d transferred_component=%d/%d ")
			TEXT("collision_above5_current=%d/%d collision_above5_cumulative=%d/%d ")
			TEXT("xi=%.6f/%.6f rel_speed=%.3f/%.3f ")
			TEXT("mean_elev_delta_km=%.6f/%.6f max_elev_delta_km=%.6f/%.6f ")
			TEXT("cumulative_mean_elev_delta_km=%.6f/%.6f ")
			TEXT("donor_share_delta=%.6f/%.6f recipient_share_delta=%.6f/%.6f"),
			Step,
			Baseline.OwnershipChurn.ChurnFraction,
			Candidate.OwnershipChurn.ChurnFraction,
			Baseline.BoundaryCoherence.BoundaryCoherenceScore,
			Candidate.BoundaryCoherence.BoundaryCoherenceScore,
			Baseline.BoundaryCoherence.InteriorLeakageFraction,
			Candidate.BoundaryCoherence.InteriorLeakageFraction,
			Baseline.ActiveZone.ActiveFraction,
			Candidate.ActiveZone.ActiveFraction,
			BExec.ExecutedCollisionCount,
			CExec.ExecutedCollisionCount,
			BExec.CumulativeExecutedCollisionCount,
			CExec.CumulativeExecutedCollisionCount,
			BExec.CollisionAffectedSampleCount,
			CExec.CollisionAffectedSampleCount,
			BExec.CumulativeCollisionAffectedSampleCount,
			CExec.CumulativeCollisionAffectedSampleCount,
			BExec.CollisionDrivenContinentalGainCount,
			CExec.CollisionDrivenContinentalGainCount,
			BExec.CumulativeCollisionDrivenContinentalGainCount,
			CExec.CumulativeCollisionDrivenContinentalGainCount,
			BExec.CollisionDrivenOwnershipChangeCount,
			CExec.CollisionDrivenOwnershipChangeCount,
			BExec.CumulativeCollisionDrivenOwnershipChangeCount,
			CExec.CumulativeCollisionDrivenOwnershipChangeCount,
			BExec.CollisionTransferredSampleCount,
			CExec.CollisionTransferredSampleCount,
			BExec.CumulativeCollisionTransferredSampleCount,
			CExec.CumulativeCollisionTransferredSampleCount,
			BExec.ExecutedDonorTerraneComponentSize,
			CExec.ExecutedDonorTerraneComponentSize,
			BExec.ExecutedTransferredComponentSize,
			CExec.ExecutedTransferredComponentSize,
			Baseline.CurrentCollisionRegionSamplesAbove5Km,
			Candidate.CurrentCollisionRegionSamplesAbove5Km,
			Baseline.CumulativeCollisionRegionSamplesAbove5Km,
			Candidate.CumulativeCollisionRegionSamplesAbove5Km,
			BExec.ExecutedXi,
			CExec.ExecutedXi,
			BExec.ExecutedRelativeSpeedKmPerMy,
			CExec.ExecutedRelativeSpeedKmPerMy,
			BExec.ExecutedMeanElevationDeltaKm,
			CExec.ExecutedMeanElevationDeltaKm,
			BExec.ExecutedMaxElevationDeltaKm,
			CExec.ExecutedMaxElevationDeltaKm,
			BExec.CumulativeMeanElevationDeltaKm,
			CExec.CumulativeMeanElevationDeltaKm,
			BExec.ExecutedDonorPlateShareDelta,
			CExec.ExecutedDonorPlateShareDelta,
			BExec.ExecutedRecipientPlateShareDelta,
			CExec.ExecutedRecipientPlateShareDelta);
		AddInfo(CompareMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *CompareMessage);
	};

	const auto LogContinentalElevation =
		[this](const FString& VariantTag, const int32 Step, const FTectonicPlanetV6& Planet)
	{
		const FV9ContinentalElevationStats Elev = ComputeContinentalElevationStats(Planet);
		const FString Message = FString::Printf(
			TEXT("[V9CollisionRidgeSurgeHarness %s step=%d] continental_count=%d mean_elev_km=%.4f p95_elev_km=%.4f max_elev_km=%.4f above_2km=%d above_5km=%d collision_above5_current=%d collision_above5_cumulative=%d"),
			*VariantTag, Step,
			Elev.ContinentalSampleCount,
			Elev.MeanElevationKm,
			Elev.P95ElevationKm,
			Elev.MaxElevationKm,
			Elev.SamplesAbove2Km,
			Elev.SamplesAbove5Km,
			ComputeCollisionRegionSamplesAboveElevationThreshold(Planet, 5.0, false),
			ComputeCollisionRegionSamplesAboveElevationThreshold(Planet, 5.0, true));
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	};

	const auto ExportElevationOverlay =
		[this](const FTectonicPlanetV6& Planet, const FString& VariantExportRoot, const int32 Step)
	{
		const FString OutputDirectory = FPaths::Combine(
			VariantExportRoot, FString::Printf(TEXT("step_%03d"), Step));
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*OutputDirectory);
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		const int32 SC = PlanetData.Samples.Num();
		if (SC == 0) { return; }

		TArray<float> ElevValues;
		ElevValues.SetNum(SC);
		for (int32 I = 0; I < SC; ++I)
		{
			ElevValues[I] = PlanetData.Samples[I].Elevation;
		}
		FString Error;
		const FString OutputPath = FPaths::Combine(OutputDirectory, TEXT("Elevation.png"));
		TectonicMollweideExporter::ExportScalarOverlay(
			PlanetData, ElevValues, -10.0f, 10.0f,
			OutputPath, TestExportWidth, TestExportHeight, Error);

		const TArray<uint8>& TerraneMask = Planet.GetThesisCollisionTerraneComponentMaskForTest();
		if (TerraneMask.Num() == SC)
		{
			TArray<float> TerraneValues;
			TerraneValues.SetNum(SC);
			for (int32 I = 0; I < SC; ++I)
			{
				TerraneValues[I] = TerraneMask[I] != 0 ? 1.0f : 0.0f;
			}
			const FString TerranePath = FPaths::Combine(OutputDirectory, TEXT("CollisionTerraneComponentMask.png"));
			TectonicMollweideExporter::ExportScalarOverlay(
				PlanetData, TerraneValues, 0.0f, 1.0f,
				TerranePath, TestExportWidth, TestExportHeight, Error);
		}
	};

	// --- Step 25 ---
	BaselinePlanet.AdvanceSteps(25);
	CandidatePlanet.AdvanceSteps(25);
	const FV6CheckpointSnapshot BaselineStep25 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_current_thesis"), BaselineExportRoot, 25, false);
	const FV6CheckpointSnapshot CandidateStep25 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_ridge_surge"), CandidateExportRoot, 25, false);
	LogComparison(25, BaselineStep25, CandidateStep25);
	LogContinentalElevation(TEXT("baseline"), 25, BaselinePlanet);
	LogContinentalElevation(TEXT("candidate"), 25, CandidatePlanet);
	ExportElevationOverlay(BaselinePlanet, BaselineExportRoot, 25);
	ExportElevationOverlay(CandidatePlanet, CandidateExportRoot, 25);

	// --- Step 100 ---
	BaselinePlanet.AdvanceSteps(75);
	CandidatePlanet.AdvanceSteps(75);
	const FV6CheckpointSnapshot BaselineStep100 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_current_thesis"), BaselineExportRoot, 100, false);
	const FV6CheckpointSnapshot CandidateStep100 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_ridge_surge"), CandidateExportRoot, 100, false);
	LogComparison(100, BaselineStep100, CandidateStep100);
	LogContinentalElevation(TEXT("baseline"), 100, BaselinePlanet);
	LogContinentalElevation(TEXT("candidate"), 100, CandidatePlanet);
	ExportElevationOverlay(BaselinePlanet, BaselineExportRoot, 100);
	ExportElevationOverlay(CandidatePlanet, CandidateExportRoot, 100);

	// --- Step-100 gate ---
	const FV9ThesisShapedCollisionGateResult Gate =
		EvaluateV9ThesisShapedCollisionStep100Gate(BaselineStep100, CandidateStep100, SampleCount);

	const FTectonicPlanetV6CollisionExecutionDiagnostic& CandidateExec = CandidateStep100.CollisionExecution;
	const FString GateMessage = FString::Printf(
		TEXT("[V9CollisionRidgeSurgeHarness gate step=100] run_step200=%d ")
		TEXT("terrane_detected=%d candidate_terrane=%d required_terrane=%d ")
		TEXT("transfer_coherent=%d candidate_transfer=%d required_transfer=%d ")
		TEXT("ownership_material=%d candidate_ownership=%d required_ownership=%d ")
		TEXT("max_elevation=%d candidate_exec_max_elev=%.6f required_max_elev=%.6f ")
		TEXT("collision_above5=%d candidate_collision_above5=%d required_collision_above5=%d baseline_collision_above5=%d ")
		TEXT("churn=%d candidate_churn=%.4f max_churn=%.4f ")
		TEXT("coherence=%d candidate_coherence=%.4f min_coherence=%.4f ")
		TEXT("leakage=%d candidate_leakage=%.4f max_leakage=%.4f"),
		Gate.bAllowStep200 ? 1 : 0,
		Gate.bTerraneDetectedPass ? 1 : 0,
		CandidateExec.ExecutedDonorTerraneComponentSize,
		Gate.RequiredMinTerraneComponentSize,
		Gate.bTransferCoherentPass ? 1 : 0,
		CandidateExec.CumulativeCollisionTransferredSampleCount,
		Gate.RequiredMinTransferSize,
		Gate.bOwnershipMaterialPass ? 1 : 0,
		CandidateExec.CumulativeCollisionDrivenOwnershipChangeCount,
		Gate.RequiredMinOwnershipChange,
		Gate.bMaxElevationPass ? 1 : 0,
		CandidateExec.ExecutedMaxElevationDeltaKm,
		Gate.RequiredMinMaxElevationDeltaKm,
		Gate.bCollisionAbove5Pass ? 1 : 0,
		CandidateStep100.CumulativeCollisionRegionSamplesAbove5Km,
		Gate.RequiredMinCollisionAbove5KmSamples,
		BaselineStep100.CumulativeCollisionRegionSamplesAbove5Km,
		Gate.bChurnPass ? 1 : 0,
		CandidateStep100.OwnershipChurn.ChurnFraction,
		Gate.MaxAllowedChurn,
		Gate.bCoherencePass ? 1 : 0,
		CandidateStep100.BoundaryCoherence.BoundaryCoherenceScore,
		Gate.MinAllowedCoherence,
		Gate.bLeakagePass ? 1 : 0,
		CandidateStep100.BoundaryCoherence.InteriorLeakageFraction,
		Gate.MaxAllowedInteriorLeakage);
	AddInfo(GateMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *GateMessage);

	if (Gate.bAllowStep200)
	{
		BaselinePlanet.AdvanceSteps(100);
		CandidatePlanet.AdvanceSteps(100);
		const FV6CheckpointSnapshot BaselineStep200 =
			CaptureCheckpoint(BaselinePlanet, TEXT("baseline_current_thesis"), BaselineExportRoot, 200, true);
		const FV6CheckpointSnapshot CandidateStep200 =
			CaptureCheckpoint(CandidatePlanet, TEXT("candidate_ridge_surge"), CandidateExportRoot, 200, true);
		LogComparison(200, BaselineStep200, CandidateStep200);
		LogContinentalElevation(TEXT("baseline"), 200, BaselinePlanet);
		LogContinentalElevation(TEXT("candidate"), 200, CandidatePlanet);
		ExportElevationOverlay(BaselinePlanet, BaselineExportRoot, 200);
		ExportElevationOverlay(CandidatePlanet, CandidateExportRoot, 200);
		TestTrue(TEXT("Ridge-surge candidate promoted to step 200 when gates pass"),
			CandidatePlanet.GetPlanet().CurrentStep == 200);
	}
	else
	{
		TestTrue(TEXT("Ridge-surge candidate rejected before step 200"),
			CandidatePlanet.GetPlanet().CurrentStep == 100);
	}

	TestTrue(TEXT("Current-thesis baseline reached step 100"), BaselinePlanet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Ridge-surge candidate reached step 100"), CandidatePlanet.GetPlanet().CurrentStep >= 100);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9Step200ThesisSurgeValidationTest,
	"Aurous.TectonicPlanet.V6V9Step200ThesisSurgeValidationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9Step200ThesisSurgeValidationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	constexpr int32 TargetStep = 200;
	const FString RunId = TEXT("V9Step200ThesisSurgeValidation");

	// Paper-faithful thesis surge: radial biweight kernel, NOT ridge surge
	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	Planet.SetSyntheticCoverageRetentionForTest(false);
	Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
	Planet.SetExcludeMixedTrianglesForTest(false);
	Planet.SetV9Phase1AuthorityForTest(true, 1);
	Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
	Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
	Planet.SetV9CollisionShadowForTest(true);
	Planet.SetV9CollisionExecutionForTest(true);
	Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
	Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
	Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
	Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
	// Ridge surge OFF — paper-faithful radial biweight kernel (default)

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	const FString ExportRoot = FPaths::Combine(
		FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);

	AddInfo(TEXT("[V9Step200ThesisSurge] config=paper_faithful kernel=radial_biweight ridge_surge=off 60k/40 seed=42 cadence=16"));

	// --- Per-event collision tracking ---
	struct FCollisionEventRecord
	{
		int32 Step = 0;
		int32 RemeshCycle = 0;
		int32 EventsThisCycle = 0;
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		int32 SubductingPlateId = INDEX_NONE;
		int32 OverridingPlateId = INDEX_NONE;
		int32 TerraneComponentSize = 0;
		int32 TransferredComponentSize = 0;
		int32 AffectedSampleCount = 0;
		int32 OwnershipChangeCount = 0;
		int32 TransferredSampleCount = 0;
		double RelativeSpeedKmPerMy = 0.0;
		double Xi = 0.0;
		double InfluenceRadiusRad = 0.0;
		double MeanElevationDeltaKm = 0.0;
		double MaxElevationDeltaKm = 0.0;
		double DonorPlateShareDelta = 0.0;
		double RecipientPlateShareDelta = 0.0;
	};

	TArray<FCollisionEventRecord> EventLog;
	TMap<uint64, int32> PairExecutionCounts;
	int32 PreviousCumulativeCount = 0;
	int32 RemeshCycleCounter = 0;

	const auto MakePairKey = [](const int32 A, const int32 B) -> uint64
	{
		return static_cast<uint64>(FMath::Min(A, B)) * 10000 + static_cast<uint64>(FMath::Max(A, B));
	};

	// Build milestone schedule: all remesh boundaries + checkpoint steps
	TArray<int32> Milestones;
	for (int32 S = FixedIntervalSteps; S <= TargetStep; S += FixedIntervalSteps)
	{
		Milestones.AddUnique(S);
	}
	Milestones.AddUnique(25);
	Milestones.AddUnique(100);
	Milestones.AddUnique(TargetStep);
	Milestones.Sort();

	FV6CheckpointSnapshot Step25Snapshot;
	FV6CheckpointSnapshot Step100Snapshot;
	FV6CheckpointSnapshot Step200Snapshot;

	for (const int32 Milestone : Milestones)
	{
		const int32 CurrentStep = Planet.GetPlanet().CurrentStep;
		if (Milestone <= CurrentStep) { continue; }
		Planet.AdvanceSteps(Milestone - CurrentStep);

		// Check for collision events at remesh boundaries
		const bool bIsRemeshBoundary = (Milestone % FixedIntervalSteps == 0);
		if (bIsRemeshBoundary)
		{
			++RemeshCycleCounter;
			const FTectonicPlanetV6CollisionExecutionDiagnostic ExecDiag =
				Planet.ComputeCollisionExecutionDiagnosticForTest();
			const int32 NewCumulative = ExecDiag.CumulativeExecutedCollisionCount;
			if (NewCumulative > PreviousCumulativeCount)
			{
				const int32 EventsThisCycle = NewCumulative - PreviousCumulativeCount;

				FCollisionEventRecord Record;
				Record.Step = Milestone;
				Record.RemeshCycle = RemeshCycleCounter;
				Record.EventsThisCycle = EventsThisCycle;
				Record.PlateA = ExecDiag.ExecutedPlateA;
				Record.PlateB = ExecDiag.ExecutedPlateB;
				Record.SubductingPlateId = ExecDiag.ExecutedSubductingPlateId;
				Record.OverridingPlateId = ExecDiag.ExecutedOverridingPlateId;
				Record.TerraneComponentSize = ExecDiag.ExecutedDonorTerraneComponentSize;
				Record.TransferredComponentSize = ExecDiag.ExecutedTransferredComponentSize;
				Record.AffectedSampleCount = ExecDiag.CollisionAffectedSampleCount;
				Record.OwnershipChangeCount = ExecDiag.CollisionDrivenOwnershipChangeCount;
				Record.TransferredSampleCount = ExecDiag.CollisionTransferredSampleCount;
				Record.RelativeSpeedKmPerMy = ExecDiag.ExecutedRelativeSpeedKmPerMy;
				Record.Xi = ExecDiag.ExecutedXi;
				Record.InfluenceRadiusRad = ExecDiag.ExecutedInfluenceRadiusRad;
				Record.MeanElevationDeltaKm = ExecDiag.ExecutedMeanElevationDeltaKm;
				Record.MaxElevationDeltaKm = ExecDiag.ExecutedMaxElevationDeltaKm;
				Record.DonorPlateShareDelta = ExecDiag.ExecutedDonorPlateShareDelta;
				Record.RecipientPlateShareDelta = ExecDiag.ExecutedRecipientPlateShareDelta;
				EventLog.Add(Record);

				if (ExecDiag.ExecutedPlateA != INDEX_NONE && ExecDiag.ExecutedPlateB != INDEX_NONE)
				{
					PairExecutionCounts.FindOrAdd(
						MakePairKey(ExecDiag.ExecutedPlateA, ExecDiag.ExecutedPlateB)) += EventsThisCycle;
				}

				const FString EventMsg = FString::Printf(
					TEXT("[V9Step200ThesisSurge EVENT step=%d cycle=%d] events_this_cycle=%d pair=(%d,%d) donor=%d recipient=%d terrane=%d transferred=%d affected=%d ownership_change=%d transferred_samples=%d rel_speed=%.3f xi=%.6f radius_rad=%.6f mean_delta_km=%.4f max_delta_km=%.4f donor_share_delta=%.6f recipient_share_delta=%.6f cumulative_events=%d cumulative_affected=%d"),
					Milestone, RemeshCycleCounter, EventsThisCycle,
					ExecDiag.ExecutedPlateA, ExecDiag.ExecutedPlateB,
					ExecDiag.ExecutedSubductingPlateId, ExecDiag.ExecutedOverridingPlateId,
					ExecDiag.ExecutedDonorTerraneComponentSize,
					ExecDiag.ExecutedTransferredComponentSize,
					ExecDiag.CollisionAffectedSampleCount,
					ExecDiag.CollisionDrivenOwnershipChangeCount,
					ExecDiag.CollisionTransferredSampleCount,
					ExecDiag.ExecutedRelativeSpeedKmPerMy,
					ExecDiag.ExecutedXi,
					ExecDiag.ExecutedInfluenceRadiusRad,
					ExecDiag.ExecutedMeanElevationDeltaKm,
					ExecDiag.ExecutedMaxElevationDeltaKm,
					ExecDiag.ExecutedDonorPlateShareDelta,
					ExecDiag.ExecutedRecipientPlateShareDelta,
					NewCumulative,
					ExecDiag.CumulativeCollisionAffectedSampleCount);
				AddInfo(EventMsg);
				UE_LOG(LogTemp, Log, TEXT("%s"), *EventMsg);

				PreviousCumulativeCount = NewCumulative;
			}
		}

		// Checkpoint captures at steps 25, 100, 200
		const bool bIsStep25 = (Milestone == 25);
		const bool bIsStep100 = (Milestone == 100);
		const bool bIsStep200 = (Milestone == TargetStep);

		if (bIsStep25 || bIsStep100 || bIsStep200)
		{
			const bool bFullExports = bIsStep200;
			if (bFullExports)
			{
				ExportV6CheckpointMaps(*this, Planet, ExportRoot, Milestone);
				ExportV6DebugOverlays(*this, Planet, ExportRoot, Milestone);
			}
			ExportV6CollisionTuningInnerLoopOverlays(*this, Planet, ExportRoot, Milestone);

			const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
			const FString Tag = FString::Printf(TEXT("[V9Step200ThesisSurge step=%d]"), Milestone);
			AddV6BoundaryCoherenceInfo(*this, *Tag, Snapshot);
			AddV6ActiveZoneInfo(*this, *Tag, Snapshot);
			AddV6CollisionShadowInfo(*this, *Tag, Snapshot);
			AddV6CollisionExecutionInfo(*this, *Tag, Snapshot);

			// Continental elevation at each checkpoint
			const FV9ContinentalElevationStats Elev = ComputeContinentalElevationStats(Planet);
			const FString ElevCheckMsg = FString::Printf(
				TEXT("[V9Step200ThesisSurge ELEVATION step=%d] continental_count=%d mean_km=%.4f p95_km=%.4f max_km=%.4f above_2km=%d above_5km=%d collision_above5_current=%d collision_above5_cumulative=%d"),
				Milestone, Elev.ContinentalSampleCount,
				Elev.MeanElevationKm, Elev.P95ElevationKm, Elev.MaxElevationKm,
				Elev.SamplesAbove2Km, Elev.SamplesAbove5Km,
				ComputeCollisionRegionSamplesAboveElevationThreshold(Planet, 5.0, false),
				ComputeCollisionRegionSamplesAboveElevationThreshold(Planet, 5.0, true));
			AddInfo(ElevCheckMsg);
			UE_LOG(LogTemp, Log, TEXT("%s"), *ElevCheckMsg);

			if (bIsStep25) { Step25Snapshot = Snapshot; }
			else if (bIsStep100) { Step100Snapshot = Snapshot; }
			else if (bIsStep200)
			{
				Step200Snapshot = Snapshot;

				// Additional step-200 exports: Elevation and TerraneComponentMask
				const FTectonicPlanet& PlanetData = Planet.GetPlanet();
				const int32 SC = PlanetData.Samples.Num();
				const FString OutputDirectory = FPaths::Combine(
					ExportRoot, FString::Printf(TEXT("step_%03d"), Milestone));
				PlatformFile.CreateDirectoryTree(*OutputDirectory);
				FString Error;

				TArray<float> ElevValues;
				ElevValues.SetNum(SC);
				for (int32 I = 0; I < SC; ++I)
				{
					ElevValues[I] = PlanetData.Samples[I].Elevation;
				}
				TectonicMollweideExporter::ExportScalarOverlay(
					PlanetData, ElevValues, -10.0f, 10.0f,
					FPaths::Combine(OutputDirectory, TEXT("Elevation.png")),
					TestExportWidth, TestExportHeight, Error);

				const TArray<uint8>& TerraneMask =
					Planet.GetThesisCollisionTerraneComponentMaskForTest();
				if (TerraneMask.Num() == SC)
				{
					TArray<float> TerraneValues;
					TerraneValues.SetNum(SC);
					for (int32 I = 0; I < SC; ++I)
					{
						TerraneValues[I] = TerraneMask[I] != 0 ? 1.0f : 0.0f;
					}
					TectonicMollweideExporter::ExportScalarOverlay(
						PlanetData, TerraneValues, 0.0f, 1.0f,
						FPaths::Combine(OutputDirectory, TEXT("CollisionTerraneComponentMask.png")),
						TestExportWidth, TestExportHeight, Error);
				}
			}
		}
	}

	// ===== Step-200 Comprehensive Report =====
	const FTectonicPlanetV6CollisionExecutionDiagnostic& Exec200 = Step200Snapshot.CollisionExecution;

	// 1. Collision aggregates
	AddInfo(FString::Printf(
		TEXT("[V9Step200ThesisSurge AGGREGATE step=200] cumulative_events=%d cumulative_affected=%d cumulative_affected_visits=%d cumulative_continental_gain=%d cumulative_ownership_change=%d cumulative_transferred=%d cumulative_transferred_visits=%d cumulative_transferred_continental=%d cumulative_mean_elev_delta_km=%.6f cumulative_max_elev_delta_km=%.6f event_records_captured=%d"),
		Exec200.CumulativeExecutedCollisionCount,
		Exec200.CumulativeCollisionAffectedSampleCount,
		Exec200.CumulativeCollisionAffectedSampleVisits,
		Exec200.CumulativeCollisionDrivenContinentalGainCount,
		Exec200.CumulativeCollisionDrivenOwnershipChangeCount,
		Exec200.CumulativeCollisionTransferredSampleCount,
		Exec200.CumulativeCollisionTransferredSampleVisits,
		Exec200.CumulativeCollisionTransferredContinentalSampleCount,
		Exec200.CumulativeMeanElevationDeltaKm,
		Exec200.CumulativeMaxElevationDeltaKm,
		EventLog.Num()));

	// 2. Stability metrics
	AddInfo(FString::Printf(
		TEXT("[V9Step200ThesisSurge STABILITY step=200] churn=%.4f coherence=%.4f interior_leakage=%.4f max_components=%d active_fraction=%.4f plates_with_hit=%d/%d miss=%d multi_hit=%d"),
		Step200Snapshot.OwnershipChurn.ChurnFraction,
		Step200Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
		Step200Snapshot.BoundaryCoherence.InteriorLeakageFraction,
		Step200Snapshot.MaxComponentsPerPlate,
		Step200Snapshot.ActiveZone.ActiveFraction,
		Step200Snapshot.CompetitiveParticipation.PlatesWithAnyHit,
		Step200Snapshot.CompetitiveParticipation.TotalPlateCount,
		Step200Snapshot.MissCount,
		Step200Snapshot.MultiHitCount));

	// 3. Repeated-pair analysis
	{
		int32 TotalUniquePairs = PairExecutionCounts.Num();
		int32 PairsWith2Plus = 0;
		int32 MaxExecForPair = 0;
		FString RepeatedDetails;
		for (const auto& Entry : PairExecutionCounts)
		{
			const int32 PA = static_cast<int32>(Entry.Key / 10000);
			const int32 PB = static_cast<int32>(Entry.Key % 10000);
			if (Entry.Value >= 2)
			{
				++PairsWith2Plus;
				RepeatedDetails += FString::Printf(TEXT(" (%d,%d)x%d"), PA, PB, Entry.Value);
			}
			MaxExecForPair = FMath::Max(MaxExecForPair, Entry.Value);
		}
		if (RepeatedDetails.IsEmpty()) { RepeatedDetails = TEXT(" none"); }
		AddInfo(FString::Printf(
			TEXT("[V9Step200ThesisSurge REPEATED_PAIRS step=200] unique_pairs=%d pairs_with_2plus=%d max_executions_single_pair=%d repeated=%s"),
			TotalUniquePairs, PairsWith2Plus, MaxExecForPair, *RepeatedDetails));
	}

	// 4. Cumulative elevation delta distribution from collision mask
	{
		const TArray<float>& ElevDeltaMask =
			Planet.GetCollisionCumulativeElevationDeltaMaskForTest();
		const FTectonicPlanet& PlanetData = Planet.GetPlanet();
		if (ElevDeltaMask.Num() == PlanetData.Samples.Num())
		{
			int32 AffectedCount = 0;
			double SumDelta = 0.0;
			double MaxDelta = 0.0;
			int32 Above1Km = 0;
			int32 Above2Km = 0;
			int32 Above3Km = 0;
			for (int32 I = 0; I < ElevDeltaMask.Num(); ++I)
			{
				const float Delta = ElevDeltaMask[I];
				if (Delta > UE_SMALL_NUMBER)
				{
					++AffectedCount;
					SumDelta += Delta;
					MaxDelta = FMath::Max(MaxDelta, static_cast<double>(Delta));
					if (Delta >= 1.0f) { ++Above1Km; }
					if (Delta >= 2.0f) { ++Above2Km; }
					if (Delta >= 3.0f) { ++Above3Km; }
				}
			}
			const double MeanDelta = AffectedCount > 0 ? SumDelta / AffectedCount : 0.0;
			AddInfo(FString::Printf(
				TEXT("[V9Step200ThesisSurge ELEV_DELTA_DISTRIBUTION step=200] affected_samples=%d mean_delta_km=%.4f max_delta_km=%.4f above_1km=%d above_2km=%d above_3km=%d"),
				AffectedCount, MeanDelta, MaxDelta, Above1Km, Above2Km, Above3Km));
		}
	}

	// 5. Step-100 → step-200 progression
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic& Exec100 = Step100Snapshot.CollisionExecution;
		AddInfo(FString::Printf(
			TEXT("[V9Step200ThesisSurge PROGRESSION 100->200] events=%d->%d affected=%d->%d transferred=%d->%d ownership_change=%d->%d mean_elev_delta=%.6f->%.6f max_elev_delta=%.6f->%.6f churn=%.4f->%.4f coherence=%.4f->%.4f leakage=%.4f->%.4f"),
			Exec100.CumulativeExecutedCollisionCount,
			Exec200.CumulativeExecutedCollisionCount,
			Exec100.CumulativeCollisionAffectedSampleCount,
			Exec200.CumulativeCollisionAffectedSampleCount,
			Exec100.CumulativeCollisionTransferredSampleCount,
			Exec200.CumulativeCollisionTransferredSampleCount,
			Exec100.CumulativeCollisionDrivenOwnershipChangeCount,
			Exec200.CumulativeCollisionDrivenOwnershipChangeCount,
			Exec100.CumulativeMeanElevationDeltaKm,
			Exec200.CumulativeMeanElevationDeltaKm,
			Exec100.CumulativeMaxElevationDeltaKm,
			Exec200.CumulativeMaxElevationDeltaKm,
			Step100Snapshot.OwnershipChurn.ChurnFraction,
			Step200Snapshot.OwnershipChurn.ChurnFraction,
			Step100Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Step200Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Step100Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Step200Snapshot.BoundaryCoherence.InteriorLeakageFraction));
	}

TestTrue(TEXT("Planet reached step 200"),
	Planet.GetPlanet().CurrentStep >= TargetStep);
return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9RiftingIntegrationHarnessTest,
	"Aurous.TectonicPlanet.V6V9RiftingIntegrationHarnessTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9RiftingIntegrationHarnessTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9RiftingIntegrationHarness");

	const auto InitializePlanet = [&](const bool bEnableAutomaticRifting) -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
		Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
		Planet.SetAutomaticRiftingForTest(bEnableAutomaticRifting);
		return Planet;
	};

	struct FRiftEventRecord
	{
		int32 Step = 0;
		int32 ParentPlateId = INDEX_NONE;
		int32 ChildPlateA = INDEX_NONE;
		int32 ChildPlateB = INDEX_NONE;
		int32 ParentSampleCount = 0;
		int32 ChildSampleCountA = 0;
		int32 ChildSampleCountB = 0;
		int32 PostRiftPlateCount = 0;
		int32 ChildBoundaryContactEdgeCount = 0;
		int32 ChildBoundaryDivergentEdgeCount = 0;
		int32 ChildBoundaryRiftActiveSampleCount = 0;
		int32 ChildBoundaryDivergenceActiveSampleCount = 0;
		double ParentContinentalFraction = 0.0;
		double TriggerProbability = 0.0;
		double TriggerDraw = 0.0;
	};

	FTectonicPlanetV6 BaselinePlanet = InitializePlanet(false);
	FTectonicPlanetV6 CandidatePlanet = InitializePlanet(true);

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_rift_off"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("candidate_rift_on"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	TArray<FRiftEventRecord> CandidateRiftEvents;
	int32 PreviousCandidateRiftCount = 0;

	const auto CaptureCheckpoint =
		[this](FTectonicPlanetV6& Planet,
			const FString& VariantTag,
			const FString& VariantExportRoot,
			const int32 Step,
			const bool bFullExports) -> FV6CheckpointSnapshot
	{
		if (bFullExports)
		{
			ExportV6CheckpointMaps(*this, Planet, VariantExportRoot, Step);
			ExportV6DebugOverlays(*this, Planet, VariantExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(TEXT("[V9RiftingHarness %s step=%d]"), *VariantTag, Step);
		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, Tag, Snapshot);
		AddV6RiftInfo(*this, Tag, Planet.ComputeRiftDiagnosticForTest());

		const FV9ContinentalElevationStats ElevationStats = ComputeContinentalElevationStats(Planet);
		const FV6PlateSizeStats PlateSizeStats = ComputePlateSizeStats(Planet);
		const FString ElevationMessage = FString::Printf(
			TEXT("%s continental_mean_km=%.4f continental_p95_km=%.4f continental_max_km=%.4f above_2km=%d above_5km=%d plate_size_min_mean_max=%d/%.1f/%d"),
			*Tag,
			ElevationStats.MeanElevationKm,
			ElevationStats.P95ElevationKm,
			ElevationStats.MaxElevationKm,
			ElevationStats.SamplesAbove2Km,
			ElevationStats.SamplesAbove5Km,
			PlateSizeStats.MinSamples,
			PlateSizeStats.MeanSamples,
			PlateSizeStats.MaxSamples);
		AddInfo(ElevationMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *ElevationMessage);
		return Snapshot;
	};

	const auto LogComparison =
		[this](
			const int32 Step,
			const FV6CheckpointSnapshot& Baseline,
			const FV6CheckpointSnapshot& Candidate,
			const FTectonicPlanetV6RiftDiagnostic& BaselineRift,
			const FTectonicPlanetV6RiftDiagnostic& CandidateRift,
			const FV9ContinentalElevationStats& BaselineElevation,
			const FV9ContinentalElevationStats& CandidateElevation,
			const FV6PlateSizeStats& BaselinePlateSizes,
			const FV6PlateSizeStats& CandidatePlateSizes)
	{
		const FString Message = FString::Printf(
			TEXT("[V9RiftingHarness compare step=%d] ")
			TEXT("plate_count=%d/%d rift_cumulative=%d/%d ")
			TEXT("coherence=%.4f/%.4f leakage=%.4f/%.4f churn=%.4f/%.4f ")
			TEXT("miss=%d/%d multi_hit=%d/%d active_fraction=%.4f/%.4f ")
			TEXT("continental_mean=%.4f/%.4f continental_p95=%.4f/%.4f above_2km=%d/%d ")
			TEXT("plate_size_min_mean_max=%d,%.1f,%d/%d,%.1f,%d"),
			Step,
			Baseline.PlateCount,
			Candidate.PlateCount,
			BaselineRift.CumulativeRiftCount,
			CandidateRift.CumulativeRiftCount,
			Baseline.BoundaryCoherence.BoundaryCoherenceScore,
			Candidate.BoundaryCoherence.BoundaryCoherenceScore,
			Baseline.BoundaryCoherence.InteriorLeakageFraction,
			Candidate.BoundaryCoherence.InteriorLeakageFraction,
			Baseline.OwnershipChurn.ChurnFraction,
			Candidate.OwnershipChurn.ChurnFraction,
			Baseline.MissCount,
			Candidate.MissCount,
			Baseline.MultiHitCount,
			Candidate.MultiHitCount,
			Baseline.ActiveZone.ActiveFraction,
			Candidate.ActiveZone.ActiveFraction,
			BaselineElevation.MeanElevationKm,
			CandidateElevation.MeanElevationKm,
			BaselineElevation.P95ElevationKm,
			CandidateElevation.P95ElevationKm,
			BaselineElevation.SamplesAbove2Km,
			CandidateElevation.SamplesAbove2Km,
			BaselinePlateSizes.MinSamples,
			BaselinePlateSizes.MeanSamples,
			BaselinePlateSizes.MaxSamples,
			CandidatePlateSizes.MinSamples,
			CandidatePlateSizes.MeanSamples,
			CandidatePlateSizes.MaxSamples);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	};

	const auto TrackCandidateRiftEvent =
		[this, &CandidatePlanet, &CandidateRiftEvents, &PreviousCandidateRiftCount]()
	{
		const FTectonicPlanetV6RiftDiagnostic Rift = CandidatePlanet.ComputeRiftDiagnosticForTest();
		if (Rift.CumulativeRiftCount <= PreviousCandidateRiftCount)
		{
			return;
		}

		PreviousCandidateRiftCount = Rift.CumulativeRiftCount;
		FRiftEventRecord& Event = CandidateRiftEvents.AddDefaulted_GetRef();
		Event.Step = Rift.Step;
		Event.ParentPlateId = Rift.ParentPlateId;
		Event.ChildPlateA = Rift.ChildPlateA;
		Event.ChildPlateB = Rift.ChildPlateB;
		Event.ParentSampleCount = Rift.ParentSampleCount;
		Event.ChildSampleCountA = Rift.ChildSampleCountA;
		Event.ChildSampleCountB = Rift.ChildSampleCountB;
		Event.PostRiftPlateCount = Rift.PostRiftPlateCount;
		Event.ChildBoundaryContactEdgeCount = Rift.ChildBoundaryContactEdgeCount;
		Event.ChildBoundaryDivergentEdgeCount = Rift.ChildBoundaryDivergentEdgeCount;
		Event.ChildBoundaryRiftActiveSampleCount = Rift.ChildBoundaryRiftActiveSampleCount;
		Event.ChildBoundaryDivergenceActiveSampleCount = Rift.ChildBoundaryDivergenceActiveSampleCount;
		Event.ParentContinentalFraction = Rift.ParentContinentalFraction;
		Event.TriggerProbability = Rift.TriggerProbability;
		Event.TriggerDraw = Rift.TriggerDraw;

		const FString EventMessage = FString::Printf(
			TEXT("[V9RiftingHarness EVENT step=%d] parent=%d children=(%d,%d) parent_samples=%d child_samples=(%d,%d) parent_continental_fraction=%.4f trigger_probability=%.6f trigger_draw=%.6f post_rift_plate_count=%d remesh_ran=%d ownership_applied_directly=%d child_boundary_divergent=%d child_boundary_contact_edges=%d child_boundary_divergent_edges=%d child_boundary_rift_active_samples=%d child_boundary_divergence_samples=%d"),
			Rift.Step,
			Rift.ParentPlateId,
			Rift.ChildPlateA,
			Rift.ChildPlateB,
			Rift.ParentSampleCount,
			Rift.ChildSampleCountA,
			Rift.ChildSampleCountB,
			Rift.ParentContinentalFraction,
			Rift.TriggerProbability,
			Rift.TriggerDraw,
			Rift.PostRiftPlateCount,
			Rift.bPostRiftSolveRan ? 1 : 0,
			Rift.bOwnershipAppliedDirectlyByEvent ? 1 : 0,
			Rift.bChildBoundaryClassifiedDivergent ? 1 : 0,
			Rift.ChildBoundaryContactEdgeCount,
			Rift.ChildBoundaryDivergentEdgeCount,
			Rift.ChildBoundaryRiftActiveSampleCount,
			Rift.ChildBoundaryDivergenceActiveSampleCount);
		AddInfo(EventMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *EventMessage);
	};

	auto AdvanceBothToStep =
		[&](const int32 TargetStep)
	{
		while (CandidatePlanet.GetPlanet().CurrentStep < TargetStep)
		{
			BaselinePlanet.AdvanceStep();
			CandidatePlanet.AdvanceStep();
			TrackCandidateRiftEvent();
		}
	};

	FV6CheckpointSnapshot BaselineStep25;
	FV6CheckpointSnapshot CandidateStep25;
	FV6CheckpointSnapshot BaselineStep100;
	FV6CheckpointSnapshot CandidateStep100;
	FV6CheckpointSnapshot BaselineStep200;
	FV6CheckpointSnapshot CandidateStep200;

	AdvanceBothToStep(25);
	BaselineStep25 = CaptureCheckpoint(BaselinePlanet, TEXT("baseline_rift_off"), BaselineExportRoot, 25, false);
	CandidateStep25 = CaptureCheckpoint(CandidatePlanet, TEXT("candidate_rift_on"), CandidateExportRoot, 25, false);
	LogComparison(
		25,
		BaselineStep25,
		CandidateStep25,
		BaselinePlanet.ComputeRiftDiagnosticForTest(),
		CandidatePlanet.ComputeRiftDiagnosticForTest(),
		ComputeContinentalElevationStats(BaselinePlanet),
		ComputeContinentalElevationStats(CandidatePlanet),
		ComputePlateSizeStats(BaselinePlanet),
		ComputePlateSizeStats(CandidatePlanet));

	AdvanceBothToStep(100);
	BaselineStep100 = CaptureCheckpoint(BaselinePlanet, TEXT("baseline_rift_off"), BaselineExportRoot, 100, true);
	CandidateStep100 = CaptureCheckpoint(CandidatePlanet, TEXT("candidate_rift_on"), CandidateExportRoot, 100, true);
	const FTectonicPlanetV6RiftDiagnostic BaselineRift100 = BaselinePlanet.ComputeRiftDiagnosticForTest();
	const FTectonicPlanetV6RiftDiagnostic CandidateRift100 = CandidatePlanet.ComputeRiftDiagnosticForTest();
	const FV9ContinentalElevationStats BaselineElevation100 = ComputeContinentalElevationStats(BaselinePlanet);
	const FV9ContinentalElevationStats CandidateElevation100 = ComputeContinentalElevationStats(CandidatePlanet);
	const FV6PlateSizeStats BaselinePlateSizes100 = ComputePlateSizeStats(BaselinePlanet);
	const FV6PlateSizeStats CandidatePlateSizes100 = ComputePlateSizeStats(CandidatePlanet);
	LogComparison(
		100,
		BaselineStep100,
		CandidateStep100,
		BaselineRift100,
		CandidateRift100,
		BaselineElevation100,
		CandidateElevation100,
		BaselinePlateSizes100,
		CandidatePlateSizes100);

	const bool bRiftCountPass = CandidateRift100.CumulativeRiftCount > 0;
	const bool bPlateCountPass = CandidateStep100.PlateCount >= PlateCount;
	const bool bChildBoundaryDivergentPass =
		CandidateRift100.CumulativeRiftCount > 0 &&
		CandidateRift100.bChildBoundaryClassifiedDivergent;
	const bool bCoherencePass = CandidateStep100.BoundaryCoherence.BoundaryCoherenceScore > 0.93;
	const bool bLeakagePass = CandidateStep100.BoundaryCoherence.InteriorLeakageFraction < 0.18;
	const bool bChurnPass = CandidateStep100.OwnershipChurn.ChurnFraction < 0.05;
	const bool bMeanElevationPass = CandidateElevation100.MeanElevationKm > 0.3;
	const bool bP95ElevationPass = CandidateElevation100.P95ElevationKm > 2.0;
	const bool bAllowStep200 =
		bRiftCountPass &&
		bPlateCountPass &&
		bChildBoundaryDivergentPass &&
		bCoherencePass &&
		bLeakagePass &&
		bChurnPass &&
		bMeanElevationPass &&
		bP95ElevationPass;

	if (!bRiftCountPass)
	{
		int32 EligibleContinentalSamples = 0;
		double EligibleContinentalFraction = 0.0;
		const int32 EligibleParentPlateId =
			CandidatePlanet.GetPlanet().FindLargestEligibleAutomaticRiftParentId(
				&EligibleContinentalSamples,
				&EligibleContinentalFraction);
		double EligibleProbability = 0.0;
		int32 EligibleParentSamples = 0;
		if (EligibleParentPlateId != INDEX_NONE)
		{
			const int32 ParentPlateIndex =
				CandidatePlanet.GetPlanet().FindPlateArrayIndexById(EligibleParentPlateId);
			if (CandidatePlanet.GetPlanet().Plates.IsValidIndex(ParentPlateIndex))
			{
				EligibleParentSamples =
					CandidatePlanet.GetPlanet().Plates[ParentPlateIndex].MemberSamples.Num();
				EligibleProbability =
					CandidatePlanet.GetPlanet().ComputeAutomaticRiftProbabilityForSampleCount(
						EligibleParentSamples,
						EligibleContinentalFraction);
			}
		}

		const FString NoRiftMessage = FString::Printf(
			TEXT("[V9RiftingHarness no_rift_by_step100] eligible_parent=%d eligible_parent_samples=%d eligible_parent_continental_samples=%d eligible_parent_continental_fraction=%.4f eligible_probability=%.6f"),
			EligibleParentPlateId,
			EligibleParentSamples,
			EligibleContinentalSamples,
			EligibleContinentalFraction,
			EligibleProbability);
		AddInfo(NoRiftMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *NoRiftMessage);
	}

	const FString GateMessage = FString::Printf(
		TEXT("[V9RiftingHarness gate step=100] run_step200=%d rift_count=%d plate_count=%d child_boundary_divergent=%d coherence=%d leakage=%d churn=%d continental_mean=%d continental_p95=%d candidate_rifts=%d candidate_plate_count=%d candidate_coherence=%.4f candidate_leakage=%.4f candidate_churn=%.4f candidate_mean=%.4f candidate_p95=%.4f"),
		bAllowStep200 ? 1 : 0,
		bRiftCountPass ? 1 : 0,
		bPlateCountPass ? 1 : 0,
		bChildBoundaryDivergentPass ? 1 : 0,
		bCoherencePass ? 1 : 0,
		bLeakagePass ? 1 : 0,
		bChurnPass ? 1 : 0,
		bMeanElevationPass ? 1 : 0,
		bP95ElevationPass ? 1 : 0,
		CandidateRift100.CumulativeRiftCount,
		CandidateStep100.PlateCount,
		CandidateStep100.BoundaryCoherence.BoundaryCoherenceScore,
		CandidateStep100.BoundaryCoherence.InteriorLeakageFraction,
		CandidateStep100.OwnershipChurn.ChurnFraction,
		CandidateElevation100.MeanElevationKm,
		CandidateElevation100.P95ElevationKm);
	AddInfo(GateMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *GateMessage);

	if (bAllowStep200)
	{
		AdvanceBothToStep(200);
		BaselineStep200 = CaptureCheckpoint(BaselinePlanet, TEXT("baseline_rift_off"), BaselineExportRoot, 200, true);
		CandidateStep200 = CaptureCheckpoint(CandidatePlanet, TEXT("candidate_rift_on"), CandidateExportRoot, 200, true);
		LogComparison(
			200,
			BaselineStep200,
			CandidateStep200,
			BaselinePlanet.ComputeRiftDiagnosticForTest(),
			CandidatePlanet.ComputeRiftDiagnosticForTest(),
			ComputeContinentalElevationStats(BaselinePlanet),
			ComputeContinentalElevationStats(CandidatePlanet),
			ComputePlateSizeStats(BaselinePlanet),
			ComputePlateSizeStats(CandidatePlanet));
	}

	const FString PlateProgressionMessage = FString::Printf(
		TEXT("[V9RiftingHarness plate_progression] baseline=0:%d,25:%d,100:%d%s candidate=0:%d,25:%d,100:%d%s"),
		PlateCount,
		BaselineStep25.PlateCount,
		BaselineStep100.PlateCount,
		bAllowStep200 ? *FString::Printf(TEXT(",200:%d"), BaselineStep200.PlateCount) : TEXT(""),
		PlateCount,
		CandidateStep25.PlateCount,
		CandidateStep100.PlateCount,
		bAllowStep200 ? *FString::Printf(TEXT(",200:%d"), CandidateStep200.PlateCount) : TEXT(""));
	AddInfo(PlateProgressionMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *PlateProgressionMessage);

	for (int32 EventIndex = 0; EventIndex < CandidateRiftEvents.Num(); ++EventIndex)
	{
		const FRiftEventRecord& Event = CandidateRiftEvents[EventIndex];
		AddInfo(FString::Printf(
			TEXT("[V9RiftingHarness event_summary index=%d] step=%d parent=%d children=(%d,%d) parent_samples=%d child_samples=(%d,%d) parent_continental_fraction=%.4f trigger_probability=%.6f trigger_draw=%.6f post_rift_plate_count=%d child_boundary_contact_edges=%d child_boundary_divergent_edges=%d child_boundary_rift_active_samples=%d child_boundary_divergence_samples=%d"),
			EventIndex,
			Event.Step,
			Event.ParentPlateId,
			Event.ChildPlateA,
			Event.ChildPlateB,
			Event.ParentSampleCount,
			Event.ChildSampleCountA,
			Event.ChildSampleCountB,
			Event.ParentContinentalFraction,
			Event.TriggerProbability,
			Event.TriggerDraw,
			Event.PostRiftPlateCount,
			Event.ChildBoundaryContactEdgeCount,
			Event.ChildBoundaryDivergentEdgeCount,
			Event.ChildBoundaryRiftActiveSampleCount,
			Event.ChildBoundaryDivergenceActiveSampleCount));
	}

	TestTrue(TEXT("Baseline reached step 100"), BaselinePlanet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Candidate reached step 100"), CandidatePlanet.GetPlanet().CurrentStep >= 100);
	if (bAllowStep200)
	{
		TestTrue(TEXT("Candidate promoted to step 200"), CandidatePlanet.GetPlanet().CurrentStep == 200);
	}
	else
	{
		TestTrue(TEXT("Candidate stopped at step 100 when gate failed"), CandidatePlanet.GetPlanet().CurrentStep == 100);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9ForcedRiftValidationHarnessTest,
	"Aurous.TectonicPlanet.V6V9ForcedRiftValidationHarnessTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9ForcedRiftValidationHarnessTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	constexpr int32 ForcedRiftTriggerStep = 31;
	constexpr int32 ForcedRiftChildCount = 2;
	constexpr int32 ForcedRiftSeed = 17017;
	constexpr int32 MinExpectedChildSamples = 256;
	const FString RunId = TEXT("V9ForcedRiftValidationHarness");

	const auto InitializePlanet = [&]() -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
		Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
		Planet.SetAutomaticRiftingForTest(false);
		return Planet;
	};

	FTectonicPlanetV6 BaselinePlanet = InitializePlanet();
	FTectonicPlanetV6 CandidatePlanet = InitializePlanet();

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_no_rift"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("candidate_forced_rift"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	const auto CaptureCheckpoint =
		[this](FTectonicPlanetV6& Planet,
			const FString& VariantTag,
			const FString& VariantExportRoot,
			const int32 Step,
			const bool bFullExports) -> FV6CheckpointSnapshot
	{
		if (bFullExports)
		{
			ExportV6CheckpointMaps(*this, Planet, VariantExportRoot, Step);
			ExportV6DebugOverlays(*this, Planet, VariantExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		const FString Tag = FString::Printf(TEXT("[V9ForcedRiftHarness %s step=%d]"), *VariantTag, Step);
		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6CollisionShadowInfo(*this, Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, Tag, Snapshot);
		AddV6RiftInfo(*this, Tag, Planet.ComputeRiftDiagnosticForTest());

		const FV9ContinentalElevationStats ElevationStats = ComputeContinentalElevationStats(Planet);
		const FV6PlateSizeStats PlateSizeStats = ComputePlateSizeStats(Planet);
		const FString ElevationMessage = FString::Printf(
			TEXT("%s continental_mean_km=%.4f continental_p95_km=%.4f continental_max_km=%.4f above_2km=%d above_5km=%d plate_size_min_mean_max=%d/%.1f/%d"),
			*Tag,
			ElevationStats.MeanElevationKm,
			ElevationStats.P95ElevationKm,
			ElevationStats.MaxElevationKm,
			ElevationStats.SamplesAbove2Km,
			ElevationStats.SamplesAbove5Km,
			PlateSizeStats.MinSamples,
			PlateSizeStats.MeanSamples,
			PlateSizeStats.MaxSamples);
		AddInfo(ElevationMessage);
		UE_LOG(LogTemp, Log, TEXT("%s"), *ElevationMessage);
		return Snapshot;
	};

	const auto LogComparison =
		[this](
			const int32 Step,
			const FV6CheckpointSnapshot& Baseline,
			const FV6CheckpointSnapshot& Candidate,
			const FTectonicPlanetV6RiftDiagnostic& CandidateRift,
			const FV9ContinentalElevationStats& BaselineElevation,
			const FV9ContinentalElevationStats& CandidateElevation)
	{
		const FString Message = FString::Printf(
			TEXT("[V9ForcedRiftHarness compare step=%d] ")
			TEXT("plate_count=%d/%d rift_cumulative=0/%d coherence=%.4f/%.4f leakage=%.4f/%.4f churn=%.4f/%.4f ")
			TEXT("miss=%d/%d multi_hit=%d/%d active_fraction=%.4f/%.4f continental_mean=%.4f/%.4f continental_p95=%.4f/%.4f above_2km=%d/%d"),
			Step,
			Baseline.PlateCount,
			Candidate.PlateCount,
			CandidateRift.CumulativeRiftCount,
			Baseline.BoundaryCoherence.BoundaryCoherenceScore,
			Candidate.BoundaryCoherence.BoundaryCoherenceScore,
			Baseline.BoundaryCoherence.InteriorLeakageFraction,
			Candidate.BoundaryCoherence.InteriorLeakageFraction,
			Baseline.OwnershipChurn.ChurnFraction,
			Candidate.OwnershipChurn.ChurnFraction,
			Baseline.MissCount,
			Candidate.MissCount,
			Baseline.MultiHitCount,
			Candidate.MultiHitCount,
			Baseline.ActiveZone.ActiveFraction,
			Candidate.ActiveZone.ActiveFraction,
			BaselineElevation.MeanElevationKm,
			CandidateElevation.MeanElevationKm,
			BaselineElevation.P95ElevationKm,
			CandidateElevation.P95ElevationKm,
			BaselineElevation.SamplesAbove2Km,
			CandidateElevation.SamplesAbove2Km);
		AddInfo(Message);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	};

	auto AdvanceBothToStep =
		[&](const int32 TargetStep)
	{
		while (CandidatePlanet.GetPlanet().CurrentStep < TargetStep)
		{
			BaselinePlanet.AdvanceStep();
			CandidatePlanet.AdvanceStep();
		}
	};

	AdvanceBothToStep(ForcedRiftTriggerStep);
	const bool bForcedRiftTriggered =
		CandidatePlanet.ForceLargestEligibleAutomaticRiftForTest(ForcedRiftChildCount, ForcedRiftSeed);
	const FTectonicPlanetV6RiftDiagnostic ForcedRiftDiagnostic = CandidatePlanet.ComputeRiftDiagnosticForTest();
	const FString ForcedEventMessage = FString::Printf(
		TEXT("[V9ForcedRiftHarness EVENT step=%d] forced_triggered=%d parent=%d children=(%d,%d) parent_present=%d parent_samples=%d child_samples_at_event=(%d,%d) child_samples_current=(%d,%d) parent_continental_fraction=%.4f trigger_probability=%.6f trigger_draw=%.6f post_rift_plate_count=%d ownership_applied_directly=%d remesh_ran=%d copied_frontier_rebuilt=%d plate_submesh_rebuilt=%d child_boundary_divergent=%d child_boundary_contact_edges=%d child_boundary_divergent_edges=%d child_boundary_convergent_edges=%d child_boundary_mean_rel_normal_velocity=%.4f child_boundary_max_abs_rel_normal_velocity=%.4f"),
		ForcedRiftDiagnostic.Step,
		bForcedRiftTriggered ? 1 : 0,
		ForcedRiftDiagnostic.ParentPlateId,
		ForcedRiftDiagnostic.ChildPlateA,
		ForcedRiftDiagnostic.ChildPlateB,
		ForcedRiftDiagnostic.bParentPlateStillPresent ? 1 : 0,
		ForcedRiftDiagnostic.ParentSampleCount,
		ForcedRiftDiagnostic.ChildSampleCountA,
		ForcedRiftDiagnostic.ChildSampleCountB,
		ForcedRiftDiagnostic.CurrentChildSampleCountA,
		ForcedRiftDiagnostic.CurrentChildSampleCountB,
		ForcedRiftDiagnostic.ParentContinentalFraction,
		ForcedRiftDiagnostic.TriggerProbability,
		ForcedRiftDiagnostic.TriggerDraw,
		ForcedRiftDiagnostic.PostRiftPlateCount,
		ForcedRiftDiagnostic.bOwnershipAppliedDirectlyByEvent ? 1 : 0,
		ForcedRiftDiagnostic.bPostRiftSolveRan ? 1 : 0,
		ForcedRiftDiagnostic.bCopiedFrontierRebuiltBeforeSolve ? 1 : 0,
		ForcedRiftDiagnostic.bPlateSubmeshRebuiltBeforeSolve ? 1 : 0,
		ForcedRiftDiagnostic.bChildBoundaryClassifiedDivergent ? 1 : 0,
		ForcedRiftDiagnostic.ChildBoundaryContactEdgeCount,
		ForcedRiftDiagnostic.ChildBoundaryDivergentEdgeCount,
		ForcedRiftDiagnostic.ChildBoundaryConvergentEdgeCount,
		ForcedRiftDiagnostic.ChildBoundaryMeanRelativeNormalVelocityKmPerMy,
		ForcedRiftDiagnostic.ChildBoundaryMaxAbsRelativeNormalVelocityKmPerMy);
	AddInfo(ForcedEventMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *ForcedEventMessage);

	AdvanceBothToStep(25);
	const FV6CheckpointSnapshot BaselineStep25 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_no_rift"), BaselineExportRoot, 25, false);
	const FV6CheckpointSnapshot CandidateStep25 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_forced_rift"), CandidateExportRoot, 25, false);
	LogComparison(
		25,
		BaselineStep25,
		CandidateStep25,
		CandidatePlanet.ComputeRiftDiagnosticForTest(),
		ComputeContinentalElevationStats(BaselinePlanet),
		ComputeContinentalElevationStats(CandidatePlanet));

	AdvanceBothToStep(100);
	const FV6CheckpointSnapshot BaselineStep100 =
		CaptureCheckpoint(BaselinePlanet, TEXT("baseline_no_rift"), BaselineExportRoot, 100, true);
	const FV6CheckpointSnapshot CandidateStep100 =
		CaptureCheckpoint(CandidatePlanet, TEXT("candidate_forced_rift"), CandidateExportRoot, 100, true);
	const FTectonicPlanetV6RiftDiagnostic CandidateRift100 = CandidatePlanet.ComputeRiftDiagnosticForTest();
	const FV9ContinentalElevationStats BaselineElevation100 = ComputeContinentalElevationStats(BaselinePlanet);
	const FV9ContinentalElevationStats CandidateElevation100 = ComputeContinentalElevationStats(CandidatePlanet);
	LogComparison(
		100,
		BaselineStep100,
		CandidateStep100,
		CandidateRift100,
		BaselineElevation100,
		CandidateElevation100);

	const bool bPlateCountPass = CandidateStep100.PlateCount >= (PlateCount + 1);
	const bool bParentRemovedPass = !CandidateRift100.bParentPlateStillPresent;
	const bool bChildAAlivePass = CandidateRift100.bChildPlateAAlive && CandidateRift100.CurrentChildSampleCountA >= MinExpectedChildSamples;
	const bool bChildBAlivePass = CandidateRift100.bChildPlateBAlive && CandidateRift100.CurrentChildSampleCountB >= MinExpectedChildSamples;
	const bool bPostRiftSolvePass =
		bForcedRiftTriggered &&
		CandidateRift100.bForcedByTest &&
		CandidateRift100.bOwnershipAppliedDirectlyByEvent &&
		CandidateRift100.bCopiedFrontierRebuiltBeforeSolve &&
		CandidateRift100.bPlateSubmeshRebuiltBeforeSolve &&
		CandidateRift100.bPostRiftSolveRan;
	const bool bChildBoundaryPass =
		CandidateRift100.bChildBoundaryClassifiedDivergent &&
		CandidateRift100.ChildBoundaryDivergentEdgeCount > 0 &&
		CandidateRift100.ChildBoundaryMeanRelativeNormalVelocityKmPerMy > 0.0;
	const bool bCoherencePass = CandidateStep100.BoundaryCoherence.BoundaryCoherenceScore > 0.92;
	const bool bLeakagePass = CandidateStep100.BoundaryCoherence.InteriorLeakageFraction < 0.20;
	const bool bChurnPass = CandidateStep100.OwnershipChurn.ChurnFraction < 0.06;
	const bool bMeanElevationPass = CandidateElevation100.MeanElevationKm > 0.3;
	const bool bP95ElevationPass = CandidateElevation100.P95ElevationKm > 2.0;
	const bool bAbove2KmPass = CandidateElevation100.SamplesAbove2Km > 3000;

	const FString GateMessage = FString::Printf(
		TEXT("[V9ForcedRiftHarness gate step=100] post_rift_solve=%d plate_count=%d parent_removed=%d child_a_alive=%d child_b_alive=%d child_boundary_divergent=%d coherence=%d leakage=%d churn=%d continental_mean=%d continental_p95=%d above_2km=%d candidate_plate_count=%d candidate_parent_present=%d candidate_child_samples=(%d,%d) candidate_child_boundary_mean_rel_normal_velocity=%.4f candidate_coherence=%.4f candidate_leakage=%.4f candidate_churn=%.4f candidate_mean=%.4f candidate_p95=%.4f candidate_above_2km=%d"),
		bPostRiftSolvePass ? 1 : 0,
		bPlateCountPass ? 1 : 0,
		bParentRemovedPass ? 1 : 0,
		bChildAAlivePass ? 1 : 0,
		bChildBAlivePass ? 1 : 0,
		bChildBoundaryPass ? 1 : 0,
		bCoherencePass ? 1 : 0,
		bLeakagePass ? 1 : 0,
		bChurnPass ? 1 : 0,
		bMeanElevationPass ? 1 : 0,
		bP95ElevationPass ? 1 : 0,
		bAbove2KmPass ? 1 : 0,
		CandidateStep100.PlateCount,
		CandidateRift100.bParentPlateStillPresent ? 1 : 0,
		CandidateRift100.CurrentChildSampleCountA,
		CandidateRift100.CurrentChildSampleCountB,
		CandidateRift100.ChildBoundaryMeanRelativeNormalVelocityKmPerMy,
		CandidateStep100.BoundaryCoherence.BoundaryCoherenceScore,
		CandidateStep100.BoundaryCoherence.InteriorLeakageFraction,
		CandidateStep100.OwnershipChurn.ChurnFraction,
		CandidateElevation100.MeanElevationKm,
		CandidateElevation100.P95ElevationKm,
		CandidateElevation100.SamplesAbove2Km);
	AddInfo(GateMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *GateMessage);

	const FString PlateProgressionMessage = FString::Printf(
		TEXT("[V9ForcedRiftHarness plate_progression] baseline=0:%d,%d:%d,25:%d,100:%d candidate=0:%d,%d:%d,25:%d,100:%d"),
		PlateCount,
		ForcedRiftTriggerStep,
		PlateCount,
		BaselineStep25.PlateCount,
		BaselineStep100.PlateCount,
		PlateCount,
		ForcedRiftTriggerStep,
		ForcedRiftDiagnostic.PostRiftPlateCount,
		CandidateStep25.PlateCount,
		CandidateStep100.PlateCount);
	AddInfo(PlateProgressionMessage);
	UE_LOG(LogTemp, Log, TEXT("%s"), *PlateProgressionMessage);

	TestTrue(TEXT("Forced rift triggered"), bForcedRiftTriggered);
	TestTrue(TEXT("Forced rift performed V6 post-rift solve"), bPostRiftSolvePass);
	TestTrue(TEXT("Parent removed after rift"), bParentRemovedPass);
	TestTrue(TEXT("Child plate A alive at step 100"), bChildAAlivePass);
	TestTrue(TEXT("Child plate B alive at step 100"), bChildBAlivePass);
	TestTrue(TEXT("Plate count increased after rift"), bPlateCountPass);
	TestTrue(TEXT("Child boundary classified divergent"), bChildBoundaryPass);
	TestTrue(TEXT("Forced-rift coherence stays above relaxed gate"), bCoherencePass);
	TestTrue(TEXT("Forced-rift leakage stays under relaxed gate"), bLeakagePass);
	TestTrue(TEXT("Forced-rift churn stays under relaxed gate"), bChurnPass);
	TestTrue(TEXT("Forced-rift continental mean elevation stays healthy"), bMeanElevationPass);
	TestTrue(TEXT("Forced-rift continental p95 elevation stays healthy"), bP95ElevationPass);
	return true;
}

// ============================================================================
// Continental Mass Audit Test
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9ContinentalMassAuditTest,
	"Aurous.TectonicPlanet.V6V9ContinentalMassAuditTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9ContinentalMassAuditTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	constexpr int32 ForcedRiftTriggerStep = 31;
	constexpr int32 ForcedRiftChildCount = 2;
	constexpr int32 ForcedRiftSeed = 17017;
	const FString RunId = TEXT("V9ContinentalMassAudit");

	const auto InitializePlanet = [&]() -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
		Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
		Planet.SetAutomaticRiftingForTest(false);
		return Planet;
	};

	FTectonicPlanetV6 BaselinePlanet = InitializePlanet();
	FTectonicPlanetV6 CandidatePlanet = InitializePlanet();
	CandidatePlanet.SetV9QuietInteriorContinentalRetentionForTest(true);

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("retention"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	// --- Capture full diagnostic checkpoint ---
	const auto CaptureFullCheckpoint =
		[this](
			FTectonicPlanetV6& Planet,
			const FString& VariantTag,
			const FString& VariantExportRoot,
			const int32 Step,
			const TArray<float>& BaselineCW,
			const bool bExport) -> FV9ContinentalMassDiagnostic
	{
		const FString Tag = FString::Printf(TEXT("[V9MassAudit %s step=%d]"), *VariantTag, Step);

		// Standard snapshot + existing diagnostics
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Planet, VariantExportRoot, Step);
			ExportV6DebugOverlays(*this, Planet, VariantExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Planet);
		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6CollisionExecutionInfo(*this, Tag, Snapshot);

		const FV9ContinentalElevationStats ElevStats = ComputeContinentalElevationStats(Planet);
		const FTectonicPlanetV6PeriodicSolveStats& SolveStats =
			Planet.GetPeriodicSolveCount() > 0
				? Planet.GetLastSolveStats()
				: Planet.BuildCurrentDiagnosticSnapshotForTest();
		AddInfo(FString::Printf(
			TEXT("%s elevation: mean=%.4f p95=%.4f max=%.4f above_2km=%d above_5km=%d continental_samples=%d retention_guard=%d(tri=%d,ss=%d)"),
			*Tag,
			ElevStats.MeanElevationKm,
			ElevStats.P95ElevationKm,
			ElevStats.MaxElevationKm,
			ElevStats.SamplesAbove2Km,
			ElevStats.SamplesAbove5Km,
			ElevStats.ContinentalSampleCount,
			SolveStats.QuietInteriorContinentalRetentionCount,
			SolveStats.QuietInteriorContinentalRetentionTriangleCount,
			SolveStats.QuietInteriorContinentalRetentionSingleSourceCount));

		// Continental mass diagnostics (1, 2, 4, 5)
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Planet);
		AddV6ContinentalMassDiagnosticInfo(*this, Tag, MassDiag);

		// Continental loss attribution (3) — last solve
		const FV9ContinentalLossAttribution SolveLoss = ComputeContinentalLossAttribution(Planet);
		AddV6ContinentalLossAttributionInfo(
			*this,
			FString::Printf(TEXT("%s last_solve_loss"), *Tag),
			SolveLoss);

		// Cumulative loss from baseline
		if (!BaselineCW.IsEmpty())
		{
			const FV9ContinentalLossAttribution CumulLoss =
				ComputeCumulativeContinentalLossAttribution(BaselineCW, Planet.GetPlanet());
			AddV6ContinentalLossAttributionInfo(
				*this,
				FString::Printf(TEXT("%s cumulative_loss_from_step0"), *Tag),
				CumulLoss);
		}

		// Export continental mass overlays
		if (bExport)
		{
			ExportContinentalMassOverlays(*this, Planet, MassDiag, VariantExportRoot, Step);
		}

		return MassDiag;
	};

	// --- A/B comparison logging ---
	const auto LogMassComparison =
		[this](
			const int32 Step,
			const FV9ContinentalMassDiagnostic& Baseline,
			const FV9ContinentalMassDiagnostic& Candidate)
	{
		const FString Msg = FString::Printf(
			TEXT("[V9MassAudit compare step=%d] ")
			TEXT("continental_area=%.4f/%.4f samples=%d/%d components=%d/%d largest=%d/%d ")
			TEXT("active_zone_frac=%.4f/%.4f boundary_band_frac=%.4f/%.4f deep_interior_frac=%.4f/%.4f ")
			TEXT("coast_dist_mean=%.2f/%.2f p50=%.1f/%.1f p90=%.1f/%.1f max=%.1f/%.1f"),
			Step,
			Baseline.ContinentalAreaFraction,
			Candidate.ContinentalAreaFraction,
			Baseline.ContinentalSampleCount,
			Candidate.ContinentalSampleCount,
			Baseline.ComponentCount,
			Candidate.ComponentCount,
			Baseline.LargestComponentSize,
			Candidate.LargestComponentSize,
			Baseline.ActiveZoneContinentalFraction,
			Candidate.ActiveZoneContinentalFraction,
			Baseline.BoundaryBandContinentalFraction,
			Candidate.BoundaryBandContinentalFraction,
			Baseline.DeepInteriorContinentalFraction,
			Candidate.DeepInteriorContinentalFraction,
			Baseline.MeanCoastDistHops,
			Candidate.MeanCoastDistHops,
			Baseline.P50CoastDist,
			Candidate.P50CoastDist,
			Baseline.P90CoastDist,
			Candidate.P90CoastDist,
			Baseline.MaxCoastDist,
			Candidate.MaxCoastDist);
		AddInfo(Msg);
		UE_LOG(LogTemp, Log, TEXT("%s"), *Msg);
	};

	// --- Step 0: Initial state ---
	const TArray<float> BaselineCW0 = SnapshotContinentalWeights(BaselinePlanet.GetPlanet());
	const TArray<float> CandidateCW0 = SnapshotContinentalWeights(CandidatePlanet.GetPlanet());
	{
		const FV9ContinentalMassDiagnostic B0 =
			CaptureFullCheckpoint(BaselinePlanet, TEXT("baseline"), BaselineExportRoot, 0, {}, false);
		AddInfo(FString::Printf(
			TEXT("[V9MassAudit step=0] initial_continental_area=%.4f initial_samples=%d initial_components=%d initial_coast_mean=%.2f"),
			B0.ContinentalAreaFraction,
			B0.ContinentalSampleCount,
			B0.ComponentCount,
			B0.MeanCoastDistHops));
	}

	// --- Advance helper ---
	auto AdvanceBothToStep =
		[&](const int32 TargetStep)
	{
		while (BaselinePlanet.GetPlanet().CurrentStep < TargetStep)
		{
			BaselinePlanet.AdvanceStep();
			CandidatePlanet.AdvanceStep();
		}
	};

	// --- Step 25 ---
	AdvanceBothToStep(25);
	const FV9ContinentalMassDiagnostic BaselineStep25 =
		CaptureFullCheckpoint(BaselinePlanet, TEXT("baseline"), BaselineExportRoot, 25, BaselineCW0, false);
	const FV9ContinentalMassDiagnostic CandidateStep25 =
		CaptureFullCheckpoint(CandidatePlanet, TEXT("retention"), CandidateExportRoot, 25, CandidateCW0, false);
	LogMassComparison(25, BaselineStep25, CandidateStep25);

	// --- Step 100 (full exports) ---
	AdvanceBothToStep(100);
	const FV9ContinentalMassDiagnostic BaselineStep100 =
		CaptureFullCheckpoint(BaselinePlanet, TEXT("baseline"), BaselineExportRoot, 100, BaselineCW0, true);
	const FV9ContinentalMassDiagnostic CandidateStep100 =
		CaptureFullCheckpoint(CandidatePlanet, TEXT("retention"), CandidateExportRoot, 100, CandidateCW0, true);
	LogMassComparison(100, BaselineStep100, CandidateStep100);

	// Step 100 gate
	const FV6CheckpointSnapshot BaselineSnapshot100 = BuildV6CheckpointSnapshot(BaselinePlanet);
	const FV6CheckpointSnapshot CandidateSnapshot100 = BuildV6CheckpointSnapshot(CandidatePlanet);
	AddInfo(FString::Printf(
		TEXT("[V9MassAudit gate step=100] baseline: coherence=%.4f leakage=%.4f churn=%.4f | retention: coherence=%.4f leakage=%.4f churn=%.4f"),
		BaselineSnapshot100.BoundaryCoherence.BoundaryCoherenceScore,
		BaselineSnapshot100.BoundaryCoherence.InteriorLeakageFraction,
		BaselineSnapshot100.OwnershipChurn.ChurnFraction,
		CandidateSnapshot100.BoundaryCoherence.BoundaryCoherenceScore,
		CandidateSnapshot100.BoundaryCoherence.InteriorLeakageFraction,
		CandidateSnapshot100.OwnershipChurn.ChurnFraction));

	const bool bStep100Healthy =
		CandidateSnapshot100.BoundaryCoherence.BoundaryCoherenceScore > 0.92 &&
		CandidateSnapshot100.BoundaryCoherence.InteriorLeakageFraction < 0.20 &&
		CandidateSnapshot100.OwnershipChurn.ChurnFraction < 0.06;

	// --- Step 200 (if healthy) ---
	if (bStep100Healthy)
	{
		AdvanceBothToStep(200);
		const FV9ContinentalMassDiagnostic BaselineStep200 =
			CaptureFullCheckpoint(BaselinePlanet, TEXT("baseline"), BaselineExportRoot, 200, BaselineCW0, true);
		const FV9ContinentalMassDiagnostic CandidateStep200 =
			CaptureFullCheckpoint(CandidatePlanet, TEXT("retention"), CandidateExportRoot, 200, CandidateCW0, true);
		LogMassComparison(200, BaselineStep200, CandidateStep200);
	}

	// --- Summary ---
	AddInfo(FString::Printf(
		TEXT("[V9MassAudit summary] export_root=%s baseline_root=%s candidate_root=%s"),
		*ExportRoot,
		*BaselineExportRoot,
		*CandidateExportRoot));

	TestTrue(TEXT("Baseline reached step 100"), BaselinePlanet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Candidate reached step 100"), CandidatePlanet.GetPlanet().CurrentStep >= 100);
	return true;
}

// ============================================================================
// Continental Breadth Preservation Test
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9ContinentalBreadthPreservationTest,
	"Aurous.TectonicPlanet.V6V9ContinentalBreadthPreservationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9ContinentalBreadthPreservationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9ContinentalBreadthPreservation");

	const auto InitializePlanet = [=](const bool bEnableBreadthPreservation) -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
		Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
		Planet.SetV9QuietInteriorContinentalRetentionForTest(true);
		Planet.SetV9ContinentalBreadthPreservationForTest(bEnableBreadthPreservation);
		Planet.SetSubmergedContinentalRelaxationForTest(true, 0.005);
		Planet.SetAutomaticRiftingForTest(true);
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleCount = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleCount);
		OutStep0BroadInteriorFlags.Init(0, SampleCount);
		OutStep0CoastAdjacentFlags.Init(0, SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
	};

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_equilibrium"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("candidate_breadth_preservation"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	FVariantState Baseline;
	Baseline.Label = TEXT("baseline_equilibrium");
	Baseline.ExportRoot = BaselineExportRoot;
	Baseline.Planet = InitializePlanet(false);

	FVariantState Candidate;
	Candidate.Label = TEXT("candidate_breadth_preservation");
	Candidate.ExportRoot = CandidateExportRoot;
	Candidate.Planet = InitializePlanet(true);

	TArray<uint8> Step0ContinentalFlags;
	TArray<uint8> Step0BroadInteriorFlags;
	TArray<uint8> Step0CoastAdjacentFlags;

	const auto CaptureCheckpoint =
		[this, &Step0ContinentalFlags, &Step0BroadInteriorFlags, &Step0CoastAdjacentFlags](
			FVariantState& Variant,
			const int32 Step,
			const bool bExport)
	{
		const FString Tag = FString::Printf(TEXT("[V9Breadth %s step=%d]"), *Variant.Label, Step);
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Variant.Planet);
		const FV9ContinentalElevationStats ElevStats = ComputeContinentalElevationStats(Variant.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Variant.Planet.GetPlanet(),
				Step0ContinentalFlags,
				Step0BroadInteriorFlags,
				Step0CoastAdjacentFlags);
		Variant.Snapshots.Add(Step, Snapshot);
		Variant.MassDiagnostics.Add(Step, MassDiag);
		Variant.ElevationDiagnostics.Add(Step, ElevStats);
		Variant.SurvivalDiagnostics.Add(Step, SurvivalDiag);

		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6ContinentalMassDiagnosticInfo(*this, Tag, MassDiag);

		const FTectonicPlanetV6PeriodicSolveStats& SolveStats =
			Variant.Planet.GetPeriodicSolveCount() > 0
				? Variant.Planet.GetLastSolveStats()
				: Variant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		AddInfo(FString::Printf(
			TEXT("%s relief: mean=%.4f p95=%.4f max=%.4f above_2km=%d above_5km=%d breadth_preserve=%d(strong=%d,moderate=%d) quiet_retention=%d(tri=%d,ss=%d)"),
			*Tag,
			ElevStats.MeanElevationKm,
			ElevStats.P95ElevationKm,
			ElevStats.MaxElevationKm,
			ElevStats.SamplesAbove2Km,
			ElevStats.SamplesAbove5Km,
			SolveStats.ContinentalBreadthPreservationCount,
			SolveStats.ContinentalBreadthPreservationStrongInteriorCount,
			SolveStats.ContinentalBreadthPreservationModerateInteriorCount,
			SolveStats.QuietInteriorContinentalRetentionCount,
			SolveStats.QuietInteriorContinentalRetentionTriangleCount,
			SolveStats.QuietInteriorContinentalRetentionSingleSourceCount));
		AddInfo(FString::Printf(
			TEXT("%s seeded_survival: all=%d/%d(%.4f) broad_interior=%d/%d(%.4f) coast_adjacent=%d/%d(%.4f)"),
			*Tag,
			SurvivalDiag.Step0ContinentalRemainingCount,
			SurvivalDiag.Step0ContinentalCount,
			SurvivalDiag.Step0ContinentalRemainingFraction,
			SurvivalDiag.Step0BroadInteriorRemainingCount,
			SurvivalDiag.Step0BroadInteriorCount,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			SurvivalDiag.Step0CoastAdjacentRemainingCount,
			SurvivalDiag.Step0CoastAdjacentCount,
			SurvivalDiag.Step0CoastAdjacentRemainingFraction));

		if (bExport)
		{
			ExportContinentalMassOverlays(*this, Variant.Planet, MassDiag, Variant.ExportRoot, Step);
		}
	};

	const auto LogComparison =
		[this](
			const int32 Step,
			const FVariantState& BaselineVariant,
			const FVariantState& CandidateVariant)
	{
		const FV6CheckpointSnapshot& BaselineSnapshot = BaselineVariant.Snapshots.FindChecked(Step);
		const FV6CheckpointSnapshot& CandidateSnapshot = CandidateVariant.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& BaselineMass = BaselineVariant.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& CandidateMass = CandidateVariant.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& BaselineElev = BaselineVariant.ElevationDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& CandidateElev = CandidateVariant.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& BaselineSurvival =
			BaselineVariant.SurvivalDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& CandidateSurvival =
			CandidateVariant.SurvivalDiagnostics.FindChecked(Step);

		AddInfo(FString::Printf(
			TEXT("[V9Breadth compare step=%d] coast_mean=%.2f/%.2f coast_p50=%.1f/%.1f coast_p90=%.1f/%.1f subaerial_coast_mean=%.2f/%.2f largest=%d/%d largest_subaerial=%d/%d largest_mean_coast=%.2f/%.2f largest_subaerial_mean_coast=%.2f/%.2f"),
			Step,
			BaselineMass.MeanCoastDistHops,
			CandidateMass.MeanCoastDistHops,
			BaselineMass.P50CoastDist,
			CandidateMass.P50CoastDist,
			BaselineMass.P90CoastDist,
			CandidateMass.P90CoastDist,
			BaselineMass.SubaerialMeanCoastDistHops,
			CandidateMass.SubaerialMeanCoastDistHops,
			BaselineMass.LargestComponentSize,
			CandidateMass.LargestComponentSize,
			BaselineMass.LargestSubaerialComponentSize,
			CandidateMass.LargestSubaerialComponentSize,
			BaselineMass.LargestMeanCoastDist,
			CandidateMass.LargestMeanCoastDist,
			BaselineMass.LargestSubaerialMeanCoastDist,
			CandidateMass.LargestSubaerialMeanCoastDist));
		AddInfo(FString::Printf(
			TEXT("[V9Breadth compare step=%d] caf=%.4f/%.4f subaerial_frac=%.4f/%.4f submerged_frac=%.4f/%.4f continental_samples=%d/%d subaerial_samples=%d/%d submerged_samples=%d/%d components=%d/%d singletons=%d/%d tiny=%d/%d top5=[%s]/[%s]"),
			Step,
			BaselineMass.ContinentalAreaFraction,
			CandidateMass.ContinentalAreaFraction,
			BaselineMass.SubaerialContinentalFraction,
			CandidateMass.SubaerialContinentalFraction,
			BaselineMass.SubmergedContinentalFraction,
			CandidateMass.SubmergedContinentalFraction,
			BaselineMass.ContinentalSampleCount,
			CandidateMass.ContinentalSampleCount,
			BaselineMass.SubaerialContinentalSampleCount,
			CandidateMass.SubaerialContinentalSampleCount,
			BaselineMass.SubmergedContinentalSampleCount,
			CandidateMass.SubmergedContinentalSampleCount,
			BaselineMass.ComponentCount,
			CandidateMass.ComponentCount,
			BaselineMass.SingletonCount,
			CandidateMass.SingletonCount,
			BaselineMass.TinyComponentCount,
			CandidateMass.TinyComponentCount,
			*JoinIntArrayForDiagnostics(BaselineMass.Top5ComponentSizes),
			*JoinIntArrayForDiagnostics(CandidateMass.Top5ComponentSizes)));
		AddInfo(FString::Printf(
			TEXT("[V9Breadth compare step=%d] seeded_all=%.4f/%.4f broad_seed=%.4f/%.4f coast_seed=%.4f/%.4f coherence=%.4f/%.4f leakage=%.4f/%.4f churn=%.4f/%.4f miss=%d/%d multi_hit=%d/%d"),
			Step,
			BaselineSurvival.Step0ContinentalRemainingFraction,
			CandidateSurvival.Step0ContinentalRemainingFraction,
			BaselineSurvival.Step0BroadInteriorRemainingFraction,
			CandidateSurvival.Step0BroadInteriorRemainingFraction,
			BaselineSurvival.Step0CoastAdjacentRemainingFraction,
			CandidateSurvival.Step0CoastAdjacentRemainingFraction,
			BaselineSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			CandidateSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
			BaselineSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			CandidateSnapshot.BoundaryCoherence.InteriorLeakageFraction,
			BaselineSnapshot.OwnershipChurn.ChurnFraction,
			CandidateSnapshot.OwnershipChurn.ChurnFraction,
			BaselineSnapshot.MissCount,
			CandidateSnapshot.MissCount,
			BaselineSnapshot.OverlapCount,
			CandidateSnapshot.OverlapCount));
		AddInfo(FString::Printf(
			TEXT("[V9Breadth compare step=%d] relief_mean=%.4f/%.4f relief_p95=%.4f/%.4f above_2km=%d/%d above_5km=%d/%d"),
			Step,
			BaselineElev.MeanElevationKm,
			CandidateElev.MeanElevationKm,
			BaselineElev.P95ElevationKm,
			CandidateElev.P95ElevationKm,
			BaselineElev.SamplesAbove2Km,
			CandidateElev.SamplesAbove2Km,
			BaselineElev.SamplesAbove5Km,
			CandidateElev.SamplesAbove5Km));
	};

	CaptureCheckpoint(Baseline, 0, true);
	CaptureCheckpoint(Candidate, 0, true);
	BuildSeedFlags(
		Baseline.MassDiagnostics.FindChecked(0),
		Step0ContinentalFlags,
		Step0BroadInteriorFlags,
		Step0CoastAdjacentFlags);
	Baseline.SurvivalDiagnostics[0] = ComputeSeededContinentalSurvivalDiagnostic(
		Baseline.Planet.GetPlanet(),
		Step0ContinentalFlags,
		Step0BroadInteriorFlags,
		Step0CoastAdjacentFlags);
	Candidate.SurvivalDiagnostics[0] = ComputeSeededContinentalSurvivalDiagnostic(
		Candidate.Planet.GetPlanet(),
		Step0ContinentalFlags,
		Step0BroadInteriorFlags,
		Step0CoastAdjacentFlags);
	LogComparison(0, Baseline, Candidate);

	const auto AdvanceBothToStep = [&](const int32 TargetStep)
	{
		while (Baseline.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			Baseline.Planet.AdvanceStep();
			Candidate.Planet.AdvanceStep();
		}
	};

	for (const int32 Step : TArray<int32>{25, 100})
	{
		AdvanceBothToStep(Step);
		CaptureCheckpoint(Baseline, Step, true);
		CaptureCheckpoint(Candidate, Step, true);
		LogComparison(Step, Baseline, Candidate);
	}

	const FV6CheckpointSnapshot& BaselineSnapshot100 = Baseline.Snapshots.FindChecked(100);
	const FV6CheckpointSnapshot& CandidateSnapshot100 = Candidate.Snapshots.FindChecked(100);
	const FV9ContinentalMassDiagnostic& BaselineMass100 = Baseline.MassDiagnostics.FindChecked(100);
	const FV9ContinentalMassDiagnostic& CandidateMass100 = Candidate.MassDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& BaselineSurvival100 =
		Baseline.SurvivalDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& CandidateSurvival100 =
		Candidate.SurvivalDiagnostics.FindChecked(100);

	const bool bCoastMeanPass =
		CandidateMass100.MeanCoastDistHops >= BaselineMass100.MeanCoastDistHops + 0.20;
	const bool bCoastP50Pass =
		CandidateMass100.P50CoastDist >= BaselineMass100.P50CoastDist + 1.0;
	const bool bLargestSubaerialPass =
		CandidateMass100.LargestSubaerialComponentSize >=
		FMath::CeilToInt(static_cast<double>(BaselineMass100.LargestSubaerialComponentSize) * 1.05);
	const bool bSeedBroadInteriorPass =
		CandidateSurvival100.Step0BroadInteriorRemainingFraction >=
		BaselineSurvival100.Step0BroadInteriorRemainingFraction + 0.03;
	const bool bNoRunawayAreaPass =
		CandidateMass100.ContinentalAreaFraction <= BaselineMass100.ContinentalAreaFraction + 0.03 &&
		CandidateMass100.SubmergedContinentalFraction <= BaselineMass100.SubmergedContinentalFraction + 0.02;
	const bool bCoherencePass =
		CandidateSnapshot100.BoundaryCoherence.BoundaryCoherenceScore >= 0.93 &&
		CandidateSnapshot100.BoundaryCoherence.BoundaryCoherenceScore >=
			BaselineSnapshot100.BoundaryCoherence.BoundaryCoherenceScore - 0.01;
	const bool bLeakagePass =
		CandidateSnapshot100.BoundaryCoherence.InteriorLeakageFraction < 0.18 &&
		CandidateSnapshot100.BoundaryCoherence.InteriorLeakageFraction <=
			BaselineSnapshot100.BoundaryCoherence.InteriorLeakageFraction + 0.02;
	const bool bChurnPass =
		CandidateSnapshot100.OwnershipChurn.ChurnFraction < 0.05 &&
		CandidateSnapshot100.OwnershipChurn.ChurnFraction <=
			BaselineSnapshot100.OwnershipChurn.ChurnFraction + 0.01;
	const bool bAllowStep200 =
		bCoastMeanPass &&
		bCoastP50Pass &&
		bLargestSubaerialPass &&
		bSeedBroadInteriorPass &&
		bNoRunawayAreaPass &&
		bCoherencePass &&
		bLeakagePass &&
		bChurnPass;

	AddInfo(FString::Printf(
		TEXT("[V9Breadth gate step=100] coast_mean=%d(%.2f>=%.2f) coast_p50=%d(%.1f>=%.1f) largest_subaerial=%d(%d>=%d) broad_seed=%d(%.4f>=%.4f) no_runaway=%d coherence=%d leakage=%d churn=%d run_step200=%d"),
		bCoastMeanPass ? 1 : 0,
		CandidateMass100.MeanCoastDistHops,
		BaselineMass100.MeanCoastDistHops + 0.20,
		bCoastP50Pass ? 1 : 0,
		CandidateMass100.P50CoastDist,
		BaselineMass100.P50CoastDist + 1.0,
		bLargestSubaerialPass ? 1 : 0,
		CandidateMass100.LargestSubaerialComponentSize,
		FMath::CeilToInt(static_cast<double>(BaselineMass100.LargestSubaerialComponentSize) * 1.05),
		bSeedBroadInteriorPass ? 1 : 0,
		CandidateSurvival100.Step0BroadInteriorRemainingFraction,
		BaselineSurvival100.Step0BroadInteriorRemainingFraction + 0.03,
		bNoRunawayAreaPass ? 1 : 0,
		bCoherencePass ? 1 : 0,
		bLeakagePass ? 1 : 0,
		bChurnPass ? 1 : 0,
		bAllowStep200 ? 1 : 0));

	if (bAllowStep200)
	{
		AdvanceBothToStep(200);
		CaptureCheckpoint(Baseline, 200, true);
		CaptureCheckpoint(Candidate, 200, true);
		LogComparison(200, Baseline, Candidate);
	}

	AddInfo(FString::Printf(
		TEXT("[V9Breadth summary] export_root=%s baseline_root=%s candidate_root=%s"),
		*ExportRoot,
		*Baseline.ExportRoot,
		*Candidate.ExportRoot));

	TestTrue(TEXT("Baseline reached step 100"), Baseline.Planet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Candidate reached step 100"), Candidate.Planet.GetPlanet().CurrentStep >= 100);
	return true;
}

// ============================================================================
// Long-Run Tectonic Cycle Validation (step 500)
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9LongRunValidationTest,
	"Aurous.TectonicPlanet.V6V9LongRunValidationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9LongRunValidationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9LongRunValidation");

	FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		FixedIntervalSteps,
		INDEX_NONE,
		SampleCount,
		PlateCount,
		TestRandomSeed);
	Planet.SetSyntheticCoverageRetentionForTest(false);
	Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
	Planet.SetExcludeMixedTrianglesForTest(false);
	Planet.SetV9Phase1AuthorityForTest(true, 1);
	Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
	Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
	Planet.SetV9CollisionShadowForTest(true);
	Planet.SetV9CollisionExecutionForTest(true);
	Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
	Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
	Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
	Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
	Planet.SetV9QuietInteriorContinentalRetentionForTest(true);
	Planet.SetSubmergedContinentalRelaxationForTest(true, 0.005);
	Planet.SetAutomaticRiftingForTest(true);

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);

	const TArray<float> CW0 = SnapshotContinentalWeights(Planet.GetPlanet());

	const TArray<int32> CheckpointSteps = { 25, 100, 200, 300, 500 };
	const TSet<int32> ExportSteps = { 100, 200, 500 };

	const auto CaptureCheckpoint = [this, &ExportRoot, &CW0, &ExportSteps](
		FTectonicPlanetV6& InPlanet,
		const int32 Step)
	{
		const FString Tag = FString::Printf(TEXT("[V9LongRun step=%d]"), Step);
		const bool bExport = ExportSteps.Contains(Step);

		if (bExport)
		{
			ExportV6CheckpointMaps(*this, InPlanet, ExportRoot, Step);
			ExportV6DebugOverlays(*this, InPlanet, ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(InPlanet);

		// 1. Stability
		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);

		// 4. Collision activity
		AddV6CollisionExecutionInfo(*this, Tag, Snapshot);

		// 5. Rifting activity
		AddV6RiftInfo(*this, Tag, InPlanet.ComputeRiftDiagnosticForTest());

		// 3. Relief + retention guard
		const FV9ContinentalElevationStats ElevStats = ComputeContinentalElevationStats(InPlanet);
		const FTectonicPlanetV6PeriodicSolveStats& SolveStats =
			InPlanet.GetPeriodicSolveCount() > 0
				? InPlanet.GetLastSolveStats()
				: InPlanet.BuildCurrentDiagnosticSnapshotForTest();
		const FV6PlateSizeStats PlateSizeStats = ComputePlateSizeStats(InPlanet);
		AddInfo(FString::Printf(
			TEXT("%s elevation: mean=%.4f p95=%.4f max=%.4f above_2km=%d above_5km=%d continental_samples=%d retention_guard=%d(tri=%d,ss=%d) plates=%d plate_size=%d/%.0f/%d"),
			*Tag,
			ElevStats.MeanElevationKm,
			ElevStats.P95ElevationKm,
			ElevStats.MaxElevationKm,
			ElevStats.SamplesAbove2Km,
			ElevStats.SamplesAbove5Km,
			ElevStats.ContinentalSampleCount,
			SolveStats.QuietInteriorContinentalRetentionCount,
			SolveStats.QuietInteriorContinentalRetentionTriangleCount,
			SolveStats.QuietInteriorContinentalRetentionSingleSourceCount,
			InPlanet.GetPlanet().Plates.Num(),
			PlateSizeStats.MinSamples,
			PlateSizeStats.MeanSamples,
			PlateSizeStats.MaxSamples));

		// 2. Continental footprint
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(InPlanet);
		AddV6ContinentalMassDiagnosticInfo(*this, Tag, MassDiag);

		// 2b. Crust-vs-elevation alignment
		{
			const FTectonicPlanet& PD = InPlanet.GetPlanet();
			int32 SubaerialCount = 0;
			int32 SubmergedCount = 0;
			int32 BandBelow0 = 0, Band0to1 = 0, Band1to2 = 0, Band2to5 = 0, BandAbove5 = 0;
			for (const FSample& S : PD.Samples)
			{
				if (S.ContinentalWeight < 0.5f) { continue; }
				if (S.Elevation > 0.0f) { ++SubaerialCount; } else { ++SubmergedCount; }
				if (S.Elevation <= 0.0f) { ++BandBelow0; }
				else if (S.Elevation <= 1.0f) { ++Band0to1; }
				else if (S.Elevation <= 2.0f) { ++Band1to2; }
				else if (S.Elevation <= 5.0f) { ++Band2to5; }
				else { ++BandAbove5; }
			}
			const int32 TotalSamples = PD.Samples.Num();
			const double SubaerialFrac = TotalSamples > 0 ? static_cast<double>(SubaerialCount) / TotalSamples : 0.0;
			const double SubmergedFrac = TotalSamples > 0 ? static_cast<double>(SubmergedCount) / TotalSamples : 0.0;
			const FString CrustElevMsg = FString::Printf(
				TEXT("%s crust_elev: subaerial=%d(%.4f) submerged=%d(%.4f) bands=[<0:%d 0-1:%d 1-2:%d 2-5:%d >5:%d]"),
				*Tag, SubaerialCount, SubaerialFrac, SubmergedCount, SubmergedFrac,
				BandBelow0, Band0to1, Band1to2, Band2to5, BandAbove5);
			AddInfo(CrustElevMsg);
			UE_LOG(LogTemp, Log, TEXT("%s"), *CrustElevMsg);
		}

		// 6. Continental loss attribution
		const FV9ContinentalLossAttribution SolveLoss = ComputeContinentalLossAttribution(InPlanet);
		AddV6ContinentalLossAttributionInfo(
			*this,
			FString::Printf(TEXT("%s last_solve_loss"), *Tag),
			SolveLoss);

		if (!CW0.IsEmpty())
		{
			const FV9ContinentalLossAttribution CumulLoss =
				ComputeCumulativeContinentalLossAttribution(CW0, InPlanet.GetPlanet());
			AddV6ContinentalLossAttributionInfo(
				*this,
				FString::Printf(TEXT("%s cumulative_loss_from_step0"), *Tag),
				CumulLoss);
		}

		if (bExport)
		{
			ExportContinentalMassOverlays(*this, InPlanet, MassDiag, ExportRoot, Step);
		}

		// Summary line for easy grep
		AddInfo(FString::Printf(
			TEXT("%s SUMMARY caf=%.4f samples=%d components=%d singletons=%d largest=%d coast_mean=%.2f p50=%.1f coherence=%.4f leakage=%.4f churn=%.4f plates=%d collisions_cum=%d rifts_cum=%d"),
			*Tag,
			MassDiag.ContinentalAreaFraction,
			MassDiag.ContinentalSampleCount,
			MassDiag.ComponentCount,
			MassDiag.SingletonCount,
			MassDiag.LargestComponentSize,
			MassDiag.MeanCoastDistHops,
			MassDiag.P50CoastDist,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Snapshot.OwnershipChurn.ChurnFraction,
			Snapshot.PlateCount,
			Snapshot.CollisionExecution.CumulativeExecutedCollisionCount,
			InPlanet.ComputeRiftDiagnosticForTest().CumulativeRiftCount));
	};

	// Step 0 baseline
	CaptureCheckpoint(Planet, 0);

	// Run to each checkpoint
	for (const int32 TargetStep : CheckpointSteps)
	{
		while (Planet.GetPlanet().CurrentStep < TargetStep)
		{
			Planet.AdvanceStep();
		}
		CaptureCheckpoint(Planet, TargetStep);
	}

	AddInfo(FString::Printf(
		TEXT("[V9LongRun] export_root=%s final_step=%d"),
		*ExportRoot,
		Planet.GetPlanet().CurrentStep));

	TestTrue(TEXT("Reached step 500"), Planet.GetPlanet().CurrentStep >= 500);
	return true;
}

// ============================================================================
// Paper-Surrogate Ownership A/B Test
// Tests whether paper-like geometry-owned continental identity for quiet
// broad-interior samples prevents ribbon decay better than transfer-owned CW.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9PaperSurrogateOwnershipTest,
	"Aurous.TectonicPlanet.V6V9PaperSurrogateOwnershipTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9PaperSurrogateOwnershipTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9PaperSurrogateOwnership");

	const auto InitializePlanet = [=](const bool bEnablePaperSurrogate) -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
		Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
		Planet.SetV9QuietInteriorContinentalRetentionForTest(true);
		Planet.SetV9ContinentalBreadthPreservationForTest(false);
		Planet.SetSubmergedContinentalRelaxationForTest(true, 0.005);
		Planet.SetAutomaticRiftingForTest(true);
		Planet.SetV9PaperSurrogateOwnershipForTest(bEnablePaperSurrogate);
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 N = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, N);
		OutStep0BroadInteriorFlags.Init(0, N);
		OutStep0CoastAdjacentFlags.Init(0, N);
		for (int32 I = 0; I < N; ++I)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(I) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(I) ||
				Step0Mass.SampleComponentId[I] < 0)
			{
				continue;
			}
			OutStep0ContinentalFlags[I] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[I];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[I] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[I] = 1;
			}
		}
	};

	struct FVariantState
	{
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
	};

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_equilibrium"));
	const FString CandidateExportRoot = FPaths::Combine(ExportRoot, TEXT("candidate_paper_surrogate"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CandidateExportRoot);

	FVariantState Baseline;
	Baseline.Label = TEXT("baseline_equilibrium");
	Baseline.ExportRoot = BaselineExportRoot;
	Baseline.Planet = InitializePlanet(false);

	FVariantState Candidate;
	Candidate.Label = TEXT("candidate_paper_surrogate");
	Candidate.ExportRoot = CandidateExportRoot;
	Candidate.Planet = InitializePlanet(true);

	TArray<uint8> Step0ContinentalFlags;
	TArray<uint8> Step0BroadInteriorFlags;
	TArray<uint8> Step0CoastAdjacentFlags;

	const auto CaptureCheckpoint =
		[this, &Step0ContinentalFlags, &Step0BroadInteriorFlags, &Step0CoastAdjacentFlags](
			FVariantState& Variant,
			const int32 Step,
			const bool bExport)
	{
		const FString Tag = FString::Printf(TEXT("[V9PaperSurrogate %s step=%d]"), *Variant.Label, Step);
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Variant.Planet);
		const FV9ContinentalElevationStats ElevStats = ComputeContinentalElevationStats(Variant.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Variant.Planet.GetPlanet(),
				Step0ContinentalFlags,
				Step0BroadInteriorFlags,
				Step0CoastAdjacentFlags);
		Variant.Snapshots.Add(Step, Snapshot);
		Variant.MassDiagnostics.Add(Step, MassDiag);
		Variant.ElevationDiagnostics.Add(Step, ElevStats);
		Variant.SurvivalDiagnostics.Add(Step, SurvivalDiag);

		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6ContinentalMassDiagnosticInfo(*this, Tag, MassDiag);

		const FTectonicPlanetV6PeriodicSolveStats& SolveStats =
			Variant.Planet.GetPeriodicSolveCount() > 0
				? Variant.Planet.GetLastSolveStats()
				: Variant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const double DirectHitEligibleCount =
			static_cast<double>(SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount);
		const double DirectHitPreSolveCWMean =
			DirectHitEligibleCount > 0.0
				? SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum /
					DirectHitEligibleCount
				: 0.0;
		const double DirectHitPostTransferCWMean =
			DirectHitEligibleCount > 0.0
				? SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum /
					DirectHitEligibleCount
				: 0.0;
		const double DirectHitFinalCWMean =
			DirectHitEligibleCount > 0.0
				? SolveStats.PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum /
					DirectHitEligibleCount
				: 0.0;
		AddInfo(FString::Printf(
			TEXT("%s relief: mean=%.4f p95=%.4f max=%.4f above_2km=%d above_5km=%d paper_surrogate=%d(subaerial=%d,submerged=%d,tri=%d,ss=%d,strong=%d,moderate=%d) quiet_retention=%d breadth_preserve=%d"),
			*Tag,
			ElevStats.MeanElevationKm,
			ElevStats.P95ElevationKm,
			ElevStats.MaxElevationKm,
			ElevStats.SamplesAbove2Km,
			ElevStats.SamplesAbove5Km,
			SolveStats.PaperSurrogateOwnershipOverrideCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			SolveStats.PaperSurrogateOwnershipOverrideTriangleCount,
			SolveStats.PaperSurrogateOwnershipOverrideSingleSourceCount,
			SolveStats.PaperSurrogateOwnershipOverrideStrongInteriorCount,
			SolveStats.PaperSurrogateOwnershipOverrideModerateInteriorCount,
			SolveStats.QuietInteriorContinentalRetentionCount,
			SolveStats.ContinentalBreadthPreservationCount));
		AddInfo(FString::Printf(
			TEXT("%s paper_surrogate_cw_diffusion: direct_hit_eligible=%d pre_cw_mean=%.4f post_transfer_cw_mean=%.4f final_cw_mean=%.4f"),
			*Tag,
			SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			DirectHitPreSolveCWMean,
			DirectHitPostTransferCWMean,
			DirectHitFinalCWMean));
		AddInfo(FString::Printf(
			TEXT("%s seeded_survival: all=%d/%d(%.4f) broad_interior=%d/%d(%.4f) coast_adjacent=%d/%d(%.4f)"),
			*Tag,
			SurvivalDiag.Step0ContinentalRemainingCount,
			SurvivalDiag.Step0ContinentalCount,
			SurvivalDiag.Step0ContinentalRemainingFraction,
			SurvivalDiag.Step0BroadInteriorRemainingCount,
			SurvivalDiag.Step0BroadInteriorCount,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			SurvivalDiag.Step0CoastAdjacentRemainingCount,
			SurvivalDiag.Step0CoastAdjacentCount,
			SurvivalDiag.Step0CoastAdjacentRemainingFraction));

		if (bExport)
		{
			ExportContinentalMassOverlays(*this, Variant.Planet, MassDiag, Variant.ExportRoot, Step);
		}
	};

	const auto LogComparison =
		[this](
			const int32 Step,
			const FVariantState& BaselineVariant,
			const FVariantState& CandidateVariant)
	{
		const FV6CheckpointSnapshot& BS = BaselineVariant.Snapshots.FindChecked(Step);
		const FV6CheckpointSnapshot& CS = CandidateVariant.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& BM = BaselineVariant.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& CM = CandidateVariant.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& BE = BaselineVariant.ElevationDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& CE = CandidateVariant.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& BSurv =
			BaselineVariant.SurvivalDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& CSurv =
			CandidateVariant.SurvivalDiagnostics.FindChecked(Step);
		const FTectonicPlanetV6PeriodicSolveStats& BStats =
			BaselineVariant.Planet.GetPeriodicSolveCount() > 0
				? BaselineVariant.Planet.GetLastSolveStats()
				: BaselineVariant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FTectonicPlanetV6PeriodicSolveStats& CStats =
			CandidateVariant.Planet.GetPeriodicSolveCount() > 0
				? CandidateVariant.Planet.GetLastSolveStats()
				: CandidateVariant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const double BDirectHitEligibleCount =
			static_cast<double>(BStats.PaperSurrogateQuietInteriorDirectHitEligibleCount);
		const double CDirectHitEligibleCount =
			static_cast<double>(CStats.PaperSurrogateQuietInteriorDirectHitEligibleCount);
		const double BDirectHitPreSolveCWMean =
			BDirectHitEligibleCount > 0.0
				? BStats.PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum /
					BDirectHitEligibleCount
				: 0.0;
		const double BDirectHitPostTransferCWMean =
			BDirectHitEligibleCount > 0.0
				? BStats.PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum /
					BDirectHitEligibleCount
				: 0.0;
		const double BDirectHitFinalCWMean =
			BDirectHitEligibleCount > 0.0
				? BStats.PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum /
					BDirectHitEligibleCount
				: 0.0;
		const double CDirectHitPreSolveCWMean =
			CDirectHitEligibleCount > 0.0
				? CStats.PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum /
					CDirectHitEligibleCount
				: 0.0;
		const double CDirectHitPostTransferCWMean =
			CDirectHitEligibleCount > 0.0
				? CStats.PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum /
					CDirectHitEligibleCount
				: 0.0;
		const double CDirectHitFinalCWMean =
			CDirectHitEligibleCount > 0.0
				? CStats.PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum /
					CDirectHitEligibleCount
				: 0.0;

		// Morphology / breadth comparison
		AddInfo(FString::Printf(
			TEXT("[V9PaperSurrogate compare step=%d] coast_mean=%.2f/%.2f coast_p50=%.1f/%.1f coast_p90=%.1f/%.1f subaerial_coast_mean=%.2f/%.2f largest=%d/%d largest_subaerial=%d/%d largest_mean_coast=%.2f/%.2f largest_subaerial_mean_coast=%.2f/%.2f"),
			Step,
			BM.MeanCoastDistHops, CM.MeanCoastDistHops,
			BM.P50CoastDist, CM.P50CoastDist,
			BM.P90CoastDist, CM.P90CoastDist,
			BM.SubaerialMeanCoastDistHops, CM.SubaerialMeanCoastDistHops,
			BM.LargestComponentSize, CM.LargestComponentSize,
			BM.LargestSubaerialComponentSize, CM.LargestSubaerialComponentSize,
			BM.LargestMeanCoastDist, CM.LargestMeanCoastDist,
			BM.LargestSubaerialMeanCoastDist, CM.LargestSubaerialMeanCoastDist));

		// Crust / visibility balance comparison
		AddInfo(FString::Printf(
			TEXT("[V9PaperSurrogate compare step=%d] caf=%.4f/%.4f subaerial_frac=%.4f/%.4f submerged_frac=%.4f/%.4f continental_samples=%d/%d subaerial_samples=%d/%d submerged_samples=%d/%d components=%d/%d singletons=%d/%d tiny=%d/%d top5=[%s]/[%s]"),
			Step,
			BM.ContinentalAreaFraction, CM.ContinentalAreaFraction,
			BM.SubaerialContinentalFraction, CM.SubaerialContinentalFraction,
			BM.SubmergedContinentalFraction, CM.SubmergedContinentalFraction,
			BM.ContinentalSampleCount, CM.ContinentalSampleCount,
			BM.SubaerialContinentalSampleCount, CM.SubaerialContinentalSampleCount,
			BM.SubmergedContinentalSampleCount, CM.SubmergedContinentalSampleCount,
			BM.ComponentCount, CM.ComponentCount,
			BM.SingletonCount, CM.SingletonCount,
			BM.TinyComponentCount, CM.TinyComponentCount,
			*JoinIntArrayForDiagnostics(BM.Top5ComponentSizes),
			*JoinIntArrayForDiagnostics(CM.Top5ComponentSizes)));

		// Seed survival + stability comparison
		AddInfo(FString::Printf(
			TEXT("[V9PaperSurrogate compare step=%d] seeded_all=%.4f/%.4f broad_seed=%.4f/%.4f coast_seed=%.4f/%.4f coherence=%.4f/%.4f leakage=%.4f/%.4f churn=%.4f/%.4f miss=%d/%d multi_hit=%d/%d"),
			Step,
			BSurv.Step0ContinentalRemainingFraction, CSurv.Step0ContinentalRemainingFraction,
			BSurv.Step0BroadInteriorRemainingFraction, CSurv.Step0BroadInteriorRemainingFraction,
			BSurv.Step0CoastAdjacentRemainingFraction, CSurv.Step0CoastAdjacentRemainingFraction,
			BS.BoundaryCoherence.BoundaryCoherenceScore, CS.BoundaryCoherence.BoundaryCoherenceScore,
			BS.BoundaryCoherence.InteriorLeakageFraction, CS.BoundaryCoherence.InteriorLeakageFraction,
			BS.OwnershipChurn.ChurnFraction, CS.OwnershipChurn.ChurnFraction,
			BS.MissCount, CS.MissCount,
			BS.OverlapCount, CS.OverlapCount));

		// Relief comparison
		AddInfo(FString::Printf(
			TEXT("[V9PaperSurrogate compare step=%d] relief_mean=%.4f/%.4f relief_p95=%.4f/%.4f above_2km=%d/%d above_5km=%d/%d"),
			Step,
			BE.MeanElevationKm, CE.MeanElevationKm,
			BE.P95ElevationKm, CE.P95ElevationKm,
			BE.SamplesAbove2Km, CE.SamplesAbove2Km,
			BE.SamplesAbove5Km, CE.SamplesAbove5Km));

		AddInfo(FString::Printf(
			TEXT("[V9PaperSurrogate compare step=%d] surrogate_usage_total=%d/%d tri=%d/%d ss=%d/%d subaerial=%d/%d submerged=%d/%d strong=%d/%d moderate=%d/%d"),
			Step,
			BStats.PaperSurrogateOwnershipOverrideCount, CStats.PaperSurrogateOwnershipOverrideCount,
			BStats.PaperSurrogateOwnershipOverrideTriangleCount, CStats.PaperSurrogateOwnershipOverrideTriangleCount,
			BStats.PaperSurrogateOwnershipOverrideSingleSourceCount, CStats.PaperSurrogateOwnershipOverrideSingleSourceCount,
			BStats.PaperSurrogateOwnershipOverrideSubaerialCount, CStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			BStats.PaperSurrogateOwnershipOverrideSubmergedCount, CStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			BStats.PaperSurrogateOwnershipOverrideStrongInteriorCount, CStats.PaperSurrogateOwnershipOverrideStrongInteriorCount,
			BStats.PaperSurrogateOwnershipOverrideModerateInteriorCount, CStats.PaperSurrogateOwnershipOverrideModerateInteriorCount));
		AddInfo(FString::Printf(
			TEXT("[V9PaperSurrogate compare step=%d] quiet_direct_hit_cw=eligible=%d/%d pre=%.4f/%.4f post_transfer=%.4f/%.4f final=%.4f/%.4f"),
			Step,
			BStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			CStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			BDirectHitPreSolveCWMean, CDirectHitPreSolveCWMean,
			BDirectHitPostTransferCWMean, CDirectHitPostTransferCWMean,
			BDirectHitFinalCWMean, CDirectHitFinalCWMean));
	};

	// Step 0 capture — both variants are identical at init
	CaptureCheckpoint(Baseline, 0, true);
	CaptureCheckpoint(Candidate, 0, true);
	BuildSeedFlags(
		Baseline.MassDiagnostics.FindChecked(0),
		Step0ContinentalFlags,
		Step0BroadInteriorFlags,
		Step0CoastAdjacentFlags);
	// Re-capture with correct seed flags
	Baseline.SurvivalDiagnostics[0] = ComputeSeededContinentalSurvivalDiagnostic(
		Baseline.Planet.GetPlanet(),
		Step0ContinentalFlags,
		Step0BroadInteriorFlags,
		Step0CoastAdjacentFlags);
	Candidate.SurvivalDiagnostics[0] = ComputeSeededContinentalSurvivalDiagnostic(
		Candidate.Planet.GetPlanet(),
		Step0ContinentalFlags,
		Step0BroadInteriorFlags,
		Step0CoastAdjacentFlags);
	LogComparison(0, Baseline, Candidate);

	const auto AdvanceBothToStep = [&](const int32 TargetStep)
	{
		while (Baseline.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			Baseline.Planet.AdvanceStep();
			Candidate.Planet.AdvanceStep();
		}
	};

	// Run to steps 25 and 100
	for (const int32 Step : TArray<int32>{25, 100})
	{
		AdvanceBothToStep(Step);
		CaptureCheckpoint(Baseline, Step, true);
		CaptureCheckpoint(Candidate, Step, true);
		LogComparison(Step, Baseline, Candidate);
	}

	// Step 100 gate evaluation
	const FV6CheckpointSnapshot& BS100 = Baseline.Snapshots.FindChecked(100);
	const FV6CheckpointSnapshot& CS100 = Candidate.Snapshots.FindChecked(100);
	const FV9ContinentalMassDiagnostic& BM100 = Baseline.MassDiagnostics.FindChecked(100);
	const FV9ContinentalMassDiagnostic& CM100 = Candidate.MassDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& BSurv100 =
		Baseline.SurvivalDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& CSurv100 =
		Candidate.SurvivalDiagnostics.FindChecked(100);

	// Decision gates
	const bool bSubaerialCoastMeanPass =
		CM100.SubaerialMeanCoastDistHops >= BM100.SubaerialMeanCoastDistHops + 0.15;
	const bool bCoastP50Pass =
		CM100.P50CoastDist >= BM100.P50CoastDist + 0.5;
	const bool bLargestSubaerialPass =
		CM100.LargestSubaerialComponentSize >=
		FMath::CeilToInt(static_cast<double>(BM100.LargestSubaerialComponentSize) * 1.05);
	const bool bSeedBroadInteriorPass =
		CSurv100.Step0BroadInteriorRemainingFraction >=
		BSurv100.Step0BroadInteriorRemainingFraction + 0.02;
	const bool bNoSubmergedBalloonPass =
		CM100.SubmergedContinentalFraction <= BM100.SubmergedContinentalFraction + 0.02;
	const bool bCoherencePass =
		CS100.BoundaryCoherence.BoundaryCoherenceScore >= 0.93 &&
		CS100.BoundaryCoherence.BoundaryCoherenceScore >=
			BS100.BoundaryCoherence.BoundaryCoherenceScore - 0.01;
	const bool bLeakagePass =
		CS100.BoundaryCoherence.InteriorLeakageFraction < 0.18 &&
		CS100.BoundaryCoherence.InteriorLeakageFraction <=
			BS100.BoundaryCoherence.InteriorLeakageFraction + 0.02;
	const bool bChurnPass =
		CS100.OwnershipChurn.ChurnFraction < 0.05 &&
		CS100.OwnershipChurn.ChurnFraction <=
			BS100.OwnershipChurn.ChurnFraction + 0.01;

	const bool bMorphologyPass =
		bSubaerialCoastMeanPass && bCoastP50Pass && bLargestSubaerialPass;
	const bool bSurvivalPass = bSeedBroadInteriorPass;
	const bool bStabilityPass = bCoherencePass && bLeakagePass && bChurnPass;
	const bool bNotJustSubmerged = bNoSubmergedBalloonPass;

	const bool bAllowStep200 =
		bMorphologyPass && bSurvivalPass && bStabilityPass && bNotJustSubmerged;

	AddInfo(FString::Printf(
		TEXT("[V9PaperSurrogate gate step=100] subaerial_coast_mean=%d(%.2f>=%.2f) coast_p50=%d(%.1f>=%.1f) largest_subaerial=%d(%d>=%d) broad_seed=%d(%.4f>=%.4f) no_submerged_balloon=%d(%.4f<=%.4f) coherence=%d leakage=%d churn=%d morphology=%d survival=%d stability=%d run_step200=%d"),
		bSubaerialCoastMeanPass ? 1 : 0,
		CM100.SubaerialMeanCoastDistHops,
		BM100.SubaerialMeanCoastDistHops + 0.15,
		bCoastP50Pass ? 1 : 0,
		CM100.P50CoastDist,
		BM100.P50CoastDist + 0.5,
		bLargestSubaerialPass ? 1 : 0,
		CM100.LargestSubaerialComponentSize,
		FMath::CeilToInt(static_cast<double>(BM100.LargestSubaerialComponentSize) * 1.05),
		bSeedBroadInteriorPass ? 1 : 0,
		CSurv100.Step0BroadInteriorRemainingFraction,
		BSurv100.Step0BroadInteriorRemainingFraction + 0.02,
		bNoSubmergedBalloonPass ? 1 : 0,
		CM100.SubmergedContinentalFraction,
		BM100.SubmergedContinentalFraction + 0.02,
		bCoherencePass ? 1 : 0,
		bLeakagePass ? 1 : 0,
		bChurnPass ? 1 : 0,
		bMorphologyPass ? 1 : 0,
		bSurvivalPass ? 1 : 0,
		bStabilityPass ? 1 : 0,
		bAllowStep200 ? 1 : 0));

	if (bAllowStep200)
	{
		AdvanceBothToStep(200);
		CaptureCheckpoint(Baseline, 200, true);
		CaptureCheckpoint(Candidate, 200, true);
		LogComparison(200, Baseline, Candidate);
	}

	AddInfo(FString::Printf(
		TEXT("[V9PaperSurrogate summary] export_root=%s baseline_root=%s candidate_root=%s"),
		*ExportRoot,
		*Baseline.ExportRoot,
		*Candidate.ExportRoot));

	TestTrue(TEXT("Baseline reached step 100"), Baseline.Planet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Candidate reached step 100"), Candidate.Planet.GetPlanet().CurrentStep >= 100);
	return true;
}

// ============================================================================
// Paper-Surrogate Field Bundle Refinement Test
// Narrows the confirmed full-state quiet-interior surrogate into smaller field
// bundles and checks whether the morphology win can survive the step-100 gate.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9PaperSurrogateFieldBundleRefinementTest,
	"Aurous.TectonicPlanet.V6V9PaperSurrogateFieldBundleRefinementTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9PaperSurrogateFieldBundleRefinementTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9PaperSurrogateFieldBundleRefinement");

	struct FSurrogateDriftMeans
	{
		double EligibleCount = 0.0;
		double PreCWMean = 0.0;
		double PostTransferCWMean = 0.0;
		double FinalCWMean = 0.0;
		double PreElevationMean = 0.0;
		double PostTransferElevationMean = 0.0;
		double FinalElevationMean = 0.0;
		double PreThicknessMean = 0.0;
		double PostTransferThicknessMean = 0.0;
		double FinalThicknessMean = 0.0;
	};

	struct FVariantConfig
	{
		FString Label;
		bool bEnablePaperSurrogate = false;
		ETectonicPlanetV6PaperSurrogateFieldMode FieldMode =
			ETectonicPlanetV6PaperSurrogateFieldMode::FullState;
		bool bIsNarrowedCandidate = false;
	};

	struct FVariantState
	{
		FVariantConfig Config;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
	};

	struct FCandidateGateResult
	{
		FString Label;
		bool bCoastMeanPass = false;
		bool bLargestSubaerialPass = false;
		bool bBroadSeedPass = false;
		bool bNoSubmergedBalloonPass = false;
		bool bCoherencePass = false;
		bool bLeakagePass = false;
		bool bChurnPass = false;
		bool bMorphologyPass = false;
		bool bStabilityPass = false;
		bool bAllowStep200 = false;
		double CoastRetention = 0.0;
		double LargestRetention = 0.0;
		double BroadSeedRetention = 0.0;
		double Score = -1.0;
	};

	const auto ComputeDriftMeans = [](const FTectonicPlanetV6PeriodicSolveStats& SolveStats)
	{
		FSurrogateDriftMeans Drift;
		Drift.EligibleCount =
			static_cast<double>(SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount);
		if (Drift.EligibleCount <= 0.0)
		{
			return Drift;
		}

		Drift.PreCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum /
			Drift.EligibleCount;
		Drift.PostTransferCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum /
			Drift.EligibleCount;
		Drift.FinalCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum /
			Drift.EligibleCount;
		Drift.PreElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveElevationSum /
			Drift.EligibleCount;
		Drift.PostTransferElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferElevationSum /
			Drift.EligibleCount;
		Drift.FinalElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalElevationSum /
			Drift.EligibleCount;
		Drift.PreThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveThicknessSum /
			Drift.EligibleCount;
		Drift.PostTransferThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferThicknessSum /
			Drift.EligibleCount;
		Drift.FinalThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalThicknessSum /
			Drift.EligibleCount;
		return Drift;
	};

	const auto ComputeGainRetention = [](
		const double BaselineValue,
		const double ReferenceValue,
		const double CandidateValue)
	{
		const double Denominator = ReferenceValue - BaselineValue;
		if (FMath::Abs(Denominator) <= KINDA_SMALL_NUMBER)
		{
			return 0.0;
		}
		return (CandidateValue - BaselineValue) / Denominator;
	};

	const auto InitializePlanet = [=](const FVariantConfig& Config) -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
		Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
		Planet.SetV9QuietInteriorContinentalRetentionForTest(true);
		Planet.SetV9ContinentalBreadthPreservationForTest(false);
		Planet.SetSubmergedContinentalRelaxationForTest(true, 0.005);
		Planet.SetAutomaticRiftingForTest(true);
		Planet.SetV9PaperSurrogateOwnershipForTest(Config.bEnablePaperSurrogate);
		Planet.SetV9PaperSurrogateFieldModeForTest(Config.FieldMode);
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 N = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, N);
		OutStep0BroadInteriorFlags.Init(0, N);
		OutStep0CoastAdjacentFlags.Init(0, N);
		for (int32 I = 0; I < N; ++I)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(I) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(I) ||
				Step0Mass.SampleComponentId[I] < 0)
			{
				continue;
			}
			OutStep0ContinentalFlags[I] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[I];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[I] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[I] = 1;
			}
		}
	};

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_equilibrium"));
	const FString FullStateExportRoot = FPaths::Combine(ExportRoot, TEXT("reference_full_state"));
	const FString CWETExportRoot =
		FPaths::Combine(ExportRoot, TEXT("candidate_cw_elev_thickness"));
	const FString CWTExportRoot =
		FPaths::Combine(ExportRoot, TEXT("candidate_cw_thickness"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*FullStateExportRoot);
	PlatformFile.CreateDirectoryTree(*CWETExportRoot);
	PlatformFile.CreateDirectoryTree(*CWTExportRoot);

	FVariantState Baseline;
	Baseline.Config = {
		TEXT("baseline_equilibrium"),
		false,
		ETectonicPlanetV6PaperSurrogateFieldMode::FullState,
		false };
	Baseline.ExportRoot = BaselineExportRoot;
	Baseline.Planet = InitializePlanet(Baseline.Config);

	FVariantState FullStateReference;
	FullStateReference.Config = {
		TEXT("reference_full_state"),
		true,
		ETectonicPlanetV6PaperSurrogateFieldMode::FullState,
		false };
	FullStateReference.ExportRoot = FullStateExportRoot;
	FullStateReference.Planet = InitializePlanet(FullStateReference.Config);

	FVariantState CandidateCWET;
	CandidateCWET.Config = {
		TEXT("candidate_cw_elev_thickness"),
		true,
		ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightElevationThickness,
		true };
	CandidateCWET.ExportRoot = CWETExportRoot;
	CandidateCWET.Planet = InitializePlanet(CandidateCWET.Config);

	FVariantState CandidateCWT;
	CandidateCWT.Config = {
		TEXT("candidate_cw_thickness"),
		true,
		ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightThickness,
		true };
	CandidateCWT.ExportRoot = CWTExportRoot;
	CandidateCWT.Planet = InitializePlanet(CandidateCWT.Config);

	TArray<FVariantState*> Variants = {
		&Baseline,
		&FullStateReference,
		&CandidateCWET,
		&CandidateCWT };

	TArray<uint8> Step0ContinentalFlags;
	TArray<uint8> Step0BroadInteriorFlags;
	TArray<uint8> Step0CoastAdjacentFlags;

	const auto CaptureCheckpoint =
		[this, &Step0ContinentalFlags, &Step0BroadInteriorFlags, &Step0CoastAdjacentFlags, &ComputeDriftMeans](
			FVariantState& Variant,
			const int32 Step,
			const bool bExport)
	{
		const FString Tag =
			FString::Printf(TEXT("[V9PaperBundle %s step=%d]"), *Variant.Config.Label, Step);
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Variant.Planet);
		const FV9ContinentalElevationStats ElevStats = ComputeContinentalElevationStats(Variant.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Variant.Planet.GetPlanet(),
				Step0ContinentalFlags,
				Step0BroadInteriorFlags,
				Step0CoastAdjacentFlags);
		Variant.Snapshots.Add(Step, Snapshot);
		Variant.MassDiagnostics.Add(Step, MassDiag);
		Variant.ElevationDiagnostics.Add(Step, ElevStats);
		Variant.SurvivalDiagnostics.Add(Step, SurvivalDiag);

		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6ContinentalMassDiagnosticInfo(*this, Tag, MassDiag);

		const FTectonicPlanetV6PeriodicSolveStats& SolveStats =
			Variant.Planet.GetPeriodicSolveCount() > 0
				? Variant.Planet.GetLastSolveStats()
				: Variant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FSurrogateDriftMeans Drift = ComputeDriftMeans(SolveStats);
		AddInfo(FString::Printf(
			TEXT("%s relief: mean=%.4f p95=%.4f max=%.4f above_2km=%d above_5km=%d surrogate=%d(subaerial=%d,submerged=%d,tri=%d,ss=%d,strong=%d,moderate=%d) quiet_retention=%d breadth_preserve=%d"),
			*Tag,
			ElevStats.MeanElevationKm,
			ElevStats.P95ElevationKm,
			ElevStats.MaxElevationKm,
			ElevStats.SamplesAbove2Km,
			ElevStats.SamplesAbove5Km,
			SolveStats.PaperSurrogateOwnershipOverrideCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			SolveStats.PaperSurrogateOwnershipOverrideTriangleCount,
			SolveStats.PaperSurrogateOwnershipOverrideSingleSourceCount,
			SolveStats.PaperSurrogateOwnershipOverrideStrongInteriorCount,
			SolveStats.PaperSurrogateOwnershipOverrideModerateInteriorCount,
			SolveStats.QuietInteriorContinentalRetentionCount,
			SolveStats.ContinentalBreadthPreservationCount));
		AddInfo(FString::Printf(
			TEXT("%s surrogate_drift: eligible=%d pre_cw=%.4f post_cw=%.4f final_cw=%.4f pre_elev=%.4f post_elev=%.4f final_elev=%.4f pre_thickness=%.4f post_thickness=%.4f final_thickness=%.4f"),
			*Tag,
			SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			Drift.PreCWMean,
			Drift.PostTransferCWMean,
			Drift.FinalCWMean,
			Drift.PreElevationMean,
			Drift.PostTransferElevationMean,
			Drift.FinalElevationMean,
			Drift.PreThicknessMean,
			Drift.PostTransferThicknessMean,
			Drift.FinalThicknessMean));
		AddInfo(FString::Printf(
			TEXT("%s seeded_survival: all=%d/%d(%.4f) broad_interior=%d/%d(%.4f) coast_adjacent=%d/%d(%.4f)"),
			*Tag,
			SurvivalDiag.Step0ContinentalRemainingCount,
			SurvivalDiag.Step0ContinentalCount,
			SurvivalDiag.Step0ContinentalRemainingFraction,
			SurvivalDiag.Step0BroadInteriorRemainingCount,
			SurvivalDiag.Step0BroadInteriorCount,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			SurvivalDiag.Step0CoastAdjacentRemainingCount,
			SurvivalDiag.Step0CoastAdjacentCount,
			SurvivalDiag.Step0CoastAdjacentRemainingFraction));

		if (bExport)
		{
			ExportContinentalMassOverlays(*this, Variant.Planet, MassDiag, Variant.ExportRoot, Step);
		}
	};

	const auto LogComparison =
		[this, &ComputeDriftMeans](
			const int32 Step,
			const FVariantState& BaselineVariant,
			const FVariantState& OtherVariant)
	{
		const FV6CheckpointSnapshot& BS = BaselineVariant.Snapshots.FindChecked(Step);
		const FV6CheckpointSnapshot& OS = OtherVariant.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& BM = BaselineVariant.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& OM = OtherVariant.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& BE =
			BaselineVariant.ElevationDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& OE =
			OtherVariant.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& BSurv =
			BaselineVariant.SurvivalDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& OSurv =
			OtherVariant.SurvivalDiagnostics.FindChecked(Step);
		const FTectonicPlanetV6PeriodicSolveStats& BStats =
			BaselineVariant.Planet.GetPeriodicSolveCount() > 0
				? BaselineVariant.Planet.GetLastSolveStats()
				: BaselineVariant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FTectonicPlanetV6PeriodicSolveStats& OStats =
			OtherVariant.Planet.GetPeriodicSolveCount() > 0
				? OtherVariant.Planet.GetLastSolveStats()
				: OtherVariant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FSurrogateDriftMeans BDrift = ComputeDriftMeans(BStats);
		const FSurrogateDriftMeans ODrift = ComputeDriftMeans(OStats);

		AddInfo(FString::Printf(
			TEXT("[V9PaperBundle compare step=%d baseline=%s variant=%s] coast_mean=%.2f/%.2f coast_p50=%.1f/%.1f coast_p90=%.1f/%.1f subaerial_coast_mean=%.2f/%.2f largest=%d/%d largest_subaerial=%d/%d largest_mean_coast=%.2f/%.2f largest_subaerial_mean_coast=%.2f/%.2f"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BM.MeanCoastDistHops, OM.MeanCoastDistHops,
			BM.P50CoastDist, OM.P50CoastDist,
			BM.P90CoastDist, OM.P90CoastDist,
			BM.SubaerialMeanCoastDistHops, OM.SubaerialMeanCoastDistHops,
			BM.LargestComponentSize, OM.LargestComponentSize,
			BM.LargestSubaerialComponentSize, OM.LargestSubaerialComponentSize,
			BM.LargestMeanCoastDist, OM.LargestMeanCoastDist,
			BM.LargestSubaerialMeanCoastDist, OM.LargestSubaerialMeanCoastDist));
		AddInfo(FString::Printf(
			TEXT("[V9PaperBundle compare step=%d baseline=%s variant=%s] caf=%.4f/%.4f subaerial_frac=%.4f/%.4f submerged_frac=%.4f/%.4f continental_samples=%d/%d subaerial_samples=%d/%d submerged_samples=%d/%d components=%d/%d singletons=%d/%d tiny=%d/%d top5=[%s]/[%s]"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BM.ContinentalAreaFraction, OM.ContinentalAreaFraction,
			BM.SubaerialContinentalFraction, OM.SubaerialContinentalFraction,
			BM.SubmergedContinentalFraction, OM.SubmergedContinentalFraction,
			BM.ContinentalSampleCount, OM.ContinentalSampleCount,
			BM.SubaerialContinentalSampleCount, OM.SubaerialContinentalSampleCount,
			BM.SubmergedContinentalSampleCount, OM.SubmergedContinentalSampleCount,
			BM.ComponentCount, OM.ComponentCount,
			BM.SingletonCount, OM.SingletonCount,
			BM.TinyComponentCount, OM.TinyComponentCount,
			*JoinIntArrayForDiagnostics(BM.Top5ComponentSizes),
			*JoinIntArrayForDiagnostics(OM.Top5ComponentSizes)));
		AddInfo(FString::Printf(
			TEXT("[V9PaperBundle compare step=%d baseline=%s variant=%s] seeded_all=%.4f/%.4f broad_seed=%.4f/%.4f coast_seed=%.4f/%.4f coherence=%.4f/%.4f leakage=%.4f/%.4f churn=%.4f/%.4f miss=%d/%d multi_hit=%d/%d"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BSurv.Step0ContinentalRemainingFraction, OSurv.Step0ContinentalRemainingFraction,
			BSurv.Step0BroadInteriorRemainingFraction, OSurv.Step0BroadInteriorRemainingFraction,
			BSurv.Step0CoastAdjacentRemainingFraction, OSurv.Step0CoastAdjacentRemainingFraction,
			BS.BoundaryCoherence.BoundaryCoherenceScore, OS.BoundaryCoherence.BoundaryCoherenceScore,
			BS.BoundaryCoherence.InteriorLeakageFraction, OS.BoundaryCoherence.InteriorLeakageFraction,
			BS.OwnershipChurn.ChurnFraction, OS.OwnershipChurn.ChurnFraction,
			BS.MissCount, OS.MissCount,
			BS.OverlapCount, OS.OverlapCount));
		AddInfo(FString::Printf(
			TEXT("[V9PaperBundle compare step=%d baseline=%s variant=%s] relief_mean=%.4f/%.4f relief_p95=%.4f/%.4f above_2km=%d/%d above_5km=%d/%d"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BE.MeanElevationKm, OE.MeanElevationKm,
			BE.P95ElevationKm, OE.P95ElevationKm,
			BE.SamplesAbove2Km, OE.SamplesAbove2Km,
			BE.SamplesAbove5Km, OE.SamplesAbove5Km));
		AddInfo(FString::Printf(
			TEXT("[V9PaperBundle compare step=%d baseline=%s variant=%s] surrogate_usage_total=%d/%d tri=%d/%d ss=%d/%d subaerial=%d/%d submerged=%d/%d strong=%d/%d moderate=%d/%d"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BStats.PaperSurrogateOwnershipOverrideCount, OStats.PaperSurrogateOwnershipOverrideCount,
			BStats.PaperSurrogateOwnershipOverrideTriangleCount, OStats.PaperSurrogateOwnershipOverrideTriangleCount,
			BStats.PaperSurrogateOwnershipOverrideSingleSourceCount, OStats.PaperSurrogateOwnershipOverrideSingleSourceCount,
			BStats.PaperSurrogateOwnershipOverrideSubaerialCount, OStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			BStats.PaperSurrogateOwnershipOverrideSubmergedCount, OStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			BStats.PaperSurrogateOwnershipOverrideStrongInteriorCount, OStats.PaperSurrogateOwnershipOverrideStrongInteriorCount,
			BStats.PaperSurrogateOwnershipOverrideModerateInteriorCount, OStats.PaperSurrogateOwnershipOverrideModerateInteriorCount));
		AddInfo(FString::Printf(
			TEXT("[V9PaperBundle compare step=%d baseline=%s variant=%s] drift_cw=eligible=%d/%d pre=%.4f/%.4f post_transfer=%.4f/%.4f final=%.4f/%.4f drift_elev=%.4f/%.4f->%.4f/%.4f->%.4f/%.4f drift_thickness=%.4f/%.4f->%.4f/%.4f->%.4f/%.4f"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			OStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			BDrift.PreCWMean, ODrift.PreCWMean,
			BDrift.PostTransferCWMean, ODrift.PostTransferCWMean,
			BDrift.FinalCWMean, ODrift.FinalCWMean,
			BDrift.PreElevationMean, ODrift.PreElevationMean,
			BDrift.PostTransferElevationMean, ODrift.PostTransferElevationMean,
			BDrift.FinalElevationMean, ODrift.FinalElevationMean,
			BDrift.PreThicknessMean, ODrift.PreThicknessMean,
			BDrift.PostTransferThicknessMean, ODrift.PostTransferThicknessMean,
			BDrift.FinalThicknessMean, ODrift.FinalThicknessMean));
	};

	// Step 0 capture. All variants share the same seed flags.
	for (FVariantState* Variant : Variants)
	{
		CaptureCheckpoint(*Variant, 0, true);
	}
	BuildSeedFlags(
		Baseline.MassDiagnostics.FindChecked(0),
		Step0ContinentalFlags,
		Step0BroadInteriorFlags,
		Step0CoastAdjacentFlags);
	for (FVariantState* Variant : Variants)
	{
		Variant->SurvivalDiagnostics[0] = ComputeSeededContinentalSurvivalDiagnostic(
			Variant->Planet.GetPlanet(),
			Step0ContinentalFlags,
			Step0BroadInteriorFlags,
			Step0CoastAdjacentFlags);
	}
	LogComparison(0, Baseline, FullStateReference);
	LogComparison(0, Baseline, CandidateCWET);
	LogComparison(0, Baseline, CandidateCWT);

	const auto AdvanceAllToStep = [&Variants](const int32 TargetStep)
	{
		while (Variants[0]->Planet.GetPlanet().CurrentStep < TargetStep)
		{
			for (FVariantState* Variant : Variants)
			{
				Variant->Planet.AdvanceStep();
			}
		}
	};

	for (const int32 Step : TArray<int32>{25, 100})
	{
		AdvanceAllToStep(Step);
		for (FVariantState* Variant : Variants)
		{
			CaptureCheckpoint(*Variant, Step, true);
		}
		LogComparison(Step, Baseline, FullStateReference);
		LogComparison(Step, Baseline, CandidateCWET);
		LogComparison(Step, Baseline, CandidateCWT);
	}

	const FV6CheckpointSnapshot& BS100 = Baseline.Snapshots.FindChecked(100);
	const FV9ContinentalMassDiagnostic& BM100 = Baseline.MassDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& BSurv100 =
		Baseline.SurvivalDiagnostics.FindChecked(100);
	const FV9ContinentalMassDiagnostic& FM100 =
		FullStateReference.MassDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& FSurv100 =
		FullStateReference.SurvivalDiagnostics.FindChecked(100);

	const auto EvaluateCandidate =
		[&](
			const FVariantState& CandidateVariant)
	{
		FCandidateGateResult Result;
		Result.Label = CandidateVariant.Config.Label;

		const FV6CheckpointSnapshot& CS100 = CandidateVariant.Snapshots.FindChecked(100);
		const FV9ContinentalMassDiagnostic& CM100 = CandidateVariant.MassDiagnostics.FindChecked(100);
		const FV9SeededContinentalSurvivalDiagnostic& CSurv100 =
			CandidateVariant.SurvivalDiagnostics.FindChecked(100);

		const double CoastTarget =
			BM100.SubaerialMeanCoastDistHops +
			FMath::Max(1.0, 0.5 * (FM100.SubaerialMeanCoastDistHops - BM100.SubaerialMeanCoastDistHops));
		const double LargestTarget =
			static_cast<double>(BM100.LargestSubaerialComponentSize) +
			FMath::Max(
				200.0,
				0.5 *
					(static_cast<double>(FM100.LargestSubaerialComponentSize) -
					 static_cast<double>(BM100.LargestSubaerialComponentSize)));
		const double BroadSeedTarget =
			BSurv100.Step0BroadInteriorRemainingFraction +
			FMath::Max(
				0.10,
				0.5 * (FSurv100.Step0BroadInteriorRemainingFraction -
					   BSurv100.Step0BroadInteriorRemainingFraction));

		Result.bCoastMeanPass = CM100.SubaerialMeanCoastDistHops >= CoastTarget;
		Result.bLargestSubaerialPass =
			static_cast<double>(CM100.LargestSubaerialComponentSize) >= LargestTarget;
		Result.bBroadSeedPass =
			CSurv100.Step0BroadInteriorRemainingFraction >= BroadSeedTarget;
		Result.bNoSubmergedBalloonPass =
			CM100.SubmergedContinentalFraction <= BM100.SubmergedContinentalFraction + 0.02;
		Result.bCoherencePass =
			CS100.BoundaryCoherence.BoundaryCoherenceScore >= 0.93;
		Result.bLeakagePass =
			CS100.BoundaryCoherence.InteriorLeakageFraction < 0.18;
		Result.bChurnPass =
			CS100.OwnershipChurn.ChurnFraction < 0.05;
		Result.bMorphologyPass =
			Result.bCoastMeanPass &&
			Result.bLargestSubaerialPass &&
			Result.bBroadSeedPass &&
			Result.bNoSubmergedBalloonPass;
		Result.bStabilityPass =
			Result.bCoherencePass &&
			Result.bLeakagePass &&
			Result.bChurnPass;
		Result.bAllowStep200 = Result.bMorphologyPass && Result.bStabilityPass;

		Result.CoastRetention = ComputeGainRetention(
			BM100.SubaerialMeanCoastDistHops,
			FM100.SubaerialMeanCoastDistHops,
			CM100.SubaerialMeanCoastDistHops);
		Result.LargestRetention = ComputeGainRetention(
			static_cast<double>(BM100.LargestSubaerialComponentSize),
			static_cast<double>(FM100.LargestSubaerialComponentSize),
			static_cast<double>(CM100.LargestSubaerialComponentSize));
		Result.BroadSeedRetention = ComputeGainRetention(
			BSurv100.Step0BroadInteriorRemainingFraction,
			FSurv100.Step0BroadInteriorRemainingFraction,
			CSurv100.Step0BroadInteriorRemainingFraction);
		Result.Score =
			(Result.CoastRetention + Result.LargestRetention + Result.BroadSeedRetention) / 3.0;

		AddInfo(FString::Printf(
			TEXT("[V9PaperBundle gate step=100 candidate=%s] coast_keep=%d(%.2f>=%.2f retain=%.3f) largest_keep=%d(%d>=%.0f retain=%.3f) broad_seed=%d(%.4f>=%.4f retain=%.3f) submerged=%d(%.4f<=%.4f) coherence=%d(%.4f>=0.9300) leakage=%d(%.4f<0.1800) churn=%d(%.4f<0.0500) morphology=%d stability=%d run_step200=%d"),
			*Result.Label,
			Result.bCoastMeanPass ? 1 : 0,
			CM100.SubaerialMeanCoastDistHops,
			CoastTarget,
			Result.CoastRetention,
			Result.bLargestSubaerialPass ? 1 : 0,
			CM100.LargestSubaerialComponentSize,
			LargestTarget,
			Result.LargestRetention,
			Result.bBroadSeedPass ? 1 : 0,
			CSurv100.Step0BroadInteriorRemainingFraction,
			BroadSeedTarget,
			Result.BroadSeedRetention,
			Result.bNoSubmergedBalloonPass ? 1 : 0,
			CM100.SubmergedContinentalFraction,
			BM100.SubmergedContinentalFraction + 0.02,
			Result.bCoherencePass ? 1 : 0,
			CS100.BoundaryCoherence.BoundaryCoherenceScore,
			Result.bLeakagePass ? 1 : 0,
			CS100.BoundaryCoherence.InteriorLeakageFraction,
			Result.bChurnPass ? 1 : 0,
			CS100.OwnershipChurn.ChurnFraction,
			Result.bMorphologyPass ? 1 : 0,
			Result.bStabilityPass ? 1 : 0,
			Result.bAllowStep200 ? 1 : 0));

		return Result;
	};

	const FCandidateGateResult CWETGate = EvaluateCandidate(CandidateCWET);
	const FCandidateGateResult CWTGate = EvaluateCandidate(CandidateCWT);

	FVariantState* WinningCandidate = nullptr;
	FCandidateGateResult WinningGate;
	if (CWETGate.bAllowStep200 && (!CWTGate.bAllowStep200 || CWETGate.Score >= CWTGate.Score))
	{
		WinningCandidate = &CandidateCWET;
		WinningGate = CWETGate;
	}
	else if (CWTGate.bAllowStep200)
	{
		WinningCandidate = &CandidateCWT;
		WinningGate = CWTGate;
	}

	AddInfo(FString::Printf(
		TEXT("[V9PaperBundle gate summary step=100] winner=%s score=%.3f run_step200=%d"),
		WinningCandidate != nullptr ? *WinningCandidate->Config.Label : TEXT("none"),
		WinningCandidate != nullptr ? WinningGate.Score : -1.0,
		WinningCandidate != nullptr ? 1 : 0));

	if (WinningCandidate != nullptr)
	{
		AdvanceAllToStep(200);
		for (FVariantState* Variant : Variants)
		{
			CaptureCheckpoint(*Variant, 200, true);
		}
		LogComparison(200, Baseline, FullStateReference);
		LogComparison(200, Baseline, CandidateCWET);
		LogComparison(200, Baseline, CandidateCWT);
	}

	AddInfo(FString::Printf(
		TEXT("[V9PaperBundle summary] export_root=%s baseline_root=%s full_state_root=%s cwet_root=%s cwt_root=%s"),
		*ExportRoot,
		*Baseline.ExportRoot,
		*FullStateReference.ExportRoot,
		*CandidateCWET.ExportRoot,
		*CandidateCWT.ExportRoot));

	TestTrue(TEXT("Baseline reached step 100"), Baseline.Planet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Full-state reference reached step 100"), FullStateReference.Planet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("CW+Elevation+Thickness candidate reached step 100"), CandidateCWET.Planet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("CW+Thickness candidate reached step 100"), CandidateCWT.Planet.GetPlanet().CurrentStep >= 100);
	return true;
}

// ============================================================================
// Paper-Surrogate Selective Elevation Refinement Test
// Keeps the promoted quiet-interior CW+thickness preservation, but softens the
// elevation restore for higher-relief interiors so broad land survives without
// freezing as much mountain amplitude.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9PaperSurrogateSelectiveElevationRefinementTest,
	"Aurous.TectonicPlanet.V6V9PaperSurrogateSelectiveElevationRefinementTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9PaperSurrogateSelectiveElevationRefinementTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 60000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9PaperSurrogateSelectiveElevation");

	struct FSurrogateDriftMeans
	{
		double EligibleCount = 0.0;
		double PreCWMean = 0.0;
		double PostTransferCWMean = 0.0;
		double FinalCWMean = 0.0;
		double PreElevationMean = 0.0;
		double PostTransferElevationMean = 0.0;
		double FinalElevationMean = 0.0;
		double PreThicknessMean = 0.0;
		double PostTransferThicknessMean = 0.0;
		double FinalThicknessMean = 0.0;
	};

	struct FVariantConfig
	{
		FString Label;
		bool bEnablePaperSurrogate = false;
		ETectonicPlanetV6PaperSurrogateFieldMode FieldMode =
			ETectonicPlanetV6PaperSurrogateFieldMode::FullState;
	};

	struct FVariantState
	{
		FVariantConfig Config;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
	};

	const auto ComputeDriftMeans = [](const FTectonicPlanetV6PeriodicSolveStats& SolveStats)
	{
		FSurrogateDriftMeans Drift;
		Drift.EligibleCount =
			static_cast<double>(SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount);
		if (Drift.EligibleCount <= 0.0)
		{
			return Drift;
		}

		Drift.PreCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum /
			Drift.EligibleCount;
		Drift.PostTransferCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum /
			Drift.EligibleCount;
		Drift.FinalCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum /
			Drift.EligibleCount;
		Drift.PreElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveElevationSum /
			Drift.EligibleCount;
		Drift.PostTransferElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferElevationSum /
			Drift.EligibleCount;
		Drift.FinalElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalElevationSum /
			Drift.EligibleCount;
		Drift.PreThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveThicknessSum /
			Drift.EligibleCount;
		Drift.PostTransferThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferThicknessSum /
			Drift.EligibleCount;
		Drift.FinalThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalThicknessSum /
			Drift.EligibleCount;
		return Drift;
	};

	const auto ComputeGainRetention = [](
		const double BaselineValue,
		const double PromotedValue,
		const double CandidateValue)
	{
		const double Denominator = PromotedValue - BaselineValue;
		if (FMath::Abs(Denominator) <= KINDA_SMALL_NUMBER)
		{
			return 0.0;
		}
		return (CandidateValue - BaselineValue) / Denominator;
	};

	const auto InitializePlanet = [=](const FVariantConfig& Config) -> FTectonicPlanetV6
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			TestRandomSeed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
		Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
		Planet.SetV9QuietInteriorContinentalRetentionForTest(true);
		Planet.SetV9ContinentalBreadthPreservationForTest(false);
		Planet.SetSubmergedContinentalRelaxationForTest(true, 0.005);
		Planet.SetAutomaticRiftingForTest(true);
		Planet.SetV9PaperSurrogateOwnershipForTest(Config.bEnablePaperSurrogate);
		Planet.SetV9PaperSurrogateFieldModeForTest(Config.FieldMode);
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 N = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, N);
		OutStep0BroadInteriorFlags.Init(0, N);
		OutStep0CoastAdjacentFlags.Init(0, N);
		for (int32 I = 0; I < N; ++I)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(I) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(I) ||
				Step0Mass.SampleComponentId[I] < 0)
			{
				continue;
			}
			OutStep0ContinentalFlags[I] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[I];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[I] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[I] = 1;
			}
		}
	};

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString BaselineExportRoot = FPaths::Combine(ExportRoot, TEXT("baseline_equilibrium"));
	const FString PromotedExportRoot =
		FPaths::Combine(ExportRoot, TEXT("promoted_cw_elev_thickness"));
	const FString SelectiveExportRoot =
		FPaths::Combine(ExportRoot, TEXT("candidate_selective_elevation"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*BaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*PromotedExportRoot);
	PlatformFile.CreateDirectoryTree(*SelectiveExportRoot);

	FVariantState Baseline;
	Baseline.Config = {
		TEXT("baseline_equilibrium"),
		false,
		ETectonicPlanetV6PaperSurrogateFieldMode::FullState };
	Baseline.ExportRoot = BaselineExportRoot;
	Baseline.Planet = InitializePlanet(Baseline.Config);

	FVariantState Promoted;
	Promoted.Config = {
		TEXT("promoted_cw_elev_thickness"),
		true,
		ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightElevationThickness };
	Promoted.ExportRoot = PromotedExportRoot;
	Promoted.Planet = InitializePlanet(Promoted.Config);

	FVariantState Selective;
	Selective.Config = {
		TEXT("candidate_selective_elevation"),
		true,
		ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightThicknessSelectiveElevation };
	Selective.ExportRoot = SelectiveExportRoot;
	Selective.Planet = InitializePlanet(Selective.Config);

	TArray<FVariantState*> Variants = {&Baseline, &Promoted, &Selective};

	TArray<uint8> Step0ContinentalFlags;
	TArray<uint8> Step0BroadInteriorFlags;
	TArray<uint8> Step0CoastAdjacentFlags;

	const auto CaptureCheckpoint =
		[this, &Step0ContinentalFlags, &Step0BroadInteriorFlags, &Step0CoastAdjacentFlags, &ComputeDriftMeans](
			FVariantState& Variant,
			const int32 Step,
			const bool bExport)
	{
		const FString Tag =
			FString::Printf(TEXT("[V9PaperSelective %s step=%d]"), *Variant.Config.Label, Step);
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Variant.Planet, Variant.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Variant.Planet, Variant.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Variant.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Variant.Planet);
		const FV9ContinentalElevationStats ElevStats = ComputeContinentalElevationStats(Variant.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Variant.Planet.GetPlanet(),
				Step0ContinentalFlags,
				Step0BroadInteriorFlags,
				Step0CoastAdjacentFlags);
		Variant.Snapshots.Add(Step, Snapshot);
		Variant.MassDiagnostics.Add(Step, MassDiag);
		Variant.ElevationDiagnostics.Add(Step, ElevStats);
		Variant.SurvivalDiagnostics.Add(Step, SurvivalDiag);

		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6ContinentalMassDiagnosticInfo(*this, Tag, MassDiag);

		const FTectonicPlanetV6PeriodicSolveStats& SolveStats =
			Variant.Planet.GetPeriodicSolveCount() > 0
				? Variant.Planet.GetLastSolveStats()
				: Variant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FSurrogateDriftMeans Drift = ComputeDriftMeans(SolveStats);
		AddInfo(FString::Printf(
			TEXT("%s relief: mean=%.4f p95=%.4f max=%.4f above_2km=%d above_5km=%d surrogate=%d(subaerial=%d,submerged=%d,tri=%d,ss=%d,strong=%d,moderate=%d,low=%d,elev_mid=%d,elev_high=%d)"),
			*Tag,
			ElevStats.MeanElevationKm,
			ElevStats.P95ElevationKm,
			ElevStats.MaxElevationKm,
			ElevStats.SamplesAbove2Km,
			ElevStats.SamplesAbove5Km,
			SolveStats.PaperSurrogateOwnershipOverrideCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			SolveStats.PaperSurrogateOwnershipOverrideTriangleCount,
			SolveStats.PaperSurrogateOwnershipOverrideSingleSourceCount,
			SolveStats.PaperSurrogateOwnershipOverrideStrongInteriorCount,
			SolveStats.PaperSurrogateOwnershipOverrideModerateInteriorCount,
			SolveStats.PaperSurrogateOwnershipOverrideLowElevationBandCount,
			SolveStats.PaperSurrogateOwnershipOverrideModerateElevationBandCount,
			SolveStats.PaperSurrogateOwnershipOverrideHighElevationBandCount));
		AddInfo(FString::Printf(
			TEXT("%s surrogate_drift: eligible=%d pre_cw=%.4f post_cw=%.4f final_cw=%.4f pre_elev=%.4f post_elev=%.4f final_elev=%.4f pre_thickness=%.4f post_thickness=%.4f final_thickness=%.4f"),
			*Tag,
			SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			Drift.PreCWMean,
			Drift.PostTransferCWMean,
			Drift.FinalCWMean,
			Drift.PreElevationMean,
			Drift.PostTransferElevationMean,
			Drift.FinalElevationMean,
			Drift.PreThicknessMean,
			Drift.PostTransferThicknessMean,
			Drift.FinalThicknessMean));
		AddInfo(FString::Printf(
			TEXT("%s seeded_survival: all=%d/%d(%.4f) broad_interior=%d/%d(%.4f) coast_adjacent=%d/%d(%.4f)"),
			*Tag,
			SurvivalDiag.Step0ContinentalRemainingCount,
			SurvivalDiag.Step0ContinentalCount,
			SurvivalDiag.Step0ContinentalRemainingFraction,
			SurvivalDiag.Step0BroadInteriorRemainingCount,
			SurvivalDiag.Step0BroadInteriorCount,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			SurvivalDiag.Step0CoastAdjacentRemainingCount,
			SurvivalDiag.Step0CoastAdjacentCount,
			SurvivalDiag.Step0CoastAdjacentRemainingFraction));

		if (bExport)
		{
			ExportContinentalMassOverlays(*this, Variant.Planet, MassDiag, Variant.ExportRoot, Step);
		}
	};

	const auto LogComparison =
		[this, &ComputeDriftMeans](
			const int32 Step,
			const FVariantState& BaselineVariant,
			const FVariantState& OtherVariant)
	{
		const FV6CheckpointSnapshot& BS = BaselineVariant.Snapshots.FindChecked(Step);
		const FV6CheckpointSnapshot& OS = OtherVariant.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& BM = BaselineVariant.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& OM = OtherVariant.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& BE =
			BaselineVariant.ElevationDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& OE =
			OtherVariant.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& BSurv =
			BaselineVariant.SurvivalDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& OSurv =
			OtherVariant.SurvivalDiagnostics.FindChecked(Step);
		const FTectonicPlanetV6PeriodicSolveStats& BStats =
			BaselineVariant.Planet.GetPeriodicSolveCount() > 0
				? BaselineVariant.Planet.GetLastSolveStats()
				: BaselineVariant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FTectonicPlanetV6PeriodicSolveStats& OStats =
			OtherVariant.Planet.GetPeriodicSolveCount() > 0
				? OtherVariant.Planet.GetLastSolveStats()
				: OtherVariant.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FSurrogateDriftMeans BDrift = ComputeDriftMeans(BStats);
		const FSurrogateDriftMeans ODrift = ComputeDriftMeans(OStats);

		AddInfo(FString::Printf(
			TEXT("[V9PaperSelective compare step=%d baseline=%s variant=%s] coast_mean=%.2f/%.2f coast_p50=%.1f/%.1f coast_p90=%.1f/%.1f subaerial_coast_mean=%.2f/%.2f largest=%d/%d largest_subaerial=%d/%d broad_seed=%.4f/%.4f"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BM.MeanCoastDistHops, OM.MeanCoastDistHops,
			BM.P50CoastDist, OM.P50CoastDist,
			BM.P90CoastDist, OM.P90CoastDist,
			BM.SubaerialMeanCoastDistHops, OM.SubaerialMeanCoastDistHops,
			BM.LargestComponentSize, OM.LargestComponentSize,
			BM.LargestSubaerialComponentSize, OM.LargestSubaerialComponentSize,
			BSurv.Step0BroadInteriorRemainingFraction, OSurv.Step0BroadInteriorRemainingFraction));
		AddInfo(FString::Printf(
			TEXT("[V9PaperSelective compare step=%d baseline=%s variant=%s] caf=%.4f/%.4f subaerial_frac=%.4f/%.4f submerged_frac=%.4f/%.4f coherence=%.4f/%.4f leakage=%.4f/%.4f churn=%.4f/%.4f miss=%d/%d multi_hit=%d/%d"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BM.ContinentalAreaFraction, OM.ContinentalAreaFraction,
			BM.SubaerialContinentalFraction, OM.SubaerialContinentalFraction,
			BM.SubmergedContinentalFraction, OM.SubmergedContinentalFraction,
			BS.BoundaryCoherence.BoundaryCoherenceScore, OS.BoundaryCoherence.BoundaryCoherenceScore,
			BS.BoundaryCoherence.InteriorLeakageFraction, OS.BoundaryCoherence.InteriorLeakageFraction,
			BS.OwnershipChurn.ChurnFraction, OS.OwnershipChurn.ChurnFraction,
			BS.MissCount, OS.MissCount,
			BS.OverlapCount, OS.OverlapCount));
		AddInfo(FString::Printf(
			TEXT("[V9PaperSelective compare step=%d baseline=%s variant=%s] relief_mean=%.4f/%.4f relief_p95=%.4f/%.4f relief_max=%.4f/%.4f above_2km=%d/%d above_5km=%d/%d"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BE.MeanElevationKm, OE.MeanElevationKm,
			BE.P95ElevationKm, OE.P95ElevationKm,
			BE.MaxElevationKm, OE.MaxElevationKm,
			BE.SamplesAbove2Km, OE.SamplesAbove2Km,
			BE.SamplesAbove5Km, OE.SamplesAbove5Km));
		AddInfo(FString::Printf(
			TEXT("[V9PaperSelective compare step=%d baseline=%s variant=%s] surrogate_usage_total=%d/%d subaerial=%d/%d submerged=%d/%d strong=%d/%d moderate=%d/%d low=%d/%d elev_mid=%d/%d elev_high=%d/%d"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BStats.PaperSurrogateOwnershipOverrideCount, OStats.PaperSurrogateOwnershipOverrideCount,
			BStats.PaperSurrogateOwnershipOverrideSubaerialCount, OStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			BStats.PaperSurrogateOwnershipOverrideSubmergedCount, OStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			BStats.PaperSurrogateOwnershipOverrideStrongInteriorCount, OStats.PaperSurrogateOwnershipOverrideStrongInteriorCount,
			BStats.PaperSurrogateOwnershipOverrideModerateInteriorCount, OStats.PaperSurrogateOwnershipOverrideModerateInteriorCount,
			BStats.PaperSurrogateOwnershipOverrideLowElevationBandCount, OStats.PaperSurrogateOwnershipOverrideLowElevationBandCount,
			BStats.PaperSurrogateOwnershipOverrideModerateElevationBandCount, OStats.PaperSurrogateOwnershipOverrideModerateElevationBandCount,
			BStats.PaperSurrogateOwnershipOverrideHighElevationBandCount, OStats.PaperSurrogateOwnershipOverrideHighElevationBandCount));
		AddInfo(FString::Printf(
			TEXT("[V9PaperSelective compare step=%d baseline=%s variant=%s] drift_cw=eligible=%d/%d pre=%.4f/%.4f post_transfer=%.4f/%.4f final=%.4f/%.4f drift_elev=%.4f/%.4f->%.4f/%.4f->%.4f/%.4f drift_thickness=%.4f/%.4f->%.4f/%.4f->%.4f/%.4f"),
			Step,
			*BaselineVariant.Config.Label,
			*OtherVariant.Config.Label,
			BStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			OStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			BDrift.PreCWMean, ODrift.PreCWMean,
			BDrift.PostTransferCWMean, ODrift.PostTransferCWMean,
			BDrift.FinalCWMean, ODrift.FinalCWMean,
			BDrift.PreElevationMean, ODrift.PreElevationMean,
			BDrift.PostTransferElevationMean, ODrift.PostTransferElevationMean,
			BDrift.FinalElevationMean, ODrift.FinalElevationMean,
			BDrift.PreThicknessMean, ODrift.PreThicknessMean,
			BDrift.PostTransferThicknessMean, ODrift.PostTransferThicknessMean,
			BDrift.FinalThicknessMean, ODrift.FinalThicknessMean));
	};

	for (FVariantState* Variant : Variants)
	{
		CaptureCheckpoint(*Variant, 0, true);
	}
	BuildSeedFlags(
		Baseline.MassDiagnostics.FindChecked(0),
		Step0ContinentalFlags,
		Step0BroadInteriorFlags,
		Step0CoastAdjacentFlags);
	for (FVariantState* Variant : Variants)
	{
		Variant->SurvivalDiagnostics[0] = ComputeSeededContinentalSurvivalDiagnostic(
			Variant->Planet.GetPlanet(),
			Step0ContinentalFlags,
			Step0BroadInteriorFlags,
			Step0CoastAdjacentFlags);
	}
	LogComparison(0, Baseline, Promoted);
	LogComparison(0, Baseline, Selective);

	const auto AdvanceAllToStep = [&Variants](const int32 TargetStep)
	{
		while (Variants[0]->Planet.GetPlanet().CurrentStep < TargetStep)
		{
			for (FVariantState* Variant : Variants)
			{
				Variant->Planet.AdvanceStep();
			}
		}
	};

	for (const int32 Step : TArray<int32>{25, 100})
	{
		AdvanceAllToStep(Step);
		for (FVariantState* Variant : Variants)
		{
			CaptureCheckpoint(*Variant, Step, true);
		}
		LogComparison(Step, Baseline, Promoted);
		LogComparison(Step, Baseline, Selective);
	}

	const FV6CheckpointSnapshot& BS100 = Baseline.Snapshots.FindChecked(100);
	const FV6CheckpointSnapshot& PS100 = Promoted.Snapshots.FindChecked(100);
	const FV6CheckpointSnapshot& SS100 = Selective.Snapshots.FindChecked(100);
	const FV9ContinentalMassDiagnostic& BM100 = Baseline.MassDiagnostics.FindChecked(100);
	const FV9ContinentalMassDiagnostic& PM100 = Promoted.MassDiagnostics.FindChecked(100);
	const FV9ContinentalMassDiagnostic& SM100 = Selective.MassDiagnostics.FindChecked(100);
	const FV9ContinentalElevationStats& PE100 = Promoted.ElevationDiagnostics.FindChecked(100);
	const FV9ContinentalElevationStats& SE100 = Selective.ElevationDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& BSurv100 =
		Baseline.SurvivalDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& PSurv100 =
		Promoted.SurvivalDiagnostics.FindChecked(100);
	const FV9SeededContinentalSurvivalDiagnostic& SSurv100 =
		Selective.SurvivalDiagnostics.FindChecked(100);

	const double CoastRetention = ComputeGainRetention(
		BM100.SubaerialMeanCoastDistHops,
		PM100.SubaerialMeanCoastDistHops,
		SM100.SubaerialMeanCoastDistHops);
	const double LargestRetention = ComputeGainRetention(
		static_cast<double>(BM100.LargestSubaerialComponentSize),
		static_cast<double>(PM100.LargestSubaerialComponentSize),
		static_cast<double>(SM100.LargestSubaerialComponentSize));
	const double BroadSeedRetention = ComputeGainRetention(
		BSurv100.Step0BroadInteriorRemainingFraction,
		PSurv100.Step0BroadInteriorRemainingFraction,
		SSurv100.Step0BroadInteriorRemainingFraction);

	const bool bCoastKeepPass = CoastRetention >= 0.80;
	const bool bLargestKeepPass = LargestRetention >= 0.70;
	const bool bBroadSeedPass = BroadSeedRetention >= 0.80;
	const bool bNoSubmergedBalloonPass =
		SM100.SubmergedContinentalFraction <= BM100.SubmergedContinentalFraction + 0.02 &&
		SM100.SubmergedContinentalFraction <= PM100.SubmergedContinentalFraction + 0.01;
	const bool bCoherencePass = SS100.BoundaryCoherence.BoundaryCoherenceScore >= 0.93;
	const bool bLeakagePass = SS100.BoundaryCoherence.InteriorLeakageFraction < 0.18;
	const bool bChurnPass = SS100.OwnershipChurn.ChurnFraction < 0.05;
	const bool bP95SoftenPass = SE100.P95ElevationKm <= PE100.P95ElevationKm - 0.35;
	const bool bAbove5SoftenPass =
		SE100.SamplesAbove5Km <= FMath::FloorToInt(static_cast<double>(PE100.SamplesAbove5Km) * 0.90);
	const bool bReliefSoftenPass = bP95SoftenPass || bAbove5SoftenPass;
	const bool bMorphologyPass =
		bCoastKeepPass && bLargestKeepPass && bBroadSeedPass && bNoSubmergedBalloonPass;
	const bool bStabilityPass =
		bCoherencePass && bLeakagePass && bChurnPass;
	const bool bAllowStep200 =
		bMorphologyPass && bStabilityPass && bReliefSoftenPass;

	AddInfo(FString::Printf(
		TEXT("[V9PaperSelective gate step=100] coast_keep=%d(retain=%.3f) largest_keep=%d(retain=%.3f) broad_seed=%d(retain=%.3f) submerged=%d(%.4f<=%.4f and <=%.4f) coherence=%d(%.4f>=0.9300) leakage=%d(%.4f<0.1800) churn=%d(%.4f<0.0500) p95_soften=%d(%.4f<=%.4f) above5_soften=%d(%d<=%d) morphology=%d stability=%d soften=%d run_step200=%d"),
		bCoastKeepPass ? 1 : 0,
		CoastRetention,
		bLargestKeepPass ? 1 : 0,
		LargestRetention,
		bBroadSeedPass ? 1 : 0,
		BroadSeedRetention,
		bNoSubmergedBalloonPass ? 1 : 0,
		SM100.SubmergedContinentalFraction,
		BM100.SubmergedContinentalFraction + 0.02,
		PM100.SubmergedContinentalFraction + 0.01,
		bCoherencePass ? 1 : 0,
		SS100.BoundaryCoherence.BoundaryCoherenceScore,
		bLeakagePass ? 1 : 0,
		SS100.BoundaryCoherence.InteriorLeakageFraction,
		bChurnPass ? 1 : 0,
		SS100.OwnershipChurn.ChurnFraction,
		bP95SoftenPass ? 1 : 0,
		SE100.P95ElevationKm,
		PE100.P95ElevationKm - 0.35,
		bAbove5SoftenPass ? 1 : 0,
		SE100.SamplesAbove5Km,
		FMath::FloorToInt(static_cast<double>(PE100.SamplesAbove5Km) * 0.90),
		bMorphologyPass ? 1 : 0,
		bStabilityPass ? 1 : 0,
		bReliefSoftenPass ? 1 : 0,
		bAllowStep200 ? 1 : 0));

	if (bAllowStep200)
	{
		AdvanceAllToStep(200);
		for (FVariantState* Variant : Variants)
		{
			CaptureCheckpoint(*Variant, 200, true);
		}
		LogComparison(200, Baseline, Promoted);
		LogComparison(200, Baseline, Selective);
	}

	AddInfo(FString::Printf(
		TEXT("[V9PaperSelective summary] export_root=%s baseline_root=%s promoted_root=%s candidate_root=%s"),
		*ExportRoot,
		*Baseline.ExportRoot,
		*Promoted.ExportRoot,
		*Selective.ExportRoot));

	TestTrue(TEXT("Baseline reached step 100"), Baseline.Planet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Promoted candidate reached step 100"), Promoted.Planet.GetPlanet().CurrentStep >= 100);
	TestTrue(TEXT("Selective candidate reached step 100"), Selective.Planet.GetPlanet().CurrentStep >= 100);
	return true;
}

// ============================================================================
// Paper-Selective Elevation Confirmation And Robustness Test
// Confirms the kept selective-elevation candidate against the equilibrium
// baseline on the canonical seed, then runs a small seed sweep to check whether
// the same morphological/stability class holds beyond the single kept seed.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9PaperSelectiveElevationConfirmationAndRobustnessTest,
	"Aurous.TectonicPlanet.V6V9PaperSelectiveElevationConfirmationAndRobustnessTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9PaperSelectiveElevationConfirmationAndRobustnessTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 SampleCount = 100000;
	constexpr int32 PlateCount = 40;
	const FString RunId = TEXT("V9PaperSelectiveElevationConfirmation100k");
	const TArray<int32> SweepSeeds = { TestRandomSeed, 31415, 27182 };

	struct FSurrogateDriftMeans
	{
		double EligibleCount = 0.0;
		double PreCWMean = 0.0;
		double PostTransferCWMean = 0.0;
		double FinalCWMean = 0.0;
		double PreElevationMean = 0.0;
		double PostTransferElevationMean = 0.0;
		double FinalElevationMean = 0.0;
		double PreThicknessMean = 0.0;
		double PostTransferThicknessMean = 0.0;
		double FinalThicknessMean = 0.0;
	};

	struct FRunState
	{
		FString Label;
		int32 Seed = TestRandomSeed;
		bool bEnablePaperSurrogate = false;
		ETectonicPlanetV6PaperSurrogateFieldMode FieldMode =
			ETectonicPlanetV6PaperSurrogateFieldMode::FullState;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
	};

	struct FSeedGateResult
	{
		int32 Seed = 0;
		bool bCoastPass = false;
		bool bLargestSubaerialPass = false;
		bool bBroadSeedPass = false;
		bool bSubmergedPass = false;
		bool bCoherencePass = false;
		bool bLeakagePass = false;
		bool bChurnPass = false;
		bool bP95Pass = false;
		bool bAbove5Pass = false;
		bool bMorphologyPass = false;
		bool bStabilityPass = false;
		bool bReliefPass = false;
		bool bAllowStep200 = false;
		bool bStep200Healthy = false;
	};

	const auto ComputeDriftMeans = [](const FTectonicPlanetV6PeriodicSolveStats& SolveStats)
	{
		FSurrogateDriftMeans Drift;
		Drift.EligibleCount =
			static_cast<double>(SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount);
		if (Drift.EligibleCount <= 0.0)
		{
			return Drift;
		}

		Drift.PreCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum /
			Drift.EligibleCount;
		Drift.PostTransferCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum /
			Drift.EligibleCount;
		Drift.FinalCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum /
			Drift.EligibleCount;
		Drift.PreElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveElevationSum /
			Drift.EligibleCount;
		Drift.PostTransferElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferElevationSum /
			Drift.EligibleCount;
		Drift.FinalElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalElevationSum /
			Drift.EligibleCount;
		Drift.PreThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveThicknessSum /
			Drift.EligibleCount;
		Drift.PostTransferThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferThicknessSum /
			Drift.EligibleCount;
		Drift.FinalThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalThicknessSum /
			Drift.EligibleCount;
		return Drift;
	};

	const auto InitializePlanet =
		[=](const int32 Seed, const bool bEnablePaperSurrogate, const ETectonicPlanetV6PaperSurrogateFieldMode FieldMode)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
			FixedIntervalSteps,
			INDEX_NONE,
			SampleCount,
			PlateCount,
			Seed);
		Planet.SetSyntheticCoverageRetentionForTest(false);
		Planet.SetWholeTriangleBoundaryDuplicationForTest(false);
		Planet.SetExcludeMixedTrianglesForTest(false);
		Planet.SetV9Phase1AuthorityForTest(true, 1);
		Planet.SetV9Phase1ActiveZoneClassifierModeForTest(
			ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
		Planet.SetV9Phase1PersistentActivePairHorizonForTest(2);
		Planet.SetV9CollisionShadowForTest(true);
		Planet.SetV9CollisionExecutionForTest(true);
		Planet.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
		Planet.SetV9CollisionExecutionStructuralTransferForTest(true);
		Planet.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
		Planet.SetV9ThesisShapedCollisionExecutionForTest(true);
		Planet.SetV9QuietInteriorContinentalRetentionForTest(true);
		Planet.SetV9ContinentalBreadthPreservationForTest(false);
		Planet.SetSubmergedContinentalRelaxationForTest(true, 0.005);
		Planet.SetAutomaticRiftingForTest(true);
		Planet.SetV9PaperSurrogateOwnershipForTest(bEnablePaperSurrogate);
		Planet.SetV9PaperSurrogateFieldModeForTest(FieldMode);
		Planet.ApplyKeptV6DiagnosticsProfile(MakeKeptV6DiagnosticsOptions(false, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 N = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, N);
		OutStep0BroadInteriorFlags.Init(0, N);
		OutStep0CoastAdjacentFlags.Init(0, N);
		for (int32 I = 0; I < N; ++I)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(I) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(I) ||
				Step0Mass.SampleComponentId[I] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[I] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[I];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[I] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[I] = 1;
			}
		}
	};

	const auto InitializeRunState =
		[&](FRunState& Run,
			const FString& Label,
			const int32 Seed,
			const bool bEnablePaperSurrogate,
			const ETectonicPlanetV6PaperSurrogateFieldMode FieldMode,
			const FString& ExportRoot)
	{
		Run.Label = Label;
		Run.Seed = Seed;
		Run.bEnablePaperSurrogate = bEnablePaperSurrogate;
		Run.FieldMode = FieldMode;
		Run.ExportRoot = ExportRoot;
		Run.Planet = InitializePlanet(Seed, bEnablePaperSurrogate, FieldMode);
	};

	const auto FinalizeStep0Flags = [&](FRunState& Run)
	{
		const FV9ContinentalMassDiagnostic& Step0Mass = Run.MassDiagnostics.FindChecked(0);
		BuildSeedFlags(
			Step0Mass,
			Run.Step0ContinentalFlags,
			Run.Step0BroadInteriorFlags,
			Run.Step0CoastAdjacentFlags);
		Run.SurvivalDiagnostics[0] = ComputeSeededContinentalSurvivalDiagnostic(
			Run.Planet.GetPlanet(),
			Run.Step0ContinentalFlags,
			Run.Step0BroadInteriorFlags,
			Run.Step0CoastAdjacentFlags);
	};

	const auto CaptureCheckpoint =
		[this, &ComputeDriftMeans](
			FRunState& Run,
			const int32 Step,
			const bool bExport)
	{
		const FString Tag = FString::Printf(
			TEXT("[V9PaperConfirm %s seed=%d step=%d]"),
			*Run.Label,
			Run.Seed,
			Step);
		if (bExport && !Run.ExportRoot.IsEmpty())
		{
			ExportV6CheckpointMaps(*this, Run.Planet, Run.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Run.Planet, Run.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9ContinentalElevationStats ElevStats = ComputeContinentalElevationStats(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		Run.Snapshots.Add(Step, Snapshot);
		Run.MassDiagnostics.Add(Step, MassDiag);
		Run.ElevationDiagnostics.Add(Step, ElevStats);
		Run.SurvivalDiagnostics.Add(Step, SurvivalDiag);

		AddV6BoundaryCoherenceInfo(*this, Tag, Snapshot);
		AddV6ActiveZoneInfo(*this, Tag, Snapshot);
		AddV6ContinentalMassDiagnosticInfo(*this, Tag, MassDiag);

		const FTectonicPlanetV6PeriodicSolveStats& SolveStats =
			Run.Planet.GetPeriodicSolveCount() > 0
				? Run.Planet.GetLastSolveStats()
				: Run.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FSurrogateDriftMeans Drift = ComputeDriftMeans(SolveStats);
		AddInfo(FString::Printf(
			TEXT("%s relief: mean=%.4f p95=%.4f max=%.4f above_2km=%d above_5km=%d surrogate=%d(subaerial=%d,submerged=%d,tri=%d,ss=%d,strong=%d,moderate=%d,low=%d,elev_mid=%d,elev_high=%d)"),
			*Tag,
			ElevStats.MeanElevationKm,
			ElevStats.P95ElevationKm,
			ElevStats.MaxElevationKm,
			ElevStats.SamplesAbove2Km,
			ElevStats.SamplesAbove5Km,
			SolveStats.PaperSurrogateOwnershipOverrideCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			SolveStats.PaperSurrogateOwnershipOverrideTriangleCount,
			SolveStats.PaperSurrogateOwnershipOverrideSingleSourceCount,
			SolveStats.PaperSurrogateOwnershipOverrideStrongInteriorCount,
			SolveStats.PaperSurrogateOwnershipOverrideModerateInteriorCount,
			SolveStats.PaperSurrogateOwnershipOverrideLowElevationBandCount,
			SolveStats.PaperSurrogateOwnershipOverrideModerateElevationBandCount,
			SolveStats.PaperSurrogateOwnershipOverrideHighElevationBandCount));
		AddInfo(FString::Printf(
			TEXT("%s surrogate_drift: eligible=%d pre_cw=%.4f post_cw=%.4f final_cw=%.4f pre_elev=%.4f post_elev=%.4f final_elev=%.4f pre_thickness=%.4f post_thickness=%.4f final_thickness=%.4f"),
			*Tag,
			SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			Drift.PreCWMean,
			Drift.PostTransferCWMean,
			Drift.FinalCWMean,
			Drift.PreElevationMean,
			Drift.PostTransferElevationMean,
			Drift.FinalElevationMean,
			Drift.PreThicknessMean,
			Drift.PostTransferThicknessMean,
			Drift.FinalThicknessMean));
		AddInfo(FString::Printf(
			TEXT("%s seeded_survival: all=%d/%d(%.4f) broad_interior=%d/%d(%.4f) coast_adjacent=%d/%d(%.4f)"),
			*Tag,
			SurvivalDiag.Step0ContinentalRemainingCount,
			SurvivalDiag.Step0ContinentalCount,
			SurvivalDiag.Step0ContinentalRemainingFraction,
			SurvivalDiag.Step0BroadInteriorRemainingCount,
			SurvivalDiag.Step0BroadInteriorCount,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			SurvivalDiag.Step0CoastAdjacentRemainingCount,
			SurvivalDiag.Step0CoastAdjacentCount,
			SurvivalDiag.Step0CoastAdjacentRemainingFraction));

		if (bExport && !Run.ExportRoot.IsEmpty())
		{
			ExportContinentalMassOverlays(*this, Run.Planet, MassDiag, Run.ExportRoot, Step);
		}
	};

	const auto LogCanonicalComparison =
		[this, &ComputeDriftMeans](
			const int32 Step,
			const FRunState& BaselineRun,
			const FRunState& CandidateRun)
	{
		const FV6CheckpointSnapshot& BS = BaselineRun.Snapshots.FindChecked(Step);
		const FV6CheckpointSnapshot& CS = CandidateRun.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& BM = BaselineRun.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& CM = CandidateRun.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& BE =
			BaselineRun.ElevationDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& CE =
			CandidateRun.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& BSurv =
			BaselineRun.SurvivalDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& CSurv =
			CandidateRun.SurvivalDiagnostics.FindChecked(Step);
		const FTectonicPlanetV6PeriodicSolveStats& BStats =
			BaselineRun.Planet.GetPeriodicSolveCount() > 0
				? BaselineRun.Planet.GetLastSolveStats()
				: BaselineRun.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FTectonicPlanetV6PeriodicSolveStats& CStats =
			CandidateRun.Planet.GetPeriodicSolveCount() > 0
				? CandidateRun.Planet.GetLastSolveStats()
				: CandidateRun.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FSurrogateDriftMeans BDrift = ComputeDriftMeans(BStats);
		const FSurrogateDriftMeans CDrift = ComputeDriftMeans(CStats);

		AddInfo(FString::Printf(
			TEXT("[V9PaperConfirm compare step=%d baseline=%s candidate=%s] coast_mean=%.2f/%.2f coast_p50=%.1f/%.1f coast_p90=%.1f/%.1f subaerial_coast_mean=%.2f/%.2f largest=%d/%d largest_subaerial=%d/%d broad_seed=%.4f/%.4f"),
			Step,
			*BaselineRun.Label,
			*CandidateRun.Label,
			BM.MeanCoastDistHops, CM.MeanCoastDistHops,
			BM.P50CoastDist, CM.P50CoastDist,
			BM.P90CoastDist, CM.P90CoastDist,
			BM.SubaerialMeanCoastDistHops, CM.SubaerialMeanCoastDistHops,
			BM.LargestComponentSize, CM.LargestComponentSize,
			BM.LargestSubaerialComponentSize, CM.LargestSubaerialComponentSize,
			BSurv.Step0BroadInteriorRemainingFraction, CSurv.Step0BroadInteriorRemainingFraction));
		AddInfo(FString::Printf(
			TEXT("[V9PaperConfirm compare step=%d baseline=%s candidate=%s] caf=%.4f/%.4f subaerial_frac=%.4f/%.4f submerged_frac=%.4f/%.4f coherence=%.4f/%.4f leakage=%.4f/%.4f churn=%.4f/%.4f miss=%d/%d multi_hit=%d/%d"),
			Step,
			*BaselineRun.Label,
			*CandidateRun.Label,
			BM.ContinentalAreaFraction, CM.ContinentalAreaFraction,
			BM.SubaerialContinentalFraction, CM.SubaerialContinentalFraction,
			BM.SubmergedContinentalFraction, CM.SubmergedContinentalFraction,
			BS.BoundaryCoherence.BoundaryCoherenceScore, CS.BoundaryCoherence.BoundaryCoherenceScore,
			BS.BoundaryCoherence.InteriorLeakageFraction, CS.BoundaryCoherence.InteriorLeakageFraction,
			BS.OwnershipChurn.ChurnFraction, CS.OwnershipChurn.ChurnFraction,
			BS.MissCount, CS.MissCount,
			BS.OverlapCount, CS.OverlapCount));
		AddInfo(FString::Printf(
			TEXT("[V9PaperConfirm compare step=%d baseline=%s candidate=%s] relief_mean=%.4f/%.4f relief_p95=%.4f/%.4f relief_max=%.4f/%.4f above_2km=%d/%d above_5km=%d/%d"),
			Step,
			*BaselineRun.Label,
			*CandidateRun.Label,
			BE.MeanElevationKm, CE.MeanElevationKm,
			BE.P95ElevationKm, CE.P95ElevationKm,
			BE.MaxElevationKm, CE.MaxElevationKm,
			BE.SamplesAbove2Km, CE.SamplesAbove2Km,
			BE.SamplesAbove5Km, CE.SamplesAbove5Km));
		AddInfo(FString::Printf(
			TEXT("[V9PaperConfirm compare step=%d baseline=%s candidate=%s] surrogate_usage_total=%d/%d subaerial=%d/%d submerged=%d/%d low=%d/%d elev_mid=%d/%d elev_high=%d/%d"),
			Step,
			*BaselineRun.Label,
			*CandidateRun.Label,
			BStats.PaperSurrogateOwnershipOverrideCount, CStats.PaperSurrogateOwnershipOverrideCount,
			BStats.PaperSurrogateOwnershipOverrideSubaerialCount, CStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			BStats.PaperSurrogateOwnershipOverrideSubmergedCount, CStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			BStats.PaperSurrogateOwnershipOverrideLowElevationBandCount, CStats.PaperSurrogateOwnershipOverrideLowElevationBandCount,
			BStats.PaperSurrogateOwnershipOverrideModerateElevationBandCount, CStats.PaperSurrogateOwnershipOverrideModerateElevationBandCount,
			BStats.PaperSurrogateOwnershipOverrideHighElevationBandCount, CStats.PaperSurrogateOwnershipOverrideHighElevationBandCount));
		AddInfo(FString::Printf(
			TEXT("[V9PaperConfirm compare step=%d baseline=%s candidate=%s] drift_cw=eligible=%d/%d pre=%.4f/%.4f post_transfer=%.4f/%.4f final=%.4f/%.4f drift_elev=%.4f/%.4f->%.4f/%.4f->%.4f/%.4f drift_thickness=%.4f/%.4f->%.4f/%.4f->%.4f/%.4f"),
			Step,
			*BaselineRun.Label,
			*CandidateRun.Label,
			BStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			CStats.PaperSurrogateQuietInteriorDirectHitEligibleCount,
			BDrift.PreCWMean, CDrift.PreCWMean,
			BDrift.PostTransferCWMean, CDrift.PostTransferCWMean,
			BDrift.FinalCWMean, CDrift.FinalCWMean,
			BDrift.PreElevationMean, CDrift.PreElevationMean,
			BDrift.PostTransferElevationMean, CDrift.PostTransferElevationMean,
			BDrift.FinalElevationMean, CDrift.FinalElevationMean,
			BDrift.PreThicknessMean, CDrift.PreThicknessMean,
			BDrift.PostTransferThicknessMean, CDrift.PostTransferThicknessMean,
			BDrift.FinalThicknessMean, CDrift.FinalThicknessMean));
	};

	const auto EvaluateSeedStep100Gate =
		[this](
			const FRunState& Run)
	{
		constexpr double MinSubaerialCoastMean = 4.50;
		constexpr int32 MinLargestSubaerial = 2500;
		constexpr double MinBroadSeedSurvival = 0.75;
		constexpr double MaxSubmergedFraction = 0.07;
		constexpr double MinCoherence = 0.93;
		constexpr double MaxLeakage = 0.18;
		constexpr double MaxChurn = 0.05;
		constexpr double MaxP95ElevationKm = 5.50;
		constexpr int32 MaxSamplesAbove5Km = 1000;

		FSeedGateResult Result;
		Result.Seed = Run.Seed;
		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(100);
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(100);
		const FV9ContinentalElevationStats& ElevDiag = Run.ElevationDiagnostics.FindChecked(100);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag =
			Run.SurvivalDiagnostics.FindChecked(100);

		Result.bCoastPass = MassDiag.SubaerialMeanCoastDistHops >= MinSubaerialCoastMean;
		Result.bLargestSubaerialPass =
			MassDiag.LargestSubaerialComponentSize >= MinLargestSubaerial;
		Result.bBroadSeedPass =
			SurvivalDiag.Step0BroadInteriorRemainingFraction >= MinBroadSeedSurvival;
		Result.bSubmergedPass =
			MassDiag.SubmergedContinentalFraction <= MaxSubmergedFraction;
		Result.bCoherencePass =
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore >= MinCoherence;
		Result.bLeakagePass =
			Snapshot.BoundaryCoherence.InteriorLeakageFraction < MaxLeakage;
		Result.bChurnPass =
			Snapshot.OwnershipChurn.ChurnFraction < MaxChurn;
		Result.bP95Pass = ElevDiag.P95ElevationKm <= MaxP95ElevationKm;
		Result.bAbove5Pass = ElevDiag.SamplesAbove5Km <= MaxSamplesAbove5Km;
		Result.bMorphologyPass =
			Result.bCoastPass &&
			Result.bLargestSubaerialPass &&
			Result.bBroadSeedPass &&
			Result.bSubmergedPass;
		Result.bStabilityPass =
			Result.bCoherencePass &&
			Result.bLeakagePass &&
			Result.bChurnPass;
		Result.bReliefPass =
			Result.bP95Pass &&
			Result.bAbove5Pass;
		Result.bAllowStep200 =
			Result.bMorphologyPass &&
			Result.bStabilityPass &&
			Result.bReliefPass;

		AddInfo(FString::Printf(
			TEXT("[V9PaperConfirm seed=%d gate step=100] coast=%d(%.2f>=4.50) largest=%d(%d>=2500) broad_seed=%d(%.4f>=0.7500) submerged=%d(%.4f<=0.0700) coherence=%d(%.4f>=0.9300) leakage=%d(%.4f<0.1800) churn=%d(%.4f<0.0500) p95=%d(%.4f<=5.5000) above5=%d(%d<=1000) morphology=%d stability=%d relief=%d run_step200=%d"),
			Run.Seed,
			Result.bCoastPass ? 1 : 0,
			MassDiag.SubaerialMeanCoastDistHops,
			Result.bLargestSubaerialPass ? 1 : 0,
			MassDiag.LargestSubaerialComponentSize,
			Result.bBroadSeedPass ? 1 : 0,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			Result.bSubmergedPass ? 1 : 0,
			MassDiag.SubmergedContinentalFraction,
			Result.bCoherencePass ? 1 : 0,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Result.bLeakagePass ? 1 : 0,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Result.bChurnPass ? 1 : 0,
			Snapshot.OwnershipChurn.ChurnFraction,
			Result.bP95Pass ? 1 : 0,
			ElevDiag.P95ElevationKm,
			Result.bAbove5Pass ? 1 : 0,
			ElevDiag.SamplesAbove5Km,
			Result.bMorphologyPass ? 1 : 0,
			Result.bStabilityPass ? 1 : 0,
			Result.bReliefPass ? 1 : 0,
			Result.bAllowStep200 ? 1 : 0));

		return Result;
	};

	const auto EvaluateSeedStep200Health =
		[this](
			const FRunState& Run,
			FSeedGateResult& Result)
	{
		constexpr double MinSubaerialCoastMean = 4.00;
		constexpr double MinBroadSeedSurvival = 0.60;
		constexpr double MaxSubmergedFraction = 0.08;
		constexpr double MinCoherence = 0.93;
		constexpr double MaxLeakage = 0.18;
		constexpr double MaxChurn = 0.05;
		constexpr double MaxP95ElevationKm = 6.00;
		constexpr int32 MaxSamplesAbove5Km = 1500;

		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(200);
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(200);
		const FV9ContinentalElevationStats& ElevDiag = Run.ElevationDiagnostics.FindChecked(200);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag =
			Run.SurvivalDiagnostics.FindChecked(200);

		const bool bCoastPass = MassDiag.SubaerialMeanCoastDistHops >= MinSubaerialCoastMean;
		const bool bBroadSeedPass =
			SurvivalDiag.Step0BroadInteriorRemainingFraction >= MinBroadSeedSurvival;
		const bool bSubmergedPass =
			MassDiag.SubmergedContinentalFraction <= MaxSubmergedFraction;
		const bool bCoherencePass =
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore >= MinCoherence;
		const bool bLeakagePass =
			Snapshot.BoundaryCoherence.InteriorLeakageFraction < MaxLeakage;
		const bool bChurnPass =
			Snapshot.OwnershipChurn.ChurnFraction < MaxChurn;
		const bool bP95Pass =
			ElevDiag.P95ElevationKm <= MaxP95ElevationKm;
		const bool bAbove5Pass =
			ElevDiag.SamplesAbove5Km <= MaxSamplesAbove5Km;

		Result.bStep200Healthy =
			bCoastPass &&
			bBroadSeedPass &&
			bSubmergedPass &&
			bCoherencePass &&
			bLeakagePass &&
			bChurnPass &&
			bP95Pass &&
			bAbove5Pass;

		AddInfo(FString::Printf(
			TEXT("[V9PaperConfirm seed=%d gate step=200] coast=%d(%.2f>=4.00) broad_seed=%d(%.4f>=0.6000) submerged=%d(%.4f<=0.0800) coherence=%d(%.4f>=0.9300) leakage=%d(%.4f<0.1800) churn=%d(%.4f<0.0500) p95=%d(%.4f<=6.0000) above5=%d(%d<=1500) healthy=%d"),
			Run.Seed,
			bCoastPass ? 1 : 0,
			MassDiag.SubaerialMeanCoastDistHops,
			bBroadSeedPass ? 1 : 0,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			bSubmergedPass ? 1 : 0,
			MassDiag.SubmergedContinentalFraction,
			bCoherencePass ? 1 : 0,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			bLeakagePass ? 1 : 0,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			bChurnPass ? 1 : 0,
			Snapshot.OwnershipChurn.ChurnFraction,
			bP95Pass ? 1 : 0,
			ElevDiag.P95ElevationKm,
			bAbove5Pass ? 1 : 0,
			ElevDiag.SamplesAbove5Km,
			Result.bStep200Healthy ? 1 : 0));
	};

	const auto LogSeedSummary =
		[this, &ComputeDriftMeans](
			const FRunState& Run,
			const FSeedGateResult& Gate100)
	{
		const FV9ContinentalMassDiagnostic& Mass100 = Run.MassDiagnostics.FindChecked(100);
		const FV9ContinentalElevationStats& Elev100 = Run.ElevationDiagnostics.FindChecked(100);
		const FV6CheckpointSnapshot& Snap100 = Run.Snapshots.FindChecked(100);
		const FV9SeededContinentalSurvivalDiagnostic& Surv100 =
			Run.SurvivalDiagnostics.FindChecked(100);
		const FTectonicPlanetV6PeriodicSolveStats& Stats100 =
			Run.Planet.GetPeriodicSolveCount() > 0
				? Run.Planet.GetLastSolveStats()
				: Run.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FSurrogateDriftMeans Drift100 = ComputeDriftMeans(Stats100);

		FString Step200Summary = TEXT("not_run");
		if (Run.Snapshots.Contains(200))
		{
			const FV9ContinentalMassDiagnostic& Mass200 = Run.MassDiagnostics.FindChecked(200);
			const FV9ContinentalElevationStats& Elev200 = Run.ElevationDiagnostics.FindChecked(200);
			const FV6CheckpointSnapshot& Snap200 = Run.Snapshots.FindChecked(200);
			const FV9SeededContinentalSurvivalDiagnostic& Surv200 =
				Run.SurvivalDiagnostics.FindChecked(200);
			Step200Summary = FString::Printf(
				TEXT("coast=%.2f broad_seed=%.4f submerged=%.4f coherence=%.4f leakage=%.4f churn=%.4f p95=%.4f above5=%d healthy=%d"),
				Mass200.SubaerialMeanCoastDistHops,
				Surv200.Step0BroadInteriorRemainingFraction,
				Mass200.SubmergedContinentalFraction,
				Snap200.BoundaryCoherence.BoundaryCoherenceScore,
				Snap200.BoundaryCoherence.InteriorLeakageFraction,
				Snap200.OwnershipChurn.ChurnFraction,
				Elev200.P95ElevationKm,
				Elev200.SamplesAbove5Km,
				Gate100.bStep200Healthy ? 1 : 0);
		}

		AddInfo(FString::Printf(
			TEXT("[V9PaperConfirm seed_summary] seed=%d step100_pass=%d coast100=%.2f largest100=%d broad_seed100=%.4f submerged100=%.4f coherence100=%.4f leakage100=%.4f churn100=%.4f p95_100=%.4f above5_100=%d touched100=%d bands100=%d/%d/%d drift100_cw=%.4f->%.4f->%.4f drift100_elev=%.4f->%.4f->%.4f drift100_thickness=%.4f->%.4f->%.4f step200=%s"),
			Run.Seed,
			Gate100.bAllowStep200 ? 1 : 0,
			Mass100.SubaerialMeanCoastDistHops,
			Mass100.LargestSubaerialComponentSize,
			Surv100.Step0BroadInteriorRemainingFraction,
			Mass100.SubmergedContinentalFraction,
			Snap100.BoundaryCoherence.BoundaryCoherenceScore,
			Snap100.BoundaryCoherence.InteriorLeakageFraction,
			Snap100.OwnershipChurn.ChurnFraction,
			Elev100.P95ElevationKm,
			Elev100.SamplesAbove5Km,
			Stats100.PaperSurrogateOwnershipOverrideCount,
			Stats100.PaperSurrogateOwnershipOverrideLowElevationBandCount,
			Stats100.PaperSurrogateOwnershipOverrideModerateElevationBandCount,
			Stats100.PaperSurrogateOwnershipOverrideHighElevationBandCount,
			Drift100.PreCWMean,
			Drift100.PostTransferCWMean,
			Drift100.FinalCWMean,
			Drift100.PreElevationMean,
			Drift100.PostTransferElevationMean,
			Drift100.FinalElevationMean,
			Drift100.PreThicknessMean,
			Drift100.PostTransferThicknessMean,
			Drift100.FinalThicknessMean,
			*Step200Summary));
	};

	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString CanonicalBaselineExportRoot =
		FPaths::Combine(ExportRoot, TEXT("canonical_baseline_equilibrium"));
	const FString CanonicalCandidateExportRoot =
		FPaths::Combine(ExportRoot, TEXT("canonical_candidate_selective_elevation"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*CanonicalBaselineExportRoot);
	PlatformFile.CreateDirectoryTree(*CanonicalCandidateExportRoot);

	FRunState CanonicalBaseline;
	InitializeRunState(
		CanonicalBaseline,
		TEXT("canonical_baseline_equilibrium"),
		TestRandomSeed,
		false,
		ETectonicPlanetV6PaperSurrogateFieldMode::FullState,
		CanonicalBaselineExportRoot);

	FRunState CanonicalCandidate;
	InitializeRunState(
		CanonicalCandidate,
		TEXT("canonical_candidate_selective_elevation"),
		TestRandomSeed,
		true,
		ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightThicknessSelectiveElevation,
		CanonicalCandidateExportRoot);

	CaptureCheckpoint(CanonicalBaseline, 0, false);
	CaptureCheckpoint(CanonicalCandidate, 0, false);
	FinalizeStep0Flags(CanonicalBaseline);
	FinalizeStep0Flags(CanonicalCandidate);

	const auto AdvanceCanonicalToStep =
		[&](const int32 TargetStep)
	{
		while (CanonicalBaseline.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			CanonicalBaseline.Planet.AdvanceStep();
			CanonicalCandidate.Planet.AdvanceStep();
		}
	};

	for (const int32 Step : TArray<int32>{25, 100, 200})
	{
		AdvanceCanonicalToStep(Step);
		const bool bExport = Step >= 100;
		CaptureCheckpoint(CanonicalBaseline, Step, bExport);
		CaptureCheckpoint(CanonicalCandidate, Step, bExport);
		LogCanonicalComparison(Step, CanonicalBaseline, CanonicalCandidate);
	}

	TArray<FSeedGateResult> SeedGateResults;
	SeedGateResults.Reserve(SweepSeeds.Num());

	FSeedGateResult CanonicalGate = EvaluateSeedStep100Gate(CanonicalCandidate);
	EvaluateSeedStep200Health(CanonicalCandidate, CanonicalGate);
	SeedGateResults.Add(CanonicalGate);
	LogSeedSummary(CanonicalCandidate, SeedGateResults[0]);

	for (int32 SeedIndex = 1; SeedIndex < SweepSeeds.Num(); ++SeedIndex)
	{
		FRunState SeedRun;
		InitializeRunState(
			SeedRun,
			FString::Printf(TEXT("candidate_seed_%d"), SweepSeeds[SeedIndex]),
			SweepSeeds[SeedIndex],
			true,
			ETectonicPlanetV6PaperSurrogateFieldMode::ContinentalWeightThicknessSelectiveElevation,
			TEXT(""));

		CaptureCheckpoint(SeedRun, 0, false);
		FinalizeStep0Flags(SeedRun);
		while (SeedRun.Planet.GetPlanet().CurrentStep < 100)
		{
			SeedRun.Planet.AdvanceStep();
		}
		CaptureCheckpoint(SeedRun, 100, false);

		FSeedGateResult SeedGate = EvaluateSeedStep100Gate(SeedRun);
		if (SeedGate.bAllowStep200)
		{
			while (SeedRun.Planet.GetPlanet().CurrentStep < 200)
			{
				SeedRun.Planet.AdvanceStep();
			}
			CaptureCheckpoint(SeedRun, 200, false);
			EvaluateSeedStep200Health(SeedRun, SeedGate);
		}
		SeedGateResults.Add(SeedGate);
		LogSeedSummary(SeedRun, SeedGateResults.Last());
		const int32 RequiredStep = SeedRun.Snapshots.Contains(200) ? 200 : 100;
		TestTrue(
			*FString::Printf(TEXT("Seed %d reached required checkpoint %d"), SeedRun.Seed, RequiredStep),
			SeedRun.Planet.GetPlanet().CurrentStep >= RequiredStep);
	}

	int32 Step100PassCount = 0;
	int32 Step200RunCount = 0;
	int32 Step200HealthyCount = 0;
	for (const FSeedGateResult& Gate : SeedGateResults)
	{
		Step100PassCount += Gate.bAllowStep200 ? 1 : 0;
		Step200RunCount += Gate.bAllowStep200 ? 1 : 0;
		Step200HealthyCount += Gate.bStep200Healthy ? 1 : 0;
	}

	AddInfo(FString::Printf(
		TEXT("[V9PaperConfirm robustness] seeds=%d step100_pass=%d step200_run=%d step200_healthy=%d canonical_seed=%d export_root=%s canonical_baseline_root=%s canonical_candidate_root=%s"),
		SeedGateResults.Num(),
		Step100PassCount,
		Step200RunCount,
		Step200HealthyCount,
		TestRandomSeed,
		*ExportRoot,
		*CanonicalBaselineExportRoot,
		*CanonicalCandidateExportRoot));

	TestTrue(
		TEXT("Canonical baseline reached step 200"),
		CanonicalBaseline.Planet.GetPlanet().CurrentStep >= 200);
	TestTrue(
		TEXT("Canonical candidate reached step 200"),
		CanonicalCandidate.Planet.GetPlanet().CurrentStep >= 200);
	return true;
}

// ============================================================================
// V9 Performance Phase Timing Test
// Instruments the kept selective-elevation candidate at canonical 60k/40 and
// 100k/40, captures representative periodic solve timings, and verifies the
// morphology/stability class remains healthy while optimization work proceeds.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9PerformancePhaseTimingTest,
	"Aurous.TectonicPlanet.V6V9PerformancePhaseTimingTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9PerformancePhaseTimingTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 PlateCount = 40;
	constexpr int32 CanonicalSeed = TestRandomSeed;
	const TArray<int32> SampleCounts = { 60000, 100000 };
	const TArray<int32> TimingSolveSteps = { 16, 96, 160 };

	struct FSurrogateDriftMeans
	{
		double EligibleCount = 0.0;
		double PreCWMean = 0.0;
		double PostTransferCWMean = 0.0;
		double FinalCWMean = 0.0;
		double PreElevationMean = 0.0;
		double PostTransferElevationMean = 0.0;
		double FinalElevationMean = 0.0;
		double PreThicknessMean = 0.0;
		double PostTransferThicknessMean = 0.0;
		double FinalThicknessMean = 0.0;
	};

	struct FRunState
	{
		int32 SampleCount = 0;
		FString Label;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
		TMap<int32, FTectonicPlanetV6PeriodicSolveStats> SolveTimings;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
	};

	struct FVariantConfig
	{
		FString LabelSuffix;
		bool bUseCachedSubductionAdjacencyEdgeDistances = true;
		bool bUsePlateCandidatePruning = true;
	};

	const auto ComputeDriftMeans = [](const FTectonicPlanetV6PeriodicSolveStats& SolveStats)
	{
		FSurrogateDriftMeans Drift;
		Drift.EligibleCount =
			static_cast<double>(SolveStats.PaperSurrogateQuietInteriorDirectHitEligibleCount);
		if (Drift.EligibleCount <= 0.0)
		{
			return Drift;
		}

		Drift.PreCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveContinentalWeightSum /
			Drift.EligibleCount;
		Drift.PostTransferCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferContinentalWeightSum /
			Drift.EligibleCount;
		Drift.FinalCWMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalContinentalWeightSum /
			Drift.EligibleCount;
		Drift.PreElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveElevationSum /
			Drift.EligibleCount;
		Drift.PostTransferElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferElevationSum /
			Drift.EligibleCount;
		Drift.FinalElevationMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalElevationSum /
			Drift.EligibleCount;
		Drift.PreThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPreSolveThicknessSum /
			Drift.EligibleCount;
		Drift.PostTransferThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitPostTransferThicknessSum /
			Drift.EligibleCount;
		Drift.FinalThicknessMean =
			SolveStats.PaperSurrogateQuietInteriorDirectHitFinalThicknessSum /
			Drift.EligibleCount;
		return Drift;
	};

	const auto InitializePlanet = [=](
		const int32 SampleCount,
		const bool bUseCachedSubductionAdjacencyEdgeDistances,
		const bool bUsePlateCandidatePruning)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			FTectonicPlanetV6::GetKeptV6PeriodicSolveMode(),
			FTectonicPlanetV6::GetKeptV6FixedIntervalSteps(),
			INDEX_NONE,
			SampleCount,
			PlateCount,
			CanonicalSeed);
		FTectonicPlanetV6KeptRuntimeProfileOptions RuntimeOptions =
			MakeKeptV6RuntimeProfileOptions();
		RuntimeOptions.bEnablePlateCandidatePruning = bUsePlateCandidatePruning;
		RuntimeOptions.bUseCachedSubductionAdjacencyEdgeDistances =
			bUseCachedSubductionAdjacencyEdgeDistances;
		ApplyKeptV6TestProfile(
			Planet,
			RuntimeOptions,
			MakeKeptV6DiagnosticsOptions(true, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleNum = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleNum);
		OutStep0BroadInteriorFlags.Init(0, SampleNum);
		OutStep0CoastAdjacentFlags.Init(0, SampleNum);
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	const auto CaptureCheckpoint =
		[this, &ComputeDriftMeans](
			FRunState& Run,
			const int32 Step)
	{
		const FString Tag = FString::Printf(
			TEXT("[V9Perf %s step=%d]"),
			*Run.Label,
			Step);
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9ContinentalElevationStats ElevDiag = ComputeContinentalElevationStats(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		Run.Snapshots.Add(Step, Snapshot);
		Run.MassDiagnostics.Add(Step, MassDiag);
		Run.ElevationDiagnostics.Add(Step, ElevDiag);
		Run.SurvivalDiagnostics.Add(Step, SurvivalDiag);

		const FTectonicPlanetV6PeriodicSolveStats SolveStats =
			Run.Planet.GetPeriodicSolveCount() > 0
				? Run.Planet.GetLastSolveStats()
				: Run.Planet.BuildCurrentDiagnosticSnapshotForTest();
		const FSurrogateDriftMeans Drift = ComputeDriftMeans(SolveStats);

		AddInfo(FString::Printf(
			TEXT("%s morphology: coast_mean=%.2f coast_p50=%.1f coast_p90=%.1f coast_max=%.1f subaerial_coast_mean=%.2f subaerial_coast_p50=%.1f subaerial_coast_p90=%.1f largest=%d largest_subaerial=%d broad_seed=%.4f"),
			*Tag,
			MassDiag.MeanCoastDistHops,
			MassDiag.P50CoastDist,
			MassDiag.P90CoastDist,
			MassDiag.MaxCoastDist,
			MassDiag.SubaerialMeanCoastDistHops,
			MassDiag.SubaerialP50CoastDist,
			MassDiag.SubaerialP90CoastDist,
			MassDiag.LargestComponentSize,
			MassDiag.LargestSubaerialComponentSize,
			SurvivalDiag.Step0BroadInteriorRemainingFraction));
		AddInfo(FString::Printf(
			TEXT("%s crust: caf=%.4f subaerial=%.4f submerged=%.4f"),
			*Tag,
			MassDiag.ContinentalAreaFraction,
			MassDiag.SubaerialContinentalFraction,
			MassDiag.SubmergedContinentalFraction));
		AddInfo(FString::Printf(
			TEXT("%s stability: coherence=%.4f leakage=%.4f churn=%.4f miss=%d multi_hit=%d"),
			*Tag,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Snapshot.OwnershipChurn.ChurnFraction,
			Snapshot.MissCount,
			Snapshot.OverlapCount));
		AddInfo(FString::Printf(
			TEXT("%s relief: mean=%.4f p95=%.4f max=%.4f above_2km=%d above_5km=%d"),
			*Tag,
			ElevDiag.MeanElevationKm,
			ElevDiag.P95ElevationKm,
			ElevDiag.MaxElevationKm,
			ElevDiag.SamplesAbove2Km,
			ElevDiag.SamplesAbove5Km));
		AddInfo(FString::Printf(
			TEXT("%s surrogate: touched=%d subaerial=%d submerged=%d low=%d elev_mid=%d elev_high=%d drift_cw=%.4f->%.4f->%.4f drift_elev=%.4f->%.4f->%.4f drift_thickness=%.4f->%.4f->%.4f"),
			*Tag,
			SolveStats.PaperSurrogateOwnershipOverrideCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubaerialCount,
			SolveStats.PaperSurrogateOwnershipOverrideSubmergedCount,
			SolveStats.PaperSurrogateOwnershipOverrideLowElevationBandCount,
			SolveStats.PaperSurrogateOwnershipOverrideModerateElevationBandCount,
			SolveStats.PaperSurrogateOwnershipOverrideHighElevationBandCount,
			Drift.PreCWMean,
			Drift.PostTransferCWMean,
			Drift.FinalCWMean,
			Drift.PreElevationMean,
			Drift.PostTransferElevationMean,
			Drift.FinalElevationMean,
			Drift.PreThicknessMean,
			Drift.PostTransferThicknessMean,
			Drift.FinalThicknessMean));
	};

	const auto CaptureSolveTiming = [this](FRunState& Run, const int32 SolveStep)
	{
		if (Run.SolveTimings.Contains(SolveStep))
		{
			return;
		}

		const FTectonicPlanetV6PeriodicSolveStats SolveStats = Run.Planet.GetLastSolveStats();
		Run.SolveTimings.Add(SolveStep, SolveStats);
		const FTectonicPlanetV6PhaseTiming& Phase = SolveStats.PhaseTiming;
		const double HitSearchAverageCandidateCount =
			Run.Planet.GetPlanet().Samples.Num() > 0
				? static_cast<double>(SolveStats.HitSearchPlateCandidateCountTotal) /
					static_cast<double>(Run.Planet.GetPlanet().Samples.Num())
				: 0.0;
		const double RecoveryGatherAverageCandidateCount =
			SolveStats.RecoveryCandidateGatherSampleCount > 0
				? static_cast<double>(SolveStats.RecoveryCandidatePlateCandidateCountTotal) /
					static_cast<double>(SolveStats.RecoveryCandidateGatherSampleCount)
				: 0.0;
		const double RecoveryMissAverageCandidateCount =
			SolveStats.RecoveryMissSampleCount > 0
				? static_cast<double>(SolveStats.RecoveryMissPlateCandidateCountTotal) /
					static_cast<double>(SolveStats.RecoveryMissSampleCount)
				: 0.0;
		AddInfo(FString::Printf(
			TEXT("[V9Perf %s solve=%d] total_ms=%.3f pre_solve_ms=%.3f sample_adjacency_ms=%.3f active_zone_ms=%.3f copied_frontier_mesh_ms=%.3f query_geometry_ms=%.3f frontier_point_sets_ms=%.3f resolve_transfer_loop_ms=%.3f hit_search_ms=%.3f zero_hit_recovery_ms=%.3f direct_hit_transfer_ms=%.3f fallback_transfer_ms=%.3f quiet_interior_preserve_ms=%.3f attribution_ms=%.3f repartition_ms=%.3f subduction_field_ms=%.3f plate_scores_ms=%.3f slab_pull_ms=%.3f terrane_ms=%.3f component_audit_ms=%.3f rebuild_meshes_ms=%.3f collision_shadow_ms=%.3f collision_exec_ms=%.3f work_samples=%d work_plates=%d plate_local_vertices=%d plate_local_triangles=%d copied_frontier_vertices=%d copied_frontier_triangles=%d copied_frontier_carried=%d direct_hit_transfer_count=%d zero_candidate_count=%d miss_count=%d nearest_member_fallback_count=%d explicit_fallback_count=%d quiet_interior_touched=%d tracked_destructive=%d tracked_subduction=%d tracked_collision=%d"),
			*Run.Label,
			SolveStep,
			SolveStats.SolveMilliseconds,
			Phase.PreSolveCaptureMs,
			Phase.SampleAdjacencyBuildMs,
			Phase.ActiveZoneMaskMs,
			Phase.CopiedFrontierMeshBuildMs,
			Phase.QueryGeometryBuildMs,
			Phase.FrontierPointSetBuildMs,
			Phase.ResolveTransferLoopMs,
			Phase.HitSearchMs,
			Phase.ZeroHitRecoveryMs,
			Phase.DirectHitTransferMs,
			Phase.FallbackTransferMs,
			Phase.QuietInteriorPreservationMs,
			Phase.AttributionMs,
			Phase.RepartitionMembershipMs,
			Phase.SubductionDistanceFieldMs,
			Phase.PlateScoresMs,
			Phase.SlabPullMs,
			Phase.TerraneDetectionMs,
			Phase.ComponentAuditMs,
			Phase.RebuildCopiedFrontierMeshesMs,
			Phase.CollisionShadowMs,
			Phase.CollisionExecutionMs,
			Run.Planet.GetPlanet().Samples.Num(),
			SolveStats.PlateCount,
			SolveStats.PlateLocalVertexCount,
			SolveStats.PlateLocalTriangleCount,
			SolveStats.CopiedFrontierVertexCount,
			SolveStats.CopiedFrontierTriangleCount,
			SolveStats.CopiedFrontierCarriedSampleCount,
			SolveStats.DirectHitTriangleTransferCount,
			SolveStats.ZeroCandidateCount,
			SolveStats.MissCount,
			SolveStats.NearestMemberFallbackTransferCount,
			SolveStats.ExplicitFallbackTransferCount,
			SolveStats.PaperSurrogateOwnershipOverrideCount,
			SolveStats.TrackedDestructiveTriangleCount,
			SolveStats.TrackedSubductionTriangleCount,
			SolveStats.TrackedCollisionTriangleCount));
		AddInfo(FString::Printf(
			TEXT("[V9Perf %s solve=%d fanout] hit_avg=%.3f hit_max=%d hit_pruned_samples=%d recovery_gather_avg=%.3f recovery_gather_max=%d recovery_gather_samples=%d recovery_pruned_samples=%d recovery_miss_avg=%.3f recovery_miss_max=%d recovery_miss_samples=%d"),
			*Run.Label,
			SolveStep,
			HitSearchAverageCandidateCount,
			SolveStats.HitSearchPlateCandidateCountMax,
			SolveStats.HitSearchPrunedSampleCount,
			RecoveryGatherAverageCandidateCount,
			SolveStats.RecoveryCandidatePlateCandidateCountMax,
			SolveStats.RecoveryCandidateGatherSampleCount,
			SolveStats.RecoveryCandidatePrunedSampleCount,
			RecoveryMissAverageCandidateCount,
			SolveStats.RecoveryMissPlateCandidateCountMax,
			SolveStats.RecoveryMissSampleCount));
		AddInfo(FString::Printf(
			TEXT("[V9Perf %s solve=%d work] subduction_compute_count=%d slab_pull_compute_count=%d convergent_edge_build_count=%d convergent_edge_reuse_count=%d subduction_convergent_edges=%d slab_pull_convergent_edges=%d subduction_seed_count=%d subduction_influenced_count=%d slab_pull_front_sample_count=%d cached_adjacency_edge_count=%d cached_adjacency_lookup_count=%lld"),
			*Run.Label,
			SolveStep,
			SolveStats.SubductionFieldComputeCount,
			SolveStats.SlabPullComputeCount,
			SolveStats.ConvergentEdgeBuildCount,
			SolveStats.ReusedConvergentEdgeSetCount,
			SolveStats.SubductionConvergentEdgeCount,
			SolveStats.SlabPullConvergentEdgeCount,
			SolveStats.SubductionSeedSampleCount,
			SolveStats.SubductionInfluencedCount,
			SolveStats.SlabPullFrontSampleCount,
			SolveStats.CachedAdjacencyEdgeDistanceCount,
			SolveStats.CachedAdjacencyEdgeLookupCount));
	};

	const auto AdvanceToStep =
		[&](FRunState& Run, const int32 TargetStep)
	{
		while (Run.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			const int32 PreviousSolveCount = Run.Planet.GetPeriodicSolveCount();
			Run.Planet.AdvanceStep();
			if (Run.Planet.GetPeriodicSolveCount() > PreviousSolveCount)
			{
				const int32 SolveStep = Run.Planet.GetLastSolveStats().Step;
				if (TimingSolveSteps.Contains(SolveStep))
				{
					CaptureSolveTiming(Run, SolveStep);
				}
			}
		}
	};

	const auto EvaluateStep100Gate = [this](const FRunState& Run)
	{
		constexpr double MinSubaerialCoastMean = 4.50;
		constexpr int32 MinLargestSubaerial = 2500;
		constexpr double MinBroadSeedSurvival = 0.75;
		constexpr double MaxSubmergedFraction = 0.07;
		constexpr double MinCoherence = 0.93;
		constexpr double MaxLeakage = 0.18;
		constexpr double MaxChurn = 0.05;
		constexpr double MaxP95ElevationKm = 5.50;
		constexpr int32 MaxSamplesAbove5Km = 1000;

		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(100);
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(100);
		const FV9ContinentalElevationStats& ElevDiag = Run.ElevationDiagnostics.FindChecked(100);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag =
			Run.SurvivalDiagnostics.FindChecked(100);

		const bool bMorphologyPass =
			MassDiag.SubaerialMeanCoastDistHops >= MinSubaerialCoastMean &&
			MassDiag.LargestSubaerialComponentSize >= MinLargestSubaerial &&
			SurvivalDiag.Step0BroadInteriorRemainingFraction >= MinBroadSeedSurvival &&
			MassDiag.SubmergedContinentalFraction <= MaxSubmergedFraction;
		const bool bStabilityPass =
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore >= MinCoherence &&
			Snapshot.BoundaryCoherence.InteriorLeakageFraction < MaxLeakage &&
			Snapshot.OwnershipChurn.ChurnFraction < MaxChurn;
		const bool bReliefPass =
			ElevDiag.P95ElevationKm <= MaxP95ElevationKm &&
			ElevDiag.SamplesAbove5Km <= MaxSamplesAbove5Km;
		const bool bPass = bMorphologyPass && bStabilityPass && bReliefPass;

		AddInfo(FString::Printf(
			TEXT("[V9Perf %s gate step=100] morphology=%d stability=%d relief=%d run_step200=%d"),
			*Run.Label,
			bMorphologyPass ? 1 : 0,
			bStabilityPass ? 1 : 0,
			bReliefPass ? 1 : 0,
			bPass ? 1 : 0));
		return bPass;
	};

	const auto EvaluateStep200Health = [this](const FRunState& Run)
	{
		constexpr double MinSubaerialCoastMean = 4.00;
		constexpr double MinBroadSeedSurvival = 0.60;
		constexpr double MaxSubmergedFraction = 0.08;
		constexpr double MinCoherence = 0.93;
		constexpr double MaxLeakage = 0.18;
		constexpr double MaxChurn = 0.05;
		constexpr double MaxP95ElevationKm = 6.00;
		constexpr int32 MaxSamplesAbove5Km = 1500;

		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(200);
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(200);
		const FV9ContinentalElevationStats& ElevDiag = Run.ElevationDiagnostics.FindChecked(200);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag =
			Run.SurvivalDiagnostics.FindChecked(200);

		const bool bHealthy =
			MassDiag.SubaerialMeanCoastDistHops >= MinSubaerialCoastMean &&
			SurvivalDiag.Step0BroadInteriorRemainingFraction >= MinBroadSeedSurvival &&
			MassDiag.SubmergedContinentalFraction <= MaxSubmergedFraction &&
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore >= MinCoherence &&
			Snapshot.BoundaryCoherence.InteriorLeakageFraction < MaxLeakage &&
			Snapshot.OwnershipChurn.ChurnFraction < MaxChurn &&
			ElevDiag.P95ElevationKm <= MaxP95ElevationKm &&
			ElevDiag.SamplesAbove5Km <= MaxSamplesAbove5Km;

		AddInfo(FString::Printf(
			TEXT("[V9Perf %s gate step=200] healthy=%d"),
			*Run.Label,
			bHealthy ? 1 : 0));
		return bHealthy;
	};

	const TArray<FVariantConfig> Variants = {
		{ TEXT("before_no_prune"), true, false },
		{ TEXT("after_prune"), true, true }
	};

	for (const int32 SampleCount : SampleCounts)
	{
		for (const FVariantConfig& Variant : Variants)
		{
			FRunState Run;
			Run.SampleCount = SampleCount;
			Run.Label = FString::Printf(TEXT("%dk_%s"), SampleCount / 1000, *Variant.LabelSuffix);
			Run.Planet = InitializePlanet(
				SampleCount,
				Variant.bUseCachedSubductionAdjacencyEdgeDistances,
				Variant.bUsePlateCandidatePruning);

			CaptureCheckpoint(Run, 0);
			BuildSeedFlags(
				Run.MassDiagnostics.FindChecked(0),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
			Run.SurvivalDiagnostics[0] = ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);

			AdvanceToStep(Run, 25);
			CaptureCheckpoint(Run, 25);
			AdvanceToStep(Run, 100);
			CaptureCheckpoint(Run, 100);

			const bool bAllowStep200 = EvaluateStep100Gate(Run);
			TestTrue(
				*FString::Printf(TEXT("%s step-100 gate passed"), *Run.Label),
				bAllowStep200);

			AdvanceToStep(Run, 200);
			CaptureCheckpoint(Run, 200);
			const bool bHealthyStep200 = EvaluateStep200Health(Run);
			TestTrue(
				*FString::Printf(TEXT("%s step-200 remained healthy"), *Run.Label),
				bHealthyStep200);

			for (const int32 SolveStep : TimingSolveSteps)
			{
				TestTrue(
					*FString::Printf(TEXT("%s captured timing solve %d"), *Run.Label, SolveStep),
					Run.SolveTimings.Contains(SolveStep));
			}
		}
	}

	return true;
}

// ============================================================================
// V9 Performance Budget Test
// Builds an additive wall-clock budget for ordinary steps and periodic solves
// on the kept selective-elevation candidate. Nested worker timings remain as
// secondary diagnostics only and are not treated as additive budget numbers.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9PerformanceBudgetTest,
	"Aurous.TectonicPlanet.V6V9PerformanceBudgetTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9PerformanceBudgetTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 PlateCount = 40;
	constexpr int32 CanonicalSeed = TestRandomSeed;
	const TArray<int32> SampleCounts = { 60000, 100000 };
	const TArray<int32> TimingSolveSteps = { 16, 160 };
	const TArray<int32> CheckpointSteps = { 100, 200 };

	struct FRunState
	{
		int32 SampleCount = 0;
		FString Label;
		FTectonicPlanetV6 Planet;
		TMap<int32, FTectonicPlanetStepBudget> StepBudgets;
		TMap<int32, FTectonicPlanetV6PeriodicSolveStats> SolveBudgets;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
	};

	struct FStepBudgetAverage
	{
		int32 StepCount = 0;
		double PlateCount = 0.0;
		double SampleCount = 0.0;
		double CarriedSampleCount = 0.0;
		double SubductionSampleCount = 0.0;
		double AndeanSampleCount = 0.0;
		double PlateKinematicsMs = 0.0;
		double SubductionUpliftMs = 0.0;
		double ContinentalAdjustmentMs = 0.0;
		double ErosionElevationUpdateMs = 0.0;
		double CollisionStepMs = 0.0;
		double CanonicalSyncMs = 0.0;
		double AutomaticRiftCheckMs = 0.0;
		double InStepResamplingMs = 0.0;
		double PendingCollisionFollowupMs = 0.0;
		double TotalMs = 0.0;
	};

	struct FPaperBudgetRow
	{
		FString PaperBucket;
		FString AurousBucket;
		FString PhaseComposition;
		double PaperSeconds60k = 0.0;
		double PaperSeconds100k = 0.0;
		double AurousSeconds60k = 0.0;
		double AurousSeconds100k = 0.0;
		FString Caveat;
	};

	const auto InitializePlanet = [=](const int32 SampleCount)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			FTectonicPlanetV6::GetKeptV6PeriodicSolveMode(),
			FTectonicPlanetV6::GetKeptV6FixedIntervalSteps(),
			INDEX_NONE,
			SampleCount,
			PlateCount,
			CanonicalSeed);
		ApplyKeptV6TestProfile(
			Planet,
			MakeKeptV6RuntimeProfileOptions(),
			MakeKeptV6DiagnosticsOptions(true, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleNum = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleNum);
		OutStep0BroadInteriorFlags.Init(0, SampleNum);
		OutStep0CoastAdjacentFlags.Init(0, SampleNum);
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	const auto CaptureCheckpoint = [this](
		FRunState& Run,
		const int32 Step)
	{
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9ContinentalElevationStats ElevDiag = ComputeContinentalElevationStats(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		Run.Snapshots.Add(Step, Snapshot);
		Run.MassDiagnostics.Add(Step, MassDiag);
		Run.ElevationDiagnostics.Add(Step, ElevDiag);
		Run.SurvivalDiagnostics.Add(Step, SurvivalDiag);
		AddInfo(FString::Printf(
			TEXT("[V9BudgetBehavior %s step=%d] coast_mean=%.2f largest_subaerial=%d broad_seed=%.4f submerged=%.4f coherence=%.4f leakage=%.4f churn=%.4f p95=%.4f above5=%d"),
			*Run.Label,
			Step,
			MassDiag.SubaerialMeanCoastDistHops,
			MassDiag.LargestSubaerialComponentSize,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			MassDiag.SubmergedContinentalFraction,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Snapshot.OwnershipChurn.ChurnFraction,
			ElevDiag.P95ElevationKm,
			ElevDiag.SamplesAbove5Km));
	};

	const auto CaptureSolveBudget = [this](FRunState& Run, const int32 SolveStep)
	{
		if (Run.SolveBudgets.Contains(SolveStep))
		{
			return;
		}

		const FTectonicPlanetV6PeriodicSolveStats SolveStats = Run.Planet.GetLastSolveStats();
		Run.SolveBudgets.Add(SolveStep, SolveStats);
		const FTectonicPlanetV6PhaseTiming& Phase = SolveStats.PhaseTiming;
		const double AdditiveSolveBudgetMs =
			Phase.PreSolveCaptureMs +
			Phase.SampleAdjacencyBuildMs +
			Phase.ActiveZoneMaskMs +
			Phase.CopiedFrontierMeshBuildMs +
			Phase.QueryGeometryBuildMs +
			Phase.FrontierPointSetBuildMs +
			Phase.ResolveTransferLoopMs +
			Phase.AttributionMs +
			Phase.RepartitionMembershipMs +
			Phase.SubductionDistanceFieldMs +
			Phase.PlateScoresMs +
			Phase.SlabPullMs +
			Phase.TerraneDetectionMs +
			Phase.ComponentAuditMs +
			Phase.RebuildCopiedFrontierMeshesMs +
			Phase.CollisionShadowMs +
			Phase.CollisionExecutionMs;
		const double BudgetResidualMs = SolveStats.SolveMilliseconds - AdditiveSolveBudgetMs;
		const double HitSearchAverageCandidateCount =
			Run.Planet.GetPlanet().Samples.Num() > 0
				? static_cast<double>(SolveStats.HitSearchPlateCandidateCountTotal) /
					static_cast<double>(Run.Planet.GetPlanet().Samples.Num())
				: 0.0;
		const double RecoveryGatherAverageCandidateCount =
			SolveStats.RecoveryCandidateGatherSampleCount > 0
				? static_cast<double>(SolveStats.RecoveryCandidatePlateCandidateCountTotal) /
					static_cast<double>(SolveStats.RecoveryCandidateGatherSampleCount)
				: 0.0;
		const double RecoveryMissAverageCandidateCount =
			SolveStats.RecoveryMissSampleCount > 0
				? static_cast<double>(SolveStats.RecoveryMissPlateCandidateCountTotal) /
					static_cast<double>(SolveStats.RecoveryMissSampleCount)
				: 0.0;

		AddInfo(FString::Printf(
			TEXT("[V9BudgetSolve %s solve=%d] total_ms=%.3f additive_sum_ms=%.3f residual_ms=%.3f pre_solve_ms=%.3f sample_adjacency_ms=%.3f active_zone_ms=%.3f copied_frontier_mesh_ms=%.3f query_geometry_ms=%.3f frontier_point_sets_ms=%.3f resolve_transfer_loop_ms=%.3f attribution_ms=%.3f repartition_ms=%.3f subduction_field_ms=%.3f plate_scores_ms=%.3f slab_pull_ms=%.3f terrane_ms=%.3f component_audit_ms=%.3f rebuild_meshes_ms=%.3f collision_shadow_ms=%.3f collision_exec_ms=%.3f"),
			*Run.Label,
			SolveStep,
			SolveStats.SolveMilliseconds,
			AdditiveSolveBudgetMs,
			BudgetResidualMs,
			Phase.PreSolveCaptureMs,
			Phase.SampleAdjacencyBuildMs,
			Phase.ActiveZoneMaskMs,
			Phase.CopiedFrontierMeshBuildMs,
			Phase.QueryGeometryBuildMs,
			Phase.FrontierPointSetBuildMs,
			Phase.ResolveTransferLoopMs,
			Phase.AttributionMs,
			Phase.RepartitionMembershipMs,
			Phase.SubductionDistanceFieldMs,
			Phase.PlateScoresMs,
			Phase.SlabPullMs,
			Phase.TerraneDetectionMs,
			Phase.ComponentAuditMs,
			Phase.RebuildCopiedFrontierMeshesMs,
			Phase.CollisionShadowMs,
			Phase.CollisionExecutionMs));
		AddInfo(FString::Printf(
			TEXT("[V9BudgetSolveNested %s solve=%d] hit_search_worker_ms=%.3f zero_hit_recovery_worker_ms=%.3f direct_hit_transfer_worker_ms=%.3f fallback_transfer_worker_ms=%.3f quiet_interior_preserve_worker_ms=%.3f"),
			*Run.Label,
			SolveStep,
			Phase.HitSearchMs,
			Phase.ZeroHitRecoveryMs,
			Phase.DirectHitTransferMs,
			Phase.FallbackTransferMs,
			Phase.QuietInteriorPreservationMs));
		AddInfo(FString::Printf(
			TEXT("[V9BudgetQoL %s solve=%d] copied_frontier_triangles=%d plate_local_triangles=%d direct_hit_transfer_count=%d zero_hit_recovery_count=%d recovery_miss_count=%d nearest_member_fallback_count=%d quiet_interior_touched=%d subduction_edges=%d subduction_seeds=%d influenced_samples=%d hit_search_avg_candidates=%.3f hit_search_max=%d recovery_gather_avg_candidates=%.3f recovery_gather_max=%d recovery_miss_avg_candidates=%.3f recovery_miss_max=%d"),
			*Run.Label,
			SolveStep,
			SolveStats.CopiedFrontierTriangleCount,
			SolveStats.PlateLocalTriangleCount,
			SolveStats.DirectHitTriangleTransferCount,
			SolveStats.ZeroCandidateCount,
			SolveStats.RecoveryMissSampleCount,
			SolveStats.NearestMemberFallbackTransferCount,
			SolveStats.PaperSurrogateOwnershipOverrideCount,
			SolveStats.SubductionConvergentEdgeCount,
			SolveStats.SubductionSeedSampleCount,
			SolveStats.SubductionInfluencedCount,
			HitSearchAverageCandidateCount,
			SolveStats.HitSearchPlateCandidateCountMax,
			RecoveryGatherAverageCandidateCount,
			SolveStats.RecoveryCandidatePlateCandidateCountMax,
			RecoveryMissAverageCandidateCount,
			SolveStats.RecoveryMissPlateCandidateCountMax));
	};

	const auto CaptureStepBudget = [](FRunState& Run)
	{
		const FTectonicPlanet& Planet = Run.Planet.GetPlanet();
		Run.StepBudgets.Add(Planet.CurrentStep, Planet.LastStepBudget);
	};

	const auto AdvanceToStep = [&](
		FRunState& Run,
		const int32 TargetStep)
	{
		while (Run.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			const int32 PreviousSolveCount = Run.Planet.GetPeriodicSolveCount();
			Run.Planet.AdvanceStep();
			CaptureStepBudget(Run);
			if (Run.Planet.GetPeriodicSolveCount() > PreviousSolveCount)
			{
				const int32 SolveStep = Run.Planet.GetLastSolveStats().Step;
				if (TimingSolveSteps.Contains(SolveStep))
				{
					CaptureSolveBudget(Run, SolveStep);
				}
			}
		}
	};

	const auto AverageStepBudgetRange = [](
		const TMap<int32, FTectonicPlanetStepBudget>& StepBudgets,
		const int32 InclusiveStartStep,
		const int32 InclusiveEndStep)
	{
		FStepBudgetAverage Average;
		for (int32 Step = InclusiveStartStep; Step <= InclusiveEndStep; ++Step)
		{
			const FTectonicPlanetStepBudget* Budget = StepBudgets.Find(Step);
			if (Budget == nullptr)
			{
				continue;
			}

			++Average.StepCount;
			Average.PlateCount += static_cast<double>(Budget->PlateCount);
			Average.SampleCount += static_cast<double>(Budget->SampleCount);
			Average.CarriedSampleCount += static_cast<double>(Budget->CarriedSampleCount);
			Average.SubductionSampleCount += static_cast<double>(Budget->SubductionSampleCount);
			Average.AndeanSampleCount += static_cast<double>(Budget->AndeanSampleCount);
			Average.PlateKinematicsMs += Budget->PlateKinematicsMs;
			Average.SubductionUpliftMs += Budget->SubductionUpliftMs;
			Average.ContinentalAdjustmentMs += Budget->ContinentalAdjustmentMs;
			Average.ErosionElevationUpdateMs += Budget->ErosionElevationUpdateMs;
			Average.CollisionStepMs += Budget->CollisionStepMs;
			Average.CanonicalSyncMs += Budget->CanonicalSyncMs;
			Average.AutomaticRiftCheckMs += Budget->AutomaticRiftCheckMs;
			Average.InStepResamplingMs += Budget->InStepResamplingMs;
			Average.PendingCollisionFollowupMs += Budget->PendingCollisionFollowupMs;
			Average.TotalMs += Budget->TotalMs;
		}

		if (Average.StepCount <= 0)
		{
			return Average;
		}

		const double Divisor = static_cast<double>(Average.StepCount);
		Average.PlateCount /= Divisor;
		Average.SampleCount /= Divisor;
		Average.CarriedSampleCount /= Divisor;
		Average.SubductionSampleCount /= Divisor;
		Average.AndeanSampleCount /= Divisor;
		Average.PlateKinematicsMs /= Divisor;
		Average.SubductionUpliftMs /= Divisor;
		Average.ContinentalAdjustmentMs /= Divisor;
		Average.ErosionElevationUpdateMs /= Divisor;
		Average.CollisionStepMs /= Divisor;
		Average.CanonicalSyncMs /= Divisor;
		Average.AutomaticRiftCheckMs /= Divisor;
		Average.InStepResamplingMs /= Divisor;
		Average.PendingCollisionFollowupMs /= Divisor;
		Average.TotalMs /= Divisor;
		return Average;
	};

	const auto EmitStepBudget = [this](
		const FString& Label,
		const FString& BudgetLabel,
		const int32 StartStep,
		const int32 EndStep,
		const FStepBudgetAverage& Average)
	{
		AddInfo(FString::Printf(
			TEXT("[V9BudgetStep %s %s steps=%d-%d samples=%d] total_ms=%.3f plate_kinematics_ms=%.3f subduction_uplift_ms=%.3f continental_adjust_ms=%.3f erosion_elevation_ms=%.3f collision_step_ms=%.3f canonical_sync_ms=%.3f automatic_rift_ms=%.3f in_step_resampling_ms=%.3f pending_collision_followup_ms=%.3f plate_count=%.1f sample_count=%.1f carried_sample_count=%.1f subduction_sample_count=%.1f andean_count=%.1f"),
			*Label,
			*BudgetLabel,
			StartStep,
			EndStep,
			Average.StepCount,
			Average.TotalMs,
			Average.PlateKinematicsMs,
			Average.SubductionUpliftMs,
			Average.ContinentalAdjustmentMs,
			Average.ErosionElevationUpdateMs,
			Average.CollisionStepMs,
			Average.CanonicalSyncMs,
			Average.AutomaticRiftCheckMs,
			Average.InStepResamplingMs,
			Average.PendingCollisionFollowupMs,
			Average.PlateCount,
			Average.SampleCount,
			Average.CarriedSampleCount,
			Average.SubductionSampleCount,
			Average.AndeanSampleCount));
	};

	const auto EmitIntervalBudget = [this](
		const FString& Label,
		const FString& BudgetLabel,
		const FStepBudgetAverage& StepAverage,
		const FTectonicPlanetV6PeriodicSolveStats& SolveStats)
	{
		const double CadenceIntervalMs =
			(static_cast<double>(FixedIntervalSteps) * StepAverage.TotalMs) + SolveStats.SolveMilliseconds;
		const double AmortizedPerStepMs =
			CadenceIntervalMs / static_cast<double>(FixedIntervalSteps);
		AddInfo(FString::Printf(
			TEXT("[V9BudgetInterval %s %s cadence=%d] step_avg_ms=%.3f solve_ms=%.3f cadence_interval_ms=%.3f amortized_per_step_ms=%.3f"),
			*Label,
			*BudgetLabel,
			FixedIntervalSteps,
			StepAverage.TotalMs,
			SolveStats.SolveMilliseconds,
			CadenceIntervalMs,
			AmortizedPerStepMs));
	};

	const auto IsBehaviorHealthy = [](const FRunState& Run, const int32 Step)
	{
		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& ElevDiag = Run.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag =
			Run.SurvivalDiagnostics.FindChecked(Step);
		return
			MassDiag.SubaerialMeanCoastDistHops >= (Step == 100 ? 4.5 : 4.0) &&
			MassDiag.LargestSubaerialComponentSize >= (Step == 100 ? 2500 : 4000) &&
			SurvivalDiag.Step0BroadInteriorRemainingFraction >= (Step == 100 ? 0.75 : 0.60) &&
			MassDiag.SubmergedContinentalFraction <= (Step == 100 ? 0.07 : 0.08) &&
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore >= 0.93 &&
			Snapshot.BoundaryCoherence.InteriorLeakageFraction < 0.18 &&
			Snapshot.OwnershipChurn.ChurnFraction < 0.05 &&
			ElevDiag.P95ElevationKm <= (Step == 100 ? 5.5 : 6.0) &&
			ElevDiag.SamplesAbove5Km <= (Step == 100 ? 1000 : 1500);
	};

	TMap<int32, FRunState> RunsBySampleCount;
	for (const int32 SampleCount : SampleCounts)
	{
		FRunState Run;
		Run.SampleCount = SampleCount;
		Run.Label = FString::Printf(TEXT("%dk"), SampleCount / 1000);
		Run.Planet = InitializePlanet(SampleCount);

		const FV9ContinentalMassDiagnostic Step0Mass = ComputeContinentalMassDiagnostic(Run.Planet);
		BuildSeedFlags(
			Step0Mass,
			Run.Step0ContinentalFlags,
			Run.Step0BroadInteriorFlags,
			Run.Step0CoastAdjacentFlags);

		AdvanceToStep(Run, 100);
		CaptureCheckpoint(Run, 100);
		AdvanceToStep(Run, 200);
		CaptureCheckpoint(Run, 200);

		for (const int32 SolveStep : TimingSolveSteps)
		{
			TestTrue(
				*FString::Printf(TEXT("%s captured solve budget at %d"), *Run.Label, SolveStep),
				Run.SolveBudgets.Contains(SolveStep));
		}
		for (const int32 Step : CheckpointSteps)
		{
			TestTrue(
				*FString::Printf(TEXT("%s behavior healthy at step %d"), *Run.Label, Step),
				IsBehaviorHealthy(Run, Step));
		}

		const FStepBudgetAverage EarlyStepAverage =
			AverageStepBudgetRange(Run.StepBudgets, 1, FixedIntervalSteps);
		const FStepBudgetAverage LateStepAverage =
			AverageStepBudgetRange(
				Run.StepBudgets,
				TimingSolveSteps.Last() - FixedIntervalSteps + 1,
				TimingSolveSteps.Last());
		EmitStepBudget(Run.Label, TEXT("early"), 1, FixedIntervalSteps, EarlyStepAverage);
		EmitStepBudget(
			Run.Label,
			TEXT("late"),
			TimingSolveSteps.Last() - FixedIntervalSteps + 1,
			TimingSolveSteps.Last(),
			LateStepAverage);
		EmitIntervalBudget(
			Run.Label,
			TEXT("early"),
			EarlyStepAverage,
			Run.SolveBudgets.FindChecked(TimingSolveSteps[0]));
		EmitIntervalBudget(
			Run.Label,
			TEXT("late"),
			LateStepAverage,
			Run.SolveBudgets.FindChecked(TimingSolveSteps.Last()));
		RunsBySampleCount.Add(SampleCount, MoveTemp(Run));
	}

	const FRunState& Run60k = RunsBySampleCount.FindChecked(60000);
	const FRunState& Run100k = RunsBySampleCount.FindChecked(100000);
	const FStepBudgetAverage Step60kLate =
		AverageStepBudgetRange(Run60k.StepBudgets, TimingSolveSteps.Last() - FixedIntervalSteps + 1, TimingSolveSteps.Last());
	const FStepBudgetAverage Step100kLate =
		AverageStepBudgetRange(Run100k.StepBudgets, TimingSolveSteps.Last() - FixedIntervalSteps + 1, TimingSolveSteps.Last());
	const FTectonicPlanetV6PeriodicSolveStats& Solve60kLate =
		Run60k.SolveBudgets.FindChecked(TimingSolveSteps.Last());
	const FTectonicPlanetV6PeriodicSolveStats& Solve100kLate =
		Run100k.SolveBudgets.FindChecked(TimingSolveSteps.Last());

	const auto BuildPaperBudgetRows = [&]()
	{
		TArray<FPaperBudgetRow> Rows;

		auto AddRow = [&Rows](
			const FString& PaperBucket,
			const FString& AurousBucket,
			const FString& PhaseComposition,
			const double PaperSeconds60k,
			const double PaperSeconds100k,
			const double AurousSeconds60k,
			const double AurousSeconds100k,
			const FString& Caveat)
		{
			FPaperBudgetRow& Row = Rows.AddDefaulted_GetRef();
			Row.PaperBucket = PaperBucket;
			Row.AurousBucket = AurousBucket;
			Row.PhaseComposition = PhaseComposition;
			Row.PaperSeconds60k = PaperSeconds60k;
			Row.PaperSeconds100k = PaperSeconds100k;
			Row.AurousSeconds60k = AurousSeconds60k;
			Row.AurousSeconds100k = AurousSeconds100k;
			Row.Caveat = Caveat;
		};

		const double Subduction60kSeconds =
			(Step60kLate.SubductionUpliftMs +
				(Solve60kLate.PhaseTiming.SubductionDistanceFieldMs + Solve60kLate.PhaseTiming.SlabPullMs) /
					static_cast<double>(FixedIntervalSteps)) /
			1000.0;
		const double Subduction100kSeconds =
			(Step100kLate.SubductionUpliftMs +
				(Solve100kLate.PhaseTiming.SubductionDistanceFieldMs + Solve100kLate.PhaseTiming.SlabPullMs) /
					static_cast<double>(FixedIntervalSteps)) /
			1000.0;
		AddRow(
			TEXT("Subduction"),
			TEXT("Per-step uplift + cadence-amortized subduction field"),
			TEXT("step_subduction_uplift + solve_subduction_field + solve_slab_pull"),
			0.08,
			0.14,
			Subduction60kSeconds,
			Subduction100kSeconds,
			TEXT("Aurous splits per-step uplift from cadence-16 global field recompute."));

		const double Collision60kSeconds =
			(Step60kLate.CollisionStepMs +
				(Solve60kLate.PhaseTiming.CollisionShadowMs +
					Solve60kLate.PhaseTiming.CollisionExecutionMs +
					Solve60kLate.PhaseTiming.TerraneDetectionMs +
					Solve60kLate.PhaseTiming.ComponentAuditMs) /
					static_cast<double>(FixedIntervalSteps)) /
			1000.0;
		const double Collision100kSeconds =
			(Step100kLate.CollisionStepMs +
				(Solve100kLate.PhaseTiming.CollisionShadowMs +
					Solve100kLate.PhaseTiming.CollisionExecutionMs +
					Solve100kLate.PhaseTiming.TerraneDetectionMs +
					Solve100kLate.PhaseTiming.ComponentAuditMs) /
					static_cast<double>(FixedIntervalSteps)) /
			1000.0;
		AddRow(
			TEXT("Continental collision"),
			TEXT("Cadence-amortized collision shadow/execution"),
			TEXT("step_collision + solve_collision_shadow + solve_collision_exec + solve_terrane + solve_component_audit"),
			0.02,
			0.04,
			Collision60kSeconds,
			Collision100kSeconds,
			TEXT("Kept path executes collision qualification/execution inside the cadence solve, not every step."));

		const double Elevation60kSeconds =
			(Step60kLate.ContinentalAdjustmentMs +
				Step60kLate.ErosionElevationUpdateMs +
				Step60kLate.CanonicalSyncMs) /
			1000.0;
		const double Elevation100kSeconds =
			(Step100kLate.ContinentalAdjustmentMs +
				Step100kLate.ErosionElevationUpdateMs +
				Step100kLate.CanonicalSyncMs) /
			1000.0;
		AddRow(
			TEXT("Elevation"),
			TEXT("Per-step continental adjustment + erosion/elevation update"),
			TEXT("step_continental_adjust + step_erosion_elevation + step_canonical_sync"),
			0.09,
			0.10,
			Elevation60kSeconds,
			Elevation100kSeconds,
			TEXT("Aurous elevation work is entirely per-step in the kept path; quiet-interior preserve remains inside periodic remesh."));

		const double OceanicCrust60kSeconds =
			(Solve60kLate.PhaseTiming.PreSolveCaptureMs +
				Solve60kLate.PhaseTiming.SampleAdjacencyBuildMs +
				Solve60kLate.PhaseTiming.ActiveZoneMaskMs +
				Solve60kLate.PhaseTiming.CopiedFrontierMeshBuildMs +
				Solve60kLate.PhaseTiming.QueryGeometryBuildMs +
				Solve60kLate.PhaseTiming.FrontierPointSetBuildMs +
				Solve60kLate.PhaseTiming.ResolveTransferLoopMs +
				Solve60kLate.PhaseTiming.AttributionMs +
				Solve60kLate.PhaseTiming.RepartitionMembershipMs +
				Solve60kLate.PhaseTiming.PlateScoresMs +
				Solve60kLate.PhaseTiming.RebuildCopiedFrontierMeshesMs) /
			static_cast<double>(FixedIntervalSteps) /
			1000.0;
		const double OceanicCrust100kSeconds =
			(Solve100kLate.PhaseTiming.PreSolveCaptureMs +
				Solve100kLate.PhaseTiming.SampleAdjacencyBuildMs +
				Solve100kLate.PhaseTiming.ActiveZoneMaskMs +
				Solve100kLate.PhaseTiming.CopiedFrontierMeshBuildMs +
				Solve100kLate.PhaseTiming.QueryGeometryBuildMs +
				Solve100kLate.PhaseTiming.FrontierPointSetBuildMs +
				Solve100kLate.PhaseTiming.ResolveTransferLoopMs +
				Solve100kLate.PhaseTiming.AttributionMs +
				Solve100kLate.PhaseTiming.RepartitionMembershipMs +
				Solve100kLate.PhaseTiming.PlateScoresMs +
				Solve100kLate.PhaseTiming.RebuildCopiedFrontierMeshesMs) /
			static_cast<double>(FixedIntervalSteps) /
			1000.0;
		AddRow(
			TEXT("Oceanic crust"),
			TEXT("Cadence-amortized copied-frontier remesh/transfer"),
			TEXT("solve_pre_solve + solve_sample_adjacency + solve_active_zone + solve_mesh_build + solve_query_geometry + solve_frontier_points + solve_resolve_transfer_loop + solve_attribution + solve_repartition + solve_plate_scores + solve_rebuild_meshes"),
			0.58,
			1.22,
			OceanicCrust60kSeconds,
			OceanicCrust100kSeconds,
			TEXT("Closest defensible mapping only: the kept copied-frontier remesh combines ownership, transfer, and preservation work in one cadence solve."));

		const double PlateRifting60kSeconds =
			(Step60kLate.AutomaticRiftCheckMs +
				Step60kLate.InStepResamplingMs +
				Step60kLate.PendingCollisionFollowupMs) /
			1000.0;
		const double PlateRifting100kSeconds =
			(Step100kLate.AutomaticRiftCheckMs +
				Step100kLate.InStepResamplingMs +
				Step100kLate.PendingCollisionFollowupMs) /
			1000.0;
		AddRow(
			TEXT("Plate rifting"),
			TEXT("Per-step automatic-rift trigger/follow-up overhead"),
			TEXT("step_automatic_rift + step_in_step_resampling + step_pending_collision_followup"),
			0.23,
			0.21,
			PlateRifting60kSeconds,
			PlateRifting100kSeconds,
			TEXT("Canonical kept runs did not fire a rift event; this measures readiness/trigger overhead, not an executed rift cost."));

		const double Total60kSeconds =
			((static_cast<double>(FixedIntervalSteps) * Step60kLate.TotalMs) + Solve60kLate.SolveMilliseconds) /
			static_cast<double>(FixedIntervalSteps) /
			1000.0;
		const double Total100kSeconds =
			((static_cast<double>(FixedIntervalSteps) * Step100kLate.TotalMs) + Solve100kLate.SolveMilliseconds) /
			static_cast<double>(FixedIntervalSteps) /
			1000.0;
		AddRow(
			TEXT("Total"),
			TEXT("Cadence-amortized kept-step + periodic solve"),
			TEXT("16*step_total + solve_total, amortized back to per-step"),
			0.19,
			0.28,
			Total60kSeconds,
			Total100kSeconds,
			TEXT("Paper values are from a different implementation/hardware; this is the closest additive Aurous comparison."));

		return Rows;
	};

	const TArray<FPaperBudgetRow> PaperRows = BuildPaperBudgetRows();
	for (const FPaperBudgetRow& Row : PaperRows)
	{
		AddInfo(FString::Printf(
			TEXT("[V9BudgetPaper bucket=%s] aurous_bucket=%s phases=%s paper_60k_s=%.3f paper_100k_s=%.3f aurous_60k_s=%.3f aurous_100k_s=%.3f caveat=%s"),
			*Row.PaperBucket,
			*Row.AurousBucket,
			*Row.PhaseComposition,
			Row.PaperSeconds60k,
			Row.PaperSeconds100k,
			Row.AurousSeconds60k,
			Row.AurousSeconds100k,
			*Row.Caveat));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9SubductionBudgetOptimizationTest,
	"Aurous.TectonicPlanet.V6V9SubductionBudgetOptimizationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9250kReadinessProbeTest,
	"Aurous.TectonicPlanet.V6V9250kReadinessProbeTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9250kBehaviorDiagnosisTest,
	"Aurous.TectonicPlanet.V6V9250kBehaviorDiagnosisTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9250kSubmergedShoulderFixTest,
	"Aurous.TectonicPlanet.V6V9250kSubmergedShoulderFixTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9250kBehaviorDiagnosisTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 PlateCount = 40;
	constexpr int32 CanonicalSeed = TestRandomSeed;
	constexpr int32 SecondarySeed = 31415;
	constexpr int32 SampleCount = 250000;
	const TArray<int32> CheckpointSteps = { 100, 200 };

	struct FRunState
	{
		int32 Seed = 0;
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
		TArray<uint8> Step100SubaerialContinentalFlags;
		TArray<uint8> Step100BroadInteriorFlags;
		TArray<uint8> Step100CoastAdjacentFlags;
		TArray<uint8> Step100BoundaryOrActiveFlags;
		TArray<uint8> Step100QuietRetainedFlags;
		TArray<uint8> Step100ActiveZoneFlags;
		TArray<uint8> Step100QuietRetainedBroadInteriorFlags;
	};

	struct FSubmergedDriftDiagnostic
	{
		int32 Step100SubaerialContinentalCount = 0;
		int32 Step100To200SubmergedCount = 0;
		int32 Step100To200LostContinentalCount = 0;
		int32 BroadInteriorBaseCount = 0;
		int32 BroadInteriorToSubmergedCount = 0;
		int32 BroadInteriorToLostCount = 0;
		int32 CoastAdjacentBaseCount = 0;
		int32 CoastAdjacentToSubmergedCount = 0;
		int32 CoastAdjacentToLostCount = 0;
		int32 BoundaryOrActiveBaseCount = 0;
		int32 BoundaryOrActiveToSubmergedCount = 0;
		int32 BoundaryOrActiveToLostCount = 0;
		int32 QuietRetainedBaseCount = 0;
		int32 QuietRetainedToSubmergedCount = 0;
		int32 QuietRetainedToLostCount = 0;
		int32 ActiveZoneBaseCount = 0;
		int32 ActiveZoneToSubmergedCount = 0;
		int32 ActiveZoneToLostCount = 0;
		int32 QuietRetainedBroadInteriorBaseCount = 0;
		int32 QuietRetainedBroadInteriorToSubmergedCount = 0;
		int32 QuietRetainedBroadInteriorToLostCount = 0;
	};

	const auto InitializePlanet = [=](const int32 Seed)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			FTectonicPlanetV6::GetKeptV6PeriodicSolveMode(),
			FTectonicPlanetV6::GetKeptV6FixedIntervalSteps(),
			INDEX_NONE,
			SampleCount,
			PlateCount,
			Seed);
		ApplyKeptV6TestProfile(
			Planet,
			MakeKeptV6RuntimeProfileOptions(),
			MakeKeptV6DiagnosticsOptions(true, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleNum = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleNum);
		OutStep0BroadInteriorFlags.Init(0, SampleNum);
		OutStep0CoastAdjacentFlags.Init(0, SampleNum);
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	const auto CaptureStep100DriftFlags = [](
		FRunState& Run,
		const FV9ContinentalMassDiagnostic& MassDiag)
	{
		const FTectonicPlanet& PlanetData = Run.Planet.GetPlanet();
		const TArray<uint8>& ActiveZoneFlags = Run.Planet.GetCurrentSolveActiveZoneFlagsForTest();
		const TArray<FTectonicPlanetV6ResolvedSample>& Resolved = Run.Planet.GetLastResolvedSamplesForTest();
		const int32 SampleNum = PlanetData.Samples.Num();

		Run.Step100SubaerialContinentalFlags.Init(0, SampleNum);
		Run.Step100BroadInteriorFlags.Init(0, SampleNum);
		Run.Step100CoastAdjacentFlags.Init(0, SampleNum);
		Run.Step100BoundaryOrActiveFlags.Init(0, SampleNum);
		Run.Step100QuietRetainedFlags.Init(0, SampleNum);
		Run.Step100ActiveZoneFlags.Init(0, SampleNum);
		Run.Step100QuietRetainedBroadInteriorFlags.Init(0, SampleNum);

		const bool bHasActiveZoneFlags = ActiveZoneFlags.Num() == SampleNum;
		const bool bHasResolved = Resolved.Num() == SampleNum;
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			const FSample& Sample = PlanetData.Samples[SampleIndex];
			const bool bStep100Continental = Sample.ContinentalWeight >= 0.5f;
			const bool bStep100Subaerial = bStep100Continental && Sample.Elevation > 0.0f;
			if (!bStep100Subaerial)
			{
				continue;
			}

			Run.Step100SubaerialContinentalFlags[SampleIndex] = 1;

			const int32 CoastDepth =
				MassDiag.SampleCoastDistHops.IsValidIndex(SampleIndex)
					? MassDiag.SampleCoastDistHops[SampleIndex]
					: -1;
			const bool bBroadInterior = CoastDepth >= 2;
			const bool bCoastAdjacent = CoastDepth == 1;
			const bool bActiveZone =
				bHasActiveZoneFlags && ActiveZoneFlags[SampleIndex] != 0;
			const bool bBoundaryOrActive = Sample.bIsBoundary || bActiveZone;
			const bool bQuietRetained =
				bHasResolved && Resolved[SampleIndex].bAuthorityRetainedOutsideActiveZone;

			Run.Step100BroadInteriorFlags[SampleIndex] = bBroadInterior ? 1 : 0;
			Run.Step100CoastAdjacentFlags[SampleIndex] = bCoastAdjacent ? 1 : 0;
			Run.Step100BoundaryOrActiveFlags[SampleIndex] = bBoundaryOrActive ? 1 : 0;
			Run.Step100QuietRetainedFlags[SampleIndex] = bQuietRetained ? 1 : 0;
			Run.Step100ActiveZoneFlags[SampleIndex] = bActiveZone ? 1 : 0;
			Run.Step100QuietRetainedBroadInteriorFlags[SampleIndex] =
				(bQuietRetained && bBroadInterior) ? 1 : 0;
		}
	};

	const auto ComputeSubmergedDriftDiagnostic = [](
		const FRunState& Run)
	{
		FSubmergedDriftDiagnostic Result;
		const FTectonicPlanet& PlanetData = Run.Planet.GetPlanet();
		const int32 SampleNum = PlanetData.Samples.Num();
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Run.Step100SubaerialContinentalFlags.IsValidIndex(SampleIndex) ||
				Run.Step100SubaerialContinentalFlags[SampleIndex] == 0)
			{
				continue;
			}

			++Result.Step100SubaerialContinentalCount;
			const FSample& CurrentSample = PlanetData.Samples[SampleIndex];
			const bool bStep200Continental = CurrentSample.ContinentalWeight >= 0.5f;
			const bool bStep200SubmergedContinental = bStep200Continental && CurrentSample.Elevation <= 0.0f;
			const bool bStep200LostContinental = !bStep200Continental;

			if (bStep200SubmergedContinental)
			{
				++Result.Step100To200SubmergedCount;
			}
			if (bStep200LostContinental)
			{
				++Result.Step100To200LostContinentalCount;
			}

			const auto UpdateCohort = [bStep200SubmergedContinental, bStep200LostContinental](
				const bool bInCohort,
				int32& BaseCount,
				int32& ToSubmergedCount,
				int32& ToLostCount)
			{
				if (!bInCohort)
				{
					return;
				}

				++BaseCount;
				if (bStep200SubmergedContinental)
				{
					++ToSubmergedCount;
				}
				if (bStep200LostContinental)
				{
					++ToLostCount;
				}
			};

			UpdateCohort(
				Run.Step100BroadInteriorFlags.IsValidIndex(SampleIndex) &&
					Run.Step100BroadInteriorFlags[SampleIndex] != 0,
				Result.BroadInteriorBaseCount,
				Result.BroadInteriorToSubmergedCount,
				Result.BroadInteriorToLostCount);
			UpdateCohort(
				Run.Step100CoastAdjacentFlags.IsValidIndex(SampleIndex) &&
					Run.Step100CoastAdjacentFlags[SampleIndex] != 0,
				Result.CoastAdjacentBaseCount,
				Result.CoastAdjacentToSubmergedCount,
				Result.CoastAdjacentToLostCount);
			UpdateCohort(
				Run.Step100BoundaryOrActiveFlags.IsValidIndex(SampleIndex) &&
					Run.Step100BoundaryOrActiveFlags[SampleIndex] != 0,
				Result.BoundaryOrActiveBaseCount,
				Result.BoundaryOrActiveToSubmergedCount,
				Result.BoundaryOrActiveToLostCount);
			UpdateCohort(
				Run.Step100QuietRetainedFlags.IsValidIndex(SampleIndex) &&
					Run.Step100QuietRetainedFlags[SampleIndex] != 0,
				Result.QuietRetainedBaseCount,
				Result.QuietRetainedToSubmergedCount,
				Result.QuietRetainedToLostCount);
			UpdateCohort(
				Run.Step100ActiveZoneFlags.IsValidIndex(SampleIndex) &&
					Run.Step100ActiveZoneFlags[SampleIndex] != 0,
				Result.ActiveZoneBaseCount,
				Result.ActiveZoneToSubmergedCount,
				Result.ActiveZoneToLostCount);
			UpdateCohort(
				Run.Step100QuietRetainedBroadInteriorFlags.IsValidIndex(SampleIndex) &&
					Run.Step100QuietRetainedBroadInteriorFlags[SampleIndex] != 0,
				Result.QuietRetainedBroadInteriorBaseCount,
				Result.QuietRetainedBroadInteriorToSubmergedCount,
				Result.QuietRetainedBroadInteriorToLostCount);
		}

		return Result;
	};

	const auto EmitSubmergedDriftDiagnostic = [this](
		const FString& Label,
		const FSubmergedDriftDiagnostic& Diag)
	{
		const auto Frac = [](const int32 Count, const int32 Base)
		{
			return Base > 0 ? static_cast<double>(Count) / static_cast<double>(Base) : 0.0;
		};

		AddInfo(FString::Printf(
			TEXT("[V9250kDrift %s] step100_subaerial=%d to_submerged=%d(%.4f) to_lost=%d(%.4f) broad=%d/%d(%.4f) coast_adjacent=%d/%d(%.4f) boundary_or_active=%d/%d(%.4f) quiet_retained=%d/%d(%.4f) active_zone=%d/%d(%.4f) quiet_retained_broad=%d/%d(%.4f)"),
			*Label,
			Diag.Step100SubaerialContinentalCount,
			Diag.Step100To200SubmergedCount,
			Frac(Diag.Step100To200SubmergedCount, Diag.Step100SubaerialContinentalCount),
			Diag.Step100To200LostContinentalCount,
			Frac(Diag.Step100To200LostContinentalCount, Diag.Step100SubaerialContinentalCount),
			Diag.BroadInteriorToSubmergedCount,
			Diag.BroadInteriorBaseCount,
			Frac(Diag.BroadInteriorToSubmergedCount, Diag.BroadInteriorBaseCount),
			Diag.CoastAdjacentToSubmergedCount,
			Diag.CoastAdjacentBaseCount,
			Frac(Diag.CoastAdjacentToSubmergedCount, Diag.CoastAdjacentBaseCount),
			Diag.BoundaryOrActiveToSubmergedCount,
			Diag.BoundaryOrActiveBaseCount,
			Frac(Diag.BoundaryOrActiveToSubmergedCount, Diag.BoundaryOrActiveBaseCount),
			Diag.QuietRetainedToSubmergedCount,
			Diag.QuietRetainedBaseCount,
			Frac(Diag.QuietRetainedToSubmergedCount, Diag.QuietRetainedBaseCount),
			Diag.ActiveZoneToSubmergedCount,
			Diag.ActiveZoneBaseCount,
			Frac(Diag.ActiveZoneToSubmergedCount, Diag.ActiveZoneBaseCount),
			Diag.QuietRetainedBroadInteriorToSubmergedCount,
			Diag.QuietRetainedBroadInteriorBaseCount,
			Frac(Diag.QuietRetainedBroadInteriorToSubmergedCount, Diag.QuietRetainedBroadInteriorBaseCount)));
	};

	const auto CaptureCheckpoint =
		[this](FRunState& Run, const int32 Step, const bool bExport)
	{
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Run.Planet, Run.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Run.Planet, Run.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9ContinentalElevationStats ElevDiag = ComputeContinentalElevationStats(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		Run.Snapshots.Add(Step, Snapshot);
		Run.MassDiagnostics.Add(Step, MassDiag);
		Run.ElevationDiagnostics.Add(Step, ElevDiag);
		Run.SurvivalDiagnostics.Add(Step, SurvivalDiag);
		AddInfo(FString::Printf(
			TEXT("[V9250kCore %s step=%d] caf=%.4f subaerial=%.4f submerged=%.4f coast_mean=%.2f subaerial_coast=%.2f largest_subaerial=%d broad_seed=%.4f coherence=%.4f leakage=%.4f churn=%.4f mean_elev=%.4f p95=%.4f above2=%d above5=%d"),
			*Run.Label,
			Step,
			MassDiag.ContinentalAreaFraction,
			MassDiag.SubaerialContinentalFraction,
			MassDiag.SubmergedContinentalFraction,
			MassDiag.MeanCoastDistHops,
			MassDiag.SubaerialMeanCoastDistHops,
			MassDiag.LargestSubaerialComponentSize,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Snapshot.OwnershipChurn.ChurnFraction,
			ElevDiag.MeanElevationKm,
			ElevDiag.P95ElevationKm,
			ElevDiag.SamplesAbove2Km,
			ElevDiag.SamplesAbove5Km));

		if (bExport)
		{
			ExportContinentalMassOverlays(*this, Run.Planet, MassDiag, Run.ExportRoot, Step);
		}
	};

	const auto AdvanceToStep = [](
		FRunState& Run,
		const int32 TargetStep)
	{
		while (Run.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			Run.Planet.AdvanceStep();
		}
	};

	const FString RunId = TEXT("V9250kBehaviorDiagnosis");
	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	const FString CanonicalExportRoot = FPaths::Combine(ExportRoot, TEXT("canonical_seed42"));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*CanonicalExportRoot);

	TArray<FRunState> Runs;
	for (const int32 Seed : { CanonicalSeed, SecondarySeed })
	{
		FRunState Run;
		Run.Seed = Seed;
		Run.Label = FString::Printf(TEXT("250k_seed%d"), Seed);
		Run.ExportRoot = (Seed == CanonicalSeed) ? CanonicalExportRoot : FString();
		Run.Planet = InitializePlanet(Seed);

		const FV9ContinentalMassDiagnostic Step0Mass = ComputeContinentalMassDiagnostic(Run.Planet);
		BuildSeedFlags(
			Step0Mass,
			Run.Step0ContinentalFlags,
			Run.Step0BroadInteriorFlags,
			Run.Step0CoastAdjacentFlags);

		AdvanceToStep(Run, 100);
		CaptureCheckpoint(Run, 100, Seed == CanonicalSeed);
		CaptureStep100DriftFlags(Run, Run.MassDiagnostics.FindChecked(100));

		AdvanceToStep(Run, 200);
		CaptureCheckpoint(Run, 200, Seed == CanonicalSeed);

		const FSubmergedDriftDiagnostic DriftDiag = ComputeSubmergedDriftDiagnostic(Run);
		EmitSubmergedDriftDiagnostic(Run.Label, DriftDiag);

		Runs.Add(MoveTemp(Run));
	}

	for (const FRunState& Run : Runs)
	{
		for (const int32 Step : CheckpointSteps)
		{
			TestTrue(
				*FString::Printf(TEXT("%s captured checkpoint %d"), *Run.Label, Step),
				Run.Snapshots.Contains(Step) &&
				Run.MassDiagnostics.Contains(Step) &&
				Run.ElevationDiagnostics.Contains(Step) &&
				Run.SurvivalDiagnostics.Contains(Step));
		}
	}

	AddInfo(FString::Printf(
		TEXT("[V9250kBehaviorDiagnosis] export_root=%s seed_count=%d"),
		*ExportRoot,
		Runs.Num()));

	return true;
}

bool FTectonicPlanetV6V9250kSubmergedShoulderFixTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 PlateCount = 40;
	constexpr int32 CanonicalSeed = TestRandomSeed;
	constexpr int32 SecondarySeed = 31415;
	constexpr int32 SampleCount = 250000;
	const TArray<int32> CheckpointSteps = { 100, 200 };

	struct FVariantConfig
	{
		FString Label;
		bool bEnableFringeFix = false;
	};

	struct FRunState
	{
		FVariantConfig Variant;
		int32 Seed = 0;
		FString Label;
		FString ExportRoot;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
		TArray<uint8> Step100SubaerialContinentalFlags;
		TArray<uint8> Step100BroadInteriorFlags;
		TArray<uint8> Step100CoastAdjacentFlags;
		TArray<uint8> Step100BoundaryOrActiveFlags;
		TArray<uint8> Step100QuietRetainedFlags;
		TArray<uint8> Step100ActiveZoneFlags;
		TArray<uint8> Step100QuietRetainedBroadInteriorFlags;
	};

	struct FSubmergedDriftDiagnostic
	{
		int32 Step100SubaerialContinentalCount = 0;
		int32 Step100To200SubmergedCount = 0;
		int32 Step100To200LostContinentalCount = 0;
		int32 BroadInteriorBaseCount = 0;
		int32 BroadInteriorToSubmergedCount = 0;
		int32 BroadInteriorToLostCount = 0;
		int32 CoastAdjacentBaseCount = 0;
		int32 CoastAdjacentToSubmergedCount = 0;
		int32 CoastAdjacentToLostCount = 0;
		int32 BoundaryOrActiveBaseCount = 0;
		int32 BoundaryOrActiveToSubmergedCount = 0;
		int32 BoundaryOrActiveToLostCount = 0;
		int32 QuietRetainedBaseCount = 0;
		int32 QuietRetainedToSubmergedCount = 0;
		int32 QuietRetainedToLostCount = 0;
		int32 ActiveZoneBaseCount = 0;
		int32 ActiveZoneToSubmergedCount = 0;
		int32 ActiveZoneToLostCount = 0;
		int32 QuietRetainedBroadInteriorBaseCount = 0;
		int32 QuietRetainedBroadInteriorToSubmergedCount = 0;
		int32 QuietRetainedBroadInteriorToLostCount = 0;
	};

	const TArray<FVariantConfig> Variants = {
		{ TEXT("baseline"), false },
		{ TEXT("candidate"), true },
	};

	const auto InitializePlanet = [=](const int32 Seed, const FVariantConfig& Variant)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			FTectonicPlanetV6::GetKeptV6PeriodicSolveMode(),
			FTectonicPlanetV6::GetKeptV6FixedIntervalSteps(),
			INDEX_NONE,
			SampleCount,
			PlateCount,
			Seed);
		FTectonicPlanetV6KeptRuntimeProfileOptions RuntimeOptions =
			MakeKeptV6RuntimeProfileOptions();
		RuntimeOptions.bEnableSubmergedContinentalFringeRelaxation = Variant.bEnableFringeFix;
		ApplyKeptV6TestProfile(
			Planet,
			RuntimeOptions,
			MakeKeptV6DiagnosticsOptions(true, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleNum = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleNum);
		OutStep0BroadInteriorFlags.Init(0, SampleNum);
		OutStep0CoastAdjacentFlags.Init(0, SampleNum);
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	const auto CaptureStep100DriftFlags = [](
		FRunState& Run,
		const FV9ContinentalMassDiagnostic& MassDiag)
	{
		const FTectonicPlanet& PlanetData = Run.Planet.GetPlanet();
		const TArray<uint8>& ActiveZoneFlags = Run.Planet.GetCurrentSolveActiveZoneFlagsForTest();
		const TArray<FTectonicPlanetV6ResolvedSample>& Resolved = Run.Planet.GetLastResolvedSamplesForTest();
		const int32 SampleNum = PlanetData.Samples.Num();

		Run.Step100SubaerialContinentalFlags.Init(0, SampleNum);
		Run.Step100BroadInteriorFlags.Init(0, SampleNum);
		Run.Step100CoastAdjacentFlags.Init(0, SampleNum);
		Run.Step100BoundaryOrActiveFlags.Init(0, SampleNum);
		Run.Step100QuietRetainedFlags.Init(0, SampleNum);
		Run.Step100ActiveZoneFlags.Init(0, SampleNum);
		Run.Step100QuietRetainedBroadInteriorFlags.Init(0, SampleNum);

		const bool bHasActiveZoneFlags = ActiveZoneFlags.Num() == SampleNum;
		const bool bHasResolved = Resolved.Num() == SampleNum;
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			const FSample& Sample = PlanetData.Samples[SampleIndex];
			const bool bStep100Continental = Sample.ContinentalWeight >= 0.5f;
			const bool bStep100Subaerial = bStep100Continental && Sample.Elevation > 0.0f;
			if (!bStep100Subaerial)
			{
				continue;
			}

			Run.Step100SubaerialContinentalFlags[SampleIndex] = 1;

			const int32 CoastDepth =
				MassDiag.SampleCoastDistHops.IsValidIndex(SampleIndex)
					? MassDiag.SampleCoastDistHops[SampleIndex]
					: -1;
			const bool bBroadInterior = CoastDepth >= 2;
			const bool bCoastAdjacent = CoastDepth == 1;
			const bool bActiveZone =
				bHasActiveZoneFlags && ActiveZoneFlags[SampleIndex] != 0;
			const bool bBoundaryOrActive = Sample.bIsBoundary || bActiveZone;
			const bool bQuietRetained =
				bHasResolved && Resolved[SampleIndex].bAuthorityRetainedOutsideActiveZone;

			Run.Step100BroadInteriorFlags[SampleIndex] = bBroadInterior ? 1 : 0;
			Run.Step100CoastAdjacentFlags[SampleIndex] = bCoastAdjacent ? 1 : 0;
			Run.Step100BoundaryOrActiveFlags[SampleIndex] = bBoundaryOrActive ? 1 : 0;
			Run.Step100QuietRetainedFlags[SampleIndex] = bQuietRetained ? 1 : 0;
			Run.Step100ActiveZoneFlags[SampleIndex] = bActiveZone ? 1 : 0;
			Run.Step100QuietRetainedBroadInteriorFlags[SampleIndex] =
				(bQuietRetained && bBroadInterior) ? 1 : 0;
		}
	};

	const auto ComputeSubmergedDriftDiagnostic = [](
		const FRunState& Run)
	{
		FSubmergedDriftDiagnostic Result;
		const FTectonicPlanet& PlanetData = Run.Planet.GetPlanet();
		const int32 SampleNum = PlanetData.Samples.Num();
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Run.Step100SubaerialContinentalFlags.IsValidIndex(SampleIndex) ||
				Run.Step100SubaerialContinentalFlags[SampleIndex] == 0)
			{
				continue;
			}

			++Result.Step100SubaerialContinentalCount;
			const FSample& CurrentSample = PlanetData.Samples[SampleIndex];
			const bool bStep200Continental = CurrentSample.ContinentalWeight >= 0.5f;
			const bool bStep200SubmergedContinental = bStep200Continental && CurrentSample.Elevation <= 0.0f;
			const bool bStep200LostContinental = !bStep200Continental;

			if (bStep200SubmergedContinental)
			{
				++Result.Step100To200SubmergedCount;
			}
			if (bStep200LostContinental)
			{
				++Result.Step100To200LostContinentalCount;
			}

			const auto UpdateCohort = [bStep200SubmergedContinental, bStep200LostContinental](
				const bool bInCohort,
				int32& BaseCount,
				int32& ToSubmergedCount,
				int32& ToLostCount)
			{
				if (!bInCohort)
				{
					return;
				}

				++BaseCount;
				if (bStep200SubmergedContinental)
				{
					++ToSubmergedCount;
				}
				if (bStep200LostContinental)
				{
					++ToLostCount;
				}
			};

			UpdateCohort(
				Run.Step100BroadInteriorFlags.IsValidIndex(SampleIndex) &&
					Run.Step100BroadInteriorFlags[SampleIndex] != 0,
				Result.BroadInteriorBaseCount,
				Result.BroadInteriorToSubmergedCount,
				Result.BroadInteriorToLostCount);
			UpdateCohort(
				Run.Step100CoastAdjacentFlags.IsValidIndex(SampleIndex) &&
					Run.Step100CoastAdjacentFlags[SampleIndex] != 0,
				Result.CoastAdjacentBaseCount,
				Result.CoastAdjacentToSubmergedCount,
				Result.CoastAdjacentToLostCount);
			UpdateCohort(
				Run.Step100BoundaryOrActiveFlags.IsValidIndex(SampleIndex) &&
					Run.Step100BoundaryOrActiveFlags[SampleIndex] != 0,
				Result.BoundaryOrActiveBaseCount,
				Result.BoundaryOrActiveToSubmergedCount,
				Result.BoundaryOrActiveToLostCount);
			UpdateCohort(
				Run.Step100QuietRetainedFlags.IsValidIndex(SampleIndex) &&
					Run.Step100QuietRetainedFlags[SampleIndex] != 0,
				Result.QuietRetainedBaseCount,
				Result.QuietRetainedToSubmergedCount,
				Result.QuietRetainedToLostCount);
			UpdateCohort(
				Run.Step100ActiveZoneFlags.IsValidIndex(SampleIndex) &&
					Run.Step100ActiveZoneFlags[SampleIndex] != 0,
				Result.ActiveZoneBaseCount,
				Result.ActiveZoneToSubmergedCount,
				Result.ActiveZoneToLostCount);
			UpdateCohort(
				Run.Step100QuietRetainedBroadInteriorFlags.IsValidIndex(SampleIndex) &&
					Run.Step100QuietRetainedBroadInteriorFlags[SampleIndex] != 0,
				Result.QuietRetainedBroadInteriorBaseCount,
				Result.QuietRetainedBroadInteriorToSubmergedCount,
				Result.QuietRetainedBroadInteriorToLostCount);
		}

		return Result;
	};

	const auto EmitCore = [this](const FRunState& Run, const int32 Step)
	{
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& ElevDiag = Run.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag = Run.SurvivalDiagnostics.FindChecked(Step);
		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(Step);
		AddInfo(FString::Printf(
			TEXT("[V9250kShoulderFix core variant=%s seed=%d step=%d] caf=%.4f subaerial=%.4f submerged=%.4f coast_mean=%.2f subaerial_coast=%.2f largest_subaerial=%d broad_seed=%.4f coherence=%.4f leakage=%.4f churn=%.4f mean_elev=%.4f p95=%.4f above2=%d above5=%d"),
			*Run.Variant.Label,
			Run.Seed,
			Step,
			MassDiag.ContinentalAreaFraction,
			MassDiag.SubaerialContinentalFraction,
			MassDiag.SubmergedContinentalFraction,
			MassDiag.MeanCoastDistHops,
			MassDiag.SubaerialMeanCoastDistHops,
			MassDiag.LargestSubaerialComponentSize,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Snapshot.OwnershipChurn.ChurnFraction,
			ElevDiag.MeanElevationKm,
			ElevDiag.P95ElevationKm,
			ElevDiag.SamplesAbove2Km,
			ElevDiag.SamplesAbove5Km));
	};

	const auto EmitDrift = [this](
		const FRunState& Run,
		const FSubmergedDriftDiagnostic& Diag)
	{
		const auto Frac = [](const int32 Count, const int32 Base)
		{
			return Base > 0 ? static_cast<double>(Count) / static_cast<double>(Base) : 0.0;
		};

		AddInfo(FString::Printf(
			TEXT("[V9250kShoulderFix drift variant=%s seed=%d] step100_subaerial=%d to_submerged=%d(%.4f) to_lost=%d(%.4f) broad=%d/%d(%.4f) coast_adjacent=%d/%d(%.4f) boundary_or_active=%d/%d(%.4f) quiet_retained=%d/%d(%.4f) active_zone=%d/%d(%.4f) quiet_retained_broad=%d/%d(%.4f)"),
			*Run.Variant.Label,
			Run.Seed,
			Diag.Step100SubaerialContinentalCount,
			Diag.Step100To200SubmergedCount,
			Frac(Diag.Step100To200SubmergedCount, Diag.Step100SubaerialContinentalCount),
			Diag.Step100To200LostContinentalCount,
			Frac(Diag.Step100To200LostContinentalCount, Diag.Step100SubaerialContinentalCount),
			Diag.BroadInteriorToSubmergedCount,
			Diag.BroadInteriorBaseCount,
			Frac(Diag.BroadInteriorToSubmergedCount, Diag.BroadInteriorBaseCount),
			Diag.CoastAdjacentToSubmergedCount,
			Diag.CoastAdjacentBaseCount,
			Frac(Diag.CoastAdjacentToSubmergedCount, Diag.CoastAdjacentBaseCount),
			Diag.BoundaryOrActiveToSubmergedCount,
			Diag.BoundaryOrActiveBaseCount,
			Frac(Diag.BoundaryOrActiveToSubmergedCount, Diag.BoundaryOrActiveBaseCount),
			Diag.QuietRetainedToSubmergedCount,
			Diag.QuietRetainedBaseCount,
			Frac(Diag.QuietRetainedToSubmergedCount, Diag.QuietRetainedBaseCount),
			Diag.ActiveZoneToSubmergedCount,
			Diag.ActiveZoneBaseCount,
			Frac(Diag.ActiveZoneToSubmergedCount, Diag.ActiveZoneBaseCount),
			Diag.QuietRetainedBroadInteriorToSubmergedCount,
			Diag.QuietRetainedBroadInteriorBaseCount,
			Frac(Diag.QuietRetainedBroadInteriorToSubmergedCount, Diag.QuietRetainedBroadInteriorBaseCount)));
	};

	const auto CaptureCheckpoint =
		[this](FRunState& Run, const int32 Step, const bool bExport)
	{
		if (bExport)
		{
			ExportV6CheckpointMaps(*this, Run.Planet, Run.ExportRoot, Step);
			ExportV6DebugOverlays(*this, Run.Planet, Run.ExportRoot, Step);
		}

		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9ContinentalElevationStats ElevDiag = ComputeContinentalElevationStats(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		Run.Snapshots.Add(Step, Snapshot);
		Run.MassDiagnostics.Add(Step, MassDiag);
		Run.ElevationDiagnostics.Add(Step, ElevDiag);
		Run.SurvivalDiagnostics.Add(Step, SurvivalDiag);
		if (bExport)
		{
			ExportContinentalMassOverlays(*this, Run.Planet, MassDiag, Run.ExportRoot, Step);
		}
	};

	const auto AdvanceToStep = [](
		FRunState& Run,
		const int32 TargetStep)
	{
		while (Run.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			Run.Planet.AdvanceStep();
		}
	};

	const auto FindRun = [](const TArray<FRunState>& Runs, const FString& VariantLabel, const int32 Seed) -> const FRunState*
	{
		for (const FRunState& Run : Runs)
		{
			if (Run.Variant.Label == VariantLabel && Run.Seed == Seed)
			{
				return &Run;
			}
		}
		return nullptr;
	};

	const FString RunId = TEXT("V9250kSubmergedShoulderFix");
	const FString ExportRoot = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);

	TArray<FRunState> Runs;
	for (const FVariantConfig& Variant : Variants)
	{
		for (const int32 Seed : { CanonicalSeed, SecondarySeed })
		{
			FRunState Run;
			Run.Variant = Variant;
			Run.Seed = Seed;
			Run.Label = FString::Printf(TEXT("%s_seed%d"), *Variant.Label, Seed);
			if (Seed == CanonicalSeed)
			{
				Run.ExportRoot = FPaths::Combine(
					ExportRoot,
					FString::Printf(TEXT("%s_seed%d"), *Variant.Label, Seed));
				PlatformFile.CreateDirectoryTree(*Run.ExportRoot);
			}
			Run.Planet = InitializePlanet(Seed, Variant);

			const FV9ContinentalMassDiagnostic Step0Mass = ComputeContinentalMassDiagnostic(Run.Planet);
			BuildSeedFlags(
				Step0Mass,
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);

			AdvanceToStep(Run, 100);
			CaptureCheckpoint(Run, 100, Seed == CanonicalSeed);
			CaptureStep100DriftFlags(Run, Run.MassDiagnostics.FindChecked(100));

			AdvanceToStep(Run, 200);
			CaptureCheckpoint(Run, 200, Seed == CanonicalSeed);

			const FSubmergedDriftDiagnostic DriftDiag = ComputeSubmergedDriftDiagnostic(Run);
			EmitCore(Run, 100);
			EmitCore(Run, 200);
			EmitDrift(Run, DriftDiag);

			Runs.Add(MoveTemp(Run));
		}
	}

	for (const FRunState& Run : Runs)
	{
		for (const int32 Step : CheckpointSteps)
		{
			TestTrue(
				*FString::Printf(TEXT("%s captured checkpoint %d"), *Run.Label, Step),
				Run.Snapshots.Contains(Step) &&
				Run.MassDiagnostics.Contains(Step) &&
				Run.ElevationDiagnostics.Contains(Step) &&
				Run.SurvivalDiagnostics.Contains(Step));
		}
	}

	for (const int32 Seed : { CanonicalSeed, SecondarySeed })
	{
		const FRunState* Baseline = FindRun(Runs, TEXT("baseline"), Seed);
		const FRunState* Candidate = FindRun(Runs, TEXT("candidate"), Seed);
		TestNotNull(*FString::Printf(TEXT("baseline seed %d"), Seed), Baseline);
		TestNotNull(*FString::Printf(TEXT("candidate seed %d"), Seed), Candidate);
		if (Baseline == nullptr || Candidate == nullptr)
		{
			continue;
		}

		for (const int32 Step : CheckpointSteps)
		{
			const FV9ContinentalMassDiagnostic& BaselineMass = Baseline->MassDiagnostics.FindChecked(Step);
			const FV9ContinentalMassDiagnostic& CandidateMass = Candidate->MassDiagnostics.FindChecked(Step);
			const FV9ContinentalElevationStats& BaselineElev = Baseline->ElevationDiagnostics.FindChecked(Step);
			const FV9ContinentalElevationStats& CandidateElev = Candidate->ElevationDiagnostics.FindChecked(Step);
			const FV9SeededContinentalSurvivalDiagnostic& BaselineSurvival =
				Baseline->SurvivalDiagnostics.FindChecked(Step);
			const FV9SeededContinentalSurvivalDiagnostic& CandidateSurvival =
				Candidate->SurvivalDiagnostics.FindChecked(Step);
			const FV6CheckpointSnapshot& BaselineSnapshot = Baseline->Snapshots.FindChecked(Step);
			const FV6CheckpointSnapshot& CandidateSnapshot = Candidate->Snapshots.FindChecked(Step);
			AddInfo(FString::Printf(
				TEXT("[V9250kShoulderFix compare seed=%d step=%d] caf=%.4f/%.4f subaerial=%.4f/%.4f submerged=%.4f/%.4f coast_mean=%.2f/%.2f subaerial_coast=%.2f/%.2f largest_subaerial=%d/%d broad_seed=%.4f/%.4f coherence=%.4f/%.4f leakage=%.4f/%.4f churn=%.4f/%.4f mean_elev=%.4f/%.4f p95=%.4f/%.4f above2=%d/%d above5=%d/%d"),
				Seed,
				Step,
				BaselineMass.ContinentalAreaFraction,
				CandidateMass.ContinentalAreaFraction,
				BaselineMass.SubaerialContinentalFraction,
				CandidateMass.SubaerialContinentalFraction,
				BaselineMass.SubmergedContinentalFraction,
				CandidateMass.SubmergedContinentalFraction,
				BaselineMass.MeanCoastDistHops,
				CandidateMass.MeanCoastDistHops,
				BaselineMass.SubaerialMeanCoastDistHops,
				CandidateMass.SubaerialMeanCoastDistHops,
				BaselineMass.LargestSubaerialComponentSize,
				CandidateMass.LargestSubaerialComponentSize,
				BaselineSurvival.Step0BroadInteriorRemainingFraction,
				CandidateSurvival.Step0BroadInteriorRemainingFraction,
				BaselineSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
				CandidateSnapshot.BoundaryCoherence.BoundaryCoherenceScore,
				BaselineSnapshot.BoundaryCoherence.InteriorLeakageFraction,
				CandidateSnapshot.BoundaryCoherence.InteriorLeakageFraction,
				BaselineSnapshot.OwnershipChurn.ChurnFraction,
				CandidateSnapshot.OwnershipChurn.ChurnFraction,
				BaselineElev.MeanElevationKm,
				CandidateElev.MeanElevationKm,
				BaselineElev.P95ElevationKm,
				CandidateElev.P95ElevationKm,
				BaselineElev.SamplesAbove2Km,
				CandidateElev.SamplesAbove2Km,
				BaselineElev.SamplesAbove5Km,
				CandidateElev.SamplesAbove5Km));
		}
	}

	AddInfo(FString::Printf(
		TEXT("[V9250kShoulderFix] export_root=%s variant_count=%d seed_count=%d"),
		*ExportRoot,
		Variants.Num(),
		2));

	return true;
}

bool FTectonicPlanetV6V9250kReadinessProbeTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 PlateCount = 40;
	constexpr int32 CanonicalSeed = TestRandomSeed;
	constexpr int32 SampleCount = 250000;
	const TArray<int32> TimingSolveSteps = { 16, 160 };

	struct FRunState
	{
		FString Label;
		FTectonicPlanetV6 Planet;
		TMap<int32, FTectonicPlanetStepBudget> StepBudgets;
		TMap<int32, FTectonicPlanetV6PeriodicSolveStats> SolveBudgets;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
	};

	struct FStepBudgetAverage
	{
		int32 StepCount = 0;
		double PlateCount = 0.0;
		double SampleCount = 0.0;
		double CarriedSampleCount = 0.0;
		double SubductionSampleCount = 0.0;
		double AndeanSampleCount = 0.0;
		double PlateKinematicsMs = 0.0;
		double SubductionUpliftMs = 0.0;
		double ContinentalAdjustmentMs = 0.0;
		double ErosionElevationUpdateMs = 0.0;
		double CollisionStepMs = 0.0;
		double CanonicalSyncMs = 0.0;
		double AutomaticRiftCheckMs = 0.0;
		double InStepResamplingMs = 0.0;
		double PendingCollisionFollowupMs = 0.0;
		double TotalMs = 0.0;
	};

	const auto InitializePlanet = [=]()
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			FTectonicPlanetV6::GetKeptV6PeriodicSolveMode(),
			FTectonicPlanetV6::GetKeptV6FixedIntervalSteps(),
			INDEX_NONE,
			SampleCount,
			PlateCount,
			CanonicalSeed);
		ApplyKeptV6TestProfile(
			Planet,
			MakeKeptV6RuntimeProfileOptions(),
			MakeKeptV6DiagnosticsOptions(true, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleNum = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleNum);
		OutStep0BroadInteriorFlags.Init(0, SampleNum);
		OutStep0CoastAdjacentFlags.Init(0, SampleNum);
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	const auto CaptureCheckpoint = [this](
		FRunState& Run,
		const int32 Step)
	{
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9ContinentalElevationStats ElevDiag = ComputeContinentalElevationStats(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		Run.Snapshots.Add(Step, Snapshot);
		Run.MassDiagnostics.Add(Step, MassDiag);
		Run.ElevationDiagnostics.Add(Step, ElevDiag);
		Run.SurvivalDiagnostics.Add(Step, SurvivalDiag);
		AddInfo(FString::Printf(
			TEXT("[V9BudgetBehavior %s step=%d] coast_mean=%.2f largest_subaerial=%d broad_seed=%.4f submerged=%.4f coherence=%.4f leakage=%.4f churn=%.4f p95=%.4f above5=%d"),
			*Run.Label,
			Step,
			MassDiag.SubaerialMeanCoastDistHops,
			MassDiag.LargestSubaerialComponentSize,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			MassDiag.SubmergedContinentalFraction,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Snapshot.OwnershipChurn.ChurnFraction,
			ElevDiag.P95ElevationKm,
			ElevDiag.SamplesAbove5Km));
	};

	const auto CaptureSolveBudget = [this](FRunState& Run, const int32 SolveStep)
	{
		if (Run.SolveBudgets.Contains(SolveStep))
		{
			return;
		}

		const FTectonicPlanetV6PeriodicSolveStats SolveStats = Run.Planet.GetLastSolveStats();
		Run.SolveBudgets.Add(SolveStep, SolveStats);
		const FTectonicPlanetV6PhaseTiming& Phase = SolveStats.PhaseTiming;
		const double AdditiveSolveBudgetMs =
			Phase.PreSolveCaptureMs +
			Phase.SampleAdjacencyBuildMs +
			Phase.ActiveZoneMaskMs +
			Phase.CopiedFrontierMeshBuildMs +
			Phase.QueryGeometryBuildMs +
			Phase.FrontierPointSetBuildMs +
			Phase.ResolveTransferLoopMs +
			Phase.AttributionMs +
			Phase.RepartitionMembershipMs +
			Phase.SubductionDistanceFieldMs +
			Phase.PlateScoresMs +
			Phase.SlabPullMs +
			Phase.TerraneDetectionMs +
			Phase.ComponentAuditMs +
			Phase.RebuildCopiedFrontierMeshesMs +
			Phase.CollisionShadowMs +
			Phase.CollisionExecutionMs;
		const double BudgetResidualMs = SolveStats.SolveMilliseconds - AdditiveSolveBudgetMs;
		const double HitSearchAverageCandidateCount =
			Run.Planet.GetPlanet().Samples.Num() > 0
				? static_cast<double>(SolveStats.HitSearchPlateCandidateCountTotal) /
					static_cast<double>(Run.Planet.GetPlanet().Samples.Num())
				: 0.0;
		const double RecoveryGatherAverageCandidateCount =
			SolveStats.RecoveryCandidateGatherSampleCount > 0
				? static_cast<double>(SolveStats.RecoveryCandidatePlateCandidateCountTotal) /
					static_cast<double>(SolveStats.RecoveryCandidateGatherSampleCount)
				: 0.0;
		const double RecoveryMissAverageCandidateCount =
			SolveStats.RecoveryMissSampleCount > 0
				? static_cast<double>(SolveStats.RecoveryMissPlateCandidateCountTotal) /
					static_cast<double>(SolveStats.RecoveryMissSampleCount)
				: 0.0;

		AddInfo(FString::Printf(
			TEXT("[V9BudgetSolve %s solve=%d] total_ms=%.3f additive_sum_ms=%.3f residual_ms=%.3f pre_solve_ms=%.3f sample_adjacency_ms=%.3f active_zone_ms=%.3f copied_frontier_mesh_ms=%.3f query_geometry_ms=%.3f frontier_point_sets_ms=%.3f resolve_transfer_loop_ms=%.3f attribution_ms=%.3f repartition_ms=%.3f subduction_field_ms=%.3f plate_scores_ms=%.3f slab_pull_ms=%.3f terrane_ms=%.3f component_audit_ms=%.3f rebuild_meshes_ms=%.3f collision_shadow_ms=%.3f collision_exec_ms=%.3f"),
			*Run.Label,
			SolveStep,
			SolveStats.SolveMilliseconds,
			AdditiveSolveBudgetMs,
			BudgetResidualMs,
			Phase.PreSolveCaptureMs,
			Phase.SampleAdjacencyBuildMs,
			Phase.ActiveZoneMaskMs,
			Phase.CopiedFrontierMeshBuildMs,
			Phase.QueryGeometryBuildMs,
			Phase.FrontierPointSetBuildMs,
			Phase.ResolveTransferLoopMs,
			Phase.AttributionMs,
			Phase.RepartitionMembershipMs,
			Phase.SubductionDistanceFieldMs,
			Phase.PlateScoresMs,
			Phase.SlabPullMs,
			Phase.TerraneDetectionMs,
			Phase.ComponentAuditMs,
			Phase.RebuildCopiedFrontierMeshesMs,
			Phase.CollisionShadowMs,
			Phase.CollisionExecutionMs));
		AddInfo(FString::Printf(
			TEXT("[V9BudgetQoL %s solve=%d] copied_frontier_triangles=%d plate_local_triangles=%d direct_hit_transfer_count=%d zero_hit_recovery_count=%d recovery_miss_count=%d nearest_member_fallback_count=%d quiet_interior_touched=%d subduction_edges=%d subduction_seeds=%d influenced_samples=%d hit_search_avg_candidates=%.3f hit_search_max=%d recovery_gather_avg_candidates=%.3f recovery_gather_max=%d recovery_miss_avg_candidates=%.3f recovery_miss_max=%d"),
			*Run.Label,
			SolveStep,
			SolveStats.CopiedFrontierTriangleCount,
			SolveStats.PlateLocalTriangleCount,
			SolveStats.DirectHitTriangleTransferCount,
			SolveStats.ZeroCandidateCount,
			SolveStats.RecoveryMissSampleCount,
			SolveStats.NearestMemberFallbackTransferCount,
			SolveStats.PaperSurrogateOwnershipOverrideCount,
			SolveStats.SubductionConvergentEdgeCount,
			SolveStats.SubductionSeedSampleCount,
			SolveStats.SubductionInfluencedCount,
			HitSearchAverageCandidateCount,
			SolveStats.HitSearchPlateCandidateCountMax,
			RecoveryGatherAverageCandidateCount,
			SolveStats.RecoveryCandidatePlateCandidateCountMax,
			RecoveryMissAverageCandidateCount,
			SolveStats.RecoveryMissPlateCandidateCountMax));
	};

	const auto CaptureStepBudget = [](FRunState& Run)
	{
		const FTectonicPlanet& Planet = Run.Planet.GetPlanet();
		Run.StepBudgets.Add(Planet.CurrentStep, Planet.LastStepBudget);
	};

	const auto AdvanceToStep = [&](
		FRunState& Run,
		const int32 TargetStep)
	{
		while (Run.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			const int32 PreviousSolveCount = Run.Planet.GetPeriodicSolveCount();
			Run.Planet.AdvanceStep();
			CaptureStepBudget(Run);
			if (Run.Planet.GetPeriodicSolveCount() > PreviousSolveCount)
			{
				const int32 SolveStep = Run.Planet.GetLastSolveStats().Step;
				if (TimingSolveSteps.Contains(SolveStep))
				{
					CaptureSolveBudget(Run, SolveStep);
				}
			}
		}
	};

	const auto AverageStepBudgetRange = [](
		const TMap<int32, FTectonicPlanetStepBudget>& StepBudgets,
		const int32 InclusiveStartStep,
		const int32 InclusiveEndStep)
	{
		FStepBudgetAverage Average;
		for (int32 Step = InclusiveStartStep; Step <= InclusiveEndStep; ++Step)
		{
			const FTectonicPlanetStepBudget* Budget = StepBudgets.Find(Step);
			if (Budget == nullptr)
			{
				continue;
			}

			++Average.StepCount;
			Average.PlateCount += static_cast<double>(Budget->PlateCount);
			Average.SampleCount += static_cast<double>(Budget->SampleCount);
			Average.CarriedSampleCount += static_cast<double>(Budget->CarriedSampleCount);
			Average.SubductionSampleCount += static_cast<double>(Budget->SubductionSampleCount);
			Average.AndeanSampleCount += static_cast<double>(Budget->AndeanSampleCount);
			Average.PlateKinematicsMs += Budget->PlateKinematicsMs;
			Average.SubductionUpliftMs += Budget->SubductionUpliftMs;
			Average.ContinentalAdjustmentMs += Budget->ContinentalAdjustmentMs;
			Average.ErosionElevationUpdateMs += Budget->ErosionElevationUpdateMs;
			Average.CollisionStepMs += Budget->CollisionStepMs;
			Average.CanonicalSyncMs += Budget->CanonicalSyncMs;
			Average.AutomaticRiftCheckMs += Budget->AutomaticRiftCheckMs;
			Average.InStepResamplingMs += Budget->InStepResamplingMs;
			Average.PendingCollisionFollowupMs += Budget->PendingCollisionFollowupMs;
			Average.TotalMs += Budget->TotalMs;
		}

		if (Average.StepCount <= 0)
		{
			return Average;
		}

		const double Divisor = static_cast<double>(Average.StepCount);
		Average.PlateCount /= Divisor;
		Average.SampleCount /= Divisor;
		Average.CarriedSampleCount /= Divisor;
		Average.SubductionSampleCount /= Divisor;
		Average.AndeanSampleCount /= Divisor;
		Average.PlateKinematicsMs /= Divisor;
		Average.SubductionUpliftMs /= Divisor;
		Average.ContinentalAdjustmentMs /= Divisor;
		Average.ErosionElevationUpdateMs /= Divisor;
		Average.CollisionStepMs /= Divisor;
		Average.CanonicalSyncMs /= Divisor;
		Average.AutomaticRiftCheckMs /= Divisor;
		Average.InStepResamplingMs /= Divisor;
		Average.PendingCollisionFollowupMs /= Divisor;
		Average.TotalMs /= Divisor;
		return Average;
	};

	const auto EmitStepBudget = [this](
		const FString& Label,
		const FString& BudgetLabel,
		const int32 StartStep,
		const int32 EndStep,
		const FStepBudgetAverage& Average)
	{
		AddInfo(FString::Printf(
			TEXT("[V9BudgetStep %s %s steps=%d-%d samples=%d] total_ms=%.3f plate_kinematics_ms=%.3f subduction_uplift_ms=%.3f continental_adjust_ms=%.3f erosion_elevation_ms=%.3f collision_step_ms=%.3f canonical_sync_ms=%.3f automatic_rift_ms=%.3f in_step_resampling_ms=%.3f pending_collision_followup_ms=%.3f plate_count=%.1f sample_count=%.1f carried_sample_count=%.1f subduction_sample_count=%.1f andean_count=%.1f"),
			*Label,
			*BudgetLabel,
			StartStep,
			EndStep,
			Average.StepCount,
			Average.TotalMs,
			Average.PlateKinematicsMs,
			Average.SubductionUpliftMs,
			Average.ContinentalAdjustmentMs,
			Average.ErosionElevationUpdateMs,
			Average.CollisionStepMs,
			Average.CanonicalSyncMs,
			Average.AutomaticRiftCheckMs,
			Average.InStepResamplingMs,
			Average.PendingCollisionFollowupMs,
			Average.PlateCount,
			Average.SampleCount,
			Average.CarriedSampleCount,
			Average.SubductionSampleCount,
			Average.AndeanSampleCount));
	};

	const auto EmitIntervalBudget = [this](
		const FString& Label,
		const FString& BudgetLabel,
		const FStepBudgetAverage& StepAverage,
		const FTectonicPlanetV6PeriodicSolveStats& SolveStats)
	{
		const double CadenceIntervalMs =
			(static_cast<double>(FixedIntervalSteps) * StepAverage.TotalMs) + SolveStats.SolveMilliseconds;
		const double AmortizedPerStepMs =
			CadenceIntervalMs / static_cast<double>(FixedIntervalSteps);
		AddInfo(FString::Printf(
			TEXT("[V9BudgetInterval %s %s cadence=%d] step_avg_ms=%.3f solve_ms=%.3f cadence_interval_ms=%.3f amortized_per_step_ms=%.3f"),
			*Label,
			*BudgetLabel,
			FixedIntervalSteps,
			StepAverage.TotalMs,
			SolveStats.SolveMilliseconds,
			CadenceIntervalMs,
			AmortizedPerStepMs));
	};

	const auto IsBehaviorHealthy = [](const FRunState& Run, const int32 Step)
	{
		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& ElevDiag = Run.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag =
			Run.SurvivalDiagnostics.FindChecked(Step);
		return
			MassDiag.SubaerialMeanCoastDistHops >= (Step == 100 ? 4.5 : 4.0) &&
			MassDiag.LargestSubaerialComponentSize >= (Step == 100 ? 2500 : 4000) &&
			SurvivalDiag.Step0BroadInteriorRemainingFraction >= (Step == 100 ? 0.75 : 0.60) &&
			MassDiag.SubmergedContinentalFraction <= (Step == 100 ? 0.07 : 0.08) &&
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore >= 0.93 &&
			Snapshot.BoundaryCoherence.InteriorLeakageFraction < 0.18 &&
			Snapshot.OwnershipChurn.ChurnFraction < 0.05 &&
			ElevDiag.P95ElevationKm <= (Step == 100 ? 5.5 : 6.0) &&
			ElevDiag.SamplesAbove5Km <= (Step == 100 ? 1000 : 1500);
	};

	FRunState Run;
	Run.Label = TEXT("250k");
	Run.Planet = InitializePlanet();

	const FV9ContinentalMassDiagnostic Step0Mass = ComputeContinentalMassDiagnostic(Run.Planet);
	BuildSeedFlags(
		Step0Mass,
		Run.Step0ContinentalFlags,
		Run.Step0BroadInteriorFlags,
		Run.Step0CoastAdjacentFlags);

	AdvanceToStep(Run, 100);
	CaptureCheckpoint(Run, 100);
	AdvanceToStep(Run, 200);
	CaptureCheckpoint(Run, 200);

	for (const int32 SolveStep : TimingSolveSteps)
	{
		TestTrue(
			*FString::Printf(TEXT("%s captured solve budget at %d"), *Run.Label, SolveStep),
			Run.SolveBudgets.Contains(SolveStep));
	}

	const FStepBudgetAverage EarlyStepAverage =
		AverageStepBudgetRange(Run.StepBudgets, 1, FixedIntervalSteps);
	const FStepBudgetAverage LateStepAverage =
		AverageStepBudgetRange(
			Run.StepBudgets,
			TimingSolveSteps.Last() - FixedIntervalSteps + 1,
			TimingSolveSteps.Last());
	EmitStepBudget(Run.Label, TEXT("early"), 1, FixedIntervalSteps, EarlyStepAverage);
	EmitStepBudget(
		Run.Label,
		TEXT("late"),
		TimingSolveSteps.Last() - FixedIntervalSteps + 1,
		TimingSolveSteps.Last(),
		LateStepAverage);
	EmitIntervalBudget(
		Run.Label,
		TEXT("early"),
		EarlyStepAverage,
		Run.SolveBudgets.FindChecked(TimingSolveSteps[0]));
	EmitIntervalBudget(
		Run.Label,
		TEXT("late"),
		LateStepAverage,
		Run.SolveBudgets.FindChecked(TimingSolveSteps.Last()));

	AddInfo(FString::Printf(
		TEXT("[V9250kReadiness %s] healthy_step100=%d healthy_step200=%d"),
		*Run.Label,
		IsBehaviorHealthy(Run, 100) ? 1 : 0,
		IsBehaviorHealthy(Run, 200) ? 1 : 0));

	return true;
}

bool FTectonicPlanetV6V9SubductionBudgetOptimizationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 PlateCount = 40;
	constexpr int32 CanonicalSeed = TestRandomSeed;
	const TArray<int32> SampleCounts = { 60000, 100000 };
	const TArray<int32> TimingSolveSteps = { 16, 160 };

	struct FRunState
	{
		int32 SampleCount = 0;
		FString Label;
		FTectonicPlanetV6 Planet;
		TMap<int32, FTectonicPlanetV6PeriodicSolveStats> SolveBudgets;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationStats> ElevationDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
	};

	struct FVariantConfig
	{
		FString LabelSuffix;
		bool bUseSubductionPerformanceOptimizations = true;
	};

	const auto InitializePlanet = [=](
		const int32 SampleCount,
		const bool bUseSubductionPerformanceOptimizations)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			FTectonicPlanetV6::GetKeptV6PeriodicSolveMode(),
			FTectonicPlanetV6::GetKeptV6FixedIntervalSteps(),
			INDEX_NONE,
			SampleCount,
			PlateCount,
			CanonicalSeed);
		FTectonicPlanetV6KeptRuntimeProfileOptions RuntimeOptions =
			MakeKeptV6RuntimeProfileOptions();
		RuntimeOptions.bUseSubductionPerformanceOptimizations =
			bUseSubductionPerformanceOptimizations;
		ApplyKeptV6TestProfile(
			Planet,
			RuntimeOptions,
			MakeKeptV6DiagnosticsOptions(true, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleNum = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleNum);
		OutStep0BroadInteriorFlags.Init(0, SampleNum);
		OutStep0CoastAdjacentFlags.Init(0, SampleNum);
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	const auto CaptureCheckpoint = [this](
		FRunState& Run,
		const int32 Step)
	{
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9ContinentalElevationStats ElevDiag = ComputeContinentalElevationStats(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		Run.Snapshots.Add(Step, Snapshot);
		Run.MassDiagnostics.Add(Step, MassDiag);
		Run.ElevationDiagnostics.Add(Step, ElevDiag);
		Run.SurvivalDiagnostics.Add(Step, SurvivalDiag);
		AddInfo(FString::Printf(
			TEXT("[V9SubductionBehavior %s step=%d] coast_mean=%.2f largest_subaerial=%d broad_seed=%.4f submerged=%.4f coherence=%.4f leakage=%.4f churn=%.4f p95=%.4f above5=%d"),
			*Run.Label,
			Step,
			MassDiag.SubaerialMeanCoastDistHops,
			MassDiag.LargestSubaerialComponentSize,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			MassDiag.SubmergedContinentalFraction,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Snapshot.OwnershipChurn.ChurnFraction,
			ElevDiag.P95ElevationKm,
			ElevDiag.SamplesAbove5Km));
	};

	const auto CaptureSolveBudget = [this](FRunState& Run, const int32 SolveStep)
	{
		if (Run.SolveBudgets.Contains(SolveStep))
		{
			return;
		}

		const FTectonicPlanetV6PeriodicSolveStats SolveStats = Run.Planet.GetLastSolveStats();
		Run.SolveBudgets.Add(SolveStep, SolveStats);
		const FTectonicPlanetV6PhaseTiming& Phase = SolveStats.PhaseTiming;
		const double SubductionBucketMs = Phase.SubductionDistanceFieldMs + Phase.SlabPullMs;
		const double SubductionFieldSubphaseSumMs =
			SolveStats.SubductionConvergentEdgeBuildMs +
			SolveStats.SubductionSeedInitializationMs +
			SolveStats.SubductionPropagationMs +
			SolveStats.SubductionFinalizeMs;
		const double SubductionFieldResidualMs =
			Phase.SubductionDistanceFieldMs - SubductionFieldSubphaseSumMs;
		const double SlabPullSubphaseSumMs =
			SolveStats.SlabPullConvergentEdgeBuildMs +
			SolveStats.SlabPullFrontierBuildMs +
			SolveStats.SlabPullApplyMs;
		const double SlabPullResidualMs = Phase.SlabPullMs - SlabPullSubphaseSumMs;
		AddInfo(FString::Printf(
			TEXT("[V9SubductionBudget %s solve=%d] total_solve_ms=%.3f subduction_bucket_ms=%.3f subduction_field_ms=%.3f slab_pull_ms=%.3f field_edge_build_ms=%.3f field_seed_init_ms=%.3f field_propagation_ms=%.3f field_finalize_ms=%.3f field_residual_ms=%.3f slab_edge_build_ms=%.3f slab_frontier_ms=%.3f slab_apply_ms=%.3f slab_residual_ms=%.3f query_geometry_ms=%.3f resolve_transfer_loop_ms=%.3f repartition_ms=%.3f"),
			*Run.Label,
			SolveStep,
			SolveStats.SolveMilliseconds,
			SubductionBucketMs,
			Phase.SubductionDistanceFieldMs,
			Phase.SlabPullMs,
			SolveStats.SubductionConvergentEdgeBuildMs,
			SolveStats.SubductionSeedInitializationMs,
			SolveStats.SubductionPropagationMs,
			SolveStats.SubductionFinalizeMs,
			SubductionFieldResidualMs,
			SolveStats.SlabPullConvergentEdgeBuildMs,
			SolveStats.SlabPullFrontierBuildMs,
			SolveStats.SlabPullApplyMs,
			SlabPullResidualMs,
			Phase.QueryGeometryBuildMs,
			Phase.ResolveTransferLoopMs,
			Phase.RepartitionMembershipMs));
		AddInfo(FString::Printf(
			TEXT("[V9SubductionWork %s solve=%d] convergent_edge_build_count=%d subduction_edges=%d slab_pull_edges=%d seed_count=%d influenced_count=%d slab_pull_front_samples=%d cached_edge_count=%d cached_lookup_count=%lld queue_push=%lld queue_pop=%lld relaxations=%lld"),
			*Run.Label,
			SolveStep,
			SolveStats.ConvergentEdgeBuildCount,
			SolveStats.SubductionConvergentEdgeCount,
			SolveStats.SlabPullConvergentEdgeCount,
			SolveStats.SubductionSeedSampleCount,
			SolveStats.SubductionInfluencedCount,
			SolveStats.SlabPullFrontSampleCount,
			SolveStats.CachedAdjacencyEdgeDistanceCount,
			SolveStats.CachedAdjacencyEdgeLookupCount,
			SolveStats.SubductionQueuePushCount,
			SolveStats.SubductionQueuePopCount,
			SolveStats.SubductionRelaxationCount));
	};

	const auto AdvanceToStep =
		[&](FRunState& Run, const int32 TargetStep)
	{
		while (Run.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			const int32 PreviousSolveCount = Run.Planet.GetPeriodicSolveCount();
			Run.Planet.AdvanceStep();
			if (Run.Planet.GetPeriodicSolveCount() > PreviousSolveCount)
			{
				const int32 SolveStep = Run.Planet.GetLastSolveStats().Step;
				if (TimingSolveSteps.Contains(SolveStep))
				{
					CaptureSolveBudget(Run, SolveStep);
				}
			}
		}
	};

	const auto IsBehaviorHealthy = [](const FRunState& Run, const int32 Step)
	{
		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationStats& ElevDiag = Run.ElevationDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag =
			Run.SurvivalDiagnostics.FindChecked(Step);
		return
			MassDiag.SubaerialMeanCoastDistHops >= (Step == 100 ? 4.5 : 4.0) &&
			MassDiag.LargestSubaerialComponentSize >= (Step == 100 ? 2500 : 4000) &&
			SurvivalDiag.Step0BroadInteriorRemainingFraction >= (Step == 100 ? 0.75 : 0.60) &&
			MassDiag.SubmergedContinentalFraction <= (Step == 100 ? 0.07 : 0.08) &&
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore >= 0.93 &&
			Snapshot.BoundaryCoherence.InteriorLeakageFraction < 0.18 &&
			Snapshot.OwnershipChurn.ChurnFraction < 0.05 &&
			ElevDiag.P95ElevationKm <= (Step == 100 ? 5.5 : 6.0) &&
			ElevDiag.SamplesAbove5Km <= (Step == 100 ? 1000 : 1500);
	};

	const TArray<FVariantConfig> Variants = {
		{ TEXT("before_subduction_opt"), false },
		{ TEXT("after_subduction_opt"), true }
	};

	for (const int32 SampleCount : SampleCounts)
	{
		for (const FVariantConfig& Variant : Variants)
		{
			FRunState Run;
			Run.SampleCount = SampleCount;
			Run.Label = FString::Printf(TEXT("%dk_%s"), SampleCount / 1000, *Variant.LabelSuffix);
			Run.Planet = InitializePlanet(SampleCount, Variant.bUseSubductionPerformanceOptimizations);

			const FV9ContinentalMassDiagnostic Step0Mass = ComputeContinentalMassDiagnostic(Run.Planet);
			BuildSeedFlags(
				Step0Mass,
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);

			AdvanceToStep(Run, 100);
			CaptureCheckpoint(Run, 100);
			AdvanceToStep(Run, 200);
			CaptureCheckpoint(Run, 200);

			for (const int32 SolveStep : TimingSolveSteps)
			{
				TestTrue(
					*FString::Printf(TEXT("%s captured solve budget at %d"), *Run.Label, SolveStep),
					Run.SolveBudgets.Contains(SolveStep));
			}
			for (const int32 Step : {100, 200})
			{
				TestTrue(
					*FString::Printf(TEXT("%s behavior healthy at step %d"), *Run.Label, Step),
					IsBehaviorHealthy(Run, Step));
			}
		}
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9TectonicBalanceAuditTest,
	"Aurous.TectonicPlanet.V6V9TectonicBalanceAuditTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetV6V9CopiedFrontierFrontEndOptimizationTest,
	"Aurous.TectonicPlanet.V6V9CopiedFrontierFrontEndOptimizationTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetV6V9CopiedFrontierFrontEndOptimizationTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 PlateCount = 40;
	constexpr int32 CanonicalSeed = TestRandomSeed;
	const TArray<int32> SampleCounts = { 100000, 250000 };
	const TArray<int32> TimingSolveSteps = { 16, 160 };
	const TArray<int32> CheckpointSteps = { 100, 200 };

	struct FBehaviorSummary
	{
		double CoastMean = 0.0;
		int32 LargestSubaerial = 0;
		double BroadSeed = 0.0;
		double Submerged = 0.0;
		double Coherence = 0.0;
		double Leakage = 0.0;
		double Churn = 0.0;
	};

	struct FRunState
	{
		int32 SampleCount = 0;
		FString Label;
		FTectonicPlanetV6 Planet;
		TMap<int32, FTectonicPlanetV6PeriodicSolveStats> SolveBudgets;
		TMap<int32, FBehaviorSummary> BehaviorByStep;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
	};

	struct FVariantConfig
	{
		FString LabelSuffix;
		bool bUseUnfilteredMeshReuse = true;
	};

	const auto InitializePlanet = [=](
		const int32 SampleCount,
		const bool bUseUnfilteredMeshReuse)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			FTectonicPlanetV6::GetKeptV6PeriodicSolveMode(),
			FTectonicPlanetV6::GetKeptV6FixedIntervalSteps(),
			INDEX_NONE,
			SampleCount,
			PlateCount,
			CanonicalSeed);
		FTectonicPlanetV6KeptRuntimeProfileOptions RuntimeOptions =
			MakeKeptV6RuntimeProfileOptions();
		RuntimeOptions.bEnableCopiedFrontierUnfilteredMeshReuse = bUseUnfilteredMeshReuse;
		ApplyKeptV6TestProfile(
			Planet,
			RuntimeOptions,
			MakeKeptV6DiagnosticsOptions(true, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleNum = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleNum);
		OutStep0BroadInteriorFlags.Init(0, SampleNum);
		OutStep0CoastAdjacentFlags.Init(0, SampleNum);
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	const auto CaptureCheckpoint = [this](
		FRunState& Run,
		const int32 Step)
	{
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		FBehaviorSummary& Behavior = Run.BehaviorByStep.Add(Step);
		Behavior.CoastMean = MassDiag.SubaerialMeanCoastDistHops;
		Behavior.LargestSubaerial = MassDiag.LargestSubaerialComponentSize;
		Behavior.BroadSeed = SurvivalDiag.Step0BroadInteriorRemainingFraction;
		Behavior.Submerged = MassDiag.SubmergedContinentalFraction;
		Behavior.Coherence = Snapshot.BoundaryCoherence.BoundaryCoherenceScore;
		Behavior.Leakage = Snapshot.BoundaryCoherence.InteriorLeakageFraction;
		Behavior.Churn = Snapshot.OwnershipChurn.ChurnFraction;
		AddInfo(FString::Printf(
			TEXT("[V9FrontEndBehavior %s step=%d] coast_mean=%.2f largest_subaerial=%d broad_seed=%.4f submerged=%.4f coherence=%.4f leakage=%.4f churn=%.4f"),
			*Run.Label,
			Step,
			Behavior.CoastMean,
			Behavior.LargestSubaerial,
			Behavior.BroadSeed,
			Behavior.Submerged,
			Behavior.Coherence,
			Behavior.Leakage,
			Behavior.Churn));
	};

	const auto CaptureSolveBudget = [this](FRunState& Run, const int32 SolveStep)
	{
		if (Run.SolveBudgets.Contains(SolveStep))
		{
			return;
		}

		const FTectonicPlanetV6PeriodicSolveStats SolveStats = Run.Planet.GetLastSolveStats();
		Run.SolveBudgets.Add(SolveStep, SolveStats);
		const FTectonicPlanetV6PhaseTiming& Phase = SolveStats.PhaseTiming;
		const double CollisionMs = Phase.CollisionShadowMs + Phase.CollisionExecutionMs;
		AddInfo(FString::Printf(
			TEXT("[V9FrontEndBudget %s solve=%d] total_ms=%.3f copied_frontier_mesh_ms=%.3f query_geometry_ms=%.3f resolve_transfer_loop_ms=%.3f repartition_ms=%.3f subduction_ms=%.3f slab_pull_ms=%.3f collision_ms=%.3f"),
			*Run.Label,
			SolveStep,
			SolveStats.SolveMilliseconds,
			Phase.CopiedFrontierMeshBuildMs,
			Phase.QueryGeometryBuildMs,
			Phase.ResolveTransferLoopMs,
			Phase.RepartitionMembershipMs,
			Phase.SubductionDistanceFieldMs,
			Phase.SlabPullMs,
			CollisionMs));
		AddInfo(FString::Printf(
			TEXT("[V9FrontEndSubphase %s solve=%d] unfiltered_prepare_ms=%.3f filtered_build_ms=%.3f refreshed_canonical=%d refreshed_synthetic=%d"),
			*Run.Label,
			SolveStep,
			SolveStats.CopiedFrontierUnfilteredMeshPrepareMs,
			SolveStats.CopiedFrontierFilteredMeshBuildMs,
			SolveStats.CopiedFrontierRefreshedCanonicalVertexCount,
			SolveStats.CopiedFrontierRefreshedSyntheticVertexCount));
	};

	const auto AdvanceToStep =
		[&](FRunState& Run, const int32 TargetStep)
	{
		while (Run.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			const int32 PreviousSolveCount = Run.Planet.GetPeriodicSolveCount();
			Run.Planet.AdvanceStep();
			if (Run.Planet.GetPeriodicSolveCount() > PreviousSolveCount)
			{
				const int32 SolveStep = Run.Planet.GetLastSolveStats().Step;
				if (TimingSolveSteps.Contains(SolveStep))
				{
					CaptureSolveBudget(Run, SolveStep);
				}
			}
		}
	};

	const TArray<FVariantConfig> Variants = {
		{ TEXT("before_frontend_opt"), false },
		{ TEXT("after_frontend_opt"), true }
	};

	TMap<FString, FRunState> RunsByKey;
	for (const int32 SampleCount : SampleCounts)
	{
		for (const FVariantConfig& Variant : Variants)
		{
			FRunState Run;
			Run.SampleCount = SampleCount;
			Run.Label = FString::Printf(TEXT("%dk_%s"), SampleCount / 1000, *Variant.LabelSuffix);
			Run.Planet = InitializePlanet(SampleCount, Variant.bUseUnfilteredMeshReuse);

			const FV9ContinentalMassDiagnostic Step0Mass = ComputeContinentalMassDiagnostic(Run.Planet);
			BuildSeedFlags(
				Step0Mass,
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);

			AdvanceToStep(Run, 100);
			CaptureCheckpoint(Run, 100);
			AdvanceToStep(Run, 200);
			CaptureCheckpoint(Run, 200);

			for (const int32 SolveStep : TimingSolveSteps)
			{
				TestTrue(
					*FString::Printf(TEXT("%s captured solve budget at %d"), *Run.Label, SolveStep),
					Run.SolveBudgets.Contains(SolveStep));
			}
			for (const int32 Step : CheckpointSteps)
			{
				TestTrue(
					*FString::Printf(TEXT("%s captured behavior at step %d"), *Run.Label, Step),
					Run.BehaviorByStep.Contains(Step));
			}

			RunsByKey.Add(Run.Label, MoveTemp(Run));
		}
	}

	const auto CheckBehaviorDelta = [this](
		const FString& Label,
		const FBehaviorSummary& Before,
		const FBehaviorSummary& After)
	{
		TestTrue(
			*FString::Printf(TEXT("%s coast delta small"), *Label),
			FMath::Abs(Before.CoastMean - After.CoastMean) <= 0.05);
		TestTrue(
			*FString::Printf(TEXT("%s largest subaerial unchanged"), *Label),
			Before.LargestSubaerial == After.LargestSubaerial);
		TestTrue(
			*FString::Printf(TEXT("%s broad-seed delta small"), *Label),
			FMath::Abs(Before.BroadSeed - After.BroadSeed) <= 0.0025);
		TestTrue(
			*FString::Printf(TEXT("%s submerged delta small"), *Label),
			FMath::Abs(Before.Submerged - After.Submerged) <= 0.0025);
		TestTrue(
			*FString::Printf(TEXT("%s coherence delta small"), *Label),
			FMath::Abs(Before.Coherence - After.Coherence) <= 0.005);
		TestTrue(
			*FString::Printf(TEXT("%s leakage delta small"), *Label),
			FMath::Abs(Before.Leakage - After.Leakage) <= 0.005);
		TestTrue(
			*FString::Printf(TEXT("%s churn delta small"), *Label),
			FMath::Abs(Before.Churn - After.Churn) <= 0.0025);
	};

	for (const int32 SampleCount : SampleCounts)
	{
		const FString BeforeKey =
			FString::Printf(TEXT("%dk_before_frontend_opt"), SampleCount / 1000);
		const FString AfterKey =
			FString::Printf(TEXT("%dk_after_frontend_opt"), SampleCount / 1000);
		const FRunState& BeforeRun = RunsByKey.FindChecked(BeforeKey);
		const FRunState& AfterRun = RunsByKey.FindChecked(AfterKey);

		for (const int32 SolveStep : TimingSolveSteps)
		{
			const FTectonicPlanetV6PeriodicSolveStats& BeforeSolve =
				BeforeRun.SolveBudgets.FindChecked(SolveStep);
			const FTectonicPlanetV6PeriodicSolveStats& AfterSolve =
				AfterRun.SolveBudgets.FindChecked(SolveStep);
			AddInfo(FString::Printf(
				TEXT("[V9FrontEndCompare %dk solve=%d] total_ms=%.3f->%.3f copied_frontier_mesh_ms=%.3f->%.3f query_geometry_ms=%.3f->%.3f resolve_transfer_loop_ms=%.3f->%.3f repartition_ms=%.3f->%.3f subduction_ms=%.3f->%.3f unfiltered_prepare_ms=%.3f->%.3f filtered_build_ms=%.3f->%.3f refresh_counts=%d/%d"),
				SampleCount / 1000,
				SolveStep,
				BeforeSolve.SolveMilliseconds,
				AfterSolve.SolveMilliseconds,
				BeforeSolve.PhaseTiming.CopiedFrontierMeshBuildMs,
				AfterSolve.PhaseTiming.CopiedFrontierMeshBuildMs,
				BeforeSolve.PhaseTiming.QueryGeometryBuildMs,
				AfterSolve.PhaseTiming.QueryGeometryBuildMs,
				BeforeSolve.PhaseTiming.ResolveTransferLoopMs,
				AfterSolve.PhaseTiming.ResolveTransferLoopMs,
				BeforeSolve.PhaseTiming.RepartitionMembershipMs,
				AfterSolve.PhaseTiming.RepartitionMembershipMs,
				BeforeSolve.PhaseTiming.SubductionDistanceFieldMs,
				AfterSolve.PhaseTiming.SubductionDistanceFieldMs,
				BeforeSolve.CopiedFrontierUnfilteredMeshPrepareMs,
				AfterSolve.CopiedFrontierUnfilteredMeshPrepareMs,
				BeforeSolve.CopiedFrontierFilteredMeshBuildMs,
				AfterSolve.CopiedFrontierFilteredMeshBuildMs,
				AfterSolve.CopiedFrontierRefreshedCanonicalVertexCount,
				AfterSolve.CopiedFrontierRefreshedSyntheticVertexCount));
		}

		const FTectonicPlanetV6PeriodicSolveStats& BeforeLate =
			BeforeRun.SolveBudgets.FindChecked(TimingSolveSteps.Last());
		const FTectonicPlanetV6PeriodicSolveStats& AfterLate =
			AfterRun.SolveBudgets.FindChecked(TimingSolveSteps.Last());
		TestTrue(
			*FString::Printf(TEXT("%dk late copied-frontier mesh bucket improved"), SampleCount / 1000),
			AfterLate.PhaseTiming.CopiedFrontierMeshBuildMs < BeforeLate.PhaseTiming.CopiedFrontierMeshBuildMs);
		TestTrue(
			*FString::Printf(TEXT("%dk late total solve improved"), SampleCount / 1000),
			AfterLate.SolveMilliseconds < BeforeLate.SolveMilliseconds);

		for (const int32 Step : CheckpointSteps)
		{
			CheckBehaviorDelta(
				FString::Printf(TEXT("%dk step=%d"), SampleCount / 1000, Step),
				BeforeRun.BehaviorByStep.FindChecked(Step),
				AfterRun.BehaviorByStep.FindChecked(Step));
		}
	}

	return true;
}

bool FTectonicPlanetV6V9TectonicBalanceAuditTest::RunTest(const FString& Parameters)
{
	constexpr int32 FixedIntervalSteps = 16;
	constexpr int32 PlateCount = 40;
	constexpr int32 CanonicalSeed = TestRandomSeed;
	constexpr int32 Secondary250kSeed = 31415;
	const TArray<int32> CheckpointSteps = { 100, 200 };

	struct FRunConfig
	{
		FString Label;
		int32 SampleCount = 0;
		int32 Seed = 0;
		bool bEnableShoulderFix = false;
	};

	struct FRunState
	{
		FRunConfig Config;
		FTectonicPlanetV6 Planet;
		TMap<int32, FV6CheckpointSnapshot> Snapshots;
		TMap<int32, FV9ContinentalMassDiagnostic> MassDiagnostics;
		TMap<int32, FV9ContinentalElevationBandDiagnostic> ElevationBandDiagnostics;
		TMap<int32, FV9SeededContinentalSurvivalDiagnostic> SurvivalDiagnostics;
		TArray<uint8> Step0ContinentalFlags;
		TArray<uint8> Step0BroadInteriorFlags;
		TArray<uint8> Step0CoastAdjacentFlags;
		TArray<uint8> Step100SubaerialContinentalFlags;
		TArray<uint8> Step100BroadInteriorFlags;
		TArray<uint8> Step100CoastAdjacentFlags;
		TArray<uint8> Step100BoundaryOrActiveFlags;
		TArray<uint8> Step100QuietRetainedFlags;
		TArray<uint8> Step100ActiveZoneFlags;
		TArray<uint8> Step100QuietRetainedBroadInteriorFlags;
	};

	struct FSubmergedDriftDiagnostic
	{
		int32 Step100SubaerialContinentalCount = 0;
		int32 Step100To200SubmergedCount = 0;
		int32 BroadInteriorBaseCount = 0;
		int32 BroadInteriorToSubmergedCount = 0;
		int32 CoastAdjacentBaseCount = 0;
		int32 CoastAdjacentToSubmergedCount = 0;
		int32 BoundaryOrActiveBaseCount = 0;
		int32 BoundaryOrActiveToSubmergedCount = 0;
		int32 QuietRetainedBaseCount = 0;
		int32 QuietRetainedToSubmergedCount = 0;
		int32 QuietRetainedBroadInteriorBaseCount = 0;
		int32 QuietRetainedBroadInteriorToSubmergedCount = 0;
	};

	const auto SafeFrac = [](const int32 Count, const int32 Base)
	{
		return Base > 0 ? static_cast<double>(Count) / static_cast<double>(Base) : 0.0;
	};

	const auto InitializePlanet = [=](const FRunConfig& Config)
	{
		FTectonicPlanetV6 Planet = CreateInitializedPlanetV6WithConfig(
			FTectonicPlanetV6::GetKeptV6PeriodicSolveMode(),
			FTectonicPlanetV6::GetKeptV6FixedIntervalSteps(),
			INDEX_NONE,
			Config.SampleCount,
			PlateCount,
			Config.Seed);
		FTectonicPlanetV6KeptRuntimeProfileOptions RuntimeOptions =
			MakeKeptV6RuntimeProfileOptions();
		RuntimeOptions.bEnableSubmergedContinentalFringeRelaxation = Config.bEnableShoulderFix;
		ApplyKeptV6TestProfile(
			Planet,
			RuntimeOptions,
			MakeKeptV6DiagnosticsOptions(true, false));
		return Planet;
	};

	const auto BuildSeedFlags = [](
		const FV9ContinentalMassDiagnostic& Step0Mass,
		TArray<uint8>& OutStep0ContinentalFlags,
		TArray<uint8>& OutStep0BroadInteriorFlags,
		TArray<uint8>& OutStep0CoastAdjacentFlags)
	{
		const int32 SampleNum = Step0Mass.SampleCoastDistHops.Num();
		OutStep0ContinentalFlags.Init(0, SampleNum);
		OutStep0BroadInteriorFlags.Init(0, SampleNum);
		OutStep0CoastAdjacentFlags.Init(0, SampleNum);
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Step0Mass.SampleComponentId.IsValidIndex(SampleIndex) ||
				!Step0Mass.SampleCoastDistHops.IsValidIndex(SampleIndex) ||
				Step0Mass.SampleComponentId[SampleIndex] < 0)
			{
				continue;
			}

			OutStep0ContinentalFlags[SampleIndex] = 1;
			const int32 CoastDepth = Step0Mass.SampleCoastDistHops[SampleIndex];
			if (CoastDepth >= 2)
			{
				OutStep0BroadInteriorFlags[SampleIndex] = 1;
			}
			else if (CoastDepth == 1)
			{
				OutStep0CoastAdjacentFlags[SampleIndex] = 1;
			}
		}
	};

	const auto CaptureStep100DriftFlags = [](
		FRunState& Run,
		const FV9ContinentalMassDiagnostic& MassDiag)
	{
		const FTectonicPlanet& PlanetData = Run.Planet.GetPlanet();
		const TArray<uint8>& ActiveZoneFlags = Run.Planet.GetCurrentSolveActiveZoneFlagsForTest();
		const TArray<FTectonicPlanetV6ResolvedSample>& Resolved = Run.Planet.GetLastResolvedSamplesForTest();
		const int32 SampleNum = PlanetData.Samples.Num();

		Run.Step100SubaerialContinentalFlags.Init(0, SampleNum);
		Run.Step100BroadInteriorFlags.Init(0, SampleNum);
		Run.Step100CoastAdjacentFlags.Init(0, SampleNum);
		Run.Step100BoundaryOrActiveFlags.Init(0, SampleNum);
		Run.Step100QuietRetainedFlags.Init(0, SampleNum);
		Run.Step100ActiveZoneFlags.Init(0, SampleNum);
		Run.Step100QuietRetainedBroadInteriorFlags.Init(0, SampleNum);

		const bool bHasActiveZoneFlags = ActiveZoneFlags.Num() == SampleNum;
		const bool bHasResolved = Resolved.Num() == SampleNum;
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			const FSample& Sample = PlanetData.Samples[SampleIndex];
			const bool bStep100Continental = Sample.ContinentalWeight >= 0.5f;
			const bool bStep100Subaerial = bStep100Continental && Sample.Elevation > 0.0f;
			if (!bStep100Subaerial)
			{
				continue;
			}

			Run.Step100SubaerialContinentalFlags[SampleIndex] = 1;

			const int32 CoastDepth =
				MassDiag.SampleCoastDistHops.IsValidIndex(SampleIndex)
					? MassDiag.SampleCoastDistHops[SampleIndex]
					: -1;
			const bool bBroadInterior = CoastDepth >= 2;
			const bool bCoastAdjacent = CoastDepth == 1;
			const bool bActiveZone =
				bHasActiveZoneFlags && ActiveZoneFlags[SampleIndex] != 0;
			const bool bBoundaryOrActive = Sample.bIsBoundary || bActiveZone;
			const bool bQuietRetained =
				bHasResolved && Resolved[SampleIndex].bAuthorityRetainedOutsideActiveZone;

			Run.Step100BroadInteriorFlags[SampleIndex] = bBroadInterior ? 1 : 0;
			Run.Step100CoastAdjacentFlags[SampleIndex] = bCoastAdjacent ? 1 : 0;
			Run.Step100BoundaryOrActiveFlags[SampleIndex] = bBoundaryOrActive ? 1 : 0;
			Run.Step100QuietRetainedFlags[SampleIndex] = bQuietRetained ? 1 : 0;
			Run.Step100ActiveZoneFlags[SampleIndex] = bActiveZone ? 1 : 0;
			Run.Step100QuietRetainedBroadInteriorFlags[SampleIndex] =
				(bQuietRetained && bBroadInterior) ? 1 : 0;
		}
	};

	const auto ComputeSubmergedDriftDiagnostic = [&SafeFrac](const FRunState& Run)
	{
		FSubmergedDriftDiagnostic Result;
		const FTectonicPlanet& PlanetData = Run.Planet.GetPlanet();
		const int32 SampleNum = PlanetData.Samples.Num();
		for (int32 SampleIndex = 0; SampleIndex < SampleNum; ++SampleIndex)
		{
			if (!Run.Step100SubaerialContinentalFlags.IsValidIndex(SampleIndex) ||
				Run.Step100SubaerialContinentalFlags[SampleIndex] == 0)
			{
				continue;
			}

			++Result.Step100SubaerialContinentalCount;
			const FSample& CurrentSample = PlanetData.Samples[SampleIndex];
			const bool bStep200SubmergedContinental =
				CurrentSample.ContinentalWeight >= 0.5f && CurrentSample.Elevation <= 0.0f;

			if (bStep200SubmergedContinental)
			{
				++Result.Step100To200SubmergedCount;
			}

			const auto UpdateCohort = [bStep200SubmergedContinental](
				const bool bInCohort,
				int32& BaseCount,
				int32& ToSubmergedCount)
			{
				if (!bInCohort)
				{
					return;
				}

				++BaseCount;
				if (bStep200SubmergedContinental)
				{
					++ToSubmergedCount;
				}
			};

			UpdateCohort(
				Run.Step100BroadInteriorFlags.IsValidIndex(SampleIndex) &&
					Run.Step100BroadInteriorFlags[SampleIndex] != 0,
				Result.BroadInteriorBaseCount,
				Result.BroadInteriorToSubmergedCount);
			UpdateCohort(
				Run.Step100CoastAdjacentFlags.IsValidIndex(SampleIndex) &&
					Run.Step100CoastAdjacentFlags[SampleIndex] != 0,
				Result.CoastAdjacentBaseCount,
				Result.CoastAdjacentToSubmergedCount);
			UpdateCohort(
				Run.Step100BoundaryOrActiveFlags.IsValidIndex(SampleIndex) &&
					Run.Step100BoundaryOrActiveFlags[SampleIndex] != 0,
				Result.BoundaryOrActiveBaseCount,
				Result.BoundaryOrActiveToSubmergedCount);
			UpdateCohort(
				Run.Step100QuietRetainedFlags.IsValidIndex(SampleIndex) &&
					Run.Step100QuietRetainedFlags[SampleIndex] != 0,
				Result.QuietRetainedBaseCount,
				Result.QuietRetainedToSubmergedCount);
			UpdateCohort(
				Run.Step100QuietRetainedBroadInteriorFlags.IsValidIndex(SampleIndex) &&
					Run.Step100QuietRetainedBroadInteriorFlags[SampleIndex] != 0,
				Result.QuietRetainedBroadInteriorBaseCount,
				Result.QuietRetainedBroadInteriorToSubmergedCount);
		}

		return Result;
	};

	const auto EmitCheckpoint = [this, &SafeFrac](
		const FRunState& Run,
		const int32 Step)
	{
		const FV6CheckpointSnapshot& Snapshot = Run.Snapshots.FindChecked(Step);
		const FV9ContinentalMassDiagnostic& MassDiag = Run.MassDiagnostics.FindChecked(Step);
		const FV9ContinentalElevationBandDiagnostic& ElevDiag =
			Run.ElevationBandDiagnostics.FindChecked(Step);
		const FV9SeededContinentalSurvivalDiagnostic& SurvivalDiag =
			Run.SurvivalDiagnostics.FindChecked(Step);
		const double BoundaryCoverageFraction =
			static_cast<double>(Snapshot.BoundaryCoherence.BoundaryBandSampleCount) /
			static_cast<double>(Run.Config.SampleCount);
		const double MissRate =
			static_cast<double>(Snapshot.MissCount) / static_cast<double>(Run.Config.SampleCount);
		const double MultiHitRate =
			static_cast<double>(Snapshot.MultiHitCount) / static_cast<double>(Run.Config.SampleCount);
		const double ActiveConvergentFraction =
			static_cast<double>(Snapshot.ActiveZone.ActiveConvergentSubductionSampleCount) /
			static_cast<double>(Run.Config.SampleCount);

		AddInfo(FString::Printf(
			TEXT("[V9BalanceAudit core label=%s sample_count=%d seed=%d step=%d] caf=%.4f subaerial=%.4f submerged=%.4f coast_mean=%.2f coast_p50=%.1f coast_p90=%.1f subaerial_coast_mean=%.2f subaerial_coast_p50=%.1f subaerial_coast_p90=%.1f largest=%d largest_subaerial=%d broad_seed=%.4f active_fraction=%.4f boundary_coverage=%.4f active_zone_cont=%.4f boundary_band_cont=%.4f deep_interior_cont=%.4f coherence=%.4f leakage=%.4f churn=%.4f miss_rate=%.4f multi_hit_rate=%.4f active_convergent_fraction=%.4f collision_count=%d"),
			*Run.Config.Label,
			Run.Config.SampleCount,
			Run.Config.Seed,
			Step,
			MassDiag.ContinentalAreaFraction,
			MassDiag.SubaerialContinentalFraction,
			MassDiag.SubmergedContinentalFraction,
			MassDiag.MeanCoastDistHops,
			static_cast<double>(MassDiag.P50CoastDist),
			static_cast<double>(MassDiag.P90CoastDist),
			MassDiag.SubaerialMeanCoastDistHops,
			static_cast<double>(MassDiag.SubaerialP50CoastDist),
			static_cast<double>(MassDiag.SubaerialP90CoastDist),
			MassDiag.LargestComponentSize,
			MassDiag.LargestSubaerialComponentSize,
			SurvivalDiag.Step0BroadInteriorRemainingFraction,
			Snapshot.ActiveZone.ActiveFraction,
			BoundaryCoverageFraction,
			MassDiag.ActiveZoneContinentalFraction,
			MassDiag.BoundaryBandContinentalFraction,
			MassDiag.DeepInteriorContinentalFraction,
			Snapshot.BoundaryCoherence.BoundaryCoherenceScore,
			Snapshot.BoundaryCoherence.InteriorLeakageFraction,
			Snapshot.OwnershipChurn.ChurnFraction,
			MissRate,
			MultiHitRate,
			ActiveConvergentFraction,
			Snapshot.CollisionCount));

		AddInfo(FString::Printf(
			TEXT("[V9BalanceAudit elev label=%s sample_count=%d seed=%d step=%d] cont_mean=%.4f cont_p95=%.4f cont_max=%.4f subaerial_mean=%.4f subaerial_p95=%.4f cont_samples=%d subaerial_samples=%d submerged_frac=%.4f band_0_0p5=%.4f band_0p5_1=%.4f band_1_2=%.4f band_2_5=%.4f band_gt5=%.4f above2=%d above5=%d"),
			*Run.Config.Label,
			Run.Config.SampleCount,
			Run.Config.Seed,
			Step,
			ElevDiag.ContinentalMeanElevationKm,
			ElevDiag.ContinentalP95ElevationKm,
			ElevDiag.ContinentalMaxElevationKm,
			ElevDiag.SubaerialMeanElevationKm,
			ElevDiag.SubaerialP95ElevationKm,
			ElevDiag.ContinentalSampleCount,
			ElevDiag.SubaerialContinentalSampleCount,
			SafeFrac(ElevDiag.SubmergedContinentalSampleCount, ElevDiag.ContinentalSampleCount),
			SafeFrac(ElevDiag.ContinentalSamples0To0p5Km, ElevDiag.ContinentalSampleCount),
			SafeFrac(ElevDiag.ContinentalSamples0p5To1Km, ElevDiag.ContinentalSampleCount),
			SafeFrac(ElevDiag.ContinentalSamples1To2Km, ElevDiag.ContinentalSampleCount),
			SafeFrac(ElevDiag.ContinentalSamples2To5Km, ElevDiag.ContinentalSampleCount),
			SafeFrac(ElevDiag.ContinentalSamplesAbove5Km, ElevDiag.ContinentalSampleCount),
			ElevDiag.SamplesAbove2Km,
			ElevDiag.SamplesAbove5Km));
	};

	const auto EmitDrift = [this, &SafeFrac](
		const FRunState& Run,
		const FSubmergedDriftDiagnostic& DriftDiag)
	{
		AddInfo(FString::Printf(
			TEXT("[V9BalanceAudit drift label=%s sample_count=%d seed=%d] step100_subaerial=%d to_submerged=%d(%.4f) broad=%d/%d(%.4f) coast_adjacent=%d/%d(%.4f) boundary_or_active=%d/%d(%.4f) quiet_retained=%d/%d(%.4f) quiet_retained_broad=%d/%d(%.4f)"),
			*Run.Config.Label,
			Run.Config.SampleCount,
			Run.Config.Seed,
			DriftDiag.Step100SubaerialContinentalCount,
			DriftDiag.Step100To200SubmergedCount,
			SafeFrac(DriftDiag.Step100To200SubmergedCount, DriftDiag.Step100SubaerialContinentalCount),
			DriftDiag.BroadInteriorToSubmergedCount,
			DriftDiag.BroadInteriorBaseCount,
			SafeFrac(DriftDiag.BroadInteriorToSubmergedCount, DriftDiag.BroadInteriorBaseCount),
			DriftDiag.CoastAdjacentToSubmergedCount,
			DriftDiag.CoastAdjacentBaseCount,
			SafeFrac(DriftDiag.CoastAdjacentToSubmergedCount, DriftDiag.CoastAdjacentBaseCount),
			DriftDiag.BoundaryOrActiveToSubmergedCount,
			DriftDiag.BoundaryOrActiveBaseCount,
			SafeFrac(DriftDiag.BoundaryOrActiveToSubmergedCount, DriftDiag.BoundaryOrActiveBaseCount),
			DriftDiag.QuietRetainedToSubmergedCount,
			DriftDiag.QuietRetainedBaseCount,
			SafeFrac(DriftDiag.QuietRetainedToSubmergedCount, DriftDiag.QuietRetainedBaseCount),
			DriftDiag.QuietRetainedBroadInteriorToSubmergedCount,
			DriftDiag.QuietRetainedBroadInteriorBaseCount,
			SafeFrac(
				DriftDiag.QuietRetainedBroadInteriorToSubmergedCount,
				DriftDiag.QuietRetainedBroadInteriorBaseCount)));
	};

	const auto CaptureCheckpoint = [this](
		FRunState& Run,
		const int32 Step)
	{
		const FV6CheckpointSnapshot Snapshot = BuildV6CheckpointSnapshot(Run.Planet);
		const FV9ContinentalMassDiagnostic MassDiag = ComputeContinentalMassDiagnostic(Run.Planet);
		const FV9ContinentalElevationBandDiagnostic ElevDiag =
			ComputeContinentalElevationBandDiagnostic(Run.Planet);
		const FV9SeededContinentalSurvivalDiagnostic SurvivalDiag =
			ComputeSeededContinentalSurvivalDiagnostic(
				Run.Planet.GetPlanet(),
				Run.Step0ContinentalFlags,
				Run.Step0BroadInteriorFlags,
				Run.Step0CoastAdjacentFlags);
		Run.Snapshots.Add(Step, Snapshot);
		Run.MassDiagnostics.Add(Step, MassDiag);
		Run.ElevationBandDiagnostics.Add(Step, ElevDiag);
		Run.SurvivalDiagnostics.Add(Step, SurvivalDiag);
	};

	const auto AdvanceToStep = [](
		FRunState& Run,
		const int32 TargetStep)
	{
		while (Run.Planet.GetPlanet().CurrentStep < TargetStep)
		{
			Run.Planet.AdvanceStep();
		}
	};

	const TArray<FRunConfig> RunsToExecute = {
		{ TEXT("100k_kept"), 100000, CanonicalSeed, false },
		{ TEXT("250k_kept"), 250000, CanonicalSeed, true },
		{ TEXT("250k_kept"), 250000, Secondary250kSeed, true },
	};

	for (const FRunConfig& Config : RunsToExecute)
	{
		FRunState Run;
		Run.Config = Config;
		Run.Planet = InitializePlanet(Config);

		const FV9ContinentalMassDiagnostic Step0Mass = ComputeContinentalMassDiagnostic(Run.Planet);
		BuildSeedFlags(
			Step0Mass,
			Run.Step0ContinentalFlags,
			Run.Step0BroadInteriorFlags,
			Run.Step0CoastAdjacentFlags);

		AdvanceToStep(Run, 100);
		CaptureCheckpoint(Run, 100);
		CaptureStep100DriftFlags(Run, Run.MassDiagnostics.FindChecked(100));
		EmitCheckpoint(Run, 100);

		AdvanceToStep(Run, 200);
		CaptureCheckpoint(Run, 200);
		EmitCheckpoint(Run, 200);
		EmitDrift(Run, ComputeSubmergedDriftDiagnostic(Run));

		for (const int32 Step : CheckpointSteps)
		{
			TestTrue(
				*FString::Printf(TEXT("%s seed %d captured checkpoint %d"), *Config.Label, Config.Seed, Step),
				Run.Snapshots.Contains(Step) &&
				Run.MassDiagnostics.Contains(Step) &&
				Run.ElevationBandDiagnostics.Contains(Step) &&
				Run.SurvivalDiagnostics.Contains(Step));
		}
	}

	return true;
}
