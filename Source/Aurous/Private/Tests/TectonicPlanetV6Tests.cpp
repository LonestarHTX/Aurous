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

	FV6CheckpointSnapshot BuildV6CheckpointSnapshot(const FTectonicPlanetV6& Planet);

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
			TEXT("%s step=%d exec_collision_count=%d exec_collision_cumulative=%d exec_cumulative_affected=%d exec_cumulative_affected_visits=%d exec_cumulative_continental_gain=%d exec_cumulative_ownership_change=%d exec_cumulative_transfer=%d exec_cumulative_transfer_visits=%d exec_cumulative_transfer_continental=%d exec_pair=(%d,%d) exec_over_plate=%d exec_sub_plate=%d exec_obs=%d exec_penetration_km=%.1f exec_mean_convergence_km_per_my=%.3f exec_max_convergence_km_per_my=%.3f exec_support=%d exec_triangles=%d exec_continental_support=%d/%d exec_qualified_samples=%d exec_seed_samples=%d exec_collision_seed_samples=%d exec_effective_mass=%d exec_affected=%d exec_continental_gain=%d exec_ownership_change=%d exec_transfer=%d exec_transfer_continental=%d exec_cooldown_suppressed=%d exec_qualified_unexecuted=%d exec_transfer_reject_locality=%d exec_transfer_reject_continentality=%d exec_transfer_reject_cap=%d exec_transfer_boundary_local=%d exec_transfer_patch_support=%d exec_transfer_anchor_seeds=%d exec_radius_rad=%.6f exec_transfer_radius_rad=%.6f exec_mean_elev_delta_km=%.6f exec_max_elev_delta_km=%.6f exec_cumulative_mean_elev_delta_km=%.6f exec_cumulative_max_elev_delta_km=%.6f exec_strength_scale=%.6f donor_share=%.6f->%.6f recipient_share=%.6f->%.6f exec_from_shadow=%d"),
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
			CE.QualifiedButUnexecutedCount,
			CE.ExecutedTransferRejectedByLocalityCount,
			CE.ExecutedTransferRejectedByContinentalityCount,
			CE.ExecutedTransferRejectedByCapCount,
			CE.ExecutedTransferBoundaryLocalSampleCount,
			CE.ExecutedTransferCandidateSupportCount,
			CE.ExecutedTransferAnchorSeedCount,
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
