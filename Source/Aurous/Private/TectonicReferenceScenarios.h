#pragma once

#include "CoreMinimal.h"
#include "TectonicPlanet.h"

constexpr double MaxAcceptableGapFraction = 0.02;
constexpr double MaxAcceptableOverlapFraction = 0.12;
constexpr double MaxAcceptableBoundaryMeanDepthHops = 1.25;
constexpr int32 MaxAcceptableBoundaryMaxDepthHops = 6;
constexpr double MaxContinentalFractionIncreaseEpsilon = 1.0e-5;
constexpr double MaxAcceptableUnaccountedPct = 10.0;
constexpr double MaxReferenceScenarioElevationCeilingKm = 10.0;

struct FReferenceScenarioBounds
{
	int32 ExpectedInitialContinentalPlateCount = 0;
	double InitialContinentalAreaFractionMin = 0.25;
	double InitialContinentalAreaFractionMax = 0.40;
	double MinimumRuntimeContinentalAreaFraction = 0.25;
	int32 MinimumCollisionEventCount = 0;
	bool bRequireAndeanWhenOceanicContinentalConvergenceExists = true;
	bool bRequireHimalayanWhenCollisionOccurs = true;
	bool bPreRiftingFloorExempt = false;
};

struct FReferenceScenarioDefinition
{
	const TCHAR* Name = TEXT("Unnamed");
	int32 SampleCount = 500000;
	int32 PlateCount = 7;
	int32 Seed = 42;
	int32 NumSteps = 60;
	FReferenceScenarioBounds Bounds;
};

struct FReferenceScenarioFailureObservation
{
	FString Label;
	int32 StepIndex = INDEX_NONE;
	FString Detail;
};

struct FReferenceScenarioObservedMetrics
{
	int32 InitialContinentalPlateCount = 0;
	double InitialContinentalAreaFraction = 0.0;
	double MinimumRuntimeContinentalAreaFraction = 1.0;
	double MaximumObservedElevationKm = -TNumericLimits<double>::Max();
	double MaximumObservedContinentalElevationKm = -TNumericLimits<double>::Max();
	int32 MaximumContinentalComponentCount = 0;
	int32 FinalCollisionEventCount = 0;
	bool bObservedOceanicContinentalConvergence = false;
	bool bObservedAndean = false;
	bool bObservedHimalayan = false;
	int32 ReconcileSteps = 0;
	int32 InvalidPlateAssignments = 0;
	int32 InvalidPreviousAssignments = 0;
	int32 InvalidTimingSnapshots = 0;
	int32 StabilizerShadowMismatchCount = 0;
	int32 UnaccountedBudgetFailures = 0;
	int32 PositionDriftCount = 0;
	int32 PingPongTransitions = 0;
	int32 GapOrphanMismatchCount = 0;
	double MaxGapFraction = 0.0;
	double MaxOverlapFraction = 0.0;
	double MaxBoundaryMeanDepthHops = 0.0;
	int32 MaxBoundaryMaxDepthHops = 0;
	int32 MaxNonGapPlateComponentCount = 0;
	int32 MaxDetachedPlateFragmentSampleCount = 0;
	int32 MaxLargestDetachedPlateFragmentSize = 0;
	double MinLargestContinentalShare = 1.0;
	int32 MinObservedProtectedPlateSampleCount = TNumericLimits<int32>::Max();
	int32 MaxEmptyProtectedPlateCount = 0;
	int32 MinExpectedProtectedPlateFloor = 0;
	double PingPongRate = 0.0;
	double StartupToMainMs = 0.0;
	double ScenarioWallMs = 0.0;
	double AverageReconcileMs = 0.0;
	double MaxReconcileMs = 0.0;
	double AverageUnaccountedPct = 0.0;
	double MaxUnaccountedPct = 0.0;
	bool bRuntimeContinentalFractionIncreased = false;
	TArray<FReferenceScenarioFailureObservation> FailureObservations;
};

int32 ComputeExpectedInitialPlateFloorSamples(int32 TotalSampleCount, int32 PlateCount);
int32 ComputeExpectedPersistentPlateFloorSamples(const FPlate& Plate);
int32 GetExpectedMaximumContinentalComponentCount(const FReferenceScenarioBounds& Bounds);
const TArray<FReferenceScenarioDefinition>& GetLockedReferenceScenarios();
const FReferenceScenarioDefinition* FindLockedReferenceScenarioByName(const FString& ScenarioName);
bool CollectReferenceScenarioObservedMetrics(
	const FReferenceScenarioDefinition& Scenario,
	const FTectonicPlanet& BasePlanet,
	FReferenceScenarioObservedMetrics& OutMetrics,
	double StartupToMainMs = 0.0,
	EContinentalStabilizerMode StabilizerMode = EContinentalStabilizerMode::Incremental);
FString FormatReferenceScenarioSummary(
	const FReferenceScenarioDefinition& Scenario,
	const FReferenceScenarioObservedMetrics& Metrics,
	bool bPass);
FString DescribeReferenceScenarioFailures(
	const FReferenceScenarioDefinition& Scenario,
	const FReferenceScenarioObservedMetrics& Metrics);
FString DescribeReferenceScenarioFailureDetails(const FReferenceScenarioObservedMetrics& Metrics);
bool DoesReferenceScenarioObservedMetricsPass(
	const FReferenceScenarioDefinition& Scenario,
	const FReferenceScenarioObservedMetrics& Metrics);
bool TryDiscoverLowestPassingReferenceScenarioSeed(
	const FReferenceScenarioDefinition& ScenarioTemplate,
	const FTectonicPlanet& BasePlanet,
	int32& OutSeed,
	FReferenceScenarioObservedMetrics& OutMetrics);



