/**
 * Persistent Prototype D crust/event state.
 *
 * This module is sidecar authority only. Projection, export, and FTectonicPlanet
 * sample fields may observe these records in later slices, but must not own them.
 */
#pragma once

#include "CoreMinimal.h"

enum class ETectonicSidecarCrustEventType : uint8
{
	None,
	DivergentSpreading,
	SubductionConsumption,
};

struct AUROUS_API FSidecarBoundaryEdgeKey
{
	int32 SampleA = INDEX_NONE;
	int32 SampleB = INDEX_NONE;

	static FSidecarBoundaryEdgeKey Make(int32 InSampleA, int32 InSampleB);

	bool IsValid() const { return SampleA != INDEX_NONE && SampleB != INDEX_NONE && SampleA != SampleB; }
	bool operator==(const FSidecarBoundaryEdgeKey& Other) const;
	bool operator<(const FSidecarBoundaryEdgeKey& Other) const;
	uint32 ComputeHash() const;
};

struct AUROUS_API FSidecarOwnerEdge
{
	FSidecarBoundaryEdgeKey Key;
	int32 LowPlateId = INDEX_NONE;
	int32 HighPlateId = INDEX_NONE;
	int32 SampleAOwnerPlateId = INDEX_NONE;
	int32 SampleBOwnerPlateId = INDEX_NONE;
	FVector3d SampleAPosition = FVector3d::ZeroVector;
	FVector3d SampleBPosition = FVector3d::ZeroVector;
	FVector3d Midpoint = FVector3d::ZeroVector;
	double LengthRad = 0.0;
};

struct AUROUS_API FSidecarActiveInterval
{
	double StartT = 0.0;
	double EndT = 0.0;

	bool IsValid(double ToleranceT = 1.0e-9) const;
	uint32 ComputeHash() const;
};

struct AUROUS_API FSidecarActiveEdgeIntervals
{
	FSidecarBoundaryEdgeKey Key;
	TArray<FSidecarActiveInterval> Intervals;

	uint32 ComputeHash() const;
};

struct AUROUS_API FSidecarOceanCrustBirthEdge
{
	FSidecarBoundaryEdgeKey Key;
	FVector3d EndpointA = FVector3d::ZeroVector;
	FVector3d EndpointB = FVector3d::ZeroVector;
	FVector3d BoundaryNormal = FVector3d::ZeroVector;
	double LengthKm = 0.0;
	double NormalSeparationKmPerMy = 0.0;

	uint32 ComputeHash() const;
};

struct AUROUS_API FSidecarDivergentSpreadingEventInput
{
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	FSidecarBoundaryEdgeKey ComponentId;
	int32 RidgeGenerationId = 0;
	int32 ParentEventId = INDEX_NONE;
	TArray<FSidecarOceanCrustBirthEdge> BirthEdges;
	TArray<int32> DebugSampleIds;
	FVector3d RidgeDirection = FVector3d::ZeroVector;
	double CreatedAreaKm2 = 0.0;
	double OceanicAgeMy = 0.0;
	double OceanicThicknessKm = 7.0;
	double ElevationSeedKm = -1.0;
};

struct AUROUS_API FSidecarSubductionConsumptionEventInput
{
	int32 CrustId = INDEX_NONE;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	int32 SubductingPlateId = INDEX_NONE;
	int32 OverridingPlateId = INDEX_NONE;
	TArray<FSidecarBoundaryEdgeKey> SourceEdges;
	TArray<FSidecarActiveEdgeIntervals> ConsumedIntervalsBySourceEdge;
	TArray<int32> DebugSampleIds;
	double ConsumedAreaKm2 = 0.0;
	double ConvergenceSpeedKmPerMy = 0.0;
	double EdgeLengthKm = 0.0;
	double CrustAgeAtConsumptionMy = 0.0;
	double ProjectedElevationAtConsumptionKm = 0.0;
};

struct AUROUS_API FSidecarOceanCrustRecord
{
	int32 CrustId = INDEX_NONE;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	FSidecarBoundaryEdgeKey ComponentId;
	int32 BirthStep = 0;
	double BirthTimeMy = 0.0;
	int32 LastUpdatedStep = 0;
	int32 LastAcceptedStep = INDEX_NONE;
	int32 RidgeGenerationId = 0;
	int32 ParentEventId = INDEX_NONE;
	TArray<FSidecarOceanCrustBirthEdge> BirthEdges;
	TArray<FSidecarBoundaryEdgeKey> LastAcceptedSourceEdges;
	TArray<FVector3d> LastAcceptedEdgeMidpoints;
	TArray<int32> DebugSampleIds;
	FVector3d RidgeDirection = FVector3d::ZeroVector;
	double CreatedAreaKm2 = 0.0;
	double AgeMy = 0.0;
	double ThicknessKm = 7.0;
	double ElevationSeedKm = -1.0;
	double ActiveAreaKm2 = 0.0;
	double ConsumedAreaKm2 = 0.0;
	double ConsumedFraction = 0.0;
	int32 LastConsumedStep = INDEX_NONE;
	int32 FullyConsumedStep = INDEX_NONE;
	bool bIsFullyConsumed = false;
	TArray<FSidecarActiveEdgeIntervals> ActiveIntervalsBySourceEdge;

	uint32 ComputeHash() const;
};

struct AUROUS_API FSidecarCrustEventRecord
{
	int32 EventId = INDEX_NONE;
	ETectonicSidecarCrustEventType EventType = ETectonicSidecarCrustEventType::None;
	int32 CrustId = INDEX_NONE;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	FSidecarBoundaryEdgeKey ComponentId;
	int32 Step = 0;
	double TimeMy = 0.0;
	int32 RidgeGenerationId = 0;
	int32 ParentEventId = INDEX_NONE;
	TArray<FSidecarBoundaryEdgeKey> SourceEdges;
	double CreatedAreaKm2 = 0.0;
	double NormalSeparationKmPerMy = 0.0;
	TArray<int32> DebugSampleIds;
	int32 SubductingPlateId = INDEX_NONE;
	int32 OverridingPlateId = INDEX_NONE;
	double PreActiveAreaKm2 = 0.0;
	double PostActiveAreaKm2 = 0.0;
	double PreConsumedAreaKm2 = 0.0;
	double PostConsumedAreaKm2 = 0.0;
	double ConsumedAreaKm2 = 0.0;
	double ConsumedFraction = 0.0;
	TArray<FSidecarActiveEdgeIntervals> ConsumedIntervalDeltasBySourceEdge;
	TArray<FSidecarActiveEdgeIntervals> PostConsumptionActiveIntervalsBySourceEdge;
	double ConvergenceSpeedKmPerMy = 0.0;
	double EdgeLengthKm = 0.0;
	double CrustAgeAtConsumptionMy = 0.0;
	double ProjectedElevationAtConsumptionKm = 0.0;

	uint32 ComputeHash() const;
};

struct AUROUS_API FSidecarOceanCrustStore
{
	int32 NextCrustId = 1;
	TArray<FSidecarOceanCrustRecord> Records;

	void Reset();
	int32 Num() const { return Records.Num(); }
	uint32 ComputeCanonicalHash() const;
};

struct AUROUS_API FSidecarCrustEventLog
{
	int32 NextEventId = 1;
	TArray<FSidecarCrustEventRecord> Events;

	void Reset();
	int32 Num() const { return Events.Num(); }
	uint32 ComputeAppendOrderHash() const;
};

AUROUS_API bool ApplyDivergentSpreadingEvent(
	FSidecarOceanCrustStore& InOutStore,
	FSidecarCrustEventLog& InOutEventLog,
	const FSidecarDivergentSpreadingEventInput& Input,
	int32 CurrentStep,
	double CurrentTimeMy,
	int32 RidgeGenerationGapSteps,
	double CoalescingToleranceRad,
	int32* OutCrustId = nullptr,
	int32* OutEventId = nullptr);

AUROUS_API void CanonicalizeActiveEdgeIntervals(
	TArray<FSidecarActiveEdgeIntervals>& InOutIntervalsBySourceEdge,
	double ActiveIntervalToleranceT = 1.0e-9);

AUROUS_API bool ApplySubductionConsumptionEvent(
	FSidecarOceanCrustStore& InOutStore,
	FSidecarCrustEventLog& InOutEventLog,
	const FSidecarSubductionConsumptionEventInput& Input,
	int32 CurrentStep,
	double CurrentTimeMy,
	double ActiveIntervalToleranceT,
	double AreaToleranceKm2,
	int32* OutEventId = nullptr);

AUROUS_API bool ReplaySubductionConsumptionEvents(
	const FSidecarOceanCrustStore& BaselineStore,
	const FSidecarCrustEventLog& EventLog,
	FSidecarOceanCrustStore& OutReplayedStore,
	double ActiveIntervalToleranceT = 1.0e-9,
	double AreaToleranceKm2 = 1.0e-6);
