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
