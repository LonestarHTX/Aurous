#include "TectonicSidecarCrust.h"

#include "Algo/Sort.h"

namespace
{
	uint32 MixCrustHashValue(uint32 Hash, const uint32 Value)
	{
		Hash ^= Value;
		Hash *= 16777619u;
		return Hash;
	}

	void MixCrustHashInt32(uint32& Hash, const int32 Value)
	{
		Hash = MixCrustHashValue(Hash, static_cast<uint32>(Value));
	}

	void MixCrustHashDouble(uint32& Hash, const double Value)
	{
		uint64 Bits = 0;
		FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		Hash = MixCrustHashValue(Hash, static_cast<uint32>(Bits & 0xffffffffu));
		Hash = MixCrustHashValue(Hash, static_cast<uint32>(Bits >> 32));
	}

	void MixCrustHashVector(uint32& Hash, const FVector3d& Value)
	{
		MixCrustHashDouble(Hash, Value.X);
		MixCrustHashDouble(Hash, Value.Y);
		MixCrustHashDouble(Hash, Value.Z);
	}
}

FSidecarBoundaryEdgeKey FSidecarBoundaryEdgeKey::Make(const int32 InSampleA, const int32 InSampleB)
{
	FSidecarBoundaryEdgeKey Key;
	Key.SampleA = FMath::Min(InSampleA, InSampleB);
	Key.SampleB = FMath::Max(InSampleA, InSampleB);
	return Key;
}

bool FSidecarBoundaryEdgeKey::operator==(const FSidecarBoundaryEdgeKey& Other) const
{
	return SampleA == Other.SampleA && SampleB == Other.SampleB;
}

bool FSidecarBoundaryEdgeKey::operator<(const FSidecarBoundaryEdgeKey& Other) const
{
	if (SampleA != Other.SampleA)
	{
		return SampleA < Other.SampleA;
	}
	return SampleB < Other.SampleB;
}

uint32 FSidecarBoundaryEdgeKey::ComputeHash() const
{
	uint32 Hash = 2166136261u;
	MixCrustHashInt32(Hash, SampleA);
	MixCrustHashInt32(Hash, SampleB);
	return Hash;
}

uint32 FSidecarOceanCrustBirthEdge::ComputeHash() const
{
	uint32 Hash = 2166136261u;
	Hash = MixCrustHashValue(Hash, Key.ComputeHash());
	MixCrustHashVector(Hash, EndpointA);
	MixCrustHashVector(Hash, EndpointB);
	MixCrustHashVector(Hash, BoundaryNormal);
	MixCrustHashDouble(Hash, LengthKm);
	MixCrustHashDouble(Hash, NormalSeparationKmPerMy);
	return Hash;
}

uint32 FSidecarOceanCrustRecord::ComputeHash() const
{
	uint32 Hash = 2166136261u;
	MixCrustHashInt32(Hash, CrustId);
	MixCrustHashInt32(Hash, PlateA);
	MixCrustHashInt32(Hash, PlateB);
	MixCrustHashInt32(Hash, BirthStep);
	MixCrustHashDouble(Hash, BirthTimeMy);
	MixCrustHashInt32(Hash, LastUpdatedStep);
	MixCrustHashInt32(Hash, RidgeGenerationId);
	MixCrustHashInt32(Hash, ParentEventId);
	MixCrustHashInt32(Hash, BirthEdges.Num());
	for (const FSidecarOceanCrustBirthEdge& Edge : BirthEdges)
	{
		Hash = MixCrustHashValue(Hash, Edge.ComputeHash());
	}
	MixCrustHashInt32(Hash, DebugSampleIds.Num());
	for (const int32 SampleId : DebugSampleIds)
	{
		MixCrustHashInt32(Hash, SampleId);
	}
	MixCrustHashVector(Hash, RidgeDirection);
	MixCrustHashDouble(Hash, CreatedAreaKm2);
	MixCrustHashDouble(Hash, AgeMy);
	MixCrustHashDouble(Hash, ThicknessKm);
	MixCrustHashDouble(Hash, ElevationSeedKm);
	return Hash;
}

uint32 FSidecarCrustEventRecord::ComputeHash() const
{
	uint32 Hash = 2166136261u;
	MixCrustHashInt32(Hash, EventId);
	MixCrustHashInt32(Hash, static_cast<int32>(EventType));
	MixCrustHashInt32(Hash, CrustId);
	MixCrustHashInt32(Hash, PlateA);
	MixCrustHashInt32(Hash, PlateB);
	MixCrustHashInt32(Hash, Step);
	MixCrustHashDouble(Hash, TimeMy);
	MixCrustHashInt32(Hash, RidgeGenerationId);
	MixCrustHashInt32(Hash, ParentEventId);
	MixCrustHashInt32(Hash, SourceEdges.Num());
	for (const FSidecarBoundaryEdgeKey& Edge : SourceEdges)
	{
		Hash = MixCrustHashValue(Hash, Edge.ComputeHash());
	}
	MixCrustHashDouble(Hash, CreatedAreaKm2);
	MixCrustHashDouble(Hash, NormalSeparationKmPerMy);
	return Hash;
}

void FSidecarOceanCrustStore::Reset()
{
	NextCrustId = 1;
	Records.Reset();
}

uint32 FSidecarOceanCrustStore::ComputeCanonicalHash() const
{
	uint32 Hash = 2166136261u;
	MixCrustHashInt32(Hash, NextCrustId);
	TArray<const FSidecarOceanCrustRecord*> SortedRecords;
	SortedRecords.Reserve(Records.Num());
	for (const FSidecarOceanCrustRecord& Record : Records)
	{
		SortedRecords.Add(&Record);
	}
	Algo::Sort(SortedRecords, [](const FSidecarOceanCrustRecord* A, const FSidecarOceanCrustRecord* B)
	{
		return A->CrustId < B->CrustId;
	});
	MixCrustHashInt32(Hash, SortedRecords.Num());
	for (const FSidecarOceanCrustRecord* Record : SortedRecords)
	{
		Hash = MixCrustHashValue(Hash, Record->ComputeHash());
	}
	return Hash;
}

void FSidecarCrustEventLog::Reset()
{
	NextEventId = 1;
	Events.Reset();
}

uint32 FSidecarCrustEventLog::ComputeAppendOrderHash() const
{
	uint32 Hash = 2166136261u;
	MixCrustHashInt32(Hash, NextEventId);
	MixCrustHashInt32(Hash, Events.Num());
	for (const FSidecarCrustEventRecord& Event : Events)
	{
		Hash = MixCrustHashValue(Hash, Event.ComputeHash());
	}
	return Hash;
}
