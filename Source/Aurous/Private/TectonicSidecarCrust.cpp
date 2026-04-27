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

	FVector3d NormalizeOrFallbackForCrust(const FVector3d& Value, const FVector3d& Fallback)
	{
		const FVector3d Normalized = Value.GetSafeNormal();
		if (!Normalized.IsNearlyZero())
		{
			return Normalized;
		}
		const FVector3d FallbackNormalized = Fallback.GetSafeNormal();
		return !FallbackNormalized.IsNearlyZero() ? FallbackNormalized : FVector3d(1.0, 0.0, 0.0);
	}

	double AngularDistanceRadForCrust(const FVector3d& A, const FVector3d& B)
	{
		const FVector3d UnitA = A.GetSafeNormal();
		const FVector3d UnitB = B.GetSafeNormal();
		if (UnitA.IsNearlyZero() || UnitB.IsNearlyZero())
		{
			return TNumericLimits<double>::Max();
		}
		return FMath::Acos(FMath::Clamp(UnitA.Dot(UnitB), -1.0, 1.0));
	}

	FVector3d ComputeBirthEdgeMidpoint(const FSidecarOceanCrustBirthEdge& Edge)
	{
		return NormalizeOrFallbackForCrust(Edge.EndpointA + Edge.EndpointB, Edge.EndpointA);
	}

	void CanonicalizeBirthEdges(TArray<FSidecarOceanCrustBirthEdge>& BirthEdges)
	{
		for (FSidecarOceanCrustBirthEdge& Edge : BirthEdges)
		{
			Edge.Key = FSidecarBoundaryEdgeKey::Make(Edge.Key.SampleA, Edge.Key.SampleB);
			Edge.EndpointA = NormalizeOrFallbackForCrust(Edge.EndpointA, FVector3d(1.0, 0.0, 0.0));
			Edge.EndpointB = NormalizeOrFallbackForCrust(Edge.EndpointB, Edge.EndpointA);
			Edge.BoundaryNormal = NormalizeOrFallbackForCrust(
				Edge.BoundaryNormal,
				FVector3d::CrossProduct(Edge.EndpointA, Edge.EndpointB));
		}
		Algo::Sort(BirthEdges, [](const FSidecarOceanCrustBirthEdge& A, const FSidecarOceanCrustBirthEdge& B)
		{
			return A.Key < B.Key;
		});
	}

	TArray<FSidecarBoundaryEdgeKey> BuildSourceEdgesFromBirthEdges(const TArray<FSidecarOceanCrustBirthEdge>& BirthEdges)
	{
		TArray<FSidecarBoundaryEdgeKey> SourceEdges;
		SourceEdges.Reserve(BirthEdges.Num());
		for (const FSidecarOceanCrustBirthEdge& Edge : BirthEdges)
		{
			if (SourceEdges.IsEmpty() || !(SourceEdges.Last() == Edge.Key))
			{
				SourceEdges.Add(Edge.Key);
			}
		}
		return SourceEdges;
	}

	TArray<FVector3d> BuildEdgeMidpointsFromBirthEdges(const TArray<FSidecarOceanCrustBirthEdge>& BirthEdges)
	{
		TArray<FVector3d> Midpoints;
		Midpoints.Reserve(BirthEdges.Num());
		for (const FSidecarOceanCrustBirthEdge& Edge : BirthEdges)
		{
			Midpoints.Add(ComputeBirthEdgeMidpoint(Edge));
		}
		return Midpoints;
	}

	double ComputeMeanNormalSeparationKmPerMy(const TArray<FSidecarOceanCrustBirthEdge>& BirthEdges)
	{
		if (BirthEdges.IsEmpty())
		{
			return 0.0;
		}
		double Sum = 0.0;
		for (const FSidecarOceanCrustBirthEdge& Edge : BirthEdges)
		{
			Sum += Edge.NormalSeparationKmPerMy;
		}
		return Sum / static_cast<double>(BirthEdges.Num());
	}

	bool HasSharedEdge(
		const TArray<FSidecarBoundaryEdgeKey>& A,
		const TArray<FSidecarBoundaryEdgeKey>& B)
	{
		int32 IndexA = 0;
		int32 IndexB = 0;
		while (A.IsValidIndex(IndexA) && B.IsValidIndex(IndexB))
		{
			if (A[IndexA] == B[IndexB])
			{
				return true;
			}
			if (A[IndexA] < B[IndexB])
			{
				++IndexA;
			}
			else
			{
				++IndexB;
			}
		}
		return false;
	}

	double ComputeClosestMidpointDistanceRad(
		const TArray<FVector3d>& A,
		const TArray<FVector3d>& B)
	{
		double BestDistanceRad = TNumericLimits<double>::Max();
		for (const FVector3d& MidpointA : A)
		{
			for (const FVector3d& MidpointB : B)
			{
				BestDistanceRad = FMath::Min(
					BestDistanceRad,
					AngularDistanceRadForCrust(MidpointA, MidpointB));
			}
		}
		return BestDistanceRad;
	}

	void AddSortedUniqueDebugSamples(
		TArray<int32>& InOutDebugSamples,
		const TArray<int32>& NewDebugSamples)
	{
		InOutDebugSamples.Append(NewDebugSamples);
		Algo::Sort(InOutDebugSamples);
		int32 WriteIndex = 0;
		for (int32 ReadIndex = 0; ReadIndex < InOutDebugSamples.Num(); ++ReadIndex)
		{
			if (WriteIndex == 0 || InOutDebugSamples[ReadIndex] != InOutDebugSamples[WriteIndex - 1])
			{
				if (WriteIndex != ReadIndex)
				{
					InOutDebugSamples[WriteIndex] = InOutDebugSamples[ReadIndex];
				}
				++WriteIndex;
			}
		}
		InOutDebugSamples.SetNum(WriteIndex);
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
	Hash = MixCrustHashValue(Hash, ComponentId.ComputeHash());
	MixCrustHashInt32(Hash, BirthStep);
	MixCrustHashDouble(Hash, BirthTimeMy);
	MixCrustHashInt32(Hash, LastUpdatedStep);
	MixCrustHashInt32(Hash, LastAcceptedStep);
	MixCrustHashInt32(Hash, RidgeGenerationId);
	MixCrustHashInt32(Hash, ParentEventId);
	MixCrustHashInt32(Hash, BirthEdges.Num());
	for (const FSidecarOceanCrustBirthEdge& Edge : BirthEdges)
	{
		Hash = MixCrustHashValue(Hash, Edge.ComputeHash());
	}
	MixCrustHashInt32(Hash, LastAcceptedSourceEdges.Num());
	for (const FSidecarBoundaryEdgeKey& Edge : LastAcceptedSourceEdges)
	{
		Hash = MixCrustHashValue(Hash, Edge.ComputeHash());
	}
	MixCrustHashInt32(Hash, LastAcceptedEdgeMidpoints.Num());
	for (const FVector3d& Midpoint : LastAcceptedEdgeMidpoints)
	{
		MixCrustHashVector(Hash, Midpoint);
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
	Hash = MixCrustHashValue(Hash, ComponentId.ComputeHash());
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

bool ApplyDivergentSpreadingEvent(
	FSidecarOceanCrustStore& InOutStore,
	FSidecarCrustEventLog& InOutEventLog,
	const FSidecarDivergentSpreadingEventInput& Input,
	const int32 CurrentStep,
	const double CurrentTimeMy,
	const int32 RidgeGenerationGapSteps,
	const double CoalescingToleranceRad,
	int32* OutCrustId,
	int32* OutEventId)
{
	if (OutCrustId != nullptr)
	{
		*OutCrustId = INDEX_NONE;
	}
	if (OutEventId != nullptr)
	{
		*OutEventId = INDEX_NONE;
	}

	if (Input.PlateA == INDEX_NONE ||
		Input.PlateB == INDEX_NONE ||
		Input.PlateA == Input.PlateB ||
		Input.BirthEdges.IsEmpty())
	{
		return false;
	}

	const int32 LowPlateId = FMath::Min(Input.PlateA, Input.PlateB);
	const int32 HighPlateId = FMath::Max(Input.PlateA, Input.PlateB);
	TArray<FSidecarOceanCrustBirthEdge> BirthEdges = Input.BirthEdges;
	CanonicalizeBirthEdges(BirthEdges);
	for (const FSidecarOceanCrustBirthEdge& BirthEdge : BirthEdges)
	{
		if (!BirthEdge.Key.IsValid())
		{
			return false;
		}
	}

	TArray<FSidecarBoundaryEdgeKey> SourceEdges = BuildSourceEdgesFromBirthEdges(BirthEdges);
	TArray<FVector3d> CurrentMidpoints = BuildEdgeMidpointsFromBirthEdges(BirthEdges);
	TArray<int32> DebugSampleIds = Input.DebugSampleIds;
	Algo::Sort(DebugSampleIds);

	TArray<int32> CandidateRecordIndices;
	CandidateRecordIndices.Reserve(InOutStore.Records.Num());
	for (int32 RecordIndex = 0; RecordIndex < InOutStore.Records.Num(); ++RecordIndex)
	{
		const FSidecarOceanCrustRecord& Record = InOutStore.Records[RecordIndex];
		if (Record.PlateA == LowPlateId && Record.PlateB == HighPlateId)
		{
			CandidateRecordIndices.Add(RecordIndex);
		}
	}
	Algo::Sort(CandidateRecordIndices, [&InOutStore](const int32 A, const int32 B)
	{
		return InOutStore.Records[A].CrustId < InOutStore.Records[B].CrustId;
	});

	int32 MatchedRecordIndex = INDEX_NONE;
	for (const int32 RecordIndex : CandidateRecordIndices)
	{
		const FSidecarOceanCrustRecord& Record = InOutStore.Records[RecordIndex];
		if (HasSharedEdge(SourceEdges, Record.LastAcceptedSourceEdges))
		{
			MatchedRecordIndex = RecordIndex;
			break;
		}
	}

	if (MatchedRecordIndex == INDEX_NONE)
	{
		double BestDistanceRad = TNumericLimits<double>::Max();
		for (const int32 RecordIndex : CandidateRecordIndices)
		{
			const FSidecarOceanCrustRecord& Record = InOutStore.Records[RecordIndex];
			const double DistanceRad = ComputeClosestMidpointDistanceRad(CurrentMidpoints, Record.LastAcceptedEdgeMidpoints);
			if (DistanceRad <= CoalescingToleranceRad &&
				(MatchedRecordIndex == INDEX_NONE ||
					DistanceRad < BestDistanceRad ||
					(FMath::IsNearlyEqual(DistanceRad, BestDistanceRad) &&
						InOutStore.Records[RecordIndex].CrustId < InOutStore.Records[MatchedRecordIndex].CrustId)))
			{
				BestDistanceRad = DistanceRad;
				MatchedRecordIndex = RecordIndex;
			}
		}
	}

	bool bExtendExistingRecord = false;
	int32 RidgeGenerationId = Input.RidgeGenerationId;
	if (MatchedRecordIndex != INDEX_NONE)
	{
		const FSidecarOceanCrustRecord& MatchedRecord = InOutStore.Records[MatchedRecordIndex];
		const int32 InactiveSteps = CurrentStep - MatchedRecord.LastAcceptedStep;
		if (InactiveSteps <= RidgeGenerationGapSteps)
		{
			bExtendExistingRecord = true;
			RidgeGenerationId = MatchedRecord.RidgeGenerationId;
		}
		else
		{
			RidgeGenerationId = MatchedRecord.RidgeGenerationId + 1;
		}
	}

	FSidecarOceanCrustRecord* TargetRecord = nullptr;
	if (bExtendExistingRecord)
	{
		TargetRecord = &InOutStore.Records[MatchedRecordIndex];
		TargetRecord->LastUpdatedStep = CurrentStep;
		TargetRecord->LastAcceptedStep = CurrentStep;
		TargetRecord->BirthEdges.Append(BirthEdges);
		TargetRecord->LastAcceptedSourceEdges = SourceEdges;
		TargetRecord->LastAcceptedEdgeMidpoints = CurrentMidpoints;
		AddSortedUniqueDebugSamples(TargetRecord->DebugSampleIds, DebugSampleIds);
		TargetRecord->CreatedAreaKm2 += Input.CreatedAreaKm2;
		TargetRecord->AgeMy = FMath::Max(0.0, CurrentTimeMy - TargetRecord->BirthTimeMy);
	}
	else
	{
		FSidecarOceanCrustRecord NewRecord;
		NewRecord.CrustId = InOutStore.NextCrustId++;
		NewRecord.PlateA = LowPlateId;
		NewRecord.PlateB = HighPlateId;
		NewRecord.ComponentId = Input.ComponentId.IsValid() ? Input.ComponentId : SourceEdges[0];
		NewRecord.BirthStep = CurrentStep;
		NewRecord.BirthTimeMy = CurrentTimeMy;
		NewRecord.LastUpdatedStep = CurrentStep;
		NewRecord.LastAcceptedStep = CurrentStep;
		NewRecord.RidgeGenerationId = RidgeGenerationId;
		NewRecord.ParentEventId = Input.ParentEventId;
		NewRecord.BirthEdges = BirthEdges;
		NewRecord.LastAcceptedSourceEdges = SourceEdges;
		NewRecord.LastAcceptedEdgeMidpoints = CurrentMidpoints;
		NewRecord.DebugSampleIds = DebugSampleIds;
		NewRecord.RidgeDirection = NormalizeOrFallbackForCrust(Input.RidgeDirection, BirthEdges[0].BoundaryNormal);
		NewRecord.CreatedAreaKm2 = Input.CreatedAreaKm2;
		NewRecord.AgeMy = Input.OceanicAgeMy;
		NewRecord.ThicknessKm = Input.OceanicThicknessKm;
		NewRecord.ElevationSeedKm = Input.ElevationSeedKm;
		InOutStore.Records.Add(NewRecord);
		TargetRecord = &InOutStore.Records.Last();
	}

	const int32 EventId = InOutEventLog.NextEventId++;
	FSidecarCrustEventRecord EventRecord;
	EventRecord.EventId = EventId;
	EventRecord.EventType = ETectonicSidecarCrustEventType::DivergentSpreading;
	EventRecord.CrustId = TargetRecord->CrustId;
	EventRecord.PlateA = LowPlateId;
	EventRecord.PlateB = HighPlateId;
	EventRecord.ComponentId = TargetRecord->ComponentId;
	EventRecord.Step = CurrentStep;
	EventRecord.TimeMy = CurrentTimeMy;
	EventRecord.RidgeGenerationId = TargetRecord->RidgeGenerationId;
	EventRecord.ParentEventId = TargetRecord->ParentEventId;
	EventRecord.SourceEdges = SourceEdges;
	EventRecord.CreatedAreaKm2 = Input.CreatedAreaKm2;
	EventRecord.NormalSeparationKmPerMy = ComputeMeanNormalSeparationKmPerMy(BirthEdges);
	InOutEventLog.Events.Add(EventRecord);

	if (OutCrustId != nullptr)
	{
		*OutCrustId = TargetRecord->CrustId;
	}
	if (OutEventId != nullptr)
	{
		*OutEventId = EventId;
	}
	return true;
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
