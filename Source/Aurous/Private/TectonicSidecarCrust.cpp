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

	void SortUniqueBoundaryEdgeKeys(TArray<FSidecarBoundaryEdgeKey>& InOutEdges)
	{
		for (FSidecarBoundaryEdgeKey& Edge : InOutEdges)
		{
			Edge = FSidecarBoundaryEdgeKey::Make(Edge.SampleA, Edge.SampleB);
		}
		Algo::Sort(InOutEdges);
		int32 WriteIndex = 0;
		for (int32 ReadIndex = 0; ReadIndex < InOutEdges.Num(); ++ReadIndex)
		{
			if (!InOutEdges[ReadIndex].IsValid())
			{
				continue;
			}
			if (WriteIndex == 0 || !(InOutEdges[ReadIndex] == InOutEdges[WriteIndex - 1]))
			{
				if (WriteIndex != ReadIndex)
				{
					InOutEdges[WriteIndex] = InOutEdges[ReadIndex];
				}
				++WriteIndex;
			}
		}
		InOutEdges.SetNum(WriteIndex);
	}

	void CanonicalizeIntervalArray(TArray<FSidecarActiveInterval>& InOutIntervals, const double ToleranceT)
	{
		for (FSidecarActiveInterval& Interval : InOutIntervals)
		{
			if (!FMath::IsFinite(Interval.StartT) || !FMath::IsFinite(Interval.EndT))
			{
				Interval.StartT = 0.0;
				Interval.EndT = 0.0;
				continue;
			}
			Interval.StartT = FMath::Clamp(Interval.StartT, 0.0, 1.0);
			Interval.EndT = FMath::Clamp(Interval.EndT, 0.0, 1.0);
			if (Interval.EndT < Interval.StartT)
			{
				Swap(Interval.StartT, Interval.EndT);
			}
		}
		Algo::Sort(InOutIntervals, [](const FSidecarActiveInterval& A, const FSidecarActiveInterval& B)
		{
			if (!FMath::IsNearlyEqual(A.StartT, B.StartT))
			{
				return A.StartT < B.StartT;
			}
			return A.EndT < B.EndT;
		});

		TArray<FSidecarActiveInterval> Merged;
		for (const FSidecarActiveInterval& Interval : InOutIntervals)
		{
			if (!Interval.IsValid(ToleranceT))
			{
				continue;
			}
			if (!Merged.IsEmpty() && Interval.StartT - Merged.Last().EndT <= ToleranceT)
			{
				Merged.Last().EndT = FMath::Max(Merged.Last().EndT, Interval.EndT);
			}
			else
			{
				Merged.Add(Interval);
			}
		}
		InOutIntervals = MoveTemp(Merged);
	}

	TArray<FSidecarActiveEdgeIntervals> BuildFullActiveIntervalsFromSourceEdges(
		const TArray<FSidecarBoundaryEdgeKey>& SourceEdges,
		const double ToleranceT)
	{
		TArray<FSidecarActiveEdgeIntervals> ActiveIntervals;
		ActiveIntervals.Reserve(SourceEdges.Num());
		for (const FSidecarBoundaryEdgeKey& SourceEdge : SourceEdges)
		{
			if (!SourceEdge.IsValid())
			{
				continue;
			}
			FSidecarActiveEdgeIntervals EdgeIntervals;
			EdgeIntervals.Key = SourceEdge;
			EdgeIntervals.Intervals.Add({ 0.0, 1.0 });
			ActiveIntervals.Add(EdgeIntervals);
		}
		CanonicalizeActiveEdgeIntervals(ActiveIntervals, ToleranceT);
		return ActiveIntervals;
	}

	TArray<FSidecarActiveEdgeIntervals> BuildFullActiveIntervalsFromBirthEdges(
		const TArray<FSidecarOceanCrustBirthEdge>& BirthEdges,
		const double ToleranceT)
	{
		return BuildFullActiveIntervalsFromSourceEdges(BuildSourceEdgesFromBirthEdges(BirthEdges), ToleranceT);
	}

	bool SubtractIntervalsForTouchedEdges(
		TArray<FSidecarActiveEdgeIntervals>& InOutActiveIntervals,
		const TArray<FSidecarActiveEdgeIntervals>& ConsumedIntervals,
		const double ToleranceT,
		TArray<FSidecarActiveEdgeIntervals>& OutTouchedPostIntervals)
	{
		OutTouchedPostIntervals.Reset();
		bool bTouchedAny = false;
		for (const FSidecarActiveEdgeIntervals& ConsumedEdge : ConsumedIntervals)
		{
			for (FSidecarActiveEdgeIntervals& ActiveEdge : InOutActiveIntervals)
			{
				if (!(ActiveEdge.Key == ConsumedEdge.Key))
				{
					continue;
				}
				bTouchedAny = true;
				TArray<FSidecarActiveInterval> Remainders = ActiveEdge.Intervals;
				for (const FSidecarActiveInterval& Consumed : ConsumedEdge.Intervals)
				{
					TArray<FSidecarActiveInterval> NextRemainders;
					for (const FSidecarActiveInterval& Active : Remainders)
					{
						if (Consumed.EndT <= Active.StartT + ToleranceT ||
							Consumed.StartT >= Active.EndT - ToleranceT)
						{
							NextRemainders.Add(Active);
							continue;
						}
						if (Consumed.StartT > Active.StartT + ToleranceT)
						{
							NextRemainders.Add({ Active.StartT, FMath::Min(Consumed.StartT, Active.EndT) });
						}
						if (Consumed.EndT < Active.EndT - ToleranceT)
						{
							NextRemainders.Add({ FMath::Max(Consumed.EndT, Active.StartT), Active.EndT });
						}
					}
					Remainders = MoveTemp(NextRemainders);
					CanonicalizeIntervalArray(Remainders, ToleranceT);
				}
				ActiveEdge.Intervals = MoveTemp(Remainders);
				FSidecarActiveEdgeIntervals TouchedPost;
				TouchedPost.Key = ActiveEdge.Key;
				TouchedPost.Intervals = ActiveEdge.Intervals;
				OutTouchedPostIntervals.Add(TouchedPost);
				break;
			}
		}
		CanonicalizeActiveEdgeIntervals(InOutActiveIntervals, ToleranceT);
		CanonicalizeActiveEdgeIntervals(OutTouchedPostIntervals, ToleranceT);
		return bTouchedAny;
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

bool FSidecarActiveInterval::IsValid(const double ToleranceT) const
{
	return FMath::IsFinite(StartT) &&
		FMath::IsFinite(EndT) &&
		StartT >= 0.0 &&
		EndT <= 1.0 &&
		EndT - StartT > ToleranceT;
}

uint32 FSidecarActiveInterval::ComputeHash() const
{
	uint32 Hash = 2166136261u;
	MixCrustHashDouble(Hash, StartT);
	MixCrustHashDouble(Hash, EndT);
	return Hash;
}

uint32 FSidecarActiveEdgeIntervals::ComputeHash() const
{
	uint32 Hash = 2166136261u;
	Hash = MixCrustHashValue(Hash, Key.ComputeHash());
	MixCrustHashInt32(Hash, Intervals.Num());
	for (const FSidecarActiveInterval& Interval : Intervals)
	{
		Hash = MixCrustHashValue(Hash, Interval.ComputeHash());
	}
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
	MixCrustHashDouble(Hash, ActiveAreaKm2);
	MixCrustHashDouble(Hash, ConsumedAreaKm2);
	MixCrustHashDouble(Hash, ConsumedFraction);
	MixCrustHashInt32(Hash, LastConsumedStep);
	MixCrustHashInt32(Hash, FullyConsumedStep);
	Hash = MixCrustHashValue(Hash, bIsFullyConsumed ? 1u : 0u);
	TArray<FSidecarActiveEdgeIntervals> CanonicalActiveIntervals = ActiveIntervalsBySourceEdge;
	CanonicalizeActiveEdgeIntervals(CanonicalActiveIntervals, 1.0e-9);
	MixCrustHashInt32(Hash, CanonicalActiveIntervals.Num());
	for (const FSidecarActiveEdgeIntervals& EdgeIntervals : CanonicalActiveIntervals)
	{
		Hash = MixCrustHashValue(Hash, EdgeIntervals.ComputeHash());
	}
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
	TArray<FSidecarBoundaryEdgeKey> CanonicalSourceEdges = SourceEdges;
	SortUniqueBoundaryEdgeKeys(CanonicalSourceEdges);
	MixCrustHashInt32(Hash, CanonicalSourceEdges.Num());
	for (const FSidecarBoundaryEdgeKey& Edge : CanonicalSourceEdges)
	{
		Hash = MixCrustHashValue(Hash, Edge.ComputeHash());
	}
	MixCrustHashDouble(Hash, CreatedAreaKm2);
	MixCrustHashDouble(Hash, NormalSeparationKmPerMy);
	MixCrustHashInt32(Hash, DebugSampleIds.Num());
	for (const int32 SampleId : DebugSampleIds)
	{
		MixCrustHashInt32(Hash, SampleId);
	}
	MixCrustHashInt32(Hash, SubductingPlateId);
	MixCrustHashInt32(Hash, OverridingPlateId);
	MixCrustHashDouble(Hash, PreActiveAreaKm2);
	MixCrustHashDouble(Hash, PostActiveAreaKm2);
	MixCrustHashDouble(Hash, PreConsumedAreaKm2);
	MixCrustHashDouble(Hash, PostConsumedAreaKm2);
	MixCrustHashDouble(Hash, ConsumedAreaKm2);
	MixCrustHashDouble(Hash, ConsumedFraction);
	TArray<FSidecarActiveEdgeIntervals> CanonicalConsumedIntervals = ConsumedIntervalDeltasBySourceEdge;
	CanonicalizeActiveEdgeIntervals(CanonicalConsumedIntervals, 1.0e-9);
	MixCrustHashInt32(Hash, CanonicalConsumedIntervals.Num());
	for (const FSidecarActiveEdgeIntervals& EdgeIntervals : CanonicalConsumedIntervals)
	{
		Hash = MixCrustHashValue(Hash, EdgeIntervals.ComputeHash());
	}
	TArray<FSidecarActiveEdgeIntervals> CanonicalPostIntervals = PostConsumptionActiveIntervalsBySourceEdge;
	CanonicalizeActiveEdgeIntervals(CanonicalPostIntervals, 1.0e-9);
	MixCrustHashInt32(Hash, CanonicalPostIntervals.Num());
	for (const FSidecarActiveEdgeIntervals& EdgeIntervals : CanonicalPostIntervals)
	{
		Hash = MixCrustHashValue(Hash, EdgeIntervals.ComputeHash());
	}
	MixCrustHashDouble(Hash, ConvergenceSpeedKmPerMy);
	MixCrustHashDouble(Hash, EdgeLengthKm);
	MixCrustHashDouble(Hash, CrustAgeAtConsumptionMy);
	MixCrustHashDouble(Hash, ProjectedElevationAtConsumptionKm);
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
		TargetRecord->ActiveAreaKm2 = FMath::Max(0.0, TargetRecord->ActiveAreaKm2 + Input.CreatedAreaKm2);
		TargetRecord->ConsumedFraction = TargetRecord->CreatedAreaKm2 > 0.0
			? FMath::Clamp(TargetRecord->ConsumedAreaKm2 / TargetRecord->CreatedAreaKm2, 0.0, 1.0)
			: 0.0;
		TargetRecord->ActiveIntervalsBySourceEdge.Append(BuildFullActiveIntervalsFromBirthEdges(BirthEdges, 1.0e-9));
		CanonicalizeActiveEdgeIntervals(TargetRecord->ActiveIntervalsBySourceEdge, 1.0e-9);
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
		NewRecord.ActiveAreaKm2 = FMath::Max(0.0, Input.CreatedAreaKm2);
		NewRecord.ConsumedAreaKm2 = 0.0;
		NewRecord.ConsumedFraction = 0.0;
		NewRecord.ActiveIntervalsBySourceEdge = BuildFullActiveIntervalsFromBirthEdges(BirthEdges, 1.0e-9);
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

void CanonicalizeActiveEdgeIntervals(
	TArray<FSidecarActiveEdgeIntervals>& InOutIntervalsBySourceEdge,
	const double ActiveIntervalToleranceT)
{
	for (FSidecarActiveEdgeIntervals& EdgeIntervals : InOutIntervalsBySourceEdge)
	{
		EdgeIntervals.Key = FSidecarBoundaryEdgeKey::Make(
			EdgeIntervals.Key.SampleA,
			EdgeIntervals.Key.SampleB);
		CanonicalizeIntervalArray(EdgeIntervals.Intervals, ActiveIntervalToleranceT);
	}

	Algo::Sort(InOutIntervalsBySourceEdge, [](const FSidecarActiveEdgeIntervals& A, const FSidecarActiveEdgeIntervals& B)
	{
		return A.Key < B.Key;
	});

	TArray<FSidecarActiveEdgeIntervals> Canonical;
	for (const FSidecarActiveEdgeIntervals& EdgeIntervals : InOutIntervalsBySourceEdge)
	{
		if (!EdgeIntervals.Key.IsValid() || EdgeIntervals.Intervals.IsEmpty())
		{
			continue;
		}
		if (!Canonical.IsEmpty() && Canonical.Last().Key == EdgeIntervals.Key)
		{
			Canonical.Last().Intervals.Append(EdgeIntervals.Intervals);
			CanonicalizeIntervalArray(Canonical.Last().Intervals, ActiveIntervalToleranceT);
		}
		else
		{
			Canonical.Add(EdgeIntervals);
		}
	}
	InOutIntervalsBySourceEdge = MoveTemp(Canonical);
}

bool ApplySubductionConsumptionEvent(
	FSidecarOceanCrustStore& InOutStore,
	FSidecarCrustEventLog& InOutEventLog,
	const FSidecarSubductionConsumptionEventInput& Input,
	const int32 CurrentStep,
	const double CurrentTimeMy,
	const double ActiveIntervalToleranceT,
	const double AreaToleranceKm2,
	int32* OutEventId)
{
	if (OutEventId != nullptr)
	{
		*OutEventId = INDEX_NONE;
	}

	if (Input.CrustId == INDEX_NONE ||
		Input.PlateA == INDEX_NONE ||
		Input.PlateB == INDEX_NONE ||
		Input.PlateA == Input.PlateB ||
		Input.ConsumedAreaKm2 < 0.0 ||
		!FMath::IsFinite(Input.ConsumedAreaKm2) ||
		Input.ConsumedIntervalsBySourceEdge.IsEmpty())
	{
		return false;
	}

	FSidecarOceanCrustRecord* TargetRecord = nullptr;
	for (FSidecarOceanCrustRecord& Record : InOutStore.Records)
	{
		if (Record.CrustId == Input.CrustId)
		{
			TargetRecord = &Record;
			break;
		}
	}
	if (TargetRecord == nullptr || TargetRecord->CreatedAreaKm2 <= 0.0)
	{
		return false;
	}

	const int32 LowPlateId = FMath::Min(Input.PlateA, Input.PlateB);
	const int32 HighPlateId = FMath::Max(Input.PlateA, Input.PlateB);
	TArray<FSidecarBoundaryEdgeKey> SourceEdges = Input.SourceEdges;
	SortUniqueBoundaryEdgeKeys(SourceEdges);
	TArray<FSidecarActiveEdgeIntervals> ConsumedIntervals = Input.ConsumedIntervalsBySourceEdge;
	CanonicalizeActiveEdgeIntervals(ConsumedIntervals, ActiveIntervalToleranceT);
	if (ConsumedIntervals.IsEmpty())
	{
		return false;
	}
	if (SourceEdges.IsEmpty())
	{
		for (const FSidecarActiveEdgeIntervals& EdgeIntervals : ConsumedIntervals)
		{
			SourceEdges.Add(EdgeIntervals.Key);
		}
		SortUniqueBoundaryEdgeKeys(SourceEdges);
	}

	if (TargetRecord->ActiveIntervalsBySourceEdge.IsEmpty())
	{
		TargetRecord->ActiveIntervalsBySourceEdge =
			BuildFullActiveIntervalsFromSourceEdges(TargetRecord->LastAcceptedSourceEdges, ActiveIntervalToleranceT);
	}
	CanonicalizeActiveEdgeIntervals(TargetRecord->ActiveIntervalsBySourceEdge, ActiveIntervalToleranceT);

	const double PreActiveAreaKm2 = TargetRecord->ActiveAreaKm2 > 0.0
		? TargetRecord->ActiveAreaKm2
		: FMath::Max(0.0, TargetRecord->CreatedAreaKm2 - TargetRecord->ConsumedAreaKm2);
	const double PreConsumedAreaKm2 = TargetRecord->ConsumedAreaKm2;
	const double AppliedConsumedAreaKm2 = FMath::Min(Input.ConsumedAreaKm2, PreActiveAreaKm2);

	TArray<FSidecarActiveEdgeIntervals> PostTouchedIntervals;
	if (!SubtractIntervalsForTouchedEdges(
			TargetRecord->ActiveIntervalsBySourceEdge,
			ConsumedIntervals,
			ActiveIntervalToleranceT,
			PostTouchedIntervals))
	{
		return false;
	}

	TargetRecord->ConsumedAreaKm2 = FMath::Clamp(
		PreConsumedAreaKm2 + AppliedConsumedAreaKm2,
		0.0,
		TargetRecord->CreatedAreaKm2);
	TargetRecord->ActiveAreaKm2 = FMath::Max(0.0, TargetRecord->CreatedAreaKm2 - TargetRecord->ConsumedAreaKm2);
	if (TargetRecord->ActiveAreaKm2 <= AreaToleranceKm2)
	{
		TargetRecord->ActiveAreaKm2 = 0.0;
		if (!TargetRecord->bIsFullyConsumed)
		{
			TargetRecord->FullyConsumedStep = CurrentStep;
		}
		TargetRecord->bIsFullyConsumed = true;
	}
	TargetRecord->ConsumedFraction = TargetRecord->CreatedAreaKm2 > 0.0
		? FMath::Clamp(TargetRecord->ConsumedAreaKm2 / TargetRecord->CreatedAreaKm2, 0.0, 1.0)
		: 0.0;
	TargetRecord->LastConsumedStep = CurrentStep;
	TargetRecord->LastUpdatedStep = CurrentStep;

	TArray<int32> DebugSampleIds = Input.DebugSampleIds;
	Algo::Sort(DebugSampleIds);

	const int32 EventId = InOutEventLog.NextEventId++;
	FSidecarCrustEventRecord EventRecord;
	EventRecord.EventId = EventId;
	EventRecord.EventType = ETectonicSidecarCrustEventType::SubductionConsumption;
	EventRecord.CrustId = TargetRecord->CrustId;
	EventRecord.PlateA = LowPlateId;
	EventRecord.PlateB = HighPlateId;
	EventRecord.ComponentId = TargetRecord->ComponentId;
	EventRecord.Step = CurrentStep;
	EventRecord.TimeMy = CurrentTimeMy;
	EventRecord.RidgeGenerationId = TargetRecord->RidgeGenerationId;
	EventRecord.ParentEventId = TargetRecord->ParentEventId;
	EventRecord.SourceEdges = SourceEdges;
	EventRecord.DebugSampleIds = DebugSampleIds;
	EventRecord.SubductingPlateId = Input.SubductingPlateId;
	EventRecord.OverridingPlateId = Input.OverridingPlateId;
	EventRecord.PreActiveAreaKm2 = PreActiveAreaKm2;
	EventRecord.PostActiveAreaKm2 = TargetRecord->ActiveAreaKm2;
	EventRecord.PreConsumedAreaKm2 = PreConsumedAreaKm2;
	EventRecord.PostConsumedAreaKm2 = TargetRecord->ConsumedAreaKm2;
	EventRecord.ConsumedAreaKm2 = AppliedConsumedAreaKm2;
	EventRecord.ConsumedFraction = TargetRecord->ConsumedFraction;
	EventRecord.ConsumedIntervalDeltasBySourceEdge = ConsumedIntervals;
	EventRecord.PostConsumptionActiveIntervalsBySourceEdge = PostTouchedIntervals;
	EventRecord.ConvergenceSpeedKmPerMy = Input.ConvergenceSpeedKmPerMy;
	EventRecord.EdgeLengthKm = Input.EdgeLengthKm;
	EventRecord.CrustAgeAtConsumptionMy = Input.CrustAgeAtConsumptionMy;
	EventRecord.ProjectedElevationAtConsumptionKm = Input.ProjectedElevationAtConsumptionKm;
	InOutEventLog.Events.Add(EventRecord);

	if (OutEventId != nullptr)
	{
		*OutEventId = EventId;
	}
	return true;
}

bool ReplaySubductionConsumptionEvents(
	const FSidecarOceanCrustStore& BaselineStore,
	const FSidecarCrustEventLog& EventLog,
	FSidecarOceanCrustStore& OutReplayedStore,
	const double ActiveIntervalToleranceT,
	const double AreaToleranceKm2)
{
	OutReplayedStore = BaselineStore;
	FSidecarCrustEventLog ReplayLog;
	for (const FSidecarCrustEventRecord& Event : EventLog.Events)
	{
		if (Event.EventType != ETectonicSidecarCrustEventType::SubductionConsumption)
		{
			continue;
		}

		FSidecarSubductionConsumptionEventInput Input;
		Input.CrustId = Event.CrustId;
		Input.PlateA = Event.PlateA;
		Input.PlateB = Event.PlateB;
		Input.SubductingPlateId = Event.SubductingPlateId;
		Input.OverridingPlateId = Event.OverridingPlateId;
		Input.SourceEdges = Event.SourceEdges;
		Input.ConsumedIntervalsBySourceEdge = Event.ConsumedIntervalDeltasBySourceEdge;
		Input.DebugSampleIds = Event.DebugSampleIds;
		Input.ConsumedAreaKm2 = Event.ConsumedAreaKm2;
		Input.ConvergenceSpeedKmPerMy = Event.ConvergenceSpeedKmPerMy;
		Input.EdgeLengthKm = Event.EdgeLengthKm;
		Input.CrustAgeAtConsumptionMy = Event.CrustAgeAtConsumptionMy;
		Input.ProjectedElevationAtConsumptionKm = Event.ProjectedElevationAtConsumptionKm;
		if (!ApplySubductionConsumptionEvent(
				OutReplayedStore,
				ReplayLog,
				Input,
				Event.Step,
				Event.TimeMy,
				ActiveIntervalToleranceT,
				AreaToleranceKm2,
				nullptr))
		{
			return false;
		}
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
