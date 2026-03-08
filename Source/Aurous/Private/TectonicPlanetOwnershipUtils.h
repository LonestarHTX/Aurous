#pragma once

#include "CoreMinimal.h"
#include "TectonicPlanet.h"

namespace TectonicPlanetOwnership
{
	inline int32 ResolveWeightedPlateOwnerFromPlateIds(const int32 PlateIds[3], const FVector3d& Barycentric)
	{
		struct FPlateWeight
		{
			int32 PlateId = INDEX_NONE;
			double Weight = 0.0;
		};

		FPlateWeight PlateWeights[3];
		auto AccumulateWeight = [&PlateWeights](const int32 PlateId, const double Weight)
		{
			if (PlateId == INDEX_NONE)
			{
				return;
			}

			for (FPlateWeight& PlateWeight : PlateWeights)
			{
				if (PlateWeight.PlateId == PlateId)
				{
					PlateWeight.Weight += Weight;
					return;
				}
				if (PlateWeight.PlateId == INDEX_NONE)
				{
					PlateWeight.PlateId = PlateId;
					PlateWeight.Weight = Weight;
					return;
				}
			}
		};

		AccumulateWeight(PlateIds[0], Barycentric.X);
		AccumulateWeight(PlateIds[1], Barycentric.Y);
		AccumulateWeight(PlateIds[2], Barycentric.Z);

		int32 BestPlateId = INDEX_NONE;
		double BestWeight = -1.0;
		for (const FPlateWeight& PlateWeight : PlateWeights)
		{
			if (PlateWeight.PlateId == INDEX_NONE)
			{
				continue;
			}

			if (PlateWeight.Weight > BestWeight + UE_DOUBLE_SMALL_NUMBER ||
				(FMath::IsNearlyEqual(PlateWeight.Weight, BestWeight, UE_DOUBLE_SMALL_NUMBER) &&
					(BestPlateId == INDEX_NONE || PlateWeight.PlateId < BestPlateId)))
			{
				BestPlateId = PlateWeight.PlateId;
				BestWeight = PlateWeight.Weight;
			}
		}

		return BestPlateId;
	}

	inline int32 ResolveWeightedPlateOwner(const FCanonicalSample* const CornerSamples[3], const FVector3d& Barycentric)
	{
		const int32 PlateIds[3] = {
			CornerSamples[0] ? CornerSamples[0]->PlateId : INDEX_NONE,
			CornerSamples[1] ? CornerSamples[1]->PlateId : INDEX_NONE,
			CornerSamples[2] ? CornerSamples[2]->PlateId : INDEX_NONE
		};
		return ResolveWeightedPlateOwnerFromPlateIds(PlateIds, Barycentric);
	}

	inline int32 ResolveWeightedPlateOwner(const TArray<FCanonicalSample>& Samples, const FDelaunayTriangle& Triangle, const FVector3d& Barycentric)
	{
		const int32 PlateIds[3] = {
			Samples.IsValidIndex(Triangle.V[0]) ? Samples[Triangle.V[0]].PlateId : INDEX_NONE,
			Samples.IsValidIndex(Triangle.V[1]) ? Samples[Triangle.V[1]].PlateId : INDEX_NONE,
			Samples.IsValidIndex(Triangle.V[2]) ? Samples[Triangle.V[2]].PlateId : INDEX_NONE
		};
		return ResolveWeightedPlateOwnerFromPlateIds(PlateIds, Barycentric);
	}

	inline bool IsPreferredWeightedPlateOwner(const TArray<FCanonicalSample>& Samples, const FDelaunayTriangle& Triangle, const int32 CandidatePlateId, const FVector3d& Barycentric)
	{
		return CandidatePlateId == ResolveWeightedPlateOwner(Samples, Triangle, Barycentric);
	}

	inline void BuildBoundaryMaskFromResolvedPlateIds(const TArray<int32>& ResolvedPlateIds, const int32 Width, const int32 Height, TArray<uint8>& OutMask)
	{
		OutMask.SetNumZeroed(Width * Height);
		if (ResolvedPlateIds.Num() != Width * Height)
		{
			return;
		}

		for (int32 Y = 0; Y < Height; ++Y)
		{
			for (int32 X = 0; X < Width; ++X)
			{
				const int32 PixelIndex = Y * Width + X;
				const int32 PlateId = ResolvedPlateIds[PixelIndex];
				if (PlateId == INDEX_NONE)
				{
					continue;
				}

				bool bBoundary = false;
				const int32 NeighborXs[4] = { X - 1, X + 1, X, X };
				const int32 NeighborYs[4] = { Y, Y, Y - 1, Y + 1 };
				for (int32 NeighborIdx = 0; NeighborIdx < 4; ++NeighborIdx)
				{
					const int32 NeighborX = NeighborXs[NeighborIdx];
					const int32 NeighborY = NeighborYs[NeighborIdx];
					if (NeighborX < 0 || NeighborX >= Width || NeighborY < 0 || NeighborY >= Height)
					{
						continue;
					}

					const int32 NeighborPlateId = ResolvedPlateIds[NeighborY * Width + NeighborX];
					if (NeighborPlateId != INDEX_NONE && NeighborPlateId != PlateId)
					{
						bBoundary = true;
						break;
					}
				}

				OutMask[PixelIndex] = bBoundary ? 255 : 0;
			}
		}
	}
}
