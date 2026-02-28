#pragma once

#include "CoreMinimal.h"
#include "Spatial/MeshAABBTree3.h"

struct FPlateTriangleSoupData
{
	int32 PlateId = INDEX_NONE;
	TArray<int32> GlobalTriangleIndices;
	TArray<UE::Geometry::FIndex3i> LocalTriangles;
	TArray<int32> LocalToCanonicalVertex;
	TMap<int32, int32> CanonicalToLocalVertex;
	TArray<FVector3d> RotatedVertices;
	uint64 ChangeStamp = 1;
};

struct FPlateTriangleSoupAdapter
{
	const FPlateTriangleSoupData* SoupData = nullptr;

	explicit FPlateTriangleSoupAdapter(const FPlateTriangleSoupData* InSoupData = nullptr)
		: SoupData(InSoupData)
	{
	}

	FORCEINLINE_DEBUGGABLE uint64 GetChangeStamp() const
	{
		return SoupData ? SoupData->ChangeStamp : 0;
	}

	FORCEINLINE_DEBUGGABLE int32 VertexCount() const
	{
		return SoupData ? SoupData->RotatedVertices.Num() : 0;
	}

	FORCEINLINE_DEBUGGABLE int32 TriangleCount() const
	{
		return SoupData ? SoupData->LocalTriangles.Num() : 0;
	}

	FORCEINLINE_DEBUGGABLE int32 MaxTriangleID() const
	{
		return TriangleCount();
	}

	FORCEINLINE_DEBUGGABLE UE::Geometry::FIndex3i GetTriangle(int32 TriangleID) const
	{
		check(SoupData);
		check(SoupData->LocalTriangles.IsValidIndex(TriangleID));
		return SoupData->LocalTriangles[TriangleID];
	}

	FORCEINLINE_DEBUGGABLE FVector3d GetVertex(int32 VertexID) const
	{
		check(SoupData);
		check(SoupData->RotatedVertices.IsValidIndex(VertexID));
		return SoupData->RotatedVertices[VertexID];
	}

	template<typename VecType>
	FORCEINLINE_DEBUGGABLE void GetTriVertices(int32 TriangleID, VecType& V0, VecType& V1, VecType& V2) const
	{
		check(SoupData);
		check(SoupData->LocalTriangles.IsValidIndex(TriangleID));
		const UE::Geometry::FIndex3i Tri = SoupData->LocalTriangles[TriangleID];
		V0 = static_cast<VecType>(SoupData->RotatedVertices[Tri.A]);
		V1 = static_cast<VecType>(SoupData->RotatedVertices[Tri.B]);
		V2 = static_cast<VecType>(SoupData->RotatedVertices[Tri.C]);
	}

	FORCEINLINE_DEBUGGABLE bool IsTriangle(int32 TriangleID) const
	{
		return SoupData && SoupData->LocalTriangles.IsValidIndex(TriangleID);
	}

	FORCEINLINE_DEBUGGABLE FVector3d GetTriNormal(int32 TriangleID) const
	{
		FVector3d V0;
		FVector3d V1;
		FVector3d V2;
		GetTriVertices(TriangleID, V0, V1, V2);
		return UE::Geometry::VectorUtil::Normal(V0, V1, V2);
	}
};

using FPlateTriangleSoupBVH = UE::Geometry::TMeshAABBTree3<FPlateTriangleSoupAdapter>;
