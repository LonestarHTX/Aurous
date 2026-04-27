#include "TectonicPlanetSidecar.h"

#include "Algo/Sort.h"
#include "Distance/DistPoint3Triangle3.h"
#include "Math/RandomStream.h"

namespace
{
	constexpr double SidecarSmallNumber = 1.0e-9;
	constexpr int32 SidecarTinyComponentMaxSize = 4;
	constexpr int32 SidecarLargePlateMinProjectedSamples = 128;
	constexpr double SidecarBoundingCapMargin = 1.0e-6;
	constexpr double SidecarTriangleEpsilon = 1.0e-12;
	constexpr double SidecarOceanicThicknessKm = 7.0;
	constexpr double SidecarRidgeElevationKm = -1.0;
	constexpr double SidecarAbyssalPlainElevationKm = -6.0;
	constexpr double SidecarDirectionDegeneracyThreshold = 1.0e-8;

	struct FSidecarProjectedMaterialRef
	{
		int32 PlateId = INDEX_NONE;
		int32 PlateIndex = INDEX_NONE;
		int32 MaterialIndex = INDEX_NONE;
		FVector3d WorldPosition = FVector3d::ZeroVector;
	};

	struct FSidecarProjectedMaterialGrid
	{
		int32 LongitudeBins = 128;
		int32 LatitudeBins = 64;
		TArray<FSidecarProjectedMaterialRef> Entries;
		TArray<TArray<int32>> Bins;
	};

	struct FSidecarFootprintRuntime
	{
		const FTectonicSidecarPlate* Plate = nullptr;
		FPlateTriangleSoupData SoupData;
		FPlateTriangleSoupAdapter Adapter;
		FPlateTriangleSoupBVH BVH;
		FSphericalBoundingCap BoundingCap;
		TArray<int32> LocalToFootprintTriangleIndex;

		FSidecarFootprintRuntime()
			: Adapter(&SoupData)
		{
		}
	};

	struct FSidecarFootprintHit
	{
		const FSidecarFootprintRuntime* Runtime = nullptr;
		const FTectonicSidecarFootprintTriangle* Triangle = nullptr;
		int32 LocalTriangleIndex = INDEX_NONE;
		int32 FootprintTriangleIndex = INDEX_NONE;
		int32 PlateId = INDEX_NONE;
		int32 GlobalTriangleIndex = INDEX_NONE;
		FVector3d Barycentric = FVector3d::ZeroVector;
		double ContainmentScore = 0.0;
		double ContinentalWeight = 0.0;
		double Elevation = 0.0;
		double Thickness = 0.0;
		double Age = 0.0;
		double DistanceRad = 0.0;
	};

	struct FSidecarExplicitProjectionScratch
	{
		TArray<uint8> ExactHitFlags;
		TArray<uint8> RecoveryFlags;
		TArray<uint8> GapFlags;
		TArray<uint8> DivergentGapFlags;
		TArray<uint8> OverlapFlags;
		TArray<uint8> OceanFillFlags;
		TArray<uint8> FabricatedFlags;
		TArray<uint8> PlateBoundaryFlags;
		TArray<uint8> DivergentBoundaryFlags;
		double LocalFootprintContinentalMass = 0.0;
		double MultiHitProjectedContinentalMass = 0.0;
		double VisibleProjectedContinentalMass = 0.0;
	};

	struct FSidecarVoronoiMaterialScratch
	{
		TArray<uint8> ExactHitFlags;
		TArray<uint8> RecoveryFlags;
		TArray<uint8> OceanFillFlags;
		TArray<uint8> DivergentOceanFillFlags;
		TArray<uint8> FabricatedFlags;
		TArray<uint8> MaterialOwnerMismatchFlags;
		TArray<uint8> MaterialOverlapFlags;
		TArray<uint8> DivergentBoundaryFlags;
		double LocalFootprintContinentalMass = 0.0;
		double MultiHitProjectedContinentalMass = 0.0;
		double VisibleProjectedContinentalMass = 0.0;
	};

	double AngularDistanceRad(const FVector3d& A, const FVector3d& B)
	{
		const double Dot = FVector3d::DotProduct(A.GetSafeNormal(), B.GetSafeNormal());
		return FMath::Acos(FMath::Clamp(Dot, -1.0, 1.0));
	}

	FVector3d NormalizeOrFallback(const FVector3d& Value, const FVector3d& Fallback)
	{
		const FVector3d Normalized = Value.GetSafeNormal();
		return Normalized.IsNearlyZero() ? Fallback.GetSafeNormal() : Normalized;
	}

	FVector3d MakeTangentFallback(const FVector3d& Normal)
	{
		const FVector3d Axis = FMath::Abs(Normal.Z) < 0.9 ? FVector3d(0.0, 0.0, 1.0) : FVector3d(1.0, 0.0, 0.0);
		return FVector3d::CrossProduct(Axis, Normal).GetSafeNormal();
	}

	FVector3d ProjectOntoTangent(const FVector3d& Vector, const FVector3d& Normal)
	{
		const FVector3d UnitNormal = Normal.GetSafeNormal();
		return Vector - (Vector.Dot(UnitNormal) * UnitNormal);
	}

	FVector3d ComputeSidecarSurfaceVelocity(
		const FTectonicSidecarPlate& Plate,
		const FVector3d& QueryPoint,
		const double PlanetRadiusKm)
	{
		if (Plate.RotationAxis.IsNearlyZero() || Plate.AngularSpeedRadPerMy <= 0.0)
		{
			return FVector3d::ZeroVector;
		}

		return FVector3d::CrossProduct(Plate.RotationAxis.GetSafeNormal(), QueryPoint.GetSafeNormal()) *
			(Plate.AngularSpeedRadPerMy * PlanetRadiusKm);
	}

	bool ComputeBoundaryLowToHighDirection(
		const FTectonicSidecarPlate& LowPlate,
		const FTectonicSidecarPlate& HighPlate,
		const FSidecarOwnerEdge& OwnerEdge,
		FVector3d& OutDirection)
	{
		const FVector3d QueryPoint = NormalizeOrFallback(OwnerEdge.Midpoint, OwnerEdge.SampleAPosition);
		const FVector3d LowCenter =
			LowPlate.WorldFromLocal.RotateVector(LowPlate.InitialCenter).GetSafeNormal();
		const FVector3d HighCenter =
			HighPlate.WorldFromLocal.RotateVector(HighPlate.InitialCenter).GetSafeNormal();
		const FVector3d CenterDirection = ProjectOntoTangent(HighCenter - LowCenter, QueryPoint);
		OutDirection = CenterDirection.GetSafeNormal();
		if (OutDirection.IsNearlyZero())
		{
			const FVector3d EdgeTangent = ProjectOntoTangent(
				OwnerEdge.SampleBPosition - OwnerEdge.SampleAPosition,
				QueryPoint).GetSafeNormal();
			OutDirection = FVector3d::CrossProduct(QueryPoint, EdgeTangent).GetSafeNormal();
			if (OutDirection.Dot(CenterDirection) < 0.0)
			{
				OutDirection *= -1.0;
			}
		}
		return !OutDirection.IsNearlyZero();
	}

	bool ComputePlateCenterSeparationKmPerMy(
		const FTectonicSidecarPlate& LowPlate,
		const FTectonicSidecarPlate& HighPlate,
		const double PlanetRadiusKm,
		double& OutSeparationKmPerMy)
	{
		OutSeparationKmPerMy = 0.0;
		const FVector3d LowCenter =
			LowPlate.WorldFromLocal.RotateVector(LowPlate.InitialCenter).GetSafeNormal();
		const FVector3d HighCenter =
			HighPlate.WorldFromLocal.RotateVector(HighPlate.InitialCenter).GetSafeNormal();
		const FVector3d QueryPoint = NormalizeOrFallback(LowCenter + HighCenter, LowCenter);
		const FVector3d LowToHigh = ProjectOntoTangent(HighCenter - LowCenter, QueryPoint).GetSafeNormal();
		if (LowToHigh.IsNearlyZero())
		{
			return false;
		}

		const FVector3d LowVelocity = ComputeSidecarSurfaceVelocity(LowPlate, QueryPoint, PlanetRadiusKm);
		const FVector3d HighVelocity = ComputeSidecarSurfaceVelocity(HighPlate, QueryPoint, PlanetRadiusKm);
		OutSeparationKmPerMy = (HighVelocity - LowVelocity).Dot(LowToHigh);
		return true;
	}

	FVector3d ComputePlanarBarycentric(
		const FVector3d& A,
		const FVector3d& B,
		const FVector3d& C,
		const FVector3d& P)
	{
		const FVector3d Normal = FVector3d::CrossProduct(B - A, C - A).GetSafeNormal();
		const double PlaneOffset = Normal.Dot(A);
		const double RayDot = Normal.Dot(P);

		FVector3d ProjectedPoint = P;
		if (FMath::Abs(RayDot) > UE_DOUBLE_SMALL_NUMBER)
		{
			ProjectedPoint = P * (PlaneOffset / RayDot);
		}
		else
		{
			ProjectedPoint = P - ((P - A).Dot(Normal) * Normal);
		}

		const FVector3d V0 = B - A;
		const FVector3d V1 = C - A;
		const FVector3d V2 = ProjectedPoint - A;
		const double Dot00 = V0.Dot(V0);
		const double Dot01 = V0.Dot(V1);
		const double Dot02 = V0.Dot(V2);
		const double Dot11 = V1.Dot(V1);
		const double Dot12 = V1.Dot(V2);
		const double Denominator = Dot00 * Dot11 - Dot01 * Dot01;
		if (FMath::Abs(Denominator) <= 1.0e-20)
		{
			return FVector3d(-1.0, -1.0, -1.0);
		}

		const double InvDenominator = 1.0 / Denominator;
		const double V = (Dot11 * Dot02 - Dot01 * Dot12) * InvDenominator;
		const double W = (Dot00 * Dot12 - Dot01 * Dot02) * InvDenominator;
		const double U = 1.0 - V - W;
		return FVector3d(U, V, W);
	}

	FVector3d NormalizeBarycentric(const FVector3d& RawBarycentric)
	{
		FVector3d Clamped(
			FMath::Max(0.0, RawBarycentric.X),
			FMath::Max(0.0, RawBarycentric.Y),
			FMath::Max(0.0, RawBarycentric.Z));
		const double Sum = Clamped.X + Clamped.Y + Clamped.Z;
		if (Sum > UE_DOUBLE_SMALL_NUMBER)
		{
			Clamped /= Sum;
		}
		return Clamped;
	}

	double ComputeContainmentScore(const FVector3d& Barycentric)
	{
		return FMath::Clamp(FMath::Min3(Barycentric.X, Barycentric.Y, Barycentric.Z), 0.0, 1.0 / 3.0);
	}

	double SphericalTriangleAreaUnit(
		const FVector3d& A,
		const FVector3d& B,
		const FVector3d& C)
	{
		const FVector3d UnitA = A.GetSafeNormal();
		const FVector3d UnitB = B.GetSafeNormal();
		const FVector3d UnitC = C.GetSafeNormal();
		const double Triple = FVector3d::DotProduct(UnitA, FVector3d::CrossProduct(UnitB, UnitC));
		const double Denominator =
			1.0 +
			FVector3d::DotProduct(UnitA, UnitB) +
			FVector3d::DotProduct(UnitB, UnitC) +
			FVector3d::DotProduct(UnitC, UnitA);
		return FMath::Abs(2.0 * FMath::Atan2(Triple, Denominator));
	}

	FSphericalBoundingCap BuildSidecarBoundingCapFromVertices(const TArray<FVector3d>& Vertices)
	{
		FSphericalBoundingCap Cap;
		if (Vertices.IsEmpty())
		{
			return Cap;
		}

		FVector3d CenterSum = FVector3d::ZeroVector;
		for (const FVector3d& Vertex : Vertices)
		{
			CenterSum += Vertex.GetSafeNormal();
		}

		Cap.Center = CenterSum.GetSafeNormal();
		if (Cap.Center.IsNearlyZero())
		{
			Cap.Center = Vertices[0].GetSafeNormal();
		}

		double MinDot = 1.0;
		for (const FVector3d& Vertex : Vertices)
		{
			MinDot = FMath::Min(MinDot, Cap.Center.Dot(Vertex.GetSafeNormal()));
		}

		Cap.CosAngle = FMath::Clamp(MinDot - SidecarBoundingCapMargin, -1.0, 1.0);
		return Cap;
	}

	FVector3d ComputeWeightedCentroid(
		const TArray<FSample>& Samples,
		const int32 PlateId,
		const bool bContinentalOnly)
	{
		FVector3d Sum = FVector3d::ZeroVector;
		double WeightSum = 0.0;
		for (const FSample& Sample : Samples)
		{
			if (Sample.PlateId != PlateId)
			{
				continue;
			}

			const double Weight = bContinentalOnly
				? (Sample.ContinentalWeight >= 0.5f ? static_cast<double>(Sample.ContinentalWeight) : 0.0)
				: 1.0;
			if (Weight <= 0.0)
			{
				continue;
			}

			Sum += Sample.Position.GetSafeNormal() * Weight;
			WeightSum += Weight;
		}

		return WeightSum > 0.0 ? Sum.GetSafeNormal() : FVector3d::ZeroVector;
	}

	void UnitToLatLonBins(
		const FVector3d& Unit,
		const int32 LongitudeBins,
		const int32 LatitudeBins,
		int32& OutLongitudeBin,
		int32& OutLatitudeBin)
	{
		const FVector3d Normalized = Unit.GetSafeNormal();
		const double Longitude = FMath::Atan2(Normalized.Y, Normalized.X);
		const double Latitude = FMath::Asin(FMath::Clamp(Normalized.Z, -1.0, 1.0));
		const double U = (Longitude + PI) / (2.0 * PI);
		const double V = (Latitude + HALF_PI) / PI;

		OutLongitudeBin = FMath::Clamp(
			static_cast<int32>(FMath::FloorToDouble(U * static_cast<double>(LongitudeBins))),
			0,
			LongitudeBins - 1);
		OutLatitudeBin = FMath::Clamp(
			static_cast<int32>(FMath::FloorToDouble(V * static_cast<double>(LatitudeBins))),
			0,
			LatitudeBins - 1);
	}

	int32 MaterialGridIndex(
		const int32 LongitudeBin,
		const int32 LatitudeBin,
		const int32 LongitudeBins)
	{
		return LatitudeBin * LongitudeBins + LongitudeBin;
	}

	uint32 HashSidecarValue(uint32 Hash, const uint32 Value)
	{
		Hash ^= Value;
		Hash *= 16777619u;
		return Hash;
	}

	void HashSidecarInt32(uint32& Hash, const int32 Value)
	{
		Hash = HashSidecarValue(Hash, static_cast<uint32>(Value));
	}

	void HashSidecarFloat(uint32& Hash, const float Value)
	{
		uint32 Bits = 0;
		FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		Hash = HashSidecarValue(Hash, Bits);
	}

	void HashSidecarDouble(uint32& Hash, const double Value)
	{
		uint64 Bits = 0;
		FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		Hash = HashSidecarValue(Hash, static_cast<uint32>(Bits & 0xffffffffu));
		Hash = HashSidecarValue(Hash, static_cast<uint32>(Bits >> 32));
	}

	void HashSidecarVector(uint32& Hash, const FVector3d& Value)
	{
		HashSidecarDouble(Hash, Value.X);
		HashSidecarDouble(Hash, Value.Y);
		HashSidecarDouble(Hash, Value.Z);
	}

	void HashSidecarQuat(uint32& Hash, const FQuat4d& Value)
	{
		HashSidecarDouble(Hash, Value.X);
		HashSidecarDouble(Hash, Value.Y);
		HashSidecarDouble(Hash, Value.Z);
		HashSidecarDouble(Hash, Value.W);
	}

	int32 CountFlags(const TArray<uint8>& Flags)
	{
		int32 Count = 0;
		for (const uint8 Flag : Flags)
		{
			Count += Flag != 0 ? 1 : 0;
		}
		return Count;
	}

	void UpdateNearest3(
		const double DistanceSq,
		const int32 MaterialIndex,
		double (&BestDistanceSq)[3],
		int32 (&BestMaterialIndex)[3])
	{
		for (int32 Slot = 0; Slot < 3; ++Slot)
		{
			if (DistanceSq >= BestDistanceSq[Slot])
			{
				continue;
			}

			for (int32 Shift = 2; Shift > Slot; --Shift)
			{
				BestDistanceSq[Shift] = BestDistanceSq[Shift - 1];
				BestMaterialIndex[Shift] = BestMaterialIndex[Shift - 1];
			}

			BestDistanceSq[Slot] = DistanceSq;
			BestMaterialIndex[Slot] = MaterialIndex;
			break;
		}
	}

	bool IsCurrentWorldPositionAtInitialStep(const int32 CurrentStep, const FTectonicSidecarConfig& Config)
	{
		return (CurrentStep == 0 || Config.bForceZeroAngularSpeeds) &&
			!Config.bForceExplicitProjectionAtRestForTest;
	}

	double GetConfiguredRecoveryToleranceRad(const FTectonicSidecarConfig& Config)
	{
		if (Config.RecoveryToleranceRad >= 0.0)
		{
			return Config.RecoveryToleranceRad;
		}

		const double ApproxSpacingRad =
			FMath::Sqrt((4.0 * PI) / FMath::Max(1.0, static_cast<double>(Config.SampleCount)));
		return FMath::Min(0.003, 0.25 * ApproxSpacingRad);
	}

	bool IsPointInsideTriangleByBarycentric(
		const FVector3d& A,
		const FVector3d& B,
		const FVector3d& C,
		const FVector3d& QueryPoint,
		FVector3d& OutBarycentric)
	{
		const FVector3d RawBarycentric = ComputePlanarBarycentric(A, B, C, QueryPoint);
		if (RawBarycentric.X < -1.0e-6 ||
			RawBarycentric.Y < -1.0e-6 ||
			RawBarycentric.Z < -1.0e-6)
		{
			return false;
		}

		OutBarycentric = NormalizeBarycentric(RawBarycentric);
		return OutBarycentric.X + OutBarycentric.Y + OutBarycentric.Z > UE_DOUBLE_SMALL_NUMBER;
	}

	bool FindContainingTriangleInBVH(
		const FPlateTriangleSoupBVH& BVH,
		const FPlateTriangleSoupAdapter& Adapter,
		const FVector3d& QueryPoint,
		int32& OutLocalTriangleId,
		FVector3d& OutA,
		FVector3d& OutB,
		FVector3d& OutC,
		FVector3d& OutBarycentric)
	{
		OutLocalTriangleId = INDEX_NONE;
		if (BVH.RootIndex < 0)
		{
			return false;
		}

		TArray<int32, TInlineAllocator<64>> NodeStack;
		NodeStack.Add(BVH.RootIndex);
		while (!NodeStack.IsEmpty())
		{
			const int32 NodeIndex = NodeStack.Pop(EAllowShrinking::No);
			if (!BVH.box_contains(NodeIndex, QueryPoint))
			{
				continue;
			}

			const int32 ListStartIndex = BVH.BoxToIndex[NodeIndex];
			if (ListStartIndex < BVH.TrianglesEnd)
			{
				const int32 TriangleCount = BVH.IndexList[ListStartIndex];
				for (int32 TriangleListIndex = 1; TriangleListIndex <= TriangleCount; ++TriangleListIndex)
				{
					const int32 LocalTriangleId = BVH.IndexList[ListStartIndex + TriangleListIndex];
					if (!Adapter.IsTriangle(LocalTriangleId))
					{
						continue;
					}

					Adapter.GetTriVertices(LocalTriangleId, OutA, OutB, OutC);
					if (IsPointInsideTriangleByBarycentric(OutA, OutB, OutC, QueryPoint, OutBarycentric))
					{
						OutLocalTriangleId = LocalTriangleId;
						return true;
					}
				}
				continue;
			}

			const int32 ChildRecord = BVH.IndexList[ListStartIndex];
			if (ChildRecord < 0)
			{
				const int32 ChildNode = (-ChildRecord) - 1;
				if (ChildNode >= 0 && BVH.box_contains(ChildNode, QueryPoint))
				{
					NodeStack.Add(ChildNode);
				}
				continue;
			}

			const int32 ChildA = ChildRecord - 1;
			const int32 ChildB = BVH.IndexList[ListStartIndex + 1] - 1;
			const bool bContainsA = ChildA >= 0 && BVH.box_contains(ChildA, QueryPoint);
			const bool bContainsB = ChildB >= 0 && BVH.box_contains(ChildB, QueryPoint);
			if (!bContainsA && !bContainsB)
			{
				continue;
			}

			if (bContainsA && bContainsB)
			{
				const double DistanceA = BVH.BoxDistanceSqr(ChildA, QueryPoint);
				const double DistanceB = BVH.BoxDistanceSqr(ChildB, QueryPoint);
				if (DistanceA <= DistanceB)
				{
					NodeStack.Add(ChildB);
					NodeStack.Add(ChildA);
				}
				else
				{
					NodeStack.Add(ChildA);
					NodeStack.Add(ChildB);
				}
				continue;
			}

			NodeStack.Add(bContainsA ? ChildA : ChildB);
		}

		return false;
	}

	void InterpolateFootprintMaterial(
		const FTectonicSidecarFootprintTriangle& Triangle,
		const FVector3d& Barycentric,
		double& OutContinentalWeight,
		double& OutElevation,
		double& OutThickness,
		double& OutAge)
	{
		OutContinentalWeight =
			(Barycentric.X * Triangle.Vertices[0].ContinentalWeight) +
			(Barycentric.Y * Triangle.Vertices[1].ContinentalWeight) +
			(Barycentric.Z * Triangle.Vertices[2].ContinentalWeight);
		OutElevation =
			(Barycentric.X * Triangle.Vertices[0].Elevation) +
			(Barycentric.Y * Triangle.Vertices[1].Elevation) +
			(Barycentric.Z * Triangle.Vertices[2].Elevation);
		OutThickness =
			(Barycentric.X * Triangle.Vertices[0].Thickness) +
			(Barycentric.Y * Triangle.Vertices[1].Thickness) +
			(Barycentric.Z * Triangle.Vertices[2].Thickness);
		OutAge =
			(Barycentric.X * Triangle.Vertices[0].Age) +
			(Barycentric.Y * Triangle.Vertices[1].Age) +
			(Barycentric.Z * Triangle.Vertices[2].Age);
	}

	void ApplyFootprintHitToSample(const FSidecarFootprintHit& Hit, FSample& OutSample)
	{
		OutSample.PlateId = Hit.PlateId;
		OutSample.ContinentalWeight = static_cast<float>(Hit.ContinentalWeight);
		OutSample.Elevation = static_cast<float>(Hit.Elevation);
		OutSample.Thickness = static_cast<float>(Hit.Thickness);
		OutSample.Age = static_cast<float>(Hit.Age);
		OutSample.RidgeDirection = FVector3d::ZeroVector;
		OutSample.FoldDirection = FVector3d::ZeroVector;
		OutSample.OrogenyType = EOrogenyType::None;
		OutSample.TerraneId = INDEX_NONE;
		OutSample.SubductionDistanceKm = -1.0f;
	}

	void ApplyFootprintMaterialToSample(const FSidecarFootprintHit& Hit, FSample& OutSample)
	{
		OutSample.ContinentalWeight = static_cast<float>(Hit.ContinentalWeight);
		OutSample.Elevation = static_cast<float>(Hit.Elevation);
		OutSample.Thickness = static_cast<float>(Hit.Thickness);
		OutSample.Age = static_cast<float>(Hit.Age);
		OutSample.RidgeDirection = FVector3d::ZeroVector;
		OutSample.FoldDirection = FVector3d::ZeroVector;
		OutSample.OrogenyType = EOrogenyType::None;
		OutSample.TerraneId = INDEX_NONE;
		OutSample.SubductionDistanceKm = -1.0f;
	}

	bool IsBetterFootprintHit(const FSidecarFootprintHit& Candidate, const FSidecarFootprintHit& Current)
	{
		if (Current.PlateId == INDEX_NONE)
		{
			return true;
		}
		if (Candidate.ContainmentScore > Current.ContainmentScore + SidecarTriangleEpsilon)
		{
			return true;
		}
		if (Candidate.ContainmentScore + SidecarTriangleEpsilon < Current.ContainmentScore)
		{
			return false;
		}
		if (Candidate.PlateId != Current.PlateId)
		{
			return Candidate.PlateId < Current.PlateId;
		}
		return Candidate.GlobalTriangleIndex < Current.GlobalTriangleIndex;
	}

	bool IsBetterVisibleMaterialHit(const FSidecarFootprintHit& Candidate, const FSidecarFootprintHit& Current)
	{
		if (Current.PlateId == INDEX_NONE)
		{
			return true;
		}
		if (Candidate.ContinentalWeight > Current.ContinentalWeight + 1.0e-6)
		{
			return true;
		}
		if (Candidate.ContinentalWeight + 1.0e-6 < Current.ContinentalWeight)
		{
			return false;
		}
		return IsBetterFootprintHit(Candidate, Current);
	}


	bool TryMakeFootprintHit(
		const FSidecarFootprintRuntime& Runtime,
		const int32 LocalTriangleIndex,
		const FVector3d& Barycentric,
		const double DistanceRad,
		FSidecarFootprintHit& OutHit)
	{
		if (Runtime.Plate == nullptr ||
			!Runtime.LocalToFootprintTriangleIndex.IsValidIndex(LocalTriangleIndex))
		{
			return false;
		}

		const int32 FootprintTriangleIndex = Runtime.LocalToFootprintTriangleIndex[LocalTriangleIndex];
		if (!Runtime.Plate->FootprintTriangles.IsValidIndex(FootprintTriangleIndex))
		{
			return false;
		}

		const FTectonicSidecarFootprintTriangle& Triangle =
			Runtime.Plate->FootprintTriangles[FootprintTriangleIndex];
		OutHit = FSidecarFootprintHit{};
		OutHit.Runtime = &Runtime;
		OutHit.Triangle = &Triangle;
		OutHit.LocalTriangleIndex = LocalTriangleIndex;
		OutHit.FootprintTriangleIndex = FootprintTriangleIndex;
		OutHit.PlateId = Runtime.Plate->PlateId;
		OutHit.GlobalTriangleIndex = Triangle.GlobalTriangleIndex;
		OutHit.Barycentric = NormalizeBarycentric(Barycentric);
		OutHit.ContainmentScore = ComputeContainmentScore(OutHit.Barycentric);
		OutHit.DistanceRad = DistanceRad;
		InterpolateFootprintMaterial(
			Triangle,
			OutHit.Barycentric,
			OutHit.ContinentalWeight,
			OutHit.Elevation,
			OutHit.Thickness,
			OutHit.Age);
		return true;
	}

	bool TryFindNearestFootprintHit(
		const TArray<TUniquePtr<FSidecarFootprintRuntime>>& Runtimes,
		const FVector3d& QueryPoint,
		const double RecoveryToleranceRad,
		FSidecarFootprintHit& OutHit)
	{
		OutHit = FSidecarFootprintHit{};
		double BestDistanceSqr = TNumericLimits<double>::Max();
		const FSidecarFootprintRuntime* BestRuntime = nullptr;
		int32 BestLocalTriangleIndex = INDEX_NONE;

		for (const TUniquePtr<FSidecarFootprintRuntime>& RuntimePtr : Runtimes)
		{
			const FSidecarFootprintRuntime* Runtime = RuntimePtr.Get();
			if (Runtime == nullptr || Runtime->BVH.RootIndex < 0)
			{
				continue;
			}

			double CandidateDistanceSqr = TNumericLimits<double>::Max();
			const int32 CandidateLocalTriangleIndex =
				Runtime->BVH.FindNearestTriangle(QueryPoint, CandidateDistanceSqr);
			if (!Runtime->SoupData.LocalTriangles.IsValidIndex(CandidateLocalTriangleIndex))
			{
				continue;
			}

			const bool bCloser = CandidateDistanceSqr + SidecarTriangleEpsilon < BestDistanceSqr;
			const bool bTie = FMath::IsNearlyEqual(CandidateDistanceSqr, BestDistanceSqr, SidecarTriangleEpsilon);
			const int32 CandidatePlateId = Runtime->Plate != nullptr ? Runtime->Plate->PlateId : INDEX_NONE;
			const int32 BestPlateId = BestRuntime != nullptr && BestRuntime->Plate != nullptr ? BestRuntime->Plate->PlateId : INDEX_NONE;
			if (!bCloser && !(bTie && CandidatePlateId < BestPlateId))
			{
				continue;
			}

			BestDistanceSqr = CandidateDistanceSqr;
			BestRuntime = Runtime;
			BestLocalTriangleIndex = CandidateLocalTriangleIndex;
		}

		if (BestRuntime == nullptr ||
			!BestRuntime->SoupData.LocalTriangles.IsValidIndex(BestLocalTriangleIndex) ||
			FMath::Sqrt(FMath::Max(BestDistanceSqr, 0.0)) > RecoveryToleranceRad)
		{
			return false;
		}

		const UE::Geometry::FIndex3i LocalTriangle = BestRuntime->SoupData.LocalTriangles[BestLocalTriangleIndex];
		if (!BestRuntime->SoupData.RotatedVertices.IsValidIndex(LocalTriangle.A) ||
			!BestRuntime->SoupData.RotatedVertices.IsValidIndex(LocalTriangle.B) ||
			!BestRuntime->SoupData.RotatedVertices.IsValidIndex(LocalTriangle.C))
		{
			return false;
		}

		const FVector3d A = BestRuntime->SoupData.RotatedVertices[LocalTriangle.A];
		const FVector3d B = BestRuntime->SoupData.RotatedVertices[LocalTriangle.B];
		const FVector3d C = BestRuntime->SoupData.RotatedVertices[LocalTriangle.C];
		UE::Geometry::FDistPoint3Triangle3d DistanceQuery(
			QueryPoint,
			UE::Geometry::TTriangle3<double>(A, B, C));
		DistanceQuery.ComputeResult();
		const FVector3d Barycentric = NormalizeBarycentric(FVector3d(
			DistanceQuery.TriangleBaryCoords.X,
			DistanceQuery.TriangleBaryCoords.Y,
			DistanceQuery.TriangleBaryCoords.Z));
		return TryMakeFootprintHit(
			*BestRuntime,
			BestLocalTriangleIndex,
			Barycentric,
			FMath::Sqrt(FMath::Max(BestDistanceSqr, 0.0)),
			OutHit);
	}

	void BuildFootprintRuntimes(
		const TArray<FTectonicSidecarPlate>& Plates,
		TArray<TUniquePtr<FSidecarFootprintRuntime>>& OutRuntimes)
	{
		OutRuntimes.Reset();
		OutRuntimes.Reserve(Plates.Num());
		for (const FTectonicSidecarPlate& Plate : Plates)
		{
			TUniquePtr<FSidecarFootprintRuntime> Runtime = MakeUnique<FSidecarFootprintRuntime>();
			Runtime->Plate = &Plate;
			Runtime->SoupData.PlateId = Plate.PlateId;
			Runtime->SoupData.ChangeStamp = 1;
			Runtime->SoupData.GlobalTriangleIndices.Reserve(Plate.FootprintTriangles.Num());
			Runtime->SoupData.LocalTriangles.Reserve(Plate.FootprintTriangles.Num());
			Runtime->SoupData.RotatedVertices.Reserve(Plate.FootprintTriangles.Num() * 3);
			Runtime->LocalToFootprintTriangleIndex.Reserve(Plate.FootprintTriangles.Num());
			TArray<FVector3d> RotatedVerticesForCap;
			RotatedVerticesForCap.Reserve(Plate.FootprintTriangles.Num() * 3);

			for (int32 FootprintTriangleIndex = 0; FootprintTriangleIndex < Plate.FootprintTriangles.Num(); ++FootprintTriangleIndex)
			{
				const FTectonicSidecarFootprintTriangle& Triangle = Plate.FootprintTriangles[FootprintTriangleIndex];
				const int32 VertexBase = Runtime->SoupData.RotatedVertices.Num();
				for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
				{
					const FVector3d Rotated =
						Plate.WorldFromLocal.RotateVector(Triangle.Vertices[CornerIndex].LocalPosition).GetSafeNormal();
					Runtime->SoupData.RotatedVertices.Add(Rotated);
					RotatedVerticesForCap.Add(Rotated);
				}
				Runtime->SoupData.LocalTriangles.Add(UE::Geometry::FIndex3i(VertexBase, VertexBase + 1, VertexBase + 2));
				Runtime->SoupData.GlobalTriangleIndices.Add(Triangle.GlobalTriangleIndex);
				Runtime->LocalToFootprintTriangleIndex.Add(FootprintTriangleIndex);
			}

			Runtime->SoupData.ChangeStamp++;
			Runtime->Adapter = FPlateTriangleSoupAdapter(&Runtime->SoupData);
			Runtime->BVH.SetMesh(&Runtime->Adapter, true);
			Runtime->BoundingCap = BuildSidecarBoundingCapFromVertices(RotatedVerticesForCap);
			OutRuntimes.Add(MoveTemp(Runtime));
		}
	}

	int32 ChooseMajorityTrianglePlateId(const int32 PlateA, const int32 PlateB, const int32 PlateC)
	{
		const int32 PlateIds[3] = { PlateA, PlateB, PlateC };
		int32 BestPlateId = INDEX_NONE;
		int32 BestCount = 0;
		for (const int32 CandidatePlateId : PlateIds)
		{
			if (CandidatePlateId == INDEX_NONE)
			{
				continue;
			}

			int32 Count = 0;
			for (const int32 PlateId : PlateIds)
			{
				Count += PlateId == CandidatePlateId ? 1 : 0;
			}
			if (Count > BestCount || (Count == BestCount && (BestPlateId == INDEX_NONE || CandidatePlateId < BestPlateId)))
			{
				BestCount = Count;
				BestPlateId = CandidatePlateId;
			}
		}
		return BestPlateId;
	}

	struct FSidecarGapClassification
	{
		bool bDivergent = false;
		int32 PrimaryPlateId = INDEX_NONE;
		int32 SecondaryPlateId = INDEX_NONE;
		FVector3d RidgeDirection = FVector3d::ZeroVector;
	};

	FSidecarGapClassification ClassifyExplicitGap(
		const TArray<FTectonicSidecarPlate>& Plates,
		const FTectonicSidecarConfig& Config,
		const FVector3d& QueryPoint)
	{
		struct FCenterCandidate
		{
			int32 PlateIndex = INDEX_NONE;
			double DistanceRad = TNumericLimits<double>::Max();
		};

		TArray<FCenterCandidate, TInlineAllocator<4>> ClosestCenters;
		for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
		{
			const FTectonicSidecarPlate& Plate = Plates[PlateIndex];
			const FVector3d CurrentCenter =
				Plate.WorldFromLocal.RotateVector(Plate.InitialCenter).GetSafeNormal();
			const double DistanceRad = AngularDistanceRad(QueryPoint, CurrentCenter);
			FCenterCandidate Candidate{ PlateIndex, DistanceRad };
			int32 InsertIndex = 0;
			while (InsertIndex < ClosestCenters.Num() && ClosestCenters[InsertIndex].DistanceRad <= DistanceRad)
			{
				++InsertIndex;
			}
			ClosestCenters.Insert(Candidate, InsertIndex);
			if (ClosestCenters.Num() > 4)
			{
				ClosestCenters.RemoveAt(4, 1, EAllowShrinking::No);
			}
		}

		FSidecarGapClassification Classification;
		if (ClosestCenters.IsEmpty() || !Plates.IsValidIndex(ClosestCenters[0].PlateIndex))
		{
			return Classification;
		}

		Classification.PrimaryPlateId = Plates[ClosestCenters[0].PlateIndex].PlateId;
		if (ClosestCenters.Num() < 2 || !Plates.IsValidIndex(ClosestCenters[1].PlateIndex))
		{
			return Classification;
		}

		const FTectonicSidecarPlate& PlateA = Plates[ClosestCenters[0].PlateIndex];
		const FTectonicSidecarPlate& PlateB = Plates[ClosestCenters[1].PlateIndex];
		Classification.SecondaryPlateId = PlateB.PlateId;
		const FVector3d CenterA = PlateA.WorldFromLocal.RotateVector(PlateA.InitialCenter).GetSafeNormal();
		const FVector3d CenterB = PlateB.WorldFromLocal.RotateVector(PlateB.InitialCenter).GetSafeNormal();
		const FVector3d Separation = ProjectOntoTangent(CenterB - CenterA, QueryPoint);
		if (Separation.SquaredLength() <= SidecarDirectionDegeneracyThreshold * SidecarDirectionDegeneracyThreshold)
		{
			return Classification;
		}

		const FVector3d SeparationDirection = Separation.GetSafeNormal();
		const FVector3d VelocityA = ComputeSidecarSurfaceVelocity(PlateA, QueryPoint, Config.PlanetRadiusKm);
		const FVector3d VelocityB = ComputeSidecarSurfaceVelocity(PlateB, QueryPoint, Config.PlanetRadiusKm);
		const double DivergenceScoreKmPerMy = (VelocityB - VelocityA).Dot(SeparationDirection);
		const double SpeedScaleKmPerMy = FMath::Max(VelocityA.Size(), VelocityB.Size());
		const double DivergenceThresholdKmPerMy =
			FMath::Max(Config.DivergenceMinKmPerMy, Config.DivergenceSpeedFraction * SpeedScaleKmPerMy);
		Classification.bDivergent = DivergenceScoreKmPerMy > DivergenceThresholdKmPerMy;

		const FVector3d RidgeDirection = ProjectOntoTangent(VelocityB - VelocityA, QueryPoint);
		Classification.RidgeDirection =
			RidgeDirection.SquaredLength() > SidecarDirectionDegeneracyThreshold * SidecarDirectionDegeneracyThreshold
				? RidgeDirection.GetSafeNormal()
				: FVector3d::ZeroVector;
		return Classification;
	}

	double ComputeLargestFlaggedComponentFraction(
		const FTectonicPlanet& Planet,
		const TArray<uint8>& Flags)
	{
		if (Planet.Samples.IsEmpty() || Flags.Num() != Planet.Samples.Num())
		{
			return 0.0;
		}

		int32 TotalFlaggedSamples = 0;
		int32 LargestComponentSize = 0;
		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());

		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (Flags[SeedSampleIndex] == 0)
			{
				continue;
			}
			++TotalFlaggedSamples;

			if (Visited[SeedSampleIndex] != 0)
			{
				continue;
			}

			TArray<int32> Stack;
			Stack.Add(SeedSampleIndex);
			Visited[SeedSampleIndex] = 1;
			int32 ComponentSize = 0;

			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				++ComponentSize;
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						!Flags.IsValidIndex(NeighborIndex) ||
						Flags[NeighborIndex] == 0 ||
						Visited[NeighborIndex] != 0)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			LargestComponentSize = FMath::Max(LargestComponentSize, ComponentSize);
		}

		return TotalFlaggedSamples > 0
			? static_cast<double>(LargestComponentSize) / static_cast<double>(TotalFlaggedSamples)
			: 0.0;
	}

	double ComputeHighContinentalFragmentationFraction(
		const FTectonicPlanet& Planet,
		const TArray<int32>* MaterialSourcePlateIds)
	{
		if (Planet.Samples.IsEmpty())
		{
			return 0.0;
		}

		TArray<uint8> Visited;
		Visited.Init(0, Planet.Samples.Num());
		TMap<int32, int32> TotalSamplesBySource;
		TMap<int32, int32> LargestComponentBySource;
		int32 TotalHighContinentalSamples = 0;

		for (int32 SeedSampleIndex = 0; SeedSampleIndex < Planet.Samples.Num(); ++SeedSampleIndex)
		{
			if (Visited[SeedSampleIndex] != 0 ||
				Planet.Samples[SeedSampleIndex].ContinentalWeight < 0.5f)
			{
				continue;
			}

			const int32 SourcePlateId =
				MaterialSourcePlateIds != nullptr && MaterialSourcePlateIds->IsValidIndex(SeedSampleIndex)
					? (*MaterialSourcePlateIds)[SeedSampleIndex]
					: Planet.Samples[SeedSampleIndex].PlateId;
			if (SourcePlateId == INDEX_NONE)
			{
				continue;
			}

			TArray<int32> Stack;
			Stack.Add(SeedSampleIndex);
			Visited[SeedSampleIndex] = 1;
			int32 ComponentSize = 0;

			while (!Stack.IsEmpty())
			{
				const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
				++ComponentSize;
				if (!Planet.SampleAdjacency.IsValidIndex(SampleIndex))
				{
					continue;
				}

				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex) ||
						Visited[NeighborIndex] != 0 ||
						Planet.Samples[NeighborIndex].ContinentalWeight < 0.5f)
					{
						continue;
					}

					const int32 NeighborSourcePlateId =
						MaterialSourcePlateIds != nullptr && MaterialSourcePlateIds->IsValidIndex(NeighborIndex)
							? (*MaterialSourcePlateIds)[NeighborIndex]
							: Planet.Samples[NeighborIndex].PlateId;
					if (NeighborSourcePlateId != SourcePlateId)
					{
						continue;
					}

					Visited[NeighborIndex] = 1;
					Stack.Add(NeighborIndex);
				}
			}

			TotalHighContinentalSamples += ComponentSize;
			TotalSamplesBySource.FindOrAdd(SourcePlateId) += ComponentSize;
			int32& LargestComponent = LargestComponentBySource.FindOrAdd(SourcePlateId);
			LargestComponent = FMath::Max(LargestComponent, ComponentSize);
		}

		int32 FragmentedHighSamples = 0;
		for (const TPair<int32, int32>& SourceEntry : TotalSamplesBySource)
		{
			FragmentedHighSamples += SourceEntry.Value - LargestComponentBySource.FindRef(SourceEntry.Key);
		}

		return TotalHighContinentalSamples > 0
			? static_cast<double>(FragmentedHighSamples) / static_cast<double>(TotalHighContinentalSamples)
			: 0.0;
	}

	void BuildProjectedMaterialGrid(
		const TArray<FTectonicSidecarPlate>& Plates,
		FSidecarProjectedMaterialGrid& OutGrid)
	{
		int32 MaterialSampleCount = 0;
		for (const FTectonicSidecarPlate& Plate : Plates)
		{
			MaterialSampleCount += Plate.MaterialSamples.Num();
		}

		if (MaterialSampleCount >= 200000)
		{
			OutGrid.LongitudeBins = 256;
			OutGrid.LatitudeBins = 128;
		}

		OutGrid.Entries.Reset();
		OutGrid.Entries.Reserve(MaterialSampleCount);
		OutGrid.Bins.Reset();
		OutGrid.Bins.SetNum(OutGrid.LongitudeBins * OutGrid.LatitudeBins);

		for (int32 PlateIndex = 0; PlateIndex < Plates.Num(); ++PlateIndex)
		{
			const FTectonicSidecarPlate& Plate = Plates[PlateIndex];
			for (int32 MaterialIndex = 0; MaterialIndex < Plate.MaterialSamples.Num(); ++MaterialIndex)
			{
				const FVector3d WorldPosition =
					Plate.WorldFromLocal.RotateVector(Plate.MaterialSamples[MaterialIndex].LocalPosition).GetSafeNormal();
				const int32 EntryIndex = OutGrid.Entries.Add({
					Plate.PlateId,
					PlateIndex,
					MaterialIndex,
					WorldPosition
				});

				int32 LongitudeBin = 0;
				int32 LatitudeBin = 0;
				UnitToLatLonBins(WorldPosition, OutGrid.LongitudeBins, OutGrid.LatitudeBins, LongitudeBin, LatitudeBin);
				const int32 BinIndex = MaterialGridIndex(LongitudeBin, LatitudeBin, OutGrid.LongitudeBins);
				if (OutGrid.Bins.IsValidIndex(BinIndex))
				{
					OutGrid.Bins[BinIndex].Add(EntryIndex);
				}
			}
		}
	}

	const FSidecarProjectedMaterialRef* FindNearestProjectedMaterial(
		const FSidecarProjectedMaterialGrid& Grid,
		const FVector3d& UnitPosition,
		double& OutNearestDistanceRad)
	{
		OutNearestDistanceRad = 0.0;
		if (Grid.Entries.IsEmpty())
		{
			return nullptr;
		}

		int32 QueryLongitudeBin = 0;
		int32 QueryLatitudeBin = 0;
		UnitToLatLonBins(
			UnitPosition,
			Grid.LongitudeBins,
			Grid.LatitudeBins,
			QueryLongitudeBin,
			QueryLatitudeBin);

		double BestDistanceSq = TNumericLimits<double>::Max();
		int32 BestEntryIndex = INDEX_NONE;
		int32 FoundCandidateCount = 0;
		const int32 MaxSearchRadius = FMath::Max(Grid.LongitudeBins, Grid.LatitudeBins);
		for (int32 Radius = 0; Radius <= MaxSearchRadius; ++Radius)
		{
			const int32 MinLatitudeBin = FMath::Max(0, QueryLatitudeBin - Radius);
			const int32 MaxLatitudeBin = FMath::Min(Grid.LatitudeBins - 1, QueryLatitudeBin + Radius);
			for (int32 LatitudeBin = MinLatitudeBin; LatitudeBin <= MaxLatitudeBin; ++LatitudeBin)
			{
				for (int32 LongitudeOffset = -Radius; LongitudeOffset <= Radius; ++LongitudeOffset)
				{
					if (Radius > 0 &&
						LatitudeBin != QueryLatitudeBin - Radius &&
						LatitudeBin != QueryLatitudeBin + Radius &&
						FMath::Abs(LongitudeOffset) != Radius)
					{
						continue;
					}

					int32 LongitudeBin = (QueryLongitudeBin + LongitudeOffset) % Grid.LongitudeBins;
					if (LongitudeBin < 0)
					{
						LongitudeBin += Grid.LongitudeBins;
					}

					const int32 BinIndex = MaterialGridIndex(LongitudeBin, LatitudeBin, Grid.LongitudeBins);
					if (!Grid.Bins.IsValidIndex(BinIndex))
					{
						continue;
					}

					for (const int32 EntryIndex : Grid.Bins[BinIndex])
					{
						if (!Grid.Entries.IsValidIndex(EntryIndex))
						{
							continue;
						}

						++FoundCandidateCount;
						const double DistanceSq =
							(Grid.Entries[EntryIndex].WorldPosition - UnitPosition).SizeSquared();
						if (DistanceSq < BestDistanceSq)
						{
							BestDistanceSq = DistanceSq;
							BestEntryIndex = EntryIndex;
						}
					}
				}
			}

			if ((Radius >= 1 && FoundCandidateCount >= 16) || Radius == MaxSearchRadius)
			{
				break;
			}
		}

		if (!Grid.Entries.IsValidIndex(BestEntryIndex))
		{
			return nullptr;
		}

		OutNearestDistanceRad = AngularDistanceRad(UnitPosition, Grid.Entries[BestEntryIndex].WorldPosition);
		return &Grid.Entries[BestEntryIndex];
	}

}

void FTectonicPlanetSidecar::Initialize(const FTectonicSidecarConfig& InConfig)
{
	Config = InConfig;
	CurrentStep = 0;
	Plates.Reset();
	PlateIndexById.Reset();
	InitialPlateIds.Reset();
	InitialContinentalWeights.Reset();
	InitialBoundaryFlags.Reset();
	LastMaterialSourcePlateIds.Reset();
	LastMaterialClassifications.Reset();
	LastMaterialOwnerMismatchFlags.Reset();
	LastMaterialOverlapCounts.Reset();
	LastDivergentBoundaryFlags.Reset();
	OceanCrustStore.Reset();
	CrustEventLog.Reset();

	InitialPlanet = FTectonicPlanet{};
	InitialPlanet.Initialize(Config.SampleCount, Config.PlanetRadiusKm);
	InitialPlanet.InitializePlates(
		Config.PlateCount,
		Config.Seed,
		Config.BoundaryWarpAmplitude,
		Config.InitialContinentalFraction);
	InitialPlanet.bEnableAutomaticRifting = false;
	InitialPlanet.bEnableContinentalCollision = false;
	InitialPlanet.bEnableSlabPull = false;
	InitialPlanet.CurrentStep = 0;

	InitialPlateIds.Reserve(InitialPlanet.Samples.Num());
	InitialContinentalWeights.Reserve(InitialPlanet.Samples.Num());
	InitialBoundaryFlags.Init(0, InitialPlanet.Samples.Num());
	for (const FSample& Sample : InitialPlanet.Samples)
	{
		InitialPlateIds.Add(Sample.PlateId);
		InitialContinentalWeights.Add(Sample.ContinentalWeight);
	}

	for (int32 SampleIndex = 0; SampleIndex < InitialPlanet.Samples.Num(); ++SampleIndex)
	{
		const int32 PlateId = InitialPlanet.Samples[SampleIndex].PlateId;
		if (!InitialPlanet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			continue;
		}

		for (const int32 NeighborIndex : InitialPlanet.SampleAdjacency[SampleIndex])
		{
			if (InitialPlanet.Samples.IsValidIndex(NeighborIndex) &&
				InitialPlanet.Samples[NeighborIndex].PlateId != PlateId)
			{
				InitialBoundaryFlags[SampleIndex] = 1;
				break;
			}
		}
		InitialPlanet.Samples[SampleIndex].bIsBoundary = InitialBoundaryFlags[SampleIndex] != 0;
	}

	BuildPlatesFromInitialPlanet();
	AssignSidecarKinematics();
}

void FTectonicPlanetSidecar::AdvanceStep()
{
	for (FTectonicSidecarPlate& Plate : Plates)
	{
		if (Plate.AngularSpeedRadPerMy <= 0.0)
		{
			continue;
		}

		const double AngleRad = Plate.AngularSpeedRadPerMy * Config.DeltaTimeMy;
		const FQuat4d StepRotation(Plate.RotationAxis.GetSafeNormal(), AngleRad);
		Plate.WorldFromLocal = (StepRotation * Plate.WorldFromLocal).GetNormalized();
	}

	++CurrentStep;
	if (Config.bEnableDivergentSpreadingEvents)
	{
		ApplyDivergentSpreadingEventsForCurrentStep();
	}
}

void FTectonicPlanetSidecar::AdvanceSteps(const int32 StepCount)
{
	for (int32 Step = 0; Step < StepCount; ++Step)
	{
		AdvanceStep();
	}
}

bool FTectonicPlanetSidecar::SetPlateKinematicsForTest(
	const int32 PlateId,
	const FVector3d& Axis,
	const double AngularSpeedRadPerMy)
{
	FTectonicSidecarPlate* Plate = FindPlateById(PlateId);
	if (Plate == nullptr)
	{
		return false;
	}

	FVector3d NormalizedAxis = NormalizeOrFallback(Axis, FVector3d(0.0, 0.0, 1.0));
	double NormalizedSpeed = AngularSpeedRadPerMy;
	if (NormalizedSpeed < 0.0)
	{
		NormalizedAxis *= -1.0;
		NormalizedSpeed = -NormalizedSpeed;
	}

	Plate->RotationAxis = NormalizedAxis;
	Plate->AngularSpeedRadPerMy = NormalizedSpeed;
	return true;
}

uint32 FTectonicPlanetSidecar::ComputeSidecarAuthorityHash() const
{
	uint32 Hash = 2166136261u;
	HashSidecarInt32(Hash, Config.SampleCount);
	HashSidecarInt32(Hash, Config.PlateCount);
	HashSidecarInt32(Hash, Config.Seed);
	HashSidecarDouble(Hash, Config.PlanetRadiusKm);
	HashSidecarDouble(Hash, Config.DeltaTimeMy);
	HashSidecarFloat(Hash, Config.InitialContinentalFraction);
	HashSidecarFloat(Hash, Config.BoundaryWarpAmplitude);
	HashSidecarDouble(Hash, Config.MinPlateSpeedKmPerMy);
	HashSidecarDouble(Hash, Config.MaxPlateSpeedKmPerMy);
	Hash = HashSidecarValue(Hash, Config.bForceZeroAngularSpeeds ? 1u : 0u);
	Hash = HashSidecarValue(Hash, static_cast<uint32>(Config.ProjectionMode));
	HashSidecarDouble(Hash, Config.RecoveryToleranceRad);
	HashSidecarDouble(Hash, Config.MeaningfulHitContainmentScore);
	HashSidecarDouble(Hash, Config.DivergenceMinKmPerMy);
	HashSidecarDouble(Hash, Config.DivergenceSpeedFraction);
	HashSidecarInt32(Hash, Config.RidgeGenerationGapSteps);
	HashSidecarDouble(Hash, Config.OverlayContinentalWeightThreshold);
	Hash = HashSidecarValue(Hash, Config.bEnableDivergentSpreadingEvents ? 1u : 0u);
	HashSidecarDouble(Hash, Config.DivergentSpreadingMinKmPerMy);
	Hash = HashSidecarValue(Hash, Config.bForceExplicitProjectionAtRestForTest ? 1u : 0u);
	HashSidecarInt32(Hash, CurrentStep);

	HashSidecarInt32(Hash, InitialPlanet.CurrentStep);
	HashSidecarInt32(Hash, InitialPlanet.Samples.Num());
	for (const FSample& Sample : InitialPlanet.Samples)
	{
		HashSidecarVector(Hash, Sample.Position);
		HashSidecarInt32(Hash, Sample.PlateId);
		HashSidecarFloat(Hash, Sample.ContinentalWeight);
		HashSidecarFloat(Hash, Sample.Elevation);
		HashSidecarFloat(Hash, Sample.Thickness);
		HashSidecarFloat(Hash, Sample.Age);
		Hash = HashSidecarValue(Hash, Sample.bIsBoundary ? 1u : 0u);
	}
	HashSidecarInt32(Hash, InitialPlanet.SampleAdjacency.Num());
	for (const TArray<int32>& Neighbors : InitialPlanet.SampleAdjacency)
	{
		HashSidecarInt32(Hash, Neighbors.Num());
		for (const int32 NeighborIndex : Neighbors)
		{
			HashSidecarInt32(Hash, NeighborIndex);
		}
	}
	HashSidecarInt32(Hash, InitialPlanet.TriangleIndices.Num());
	for (const FIntVector& Triangle : InitialPlanet.TriangleIndices)
	{
		HashSidecarInt32(Hash, Triangle.X);
		HashSidecarInt32(Hash, Triangle.Y);
		HashSidecarInt32(Hash, Triangle.Z);
	}

	TArray<int32> PlateIndexKeys;
	PlateIndexById.GetKeys(PlateIndexKeys);
	Algo::Sort(PlateIndexKeys);
	HashSidecarInt32(Hash, PlateIndexKeys.Num());
	for (const int32 PlateId : PlateIndexKeys)
	{
		HashSidecarInt32(Hash, PlateId);
		HashSidecarInt32(Hash, PlateIndexById.FindRef(PlateId));
	}
	HashSidecarInt32(Hash, InitialPlateIds.Num());
	for (const int32 PlateId : InitialPlateIds)
	{
		HashSidecarInt32(Hash, PlateId);
	}
	HashSidecarInt32(Hash, InitialContinentalWeights.Num());
	for (const float ContinentalWeight : InitialContinentalWeights)
	{
		HashSidecarFloat(Hash, ContinentalWeight);
	}
	HashSidecarInt32(Hash, InitialBoundaryFlags.Num());
	for (const uint8 BoundaryFlag : InitialBoundaryFlags)
	{
		Hash = HashSidecarValue(Hash, static_cast<uint32>(BoundaryFlag));
	}

	HashSidecarInt32(Hash, Plates.Num());
	for (const FTectonicSidecarPlate& Plate : Plates)
	{
		HashSidecarInt32(Hash, Plate.PlateId);
		HashSidecarVector(Hash, Plate.InitialCenter);
		HashSidecarVector(Hash, Plate.RotationAxis);
		HashSidecarDouble(Hash, Plate.AngularSpeedRadPerMy);
		HashSidecarQuat(Hash, Plate.WorldFromLocal);
		HashSidecarDouble(Hash, Plate.InitialContinentalMass);
		HashSidecarDouble(Hash, Plate.InitialFootprintContinentalMass);
		HashSidecarDouble(Hash, Plate.SupportDistanceThresholdRad);
		HashSidecarVector(Hash, Plate.InitialCoreCentroidLocal);
		HashSidecarInt32(Hash, Plate.InitialContinentalCoreSampleCount);
		HashSidecarInt32(Hash, Plate.MaterialSamples.Num());
		for (const FTectonicSidecarMaterialSample& Sample : Plate.MaterialSamples)
		{
			HashSidecarVector(Hash, Sample.LocalPosition);
			HashSidecarInt32(Hash, Sample.InitialCanonicalSampleIndex);
			HashSidecarFloat(Hash, Sample.ContinentalWeight);
			HashSidecarFloat(Hash, Sample.Elevation);
			HashSidecarFloat(Hash, Sample.Thickness);
			HashSidecarFloat(Hash, Sample.Age);
		}
		HashSidecarInt32(Hash, Plate.MaterialGrid.LongitudeBins);
		HashSidecarInt32(Hash, Plate.MaterialGrid.LatitudeBins);
		HashSidecarInt32(Hash, Plate.MaterialGrid.Bins.Num());
		for (const TArray<int32>& Bin : Plate.MaterialGrid.Bins)
		{
			HashSidecarInt32(Hash, Bin.Num());
			for (const int32 SampleIndex : Bin)
			{
				HashSidecarInt32(Hash, SampleIndex);
			}
		}
		HashSidecarInt32(Hash, Plate.FootprintTriangles.Num());
		for (const FTectonicSidecarFootprintTriangle& Triangle : Plate.FootprintTriangles)
		{
			HashSidecarInt32(Hash, Triangle.GlobalTriangleIndex);
			HashSidecarDouble(Hash, Triangle.UnitSphereArea);
			for (const FTectonicSidecarFootprintVertex& Vertex : Triangle.Vertices)
			{
				HashSidecarVector(Hash, Vertex.LocalPosition);
				HashSidecarInt32(Hash, Vertex.InitialCanonicalSampleIndex);
				HashSidecarFloat(Hash, Vertex.ContinentalWeight);
				HashSidecarFloat(Hash, Vertex.Elevation);
				HashSidecarFloat(Hash, Vertex.Thickness);
				HashSidecarFloat(Hash, Vertex.Age);
			}
		}
	}

	Hash = HashSidecarValue(Hash, OceanCrustStore.ComputeCanonicalHash());
	Hash = HashSidecarValue(Hash, CrustEventLog.ComputeAppendOrderHash());
	return Hash;
}

bool FTectonicPlanetSidecar::ApplyDivergentSpreadingEventForTest(
	const FSidecarDivergentSpreadingEventInput& Input,
	int32* OutCrustId,
	int32* OutEventId)
{
	if (Input.PlateA == INDEX_NONE ||
		Input.PlateB == INDEX_NONE ||
		Input.PlateA == Input.PlateB ||
		FindPlateById(Input.PlateA) == nullptr ||
		FindPlateById(Input.PlateB) == nullptr ||
		Input.BirthEdges.IsEmpty())
	{
		return false;
	}

	const double CurrentTimeMy = static_cast<double>(CurrentStep) * Config.DeltaTimeMy;
	return ApplyDivergentSpreadingEvent(
		OceanCrustStore,
		CrustEventLog,
		Input,
		CurrentStep,
		CurrentTimeMy,
		Config.RidgeGenerationGapSteps,
		0.0,
		OutCrustId,
		OutEventId);
}

TArray<FSidecarOwnerEdge> FTectonicPlanetSidecar::EnumerateOwnerEdgesSorted() const
{
	TArray<FSidecarOwnerEdge> Edges;
	const int32 SampleCount = InitialPlanet.Samples.Num();
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		if (!InitialPlanet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			continue;
		}

		const FVector3d SamplePosition = InitialPlanet.Samples[SampleIndex].Position.GetSafeNormal();
		const int32 OwnerPlateId = FindNearestRotatedCenterPlateId(SamplePosition);
		for (const int32 NeighborIndex : InitialPlanet.SampleAdjacency[SampleIndex])
		{
			if (NeighborIndex <= SampleIndex || !InitialPlanet.Samples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const FVector3d NeighborPosition = InitialPlanet.Samples[NeighborIndex].Position.GetSafeNormal();
			const int32 NeighborOwnerPlateId = FindNearestRotatedCenterPlateId(NeighborPosition);
			if (OwnerPlateId == NeighborOwnerPlateId ||
				OwnerPlateId == INDEX_NONE ||
				NeighborOwnerPlateId == INDEX_NONE)
			{
				continue;
			}

			FSidecarOwnerEdge Edge;
			Edge.Key = FSidecarBoundaryEdgeKey::Make(SampleIndex, NeighborIndex);
			Edge.SampleAOwnerPlateId = OwnerPlateId;
			Edge.SampleBOwnerPlateId = NeighborOwnerPlateId;
			Edge.LowPlateId = FMath::Min(OwnerPlateId, NeighborOwnerPlateId);
			Edge.HighPlateId = FMath::Max(OwnerPlateId, NeighborOwnerPlateId);
			Edge.SampleAPosition = SamplePosition;
			Edge.SampleBPosition = NeighborPosition;
			Edge.Midpoint = NormalizeOrFallback(SamplePosition + NeighborPosition, SamplePosition);
			Edge.LengthRad = AngularDistanceRad(SamplePosition, NeighborPosition);
			Edges.Add(Edge);
		}
	}

	Algo::Sort(Edges, [](const FSidecarOwnerEdge& A, const FSidecarOwnerEdge& B)
	{
		return A.Key < B.Key;
	});

	int32 WriteIndex = 0;
	for (int32 ReadIndex = 0; ReadIndex < Edges.Num(); ++ReadIndex)
	{
		if (WriteIndex == 0 || !(Edges[ReadIndex].Key == Edges[WriteIndex - 1].Key))
		{
			if (WriteIndex != ReadIndex)
			{
				Edges[WriteIndex] = Edges[ReadIndex];
			}
			++WriteIndex;
		}
	}
	Edges.SetNum(WriteIndex);
	return Edges;
}

bool FTectonicPlanetSidecar::ComputeBoundaryNormalSeparationKmPerMy(
	const FSidecarOwnerEdge& OwnerEdge,
	double& OutSeparationKmPerMy) const
{
	OutSeparationKmPerMy = 0.0;
	const FTectonicSidecarPlate* LowPlate = FindPlateById(OwnerEdge.LowPlateId);
	const FTectonicSidecarPlate* HighPlate = FindPlateById(OwnerEdge.HighPlateId);
	if (LowPlate == nullptr || HighPlate == nullptr || LowPlate->PlateId == HighPlate->PlateId)
	{
		return false;
	}

	FVector3d LowToHigh = FVector3d::ZeroVector;
	if (!ComputeBoundaryLowToHighDirection(*LowPlate, *HighPlate, OwnerEdge, LowToHigh))
	{
		return false;
	}

	const FVector3d QueryPoint = NormalizeOrFallback(OwnerEdge.Midpoint, OwnerEdge.SampleAPosition);
	const FVector3d LowVelocity = ComputeSidecarSurfaceVelocity(*LowPlate, QueryPoint, Config.PlanetRadiusKm);
	const FVector3d HighVelocity = ComputeSidecarSurfaceVelocity(*HighPlate, QueryPoint, Config.PlanetRadiusKm);
	OutSeparationKmPerMy = (HighVelocity - LowVelocity).Dot(LowToHigh);
	return true;
}

void FTectonicPlanetSidecar::ApplyDivergentSpreadingEventsForCurrentStep()
{
	struct FAcceptedDivergentEdge
	{
		FSidecarOwnerEdge OwnerEdge;
		FVector3d BoundaryNormal = FVector3d::ZeroVector;
		double NormalSeparationKmPerMy = 0.0;
		double EdgeLengthKm = 0.0;
		double CreatedAreaKm2 = 0.0;
	};

	struct FEndpointRef
	{
		int32 SampleId = INDEX_NONE;
		int32 LocalEdgeIndex = INDEX_NONE;
	};

	struct FEdgeComponent
	{
		int32 LowPlateId = INDEX_NONE;
		int32 HighPlateId = INDEX_NONE;
		FSidecarBoundaryEdgeKey ComponentId;
		TArray<int32> LocalEdgeIndices;
	};

	const double CurrentTimeMy = static_cast<double>(CurrentStep) * Config.DeltaTimeMy;
	for (FSidecarOceanCrustRecord& Record : OceanCrustStore.Records)
	{
		Record.AgeMy = FMath::Max(0.0, CurrentTimeMy - Record.BirthTimeMy);
	}

	TArray<FAcceptedDivergentEdge> AcceptedEdges;
	const TArray<FSidecarOwnerEdge> OwnerEdges = EnumerateOwnerEdgesSorted();
	for (const FSidecarOwnerEdge& OwnerEdge : OwnerEdges)
	{
		double NormalSeparationKmPerMy = 0.0;
		if (!ComputeBoundaryNormalSeparationKmPerMy(OwnerEdge, NormalSeparationKmPerMy) ||
			NormalSeparationKmPerMy < Config.DivergentSpreadingMinKmPerMy)
		{
			continue;
		}

		const FTectonicSidecarPlate* LowPlate = FindPlateById(OwnerEdge.LowPlateId);
		const FTectonicSidecarPlate* HighPlate = FindPlateById(OwnerEdge.HighPlateId);
		if (LowPlate == nullptr || HighPlate == nullptr)
		{
			continue;
		}
		double PairSeparationKmPerMy = 0.0;
		if (!ComputePlateCenterSeparationKmPerMy(
				*LowPlate,
				*HighPlate,
				Config.PlanetRadiusKm,
				PairSeparationKmPerMy) ||
			PairSeparationKmPerMy < Config.DivergentSpreadingMinKmPerMy)
		{
			continue;
		}

		FVector3d BoundaryNormal = FVector3d::ZeroVector;
		if (!ComputeBoundaryLowToHighDirection(*LowPlate, *HighPlate, OwnerEdge, BoundaryNormal))
		{
			continue;
		}

		FAcceptedDivergentEdge AcceptedEdge;
		AcceptedEdge.OwnerEdge = OwnerEdge;
		AcceptedEdge.BoundaryNormal = BoundaryNormal;
		AcceptedEdge.NormalSeparationKmPerMy = NormalSeparationKmPerMy;
		AcceptedEdge.EdgeLengthKm = OwnerEdge.LengthRad * Config.PlanetRadiusKm;
		AcceptedEdge.CreatedAreaKm2 =
			AcceptedEdge.EdgeLengthKm *
			NormalSeparationKmPerMy *
			Config.DeltaTimeMy;
		if (AcceptedEdge.CreatedAreaKm2 > 0.0)
		{
			AcceptedEdges.Add(AcceptedEdge);
		}
	}

	Algo::Sort(AcceptedEdges, [](const FAcceptedDivergentEdge& A, const FAcceptedDivergentEdge& B)
	{
		if (A.OwnerEdge.LowPlateId != B.OwnerEdge.LowPlateId)
		{
			return A.OwnerEdge.LowPlateId < B.OwnerEdge.LowPlateId;
		}
		if (A.OwnerEdge.HighPlateId != B.OwnerEdge.HighPlateId)
		{
			return A.OwnerEdge.HighPlateId < B.OwnerEdge.HighPlateId;
		}
		return A.OwnerEdge.Key < B.OwnerEdge.Key;
	});

	int32 BlockStart = 0;
	const double CoalescingToleranceRad =
		2.0 * FMath::Sqrt((4.0 * PI) / FMath::Max(1.0, static_cast<double>(Config.SampleCount)));
	while (BlockStart < AcceptedEdges.Num())
	{
		int32 BlockEnd = BlockStart + 1;
		while (BlockEnd < AcceptedEdges.Num() &&
			AcceptedEdges[BlockEnd].OwnerEdge.LowPlateId == AcceptedEdges[BlockStart].OwnerEdge.LowPlateId &&
			AcceptedEdges[BlockEnd].OwnerEdge.HighPlateId == AcceptedEdges[BlockStart].OwnerEdge.HighPlateId)
		{
			++BlockEnd;
		}

		const int32 BlockCount = BlockEnd - BlockStart;
		TArray<int32> Parent;
		Parent.SetNumUninitialized(BlockCount);
		for (int32 Index = 0; Index < BlockCount; ++Index)
		{
			Parent[Index] = Index;
		}

		auto FindRoot = [&Parent](int32 Index)
		{
			while (Parent[Index] != Index)
			{
				Parent[Index] = Parent[Parent[Index]];
				Index = Parent[Index];
			}
			return Index;
		};

		auto UnionRoots = [&Parent, &FindRoot](const int32 A, const int32 B)
		{
			const int32 RootA = FindRoot(A);
			const int32 RootB = FindRoot(B);
			if (RootA == RootB)
			{
				return;
			}
			if (RootA < RootB)
			{
				Parent[RootB] = RootA;
			}
			else
			{
				Parent[RootA] = RootB;
			}
		};

		TArray<FEndpointRef> EndpointRefs;
		EndpointRefs.Reserve(BlockCount * 2);
		for (int32 LocalIndex = 0; LocalIndex < BlockCount; ++LocalIndex)
		{
			const FSidecarBoundaryEdgeKey& Key = AcceptedEdges[BlockStart + LocalIndex].OwnerEdge.Key;
			EndpointRefs.Add({ Key.SampleA, LocalIndex });
			EndpointRefs.Add({ Key.SampleB, LocalIndex });
		}
		Algo::Sort(EndpointRefs, [](const FEndpointRef& A, const FEndpointRef& B)
		{
			if (A.SampleId != B.SampleId)
			{
				return A.SampleId < B.SampleId;
			}
			return A.LocalEdgeIndex < B.LocalEdgeIndex;
		});

		int32 EndpointStart = 0;
		while (EndpointStart < EndpointRefs.Num())
		{
			int32 EndpointEnd = EndpointStart + 1;
			while (EndpointEnd < EndpointRefs.Num() &&
				EndpointRefs[EndpointEnd].SampleId == EndpointRefs[EndpointStart].SampleId)
			{
				++EndpointEnd;
			}
			for (int32 EndpointIndex = EndpointStart + 1; EndpointIndex < EndpointEnd; ++EndpointIndex)
			{
				UnionRoots(EndpointRefs[EndpointStart].LocalEdgeIndex, EndpointRefs[EndpointIndex].LocalEdgeIndex);
			}
			EndpointStart = EndpointEnd;
		}

		TArray<int32> RootToComponentIndex;
		RootToComponentIndex.Init(INDEX_NONE, BlockCount);
		TArray<FEdgeComponent> Components;
		for (int32 LocalIndex = 0; LocalIndex < BlockCount; ++LocalIndex)
		{
			const int32 Root = FindRoot(LocalIndex);
			int32& ComponentIndex = RootToComponentIndex[Root];
			if (ComponentIndex == INDEX_NONE)
			{
				ComponentIndex = Components.Num();
				FEdgeComponent Component;
				Component.LowPlateId = AcceptedEdges[BlockStart + LocalIndex].OwnerEdge.LowPlateId;
				Component.HighPlateId = AcceptedEdges[BlockStart + LocalIndex].OwnerEdge.HighPlateId;
				Component.ComponentId = AcceptedEdges[BlockStart + LocalIndex].OwnerEdge.Key;
				Components.Add(Component);
			}
			FEdgeComponent& Component = Components[ComponentIndex];
			if (AcceptedEdges[BlockStart + LocalIndex].OwnerEdge.Key < Component.ComponentId)
			{
				Component.ComponentId = AcceptedEdges[BlockStart + LocalIndex].OwnerEdge.Key;
			}
			Component.LocalEdgeIndices.Add(LocalIndex);
		}

		Algo::Sort(Components, [](const FEdgeComponent& A, const FEdgeComponent& B)
		{
			return A.ComponentId < B.ComponentId;
		});

		for (const FEdgeComponent& Component : Components)
		{
			FSidecarDivergentSpreadingEventInput EventInput;
			EventInput.PlateA = Component.LowPlateId;
			EventInput.PlateB = Component.HighPlateId;
			EventInput.ComponentId = Component.ComponentId;
			EventInput.RidgeGenerationId = 0;
			FVector3d WeightedRidgeDirection = FVector3d::ZeroVector;

			for (const int32 LocalEdgeIndex : Component.LocalEdgeIndices)
			{
				const FAcceptedDivergentEdge& AcceptedEdge = AcceptedEdges[BlockStart + LocalEdgeIndex];
				FSidecarOceanCrustBirthEdge BirthEdge;
				BirthEdge.Key = AcceptedEdge.OwnerEdge.Key;
				BirthEdge.EndpointA = AcceptedEdge.OwnerEdge.SampleAPosition;
				BirthEdge.EndpointB = AcceptedEdge.OwnerEdge.SampleBPosition;
				BirthEdge.BoundaryNormal = AcceptedEdge.BoundaryNormal;
				BirthEdge.LengthKm = AcceptedEdge.EdgeLengthKm;
				BirthEdge.NormalSeparationKmPerMy = AcceptedEdge.NormalSeparationKmPerMy;
				EventInput.BirthEdges.Add(BirthEdge);
				EventInput.DebugSampleIds.Add(AcceptedEdge.OwnerEdge.Key.SampleA);
				EventInput.DebugSampleIds.Add(AcceptedEdge.OwnerEdge.Key.SampleB);
				EventInput.CreatedAreaKm2 += AcceptedEdge.CreatedAreaKm2;
				WeightedRidgeDirection += AcceptedEdge.BoundaryNormal * AcceptedEdge.CreatedAreaKm2;
			}

			EventInput.RidgeDirection = NormalizeOrFallback(
				WeightedRidgeDirection,
				EventInput.BirthEdges[0].BoundaryNormal);
			ApplyDivergentSpreadingEvent(
				OceanCrustStore,
				CrustEventLog,
				EventInput,
				CurrentStep,
				CurrentTimeMy,
				Config.RidgeGenerationGapSteps,
				CoalescingToleranceRad);
		}

		BlockStart = BlockEnd;
	}
}

void FTectonicPlanetSidecar::ProjectToPlanet(
	FTectonicPlanet& OutPlanet,
	FTectonicSidecarProjectionDiagnostics* OutDiagnostics) const
{
	OutPlanet = InitialPlanet;
	OutPlanet.CurrentStep = CurrentStep;
	ResetLastProjectionDebug(OutPlanet.Samples.Num());

	const bool bInitialProjection = IsCurrentWorldPositionAtInitialStep(CurrentStep, Config);
	if (Config.ProjectionMode == ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial)
	{
		ProjectVoronoiOwnershipDecoupledMaterialToPlanet(OutPlanet, OutDiagnostics);
		return;
	}
	if (Config.ProjectionMode == ETectonicSidecarProjectionMode::ExplicitFootprints && !bInitialProjection)
	{
		ProjectExplicitFootprintsToPlanet(OutPlanet, OutDiagnostics);
		return;
	}

	TArray<double> NearestSupportDistancesRad;
	NearestSupportDistancesRad.Init(0.0, OutPlanet.Samples.Num());

	FSidecarProjectedMaterialGrid ProjectedMaterialGrid;
	if (!bInitialProjection)
	{
		BuildProjectedMaterialGrid(Plates, ProjectedMaterialGrid);
	}

	for (int32 SampleIndex = 0; SampleIndex < OutPlanet.Samples.Num(); ++SampleIndex)
	{
		FSample& ProjectedSample = OutPlanet.Samples[SampleIndex];
		const FVector3d UnitPosition = ProjectedSample.Position.GetSafeNormal();

		int32 WinningPlateId = INDEX_NONE;
		if (bInitialProjection && InitialPlateIds.IsValidIndex(SampleIndex))
		{
			WinningPlateId = InitialPlateIds[SampleIndex];
		}
		else
		{
			double NearestProjectedMaterialDistanceRad = 0.0;
			const FSidecarProjectedMaterialRef* NearestMaterial =
				FindNearestProjectedMaterial(ProjectedMaterialGrid, UnitPosition, NearestProjectedMaterialDistanceRad);
			WinningPlateId = NearestMaterial != nullptr
				? NearestMaterial->PlateId
				: FindWinningPlateIdForSample(SampleIndex, UnitPosition);
		}

		ProjectedSample.PlateId = WinningPlateId;
		ProjectedSample.SubductionDistanceKm = -1.0f;
		ProjectedSample.RidgeDirection = FVector3d::ZeroVector;
		ProjectedSample.FoldDirection = FVector3d::ZeroVector;
		ProjectedSample.OrogenyType = EOrogenyType::None;

		const FTectonicSidecarPlate* Plate = FindPlateById(WinningPlateId);
		if (Plate == nullptr)
		{
			ProjectedSample.ContinentalWeight = 0.0f;
			ProjectedSample.Elevation = -4.0f;
			ProjectedSample.Thickness = 7.0f;
			ProjectedSample.Age = 0.0f;
			continue;
		}

		if (bInitialProjection &&
			InitialPlanet.Samples.IsValidIndex(SampleIndex) &&
			InitialPlanet.Samples[SampleIndex].PlateId == WinningPlateId)
		{
			const FSample& InitialSample = InitialPlanet.Samples[SampleIndex];
			ProjectedSample.ContinentalWeight = InitialSample.ContinentalWeight;
			ProjectedSample.Elevation = InitialSample.Elevation;
			ProjectedSample.Thickness = InitialSample.Thickness;
			ProjectedSample.Age = InitialSample.Age;
			continue;
		}

		const FVector3d QueryLocalPosition =
			Plate->WorldFromLocal.Inverse().RotateVector(UnitPosition).GetSafeNormal();
		double NearestDistanceRad = 0.0;
		SamplePlateMaterial(
			*Plate,
			QueryLocalPosition,
			ProjectedSample.ContinentalWeight,
			ProjectedSample.Elevation,
			ProjectedSample.Thickness,
			ProjectedSample.Age,
			NearestDistanceRad);
		NearestSupportDistancesRad[SampleIndex] = NearestDistanceRad;
	}

	RecomputeProjectedBoundariesAndPlateMembers(OutPlanet);

	if (OutDiagnostics != nullptr)
	{
		FillProjectionDiagnostics(OutPlanet, NearestSupportDistancesRad, *OutDiagnostics);
	}
}

void FTectonicPlanetSidecar::ProjectExplicitFootprintsToPlanet(
	FTectonicPlanet& OutPlanet,
	FTectonicSidecarProjectionDiagnostics* OutDiagnostics) const
{
	OutPlanet = InitialPlanet;
	OutPlanet.CurrentStep = CurrentStep;

	const int32 SampleCount = OutPlanet.Samples.Num();
	FSidecarExplicitProjectionScratch Scratch;
	Scratch.ExactHitFlags.Init(0, SampleCount);
	Scratch.RecoveryFlags.Init(0, SampleCount);
	Scratch.GapFlags.Init(0, SampleCount);
	Scratch.DivergentGapFlags.Init(0, SampleCount);
	Scratch.OverlapFlags.Init(0, SampleCount);
	Scratch.OceanFillFlags.Init(0, SampleCount);
	Scratch.FabricatedFlags.Init(0, SampleCount);
	Scratch.PlateBoundaryFlags.Init(0, SampleCount);
	Scratch.DivergentBoundaryFlags.Init(0, SampleCount);

	for (const FTectonicSidecarPlate& Plate : Plates)
	{
		Scratch.LocalFootprintContinentalMass += Plate.InitialFootprintContinentalMass;
	}

	for (FPlate& Plate : OutPlanet.Plates)
	{
		Plate.MemberSamples.Reset();
		if (const FTectonicSidecarPlate* SidecarPlate = FindPlateById(Plate.Id))
		{
			Plate.RotationAxis = SidecarPlate->RotationAxis;
			Plate.AngularSpeed = SidecarPlate->AngularSpeedRadPerMy;
			Plate.CumulativeRotation = SidecarPlate->WorldFromLocal;
		}
	}

	TArray<TUniquePtr<FSidecarFootprintRuntime>> Runtimes;
	BuildFootprintRuntimes(Plates, Runtimes);

	const double SampleAreaUnit = SampleCount > 0 ? (4.0 * PI) / static_cast<double>(SampleCount) : 0.0;
	const double RecoveryToleranceRad = GetConfiguredRecoveryToleranceRad(Config);
	const int32 FallbackPlateId = !Plates.IsEmpty() ? Plates[0].PlateId : INDEX_NONE;

	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		FSample& ProjectedSample = OutPlanet.Samples[SampleIndex];
		const FVector3d UnitPosition = ProjectedSample.Position.GetSafeNormal();
		ProjectedSample.SubductionDistanceKm = -1.0f;
		ProjectedSample.RidgeDirection = FVector3d::ZeroVector;
		ProjectedSample.FoldDirection = FVector3d::ZeroVector;
		ProjectedSample.OrogenyType = EOrogenyType::None;
		ProjectedSample.TerraneId = INDEX_NONE;
		ProjectedSample.bIsBoundary = false;

		TArray<FSidecarFootprintHit, TInlineAllocator<8>> Hits;
		for (const TUniquePtr<FSidecarFootprintRuntime>& RuntimePtr : Runtimes)
		{
			const FSidecarFootprintRuntime* Runtime = RuntimePtr.Get();
			if (Runtime == nullptr ||
				Runtime->Plate == nullptr ||
				Runtime->BVH.RootIndex < 0 ||
				Runtime->BoundingCap.Center.IsNearlyZero() ||
				Runtime->BoundingCap.Center.Dot(UnitPosition) < Runtime->BoundingCap.CosAngle)
			{
				continue;
			}

			int32 LocalTriangleIndex = INDEX_NONE;
			FVector3d A = FVector3d::ZeroVector;
			FVector3d B = FVector3d::ZeroVector;
			FVector3d C = FVector3d::ZeroVector;
			FVector3d Barycentric = FVector3d::ZeroVector;
			if (!FindContainingTriangleInBVH(
				Runtime->BVH,
				Runtime->Adapter,
				UnitPosition,
				LocalTriangleIndex,
				A,
				B,
				C,
				Barycentric))
			{
				continue;
			}

			FSidecarFootprintHit Hit;
			if (TryMakeFootprintHit(*Runtime, LocalTriangleIndex, Barycentric, 0.0, Hit))
			{
				Hits.Add(Hit);
			}
		}

		if (!Hits.IsEmpty())
		{
			int32 MeaningfulHitCount = 0;
			FSidecarFootprintHit BestHit;
			for (const FSidecarFootprintHit& Hit : Hits)
			{
				Scratch.MultiHitProjectedContinentalMass += SampleAreaUnit * Hit.ContinentalWeight;
				MeaningfulHitCount += Hit.ContainmentScore >= Config.MeaningfulHitContainmentScore ? 1 : 0;
				if (IsBetterFootprintHit(Hit, BestHit))
				{
					BestHit = Hit;
				}
			}

			ApplyFootprintHitToSample(BestHit, ProjectedSample);
			Scratch.ExactHitFlags[SampleIndex] = 1;
			Scratch.VisibleProjectedContinentalMass += SampleAreaUnit * BestHit.ContinentalWeight;
			if (MeaningfulHitCount > 1)
			{
				Scratch.OverlapFlags[SampleIndex] = 1;
			}
			continue;
		}

		FSidecarFootprintHit RecoveryHit;
		if (TryFindNearestFootprintHit(Runtimes, UnitPosition, RecoveryToleranceRad, RecoveryHit))
		{
			ApplyFootprintHitToSample(RecoveryHit, ProjectedSample);
			Scratch.RecoveryFlags[SampleIndex] = 1;
			Scratch.VisibleProjectedContinentalMass += SampleAreaUnit * RecoveryHit.ContinentalWeight;
			continue;
		}

		const FSidecarGapClassification GapClassification =
			ClassifyExplicitGap(Plates, Config, UnitPosition);
		ProjectedSample.PlateId = GapClassification.PrimaryPlateId != INDEX_NONE
			? GapClassification.PrimaryPlateId
			: FallbackPlateId;
		ProjectedSample.ContinentalWeight = 0.0f;
		ProjectedSample.Thickness = static_cast<float>(SidecarOceanicThicknessKm);
		ProjectedSample.Age = 0.0f;
		ProjectedSample.Elevation = static_cast<float>(
			GapClassification.bDivergent ? SidecarRidgeElevationKm : SidecarAbyssalPlainElevationKm);
		ProjectedSample.RidgeDirection = GapClassification.bDivergent
			? GapClassification.RidgeDirection
			: FVector3d::ZeroVector;
		ProjectedSample.FoldDirection = FVector3d::ZeroVector;
		ProjectedSample.OrogenyType = EOrogenyType::None;
		ProjectedSample.TerraneId = INDEX_NONE;
		ProjectedSample.SubductionDistanceKm = -1.0f;

		Scratch.GapFlags[SampleIndex] = 1;
		Scratch.OceanFillFlags[SampleIndex] = 1;
		if (GapClassification.bDivergent)
		{
			Scratch.DivergentGapFlags[SampleIndex] = 1;
		}
	}

	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		FSample& Sample = OutPlanet.Samples[SampleIndex];
		const int32 PlateArrayIndex = OutPlanet.FindPlateArrayIndexById(Sample.PlateId);
		if (OutPlanet.Plates.IsValidIndex(PlateArrayIndex))
		{
			OutPlanet.Plates[PlateArrayIndex].MemberSamples.Add(SampleIndex);
		}

		if (!OutPlanet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			continue;
		}

		const bool bSampleGap = Scratch.GapFlags[SampleIndex] != 0;
		bool bIsBoundary = false;
		bool bIsPlateBoundary = false;
		bool bIsDivergentBoundary = false;
		for (const int32 NeighborIndex : OutPlanet.SampleAdjacency[SampleIndex])
		{
			if (!OutPlanet.Samples.IsValidIndex(NeighborIndex) ||
				!Scratch.GapFlags.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const bool bNeighborGap = Scratch.GapFlags[NeighborIndex] != 0;
			const bool bOwnerChanged = OutPlanet.Samples[NeighborIndex].PlateId != Sample.PlateId;
			if (!bSampleGap && !bNeighborGap && bOwnerChanged)
			{
				bIsBoundary = true;
				bIsPlateBoundary = true;
			}
			else if (bSampleGap != bNeighborGap)
			{
				bIsBoundary = true;
				if (Scratch.DivergentGapFlags[SampleIndex] != 0 ||
					Scratch.DivergentGapFlags[NeighborIndex] != 0)
				{
					bIsDivergentBoundary = true;
				}
			}
			else if (bSampleGap && bNeighborGap && bOwnerChanged)
			{
				bIsBoundary = true;
				if (Scratch.DivergentGapFlags[SampleIndex] != 0 ||
					Scratch.DivergentGapFlags[NeighborIndex] != 0)
				{
					bIsDivergentBoundary = true;
				}
			}
		}

		Sample.bIsBoundary = bIsBoundary;
		Scratch.PlateBoundaryFlags[SampleIndex] = bIsPlateBoundary ? 1 : 0;
		Scratch.DivergentBoundaryFlags[SampleIndex] = bIsDivergentBoundary ? 1 : 0;
		if (Scratch.GapFlags[SampleIndex] == 0 &&
			Scratch.ExactHitFlags[SampleIndex] == 0 &&
			Scratch.RecoveryFlags[SampleIndex] == 0)
		{
			Scratch.FabricatedFlags[SampleIndex] = 1;
		}
	}

	if (OutDiagnostics != nullptr)
	{
		TArray<double> NearestSupportDistancesRad;
		NearestSupportDistancesRad.Init(0.0, SampleCount);
		FillProjectionDiagnostics(OutPlanet, NearestSupportDistancesRad, *OutDiagnostics);

		auto CountFlags = [](const TArray<uint8>& Flags)
		{
			int32 Count = 0;
			for (const uint8 Flag : Flags)
			{
				Count += Flag != 0 ? 1 : 0;
			}
			return Count;
		};

		OutDiagnostics->LocalFootprintContinentalMass = Scratch.LocalFootprintContinentalMass;
		OutDiagnostics->MultiHitProjectedContinentalMass = Scratch.MultiHitProjectedContinentalMass;
		OutDiagnostics->VisibleProjectedContinentalMass = Scratch.VisibleProjectedContinentalMass;
		OutDiagnostics->FootprintMassRelativeError =
			FMath::Abs(Scratch.MultiHitProjectedContinentalMass - Scratch.LocalFootprintContinentalMass) /
			FMath::Max(Scratch.LocalFootprintContinentalMass, SidecarSmallNumber);
		OutDiagnostics->LocalContinentalMass = Scratch.LocalFootprintContinentalMass;
		OutDiagnostics->ProjectedContinentalMass = Scratch.VisibleProjectedContinentalMass;
		OutDiagnostics->MaterialMassRelativeError =
			FMath::Abs(Scratch.VisibleProjectedContinentalMass - Scratch.LocalFootprintContinentalMass) /
			FMath::Max(Scratch.LocalFootprintContinentalMass, SidecarSmallNumber);

		OutDiagnostics->ExactFootprintHitSampleCount = CountFlags(Scratch.ExactHitFlags);
		OutDiagnostics->RecoveredFootprintSampleCount = CountFlags(Scratch.RecoveryFlags);
		OutDiagnostics->GapSampleCount = CountFlags(Scratch.GapFlags);
		OutDiagnostics->DivergentGapSampleCount = CountFlags(Scratch.DivergentGapFlags);
		OutDiagnostics->NonDivergentGapSampleCount =
			OutDiagnostics->GapSampleCount - OutDiagnostics->DivergentGapSampleCount;
		OutDiagnostics->OverlapSampleCount = CountFlags(Scratch.OverlapFlags);
		OutDiagnostics->OceanFillSampleCount = CountFlags(Scratch.OceanFillFlags);
		OutDiagnostics->FabricatedMaterialSampleCount = CountFlags(Scratch.FabricatedFlags);
		OutDiagnostics->ClassifiedSampleCount =
			OutDiagnostics->ExactFootprintHitSampleCount +
			OutDiagnostics->RecoveredFootprintSampleCount +
			OutDiagnostics->GapSampleCount;

		const double Denominator = FMath::Max(1.0, static_cast<double>(SampleCount));
		OutDiagnostics->GapFraction =
			static_cast<double>(OutDiagnostics->GapSampleCount) / Denominator;
		OutDiagnostics->DivergentGapFraction =
			static_cast<double>(OutDiagnostics->DivergentGapSampleCount) / Denominator;
		OutDiagnostics->NonDivergentGapFraction =
			static_cast<double>(OutDiagnostics->NonDivergentGapSampleCount) / Denominator;
		OutDiagnostics->OverlapFraction =
			static_cast<double>(OutDiagnostics->OverlapSampleCount) / Denominator;
		OutDiagnostics->OceanFillFraction =
			static_cast<double>(OutDiagnostics->OceanFillSampleCount) / Denominator;
		OutDiagnostics->FabricatedMaterialFraction =
			static_cast<double>(OutDiagnostics->FabricatedMaterialSampleCount) / Denominator;
		OutDiagnostics->PlateBoundaryFraction =
			static_cast<double>(CountFlags(Scratch.PlateBoundaryFlags)) / Denominator;
		OutDiagnostics->DivergentBoundaryFraction =
			static_cast<double>(CountFlags(Scratch.DivergentBoundaryFlags)) / Denominator;
		OutDiagnostics->LargestDivergentGapComponentFraction =
			ComputeLargestFlaggedComponentFraction(OutPlanet, Scratch.DivergentGapFlags);
	}
}

void FTectonicPlanetSidecar::ProjectVoronoiOwnershipDecoupledMaterialToPlanet(
	FTectonicPlanet& OutPlanet,
	FTectonicSidecarProjectionDiagnostics* OutDiagnostics) const
{
	OutPlanet = InitialPlanet;
	OutPlanet.CurrentStep = CurrentStep;

	const int32 SampleCount = OutPlanet.Samples.Num();
	ResetLastProjectionDebug(SampleCount);

	FSidecarVoronoiMaterialScratch Scratch;
	Scratch.ExactHitFlags.Init(0, SampleCount);
	Scratch.RecoveryFlags.Init(0, SampleCount);
	Scratch.OceanFillFlags.Init(0, SampleCount);
	Scratch.DivergentOceanFillFlags.Init(0, SampleCount);
	Scratch.FabricatedFlags.Init(0, SampleCount);
	Scratch.MaterialOwnerMismatchFlags.Init(0, SampleCount);
	Scratch.MaterialOverlapFlags.Init(0, SampleCount);
	Scratch.DivergentBoundaryFlags.Init(0, SampleCount);

	for (const FTectonicSidecarPlate& Plate : Plates)
	{
		Scratch.LocalFootprintContinentalMass += Plate.InitialFootprintContinentalMass;
	}

	TArray<TUniquePtr<FSidecarFootprintRuntime>> Runtimes;
	BuildFootprintRuntimes(Plates, Runtimes);
	FSidecarProjectedMaterialGrid ProjectedMaterialGrid;
	BuildProjectedMaterialGrid(Plates, ProjectedMaterialGrid);

	const double SampleAreaUnit = SampleCount > 0 ? (4.0 * PI) / static_cast<double>(SampleCount) : 0.0;
	const int32 FallbackPlateId = !Plates.IsEmpty() ? Plates[0].PlateId : INDEX_NONE;

	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		FSample& ProjectedSample = OutPlanet.Samples[SampleIndex];
		const FVector3d UnitPosition = ProjectedSample.Position.GetSafeNormal();
		const int32 OwnerPlateId = FindNearestRotatedCenterPlateId(UnitPosition);

		ProjectedSample.PlateId = OwnerPlateId != INDEX_NONE ? OwnerPlateId : FallbackPlateId;
		ProjectedSample.SubductionDistanceKm = -1.0f;
		ProjectedSample.RidgeDirection = FVector3d::ZeroVector;
		ProjectedSample.FoldDirection = FVector3d::ZeroVector;
		ProjectedSample.OrogenyType = EOrogenyType::None;
		ProjectedSample.TerraneId = INDEX_NONE;
		ProjectedSample.bIsBoundary = false;

		TArray<FSidecarFootprintHit, TInlineAllocator<8>> Hits;
		for (const TUniquePtr<FSidecarFootprintRuntime>& RuntimePtr : Runtimes)
		{
			const FSidecarFootprintRuntime* Runtime = RuntimePtr.Get();
			if (Runtime == nullptr ||
				Runtime->Plate == nullptr ||
				Runtime->BVH.RootIndex < 0 ||
				Runtime->BoundingCap.Center.IsNearlyZero() ||
				Runtime->BoundingCap.Center.Dot(UnitPosition) < Runtime->BoundingCap.CosAngle)
			{
				continue;
			}

			int32 LocalTriangleIndex = INDEX_NONE;
			FVector3d A = FVector3d::ZeroVector;
			FVector3d B = FVector3d::ZeroVector;
			FVector3d C = FVector3d::ZeroVector;
			FVector3d Barycentric = FVector3d::ZeroVector;
			if (!FindContainingTriangleInBVH(
				Runtime->BVH,
				Runtime->Adapter,
				UnitPosition,
				LocalTriangleIndex,
				A,
				B,
				C,
				Barycentric))
			{
				continue;
			}

			FSidecarFootprintHit Hit;
			if (TryMakeFootprintHit(*Runtime, LocalTriangleIndex, Barycentric, 0.0, Hit))
			{
				Hits.Add(Hit);
			}
		}

		int32 MaterialSourcePlateId = INDEX_NONE;
		uint8 MaterialClassification =
			static_cast<uint8>(ETectonicSidecarMaterialClassification::None);
		int32 MeaningfulHitCount = 0;

		if (!Hits.IsEmpty())
		{
			FSidecarFootprintHit BestHit;
			for (const FSidecarFootprintHit& Hit : Hits)
			{
				Scratch.MultiHitProjectedContinentalMass += SampleAreaUnit * Hit.ContinentalWeight;
				MeaningfulHitCount += Hit.ContainmentScore >= Config.MeaningfulHitContainmentScore ? 1 : 0;
				if (IsBetterVisibleMaterialHit(Hit, BestHit))
				{
					BestHit = Hit;
				}
			}

			ApplyFootprintMaterialToSample(BestHit, ProjectedSample);
			MaterialSourcePlateId = BestHit.PlateId;
			MaterialClassification =
				static_cast<uint8>(ETectonicSidecarMaterialClassification::ExactFootprint);
			Scratch.ExactHitFlags[SampleIndex] = 1;
			Scratch.VisibleProjectedContinentalMass += SampleAreaUnit * BestHit.ContinentalWeight;
			if (MeaningfulHitCount > 1)
			{
				Scratch.MaterialOverlapFlags[SampleIndex] = 1;
			}
		}
		else
		{
			double NearestProjectedMaterialDistanceRad = 0.0;
			const FSidecarProjectedMaterialRef* NearestProjectedMaterial =
				FindNearestProjectedMaterial(ProjectedMaterialGrid, UnitPosition, NearestProjectedMaterialDistanceRad);
			const FTectonicSidecarPlate* MaterialPlate = NearestProjectedMaterial != nullptr
				? FindPlateById(NearestProjectedMaterial->PlateId)
				: nullptr;
			if (MaterialPlate != nullptr)
			{
				const FVector3d QueryLocalPosition =
					MaterialPlate->WorldFromLocal.Inverse().RotateVector(UnitPosition).GetSafeNormal();
				double NearestLocalDistanceRad = 0.0;
				SamplePlateMaterial(
					*MaterialPlate,
					QueryLocalPosition,
					ProjectedSample.ContinentalWeight,
					ProjectedSample.Elevation,
					ProjectedSample.Thickness,
					ProjectedSample.Age,
					NearestLocalDistanceRad);
				ProjectedSample.RidgeDirection = FVector3d::ZeroVector;
				ProjectedSample.FoldDirection = FVector3d::ZeroVector;
				ProjectedSample.OrogenyType = EOrogenyType::None;
				ProjectedSample.TerraneId = INDEX_NONE;
				ProjectedSample.SubductionDistanceKm = -1.0f;
				MaterialSourcePlateId = MaterialPlate->PlateId;
				MaterialClassification =
					static_cast<uint8>(ETectonicSidecarMaterialClassification::ProjectedMaterialCloud);
				Scratch.RecoveryFlags[SampleIndex] = 1;
				Scratch.VisibleProjectedContinentalMass +=
					SampleAreaUnit * static_cast<double>(ProjectedSample.ContinentalWeight);
			}
			else
			{
				const FSidecarGapClassification DivergenceClassification =
					ClassifyExplicitGap(Plates, Config, UnitPosition);
				ProjectedSample.ContinentalWeight = 0.0f;
				ProjectedSample.Thickness = static_cast<float>(SidecarOceanicThicknessKm);
				ProjectedSample.Age = 0.0f;
				ProjectedSample.Elevation = static_cast<float>(
					DivergenceClassification.bDivergent ? SidecarRidgeElevationKm : SidecarAbyssalPlainElevationKm);
				ProjectedSample.RidgeDirection = DivergenceClassification.bDivergent
					? DivergenceClassification.RidgeDirection
					: FVector3d::ZeroVector;
				ProjectedSample.FoldDirection = FVector3d::ZeroVector;
				ProjectedSample.OrogenyType = EOrogenyType::None;
				ProjectedSample.TerraneId = INDEX_NONE;
				ProjectedSample.SubductionDistanceKm = -1.0f;

				MaterialClassification = static_cast<uint8>(
					DivergenceClassification.bDivergent
						? ETectonicSidecarMaterialClassification::DivergentOceanFill
						: ETectonicSidecarMaterialClassification::OceanFallback);
				Scratch.OceanFillFlags[SampleIndex] = 1;
				if (DivergenceClassification.bDivergent)
				{
					Scratch.DivergentOceanFillFlags[SampleIndex] = 1;
				}
			}
		}

		if (LastMaterialSourcePlateIds.IsValidIndex(SampleIndex))
		{
			LastMaterialSourcePlateIds[SampleIndex] = MaterialSourcePlateId;
		}
		if (LastMaterialClassifications.IsValidIndex(SampleIndex))
		{
			LastMaterialClassifications[SampleIndex] = MaterialClassification;
		}
		if (LastMaterialOverlapCounts.IsValidIndex(SampleIndex))
		{
			LastMaterialOverlapCounts[SampleIndex] = MeaningfulHitCount;
		}

		const bool bOwnerMismatch =
			MaterialSourcePlateId != INDEX_NONE && MaterialSourcePlateId != ProjectedSample.PlateId;
		if (bOwnerMismatch)
		{
			Scratch.MaterialOwnerMismatchFlags[SampleIndex] = 1;
			if (LastMaterialOwnerMismatchFlags.IsValidIndex(SampleIndex))
			{
				LastMaterialOwnerMismatchFlags[SampleIndex] = 1;
			}
		}

		if (MaterialSourcePlateId == INDEX_NONE && ProjectedSample.ContinentalWeight > 0.0f)
		{
			Scratch.FabricatedFlags[SampleIndex] = 1;
		}
	}

	RecomputeProjectedBoundariesAndPlateMembers(OutPlanet);

	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const FSample& Sample = OutPlanet.Samples[SampleIndex];
		if (!Sample.bIsBoundary)
		{
			continue;
		}

		const FSidecarGapClassification DivergenceClassification =
			ClassifyExplicitGap(Plates, Config, Sample.Position.GetSafeNormal());
		if (DivergenceClassification.bDivergent)
		{
			Scratch.DivergentBoundaryFlags[SampleIndex] = 1;
			if (LastDivergentBoundaryFlags.IsValidIndex(SampleIndex))
			{
				LastDivergentBoundaryFlags[SampleIndex] = 1;
			}
		}
	}

	if (OutDiagnostics != nullptr)
	{
		TArray<double> NearestSupportDistancesRad;
		NearestSupportDistancesRad.Init(0.0, SampleCount);
		FillProjectionDiagnostics(OutPlanet, NearestSupportDistancesRad, *OutDiagnostics);

		const double Denominator = FMath::Max(1.0, static_cast<double>(SampleCount));
		const int32 ExactHitCount = CountFlags(Scratch.ExactHitFlags);
		const int32 RecoveryCount = CountFlags(Scratch.RecoveryFlags);
		const int32 OceanFillCount = CountFlags(Scratch.OceanFillFlags);
		const int32 MaterialOverlapCount = CountFlags(Scratch.MaterialOverlapFlags);
		const int32 MaterialMismatchCount = CountFlags(Scratch.MaterialOwnerMismatchFlags);
		const int32 FabricatedCount = CountFlags(Scratch.FabricatedFlags);
		const int32 DivergentBoundaryCount = CountFlags(Scratch.DivergentBoundaryFlags);

		OutDiagnostics->LocalFootprintContinentalMass = Scratch.LocalFootprintContinentalMass;
		OutDiagnostics->MultiHitProjectedContinentalMass = Scratch.MultiHitProjectedContinentalMass;
		OutDiagnostics->VisibleProjectedContinentalMass = Scratch.VisibleProjectedContinentalMass;
		OutDiagnostics->FootprintMassRelativeError =
			FMath::Abs(Scratch.MultiHitProjectedContinentalMass - Scratch.LocalFootprintContinentalMass) /
			FMath::Max(Scratch.LocalFootprintContinentalMass, SidecarSmallNumber);
		OutDiagnostics->LocalContinentalMass = Scratch.LocalFootprintContinentalMass;
		OutDiagnostics->ProjectedContinentalMass = Scratch.VisibleProjectedContinentalMass;
		OutDiagnostics->MaterialMassRelativeError =
			FMath::Abs(Scratch.VisibleProjectedContinentalMass - Scratch.LocalFootprintContinentalMass) /
			FMath::Max(Scratch.LocalFootprintContinentalMass, SidecarSmallNumber);

		OutDiagnostics->ExactFootprintHitSampleCount = ExactHitCount;
		OutDiagnostics->RecoveredFootprintSampleCount = RecoveryCount;
		OutDiagnostics->GapSampleCount = 0;
		OutDiagnostics->DivergentGapSampleCount = 0;
		OutDiagnostics->NonDivergentGapSampleCount = 0;
		OutDiagnostics->OverlapSampleCount = MaterialOverlapCount;
		OutDiagnostics->OceanFillSampleCount = OceanFillCount;
		OutDiagnostics->FabricatedMaterialSampleCount = FabricatedCount;
		OutDiagnostics->ClassifiedSampleCount = SampleCount;

		OutDiagnostics->GapFraction = 0.0;
		OutDiagnostics->DivergentGapFraction = 0.0;
		OutDiagnostics->NonDivergentGapFraction = 0.0;
		OutDiagnostics->OverlapFraction = static_cast<double>(MaterialOverlapCount) / Denominator;
		OutDiagnostics->OceanFillFraction = static_cast<double>(OceanFillCount) / Denominator;
		OutDiagnostics->FabricatedMaterialFraction = static_cast<double>(FabricatedCount) / Denominator;
		OutDiagnostics->PlateBoundaryFraction = OutDiagnostics->BoundaryFraction;
		OutDiagnostics->DivergentBoundaryFraction = static_cast<double>(DivergentBoundaryCount) / Denominator;
		OutDiagnostics->LargestDivergentGapComponentFraction =
			ComputeLargestFlaggedComponentFraction(OutPlanet, Scratch.DivergentBoundaryFlags);

		OutDiagnostics->OwnershipGapSampleCount = 0;
		OutDiagnostics->OwnershipOverlapSampleCount = 0;
		OutDiagnostics->OwnershipGapFraction = 0.0;
		OutDiagnostics->OwnershipOverlapFraction = 0.0;
		OutDiagnostics->CarriedContinentalMassRelativeError = OutDiagnostics->MaterialMassRelativeError;
		OutDiagnostics->MaterialFabricatedFraction = OutDiagnostics->FabricatedMaterialFraction;
		OutDiagnostics->MaterialOwnerMismatchSampleCount = MaterialMismatchCount;
		OutDiagnostics->MaterialOwnerMismatchFraction =
			static_cast<double>(MaterialMismatchCount) / Denominator;
		OutDiagnostics->MaterialOverlapSupportSampleCount = MaterialOverlapCount;
		OutDiagnostics->MaterialOverlapFraction =
			static_cast<double>(MaterialOverlapCount) / Denominator;

		const double InitialHighCWFragmentation =
			ComputeHighContinentalFragmentationFraction(InitialPlanet, nullptr);
		const double CurrentHighCWFragmentation =
			ComputeHighContinentalFragmentationFraction(OutPlanet, &LastMaterialSourcePlateIds);
		OutDiagnostics->HighCWFragmentationDelta =
			FMath::Max(0.0, CurrentHighCWFragmentation - InitialHighCWFragmentation);
	}
}

uint32 FTectonicPlanetSidecar::ComputeProjectionHash(const FTectonicPlanet& Planet)
{
	uint32 Hash = 2166136261u;
	for (const FSample& Sample : Planet.Samples)
	{
		Hash = HashSidecarValue(Hash, static_cast<uint32>(Sample.PlateId + 0x9e3779b9));
		Hash = HashSidecarValue(Hash, static_cast<uint32>(FMath::RoundToInt(Sample.ContinentalWeight * 10000.0f)));
		Hash = HashSidecarValue(Hash, static_cast<uint32>(FMath::RoundToInt(Sample.Elevation * 1000.0f)));
		Hash = HashSidecarValue(Hash, Sample.bIsBoundary ? 1u : 0u);
	}
	return Hash;
}

void FTectonicPlanetSidecar::BuildPlatesFromInitialPlanet()
{
	for (const FPlate& InitialPlate : InitialPlanet.Plates)
	{
		FTectonicSidecarPlate& Plate = Plates.AddDefaulted_GetRef();
		Plate.PlateId = InitialPlate.Id;
		PlateIndexById.Add(Plate.PlateId, Plates.Num() - 1);
	}

	for (int32 SampleIndex = 0; SampleIndex < InitialPlanet.Samples.Num(); ++SampleIndex)
	{
		const FSample& Sample = InitialPlanet.Samples[SampleIndex];
		FTectonicSidecarPlate* Plate = FindPlateById(Sample.PlateId);
		if (Plate == nullptr)
		{
			continue;
		}

		FTectonicSidecarMaterialSample& MaterialSample = Plate->MaterialSamples.AddDefaulted_GetRef();
		MaterialSample.LocalPosition = Sample.Position.GetSafeNormal();
		MaterialSample.InitialCanonicalSampleIndex = SampleIndex;
		MaterialSample.ContinentalWeight = Sample.ContinentalWeight;
		MaterialSample.Elevation = Sample.Elevation;
		MaterialSample.Thickness = Sample.Thickness;
		MaterialSample.Age = Sample.Age;
	}

	BuildFootprintsFromInitialPlanet();

	for (FTectonicSidecarPlate& Plate : Plates)
	{
		FVector3d CenterSum = FVector3d::ZeroVector;
		for (const FTectonicSidecarMaterialSample& MaterialSample : Plate.MaterialSamples)
		{
			CenterSum += MaterialSample.LocalPosition;
		}
		Plate.InitialCenter = NormalizeOrFallback(CenterSum, FVector3d(0.0, 0.0, 1.0));
		BuildMaterialGrid(Plate);
		ComputeInitialPlateDiagnostics(Plate);
	}
}

void FTectonicPlanetSidecar::BuildFootprintsFromInitialPlanet()
{
	for (FTectonicSidecarPlate& Plate : Plates)
	{
		Plate.FootprintTriangles.Reset();
		Plate.InitialFootprintContinentalMass = 0.0;
	}

	for (int32 GlobalTriangleIndex = 0; GlobalTriangleIndex < InitialPlanet.TriangleIndices.Num(); ++GlobalTriangleIndex)
	{
		const FIntVector& TriangleIndices = InitialPlanet.TriangleIndices[GlobalTriangleIndex];
		const int32 VertexIndices[3] = {
			TriangleIndices.X,
			TriangleIndices.Y,
			TriangleIndices.Z
		};

		if (!InitialPlanet.Samples.IsValidIndex(VertexIndices[0]) ||
			!InitialPlanet.Samples.IsValidIndex(VertexIndices[1]) ||
			!InitialPlanet.Samples.IsValidIndex(VertexIndices[2]))
		{
			continue;
		}

		const FSample& SampleA = InitialPlanet.Samples[VertexIndices[0]];
		const FSample& SampleB = InitialPlanet.Samples[VertexIndices[1]];
		const FSample& SampleC = InitialPlanet.Samples[VertexIndices[2]];
		const int32 OwnerPlateId = ChooseMajorityTrianglePlateId(
			SampleA.PlateId,
			SampleB.PlateId,
			SampleC.PlateId);
		FTectonicSidecarPlate* OwnerPlate = FindPlateById(OwnerPlateId);
		if (OwnerPlate == nullptr)
		{
			continue;
		}

		FTectonicSidecarFootprintTriangle& FootprintTriangle =
			OwnerPlate->FootprintTriangles.AddDefaulted_GetRef();
		FootprintTriangle.GlobalTriangleIndex = GlobalTriangleIndex;
		FootprintTriangle.UnitSphereArea = SphericalTriangleAreaUnit(
			SampleA.Position,
			SampleB.Position,
			SampleC.Position);

		for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
		{
			const FSample& VertexSample = InitialPlanet.Samples[VertexIndices[CornerIndex]];
			FTectonicSidecarFootprintVertex& FootprintVertex = FootprintTriangle.Vertices[CornerIndex];
			FootprintVertex.LocalPosition = VertexSample.Position.GetSafeNormal();
			FootprintVertex.InitialCanonicalSampleIndex = VertexIndices[CornerIndex];
			FootprintVertex.ContinentalWeight = VertexSample.ContinentalWeight;
			FootprintVertex.Elevation = VertexSample.Elevation;
			FootprintVertex.Thickness = VertexSample.Thickness;
			FootprintVertex.Age = VertexSample.Age;
		}

		const double MeanContinentalWeight =
			(static_cast<double>(FootprintTriangle.Vertices[0].ContinentalWeight) +
				static_cast<double>(FootprintTriangle.Vertices[1].ContinentalWeight) +
				static_cast<double>(FootprintTriangle.Vertices[2].ContinentalWeight)) /
			3.0;
		OwnerPlate->InitialFootprintContinentalMass +=
			FootprintTriangle.UnitSphereArea * MeanContinentalWeight;
	}
}

void FTectonicPlanetSidecar::BuildMaterialGrid(FTectonicSidecarPlate& Plate) const
{
	const int32 BinCount = Plate.MaterialGrid.LongitudeBins * Plate.MaterialGrid.LatitudeBins;
	Plate.MaterialGrid.Bins.Reset();
	Plate.MaterialGrid.Bins.SetNum(BinCount);

	for (int32 MaterialIndex = 0; MaterialIndex < Plate.MaterialSamples.Num(); ++MaterialIndex)
	{
		int32 LongitudeBin = 0;
		int32 LatitudeBin = 0;
		UnitToLatLonBins(
			Plate.MaterialSamples[MaterialIndex].LocalPosition,
			Plate.MaterialGrid.LongitudeBins,
			Plate.MaterialGrid.LatitudeBins,
			LongitudeBin,
			LatitudeBin);

		const int32 BinIndex = MaterialGridIndex(LongitudeBin, LatitudeBin, Plate.MaterialGrid.LongitudeBins);
		if (Plate.MaterialGrid.Bins.IsValidIndex(BinIndex))
		{
			Plate.MaterialGrid.Bins[BinIndex].Add(MaterialIndex);
		}
	}
}

void FTectonicPlanetSidecar::ComputeInitialPlateDiagnostics(FTectonicSidecarPlate& Plate) const
{
	FVector3d CoreCentroidSum = FVector3d::ZeroVector;
	double CoreWeightSum = 0.0;
	TArray<double> SamePlateEdgeDistancesRad;

	for (const FTectonicSidecarMaterialSample& MaterialSample : Plate.MaterialSamples)
	{
		Plate.InitialContinentalMass += MaterialSample.ContinentalWeight;
		if (MaterialSample.ContinentalWeight >= 0.5f)
		{
			CoreCentroidSum += MaterialSample.LocalPosition * static_cast<double>(MaterialSample.ContinentalWeight);
			CoreWeightSum += MaterialSample.ContinentalWeight;
			++Plate.InitialContinentalCoreSampleCount;
		}

		const int32 SampleIndex = MaterialSample.InitialCanonicalSampleIndex;
		if (!InitialPlanet.SampleAdjacency.IsValidIndex(SampleIndex) ||
			!InitialPlanet.Samples.IsValidIndex(SampleIndex))
		{
			continue;
		}

		for (const int32 NeighborIndex : InitialPlanet.SampleAdjacency[SampleIndex])
		{
			if (!InitialPlanet.Samples.IsValidIndex(NeighborIndex) ||
				InitialPlanet.Samples[NeighborIndex].PlateId != Plate.PlateId)
			{
				continue;
			}

			SamePlateEdgeDistancesRad.Add(AngularDistanceRad(
				InitialPlanet.Samples[SampleIndex].Position,
				InitialPlanet.Samples[NeighborIndex].Position));
		}
	}

	Plate.InitialCoreCentroidLocal =
		CoreWeightSum > 0.0 ? CoreCentroidSum.GetSafeNormal() : FVector3d::ZeroVector;

	if (!SamePlateEdgeDistancesRad.IsEmpty())
	{
		SamePlateEdgeDistancesRad.Sort();
		const double MedianDistanceRad = SamePlateEdgeDistancesRad[SamePlateEdgeDistancesRad.Num() / 2];
		Plate.SupportDistanceThresholdRad = MedianDistanceRad * 3.0;
	}
	else
	{
		const double ApproxSampleSpacingRad =
			FMath::Sqrt((4.0 * PI) / FMath::Max(1.0, static_cast<double>(Config.SampleCount)));
		Plate.SupportDistanceThresholdRad = ApproxSampleSpacingRad * 3.0;
	}
}

void FTectonicPlanetSidecar::AssignSidecarKinematics()
{
	FRandomStream Stream(Config.Seed ^ 0x51dc4a3f);
	for (FTectonicSidecarPlate& Plate : Plates)
	{
		const FVector3d Center = NormalizeOrFallback(Plate.InitialCenter, FVector3d(0.0, 0.0, 1.0));
		FVector3d Tangent = Stream.VRand();
		Tangent -= Center * FVector3d::DotProduct(Tangent, Center);
		Tangent = NormalizeOrFallback(Tangent, MakeTangentFallback(Center));

		Plate.RotationAxis = NormalizeOrFallback(FVector3d::CrossProduct(Center, Tangent), FVector3d(0.0, 0.0, 1.0));
		const double SpeedKmPerMy = Config.bForceZeroAngularSpeeds
			? 0.0
			: FMath::Lerp(Config.MinPlateSpeedKmPerMy, Config.MaxPlateSpeedKmPerMy, Stream.FRand());
		Plate.AngularSpeedRadPerMy = SpeedKmPerMy / FMath::Max(Config.PlanetRadiusKm, SidecarSmallNumber);
		Plate.WorldFromLocal = FQuat4d::Identity;
	}
}

int32 FTectonicPlanetSidecar::FindWinningPlateIdForSample(
	const int32 SampleIndex,
	const FVector3d& UnitPosition) const
{
	if (IsCurrentWorldPositionAtInitialStep(CurrentStep, Config) && InitialPlateIds.IsValidIndex(SampleIndex))
	{
		return InitialPlateIds[SampleIndex];
	}

	int32 BestPlateId = INDEX_NONE;
	double BestScore = -TNumericLimits<double>::Max();
	for (const FTectonicSidecarPlate& Plate : Plates)
	{
		const FVector3d CurrentCenter = Plate.WorldFromLocal.RotateVector(Plate.InitialCenter).GetSafeNormal();
		const double Score = FVector3d::DotProduct(UnitPosition, CurrentCenter);
		if (Score > BestScore ||
			(FMath::IsNearlyEqual(Score, BestScore) && Plate.PlateId < BestPlateId))
		{
			BestScore = Score;
			BestPlateId = Plate.PlateId;
		}
	}

	return BestPlateId;
}

int32 FTectonicPlanetSidecar::FindNearestRotatedCenterPlateId(const FVector3d& UnitPosition) const
{
	int32 BestPlateId = INDEX_NONE;
	double BestScore = -TNumericLimits<double>::Max();
	for (const FTectonicSidecarPlate& Plate : Plates)
	{
		const FVector3d CurrentCenter = Plate.WorldFromLocal.RotateVector(Plate.InitialCenter).GetSafeNormal();
		const double Score = FVector3d::DotProduct(UnitPosition, CurrentCenter);
		if (Score > BestScore ||
			(FMath::IsNearlyEqual(Score, BestScore) && (BestPlateId == INDEX_NONE || Plate.PlateId < BestPlateId)))
		{
			BestScore = Score;
			BestPlateId = Plate.PlateId;
		}
	}
	return BestPlateId;
}

const FTectonicSidecarPlate* FTectonicPlanetSidecar::FindPlateById(const int32 PlateId) const
{
	const int32* PlateIndex = PlateIndexById.Find(PlateId);
	return PlateIndex != nullptr && Plates.IsValidIndex(*PlateIndex) ? &Plates[*PlateIndex] : nullptr;
}

FTectonicSidecarPlate* FTectonicPlanetSidecar::FindPlateById(const int32 PlateId)
{
	const int32* PlateIndex = PlateIndexById.Find(PlateId);
	return PlateIndex != nullptr && Plates.IsValidIndex(*PlateIndex) ? &Plates[*PlateIndex] : nullptr;
}

void FTectonicPlanetSidecar::SamplePlateMaterial(
	const FTectonicSidecarPlate& Plate,
	const FVector3d& QueryLocalPosition,
	float& OutContinentalWeight,
	float& OutElevation,
	float& OutThickness,
	float& OutAge,
	double& OutNearestDistanceRad) const
{
	OutContinentalWeight = 0.0f;
	OutElevation = -4.0f;
	OutThickness = 7.0f;
	OutAge = 0.0f;
	OutNearestDistanceRad = 0.0;

	if (Plate.MaterialSamples.IsEmpty())
	{
		return;
	}

	double BestDistanceSq[3] = {
		TNumericLimits<double>::Max(),
		TNumericLimits<double>::Max(),
		TNumericLimits<double>::Max()
	};
	int32 BestMaterialIndex[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };

	int32 QueryLongitudeBin = 0;
	int32 QueryLatitudeBin = 0;
	UnitToLatLonBins(
		QueryLocalPosition,
		Plate.MaterialGrid.LongitudeBins,
		Plate.MaterialGrid.LatitudeBins,
		QueryLongitudeBin,
		QueryLatitudeBin);

	int32 FoundCandidateCount = 0;
	const int32 MaxSearchRadius = FMath::Max(Plate.MaterialGrid.LongitudeBins, Plate.MaterialGrid.LatitudeBins);
	for (int32 Radius = 0; Radius <= MaxSearchRadius; ++Radius)
	{
		BestDistanceSq[0] = BestDistanceSq[1] = BestDistanceSq[2] = TNumericLimits<double>::Max();
		BestMaterialIndex[0] = BestMaterialIndex[1] = BestMaterialIndex[2] = INDEX_NONE;
		FoundCandidateCount = 0;

		const int32 MinLatitudeBin = FMath::Max(0, QueryLatitudeBin - Radius);
		const int32 MaxLatitudeBin = FMath::Min(Plate.MaterialGrid.LatitudeBins - 1, QueryLatitudeBin + Radius);
		for (int32 LatitudeBin = MinLatitudeBin; LatitudeBin <= MaxLatitudeBin; ++LatitudeBin)
		{
			for (int32 LongitudeOffset = -Radius; LongitudeOffset <= Radius; ++LongitudeOffset)
			{
				int32 LongitudeBin = (QueryLongitudeBin + LongitudeOffset) % Plate.MaterialGrid.LongitudeBins;
				if (LongitudeBin < 0)
				{
					LongitudeBin += Plate.MaterialGrid.LongitudeBins;
				}

				const int32 BinIndex = MaterialGridIndex(LongitudeBin, LatitudeBin, Plate.MaterialGrid.LongitudeBins);
				if (!Plate.MaterialGrid.Bins.IsValidIndex(BinIndex))
				{
					continue;
				}

				for (const int32 MaterialIndex : Plate.MaterialGrid.Bins[BinIndex])
				{
					if (!Plate.MaterialSamples.IsValidIndex(MaterialIndex))
					{
						continue;
					}

					++FoundCandidateCount;
					const double DistanceSq =
						(Plate.MaterialSamples[MaterialIndex].LocalPosition - QueryLocalPosition).SizeSquared();
					UpdateNearest3(DistanceSq, MaterialIndex, BestDistanceSq, BestMaterialIndex);
				}
			}
		}

		if (FoundCandidateCount >= 3 || Radius == MaxSearchRadius)
		{
			break;
		}
	}

	if (BestMaterialIndex[0] == INDEX_NONE)
	{
		for (int32 MaterialIndex = 0; MaterialIndex < Plate.MaterialSamples.Num(); ++MaterialIndex)
		{
			const double DistanceSq =
				(Plate.MaterialSamples[MaterialIndex].LocalPosition - QueryLocalPosition).SizeSquared();
			UpdateNearest3(DistanceSq, MaterialIndex, BestDistanceSq, BestMaterialIndex);
		}
	}

	double WeightSum = 0.0;
	double ContinentalWeightSum = 0.0;
	double ElevationSum = 0.0;
	double ThicknessSum = 0.0;
	double AgeSum = 0.0;

	for (int32 Slot = 0; Slot < 3; ++Slot)
	{
		const int32 MaterialIndex = BestMaterialIndex[Slot];
		if (!Plate.MaterialSamples.IsValidIndex(MaterialIndex))
		{
			continue;
		}

		const FTectonicSidecarMaterialSample& MaterialSample = Plate.MaterialSamples[MaterialIndex];
		const double Distance = FMath::Sqrt(FMath::Max(BestDistanceSq[Slot], 0.0));
		const double Weight = 1.0 / FMath::Max(Distance, 1.0e-6);
		WeightSum += Weight;
		ContinentalWeightSum += Weight * MaterialSample.ContinentalWeight;
		ElevationSum += Weight * MaterialSample.Elevation;
		ThicknessSum += Weight * MaterialSample.Thickness;
		AgeSum += Weight * MaterialSample.Age;
	}

	if (WeightSum > 0.0)
	{
		OutContinentalWeight = static_cast<float>(ContinentalWeightSum / WeightSum);
		OutElevation = static_cast<float>(ElevationSum / WeightSum);
		OutThickness = static_cast<float>(ThicknessSum / WeightSum);
		OutAge = static_cast<float>(AgeSum / WeightSum);
	}

	if (BestMaterialIndex[0] != INDEX_NONE)
	{
		const FVector3d BestLocalPosition = Plate.MaterialSamples[BestMaterialIndex[0]].LocalPosition;
		OutNearestDistanceRad = AngularDistanceRad(QueryLocalPosition, BestLocalPosition);
	}
}

void FTectonicPlanetSidecar::RecomputeProjectedBoundariesAndPlateMembers(FTectonicPlanet& InOutPlanet) const
{
	for (FPlate& Plate : InOutPlanet.Plates)
	{
		Plate.MemberSamples.Reset();
		if (const FTectonicSidecarPlate* SidecarPlate = FindPlateById(Plate.Id))
		{
			Plate.RotationAxis = SidecarPlate->RotationAxis;
			Plate.AngularSpeed = SidecarPlate->AngularSpeedRadPerMy;
			Plate.CumulativeRotation = SidecarPlate->WorldFromLocal;
		}
	}

	for (int32 SampleIndex = 0; SampleIndex < InOutPlanet.Samples.Num(); ++SampleIndex)
	{
		FSample& Sample = InOutPlanet.Samples[SampleIndex];
		Sample.bIsBoundary = false;

		const int32 PlateArrayIndex = InOutPlanet.FindPlateArrayIndexById(Sample.PlateId);
		if (InOutPlanet.Plates.IsValidIndex(PlateArrayIndex))
		{
			InOutPlanet.Plates[PlateArrayIndex].MemberSamples.Add(SampleIndex);
		}

		if (!InOutPlanet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			continue;
		}

		for (const int32 NeighborIndex : InOutPlanet.SampleAdjacency[SampleIndex])
		{
			if (InOutPlanet.Samples.IsValidIndex(NeighborIndex) &&
				InOutPlanet.Samples[NeighborIndex].PlateId != Sample.PlateId)
			{
				Sample.bIsBoundary = true;
				break;
			}
		}
	}
}

void FTectonicPlanetSidecar::FillProjectionDiagnostics(
	const FTectonicPlanet& ProjectedPlanet,
	const TArray<double>& NearestSupportDistancesRad,
	FTectonicSidecarProjectionDiagnostics& OutDiagnostics) const
{
	OutDiagnostics = FTectonicSidecarProjectionDiagnostics{};
	OutDiagnostics.Step = CurrentStep;
	OutDiagnostics.SampleCount = ProjectedPlanet.Samples.Num();
	OutDiagnostics.PlateCount = Plates.Num();
	OutDiagnostics.ProjectionHash = ComputeProjectionHash(ProjectedPlanet);

	for (const FTectonicSidecarPlate& Plate : Plates)
	{
		OutDiagnostics.LocalContinentalMass += Plate.InitialContinentalMass;
	}
	for (const FSample& Sample : ProjectedPlanet.Samples)
	{
		OutDiagnostics.ProjectedContinentalMass += Sample.ContinentalWeight;
	}
	OutDiagnostics.MaterialMassRelativeError =
		FMath::Abs(OutDiagnostics.ProjectedContinentalMass - OutDiagnostics.LocalContinentalMass) /
		FMath::Max(OutDiagnostics.LocalContinentalMass, SidecarSmallNumber);

	double SupportDistanceSumKm = 0.0;
	for (int32 SampleIndex = 0; SampleIndex < ProjectedPlanet.Samples.Num(); ++SampleIndex)
	{
		const FSample& Sample = ProjectedPlanet.Samples[SampleIndex];
		const FTectonicSidecarPlate* Plate = FindPlateById(Sample.PlateId);
		const double DistanceRad = NearestSupportDistancesRad.IsValidIndex(SampleIndex)
			? NearestSupportDistancesRad[SampleIndex]
			: 0.0;
		const double DistanceKm = DistanceRad * Config.PlanetRadiusKm;
		SupportDistanceSumKm += DistanceKm;
		OutDiagnostics.MaxSupportDistanceKm = FMath::Max(OutDiagnostics.MaxSupportDistanceKm, DistanceKm);
		if (Plate != nullptr &&
			Plate->SupportDistanceThresholdRad > 0.0 &&
			DistanceRad > Plate->SupportDistanceThresholdRad)
		{
			++OutDiagnostics.OutOfSupportSampleCount;
		}
	}
	OutDiagnostics.MeanSupportDistanceKm = ProjectedPlanet.Samples.Num() > 0
		? SupportDistanceSumKm / static_cast<double>(ProjectedPlanet.Samples.Num())
		: 0.0;
	OutDiagnostics.OutOfSupportFraction = ProjectedPlanet.Samples.Num() > 0
		? static_cast<double>(OutDiagnostics.OutOfSupportSampleCount) / static_cast<double>(ProjectedPlanet.Samples.Num())
		: 0.0;

	int32 BoundaryCount = 0;
	for (const FSample& Sample : ProjectedPlanet.Samples)
	{
		BoundaryCount += Sample.bIsBoundary ? 1 : 0;
	}
	OutDiagnostics.BoundaryFraction = ProjectedPlanet.Samples.Num() > 0
		? static_cast<double>(BoundaryCount) / static_cast<double>(ProjectedPlanet.Samples.Num())
		: 0.0;

	int32 BoundaryNoiseCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < ProjectedPlanet.Samples.Num(); ++SampleIndex)
	{
		const FSample& Sample = ProjectedPlanet.Samples[SampleIndex];
		if (!Sample.bIsBoundary || !ProjectedPlanet.SampleAdjacency.IsValidIndex(SampleIndex))
		{
			continue;
		}

		TArray<int32, TInlineAllocator<8>> OneRingPlateIds;
		OneRingPlateIds.Add(Sample.PlateId);
		for (const int32 NeighborIndex : ProjectedPlanet.SampleAdjacency[SampleIndex])
		{
			if (!ProjectedPlanet.Samples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = ProjectedPlanet.Samples[NeighborIndex].PlateId;
			if (!OneRingPlateIds.Contains(NeighborPlateId))
			{
				OneRingPlateIds.Add(NeighborPlateId);
			}
		}

		if (OneRingPlateIds.Num() > 3)
		{
			++BoundaryNoiseCount;
		}
	}
	OutDiagnostics.BoundaryNoiseSampleCount = BoundaryNoiseCount;
	OutDiagnostics.BoundaryNoiseFraction = ProjectedPlanet.Samples.Num() > 0
		? static_cast<double>(BoundaryNoiseCount) / static_cast<double>(ProjectedPlanet.Samples.Num())
		: 0.0;

	if (CurrentStep == 0)
	{
		int32 PlateMismatchCount = 0;
		double ContinentalErrorSum = 0.0;
		int32 InitialBoundaryCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < ProjectedPlanet.Samples.Num(); ++SampleIndex)
		{
			if (InitialPlateIds.IsValidIndex(SampleIndex) &&
				ProjectedPlanet.Samples[SampleIndex].PlateId != InitialPlateIds[SampleIndex])
			{
				++PlateMismatchCount;
			}
			if (InitialContinentalWeights.IsValidIndex(SampleIndex))
			{
				ContinentalErrorSum += FMath::Abs(
					static_cast<double>(ProjectedPlanet.Samples[SampleIndex].ContinentalWeight) -
					static_cast<double>(InitialContinentalWeights[SampleIndex]));
			}
			InitialBoundaryCount += InitialBoundaryFlags.IsValidIndex(SampleIndex) && InitialBoundaryFlags[SampleIndex] != 0 ? 1 : 0;
		}
		OutDiagnostics.ColdStartPlateMismatchFraction = ProjectedPlanet.Samples.Num() > 0
			? static_cast<double>(PlateMismatchCount) / static_cast<double>(ProjectedPlanet.Samples.Num())
			: 0.0;
		OutDiagnostics.ColdStartContinentalMeanAbsError = ProjectedPlanet.Samples.Num() > 0
			? ContinentalErrorSum / static_cast<double>(ProjectedPlanet.Samples.Num())
			: 0.0;
		const double InitialBoundaryFraction = ProjectedPlanet.Samples.Num() > 0
			? static_cast<double>(InitialBoundaryCount) / static_cast<double>(ProjectedPlanet.Samples.Num())
			: 0.0;
		OutDiagnostics.ColdStartBoundaryFractionDelta =
			FMath::Abs(OutDiagnostics.BoundaryFraction - InitialBoundaryFraction);
	}

	double LocalCoreDriftSumKm = 0.0;
	double KinematicDriftSumKm = 0.0;
	double ProjectedDriftSumKm = 0.0;
	double DriftMismatchSumKm = 0.0;
	for (const FTectonicSidecarPlate& Plate : Plates)
	{
		if (Plate.InitialContinentalCoreSampleCount < 16 ||
			Plate.InitialCoreCentroidLocal.IsNearlyZero())
		{
			continue;
		}

		const FVector3d InitialWorldCentroid = Plate.InitialCoreCentroidLocal.GetSafeNormal();
		const FVector3d ExpectedWorldCentroid =
			Plate.WorldFromLocal.RotateVector(Plate.InitialCoreCentroidLocal).GetSafeNormal();
		const FVector3d ProjectedWorldCentroid =
			ComputeWeightedCentroid(ProjectedPlanet.Samples, Plate.PlateId, true);
		if (ProjectedWorldCentroid.IsNearlyZero())
		{
			continue;
		}

		const FVector3d ProjectedLocalCentroid =
			Plate.WorldFromLocal.Inverse().RotateVector(ProjectedWorldCentroid).GetSafeNormal();
		const double LocalCoreDriftKm =
			AngularDistanceRad(Plate.InitialCoreCentroidLocal, ProjectedLocalCentroid) * Config.PlanetRadiusKm;
		const double KinematicDriftKm =
			AngularDistanceRad(InitialWorldCentroid, ExpectedWorldCentroid) * Config.PlanetRadiusKm;
		const double ProjectedDriftKm =
			AngularDistanceRad(InitialWorldCentroid, ProjectedWorldCentroid) * Config.PlanetRadiusKm;
		const double DriftMismatchKm =
			AngularDistanceRad(ExpectedWorldCentroid, ProjectedWorldCentroid) * Config.PlanetRadiusKm;

		LocalCoreDriftSumKm += LocalCoreDriftKm;
		KinematicDriftSumKm += KinematicDriftKm;
		ProjectedDriftSumKm += ProjectedDriftKm;
		DriftMismatchSumKm += DriftMismatchKm;
		OutDiagnostics.MaxProjectedLocalCoreDriftKm =
			FMath::Max(OutDiagnostics.MaxProjectedLocalCoreDriftKm, LocalCoreDriftKm);
		OutDiagnostics.MaxDriftMismatchKm =
			FMath::Max(OutDiagnostics.MaxDriftMismatchKm, DriftMismatchKm);
		++OutDiagnostics.DriftPlateCount;
	}

	if (OutDiagnostics.DriftPlateCount > 0)
	{
		const double PlateCount = static_cast<double>(OutDiagnostics.DriftPlateCount);
		OutDiagnostics.MeanProjectedLocalCoreDriftKm = LocalCoreDriftSumKm / PlateCount;
		OutDiagnostics.MeanKinematicDriftKm = KinematicDriftSumKm / PlateCount;
		OutDiagnostics.MeanProjectedDriftKm = ProjectedDriftSumKm / PlateCount;
		OutDiagnostics.MeanDriftMismatchKm = DriftMismatchSumKm / PlateCount;
	}

	TArray<uint8> Visited;
	Visited.Init(0, ProjectedPlanet.Samples.Num());
	TMap<int32, int32> TotalSamplesByPlate;
	TMap<int32, int32> LargestComponentByPlate;
	int32 TinyComponentSampleCount = 0;

	for (int32 SeedSampleIndex = 0; SeedSampleIndex < ProjectedPlanet.Samples.Num(); ++SeedSampleIndex)
	{
		if (Visited[SeedSampleIndex] != 0)
		{
			continue;
		}

		const int32 PlateId = ProjectedPlanet.Samples[SeedSampleIndex].PlateId;
		TArray<int32> Stack;
		Stack.Add(SeedSampleIndex);
		Visited[SeedSampleIndex] = 1;
		int32 ComponentSize = 0;

		while (!Stack.IsEmpty())
		{
			const int32 SampleIndex = Stack.Pop(EAllowShrinking::No);
			++ComponentSize;
			if (!ProjectedPlanet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			for (const int32 NeighborIndex : ProjectedPlanet.SampleAdjacency[SampleIndex])
			{
				if (!ProjectedPlanet.Samples.IsValidIndex(NeighborIndex) ||
					Visited[NeighborIndex] != 0 ||
					ProjectedPlanet.Samples[NeighborIndex].PlateId != PlateId)
				{
					continue;
				}

				Visited[NeighborIndex] = 1;
				Stack.Add(NeighborIndex);
			}
		}

		++OutDiagnostics.RegionComponentCount;
		TotalSamplesByPlate.FindOrAdd(PlateId) += ComponentSize;
		int32& LargestComponent = LargestComponentByPlate.FindOrAdd(PlateId);
		LargestComponent = FMath::Max(LargestComponent, ComponentSize);
		if (ComponentSize <= SidecarTinyComponentMaxSize)
		{
			TinyComponentSampleCount += ComponentSize;
		}
	}

	double DominantFractionSum = 0.0;
	int32 LargePlateCount = 0;
	int32 FragmentedSampleCount = 0;
	for (const TPair<int32, int32>& PlateEntry : TotalSamplesByPlate)
	{
		const int32 TotalSamples = PlateEntry.Value;
		const int32 LargestComponent = LargestComponentByPlate.FindRef(PlateEntry.Key);
		FragmentedSampleCount += TotalSamples - LargestComponent;
		if (TotalSamples >= SidecarLargePlateMinProjectedSamples)
		{
			DominantFractionSum += static_cast<double>(LargestComponent) / static_cast<double>(TotalSamples);
			++LargePlateCount;
		}
		if (LargestComponent < TotalSamples)
		{
			++OutDiagnostics.MultiComponentPlateCount;
		}
	}

	OutDiagnostics.LargePlateMeanDominantComponentFraction = LargePlateCount > 0
		? DominantFractionSum / static_cast<double>(LargePlateCount)
		: 1.0;
	OutDiagnostics.TinyComponentFraction = ProjectedPlanet.Samples.Num() > 0
		? static_cast<double>(TinyComponentSampleCount) / static_cast<double>(ProjectedPlanet.Samples.Num())
		: 0.0;
	OutDiagnostics.OwnerFragmentedSampleCount = FragmentedSampleCount;
	OutDiagnostics.OwnerFragmentationFraction = ProjectedPlanet.Samples.Num() > 0
		? static_cast<double>(FragmentedSampleCount) / static_cast<double>(ProjectedPlanet.Samples.Num())
		: 0.0;
	OutDiagnostics.CarriedContinentalMassRelativeError = OutDiagnostics.MaterialMassRelativeError;
	OutDiagnostics.MaterialFabricatedFraction = OutDiagnostics.FabricatedMaterialFraction;
	OutDiagnostics.MaterialOverlapFraction = OutDiagnostics.OverlapFraction;
}

void FTectonicPlanetSidecar::ResetLastProjectionDebug(const int32 SampleCount) const
{
	LastMaterialSourcePlateIds.Init(INDEX_NONE, SampleCount);
	LastMaterialClassifications.Init(
		static_cast<uint8>(ETectonicSidecarMaterialClassification::None),
		SampleCount);
	LastMaterialOwnerMismatchFlags.Init(0, SampleCount);
	LastMaterialOverlapCounts.Init(0, SampleCount);
	LastDivergentBoundaryFlags.Init(0, SampleCount);
}
