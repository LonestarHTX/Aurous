#include "Misc/AutomationTest.h"

#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Interface/Core/RealtimeMeshDataStream.h"
#include "Math/RandomStream.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "ShewchukPredicates.h"
#include "TectonicPlanet.h"
#include "TectonicPlanetActor.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr int32 TestSampleCount = 500000;
	constexpr int32 TestPlateCount = 7;
	constexpr int32 TestSeed = 42;

	struct FCachedPlanetState
	{
		FTectonicPlanet Planet;
		double InitializeSeconds = 0.0;
		bool bInitialized = false;
	};

	FCachedPlanetState& GetCachedPlanetState()
	{
		static FCachedPlanetState State;
		if (!State.bInitialized)
		{
			const double StartSeconds = FPlatformTime::Seconds();
			State.Planet.Initialize(TestSampleCount);
			State.InitializeSeconds = FPlatformTime::Seconds() - StartSeconds;
			State.bInitialized = true;

			UE_LOG(LogTemp, Log, TEXT("Initialize(%d) wall time: %.3f s"), TestSampleCount, State.InitializeSeconds);
		}

		return State;
	}

	const FTectonicPlanet& GetCachedPlanet()
	{
		return GetCachedPlanetState().Planet;
	}

	FTectonicPlanet MakePlanetCopy()
	{
		return GetCachedPlanet();
	}

	FVector MakeRandomUnitVector(FRandomStream& Random)
	{
		const double Z = Random.FRandRange(-1.0f, 1.0f);
		const double Phi = Random.FRandRange(0.0f, 2.0f * PI);
		const double RadiusXY = FMath::Sqrt(FMath::Max(0.0, 1.0 - Z * Z));
		return FVector(RadiusXY * FMath::Cos(Phi), RadiusXY * FMath::Sin(Phi), Z);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFibonacciSphereTest, "Aurous.TectonicPlanet.FibonacciSphere",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFibonacciSphereTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanet& Planet = GetCachedPlanet();
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();

	TestEqual(TEXT("Sample count"), Samples.Num(), TestSampleCount);

	double MinRadius = TNumericLimits<double>::Max();
	double MaxRadius = 0.0;
	double MaxRadiusError = 0.0;
	int32 OffSphereCount = 0;

	for (const FCanonicalSample& Sample : Samples)
	{
		const double Magnitude = Sample.Position.Size();
		MinRadius = FMath::Min(MinRadius, Magnitude);
		MaxRadius = FMath::Max(MaxRadius, Magnitude);
		MaxRadiusError = FMath::Max(MaxRadiusError, FMath::Abs(Magnitude - 1.0));
		OffSphereCount += FMath::IsNearlyEqual(Magnitude, 1.0, 1e-10) ? 0 : 1;
	}

	int32 CoincidentSanityFailures = 0;
	for (int32 I = 0; I < FMath::Min(1000, Samples.Num()); ++I)
	{
		for (int32 J = I + 1; J < FMath::Min(I + 10, Samples.Num()); ++J)
		{
			const double Distance = FVector::Dist(Samples[I].Position, Samples[J].Position);
			CoincidentSanityFailures += (Distance <= 1e-8) ? 1 : 0;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Fibonacci metrics (500k): min|p|=%.15f max|p|=%.15f max|r-1|=%.3e offSphere=%d coincidenceSanityFailures=%d"),
		MinRadius, MaxRadius, MaxRadiusError, OffSphereCount, CoincidentSanityFailures);

	TestEqual(TEXT("Off-sphere sample count"), OffSphereCount, 0);
	TestEqual(TEXT("Coincident sanity failures"), CoincidentSanityFailures, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDelaunayTriangulationTest, "Aurous.TectonicPlanet.DelaunayTriangulation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FDelaunayTriangulationTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanet& Planet = GetCachedPlanet();
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();

	const int32 ExpectedTriangles = 2 * Samples.Num() - 4;
	TestEqual(TEXT("Triangle count matches Euler formula"), Triangles.Num(), ExpectedTriangles);
	TestEqual(TEXT("Triangle count at 500k"), Triangles.Num(), 999996);

	TArray<uint8> ReferencedVertices;
	ReferencedVertices.Init(0, Samples.Num());

	int32 InvalidIndexCount = 0;
	int32 DegenerateIndexCount = 0;
	int32 SmallAreaCount = 0;
	double MinArea2 = TNumericLimits<double>::Max();
	double MaxArea2 = 0.0;

	for (const FDelaunayTriangle& Triangle : Triangles)
	{
		const int32 AIndex = Triangle.V[0];
		const int32 BIndex = Triangle.V[1];
		const int32 CIndex = Triangle.V[2];

		if (!Samples.IsValidIndex(AIndex) || !Samples.IsValidIndex(BIndex) || !Samples.IsValidIndex(CIndex))
		{
			++InvalidIndexCount;
			continue;
		}

		ReferencedVertices[AIndex] = 1;
		ReferencedVertices[BIndex] = 1;
		ReferencedVertices[CIndex] = 1;

		if (AIndex == BIndex || BIndex == CIndex || AIndex == CIndex)
		{
			++DegenerateIndexCount;
			continue;
		}

		const FVector& A = Samples[AIndex].Position;
		const FVector& B = Samples[BIndex].Position;
		const FVector& C = Samples[CIndex].Position;
		const double Area2 = FVector::CrossProduct(B - A, C - A).Size();
		MinArea2 = FMath::Min(MinArea2, Area2);
		MaxArea2 = FMath::Max(MaxArea2, Area2);
		SmallAreaCount += (Area2 < static_cast<double>(SMALL_NUMBER)) ? 1 : 0;
	}

	int32 MissingVertexCount = 0;
	for (const uint8 bReferenced : ReferencedVertices)
	{
		MissingVertexCount += (bReferenced == 0) ? 1 : 0;
	}

	UE_LOG(LogTemp, Log, TEXT("Delaunay metrics (500k): Triangles=%d Expected=%d InvalidIdx=%d DegenerateIdx=%d SmallArea=%d Min|cross|=%.6e Max|cross|=%.6e MissingVerts=%d"),
		Triangles.Num(),
		ExpectedTriangles,
		InvalidIndexCount,
		DegenerateIndexCount,
		SmallAreaCount,
		(MinArea2 == TNumericLimits<double>::Max()) ? 0.0 : MinArea2,
		MaxArea2,
		MissingVertexCount);

	TestEqual(TEXT("Invalid triangle index count"), InvalidIndexCount, 0);
	TestEqual(TEXT("Degenerate index triangle count"), DegenerateIndexCount, 0);
	TestEqual(TEXT("Small-area triangle count"), SmallAreaCount, 0);
	TestEqual(TEXT("Missing referenced vertices"), MissingVertexCount, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAdjacencyGraphTest, "Aurous.TectonicPlanet.AdjacencyGraph",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FAdjacencyGraphTest::RunTest(const FString& Parameters)
{
	const FTectonicPlanet& Planet = GetCachedPlanet();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();

	TestEqual(TEXT("Adjacency array size"), Adjacency.Num(), Samples.Num());
	TestEqual(TEXT("Adjacency array size at 500k"), Adjacency.Num(), TestSampleCount);

	int32 Val5 = 0;
	int32 Val6 = 0;
	int32 Val7 = 0;
	int32 ValOther = 0;
	int32 LowValenceCount = 0;
	int32 InvalidNeighborRefs = 0;
	int32 AsymmetricRefs = 0;

	for (int32 VertexIndex = 0; VertexIndex < Adjacency.Num(); ++VertexIndex)
	{
		const TArray<int32>& Neighbors = Adjacency[VertexIndex];
		const int32 Valence = Neighbors.Num();

		LowValenceCount += (Valence < 3) ? 1 : 0;

		if (Valence == 5)
		{
			++Val5;
		}
		else if (Valence == 6)
		{
			++Val6;
		}
		else if (Valence == 7)
		{
			++Val7;
		}
		else
		{
			++ValOther;
		}

		for (const int32 NeighborIndex : Neighbors)
		{
			if (!Adjacency.IsValidIndex(NeighborIndex))
			{
				++InvalidNeighborRefs;
				continue;
			}

			if (!Adjacency[NeighborIndex].Contains(VertexIndex))
			{
				++AsymmetricRefs;
			}
		}
	}

	const int32 TotalVertices = FMath::Max(1, Adjacency.Num());
	const double PctVal5 = static_cast<double>(Val5) / static_cast<double>(TotalVertices);
	const double PctVal6 = static_cast<double>(Val6) / static_cast<double>(TotalVertices);
	const double PctVal7 = static_cast<double>(Val7) / static_cast<double>(TotalVertices);
	const double PctOther = static_cast<double>(ValOther) / static_cast<double>(TotalVertices);
	const double PctNormal = static_cast<double>(Val5 + Val6 + Val7) / static_cast<double>(TotalVertices);

	UE_LOG(LogTemp, Log, TEXT("Adjacency valence distribution (500k): V5=%d (%.2f%%) V6=%d (%.2f%%) V7=%d (%.2f%%) Other=%d (%.2f%%)"),
		Val5, PctVal5 * 100.0,
		Val6, PctVal6 * 100.0,
		Val7, PctVal7 * 100.0,
		ValOther, PctOther * 100.0);
	UE_LOG(LogTemp, Log, TEXT("Adjacency integrity (500k): LowValence=%d InvalidRefs=%d AsymmetricRefs=%d NormalValencePct=%.2f%%"),
		LowValenceCount, InvalidNeighborRefs, AsymmetricRefs, PctNormal * 100.0);

	TestEqual(TEXT("Vertices with valence < 3"), LowValenceCount, 0);
	TestEqual(TEXT("Invalid neighbor references"), InvalidNeighborRefs, 0);
	TestEqual(TEXT("Asymmetric neighbor references"), AsymmetricRefs, 0);
	TestTrue(TEXT("Majority valence 5-7"), PctNormal > 0.95);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlateInitTest, "Aurous.TectonicPlanet.PlateInitialization",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPlateInitTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FPlate>& Plates = Planet.GetPlates();

	TestEqual(TEXT("Plate count"), Plates.Num(), TestPlateCount);

	int32 InvalidPlateAssignments = 0;
	bool bHasContinental = false;
	bool bHasOceanic = false;
	for (const FCanonicalSample& Sample : Samples)
	{
		InvalidPlateAssignments += (Sample.PlateId < 0 || Sample.PlateId >= TestPlateCount) ? 1 : 0;
		bHasContinental |= (Sample.CrustType == ECrustType::Continental);
		bHasOceanic |= (Sample.CrustType == ECrustType::Oceanic);
	}

	int32 TotalAssigned = 0;
	int32 EmptyPlates = 0;
	int32 InvalidAxes = 0;
	int32 InvalidAngularSpeeds = 0;
	for (const FPlate& Plate : Plates)
	{
		EmptyPlates += (Plate.SampleIndices.Num() == 0) ? 1 : 0;
		TotalAssigned += Plate.SampleIndices.Num();
		InvalidAxes += !FMath::IsNearlyEqual(Plate.RotationAxis.Size(), 1.0, 1e-8) ? 1 : 0;
		InvalidAngularSpeeds += (Plate.AngularSpeed < (0.5f * 3.14e-2f) || Plate.AngularSpeed > (1.5f * 3.14e-2f)) ? 1 : 0;
	}

	UE_LOG(LogTemp, Log, TEXT("Plate init metrics (500k): Plates=%d TotalAssigned=%d EmptyPlates=%d InvalidAssignments=%d InvalidAxes=%d InvalidSpeeds=%d"),
		Plates.Num(), TotalAssigned, EmptyPlates, InvalidPlateAssignments, InvalidAxes, InvalidAngularSpeeds);

	TestEqual(TEXT("Invalid plate assignments"), InvalidPlateAssignments, 0);
	TestEqual(TEXT("All samples assigned to plates"), TotalAssigned, Samples.Num());
	TestEqual(TEXT("Empty plate count"), EmptyPlates, 0);
	TestEqual(TEXT("Invalid rotation axis count"), InvalidAxes, 0);
	TestEqual(TEXT("Invalid angular speed count"), InvalidAngularSpeeds, 0);
	TestTrue(TEXT("Has continental crust"), bHasContinental);
	TestTrue(TEXT("Has oceanic crust"), bHasOceanic);

	FTectonicPlanet Planet2 = MakePlanetCopy();
	Planet2.InitializePlates(TestPlateCount, TestSeed);

	int32 DeterminismMismatches = 0;
	const TArray<FCanonicalSample>& Samples2 = Planet2.GetSamples();
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		DeterminismMismatches += (Samples2[SampleIndex].PlateId != Samples[SampleIndex].PlateId) ? 1 : 0;
	}

	UE_LOG(LogTemp, Log, TEXT("Plate determinism metrics (500k): mismatches=%d"), DeterminismMismatches);
	TestEqual(TEXT("Determinism mismatches"), DeterminismMismatches, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTriangleClassificationTest, "Aurous.TectonicPlanet.TriangleClassification",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FTriangleClassificationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();
	const TArray<FPlate>& Plates = Planet.GetPlates();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();

	int32 TotalInterior = 0;
	int32 InvalidInteriorTriangles = 0;
	int32 InteriorClassificationMismatches = 0;

	TArray<uint8> BoundaryTriangleMask;
	BoundaryTriangleMask.Init(0, Triangles.Num());
	int32 UniqueBoundaryTriangleCount = 0;
	int32 InvalidBoundaryTriangleRefs = 0;

	for (const FPlate& Plate : Plates)
	{
		TotalInterior += Plate.InteriorTriangles.Num();

		for (const int32 TriangleIndex : Plate.InteriorTriangles)
		{
			if (!Triangles.IsValidIndex(TriangleIndex))
			{
				++InvalidInteriorTriangles;
				continue;
			}

			const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
			const bool bInteriorMatches =
				Samples[Triangle.V[0]].PlateId == Plate.Id &&
				Samples[Triangle.V[1]].PlateId == Plate.Id &&
				Samples[Triangle.V[2]].PlateId == Plate.Id;
			InteriorClassificationMismatches += bInteriorMatches ? 0 : 1;
		}

		for (const int32 TriangleIndex : Plate.BoundaryTriangles)
		{
			if (!Triangles.IsValidIndex(TriangleIndex))
			{
				++InvalidBoundaryTriangleRefs;
				continue;
			}

			if (BoundaryTriangleMask[TriangleIndex] == 0)
			{
				BoundaryTriangleMask[TriangleIndex] = 1;
				++UniqueBoundaryTriangleCount;
			}
		}
	}

	int32 BoundaryClassificationMismatches = 0;
	for (int32 TriangleIndex = 0; TriangleIndex < BoundaryTriangleMask.Num(); ++TriangleIndex)
	{
		if (BoundaryTriangleMask[TriangleIndex] == 0)
		{
			continue;
		}

		const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
		const int32 P0 = Samples[Triangle.V[0]].PlateId;
		const int32 P1 = Samples[Triangle.V[1]].PlateId;
		const int32 P2 = Samples[Triangle.V[2]].PlateId;
		const bool bSpansPlates = (P0 != P1) || (P1 != P2) || (P0 != P2);
		BoundaryClassificationMismatches += bSpansPlates ? 0 : 1;
	}

	int32 BoundaryFlagMismatches = 0;
	int32 BoundarySampleCount = 0;
	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		bool bHasDifferentNeighbor = false;
		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (Samples[NeighborIndex].PlateId != Samples[SampleIndex].PlateId)
			{
				bHasDifferentNeighbor = true;
				break;
			}
		}

		BoundaryFlagMismatches += (Samples[SampleIndex].bIsBoundary == bHasDifferentNeighbor) ? 0 : 1;
		BoundarySampleCount += Samples[SampleIndex].bIsBoundary ? 1 : 0;
	}

	const int32 TotalTriangles = Triangles.Num();
	const double InteriorTriangleFraction = (TotalTriangles > 0) ? static_cast<double>(TotalInterior) / static_cast<double>(TotalTriangles) : 0.0;
	const double BoundarySampleFraction = (Samples.Num() > 0) ? static_cast<double>(BoundarySampleCount) / static_cast<double>(Samples.Num()) : 0.0;

	UE_LOG(LogTemp, Log, TEXT("Triangle classification metrics (500k): Interior=%d/%d (%.2f%%), BoundaryUnique=%d, BoundarySamples=%d/%d (%.2f%%)"),
		TotalInterior,
		TotalTriangles,
		InteriorTriangleFraction * 100.0,
		UniqueBoundaryTriangleCount,
		BoundarySampleCount,
		Samples.Num(),
		BoundarySampleFraction * 100.0);
	UE_LOG(LogTemp, Log, TEXT("Triangle classification integrity (500k): InvalidInteriorRefs=%d InvalidBoundaryRefs=%d InteriorMismatch=%d BoundaryMismatch=%d BoundaryFlagMismatch=%d"),
		InvalidInteriorTriangles,
		InvalidBoundaryTriangleRefs,
		InteriorClassificationMismatches,
		BoundaryClassificationMismatches,
		BoundaryFlagMismatches);

	TestEqual(TEXT("Interior + boundary = total triangles"), TotalInterior + UniqueBoundaryTriangleCount, TotalTriangles);
	TestEqual(TEXT("Invalid interior triangle refs"), InvalidInteriorTriangles, 0);
	TestEqual(TEXT("Invalid boundary triangle refs"), InvalidBoundaryTriangleRefs, 0);
	TestEqual(TEXT("Interior classification mismatches"), InteriorClassificationMismatches, 0);
	TestEqual(TEXT("Boundary classification mismatches"), BoundaryClassificationMismatches, 0);
	TestEqual(TEXT("Boundary flag mismatches"), BoundaryFlagMismatches, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundaryClassificationTest, "Aurous.TectonicPlanet.BoundaryClassification",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FBoundaryClassificationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();

	int32 BoundarySampleCount = 0;
	int32 MissingBoundaryNormal = 0;
	int32 NonTangentBoundaryNormal = 0;
	int32 CrossDirectionMismatch = 0;
	int32 InteriorDirectionMismatch = 0;
	int32 BoundaryTypeNoneCount = 0;
	int32 BoundaryTypeMismatch = 0;
	int32 ConvergentCount = 0;
	int32 DivergentCount = 0;
	int32 TransformCount = 0;

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = Samples[SampleIndex];
		if (!Sample.bIsBoundary)
		{
			continue;
		}

		++BoundarySampleCount;
		const FVector Position = Sample.Position.GetSafeNormal();
		if (Sample.BoundaryNormal.IsNearlyZero())
		{
			++MissingBoundaryNormal;
			continue;
		}

		const FVector BoundaryNormal = Sample.BoundaryNormal.GetSafeNormal();
		const double TangencyAbsDot = FMath::Abs(FVector::DotProduct(BoundaryNormal, Position));
		NonTangentBoundaryNormal += (TangencyAbsDot > 1e-4) ? 1 : 0;

		FVector MeanCrossDirection = FVector::ZeroVector;
		FVector MeanInteriorDirection = FVector::ZeroVector;
		int32 CrossPlateIds[16];
		int32 CrossPlateCounts[16];
		int32 NumCrossPlates = 0;
		for (int32 SlotIndex = 0; SlotIndex < 16; ++SlotIndex)
		{
			CrossPlateIds[SlotIndex] = INDEX_NONE;
			CrossPlateCounts[SlotIndex] = 0;
		}

		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!Samples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const FCanonicalSample& Neighbor = Samples[NeighborIndex];
			const FVector Direction = (Neighbor.Position - Sample.Position).GetSafeNormal();
			if (Direction.IsNearlyZero())
			{
				continue;
			}

			if (Neighbor.PlateId != Sample.PlateId)
			{
				MeanCrossDirection += Direction;
				int32 ExistingSlot = INDEX_NONE;
				for (int32 SlotIndex = 0; SlotIndex < NumCrossPlates; ++SlotIndex)
				{
					if (CrossPlateIds[SlotIndex] == Neighbor.PlateId)
					{
						ExistingSlot = SlotIndex;
						break;
					}
				}

				if (ExistingSlot != INDEX_NONE)
				{
					++CrossPlateCounts[ExistingSlot];
				}
				else if (NumCrossPlates < 16)
				{
					CrossPlateIds[NumCrossPlates] = Neighbor.PlateId;
					CrossPlateCounts[NumCrossPlates] = 1;
					++NumCrossPlates;
				}
			}
			else
			{
				MeanInteriorDirection += Direction;
			}
		}

		const FVector CrossTangent = MeanCrossDirection - FVector::DotProduct(MeanCrossDirection, Position) * Position;
		if (!CrossTangent.IsNearlyZero())
		{
			const double Alignment = FVector::DotProduct(BoundaryNormal, CrossTangent.GetSafeNormal());
			CrossDirectionMismatch += (Alignment < 0.5) ? 1 : 0;
		}

		const FVector InteriorTangent = MeanInteriorDirection - FVector::DotProduct(MeanInteriorDirection, Position) * Position;
		if (!InteriorTangent.IsNearlyZero())
		{
			const double InteriorAlignment = FVector::DotProduct(BoundaryNormal, InteriorTangent.GetSafeNormal());
			InteriorDirectionMismatch += (InteriorAlignment > 0.0) ? 1 : 0;
		}

		switch (Sample.BoundaryType)
		{
		case EBoundaryType::Convergent:
			++ConvergentCount;
			break;
		case EBoundaryType::Divergent:
			++DivergentCount;
			break;
		case EBoundaryType::Transform:
			++TransformCount;
			break;
		case EBoundaryType::None:
		default:
			++BoundaryTypeNoneCount;
			break;
		}

		if (NumCrossPlates > 0)
		{
			int32 DominantCrossPlate = CrossPlateIds[0];
			int32 DominantCount = CrossPlateCounts[0];
			for (int32 SlotIndex = 1; SlotIndex < NumCrossPlates; ++SlotIndex)
			{
				if (CrossPlateCounts[SlotIndex] > DominantCount ||
					(CrossPlateCounts[SlotIndex] == DominantCount && CrossPlateIds[SlotIndex] < DominantCrossPlate))
				{
					DominantCrossPlate = CrossPlateIds[SlotIndex];
					DominantCount = CrossPlateCounts[SlotIndex];
				}
			}

			const FVector RelativeVelocity = Planet.ComputeSurfaceVelocity(Sample.PlateId, Position) - Planet.ComputeSurfaceVelocity(DominantCrossPlate, Position);
			const double RelativeSpeed = RelativeVelocity.Size();
			const double NormalComponent = FVector::DotProduct(RelativeVelocity, BoundaryNormal);
			EBoundaryType ExpectedType = EBoundaryType::Transform;
			if (RelativeSpeed > 1e-12)
			{
				if (FMath::Abs(NormalComponent) < 0.3 * RelativeSpeed)
				{
					ExpectedType = EBoundaryType::Transform;
				}
				else
				{
					ExpectedType = (NormalComponent < 0.0) ? EBoundaryType::Convergent : EBoundaryType::Divergent;
				}
			}

			BoundaryTypeMismatch += (Sample.BoundaryType == ExpectedType) ? 0 : 1;
		}
	}

	Planet.StepSimulation();
	const TArray<FCanonicalSample>& ReconciledSamples = Planet.GetSamples();
	int32 GapSampleCount = 0;
	int32 MissingFlankingA = 0;
	int32 InvalidFlankingPair = 0;
	for (int32 SampleIndex = 0; SampleIndex < ReconciledSamples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = ReconciledSamples[SampleIndex];
		if (!Sample.bGapDetected)
		{
			continue;
		}

		++GapSampleCount;
		if (Sample.FlankingPlateIdA < 0 || Sample.FlankingPlateIdA >= TestPlateCount)
		{
			++MissingFlankingA;
			continue;
		}

		int32 UniqueNeighborPlateCount = 0;
		int32 UniqueNeighborPlates[16];
		for (int32 SlotIndex = 0; SlotIndex < 16; ++SlotIndex)
		{
			UniqueNeighborPlates[SlotIndex] = INDEX_NONE;
		}
		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!ReconciledSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = ReconciledSamples[NeighborIndex].PlateId;
			if (NeighborPlateId < 0 || NeighborPlateId >= TestPlateCount)
			{
				continue;
			}

			bool bAlreadyAdded = false;
			for (int32 SlotIndex = 0; SlotIndex < UniqueNeighborPlateCount; ++SlotIndex)
			{
				if (UniqueNeighborPlates[SlotIndex] == NeighborPlateId)
				{
					bAlreadyAdded = true;
					break;
				}
			}
			if (!bAlreadyAdded && UniqueNeighborPlateCount < 16)
			{
				UniqueNeighborPlates[UniqueNeighborPlateCount++] = NeighborPlateId;
			}
		}

		if (UniqueNeighborPlateCount >= 2)
		{
			const bool bValidPair =
				Sample.FlankingPlateIdB >= 0 &&
				Sample.FlankingPlateIdB < TestPlateCount &&
				Sample.FlankingPlateIdA != Sample.FlankingPlateIdB;
			InvalidFlankingPair += bValidPair ? 0 : 1;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Boundary classification metrics (500k): Boundary=%d MissingNormal=%d NonTangent=%d CrossMismatch=%d InteriorMismatch=%d TypeNone=%d TypeMismatch=%d Cvg=%d Div=%d Trn=%d"),
		BoundarySampleCount,
		MissingBoundaryNormal,
		NonTangentBoundaryNormal,
		CrossDirectionMismatch,
		InteriorDirectionMismatch,
		BoundaryTypeNoneCount,
		BoundaryTypeMismatch,
		ConvergentCount,
		DivergentCount,
		TransformCount);
	UE_LOG(LogTemp, Log, TEXT("Gap flanking metrics (500k): Gap=%d MissingA=%d InvalidPair=%d"),
		GapSampleCount,
		MissingFlankingA,
		InvalidFlankingPair);

	TestTrue(TEXT("Boundary samples exist"), BoundarySampleCount > 0);
	TestEqual(TEXT("Missing boundary normals"), MissingBoundaryNormal, 0);
	TestEqual(TEXT("Boundary normals tangent to surface"), NonTangentBoundaryNormal, 0);
	TestTrue(TEXT("Boundary normals align with cross-plate direction"), CrossDirectionMismatch <= FMath::Max(1, BoundarySampleCount / 100));
	TestTrue(TEXT("Boundary normals point away from plate interior"), InteriorDirectionMismatch <= FMath::Max(1, BoundarySampleCount / 100));
	TestEqual(TEXT("Boundary type none count for boundary samples"), BoundaryTypeNoneCount, 0);
	TestEqual(TEXT("Boundary type matches velocity rule"), BoundaryTypeMismatch, 0);
	TestTrue(TEXT("At least one boundary type populated"), (ConvergentCount + DivergentCount + TransformCount) > 0);
	TestTrue(TEXT("Gap samples exist after reconcile"), GapSampleCount > 0);
	TestEqual(TEXT("Gap samples have primary flanking plate"), MissingFlankingA, 0);
	TestEqual(TEXT("Gap samples with 2+ neighboring plates have valid flanking pair"), InvalidFlankingPair, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoundaryClassificationKnownPairTest, "Aurous.TectonicPlanet.BoundaryClassificationKnownPair",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FBoundaryClassificationKnownPairTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& InitialSamples = Planet.GetSamples();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();
	int32 TargetSampleIndex = INDEX_NONE;
	int32 PlateA = INDEX_NONE;
	int32 PlateB = INDEX_NONE;
	for (int32 SampleIndex = 0; SampleIndex < InitialSamples.Num(); ++SampleIndex)
	{
		const FCanonicalSample& Sample = InitialSamples[SampleIndex];
		if (!Sample.bIsBoundary || Sample.PlateId < 0 || Sample.PlateId >= TestPlateCount)
		{
			continue;
		}

		int32 UniqueCrossPlates[8];
		int32 NumUniqueCrossPlates = 0;
		for (int32 SlotIndex = 0; SlotIndex < 8; ++SlotIndex)
		{
			UniqueCrossPlates[SlotIndex] = INDEX_NONE;
		}

		for (const int32 NeighborIndex : Adjacency[SampleIndex])
		{
			if (!InitialSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}

			const int32 NeighborPlateId = InitialSamples[NeighborIndex].PlateId;
			if (NeighborPlateId < 0 || NeighborPlateId >= TestPlateCount || NeighborPlateId == Sample.PlateId)
			{
				continue;
			}

			bool bAlreadyAdded = false;
			for (int32 SlotIndex = 0; SlotIndex < NumUniqueCrossPlates; ++SlotIndex)
			{
				if (UniqueCrossPlates[SlotIndex] == NeighborPlateId)
				{
					bAlreadyAdded = true;
					break;
				}
			}
			if (!bAlreadyAdded && NumUniqueCrossPlates < 8)
			{
				UniqueCrossPlates[NumUniqueCrossPlates++] = NeighborPlateId;
			}
		}

		if (NumUniqueCrossPlates == 1)
		{
			TargetSampleIndex = SampleIndex;
			PlateA = Sample.PlateId;
			PlateB = UniqueCrossPlates[0];
			break;
		}
	}

	TestTrue(TEXT("Found boundary sample with single opposing plate"), TargetSampleIndex != INDEX_NONE);
	if (TargetSampleIndex == INDEX_NONE)
	{
		return false;
	}

	const FVector P = InitialSamples[TargetSampleIndex].Position.GetSafeNormal();
	FVector N = InitialSamples[TargetSampleIndex].BoundaryNormal.GetSafeNormal();
	if (N.IsNearlyZero())
	{
		for (const int32 NeighborIndex : Adjacency[TargetSampleIndex])
		{
			if (!InitialSamples.IsValidIndex(NeighborIndex))
			{
				continue;
			}
			if (InitialSamples[NeighborIndex].PlateId != PlateA)
			{
				N += (InitialSamples[NeighborIndex].Position - InitialSamples[TargetSampleIndex].Position).GetSafeNormal();
			}
		}
		N = (N - FVector::DotProduct(N, P) * P).GetSafeNormal();
	}
	TestFalse(TEXT("Boundary normal valid"), N.IsNearlyZero());
	if (N.IsNearlyZero())
	{
		return false;
	}

	FVector Axis = FVector::CrossProduct(P, N).GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		Axis = FVector::CrossProduct(P, FVector::UpVector).GetSafeNormal();
	}
	TestFalse(TEXT("Constructed rotation axis valid"), Axis.IsNearlyZero());
	if (Axis.IsNearlyZero())
	{
		return false;
	}

	TArray<FPlate>& MutablePlates = Planet.GetPlates();
	for (FPlate& Plate : MutablePlates)
	{
		Plate.AngularSpeed = 0.0f;
	}
	MutablePlates[PlateA].RotationAxis = Axis;
	MutablePlates[PlateA].AngularSpeed = 0.05f;
	MutablePlates[PlateB].RotationAxis = -Axis;
	MutablePlates[PlateB].AngularSpeed = 0.05f;

	Planet.ClassifyTriangles();
	const FCanonicalSample& UpdatedSample = Planet.GetSamples()[TargetSampleIndex];
	const FVector RelativeVelocity = Planet.ComputeSurfaceVelocity(PlateA, P) - Planet.ComputeSurfaceVelocity(PlateB, P);
	const double RelativeAlongBoundary = FVector::DotProduct(RelativeVelocity, UpdatedSample.BoundaryNormal.GetSafeNormal());

	UE_LOG(LogTemp, Log, TEXT("Boundary known-pair metrics (500k): sample=%d plateA=%d plateB=%d relAlongNormal=%.6e type=%d"),
		TargetSampleIndex,
		PlateA,
		PlateB,
		RelativeAlongBoundary,
		static_cast<int32>(UpdatedSample.BoundaryType));

	TestTrue(TEXT("Relative velocity points along boundary normal for separating pair"), RelativeAlongBoundary > 0.0);
	TestEqual(TEXT("Known separating pair classified as divergent"), UpdatedSample.BoundaryType, EBoundaryType::Divergent);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCarriedWorkspaceTest, "Aurous.TectonicPlanet.CarriedWorkspace",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCarriedWorkspaceTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FPlate>& Plates = Planet.GetPlates();

	int32 TotalCarried = 0;
	int32 CountMismatches = 0;
	int32 InvalidCarriedIndices = 0;
	int32 IndexOrderMismatches = 0;
	int32 FieldMismatches = 0;

	for (const FPlate& Plate : Plates)
	{
		CountMismatches += (Plate.CarriedSamples.Num() == Plate.SampleIndices.Num()) ? 0 : 1;
		TotalCarried += Plate.CarriedSamples.Num();

		const int32 NumToCheck = FMath::Min(Plate.CarriedSamples.Num(), Plate.SampleIndices.Num());
		for (int32 LocalIndex = 0; LocalIndex < NumToCheck; ++LocalIndex)
		{
			const FCarriedSampleData& Carried = Plate.CarriedSamples[LocalIndex];
			const int32 CanonicalIndex = Carried.CanonicalSampleIndex;
			if (!Samples.IsValidIndex(CanonicalIndex))
			{
				++InvalidCarriedIndices;
				continue;
			}

			IndexOrderMismatches += (Plate.SampleIndices[LocalIndex] == CanonicalIndex) ? 0 : 1;

			const FCanonicalSample& Canonical = Samples[CanonicalIndex];
			const bool bMatches =
				FMath::IsNearlyEqual(Carried.Elevation, Canonical.Elevation) &&
				FMath::IsNearlyEqual(Carried.Thickness, Canonical.Thickness) &&
				FMath::IsNearlyEqual(Carried.Age, Canonical.Age) &&
				Carried.RidgeDirection.Equals(Canonical.RidgeDirection, 1e-6) &&
				Carried.FoldDirection.Equals(Canonical.FoldDirection, 1e-6) &&
				Carried.OrogenyType == Canonical.OrogenyType &&
				FMath::IsNearlyEqual(Carried.OrogenyAge, Canonical.OrogenyAge) &&
				Carried.CrustType == Canonical.CrustType;
			FieldMismatches += bMatches ? 0 : 1;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Carried workspace metrics (500k): TotalCarried=%d CountMismatch=%d InvalidIndices=%d IndexOrderMismatch=%d FieldMismatch=%d"),
		TotalCarried, CountMismatches, InvalidCarriedIndices, IndexOrderMismatches, FieldMismatches);

	TestEqual(TEXT("Total carried samples matches canonical sample count"), TotalCarried, Samples.Num());
	TestEqual(TEXT("Per-plate carried sample count mismatches"), CountMismatches, 0);
	TestEqual(TEXT("Invalid carried canonical indices"), InvalidCarriedIndices, 0);
	TestEqual(TEXT("Carried/sample index order mismatches"), IndexOrderMismatches, 0);
	TestEqual(TEXT("Carried field mismatches"), FieldMismatches, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlateMotionTriggerTest, "Aurous.TectonicPlanet.PlateMotionTrigger",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPlateMotionTriggerTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	TestTrue(TEXT("Threshold positive"), Planet.GetReconcileDisplacementThreshold() > 0.0);
	TestFalse(TEXT("ShouldReconcile false immediately after plate init"), Planet.ShouldReconcile());
	TestEqual(TEXT("Timestep counter unchanged before StepSimulation"), Planet.GetTimestepCounter(), static_cast<int64>(0));

	const int32 Steps = 2;
	for (int32 StepIndex = 0; StepIndex < Steps; ++StepIndex)
	{
		Planet.AdvancePlateMotionStep();
	}

	TestTrue(TEXT("ShouldReconcile true after accumulated motion exceeds threshold"), Planet.ShouldReconcile());
	TestTrue(TEXT("Max displacement exceeds threshold"),
		Planet.GetMaxAngularDisplacementSinceReconcile() > Planet.GetReconcileDisplacementThreshold());

	int32 RotationMismatchCount = 0;
	int32 InvalidRotationNormCount = 0;
	int32 DisplacementMismatchCount = 0;
	int32 OnDemandPositionMismatchCount = 0;

	for (const FPlate& Plate : Planet.GetPlates())
	{
		const double ExpectedAngle = FMath::Abs(static_cast<double>(Plate.AngularSpeed)) * static_cast<double>(Steps);
		const double ActualAngle = static_cast<double>(Plate.CumulativeRotation.AngularDistance(FQuat::Identity));
		RotationMismatchCount += FMath::IsNearlyEqual(ActualAngle, ExpectedAngle, 1e-5) ? 0 : 1;
		InvalidRotationNormCount += FMath::IsNearlyEqual(static_cast<double>(Plate.CumulativeRotation.Size()), 1.0, 1e-6) ? 0 : 1;
		DisplacementMismatchCount += FMath::IsNearlyEqual(Plate.AngularDisplacementSinceReconcile, ExpectedAngle, 1e-8) ? 0 : 1;

		if (Plate.SampleIndices.Num() > 0)
		{
			const int32 SampleIndex = Plate.SampleIndices[0];
			const FVector RotatedViaPlanet = Planet.GetRotatedSamplePosition(Plate, SampleIndex);
			const FVector RotatedDirect = Plate.CumulativeRotation.RotateVector(Samples[SampleIndex].Position);
			OnDemandPositionMismatchCount += RotatedViaPlanet.Equals(RotatedDirect, 1e-8) ? 0 : 1;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Plate motion trigger metrics (500k): MaxDisp=%.6e Threshold=%.6e RotationMismatch=%d RotationNormMismatch=%d DisplacementMismatch=%d OnDemandPosMismatch=%d"),
		Planet.GetMaxAngularDisplacementSinceReconcile(),
		Planet.GetReconcileDisplacementThreshold(),
		RotationMismatchCount,
		InvalidRotationNormCount,
		DisplacementMismatchCount,
		OnDemandPositionMismatchCount);

	TestEqual(TEXT("Rotation accumulation mismatches"), RotationMismatchCount, 0);
	TestEqual(TEXT("Quaternion normalization mismatches"), InvalidRotationNormCount, 0);
	TestEqual(TEXT("Displacement accumulation mismatches"), DisplacementMismatchCount, 0);
	TestEqual(TEXT("On-demand rotated position mismatches"), OnDemandPositionMismatchCount, 0);

	Planet.ResetDisplacementTracking();
	TestFalse(TEXT("ShouldReconcile false after reset"), Planet.ShouldReconcile());
	TestTrue(TEXT("Max displacement near zero after reset"),
		FMath::IsNearlyZero(Planet.GetMaxAngularDisplacementSinceReconcile(), 1e-12));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStepSimulationTest, "Aurous.TectonicPlanet.StepSimulation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FStepSimulationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	TestEqual(TEXT("Initial timestep"), Planet.GetTimestepCounter(), static_cast<int64>(0));
	TestEqual(TEXT("Initial reconcile count"), Planet.GetReconcileCount(), 0);

	Planet.StepSimulation();

	TestEqual(TEXT("Timestep incremented"), Planet.GetTimestepCounter(), static_cast<int64>(1));
	TestTrue(TEXT("Reconcile stub invoked at least once"), Planet.GetReconcileCount() >= 1);
	TestFalse(TEXT("Displacement trigger cleared after StepSimulation reconcile"), Planet.ShouldReconcile());
	TestTrue(TEXT("Max displacement reset after StepSimulation"),
		FMath::IsNearlyZero(Planet.GetMaxAngularDisplacementSinceReconcile(), 1e-12));

	int32 ZeroRotationPlates = 0;
	int32 NonZeroPlateDisplacements = 0;
	for (const FPlate& Plate : Planet.GetPlates())
	{
		ZeroRotationPlates += Plate.CumulativeRotation.AngularDistance(FQuat::Identity) <= 1e-8 ? 1 : 0;
		NonZeroPlateDisplacements += Plate.AngularDisplacementSinceReconcile > 0.0 ? 1 : 0;
	}

	UE_LOG(LogTemp, Log, TEXT("StepSimulation metrics (500k): ReconcileCount=%d ZeroRotationPlates=%d NonZeroDisplacementsPostReset=%d"),
		Planet.GetReconcileCount(),
		ZeroRotationPlates,
		NonZeroPlateDisplacements);

	TestEqual(TEXT("All plates rotated after StepSimulation"), ZeroRotationPlates, 0);
	TestEqual(TEXT("Per-plate displacement reset after reconcile"), NonZeroPlateDisplacements, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpatialSoupConstructionTest, "Aurous.TectonicPlanet.SpatialSoupConstruction",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FSpatialSoupConstructionTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);
	Planet.AdvancePlateMotionStep();
	Planet.RebuildSpatialQueryData();

	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FPlate>& Plates = Planet.GetPlates();

	int32 MissingSoupState = 0;
	int32 TriangleCountMismatches = 0;
	int32 MissingCapState = 0;
	int32 InvalidCapCosine = 0;
	int32 MissingRotatedVertexChecks = 0;
	int32 RotatedVertexMismatches = 0;
	int32 TotalCheckedVertices = 0;

	for (const FPlate& Plate : Plates)
	{
		int32 SoupTriangleCount = 0;
		int32 SoupVertexCount = 0;
		if (!Planet.GetPlateSoupTriangleAndVertexCounts(Plate.Id, SoupTriangleCount, SoupVertexCount))
		{
			++MissingSoupState;
			continue;
		}

		TriangleCountMismatches += (SoupTriangleCount == Plate.InteriorTriangles.Num()) ? 0 : 1;

		FVector CapCenter = FVector::ZeroVector;
		double CapCosHalfAngle = -2.0;
		if (!Planet.GetPlateBoundingCap(Plate.Id, CapCenter, CapCosHalfAngle))
		{
			++MissingCapState;
		}
		else
		{
			InvalidCapCosine += (CapCosHalfAngle < -1.0 || CapCosHalfAngle > 1.0) ? 1 : 0;
		}

		int32 CheckedForPlate = 0;
		for (const int32 SampleIndex : Plate.SampleIndices)
		{
			if (!Samples.IsValidIndex(SampleIndex))
			{
				continue;
			}

			FVector SoupRotatedPosition = FVector::ZeroVector;
			if (!Planet.GetPlateSoupRotatedVertex(Plate.Id, SampleIndex, SoupRotatedPosition))
			{
				continue;
			}

			const FVector ExpectedPosition = Planet.GetRotatedSamplePosition(Plate, SampleIndex);
			RotatedVertexMismatches += SoupRotatedPosition.Equals(ExpectedPosition, 1e-8) ? 0 : 1;
			++CheckedForPlate;
			++TotalCheckedVertices;

			if (CheckedForPlate >= 3)
			{
				break;
			}
		}

		MissingRotatedVertexChecks += (CheckedForPlate == 0) ? 1 : 0;
	}

	UE_LOG(LogTemp, Log, TEXT("Spatial soup metrics (500k): MissingSoup=%d TriangleMismatch=%d MissingCap=%d InvalidCapCos=%d MissingVertexChecks=%d RotatedVertexMismatch=%d CheckedVertices=%d"),
		MissingSoupState,
		TriangleCountMismatches,
		MissingCapState,
		InvalidCapCosine,
		MissingRotatedVertexChecks,
		RotatedVertexMismatches,
		TotalCheckedVertices);

	TestEqual(TEXT("All plates have soup state"), MissingSoupState, 0);
	TestEqual(TEXT("Soup triangle count matches plate interior triangles"), TriangleCountMismatches, 0);
	TestEqual(TEXT("All plates have bounding caps"), MissingCapState, 0);
	TestEqual(TEXT("Bounding cap cosine in valid range"), InvalidCapCosine, 0);
	TestEqual(TEXT("Each plate had at least one soup vertex check"), MissingRotatedVertexChecks, 0);
	TestEqual(TEXT("Rotated soup vertex mismatches"), RotatedVertexMismatches, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContainmentQueryTest, "Aurous.TectonicPlanet.ContainmentQuery",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FContainmentQueryTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	for (int32 StepIndex = 0; StepIndex < 3; ++StepIndex)
	{
		Planet.AdvancePlateMotionStep();
	}
	Planet.RebuildSpatialQueryData();

	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();
	const TArray<FPlate>& Plates = Planet.GetPlates();

	int32 NumPlateQueries = 0;
	int32 MissingContainment = 0;
	int32 PlateIdMismatches = 0;
	int32 InvalidBarycentrics = 0;
	int32 MissingInteriorTriangles = 0;

	for (const FPlate& Plate : Plates)
	{
		if (Plate.InteriorTriangles.Num() == 0)
		{
			++MissingInteriorTriangles;
			continue;
		}

		const int32 TriangleIndex = Plate.InteriorTriangles[0];
		if (!Triangles.IsValidIndex(TriangleIndex))
		{
			++MissingContainment;
			continue;
		}

		const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
		const FVector A = Planet.GetRotatedSamplePosition(Plate, Triangle.V[0]);
		const FVector B = Planet.GetRotatedSamplePosition(Plate, Triangle.V[1]);
		const FVector C = Planet.GetRotatedSamplePosition(Plate, Triangle.V[2]);
		const FVector QueryPosition = (A * 0.98 + B * 0.01 + C * 0.01).GetSafeNormal();

		const FContainmentQueryResult Result = Planet.QueryContainment(QueryPosition);
		++NumPlateQueries;

		if (!Result.bFoundContainingPlate)
		{
			++MissingContainment;
			continue;
		}

		PlateIdMismatches += (Result.PlateId == Plate.Id) ? 0 : 1;
		const double BarySum = Result.Barycentric.X + Result.Barycentric.Y + Result.Barycentric.Z;
		const bool bValidBarycentric =
			FMath::IsNearlyEqual(BarySum, 1.0, 1e-4) &&
			Result.Barycentric.X >= -1e-3 &&
			Result.Barycentric.Y >= -1e-3 &&
			Result.Barycentric.Z >= -1e-3;
		InvalidBarycentrics += bValidBarycentric ? 0 : 1;
	}

	UE_LOG(LogTemp, Log, TEXT("Containment metrics (500k): PlateQueries=%d Missing=%d PlateMismatch=%d InvalidBary=%d MissingInterior=%d"),
		NumPlateQueries,
		MissingContainment,
		PlateIdMismatches,
		InvalidBarycentrics,
		MissingInteriorTriangles);

	TestTrue(TEXT("Executed plate containment queries"), NumPlateQueries > 0);
	TestEqual(TEXT("Every plate had at least one interior triangle"), MissingInteriorTriangles, 0);
	TestEqual(TEXT("Missing containment results"), MissingContainment, 0);
	TestEqual(TEXT("Plate ID mismatches"), PlateIdMismatches, 0);
	TestEqual(TEXT("Invalid barycentric results"), InvalidBarycentrics, 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContainmentBroadPhaseTest, "Aurous.TectonicPlanet.ContainmentBroadPhase",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FContainmentBroadPhaseTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);
	Planet.RebuildSpatialQueryData();

	const int32 NumPlates = Planet.GetPlates().Num();
	FRandomStream Random(1337);

	const int32 NumQueries = 1024;
	int32 NumQueriesWithCull = 0;
	int32 GapCount = 0;
	int32 InvalidCandidateCounts = 0;
	int64 TotalCandidates = 0;

	for (int32 QueryIndex = 0; QueryIndex < NumQueries; ++QueryIndex)
	{
		const FVector QueryDirection = MakeRandomUnitVector(Random);
		const FContainmentQueryResult Result = Planet.QueryContainment(QueryDirection);
		InvalidCandidateCounts += (Result.NumCapCandidates < 0 || Result.NumCapCandidates > NumPlates) ? 1 : 0;
		NumQueriesWithCull += (Result.NumCapCandidates < NumPlates) ? 1 : 0;
		GapCount += Result.bGap ? 1 : 0;
		TotalCandidates += Result.NumCapCandidates;
	}

	const double AverageCandidates = static_cast<double>(TotalCandidates) / static_cast<double>(NumQueries);
	UE_LOG(LogTemp, Log, TEXT("Containment broad-phase metrics (500k): Queries=%d AvgCapCandidates=%.3f NumWithCull=%d GapCount=%d InvalidCandidateCounts=%d"),
		NumQueries,
		AverageCandidates,
		NumQueriesWithCull,
		GapCount,
		InvalidCandidateCounts);

	TestEqual(TEXT("Invalid cap-candidate count results"), InvalidCandidateCounts, 0);
	TestTrue(TEXT("Broad phase culled at least one plate in some queries"), NumQueriesWithCull > 0);
	TestTrue(TEXT("Average broad-phase candidates is below total plate count"), AverageCandidates < static_cast<double>(NumPlates));
	TestTrue(TEXT("Found at least one gap query"), GapCount > 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FActorConstructionTest, "Aurous.TectonicPlanet.ActorConstruction",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FActorConstructionTest::RunTest(const FString& Parameters)
{
	UWorld* World = UWorld::CreateWorld(
		EWorldType::Game,
		false,
		FName(TEXT("AurousActorConstructionTestWorld")),
		nullptr,
		true);
	TestNotNull(TEXT("Transient world created"), World);
	if (!World)
	{
		return false;
	}

	ATectonicPlanetActor* Actor = NewObject<ATectonicPlanetActor>(World, NAME_None, RF_Transient);
	TestNotNull(TEXT("Actor created"), Actor);
	if (!Actor)
	{
		World->DestroyWorld(false);
		World->RemoveFromRoot();
		return false;
	}

	Actor->NumSamples = TestSampleCount;
	Actor->NumPlates = TestPlateCount;
	Actor->RandomSeed = TestSeed;

	URealtimeMeshComponent* MeshComponent = Cast<URealtimeMeshComponent>(Actor->GetRootComponent());
	TestNotNull(TEXT("Actor root is realtime mesh component"), MeshComponent);

	Actor->GeneratePlanet();

	URealtimeMeshSimple* RealtimeMesh = MeshComponent ? MeshComponent->GetRealtimeMeshAs<URealtimeMeshSimple>() : nullptr;
	TestNotNull(TEXT("RealtimeMeshSimple created after GeneratePlanet"), RealtimeMesh);

	int32 PositionVertexCount = -1;
	int32 SectionGroupCount = 0;
	if (RealtimeMesh)
	{
		const TArray<FRealtimeMeshSectionGroupKey> SectionGroups = RealtimeMesh->GetSectionGroups(FRealtimeMeshLODKey(0));
		SectionGroupCount = SectionGroups.Num();
		TestTrue(TEXT("Has at least one section group"), SectionGroupCount > 0);

		if (SectionGroups.Num() > 0)
		{
			RealtimeMesh->ProcessMesh(SectionGroups[0],
				[&PositionVertexCount](const RealtimeMesh::FRealtimeMeshStreamSet& Streams)
				{
					const RealtimeMesh::FRealtimeMeshStream* PositionStream = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::Position);
					PositionVertexCount = PositionStream ? PositionStream->Num() : -1;
				});
		}
	}

	TestEqual(TEXT("RealtimeMesh vertex count matches sample count"), PositionVertexCount, TestSampleCount);

	if (Actor)
	{
		const EPlanetDebugMode Modes[] = {
			EPlanetDebugMode::PlateId,
			EPlanetDebugMode::CrustType,
			EPlanetDebugMode::Boundary,
			EPlanetDebugMode::BoundaryType,
			EPlanetDebugMode::Elevation
		};
		for (EPlanetDebugMode Mode : Modes)
		{
			Actor->DebugMode = Mode;
			Actor->UpdateDebugColors();
		}
	}

	World->DestroyWorld(false);
	World->RemoveFromRoot();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReconcileLongRunTest, "Aurous.TectonicPlanet.ReconcileLongRun",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FReconcileLongRunTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const int32 NumSteps = 60;
	const int32 SampleStride = FMath::Max(1, TestSampleCount / 2048);

	TArray<int32> WatchSampleIndices;
	for (int32 SampleIndex = 0; SampleIndex < TestSampleCount; SampleIndex += SampleStride)
	{
		WatchSampleIndices.Add(SampleIndex);
	}

	const TArray<FCanonicalSample>& InitialSamples = Planet.GetSamples();
	TArray<FVector> WatchInitialPositions;
	TArray<int32> WatchPreviousPlateIds;
	TArray<int32> WatchCurrentPlateIds;
	WatchInitialPositions.Reserve(WatchSampleIndices.Num());
	WatchPreviousPlateIds.Reserve(WatchSampleIndices.Num());
	WatchCurrentPlateIds.Reserve(WatchSampleIndices.Num());
	for (const int32 SampleIndex : WatchSampleIndices)
	{
		WatchInitialPositions.Add(InitialSamples[SampleIndex].Position);
		WatchPreviousPlateIds.Add(INDEX_NONE);
		WatchCurrentPlateIds.Add(InitialSamples[SampleIndex].PlateId);
	}

	int32 ReconcileSteps = 0;
	int32 InvalidPlateAssignments = 0;
	int32 InvalidPreviousAssignments = 0;
	int32 InvalidTimingSnapshots = 0;
	int32 PositionDriftCount = 0;
	int32 PingPongTransitions = 0;
	int32 GapOrphanMismatchCount = 0;

	for (int32 StepIndex = 0; StepIndex < NumSteps; ++StepIndex)
	{
		Planet.StepSimulation();
		const TArray<FCanonicalSample>& Samples = Planet.GetSamples();

		for (const FCanonicalSample& Sample : Samples)
		{
			if (Sample.PlateId < 0 || Sample.PlateId >= TestPlateCount)
			{
				++InvalidPlateAssignments;
			}

			if (Sample.PrevPlateId < 0 || Sample.PrevPlateId >= TestPlateCount)
			{
				++InvalidPreviousAssignments;
			}

			if (Sample.bGapDetected && (Sample.PlateId < 0 || Sample.PlateId >= TestPlateCount))
			{
				++GapOrphanMismatchCount;
			}
		}

		for (int32 WatchIndex = 0; WatchIndex < WatchSampleIndices.Num(); ++WatchIndex)
		{
			const int32 SampleIndex = WatchSampleIndices[WatchIndex];
			const FCanonicalSample& Sample = Samples[SampleIndex];

			PositionDriftCount += Sample.Position.Equals(WatchInitialPositions[WatchIndex], 1e-10) ? 0 : 1;

			if (Planet.WasReconcileTriggeredLastStep())
			{
				const int32 CurrentPlateId = Sample.PlateId;
				const int32 PreviousPlateId = WatchCurrentPlateIds[WatchIndex];
				const int32 OldPlateId = WatchPreviousPlateIds[WatchIndex];
				if (OldPlateId != INDEX_NONE && OldPlateId == CurrentPlateId && PreviousPlateId != CurrentPlateId)
				{
					++PingPongTransitions;
				}

				WatchPreviousPlateIds[WatchIndex] = PreviousPlateId;
				WatchCurrentPlateIds[WatchIndex] = CurrentPlateId;
			}
		}

		if (Planet.WasReconcileTriggeredLastStep())
		{
			++ReconcileSteps;
			const FReconcilePhaseTimings& Timings = Planet.GetLastReconcileTimings();
			const bool bTimingsValid =
				Timings.Phase1BuildSpatialMs >= 0.0 &&
				Timings.Phase2OwnershipMs >= 0.0 &&
				Timings.Phase3InterpolationMs >= 0.0 &&
				Timings.Phase4GapMs >= 0.0 &&
				Timings.Phase5OverlapMs >= 0.0 &&
				Timings.Phase6MembershipMs >= 0.0 &&
				Timings.Phase7TerraneMs >= 0.0 &&
				Timings.TotalMs > 0.0;
			InvalidTimingSnapshots += bTimingsValid ? 0 : 1;
		}
	}

	const double PingPongRate = (WatchSampleIndices.Num() > 0 && ReconcileSteps > 0)
		? static_cast<double>(PingPongTransitions) / static_cast<double>(WatchSampleIndices.Num() * ReconcileSteps)
		: 0.0;

	UE_LOG(LogTemp, Log, TEXT("Reconcile long-run metrics (500k): Steps=%d Reconciles=%d InvalidPlates=%d InvalidPrev=%d InvalidTiming=%d PositionDrift=%d PingPong=%d (%.3f%%) GapOrphanMismatch=%d"),
		NumSteps,
		ReconcileSteps,
		InvalidPlateAssignments,
		InvalidPreviousAssignments,
		InvalidTimingSnapshots,
		PositionDriftCount,
		PingPongTransitions,
		PingPongRate * 100.0,
		GapOrphanMismatchCount);

	TestTrue(TEXT("Reconcile triggered repeatedly over long run"), ReconcileSteps >= 10);
	TestEqual(TEXT("No orphan samples during long run"), InvalidPlateAssignments, 0);
	TestEqual(TEXT("PrevPlateId remains valid during long run"), InvalidPreviousAssignments, 0);
	TestEqual(TEXT("Per-phase timing snapshots are valid"), InvalidTimingSnapshots, 0);
	TestEqual(TEXT("Canonical positions are immutable across buffer swaps"), PositionDriftCount, 0);
	TestEqual(TEXT("Gap fallback does not produce orphan samples"), GapOrphanMismatchCount, 0);
	TestTrue(TEXT("Boundary ping-pong transition rate remains bounded"), PingPongRate <= 0.10);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FActorSimulationThreadTest, "Aurous.TectonicPlanet.ActorSimulationThread",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FActorSimulationThreadTest::RunTest(const FString& Parameters)
{
	UWorld* World = UWorld::CreateWorld(
		EWorldType::Game,
		false,
		FName(TEXT("AurousActorSimulationThreadTestWorld")),
		nullptr,
		true);
	TestNotNull(TEXT("Transient world created"), World);
	if (!World)
	{
		return false;
	}

	ATectonicPlanetActor* Actor = NewObject<ATectonicPlanetActor>(World, NAME_None, RF_Transient);
	TestNotNull(TEXT("Actor created"), Actor);
	if (!Actor)
	{
		World->DestroyWorld(false);
		World->RemoveFromRoot();
		return false;
	}

	Actor->NumSamples = TestSampleCount;
	Actor->NumPlates = TestPlateCount;
	Actor->RandomSeed = TestSeed;
	Actor->SimulationTimestepsPerSecond = 30.0f;

	Actor->GeneratePlanet();
	const int64 InitialTimestep = Actor->GetBackgroundTimestepCounterThreadSafe();
	Actor->StartSimulation();

	bool bObservedTimestepAdvance = false;
	bool bObservedReconcileTrigger = false;
	for (int32 PollIndex = 0; PollIndex < 120; ++PollIndex)
	{
		FPlatformProcess::SleepNoStats(0.01f);
		const int64 CurrentTimestep = Actor->GetBackgroundTimestepCounterThreadSafe();
		const bool bBackgroundTriggered = Actor->IsBackgroundReconcileTriggeredThreadSafe();

		if (CurrentTimestep > InitialTimestep)
		{
			bObservedTimestepAdvance = true;
		}

		bObservedReconcileTrigger |= bBackgroundTriggered;

		if (PollIndex % 30 == 0)
		{
			Actor->UpdateDebugColors();
		}

		if (bObservedTimestepAdvance && bObservedReconcileTrigger)
		{
			break;
		}
	}

	Actor->StopSimulation();

	UE_LOG(LogTemp, Log, TEXT("Actor simulation-thread metrics (500k): InitialStep=%lld FinalStep=%lld ObservedAdvance=%d ObservedReconcile=%d RunningAfterStop=%d"),
		InitialTimestep,
		Actor->GetBackgroundTimestepCounterThreadSafe(),
		bObservedTimestepAdvance ? 1 : 0,
		bObservedReconcileTrigger ? 1 : 0,
		Actor->IsSimulationThreadRunningThreadSafe() ? 1 : 0);

	TestTrue(TEXT("Background simulation advances timestep counter"), bObservedTimestepAdvance);
	TestTrue(TEXT("Background simulation triggers reconciliation"), bObservedReconcileTrigger);
	TestFalse(TEXT("Simulation thread reports stopped after StopSimulation"), Actor->IsSimulationThreadRunningThreadSafe());

	World->DestroyWorld(false);
	World->RemoveFromRoot();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlateMotionInvestigationTest, "Aurous.Diagnostics.PlateMotionInvestigation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPlateMotionInvestigationTest::RunTest(const FString& Parameters)
{
	FTectonicPlanet Planet = MakePlanetCopy();
	Planet.InitializePlates(TestPlateCount, TestSeed);

	const TArray<FCanonicalSample>& InitialSamples = Planet.GetSamples();
	TArray<int32> InitialPlateIds;
	InitialPlateIds.Reserve(InitialSamples.Num());
	TArray<int32> PreviousPlateIds;
	PreviousPlateIds.Reserve(InitialSamples.Num());
	for (const FCanonicalSample& Sample : InitialSamples)
	{
		InitialPlateIds.Add(Sample.PlateId);
		PreviousPlateIds.Add(Sample.PlateId);
	}

	TArray<int32> BoundaryWatchIndices;
	BoundaryWatchIndices.Reserve(10);
	for (int32 SampleIndex = 0; SampleIndex < InitialSamples.Num() && BoundaryWatchIndices.Num() < 10; ++SampleIndex)
	{
		if (InitialSamples[SampleIndex].bIsBoundary)
		{
			BoundaryWatchIndices.Add(SampleIndex);
		}
	}
	if (BoundaryWatchIndices.Num() == 0)
	{
		BoundaryWatchIndices.Add(0);
	}

	const int32 ContainmentWatchIndex = BoundaryWatchIndices[0];
	const FVector ContainmentWatchPosition = InitialSamples[ContainmentWatchIndex].Position;
	const FContainmentQueryResult Step0Containment = Planet.QueryContainment(ContainmentWatchPosition);

	UE_LOG(LogTemp, Log,
		TEXT("Motion diagnostic step0 containment: sample=%d plate=%d found=%d containing=%d"),
		ContainmentWatchIndex,
		Step0Containment.PlateId,
		Step0Containment.bFoundContainingPlate ? 1 : 0,
		Step0Containment.NumContainingPlates);

	TArray<int32> PlateChangesPerReconcile;
	TArray<int32> Step50PlateIds;
	TArray<int32> Step100PlateIds;
	const int32 NumSteps = 100;
	for (int32 StepIndex = 0; StepIndex < NumSteps; ++StepIndex)
	{
		Planet.StepSimulation();
		const TArray<FCanonicalSample>& Samples = Planet.GetSamples();

		if (StepIndex == 49)
		{
			Step50PlateIds.Reserve(BoundaryWatchIndices.Num());
			for (const int32 SampleIndex : BoundaryWatchIndices)
			{
				Step50PlateIds.Add(Samples[SampleIndex].PlateId);
			}
		}

		if (Planet.WasReconcileTriggeredLastStep())
		{
			int32 ChangedThisReconcile = 0;
			for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
			{
				if (Samples[SampleIndex].PlateId != PreviousPlateIds[SampleIndex])
				{
					++ChangedThisReconcile;
				}
				PreviousPlateIds[SampleIndex] = Samples[SampleIndex].PlateId;
			}
			PlateChangesPerReconcile.Add(ChangedThisReconcile);
		}
	}

	const TArray<FCanonicalSample>& FinalSamples = Planet.GetSamples();
	for (const int32 SampleIndex : BoundaryWatchIndices)
	{
		Step100PlateIds.Add(FinalSamples[SampleIndex].PlateId);
	}

	int32 ChangedFromInitial = 0;
	for (int32 SampleIndex = 0; SampleIndex < FinalSamples.Num(); ++SampleIndex)
	{
		ChangedFromInitial += (FinalSamples[SampleIndex].PlateId != InitialPlateIds[SampleIndex]) ? 1 : 0;
	}

	const FContainmentQueryResult Step100Containment = Planet.QueryContainment(ContainmentWatchPosition);

	UE_LOG(LogTemp, Log,
		TEXT("Motion diagnostic step100 containment: sample=%d plate=%d found=%d containing=%d"),
		ContainmentWatchIndex,
		Step100Containment.PlateId,
		Step100Containment.bFoundContainingPlate ? 1 : 0,
		Step100Containment.NumContainingPlates);

	for (const FPlate& Plate : Planet.GetPlates())
	{
		const double PrincipalAngle = static_cast<double>(Plate.CumulativeRotation.AngularDistance(FQuat::Identity));
		const double IntegratedDisplacement = FMath::Abs(static_cast<double>(Plate.AngularSpeed)) * static_cast<double>(NumSteps);
		UE_LOG(LogTemp, Log,
			TEXT("Motion diagnostic plate %d: speed=%.6f integrated=%.6f principalAngle=%.6f quat=(%.6f, %.6f, %.6f, %.6f)"),
			Plate.Id,
			Plate.AngularSpeed,
			IntegratedDisplacement,
			PrincipalAngle,
			Plate.CumulativeRotation.X,
			Plate.CumulativeRotation.Y,
			Plate.CumulativeRotation.Z,
			Plate.CumulativeRotation.W);
	}

	double MeanChangedPerReconcile = 0.0;
	int32 MaxChangedPerReconcile = 0;
	for (const int32 ChangedCount : PlateChangesPerReconcile)
	{
		MeanChangedPerReconcile += static_cast<double>(ChangedCount);
		MaxChangedPerReconcile = FMath::Max(MaxChangedPerReconcile, ChangedCount);
	}
	if (PlateChangesPerReconcile.Num() > 0)
	{
		MeanChangedPerReconcile /= static_cast<double>(PlateChangesPerReconcile.Num());
	}

	FString BoundaryStep50;
	FString BoundaryStep100;
	for (int32 WatchIndex = 0; WatchIndex < BoundaryWatchIndices.Num(); ++WatchIndex)
	{
		const int32 SampleIndex = BoundaryWatchIndices[WatchIndex];
		const int32 Step0Plate = InitialPlateIds[SampleIndex];
		const int32 Step50Plate = Step50PlateIds.IsValidIndex(WatchIndex) ? Step50PlateIds[WatchIndex] : INDEX_NONE;
		const int32 Step100Plate = Step100PlateIds.IsValidIndex(WatchIndex) ? Step100PlateIds[WatchIndex] : INDEX_NONE;
		BoundaryStep50 += FString::Printf(TEXT("[%d:%d->%d] "), SampleIndex, Step0Plate, Step50Plate);
		BoundaryStep100 += FString::Printf(TEXT("[%d:%d->%d] "), SampleIndex, Step0Plate, Step100Plate);
	}

	UE_LOG(LogTemp, Log,
		TEXT("Motion diagnostic ownership: steps=%d reconciles=%d changed_step0_to_100=%d (%.3f%%) mean_changed_per_reconcile=%.1f max_changed_per_reconcile=%d"),
		NumSteps,
		PlateChangesPerReconcile.Num(),
		ChangedFromInitial,
		(100.0 * static_cast<double>(ChangedFromInitial)) / static_cast<double>(FinalSamples.Num()),
		MeanChangedPerReconcile,
		MaxChangedPerReconcile);
	UE_LOG(LogTemp, Log, TEXT("Motion diagnostic boundary watch step0->50: %s"), *BoundaryStep50);
	UE_LOG(LogTemp, Log, TEXT("Motion diagnostic boundary watch step0->100: %s"), *BoundaryStep100);

	TestTrue(TEXT("Containment found at step 0"), Step0Containment.bFoundContainingPlate);
	TestTrue(TEXT("Containment found at step 100"), Step100Containment.bFoundContainingPlate);
	TestTrue(TEXT("Ownership changed for at least one sample over 100 steps"), ChangedFromInitial > 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FShewchukPredicatesTest, "Aurous.TectonicPlanet.ShewchukPredicates",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FShewchukPredicatesTest::RunTest(const FString& Parameters)
{
	exactinit();

	double O[3] = { 0.0, 0.0, 0.0 };
	double A[3] = { 1.0, 0.0, 0.0 };
	double B[3] = { 0.0, 1.0, 0.0 };
	double C[3] = { 0.0, 0.0, 1.0 };

	const double Result = orient3d(O, A, B, C);
	TestTrue(TEXT("orient3d non-zero for tetrahedron"), !FMath::IsNearlyZero(Result, 1e-15));

	const double ResultFlipped = orient3d(O, B, A, C);
	TestTrue(TEXT("orient3d flips sign when swapping points"), Result * ResultFlipped < 0.0);

	double D[3] = { 1.0, 1.0, 0.0 };
	const double ResultCoplanar = orient3d(O, A, B, D);
	TestTrue(TEXT("orient3d zero for coplanar"), FMath::IsNearlyZero(ResultCoplanar, 1e-15));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
