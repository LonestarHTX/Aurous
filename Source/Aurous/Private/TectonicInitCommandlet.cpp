#include "TectonicInitCommandlet.h"

#include "HAL/PlatformTime.h"
#include "Misc/Parse.h"
#include "ShewchukPredicates.h"
#include "TectonicPlanet.h"

namespace
{
	struct FValidationSummary
	{
		int32 ErrorCount = 0;
		int32 WarningCount = 0;
	};

	void LogError(FValidationSummary& Summary, const TCHAR* Message)
	{
		++Summary.ErrorCount;
		UE_LOG(LogTemp, Error, TEXT("%s"), Message);
	}

	void LogWarning(FValidationSummary& Summary, const TCHAR* Message)
	{
		++Summary.WarningCount;
		UE_LOG(LogTemp, Warning, TEXT("%s"), Message);
	}
}

#define AUR_LOG_ERRORF(SummaryRef, FormatLiteral, ...) \
	do \
	{ \
		++(SummaryRef).ErrorCount; \
		UE_LOG(LogTemp, Error, FormatLiteral, ##__VA_ARGS__); \
	} while (false)

#define AUR_LOG_WARNINGF(SummaryRef, FormatLiteral, ...) \
	do \
	{ \
		++(SummaryRef).WarningCount; \
		UE_LOG(LogTemp, Warning, FormatLiteral, ##__VA_ARGS__); \
	} while (false)

UTectonicInitCommandlet::UTectonicInitCommandlet()
{
	IsClient = false;
	IsServer = false;
	IsEditor = true;
	LogToConsole = true;
	ShowErrorCount = true;
}

int32 UTectonicInitCommandlet::Main(const FString& Params)
{
	int32 NumSamples = 500000;
	FParse::Value(*Params, TEXT("Samples="), NumSamples);
	FParse::Value(*Params, TEXT("Count="), NumSamples);

	const bool bStrict = FParse::Param(*Params, TEXT("Strict"));
	const double StartSeconds = FPlatformTime::Seconds();

	UE_LOG(LogTemp, Display, TEXT("TectonicInitCommandlet: Initializing planet with %d samples"), NumSamples);

	FTectonicPlanet Planet;
	Planet.Initialize(NumSamples);

	const double EndSeconds = FPlatformTime::Seconds();
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();

	FValidationSummary Summary;

	if (Samples.Num() != NumSamples)
	{
		AUR_LOG_ERRORF(Summary, TEXT("Sample count mismatch: requested=%d actual=%d"), NumSamples, Samples.Num());
	}

	double MinRadius = TNumericLimits<double>::Max();
	double MaxRadius = 0.0;
	double MaxRadiusError = 0.0;

	for (const FCanonicalSample& Sample : Samples)
	{
		const double Radius = Sample.Position.Size();
		MinRadius = FMath::Min(MinRadius, Radius);
		MaxRadius = FMath::Max(MaxRadius, Radius);
		MaxRadiusError = FMath::Max(MaxRadiusError, FMath::Abs(Radius - 1.0));
	}

	UE_LOG(LogTemp, Display, TEXT("Radius stats: min=%.15f max=%.15f max|r-1|=%.3e"), MinRadius, MaxRadius, MaxRadiusError);
	if (MaxRadiusError > 1e-9)
	{
		AUR_LOG_WARNINGF(Summary, TEXT("Max radius error %.3e exceeds tight tolerance 1e-9"), MaxRadiusError);
	}

	const int32 ExpectedTriangleCount = (Samples.Num() > 0) ? (2 * Samples.Num() - 4) : 0;
	const int32 TriangleCountDelta = Triangles.Num() - ExpectedTriangleCount;
	UE_LOG(LogTemp, Display, TEXT("Triangle stats: actual=%d expected~=%d delta=%d"), Triangles.Num(), ExpectedTriangleCount, TriangleCountDelta);
	if (Samples.Num() >= 4 && Triangles.Num() != ExpectedTriangleCount)
	{
		AUR_LOG_WARNINGF(Summary, TEXT("Triangle count differs from 2N-4 by %d"), TriangleCountDelta);
	}

	TArray<int32> TriangleIncidenceCounts;
	TriangleIncidenceCounts.Init(0, Samples.Num());

	int32 InvalidTriangleIndexCount = 0;
	int32 DegenerateTriangleCount = 0;
	double MinTriangleArea2 = TNumericLimits<double>::Max();
	double MaxTriangleArea2 = 0.0;

	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
		const int32 AIndex = Triangle.V[0];
		const int32 BIndex = Triangle.V[1];
		const int32 CIndex = Triangle.V[2];

		if (!Samples.IsValidIndex(AIndex) || !Samples.IsValidIndex(BIndex) || !Samples.IsValidIndex(CIndex))
		{
			++InvalidTriangleIndexCount;
			continue;
		}

		if (AIndex == BIndex || BIndex == CIndex || CIndex == AIndex)
		{
			++DegenerateTriangleCount;
			continue;
		}

		const FVector& A = Samples[AIndex].Position;
		const FVector& B = Samples[BIndex].Position;
		const FVector& C = Samples[CIndex].Position;
		const double Area2 = FVector::CrossProduct(B - A, C - A).Size();
		MinTriangleArea2 = FMath::Min(MinTriangleArea2, Area2);
		MaxTriangleArea2 = FMath::Max(MaxTriangleArea2, Area2);
		if (Area2 <= static_cast<double>(SMALL_NUMBER))
		{
			++DegenerateTriangleCount;
			continue;
		}

		++TriangleIncidenceCounts[AIndex];
		++TriangleIncidenceCounts[BIndex];
		++TriangleIncidenceCounts[CIndex];
	}

	int32 SamplesMissingFromTriangles = 0;
	for (const int32 IncidenceCount : TriangleIncidenceCounts)
	{
		SamplesMissingFromTriangles += (IncidenceCount == 0) ? 1 : 0;
	}

	UE_LOG(LogTemp, Display, TEXT("Triangle quality: invalidIndex=%d degenerate=%d min|cross|=%.6e max|cross|=%.6e missingVertices=%d"),
		InvalidTriangleIndexCount,
		DegenerateTriangleCount,
		(MinTriangleArea2 == TNumericLimits<double>::Max()) ? 0.0 : MinTriangleArea2,
		MaxTriangleArea2,
		SamplesMissingFromTriangles);

	if (InvalidTriangleIndexCount > 0)
	{
		AUR_LOG_ERRORF(Summary, TEXT("Found %d triangles with invalid vertex indices"), InvalidTriangleIndexCount);
	}
	if (DegenerateTriangleCount > 0)
	{
		AUR_LOG_ERRORF(Summary, TEXT("Found %d degenerate triangles"), DegenerateTriangleCount);
	}
	if (SamplesMissingFromTriangles > 0)
	{
		AUR_LOG_ERRORF(Summary, TEXT("Found %d samples not referenced by any triangle"), SamplesMissingFromTriangles);
	}

	if (Adjacency.Num() != Samples.Num())
	{
		AUR_LOG_ERRORF(Summary, TEXT("Adjacency size mismatch: samples=%d adjacency=%d"), Samples.Num(), Adjacency.Num());
	}
	else
	{
		int32 ZeroValenceVertices = 0;
		int32 InvalidNeighbors = 0;
		int32 AsymmetricEdges = 0;
		int32 MinValence = MAX_int32;
		int32 MaxValence = 0;
		int64 TotalValence = 0;

		for (int32 VertexIndex = 0; VertexIndex < Adjacency.Num(); ++VertexIndex)
		{
			const TArray<int32>& Neighbors = Adjacency[VertexIndex];
			const int32 Valence = Neighbors.Num();

			ZeroValenceVertices += (Valence == 0) ? 1 : 0;
			MinValence = FMath::Min(MinValence, Valence);
			MaxValence = FMath::Max(MaxValence, Valence);
			TotalValence += Valence;

			for (const int32 NeighborIndex : Neighbors)
			{
				if (!Adjacency.IsValidIndex(NeighborIndex))
				{
					++InvalidNeighbors;
					continue;
				}

				if (!Adjacency[NeighborIndex].Contains(VertexIndex))
				{
					++AsymmetricEdges;
				}
			}
		}

		const double AverageValence = (Adjacency.Num() > 0) ? static_cast<double>(TotalValence) / static_cast<double>(Adjacency.Num()) : 0.0;
		UE_LOG(LogTemp, Display, TEXT("Adjacency stats: vertices=%d min/max/avg valence=%d/%d/%.3f zeroValence=%d invalidNeighbors=%d asymmetricRefs=%d"),
			Adjacency.Num(),
			(MinValence == MAX_int32) ? 0 : MinValence,
			MaxValence,
			AverageValence,
			ZeroValenceVertices,
			InvalidNeighbors,
			AsymmetricEdges);

		if (ZeroValenceVertices > 0)
		{
			AUR_LOG_ERRORF(Summary, TEXT("Found %d zero-valence vertices"), ZeroValenceVertices);
		}
		if (InvalidNeighbors > 0)
		{
			AUR_LOG_ERRORF(Summary, TEXT("Found %d invalid adjacency references"), InvalidNeighbors);
		}
		if (AsymmetricEdges > 0)
		{
			AUR_LOG_ERRORF(Summary, TEXT("Found %d asymmetric adjacency references"), AsymmetricEdges);
		}
		if (MinValence < 5 || MaxValence > 7)
		{
			AUR_LOG_WARNINGF(Summary, TEXT("Valence outlier range observed (expected typical 5-7): min=%d max=%d"), MinValence, MaxValence);
		}
	}

	// Basic smoke test that Shewchuk predicates are callable after Initialize() invokes exactinit().
	const double Pa[3] = { 1.0, 0.0, 0.0 };
	const double Pb[3] = { 0.0, 1.0, 0.0 };
	const double Pc[3] = { 0.0, 0.0, 1.0 };
	const double Pd[3] = { 0.0, 0.0, 0.0 };
	const double Orientation = orient3d(Pa, Pb, Pc, Pd);
	UE_LOG(LogTemp, Display, TEXT("Shewchuk orient3d smoke test result: %.6e"), Orientation);
	if (Orientation == 0.0)
	{
		LogWarning(Summary, TEXT("orient3d smoke test returned zero (unexpected for tetrahedron test points)"));
	}

	UE_LOG(LogTemp, Display, TEXT("Initialization completed in %.3f s with %d error(s), %d warning(s)."),
		EndSeconds - StartSeconds,
		Summary.ErrorCount,
		Summary.WarningCount);

	if (Summary.ErrorCount > 0)
	{
		return 1;
	}

	if (bStrict && Summary.WarningCount > 0)
	{
		UE_LOG(LogTemp, Display, TEXT("Strict mode enabled: treating warnings as failure."));
		return 2;
	}

	return 0;
}

#undef AUR_LOG_WARNINGF
#undef AUR_LOG_ERRORF
