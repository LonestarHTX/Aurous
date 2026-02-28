#include "TectonicPlanetActor.h"

#include "Async/Async.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Interface/Core/RealtimeMeshBuilder.h"
#include "Interface/Core/RealtimeMeshDataStream.h"
#include "Materials/MaterialInterface.h"
#include "Misc/ScopeLock.h"
#include "Templates/UniquePtr.h"
#include "UObject/UnrealType.h"

ATectonicPlanetActor::ATectonicPlanetActor()
{
	PrimaryActorTick.bCanEverTick = true;

	MeshComponent = CreateDefaultSubobject<URealtimeMeshComponent>(TEXT("PlanetMesh"));
	MeshComponent->SetGenerateOverlapEvents(false);
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetRootComponent(MeshComponent);
}

void ATectonicPlanetActor::BeginPlay()
{
	Super::BeginPlay();

	if (Planet.GetNumSamples() == 0)
	{
		GeneratePlanet();
	}

	if (bAutoStartSimulation)
	{
		StartSimulation();
	}
}

void ATectonicPlanetActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopSimulation();
	Super::EndPlay(EndPlayReason);
}

void ATectonicPlanetActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	CurrentTimestepCounter = BackgroundTimestepCounter.Load();
	const bool bTriggered = BackgroundReconcileTriggered.Load();
	bReconciliationTriggeredThisFrame = bTriggered;
	BackgroundReconcileTriggered.Store(false);
	bSimulationRunning = bSimulationThreadRunning.Load();

	if (bHasPendingColorUpdate.Load())
	{
		bHasPendingColorUpdate.Store(false);
		UpdateDebugColors();
	}
}

void ATectonicPlanetActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	const bool bStructuralChanged =
		(CachedNumSamples != NumSamples) ||
		(CachedNumPlates != NumPlates) ||
		(CachedRandomSeed != RandomSeed) ||
		(!FMath::IsNearlyEqual(CachedPlanetRenderRadius, PlanetRenderRadius)) ||
		(CachedPlanetMaterial.Get() != PlanetMaterial);

	if (!bStructuralChanged && Planet.GetNumSamples() > 0 && MeshGroupKey.IsValid())
	{
		UpdateDebugColors();

		if (MeshComponent)
		{
			if (PlanetMaterial)
			{
				MeshComponent->SetMaterial(0, PlanetMaterial);
				bWarnedMissingPlanetMaterial = false;
			}
			else if (!bWarnedMissingPlanetMaterial)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("ATectonicPlanetActor: PlanetMaterial is null. Create an unlit material with VertexColor -> BaseColor (and/or Emissive), assign it to PlanetMaterial to see debug colors."));
				bWarnedMissingPlanetMaterial = true;
			}
		}

		return;
	}

	CachedNumSamples = NumSamples;
	CachedNumPlates = NumPlates;
	CachedRandomSeed = RandomSeed;
	CachedPlanetRenderRadius = PlanetRenderRadius;
	CachedPlanetMaterial = PlanetMaterial;
	GeneratePlanet();
}

void ATectonicPlanetActor::GeneratePlanet()
{
	if (!MeshComponent)
	{
		UE_LOG(LogTemp, Warning, TEXT("TectonicPlanetActor: MeshComponent is null"));
		return;
	}

	if (NumSamples < 4)
	{
		UE_LOG(LogTemp, Warning, TEXT("TectonicPlanetActor: NumSamples must be >= 4 (got %d)"), NumSamples);
		return;
	}

	if (NumPlates <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("TectonicPlanetActor: NumPlates must be > 0 (got %d)"), NumPlates);
		return;
	}

	const bool bWasRunning = bSimulationThreadShouldRun.Load();
	if (bWasRunning)
	{
		StopSimulation();
	}

	const double StartTime = FPlatformTime::Seconds();
	{
		FScopeLock Lock(&PlanetMutex);
		Planet.Initialize(NumSamples);
		Planet.InitializePlates(NumPlates, RandomSeed);
		BackgroundTimestepCounter.Store(Planet.GetTimestepCounter());
		BackgroundReconcileTriggered.Store(false);
	}
	const double SimTime = FPlatformTime::Seconds();

	BuildMesh();
	const double EndTime = FPlatformTime::Seconds();

	UE_LOG(LogTemp, Log, TEXT("TectonicPlanetActor: Generated %d samples / %d triangles in %.3f s (sim %.3f s, mesh %.3f s)"),
		Planet.GetSamples().Num(),
		Planet.GetTriangles().Num(),
		EndTime - StartTime,
		SimTime - StartTime,
		EndTime - SimTime);

	if (bWasRunning)
	{
		StartSimulation();
	}
}

void ATectonicPlanetActor::UpdateDebugColors()
{
	if (!MeshComponent)
	{
		return;
	}

	URealtimeMeshSimple* RealtimeMesh = MeshComponent->GetRealtimeMeshAs<URealtimeMeshSimple>();
	if (!RealtimeMesh)
	{
		return;
	}

	FScopeLock Lock(&PlanetMutex);
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	if (Samples.Num() <= 0)
	{
		return;
	}

	const int32 NumVerts = Samples.Num();
	RealtimeMesh->EditMeshInPlace(MeshGroupKey,
		[this, NumVerts, &Samples](RealtimeMesh::FRealtimeMeshStreamSet& Streams) -> TSet<FRealtimeMeshStreamKey>
		{
			RealtimeMesh::FRealtimeMeshStream* ColorStream = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::Color);
			if (!ColorStream || ColorStream->Num() < NumVerts)
			{
				return TSet<FRealtimeMeshStreamKey>();
			}

			FColor* Colors = ColorStream->GetData<FColor>();
			if (!Colors)
			{
				return TSet<FRealtimeMeshStreamKey>();
			}

			for (int32 Index = 0; Index < NumVerts; ++Index)
			{
				Colors[Index] = Samples.IsValidIndex(Index) ? GetDebugColor(Samples[Index]) : FColor::Black;
			}

			TSet<FRealtimeMeshStreamKey> DirtyStreams;
			DirtyStreams.Add(RealtimeMesh::FRealtimeMeshStreams::Color);
			return DirtyStreams;
		});
}

void ATectonicPlanetActor::StartSimulation()
{
	if (bSimulationThreadShouldRun.Load())
	{
		return;
	}

	if (Planet.GetNumSamples() == 0)
	{
		GeneratePlanet();
	}

	bSimulationThreadShouldRun.Store(true);
	bSimulationThreadRunning.Store(false);
	SimulationFuture = MakeUnique<TFuture<void>>(Async(EAsyncExecution::Thread,
		[this]()
		{
			SimulationThreadLoop();
		}));
	bSimulationRunning = true;
}

void ATectonicPlanetActor::StopSimulation()
{
	if (!bSimulationThreadShouldRun.Load() && !SimulationFuture.IsValid())
	{
		return;
	}

	bSimulationThreadShouldRun.Store(false);
	if (SimulationFuture.IsValid())
	{
		SimulationFuture->Wait();
		SimulationFuture.Reset();
	}

	bSimulationThreadRunning.Store(false);
	bSimulationRunning = false;
}

void ATectonicPlanetActor::StepSimulationSteps(int32 NumSteps)
{
	if (NumSteps <= 0)
	{
		return;
	}

	const bool bWasRunning = bSimulationThreadShouldRun.Load();
	if (bWasRunning)
	{
		StopSimulation();
	}

	bool bAnyReconcileTriggered = false;
	{
		FScopeLock Lock(&PlanetMutex);
		if (Planet.GetNumSamples() == 0 || Planet.GetPlates().Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("ATectonicPlanetActor::StepSimulationSteps called before planet/plates are initialized."));
		}
		else
		{
			for (int32 StepIndex = 0; StepIndex < NumSteps; ++StepIndex)
			{
				Planet.StepSimulation();
				bAnyReconcileTriggered |= Planet.WasReconcileTriggeredLastStep();
			}
			BackgroundTimestepCounter.Store(Planet.GetTimestepCounter());
			BackgroundReconcileTriggered.Store(bAnyReconcileTriggered);
		}
	}

	if (bAnyReconcileTriggered)
	{
		bHasPendingColorUpdate.Store(true);
	}

	if (bWasRunning)
	{
		StartSimulation();
	}
}

void ATectonicPlanetActor::ForceReconcile()
{
	const bool bWasRunning = bSimulationThreadShouldRun.Load();
	if (bWasRunning)
	{
		StopSimulation();
	}

	bool bDidReconcile = false;
	{
		FScopeLock Lock(&PlanetMutex);
		if (Planet.GetNumSamples() == 0 || Planet.GetPlates().Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("ATectonicPlanetActor::ForceReconcile called before planet/plates are initialized."));
		}
		else
		{
			Planet.Reconcile();
			BackgroundTimestepCounter.Store(Planet.GetTimestepCounter());
			BackgroundReconcileTriggered.Store(true);
			bDidReconcile = true;
		}
	}

	if (bDidReconcile)
	{
		bHasPendingColorUpdate.Store(true);
	}

	if (bWasRunning)
	{
		StartSimulation();
	}
}

bool ATectonicPlanetActor::GetLastReconcileTimingsThreadSafe(FReconcilePhaseTimings& OutTimings, int32& OutReconcileCount) const
{
	FScopeLock Lock(&PlanetMutex);
	OutReconcileCount = Planet.GetReconcileCount();
	if (OutReconcileCount <= 0)
	{
		OutTimings = FReconcilePhaseTimings{};
		return false;
	}

	OutTimings = Planet.GetLastReconcileTimings();
	return true;
}

bool ATectonicPlanetActor::GetPlanetExportDataThreadSafe(
	TArray<FCanonicalSample>& OutSamples,
	TArray<FDelaunayTriangle>& OutTriangles,
	int64& OutTimestep) const
{
	FScopeLock Lock(&PlanetMutex);
	if (Planet.GetNumSamples() == 0 || Planet.GetTriangles().Num() == 0)
	{
		OutSamples.Reset();
		OutTriangles.Reset();
		OutTimestep = Planet.GetTimestepCounter();
		return false;
	}

	OutSamples = Planet.GetSamples();
	OutTriangles = Planet.GetTriangles();
	OutTimestep = Planet.GetTimestepCounter();
	return true;
}

bool ATectonicPlanetActor::GetControlPanelMetricsSnapshotThreadSafe(FPlanetControlPanelMetricsSnapshot& OutSnapshot) const
{
	FScopeLock Lock(&PlanetMutex);
	OutSnapshot = FPlanetControlPanelMetricsSnapshot{};

	OutSnapshot.SampleCount = Planet.GetNumSamples();
	OutSnapshot.TriangleCount = Planet.GetTriangles().Num();
	OutSnapshot.PlateCount = Planet.GetPlates().Num();
	OutSnapshot.BoundarySampleCount = Planet.GetBoundarySampleCount();
	OutSnapshot.Timestep = Planet.GetTimestepCounter();
	OutSnapshot.ReconcileCount = Planet.GetReconcileCount();
	OutSnapshot.bReconcileTriggeredLastStep = Planet.WasReconcileTriggeredLastStep();

	OutSnapshot.PlateSampleCounts.SetNum(OutSnapshot.PlateCount);
	for (int32 PlateIndex = 0; PlateIndex < OutSnapshot.PlateCount; ++PlateIndex)
	{
		OutSnapshot.PlateSampleCounts[PlateIndex] = Planet.GetPlates()[PlateIndex].SampleIndices.Num();
	}

	if (OutSnapshot.ReconcileCount > 0)
	{
		OutSnapshot.Timings = Planet.GetLastReconcileTimings();
		OutSnapshot.bHasTimings = true;
	}

	return OutSnapshot.SampleCount > 0;
}

void ATectonicPlanetActor::SimulationThreadLoop()
{
	bSimulationThreadRunning.Store(true);

	while (bSimulationThreadShouldRun.Load())
	{
		const double StepStart = FPlatformTime::Seconds();
		bool bTriggeredReconcile = false;
		int64 Timestep = 0;
		{
			FScopeLock Lock(&PlanetMutex);
			Planet.StepSimulation();
			bTriggeredReconcile = Planet.WasReconcileTriggeredLastStep();
			Timestep = Planet.GetTimestepCounter();
		}

		BackgroundTimestepCounter.Store(Timestep);
		BackgroundReconcileTriggered.Store(bTriggeredReconcile);
		if (bTriggeredReconcile)
		{
			bHasPendingColorUpdate.Store(true);
		}

		const double TargetDeltaTime = 1.0 / static_cast<double>(FMath::Max(0.1f, SimulationTimestepsPerSecond));
		const double Elapsed = FPlatformTime::Seconds() - StepStart;
		const double SleepTime = TargetDeltaTime - Elapsed;
		if (SleepTime > 0.0)
		{
			FPlatformProcess::SleepNoStats(static_cast<float>(SleepTime));
		}
	}

	bSimulationThreadRunning.Store(false);
}

void ATectonicPlanetActor::BuildMesh()
{
	if (!MeshComponent)
	{
		return;
	}

	URealtimeMeshSimple* RealtimeMesh = Cast<URealtimeMeshSimple>(MeshComponent->GetRealtimeMesh());
	if (!RealtimeMesh)
	{
		RealtimeMesh = MeshComponent->InitializeRealtimeMesh<URealtimeMeshSimple>();
	}
	if (!RealtimeMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("TectonicPlanetActor: Failed to initialize realtime mesh"));
		return;
	}

	if (MeshGroupKey.IsValid())
	{
		RealtimeMesh->RemoveSectionGroup(MeshGroupKey);
		MeshGroupKey = FRealtimeMeshSectionGroupKey();
	}

	FScopeLock Lock(&PlanetMutex);
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();
	const int32 NumVerts = Samples.Num();
	const int32 NumTris = Triangles.Num();

	if (NumVerts <= 0 || NumTris <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("TectonicPlanetActor: No mesh data to build (Verts=%d Tris=%d)"), NumVerts, NumTris);
		return;
	}

	RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
	RealtimeMesh::TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder(StreamSet);
	Builder.EnableTangents();
	Builder.EnableColors();
	Builder.EnablePolyGroups();

	Builder.ReserveNumVertices(NumVerts);
	Builder.ReserveNumTriangles(NumTris);

	for (int32 Index = 0; Index < NumVerts; ++Index)
	{
		const FCanonicalSample& Sample = Samples[Index];

		const FVector3f Position = FVector3f(Sample.Position * static_cast<double>(PlanetRenderRadius));
		const FVector3f Normal = FVector3f(Sample.Position);

		FVector Tangent3d = FVector::CrossProduct(FVector::UpVector, Sample.Position);
		if (Tangent3d.IsNearlyZero())
		{
			Tangent3d = FVector::CrossProduct(FVector::RightVector, Sample.Position);
		}
		Tangent3d.Normalize();
		const FVector3f Tangent = FVector3f(Tangent3d);

		Builder.AddVertex(Position)
			.SetNormalAndTangent(Normal, Tangent)
			.SetColor(GetDebugColor(Sample));
	}

	for (int32 TriIndex = 0; TriIndex < NumTris; ++TriIndex)
	{
		const FDelaunayTriangle& Tri = Triangles[TriIndex];
		Builder.AddTriangle(static_cast<uint32>(Tri.V[0]), static_cast<uint32>(Tri.V[2]), static_cast<uint32>(Tri.V[1]), 0);
	}

	RealtimeMesh->SetupMaterialSlot(0, FName(TEXT("PlanetSurface")));

	MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(0, FName(TEXT("PlanetSurface")));
	RealtimeMesh->CreateSectionGroup(MeshGroupKey, MoveTemp(StreamSet));

	const FRealtimeMeshSectionKey SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(MeshGroupKey, 0);
	RealtimeMesh->UpdateSectionConfig(SectionKey, FRealtimeMeshSectionConfig(0));

	if (PlanetMaterial)
	{
		MeshComponent->SetMaterial(0, PlanetMaterial);
		bWarnedMissingPlanetMaterial = false;
	}
	else if (!bWarnedMissingPlanetMaterial)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("ATectonicPlanetActor: PlanetMaterial is null. Create an unlit material with VertexColor -> BaseColor (and/or Emissive), assign it to PlanetMaterial to see debug colors."));
		bWarnedMissingPlanetMaterial = true;
	}
}

FColor ATectonicPlanetActor::GetDebugColor(const FCanonicalSample& Sample) const
{
	switch (DebugMode)
	{
	case EPlanetDebugMode::PlateId:
	{
		static const FColor PlateColors[] = {
			FColor(230, 25, 75),
			FColor(60, 180, 75),
			FColor(255, 225, 25),
			FColor(0, 130, 200),
			FColor(245, 130, 48),
			FColor(145, 30, 180),
			FColor(70, 240, 240),
			FColor(240, 50, 230),
			FColor(210, 245, 60),
			FColor(250, 190, 212),
			FColor(0, 128, 128),
			FColor(220, 190, 255),
		};

		if (Sample.PlateId < 0)
		{
			return FColor::Black;
		}

		const int32 ColorIdx = Sample.PlateId % UE_ARRAY_COUNT(PlateColors);
		return PlateColors[ColorIdx];
	}

	case EPlanetDebugMode::CrustType:
		return (Sample.CrustType == ECrustType::Continental) ? FColor(139, 90, 43) : FColor(0, 105, 148);

	case EPlanetDebugMode::Boundary:
		return Sample.bIsBoundary ? FColor::Red : FColor(64, 64, 64);

	case EPlanetDebugMode::BoundaryType:
		switch (Sample.BoundaryType)
		{
		case EBoundaryType::Convergent:
			return FColor::Red;
		case EBoundaryType::Divergent:
			return FColor(0, 90, 255);
		case EBoundaryType::Transform:
			return FColor(0, 210, 80);
		case EBoundaryType::None:
		default:
			return FColor::Black;
		}

	case EPlanetDebugMode::Elevation:
	{
		const float T = FMath::Clamp((Sample.Elevation + 10.0f) / 20.0f, 0.0f, 1.0f);
		auto LerpByte = [](uint8 A, uint8 B, float Alpha) -> uint8
		{
			return static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(static_cast<float>(A), static_cast<float>(B), Alpha)), 0, 255));
		};

		if (T < 0.5f)
		{
			const float OceanT = T / 0.5f;
			return FColor(LerpByte(0, 0, OceanT), LerpByte(20, 200, OceanT), LerpByte(80, 220, OceanT));
		}

		const float LandT = (T - 0.5f) / 0.5f;
		if (LandT < 0.5f)
		{
			const float SubT = LandT / 0.5f;
			return FColor(LerpByte(34, 139, SubT), LerpByte(139, 90, SubT), LerpByte(34, 43, SubT));
		}

		const float SubT = (LandT - 0.5f) / 0.5f;
		return FColor(LerpByte(139, 255, SubT), LerpByte(90, 255, SubT), LerpByte(43, 255, SubT));
	}

	default:
		return FColor::White;
	}
}

#if WITH_EDITOR
void ATectonicPlanetActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif
