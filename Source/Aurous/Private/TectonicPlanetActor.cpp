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

	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	GetPublishedPlanetState(PublishedSamples, PublishedTriangles, PublishedMetrics);
	if (!PublishedSamples.IsValid() || PublishedSamples->Num() == 0)
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

bool ATectonicPlanetActor::ShouldTickIfViewportsOnly() const
{
	return true;
}

void ATectonicPlanetActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	GetPublishedPlanetState(PublishedSamples, PublishedTriangles, PublishedMetrics);
	const bool bHasPublishedPlanet = PublishedSamples.IsValid() && PublishedSamples->Num() > 0;

	const bool bStructuralChanged =
		(CachedNumSamples != NumSamples) ||
		(CachedNumPlates != NumPlates) ||
		(CachedRandomSeed != RandomSeed) ||
		(!FMath::IsNearlyEqual(CachedPlanetRenderRadius, PlanetRenderRadius)) ||
		(!FMath::IsNearlyEqual(CachedTargetContinentalAreaFraction, TargetContinentalAreaFraction)) ||
		(!FMath::IsNearlyEqual(CachedMinContinentalAreaFraction, MinContinentalAreaFraction)) ||
		(!FMath::IsNearlyEqual(CachedMaxContinentalAreaFraction, MaxContinentalAreaFraction)) ||
		(!FMath::IsNearlyEqual(CachedMinContinentalPlateFraction, MinContinentalPlateFraction)) ||
		(CachedPlanetMaterial.Get() != PlanetMaterial);

	if (!bStructuralChanged && bHasPublishedPlanet && MeshGroupKey.IsValid())
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
	CachedTargetContinentalAreaFraction = TargetContinentalAreaFraction;
	CachedMinContinentalAreaFraction = MinContinentalAreaFraction;
	CachedMaxContinentalAreaFraction = MaxContinentalAreaFraction;
	CachedMinContinentalPlateFraction = MinContinentalPlateFraction;
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
	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	{
		FScopeLock Lock(&PlanetMutex);
		Planet.Initialize(NumSamples);
		Planet.SetTargetContinentalAreaFraction(TargetContinentalAreaFraction);
		Planet.SetMinContinentalAreaFraction(MinContinentalAreaFraction);
		Planet.SetMaxContinentalAreaFraction(MaxContinentalAreaFraction);
		Planet.SetMinContinentalPlateFraction(MinContinentalPlateFraction);
		Planet.InitializePlates(NumPlates, RandomSeed);
		PublishedMetrics = BuildMetricsSnapshotFromPlanet_NoLock();
		PublishedSamples = MakeShared<TArray<FCanonicalSample>, ESPMode::ThreadSafe>();
		*PublishedSamples = Planet.GetSamples();
		PublishedTriangles = MakeShared<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe>();
		*PublishedTriangles = Planet.GetTriangles();
		BackgroundTimestepCounter.Store(PublishedMetrics.Timestep);
		BackgroundReconcileTriggered.Store(false);
	}
	PublishPlanetState(MoveTemp(PublishedSamples), MoveTemp(PublishedTriangles), PublishedMetrics);
	const double SimTime = FPlatformTime::Seconds();

	BuildMesh();
	const double EndTime = FPlatformTime::Seconds();

	UE_LOG(LogTemp, Log, TEXT("TectonicPlanetActor: Generated %d samples / %d triangles in %.3f s (sim %.3f s, mesh %.3f s)"),
		PublishedMetrics.SampleCount,
		PublishedMetrics.TriangleCount,
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

	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	GetPublishedPlanetState(PublishedSamples, PublishedTriangles, PublishedMetrics);
	if (!PublishedSamples.IsValid() || PublishedSamples->Num() <= 0)
	{
		return;
	}

	const int32 NumVerts = PublishedSamples->Num();
	RealtimeMesh->EditMeshInPlace(MeshGroupKey,
		[this, NumVerts, PublishedSamples](RealtimeMesh::FRealtimeMeshStreamSet& Streams) -> TSet<FRealtimeMeshStreamKey>
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

			const TArray<FCanonicalSample>& Samples = *PublishedSamples;
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

	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	GetPublishedPlanetState(PublishedSamples, PublishedTriangles, PublishedMetrics);
	if (!PublishedSamples.IsValid() || PublishedSamples->Num() == 0)
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
	bool bHasPlanetData = false;
	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	{
		FScopeLock Lock(&PlanetMutex);
		if (Planet.GetNumSamples() == 0 || Planet.GetPlates().Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("ATectonicPlanetActor::StepSimulationSteps called before planet/plates are initialized."));
		}
		else
		{
			bHasPlanetData = true;
			for (int32 StepIndex = 0; StepIndex < NumSteps; ++StepIndex)
			{
				Planet.StepSimulation();
				bAnyReconcileTriggered |= Planet.WasReconcileTriggeredLastStep();
			}
			PublishedMetrics = BuildMetricsSnapshotFromPlanet_NoLock();
			BackgroundTimestepCounter.Store(PublishedMetrics.Timestep);
			BackgroundReconcileTriggered.Store(bAnyReconcileTriggered);
			if (bAnyReconcileTriggered)
			{
				PublishedSamples = MakeShared<TArray<FCanonicalSample>, ESPMode::ThreadSafe>();
				*PublishedSamples = Planet.GetSamples();
			}
		}
	}

	if (bHasPlanetData)
	{
		PublishPlanetState(MoveTemp(PublishedSamples), nullptr, PublishedMetrics);
	}

	if (bAnyReconcileTriggered)
	{
		bHasPendingColorUpdate.Store(false);
		UpdateDebugColors();
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
	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	{
		FScopeLock Lock(&PlanetMutex);
		if (Planet.GetNumSamples() == 0 || Planet.GetPlates().Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("ATectonicPlanetActor::ForceReconcile called before planet/plates are initialized."));
		}
		else
		{
			Planet.Reconcile();
			PublishedMetrics = BuildMetricsSnapshotFromPlanet_NoLock();
			PublishedSamples = MakeShared<TArray<FCanonicalSample>, ESPMode::ThreadSafe>();
			*PublishedSamples = Planet.GetSamples();
			BackgroundTimestepCounter.Store(PublishedMetrics.Timestep);
			BackgroundReconcileTriggered.Store(true);
			bDidReconcile = true;
		}
	}

	if (bDidReconcile)
	{
		PublishPlanetState(MoveTemp(PublishedSamples), nullptr, PublishedMetrics);
		bHasPendingColorUpdate.Store(false);
		UpdateDebugColors();
	}

	if (bWasRunning)
	{
		StartSimulation();
	}
}

FPlanetControlPanelMetricsSnapshot ATectonicPlanetActor::BuildMetricsSnapshotFromPlanet_NoLock() const
{
	FPlanetControlPanelMetricsSnapshot Snapshot;
	Snapshot.SampleCount = Planet.GetNumSamples();
	Snapshot.TriangleCount = Planet.GetTriangles().Num();
	Snapshot.PlateCount = Planet.GetPlates().Num();
	Snapshot.BoundarySampleCount = Planet.GetBoundarySampleCount();
	Snapshot.BoundaryMeanDepthHops = Planet.GetBoundaryMeanDepthHops();
	Snapshot.BoundaryMaxDepthHops = Planet.GetBoundaryMaxDepthHops();
	Snapshot.BoundaryDeepSampleCount = Planet.GetBoundaryDeepSampleCount();
	Snapshot.ContinentalSampleCount = Planet.GetContinentalSampleCount();
	Snapshot.ContinentalPlateCount = Planet.GetContinentalPlateCount();
	Snapshot.ContinentalAreaFraction = Planet.GetContinentalAreaFraction();
	Snapshot.ContinentalComponentCount = Planet.GetContinentalComponentCount();
	Snapshot.LargestContinentalComponentSize = Planet.GetLargestContinentalComponentSize();
	Snapshot.MaxPlateComponentCount = Planet.GetMaxPlateComponentCount();
	Snapshot.DetachedPlateFragmentSampleCount = Planet.GetDetachedPlateFragmentSampleCount();
	Snapshot.LargestDetachedPlateFragmentSize = Planet.GetLargestDetachedPlateFragmentSize();
	Snapshot.SubductionFrontSampleCount = Planet.GetSubductionFrontSampleCount();
	Snapshot.AndeanSampleCount = Planet.GetAndeanSampleCount();
	Snapshot.TrackedTerraneCount = Planet.GetTrackedTerraneCount();
	Snapshot.ActiveTerraneCount = Planet.GetActiveTerraneCount();
	Snapshot.MergedTerraneCount = Planet.GetMergedTerraneCount();
	Snapshot.CollisionEventCount = Planet.GetCollisionEventCount();
	Snapshot.HimalayanSampleCount = Planet.GetHimalayanSampleCount();
	Snapshot.PendingCollisionSampleCount = Planet.GetPendingCollisionSampleCount();
	Snapshot.MaxSubductionDistanceKm = Planet.GetMaxSubductionDistanceKm();
	Snapshot.MinProtectedPlateSampleCount = Planet.GetMinProtectedPlateSampleCount();
	Snapshot.EmptyProtectedPlateCount = Planet.GetEmptyProtectedPlateCount();
	Snapshot.RescuedProtectedPlateCount = Planet.GetRescuedProtectedPlateCount();
	Snapshot.RescuedProtectedSampleCount = Planet.GetRescuedProtectedSampleCount();
	Snapshot.RepeatedlyRescuedProtectedSampleCount = Planet.GetRepeatedlyRescuedProtectedSampleCount();
	Snapshot.Timestep = Planet.GetTimestepCounter();
	Snapshot.ReconcileCount = Planet.GetReconcileCount();
	Snapshot.bReconcileTriggeredLastStep = Planet.WasReconcileTriggeredLastStep();
	Snapshot.PlateSampleCounts.SetNum(Snapshot.PlateCount);
	for (int32 PlateIndex = 0; PlateIndex < Snapshot.PlateCount; ++PlateIndex)
	{
		Snapshot.PlateSampleCounts[PlateIndex] = Planet.GetPlates()[PlateIndex].SampleIndices.Num();
	}
	if (Snapshot.ReconcileCount > 0)
	{
		Snapshot.Timings = Planet.GetLastReconcileTimings();
		Snapshot.bHasTimings = true;
	}
	return Snapshot;
}
void ATectonicPlanetActor::PublishPlanetState(
	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> InSamples,
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> InTriangles,
	const FPlanetControlPanelMetricsSnapshot& InMetrics)
{
	FScopeLock PublishedStateLock(&PublishedStateMutex);
	if (InSamples.IsValid())
	{
		PublishedState.Samples = MoveTemp(InSamples);
	}
	if (InTriangles.IsValid())
	{
		PublishedState.Triangles = MoveTemp(InTriangles);
	}
	PublishedState.Metrics = InMetrics;
}

void ATectonicPlanetActor::GetPublishedPlanetState(
	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe>& OutSamples,
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe>& OutTriangles,
	FPlanetControlPanelMetricsSnapshot& OutMetrics) const
{
	FScopeLock PublishedStateLock(&PublishedStateMutex);
	OutSamples = PublishedState.Samples;
	OutTriangles = PublishedState.Triangles;
	OutMetrics = PublishedState.Metrics;
}

bool ATectonicPlanetActor::GetLastReconcileTimingsThreadSafe(FReconcilePhaseTimings& OutTimings, int32& OutReconcileCount) const
{
	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	GetPublishedPlanetState(PublishedSamples, PublishedTriangles, PublishedMetrics);
	OutReconcileCount = PublishedMetrics.ReconcileCount;
	if (OutReconcileCount <= 0 || !PublishedMetrics.bHasTimings)
	{
		OutTimings = FReconcilePhaseTimings{};
		return false;
	}

	OutTimings = PublishedMetrics.Timings;
	return true;
}


bool ATectonicPlanetActor::GetPlanetExportDataThreadSafe(
	TArray<FCanonicalSample>& OutSamples,
	TArray<FDelaunayTriangle>& OutTriangles,
	int64& OutTimestep) const
{
	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	GetPublishedPlanetState(PublishedSamples, PublishedTriangles, PublishedMetrics);
	if (!PublishedSamples.IsValid() || !PublishedTriangles.IsValid() || PublishedSamples->Num() == 0 || PublishedTriangles->Num() == 0)
	{
		OutSamples.Reset();
		OutTriangles.Reset();
		OutTimestep = PublishedMetrics.Timestep;
		return false;
	}

	OutSamples = *PublishedSamples;
	OutTriangles = *PublishedTriangles;
	OutTimestep = PublishedMetrics.Timestep;
	return true;
}

bool ATectonicPlanetActor::GetControlPanelMetricsSnapshotThreadSafe(FPlanetControlPanelMetricsSnapshot& OutSnapshot) const
{
	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	GetPublishedPlanetState(PublishedSamples, PublishedTriangles, OutSnapshot);
	return OutSnapshot.SampleCount > 0;
}

void ATectonicPlanetActor::SimulationThreadLoop()
{
	bSimulationThreadRunning.Store(true);

	while (bSimulationThreadShouldRun.Load())
	{
		const double StepStart = FPlatformTime::Seconds();
		bool bTriggeredReconcile = false;
		TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
		FPlanetControlPanelMetricsSnapshot PublishedMetrics;
		{
			FScopeLock Lock(&PlanetMutex);
			Planet.StepSimulation();
			bTriggeredReconcile = Planet.WasReconcileTriggeredLastStep();
			PublishedMetrics = BuildMetricsSnapshotFromPlanet_NoLock();
			if (bTriggeredReconcile)
			{
				PublishedSamples = MakeShared<TArray<FCanonicalSample>, ESPMode::ThreadSafe>();
				*PublishedSamples = Planet.GetSamples();
			}
		}

		PublishPlanetState(MoveTemp(PublishedSamples), nullptr, PublishedMetrics);
		BackgroundTimestepCounter.Store(PublishedMetrics.Timestep);
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

	TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> PublishedSamples;
	TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> PublishedTriangles;
	FPlanetControlPanelMetricsSnapshot PublishedMetrics;
	GetPublishedPlanetState(PublishedSamples, PublishedTriangles, PublishedMetrics);
	if (!PublishedSamples.IsValid() || !PublishedTriangles.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("TectonicPlanetActor: No published planet snapshot available for mesh build"));
		return;
	}

	const TArray<FCanonicalSample>& Samples = *PublishedSamples;
	const TArray<FDelaunayTriangle>& Triangles = *PublishedTriangles;
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
		Builder.AddTriangle(static_cast<uint32>(Tri.V[0]), static_cast<uint32>(Tri.V[1]), static_cast<uint32>(Tri.V[2]), 0);
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

	switch (DebugMode)
	{
	case EPlanetDebugMode::PlateId:
	{
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

	case EPlanetDebugMode::CrustAge:
	{
		// Young crust -> warm colors, old crust -> cool colors.
		const float AgeMy = FMath::Max(0.0f, Sample.Age);
		const float T = FMath::Clamp(AgeMy / 200.0f, 0.0f, 1.0f);
		auto LerpByte = [](uint8 A, uint8 B, float Alpha) -> uint8
		{
			return static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(static_cast<float>(A), static_cast<float>(B), Alpha)), 0, 255));
		};

		if (T < 0.5f)
		{
			const float SubT = T / 0.5f;
			return FColor(LerpByte(255, 255, SubT), LerpByte(80, 220, SubT), LerpByte(40, 80, SubT));
		}

		const float SubT = (T - 0.5f) / 0.5f;
		return FColor(LerpByte(255, 40, SubT), LerpByte(220, 100, SubT), LerpByte(80, 255, SubT));
	}

	case EPlanetDebugMode::SubductionRole:
		if (Sample.bIsSubductionFront)
		{
			return FColor::White;
		}
		switch (Sample.SubductionRole)
		{
		case ESubductionRole::Overriding:
			return FColor(255, 140, 40);
		case ESubductionRole::Subducting:
			return FColor(40, 180, 255);
		case ESubductionRole::None:
		default:
			return FColor::Black;
		}

	case EPlanetDebugMode::SubductionDistance:
	{
		if (Sample.SubductionRole == ESubductionRole::None || Sample.SubductionDistanceKm < 0.0f)
		{
			return FColor::Black;
		}

		const float RadiusKm = (Sample.SubductionRole == ESubductionRole::Subducting)
			? 400.0f
			: 1800.0f;
		const float T = 1.0f - FMath::Clamp(Sample.SubductionDistanceKm / RadiusKm, 0.0f, 1.0f);
		const uint8 Intensity = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(T * 255.0f), 0, 255));
		return (Sample.SubductionRole == ESubductionRole::Subducting)
			? FColor(0, Intensity / 2, Intensity)
			: FColor(Intensity, Intensity / 2, 0);
	}

	case EPlanetDebugMode::OrogenyType:
		switch (Sample.OrogenyType)
		{
		case EOrogenyType::Andean:
			return FColor(255, 120, 0);
		case EOrogenyType::Himalayan:
			return FColor(255, 0, 200);
		case EOrogenyType::None:
		default:
			return FColor::Black;
		}

	case EPlanetDebugMode::TerraneId:
		if (Sample.TerraneId < 0)
		{
			return FColor::Black;
		}
		return PlateColors[Sample.TerraneId % UE_ARRAY_COUNT(PlateColors)];

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

