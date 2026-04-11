#include "TectonicPlanetV6PreviewActor.h"

#include "DrawDebugHelpers.h"
#include "HAL/PlatformTime.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "TectonicPlanetVisualization.h"

using namespace RealtimeMesh;

namespace
{
	UMaterialInterface* ResolvePlanetMaterialV6(UMaterialInterface* PreferredMaterial)
	{
		if (PreferredMaterial)
		{
			return PreferredMaterial;
		}

		return LoadObject<UMaterialInterface>(
			nullptr,
			TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
	}

	FColor GetV6VisualizationColor(const FSample& Sample, const ETectonicMapExportMode Mode)
	{
		switch (Mode)
		{
		case ETectonicMapExportMode::PlateId:
			return TectonicPlanetVisualization::GetPlateColor(Sample.PlateId);
		case ETectonicMapExportMode::CrustType:
			return TectonicPlanetVisualization::GetCrustTypeColor(Sample.ContinentalWeight);
		case ETectonicMapExportMode::ContinentalWeight:
			return TectonicPlanetVisualization::GetContinentalWeightColor(Sample.ContinentalWeight);
		case ETectonicMapExportMode::BoundaryMask:
			return TectonicPlanetVisualization::GetBoundaryMaskColor(Sample.bIsBoundary);
		case ETectonicMapExportMode::SubductionDistance:
			return TectonicPlanetVisualization::GetSubductionDistanceColor(Sample.SubductionDistanceKm);
		case ETectonicMapExportMode::Elevation:
		default:
			return TectonicPlanetVisualization::GetElevationColor(Sample.Elevation);
		}
	}
}

ATectonicPlanetV6PreviewActor::ATectonicPlanetV6PreviewActor()
{
	PrimaryActorTick.bCanEverTick = false;

	MeshComponent = CreateDefaultSubobject<URealtimeMeshComponent>(TEXT("PlanetMeshV6"));
	MeshComponent->SetGenerateOverlapEvents(false);
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetRootComponent(MeshComponent);
}

void ATectonicPlanetV6PreviewActor::BeginPlay()
{
	Super::BeginPlay();
}

void ATectonicPlanetV6PreviewActor::BeginDestroy()
{
	Super::BeginDestroy();
}

void ATectonicPlanetV6PreviewActor::ConfigureKeptCandidateDefaults()
{
	PlanetV6.SetPeriodicSolveModeForTest(
		ETectonicPlanetV6PeriodicSolveMode::ThesisPartitionedFrontierProcessSpike,
		16);
	PlanetV6.SetSyntheticCoverageRetentionForTest(false);
	PlanetV6.SetWholeTriangleBoundaryDuplicationForTest(false);
	PlanetV6.SetExcludeMixedTrianglesForTest(false);
	PlanetV6.SetV9Phase1AuthorityForTest(true, 1);
	PlanetV6.SetV9Phase1ActiveZoneClassifierModeForTest(
		ETectonicPlanetV6ActiveZoneClassifierMode::PersistentPairLocalTightFreshAdmission);
	PlanetV6.SetV9Phase1PersistentActivePairHorizonForTest(2);
	PlanetV6.SetV9CollisionShadowForTest(true);
	PlanetV6.SetV9CollisionExecutionForTest(true);
	PlanetV6.SetV9CollisionExecutionEnhancedConsequencesForTest(true);
	PlanetV6.SetV9CollisionExecutionStructuralTransferForTest(true);
	PlanetV6.SetV9CollisionExecutionRefinedStructuralTransferForTest(true);
	PlanetV6.SetV9ThesisShapedCollisionExecutionForTest(true);
	PlanetV6.SetV9QuietInteriorContinentalRetentionForTest(true);
	PlanetV6.SetAutomaticRiftingForTest(bEnableStochasticRifting);
}

void ATectonicPlanetV6PreviewActor::Generate()
{
#if WITH_EDITOR
	const bool bIsEditorContext = GIsEditor && (!GetWorld() || !GetWorld()->IsGameWorld());
	if (bIsEditorContext) { Modify(); }
#endif

	PlanetV6.Initialize(
		SampleCount,
		PlateCount,
		RandomSeed,
		BoundaryWarpAmplitude,
		ContinentalFraction,
		PlanetRadiusKm);
	ConfigureKeptCandidateDefaults();
	bInitialized = true;

	UE_LOG(LogTemp, Log,
		TEXT("TectonicPlanetV6Preview: Generated. samples=%d plates=%d seed=%d solve_mode=ThesisPartitionedFrontierProcess cadence=16 retention=ON rifting=%s"),
		SampleCount, PlateCount, RandomSeed,
		bEnableStochasticRifting ? TEXT("ON") : TEXT("OFF"));

	UpdateStatusReadout();
	BuildMesh();

#if WITH_EDITOR
	if (bIsEditorContext) { MarkPackageDirty(); }
#endif
}

void ATectonicPlanetV6PreviewActor::Advance1Step()
{
	AdvanceSteps(1);
}

void ATectonicPlanetV6PreviewActor::Advance16Steps()
{
	AdvanceSteps(16);
}

void ATectonicPlanetV6PreviewActor::Advance100Steps()
{
	AdvanceSteps(100);
}

void ATectonicPlanetV6PreviewActor::AdvanceToNextSolve()
{
	if (!bInitialized) { return; }

	const int32 Interval = PlanetV6.ComputePeriodicSolveInterval();
	const int32 CurrentStepVal = PlanetV6.GetPlanet().CurrentStep;
	int32 StepsToNext = Interval - (CurrentStepVal % Interval);
	if (StepsToNext == 0) { StepsToNext = Interval; }
	AdvanceSteps(StepsToNext);
}

void ATectonicPlanetV6PreviewActor::AdvanceSteps(const int32 StepCount)
{
	if (StepCount <= 0 || !bInitialized) { return; }

#if WITH_EDITOR
	const bool bIsEditorContext = GIsEditor && (!GetWorld() || !GetWorld()->IsGameWorld());
	if (bIsEditorContext) { Modify(); }
#endif

	const double StartTime = FPlatformTime::Seconds();
	PlanetV6.AdvanceSteps(StepCount);
	LastAdvanceStepMs = ((FPlatformTime::Seconds() - StartTime) * 1000.0) / static_cast<double>(StepCount);

	UpdateStatusReadout();
	BuildMesh();

	UE_LOG(LogTemp, Log,
		TEXT("TectonicPlanetV6Preview: Advanced %d steps to step %d (%.1f ms/step). plates=%d caf=%.4f continental=%d solves=%d collisions=%d"),
		StepCount,
		CurrentStep,
		LastAdvanceStepMs,
		CurrentPlateCount,
		ContinentalAreaFraction,
		ContinentalSampleCount,
		PeriodicSolveCount,
		CumulativeCollisionCount);

#if WITH_EDITOR
	if (bIsEditorContext) { MarkPackageDirty(); }
#endif
}

void ATectonicPlanetV6PreviewActor::ShowElevation()
{
	VisualizationMode = ETectonicMapExportMode::Elevation;
	if (bInitialized) { BuildMesh(); }
}

void ATectonicPlanetV6PreviewActor::ShowPlateId()
{
	VisualizationMode = ETectonicMapExportMode::PlateId;
	if (bInitialized) { BuildMesh(); }
}

void ATectonicPlanetV6PreviewActor::ShowContinentalWeight()
{
	VisualizationMode = ETectonicMapExportMode::ContinentalWeight;
	if (bInitialized) { BuildMesh(); }
}

void ATectonicPlanetV6PreviewActor::ShowBoundaryMask()
{
	VisualizationMode = ETectonicMapExportMode::BoundaryMask;
	if (bInitialized) { BuildMesh(); }
}

void ATectonicPlanetV6PreviewActor::UpdateStatusReadout()
{
	const FTectonicPlanet& Planet = PlanetV6.GetPlanet();
	CurrentStep = Planet.CurrentStep;
	CurrentPlateCount = Planet.Plates.Num();
	PeriodicSolveCount = PlanetV6.GetPeriodicSolveCount();

	int32 ContinentalCount = 0;
	for (const FSample& Sample : Planet.Samples)
	{
		ContinentalCount += Sample.ContinentalWeight >= 0.5f ? 1 : 0;
	}
	ContinentalSampleCount = ContinentalCount;
	ContinentalAreaFraction = Planet.Samples.Num() > 0
		? static_cast<double>(ContinentalCount) / static_cast<double>(Planet.Samples.Num())
		: 0.0;

	if (PlanetV6.GetPeriodicSolveCount() > 0)
	{
		const FTectonicPlanetV6CollisionExecutionDiagnostic CollisionDiag =
			PlanetV6.ComputeCollisionExecutionDiagnosticForTest();
		CumulativeCollisionCount = CollisionDiag.CumulativeExecutedCollisionCount;
	}

	const FTectonicPlanetV6RiftDiagnostic RiftDiag = PlanetV6.ComputeRiftDiagnosticForTest();
	CumulativeRiftCount = RiftDiag.CumulativeRiftCount;
}

void ATectonicPlanetV6PreviewActor::BuildMesh()
{
	if (!MeshComponent) { RefreshDebugOverlays(); return; }

	URealtimeMeshSimple* RealtimeMesh = MeshComponent->InitializeRealtimeMesh<URealtimeMeshSimple>();
	if (!RealtimeMesh) { RefreshDebugOverlays(); return; }

	RealtimeMesh->Reset();

	const FTectonicPlanet& Planet = PlanetV6.GetPlanet();
	if (Planet.Samples.IsEmpty() || Planet.TriangleIndices.IsEmpty())
	{
		RefreshDebugOverlays();
		return;
	}

	FRealtimeMeshStreamSet StreamSet;
	TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder(StreamSet);
	Builder.EnableTangents();
	Builder.EnableColors();
	Builder.EnablePolyGroups();
	Builder.ReserveNumVertices(Planet.Samples.Num());
	Builder.ReserveNumTriangles(Planet.TriangleIndices.Num());

	const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);

	for (const FSample& Sample : Planet.Samples)
	{
		const FVector3d Normal = Sample.Position.GetSafeNormal();
		FVector3d Tangent = FVector3d(0.0, 0.0, 1.0) ^ Normal;
		if (Tangent.IsNearlyZero())
		{
			Tangent = FVector3d(1.0, 0.0, 0.0) ^ Normal;
		}
		Tangent.Normalize();

		double VertexRadius = RenderRadius;
		if (bDisplaceByElevation)
		{
			VertexRadius += static_cast<double>(Sample.Elevation) * VisualScale * ElevationDisplacementScale;
		}

		Builder.AddVertex(FVector3f(Normal * VertexRadius))
			.SetNormalAndTangent(FVector3f(Normal), FVector3f(Tangent))
			.SetColor(GetV6VisualizationColor(Sample, VisualizationMode));
	}

	for (const FIntVector& Triangle : Planet.TriangleIndices)
	{
		Builder.AddTriangle(
			static_cast<uint32>(Triangle.X),
			static_cast<uint32>(Triangle.Y),
			static_cast<uint32>(Triangle.Z),
			0);
	}

	RealtimeMesh->SetupMaterialSlot(0, FName(TEXT("PlanetSurface")));
	const FRealtimeMeshSectionGroupKey GroupKey =
		FRealtimeMeshSectionGroupKey::Create(0, FName(TEXT("PlanetSurface")));
	RealtimeMesh->CreateSectionGroup(GroupKey, StreamSet);
	RealtimeMesh->UpdateSectionConfig(
		FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey, 0),
		FRealtimeMeshSectionConfig(0));

	if (UMaterialInterface* SurfaceMaterial = ResolvePlanetMaterialV6(PlanetMaterial))
	{
		MeshComponent->SetMaterial(0, SurfaceMaterial);
	}

	RefreshDebugOverlays();
}

void ATectonicPlanetV6PreviewActor::RefreshDebugOverlays()
{
#if ENABLE_DRAW_DEBUG
	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);

		if (!bInitialized) { return; }

		const FTectonicPlanet& Planet = PlanetV6.GetPlanet();
		if (Planet.Samples.IsEmpty() || Planet.Plates.IsEmpty()) { return; }

		const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
		const double SurfaceOffset = RenderRadius * 1.005;

		if (bShowPlateBoundaries)
		{
			const FColor BoundaryColor = FColor::Yellow;
			for (int32 I = 0; I < Planet.SampleAdjacency.Num(); ++I)
			{
				const int32 PlateI = Planet.Samples[I].PlateId;
				for (const int32 J : Planet.SampleAdjacency[I])
				{
					if (J > I && Planet.Samples[J].PlateId != PlateI)
					{
						const FVector Start = FVector(Planet.Samples[I].Position.GetSafeNormal() * SurfaceOffset);
						const FVector End = FVector(Planet.Samples[J].Position.GetSafeNormal() * SurfaceOffset);
						DrawDebugLine(World, Start, End, BoundaryColor, true, -1.0f, 0, 1.0f);
					}
				}
			}
		}

		if (bShowPlateVelocities)
		{
			double MaxAngularSpeed = 0.0;
			for (const FPlate& Plate : Planet.Plates)
			{
				MaxAngularSpeed = FMath::Max(MaxAngularSpeed, Plate.AngularSpeed);
			}
			if (MaxAngularSpeed <= UE_DOUBLE_SMALL_NUMBER) { return; }

			for (const FPlate& Plate : Planet.Plates)
			{
				if (Plate.MemberSamples.IsEmpty()) { continue; }

				FVector3d Centroid = FVector3d::ZeroVector;
				for (const int32 SampleIndex : Plate.MemberSamples)
				{
					if (Planet.Samples.IsValidIndex(SampleIndex))
					{
						Centroid += Planet.Samples[SampleIndex].Position;
					}
				}
				Centroid = Centroid.GetSafeNormal();
				if (Centroid.IsNearlyZero()) { continue; }

				const FVector3d RotationAxis = Plate.RotationAxis.GetSafeNormal();
				const FVector3d VelocityDir = (RotationAxis ^ Centroid).GetSafeNormal();
				const double SpeedFraction = FMath::Clamp(Plate.AngularSpeed / MaxAngularSpeed, 0.0, 1.0);
				const double ArrowLength = RenderRadius * 0.3 * SpeedFraction;

				const FVector ArrowStart = FVector(Centroid * SurfaceOffset);
				const FVector ArrowEnd = FVector(Centroid * SurfaceOffset + VelocityDir * ArrowLength);
				const FColor ArrowColor = TectonicPlanetVisualization::GetPlateColor(Plate.Id);
				DrawDebugDirectionalArrow(World, ArrowStart, ArrowEnd, ArrowLength * 0.3, ArrowColor, true, -1.0f, 0, 2.0f);
			}
		}
	}
#endif
}
