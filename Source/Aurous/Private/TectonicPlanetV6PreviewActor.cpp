#include "TectonicPlanetV6PreviewActor.h"

#include "DrawDebugHelpers.h"
#include "Editor/TectonicEditorExportHelpers.h"
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "TectonicMollweideExporter.h"
#include "TectonicPlanetVisualization.h"

using namespace RealtimeMesh;

namespace
{
	constexpr float EnhancedOceanDisplaySeaLevelKm = 0.0f;
	constexpr int32 EnhancedOceanDisplaySmoothingPasses = 2;

	FString BuildV6PreviewRunId(const int32 RandomSeed, const int32 SampleCount, const int32 PlateCount)
	{
		return FString::Printf(TEXT("V6Preview_S%d_N%d_P%d"), RandomSeed, SampleCount, PlateCount);
	}

	FTectonicPlanetV6KeptDiagnosticsOptions BuildV6PreviewKeptDiagnosticsOptions()
	{
		FTectonicPlanetV6KeptDiagnosticsOptions Options;
		Options.bEnableDetailedCopiedFrontierAttribution = false;
		return Options;
	}

	FString DescribeV6PreviewKeptProfile(const bool bEnableAutomaticRifting)
	{
		const FTectonicPlanetV6KeptDiagnosticsOptions DiagnosticsOptions =
			BuildV6PreviewKeptDiagnosticsOptions();
		return FString::Printf(
			TEXT("%s diagnostics=lean phase_timing=%s detailed_copied_frontier_attribution=%s"),
			*FTectonicPlanetV6::DescribeKeptV6RuntimeProfile(
				bEnableAutomaticRifting,
				true),
			DiagnosticsOptions.bEnablePhaseTiming ? TEXT("ON") : TEXT("OFF"),
			DiagnosticsOptions.bEnableDetailedCopiedFrontierAttribution ? TEXT("ON") : TEXT("OFF"));
	}

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

	bool ShouldSmoothOceanDisplaySample(const FSample& Sample)
	{
		return Sample.ContinentalWeight < 0.5f && Sample.Elevation <= EnhancedOceanDisplaySeaLevelKm;
	}

	float ComputeOceanDisplayBlendAlpha(const float RawElevationKm)
	{
		const float DepthKm = FMath::Max(-RawElevationKm, 0.0f);
		return FMath::Clamp(0.35f + 0.10f * DepthKm, 0.35f, 0.80f);
	}

	void BuildEnhancedPreviewDisplayElevations(const FTectonicPlanet& Planet, TArray<float>& OutDisplayElevations)
	{
		const int32 SampleCount = Planet.Samples.Num();
		OutDisplayElevations.SetNum(SampleCount);
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			OutDisplayElevations[SampleIndex] = Planet.Samples[SampleIndex].Elevation;
		}

		if (SampleCount == 0 || Planet.SampleAdjacency.Num() != SampleCount)
		{
			return;
		}

		TArray<float> PreviousElevations = OutDisplayElevations;
		for (int32 PassIndex = 0; PassIndex < EnhancedOceanDisplaySmoothingPasses; ++PassIndex)
		{
			for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
			{
				const FSample& Sample = Planet.Samples[SampleIndex];
				if (!ShouldSmoothOceanDisplaySample(Sample))
				{
					OutDisplayElevations[SampleIndex] = Sample.Elevation;
					continue;
				}

				float WeightedSum = PreviousElevations[SampleIndex] * 4.0f;
				float WeightSum = 4.0f;
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (!Planet.Samples.IsValidIndex(NeighborIndex))
					{
						continue;
					}

					const FSample& Neighbor = Planet.Samples[NeighborIndex];
					if (!ShouldSmoothOceanDisplaySample(Neighbor))
					{
						continue;
					}

					WeightedSum += PreviousElevations[NeighborIndex];
					WeightSum += 1.0f;
				}

				const float SmoothedElevation = WeightSum > UE_SMALL_NUMBER
					? WeightedSum / WeightSum
					: PreviousElevations[SampleIndex];
				const float BlendAlpha = ComputeOceanDisplayBlendAlpha(Sample.Elevation);
				OutDisplayElevations[SampleIndex] =
					FMath::Lerp(PreviousElevations[SampleIndex], SmoothedElevation, BlendAlpha);
			}

			PreviousElevations = OutDisplayElevations;
		}
	}

	FColor GetV6VisualizationColor(
		const FSample& Sample,
		const ETectonicMapExportMode Mode,
		const ETectonicElevationPresentationMode ElevationPresentationMode,
		const float DisplayElevationKm)
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
		case ETectonicMapExportMode::GapMask:
			return TectonicPlanetVisualization::GetGapMaskColor(false);
		case ETectonicMapExportMode::OverlapMask:
		case ETectonicMapExportMode::MaterialOverlap:
			return TectonicPlanetVisualization::GetOverlapMaskColor(false);
		case ETectonicMapExportMode::MaterialClassification:
			return TectonicPlanetVisualization::GetGapMaskColor(false);
		case ETectonicMapExportMode::SubductionDistance:
			return TectonicPlanetVisualization::GetSubductionDistanceColor(Sample.SubductionDistanceKm);
		case ETectonicMapExportMode::Elevation:
		default:
			return TectonicPlanetVisualization::GetElevationColor(DisplayElevationKm, ElevationPresentationMode);
		}
	}
}

FString ATectonicPlanetV6PreviewActor::GetRuntimeConfigSummary() const
{
	return DescribeV6PreviewKeptProfile(bEnableStochasticRifting);
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
#if WITH_EDITOR
	TectonicEditorExportHelpers::ReleaseMeshScreenshotCaptureResources(
		EditorExportCaptureComponent,
		EditorExportRenderTarget);
#endif
	Super::BeginDestroy();
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
	FTectonicPlanetV6KeptRuntimeProfileOptions RuntimeOptions;
	RuntimeOptions.bEnableAutomaticRifting = bEnableStochasticRifting;
	PlanetV6.ApplyKeptV6RuntimeProfile(RuntimeOptions);
	PlanetV6.ApplyKeptV6DiagnosticsProfile(BuildV6PreviewKeptDiagnosticsOptions());
	bInitialized = true;

	UE_LOG(LogTemp, Log,
		TEXT("TectonicPlanetV6Preview: Generated. samples=%d plates=%d seed=%d %s"),
		SampleCount, PlateCount, RandomSeed,
		*GetRuntimeConfigSummary());

	UpdateStatusReadout();
	BuildMesh();

#if WITH_EDITOR
	if (bIsEditorContext) { MarkPackageDirty(); }
#endif
}

void ATectonicPlanetV6PreviewActor::GeneratePlanet(
	const int32 InSampleCount,
	const int32 InPlateCount,
	const int32 InRandomSeed,
	const float InBoundaryWarpAmplitude,
	const float InContinentalFraction)
{
	SampleCount = InSampleCount;
	PlateCount = InPlateCount;
	RandomSeed = InRandomSeed;
	BoundaryWarpAmplitude = InBoundaryWarpAmplitude;
	ContinentalFraction = InContinentalFraction;
	LastAdvanceStepMs = 0.0;
	Generate();
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

void ATectonicPlanetV6PreviewActor::AdvancePlanetSteps(const int32 InStepCount)
{
	AdvanceSteps(InStepCount);
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
	ShowElevationRaw();
}

void ATectonicPlanetV6PreviewActor::ShowElevationRaw()
{
	ElevationPresentationMode = ETectonicElevationPresentationMode::Raw;
	VisualizationMode = ETectonicMapExportMode::Elevation;
	if (bInitialized) { BuildMesh(); }
}

void ATectonicPlanetV6PreviewActor::ShowElevationEnhanced()
{
	ElevationPresentationMode = ETectonicElevationPresentationMode::Enhanced;
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

void ATectonicPlanetV6PreviewActor::BuildMeshWithMode(const ETectonicMapExportMode Mode)
{
	VisualizationMode = Mode;
	if (bInitialized)
	{
		BuildMesh();
	}
}

void ATectonicPlanetV6PreviewActor::SetShowPlateVelocities(const bool bShow)
{
	bShowPlateVelocities = bShow;
	RefreshDebugOverlays();
}

void ATectonicPlanetV6PreviewActor::SetShowPlateBoundaries(const bool bShow)
{
	bShowPlateBoundaries = bShow;
	RefreshDebugOverlays();
}

bool ATectonicPlanetV6PreviewActor::ExportCurrentMaps(
	const ETectonicMapExportMode Mode,
	const int32 Width,
	const int32 Height,
	FString& OutDirectory,
	FString& OutError) const
{
	if (!bInitialized)
	{
		OutError = TEXT("V6 preview actor has not generated a planet yet.");
		return false;
	}

	OutDirectory = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		BuildV6PreviewRunId(RandomSeed, SampleCount, PlateCount),
		FString::Printf(TEXT("Step_%d"), PlanetV6.GetPlanet().CurrentStep));

	FTectonicMollweideExportOptions Options;
	Options.Mode = Mode;
	Options.Width = Width;
	Options.Height = Height;
	Options.OutputDirectory = OutDirectory;

	FTectonicMollweideExportStats Stats;
	if (!TectonicMollweideExporter::ExportPlanet(PlanetV6.GetPlanet(), Options, Stats, OutError))
	{
		return false;
	}

#if WITH_EDITOR
	const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
	ATectonicPlanetV6PreviewActor* MutableThis = const_cast<ATectonicPlanetV6PreviewActor*>(this);
	if (!TectonicEditorExportHelpers::ExportCurrentMeshScreenshots(
		*MutableThis,
		MeshComponent,
		RenderRadius,
		GetActorLocation(),
		PlanetV6.GetPlanet().CurrentStep,
		OutDirectory,
		MutableThis->EditorExportCaptureComponent,
		MutableThis->EditorExportRenderTarget,
		[this, RenderRadius](TArray<TectonicEditorExportHelpers::FMeshScreenshotOverlayLine>& OutLines)
		{
			if (!bInitialized)
			{
				return;
			}

			const FTectonicPlanet& Planet = PlanetV6.GetPlanet();
			if (Planet.Samples.IsEmpty() || Planet.Plates.IsEmpty())
			{
				return;
			}

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
							OutLines.Add({
								FVector(Planet.Samples[I].Position.GetSafeNormal() * SurfaceOffset),
								FVector(Planet.Samples[J].Position.GetSafeNormal() * SurfaceOffset),
								BoundaryColor,
								1 });
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
				if (MaxAngularSpeed <= UE_DOUBLE_SMALL_NUMBER)
				{
					return;
				}

				for (const FPlate& Plate : Planet.Plates)
				{
					if (Plate.MemberSamples.IsEmpty())
					{
						continue;
					}

					FVector3d Centroid = FVector3d::ZeroVector;
					for (const int32 SampleIndex : Plate.MemberSamples)
					{
						if (Planet.Samples.IsValidIndex(SampleIndex))
						{
							Centroid += Planet.Samples[SampleIndex].Position;
						}
					}
					Centroid = Centroid.GetSafeNormal();
					if (Centroid.IsNearlyZero())
					{
						continue;
					}

					const FVector3d RotationAxis = Plate.RotationAxis.GetSafeNormal();
					const FVector3d VelocityDir = (RotationAxis ^ Centroid).GetSafeNormal();
					const double SpeedFraction = FMath::Clamp(Plate.AngularSpeed / MaxAngularSpeed, 0.0, 1.0);
					const double ArrowLength = RenderRadius * 0.3 * SpeedFraction;
					const FVector ArrowStart = FVector(Centroid * SurfaceOffset);
					const FVector ArrowEnd = FVector(Centroid * SurfaceOffset + VelocityDir * ArrowLength);
					FVector3d ArrowSide = VelocityDir ^ Centroid;
					if (ArrowSide.IsNearlyZero())
					{
						ArrowSide = Centroid ^ FVector3d(0.0, 0.0, 1.0);
					}
					ArrowSide.Normalize();
					const double HeadLength = ArrowLength * 0.3;
					const double HeadWidth = ArrowLength * 0.12;
					const FVector HeadLeft = ArrowEnd - FVector(VelocityDir * HeadLength) + FVector(ArrowSide * HeadWidth);
					const FVector HeadRight = ArrowEnd - FVector(VelocityDir * HeadLength) - FVector(ArrowSide * HeadWidth);
					const FColor ArrowColor = TectonicPlanetVisualization::GetPlateColor(Plate.Id);

					OutLines.Add({ ArrowStart, ArrowEnd, ArrowColor, 2 });
					OutLines.Add({ ArrowEnd, HeadLeft, ArrowColor, 2 });
					OutLines.Add({ ArrowEnd, HeadRight, ArrowColor, 2 });
				}
			}
		},
		OutError))
	{
		return false;
	}
#endif

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[V6 Preview Export Step=%d] export_dir=%s resolved_pixels=%lld uncovered_interior=%d files=%d"),
		PlanetV6.GetPlanet().CurrentStep,
		*OutDirectory,
		static_cast<long long>(Stats.ResolvedPixelCount),
		Stats.UncoveredInteriorPixelCount,
		Stats.WrittenFiles.Num());
	return true;
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

	TArray<float> DisplayElevations;
	const bool bNeedEnhancedDisplayElevations =
		ElevationPresentationMode == ETectonicElevationPresentationMode::Enhanced &&
		(VisualizationMode == ETectonicMapExportMode::Elevation ||
			(bDisplaceByElevation && bUseEnhancedElevationForDisplacement));
	if (bNeedEnhancedDisplayElevations)
	{
		BuildEnhancedPreviewDisplayElevations(Planet, DisplayElevations);
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
			const float DisplacementElevationKm =
				(bNeedEnhancedDisplayElevations && bUseEnhancedElevationForDisplacement)
					? DisplayElevations[&Sample - Planet.Samples.GetData()]
					: Sample.Elevation;
			VertexRadius += static_cast<double>(DisplacementElevationKm) * VisualScale * ElevationDisplacementScale;
		}

		const int32 SampleIndex = &Sample - Planet.Samples.GetData();
		const float DisplayElevationKm =
			bNeedEnhancedDisplayElevations
				? DisplayElevations[SampleIndex]
				: Sample.Elevation;

		Builder.AddVertex(FVector3f(Normal * VertexRadius))
			.SetNormalAndTangent(FVector3f(Normal), FVector3f(Tangent))
			.SetColor(GetV6VisualizationColor(
				Sample,
				VisualizationMode,
				ElevationPresentationMode,
				DisplayElevationKm));
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
