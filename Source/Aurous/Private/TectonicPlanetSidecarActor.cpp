#include "TectonicPlanetSidecarActor.h"

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
	FString BuildSidecarCRunId(const int32 RandomSeed, const int32 SampleCount, const int32 PlateCount)
	{
		return FString::Printf(TEXT("SidecarC_S%d_N%d_P%d"), RandomSeed, SampleCount, PlateCount);
	}

	UMaterialInterface* ResolveSidecarPlanetMaterial(UMaterialInterface* PreferredMaterial)
	{
		if (PreferredMaterial)
		{
			return PreferredMaterial;
		}

		return LoadObject<UMaterialInterface>(
			nullptr,
			TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
	}

	double ComputeEffectiveRecoveryToleranceRad(const FTectonicSidecarConfig& Config)
	{
		if (Config.RecoveryToleranceRad >= 0.0)
		{
			return Config.RecoveryToleranceRad;
		}

		const double ApproxSpacingRad =
			FMath::Sqrt((4.0 * PI) / FMath::Max(1.0, static_cast<double>(Config.SampleCount)));
		return FMath::Min(0.003, 0.25 * ApproxSpacingRad);
	}

	bool IsOceanMaterialClass(const uint8 Classification)
	{
		return Classification == static_cast<uint8>(ETectonicSidecarMaterialClassification::OceanFallback) ||
			Classification == static_cast<uint8>(ETectonicSidecarMaterialClassification::DivergentOceanFill);
	}

	FColor GetSidecarVisualizationColor(
		const FTectonicPlanetSidecar& Sidecar,
		const FSample& Sample,
		const int32 SampleIndex,
		const ETectonicMapExportMode Mode,
		const ETectonicElevationPresentationMode ElevationPresentationMode)
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
			return TectonicPlanetVisualization::GetOverlapMaskColor(false);
		case ETectonicMapExportMode::MaterialClassification:
			{
				const TArray<uint8>& Classes = Sidecar.GetLastMaterialClassifications();
				const bool bOceanFill = Classes.IsValidIndex(SampleIndex) && IsOceanMaterialClass(Classes[SampleIndex]);
				return TectonicPlanetVisualization::GetGapMaskColor(bOceanFill);
			}
		case ETectonicMapExportMode::MaterialOverlap:
			{
				const TArray<int32>& Overlaps = Sidecar.GetLastMaterialOverlapCounts();
				const bool bOverlap = Overlaps.IsValidIndex(SampleIndex) && Overlaps[SampleIndex] > 1;
				return TectonicPlanetVisualization::GetOverlapMaskColor(bOverlap);
			}
		case ETectonicMapExportMode::SubductionDistance:
			return TectonicPlanetVisualization::GetSubductionDistanceColor(Sample.SubductionDistanceKm);
		case ETectonicMapExportMode::Elevation:
		case ETectonicMapExportMode::CombinedTectonicSummary:
		case ETectonicMapExportMode::All:
		default:
			return TectonicPlanetVisualization::GetElevationColor(Sample.Elevation, ElevationPresentationMode);
		}
	}

	bool ExportScalarOverlayFromInts(
		const FTectonicPlanet& Planet,
		const TArray<int32>& Values,
		const float MinValue,
		const float MaxValue,
		const FString& OutputPath,
		const int32 Width,
		const int32 Height,
		FString& OutError)
	{
		TArray<float> FloatValues;
		FloatValues.Reserve(Values.Num());
		for (const int32 Value : Values)
		{
			FloatValues.Add(static_cast<float>(Value));
		}

		return TectonicMollweideExporter::ExportScalarOverlay(
			Planet,
			FloatValues,
			MinValue,
			MaxValue,
			OutputPath,
			Width,
			Height,
			OutError);
	}

	bool ExportScalarOverlayFromFloats(
		const FTectonicPlanet& Planet,
		const TArray<float>& Values,
		const float MinValue,
		const float MaxValue,
		const FString& OutputPath,
		const int32 Width,
		const int32 Height,
		FString& OutError)
	{
		return TectonicMollweideExporter::ExportScalarOverlay(
			Planet,
			Values,
			MinValue,
			MaxValue,
			OutputPath,
			Width,
			Height,
			OutError);
	}

	bool ExportScalarOverlayFromBytes(
		const FTectonicPlanet& Planet,
		const TArray<uint8>& Values,
		const float MinValue,
		const float MaxValue,
		const FString& OutputPath,
		const int32 Width,
		const int32 Height,
		FString& OutError)
	{
		TArray<int32> IntValues;
		IntValues.Reserve(Values.Num());
		for (const uint8 Value : Values)
		{
			IntValues.Add(static_cast<int32>(Value));
		}
		return ExportScalarOverlayFromInts(Planet, IntValues, MinValue, MaxValue, OutputPath, Width, Height, OutError);
	}
}

ATectonicPlanetSidecarActor::ATectonicPlanetSidecarActor()
{
	PrimaryActorTick.bCanEverTick = false;

	MeshComponent = CreateDefaultSubobject<URealtimeMeshComponent>(TEXT("PlanetMeshSidecarC"));
	MeshComponent->SetGenerateOverlapEvents(false);
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetRootComponent(MeshComponent);
}

void ATectonicPlanetSidecarActor::BeginPlay()
{
	Super::BeginPlay();
}

void ATectonicPlanetSidecarActor::BeginDestroy()
{
#if WITH_EDITOR
	TectonicEditorExportHelpers::ReleaseMeshScreenshotCaptureResources(
		EditorExportCaptureComponent,
		EditorExportRenderTarget);
#endif
	Super::BeginDestroy();
}

FTectonicSidecarConfig ATectonicPlanetSidecarActor::BuildSidecarConfig() const
{
	FTectonicSidecarConfig Config;
	Config.SampleCount = SampleCount;
	Config.PlateCount = PlateCount;
	Config.Seed = RandomSeed;
	Config.PlanetRadiusKm = PlanetRadiusKm;
	Config.DeltaTimeMy = DeltaTimeMy;
	Config.InitialContinentalFraction = ContinentalFraction;
	Config.BoundaryWarpAmplitude = BoundaryWarpAmplitude;
	Config.MinPlateSpeedKmPerMy = FMath::Max(0.0, MinPlateSpeedKmPerMy);
	Config.MaxPlateSpeedKmPerMy = FMath::Max(Config.MinPlateSpeedKmPerMy, MaxPlateSpeedKmPerMy);
	Config.bForceZeroAngularSpeeds = bForceZeroAngularSpeeds;
	Config.ProjectionMode = ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial;
	Config.RecoveryToleranceRad = RecoveryToleranceRad;
	Config.DiagnosticOverlapContainmentScore = DiagnosticOverlapContainmentScore;
	Config.DivergenceMinKmPerMy = DivergenceMinKmPerMy;
	Config.DivergenceSpeedFraction = DivergenceSpeedFraction;
	return Config;
}

FString ATectonicPlanetSidecarActor::GetRuntimeConfigSummary() const
{
	const FTectonicSidecarConfig Config = BuildSidecarConfig();
	return FString::Printf(
		TEXT("mode=VoronoiOwnershipDecoupledMaterial dt=%.2fMy speed=%.2f-%.2fkm/My recovery=%.5frad divergence=min%.2fkm/My frac%.2f zero_motion=%s"),
		Config.DeltaTimeMy,
		Config.MinPlateSpeedKmPerMy,
		Config.MaxPlateSpeedKmPerMy,
		ComputeEffectiveRecoveryToleranceRad(Config),
		Config.DivergenceMinKmPerMy,
		Config.DivergenceSpeedFraction,
		Config.bForceZeroAngularSpeeds ? TEXT("true") : TEXT("false"));
}

void ATectonicPlanetSidecarActor::Generate()
{
#if WITH_EDITOR
	const bool bIsEditorContext = GIsEditor && (!GetWorld() || !GetWorld()->IsGameWorld());
	if (bIsEditorContext)
	{
		Modify();
	}
#endif

	const FTectonicSidecarConfig Config = BuildSidecarConfig();
	checkf(
		Config.ProjectionMode == ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial,
		TEXT("Sidecar C actor must remain on VoronoiOwnershipDecoupledMaterial projection mode."));
	Sidecar.Initialize(Config);
	bInitialized = true;
	LastAdvanceStepMs = 0.0;
	ProjectSidecar();
	UpdateStatusReadout();
	BuildMesh();

	UE_LOG(
		LogTemp,
		Log,
		TEXT("TectonicPlanetSidecarActor: Generated Prototype C. samples=%d plates=%d seed=%d %s"),
		SampleCount,
		PlateCount,
		RandomSeed,
		*GetRuntimeConfigSummary());

#if WITH_EDITOR
	if (bIsEditorContext)
	{
		MarkPackageDirty();
	}
#endif
}

void ATectonicPlanetSidecarActor::GeneratePlanet(
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
	Generate();
}

void ATectonicPlanetSidecarActor::Advance1Step()
{
	AdvanceSteps(1);
}

void ATectonicPlanetSidecarActor::Advance16Steps()
{
	AdvanceSteps(16);
}

void ATectonicPlanetSidecarActor::Advance100Steps()
{
	AdvanceSteps(100);
}

void ATectonicPlanetSidecarActor::AdvancePlanetSteps(const int32 InStepCount)
{
	AdvanceSteps(InStepCount);
}

void ATectonicPlanetSidecarActor::AdvanceSteps(const int32 StepCount)
{
	if (StepCount <= 0 || !bInitialized)
	{
		return;
	}

#if WITH_EDITOR
	const bool bIsEditorContext = GIsEditor && (!GetWorld() || !GetWorld()->IsGameWorld());
	if (bIsEditorContext)
	{
		Modify();
	}
#endif

	const double StartTime = FPlatformTime::Seconds();
	Sidecar.AdvanceSteps(StepCount);
	ProjectSidecar();
	LastAdvanceStepMs = ((FPlatformTime::Seconds() - StartTime) * 1000.0) / static_cast<double>(StepCount);

	UpdateStatusReadout();
	BuildMesh();

	UE_LOG(
		LogTemp,
		Log,
		TEXT("TectonicPlanetSidecarActor: Advanced %d steps to step %d (%.1f ms/step). boundary=%.5f owner_frag=%.5f material_mismatch=%.5f material_overlap=%.5f"),
		StepCount,
		CurrentStep,
		LastAdvanceStepMs,
		LastDiagnostics.BoundaryFraction,
		LastDiagnostics.OwnerFragmentationFraction,
		LastDiagnostics.MaterialOwnerMismatchFraction,
		LastDiagnostics.MaterialOverlapFraction);

#if WITH_EDITOR
	if (bIsEditorContext)
	{
		MarkPackageDirty();
	}
#endif
}

void ATectonicPlanetSidecarActor::ProjectSidecar()
{
	Sidecar.ProjectToPlanet(ProjectedPlanet, &LastDiagnostics);
}

void ATectonicPlanetSidecarActor::ShowElevation()
{
	VisualizationMode = ETectonicMapExportMode::Elevation;
	if (bInitialized)
	{
		BuildMesh();
	}
}

void ATectonicPlanetSidecarActor::ShowPlateId()
{
	VisualizationMode = ETectonicMapExportMode::PlateId;
	if (bInitialized)
	{
		BuildMesh();
	}
}

void ATectonicPlanetSidecarActor::ShowContinentalWeight()
{
	VisualizationMode = ETectonicMapExportMode::ContinentalWeight;
	if (bInitialized)
	{
		BuildMesh();
	}
}

void ATectonicPlanetSidecarActor::ShowBoundaryMask()
{
	VisualizationMode = ETectonicMapExportMode::BoundaryMask;
	if (bInitialized)
	{
		BuildMesh();
	}
}

void ATectonicPlanetSidecarActor::ShowMaterialClassification()
{
	VisualizationMode = ETectonicMapExportMode::MaterialClassification;
	if (bInitialized)
	{
		BuildMesh();
	}
}

void ATectonicPlanetSidecarActor::ShowMaterialOverlap()
{
	VisualizationMode = ETectonicMapExportMode::MaterialOverlap;
	if (bInitialized)
	{
		BuildMesh();
	}
}

void ATectonicPlanetSidecarActor::BuildMeshWithMode(const ETectonicMapExportMode Mode)
{
	VisualizationMode = Mode;
	if (bInitialized)
	{
		BuildMesh();
	}
}

void ATectonicPlanetSidecarActor::SetShowPlateVelocities(const bool bShow)
{
	bShowPlateVelocities = bShow;
	RefreshDebugOverlays();
}

void ATectonicPlanetSidecarActor::SetShowPlateBoundaries(const bool bShow)
{
	bShowPlateBoundaries = bShow;
	RefreshDebugOverlays();
}

bool ATectonicPlanetSidecarActor::ExportCurrentMaps(
	const ETectonicMapExportMode Mode,
	const int32 Width,
	const int32 Height,
	FString& OutDirectory,
	FString& OutError) const
{
	if (!bInitialized)
	{
		OutError = TEXT("Sidecar C actor has not generated a planet yet.");
		return false;
	}

	OutDirectory = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		BuildSidecarCRunId(RandomSeed, SampleCount, PlateCount),
		FString::Printf(TEXT("Step_%d"), ProjectedPlanet.CurrentStep));

	FTectonicMollweideExportOptions Options;
	Options.Mode = Mode;
	Options.Width = Width;
	Options.Height = Height;
	Options.OutputDirectory = OutDirectory;

	FTectonicMollweideExportStats Stats;
	if (!TectonicMollweideExporter::ExportPlanet(ProjectedPlanet, Options, Stats, OutError))
	{
		return false;
	}

	if (Mode == ETectonicMapExportMode::All)
	{
		const float SourceMax = FMath::Max(1.0f, static_cast<float>(FMath::Max(1, PlateCount)));
		if (!ExportScalarOverlayFromInts(
			ProjectedPlanet,
			Sidecar.GetLastMaterialSourcePlateIds(),
			-1.0f,
			SourceMax,
			FPaths::Combine(OutDirectory, TEXT("MaterialSource.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}

		if (!ExportScalarOverlayFromBytes(
			ProjectedPlanet,
			Sidecar.GetLastMaterialClassifications(),
			0.0f,
			static_cast<float>(static_cast<uint8>(ETectonicSidecarMaterialClassification::DivergentOceanFill)),
			FPaths::Combine(OutDirectory, TEXT("MaterialClassification.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}

		if (!ExportScalarOverlayFromBytes(
			ProjectedPlanet,
			Sidecar.GetLastMaterialOwnerMismatchFlags(),
			0.0f,
			1.0f,
			FPaths::Combine(OutDirectory, TEXT("MaterialOwnerMismatch.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}

		if (!ExportScalarOverlayFromInts(
			ProjectedPlanet,
			Sidecar.GetLastMaterialOverlapCounts(),
			0.0f,
			SourceMax,
			FPaths::Combine(OutDirectory, TEXT("MaterialOverlap.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}

		if (!ExportScalarOverlayFromBytes(
			ProjectedPlanet,
			Sidecar.GetLastDivergentBoundaryFlags(),
			0.0f,
			1.0f,
			FPaths::Combine(OutDirectory, TEXT("DivergentBoundary.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}

		const float CrustIdMax = FMath::Max(1.0f, static_cast<float>(Sidecar.GetOceanCrustStore().NextCrustId));
		if (!ExportScalarOverlayFromInts(
			ProjectedPlanet,
			Sidecar.GetLastOceanCrustIds(),
			-1.0f,
			CrustIdMax,
			FPaths::Combine(OutDirectory, TEXT("OceanCrustId.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}

		if (!ExportScalarOverlayFromFloats(
			ProjectedPlanet,
			Sidecar.GetLastOceanCrustAgesMy(),
			0.0f,
			FMath::Max(1.0f, static_cast<float>(ProjectedPlanet.CurrentStep * Sidecar.GetConfig().DeltaTimeMy)),
			FPaths::Combine(OutDirectory, TEXT("OceanCrustAge.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}

		if (!ExportScalarOverlayFromFloats(
			ProjectedPlanet,
			Sidecar.GetLastOceanCrustThicknessKm(),
			0.0f,
			10.0f,
			FPaths::Combine(OutDirectory, TEXT("OceanCrustThickness.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}

		if (!ExportScalarOverlayFromBytes(
			ProjectedPlanet,
			Sidecar.GetLastCrustEventOverlayFlags(),
			0.0f,
			1.0f,
			FPaths::Combine(OutDirectory, TEXT("CrustEventOverlay.png")),
			Width,
			Height,
			OutError))
		{
			return false;
		}
	}

#if WITH_EDITOR
	const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
	ATectonicPlanetSidecarActor* MutableThis = const_cast<ATectonicPlanetSidecarActor*>(this);
	if (!TectonicEditorExportHelpers::ExportCurrentMeshScreenshots(
		*MutableThis,
		MeshComponent,
		RenderRadius,
		GetActorLocation(),
		ProjectedPlanet.CurrentStep,
		OutDirectory,
		MutableThis->EditorExportCaptureComponent,
		MutableThis->EditorExportRenderTarget,
		[this, RenderRadius](TArray<TectonicEditorExportHelpers::FMeshScreenshotOverlayLine>& OutLines)
		{
			if (!bInitialized || ProjectedPlanet.Samples.IsEmpty())
			{
				return;
			}

			const double SurfaceOffset = RenderRadius * 1.005;
			if (bShowPlateBoundaries)
			{
				for (int32 I = 0; I < ProjectedPlanet.SampleAdjacency.Num(); ++I)
				{
					const int32 PlateI = ProjectedPlanet.Samples[I].PlateId;
					for (const int32 J : ProjectedPlanet.SampleAdjacency[I])
					{
						if (J > I && ProjectedPlanet.Samples.IsValidIndex(J) && ProjectedPlanet.Samples[J].PlateId != PlateI)
						{
							OutLines.Add({
								FVector(ProjectedPlanet.Samples[I].Position.GetSafeNormal() * SurfaceOffset),
								FVector(ProjectedPlanet.Samples[J].Position.GetSafeNormal() * SurfaceOffset),
								FColor::Yellow,
								1 });
						}
					}
				}
			}

			if (bShowPlateVelocities)
			{
				double MaxAngularSpeed = 0.0;
				for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
				{
					MaxAngularSpeed = FMath::Max(MaxAngularSpeed, Plate.AngularSpeedRadPerMy);
				}
				if (MaxAngularSpeed <= UE_DOUBLE_SMALL_NUMBER)
				{
					return;
				}

				for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
				{
					const FVector3d Center = Plate.WorldFromLocal.RotateVector(Plate.InitialCenter).GetSafeNormal();
					if (Center.IsNearlyZero())
					{
						continue;
					}

					const FVector3d VelocityDir = (Plate.RotationAxis.GetSafeNormal() ^ Center).GetSafeNormal();
					if (VelocityDir.IsNearlyZero())
					{
						continue;
					}

					const double SpeedFraction = FMath::Clamp(Plate.AngularSpeedRadPerMy / MaxAngularSpeed, 0.0, 1.0);
					const double ArrowLength = RenderRadius * 0.3 * SpeedFraction;
					const FVector ArrowStart = FVector(Center * SurfaceOffset);
					const FVector ArrowEnd = FVector(Center * SurfaceOffset + VelocityDir * ArrowLength);
					FVector3d ArrowSide = VelocityDir ^ Center;
					if (ArrowSide.IsNearlyZero())
					{
						ArrowSide = Center ^ FVector3d(0.0, 0.0, 1.0);
					}
					ArrowSide.Normalize();
					const double HeadLength = ArrowLength * 0.3;
					const double HeadWidth = ArrowLength * 0.12;
					const FColor ArrowColor = TectonicPlanetVisualization::GetPlateColor(Plate.PlateId);
					OutLines.Add({ ArrowStart, ArrowEnd, ArrowColor, 2 });
					OutLines.Add({ ArrowEnd, ArrowEnd - FVector(VelocityDir * HeadLength) + FVector(ArrowSide * HeadWidth), ArrowColor, 2 });
					OutLines.Add({ ArrowEnd, ArrowEnd - FVector(VelocityDir * HeadLength) - FVector(ArrowSide * HeadWidth), ArrowColor, 2 });
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
		TEXT("[Sidecar C Export Step=%d] export_dir=%s resolved_pixels=%lld uncovered_interior=%d files=%d boundary=%.5f owner_frag=%.5f material_mismatch=%.5f"),
		ProjectedPlanet.CurrentStep,
		*OutDirectory,
		static_cast<long long>(Stats.ResolvedPixelCount),
		Stats.UncoveredInteriorPixelCount,
		Stats.WrittenFiles.Num(),
		LastDiagnostics.BoundaryFraction,
		LastDiagnostics.OwnerFragmentationFraction,
		LastDiagnostics.MaterialOwnerMismatchFraction);
	return true;
}

void ATectonicPlanetSidecarActor::UpdateStatusReadout()
{
	CurrentStep = ProjectedPlanet.CurrentStep;
	CurrentPlateCount = ProjectedPlanet.Plates.Num();

	int32 ContinentalCount = 0;
	for (const FSample& Sample : ProjectedPlanet.Samples)
	{
		ContinentalCount += Sample.ContinentalWeight >= 0.5f ? 1 : 0;
	}

	ContinentalSampleCount = ContinentalCount;
	ContinentalAreaFraction = ProjectedPlanet.Samples.Num() > 0
		? static_cast<double>(ContinentalCount) / static_cast<double>(ProjectedPlanet.Samples.Num())
		: 0.0;
}

void ATectonicPlanetSidecarActor::BuildMesh()
{
	if (!MeshComponent)
	{
		RefreshDebugOverlays();
		return;
	}

	URealtimeMeshSimple* RealtimeMesh = MeshComponent->InitializeRealtimeMesh<URealtimeMeshSimple>();
	if (!RealtimeMesh)
	{
		RefreshDebugOverlays();
		return;
	}

	RealtimeMesh->Reset();
	if (ProjectedPlanet.Samples.IsEmpty() || ProjectedPlanet.TriangleIndices.IsEmpty())
	{
		RefreshDebugOverlays();
		return;
	}

	FRealtimeMeshStreamSet StreamSet;
	TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder(StreamSet);
	Builder.EnableTangents();
	Builder.EnableColors();
	Builder.EnablePolyGroups();
	Builder.ReserveNumVertices(ProjectedPlanet.Samples.Num());
	Builder.ReserveNumTriangles(ProjectedPlanet.TriangleIndices.Num());

	const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
	for (int32 SampleIndex = 0; SampleIndex < ProjectedPlanet.Samples.Num(); ++SampleIndex)
	{
		const FSample& Sample = ProjectedPlanet.Samples[SampleIndex];
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
			.SetColor(GetSidecarVisualizationColor(
				Sidecar,
				Sample,
				SampleIndex,
				VisualizationMode,
				ElevationPresentationMode));
	}

	for (const FIntVector& Triangle : ProjectedPlanet.TriangleIndices)
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

	if (UMaterialInterface* SurfaceMaterial = ResolveSidecarPlanetMaterial(PlanetMaterial))
	{
		MeshComponent->SetMaterial(0, SurfaceMaterial);
	}

	RefreshDebugOverlays();
}

void ATectonicPlanetSidecarActor::RefreshDebugOverlays()
{
#if ENABLE_DRAW_DEBUG
	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);

		if (!bInitialized || ProjectedPlanet.Samples.IsEmpty())
		{
			return;
		}

		const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
		const double SurfaceOffset = RenderRadius * 1.005;

		if (bShowPlateBoundaries)
		{
			for (int32 I = 0; I < ProjectedPlanet.SampleAdjacency.Num(); ++I)
			{
				const int32 PlateI = ProjectedPlanet.Samples[I].PlateId;
				for (const int32 J : ProjectedPlanet.SampleAdjacency[I])
				{
					if (J > I && ProjectedPlanet.Samples.IsValidIndex(J) && ProjectedPlanet.Samples[J].PlateId != PlateI)
					{
						DrawDebugLine(
							World,
							FVector(ProjectedPlanet.Samples[I].Position.GetSafeNormal() * SurfaceOffset),
							FVector(ProjectedPlanet.Samples[J].Position.GetSafeNormal() * SurfaceOffset),
							FColor::Yellow,
							true,
							-1.0f,
							0,
							1.0f);
					}
				}
			}
		}

		if (bShowPlateVelocities)
		{
			double MaxAngularSpeed = 0.0;
			for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
			{
				MaxAngularSpeed = FMath::Max(MaxAngularSpeed, Plate.AngularSpeedRadPerMy);
			}
			if (MaxAngularSpeed <= UE_DOUBLE_SMALL_NUMBER)
			{
				return;
			}

			for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
			{
				const FVector3d Center = Plate.WorldFromLocal.RotateVector(Plate.InitialCenter).GetSafeNormal();
				if (Center.IsNearlyZero())
				{
					continue;
				}

				const FVector3d VelocityDir = (Plate.RotationAxis.GetSafeNormal() ^ Center).GetSafeNormal();
				if (VelocityDir.IsNearlyZero())
				{
					continue;
				}

				const double SpeedFraction = FMath::Clamp(Plate.AngularSpeedRadPerMy / MaxAngularSpeed, 0.0, 1.0);
				const double ArrowLength = RenderRadius * 0.3 * SpeedFraction;
				DrawDebugDirectionalArrow(
					World,
					FVector(Center * SurfaceOffset),
					FVector(Center * SurfaceOffset + VelocityDir * ArrowLength),
					ArrowLength * 0.3,
					TectonicPlanetVisualization::GetPlateColor(Plate.PlateId),
					true,
					-1.0f,
					0,
					2.0f);
			}
		}
	}
#endif
}
