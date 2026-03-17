#include "TectonicPlanetActor.h"

#include "Components/SceneCaptureComponent2D.h"
#include "DrawDebugHelpers.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "HAL/PlatformFileManager.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInterface.h"
#include "Misc/FileHelper.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "TectonicMollweideExporter.h"
#include "TectonicPlanetVisualization.h"

#if WITH_EDITOR
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#endif

using namespace RealtimeMesh;

namespace
{
	FString BuildTimestampedRunId(const int32 RandomSeed, const int32 SampleCount, const int32 PlateCount)
	{
		return FString::Printf(
			TEXT("%s-seed%d-samples%d-plates%d"),
			*FDateTime::UtcNow().ToString(TEXT("%Y%m%dT%H%M%SZ")),
			RandomSeed,
			SampleCount,
			PlateCount);
	}

	FString BuildEditorRunId(const int32 RandomSeed, const int32 SampleCount, const int32 PlateCount)
	{
		return FString::Printf(
			TEXT("Editor-seed%d-samples%d-plates%d"),
			RandomSeed,
			SampleCount,
			PlateCount);
	}

	UMaterialInterface* ResolvePlanetMaterial(UMaterialInterface* PreferredMaterial)
	{
		if (PreferredMaterial)
		{
			return PreferredMaterial;
		}

		return LoadObject<UMaterialInterface>(
			nullptr,
			TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
	}

#if WITH_EDITOR
	constexpr int32 PlanetScreenshotResolution = 2048;
	constexpr float PlanetScreenshotFovDegrees = 45.0f;
	constexpr double PlanetScreenshotFrameFillFraction = 0.8;

	struct FPlanetScreenshotView
	{
		const TCHAR* Name = TEXT("");
		FVector Direction = FVector::ZeroVector;
		const TCHAR* FileName = TEXT("");
	};

	bool WritePng(const FString& OutputPath, const TArray<FColor>& Pixels, const int32 Width, const int32 Height, FString& OutError)
	{
		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
		if (!ImageWrapper.IsValid())
		{
			OutError = TEXT("Failed to create PNG image wrapper.");
			return false;
		}

		if (!ImageWrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8))
		{
			OutError = TEXT("Failed to populate PNG image wrapper.");
			return false;
		}

		const TArray64<uint8>& Compressed = ImageWrapper->GetCompressed();
		if (!FFileHelper::SaveArrayToFile(Compressed, *OutputPath))
		{
			OutError = FString::Printf(TEXT("Failed to write PNG file: %s"), *OutputPath);
			return false;
		}

		return true;
	}

	double ComputePlanetCaptureDistance(const double RenderRadius, const float HorizontalFovDegrees)
	{
		const double HalfFovRadians = FMath::DegreesToRadians(static_cast<double>(HorizontalFovDegrees) * 0.5);
		const double DesiredHalfAngle = FMath::Atan(FMath::Tan(HalfFovRadians) * PlanetScreenshotFrameFillFraction);
		return RenderRadius / FMath::Max(FMath::Sin(DesiredHalfAngle), UE_DOUBLE_SMALL_NUMBER);
	}
#endif

	FColor GetVisualizationColor(const FSample& Sample, const ETectonicMapExportMode Mode)
	{
		switch (Mode)
		{
		case ETectonicMapExportMode::PlateId:
			return TectonicPlanetVisualization::GetPlateColor(Sample.PlateId);

		case ETectonicMapExportMode::CrustType:
			return TectonicPlanetVisualization::GetCrustTypeColor(Sample.ContinentalWeight);

		case ETectonicMapExportMode::ContinentalWeight:
			return TectonicPlanetVisualization::GetContinentalWeightColor(Sample.ContinentalWeight);

		case ETectonicMapExportMode::Elevation:
		case ETectonicMapExportMode::BoundaryMask:
		case ETectonicMapExportMode::GapMask:
		case ETectonicMapExportMode::OverlapMask:
		case ETectonicMapExportMode::All:
		default:
			return TectonicPlanetVisualization::GetElevationColor(Sample.Elevation);
		}
	}

	FTectonicPlanetMetrics BuildPlanetMetrics(const FTectonicPlanet& Planet)
	{
		FTectonicPlanetMetrics Metrics;
		Metrics.CurrentStep = Planet.CurrentStep;
		Metrics.SampleCount = Planet.Samples.Num();
		Metrics.TriangleCount = Planet.TriangleIndices.Num();
		Metrics.MinElevation = TNumericLimits<double>::Max();
		Metrics.MaxElevation = -TNumericLimits<double>::Max();

		if (Planet.SampleAdjacency.Num() > 0)
		{
			int64 TotalNeighborCount = 0;
			Metrics.MinNeighborCount = TNumericLimits<int32>::Max();
			for (const TArray<int32>& Neighbors : Planet.SampleAdjacency)
			{
				const int32 NeighborCount = Neighbors.Num();
				Metrics.MinNeighborCount = FMath::Min(Metrics.MinNeighborCount, NeighborCount);
				Metrics.MaxNeighborCount = FMath::Max(Metrics.MaxNeighborCount, NeighborCount);
				TotalNeighborCount += NeighborCount;
			}

			Metrics.UniqueEdgeCount = TotalNeighborCount / 2;
			Metrics.MeanNeighborCount = static_cast<double>(TotalNeighborCount) / static_cast<double>(Planet.SampleAdjacency.Num());
			if (Metrics.MinNeighborCount == TNumericLimits<int32>::Max())
			{
				Metrics.MinNeighborCount = 0;
			}
		}

		Metrics.PlateCount = Planet.Plates.Num();
		if (!Planet.Plates.IsEmpty())
		{
			int64 TotalPlateSamples = 0;
			Metrics.MinSamplesPerPlate = TNumericLimits<int32>::Max();
			TArray<int32> TerranesPerPlate;
			TerranesPerPlate.Init(0, Planet.Plates.Num());
			for (const FTerrane& Terrane : Planet.Terranes)
			{
				if (TerranesPerPlate.IsValidIndex(Terrane.PlateId))
				{
					++TerranesPerPlate[Terrane.PlateId];
				}
			}

			Metrics.MinTerranesPerPlate = TNumericLimits<int32>::Max();
			for (const FPlate& Plate : Planet.Plates)
			{
				const int32 PlateSampleCount = Plate.MemberSamples.Num();
				TotalPlateSamples += PlateSampleCount;
				Metrics.MinSamplesPerPlate = FMath::Min(Metrics.MinSamplesPerPlate, PlateSampleCount);
				Metrics.MaxSamplesPerPlate = FMath::Max(Metrics.MaxSamplesPerPlate, PlateSampleCount);

				int32 ContinentalSamplesForPlate = 0;
				for (const int32 SampleIndex : Plate.MemberSamples)
				{
					if (Planet.Samples.IsValidIndex(SampleIndex) && Planet.Samples[SampleIndex].ContinentalWeight >= 0.5f)
					{
						++ContinentalSamplesForPlate;
					}
				}

				Metrics.ContinentalSeededPlateCount += ContinentalSamplesForPlate > 0 ? 1 : 0;
				Metrics.OceanicOnlyPlateCount += ContinentalSamplesForPlate <= 0 ? 1 : 0;

				const int32 PlateTerraneCount = TerranesPerPlate.IsValidIndex(Plate.Id) ? TerranesPerPlate[Plate.Id] : 0;
				Metrics.MinTerranesPerPlate = FMath::Min(Metrics.MinTerranesPerPlate, PlateTerraneCount);
				Metrics.MaxTerranesPerPlate = FMath::Max(Metrics.MaxTerranesPerPlate, PlateTerraneCount);
			}

			Metrics.MeanSamplesPerPlate = static_cast<double>(TotalPlateSamples) / static_cast<double>(Planet.Plates.Num());
			if (Metrics.MinSamplesPerPlate == TNumericLimits<int32>::Max())
			{
				Metrics.MinSamplesPerPlate = 0;
			}
			if (Metrics.MinTerranesPerPlate == TNumericLimits<int32>::Max())
			{
				Metrics.MinTerranesPerPlate = 0;
			}
		}

		Metrics.TerraneCount = Planet.Terranes.Num();
		for (const FSample& Sample : Planet.Samples)
		{
			Metrics.ContinentalFraction += (Sample.ContinentalWeight >= 0.5f) ? 1.0 : 0.0;
			Metrics.BoundarySampleCount += Sample.bIsBoundary ? 1 : 0;
			Metrics.MeanElevation += Sample.Elevation;
			Metrics.MinElevation = FMath::Min(Metrics.MinElevation, static_cast<double>(Sample.Elevation));
			Metrics.MaxElevation = FMath::Max(Metrics.MaxElevation, static_cast<double>(Sample.Elevation));
		}

		if (!Planet.Samples.IsEmpty())
		{
			Metrics.ContinentalFraction /= static_cast<double>(Planet.Samples.Num());
			Metrics.BoundarySampleFraction = static_cast<double>(Metrics.BoundarySampleCount) / static_cast<double>(Planet.Samples.Num());
			Metrics.MeanElevation /= static_cast<double>(Planet.Samples.Num());
		}
		else
		{
			Metrics.MinElevation = 0.0;
			Metrics.MaxElevation = 0.0;
		}

		for (const FIntVector& Triangle : Planet.TriangleIndices)
		{
			const int32 PlateA = Planet.Samples.IsValidIndex(Triangle.X) ? Planet.Samples[Triangle.X].PlateId : INDEX_NONE;
			const int32 PlateB = Planet.Samples.IsValidIndex(Triangle.Y) ? Planet.Samples[Triangle.Y].PlateId : INDEX_NONE;
			const int32 PlateC = Planet.Samples.IsValidIndex(Triangle.Z) ? Planet.Samples[Triangle.Z].PlateId : INDEX_NONE;
			if (PlateA != INDEX_NONE && PlateA == PlateB && PlateB == PlateC)
			{
				++Metrics.InteriorTriangleCount;
			}
			else
			{
				++Metrics.BoundaryTriangleCount;
			}
		}

		return Metrics;
	}
}

ATectonicPlanetActor::ATectonicPlanetActor()
{
	PrimaryActorTick.bCanEverTick = false;

	MeshComponent = CreateDefaultSubobject<URealtimeMeshComponent>(TEXT("PlanetMesh"));
	MeshComponent->SetGenerateOverlapEvents(false);
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetRootComponent(MeshComponent);
}

FTectonicPlanetMetrics ATectonicPlanetActor::GetMetrics() const
{
	return BuildPlanetMetrics(Planet);
}

void ATectonicPlanetActor::GeneratePlanet(
	const int32 InSampleCount,
	const int32 InPlateCount,
	const int32 InRandomSeed,
	const float InBoundaryWarpAmplitude,
	const float InContinentalFraction)
{
#if WITH_EDITOR
	const bool bIsEditorPreviewChange = GIsEditor && (!GetWorld() || !GetWorld()->IsGameWorld());
	if (bIsEditorPreviewChange)
	{
		Modify();
	}
#endif

	SampleCount = InSampleCount;
	PlateCount = InPlateCount;
	RandomSeed = InRandomSeed;
	BoundaryWarpAmplitude = InBoundaryWarpAmplitude;
	ContinentalFraction = InContinentalFraction;
	LastAdvanceStepMs = 0.0;

	Planet.Initialize(SampleCount, PlanetRadiusKm);
	Planet.InitializePlates(PlateCount, RandomSeed, BoundaryWarpAmplitude, ContinentalFraction);
	BuildMeshWithMode(VisualizationMode);

#if WITH_EDITOR
	if (bIsEditorPreviewChange)
	{
		MarkPackageDirty();
	}
#endif
}

void ATectonicPlanetActor::AdvancePlanetSteps(const int32 InStepCount)
{
	if (InStepCount <= 0 || Planet.Plates.IsEmpty())
	{
		return;
	}

#if WITH_EDITOR
	const bool bIsEditorPreviewChange = GIsEditor && (!GetWorld() || !GetWorld()->IsGameWorld());
	if (bIsEditorPreviewChange)
	{
		Modify();
	}
#endif

	const double StartTime = FPlatformTime::Seconds();
	for (int32 StepIndex = 0; StepIndex < InStepCount; ++StepIndex)
	{
		Planet.AdvanceStep();
	}
	LastAdvanceStepMs = ((FPlatformTime::Seconds() - StartTime) * 1000.0) / static_cast<double>(InStepCount);
	BuildMeshWithMode(VisualizationMode);

#if WITH_EDITOR
	if (bIsEditorPreviewChange)
	{
		MarkPackageDirty();
	}
#endif
}

void ATectonicPlanetActor::BuildMesh()
{
	BuildMeshWithMode(VisualizationMode);
}

void ATectonicPlanetActor::SetShowPlateVelocities(const bool bShow)
{
	bShowPlateVelocities = bShow;
	RefreshDebugOverlays();
}

void ATectonicPlanetActor::SetShowPlateBoundaries(const bool bShow)
{
	bShowPlateBoundaries = bShow;
	RefreshDebugOverlays();
}

void ATectonicPlanetActor::SetShowBoundaryTypes(const bool bShow)
{
	bShowBoundaryTypes = bShow;
	RefreshDebugOverlays();
}

void ATectonicPlanetActor::BuildMeshWithMode(const ETectonicMapExportMode Mode)
{
	VisualizationMode = Mode;

	if (!MeshComponent)
	{
		RefreshDebugOverlays();
		return;
	}

	URealtimeMeshSimple* RealtimeMesh = MeshComponent->InitializeRealtimeMesh<URealtimeMeshSimple>();
	if (!RealtimeMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("TectonicPlanetActor: failed to initialize realtime mesh."));
		RefreshDebugOverlays();
		return;
	}

	RealtimeMesh->Reset();
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

		Builder.AddVertex(FVector3f(Sample.Position * RenderRadius))
			.SetNormalAndTangent(FVector3f(Normal), FVector3f(Tangent))
			.SetColor(GetVisualizationColor(Sample, VisualizationMode));
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
	const FRealtimeMeshSectionGroupKey GroupKey = FRealtimeMeshSectionGroupKey::Create(0, FName(TEXT("PlanetSurface")));
	RealtimeMesh->CreateSectionGroup(GroupKey, StreamSet);
	RealtimeMesh->UpdateSectionConfig(FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey, 0), FRealtimeMeshSectionConfig(0));

	if (UMaterialInterface* SurfaceMaterial = ResolvePlanetMaterial(PlanetMaterial))
	{
		MeshComponent->SetMaterial(0, SurfaceMaterial);
	}
	else
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("TectonicPlanetActor: no vertex-color surface material available. Assign PlanetMaterial to preview vertex colors."));
	}

	RefreshDebugOverlays();
}

void ATectonicPlanetActor::RefreshDebugOverlays()
{
#if ENABLE_DRAW_DEBUG
	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);

		if (bShowPlateVelocities)
		{
			DrawPlateVelocityArrows();
		}

		if (bShowBoundaryTypes)
		{
			DrawBoundaryTypes();
		}
		else if (bShowPlateBoundaries)
		{
			DrawPlateBoundaries();
		}
	}
#endif
}

void ATectonicPlanetActor::DrawPlateVelocityArrows()
{
#if ENABLE_DRAW_DEBUG
	UWorld* World = GetWorld();
	if (!World || Planet.Plates.IsEmpty() || Planet.Samples.IsEmpty())
	{
		return;
	}

	const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
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

		const FVector3d AngularVelocity = Plate.RotationAxis * Plate.AngularSpeed;
		const FVector3d SurfaceVelocity = AngularVelocity ^ Centroid;
		const FVector3d VelocityDirection = SurfaceVelocity.GetSafeNormal();
		if (VelocityDirection.IsNearlyZero())
		{
			continue;
		}

		double ArrowLength = RenderRadius * 0.15 * (Plate.AngularSpeed / MaxAngularSpeed);
		ArrowLength = FMath::Clamp(ArrowLength, RenderRadius * 0.03, RenderRadius * 0.25);

		const FVector Start = FVector(Centroid * RenderRadius * 1.01);
		const FVector End = Start + FVector(VelocityDirection * ArrowLength);
		DrawDebugDirectionalArrow(
			World,
			Start,
			End,
			static_cast<float>(ArrowLength * 0.15),
			TectonicPlanetVisualization::GetPlateColor(Plate.Id),
			true,
			-1.0f,
			0,
			2.0f);
	}
#endif
}

void ATectonicPlanetActor::DrawPlateBoundaries()
{
#if ENABLE_DRAW_DEBUG
	UWorld* World = GetWorld();
	if (!World || Planet.Samples.IsEmpty() || Planet.SampleAdjacency.IsEmpty())
	{
		return;
	}

	const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
	const double SurfaceOffset = RenderRadius * 1.005;
	const FColor BoundaryColor = FColor::White;

	for (int32 SampleI = 0; SampleI < Planet.SampleAdjacency.Num(); ++SampleI)
	{
		const int32 PlateI = Planet.Samples[SampleI].PlateId;
		for (const int32 SampleJ : Planet.SampleAdjacency[SampleI])
		{
			if (SampleJ > SampleI && Planet.Samples[SampleJ].PlateId != PlateI)
			{
				const FVector Start = FVector(Planet.Samples[SampleI].Position.GetSafeNormal() * SurfaceOffset);
				const FVector End = FVector(Planet.Samples[SampleJ].Position.GetSafeNormal() * SurfaceOffset);
				DrawDebugLine(World, Start, End, BoundaryColor, true, -1.0f, 0, 1.0f);
			}
		}
	}
#endif
}

void ATectonicPlanetActor::DrawBoundaryTypes()
{
#if ENABLE_DRAW_DEBUG
	UWorld* World = GetWorld();
	if (!World || Planet.Samples.IsEmpty() || Planet.SampleAdjacency.IsEmpty() || Planet.Plates.IsEmpty())
	{
		return;
	}

	const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
	const double SurfaceOffset = RenderRadius * 1.005;

	const FColor DivergentColor = FColor::Red;
	const FColor ConvergentColor = FColor::Cyan;
	const FColor TransformColor = FColor::Yellow;
	constexpr double TransformThreshold = 0.3;

	for (int32 SampleI = 0; SampleI < Planet.SampleAdjacency.Num(); ++SampleI)
	{
		const int32 PlateI = Planet.Samples[SampleI].PlateId;
		if (!Planet.Plates.IsValidIndex(PlateI))
		{
			continue;
		}

		for (const int32 SampleJ : Planet.SampleAdjacency[SampleI])
		{
			if (SampleJ <= SampleI)
			{
				continue;
			}

			const int32 PlateJ = Planet.Samples[SampleJ].PlateId;
			if (PlateJ == PlateI || !Planet.Plates.IsValidIndex(PlateJ))
			{
				continue;
			}

			const FVector3d& PosI = Planet.Samples[SampleI].Position;
			const FVector3d& PosJ = Planet.Samples[SampleJ].Position;
			const FVector3d Midpoint = (PosI + PosJ).GetSafeNormal();

			const FVector3d AngVelA = Planet.Plates[PlateI].RotationAxis * Planet.Plates[PlateI].AngularSpeed;
			const FVector3d AngVelB = Planet.Plates[PlateJ].RotationAxis * Planet.Plates[PlateJ].AngularSpeed;
			const FVector3d SurfVelA = AngVelA ^ Midpoint;
			const FVector3d SurfVelB = AngVelB ^ Midpoint;
			const FVector3d RelVel = SurfVelB - SurfVelA;

			const FVector3d Separation = (PosJ - PosI).GetSafeNormal();
			const double RelSpeed = RelVel.Size();

			FColor EdgeColor = TransformColor;
			if (RelSpeed > UE_DOUBLE_SMALL_NUMBER)
			{
				const double NormalComponent = FVector3d::DotProduct(RelVel, Separation);
				const double Ratio = NormalComponent / RelSpeed;
				if (Ratio > TransformThreshold)
				{
					EdgeColor = DivergentColor;
				}
				else if (Ratio < -TransformThreshold)
				{
					EdgeColor = ConvergentColor;
				}
			}

			const FVector Start = FVector(PosI.GetSafeNormal() * SurfaceOffset);
			const FVector End = FVector(PosJ.GetSafeNormal() * SurfaceOffset);
			DrawDebugLine(World, Start, End, EdgeColor, true, -1.0f, 0, 1.5f);
		}
	}
#endif
}

#if WITH_EDITOR
bool ATectonicPlanetActor::EnsureEditorExportCaptureResources(const int32 CaptureResolution, FString& OutError)
{
	if (!MeshComponent)
	{
		OutError = TEXT("Planet mesh component is not available for 3D export.");
		return false;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		OutError = TEXT("3D export requires a live actor in a world.");
		return false;
	}

	if (!EditorExportRenderTarget)
	{
		EditorExportRenderTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("PlanetExportRenderTarget"), RF_Transient);
		if (!EditorExportRenderTarget)
		{
			OutError = TEXT("Failed to allocate the 3D export render target.");
			return false;
		}

		EditorExportRenderTarget->ClearColor = FLinearColor::Black;
		EditorExportRenderTarget->AddressX = TA_Clamp;
		EditorExportRenderTarget->AddressY = TA_Clamp;
		EditorExportRenderTarget->bAutoGenerateMips = false;
		EditorExportRenderTarget->RenderTargetFormat = RTF_RGBA8;
	}

	if (EditorExportRenderTarget->RenderTargetFormat != RTF_RGBA8 ||
		EditorExportRenderTarget->SizeX != CaptureResolution ||
		EditorExportRenderTarget->SizeY != CaptureResolution)
	{
		EditorExportRenderTarget->RenderTargetFormat = RTF_RGBA8;
		EditorExportRenderTarget->InitAutoFormat(CaptureResolution, CaptureResolution);
	}
	else if (!EditorExportRenderTarget->GetResource())
	{
		EditorExportRenderTarget->UpdateResourceImmediate(true);
	}

	if (!EditorExportCaptureComponent)
	{
		EditorExportCaptureComponent = NewObject<USceneCaptureComponent2D>(this, TEXT("PlanetExportCapture"), RF_Transient);
		if (!EditorExportCaptureComponent)
		{
			OutError = TEXT("Failed to allocate the 3D export scene capture component.");
			return false;
		}

		EditorExportCaptureComponent->CreationMethod = EComponentCreationMethod::Instance;
		EditorExportCaptureComponent->SetupAttachment(RootComponent);
		AddOwnedComponent(EditorExportCaptureComponent);
		EditorExportCaptureComponent->RegisterComponentWithWorld(World);
	}
	else if (!EditorExportCaptureComponent->IsRegistered())
	{
		EditorExportCaptureComponent->RegisterComponentWithWorld(World);
	}

	EditorExportCaptureComponent->TextureTarget = EditorExportRenderTarget;
	EditorExportCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	EditorExportCaptureComponent->ProjectionType = ECameraProjectionMode::Perspective;
	EditorExportCaptureComponent->FOVAngle = PlanetScreenshotFovDegrees;
	EditorExportCaptureComponent->bCaptureEveryFrame = false;
	EditorExportCaptureComponent->bCaptureOnMovement = false;
	EditorExportCaptureComponent->bAlwaysPersistRenderingState = false;
	EditorExportCaptureComponent->bExcludeFromSceneTextureExtents = true;
	EditorExportCaptureComponent->bUseRayTracingIfEnabled = false;
	EditorExportCaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
	EditorExportCaptureComponent->PostProcessBlendWeight = 0.0f;
	EditorExportCaptureComponent->UnlitViewmode = ESceneCaptureUnlitViewmode::Capture;
	EditorExportCaptureComponent->ShowFlags.SetLighting(false);
	EditorExportCaptureComponent->ShowFlags.SetPostProcessing(false);
	EditorExportCaptureComponent->ClearShowOnlyComponents();
	EditorExportCaptureComponent->ShowOnlyActors.Reset();
	EditorExportCaptureComponent->ShowOnlyComponent(MeshComponent);

	return true;
}

void ATectonicPlanetActor::ReleaseEditorExportCaptureResources()
{
	if (EditorExportCaptureComponent)
	{
		EditorExportCaptureComponent->TextureTarget = nullptr;
		EditorExportCaptureComponent->DestroyComponent();
		EditorExportCaptureComponent = nullptr;
	}

	EditorExportRenderTarget = nullptr;
}

bool ATectonicPlanetActor::ExportCurrentMeshScreenshots(const FString& OutputDirectory, FString& OutError) const
{
	if (Planet.Samples.IsEmpty() || Planet.TriangleIndices.IsEmpty())
	{
		OutError = TEXT("Planet has no mesh data to export.");
		return false;
	}

	ATectonicPlanetActor* MutableThis = const_cast<ATectonicPlanetActor*>(this);
	if (!MutableThis->EnsureEditorExportCaptureResources(PlanetScreenshotResolution, OutError))
	{
		return false;
	}

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!PlatformFile.CreateDirectoryTree(*OutputDirectory))
	{
		OutError = FString::Printf(TEXT("Failed to create export directory: %s"), *OutputDirectory);
		return false;
	}

	const double RenderRadius = PlanetRadiusKm * FMath::Max(VisualScale, UE_DOUBLE_SMALL_NUMBER);
	if (RenderRadius <= UE_DOUBLE_SMALL_NUMBER)
	{
		OutError = TEXT("Planet render radius must be positive for 3D export.");
		return false;
	}

	USceneCaptureComponent2D* CaptureComponent = MutableThis->EditorExportCaptureComponent;
	UTextureRenderTarget2D* RenderTarget = MutableThis->EditorExportRenderTarget;
	if (!CaptureComponent || !RenderTarget)
	{
		OutError = TEXT("3D export capture resources are unavailable.");
		return false;
	}

	FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	if (!RenderTargetResource)
	{
		OutError = TEXT("Failed to resolve the 3D export render target resource.");
		return false;
	}

	const double CameraDistance = ComputePlanetCaptureDistance(RenderRadius, CaptureComponent->FOVAngle);
	const FPlanetScreenshotView Views[] = {
		{ TEXT("Front"), FVector(1.0, 0.0, 0.0), TEXT("3D_Front.png") },
		{ TEXT("Back"), FVector(-1.0, 0.0, 0.0), TEXT("3D_Back.png") },
		{ TEXT("Right"), FVector(0.0, 1.0, 0.0), TEXT("3D_Right.png") },
		{ TEXT("Left"), FVector(0.0, -1.0, 0.0), TEXT("3D_Left.png") },
		{ TEXT("Top"), FVector(0.0, 0.0, 1.0), TEXT("3D_Top.png") },
		{ TEXT("Oblique"), FVector(1.0, 1.0, 0.7).GetSafeNormal(), TEXT("3D_Oblique.png") }
	};

	TArray<FColor> Pixels;
	for (const FPlanetScreenshotView& View : Views)
	{
		const FVector RelativeLocation = View.Direction * static_cast<float>(CameraDistance);
		CaptureComponent->SetRelativeLocation(RelativeLocation);
		CaptureComponent->SetRelativeRotation((-View.Direction).Rotation());
		CaptureComponent->bCameraCutThisFrame = true;
		CaptureComponent->CaptureScene();

		Pixels.Reset();
		if (!RenderTargetResource->ReadPixels(Pixels))
		{
			OutError = FString::Printf(TEXT("Failed to read pixels for the %s 3D export."), View.Name);
			return false;
		}

		if (Pixels.Num() != RenderTarget->SizeX * RenderTarget->SizeY)
		{
			OutError = FString::Printf(
				TEXT("Unexpected pixel count for the %s 3D export. Expected %d, got %d."),
				View.Name,
				RenderTarget->SizeX * RenderTarget->SizeY,
				Pixels.Num());
			return false;
		}

		for (FColor& Pixel : Pixels)
		{
			Pixel.A = 255;
		}

		const FString OutputPath = FPaths::Combine(OutputDirectory, View.FileName);
		if (!WritePng(OutputPath, Pixels, RenderTarget->SizeX, RenderTarget->SizeY, OutError))
		{
			OutError = FString::Printf(TEXT("Failed to write the %s 3D export: %s"), View.Name, *OutError);
			return false;
		}
	}

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Editor Export Step=%d] wrote 3D planet screenshots to %s at %dx%d"),
		Planet.CurrentStep,
		*OutputDirectory,
		RenderTarget->SizeX,
		RenderTarget->SizeY);
	return true;
}
#endif

bool ATectonicPlanetActor::ExportCurrentMaps(
	const ETectonicMapExportMode Mode,
	const int32 Width,
	const int32 Height,
	FString& OutDirectory,
	FString& OutError) const
{
	OutDirectory = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		BuildEditorRunId(RandomSeed, SampleCount, PlateCount),
		FString::Printf(TEXT("Step_%d"), Planet.CurrentStep));

	FTectonicMollweideExportOptions Options;
	Options.Mode = Mode;
	Options.Width = Width;
	Options.Height = Height;
	Options.OutputDirectory = OutDirectory;

	FTectonicMollweideExportStats Stats;
	if (!TectonicMollweideExporter::ExportPlanet(Planet, Options, Stats, OutError))
	{
		return false;
	}

#if WITH_EDITOR
	if (!ExportCurrentMeshScreenshots(OutDirectory, OutError))
	{
		return false;
	}
#endif

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Editor Export Step=%d] export_dir=%s resolved_pixels=%lld uncovered_interior=%d files=%d"),
		Planet.CurrentStep,
		*OutDirectory,
		static_cast<long long>(Stats.ResolvedPixelCount),
		Stats.UncoveredInteriorPixelCount,
		Stats.WrittenFiles.Num());
	return true;
}

void ATectonicPlanetActor::ExportInitialMaps(const FString& RunId, const double SampleDerivedContinentalFraction) const
{
	if (!bExportMapsOnInit)
	{
		return;
	}

	FTectonicMollweideExportOptions Options;
	Options.Mode = ExportMode;
	Options.Width = ExportWidth;
	Options.Height = ExportHeight;
	Options.OutputDirectory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId, TEXT("Step_0"));

	FTectonicMollweideExportStats Stats;
	FString ExportError;
	if (!TectonicMollweideExporter::ExportPlanet(Planet, Options, Stats, ExportError))
	{
		UE_LOG(LogTemp, Warning, TEXT("TectonicPlanetActor: initial map export failed: %s"), *ExportError);
		return;
	}

	const double ContinentalFractionDelta = FMath::Abs(Stats.ContinentalPixelFraction - SampleDerivedContinentalFraction);
	ensureAlwaysMsgf(
		ContinentalFractionDelta <= 0.05,
		TEXT("Exporter continental coverage drifted by more than 5%%. sample=%.3f export=%.3f delta=%.3f"),
		SampleDerivedContinentalFraction,
		Stats.ContinentalPixelFraction,
		ContinentalFractionDelta);

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Checkpoint RunId=%s Step=0] export_dir=%s resolved_pixels=%lld export_continental_fraction=%.3f uncovered_interior=%d files=%d"),
		*RunId,
		*Options.OutputDirectory,
		static_cast<long long>(Stats.ResolvedPixelCount),
		Stats.ContinentalPixelFraction,
		Stats.UncoveredInteriorPixelCount,
		Stats.WrittenFiles.Num());

	for (const FString& WrittenFile : Stats.WrittenFiles)
	{
		UE_LOG(LogTemp, Log, TEXT("  wrote %s"), *WrittenFile);
	}
}

void ATectonicPlanetActor::BeginDestroy()
{
#if ENABLE_DRAW_DEBUG
	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);
	}
#endif

#if WITH_EDITOR
	ReleaseEditorExportCaptureResources();
#endif

	Super::BeginDestroy();
}

void ATectonicPlanetActor::BeginPlay()
{
	Super::BeginPlay();

	GeneratePlanet(SampleCount, PlateCount, RandomSeed, BoundaryWarpAmplitude, ContinentalFraction);

	const FTectonicPlanetMetrics Metrics = GetMetrics();
	const int32 ExpectedTriangleCount = (Metrics.SampleCount >= 4) ? (2 * Metrics.SampleCount - 4) : 0;
	const int64 ExpectedEdgeCount = (Metrics.SampleCount >= 3) ? (3ll * Metrics.SampleCount - 6ll) : 0ll;
	const FString RunId = BuildTimestampedRunId(RandomSeed, SampleCount, PlateCount);

	ensureMsgf(
		Metrics.TriangleCount == ExpectedTriangleCount,
		TEXT("SDT triangle count mismatch: got %d, expected %d."),
		Metrics.TriangleCount,
		ExpectedTriangleCount);
	ensureMsgf(
		Metrics.UniqueEdgeCount == ExpectedEdgeCount,
		TEXT("SDT edge count mismatch: got %lld, expected %lld."),
		static_cast<long long>(Metrics.UniqueEdgeCount),
		static_cast<long long>(ExpectedEdgeCount));

	UE_LOG(
		LogTemp,
		Log,
		TEXT("TectonicPlanetActor: initialized step0 samples=%d triangles=%d expected_triangles=%d edges=%lld expected_edges=%lld neighbor_min=%d neighbor_max=%d neighbor_mean=%.3f plates=%d plate_samples[min/max/mean]=%d/%d/%.1f continental_fraction=%.3f seeded_plates=%d oceanic_only_plates=%d terranes=%d terranes_per_plate[min/max]=%d/%d triangles[interior/boundary]=%d/%d boundary_samples=%d boundary_fraction=%.3f elevation[min/max/mean]=%.3f/%.3f/%.3f radius_km=%.1f seed=%d warp=%.2f"),
		Metrics.SampleCount,
		Metrics.TriangleCount,
		ExpectedTriangleCount,
		static_cast<long long>(Metrics.UniqueEdgeCount),
		static_cast<long long>(ExpectedEdgeCount),
		Metrics.MinNeighborCount,
		Metrics.MaxNeighborCount,
		Metrics.MeanNeighborCount,
		Metrics.PlateCount,
		Metrics.MinSamplesPerPlate,
		Metrics.MaxSamplesPerPlate,
		Metrics.MeanSamplesPerPlate,
		Metrics.ContinentalFraction,
		Metrics.ContinentalSeededPlateCount,
		Metrics.OceanicOnlyPlateCount,
		Metrics.TerraneCount,
		Metrics.MinTerranesPerPlate,
		Metrics.MaxTerranesPerPlate,
		Metrics.InteriorTriangleCount,
		Metrics.BoundaryTriangleCount,
		Metrics.BoundarySampleCount,
		Metrics.BoundarySampleFraction,
		Metrics.MinElevation,
		Metrics.MaxElevation,
		Metrics.MeanElevation,
		PlanetRadiusKm,
		RandomSeed,
		BoundaryWarpAmplitude);

	UE_LOG(
		LogTemp,
		Log,
		TEXT("[Checkpoint RunId=%s Step=0] continental_area_fraction=%.3f overlap_count=0 gap_count=0 plate_count=%d boundary_sample_fraction=%.3f elevation_min=%.3f elevation_max=%.3f elevation_mean=%.3f"),
		*RunId,
		Metrics.ContinentalFraction,
		Metrics.PlateCount,
		Metrics.BoundarySampleFraction,
		Metrics.MinElevation,
		Metrics.MaxElevation,
		Metrics.MeanElevation);

	ExportInitialMaps(RunId, Metrics.ContinentalFraction);
}
