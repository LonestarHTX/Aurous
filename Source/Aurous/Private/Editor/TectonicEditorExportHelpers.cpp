#include "Editor/TectonicEditorExportHelpers.h"

#if WITH_EDITOR

#include "Components/PrimitiveComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformFileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

namespace
{
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

	bool ProjectWorldPointToPixel(
		const USceneCaptureComponent2D& CaptureComponent,
		const int32 Width,
		const int32 Height,
		const FVector& WorldPoint,
		FVector2D& OutPixel)
	{
		const FVector LocalPoint = CaptureComponent.GetComponentTransform().InverseTransformPositionNoScale(WorldPoint);
		if (LocalPoint.X <= UE_KINDA_SMALL_NUMBER)
		{
			return false;
		}

		const double AspectRatio = Height > 0 ? static_cast<double>(Width) / static_cast<double>(Height) : 1.0;
		const double TanHalfHFov = FMath::Tan(FMath::DegreesToRadians(static_cast<double>(CaptureComponent.FOVAngle) * 0.5));
		const double TanHalfVFov = TanHalfHFov / FMath::Max(AspectRatio, UE_DOUBLE_SMALL_NUMBER);
		if (TanHalfHFov <= UE_DOUBLE_SMALL_NUMBER || TanHalfVFov <= UE_DOUBLE_SMALL_NUMBER)
		{
			return false;
		}

		const double NdcX = static_cast<double>(LocalPoint.Y) / (static_cast<double>(LocalPoint.X) * TanHalfHFov);
		const double NdcY = static_cast<double>(LocalPoint.Z) / (static_cast<double>(LocalPoint.X) * TanHalfVFov);
		OutPixel.X = static_cast<float>((NdcX * 0.5 + 0.5) * static_cast<double>(Width - 1));
		OutPixel.Y = static_cast<float>((0.5 - NdcY * 0.5) * static_cast<double>(Height - 1));
		return true;
	}

	void PaintPixel(
		TArray<FColor>& Pixels,
		const int32 Width,
		const int32 Height,
		const int32 X,
		const int32 Y,
		const FColor Color)
	{
		if (X < 0 || X >= Width || Y < 0 || Y >= Height)
		{
			return;
		}

		Pixels[Y * Width + X] = Color;
	}

	void DrawProjectedLine(
		TArray<FColor>& Pixels,
		const int32 Width,
		const int32 Height,
		const FVector2D& Start,
		const FVector2D& End,
		const FColor Color,
		const int32 Thickness)
	{
		const double Dx = static_cast<double>(End.X - Start.X);
		const double Dy = static_cast<double>(End.Y - Start.Y);
		const int32 Steps = FMath::Max(1, FMath::CeilToInt(FMath::Max(FMath::Abs(Dx), FMath::Abs(Dy))));
		const int32 Radius = FMath::Max(0, Thickness - 1);

		for (int32 StepIndex = 0; StepIndex <= Steps; ++StepIndex)
		{
			const double Alpha = static_cast<double>(StepIndex) / static_cast<double>(Steps);
			const int32 PixelX = FMath::RoundToInt(Start.X + static_cast<float>(Dx * Alpha));
			const int32 PixelY = FMath::RoundToInt(Start.Y + static_cast<float>(Dy * Alpha));
			for (int32 OffsetY = -Radius; OffsetY <= Radius; ++OffsetY)
			{
				for (int32 OffsetX = -Radius; OffsetX <= Radius; ++OffsetX)
				{
					PaintPixel(Pixels, Width, Height, PixelX + OffsetX, PixelY + OffsetY, Color);
				}
			}
		}
	}

	void ApplyOverlayLinesToPixels(
		const USceneCaptureComponent2D& CaptureComponent,
		const FVector& PlanetCenter,
		TArray<FColor>& Pixels,
		const int32 Width,
		const int32 Height,
		const TArray<TectonicEditorExportHelpers::FMeshScreenshotOverlayLine>& OverlayLines)
	{
		const FVector CameraLocation = CaptureComponent.GetComponentLocation();
		const FVector CameraDirection = (CameraLocation - PlanetCenter).GetSafeNormal();

		for (const TectonicEditorExportHelpers::FMeshScreenshotOverlayLine& OverlayLine : OverlayLines)
		{
			const FVector Midpoint = (OverlayLine.Start + OverlayLine.End) * 0.5f;
			const double StartFacing = FVector::DotProduct((OverlayLine.Start - PlanetCenter).GetSafeNormal(), CameraDirection);
			const double EndFacing = FVector::DotProduct((OverlayLine.End - PlanetCenter).GetSafeNormal(), CameraDirection);
			const double MidFacing = FVector::DotProduct((Midpoint - PlanetCenter).GetSafeNormal(), CameraDirection);
			if (StartFacing <= 0.0 && EndFacing <= 0.0 && MidFacing <= 0.0)
			{
				continue;
			}

			FVector2D PixelStart;
			FVector2D PixelEnd;
			if (!ProjectWorldPointToPixel(CaptureComponent, Width, Height, OverlayLine.Start, PixelStart) ||
				!ProjectWorldPointToPixel(CaptureComponent, Width, Height, OverlayLine.End, PixelEnd))
			{
				continue;
			}

			DrawProjectedLine(
				Pixels,
				Width,
				Height,
				PixelStart,
				PixelEnd,
				OverlayLine.Color,
				FMath::Max(1, OverlayLine.Thickness));
		}
	}
}

bool TectonicEditorExportHelpers::EnsureMeshScreenshotCaptureResources(
	AActor& OwnerActor,
	UPrimitiveComponent* ShowOnlyComponent,
	const int32 CaptureResolution,
	TObjectPtr<USceneCaptureComponent2D>& InOutCaptureComponent,
	TObjectPtr<UTextureRenderTarget2D>& InOutRenderTarget,
	FString& OutError)
{
	if (!ShowOnlyComponent)
	{
		OutError = TEXT("Planet mesh component is not available for 3D export.");
		return false;
	}

	UWorld* World = OwnerActor.GetWorld();
	if (!World)
	{
		OutError = TEXT("3D export requires a live actor in a world.");
		return false;
	}

	if (!InOutRenderTarget)
	{
		InOutRenderTarget = NewObject<UTextureRenderTarget2D>(&OwnerActor, TEXT("PlanetExportRenderTarget"), RF_Transient);
		if (!InOutRenderTarget)
		{
			OutError = TEXT("Failed to allocate the 3D export render target.");
			return false;
		}

		InOutRenderTarget->ClearColor = FLinearColor::Black;
		InOutRenderTarget->AddressX = TA_Clamp;
		InOutRenderTarget->AddressY = TA_Clamp;
		InOutRenderTarget->bAutoGenerateMips = false;
		InOutRenderTarget->RenderTargetFormat = RTF_RGBA8;
	}

	if (InOutRenderTarget->RenderTargetFormat != RTF_RGBA8 ||
		InOutRenderTarget->SizeX != CaptureResolution ||
		InOutRenderTarget->SizeY != CaptureResolution)
	{
		InOutRenderTarget->RenderTargetFormat = RTF_RGBA8;
		InOutRenderTarget->InitAutoFormat(CaptureResolution, CaptureResolution);
	}
	else if (!InOutRenderTarget->GetResource())
	{
		InOutRenderTarget->UpdateResourceImmediate(true);
	}

	if (!InOutCaptureComponent)
	{
		InOutCaptureComponent = NewObject<USceneCaptureComponent2D>(&OwnerActor, TEXT("PlanetExportCapture"), RF_Transient);
		if (!InOutCaptureComponent)
		{
			OutError = TEXT("Failed to allocate the 3D export scene capture component.");
			return false;
		}

		InOutCaptureComponent->CreationMethod = EComponentCreationMethod::Instance;
		if (USceneComponent* RootComponent = OwnerActor.GetRootComponent())
		{
			InOutCaptureComponent->SetupAttachment(RootComponent);
		}
		OwnerActor.AddOwnedComponent(InOutCaptureComponent);
		InOutCaptureComponent->RegisterComponentWithWorld(World);
	}
	else if (!InOutCaptureComponent->IsRegistered())
	{
		InOutCaptureComponent->RegisterComponentWithWorld(World);
	}

	InOutCaptureComponent->TextureTarget = InOutRenderTarget;
	InOutCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	InOutCaptureComponent->ProjectionType = ECameraProjectionMode::Perspective;
	InOutCaptureComponent->FOVAngle = PlanetScreenshotFovDegrees;
	InOutCaptureComponent->bCaptureEveryFrame = false;
	InOutCaptureComponent->bCaptureOnMovement = false;
	InOutCaptureComponent->bAlwaysPersistRenderingState = false;
	InOutCaptureComponent->bExcludeFromSceneTextureExtents = true;
	InOutCaptureComponent->bUseRayTracingIfEnabled = false;
	InOutCaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
	InOutCaptureComponent->PostProcessBlendWeight = 0.0f;
	InOutCaptureComponent->UnlitViewmode = ESceneCaptureUnlitViewmode::Capture;
	InOutCaptureComponent->ShowFlags.SetLighting(false);
	InOutCaptureComponent->ShowFlags.SetPostProcessing(false);
	InOutCaptureComponent->ClearShowOnlyComponents();
	InOutCaptureComponent->ShowOnlyActors.Reset();
	InOutCaptureComponent->ShowOnlyComponent(ShowOnlyComponent);

	return true;
}

void TectonicEditorExportHelpers::ReleaseMeshScreenshotCaptureResources(
	TObjectPtr<USceneCaptureComponent2D>& InOutCaptureComponent,
	TObjectPtr<UTextureRenderTarget2D>& InOutRenderTarget)
{
	if (InOutCaptureComponent)
	{
		InOutCaptureComponent->TextureTarget = nullptr;
		InOutCaptureComponent->DestroyComponent();
		InOutCaptureComponent = nullptr;
	}

	InOutRenderTarget = nullptr;
}

bool TectonicEditorExportHelpers::ExportCurrentMeshScreenshots(
	AActor& OwnerActor,
	UPrimitiveComponent* ShowOnlyComponent,
	const double RenderRadius,
	const FVector& PlanetCenter,
	const int32 CurrentStep,
	const FString& OutputDirectory,
	TObjectPtr<USceneCaptureComponent2D>& InOutCaptureComponent,
	TObjectPtr<UTextureRenderTarget2D>& InOutRenderTarget,
	TFunction<void(TArray<FMeshScreenshotOverlayLine>&)> BuildOverlayLines,
	FString& OutError)
{
	if (RenderRadius <= UE_DOUBLE_SMALL_NUMBER)
	{
		OutError = TEXT("Planet render radius must be positive for 3D export.");
		return false;
	}

	if (!EnsureMeshScreenshotCaptureResources(
		OwnerActor,
		ShowOnlyComponent,
		PlanetScreenshotResolution,
		InOutCaptureComponent,
		InOutRenderTarget,
		OutError))
	{
		return false;
	}

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!PlatformFile.CreateDirectoryTree(*OutputDirectory))
	{
		OutError = FString::Printf(TEXT("Failed to create export directory: %s"), *OutputDirectory);
		return false;
	}

	USceneCaptureComponent2D* CaptureComponent = InOutCaptureComponent.Get();
	UTextureRenderTarget2D* RenderTarget = InOutRenderTarget.Get();
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

	TArray<FMeshScreenshotOverlayLine> OverlayLines;
	if (BuildOverlayLines)
	{
		BuildOverlayLines(OverlayLines);
	}

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

		if (!OverlayLines.IsEmpty())
		{
			ApplyOverlayLinesToPixels(
				*CaptureComponent,
				PlanetCenter,
				Pixels,
				RenderTarget->SizeX,
				RenderTarget->SizeY,
				OverlayLines);
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
		CurrentStep,
		*OutputDirectory,
		RenderTarget->SizeX,
		RenderTarget->SizeY);
	return true;
}

#endif
