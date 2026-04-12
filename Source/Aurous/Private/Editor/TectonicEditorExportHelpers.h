#pragma once

#include "CoreMinimal.h"

#if WITH_EDITOR

class AActor;
class UPrimitiveComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

namespace TectonicEditorExportHelpers
{
	struct FMeshScreenshotOverlayLine
	{
		FVector Start = FVector::ZeroVector;
		FVector End = FVector::ZeroVector;
		FColor Color = FColor::White;
		int32 Thickness = 1;
	};

	AUROUS_API bool EnsureMeshScreenshotCaptureResources(
		AActor& OwnerActor,
		UPrimitiveComponent* ShowOnlyComponent,
		int32 CaptureResolution,
		TObjectPtr<USceneCaptureComponent2D>& InOutCaptureComponent,
		TObjectPtr<UTextureRenderTarget2D>& InOutRenderTarget,
		FString& OutError);

	AUROUS_API void ReleaseMeshScreenshotCaptureResources(
		TObjectPtr<USceneCaptureComponent2D>& InOutCaptureComponent,
		TObjectPtr<UTextureRenderTarget2D>& InOutRenderTarget);

	AUROUS_API bool ExportCurrentMeshScreenshots(
		AActor& OwnerActor,
		UPrimitiveComponent* ShowOnlyComponent,
		double RenderRadius,
		const FVector& PlanetCenter,
		int32 CurrentStep,
		const FString& OutputDirectory,
		TObjectPtr<USceneCaptureComponent2D>& InOutCaptureComponent,
		TObjectPtr<UTextureRenderTarget2D>& InOutRenderTarget,
		TFunction<void(TArray<FMeshScreenshotOverlayLine>&)> BuildOverlayLines,
		FString& OutError);
}

#endif
