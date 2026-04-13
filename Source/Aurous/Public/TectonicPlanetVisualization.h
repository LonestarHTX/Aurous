#pragma once

#include "CoreMinimal.h"
#include "TectonicPlanetVisualization.generated.h"

UENUM(BlueprintType)
enum class ETectonicMapExportMode : uint8
{
	Elevation UMETA(DisplayName = "Elevation"),
	PlateId UMETA(DisplayName = "Plate ID"),
	CrustType UMETA(DisplayName = "Crust Type"),
	ContinentalWeight UMETA(DisplayName = "Continental Weight"),
	SubductionDistance UMETA(DisplayName = "Subduction Distance"),
	BoundaryMask UMETA(DisplayName = "Boundary Mask"),
	GapMask UMETA(DisplayName = "Gap Mask"),
	OverlapMask UMETA(DisplayName = "Overlap Mask"),
	CombinedTectonicSummary UMETA(DisplayName = "Combined Tectonic Summary"),
	All UMETA(DisplayName = "All")
};

UENUM(BlueprintType)
enum class ETectonicElevationPresentationMode : uint8
{
	Raw UMETA(DisplayName = "Raw"),
	Enhanced UMETA(DisplayName = "Enhanced")
};

namespace TectonicPlanetVisualization
{
	AUROUS_API FColor GetElevationColor(
		double ElevationKm,
		ETectonicElevationPresentationMode PresentationMode = ETectonicElevationPresentationMode::Raw);
	AUROUS_API FColor GetPlateColor(int32 PlateId);
	AUROUS_API FColor GetCrustTypeColor(float ContinentalWeight);
	AUROUS_API FColor GetContinentalWeightColor(float ContinentalWeight);
	AUROUS_API FColor GetSubductionDistanceColor(float SubductionDistanceKm);
	AUROUS_API FColor GetBoundaryMaskColor(bool bIsBoundary);
	AUROUS_API FColor GetGapMaskColor(bool bIsGap);
	AUROUS_API FColor GetOverlapMaskColor(bool bIsOverlap);
	AUROUS_API const TCHAR* GetExportModeName(ETectonicMapExportMode Mode);
}
