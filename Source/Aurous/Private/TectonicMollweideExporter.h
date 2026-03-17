#pragma once

#include "CoreMinimal.h"
#include "TectonicPlanetVisualization.h"

struct FTectonicPlanet;

struct FTectonicMollweideExportOptions
{
	ETectonicMapExportMode Mode = ETectonicMapExportMode::All;
	int32 Width = 4096;
	int32 Height = 2048;
	FString OutputDirectory;
};

struct FTectonicMollweideExportStats
{
	ETectonicMapExportMode Mode = ETectonicMapExportMode::All;
	int32 Width = 0;
	int32 Height = 0;
	int64 ResolvedPixelCount = 0;
	double ContinentalPixelFraction = 0.0;
	int32 PreGapFillUncoveredInteriorPixelCount = 0;
	int32 UncoveredInteriorPixelCount = 0;
	TArray<FString> WrittenFiles;
};

namespace TectonicMollweideExporter
{
	AUROUS_API bool ExportPlanet(
		const FTectonicPlanet& Planet,
		const FTectonicMollweideExportOptions& Options,
		FTectonicMollweideExportStats& OutStats,
		FString& OutError);
}
