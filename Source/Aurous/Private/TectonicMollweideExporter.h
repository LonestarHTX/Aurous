#pragma once

#include "CoreMinimal.h"

class FTectonicPlanet;

enum class ETectonicMollweideExportMode : uint8
{
	Elevation,
	PlateId,
	CrustType,
	All
};

struct FTectonicMollweideExportOptions
{
	ETectonicMollweideExportMode Mode = ETectonicMollweideExportMode::All;
	int32 Width = 4096;
	int32 Height = 2048;
	FString OutputPath;
};

struct FTectonicMollweideExportStats
{
	ETectonicMollweideExportMode Mode = ETectonicMollweideExportMode::All;
	int32 Width = 0;
	int32 Height = 0;
	double MinElevation = 0.0;
	double MaxElevation = 0.0;
	int64 InsideEllipsePixelCount = 0;
	int64 ContainmentFailureCount = 0;
	int64 NeighborProbeFallbackCount = 0;
	int64 ClearFallbackCount = 0;
	int64 ImageFillFallbackCount = 0;
	double ExportTimeMs = 0.0;
	TArray<FString> WrittenFiles;
};

namespace TectonicMollweideExporter
{
	bool TryParseMode(const FString& Value, ETectonicMollweideExportMode& OutMode);
	const TCHAR* GetModeName(ETectonicMollweideExportMode Mode);
	bool ExportPlanet(
		FTectonicPlanet& Planet,
		const FTectonicMollweideExportOptions& Options,
		FTectonicMollweideExportStats& OutStats,
		FString& OutError);
}
