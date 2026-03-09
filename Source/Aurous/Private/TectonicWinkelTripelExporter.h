#pragma once

#include "CoreMinimal.h"

class FTectonicPlanet;

enum class ETectonicWinkelTripelExportMode : uint8
{
	Elevation,
	PlateId,
	CrustType,
	All
};

struct FTectonicWinkelTripelExportOptions
{
	ETectonicWinkelTripelExportMode Mode = ETectonicWinkelTripelExportMode::All;
	int32 Width = 4096;
	int32 Height = 2048;
	FString OutputPath;
};

struct FTectonicWinkelTripelExportStats
{
	ETectonicWinkelTripelExportMode Mode = ETectonicWinkelTripelExportMode::All;
	int32 Width = 0;
	int32 Height = 0;
	double MinElevation = 0.0;
	double MaxElevation = 0.0;
	int64 InsideProjectionPixelCount = 0;
	int64 ContainmentFailureCount = 0;
	int64 NeighborProbeFallbackCount = 0;
	int64 ClearFallbackCount = 0;
	int64 ImageFillFallbackCount = 0;
	double ExportTimeMs = 0.0;
	TArray<FString> WrittenFiles;
};

namespace TectonicWinkelTripelExporter
{
	bool TryParseMode(const FString& Value, ETectonicWinkelTripelExportMode& OutMode);
	const TCHAR* GetModeName(ETectonicWinkelTripelExportMode Mode);
	bool ExportPlanet(
		FTectonicPlanet& Planet,
		const FTectonicWinkelTripelExportOptions& Options,
		FTectonicWinkelTripelExportStats& OutStats,
		FString& OutError);
}
