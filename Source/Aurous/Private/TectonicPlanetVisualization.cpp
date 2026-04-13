#include "TectonicPlanetVisualization.h"

namespace
{
	uint8 LerpByte(const uint8 A, const uint8 B, const double Alpha)
	{
		return static_cast<uint8>(FMath::Clamp(
			FMath::RoundToInt(FMath::Lerp(static_cast<double>(A), static_cast<double>(B), Alpha)),
			0,
			255));
	}

	FColor LerpColor(const FColor& A, const FColor& B, const double Alpha)
	{
		const double ClampedAlpha = FMath::Clamp(Alpha, 0.0, 1.0);
		return FColor(
			LerpByte(A.R, B.R, ClampedAlpha),
			LerpByte(A.G, B.G, ClampedAlpha),
			LerpByte(A.B, B.B, ClampedAlpha),
			255);
	}

	FColor GetRawElevationColor(const double ElevationKm)
	{
		const FColor DeepOcean(7, 36, 97);
		const FColor MidOcean(36, 102, 170);
		const FColor ShallowOcean(108, 179, 214);
		const FColor CoastalGreen(76, 134, 68);
		const FColor HighlandTan(171, 145, 86);
		const FColor MountainBrown(123, 90, 60);
		const FColor Snow(245, 245, 245);

		if (ElevationKm <= -6.0)
		{
			return DeepOcean;
		}

		if (ElevationKm < -1.0)
		{
			return LerpColor(DeepOcean, MidOcean, (ElevationKm + 6.0) / 5.0);
		}

		if (ElevationKm < 0.0)
		{
			return LerpColor(MidOcean, ShallowOcean, ElevationKm + 1.0);
		}

		if (ElevationKm < 2.0)
		{
			return LerpColor(CoastalGreen, HighlandTan, ElevationKm / 2.0);
		}

		if (ElevationKm < 5.0)
		{
			return LerpColor(HighlandTan, MountainBrown, (ElevationKm - 2.0) / 3.0);
		}

		const double SnowAlpha = FMath::Clamp((ElevationKm - 5.0) / 3.0, 0.0, 1.0);
		return LerpColor(MountainBrown, Snow, SnowAlpha);
	}

	FColor GetEnhancedElevationColor(const double ElevationKm)
	{
		const FColor DeepOcean(9, 34, 90);
		const FColor MidOcean(32, 94, 160);
		const FColor ShallowOcean(95, 168, 209);
		const FColor CoastalPlain(92, 146, 82);
		const FColor ElevatedPlain(126, 165, 102);
		const FColor UplandPlain(162, 177, 120);
		const FColor Plateau(188, 175, 124);
		const FColor Highland(153, 127, 91);
		const FColor MountainBrown(121, 91, 68);
		const FColor Snow(245, 245, 245);

		if (ElevationKm <= -6.0)
		{
			return DeepOcean;
		}

		if (ElevationKm < -1.0)
		{
			return LerpColor(DeepOcean, MidOcean, (ElevationKm + 6.0) / 5.0);
		}

		if (ElevationKm < 0.0)
		{
			return LerpColor(MidOcean, ShallowOcean, ElevationKm + 1.0);
		}

		if (ElevationKm < 0.5)
		{
			return LerpColor(CoastalPlain, ElevatedPlain, ElevationKm / 0.5);
		}

		if (ElevationKm < 1.0)
		{
			return LerpColor(ElevatedPlain, UplandPlain, (ElevationKm - 0.5) / 0.5);
		}

		if (ElevationKm < 2.0)
		{
			return LerpColor(UplandPlain, Plateau, (ElevationKm - 1.0) / 1.0);
		}

		if (ElevationKm < 4.0)
		{
			return LerpColor(Plateau, Highland, (ElevationKm - 2.0) / 2.0);
		}

		if (ElevationKm < 6.5)
		{
			return LerpColor(Highland, MountainBrown, (ElevationKm - 4.0) / 2.5);
		}

		const double SnowAlpha = FMath::Clamp((ElevationKm - 6.5) / 2.0, 0.0, 1.0);
		return LerpColor(MountainBrown, Snow, SnowAlpha);
	}
}

FColor TectonicPlanetVisualization::GetElevationColor(
	const double ElevationKm,
	const ETectonicElevationPresentationMode PresentationMode)
{
	switch (PresentationMode)
	{
	case ETectonicElevationPresentationMode::Enhanced:
		return GetEnhancedElevationColor(ElevationKm);

	case ETectonicElevationPresentationMode::Raw:
	default:
		return GetRawElevationColor(ElevationKm);
	}
}

FColor TectonicPlanetVisualization::GetPlateColor(const int32 PlateId)
{
	if (PlateId < 0)
	{
		return FColor::Black;
	}

	const double HueDegrees = FMath::Fmod(static_cast<double>(PlateId) * 137.50776405003785, 360.0);
	return FLinearColor::MakeFromHSV8(
		static_cast<uint8>((HueDegrees / 360.0) * 255.0),
		195,
		235).ToFColor(true);
}

FColor TectonicPlanetVisualization::GetCrustTypeColor(const float ContinentalWeight)
{
	return (ContinentalWeight >= 0.5f) ? FColor(168, 113, 56) : FColor(13, 70, 122);
}

FColor TectonicPlanetVisualization::GetContinentalWeightColor(const float ContinentalWeight)
{
	const uint8 Value = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(255.0f * ContinentalWeight), 0, 255));
	return FColor(Value, Value, Value, 255);
}

FColor TectonicPlanetVisualization::GetSubductionDistanceColor(const float SubductionDistanceKm)
{
	if (SubductionDistanceKm < 0.0f)
	{
		return FColor(96, 96, 96);
	}

	const double Alpha = FMath::Clamp(static_cast<double>(SubductionDistanceKm) / 1803.0, 0.0, 1.0);
	return LerpColor(FColor(32, 96, 255), FColor(224, 48, 32), Alpha);
}

FColor TectonicPlanetVisualization::GetBoundaryMaskColor(const bool bIsBoundary)
{
	return bIsBoundary ? FColor::White : FColor::Black;
}

FColor TectonicPlanetVisualization::GetGapMaskColor(const bool bIsGap)
{
	return bIsGap ? FColor::White : FColor::Black;
}

FColor TectonicPlanetVisualization::GetOverlapMaskColor(const bool bIsOverlap)
{
	return bIsOverlap ? FColor::White : FColor::Black;
}

const TCHAR* TectonicPlanetVisualization::GetExportModeName(const ETectonicMapExportMode Mode)
{
	switch (Mode)
	{
	case ETectonicMapExportMode::Elevation:
		return TEXT("Elevation");
	case ETectonicMapExportMode::PlateId:
		return TEXT("PlateId");
	case ETectonicMapExportMode::CrustType:
		return TEXT("CrustType");
	case ETectonicMapExportMode::ContinentalWeight:
		return TEXT("ContinentalWeight");
	case ETectonicMapExportMode::SubductionDistance:
		return TEXT("SubductionDistance");
	case ETectonicMapExportMode::BoundaryMask:
		return TEXT("BoundaryMask");
	case ETectonicMapExportMode::GapMask:
		return TEXT("GapMask");
	case ETectonicMapExportMode::OverlapMask:
		return TEXT("OverlapMask");
	case ETectonicMapExportMode::CombinedTectonicSummary:
		return TEXT("CombinedTectonicSummary");
	case ETectonicMapExportMode::All:
	default:
		return TEXT("All");
	}
}
