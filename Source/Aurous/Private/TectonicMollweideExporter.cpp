#include "TectonicMollweideExporter.h"

#include "HAL/PlatformFileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "TectonicPlanet.h"
#include "TectonicPlanetVisualization.h"

#if WITH_EDITOR
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#endif

namespace
{
	constexpr double MollweideSqrtTwo = 1.4142135623730950488;
	constexpr double MollweideHalfWidth = 2.0 * MollweideSqrtTwo;
	constexpr double MollweideHalfHeight = MollweideSqrtTwo;
	constexpr double RasterEpsilon = 1.0e-8;
	constexpr int32 EnhancedOceanDisplaySmoothingPasses = 2;

	struct FProjectedSample
	{
		FVector2d Pixel = FVector2d::ZeroVector;
	};

	struct FRasterBuffers
	{
		TArray<float> Elevation;
		TArray<float> ContinentalWeight;
		TArray<float> SubductionDistanceKm;
		TArray<int32> PlateId;
		TArray<uint8> BoundaryMask;
		TArray<uint8> Coverage;
	};

	bool ShouldExportMode(const ETectonicMapExportMode RequestedMode, const ETectonicMapExportMode CandidateMode)
	{
		return RequestedMode == ETectonicMapExportMode::All || RequestedMode == CandidateMode;
	}

	bool ShouldSmoothOceanDisplayPixel(const FRasterBuffers& Buffers, const int32 PixelIndex)
	{
		return Buffers.ContinentalWeight[PixelIndex] < 0.5f && Buffers.Elevation[PixelIndex] <= 0.0f;
	}

	float ComputeOceanDisplayBlendAlpha(const float RawElevationKm)
	{
		const float DepthKm = FMath::Max(-RawElevationKm, 0.0f);
		return FMath::Clamp(0.35f + 0.10f * DepthKm, 0.35f, 0.80f);
	}

	double SolveMollweideTheta(const double Latitude)
	{
		if (FMath::Abs(FMath::Abs(Latitude) - HALF_PI) < 1.0e-10)
		{
			return Latitude;
		}

		double Theta = Latitude;
		for (int32 Iteration = 0; Iteration < 8; ++Iteration)
		{
			const double TwoTheta = 2.0 * Theta;
			const double Function = TwoTheta + FMath::Sin(TwoTheta) - PI * FMath::Sin(Latitude);
			const double Derivative = 2.0 + 2.0 * FMath::Cos(TwoTheta);
			if (FMath::Abs(Derivative) < UE_DOUBLE_SMALL_NUMBER)
			{
				break;
			}

			Theta -= Function / Derivative;
		}

		return FMath::Clamp(Theta, -HALF_PI, HALF_PI);
	}

	FVector2d ProjectMollweidePixel(const FVector3d& UnitDirection, const int32 Width, const int32 Height)
	{
		const double Longitude = FMath::Atan2(UnitDirection.Y, UnitDirection.X);
		const double Latitude = FMath::Asin(FMath::Clamp(UnitDirection.Z, -1.0, 1.0));
		const double Theta = SolveMollweideTheta(Latitude);
		const double ProjectionX = (2.0 * MollweideSqrtTwo / PI) * Longitude * FMath::Cos(Theta);
		const double ProjectionY = MollweideSqrtTwo * FMath::Sin(Theta);
		double U = (ProjectionX + MollweideHalfWidth) / (2.0 * MollweideHalfWidth);
		if (U >= 1.0)
		{
			U -= 1.0;
		}
		else if (U < 0.0)
		{
			U += 1.0;
		}

		const double V = (MollweideHalfHeight - ProjectionY) / (2.0 * MollweideHalfHeight);
		return FVector2d(U * static_cast<double>(Width), V * static_cast<double>(Height));
	}

	double EdgeFunction(const FVector2d& A, const FVector2d& B, const FVector2d& P)
	{
		return (P.X - A.X) * (B.Y - A.Y) - (P.Y - A.Y) * (B.X - A.X);
	}

	void UnwrapTriangleX(FVector2d& A, FVector2d& B, FVector2d& C, const double Width)
	{
		double MinX = FMath::Min3(A.X, B.X, C.X);
		double MaxX = FMath::Max3(A.X, B.X, C.X);
		if ((MaxX - MinX) <= (Width * 0.5))
		{
			return;
		}

		auto ShiftIfNeeded = [Width](FVector2d& Point)
		{
			if (Point.X < Width * 0.5)
			{
				Point.X += Width;
			}
		};

		ShiftIfNeeded(A);
		ShiftIfNeeded(B);
		ShiftIfNeeded(C);
	}

	int32 WrapPixelX(const int32 X, const int32 Width)
	{
		int32 WrappedX = X % Width;
		if (WrappedX < 0)
		{
			WrappedX += Width;
		}
		return WrappedX;
	}

	bool IsInsideMollweideEllipse(const int32 PixelX, const int32 PixelY, const int32 Width, const int32 Height)
	{
		const double ProjectionX =
			((static_cast<double>(PixelX) + 0.5) / static_cast<double>(Width)) * (2.0 * MollweideHalfWidth) - MollweideHalfWidth;
		const double ProjectionY =
			MollweideHalfHeight - ((static_cast<double>(PixelY) + 0.5) / static_cast<double>(Height)) * (2.0 * MollweideHalfHeight);
		const double NormalizedX = ProjectionX / MollweideHalfWidth;
		const double NormalizedY = ProjectionY / MollweideHalfHeight;
		return (NormalizedX * NormalizedX + NormalizedY * NormalizedY) <= (1.0 + RasterEpsilon);
	}

	int32 CountUncoveredInteriorPixels(const FRasterBuffers& Buffers, const int32 Width, const int32 Height)
	{
		int32 UncoveredInteriorCount = 0;
		for (int32 PixelY = 0; PixelY < Height; ++PixelY)
		{
			for (int32 PixelX = 0; PixelX < Width; ++PixelX)
			{
				if (!IsInsideMollweideEllipse(PixelX, PixelY, Width, Height))
				{
					continue;
				}

				const int32 PixelIndex = PixelY * Width + PixelX;
				if (Buffers.Coverage[PixelIndex] == 0)
				{
					++UncoveredInteriorCount;
				}
			}
		}

		return UncoveredInteriorCount;
	}

	void FillRasterGaps(const FTectonicMollweideExportOptions& Options, FRasterBuffers& Buffers)
	{
		constexpr int32 MaxGapFillIterations = 16;
		int32 PreviousUncoveredInteriorCount = CountUncoveredInteriorPixels(Buffers, Options.Width, Options.Height);

		for (int32 Iteration = 0; Iteration < MaxGapFillIterations; ++Iteration)
		{
			FRasterBuffers PreviousBuffers = Buffers;
			int32 FilledPixelCountThisIteration = 0;

			for (int32 PixelY = 0; PixelY < Options.Height; ++PixelY)
			{
				for (int32 PixelX = 0; PixelX < Options.Width; ++PixelX)
				{
					if (!IsInsideMollweideEllipse(PixelX, PixelY, Options.Width, Options.Height))
					{
						continue;
					}

					const int32 PixelIndex = PixelY * Options.Width + PixelX;
					if (PreviousBuffers.Coverage[PixelIndex] != 0)
					{
						continue;
					}

					int32 BestNeighborIndex = INDEX_NONE;
					for (int32 OffsetY = -1; OffsetY <= 1; ++OffsetY)
					{
						const int32 NeighborY = PixelY + OffsetY;
						if (NeighborY < 0 || NeighborY >= Options.Height)
						{
							continue;
						}

						for (int32 OffsetX = -1; OffsetX <= 1; ++OffsetX)
						{
							if (OffsetX == 0 && OffsetY == 0)
							{
								continue;
							}

							const int32 NeighborX = WrapPixelX(PixelX + OffsetX, Options.Width);
							const int32 NeighborIndex = NeighborY * Options.Width + NeighborX;
							if (PreviousBuffers.Coverage[NeighborIndex] == 0)
							{
								continue;
							}

							if (BestNeighborIndex == INDEX_NONE || NeighborIndex < BestNeighborIndex)
							{
								BestNeighborIndex = NeighborIndex;
							}
						}
					}

					if (BestNeighborIndex == INDEX_NONE)
					{
						continue;
					}

					Buffers.Elevation[PixelIndex] = PreviousBuffers.Elevation[BestNeighborIndex];
					Buffers.ContinentalWeight[PixelIndex] = PreviousBuffers.ContinentalWeight[BestNeighborIndex];
					Buffers.SubductionDistanceKm[PixelIndex] = PreviousBuffers.SubductionDistanceKm[BestNeighborIndex];
					Buffers.PlateId[PixelIndex] = PreviousBuffers.PlateId[BestNeighborIndex];
					Buffers.BoundaryMask[PixelIndex] = PreviousBuffers.BoundaryMask[BestNeighborIndex];
					Buffers.Coverage[PixelIndex] = 1;
					++FilledPixelCountThisIteration;
				}
			}

			if (FilledPixelCountThisIteration == 0)
			{
				break;
			}

			const int32 CurrentUncoveredInteriorCount = CountUncoveredInteriorPixels(Buffers, Options.Width, Options.Height);
			if (CurrentUncoveredInteriorCount <= 0 || CurrentUncoveredInteriorCount >= PreviousUncoveredInteriorCount)
			{
				break;
			}

			PreviousUncoveredInteriorCount = CurrentUncoveredInteriorCount;
		}
	}

	bool WritePng(const FString& OutputPath, const TArray<FColor>& Pixels, const int32 Width, const int32 Height, FString& OutError)
	{
#if WITH_EDITOR
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
#else
		OutError = TEXT("Mollweide export is only available in editor builds.");
		return false;
#endif
	}

	void BuildRasterBuffers(
		const FTectonicPlanet& Planet,
		const FTectonicMollweideExportOptions& Options,
		FRasterBuffers& OutBuffers,
		int32& OutPreGapFillUncoveredInteriorPixelCount,
		int32& OutPostGapFillUncoveredInteriorPixelCount)
	{
		const int32 PixelCount = Options.Width * Options.Height;
		OutBuffers.Elevation.Init(0.0f, PixelCount);
		OutBuffers.ContinentalWeight.Init(0.0f, PixelCount);
		OutBuffers.SubductionDistanceKm.Init(-1.0f, PixelCount);
		OutBuffers.PlateId.Init(INDEX_NONE, PixelCount);
		OutBuffers.BoundaryMask.Init(0, PixelCount);
		OutBuffers.Coverage.Init(0, PixelCount);

		TArray<FProjectedSample> ProjectedSamples;
		ProjectedSamples.SetNum(Planet.Samples.Num());
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			ProjectedSamples[SampleIndex].Pixel = ProjectMollweidePixel(Planet.Samples[SampleIndex].Position, Options.Width, Options.Height);
		}

		for (const FIntVector& Triangle : Planet.TriangleIndices)
		{
			if (!Planet.Samples.IsValidIndex(Triangle.X) || !Planet.Samples.IsValidIndex(Triangle.Y) || !Planet.Samples.IsValidIndex(Triangle.Z))
			{
				continue;
			}

			FVector2d P0 = ProjectedSamples[Triangle.X].Pixel;
			FVector2d P1 = ProjectedSamples[Triangle.Y].Pixel;
			FVector2d P2 = ProjectedSamples[Triangle.Z].Pixel;
			UnwrapTriangleX(P0, P1, P2, static_cast<double>(Options.Width));

			const double TwiceArea = EdgeFunction(P0, P1, P2);
			if (FMath::Abs(TwiceArea) <= RasterEpsilon)
			{
				continue;
			}

			const int32 MinPixelX = FMath::FloorToInt(FMath::Min3(P0.X, P1.X, P2.X));
			const int32 MaxPixelX = FMath::CeilToInt(FMath::Max3(P0.X, P1.X, P2.X));
			const double ExpectedEdgePx = static_cast<double>(Options.Width) * FMath::Sqrt(4.0 * PI / FMath::Max(Planet.Samples.Num(), 1)) / PI;
			const int32 MaxTriangleWidthPx = FMath::Max(FMath::CeilToInt(ExpectedEdgePx * 30.0), 50);
			if ((MaxPixelX - MinPixelX) > MaxTriangleWidthPx)
			{
				continue;
			}

			const int32 MinPixelY = FMath::Clamp(FMath::FloorToInt(FMath::Min3(P0.Y, P1.Y, P2.Y)), 0, Options.Height - 1);
			const int32 MaxPixelY = FMath::Clamp(FMath::CeilToInt(FMath::Max3(P0.Y, P1.Y, P2.Y)), 0, Options.Height - 1);
			if (MinPixelY > MaxPixelY)
			{
				continue;
			}

			const FSample& Sample0 = Planet.Samples[Triangle.X];
			const FSample& Sample1 = Planet.Samples[Triangle.Y];
			const FSample& Sample2 = Planet.Samples[Triangle.Z];

			for (int32 PixelY = MinPixelY; PixelY <= MaxPixelY; ++PixelY)
			{
				for (int32 PixelX = MinPixelX; PixelX <= MaxPixelX; ++PixelX)
				{
					const FVector2d PixelCenter(static_cast<double>(PixelX) + 0.5, static_cast<double>(PixelY) + 0.5);
					const double W0 = EdgeFunction(P1, P2, PixelCenter) / TwiceArea;
					const double W1 = EdgeFunction(P2, P0, PixelCenter) / TwiceArea;
					const double W2 = EdgeFunction(P0, P1, PixelCenter) / TwiceArea;
					if (W0 < -RasterEpsilon || W1 < -RasterEpsilon || W2 < -RasterEpsilon)
					{
						continue;
					}

					const int32 WrappedX = WrapPixelX(PixelX, Options.Width);
					const int32 BufferIndex = PixelY * Options.Width + WrappedX;
					const double Elevation = W0 * Sample0.Elevation + W1 * Sample1.Elevation + W2 * Sample2.Elevation;
					const double ContinentalWeight = W0 * Sample0.ContinentalWeight + W1 * Sample1.ContinentalWeight + W2 * Sample2.ContinentalWeight;
					double SubductionDistanceWeightSum = 0.0;
					double WeightedSubductionDistanceKm = 0.0;
					if (Sample0.SubductionDistanceKm >= 0.0f)
					{
						WeightedSubductionDistanceKm += W0 * Sample0.SubductionDistanceKm;
						SubductionDistanceWeightSum += W0;
					}
					if (Sample1.SubductionDistanceKm >= 0.0f)
					{
						WeightedSubductionDistanceKm += W1 * Sample1.SubductionDistanceKm;
						SubductionDistanceWeightSum += W1;
					}
					if (Sample2.SubductionDistanceKm >= 0.0f)
					{
						WeightedSubductionDistanceKm += W2 * Sample2.SubductionDistanceKm;
						SubductionDistanceWeightSum += W2;
					}

					OutBuffers.Elevation[BufferIndex] = static_cast<float>(Elevation);
					OutBuffers.ContinentalWeight[BufferIndex] = static_cast<float>(ContinentalWeight);
					OutBuffers.SubductionDistanceKm[BufferIndex] =
						SubductionDistanceWeightSum > RasterEpsilon
							? static_cast<float>(WeightedSubductionDistanceKm / SubductionDistanceWeightSum)
							: -1.0f;

					int32 DominantPlateId = Sample0.PlateId;
					double DominantWeight = W0;
					if (W1 > DominantWeight)
					{
						DominantWeight = W1;
						DominantPlateId = Sample1.PlateId;
					}
					if (W2 > DominantWeight)
					{
						DominantPlateId = Sample2.PlateId;
					}

					OutBuffers.PlateId[BufferIndex] = DominantPlateId;
					OutBuffers.BoundaryMask[BufferIndex] =
						(Sample0.bIsBoundary && W0 >= W1 && W0 >= W2) ||
						(Sample1.bIsBoundary && W1 >= W0 && W1 >= W2) ||
						(Sample2.bIsBoundary && W2 >= W0 && W2 >= W1)
						? 1
						: 0;
					OutBuffers.Coverage[BufferIndex] = 1;
				}
			}
		}

		OutPreGapFillUncoveredInteriorPixelCount = CountUncoveredInteriorPixels(OutBuffers, Options.Width, Options.Height);
		FillRasterGaps(Options, OutBuffers);
		OutPostGapFillUncoveredInteriorPixelCount = CountUncoveredInteriorPixels(OutBuffers, Options.Width, Options.Height);
	}

	void BuildImageForMode(
		const FRasterBuffers& Buffers,
		const FTectonicMollweideExportOptions& Options,
		const ETectonicMapExportMode Mode,
		TArray<FColor>& OutPixels)
	{
		const int32 PixelCount = Options.Width * Options.Height;
		OutPixels.SetNum(PixelCount);

		for (int32 PixelIndex = 0; PixelIndex < PixelCount; ++PixelIndex)
		{
			const int32 PixelX = PixelIndex % Options.Width;
			const int32 PixelY = PixelIndex / Options.Width;
			if (Buffers.Coverage[PixelIndex] == 0 || !IsInsideMollweideEllipse(PixelX, PixelY, Options.Width, Options.Height))
			{
				OutPixels[PixelIndex] = FColor::Black;
				continue;
			}

			switch (Mode)
			{
			case ETectonicMapExportMode::Elevation:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetElevationColor(Buffers.Elevation[PixelIndex]);
				break;

			case ETectonicMapExportMode::PlateId:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetPlateColor(Buffers.PlateId[PixelIndex]);
				break;

			case ETectonicMapExportMode::CrustType:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetCrustTypeColor(Buffers.ContinentalWeight[PixelIndex]);
				break;

			case ETectonicMapExportMode::ContinentalWeight:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetContinentalWeightColor(Buffers.ContinentalWeight[PixelIndex]);
				break;

			case ETectonicMapExportMode::SubductionDistance:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetSubductionDistanceColor(Buffers.SubductionDistanceKm[PixelIndex]);
				break;

			case ETectonicMapExportMode::BoundaryMask:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetBoundaryMaskColor(Buffers.BoundaryMask[PixelIndex] != 0);
				break;

			case ETectonicMapExportMode::GapMask:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetGapMaskColor(false);
				break;

			case ETectonicMapExportMode::OverlapMask:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetOverlapMaskColor(false);
				break;

			case ETectonicMapExportMode::MaterialClassification:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetGapMaskColor(false);
				break;

			case ETectonicMapExportMode::MaterialOverlap:
				OutPixels[PixelIndex] = TectonicPlanetVisualization::GetOverlapMaskColor(false);
				break;

			case ETectonicMapExportMode::All:
			default:
				OutPixels[PixelIndex] = FColor::Black;
				break;
			}
		}
	}

	void BuildEnhancedDisplayElevationBuffer(
		const FRasterBuffers& Buffers,
		const FTectonicMollweideExportOptions& Options,
		TArray<float>& OutEnhancedElevation)
	{
		OutEnhancedElevation = Buffers.Elevation;
		if (OutEnhancedElevation.IsEmpty())
		{
			return;
		}

		TArray<float> PreviousElevation = OutEnhancedElevation;
		for (int32 PassIndex = 0; PassIndex < EnhancedOceanDisplaySmoothingPasses; ++PassIndex)
		{
			for (int32 PixelY = 0; PixelY < Options.Height; ++PixelY)
			{
				for (int32 PixelX = 0; PixelX < Options.Width; ++PixelX)
				{
					const int32 PixelIndex = PixelY * Options.Width + PixelX;
					if (Buffers.Coverage[PixelIndex] == 0 ||
						!IsInsideMollweideEllipse(PixelX, PixelY, Options.Width, Options.Height) ||
						!ShouldSmoothOceanDisplayPixel(Buffers, PixelIndex))
					{
						OutEnhancedElevation[PixelIndex] = Buffers.Elevation[PixelIndex];
						continue;
					}

					float WeightedSum = PreviousElevation[PixelIndex] * 4.0f;
					float WeightSum = 4.0f;
					for (int32 OffsetY = -1; OffsetY <= 1; ++OffsetY)
					{
						const int32 NeighborY = PixelY + OffsetY;
						if (NeighborY < 0 || NeighborY >= Options.Height)
						{
							continue;
						}

						for (int32 OffsetX = -1; OffsetX <= 1; ++OffsetX)
						{
							if (OffsetX == 0 && OffsetY == 0)
							{
								continue;
							}

							const int32 NeighborX = WrapPixelX(PixelX + OffsetX, Options.Width);
							const int32 NeighborIndex = NeighborY * Options.Width + NeighborX;
							if (Buffers.Coverage[NeighborIndex] == 0 ||
								!ShouldSmoothOceanDisplayPixel(Buffers, NeighborIndex))
							{
								continue;
							}

							const float NeighborWeight = (OffsetX == 0 || OffsetY == 0) ? 2.0f : 1.0f;
							WeightedSum += PreviousElevation[NeighborIndex] * NeighborWeight;
							WeightSum += NeighborWeight;
						}
					}

					const float SmoothedElevation = WeightSum > UE_SMALL_NUMBER
						? WeightedSum / WeightSum
						: PreviousElevation[PixelIndex];
					const float BlendAlpha = ComputeOceanDisplayBlendAlpha(Buffers.Elevation[PixelIndex]);
					OutEnhancedElevation[PixelIndex] =
						FMath::Lerp(PreviousElevation[PixelIndex], SmoothedElevation, BlendAlpha);
				}
			}

			PreviousElevation = OutEnhancedElevation;
		}
	}

	void BuildEnhancedElevationImage(
		const FRasterBuffers& Buffers,
		const FTectonicMollweideExportOptions& Options,
		TArray<FColor>& OutPixels)
	{
		TArray<float> EnhancedElevation;
		BuildEnhancedDisplayElevationBuffer(Buffers, Options, EnhancedElevation);

		const int32 PixelCount = Options.Width * Options.Height;
		OutPixels.SetNum(PixelCount);
		for (int32 PixelIndex = 0; PixelIndex < PixelCount; ++PixelIndex)
		{
			const int32 PixelX = PixelIndex % Options.Width;
			const int32 PixelY = PixelIndex / Options.Width;
			if (Buffers.Coverage[PixelIndex] == 0 || !IsInsideMollweideEllipse(PixelX, PixelY, Options.Width, Options.Height))
			{
				OutPixels[PixelIndex] = FColor::Black;
				continue;
			}

			OutPixels[PixelIndex] = TectonicPlanetVisualization::GetElevationColor(
				EnhancedElevation[PixelIndex],
				ETectonicElevationPresentationMode::Enhanced);
		}
	}

	void DrawLineOnImage(TArray<FColor>& Pixels, const int32 Width, const int32 Height,
		int32 X0, int32 Y0, int32 X1, int32 Y1, const FColor& Color, const int32 Thickness)
	{
		const int32 DeltaX = FMath::Abs(X1 - X0);
		const int32 DeltaY = FMath::Abs(Y1 - Y0);
		const int32 StepX = (X0 < X1) ? 1 : -1;
		const int32 StepY = (Y0 < Y1) ? 1 : -1;
		int32 Error = DeltaX - DeltaY;

		const int32 HalfThick = Thickness / 2;
		auto PlotThick = [&](int32 CenterX, int32 CenterY)
		{
			for (int32 OffsetY = -HalfThick; OffsetY <= HalfThick; ++OffsetY)
			{
				for (int32 OffsetX = -HalfThick; OffsetX <= HalfThick; ++OffsetX)
				{
					const int32 PlotX = CenterX + OffsetX;
					const int32 PlotY = CenterY + OffsetY;
					if (PlotX >= 0 && PlotX < Width && PlotY >= 0 && PlotY < Height)
					{
						Pixels[PlotY * Width + PlotX] = Color;
					}
				}
			}
		};

		for (;;)
		{
			PlotThick(X0, Y0);
			if (X0 == X1 && Y0 == Y1)
			{
				break;
			}

			const int32 Error2 = 2 * Error;
			if (Error2 > -DeltaY)
			{
				Error -= DeltaY;
				X0 += StepX;
			}
			if (Error2 < DeltaX)
			{
				Error += DeltaX;
				Y0 += StepY;
			}
		}
	}

	void DrawArrowhead(TArray<FColor>& Pixels, const int32 Width, const int32 Height,
		const FVector2d& Tip, const FVector2d& From, const FColor& Color, const double HeadLength)
	{
		FVector2d Direction = Tip - From;
		const double Length = Direction.Length();
		if (Length < 1.0)
		{
			return;
		}
		Direction /= Length;
		const FVector2d Perp(-Direction.Y, Direction.X);
		const FVector2d Base = Tip - Direction * HeadLength;
		const FVector2d Left = Base + Perp * HeadLength * 0.5;
		const FVector2d Right = Base - Perp * HeadLength * 0.5;

		DrawLineOnImage(Pixels, Width, Height,
			FMath::RoundToInt(Tip.X), FMath::RoundToInt(Tip.Y),
			FMath::RoundToInt(Left.X), FMath::RoundToInt(Left.Y),
			Color, 2);
		DrawLineOnImage(Pixels, Width, Height,
			FMath::RoundToInt(Tip.X), FMath::RoundToInt(Tip.Y),
			FMath::RoundToInt(Right.X), FMath::RoundToInt(Right.Y),
			Color, 2);
	}

	struct FCombinedMapArrowStats
	{
		int32 ArrowsDrawn = 0;
		int32 ArrowsSkippedNearZero = 0;
		int32 ArrowsSkippedOffEllipse = 0;
		int32 ArrowsSkippedSeamOrTooLong = 0;
		double SpeedSum = 0.0;
		double MaxSpeed = 0.0;
	};

	void BuildCombinedTectonicSummaryImage(
		const FTectonicPlanet& Planet,
		const FRasterBuffers& Buffers,
		const FTectonicMollweideExportOptions& Options,
		TArray<FColor>& OutPixels,
		FCombinedMapArrowStats& OutArrowStats)
	{
		OutArrowStats = FCombinedMapArrowStats{};

		// Layer 1: Enhanced elevation base
		TArray<float> EnhancedElevation;
		BuildEnhancedDisplayElevationBuffer(Buffers, Options, EnhancedElevation);

		const int32 PixelCount = Options.Width * Options.Height;
		OutPixels.SetNum(PixelCount);
		for (int32 PixelIndex = 0; PixelIndex < PixelCount; ++PixelIndex)
		{
			const int32 PixelX = PixelIndex % Options.Width;
			const int32 PixelY = PixelIndex / Options.Width;
			if (Buffers.Coverage[PixelIndex] == 0 || !IsInsideMollweideEllipse(PixelX, PixelY, Options.Width, Options.Height))
			{
				OutPixels[PixelIndex] = FColor::Black;
				continue;
			}

			OutPixels[PixelIndex] = TectonicPlanetVisualization::GetElevationColor(
				EnhancedElevation[PixelIndex],
				ETectonicElevationPresentationMode::Enhanced);
		}

		// Layer 2: Semi-transparent white boundary overlay
		constexpr double BoundaryBlendAlpha = 0.45;
		const FColor BoundaryOverlayColor(255, 255, 255);
		for (int32 PixelIndex = 0; PixelIndex < PixelCount; ++PixelIndex)
		{
			if (Buffers.Coverage[PixelIndex] == 0 || Buffers.BoundaryMask[PixelIndex] == 0)
			{
				continue;
			}
			const FColor& Base = OutPixels[PixelIndex];
			OutPixels[PixelIndex] = FColor(
				static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(static_cast<double>(Base.R), 255.0, BoundaryBlendAlpha)), 0, 255)),
				static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(static_cast<double>(Base.G), 255.0, BoundaryBlendAlpha)), 0, 255)),
				static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(static_cast<double>(Base.B), 255.0, BoundaryBlendAlpha)), 0, 255)),
				255);
		}

		// Layer 3: Velocity arrows from plate rotation data
		if (Planet.Plates.IsEmpty() || Planet.Samples.IsEmpty())
		{
			return;
		}

		// Display time horizon scaled to image width so typical plate speeds (~10 km/My) produce ~30px arrows
		constexpr double DisplayTimeMy = 30.0;
		constexpr double NearZeroSpeedThreshold = 0.5; // km/My
		constexpr double ArrowGridSpacingDeg = 18.0;
		constexpr double MaxProjectedArrowLengthPx = 200.0;
		constexpr double SeamProximityFraction = 0.05;
		const FColor ArrowColor(220, 40, 40);
		const int32 ArrowShaftThickness = FMath::Max(Options.Width / 1500, 2);
		const double ArrowHeadLength = FMath::Max(static_cast<double>(Options.Width) / 300.0, 6.0);

		// Build a lookup: for each sample, which plate index owns it?
		TMap<int32, int32> PlateIdToArrayIndex;
		for (int32 PlateArrayIndex = 0; PlateArrayIndex < Planet.Plates.Num(); ++PlateArrayIndex)
		{
			PlateIdToArrayIndex.Add(Planet.Plates[PlateArrayIndex].Id, PlateArrayIndex);
		}

		// Place arrows on a regular lat/lon grid
		for (double LatDeg = -80.0; LatDeg <= 80.0; LatDeg += ArrowGridSpacingDeg)
		{
			for (double LonDeg = -180.0; LonDeg < 180.0; LonDeg += ArrowGridSpacingDeg)
			{
				const double LatRad = FMath::DegreesToRadians(LatDeg);
				const double LonRad = FMath::DegreesToRadians(LonDeg);
				const FVector3d Origin(
					FMath::Cos(LatRad) * FMath::Cos(LonRad),
					FMath::Cos(LatRad) * FMath::Sin(LonRad),
					FMath::Sin(LatRad));

				// Find nearest sample to determine plate ownership
				int32 NearestSampleIndex = INDEX_NONE;
				double BestDot = -2.0;
				for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
				{
					const double Dot = FVector3d::DotProduct(Origin, Planet.Samples[SampleIndex].Position);
					if (Dot > BestDot)
					{
						BestDot = Dot;
						NearestSampleIndex = SampleIndex;
					}
				}
				if (NearestSampleIndex == INDEX_NONE)
				{
					continue;
				}

				const int32 OwnerPlateId = Planet.Samples[NearestSampleIndex].PlateId;
				const int32* PlateArrayIndexPtr = PlateIdToArrayIndex.Find(OwnerPlateId);
				if (!PlateArrayIndexPtr)
				{
					continue;
				}

				const FPlate& Plate = Planet.Plates[*PlateArrayIndexPtr];
				const FVector3d Axis = Plate.RotationAxis.GetSafeNormal();
				const FVector3d OriginNorm = Origin.GetSafeNormal();
				const FVector3d Tangent = FVector3d::CrossProduct(Axis, OriginNorm);
				const double TangentLength = Tangent.Size();

				if (TangentLength < 1.0e-6)
				{
					++OutArrowStats.ArrowsSkippedNearZero;
					continue;
				}

				const double SpeedKmPerMy = TangentLength * Plate.AngularSpeed * Planet.PlanetRadiusKm;

				if (SpeedKmPerMy < NearZeroSpeedThreshold)
				{
					++OutArrowStats.ArrowsSkippedNearZero;
					continue;
				}

				// Tangent-on-sphere tip construction
				const FVector3d TangentDir = Tangent.GetSafeNormal();
				const double DisplacementRad = (SpeedKmPerMy * DisplayTimeMy) / Planet.PlanetRadiusKm;
				const FVector3d TipPosition = (OriginNorm + TangentDir * DisplacementRad).GetSafeNormal();

				// Project origin and tip through Mollweide
				const FVector2d OriginPixel = ProjectMollweidePixel(OriginNorm, Options.Width, Options.Height);
				const FVector2d TipPixel = ProjectMollweidePixel(TipPosition, Options.Width, Options.Height);

				// Check ellipse containment for both endpoints
				const int32 OriginPx = FMath::RoundToInt(OriginPixel.X);
				const int32 OriginPy = FMath::RoundToInt(OriginPixel.Y);
				const int32 TipPx = FMath::RoundToInt(TipPixel.X);
				const int32 TipPy = FMath::RoundToInt(TipPixel.Y);

				if (!IsInsideMollweideEllipse(OriginPx, OriginPy, Options.Width, Options.Height) ||
					!IsInsideMollweideEllipse(TipPx, TipPy, Options.Width, Options.Height))
				{
					++OutArrowStats.ArrowsSkippedOffEllipse;
					continue;
				}

				// Check for seam crossing or unreasonably long projected arrows
				const double ProjectedLength = (TipPixel - OriginPixel).Size();
				const double SeamThreshold = Options.Width * SeamProximityFraction;
				const bool bCrossesSeam = FMath::Abs(TipPixel.X - OriginPixel.X) > Options.Width * 0.4;

				if (bCrossesSeam || ProjectedLength > MaxProjectedArrowLengthPx)
				{
					++OutArrowStats.ArrowsSkippedSeamOrTooLong;
					continue;
				}

				if (ProjectedLength < 2.0)
				{
					++OutArrowStats.ArrowsSkippedNearZero;
					continue;
				}

				// Draw shaft
				DrawLineOnImage(OutPixels, Options.Width, Options.Height,
					OriginPx, OriginPy, TipPx, TipPy,
					ArrowColor, ArrowShaftThickness);

				// Draw arrowhead
				DrawArrowhead(OutPixels, Options.Width, Options.Height,
					TipPixel, OriginPixel, ArrowColor, ArrowHeadLength);

				++OutArrowStats.ArrowsDrawn;
				OutArrowStats.SpeedSum += SpeedKmPerMy;
				OutArrowStats.MaxSpeed = FMath::Max(OutArrowStats.MaxSpeed, SpeedKmPerMy);
			}
		}
	}
}

bool TectonicMollweideExporter::ExportPlanet(
	const FTectonicPlanet& Planet,
	const FTectonicMollweideExportOptions& Options,
	FTectonicMollweideExportStats& OutStats,
	FString& OutError)
{
	OutStats = FTectonicMollweideExportStats{};
	OutStats.Mode = Options.Mode;
	OutStats.Width = Options.Width;
	OutStats.Height = Options.Height;

	if (Planet.Samples.IsEmpty() || Planet.TriangleIndices.IsEmpty())
	{
		OutError = TEXT("Planet has no canonical mesh data to export.");
		return false;
	}

	if (Options.Width <= 0 || Options.Height <= 0)
	{
		OutError = TEXT("Export dimensions must be positive.");
		return false;
	}

	if (Options.OutputDirectory.IsEmpty())
	{
		OutError = TEXT("Export output directory is empty.");
		return false;
	}

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!PlatformFile.CreateDirectoryTree(*Options.OutputDirectory))
	{
		OutError = FString::Printf(TEXT("Failed to create export directory: %s"), *Options.OutputDirectory);
		return false;
	}

	FRasterBuffers Buffers;
	BuildRasterBuffers(
		Planet,
		Options,
		Buffers,
		OutStats.PreGapFillUncoveredInteriorPixelCount,
		OutStats.UncoveredInteriorPixelCount);

	int64 ContinentalPixelCount = 0;
	for (const uint8 Covered : Buffers.Coverage)
	{
		OutStats.ResolvedPixelCount += Covered != 0 ? 1 : 0;
	}
	for (int32 PixelIndex = 0; PixelIndex < Buffers.Coverage.Num(); ++PixelIndex)
	{
		if (Buffers.Coverage[PixelIndex] != 0 && Buffers.ContinentalWeight[PixelIndex] >= 0.5f)
		{
			++ContinentalPixelCount;
		}
	}
	if (OutStats.ResolvedPixelCount > 0)
	{
		OutStats.ContinentalPixelFraction =
			static_cast<double>(ContinentalPixelCount) / static_cast<double>(OutStats.ResolvedPixelCount);
	}

	TArray<FColor> ImagePixels;
	const ETectonicMapExportMode ModesToWrite[] = {
		ETectonicMapExportMode::Elevation,
		ETectonicMapExportMode::PlateId,
		ETectonicMapExportMode::CrustType,
		ETectonicMapExportMode::ContinentalWeight,
		ETectonicMapExportMode::SubductionDistance,
		ETectonicMapExportMode::BoundaryMask,
		ETectonicMapExportMode::GapMask,
		ETectonicMapExportMode::OverlapMask
	};

	for (const ETectonicMapExportMode Mode : ModesToWrite)
	{
		if (!ShouldExportMode(Options.Mode, Mode))
		{
			continue;
		}

		BuildImageForMode(Buffers, Options, Mode, ImagePixels);
		const FString OutputPath = FPaths::Combine(
			Options.OutputDirectory,
			FString::Printf(TEXT("%s.png"), TectonicPlanetVisualization::GetExportModeName(Mode)));

		FString WriteError;
		if (!WritePng(OutputPath, ImagePixels, Options.Width, Options.Height, WriteError))
		{
			OutError = WriteError;
			return false;
		}

		OutStats.WrittenFiles.Add(OutputPath);

		if (Mode == ETectonicMapExportMode::Elevation)
		{
			BuildEnhancedElevationImage(Buffers, Options, ImagePixels);
			const FString EnhancedOutputPath = FPaths::Combine(
				Options.OutputDirectory,
				TEXT("ElevationEnhanced.png"));

			FString EnhancedWriteError;
			if (!WritePng(EnhancedOutputPath, ImagePixels, Options.Width, Options.Height, EnhancedWriteError))
			{
				OutError = EnhancedWriteError;
				return false;
			}

			OutStats.WrittenFiles.Add(EnhancedOutputPath);
		}
	}

	// Combined Tectonic Summary: special-cased because it needs the full Planet for velocity arrows
	if (ShouldExportMode(Options.Mode, ETectonicMapExportMode::CombinedTectonicSummary))
	{
		FCombinedMapArrowStats ArrowStats;
		BuildCombinedTectonicSummaryImage(Planet, Buffers, Options, ImagePixels, ArrowStats);

		const FString CombinedOutputPath = FPaths::Combine(
			Options.OutputDirectory,
			TEXT("CombinedTectonicSummary.png"));

		FString CombinedWriteError;
		if (!WritePng(CombinedOutputPath, ImagePixels, Options.Width, Options.Height, CombinedWriteError))
		{
			OutError = CombinedWriteError;
			return false;
		}

		OutStats.WrittenFiles.Add(CombinedOutputPath);

		const double MeanSpeed = ArrowStats.ArrowsDrawn > 0
			? ArrowStats.SpeedSum / static_cast<double>(ArrowStats.ArrowsDrawn)
			: 0.0;

		UE_LOG(LogTemp, Log,
			TEXT("CombinedTectonicSummary arrows: drawn=%d, skipped_near_zero=%d, skipped_off_ellipse=%d, skipped_seam_or_too_long=%d, mean_speed=%.2f km/My, max_speed=%.2f km/My"),
			ArrowStats.ArrowsDrawn,
			ArrowStats.ArrowsSkippedNearZero,
			ArrowStats.ArrowsSkippedOffEllipse,
			ArrowStats.ArrowsSkippedSeamOrTooLong,
			MeanSpeed,
			ArrowStats.MaxSpeed);
	}

	return true;
}

bool TectonicMollweideExporter::ExportScalarOverlay(
	const FTectonicPlanet& Planet,
	const TArray<float>& PerSampleValues,
	const float MinValue,
	const float MaxValue,
	const FString& OutputPath,
	const int32 Width,
	const int32 Height,
	FString& OutError)
{
	if (Planet.Samples.IsEmpty() || Planet.TriangleIndices.IsEmpty())
	{
		OutError = TEXT("Planet has no canonical mesh data to export.");
		return false;
	}

	if (PerSampleValues.Num() != Planet.Samples.Num())
	{
		OutError = FString::Printf(
			TEXT("PerSampleValues size (%d) does not match Samples size (%d)."),
			PerSampleValues.Num(), Planet.Samples.Num());
		return false;
	}

	if (Width <= 0 || Height <= 0)
	{
		OutError = TEXT("Export dimensions must be positive.");
		return false;
	}

	const FString OutputDirectory = FPaths::GetPath(OutputPath);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!PlatformFile.CreateDirectoryTree(*OutputDirectory))
	{
		OutError = FString::Printf(TEXT("Failed to create output directory: %s"), *OutputDirectory);
		return false;
	}

	const int32 PixelCount = Width * Height;
	TArray<float> ScalarBuffer;
	TArray<uint8> CoverageBuffer;
	ScalarBuffer.Init(0.0f, PixelCount);
	CoverageBuffer.Init(0, PixelCount);

	// Project samples
	TArray<FVector2d> ProjectedPixels;
	ProjectedPixels.SetNum(Planet.Samples.Num());
	for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
	{
		ProjectedPixels[SampleIndex] = ProjectMollweidePixel(
			Planet.Samples[SampleIndex].Position, Width, Height);
	}

	// Rasterize triangles
	for (const FIntVector& Triangle : Planet.TriangleIndices)
	{
		if (!Planet.Samples.IsValidIndex(Triangle.X) ||
			!Planet.Samples.IsValidIndex(Triangle.Y) ||
			!Planet.Samples.IsValidIndex(Triangle.Z))
		{
			continue;
		}

		FVector2d P0 = ProjectedPixels[Triangle.X];
		FVector2d P1 = ProjectedPixels[Triangle.Y];
		FVector2d P2 = ProjectedPixels[Triangle.Z];
		UnwrapTriangleX(P0, P1, P2, static_cast<double>(Width));

		const double TwiceArea = EdgeFunction(P0, P1, P2);
		if (FMath::Abs(TwiceArea) <= RasterEpsilon)
		{
			continue;
		}

		const int32 MinPixelX = FMath::FloorToInt(FMath::Min3(P0.X, P1.X, P2.X));
		const int32 MaxPixelX = FMath::CeilToInt(FMath::Max3(P0.X, P1.X, P2.X));
		const double ExpectedEdgePx = static_cast<double>(Width) *
			FMath::Sqrt(4.0 * PI / FMath::Max(Planet.Samples.Num(), 1)) / PI;
		const int32 MaxTriangleWidthPx = FMath::Max(FMath::CeilToInt(ExpectedEdgePx * 30.0), 50);
		if ((MaxPixelX - MinPixelX) > MaxTriangleWidthPx)
		{
			continue;
		}

		const int32 MinPixelY = FMath::Clamp(FMath::FloorToInt(FMath::Min3(P0.Y, P1.Y, P2.Y)), 0, Height - 1);
		const int32 MaxPixelY = FMath::Clamp(FMath::CeilToInt(FMath::Max3(P0.Y, P1.Y, P2.Y)), 0, Height - 1);
		if (MinPixelY > MaxPixelY)
		{
			continue;
		}

		const float V0 = PerSampleValues[Triangle.X];
		const float V1 = PerSampleValues[Triangle.Y];
		const float V2 = PerSampleValues[Triangle.Z];

		for (int32 PixelY = MinPixelY; PixelY <= MaxPixelY; ++PixelY)
		{
			for (int32 PixelX = MinPixelX; PixelX <= MaxPixelX; ++PixelX)
			{
				const FVector2d PixelCenter(
					static_cast<double>(PixelX) + 0.5,
					static_cast<double>(PixelY) + 0.5);
				const double W0 = EdgeFunction(P1, P2, PixelCenter) / TwiceArea;
				const double W1 = EdgeFunction(P2, P0, PixelCenter) / TwiceArea;
				const double W2 = EdgeFunction(P0, P1, PixelCenter) / TwiceArea;
				if (W0 < -RasterEpsilon || W1 < -RasterEpsilon || W2 < -RasterEpsilon)
				{
					continue;
				}

				const int32 WrappedX = WrapPixelX(PixelX, Width);
				const int32 BufferIndex = PixelY * Width + WrappedX;
				ScalarBuffer[BufferIndex] = static_cast<float>(W0 * V0 + W1 * V1 + W2 * V2);
				CoverageBuffer[BufferIndex] = 1;
			}
		}
	}

	// Gap fill (iterative nearest-neighbor, same logic as core exporter)
	constexpr int32 MaxGapFillIterations = 16;
	for (int32 Iteration = 0; Iteration < MaxGapFillIterations; ++Iteration)
	{
		TArray<float> PreviousScalar = ScalarBuffer;
		TArray<uint8> PreviousCoverage = CoverageBuffer;
		int32 FilledCount = 0;

		for (int32 PixelY = 0; PixelY < Height; ++PixelY)
		{
			for (int32 PixelX = 0; PixelX < Width; ++PixelX)
			{
				const int32 PixelIndex = PixelY * Width + PixelX;
				if (PreviousCoverage[PixelIndex] != 0)
				{
					continue;
				}
				if (!IsInsideMollweideEllipse(PixelX, PixelY, Width, Height))
				{
					continue;
				}

				int32 BestNeighborIndex = INDEX_NONE;
				for (int32 OffsetY = -1; OffsetY <= 1; ++OffsetY)
				{
					const int32 NeighborY = PixelY + OffsetY;
					if (NeighborY < 0 || NeighborY >= Height)
					{
						continue;
					}
					for (int32 OffsetX = -1; OffsetX <= 1; ++OffsetX)
					{
						if (OffsetX == 0 && OffsetY == 0) { continue; }
						const int32 NeighborX = WrapPixelX(PixelX + OffsetX, Width);
						const int32 NeighborIndex = NeighborY * Width + NeighborX;
						if (PreviousCoverage[NeighborIndex] != 0 &&
							(BestNeighborIndex == INDEX_NONE || NeighborIndex < BestNeighborIndex))
						{
							BestNeighborIndex = NeighborIndex;
						}
					}
				}

				if (BestNeighborIndex != INDEX_NONE)
				{
					ScalarBuffer[PixelIndex] = PreviousScalar[BestNeighborIndex];
					CoverageBuffer[PixelIndex] = 1;
					++FilledCount;
				}
			}
		}

		if (FilledCount == 0)
		{
			break;
		}
	}

	// Build grayscale image
	const float Range = FMath::Max(MaxValue - MinValue, UE_SMALL_NUMBER);
	TArray<FColor> Pixels;
	Pixels.SetNum(PixelCount);
	for (int32 PixelIndex = 0; PixelIndex < PixelCount; ++PixelIndex)
	{
		const int32 PixelX = PixelIndex % Width;
		const int32 PixelY = PixelIndex / Width;
		if (CoverageBuffer[PixelIndex] == 0 || !IsInsideMollweideEllipse(PixelX, PixelY, Width, Height))
		{
			Pixels[PixelIndex] = FColor::Black;
			continue;
		}

		const float Normalized = FMath::Clamp((ScalarBuffer[PixelIndex] - MinValue) / Range, 0.0f, 1.0f);
		const uint8 Intensity = static_cast<uint8>(Normalized * 255.0f);
		Pixels[PixelIndex] = FColor(Intensity, Intensity, Intensity, 255);
	}

	FString WriteError;
	if (!WritePng(OutputPath, Pixels, Width, Height, WriteError))
	{
		OutError = WriteError;
		return false;
	}

	return true;
}
