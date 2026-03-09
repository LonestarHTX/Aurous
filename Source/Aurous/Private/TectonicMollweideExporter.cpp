#include "TectonicMollweideExporter.h"

#include "Async/ParallelFor.h"
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"
#include "TectonicPlanet.h"

#if WITH_EDITOR
#include "HAL/PlatformFileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/FileHelper.h"
#include "Modules/ModuleManager.h"
#endif

namespace
{
	constexpr double MollweideSqrtTwo = 1.4142135623730950488;
	constexpr double MollweideHalfWidth = 2.0 * MollweideSqrtTwo;
	constexpr double MollweideHalfHeight = MollweideSqrtTwo;
	constexpr double MollweideInverseEpsilon = 1e-12;
	constexpr double MollweideEllipseTolerance = 1e-9;
	const FColor ProjectionBackgroundColor(12, 12, 16);
	const FColor UnresolvedFallbackColor(255, 0, 255);
	const FColor OceanicCrustColor(0, 105, 148);
	const FColor ContinentalCrustColor(139, 90, 43);

	struct FResolvedPixelSample
	{
		bool bResolved = false;
		bool bFromNeighborProbe = false;
		int32 TriangleIndex = INDEX_NONE;
		int32 PlateId = INDEX_NONE;
		FVector Barycentric = FVector::ZeroVector;
	};

	struct FRowExportStats
	{
		int64 InsideEllipsePixelCount = 0;
		int64 ContainmentFailureCount = 0;
		int64 NeighborProbeFallbackCount = 0;
		int64 ClearFallbackCount = 0;
	};

	uint32 MixDeterministicUint32(uint32 Value)
	{
		Value ^= Value >> 16;
		Value *= 0x7feb352dU;
		Value ^= Value >> 15;
		Value *= 0x846ca68bU;
		Value ^= Value >> 16;
		return Value;
	}

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

	FColor GetPlateColor(const int32 PlateId)
	{
		static const FColor BasePalette[] = {
			FColor(230, 25, 75),
			FColor(60, 180, 75),
			FColor(255, 225, 25),
			FColor(0, 130, 200),
			FColor(245, 130, 48),
			FColor(145, 30, 180),
			FColor(70, 240, 240),
			FColor(240, 50, 230),
			FColor(210, 245, 60),
			FColor(250, 190, 212),
			FColor(0, 128, 128),
			FColor(220, 190, 255)
		};

		if (PlateId < 0)
		{
			return FColor::Black;
		}

		if (PlateId < UE_ARRAY_COUNT(BasePalette))
		{
			return BasePalette[PlateId];
		}

		const uint32 Hash = MixDeterministicUint32(static_cast<uint32>(PlateId));
		const uint8 Hue = static_cast<uint8>(Hash & 0xffU);
		const uint8 Saturation = static_cast<uint8>(180 + ((Hash >> 8) % 48U));
		const uint8 Value = static_cast<uint8>(210 + ((Hash >> 16) % 36U));
		return FLinearColor::MakeFromHSV8(Hue, Saturation, Value).ToFColor(true);
	}

	FColor GetElevationColor(const double Elevation, const double MinElevation, const double MaxElevation)
	{
		const FColor DeepOcean(9, 24, 79);
		const FColor MidOcean(24, 92, 173);
		const FColor ShallowOcean(86, 168, 199);
		const FColor LowLand(68, 120, 54);
		const FColor MidLand(186, 168, 120);
		const FColor HighLand(248, 248, 248);

		if (MaxElevation <= MinElevation + UE_DOUBLE_SMALL_NUMBER)
		{
			return LowLand;
		}

		if (MaxElevation <= 0.0)
		{
			const double Alpha = FMath::Clamp((Elevation - MinElevation) / FMath::Max(MaxElevation - MinElevation, 1e-9), 0.0, 1.0);
			return (Alpha < 0.65)
				? LerpColor(DeepOcean, MidOcean, Alpha / 0.65)
				: LerpColor(MidOcean, ShallowOcean, (Alpha - 0.65) / 0.35);
		}

		if (MinElevation >= 0.0)
		{
			const double Alpha = FMath::Clamp((Elevation - MinElevation) / FMath::Max(MaxElevation - MinElevation, 1e-9), 0.0, 1.0);
			return (Alpha < 0.55)
				? LerpColor(LowLand, MidLand, Alpha / 0.55)
				: LerpColor(MidLand, HighLand, (Alpha - 0.55) / 0.45);
		}

		if (Elevation <= 0.0)
		{
			const double Alpha = FMath::Clamp((Elevation - MinElevation) / FMath::Max(-MinElevation, 1e-9), 0.0, 1.0);
			return (Alpha < 0.65)
				? LerpColor(DeepOcean, MidOcean, Alpha / 0.65)
				: LerpColor(MidOcean, ShallowOcean, (Alpha - 0.65) / 0.35);
		}

		const double Alpha = FMath::Clamp(Elevation / FMath::Max(MaxElevation, 1e-9), 0.0, 1.0);
		return (Alpha < 0.55)
			? LerpColor(LowLand, MidLand, Alpha / 0.55)
			: LerpColor(MidLand, HighLand, (Alpha - 0.55) / 0.45);
	}

	int32 GetDominantCorner(const FVector& Barycentric)
	{
		if (Barycentric.X >= Barycentric.Y && Barycentric.X >= Barycentric.Z)
		{
			return 0;
		}

		if (Barycentric.Y >= Barycentric.X && Barycentric.Y >= Barycentric.Z)
		{
			return 1;
		}

		return 2;
	}

	bool TriangleHasMixedCrustType(
		const FCanonicalSample& Sample0,
		const FCanonicalSample& Sample1,
		const FCanonicalSample& Sample2)
	{
		return Sample0.CrustType != Sample1.CrustType ||
			Sample0.CrustType != Sample2.CrustType ||
			Sample1.CrustType != Sample2.CrustType;
	}

	const FCanonicalSample& GetDominantVertexSample(
		const FCanonicalSample& Sample0,
		const FCanonicalSample& Sample1,
		const FCanonicalSample& Sample2,
		const FVector& Barycentric)
	{
		switch (GetDominantCorner(Barycentric))
		{
		case 0:
			return Sample0;
		case 1:
			return Sample1;
		case 2:
		default:
			return Sample2;
		}
	}

	double WrapHorizontalPixelSample(const double PixelSampleX, const int32 Width)
	{
		const double WidthAsDouble = static_cast<double>(Width);
		double WrappedSample = FMath::Fmod(PixelSampleX, WidthAsDouble);
		if (WrappedSample < 0.0)
		{
			WrappedSample += WidthAsDouble;
		}
		return WrappedSample;
	}

	bool TryInverseProjectMollweide(
		const double PixelSampleX,
		const double PixelSampleY,
		const int32 Width,
		const int32 Height,
		FVector& OutDirection)
	{
		const double WrappedPixelSampleX = WrapHorizontalPixelSample(PixelSampleX, Width);
		const double NormalizedX = (2.0 * (WrappedPixelSampleX / static_cast<double>(Width))) - 1.0;
		const double NormalizedY = 1.0 - (2.0 * (PixelSampleY / static_cast<double>(Height)));
		const double ProjectionX = NormalizedX * MollweideHalfWidth;
		const double ProjectionY = NormalizedY * MollweideHalfHeight;
		const double EllipseValue = (ProjectionX * ProjectionX) / 8.0 + (ProjectionY * ProjectionY) / 2.0;
		if (EllipseValue > 1.0 + MollweideEllipseTolerance)
		{
			return false;
		}

		const double Theta = FMath::Asin(FMath::Clamp(ProjectionY / MollweideHalfHeight, -1.0, 1.0));
		const double CosTheta = FMath::Cos(Theta);
		double Longitude = 0.0;
		if (FMath::Abs(CosTheta) > MollweideInverseEpsilon)
		{
			Longitude = (PI * ProjectionX) / (2.0 * MollweideSqrtTwo * CosTheta);
		}
		Longitude = FMath::Clamp(Longitude, -PI, PI);

		const double LatitudeTerm = (2.0 * Theta + FMath::Sin(2.0 * Theta)) / PI;
		const double Latitude = FMath::Asin(FMath::Clamp(LatitudeTerm, -1.0, 1.0));
		const double CosLatitude = FMath::Cos(Latitude);
		OutDirection = FVector(
			CosLatitude * FMath::Cos(Longitude),
			CosLatitude * FMath::Sin(Longitude),
			FMath::Sin(Latitude)).GetSafeNormal();
		return !OutDirection.IsNearlyZero();
	}

	bool TryResolveContainmentSample(
		const TArray<FCanonicalSample>& Samples,
		const TArray<FDelaunayTriangle>& Triangles,
		const FContainmentQueryResult& Containment,
		FResolvedPixelSample& OutResolved)
	{
		if (!Containment.bFoundContainingPlate || !Triangles.IsValidIndex(Containment.TriangleIndex))
		{
			return false;
		}

		const FDelaunayTriangle& Triangle = Triangles[Containment.TriangleIndex];
		if (!Samples.IsValidIndex(Triangle.V[0]) || !Samples.IsValidIndex(Triangle.V[1]) || !Samples.IsValidIndex(Triangle.V[2]))
		{
			return false;
		}

		OutResolved.bResolved = true;
		OutResolved.TriangleIndex = Containment.TriangleIndex;
		OutResolved.PlateId = Containment.PlateId;
		OutResolved.Barycentric = Containment.Barycentric;
		return true;
	}

	bool TryResolvePixelSample(
		const FTectonicPlanet& Planet,
		const TArray<FCanonicalSample>& Samples,
		const TArray<FDelaunayTriangle>& Triangles,
		const int32 PixelX,
		const int32 PixelY,
		const int32 Width,
		const int32 Height,
		FResolvedPixelSample& OutResolved,
		FRowExportStats& OutRowStats)
	{
		static const FVector2d ProbeOffsets[] = {
			FVector2d(0.0, 0.0),
			FVector2d(-0.35, 0.0),
			FVector2d(0.35, 0.0),
			FVector2d(0.0, -0.35),
			FVector2d(0.0, 0.35),
			FVector2d(-0.25, -0.25),
			FVector2d(0.25, -0.25),
			FVector2d(-0.25, 0.25),
			FVector2d(0.25, 0.25)
		};

		for (int32 ProbeIndex = 0; ProbeIndex < UE_ARRAY_COUNT(ProbeOffsets); ++ProbeIndex)
		{
			const FVector2d& ProbeOffset = ProbeOffsets[ProbeIndex];
			FVector QueryDirection;
			if (!TryInverseProjectMollweide(
				static_cast<double>(PixelX) + 0.5 + ProbeOffset.X,
				static_cast<double>(PixelY) + 0.5 + ProbeOffset.Y,
				Width,
				Height,
				QueryDirection))
			{
				continue;
			}

			FResolvedPixelSample Candidate;
			Candidate.bFromNeighborProbe = ProbeIndex > 0;
			if (!TryResolveContainmentSample(Samples, Triangles, Planet.QueryContainment(QueryDirection), Candidate))
			{
				if (ProbeIndex == 0)
				{
					++OutRowStats.ContainmentFailureCount;
				}
				continue;
			}

			if (Candidate.bFromNeighborProbe)
			{
				++OutRowStats.NeighborProbeFallbackCount;
			}

			OutResolved = Candidate;
			return true;
		}

		++OutRowStats.ClearFallbackCount;
		return false;
	}

	bool SaveColorPng(const FString& FilePath, const int32 Width, const int32 Height, const TArray<FColor>& Pixels)
	{
#if !WITH_EDITOR
		return false;
#else
		if (Pixels.Num() != Width * Height)
		{
			return false;
		}

		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		const TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
		if (!Wrapper.IsValid())
		{
			return false;
		}

		if (!Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8))
		{
			return false;
		}

		const FString Directory = FPaths::GetPath(FilePath);
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		if (!Directory.IsEmpty() && !PlatformFile.CreateDirectoryTree(*Directory))
		{
			return false;
		}

		const TArray64<uint8>& CompressedData = Wrapper->GetCompressed();
		return FFileHelper::SaveArrayToFile(CompressedData, *FilePath);
#endif
	}

	int32 FindNearestSampleByDelaunayWalk(
		const TArray<FCanonicalSample>& Samples,
		const TArray<TArray<int32>>& Adjacency,
		const FVector& QueryDirection,
		const int32 StartSampleIndex)
	{
		if (!Samples.IsValidIndex(StartSampleIndex))
		{
			return 0;
		}

		int32 CurrentIndex = StartSampleIndex;
		double CurrentDot = FVector::DotProduct(QueryDirection, Samples[CurrentIndex].Position);

		for (int32 Iteration = 0; Iteration < Samples.Num(); ++Iteration)
		{
			bool bImproved = false;
			if (Adjacency.IsValidIndex(CurrentIndex))
			{
				for (const int32 NeighborIndex : Adjacency[CurrentIndex])
				{
					if (!Samples.IsValidIndex(NeighborIndex))
					{
						continue;
					}

					const double NeighborDot = FVector::DotProduct(QueryDirection, Samples[NeighborIndex].Position);
					if (NeighborDot > CurrentDot)
					{
						CurrentDot = NeighborDot;
						CurrentIndex = NeighborIndex;
						bImproved = true;
					}
				}
			}

			if (!bImproved)
			{
				break;
			}
		}

		return CurrentIndex;
	}

	void ResolveUnresolvedPixelsViaNearestSample(
		const TArray<FCanonicalSample>& Samples,
		const TArray<TArray<int32>>& Adjacency,
		const int32 Width,
		const int32 Height,
		TArray<uint8>& InOutPixelState,
		TArray<FColor>* ElevationPixels,
		TArray<FColor>* PlateIdPixels,
		TArray<FColor>* CrustTypePixels,
		const double MinElevation,
		const double MaxElevation,
		int64& OutFilledPixelCount)
	{
		for (int32 PixelIndex = 0; PixelIndex < InOutPixelState.Num(); ++PixelIndex)
		{
			if (InOutPixelState[PixelIndex] != 2)
			{
				continue;
			}

			const int32 PixelX = PixelIndex % Width;
			const int32 PixelY = PixelIndex / Width;

			FVector QueryDirection;
			if (!TryInverseProjectMollweide(
				static_cast<double>(PixelX) + 0.5,
				static_cast<double>(PixelY) + 0.5,
				Width,
				Height,
				QueryDirection))
			{
				continue;
			}

			const int32 NearestSampleIndex = FindNearestSampleByDelaunayWalk(Samples, Adjacency, QueryDirection, 0);
			const FCanonicalSample& NearestSample = Samples[NearestSampleIndex];

			if (ElevationPixels)
			{
				(*ElevationPixels)[PixelIndex] = GetElevationColor(
					static_cast<double>(NearestSample.Elevation), MinElevation, MaxElevation);
			}
			if (PlateIdPixels)
			{
				(*PlateIdPixels)[PixelIndex] = GetPlateColor(NearestSample.PlateId);
			}
			if (CrustTypePixels)
			{
				(*CrustTypePixels)[PixelIndex] = (NearestSample.CrustType == ECrustType::Continental)
					? ContinentalCrustColor
					: OceanicCrustColor;
			}

			InOutPixelState[PixelIndex] = 1;
			++OutFilledPixelCount;
		}
	}

	FString ResolveOutputSpecifierToAbsolutePath(const FString& OutputSpecifier)
	{
		if (OutputSpecifier.IsEmpty())
		{
			return FPaths::ConvertRelativePathToFull(FPaths::Combine(
				FPaths::ProjectSavedDir(),
				TEXT("Mollweide"),
				TEXT("TectonicMollweide.png")));
		}

		if (FPaths::IsRelative(OutputSpecifier))
		{
			return FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), OutputSpecifier));
		}

		return FPaths::ConvertRelativePathToFull(OutputSpecifier);
	}

	FString MakeOutputFilePath(
		const FString& OutputSpecifier,
		const ETectonicMollweideExportMode RequestedMode,
		const ETectonicMollweideExportMode FileMode)
	{
		if (OutputSpecifier.IsEmpty())
		{
			if (RequestedMode == ETectonicMollweideExportMode::All)
			{
				return MakeOutputFilePath(
					FPaths::Combine(TEXT("Saved"), TEXT("Mollweide"), TEXT("TectonicMollweide.png")),
					RequestedMode,
					FileMode);
			}

			return MakeOutputFilePath(
				FPaths::Combine(TEXT("Saved"), TEXT("Mollweide"), FString::Printf(TEXT("TectonicMollweide_%s.png"), TectonicMollweideExporter::GetModeName(FileMode))),
				RequestedMode,
				FileMode);
		}

		const FString AbsoluteOutputPath = ResolveOutputSpecifierToAbsolutePath(OutputSpecifier);
		const FString Directory = FPaths::GetPath(AbsoluteOutputPath);
		const FString Extension = FPaths::GetExtension(AbsoluteOutputPath, false);
		const FString ModeSuffix = FString(TectonicMollweideExporter::GetModeName(FileMode));

		if (RequestedMode != ETectonicMollweideExportMode::All)
		{
			if (Extension.IsEmpty())
			{
				return AbsoluteOutputPath + TEXT(".png");
			}

			return AbsoluteOutputPath;
		}

		const FString Stem = FPaths::GetBaseFilename(AbsoluteOutputPath);
		const FString SafeExtension = Extension.IsEmpty() ? TEXT("png") : Extension;
		return FPaths::Combine(Directory, FString::Printf(TEXT("%s_%s.%s"), *Stem, *ModeSuffix, *SafeExtension));
	}
}

namespace TectonicMollweideExporter
{
	bool TryParseMode(const FString& Value, ETectonicMollweideExportMode& OutMode)
	{
		if (Value.Equals(TEXT("Elevation"), ESearchCase::IgnoreCase))
		{
			OutMode = ETectonicMollweideExportMode::Elevation;
			return true;
		}

		if (Value.Equals(TEXT("PlateId"), ESearchCase::IgnoreCase) || Value.Equals(TEXT("Plate"), ESearchCase::IgnoreCase))
		{
			OutMode = ETectonicMollweideExportMode::PlateId;
			return true;
		}

		if (Value.Equals(TEXT("CrustType"), ESearchCase::IgnoreCase) || Value.Equals(TEXT("Crust"), ESearchCase::IgnoreCase))
		{
			OutMode = ETectonicMollweideExportMode::CrustType;
			return true;
		}

		if (Value.Equals(TEXT("All"), ESearchCase::IgnoreCase))
		{
			OutMode = ETectonicMollweideExportMode::All;
			return true;
		}

		return false;
	}

	const TCHAR* GetModeName(const ETectonicMollweideExportMode Mode)
	{
		switch (Mode)
		{
		case ETectonicMollweideExportMode::Elevation:
			return TEXT("Elevation");
		case ETectonicMollweideExportMode::PlateId:
			return TEXT("PlateId");
		case ETectonicMollweideExportMode::CrustType:
			return TEXT("CrustType");
		case ETectonicMollweideExportMode::All:
		default:
			return TEXT("All");
		}
	}

	bool ExportPlanet(
		FTectonicPlanet& Planet,
		const FTectonicMollweideExportOptions& Options,
		FTectonicMollweideExportStats& OutStats,
		FString& OutError)
	{
#if !WITH_EDITOR
		OutError = TEXT("Mollweide export requires an editor build.");
		return false;
#else
		const double ExportStartSeconds = FPlatformTime::Seconds();
		OutStats = FTectonicMollweideExportStats{};
		OutStats.Mode = Options.Mode;
		OutStats.Width = Options.Width;
		OutStats.Height = Options.Height;

		if (Options.Width <= 0 || Options.Height <= 0)
		{
			OutError = TEXT("Mollweide export resolution must be positive.");
			return false;
		}

		if (Planet.GetPlates().Num() <= 0)
		{
			OutError = TEXT("Planet has no initialized plates to export.");
			return false;
		}

		const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
		const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();
		if (Samples.Num() <= 0 || Triangles.Num() <= 0)
		{
			OutError = TEXT("Planet has no sample or triangle data to export.");
			return false;
		}

		Planet.RebuildSpatialQueryData();

		double MinElevation = TNumericLimits<double>::Max();
		double MaxElevation = -TNumericLimits<double>::Max();
		for (const FCanonicalSample& Sample : Samples)
		{
			MinElevation = FMath::Min(MinElevation, static_cast<double>(Sample.Elevation));
			MaxElevation = FMath::Max(MaxElevation, static_cast<double>(Sample.Elevation));
		}

		if (MinElevation > MaxElevation)
		{
			MinElevation = 0.0;
			MaxElevation = 0.0;
		}

		OutStats.MinElevation = MinElevation;
		OutStats.MaxElevation = MaxElevation;

		const bool bExportElevation = Options.Mode == ETectonicMollweideExportMode::All || Options.Mode == ETectonicMollweideExportMode::Elevation;
		const bool bExportPlateId = Options.Mode == ETectonicMollweideExportMode::All || Options.Mode == ETectonicMollweideExportMode::PlateId;
		const bool bExportCrustType = Options.Mode == ETectonicMollweideExportMode::All || Options.Mode == ETectonicMollweideExportMode::CrustType;
		const int32 NumPixels = Options.Width * Options.Height;
		TArray<uint8> PixelState;
		PixelState.Init(0, NumPixels);

		TArray<FColor> ElevationPixels;
		TArray<FColor> PlateIdPixels;
		TArray<FColor> CrustTypePixels;
		if (bExportElevation)
		{
			ElevationPixels.Init(ProjectionBackgroundColor, NumPixels);
		}
		if (bExportPlateId)
		{
			PlateIdPixels.Init(ProjectionBackgroundColor, NumPixels);
		}
		if (bExportCrustType)
		{
			CrustTypePixels.Init(ProjectionBackgroundColor, NumPixels);
		}

		TArray<FRowExportStats> RowStats;
		RowStats.SetNum(Options.Height);

		ParallelFor(Options.Height, [&](const int32 PixelY)
		{
			FRowExportStats& Stats = RowStats[PixelY];
			for (int32 PixelX = 0; PixelX < Options.Width; ++PixelX)
			{
				const int32 PixelIndex = PixelY * Options.Width + PixelX;
				FVector CenterDirection;
				if (!TryInverseProjectMollweide(
					static_cast<double>(PixelX) + 0.5,
					static_cast<double>(PixelY) + 0.5,
					Options.Width,
					Options.Height,
					CenterDirection))
				{
					continue;
				}

				++Stats.InsideEllipsePixelCount;

				FResolvedPixelSample Resolved;
				if (!TryResolvePixelSample(Planet, Samples, Triangles, PixelX, PixelY, Options.Width, Options.Height, Resolved, Stats))
				{
					PixelState[PixelIndex] = 2;
					if (bExportElevation)
					{
						ElevationPixels[PixelIndex] = UnresolvedFallbackColor;
					}
					if (bExportPlateId)
					{
						PlateIdPixels[PixelIndex] = UnresolvedFallbackColor;
					}
					if (bExportCrustType)
					{
						CrustTypePixels[PixelIndex] = UnresolvedFallbackColor;
					}
					continue;
				}

				PixelState[PixelIndex] = 1;
				const FDelaunayTriangle& Triangle = Triangles[Resolved.TriangleIndex];
				const FCanonicalSample& Sample0 = Samples[Triangle.V[0]];
				const FCanonicalSample& Sample1 = Samples[Triangle.V[1]];
				const FCanonicalSample& Sample2 = Samples[Triangle.V[2]];
				const bool bMixedCrustTriangle = TriangleHasMixedCrustType(Sample0, Sample1, Sample2);
				const FCanonicalSample& DominantSample = GetDominantVertexSample(Sample0, Sample1, Sample2, Resolved.Barycentric);
				const double W0 = Resolved.Barycentric.X;
				const double W1 = Resolved.Barycentric.Y;
				const double W2 = Resolved.Barycentric.Z;

				if (bExportElevation)
				{
					const double Elevation = bMixedCrustTriangle
						? static_cast<double>(DominantSample.Elevation)
						: (W0 * static_cast<double>(Sample0.Elevation) +
							W1 * static_cast<double>(Sample1.Elevation) +
							W2 * static_cast<double>(Sample2.Elevation));
					ElevationPixels[PixelIndex] = GetElevationColor(Elevation, MinElevation, MaxElevation);
				}

				if (bExportPlateId)
				{
					PlateIdPixels[PixelIndex] = GetPlateColor(Resolved.PlateId);
				}

				if (bExportCrustType)
				{
					const ECrustType CrustType = bMixedCrustTriangle
						? DominantSample.CrustType
						: Sample0.CrustType;
					CrustTypePixels[PixelIndex] = (CrustType == ECrustType::Continental)
						? ContinentalCrustColor
						: OceanicCrustColor;
				}
			}
		});

		for (const FRowExportStats& Stats : RowStats)
		{
			OutStats.InsideEllipsePixelCount += Stats.InsideEllipsePixelCount;
			OutStats.ContainmentFailureCount += Stats.ContainmentFailureCount;
			OutStats.NeighborProbeFallbackCount += Stats.NeighborProbeFallbackCount;
			OutStats.ClearFallbackCount += Stats.ClearFallbackCount;
		}

		ResolveUnresolvedPixelsViaNearestSample(
			Samples,
			Planet.GetAdjacency(),
			Options.Width,
			Options.Height,
			PixelState,
			bExportElevation ? &ElevationPixels : nullptr,
			bExportPlateId ? &PlateIdPixels : nullptr,
			bExportCrustType ? &CrustTypePixels : nullptr,
			MinElevation,
			MaxElevation,
			OutStats.ImageFillFallbackCount);

		if (bExportElevation)
		{
			const FString ElevationPath = MakeOutputFilePath(Options.OutputPath, Options.Mode, ETectonicMollweideExportMode::Elevation);
			if (!SaveColorPng(ElevationPath, Options.Width, Options.Height, ElevationPixels))
			{
				OutError = FString::Printf(TEXT("Failed to write elevation PNG: %s"), *ElevationPath);
				OutStats.ExportTimeMs = (FPlatformTime::Seconds() - ExportStartSeconds) * 1000.0;
				return false;
			}
			OutStats.WrittenFiles.Add(ElevationPath);
		}

		if (bExportPlateId)
		{
			const FString PlateIdPath = MakeOutputFilePath(Options.OutputPath, Options.Mode, ETectonicMollweideExportMode::PlateId);
			if (!SaveColorPng(PlateIdPath, Options.Width, Options.Height, PlateIdPixels))
			{
				OutError = FString::Printf(TEXT("Failed to write plate-id PNG: %s"), *PlateIdPath);
				OutStats.ExportTimeMs = (FPlatformTime::Seconds() - ExportStartSeconds) * 1000.0;
				return false;
			}
			OutStats.WrittenFiles.Add(PlateIdPath);
		}

		if (bExportCrustType)
		{
			const FString CrustTypePath = MakeOutputFilePath(Options.OutputPath, Options.Mode, ETectonicMollweideExportMode::CrustType);
			if (!SaveColorPng(CrustTypePath, Options.Width, Options.Height, CrustTypePixels))
			{
				OutError = FString::Printf(TEXT("Failed to write crust-type PNG: %s"), *CrustTypePath);
				OutStats.ExportTimeMs = (FPlatformTime::Seconds() - ExportStartSeconds) * 1000.0;
				return false;
			}
			OutStats.WrittenFiles.Add(CrustTypePath);
		}

		OutStats.ExportTimeMs = (FPlatformTime::Seconds() - ExportStartSeconds) * 1000.0;
		return true;
#endif
	}
}
