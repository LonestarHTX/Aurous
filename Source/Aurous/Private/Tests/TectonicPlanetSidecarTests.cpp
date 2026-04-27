#include "Misc/AutomationTest.h"

#include "Misc/FileHelper.h"
#include "HAL/PlatformFile.h"
#include "HAL/PlatformFileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"
#include "TectonicMollweideExporter.h"
#include "TectonicPlanetSidecar.h"

namespace
{
	constexpr int32 SidecarTestExportWidth = 2048;
	constexpr int32 SidecarTestExportHeight = 1024;
	constexpr double ControlledPairSpeedKmPerMy = 80.0;
	constexpr int32 PrototypeCResolutionCompareStep = 40;
	constexpr double PrototypeCRuntimeBudgetSeconds = 90.0;

	FString BuildSidecarExportRoot(const FString& RunId)
	{
		return FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MapExports"), RunId);
	}

	double AngularDistanceRadForTest(const FVector3d& A, const FVector3d& B)
	{
		const FVector3d UnitA = A.GetSafeNormal();
		const FVector3d UnitB = B.GetSafeNormal();
		if (UnitA.IsNearlyZero() || UnitB.IsNearlyZero())
		{
			return 0.0;
		}
		return FMath::Acos(FMath::Clamp(UnitA.Dot(UnitB), -1.0, 1.0));
	}

	FVector3d NormalizeOrFallbackForTest(const FVector3d& Value, const FVector3d& Fallback)
	{
		const FVector3d Normalized = Value.GetSafeNormal();
		return Normalized.IsNearlyZero() ? Fallback.GetSafeNormal() : Normalized;
	}

	FVector3d MakeTangentFallbackForTest(const FVector3d& Normal)
	{
		const FVector3d Axis = FMath::Abs(Normal.Z) < 0.9 ? FVector3d(0.0, 0.0, 1.0) : FVector3d(1.0, 0.0, 0.0);
		return FVector3d::CrossProduct(Axis, Normal).GetSafeNormal();
	}

	FVector3d ProjectOntoTangentForTest(const FVector3d& Vector, const FVector3d& Normal)
	{
		const FVector3d UnitNormal = Normal.GetSafeNormal();
		return Vector - (Vector.Dot(UnitNormal) * UnitNormal);
	}

	bool ExportSidecarCheckpointMaps(
		FAutomationTestBase& Test,
		const FString& Prefix,
		const FTectonicPlanet& Planet,
		const FString& ExportRoot,
		const int32 Step,
		const FString& Label)
	{
		const FString OutputDirectory = FPaths::Combine(
			ExportRoot,
			Label,
			FString::Printf(TEXT("step_%03d"), Step));
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*OutputDirectory);

		FTectonicMollweideExportOptions Options;
		Options.Mode = ETectonicMapExportMode::All;
		Options.Width = SidecarTestExportWidth;
		Options.Height = SidecarTestExportHeight;
		Options.OutputDirectory = OutputDirectory;

		FTectonicMollweideExportStats ExportStats;
		FString Error;
		if (!TectonicMollweideExporter::ExportPlanet(Planet, Options, ExportStats, Error))
		{
			Test.AddError(FString::Printf(
				TEXT("[%s] export failed label=%s step=%d error=%s"),
				*Prefix,
				*Label,
				Step,
				*Error));
			return false;
		}

		Test.AddInfo(FString::Printf(
			TEXT("[%sExport] label=%s step=%d dir=%s files=%d continental_pixels=%.4f uncovered=%d"),
			*Prefix,
			*Label,
			Step,
			*OutputDirectory,
			ExportStats.WrittenFiles.Num(),
			ExportStats.ContinentalPixelFraction,
			ExportStats.UncoveredInteriorPixelCount));
		return true;
	}

	void AddSidecarDiagnosticsInfo(
		FAutomationTestBase& Test,
		const FString& Prefix,
		const FString& Label,
		const FTectonicSidecarProjectionDiagnostics& D)
	{
		Test.AddInfo(FString::Printf(
			TEXT("[%sDiagnostic] label=%s step=%d hash=%u local_mass=%.6f projected_mass=%.6f mass_rel_error=%.5f footprint_local=%.6f footprint_multi=%.6f footprint_visible=%.6f footprint_rel_error=%.5f fabricated=%.6f gap=%.6f divergent_gap=%.6f nondivergent_gap=%.6f largest_divergent_component=%.6f overlap=%.6f ocean_fill=%.6f boundary=%.6f plate_boundary=%.6f divergent_boundary=%.6f exact=%d recovered=%d gap_count=%d ocean_count=%d overlap_count=%d fabricated_count=%d classified=%d out_support=%.5f mean_support_km=%.2f max_support_km=%.2f mean_local_core_drift_km=%.2f max_local_core_drift_km=%.2f mean_kinematic_drift_km=%.2f mean_projected_drift_km=%.2f mean_drift_mismatch_km=%.2f max_drift_mismatch_km=%.2f drift_plates=%d dominant_component=%.5f tiny_component_fraction=%.5f region_components=%d multi_component_plates=%d cold_plate_mismatch=%.6f cold_cw_error=%.6f cold_boundary_delta=%.6f ownership_gap=%.6f ownership_overlap=%.6f owner_fragmentation=%.6f boundary_noise=%.6f carried_mass_rel_error=%.5f material_fabricated=%.6f material_owner_mismatch=%.6f material_overlap=%.6f high_cw_fragmentation_delta=%.6f"),
			*Prefix,
			*Label,
			D.Step,
			D.ProjectionHash,
			D.LocalContinentalMass,
			D.ProjectedContinentalMass,
			D.MaterialMassRelativeError,
			D.LocalFootprintContinentalMass,
			D.MultiHitProjectedContinentalMass,
			D.VisibleProjectedContinentalMass,
			D.FootprintMassRelativeError,
			D.FabricatedMaterialFraction,
			D.GapFraction,
			D.DivergentGapFraction,
			D.NonDivergentGapFraction,
			D.LargestDivergentGapComponentFraction,
			D.OverlapFraction,
			D.OceanFillFraction,
			D.BoundaryFraction,
			D.PlateBoundaryFraction,
			D.DivergentBoundaryFraction,
			D.ExactFootprintHitSampleCount,
			D.RecoveredFootprintSampleCount,
			D.GapSampleCount,
			D.OceanFillSampleCount,
			D.OverlapSampleCount,
			D.FabricatedMaterialSampleCount,
			D.ClassifiedSampleCount,
			D.OutOfSupportFraction,
			D.MeanSupportDistanceKm,
			D.MaxSupportDistanceKm,
			D.MeanProjectedLocalCoreDriftKm,
			D.MaxProjectedLocalCoreDriftKm,
			D.MeanKinematicDriftKm,
			D.MeanProjectedDriftKm,
			D.MeanDriftMismatchKm,
			D.MaxDriftMismatchKm,
			D.DriftPlateCount,
			D.LargePlateMeanDominantComponentFraction,
			D.TinyComponentFraction,
			D.RegionComponentCount,
			D.MultiComponentPlateCount,
			D.ColdStartPlateMismatchFraction,
			D.ColdStartContinentalMeanAbsError,
			D.ColdStartBoundaryFractionDelta,
			D.OwnershipGapFraction,
			D.OwnershipOverlapFraction,
			D.OwnerFragmentationFraction,
			D.BoundaryNoiseFraction,
			D.CarriedContinentalMassRelativeError,
			D.MaterialFabricatedFraction,
			D.MaterialOwnerMismatchFraction,
			D.MaterialOverlapFraction,
			D.HighCWFragmentationDelta));
	}

	uint32 HashSidecarDebugValues(const FTectonicPlanetSidecar& Sidecar)
	{
		uint32 Hash = 2166136261u;
		auto Mix = [&Hash](const uint32 Value)
		{
			Hash ^= Value;
			Hash *= 16777619u;
		};

		for (const int32 SourcePlateId : Sidecar.GetLastMaterialSourcePlateIds())
		{
			Mix(static_cast<uint32>(SourcePlateId + 0x9e3779b9));
		}
		for (const uint8 Classification : Sidecar.GetLastMaterialClassifications())
		{
			Mix(static_cast<uint32>(Classification));
		}
		for (const uint8 MismatchFlag : Sidecar.GetLastMaterialOwnerMismatchFlags())
		{
			Mix(static_cast<uint32>(MismatchFlag));
		}
		for (const int32 OverlapCount : Sidecar.GetLastMaterialOverlapCounts())
		{
			Mix(static_cast<uint32>(OverlapCount));
		}
		for (const uint8 DivergentBoundaryFlag : Sidecar.GetLastDivergentBoundaryFlags())
		{
			Mix(static_cast<uint32>(DivergentBoundaryFlag));
		}
		return Hash;
	}

	void MixHashUInt32(uint32& Hash, const uint32 Value)
	{
		Hash ^= Value;
		Hash *= 16777619u;
	}

	void MixHashInt32(uint32& Hash, const int32 Value)
	{
		MixHashUInt32(Hash, static_cast<uint32>(Value));
	}

	void MixHashUInt64(uint32& Hash, const uint64 Value)
	{
		MixHashUInt32(Hash, static_cast<uint32>(Value & 0xffffffffull));
		MixHashUInt32(Hash, static_cast<uint32>(Value >> 32));
	}

	void MixHashDouble(uint32& Hash, const double Value)
	{
		uint64 Bits = 0;
		FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		MixHashUInt64(Hash, Bits);
	}

	void MixHashFloat(uint32& Hash, const float Value)
	{
		uint32 Bits = 0;
		FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		MixHashUInt32(Hash, Bits);
	}

	void MixHashVector(uint32& Hash, const FVector3d& Value)
	{
		MixHashDouble(Hash, Value.X);
		MixHashDouble(Hash, Value.Y);
		MixHashDouble(Hash, Value.Z);
	}

	uint32 HashSidecarAuthorityState(const FTectonicPlanetSidecar& Sidecar)
	{
		uint32 Hash = 2166136261u;
		const FTectonicSidecarConfig& Config = Sidecar.GetConfig();
		MixHashInt32(Hash, Config.SampleCount);
		MixHashInt32(Hash, Config.PlateCount);
		MixHashInt32(Hash, Config.Seed);
		MixHashDouble(Hash, Config.PlanetRadiusKm);
		MixHashDouble(Hash, Config.DeltaTimeMy);
		MixHashFloat(Hash, Config.InitialContinentalFraction);
		MixHashFloat(Hash, Config.BoundaryWarpAmplitude);
		MixHashDouble(Hash, Config.MinPlateSpeedKmPerMy);
		MixHashDouble(Hash, Config.MaxPlateSpeedKmPerMy);
		MixHashUInt32(Hash, Config.bForceZeroAngularSpeeds ? 1u : 0u);
		MixHashUInt32(Hash, static_cast<uint32>(Config.ProjectionMode));
		MixHashDouble(Hash, Config.RecoveryToleranceRad);
		MixHashDouble(Hash, Config.DiagnosticOverlapContainmentScore);
		MixHashDouble(Hash, Config.DivergenceMinKmPerMy);
		MixHashDouble(Hash, Config.DivergenceSpeedFraction);
		MixHashInt32(Hash, Config.RidgeGenerationGapSteps);
		MixHashDouble(Hash, Config.OverlayContinentalWeightThreshold);
		MixHashUInt32(Hash, Config.bEnableDivergentSpreadingEvents ? 1u : 0u);
		MixHashDouble(Hash, Config.DivergentSpreadingMinKmPerMy);
		MixHashUInt32(Hash, Config.bEnableDOceanCrustProjection ? 1u : 0u);
		MixHashUInt32(Hash, Config.bForceExplicitProjectionAtRestForTest ? 1u : 0u);
		MixHashInt32(Hash, Sidecar.GetCurrentStep());
		const FTectonicPlanet& InitialPlanet = Sidecar.GetInitialPlanet();
		MixHashInt32(Hash, InitialPlanet.CurrentStep);
		MixHashInt32(Hash, InitialPlanet.Samples.Num());
		for (const FSample& Sample : InitialPlanet.Samples)
		{
			MixHashVector(Hash, Sample.Position);
			MixHashInt32(Hash, Sample.PlateId);
			MixHashFloat(Hash, Sample.ContinentalWeight);
			MixHashFloat(Hash, Sample.Elevation);
			MixHashFloat(Hash, Sample.Thickness);
			MixHashFloat(Hash, Sample.Age);
			MixHashUInt32(Hash, Sample.bIsBoundary ? 1u : 0u);
		}
		MixHashInt32(Hash, InitialPlanet.TriangleIndices.Num());
		for (const FIntVector& Triangle : InitialPlanet.TriangleIndices)
		{
			MixHashInt32(Hash, Triangle.X);
			MixHashInt32(Hash, Triangle.Y);
			MixHashInt32(Hash, Triangle.Z);
		}
		for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
		{
			MixHashInt32(Hash, Plate.PlateId);
			MixHashVector(Hash, Plate.InitialCenter);
			MixHashVector(Hash, Plate.RotationAxis);
			MixHashDouble(Hash, Plate.AngularSpeedRadPerMy);
			MixHashDouble(Hash, Plate.WorldFromLocal.X);
			MixHashDouble(Hash, Plate.WorldFromLocal.Y);
			MixHashDouble(Hash, Plate.WorldFromLocal.Z);
			MixHashDouble(Hash, Plate.WorldFromLocal.W);
			MixHashDouble(Hash, Plate.InitialContinentalMass);
			MixHashDouble(Hash, Plate.InitialFootprintContinentalMass);
			MixHashDouble(Hash, Plate.SupportDistanceThresholdRad);
			MixHashVector(Hash, Plate.InitialCoreCentroidLocal);
			MixHashInt32(Hash, Plate.InitialContinentalCoreSampleCount);
			MixHashInt32(Hash, Plate.MaterialSamples.Num());
			for (const FTectonicSidecarMaterialSample& Sample : Plate.MaterialSamples)
			{
				MixHashVector(Hash, Sample.LocalPosition);
				MixHashInt32(Hash, Sample.InitialCanonicalSampleIndex);
				MixHashFloat(Hash, Sample.ContinentalWeight);
				MixHashFloat(Hash, Sample.Elevation);
				MixHashFloat(Hash, Sample.Thickness);
				MixHashFloat(Hash, Sample.Age);
			}
			MixHashInt32(Hash, Plate.MaterialGrid.LongitudeBins);
			MixHashInt32(Hash, Plate.MaterialGrid.LatitudeBins);
			MixHashInt32(Hash, Plate.MaterialGrid.Bins.Num());
			for (const TArray<int32>& Bin : Plate.MaterialGrid.Bins)
			{
				MixHashInt32(Hash, Bin.Num());
				for (const int32 SampleIndex : Bin)
				{
					MixHashInt32(Hash, SampleIndex);
				}
			}
			MixHashInt32(Hash, Plate.FootprintTriangles.Num());
			for (const FTectonicSidecarFootprintTriangle& Triangle : Plate.FootprintTriangles)
			{
				MixHashInt32(Hash, Triangle.GlobalTriangleIndex);
				MixHashDouble(Hash, Triangle.UnitSphereArea);
				for (const FTectonicSidecarFootprintVertex& Vertex : Triangle.Vertices)
				{
					MixHashVector(Hash, Vertex.LocalPosition);
					MixHashInt32(Hash, Vertex.InitialCanonicalSampleIndex);
					MixHashFloat(Hash, Vertex.ContinentalWeight);
					MixHashFloat(Hash, Vertex.Elevation);
					MixHashFloat(Hash, Vertex.Thickness);
					MixHashFloat(Hash, Vertex.Age);
				}
			}
		}
		MixHashUInt32(Hash, Sidecar.GetOceanCrustStore().ComputeCanonicalHash());
		MixHashUInt32(Hash, Sidecar.GetCrustEventLog().ComputeAppendOrderHash());
		return Hash;
	}

	uint32 HashProjectionDiagnosticsForTest(const FTectonicSidecarProjectionDiagnostics& D)
	{
		uint32 Hash = 2166136261u;
		MixHashInt32(Hash, D.Step);
		MixHashInt32(Hash, D.SampleCount);
		MixHashInt32(Hash, D.PlateCount);
		MixHashUInt32(Hash, D.ProjectionHash);
		MixHashDouble(Hash, D.LocalContinentalMass);
		MixHashDouble(Hash, D.ProjectedContinentalMass);
		MixHashDouble(Hash, D.MaterialMassRelativeError);
		MixHashDouble(Hash, D.LocalFootprintContinentalMass);
		MixHashDouble(Hash, D.MultiHitProjectedContinentalMass);
		MixHashDouble(Hash, D.VisibleProjectedContinentalMass);
		MixHashDouble(Hash, D.FootprintMassRelativeError);
		MixHashDouble(Hash, D.FabricatedMaterialFraction);
		MixHashDouble(Hash, D.GapFraction);
		MixHashDouble(Hash, D.DivergentGapFraction);
		MixHashDouble(Hash, D.NonDivergentGapFraction);
		MixHashDouble(Hash, D.OverlapFraction);
		MixHashDouble(Hash, D.OceanFillFraction);
		MixHashDouble(Hash, D.LargestDivergentGapComponentFraction);
		MixHashDouble(Hash, D.PlateBoundaryFraction);
		MixHashDouble(Hash, D.DivergentBoundaryFraction);
		MixHashInt32(Hash, D.ExactFootprintHitSampleCount);
		MixHashInt32(Hash, D.RecoveredFootprintSampleCount);
		MixHashInt32(Hash, D.GapSampleCount);
		MixHashInt32(Hash, D.DivergentGapSampleCount);
		MixHashInt32(Hash, D.NonDivergentGapSampleCount);
		MixHashInt32(Hash, D.OverlapSampleCount);
		MixHashInt32(Hash, D.OceanFillSampleCount);
		MixHashInt32(Hash, D.FabricatedMaterialSampleCount);
		MixHashInt32(Hash, D.ClassifiedSampleCount);
		MixHashInt32(Hash, D.OutOfSupportSampleCount);
		MixHashDouble(Hash, D.OutOfSupportFraction);
		MixHashDouble(Hash, D.MaxSupportDistanceKm);
		MixHashDouble(Hash, D.MeanSupportDistanceKm);
		MixHashDouble(Hash, D.MeanProjectedLocalCoreDriftKm);
		MixHashDouble(Hash, D.MaxProjectedLocalCoreDriftKm);
		MixHashDouble(Hash, D.MeanKinematicDriftKm);
		MixHashDouble(Hash, D.MeanProjectedDriftKm);
		MixHashDouble(Hash, D.MeanDriftMismatchKm);
		MixHashDouble(Hash, D.MaxDriftMismatchKm);
		MixHashInt32(Hash, D.DriftPlateCount);
		MixHashDouble(Hash, D.BoundaryFraction);
		MixHashDouble(Hash, D.LargePlateMeanDominantComponentFraction);
		MixHashDouble(Hash, D.TinyComponentFraction);
		MixHashInt32(Hash, D.RegionComponentCount);
		MixHashInt32(Hash, D.MultiComponentPlateCount);
		MixHashDouble(Hash, D.ColdStartPlateMismatchFraction);
		MixHashDouble(Hash, D.ColdStartContinentalMeanAbsError);
		MixHashDouble(Hash, D.ColdStartBoundaryFractionDelta);
		MixHashDouble(Hash, D.OwnershipGapFraction);
		MixHashDouble(Hash, D.OwnershipOverlapFraction);
		MixHashDouble(Hash, D.OwnerFragmentationFraction);
		MixHashDouble(Hash, D.BoundaryNoiseFraction);
		MixHashDouble(Hash, D.CarriedContinentalMassRelativeError);
		MixHashDouble(Hash, D.MaterialFabricatedFraction);
		MixHashDouble(Hash, D.MaterialOwnerMismatchFraction);
		MixHashDouble(Hash, D.MaterialOverlapFraction);
		MixHashDouble(Hash, D.HighCWFragmentationDelta);
		MixHashInt32(Hash, D.OwnershipGapSampleCount);
		MixHashInt32(Hash, D.OwnershipOverlapSampleCount);
		MixHashInt32(Hash, D.OwnerFragmentedSampleCount);
		MixHashInt32(Hash, D.BoundaryNoiseSampleCount);
		MixHashInt32(Hash, D.MaterialOwnerMismatchSampleCount);
		MixHashInt32(Hash, D.MaterialOverlapSupportSampleCount);
		return Hash;
	}

	bool CheckProjectedSampleArraysEqual(
		FAutomationTestBase& Test,
		const FString& Label,
		const FTectonicPlanet& A,
		const FTectonicPlanet& B)
	{
		if (A.Samples.Num() != B.Samples.Num())
		{
			Test.AddError(FString::Printf(TEXT("%s sample count mismatch %d != %d"), *Label, A.Samples.Num(), B.Samples.Num()));
			return false;
		}
		for (int32 SampleIndex = 0; SampleIndex < A.Samples.Num(); ++SampleIndex)
		{
			const FSample& SA = A.Samples[SampleIndex];
			const FSample& SB = B.Samples[SampleIndex];
			const bool bEqual =
				(SA.Position - SB.Position).SizeSquared() <= 1.0e-24 &&
				SA.PlateId == SB.PlateId &&
				SA.ContinentalWeight == SB.ContinentalWeight &&
				SA.Elevation == SB.Elevation &&
				SA.Thickness == SB.Thickness &&
				SA.Age == SB.Age &&
				(SA.RidgeDirection - SB.RidgeDirection).SizeSquared() <= 1.0e-24 &&
				(SA.FoldDirection - SB.FoldDirection).SizeSquared() <= 1.0e-24 &&
				SA.OrogenyType == SB.OrogenyType &&
				SA.TerraneId == SB.TerraneId &&
				SA.SubductionDistanceKm == SB.SubductionDistanceKm &&
				SA.bIsBoundary == SB.bIsBoundary;
			if (!bEqual)
			{
				Test.AddError(FString::Printf(
					TEXT("%s sample %d differed PlateId %d/%d CW %.9f/%.9f Elev %.9f/%.9f Thick %.9f/%.9f Age %.9f/%.9f Boundary %d/%d RidgeDelta %.12g FoldDelta %.12g Subduction %.9f/%.9f Orogeny %d/%d Terrane %d/%d PosRad %.12g"),
					*Label,
					SampleIndex,
					SA.PlateId,
					SB.PlateId,
					SA.ContinentalWeight,
					SB.ContinentalWeight,
					SA.Elevation,
					SB.Elevation,
					SA.Thickness,
					SB.Thickness,
					SA.Age,
					SB.Age,
					SA.bIsBoundary ? 1 : 0,
					SB.bIsBoundary ? 1 : 0,
					(SA.RidgeDirection - SB.RidgeDirection).Size(),
					(SA.FoldDirection - SB.FoldDirection).Size(),
					SA.SubductionDistanceKm,
					SB.SubductionDistanceKm,
					static_cast<int32>(SA.OrogenyType),
					static_cast<int32>(SB.OrogenyType),
					SA.TerraneId,
					SB.TerraneId,
					(SA.Position - SB.Position).Size()));
				return false;
			}
		}
		return true;
	}

	const FTectonicSidecarPlate* FindSidecarPlateByIdForTest(
		const FTectonicPlanetSidecar& Sidecar,
		const int32 PlateId)
	{
		for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
		{
			if (Plate.PlateId == PlateId)
			{
				return &Plate;
			}
		}
		return nullptr;
	}

	int32 FindStrongContinentalPlateIdForTest(const FTectonicPlanetSidecar& Sidecar)
	{
		int32 BestPlateId = INDEX_NONE;
		double BestMass = 0.0;
		for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
		{
			if (Plate.InitialContinentalMass > BestMass)
			{
				BestMass = Plate.InitialContinentalMass;
				BestPlateId = Plate.PlateId;
			}
		}
		return BestPlateId;
	}

	bool ComputePlateLocalContinentalCentroidForTest(
		const FTectonicSidecarPlate& Plate,
		FVector3d& OutCentroid,
		double& OutWeight)
	{
		FVector3d WeightedSum = FVector3d::ZeroVector;
		double WeightSum = 0.0;
		for (const FTectonicSidecarMaterialSample& Sample : Plate.MaterialSamples)
		{
			const double Weight = FMath::Max(0.0f, Sample.ContinentalWeight);
			if (Weight <= UE_DOUBLE_SMALL_NUMBER)
			{
				continue;
			}
			WeightedSum += Sample.LocalPosition.GetSafeNormal() * Weight;
			WeightSum += Weight;
		}

		OutCentroid = WeightSum > UE_DOUBLE_SMALL_NUMBER
			? WeightedSum.GetSafeNormal()
			: FVector3d::ZeroVector;
		OutWeight = WeightSum;
		return !OutCentroid.IsNearlyZero();
	}

	FVector3d ComputeProjectedContinentalCentroidForTest(
		const FTectonicPlanet& Planet,
		const TArray<int32>* MaterialSourcePlateIds = nullptr,
		const int32 SourcePlateId = INDEX_NONE)
	{
		FVector3d WeightedSum = FVector3d::ZeroVector;
		double WeightSum = 0.0;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			if (SourcePlateId != INDEX_NONE)
			{
				if (MaterialSourcePlateIds == nullptr ||
					!MaterialSourcePlateIds->IsValidIndex(SampleIndex) ||
					(*MaterialSourcePlateIds)[SampleIndex] != SourcePlateId)
				{
					continue;
				}
			}

			const FSample& Sample = Planet.Samples[SampleIndex];
			const double Weight = FMath::Max(0.0f, Sample.ContinentalWeight);
			if (Weight <= UE_DOUBLE_SMALL_NUMBER)
			{
				continue;
			}
			WeightedSum += Sample.Position.GetSafeNormal() * Weight;
			WeightSum += Weight;
		}
		return WeightSum > UE_DOUBLE_SMALL_NUMBER
			? WeightedSum.GetSafeNormal()
			: FVector3d::ZeroVector;
	}

	int32 ComputeExpectedVoronoiOwnerForTest(
		const FTectonicPlanetSidecar& Sidecar,
		const FVector3d& UnitPosition)
	{
		int32 BestPlateId = INDEX_NONE;
		double BestScore = -TNumericLimits<double>::Max();
		for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
		{
			const FVector3d CurrentCenter = Plate.WorldFromLocal.RotateVector(Plate.InitialCenter).GetSafeNormal();
			const double Score = FVector3d::DotProduct(UnitPosition, CurrentCenter);
			if (Score > BestScore ||
				(FMath::IsNearlyEqual(Score, BestScore) && (BestPlateId == INDEX_NONE || Plate.PlateId < BestPlateId)))
			{
				BestScore = Score;
				BestPlateId = Plate.PlateId;
			}
		}
		return BestPlateId;
	}

	bool CheckPrototypeCIndependentOwnershipAndBoundaries(
		FAutomationTestBase& Test,
		const FString& Label,
		const FTectonicPlanetSidecar& Sidecar,
		const FTectonicPlanet& Planet)
	{
		TArray<int32> ExpectedOwners;
		ExpectedOwners.SetNum(Planet.Samples.Num());

		int32 OwnerMismatchCount = 0;
		int32 FirstOwnerMismatchIndex = INDEX_NONE;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const int32 ExpectedOwner = ComputeExpectedVoronoiOwnerForTest(
				Sidecar,
				Planet.Samples[SampleIndex].Position.GetSafeNormal());
			ExpectedOwners[SampleIndex] = ExpectedOwner;
			if (Planet.Samples[SampleIndex].PlateId != ExpectedOwner)
			{
				++OwnerMismatchCount;
				if (FirstOwnerMismatchIndex == INDEX_NONE)
				{
					FirstOwnerMismatchIndex = SampleIndex;
				}
			}
		}

		TArray<uint8> ExpectedBoundaryFlags;
		ExpectedBoundaryFlags.SetNumZeroed(Planet.Samples.Num());
		int32 BoundaryMismatchCount = 0;
		int32 FirstBoundaryMismatchIndex = INDEX_NONE;
		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			bool bExpectedBoundary = false;
			if (Planet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				for (const int32 NeighborIndex : Planet.SampleAdjacency[SampleIndex])
				{
					if (ExpectedOwners.IsValidIndex(NeighborIndex) &&
						ExpectedOwners[NeighborIndex] != ExpectedOwners[SampleIndex])
					{
						bExpectedBoundary = true;
						break;
					}
				}
			}
			ExpectedBoundaryFlags[SampleIndex] = bExpectedBoundary ? 1 : 0;

			if (Planet.Samples[SampleIndex].bIsBoundary != bExpectedBoundary)
			{
				++BoundaryMismatchCount;
				if (FirstBoundaryMismatchIndex == INDEX_NONE)
				{
					FirstBoundaryMismatchIndex = SampleIndex;
				}
			}
		}

		Test.TestEqual(
			FString::Printf(TEXT("%s independently recomputed Voronoi owners match every sample"), *Label),
			OwnerMismatchCount,
			0);
		Test.TestEqual(
			FString::Printf(TEXT("%s independently recomputed raw adjacency boundaries match every sample"), *Label),
			BoundaryMismatchCount,
			0);

		if (OwnerMismatchCount > 0 && Planet.Samples.IsValidIndex(FirstOwnerMismatchIndex))
		{
			Test.AddError(FString::Printf(
				TEXT("%s first owner mismatch sample=%d projected=%d expected=%d"),
				*Label,
				FirstOwnerMismatchIndex,
				Planet.Samples[FirstOwnerMismatchIndex].PlateId,
				ExpectedOwners[FirstOwnerMismatchIndex]));
		}
		if (BoundaryMismatchCount > 0 && Planet.Samples.IsValidIndex(FirstBoundaryMismatchIndex))
		{
			Test.AddError(FString::Printf(
				TEXT("%s first boundary mismatch sample=%d projected=%s expected=%s"),
				*Label,
				FirstBoundaryMismatchIndex,
				Planet.Samples[FirstBoundaryMismatchIndex].bIsBoundary ? TEXT("true") : TEXT("false"),
				ExpectedBoundaryFlags.IsValidIndex(FirstBoundaryMismatchIndex) && ExpectedBoundaryFlags[FirstBoundaryMismatchIndex] != 0
					? TEXT("true")
					: TEXT("false")));
		}

		return OwnerMismatchCount == 0 && BoundaryMismatchCount == 0;
	}

	void SetUniformPlateKinematicsForTest(
		FTectonicPlanetSidecar& Sidecar,
		const FVector3d& Axis,
		const double AngularSpeedRadPerMy)
	{
		TArray<int32> PlateIds;
		for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
		{
			PlateIds.Add(Plate.PlateId);
		}
		for (const int32 PlateId : PlateIds)
		{
			Sidecar.SetPlateKinematicsForTest(PlateId, Axis, AngularSpeedRadPerMy);
		}
	}

	struct FPrototypeCResolutionSummary
	{
		double Step0MeanContinentalWeight = 0.0;
		double StepNMeanContinentalWeight = 0.0;
		double ContinentalCentroidDriftKm = 0.0;
		double BoundaryFraction = 0.0;
		double CarriedMassRelativeError = 0.0;
	};

	FPrototypeCResolutionSummary BuildPrototypeCResolutionSummary(
		const FTectonicSidecarConfig& Config,
		const int32 TargetStep)
	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(Config);

		FTectonicPlanet Step0Projected;
		FTectonicSidecarProjectionDiagnostics Step0Diagnostics;
		Sidecar.ProjectToPlanet(Step0Projected, &Step0Diagnostics);

		Sidecar.AdvanceSteps(TargetStep);
		FTectonicPlanet StepNProjected;
		FTectonicSidecarProjectionDiagnostics StepNDiagnostics;
		Sidecar.ProjectToPlanet(StepNProjected, &StepNDiagnostics);

		FPrototypeCResolutionSummary Summary;
		Summary.Step0MeanContinentalWeight = Step0Diagnostics.VisibleProjectedContinentalMass / (4.0 * PI);
		Summary.StepNMeanContinentalWeight = StepNDiagnostics.VisibleProjectedContinentalMass / (4.0 * PI);
		Summary.ContinentalCentroidDriftKm = StepNDiagnostics.MeanProjectedDriftKm;
		Summary.BoundaryFraction = StepNDiagnostics.BoundaryFraction;
		Summary.CarriedMassRelativeError = StepNDiagnostics.CarriedContinentalMassRelativeError;
		return Summary;
	}

	bool ExportScalarOverlayFromInts(
		FAutomationTestBase& Test,
		const FString& Prefix,
		const FTectonicPlanet& Planet,
		const TArray<int32>& Values,
		const float MinValue,
		const float MaxValue,
		const FString& OutputPath)
	{
		TArray<float> FloatValues;
		FloatValues.Reserve(Values.Num());
		for (const int32 Value : Values)
		{
			FloatValues.Add(static_cast<float>(Value));
		}

		FString Error;
		if (!TectonicMollweideExporter::ExportScalarOverlay(
			Planet,
			FloatValues,
			MinValue,
			MaxValue,
			OutputPath,
			SidecarTestExportWidth,
			SidecarTestExportHeight,
			Error))
		{
			Test.AddError(FString::Printf(TEXT("[%s] scalar overlay failed path=%s error=%s"), *Prefix, *OutputPath, *Error));
			return false;
		}
		return true;
	}

	bool ExportScalarOverlayFromFloats(
		FAutomationTestBase& Test,
		const FString& Prefix,
		const FTectonicPlanet& Planet,
		const TArray<float>& Values,
		const float MinValue,
		const float MaxValue,
		const FString& OutputPath)
	{
		FString Error;
		if (!TectonicMollweideExporter::ExportScalarOverlay(
			Planet,
			Values,
			MinValue,
			MaxValue,
			OutputPath,
			SidecarTestExportWidth,
			SidecarTestExportHeight,
			Error))
		{
			Test.AddError(FString::Printf(TEXT("[%s] scalar overlay failed path=%s error=%s"), *Prefix, *OutputPath, *Error));
			return false;
		}
		return true;
	}

	bool ExportScalarOverlayFromBytes(
		FAutomationTestBase& Test,
		const FString& Prefix,
		const FTectonicPlanet& Planet,
		const TArray<uint8>& Values,
		const float MinValue,
		const float MaxValue,
		const FString& OutputPath)
	{
		TArray<int32> IntValues;
		IntValues.Reserve(Values.Num());
		for (const uint8 Value : Values)
		{
			IntValues.Add(static_cast<int32>(Value));
		}
		return ExportScalarOverlayFromInts(Test, Prefix, Planet, IntValues, MinValue, MaxValue, OutputPath);
	}

	bool ExportSidecarCCheckpointMaps(
		FAutomationTestBase& Test,
		const FTectonicPlanetSidecar& Sidecar,
		const FTectonicPlanet& Planet,
		const FString& ExportRoot,
		const int32 Step,
		const FString& Label)
	{
		const FString Prefix = TEXT("SidecarPrototypeC");
		const FString OutputDirectory = FPaths::Combine(
			ExportRoot,
			Label,
			FString::Printf(TEXT("step_%03d"), Step));
		if (!ExportSidecarCheckpointMaps(Test, Prefix, Planet, ExportRoot, Step, Label))
		{
			return false;
		}

		bool bExported = true;
		bExported &= ExportScalarOverlayFromInts(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastMaterialSourcePlateIds(),
			-1.0f,
			static_cast<float>(FMath::Max(1, Sidecar.GetConfig().PlateCount)),
			FPaths::Combine(OutputDirectory, TEXT("MaterialSource.png")));
		bExported &= ExportScalarOverlayFromBytes(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastMaterialClassifications(),
			0.0f,
			static_cast<float>(static_cast<uint8>(ETectonicSidecarMaterialClassification::DivergentOceanFill)),
			FPaths::Combine(OutputDirectory, TEXT("MaterialClassification.png")));
		bExported &= ExportScalarOverlayFromBytes(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastMaterialOwnerMismatchFlags(),
			0.0f,
			1.0f,
			FPaths::Combine(OutputDirectory, TEXT("MaterialOwnerMismatch.png")));
		bExported &= ExportScalarOverlayFromInts(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastMaterialOverlapCounts(),
			0.0f,
			4.0f,
			FPaths::Combine(OutputDirectory, TEXT("MaterialOverlap.png")));
		bExported &= ExportScalarOverlayFromBytes(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastDivergentBoundaryFlags(),
			0.0f,
			1.0f,
			FPaths::Combine(OutputDirectory, TEXT("DivergentBoundary.png")));
		const TCHAR* MandatoryOverlayNames[] = {
			TEXT("MaterialSource.png"),
			TEXT("MaterialClassification.png"),
			TEXT("MaterialOwnerMismatch.png"),
			TEXT("MaterialOverlap.png"),
			TEXT("DivergentBoundary.png")
		};
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		for (const TCHAR* OverlayName : MandatoryOverlayNames)
		{
			const FString OverlayPath = FPaths::Combine(OutputDirectory, OverlayName);
			if (!PlatformFile.FileExists(*OverlayPath))
			{
				Test.AddError(FString::Printf(TEXT("[%s] missing mandatory sidecar overlay %s"), *Prefix, *OverlayPath));
				bExported = false;
			}
		}
		return bExported;
	}

	bool ExportSidecarDOverlays(
		FAutomationTestBase& Test,
		const FTectonicPlanetSidecar& Sidecar,
		const FTectonicPlanet& Planet,
		const FString& OutputDirectory)
	{
		const FString Prefix = TEXT("SidecarPrototypeD");
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*OutputDirectory);

		bool bExported = true;
		bExported &= ExportScalarOverlayFromInts(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastOceanCrustIds(),
			-1.0f,
			FMath::Max(1.0f, static_cast<float>(Sidecar.GetOceanCrustStore().NextCrustId)),
			FPaths::Combine(OutputDirectory, TEXT("OceanCrustId.png")));
		bExported &= ExportScalarOverlayFromFloats(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastOceanCrustAgesMy(),
			0.0f,
			FMath::Max(1.0f, static_cast<float>(Planet.CurrentStep * Sidecar.GetConfig().DeltaTimeMy)),
			FPaths::Combine(OutputDirectory, TEXT("OceanCrustAge.png")));
		bExported &= ExportScalarOverlayFromFloats(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastOceanCrustThicknessKm(),
			0.0f,
			10.0f,
			FPaths::Combine(OutputDirectory, TEXT("OceanCrustThickness.png")));
		bExported &= ExportScalarOverlayFromBytes(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastCrustEventOverlayFlags(),
			0.0f,
			1.0f,
			FPaths::Combine(OutputDirectory, TEXT("CrustEventOverlay.png")));
		bExported &= ExportScalarOverlayFromBytes(
			Test,
			Prefix,
			Planet,
			Sidecar.GetLastDivergentBoundaryFlags(),
			0.0f,
			1.0f,
			FPaths::Combine(OutputDirectory, TEXT("DivergentBoundary.png")));

		const TCHAR* MandatoryOverlayNames[] = {
			TEXT("OceanCrustAge.png"),
			TEXT("OceanCrustThickness.png"),
			TEXT("OceanCrustId.png"),
			TEXT("CrustEventOverlay.png"),
			TEXT("DivergentBoundary.png")
		};
		for (const TCHAR* OverlayName : MandatoryOverlayNames)
		{
			const FString OverlayPath = FPaths::Combine(OutputDirectory, OverlayName);
			if (!PlatformFile.FileExists(*OverlayPath))
			{
				Test.AddError(FString::Printf(TEXT("[%s] missing mandatory D overlay %s"), *Prefix, *OverlayPath));
				bExported = false;
			}
		}
		return bExported;
	}

	bool CheckSidecarADiagnosticGate(
		FAutomationTestBase& Test,
		const FTectonicSidecarProjectionDiagnostics& D)
	{
		bool bPassed = true;
		auto Require = [&Test, &bPassed](const bool bCondition, const FString& Message)
		{
			if (!bCondition)
			{
				Test.AddError(Message);
				bPassed = false;
			}
		};
		auto LogMotionEvidence = [&Test](const bool bCondition, const FString& Message)
		{
			if (!bCondition)
			{
				Test.AddInfo(FString::Printf(TEXT("[SidecarPrototypeAExpectedMotionFailure] %s"), *Message));
			}
		};

		if (D.Step == 0)
		{
			Require(D.ColdStartPlateMismatchFraction <= 0.0,
				FString::Printf(TEXT("Cold-start plate mismatch was %.6f"), D.ColdStartPlateMismatchFraction));
			Require(D.ColdStartContinentalMeanAbsError <= 1.0e-6,
				FString::Printf(TEXT("Cold-start continental mismatch was %.8f"), D.ColdStartContinentalMeanAbsError));
			Require(D.ColdStartBoundaryFractionDelta <= 1.0e-6,
				FString::Printf(TEXT("Cold-start boundary delta was %.8f"), D.ColdStartBoundaryFractionDelta));
			return bPassed;
		}

		LogMotionEvidence(D.MaterialMassRelativeError <= 0.05,
			FString::Printf(TEXT("material conservation would fail promotion at step %d: %.5f"), D.Step, D.MaterialMassRelativeError));
		LogMotionEvidence(D.OutOfSupportFraction <= (D.Step <= 100 ? 0.05 : 0.10),
			FString::Printf(TEXT("out-of-support would fail promotion at step %d: %.5f"), D.Step, D.OutOfSupportFraction));
		LogMotionEvidence(D.MeanProjectedLocalCoreDriftKm <= 250.0,
			FString::Printf(TEXT("mean plate-local drift would fail promotion at step %d: %.2f km"), D.Step, D.MeanProjectedLocalCoreDriftKm));
		LogMotionEvidence(D.TinyComponentFraction <= 0.05,
			FString::Printf(TEXT("tiny component fraction would fail promotion at step %d: %.5f"), D.Step, D.TinyComponentFraction));
		LogMotionEvidence(D.LargePlateMeanDominantComponentFraction >= 0.90,
			FString::Printf(TEXT("dominant component fraction would fail promotion at step %d: %.5f"), D.Step, D.LargePlateMeanDominantComponentFraction));
		return bPassed;
	}

	void AdvanceToStep(FTectonicPlanetSidecar& Sidecar, const int32 TargetStep)
	{
		const int32 RemainingSteps = TargetStep - Sidecar.GetCurrentStep();
		if (RemainingSteps > 0)
		{
			Sidecar.AdvanceSteps(RemainingSteps);
		}
	}

	FTectonicSidecarConfig MakeSidecarConfig(
		const ETectonicSidecarProjectionMode ProjectionMode,
		const int32 SampleCount = 60000,
		const int32 PlateCount = 40)
	{
		FTectonicSidecarConfig Config;
		Config.SampleCount = SampleCount;
		Config.PlateCount = PlateCount;
		Config.Seed = 42;
		Config.InitialContinentalFraction = 0.30f;
		Config.BoundaryWarpAmplitude =
			ProjectionMode == ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial ? 0.0f : 0.12f;
		Config.ProjectionMode = ProjectionMode;
		return Config;
	}

	bool CheckProjectionValidity(
		FAutomationTestBase& Test,
		const FString& Label,
		const FTectonicPlanet& Planet,
		const FTectonicSidecarProjectionDiagnostics& D)
	{
		bool bValid = true;
		auto Require = [&Test, &bValid](const bool bCondition, const FString& Message)
		{
			if (!bCondition)
			{
				Test.AddError(Message);
				bValid = false;
			}
		};

		Require(D.FabricatedMaterialSampleCount == 0,
			FString::Printf(TEXT("%s fabricated material count was %d"), *Label, D.FabricatedMaterialSampleCount));
		Require(D.ClassifiedSampleCount == Planet.Samples.Num(),
			FString::Printf(
				TEXT("%s classified count mismatch classified=%d samples=%d"),
				*Label,
				D.ClassifiedSampleCount,
				Planet.Samples.Num()));
		Require(D.ExactFootprintHitSampleCount + D.RecoveredFootprintSampleCount + D.GapSampleCount == Planet.Samples.Num(),
			FString::Printf(TEXT("%s exact+recovered+gap did not sum to sample count"), *Label));

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const FSample& Sample = Planet.Samples[SampleIndex];
			const bool bFinite =
				FMath::IsFinite(Sample.Position.X) &&
				FMath::IsFinite(Sample.Position.Y) &&
				FMath::IsFinite(Sample.Position.Z) &&
				FMath::IsFinite(Sample.ContinentalWeight) &&
				FMath::IsFinite(Sample.Elevation) &&
				FMath::IsFinite(Sample.Thickness) &&
				FMath::IsFinite(Sample.Age);
			Require(bFinite, FString::Printf(TEXT("%s sample %d had a non-finite field"), *Label, SampleIndex));
			Require(Planet.FindPlateArrayIndexById(Sample.PlateId) != INDEX_NONE,
				FString::Printf(TEXT("%s sample %d had invalid plate id %d"), *Label, SampleIndex, Sample.PlateId));
			if (!bFinite || Planet.FindPlateArrayIndexById(Sample.PlateId) == INDEX_NONE)
			{
				break;
			}
		}

		return bValid;
	}

	bool CheckPrototypeCProjectionValidity(
		FAutomationTestBase& Test,
		const FString& Label,
		const FTectonicPlanet& Planet,
		const FTectonicSidecarProjectionDiagnostics& D)
	{
		bool bValid = true;
		auto Require = [&Test, &bValid](const bool bCondition, const FString& Message)
		{
			if (!bCondition)
			{
				Test.AddError(Message);
				bValid = false;
			}
		};

		Require(D.ClassifiedSampleCount == Planet.Samples.Num(),
			FString::Printf(TEXT("%s classified count mismatch classified=%d samples=%d"), *Label, D.ClassifiedSampleCount, Planet.Samples.Num()));
		Require(D.OwnershipGapSampleCount == 0 && D.OwnershipGapFraction <= 0.0,
			FString::Printf(TEXT("%s ownership gap fraction was %.6f"), *Label, D.OwnershipGapFraction));
		Require(D.OwnershipOverlapSampleCount == 0 && D.OwnershipOverlapFraction <= 0.0,
			FString::Printf(TEXT("%s ownership overlap fraction was %.6f"), *Label, D.OwnershipOverlapFraction));
		Require(D.MaterialFabricatedFraction <= 0.0 && D.FabricatedMaterialSampleCount == 0,
			FString::Printf(TEXT("%s material fabricated fraction was %.6f count=%d"), *Label, D.MaterialFabricatedFraction, D.FabricatedMaterialSampleCount));

		for (int32 SampleIndex = 0; SampleIndex < Planet.Samples.Num(); ++SampleIndex)
		{
			const FSample& Sample = Planet.Samples[SampleIndex];
			const bool bFinite =
				FMath::IsFinite(Sample.Position.X) &&
				FMath::IsFinite(Sample.Position.Y) &&
				FMath::IsFinite(Sample.Position.Z) &&
				FMath::IsFinite(Sample.ContinentalWeight) &&
				FMath::IsFinite(Sample.Elevation) &&
				FMath::IsFinite(Sample.Thickness) &&
				FMath::IsFinite(Sample.Age);
			Require(bFinite, FString::Printf(TEXT("%s sample %d had a non-finite field"), *Label, SampleIndex));
			Require(Planet.FindPlateArrayIndexById(Sample.PlateId) != INDEX_NONE,
				FString::Printf(TEXT("%s sample %d had invalid owner plate id %d"), *Label, SampleIndex, Sample.PlateId));
			if (!bFinite || Planet.FindPlateArrayIndexById(Sample.PlateId) == INDEX_NONE)
			{
				break;
			}
		}

		return bValid;
	}

	bool CheckPrototypeCOwnershipGate(
		FAutomationTestBase& Test,
		const FString& Label,
		const FTectonicSidecarProjectionDiagnostics& D,
		const double Step0BoundaryFraction)
	{
		bool bValid = true;
		auto Require = [&Test, &bValid](const bool bCondition, const FString& Message)
		{
			if (!bCondition)
			{
				Test.AddError(Message);
				bValid = false;
			}
		};

		const double BoundaryLimit = FMath::Min(0.14, Step0BoundaryFraction + 0.02);
		Require(D.OwnershipGapFraction <= 0.0,
			FString::Printf(TEXT("%s ownership gap fraction was %.6f"), *Label, D.OwnershipGapFraction));
		Require(D.OwnershipOverlapFraction <= 0.0,
			FString::Printf(TEXT("%s ownership overlap fraction was %.6f"), *Label, D.OwnershipOverlapFraction));
		Require(D.BoundaryFraction <= BoundaryLimit,
			FString::Printf(TEXT("%s boundary fraction %.6f exceeded limit %.6f"), *Label, D.BoundaryFraction, BoundaryLimit));
		Require(D.OwnerFragmentationFraction <= 0.005,
			FString::Printf(TEXT("%s owner fragmentation fraction was %.6f"), *Label, D.OwnerFragmentationFraction));
		Require(D.BoundaryNoiseFraction <= 0.02,
			FString::Printf(TEXT("%s boundary noise fraction was %.6f"), *Label, D.BoundaryNoiseFraction));
		Require(D.MaterialFabricatedFraction <= 0.0,
			FString::Printf(TEXT("%s material fabricated fraction was %.6f"), *Label, D.MaterialFabricatedFraction));
		const double MaterialMassLimit = D.SampleCount >= 200000 ? 0.05 : 0.13;
		Require(D.CarriedContinentalMassRelativeError <= MaterialMassLimit,
			FString::Printf(TEXT("%s carried continental mass relative error %.5f exceeded limit %.5f"), *Label, D.CarriedContinentalMassRelativeError, MaterialMassLimit));
		Require(D.HighCWFragmentationDelta <= 0.05,
			FString::Printf(TEXT("%s high-CW fragmentation delta was %.6f"), *Label, D.HighCWFragmentationDelta));
		return bValid;
	}

	bool CheckPrototypeCSourceContaminationGate(FAutomationTestBase& Test)
	{
		const TCHAR* SourcePaths[] = {
			TEXT("Source/Aurous/Public/TectonicPlanetSidecar.h"),
			TEXT("Source/Aurous/Private/TectonicPlanetSidecar.cpp"),
			TEXT("Source/Aurous/Public/TectonicPlanetSidecarActor.h"),
			TEXT("Source/Aurous/Private/TectonicPlanetSidecarActor.cpp")
		};
		const TCHAR* ForbiddenTokens[] = {
			TEXT("TectonicPlanetV6"),
			TEXT("QueryOwnership"),
			TEXT("TriggerEventResampling"),
			TEXT("EGapResolutionPath"),
			TEXT("NonDivergentFallbackOceanized"),
			TEXT("PreserveOwnership"),
			TEXT("QuietInterior"),
			TEXT("ActiveZone"),
			TEXT("ThesisRemesh"),
			TEXT("GenericQueryCompetition"),
			TEXT("RetainedSyntheticCoverage"),
			TEXT("InteriorAdvection")
		};

		bool bPassed = true;
		for (const TCHAR* RelativePath : SourcePaths)
		{
			const FString FullPath = FPaths::Combine(FPaths::ProjectDir(), RelativePath);
			FString Contents;
			if (!FFileHelper::LoadFileToString(Contents, *FullPath))
			{
				Test.AddError(FString::Printf(TEXT("Prototype C contamination gate could not read %s"), *FullPath));
				bPassed = false;
				continue;
			}
			for (const TCHAR* Token : ForbiddenTokens)
			{
				if (Contents.Contains(Token))
				{
					Test.AddError(FString::Printf(
						TEXT("Prototype C contamination gate found forbidden token '%s' in %s"),
						Token,
						*FullPath));
					bPassed = false;
				}
			}
		}
		return bPassed;
	}

	bool FindStrongBoundaryPlatePair(
		const FTectonicPlanetSidecar& Sidecar,
		int32& OutPlateA,
		int32& OutPlateB,
		FVector3d& OutCenterA,
		FVector3d& OutCenterB,
		FVector3d& OutBoundaryPoint)
	{
		TMap<int64, int32> BoundaryEdgeCountsByPair;
		const FTectonicPlanet& InitialPlanet = Sidecar.GetInitialPlanet();
		for (int32 SampleIndex = 0; SampleIndex < InitialPlanet.Samples.Num(); ++SampleIndex)
		{
			if (!InitialPlanet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 PlateA = InitialPlanet.Samples[SampleIndex].PlateId;
			for (const int32 NeighborIndex : InitialPlanet.SampleAdjacency[SampleIndex])
			{
				if (!InitialPlanet.Samples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				const int32 PlateB = InitialPlanet.Samples[NeighborIndex].PlateId;
				if (PlateA == PlateB || PlateA == INDEX_NONE || PlateB == INDEX_NONE)
				{
					continue;
				}

				const int32 LowPlateId = FMath::Min(PlateA, PlateB);
				const int32 HighPlateId = FMath::Max(PlateA, PlateB);
				const int64 PairKey = (static_cast<int64>(LowPlateId) << 32) | static_cast<uint32>(HighPlateId);
				++BoundaryEdgeCountsByPair.FindOrAdd(PairKey);
			}
		}

		int64 BestPairKey = 0;
		int32 BestBoundaryEdgeCount = 0;
		for (const TPair<int64, int32>& PairEntry : BoundaryEdgeCountsByPair)
		{
			if (PairEntry.Value > BestBoundaryEdgeCount)
			{
				BestPairKey = PairEntry.Key;
				BestBoundaryEdgeCount = PairEntry.Value;
			}
		}

		if (BestBoundaryEdgeCount <= 0)
		{
			return false;
		}

		OutPlateA = static_cast<int32>(BestPairKey >> 32);
		OutPlateB = static_cast<int32>(BestPairKey & 0xffffffff);
		for (int32 SampleIndex = 0; SampleIndex < InitialPlanet.Samples.Num(); ++SampleIndex)
		{
			if (!InitialPlanet.SampleAdjacency.IsValidIndex(SampleIndex))
			{
				continue;
			}

			const int32 SamplePlateId = InitialPlanet.Samples[SampleIndex].PlateId;
			for (const int32 NeighborIndex : InitialPlanet.SampleAdjacency[SampleIndex])
			{
				if (!InitialPlanet.Samples.IsValidIndex(NeighborIndex))
				{
					continue;
				}

				const int32 NeighborPlateId = InitialPlanet.Samples[NeighborIndex].PlateId;
				if ((SamplePlateId == OutPlateA && NeighborPlateId == OutPlateB) ||
					(SamplePlateId == OutPlateB && NeighborPlateId == OutPlateA))
				{
					OutBoundaryPoint = NormalizeOrFallbackForTest(
						InitialPlanet.Samples[SampleIndex].Position.GetSafeNormal() +
						InitialPlanet.Samples[NeighborIndex].Position.GetSafeNormal(),
						InitialPlanet.Samples[SampleIndex].Position.GetSafeNormal());
					break;
				}
			}

			if (!OutBoundaryPoint.IsNearlyZero())
			{
				break;
			}
		}
		for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
		{
			if (Plate.PlateId == OutPlateA)
			{
				OutCenterA = Plate.InitialCenter.GetSafeNormal();
			}
			else if (Plate.PlateId == OutPlateB)
			{
				OutCenterB = Plate.InitialCenter.GetSafeNormal();
			}
		}

		return OutPlateA != INDEX_NONE &&
			OutPlateB != INDEX_NONE &&
			!OutCenterA.IsNearlyZero() &&
			!OutCenterB.IsNearlyZero() &&
			!OutBoundaryPoint.IsNearlyZero();
	}

	void ZeroAllPlateKinematics(FTectonicPlanetSidecar& Sidecar)
	{
		TArray<int32> PlateIds;
		for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
		{
			PlateIds.Add(Plate.PlateId);
		}
		for (const int32 PlateId : PlateIds)
		{
			Sidecar.SetPlateKinematicsForTest(PlateId, FVector3d(0.0, 0.0, 1.0), 0.0);
		}
	}

	void ConfigureControlledPair(
		FTectonicPlanetSidecar& Sidecar,
		const int32 PlateA,
		const int32 PlateB,
		const FVector3d& CenterA,
		const FVector3d& CenterB,
		const FVector3d& BoundaryPoint,
		const bool bDivergent)
	{
		ZeroAllPlateKinematics(Sidecar);
		const FVector3d UnitCenterA = NormalizeOrFallbackForTest(CenterA, BoundaryPoint);
		const FVector3d UnitCenterB = NormalizeOrFallbackForTest(CenterB, BoundaryPoint);
		if (!bDivergent)
		{
			const FVector3d AxisA = NormalizeOrFallbackForTest(
				FVector3d::CrossProduct(UnitCenterA, UnitCenterB),
				MakeTangentFallbackForTest(UnitCenterA));
			const double AngleRad = FMath::Acos(FMath::Clamp(UnitCenterA.Dot(UnitCenterB), -1.0, 1.0));
			const double AngularSpeedRadPerMy = AngleRad / FMath::Max(Sidecar.GetConfig().DeltaTimeMy, 1.0e-6);
			Sidecar.SetPlateKinematicsForTest(PlateA, AxisA, AngularSpeedRadPerMy);
			Sidecar.SetPlateKinematicsForTest(PlateB, AxisA, 0.0);
			return;
		}

		const FVector3d DirectionAtoB = NormalizeOrFallbackForTest(
			ProjectOntoTangentForTest(UnitCenterB - UnitCenterA, UnitCenterA),
			MakeTangentFallbackForTest(UnitCenterA));
		const FVector3d DirectionBtoA = NormalizeOrFallbackForTest(
			ProjectOntoTangentForTest(UnitCenterA - UnitCenterB, UnitCenterB),
			MakeTangentFallbackForTest(UnitCenterB));
		const FVector3d VelocityDirectionA = bDivergent ? -DirectionAtoB : DirectionAtoB;
		const FVector3d VelocityDirectionB = bDivergent ? -DirectionBtoA : DirectionBtoA;
		const FVector3d AxisA = NormalizeOrFallbackForTest(
			FVector3d::CrossProduct(UnitCenterA, VelocityDirectionA),
			MakeTangentFallbackForTest(UnitCenterA));
		const FVector3d AxisB = NormalizeOrFallbackForTest(
			FVector3d::CrossProduct(UnitCenterB, VelocityDirectionB),
			MakeTangentFallbackForTest(UnitCenterB));
		const double AngularSpeedRadPerMy =
			ControlledPairSpeedKmPerMy / FMath::Max(Sidecar.GetConfig().PlanetRadiusKm, 1.0);
		Sidecar.SetPlateKinematicsForTest(PlateA, AxisA, AngularSpeedRadPerMy);
		Sidecar.SetPlateKinematicsForTest(PlateB, AxisB, AngularSpeedRadPerMy);
	}

	bool FindOwnerEdgeForPair(
		const FTectonicPlanetSidecar& Sidecar,
		const int32 PlateA,
		const int32 PlateB,
		FSidecarOwnerEdge& OutEdge)
	{
		const int32 LowPlateId = FMath::Min(PlateA, PlateB);
		const int32 HighPlateId = FMath::Max(PlateA, PlateB);
		for (const FSidecarOwnerEdge& Edge : Sidecar.EnumerateOwnerEdgesSorted())
		{
			if (Edge.LowPlateId == LowPlateId && Edge.HighPlateId == HighPlateId)
			{
				OutEdge = Edge;
				return true;
			}
		}
		return false;
	}

	FSidecarDivergentSpreadingEventInput MakeDivergentEventInputForTest(
		const FSidecarOwnerEdge& Edge,
		const int32 PlateA,
		const int32 PlateB,
		const double NormalSeparationKmPerMy)
	{
		FSidecarOceanCrustBirthEdge BirthEdge;
		BirthEdge.Key = Edge.Key;
		BirthEdge.EndpointA = Edge.SampleAPosition * 2.0;
		BirthEdge.EndpointB = Edge.SampleBPosition * 3.0;
		BirthEdge.BoundaryNormal = NormalizeOrFallbackForTest(
			FVector3d::CrossProduct(Edge.Midpoint, Edge.SampleBPosition - Edge.SampleAPosition),
			MakeTangentFallbackForTest(Edge.Midpoint));
		BirthEdge.LengthKm = Edge.LengthRad * 6371.0;
		BirthEdge.NormalSeparationKmPerMy = NormalSeparationKmPerMy;

		FSidecarDivergentSpreadingEventInput Input;
		Input.PlateA = PlateA;
		Input.PlateB = PlateB;
		Input.ComponentId = Edge.Key;
		Input.RidgeGenerationId = 7;
		Input.BirthEdges.Add(BirthEdge);
		Input.DebugSampleIds = { Edge.Key.SampleB, Edge.Key.SampleA };
		Input.RidgeDirection = BirthEdge.BoundaryNormal;
		Input.CreatedAreaKm2 = 123.0;
		Input.OceanicAgeMy = 0.0;
		Input.OceanicThicknessKm = 7.0;
		Input.ElevationSeedKm = -1.0;
		return Input;
	}

	double ComputePlateCenterSeparationKmPerMyForTest(
		const FTectonicPlanetSidecar& Sidecar,
		int32 LowPlateId,
		int32 HighPlateId);

	double ComputeExpectedDivergentAreaKm2ForTest(const FTectonicPlanetSidecar& Sidecar)
	{
		double AreaKm2 = 0.0;
		const FTectonicSidecarConfig& Config = Sidecar.GetConfig();
		for (const FSidecarOwnerEdge& Edge : Sidecar.EnumerateOwnerEdgesSorted())
		{
			double NormalSeparationKmPerMy = 0.0;
			const double PairSeparationKmPerMy = ComputePlateCenterSeparationKmPerMyForTest(
				Sidecar,
				Edge.LowPlateId,
				Edge.HighPlateId);
			if (PairSeparationKmPerMy >= Config.DivergentSpreadingMinKmPerMy &&
				Sidecar.ComputeBoundaryNormalSeparationKmPerMy(Edge, NormalSeparationKmPerMy) &&
				NormalSeparationKmPerMy >= Config.DivergentSpreadingMinKmPerMy)
			{
				AreaKm2 +=
					Edge.LengthRad *
					Config.PlanetRadiusKm *
					NormalSeparationKmPerMy *
					Config.DeltaTimeMy;
			}
		}
		return AreaKm2;
	}

	double SumEventAreaForStep(const FSidecarCrustEventLog& EventLog, const int32 Step)
	{
		double AreaKm2 = 0.0;
		for (const FSidecarCrustEventRecord& Event : EventLog.Events)
		{
			if (Event.Step == Step)
			{
				AreaKm2 += Event.CreatedAreaKm2;
			}
		}
		return AreaKm2;
	}

	const FSidecarOceanCrustRecord* FindCrustRecordByIdForTest(
		const FSidecarOceanCrustStore& Store,
		const int32 CrustId)
	{
		for (const FSidecarOceanCrustRecord& Record : Store.Records)
		{
			if (Record.CrustId == CrustId)
			{
				return &Record;
			}
		}
		return nullptr;
	}

	double ComputePlateCenterSeparationKmPerMyForTest(
		const FTectonicPlanetSidecar& Sidecar,
		const int32 LowPlateId,
		const int32 HighPlateId)
	{
		const FTectonicSidecarPlate* LowPlate = FindSidecarPlateByIdForTest(Sidecar, LowPlateId);
		const FTectonicSidecarPlate* HighPlate = FindSidecarPlateByIdForTest(Sidecar, HighPlateId);
		if (LowPlate == nullptr || HighPlate == nullptr)
		{
			return 0.0;
		}

		const FVector3d LowCenter =
			LowPlate->WorldFromLocal.RotateVector(LowPlate->InitialCenter).GetSafeNormal();
		const FVector3d HighCenter =
			HighPlate->WorldFromLocal.RotateVector(HighPlate->InitialCenter).GetSafeNormal();
		const FVector3d QueryPoint = NormalizeOrFallbackForTest(LowCenter + HighCenter, LowCenter);
		const FVector3d LowToHigh = ProjectOntoTangentForTest(HighCenter - LowCenter, QueryPoint).GetSafeNormal();
		if (LowToHigh.IsNearlyZero())
		{
			return 0.0;
		}

		const FVector3d LowVelocity =
			FVector3d::CrossProduct(LowPlate->RotationAxis.GetSafeNormal(), QueryPoint) *
			(LowPlate->AngularSpeedRadPerMy * Sidecar.GetConfig().PlanetRadiusKm);
		const FVector3d HighVelocity =
			FVector3d::CrossProduct(HighPlate->RotationAxis.GetSafeNormal(), QueryPoint) *
			(HighPlate->AngularSpeedRadPerMy * Sidecar.GetConfig().PlanetRadiusKm);
		return (HighVelocity - LowVelocity).Dot(LowToHigh);
	}

	bool CheckPrototypeDSourceHygieneGate(FAutomationTestBase& Test)
	{
		const TCHAR* SourcePaths[] =
		{
			TEXT("Source/Aurous/Public/TectonicSidecarCrust.h"),
			TEXT("Source/Aurous/Private/TectonicSidecarCrust.cpp")
		};
		const TCHAR* ForbiddenTokens[] =
		{
			TEXT("TectonicPlanetV6"),
			TEXT("QueryOwnership"),
			TEXT("ActiveZone"),
			TEXT("QuietInterior"),
			TEXT("OceanFill"),
			TEXT("Recovered"),
			TEXT("Recover"),
			TEXT("Repair"),
			TEXT("Heal"),
			TEXT("Backfill"),
			TEXT("Resync"),
			TEXT("Promote"),
			TEXT("Reclassify")
		};

		bool bPassed = true;
		for (const TCHAR* RelativePath : SourcePaths)
		{
			const FString FullPath = FPaths::Combine(FPaths::ProjectDir(), RelativePath);
			FString Contents;
			if (!FFileHelper::LoadFileToString(Contents, *FullPath))
			{
				Test.AddError(FString::Printf(TEXT("Prototype D source hygiene could not read %s"), *FullPath));
				bPassed = false;
				continue;
			}
			for (const TCHAR* Token : ForbiddenTokens)
			{
				if (Contents.Contains(Token))
				{
					Test.AddError(FString::Printf(
						TEXT("Prototype D source hygiene found forbidden token '%s' in %s"),
						Token,
						*FullPath));
					bPassed = false;
				}
			}
		}

		const FString SidecarHeaderPath = FPaths::Combine(
			FPaths::ProjectDir(),
			TEXT("Source/Aurous/Public/TectonicPlanetSidecar.h"));
		FString SidecarHeader;
		if (!FFileHelper::LoadFileToString(SidecarHeader, *SidecarHeaderPath))
		{
			Test.AddError(TEXT("Prototype D source hygiene could not read TectonicPlanetSidecar.h"));
			return false;
		}

		if (!SidecarHeader.Contains(TEXT("ApplyDivergentSpreadingEventForTest")))
		{
			Test.AddError(TEXT("Prototype D source hygiene expected the test-only Apply...EventForTest seeding API"));
			bPassed = false;
		}
		if (SidecarHeader.Contains(TEXT("ApplyDivergentSpreadingRepair")) ||
			SidecarHeader.Contains(TEXT("ApplyDivergentSpreadingRecovery")) ||
			SidecarHeader.Contains(TEXT("ApplyDivergentSpreadingPromote")))
		{
			Test.AddError(TEXT("Prototype D source hygiene found repair-shaped Apply API in sidecar header"));
			bPassed = false;
		}

		const FString SidecarCppPath = FPaths::Combine(
			FPaths::ProjectDir(),
			TEXT("Source/Aurous/Private/TectonicPlanetSidecar.cpp"));
		FString SidecarCpp;
		if (!FFileHelper::LoadFileToString(SidecarCpp, *SidecarCppPath))
		{
			Test.AddError(TEXT("Prototype D source hygiene could not read TectonicPlanetSidecar.cpp"));
			return false;
		}
		const FString StartMarker = TEXT("void FTectonicPlanetSidecar::ApplyDivergentSpreadingEventsForCurrentStep()");
		const FString EndMarker = TEXT("void FTectonicPlanetSidecar::ProjectToPlanet");
		const int32 StartIndex = SidecarCpp.Find(StartMarker);
		const int32 EndIndex = SidecarCpp.Find(EndMarker);
		if (StartIndex == INDEX_NONE || EndIndex == INDEX_NONE || EndIndex <= StartIndex)
		{
			Test.AddError(TEXT("Prototype D source hygiene could not isolate event detection region"));
			bPassed = false;
		}
		else
		{
			const FString DetectionRegion = SidecarCpp.Mid(StartIndex, EndIndex - StartIndex);
			const TCHAR* DetectionForbiddenTokens[] =
			{
				TEXT("LastDivergentBoundaryFlags"),
				TEXT("OceanFallback"),
				TEXT("DivergentOceanFill"),
				TEXT("DiagnosticOverlapContainmentScore"),
				TEXT("QueryOwnership"),
				TEXT("Repair"),
				TEXT("Recover"),
				TEXT("Backfill"),
				TEXT("Promote"),
				TEXT("Reclassify")
			};
			for (const TCHAR* Token : DetectionForbiddenTokens)
			{
				if (DetectionRegion.Contains(Token))
				{
					Test.AddError(FString::Printf(
						TEXT("Prototype D detection hygiene found forbidden token '%s'"),
						Token));
					bPassed = false;
				}
			}
		}

		return bPassed;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSidecarPrototypeATest,
	"Aurous.TectonicPlanet.SidecarPrototypeA",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSidecarPrototypeATest::RunTest(const FString& Parameters)
{
	const FString RunId = TEXT("SidecarPrototypeA");
	const FString ExportRoot = BuildSidecarExportRoot(RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);

	{
		FTectonicPlanetSidecar A;
		FTectonicPlanetSidecar B;
		const FTectonicSidecarConfig Config =
			MakeSidecarConfig(ETectonicSidecarProjectionMode::MaterialSupportDiagnostic);
		A.Initialize(Config);
		B.Initialize(Config);
		A.AdvanceSteps(100);
		B.AdvanceSteps(100);

		FTectonicPlanet ProjectedA;
		FTectonicPlanet ProjectedB;
		FTectonicSidecarProjectionDiagnostics DiagnosticsA;
		FTectonicSidecarProjectionDiagnostics DiagnosticsB;
		A.ProjectToPlanet(ProjectedA, &DiagnosticsA);
		B.ProjectToPlanet(ProjectedB, &DiagnosticsB);
		TestEqual(TEXT("Prototype A deterministic projection hash at step 100"), DiagnosticsA.ProjectionHash, DiagnosticsB.ProjectionHash);
	}

	{
		FTectonicSidecarConfig Config =
			MakeSidecarConfig(ETectonicSidecarProjectionMode::MaterialSupportDiagnostic);
		Config.bForceZeroAngularSpeeds = true;

		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(Config);
		FTectonicPlanet Projected0;
		FTectonicSidecarProjectionDiagnostics Diagnostics0;
		Sidecar.ProjectToPlanet(Projected0, &Diagnostics0);

		Sidecar.AdvanceSteps(100);
		FTectonicPlanet Projected100;
		FTectonicSidecarProjectionDiagnostics Diagnostics100;
		Sidecar.ProjectToPlanet(Projected100, &Diagnostics100);

		TestEqual(TEXT("Prototype A zero-motion projection hash remains stable"), Diagnostics0.ProjectionHash, Diagnostics100.ProjectionHash);
	}

	FTectonicPlanetSidecar Sidecar;
	Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::MaterialSupportDiagnostic));

	const int32 Checkpoints[] = { 0, 100, 200, 400 };
	for (const int32 Checkpoint : Checkpoints)
	{
		AdvanceToStep(Sidecar, Checkpoint);
		FTectonicPlanet Projected;
		FTectonicSidecarProjectionDiagnostics Diagnostics;
		Sidecar.ProjectToPlanet(Projected, &Diagnostics);
		AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeA"), TEXT("60k40_diagnostic"), Diagnostics);
		CheckSidecarADiagnosticGate(*this, Diagnostics);
		ExportSidecarCheckpointMaps(*this, TEXT("SidecarPrototypeA"), Projected, ExportRoot, Checkpoint, TEXT("60k40_diagnostic"));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSidecarPrototypeBTest,
	"Aurous.TectonicPlanet.SidecarPrototypeB",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSidecarPrototypeBTest::RunTest(const FString& Parameters)
{
	const FString RunId = TEXT("SidecarPrototypeB");
	const FString ExportRoot = BuildSidecarExportRoot(RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);

	double IdentityNoiseFloor = 0.0;

	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::ExplicitFootprints));
		FTectonicPlanet Projected;
		FTectonicSidecarProjectionDiagnostics Diagnostics;
		Sidecar.ProjectToPlanet(Projected, &Diagnostics);
		AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeB"), TEXT("cold_start"), Diagnostics);
		TestEqual(TEXT("Prototype B cold-start hash matches initial V6 cold start"),
			Diagnostics.ProjectionHash,
			FTectonicPlanetSidecar::ComputeProjectionHash(Sidecar.GetInitialPlanet()));
		TestTrue(TEXT("Prototype B cold-start plate parity"), Diagnostics.ColdStartPlateMismatchFraction <= 0.0);
		TestTrue(TEXT("Prototype B cold-start continental parity"), Diagnostics.ColdStartContinentalMeanAbsError <= 1.0e-6);
		TestTrue(TEXT("Prototype B cold-start boundary parity"), Diagnostics.ColdStartBoundaryFractionDelta <= 1.0e-6);
	}

	{
		FTectonicSidecarConfig Config = MakeSidecarConfig(ETectonicSidecarProjectionMode::ExplicitFootprints);
		Config.bForceExplicitProjectionAtRestForTest = true;
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(Config);

		FTectonicPlanet Projected;
		FTectonicSidecarProjectionDiagnostics Diagnostics;
		Sidecar.ProjectToPlanet(Projected, &Diagnostics);
		IdentityNoiseFloor = Diagnostics.FootprintMassRelativeError;
		AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeB"), TEXT("forced_identity_calibration"), Diagnostics);
		CheckProjectionValidity(*this, TEXT("forced_identity_calibration"), Projected, Diagnostics);
		TestTrue(
			TEXT("Prototype B forced identity projection mass error is within calibration noise"),
			IdentityNoiseFloor <= 0.01);
	}

	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::ExplicitFootprints));
		const double MotionMassLimit = FMath::Min(0.05, IdentityNoiseFloor + 0.04);

		const int32 Checkpoints[] = { 0, 100, 200, 400 };
		for (const int32 Checkpoint : Checkpoints)
		{
			AdvanceToStep(Sidecar, Checkpoint);
			FTectonicPlanet Projected;
			FTectonicSidecarProjectionDiagnostics Diagnostics;
			Sidecar.ProjectToPlanet(Projected, &Diagnostics);
			AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeB"), TEXT("60k40"), Diagnostics);
			if (Checkpoint > 0)
			{
				CheckProjectionValidity(*this, FString::Printf(TEXT("60k40_step_%d"), Checkpoint), Projected, Diagnostics);
				TestTrue(
					FString::Printf(TEXT("Prototype B motion footprint mass error step %d"), Checkpoint),
					Diagnostics.FootprintMassRelativeError <= MotionMassLimit);
				TestTrue(
					FString::Printf(TEXT("Prototype B hard mass cap step %d"), Checkpoint),
					Diagnostics.FootprintMassRelativeError <= 0.05);
				TestTrue(
					FString::Printf(TEXT("Prototype B moving projection overlap exists step %d"), Checkpoint),
					Diagnostics.OverlapSampleCount > 0);
			}
			ExportSidecarCheckpointMaps(*this, TEXT("SidecarPrototypeB"), Projected, ExportRoot, Checkpoint, TEXT("60k40"));
		}
	}

	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::ExplicitFootprints));
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d CenterA = FVector3d::ZeroVector;
		FVector3d CenterB = FVector3d::ZeroVector;
		FVector3d BoundaryPoint = FVector3d::ZeroVector;
		TestTrue(TEXT("Prototype B found controlled divergent boundary pair"),
			FindStrongBoundaryPlatePair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint));
		ConfigureControlledPair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, true);

		const int32 Checkpoints[] = { 1, 10, 25 };
		for (const int32 Checkpoint : Checkpoints)
		{
			AdvanceToStep(Sidecar, Checkpoint);
			FTectonicPlanet Projected;
			FTectonicSidecarProjectionDiagnostics Diagnostics;
			Sidecar.ProjectToPlanet(Projected, &Diagnostics);
			AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeB"), TEXT("controlled_divergent"), Diagnostics);
			CheckProjectionValidity(*this, FString::Printf(TEXT("controlled_divergent_step_%d"), Checkpoint), Projected, Diagnostics);
			TestTrue(
				FString::Printf(TEXT("Prototype B divergent gaps exist at step %d"), Checkpoint),
				Diagnostics.DivergentGapSampleCount > 0);
			TestTrue(
				FString::Printf(TEXT("Prototype B divergent ocean fill exists at step %d"), Checkpoint),
				Diagnostics.OceanFillSampleCount > 0);
			TestTrue(
				FString::Printf(TEXT("Prototype B divergent gap corridor coherent at step %d"), Checkpoint),
				Diagnostics.LargestDivergentGapComponentFraction >= 0.70);
		}
	}

	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::ExplicitFootprints));
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d CenterA = FVector3d::ZeroVector;
		FVector3d CenterB = FVector3d::ZeroVector;
		FVector3d BoundaryPoint = FVector3d::ZeroVector;
		TestTrue(TEXT("Prototype B found controlled convergent boundary pair"),
			FindStrongBoundaryPlatePair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint));
		ConfigureControlledPair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, false);
		Sidecar.AdvanceSteps(1);

		for (const FTectonicSidecarPlate& Plate : Sidecar.GetPlates())
		{
			if (Plate.PlateId == PlateA)
			{
				const FVector3d CurrentCenter =
					Plate.WorldFromLocal.RotateVector(Plate.InitialCenter).GetSafeNormal();
				const double CenterDistanceRad =
					FMath::Acos(FMath::Clamp(CurrentCenter.Dot(CenterB.GetSafeNormal()), -1.0, 1.0));
				AddInfo(FString::Printf(
					TEXT("[SidecarPrototypeBControlledConvergent] plate_a=%d plate_b=%d center_distance_rad=%.6f center_distance_km=%.2f"),
					PlateA,
					PlateB,
					CenterDistanceRad,
					CenterDistanceRad * Sidecar.GetConfig().PlanetRadiusKm));
				break;
			}
		}

		FTectonicPlanet Projected;
		FTectonicSidecarProjectionDiagnostics Diagnostics;
		Sidecar.ProjectToPlanet(Projected, &Diagnostics);
		AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeB"), TEXT("controlled_convergent"), Diagnostics);
		CheckProjectionValidity(*this, TEXT("controlled_convergent"), Projected, Diagnostics);
		if (Diagnostics.OverlapSampleCount <= 0)
		{
			AddInfo(TEXT("[SidecarPrototypeBControlledConvergent] direct center-on-center pair produced no meaningful overlap; default moving projection covers overlap gate."));
		}
		TestTrue(TEXT("Prototype B convergent ownership remains coherent"),
			Diagnostics.TinyComponentFraction <= 0.05 &&
			Diagnostics.LargePlateMeanDominantComponentFraction >= 0.80);
	}

	{
		FTectonicSidecarConfig Config = MakeSidecarConfig(
			ETectonicSidecarProjectionMode::ExplicitFootprints,
			250000,
			40);
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(Config);
		Sidecar.AdvanceSteps(40);

		FTectonicPlanet Projected;
		FTectonicSidecarProjectionDiagnostics Diagnostics;
		Sidecar.ProjectToPlanet(Projected, &Diagnostics);
		AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeB"), TEXT("250k40_smoke"), Diagnostics);
		CheckProjectionValidity(*this, TEXT("250k40_smoke"), Projected, Diagnostics);
		ExportSidecarCheckpointMaps(*this, TEXT("SidecarPrototypeB"), Projected, ExportRoot, 40, TEXT("250k40_smoke"));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSidecarPrototypeCTest,
	"Aurous.TectonicPlanet.SidecarPrototypeC",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSidecarPrototypeCTest::RunTest(const FString& Parameters)
{
	const double TestStartSeconds = FPlatformTime::Seconds();
	const FString RunId = TEXT("SidecarPrototypeC");
	const FString ExportRoot = BuildSidecarExportRoot(RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);
	CheckPrototypeCSourceContaminationGate(*this);

	{
		FTectonicPlanetSidecar A;
		FTectonicPlanetSidecar B;
		const FTectonicSidecarConfig Config =
			MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial);
		A.Initialize(Config);
		B.Initialize(Config);
		A.AdvanceSteps(100);
		B.AdvanceSteps(100);

		FTectonicPlanet ProjectedA;
		FTectonicPlanet ProjectedB;
		FTectonicSidecarProjectionDiagnostics DiagnosticsA;
		FTectonicSidecarProjectionDiagnostics DiagnosticsB;
		A.ProjectToPlanet(ProjectedA, &DiagnosticsA);
		const uint32 DebugHashA = HashSidecarDebugValues(A);
		B.ProjectToPlanet(ProjectedB, &DiagnosticsB);
		const uint32 DebugHashB = HashSidecarDebugValues(B);
		TestEqual(TEXT("Prototype C deterministic projection hash at step 100"), DiagnosticsA.ProjectionHash, DiagnosticsB.ProjectionHash);
		TestEqual(TEXT("Prototype C deterministic material debug hash at step 100"), DebugHashA, DebugHashB);
	}

	{
		FTectonicSidecarConfig Config =
			MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial);
		Config.bForceZeroAngularSpeeds = true;

		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(Config);
		FTectonicPlanet Projected0;
		FTectonicSidecarProjectionDiagnostics Diagnostics0;
		Sidecar.ProjectToPlanet(Projected0, &Diagnostics0);
		CheckPrototypeCIndependentOwnershipAndBoundaries(*this, TEXT("zero_motion_step_0"), Sidecar, Projected0);
		const uint32 DebugHash0 = HashSidecarDebugValues(Sidecar);

		const int32 Checkpoints[] = { 100, 200, 400 };
		for (const int32 Checkpoint : Checkpoints)
		{
			AdvanceToStep(Sidecar, Checkpoint);
			FTectonicPlanet Projected;
			FTectonicSidecarProjectionDiagnostics Diagnostics;
			Sidecar.ProjectToPlanet(Projected, &Diagnostics);
			CheckPrototypeCIndependentOwnershipAndBoundaries(
				*this,
				FString::Printf(TEXT("zero_motion_step_%d"), Checkpoint),
				Sidecar,
				Projected);
			TestEqual(
				FString::Printf(TEXT("Prototype C zero-motion projection hash remains stable at step %d"), Checkpoint),
				Diagnostics0.ProjectionHash,
				Diagnostics.ProjectionHash);
			TestEqual(
				FString::Printf(TEXT("Prototype C zero-motion debug hash remains stable at step %d"), Checkpoint),
				DebugHash0,
				HashSidecarDebugValues(Sidecar));
		}
	}

	{
		FTectonicSidecarConfig Config =
			MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial);
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(Config);

		const int32 PlateId = FindStrongContinentalPlateIdForTest(Sidecar);
		const FTectonicSidecarPlate* InitialPlate = FindSidecarPlateByIdForTest(Sidecar, PlateId);
		TestTrue(TEXT("Prototype C rigid advection found a continental source plate"), InitialPlate != nullptr);
		if (InitialPlate != nullptr)
		{
			FVector3d LocalCentroid = FVector3d::ZeroVector;
			double LocalWeight = 0.0;
			TestTrue(
				TEXT("Prototype C rigid advection found plate-local continental centroid"),
				ComputePlateLocalContinentalCentroidForTest(*InitialPlate, LocalCentroid, LocalWeight));

			FTectonicPlanet Projected0;
			FTectonicSidecarProjectionDiagnostics Diagnostics0;
			Sidecar.ProjectToPlanet(Projected0, &Diagnostics0);
			const FVector3d ProjectedCentroid0 = ComputeProjectedContinentalCentroidForTest(
				Projected0,
				&Sidecar.GetLastMaterialSourcePlateIds(),
				PlateId);

			const FVector3d Axis = NormalizeOrFallbackForTest(
				FVector3d::CrossProduct(LocalCentroid, FVector3d(0.31, 0.73, 0.61).GetSafeNormal()),
				MakeTangentFallbackForTest(LocalCentroid));
			const double AngularSpeedRadPerMy =
				35.0 / FMath::Max(Config.PlanetRadiusKm, 1.0);
			const int32 StepCount = 80;
			SetUniformPlateKinematicsForTest(Sidecar, Axis, AngularSpeedRadPerMy);
			Sidecar.AdvanceSteps(StepCount);

			const double ExpectedAngleRad =
				AngularSpeedRadPerMy * Config.DeltaTimeMy * static_cast<double>(StepCount);
			const FQuat4d ExpectedRotation(Axis, ExpectedAngleRad);
			const FVector3d ExpectedLocalTruthCentroid =
				ExpectedRotation.RotateVector(LocalCentroid).GetSafeNormal();

			const FTectonicSidecarPlate* AdvancedPlate = FindSidecarPlateByIdForTest(Sidecar, PlateId);
			TestTrue(TEXT("Prototype C rigid advection retained the source plate"), AdvancedPlate != nullptr);
			if (AdvancedPlate != nullptr)
			{
				const FVector3d ActualLocalTruthCentroid =
					AdvancedPlate->WorldFromLocal.RotateVector(LocalCentroid).GetSafeNormal();
				const double TruthErrorKm =
					AngularDistanceRadForTest(ExpectedLocalTruthCentroid, ActualLocalTruthCentroid) *
					Config.PlanetRadiusKm;
				TestTrue(
					FString::Printf(TEXT("Prototype C carried material follows analytic rigid rotation, error %.6f km"), TruthErrorKm),
					TruthErrorKm <= 1.0e-3);

				FTectonicPlanet Projected;
				FTectonicSidecarProjectionDiagnostics Diagnostics;
				Sidecar.ProjectToPlanet(Projected, &Diagnostics);
				CheckPrototypeCProjectionValidity(*this, TEXT("rigid_advection"), Projected, Diagnostics);
				CheckPrototypeCIndependentOwnershipAndBoundaries(*this, TEXT("rigid_advection"), Sidecar, Projected);

				const FVector3d ProjectedCentroid = ComputeProjectedContinentalCentroidForTest(
					Projected,
					&Sidecar.GetLastMaterialSourcePlateIds(),
					PlateId);
				const FVector3d ExpectedProjectedCentroid =
					ExpectedRotation.RotateVector(ProjectedCentroid0).GetSafeNormal();
				const double ProjectedErrorKm =
					AngularDistanceRadForTest(ExpectedProjectedCentroid, ProjectedCentroid) *
					Config.PlanetRadiusKm;
				const double ExpectedDriftKm =
					AngularDistanceRadForTest(ProjectedCentroid0, ExpectedProjectedCentroid) *
					Config.PlanetRadiusKm;
				const double SampleSpacingKm =
					FMath::Sqrt((4.0 * PI) / FMath::Max(1.0, static_cast<double>(Config.SampleCount))) *
					Config.PlanetRadiusKm;
				const double ProjectedCentroidToleranceKm = 4.0 * SampleSpacingKm;
				AddInfo(FString::Printf(
					TEXT("[SidecarPrototypeCRigidAdvection] plate=%d weight=%.2f expected_drift_km=%.2f projected_error_km=%.2f tolerance_km=%.2f"),
					PlateId,
					LocalWeight,
					ExpectedDriftKm,
					ProjectedErrorKm,
					ProjectedCentroidToleranceKm));
				TestTrue(
					TEXT("Prototype C rigid advection expected drift is meaningfully larger than lattice spacing"),
					ExpectedDriftKm >= 8.0 * SampleSpacingKm);
				TestTrue(
					FString::Printf(TEXT("Prototype C projected material centroid follows rigid rotation within lattice tolerance, error %.2f km"), ProjectedErrorKm),
					ProjectedErrorKm <= ProjectedCentroidToleranceKm);
				TestTrue(
					TEXT("Prototype C rigid advection does not fabricate material"),
					Diagnostics.MaterialFabricatedFraction <= 0.0);
				TestTrue(
					TEXT("Prototype C rigid event isolation has no generic ocean fill"),
					Diagnostics.OceanFillFraction <= 0.0);
				TestTrue(
					TEXT("Prototype C rigid event isolation has no material overlap"),
					Diagnostics.MaterialOverlapFraction <= 0.01);
				TestTrue(
					TEXT("Prototype C rigid advection has no divergent boundary signal"),
					Diagnostics.DivergentBoundaryFraction <= 1.0e-6);
			}
		}
	}

	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial));
		Sidecar.AdvanceSteps(40);

		const uint32 AuthorityHashBefore = HashSidecarAuthorityState(Sidecar);
		FTectonicPlanet ProjectedA;
		FTectonicSidecarProjectionDiagnostics DiagnosticsA;
		Sidecar.ProjectToPlanet(ProjectedA, &DiagnosticsA);
		const uint32 DebugHashA = HashSidecarDebugValues(Sidecar);
		const uint32 AuthorityHashAfterA = HashSidecarAuthorityState(Sidecar);

		FTectonicPlanet ProjectedB;
		FTectonicSidecarProjectionDiagnostics DiagnosticsB;
		Sidecar.ProjectToPlanet(ProjectedB, &DiagnosticsB);
		const uint32 DebugHashB = HashSidecarDebugValues(Sidecar);
		const uint32 AuthorityHashAfterB = HashSidecarAuthorityState(Sidecar);

		TestEqual(TEXT("Prototype C projection idempotence preserves authority on first projection"), AuthorityHashBefore, AuthorityHashAfterA);
		TestEqual(TEXT("Prototype C projection idempotence preserves authority on second projection"), AuthorityHashBefore, AuthorityHashAfterB);
		TestEqual(TEXT("Prototype C projection idempotence projection hash is stable"), DiagnosticsA.ProjectionHash, DiagnosticsB.ProjectionHash);
		TestEqual(TEXT("Prototype C projection idempotence debug hash is stable"), DebugHashA, DebugHashB);
	}

	double Step0BoundaryFraction = 0.0;
	const FPrototypeCResolutionSummary Resolution60k40 = BuildPrototypeCResolutionSummary(
		MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial),
		PrototypeCResolutionCompareStep);
	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial));
		bool bDefaultMotionProducedMaterialOverlap = false;

		const int32 Checkpoints[] = { 0, 100, 200, 400 };
		for (const int32 Checkpoint : Checkpoints)
		{
			AdvanceToStep(Sidecar, Checkpoint);
			FTectonicPlanet Projected;
			FTectonicSidecarProjectionDiagnostics Diagnostics;
			Sidecar.ProjectToPlanet(Projected, &Diagnostics);
			if (Checkpoint == 0)
			{
				Step0BoundaryFraction = Diagnostics.BoundaryFraction;
			}
			if (Checkpoint > 0 && Diagnostics.MaterialOverlapSupportSampleCount > 0)
			{
				bDefaultMotionProducedMaterialOverlap = true;
			}
			AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeC"), TEXT("60k40"), Diagnostics);
			CheckPrototypeCProjectionValidity(
				*this,
				FString::Printf(TEXT("60k40_step_%d"), Checkpoint),
				Projected,
				Diagnostics);
			CheckPrototypeCIndependentOwnershipAndBoundaries(
				*this,
				FString::Printf(TEXT("60k40_step_%d"), Checkpoint),
				Sidecar,
				Projected);
			CheckPrototypeCOwnershipGate(
				*this,
				FString::Printf(TEXT("60k40_step_%d"), Checkpoint),
				Diagnostics,
				Step0BoundaryFraction);
			ExportSidecarCCheckpointMaps(*this, Sidecar, Projected, ExportRoot, Checkpoint, TEXT("60k40"));
		}
		TestTrue(
			TEXT("Prototype C default differential motion exposes material overlap as a separate signal"),
			bDefaultMotionProducedMaterialOverlap);
	}

	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial));
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d CenterA = FVector3d::ZeroVector;
		FVector3d CenterB = FVector3d::ZeroVector;
		FVector3d BoundaryPoint = FVector3d::ZeroVector;
		TestTrue(TEXT("Prototype C found controlled divergent boundary pair"),
			FindStrongBoundaryPlatePair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint));
		ConfigureControlledPair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, true);

		const int32 Checkpoints[] = { 1, 10, 25 };
		for (const int32 Checkpoint : Checkpoints)
		{
			AdvanceToStep(Sidecar, Checkpoint);
			FTectonicPlanet Projected;
			FTectonicSidecarProjectionDiagnostics Diagnostics;
			Sidecar.ProjectToPlanet(Projected, &Diagnostics);
			AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeC"), TEXT("controlled_divergent"), Diagnostics);
			CheckPrototypeCProjectionValidity(
				*this,
				FString::Printf(TEXT("controlled_divergent_step_%d"), Checkpoint),
				Projected,
				Diagnostics);
			CheckPrototypeCIndependentOwnershipAndBoundaries(
				*this,
				FString::Printf(TEXT("controlled_divergent_step_%d"), Checkpoint),
				Sidecar,
				Projected);
			TestTrue(
				FString::Printf(TEXT("Prototype C divergent boundary diagnostic exists at step %d"), Checkpoint),
				Diagnostics.DivergentBoundaryFraction > 0.0);
			TestTrue(
				FString::Printf(TEXT("Prototype C divergent event isolation has no ownership gaps at step %d"), Checkpoint),
				Diagnostics.OwnershipGapFraction <= 0.0);
			TestTrue(
				FString::Printf(TEXT("Prototype C divergent event isolation has no generic ocean fill at step %d"), Checkpoint),
				Diagnostics.OceanFillFraction <= 0.0);
			TestTrue(
				FString::Printf(TEXT("Prototype C divergent event isolation does not create convergent material overlap at step %d"), Checkpoint),
				Diagnostics.MaterialOverlapFraction <= 0.01);
			TestTrue(
				FString::Printf(TEXT("Prototype C divergent ownership has no gaps at step %d"), Checkpoint),
				Diagnostics.OwnershipGapFraction <= 0.0);
			TestTrue(
				FString::Printf(TEXT("Prototype C divergent ownership remains coherent at step %d"), Checkpoint),
				Diagnostics.OwnerFragmentationFraction <= 0.005);
		}
	}

	{
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial));
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d CenterA = FVector3d::ZeroVector;
		FVector3d CenterB = FVector3d::ZeroVector;
		FVector3d BoundaryPoint = FVector3d::ZeroVector;
		TestTrue(TEXT("Prototype C found controlled convergent boundary pair"),
			FindStrongBoundaryPlatePair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint));
		ConfigureControlledPair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, false);
		Sidecar.AdvanceSteps(1);

		FTectonicPlanet Projected;
		FTectonicSidecarProjectionDiagnostics Diagnostics;
		Sidecar.ProjectToPlanet(Projected, &Diagnostics);
		AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeC"), TEXT("controlled_convergent"), Diagnostics);
		CheckPrototypeCProjectionValidity(*this, TEXT("controlled_convergent"), Projected, Diagnostics);
		CheckPrototypeCIndependentOwnershipAndBoundaries(*this, TEXT("controlled_convergent"), Sidecar, Projected);
		TestTrue(TEXT("Prototype C controlled convergent projection exposes material-owner mismatch"),
			Diagnostics.MaterialOwnerMismatchSampleCount > 0);
		TestTrue(TEXT("Prototype C convergent event isolation does not look like divergent boundary creation"),
			Diagnostics.DivergentBoundaryFraction <= 0.01);
		TestTrue(TEXT("Prototype C convergent event isolation has no generic ocean fill"),
			Diagnostics.OceanFillFraction <= 0.0);
		TestTrue(TEXT("Prototype C convergent ownership remains coherent"),
			Diagnostics.OwnerFragmentationFraction <= 0.005 &&
			Diagnostics.BoundaryNoiseFraction <= 0.02);
	}

	{
		FTectonicSidecarConfig Config = MakeSidecarConfig(
			ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial,
			250000,
			40);
		FTectonicPlanetSidecar BaselineSidecar;
		BaselineSidecar.Initialize(Config);
		FTectonicPlanet BaselineProjected;
		FTectonicSidecarProjectionDiagnostics BaselineDiagnostics;
		BaselineSidecar.ProjectToPlanet(BaselineProjected, &BaselineDiagnostics);

		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(Config);
		Sidecar.AdvanceSteps(40);

		FTectonicPlanet Projected;
		FTectonicSidecarProjectionDiagnostics Diagnostics;
		Sidecar.ProjectToPlanet(Projected, &Diagnostics);
		AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeC"), TEXT("250k40_smoke"), Diagnostics);

		const FPrototypeCResolutionSummary Resolution250k40{
			BaselineDiagnostics.VisibleProjectedContinentalMass / (4.0 * PI),
			Diagnostics.VisibleProjectedContinentalMass / (4.0 * PI),
			Diagnostics.MeanProjectedDriftKm,
			Diagnostics.BoundaryFraction,
			Diagnostics.CarriedContinentalMassRelativeError
		};
		AddInfo(FString::Printf(
			TEXT("[SidecarPrototypeCResolutionIndependence] step=%d 60k_mean_cw=%.5f 250k_mean_cw=%.5f 60k_drift_km=%.2f 250k_drift_km=%.2f 60k_boundary=%.5f 250k_boundary=%.5f"),
			PrototypeCResolutionCompareStep,
			Resolution60k40.StepNMeanContinentalWeight,
			Resolution250k40.StepNMeanContinentalWeight,
			Resolution60k40.ContinentalCentroidDriftKm,
			Resolution250k40.ContinentalCentroidDriftKm,
			Resolution60k40.BoundaryFraction,
			Resolution250k40.BoundaryFraction));
		TestTrue(
			TEXT("Prototype C resolution independence preserves step-0 continental area trend"),
			FMath::Abs(Resolution60k40.Step0MeanContinentalWeight - Resolution250k40.Step0MeanContinentalWeight) <= 0.03);
		TestTrue(
			TEXT("Prototype C resolution independence preserves step-40 continental area trend"),
			FMath::Abs(Resolution60k40.StepNMeanContinentalWeight - Resolution250k40.StepNMeanContinentalWeight) <= 0.04);
		TestTrue(
			TEXT("Prototype C resolution independence preserves continental centroid drift trend"),
			FMath::Abs(Resolution60k40.ContinentalCentroidDriftKm - Resolution250k40.ContinentalCentroidDriftKm) <= 100.0);
		TestTrue(
			TEXT("Prototype C resolution independence has no material fabrication at 250k"),
			Diagnostics.MaterialFabricatedFraction <= 0.0);

		CheckPrototypeCProjectionValidity(*this, TEXT("250k40_smoke"), Projected, Diagnostics);
		CheckPrototypeCIndependentOwnershipAndBoundaries(*this, TEXT("250k40_smoke"), Sidecar, Projected);
		CheckPrototypeCOwnershipGate(*this, TEXT("250k40_smoke"), Diagnostics, BaselineDiagnostics.BoundaryFraction);
		ExportSidecarCCheckpointMaps(*this, Sidecar, Projected, ExportRoot, 40, TEXT("250k40_smoke"));
	}

	const double ElapsedSeconds = FPlatformTime::Seconds() - TestStartSeconds;
	AddInfo(FString::Printf(
		TEXT("[SidecarPrototypeCRuntime] elapsed_seconds=%.2f budget_seconds=%.2f"),
		ElapsedSeconds,
		PrototypeCRuntimeBudgetSeconds));
	TestTrue(
		FString::Printf(TEXT("Prototype C automation runtime stays under %.0f seconds"), PrototypeCRuntimeBudgetSeconds),
		ElapsedSeconds <= PrototypeCRuntimeBudgetSeconds);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTectonicPlanetSidecarPrototypeDTest,
	"Aurous.TectonicPlanet.SidecarPrototypeD",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTectonicPlanetSidecarPrototypeDTest::RunTest(const FString& Parameters)
{
	CheckPrototypeDSourceHygieneGate(*this);

	const FTectonicSidecarConfig Config =
		MakeSidecarConfig(ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial);
	TestEqual(TEXT("Prototype D config ridge generation gap defaults to 5 steps"), Config.RidgeGenerationGapSteps, 5);
	TestEqual(TEXT("Prototype D config overlay continental threshold defaults to 0.5"), Config.OverlayContinentalWeightThreshold, 0.5);
	TestEqual(TEXT("Prototype D ocean crust projection defaults off"), Config.bEnableDOceanCrustProjection, false);

	FTectonicPlanetSidecar EmptyA;
	FTectonicPlanetSidecar EmptyB;
	EmptyA.Initialize(Config);
	EmptyB.Initialize(Config);

	TestEqual(TEXT("Prototype D empty crust store starts empty"), EmptyA.GetOceanCrustStore().Num(), 0);
	TestEqual(TEXT("Prototype D empty event log starts empty"), EmptyA.GetCrustEventLog().Num(), 0);
	TestEqual(
		TEXT("Prototype D empty authority hash is deterministic across identical configs"),
		EmptyA.ComputeSidecarAuthorityHash(),
		EmptyB.ComputeSidecarAuthorityHash());

	{
		FTectonicPlanet EmptyProjectedA;
		FTectonicPlanet EmptyProjectedB;
		FTectonicSidecarProjectionDiagnostics EmptyDiagnosticsA;
		FTectonicSidecarProjectionDiagnostics EmptyDiagnosticsB;
		const uint32 EmptyAuthorityBeforeProjection = EmptyA.ComputeSidecarAuthorityHash();
		EmptyA.ProjectToPlanet(EmptyProjectedA, &EmptyDiagnosticsA);
		EmptyA.ProjectToPlanet(EmptyProjectedB, &EmptyDiagnosticsB);
		CheckProjectedSampleArraysEqual(*this, TEXT("Prototype D empty Phase3/Slice2 baseline"), EmptyProjectedA, EmptyProjectedB);
		TestEqual(
			TEXT("Prototype D empty Phase3/Slice2 diagnostics struct equality"),
			HashProjectionDiagnosticsForTest(EmptyDiagnosticsA),
			HashProjectionDiagnosticsForTest(EmptyDiagnosticsB));
		TestEqual(
			TEXT("Prototype D empty Phase3/Slice2 authority hash equality"),
			EmptyAuthorityBeforeProjection,
			EmptyA.ComputeSidecarAuthorityHash());
		TestEqual(
			TEXT("Prototype D empty Phase3/Slice2 projection hash equality"),
			EmptyDiagnosticsA.ProjectionHash,
			EmptyDiagnosticsB.ProjectionHash);

		int32 NonBackgroundCrustIds = 0;
		int32 NonBackgroundCrustAges = 0;
		int32 NonBackgroundCrustThickness = 0;
		int32 NonBackgroundCrustEvents = 0;
		for (int32 SampleIndex = 0; SampleIndex < EmptyProjectedA.Samples.Num(); ++SampleIndex)
		{
			NonBackgroundCrustIds += EmptyA.GetLastOceanCrustIds()[SampleIndex] != INDEX_NONE ? 1 : 0;
			NonBackgroundCrustAges += EmptyA.GetLastOceanCrustAgesMy()[SampleIndex] != 0.0f ? 1 : 0;
			NonBackgroundCrustThickness += EmptyA.GetLastOceanCrustThicknessKm()[SampleIndex] != 0.0f ? 1 : 0;
			NonBackgroundCrustEvents += EmptyA.GetLastCrustEventOverlayFlags()[SampleIndex] != 0 ? 1 : 0;
		}
		TestEqual(TEXT("Prototype D empty ocean crust id overlay is uniform background"), NonBackgroundCrustIds, 0);
		TestEqual(TEXT("Prototype D empty ocean crust age overlay is uniform background"), NonBackgroundCrustAges, 0);
		TestEqual(TEXT("Prototype D empty ocean crust thickness overlay is uniform background"), NonBackgroundCrustThickness, 0);
		TestEqual(TEXT("Prototype D empty crust event overlay is uniform background"), NonBackgroundCrustEvents, 0);
	}

	{
		FTectonicPlanetSidecar MotionOnly;
		MotionOnly.Initialize(Config);
		MotionOnly.AdvanceSteps(5);
		TestEqual(TEXT("Prototype D AdvanceStep does not create crust"), MotionOnly.GetOceanCrustStore().Num(), 0);
		TestEqual(TEXT("Prototype D AdvanceStep does not append events"), MotionOnly.GetCrustEventLog().Num(), 0);
	}

	{
		const uint32 AuthorityHashBefore = EmptyA.ComputeSidecarAuthorityHash();
		FTectonicPlanet ProjectedA;
		FTectonicPlanet ProjectedB;
		EmptyA.ProjectToPlanet(ProjectedA);
		const uint32 AuthorityHashAfterA = EmptyA.ComputeSidecarAuthorityHash();
		EmptyA.ProjectToPlanet(ProjectedB);
		const uint32 AuthorityHashAfterB = EmptyA.ComputeSidecarAuthorityHash();
		TestEqual(TEXT("Prototype D projection idempotence preserves authority on first projection"), AuthorityHashBefore, AuthorityHashAfterA);
		TestEqual(TEXT("Prototype D projection idempotence preserves authority on second projection"), AuthorityHashBefore, AuthorityHashAfterB);
		TestEqual(TEXT("Prototype D projection idempotence preserves crust count"), EmptyA.GetOceanCrustStore().Num(), 0);
		TestEqual(TEXT("Prototype D projection idempotence preserves event count"), EmptyA.GetCrustEventLog().Num(), 0);

		if (ProjectedA.Samples.Num() > 0)
		{
			ProjectedA.Samples[0].PlateId = 123456;
			ProjectedA.Samples[0].ContinentalWeight = 0.987f;
			ProjectedA.Samples[0].Elevation = 42.0f;
			ProjectedA.Samples[0].bIsBoundary = !ProjectedA.Samples[0].bIsBoundary;
		}
		TestEqual(
			TEXT("Prototype D mutating projected FTectonicPlanet output does not affect sidecar authority"),
			AuthorityHashBefore,
			EmptyA.ComputeSidecarAuthorityHash());
	}

	const TArray<FSidecarOwnerEdge> EmptyEdges = EmptyA.EnumerateOwnerEdgesSorted();
	TestTrue(TEXT("Prototype D owner-edge helper returns edges"), EmptyEdges.Num() > 0);
	for (int32 EdgeIndex = 0; EdgeIndex < EmptyEdges.Num(); ++EdgeIndex)
	{
		const FSidecarOwnerEdge& Edge = EmptyEdges[EdgeIndex];
		TestTrue(TEXT("Prototype D owner edge key is valid"), Edge.Key.IsValid());
		TestTrue(TEXT("Prototype D owner edge crosses two owners"), Edge.LowPlateId != Edge.HighPlateId);
		if (EdgeIndex > 0)
		{
			TestTrue(TEXT("Prototype D owner edges are sorted and unique"),
				EmptyEdges[EdgeIndex - 1].Key < Edge.Key);
		}
	}

	const TArray<FSidecarOwnerEdge> EmptyEdgesRepeat = EmptyB.EnumerateOwnerEdgesSorted();
	TestEqual(TEXT("Prototype D owner-edge helper deterministic count"), EmptyEdges.Num(), EmptyEdgesRepeat.Num());
	const int32 EdgeCompareCount = FMath::Min(EmptyEdges.Num(), EmptyEdgesRepeat.Num());
	for (int32 EdgeIndex = 0; EdgeIndex < EdgeCompareCount; ++EdgeIndex)
	{
		TestEqual(TEXT("Prototype D owner-edge helper deterministic sample A"),
			EmptyEdges[EdgeIndex].Key.SampleA,
			EmptyEdgesRepeat[EdgeIndex].Key.SampleA);
		TestEqual(TEXT("Prototype D owner-edge helper deterministic sample B"),
			EmptyEdges[EdgeIndex].Key.SampleB,
			EmptyEdgesRepeat[EdgeIndex].Key.SampleB);
		TestEqual(TEXT("Prototype D owner-edge helper deterministic low plate"),
			EmptyEdges[EdgeIndex].LowPlateId,
			EmptyEdgesRepeat[EdgeIndex].LowPlateId);
		TestEqual(TEXT("Prototype D owner-edge helper deterministic high plate"),
			EmptyEdges[EdgeIndex].HighPlateId,
			EmptyEdgesRepeat[EdgeIndex].HighPlateId);
	}

	FTectonicPlanetSidecar SeedA;
	FTectonicPlanetSidecar SeedB;
	SeedA.Initialize(Config);
	SeedB.Initialize(Config);
	SeedA.AdvanceSteps(3);
	SeedB.AdvanceSteps(3);
	const TArray<FSidecarOwnerEdge> SeedEdges = SeedA.EnumerateOwnerEdgesSorted();
	TestTrue(TEXT("Prototype D seed sidecar has owner edges"), SeedEdges.Num() >= 2);
	if (SeedEdges.Num() >= 2)
	{
		const FSidecarOwnerEdge SeedEdge = SeedEdges[0];
		const FSidecarDivergentSpreadingEventInput Input =
			MakeDivergentEventInputForTest(SeedEdge, SeedEdge.HighPlateId, SeedEdge.LowPlateId, 12.5);
		const uint32 HashBeforeSeed = SeedA.ComputeSidecarAuthorityHash();
		int32 CrustId = INDEX_NONE;
		int32 EventId = INDEX_NONE;
		TestTrue(TEXT("Prototype D test-only divergent seed succeeds"),
			SeedA.ApplyDivergentSpreadingEventForTest(Input, &CrustId, &EventId));
		TestTrue(TEXT("Prototype D test-only divergent seed outputs valid crust id"), CrustId > 0);
		TestTrue(TEXT("Prototype D test-only divergent seed outputs valid event id"), EventId > 0);
		TestEqual(TEXT("Prototype D test-only divergent seed appends one crust record"), SeedA.GetOceanCrustStore().Num(), 1);
		TestEqual(TEXT("Prototype D test-only divergent seed appends one event record"), SeedA.GetCrustEventLog().Num(), 1);
		TestTrue(TEXT("Prototype D seeded state changes authority hash"),
			HashBeforeSeed != SeedA.ComputeSidecarAuthorityHash());

		const FSidecarOceanCrustRecord& CrustRecord = SeedA.GetOceanCrustStore().Records[0];
		const FSidecarCrustEventRecord& EventRecord = SeedA.GetCrustEventLog().Events[0];
		TestEqual(TEXT("Prototype D crust id starts at 1"), CrustRecord.CrustId, 1);
		TestEqual(TEXT("Prototype D event id starts at 1"), EventRecord.EventId, 1);
		TestEqual(TEXT("Prototype D crust birth step matches sidecar step"), CrustRecord.BirthStep, SeedA.GetCurrentStep());
		TestEqual(TEXT("Prototype D event step matches sidecar step"), EventRecord.Step, SeedA.GetCurrentStep());
		TestEqual(TEXT("Prototype D crust canonical low plate"), CrustRecord.PlateA, SeedEdge.LowPlateId);
		TestEqual(TEXT("Prototype D crust canonical high plate"), CrustRecord.PlateB, SeedEdge.HighPlateId);
		TestEqual(TEXT("Prototype D event canonical low plate"), EventRecord.PlateA, SeedEdge.LowPlateId);
		TestEqual(TEXT("Prototype D event canonical high plate"), EventRecord.PlateB, SeedEdge.HighPlateId);
		TestEqual(TEXT("Prototype D crust birth edge source sample A is canonicalized"),
			CrustRecord.BirthEdges[0].Key.SampleA,
			SeedEdge.Key.SampleA);
		TestEqual(TEXT("Prototype D crust birth edge source sample B is canonicalized"),
			CrustRecord.BirthEdges[0].Key.SampleB,
			SeedEdge.Key.SampleB);
		TestTrue(TEXT("Prototype D birth endpoint A is stored normalized from input"),
			AngularDistanceRadForTest(CrustRecord.BirthEdges[0].EndpointA, Input.BirthEdges[0].EndpointA.GetSafeNormal()) <= 1.0e-10);
		TestTrue(TEXT("Prototype D birth endpoint B is stored normalized from input"),
			AngularDistanceRadForTest(CrustRecord.BirthEdges[0].EndpointB, Input.BirthEdges[0].EndpointB.GetSafeNormal()) <= 1.0e-10);
		TestEqual(TEXT("Prototype D event source edge count"), EventRecord.SourceEdges.Num(), 1);
		TestEqual(TEXT("Prototype D event source edge sample A is canonicalized"),
			EventRecord.SourceEdges[0].SampleA,
			SeedEdge.Key.SampleA);
		TestEqual(TEXT("Prototype D event source edge sample B is canonicalized"),
			EventRecord.SourceEdges[0].SampleB,
			SeedEdge.Key.SampleB);
		TestEqual(TEXT("Prototype D debug sample ids are sorted"), CrustRecord.DebugSampleIds[0], SeedEdge.Key.SampleA);
		TestEqual(TEXT("Prototype D debug sample ids are sorted"), CrustRecord.DebugSampleIds[1], SeedEdge.Key.SampleB);

		int32 CrustIdB = INDEX_NONE;
		int32 EventIdB = INDEX_NONE;
		TestTrue(TEXT("Prototype D same seed succeeds on identical sidecar"),
			SeedB.ApplyDivergentSpreadingEventForTest(Input, &CrustIdB, &EventIdB));
		TestEqual(TEXT("Prototype D same seeded state gives identical authority hash"),
			SeedA.ComputeSidecarAuthorityHash(),
			SeedB.ComputeSidecarAuthorityHash());

		const FSidecarDivergentSpreadingEventInput Input2 =
			MakeDivergentEventInputForTest(SeedEdges[1], SeedEdges[1].LowPlateId, SeedEdges[1].HighPlateId, 15.0);
		TestTrue(TEXT("Prototype D second seed succeeds for hash policy"),
			SeedA.ApplyDivergentSpreadingEventForTest(Input2, &CrustId, &EventId));
		TestEqual(TEXT("Prototype D second seed appends second crust record"), SeedA.GetOceanCrustStore().Num(), 2);
		TestEqual(TEXT("Prototype D second seed appends second event record"), SeedA.GetCrustEventLog().Num(), 2);

		FSidecarOceanCrustStore StoreForward = SeedA.GetOceanCrustStore();
		FSidecarOceanCrustStore StoreReordered = StoreForward;
		Swap(StoreReordered.Records[0], StoreReordered.Records[1]);
		TestEqual(TEXT("Prototype D crust store hash is stable under record reorder"),
			StoreForward.ComputeCanonicalHash(),
			StoreReordered.ComputeCanonicalHash());

		FSidecarCrustEventLog LogForward = SeedA.GetCrustEventLog();
		FSidecarCrustEventLog LogReordered = LogForward;
		Swap(LogReordered.Events[0], LogReordered.Events[1]);
		TestTrue(TEXT("Prototype D event log hash changes when append order changes"),
			LogForward.ComputeAppendOrderHash() != LogReordered.ComputeAppendOrderHash());

		const uint32 SeedAuthorityBeforeProjection = SeedA.ComputeSidecarAuthorityHash();
		const int32 SeedCrustCountBeforeProjection = SeedA.GetOceanCrustStore().Num();
		const int32 SeedEventCountBeforeProjection = SeedA.GetCrustEventLog().Num();
		FTectonicPlanet SeedProjectedA;
		FTectonicPlanet SeedProjectedB;
		SeedA.ProjectToPlanet(SeedProjectedA);
		SeedA.ProjectToPlanet(SeedProjectedB);
		TestEqual(TEXT("Prototype D seeded projection preserves authority hash"),
			SeedAuthorityBeforeProjection,
			SeedA.ComputeSidecarAuthorityHash());
		TestEqual(TEXT("Prototype D seeded projection preserves crust count"),
			SeedCrustCountBeforeProjection,
			SeedA.GetOceanCrustStore().Num());
		TestEqual(TEXT("Prototype D seeded projection preserves event count"),
			SeedEventCountBeforeProjection,
			SeedA.GetCrustEventLog().Num());
	}

	{
		FTectonicPlanetSidecar VelocitySidecar;
		VelocitySidecar.Initialize(Config);
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d CenterA = FVector3d::ZeroVector;
		FVector3d CenterB = FVector3d::ZeroVector;
		FVector3d BoundaryPoint = FVector3d::ZeroVector;
		TestTrue(TEXT("Prototype D found controlled boundary pair"),
			FindStrongBoundaryPlatePair(VelocitySidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint));

		FSidecarOwnerEdge PairEdge;
		TestTrue(TEXT("Prototype D found owner edge for controlled pair"),
			FindOwnerEdgeForPair(VelocitySidecar, PlateA, PlateB, PairEdge));

		ConfigureControlledPair(VelocitySidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, true);
		double DivergentSpeedKmPerMy = 0.0;
		TestTrue(TEXT("Prototype D boundary velocity helper computes divergent speed"),
			VelocitySidecar.ComputeBoundaryNormalSeparationKmPerMy(PairEdge, DivergentSpeedKmPerMy));
		AddInfo(FString::Printf(TEXT("[SidecarPrototypeDBoundaryVelocity] divergent_km_per_my=%.3f"), DivergentSpeedKmPerMy));
		TestTrue(TEXT("Prototype D boundary velocity helper reports positive divergent speed"),
			DivergentSpeedKmPerMy > 1.0);

		ConfigureControlledPair(VelocitySidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, false);
		double ConvergentSpeedKmPerMy = 0.0;
		TestTrue(TEXT("Prototype D boundary velocity helper computes convergent speed"),
			VelocitySidecar.ComputeBoundaryNormalSeparationKmPerMy(PairEdge, ConvergentSpeedKmPerMy));
		AddInfo(FString::Printf(TEXT("[SidecarPrototypeDBoundaryVelocity] convergent_km_per_my=%.3f"), ConvergentSpeedKmPerMy));
		TestTrue(TEXT("Prototype D boundary velocity helper reports negative convergent speed"),
			ConvergentSpeedKmPerMy < -1.0);

		ZeroAllPlateKinematics(VelocitySidecar);
		double PerpendicularSpeedKmPerMy = 0.0;
		TestTrue(TEXT("Prototype D boundary velocity helper computes zero-relative control"),
			VelocitySidecar.ComputeBoundaryNormalSeparationKmPerMy(PairEdge, PerpendicularSpeedKmPerMy));
		AddInfo(FString::Printf(TEXT("[SidecarPrototypeDBoundaryVelocity] zero_relative_km_per_my=%.6f"), PerpendicularSpeedKmPerMy));
		TestTrue(TEXT("Prototype D boundary velocity helper reports approximately zero perpendicular/zero-relative speed"),
			FMath::Abs(PerpendicularSpeedKmPerMy) <= 1.0e-6);
	}

	{
		FTectonicSidecarConfig Default250kConfig = MakeSidecarConfig(
			ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial,
			250000,
			40);
		FTectonicPlanetSidecar Default250kSidecar;
		Default250kSidecar.Initialize(Default250kConfig);
		const int32 DefaultCheckpoints[] = { 0, 10, 25, 40 };
		for (const int32 Checkpoint : DefaultCheckpoints)
		{
			AdvanceToStep(Default250kSidecar, Checkpoint);
			TestEqual(
				FString::Printf(TEXT("Prototype D default-off 250k checkpoint %d creates zero crust"), Checkpoint),
				Default250kSidecar.GetOceanCrustStore().Num(),
				0);
			TestEqual(
				FString::Printf(TEXT("Prototype D default-off 250k checkpoint %d appends zero events"), Checkpoint),
				Default250kSidecar.GetCrustEventLog().Num(),
				0);
		}
	}

	{
		FTectonicSidecarConfig EventConfig = Config;
		EventConfig.bEnableDivergentSpreadingEvents = true;
		EventConfig.bEnableDOceanCrustProjection = true;
		EventConfig.DivergentSpreadingMinKmPerMy = 10.0;

		FTectonicSidecarConfig ExpectedConfig = EventConfig;
		ExpectedConfig.bEnableDivergentSpreadingEvents = false;

		FTectonicPlanetSidecar EventSidecar;
		FTectonicPlanetSidecar ExpectedSidecar;
		EventSidecar.Initialize(EventConfig);
		ExpectedSidecar.Initialize(ExpectedConfig);

		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d CenterA = FVector3d::ZeroVector;
		FVector3d CenterB = FVector3d::ZeroVector;
		FVector3d BoundaryPoint = FVector3d::ZeroVector;
		TestTrue(TEXT("Prototype D runtime found controlled divergent pair"),
			FindStrongBoundaryPlatePair(EventSidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint));
		ConfigureControlledPair(EventSidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, true);
		ConfigureControlledPair(ExpectedSidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, true);

		TArray<int32> InitialCrustIds;
		double PreviousTotalAreaKm2 = 0.0;
		const int32 RuntimeCheckpoints[] = { 1, 10, 25 };
		int32 NextCheckpointIndex = 0;
		for (int32 Step = 1; Step <= 25; ++Step)
		{
			ExpectedSidecar.AdvanceStep();
			const double ExpectedStepAreaKm2 = ComputeExpectedDivergentAreaKm2ForTest(ExpectedSidecar);

			EventSidecar.AdvanceStep();
			const double ActualStepAreaKm2 = SumEventAreaForStep(EventSidecar.GetCrustEventLog(), EventSidecar.GetCurrentStep());
			const double AreaToleranceKm2 = FMath::Max(1.0e-6, ExpectedStepAreaKm2 * 0.01);
			TestTrue(
				FString::Printf(TEXT("Prototype D runtime analytic area step %d expected=%.6f actual=%.6f tolerance=%.6f"),
					Step,
					ExpectedStepAreaKm2,
					ActualStepAreaKm2,
					AreaToleranceKm2),
				FMath::Abs(ExpectedStepAreaKm2 - ActualStepAreaKm2) <= AreaToleranceKm2);

			if (Step == 1)
			{
				TestTrue(TEXT("Prototype D runtime divergent pair creates persistent crust at R+1"),
					EventSidecar.GetOceanCrustStore().Num() > 0);
				TestTrue(TEXT("Prototype D runtime divergent pair appends event records at R+1"),
					EventSidecar.GetCrustEventLog().Num() > 0);
				for (const FSidecarOceanCrustRecord& Record : EventSidecar.GetOceanCrustStore().Records)
				{
					InitialCrustIds.Add(Record.CrustId);
				}
			}

			if (NextCheckpointIndex < UE_ARRAY_COUNT(RuntimeCheckpoints) &&
				Step == RuntimeCheckpoints[NextCheckpointIndex])
			{
				double TotalAreaKm2 = 0.0;
				for (const FSidecarOceanCrustRecord& Record : EventSidecar.GetOceanCrustStore().Records)
				{
					TotalAreaKm2 += Record.CreatedAreaKm2;
					const double ExpectedAgeMy =
						static_cast<double>(EventSidecar.GetCurrentStep() - Record.BirthStep) *
						EventConfig.DeltaTimeMy;
					TestTrue(
						FString::Printf(TEXT("Prototype D runtime crust age matches current step at R+%d"), Step),
						FMath::Abs(Record.AgeMy - ExpectedAgeMy) <= 1.0e-6);
				}
				TestTrue(
					FString::Printf(TEXT("Prototype D runtime cumulative crust area is monotonic at R+%d"), Step),
					TotalAreaKm2 + 1.0e-6 >= PreviousTotalAreaKm2);
				PreviousTotalAreaKm2 = TotalAreaKm2;

				for (const int32 CrustId : InitialCrustIds)
				{
					TestTrue(
						FString::Printf(TEXT("Prototype D runtime initial crust id %d persists at R+%d"), CrustId, Step),
						FindCrustRecordByIdForTest(EventSidecar.GetOceanCrustStore(), CrustId) != nullptr);
				}
				++NextCheckpointIndex;
			}
		}

		const uint32 AuthorityBeforeProjection = EventSidecar.ComputeSidecarAuthorityHash();
		const int32 CrustCountBeforeProjection = EventSidecar.GetOceanCrustStore().Num();
		const int32 EventCountBeforeProjection = EventSidecar.GetCrustEventLog().Num();
		FTectonicPlanet BaselineProjected;
		FTectonicSidecarProjectionDiagnostics BaselineProjectionDiagnostics;
		ExpectedSidecar.ProjectToPlanet(BaselineProjected, &BaselineProjectionDiagnostics);
		FTectonicPlanet Projected;
		FTectonicSidecarProjectionDiagnostics ProjectedDiagnostics;
		EventSidecar.ProjectToPlanet(Projected, &ProjectedDiagnostics);
		TestEqual(TEXT("Prototype D runtime projection remains idempotent with D state"),
			AuthorityBeforeProjection,
			EventSidecar.ComputeSidecarAuthorityHash());
		TestEqual(TEXT("Prototype D runtime projection preserves crust count"),
			CrustCountBeforeProjection,
			EventSidecar.GetOceanCrustStore().Num());
		TestEqual(TEXT("Prototype D runtime projection preserves event count"),
			EventCountBeforeProjection,
			EventSidecar.GetCrustEventLog().Num());

		int32 ProjectedCrustSampleCount = 0;
		for (int32 SampleIndex = 0; SampleIndex < Projected.Samples.Num(); ++SampleIndex)
		{
			const FSample& BaselineSample = BaselineProjected.Samples[SampleIndex];
			const FSample& ProjectedSample = Projected.Samples[SampleIndex];
			TestEqual(TEXT("Prototype D projection never changes PlateId"),
				ProjectedSample.PlateId,
				BaselineSample.PlateId);
			TestEqual(TEXT("Prototype D projection never changes bIsBoundary"),
				ProjectedSample.bIsBoundary,
				BaselineSample.bIsBoundary);
			if (BaselineSample.ContinentalWeight >= static_cast<float>(EventConfig.OverlayContinentalWeightThreshold))
			{
				TestEqual(TEXT("Prototype D projection does not overwrite high-CW continental material"),
					ProjectedSample.ContinentalWeight,
					BaselineSample.ContinentalWeight);
				TestEqual(TEXT("Prototype D projection does not overwrite high-CW elevation"),
					ProjectedSample.Elevation,
					BaselineSample.Elevation);
				TestEqual(TEXT("Prototype D projection does not overwrite high-CW thickness"),
					ProjectedSample.Thickness,
					BaselineSample.Thickness);
				TestEqual(TEXT("Prototype D projection does not overwrite high-CW age"),
					ProjectedSample.Age,
					BaselineSample.Age);
			}

			if (EventSidecar.GetLastOceanCrustIds().IsValidIndex(SampleIndex) &&
				EventSidecar.GetLastOceanCrustIds()[SampleIndex] != INDEX_NONE)
			{
				++ProjectedCrustSampleCount;
				const int32 CrustId = EventSidecar.GetLastOceanCrustIds()[SampleIndex];
				const FSidecarOceanCrustRecord* Record =
					FindCrustRecordByIdForTest(EventSidecar.GetOceanCrustStore(), CrustId);
				TestTrue(TEXT("Prototype D projected crust id resolves to persistent record"), Record != nullptr);
				if (Record != nullptr)
				{
					const double ExpectedAgeMy =
						static_cast<double>(EventSidecar.GetCurrentStep() - Record->BirthStep) *
						EventConfig.DeltaTimeMy;
					TestTrue(TEXT("Prototype D projected age matches independent step math"),
						FMath::Abs(static_cast<double>(ProjectedSample.Age) - ExpectedAgeMy) <= 1.0e-6);
					TestTrue(TEXT("Prototype D age overlay matches independent step math"),
						FMath::Abs(static_cast<double>(EventSidecar.GetLastOceanCrustAgesMy()[SampleIndex]) - ExpectedAgeMy) <= 1.0e-6);
					TestTrue(TEXT("Prototype D projected thickness uses Slice 2 placeholder bounds"),
						ProjectedSample.Thickness >= 6.9f && ProjectedSample.Thickness <= 7.1f);
					TestTrue(TEXT("Prototype D thickness overlay uses Slice 2 placeholder bounds"),
						EventSidecar.GetLastOceanCrustThicknessKm()[SampleIndex] >= 6.9f &&
						EventSidecar.GetLastOceanCrustThicknessKm()[SampleIndex] <= 7.1f);
					TestTrue(TEXT("Prototype D projected elevation uses Slice 2 placeholder bounds"),
						ProjectedSample.Elevation >= -1.1f && ProjectedSample.Elevation <= -0.9f);
				}
			}
		}
		TestTrue(TEXT("Prototype D projection marks nonblank persistent ocean crust samples"),
			ProjectedCrustSampleCount > 0);

		const FString DOverlayDirectory = FPaths::Combine(
			BuildSidecarExportRoot(TEXT("SidecarPrototypeD")),
			TEXT("slice3_projection"),
			FString::Printf(TEXT("step_%03d"), EventSidecar.GetCurrentStep()));
		TestTrue(TEXT("Prototype D mandatory overlays export"),
			ExportSidecarDOverlays(*this, EventSidecar, Projected, DOverlayDirectory));

		FTectonicPlanetSidecar DeterminismA;
		FTectonicPlanetSidecar DeterminismB;
		DeterminismA.Initialize(EventConfig);
		DeterminismB.Initialize(EventConfig);
		ConfigureControlledPair(DeterminismA, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, true);
		ConfigureControlledPair(DeterminismB, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, true);
		DeterminismA.AdvanceSteps(25);
		DeterminismB.AdvanceSteps(25);
		TestEqual(TEXT("Prototype D runtime deterministic crust store hash"),
			DeterminismA.GetOceanCrustStore().ComputeCanonicalHash(),
			DeterminismB.GetOceanCrustStore().ComputeCanonicalHash());
		TestEqual(TEXT("Prototype D runtime deterministic event log hash"),
			DeterminismA.GetCrustEventLog().ComputeAppendOrderHash(),
			DeterminismB.GetCrustEventLog().ComputeAppendOrderHash());
	}

	{
		FTectonicSidecarConfig BelowThresholdConfig = Config;
		BelowThresholdConfig.bEnableDivergentSpreadingEvents = true;
		BelowThresholdConfig.DivergentSpreadingMinKmPerMy = 1000000.0;
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(BelowThresholdConfig);
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d CenterA = FVector3d::ZeroVector;
		FVector3d CenterB = FVector3d::ZeroVector;
		FVector3d BoundaryPoint = FVector3d::ZeroVector;
		TestTrue(TEXT("Prototype D below-threshold found boundary pair"),
			FindStrongBoundaryPlatePair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint));
		ConfigureControlledPair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, true);
		Sidecar.AdvanceSteps(1);
		TestEqual(TEXT("Prototype D below-threshold divergent motion creates zero crust"), Sidecar.GetOceanCrustStore().Num(), 0);
		TestEqual(TEXT("Prototype D below-threshold divergent motion appends zero events"), Sidecar.GetCrustEventLog().Num(), 0);
	}

	{
		FTectonicSidecarConfig EventConfig = MakeSidecarConfig(
			ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial,
			60000,
			2);
		EventConfig.bEnableDivergentSpreadingEvents = true;
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(EventConfig);
		int32 PlateA = INDEX_NONE;
		int32 PlateB = INDEX_NONE;
		FVector3d CenterA = FVector3d::ZeroVector;
		FVector3d CenterB = FVector3d::ZeroVector;
		FVector3d BoundaryPoint = FVector3d::ZeroVector;
		TestTrue(TEXT("Prototype D convergent negative-control found boundary pair"),
			FindStrongBoundaryPlatePair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint));
		ConfigureControlledPair(Sidecar, PlateA, PlateB, CenterA, CenterB, BoundaryPoint, false);
		Sidecar.AdvanceSteps(1);
		TestEqual(TEXT("Prototype D convergent motion creates zero crust"), Sidecar.GetOceanCrustStore().Num(), 0);
		TestEqual(TEXT("Prototype D convergent motion appends zero events"), Sidecar.GetCrustEventLog().Num(), 0);
	}

	{
		FTectonicSidecarConfig EventConfig = Config;
		EventConfig.bEnableDivergentSpreadingEvents = true;
		FTectonicPlanetSidecar Sidecar;
		Sidecar.Initialize(EventConfig);
		ZeroAllPlateKinematics(Sidecar);
		Sidecar.AdvanceSteps(1);
		TestEqual(TEXT("Prototype D zero-relative motion creates zero crust"), Sidecar.GetOceanCrustStore().Num(), 0);
		TestEqual(TEXT("Prototype D zero-relative motion appends zero events"), Sidecar.GetCrustEventLog().Num(), 0);
	}

	return true;
}
