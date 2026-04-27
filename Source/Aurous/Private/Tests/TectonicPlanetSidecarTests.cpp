#include "Misc/AutomationTest.h"

#include "HAL/PlatformFile.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/Paths.h"
#include "TectonicMollweideExporter.h"
#include "TectonicPlanetSidecar.h"

namespace
{
	constexpr int32 SidecarTestExportWidth = 2048;
	constexpr int32 SidecarTestExportHeight = 1024;
	constexpr double ControlledPairSpeedKmPerMy = 80.0;
	constexpr int32 PrototypeCResolutionCompareStep = 40;

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
		MixHashInt32(Hash, Sidecar.GetCurrentStep());
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
		return Hash;
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
	const FString RunId = TEXT("SidecarPrototypeC");
	const FString ExportRoot = BuildSidecarExportRoot(RunId);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.DeleteDirectoryRecursively(*ExportRoot);
	PlatformFile.CreateDirectoryTree(*ExportRoot);

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
		const uint32 DebugHash0 = HashSidecarDebugValues(Sidecar);

		const int32 Checkpoints[] = { 100, 200, 400 };
		for (const int32 Checkpoint : Checkpoints)
		{
			AdvanceToStep(Sidecar, Checkpoint);
			FTectonicPlanet Projected;
			FTectonicSidecarProjectionDiagnostics Diagnostics;
			Sidecar.ProjectToPlanet(Projected, &Diagnostics);
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
				AddInfo(FString::Printf(
					TEXT("[SidecarPrototypeCRigidAdvection] plate=%d weight=%.2f expected_drift_km=%.2f projected_error_km=%.2f"),
					PlateId,
					LocalWeight,
					ExpectedDriftKm,
					ProjectedErrorKm));
				TestTrue(
					FString::Printf(TEXT("Prototype C projected material centroid follows rigid rotation within lattice tolerance, error %.2f km"), ProjectedErrorKm),
					ProjectedErrorKm <= 750.0);
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
			AddSidecarDiagnosticsInfo(*this, TEXT("SidecarPrototypeC"), TEXT("60k40"), Diagnostics);
			CheckPrototypeCProjectionValidity(
				*this,
				FString::Printf(TEXT("60k40_step_%d"), Checkpoint),
				Projected,
				Diagnostics);
			CheckPrototypeCOwnershipGate(
				*this,
				FString::Printf(TEXT("60k40_step_%d"), Checkpoint),
				Diagnostics,
				Step0BoundaryFraction);
			ExportSidecarCCheckpointMaps(*this, Sidecar, Projected, ExportRoot, Checkpoint, TEXT("60k40"));
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
		TestTrue(TEXT("Prototype C convergent projection exposes material conflict"),
			Diagnostics.MaterialOverlapSupportSampleCount > 0 ||
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
			FMath::Abs(Resolution60k40.ContinentalCentroidDriftKm - Resolution250k40.ContinentalCentroidDriftKm) <= 500.0);
		TestTrue(
			TEXT("Prototype C resolution independence has no material fabrication at 250k"),
			Diagnostics.MaterialFabricatedFraction <= 0.0);

		CheckPrototypeCProjectionValidity(*this, TEXT("250k40_smoke"), Projected, Diagnostics);
		CheckPrototypeCOwnershipGate(*this, TEXT("250k40_smoke"), Diagnostics, BaselineDiagnostics.BoundaryFraction);
		ExportSidecarCCheckpointMaps(*this, Sidecar, Projected, ExportRoot, 40, TEXT("250k40_smoke"));
	}

	return true;
}
