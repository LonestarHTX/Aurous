#include "TectonicInitCommandlet.h"

#include "CoreGlobals.h"
#include "HAL/PlatformTime.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "ShewchukPredicates.h"
#include "TectonicMollweideExporter.h"
#include "TectonicPlanet.h"
#include "TectonicReferenceScenarios.h"
#include "TectonicWinkelTripelExporter.h"

namespace
{
	struct FValidationSummary
	{
		int32 ErrorCount = 0;
		int32 WarningCount = 0;
	};

	void LogError(FValidationSummary& Summary, const TCHAR* Message)
	{
		++Summary.ErrorCount;
		UE_LOG(LogTemp, Error, TEXT("%s"), Message);
	}

	void LogWarning(FValidationSummary& Summary, const TCHAR* Message)
	{
		++Summary.WarningCount;
		UE_LOG(LogTemp, Warning, TEXT("%s"), Message);
	}

	bool TryParseContinentalStabilizerMode(const FString& Value, EContinentalStabilizerMode& OutMode)
	{
		if (Value.Equals(TEXT("Disabled"), ESearchCase::IgnoreCase))
		{
			OutMode = EContinentalStabilizerMode::Disabled;
			return true;
		}

		if (Value.Equals(TEXT("Legacy"), ESearchCase::IgnoreCase) ||
			Value.Equals(TEXT("Incremental"), ESearchCase::IgnoreCase) ||
			Value.Equals(TEXT("Shadow"), ESearchCase::IgnoreCase))
		{
			OutMode = EContinentalStabilizerMode::Incremental;
			return true;
		}

		return false;
	}

	const TCHAR* ContinentalStabilizerModeToString(const EContinentalStabilizerMode Mode)
	{
		switch (Mode)
		{
		case EContinentalStabilizerMode::Disabled:
			return TEXT("Disabled");
		case EContinentalStabilizerMode::Incremental:
		case EContinentalStabilizerMode::Legacy:
		case EContinentalStabilizerMode::Shadow:
		default:
			return TEXT("Incremental");
		}
	}

	bool TryParseBenchmarkMode(const FString& Value, ETectonicBenchmarkMode& OutMode)
	{
		if (Value.Equals(TEXT("PaperCore"), ESearchCase::IgnoreCase))
		{
			OutMode = ETectonicBenchmarkMode::PaperCore;
			return true;
		}

		if (Value.Equals(TEXT("ScenarioCore"), ESearchCase::IgnoreCase))
		{
			OutMode = ETectonicBenchmarkMode::ScenarioCore;
			return true;
		}

		if (Value.Equals(TEXT("ScenarioFull"), ESearchCase::IgnoreCase) ||
			Value.Equals(TEXT("Full"), ESearchCase::IgnoreCase))
		{
			OutMode = ETectonicBenchmarkMode::Full;
			return true;
		}

		return false;
	}

	const TCHAR* BenchmarkModeToString(const ETectonicBenchmarkMode Mode)
	{
		switch (Mode)
		{
		case ETectonicBenchmarkMode::PaperCore:
			return TEXT("PaperCore");
		case ETectonicBenchmarkMode::ScenarioCore:
			return TEXT("ScenarioCore");
		case ETectonicBenchmarkMode::Full:
		default:
			return TEXT("ScenarioFull");
		}
	}

	void ParseIntList(const FString& Value, TArray<int32>& OutValues)
	{
		OutValues.Reset();
		TArray<FString> Parts;
		Value.ParseIntoArray(Parts, TEXT(","), true);
		for (const FString& Part : Parts)
		{
			OutValues.Add(FCString::Atoi(*Part));
		}
	}

	void ParseVectorList(const FString& Value, TArray<FVector>& OutValues)
	{
		OutValues.Reset();
		TArray<FString> Parts;
		Value.ParseIntoArray(Parts, TEXT(";"), true);
		for (const FString& Part : Parts)
		{
			TArray<FString> Components;
			Part.ParseIntoArray(Components, TEXT(","), true);
			if (Components.Num() != 3)
			{
				continue;
			}

			OutValues.Add(FVector(
				FCString::Atof(*Components[0]),
				FCString::Atof(*Components[1]),
				FCString::Atof(*Components[2])));
		}
	}

	FString MakeSafeOutputSuffix(const FString& Value)
	{
		FString SafeValue = Value;
		for (TCHAR& Character : SafeValue)
		{
			if (!FChar::IsAlnum(Character))
			{
				Character = TEXT('_');
			}
		}
		return SafeValue;
	}

	FString AppendSuffixToOutputSpecifier(
		const FString& OutputSpecifier,
		const FString& Suffix,
		const FString& DefaultRelativePath = FPaths::Combine(TEXT("Saved"), TEXT("Mollweide"), TEXT("TectonicMollweide.png")))
	{
		const FString SafeSuffix = MakeSafeOutputSuffix(Suffix);
		if (SafeSuffix.IsEmpty())
		{
			return OutputSpecifier;
		}

		if (OutputSpecifier.IsEmpty())
		{
			const FString Extension = FPaths::GetExtension(DefaultRelativePath, false);
			if (Extension.IsEmpty())
			{
				return DefaultRelativePath + TEXT("_") + SafeSuffix;
			}

			const FString Directory = FPaths::GetPath(DefaultRelativePath);
			const FString Stem = FPaths::GetBaseFilename(DefaultRelativePath);
			return FPaths::Combine(Directory, FString::Printf(TEXT("%s_%s.%s"), *Stem, *SafeSuffix, *Extension));
		}

		const FString Extension = FPaths::GetExtension(OutputSpecifier, false);
		if (Extension.IsEmpty())
		{
			return OutputSpecifier + TEXT("_") + SafeSuffix;
		}

		const FString Directory = FPaths::GetPath(OutputSpecifier);
		const FString Stem = FPaths::GetBaseFilename(OutputSpecifier);
		return FPaths::Combine(Directory, FString::Printf(TEXT("%s_%s.%s"), *Stem, *SafeSuffix, *Extension));
	}

	void LogMollweideExportSummary(const FTectonicMollweideExportStats& ExportStats)
	{
		UE_LOG(
			LogTemp,
			Display,
			TEXT("mollweide_export mode=%s resolution=%dx%d inside_ellipse=%lld containment_failures=%lld probe_fallbacks=%lld clear_fallbacks=%lld image_fill_fallbacks=%lld min_elevation=%.3f max_elevation=%.3f export=%.1fms plate_rule=ContainmentPlateId crust_rule=DominantBarycentricVertex files=%s"),
			TectonicMollweideExporter::GetModeName(ExportStats.Mode),
			ExportStats.Width,
			ExportStats.Height,
			static_cast<long long>(ExportStats.InsideEllipsePixelCount),
			static_cast<long long>(ExportStats.ContainmentFailureCount),
			static_cast<long long>(ExportStats.NeighborProbeFallbackCount),
			static_cast<long long>(ExportStats.ClearFallbackCount),
			static_cast<long long>(ExportStats.ImageFillFallbackCount),
			ExportStats.MinElevation,
			ExportStats.MaxElevation,
			ExportStats.ExportTimeMs,
			*FString::Join(ExportStats.WrittenFiles, TEXT(";")));
	}

	void LogWinkelTripelExportSummary(const FTectonicWinkelTripelExportStats& ExportStats)
	{
		UE_LOG(
			LogTemp,
			Display,
			TEXT("winkel_tripel_export mode=%s resolution=%dx%d inside_projection=%lld containment_failures=%lld probe_fallbacks=%lld clear_fallbacks=%lld image_fill_fallbacks=%lld min_elevation=%.3f max_elevation=%.3f export=%.1fms plate_rule=ContainmentPlateId crust_rule=DominantBarycentricVertex files=%s"),
			TectonicWinkelTripelExporter::GetModeName(ExportStats.Mode),
			ExportStats.Width,
			ExportStats.Height,
			static_cast<long long>(ExportStats.InsideProjectionPixelCount),
			static_cast<long long>(ExportStats.ContainmentFailureCount),
			static_cast<long long>(ExportStats.NeighborProbeFallbackCount),
			static_cast<long long>(ExportStats.ClearFallbackCount),
			static_cast<long long>(ExportStats.ImageFillFallbackCount),
			ExportStats.MinElevation,
			ExportStats.MaxElevation,
			ExportStats.ExportTimeMs,
			*FString::Join(ExportStats.WrittenFiles, TEXT(";")));
	}
}

#define AUR_LOG_ERRORF(SummaryRef, FormatLiteral, ...) \
	do \
	{ \
		++(SummaryRef).ErrorCount; \
		UE_LOG(LogTemp, Error, FormatLiteral, ##__VA_ARGS__); \
	} while (false)

#define AUR_LOG_WARNINGF(SummaryRef, FormatLiteral, ...) \
	do \
	{ \
		++(SummaryRef).WarningCount; \
		UE_LOG(LogTemp, Warning, FormatLiteral, ##__VA_ARGS__); \
	} while (false)

UTectonicInitCommandlet::UTectonicInitCommandlet()
{
	IsClient = false;
	IsServer = false;
	IsEditor = true;
	LogToConsole = true;
	ShowErrorCount = true;
}

int32 UTectonicInitCommandlet::Main(const FString& Params)
{
	int32 NumSamples = 500000;
	FParse::Value(*Params, TEXT("Samples="), NumSamples);
	FParse::Value(*Params, TEXT("Count="), NumSamples);

	const bool bStrict = FParse::Param(*Params, TEXT("Strict"));
	const bool bDisableP6 = FParse::Param(*Params, TEXT("DisableP6"));
	const bool bDisableP6Connectivity = FParse::Param(*Params, TEXT("DisableP6Connectivity"));
	const bool bDisableP3ContenderSupport = FParse::Param(*Params, TEXT("DisableP3ContenderSupport"));
	const bool bValidateScopedP6FullScan = FParse::Param(*Params, TEXT("ValidateScopedP6FullScan"));
	const bool bLogP6Mutations = FParse::Param(*Params, TEXT("LogP6Mutations"));
	const bool bLogP6Fragments = FParse::Param(*Params, TEXT("LogP6Fragments"));
	const bool bLogP6FragmentMembers = FParse::Param(*Params, TEXT("LogP6FragmentMembers"));
	const double StartSeconds = FPlatformTime::Seconds();
	const double StartupToMainMs = FMath::Max(0.0, (StartSeconds - GStartTime) * 1000.0);
	FString TraceSampleIndicesValue;
	FString TraceDirectionsValue;
	TArray<int32> TraceSampleIndices;
	TArray<FVector> TraceDirections;
	if (FParse::Value(*Params, TEXT("TraceSampleIndices="), TraceSampleIndicesValue))
	{
		ParseIntList(TraceSampleIndicesValue, TraceSampleIndices);
	}
	if (FParse::Value(*Params, TEXT("TraceDirections="), TraceDirectionsValue))
	{
		ParseVectorList(TraceDirectionsValue, TraceDirections);
	}

	FString ScenarioName;
	FParse::Value(*Params, TEXT("Scenario="), ScenarioName);

	EContinentalStabilizerMode StabilizerMode = EContinentalStabilizerMode::Disabled;
	FString StabilizerModeValue;
	if (FParse::Value(*Params, TEXT("StabilizerMode="), StabilizerModeValue) && !TryParseContinentalStabilizerMode(StabilizerModeValue, StabilizerMode))
	{
		UE_LOG(LogTemp, Error, TEXT("Unknown stabilizer mode '%s'. Expected Disabled or Incremental (Legacy and Shadow remain accepted aliases)."), *StabilizerModeValue);
		return 1;
	}

	ETectonicBenchmarkMode BenchmarkMode = ETectonicBenchmarkMode::Full;
	FString BenchmarkModeValue;
	const bool bBenchmarkModeRequested = FParse::Value(*Params, TEXT("BenchmarkMode="), BenchmarkModeValue);
	if (bBenchmarkModeRequested && !TryParseBenchmarkMode(BenchmarkModeValue, BenchmarkMode))
	{
		UE_LOG(LogTemp, Error, TEXT("Unknown benchmark mode '%s'. Expected PaperCore, ScenarioCore, or ScenarioFull."), *BenchmarkModeValue);
		return 1;
	}

	const bool bExportMollweide = FParse::Param(*Params, TEXT("ExportMollweide"));
	FTectonicMollweideExportOptions MollweideExportOptions;
	FString MollweideModeValue;
	if (FParse::Value(*Params, TEXT("MollweideMode="), MollweideModeValue) &&
		!TectonicMollweideExporter::TryParseMode(MollweideModeValue, MollweideExportOptions.Mode))
	{
		UE_LOG(LogTemp, Error, TEXT("Unknown Mollweide mode '%s'. Expected Elevation, PlateId, CrustType, or All."), *MollweideModeValue);
		return 1;
	}
	FParse::Value(*Params, TEXT("MollweideWidth="), MollweideExportOptions.Width);
	FParse::Value(*Params, TEXT("MollweideHeight="), MollweideExportOptions.Height);
	FParse::Value(*Params, TEXT("MollweideOut="), MollweideExportOptions.OutputPath);
	if (bExportMollweide && (MollweideExportOptions.Width <= 0 || MollweideExportOptions.Height <= 0))
	{
		UE_LOG(LogTemp, Error, TEXT("Mollweide export resolution must be positive. width=%d height=%d"), MollweideExportOptions.Width, MollweideExportOptions.Height);
		return 1;
	}

	const bool bExportWinkelTripel = FParse::Param(*Params, TEXT("ExportWinkelTripel"));
	FTectonicWinkelTripelExportOptions WinkelTripelExportOptions;
	FString WinkelTripelModeValue;
	if (FParse::Value(*Params, TEXT("WinkelTripelMode="), WinkelTripelModeValue) &&
		!TectonicWinkelTripelExporter::TryParseMode(WinkelTripelModeValue, WinkelTripelExportOptions.Mode))
	{
		UE_LOG(LogTemp, Error, TEXT("Unknown Winkel Tripel mode '%s'. Expected Elevation, PlateId, CrustType, or All."), *WinkelTripelModeValue);
		return 1;
	}
	FParse::Value(*Params, TEXT("WinkelTripelWidth="), WinkelTripelExportOptions.Width);
	FParse::Value(*Params, TEXT("WinkelTripelHeight="), WinkelTripelExportOptions.Height);
	FParse::Value(*Params, TEXT("WinkelTripelOut="), WinkelTripelExportOptions.OutputPath);
	if (bExportWinkelTripel && (WinkelTripelExportOptions.Width <= 0 || WinkelTripelExportOptions.Height <= 0))
	{
		UE_LOG(LogTemp, Error, TEXT("Winkel Tripel export resolution must be positive. width=%d height=%d"), WinkelTripelExportOptions.Width, WinkelTripelExportOptions.Height);
		return 1;
	}

	const bool bOwnershipAblation = FParse::Param(*Params, TEXT("OwnershipAblation"));
	if (bOwnershipAblation)
	{
		struct FAblationWorkload
		{
			const TCHAR* Name;
			int32 Samples;
			int32 Plates;
			int32 Seed;
			int32 Steps;
			ETectonicBenchmarkMode Mode;
		};
		const FAblationWorkload Workloads[] = {
			{ TEXT("PaperCore"), 500000, 7, 42, 10, ETectonicBenchmarkMode::PaperCore },
			{ TEXT("Smoke7"), 500000, 7, 1, 10, ETectonicBenchmarkMode::Full },
			{ TEXT("Stress40"), 500000, 40, 4, 5, ETectonicBenchmarkMode::Full }
		};

		FString OnlyWorkload;
		FParse::Value(*Params, TEXT("Scenario="), OnlyWorkload);

		struct FAblationModeConfig
		{
			const TCHAR* Label;
			bool bPaperSimple;
			bool bContenderSupport;
			bool bP6Enabled;
		};
		const FAblationModeConfig Modes[] = {
			{ TEXT("A_PaperSimple"), true, false, false },
			{ TEXT("B_PaperSimple+Guard"), true, true, false },
			{ TEXT("C_Baseline"), false, true, true }
		};

		struct FAblationResult
		{
			const TCHAR* Workload = TEXT("");
			const TCHAR* Mode = TEXT("");
			int32 TotalFragments = 0;
			int32 MaxFragmentSize = 0;
			int32 Persistent3Plus = 0;
			int32 ShadowSanitize = 0;
			int32 ShadowConnectivity = 0;
			int32 ShadowRescueEligible = 0;
			int32 TotalGapSamples = 0;
			int32 TotalOverlapSamples = 0;
			double AvgReconcileMs = 0.0;
			double MaxReconcileMs = 0.0;
			bool bScenarioPass = true;
		};
		TArray<FAblationResult> Results;

		// Pre-initialize base planet for 500k samples (shared across all runs).
		FTectonicPlanet BasePlanet;
		BasePlanet.Initialize(500000);

		for (const FAblationWorkload& Workload : Workloads)
		{
			if (!OnlyWorkload.IsEmpty() && !OnlyWorkload.Equals(Workload.Name, ESearchCase::IgnoreCase))
			{
				continue;
			}

			for (const FAblationModeConfig& ModeConfig : Modes)
			{
				UE_LOG(LogTemp, Display, TEXT("=== Ablation: %s / %s ==="), Workload.Name, ModeConfig.Label);

				FTectonicPlanet Planet = BasePlanet;
				Planet.InitializePlates(Workload.Plates, Workload.Seed);
				Planet.SetBenchmarkMode(Workload.Mode);
				Planet.SetPaperSimpleOwnership(ModeConfig.bPaperSimple);
				Planet.SetRequireP3ContenderSupport(ModeConfig.bContenderSupport);
				Planet.SetEnablePhase6OwnershipRepair(ModeConfig.bP6Enabled);
				Planet.SetEnableConnectivityEnforcement(true);
				Planet.SetTrackP6DisabledFragments(!ModeConfig.bP6Enabled);

				FAblationResult Result;
				Result.Workload = Workload.Name;
				Result.Mode = ModeConfig.Label;

				double TotalReconcileMs = 0.0;
				int32 ReconcilesExecuted = 0;

				for (int32 StepIndex = 0; StepIndex < Workload.Steps; ++StepIndex)
				{
					Planet.StepSimulation();
					if (Planet.WasReconcileTriggeredLastStep())
					{
						const FReconcilePhaseTimings& Timings = Planet.GetLastReconcileTimings();
						TotalReconcileMs += Timings.TotalMs;
						Result.MaxReconcileMs = FMath::Max(Result.MaxReconcileMs, Timings.TotalMs);
						Result.TotalGapSamples += Timings.GapSamples;
						Result.TotalOverlapSamples += Timings.OverlapSamples;
						// Track max fragment metrics across reconciles.
						Result.TotalFragments = FMath::Max(Result.TotalFragments, Planet.GetMaxPlateComponentCount());
						Result.MaxFragmentSize = FMath::Max(Result.MaxFragmentSize, Planet.GetLargestDetachedPlateFragmentSize());
						if (ModeConfig.bP6Enabled)
						{
							Result.ShadowSanitize += Timings.Phase6SanitizeMutationCount;
							Result.ShadowConnectivity += Timings.Phase6ConnectivityMutationCount;
							Result.ShadowRescueEligible += Timings.Phase6ProtectedRescueMutationCount;
						}
						++ReconcilesExecuted;
					}
				}

				if (ReconcilesExecuted > 0)
				{
					Result.AvgReconcileMs = TotalReconcileMs / static_cast<double>(ReconcilesExecuted);
				}

				Results.Add(Result);

				UE_LOG(LogTemp, Display,
					TEXT("ablation_result workload=%s mode=%s reconciles=%d avg_reconcile=%.1fms max_reconcile=%.1fms total_gaps=%d total_overlaps=%d max_plate_components=%d max_detached_fragment=%d"),
					Workload.Name, ModeConfig.Label,
					ReconcilesExecuted,
					Result.AvgReconcileMs, Result.MaxReconcileMs,
					Result.TotalGapSamples, Result.TotalOverlapSamples,
					Result.TotalFragments, Result.MaxFragmentSize);
			}
		}

		// Summary comparison table.
		UE_LOG(LogTemp, Display, TEXT(""));
		UE_LOG(LogTemp, Display, TEXT("=== Ownership Ablation Comparison ==="));
		UE_LOG(LogTemp, Display, TEXT("%-14s %-24s %8s %8s %8s %8s %10s %10s"),
			TEXT("Workload"), TEXT("Mode"), TEXT("Frags"), TEXT("MaxFrag"), TEXT("Gaps"), TEXT("Overlaps"), TEXT("AvgMs"), TEXT("MaxMs"));
		for (const FAblationResult& R : Results)
		{
			UE_LOG(LogTemp, Display, TEXT("%-14s %-24s %8d %8d %8d %8d %10.1f %10.1f"),
				R.Workload, R.Mode, R.TotalFragments, R.MaxFragmentSize,
				R.TotalGapSamples, R.TotalOverlapSamples,
				R.AvgReconcileMs, R.MaxReconcileMs);
		}

		return 0;
	}

	if (bBenchmarkModeRequested)
	{
		int32 BenchmarkSamples = NumSamples;
		int32 PlateCount = 7;
		int32 Seed = 42;
		int32 TargetSteps = 0;
		int32 TargetReconciles = 1;
		FParse::Value(*Params, TEXT("Plates="), PlateCount);
		FParse::Value(*Params, TEXT("Seed="), Seed);
		FParse::Value(*Params, TEXT("Steps="), TargetSteps);
		FParse::Value(*Params, TEXT("Reconciles="), TargetReconciles);

		if (!ScenarioName.IsEmpty())
		{
			const FReferenceScenarioDefinition* LockedScenario = FindLockedReferenceScenarioByName(ScenarioName);
			if (!LockedScenario)
			{
				UE_LOG(LogTemp, Error, TEXT("Unknown benchmark scenario '%s'."), *ScenarioName);
				return 1;
			}

			BenchmarkSamples = LockedScenario->SampleCount;
			PlateCount = LockedScenario->PlateCount;
			Seed = LockedScenario->Seed;
			if (TargetSteps <= 0)
			{
				TargetSteps = LockedScenario->NumSteps;
			}
		}

		FParse::Value(*Params, TEXT("Samples="), BenchmarkSamples);
		FParse::Value(*Params, TEXT("Count="), BenchmarkSamples);
		FParse::Value(*Params, TEXT("Plates="), PlateCount);
		FParse::Value(*Params, TEXT("Seed="), Seed);

		FTectonicPlanet Planet;
		Planet.Initialize(BenchmarkSamples);
		Planet.InitializePlates(PlateCount, Seed);
		Planet.SetContinentalStabilizerMode(StabilizerMode);
		Planet.SetBenchmarkMode(BenchmarkMode);
		Planet.SetEnableConnectivityEnforcement(!bDisableP6Connectivity);
		Planet.SetEnablePhase6OwnershipRepair(!bDisableP6);
		Planet.SetRequireP3ContenderSupport(!bDisableP3ContenderSupport);
		Planet.SetValidateScopedP6Scans(bValidateScopedP6FullScan);
		Planet.SetLogP6MutationDiagnostics(bLogP6Mutations);
		Planet.SetTrackP6DisabledFragments(bLogP6Fragments);
		Planet.SetLogP6FragmentMembers(bLogP6FragmentMembers);
		Planet.SetP3P5TraceSampleIndices(TraceSampleIndices);
		Planet.SetP3P5TraceReferenceDirections(TraceDirections);

		double TotalReconcileMs = 0.0;
		double MaxReconcileMs = 0.0;
		double TotalPhase2Ms = 0.0;
		double TotalPhase6Ms = 0.0;
		double TotalPhase6OwnershipMs = 0.0;
		double TotalPhase7Ms = 0.0;
		double TotalPhase8bMs = 0.0;
		double TotalPhase9Ms = 0.0;
		int32 StepsExecuted = 0;
		int32 ReconcilesExecuted = 0;
		const double BenchmarkStartSeconds = FPlatformTime::Seconds();

		auto RecordReconcileTiming = [&Planet, &TotalReconcileMs, &MaxReconcileMs, &TotalPhase2Ms, &TotalPhase6Ms, &TotalPhase6OwnershipMs, &TotalPhase7Ms, &TotalPhase8bMs, &TotalPhase9Ms, &ReconcilesExecuted]()
		{
			const FReconcilePhaseTimings& Timings = Planet.GetLastReconcileTimings();
			TotalReconcileMs += Timings.TotalMs;
			MaxReconcileMs = FMath::Max(MaxReconcileMs, Timings.TotalMs);
			TotalPhase2Ms += Timings.Phase2OwnershipMs;
			TotalPhase6Ms += Timings.Phase6MembershipMs;
			TotalPhase6OwnershipMs += Timings.Phase6SanitizeOwnershipMs;
			TotalPhase7Ms += Timings.Phase7TerraneMs;
			TotalPhase8bMs += Timings.Phase8PostCollisionRefreshMs;
			TotalPhase9Ms += Timings.Phase9SubductionMs;
			++ReconcilesExecuted;
		};

		if (TargetSteps > 0)
		{
			for (int32 StepIndex = 0; StepIndex < TargetSteps; ++StepIndex)
			{
				Planet.StepSimulation();
				++StepsExecuted;
				if (Planet.WasReconcileTriggeredLastStep())
				{
					RecordReconcileTiming();
				}
			}
		}
		else
		{
			while (ReconcilesExecuted < FMath::Max(1, TargetReconciles))
			{
				Planet.StepSimulation();
				++StepsExecuted;
				if (Planet.WasReconcileTriggeredLastStep())
				{
					RecordReconcileTiming();
				}
			}
		}

		const double BenchmarkWallMs = (FPlatformTime::Seconds() - BenchmarkStartSeconds) * 1000.0;
		if (ReconcilesExecuted <= 0)
		{
			UE_LOG(LogTemp, Error, TEXT("Benchmark completed without any reconcile. mode=%s samples=%d plates=%d seed=%d steps=%d"), BenchmarkModeToString(BenchmarkMode), BenchmarkSamples, PlateCount, Seed, StepsExecuted);
			return 1;
		}

		if (bExportMollweide)
		{
			FTectonicMollweideExportStats ExportStats;
			FString ExportError;
			if (!TectonicMollweideExporter::ExportPlanet(Planet, MollweideExportOptions, ExportStats, ExportError))
			{
				UE_LOG(LogTemp, Error, TEXT("Mollweide export failed: %s"), *ExportError);
				return 1;
			}

			LogMollweideExportSummary(ExportStats);
		}

		if (bExportWinkelTripel)
		{
			FTectonicWinkelTripelExportStats ExportStats;
			FString ExportError;
			if (!TectonicWinkelTripelExporter::ExportPlanet(Planet, WinkelTripelExportOptions, ExportStats, ExportError))
			{
				UE_LOG(LogTemp, Error, TEXT("Winkel Tripel export failed: %s"), *ExportError);
				return 1;
			}

			LogWinkelTripelExportSummary(ExportStats);
		}

		UE_LOG(
			LogTemp,
			Display,
			TEXT("benchmark_mode=%s scenario=%s samples=%d plates=%d seed=%d steps=%d reconciles=%d wall=%.1fms avg_reconcile=%.1fms max_reconcile=%.1fms avg_p2=%.1fms avg_p6=%.1fms avg_p6_ownership=%.1fms avg_p7=%.1fms avg_p8b=%.1fms avg_p9=%.1fms stabilizer_mode=%s p6=%s connectivity=%s p3_contender_support=%s p6_validation=%s p6_mutation_log=%s p6_fragment_log=%s p6_fragment_members=%s trace_samples=%d trace_directions=%d"),
			BenchmarkModeToString(BenchmarkMode),
			ScenarioName.IsEmpty() ? TEXT("Custom") : *ScenarioName,
			BenchmarkSamples,
			PlateCount,
			Seed,
			StepsExecuted,
			ReconcilesExecuted,
			BenchmarkWallMs,
			TotalReconcileMs / static_cast<double>(ReconcilesExecuted),
			MaxReconcileMs,
			TotalPhase2Ms / static_cast<double>(ReconcilesExecuted),
			TotalPhase6Ms / static_cast<double>(ReconcilesExecuted),
			TotalPhase6OwnershipMs / static_cast<double>(ReconcilesExecuted),
			TotalPhase7Ms / static_cast<double>(ReconcilesExecuted),
			TotalPhase8bMs / static_cast<double>(ReconcilesExecuted),
			TotalPhase9Ms / static_cast<double>(ReconcilesExecuted),
			ContinentalStabilizerModeToString(StabilizerMode),
			bDisableP6 ? TEXT("Disabled") : TEXT("Enabled"),
			bDisableP6Connectivity ? TEXT("Disabled") : TEXT("Enabled"),
			bDisableP3ContenderSupport ? TEXT("Disabled") : TEXT("Enabled"),
			bValidateScopedP6FullScan ? TEXT("Enabled") : TEXT("Disabled"),
			bLogP6Mutations ? TEXT("Enabled") : TEXT("Disabled"),
			bLogP6Fragments ? TEXT("Enabled") : TEXT("Disabled"),
			bLogP6FragmentMembers ? TEXT("Enabled") : TEXT("Disabled"),
			TraceSampleIndices.Num(),
			TraceDirections.Num());
		return 0;
	}

	const bool bSeedSweep = FParse::Param(*Params, TEXT("SeedSweep")) || FParse::Param(*Params, TEXT("SweepReferenceSeeds"));
	if ((bExportMollweide || bExportWinkelTripel) && bSeedSweep)
	{
		UE_LOG(LogTemp, Error, TEXT("Projection export does not support seed sweeps. Run a single benchmark or scenario export instead."));
		return 1;
	}
	const bool bRunReferenceScenario = !ScenarioName.IsEmpty() || bSeedSweep;
	if (bRunReferenceScenario)
	{
		TArray<FReferenceScenarioDefinition> ScenariosToRun;
		if (!ScenarioName.IsEmpty())
		{
			const FReferenceScenarioDefinition* LockedScenario = FindLockedReferenceScenarioByName(ScenarioName);
			if (!LockedScenario)
			{
				UE_LOG(LogTemp, Error, TEXT("Unknown reference scenario '%s'."), *ScenarioName);
				return 1;
			}

			ScenariosToRun.Add(*LockedScenario);
		}
		else
		{
			ScenariosToRun = GetLockedReferenceScenarios();
		}

		TMap<int32, FTectonicPlanet> BasePlanets;
		auto GetBasePlanet = [&BasePlanets](const int32 SampleCount) -> FTectonicPlanet&
		{
			FTectonicPlanet& BasePlanet = BasePlanets.FindOrAdd(SampleCount);
			if (BasePlanet.GetNumSamples() != SampleCount)
			{
				BasePlanet.Initialize(SampleCount);
			}
			return BasePlanet;
		};

		int32 ExitCode = 0;
		for (const FReferenceScenarioDefinition& LockedScenario : ScenariosToRun)
		{
			FReferenceScenarioDefinition Scenario = LockedScenario;
			int32 OverrideSamples = Scenario.SampleCount;
			if (FParse::Value(*Params, TEXT("Samples="), OverrideSamples) || FParse::Value(*Params, TEXT("Count="), OverrideSamples))
			{
				Scenario.SampleCount = OverrideSamples;
			}
			int32 OverrideSeed = Scenario.Seed;
			if (FParse::Value(*Params, TEXT("Seed="), OverrideSeed))
			{
				Scenario.Seed = OverrideSeed;
			}
			int32 OverrideSteps = Scenario.NumSteps;
			if (FParse::Value(*Params, TEXT("Steps="), OverrideSteps))
			{
				Scenario.NumSteps = OverrideSteps;
			}

			FTectonicPlanet& BasePlanet = GetBasePlanet(Scenario.SampleCount);
			BasePlanet.SetEnableConnectivityEnforcement(!bDisableP6Connectivity);
			BasePlanet.SetEnablePhase6OwnershipRepair(!bDisableP6);
			BasePlanet.SetRequireP3ContenderSupport(!bDisableP3ContenderSupport);
			BasePlanet.SetValidateScopedP6Scans(bValidateScopedP6FullScan);
			BasePlanet.SetLogP6MutationDiagnostics(bLogP6Mutations);
			BasePlanet.SetTrackP6DisabledFragments(bLogP6Fragments);
			BasePlanet.SetLogP6FragmentMembers(bLogP6FragmentMembers);
			BasePlanet.SetP3P5TraceSampleIndices(TraceSampleIndices);
			BasePlanet.SetP3P5TraceReferenceDirections(TraceDirections);
			if (bSeedSweep)
			{
				int32 LockedSeed = INDEX_NONE;
				FReferenceScenarioObservedMetrics Metrics;
				if (!TryDiscoverLowestPassingReferenceScenarioSeed(Scenario, BasePlanet, LockedSeed, Metrics))
				{
					UE_LOG(LogTemp, Error, TEXT("Reference scenario sweep failed for %s: no passing seed in [1, 256]."), Scenario.Name);
					return 1;
				}

				FReferenceScenarioDefinition SummaryScenario = Scenario;
				SummaryScenario.Seed = LockedSeed;
				const bool bPass = DoesReferenceScenarioObservedMetricsPass(SummaryScenario, Metrics);
				UE_LOG(
					LogTemp,
					Display,
					TEXT("%s stabilizer_mode=%s p6=%s connectivity=%s p3_contender_support=%s p6_validation=%s p6_mutation_log=%s p6_fragment_log=%s sweep=true"),
					*FormatReferenceScenarioSummary(SummaryScenario, Metrics, bPass),
					ContinentalStabilizerModeToString(EContinentalStabilizerMode::Disabled),
					bDisableP6 ? TEXT("Disabled") : TEXT("Enabled"),
					bDisableP6Connectivity ? TEXT("Disabled") : TEXT("Enabled"),
					bDisableP3ContenderSupport ? TEXT("Disabled") : TEXT("Enabled"),
					bValidateScopedP6FullScan ? TEXT("Enabled") : TEXT("Disabled"),
					bLogP6Mutations ? TEXT("Enabled") : TEXT("Disabled"),
					bLogP6Fragments ? TEXT("Enabled") : TEXT("Disabled"));
				continue;
			}

			FReferenceScenarioObservedMetrics Metrics;
			const bool bCollected = CollectReferenceScenarioObservedMetrics(Scenario, BasePlanet, Metrics, StartupToMainMs, StabilizerMode);
			const bool bPass = bCollected && DoesReferenceScenarioObservedMetricsPass(Scenario, Metrics);
			if (bExportMollweide && bCollected)
			{
				FTectonicMollweideExportOptions ScenarioExportOptions = MollweideExportOptions;
				if (ScenariosToRun.Num() > 1)
				{
					ScenarioExportOptions.OutputPath = AppendSuffixToOutputSpecifier(MollweideExportOptions.OutputPath, Scenario.Name);
				}

				FTectonicMollweideExportStats ExportStats;
				FString ExportError;
				if (!TectonicMollweideExporter::ExportPlanet(BasePlanet, ScenarioExportOptions, ExportStats, ExportError))
				{
					UE_LOG(LogTemp, Error, TEXT("Mollweide export failed for scenario %s: %s"), Scenario.Name, *ExportError);
					return 1;
				}

				LogMollweideExportSummary(ExportStats);
			}
			else if (bExportMollweide && !bCollected)
			{
				UE_LOG(LogTemp, Error, TEXT("Mollweide export skipped for scenario %s because scenario execution failed."), Scenario.Name);
				return 1;
			}

			if (bExportWinkelTripel && bCollected)
			{
				FTectonicWinkelTripelExportOptions ScenarioExportOptions = WinkelTripelExportOptions;
				if (ScenariosToRun.Num() > 1)
				{
					ScenarioExportOptions.OutputPath = AppendSuffixToOutputSpecifier(
						WinkelTripelExportOptions.OutputPath,
						Scenario.Name,
						FPaths::Combine(TEXT("Saved"), TEXT("WinkelTripel"), TEXT("TectonicWinkelTripel.png")));
				}

				FTectonicWinkelTripelExportStats ExportStats;
				FString ExportError;
				if (!TectonicWinkelTripelExporter::ExportPlanet(BasePlanet, ScenarioExportOptions, ExportStats, ExportError))
				{
					UE_LOG(LogTemp, Error, TEXT("Winkel Tripel export failed for scenario %s: %s"), Scenario.Name, *ExportError);
					return 1;
				}

				LogWinkelTripelExportSummary(ExportStats);
			}
			else if (bExportWinkelTripel && !bCollected)
			{
				UE_LOG(LogTemp, Error, TEXT("Winkel Tripel export skipped for scenario %s because scenario execution failed."), Scenario.Name);
				return 1;
			}

			UE_LOG(
				LogTemp,
				Display,
				TEXT("%s stabilizer_mode=%s p6=%s connectivity=%s p3_contender_support=%s p6_validation=%s p6_mutation_log=%s p6_fragment_log=%s"),
				*FormatReferenceScenarioSummary(Scenario, Metrics, bPass),
				ContinentalStabilizerModeToString(StabilizerMode),
				bDisableP6 ? TEXT("Disabled") : TEXT("Enabled"),
				bDisableP6Connectivity ? TEXT("Disabled") : TEXT("Enabled"),
				bDisableP3ContenderSupport ? TEXT("Disabled") : TEXT("Enabled"),
				bValidateScopedP6FullScan ? TEXT("Enabled") : TEXT("Disabled"),
				bLogP6Mutations ? TEXT("Enabled") : TEXT("Disabled"),
				bLogP6Fragments ? TEXT("Enabled") : TEXT("Disabled"));
			if (!bCollected || !bPass)
			{
				UE_LOG(
					LogTemp,
					Warning,
					TEXT("Reference scenario [%s] failure details: %s"),
					Scenario.Name,
					*DescribeReferenceScenarioFailureDetails(Metrics));
				ExitCode = 1;
			}
		}

		return ExitCode;
	}

	if (bExportMollweide || bExportWinkelTripel)
	{
		UE_LOG(LogTemp, Error, TEXT("Projection export requires a simulated planet state. Use -BenchmarkMode=PaperCore|ScenarioCore|ScenarioFull or -Scenario=..."));
		return 1;
	}

	UE_LOG(LogTemp, Display, TEXT("TectonicInitCommandlet: Initializing planet with %d samples"), NumSamples);

	FTectonicPlanet Planet;
	Planet.Initialize(NumSamples);

	const double EndSeconds = FPlatformTime::Seconds();
	const TArray<FCanonicalSample>& Samples = Planet.GetSamples();
	const TArray<FDelaunayTriangle>& Triangles = Planet.GetTriangles();
	const TArray<TArray<int32>>& Adjacency = Planet.GetAdjacency();

	FValidationSummary Summary;

	if (Samples.Num() != NumSamples)
	{
		AUR_LOG_ERRORF(Summary, TEXT("Sample count mismatch: requested=%d actual=%d"), NumSamples, Samples.Num());
	}

	double MinRadius = TNumericLimits<double>::Max();
	double MaxRadius = 0.0;
	double MaxRadiusError = 0.0;

	for (const FCanonicalSample& Sample : Samples)
	{
		const double Radius = Sample.Position.Size();
		MinRadius = FMath::Min(MinRadius, Radius);
		MaxRadius = FMath::Max(MaxRadius, Radius);
		MaxRadiusError = FMath::Max(MaxRadiusError, FMath::Abs(Radius - 1.0));
	}

	UE_LOG(LogTemp, Display, TEXT("Radius stats: min=%.15f max=%.15f max|r-1|=%.3e"), MinRadius, MaxRadius, MaxRadiusError);
	if (MaxRadiusError > 1e-9)
	{
		AUR_LOG_WARNINGF(Summary, TEXT("Max radius error %.3e exceeds tight tolerance 1e-9"), MaxRadiusError);
	}

	const int32 ExpectedTriangleCount = (Samples.Num() > 0) ? (2 * Samples.Num() - 4) : 0;
	const int32 TriangleCountDelta = Triangles.Num() - ExpectedTriangleCount;
	UE_LOG(LogTemp, Display, TEXT("Triangle stats: actual=%d expected~=%d delta=%d"), Triangles.Num(), ExpectedTriangleCount, TriangleCountDelta);
	if (Samples.Num() >= 4 && Triangles.Num() != ExpectedTriangleCount)
	{
		AUR_LOG_WARNINGF(Summary, TEXT("Triangle count differs from 2N-4 by %d"), TriangleCountDelta);
	}

	TArray<int32> TriangleIncidenceCounts;
	TriangleIncidenceCounts.Init(0, Samples.Num());

	int32 InvalidTriangleIndexCount = 0;
	int32 DegenerateTriangleCount = 0;
	double MinTriangleArea2 = TNumericLimits<double>::Max();
	double MaxTriangleArea2 = 0.0;

	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
		const int32 AIndex = Triangle.V[0];
		const int32 BIndex = Triangle.V[1];
		const int32 CIndex = Triangle.V[2];

		if (!Samples.IsValidIndex(AIndex) || !Samples.IsValidIndex(BIndex) || !Samples.IsValidIndex(CIndex))
		{
			++InvalidTriangleIndexCount;
			continue;
		}

		if (AIndex == BIndex || BIndex == CIndex || CIndex == AIndex)
		{
			++DegenerateTriangleCount;
			continue;
		}

		const FVector& A = Samples[AIndex].Position;
		const FVector& B = Samples[BIndex].Position;
		const FVector& C = Samples[CIndex].Position;
		const double Area2 = FVector::CrossProduct(B - A, C - A).Size();
		MinTriangleArea2 = FMath::Min(MinTriangleArea2, Area2);
		MaxTriangleArea2 = FMath::Max(MaxTriangleArea2, Area2);
		if (Area2 <= static_cast<double>(SMALL_NUMBER))
		{
			++DegenerateTriangleCount;
			continue;
		}

		++TriangleIncidenceCounts[AIndex];
		++TriangleIncidenceCounts[BIndex];
		++TriangleIncidenceCounts[CIndex];
	}

	int32 SamplesMissingFromTriangles = 0;
	for (const int32 IncidenceCount : TriangleIncidenceCounts)
	{
		SamplesMissingFromTriangles += (IncidenceCount == 0) ? 1 : 0;
	}

	UE_LOG(LogTemp, Display, TEXT("Triangle quality: invalidIndex=%d degenerate=%d min|cross|=%.6e max|cross|=%.6e missingVertices=%d"),
		InvalidTriangleIndexCount,
		DegenerateTriangleCount,
		(MinTriangleArea2 == TNumericLimits<double>::Max()) ? 0.0 : MinTriangleArea2,
		MaxTriangleArea2,
		SamplesMissingFromTriangles);

	if (InvalidTriangleIndexCount > 0)
	{
		AUR_LOG_ERRORF(Summary, TEXT("Found %d triangles with invalid vertex indices"), InvalidTriangleIndexCount);
	}
	if (DegenerateTriangleCount > 0)
	{
		AUR_LOG_ERRORF(Summary, TEXT("Found %d degenerate triangles"), DegenerateTriangleCount);
	}
	if (SamplesMissingFromTriangles > 0)
	{
		AUR_LOG_ERRORF(Summary, TEXT("Found %d samples not referenced by any triangle"), SamplesMissingFromTriangles);
	}

	if (Adjacency.Num() != Samples.Num())
	{
		AUR_LOG_ERRORF(Summary, TEXT("Adjacency size mismatch: samples=%d adjacency=%d"), Samples.Num(), Adjacency.Num());
	}
	else
	{
		int32 ZeroValenceVertices = 0;
		int32 InvalidNeighbors = 0;
		int32 AsymmetricEdges = 0;
		int32 MinValence = MAX_int32;
		int32 MaxValence = 0;
		int64 TotalValence = 0;

		for (int32 VertexIndex = 0; VertexIndex < Adjacency.Num(); ++VertexIndex)
		{
			const TArray<int32>& Neighbors = Adjacency[VertexIndex];
			const int32 Valence = Neighbors.Num();

			ZeroValenceVertices += (Valence == 0) ? 1 : 0;
			MinValence = FMath::Min(MinValence, Valence);
			MaxValence = FMath::Max(MaxValence, Valence);
			TotalValence += Valence;

			for (const int32 NeighborIndex : Neighbors)
			{
				if (!Adjacency.IsValidIndex(NeighborIndex))
				{
					++InvalidNeighbors;
					continue;
				}

				if (!Adjacency[NeighborIndex].Contains(VertexIndex))
				{
					++AsymmetricEdges;
				}
			}
		}

		const double AverageValence = (Adjacency.Num() > 0) ? static_cast<double>(TotalValence) / static_cast<double>(Adjacency.Num()) : 0.0;
		UE_LOG(LogTemp, Display, TEXT("Adjacency stats: vertices=%d min/max/avg valence=%d/%d/%.3f zeroValence=%d invalidNeighbors=%d asymmetricRefs=%d"),
			Adjacency.Num(),
			(MinValence == MAX_int32) ? 0 : MinValence,
			MaxValence,
			AverageValence,
			ZeroValenceVertices,
			InvalidNeighbors,
			AsymmetricEdges);

		if (ZeroValenceVertices > 0)
		{
			AUR_LOG_ERRORF(Summary, TEXT("Found %d zero-valence vertices"), ZeroValenceVertices);
		}
		if (InvalidNeighbors > 0)
		{
			AUR_LOG_ERRORF(Summary, TEXT("Found %d invalid adjacency references"), InvalidNeighbors);
		}
		if (AsymmetricEdges > 0)
		{
			AUR_LOG_ERRORF(Summary, TEXT("Found %d asymmetric adjacency references"), AsymmetricEdges);
		}
		if (MinValence < 5 || MaxValence > 7)
		{
			AUR_LOG_WARNINGF(Summary, TEXT("Valence outlier range observed (expected typical 5-7): min=%d max=%d"), MinValence, MaxValence);
		}
	}

	// Basic smoke test that Shewchuk predicates are callable after Initialize() invokes exactinit().
	const double Pa[3] = { 1.0, 0.0, 0.0 };
	const double Pb[3] = { 0.0, 1.0, 0.0 };
	const double Pc[3] = { 0.0, 0.0, 1.0 };
	const double Pd[3] = { 0.0, 0.0, 0.0 };
	const double Orientation = orient3d(Pa, Pb, Pc, Pd);
	UE_LOG(LogTemp, Display, TEXT("Shewchuk orient3d smoke test result: %.6e"), Orientation);
	if (Orientation == 0.0)
	{
		LogWarning(Summary, TEXT("orient3d smoke test returned zero (unexpected for tetrahedron test points)"));
	}

	UE_LOG(LogTemp, Display, TEXT("Initialization completed in %.3f s with %d error(s), %d warning(s)."),
		EndSeconds - StartSeconds,
		Summary.ErrorCount,
		Summary.WarningCount);

	if (Summary.ErrorCount > 0)
	{
		return 1;
	}

	if (bStrict && Summary.WarningCount > 0)
	{
		UE_LOG(LogTemp, Display, TEXT("Strict mode enabled: treating warnings as failure."));
		return 2;
	}

	return 0;
}

#undef AUR_LOG_WARNINGF
#undef AUR_LOG_ERRORF
