#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "TectonicPlanet.h"
#include "TectonicPlanetActor.generated.h"

struct FPropertyChangedEvent;
class UMaterialInterface;
template<typename ResultType> class TFuture;

struct FPlanetControlPanelMetricsSnapshot
{
	int32 SampleCount = 0;
	int32 TriangleCount = 0;
	int32 PlateCount = 0;
	TArray<int32> PlateSampleCounts;
	int32 BoundarySampleCount = 0;
	double BoundaryMeanDepthHops = 0.0;
	int32 BoundaryMaxDepthHops = 0;
	int32 BoundaryDeepSampleCount = 0;
	int32 ContinentalSampleCount = 0;
	int32 ContinentalPlateCount = 0;
	double ContinentalAreaFraction = 0.0;
	int32 ContinentalComponentCount = 0;
	int32 LargestContinentalComponentSize = 0;
	int32 MaxPlateComponentCount = 0;
	int32 DetachedPlateFragmentSampleCount = 0;
	int32 LargestDetachedPlateFragmentSize = 0;
	int32 SubductionFrontSampleCount = 0;
	int32 AndeanSampleCount = 0;
	int32 TrackedTerraneCount = 0;
	int32 ActiveTerraneCount = 0;
	int32 MergedTerraneCount = 0;
	int32 CollisionEventCount = 0;
	int32 HimalayanSampleCount = 0;
	int32 PendingCollisionSampleCount = 0;
	float MaxSubductionDistanceKm = 0.0f;
	int32 MinProtectedPlateSampleCount = 0;
	int32 EmptyProtectedPlateCount = 0;
	int32 RescuedProtectedPlateCount = 0;
	int32 RescuedProtectedSampleCount = 0;
	int32 RepeatedlyRescuedProtectedSampleCount = 0;
	int64 Timestep = 0;
	int32 ReconcileCount = 0;
	bool bReconcileTriggeredLastStep = false;
	FReconcilePhaseTimings Timings;
	bool bHasTimings = false;
};

UENUM(BlueprintType)
enum class EPlanetDebugMode : uint8
{
	PlateId		UMETA(DisplayName = "Plate ID"),
	CrustType	UMETA(DisplayName = "Crust Type"),
	Boundary	UMETA(DisplayName = "Boundary"),
	BoundaryType UMETA(DisplayName = "Boundary Type"),
	Elevation	UMETA(DisplayName = "Elevation"),
	CrustAge	UMETA(DisplayName = "Crust Age"),
	SubductionRole UMETA(DisplayName = "Subduction Role"),
	SubductionDistance UMETA(DisplayName = "Subduction Distance"),
	OrogenyType UMETA(DisplayName = "Orogeny Type"),
	TerraneId UMETA(DisplayName = "Terrane ID"),
};

UCLASS(BlueprintType, Blueprintable)
class AUROUS_API ATectonicPlanetActor : public AActor
{
	GENERATED_BODY()

public:
	ATectonicPlanetActor();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet")
	int32 NumSamples = 500000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet")
	int32 NumPlates = 7;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet")
	int32 RandomSeed = 42;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet")
	float PlanetRenderRadius = 6370.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Initialization", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TargetContinentalAreaFraction = 0.30f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Initialization", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinContinentalAreaFraction = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Initialization", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MaxContinentalAreaFraction = 0.40f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Initialization", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinContinentalPlateFraction = 0.15f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Rendering")
	TObjectPtr<UMaterialInterface> PlanetMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Debug")
	EPlanetDebugMode DebugMode = EPlanetDebugMode::PlateId;

	UFUNCTION(BlueprintCallable, Category = "Planet")
	void GeneratePlanet();

	UFUNCTION(BlueprintCallable, Category = "Planet|Debug")
	void UpdateDebugColors();

	UFUNCTION(BlueprintCallable, Category = "Planet|Simulation")
	void StartSimulation();

	UFUNCTION(BlueprintCallable, Category = "Planet|Simulation")
	void StopSimulation();

	UFUNCTION(BlueprintCallable, Category = "Planet|Simulation")
	void StepSimulationSteps(int32 NumSteps = 1);

	UFUNCTION(BlueprintCallable, Category = "Planet|Simulation")
	void ForceReconcile();

	UFUNCTION(BlueprintPure, Category = "Planet|Simulation")
	int64 GetBackgroundTimestepCounterThreadSafe() const { return BackgroundTimestepCounter.Load(); }

	UFUNCTION(BlueprintPure, Category = "Planet|Simulation")
	bool IsBackgroundReconcileTriggeredThreadSafe() const { return BackgroundReconcileTriggered.Load(); }

	UFUNCTION(BlueprintPure, Category = "Planet|Simulation")
	bool IsSimulationThreadRunningThreadSafe() const { return bSimulationThreadRunning.Load(); }

	bool GetLastReconcileTimingsThreadSafe(FReconcilePhaseTimings& OutTimings, int32& OutReconcileCount) const;
	bool GetPlanetExportDataThreadSafe(TArray<FCanonicalSample>& OutSamples, TArray<FDelaunayTriangle>& OutTriangles, int64& OutTimestep) const;
	bool GetControlPanelMetricsSnapshotThreadSafe(FPlanetControlPanelMetricsSnapshot& OutSnapshot) const;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Simulation")
	bool bAutoStartSimulation = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Simulation", meta = (ClampMin = "0.1", ClampMax = "120.0"))
	float SimulationTimestepsPerSecond = 10.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet|Simulation")
	bool bSimulationRunning = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet|Simulation")
	int64 CurrentTimestepCounter = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet|Simulation")
	bool bReconciliationTriggeredThisFrame = false;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;
	virtual bool ShouldTickIfViewportsOnly() const override;
	virtual void OnConstruction(const FTransform& Transform) override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet")
	TObjectPtr<URealtimeMeshComponent> MeshComponent;

private:
	struct FPublishedPlanetState
	{
		TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> Samples;
		TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> Triangles;
		FPlanetControlPanelMetricsSnapshot Metrics;
	};

	void BuildMesh();
	FColor GetDebugColor(const FCanonicalSample& Sample) const;
	void SimulationThreadLoop();
	FPlanetControlPanelMetricsSnapshot BuildMetricsSnapshotFromPlanet_NoLock() const;
	void PublishPlanetState(
		TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe> InSamples,
		TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe> InTriangles,
		const FPlanetControlPanelMetricsSnapshot& InMetrics);
	void GetPublishedPlanetState(
		TSharedPtr<TArray<FCanonicalSample>, ESPMode::ThreadSafe>& OutSamples,
		TSharedPtr<TArray<FDelaunayTriangle>, ESPMode::ThreadSafe>& OutTriangles,
		FPlanetControlPanelMetricsSnapshot& OutMetrics) const;

	FTectonicPlanet Planet;
	FRealtimeMeshSectionGroupKey MeshGroupKey;
	bool bWarnedMissingPlanetMaterial = false;
	int32 CachedNumSamples = -1;
	int32 CachedNumPlates = -1;
	int32 CachedRandomSeed = -1;
	float CachedPlanetRenderRadius = TNumericLimits<float>::Lowest();
	float CachedTargetContinentalAreaFraction = TNumericLimits<float>::Lowest();
	float CachedMinContinentalAreaFraction = TNumericLimits<float>::Lowest();
	float CachedMaxContinentalAreaFraction = TNumericLimits<float>::Lowest();
	float CachedMinContinentalPlateFraction = TNumericLimits<float>::Lowest();
	TWeakObjectPtr<UMaterialInterface> CachedPlanetMaterial;
	TUniquePtr<TFuture<void>> SimulationFuture;
	mutable FCriticalSection PlanetMutex;
	mutable FCriticalSection PublishedStateMutex;
	FPublishedPlanetState PublishedState;
	TAtomic<bool> bSimulationThreadShouldRun { false };
	TAtomic<bool> bSimulationThreadRunning { false };
	TAtomic<bool> bHasPendingColorUpdate { false };
	TAtomic<int64> BackgroundTimestepCounter { 0 };
	TAtomic<bool> BackgroundReconcileTriggered { false };
};
