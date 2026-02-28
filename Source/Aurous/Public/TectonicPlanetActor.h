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
	virtual void OnConstruction(const FTransform& Transform) override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet")
	TObjectPtr<URealtimeMeshComponent> MeshComponent;

private:
	void BuildMesh();
	// Pure color mapping from already-fetched sample data.
	FColor GetDebugColor(const FCanonicalSample& Sample) const;
	void SimulationThreadLoop();

	FTectonicPlanet Planet;
	FRealtimeMeshSectionGroupKey MeshGroupKey;
	bool bWarnedMissingPlanetMaterial = false;
	int32 CachedNumSamples = -1;
	int32 CachedNumPlates = -1;
	int32 CachedRandomSeed = -1;
	float CachedPlanetRenderRadius = TNumericLimits<float>::Lowest();
	TWeakObjectPtr<UMaterialInterface> CachedPlanetMaterial;
	TUniquePtr<TFuture<void>> SimulationFuture;
	mutable FCriticalSection PlanetMutex;
	TAtomic<bool> bSimulationThreadShouldRun { false };
	TAtomic<bool> bSimulationThreadRunning { false };
	TAtomic<bool> bHasPendingColorUpdate { false };
	TAtomic<int64> BackgroundTimestepCounter { 0 };
	TAtomic<bool> BackgroundReconcileTriggered { false };
};
