#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TectonicPlanet.h"
#include "TectonicPlanetSidecar.h"
#include "TectonicPlanetVisualization.h"
#include "TectonicPlanetSidecarActor.generated.h"

class UMaterialInterface;
class URealtimeMeshComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

UCLASS(BlueprintType, Blueprintable, meta = (DisplayName = "Tectonic Planet Sidecar C"))
class AUROUS_API ATectonicPlanetSidecarActor : public AActor
{
	GENERATED_BODY()

public:
	ATectonicPlanetSidecarActor();

	const FTectonicPlanet& GetProjectedPlanet() const { return ProjectedPlanet; }
	const FTectonicPlanetSidecar& GetSidecar() const { return Sidecar; }
	const FTectonicSidecarProjectionDiagnostics& GetLastDiagnostics() const { return LastDiagnostics; }

	int32 GetConfiguredSampleCount() const { return SampleCount; }
	int32 GetConfiguredPlateCount() const { return PlateCount; }
	int32 GetConfiguredRandomSeed() const { return RandomSeed; }
	float GetConfiguredBoundaryWarpAmplitude() const { return BoundaryWarpAmplitude; }
	float GetConfiguredContinentalFraction() const { return ContinentalFraction; }
	int32 GetConfiguredExportWidth() const { return ExportWidth; }
	ETectonicMapExportMode GetVisualizationMode() const { return VisualizationMode; }
	ETectonicElevationPresentationMode GetElevationPresentationMode() const { return ElevationPresentationMode; }
	double GetLastAdvanceStepMs() const { return LastAdvanceStepMs; }
	double GetContinentalAreaFraction() const { return ContinentalAreaFraction; }
	bool GetShowPlateVelocities() const { return bShowPlateVelocities; }
	bool GetShowPlateBoundaries() const { return bShowPlateBoundaries; }
	FString GetActiveRuntimePresetLabel() const { return TEXT("SidecarPrototypeC"); }
	FString GetRuntimeConfigSummary() const;

	void GeneratePlanet(int32 InSampleCount, int32 InPlateCount, int32 InRandomSeed, float InBoundaryWarpAmplitude, float InContinentalFraction);
	void AdvancePlanetSteps(int32 InStepCount);
	void BuildMeshWithMode(ETectonicMapExportMode Mode);
	void SetShowPlateVelocities(bool bShow);
	void SetShowPlateBoundaries(bool bShow);
	bool ExportCurrentMaps(ETectonicMapExportMode Mode, int32 Width, int32 Height, FString& OutDirectory, FString& OutError) const;

	UFUNCTION(CallInEditor, Category = "Sidecar C|Controls")
	void Generate();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Controls")
	void Advance1Step();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Controls")
	void Advance16Steps();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Controls")
	void Advance100Steps();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Visualization")
	void ShowElevation();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Visualization")
	void ShowPlateId();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Visualization")
	void ShowContinentalWeight();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Visualization")
	void ShowBoundaryMask();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Visualization")
	void ShowMaterialClassification();

	UFUNCTION(CallInEditor, Category = "Sidecar C|Visualization")
	void ShowMaterialOverlap();

protected:
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Planet")
	int32 SampleCount = 60000;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Planet")
	int32 PlateCount = 40;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Planet")
	int32 RandomSeed = 42;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Planet")
	double PlanetRadiusKm = 6371.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Planet", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BoundaryWarpAmplitude = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Planet", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ContinentalFraction = 0.30f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Runtime", meta = (ClampMin = "0.0"))
	double DeltaTimeMy = 2.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Runtime", meta = (ClampMin = "0.0"))
	double MinPlateSpeedKmPerMy = 5.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Runtime", meta = (ClampMin = "0.0"))
	double MaxPlateSpeedKmPerMy = 20.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Runtime")
	bool bForceZeroAngularSpeeds = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Projection", meta = (ClampMin = "-1.0"))
	double RecoveryToleranceRad = -1.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Projection", meta = (ClampMin = "0.0"))
	double DiagnosticOverlapContainmentScore = 0.02;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Projection", meta = (ClampMin = "0.0"))
	double DivergenceMinKmPerMy = 0.5;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Projection", meta = (ClampMin = "0.0"))
	double DivergenceSpeedFraction = 0.05;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Export", meta = (ClampMin = "256"))
	int32 ExportWidth = 4096;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Rendering", meta = (ClampMin = "0.0001"))
	double VisualScale = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Rendering")
	bool bDisplaceByElevation = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Rendering")
	ETectonicElevationPresentationMode ElevationPresentationMode = ETectonicElevationPresentationMode::Raw;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Rendering", meta = (ClampMin = "0.0", EditCondition = "bDisplaceByElevation"))
	double ElevationDisplacementScale = 5.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Rendering")
	TObjectPtr<UMaterialInterface> PlanetMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Debug")
	bool bShowPlateVelocities = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sidecar C|Debug")
	bool bShowPlateBoundaries = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sidecar C|Status")
	int32 CurrentStep = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sidecar C|Status")
	int32 CurrentPlateCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sidecar C|Status")
	double ContinentalAreaFraction = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sidecar C|Status")
	int32 ContinentalSampleCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sidecar C|Status")
	double LastAdvanceStepMs = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sidecar C|Status")
	FString SimulationPath = TEXT("Prototype C: Voronoi Ownership + Decoupled Material");

private:
	FTectonicSidecarConfig BuildSidecarConfig() const;
	void AdvanceSteps(int32 StepCount);
	void ProjectSidecar();
	void BuildMesh();
	void UpdateStatusReadout();
	void RefreshDebugOverlays();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sidecar C", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<URealtimeMeshComponent> MeshComponent = nullptr;

#if WITH_EDITORONLY_DATA
	UPROPERTY(Transient)
	TObjectPtr<USceneCaptureComponent2D> EditorExportCaptureComponent = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> EditorExportRenderTarget = nullptr;
#endif

	FTectonicPlanetSidecar Sidecar;
	FTectonicPlanet ProjectedPlanet;
	FTectonicSidecarProjectionDiagnostics LastDiagnostics;
	ETectonicMapExportMode VisualizationMode = ETectonicMapExportMode::Elevation;
	bool bInitialized = false;
};
