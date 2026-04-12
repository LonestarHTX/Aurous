#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TectonicPlanetV6.h"
#include "TectonicPlanetVisualization.h"
#include "TectonicPlanetV6PreviewActor.generated.h"

class UMaterialInterface;
class URealtimeMeshComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

UCLASS(BlueprintType, Blueprintable, meta = (DisplayName = "Tectonic Planet V6 Preview"))
class AUROUS_API ATectonicPlanetV6PreviewActor : public AActor
{
	GENERATED_BODY()

public:
	ATectonicPlanetV6PreviewActor();

	const FTectonicPlanetV6& GetPlanetV6() const { return PlanetV6; }
	int32 GetConfiguredSampleCount() const { return SampleCount; }
	int32 GetConfiguredPlateCount() const { return PlateCount; }
	int32 GetConfiguredRandomSeed() const { return RandomSeed; }
	float GetConfiguredBoundaryWarpAmplitude() const { return BoundaryWarpAmplitude; }
	float GetConfiguredContinentalFraction() const { return ContinentalFraction; }
	int32 GetConfiguredExportWidth() const { return ExportWidth; }
	ETectonicMapExportMode GetVisualizationMode() const { return VisualizationMode; }
	double GetLastAdvanceStepMs() const { return LastAdvanceStepMs; }
	double GetContinentalAreaFraction() const { return ContinentalAreaFraction; }
	int32 GetPeriodicSolveCountValue() const { return PeriodicSolveCount; }
	int32 GetCumulativeCollisionCountValue() const { return CumulativeCollisionCount; }
	int32 GetCumulativeRiftCountValue() const { return CumulativeRiftCount; }
	bool GetShowPlateVelocities() const { return bShowPlateVelocities; }
	bool GetShowPlateBoundaries() const { return bShowPlateBoundaries; }
	bool GetShowBoundaryTypes() const { return false; }
	FString GetActiveRuntimePresetLabel() const { return TEXT("V6KeptCandidate"); }
	FString GetRuntimeConfigSummary() const;

	void GeneratePlanet(int32 InSampleCount, int32 InPlateCount, int32 InRandomSeed, float InBoundaryWarpAmplitude, float InContinentalFraction);
	void AdvancePlanetSteps(int32 InStepCount);
	void BuildMeshWithMode(ETectonicMapExportMode Mode);
	void SetShowPlateVelocities(bool bShow);
	void SetShowPlateBoundaries(bool bShow);
	void SetShowBoundaryTypes(bool bShow);
	bool ExportCurrentMaps(ETectonicMapExportMode Mode, int32 Width, int32 Height, FString& OutDirectory, FString& OutError) const;

	// Editor-callable controls (via details panel CallInEditor buttons)
	UFUNCTION(CallInEditor, Category = "V6 Preview|Controls")
	void Generate();

	UFUNCTION(CallInEditor, Category = "V6 Preview|Controls")
	void Advance1Step();

	UFUNCTION(CallInEditor, Category = "V6 Preview|Controls")
	void Advance16Steps();

	UFUNCTION(CallInEditor, Category = "V6 Preview|Controls")
	void Advance100Steps();

	UFUNCTION(CallInEditor, Category = "V6 Preview|Controls")
	void AdvanceToNextSolve();

	// Visualization mode switching
	UFUNCTION(CallInEditor, Category = "V6 Preview|Visualization")
	void ShowElevation();

	UFUNCTION(CallInEditor, Category = "V6 Preview|Visualization")
	void ShowPlateId();

	UFUNCTION(CallInEditor, Category = "V6 Preview|Visualization")
	void ShowContinentalWeight();

	UFUNCTION(CallInEditor, Category = "V6 Preview|Visualization")
	void ShowBoundaryMask();

protected:
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;

	// Planet parameters
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Planet")
	int32 SampleCount = 60000;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Planet")
	int32 PlateCount = 40;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Planet")
	int32 RandomSeed = 42;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Planet")
	double PlanetRadiusKm = 6371.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Planet", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BoundaryWarpAmplitude = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Planet", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ContinentalFraction = 0.30f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Planet")
	bool bEnableStochasticRifting = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Export", meta = (ClampMin = "256"))
	int32 ExportWidth = 4096;

	// Rendering
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Rendering", meta = (ClampMin = "0.0001"))
	double VisualScale = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Rendering")
	bool bDisplaceByElevation = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Rendering", meta = (ClampMin = "0.0", EditCondition = "bDisplaceByElevation"))
	double ElevationDisplacementScale = 5.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Rendering")
	TObjectPtr<UMaterialInterface> PlanetMaterial = nullptr;

	// Debug overlays
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Debug")
	bool bShowPlateVelocities = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "V6 Preview|Debug")
	bool bShowPlateBoundaries = false;

	// Status readout (read-only, updated after each operation)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	int32 CurrentStep = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	int32 CurrentPlateCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	double ContinentalAreaFraction = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	int32 ContinentalSampleCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	double LastAdvanceStepMs = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	int32 PeriodicSolveCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	int32 CumulativeCollisionCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	int32 CumulativeRiftCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview|Status")
	FString SimulationPath = TEXT("V6/v9 Kept Candidate");

private:
	void AdvanceSteps(int32 StepCount);
	void BuildMesh();
	void UpdateStatusReadout();
	void RefreshDebugOverlays();
	void ConfigureKeptCandidateDefaults();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "V6 Preview", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<URealtimeMeshComponent> MeshComponent = nullptr;

#if WITH_EDITORONLY_DATA
	UPROPERTY(Transient)
	TObjectPtr<USceneCaptureComponent2D> EditorExportCaptureComponent = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> EditorExportRenderTarget = nullptr;
#endif

	FTectonicPlanetV6 PlanetV6;
	ETectonicMapExportMode VisualizationMode = ETectonicMapExportMode::Elevation;
	bool bInitialized = false;
};
