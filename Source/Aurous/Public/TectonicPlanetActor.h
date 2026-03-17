#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TectonicPlanet.h"
#include "TectonicPlanetVisualization.h"
#include "TectonicPlanetActor.generated.h"

class UMaterialInterface;
class URealtimeMeshComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

struct FTectonicPlanetMetrics
{
	int32 CurrentStep = 0;
	int32 SampleCount = 0;
	int32 TriangleCount = 0;
	int64 UniqueEdgeCount = 0;
	int32 MinNeighborCount = 0;
	int32 MaxNeighborCount = 0;
	double MeanNeighborCount = 0.0;
	int32 PlateCount = 0;
	int32 MinSamplesPerPlate = 0;
	int32 MaxSamplesPerPlate = 0;
	double MeanSamplesPerPlate = 0.0;
	double ContinentalFraction = 0.0;
	int32 ContinentalSeededPlateCount = 0;
	int32 OceanicOnlyPlateCount = 0;
	int32 TerraneCount = 0;
	int32 MinTerranesPerPlate = 0;
	int32 MaxTerranesPerPlate = 0;
	int32 InteriorTriangleCount = 0;
	int32 BoundaryTriangleCount = 0;
	int32 BoundarySampleCount = 0;
	double BoundarySampleFraction = 0.0;
	double MinElevation = 0.0;
	double MaxElevation = 0.0;
	double MeanElevation = 0.0;
};

UCLASS(BlueprintType, Blueprintable)
class AUROUS_API ATectonicPlanetActor : public AActor
{
	GENERATED_BODY()

public:
	ATectonicPlanetActor();

	FTectonicPlanet& GetPlanet() { return Planet; }
	const FTectonicPlanet& GetPlanet() const { return Planet; }
	FTectonicPlanetMetrics GetMetrics() const;
	void GeneratePlanet(int32 InSampleCount, int32 InPlateCount, int32 InRandomSeed, float InBoundaryWarpAmplitude, float InContinentalFraction);
	void AdvancePlanetSteps(int32 InStepCount);
	void BuildMesh();
	void BuildMeshWithMode(ETectonicMapExportMode Mode);
	void SetShowPlateVelocities(bool bShow);
	void SetShowPlateBoundaries(bool bShow);
	void SetShowBoundaryTypes(bool bShow);
	bool ExportCurrentMaps(ETectonicMapExportMode Mode, int32 Width, int32 Height, FString& OutDirectory, FString& OutError) const;
	void ExportInitialMaps(const FString& RunId, double SampleDerivedContinentalFraction) const;

	int32 GetConfiguredSampleCount() const { return SampleCount; }
	int32 GetConfiguredPlateCount() const { return PlateCount; }
	int32 GetConfiguredRandomSeed() const { return RandomSeed; }
	double GetConfiguredPlanetRadiusKm() const { return PlanetRadiusKm; }
	float GetConfiguredBoundaryWarpAmplitude() const { return BoundaryWarpAmplitude; }
	float GetConfiguredContinentalFraction() const { return ContinentalFraction; }
	int32 GetConfiguredExportWidth() const { return ExportWidth; }
	ETectonicMapExportMode GetVisualizationMode() const { return VisualizationMode; }
	double GetLastAdvanceStepMs() const { return LastAdvanceStepMs; }
	bool GetShowPlateVelocities() const { return bShowPlateVelocities; }
	bool GetShowPlateBoundaries() const { return bShowPlateBoundaries; }
	bool GetShowBoundaryTypes() const { return bShowBoundaryTypes; }

protected:
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet")
	int32 SampleCount = 60000;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet")
	int32 PlateCount = 7;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Initialization")
	int32 RandomSeed = 42;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet")
	double PlanetRadiusKm = 6371.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Initialization", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BoundaryWarpAmplitude = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Initialization", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ContinentalFraction = 0.30f;

private:
#if WITH_EDITOR
	bool ExportCurrentMeshScreenshots(const FString& OutputDirectory, FString& OutError) const;
	bool EnsureEditorExportCaptureResources(int32 CaptureResolution, FString& OutError);
	void ReleaseEditorExportCaptureResources();
#endif

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Rendering", meta = (AllowPrivateAccess = "true", ClampMin = "0.0001"))
	double VisualScale = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Rendering", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UMaterialInterface> PlanetMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Export", meta = (AllowPrivateAccess = "true"))
	bool bExportMapsOnInit = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Export", meta = (AllowPrivateAccess = "true"))
	ETectonicMapExportMode ExportMode = ETectonicMapExportMode::All;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Export", meta = (AllowPrivateAccess = "true", ClampMin = "256"))
	int32 ExportWidth = 4096;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Export", meta = (AllowPrivateAccess = "true", ClampMin = "128"))
	int32 ExportHeight = 2048;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Debug", meta = (AllowPrivateAccess = "true"))
	bool bShowPlateVelocities = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Debug", meta = (AllowPrivateAccess = "true"))
	bool bShowPlateBoundaries = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Planet|Debug", meta = (AllowPrivateAccess = "true"))
	bool bShowBoundaryTypes = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<URealtimeMeshComponent> MeshComponent = nullptr;

#if WITH_EDITORONLY_DATA
	UPROPERTY(Transient)
	TObjectPtr<USceneCaptureComponent2D> EditorExportCaptureComponent = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> EditorExportRenderTarget = nullptr;
#endif

	FTectonicPlanet Planet;
	ETectonicMapExportMode VisualizationMode = ETectonicMapExportMode::Elevation;
	double LastAdvanceStepMs = 0.0;

	void RefreshDebugOverlays();
	void DrawPlateVelocityArrows();
	void DrawPlateBoundaries();
	void DrawBoundaryTypes();
};
