#pragma once

#include "CoreMinimal.h"
#include "TectonicPlanet.h"
#include "Styling/SlateColor.h"

#if WITH_EDITOR
#include "Widgets/SCompoundWidget.h"

class ATectonicPlanetActor;
class STextComboBox;

class SAurousTectonicControlPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAurousTectonicControlPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	ATectonicPlanetActor* GetSelectedActor() const;
	ATectonicPlanetActor* FindActorInEditorSelection() const;
	void SetSelectedActor(ATectonicPlanetActor* InActor);
	void SyncPendingSettingsFromActor();
	void ApplyPendingSettingsToActor(bool bApplyDebugModeOnly = false) const;

	FReply OnUseSelectedActorClicked();
	FReply OnClearSelectedActorClicked();
	FReply OnGenerateClicked();
	FReply OnStartClicked();
	FReply OnStopClicked();
	FReply OnStep1Clicked();
	FReply OnStep10Clicked();
	FReply OnForceReconcileClicked();
	FReply OnUpdateColorsClicked();
	FReply OnExportMapsClicked();
	EActiveTimerReturnType HandleMetricsActiveTimer(double CurrentTime, float DeltaTime);

	FText GetSelectedActorText() const;
	FText GetMetricsText() const;
	FText GetExportHeightText() const;
	FText GetSimulationStatusText() const;
	FSlateColor GetSimulationStatusColor() const;

	int32 DebugModeToIndex(uint8 InDebugModeValue) const;
	uint8 IndexToDebugModeValue(int32 InIndex) const;
	void InvalidateExportCache();
	void UpdateMetricsActiveTimerRegistration();
	bool BuildExportPixelCache(
		int32 Width,
		int32 Height,
		const TArray<FCanonicalSample>& Samples,
		const TArray<FDelaunayTriangle>& Triangles);
	bool ExportMapsToDirectory(
		const FString& OutputDirectory,
		int32 Width,
		int32 Height,
		const TArray<FCanonicalSample>& Samples,
		const TArray<FDelaunayTriangle>& Triangles) const;

	struct FExportPixelSample
	{
		int32 TriangleIndex = INDEX_NONE;
		FVector Barycentric = FVector::ZeroVector;
		uint8 NearestVertexCorner = 0;
		bool bValid = false;
	};

private:
	TWeakObjectPtr<ATectonicPlanetActor> SelectedActor;

	int32 PendingNumSamples = 500000;
	int32 PendingNumPlates = 7;
	int32 PendingRandomSeed = 42;
	float PendingSimulationRate = 10.0f;
	int32 PendingDebugModeIndex = 0;
	int32 PendingExportWidth = 2048;

	TArray<FExportPixelSample> CachedExportPixelSamples;
	int32 CachedExportWidth = 0;
	int32 CachedExportHeight = 0;
	int32 CachedExportSampleCount = 0;
	int32 CachedExportTriangleCount = 0;
	int32 CachedValidPixelCount = 0;
	double LastStepCommandTime = -1.0;
	TWeakPtr<FActiveTimerHandle> MetricsActiveTimerHandle;
	TSharedPtr<STextBlock> MetricsTextBlock;

	TArray<TSharedPtr<FString>> DebugModeOptions;
	TSharedPtr<STextComboBox> DebugModeComboBox;
};

#endif // WITH_EDITOR
