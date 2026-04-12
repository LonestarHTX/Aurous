#pragma once

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "TectonicPlanetVisualization.h"
#include "Widgets/SCompoundWidget.h"

class AActor;
class ATectonicPlanetActor;
class ATectonicPlanetV6PreviewActor;
class STextComboBox;

class SAurousTectonicControlPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAurousTectonicControlPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	AActor* GetSelectedActor() const;
	ATectonicPlanetActor* GetSelectedLegacyActor() const;
	ATectonicPlanetV6PreviewActor* GetSelectedV6Actor() const;
	AActor* FindActorInEditorSelection() const;
	void SetSelectedActor(AActor* InActor);
	void SyncPendingSettingsFromActor();
	void RefreshMetrics();
	ETectonicMapExportMode GetSelectedVisualizationMode() const;
	void SelectVisualizationMode(ETectonicMapExportMode Mode);
	bool EnsureSelectedActor(const TCHAR* ActionName) const;

	FReply OnUseSelectedClicked();
	FReply OnClearClicked();
	FReply OnGenerateClicked();
	FReply OnStep1Clicked();
	FReply OnStep10Clicked();
	FReply OnStep50Clicked();
	FReply OnExportMapsClicked();
	FReply OnRefreshMetricsClicked();
	void OnVisualizationModeChanged(TSharedPtr<FString> NewSelection, ESelectInfo::Type SelectInfo);

	FText GetSelectedActorText() const;
	FText GetCurrentStepText() const;
	FText GetTimingText() const;
	FText GetMetricsText() const;

private:
	TWeakObjectPtr<AActor> SelectedActor;
	int32 PendingSampleCount = 60000;
	int32 PendingPlateCount = 40;
	int32 PendingRandomSeed = 42;
	float PendingBoundaryWarpAmplitude = 0.2f;
	float PendingContinentalFraction = 0.30f;
	int32 PendingExportWidth = 4096;
	int32 CachedCurrentStep = 0;
	double CachedLastAdvanceStepMs = 0.0;
	FText CachedMetricsText = FText::FromString(TEXT("No actor selected."));
	TArray<TSharedPtr<FString>> VisualizationOptions;
	TSharedPtr<STextComboBox> VisualizationComboBox;
};

#endif
