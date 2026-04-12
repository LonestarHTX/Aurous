#include "Editor/SAurousTectonicControlPanel.h"

#if WITH_EDITOR

#include "TectonicPlanetActor.h"
#include "TectonicPlanetV6PreviewActor.h"
#include "Editor.h"
#include "Engine/Selection.h"
#include "Misc/MessageDialog.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Input/STextComboBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SGridPanel.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "SAurousTectonicControlPanel"

namespace
{
	constexpr int32 MinimumExportHeight = 128;

	FText FormatMetricLine(const TCHAR* Label, const FString& Value)
	{
		return FText::FromString(FString::Printf(TEXT("%s: %s"), Label, *Value));
	}
}

void SAurousTectonicControlPanel::Construct(const FArguments& InArgs)
{
	VisualizationOptions = {
		MakeShared<FString>(TEXT("Elevation")),
		MakeShared<FString>(TEXT("PlateId")),
		MakeShared<FString>(TEXT("CrustType")),
		MakeShared<FString>(TEXT("ContinentalWeight"))
	};

	SetSelectedActor(FindActorInEditorSelection());

	ChildSlot
	[
		SNew(SBorder)
		.Padding(10.0f)
		[
			SNew(SScrollBox)

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SVerticalBox)

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 6.0f)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("ActorSelectionHeading", "Actor Selection"))
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 8.0f)
				[
					SNew(SUniformGridPanel)
					.SlotPadding(6.0f)

					+ SUniformGridPanel::Slot(0, 0)
					[
						SNew(SButton)
						.Text(LOCTEXT("UseSelectedButton", "Use Selected"))
						.OnClicked(this, &SAurousTectonicControlPanel::OnUseSelectedClicked)
					]

					+ SUniformGridPanel::Slot(1, 0)
					[
						SNew(SButton)
						.Text(LOCTEXT("ClearSelectedButton", "Clear"))
						.OnClicked(this, &SAurousTectonicControlPanel::OnClearClicked)
					]
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				[
					SNew(STextBlock)
					.Text(this, &SAurousTectonicControlPanel::GetSelectedActorText)
				]
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SSeparator)
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SVerticalBox)

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 6.0f)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("GenerationHeading", "Generation Settings"))
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				[
					SNew(SGridPanel)
					.FillColumn(1, 1.0f)

					+ SGridPanel::Slot(0, 0)
					.Padding(0.0f, 0.0f, 8.0f, 6.0f)
					[
						SNew(STextBlock).Text(LOCTEXT("SampleCountLabel", "Sample Count"))
					]
					+ SGridPanel::Slot(1, 0)
					.Padding(0.0f, 0.0f, 0.0f, 6.0f)
					[
						SNew(SSpinBox<int32>)
						.MinValue(4)
						.MaxValue(1000000)
						.Value_Lambda([this]() { return PendingSampleCount; })
						.OnValueChanged_Lambda([this](const int32 Value) { PendingSampleCount = Value; })
					]

					+ SGridPanel::Slot(0, 1)
					.Padding(0.0f, 0.0f, 8.0f, 6.0f)
					[
						SNew(STextBlock).Text(LOCTEXT("PlateCountLabel", "Plate Count"))
					]
					+ SGridPanel::Slot(1, 1)
					.Padding(0.0f, 0.0f, 0.0f, 6.0f)
					[
						SNew(SSpinBox<int32>)
						.MinValue(1)
						.MaxValue(100)
						.Value_Lambda([this]() { return PendingPlateCount; })
						.OnValueChanged_Lambda([this](const int32 Value) { PendingPlateCount = Value; })
					]

					+ SGridPanel::Slot(0, 2)
					.Padding(0.0f, 0.0f, 8.0f, 6.0f)
					[
						SNew(STextBlock).Text(LOCTEXT("RandomSeedLabel", "Random Seed"))
					]
					+ SGridPanel::Slot(1, 2)
					.Padding(0.0f, 0.0f, 0.0f, 6.0f)
					[
						SNew(SSpinBox<int32>)
						.MinValue(0)
						.MaxValue(TNumericLimits<int32>::Max())
						.Value_Lambda([this]() { return PendingRandomSeed; })
						.OnValueChanged_Lambda([this](const int32 Value) { PendingRandomSeed = Value; })
					]

					+ SGridPanel::Slot(0, 3)
					.Padding(0.0f, 0.0f, 8.0f, 6.0f)
					[
						SNew(STextBlock).Text(LOCTEXT("BoundaryWarpLabel", "Boundary Warp"))
					]
					+ SGridPanel::Slot(1, 3)
					.Padding(0.0f, 0.0f, 0.0f, 6.0f)
					[
						SNew(SSpinBox<float>)
						.MinValue(0.0f)
						.MaxValue(1.0f)
						.Value_Lambda([this]() { return PendingBoundaryWarpAmplitude; })
						.OnValueChanged_Lambda([this](const float Value) { PendingBoundaryWarpAmplitude = Value; })
					]

					+ SGridPanel::Slot(0, 4)
					.Padding(0.0f, 0.0f, 8.0f, 0.0f)
					[
						SNew(STextBlock).Text(LOCTEXT("ContinentalFractionLabel", "Continental Fraction"))
					]
					+ SGridPanel::Slot(1, 4)
					[
						SNew(SSpinBox<float>)
						.MinValue(0.0f)
						.MaxValue(1.0f)
						.Value_Lambda([this]() { return PendingContinentalFraction; })
						.OnValueChanged_Lambda([this](const float Value) { PendingContinentalFraction = Value; })
					]
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 8.0f, 0.0f, 0.0f)
				[
					SNew(SButton)
					.Text(LOCTEXT("GenerateButton", "Generate"))
					.OnClicked(this, &SAurousTectonicControlPanel::OnGenerateClicked)
				]
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SSeparator)
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SVerticalBox)

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 6.0f)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("SimulationHeading", "Simulation Controls"))
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				[
					SNew(STextBlock)
					.Text(this, &SAurousTectonicControlPanel::GetCurrentStepText)
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 4.0f, 0.0f, 8.0f)
				[
					SNew(STextBlock)
					.Text(this, &SAurousTectonicControlPanel::GetTimingText)
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				[
					SNew(SUniformGridPanel)
					.SlotPadding(6.0f)

					+ SUniformGridPanel::Slot(0, 0)
					[
						SNew(SButton)
						.Text(LOCTEXT("Step1Button", "Step 1"))
						.OnClicked(this, &SAurousTectonicControlPanel::OnStep1Clicked)
					]

					+ SUniformGridPanel::Slot(1, 0)
					[
						SNew(SButton)
						.Text(LOCTEXT("Step10Button", "Step 10"))
						.OnClicked(this, &SAurousTectonicControlPanel::OnStep10Clicked)
					]

					+ SUniformGridPanel::Slot(2, 0)
					[
						SNew(SButton)
						.Text(LOCTEXT("Step50Button", "Step 50"))
						.OnClicked(this, &SAurousTectonicControlPanel::OnStep50Clicked)
					]
				]
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SSeparator)
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SVerticalBox)

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 6.0f)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("VisualizationHeading", "Visualization"))
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				[
					SAssignNew(VisualizationComboBox, STextComboBox)
					.OptionsSource(&VisualizationOptions)
					.OnSelectionChanged(this, &SAurousTectonicControlPanel::OnVisualizationModeChanged)
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 8.0f, 0.0f, 0.0f)
				[
					SNew(SHorizontalBox)

					+ SHorizontalBox::Slot()
					.AutoWidth()
					.VAlign(VAlign_Center)
					.Padding(0.0f, 0.0f, 8.0f, 0.0f)
					[
						SNew(SCheckBox)
						.IsChecked_Lambda([this]()
						{
							if (const ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
							{
								return Actor->GetShowPlateVelocities()
									? ECheckBoxState::Checked
									: ECheckBoxState::Unchecked;
							}
							if (const ATectonicPlanetV6PreviewActor* Actor = GetSelectedV6Actor())
							{
								return Actor->GetShowPlateVelocities()
									? ECheckBoxState::Checked
									: ECheckBoxState::Unchecked;
							}

							return ECheckBoxState::Unchecked;
						})
						.OnCheckStateChanged_Lambda([this](const ECheckBoxState NewState)
						{
							if (ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
							{
								Actor->SetShowPlateVelocities(NewState == ECheckBoxState::Checked);
							}
							else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
							{
								V6Actor->SetShowPlateVelocities(NewState == ECheckBoxState::Checked);
							}
						})
					]

					+ SHorizontalBox::Slot()
					.AutoWidth()
					.VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("ShowPlateVelocities", "Show Plate Velocities"))
					]
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 4.0f, 0.0f, 0.0f)
				[
					SNew(SHorizontalBox)

					+ SHorizontalBox::Slot()
					.AutoWidth()
					.VAlign(VAlign_Center)
					.Padding(0.0f, 0.0f, 8.0f, 0.0f)
					[
						SNew(SCheckBox)
						.IsChecked_Lambda([this]()
						{
							if (const ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
							{
								return Actor->GetShowPlateBoundaries()
									? ECheckBoxState::Checked
									: ECheckBoxState::Unchecked;
							}
							if (const ATectonicPlanetV6PreviewActor* Actor = GetSelectedV6Actor())
							{
								return Actor->GetShowPlateBoundaries()
									? ECheckBoxState::Checked
									: ECheckBoxState::Unchecked;
							}

							return ECheckBoxState::Unchecked;
						})
						.OnCheckStateChanged_Lambda([this](const ECheckBoxState NewState)
						{
							if (ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
							{
								Actor->SetShowPlateBoundaries(NewState == ECheckBoxState::Checked);
							}
							else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
							{
								V6Actor->SetShowPlateBoundaries(NewState == ECheckBoxState::Checked);
							}
						})
					]

					+ SHorizontalBox::Slot()
					.AutoWidth()
					.VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("ShowPlateBoundaries", "Show Plate Boundaries"))
					]
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 4.0f, 0.0f, 0.0f)
				[
					SNew(SHorizontalBox)

					+ SHorizontalBox::Slot()
					.AutoWidth()
					.VAlign(VAlign_Center)
					.Padding(0.0f, 0.0f, 8.0f, 0.0f)
					[
						SNew(SCheckBox)
						.IsChecked_Lambda([this]()
						{
							if (const ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
							{
								return Actor->GetShowBoundaryTypes()
									? ECheckBoxState::Checked
									: ECheckBoxState::Unchecked;
							}
							if (const ATectonicPlanetV6PreviewActor* Actor = GetSelectedV6Actor())
							{
								return Actor->GetShowBoundaryTypes()
									? ECheckBoxState::Checked
									: ECheckBoxState::Unchecked;
							}

							return ECheckBoxState::Unchecked;
						})
						.OnCheckStateChanged_Lambda([this](const ECheckBoxState NewState)
						{
							if (ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
							{
								Actor->SetShowBoundaryTypes(NewState == ECheckBoxState::Checked);
							}
							else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
							{
								V6Actor->SetShowBoundaryTypes(NewState == ECheckBoxState::Checked);
							}
						})
					]

					+ SHorizontalBox::Slot()
					.AutoWidth()
					.VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("ShowBoundaryTypes", "Show Boundary Types (R=Div G=Xform C=Conv)"))
					]
				]
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SSeparator)
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SVerticalBox)

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 6.0f)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("ExportHeading", "Export"))
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				[
					SNew(SGridPanel)
					.FillColumn(1, 1.0f)

					+ SGridPanel::Slot(0, 0)
					.Padding(0.0f, 0.0f, 8.0f, 0.0f)
					[
						SNew(STextBlock).Text(LOCTEXT("ExportWidthLabel", "Export Width"))
					]

					+ SGridPanel::Slot(1, 0)
					[
						SNew(SSpinBox<int32>)
						.MinValue(256)
						.MaxValue(16384)
						.Value_Lambda([this]() { return PendingExportWidth; })
						.OnValueChanged_Lambda([this](const int32 Value) { PendingExportWidth = Value; })
					]
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 8.0f, 0.0f, 0.0f)
				[
					SNew(SButton)
					.Text(LOCTEXT("ExportMapsButton", "Export Maps"))
					.OnClicked(this, &SAurousTectonicControlPanel::OnExportMapsClicked)
				]
			]

			+ SScrollBox::Slot()
			.Padding(0.0f, 0.0f, 0.0f, 12.0f)
			[
				SNew(SSeparator)
			]

			+ SScrollBox::Slot()
			[
				SNew(SVerticalBox)

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 6.0f)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("MetricsHeading", "Metrics"))
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				[
					SNew(SButton)
					.Text(LOCTEXT("RefreshMetricsButton", "Refresh"))
					.OnClicked(this, &SAurousTectonicControlPanel::OnRefreshMetricsClicked)
				]

				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 8.0f, 0.0f, 0.0f)
				[
					SNew(STextBlock)
					.AutoWrapText(true)
					.Text(this, &SAurousTectonicControlPanel::GetMetricsText)
				]
			]
		]
	];

	SelectVisualizationMode(ETectonicMapExportMode::Elevation);
	SyncPendingSettingsFromActor();
	RefreshMetrics();
}

AActor* SAurousTectonicControlPanel::GetSelectedActor() const
{
	return SelectedActor.Get();
}

ATectonicPlanetActor* SAurousTectonicControlPanel::GetSelectedLegacyActor() const
{
	return Cast<ATectonicPlanetActor>(SelectedActor.Get());
}

ATectonicPlanetV6PreviewActor* SAurousTectonicControlPanel::GetSelectedV6Actor() const
{
	return Cast<ATectonicPlanetV6PreviewActor>(SelectedActor.Get());
}

AActor* SAurousTectonicControlPanel::FindActorInEditorSelection() const
{
	if (!GEditor)
	{
		return nullptr;
	}

	if (USelection* Selection = GEditor->GetSelectedActors())
	{
		for (FSelectionIterator It(*Selection); It; ++It)
		{
			if (ATectonicPlanetActor* Actor = Cast<ATectonicPlanetActor>(*It))
			{
				return Actor;
			}
			if (ATectonicPlanetV6PreviewActor* Actor = Cast<ATectonicPlanetV6PreviewActor>(*It))
			{
				return Actor;
			}
		}
	}

	return nullptr;
}

void SAurousTectonicControlPanel::SetSelectedActor(AActor* InActor)
{
	SelectedActor = InActor;
	SyncPendingSettingsFromActor();
	RefreshMetrics();
}

void SAurousTectonicControlPanel::SyncPendingSettingsFromActor()
{
	if (ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
	{
		PendingSampleCount = Actor->GetConfiguredSampleCount();
		PendingPlateCount = Actor->GetConfiguredPlateCount();
		PendingRandomSeed = Actor->GetConfiguredRandomSeed();
		PendingBoundaryWarpAmplitude = Actor->GetConfiguredBoundaryWarpAmplitude();
		PendingContinentalFraction = Actor->GetConfiguredContinentalFraction();
		PendingExportWidth = Actor->GetConfiguredExportWidth();
		SelectVisualizationMode(Actor->GetVisualizationMode());
	}
	else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
	{
		PendingSampleCount = V6Actor->GetConfiguredSampleCount();
		PendingPlateCount = V6Actor->GetConfiguredPlateCount();
		PendingRandomSeed = V6Actor->GetConfiguredRandomSeed();
		PendingBoundaryWarpAmplitude = V6Actor->GetConfiguredBoundaryWarpAmplitude();
		PendingContinentalFraction = V6Actor->GetConfiguredContinentalFraction();
		PendingExportWidth = V6Actor->GetConfiguredExportWidth();
		SelectVisualizationMode(V6Actor->GetVisualizationMode());
	}
}

void SAurousTectonicControlPanel::RefreshMetrics()
{
	if (ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
	{
		const FTectonicPlanetMetrics Metrics = Actor->GetMetrics();
		CachedCurrentStep = Metrics.CurrentStep;
		CachedLastAdvanceStepMs = Actor->GetLastAdvanceStepMs();
		CachedMetricsText = FText::FromString(FString::Printf(
			TEXT("Preset: %s\nRuntime: %s\nStep: %d\nSamples: %d\nPlates: %d\nTerranes: %d\nContinental Fraction: %.3f\nElevation Min/Max/Mean: %.3f / %.3f / %.3f"),
			*Actor->GetActiveRuntimePresetLabel(),
			*Actor->GetRuntimeConfigSummary(),
			Metrics.CurrentStep,
			Metrics.SampleCount,
			Metrics.PlateCount,
			Metrics.TerraneCount,
			Metrics.ContinentalFraction,
			Metrics.MinElevation,
			Metrics.MaxElevation,
			Metrics.MeanElevation));
	}
	else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
	{
		const FTectonicPlanet& Planet = V6Actor->GetPlanetV6().GetPlanet();
		double MinElevation = 0.0;
		double MaxElevation = 0.0;
		double MeanElevation = 0.0;
		if (!Planet.Samples.IsEmpty())
		{
			MinElevation = TNumericLimits<double>::Max();
			MaxElevation = -TNumericLimits<double>::Max();
			for (const FSample& Sample : Planet.Samples)
			{
				const double Elevation = static_cast<double>(Sample.Elevation);
				MinElevation = FMath::Min(MinElevation, Elevation);
				MaxElevation = FMath::Max(MaxElevation, Elevation);
				MeanElevation += Elevation;
			}
			MeanElevation /= static_cast<double>(Planet.Samples.Num());
		}
		CachedCurrentStep = Planet.CurrentStep;
		CachedLastAdvanceStepMs = V6Actor->GetLastAdvanceStepMs();
		CachedMetricsText = FText::FromString(FString::Printf(
			TEXT("Preset: %s\nRuntime: %s\nStep: %d\nSamples: %d\nPlates: %d\nContinental Fraction: %.3f\nPeriodic Solves: %d\nCollisions: %d\nRifts: %d\nElevation Min/Max/Mean: %.3f / %.3f / %.3f"),
			*V6Actor->GetActiveRuntimePresetLabel(),
			*V6Actor->GetRuntimeConfigSummary(),
			Planet.CurrentStep,
			Planet.Samples.Num(),
			Planet.Plates.Num(),
			V6Actor->GetContinentalAreaFraction(),
			V6Actor->GetPeriodicSolveCountValue(),
			V6Actor->GetCumulativeCollisionCountValue(),
			V6Actor->GetCumulativeRiftCountValue(),
			MinElevation,
			MaxElevation,
			MeanElevation));
	}
	else
	{
		CachedCurrentStep = 0;
		CachedLastAdvanceStepMs = 0.0;
		CachedMetricsText = FText::FromString(TEXT("No actor selected."));
	}
}

ETectonicMapExportMode SAurousTectonicControlPanel::GetSelectedVisualizationMode() const
{
	if (!VisualizationComboBox.IsValid())
	{
		return ETectonicMapExportMode::Elevation;
	}

	const TSharedPtr<FString> SelectedItem = VisualizationComboBox->GetSelectedItem();
	if (!SelectedItem.IsValid())
	{
		return ETectonicMapExportMode::Elevation;
	}

	if (*SelectedItem == TEXT("PlateId"))
	{
		return ETectonicMapExportMode::PlateId;
	}
	if (*SelectedItem == TEXT("CrustType"))
	{
		return ETectonicMapExportMode::CrustType;
	}
	if (*SelectedItem == TEXT("ContinentalWeight"))
	{
		return ETectonicMapExportMode::ContinentalWeight;
	}
	return ETectonicMapExportMode::Elevation;
}

void SAurousTectonicControlPanel::SelectVisualizationMode(const ETectonicMapExportMode Mode)
{
	if (!VisualizationComboBox.IsValid())
	{
		return;
	}

	int32 OptionIndex = 0;
	switch (Mode)
	{
	case ETectonicMapExportMode::PlateId:
		OptionIndex = 1;
		break;

	case ETectonicMapExportMode::CrustType:
		OptionIndex = 2;
		break;

	case ETectonicMapExportMode::ContinentalWeight:
		OptionIndex = 3;
		break;

	case ETectonicMapExportMode::Elevation:
	default:
		OptionIndex = 0;
		break;
	}

	if (VisualizationOptions.IsValidIndex(OptionIndex))
	{
		VisualizationComboBox->SetSelectedItem(VisualizationOptions[OptionIndex]);
	}
}

bool SAurousTectonicControlPanel::EnsureSelectedActor(const TCHAR* ActionName) const
{
	if (GetSelectedActor())
	{
		return true;
	}

	FMessageDialog::Open(
		EAppMsgType::Ok,
		FText::FromString(FString::Printf(TEXT("%s requires a selected ATectonicPlanetActor or ATectonicPlanetV6PreviewActor."), ActionName)));
	return false;
}

FReply SAurousTectonicControlPanel::OnUseSelectedClicked()
{
	SetSelectedActor(FindActorInEditorSelection());
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnClearClicked()
{
	SetSelectedActor(nullptr);
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnGenerateClicked()
{
	if (!EnsureSelectedActor(TEXT("Generate")))
	{
		return FReply::Handled();
	}

	if (ATectonicPlanetActor* LegacyActor = GetSelectedLegacyActor())
	{
		LegacyActor->GeneratePlanet(
			PendingSampleCount,
			PendingPlateCount,
			PendingRandomSeed,
			PendingBoundaryWarpAmplitude,
			PendingContinentalFraction);
		LegacyActor->BuildMeshWithMode(GetSelectedVisualizationMode());
	}
	else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
	{
		V6Actor->GeneratePlanet(
			PendingSampleCount,
			PendingPlateCount,
			PendingRandomSeed,
			PendingBoundaryWarpAmplitude,
			PendingContinentalFraction);
		V6Actor->BuildMeshWithMode(GetSelectedVisualizationMode());
	}
	RefreshMetrics();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnStep1Clicked()
{
	if (!EnsureSelectedActor(TEXT("Step 1")))
	{
		return FReply::Handled();
	}

	if (ATectonicPlanetActor* LegacyActor = GetSelectedLegacyActor())
	{
		LegacyActor->BuildMeshWithMode(GetSelectedVisualizationMode());
		LegacyActor->AdvancePlanetSteps(1);
	}
	else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
	{
		V6Actor->BuildMeshWithMode(GetSelectedVisualizationMode());
		V6Actor->AdvancePlanetSteps(1);
	}
	RefreshMetrics();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnStep10Clicked()
{
	if (!EnsureSelectedActor(TEXT("Step 10")))
	{
		return FReply::Handled();
	}

	if (ATectonicPlanetActor* LegacyActor = GetSelectedLegacyActor())
	{
		LegacyActor->BuildMeshWithMode(GetSelectedVisualizationMode());
		LegacyActor->AdvancePlanetSteps(10);
	}
	else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
	{
		V6Actor->BuildMeshWithMode(GetSelectedVisualizationMode());
		V6Actor->AdvancePlanetSteps(10);
	}
	RefreshMetrics();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnStep50Clicked()
{
	if (!EnsureSelectedActor(TEXT("Step 50")))
	{
		return FReply::Handled();
	}

	if (ATectonicPlanetActor* LegacyActor = GetSelectedLegacyActor())
	{
		LegacyActor->BuildMeshWithMode(GetSelectedVisualizationMode());
		LegacyActor->AdvancePlanetSteps(50);
	}
	else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
	{
		V6Actor->BuildMeshWithMode(GetSelectedVisualizationMode());
		V6Actor->AdvancePlanetSteps(50);
	}
	RefreshMetrics();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnExportMapsClicked()
{
	if (!EnsureSelectedActor(TEXT("Export Maps")))
	{
		return FReply::Handled();
	}

	FString OutputDirectory;
	FString ExportError;
	const int32 ExportHeight = FMath::Max(MinimumExportHeight, PendingExportWidth / 2);
	bool bExported = false;
	if (ATectonicPlanetActor* LegacyActor = GetSelectedLegacyActor())
	{
		bExported = LegacyActor->ExportCurrentMaps(ETectonicMapExportMode::All, PendingExportWidth, ExportHeight, OutputDirectory, ExportError);
	}
	else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
	{
		bExported = V6Actor->ExportCurrentMaps(ETectonicMapExportMode::All, PendingExportWidth, ExportHeight, OutputDirectory, ExportError);
	}

	if (!bExported)
	{
		FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(ExportError));
	}
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnRefreshMetricsClicked()
{
	RefreshMetrics();
	return FReply::Handled();
}

void SAurousTectonicControlPanel::OnVisualizationModeChanged(TSharedPtr<FString> NewSelection, ESelectInfo::Type SelectInfo)
{
	if (SelectInfo == ESelectInfo::Direct)
	{
		return;
	}

	if (ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
	{
		Actor->BuildMeshWithMode(GetSelectedVisualizationMode());
		RefreshMetrics();
	}
	else if (ATectonicPlanetV6PreviewActor* V6Actor = GetSelectedV6Actor())
	{
		V6Actor->BuildMeshWithMode(GetSelectedVisualizationMode());
		RefreshMetrics();
	}
}

FText SAurousTectonicControlPanel::GetSelectedActorText() const
{
	if (const ATectonicPlanetActor* Actor = GetSelectedLegacyActor())
	{
		return FormatMetricLine(TEXT("Selected"), FString::Printf(TEXT("%s (Legacy)"), *Actor->GetActorNameOrLabel()));
	}

	if (const ATectonicPlanetV6PreviewActor* Actor = GetSelectedV6Actor())
	{
		return FormatMetricLine(TEXT("Selected"), FString::Printf(TEXT("%s (V6 Preview)"), *Actor->GetActorNameOrLabel()));
	}

	return FormatMetricLine(TEXT("Selected"), TEXT("None"));
}

FText SAurousTectonicControlPanel::GetCurrentStepText() const
{
	return FText::FromString(FString::Printf(TEXT("Step: %d"), CachedCurrentStep));
}

FText SAurousTectonicControlPanel::GetTimingText() const
{
	return FText::FromString(FString::Printf(TEXT("Per-step timing: %.3f ms"), CachedLastAdvanceStepMs));
}

FText SAurousTectonicControlPanel::GetMetricsText() const
{
	return CachedMetricsText;
}

#undef LOCTEXT_NAMESPACE

#endif
