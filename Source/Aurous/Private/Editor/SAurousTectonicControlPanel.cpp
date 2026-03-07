#include "Editor/SAurousTectonicControlPanel.h"

#if WITH_EDITOR

#include "TectonicPlanetActor.h"
#include "TectonicPlanetOwnershipUtils.h"
#include "Editor.h"
#include "Engine/Selection.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformFileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAABBTree3.h"
#include "Misc/FileHelper.h"
#include "Misc/MessageDialog.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Styling/AppStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Input/STextComboBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SExpandableArea.h"
#include "Widgets/Layout/SGridPanel.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/Layout/SWrapBox.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
	constexpr float ExportSubductionInfluenceRadiusKm = 1800.0f;
	constexpr float ExportSubductionPeakDistanceKm = 350.0f;
	constexpr float ExportSubductionTrenchRadiusKm = 400.0f;
	constexpr float ExportCollisionInfluenceRadiusKm = 4200.0f;
	constexpr float ExportMaxSubductionSpeedMmPerYear = 100.0f;

	FORCEINLINE double Dot3(const FVector3d& A, const FVector3d& B)
	{
		return A.X * B.X + A.Y * B.Y + A.Z * B.Z;
	}

	float SmoothStep01(const float X)
	{
		const float Clamped = FMath::Clamp(X, 0.0f, 1.0f);
		return (3.0f * Clamped * Clamped) - (2.0f * Clamped * Clamped * Clamped);
	}

	int32 ResolveWeightedDominantValueFromInts(const int32 Values[3], const FVector3d& Barycentric)
	{
		struct FValueWeight
		{
			int32 Value = INDEX_NONE;
			double Weight = 0.0;
		};

		FValueWeight Weights[3];
		auto Accumulate = [&Weights](const int32 Value, const double Weight)
		{
			if (Value == INDEX_NONE)
			{
				return;
			}

			for (FValueWeight& Entry : Weights)
			{
				if (Entry.Value == Value)
				{
					Entry.Weight += Weight;
					return;
				}
				if (Entry.Value == INDEX_NONE)
				{
					Entry.Value = Value;
					Entry.Weight = Weight;
					return;
				}
			}
		};

		Accumulate(Values[0], Barycentric.X);
		Accumulate(Values[1], Barycentric.Y);
		Accumulate(Values[2], Barycentric.Z);

		int32 BestValue = INDEX_NONE;
		double BestWeight = -1.0;
		for (const FValueWeight& Entry : Weights)
		{
			if (Entry.Value == INDEX_NONE)
			{
				continue;
			}
			if (Entry.Weight > BestWeight + UE_DOUBLE_SMALL_NUMBER ||
				(FMath::IsNearlyEqual(Entry.Weight, BestWeight, UE_DOUBLE_SMALL_NUMBER) &&
					(BestValue == INDEX_NONE || Entry.Value < BestValue)))
			{
				BestValue = Entry.Value;
				BestWeight = Entry.Weight;
			}
		}

		return BestValue;
	}

	FColor GetSubductionRoleColor(const ESubductionRole Role, const bool bIsFront)
	{
		if (bIsFront)
		{
			return FColor::White;
		}

		switch (Role)
		{
		case ESubductionRole::Overriding:
			return FColor(255, 140, 40);
		case ESubductionRole::Subducting:
			return FColor(40, 180, 255);
		case ESubductionRole::None:
		default:
			return FColor::Black;
		}
	}

	FColor GetOrogenyColor(const EOrogenyType OrogenyType)
	{
		switch (OrogenyType)
		{
		case EOrogenyType::Andean:
			return FColor(255, 120, 0);
		case EOrogenyType::Himalayan:
			return FColor(255, 0, 200);
		case EOrogenyType::None:
		default:
			return FColor::Black;
		}
	}

	float ComputeSubductionInfluenceForExport(const float DistanceKm, const float SpeedMmPerYear)
	{
		if (DistanceKm < 0.0f || DistanceKm > ExportSubductionInfluenceRadiusKm)
		{
			return 0.0f;
		}

		float DistanceInfluence = 0.0f;
		if (DistanceKm <= ExportSubductionPeakDistanceKm)
		{
			DistanceInfluence = SmoothStep01(DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, ExportSubductionPeakDistanceKm));
		}
		else
		{
			const float FalloffAlpha = (DistanceKm - ExportSubductionPeakDistanceKm) /
				FMath::Max(KINDA_SMALL_NUMBER, ExportSubductionInfluenceRadiusKm - ExportSubductionPeakDistanceKm);
			DistanceInfluence = 1.0f - SmoothStep01(FalloffAlpha);
		}

		const float SpeedInfluence = FMath::Clamp(SpeedMmPerYear / FMath::Max(KINDA_SMALL_NUMBER, ExportMaxSubductionSpeedMmPerYear), 0.0f, 1.0f);
		return DistanceInfluence * SpeedInfluence;
	}

	float ComputeTrenchInfluenceForExport(const float DistanceKm, const float SpeedMmPerYear)
	{
		if (DistanceKm < 0.0f || DistanceKm > ExportSubductionTrenchRadiusKm)
		{
			return 0.0f;
		}

		const float DistanceInfluence = FMath::Square(
			FMath::Clamp(1.0f - (DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, ExportSubductionTrenchRadiusKm)), 0.0f, 1.0f));
		const float SpeedInfluence = FMath::Clamp(SpeedMmPerYear / FMath::Max(KINDA_SMALL_NUMBER, ExportMaxSubductionSpeedMmPerYear), 0.0f, 1.0f);
		return DistanceInfluence * SpeedInfluence;
	}

	float ComputeCollisionInfluenceForExport(const float DistanceKm, const float SpeedMmPerYear)
	{
		if (DistanceKm < 0.0f || DistanceKm > ExportCollisionInfluenceRadiusKm)
		{
			return 0.0f;
		}

		const float RadiusAlpha = FMath::Clamp(DistanceKm / FMath::Max(KINDA_SMALL_NUMBER, ExportCollisionInfluenceRadiusKm), 0.0f, 1.0f);
		const float DistanceInfluence = FMath::Square(1.0f - (RadiusAlpha * RadiusAlpha));
		const float SpeedInfluence = FMath::Clamp(SpeedMmPerYear / FMath::Max(KINDA_SMALL_NUMBER, ExportMaxSubductionSpeedMmPerYear), 0.0f, 1.0f);
		return DistanceInfluence * SpeedInfluence;
	}

	FVector3d PixelToSphereDirection(const int32 X, const int32 Y, const int32 Width, const int32 Height)
	{
		const double U = (static_cast<double>(X) + 0.5) / static_cast<double>(Width);
		const double V = (static_cast<double>(Y) + 0.5) / static_cast<double>(Height);
		const double Longitude = (U * 2.0 - 1.0) * static_cast<double>(PI);
		const double Latitude = (0.5 - V) * static_cast<double>(PI);
		const double CosLatitude = FMath::Cos(Latitude);
		return FVector3d(
			CosLatitude * FMath::Cos(Longitude),
			CosLatitude * FMath::Sin(Longitude),
			FMath::Sin(Latitude));
	}

	FVector3d ComputePlanarBarycentric(const FVector3d& A, const FVector3d& B, const FVector3d& C, const FVector3d& P)
	{
		// Robust barycentrics for point projected onto triangle plane:
		// weights by oriented sub-triangle areas w.r.t. the triangle normal.
		const FVector3d TriangleNormal = (B - A).Cross(C - A);
		const double Denominator = Dot3(TriangleNormal, TriangleNormal);
		constexpr double DegenerateTriangleEpsilon = 1e-24;
		if (Denominator <= DegenerateTriangleEpsilon)
		{
			return FVector3d(-1.0, -1.0, -1.0);
		}

		const double U = Dot3(TriangleNormal, (B - P).Cross(C - P)) / Denominator;
		const double V = Dot3(TriangleNormal, (C - P).Cross(A - P)) / Denominator;
		const double W = Dot3(TriangleNormal, (A - P).Cross(B - P)) / Denominator;
		if (!FMath::IsFinite(U) || !FMath::IsFinite(V) || !FMath::IsFinite(W))
		{
			return FVector3d(-1.0, -1.0, -1.0);
		}

		const double Sum = U + V + W;
		if (FMath::Abs(Sum) <= 1e-15)
		{
			return FVector3d(-1.0, -1.0, -1.0);
		}

		return FVector3d(U / Sum, V / Sum, W / Sum);
	}

	FColor GetPlateDebugColor(const int32 PlateId)
	{
		static const FColor PlateColors[] = {
			FColor(230, 25, 75),
			FColor(60, 180, 75),
			FColor(255, 225, 25),
			FColor(0, 130, 200),
			FColor(245, 130, 48),
			FColor(145, 30, 180),
			FColor(70, 240, 240),
			FColor(240, 50, 230),
			FColor(210, 245, 60),
			FColor(250, 190, 212),
			FColor(0, 128, 128),
			FColor(220, 190, 255)
		};

		if (PlateId < 0)
		{
			return FColor::Black;
		}

		const int32 ColorIndex = PlateId % UE_ARRAY_COUNT(PlateColors);
		return PlateColors[ColorIndex];
	}

	FColor GetTerraneDebugColor(const int32 TerraneId)
	{
		return GetPlateDebugColor(TerraneId);
	}

	bool SaveColorPng(const FString& FilePath, const int32 Width, const int32 Height, const TArray<FColor>& Pixels)
	{
		if (Pixels.Num() != Width * Height)
		{
			return false;
		}

		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		const TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
		if (!Wrapper.IsValid())
		{
			return false;
		}

		if (!Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8))
		{
			return false;
		}

		const TArray64<uint8>& CompressedData = Wrapper->GetCompressed();
		return FFileHelper::SaveArrayToFile(CompressedData, *FilePath);
	}

	bool SaveGray8Png(const FString& FilePath, const int32 Width, const int32 Height, const TArray<uint8>& Pixels)
	{
		if (Pixels.Num() != Width * Height)
		{
			return false;
		}

		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		const TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
		if (!Wrapper.IsValid())
		{
			return false;
		}

		if (!Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(uint8), Width, Height, ERGBFormat::Gray, 8))
		{
			return false;
		}

		const TArray64<uint8>& CompressedData = Wrapper->GetCompressed();
		return FFileHelper::SaveArrayToFile(CompressedData, *FilePath);
	}

	bool SaveGray16Png(const FString& FilePath, const int32 Width, const int32 Height, const TArray<uint16>& Pixels)
	{
		if (Pixels.Num() != Width * Height)
		{
			return false;
		}

		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		const TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
		if (!Wrapper.IsValid())
		{
			return false;
		}

		if (!Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(uint16), Width, Height, ERGBFormat::Gray, 16))
		{
			return false;
		}

		const TArray64<uint8>& CompressedData = Wrapper->GetCompressed();
		return FFileHelper::SaveArrayToFile(CompressedData, *FilePath);
	}
}

#define LOCTEXT_NAMESPACE "SAurousTectonicControlPanel"

void SAurousTectonicControlPanel::Construct(const FArguments& InArgs)
{
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Plate ID")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Crust Type")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Boundary")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Boundary Type")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Elevation")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Crust Age")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Subduction Role")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Subduction Distance")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Orogeny Type")));
	DebugModeOptions.Add(MakeShared<FString>(TEXT("Terrane ID")));

	SetSelectedActor(FindActorInEditorSelection());

	ChildSlot
	[
		SNew(SBorder)
		.Padding(10.0f)
		[
			SNew(SVerticalBox)

			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 8.0f)
			[
				SNew(SExpandableArea)
				.InitiallyCollapsed(false)
				.HeaderContent()
				[
					SNew(STextBlock)
					.Text(LOCTEXT("ActorBindingSection", "Actor Binding"))
					.Font(FAppStyle::GetFontStyle("BoldFont"))
				]
				.BodyContent()
				[
					SNew(SVerticalBox)

					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0.0f, 4.0f, 0.0f, 8.0f)
					[
						SNew(STextBlock)
						.Text(this, &SAurousTectonicControlPanel::GetSelectedActorText)
					]

					+ SVerticalBox::Slot()
					.AutoHeight()
					[
						SNew(SWrapBox)
						+ SWrapBox::Slot()
						.Padding(0.0f, 0.0f, 6.0f, 6.0f)
						[
							SNew(SButton)
							.Text(LOCTEXT("UseSelectedActorButton", "Use Selected Actor"))
							.ToolTipText(LOCTEXT("UseSelectedActorTooltip", "Bind this panel to the currently selected ATectonicPlanetActor."))
							.OnClicked(this, &SAurousTectonicControlPanel::OnUseSelectedActorClicked)
						]

						+ SWrapBox::Slot()
						.Padding(0.0f, 0.0f, 6.0f, 6.0f)
						[
							SNew(SButton)
							.Text(LOCTEXT("ClearSelectedActorButton", "Clear"))
							.ToolTipText(LOCTEXT("ClearSelectedActorTooltip", "Unbind the current actor from this panel."))
							.OnClicked(this, &SAurousTectonicControlPanel::OnClearSelectedActorClicked)
						]
					]
				]
			]

			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 8.0f)
			[
				SNew(SSeparator)
			]

			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 8.0f)
			[
				SNew(SExpandableArea)
				.InitiallyCollapsed(false)
				.HeaderContent()
				[
					SNew(STextBlock)
					.Text(LOCTEXT("GenerationSettingsSection", "Generation Settings"))
					.Font(FAppStyle::GetFontStyle("BoldFont"))
				]
				.BodyContent()
				[
					SNew(SVerticalBox)

					+ SVerticalBox::Slot()
					.AutoHeight()
					[
						SNew(SGridPanel)
						.FillColumn(1, 1.0f)

						+ SGridPanel::Slot(0, 0)
						.Padding(0.0f, 0.0f, 8.0f, 6.0f)
						.VAlign(VAlign_Center)
						[
							SNew(STextBlock).Text(LOCTEXT("NumSamplesLabel", "Num Samples"))
						]
						+ SGridPanel::Slot(1, 0)
						.Padding(0.0f, 0.0f, 0.0f, 6.0f)
						[
							SNew(SSpinBox<int32>)
							.MinValue(4)
							.MaxValue(2000000)
							.Value_Lambda([this]() { return PendingNumSamples; })
							.OnValueChanged_Lambda([this](int32 NewValue) { PendingNumSamples = NewValue; })
						]

						+ SGridPanel::Slot(0, 1)
						.Padding(0.0f, 0.0f, 8.0f, 6.0f)
						.VAlign(VAlign_Center)
						[
							SNew(STextBlock).Text(LOCTEXT("NumPlatesLabel", "Num Plates"))
						]
						+ SGridPanel::Slot(1, 1)
						.Padding(0.0f, 0.0f, 0.0f, 6.0f)
						[
							SNew(SSpinBox<int32>)
							.MinValue(1)
							.MaxValue(64)
							.Value_Lambda([this]() { return PendingNumPlates; })
							.OnValueChanged_Lambda([this](int32 NewValue) { PendingNumPlates = NewValue; })
						]

						+ SGridPanel::Slot(0, 2)
						.Padding(0.0f, 0.0f, 8.0f, 6.0f)
						.VAlign(VAlign_Center)
						[
							SNew(STextBlock).Text(LOCTEXT("SeedLabel", "Random Seed"))
						]
						+ SGridPanel::Slot(1, 2)
						.Padding(0.0f, 0.0f, 0.0f, 6.0f)
						[
							SNew(SSpinBox<int32>)
							.MinValue(-1000000000)
							.MaxValue(1000000000)
							.Value_Lambda([this]() { return PendingRandomSeed; })
							.OnValueChanged_Lambda([this](int32 NewValue) { PendingRandomSeed = NewValue; })
						]
					]

					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0.0f, 2.0f, 0.0f, 0.0f)
					[
						SNew(SButton)
						.Text(LOCTEXT("GenerateButton", "Generate"))
						.IsEnabled_Lambda([this]()
						{
							const ATectonicPlanetActor* Actor = GetSelectedActor();
							return !Actor || !Actor->IsSimulationThreadRunningThreadSafe();
						})
						.OnClicked(this, &SAurousTectonicControlPanel::OnGenerateClicked)
					]
				]
			]

			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 8.0f)
			[
				SNew(SSeparator)
			]

			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 8.0f)
			[
				SNew(SExpandableArea)
				.InitiallyCollapsed(false)
				.HeaderContent()
				[
					SNew(STextBlock)
					.Text(LOCTEXT("SimulationControlsSection", "Simulation Controls"))
					.Font(FAppStyle::GetFontStyle("BoldFont"))
				]
				.BodyContent()
				[
					SNew(SVerticalBox)

					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0.0f, 2.0f, 0.0f, 8.0f)
					[
						SNew(SHorizontalBox)

						+ SHorizontalBox::Slot()
						.AutoWidth()
						.Padding(0.0f, 0.0f, 8.0f, 0.0f)
						[
							SNew(STextBlock)
							.Text(LOCTEXT("SimulationStatusLabel", "Status:"))
						]

						+ SHorizontalBox::Slot()
						.AutoWidth()
						[
							SNew(STextBlock)
							.Text(this, &SAurousTectonicControlPanel::GetSimulationStatusText)
							.ColorAndOpacity(this, &SAurousTectonicControlPanel::GetSimulationStatusColor)
							.Font(FAppStyle::GetFontStyle("BoldFont"))
						]
					]

					+ SVerticalBox::Slot()
					.AutoHeight()
					[
						SNew(SGridPanel)
						.FillColumn(1, 1.0f)

						+ SGridPanel::Slot(0, 0)
						.Padding(0.0f, 0.0f, 8.0f, 6.0f)
						.VAlign(VAlign_Center)
						[
							SNew(STextBlock).Text(LOCTEXT("SimRateLabel", "Sim Steps / Sec"))
						]
						+ SGridPanel::Slot(1, 0)
						.Padding(0.0f, 0.0f, 0.0f, 6.0f)
						[
							SNew(SSpinBox<float>)
							.MinValue(0.1f)
							.MaxValue(240.0f)
							.Value_Lambda([this]() { return PendingSimulationRate; })
							.OnValueChanged_Lambda([this](float NewValue) { PendingSimulationRate = NewValue; })
						]

						+ SGridPanel::Slot(0, 1)
						.Padding(0.0f, 0.0f, 8.0f, 6.0f)
						.VAlign(VAlign_Center)
						[
							SNew(STextBlock).Text(LOCTEXT("DebugModeLabel", "Debug Mode"))
						]
						+ SGridPanel::Slot(1, 1)
						.Padding(0.0f, 0.0f, 0.0f, 6.0f)
						[
							SAssignNew(DebugModeComboBox, STextComboBox)
							.OptionsSource(&DebugModeOptions)
							.InitiallySelectedItem(DebugModeOptions.IsValidIndex(PendingDebugModeIndex) ? DebugModeOptions[PendingDebugModeIndex] : nullptr)
							.OnSelectionChanged_Lambda([this](TSharedPtr<FString> NewSelection, ESelectInfo::Type)
							{
								const int32 NewIndex = DebugModeOptions.IndexOfByKey(NewSelection);
								if (NewIndex == INDEX_NONE)
								{
									return;
								}

								PendingDebugModeIndex = NewIndex;
								if (ATectonicPlanetActor* Actor = GetSelectedActor())
								{
									ApplyPendingSettingsToActor(true);
									Actor->UpdateDebugColors();
								}
							})
						]
					]

					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0.0f, 4.0f, 0.0f, 0.0f)
					[
						SNew(SUniformGridPanel)
						.SlotPadding(FMargin(0.0f, 0.0f, 6.0f, 6.0f))
						.MinDesiredSlotWidth(120.0f)

						+ SUniformGridPanel::Slot(0, 0)
						[
							SNew(SButton)
							.Text(LOCTEXT("StartButton", "Start"))
							.IsEnabled_Lambda([this]()
							{
								const ATectonicPlanetActor* Actor = GetSelectedActor();
								return !Actor || !Actor->IsSimulationThreadRunningThreadSafe();
							})
							.OnClicked(this, &SAurousTectonicControlPanel::OnStartClicked)
						]
						+ SUniformGridPanel::Slot(1, 0)
						[
							SNew(SButton)
							.Text(LOCTEXT("StopButton", "Stop"))
							.IsEnabled_Lambda([this]()
							{
								const ATectonicPlanetActor* Actor = GetSelectedActor();
								return !Actor || Actor->IsSimulationThreadRunningThreadSafe();
							})
							.OnClicked(this, &SAurousTectonicControlPanel::OnStopClicked)
						]
						+ SUniformGridPanel::Slot(2, 0)
						[
							SNew(SButton)
							.Text(LOCTEXT("UpdateColorsButton", "Update Colors"))
							.OnClicked(this, &SAurousTectonicControlPanel::OnUpdateColorsClicked)
						]
						+ SUniformGridPanel::Slot(0, 1)
						[
							SNew(SButton)
							.Text(LOCTEXT("Step1Button", "Step x1"))
							.IsEnabled_Lambda([this]()
							{
								const ATectonicPlanetActor* Actor = GetSelectedActor();
								return !Actor || !Actor->IsSimulationThreadRunningThreadSafe();
							})
							.OnClicked(this, &SAurousTectonicControlPanel::OnStep1Clicked)
						]
						+ SUniformGridPanel::Slot(1, 1)
						[
							SNew(SButton)
							.Text(LOCTEXT("Step10Button", "Step x10"))
							.IsEnabled_Lambda([this]()
							{
								const ATectonicPlanetActor* Actor = GetSelectedActor();
								return !Actor || !Actor->IsSimulationThreadRunningThreadSafe();
							})
							.OnClicked(this, &SAurousTectonicControlPanel::OnStep10Clicked)
						]
						+ SUniformGridPanel::Slot(2, 1)
						[
							SNew(SButton)
							.Text(LOCTEXT("ForceReconcileButton", "Force Reconcile"))
							.IsEnabled_Lambda([this]()
							{
								const ATectonicPlanetActor* Actor = GetSelectedActor();
								return !Actor || !Actor->IsSimulationThreadRunningThreadSafe();
							})
							.OnClicked(this, &SAurousTectonicControlPanel::OnForceReconcileClicked)
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
							SNew(STextBlock)
							.Text(LOCTEXT("ExportWidthLabel", "Export Width"))
						]

						+ SHorizontalBox::Slot()
						.AutoWidth()
						.Padding(0.0f, 0.0f, 8.0f, 0.0f)
						[
							SNew(SSpinBox<int32>)
							.MinValue(256)
							.MaxValue(8192)
							.Delta(256)
							.Value_Lambda([this]() { return PendingExportWidth; })
							.OnValueChanged_Lambda([this](int32 NewValue)
							{
								const int32 Clamped = FMath::Clamp(NewValue, 256, 8192);
								PendingExportWidth = (Clamped % 2 == 0) ? Clamped : (Clamped - 1);
							})
						]

						+ SHorizontalBox::Slot()
						.AutoWidth()
						.VAlign(VAlign_Center)
						.Padding(0.0f, 0.0f, 12.0f, 0.0f)
						[
							SNew(STextBlock)
							.Text(this, &SAurousTectonicControlPanel::GetExportHeightText)
						]

						+ SHorizontalBox::Slot()
						.AutoWidth()
						[
							SNew(SButton)
							.Text(LOCTEXT("ExportMapsButton", "Export Maps"))
							.ToolTipText(LOCTEXT("ExportMapsTooltip", "Export equirectangular tectonic maps to Saved/MapExports/Step_{N}/"))
							.OnClicked(this, &SAurousTectonicControlPanel::OnExportMapsClicked)
						]
					]
				]
			]

			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 0.0f, 0.0f, 8.0f)
			[
				SNew(SSeparator)
			]

			+ SVerticalBox::Slot()
			.FillHeight(1.0f)
			[
				SNew(SExpandableArea)
				.InitiallyCollapsed(false)
				.HeaderContent()
				[
					SNew(STextBlock)
					.Text(LOCTEXT("MetricsSection", "Metrics"))
					.Font(FAppStyle::GetFontStyle("BoldFont"))
				]
				.BodyContent()
				[
					SNew(SScrollBox)
					+ SScrollBox::Slot()
					[
						SAssignNew(MetricsTextBlock, STextBlock)
						.Text(this, &SAurousTectonicControlPanel::GetMetricsText)
						.AutoWrapText(true)
					]
				]
			]
		]
	];

	UpdateMetricsActiveTimerRegistration();
}

ATectonicPlanetActor* SAurousTectonicControlPanel::GetSelectedActor() const
{
	return SelectedActor.Get();
}

ATectonicPlanetActor* SAurousTectonicControlPanel::FindActorInEditorSelection() const
{
	if (!GEditor)
	{
		return nullptr;
	}

	USelection* SelectedActors = GEditor->GetSelectedActors();
	if (!SelectedActors)
	{
		return nullptr;
	}

	for (FSelectionIterator It(*SelectedActors); It; ++It)
	{
		if (ATectonicPlanetActor* PlanetActor = Cast<ATectonicPlanetActor>(*It))
		{
			return PlanetActor;
		}
	}

	return nullptr;
}

void SAurousTectonicControlPanel::SetSelectedActor(ATectonicPlanetActor* InActor)
{
	SelectedActor = InActor;
	InvalidateExportCache();
	SyncPendingSettingsFromActor();
	UpdateMetricsActiveTimerRegistration();
}

void SAurousTectonicControlPanel::SyncPendingSettingsFromActor()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return;
	}

	PendingNumSamples = Actor->NumSamples;
	PendingNumPlates = Actor->NumPlates;
	PendingRandomSeed = Actor->RandomSeed;
	PendingSimulationRate = Actor->SimulationTimestepsPerSecond;
	PendingDebugModeIndex = DebugModeToIndex(static_cast<uint8>(Actor->DebugMode));

	if (DebugModeComboBox.IsValid() && DebugModeOptions.IsValidIndex(PendingDebugModeIndex))
	{
		DebugModeComboBox->SetSelectedItem(DebugModeOptions[PendingDebugModeIndex]);
	}
}

void SAurousTectonicControlPanel::ApplyPendingSettingsToActor(bool bApplyDebugModeOnly) const
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return;
	}

	Actor->Modify();

	if (!bApplyDebugModeOnly)
	{
		Actor->NumSamples = FMath::Max(4, PendingNumSamples);
		Actor->NumPlates = FMath::Max(1, PendingNumPlates);
		Actor->RandomSeed = PendingRandomSeed;
		Actor->SimulationTimestepsPerSecond = FMath::Max(0.1f, PendingSimulationRate);
	}

	Actor->DebugMode = static_cast<EPlanetDebugMode>(IndexToDebugModeValue(PendingDebugModeIndex));
}

FReply SAurousTectonicControlPanel::OnUseSelectedActorClicked()
{
	ATectonicPlanetActor* Actor = FindActorInEditorSelection();
	SetSelectedActor(Actor);

	if (!Actor)
	{
		FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("NoPlanetActorSelected", "Select an ATectonicPlanetActor in the level first."));
	}

	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnClearSelectedActorClicked()
{
	SetSelectedActor(nullptr);
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnGenerateClicked()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return OnUseSelectedActorClicked();
	}

	ApplyPendingSettingsToActor(false);
	Actor->GeneratePlanet();
	InvalidateExportCache();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnStartClicked()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return OnUseSelectedActorClicked();
	}

	ApplyPendingSettingsToActor(false);
	Actor->StartSimulation();
	LastStepCommandTime = -1.0;
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnStopClicked()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return OnUseSelectedActorClicked();
	}

	Actor->StopSimulation();
	LastStepCommandTime = -1.0;
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnStep1Clicked()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return OnUseSelectedActorClicked();
	}

	Actor->StepSimulationSteps(1);
	LastStepCommandTime = FPlatformTime::Seconds();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnStep10Clicked()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return OnUseSelectedActorClicked();
	}

	Actor->StepSimulationSteps(10);
	LastStepCommandTime = FPlatformTime::Seconds();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnForceReconcileClicked()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return OnUseSelectedActorClicked();
	}

	Actor->ForceReconcile();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnUpdateColorsClicked()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return OnUseSelectedActorClicked();
	}

	ApplyPendingSettingsToActor(true);
	Actor->UpdateDebugColors();
	return FReply::Handled();
}

FReply SAurousTectonicControlPanel::OnExportMapsClicked()
{
	ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return OnUseSelectedActorClicked();
	}

	const int32 ExportWidth = FMath::Max(256, (PendingExportWidth % 2 == 0) ? PendingExportWidth : PendingExportWidth - 1);
	const int32 ExportHeight = ExportWidth / 2;

	TArray<FCanonicalSample> Samples;
	TArray<FDelaunayTriangle> Triangles;
	int64 Timestep = 0;
	if (!Actor->GetPlanetExportDataThreadSafe(Samples, Triangles, Timestep))
	{
		FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("ExportNoPlanetData", "No planet data available. Generate the planet first."));
		return FReply::Handled();
	}

	const bool bCacheValid =
		CachedExportWidth == ExportWidth &&
		CachedExportHeight == ExportHeight &&
		CachedExportSampleCount == Samples.Num() &&
		CachedExportTriangleCount == Triangles.Num() &&
		CachedValidPixelCount > 0 &&
		CachedExportPixelSamples.Num() == ExportWidth * ExportHeight;

	if (!bCacheValid)
	{
		if (!BuildExportPixelCache(ExportWidth, ExportHeight, Samples, Triangles))
		{
			FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("ExportCacheBuildFailed", "Failed to build export sampling cache."));
			return FReply::Handled();
		}
	}

	const FString OutputDirectory = FPaths::Combine(
		FPaths::ProjectSavedDir(),
		TEXT("MapExports"),
		FString::Printf(TEXT("Step_%lld"), Timestep));
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!PlatformFile.CreateDirectoryTree(*OutputDirectory))
	{
		FMessageDialog::Open(EAppMsgType::Ok, FText::Format(
			LOCTEXT("ExportCreateDirFailed", "Failed to create export directory:\n{0}"),
			FText::FromString(OutputDirectory)));
		return FReply::Handled();
	}

	if (!ExportMapsToDirectory(OutputDirectory, ExportWidth, ExportHeight, Samples, Triangles))
	{
		FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("ExportWriteFailed", "Failed to write one or more exported maps. Check Output Log for details."));
		return FReply::Handled();
	}

	UE_LOG(LogTemp, Log, TEXT("Exported tectonic maps to: %s"), *OutputDirectory);
	FMessageDialog::Open(EAppMsgType::Ok, FText::Format(
		LOCTEXT("ExportComplete", "Exported maps to:\n{0}"),
		FText::FromString(OutputDirectory)));

	return FReply::Handled();
}

FText SAurousTectonicControlPanel::GetSelectedActorText() const
{
	const ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return LOCTEXT("NoActorBoundText", "Bound Actor: None");
	}

	return FText::Format(
		LOCTEXT("BoundActorText", "Bound Actor: {0}"),
		FText::FromString(Actor->GetName()));
}

FText SAurousTectonicControlPanel::GetMetricsText() const
{
	const ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return LOCTEXT("NoMetricsText", "Select an ATectonicPlanetActor to view live simulation metrics.");
	}

	const bool bRunning = Actor->IsSimulationThreadRunningThreadSafe();
	FPlanetControlPanelMetricsSnapshot Snapshot;
	Actor->GetControlPanelMetricsSnapshotThreadSafe(Snapshot);

	const double GeologicalAgeMy = static_cast<double>(Snapshot.Timestep) * 2.0;
	const double ReconcileRatio = (Snapshot.Timestep > 0)
		? (static_cast<double>(Snapshot.ReconcileCount) / static_cast<double>(Snapshot.Timestep))
		: 0.0;
	const double BoundaryPct = (Snapshot.SampleCount > 0)
		? (100.0 * static_cast<double>(Snapshot.BoundarySampleCount) / static_cast<double>(Snapshot.SampleCount))
		: 0.0;
	const double BoundaryDeepPct = (Snapshot.BoundarySampleCount > 0)
		? (100.0 * static_cast<double>(Snapshot.BoundaryDeepSampleCount) / static_cast<double>(Snapshot.BoundarySampleCount))
		: 0.0;
	const double LargestContinentalPct = (Snapshot.ContinentalSampleCount > 0)
		? (100.0 * static_cast<double>(Snapshot.LargestContinentalComponentSize) / static_cast<double>(Snapshot.ContinentalSampleCount))
		: 0.0;

	FString Metrics;
	Metrics += FString::Printf(
		TEXT("Simulation State: %s\n")
		TEXT("Reconcile Triggered Last Step: %s\n")
		TEXT("Current Timestep: %lld\n")
		TEXT("Geological Age: %.1f My\n")
		TEXT("Reconciliation Cadence: %d/%lld = %.3f\n"),
		bRunning ? TEXT("Running") : TEXT("Stopped"),
		Snapshot.bReconcileTriggeredLastStep ? TEXT("Yes") : TEXT("No"),
		Snapshot.Timestep,
		GeologicalAgeMy,
		Snapshot.ReconcileCount,
		Snapshot.Timestep,
		ReconcileRatio);

	Metrics += TEXT("\nPost-Generation Stats:\n");
	Metrics += FString::Printf(
		TEXT("  Samples: %d\n")
		TEXT("  Triangles: %d\n")
		TEXT("  Plates: %d\n")
		TEXT("  Boundary Samples: %d (%.2f%%)\n")
		TEXT("  Boundary Mean Depth: %.2f hops\n")
		TEXT("  Boundary Max Depth: %d hops\n")
		TEXT("  Deep Boundary Samples (2+ hops): %d (%.2f%% of boundary)\n")
		TEXT("  Continental Samples: %d\n")
		TEXT("  Continental Components: %d\n")
		TEXT("  Largest Continental Component: %d (%.2f%% of continental)\n")
		TEXT("  Max Non-Gap Plate Components: %d\n")
		TEXT("  Detached Non-Gap Fragment Samples: %d\n")
		TEXT("  Largest Detached Non-Gap Fragment: %d\n")
		TEXT("  Subduction Front Samples: %d\n")
		TEXT("  Andean Samples: %d\n")
		TEXT("  Tracked Terranes: %d\n")
		TEXT("  Active Terranes: %d\n")
		TEXT("  Merged Terranes: %d\n")
		TEXT("  Collision Events: %d\n")
		TEXT("  Himalayan Samples: %d\n")
		TEXT("  Pending Collision Samples: %d\n")
		TEXT("  Max Subduction Distance: %.1f km\n"),
		Snapshot.SampleCount,
		Snapshot.TriangleCount,
		Snapshot.PlateCount,
		Snapshot.BoundarySampleCount,
		BoundaryPct,
		Snapshot.BoundaryMeanDepthHops,
		Snapshot.BoundaryMaxDepthHops,
		Snapshot.BoundaryDeepSampleCount,
		BoundaryDeepPct,
		Snapshot.ContinentalSampleCount,
		Snapshot.ContinentalComponentCount,
		Snapshot.LargestContinentalComponentSize,
		LargestContinentalPct,
		Snapshot.MaxPlateComponentCount,
		Snapshot.DetachedPlateFragmentSampleCount,
		Snapshot.LargestDetachedPlateFragmentSize,
		Snapshot.SubductionFrontSampleCount,
		Snapshot.AndeanSampleCount,
		Snapshot.TrackedTerraneCount,
		Snapshot.ActiveTerraneCount,
		Snapshot.MergedTerraneCount,
		Snapshot.CollisionEventCount,
		Snapshot.HimalayanSampleCount,
		Snapshot.PendingCollisionSampleCount,
		Snapshot.MaxSubductionDistanceKm);

	Metrics += TEXT("\nPer-Plate Sample Counts:\n");
	if (Snapshot.PlateSampleCounts.Num() == 0)
	{
		Metrics += TEXT("  (No plate data available yet)\n");
	}
	else
	{
		for (int32 PlateIndex = 0; PlateIndex < Snapshot.PlateSampleCounts.Num(); ++PlateIndex)
		{
			const int32 PlateSamples = Snapshot.PlateSampleCounts[PlateIndex];
			const double PlatePct = (Snapshot.SampleCount > 0)
				? (100.0 * static_cast<double>(PlateSamples) / static_cast<double>(Snapshot.SampleCount))
				: 0.0;
			Metrics += FString::Printf(
				TEXT("  Plate %d: %d (%.2f%%)%s\n"),
				PlateIndex,
				PlateSamples,
				PlatePct,
				(PlateSamples <= 0) ? TEXT("  [DEGENERATE]") : TEXT(""));
		}
	}

	if (!Snapshot.bHasTimings)
	{
		Metrics += TEXT("\nTiming Breakdown: no reconcile timing snapshot available yet.");
		return FText::FromString(Metrics);
	}

	const FReconcilePhaseTimings& Timings = Snapshot.Timings;
	const int32 TotalPhase2Samples = Timings.Phase2FastPathResolvedSamples + Timings.Phase2FullQuerySamples;
	const double FastPathRate = (TotalPhase2Samples > 0)
		? (100.0 * static_cast<double>(Timings.Phase2FastPathResolvedSamples) / static_cast<double>(TotalPhase2Samples))
		: 0.0;
	const double GapPct = (Snapshot.SampleCount > 0)
		? (100.0 * static_cast<double>(Timings.GapSamples) / static_cast<double>(Snapshot.SampleCount))
		: 0.0;
	const double OverlapPct = (Snapshot.SampleCount > 0)
		? (100.0 * static_cast<double>(Timings.OverlapSamples) / static_cast<double>(Snapshot.SampleCount))
		: 0.0;

	Metrics += TEXT("\nTiming Breakdown (ms):\n");
	Metrics += FString::Printf(TEXT("  P1 Build Spatial: %.3f\n"), Timings.Phase1BuildSpatialMs);
	Metrics += FString::Printf(TEXT("    Soup Extract (sum/max): %.3f / %.3f\n"), Timings.Phase1SoupExtractTotalMs, Timings.Phase1SoupExtractMaxMs);
	Metrics += FString::Printf(TEXT("    BVH Build (sum/max): %.3f / %.3f\n"), Timings.Phase1BVHBuildTotalMs, Timings.Phase1BVHBuildMaxMs);
	Metrics += FString::Printf(TEXT("    Cap Update (sum/max): %.3f / %.3f\n"), Timings.Phase1CapBuildTotalMs, Timings.Phase1CapBuildMaxMs);
	Metrics += FString::Printf(TEXT("  P2 Ownership: %.3f\n"), Timings.Phase2OwnershipMs);
	Metrics += FString::Printf(TEXT("  P3 Interpolation: %.3f\n"), Timings.Phase3InterpolationMs);
	Metrics += FString::Printf(TEXT("  P4 Gap Detection: %.3f\n"), Timings.Phase4GapMs);
	Metrics += FString::Printf(TEXT("  P5 Overlap Detection: %.3f\n"), Timings.Phase5OverlapMs);
	Metrics += FString::Printf(TEXT("  P6 Membership: %.3f\n"), Timings.Phase6MembershipMs);
	Metrics += FString::Printf(TEXT("    Classify Triangles: %.3f\n"), Timings.Phase6ClassifyTrianglesMs);
	Metrics += FString::Printf(TEXT("    Spatial Rebuild: %.3f\n"), Timings.Phase6SpatialRebuildMs);
	Metrics += FString::Printf(TEXT("  P7 Terranes: %.3f\n"), Timings.Phase7TerraneMs);
	Metrics += FString::Printf(TEXT("  P8 Collision: %.3f\n"), Timings.Phase8CollisionMs);
	Metrics += FString::Printf(TEXT("  P8b Refresh: %.3f\n"), Timings.Phase8PostCollisionRefreshMs);
	Metrics += FString::Printf(TEXT("  P9 Subduction: %.3f\n"), Timings.Phase9SubductionMs);
	Metrics += FString::Printf(TEXT("  Total: %.3f\n"), Timings.TotalMs);
	Metrics += FString::Printf(TEXT("\nFast Path: %d / %d (%.2f%%)\n"), Timings.Phase2FastPathResolvedSamples, TotalPhase2Samples, FastPathRate);
	Metrics += FString::Printf(TEXT("Gap Samples: %d (%.2f%%) | Artifact Resolved: %d | Divergent Gaps: %d | Overlap Samples: %d (%.2f%%)\n"), Timings.GapSamples, GapPct, Timings.ArtifactGapResolvedSamples, Timings.DivergentGapSamples, Timings.OverlapSamples, OverlapPct);
	Metrics += FString::Printf(TEXT("Spatial Dirty/Rebuilt Plates: %d / %d"), Timings.Phase6SpatialDirtyPlateCount, Timings.Phase6SpatialRebuiltPlateCount);

	return FText::FromString(Metrics);
}

FText SAurousTectonicControlPanel::GetExportHeightText() const
{
	const int32 Width = FMath::Max(256, (PendingExportWidth % 2 == 0) ? PendingExportWidth : PendingExportWidth - 1);
	const int32 Height = Width / 2;
	return FText::FromString(FString::Printf(TEXT("Height: %d"), Height));
}

FText SAurousTectonicControlPanel::GetSimulationStatusText() const
{
	const ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return LOCTEXT("StatusNoActor", "No Actor");
	}

	if (Actor->IsSimulationThreadRunningThreadSafe())
	{
		return LOCTEXT("StatusRunning", "Running");
	}

	const double NowSeconds = FPlatformTime::Seconds();
	if (LastStepCommandTime > 0.0 && (NowSeconds - LastStepCommandTime) <= 0.35)
	{
		return LOCTEXT("StatusStepping", "Stepping");
	}

	return LOCTEXT("StatusStopped", "Stopped");
}

FSlateColor SAurousTectonicControlPanel::GetSimulationStatusColor() const
{
	const ATectonicPlanetActor* Actor = GetSelectedActor();
	if (!Actor)
	{
		return FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f));
	}

	if (Actor->IsSimulationThreadRunningThreadSafe())
	{
		return FSlateColor(FLinearColor(0.2f, 0.85f, 0.2f));
	}

	const double NowSeconds = FPlatformTime::Seconds();
	if (LastStepCommandTime > 0.0 && (NowSeconds - LastStepCommandTime) <= 0.35)
	{
		return FSlateColor(FLinearColor(0.9f, 0.8f, 0.2f));
	}

	return FSlateColor(FLinearColor(0.7f, 0.7f, 0.7f));
}

EActiveTimerReturnType SAurousTectonicControlPanel::HandleMetricsActiveTimer(double CurrentTime, float DeltaTime)
{
	(void)CurrentTime;
	(void)DeltaTime;

	if (!GetSelectedActor())
	{
		MetricsActiveTimerHandle.Reset();
		return EActiveTimerReturnType::Stop;
	}

	if (MetricsTextBlock.IsValid())
	{
		MetricsTextBlock->Invalidate(EInvalidateWidgetReason::Paint);
	}
	else
	{
		Invalidate(EInvalidateWidgetReason::Paint);
	}

	return EActiveTimerReturnType::Continue;
}

void SAurousTectonicControlPanel::UpdateMetricsActiveTimerRegistration()
{
	const bool bShouldBeActive = GetSelectedActor() != nullptr;
	const TSharedPtr<FActiveTimerHandle> ExistingHandle = MetricsActiveTimerHandle.Pin();
	if (bShouldBeActive)
	{
		if (!ExistingHandle.IsValid())
		{
			MetricsActiveTimerHandle = RegisterActiveTimer(
				0.2f,
				FWidgetActiveTimerDelegate::CreateSP(this, &SAurousTectonicControlPanel::HandleMetricsActiveTimer));
		}
		return;
	}

	if (ExistingHandle.IsValid())
	{
		UnRegisterActiveTimer(ExistingHandle.ToSharedRef());
		MetricsActiveTimerHandle.Reset();
	}
}

int32 SAurousTectonicControlPanel::DebugModeToIndex(uint8 InDebugModeValue) const
{
	switch (static_cast<EPlanetDebugMode>(InDebugModeValue))
	{
	case EPlanetDebugMode::PlateId:
		return 0;
	case EPlanetDebugMode::CrustType:
		return 1;
	case EPlanetDebugMode::Boundary:
		return 2;
	case EPlanetDebugMode::BoundaryType:
		return 3;
	case EPlanetDebugMode::Elevation:
		return 4;
	case EPlanetDebugMode::CrustAge:
		return 5;
	case EPlanetDebugMode::SubductionRole:
		return 6;
	case EPlanetDebugMode::SubductionDistance:
		return 7;
	case EPlanetDebugMode::OrogenyType:
		return 8;
	case EPlanetDebugMode::TerraneId:
		return 9;
	default:
		return 0;
	}
}

uint8 SAurousTectonicControlPanel::IndexToDebugModeValue(int32 InIndex) const
{
	switch (InIndex)
	{
	case 0:
		return static_cast<uint8>(EPlanetDebugMode::PlateId);
	case 1:
		return static_cast<uint8>(EPlanetDebugMode::CrustType);
	case 2:
		return static_cast<uint8>(EPlanetDebugMode::Boundary);
	case 3:
		return static_cast<uint8>(EPlanetDebugMode::BoundaryType);
	case 4:
		return static_cast<uint8>(EPlanetDebugMode::Elevation);
	case 5:
		return static_cast<uint8>(EPlanetDebugMode::CrustAge);
	case 6:
		return static_cast<uint8>(EPlanetDebugMode::SubductionRole);
	case 7:
		return static_cast<uint8>(EPlanetDebugMode::SubductionDistance);
	case 8:
		return static_cast<uint8>(EPlanetDebugMode::OrogenyType);
	case 9:
		return static_cast<uint8>(EPlanetDebugMode::TerraneId);
	default:
		return static_cast<uint8>(EPlanetDebugMode::PlateId);
	}
}

void SAurousTectonicControlPanel::InvalidateExportCache()
{
	CachedExportPixelSamples.Reset();
	CachedExportWidth = 0;
	CachedExportHeight = 0;
	CachedExportSampleCount = 0;
	CachedExportTriangleCount = 0;
	CachedValidPixelCount = 0;
}

bool SAurousTectonicControlPanel::BuildExportPixelCache(
	const int32 Width,
	const int32 Height,
	const TArray<FCanonicalSample>& Samples,
	const TArray<FDelaunayTriangle>& Triangles)
{
	if (Width <= 0 || Height <= 0 || Samples.Num() == 0 || Triangles.Num() == 0)
	{
		return false;
	}

	UE::Geometry::FDynamicMesh3 CanonicalMesh;

	for (int32 SampleIndex = 0; SampleIndex < Samples.Num(); ++SampleIndex)
	{
		const FVector& Position = Samples[SampleIndex].Position;
		CanonicalMesh.AppendVertex(FVector3d(Position.X, Position.Y, Position.Z));
	}

	TArray<int32> DynamicTriangleToGlobal;
	DynamicTriangleToGlobal.Init(INDEX_NONE, Triangles.Num());
	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		const FDelaunayTriangle& Triangle = Triangles[TriangleIndex];
		if (!Samples.IsValidIndex(Triangle.V[0]) || !Samples.IsValidIndex(Triangle.V[1]) || !Samples.IsValidIndex(Triangle.V[2]))
		{
			continue;
		}

		const int32 DynamicTriangleId = CanonicalMesh.AppendTriangle(UE::Geometry::FIndex3i(Triangle.V[0], Triangle.V[1], Triangle.V[2]));
		if (DynamicTriangleId < 0)
		{
			continue;
		}

		if (DynamicTriangleToGlobal.Num() <= DynamicTriangleId)
		{
			DynamicTriangleToGlobal.SetNum(DynamicTriangleId + 1);
		}
		DynamicTriangleToGlobal[DynamicTriangleId] = TriangleIndex;
	}

	if (CanonicalMesh.TriangleCount() <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Map export mesh has zero valid triangles."));
		return false;
	}

	UE::Geometry::FDynamicMeshAABBTree3 BVH(&CanonicalMesh, true);
	BVH.Build();

	CachedExportPixelSamples.SetNumUninitialized(Width * Height);
	int32 ValidPixelCount = 0;
	int32 BarycentricFallbackCount = 0;
	for (int32 Y = 0; Y < Height; ++Y)
	{
		for (int32 X = 0; X < Width; ++X)
		{
			FExportPixelSample PixelSample;
			const int32 PixelIndex = Y * Width + X;
			const FVector3d QueryDirection = PixelToSphereDirection(X, Y, Width, Height);

			double DistanceSquared = TNumericLimits<double>::Max();
			UE::Geometry::IMeshSpatial::FQueryOptions QueryOptions;
			QueryOptions.MaxDistance = 4.0;
			const int32 DynamicTriangleId = BVH.FindNearestTriangle(QueryDirection, DistanceSquared, QueryOptions);
			if (DynamicTriangleId == INDEX_NONE || !DynamicTriangleToGlobal.IsValidIndex(DynamicTriangleId))
			{
				CachedExportPixelSamples[PixelIndex] = PixelSample;
				continue;
			}

			const int32 GlobalTriangleIndex = DynamicTriangleToGlobal[DynamicTriangleId];
			if (!Triangles.IsValidIndex(GlobalTriangleIndex))
			{
				CachedExportPixelSamples[PixelIndex] = PixelSample;
				continue;
			}

			const UE::Geometry::FIndex3i Triangle = CanonicalMesh.GetTriangle(DynamicTriangleId);
			if (!CanonicalMesh.IsVertex(Triangle.A) || !CanonicalMesh.IsVertex(Triangle.B) || !CanonicalMesh.IsVertex(Triangle.C))
			{
				CachedExportPixelSamples[PixelIndex] = PixelSample;
				continue;
			}

			const FVector3d A = CanonicalMesh.GetVertex(Triangle.A);
			const FVector3d B = CanonicalMesh.GetVertex(Triangle.B);
			const FVector3d C = CanonicalMesh.GetVertex(Triangle.C);
			FVector3d Barycentric = ComputePlanarBarycentric(A, B, C, QueryDirection);
			bool bNeedsFallback = false;
			if (Barycentric.X < -1e-4 || Barycentric.Y < -1e-4 || Barycentric.Z < -1e-4)
			{
				bNeedsFallback = true;
			}
			else
			{
				Barycentric.X = FMath::Max(0.0, Barycentric.X);
				Barycentric.Y = FMath::Max(0.0, Barycentric.Y);
				Barycentric.Z = FMath::Max(0.0, Barycentric.Z);
				const double ClampedSum = Barycentric.X + Barycentric.Y + Barycentric.Z;
				if (ClampedSum <= 1e-12)
				{
					bNeedsFallback = true;
				}
				else
				{
					Barycentric /= ClampedSum;
				}
			}

			if (bNeedsFallback)
			{
				// Robust fallback: area-based non-negative weights around the query point.
				const FVector3d C0 = (B - QueryDirection).Cross(C - QueryDirection);
				const FVector3d C1 = (C - QueryDirection).Cross(A - QueryDirection);
				const FVector3d C2 = (A - QueryDirection).Cross(B - QueryDirection);
				const double W0 = FMath::Sqrt(FMath::Max(0.0, C0.SquaredLength()));
				const double W1 = FMath::Sqrt(FMath::Max(0.0, C1.SquaredLength()));
				const double W2 = FMath::Sqrt(FMath::Max(0.0, C2.SquaredLength()));
				const double WSum = W0 + W1 + W2;
				if (WSum <= UE_DOUBLE_SMALL_NUMBER)
				{
					CachedExportPixelSamples[PixelIndex] = PixelSample;
					continue;
				}

				Barycentric = FVector3d(W0 / WSum, W1 / WSum, W2 / WSum);
				++BarycentricFallbackCount;
			}

			uint8 NearestCorner = 0;
			if (Barycentric.Y >= Barycentric.X && Barycentric.Y >= Barycentric.Z)
			{
				NearestCorner = 1;
			}
			else if (Barycentric.Z >= Barycentric.X && Barycentric.Z >= Barycentric.Y)
			{
				NearestCorner = 2;
			}

			PixelSample.TriangleIndex = GlobalTriangleIndex;
			PixelSample.Barycentric = FVector(Barycentric.X, Barycentric.Y, Barycentric.Z);
			PixelSample.NearestVertexCorner = NearestCorner;
			PixelSample.bValid = true;
			CachedExportPixelSamples[PixelIndex] = PixelSample;
			++ValidPixelCount;
		}
	}

	CachedExportWidth = Width;
	CachedExportHeight = Height;
	CachedExportSampleCount = Samples.Num();
	CachedExportTriangleCount = Triangles.Num();
	CachedValidPixelCount = ValidPixelCount;
	UE_LOG(LogTemp, Log, TEXT("Map export cache: %dx%d valid=%d invalid=%d bary_fallback=%d"),
		Width,
		Height,
		CachedValidPixelCount,
		(Width * Height) - CachedValidPixelCount,
		BarycentricFallbackCount);
	return true;
}

bool SAurousTectonicControlPanel::ExportMapsToDirectory(
	const FString& OutputDirectory,
	const int32 Width,
	const int32 Height,
	const TArray<FCanonicalSample>& Samples,
	const TArray<FDelaunayTriangle>& Triangles) const
{
	if (CachedExportPixelSamples.Num() != Width * Height)
	{
		UE_LOG(LogTemp, Warning, TEXT("ExportMapsToDirectory: pixel cache size mismatch."));
		return false;
	}

	double MinElevation = TNumericLimits<double>::Max();
	double MaxElevation = -TNumericLimits<double>::Max();
	for (const FCanonicalSample& Sample : Samples)
	{
		MinElevation = FMath::Min(MinElevation, static_cast<double>(Sample.Elevation));
		MaxElevation = FMath::Max(MaxElevation, static_cast<double>(Sample.Elevation));
	}
	if (MinElevation > MaxElevation)
	{
		MinElevation = -1.0;
		MaxElevation = 1.0;
	}
	const double ElevationRange = FMath::Max(MaxElevation - MinElevation, 1e-6);

	const int32 NumPixels = Width * Height;
	TArray<int32> ResolvedPlateIds;
	TArray<FColor> PlateIdPixels;
	TArray<FColor> CrustTypePixels;
	TArray<FColor> SubductionRolePixels;
	TArray<FColor> OrogenyTypePixels;
	TArray<FColor> TerraneIdPixels;
	TArray<uint16> ElevationPixels;
	TArray<uint16> OwnershipMarginPixels;
	TArray<uint16> DistanceToFrontPixels;
	TArray<uint16> SubductionInfluencePixels;
	TArray<uint16> TrenchInfluencePixels;
	TArray<uint16> CollisionInfluencePixels;
	TArray<uint8> BoundaryMaskPixels;
	TArray<uint8> SubductionFrontMaskPixels;
	TArray<uint8> CollisionFrontMaskPixels;
	TArray<uint8> HimalayanMaskPixels;
	TArray<uint8> GapMaskPixels;
	TArray<uint8> OverlapMaskPixels;

	ResolvedPlateIds.SetNumUninitialized(NumPixels);
	PlateIdPixels.SetNumUninitialized(NumPixels);
	CrustTypePixels.SetNumUninitialized(NumPixels);
	SubductionRolePixels.SetNumUninitialized(NumPixels);
	OrogenyTypePixels.SetNumUninitialized(NumPixels);
	TerraneIdPixels.SetNumUninitialized(NumPixels);
	ElevationPixels.SetNumUninitialized(NumPixels);
	OwnershipMarginPixels.SetNumUninitialized(NumPixels);
	DistanceToFrontPixels.SetNumUninitialized(NumPixels);
	SubductionInfluencePixels.SetNumUninitialized(NumPixels);
	TrenchInfluencePixels.SetNumUninitialized(NumPixels);
	CollisionInfluencePixels.SetNumUninitialized(NumPixels);
	BoundaryMaskPixels.SetNumZeroed(NumPixels);
	SubductionFrontMaskPixels.SetNumUninitialized(NumPixels);
	CollisionFrontMaskPixels.SetNumUninitialized(NumPixels);
	HimalayanMaskPixels.SetNumUninitialized(NumPixels);
	GapMaskPixels.SetNumUninitialized(NumPixels);
	OverlapMaskPixels.SetNumUninitialized(NumPixels);

	for (int32 PixelIndex = 0; PixelIndex < NumPixels; ++PixelIndex)
	{
		const FExportPixelSample& PixelSample = CachedExportPixelSamples[PixelIndex];
		if (!PixelSample.bValid || !Triangles.IsValidIndex(PixelSample.TriangleIndex))
		{
			ResolvedPlateIds[PixelIndex] = INDEX_NONE;
			PlateIdPixels[PixelIndex] = FColor::Black;
			CrustTypePixels[PixelIndex] = FColor::Black;
			ElevationPixels[PixelIndex] = 0;
			OwnershipMarginPixels[PixelIndex] = 0;
			SubductionRolePixels[PixelIndex] = FColor::Black;
			OrogenyTypePixels[PixelIndex] = FColor::Black;
			TerraneIdPixels[PixelIndex] = FColor::Black;
			DistanceToFrontPixels[PixelIndex] = 0;
			SubductionInfluencePixels[PixelIndex] = 0;
			TrenchInfluencePixels[PixelIndex] = 0;
			CollisionInfluencePixels[PixelIndex] = 0;
			SubductionFrontMaskPixels[PixelIndex] = 0;
			CollisionFrontMaskPixels[PixelIndex] = 0;
			HimalayanMaskPixels[PixelIndex] = 0;
			GapMaskPixels[PixelIndex] = 0;
			OverlapMaskPixels[PixelIndex] = 0;
			continue;
		}

		const FDelaunayTriangle& Triangle = Triangles[PixelSample.TriangleIndex];
		if (!Samples.IsValidIndex(Triangle.V[0]) || !Samples.IsValidIndex(Triangle.V[1]) || !Samples.IsValidIndex(Triangle.V[2]))
		{
			ResolvedPlateIds[PixelIndex] = INDEX_NONE;
			PlateIdPixels[PixelIndex] = FColor::Black;
			CrustTypePixels[PixelIndex] = FColor::Black;
			ElevationPixels[PixelIndex] = 0;
			OwnershipMarginPixels[PixelIndex] = 0;
			SubductionRolePixels[PixelIndex] = FColor::Black;
			OrogenyTypePixels[PixelIndex] = FColor::Black;
			TerraneIdPixels[PixelIndex] = FColor::Black;
			DistanceToFrontPixels[PixelIndex] = 0;
			SubductionInfluencePixels[PixelIndex] = 0;
			TrenchInfluencePixels[PixelIndex] = 0;
			CollisionInfluencePixels[PixelIndex] = 0;
			SubductionFrontMaskPixels[PixelIndex] = 0;
			CollisionFrontMaskPixels[PixelIndex] = 0;
			HimalayanMaskPixels[PixelIndex] = 0;
			GapMaskPixels[PixelIndex] = 0;
			OverlapMaskPixels[PixelIndex] = 0;
			continue;
		}

		const FCanonicalSample* CornerSamples[3] = {
			&Samples[Triangle.V[0]],
			&Samples[Triangle.V[1]],
			&Samples[Triangle.V[2]]
		};

		const FVector Barycentric = PixelSample.Barycentric;
		const FVector3d BarycentricD(Barycentric.X, Barycentric.Y, Barycentric.Z);
		const double W0 = Barycentric.X;
		const double W1 = Barycentric.Y;
		const double W2 = Barycentric.Z;

		const double Elevation =
			W0 * static_cast<double>(CornerSamples[0]->Elevation) +
			W1 * static_cast<double>(CornerSamples[1]->Elevation) +
			W2 * static_cast<double>(CornerSamples[2]->Elevation);
		const double ElevationNormalized = FMath::Clamp((Elevation - MinElevation) / ElevationRange, 0.0, 1.0);
		ElevationPixels[PixelIndex] = static_cast<uint16>(FMath::RoundToInt(ElevationNormalized * 65535.0));

		const double Margin =
			W0 * static_cast<double>(CornerSamples[0]->OwnershipMargin) +
			W1 * static_cast<double>(CornerSamples[1]->OwnershipMargin) +
			W2 * static_cast<double>(CornerSamples[2]->OwnershipMargin);
		const double MarginNormalized = FMath::Clamp(Margin, 0.0, 1.0);
		OwnershipMarginPixels[PixelIndex] = static_cast<uint16>(FMath::RoundToInt(MarginNormalized * 65535.0));

		const int32 ResolvedPlateId = TectonicPlanetOwnership::ResolveWeightedPlateOwner(CornerSamples, BarycentricD);
		ResolvedPlateIds[PixelIndex] = ResolvedPlateId;
		PlateIdPixels[PixelIndex] = GetPlateDebugColor(ResolvedPlateId);

		const int32 NearestCorner = FMath::Clamp(static_cast<int32>(PixelSample.NearestVertexCorner), 0, 2);
		const FCanonicalSample& NearestSample = *CornerSamples[NearestCorner];
		const int32 RoleValues[3] = {
			static_cast<int32>(CornerSamples[0]->SubductionRole),
			static_cast<int32>(CornerSamples[1]->SubductionRole),
			static_cast<int32>(CornerSamples[2]->SubductionRole)
		};
		const int32 OrogenyValues[3] = {
			static_cast<int32>(CornerSamples[0]->OrogenyType),
			static_cast<int32>(CornerSamples[1]->OrogenyType),
			static_cast<int32>(CornerSamples[2]->OrogenyType)
		};
		const int32 TerraneValues[3] = {
			CornerSamples[0]->TerraneId,
			CornerSamples[1]->TerraneId,
			CornerSamples[2]->TerraneId
		};
		const int32 FrontValues[3] = {
			CornerSamples[0]->bIsSubductionFront ? 1 : 0,
			CornerSamples[1]->bIsSubductionFront ? 1 : 0,
			CornerSamples[2]->bIsSubductionFront ? 1 : 0
		};
		const int32 CollisionFrontValues[3] = {
			CornerSamples[0]->bIsCollisionFront ? 1 : 0,
			CornerSamples[1]->bIsCollisionFront ? 1 : 0,
			CornerSamples[2]->bIsCollisionFront ? 1 : 0
		};
		const ESubductionRole ResolvedRole = static_cast<ESubductionRole>(ResolveWeightedDominantValueFromInts(RoleValues, BarycentricD));
		const EOrogenyType ResolvedOrogenyType = static_cast<EOrogenyType>(ResolveWeightedDominantValueFromInts(OrogenyValues, BarycentricD));
		const int32 ResolvedTerraneId = ResolveWeightedDominantValueFromInts(TerraneValues, BarycentricD);
		const bool bResolvedFront = ResolveWeightedDominantValueFromInts(FrontValues, BarycentricD) > 0;
		const bool bResolvedCollisionFront = ResolveWeightedDominantValueFromInts(CollisionFrontValues, BarycentricD) > 0;

		double RoleDistance = 0.0;
		double RoleDistanceWeight = 0.0;
		double RoleSpeed = 0.0;
		double RoleSpeedWeight = 0.0;
		for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
		{
			const FCanonicalSample& CornerSample = *CornerSamples[CornerIndex];
			const double CornerWeight = (CornerIndex == 0) ? W0 : ((CornerIndex == 1) ? W1 : W2);
			if (CornerSample.SubductionRole == ResolvedRole && CornerSample.SubductionDistanceKm >= 0.0f)
			{
				RoleDistance += CornerWeight * static_cast<double>(CornerSample.SubductionDistanceKm);
				RoleDistanceWeight += CornerWeight;
				RoleSpeed += CornerWeight * static_cast<double>(CornerSample.SubductionConvergenceSpeedMmPerYear);
				RoleSpeedWeight += CornerWeight;
			}
		}

		const double ResolvedDistanceKm = (RoleDistanceWeight > UE_DOUBLE_SMALL_NUMBER) ? (RoleDistance / RoleDistanceWeight) : -1.0;
		const double ResolvedSpeedMmPerYear = (RoleSpeedWeight > UE_DOUBLE_SMALL_NUMBER) ? (RoleSpeed / RoleSpeedWeight) : 0.0;
		const double DistanceNormalized = (ResolvedDistanceKm >= 0.0)
			? FMath::Clamp(ResolvedDistanceKm / static_cast<double>(ExportSubductionInfluenceRadiusKm), 0.0, 1.0)
			: 0.0;

		double CollisionDistance = 0.0;
		double CollisionDistanceWeight = 0.0;
		double CollisionSpeed = 0.0;
		double CollisionSpeedWeight = 0.0;
		for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
		{
			const FCanonicalSample& CornerSample = *CornerSamples[CornerIndex];
			if (CornerSample.CollisionDistanceKm < 0.0f)
			{
				continue;
			}

			const double CornerWeight = (CornerIndex == 0) ? W0 : ((CornerIndex == 1) ? W1 : W2);
			CollisionDistance += CornerWeight * static_cast<double>(CornerSample.CollisionDistanceKm);
			CollisionDistanceWeight += CornerWeight;
			CollisionSpeed += CornerWeight * static_cast<double>(CornerSample.CollisionConvergenceSpeedMmPerYear);
			CollisionSpeedWeight += CornerWeight;
		}

		const double ResolvedCollisionDistanceKm = (CollisionDistanceWeight > UE_DOUBLE_SMALL_NUMBER) ? (CollisionDistance / CollisionDistanceWeight) : -1.0;
		const double ResolvedCollisionSpeedMmPerYear = (CollisionSpeedWeight > UE_DOUBLE_SMALL_NUMBER) ? (CollisionSpeed / CollisionSpeedWeight) : 0.0;

		CrustTypePixels[PixelIndex] = (NearestSample.CrustType == ECrustType::Continental)
			? FColor(139, 90, 43)
			: FColor(0, 105, 148);
		SubductionRolePixels[PixelIndex] = GetSubductionRoleColor(ResolvedRole, bResolvedFront);
		OrogenyTypePixels[PixelIndex] = GetOrogenyColor(ResolvedOrogenyType);
		TerraneIdPixels[PixelIndex] = GetTerraneDebugColor(ResolvedTerraneId);
		DistanceToFrontPixels[PixelIndex] = static_cast<uint16>(FMath::RoundToInt(DistanceNormalized * 65535.0));
		SubductionInfluencePixels[PixelIndex] = static_cast<uint16>(FMath::RoundToInt(
			((ResolvedRole == ESubductionRole::Overriding) ? ComputeSubductionInfluenceForExport(static_cast<float>(ResolvedDistanceKm), static_cast<float>(ResolvedSpeedMmPerYear)) : 0.0f) * 65535.0f));
		TrenchInfluencePixels[PixelIndex] = static_cast<uint16>(FMath::RoundToInt(
			((ResolvedRole == ESubductionRole::Subducting) ? ComputeTrenchInfluenceForExport(static_cast<float>(ResolvedDistanceKm), static_cast<float>(ResolvedSpeedMmPerYear)) : 0.0f) * 65535.0f));
		CollisionInfluencePixels[PixelIndex] = static_cast<uint16>(FMath::RoundToInt(
			ComputeCollisionInfluenceForExport(static_cast<float>(ResolvedCollisionDistanceKm), static_cast<float>(ResolvedCollisionSpeedMmPerYear)) * 65535.0f));
		SubductionFrontMaskPixels[PixelIndex] = bResolvedFront ? 255 : 0;
		CollisionFrontMaskPixels[PixelIndex] = bResolvedCollisionFront ? 255 : 0;
		HimalayanMaskPixels[PixelIndex] = (ResolvedOrogenyType == EOrogenyType::Himalayan) ? 255 : 0;
		GapMaskPixels[PixelIndex] = (CornerSamples[0]->bGapDetected || CornerSamples[1]->bGapDetected || CornerSamples[2]->bGapDetected) ? 255 : 0;
		OverlapMaskPixels[PixelIndex] = (CornerSamples[0]->bOverlapDetected || CornerSamples[1]->bOverlapDetected || CornerSamples[2]->bOverlapDetected) ? 255 : 0;
	}

	TectonicPlanetOwnership::BuildBoundaryMaskFromResolvedPlateIds(ResolvedPlateIds, Width, Height, BoundaryMaskPixels);
	bool bAllSucceeded = true;
	const auto SaveChecked = [&bAllSucceeded](const bool bSucceeded, const TCHAR* MapName)
	{
		if (!bSucceeded)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to export map: %s"), MapName);
			bAllSucceeded = false;
		}
	};

	SaveChecked(SaveColorPng(FPaths::Combine(OutputDirectory, TEXT("PlateId.png")), Width, Height, PlateIdPixels), TEXT("PlateId"));
	SaveChecked(SaveGray16Png(FPaths::Combine(OutputDirectory, TEXT("Elevation.png")), Width, Height, ElevationPixels), TEXT("Elevation"));
	SaveChecked(SaveGray8Png(FPaths::Combine(OutputDirectory, TEXT("BoundaryMask.png")), Width, Height, BoundaryMaskPixels), TEXT("BoundaryMask"));
	SaveChecked(SaveColorPng(FPaths::Combine(OutputDirectory, TEXT("CrustType.png")), Width, Height, CrustTypePixels), TEXT("CrustType"));
	SaveChecked(SaveGray16Png(FPaths::Combine(OutputDirectory, TEXT("OwnershipMargin.png")), Width, Height, OwnershipMarginPixels), TEXT("OwnershipMargin"));
	SaveChecked(SaveColorPng(FPaths::Combine(OutputDirectory, TEXT("SubductionRole.png")), Width, Height, SubductionRolePixels), TEXT("SubductionRole"));
	SaveChecked(SaveGray8Png(FPaths::Combine(OutputDirectory, TEXT("SubductionFrontMask.png")), Width, Height, SubductionFrontMaskPixels), TEXT("SubductionFrontMask"));
	SaveChecked(SaveGray16Png(FPaths::Combine(OutputDirectory, TEXT("DistanceToFront.png")), Width, Height, DistanceToFrontPixels), TEXT("DistanceToFront"));
	SaveChecked(SaveGray16Png(FPaths::Combine(OutputDirectory, TEXT("SubductionInfluence.png")), Width, Height, SubductionInfluencePixels), TEXT("SubductionInfluence"));
	SaveChecked(SaveGray16Png(FPaths::Combine(OutputDirectory, TEXT("TrenchInfluence.png")), Width, Height, TrenchInfluencePixels), TEXT("TrenchInfluence"));
	SaveChecked(SaveColorPng(FPaths::Combine(OutputDirectory, TEXT("OrogenyType.png")), Width, Height, OrogenyTypePixels), TEXT("OrogenyType"));
	SaveChecked(SaveColorPng(FPaths::Combine(OutputDirectory, TEXT("TerraneId.png")), Width, Height, TerraneIdPixels), TEXT("TerraneId"));
	SaveChecked(SaveGray8Png(FPaths::Combine(OutputDirectory, TEXT("CollisionFrontMask.png")), Width, Height, CollisionFrontMaskPixels), TEXT("CollisionFrontMask"));
	SaveChecked(SaveGray8Png(FPaths::Combine(OutputDirectory, TEXT("HimalayanMask.png")), Width, Height, HimalayanMaskPixels), TEXT("HimalayanMask"));
	SaveChecked(SaveGray16Png(FPaths::Combine(OutputDirectory, TEXT("CollisionInfluence.png")), Width, Height, CollisionInfluencePixels), TEXT("CollisionInfluence"));
	SaveChecked(SaveGray8Png(FPaths::Combine(OutputDirectory, TEXT("GapMask.png")), Width, Height, GapMaskPixels), TEXT("GapMask"));
	SaveChecked(SaveGray8Png(FPaths::Combine(OutputDirectory, TEXT("OverlapMask.png")), Width, Height, OverlapMaskPixels), TEXT("OverlapMask"));

	return bAllSucceeded;
}

#undef LOCTEXT_NAMESPACE

#endif // WITH_EDITOR
