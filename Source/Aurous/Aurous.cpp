// Copyright Epic Games, Inc. All Rights Reserved.

#include "Aurous.h"
#include "Modules/ModuleManager.h"
#include "ShewchukPredicates.h"

#if WITH_EDITOR
#include "Editor/SAurousTectonicControlPanel.h"
#include "Framework/Docking/TabManager.h"
#include "ToolMenus.h"
#include "Widgets/Docking/SDockTab.h"
#endif

#define LOCTEXT_NAMESPACE "FAurousModule"

class FAurousModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		exactinit();

#if WITH_EDITOR
		FGlobalTabmanager::Get()->RegisterNomadTabSpawner(
			TectonicControlPanelTabName,
			FOnSpawnTab::CreateRaw(this, &FAurousModule::SpawnTectonicControlPanelTab))
			.SetDisplayName(LOCTEXT("TectonicControlPanelTabTitle", "Tectonic Control Panel"))
			.SetTooltipText(LOCTEXT("TectonicControlPanelTabTooltip", "Control ATectonicPlanetActor instances in the editor."))
			.SetMenuType(ETabSpawnerMenuType::Hidden);

		UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FAurousModule::RegisterMenus));
#endif
	}

	virtual void ShutdownModule() override
	{
#if WITH_EDITOR
		if (UToolMenus::TryGet())
		{
			UToolMenus::UnRegisterStartupCallback(this);
			UToolMenus::UnregisterOwner(this);
		}

		if (FGlobalTabmanager::Get()->HasTabSpawner(TectonicControlPanelTabName))
		{
			FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(TectonicControlPanelTabName);
		}
#endif
	}

#if WITH_EDITOR
private:
	static const FName TectonicControlPanelTabName;

	void RegisterMenus()
	{
		FToolMenuOwnerScoped OwnerScoped(this);

		UToolMenu* WindowMenu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Window");
		FToolMenuSection& Section = WindowMenu->FindOrAddSection("Aurous");
		Section.AddMenuEntry(
			"OpenAurousTectonicControlPanel",
			LOCTEXT("OpenTectonicControlPanelLabel", "Tectonic Control Panel"),
			LOCTEXT("OpenTectonicControlPanelTooltip", "Open the Aurous tectonic control panel."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateRaw(this, &FAurousModule::OpenTectonicControlPanelTab)));
	}

	void OpenTectonicControlPanelTab()
	{
		FGlobalTabmanager::Get()->TryInvokeTab(TectonicControlPanelTabName);
	}

	TSharedRef<SDockTab> SpawnTectonicControlPanelTab(const FSpawnTabArgs& SpawnTabArgs)
	{
		return SNew(SDockTab)
			.TabRole(ETabRole::NomadTab)
			[
				SNew(SAurousTectonicControlPanel)
			];
	}
#endif
};

#if WITH_EDITOR
const FName FAurousModule::TectonicControlPanelTabName(TEXT("AurousTectonicControlPanel"));
#endif

IMPLEMENT_PRIMARY_GAME_MODULE(FAurousModule, Aurous, "Aurous");

#undef LOCTEXT_NAMESPACE
