// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class Aurous : ModuleRules
{
	public Aurous(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		CppStandard = CppStandardVersion.Default;
		CppCompileWarningSettings.UndefinedIdentifierWarningLevel = WarningLevel.Off;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "GeometryCore", "RealtimeMeshComponent" });

		PrivateDependencyModuleNames.AddRange(new string[] {  });

		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.AddRange(new string[]
			{
				"ImageWrapper",
				"Slate",
				"SlateCore",
				"ToolMenus",
				"UnrealEd",
				"LevelEditor"
			});
		}

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
