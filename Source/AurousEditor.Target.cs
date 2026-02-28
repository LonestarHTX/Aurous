// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class AurousEditorTarget : TargetRules
{
	public AurousEditorTarget( TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V6;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_7;
		bOverrideBuildEnvironment = true;
		ExtraModuleNames.Add("Aurous");

		if (Platform == UnrealTargetPlatform.Win64)
		{
			AdditionalCompilerArguments = (AdditionalCompilerArguments ?? string.Empty) + " /fp:strict";
		}
	}
}
