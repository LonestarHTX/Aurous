// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class AurousTarget : TargetRules
{
	public AurousTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V6;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_7;
		ExtraModuleNames.Add("Aurous");

		if (Platform == UnrealTargetPlatform.Win64)
		{
			AdditionalCompilerArguments = (AdditionalCompilerArguments ?? string.Empty) + " /fp:strict";
		}
	}
}
