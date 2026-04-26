@echo off
"C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" "C:\Users\Michael\Documents\Unreal Projects\Aurous\Aurous.uproject" -unattended -nopause -nosplash -nullrhi -nosound -nop4 -ExecCmds="Automation RunTests Aurous.TectonicPlanet.V6V9Step200ThesisSurgeValidationTest" -TestExit="Automation Test Queue Empty" -log
