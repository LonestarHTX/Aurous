# Repository Guidelines

## Project Structure & Module Organization
This repository is an Unreal Engine 5.7 project (`Aurous.uproject`) with one game module and a bundled plugin.

- `Source/Aurous/`: primary C++ game module (`Aurous.Build.cs`, target entrypoints, module bootstrap).
- `Config/`: project settings (`DefaultEngine.ini`, `DefaultInput.ini`, etc.).
- `Content/`: Unreal assets and maps (binary `.uasset` content).
- `Plugins/RealtimeMeshComponent/`: third-party/runtime plugin source, editor modules, and plugin tests.
- Generated directories (`Binaries/`, `Intermediate/`, `Saved/`, `DerivedDataCache/`) are build/runtime artifacts and should generally not be edited manually.

## Build, Test, and Development Commands
Use Unreal Engine 5.7 (per `EngineAssociation`).

- Open in editor: launch `Aurous.uproject` from Epic Launcher or Explorer.
- Build editor target (Windows example):
  `Engine/Build/BatchFiles/Build.bat AurousEditor Win64 Development -Project="...\Aurous.uproject" -WaitMutex`
- Build from Visual Studio: open `Aurous.sln`, build `AurousEditor` in `Development Editor | Win64`.
- Run automated tests (headless example):
  `UnrealEditor-Cmd.exe ".../Aurous.uproject" -unattended -nop4 -ExecCmds="Automation RunTests RealtimeMesh; Quit"`

## Coding Style & Naming Conventions
Follow Unreal/Epic C++ conventions.

- Use tabs/UE-style formatting consistent with existing files.
- Class/type names use Unreal prefixes (`A`, `U`, `F`, `I`) and `PascalCase`.
- Methods/properties use `PascalCase`; local variables are descriptive and concise.
- Keep module names aligned across files (`Aurous.Build.cs`, `Aurous.Target.cs`).
- Prefer config changes in `Config/Default*.ini` over hardcoded values.

## Testing Guidelines
This project currently includes plugin-focused automation/functional tests in `Plugins/RealtimeMeshComponent/Source/RealtimeMeshTests` and `.../RealtimeMeshProTests`.

- No coverage threshold is defined in this workspace snapshot.
- Name tests by feature + behavior (examples already present: `FRealtimeMeshStreamKeyConstructorTest`, `ARealtimeMeshProTest_*`).
- For new game-module tests, place code under a dedicated `Tests` area in `Source/Aurous/Private/`.

## Commit & Pull Request Guidelines
No `.git` metadata is present in this workspace copy, so commit conventions cannot be inferred from local history.

- Use short, imperative commit subjects (example: `Add tectonic planet mesh generator actor`).
- PRs should include purpose, test evidence (editor/manual/automation), and screenshots for `Content/` or visual changes.
- Link related issues/tasks and note any engine/plugin version assumptions (UE 5.7, RealtimeMeshComponent).
