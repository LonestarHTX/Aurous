## Repo Notes

### Windows Command Hygiene

- This repo is usually driven from `bash`, but many commands invoke Windows tools. Be explicit about shell boundaries.
- When calling `powershell.exe -NoProfile -Command` from `bash`, wrap the entire PowerShell script in single quotes so `bash` does not expand PowerShell variables like `$_`.
- Prefer `Get-Process -Name UnrealEditor,UnrealEditor-Cmd -ErrorAction SilentlyContinue` for quick Unreal process checks. Use `Get-CimInstance Win32_Process` only when the command line is actually needed.
- When invoking Unreal `Build.bat`, prefer `cmd.exe /c` rather than nesting the batch invocation inside PowerShell.

### Known Good Command Shapes

Check Unreal processes:

```bash
powershell.exe -NoProfile -Command 'Get-Process -Name UnrealEditor,UnrealEditor-Cmd -ErrorAction SilentlyContinue | Format-Table Id,ProcessName -AutoSize'
```

Check Unreal processes with command lines:

```bash
powershell.exe -NoProfile -Command 'Get-CimInstance Win32_Process | Where-Object { $_.Name -in @("UnrealEditor.exe","UnrealEditor-Cmd.exe") } | Select-Object ProcessId,Name,CommandLine | Format-List'
```

Kill a locking Unreal process:

```bash
powershell.exe -NoProfile -Command 'Stop-Process -Id 123456 -Force'
```

Build the editor target and write a log:

```bash
cmd.exe /c ""C:\Program Files\Epic Games\UE_5.7\Engine\Build\BatchFiles\Build.bat" AurousEditor Win64 Development "C:\Users\Michael\Documents\Unreal Projects\Aurous\Aurous.uproject" > "C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs\Build.log" 2>&1"
```

Tail a build log:

```bash
powershell.exe -NoProfile -Command 'Get-Content "C:\Users\Michael\Documents\Unreal Projects\Aurous\Saved\Logs\Build.log" -Tail 80'
```

### Avoid

- Do not pass PowerShell one-liners in double quotes from `bash` when they contain `$_`, `$var`, or subexpressions.
- Do not improvise nested `cmd.exe` + `powershell.exe` + batch quoting when a direct `cmd.exe /c` or single-quoted PowerShell command will do.

### Formatting

- Box-drawing Unicode tables are acceptable when they make dense metrics easier to compare.
- Prefer them for budget, timing, morphology, or gate summaries where row/column alignment matters.
- Do not force boxed tables for casual notes when plain prose or Markdown is clearer.
