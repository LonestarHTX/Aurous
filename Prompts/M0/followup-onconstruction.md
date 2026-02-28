# Follow-Up: OnConstruction + Debug Mode Live Switching

## Goal

The `ATectonicPlanetActor` currently generates the planet in `BeginPlay`, requiring Play mode to see anything. Change it to use `OnConstruction` so the planet renders immediately when placed in the editor, and regenerates live when properties change in the details panel.

## Changes Required

### 1. Replace BeginPlay with OnConstruction

In `Source/Aurous/Public/TectonicPlanetActor.h`, change the protected override declaration:

```cpp
// Remove this:
virtual void BeginPlay() override;

// Add this:
virtual void OnConstruction(const FTransform& Transform) override;
```

In `Source/Aurous/Private/TectonicPlanetActor.cpp`, replace the implementation:

```cpp
// Remove BeginPlay entirely. Replace with:
void ATectonicPlanetActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	GeneratePlanet();
}
```

This fires when:
- The actor is placed in the editor
- Any `UPROPERTY` is modified in the details panel
- The level loads

So the planet mesh appears immediately on placement, and changing `NumSamples`, `NumPlates`, `RandomSeed`, or `DebugMode` in the details panel regenerates it live without hitting Play.

### 2. Make OnConstruction safe to call repeatedly

`GeneratePlanet()` calls `BuildMesh()` which calls `InitializeRealtimeMesh<URealtimeMeshSimple>()`. Verify that calling this repeatedly doesn't leak or crash — it should reset the mesh each time, which is correct since we're rebuilding from scratch. If `InitializeRealtimeMesh` asserts on a second call, guard it:

```cpp
URealtimeMeshSimple* RealtimeMesh = Cast<URealtimeMeshSimple>(MeshComponent->GetRealtimeMesh());
if (!RealtimeMesh)
{
	RealtimeMesh = MeshComponent->InitializeRealtimeMesh<URealtimeMeshSimple>();
}
```

Then clear the existing section group before creating the new one if needed.

### 3. Make DebugMode changes lightweight

Right now changing `DebugMode` triggers full `OnConstruction` → `GeneratePlanet()` which rebuilds the entire 500k-vertex mesh from scratch. That's ~2.4 seconds of initialization for what should be a color-only update.

Add a `PostEditChangeProperty` override that detects `DebugMode` changes and calls `UpdateDebugColors()` instead of regenerating:

```cpp
// In TectonicPlanetActor.h, protected section:
#if WITH_EDITOR
virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

// In TectonicPlanetActor.cpp:
#if WITH_EDITOR
void ATectonicPlanetActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	const FName PropertyName = PropertyChangedEvent.GetPropertyName();

	if (PropertyName == GET_MEMBER_NAME_CHECKED(ATectonicPlanetActor, DebugMode))
	{
		// Color-only update, skip full regeneration
		UpdateDebugColors();
		Super::PostEditChangeProperty(PropertyChangedEvent);
		return;
	}

	// For all other properties (NumSamples, NumPlates, RandomSeed, etc.)
	// let OnConstruction handle full rebuild
	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif
```

This way `DebugMode` switches are instant, while structural changes (sample count, plate count, seed) do the full rebuild.

### 4. Build and verify

Rebuild and confirm:

```
Build.bat AurousEditor Win64 Development -Project="<path>\Aurous.uproject" -WaitMutex
```

The existing 6 automation tests should still pass — this change doesn't affect the data model. Run them to confirm nothing broke:

```
UnrealEditor-Cmd.exe "<path>\Aurous.uproject" -unattended -nopause -nosplash -nullrhi -nosound -nop4 -ExecCmds="Automation RunTests Aurous.TectonicPlanet" -TestExit="Automation Test Queue Empty" -log
```

## Acceptance Criteria

1. Placing `ATectonicPlanetActor` in the editor shows the planet mesh immediately — no Play required
2. Changing `NumSamples`, `NumPlates`, or `RandomSeed` in the details panel regenerates the mesh
3. Changing `DebugMode` switches vertex colors instantly without a full rebuild
4. No crashes or asserts when properties are changed repeatedly
5. All 6 existing automation tests still pass at 500k
6. `BeginPlay` is no longer used for planet generation (remove it or leave it empty)
