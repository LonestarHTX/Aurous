# Follow-Up: Fix Inside-Out Mesh + Create Vertex Color Material

## Problem 1: Mesh is inside-out

The planet sphere is rendering inside-out — visible only from the inside, backface-culled from the outside. This means the triangle winding from `TConvexHull3` produces inward-facing normals when used directly.

### Fix

In `BuildMesh()` in `Source/Aurous/Private/TectonicPlanetActor.cpp`, the winding check exists but either isn't flipping or is flipping the wrong way. Replace the current winding logic with a forced flip — swap V[1] and V[2] for every triangle:

```cpp
// When adding triangles, swap winding order
for (int32 i = 0; i < NumTris; ++i)
{
    const FDelaunayTriangle& Tri = Triangles[i];
    // Swap V[1] and V[2] to flip winding from CW to CCW (or vice versa)
    Builder.AddTriangle(Tri.V[0], Tri.V[2], Tri.V[1]);
}
```

Remove or simplify the existing winding detection check — it clearly got the wrong answer. A hardcoded swap is more reliable here since the hull convention is consistent for all triangles.

Also flip the vertex normals to point outward. They should already be `FVector3f(Sample.Position)` which points outward on a unit sphere — verify this is the case. If normals were negated or computed differently, fix them to:

```cpp
FVector3f Normal = FVector3f(Sample.Position);  // Outward on unit sphere
```

## Problem 2: Planet is flat gray (no vertex colors visible)

The default UE5 material ignores vertex colors entirely — it renders as flat gray lit surface. We need a material that reads vertex colors.

### Fix

Create the material programmatically in the actor constructor or `OnConstruction`. This is a simple unlit material that pipes vertex color to base color:

In `Source/Aurous/Private/TectonicPlanetActor.cpp`, at the end of `BuildMesh()` after creating the section group, apply a default vertex color material if none is assigned:

```cpp
// Create and apply a vertex color material if no override is set
if (!MeshComponent->GetMaterial(0) || MeshComponent->GetMaterial(0) == UMaterial::GetDefaultMaterial(MD_Surface))
{
    static UMaterial* VertexColorMaterial = nullptr;
    if (!VertexColorMaterial)
    {
        VertexColorMaterial = LoadObject<UMaterial>(nullptr,
            TEXT("/Game/Materials/M_VertexColor.M_VertexColor"));
    }
    if (VertexColorMaterial)
    {
        MeshComponent->SetMaterial(0, VertexColorMaterial);
    }
}
```

But this requires a material asset to exist. Since we can't create material assets from C++, instead use a **material instance dynamic** from the engine's built-in vertex color debug material, OR add a `UPROPERTY` for the material so the user can assign it:

**Option A (simpler, recommended):** Add a material property with a clear comment:

```cpp
// In TectonicPlanetActor.h, public section:
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Rendering")
TObjectPtr<UMaterialInterface> PlanetMaterial;
```

Then in `BuildMesh()`, after creating the section group:

```cpp
if (PlanetMaterial)
{
    MeshComponent->SetMaterial(0, PlanetMaterial);
}
```

**Option B (zero manual setup):** Create a dynamic material instance at runtime that shows vertex colors. UE5 has no built-in vertex-color-passthrough material we can reference by path, so create one from code:

```cpp
// In BuildMesh(), after creating the section group:
if (!MeshComponent->GetMaterial(0) || !PlanetMaterial)
{
    UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(
        UMaterial::GetDefaultMaterial(MD_Surface), this);
    MeshComponent->SetMaterial(0, MID);
}
```

This won't show vertex colors either since the default material graph doesn't have a vertex color node.

**The real fix:** Create a material asset. Add a simple Python/editor script or document the manual steps. But for now, the most practical approach is:

1. Add the `PlanetMaterial` UPROPERTY so the user can assign a material from the details panel.
2. Also create the material asset via an editor utility script that runs once. Add this file:

**`Source/Aurous/Private/TectonicPlanetActor.cpp`** — in `BuildMesh()`, after section creation, generate a minimal vertex color material if one doesn't exist:

```cpp
#include "Materials/MaterialInstanceDynamic.h"

// At the end of BuildMesh():
// Apply user-assigned material, or fall back to a basic unlit vertex color material
if (PlanetMaterial)
{
    MeshComponent->SetMaterial(0, PlanetMaterial);
}
else
{
    // Auto-create a basic material that shows vertex colors
    // The user can override this by setting PlanetMaterial in the details panel
    UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(
        LoadObject<UMaterial>(nullptr, TEXT("/Engine/EngineMaterials/DefaultMaterial.DefaultMaterial")),
        this);
    MeshComponent->SetMaterial(0, MID);
    UE_LOG(LogTemp, Warning, TEXT("ATectonicPlanetActor: No PlanetMaterial assigned. Vertex colors won't be visible. "
        "Create an unlit material with VertexColor -> BaseColor and assign it to PlanetMaterial."));
}
```

**Additionally, create the material asset** by adding a one-time editor utility blueprint or Python script at `Content/Python/create_vertex_color_material.py`:

```python
import unreal

# Create a simple unlit material that displays vertex colors
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
mat_factory = unreal.MaterialFactoryNew()

mat = asset_tools.create_asset("M_VertexColor", "/Game/Materials", unreal.Material, mat_factory)

# Can't fully wire material graph from Python — user must open it and:
# 1. Add a VertexColor node
# 2. Connect its RGB output to Base Color
# 3. Set Shading Model to Unlit
# 4. Save

unreal.log("Created /Game/Materials/M_VertexColor — open in Material Editor to wire VertexColor -> BaseColor, set Unlit")
```

**Alternatively, skip the Python script** and just document it clearly. The cleanest solution for now:

1. Add `PlanetMaterial` UPROPERTY to the actor
2. Log a warning if it's null
3. Document the 30-second manual material creation in a code comment

### Recommended implementation

Do all of the following:

1. Add `PlanetMaterial` UPROPERTY (EditAnywhere) to `ATectonicPlanetActor`
2. Apply it in `BuildMesh()` if set
3. Log a warning if null explaining what material to create
4. Add the `PlanetMaterial` property to the `PostEditChangeProperty` list that triggers full rebuild

## Build and Verify

Rebuild:
```
Build.bat AurousEditor Win64 Development -Project="<path>\Aurous.uproject" -WaitMutex
```

Run existing tests to confirm nothing broke:
```
UnrealEditor-Cmd.exe "<path>\Aurous.uproject" -unattended -nopause -nosplash -nullrhi -nosound -nop4 -ExecCmds="Automation RunTests Aurous.TectonicPlanet" -TestExit="Automation Test Queue Empty" -log
```

## Acceptance Criteria

1. The planet sphere is visible from outside (not inside-out) — winding is correct
2. A `PlanetMaterial` UPROPERTY exists on the actor for assigning a vertex color material
3. If `PlanetMaterial` is null, a log warning explains what to create
4. When a proper vertex color material is assigned, plate colors are visible
5. All 6 existing automation tests still pass
6. No crashes on repeated OnConstruction calls
