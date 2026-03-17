# Milestone 0 — Codex Prompt Sequence

Three self-contained prompts that implement M0: static foundation for procedural tectonic planets in Aurous (UE5.7 C++). Each prompt gives a coding agent everything it needs to produce correct, compiling code.

---

## Prompt 1: Data Model + Fibonacci Sampling + Spherical Delaunay Triangulation

### Goal

Create the core data structures for a procedural tectonic planet, generate Fibonacci sphere sampling, and build a Spherical Delaunay Triangulation (SDT) using UE5's convex hull API. Also integrate Shewchuk's exact arithmetic predicates as a third-party source file (needed starting in Milestone 1).

### Context

This is an Unreal Engine 5.7 C++ project. The game module is `Aurous` at `Source/Aurous/`. It currently contains only bootstrap code (`Aurous.h`, `Aurous.cpp`, `Aurous.Build.cs`). The module uses explicit PCH (`PCHUsageMode.UseExplicitOrSharedPCHs`) and no Unity builds (`bUseUnity = false`). Follow Unreal/Epic C++ conventions: tabs for indentation, `PascalCase` for types/methods/properties, Unreal prefixes (`F` for structs/value types, `U` for UObjects).

The planet is a fixed set of N sample points on a unit sphere. These points never move or change count. A Spherical Delaunay Triangulation is built once over them and never rebuilt. It provides the neighbor graph, render mesh topology, and source topology for plate triangle soups.

### Files to Create/Modify

1. **`Source/Aurous/Aurous.Build.cs`** — Add `"GeometryCore"` to `PublicDependencyModuleNames`.

2. **`Source/Aurous/Public/TectonicPlanet.h`** — Core data model header.

3. **`Source/Aurous/Private/TectonicPlanet.cpp`** — Fibonacci sampling, SDT construction, neighbor graph extraction.

4. **`Source/Aurous/Private/ShewchukPredicates.h`** — Thin C++ wrapper header for predicates.c.

5. **`Source/Aurous/Private/ThirdParty/predicates.c`** — Shewchuk's exact predicates (public domain). Download from https://www.cs.cmu.edu/~quake/robust.html or use the well-known version. The file must compile as C with no modifications.

### Data Structures

Define these in `TectonicPlanet.h`:

```cpp
// Crust type enumeration
UENUM()
enum class ECrustType : uint8
{
	Oceanic,
	Continental
};

// Orogeny type enumeration
UENUM()
enum class EOrogenyType : uint8
{
	None,
	Andean,
	Himalayan
};

// One sample point on the canonical sphere
struct FCanonicalSample
{
	FVector Position;          // Fixed location on unit sphere (double-precision via FVector)
	int32 PlateId = -1;        // Owning plate (-1 = unassigned)
	int32 PrevPlateId = -1;    // Owner from previous reconciliation
	float OwnershipMargin = 0; // Distance ratio between nearest and second-nearest plate
	bool bIsBoundary = false;  // True if any Delaunay neighbor has different PlateId
	ECrustType CrustType = ECrustType::Continental;
	float Elevation = 0.0f;    // Surface elevation relative to sea level (km)
	float Thickness = 0.0f;    // Crust thickness (km)
	float Age = 0.0f;          // Crust age (My)
	FVector RidgeDirection = FVector::ZeroVector;  // Oceanic only, tangent to sphere
	FVector FoldDirection = FVector::ZeroVector;   // Continental only, tangent to sphere
	EOrogenyType OrogenyType = EOrogenyType::None;
	float OrogenyAge = 0.0f;
	int32 TerraneId = -1;      // Stable terrane identifier (-1 if oceanic)
};

// Triangle from the global SDT (indices into Samples array)
struct FDelaunayTriangle
{
	int32 V[3];  // Vertex indices
};

// Forward declare FPlate (defined after FTectonicPlanet or in same header)
struct FPlate
{
	int32 Id = -1;
	FVector RotationAxis = FVector::UpVector;  // Normalized axis through planet center
	float AngularSpeed = 0.0f;                 // Radians per timestep
	TArray<int32> SampleIndices;               // Indices of canonical samples belonging to this plate
	TArray<int32> InteriorTriangles;           // Indices into global Triangles array (all 3 verts same plate)
	TArray<int32> BoundaryTriangles;           // Indices into global Triangles array (verts span plates)
};

// The planet simulation state
class AUROUS_API FTectonicPlanet
{
public:
	// Initialize with N Fibonacci sphere points and build SDT
	void Initialize(int32 NumSamples = 500000);

	// Accessors
	const TArray<FCanonicalSample>& GetSamples() const { return Samples; }
	const TArray<FDelaunayTriangle>& GetTriangles() const { return Triangles; }
	const TArray<TArray<int32>>& GetAdjacency() const { return Adjacency; }
	const TArray<FPlate>& GetPlates() const { return Plates; }
	TArray<FPlate>& GetPlates() { return Plates; }
	int32 GetNumSamples() const { return Samples.Num(); }

	// Plate initialization (implemented in Prompt 2)
	void InitializePlates(int32 NumPlates = 7, int32 RandomSeed = 42);

	// Triangle classification (implemented in Prompt 2)
	void ClassifyTriangles();

	// Recompute is_boundary flags from Delaunay neighbor graph
	void UpdateBoundaryFlags();

private:
	void GenerateFibonacciSphere(int32 N);
	void BuildDelaunayTriangulation();
	void BuildAdjacencyGraph();

	TArray<FCanonicalSample> Samples;
	TArray<FDelaunayTriangle> Triangles;
	TArray<TArray<int32>> Adjacency;  // Per-vertex neighbor list (Delaunay neighbors)
	TArray<FPlate> Plates;

	// Planet parameters
	static constexpr double PlanetRadius = 6370.0;  // km
};
```

### Implementation Details

**Fibonacci Sphere Sampling** (`GenerateFibonacciSphere`):

Use the golden-ratio Fibonacci spiral on a unit sphere:
```
golden_ratio = (1 + sqrt(5)) / 2
for i = 0 to N-1:
    theta = acos(1 - 2*(i + 0.5) / N)
    phi = 2 * PI * i / golden_ratio
    position = (sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta))
```

Store as `FVector` (which is double-precision in UE5.7). Points live on the unit sphere.

**Spherical Delaunay Triangulation** (`BuildDelaunayTriangulation`):

The 3D convex hull of points on a unit sphere is mathematically equivalent to the Spherical Delaunay Triangulation. Use UE5's `TConvexHull3<double>` from `CompGeom/ConvexHull3.h` in the `GeometryCore` module:

```cpp
#include "CompGeom/ConvexHull3.h"

void FTectonicPlanet::BuildDelaunayTriangulation()
{
    TArray<FVector3d> Points;
    Points.Reserve(Samples.Num());
    for (const FCanonicalSample& S : Samples)
    {
        Points.Add(FVector3d(S.Position));
    }

    UE::Geometry::TConvexHull3<double> Hull;
    // Solve(true) requests convex hull with adjacency/neighbor info
    bool bSuccess = Hull.Solve(Points);
    check(bSuccess);

    // Extract triangles from hull
    const TArray<UE::Geometry::FIndex3i>& HullTriangles = Hull.GetTriangles();
    Triangles.Reserve(HullTriangles.Num());
    for (const UE::Geometry::FIndex3i& Tri : HullTriangles)
    {
        FDelaunayTriangle DT;
        DT.V[0] = Tri.A;
        DT.V[1] = Tri.B;
        DT.V[2] = Tri.C;
        Triangles.Add(DT);
    }

    // Verify: triangle count should be ~2*N - 4 for convex hull of N points
    UE_LOG(LogTemp, Log, TEXT("SDT: %d samples -> %d triangles (expected ~%d)"),
        Samples.Num(), Triangles.Num(), 2 * Samples.Num() - 4);
}
```

**Important:** `TConvexHull3<double>` lives in namespace `UE::Geometry`. The header path is `CompGeom/ConvexHull3.h`. You must include `GeometryCore` in the module's `PublicDependencyModuleNames` in `Aurous.Build.cs`. Verify at compile time that `FIndex3i` has members `A`, `B`, `C` — if the engine uses `V[3]` instead, adapt accordingly.

**Adjacency Graph** (`BuildAdjacencyGraph`):

Build a per-vertex neighbor list from the triangle list. For each triangle, each pair of vertices is adjacent:

```cpp
void FTectonicPlanet::BuildAdjacencyGraph()
{
    Adjacency.SetNum(Samples.Num());
    for (const FDelaunayTriangle& Tri : Triangles)
    {
        auto AddEdge = [this](int32 A, int32 B)
        {
            Adjacency[A].AddUnique(B);
            Adjacency[B].AddUnique(A);
        };
        AddEdge(Tri.V[0], Tri.V[1]);
        AddEdge(Tri.V[1], Tri.V[2]);
        AddEdge(Tri.V[2], Tri.V[0]);
    }
}
```

**Boundary Flag Update** (`UpdateBoundaryFlags`):

```cpp
void FTectonicPlanet::UpdateBoundaryFlags()
{
    for (int32 i = 0; i < Samples.Num(); ++i)
    {
        bool bBoundary = false;
        for (int32 Neighbor : Adjacency[i])
        {
            if (Samples[Neighbor].PlateId != Samples[i].PlateId)
            {
                bBoundary = true;
                break;
            }
        }
        Samples[i].bIsBoundary = bBoundary;
    }
}
```

**`Initialize` method:**

```cpp
void FTectonicPlanet::Initialize(int32 NumSamples)
{
    GenerateFibonacciSphere(NumSamples);
    BuildDelaunayTriangulation();
    BuildAdjacencyGraph();
}
```

**Shewchuk Predicates Integration:**

1. Place the public-domain `predicates.c` file at `Source/Aurous/Private/ThirdParty/predicates.c`. This is a single C file implementing `exactinit()`, `orient2d()`, `orient3d()`, `incircle()`, `insphere()` with adaptive-precision arithmetic.

2. Create `Source/Aurous/Private/ShewchukPredicates.h`:
```cpp
#pragma once

// Wrapper for Shewchuk's exact geometric predicates (predicates.c)
// These are used for robust point-in-triangle containment testing
// in Milestone 1's BVH containment queries.

extern "C"
{
    void exactinit();
    double orient2d(const double* pa, const double* pb, const double* pc);
    double orient3d(const double* pa, const double* pb, const double* pc, const double* pd);
    double incircle(const double* pa, const double* pb, const double* pc, const double* pd);
    double insphere(const double* pa, const double* pb, const double* pc, const double* pd, const double* pe);
}
```

3. In `Aurous.Build.cs`, ensure `predicates.c` is compiled. Since it's in `Private/ThirdParty/`, UE's build system will pick it up if it has a `.c` extension. You may need to add:
```csharp
// Allow C compilation for Shewchuk predicates
bEnableUndefinedIdentifierWarnings = false;
```

Also add `CppStandard = CppStandardVersion.Default;` and ensure the `.c` file compiles. If needed, set the file as an additional source file or wrap it:
```csharp
// In the constructor after existing code:
PrivateDefinitions.Add("ANSI_DECLARATORS");
```

Call `exactinit()` once during `FTectonicPlanet::Initialize()` before any predicate use.

### Acceptance Criteria

1. `Aurous.Build.cs` compiles with `GeometryCore` dependency added
2. `FTectonicPlanet::Initialize(500000)` runs without errors
3. Fibonacci points are uniformly distributed on the unit sphere (all positions have magnitude ~1.0)
4. SDT produces ~999,996 triangles (2N - 4) from 500,000 points
5. Every sample appears in at least one triangle; no degenerate slivers
6. Adjacency graph has an entry for every vertex; typical valence is 6 (range 5–7)
7. `predicates.c` compiles as part of the module; `exactinit()` can be called
8. All code follows UE coding conventions (tabs, PascalCase, F-prefix structs)

---

## Prompt 2: Plate Initialization + Triangle Classification

### Goal

Initialize tectonic plates via spherical Voronoi partitioning from random centroids, assign all canonical samples to plates, classify global SDT triangles as interior vs. boundary, and compute boundary flags.

### Context

This is an Unreal Engine 5.7 C++ project. The `FTectonicPlanet` class (from Prompt 1) already has:
- `Samples` array of `FCanonicalSample` with positions on the unit sphere
- `Triangles` array of `FDelaunayTriangle` (SDT, never changes)
- `Adjacency` graph (per-vertex Delaunay neighbor lists)
- `Plates` array (empty, to be populated)
- Declared but unimplemented: `InitializePlates()`, `ClassifyTriangles()`, `UpdateBoundaryFlags()`
- `UpdateBoundaryFlags()` is already implemented per Prompt 1

This prompt implements `InitializePlates()` and `ClassifyTriangles()` in `Source/Aurous/Private/TectonicPlanet.cpp`.

### Files to Modify

1. **`Source/Aurous/Private/TectonicPlanet.cpp`** — Implement `InitializePlates()` and `ClassifyTriangles()`.

2. **`Source/Aurous/Public/TectonicPlanet.h`** — Add any needed helper declarations (should be minimal).

### Implementation Details

**`InitializePlates(int32 NumPlates, int32 RandomSeed)`:**

1. **Generate random plate centroids** on the unit sphere. Use `FRandomStream` seeded with `RandomSeed` for reproducibility. Generate `NumPlates` centroids (default 7) uniformly on the sphere:
   ```
   z = RandomStream.FRandRange(-1.0, 1.0)
   phi = RandomStream.FRandRange(0.0, 2*PI)
   r = sqrt(1 - z*z)
   centroid = FVector(r*cos(phi), r*sin(phi), z)
   ```

2. **Assign each canonical sample to the nearest plate centroid** using geodesic (great-circle) distance. Since all points are on the unit sphere, geodesic distance = `acos(dot(A, B))`. For comparison purposes, you can use `dot(A, B)` directly (larger dot = smaller distance) to avoid the `acos`:
   ```cpp
   for (int32 i = 0; i < Samples.Num(); ++i)
   {
       double BestDot = -2.0;
       int32 BestPlate = 0;
       for (int32 p = 0; p < NumPlates; ++p)
       {
           double D = FVector::DotProduct(Samples[i].Position, Centroids[p]);
           if (D > BestDot)
           {
               BestDot = D;
               BestPlate = p;
           }
       }
       Samples[i].PlateId = BestPlate;
       Samples[i].PrevPlateId = BestPlate;
   }
   ```

3. **Create plate objects.** For each plate, set:
   - `Id` = plate index (0 to NumPlates-1)
   - `RotationAxis` = random unit vector on sphere (from same `FRandomStream`)
   - `AngularSpeed` = random value in `[0.5, 1.5] * BaseSpeed` where `BaseSpeed` is ~`100 mm/year * 2 My / PlanetRadius` converted to radians per timestep. For M0 this is static, so just store a plausible value. Use `BaseSpeed = (0.1 * 2e6) / (6370e3) ≈ 3.14e-2` radians/timestep.
   - `SampleIndices` = all sample indices with this PlateId (build by scanning Samples)

4. **Assign initial crust types.** Randomly designate 1–3 plates as continental, the rest as oceanic. Set each sample's `CrustType` based on its plate. Set initial elevation: continental = +0.5 km, oceanic = -4.0 km.

**`ClassifyTriangles()`:**

For each triangle in the global SDT, check if all three vertices belong to the same plate:

```cpp
void FTectonicPlanet::ClassifyTriangles()
{
    // Clear existing classification
    for (FPlate& Plate : Plates)
    {
        Plate.InteriorTriangles.Empty();
        Plate.BoundaryTriangles.Empty();
    }

    for (int32 TriIdx = 0; TriIdx < Triangles.Num(); ++TriIdx)
    {
        const FDelaunayTriangle& Tri = Triangles[TriIdx];
        int32 P0 = Samples[Tri.V[0]].PlateId;
        int32 P1 = Samples[Tri.V[1]].PlateId;
        int32 P2 = Samples[Tri.V[2]].PlateId;

        if (P0 == P1 && P1 == P2)
        {
            // Interior triangle: all vertices belong to the same plate
            Plates[P0].InteriorTriangles.Add(TriIdx);
        }
        else
        {
            // Boundary triangle: vertices span multiple plates
            // Add to all involved plates' boundary lists
            Plates[P0].BoundaryTriangles.AddUnique(TriIdx);
            Plates[P1].BoundaryTriangles.AddUnique(TriIdx);
            if (P2 != P0 && P2 != P1)
            {
                Plates[P2].BoundaryTriangles.AddUnique(TriIdx);
            }
        }
    }
}
```

After classification, call `UpdateBoundaryFlags()` to set `bIsBoundary` on each sample.

**Wire up in `InitializePlates`:** At the end of `InitializePlates`, call `ClassifyTriangles()` and `UpdateBoundaryFlags()`.

### Acceptance Criteria

1. `InitializePlates(7, 42)` produces 7 plates with non-empty sample sets
2. Every sample has a valid `PlateId` in range `[0, NumPlates)`
3. The sum of all `Plate.SampleIndices.Num()` equals total sample count
4. `ClassifyTriangles()` assigns every triangle to exactly one plate (interior) or marks it as boundary
5. The sum of interior triangles across all plates + unique boundary triangles = total triangle count
6. `bIsBoundary` is true for samples adjacent to a different-plate sample; false for interior samples
7. 1–3 plates are continental, rest are oceanic; continental samples have ~+0.5 km elevation, oceanic ~-4.0 km
8. Deterministic: same seed produces same plate configuration every time
9. All code follows UE coding conventions

---

## Prompt 3: Planet Actor + RealtimeMesh Rendering + Debug Visualization

### Goal

Create a placeable UE5 actor that generates a tectonic planet on `BeginPlay`, renders it as a runtime mesh using the RealtimeMeshComponent plugin, and supports debug visualization modes (plate coloring, boundary overlay, elevation ramp).

### Context

This is an Unreal Engine 5.7 C++ project with a bundled `RealtimeMeshComponent` plugin (v5.3.2) at `Plugins/RealtimeMeshComponent/`. The `FTectonicPlanet` class (Prompts 1–2) provides `Samples`, `Triangles`, `Adjacency`, and `Plates` data. This prompt creates the actor and rendering layer.

**RealtimeMeshComponent API summary** (verified from plugin source):

- `URealtimeMeshComponent` has `InitializeRealtimeMesh<URealtimeMeshSimple>()` which returns a `URealtimeMeshSimple*`.
- `URealtimeMeshSimple` has static helper: `URealtimeMeshSimple::InitializeRealtimeMeshSimple(URealtimeMeshComponent* Owner)`.
- Mesh data is built via `FRealtimeMeshStreamSet` + `TRealtimeMeshBuilderLocal<IndexType, TangentType, TexCoordType, NumTexCoords>`.
- Section groups are created with `RealtimeMesh->CreateSectionGroup(GroupKey, MoveTemp(StreamSet))`.
- In-place updates via `RealtimeMesh->EditMeshInPlace(GroupKey, [](FRealtimeMeshStreamSet&) -> TSet<FRealtimeMeshStreamKey> { ... })`.
- Key includes: `#include "RealtimeMeshSimple.h"`, `#include "RealtimeMeshComponent.h"`.
- Builder needs `#include "Interface/Core/RealtimeMeshBuilder.h"`.
- Stream keys: `FRealtimeMeshStreams::Position`, `::Tangents`, `::Color`, `::Triangles`.
- Section group key: `FRealtimeMeshSectionGroupKey::Create(0, FName("PlanetMesh"))`.

### Files to Create/Modify

1. **`Source/Aurous/Aurous.Build.cs`** — Add `"RealtimeMeshComponent"` to `PrivateDependencyModuleNames`.

2. **`Source/Aurous/Public/TectonicPlanetActor.h`** — Actor class header.

3. **`Source/Aurous/Private/TectonicPlanetActor.cpp`** — Actor implementation.

### Data Structures

```cpp
// Debug visualization mode
UENUM(BlueprintType)
enum class EPlanetDebugMode : uint8
{
	PlateId       UMETA(DisplayName = "Plate ID"),
	CrustType     UMETA(DisplayName = "Crust Type"),
	Boundary      UMETA(DisplayName = "Boundary"),
	Elevation     UMETA(DisplayName = "Elevation"),
};
```

### Actor Class Design

```cpp
UCLASS()
class AUROUS_API ATectonicPlanetActor : public AActor
{
	GENERATED_BODY()

public:
	ATectonicPlanetActor();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet")
	int32 NumSamples = 500000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet")
	int32 NumPlates = 7;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet")
	int32 RandomSeed = 42;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet")
	float PlanetRenderRadius = 6370.0f;  // Visual radius in UE units (cm by default, but we use km-scale)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Debug")
	EPlanetDebugMode DebugMode = EPlanetDebugMode::PlateId;

	UFUNCTION(BlueprintCallable, Category = "Planet")
	void GeneratePlanet();

	UFUNCTION(BlueprintCallable, Category = "Planet|Debug")
	void UpdateDebugColors();

protected:
	virtual void OnConstruction(const FTransform& Transform) override;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet")
	TObjectPtr<URealtimeMeshComponent> MeshComponent;

private:
	void BuildMesh();
	FColor GetDebugColor(int32 SampleIndex) const;

	FTectonicPlanet Planet;
	FRealtimeMeshSectionGroupKey MeshGroupKey;
};
```

### Implementation Details

**Constructor:**
```cpp
ATectonicPlanetActor::ATectonicPlanetActor()
{
	PrimaryActorTick.bCanEverTick = false;

	MeshComponent = CreateDefaultSubobject<URealtimeMeshComponent>(TEXT("PlanetMesh"));
	SetRootComponent(MeshComponent);
}
```

**`OnConstruction`:**

This fires when the actor is placed in the editor and whenever a `UPROPERTY` changes in the details panel — so the planet regenerates live without hitting Play.

```cpp
void ATectonicPlanetActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	GeneratePlanet();
}
```

**`GeneratePlanet`:**
```cpp
void ATectonicPlanetActor::GeneratePlanet()
{
	Planet.Initialize(NumSamples);
	Planet.InitializePlates(NumPlates, RandomSeed);
	BuildMesh();
}
```

**`BuildMesh`:**

Build the full planet mesh from the SDT. Each vertex position = sample position * PlanetRenderRadius. Normal = sample position (outward on sphere). Color = debug color based on current mode.

```cpp
void ATectonicPlanetActor::BuildMesh()
{
	URealtimeMeshSimple* RealtimeMesh = MeshComponent->InitializeRealtimeMesh<URealtimeMeshSimple>();

	const auto& Samples = Planet.GetSamples();
	const auto& Triangles = Planet.GetTriangles();
	const int32 NumVerts = Samples.Num();
	const int32 NumTris = Triangles.Num();

	// Build stream set using the builder
	// Use uint32 indices since vertex count > 65535
	FRealtimeMeshStreamSet StreamSet;
	TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder(StreamSet);
	Builder.EnableTangents();
	Builder.EnableColors();

	// Reserve space
	Builder.ReserveNumVertices(NumVerts);
	Builder.ReserveNumTriangles(NumTris);

	// Add vertices
	for (int32 i = 0; i < NumVerts; ++i)
	{
		const FCanonicalSample& Sample = Samples[i];
		FVector3f Pos = FVector3f(Sample.Position * PlanetRenderRadius);
		FVector3f Normal = FVector3f(Sample.Position);  // Unit sphere position = outward normal
		FVector3f Tangent = FVector3f(FVector::CrossProduct(FVector::UpVector, Sample.Position));
		if (Tangent.IsNearlyZero())
		{
			Tangent = FVector3f(FVector::CrossProduct(FVector::RightVector, Sample.Position));
		}
		Tangent.Normalize();

		Builder.AddVertex(Pos)
			.SetNormalAndTangent(Normal, Tangent)
			.SetColor(GetDebugColor(i));
	}

	// Add triangles
	for (int32 i = 0; i < NumTris; ++i)
	{
		const FDelaunayTriangle& Tri = Triangles[i];
		Builder.AddTriangle(Tri.V[0], Tri.V[1], Tri.V[2]);
	}

	// Create section group
	MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(0, FName("PlanetSurface"));
	RealtimeMesh->CreateSectionGroup(MeshGroupKey, MoveTemp(StreamSet));

	// Configure section with material slot 0
	FRealtimeMeshSectionKey SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(MeshGroupKey, 0);
	RealtimeMesh->UpdateSectionConfig(SectionKey, FRealtimeMeshSectionConfig(0));
}
```

**`UpdateDebugColors`:**

Update vertex colors in-place without rebuilding the entire mesh:

```cpp
void ATectonicPlanetActor::UpdateDebugColors()
{
	URealtimeMeshSimple* RealtimeMesh = Cast<URealtimeMeshSimple>(MeshComponent->GetRealtimeMesh());
	if (!RealtimeMesh) return;

	const int32 NumVerts = Planet.GetNumSamples();

	RealtimeMesh->EditMeshInPlace(MeshGroupKey,
		[this, NumVerts](FRealtimeMeshStreamSet& Streams) -> TSet<FRealtimeMeshStreamKey>
		{
			FRealtimeMeshStream* ColorStream = Streams.Find(FRealtimeMeshStreams::Color);
			if (!ColorStream || ColorStream->Num() < NumVerts) return {};

			FColor* Colors = ColorStream->GetData<FColor>();
			for (int32 i = 0; i < NumVerts; ++i)
			{
				Colors[i] = GetDebugColor(i);
			}

			return { FRealtimeMeshStreams::Color };
		});
}
```

**`GetDebugColor`:**

```cpp
FColor ATectonicPlanetActor::GetDebugColor(int32 SampleIndex) const
{
	const FCanonicalSample& Sample = Planet.GetSamples()[SampleIndex];

	switch (DebugMode)
	{
	case EPlanetDebugMode::PlateId:
	{
		// Deterministic unique color per plate using golden-ratio hue distribution
		static const FColor PlateColors[] = {
			FColor(230, 25, 75),    // Red
			FColor(60, 180, 75),    // Green
			FColor(255, 225, 25),   // Yellow
			FColor(0, 130, 200),    // Blue
			FColor(245, 130, 48),   // Orange
			FColor(145, 30, 180),   // Purple
			FColor(70, 240, 240),   // Cyan
			FColor(240, 50, 230),   // Magenta
			FColor(210, 245, 60),   // Lime
			FColor(250, 190, 212),  // Pink
			FColor(0, 128, 128),    // Teal
			FColor(220, 190, 255),  // Lavender
		};
		int32 ColorIdx = Sample.PlateId % UE_ARRAY_COUNT(PlateColors);
		return (Sample.PlateId >= 0) ? PlateColors[ColorIdx] : FColor::Black;
	}

	case EPlanetDebugMode::CrustType:
		return (Sample.CrustType == ECrustType::Continental)
			? FColor(139, 90, 43)   // Brown for continental
			: FColor(0, 105, 148);  // Dark blue for oceanic

	case EPlanetDebugMode::Boundary:
		return Sample.bIsBoundary ? FColor::Red : FColor(64, 64, 64);

	case EPlanetDebugMode::Elevation:
	{
		// Color ramp: deep ocean (dark blue) -> sea level (blue/green) -> mountains (white)
		// Range: -10 km to +10 km
		float T = FMath::Clamp((Sample.Elevation + 10.0f) / 20.0f, 0.0f, 1.0f);
		if (T < 0.5f)
		{
			// Ocean: dark blue to cyan
			float OceanT = T / 0.5f;
			return FColor(
				FMath::Lerp(0, 0, OceanT),
				FMath::Lerp(20, 200, OceanT),
				FMath::Lerp(80, 220, OceanT));
		}
		else
		{
			// Land: green to brown to white
			float LandT = (T - 0.5f) / 0.5f;
			if (LandT < 0.5f)
			{
				float SubT = LandT / 0.5f;
				return FColor(
					FMath::Lerp(34, 139, SubT),
					FMath::Lerp(139, 90, SubT),
					FMath::Lerp(34, 43, SubT));
			}
			else
			{
				float SubT = (LandT - 0.5f) / 0.5f;
				return FColor(
					FMath::Lerp(139, 255, SubT),
					FMath::Lerp(90, 255, SubT),
					FMath::Lerp(43, 255, SubT));
			}
		}
	}

	default:
		return FColor::White;
	}
}
```

**Triangle Winding:** The convex hull produces outward-facing triangles when viewed from outside the sphere. If the planet appears inside-out at runtime, reverse the winding order by swapping `V[1]` and `V[2]` when adding triangles to the builder. Detect this by checking if the triangle normal (cross product of two edges) points the same direction as the vertex position (should have positive dot product for outward-facing).

Add winding check in `BuildMesh`:
```cpp
// Before adding triangles, check winding of first triangle
{
    const FDelaunayTriangle& Tri = Triangles[0];
    FVector A = Samples[Tri.V[0]].Position;
    FVector B = Samples[Tri.V[1]].Position;
    FVector C = Samples[Tri.V[2]].Position;
    FVector TriNormal = FVector::CrossProduct(B - A, C - A);
    FVector Center = (A + B + C) / 3.0;
    bool bNeedsFlip = FVector::DotProduct(TriNormal, Center) < 0;
    // Use bNeedsFlip when adding triangles
}
```

### Build System Changes

In `Aurous.Build.cs`, add `"RealtimeMeshComponent"` to `PrivateDependencyModuleNames`:

```csharp
PrivateDependencyModuleNames.AddRange(new string[] { "RealtimeMeshComponent" });
```

### Required Includes

In `TectonicPlanetActor.h`:
```cpp
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "TectonicPlanet.h"
#include "TectonicPlanetActor.generated.h"
```

In `TectonicPlanetActor.cpp`:
```cpp
#include "TectonicPlanetActor.h"
#include "Interface/Core/RealtimeMeshBuilder.h"
#include "Interface/Core/RealtimeMeshDataStream.h"
```

### Material Setup

The actor should work with the default material initially. The mesh uses vertex colors for debug visualization. To see vertex colors, the user should assign a material that reads from the vertex color node. Add a note in code comments:

```cpp
// NOTE: Assign a material that uses VertexColor node to see debug visualization.
// A simple unlit material: BaseColor = VertexColor, Shading Model = Unlit
```

For M0, do not create materials programmatically. The actor should render even without a custom material (just with default lit shading + vertex colors).

### Acceptance Criteria

1. `ATectonicPlanetActor` is placeable in the UE5 editor (appears in actor list)
2. On placement in the editor, the actor generates a planet mesh visible in the viewport immediately — no Play required
3. The mesh is a sphere with ~1M triangles and 500k vertices
4. Vertex colors show plate assignments (each plate a distinct color) in PlateId mode
5. `UpdateDebugColors()` can switch between PlateId, CrustType, Boundary, and Elevation views
6. Changing `NumSamples`, `NumPlates`, `RandomSeed`, or `DebugMode` in the details panel regenerates/updates the mesh live
7. The mesh uses `uint32` indices (vertex count is 500k)
8. Triangle winding is correct (sphere is visible from outside, not inside-out)
9. No crashes or ensure failures during generation or repeated `OnConstruction` calls
10. All code follows UE coding conventions
