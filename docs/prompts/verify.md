# Verification Prompt: Build and Validate Milestone 0

## Goal

Compile the Aurous project headless and run automated tests to verify every Milestone 0 component works correctly. Do NOT skip this step — the code must compile and pass all tests before you are done.

## Build

Build the project using the Unreal Engine headless build pipeline. Find the engine installation via the `EngineAssociation` field in `Aurous.uproject` (it targets UE 5.7). Use UnrealBuildTool:

```bash
# Linux
<EnginePath>/Engine/Build/BatchFiles/Linux/Build.sh AurousEditor Linux Development -Project="<ProjectPath>/Aurous.uproject" -WaitMutex

# Windows
<EnginePath>/Engine/Build/BatchFiles/Build.bat AurousEditor Win64 Development -Project="<ProjectPath>/Aurous.uproject" -WaitMutex
```

Fix any compilation errors until the build succeeds with zero errors. Common issues to watch for:

- `TConvexHull3` may be in namespace `UE::Geometry` — check the actual header `CompGeom/ConvexHull3.h`
- `FIndex3i` members may be `A/B/C` or `V[3]` depending on engine version — verify and adapt
- `FVector` is double-precision in UE5.4+ — make sure `FVector3d` conversions are correct
- `predicates.c` may need `REAL` defined as `double` and `ANSI_DECLARATORS` defined — check compiler warnings
- `RealtimeMeshComponent` module name in Build.cs must match the plugin's `.uplugin` `"Name"` field exactly
- Missing includes: `#include "Interface/Core/RealtimeMeshBuilder.h"` is needed for `TRealtimeMeshBuilderLocal`

## Write Automated Tests

Create the file `Source/Aurous/Private/Tests/TectonicPlanetTests.cpp` with UE5 automation tests. Use the `IMPLEMENT_SIMPLE_AUTOMATION_TEST` macro with the `EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter` flags.

Write these test cases:

### Test 1: Fibonacci Sphere Quality
```cpp
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFibonacciSphereTest, "Aurous.TectonicPlanet.FibonacciSphere",
    EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFibonacciSphereTest::RunTest(const FString& Parameters)
{
    FTectonicPlanet Planet;
    Planet.Initialize(500000);

    const auto& Samples = Planet.GetSamples();

    // Correct count
    TestEqual(TEXT("Sample count"), Samples.Num(), 500000);

    // All points on unit sphere (magnitude ~1.0)
    for (int32 i = 0; i < Samples.Num(); ++i)
    {
        double Mag = Samples[i].Position.Size();
        TestTrue(FString::Printf(TEXT("Sample %d on unit sphere (mag=%f)"), i, Mag),
            FMath::IsNearlyEqual(Mag, 1.0, 1e-10));
    }

    // No duplicate points (check first 1000 pairs as a sanity check)
    for (int32 i = 0; i < FMath::Min(1000, Samples.Num()); ++i)
    {
        for (int32 j = i + 1; j < FMath::Min(i + 10, Samples.Num()); ++j)
        {
            double Dist = FVector::Dist(Samples[i].Position, Samples[j].Position);
            TestTrue(FString::Printf(TEXT("Samples %d and %d not coincident"), i, j),
                Dist > 1e-8);
        }
    }

    return true;
}
```

### Test 2: Delaunay Triangulation Validity
```cpp
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDelaunayTriangulationTest, "Aurous.TectonicPlanet.DelaunayTriangulation",
    EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FDelaunayTriangulationTest::RunTest(const FString& Parameters)
{
    FTectonicPlanet Planet;
    Planet.Initialize(500000);

    const auto& Samples = Planet.GetSamples();
    const auto& Triangles = Planet.GetTriangles();

    // Euler formula for convex hull: F = 2V - 4
    int32 ExpectedTris = 2 * Samples.Num() - 4;
    TestEqual(TEXT("Triangle count matches Euler formula"), Triangles.Num(), ExpectedTris);

    // All triangle indices valid
    for (int32 i = 0; i < Triangles.Num(); ++i)
    {
        for (int32 v = 0; v < 3; ++v)
        {
            TestTrue(FString::Printf(TEXT("Tri %d V[%d] in range"), i, v),
                Triangles[i].V[v] >= 0 && Triangles[i].V[v] < Samples.Num());
        }
        // No degenerate triangles
        TestTrue(FString::Printf(TEXT("Tri %d non-degenerate"), i),
            Triangles[i].V[0] != Triangles[i].V[1] &&
            Triangles[i].V[1] != Triangles[i].V[2] &&
            Triangles[i].V[0] != Triangles[i].V[2]);
    }

    // Every vertex appears in at least one triangle
    TSet<int32> ReferencedVerts;
    for (const auto& Tri : Triangles)
    {
        ReferencedVerts.Add(Tri.V[0]);
        ReferencedVerts.Add(Tri.V[1]);
        ReferencedVerts.Add(Tri.V[2]);
    }
    TestEqual(TEXT("All vertices referenced"), ReferencedVerts.Num(), Samples.Num());

    return true;
}
```

### Test 3: Adjacency Graph Properties
```cpp
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAdjacencyGraphTest, "Aurous.TectonicPlanet.AdjacencyGraph",
    EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FAdjacencyGraphTest::RunTest(const FString& Parameters)
{
    FTectonicPlanet Planet;
    Planet.Initialize(500000);

    const auto& Adjacency = Planet.GetAdjacency();
    const auto& Samples = Planet.GetSamples();

    // Every vertex has an adjacency entry
    TestEqual(TEXT("Adjacency array size"), Adjacency.Num(), Samples.Num());

    // Check valence distribution — Fibonacci sphere SDT should be mostly valence 6
    int32 Val5 = 0, Val6 = 0, Val7 = 0, ValOther = 0;
    for (int32 i = 0; i < Adjacency.Num(); ++i)
    {
        int32 Valence = Adjacency[i].Num();
        TestTrue(FString::Printf(TEXT("Vertex %d has neighbors (valence=%d)"), i, Valence),
            Valence >= 3);

        if (Valence == 5) Val5++;
        else if (Valence == 6) Val6++;
        else if (Valence == 7) Val7++;
        else ValOther++;
    }

    // Vast majority should be valence 5-7
    float PctNormal = (float)(Val5 + Val6 + Val7) / Adjacency.Num();
    TestTrue(FString::Printf(TEXT("%.1f%% vertices have valence 5-7"), PctNormal * 100),
        PctNormal > 0.95f);

    UE_LOG(LogTemp, Log, TEXT("Adjacency valence distribution: V5=%d V6=%d V7=%d Other=%d"),
        Val5, Val6, Val7, ValOther);

    // Symmetry check: if A is neighbor of B, B must be neighbor of A
    for (int32 i = 0; i < FMath::Min(1000, Adjacency.Num()); ++i)
    {
        for (int32 Neighbor : Adjacency[i])
        {
            TestTrue(FString::Printf(TEXT("Adjacency symmetry %d<->%d"), i, Neighbor),
                Adjacency[Neighbor].Contains(i));
        }
    }

    return true;
}
```

### Test 4: Plate Initialization
```cpp
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlateInitTest, "Aurous.TectonicPlanet.PlateInitialization",
    EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FPlateInitTest::RunTest(const FString& Parameters)
{
    FTectonicPlanet Planet;
    Planet.Initialize(500000);
    Planet.InitializePlates(7, 42);

    const auto& Samples = Planet.GetSamples();
    const auto& Plates = Planet.GetPlates();

    // Correct number of plates
    TestEqual(TEXT("Plate count"), Plates.Num(), 7);

    // Every sample assigned to a valid plate
    for (int32 i = 0; i < Samples.Num(); ++i)
    {
        TestTrue(FString::Printf(TEXT("Sample %d has valid PlateId"), i),
            Samples[i].PlateId >= 0 && Samples[i].PlateId < 7);
    }

    // Sum of plate sample counts equals total
    int32 TotalAssigned = 0;
    for (const auto& Plate : Plates)
    {
        TestTrue(FString::Printf(TEXT("Plate %d non-empty"), Plate.Id),
            Plate.SampleIndices.Num() > 0);
        TotalAssigned += Plate.SampleIndices.Num();
    }
    TestEqual(TEXT("All samples assigned to plates"), TotalAssigned, Samples.Num());

    // At least one continental and one oceanic plate
    int32 NumContinental = 0;
    for (const auto& Sample : Samples)
    {
        if (Sample.CrustType == ECrustType::Continental)
        {
            NumContinental++;
            break;
        }
    }
    TestTrue(TEXT("Has continental crust"), NumContinental > 0);

    // Determinism: same seed produces same result
    FTectonicPlanet Planet2;
    Planet2.Initialize(10000);
    Planet2.InitializePlates(7, 42);
    for (int32 i = 0; i < Samples.Num(); ++i)
    {
        TestEqual(FString::Printf(TEXT("Determinism sample %d"), i),
            Planet2.GetSamples()[i].PlateId, Samples[i].PlateId);
    }

    return true;
}
```

### Test 5: Triangle Classification
```cpp
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTriangleClassificationTest, "Aurous.TectonicPlanet.TriangleClassification",
    EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FTriangleClassificationTest::RunTest(const FString& Parameters)
{
    FTectonicPlanet Planet;
    Planet.Initialize(500000);
    Planet.InitializePlates(7, 42);

    const auto& Samples = Planet.GetSamples();
    const auto& Triangles = Planet.GetTriangles();
    const auto& Plates = Planet.GetPlates();

    // Count interior triangles across all plates
    int32 TotalInterior = 0;
    TSet<int32> BoundarySet;
    for (const auto& Plate : Plates)
    {
        TotalInterior += Plate.InteriorTriangles.Num();

        // Verify all interior triangles actually have all 3 verts on this plate
        for (int32 TriIdx : Plate.InteriorTriangles)
        {
            const auto& Tri = Triangles[TriIdx];
            TestTrue(FString::Printf(TEXT("Interior tri %d: all verts on plate %d"), TriIdx, Plate.Id),
                Samples[Tri.V[0]].PlateId == Plate.Id &&
                Samples[Tri.V[1]].PlateId == Plate.Id &&
                Samples[Tri.V[2]].PlateId == Plate.Id);
        }

        for (int32 TriIdx : Plate.BoundaryTriangles)
        {
            BoundarySet.Add(TriIdx);
        }
    }

    // Interior + unique boundary = total
    TestEqual(TEXT("Interior + boundary = total triangles"),
        TotalInterior + BoundarySet.Num(), Triangles.Num());

    // Boundary triangles actually span plates
    for (int32 TriIdx : BoundarySet)
    {
        const auto& Tri = Triangles[TriIdx];
        int32 P0 = Samples[Tri.V[0]].PlateId;
        int32 P1 = Samples[Tri.V[1]].PlateId;
        int32 P2 = Samples[Tri.V[2]].PlateId;
        TestTrue(FString::Printf(TEXT("Boundary tri %d spans plates"), TriIdx),
            P0 != P1 || P1 != P2 || P0 != P2);
    }

    // Boundary flags consistent
    for (int32 i = 0; i < Samples.Num(); ++i)
    {
        bool bHasDifferentNeighbor = false;
        for (int32 Neighbor : Planet.GetAdjacency()[i])
        {
            if (Samples[Neighbor].PlateId != Samples[i].PlateId)
            {
                bHasDifferentNeighbor = true;
                break;
            }
        }
        TestEqual(FString::Printf(TEXT("Sample %d boundary flag"), i),
            Samples[i].bIsBoundary, bHasDifferentNeighbor);
    }

    return true;
}
```

### Test 6: Shewchuk Predicates Linkage
```cpp
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FShewchukPredicatesTest, "Aurous.TectonicPlanet.ShewchukPredicates",
    EAutomationTestFlags::ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FShewchukPredicatesTest::RunTest(const FString& Parameters)
{
    // Verify exactinit can be called
    exactinit();

    // orient3d: four points forming a known orientation
    // Origin, +X, +Y, +Z should give positive orient3d
    double O[3] = {0, 0, 0};
    double A[3] = {1, 0, 0};
    double B[3] = {0, 1, 0};
    double C[3] = {0, 0, 1};

    double Result = orient3d(O, A, B, C);
    TestTrue(TEXT("orient3d positive for right-hand tetrahedron"), Result > 0);

    // Flip one pair -> negative
    double ResultFlipped = orient3d(O, B, A, C);
    TestTrue(TEXT("orient3d negative when flipped"), ResultFlipped < 0);

    // Coplanar points -> zero
    double D[3] = {1, 1, 0};
    double ResultCoplanar = orient3d(O, A, B, D);
    TestTrue(TEXT("orient3d zero for coplanar"), FMath::IsNearlyZero(ResultCoplanar, 1e-15));

    return true;
}
```

Add `#include "ShewchukPredicates.h"` at the top of the test file alongside `#include "TectonicPlanet.h"`.

## Run Tests

Execute the tests headless using the UE automation framework:

```bash
# Linux
<EnginePath>/Engine/Binaries/Linux/UnrealEditor-Cmd "<ProjectPath>/Aurous.uproject" \
    -unattended -nopause -nosplash -nullrhi -nosound -nop4 \
    -ExecCmds="Automation RunTests Aurous.TectonicPlanet; Quit" \
    -log

# Windows
<EnginePath>/Engine/Binaries/Win64/UnrealEditor-Cmd.exe "<ProjectPath>/Aurous.uproject" \
    -unattended -nopause -nosplash -nullrhi -nosound -nop4 \
    -ExecCmds="Automation RunTests Aurous.TectonicPlanet; Quit" \
    -log
```

All 6 tests must pass. If any test fails, fix the implementation and re-run until they all pass. Report the full test output.

## Expected Numeric Results

Log these values and report them (they don't need to match exactly, but should be in range):

| Metric | Expected |
|---|---|
| Sample count | 500,000 (exact) |
| Triangle count | 999,996 (exact: 2N-4) |
| Valence-6 vertices | ~85-95% of total |
| Valence-5 vertices | ~5-12% |
| Plates | 7 (exact) |
| Interior triangle fraction | ~85-95% of total triangles |
| Boundary sample fraction | ~5-15% of total samples |
