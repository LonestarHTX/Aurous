# Follow-Up: 500k Validation + Plate Implementation Audit

## Goal

Two things need to happen before moving on:

1. **All tests must run at 500,000 samples**, not 60k or 10k. This is the production target and the resolution where numerical robustness actually matters.
2. **Audit and fix the plate initialization and triangle classification** against the spec below. You implemented these to pass the test assertions, but the full spec has details that the tests don't cover.

## Part 1: Bump All Tests to 500k

In `Source/Aurous/Private/Tests/TectonicPlanetTests.cpp`, change every `Planet.Initialize(...)` call to use `500000`. No test should use 60000, 10000, or any other value. The adjacency test should NOT use a smaller value "for speed" — 500k is the point.

Update the assertions accordingly:
- Sample count: `500000`
- Triangle count: `999996` (2N - 4)
- Plate tests, triangle classification tests: all at 500k

Also update the `FTectonicPlanet::Initialize` default parameter to `500000` if it isn't already (check `Source/Aurous/Public/TectonicPlanet.h`).

Rebuild and re-run:

```
Build.bat AurousEditor Win64 Development -Project="<path>\Aurous.uproject" -WaitMutex
UnrealEditor-Cmd.exe "<path>\Aurous.uproject" -unattended -nopause -nosplash -nullrhi -nosound -nop4 -ExecCmds="Automation RunTests Aurous.TectonicPlanet" -TestExit="Automation Test Queue Empty" -log
```

Report these metrics from the 500k run:
- Total wall time for `Initialize(500000)` (add timing around it if not already present)
- Triangle count (must be exactly 999,996)
- Valence distribution (V5/V6/V7/Other counts and percentages)
- Any degenerate triangles (area < epsilon) — report count if any
- Interior triangle fraction
- Boundary sample fraction

**This is the critical validation the architecture doc flags as an open question.** If `TConvexHull3` produces degenerate slivers or drops vertices at 500k co-spherical points, we need to know now.

## Part 2: Audit Plate Initialization Against Spec

Review your `InitializePlates()` implementation in `Source/Aurous/Private/TectonicPlanet.cpp` and ensure it matches all of the following. Fix anything that's missing or different.

### 2.1 — Plate Centroids

Generate `NumPlates` (default 7) random centroids uniformly on the unit sphere using `FRandomStream` seeded with `RandomSeed`:

```cpp
FRandomStream Rng(RandomSeed);
TArray<FVector> Centroids;
for (int32 p = 0; p < NumPlates; ++p)
{
    double Z = Rng.FRandRange(-1.0f, 1.0f);
    double Phi = Rng.FRandRange(0.0f, 2.0f * PI);
    double R = FMath::Sqrt(1.0 - Z * Z);
    Centroids.Add(FVector(R * FMath::Cos(Phi), R * FMath::Sin(Phi), Z));
}
```

### 2.2 — Sample Assignment

Assign each sample to the nearest centroid by dot product (no `acos` needed — larger dot = closer on the sphere):

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

### 2.3 — Plate Object Fields

Each `FPlate` must have all of these set:

- `Id` = plate index (0 to NumPlates-1)
- `RotationAxis` = a random unit vector on the sphere, generated from the same `FRandomStream` (so deterministic). Use the same z/phi method as centroids.
- `AngularSpeed` = `Rng.FRandRange(0.5f, 1.5f) * 3.14e-2f` (radians per timestep, derived from 100 mm/year max speed over 2 My timestep at planet radius 6370 km)
- `SampleIndices` = populated by scanning all samples and collecting indices where `PlateId == this plate's Id`

### 2.4 — Crust Types and Elevation

Using the same `FRandomStream`, randomly designate 1–3 plates as continental (the rest are oceanic). A simple approach:

```cpp
int32 NumContinental = Rng.RandRange(1, 3);
TSet<int32> ContinentalPlates;
while (ContinentalPlates.Num() < NumContinental)
{
    ContinentalPlates.Add(Rng.RandRange(0, NumPlates - 1));
}
```

Then for each sample, set:
- Continental plate → `CrustType = ECrustType::Continental`, `Elevation = 0.5f` (km above sea level)
- Oceanic plate → `CrustType = ECrustType::Oceanic`, `Elevation = -4.0f` (km, abyssal plain depth)

### 2.5 — Triangle Classification

`ClassifyTriangles()` must:
- Clear each plate's `InteriorTriangles` and `BoundaryTriangles` arrays
- For each triangle: if all 3 vertices share the same `PlateId`, add to that plate's `InteriorTriangles`. Otherwise, add to the `BoundaryTriangles` of every involved plate.
- Call `UpdateBoundaryFlags()` afterward

### 2.6 — Wire-Up

At the end of `InitializePlates()`, call `ClassifyTriangles()` then `UpdateBoundaryFlags()`.

### 2.7 — Verification

After fixing, rebuild and re-run all 6 tests at 500k. All must pass. Report any changes you made.

## Do NOT Do

- Do not reduce sample counts for speed — 500k is the target
- Do not create new files beyond what already exists (unless a missing header is needed)
- Do not modify the `FCanonicalSample` or `FPlate` struct definitions unless fixing a compile error
- Do not remove the `TectonicInitCommandlet` if it exists — just leave it alone
