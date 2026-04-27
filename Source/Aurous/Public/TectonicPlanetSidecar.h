#pragma once

#include "CoreMinimal.h"
#include "TectonicPlanet.h"

enum class ETectonicSidecarProjectionMode : uint8
{
	MaterialSupportDiagnostic,
	ExplicitFootprints,
	VoronoiOwnershipDecoupledMaterial,
};

enum class ETectonicSidecarMaterialClassification : uint8
{
	None,
	ExactFootprint,
	RecoveredFootprint,
	ProjectedMaterialCloud,
	OceanFallback,
	DivergentOceanFill,
};

struct AUROUS_API FTectonicSidecarConfig
{
	int32 SampleCount = 60000;
	int32 PlateCount = 40;
	int32 Seed = 42;
	double PlanetRadiusKm = 6371.0;
	double DeltaTimeMy = 2.0;
	float InitialContinentalFraction = 0.30f;
	float BoundaryWarpAmplitude = 0.12f;
	double MinPlateSpeedKmPerMy = 5.0;
	double MaxPlateSpeedKmPerMy = 20.0;
	bool bForceZeroAngularSpeeds = false;
	ETectonicSidecarProjectionMode ProjectionMode = ETectonicSidecarProjectionMode::VoronoiOwnershipDecoupledMaterial;
	double RecoveryToleranceRad = -1.0;
	double MeaningfulHitContainmentScore = 0.02;
	double DivergenceMinKmPerMy = 0.5;
	double DivergenceSpeedFraction = 0.05;
	bool bForceExplicitProjectionAtRestForTest = false;
};

struct AUROUS_API FTectonicSidecarMaterialSample
{
	FVector3d LocalPosition = FVector3d::ZeroVector;
	int32 InitialCanonicalSampleIndex = INDEX_NONE;
	float ContinentalWeight = 0.0f;
	float Elevation = 0.0f;
	float Thickness = 0.0f;
	float Age = 0.0f;
};

struct AUROUS_API FTectonicSidecarMaterialGrid
{
	int32 LongitudeBins = 64;
	int32 LatitudeBins = 32;
	TArray<TArray<int32>> Bins;
};

struct AUROUS_API FTectonicSidecarFootprintVertex
{
	FVector3d LocalPosition = FVector3d::ZeroVector;
	int32 InitialCanonicalSampleIndex = INDEX_NONE;
	float ContinentalWeight = 0.0f;
	float Elevation = 0.0f;
	float Thickness = 0.0f;
	float Age = 0.0f;
};

struct AUROUS_API FTectonicSidecarFootprintTriangle
{
	int32 GlobalTriangleIndex = INDEX_NONE;
	FTectonicSidecarFootprintVertex Vertices[3];
	double UnitSphereArea = 0.0;
};

struct AUROUS_API FTectonicSidecarPlate
{
	int32 PlateId = INDEX_NONE;
	FVector3d InitialCenter = FVector3d::ZeroVector;
	FVector3d RotationAxis = FVector3d(0.0, 0.0, 1.0);
	double AngularSpeedRadPerMy = 0.0;
	FQuat4d WorldFromLocal = FQuat4d::Identity;
	TArray<FTectonicSidecarMaterialSample> MaterialSamples;
	FTectonicSidecarMaterialGrid MaterialGrid;
	TArray<FTectonicSidecarFootprintTriangle> FootprintTriangles;
	FVector3d InitialCoreCentroidLocal = FVector3d::ZeroVector;
	double InitialContinentalMass = 0.0;
	double InitialFootprintContinentalMass = 0.0;
	double SupportDistanceThresholdRad = 0.0;
	int32 InitialContinentalCoreSampleCount = 0;
};

struct AUROUS_API FTectonicSidecarProjectionDiagnostics
{
	int32 Step = 0;
	int32 SampleCount = 0;
	int32 PlateCount = 0;
	uint32 ProjectionHash = 0;

	double LocalContinentalMass = 0.0;
	double ProjectedContinentalMass = 0.0;
	double MaterialMassRelativeError = 0.0;

	// Sum over all carried footprint triangles, area-weighted by spherical triangle area and mean vertex continental weight.
	double LocalFootprintContinentalMass = 0.0;
	// Sum of every containing footprint hit on the lattice, counting overlaps multiple times.
	double MultiHitProjectedContinentalMass = 0.0;
	// Sum of final visible sample continental weight after overlap winner and gap fill.
	double VisibleProjectedContinentalMass = 0.0;
	// abs(MultiHitProjectedContinentalMass - LocalFootprintContinentalMass) / LocalFootprintContinentalMass.
	double FootprintMassRelativeError = 0.0;
	// Non-gap visible samples not sourced from exact footprint containment or bounded recovery.
	double FabricatedMaterialFraction = 0.0;
	// Samples with no exact footprint hit and no bounded recovery.
	double GapFraction = 0.0;
	// Gap samples whose flanking plate velocities separate above the configured divergence threshold.
	double DivergentGapFraction = 0.0;
	// Gap samples not classified as divergent.
	double NonDivergentGapFraction = 0.0;
	// Samples contained by more than one meaningful footprint.
	double OverlapFraction = 0.0;
	// Samples filled as projected-only oceanic crust.
	double OceanFillFraction = 0.0;
	// Largest connected divergent-gap component divided by total divergent-gap samples.
	double LargestDivergentGapComponentFraction = 0.0;
	// Boundary fraction from non-gap visible owner changes.
	double PlateBoundaryFraction = 0.0;
	// Boundary fraction from divergent gap/non-gap or divergent gap owner changes.
	double DivergentBoundaryFraction = 0.0;
	int32 ExactFootprintHitSampleCount = 0;
	int32 RecoveredFootprintSampleCount = 0;
	int32 GapSampleCount = 0;
	int32 DivergentGapSampleCount = 0;
	int32 NonDivergentGapSampleCount = 0;
	int32 OverlapSampleCount = 0;
	int32 OceanFillSampleCount = 0;
	int32 FabricatedMaterialSampleCount = 0;
	int32 ClassifiedSampleCount = 0;

	int32 OutOfSupportSampleCount = 0;
	double OutOfSupportFraction = 0.0;
	double MaxSupportDistanceKm = 0.0;
	double MeanSupportDistanceKm = 0.0;

	double MeanProjectedLocalCoreDriftKm = 0.0;
	double MaxProjectedLocalCoreDriftKm = 0.0;
	double MeanKinematicDriftKm = 0.0;
	double MeanProjectedDriftKm = 0.0;
	double MeanDriftMismatchKm = 0.0;
	double MaxDriftMismatchKm = 0.0;
	int32 DriftPlateCount = 0;

	double BoundaryFraction = 0.0;
	double LargePlateMeanDominantComponentFraction = 0.0;
	double TinyComponentFraction = 0.0;
	int32 RegionComponentCount = 0;
	int32 MultiComponentPlateCount = 0;

	double ColdStartPlateMismatchFraction = 0.0;
	double ColdStartContinentalMeanAbsError = 0.0;
	double ColdStartBoundaryFractionDelta = 0.0;

	// Ownership gaps are impossible in Prototype C's nearest-center Voronoi partition.
	double OwnershipGapFraction = 0.0;
	// Ownership overlaps are impossible in Prototype C's nearest-center Voronoi partition.
	double OwnershipOverlapFraction = 0.0;
	// Per-plate samples outside that plate's largest connected owner component, divided by sample count.
	double OwnerFragmentationFraction = 0.0;
	// Boundary samples whose one-ring owner set has more than three distinct plate ids, divided by sample count.
	double BoundaryNoiseFraction = 0.0;
	// Projected visible continental mass vs carried sidecar footprint continental mass.
	double CarriedContinentalMassRelativeError = 0.0;
	// Visible non-ocean material not sourced from exact or bounded recovered carried support.
	double MaterialFabricatedFraction = 0.0;
	// Samples whose visible material source plate differs from Voronoi ownership plate.
	double MaterialOwnerMismatchFraction = 0.0;
	// Samples hit by more than one meaningful carried material support.
	double MaterialOverlapFraction = 0.0;
	// Increase in high-continental-weight material fragmentation relative to step zero.
	double HighCWFragmentationDelta = 0.0;
	int32 OwnershipGapSampleCount = 0;
	int32 OwnershipOverlapSampleCount = 0;
	int32 OwnerFragmentedSampleCount = 0;
	int32 BoundaryNoiseSampleCount = 0;
	int32 MaterialOwnerMismatchSampleCount = 0;
	int32 MaterialOverlapSupportSampleCount = 0;
};

class AUROUS_API FTectonicPlanetSidecar
{
public:
	void Initialize(const FTectonicSidecarConfig& InConfig);
	void AdvanceStep();
	void AdvanceSteps(int32 StepCount);

	void ProjectToPlanet(
		FTectonicPlanet& OutPlanet,
		FTectonicSidecarProjectionDiagnostics* OutDiagnostics = nullptr) const;
	bool SetPlateKinematicsForTest(int32 PlateId, const FVector3d& Axis, double AngularSpeedRadPerMy);

	const FTectonicSidecarConfig& GetConfig() const { return Config; }
	const FTectonicPlanet& GetInitialPlanet() const { return InitialPlanet; }
	const TArray<FTectonicSidecarPlate>& GetPlates() const { return Plates; }
	const TArray<int32>& GetLastMaterialSourcePlateIds() const { return LastMaterialSourcePlateIds; }
	const TArray<uint8>& GetLastMaterialClassifications() const { return LastMaterialClassifications; }
	const TArray<uint8>& GetLastMaterialOwnerMismatchFlags() const { return LastMaterialOwnerMismatchFlags; }
	const TArray<int32>& GetLastMaterialOverlapCounts() const { return LastMaterialOverlapCounts; }
	const TArray<uint8>& GetLastDivergentBoundaryFlags() const { return LastDivergentBoundaryFlags; }
	int32 GetCurrentStep() const { return CurrentStep; }

	static uint32 ComputeProjectionHash(const FTectonicPlanet& Planet);

private:
	FTectonicSidecarConfig Config;
	FTectonicPlanet InitialPlanet;
	TArray<FTectonicSidecarPlate> Plates;
	TMap<int32, int32> PlateIndexById;
	TArray<int32> InitialPlateIds;
	TArray<float> InitialContinentalWeights;
	TArray<uint8> InitialBoundaryFlags;
	mutable TArray<int32> LastMaterialSourcePlateIds;
	mutable TArray<uint8> LastMaterialClassifications;
	mutable TArray<uint8> LastMaterialOwnerMismatchFlags;
	mutable TArray<int32> LastMaterialOverlapCounts;
	mutable TArray<uint8> LastDivergentBoundaryFlags;
	int32 CurrentStep = 0;

	void BuildPlatesFromInitialPlanet();
	void BuildFootprintsFromInitialPlanet();
	void BuildMaterialGrid(FTectonicSidecarPlate& Plate) const;
	void ComputeInitialPlateDiagnostics(FTectonicSidecarPlate& Plate) const;
	void AssignSidecarKinematics();
	void ProjectExplicitFootprintsToPlanet(
		FTectonicPlanet& OutPlanet,
		FTectonicSidecarProjectionDiagnostics* OutDiagnostics) const;
	void ProjectVoronoiOwnershipDecoupledMaterialToPlanet(
		FTectonicPlanet& OutPlanet,
		FTectonicSidecarProjectionDiagnostics* OutDiagnostics) const;
	int32 FindWinningPlateIdForSample(int32 SampleIndex, const FVector3d& UnitPosition) const;
	int32 FindNearestRotatedCenterPlateId(const FVector3d& UnitPosition) const;
	const FTectonicSidecarPlate* FindPlateById(int32 PlateId) const;
	FTectonicSidecarPlate* FindPlateById(int32 PlateId);
	void SamplePlateMaterial(
		const FTectonicSidecarPlate& Plate,
		const FVector3d& QueryLocalPosition,
		float& OutContinentalWeight,
		float& OutElevation,
		float& OutThickness,
		float& OutAge,
		double& OutNearestDistanceRad) const;
	void RecomputeProjectedBoundariesAndPlateMembers(FTectonicPlanet& InOutPlanet) const;
	void FillProjectionDiagnostics(
		const FTectonicPlanet& ProjectedPlanet,
		const TArray<double>& NearestSupportDistancesRad,
		FTectonicSidecarProjectionDiagnostics& OutDiagnostics) const;
	void ResetLastProjectionDebug(int32 SampleCount) const;
};
