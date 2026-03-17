# Boundary-Triangle Handling in Cortial et al.’s Procedural Tectonic Planets Resampling

## Context and what the “reconciliation” step is actually doing

In the author’s later doctoral thesis (which explicitly builds on the same tectonic-planet pipeline), the “periodic global operation” is described as **episodic global remeshing** that resolves **divergence-created gaps** by **reusing a precomputed global Spherical Delaunay Triangulation (TDS)** and **rebuilding plate meshes from it**. citeturn39view0turn37view0

The workflow relevant to your ambiguity is spelled out most directly in the thesis’ Chapter 3, in the section that deals with divergence handling and rifting. The key design choice is:

- They **do not incrementally “patch” holes** by expensive local Delaunay updates each timestep; instead they **let gaps grow for a period** and then do a **global operation** at the end of that period. citeturn37view0  
- That global operation is described as **“re-mailler complètement la planète”** (“remesh the entire planet”), explicitly by **re-using the global spherical Delaunay triangulation** that was computed once and reused many times. citeturn37view0turn39view0

This is the closest “implementation-level” description I found of the mechanism you called “global resampling / reconciliation,” including the interpolation and the “triangle soup containment” idea—implemented as **radial ray–triangle intersection against per-plate BVHs**, followed by **barycentric interpolation** of crust parameters. citeturn37view0turn35view5

## What the sources say about data structures: global triangulation, per-plate meshes, and rigid motion

### Global triangulation is precomputed, then reused

The thesis describes the crust as a sampled function over the sphere using a large point set, and then triangulating those samples with a **Spherical Delaunay Triangulation (TDS)** so the crust can be evaluated **continuously via barycentric interpolation**. citeturn39view0

Crucially, it also states that:

- The Fibonacci sampling + triangulation are **precomputed once** and **re-used**, including for **episodic remeshing during simulation**. citeturn39view0turn37view0

So: **there is a single global triangulation used as the canonical mesh substrate**, not a new global triangulation built from scratch every reconciliation.

### Plates move as rigid bodies via a single rotation per plate

The plate-motion model is explicitly a **rigid geodesic motion on the sphere**, i.e. a **rotation about an axis through the planet center**. The thesis states each plate evolves via a rigid geodesic motion \(G\) that is a rotation defined by an axis \(w\) through the center, with surface velocity \(s(p)=\omega\, w \times p\). citeturn38view0turn39view0

That directly supports your interpretation (3): for any geometry belonging to a plate mesh, you should apply a **uniform rigid transform per plate**, not a per-vertex mixture of transforms. citeturn38view0

### Plates are created and recreated by partitioning and duplicating sub-triangulations

In the episodic divergence/remeshing description, after assigning each global vertex to a plate (or identifying it as “in divergence”), the thesis describes:

- **“partitionner l’ensemble des triangles de la TDS globale afin de re-créer les plaques tectoniques”** and then
- **duplicating sub-triangulations**, with **duplication / re-indexing**, exactly as was done at initial creation. citeturn37view0turn39view0

This “duplication/ré-indiçage” language is strong evidence that the implementation treats plates as **independent triangle soups / meshes**, potentially with duplicated boundary vertices/triangles, rather than one shared global vertex buffer forced to satisfy multiple plate motions. citeturn37view0

## The reconciliation algorithm described in the thesis (and what it implies for boundary triangles)

The thesis gives a step-by-step algorithm for the episodic global divergence treatment (the global “re-meshing” step). I’ll restate it faithfully, because it directly answers your questions about how triangles are used during resampling and how boundary zones are handled.

### Preprocessing per plate: boundaries and BVHs

At the end of each remeshing period, for each plate in parallel they:

- Recompute the plate boundary **ignoring triangles in subduction**,
- Destroy plates that are entirely subducted,
- Build a **BVH over that plate’s triangles**. citeturn37view0

This is important because it tells you what their “containment” primitive is: **ray–triangle intersection against each plate’s BVH**, not “point-in-spherical-polygon” in some abstract plate boundary. citeturn37view0

### For each global vertex, find intersected plate triangle and barycentrically interpolate

For each global vertex \(p\) of the global TDS:

1. Cast a ray from the planet center through \(p\).
2. For each existing plate, test ray–triangle intersection via that plate’s BVH.
3. If the intersected triangle is marked as **subduction or collision**, ignore that intersection.
4. Otherwise, accept the intersection and do **barycentric interpolation of crust parameters** from the intersected triangle and assign them to \(p\).
5. Record the plate index that was intersected. citeturn37view0turn35view5

This answers your ambiguity about “parameters computed using barycentric interpolation of crust data from the plate they intersect”: in this implementation, “intersect” is literally **radial ray–triangle intersection with a triangle belonging to a particular plate mesh**, followed by barycentric interpolation on that triangle. citeturn37view0turn35view5

### If no plate triangle intersects, treat it as divergent gap and create new oceanic crust

If no valid intersection is found, the vertex is considered to lie in a **divergence gap**.

Then they:

- Search plate boundaries to find the closest boundary point \(q_1\) on some plate and a second closest boundary point \(q_2\) on a different plate,
- Use these to compute new oceanic crust parameters at \(p\), including estimating a “mid-ocean ridge” point by taking the midpoint on the sphere between \(q_1\) and \(q_2\),
- Record (for bookkeeping) the index of the closest existing plate. citeturn37view0

This is the key mechanism that prevents the “20% uncovered gap zone” failure mode you saw when you excluded boundary triangles: they **explicitly fill divergence gaps** during the remeshing/resampling pass. citeturn37view0

### Rebuild plates by assigning triangles to plates and duplicating sub-triangulations

After every global vertex has crust parameters and a recorded plate index, they:

- Partition the **triangles of the global TDS** to recreate the tectonic plates,
- Use the recorded per-vertex plate indices as the basis for triangle-to-plate assignment,
- Then **duplicate and re-index** the relevant sub-triangulations so each plate becomes its own mesh again. citeturn37view0turn39view0

They state the *effect* of this is: preserving plate shapes from the previous timestep while **augmenting** them to cover divergence zones, “**de manière étanche**” (watertight / fully covering), and that plate geodesic motions are preserved. citeturn37view0

They also note a side-effect: subductions are effectively reset because subducted triangles are removed at remeshing time, requiring subduction state to be reconstructed afterward. citeturn37view0

### Direct implication for boundary triangles

This text does not define “boundary triangle” in your exact sense (“a global triangle whose three vertices belong to multiple plates”), but it does imply a consistent mechanism:

- **Plates are meshes made of triangles that belong to that plate** (because BVHs are built “on its triangles,” and intersections are attributed to “the intersected plate”). citeturn37view0  
- Plates are rebuilt by **partitioning the global TDS triangles and duplicating/reindexing sub-triangulations**. citeturn37view0turn39view0

From that, the most defensible reading is:

- During resampling / reconciliation, **a triangle is not allowed to deform via per-vertex plate transforms**. Instead, triangles used in containment/intersection belong to a **specific plate mesh**, and that mesh is transformed rigidly by that plate’s rotation. citeturn38view0turn37view0  
- “Boundary triangles” in the *global* triangulation are handled by the **triangle-to-plate assignment + duplication/reindexing step**, so that each plate ends up with a valid triangle soup suitable for BVH intersection, and the planet remains “étanche” after the remeshing. citeturn37view0  

In other words, the approach is structurally aligned with your interpretation (3), but with an additional crucial ingredient: **the periodic global remeshing step includes explicit divergence-gap filling and then an explicit rebuild of per-plate triangle soups from the global TDS.** citeturn37view0turn39view0

## Answers to your specific questions

### Does each plate maintain its own separate Delaunay triangulation, or is the global triangulation partitioned?

The thesis describes a **global spherical Delaunay triangulation (TDS)** of the sample points, computed once and reused. citeturn39view0

Plates are then (re)constructed by **partitioning the triangles of that global TDS** and **duplicating / re-indexing** sub-triangulations into per-plate meshes. citeturn37view0turn39view0

So the most consistent interpretation is: **global TDS + per-plate sub-triangulations (meshes) obtained by partition + duplication**, not “each plate owns an independently computed Delaunay triangulation.” citeturn37view0turn39view0

### How are boundary triangles handled—included in one plate’s soup, excluded, or something else?

They aim for an “étanche” rebuild (complete coverage) after remeshing, and they explicitly do so by (a) assigning each global vertex to a plate or to divergence fill, and then (b) **partitioning triangles of the global TDS** and **duplicating sub-triangulations** into new plate meshes. citeturn37view0

That implies boundary-adjacent triangles are **not simply dropped** (which would cause holes), but are handled by the triangle-partitioning and duplication process. citeturn37view0

What the text does *not* explicitly specify (in the excerpted material) is the exact *rule* mapping a mixed-plate triangle to a plate (e.g., barycenter-in-cell, majority vertex label, etc.). What it does specify is that triangles are assigned to plates “d’après l’assignation des triangles aux plaques,” driven by per-vertex plate indices, and then duplicated/reindexed into per-plate meshes. citeturn37view0turn39view0

So the resolved ambiguity is the *category* of handling: **“something else” = rebuild plates by partitioning the global TDS triangles and duplicating sub-triangulations**, rather than “exclude boundary triangles.” citeturn37view0

### What rotation is applied to boundary triangle vertices during containment queries?

The plate motion model is a **rigid geodesic rotation** per plate (rotation about axis through the center). citeturn38view0turn39view0

Containment/intersection during resampling is performed by intersecting rays against triangles stored in **per-plate BVHs** (“BVH sur ses triangles”), and the outcome is attributed to the **plate intersected**. citeturn37view0

Taken together, the evidence supports:

- **Uniform rigid rotation per plate** for all vertices of triangles belonging to that plate mesh, including triangles along boundaries, because the plate is explicitly a rigid rotating body. citeturn38view0turn37view0

This explicitly contradicts your attempted interpretation (2) (per-vertex rotation) as a plausible reading of the described implementation, because per-vertex mixed rotations would not be consistent with “chaque plaque” being a rigid rotation and with BVHs built “on its triangles.” citeturn38view0turn37view0

### Is there any per-plate retriangulation at resampling time?

The thesis explicitly motivates avoiding costly Delaunay updates (“augmentations de triangulations de Delaunay coûteuses”) and instead proposes **complete remeshing** using the **already-built global TDS**. citeturn37view0turn39view0

So, rather than per-plate retriangulation, the step is:

- **Global remeshing by reusing the global TDS**, then
- **Rebuilding plate meshes by partitioning and duplicating sub-triangulations**. citeturn37view0turn39view0

### Does any related work describe “foreign vertices” or “shared vertices” in plate triangle soups?

The strongest relevant phrase is the repeated use of **duplication and re-indexing** (“duplication/ré-indiçage”) when converting global-TDS subsets into plate meshes, both at initial state and after remeshing. citeturn37view0turn39view0

That is not the same as an explicit “foreign vertex” terminology, but it is the standard mechanism by which boundary vertices/triangles become **non-shared across plates** (each plate gets its own copy), enabling rigid transforms per plate without per-vertex mixing. This is an inference, but it is directly motivated by the text’s duplication/reindexing language and the rigid-motion model. citeturn37view0turn38view0

### Any implementation code, pseudocode, or algorithmic detail beyond the published paper?

The thesis provides concrete algorithmic detail for the periodic remeshing/resampling step, including:

- Remeshing period range and dependence on max plate speed (with an example range between 32 Ma and 128 Ma in the implementation), citeturn37view0  
- BVH construction per plate and exclusion of subduction/collision triangles during sampling, citeturn37view0  
- Radial ray–triangle intersection per global TDS vertex and barycentric interpolation of crust parameters on the intersected triangle, citeturn37view0turn35view5  
- Explicit divergence handling when there is no intersection, based on nearest boundary points, citeturn37view0  
- Rebuilding plates by partitioning global-TDS triangles and duplicating/reindexing sub-triangulations. citeturn37view0

This is substantially more operational detail than what is visible in the (publisher-preview) copy of the 2019 article accessible in the sources I could open. citeturn23view2turn23view3turn23view4turn23view5

## Practical resolution of your three interpretations

Based on the thesis’ explicit description of the periodic “global remeshing” step:

- **Exclude boundary triangles entirely** is inconsistent with their stated goal of an “étanche” (fully covering) remeshing, and inconsistent with their explicit divergence-gap filling. citeturn37view0  
- **Include boundary triangles with per-vertex rotation** conflicts with the plate motion model being a rigid rotation per plate and with the “BVH on its triangles” containment/intersection model. citeturn38view0turn37view0  
- **Include boundary triangles with uniform rigid rotation** matches the plate-motion model and matches how triangles are treated in BVH intersection. citeturn38view0turn37view0  

But the key missing piece (relative to your interpretation (3) as written) is:

- They do **not** rely on a static per-plate interior-triangle subset over long divergence intervals; instead they periodically **rebuild the plate meshes from the global TDS**, including explicit handling of divergence zones where no intersection is found. citeturn37view0turn39view0

## Where this sits in the broader publication trail you referenced

- The article appears in entity["organization","Computer Graphics Forum","wiley journal"] / entity["organization","Eurographics","annual cg conference"] 2019 programming (front matter / preface listing). citeturn14search2  
- entity["people","Eric Guérin","computer graphics researcher"]’s publication page lists the 2019 paper and the 2020 follow-up (with “pdf” links, though the HAL-hosted PDFs were not fetchable from the environment used here). citeturn18view0turn19view0  
- A entity["video_game","Star Citizen","cloud imperium games"] reference appears in the publisher preview text and also appears in the thesis as an industry example motivating planet-scale terrain. citeturn23view4turn33view0  
- A entity["tv_show","YouTube","video platform"] entry exists for an entity["organization","Eurographics","annual cg conference"] 2019 video about the paper. citeturn17search7

The decisive boundary-triangle/resampling details, however, are contained in the thesis’ description of the episodic remeshing/resampling step and its rebuild of per-plate triangle soups. citeturn37view0turn39view0