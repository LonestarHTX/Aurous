# **Computational Frameworks for Subduction and Crustal Dynamics in Procedural Tectonic Planets**

The synthesis of planetary-scale terrains has evolved from purely stochastic methods to kinematic simulations that approximate the physical reality of lithospheric movement. The methodology proposed by Cortial et al. (2019) and further detailed in Cortial’s 2020 doctoral thesis represents a shift toward procedural models where the morphology of the planet is a direct consequence of tectonic plate interactions.1 Within this framework, subduction is the most complex mechanism to implement, as it involves the convergence of disparate crustal types, the vertical deformation of the overriding plate, and the kinematic feedback of slab pull that drives further plate motion.1

## **Geometric Representation of the Planetary Lithosphere**

A procedural tectonic planet requires a discrete representation that can handle spherical topology without the distortions inherent in rectilinear grids. Cortial et al. utilize a mesh-based approach where the sphere is partitioned into distinct tectonic plates.1 Each plate is a collection of samples—vertices in a mesh—that move as a rigid or semi-rigid body across the planetary surface. The foundational geometry often begins as a subdivided icosahedron or a spherical Delaunay triangulation, providing a relatively uniform distribution of samples.2

### **Subduction Front Geometry and Representation**

The subduction front is the primary interface where lithospheric interaction occurs. It is geometrically represented as a set of directed edges along the boundary of the overriding plate.3 Identifying these edges requires a robust topological analysis of the plate boundaries. An edge is classified as part of a subduction front if it satisfies two conditions: first, it must be on the boundary of a plate; second, the relative velocity vector of the neighboring plate must be oriented toward the boundary, indicating convergence.1  
The orientation of these Delaunay edges is critical. The system assigns a normal vector to the front that points into the interior of the overriding plate, defining the direction in which the distance field will be propagated.1 This orientation ensures that the resulting mountain ranges and volcanic arcs are correctly positioned on the overriding crust rather than the plunging slab.1

| Geometric Property | Specification in Cortial Framework | Implementation Utility |
| :---- | :---- | :---- |
| Primary Primitive | Spherical Delaunay Triangulation 2 | Maintains topological consistency during drift |
| Front Definition | Directed Delaunay Edges 3 | Defines the locus of crustal deformation |
| Boundary Marker | OpposingPlateId per vertex 3 | Facilitates local collision detection |
| Orientation Vector | Surface Normal $\\times$ Edge Tangent 1 | Orients the uplift falloff function |

The representation of the front as a series of edges allows the simulation to handle complex boundary shapes, including curved trenches and segmented fault lines. In implementations like Driftworld Tectonics, these edges are stored in a dynamic list that is re-evaluated at each simulation step to account for the rotation of the plates.3

## **Distance-to-Subduction-Front Computation**

The morphological impact of subduction—such as the height of the volcanic arc and the depth of the trench—is a function of the distance from any given point $p$ on the plate to the nearest subduction front $\\Gamma$.1 Computing this distance, $d(p, \\Gamma)$, accurately and efficiently is a core challenge.

### **Graph-Based Propagation vs. Euclidean Distance**

While Euclidean distance is mathematically straightforward, it fails to account for the mesh's connectivity and can lead to artifacts where the deformation "leaks" through thin sections of a plate or across non-connected regions.2 Cortial’s research favors a graph-based approach, specifically utilizing a Breadth-First Search (BFS) or Dijkstra-based traversal on the Delaunay graph of the plate.2

1. **Initial Seeds:** The vertices that constitute the subduction front edges are designated as initial seeds with a distance value of zero.3  
2. **Breadth-First Search (BFS):** Starting from these seeds, the distance values are propagated to neighboring vertices. Each vertex $v$ updates its distance $d(v)$ by taking the minimum of its current value and the distance of its neighbor plus the edge length.2  
3. **Resampling:** As plates move and the mesh deforms, the graph may become stretched. The system periodically resamples the plate or performs local edge flips to maintain a high-quality Delaunay triangulation, which in turn ensures that the BFS distance remains a close approximation of the geodesic distance on the sphere.1

This propagation is often implemented in a compute shader to leverage the massive parallelism of the GPU. In the Driftworld implementation (CSPlateInteractions.compute), the distance field is calculated in multiple passes, where each pass expands the influence of the subduction front further into the plate's interior.3

## **Mathematical Modeling of Crustal Uplift**

The defining feature of a subduction zone is the creation of significant vertical relief. Cortial et al. (2019) model this as an uplift $\\Delta z(p)$ applied to the vertices of the overriding plate. The magnitude of this uplift is determined by a product of several transfer functions that capture the physics of convergence.1  
The general formula for subduction uplift $ue\_j$ is:

$$ue\_j \= u\_0 \\cdot g(v) \\cdot f(d) \\cdot h(ez\_i)$$  
where $u\_0$ is a base uplift coefficient.1

### **The Piecewise Cubic Falloff Function**

The function $f(d)$ defines the cross-sectional profile of the mountain range. To achieve a realistic look, Cortial utilizes a piecewise cubic function rather than a simple linear decay.1 This allows the uplift to have a "smoothstart" near the trench, reach a peak at a specific distance (the volcanic arc), and then fade out smoothly into the back-arc basin.1

| Distance Parameter | Role in Uplift Profile | Physical Analog |
| :---- | :---- | :---- |
| $d\_{min} \= 0$ | Start of the subduction front | The oceanic trench |
| $d\_{peak}$ | Point of maximum uplift | The volcanic arc or mountain crest 1 |
| $d\_{max}$ | Maximum influence radius | Limits of crustal thickening 1 |
| $x \= (d \- d\_{min}) / (d\_{max} \- d\_{min})$ | Normalized distance for cubic calc | Spatial interpolation factor |

The piecewise cubic function is typically defined as $f(x) \= 1 \- 3x^2 \+ 2x^3$ (or a variation thereof), which provides C1 continuity at the boundaries, preventing sharp "seams" in the terrain where the subduction influence ends.1 The influence radius $d\_{max}$ is a critical parameter; empirical values in the simulation often range from 1000 km to 1800 km, reflecting the broad scale of mantle-induced lithospheric warping.1

### **Scaling by Velocity and Crustal Elevation**

The uplift is not solely dependent on distance. The relative convergence velocity $v$ and the nature of the subducting crust $ez\_i$ play vital roles:

* **Velocity Transfer Function ($g(v)$):** This is typically a linear function $g(v) \= v / v\_0$, where $v\_0$ is a reference velocity. Higher convergence rates lead to more rapid crustal thickening and thus higher peaks.1  
* **Height Transfer Function ($h(ez\_i)$):** This function accounts for the elevation (and thus thickness) of the plunging plate. It is defined as $h(ez\_i) \= ez\_i^2$. If the subducting plate is a high-standing continental block or a thick oceanic plateau, the resulting uplift on the overriding plate is significantly enhanced compared to the subduction of deep, thin oceanic crust.1

## **Kinematic Drivers and Slab Pull Mechanics**

In a dynamic tectonic model, the movement of the plates is not just an input but is driven by the forces generated at their boundaries. The most significant force in global tectonics is slab pull—the gravitational pull of the subducting plate as it sinks into the mantle.1

### **Torque Formula and Accumulation**

Because the plates are on a sphere, all forces must be converted into torques acting around the center of the planet. For each vertex $p$ on a subducting boundary, a force vector $F\_{slab}$ is calculated. This force is directed toward the trench and scaled by the local convergence rate and the density of the plunging slab.1  
The torque $\\tau$ for a single boundary segment is:

$$\\tau \= p \\times F\_{slab}$$  
The total torque for a plate is the vector sum of all individual torques along its active subduction zones:

$$\\tau\_{total} \= \\sum\_{i \\in \\text{front}} \\tau\_i$$  
This accumulated torque is used to update the plate's angular velocity $\\omega$ at each timestep.3 This creates a positive feedback loop: as a plate subducts more of its crust, the slab pull force increases, which increases the plate's speed, which in turn increases the subduction rate and the resulting uplift on the overriding plate.1

### **Plate Rotation Axis Modification**

The resulting $\\tau\_{total}$ modifies the plate's rotation axis. In implementation, each plate $P\_j$ maintains a rotation matrix $R\_j$ and an angular velocity vector $\\Omega\_j$. The new rotation is computed by integrating the angular acceleration $\\alpha \= \\tau / I$, where $I$ is the moment of inertia of the plate (approximated based on its area and crustal mass).1 This ensures that the plates move realistically across the sphere, responding to the complex geometry of their boundaries.

## **Subduction-to-Collision Transition**

One of the most difficult transitions to model is the "suturing" of two continental masses. Unlike oceanic crust, which is dense and easily recycled into the mantle, continental crust is buoyant and resists subduction.1

### **State Machine and Terrane Metadata**

To manage this, the framework uses a state machine that relies on metadata stored at each vertex:

* **TerraneId:** Identifies the crustal archetype (e.g., 0 for Oceanic, 1 for Continental).3  
* **OpposingPlateId:** Identifies which plate is currently being interacted with at a boundary.3

When a vertex on the subducting plate with a TerraneId of 1 (Continental) reaches the subduction front, the simulation triggers a state transition from "Subduction" to "Collision".1 In the collision state, the uplift logic changes. Instead of a smooth cubic profile on only the overriding plate, the system applies a "crumpling" effect to both plates.1

### **The Collision Coefficient and Centroid Folding**

During collision, the discrete collision coefficient $\\Delta c$ is significantly increased. This coefficient determines how much of the horizontal convergence is converted into vertical elevation.1 The folding direction $f(p,t)$ is updated based on the centroid $q$ of the colliding continental blocks:

$$f(p,t \+ \\delta t) \= (n \\times \\frac{p \- q}{\\|p \- q\\|}) \\times n$$  
where $n$ is the surface normal at $p$.1 This formula ensures that the mountain ranges (like the Himalayas) form along the contact zone and are oriented perpendicular to the direction of maximum compression.1

## **Triangle Exclusion and Containment**

As plates overlap during subduction, it is essential to manage the "hidden" geometry. Vertices of the plunging plate still exist in the simulation but must not contribute to the visible terrain or to future collision detection.2

### **Subduction Marking and Marking Logic**

Cortial’s thesis describes a "subduction marking" process where vertices of the plunging plate are flagged once they cross the subduction front.2

* **Exclusion from Triangulation:** Marked vertices are excluded from the Delaunay triangulation used for rendering the planet's surface, preventing the "inner" side of the subducting slab from creating visual artifacts.2  
* **Containment Query Filtering:** During the simulation, the system frequently checks which plate a sample $p$ belongs to (containment). Subducted vertices are ignored during these queries to prevent them from interfering with the movement of the overriding plate.1  
* **Sample Re-attachment:** In some cases, as vertices pass deep under the overriding plate, they are "detached" from the plunging plate and "re-attached" to the overriding plate’s data structure to represent the accretion of material.1

This marking is often distance-based: if $d(p) \> d\_{threshold}$, the vertex is considered recycled into the mantle and is effectively deactivated.1

## **Analysis of Driftworld’s Implementation**

The Driftworld Tectonics project provides a practical window into the implementation of these concepts within a modern game engine like Unity.3

### **Compute Shader Logic (CSPlateInteractions)**

The CSPlateInteractions.compute shader is responsible for the heavy lifting of the subduction simulation. It processes thousands of vertices in parallel to update the distance fields and uplift values.3

* **Kernel: IdentifyFronts:** Each thread checks a boundary vertex. If the relative velocity indicates convergence, it writes to a SubductionFront buffer.3  
* **Kernel: PropagateDistance:** This kernel uses an iterative approach to fill the distance field. It relies on the neighbor-list of each vertex to spread the distance value from the front.3  
* **Kernel: ApplyUplift:** This kernel implements the piecewise cubic formula $f(d)$ and scales it by the pre-computed velocity and height factors.3

### **Planet.cs and Kinematic Management**

The Planet.cs script acts as the orchestrator. It calculates the global plate properties, such as the area-weighted centroid and the moment of inertia.3 It then iterates through all plates to accumulate the torques from slab pull (subduction zones) and ridge push (divergent zones). The final rotation for each plate is updated using a high-precision integration scheme to ensure numerical stability over thousands of simulation years.3

## **Artifacts, Stability, and Parameter Sensitivity**

Maintaining a stable tectonic simulation is notoriously difficult due to the large time scales and the discrete nature of the mesh. Section 5 of the Driftworld documentation highlights several common issues.6

### **Parameter Sensitivity and Numerical Stability**

| Parameter | Impact on Stability | Sensitivity |
| :---- | :---- | :---- |
| Time Step ($\\delta t$) | High $\\delta t$ causes "jumping" vertices 6 | Extreme \- requires sub-stepping |
| Subduction Radius ($d\_{max}$) | Large radius can overlap other boundaries 4 | Moderate \- affects mountain width |
| Base Uplift ($u\_0$) | Excessive $u\_0$ creates unrealistic peaks 1 | Low \- primarily aesthetic |
| Velocity Scalar ($v\_0$) | High sensitivity to plate speed 1 | High \- dictates global relief scale |

One major artifact is "mesh jitter," where the discrete movement of the plates causes the subduction front to flicker between neighbor vertices. This is mitigated in Driftworld by using an exponential moving average for the front positions and the relative velocity vectors.3

### **Geometric Artifacts**

"Stitching" artifacts occur at the subduction front where the boundary of the overriding plate and the plunging plate do not perfectly align.3 The Cortial framework addresses this through "sample re-attachment," which effectively heals the gap by transferring ownership of the vertices at the interface.1 Additionally, "Z-fighting" between overlapping plates is resolved by the triangle exclusion logic mentioned in the thesis, ensuring only the top-most (overriding) crust is rendered.2

## **Conclusion and Morphological Integration**

The subduction model developed by Cortial et al. and refined in the Driftworld implementation provides a robust, geologically-inspired foundation for planetary synthesis. By treating subduction as a combination of distance-based procedural deformation and force-based kinematic simulation, the model captures the essential large-scale features of a tectonic planet.1  
This tectonic "scaffold" is not the end of the process but the beginning. The resulting elevation maps are used to guide orometry-based analysis to build Divide Trees, which in turn define the river networks and drainage basins that create the final, detailed terrain.5 The hyper-amplification of these tectonic features ensures that every peak and valley is hydrologically consistent with the planetary-scale forces that created them.8 Reimplementing this system requires not just a mastery of spherical geometry and compute shaders, but a deep understanding of the delicate balance between the forces that pull plates apart and the collisions that build worlds.1

#### **Works cited**

1. Procedural Tectonic Planets \- CNRS, accessed March 15, 2026, [https://perso.liris.cnrs.fr/eric.galin/Articles/2019-planets.pdf](https://perso.liris.cnrs.fr/eric.galin/Articles/2019-planets.pdf)  
2. Modélisation de terrains planétaires réalistes | PDF | Géomorphologie | Système d'information géographique \- Scribd, accessed March 15, 2026, [https://fr.scribd.com/document/659027540/These-Synthese-de-Terrain-a-l-Echelle-Planetaire](https://fr.scribd.com/document/659027540/These-Synthese-de-Terrain-a-l-Echelle-Planetaire)  
3. hecubah/driftworld-tectonics: A procedural planet creation project. \- GitHub, accessed March 15, 2026, [https://github.com/hecubah/driftworld-tectonics](https://github.com/hecubah/driftworld-tectonics)  
4. broad-band seismic stations: Topics by Science.gov, accessed March 15, 2026, [https://www.science.gov/topicpages/b/broad-band+seismic+stations](https://www.science.gov/topicpages/b/broad-band+seismic+stations)  
5. (PDF) Orometry-based Terrain Analysis and Synthesis \- ResearchGate, accessed March 15, 2026, [https://www.researchgate.net/publication/336823583\_Orometry-based\_Terrain\_Analysis\_and\_Synthesis](https://www.researchgate.net/publication/336823583_Orometry-based_Terrain_Analysis_and_Synthesis)  
6. accessed December 31, 1969, [https://github.com/hecubah/driftworld-tectonics/blob/master/Doc/driftworld-tectonics.pdf](https://github.com/hecubah/driftworld-tectonics/blob/master/Doc/driftworld-tectonics.pdf)  
7. Procedural Generation of Landscapes with Water Bodies Using Artificial Drainage Basins, accessed March 15, 2026, [https://www.researchgate.net/publication/366754119\_Procedural\_Generation\_of\_Landscapes\_with\_Water\_Bodies\_Using\_Artificial\_Drainage\_Basins](https://www.researchgate.net/publication/366754119_Procedural_Generation_of_Landscapes_with_Water_Bodies_Using_Artificial_Drainage_Basins)  
8. Real-Time Hyper-Amplification of Planets | Request PDF \- ResearchGate, accessed March 15, 2026, [https://www.researchgate.net/publication/343147863\_Real-Time\_Hyper-Amplification\_of\_Planets](https://www.researchgate.net/publication/343147863_Real-Time_Hyper-Amplification_of_Planets)