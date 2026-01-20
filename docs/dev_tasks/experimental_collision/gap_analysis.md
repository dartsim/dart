# Gap Analysis: Experimental Collision vs Reference Backends

> **Last Updated**: 2026-01-20
> **Purpose**: Track implementation gaps against FCL, Bullet, and ODE superset
> **Goal**: Feature parity + performance wins before DART integration

---

## Executive Summary

The experimental collision module has **strong primitive coverage** but significant gaps in:

1. **Shapes**: Missing Cone, Ellipsoid, HeightField, Compound, Octree
2. **Collision pairs**: Missing Plane-Plane, some Cylinder pairs for specialized algorithms
3. **Distance queries**: Missing Cylinder, Plane distance; limited to primitives + Convex/Mesh via GJK
4. **Advanced features**: No collision filtering, no compound shapes, no persistent manifolds
5. **DART integration**: Not yet wired as a CollisionDetector backend

---

## 1. Shape Support

### Current Implementation

| Shape           | Experimental | FCL | Bullet | ODE     | Gap                    |
| --------------- | ------------ | --- | ------ | ------- | ---------------------- |
| Sphere          | ✅           | ✅  | ✅     | ✅      | None                   |
| Box             | ✅           | ✅  | ✅     | ✅      | None                   |
| Capsule         | ✅           | ✅  | ✅     | ✅      | None                   |
| Cylinder        | ✅           | ✅  | ✅     | ✅      | None                   |
| Plane/Halfspace | ✅           | ✅  | ✅     | ✅      | None                   |
| Convex Hull     | ✅           | ✅  | ✅     | ✅      | None                   |
| Triangle Mesh   | ✅           | ✅  | ✅     | ✅      | None                   |
| SDF             | ✅           | ❌  | ✅     | ❌      | **Ahead** of FCL/ODE   |
| Cone            | ❌           | ✅  | ✅     | ❌      | **Missing**            |
| Ellipsoid       | ❌           | ✅  | ❌     | ❌      | **Missing** (FCL only) |
| HeightField     | ❌           | ❌  | ✅     | ✅      | **Missing**            |
| Compound        | ❌           | ❌  | ✅     | partial | **Missing**            |
| Octree/Voxels   | ❌           | ✅  | ❌     | ❌      | **Missing** (FCL only) |
| 2D Shapes       | ❌           | ❌  | ✅     | ❌      | N/A (not needed)       |

### Priority Assessment

| Shape           | Priority | Rationale                                                          |
| --------------- | -------- | ------------------------------------------------------------------ |
| **Cone**        | Medium   | Used in some robotics (tool tips), but can approximate with convex |
| **HeightField** | Medium   | Terrain simulation; common in games, less in robotics              |
| **Compound**    | High     | Multi-part robots; currently must use multiple objects             |
| **Ellipsoid**   | Low      | Rare in robotics; can approximate with convex or scaled sphere     |
| **Octree**      | Low      | Mapping/planning use case; specialized                             |

---

## 2. Collision Query Support

### Collision Detection (Contact Generation)

| Shape Pair        | Experimental | FCL | Bullet | ODE | Notes                    |
| ----------------- | ------------ | --- | ------ | --- | ------------------------ |
| Sphere-Sphere     | ✅           | ✅  | ✅     | ✅  |                          |
| Sphere-Box        | ✅           | ✅  | ✅     | ✅  |                          |
| Sphere-Capsule    | ✅           | ✅  | ✅     | ✅  |                          |
| Sphere-Cylinder   | ✅           | ✅  | ✅     | ✅  |                          |
| Sphere-Plane      | ✅           | ✅  | ✅     | ✅  |                          |
| Box-Box           | ✅           | ✅  | ✅     | ✅  | SAT-based                |
| Box-Capsule       | ✅           | ✅  | ✅     | ✅  |                          |
| Box-Cylinder      | ✅           | ✅  | ✅     | ✅  |                          |
| Box-Plane         | ✅           | ✅  | ✅     | ✅  |                          |
| Capsule-Capsule   | ✅           | ✅  | ✅     | ✅  |                          |
| Capsule-Cylinder  | ✅           | ✅  | ✅     | ✅  |                          |
| Capsule-Plane     | ✅           | ✅  | ✅     | ✅  |                          |
| Cylinder-Cylinder | ✅           | ✅  | ✅     | ✅  |                          |
| Cylinder-Plane    | ✅           | ✅  | ✅     | ✅  |                          |
| Plane-Plane       | ❌           | ✅  | ✅     | ✅  | **Missing** (degenerate) |
| Convex-Convex     | ✅           | ✅  | ✅     | ✅  | GJK/EPA                  |
| Convex-Mesh       | ✅           | ✅  | ✅     | ✅  | GJK/EPA                  |
| Mesh-Mesh         | ✅           | ✅  | ✅     | ✅  | GJK/EPA (not optimal)    |
| Sphere-Convex     | ✅           | ✅  | ✅     | ✅  | Via GJK fallback         |
| Box-Convex        | ✅           | ✅  | ✅     | ✅  | Via GJK fallback         |
| Primitive-Mesh    | ✅           | ✅  | ✅     | ✅  | Via GJK fallback         |

**Collision Coverage: ~95%** - Missing only Plane-Plane (degenerate case, rarely needed).

### Distance Queries

| Shape Pair        | Experimental | FCL | Bullet | ODE | Notes              |
| ----------------- | ------------ | --- | ------ | --- | ------------------ |
| Sphere-Sphere     | ✅           | ✅  | ✅     | ❌  |                    |
| Sphere-Box        | ✅           | ✅  | ✅     | ❌  |                    |
| Sphere-Capsule    | ✅           | ✅  | ✅     | ❌  | Via Capsule-Sphere |
| Sphere-Cylinder   | ✅           | ✅  | ✅     | ❌  | GJK-based          |
| Sphere-Plane      | ✅           | ✅  | ✅     | ❌  | Support-function   |
| Box-Box           | ✅           | ✅  | ✅     | ❌  |                    |
| Box-Capsule       | ✅           | ✅  | ✅     | ❌  |                    |
| Box-Cylinder      | ✅           | ✅  | ✅     | ❌  | GJK-based          |
| Box-Plane         | ✅           | ✅  | ✅     | ❌  | Support-function   |
| Capsule-Capsule   | ✅           | ✅  | ✅     | ❌  |                    |
| Capsule-Cylinder  | ✅           | ✅  | ✅     | ❌  | GJK-based          |
| Capsule-Plane     | ✅           | ✅  | ✅     | ❌  | Support-function   |
| Cylinder-Cylinder | ✅           | ✅  | ✅     | ❌  | GJK-based          |
| Cylinder-Plane    | ✅           | ✅  | ✅     | ❌  | Support-function   |
| Convex-Convex     | ✅           | ✅  | ✅     | ❌  | GJK-based          |
| Convex-Mesh       | ✅           | ✅  | ✅     | ❌  | GJK-based          |
| Mesh-Mesh         | ✅           | ✅  | ✅     | ❌  | GJK-based          |
| Sphere-SDF        | ✅           | ❌  | ✅     | ❌  | **Ahead**          |
| Capsule-SDF       | ✅           | ❌  | ✅     | ❌  | **Ahead**          |

**Distance Coverage: ~85%** - Remaining gaps are plane-plane and SDF pairs beyond sphere/capsule.

### Raycast Queries

| Target Shape | Experimental | FCL | Bullet | ODE | Notes                  |
| ------------ | ------------ | --- | ------ | --- | ---------------------- |
| Sphere       | ✅           | ❌  | ✅     | ✅  | FCL has no ray API     |
| Box          | ✅           | ❌  | ✅     | ✅  |                        |
| Capsule      | ✅           | ❌  | ✅     | ✅  |                        |
| Cylinder     | ✅           | ❌  | ✅     | ✅  |                        |
| Plane        | ✅           | ❌  | ✅     | ✅  |                        |
| Convex       | ✅           | ❌  | ✅     | ✅  | GJK-based              |
| Mesh         | ✅           | ❌  | ✅     | ✅  | Moller-Trumbore        |
| HeightField  | ❌           | ❌  | ✅     | ✅  | **Missing** (no shape) |
| Octree       | ❌           | ❌  | ❌     | ❌  | N/A                    |

**Raycast Coverage: 100%** for implemented shapes. Ahead of FCL (no ray API).

### Continuous Collision Detection (CCD)

| Query Type                | Experimental | FCL     | Bullet | ODE | Notes         |
| ------------------------- | ------------ | ------- | ------ | --- | ------------- |
| Sphere-cast (all shapes)  | ✅           | ✅      | ✅     | ❌  | 7 targets     |
| Capsule-cast (all shapes) | ✅           | partial | ✅     | ❌  | 7 targets     |
| Conservative Advancement  | ✅           | ✅      | ✅     | ❌  | Convex-Convex |
| Box-cast                  | ❌           | partial | ✅     | ❌  | **Missing**   |
| General convex sweep      | ❌           | ✅      | ✅     | ❌  | **Missing**   |
| Mesh CCD                  | ❌           | ✅      | ✅     | ❌  | **Missing**   |

**CCD Coverage: ~70%** - Strong sphere/capsule cast; missing general convex sweep.

---

## 3. Broad-Phase Algorithms

| Algorithm               | Experimental | FCL | Bullet | ODE | Notes                  |
| ----------------------- | ------------ | --- | ------ | --- | ---------------------- |
| Brute Force             | ✅           | ✅  | ✅     | ✅  | O(n²) baseline         |
| AABB Tree (dynamic BVH) | ✅           | ✅  | ✅     | ❌  | SAH insertion          |
| Sweep-and-Prune         | ✅           | ✅  | ✅     | ✅  | Sorted endpoints       |
| Spatial Hash            | ✅           | ✅  | ❌     | ✅  | O(1) avg               |
| Interval Tree           | ❌           | ✅  | ❌     | ❌  | **Missing** (FCL only) |
| Quadtree/Octree space   | ❌           | ❌  | ❌     | ✅  | **Missing** (ODE only) |

**Broad-Phase Coverage: ~80%** - Good coverage of common algorithms.

### Broad-Phase Features

| Feature                | Experimental | FCL     | Bullet  | ODE | Notes         |
| ---------------------- | ------------ | ------- | ------- | --- | ------------- |
| Bulk build API         | ✅           | ❌      | partial | ❌  | **Ahead**     |
| Bulk update API        | ✅           | ❌      | partial | ❌  | **Ahead**     |
| Reusable pair buffer   | ✅           | ❌      | ✅      | ❌  | **Ahead**     |
| Filtered pair query    | ✅           | ✅      | ✅      | ✅  | With callback |
| Incremental update     | ✅           | ✅      | ✅      | ✅  | Fat AABBs     |
| Deterministic ordering | ✅           | partial | ❌      | ❌  | **Ahead**     |

---

## 4. Narrow-Phase Algorithms

| Algorithm              | Experimental | FCL | Bullet | ODE | Notes              |
| ---------------------- | ------------ | --- | ------ | --- | ------------------ |
| Specialized primitives | ✅           | ✅  | ✅     | ✅  | Per-pair optimized |
| GJK (intersection)     | ✅           | ✅  | ✅     | ✅  | Via libccd pattern |
| EPA (penetration)      | ✅           | ✅  | ✅     | ✅  |                    |
| MPR                    | ✅           | ❌  | ✅     | ❌  | Implemented        |
| SAT (box-box)          | ✅           | ✅  | ✅     | ✅  |                    |
| Polygon clipping       | partial      | ✅  | ✅     | ✅  | Box-box only       |
| BVH traversal          | ❌           | ✅  | ✅     | ✅  | **Missing**        |

**Narrow-Phase Coverage: ~75%** - Missing BVH traversal for mesh-mesh optimization.

---

## 5. Bounding Volume Types

FCL's strength is its rich set of bounding volume types for mesh collision. This enables trading off tightness vs query cost based on the use case.

### BV Type Comparison

| BV Type    | Experimental | FCL | Bullet | Coal | Parry | ODE | Notes                                      |
| ---------- | ------------ | --- | ------ | ---- | ----- | --- | ------------------------------------------ |
| **AABB**   | ✅           | ✅  | ✅     | ✅   | ✅    | ✅  | Axis-aligned; fast overlap test; loose fit |
| **OBB**    | ❌           | ✅  | ❌     | ✅   | ✅    | ❌  | Oriented; tighter fit; costlier test       |
| **RSS**    | ❌           | ✅  | ❌     | ✅   | ❌    | ❌  | Rectangle-swept sphere; good for distance  |
| **OBBRSS** | ❌           | ✅  | ❌     | ✅   | ❌    | ❌  | Hybrid OBB+RSS                             |
| **kDOP**   | ❌           | ✅  | ❌     | ✅   | ❌    | ❌  | k-discrete oriented polytope (k=16,18,24)  |
| **kIOS**   | ❌           | ✅  | ❌     | ✅   | ❌    | ❌  | k inner/outer spheres                      |
| **Sphere** | ❌           | ✅  | ✅     | ✅   | ✅    | ❌  | Simplest; very loose fit                   |

### BV Type Characteristics

| BV Type     | Tightness | Overlap Test         | Update Cost   | Memory   | Best Use Case                          |
| ----------- | --------- | -------------------- | ------------- | -------- | -------------------------------------- |
| **AABB**    | Poor      | O(1), 6 comparisons  | O(1)          | 24 bytes | Dynamic objects, broad-phase           |
| **OBB**     | Good      | O(1), ~15 ops        | O(n) vertices | 60 bytes | Static meshes, narrow elongated shapes |
| **RSS**     | Good      | O(1), moderate       | O(n) vertices | 52 bytes | Distance queries, robot links          |
| **OBBRSS**  | Very good | O(1), moderate       | O(n) vertices | 76 bytes | Mixed collision + distance             |
| **kDOP-16** | Moderate  | O(1), 16 comparisons | O(n) vertices | 64 bytes | General static meshes                  |
| **kDOP-24** | Good      | O(1), 24 comparisons | O(n) vertices | 96 bytes | Complex static meshes                  |
| **kIOS**    | Very good | O(k²)                | O(n) vertices | Variable | Highly concave shapes                  |
| **Sphere**  | Poor      | O(1), 1 comparison   | O(1)          | 16 bytes | Quick culling, simple shapes           |

### Current State in Experimental

The experimental module currently only supports **AABB** for:

- Broad-phase (AABB tree, spatial hash, sweep-and-prune)
- Shape bounding (via `Aabb` class)
- Mesh shape (AABB only, no BVH with alternative BV types)

### Gap Assessment

| Feature                    | Priority | Rationale                                                   |
| -------------------------- | -------- | ----------------------------------------------------------- |
| **OBB support**            | Medium   | Better fit for elongated robot links; FCL uses this heavily |
| **RSS support**            | Medium   | Enables efficient distance queries on meshes                |
| **BVH with selectable BV** | Low      | FCL's templated BVHModel; complexity vs benefit             |
| **kDOP support**           | Low      | Specialized; AABB/OBB cover most cases                      |

### Recommendation

1. **Phase 1**: Keep AABB-only for now; sufficient for most robotics use cases
2. **Phase 2**: Add OBB support for mesh BVH if performance on elongated shapes is poor
3. **Phase 3**: Consider RSS if distance query performance on meshes becomes a bottleneck

FCL's BV flexibility is powerful but adds significant template complexity. For experimental collision, prioritize **correctness and performance with AABB first**, then add OBB/RSS only if benchmarks show clear need.

---

## 6. Advanced Features

| Feature                            | Experimental | FCL | Bullet  | ODE     | Priority |
| ---------------------------------- | ------------ | --- | ------- | ------- | -------- |
| Collision filtering (groups/masks) | ❌           | ✅  | ✅      | ✅      | **High** |
| Collision callbacks                | partial      | ✅  | ✅      | ✅      | Medium   |
| Persistent manifolds               | ❌           | ❌  | ✅      | ❌      | **High** |
| Contact reduction                  | partial      | ❌  | ✅      | ❌      | Medium   |
| Security margins                   | ❌           | ❌  | ✅      | ❌      | Low      |
| Compound shapes                    | ❌           | ❌  | ✅      | partial | **High** |
| Multi-threading                    | ❌           | ❌  | partial | ❌      | **High** |
| Contact warm-starting              | ❌           | ❌  | ✅      | ❌      | Medium   |
| Tolerance verification             | ❌           | ✅  | ❌      | ❌      | Low      |

---

## 7. Special Features Survey

### 7.1 Articulated Body Support

**Question**: Should the collision library have special support for articulated bodies (robots)?

| Feature                   | FCL | Bullet      | Coal/HPP-FCL | Drake | DART Pattern              |
| ------------------------- | --- | ----------- | ------------ | ----- | ------------------------- |
| Self-collision filtering  | ❌  | partial     | ✅           | ✅    | Higher-level              |
| Adjacent link filtering   | ❌  | ❌          | ✅           | ✅    | `BodyNodeCollisionFilter` |
| Kinematic chain awareness | ❌  | btMultiBody | ✅           | ✅    | `dynamics` layer          |

**Assessment**: Articulated body support is typically a **physics engine concern**, not collision library.

- **DART's approach**: `BodyNodeCollisionFilter` handles adjacent body filtering at the dynamics layer
- **FCL/Bullet**: Provide generic collision filtering; articulated logic is external
- **Coal/HPP-FCL**: Added robotics-specific features (adjacent pair filtering, broadphase managers)
- **Drake**: Handles at SceneGraph level with collision filter groups

**Recommendation**: **Not needed in experimental collision module**

- Keep collision library generic (shapes, queries, filtering by ID/group)
- DART's existing `BodyNodeCollisionFilter` pattern works well
- If needed, add generic "pair blacklist" or "group-based filtering" (already planned)
- Priority: **N/A** (handled at higher level)

---

### 7.2 Differentiable Collision Detection

**Question**: Should experimental collision provide gradients for optimization?

| Feature                 | FCL | Bullet  | MuJoCo | Warp | DiffCo  | Experimental  |
| ----------------------- | --- | ------- | ------ | ---- | ------- | ------------- |
| Distance gradients      | ❌  | partial | ✅     | ✅   | ✅      | partial (SDF) |
| Penetration gradients   | ❌  | ❌      | ✅     | ✅   | ✅      | ❌            |
| Contact point Jacobians | ❌  | ❌      | ✅     | ✅   | partial | ❌            |
| Smoothed/randomized     | ❌  | ❌      | ❌     | ❌   | ✅      | ❌            |

**Current State in Experimental**:

- SDF shapes provide distance + gradient queries (via `SignedDistanceField`)
- GJK/EPA witness points can derive gradients (not explicitly exposed)
- No explicit gradient API for primitives

**Key References**:

- Montaut 2022: Randomized smoothing for stable gradients near feature switches
- Macklin 2020: SDF contact with local optimization
- MuJoCo: Analytical derivatives for all contacts

**Use Cases**:

- Trajectory optimization (CHOMP, TrajOpt)
- Differentiable simulation for learning
- Gradient-based motion planning

**Recommendation**: **Medium priority, phased approach**

1. **Phase 1**: Expose GJK/EPA witness-point-based gradients explicitly (low effort)
2. **Phase 2**: Add gradient API to distance queries: `distanceWithGradient()`
3. **Phase 3**: Consider smoothed collision (Montaut-style) for optimization stability

- Priority: **P2** (useful for research, not blocking integration)

---

### 7.3 Swept Volume / Shape Casting

**Question**: Beyond CCD (first impact), should we support full swept volume queries?

| Feature                  | Experimental | FCL     | Bullet | Parry   |
| ------------------------ | ------------ | ------- | ------ | ------- |
| Sphere cast              | ✅           | ✅      | ✅     | ✅      |
| Capsule cast             | ✅           | partial | ✅     | ✅      |
| Box cast                 | ❌           | partial | ✅     | ✅      |
| General convex cast      | ❌           | ✅      | ✅     | ✅      |
| Full swept volume        | ❌           | ❌      | ❌     | ❌      |
| Conservative advancement | ✅           | ✅      | ✅     | partial |

**Swept Volume vs CCD**:

- **CCD**: Find _first_ time of impact (what we have)
- **Swept Volume**: The _entire_ volume traced by motion (more expensive)

**Implementation Approaches**:

- Minkowski sum of shape + motion path (exact but expensive)
- Sampling-based approximation (practical)
- Conservative advancement (iterative, what we have)

**Use Cases**:

- Path planning: Check entire trajectory at once
- Safety verification: Prove no collision over motion
- Manufacturing: Tool path verification

**Assessment**: Full swept volume is rarely needed; CCD + dense sampling usually sufficient.

**Recommendation**: **Low priority**

- Current sphere/capsule cast covers most robotics needs
- Add box cast and general convex cast (P2)
- Full swept volume: defer unless specific use case arises
- Priority: **P2-P3**

---

### 7.4 Convex Decomposition

**Question**: Should experimental collision include convex decomposition?

| Tool        | Type                  | Quality | Speed  | Integration        |
| ----------- | --------------------- | ------- | ------ | ------------------ |
| **V-HACD**  | Volumetric            | Good    | Medium | Standalone library |
| **CoACD**   | Recent                | Better  | Medium | Standalone library |
| **HACD**    | Original              | Fair    | Fast   | Legacy             |
| **Bullet**  | btConvexDecomposition | Fair    | Fast   | Integrated         |
| **trimesh** | Python wrapper        | Varies  | Varies | V-HACD backend     |

**How It's Used**:

1. **Preprocessing**: Decompose mesh offline, store convex pieces
2. **Runtime**: Use compound shape or multiple collision objects
3. **Trade-off**: More pieces = better fit but more collision pairs

**Current State**:

- Experimental collision has `ConvexShape` and `MeshShape`
- No built-in decomposition
- Can use external tools (V-HACD, CoACD) and load result as compound

**Alternatives**:

- Use original mesh with BVH traversal (FCL approach)
- SDF-based collision for complex shapes
- Point cloud sampling

**Recommendation**: **Keep external, add compound shape support**

- Convex decomposition is a **preprocessing tool**, not runtime collision
- Many good standalone tools exist (V-HACD, CoACD)
- **Critical**: Add compound shape support (already in P0 gaps)
- Optionally add V-HACD integration as utility function
- Priority: **P3** (decomposition itself), **P0** (compound shapes to use results)

---

### 7.5 Summary: Special Features Priority

| Feature                           | Priority | Rationale                         |
| --------------------------------- | -------- | --------------------------------- |
| Articulated body filtering        | N/A      | Handled at DART dynamics layer    |
| Compound shapes                   | **P0**   | Required to use decomposed meshes |
| Distance gradients (explicit)     | **P2**   | Useful for optimization research  |
| Box/convex shape cast             | **P2**   | Complete CCD coverage             |
| Smoothed differentiable collision | **P3**   | Research feature                  |
| Full swept volume                 | **P3**   | Rarely needed; CCD sufficient     |
| Built-in convex decomposition     | **P3**   | Use external tools                |

---

## 8. DART Integration Status

| Component                                | Status         | Notes                    |
| ---------------------------------------- | -------------- | ------------------------ |
| CollisionDetector backend                | ❌ Not started | Required for integration |
| CollisionGroup support                   | ❌ Not started |                          |
| Shape adapters (dynamics → experimental) | ❌ Not started |                          |
| Integration tests                        | ❌ Not started |                          |
| Feature parity with FCL backend          | ❌ In progress |                          |
| Documentation                            | partial        | Dev docs only            |

---

## 9. Priority Roadmap

### Phase 1: Critical Gaps (Pre-Integration)

| Gap                     | Priority | Effort | Blocks                        |
| ----------------------- | -------- | ------ | ----------------------------- |
| Collision filtering     | **P0**   | Medium | Integration                   |
| Compound shapes         | **P0**   | High   | Multi-part robots             |
| Cylinder distance pairs | **P1**   | Low    | Distance query completeness   |
| Plane distance pairs    | **P1**   | Low    | Distance query completeness   |
| BVH traversal for mesh  | **P1**   | High   | Performance on complex meshes |

### Phase 2: Performance Parity

| Gap                   | Priority | Effort | Notes                       |
| --------------------- | -------- | ------ | --------------------------- |
| Parallel narrowphase  | **P0**   | High   | Per parallelization_plan.md |
| Persistent manifolds  | **P1**   | Medium | Stability improvement       |
| Contact warm-starting | **P2**   | Medium | Solver performance          |

### Phase 3: Feature Completeness

| Gap                      | Priority | Effort | Notes                   |
| ------------------------ | -------- | ------ | ----------------------- |
| Cone shape               | **P2**   | Low    | Convex fallback exists  |
| HeightField              | **P2**   | Medium | Terrain support         |
| General convex sweep CCD | **P2**   | Medium | Full CCD coverage       |
| Ellipsoid                | **P3**   | Low    | Rare use case           |
| Octree/voxels            | **P3**   | High   | Specialized mapping use |

### Phase 4: DART Integration

| Task                          | Priority | Effort | Notes                         |
| ----------------------------- | -------- | ------ | ----------------------------- |
| ExperimentalCollisionDetector | **P0**   | Medium | Backend interface             |
| Shape adapters                | **P0**   | Medium | dart::dynamics → experimental |
| Integration tests             | **P0**   | Low    | Reuse existing tests          |
| Performance validation        | **P0**   | Medium | Must beat/match FCL           |
| Documentation                 | **P1**   | Low    | API docs, examples            |

---

## 10. Metrics Summary

| Category          | Experimental | FCL   | Bullet | ODE  | Notes                                 |
| ----------------- | ------------ | ----- | ------ | ---- | ------------------------------------- |
| Shapes            | 8/11 (73%)   | 10/11 | 10/11  | 8/11 | Missing Cone, HeightField, Compound   |
| Collision pairs   | ~95%         | ~98%  | ~98%   | ~90% | Excellent                             |
| Distance pairs    | ~65%         | ~95%  | ~95%   | 0%   | Gap in Cylinder/Plane pairs           |
| Raycast           | 100%\*       | 0%    | 100%   | 100% | \*For implemented shapes              |
| CCD               | ~70%         | ~80%  | ~90%   | 0%   | Good sphere/capsule; no general sweep |
| Broad-phase       | ~80%         | ~90%  | ~70%   | ~80% | Strong coverage                       |
| BV types          | 1/7 (14%)    | 7/7   | 2/7    | 1/7  | AABB only; FCL excels here            |
| Advanced features | ~30%         | ~60%  | ~90%   | ~50% | Major gap                             |

### Overall Assessment

**Strengths:**

- Excellent primitive collision coverage
- Strong raycast support (better than FCL)
- Modern bulk APIs for ECS/batch processing
- Deterministic ordering guarantees
- SDF shape support (unique feature)

**Critical Gaps:**

1. **No collision filtering** - Blocks practical use
2. **No compound shapes** - Blocks multi-part robots
3. **No parallel narrowphase** - Performance limiter
4. **No DART integration** - Not usable as backend yet

**Recommendation:** Focus on collision filtering and compound shapes before DART integration. Parallel narrowphase can proceed in parallel as it's orthogonal to feature work.

---

## References

- [reference_comparison.md](./reference_comparison.md) - Detailed backend comparison
- [progress.md](./progress.md) - Implementation status tracker
- [parallelization_plan.md](./parallelization_plan.md) - Multi-threading roadmap
- [design.md](./design.md) - API design and architecture
