# Benchmark Taxonomy

This file defines the benchmark families that the performance wave must cover.
It intentionally uses feature and algorithm language rather than naming
comparison-only implementations.

## Benchmark Manifest Rule

Do not maintain a hand-written list of every Google Benchmark case as the source
of truth. Generate the manifest from benchmark JSON and map each case into the
families below. The generated manifest for a given optimization slice should
record:

- benchmark name;
- feature/algorithm family;
- native timing distribution;
- strongest comparison timing distribution;
- comparison status: `lead`, `behind`, `non-comparable`, or `needs-rerun`;
- build type, compiler, CPU, thread policy, repetition count, and date.

## Required Families

### Pair Narrow Phase

Primitive, convex, mesh, compound, and field-backed pair queries. This family
owns analytic pair dispatch, generic convex algorithms, contact reduction,
normal/depth semantics, and pair-order determinism.

Acceptance: every comparable pair query leads the strongest comparison baseline
without weakening contact count, normal direction, signed/unclamped distance, or
determinism tests.

### Distance Queries

Closest-point and signed/unclamped distance paths for primitive, convex, mesh,
compound, and field-backed shapes.

Acceptance: overlap depth remains available through the signed/unclamped path,
public lower-bound clamping remains correct, and query cost leads the strongest
comparison baseline for each comparable shape family.

### Ray And Cast Queries

Closest-hit, all-hit, filtered, null-hit, batch-ray, sphere-cast, capsule-cast,
and shape-cast workloads.

Acceptance: ray/cast semantics match public DART contracts, filtering remains
correct, and batch workloads are measured separately from one-off query overhead.

### Broadphase And Pair Generation

Brute force, sweep-and-prune, tree, hash/grid, unbounded-shape routing, dynamic
updates, dirty transforms, pair filtering, and pair-cache invalidation.

Acceptance: update, pair-generation, and traversal costs are reported
separately; no finite-grid path hashes infinite bounds; large-scene benchmarks
lead comparable comparison baselines.

### Public Adapter And Package Path

`DartCollisionDetector` and `DartCollisionGroup` public routes, legacy C++
facades, package components, and installed-header dependency surfaces.

Acceptance: public adapter overhead is measured separately from native kernel
cost, compatibility facades remain native-backed, and package isolation checks
stay green.

### Mesh, Convex, And Field Heavy Scenes

Triangle-heavy scenes, convex fragment scenes, terrain/height fields, dense
field queries, and mixed primitive/mesh workloads.

Acceptance: build, update, traversal, and narrowphase costs are separated; cache
or warm-start optimizations show stable wins across representative scenes.

### Pipeline Breakdown

End-to-end query pipeline timing: shape adaptation, AABB update, broadphase,
narrowphase, result assembly, allocation, and public API overhead.

Acceptance: the first optimization slice is chosen from measured dominant cost,
and every claimed win can be attributed to a pipeline component.

### Scale And Variance

Object-count sweeps, dense/sparse distributions, edge cases, thin features,
grazing contacts, deep penetrations, and large-scene query batches.

Acceptance: reported wins survive repetitions and scene shape variation; rows
with high variance are not promoted to hard gates until the variance source is
understood.

## Future Families

### Multi-Core CPU

Pair queues, deterministic task scheduling, broadphase partitioning,
thread-local scratch, batch reductions, and non-regression against the optimized
single-core path.

### Single GPU

Large batch workloads with enough arithmetic intensity to justify transfer,
packaging, and maintenance cost: broadphase pair generation, batched ray/cast
queries, mesh/triangle traversal, and dense field queries.
