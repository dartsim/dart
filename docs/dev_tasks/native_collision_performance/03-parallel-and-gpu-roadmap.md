# Parallel And GPU Roadmap

This file is future scope. Do not begin public API design here until the
single-core CPU wave has established a strong baseline and benchmark inventory.

## Multi-Core CPU Direction

Candidate workloads:

- broadphase pair generation for large scenes;
- batched narrowphase queues;
- batched ray/cast queries;
- mesh-heavy traversal;
- pipeline stages with independent object or pair work.

Design constraints:

- optimized single-core remains the fallback and non-regression baseline;
- output order and floating-point tolerances remain deterministic enough for
  tests and downstream consumers;
- thread-local scratch avoids allocator contention;
- scheduling, task graph, and worker-count policy stay internal until public
  control is justified;
- compatibility facades and package surfaces do not expose threading internals.

Prototype gate:

- prove speedup over optimized single-core on representative large workloads;
- prove no regression on small scenes where scheduling overhead dominates;
- record determinism and variance evidence;
- document memory overhead and allocator behavior.

## Single-GPU Direction

Candidate workloads:

- large batch broadphase pair generation;
- batched ray/cast queries;
- mesh or triangle traversal with high query counts;
- dense field queries;
- future data-parallel collision kernels where transfer cost is amortized.

Design constraints:

- CPU fallback is mandatory;
- transfer, synchronization, and build/update costs count in the benchmark;
- package and platform constraints are part of the gate, not an afterthought;
- no public stream, device-memory, or backend-selection API until the prototype
  wins on workloads that justify the maintenance cost;
- kernels should consume native data structures or a clearly documented
  transformation layer.

Prototype gate:

- select one workload family from measured single-core bottlenecks;
- implement an internal prototype without public API commitment;
- compare against optimized single-core and multi-core CPU candidates;
- record package feasibility and unsupported-platform behavior;
- decide whether the result warrants a design proposal.

## Do Not Start Yet

- public accelerator API;
- dependency or package commitments;
- permanent generated-kernel infrastructure;
- multi-backend abstraction before a winning workload is demonstrated.
