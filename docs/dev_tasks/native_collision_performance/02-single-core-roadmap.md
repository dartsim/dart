# Single-Core CPU Roadmap

The single-core wave optimizes the default CPU runtime before parallel or GPU
work. Every step must preserve correctness, determinism, compatibility facades,
and package isolation.

## Phase 1 — Baseline Reproducibility

Deliverables:

- benchmark manifest generated from current benchmark JSON;
- single-core run policy recorded in the manifest;
- row-level comparison table grouped by feature/algorithm family;
- `behind` rows sorted by total native time and relative gap.

Rules:

- Use release builds.
- Use one worker thread for benchmarked collision work.
- Record CPU, compiler, build options, repetition count, and metric.
- Treat high-variance rows as measurement tasks before optimization tasks.

## Phase 2 — Public Adapter And Allocation Cost

Hypothesis: visible end-to-end regressions often come from adaptation,
allocation, result assembly, or stale cache churn rather than narrowphase math.

Candidate work:

- avoid rebuilding native scene state for unchanged groups;
- reuse scratch/result buffers across query calls;
- remove unnecessary shape-frame adaptation work;
- keep public compatibility facades native-backed without extra dispatch layers;
- measure dirty-world and clean-world query paths separately.

Exit gate: adapter and pipeline breakdown rows identify remaining hot kernels
after avoidable overhead is removed.

## Phase 3 — Analytic Primitive Kernels

Hypothesis: primitive-pair workloads should not route through generic convex or
mesh fallbacks when an analytic path is faster and more stable.

Candidate work:

- specialize common pair kernels by feature family;
- reduce SAT and clipping overhead for box-like shapes;
- tighten capsule/cylinder cap/side dispatch;
- keep signed/unclamped distance and contact normal semantics tested beside the
  optimized code.

Exit gate: every comparable primitive collide/distance/raycast row leads the
strongest comparison baseline or is documented as non-comparable.

## Phase 4 — Broadphase And Dynamic Updates

Hypothesis: large-scene performance depends more on update and pair-generation
cost than on individual pair kernels.

Candidate work:

- separate static, sleeping, dirty-transform, and dirty-shape update paths;
- improve tree/hash/sweep update thresholds;
- add warm-start or front-list traversal only when repeated-query benchmarks
  justify the added complexity;
- keep unbounded-shape routing outside finite grid hashing.

Exit gate: broadphase update, pair generation, and traversal rows lead the
strongest comparison baselines across dense and sparse distributions.

## Phase 5 — Mesh, Convex, And Field Workloads

Hypothesis: heavy-shape scenarios need data-local traversal and cache reuse
without sacrificing deterministic output.

Candidate work:

- profile build/update/traversal/query separately;
- cache mesh adjacency, triangle bounds, convex support data, and field query
  scratch where ownership is stable;
- avoid layouts that block future SIMD or parallel traversal;
- add scene-level benchmarks when a microbenchmark win does not translate to
  end-to-end performance.

Exit gate: mesh-heavy, convex-heavy, terrain/field, and mixed-scene rows lead
the strongest comparable baselines or have a documented non-comparable reason.

## Phase 6 — Regression Guardrails

Candidate work:

- convert stable benchmark wins into lightweight guard checks;
- keep full benchmark JSON artifacts for release or PR evidence;
- document variance and environment assumptions;
- update the durable performance matrix before deleting this task folder.

Exit gate: new guardrails catch the optimized rows without making CI flaky.
