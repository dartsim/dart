# Native Collision Performance — Dev Task

## Current Status

- [x] Phase 0: Feature-complete native runtime is merged and available as the
      default collision path.
- [ ] Phase 1: Single-core CPU benchmark inventory and reproducibility gate.
      First manifest generated from the current collision benchmark JSON;
      final acceptance still needs a quieter controlled rerun.
- [ ] Phase 2: Single-core CPU optimization until every comparable benchmark
      family exceeds the strongest comparison baseline. The first public
      adapter edge-case slice now leads in the current noisy inventory run;
      raw pair narrow phase rows remain the largest measured gaps.
- [ ] Phase 3: Regression guardrails and release-ready evidence transfer.
- [ ] Phase 4: Multi-core CPU roadmap and prototype gate.
- [ ] Phase 5: Single-GPU roadmap and prototype gate.

## Goal

Make the native collision runtime the fastest single-core CPU path for every
DART-owned collision benchmark family while preserving feature completeness,
determinism, package isolation, and downstream compatibility. Future waves
should extend the same evidence discipline to multi-core CPU and single-GPU
execution without exposing premature public backend APIs.

## Language Rule

This task tracks performance by DART-owned feature, algorithm, benchmark family,
paper lineage, or article topic. Do not name comparison-only implementations in
this task folder. Treat baseline implementations as comparison data used only to
define comparable behavior and performance bars.

## Non-Goals

- No public multi-core or GPU API commitment during the single-core wave.
- No runtime fallback to comparison-only implementations.
- No benchmark-only shortcut that changes public DART collision semantics.
- No feature-completeness rework unless a performance investigation exposes a
  correctness or coverage gap.

## Key Decisions

- **Single-core first:** optimize the scalar/default CPU path before parallel
  scheduling or accelerator work, so later speedups are additive rather than
  hiding inefficient kernels.
- **Benchmark-family ownership:** DART owns the taxonomy in the persistent
  native collision matrix; this task turns deferred performance rows into
  executable optimization work.
- **Strongest-baseline rule:** each comparable benchmark row must beat the
  strongest comparison baseline for that feature/algorithm family, not only a
  selected baseline.
- **No named baseline targets:** use feature and algorithm language in planning
  docs so DART is optimizing toward its own quality bar.
- **Evidence before architecture:** multi-core CPU and single-GPU work need
  prototype benchmark evidence before public API, packaging, or dependency
  commitments.

## Workstreams

1. **Inventory and harness**
   - Generate a current benchmark manifest from Google Benchmark output and the
     persistent collision coverage matrix.
   - Classify every benchmark into a feature/algorithm family and comparable
     comparison-baseline set.
   - Verify every correctness row in the persistent matrix maps to an existing
     test, benchmark, non-comparable rationale, or deferred performance row.
   - Record single-core run policy: CPU isolation where available, thread count,
     build type, compiler, repetition count, JSON output path, and comparison
     metric.

2. **Single-core hot paths**
   - Remove avoidable allocations, virtual dispatch, shape adaptation churn, and
     cache misses from public adapter and native query paths.
   - Specialize primitive narrow phase, distance, raycast, cast, and contact
     reduction paths where analytic algorithms beat generic fallbacks.
   - Improve dynamic broadphase update, pair generation, and query pruning for
     mixed scenes and large object counts.
   - Keep unbounded-shape handling, signed/unclamped distance contracts, and
     deterministic result ordering intact.

3. **Mesh, convex, and field-heavy workloads**
   - Profile build, update, traversal, and query costs separately.
   - Add warm-start/front-list or cache reuse only when benchmark evidence shows
     stable improvement across representative scenes.
   - Avoid data layouts that block future SIMD or parallel traversal.

4. **Benchmark guardrails**
   - Convert performance wins into repeatable checks only after variance is
     understood.
   - Keep correctness tests adjacent to optimized paths so speed work does not
     weaken feature parity.
   - Store durable dashboards and acceptance evidence outside this task before
     completion.

5. **Future multi-core CPU**
   - Prototype deterministic pair scheduling, thread-local scratch, batched
     narrow-phase queues, broadphase partitioning, and reduction order.
   - Require single-core non-regression before enabling parallel execution.
   - Keep scheduler, thread count, allocator, and task graph details internal
     until API needs are proven.

6. **Future single-GPU**
   - Prototype only workloads with high arithmetic intensity or large batches:
     broadphase pair generation, batched ray queries, mesh/triangle traversal,
     and dense field queries.
   - Define CPU fallback, data transfer cost accounting, package constraints,
     and determinism expectations before choosing a public backend path.
   - Treat GPU support as an internal benchmark prototype until it beats the
     optimized CPU path on workloads that justify the maintenance cost.

## Immediate Next Steps

1. Use `05-benchmark-manifest.md` as the current row-level inventory and gap
   table.
2. Re-run the broad collision benchmark guard on a quieter controlled
   single-core configuration before using any result as final acceptance
   evidence.
3. Continue with the largest remaining measured raw pair narrow-phase gaps,
   then verify the targeted row and the broad collision guard.
4. Keep the manifest generated from JSON until durable dashboard data is
   promoted out of this task folder. Generate it with an explicit output path:
   `pixi run python scripts/generate_collision_benchmark_manifest.py --output docs/dev_tasks/native_collision_performance/05-benchmark-manifest.md`.

## Completion Criteria

The single-core wave is complete only when:

- every native collision correctness row has current test evidence or an
  explicit non-comparable/deferred rationale;
- every comparable benchmark family has a current JSON result;
- every row exceeds the strongest comparison baseline or is marked
  non-comparable with a feature-level rationale;
- correctness, package isolation, and compatibility gates still pass;
- benchmark evidence includes variance and environment details;
- the durable performance dashboard or matrix is promoted out of this
  `docs/dev_tasks/` folder; and
- `scripts/generate_collision_benchmark_manifest.py` is removed with the
  temporary task evidence or retargeted to a durable dashboard output; and
- this task folder is deleted in the completing change.

## Verification

Planning/docs changes:

```bash
pixi run lint-md
pixi run check-lint-md
pixi run check-docs-policy
pixi run check-lint-spell
pixi run lint
```

Implementation changes later in this task should use the relevant stronger
gates: focused collision tests, `pixi run build`, `pixi run test-unit`,
`pixi run bm-collision-check`, and targeted benchmark JSON comparisons for the
touched feature/algorithm family.

## Related

- `docs/plans/dashboard.md` — operating state for the active performance plan.
- `docs/plans/035-native-collision-dashboard.md` — durable feature/performance
  dashboard from the feature-complete pass.
- `docs/plans/035-native-collision/coverage-matrix.md` — persistent row-level
  coverage and deferred performance taxonomy.
- `dart/collision/AGENTS.md` — collision compatibility and testing guidance.
