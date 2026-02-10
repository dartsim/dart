# Resume: Multi-Core CPU Execution

## Last Session Summary

Added comprehensive tests and real-world benchmarks proving the compute graph
feature is correct and beneficial. 18 unit tests pass (including 200-step
multi-DOF correctness, edge cases, TaskflowExecutor). Benchmarks show 2–5x
speedup for 32+ skeleton scenarios with batching.

## Current Branch

`feature/multi_core` — 8 commits ahead of main

## Immediate Next Step

Create PR to merge Phase 1+2 to main. All evidence gathered:

- 18/18 unit tests pass (correctness proven)
- Benchmarks show clear benefit with batching (2–5x for 32+ skeletons)
- No regressions in broader test suite (146/146 actual tests pass)
- Feature is opt-in (disabled by default via `GraphExecutionConfig`)

## Context That Would Be Lost

- Batching is critical — without it, Taskflow overhead dominates
- Crossover: ~32 skeletons with auto-batch; batch=4+ gives 2–5x at 32+ skeletons
- 12 "failed" tests in broader suite are `simulation-experimental` (Not Run)
- Multi-step test proves bit-exact match over 200 steps with 3 multi-DOF skeletons

## How to Resume

```bash
git checkout feature/multi_core
git status && git log -3 --oneline

# Verify build + tests
pixi run build
ctest --test-dir build/default/cpp/Release -R compute_graph -V

# Run benchmarks
./build/default/cpp/Release/bin/bm_compute_graph --benchmark_filter="BM_WorldStep" --benchmark_min_time=0.5s
```

Then: Create PR to main with benchmark results as evidence.

## Key Files to Load

```
@docs/dev_tasks/multi_core_cpu/README.md      # Status, architecture, usage
@docs/dev_tasks/multi_core_cpu/01_progress.md  # Benchmark results, test coverage
```

## Architecture Quick Reference

```
World::step()
    └── WorldStepGraph::step()
            └── GraphExecutor::execute(ComputeGraph)
                    ├── SequentialExecutor (single-threaded, deterministic)
                    └── TaskflowExecutor (parallel, Taskflow backend)

Graph structure per step:
  [BatchForwardDynamics x N] --> [ConstraintSolve] --> [BatchIntegratePositions x N] --> [TimeAdvance]
```
