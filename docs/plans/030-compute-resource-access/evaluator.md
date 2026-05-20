# Compute Resource Access Evaluator

Use this evaluator when implementing the PLAN-030 resource-access metadata
milestone. Passing this evaluator is required before moving durable decisions
out of the active dev-task folder.

## Pass Condition

The milestone passes only when all of these are true:

- resource access metadata is descriptive and does not replace explicit graph
  dependencies as the correctness source of truth;
- read/read sharing, disjoint writes, same-resource hazards, scratch resources,
  and explicit reductions have focused tests;
- DOT output can expose resource access metadata without introducing a GUI or
  rendering dependency;
- sequential and Taskflow execution semantics stay unchanged for existing graph
  and world tests;
- the compute graph benchmark still builds and can run a focused smoke filter;
- no GPU, rendering, solver-registry, or classic `World` public API commitment
  is introduced.

## Required Local Evidence

Run the strongest subset that matches the touched files, and explain any
unavailable proof source.

```bash
pixi run lint
pixi run build
pixi run test-unit simulation --verbose
pixi run bm --target bm_compute_graph -- --benchmark_filter=BM_ComputeGraph --benchmark_min_time=1ms
```

If the implementation only touches docs or planning state, use the AI/docs gates
from `docs/ai/verification.md` instead.

## Focused Review Checklist

- Access modes include read, write, read-write or mutation, reduce, and scratch
  semantics.
- The validator reports hazards without mutating graph dependencies.
- Reduction metadata is explicit enough that multi-writer reductions are not
  silently treated as ordinary writes.
- DOT output remains optional and readable for larger graphs.
- Taskflow remains behind the experimental executor boundary.
- Existing classic simulation behavior is unaffected.

## Completion Report Shape

Record:

- files or surfaces touched;
- evaluator commands and outcomes;
- any skipped command with the exact blocker;
- remaining uncertainty, especially around scheduling, dependency inference, or
  future GPU backend decisions.
