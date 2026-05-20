# Resume: Compute Resource Access

## Last Session Summary

The first compute-graph milestone was split into its own PR. This new dev task
tracks the next PR: resource read/write metadata for compute nodes, with
validation and visualization before any automatic dependency inference.

## Current Branch

`feature/experimental-compute-graph` — this task is currently a planning
handoff for the follow-up branch or PR.

## Immediate Next Step

Start a follow-up branch from the merged compute-graph PR, then implement the
resource-access metadata layer in `dart/simulation/experimental/compute/`.

## Context That Would Be Lost

- Explicit graph edges remain the correctness source of truth for the first
  resource-access PR.
- Access metadata should improve diagnostics, DOT output, profiling context,
  and future graph-shaping decisions.
- Dependency inference should wait until validation tests cover read/write,
  write/write, read-write, scratch, and reduction cases.
- The first resource identifiers can be stable strings; typed ECS/component IDs
  can follow only if needed.
- Do not add GPU residency, stream, memory-transfer, collision, or constraint
  APIs in this PR.

## How to Resume

```bash
git checkout feature/experimental-compute-graph
git status && git log -3 --oneline
```

Then inspect `docs/dev_tasks/compute_resource_access/README.md` and implement
the first metadata/validation slice.
