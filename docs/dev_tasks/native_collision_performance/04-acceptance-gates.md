# Acceptance Gates

## Objective Checklist

The performance wave is complete only when each explicit requirement below has
direct evidence.

| Requirement                   | Required evidence                                                                 |
| ----------------------------- | --------------------------------------------------------------------------------- |
| Feature-complete base         | Native collision feature matrix has no active feature gap rows.                   |
| All correctness cases covered | Persistent matrix maps every correctness row to tests or explicit rationale.      |
| All benchmark cases covered   | Generated benchmark manifest maps every collision benchmark into a family.        |
| Strongest-baseline rule       | Every comparable row leads the strongest comparison timing distribution.          |
| Single-core CPU now           | Primary evidence uses one collision worker thread and records environment detail. |
| Multi-core CPU future         | Roadmap and prototype gate exist; no premature public API is exposed.             |
| Single-GPU future             | Roadmap and prototype gate exist; no premature public API is exposed.             |
| No explicit baseline names    | Task docs use feature, algorithm, paper, or article language only.                |
| Compatibility preserved       | Runtime isolation and compatibility facade audits pass.                           |
| Correctness preserved         | Focused collision tests pass for every optimized family.                          |
| Durable handoff               | Performance dashboard/matrix promoted before this dev-task folder is deleted.     |

## Planning Gates

For edits limited to this task folder or plan links:

```bash
pixi run lint-md
pixi run check-lint-md
pixi run check-docs-policy
pixi run check-lint-spell
pixi run lint
```

## Implementation Gates

For code changes in the single-core wave:

```bash
pixi run lint
pixi run build
pixi run test-unit
pixi run bm-collision-check
```

Add targeted CTest or benchmark commands for the touched feature family. If a
benchmark row is not covered by `bm-collision-check`, add or document the
focused Google Benchmark invocation in the row evidence.

## Benchmark Evidence Rules

- Use JSON output for every claimed performance result.
- Compare distributions, not one-off timings.
- Record enough environment detail to reproduce the run.
- Mark non-comparable rows explicitly with feature-level rationale.
- Do not hide slower rows behind aggregate averages.
- Do not promote flaky benchmark thresholds to CI until variance is understood.

## Completion Audit

Before completing this dev task:

1. Restate the objective as single-core CPU performance leadership across every
   comparable DART-owned collision benchmark family, with future multi-core CPU
   and single-GPU roadmaps in place.
2. Map every row in the generated benchmark manifest to evidence.
3. Map every correctness row in the persistent matrix to test evidence or
   explicit rationale.
4. Verify correctness and compatibility gates cover every optimized surface.
5. Promote durable results into `docs/plans/` or `docs/onboarding/`.
6. Delete `docs/dev_tasks/native_collision_performance/` in the completing
   change.
