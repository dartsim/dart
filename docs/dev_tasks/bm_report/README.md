# Benchmark Reports

## Status

- Implemented: local report generation, PR comparisons, and dashboard publishing.

## Goals

- Let contributors run benchmarks on stable hardware and commit JSON reports.
- Compare PR reports against the base branch for the same benchmark + runner.
- Publish time-series graphs from committed reports.

## Decisions

- Reports live in `benchmarks/reports/` and are committed alongside code.
- Reports use Google Benchmark JSON; metadata is recorded under `context.dart_bench`.
- Raw hostnames and executable paths are removed from stored reports; use a non-sensitive runner id instead.
- CI compares only matching runner ids to avoid cross-machine noise.

## Next Steps

- Collect initial reports from a stable runner and confirm charts look sane.

## Pointers

- Local wrapper: `pixi run bm-report`.
- CI workflows: see the benchmark report workflows under `.github/workflows/`.
- Resume prompt: `docs/dev_tasks/bm_report/resume-prompt.md`.
