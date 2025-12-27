# LCP Solver Comparison Report (Template)

Fill this template when adding or evaluating a solver.

## Summary

- Solver name:
- Commit / branch:
- Build type and flags:
- Machine (CPU, cores, OS):
- Notes on determinism (seed, randomized ordering):

## Contract Settings

- `maxIterations`:
- `absoluteTolerance`:
- `relativeTolerance`:
- `complementarityTolerance`:
- `relaxation` (if applicable):
- Warm start enabled:

## Correctness (Tests)

| Suite | Result | Notes |
| --- | --- | --- |
| LCP comparison harness | PASS/FAIL | |
| LCP edge cases | PASS/FAIL | |
| LCP stress tests | PASS/FAIL | |

## Benchmark Results (Release Build)

Include JSON/CSV artifact path or command used.

### Standard SPD

| Size | Solver | Time | Iterations | Residual | Complementarity | Contract OK |
| --- | --- | --- | --- | --- | --- | --- |
| 12 | | | | | | |
| 24 | | | | | | |
| 48 | | | | | | |
| 96 | | | | | | |

### Boxed (Active Bounds)

| Size | Solver | Time | Iterations | Residual | Complementarity | Contract OK |
| --- | --- | --- | --- | --- | --- | --- |
| 12 | | | | | | |
| 24 | | | | | | |
| 48 | | | | | | |

### Friction Index

| Contacts | Solver | Time | Iterations | Residual | Complementarity | Contract OK |
| --- | --- | --- | --- | --- | --- | --- |
| 4 | | | | | | |
| 16 | | | | | | |
| 64 | | | | | | |

### Scaling

| Size | Scale | Solver | Time | Iterations | Residual | Complementarity | Contract OK |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 12 | 1e-6 | | | | | | |
| 12 | 1e6 | | | | | | |

## Notes and Follow-ups

- Regressions or anomalies:
- Any solver-specific tuning applied:
- Next action items:
