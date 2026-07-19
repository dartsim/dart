# FBF CPU evidence summary

Raw timings cover `World::step()` only. A realtime or paper-target verdict is emitted only when its corresponding evidence contract is valid. Blank target cells mean unevaluated, not failed.

A requested case/thread group with no measured rows remains visible and is marked incomplete; `--accept-nonzero` never turns a failed process into successful evidence.

| Scenario | Frontend | Threads | Affinity | Samples | Failed processes | Complete | Outcome | Controlled affinity | Mean ms | p95 ms | RTF | Realtime evaluated/met | Residual pass | Fallbacks | Paper ref ms | Paper evaluated/met |
| --- | --- | ---: | --- | ---: | ---: | --- | --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- |
| card_house_26_settle_projectile_full | native | 1 | 4 | 89 | 1 | False | None | True | 59.8234 | 74.3458 | 0.278598 | False/None | 0.000 | 0 | 199 | False/None |

## Physical outcome audit

| Scenario | Threads | Valid | Reasons | Details |
| --- | ---: | --- | --- | --- |
| card_house_26_settle_projectile_full | 1 | None | physical_outcome_contract_unavailable | no fixture-derived trajectory outcome contract |

Contract rejection reasons are preserved in `summary.csv` and `summary.json`. Current multi-thread trace labels identify a threaded world with a serial FBF island solve, so their raw speedups are not multicore-solver claims.

Schema v5 preserves all 60 v4 trace fields and appends the resolved paper-CPU solver knobs. The paper's local block formula, gamma-recovery rule, and DART split-impulse equivalence are not published, so those gaps keep an apples-to-apples paper verdict unevaluated.

See `metadata.json`, `invocations.json`, `raw.csv`, and the per-process files under `raw/` for the complete evidence chain.
