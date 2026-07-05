# DART 6 Simulation Allocation Results

## Run Context

Baseline artifacts are under `baseline/` and were captured on
2026-07-05T09:49:54Z at commit
`d557fc069feb22f60ecb713f02d396603ed513eb`.

Improved artifacts are under `improved/` and were captured on
2026-07-05T19:43:52Z at commit
`23ee2b618516bb196aad4ee1f893d6cbc70daa11` with the WP-D6M.5/WP-D6M.6
working tree changes present. The host was the same i9-13950HX Linux machine.
Both Google Benchmark captures warned that CPU scaling was enabled.

## Allocation Comparison

The strict allocation target is the native DART collision scene. External
Bullet/ODE backends are compatibility-gated on the World base allocator only,
because global/raw counters include backend-internal allocations.

| Scene / gate | Baseline operator-new / step | Final operator-new / step | Baseline raw malloc / step | Final raw malloc / step | Final World base alloc / step |
| --- | ---: | ---: | ---: | ---: | ---: |
| Native DART boxes, steady-state report | 5835.020 | 0.000 | 5862.020 | 0.000 | 0.000 |
| Native DART explicit first post-bake gate | N/A | 0.000 | N/A | 0.000 | 0.000 |
| Native DART implicit second-step gate | N/A | 0.000 | N/A | 0.000 | 0.000 |
| Bullet boxes, backend comparison | 5831.690 | 0.000 | 5859.530 | 0.840 | 0.000 |

The native DART collision path therefore drops from 5835.020 operator-new
allocations per measured step to zero and from 5862.020 raw malloc-family
allocations per measured step to zero.

## Timing Comparison

The full WP-D6M.2 Google Benchmark suite was rerun on the final tree and is
recorded in `improved/*.json`. These numbers are reported honestly, but they
are not a clean broad speedup claim: the boxes Google Benchmark advances one
world through google-benchmark-controlled automatic iterations, and the final
contact-container capture shows the cost of the implicit first-step prewarm in
short-lived worlds.

| Benchmark | Baseline ms/step | Final ms/step | Change | Baseline CV | Final CV |
| --- | ---: | ---: | ---: | ---: | ---: |
| `BM_RunBoxes/2` | 0.02466 | 0.04773 | +93.6% | 14.78% | 50.53% |
| `BM_RunBoxes/4` | 0.4403 | 0.6513 | +47.9% | 5.06% | 6.04% |
| `BM_RunBoxes/8` | 5.318 | 9.168 | +72.4% | 2.56% | 44.94% |
| `BM_ContactContainerActive/60/DART/1` | 1.827 | 4.640 | +154.0% | 1.50% | 9.79% |
| `BM_ContactContainerActive/60/DART/16` | 1.893 | 4.415 | +133.3% | 5.85% | 11.71% |
| `BM_ContactContainerActive/60/ODE/1` | 7.034 | 16.13 | +129.3% | 0.90% | 4.99% |
| `BM_ContactContainerActive/60/ODE/16` | 7.162 | 10.43 | +45.7% | 4.78% | 35.51% |
| `BM_ContactContainerActive/120/DART/1` | 21.67 | 25.92 | +19.6% | 1.84% | 1.01% |
| `BM_ContactContainerActive/120/DART/16` | 21.30 | 22.97 | +7.8% | 2.73% | 2.86% |
| `BM_ContactContainerActive/120/ODE/1` | 31.15 | 34.35 | +10.3% | 1.71% | 3.58% |
| `BM_ContactContainerActive/120/ODE/16` | 31.71 | 34.20 | +7.9% | 2.72% | 4.19% |

`boxes_headless` gives the cleanest same-scene hot-loop signal because it
constructs one world, runs exactly 2000 steps, prints determinism checkpoints,
and excludes world construction from the timer:

| Driver | Baseline elapsed | Final elapsed | Change | Baseline steps/s | Final steps/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| `boxes_headless 8 2000 500` | 9325.991 ms | 8147.949 ms | -12.6% | 214.5 | 245.5 |

The final `BM_RunBoxes/8` single-iteration sanity run under the same binary was
8511 ms for 2000 steps, consistent with `boxes_headless` and much lower than
the full google-benchmark aggregate for `BM_RunBoxes/8`. The aggregate boxes
rows should therefore be treated as stateful benchmark-harness evidence, not as
the primary 2000-step hot-loop comparison.

## Determinism

The final `boxes_headless 8 2000 500` checkpoints match
`baseline/boxes_headless.log` exactly. The step-checkpoint diff was empty.

## Performance Caveat

The zero-allocation contract is proven locally. The timing packet is mixed.
Two smaller implicit-preparation shortcuts were tested and rejected because
they weakened the hard allocation gates:

- Skipping solver prewarm on implicit first `step()` left the implicit second
  native step with 8 operator-new/raw-malloc allocations totaling 4080 bytes.
- Reducing `ConstraintSolver::prepareForSimulation()` from two passes to one
  left the explicit first post-bake native step with the same 8 allocations.

The current implementation keeps the strict contract. Before the PR advertises
a broad runtime speedup, the implicit first-step prewarm cost for short-lived
worlds should either be accepted as the cost of the strict first-post-bake
contract or replaced by an exact cheaper reservation strategy with the same
gates.
