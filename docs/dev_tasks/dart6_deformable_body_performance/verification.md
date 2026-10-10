# Deformable-body verification recipes

Historical exact-revision evidence belongs to merged PRs #3382, #3408, and
#3423 and their hosted checks. No incomplete local run establishes a detector
ranking. The 6.21 lane must record exact revision SHAs, commands, raw rows,
host state, eligibility, and results for each acceptance item.

## Focused numerical gates

Run `pixi run build` and focused soft-body integration tests for dynamics
changes. `test_SoftDynamics` covers representative equations, finite state,
thread determinism, energy, and contact-force/CoP. Soft-body heap and
base-allocator checks live in `INTEGRATION_StepAllocation`; after building,
run them from the repository root:

```bash
GTEST_FILTER='StepAllocation.*Soft*' pixi run ctest \
  --test-dir build/default/cpp/Release -R '^INTEGRATION_StepAllocation$' \
  --output-on-failure --no-tests=error
```

GUI-free model tests cover `AdaptiveSoftContact`, `SoftWorm`, and
`SoftFootSimbicon`.
`test_SoftFootSimbiconPushSweep` is a single-trajectory measurement, not robust
push-recovery parity; #3431's counter-evidence requires phase/noise-aware
verification before that row closes.

## Benchmark recipes

```bash
pixi run cmake --build build/default/cpp/Release --target BM_INTEGRATION_soft_body --parallel 8
pixi run bm-soft-body -- --benchmark_filter=BM_SoftBodyStep/.* --benchmark_min_time=0.01s
pixi run bm-soft-body-paired --output-dir build/soft-body-paired
```

The paired protocol needs `COMPLETE.json` and every required raw row before a
verdict is valid. See [profiling](../../onboarding/profiling.md#soft-body-benchmarks)
for balanced comparisons and eligibility. Record single-thread and host-capped
multi-thread behavior and timing; a manual disposition does not convert FAIL
to PASS.

## GUI and downstream evidence

Use the [simulation verification route](../../ai/verification.md#simulation-verification-route)
for behavior-bearing scenes, pairing a numerical oracle with assessed visuals.
The soft-foot capture recipe is:

```bash
DART_DEMO_SOFT_FOOT_FEET=soft DART_DEMO_SOFT_FOOT_PUSH_STEP=650 \
DART_DEMO_SOFT_FOOT_PUSH_N=6000 \
  xvfb-run -a -s '-screen 0 1280x1024x24' \
  ./build/default/cpp/Release/bin/dart-demos --scene soft_foot_simbicon \
  --headless --steps 2250 --shot end.png
```

Use `rigid` for the matched control. Headless capture still needs an X server;
with a display, omit the wrapper. Do not infer correctness from a still alone.
Run `pixi run -e gazebo test-gz` for downstream-sensitive collision or solver
changes, plus the pre-default gates in the design owner. Run `pixi run lint`
before each commit.
