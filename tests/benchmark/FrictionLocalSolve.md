# Friction local solves

Build and run the internal local-solve benchmark:

```bash
pixi run config
pixi run cmake --build build/default/cpp/Release --target bm_friction_local_solve -j 32
pixi run ./build/default/cpp/Release/bin/bm_friction_local_solve \
  --benchmark_out=/tmp/friction-local-solve.json --benchmark_out_format=json
```

The eight cases cover interior, apex, boundary, and exact Coulomb solves for
ellipse and box cones. Each uses a positive definite, coupled 3-by-3 block and
known primal/dual contact data. Every timed result must be certified; the
impulse and certificate are protected from compiler elimination. The counters
report cone QPs and certified slow-path fallbacks per solve.

Times are nanoseconds per solve. The `median_call_ns` and `p99_call_ns` counters
come from 10,000 individual `steady_clock` samples, collected once per case
outside the Google Benchmark timed loop. They include timestamp overhead,
especially visible in the interior and apex cases; `clock_pair_median_ns`
reports a back-to-back timestamp measurement without subtracting it.

The median and custom `p99` aggregate rows also report quantiles of **51
repetition means**, each measured over a batch lasting at least 0.02 seconds.
These describe throughput variability across batches with no per-call timer.
The JSON includes both wall-clock and CPU times. Run on an idle machine and
record the compiler, CPU, and build configuration alongside results.

The ellipse boundary path uses an angle scan with Newton refinement and a
certified dense-scan fallback. This benchmark measures that production choice;
it does not compare against a secular implementation.

## Local measurement

On 2026-10-07, GCC 13.3.0 with `-O3 -DNDEBUG`, Google Benchmark 1.9.5 Release,
and an AMD Ryzen Threadripper 3970X produced the following results. All sampled
and timed solves were certified; no benchmark solve used the slow fallback.

| Case             | Individual median (ns) | Individual p99 (ns) | Batch CPU median (ns/solve) | QPs/solve |
| ---------------- | ---------------------: | ------------------: | --------------------------: | --------: |
| Ellipse interior |                    726 |                 994 |                         696 |         1 |
| Ellipse apex     |                    238 |                 308 |                         239 |         1 |
| Ellipse boundary |                 10,537 |              17,943 |                      10,617 |         1 |
| Ellipse exact    |                 69,591 |           3,093,978 |                      74,698 |         8 |
| Box interior     |                    507 |                 845 |                         460 |         1 |
| Box apex         |                    199 |                 259 |                         199 |         1 |
| Box boundary     |                  4,603 |               6,411 |                       4,840 |         1 |
| Box exact        |                 22,952 |              37,585 |                      22,107 |         6 |

The host was running other builds: load averages were 77.35, 85.68, and 63.59
on 64 logical CPUs, with frequency scaling enabled. The individual wall-clock
tails include scheduling delays, especially the ellipse exact case. Batch CPU
times are reported separately to distinguish throughput from those delays.
Back-to-back clock medians were 20–29 ns. These measurements are a local
observation, not a performance threshold.

## Certificate gate

Run the fixed-seed certificate gate with the full 100,000-problem count:

```bash
pixi run cmake --build build/default/cpp/Release --target test_FrictionCone -j 32
DART_FRICTION_GATE_COUNT=100000 pixi run \
  ./build/default/cpp/Release/tests/unit/constraint/test_FrictionCone
```

The default count keeps CI short. The test reports certified problems,
fast-path failures, and fallback counts. These detail blocks are not connected
to existing solvers, so this change has no simulation or visual behavior to
capture; its evidence is the numerical certificates, regression tests, and
local-solve timings.
