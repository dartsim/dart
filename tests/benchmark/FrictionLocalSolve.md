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

The ellipse boundary fast path uses a two-branch secular solve. With
`D = diag(1, mu1, mu2)`, it solves the boundary KKT system in `D*H*D`
coordinates using a 2-by-2 tangential Schur complement. The sign of the
numerator at the positive generalized eigenvalue selects the branch below or
above that pole; the singular pole case is handled separately. It never forms
or inverts a matrix scaled by `1/mu^2`. The relative `1e-10` KKT certificate,
counted dense angle scan, and bisection fallback remain unchanged.

## Instruction-count comparison

On 2026-10-07, GCC 13.3.0 with `-O3 -DNDEBUG`, Google Benchmark 1.9.5 Release,
and Valgrind 3.22.0 on an x86-64 AMD Ryzen Threadripper 3970X produced the
following Callgrind instruction counts (`Ir`). The angle variant is the
32-angle scan with Newton refinement from `9a3517a6634`; both variants include
the opening-contact shortcut and the same Callgrind driver. Each row uses
1,000 certified solves with zero fallbacks. All solver arithmetic is `double`.

| Case                  | Angle instructions/solve | Secular instructions/solve | QPs/solve |
| --------------------- | -----------------------: | -------------------------: | --------: |
| Ellipse interior      |                    1,621 |                      1,629 |         1 |
| Ellipse apex          |                      712 |                        712 |         1 |
| Ellipse boundary      |                   12,073 |                      3,598 |         1 |
| Ellipse exact         |                   82,577 |                     23,993 |         8 |
| Box interior          |                    1,330 |                      1,338 |         1 |
| Box apex              |                      661 |                        661 |         1 |
| Box boundary          |                    8,841 |                      8,866 |         1 |
| Box exact             |                   42,653 |                     42,778 |         6 |
| Ellipse exact opening |                    1,131 |                      1,131 |         0 |
| Box exact opening     |                      884 |                        884 |         0 |

The secular path wins: 8,475 fewer instructions per ellipse boundary solve
(70.2%, or a 3.36-to-1 instruction-count ratio) and 58,584 fewer per exact
solve (70.9%, or 3.44 to 1). It is the production fast path; the coarse Newton
angle path has been removed. The angle code retained in production serves only
the certified dense-scan fallback. The box algorithm is identical in both
variants; small count differences come from compiling the shared solver with
the different ellipse paths.

Reproduce a production measurement after building the benchmark:

```bash
valgrind --tool=callgrind --instr-atstart=no \
  --callgrind-out-file=/tmp/callgrind.friction.ellipse_boundary \
  ./build/default/cpp/Release/bin/bm_friction_local_solve \
  --callgrind ellipse_boundary
callgrind_annotate --inclusive=yes --threshold=100 --auto=no \
  /tmp/callgrind.friction.ellipse_boundary
```

Use the table's names in lower case with underscores, such as `ellipse_exact`
or `box_exact_opening`. This mode requires `valgrind/callgrind.h` at build time
and performs 1,000 solves per invocation. Instrumentation excludes problem
setup, warmup, allocation, timers, and post-loop result validation. Divide the
inclusive `Ir` of `solveConeQp` or `solveExactContact` by 1,000 to exclude the
loop and harness overhead. The driver reports certification, QPs, and counted
fallbacks for every case.

The machine is heavily loaded by other builds. Wall times must be re-measured
on a quiet host; these instruction counts select the production fast path and
do not establish a wall-time speedup.

## Certificate gate

Run the fixed-seed certificate gate with the full 100,000-problem count:

```bash
pixi run cmake --build build/default/cpp/Release --target test_FrictionCone -j 32
DART_FRICTION_GATE_COUNT=100000 pixi run \
  ./build/default/cpp/Release/tests/unit/constraint/test_FrictionCone
```

The default count keeps CI short. The test reports certified problems,
fast-path failures, and fallback counts. With the chosen secular path and
`DART_FRICTION_GATE_COUNT=100000`, all 100,000 problems were certified:
5 fast-path failures (0.005%), 5 counted fallbacks, and a worst independent
relative certificate of `9.38623e-11` against the unchanged `1e-10` threshold.
Regression tests cover both secular branches, the singular pole, opening
contacts with zero QPs, and a counted fallback from the gate's case 14806.
The previous angle-specific fallback case 1481 remains a certificate regression.

These detail blocks are not connected to existing solvers, so this change has
no simulation or visual behavior to
capture; its evidence is the numerical certificates, regression tests, and
local-solve instruction counts.
