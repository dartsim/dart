# Performance Evidence: Contact-Aware Inverse Dynamics

Benchmark: `tests/benchmark/integration/bm_inverse_dynamics.cpp`
(`BM_INTEGRATION_inverse_dynamics`, Release, --benchmark_min_time=0.5s,
2026-06-11, 10-core x86_64 dev host under load — relative numbers matter).

## Recursive vs dense inverse dynamics (design justification)

| Links | RNE `computeInverseDynamics` | Dense `M*ddq + Cg` |
| ----- | ---------------------------- | ------------------- |
| 10    | 3.7 us                       | 10.2 us             |
| 20    | 7.6 us                       | 32.0 us             |
| 40    | 13.3 us                      | 97.7 us             |

Linear vs quadratic scaling: ContactInverseDynamics builds on the recursive
path.

## NNLS dual-tolerance optimization (loose pass + strict fallback)

`ContactInverseDynamics::compute()` on a 12-DOF floating biped; before =
strict NNLS tolerance always; after = dual tolerance matched to the residual
tolerance with strict re-solve fallback (commit on this branch):

| Case                  | Before  | After   | Speedup |
| --------------------- | ------- | ------- | ------- |
| 2 contacts x 4 basis  | 22.8 us | 18.6 us | 1.2x    |
| 4 contacts x 4 basis  | 54.9 us | 11.2 us | 4.9x    |
| 8 contacts x 4 basis  | 259 us  | 14.7 us | 18x     |
| 4 contacts x 8 basis  | 291 us  | 14.6 us | 20x     |
| 4 contacts x 16 basis | 1938 us | 27.8 us | 70x     |

The "after" numbers additionally include per-iteration position dirtying
(transform/Jacobian cache recompute, added after review), which the "before"
numbers did not pay; the speedup is therefore understated.

All 12 `UNIT_dynamics_ContactInverseDynamics` and 10
`UNIT_math_NonNegativeLeastSquares` tests pass before and after; the
forward-dynamics round-trip test pins solution accuracy.

Every contact-ID evaluation is now well under a 60 Hz GUI frame budget
(< 40 us), so the example can run the solver every frame.
