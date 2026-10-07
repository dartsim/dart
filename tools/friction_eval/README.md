# Friction-solver evaluation harness

Measures DART's contact-friction backends on analytic (L2), robustness (L3)
and performance (P1) scenes and on frozen problems (L1), and prototypes the two
Dantzig-internal candidates for the 6.20 default outside the library:

- **DZ+R** (`--solver dzr`): Dantzig, then the box law re-solved from
  Dantzig's impulses with the friction bounds refreshed every sweep. Dantzig
  freezes them at the frictionless normal impulses.
- **VA** (`--va`): a contact-surface handler that aligns the first friction
  axis with the free slip velocity of isotropic contacts.

Nothing here changes `dart/`. Only T0 runs in CI, as
`tests/integration/test_FrictionAnalytic.cpp`, which shares the scenes.

| File | Role |
| --- | --- |
| `friction_scenes.hpp` | Scenes with exact-Coulomb (`pred_exact_*`) and box-law (`pred_box_*`) references |
| `friction_eval.cpp` | One cell, a bisection, or the L1 bank; the telemetry wrapper and problem dumps, PGS-tight, DZ+R, VA, the law audit; long CSV |
| `friction_eval.py` | E1 matrix runner, report tables and scores (stdlib only) |
| `CMakeLists.txt` | Standalone `find_package(DART)` build, for DART 6.19.4 and the 6.20 line |

## Build and check

```bash
pixi run -e gazebo bash -c 'cmake -G Ninja -S tools/friction_eval -B build/fe \
  -DCMAKE_PREFIX_PATH="<DART install prefix>;$CONDA_PREFIX" && cmake --build build/fe'
build/fe/friction_eval --self-test
python3 tools/friction_eval/friction_eval.py --self-test --bin build/fe/friction_eval
```

The 6.20 line still reports version 6.19.4, so the code detects it by a header
it added (`FRICTION_EVAL_DART620`). Split impulse, threads, per-pair contact
caps, the matrix-free path (`--solver mf-pgs`) and the convex-wedge arches
(C4, R6) need it.

## Run

```bash
friction_eval --scene A4 --param phi=45,k=1.2 --solver dantzig --detector ode
friction_eval --scene C2 --param muw=0.5 --bisect alpha=20:70:slid --solver dzr
friction_eval --scene R5 --dump DIR --dump-steps 100,500   # L1 problem dumps
friction_eval --l1 [DIR/*.lcp]   # L1 bank: built-in families, or the given dumps
python3 tools/friction_eval/friction_eval.py run --bin B619=PATH --bin B620=PATH --out DIR -j 8
python3 tools/friction_eval/friction_eval.py report DIR > summary.md
```

Options: `--solver dantzig|pgs|pgs100|pgs-tight|dzr|mf-pgs`,
`--detector ode|dart|fcl|bullet` (FCL with its analytic primitives),
`--dt`, `--erp`, `--cfm`, `--max-erv`, `--deactivation on|off` (default off),
`--split on|off`, `--threads`, `--max-contacts`, `--max-contacts-per-pair`,
`--label`, and `--perf`, which drops the wrapper and the audit for timing.
`friction_eval --list` prints the scene ids: A1-A8 and A10-A13 analytic, C1,
C2, C4 and C5 coupled thresholds, R1-R3, R5, R6 and R9 robustness, and P1
(contact_benchmark's generated objects). C4 and R6, the masonry arches, need
the 6.20 line. The design's A9, C3 and R4 are not implemented; its R7 (dt) and
R8 (mu edge cases) are runner sweeps over other scenes. One process runs one
cell, because ERP, CFM and ERV are process-wide. The runner fails a cell that
exits nonzero, reports a non-finite state or a failed measurement (below), or
lacks its result rows, and writes its rows to `failed.csv` instead of
`cells.csv`, so they reach no table; the
accuracy score counts a failed cell as measuring nothing (-1 for each
measurement B620 made). It runs the wall-clock cells (`--perf` without `--ir`)
alone after the parallel phase, and skips the Callgrind cells (`--ir`) when
Valgrind is not installed. If every selected cell is skipped, the runner exits
with status 2 and prints the skip reasons, just as an empty selection fails.
`report` lists skipped and failed cells.

## Output

Each cell prints `label,dart,scene,params,solver,detector,dt,split,deactivation,metric,value`:

- scene metrics next to their `pred_exact_*` and `pred_box_*` references;
- the per-contact law audit from public API: `cone_viol_max`,
  `slip_dir_err_*_deg`, `stick_slip_max`, `dilatancy_mean`, `un_min`, and
  `fl_eligible_frac`, the share of root steps the 6.20 World's drift
  suppression could edit;
- solver telemetry from the wrapper: `solves`, `primary_failures` (a
  non-finite result counts, as it does for DART), `fallbacks`,
  the box-law residual of the applied impulses (`nat_res_max` in m/s,
  `box_viol_max` relative to the final normal impulse, `cfm_floor_max`), and
  `tight_*` and `dzr_refreshed` for PGS-tight and DZ+R;
- `finite`, `contacts_*`, `energy_rise_max`, `wall_ms_per_step` and
  `state_hash`;
- `started`, printed first and flushed, so a cell that crashes still leaves its
  key.

A metric that is unavailable is usually omitted; older harnesses, including
E1's, emitted NaN instead. The runner accepts those documented no-value markers
and rejects other NaNs. A10 must emit a finite `v_err`: without a final contact,
its contact-frame velocity measurement failed even if the state is finite.
All infinite metric values fail a cell. The infinities used internally for A4's
axis bounds and L1's unbounded constraints are not intended output markers.
`report` never treats a NaN as equal, and counts a metric that only one side
reports as a difference.

The metric classification from `friction_scenes.hpp` and `friction_eval.cpp` is:

| Scene / source | Metric | Unavailable or non-finite condition | Verdict |
| --- | --- | --- | --- |
| All scenes, law audit | `slip_dir_err_*_deg`, `dilatancy_mean` | No qualifying sliding contacts; moving surfaces are excluded | No value; NaN allowed |
| A3 | `onset_deg` | Sliding onset never reached | No value; NaN allowed |
| A4, A12 | `force_ratio`, `dir_err_deg` | No loaded sliding samples | No value; NaN allowed |
| A5 | `stop_time`, `creep` | Stop not reached, or no interval after stop | No value; NaN allowed |
| A5 | `dist_ratio` | Older stopping-distance reference at `mu=0`, or zero launch speed | No reference; NaN allowed only at `mu=0` or `v0=0` |
| A6 | `alpha_ratio_*`, `box_err_max` | No samples with spin above 2 rad/s and positive reference torque | No value; NaN allowed |
| A7 | `roll_step` | Rolling not reached | No value; NaN allowed |
| A7 | `v_roll_err` | Conserved reference `pred_v_roll` is zero | No relative-error reference; NaN allowed only when the reference is zero |
| A8 | `speed_loss_per_m`, `vz_rms` | Invalid normalization or arithmetic despite a finite state | Measurement failed |
| A10 | `sync_time` | Belt synchronization never reached | No value; NaN allowed |
| A10 | `v_err` | No contact in the final collision result | Measurement failed; row required and finite |
| A13 | `v_steady`, `v_err` | Non-finite average or error | Measurement failed |
| C1 | `front_share` | No loaded, untipped sliding samples | No value; NaN allowed |
| C4, R6 | `collapse_time` | Arch never collapsed | No value; NaN allowed |
| R1, R3 | `rest_time` | Stack never rested | No value; NaN allowed |
| R5 | `standing_1s` | Horizon ends before 1 s observation | No value; NaN allowed |
| R9 | `yaw_rate` | No observation interval after 2 s spin-up | No value; NaN allowed only at `T <= 2` |
| Bisection (A4, A11, A12, C2) | `threshold` | Endpoints agree, so threshold is not bracketed | No value; NaN allowed only when `at_lo == at_hi`; otherwise required and finite |
| All predictions | `pred_*` | No applicable prediction (e.g. no predicted slide, C1 tipped regime, old A5 `mu=0` reference) | No reference; NaN allowed |
| Solver telemetry | Means, residuals, `tight_*`, `va_aligned_frac` | No solves, audited solves, or handler calls | No value; rows omitted; emitted non-finite values are failures |
| All other emitted metrics, including `steps`, `at_lo`, `at_hi` | Numeric metric values | Non-finite arithmetic or invalid normalization | Measurement failed |

No scene intentionally emits infinity as a metric. Zero denominators in custom
parameter runs (e.g. A4 `mu=0`, A7 zero friction, A10 zero friction, A11
zero friction) or overflow can produce infinity; those outputs fail rather
than masquerading as successful measurements. A5's current distance reference
is over the finite simulation horizon, so it is finite at `mu=0` for a nonzero
launch speed; the allowance preserves older E1 output.

Auditing E1's `cells.csv` finds 3,566 NaN values, no infinities, and no new
measurement failures under this rule; all 84 A10 cells have finite `v_err`.
The older E1 bisection format lacks `finite` and `at_hi` in 868 cells, which
already fail the current runner's row-presence checks. Their metric values
also pass the new rule when the missing structural metadata is supplied for
the audit (endpoint agreement inferred from the old NaN threshold marker).

R1 and R3 report `rest_time` at the end of the first 0.1 s interval during
which every mobile body's linear speed is below 0.001 m/s and angular speed
is below 0.001 rad/s, independent of deactivation. A4's `pred_exact_accel` is
reported only for isotropic friction (`mu2 == mu`); the anisotropic static
ellipse reference remains available as `pred_exact_cap`.

PGS-tight and DZ+R stop on the box-law residual (at most 1e-6 m/s, 10-sweep
chunks of one-sweep calls, 1000-sweep cap); PGS's own relative-change test stops
early on stacks and never passes on rows near zero.

## Limits

- The wrapper is a custom backend type, so the 6.20 line solves islands
  serially with it. Use `--perf` for timing rows.
- VA is a user handler, which disables the default handler's fast paths, so
  VA rows are physics-only.
- With `--solver mf-pgs`, the 6.20 line solves large contact-only groups of
  free bodies matrix-free, past the wrapper, so the solver telemetry covers
  only the other groups and is omitted when the wrapper saw no solve.
- Split-impulse cells need the split-impulse fix (#3567), which `main` has.
