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
friction_eval.py run --bin B619=PATH --bin B620=PATH --out DIR -j 8
friction_eval.py report DIR > summary.md
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
exits nonzero, reports a non-finite state, or lacks its result rows, and writes
its rows to `failed.csv` instead of `cells.csv`, so they reach no table or
score. It runs the wall-clock cells (`--perf` without `--ir`) alone after the
parallel phase, and skips the Callgrind cells (`--ir`) when Valgrind is not
installed. `report` lists skipped and failed cells.

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
  `state_hash`.

A metric that is undefined for a run (an onset that never happened, a ratio
without samples) is omitted, so a NaN marks a failed measurement. `report`
never treats a NaN as equal, and counts a metric that only one side reports
as a difference.

PGS-tight and DZ+R stop on the box-law residual (at most 1e-6 m/s, 10-sweep
chunks, 1000-sweep cap); PGS's own relative-change test stops early on stacks
and never passes on rows near zero.

## Limits

- The wrapper is a custom backend type, so the 6.20 line solves islands
  serially with it. Use `--perf` for timing rows.
- VA is a user handler, which disables the default handler's fast paths, so
  VA rows are physics-only.
- With `--solver mf-pgs`, the 6.20 line solves large contact-only groups of
  free bodies matrix-free, past the wrapper, so the solver telemetry covers
  only the other groups and is omitted when the wrapper saw no solve.
- Split-impulse cells wait for PR-0 and are listed as pending by the runner.
