# Pinned current-author incline sweep reference

Status: `valid_current_source_scientific_negative`.

Pinned current-author CPU diagnostic over all seven friction cells exposed by the public cube-on-incline runner, preserving the FBF results plus every FBF residual-history step.

## Recorded grid

| mu | FBF displacement (m) | FBF configured flags | FBF initial / outer accepts | Natural <= 1e-6 / > 1e-6 among true flags |
|---:|---:|---:|---:|---:|
| 0.3 | 3.5392831695743054 | 120/120 | 6 / 114 | 20 / 100 |
| 0.4 | 1.7698922978656797 | 120/120 | 0 / 120 | 8 / 112 |
| 0.45 | 0.88519761171157785 | 120/120 | 10 / 110 | 35 / 85 |
| 0.5 | 0.00050189263718551148 | 120/120 | 13 / 107 | 35 / 85 |
| 0.55 | 0.00042814879244091057 | 119/120 | 64 / 55 | 119 / 0 |
| 0.6 | 0.00035627086822120473 | 120/120 | 70 / 50 | 120 / 0 |
| 0.8 | 6.8839139364876853e-05 | 120/120 | 72 / 48 | 119 / 1 |

Every FBF cell records 120 steps. The retained histories record four contacts per step. FBF has 839/840 true configured convergence flags. The sole false flag is `mu=.55`, step 1, at the 200-outer cap. Its natural `final_residual` is `3.273267262002487e-8`, while the configured terminal `r_coulomb` is `1.5311460572898186e-6`.

Of the 839 true FBF flags, 235 use the initial natural-residual shortcut and 604 use the configured nonnegative `coulomb_rel < 1e-6` outer gate. Only 456 true flags also have natural `final_residual <= 1e-6`; 383 true flags have a larger natural value. The two residual fields are not interchangeable.


## Timing and parity boundary

The retained FBF lane is a current-source run. FBF has one configured nonconvergence at mu=.55 step 1; natural final_residual and the configured nonnegative coulomb_rel gate are distinct. Source step times include uncontrolled first-use/JIT and instrumentation effects and are diagnostic only. This is not a historical paper invocation, full published sweep, DART comparison, full-state or cross-solver parity result, approved golden, renderer/media evidence, paper timing, real-time, or performance evidence.

No manual image inspection is recorded. `comparison.svg` is a deterministic rendering of `comparison.csv`; the numeric CSV is authoritative.

## Verification

```bash
python3 scripts/finalize_fbf_author_incline_reference.py --verify-only
```
