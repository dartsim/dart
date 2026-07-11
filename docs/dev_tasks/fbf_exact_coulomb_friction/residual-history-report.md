# FBF Residual-History Report

This report records the current Figure 9 residual-convergence evidence for
the DART 6.20 exact-Coulomb FBF prototype. It is intentionally scoped to
reproducible DART-side artifacts. It does not claim paper parity.

## Scope

Implemented evidence:

- bounded opt-in per-outer residual samples in the internal exact-FBF solver,
- bounded retained per-solve residual-history records in the opt-in DART
  constraint solver,
- `fbf_paper_trace ... residual_history` CSV export for exact-FBF runs,
- one-step residual-history smokes for backspin, reduced 26-card card house,
  reduced 25-stone arch, and reduced 101-stone arch,
- individual SVG plots plus a combined Figure 9-style SVG panel for the
  current reduced-scaffold evidence.

Still missing:

- paper-matched Figure 9 comparisons,
- full-contact 26-card and arch histories,
- long-run settle/projectile/balance histories,
- external baseline histories from Kamino, MuJoCo, or the paper implementation.

The card-house cap-probe helper also has an optional retained-history summary
for diagnosing fallback boundaries. That output is not a replacement for the
`fbf_paper_trace ... residual_history` CSV rows, but it records whether a
failed cap row reached the final configured FBF iteration and how the residual
was moving near the cap.

## CSV Schema

The `residual_history` trace scope emits:

```text
step,time,scenario,solver,solve_index,solve_contacts,outer_iteration,
residual,primal_feasibility,dual_feasibility,complementarity,step_size,
safe_step_size,coupling_variation_ratio,shrink_iterations,contacts,
exact_solves,warm_starts,fallbacks,status
```

`solve_index` is reset when the trace tool clears the retained records before
each simulated step. `solve_contacts` is the number of contacts in that exact
solve group. `contacts` is the full collision-result contact count for the
sampled step.

## Commands

```bash
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 1 1 nan residual_history > /tmp/fbf_backspin_residual_history.csv
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan residual_history > /tmp/fbf_card_house_residual_history.csv
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan residual_history > /tmp/fbf_arch25_residual_history.csv
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan residual_history > /tmp/fbf_arch101_residual_history.csv
```

## Latest Local Results

Run date: 2026-07-09, Release build, after implementing the paper's fixed
per-solve residual scales, applying fixture-specific safe-step multipliers,
rerunning the full paper-fixture CTest, and regenerating the individual and
combined SVG artifacts from the current trace executable. The reduced arch
defaults remain 48 contacts / two contacts per pair for the 25-stone arch and
38 contacts / two contacts per pair for the 101-stone arch, with a 20000
outer-iteration arch budget and `step_size_scale=10`.

| Scenario | Rows | Exact groups | Last group contacts | Last outer iteration | Last residual | Fallbacks | Status |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `backspin` | 18 | 1 | 1 | 17 | `7.8906612729366679e-07` | 0 | `success` |
| `card_house_26_reduced_contact` | 1911 | 3 | 1 | 0 | `2.2183427389532158e-16` | 0 | `success` |
| `masonry_arch_25_reduced_contact` | 2646 | 1 | 48 | 2645 | `9.9650461738801354e-07` | 0 | `success` |
| `masonry_arch_101_reduced_contact` | 3913 | 1 | 38 | 3912 | `9.9001648754006507e-07` | 0 | `success` |

The reduced 26-card scene has three exact groups in the sampled step:

| Solve index | Contacts | Rows | Initial residual | Final iteration | Final residual | Status |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | 54 | 1909 | `0.80948819725465004` | 1908 | `9.943369051914846e-07` | `success` |
| 1 | 1 | 1 | `3.9193471180919099e-16` | 0 | `3.9193471180919099e-16` | `success` |
| 2 | 1 | 1 | `2.2183427389532158e-16` | 0 | `2.2183427389532158e-16` | `success` |

## Cap-Probe Failure Diagnostics

The benchmark-only cap probe can retain bounded residual-history samples for
the current fallback boundary:

```bash
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 56,60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001
```

Latest local results from the same run date and build, after adding the
paper-scaled residual and dense residual-polish diagnostic:

| Max contacts | Clean exact | Records | Rows | Failed rows | Dense polishes | Failed last iteration | Failed first residual | Failed best/final residual | Pre-polish history last | Failed tail ratio |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 56 | 1 | 3 | 1911 | 0 | 0 | 0 | n/a | n/a | n/a | n/a |
| 60 | 0 | 2 | 30002 | 30001 | 1 | 30000 | `1.1460517049589893e-05` | `5.3340525726528403e-06` | `5.3375628811483052e-06` | `1.0000019065431343` |

The 60-contact diagnostic reaches the 30000 outer-iteration cap. The residual
is dual-feasibility dominated and changes very slowly at the tail. Dense
polish improves the final failed residual from the retained-history tail to
the best failed residual, but the result is still above the paper-scaled
`1e-6` tolerance; the current blocker is solver convergence at the
contact-rich boundary, not a missing retained-history row.

## Plot Artifacts

The current SVG plots were generated with:

```bash
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py /tmp/fbf_backspin_residual_history.csv docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_backspin_residual_history.svg --title "Backspin residual history"
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py /tmp/fbf_card_house_residual_history.csv docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_card_house_residual_history.svg --title "Reduced 26-card residual history"
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py /tmp/fbf_arch25_residual_history.csv docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_arch25_residual_history.svg --title "Reduced 25-stone arch residual history"
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py /tmp/fbf_arch101_residual_history.csv docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_arch101_residual_history.svg --title "Reduced 101-stone arch residual history"
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py \
  --title "Current reduced exact-FBF residual histories" \
  --output docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_residual_history_panel.svg \
  --panel /tmp/fbf_backspin_residual_history.csv "Backspin, first exact step" \
  --panel /tmp/fbf_card_house_residual_history.csv "Reduced 26-card, one step" \
  --panel /tmp/fbf_arch25_residual_history.csv "Reduced 25-stone arch, one step" \
  --panel /tmp/fbf_arch101_residual_history.csv "Reduced 101-stone arch, one step"
```

- [Backspin residual history](assets/fbf_backspin_residual_history.svg)
- [Reduced 26-card residual history](assets/fbf_card_house_residual_history.svg)
- [Reduced 25-stone arch residual history](assets/fbf_arch25_residual_history.svg)
- [Reduced 101-stone arch residual history](assets/fbf_arch101_residual_history.svg)
- [Combined reduced residual-history panel](assets/fbf_residual_history_panel.svg)

## Interpretation

These artifacts show that the current exact-FBF path can emit per-outer
convergence histories for every exact group retained during a sampled step.
They also show that the reduced contact-rich scenes still require many outer
iterations: the first 26-card group reaches 1908 iterations, and the reduced
25-stone and 101-stone arch groups reach 2645 and 3912 iterations.

This is useful Figure 9 evidence, but it remains reduced-scaffold evidence.
Paper parity still requires full-contact and long-run histories,
paper-matched comparison plots, and external baseline data when those
implementations are available.
