---
name: dart-perf
description: "DART Perf: reproduce workloads, attribute bottlenecks, optimize with behavior guards, and report performance claims"
---

# DART Performance

## Required Reading

@docs/ai/principles.md
@docs/onboarding/profiling.md

Follow `docs/onboarding/profiling.md` § "Performance Methodology", the shared
standard for agents and contributors. Reproduce the user's scenario and state
deviations before optimizing. Choose exact-count A/B gates for regressions and
controlled wall-time repeats for speed claims; pair every claim with behavior
guards. Attribute engine and integration costs by phase before choosing a
change, then verify the full workload and downstream compatibility.

For exact-count commands and policy, use the owner's "Revision Comparisons"
section. For visible simulation changes, load `dart-verify-sim` and follow the
linked verification route. For publication, follow the linked evidence flow;
uploading requires explicit approval.

## Output

Leave reproducible evidence: named DART baseline and candidate revisions,
scenario/deviations, setup and timed windows, count-gate results, wall-time
median/min-max when measured, behavior results, attribution, and compatibility
limitations. Report the user effect first with plots using the linked
"PR Descriptions" guidance. Public comparisons use DART revisions only.
