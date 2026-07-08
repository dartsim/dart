# DART 6 AI Infrastructure Bucket Decision

## Decision

DART 6.20 adopts the AI-infrastructure documentation buckets that solve
release-branch maintenance problems now:

- `docs/plans/` for living priority, roadmap, and gate state.
- `docs/design/` for durable release-branch design rationale.
- `docs/background/` for theory, paper, and reference foundations.
- `docs/assets/` for durable repository documentation assets.

This is a DART 6 adaptation, not a wholesale import of DART 7 project state.
The release branch remains a compatibility lane for the established DART 6 API,
installed headers, package components, and Gazebo/gz-physics consumers.

## Rationale

Before these buckets, long-running DART 6 work had to keep roadmap dashboards,
design decisions, paper matrices, and reusable documentation evidence inside
`docs/dev_tasks/`. That made task retirement harder because durable facts had
no precise owner after the temporary folder was removed.

The new buckets separate lifecycle from topic:

- `docs/plans/` owns mutable operating state.
- `docs/design/` owns durable engineering decisions.
- `docs/background/` owns reusable theory and reference context.
- `docs/assets/` owns durable doc media.
- `docs/dev_tasks/` stays temporary and branch/session-local.

## DART 6 Differences From DART 7

DART 7 `main` can use clean API-breaking changes to simplify the future
simulation stack. DART 6.20 cannot use that latitude as proof that a
release-branch change is safe.

For DART 6:

- DART 7 docs and code are reference evidence only.
- Public headers, exported package components, and gz-physics/gz-sim behavior
  remain compatibility constraints.
- GPU, clean-break API shape, EnTT world plumbing, and DART 7-only package
  restructures stay out of release-branch plans unless a maintainer explicitly
  accepts a scoped backport.
- Durable DART 6 decisions should say which compatibility surface they protect
  and which gate proves it.

## Intentional Non-Adoption

`docs/python_api/` from DART 7 is not adopted in this AI-infra change. On this
release branch, dartpy user-facing documentation remains under
`docs/readthedocs/dartpy/`, and C++ API generation input remains under
`docs/doxygen/`. Moving DART 6 to the DART 7 reusable Python autodoc layout
would be a separate documentation-build change, not an AI-infra bucket
requirement.
