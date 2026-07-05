# DART 7 Documentation Migration Plan

The published `latest` documentation is built from `main` and is meant to
document **DART 7**. DART 7 is an in-progress redesign, so some pages still carry
**DART 6** content that has not been rewritten for the DART 7 API yet.

This plan tracks those pages so the legacy content is **clearly marked but not
lost**, and so nothing is forgotten when DART 7 grows the capability needed to
port a page. It is the counterpart to the per-page notices on the site.

## How this works

- Every page in `latest` whose content is still DART 6 carries a **legacy
  notice** at the top. The reusable RST notice lives in
  [`docs/readthedocs/_includes/legacy_dart6.rst`](../readthedocs/_includes/legacy_dart6.rst)
  (included via `.. include:: /_includes/legacy_dart6.rst`); Markdown pages use
  an equivalent inline admonition.
- Each notice links back to this plan, and this plan lists every notice. That
  closed loop means the legacy content stays visible and accountable instead of
  being deleted and forgotten.
- **When you port a page to DART 7**: rewrite the content against the current
  `dart.World` facade, remove the legacy notice (delete the `.. include::` line
  or the inline admonition), and update the page's row below to `Ported`. When
  the last row is ported, delete this file and the `_includes/legacy_dart6.rst`
  fragment.

Principle: prefer **same-or-better DART 7 coverage** over deletion. Only drop a
page outright when the underlying capability is intentionally removed from DART 7
(for example, the SKEL format).

## Status

Legend: **Legacy** = DART 6 content, notice shown · **Partial** = some DART 7,
some legacy · **Ported** = fully DART 7, notice removed.

| Page                                   | Status           | What is still DART 6                                                                                                        | DART 7 blocker                                                                                                           | Action when unblocked                                                                                                                                     |
| -------------------------------------- | ---------------- | --------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `shared/inverse_kinematics/index.rst`  | Legacy           | Entire modular IK framework (`InverseKinematics`, `EndEffector`, `createIK`/`getIK`/`solveAndApply`, `SimpleFrame` targets) | DART 7 facade exposes no IK framework yet                                                                                | Write a DART 7 IK user-guide page, then rewrite or redirect                                                                                               |
| `shared/inverse_kinematics/ikfast.rst` | Legacy           | `SharedLibraryIkFast` analytic IK plugged into `InverseKinematics`                                                          | No facade IK hook for analytic solvers                                                                                   | Rewrite once DART 7 exposes analytic IK                                                                                                                   |
| `dartpy/user_guide/examples.rst`       | Legacy           | camelCase `addSkeleton`/`getPositions`/`parseSkeleton`/`getSimFrames` hello-world                                           | None — superseded                                                                                                        | Already linked to the DART 7 {doc}`hello_dart </user_guide/getting_started/hello_dart>`; retire this page once the User Guide fully covers loading models |
| `topics/control-theory.md`             | Partial          | PD/inverse-dynamics code uses the DART 6 Skeleton API; the math is version-neutral                                          | No facade generalized-state/actuator API (`get_positions`/`set_forces`, `ActuatorType`, mass/Coriolis/gravity accessors) | Update code blocks when the facade control API lands                                                                                                      |
| `topics/control-gain-tuning.md`        | Partial          | DOF error/state reads (`getPositionDifferences`, etc.); math is version-neutral                                             | No facade generalized-state API                                                                                          | Update code blocks when available                                                                                                                         |
| `topics/numerical-methods.md`          | Partial          | Custom-integrator loop and `getConstraintSolver()` sections                                                                 | No documented facade custom-integrator path; solver selection now uses `RigidBodySolver`/`ContactSolverMethod`           | Rewrite the integrator/solver-settings sections for the DART 7 facade                                                                                     |
| `topics/simulation-stability.md`       | Partial          | Controller-gotchas and compliance-over-gains rely on DART 6 actuator/DOF-spring APIs                                        | No facade actuator/per-DOF spring-damping API                                                                            | Update those sections when available                                                                                                                      |
| `topics/skel-file-format.md`           | Legacy (removed) | `dart::io::readSkeleton` SKEL loading                                                                                       | SKEL is intentionally removed from DART 7                                                                                | No port; remove from `latest`; use a `release-6.*` branch or git history for legacy SKEL reference material                                               |
| `overview.rst`                         | Partial          | Feature bullets for modular IK, whole-body IK, IkFast, and soft body nodes                                                  | Those subsystems are not in the DART 7 facade yet                                                                        | Remove the targeted note and update the bullets as each subsystem lands                                                                                   |

## Pages reviewed and kept as-is (DART 7-current or version-neutral)

`index.rst`, `gallery.rst`, `architecture.md`, `papers.md`,
`community/*`, `topics/index.rst`, `tutorials/index.rst`,
`dartpy/user_guide/installation.rst`, `dart/user_guide/installation.rst`,
`dart/user_guide/api_boundaries.rst`, `dart/user_guide/migration_guide.rst`, and
the new `user_guide/**` pages.
