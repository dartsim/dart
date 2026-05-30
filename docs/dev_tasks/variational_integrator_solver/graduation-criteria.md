# VI Solver — Graduation Criteria (experimental → supported)

When may the variational integrator stop being an _experimental_ `World`
integration family and become a **supported** DART solver? This file is the
checklist. It complements the [North Star + Gaps](README.md#north-star); the
North Star says _where_ we are going, this says _when we have arrived_ well
enough to commit to API stability and user-facing support.

"Supported" in DART means: a stable public surface we will not break without a
deprecation cycle, CI gates that keep it green, user-facing docs, and a
maintainer who owns regressions. The VI is opt-in behind the method-name facade
(`World::setMultibodyOptions({.integrationFamily = "variational integrator"})`),
so graduation does not change defaults — it changes the _promise_ we make about
the family.

## Criteria

Each item is binary and evidence-backed (mirroring the PLAN-082 acceptance-gate
rigor). `[x]` = met today, `[ ]` = open.

### Correctness & numerics

- [x] Symplectic / no-secular-drift on a passive chain over `1e5` steps; ≥50×
      better energy behavior than semi-implicit Euler (headline gate).
- [x] Momentum conservation (linear + world angular) for a force-free floating
      body.
- [x] Analytic single-step accuracy (revolute pendulum, prismatic free-fall) to
      `1e-9`.
- [x] Manifold-correct floating/spherical retraction (FD-verified).
- [x] Holonomic loop closures (Point / Distance / Rigid) hold over a long
      horizon; constraint Jacobians FD-verified.
- [ ] **Convergence is robust at scale.** RIQN converges within budget for the
      chains we claim to support, including very long/stiff systems — needs the
      [≥100-link exact recursive-Jacobian preconditioner](README.md#gaps-from-current-progress-road-to-the-north-star)
      and ideally the manifold-aware Anderson for floating chains. Until then,
      document the supported envelope (DOF/stiffness bounds) explicitly.
- [ ] **Contact & friction** available _or_ the supported envelope explicitly
      excludes contact. A "supported" solver that silently cannot do contact is
      a documentation trap; either Phase C lands (compliant/AL rungs) or the
      docs state the contact-free envelope as a first-class limitation.

### API & integration

- [x] Pure method-name facade; no solver/stage/component/backend types exposed
      (`check-api-boundaries` green).
- [x] Scalable config via the `MultibodyOptions` value object.
- [x] **Zero runtime overhead** when not selected (default path unchanged;
      `DefaultPathDoesNotEngageVariationalIntegrator` gate).
- [x] Determinism (bit-identical reruns) + binary save/load round-trips the
      two-step history without re-bootstrapping.
- [ ] **API frozen with a deprecation policy.** The `integrationFamily` name,
      `MultibodyOptions` fields, and the non-convergence error contract are
      declared stable; future additions are additive.

### Engineering quality

- [x] Defined non-convergence error (no silent NaN/fallback).
- [x] Internal dedup of shared spatial-algebra/kinematic-tree helpers (no
      duplicate maintenance burden across the SI and VI paths).
- [x] dartpy parity (selector + `Link.apply_force`) with stubs + tests.
- [x] GUI demo scenes for visual verification.
- [ ] **Performance characterized for users**, not just gated: a published
      per-step cost vs DOF curve and a VI-vs-semi-implicit guidance note (when
      to pick the VI), beyond the internal O(n) benchmark.

## Explicit non-blockers (future / stretch)

These are north-star ambitions, **not** graduation gates — graduating without
them is fine as long as the supported envelope says so:

- Variable time step (the symplectic guarantees assume fixed `Δt`).
- GPU / batched execution.
- Contact rungs beyond the first compliant/AL one.

## Process to propose graduation

1. Close the open `[ ]` items above (or, for contact, ratify the contact-free
   supported envelope in the design doc).
2. Open a `PLAN-` entry proposing the promotion, linking the evidence for each
   criterion, with an owner.
3. Freeze the facade surface + write the user-facing "choosing an integrator"
   doc.
4. Adversarial review (mirroring the PLAN-082 4-agent review) before flipping
   the family from experimental to supported.

Until then the VI stays a fully-functional **experimental** family — usable and
opt-in, but without the stability promise. Keep this file in sync with the
North Star as the open items close.
