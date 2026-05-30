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
- [x] **Convergence is robust at scale.** The exact recursive-Jacobian Newton
      preconditioner lands ~3 iterations independent of length (verified to 128
      links) and the manifold-aware tangent-space Anderson acceleration converges
      spherical/floating chains; the supported DOF/stiffness bounds are documented
      explicitly in [`supported-envelope.md`](supported-envelope.md). (Extending
      the manifold preconditioner to very long floating chains stays a
      north-star item, but the supported envelope is now stated, not implicit.)
- [x] **Contact & friction** available, with the supported envelope stated.
      Phase C landed the compliant/AL rungs — **C1** lagged Coulomb friction,
      **C2** compliant ground contact, **C3** augmented-Lagrangian drift-free
      contact (`makeVariationalGroundContactHook` /
      `VariationalGroundContactSolver`), reachable from `World::step()` via
      `Multibody::setGroundContact`, plus a **link-vs-link** sphere-sphere slice
      (`makeVariationalLinkSphereContactHook`). Supported contact envelope:
      link-point-vs-analytic-ground and sphere-sphere link pairs, `k ≲ 1e4·mg`
      (see [`supported-envelope.md`](supported-envelope.md)); **arbitrary link
      geometry** (the rigid-IPC-stack adapter) is a stated first-class
      limitation / future workstream, not a silent gap.

### API & integration

- [x] Pure method-name facade; no solver/stage/component/backend types exposed
      (`check-api-boundaries` green).
- [x] Scalable config via the `MultibodyOptions` value object.
- [x] **Zero runtime overhead** when not selected (default path unchanged;
      `DefaultPathDoesNotEngageVariationalIntegrator` gate).
- [x] Determinism (bit-identical reruns) + binary save/load round-trips the
      two-step history without re-bootstrapping.
- [x] **API frozen with a deprecation policy (surface declared; committed at the
      graduation flip).** The stable public surface is: the
      `"variational integrator"` `integrationFamily` name; the `MultibodyOptions`
      value-object fields; the loop-closure API (`add_loop_closure` /
      `LoopClosureSpec`); the ground/link-contact API
      (`Multibody::setGroundContact` / `addGroundContactPoint`, dartpy
      `set_ground_contact` / `add_ground_contact_point`); and the non-convergence
      error contract (a hard `InvalidOperationException`, never a silent
      NaN/fallback). Deprecation policy: none of these are changed or removed
      without a deprecation cycle; future capability is **additive** (new
      `MultibodyOptions` fields default to current behavior, new contact builders
      are additive). The freeze is committed when the family flips to supported
      (review step below).

### Engineering quality

- [x] Defined non-convergence error (no silent NaN/fallback).
- [x] Internal dedup of shared spatial-algebra/kinematic-tree helpers (no
      duplicate maintenance burden across the SI and VI paths).
- [x] dartpy parity (selector + `Link.apply_force`) with stubs + tests.
- [x] GUI demo scenes for visual verification.
- [x] **Performance characterized for users**, not just gated: the per-step cost
      vs DOF curve (O(N) fit) and a VI-vs-semi-implicit guidance note (when to
      pick the VI) are published in [`performance.md`](performance.md), beyond the
      internal O(n) benchmark.

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

## Readiness assessment (2026-05-30)

**All graduation criteria above are now met or declared.** Correctness/numerics
(incl. convergence-at-scale), API & integration (incl. the API-freeze surface
declared above), and engineering quality (incl. Phase C contact and the published
performance + choosing-an-integrator guidance in
[`performance.md`](performance.md)) are all `[x]`. **Process steps 1 and 3 are
executed**; the VI is **ready to propose for graduation**.

The remaining steps are maintainer-owned and cannot be self-approved: **(2)** open
the graduation `PLAN-` entry linking this evidence with an owner, and **(4)** the
adversarial review before the `experimental → supported` flip. The
arbitrary-geometry link-vs-link adapter and the C4 barrier are **explicit
non-blockers** (future workstreams), not graduation gates.

Until that review and flip, the VI stays a fully-functional **experimental**
family — usable and opt-in, but without the stability promise. Keep this file in
sync with the North Star as the remaining process steps complete.
