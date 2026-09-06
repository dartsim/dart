# DART 7 M1 Oracle Manifest

## Goal And Traceability

Prepare the independent numerical acceptance specification for
[WP-040.1](../../plans/040-dart7-release-hardening.md#wp-0401-oracle-manifest-and-acceptance-specification).
Every M1 pass/fail decision must follow fixed physical inputs and independent
expected results, so agreement between two incorrect backends cannot qualify
DART 7. This task was prepared by the maintainer-approved handoff cleanup on
2026-09-06; oracle derivation has not started and no execution owner is claimed.

Priority and next action belong to
[PLAN-040 in the dashboard](../../plans/dashboard.md#plan-040-dart-7-readiness-milestones).
The packet owns scope, acceptance and dependencies; this folder holds only the
temporary working handoff. Read [RESUME.md](RESUME.md) before claiming it.

## Deliverable And Scope

Produce an owner-linked numerical manifest under PLAN-040, with intended final
path `docs/plans/040-dart7-release-hardening/m1-oracle-manifest.md`. Create and
link that durable artifact as the specification is authored, not as an empty
placeholder. Derive each RB-01 through RB-07 row from the packet's independent
analytic or converged-reference contract:

- fixed scene inputs, units, force/torque frames and lifecycle, timestep and
  refinement sequence, solver budget and finite horizon;
- expected physical quantities and derivations, absolute-plus-relative error
  budgets, contact-event and penetration bounds, and supported scale/speed
  envelope;
- refinement acceptance, restart points and same-backend versus cross-backend
  comparisons, including CPU-to-CPU, CUDA-to-CUDA and both cross-device paths;
- declared unsupported cases, reproducible oracle procedures, source/test
  references, and independent review evidence rejecting circular tolerances.

The starting fixture values in PLAN-040 are proposals. Derive and justify the
missing inputs and budgets before accepting them; do not fit tolerances to DART
output. Existing tests and scene builders may establish feasibility but are not
the definition of correctness.

## Constraints And Non-Goals

- Float64 first. Every admitted M1 example ultimately requires complete CPU
  and actual CUDA execution; skipped GPU tests or CPU fallback cannot pass.
- M1 covers one dynamic sphere or box and the specified static-plane examples.
  Keep the 100-body/scaling corpus in M2 and broader robotics in M3.
- This packet writes and reviews the numerical specification. It does not fix
  physics, adopt a compute library, implement CUDA/checkpoints, migrate policy
  checkers, or claim M1/paper parity. PLAN-030/080/041/042/122 keep their owners.
- Existing promotion checkers remain enforced until WP-040.2 lands. DART 6
  comparisons are supplemental; independent physical oracles are primary.

## Intake, Acceptance And Gates

- **Dependencies:** WP-040.1 has no predecessor packet. Downstream work must
  wait for its accepted manifest; task creation does not satisfy that gate.
- **Architecture impact:** numerical/API/evidence contracts. At acceptance,
  update the PLAN-040 manifest, architecture assessment and affected capability
  rows, as the packet requires. No model, solver or public API changes are
  implied by this handoff.
- **Execution choice:** Astra Max, one sequential implementation/synthesis
  owner; no parallel writers are authorized. Record the effective session
  settings and owner on claim. Use independent or role-separated review under
  the repository verification policy; this handoff is not a review pass.
- **Open work:** exact budgets and fixture inputs are the specification's
  deliverable, not accepted defaults. Escalate a change to M1 scope or method
  policy to its owner rather than silently broaden the packet.
- **Acceptance:** every RB row contains the complete evidence contract above;
  independent review accepts the derivations and rejects implementation-fitted
  thresholds. Record two clean review passes under
  [verification policy](../../ai/verification.md).
- **Gates:** `pixi run lint-md`, `pixi run check-lint-md`,
  `pixi run check-docs-policy`, `pixi run check-lint-spell`; run full
  `pixi run lint` before a commit and the policy's full CPU/CUDA gates before
  publication. Name any focused feasibility tests actually run and their limits.

## Completion And Handoff

Keep current owner, revision, decisions and evidence in RESUME while active.
At WP-040.1 acceptance, promote all durable derivations and review evidence into
PLAN-040 and its manifest, update packet state and remove this task folder in
the completing change. Do not create successor task folders until their
prerequisites pass and the dashboard selects their work.
