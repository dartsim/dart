# Contact-Aware Inverse Dynamics + IK Utilities (DART 6.19) — Dev Task

## Current Status

- [x] Phase 0: Scope + branch setup (`feature/contact-inverse-dynamics-6.19` off `origin/release-6.19`)
- [ ] Phase 1: Subsystem mapping (ID/IK/QP infra/tests/benchmarks/GUI/dartpy/gz-compat)
- [ ] Phase 2: Design (API + algorithm) and test specification
- [ ] Phase 3: Core implementation (C++) with unit tests
- [ ] Phase 4: Benchmarks + performance pass
- [ ] Phase 5: GUI example (OSG + ImGui) visual verification
- [ ] Phase 6: dartpy bindings + Python tests
- [ ] Phase 7: CHANGELOG, docs, lint/test-all gates, PR prep, dev-task cleanup

## Goal

DART 6 can compute generalized forces for a tracked motion
(`Skeleton::computeInverseDynamics`), but when the motion involves contacts
(e.g., a humanoid standing/walking), users must bring their own QP solver to
split the result into feasible joint torques and contact forces — the
floating-base component of the generalized forces is not actuatable and must be
realized by contact wrenches inside friction cones. Ship a built-in,
dependency-free API on the 6.x LTS line that:

1. computes joint torques **and** contact forces for a desired acceleration
   given contact point/normal/friction specs (friction-cone-consistent wrench
   distribution),
2. exposes supporting utilities so per-axis actuation torques are unambiguous
   for 1-DOF joints,
3. is validated by unit tests (consistency with forward dynamics), benchmarked,
   bound in dartpy, and demonstrated in an ImGui + OSG GUI example.

## Non-Goals

- No changes to existing simulation/constraint-solver behavior (gz-physics
  backward compatibility is a hard requirement; additive API only).
- No new external dependencies (no off-the-shelf QP library).
- No DART 7 (`main`) port in this task; record a follow-up note instead.

## Key Decisions

- Target branch: `release-6.19` (next 6.x minor; LTS line).
- Record algorithm/API decisions in `01-design.md` as they are made.

## Immediate Next Steps

1. Finish subsystem mapping (workflow `understand-dart619-id-ik`).
2. Write `01-design.md` (API surface, algorithm, test plan) and review it.
