# Decisions: DART 6 Citation-Driven Contact Trust

## 2026-08-14 — Branch manifest owns release-6.20 lane state

**Decision:** This branch's `claims-manifest.json` (design-doc sidecar) is the
single owner of `release-6.20` lane status, dispositions, and packets. The
`main` PLAN-123 manifest keeps its dart6 lanes as routing pointers and mirrors
promoted results only at explicit sync points.

**Why:** Branches cannot share one mutable file; dual live owners would drift.
The corpus (claim IDs, titles, bounded claims) stays owned on `main` and is
referenced by `corpus_reference`, so row identity cannot fork.

**Revisit when:** A release process needs automated cross-branch mirroring.

## 2026-08-14 — Evidence lives in the design sidecar, not the dev task

**Decision:** The manifest and packets live under
`docs/design/dart6_citation_driven_contact_trust/`; the dev-task folder holds
only working state and is deleted at completion.

**Why:** Dev-task folders are removed in the completing PR; evidence packets
are durable regression guards and must survive that cleanup.

**Revisit when:** Never for this task; new evidence types choose their durable
owner the same way.

## 2026-08-14 — Unsupported solver internals stay typed unsupported

**Decision:** Per-solve LCP iterations, residuals, and Dantzig-vs-PGS fallback
events are recorded as typed `unsupported` metrics on this branch instead of
zeros or library changes; opt-in diagnostics may come later under the Phase 4
compatibility rules.

**Why:** The LTS contract forbids speculative instrumentation, and the
north-star rule forbids encoding unsupported as numeric zero.

**Revisit when:** A confirmed defect needs per-solve visibility; then design
the smallest opt-in diagnostic with layout/vtable preservation.

## 2026-08-14 — Maintenance/evidence lane only

**Decision:** DART 6 reproduces claims, guards current behavior, and fixes
confirmed defects without receiving DART 7 solver/model/contact architecture.

**Why:** `release-6.20` is an ABI/default/downstream compatibility line.

**Revisit when:** A maintainer authorizes a new active LTS architecture or
release boundary.

## 2026-08-14 — Existing PLAN-621/622 owners win

**Decision:** A corpus row already owned by performance-generalization or
deformable work remains there; this task references its evidence.

**Why:** Duplicate task graphs create stale contradictory handoffs.

**Revisit when:** Those tasks complete and their durable evidence owner changes.

## 2026-08-14 — Legacy force fields are measured, not silently redefined

**Decision:** Tests and analysis distinguish collision geometry, impulses,
legacy force/wrench outputs, and derived averages/filters. Released semantics
are not changed implicitly.

**Why:** Downstream Gazebo and user code rely on the current data contract.

**Revisit when:** An additive API is proven ABI-safe and needed by multiple
DART 6 consumers.

## 2026-08-14 — No new solver family

**Decision:** Exact-cone, IPC, AVBD/VBD, differentiable, and batch architecture
are DART 7-only. DART 6 may improve existing backends/solvers after a negative
control.

**Why:** New solver architecture is incompatible with LTS scope and would
duplicate DART 7.

**Revisit when:** Never for 6.20 absent explicit maintainer direction.

## 2026-08-14 — Downstream validation is part of correctness

**Decision:** Collision, constraint, `World::step`, parser, component, or
default-sensitive changes require gz-physics/gz-sim evidence, not only DART
tests.

**Why:** Gazebo integration is a primary DART 6 compatibility consumer.

**Revisit when:** The maintained downstream contract changes.
