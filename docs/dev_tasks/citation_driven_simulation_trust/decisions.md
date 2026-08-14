# Decisions: Citation-Driven Simulation Trust

## 2026-08-14 — Machine manifest and packets are JSON

**Decision:** `claims-manifest.json` and evidence packets use JSON with schema
tags (`dart.citation_claim_manifest/v1`, `dart.citation_claim_evidence/v1`).
The corpus markdown stays the human authority for claim text/oracles; the
manifest owns live per-lane status/dispositions; the validator enforces exact
claim-ID agreement between the two.

**Why:** Every existing evidence validator in `scripts/` is JSON with no YAML
dependency anywhere in the tooling environment; a second format would split
the evidence system.

**Revisit when:** A packet needs human-authored long-form fields that JSON
makes error-prone.

## 2026-08-14 — Negative controls are committed packets that must fail

**Decision:** Intentionally incomplete packets live permanently under
`evidence/negative-controls/`; the gate requires each to produce >= 3
validation errors and fails if one starts passing. Packets do not carry a
self-describing negative-control flag the validator could special-case.

**Why:** A fail-closed claim needs a permanent executable counterexample; a
flag the validator reads would let the special case rot into a bypass.

**Revisit when:** A schema change legitimately shrinks a control below three
errors; then strengthen the control, not the threshold.

## 2026-08-14 — Structural validation in CI, freshness on demand

**Decision:** `check-citation-evidence` (in `check-lint`) validates structure,
consistency, and cross-links of committed packets. `--freshness` (packet
commit must equal current HEAD) is used when writing/regenerating packets,
not in CI.

**Why:** Squash merges legitimately retire topic-branch commits, so an
ancestry gate would poison every packet after merge; the commit still binds
the packet to its exact source for reproduction.

**Revisit when:** Evidence regeneration becomes automated enough to re-stamp
packets on merge.

## 2026-08-14 — `main` manifest dart6 lanes are routing pointers

**Decision:** The PLAN-123 manifest on `main` records dart6 lanes as
routing/audit pointers; the `release-6.20` branch manifest
(`docs/design/dart6_citation_driven_contact_trust/claims-manifest.json` on
that branch) owns live DART 6 lane status/dispositions. `main` mirrors
promoted DART 6 results only at explicit sync points.

**Why:** Branches cannot share one mutable file, and two live owners of the
same lane state would drift; claim identity stays single-owned by the corpus
on `main`.

**Revisit when:** A release process needs automated cross-branch mirroring.

## 2026-08-14 — CT-001 disposition thresholds

**Decision:** The rolling-direction packet calls the claim `reproduced` when
any solver shows max |lateral drift| > 1e-4 m, max |heading error| > 0.1 deg,
or relative travel spread > 1% across the launch-angle sweep; otherwise
`unresolved`. Measured on `main` 20501341226: lateral drift up to 2.1e-3 m
and travel spread 4.8e-3 with the symmetry-consistent sign structure (zero at
0/45/90 deg), under both SEQUENTIAL_IMPULSE and BOXED_LCP.

**Why:** The bounded claim is rotational-symmetry breaking; the thresholds sit
an order of magnitude above observed numerical noise at the symmetry angles
(heading error ~1e-14 deg) and well below the measured signal.

**Revisit when:** A wider sweep, other shapes/speeds, or an exact-cone method
needs a shared anisotropy metric; then promote the metric definition to the
durable design doc.

## 2026-08-14 — One DART 7 durable owner, separate DART 6 adaptation

**Decision:** DART 7 owns the long-term contact-trust architecture and PLAN-123.
DART 6 owns a branch-local compatibility design and dev task.

**Why:** DART 7 can evolve clean-break internals and APIs; DART 6 must preserve
ABI, defaults, packages, language/binding/rendering floors, and downstream
behavior. Shared defects still receive separate branch PRs.

**Revisit when:** DART 6.20 leaves maintenance or a new active LTS branch
changes the compatibility boundary.

## 2026-08-14 — Claims are stable rows, not prose conclusions

**Decision:** Every material external claim receives a stable ID, bounded
oracle, branch/version, evidence packet, disposition, and claim boundary.

**Why:** Citation sentiment and benchmark prose cannot distinguish historical
versions, invalid comparisons, fixed defects, and current limitations.

**Revisit when:** The evidence schema proves too rigid for a named source type;
extend it without weakening required provenance.

## 2026-08-14 — Raw impulses remain authoritative

**Decision:** Impulse-based methods report raw impulse. Average wrench,
continuous force, filtering, and event integration are distinct typed/metadata
semantics.

**Why:** Labeling `impulse / dt` as instantaneous force creates incorrect
biomechanics/control interpretations and hides timestep dependence.

**Revisit when:** A shared public result surface is designed from at least two
solver families and two downstream consumers.

## 2026-08-14 — Extend current DART-owned observability

**Decision:** Extend `ResolvedSolverConfiguration`, `StepMetrics`,
`WorldStepProfile`, `LcpResult`, and current contact/problem structures before
adding parallel diagnostics.

**Why:** Method identity and physical metrics already have owners; duplication
would make evidence inconsistent.

**Revisit when:** A metric has incompatible mathematical meaning across
families; keep the common field absent and add a solver-specific nested report.

## 2026-08-14 — Exact-cone work is evidence-gated

**Decision:** Do not begin with a public ADMM/exact-cone solver. First build the
corpus, contact semantics, and matched comparison. Then record a GO/NO-GO.

**Why:** DART already has multiple algorithms and an ADMM boxed-LCP solver. The
missing value is a true cone-contact problem and a demonstrated Pareto region,
not another algorithm name.

**Revisit when:** WS1–WS4 evidence identifies named failures that an exact cone
can plausibly solve and defines promotion metrics.

## 2026-08-14 — Companion layers stay optional

**Decision:** Planning, Gym/Torch/JAX integration, sensors, datasets,
biomechanics, and application controllers are companion candidates. Core
receives only smaller primitives justified by multiple consumers.

**Why:** DART should remain a physics/research platform without forcing
third-party application stacks or frameworks into the core dependency graph.

**Revisit when:** A second core consumer proves a stable smaller abstraction.

## 2026-08-14 — First-wave corpus is capped

**Decision:** The first implementation wave has six fixture families.

**Why:** A bounded set can be completed and reviewed; an open-ended paper
backlog would prevent closeout.

**Revisit when:** All six have branch-qualified dispositions or the maintainer
explicitly changes the cap.
