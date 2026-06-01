# Simultaneous-Impact Intake For PLAN-082

This sidecar intakes the `awesome-robotics-simulation` simultaneous-contact
papers and newer adjacent contact research into the rigid contact roadmap. It
does **not** create a new solver commitment. PLAN-082 remains the owner because
the question is whether DART needs an explicit instantaneous rigid-impact
operator in addition to the active rigid IPC finite-time contact path, and how
that decision compares against IPC, VBD, and AVBD work already tracked by the
living plans.

## Scope

In scope:

- simultaneous rigid-body impact and restitution models;
- high-speed, nearly simultaneous contact cases where event ordering matters;
- benchmarks that reveal break-away, symmetry, finite termination, energy
  conservation or non-injection, and drift behavior;
- comparison against DART's current sequential impulse path, boxed-LCP contact,
  rigid IPC, and the AVBD/VI contact-roadmap alternatives.

Out of scope for this intake:

- a public solver registry, a named upstream solver selector, or a dependency on
  any reference implementation;
- replacing PLAN-082 rigid IPC, PLAN-081 deformable IPC, or PLAN-104 VBD/AVBD;
- claiming differentiable impact support. If derivatives become the reason to
  adopt one of these models, route that decision through PLAN-110 after the
  forward impact operator exists.

## Source Set

The seed list is the
[`awesome-robotics-simulation` simultaneous-contact section](https://github.com/jslee02/awesome-robotics-simulation#simultaneous-contact),
which names:

- `smith-2012-rosi`: Reflections on Simultaneous Impact.
- `zhang-2015-qce`: Quadratic Contact Energy Model for Multi-impact Simulation.
- `vouga-2017-all-well`: All's Well That Ends Well: Guaranteed Resolution of
  Simultaneous Rigid Body Impact.

Additional current references for this intake:

- `halm-posa-2024-set-valued-impact`: set-valued simultaneous, inelastic,
  frictional impacts for robotics.
- `lelidec-2024-contact-models`: robotics contact-model survey and benchmark
  criteria.
- `avbd-2025`: augmented-Lagrangian contact/constraint handling in the VBD
  family.
- `ogc-2025`: offset geometric contact, a newer codimensional finite-time
  contact alternative now tracked for implementation planning under
  [`PLAN-104 OGC gap audit`](../104-vertex-block-descent-solver/ogc-gap-audit.md)
  before it is used as evidence against more IPC-specific machinery.

Detailed citations live in [`../../readthedocs/papers.md`](../../readthedocs/papers.md).

## Taxonomy

| Family                         | References                                                                                     | What it solves                                                                                    | DART stance                                                                                                                     |
| ------------------------------ | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| Instantaneous impact operators | `smith-2012-rosi`, `zhang-2015-qce`, `vouga-2017-all-well`, `halm-posa-2024-set-valued-impact` | Post-impact velocity/impulse selection when several rigid contacts occur at the same instant.     | Evaluate as a rigid-impact benchmark and possible restitution backstop; do not start with public API or broad implementation.   |
| Finite-time hard contact       | `ipc-2020`, `rigid-ipc-2021`                                                                   | Persistent non-penetrating contact, friction, CCD line search, barriers, and optimization solves. | Active path for robust geometry and stacking; likely supersedes many "simultaneous contact" stability goals without impact map. |
| Block/augmented contact        | `chen-2024-vbd`, `avbd-2025`                                                                   | Fast implicit-Euler minimization with local blocks, augmented forces, constraints, stacking.      | Existing PLAN-104/VI-contact-roadmap evidence; compare before adding a separate instantaneous-impact solver.                    |
| Geometry-contact alternatives  | `ogc-2025`                                                                                     | Penetration-free codimensional contact using offset geometry and local displacement bounds.       | Implement/evaluate under PLAN-104 first; not a rigid-impact replacement.                                                        |

## Initial Verdict

Do **not** open a standalone simultaneous-impact implementation now.

The active rigid IPC path already owns rigid contact robustness, conservative
CCD, barrier/contact objectives, projected Newton, friction, runtime fixture
behavior, and comparison evidence. AVBD is the best currently recorded
augmented-Lagrangian alternative for hard, drift-free constraints without a
global PSD solve in the variational-integrator contact roadmap. Those finite-time
paths should stay higher priority than an event-level impact map unless DART
needs a capability they do not provide:

- restitution or bounce behavior that the implicit barrier/contact paths cannot
  represent honestly;
- high-speed nearly simultaneous rigid impacts where contact ordering uncertainty
  dominates the result;
- a robotics-control use case that needs set-valued or uncertainty-aware impact
  outcomes rather than a single post-impact velocity;
- a regression corpus where the current sequential impulse / boxed-LCP / rigid
  IPC paths inject energy, fail to terminate, lose symmetry, or miss break-away.

Until one of those triggers is demonstrated, simultaneous-impact work should
remain a benchmark/intake sidecar under PLAN-082.

## Research Checklist

Before any implementation slice, build a row-level comparison matrix over the
seed papers and DART's current solvers:

| Axis                         | Evidence to record                                                                                                      |
| ---------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| Contact scope                | frictionless/frictional, elastic/inelastic, rigid-only, articulated, granular, persistent contact, restitution support. |
| Physical desiderata          | feasibility, symmetry, break-away, kinetic-energy conservation/non-injection, finite termination, no drift.             |
| Solver shape                 | LCP, generalized reflections, pairwise propagation, quadratic energy, set-valued differential inclusion, optimization.  |
| Outputs                      | unique post-impact velocity, bounded set of outcomes, per-contact impulses, diagnostics, uncertainty information.       |
| DART fit                     | reuse of `dart/math/lcp`, current contact Jacobians, rigid IPC CCD/barriers, boxed-LCP snapshots, benchmark harnesses.  |
| Differentiability            | whether a fixed-mode Jacobian is possible and whether mode/order switches make the gradient a subgradient only.         |
| Supersession by IPC/AVBD/OGC | which required scenes are already better handled by finite-time contact, augmented-Lagrangian constraints, or OGC.      |

## Candidate Corpus

Start with tests and benchmarks that are solver-independent and can be run
against current DART paths before a new operator exists:

- Newton's cradle and chained balls for wave propagation and break-away;
- two-corner rocking block or heel-toe foot strike for impact-order ambiguity;
- billiards break or dense granular pattern for simultaneous impact sensitivity;
- symmetric stacked rigid bodies for symmetry preservation and energy behavior;
- high-speed block/peg examples that distinguish restitution from persistent
  contact;
- a no-restitution stacking scene already covered by rigid IPC/AVBD to prove
  whether the finite-time path makes an impact operator unnecessary.

Each scene needs expected invariants before it becomes acceptance evidence:
post-impact feasibility, kinetic-energy bound, drift bound over repeated impacts,
termination/iteration budget, and whether break-away or sticking is expected.

## Sequencing

1. **Literature matrix and catalog** — Keep this sidecar and the research catalog
   current. Record which simultaneous-impact desiderata matter to DART and which
   are already covered by rigid IPC, AVBD, or OGC-style finite-time contact.
2. **Solver-neutral corpus** — Add DART-owned fixture descriptions and focused
   checks for the candidate corpus. Run them against sequential impulse, boxed
   LCP, and rigid IPC first; file gaps as evidence rather than adding a solver.
3. **Minimal internal baseline** — If the corpus exposes a real gap, prototype one
   internal impact operator on the smallest frictionless rigid scenes:
   LCP-inelastic baseline plus either generalized-reflection termination wrapping
   or QCE-style quadratic energy. Expose no public API.
4. **Decision gate** — Compare the baseline with rigid IPC and AVBD on the same
   corpus. If finite-time contact meets the required invariants, park the impact
   operator. If restitution/order uncertainty remains uniquely valuable, promote a
   narrow PLAN-082 workstream.
5. **Promotion slice** — Only after the gate, add an opt-in internal rigid-impact
   mode with DART-owned names, diagnostics, serialization behavior, focused tests,
   benchmark JSON, and no solver-registry leak.

## Acceptance Before Promotion

An explicit simultaneous-impact operator is not promotable until DART has:

- a completed literature matrix covering all seed and current references above;
- solver-neutral DART tests/benchmarks for the candidate corpus;
- evidence showing the active rigid IPC/AVBD paths do not already satisfy the
  required invariants for the target scenes;
- an internal prototype with finite termination, feasibility, energy, break-away,
  and symmetry evidence on named scenes;
- a public-boundary review proving no upstream project name, solver registry,
  reverse-pass cache, or ECS storage leaks into the facade;
- a dashboard update that either promotes the workstream under PLAN-082 or parks
  this sidecar with the reason.

## Open Questions

- Is DART's near-term need accurate restitution/bounce, or only robust
  non-penetrating contact and stacking?
- Should set-valued impact outcomes be exposed to control/learning users, or are
  they only benchmark evidence for uncertainty-aware simulation?
- Can rigid IPC's finite-time barrier solve and AVBD-style augmented forces cover
  the candidate corpus without an event-level operator?
- If an event-level operator lands, how does it compose with rigid IPC inside one
  timestep without double-counting impulses or violating the CCD line-search
  contract?
- Does PLAN-110 need derivatives through impact outcomes, or is finite-difference
  / subgradient documentation enough for impact scenes?
