# PLAN-083 IPC Variant Consolidation Map

This sidecar records how DART should fold the IPC family of methods into the
unified Newton-barrier multibody direction without duplicating plan ownership.
It is a routing map, not a second dashboard: `dashboard.md` owns priority,
status, horizon, next step, and gate.

## Consolidation Rule

PLAN-083 owns the shared DART IPC method family: public capability names,
internal Newton-barrier primitive contracts, cross-variant benchmark shape,
CPU/GPU parity rules, and py-demos integration discipline. Use IPC as the
representative solver-family name when the unified Newton-barrier method is the
most advanced shared IPC variant; use Newton-barrier for the implementation
contracts and the paper-specific method lineage.

Variant plans keep their specialized obligations until shared evidence proves a
piece should move:

- PLAN-081 owns deformable IPC, codimensional IPC, mesh/material state,
  deformable scene corpora, PD-IPC GPU evaluation, and SPB recovery.
- PLAN-082 owns exact/reduced rigid IPC, curved rigid CCD, rigid fixture
  replay, rigid barrier/friction solves, and rigid comparison baselines.
- PLAN-083 owns ABD and the unified paper/deck rows until an ABD runtime task
  graduates into a dedicated implementation tracker, and owns the PPF cubic
  barrier / strain-limiting intake until evidence routes pieces to a variant
  owner.
- PLAN-030 owns private CPU/GPU compute policy and gates.
- PLAN-103 owns the py-demos category surface once a runtime path exists.

Do not move a primitive, API knob, benchmark packet, or example out of its
variant owner just because names look similar. Promote only after a second
variant uses it and cross-variant tests prove the behavior is shared.

## Variant Matrix

| Variant                                                               | Owner               | Unified role                                                                                                                                                       | Shared candidates                                                                                                                                         | Must stay variant-owned until                                                                                                                                | Public naming policy                                                                                      |
| --------------------------------------------------------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------- |
| Deformable IPC (`ipc-2020`)                                           | PLAN-081            | Fullspace deformable contact/inversion-free target and primary FEM/codimensional corpus.                                                                           | C2 clamped-log barrier, PT/EE distances, tangent bases, CCD line search, projected Newton, PSD projection, lagged friction, diagnostics, benchmark JSON.  | Mesh-backed FEM, material models, boundary controls, scene import/restart, upstream corpus parity, and deformable-specific failure modes have stable tests.  | Public APIs name deformable implicit-barrier contact capabilities, not IPC solver identities.             |
| Codimensional IPC (`ipc-2020`/C-IPC lineage)                          | PLAN-081            | Rod/shell/particle contact and mixed-dimensional evidence for the unified paper scenes.                                                                            | Candidate sets, codimensional distance kernels, friction tangents, visual evidence categories.                                                            | Rod/shell state, codimensional obstacle semantics, and py-demo scene assets are DART-owned and verified.                                                     | Public APIs name geometry/domain capabilities, not C-IPC.                                                 |
| Rigid IPC (`rigid-ipc-2021`)                                          | PLAN-082            | Exact/reduced rigid correctness oracle for barrier contact, curved trajectory CCD, and rigid fixture parity.                                                       | Reduced-coordinate barrier wrappers, CCD feasibility bounds, projected Newton diagnostics, lagged friction, comparison packet shape.                      | The audited fixture/test/benchmark/comparison corpus is covered or explicitly classified, and ABD has beaten matched rows without losing correctness.        | Public APIs name rigid implicit-barrier contact policies, not rigid IPC.                                  |
| Affine body dynamics (`lan-2022-abd`)                                 | PLAN-083            | Stiff-body representation that makes rigid/deformable unification practical with piecewise-linear contact trajectories.                                            | 12-DOF affine state adapters, orthogonality energy, affine primitive/friction chain rules, ABD comparison packets.                                        | Rigid-equivalence, energy behavior, contact quality, paper/deck benchmarks, and Bullet/reference comparisons prove where ABD replaces or augments rigid IPC. | Public APIs may name affine stiff-body dynamics after runtime evidence; internal rows may cite ABD.       |
| Unified Newton-barrier multibody (`chen-2022-unified-newton-barrier`) | PLAN-083            | Target integrated method: mixed rigid/deformable/codimensional contact, articulation constraints, BDF-2, Rayleigh restitution, paper examples, and CPU/GPU parity. | Public method-family vocabulary, shared primitive contracts, cross-variant benchmark schema, py-demo category mapping.                                    | Every manifest row is mapped to a DART artifact, CPU/GPU evidence exists where applicable, and public API boundaries are reviewed.                           | Public APIs use DART-owned method and capability names only.                                              |
| PD-IPC GPU (`lan-2022-pdipc`)                                         | PLAN-081 + PLAN-030 | GPU-accelerated deformable IPC candidate, not the default unified GPU answer.                                                                                      | GPU contact culling, projective IPC reference loops, A-Jacobi-style solver evidence, CPU/GPU packet shape.                                                | CPU reference tests, conservative CCD validation, A-Jacobi convergence, and same-host CPU/GPU parity beat DART CPU baselines.                                | No public PD-IPC, A-Jacobi, CUDA, stream, device, or memory-pool surface.                                 |
| SPB recovery (`chen-2023-spb`)                                        | PLAN-081            | Preexisting-intersection recovery sidecar for volumetric deformables; not a replacement for CCD/barriers.                                                          | Recovery diagnostics, hybrid CCD/DCD comparison packets, possible tetrahedral recovery constraints.                                                       | Tetrahedral query/recovery tests and limitations prove the method is safe for the specific solver path.                                                      | No public SPB/reference-project/Embree/MeshFrame/CuMatrix leakage.                                        |
| PPF cubic barrier and strain limiting (`ando-2024-cubic-barrier`)     | PLAN-083 + PLAN-081 | Contact/strain-limiting method and GPU software-stack baseline for shells, solids, rods, stitches, and high contact counts.                                        | Cubic barrier scalar/derivatives, dynamic stiffness policy, strain-limit rows, ACCD diagnostics, solver logs, contact benchmark schema, frontend lessons. | Paper formulas, repository scenes/API, precision caveats, shell/rod/solid state adapters, and CPU/GPU comparison packets are audited.                        | Public APIs name contact/strain-limit policies and diagnostics, not PPF, ZOZO, CUDA, or server frontends. |
| VBD/OGC adjacent contact                                              | PLAN-104            | Alternative/deformable performance and geometry-contact baselines that may reuse shared distance/barrier evidence.                                                 | Benchmark schema, visual evidence discipline, contact comparison packets.                                                                                 | Their block-descent or offset-contact assumptions are proven compatible with the unified Newton-barrier contract.                                            | Keep VBD/OGC names in internal provenance and plan docs, not as broad public solver registry names.       |

## Shared API Boundary

The public DART 7 facade should expose stable DART concepts:

- body/domain capabilities: deformable body, rigid body, affine stiff body,
  rods, shells, particles, codimensional geometry;
- method capabilities: implicit barrier contact, conservative CCD,
  variational friction, BDF-2 integration, energy diagnostics, restitution
  damping, affine stiff-body approximation;
- policy/options values with documented units, defaults, serialization, and
  diagnostics.

The public facade must not expose upstream project names, solver registries,
coupler registries, ECS storage, implementation namespaces, CUDA resources, or
backend task handles. Internal tests, manifests, benchmark JSON, and paper
provenance may cite IPC, rigid IPC, ABD, PD-IPC, SPB, VBD, OGC, Bullet, and
paper figure/table IDs.

## Implementation Promotion Sequence

1. **Keep variant rows classified.** Maintain the PLAN-083 paper/deck manifest
   plus PLAN-081/082/104 sidecar manifests as the row owners for paper figures,
   corpus scenes, unit tests, benchmark rows, and visual evidence.
2. **Promote second-use primitives.** When both deformable and rigid/ABD paths
   need the same primitive, move the stable part into an internal
   Newton-barrier owner with cross-variant tests. Keep unstable or
   domain-specific terms in the variant owner.
3. **Promote benchmark packets before runtime claims.** A runtime or py-demo
   slice should follow a comparison packet that records scene parameters,
   accuracy, CPU timing, GPU timing if present, DART incumbent comparison,
   reference implementation comparison, and paper/deck number comparison.
4. **Promote APIs last.** Public options and Python bindings should follow
   runtime evidence, serialization/restart tests, API-boundary checks, and
   py-demo evidence. A name is promotable only if it is DART-owned and applies
   across the intended variants.
5. **Promote GPU privately first.** GPU work starts behind PLAN-030 gates:
   same-scene CPU/GPU result parity, setup/transfer/readback timing, no public
   backend resources, and a speedup packet against DART CPU before broader
   claims.

## Evidence Required For Consolidation

Before a row can be called unified, it needs evidence at the same scope as the
claim:

- correctness: derivative checks, PSD behavior, line-search feasibility,
  no-intersection/no-inversion where applicable, constraint residuals,
  friction/stiction, restitution energy, determinism, and serialization;
- coverage: manifest row links to concrete DART tests, benchmarks, py-demos,
  visual captures, or a maintained manual/not-applicable rationale;
- performance: matched scene parameters and matched accuracy against DART
  incumbents, audited references, and paper/deck numbers;
- GPU: CPU/GPU result parity on the same DART scene plus timing breakdowns
  that include setup, transfer, kernel/solve, and readback where applicable;
- API: `check-api-boundaries` proves public headers and dartpy bindings do not
  leak upstream project, ECS, registry, solver, coupler, or backend types.

## Current Next Consolidation Move

The next bounded implementation move remains Phase 2/3 of PLAN-083, now tracked in
the durable plan file
[`../083-unified-newton-barrier-multibody.md`](../083-unified-newton-barrier-multibody.md)
and these sidecars (the temporary dev-task folder was retired on 2026-07-04):

1. Promote the existing `bm_affine_body_dynamics` smoke rows into comparison
   packets linked to the PLAN-083 manifest.
2. Use that packet to decide whether a two-body affine contact micro-solve is
   needed before generalizing shared projected-Newton and PSD contracts.
3. In parallel at the planning layer, classify the PPF cubic-barrier paper and
   repository through
   [`ppf-contact-solver-intake.md`](ppf-contact-solver-intake.md) so shell,
   rod, strain-limit, solver-log, and GPU-platform lessons do not bypass the
   shared Newton-barrier owner.
4. Only after those contracts are stable, start runtime ABD stepping and the
   first py-demo rows. GPU packets remain private and benchmark-gated.
