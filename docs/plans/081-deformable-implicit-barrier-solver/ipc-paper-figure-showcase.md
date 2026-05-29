# IPC Paper Figure Showcase Plan

This file enumerates the figure-by-figure visual and statistical showcase from
the IPC paper that DART must reproduce as part of PLAN-081 completion. It pairs
the [`ipc-paper-gap-audit.md`](ipc-paper-gap-audit.md) algorithm gap matrix and
the [`ipc_scene_corpus_manifest.json`](ipc_scene_corpus_manifest.json) scene
inventory: the gap audit owns _what kernels and Algorithm 1 phases_ must land,
the manifest owns _which upstream scene paths_ exist, and this document owns
_which paper figures and benchmark tables a complete DART implementation must
be able to redraw_.

## Source Snapshot

- Paper: Li et al., "Incremental Potential Contact: Intersection- and
  Inversion-free, Large-Deformation Dynamics," ACM Transactions on Graphics
  39(4), article 49, 2020. DOI:
  <https://doi.org/10.1145/3386569.3392425>. PDF audited at
  <https://cims.nyu.edu/gcl/papers/2020-IPC.pdf>.
- Upstream reference implementation:
  <https://github.com/ipc-sim/IPC>, audited at commit
  `573d2c7e04104d3f9baf526bdaee7745891a571a`. Treat as a reference and
  performance baseline only; DART keeps a clean-room implementation.
- Erleben unit-test reference (for Fig. 11): Erleben, K. "Methodology for
  Assessing Mesh-Based Contact Point Methods." ACM Transactions on Graphics
  37(3), article 39, 2018.
- Verschoor & Jalba reference (for Fig. 16): used by the paper as the
  Armadillo roller stick-slip baseline.

## Goals Captured Here

The PR stack rolling through #2719→#2732 and follow-ups is judged complete
only when DART can:

1. **Reproduce every figure listed in the table below**, either as a
   user-runnable example, a unit test with before/after snapshots, or a
   long-horizon headless capture wired into CI/visual evidence.
2. **Reproduce the benchmark statistics in Table 1 and Fig. 23**, with
   per-scene timing, iteration counts, contact counts, and memory footprint
   recorded as DART benchmark artifacts.
3. **Match or beat the paper's per-scene timings and iteration counts** on
   the listed hardware classes, or document a justified deviation when the
   target is intentionally different. Beating the upstream `ipc-sim/IPC`
   reference implementation on shared workloads is the long-term performance
   target.

Internal kernels with only unit tests and microbenchmarks (PRs #2719→#2732 to
date) do not satisfy these goals; they enable them. Every kernel must
eventually back at least one figure entry in the table.

Infrastructure progress note: a one-way conservative moving rigid box surface
CCD limiter has landed as Phase-2 scaffolding (the deformable line search now
predicts a free obstacle's swept motion and stays out of its corridor). It is
enabling infrastructure for the moving-obstacle figures (Fig. 8, 13, 15, 16,
18), which remain `planned`: those figures need two-way contact forces /
friction and timing-aware coupling that this one-way, timing-agnostic limiter
does not provide.

Progress note (contact forces): the deformable solve now adds IPC clamped-log
self-contact barrier FORCES (point-triangle + edge-edge), so self-contact is
smoothly repulsive rather than only CCD-limited — the first contact-force step
toward the figures. All figure rows remain `planned`: they still require
projected Newton (Phase 3, for stiff barriers to converge), barrier forces
against rigid/codimensional obstacles, and friction (Phase 6, for the
stick-slip / card-house / arch / roller figures).

## Showcase Catalog

Status column: `planned` (not yet implemented), `in-progress` (PR open),
`landed` (merged on main with passing tests/example/benchmark), or
`reference-beaten` (DART now equals or beats upstream/paper performance for
the scene).

| Paper ref       | Showcase name                                      | DART target type          | Physics features stressed                                                                                          | Prerequisite kernels / Algorithm 1 phases                                                      | Scene corpus match (see manifest)                    | Performance reference (paper)                                              | Status  |
| --------------- | -------------------------------------------------- | ------------------------- | ------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------- | ---------------------------------------------------- | -------------------------------------------------------------------------- | ------- |
| Fig. 2 (left)   | Mat on triangle (knife) obstacles                  | Example + headless replay | Nonsmooth contact, codimensional triangle obstacles, large deformation                                             | Codimensional collision objects, point-triangle / edge-edge barrier, CCD line search           | `mat-on-knives` family scenes in corpus              | Fig. 23 row `Mat on knives`: 3.2K nodes, 1.4 s/step, 5.5 iters             | planned |
| Fig. 2 (right)  | Ball on point-matrix obstacle                      | Example + headless replay | Codimensional point obstacles, nonsmooth contact, soft-ball deformation                                            | Codimensional collision (vertex obstacles), barrier, CCD                                       | `ball-on-points` family scenes in corpus             | Fig. 23 row `Ball on points`: 7K nodes, 2.8 s/step                         | planned |
| Fig. 4          | Rod twist 100 s (rod bundle)                       | Long-horizon benchmark    | Extreme stress test, conforming contact, repeated buckling, intersection- and inversion-free over 100 s            | Full IPC solver (barrier + projected Newton + CCD + friction-off)                              | `rods-twist-100s` family scenes in corpus            | Fig. 23 row `Rods twist (100s)`: 53K nodes, 141.5 s/step                   | planned |
| Fig. 5          | Stiff card house (impact)                          | Example + benchmark       | Frictionally stable stack, stiff thin boards (E = 0.1 GPa), high-speed impact, elastic bending/rebound             | IPC friction (lagged smoothed), high stiffness materials, CCD line search                      | `card-house-hit` family scenes in corpus             | Fig. 23 row `Hit board house`: 6K nodes, 10.0 s/step                       | planned |
| Fig. 7          | Masonry arch (cement, 20 m, high E)                | Example + benchmark       | Frictionally driven static equilibrium, tight geometric accuracy (`d̂=1 µm`), very stiff material (E = 20 GPa)      | IPC friction, fine accuracy thresholds, projected Newton stability at high stiffness           | `cement-arch` family scenes in corpus                | Fig. 23 row `Cement Arch`: 216 nodes, 0.05 s/step                          | planned |
| Fig. 8          | Soft ball through rollers (frictional contact)     | Example + benchmark       | Large deformation, frictional driving, ball pulled through rotating rollers, extreme compression                   | IPC friction (`µ = 0.5`), large deformation, codimensional/edge contact with codim mesh roller | `ball-mesh-roller` family scenes in corpus           | Fig. 23 row `Ball mesh roller`: 7K nodes, 63.3 s/step                      | planned |
| Fig. 10         | Aligned/close/nonsmooth contact unit tests         | Unit tests + snapshots    | Exact-alignment corners/asperities, deformable-deformable contact under gravity, conforming + nonsmooth resolution | Distance kernels, candidate set, CCD step bound, barrier, projected Newton minimal             | `unit-tests-aligned-nonsmooth` set in corpus         | None (correctness only; documented as paper §7.1)                          | planned |
| Fig. 11         | Erleben mesh-collision test cases                  | Unit tests + snapshots    | Fundamental mesh-based collision robustness, frame-rate-size time steps                                            | Distance kernels, candidate set, CCD, barrier, intersection-/inversion-free invariants         | `erleben-tests` set in corpus                        | None (correctness/robustness; baseline = Erleben 2018)                     | planned |
| Fig. 12         | Large mass and stiffness ratios                    | Example + benchmark       | Very wide material parameter range, stiff/soft pairings, ratio stability                                           | Material parameter sweep, stiffness adaptation, projected Newton conditioning                  | `mass-stiffness-ratios` family scenes in corpus      | None tabulated; behavior fidelity                                          | planned |
| Fig. 13         | Funnel test (neo-Hookean dolphin)                  | Example + benchmark       | Stiff neo-Hookean material, codimensional mesh obstacle (funnel), extreme deformation, rest-shape recovery         | Neo-Hookean material, codimensional contact, intersection-/inversion-free over the pull        | `dolphin-funnel` family scenes in corpus             | Fig. 23 row `Dolphin funnel`: 8K nodes, 27.9 s/step                        | planned |
| Fig. 14         | Volumetric mat extreme twist 100 s                 | Long-horizon benchmark    | Extreme twisting, all 45K nodes lying on surface, 100 s of simulated time                                          | Full IPC solver under stress, projected Newton stability, conforming contact at scale          | `mat-twist-100s` family scenes in corpus             | Fig. 23 row `Mat twist (100s)`: 45K nodes, 776.2 s/step                    | planned |
| Fig. 15         | Trash compactor (octocat + bunch)                  | Example + benchmark       | Six moving walls compress models to a small cube and release, extreme compression                                  | Active contact at scale, projected Newton, full-stiffness barrier, restart after release       | `trash-compactor` family scenes in corpus            | Fig. 23 row `Trash compactor: ball, mat and bunny`: 15K nodes, 61.9 s/step | planned |
| Fig. 16         | Armadillo roller (stick-slip)                      | Example + benchmark       | Stick-slip behavior under high friction and moderate stiffness, replication of Verschoor & Jalba 2019 baseline     | IPC friction, lagged friction convergence, FCR material variant                                | `armadillo-roller-stickslip` family scenes in corpus | Fig. 23 row `Stick-slip Armadillo roller (FCR)`: 67K nodes, 346.0 s/step   | planned |
| Fig. 17         | Pin-cushions (codimensional pins, ball)            | Example + headless replay | Codimensional line-segment obstacles compressing a soft ball, intersection-free invariant                          | Codimensional (edge) collision objects, barrier, CCD                                           | `pin-cushion` family scenes in corpus                | None tabulated; geometric correctness                                      | planned |
| Fig. 18         | Codimensional rollers (edges only / vertices only) | Example + headless replay | Codimensional obstacle robustness when rollers are degraded to edges or vertices only                              | Codimensional collision objects at all codims, barrier, CCD                                    | `ball-mesh-roller-codim` family scenes in corpus     | None tabulated; geometric correctness                                      | planned |
| Fig. 20         | Stick-slip oscillation (dragged elastic rod)       | Example + benchmark       | Friction-driven oscillations, elastic rod dragged along a surface                                                  | IPC friction, elastic rod material, persistent contact                                         | `stick-slip-rod` family scenes in corpus             | None tabulated; friction behavior fidelity                                 | planned |
| Table 1         | Time-step sweep on twisted rods                    | Parametric benchmark      | Iteration/timing tradeoffs across `h ∈ {0.002 … 2}` s for the same twisted-rods scene                              | Full IPC solver, parametric run harness, scene reuse                                           | Same rods scene as Fig. 4, with `h` sweep            | Paper Table 1 (full row by row)                                            | planned |
| Fig. 22         | Squishy ball into glass wall (688K nodes)          | Stress benchmark          | Very large mesh, hairy/tendrilled geometry, intersection-free under maximal compression                            | Solver scalability to ~700K nodes, AMGCL linear solver path, large barrier system stability    | `squishy-ball-glass-wall` family scenes in corpus    | Fig. 23 row `Squishy ball (AMGCL)`: 688K nodes, 328.3 s/step               | planned |
| Fig. 23 (Table) | Simulation statistics table                        | Benchmark report          | Per-scene geometry, h, materials, accuracies, contacts, machine, memory, timing, iterations (and friction iters)   | Benchmark harness that records all of the above per scene                                      | Cross-references rows above                          | Paper Fig. 23 itself                                                       | planned |

The scene-corpus "family" labels are descriptive; the authoritative scene path
lives in
[`ipc_scene_corpus_manifest.json`](ipc_scene_corpus_manifest.json). When a
figure entry above lands, the matching manifest rows must point to the DART
test/example/benchmark target that exercises them.

## Per-Figure Implementation Hooks

Each figure above expands a per-figure ownership block when work begins. The
template below keeps a planning slice and an implementing PR honest about what
"matches the figure" means.

```
### Fig. N — <Showcase name>

- DART target: <test target name / example name / benchmark target name>
- Scene paths (from manifest): <list of upstream .txt paths>
- Required kernels (from gap audit): <bullet list>
- Materials: <neo-Hookean / FCR / linear elastic / etc., E, ν, ρ>
- Time step h and accuracies: <values from paper>
- Friction parameters (if any): <µ, ε_v, lagged iterations>
- Headless visual evidence: <command line invoking `experimental_deformable_gui`
  or successor, with output destination>
- Statistic reproduction: <which numbers from Table 1 / Fig. 23 the benchmark
  must emit>
- Acceptance criteria:
  - Behavior matches paper figure qualitatively (snapshot diff or attached
    headless capture).
  - Per-step timing within X % of paper or upstream reference baseline.
  - Iteration counts and contact counts within Y % of paper Fig. 23 row.
- Status notes (PR refs, gating tests, follow-ups).
```

Add a per-figure block in place above the showcase catalog when the figure
moves out of `planned`. Do not delete the table row; update its `Status`
column instead.

## Sourcing The Required Assets

- Mesh and scene assets are obtained from the upstream
  `ipc-sim/IPC` repository at commit
  `573d2c7e04104d3f9baf526bdaee7745891a571a` (as recorded in the manifest).
  DART must not vendor those assets or take a runtime dependency on
  `ipc-sim/IPC`; the assets are pulled into local fixtures only for tests,
  examples, and benchmarks.
- Where the paper uses Verschoor & Jalba 2019 or Erleben 2018 baselines,
  those reference assets/parameters must be cited per-figure and matched
  independently in DART-owned code.
- For DART-original codimensional or material variants required to keep the
  showcase runnable on the existing `World` facade, document the deviation
  in the per-figure block.

## Performance Target Discipline

The figure showcase is also the performance contract. For every row that has
paper performance numbers (Table 1, Fig. 23 entries):

- DART benchmark output for the scene must include node/tet/face counts, `h`,
  materials, accuracies (`d̂`, `ε_v`, `ε_d`), contact-count avg/max,
  per-step time, per-step Newton iteration count, total simulated time, and
  memory footprint. These are the columns shown in Fig. 23.
- The benchmark must be runnable on a machine class comparable to what the
  paper reports (Intel Core i7 4-core 2.9 GHz / Xeon 8-core 3.0 GHz / i9
  8-core 3.6 GHz) and the row in this document must be annotated with the
  actual hardware used in DART.
- The first complete pass should land all figure entries as `landed`. The
  performance follow-up pass should drive each entry to `reference-beaten`
  by either matching/beating the paper number or matching/beating the
  upstream reference implementation on the same workload.

## Workstream Tie-In

The plan body in
[`../081-deformable-implicit-barrier-solver.md`](../081-deformable-implicit-barrier-solver.md)
already lists IPC paper parity expansion as Workstream 4. This file owns the
per-figure breakdown of that workstream and must be referenced by any future
slice PR description that adds, improves, or finalizes one of the paper
figures.

The dev-task tracking folder in
[`../../dev_tasks/ipc_deformable_solver/`](../../dev_tasks/ipc_deformable_solver/)
should not duplicate the figure catalog above. When that dev-task closes,
durable per-figure status moves into the `Status` column here.

## Update Cadence

- Update the `Status` column on every PR that lands paper-figure surface.
- When a row moves into `in-progress`, add the per-figure ownership block
  above the catalog table.
- When a row moves into `reference-beaten`, record the hardware, run command,
  and observed numbers in the per-figure block; never silently overwrite
  the paper-reference column.
- Treat any PR that lands an internal IPC kernel without advancing at least
  one row's `Status` toward landed as incomplete against PLAN-081
  acceptance and call it out in the PR description with a follow-up note.
