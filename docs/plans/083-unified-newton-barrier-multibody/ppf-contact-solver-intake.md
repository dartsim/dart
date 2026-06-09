# PLAN-083 PPF Contact Solver Intake

This sidecar records how DART should investigate
<https://github.com/st-tech/ppf-contact-solver> and the related cubic-barrier
paper without turning either into a dependency or public solver identity.
`dashboard.md` remains the source of truth for priority and gates.

## Classification

- **Paper-backed method:** Ryoichi Ando, "A Cubic Barrier with
  Elasticity-Inclusive Dynamic Stiffness," _ACM Transactions on Graphics_
  43(6), 2024, DOI <https://doi.org/10.1145/3687908>. The paper introduces a
  cubic barrier with elasticity-inclusive dynamic stiffness for
  penetration-free contact resolution and strain limiting.
- **Software baseline:** ZOZO's Contact Solver is an independent Apache-2.0
  solver stack that started as an in-house physics engine. It should be treated
  as reference implementation, API, diagnostics, example-corpus, and platform
  evidence, not as an engine to wrap or vendor.
- **Platform lineage:** The public repository does not present itself as a
  wrapper around another simulator. It has its own Rust workspace, Python
  extension/frontends, C++/CUDA solver kernels, server process, Docker/Windows
  deployment path, JupyterLab workflows, and Blender add-on.
- **DART owner:** PLAN-083 owns the Newton-barrier/contact/strain-limiting
  intake. PLAN-081 owns shell/solid/rod deformable state if implementation
  needs new codimensional or volumetric model support. PLAN-030 owns private
  CPU/GPU gates. PLAN-101/103 may reuse frontend/demo lessons after a DART
  runtime path exists.

## Strategic DART 7 Pipeline Investigation

The investigation target is a DART-owned contact and strain-limiting capability
for shells, solids, rods, stitches, and colliders. PPF should influence the
pipeline where it exposes a better contract, but DART should keep the public
`World` facade and solver-family vocabulary aligned with the north star.

- **Domain and model.** Audit PPF's shell triangle, tetrahedral solid, rod,
  stitch, collider, per-object parameter, and mesh-asset concepts against
  DART's deformable-domain model/state split. Record which data belongs in
  existing `DeformableBody` facilities, which requires codimensional shell/rod
  adapters, and which should remain frontend/example metadata.
- **Contact and constraints.** Evaluate the cubic barrier, log/quadratic
  comparison modes, dynamic stiffness policy, strain limits, friction, ACCD
  tolerances, and stitch/fixed constraints as candidate internal Newton-barrier
  rows. Compare each against DART's current IPC, rigid IPC, VBD/AVBD, and
  planned unified Newton-barrier rows before promoting shared primitives.
- **Elasticity and tangents.** Audit the elasticity-inclusive dynamic stiffness
  formulas, symbolic or eigen-filtered force Jacobians, shell/solid tangent
  systems, and cited eigenvalue work. Decide whether DART needs a shared
  elastic tangent/eigen-filter owner or variant-local kernels first.
- **Solver schedule.** Map PPF's Newton, line search, CG, CCD, barrier,
  friction, strain-limiting, and logging phases to the DART 7
  model/state/control/contact/solver schedule. Any runtime slice must step
  through `World::step()` and emit DART-owned diagnostics.
- **Couplers.** Mixed shell/solid/rod and rigid/deformable interactions should
  use DART contact buffers and future coupler seams. Do not add a PPF-specific
  side channel between domains.
- **Backend and performance.** PPF is GPU-first and emphasizes single-precision
  CUDA and very large contact counts. DART should still land CPU-verifiable
  equations first, then add private CUDA packets with same-scene CPU/GPU result
  parity, contact-count scaling, memory, setup/transfer/readback timing, and
  large-scene stress gates.
- **API and product surface.** PPF's method-chaining Python frontends, queryable
  solver logs, headless workflows, remote Blender usage, notebooks, Docker,
  Windows, and cloud paths are useful product evidence. DART should adopt
  the durable ideas -- fluent dartpy construction where it fits, structured
  logs, reproducible examples, and remote/headless validation -- without
  importing PPF names, server architecture, CUDA handles, or notebook
  assumptions into public APIs.

## DART Improvements To Consider

- Codimensional shell and rod deformable state adapters compatible with the
  existing deformable and unified Newton-barrier plans.
- A typed contact/constraint-row schema for contact, friction, strain limits,
  stitches, and fixed/boundary constraints.
- ACCD and CCD precision diagnostics, including single-precision failure-mode
  tests and conservative CPU reference comparisons.
- Per-object material, contact, strain-limit, wind, air-friction, and solver
  parameter validation with serialization/restart behavior for result-affecting
  values.
- Shared solver-log counters for CCD, active contacts, barrier rows,
  strain-limit rows, linear-solve iterations, Newton steps, line-search cuts,
  and convergence reasons.
- Large-scale contact benchmark packets and stress scenes that include
  shell/solid/rod contact counts, memory, CPU/GPU timing, and visual evidence.
- Example/front-end guidance for dartpy, `py-demos`, Blender-style inspection,
  and cloud/headless runs after the core runtime path exists.

## Investigation Matrix

| Source                                      | Key questions                                                                                                               | DART output                                                                                                |
| ------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| Cubic-barrier TOG paper                     | Which cubic barrier, dynamic stiffness, strain-limit, friction, and solver formulas beat current DART IPC/VBD/AVBD choices? | Method audit with formula parity tests, comparison packets, and shared-primitive go/no-go decisions.       |
| PPF repository core crates and CUDA kernels | Which data layouts, kernels, diagnostics, and precision choices are reusable as DART-owned internal contracts?              | CPU reference implementation plan, private CUDA packet plan, benchmark schema, and API-boundary checklist. |
| Python/Jupyter/Blender frontends            | Which workflows improve dartpy examples, remote inspection, and solver logs without leaking PPF architecture?               | `py-demos` and DART tooling recommendations after runtime evidence exists.                                 |
| Example corpus                              | Which scenes cover shells, rods, stitches, friction, strain limiting, large contact counts, and failure cases?              | Manifest rows linking scenes to tests, benchmark JSON, visual evidence, and paper/reference comparisons.   |

## Completion Gate

PPF intake is not complete until DART has:

- classified every relevant paper equation, repository module, example scene,
  article note, API surface, and deployment/platform lesson as adopt, compare,
  defer, or reject;
- mapped each adopted or compared method feature to PLAN-081, PLAN-083,
  PLAN-030, PLAN-101, or PLAN-103 ownership;
- reproduced the selected paper/reference scenes with DART-owned tests,
  benchmark JSON, and headless visual evidence before claiming parity;
- compared cubic barrier, dynamic stiffness, strain limiting, friction, ACCD,
  and GPU behavior against DART IPC, rigid IPC, VBD/AVBD, and any existing
  deformable incumbents at matched scene parameters; and
- kept public APIs free of PPF, ZOZO, server, CUDA, ECS, solver-registry,
  backend-resource, and upstream implementation types.
