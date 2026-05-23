# DART Design Docs

This directory holds durable technical design proposals and decision rationale.
Use it for architecture, API shape, tradeoffs, constraints, and design rules
that should outlive a particular roadmap sequence.

Design docs may be revised as evidence changes, but they should not own active
priority, horizon, next step, gate, or implementation handoff state. Keep those
fields in `docs/plans/dashboard.md` or `docs/dev_tasks/<task>/`.

## Files

| File                                                                             | Purpose                                                                                                                             |
| -------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| [`algorithm_extension_contracts.md`](algorithm_extension_contracts.md)           | Durable rules for research-facing algorithm extension and baseline contracts                                                        |
| [`hierarchical_allocator.md`](hierarchical_allocator.md)                         | World-level memory management proposal and allocator ownership rationale                                                            |
| [`scalable_compute_decisions.md`](scalable_compute_decisions.md)                 | Workload-first CPU, SIMD, and GPU decision framework for scalable computation                                                       |
| [`simulation_experimental_cpp_api.md`](simulation_experimental_cpp_api.md)       | C++ API shape for promoting `dart::simulation::experimental` to the DART 8 simulation API                                           |
| [`simulation_experimental_python_api.md`](simulation_experimental_python_api.md) | Python API shape for `dartpy.simulation_experimental` over the experimental simulation stack                                        |
| [`simulation_solver_architecture.md`](simulation_solver_architecture.md)         | Internal architecture for solvers, domain assignment, multi-physics coupling, and the step schedule behind the experimental `World` |

## Placement Rules

- Put roadmap priority, status, horizon, next step, and gate in
  `docs/plans/dashboard.md`.
- Put active multi-session implementation state in `docs/dev_tasks/<task>/`.
- Put landed developer explanations in `docs/onboarding/`.
- Put user-facing instructions in `docs/readthedocs/` or `README.md`.
- Link to `docs/onboarding/api-boundaries.md` for public/internal API policy
  instead of duplicating it.

When a design becomes part of active roadmap work, keep the design doc as the
rationale owner and link it from the relevant plan.
