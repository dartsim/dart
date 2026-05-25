# DART Design Docs

This directory holds durable technical design proposals and decision rationale.
Use it for architecture, API shape, tradeoffs, constraints, and design rules
that should outlive a particular roadmap sequence.

Design docs may be revised as evidence changes, but they should not own active
priority, horizon, next step, gate, or implementation handoff state. Keep those
fields in `docs/plans/dashboard.md` or `docs/dev_tasks/<task>/`.

## Files

| File                                                                             | Purpose                                                                                                                                  |
| -------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| [`algorithm_extension_contracts.md`](algorithm_extension_contracts.md)           | Durable rules for research-facing algorithm extension and baseline contracts                                                             |
| [`compute_backend_research.md`](compute_backend_research.md)                     | Evidence survey and DART workload-candidate ranking behind the scalable-compute roadmap                                                  |
| [`dartsim_gui_simulator.md`](dartsim_gui_simulator.md)                           | Headless editor engine + thin ImGui/Filament GUI architecture for the dartsim simulator                                                  |
| [`demos_app.md`](demos_app.md)                                                   | `dart-demos` architecture: GUI examples consolidated into one app as runtime-switchable scenes; examples-vs-renderer-fixtures split      |
| [`filament_fidelity_profile.md`](filament_fidelity_profile.md)                   | Renderer-neutral fidelity-profile and camera-sensor seam for real-time vs offline rendering                                              |
| [`hierarchical_allocator.md`](hierarchical_allocator.md)                         | World-level memory management proposal and allocator ownership rationale                                                                 |
| [`lie_group_batch.md`](lie_group_batch.md)                                       | Consolidated batch (data-parallel) strategy for the typed Lie group API (SO3/SE3)                                                        |
| [`renderer_realtime_and_scalability.md`](renderer_realtime_and_scalability.md)   | Viewer real-time loop (interpolation/RTF) and large-scene scalability (instancing) roadmap                                               |
| [`scalable_compute_decisions.md`](scalable_compute_decisions.md)                 | Workload-first CPU, SIMD, and GPU decision framework for scalable computation                                                            |
| [`simulation_experimental_cpp_api.md`](simulation_experimental_cpp_api.md)       | C++ API shape for promoting `dart::simulation::experimental` to the DART 8 simulation API                                                |
| [`simulation_experimental_python_api.md`](simulation_experimental_python_api.md) | Python API shape for `dartpy.simulation_experimental` over the experimental simulation stack                                             |
| [`simulation_experimental_references.md`](simulation_experimental_references.md) | Managed catalog of research papers, textbooks, standards, and comparative engines for the experimental world (status, priority, verdict) |
| [`simulation_solver_architecture.md`](simulation_solver_architecture.md)         | Internal architecture for solvers, domain assignment, multi-physics coupling, and the step schedule behind the experimental `World`      |

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
