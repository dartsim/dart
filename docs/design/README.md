# DART Design Docs

This directory holds durable technical design proposals and decision rationale.
Use it for architecture, API shape, tradeoffs, constraints, and design rules
that should outlive a particular roadmap sequence.

Design docs may be revised as evidence changes, but they should not own active
priority, horizon, next step, gate, or implementation handoff state. Keep those
fields in `docs/plans/dashboard.md` or `docs/dev_tasks/<task>/`.
For cross-bucket placement decisions, use
[`docs/information-architecture.md`](../information-architecture.md); this
README owns design-specific routing.

The published, single-page synthesis of these design docs — the DART 7
multi-physics / multi-solver / multi-backend architecture as abstracted pipeline
boxes with the available options at each seam — is
[`docs/readthedocs/architecture.md`](../readthedocs/architecture.md). It is a
navigational map that links back to the owner docs below; it does not own any
rule itself.

## Files

| File                                                                               | Purpose                                                                                                                                                                                                        |
| ---------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`agent_sim_verification.md`](agent_sim_verification.md)                           | Research and design record for agent-facing 3D-scene/physics verification: the text-first-corroborate-with-images decision, measured A/B evidence, image tolerance policy, and the rerun opt-in                |
| [`algorithm_extension_contracts.md`](algorithm_extension_contracts.md)             | Durable rules for research-facing algorithm extension and baseline contracts                                                                                                                                   |
| [`ai_spec_kit_assessment.md`](ai_spec_kit_assessment.md)                           | Assessment of GitHub Spec Kit for DART AI workflows, including the decision to adapt its lifecycle ideas without installing it as the default DART workflow                                                    |
| [`batched_world_device_residency.md`](batched_world_device_residency.md)           | Contract for homogeneous batched `World` semantics, SoA Model/State blocks, and internal device-residency boundaries (PLAN-091 WP-091.33)                                                                      |
| [`compute_backend_research.md`](compute_backend_research.md)                       | Evidence survey and DART workload-candidate ranking behind the scalable-compute roadmap                                                                                                                        |
| [`cpp23_modernization.md`](cpp23_modernization.md)                                 | C++23 adoption: compiler floor, portability gate (adopt-now/guarded/deferred), phased rollout, and per-phase execution status                                                                                  |
| [`dartsim_gui_simulator.md`](dartsim_gui_simulator.md)                             | Headless editor engine + thin ImGui/Filament GUI architecture for the dartsim simulator                                                                                                                        |
| [`dartsim_gui_toolkit_decisions.md`](dartsim_gui_toolkit_decisions.md)             | Toolkit (ImGui vs Qt), app-shell language (C++ vs Python), Filament headless, and deploy decisions for the dartsim application                                                                                 |
| [`dart7_architecture_assessment.md`](dart7_architecture_assessment.md)             | Verified findings, standing rules, and competitor lessons from the June 2026 DART 7 architecture review (executed by PLAN-091)                                                                                 |
| [`dart7_clean_break_strategy.md`](dart7_clean_break_strategy.md)                   | Release-topology decision: DART 7 is the clean-break line, `release-6.*` carries compatibility/gz-physics support, and promotion is parity-gated                                                               |
| [`dart7_handle_opaque_storage.md`](dart7_handle_opaque_storage.md)                 | PLAN-041 WS5 design: de-ECS the DART 7 handle classes (opaque `Entity`, internal validated-access base) so public headers hide ECS storage                                                                     |
| [`dart7_promotion_readiness_audit.md`](dart7_promotion_readiness_audit.md)         | PLAN-041 WS1 readiness audit: frozen public subset, internals-to-hide inventory, boundary-blocker checklist, and parity-evidence map                                                                           |
| [`dart7_world_namespace_transaction.md`](dart7_world_namespace_transaction.md)     | Durable rationale for clearing the classic `dart::simulation::World` name collision (PLAN-041 workstream 4): occupancy map, option (a) vs (b), recommended atomic replacement, and the `dartpy.World` decision |
| [`demos_app.md`](demos_app.md)                                                     | `dart-demos` architecture: GUI examples consolidated into one app as runtime-switchable scenes; examples-vs-renderer-fixtures split                                                                            |
| [`differentiable_simulation.md`](differentiable_simulation.md)                     | Opt-in analytic differentiable-simulation architecture and public API, including the Nimble LCP-gradient path and Dojo evaluation track                                                                        |
| [`filament_fidelity_profile.md`](filament_fidelity_profile.md)                     | Renderer-neutral fidelity-profile and camera-sensor seam for real-time vs offline rendering                                                                                                                    |
| [`hierarchical_allocator.md`](hierarchical_allocator.md)                           | World-level memory management proposal and allocator ownership rationale                                                                                                                                       |
| [`lie_group_batch.md`](lie_group_batch.md)                                         | Consolidated batch (data-parallel) strategy for the typed Lie group API (SO3/SE3)                                                                                                                              |
| [`local_verification_pipeline.md`](local_verification_pipeline.md)                 | Tiered, cache-aware local verification workflow and the systematic `verify-*`/`test-*`/`bench-*` task-naming scheme (tier × action × scope axes)                                                               |
| [`memory_layout_diagnostics.md`](memory_layout_diagnostics.md)                     | Cross-branch contract for truthful contiguous-region maps, DART 6 address atlases, semantic data categories, and cache-evidence boundaries                                                                     |
| [`multi-core-cpu-compute-graph.md`](multi-core-cpu-compute-graph.md)               | Archived Taskflow multi-core CPU compute-graph architecture (pipeline decomposition, determinism) preserved as a backend-agnostic reference                                                                    |
| [`renderer_fidelity_and_debug_visuals.md`](renderer_fidelity_and_debug_visuals.md) | Material/lighting/texture fidelity gaps (per-shape PBR, IBL reflections, mipmaps) and debug visualization as a first-class `dart::gui` feature (`debugProvider`, debug primitives, panel tuning)               |
| [`renderer_realtime_and_scalability.md`](renderer_realtime_and_scalability.md)     | Viewer real-time loop (interpolation/RTF) and large-scene scalability (instancing) roadmap                                                                                                                     |
| [`scalable_compute_decisions.md`](scalable_compute_decisions.md)                   | Workload-first CPU, SIMD, and GPU decision framework for scalable computation                                                                                                                                  |
| [`shared_cuda_device_substrate.md`](shared_cuda_device_substrate.md)               | Shared GPU device-runtime substrate so CUDA solvers reuse common blocks (runtime probe, error mapping, launch config, device buffer, single-sourced kernels) instead of reinventing them (PLAN-031)            |
| [`simulation_cpp_api.md`](simulation_cpp_api.md)                                   | C++ API shape for the official DART 7 clean-break `dart::simulation` API                                                                                                                                       |
| [`simulation_python_api.md`](simulation_python_api.md)                             | Python API shape for `dartpy.simulation` and the `dartpy.World` root convenience alias                                                                                                                         |
| [`simulation_experimental_references.md`](simulation_experimental_references.md)   | Compatibility pointer to the consolidated research reference catalog                                                                                                                                           |
| [`../readthedocs/papers.md`](../readthedocs/papers.md)                             | Managed catalog of research papers, textbooks, standards, and comparative engines for the DART 7 World (status, priority, verdict)                                                                             |
| [`simulation_solver_architecture.md`](simulation_solver_architecture.md)           | Internal architecture for solvers, domain assignment, multi-physics coupling, and the step schedule behind the DART 7 `World`                                                                                  |
| [`simulation_variational_integrator.md`](simulation_variational_integrator.md)     | Architecture and math rationale for the linear-time variational integrator (discrete-mechanics) integration family (PLAN-084)                                                                                  |

## Placement Rules

Use [`docs/information-architecture.md`](../information-architecture.md) for
the full placement matrix. The local rules below describe what this bucket
should and should not own:

- Put roadmap priority, status, horizon, next step, and gate in
  `docs/plans/dashboard.md`.
- Put active multi-session implementation state in `docs/dev_tasks/<task>/`.
- Put landed developer explanations in `docs/onboarding/`.
- Put user-facing instructions in `docs/readthedocs/` or `README.md`.
- Link to `docs/onboarding/api-boundaries.md` for public/internal API policy
  instead of duplicating it.

When a design becomes part of active roadmap work, keep the design doc as the
rationale owner and link it from the relevant plan.
