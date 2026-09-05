# Rigid-Body Solver — Architecture Principles

Status: superseded by the current durable contracts.

This former task-local policy no longer governs implementation. In particular,
matching DART 6 algorithms/results is not the physical acceptance criterion for
the independently designed DART 7 engine.

Read these owners before starting a slice:

- [PLAN-040](../../plans/040-dart7-release-hardening.md): independent oracles,
  readiness, continuous audits and prerequisite order. Existing promotion
  checker rules remain enforced until WP-040.2 migrates them.
- [Solver architecture](../../design/simulation_solver_architecture.md):
  physical representations, solver compatibility, coupling and state ownership.
- [Compute decisions](../../design/scalable_compute_decisions.md):
  cache-friendly data, portable kernels and execution-graph contracts.
- [PLAN-080](../../plans/080-rigid-body-dynamics-solver.md): admitted rigid
  implementation packets and evidence.

The task [README](README.md) and [RESUME](RESUME.md) retain historical slice
evidence and route agents to those current owners.
