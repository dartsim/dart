# Solver Family Intake Checklist

This checklist is the plan-owned intake surface for new solver, algorithm,
paper, or major component work. PLAN-020 in [`dashboard.md`](dashboard.md) owns
the operating gate; durable architecture rationale lives in
[`../design/algorithm_extension_contracts.md`](../design/algorithm_extension_contracts.md)
and
[`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md).

Before implementation starts, the owner plan or dev task should record:

1. **Family routing.** Name the existing DART solver family and owner plan that
   receives the work, or justify a new family. Examples: PLAN-081/082/083 for
   IPC and Newton-barrier variants, PLAN-104 for VBD/OGC/AVBD variants,
   PLAN-110 plus PLAN-082 for differentiable variational rigid-body work.
2. **Shared-component inventory.** List the collision, kinematics, model/state,
   contact buffer, numeric optimization, linear-solver, compute-backend,
   diagnostics, benchmark, and example components the slice will reuse. Any new
   duplicate should name the missing contract that prevents reuse.
3. **Promotion trigger.** Define what second-use evidence would move a
   variant-local piece into a shared internal owner, and what tests will prove
   old and new consumers still agree.
4. **Apples-to-apples evidence.** Define the DART incumbent, reference
   implementation, paper number, scene corpus, accuracy metric, and benchmark
   JSON shape used for comparison. A performance claim without matched accuracy
   and matched scene parameters is not a completion claim.
5. **Public boundary.** Confirm public APIs and dartpy bindings expose
   DART-owned domains, method families, policies, diagnostics, and value types,
   not upstream project names, solver registries, ECS storage, backend
   resources, or reverse-pass caches.
6. **Configuration surface.** Define the default `World`/options path, the
   advanced nested options, validation rules, serialization expectations, and
   diagnostics. The common path should be simple, and invalid or incompatible
   option combinations should fail before they produce misleading simulation
   results.
7. **Failure and fallback semantics.** Record unsupported-feature errors,
   fallback behavior, non-convergence handling, determinism requirements, and
   serialization/restart expectations before promoting a runtime path.
8. **World-step schedule integration.** Record which built-in schedule slots the
   runtime path adds, replaces, or deliberately avoids in
   `detail/world_step_schedule.hpp`; whether the method participates in the
   prepare / pre-couple / couple / post-couple lifecycle; which domain-presence
   flags activate it so empty domains do not add placeholder work; and which
   focused tests prove default `World::step()` and custom-final-stage stepping
   share the same dynamics schedule without exceeding the built-in inline stage
   capacity.
