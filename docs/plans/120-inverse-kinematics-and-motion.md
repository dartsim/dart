# PLAN-120: Inverse Kinematics And Motion Synthesis

- Operating state: `PLAN-120` in [`dashboard.md`](dashboard.md)
- Outcome: the DART 7 `World` gains a DART-owned inverse-kinematics
  surface that covers the useful DART 6 IK behaviors, exposes a portfolio of
  pose and motion-level IK methods, and lets users either choose a method
  explicitly or ask for an `auto` policy that selects and explains a reasonable
  method for the current robot, target set, constraints, seeds, and recent trial
  evidence. The design must treat IK as more than independent per-frame pose
  solving: motion-level solves need continuity, singularity avoidance,
  local-minimum escape, collision/limit handling, and manifold-correct state
  distances.
- Current evidence:
  - Classic DART already has the first parity target:
    `dart/dynamics/inverse_kinematics.hpp` provides `InverseKinematics`,
    task-space regions, objectives, null-space objectives, DOF selection,
    Jacobian transpose, damped least squares, analytical methods, and
    `IKFast`; `dart/dynamics/hierarchical_ik.hpp` provides `CompositeIK` and
    `WholeBodyIK` with hierarchy-level/null-space behavior.
  - Classic DART already protects some non-Euclidean IK math: Jacobian methods
    call `convertJacobianMethodOutputToGradient(...)` because FreeJoint and
    BallJoint cannot be integrated by plain vector addition.
  - The DART 7 API design already reserves the right seams:
    kinematics-only `World::sync(WorldSyncStage::Kinematics)`, future explicit
    state spaces, functional rollouts, collision-query stages, and a solver
    capability matrix in
    [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md)
    and
    [`../design/simulation_python_api.md`](../design/simulation_python_api.md).
  - PLAN-100 records the DART 7 Lie-group surface in
    `dart/math/lie_group/`; IK must use the same manifold/retraction
    conventions for distance, interpolation, and update, not Euler-angle or
    raw Euclidean shortcuts.

## Owner Docs

- Public facade rules:
  [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md),
  [`../design/simulation_python_api.md`](../design/simulation_python_api.md)
- Solver architecture:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Lie-group owner:
  [`../onboarding/architecture.md#12-math-module-dartmath`](../onboarding/architecture.md#12-math-module-dartmath)
  and PLAN-100 in [`dashboard.md`](dashboard.md)
- Research references:
  [`nakamura-1987-task-priority`](../readthedocs/papers.md#nakamura-1987-task-priority),
  [`buss-kim-2005-sdls`](../readthedocs/papers.md#buss-kim-2005-sdls),
  [`aristidou-lasenby-2011-fabrik`](../readthedocs/papers.md#aristidou-lasenby-2011-fabrik),
  [`beeson-ames-2015-tracik`](../readthedocs/papers.md#beeson-ames-2015-tracik),
  [`starke-2020-bioik`](../readthedocs/papers.md#starke-2020-bioik),
  [`rakita-2018-relaxedik`](../readthedocs/papers.md#rakita-2018-relaxedik),
  [`zucker-2013-chomp`](../readthedocs/papers.md#zucker-2013-chomp),
  [`mukadam-2018-gpmp2`](../readthedocs/papers.md#mukadam-2018-gpmp2),
  and [`ames-2022-ikflow`](../readthedocs/papers.md#ames-2022-ikflow)
- Future durable design doc, before implementation starts:
  `docs/design/inverse_kinematics_motion.md`

## Scope

This plan covers IK and kinematic motion synthesis for the DART 7
simulation stack. It is an algorithm-extensibility plan, not a near-term DART 7
promotion promise.

It includes:

- pose IK for one or more task targets;
- whole-body and hierarchical/task-priority IK parity with classic DART where
  the DART 7 `World` owns matching topology and state;
- analytical, Jacobian-based, optimization-based, heuristic/evolutionary,
  statistical, and learned proposal methods behind DART-owned capability names;
- motion-level IK over a horizon, including continuity and feasibility costs;
- an `auto` selector with deterministic diagnostics and benchmark evidence;
- C++ and dartpy surfaces that share the same capability model.

It does not include:

- replacing full dynamics control, inverse dynamics, or trajectory optimization
  under actuation/contact constraints;
- exposing solver registries, plugin loaders, backend task systems, learned
  model internals, or ECS storage as public API;
- making learned/AI solvers default before datasets, error bounds, fallback
  behavior, and deterministic diagnostics are proven.

## Method Families To Evaluate

| Family                              | DART role                                                                                                                                                                                                               | First design question                                                                                                    |
| ----------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| Classic whole-body/task-priority IK | Parity foundation. Preserve DART 6 `WholeBodyIK`, `CompositeIK`, hierarchy levels, null-space objectives, and task-space regions where DART 7 topology supports them.                                                   | What public task and constraint value objects replace the classic `Skeleton`/`JacobianNode` attachment model?            |
| Jacobian methods                    | Required local baseline. Start from Jacobian transpose and damped least squares, then evaluate selectively damped or dynamically weighted variants for singularities and limits.                                        | Which manifold tangent, damping, weighting, and trust-region rules are stable enough to expose?                          |
| Analytical methods                  | Fast exact branch when robot geometry supports it. Keep `IKFast`/generated-code parity as a bridge, but expose analytical capability rather than an engine-named public solver.                                         | How are multiple analytical branches scored against continuity, limits, collision, and user preference?                  |
| Constrained optimization            | Generic constrained baseline. Evaluate SQP/QP/TRAC-IK-like approaches for joint limits, collision, manipulability, task priorities, and secondary objectives.                                                           | Which constraints must be common DART task objects rather than solver-specific callback functions?                       |
| Heuristic and evolutionary methods  | Global/multi-objective fallback. Evaluate FABRIK/CCD-style fast heuristics and BioIK-style memetic optimization as seed generators or direct solvers.                                                                   | Which methods remain chain-only, and which can honestly support tree, whole-body, closed-chain, or multi-effector tasks? |
| Statistical trajectory methods      | Motion-level prior. Evaluate GPMP2-style Gaussian-process priors for continuous-time smoothness, replanning, and uncertainty-aware waypoint interpolation.                                                              | What state-space metric and collision-query interface make the prior manifold-correct?                                   |
| Learned proposal methods            | Deferred accelerator for diverse seeds, not a correctness source. Evaluate IKFlow-like generators only as proposal distributions refined by checked DART solvers.                                                       | What datasets, robot-family generalization tests, fallback rules, and model-version diagnostics are required?            |
| Auto policy                         | User-facing selector, not hidden magic. Select, rank, or fall back among methods from robot dimensions, task count, joint manifold, limits, constraints, warm-start history, requested horizon, and pre-trial outcomes. | What diagnostics prove why `auto` chose a method and when it tried or rejected alternatives?                             |

## Workstreams

0. **Design inventory and benchmark packet** - map classic DART IK features to
   DART 7 `World` concepts; define a small scene set covering open-chain
   arms, whole-body/multi-effector targets, redundant arms, ball/free joints,
   joint limits, near-singular poses, unreachable targets, and collision-query
   tasks. Create `docs/design/inverse_kinematics_motion.md` before code starts.
1. **State-space and task model** - define manifold-aware state, tangent,
   distance, interpolation, and retraction contracts. IK distances live in
   tangent spaces (`SO(3)`, `SE(3)`, and product groups), with explicit weights
   and units; raw Euler-angle or simple Euclidean position differences are not
   acceptable for orientation or free-joint motion.
2. **Pose IK parity foundation** - implement the DART 6 parity slice for
   task-space targets, DOF selection, joint limits, seeds, objective and
   null-space objective handling, whole-body/hierarchical composition, and
   analytical-solver branch scoring where supported.
3. **Solver portfolio** - add method families only behind documented capability
   values. Each method records supported topology, coordinate support,
   constraints, determinism, differentiability, execution shape, failure modes,
   and fallback behavior.
4. **Motion-level IK** - add horizon-based solves that optimize a trajectory or
   rollout rather than solving frames independently. Required costs include
   target error, joint displacement, velocity/acceleration or smoothness,
   singularity/manipulability margin, joint/closure/collision limits, and
   branch-continuity penalties for analytical or multi-solution methods.
5. **Auto selection and pre-trials** - implement `auto` as an evaluator-driven
   policy that can run bounded pre-trials, score candidates, cache recent
   outcomes, and return diagnostics. It must be deterministic under a supplied
   seed and must expose when it falls back to a safer method.
6. **C++/Python promotion** - align C++ and dartpy naming, examples, docs,
   serialization, and error reporting. Python may expose ergonomic builders, but
   it must not expose C++ solver registries, backend names, or learned-model
   runtime internals.

## Acceptance Criteria

- The design inventory names every classic DART IK feature that is carried
  forward, intentionally changed, deferred, or rejected, with a migration note
  for users of `InverseKinematics`, `WholeBodyIK`, `CompositeIK`, and `IKFast`.
- State-space tests prove manifold-correct integration, interpolation, and
  distance for revolute/prismatic chains, spherical joints, free joints, and
  mixed product spaces; pose and motion costs use those operations.
- Pose IK parity tests cover task-space regions, Jacobian transpose, damped
  least squares, whole-body hierarchy/null-space behavior, seeds, joint limits,
  analytical multi-branch scoring, unreachable targets, and deterministic
  failure diagnostics.
- Motion-level IK is not complete until long-horizon tests show continuity
  across equivalent analytical branches, warm-started and cold-started
  trajectories, near-singular targets, local-minimum escape attempts,
  collision/closure constraints when those owners exist, and no per-frame
  discontinuity hidden by interpolation.
- Each solver family has a benchmark row reporting solve time, success rate,
  final task error, limit/collision violations, singularity margin, trajectory
  smoothness, branch switches, and memory allocations on the shared scene set.
- `auto` is complete only when it is explained by a checked scoring policy,
  stable under a seed, benchmarked against explicit method choices, and able to
  return a structured reason for each accepted, rejected, or fallback method.
- Public APIs remain backend-neutral and method-family oriented. They do not
  expose solver registry types, ECS components, generated-code internals,
  learned-model tensor backends, or implementation-specific cache ownership;
  `check-api-boundaries` stays green when code exists.
- C++ and Python examples cover at least one pose IK solve, one whole-body
  multi-target solve, one analytical or generated-code branch, and one
  motion-level solve over a horizon.

## Revision Triggers

- Experimental `World` gains or changes public multibody topology, state-space,
  collision-query, closure-projection, rollout, or model-loading APIs.
- PLAN-080 changes articulated-body state ownership or Jacobian availability.
- PLAN-100 changes Lie-group tangent/retraction semantics.
- A benchmark shows a method family is not competitive enough to keep, or a
  learned/statistical method becomes reliable enough to promote from deferred
  proposal generation to a checked solver family.
- A maintainer changes the DART 8 promotion scope for kinematics-only,
  planning, rollout, or optimization APIs.
