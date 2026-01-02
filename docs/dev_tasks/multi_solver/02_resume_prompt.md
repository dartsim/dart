# Multi Solver Resume Prompt

Use the prompt below in a fresh Codex agent chat to resume this task with
context.

```text
You are Codex, running in the DART repo.

Repo: /home/js/dev/dartsim/dart/opt
Branch: feature/multi_solver-next (tracks origin/feature/multi_solver-next)
Base: origin/main

Goal:
- Orchestrate multiple solvers in World without exposing solver pointers or
  EnTT APIs in the public DART API. Keep Gazebo gate green.

Constraints:
- Use pixi tasks (docs/onboarding/building.md, docs/onboarding/testing.md).
- No public exposure of entt::registry or raw solver pointers.
- Keep docs concise (docs/onboarding/code-style.md).

State summary:
- World routing and solver scheduling live in:
  dart/simulation/World.hpp
  dart/simulation/World.cpp
- Solver base and types:
  dart/simulation/solver/Solver.hpp
  dart/simulation/solver/SolverTypes.hpp
- Classic and ECS solver scaffolding:
  dart/simulation/solver/classic_rigid/*
  dart/simulation/solver/rigid/*
- ECS internals (internal-only):
  dart/simulation/detail/WorldEcsAccess.hpp
  dart/simulation/detail/RigidSolverComponents.hpp
- Object scaffolding:
  dart/simulation/object/Object.hpp
  dart/simulation/object/ObjectWith.hpp (internal-only)
- Tests cover routing and solver enable/disable:
  tests/integration/simulation/test_World.cpp

Recent change:
- RigidSolverType moved to SolverTypes.hpp so World.hpp can forward declare
  Solver and avoid pulling in solver internals.

Docs:
- AGENTS.md
- docs/onboarding/ci-cd.md
- docs/onboarding/build-system.md
- docs/dev_tasks/multi_solver/00_plan.md
- docs/dev_tasks/multi_solver/01_next.md
- docs/dev_tasks/multi_solver/02_next.md

First actions (do in order):
1) git fetch origin
2) git checkout feature/multi_solver-next
3) git status -sb
4) git log --oneline --decorate -n 30
5) git symbolic-ref refs/remotes/origin/HEAD
6) git diff --stat origin/main...HEAD
7) git diff origin/main...HEAD -- dart/simulation/World.hpp dart/simulation/World.cpp

Tests:
- Last run: DART_PARALLEL_JOBS=12 CTEST_PARALLEL_LEVEL=12 pixi run test (passed).
- Gazebo gate not run yet: DART_PARALLEL_JOBS=12 pixi run -e gazebo test-gz.

Next milestone:
- See docs/dev_tasks/multi_solver/02_next.md.
```
