# Codex Resume Prompt

Use this prompt when starting a new Codex agent on this task.

```
You are a coding agent working in /home/js/dev/dartsim/dart/dart7 on the
"Persistent contact support" task (working title: ContactPatchCache).

Goal: Add a DART-level persistent per-pair contact set (max 4 points) to reduce
contact jitter, with a runtime feature flag default OFF. Keep backends as raw
contact providers. Preserve CollisionResult semantics when the flag is OFF.

Status:
- Stage 0-5 complete (docs, implementation, tests, GUI demo, benchmarks)

Key docs to read:
- docs/dev_tasks/contact_manifold/00_overview.md
- docs/dev_tasks/contact_manifold/01_design.md
- docs/dev_tasks/contact_manifold/02_integration_plan.md
- docs/dev_tasks/contact_manifold/03_testing.md
- docs/dev_tasks/contact_manifold/04_benchmarking.md
- docs/dev_tasks/contact_manifold/05_demo.md
- CONTRIBUTING.md and docs/onboarding/building.md, testing.md

Key code entry points:
- dart/constraint/ConstraintSolver.cpp (updateConstraints)
- dart/constraint/ConstraintSolver.hpp
- dart/constraint/ContactPatchCache.hpp/.cpp
- dart/collision/CollisionResult.hpp/.cpp
- dart/simulation/World.cpp (CollisionResult usage)
- dart/collision/ode/OdeCollisionDetector.cpp (backend contact history)

Decisions so far:
- Name chosen: ContactPatchCache
- Use a separate persistent contact list for constraints; do not change
  CollisionResult behavior
- Soft contacts remain on the legacy path initially
- Determinism requires stable ordering (no unordered iteration in output)
- Cached contacts are emitted for unseen pairs up to maxSeparationFrames

Constraints:
- No mention of any external reference project in code or docs
- Keep feature OFF by default
- Add tests, benchmarks, and GUI demo per the Stage plan

Next actions:
1) Review open issues: contact force reporting when cache is enabled
```
