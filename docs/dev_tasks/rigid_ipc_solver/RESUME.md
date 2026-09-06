# Resume: Rigid IPC Solver

## Current Reality (2026-09-04)

Use this folder's `README.md`, `docs/plans/dashboard.md`, and the current code as
the live status. Every earlier rigid IPC feature branch has landed on `main` or been retired
from `origin` (no `feature/rigid-ipc*` branch remains); per-slice session notes
live in git history. Current rigid IPC work should keep
using the DART-owned
`RigidBodySolver::Ipc` method-family opt-in, the shared built-in World step
schedule, and the open README/dashboard next steps: robust normal-push for
kinematic obstacles, the performance climb, remaining corpus/parity coverage,
and articulated-scene support without exposing solver registries, ECS storage,
external project names, or backend resources in public API.

## How To Resume

1. `git fetch origin main && git status --short --branch`; start new slices
   from current `main` (no rigid IPC feature branch is open).
2. Read `README.md` here: "Current Status" for the phase checklist,
   "Immediate Next Steps" for the ordered slices, and "Verification" for the
   manifest and test commands plus the pinned upstream commit.
3. Update the PLAN-082 dashboard entry and this snapshot when a slice lands;
   do not append session logs here.
