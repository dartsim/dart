# Resume: Contact-Aware Inverse Dynamics + IK Utilities (DART 6.19)

## Last Session Summary

Branch created from `origin/release-6.19`; subsystem mapping and Release build
in progress. No code changes yet.

## Current Branch

`feature/contact-inverse-dynamics-6.19` — clean (only this dev-task folder
added).

## Immediate Next Step

Write `01-design.md` from the subsystem-mapping results, then implement the
contact-aware inverse dynamics API test-first.

## Context That Would Be Lost

- Feature gap: DART 6 ID gives generalized forces only; users need a built-in
  way to obtain joint torques + friction-consistent contact forces for tracked
  motions with contacts (currently requires an external QP).
- Deliverable gates: unit tests + benchmarks + ImGui/OSG GUI example + dartpy
  bindings; additive-only API (gz-physics LTS compatibility).
- Build/test entry points on this branch: `pixi run config|build|test|test-all`,
  `pixi run lint`, benchmarks via `pixi run bm*`, examples via `pixi run ex ...`.

## How to Resume

```bash
git checkout feature/contact-inverse-dynamics-6.19
git status && git log -3 --oneline
```

Then: continue from "Immediate Next Steps" in `README.md`.
