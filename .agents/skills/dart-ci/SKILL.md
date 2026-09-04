---
name: dart-ci
description: "DART CI: GitHub Actions, cache debugging, and platform-specific failures"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-ci/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART CI/CD Troubleshooting

Load this skill when debugging CI failures or working with GitHub Actions.

When the failing claim depends on 3D structure or behavior, also load
`dart-verify-sim` and reproduce it with a text oracle plus assessed visual
evidence, or record why that renderer is unavailable in the failing environment.

## Full Documentation

For complete CI/CD guide: `docs/onboarding/ci-cd.md`

## Common Failure Modes

| Failure Type         | Solution                                                 |
| -------------------- | -------------------------------------------------------- |
| Formatting fails     | `pixi run lint`; push only after approval                |
| Codecov patch fails  | Inspect coverage upload/reporting before adding tests    |
| FreeBSD RTTI fails   | Use type enums + `static_cast` instead of `dynamic_cast` |
| macOS ARM64 SEGFAULT | Replace `alloca()`/VLAs with `std::vector<T>`            |
| RTD build fails      | Use defensive `.get(key, default)` patterns              |
| gz-physics fails     | Reproduce with `pixi run -e gazebo test-gz`              |

## CUDA Runner Policy

The project has a trusted `ubuntu-latest-gpu` runner for same-repository CUDA
runtime validation, but it must never run untrusted fork-PR code. Consequences:

- Same-repository PRs, protected branch pushes, and manual dispatches use the
  GPU runner and run `pixi run --locked -e cuda test-cuda`.
- Fork PRs use a GitHub-hosted fallback and compile CUDA targets without
  running GPU-only steps.
- Local CUDA validation is `pixi run -e cuda test-all` on Linux hosts with a
  visible NVIDIA CUDA runtime; local Pixi config auto-detects visible GPU
  compute capabilities for `DART_CUDA_ARCHITECTURES`.
- `pixi run check-phase5-cuda-workflow` enforces the trusted-event GPU guard
  and fork-PR hosted fallback in `ci_cuda.yml`.

## Caching And Timing

Use the cache policy, current timing guidance, and investigation steps in
`docs/onboarding/ci-cd.md`. Treat observed job duration and cache diagnostics
from the affected run as the current evidence; do not copy mutable timing or
hit-rate estimates into this skill.
