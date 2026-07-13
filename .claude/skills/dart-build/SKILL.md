---
name: dart-build
description: "DART Build: CMake, pixi, dependencies, and build troubleshooting"
---

# DART Build System

Load this skill when working with DART's build system.

## Quick Commands

```bash
pixi run config       # Configure CMake
pixi run build        # Build
pixi run test         # Run tests
pixi run test-all     # Full validation (lint + build + tests)
pixi run -e cuda test-all # CUDA-enabled full validation on Linux CUDA hosts
pixi run lint         # Format code
```

## Full Documentation

For complete build instructions, read: `docs/onboarding/building.md`

For build system internals (CMake, dependencies): `docs/onboarding/build-system.md`

## Common Issues

| Issue              | Solution                                |
| ------------------ | --------------------------------------- |
| CMake not found    | `pixi install`                          |
| Missing dependency | Check `pixi.toml`                       |
| Build failure      | `pixi run config` then `pixi run build` |
| Linking error      | Check CMakeLists.txt in relevant module |

## CUDA Notes

- On Linux hosts with visible NVIDIA GPUs, the CUDA Pixi config auto-detects
  compute capabilities for `DART_CUDA_ARCHITECTURES`; set
  `DART_CUDA_ARCHITECTURES=<arch>` only when overriding that detected list.
- If CUDA runtime tests fail with unsupported PTX/toolchain errors, verify the
  generated CUDA flags include the local GPU architecture before assuming a
  kernel bug.

## Pixi Lockfiles

- Do not edit `pixi.lock` by hand. Change dependency constraints in
  `pixi.toml`, then run a Pixi command such as `pixi install` to let Pixi
  regenerate the lockfile.
- If `pixi.lock` changes format or has broad solver churn, keep the generated
  output and call it out in the PR rather than attempting to splice lockfile
  entries manually.

## Filament GUI Visual Checks

- For Filament GUI, scene, or simulation rendering changes, load
  `dart-verify-sim`. Pair a text correctness oracle with `agent-capture`, view
  assessment, and claim-relevant engine debug layers; inspect the artifact and
  sidecar instead of relying only on command success.
- If the user explicitly asks for `pixi run ex dartsim`, also run that exact
  entry point and terminate it after confirming the GUI binary launches.

## Key Files

- `pixi.toml` - Package management
- `CMakeLists.txt` - Build configuration
- `cmake/` - CMake modules
