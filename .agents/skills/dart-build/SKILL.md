---
name: dart-build
description: "DART Build: CMake, pixi, dependencies, and build troubleshooting"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-build/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART Build System

Load this skill when working with DART's build system.

## Quick Commands

```bash
pixi run config       # Configure CMake
pixi run build        # Build
pixi run test         # Run tests
pixi run test-all     # Build all CMake targets
pixi run -e gazebo test-gz # Gazebo/gz-physics compatibility gate
pixi run lint         # Format code
```

## Full Documentation

For complete build instructions, read: `docs/onboarding/building.md`

For build system internals (CMake, dependencies): `docs/onboarding/build-system.md`

## Common Issues

| Issue              | Solution                                   |
| ------------------ | ------------------------------------------ |
| CMake not found    | `pixi install`                             |
| Missing dependency | Check `pixi.toml`                          |
| Build failure      | `pixi run config` then `pixi run build` |
| Linking error      | Check CMakeLists.txt in relevant module    |

## Release Branch Notes

- This DART 6.20 support branch prioritizes compatibility with installed
  headers, package exports, and downstream Gazebo/gz-physics behavior.
- For package, collision, constraint, or dependency changes that could affect
  downstream users, run the Gazebo gate when practical:
  `pixi run -e gazebo test-gz`.

## Pixi Lockfiles

- Do not edit `pixi.lock` by hand. Change dependency constraints in
  `pixi.toml`, then run a Pixi command such as `pixi install` to let Pixi
  regenerate the lockfile.
- If `pixi.lock` changes format or has broad solver churn, keep the generated
  output and call it out in the PR rather than attempting to splice lockfile
  entries manually.

## GUI Visual Checks

- For OSG GUI, scene, or simulation rendering changes, load `dart-verify-sim`.
  Pair a focused text correctness oracle with `agent-capture`, view assessment,
  and claim-relevant core `DebugOverlay` layers; inspect the artifact and
  sidecar instead of relying only on command success.

## Key Files

- `pixi.toml` - Package management
- `CMakeLists.txt` - Build configuration
- `cmake/` - CMake modules
