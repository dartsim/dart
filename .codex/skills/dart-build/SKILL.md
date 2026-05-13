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
pixi run configure    # Configure CMake
pixi run build        # Build
pixi run test         # Run tests
pixi run test-all     # Full validation (lint + build + tests)
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
| Build failure      | `pixi run configure` then `pixi run build` |
| Linking error      | Check CMakeLists.txt in relevant module    |

## Pixi Lockfiles

- Do not edit `pixi.lock` by hand. Change dependency constraints in
  `pixi.toml`, then run a Pixi command such as `pixi install` to let Pixi
  regenerate the lockfile.
- If `pixi.lock` changes format or has broad solver churn, keep the generated
  output and call it out in the PR rather than attempting to splice lockfile
  entries manually.

## Filament GUI Visual Checks

- For Filament GUI rendering changes, run a bounded headless capture through
  `pixi run ex filament_gui --headless --frames ... --width ... --height ... --screenshot ...`.
  Prefer at least 1280px width when judging visual quality.
- Inspect the rendered image yourself; do not rely only on command success.
  `examples/filament_gui/analyze_headless_smoke.py` is useful for a smoke
  signal, but it is not a substitute for visual inspection.
- If the user explicitly asks for `pixi run ex filament_gui`, also run that
  exact entry point and terminate it after confirming the GUI binary launches.

## Key Files

- `pixi.toml` - Package management
- `CMakeLists.txt` - Build configuration
- `cmake/` - CMake modules
