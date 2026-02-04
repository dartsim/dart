---
name: dart-build
description: DART build system knowledge - CMake, pixi, dependencies, troubleshooting
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

## Key Files

- `pixi.toml` - Package management
- `CMakeLists.txt` - Build configuration
- `cmake/` - CMake modules
