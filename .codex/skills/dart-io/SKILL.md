---
name: dart-io
description: DART model loading - URDF, SDF, MJCF, SKEL parsers and dart::io unified API
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-io/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART Model Loading (`dart::io`)

Load this skill when working with robot model files or parsers.

## Quick Start

```cpp
#include <dart/io/Read.hpp>

// Format auto-detection
auto world = dart::io::readWorld("dart://sample/skel/chain.skel");
auto skel = dart::io::readSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
```

## Full Documentation

For complete I/O guide: `docs/onboarding/io-parsing.md`

For module-specific details: `dart/io/AGENTS.md`

## Supported Formats

| Format | Extension        | Use Case      |
| ------ | ---------------- | ------------- |
| URDF   | `.urdf`          | ROS robots    |
| SDF    | `.sdf`, `.world` | Gazebo models |
| MJCF   | `.xml`           | MuJoCo models |
| SKEL   | `.skel`          | Legacy DART   |

## Common Patterns

```cpp
// URDF with package resolution
dart::io::ReadOptions options;
options.addPackageDirectory("my_robot", "/path/to/my_robot");
auto skel = dart::io::readSkeleton("package://my_robot/urdf/robot.urdf", options);

// Force specific format
options.format = dart::io::ModelFormat::Sdf;
```

## Key Files

- API: `dart/io/Read.hpp`
- Tests: `tests/unit/io/test_Read.cpp`
