---
name: dart-io
description: "DART IO: URDF, SDF, MJCF parsers, and dart::io loading"
---

# DART Model Loading (`dart::io`)

Load this skill when working with robot model files or parsers.

When correctness depends on the loaded model's 3D hierarchy, transforms,
collision geometry, or simulated behavior, also load `dart-verify-sim` for a
text scene/behavior oracle plus claim-tied visual corroboration.

## Quick Start

```cpp
#include <dart/io/read.hpp>

// Format auto-detection
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

- API: `dart/io/read.hpp`
- Tests: `tests/unit/io/test_read.cpp`
