---
name: dart-io
description: "DART IO: URDF, SDF, MJCF, SKEL parsers, and dart::utils loading"
---

# DART Model Loading (`dart::utils`)

Load this skill when working with robot model files or parsers.

When the claim depends on the loaded hierarchy, transforms, collision geometry,
or simulated behavior, also load `dart-verify-sim` for a text scene/behavior
oracle plus claim-tied OSG visual corroboration.

## Quick Start

```cpp
#include <dart/utils/urdf/DartLoader.hpp>

dart::utils::DartLoader loader;
auto skel = loader.parseSkeleton(
    "dart://sample/urdf/KR5/KR5 sixx R650.urdf");
```

## Full Documentation

For complete I/O guide: `docs/onboarding/io-parsing.md`

For repository-specific guidance: `AGENTS.md`

## Supported Formats

| Format | Extension        | Use Case      |
| ------ | ---------------- | ------------- |
| URDF   | `.urdf`          | ROS robots    |
| SDF    | `.sdf`, `.world` | Gazebo models |
| MJCF   | `.xml`           | MuJoCo models |
| SKEL   | `.skel`          | Legacy DART   |

## Common Patterns

```cpp
#include <dart/utils/sdf/SdfParser.hpp>

// URDF with package resolution
dart::utils::DartLoader loader;
loader.addPackageDirectory("my_robot", "/path/to/my_robot");
auto skel = loader.parseSkeleton("package://my_robot/urdf/robot.urdf");

// Force specific format
auto world = dart::utils::SdfParser::readWorld("dart://sample/sdf/world.sdf");
```

## Key Files

- URDF API: `dart/utils/urdf/DartLoader.hpp`
- SDF API: `dart/utils/sdf/SdfParser.hpp`
- SKEL API: `dart/utils/SkelParser.hpp`
- Tests: `tests/integration/test_DartLoader.cpp`,
  `tests/integration/test_SdfParser.cpp`, `tests/integration/test_SkelParser.cpp`
