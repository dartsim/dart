---
name: dart-io
description: DART model loading - URDF, SDF, MJCF, SKEL parsers and dart::io unified API
---

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

## Supported Formats

| Format | Extension | Use Case |
|--------|-----------|----------|
| URDF | `.urdf` | ROS robots, most common |
| SDF | `.sdf`, `.world` | Gazebo models |
| MJCF | `.xml` | MuJoCo models |
| SKEL | `.skel` | Legacy DART format |

## Key APIs

- `dart::io::readWorld(uri, options)` - Load complete simulation world
- `dart::io::readSkeleton(uri, options)` - Load single robot/model
- `dart::io::ModelFormat` - Force specific format
- `dart::io::ReadOptions` - Configuration options

## Common Patterns

### URDF with Package Resolution

```cpp
dart::io::ReadOptions options;
options.addPackageDirectory("my_robot", "/path/to/my_robot");
auto skel = dart::io::readSkeleton("package://my_robot/urdf/robot.urdf", options);
```

### Force Format (Ambiguous Extension)

```cpp
dart::io::ReadOptions options;
options.format = dart::io::ModelFormat::Sdf;
auto world = dart::io::readWorld("file:///path/to/model.xml", options);
```

### SDF Default Root Joint

```cpp
dart::io::ReadOptions options;
options.format = dart::io::ModelFormat::Sdf;
options.sdfDefaultRootJointType = dart::io::RootJointType::Fixed;
auto world = dart::io::readWorld(uri, options);
```

### Custom Resource Retriever

```cpp
dart::io::ReadOptions options;
options.resourceRetriever = myRetriever;
auto skel = dart::io::readSkeleton(uri, options);
```

## Python Usage

```python
import dartpy as dart

parser = dart.io.UrdfParser()
robot = parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
```

## Key Files

- API: `dart/io/Read.hpp`
- Implementation: `dart/io/Read.cpp`
- Tests: `tests/unit/io/test_Read.cpp`

## Design Notes

- One front door (`dart::io`), not one mega-parser
- Format-specific features: use underlying parser directly (e.g., `dart::utils::UrdfParser`)
- SKEL is legacy XML format; prefer URDF/SDF/MJCF for new models
