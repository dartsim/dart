# dart/io/

Agent guidelines for the IO/parsing module.

## Overview

Unified model loading API for URDF, SDF, MJCF, and SKEL formats.

## Supported Formats

| Format | Function                   | Notes             |
| ------ | -------------------------- | ----------------- |
| URDF   | `dart::io::readSkeleton()` | ROS standard      |
| SDF    | `dart::io::readSkeleton()` | Gazebo format     |
| MJCF   | `dart::io::readSkeleton()` | MuJoCo format     |
| SKEL   | `dart::io::readSkeleton()` | DART legacy (XML) |

## Key Concepts

- **Unified API**: `dart::io::readSkeleton(path, options)` auto-detects format
- **ReadOptions**: Package directories, resource retrieval callbacks
- **Package resolution**: `package://` URIs for ROS compatibility

## Code Patterns

```cpp
dart::io::ReadOptions options;
options.addPackageDirectory("my_robot", "/path/to/my_robot");
auto skeleton = dart::io::readSkeleton("robot.urdf", options);
```

## Internal Parsers

Located in `dart/utils/`:

- `urdf/UrdfParser.hpp` - URDF parsing
- `sdf/SdfParser.hpp` - SDF parsing
- `mjcf/MjcfParser.hpp` - MJCF parsing
- `skel/SkelParser.hpp` - SKEL parsing

## Testing

Unit tests: `tests/unit/io/`
Integration tests: `tests/integration/test_*Parser*.cpp`

## See Also

- @docs/onboarding/io-parsing.md - Detailed parsing documentation
- @data/ - Sample model files for testing
