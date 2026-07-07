# dart/io/

Agent guidelines for the IO/parsing module.

## Overview

Unified model loading API for URDF, SDF, and MJCF formats.

## Supported Formats

| Format | Function                   | Notes         |
| ------ | -------------------------- | ------------- |
| URDF   | `dart::io::readSkeleton()` | ROS standard  |
| SDF    | `dart::io::readSkeleton()` | Gazebo format |
| MJCF   | `dart::io::readSkeleton()` | MuJoCo format |

## Key Concepts

- **Unified API**: `dart::io::readSkeleton(path, options)` auto-detects format
- **ReadOptions**: Package directories, resource retrieval callbacks
- **Package resolution**: `package://` URIs for ROS compatibility
- **SKEL removal**: DART 7 removed legacy SKEL support; use `release-6.*`
  when parity evidence requires old SKEL behavior.

## Code Patterns

```cpp
dart::io::ReadOptions options;
options.addPackageDirectory("my_robot", "/path/to/my_robot");
auto skeleton = dart::io::readSkeleton("robot.urdf", options);
```

## Internal Parsers

Located in `dart/io/`:

- `urdf/urdf_parser.hpp` - URDF parsing
- `sdf/sdf_parser.hpp` - SDF parsing
- `mjcf/mjcf_parser.hpp` - MJCF parsing

## Testing

Unit tests: `tests/unit/io/`
Integration tests: `tests/integration/test_*Parser*.cpp`

## See Also

- @docs/onboarding/io-parsing.md - Detailed parsing documentation
- @data/ - Sample model files for testing
