# Build System Notes

DART 6.20 uses CMake through Pixi tasks. Dependency cleanup must preserve
exported CMake package behavior unless a maintainer explicitly accepts a
compatibility break.

Important checks:

- `pixi run config`
- `pixi run build`
- `pixi run test`
- `pixi run -e gazebo test-gz` for Gazebo/gz-physics compatibility surfaces

When moving optional dependencies out of the default environment, verify both
dependency-present and dependency-absent configure paths where practical.
