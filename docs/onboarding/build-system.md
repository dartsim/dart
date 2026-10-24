# Build System Notes

DART 6 uses CMake through Pixi tasks. Dependency cleanup must preserve
exported CMake package behavior unless a maintainer explicitly accepts a
compatibility break.

Important checks:

- `pixi run config`
- `pixi run build`
- `pixi run test`
- `pixi run -e gazebo test-gz` for Gazebo/gz-physics compatibility surfaces

When moving optional dependencies out of the default environment, verify both
dependency-present and dependency-absent configure paths where practical.

## Bullet Precision

DART 6.20 preserves its existing system Bullet library selection by default.
Set `DART_BULLET_FLOAT64=ON` to prefer available `-float64` libraries with
Bullet 3.17 or newer. Double precision fixes centimeter-scale shapes losing
contacts on heightmaps ([gz-physics#1117](https://github.com/gazebosim/gz-physics/issues/1117)).
Unsuffixed Bullet libraries may themselves use either scalar precision.

Changing scalar precision changes the native Bullet ABI exposed by DART.
Rebuild consumers of DART's Bullet headers, conversion functions, or native
object accessors when opting in. A process must load only one Bullet
precision, including other physics plugins. The default `OFF` preserves
unsuffixed-library selection and existing DART 6.20 binary compatibility.

The installed `collision-bullet` CMake component checks that consumers select
the same Bullet filename variant, scalar precision, and minimum version as
DART. A build using Bullet 3.17 or newer requires at least 3.17 in its
consumers because it uses the typed heightmap constructor. The core `dart.pc`
continues to describe `dart`; Bullet remains a separate collision component.

On CMake 3.24 or newer, built Bullet targets with direct `$<LINK_ONLY:...>`
dependencies remain unverified because their creation-time CMP0131 policy is
unknown. Interface-only link dependencies remain supported. Older CMake uses
its fixed legacy compile-usage behavior.
