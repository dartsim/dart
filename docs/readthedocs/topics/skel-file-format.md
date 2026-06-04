# SKEL File Format

SKEL is DART's native XML format for describing worlds, skeletons, robots, and
environments. Its structure is based on [SDFormat](http://sdformat.org/) with a
few DART-specific extensions.

DART can also load URDF, SDFormat, and MJCF files — prefer those when you need to
interchange models with other tools. SKEL is convenient when you want a compact,
DART-focused scene or want to exercise DART-specific options directly.

## Loading a SKEL file

A SKEL file is read into a `World` (or, for a single model, a `Skeleton`).
Bundled sample files are addressable with the `dart://sample/...` URI scheme; you
can also pass an ordinary file path.

```cpp
#include <dart/io/read.hpp>
#include <dart/simulation/world.hpp>

auto world = dart::io::readWorld(
    dart::common::Uri("dart://sample/skel/cubes.skel"));
// A single model: dart::io::readSkeleton(uri)
```

```python
import dartpy as dart

world = dart.io.SkelParser.read_world("dart://sample/skel/cubes.skel")
```

## Document structure

A SKEL document has a `<skel>` root containing one `<world>`. A world holds
`<physics>` settings and one or more `<skeleton>` elements; each skeleton is a
tree of `<body>` elements connected by `<joint>` elements.

```text
<skel version="1.0">
  <world>
    <physics> ... </physics>
    <skeleton>
      <body>  ... </body>
      <joint> ... </joint>
    </skeleton>
  </world>
</skel>
```

## Physics

`<physics>` configures the world:

- `<time_step>` — integration step in seconds (e.g. `0.001`).
- `<gravity>` — gravity vector as `x y z` (e.g. `0 -9.81 0`).
- `<collision_detector>` — collision backend, e.g. `dart` or `fcl`.

## Skeletons, bodies, and shapes

Each `<skeleton name="...">` contains bodies and joints. A `<body name="...">`
may specify:

- `<transformation>` — pose as three translation values followed by XYZ Euler
  angles (six numbers); defaults to the identity.
- `<inertia>` — `<mass>`, an inertial `<offset>`, and an optional
  `<moment_of_inertia>`.
- `<visualization_shape>` and `<collision_shape>` — each wraps a `<geometry>`
  (plus an optional `<transformation>` and, for visuals, `<color>`).

`<geometry>` supports `box`, `sphere`, `ellipsoid`, `cylinder`, `capsule`,
`cone`, `pyramid`, `plane`, `multi_sphere`, and `mesh`. Meshes are imported with
[Assimp](https://www.assimp.org/), so any Assimp-supported mesh format can be
referenced.

## Joints

A `<joint type="..." name="...">` connects a `<parent>` and a `<child>` body. Use
`world` as the parent name to attach a body to the world frame.

Supported `type` values are `weld`, `revolute`, `prismatic`, `screw`,
`universal`, `ball`, `euler`, `translational`, `planar`, and `free`.

For joints with movable axes, `<axis>` (and `<axis2>`, `<axis3>` for
multi-DOF joints) carry:

- `<xyz>` — the axis direction.
- `<dynamics>` — `<damping>`, `<friction>`, `<spring_stiffness>`, and
  `<spring_rest_position>`.
- `<limit>` — `<lower>` and `<upper>` bounds.

The joint's `actuator` attribute may be `force` (the default), `passive`,
`servo`, `acceleration`, `velocity`, or `locked`. Initial state can be set with
`<init_pos>` and `<init_vel>`.

## A minimal example

A single free-floating box falling under gravity:

```xml
<?xml version="1.0" ?>
<skel version="1.0">
  <world name="my world">
    <physics>
      <time_step>0.001</time_step>
      <gravity>0 -9.81 0</gravity>
      <collision_detector>dart</collision_detector>
    </physics>

    <skeleton name="box skeleton">
      <body name="box">
        <inertia>
          <mass>1</mass>
        </inertia>
        <visualization_shape>
          <geometry>
            <box><size>0.1 0.05 0.1</size></box>
          </geometry>
          <color>0.8 0.3 0.3</color>
        </visualization_shape>
        <collision_shape>
          <geometry>
            <box><size>0.1 0.05 0.1</size></box>
          </geometry>
        </collision_shape>
      </body>

      <joint type="free" name="joint 1">
        <parent>world</parent>
        <child>box</child>
      </joint>
    </skeleton>
  </world>
</skel>
```

## More examples and the authoritative schema

DART ships dozens of sample `.skel` files under
[`data/skel/`](https://github.com/dartsim/dart/tree/main/data/skel); they are the
best reference for real-world usage. For the exact set of recognized elements and
attributes, the parser source is authoritative:
[`dart/utils/skel_parser.cpp`](https://github.com/dartsim/dart/blob/main/dart/utils/skel_parser.cpp).
