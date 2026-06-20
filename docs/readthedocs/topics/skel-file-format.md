# Legacy SKEL File Format

```{admonition} DART 6 feature — removed in DART 7
:class: caution

SKEL is a **DART 6** format. DART 7 removes SKEL support, so the loader shown
here is not part of the DART 7 API. Use URDF, SDFormat, or MJCF for new models,
or a `release-6.*` branch when you need SKEL. Status is tracked in the
[documentation migration plan](https://github.com/dartsim/dart/blob/main/docs/onboarding/dart7-docs-migration.md).
```

SKEL is DART's legacy XML format for describing worlds, skeletons, robots, and
environments. Its structure is based on [SDFormat](http://sdformat.org/) with a
few DART-specific extensions.

DART 7 is removing SKEL support instead of redesigning SKEL as YAML. Use URDF,
SDFormat, or MJCF for new models, and migrate existing SKEL assets before
depending on DART 7. Use a `release-6.*` branch when you need compatibility with
old SKEL assets.

## Legacy loading (DART 6 / migration only)

On DART 6, `dart::io::readSkeleton` could load a SKEL file from a file path.
Whole-world SKEL loading through the old public DART 6 API remains available on
`release-6.*` branches for parity work.

```cpp
#include <dart/io/Read.hpp>

auto skeleton = dart::io::readSkeleton("path/to/legacy.skel");
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

Historical DART revisions and `release-6.*` branches include sample `.skel`
files under `data/skel/`; treat them as migration references rather than
templates for new models. For the exact set of recognized elements and
attributes, the legacy parser source is authoritative:
`dart/utils/skel_parser.cpp`.
