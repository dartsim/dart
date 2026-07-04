# Unified Skeleton Loading (`dart::io`)

## Why this exists

DART historically exposed multiple model-loading entry points with inconsistent
naming and structure (e.g., `SdfParser`, `UrdfParser`, and legacy loaders).

For most applications, this made “load a model from a URI” harder than it needed
to be:

- Call sites had to choose a parser type up-front.
- Format inference and error handling were duplicated.
- The “standard” API varied across examples/tests/tutorials.

The `dart::io` component provides a consolidated front door for reading
`Skeleton`s while keeping the underlying format-specific parsers available when
needed. The DART 6 whole-`World` front door was retired from `main` during the
DART 7 API promotion; use a `release-6.*` branch when parity evidence requires the
old public whole-world loader.

## Task context (issue #604)

This unified API and documentation were introduced while addressing issue #604
(parser naming) and migrating tests/examples/tutorials to the `dart::io`
component APIs. The DART 7 surface keeps a single, consistent skeleton-loading
front door (`dart::io`) without introducing new nested namespaces, while keeping
parser-specific customization available via `dart::utils::*` parsers where
needed. DART 7 removed SKEL rather than redesigning it as YAML; migrate legacy
SKEL assets to URDF, SDF, or MJCF. Unit coverage for the promoted options
lives in `tests/unit/io/test_read.cpp`.

## Scope and design decision

### One front door, not one mega-parser

`dart::io::readSkeleton` is the preferred entry point for new C++ code.
Internally it delegates to the format-specific parsers in `dart::utils`.
Whole-world loading is not part of the DART 7 public `dart::io` API.

This is intentionally _not_ an attempt to expose every parser-specific knob through one API.
Instead:

- `dart::io::ReadOptions` carries **common** options and a **small set** of
  high-frequency format-specific options that are hard to avoid in real projects.
- If you need a format-specific feature that is not represented in `ReadOptions`, use the
  underlying parser directly (e.g., `dart::utils::UrdfParser`).

This keeps the common-path API simple while preserving full capability for advanced use.

### No new nested namespaces

The unified API lives in `dart::io` (component library: `dart-io`). Adding new nested
namespaces for each format is intentionally avoided.

## Key APIs

- `dart::io::readSkeleton(const common::Uri&, const ReadOptions&)`
- `dart::io::ModelFormat` (format selection or inference)
- `dart::io::ReadOptions` (common + minimal format-specific options)

Source of truth:

- `dart/io/read.hpp`
- `dart/io/read.cpp`
- `dart/utils/sdf/sdf_writer.hpp` for the first parser-specific SDF writer
- `dart/utils/sdf/` uses libsdformat DOM APIs for SDF structure, model/link
  static and self-collision state, world gravity, inertial,
  visual/collision/material, visual shadow and visibility metadata, joint
  topology/pose/axis frame
  resolution/dynamics/limits,
  collision-surface contact bitmask, bounce restitution, ODE friction/slip
  metadata, and supported geometry semantics;
  avoid raw XML-level SDF parsing when sdformat exposes the value. When
  DART must preserve an authored/default distinction or schema field that the
  high-level DOM class does not expose, use a narrow sdformat `Element` bridge:
  `Element::GetExplicitlySetInFile()` for authored/default checks and typed
  `Element::Get<T>` conversion for extension values. Do not add DART-side SDF
  XML tokenization, child-existence probing, or text reparsing.

## Start here next time

- API surface: `dart/io/read.hpp`
- Implementation: `dart/io/read.cpp`
- Unit coverage for `ReadOptions`: `tests/unit/io/test_read.cpp`
- SDF writer API: `dart/utils/sdf/sdf_writer.hpp`
- SDF writer round-trip coverage: `tests/integration/io/test_sdf_writer.cpp`

## Common usage patterns

### Load a skeleton with format inference

```cpp
#include <dart/io/read.hpp>

auto skel = dart::io::readSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
```

### Force a format (when the URI is ambiguous)

```cpp
dart::io::ReadOptions options;
options.format = dart::io::ModelFormat::Sdf;
auto skel = dart::io::readSkeleton("file:///path/to/model.xml", options);
```

### Custom resource retrieval

```cpp
dart::io::ReadOptions options;
options.resourceRetriever = myRetriever;
auto skel = dart::io::readSkeleton(uri, options);
```

### SDF default root joint type for skeleton imports

Some SDF workflows rely on a default root joint type when it is not explicitly
specified by the model. Configure this through `ReadOptions`:

```cpp
dart::io::ReadOptions options;
options.format = dart::io::ModelFormat::Sdf;
options.sdfDefaultRootJointType = dart::io::RootJointType::Fixed;
auto skel = dart::io::readSkeleton("dart://sample/sdf/test/box.sdf", options);
```

### URDF `package://` resolution (parser-specific customization)

URDF commonly references meshes/textures via `package://<name>/...`. Register package
directories via `ReadOptions`:

```cpp
dart::io::ReadOptions options;
options.addPackageDirectory("my_robot", "/path/to/my_robot");
auto skel = dart::io::readSkeleton("package://my_robot/urdf/robot.urdf", options);
```

This is the `dart::io` equivalent of `dart::utils::UrdfParser::addPackageDirectory`.

## When to add a new option to `ReadOptions`

Prefer adding an option to `ReadOptions` only when:

- It is required for common real-world usage (not just a niche workflow), and
- The option can be expressed without forcing callers to include format-specific parser
  headers at call sites, and
- It does not meaningfully complicate the default/simple usage.

Otherwise, prefer using the underlying parser API directly.

## SDF writing

DART 7's first export surface is parser-specific:
`dart::utils::SdfParser::tryWriteSkeletonToString`. It serializes a
conservative `Skeleton` subset to SDF text and returns `common::Result` so
unsupported constructs produce explicit errors instead of silently dropping
state. The writer builds libsdformat DOM objects and serializes through
sdformat; DART owns the `Skeleton`-to-SDF semantic mapping and diagnostics.

The initial writer covers BodyNode links, root FreeJoint/WeldJoint placement,
explicit parent-world root joints for supported SDF joint types (revolute,
continuous, prismatic, screw, universal, and topology-only ball), and
revolute/continuous/prismatic/weld/screw/universal child joints with passive
dynamics metadata (damping, Coulomb friction, spring reference, and spring
stiffness), sdformat-normalized screw thread pitch (legacy `<thread_pitch>`
before SDF 1.10 and modern `<screw_thread_pitch>` for SDF 1.10+), SDF 1.11+
mimic metadata for axis/axis2 follower joints with motor enforcement,
topology-only ball child joints, model self-collision state, link gravity mode,
non-default skeleton gravity through a root SDF `<world><gravity>` value,
inertial parameters, local joint/shape poses, and
box/sphere/cylinder/capsule/cone/ellipsoid/mesh visual or collision geometry
with visual shadow/hidden/transparency state, explicit visual material colors,
PBR
metallic/roughness factors, and collision-surface contact disable bitmasks,
zero-threshold bounce restitution, and ODE friction/slip metadata. The
round-trip coverage includes absolute
non-file mesh URI preservation through a custom retriever. Targetless relative
mesh references and relative or
host-qualified `file` mesh URIs are rejected because the writer has no
destination SDF URI for resource resolution or generated asset placement.
DART `HeightmapShape` is also rejected with a targeted source-URI/resource
policy diagnostic instead of synthesizing SDF heightmap XML from DART's sampled
grid.
`WriteOptions` can exclude visual or collision entries. Missing mesh URIs,
invalid PBR material factors, invalid collision surface friction or restitution
values, pre-SDF-1.11 mimic output, coupler-style mimic
enforcement, ball joint limits, and ball joint dynamics are reported as
unsupported instead of being silently dropped. It is not a general project
save/load format and does not make YAML a model format. Use the writer only
when the target scene fits the documented subset, and expand the round-trip
tests before broadening the supported contract.

Keep export APIs format-owned until DART has more than one accepted writer
contract. The current SDF writer therefore lives on
`dart::utils::SdfParser`, while `dart::io` remains the read-side skeleton front
door. Do not add `dart::io::writeSkeleton()` or overload `ReadOptions` with
write policy until there is a reviewed multi-format `WriteOptions` design.
DART-owned project or editor save/load belongs in the scene/project layer, not
in interchange-format parser utilities.

## Notes about Python

The consolidated API is primarily for **C++** (`dart::io`). However, the Python
bindings (`dartpy`) also expose parsers under `dart.io` (e.g.
`dart.io.UrdfParser`). The legacy `dart.utils.*` parsers are deprecated and
should be avoided in new code. Treat SKEL as a DART 6 compatibility format and
use a `release-6.*` branch for old SKEL assets.
