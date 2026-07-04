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
state.

The initial writer covers BodyNode links, root FreeJoint/WeldJoint placement,
revolute/prismatic/weld child joints, inertial parameters, and
box/sphere/cylinder/mesh visual or collision geometry with explicit visual
material colors. It is not a general project save/load format and does not make
YAML a model format. Use the writer only when the target scene fits the
documented subset, and expand the round-trip tests before broadening the
supported contract.

## Notes about Python

The consolidated API is primarily for **C++** (`dart::io`). However, the Python
bindings (`dartpy`) also expose parsers under `dart.io` (e.g.
`dart.io.UrdfParser`). The legacy `dart.utils.*` parsers are deprecated and
should be avoided in new code. Treat SKEL as a DART 6 compatibility format and
use a `release-6.*` branch for old SKEL assets.
