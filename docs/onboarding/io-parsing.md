# Unified Model Loading (`dart::io`)

## Why this exists

DART historically exposed multiple model-loading entry points with inconsistent naming and
structure (e.g., `SkelParser`, `SdfParser`, `UrdfParser`, and legacy loaders).

For most applications, this made “load a model from a URI” harder than it needed to be:

- Call sites had to choose a parser type up-front.
- Format inference and error handling were duplicated.
- The “standard” API varied across examples/tests/tutorials.

The `dart::io` component provides a consolidated front door for reading `World`s and
`Skeleton`s while keeping the underlying format-specific parsers available when needed.

## Task context (issue #604)

This unified API and documentation were introduced while addressing issue #604 (parser naming)
and migrating tests/examples/tutorials to the `dart::io` component APIs. The goal is a single,
consistent “front door” (`dart::io`) without introducing new nested namespaces, while keeping
full parser-specific customization available via `dart::utils::*` parsers. Unit coverage for
the promoted options lives in `tests/unit/io/test_Read.cpp`.

## Scope and design decision

### One front door, not one mega-parser

`dart::io::readWorld` / `dart::io::readSkeleton` are the preferred entry points for new C++
code. Internally they delegate to the format-specific parsers in `dart::utils`.

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

- `dart::io::readWorld(const common::Uri&, const ReadOptions&)`
- `dart::io::readSkeleton(const common::Uri&, const ReadOptions&)`
- `dart::io::ModelFormat` (format selection or inference)
- `dart::io::ReadOptions` (common + minimal format-specific options)

Source of truth:

- `dart/io/Read.hpp`
- `dart/io/Read.cpp`

## Start here next time

- API surface: `dart/io/Read.hpp`
- Implementation: `dart/io/Read.cpp`
- Unit coverage for `ReadOptions`: `tests/unit/io/test_Read.cpp`

## Common usage patterns

### Load with format inference

```cpp
#include <dart/io/Read.hpp>

auto world = dart::io::readWorld("dart://sample/skel/chain.skel");
auto skel = dart::io::readSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
```

### Force a format (when the URI is ambiguous)

```cpp
dart::io::ReadOptions options;
options.format = dart::io::ModelFormat::Sdf;
auto world = dart::io::readWorld("file:///path/to/model.xml", options);
```

### Custom resource retrieval

```cpp
dart::io::ReadOptions options;
options.resourceRetriever = myRetriever;
auto skel = dart::io::readSkeleton(uri, options);
```

### SDF default root joint type

Some SDF workflows rely on a default root joint type when it is not explicitly
specified by the model. Configure this through `ReadOptions`:

```cpp
dart::io::ReadOptions options;
options.format = dart::io::ModelFormat::Sdf;
options.sdfDefaultRootJointType = dart::io::RootJointType::Fixed;
auto world = dart::io::readWorld("dart://sample/sdf/test/force_torque_test.world", options);
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

## Notes about Python

The consolidated API is primarily for **C++** (`dart::io`). However, the Python bindings
(`dartpy`) also expose parsers under `dart.io` (e.g. `dart.io.UrdfParser`). The legacy
`dart.utils.*` parsers are deprecated and should be avoided in new code.
