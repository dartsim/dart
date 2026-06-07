Public API Boundaries
=====================

DART 7 keeps the user-facing API intentionally smaller than the implementation
surface. New user code should prefer documented headers, tutorials, and
``dartpy`` symbols. Names under ``detail`` or ``internal`` are implementation
details and may change without a migration window.

What Is Supported
-----------------

Supported public APIs are documented, tested, and intended to remain stable
within a major release line. In C++, they live in public module headers such as
``dart/dynamics/*``, ``dart/simulation/*``, and ``dart/io/*``. In Python, they
appear through ``dartpy`` and its documented submodules.

DART may also keep compatibility APIs for existing downstream projects. These
APIs can be deprecated and replaced by newer DART 7 interfaces, but they remain
available until the documented removal point.

Some legacy public headers expose implementation types through aliases, base
classes, or signatures. Treat those names as compatibility debt rather than new
API to build on: prefer the documented replacement when one exists, and expect
them to move behind public wrappers over time.

What Is Internal
----------------

Do not build application code against:

* headers or namespaces named ``detail`` or ``internal``;
* implementation headers such as ``*-impl.hpp``;
* component storage under ``dart/simulation/comps``;
* backend adapters that are not documented extension points.

These names exist so DART can change implementation strategy, reduce
dependencies, and improve performance without forcing user migrations.

Python API
----------

``dartpy`` exposes the supported user-facing API rather than every C++ class.
If a C++ detail type is missing from Python, that is usually intentional. Prefer
filing an issue that describes the workflow you need instead of relying on a
private C++ implementation detail.

Migration Guidance
------------------

When a documented public API changes, the release notes and migration guide
describe the replacement. Compatibility shims may emit deprecation warnings
before removal in a later major release.
