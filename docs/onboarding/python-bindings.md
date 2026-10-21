# Python Bindings

DART Python bindings are built through the Pixi/CMake workflow.

On `main`, `dartpy` is a Python package with one private nanobind extension,
`dartpy._core`, containing core, collision backends enabled in the build, IO
parsers and the Filament GUI bindings (`dartpy.gui`). It links `dart`,
`dart-io` and `dart-gui`. Enabling `DART_BUILD_DARTPY` requires both
`DART_BUILD_IO=ON` and `DART_BUILD_GUI=ON`; for a headless C++ installation,
set `DART_BUILD_DARTPY=OFF`. See
[build system notes](build-system.md) for the C++ package layout and legacy
forwarding targets.

The package layout (DART 7 design decisions D6, D19, D20 and D22):

- `python/dartpy/package/` holds the pure-Python files: `__init__.py`,
  `py.typed` and `experimental/__init__.py`; `dartpy.experimental` is empty
  and outside SemVer. They sit in a subdirectory so that `python/dartpy/`,
  the C++ binding directory, is never importable as the `dartpy` package:
  tests and examples put `python/` on `sys.path` ahead of the build tree.
- `python/dartpy/module.cpp` defines `NB_MODULE(_core)`. The package imports
  the extension while it initializes, and the extension binds every public
  submodule into the package module it finds in `sys.modules`. nanobind
  therefore registers `dartpy.dynamics`, `dartpy.utils.SdfParser` and the
  other submodules in `sys.modules` under their public names, and every class,
  enum and function keeps its canonical public `__module__`
  (`dartpy.dynamics.Skeleton.__module__ == "dartpy.dynamics"`). No
  `dartpy._*` module leaks into a public `__module__`;
  `python/tests/unit/bindings/test_metadata.py` walks the package to check
  this, and `scripts/nanobind/api_surface.py --snapshot` records the public
  surface with its module paths.
- The CMake target is still `dartpy`; it builds `_core` into the staged
  package at `${DART_DARTPY_BUILD_DIR}/dartpy`, and a build step it depends on
  copies the pure-Python files there, so each configuration a multi-config
  generator builds gets a complete package and an edited file re-stages
  without a reconfigure. `DART_DARTPY_BUILD_DIR` is the Python build directory, or its
  `$<CONFIG>` subdirectory for multi-config generators, and `DART_PYTHONPATH`
  is exactly that root. `PYTHONPATH` entries in `pixi.toml` therefore end in
  `python` (Linux and macOS) or `python/$BUILD_TYPE` (Windows).
- Development builds use the version-specific CPython API
  (`DART_DARTPY_STABLE_ABI=OFF`, the default), which is the faster call path
  and the one the test suite exercises; wheels build for the stable ABI.

With `DART_BUILD_UTILS_ASSETS=ON`, `dart-io` also includes the optional online
robot model retriever, exposed as `dartpy.utils.ModelResourceRetriever`.
See [IO and model loading](io-parsing.md#online-robot-models) for its
dependencies and verified-cache workflow.

```bash
pixi run build-py-dev
pixi run test-py
```

For a Python session with the local bindings imported as `dart`, run:

```bash
pixi run dartpy
pixi run dartpy -c "import dartpy; print(dartpy.__file__)"
pixi run dartpy -m pytest python/tests/unit/math/test_random.py -q
```

`dartpy` builds the bindings with `build-py-dev`, sets `PYTHONPATH` to the
development build, and forwards arguments to Python. Interactive sessions
automatically run `import dartpy as dart` through `PYTHONSTARTUP`; commands
using `-c`, `-m`, or a script keep their normal behavior. The first run compiles
the bindings; later runs reuse the incremental build and compiler cache,
rebuilding changed sources as needed. Exit Python with `exit()` or Ctrl-D.
Use `pixi run test-py` for the full guarded Python test suite.

Run tasks that use the same build directory sequentially: simultaneous builds
share outputs and can remove dependency files another compiler-cache process
needs.

If Ninja repeatedly reports `premature end of file; recovering`, stop all
builds using that directory and run
`pixi run ninja -C <build-directory> -t recompact`. The next build regenerates
dependency records; later unchanged builds should report `no work to do`.

Run Python tests when dependency, package, or target changes can alter dartpy
imports, linked components, or installed package behavior.

## Nanobind binder

The current implementation on `main` uses nanobind as the only dartpy binder.
Sources live directly under `python/dartpy/`. The module keeps the DART
namespaces, camelCase names, and overloads, including the interim
`dartpy.gui` module backed by Filament. The DART 7.0 Python contract is defined
in [the DART 7 Python API design](../design/dart7_python_api.md): PEP 8 names
without compatibility aliases and a Python package with a private extension.
M6a landed the renderer in [#3696](https://github.com/dartsim/dart/pull/3696);
M6b still replaces the interim GUI module with `dart.render` and `dart.viewer`
before the first release candidate. `release-6.20` retains pybind11 and OSG.

Python 3.12 or newer and nanobind 3.1 or newer are required.
`DART_USE_SYSTEM_NANOBIND` defaults to `OFF`, fetching nanobind v3.1.0 with its
`robin_map` submodule. With `ON`, CMake uses `nanobind_DIR` when it is set and
otherwise locates nanobind's CMake directory using
`python -m nanobind --cmake_dir`. Set `nanobind_DIR` when the build's Python
cannot import nanobind, as in cross builds and isolated pip builds. Source
builds through `pip` use scikit-build-core (`pyproject.toml`) and keep the
CMake build tree under `build/<wheel tag>`; the `SKBUILD_BUILD_DIR`
environment variable moves it, for example to a larger disk. Tests and
examples import the staged package from `DART_PYTHONPATH`. The
`DART_DARTPY_BINDER` and `DART_USE_SYSTEM_PYBIND11` options are removed.
Passing the legacy binder value `pybind11` fails configure; `nanobind` is
accepted for compatibility and removed from the cache.

### Changes from DART 6.20 (pybind11)

- Python recognizes only the primary C++ base for `isinstance`, `issubclass`,
  and the method resolution order. Secondary-base methods remain callable,
  and secondary-base arguments still accept the derived objects.
- Exception types and messages can differ, including unknown C++ exceptions.
- Boolean parameters accept only Python `True` and `False`. Convert NumPy
  booleans explicitly, for example `joint.setLimitEnforcement(bool(flag))`.
- String parameters accept `str`; decode `bytes` explicitly before passing it.
- pybind11 extension modules can no longer pass DART objects to dartpy or
  receive them from it, because nanobind does not share pybind11's type
  registry.

Python overrides and subclass state stay alive while C++ uses the object.
Wrappers for graph objects, including joints and degrees of freedom, keep
their skeleton alive. Owned Eigen vector and matrix returns are read-only
NumPy copies; call `.copy()` before writing and use a setter to update DART
state. Existing live views preserve their mutability, including read-only
views of const Eigen data. Inputs and caller-provided output buffers are
unaffected.

### GUI ownership

GUI Python ancestry follows the primary base: `Viewer`, plus `DragAndDrop`,
`SimpleFrameDnD`, `SimpleFrameShapeDnD`, `BodyNodeDnD`, and
`InteractiveFrameDnD`, omit `common.Subject` from Python ancestry.
`InteractiveTool` and `InteractiveFrame` omit `dynamics.Detachable`, inherited
through `SimpleFrame`. These secondary bases remain usable through native
arguments and pointer returns; only `isinstance`, `issubclass`, and MRO differ.

Each viewer's retention registry keeps the Python wrappers and overrides of its
world nodes, attachments, widgets, and event handlers alive while the viewer
holds them; removing one releases it. The viewer's GC slots detach these native
edges before clearing Python references. `enableDragAndDrop` returns wrappers
that share ownership of the native interaction.

### Viewer app callbacks

`dartpy.viewer` binds the private app host in `dart/gui/app`; a vanilla
system ImGui compiles out both. The host never owns a Python reference. Each
callable belongs to the Python object that accepted it: a `Scene`,
`SceneSetup`, `KeyAction` or `Control`, or the `SceneContext` for teardowns
and event handlers. That object shows it to the cyclic GC, while the host
holds weak handles. A `Control` also keeps the wrappers of its children, so a
callback set on a child outlives the child's Python name. During `run()`, a
session keeps the installed scene's setup and context alive until the
scene's last teardown.

`run()` accepts the main thread only and releases the GIL while the host
runs; each callback takes it back. An ordinary exception from a callback is
logged with its traceback, disables only that callback, and makes
`--cycle-scenes` exit nonzero. `KeyboardInterrupt`, `SystemExit` and other
`BaseException`s stop the run instead: the scene is torn down, with only
teardowns still called, and `run()` re-raises the exception. The host polls
for Ctrl+C once per frame and per step, and an interrupt also ends a
real-time catch-up between steps. Event handlers follow the same rules,
although the viewer calls them, not the host. `SceneContext.add_attachment`
takes native visuals only, such as `dartpy.gui.GridVisual`: Python never
draws, so per-frame Python logic belongs in `pre_refresh`. A
`BaseException` from an interim `dartpy.gui` override that a scene installs
on the viewer directly still stops the run.

The host clears a panel's callbacks at teardown, and a scene's activation
reconnects them from the Python callables, so a factory may return the same
`Panel` each time. `Control.edit()` applies a value as a user edit does, or
clicks a button, which lets scripted tests drive the host's own dispatch.
Panel values are validated in C++; decimal precision is expressed as a
number rather than a printf format string. See
[scene rules](../../examples/README.md#adding-catalog-scenes) and
[catalog verification](testing.md#demo-catalog-verification) when extending
the source-tree app.

### Binding infrastructure

The cast registry records C++ inheritance edges and computes new paths
incrementally as classes register. DART casters use these paths to adjust
secondary and virtual base pointers. Every binding translation unit starts
with `detail/dart_nb.hpp`, which selects the common pointer casters and checks
their selection with compile-time assertions. Add `detail/eigen.hpp` for Eigen
bindings and `detail/array.hpp` to preserve pybind11's `FixedSize` array
annotations using nanobind's array conversion. Use
`detail/optimizer_properties.hpp` for optimizer property values and
`detail/ik_properties.hpp` for inverse-kinematics property values. The property
headers also assert their caster selection. Find an aspect's entry in a
composite state with `detail/composite_state.hpp`, which matches type names:
`state.get<T>()` misses on macOS arm64, where dartpy's hidden copy of an aspect
template's typeinfo never equals libdart's. CMake runs
`scripts/nanobind/check_guards.py` to enforce the first include and required
headers and reject conflicting stock Eigen, `shared_ptr`, and `array.h`
casters. Missing optional STL casters fail compilation. Keep DART headers and
STL casters limited to the types that a translation unit uses.

Factory-backed Python subclasses create the native object when the base
initializer receives its arguments, preserving translated initializers and
Python identity. A small attachment helper in `detail/construction.cpp` uses
nanobind internals to retain the original factory control block without moving
the native object. Compile-time ABI/layout checks guard this dependency; audit
the helper when nanobind changes its internals ABI.

The Eigen caster preserves Eigen dimensions, strides, and ownership. By-value
results move into a heap owner. At runtime, NumPy 2 uses its C API to export
the array; NumPy 1 uses nanobind's `numpy.asarray` export with that same owner.
Both paths keep returned storage alive and read-only. Explicit copy-policy
exports and empty owned arrays also clear NumPy's writeability flag after
exporting. The NumPy export cache assumes the GIL and a single interpreter;
isolated interpreters and free-threaded Python need a separate ownership and
synchronization design.

`optimizer.Function.evalGradient` overrides receive an owned NumPy output
array initialized from the native gradient. In-place updates are copied back
after the callback; changing the output size raises `ValueError`. Retaining the
array keeps its storage alive independently of later solver calls.

`SoftBodyNode.get_point_mass(index)` checks bounds and returns a borrowed
`PointMass` that retains its body and Skeleton. Point positions, velocities,
accelerations, world position and forces return independent read-only NumPy copies; positions
and velocities can be assigned through their properties. These setters preserve
the engine's nonfinite-value behavior, so diagnostic callers must check state
before using it. Adaptive-contact settings are ordinary SoftBodyNode properties;
the engine determines activation membership.
If a soft-body copy replaces topology and deletes a point, its retained Python
handle raises `TypeError` on native access, including point-impulse arguments.
Destruction notifications prevent address reuse from reviving that handle.

Point masses expose mass, rest position, world velocity and collision status;
the four `reset_*` methods reset their position, velocity, acceleration and force
state separately. `Skeleton.clear_colliding_bodies()` clears body and point
collision flags. Soft-body generator properties expose independent arrays of
point rest positions, masses and faces, and `SoftBodyNode.get_face()` returns a
checked, copied face.

`Skeleton.getConfiguration()` returns an owning `SkeletonConfiguration` value
for `setConfiguration()`. `BodyNode.get_composite_state()` returns an owning
`common.CompositeState` for `set_composite_state()`. These snapshots retain
their copied state independently of the original graph. Restoring body or
skeleton snapshots with different soft-point counts raises `ValueError` before
mutation. `Skeleton.setState()` retains C++ joint restore semantics, including
deriving FORCE-joint commands from restored forces.

`MeshShape.mesh_vertex_groups` returns independent, read-only arrays of unscaled
vertices in submesh order. `VoxelGridShape.validate_occupancy_update()` checks
the same inputs as `updateOccupancy()` without changing occupancy, allowing
callers to validate a cloud before changing other sensor state.

`math.TriMesh(vertices, triangles)` owns double-precision geometry and checks
finite vertices and valid integral triangle indices. Its array properties return
independent copies. `MeshShape(scale, mesh)` accepts this ordinary mesh, and
`MeshShape.tri_mesh` returns its owning native mesh.

Registered owners expose native Python references to cyclic GC through
traverse and clear slots. Coverage includes objective/solver and constraint
state, collision options, composite retrievers, IK/error methods and their
properties, skeleton/body/shape state, contact inverse dynamics, and parser
options/loaders. Collection stays conservative when native aliases exist
outside the visible ownership graph or several owners share one Python pin.
Direct traversal requires exclusive native ownership. Borrowed graph and field
proxies, including body and shape nodes, cannot prove that exclusivity and
retain their cycles until Python back-references are cleared explicitly. This
prevents GC from clearing simulation state still used by a live native owner.
Directly constructed `InverseKinematics` objects also keep an internal native
reference and require that cleanup; `getOrCreateIK()` owners can prove
exclusivity when no other native references remain. Skeleton-owned whole-body
IK wrappers also track their retained Skeleton: cycles through the IK wrapper
collect when the wrapper and that Skeleton are the IK's only native owners,
and the wrapper is the Skeleton's only native owner. Live Skeleton, body,
World, or other native aliases keep these cycles conservative. A node IK cycle
whose Python child also retains its Skeleton has the body's additional IK
reference and therefore requires explicit back-reference cleanup too.
A weak-owner registry alone cannot solve this: native `shared_ptr` aliases
outside binding setters are invisible, and each shared pin owns only one
Python reference. Reporting it once per owner would miscount GC references;
clearing a chosen owner could release a still-used Python override. Supporting
this case requires tracking the native ownership graph.

Private local-retriever state in `PackageResourceRetriever`, indirect
`Linkage`/`Chain` graph ownership, and callbacks stored in `RaycastOption.mFilter`
are outside this enumeration. Clear Python back-references explicitly, for
example `child.owner = None`, and clear stored callbacks when releasing such
cycles. Private native owners need public traversal/reset APIs, and callbacks
need GC-visible ownership before those cases can participate safely.

Reusable porting and API probes live under `scripts/nanobind/`. The regular
`python/tests/` suite tests the nanobind behavior, including ownership and the accepted
changes from DART 6.20. Regular Python CI jobs cover GUI construction and
ownership; `CI Filament` runs the rendering tests. Windows uses its existing
unity build.

## Wheels

`pyproject.toml` builds wheels with scikit-build-core: it builds only the
`dartpy` CMake target and installs only the `python` component, so a wheel
holds the `dartpy` package (`__init__.py`, `py.typed`, `experimental/`, the
`_core` extension) plus the licenses, and no test package. The version comes
from `package.xml`. `scripts/pep517/dartpy_backend.py` is a thin PEP 517
wrapper around scikit-build-core, which resolves license files before CMake
runs: CMake stages the fetched Filament SDK license and notices inside the
package, and the wrapper moves them to `dist-info/licenses/filament/` with
matching `License-File` headers after the wheel is built. The wrapper exports
only `build_wheel` and `get_requires_for_build_wheel`: it has no
`prepare_metadata_for_build_wheel`, so frontends read METADATA from the built
wheel, where the headers match. dartpy is wheel-only (D20): there is no
`setup.py`, and `build_sdist` and `build_editable` raise
`UnsupportedOperation`, so `pip install -e .` fails by design. Develop with
`pixi run build-py-dev` and `DART_PYTHONPATH` instead.

macOS and Windows wheel packaging is Pixi-managed. `wheel-build-core`,
`wheel-repair-<platform>-core`, `wheel-verify-core`, and `wheel-test-core`
implement the pipeline; the `py312-wheel` environment exposes `wheel-build`,
`wheel-repair`, `wheel-verify` and `wheel-test`, and `py313-wheel` exposes
`wheel-test-abi3`. `wheel-build-core` passes only the environment-specific
CMake options (`CMAKE_PREFIX_PATH`, `DART_USE_SYSTEM_IMGUI` and host linker
flags) through `CMAKE_ARGS`; the fixed wheel configuration lives in
`pyproject.toml`.

Wheels require Python 3.12 and use the nanobind binder for CPython's stable
ABI (`DART_DARTPY_STABLE_ABI=ON`, which needs CMake 3.26), so every build is
tagged `cp312-abi3` and one wheel per platform serves Python 3.12 and every
later version. The release matrix builds four wheels: Linux x86_64 and
aarch64, macOS arm64, and Windows x64. Each row checks the wheel with
`abi3audit --strict` and tests it on Python 3.13: the Linux driver below does
both in its containers, and macOS and Windows run
`pixi run -e py313-wheel wheel-test-abi3`. Stable-ABI calls cost up to about 5%
more instructions on NumPy-heavy setters. `scripts/verify_wheel.py` rejects a
wheel that is not `cp312-abi3`, lacks the package files or the Filament
license, or ships files outside the package.

Linux release wheels use `python3 scripts/wheel_linux.py cp312` on native
x86_64 or aarch64 hosts with Docker.
The driver builds `docker/wheels/Dockerfile.manylinux_2_28`, which fetches each
source-built native dependency as a release or tag archive checked with
`sha256sum` (update the hash with the version). It compiles those dependencies and dartpy
with the manylinux toolchain, requires `auditwheel repair --plat manylinux_2_28_<arch>`, and runs the wheel smoke
test in a fresh manylinux 2.28 base container. Outputs are in `dist/`.
The source checkout is mounted read-only; temporary container builds avoid
reusing host CMake caches.
Wheel configurations request Python's `Development.Module` component rather
than its embedding library, which manylinux deliberately omits.

A Conda compiler sysroot pin alone does not establish manylinux compatibility:
prebuilt dependencies can require newer `GLIBCXX` or `CXXABI` symbols even
when their glibc floor is 2.28. The manylinux toolchain supplies newer compiler
features while retaining the platform's C++ runtime ABI. The local Linux
Pixi wheel tasks remain useful for diagnostics; their repair now requires
2.28 and rejects incompatible dependencies.

`.github/workflows/publish_dartpy.yml` runs these platform pipelines. For a
non-publishing validation of every row after an approved push, dispatch:

```bash
gh workflow run publish_dartpy.yml --ref <branch> \
  -f ref=main -f checkout_ref=<candidate-sha> -f publish=false
```

Use the candidate branch for the workflow definition and the exact candidate
SHA for the checkout. Inspect both architectures' repaired tags and fresh
container smoke tests before release.

The repaired-wheel smoke test requires all four parser APIs and constructs a
`dartpy.gui` interactive frame and a viewer with a world node without opening a
display. It also parses an inline URDF model, so missing IO or GUI support fails
acceptance. Linux release wheels run this check in the fresh manylinux
container without a display.
