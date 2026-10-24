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

Python 3.12 or newer, NumPy 1.26.4 or newer (including NumPy 2.x), and
nanobind 3.1.x are required. CMake discovers Python once in
`python/CMakeLists.txt` and caches the selected interpreter as
`Python3_EXECUTABLE` for tests, examples and tutorials. Select the interpreter
with `Python_EXECUTABLE`; `Python3_EXECUTABLE` only seeds a fresh configure.
If both are set and differ, CMake warns unless `Python3_EXECUTABLE` still holds
the value DART last cached, and uses `Python_EXECUTABLE`.
Use `Python_ROOT_DIR` and other `Python_*` hints; the `Python3_*` forms no longer
apply. C++ builds with
`DART_BUILD_DARTPY=OFF` do not discover Python.
`DART_USE_SYSTEM_NANOBIND` defaults to `OFF`, fetching the nanobind 3.1.0
PyPI sdist by URL and SHA-256 with TLS verification. The sdist bundles
`robin_map`. With `ON`, CMake uses `nanobind_DIR` when it is set and otherwise
locates nanobind's CMake directory using
`python -m nanobind --cmake_dir`. Set `nanobind_DIR` when the build's Python
cannot import nanobind, as in cross builds and isolated pip builds. Source
builds through `pip` use scikit-build-core (`pyproject.toml`) and keep the
CMake build tree under `build/<wheel tag>`; the `SKBUILD_BUILD_DIR`
environment variable moves it, for example to a larger disk. Tests and
examples import the staged package from `DART_PYTHONPATH`. The
`DART_DARTPY_BINDER` and `DART_USE_SYSTEM_PYBIND11` options are removed.
Passing the legacy binder value `pybind11` fails configure; `nanobind` is
accepted for compatibility and removed from the cache.

`python/dartpy/detail/construction.cpp` checks nanobind's internal factory
attachment ABI (`NB_INTERNALS_VERSION=22`), pinning system nanobind to 3.1.x.
CMake checks both the selected headers' version and internals version at
configure time for system and fetched builds. When bumping nanobind, audit
that check and re-diff `python/dartpy/detail/dart_eigen_dense.h` against
nanobind's stock `include/nanobind/eigen/dense.h` to preserve DART's NumPy
compatibility changes. The Eigen 5 toolchain CI lane also builds and tests
with `DART_USE_SYSTEM_NANOBIND=ON`.

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

### Errors and logging

`dartpy.errors` holds the exception hierarchy. The errors dartpy defines
derive from `DartError`, which has three branches named after their builtin
parent: `ValidationError` (`ValueError`) for invalid arguments and inputs,
`NotFoundError` (`KeyError`) for failed name lookups, whose message lists the
valid names, and `OperationError` (`RuntimeError`) for operations that fail
or run in the wrong state. `WorldBusyError` and `ModelFrozenError` derive
from `OperationError`; `CheckpointMismatchError` and `NameConflictError`
derive from `ValidationError`. The design doc's top-level allowlist excludes
errors, so they live in their own submodule and their `__module__` is
`dartpy.errors`.

Not every exception dartpy raises is a `DartError`. A translator registered
before every submodule maps the C++ exceptions DART throws:
`std::invalid_argument` and `std::length_error` to `ValidationError`, and
`std::runtime_error`, `std::logic_error` and every other `std::exception`
subclass not listed next to `OperationError`. `std::out_of_range`
(`IndexError`), `std::domain_error` and `std::range_error` (`ValueError`),
`std::overflow_error` (`OverflowError`) and `std::bad_alloc` (`MemoryError`)
keep nanobind's builtin mapping on purpose, and nanobind's own argument
conversion failures raise plain `TypeError`. Register more specific nanobind
exceptions in their submodule with `dartnb::operationError()` as the base, as
`dartpy.gui` and `dartpy.viewer` do; nanobind runs later registrations first.
Binding-side `nb::value_error` and `nb::type_error` checks are still plain
builtins; bad input raises, it is never logged and ignored.

DART's C++ diagnostics reach `logging.getLogger("dartpy")`. The bridge in
`detail/logging.cpp` registers a `dart::common::setLogCallback()` callback
for the `DART_WARN` family and redirects the `dtwarn`/`dterr`/`dtmsg`/`dtdbg`
console streams through `dart::common::setConsoleStreams()`. The callback
hook lives in libdart, so it receives diagnostics emitted inside libdart and
from the extension. The logging implementation formats each message once
and offers it to the callback before acquiring or initializing sink state.
A message the callback declines goes to the selected sink or console streams,
subject to the runtime threshold. The callback receives every message that
`DART_ACTIVE_LOG_LEVEL` compiles in, regardless of `setLogLevel()`, including
`Off`; `logging.getLogger("dartpy")` filters by its own level. The
bridge does not apply that level before queueing a record. Python's
`logging` has no hook that fires when a level changes, so a cached
threshold could go stale and silently drop messages a user just enabled,
and DART formats the message before the callback runs, so the check would
save only a string copy and a queue push. The console
hook takes stream providers, and the bridge hands every thread its own
`std::ostream` over a per-thread line buffer, so concurrent writers share
neither manipulator state nor a partial line; a thread-local or static
destructor that logs after that stream is destroyed gets a leaked fallback
stream instead. Logging threads never touch Python: each completed line or
`DART_WARN` message goes into a bounded queue (1024 records, oldest dropped
and the count reported), under one mutex that also guards the pass-through
flag and the pending-call bookkeeping. `Py_AddPendingCall` drains the queue on the main
thread at its next safe point, which is right after the binding call that
logged returns. Records from C++ worker threads wait until the main thread
runs Python again or `dartpy.common.flush_logs()` is called;
`flush_logs()` delivers on the thread that calls it and returns the number
of records it drained, counting the drop notice. Delivery catches Python
and C++ exceptions per record and reports them as unraisable or on stderr,
so one failing handler loses only its record. Levels map one to one;
trace maps to level 5. On POSIX, `pthread_atfork` handlers hold the queue
mutex, the only lock on the bridged path, across `fork()` and give the
child an unlocked, empty queue, so a child forked from a single-threaded
process can log and flush. Forking a process that has other running threads
is unsupported: a thread the child does not have may hold a lock below the
bridge, such as libstdc++'s locale mutex that any stream construction takes
under a non-classic global locale, or a selected log sink mutex, and the child then
blocks on its first log. CPython warns about forking a multi-threaded process
for the same reason; use the `spawn` or `forkserver` start method there.
Python's `atexit` drains the queue and switches the bridge to pass-through,
so diagnostics logged while the interpreter shuts down print to stderr
without touching Python. There is no default handler: with no logging
configuration, Python's last-resort handler prints warnings and errors to
stderr, and info messages stay quiet; set the logger's level or handlers to
silence or route DART output.
The GUI App separately observes callback-consumed records at the native sink
threshold and queues its panel notifications until the GUI owner drains them.
It neither installs another Python bridge nor invokes Python from producers.
Closing a capture discards pending panel notifications without changing the
automatic Python bridge. The superseded manual enable/disable API is removed;
configure the Python logger and use `flush_logs()` for explicit delivery.

`dartpy._testing` holds hooks for dartpy's own tests (worker-thread logging,
destructor-time logging, throwing a named C++ exception). It is not part of
the API: `python/dartpy/CMakeLists.txt` compiles it in only when
`BUILD_TESTING` is on and `DART_BUILD_WHEELS` is off, and the tests that
use it skip when it is absent.

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

### Transform and Rotation values

`dartpy.math.Transform` and `dartpy.math.Rotation` (re-exported at the top
level) are immutable values backed by contiguous `Eigen::Isometry3d` storage:
one item for a scalar, `N >= 1` items for a batch built from arrays with a
leading axis (an empty leading axis raises `ValueError`). Constructors are
keyword-only and take at most one rotation form (`rotation=` a 3x3 or a
`Rotation`, `wxyz=`, `xyzw=`, `rpy=`) plus `translation=` for a `Transform`.
Alternatively, `Transform(matrix=...)` takes a homogeneous matrix that already
contains translation and excludes `translation=` and all other rotation forms.
No rotation form gives the identity, and
`identity()` spells that out. `Rotation.from_axis_angle(axis, angle)` builds a
scalar rotation from a nonzero axis of any magnitude (it is scaled by its
largest component before normalization) and an angle in radians. `rpy` is
extrinsic X-Y-Z, `Rz(yaw) Ry(pitch) Rx(roll)`, the URDF/SDF convention;
`Rotation.as_rpy()` inverts it with roll set to 0 at gimbal lock. It does not
use `dart::math::eulerXYZToMatrix`, whose convention differs. Read-only array
properties are `matrix` (3x3 for a `Rotation`, 4x4 for a `Transform`),
`translation`, `wxyz` and `xyzw`, each with a leading `N` for a batch, and
`Transform.rotation` returns the rotation part as a `Rotation`. Only `@` is
defined: it composes values, with a scalar broadcasting against a batch
and batches requiring equal length. Scalar values apply to `(..., 3)` point
arrays of any rank (rows are points). Batched values accept a single `(3,)`
point broadcast to every value or a pairwise `(N, 3)` point array.
`__array_ufunc__ = None`, so `points @ value` raises `TypeError`
instead of letting NumPy broadcast.

Validation raises `ValueError` and checks finite values,
`|R^T R - I|_F`, `|det R - 1|`, the homogeneous row of a 4x4 input and
`|norm(q) - 1|`, all against a default tolerance of `1e-5`. The calibration
measured composition of up to 1e4 rotations (below 1e-13), quaternion round
trips (below 1e-14), text with six to nine significant digits (2.3e-6 down to
2.4e-9), float32 round trips (1.2e-7) and per-entry noise (1e-6 gives 1e-5,
1e-4 gives 1e-3); the tolerance sits one order of magnitude above the worst
legitimate source and one below inputs that must fail. `normalize=True`
projects a matrix onto the nearest rotation through SVD (`U V^T`) and rescales
a quaternion coefficient by coefficient (each is divided by the largest
magnitude, then by the norm of that scaled vector, so coefficients near 1e308
or subnormal still normalize). It accepts a matrix only when its determinant
is positive, every singular value lies within `1e-2` of 1, and the relative
spread of the singular values `(max - min) / max` is at most `1e-3`. Stretch
alone bounds the operator-norm distance to the nearest rotation but admits a
shear of 0.005 (singular values `1 +- 0.0025`); the spread catches shear and
non-uniform scale while a uniform `1.001` scale or float32 rounding (spread
about 1e-7) still passes. Reflections, and shear or non-uniform scale beyond
that spread, raise, and
the error message suggests `normalize=True` only for inputs that meet both
criteria. Composition and
`inverse()` never normalize: a chain of 1e4 compositions drifts by about 1e-13,
and `Transform(matrix=value.matrix, normalize=True)` restores exactness when
needed. Composition and `inverse()` raise `ValueError` if any result entry is
nonfinite or its rotation leaves the near-rotation envelope above; batch errors
identify the first failing index.

Quaternion order is always named (`wxyz=`/`xyzw=` in, `.wxyz`/`.xyzw` out).
Matrix-to-quaternion conversion uses Eigen's largest-diagonal branch, and the
sign policy makes `w >= 0` and, for `w == 0`, the first nonzero of `(x, y, z)`
positive. The rule reads the exact bits: a `w` that rounds to `-1e-17` negates
the quaternion, and only an exact zero defers to the next coefficient. Array
properties return read-only copies. `==` and `hash()` are bit-exact over the
stored 4x4 matrix, so `-0.0` and `0.0` are different values; `hash()` works for
scalars only (batches raise `TypeError`, like NumPy arrays). Pickling keeps
the exact bits, and unpickling validates byte length, finite entries, the
homogeneous row, a rotation inside the `normalize=True` near-rotation envelope
(positive determinant, singular-value stretch at most `1e-2` and relative
spread at most `1e-3`), and zero translation for a `Rotation`. It preserves
drift beyond the construction tolerance without projecting; crafted state
outside these invariants raises `ValueError`. Restoring state on an already
initialized value raises `TypeError`; pickle, copy and deepcopy create new values.
`Transform.from_isometry()`
validates a `dartpy.math.Isometry3` and `to_isometry()` returns a mutable
copy for the camelCase API; there is no implicit conversion in either
direction.

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

Only Linux release wheels enable `DART_BUILD_UTILS_ASSETS=ON` and expose
`dartpy.utils.ModelResourceRetriever`. Linux builds shared libcurl 8.23.0 and
OpenSSL 3.5.9 from SHA-256-checked release archives. OpenSSL 3.5 is the
[LTS line supported through April 2030](https://openssl-library.org/source/).
Shared libraries follow the image's existing dependencies and let auditwheel
bundle and relocate libcurl, libssl and libcrypto. curl enables HTTP/HTTPS
(HTTPS requires the HTTP implementation), with other transfer protocols and
optional compression, SSH, IDN, PSL and HTTP/2 dependencies disabled; the
retriever restricts transfers and redirects to HTTPS. Linux strips the three
TLS libraries before repair and disables unused OpenSSL protocols, algorithms
and subsystems, while retaining OCSP and the modern TLS client algorithms.
Linux OpenSSL's config and modules directories are dedicated to dartpy to
avoid another installation's configuration.
The wheel's `dist-info/licenses/` also includes upstream notices for curl
and OpenSSL from `python/dartpy/licenses/`; the eight deferred-platform notices
remain in the repository without shipping in wheels.
macOS and Windows wheels keep this component disabled. Their locked
conda-forge curl closure would add **18,163,786 compressed bytes** on macOS
arm64 (14 libraries) and **16,546,729 bytes** on Windows x64 (6 libraries),
using raw deflate level 6. ICU data alone accounts for 13,057,387 and
12,798,716 bytes respectively. Against main's 14,083,197-byte macOS and
12,597,754-byte Windows wheels, the projected totals are about 32.25 MB
(+129%) and 29.14 MB (+131%). Linux's measured increase is 2,649,828 bytes,
from 18,653,674 to 21,303,502 bytes. The macOS/Windows cost is about eight
times the roughly 2 MB estimate used for the original decision. The open
maintainer choice is to accept that closure or build a minimal curl from the
pinned source archive before enabling those platforms. The repair tooling and
notices remain ready, but their current wheels require neither the retriever
nor its TLS smoke. No platform uses a Python runtime CA dependency.

`scripts/verify_wheel.py` prints every bundled library's compressed/unpacked
size and rejects wheels above these compressed-byte budgets:

| Platform | Recorded wheel size (bytes) | Maximum (bytes) |
| --- | ---: | ---: |
| Linux x86_64 | 21,303,502 | 23,000,000 |
| Linux aarch64 | Not yet measured locally | 23,000,000 |
| macOS arm64 (retriever off) | 14,083,197 | 16,000,000 |
| Windows x64 (retriever off) | 12,597,754 | 14,500,000 |

The macOS/Windows baselines are main's
[nightly run 40095159727](https://github.com/dartsim/dart/actions/runs/40095159727),
not newly built artifacts from this change. That run's Linux aarch64 wheel
without the retriever was 17,151,246 bytes; its final retriever cost is still
unmeasured. It shares the 23 MB Linux limit until a native aarch64 build
provides a measured budget, leaving roughly 3 MB of estimated headroom.
Budget changes require reviewing the inventory,
license coverage and dependency cost. See [TLS trust lookup](io-parsing.md#tls-trust)
for the Apple guard against build-environment CA/configuration paths,
including its effect on macOS scikit-build source installs.
The wheel's Python install component does not include DART's sample model
manifests. Register a local manifest with `addManifest(file_uri)`, or point
`DART_DATA_PATH` to the `data/` directory of a DART checkout to use those samples.
Without it, missing-sample warnings from the constructor are expected. A Linux
wheel can retrieve models from a user-registered local manifest and its
`model://` URI without `DART_DATA_PATH`.

Wheels require Python 3.12 and use the nanobind binder for CPython's stable
ABI (`DART_DARTPY_STABLE_ABI=ON`); source builds require CMake 3.28 or newer.
Linux scikit-build source installs (`pip install .`) also enable the retriever
and require libcurl >= 8.5 and OpenSSL Crypto >= 3.0.13 development packages;
macOS/Windows source installs need them when explicitly enabling utils-assets.
Stock packages meet these floors on Ubuntu 24.04 or newer; Debian 12 (libcurl
7.88.1) and Ubuntu 22.04 (libcurl 7.81, OpenSSL 3.0.2) are too old. To build
without the
retriever or those development packages, use
`CMAKE_ARGS=-DDART_BUILD_UTILS_ASSETS=OFF pip install .`.
Every build is tagged `cp312-abi3`, and one wheel per platform serves Python 3.12 and every
later version. The release matrix builds four wheels: Linux x86_64 and
aarch64, macOS arm64, and Windows x64. Each row checks the wheel with
`abi3audit --strict` and tests it on Python 3.13: the Linux driver below does
both in its containers, and macOS and Windows run
`pixi run -e py313-wheel wheel-test-abi3`. Stable-ABI calls cost up to about 5%
more instructions on NumPy-heavy setters. `scripts/verify_wheel.py` rejects a
wheel that is not `cp312-abi3`, exceeds its size budget, lacks package files,
`LICENSE`, `NOTICE`, the Filament license or required curl/OpenSSL notices, or
ships files outside the package.

Linux release wheels use `python3 scripts/wheel_linux.py cp312` on native
x86_64 or aarch64 hosts with Docker.
The driver builds `docker/wheels/Dockerfile.manylinux_2_28`, which fetches each
source-built native dependency as a release or tag archive checked with
`sha256sum` (update the hash with the version). It compiles those dependencies and dartpy
with the manylinux toolchain, requires `auditwheel repair --plat manylinux_2_28_<arch>`, and runs the wheel smoke
test in a fresh manylinux 2.28 base container. The TLS smoke temporarily hides
curl's pinned default bundle and places the test CA in the Debian/Ubuntu search
location alongside the real system roots, with `CURL_CA_BUNDLE` and
`SSL_CERT_FILE` unset. It also rejects the test server with only the real system
roots at that fallback location. It then hides every
searched bundle and requires a trust-anchor failure. A separate guard process
restores displaced files even if the smoke is killed by its timeout. Outputs
are in `dist/`.
The source checkout is mounted read-only; temporary container builds avoid
reusing host CMake caches.
For a curl or OpenSSL security update, bump the version and SHA-256 in
`Dockerfile.manylinux_2_28`, refresh notices if their license texts changed,
refresh the Pixi lockfile for source builds and future macOS/Windows wheels,
run the wheel gates, and release patch wheels; installed wheels cannot update
their bundled TLS libraries independently.
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

Also run the cross-distribution TLS release gate on a native Docker host:

```bash
python3 scripts/test_wheel_tls_distros.py dist/dartpy-*-cp312-abi3-manylinux_2_28_$(uname -m).whl
```

Pass one wheel matching the host architecture. This uses disposable
`ubuntu:24.04` and `debian:12` images with Python 3.12, tests untrusted and
wrong-host certificates, verifies an explicit CA, and exercises system CA
fallback with real roots plus the test CA, rejection with real roots alone,
and fail-closed behavior with every CA file absent, without environment overrides. The fallback test changes system trust files temporarily; it requires
a root Linux container with `/.dockerenv` or `/run/.containerenv`.

The repaired-wheel smoke test requires all four parser APIs and, on Linux,
the model retriever, and constructs a
`dartpy.gui` interactive frame and a viewer with a world node without opening a
display. It also parses an inline URDF model, so missing IO or GUI support fails
acceptance. On Linux it also downloads a manifest-pinned model from a local HTTPS
server with `CURL_CA_BUNDLE` set to the test CA, rejects that server without
the CA, and rejects its CA-trusted certificate when requested as `127.0.0.1`
instead of `localhost`. Both rejections must report TLS verification errors
before any HTTP request. This uses only Python's standard library and the
committed TLS test fixture; no external server or runtime CA package is
required. Linux release wheels run these checks in fresh manylinux containers
on Python 3.12 and 3.13
without a display. macOS and Windows run the package/parser/GUI smoke and
strict ABI audit outside the Pixi library paths. Before enabling their
retrievers, CI must additionally show rejected untrusted TLS, successful
trusted downloads, system trust without a build-environment default, and the
repaired TLS library inventory within an approved size budget.

On Python 3.12, the smoke test pins NumPy 1.26.4 and round-trips vector and
matrix arrays through dartpy. The Python 3.13 abi3 smoke test uses current NumPy.
