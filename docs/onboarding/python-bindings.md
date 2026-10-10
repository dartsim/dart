# Python Bindings

DART Python bindings are built through the Pixi/CMake workflow.

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

DART 6.21 adds an opt-in binder under `python/dartpy_nanobind/`.
`DART_DARTPY_BINDER` selects `pybind11` (the default) or `nanobind`; both build
the `dartpy` module with the same namespaces, names, and overloads.
The existing pybind11 sources stay under `python/dartpy/` during the
transition. The nanobind binder includes `dartpy.gui.osg` when `DART_BUILD_GUI_OSG` is enabled.

Configure the Pixi build, select the binder in its CMake cache, and run the
regular tests:

```bash
pixi run config
# Linux/macOS:
pixi run cmake -S . -B build/default/cpp/Release -DDART_DARTPY_BINDER=nanobind
# Windows: use build/default/cpp instead of build/default/cpp/Release.
pixi run test-py
```

Use `-DDART_DARTPY_BINDER=pybind11` to switch back. `DART_USE_SYSTEM_NANOBIND`
defaults to `OFF`, which fetches nanobind v3.1.0 with its `robin_map`
submodule. With `ON`, CMake locates the package's CMake directory using
`python -m nanobind --cmake_dir` and requires nanobind 3.1 or newer. The
binder requires Python 3.10 or newer. The installed location, module name, and
configured build output `${DART_PYTHON_BUILD_DIR}/dartpy` match the pybind11
build, preserving the Pixi example runners. The generated target runtime path
remains authoritative for tests and CMake example targets.

### API differences

- Python recognizes only the primary C++ base for `isinstance`, `issubclass`,
  and the method resolution order. Secondary-base methods remain callable,
  and secondary-base arguments still accept the derived objects.
- Exception types and messages can differ, including unknown C++ exceptions.
- Boolean parameters accept only Python `True` and `False`. Convert NumPy
  booleans explicitly, for example `joint.setLimitEnforcement(bool(flag))`.
- String parameters accept `str`; decode `bytes` explicitly before passing it.

Python overrides and subclass state stay alive while C++ uses the object.
Wrappers for graph objects, including joints and degrees of freedom, keep
their skeleton alive. Views of const Eigen data are read-only; code that
needs a writable independent array should make an explicit copy.

### GUI ownership

GUI Python ancestry follows the primary base: `Viewer` and `ImGuiViewer`, plus
`DragAndDrop`, `SimpleFrameDnD`, `SimpleFrameShapeDnD`, `BodyNodeDnD`, and
`InteractiveFrameDnD`, omit `common.Subject` from Python ancestry.
`InteractiveTool` and `InteractiveFrame` omit `dynamics.Detachable`, inherited
through `SimpleFrame`. These secondary bases remain usable through native
arguments and pointer returns; only `isinstance`, `issubclass`, and MRO differ.


OSG trampoline aliases occupy Python instance storage. Their constructors take
one OSG reference and their destructors release it without deleting that
co-located memory. Native viewers use heap factories through the hybrid
construction helper, preserving custom Python subclass initializers.

Viewer retention registries keep Python overrides alive for native world-node,
event-handler, and attachment edges. Their GC slots detach native edges before
clearing Python references. Factory deleters retain the registry until native
destruction finishes. Removed wrappers remain retained while camera callbacks
or native callback snapshots still reference them; they can outlive removal
until viewer destruction. `ref_ptr` returns attach ownership only when creating
a wrapper, preserving identity and avoiding repeated-getter reference growth.

DragAndDrop wrappers observe native destruction notifications and become invalid
when native code deletes the object. Calls through invalid wrappers raise a
Python error instead of accessing freed memory. The pybind11 binder does not
invalidate these wrappers; using one after native deletion is unsupported.

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
headers also assert their caster selection. CMake runs
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
Both paths keep returned storage alive. The NumPy export cache assumes the
GIL and a single interpreter; isolated interpreters and free-threaded Python
need a separate ownership and synchronization design.

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
exclusivity when no other native references remain. An IK cycle whose Python
child also retains its Skeleton has the body's additional IK reference and
therefore requires explicit back-reference cleanup too.
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
`python/tests/` suite runs against the selected binder and marks accepted
binder differences explicitly. Linux and macOS CI add a Python-only nanobind
job; Windows adds a Python matrix row using its existing unity build.

### pybind11 extensions

A pybind11 extension cannot see nanobind's type registry, so it cannot pass or
return DART objects by registering nothing, as it could with the pybind11
binder. The nanobind binder instead exports the `dartpy._C_API` capsule, and
the installed header `dartpy/pybind11_interop.hpp` turns it into pybind11
casters:

```cpp
#include <dartpy/pybind11_interop.hpp> // before any binding code

PYBIND11_MODULE(my_robot, m)
{
  m.def("base", [](const dart::dynamics::SkeletonPtr& skeleton) {
    return skeleton->getBodyNode(0);
  }, pybind11::return_value_policy::reference);
}
```

The casters return the existing dartpy wrapper when there is one, adjust base
pointers, keep a returned graph object's skeleton alive, and share ownership
for `std::shared_ptr` arguments and results. They cover `Entity`, `Frame`,
`SimpleFrame`, `JacobianNode`, `BodyNode`, `ShapeNode`, `Joint`,
`DegreeOfFreedom`, `MetaSkeleton`, `Skeleton`, `World`, and copies of
`Eigen::Isometry3d` (`dartpy.math.Isometry3`, which also accepts 4x4 arrays).
Add another polymorphic class that dartpy binds with
`DARTPY_PYBIND11_INTEROP_OBJECT(Type, "dartpy.module.Name")` at global scope.

- Include the header in every translation unit that binds these types, and
  never register them with `pybind11::class_`.
- Raw-pointer and reference results need an explicit `reference` or
  `reference_internal` policy. Other policies, including the default
  `automatic`, would take, copy, or move the object and raise an error; return
  a `std::shared_ptr` when ownership moves to Python.
- Keep stored raw pointers' owners alive yourself, for example with
  `pybind11::keep_alive`.
- The table passes `std::type_info` and `std::shared_ptr` across modules: build
  the extension with the same DART headers and C++ standard library ABI as
  dartpy, and link the same shared DART libraries that dartpy loads.
  Importing an extension with a mismatched DART version or standard library
  raises `ImportError`.

`python/tests/interop/` builds an example extension that the regular suite
exercises when pybind11 is available.

## Wheels

Wheel packaging is Pixi-managed on this branch. `wheel-build-core`,
`wheel-repair-<platform>-core`, `wheel-verify-core`, and `wheel-test-core`
implement the pipeline, and the `py310-wheel`..`py313-wheel` environments
expose `wheel-build` / `wheel-repair` / `wheel-verify` / `wheel-test` per
Python version. `.github/workflows/publish_dartpy.yml` runs the same tasks
for released wheels; reproduce a CI wheel step locally with
`pixi run -e <pyXY-wheel> wheel-build` and friends.

Wheels use the nanobind binder. On Python 3.12 and newer, `wheel-build` sets
`DART_DARTPY_STABLE_ABI=ON`, which builds the module for CPython's stable ABI
and tags the wheel `cp312-abi3`, so one wheel per platform serves Python 3.12
and every later version. The release matrix therefore builds five wheels:
`cp312-abi3` for Linux, macOS, and Windows, plus Linux `cp310` and `cp311`. Each
abi3 row also checks the wheel with `abi3audit --strict` and tests it on
Python 3.13 through `pixi run -e py313-wheel wheel-test-abi3`. Stable-ABI calls
cost up to about 5% more instructions on NumPy-heavy setters; CMake refuses
`DART_DARTPY_STABLE_ABI` on Python older than 3.12.
