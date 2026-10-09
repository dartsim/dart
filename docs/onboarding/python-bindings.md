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

DART 6.21 adds an opt-in non-GUI binder under `python/dartpy_nanobind/`.
`DART_DARTPY_BINDER` selects `pybind11` (the default) or `nanobind`; both build
the `dartpy` module with the same non-GUI namespaces, names, and overloads.
The existing pybind11 sources stay under `python/dartpy/` during the
transition. The nanobind binder currently exposes no OSG GUI classes.

Configure the Pixi build, select the binder in its CMake cache, and run the
regular tests:

```bash
pixi run config
pixi run cmake -S . -B build/default/cpp/Release -DDART_DARTPY_BINDER=nanobind
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

Python implementations retained by one C++ owner participate in cyclic GC
through the owner's traverse and clear slots. A Python object pinned by
several C++ owners in a cycle is conservatively retained. Explicitly clear
Python back-references, for example `child.owner = None`, when releasing such
cycles. A weak-owner registry alone cannot solve this: native `shared_ptr`
aliases outside binding setters are invisible, and each shared pin owns only
one Python reference. Reporting it once per owner would miscount GC references;
clearing a chosen owner could release a still-used Python override. Supporting
this case requires tracking the native ownership graph.

Reusable porting and API probes live under `scripts/nanobind/`. The regular
`python/tests/` suite runs against the selected binder and marks accepted
binder differences explicitly. Linux and macOS CI add a Python-only nanobind
job; Windows adds a Python matrix row using its existing unity build.

## Wheels

Wheel packaging is Pixi-managed on this branch. `wheel-build-core`,
`wheel-repair-<platform>-core`, `wheel-verify-core`, and `wheel-test-core`
implement the pipeline, and the `py310-wheel`..`py313-wheel` environments
expose `wheel-build` / `wheel-repair` / `wheel-verify` / `wheel-test` per
Python version. `.github/workflows/publish_dartpy.yml` runs the same tasks
for released wheels; reproduce a CI wheel step locally with
`pixi run -e <pyXY-wheel> wheel-build` and friends.
