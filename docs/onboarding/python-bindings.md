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

## Wheels

macOS and Windows wheel packaging is Pixi-managed. `wheel-build-core`,
`wheel-repair-<platform>-core`, `wheel-verify-core`, and `wheel-test-core`
implement the pipeline, and the `py310-wheel`..`py313-wheel` environments
expose the corresponding tasks.

Linux release wheels use `python3 scripts/wheel_linux.py cp313` (also
`cp310`, `cp311`, `cp312`) on native x86_64 or aarch64 hosts with Docker.
The driver builds `docker/wheels/Dockerfile.manylinux_2_28`, compiles native
dependencies and dartpy with the manylinux toolchain, requires
`auditwheel repair --plat manylinux_2_28_<arch>`, and runs the wheel smoke
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
SHA for the checkout. `ref=main` enables the release-only Python rows without
requesting publication. Inspect both architectures' repaired tags and fresh
container smoke tests before release.
