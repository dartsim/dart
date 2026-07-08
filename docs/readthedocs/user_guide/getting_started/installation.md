# Install DART

DART 7 is **Python-first**, but because it is still in development, public
package channels still resolve stable DART 6 artifacts today. The current DART 7
path is a source build from this repository; the CPython 3.14 wheel lane becomes
an install path after a non-yanked DART 7 `dartpy` wheel is published.

```{admonition} Public package channels still install DART 6
:class: important

`pip install --pre dartpy`, `uv add dartpy --prerelease allow`,
`pixi add dartpy`, and `conda install dartpy` do not currently provide the DART
7 API used by this guide. They resolve the **stable DART 6** line, or no usable
non-yanked DART 7 wheel. Use a source build for DART 7 examples until a
non-yanked DART 7 wheel is available on PyPI.
```

## Build from source

A source checkout tracks the `main` branch and gives you the full DART 7 API
plus the interactive demos. DART uses [pixi](https://pixi.sh) to provide a
reproducible toolchain:

```bash
git clone https://github.com/dartsim/dart.git
cd dart
pixi install            # fetch the toolchain and dependencies
pixi run build          # build C++ and the dartpy bindings
```

Run a headless smoke check to confirm the build works:

```bash
pixi run py-demos -- --scene rigid_body --headless --frames 1
```

Verify the install by creating a tiny world in code. Point `PYTHONPATH` at the
built bindings before importing `dartpy`; this does not need any sample data
files:

```bash
PYTHONPATH=build/default/cpp/Release/python python - <<'PY'
import dartpy as dart

world = dart.World()
world.add_rigid_body("box", dart.RigidBodyOptions())
world.enter_simulation_mode()
world.step()
print(f"Stepped one frame to t = {world.time:.4f} s")
PY
```

If that prints without error, you are ready for {doc}`hello_dart`.

Use the same prefix for your own scripts:

```bash
PYTHONPATH=build/default/cpp/Release/python python your_script.py
```

## Python package status

The tracked DART 7 wheel workflow builds `dartpy` for CPython 3.14 on Linux,
macOS, and Windows. After a non-yanked DART 7 wheel is published on PyPI, opt
into pre-release resolution so the DART 7 wheel is preferred over the stable
DART 6 line:

```bash
pip install --pre dartpy
uv add dartpy --prerelease allow
```

The default `pixi add dartpy` and `conda install -c conda-forge dartpy` channels
currently track the stable **DART 6** line; use them for DART 6, or build from
source for the DART 7 examples in this guide.

For more detail on supported platforms and wheels, see the
{doc}`Python installation reference </dartpy/user_guide/installation>`. For
build troubleshooting, see the developer
[building guide](https://github.com/dartsim/dart/blob/main/docs/onboarding/building.md).

## Next

You have DART installed — now run your first simulation in {doc}`hello_dart`.
