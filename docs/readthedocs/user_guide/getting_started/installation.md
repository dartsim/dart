# Install DART

DART 7 is **Python-first**, but because it is still in development the default
package channels may still resolve stable DART 6 artifacts. The two DART 7 paths
tracked by this source tree are the CPython 3.14 wheel lane and a source build.

```{admonition} Default channels still install DART 6
:class: important

Plain `uv add dartpy`, `pixi add dartpy`, and `conda install dartpy` resolve the
**stable DART 6** line, which uses a different API — `dart.World()` and
`add_rigid_body` from this guide do not exist there. Install DART 7 explicitly
with one of the options below, or build from source.
```

## Python package (pre-release)

The tracked DART 7 wheel workflow builds `dartpy` for CPython 3.14 on Linux,
macOS, and Windows. Opt into pre-releases when using PyPI so a DART 7 wheel is
preferred over the stable DART 6 line:

```bash
pip install --pre dartpy           # pip (PyPI)
uv add dartpy --prerelease allow    # uv (PyPI)
```

The default `pixi add dartpy` and `conda install -c conda-forge dartpy` channels
currently track the stable **DART 6** line; use them for DART 6, or build from
source (below) for the DART 7 examples in this guide.

Verify the install by creating a tiny world in code — this does not need any
sample data files:

```python
import dartpy as dart

world = dart.World()
world.add_rigid_body("box", dart.RigidBodyOptions())
world.enter_simulation_mode()
world.step()
print(f"Stepped one frame to t = {world.time:.4f} s")
```

If that prints without error, you are ready for {doc}`hello_dart`.

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

To use your source build from a plain Python interpreter, point `PYTHONPATH` at
the built bindings (the exact path is printed at the end of `pixi run build`),
for example:

```bash
PYTHONPATH=build/default/cpp/Release/python python your_script.py
```

For more detail on supported platforms and wheels, see the
{doc}`Python installation reference </dartpy/user_guide/installation>`. For
build troubleshooting, see the developer
[building guide](https://github.com/dartsim/dart/blob/main/docs/onboarding/building.md).

## Next

You have DART installed — now run your first simulation in {doc}`hello_dart`.
