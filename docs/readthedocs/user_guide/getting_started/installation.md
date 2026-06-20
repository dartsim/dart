# Install DART

DART 7 is **Python-first**: the quickest way to start is the `dartpy` package.
Building from source gives you the newest DART 7 surface and the interactive
viewer.

```{admonition} Pre-release packages
:class: note

DART 7 artifacts are published as pre-releases while the API stabilizes. Until a
final DART 7 release is tagged, package managers may still resolve the latest
DART 6 artifacts unless you opt into the pre-release. When in doubt, build from
source for the examples in this guide.
```

## Python package

Use your preferred package manager to add `dartpy` to an environment:

```bash
uv add dartpy                 # uv (recommended for Python-first projects)
pip install dartpy --pre      # PyPI wheels; --pre opts into DART 7 pre-releases
pixi add dartpy               # Pixi environment
conda install -c conda-forge dartpy
```

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
