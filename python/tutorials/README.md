# DART Python Tutorials

Jupyter / Colab notebooks that exercise DART's Python API. Per PLAN-103, the
notebooks **import** the demo scene modules from `python/examples/demos` so
scene logic is single-sourced; new tutorials extend this surface instead of
copying scene code.

## Notebooks

- [`01_browse_demos.ipynb`](01_browse_demos.ipynb) — list the demo catalog,
  build a scene, step it, and run the cycle from a notebook.

## Run locally

From the repo root:

```bash
pixi run build
PYTHONPATH=build/default/cpp/Release/python:python \
    jupyter notebook python/tutorials
```

The first cell of each notebook auto-fixes `sys.path` so `examples.demos`
imports work when the notebook is opened from `python/tutorials/`.

## Colab

Cloud publication (the wheel install + the "open in Colab" links) is owned by
**PLAN-012 (Cloud Dartpy Tutorials)**. The notebook pattern these tutorials use
in Colab is:

```python
# Install the DART 7 dartpy wheel + checkout this repo to import examples.demos.
%pip install dartpy
!git clone --depth 1 https://github.com/dartsim/dart /tmp/dart
%cd /tmp/dart
import sys; sys.path.insert(0, 'python')
from examples.demos import make_demo_scenes, run
```

PLAN-012 will publish stable Colab-ready notebook links once the DART 7 dartpy
wheel is on PyPI and the Colab runtime spike (headless simulation + a
Filament-backed inline frame) is green.

## Add a tutorial

1. Create `python/tutorials/NN_<topic>.ipynb`.
2. Use the same `sys.path` setup cell as `01_browse_demos.ipynb` so
   `examples.demos` imports work locally and on Colab.
3. Import the scene modules you want to demonstrate; do not duplicate scene
   logic in the notebook.
4. Update this README's "Notebooks" list.
