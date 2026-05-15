# DART Python Examples

## Overview

- Examples live flat under `python/examples/` for simpler discovery and tooling.
- Legacy Python examples that depended on the removed OpenSceneGraph GUI
  classes were removed. The remaining example is headless and works with the
  maintained dartpy core API.
- GUI-oriented Python coverage now lives in tests for
  `dartpy.gui.experimental`, which exposes backend-hidden Filament descriptors.

## Categories (Ordered)

### 00 Getting Started

- `hello_world`

## Run Examples

If you are working inside the DART repo, prefer the `pixi run` entry points
documented in `docs/onboarding/building.md` when available.

For example:

    $ pixi run py-ex hello_world

Or, without pixi:

    $ PYTHONPATH=build/<env>/cpp/<build_type>/python \
      python python/examples/hello_world/main.py
