# DART Documentation

This directory contains the source for DART's documentation, including both C++ and Python API documentation.

## Building Documentation Locally

### Prerequisites

1. Install pixi (if not already installed):

   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   ```

2. Build dartpy (Python bindings):
   ```bash
   pixi run build-py-dev
   ```

### Build and Serve Documentation

```bash
# Build documentation
pixi run docs-build

# Serve documentation (opens on http://localhost:8000)
pixi run docs-serve
```

The documentation will be available at: `http://localhost:8000`

### Korean Documentation

```bash
# Build Korean documentation
pixi run docs-build-ko

# Serve Korean documentation (opens on http://localhost:8001)
pixi run docs-serve-ko
```

## Python API Documentation

### How It Works

The Python API documentation is generated using Sphinx autodoc, which requires the `dartpy` module (nanobind extension) to be importable.

**Local builds:**

- The `docs-build` task sets `PYTHONPATH` to the compiled dartpy module
- Sphinx autodoc imports and introspects the module to generate full API documentation
- All classes, methods, type hints, and docstrings are included

**Read the Docs builds:**

- Read the Docs cannot compile the C++ extension, so `conf.py` falls back to generated stubs under `python/stubs/dartpy`.
- If compatible wheels are available for RTD’s image, the requirements file pins one so autodoc can use the real module; otherwise the stubs keep the API pages rendering.

### Generating Stub Files

For IDE support and type checking, you can generate Python stub files (`.pyi`):

```bash
pixi run generate-stubs
```

Stub files will be generated in `/python/stubs/dartpy/` and can be used by IDEs like PyCharm, VS Code, and type checkers like mypy.

## Documentation Build Approach

### Local Builds ✅

**Full Python API documentation** is generated using the compiled dartpy module:

```bash
# Build documentation (includes Python API)
pixi run docs-build

# Serve documentation
pixi run docs-serve  # English at http://localhost:8000
pixi run docs-serve-ko  # Korean at http://localhost:8001
```

The `docs-build` task:

1. Builds the compiled dartpy module (`build-py-dev`)
2. Sets PYTHONPATH to the compiled module location
3. Runs Sphinx autodoc to introspect and document the module
4. Generates complete API documentation with all classes, methods, and type hints

### Read the Docs ✅

Read the Docs currently relies on prebuilt `dartpy` wheels when they match the RTD
image; otherwise it uses the shipped stubs. The requirements pin may change as RTD
images and wheel compatibility evolve—adjust `docs/readthedocs/requirements.txt` and
`.readthedocs.yml` when newer wheels are usable so autodoc can introspect the live module.

### Future: keep wheels fresh 🎯

To ensure documentation keeps working everywhere:

1. Keep cibuildwheel jobs green so every tagged release produces fresh wheels.
2. Publish those wheels to PyPI (or a public index) before kicking off RTD builds.
3. Periodically audit `.readthedocs.yml` so it tracks the latest compatible wheel
   versions and Python ranges.

## Directory Structure

```
docs/readthedocs/
├── conf.py                          # Sphinx configuration
├── index.rst                         # Main documentation index
├── developer_resources.rst           # Points to docs/onboarding/ for developer docs
├── dart/                            # C++ user documentation
│   └── user_guide/
├── dartpy/                          # Python user documentation
│   ├── user_guide/
│   ├── developer_guide/             # API docs architecture info only
│   ├── python_api_reference.rst     # Main API reference page
│   └── modules/                     # Individual module documentation
│       ├── collision.rst
│       ├── dynamics.rst
│       ├── simulation.rst
│       └── ...
├── _static/                         # Static assets (images, CSS)
└── locales/                         # Translations (Korean, etc.)

> **Note:** Developer documentation (build guides, contribution guidelines, code style)
> has been consolidated to docs/onboarding/ for better maintenance and LLM accessibility.
```

## Contributing

When adding new Python API documentation:

1. Update `/docs/readthedocs/dartpy/python_api_reference.rst` to add new modules to the toctree
2. Create corresponding `.rst` files in `/docs/readthedocs/dartpy/modules/`
3. Use the `automodule` directive to auto-generate documentation
4. Test locally with `pixi run docs-build` and `pixi run docs-serve`

Example module file:

```rst
dartpy Module
========================

.. automodule:: dartpy
   :members:
   :undoc-members:
   :show-inheritance:
```
