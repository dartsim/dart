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

The Python API documentation is generated using Sphinx autodoc, which requires the `dartpy` module to be importable.

**Local builds:**
- The `docs-build` task sets `PYTHONPATH` to the compiled dartpy module
- Sphinx autodoc imports and introspects the module to generate full API documentation
- All classes, methods, type hints, and docstrings are included

**Read the Docs builds:**
- dartpy is a C++ extension built with pybind11 and cannot be compiled on Read the Docs
- API documentation pages will be empty until we implement pre-built wheels
- The documentation structure and navigation will still work correctly

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

Read the Docs now installs `dartpy` prior to running Sphinx so the Python API pages
render there as well. RTD's Ubuntu 22.04 images are still on glibc 2.35, so the
requirements file pins `dartpy==6.16.0`, the last release whose wheels remain compatible
with glibc 2.27. That wheel set only targets CPython 3.12+ (cp312–cp314), so `.readthedocs.yml`
now pins the RTD runtime to Python 3.12. Local builds can continue using the
`pip install --pre dartpy` flow for the bleeding-edge bindings. Once RTD upgrades
(or we ship compatible wheels), update `docs/readthedocs/requirements.txt` and
`.readthedocs.yml` to match and remove these temporary pins.

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

Note: Developer documentation (build guides, contribution guidelines, code style)
has been consolidated to docs/onboarding/ for better maintenance and LLM accessibility.
```

## Contributing

When adding new Python API documentation:

1. Update `/docs/readthedocs/dartpy/python_api_reference.rst` to add new modules to the toctree
2. Create corresponding `.rst` files in `/docs/readthedocs/dartpy/modules/`
3. Use the `automodule` directive to auto-generate documentation
4. Test locally with `pixi run docs-build` and `pixi run docs-serve`

Example module file:
```rst
dartpy.simulation Module
========================

.. automodule:: dartpy.simulation
   :members:
   :undoc-members:
   :show-inheritance:
```
