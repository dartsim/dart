# API Documentation

## Overview

DART's primary documentation site is the Sphinx project under
`docs/readthedocs`. It owns user guides, tutorials, community pages, the
published dartpy API reference, and the embedded C++ Doxygen bundle. The Read
the Docs (RTD) build runs `sphinx-build` against that tree, renders the prose
content, and executes Doxygen during the `builder-inited` hook to generate the
HTML C++ API bundle. The generated artifacts are staged inside
`docs/readthedocs/_generated/cpp-api` and copied into the built site so each RTD
version receives the matching C++ API reference with no dependency on GitHub
Pages.

The reusable dartpy module pages live under `docs/python_api`. RTD includes them
through shim pages in `docs/readthedocs/dartpy/api/modules/`. During the main RTD
build, `docs/readthedocs/conf.py` imports a live `dartpy` module when one is
already available; otherwise it falls back to the committed stubs under
`python/stubs/dartpy` so the published API layout follows this repository rather
than an older wheel.

### Architecture Diagram

```text
┌────────────────────────────────────────────────────────────────┐
│                    DART Documentation System                    │
└────────────────────────────────────────────────────────────────┘

┌──────────────────────────────┐
│   Read the Docs build        │
│   docs/readthedocs/          │
├──────────────────────────────┤
│ Contains:                    │
│ • User guides & tutorials    │
│ • dartpy API reference       │
│ • C++ API (Doxygen bundle)   │
│                              │
│ Build: pixi run docs-build   │
│ Deploy: RTD (docs.dartsim.org│
└──────────────────────────────┘

┌──────────────────────────────┐
│ Reusable API inputs          │
├──────────────────────────────┤
│ docs/python_api/ modules     │
│   → included by RTD shims    │
│ docs/doxygen/ config/input   │
│   → consumed by RTD build    │
│ Local standalone helpers     │
│   → pixi run api-docs-*      │
└──────────────────────────────┘
```

## Documentation Surfaces

The generated C++ API reference is a user-facing public API surface. It should
favor public and compatibility APIs, while implementation details belong under
`detail/` or `internal/` and should be marked with `@internal` when they must
appear in public headers. See [api-boundaries.md](api-boundaries.md) for the
boundary policy.

### Read the Docs (primary site)

- **URL:** <https://docs.dartsim.org>
- **Source:** `docs/readthedocs/`
- **Contents:**
  - User, tutorial, and community guides
  - Published dartpy API reference using `docs/python_api` module pages
  - `dart/cpp_api_reference` which immediately redirects to
    `../cpp-api/index.html`, the freshly generated Doxygen site.
- **Key details:**
  - `docs/readthedocs/conf.py` renders a tailored Doxyfile and runs Doxygen
    during `builder-inited`.
  - Output lands in `_generated/cpp-api/`, is ignored by git, and then copied
    into the `html_extra_path` when Sphinx finishes.
  - Every RTD version therefore contains the matching C++ API bundle without an
    iframe.

### Python API module pages

- **Source:** `docs/python_api/`
- **Published through:** `docs/readthedocs/dartpy/api/modules/` include shims
- **Local standalone build:** `pixi run api-docs-py`
- **Status:** The primary RTD site uses a live import when available and falls
  back to committed stubs. The standalone local project builds against a freshly
  compiled `dartpy` extension.
- **Serving:** `pixi run api-docs-serve-py` hosts the standalone generated site
  on port 8003 for quick inspection.

### Doxygen input

- **Source:** `docs/doxygen/`
- **Published through:** `docs/readthedocs/conf.py` during `docs-build`
- **Status:** `Doxyfile.in` should include only C/C++ and Doxygen input
  patterns so agent Markdown and other repository notes do not leak into the C++
  API reference.

## Building Locally

### Read the Docs equivalent

```bash
pixi run docs-build        # Build the RTD site (includes C++ Doxygen bundle)
pixi run docs-serve        # Serve at http://localhost:8000
```

### API documentation helpers

```bash
pixi run api-docs-cpp      # Build the standalone Doxygen site (docs target)
pixi run api-docs-serve-cpp
pixi run api-docs-py       # Build docs/python_api against the local dartpy build
pixi run api-docs-serve-py
pixi run api-docs-build    # Convenience task that runs both builders
```

## Configuration Files

| File                       | Purpose                                                             |
| -------------------------- | ------------------------------------------------------------------- |
| `.readthedocs.yml`         | RTD build definition                                                |
| `docs/readthedocs/conf.py` | Sphinx configuration for the main documentation site (runs Doxygen) |
| `docs/python_api/conf.py`  | Standalone Python API Sphinx configuration                          |
| `docs/doxygen/Doxyfile.in` | Doxygen template consumed by `docs/readthedocs/conf.py`             |
| `pixi.toml`                | Local task definitions (`docs-build`, `api-docs-*`, etc.)           |

## Deployment Pipeline

### Read the Docs

- **Trigger:** push to `main` or manual rebuilds from the RTD dashboard.
- **Steps:**
  1. RTD reads `.readthedocs.yml`.
  2. The pixi environment installs the full toolchain (including Doxygen).
  3. `sphinx-build` runs inside `docs/readthedocs/`. When the builder fires the
     `builder-inited` event, `conf.py` renders `Doxyfile.in`, runs Doxygen, and
     stages the HTML bundle inside `_generated/cpp-api/`.
  4. Sphinx copies `_generated/cpp-api/` into the output tree so the RTD site
     serves the API reference next to the rest of the pages.

### GitHub Pages

GitHub Pages is no longer part of the primary documentation pipeline.
Historical C++ API builds remain available at their existing
`https://dartsim.github.io/dart/v6.*` URLs, and the performance dashboard is
published from the `gh-pages` branch at
`https://dartsim.github.io/dart/performance/`. The Community Signals dashboard
is published from the same branch at
`https://dartsim.github.io/dart/community-signals/`.

The `gh-pages` branch stores those generated static surfaces; the
`Deploy GitHub Pages` workflow packages and deploys the branch tip as a Pages
artifact after the dashboard publishers complete. User-facing documentation
changes should still go to Read the Docs unless they specifically maintain one
of those GitHub Pages surfaces.

## Troubleshooting

### Python API appears empty on RTD

- **Cause:** RTD could not import a live `dartpy` module and the committed
  stubs under `python/stubs/dartpy` were missing or not importable.
- **Workaround:** run `pixi run docs-build` locally and inspect the autodoc
  warning. Use `pixi run generate-stubs` when the stubs need to be refreshed.
- **Long-term fix:** keep committed stubs in sync with the current bindings, and
  optionally let RTD import a current release wheel when one is compatible.

### Python API appears empty locally

- **Cause:** for the standalone Python API project, missing `PYTHONPATH`
  reference to the compiled `dartpy`; for the RTD-style project, missing or stale
  committed stubs.
- **Fix:** use `pixi run api-docs-py` when you need the compiled extension, or
  refresh stubs with `pixi run generate-stubs` when validating the RTD fallback.

```bash
pixi run docs-build        # RTD-style docs; uses live dartpy if importable
pixi run api-docs-py       # Standalone Python API docs; builds dartpy first
```

### C++ API missing on RTD

- **Cause:** Doxygen is not installed in the active environment or failed during
  the build.
- **Fix:** ensure `doxygen` is listed in the pixi dependencies (already true for
  the default environment). When testing locally run `pixi run docs-build` to
  confirm `_generated/cpp-api/index.html` exists before pushing.
