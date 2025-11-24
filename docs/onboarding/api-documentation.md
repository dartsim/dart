# API Documentation

## Overview

DART now ships every piece of documentation—user guides, tutorials, developer
notes, and the C++ API reference—through a single Sphinx project under
`docs/readthedocs`. The Read the Docs (RTD) build runs `sphinx-build` against
that tree, renders the prose content, and executes Doxygen during the
`builder-inited` hook to generate the HTML C++ API bundle. The generated
artifacts are staged inside `docs/readthedocs/_generated/cpp-api` and copied
into the built site so each RTD version automatically receives the matching API
reference with no dependency on GitHub Pages.

The Python API reference (`docs/python_api`) remains a dedicated Sphinx project.
It still requires a compiled `dartpy` module, so it is primarily useful for
local development until prebuilt wheels become available on RTD.

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
│ • Developer docs             │
│ • C++ API (Doxygen bundle)   │
│                              │
│ Build: pixi run docs-build   │
│ Deploy: RTD (docs.dartsim.org│
└──────────────────────────────┘

┌──────────────────────────────┐
│ Local API helpers            │
├──────────────────────────────┤
│ docs/python_api/ (Sphinx)    │
│   → pixi run api-docs-py     │
│ C++ Doxygen target           │
│   → pixi run api-docs-cpp    │
└──────────────────────────────┘
```

## Documentation Surfaces

### Read the Docs (primary site)

- **URL:** <https://docs.dartsim.org>
- **Source:** `docs/readthedocs/`
- **Contents:**
  - User, tutorial, and community guides
  - Developer onboarding and workflow docs
  - `dart/cpp_api_reference` which immediately redirects to
    `../cpp-api/index.html`, the freshly generated Doxygen site.
- **Key details:**
  - `docs/readthedocs/conf.py` renders a tailored Doxyfile and runs Doxygen
    during `builder-inited`.
  - Output lands in `_generated/cpp-api/`, is ignored by git, and then copied
    into the `html_extra_path` when Sphinx finishes.
  - Every RTD version therefore contains the matching C++ API bundle without an
    iframe.

### Python API reference (local project)

- **Source:** `docs/python_api/`
- **Build tool:** `pixi run api-docs-py`
- **Status:** Requires a compiled `dartpy`, so it is built locally for now. Once
  dartpy wheels are available we can consider importing it into RTD.
- **Serving:** `pixi run api-docs-serve-py` hosts the generated site on port
  8003 for quick inspection.

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

The GitHub Pages workflow and the `gh-pages` branch are no longer part of the
documentation pipeline. Historical builds remain available at their existing
URLs, but new changes are exclusively published via Read the Docs.

## Troubleshooting

### Python API appears empty on RTD

- **Cause:** RTD cannot `pip install dartpy` yet, so autodoc stubs fail.
- **Workaround:** build the Python API docs locally with `pixi run api-docs-py`
  and serve them via `pixi run api-docs-serve-py`.
- **Long-term fix:** publish wheels so RTD can import dartpy directly.

### Python API appears empty locally

- **Cause:** missing `PYTHONPATH` reference to the compiled `dartpy`.
- **Fix:** use the pixi helpers—they export the correct path before invoking
  Sphinx.

```bash
pixi run docs-build        # RTD-style docs (sets PYTHONPATH)
pixi run api-docs-py       # Standalone Python API docs
```

### C++ API missing on RTD

- **Cause:** Doxygen is not installed in the active environment or failed during
  the build.
- **Fix:** ensure `doxygen` is listed in the pixi dependencies (already true for
  the default environment). When testing locally run `pixi run docs-build` to
  confirm `_generated/cpp-api/index.html` exists before pushing.
