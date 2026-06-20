# Read the Docs Site

This directory is the Sphinx source for DART's published documentation site.
For the full documentation architecture, API-reference ownership, and
troubleshooting guidance, see
[`../onboarding/api-documentation.md`](../onboarding/api-documentation.md).

## Build And Serve Locally

```bash
pixi run docs-build
pixi run docs-serve
```

The English site is served from `docs/readthedocs/_build/html` on
<http://localhost:8000>. Korean docs use:

```bash
pixi run docs-build-ko
pixi run docs-serve-ko
```

## Source Layout

```text
docs/readthedocs/
├── conf.py                      # Main Sphinx config; also runs Doxygen
├── index.rst                    # Published site entrypoint
├── dart/                        # C++ user docs and C++ API redirect
├── dartpy/                      # Python user docs and API reference entrypoint
│   ├── user_guide/
│   ├── python_api_reference.rst
│   └── api/modules/             # Include shims for docs/python_api/modules/
├── shared/                      # Shared topic fragments included by pages
├── topics/                      # Published topic guides
├── tutorials/                   # Published tutorials
├── community/                   # Community pages
├── _ext/                        # Local Sphinx extensions
├── _static/                     # Site-local static assets
└── locales/                     # Translation catalogs
```

Generated build outputs are ignored by git:

- `_build/`
- `_generated/`
- `_generated_stubs/`
- `__pycache__/`

Use `pixi run docs-clean` to remove generated documentation outputs.

## API Reference Inputs

- C++ API docs are generated from `docs/doxygen/Doxyfile.in` during
  `pixi run docs-build` and staged under `_generated/cpp-api/`.
- dartpy API module pages are owned by `docs/python_api/modules/`.
  `docs/readthedocs/dartpy/api/modules/` contains lightweight include shims so
  the same module pages can be reused by the published RTD site and the
  standalone local Python API builder.
- `docs/readthedocs/conf.py` imports a live `dartpy` module when available and
  otherwise falls back to the committed stubs under `python/stubs/dartpy`.

## Editing Guidance

- Add user-facing pages under `dart/`, `dartpy/`, `topics/`, `tutorials/`, or
  `community/`.
- Update `index.rst` when adding a new published page that should appear in the
  site navigation.
- Update `docs/python_api/modules/` for dartpy module reference pages, not the
  include shims under `dartpy/api/modules/`.
- Keep developer and maintainer workflow docs under `docs/onboarding/` unless
  they are intentionally published as user-facing RTD pages.
- This site documents DART 7. Pages that still carry **DART 6** content show a
  legacy notice (`.. include:: /_includes/legacy_dart6.rst` for RST, or an inline
  admonition for Markdown) and are tracked in
  [`docs/onboarding/dart7-docs-migration.md`](../onboarding/dart7-docs-migration.md).
  When you port a page to DART 7, remove its notice and update that plan.
- Verify with `pixi run docs-build` for published-site changes when feasible.
