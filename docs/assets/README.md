# Documentation Assets

This directory stores source-controlled assets used by repository documentation
outside the Read the Docs site-local `_static/` tree.

Use this directory for durable assets that should survive beyond one PR, such
as reusable images, diagrams, or small media files referenced by docs under
`docs/`.

Do not use this directory for transient review evidence, before/after captures,
or local debugging artifacts. PR evidence belongs in the PR description or
comment thread, using GitHub-hosted attachments when needed. Site-local assets
used only by the published Sphinx documentation belong under
`docs/readthedocs/_static/`.

## `architecture/`

The typed archify views (`*.architecture.json`, `*.dataflow.json`) and the
runtime fixture (`compute-graph.runtime.json`) that make up the living
architecture map embedded by
[`readthedocs/architecture.md`](../readthedocs/architecture.md). They are the
source of truth: `scripts/render_architecture_map.py` renders them during the
docs build, `pixi run check-architecture-map` blocks drift, and
`pixi run check-architecture-map-runtime` reports schedule drift. [`design/architecture_map.md`](../design/architecture_map.md) owns the
decisions; the `dart-architecture` skill owns the update and audit procedures.
