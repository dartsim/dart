# Documentation Assets

This directory stores source-controlled assets used by repository
documentation outside the Read the Docs site-local `_static/` tree.

Use this directory for durable assets that should survive beyond one PR, such
as reusable diagrams, small media files, or images referenced by docs under
`docs/`.

Do not use this directory for transient review evidence, before/after captures,
or local debugging artifacts. PR evidence belongs in the PR description or
comment thread, using GitHub-hosted attachments when needed. Site-local assets
used only by the published Sphinx documentation belong under
`docs/readthedocs/_static/`.
