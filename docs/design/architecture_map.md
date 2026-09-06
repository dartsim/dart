# Architecture Map

Status: accepted 2026-09-05; implemented by PLAN-130 (archived). This document
owns the durable decisions behind the living architecture map that
[`docs/readthedocs/architecture.md`](../readthedocs/architecture.md) embeds.
Procedures live in the `dart-architecture` skill; the views live under
[`docs/assets/architecture/`](../assets/README.md).

## Purpose

Developers and advanced users need one current picture of the DART 7 core
simulation framework: its architecture, the `World::step()` data flow, the
compute graph, and where the framework sits in the whole library. The picture
must stay current without periodic cleanups, so it is generated from typed
sources that CI checks against the code.

## Decisions

1. **Typed JSON is the source of truth, pictures are build output.** One
   archify view per topic under `docs/assets/architecture/`
   (`*.architecture.json`, `*.dataflow.json`). Rendered HTML lands in
   `docs/readthedocs/_generated/architecture-map/`, which is gitignored and
   copied into the site through `html_extra_path`, exactly like the Doxygen
   bundle. Nothing rendered is committed.
2. **Renderer: archify, pinned by tag, fetched, never vendored.**
   [archify](https://github.com/tt-a1i/archify) (MIT) compiles the JSON into
   self-contained interactive HTML with source-evidence markers and
   compare/delta receipts. `scripts/render_architecture_map.py` clones the
   pinned tag into `.deps/archify` (the convention already used for
   gz-physics), verifies the recorded commit, and runs it with
   `ARCHIFY_UPDATE_CHECK_DISABLED=1`. Mermaid was rejected because it has no
   evidence markers, guided views, or geometry validation; vendoring was
   rejected to keep the tree free of a second JavaScript codebase.
3. **Node.js comes from pixi and Read the Docs, and its absence degrades.**
   `nodejs` sits in the default pixi dependencies (prettier already pulled it
   in, so the lockfile did not change) and `.readthedocs.yml` declares
   `build.tools.nodejs`. `docs/readthedocs/conf.py` runs the driver on
   `builder-inited` next to the Doxygen hook. Without Node.js or the
   checkout, the driver writes text fallbacks and the build warns; an invalid
   view is a content error and fails the build.
4. **Evidence is verified at the current tree.** Tracked views omit
   `meta.repository`; the driver stamps the current `HEAD` into a temporary
   copy and runs archify with `--repo-root`, so `SRC` markers open the built
   commit. The blocking gate, `scripts/check_architecture_map.py`, needs no
   Node.js and runs inside `pixi run check-lint`.
5. **Two liveness tiers.** Blocking: cited paths exist, cited line ranges
   still hold the symbol they are labelled with, cited symbols resolve;
   every `dart/simulation/<dir>`, `BuiltInWorldStepStageSlot`
   enumerator, enumerator of the public selector enums in `world_options.hpp`
   and `multibody/multibody_options.hpp`, `WorldStepStage` subclass, and
   `dart/<module>` directory appears in the
   view that owns it (allowlists carry a reason per exemption); stage ids are
   the enumerator names in snake case wherever a view mentions a stage; the
   page embeds every view and names every source. Advisory:
   `scripts/check_architecture_map_runtime.py` compares the committed
   `compute-graph.runtime.json` fixture, recorded by
   `tests/unit/simulation/compute/test_architecture_probe.cpp`, with the views
   and with a fresh probe dump when the binary exists; `--strict` promotes it.
6. **One status vocabulary and one type legend.** Node tags reuse the
   assessment's `Implemented`, `Partial`, `Planned`, `Undecided`; dashed
   relationships lead to planned work. Archify's fixed component palette maps
   to DART meanings documented once on the page and in the skill.
7. **Views are English, like code identifiers.** Archify's viewer supports
   `en` and `zh-CN` only and JSON labels are outside gettext extraction; the
   page prose around each embed stays translatable, so the Korean build keeps
   working.
8. **Rendered output is offline-clean and matches the site theme.** The driver
   strips the Google Fonts links (archify loads them asynchronously with a
   system fallback, so this is a privacy choice, not a reliability one) and
   the page embeds each view with `?theme=light&embed=1` so it matches
   `sphinx_rtd_theme`; the "open the interactive view" link gives the full
   viewer.
9. **Owners trigger updates, not new surfaces.** The assessment's standing
   audit rule and the "Keeping Docs Current" table in
   [`docs/README.md`](../README.md) name the map as an affected artifact, the
   pull-request template carries one checklist line, and the
   `dart-architecture` skill owns the update and audit procedures. No new
   command was added; `pixi run` tasks are the tool-independent public path.
10. **The classic page shrank instead of duplicating the map.**
    [`docs/onboarding/architecture.md`](../onboarding/architecture.md) keeps
    the DART 6 classic core in about 120 lines; its module facts moved into
    the library-context view.

## Non-Goals

No second renderer or Mermaid projection; no class-level or per-file
diagrams; no automatic generation of the architecture views from source (only
the runtime probe is derived); no archify fork or `development` channel; no
archify `visual-check` browser evidence; no ECS storage, collision-library
internals, or CUDA residency detail in any view; no fifth view; no change to
the linted availability table on the page; no dartsim GUI integration.

## Files

| File                                                        | Role                                                         |
| ----------------------------------------------------------- | ------------------------------------------------------------ |
| `docs/assets/architecture/*.json`                           | Four views plus the runtime fixture                          |
| `scripts/render_architecture_map.py`                        | Pinned archify fetch, revision stamping, render, fallbacks   |
| `scripts/check_architecture_map.py`                         | Blocking structure, evidence, coverage, and page-embed gate  |
| `scripts/check_architecture_map_runtime.py`                 | Advisory drift check and fixture regeneration                |
| `tests/unit/simulation/compute/test_architecture_probe.cpp` | Recording executor over a reference scene                    |
| `docs/readthedocs/architecture.md`                          | Published page that embeds the views with translatable prose |
| `.claude/skills/dart-architecture/SKILL.md`                 | Update and audit procedures and archify authoring limits     |
