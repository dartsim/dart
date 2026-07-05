DART 7 World Performance Dashboard
==================================

DART publishes a public performance dashboard that tracks DART 7 World
Google Benchmark results over time:

* Dashboard:
  `https://dartsim.github.io/dart/performance/ <https://dartsim.github.io/dart/performance/>`_

The live dashboard is embedded below. If it does not load (for example before
the first publication), open it directly with the link above.

.. raw:: html

   <iframe src="https://dartsim.github.io/dart/performance/"
           title="DART 7 World performance dashboard"
           width="100%" height="720" loading="lazy"
           style="border: 1px solid #d0d7de; border-radius: 6px;"></iframe>

It is scoped to the DART 7 World and its solver families, tracking
their end-to-end ``World::step`` (and kinematics-update) throughput:

* **Core step & scaling** — kinematics updates, world-step throughput with
  sequential vs parallel compute executors, rigid-body step scaling, the
  contact-shaped and contact-island scalable-compute proxies, and the Phase 5
  CPU baseline row.
* **Rigid-body dynamics solver** — a stacked-box World step with the default
  sequential-impulse contact solve and the opt-in IPC barrier solve.
* **Deformable solver (Vertex Block Descent)** — a deformable-grid World step
  with the default gradient-descent solver and the VBD solver.
* **Deformable solver (FEM)** — a neo-Hookean FEM beam World step through the
  sparse projected-Newton solve.
* **Augmented VBD rigid** — a fixed-joint rigid-chain World step routed through
  the AVBD contact projection.

It deliberately excludes internal micro-kernels (distance, barrier,
tangent-stencil, candidate-set, …), CUDA/GPU-only rows, and
``DART_BUILD_DIFF``-gated rows — they either need hardware or a build flag the
GitHub-hosted runner does not provide — along with unrelated SIMD and
robot-loader surfaces, so the headline charts stay focused on experimental
World step throughput.

The dashboard is built entirely on GitHub infrastructure. A GitHub Actions
workflow runs DART's benchmark suites and hands the Google Benchmark JSON to
`benchmark-action/github-action-benchmark
<https://github.com/benchmark-action/github-action-benchmark>`_, which stores
the history on the ``gh-pages`` branch and renders an interactive Chart.js page.
There is no external account, API token, or third-party service to maintain.

The dashboard URL returns ``404`` until a ``main`` push, scheduled, or manually
dispatched run of the **Performance Dashboard** workflow has published to
``gh-pages`` and GitHub Pages has finished building the branch.

Readable labels
---------------

Raw Google Benchmark names like ``BM_VbdWorldStepVbd/16`` are rewritten into
readable chart titles such as ``Deformable world step (VBD) · 16×16 grid`` by
``scripts/benchmark_display_names.py``. The merge step applies this with
``--humanize`` so the published chart titles (and the per-PR comparison table)
read in plain language; the local preview groups the charts into the solver
families above and labels the axes (commit on x, time-per-op on y, lower is
better). Because ``github-action-benchmark`` keys history by the series name,
the first publish after this change starts fresh history lines under the new
readable names. The run-time ``--benchmark_filter`` and the
``check_compute_graph_benchmarks.py`` gate keep using the raw names, so the
rename is purely cosmetic.

How it works
------------

The ``Performance Dashboard`` workflow
(``.github/workflows/performance_dashboard.yml``) runs on every push to ``main``
that touches benchmark-relevant paths, twice weekly on a schedule, and on
manual dispatch. Each run:

#. builds and runs the bounded benchmark surfaces with
   ``scripts/run_performance_dashboard_benchmarks.py``;
#. merges the per-target JSON into one file with
   ``scripts/merge_benchmark_results.py``;
#. publishes the result with ``github-action-benchmark``, which appends a point
   to the per-benchmark history and updates the hosted page.

When a benchmark regresses past the alert threshold the action posts a comment
on the offending commit. GitHub-hosted runners are noisy, so the threshold is
set conservatively (``200%``) and a regression does not fail the build.

Preview a run locally
---------------------

You can render the same dashboard locally before anything is published::

   pixi run bm-dashboard-surfaces      # build and run the benchmark surfaces
   pixi run bm-dashboard-preview       # render build/performance-dashboard/index.html

Open ``build/performance-dashboard/index.html`` in a browser. The preview reads
the same ``window.BENCHMARK_DATA`` shape that the hosted dashboard uses, so it
matches what will be published. Pass ``--append`` to
``scripts/preview_performance_dashboard.py`` to accumulate multiple runs into a
local history.

Add a benchmark to the dashboard
--------------------------------

The dashboard reuses DART's existing Google Benchmark targets. To track a new
surface, add a ``BenchmarkSpec`` entry to
``scripts/run_performance_dashboard_benchmarks.py`` pointing at the target and a
bounded ``--benchmark_filter`` (prefer the CPU-only, end-to-end ``World::step``
rows). For readable chart titles, add a matching entry to ``SURFACES`` in
``scripts/benchmark_display_names.py``; unmapped names still render via a generic
fallback. No dashboard code changes are required; the action picks up whatever
rows appear in the merged JSON.

Setup (maintainers)
-------------------

The dashboard stores generated files on the ``gh-pages`` branch, and the
``Deploy GitHub Pages`` workflow publishes that branch tip through GitHub Pages
artifacts. Repository Pages settings must use GitHub Actions as the source;
legacy ``gh-pages`` branch publishing can leave dashboard updates queued or
served from a stale deployment.

Future work
-----------

The core pipeline is intentionally minimal. Natural extensions, added once the
baseline is stable, include per-PR regression comments and an optional secondary
backend such as `Bencher <https://bencher.dev>`_ or
`CodSpeed <https://codspeed.io>`_ for noise-controlled regression gating.
