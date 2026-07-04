DART 6 Performance Dashboard
============================

DART publishes a public performance dashboard for the DART 6 LTS benchmark
slice:

* Dashboard:
  `https://dartsim.github.io/dart/performance/dart6/ <https://dartsim.github.io/dart/performance/dart6/>`_

The live dashboard is embedded below. If it does not load, open it directly
with the link above. The URL returns ``404`` until the first successful
publication to GitHub Pages.

.. raw:: html

   <iframe src="https://dartsim.github.io/dart/performance/dart6/"
           title="DART 6 performance dashboard"
           width="100%" height="720" loading="lazy"
           style="border: 1px solid #d0d7de; border-radius: 6px;"></iframe>

Scope
-----

The DART 6 dashboard tracks bounded, CPU-only Google Benchmark rows that exist
on the maintained DART 6 branch:

* **Benchmark baseline** - the empty Google Benchmark target, used as runner
  overhead context.
* **Kinematics and dynamics** - representative ``BM_Kinematics`` and
  ``BM_Dynamics`` rows from the SKEL corpus.
* **Inverse dynamics** - recursive inverse dynamics, dense mass-matrix inverse
  dynamics, and contact-aware inverse dynamics rows.
* **Contact and collision** - stacked-box world-step rows when the Bullet
  collision target is available.
* **Contact-rich container** - active DART-native and ODE
  ``BM_ContactContainerActive`` rows from the DART 6 contact-container
  benchmark.

This dashboard is intentionally separate from the DART 7 World dashboard. DART
7 tracks experimental ``World::step`` solver-family throughput; DART 6 tracks
the stable LTS benchmark surfaces available on the current ``release-6.*`` line
including the contact-rich DART 6 rows.

Readable labels
---------------

Raw Google Benchmark names such as ``BM_ContactInverseDynamics/4`` are rewritten
into readable chart titles such as ``Contact inverse dynamics - 4 contacts`` by
``scripts/benchmark_display_names.py``. The merge step applies this with
``--humanize`` so the published chart titles and local preview read in plain
language. The run-time ``--benchmark_filter`` values still use the raw benchmark
names, so the rename is only a presentation layer.

How it works
------------

The ``DART 6 Performance Dashboard`` workflow
(``.github/workflows/performance_dashboard_dart6.yml``) runs on pushes to
``release-6.*`` branches that touch benchmark-relevant paths, on manual
dispatch, and as a reusable workflow for a default-branch scheduler. Push runs
benchmark the pushed release branch; manual and reusable runs may optionally
name a DART 6 release branch, otherwise they discover the newest
``release-6.*`` branch. Each run:

#. configures the release build with Pixi;
#. runs the bounded benchmark surfaces with
   ``scripts/run_performance_dashboard_benchmarks.py``;
#. merges the per-target JSON into one file with
   ``scripts/merge_benchmark_results.py``;
#. publishes the result with
   `benchmark-action/github-action-benchmark
   <https://github.com/benchmark-action/github-action-benchmark>`_, which
   appends a point to the per-benchmark history and updates the hosted page.

The action stores history on the ``gh-pages`` branch under ``performance/dart6``
and renders an interactive Chart.js page. There is no external account, API
token, or third-party service to maintain.

GitHub Actions only runs ``schedule`` triggers from workflows present on the
repository default branch. Keep recurring DART 6 dashboard publication in a
default-branch scheduler that calls this release workflow with
``workflow_call``; do not add a release-branch-only cron and expect it to run.

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
DART 6 surface, add a ``BenchmarkSpec`` entry to
``scripts/run_performance_dashboard_benchmarks.py`` with a bounded
``--benchmark_filter`` and add a matching readable label in
``scripts/benchmark_display_names.py``. Prefer stable, CPU-only rows that can
run on GitHub-hosted Ubuntu without special hardware.

Setup (maintainers)
-------------------

The dashboard requires GitHub Pages to serve from the ``gh-pages`` branch
(Settings -> Pages -> Source: ``gh-pages`` / root). The workflow's
``contents: write`` permission lets the action create and update that branch.
