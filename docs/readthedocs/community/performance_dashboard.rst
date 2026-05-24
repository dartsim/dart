Performance Dashboard
=====================

DART publishes a public performance dashboard that tracks Google Benchmark
results over time:

* Dashboard:
  `https://dartsim.github.io/dart/performance/ <https://dartsim.github.io/dart/performance/>`_

The live dashboard is embedded below. If it does not load (for example before
the first publication), open it directly with the link above.

.. raw:: html

   <iframe src="https://dartsim.github.io/dart/performance/"
           title="DART performance dashboard"
           width="100%" height="720" loading="lazy"
           style="border: 1px solid #d0d7de; border-radius: 6px;"></iframe>

It leads with end-to-end world-step cases — stepping the experimental World
(sequential vs the parallel compute executor) and real robot models such as
Atlas — alongside focused lower-level kernels, so the headline numbers reflect
real simulation performance rather than micro-operations.

The dashboard is built entirely on GitHub infrastructure. A GitHub Actions
workflow runs DART's benchmark suites and hands the Google Benchmark JSON to
`benchmark-action/github-action-benchmark
<https://github.com/benchmark-action/github-action-benchmark>`_, which stores
the history on the ``gh-pages`` branch and renders an interactive Chart.js page.
There is no external account, API token, or third-party service to maintain.

The dashboard URL returns ``404`` until the first ``main`` push, scheduled, or
manually dispatched run of the **Performance Dashboard** workflow has published
to ``gh-pages``.

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
bounded ``--benchmark_filter``. No dashboard code changes are required; the
action picks up whatever rows appear in the merged JSON.

Setup (maintainers)
-------------------

The dashboard requires GitHub Pages to serve from the ``gh-pages`` branch
(Settings -> Pages -> Source: ``gh-pages`` / root). The workflow's
``contents: write`` permission lets the action create and update that branch.

Future work
-----------

The core pipeline is intentionally minimal. Natural extensions, added once the
baseline is stable, include per-PR regression comments and an optional secondary
backend such as `Bencher <https://bencher.dev>`_ or
`CodSpeed <https://codspeed.io>`_ for noise-controlled regression gating.
