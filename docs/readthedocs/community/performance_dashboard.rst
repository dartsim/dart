Performance Dashboard
=====================

DART's performance dashboard is the public benchmark history for the project.
It is published as a static site on GitHub Pages:

* Dashboard:
  `https://dartsim.github.io/dart/performance/ <https://dartsim.github.io/dart/performance/>`_
* Health endpoint:
  `status.json <https://dartsim.github.io/dart/performance/status.json>`_
* Normalized data:
  `data.json <https://dartsim.github.io/dart/performance/data.json>`_

The dashboard URL can return ``404`` until the first main-push, scheduled, or
manually dispatched dashboard workflow has published ``gh-pages/performance/``.
After that first publication, ``status.json`` is the fastest way to check
whether the hosted dashboard is present and fresh.
The dashboard page and ``summary.md`` also render an
``External Competitor Status`` section from the same generated metadata, so
readers can see whether external comparison rows are live without inspecting
raw JSON.

Quick Start
-----------

After the first publication, use the dashboard in this order:

* Open the dashboard page for the human-readable trend, latest-run,
  native/reference, external-competitor, service-decision, and freshness
  views.
* Use ``status.json`` for health checks, monitoring, freshness, latest-run
  identity, compact trend/testbed metadata, and public link verification.
* Download ``data.json`` when scripts need the full normalized benchmark
  history, comparison coverage, service decision, and publication contract.
* Use ``summary.md`` for release notes, GitHub Actions summaries, or compact
  text reports.
* Add competitor rows by feeding repeatable Google Benchmark JSON into the
  same dashboard render and following the backend suffix rule in
  `Adding External Competitor Rows`_.
* Maintainers launch the first hosted version by landing the workflow on
  ``main``, dispatching ``performance_dashboard.yml``, then running the live
  launch checks in `Maintainer Launch`_.

What It Shows
-------------

The dashboard tracks benchmark measurements over time rather than relying on
short-lived GitHub Actions artifacts. It includes latest-run summaries,
previous-run changes, reference-backend comparisons, backend matrices,
reference/competitor coverage, recent run history, and freshness status.

Native/reference comparisons are shown only where DART has reproducible
benchmark input data for the compared backend. Historical external comparisons,
such as gallery videos or ROSCon-era physics-engine comparisons, are useful
context but are not treated as live dashboard data unless they are backed by a
repeatable dashboard benchmark. The coverage table lists deferred competitor
or reference candidates, such as MuJoCo, Drake, Gazebo/gz-physics, and
PyBullet, as a harness plan rather than as a performance claim.

Using The Hosted Dashboard
--------------------------

Use the dashboard page for interactive inspection and ``status.json`` for a
quick health check. A fresh ``status.json`` response means the hosted files are
present, the latest run identity is known, and the dashboard has updated within
the expected publisher cadence. It also exposes the live external competitor
row count, backend names, surfaces, and sample comparison groups, so monitoring
can tell whether competitor harnesses are present without downloading the full
history. The compact ``trend_summary`` field reports latest primary/reference
trend row counts, status counts, threshold counts, and largest improvement or
regression snapshots, so monitoring can detect over-time movement without
downloading the full history. ``data.json`` is the normalized durable history
for scripts or external analysis, while ``summary.md`` is the compact text view
used by CI summaries and release notes.
The compact ``testbed_summary`` field records the latest run's testbed,
source-file count, host names, CPU counts, and CPU-scaling/ASLR state counts so
readers can tell whether apparent over-time movement may come from a changed
benchmark environment.

For a quick terminal check after the first publication:

.. code-block:: bash

   curl -fsSL https://dartsim.github.io/dart/performance/status.json | python -m json.tool
   curl -fsSL https://dartsim.github.io/dart/performance/summary.md

For comparisons, read "native" or "primary" rows as DART's current benchmarked
implementation and "reference" rows as explicitly reproduced backend data from
the same dashboard input. Deferred competitors in the coverage table are the
next harnesses to build, not measured results. Use the run source link and
testbed fields before comparing absolute timings across machines or dates.
The generated ``comparison_metric_contract`` in ``data.json`` and
``status.json`` defines the ratio fields used by scripts and readers:
``backend_vs_primary_ratio`` is ``backend_ns / primary_ns`` for backend-matrix
rows, while latest native/reference comparison rows use
``native_ns / best_reference_ns``. In the backend matrix, ``ratio < 1`` means
the backend/reference row was faster than the primary DART row; in the latest
native/reference table, ``ratio < 1`` means the primary DART row leads the best
reference row.

Adding External Competitor Rows
-------------------------------

External engines become live competitors only through repeatable Google
Benchmark JSON that is fed into the same dashboard render as DART's native
rows. Name those rows with a backend suffix before any Google Benchmark
parameters:

.. code-block:: text

   BM_Distance_BoxSphere_Native
   BM_Distance_BoxSphere_MuJoCo
   BM_KR5ForwardDynamics_DART/1
   BM_KR5ForwardDynamics_Drake/1

The prefix before ``_<Backend>`` is the comparison group. A DART row such as
``BM_Distance_BoxSphere_Native`` and a competitor row such as
``BM_Distance_BoxSphere_MuJoCo`` are compared because they share the
``BM_Distance_BoxSphere`` group. Supported external competitor suffixes are
``Chrono``, ``Drake``, ``Gazebo``, ``GzPhysics``, ``Isaac``, ``IsaacGym``,
``MuJoCo``, ``PyBullet``, ``RaiSim``, and ``Simbody``. The generated
``data.json`` and ``status.json`` files publish this as
``comparison_input_contract`` so external harnesses and checks can validate the
same rule programmatically.

How It Updates
--------------

The main-push/scheduled/manual GitHub Actions workflow runs DART's Google
Benchmark surfaces, normalizes the JSON results, appends them to durable
dashboard history, renders the static site, and publishes it to the
repository's ``gh-pages`` branch. On first publication, it may seed history
from recent benchmark artifacts, but only artifacts from the publishing branch
are eligible, so pull request or feature-branch artifacts cannot seed the
public ``main`` dashboard history. It loads existing history before cleaning
the target ``gh-pages/performance/`` files, so old dashboard files do not
remain published after the current static payload is written. The generated
``status.json`` endpoint records the latest run identity, source commit,
measurement counts, dashboard file manifest, freshness state, compact live
external-competitor backend metadata, ``trend_summary`` change-over-time
metadata, and ``testbed_summary`` environment metadata. It also records the
canonical website, dashboard, status, data, summary, and guide URLs so endpoint
checks prove the hosted dashboard is wired to the intended public DART
surfaces. The published workflow verifies that the public ``data.json``
endpoint matches the generated durable history file.

The generated dashboard also includes the service decision used for the first
public version: DART-owned GitHub Pages is the canonical dashboard and durable
data owner, Bencher is the recommended external history/threshold service after
maintainer approval, CodSpeed is deferred to a later microbenchmark pilot,
``github-action-benchmark`` is documented as an off-the-shelf Pages action that
is not selected because DART needs custom native/reference and competitor
coverage metadata, Conbench remains a self-hosted fallback if the lighter setup
becomes limiting, Airspeed Velocity is a not-selected static benchmark
dashboard framework because it would require an ASV benchmark-suite adapter
instead of directly using DART's existing Google Benchmark JSON, LLVM LNT is a
not-selected self-hosted performance-tracking server option because it would
require adapter/schema and infrastructure ownership before it could replace the
first DART-owned static dashboard, and
OpenBenchmarking.org/Phoronix Test Suite is reserved for future public
ecosystem benchmark campaigns rather than the first Google Benchmark JSON
dashboard. It also publishes ``publication_contract`` in
``data.json`` and
``status.json``. That contract records the GitHub Pages host, ``gh-pages``
source branch, ``performance/`` dashboard path, ``Performance Dashboard``
workflow, ``refs/heads/main`` publish ref, required ``schedule_crons`` cadence,
expected update interval, stale-after window, required ``actions: read``,
``contents: write``, and ``pages: write`` permissions, required hosted
endpoints, required canonical website links to the dashboard, health endpoint,
and guide, launch checks, and the maintainer approval boundary for pushes, PR
creation, merges, workflow dispatches, and Pages publication. The launch checks
require the latest successful ``main`` workflow run to expose the
``performance-dashboard-site-<run_id>-<run_attempt>`` artifact and require the
public ``status.json`` ``latest_run_id`` and source SHA to match that latest
workflow run before treating the hosted dashboard as live.
The compact ``service_decision_summary`` field states the recommendation
directly for monitoring and release notes: use DART-owned GitHub Pages as the
canonical dashboard/data host, then add Bencher as an opt-in external
history/threshold companion after maintainer approval.
The same files publish ``comparison_metric_contract`` so ratio formulas and
status meanings stay machine-readable alongside the rendered dashboard.

``freshness.state`` should be ``fresh`` after the expected scheduled publisher
cadence. A stale state means the dashboard is still hosted, but the benchmark
history has not been updated recently enough to trust as current performance
evidence.

Developer Preview
-----------------

Contributors can render and verify the same static payload locally before it is
published:

.. code-block:: bash

   pixi run bm-dashboard
   pixi run check-bm-dashboard
   pixi run check-bm-dashboard-publication
   pixi run check-bm-dashboard-launch-preflight

Open ``build/performance-dashboard/index.html`` for the local preview. The
publication preflight verifies the local files, confirms this Read the Docs
site is reachable, and reports whether the dashboard guide and public dashboard
endpoint are already live. Before the branch lands on ``main``, it is expected
to report the guide and dashboard endpoint as unpublished. The launch preflight
adds the hosted-path checks that matter before opening or merging the dashboard
publisher PR: repository Pages configuration, the ``gh-pages`` source-branch
dashboard path, local workflow registration shape, current external
service-decision evidence URLs, recent seed artifact download, workflow-shaped
``gh-pages/performance/`` rendering, and publication endpoint state.

Maintainer Launch
-----------------

The hosted dashboard is created by the reviewed workflow on ``main``. After the
dashboard workflow lands, maintainers should confirm that GitHub Actions has
registered it:

.. code-block:: bash

   pixi run check-bm-dashboard-pages-branch
   pixi run check-bm-dashboard-pages-build
   pixi run check-bm-dashboard-workflow-registration

The Pages branch check may report that ``gh-pages/performance/`` is absent
before the first publisher run. That is acceptable before dispatching the
workflow; the post-publication live launch gate requires that path to exist and
contain the dashboard files.

Then dispatch the ``Performance Dashboard`` workflow from the GitHub Actions UI
or with the GitHub CLI:

.. code-block:: bash

   gh workflow run performance_dashboard.yml --ref main

When the run completes, verify the public endpoints:

.. code-block:: bash

   pixi run check-bm-dashboard-workflow-run
   pixi run check-bm-dashboard-launch-live

That check requires the generated local dashboard to be valid, this Read the
Docs guide to be reachable, the dashboard workflow to be registered on
``main``, the latest ``main`` dashboard workflow run to have completed
successfully with the hosted-site dashboard artifact, the repository's Pages
configuration to still point at the expected ``gh-pages`` branch-source site,
the latest GitHub Pages build to be
``built`` for the current ``gh-pages`` commit, a fresh ``status.json`` endpoint
whose ``latest_run_id`` and SHA match the latest successful ``main`` dashboard
workflow run, and reachable ``data.json`` history endpoint, dashboard HTML
page, and ``summary.md`` page that match the generated payload exactly. It is
expected to fail before the first approved workflow publication because the
workflow, guide, latest run, and ``gh-pages/performance/`` are not all live yet.
