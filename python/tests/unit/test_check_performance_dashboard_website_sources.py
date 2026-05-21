import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_website_sources.py"


def _docs_root(tmp_path: Path, *, index: str | None = None, guide: str | None = None):
    docs_root = tmp_path / "readthedocs"
    (docs_root / "community").mkdir(parents=True)
    (docs_root / "index.rst").write_text(
        index
        or """
Project Stats
-------------

Benchmark history is covered by the
:doc:`DART Performance Dashboard <community/performance_dashboard>`.
Open https://dartsim.github.io/dart/performance/ after publication.
Check https://dartsim.github.io/dart/performance/status.json for health.

.. toctree::
   :maxdepth: 1

   community/performance_dashboard
""",
        encoding="utf-8",
    )
    (docs_root / "community" / "performance_dashboard.rst").write_text(
        guide
        or """
Performance Dashboard
=====================

https://dartsim.github.io/dart/performance/
https://dartsim.github.io/dart/performance/status.json
https://dartsim.github.io/dart/performance/data.json
summary.md
Quick Start
Open the dashboard page
Use ``status.json``
Download ``data.json``
Use ``summary.md``
Add competitor rows
Maintainers launch
publication_contract
schedule_crons
latest_run_id
latest successful ``main`` workflow run
feature-branch artifacts cannot seed the
expected update interval
stale-after window
refs/heads/main
actions: read
contents: write
pages: write
comparison_input_contract
comparison_metric_contract
External Competitor Status
trend_summary
testbed_summary
curl -fsSL https://dartsim.github.io/dart/performance/status.json
python -m json.tool
backend_ns / primary_ns
native_ns / best_reference_ns
ratio < 1
BM_Distance_BoxSphere_MuJoCo
BM_KR5ForwardDynamics_Drake/1
Supported external competitor suffixes
github-action-benchmark
Airspeed Velocity
LLVM LNT

.. code-block:: bash

   pixi run check-bm-dashboard-launch-preflight
   pixi run check-bm-dashboard-pages-branch
   pixi run check-bm-dashboard-pages-build
   pixi run check-bm-dashboard-workflow-run
   pixi run check-bm-dashboard-launch-live
   gh workflow run performance_dashboard.yml --ref main
""",
        encoding="utf-8",
    )
    return docs_root


def _run(docs_root: Path):
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--docs-root", str(docs_root)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def test_website_sources_accept_wired_dashboard_guide(tmp_path):
    result = _run(_docs_root(tmp_path))

    assert result.returncode == 0
    assert "website sources verified" in result.stdout


def test_website_sources_reject_missing_toctree_entry(tmp_path):
    docs_root = _docs_root(
        tmp_path,
        index="""
Project Stats
-------------

:doc:`DART Performance Dashboard <community/performance_dashboard>`.
https://dartsim.github.io/dart/performance/
https://dartsim.github.io/dart/performance/status.json
""",
    )

    result = _run(docs_root)

    assert result.returncode != 0
    assert "toctree" in result.stderr


def test_website_sources_reject_missing_direct_dashboard_url(tmp_path):
    docs_root = _docs_root(
        tmp_path,
        index="""
Project Stats
-------------

Benchmark history is covered by the
:doc:`DART Performance Dashboard <community/performance_dashboard>`.

.. toctree::
   :maxdepth: 1

   community/performance_dashboard
""",
    )

    result = _run(docs_root)

    assert result.returncode != 0
    assert "hosted dashboard URLs" in result.stderr


def test_website_sources_reject_missing_launch_guidance(tmp_path):
    docs_root = _docs_root(
        tmp_path,
        guide="""
Performance Dashboard
=====================

https://dartsim.github.io/dart/performance/
""",
    )

    result = _run(docs_root)

    assert result.returncode != 0
    assert "missing dashboard launch guidance" in result.stderr
