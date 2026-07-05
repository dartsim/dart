import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "generate_community_signals_dashboard.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "generate_community_signals_dashboard", SCRIPT
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _minimal_dashboard_data():
    return {
        "generated_at": "2026-07-05T20:00:00Z",
        "summary": {
            "latest_release": "v6.19.3",
            "latest_release_date": "2026-06-27",
            "latest_push": "2026-07-05",
            "stars": 1161,
            "forks": 301,
            "watchers": 50,
            "created_prs_12m": 1277,
            "merged_prs_12m": 1196,
            "open_issues": 1,
            "open_prs": 4,
            "contributors_login": 54,
            "contributors_with_anon": 78,
        },
        "traffic": {
            "available": False,
            "unique_cloners": None,
            "clones": None,
            "clone_window": None,
        },
        "packages": {
            "conda": {
                "dartsim": 641882,
                "dartpy": None,
                "dartsim-cpp": 124233,
                "combined_gross": None,
            },
            "pepy_dartpy_all_time": 258524,
            "homebrew_installs_365d": 18009,
        },
        "citations": {
            "indexed_min": 189,
            "indexed_max": 306,
            "google_scholar": 395,
            "google_scholar_checked": "2026-07-05",
        },
        "gazebo": {
            "docs_default_dart": True,
            "package_depends_dart": True,
            "requires_dart_610": True,
            "dartsim_plugin_files": 41,
        },
        "sources": {
            "github_repo": "#repo",
            "github_created_prs_12m": "#created",
            "github_merged_prs_12m": "#merged",
            "github_issues": "#issues",
            "github_prs": "#prs",
            "github_traffic_docs": "#traffic",
            "anaconda_dartsim": "#conda",
            "pepy_dartpy": "#pepy",
            "homebrew_dartsim": "#brew",
            "google_scholar": "#scholar",
            "crossref": "#crossref",
            "openalex": "#openalex",
            "semantic_scholar": "#semantic",
            "gazebo_physics": "#gazebo",
            "gz_physics_package": "#package",
            "gz_physics_cmake": "#cmake",
            "gz_physics_tree": "#tree",
        },
        "downstream": [],
        "errors": [],
    }


def test_complete_sum_requires_every_value():
    module = _load_module()

    assert module._complete_sum({"a": 10, "b": 20, "c": 30}) == 60
    assert module._complete_sum({"a": 10, "b": None, "c": 30}) is None


def test_partial_conda_total_renders_unavailable_not_partial_zero():
    module = _load_module()

    html = module.render_html(_minimal_dashboard_data())

    assert "gross conda artifact total unavailable" in html
    assert "0 gross conda artifacts" not in html
    assert "unavailable gross conda" not in html
    assert "642K conda dartsim downloads" in html
