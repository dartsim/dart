"""Tests for scripts/check_architecture_map_runtime.py (PLAN-130 WP-130.7)."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_architecture_map_runtime.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_architecture_map_runtime", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


camr = _load_module()


def _step_view() -> dict:
    return {
        "meta": {
            "views": [
                {
                    "id": "fused-multibody",
                    "focus": ["sync", "rigid_body_velocity", "kinematics"],
                }
            ]
        },
        "nodes": [
            {"id": "sync"},
            {"id": "rigid_body_velocity"},
            {"id": "kinematics"},
        ],
    }


def _compute_view() -> dict:
    return {
        "components": [
            {
                "id": "graph",
                "label": "Semantic compute graph",
                "sublabel": "ComputeGraph · edges",
            },
        ],
        "cards": [
            {"title": "What runs today", "items": ["Kinematics caches its graph"]}
        ],
    }


def _fixture() -> dict:
    return {
        "schema_version": 1,
        "guided_view": "fused-multibody",
        "stages": ["rigid_body_velocity", "kinematics"],
        "graphs": [
            {
                "nodes": ["kinematics:frames", "kinematics:shapes"],
                "edges": [["kinematics:frames", "kinematics:shapes"]],
            }
        ],
        "graph_vocabulary": ["kinematics"],
    }


@pytest.fixture
def views(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    step = tmp_path / "world-step.dataflow.json"
    compute = tmp_path / "compute-graph.architecture.json"
    step.write_text(json.dumps(_step_view()), encoding="utf-8")
    compute.write_text(json.dumps(_compute_view()), encoding="utf-8")
    monkeypatch.setattr(camr, "STEP_VIEW", step)
    monkeypatch.setattr(camr, "COMPUTE_VIEW", compute)
    return tmp_path


def test_fixture_agrees_with_views(views: Path) -> None:
    findings = camr.check_against_views(_fixture(), _step_view(), _compute_view())
    assert findings == []


def test_unknown_stage_and_focus_and_vocabulary_are_reported() -> None:
    fixture = _fixture()
    fixture["stages"].append("ghost_stage")
    fixture["graphs"].append({"nodes": ["mystery:node"], "edges": []})
    findings = camr.check_against_views(fixture, _step_view(), _compute_view())
    assert any("recorded stage `ghost_stage` has no node" in f for f in findings)
    assert any(
        "does not focus recorded stage(s) ['ghost_stage']" in f for f in findings
    )
    assert any("compute node `mystery:node` is not named" in f for f in findings)


def test_guided_view_order_must_match_recorded_order() -> None:
    fixture = _fixture()
    step = _step_view()
    step["meta"]["views"][0]["focus"] = ["kinematics", "rigid_body_velocity", "sync"]
    findings = camr.check_against_views(fixture, step, _compute_view())
    assert any("lists stages in the order" in f for f in findings)


def test_failing_probe_binary_is_a_finding_and_strict_fails(
    views: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    fixture_path = views / "compute-graph.runtime.json"
    fixture_path.write_text(json.dumps(_fixture()), encoding="utf-8")
    monkeypatch.setattr(camr, "find_probe_binary", lambda: Path("/nonexistent/probe"))
    monkeypatch.setattr(camr, "run_probe", lambda binary: None)
    assert camr.main(["--fixture", str(fixture_path)]) == 0
    assert "probe binary failed" in capsys.readouterr().out
    assert camr.main(["--fixture", str(fixture_path), "--strict"]) == 1
    assert (
        camr.main(
            ["--fixture", str(fixture_path), "--probe-binary", str(views / "missing")]
        )
        == 1
    )


def test_missing_guided_view_is_reported() -> None:
    fixture = _fixture()
    fixture["guided_view"] = "nope"
    findings = camr.check_against_views(fixture, _step_view(), _compute_view())
    assert any("guided view `nope`" in f for f in findings)


def test_compare_dumps_detects_drift() -> None:
    fixture = _fixture()
    fresh = json.loads(json.dumps(fixture))
    assert camr.compare_dumps(fixture, fresh) == []
    fresh["stages"] = ["kinematics"]
    fresh["graphs"][0]["nodes"] = ["kinematics:frames"]
    findings = camr.compare_dumps(fixture, fresh)
    assert any("stage list drifted" in f for f in findings)
    assert any("node set drifted" in f for f in findings)
    fresh["graphs"].append({"nodes": [], "edges": []})
    assert any("graph count drifted" in f for f in camr.compare_dumps(fixture, fresh))


def test_main_advisory_and_strict_exit_codes(
    views: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    fixture_path = views / "compute-graph.runtime.json"
    fixture = _fixture()
    fixture["stages"].append("ghost_stage")
    fixture_path.write_text(json.dumps(fixture), encoding="utf-8")
    monkeypatch.setattr(camr, "find_probe_binary", lambda: None)
    assert camr.main(["--fixture", str(fixture_path)]) == 0
    out = capsys.readouterr().out
    assert "ADVISORY:" in out and "probe binary not built" in out
    assert camr.main(["--fixture", str(fixture_path), "--strict"]) == 1
    assert "ERROR:" in capsys.readouterr().out


def test_main_compares_probe_output_and_regenerates(
    views: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    fixture_path = views / "compute-graph.runtime.json"
    fixture_path.write_text(json.dumps(_fixture()), encoding="utf-8")
    probe = views / "probe.json"
    fresh = _fixture()
    fresh["scene"] = "reference"
    fresh["graphs"][0]["dot"] = "digraph ComputeGraph {}"
    probe.write_text(json.dumps(fresh), encoding="utf-8")
    assert (
        camr.main(["--fixture", str(fixture_path), "--probe-output", str(probe)]) == 0
    )
    assert "agrees with the views and the fresh probe" in capsys.readouterr().out

    fresh["stages"] = ["kinematics"]
    probe.write_text(json.dumps(fresh), encoding="utf-8")
    assert (
        camr.main(
            ["--fixture", str(fixture_path), "--probe-output", str(probe), "--strict"]
        )
        == 1
    )

    assert (
        camr.main(
            [
                "--fixture",
                str(fixture_path),
                "--probe-output",
                str(probe),
                "--regenerate",
            ]
        )
        == 0
    )
    regenerated = json.loads(fixture_path.read_text(encoding="utf-8"))
    assert regenerated["stages"] == ["kinematics"]
    assert regenerated["guided_view"] == "fused-multibody"
    assert regenerated["graph_vocabulary"] == ["kinematics"]
    assert "dot" not in regenerated["graphs"][0]
    assert "test_architecture_probe.cpp" in regenerated["source"]


def test_probe_candidates_cover_windows_executables() -> None:
    candidates = camr.PROBE_BINARY_CANDIDATES
    assert "build/default/cpp/Release/bin/test_architecture_probe" in candidates
    assert "build/default/cpp/Release/bin/test_architecture_probe.exe" in candidates
    assert "build/default/cpp/Debug/bin/test_architecture_probe.exe" in candidates


def test_regenerate_without_probe_fails(
    views: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(camr, "find_probe_binary", lambda: None)
    assert camr.main(["--fixture", str(views / "f.json"), "--regenerate"]) == 1


def test_real_fixture_agrees_with_real_views() -> None:
    if not camr.FIXTURE.is_file():
        pytest.skip("fixture not generated yet")
    findings = camr.check_against_views(
        camr.load_json(camr.FIXTURE),
        camr.load_json(camr.STEP_VIEW),
        camr.load_json(camr.COMPUTE_VIEW),
    )
    assert findings == [], "\n".join(findings)
