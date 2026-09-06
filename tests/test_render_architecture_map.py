"""Tests for scripts/render_architecture_map.py (PLAN-130 WP-130.1).

These tests never run Node.js or archify; they cover discovery, stamping,
post-processing, fallbacks, and the exit-code contract with archify mocked.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "render_architecture_map.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("render_architecture_map", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ram = _load_module()


def _architecture_ir(with_sources: bool = True) -> dict:
    component = {
        "id": "world",
        "type": "external",
        "label": "World facade",
        "sublabel": "dart::simulation::World",
        "tag": "Implemented",
        "pos": [40, 100],
        "size": [160, 64],
    }
    if with_sources:
        component["sources"] = [{"path": "dart/simulation/world.hpp", "line": 1}]
    return {
        "schema_version": 1,
        "diagram_type": "architecture",
        "meta": {"title": "Test <map>"},
        "components": [component],
        "connections": [],
    }


def _dataflow_ir() -> dict:
    return {
        "schema_version": 1,
        "diagram_type": "dataflow",
        "meta": {"title": "Step flow"},
        "stages": [{"label": "Velocity"}, {"label": "Position"}],
        "nodes": [
            {
                "id": "rigid_body_velocity",
                "type": "backend",
                "label": "Rigid velocity",
                "stage": 0,
                "row": 0,
                "tag": "Implemented",
            },
            {
                "id": "rigid_body_position",
                "type": "backend",
                "label": "Rigid position",
                "tag": "Implemented",
                "stage": 1,
                "row": 0,
            },
        ],
        "flows": [
            {
                "id": "velocity-position",
                "from": "rigid_body_velocity",
                "to": "rigid_body_position",
                "label": "velocities & impulses",
            }
        ],
    }


@pytest.fixture
def ir_dir(tmp_path: Path) -> Path:
    directory = tmp_path / "ir"
    directory.mkdir()
    (directory / "framework.architecture.json").write_text(
        json.dumps(_architecture_ir()), encoding="utf-8"
    )
    (directory / "step.dataflow.json").write_text(
        json.dumps(_dataflow_ir()), encoding="utf-8"
    )
    (directory / "compute-graph.runtime.json").write_text("{}", encoding="utf-8")
    (directory / "README.md").write_text("not a view", encoding="utf-8")
    return directory


def test_discover_views_recognizes_only_view_suffixes(ir_dir: Path) -> None:
    views = ram.discover_views(ir_dir)
    assert [(v.name, v.diagram_type) for v in views] == [
        ("framework", "architecture"),
        ("step", "dataflow"),
    ]


def test_discover_views_missing_dir_is_empty(tmp_path: Path) -> None:
    assert ram.discover_views(tmp_path / "nope") == []


def test_stamp_repository_adds_revision_without_mutating_input() -> None:
    ir = _architecture_ir()
    stamped = ram.stamp_repository(ir, "a" * 40)
    assert stamped["meta"]["repository"] == {
        "url": ram.DART_REPOSITORY_URL,
        "revision": "a" * 40,
    }
    assert "repository" not in ir["meta"]


def test_declares_sources() -> None:
    assert ram.declares_sources(_architecture_ir(with_sources=True))
    assert not ram.declares_sources(_architecture_ir(with_sources=False))


def test_strip_external_fonts_removes_google_font_links_only() -> None:
    text = (
        "<head>\n"
        '  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>\n'
        '  <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono"\n'
        '        rel="stylesheet" media="print" onload="this.media=\'all\'">\n'
        "  <noscript>\n"
        '    <link href="https://fonts.googleapis.com/css2?family=X" rel="stylesheet">\n'
        "  </noscript>\n"
        '  <link rel="icon" href="data:,">\n'
        "</head>"
    )
    stripped = ram.strip_external_fonts(text)
    assert "fonts.googleapis.com" not in stripped
    assert "fonts.gstatic.com" not in stripped
    assert "<noscript>" not in stripped
    assert '<link rel="icon" href="data:,">' in stripped


def test_prepend_source_comment_after_doctype() -> None:
    comment = "<!-- generated -->\n"
    text = "<!DOCTYPE html>\n<html></html>"
    result = ram.prepend_source_comment(text, comment)
    assert result.startswith("<!DOCTYPE html>\n<!-- generated -->\n<html>")
    assert ram.prepend_source_comment("<html></html>", comment).startswith(comment)


def test_source_comment_names_source_script_and_revision(ir_dir: Path) -> None:
    view = ram.discover_views(ir_dir)[0]
    comment = ram.source_comment(view, "b" * 40)
    assert ram.SCRIPT_RELPATH in comment
    assert view.path.name in comment
    assert "b" * 40 in comment
    assert ram.ARCHIFY_TAG in comment


def test_fallback_html_renders_dataflow_and_escapes(ir_dir: Path) -> None:
    views = {v.name: v for v in ram.discover_views(ir_dir)}
    text = ram.fallback_html(views["step"], "Node.js missing.", None)
    assert "<title>Step flow</title>" in text
    assert "<h2>Velocity</h2>" in text and "<h2>Position</h2>" in text
    assert "Rigid velocity" in text and "[Implemented]" in text
    assert "velocities &amp; impulses" in text
    arch = ram.fallback_html(views["framework"], "Node.js missing.", None)
    assert "Test &lt;map&gt;" in arch
    assert "World facade" in arch and "dart::simulation::World" in arch


def test_main_writes_fallbacks_when_node_missing(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(ram, "node_executable", lambda *a, **k: None)
    out = tmp_path / "out"
    code = ram.main(["--ir-dir", str(ir_dir), "--output-dir", str(out), "--no-fetch"])
    assert code == ram.EXIT_UNAVAILABLE
    rendered = sorted(p.name for p in out.iterdir())
    assert rendered == ["framework.html", "framework.md", "step.html", "step.md"]
    text = (out / "framework.html").read_text(encoding="utf-8")
    assert "Text rendering" in text
    assert ram.SCRIPT_RELPATH in text


def test_main_strict_and_check_fail_without_toolchain(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(ram, "node_executable", lambda *a, **k: None)
    out = tmp_path / "out"
    assert (
        ram.main(["--ir-dir", str(ir_dir), "--output-dir", str(out), "--strict"])
        == ram.EXIT_FAILED
    )
    assert (
        ram.main(["--ir-dir", str(ir_dir), "--output-dir", str(out), "--check"])
        == ram.EXIT_FAILED
    )
    assert not out.exists()


def test_malformed_view_fails_even_without_toolchain(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (ir_dir / "broken.architecture.json").write_text("{not json", encoding="utf-8")
    monkeypatch.setattr(ram, "node_executable", lambda *a, **k: None)
    out = tmp_path / "out"
    code = ram.main(["--ir-dir", str(ir_dir), "--output-dir", str(out), "--no-fetch"])
    assert code == ram.EXIT_FAILED
    assert not out.exists()


def test_view_shape_error_reports_type_title_and_nodes(tmp_path: Path) -> None:
    path = tmp_path / "x.dataflow.json"
    view = ram.View(path=path, name="x", diagram_type="dataflow")
    path.write_text(json.dumps({"diagram_type": "architecture"}), encoding="utf-8")
    assert "does not match" in ram.view_shape_error(view)
    path.write_text(
        json.dumps({"diagram_type": "dataflow", "meta": {}}), encoding="utf-8"
    )
    assert "meta.title" in ram.view_shape_error(view)
    path.write_text(
        json.dumps({"diagram_type": "dataflow", "meta": {"title": "t"}, "nodes": []}),
        encoding="utf-8",
    )
    assert "`nodes`" in ram.view_shape_error(view)
    path.write_text(json.dumps(_dataflow_ir()), encoding="utf-8")
    assert ram.view_shape_error(view) is None


def test_dangling_flow_fails_even_without_toolchain(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ir = _dataflow_ir()
    ir["flows"][0]["to"] = "ghost"
    (ir_dir / "step.dataflow.json").write_text(json.dumps(ir), encoding="utf-8")
    monkeypatch.setattr(ram, "node_executable", lambda *a, **k: None)
    out = tmp_path / "out"
    code = ram.main(["--ir-dir", str(ir_dir), "--output-dir", str(out), "--no-fetch"])
    assert code == ram.EXIT_FAILED
    assert not out.exists()


def test_text_summaries_mark_planned_relationships(ir_dir: Path) -> None:
    view = next(
        v for v in ram.discover_views(ir_dir) if v.diagram_type == "architecture"
    )
    ir = json.loads(view.path.read_text(encoding="utf-8"))
    ids = [c["id"] for c in ir["components"]]
    ir.setdefault("connections", []).append(
        {
            "id": "p",
            "from": ids[0],
            "to": ids[-1],
            "label": "later",
            "variant": "dashed",
        }
    )
    view.path.write_text(json.dumps(ir), encoding="utf-8")
    assert "later (planned)" in ram.view_summary_markdown(view)
    assert "later (planned)" in ram.fallback_html(view, "no node", None)


def test_summaries_and_fallback_cite_sources(ir_dir: Path) -> None:
    view = next(
        v for v in ram.discover_views(ir_dir) if v.diagram_type == "architecture"
    )
    ir = json.loads(view.path.read_text(encoding="utf-8"))
    ir["components"][0]["sources"] = [
        {"path": "dart/simulation/world.hpp", "line": 3, "end_line": 5},
        {"path": "dart/simulation/world_options.hpp"},
    ]
    view.path.write_text(json.dumps(ir), encoding="utf-8")
    markdown = ram.view_summary_markdown(view)
    assert "`dart/simulation/world.hpp:3-5`" in markdown
    assert "`dart/simulation/world_options.hpp`" in markdown
    page = ram.fallback_html(view, "no node", None)
    assert "<code>dart/simulation/world.hpp:3-5</code>" in page


def test_summaries_and_fallback_include_cards(ir_dir: Path) -> None:
    view = next(
        v for v in ram.discover_views(ir_dir) if v.diagram_type == "architecture"
    )
    ir = json.loads(view.path.read_text(encoding="utf-8"))
    ir["cards"] = [{"title": "Public selectors", "items": ["RigidBodySolver: Ipc"]}]
    view.path.write_text(json.dumps(ir), encoding="utf-8")
    markdown = ram.view_summary_markdown(view)
    assert "*Public selectors*" in markdown and "- RigidBodySolver: Ipc" in markdown
    page = ram.fallback_html(view, "no node", None)
    assert "<h2>Public selectors</h2>" in page and "RigidBodySolver: Ipc" in page


def test_main_fails_when_a_required_view_is_missing_from_the_default_dir(
    ir_dir: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(ram, "DEFAULT_IR_DIR", ir_dir)
    assert ram.main(["--output-dir", str(tmp_path / "out")]) == ram.EXIT_FAILED
    captured = capsys.readouterr()
    assert (
        "required view `simulation-framework` is missing" in captured.out + captured.err
    )


def test_text_summaries_are_written_for_non_html_builders(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(ram, "node_executable", lambda *a, **k: None)
    out = tmp_path / "out"
    ram.main(["--ir-dir", str(ir_dir), "--output-dir", str(out), "--no-fetch"])
    summary = (out / "step.md").read_text(encoding="utf-8")
    assert summary.startswith("**Step flow**")
    assert "*Velocity*" in summary and "**Rigid velocity**" in summary
    assert "Rigid velocity → Rigid position: velocities & impulses" in summary
    framework = (out / "framework.md").read_text(encoding="utf-8")
    assert (
        "**World facade** (external, Implemented): dart::simulation::World" in framework
    )


def test_duplicate_view_names_fail_before_writing(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (ir_dir / "framework.dataflow.json").write_text(
        json.dumps(_dataflow_ir()), encoding="utf-8"
    )
    monkeypatch.setattr(ram, "node_executable", lambda *a, **k: None)
    out = tmp_path / "out"
    code = ram.main(["--ir-dir", str(ir_dir), "--output-dir", str(out), "--no-fetch"])
    assert code == ram.EXIT_FAILED
    assert not out.exists()


def test_main_without_views_is_a_noop(tmp_path: Path) -> None:
    empty = tmp_path / "empty"
    empty.mkdir()
    assert ram.main(["--ir-dir", str(empty), "--output-dir", str(tmp_path / "o")]) == 0


def test_main_unknown_view_fails(ir_dir: Path, tmp_path: Path) -> None:
    code = ram.main(
        [
            "--ir-dir",
            str(ir_dir),
            "--output-dir",
            str(tmp_path / "o"),
            "--views",
            "ghost",
        ]
    )
    assert code == ram.EXIT_FAILED


def test_view_shape_error_rejects_scalar_entries(ir_dir: Path) -> None:
    view = ram.discover_views(ir_dir)[0]
    ir = json.loads(view.path.read_text(encoding="utf-8"))
    key = "components" if view.diagram_type == "architecture" else "nodes"
    ir[key].append("not-a-node")
    view.path.write_text(json.dumps(ir), encoding="utf-8")
    message = ram.view_shape_error(ram.discover_views(ir_dir)[0])
    assert (
        message is not None and f"`{key}[" in message and "must be an object" in message
    )


def test_ensure_archify_no_fetch_without_checkout(tmp_path: Path) -> None:
    assert ram.ensure_archify(tmp_path / "missing", fetch=False) is None


def test_ensure_archify_refuses_to_replace_a_foreign_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    foreign = tmp_path / "archify"
    foreign.mkdir()
    (foreign / "my-work.txt").write_text("keep me", encoding="utf-8")
    monkeypatch.setattr(ram, "_git_head", lambda path: "0" * 40)

    def fail_clone(*args, **kwargs):  # pragma: no cover - must not be reached
        raise AssertionError("clone attempted on a foreign directory")

    monkeypatch.setattr(ram.subprocess, "run", fail_clone)
    assert ram.ensure_archify(foreign) is None
    assert (foreign / "my-work.txt").read_text(encoding="utf-8") == "keep me"
    captured = capsys.readouterr()
    assert ram.CACHE_MARKER in captured.out + captured.err


def test_ensure_archify_rejects_a_modified_pinned_checkout(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cache = tmp_path / "archify"
    cli = cache / ram.ARCHIFY_CLI
    cli.parent.mkdir(parents=True)
    cli.write_text("tampered", encoding="utf-8")
    monkeypatch.setattr(ram, "_git_head", lambda path: ram.ARCHIFY_COMMIT)
    monkeypatch.setattr(ram, "_worktree_clean", lambda path: False)
    monkeypatch.setattr(ram.subprocess, "run", lambda *a, **k: None)
    # Not created by this script: refused, left untouched.
    assert ram.ensure_archify(cache) is None
    assert cli.read_text(encoding="utf-8") == "tampered"
    # Created by this script: replaced by a fresh clone.
    (cache / ram.CACHE_MARKER).write_text("ours", encoding="utf-8")

    def fake_clone(args, **kwargs):
        target = Path(args[-1]) / ram.ARCHIFY_CLI
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("pristine", encoding="utf-8")

    monkeypatch.setattr(ram.subprocess, "run", fake_clone)
    assert ram.ensure_archify(cache) == cache
    assert cli.read_text(encoding="utf-8") == "pristine"


def test_ensure_archify_replaces_only_its_own_stale_cache(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cache = tmp_path / "archify"
    cache.mkdir()
    (cache / ram.CACHE_MARKER).write_text("stale", encoding="utf-8")
    (cache / "stale.txt").write_text("old", encoding="utf-8")
    monkeypatch.setattr(ram, "_git_head", lambda path: ram.ARCHIFY_COMMIT)
    monkeypatch.setattr(ram, "_worktree_clean", lambda path: True)

    def fake_clone(args, **kwargs):
        target = Path(args[-1])
        cli = target / ram.ARCHIFY_CLI
        cli.parent.mkdir(parents=True, exist_ok=True)
        cli.write_text("", encoding="utf-8")

    monkeypatch.setattr(ram.subprocess, "run", fake_clone)
    assert ram.ensure_archify(cache) == cache
    assert not (cache / "stale.txt").exists()
    assert ram.ARCHIFY_COMMIT in (cache / ram.CACHE_MARKER).read_text(encoding="utf-8")


def test_render_view_reports_validation_diagnostics(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    view = ram.discover_views(ir_dir)[0]
    calls: list[list[str]] = []

    def fake_run(node, archify_dir, args, cwd):
        calls.append(args)
        return 1, {
            "ok": False,
            "diagnostics": [
                {
                    "code": "repository-evidence/file-missing",
                    "severity": "error",
                    "message": "nope",
                    "supportedFixes": ["fix the path"],
                }
            ],
        }

    monkeypatch.setattr(ram, "run_archify", fake_run)
    ok, message = ram.render_view(
        view,
        node=Path("node"),
        archify_dir=tmp_path,
        output_dir=tmp_path / "out",
        revision="c" * 40,
    )
    assert not ok
    assert "repository-evidence/file-missing" in message
    assert "fix the path" in message
    assert calls and calls[0][0] == "validate"
    assert "--repo-root" in calls[0]


def test_render_view_requires_revision_for_source_evidence(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    view = ram.discover_views(ir_dir)[0]
    monkeypatch.setattr(ram, "run_archify", lambda *a, **k: (0, {"ok": True}))
    ok, message = ram.render_view(
        view,
        node=Path("node"),
        archify_dir=tmp_path,
        output_dir=tmp_path,
        revision=None,
    )
    assert not ok
    assert "revision is unknown" in message


def test_render_view_success_writes_post_processed_html(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    view = ram.discover_views(ir_dir)[0]
    seen: list[list[str]] = []

    def fake_run(node, archify_dir, args, cwd):
        seen.append(args)
        if args[0] == "deliver":
            output = Path(args[3])
            output.write_text(
                "<!DOCTYPE html>\n<html><head>"
                '<link href="https://fonts.googleapis.com/css2?family=X" rel="stylesheet">'
                "</head><body>ok</body></html>",
                encoding="utf-8",
            )
        return 0, {"ok": True}

    monkeypatch.setattr(ram, "run_archify", fake_run)
    out = tmp_path / "out"
    ok, message = ram.render_view(
        view,
        node=Path("node"),
        archify_dir=tmp_path,
        output_dir=out,
        revision="d" * 40,
    )
    assert ok, message
    text = (out / "framework.html").read_text(encoding="utf-8")
    assert text.startswith(
        "<!DOCTYPE html>\n<!-- Generated by scripts/render_architecture_map.py"
    )
    assert "fonts.googleapis.com" not in text
    assert "<body>ok</body>" in text
    assert [args[0] for args in seen] == ["validate", "deliver"]
    # The stamped candidate, not the tracked file, is what archify sees.
    candidate = Path(seen[0][2])
    assert candidate.name == view.path.name and candidate != view.path


def test_render_view_dataflow_skips_repo_root(
    ir_dir: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    view = [v for v in ram.discover_views(ir_dir) if v.diagram_type == "dataflow"][0]
    seen: list[list[str]] = []

    def fake_run(node, archify_dir, args, cwd):
        seen.append(args)
        return 0, {"ok": True}

    monkeypatch.setattr(ram, "run_archify", fake_run)
    ok, _ = ram.render_view(
        view,
        node=Path("node"),
        archify_dir=tmp_path,
        output_dir=tmp_path / "out",
        revision="e" * 40,
        check_only=True,
    )
    assert ok
    assert len(seen) == 1 and "--repo-root" not in seen[0]
