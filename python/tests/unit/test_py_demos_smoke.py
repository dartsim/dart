"""Unit coverage for the full-catalog py-demos smoke harness."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path
from types import ModuleType

ROOT = Path(__file__).resolve().parents[3]
SMOKE_SCRIPT = ROOT / "scripts" / "py_demos_smoke.py"


def _load_smoke_module() -> ModuleType:
    spec = importlib.util.spec_from_file_location("py_demos_smoke", SMOKE_SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register before exec so dataclasses can resolve string annotations
    # (``from __future__ import annotations``) via ``sys.modules[__module__]``
    # under Python 3.14; otherwise SceneResult's @dataclass raises AttributeError.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_select_scene_ids_preserves_registry_order_and_rejects_unknown() -> None:
    module = _load_smoke_module()

    scene_ids = ["rigid_body", "contact", "vbd_cloth"]

    assert module._select_scene_ids(scene_ids, None) == scene_ids
    assert module._select_scene_ids(scene_ids, ["vbd_cloth", "rigid_body"]) == [
        "rigid_body",
        "vbd_cloth",
    ]

    try:
        module._select_scene_ids(scene_ids, ["missing_scene"])
    except ValueError as exc:
        assert "missing_scene" in str(exc)
    else:  # pragma: no cover - assertion path
        raise AssertionError("unknown --only scene should fail")


def test_main_rejects_unknown_only_without_running_workers(monkeypatch, capsys) -> None:
    module = _load_smoke_module()
    worker_calls: list[str] = []

    monkeypatch.setattr(module, "_list_scene_ids", lambda: ["rigid_body"])
    monkeypatch.setattr(
        module,
        "_run_worker",
        lambda scene_id, *_args: worker_calls.append(scene_id),
    )

    rc = module.main(["--only", "missing_scene"])

    assert rc == 2
    assert worker_calls == []
    assert "unknown scene id(s) for --only: missing_scene" in capsys.readouterr().err


def test_main_runs_selected_scenes_in_registry_order_and_writes_json(
    monkeypatch, tmp_path
) -> None:
    module = _load_smoke_module()
    calls: list[tuple[str, int, bool | None, float]] = []

    monkeypatch.setattr(
        module,
        "_list_scene_ids",
        lambda: ["rigid_body", "contact", "vbd_cloth"],
    )

    def fake_run_worker(
        scene_id: str, frames: int, gpu_pref: bool | None, timeout: float
    ):
        calls.append((scene_id, frames, gpu_pref, timeout))
        status = "fail" if scene_id == "contact" else "ok"
        return module.SceneResult(scene_id, status, 0.25, "scripted failure")

    monkeypatch.setattr(module, "_run_worker", fake_run_worker)
    json_out = tmp_path / "smoke.json"

    rc = module.main(
        [
            "--frames",
            "7",
            "--timeout",
            "3",
            "--gpu",
            "--only",
            "contact",
            "rigid_body",
            "--json-out",
            str(json_out),
        ]
    )

    assert rc == 1
    assert calls == [
        ("rigid_body", 7, True, 3.0),
        ("contact", 7, True, 3.0),
    ]
    assert '"scene_id": "contact"' in json_out.read_text(encoding="utf-8")
    assert '"status": "fail"' in json_out.read_text(encoding="utf-8")


def test_run_worker_classifies_signal_exit_as_crash(monkeypatch) -> None:
    module = _load_smoke_module()

    def fake_run(*_args, **_kwargs):
        return subprocess.CompletedProcess(
            args=["python", "scripts/py_demos_smoke.py"],
            returncode=-11,
            stdout="line 1\nsegmentation fault",
        )

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    result = module._run_worker("rigid_body", frames=2, gpu_pref=None, timeout=5.0)

    assert result.status == "crash"
    assert result.scene_id == "rigid_body"
    assert "segmentation fault" in result.detail


def test_run_worker_reports_timeout(monkeypatch) -> None:
    module = _load_smoke_module()

    def fake_run(*_args, **_kwargs):
        raise subprocess.TimeoutExpired(cmd=["python"], timeout=2.0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    result = module._run_worker("rigid_body", frames=2, gpu_pref=False, timeout=2.0)

    assert result.status == "timeout"
    assert result.scene_id == "rigid_body"
    assert "exceeded 2s" in result.detail
