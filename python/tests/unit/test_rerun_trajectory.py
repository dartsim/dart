"""Unit coverage for optional rerun trajectory logging."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import rerun_trajectory


def _tiny_trajectory_tsv() -> str:
    columns = (
        "# columns: frame time body pos_x pos_y pos_z lin_x lin_y lin_z "
        "ang_x ang_y ang_z contact_count"
    )
    row = (
        "1\t1.00000000000000002e-02\tbox\t0.0\t0.0\t1.0\t0.0\t0.0\t0.0"
        "\t0.0\t0.0\t0.0\t0"
    )
    return "\n".join(
        [
            "# dart.trajectory/v0 scene=tiny",
            "# gravity: [0.0,0.0,-9.81]",
            "# time_step: 1.00000000000000002e-02",
            "# rest_tolerance: 0.00000000000000000e+00",
            (
                '# units: {"angle":"radian","length":"meter",'
                '"mass":"kilogram","time":"second"}'
            ),
            columns,
            row,
        ]
    ) + "\n"


def test_no_rerun_fallback_is_clean(monkeypatch, capsys, tmp_path) -> None:
    real_import_module = importlib.import_module

    def fake_import_module(name: str, package: str | None = None):
        if name == "rerun":
            raise ImportError("simulated missing rerun")
        return real_import_module(name, package)

    monkeypatch.setattr(rerun_trajectory.importlib, "import_module", fake_import_module)
    trajectory_path = tmp_path / "tiny.tsv"
    trajectory_path.write_text(_tiny_trajectory_tsv(), encoding="utf-8")
    output_path = tmp_path / "tiny.rrd"

    exit_code = rerun_trajectory.main(
        ["--trajectory-tsv", str(trajectory_path), "--save", str(output_path)]
    )

    captured = capsys.readouterr()
    expected = (
        "rerun-sdk not installed; pip install rerun-sdk "
        "or add it to the dev environment"
    )
    assert exit_code != 0
    assert expected in captured.err
    assert "Traceback" not in captured.err
    assert "Traceback" not in captured.out
    assert not output_path.exists()


def test_save_rrd_smoke_when_rerun_available(tmp_path) -> None:
    pytest.importorskip("rerun")
    trajectory_path = tmp_path / "tiny.tsv"
    trajectory_path.write_text(_tiny_trajectory_tsv(), encoding="utf-8")
    output_path = tmp_path / "tiny.rrd"

    exit_code = rerun_trajectory.main(
        [
            "--trajectory-tsv",
            str(trajectory_path),
            "--save",
            str(output_path),
            "--application-id",
            "dart.trajectory.test",
        ]
    )

    assert exit_code == 0
    assert output_path.exists()
    assert output_path.stat().st_size > 0


def test_contacts_without_files_record_from_one_runner(monkeypatch) -> None:
    calls: list[tuple[str, object]] = []
    runner = object()

    class Args:
        trajectory_tsv = None
        contact_jsonl = None
        contacts = True
        steps = 4
        scene = "demo"
        factory = None
        body = ["box"]

    def fake_resolve_world_runner(*, scene, factory):
        assert scene == "demo"
        assert factory is None
        calls.append(("resolve", runner))
        return runner

    def fake_record_trajectory_and_contact_events(active_runner, steps, bodies):
        assert active_runner is runner
        assert steps == 4
        assert bodies == ["box"]
        calls.append(("combined", active_runner))
        return _tiny_trajectory_tsv(), "{}\n"

    monkeypatch.setattr(
        rerun_trajectory.trajectory_record,
        "resolve_world_runner",
        fake_resolve_world_runner,
    )
    monkeypatch.setattr(
        rerun_trajectory.trajectory_record,
        "record_trajectory_and_contact_events",
        fake_record_trajectory_and_contact_events,
    )

    trajectory_text, contact_text = rerun_trajectory._load_recording_texts(Args())

    assert trajectory_text == _tiny_trajectory_tsv()
    assert contact_text == "{}\n"
    assert calls == [("resolve", runner), ("combined", runner)]
