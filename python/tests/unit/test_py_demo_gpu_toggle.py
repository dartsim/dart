"""Unit tests for the py-demos GPU (CUDA) deformable-solve toggle.

These exercise the runner's preference resolution, flag stripping, panel
injection, and the dartpy bindings' safe no-op behavior on a non-CUDA build.
They never require a CUDA device or a CUDA-enabled build; the GPU path itself is
covered by `pixi run -e cuda py-demos`.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

import dartpy as dart
import pytest
from examples.demos.runner import (
    CAPTURE_METRICS_EVENT_NAME,
    CAPTURE_METRICS_INFO_KEY,
    PythonDemoScene,
    SceneSetup,
    _configure_gpu_compute,
    _gpu_preference,
    _make_gpu_panel,
    _make_world_factory,
    _strip_gpu_flags,
    _strip_runner_local_flags,
)


class _FakeBuilder:
    """Minimal PanelBuilder stand-in with a scripted checkbox result."""

    def __init__(self, checkbox_result: tuple[bool, bool] = (False, False)) -> None:
        self.texts: list[str] = []
        self.checkbox_calls: list[tuple[str, bool]] = []
        self._checkbox_result = checkbox_result

    def text(self, value: str) -> None:
        self.texts.append(value)

    def separator(self) -> None:
        self.texts.append("---")

    def checkbox(self, label: str, value: bool) -> tuple[bool, bool]:
        self.checkbox_calls.append((label, value))
        return self._checkbox_result


class _FakeSx:
    """Stand-in for dartpy's GPU control functions."""

    def __init__(self, available: bool, enabled: bool = False) -> None:
        self._available = available
        self._enabled = enabled
        self.set_calls: list[bool] = []

    def is_accelerated_deformable_solve_available(self) -> bool:
        return self._available

    def set_accelerated_deformable_solve(self, enable: bool) -> bool:
        self.set_calls.append(bool(enable))
        # Mirror the real binding: only enables when a device is available.
        self._enabled = bool(enable) and self._available
        return self._enabled

    def is_accelerated_deformable_solve_enabled(self) -> bool:
        return self._enabled


def test_gpu_preference_cli_overrides_env(monkeypatch):
    monkeypatch.setenv("DART_PY_DEMOS_GPU", "off")
    assert _gpu_preference(True) == "on"
    assert _gpu_preference(False) == "off"


def test_gpu_preference_env_and_default(monkeypatch):
    monkeypatch.setenv("DART_PY_DEMOS_GPU", "on")
    assert _gpu_preference(None) == "on"
    monkeypatch.setenv("DART_PY_DEMOS_GPU", "OFF")
    assert _gpu_preference(None) == "off"
    monkeypatch.delenv("DART_PY_DEMOS_GPU", raising=False)
    assert _gpu_preference(None) == "auto"


def test_strip_gpu_flags():
    assert _strip_gpu_flags(["--scene", "x", "--gpu", "--frames", "4"]) == [
        "--scene",
        "x",
        "--frames",
        "4",
    ]
    assert _strip_gpu_flags(["--no-gpu"]) == []
    assert _strip_gpu_flags(["--scene", "y"]) == ["--scene", "y"]


def test_strip_runner_local_flags():
    assert _strip_runner_local_flags(
        [
            "--scene",
            "x",
            "--gpu",
            "--capture-metrics-event-log",
            "/tmp/metrics.jsonl",
            "--frames",
            "4",
        ]
    ) == [
        "--scene",
        "x",
        "--frames",
        "4",
    ]
    assert _strip_runner_local_flags(
        ["--capture-metrics-event-log=/tmp/metrics.jsonl", "--no-gpu"]
    ) == []


def test_configure_gpu_compute_cpu_build_is_noop(monkeypatch, capsys):
    monkeypatch.delenv("DART_PY_DEMOS_GPU", raising=False)
    sx = _FakeSx(available=False)
    panel = _configure_gpu_compute(sx, None)
    assert panel is None
    # Auto on a CPU build never enables the GPU backend.
    assert all(call is False for call in sx.set_calls)
    assert "CPU" in capsys.readouterr().out


def test_configure_gpu_compute_available_auto_enables(monkeypatch):
    monkeypatch.delenv("DART_PY_DEMOS_GPU", raising=False)
    sx = _FakeSx(available=True)
    panel = _configure_gpu_compute(sx, None)
    assert panel is not None
    assert sx.is_accelerated_deformable_solve_enabled() is True


def test_configure_gpu_compute_no_binding_returns_none():
    fake_dart = SimpleNamespace()
    assert _configure_gpu_compute(fake_dart, None) is None


def test_gpu_panel_checkbox_toggles_backend():
    sx = _FakeSx(available=True, enabled=False)
    panel = _make_gpu_panel(sx)
    panel.build(_FakeBuilder(checkbox_result=(True, True)), None)
    assert sx.set_calls == [True]


def test_world_factory_injects_gpu_panel():
    sx = _FakeSx(available=True)
    gpu_panel = _make_gpu_panel(sx)
    sentinel_world = object()
    scene = PythonDemoScene(
        id="t",
        title="T",
        category="C",
        summary="s",
        build=lambda: SceneSetup(world=sentinel_world),
    )
    result = _make_world_factory(scene, gpu_panel)()
    # The viewer factory returns (pre_step, force_drag, panels, provider).
    assert isinstance(result, tuple) and len(result) == 4
    assert result[0] is None
    assert result[1] is None
    assert result[2] == [gpu_panel]
    assert result[3] is None


def test_world_factory_without_gpu_panel_is_unchanged():
    sentinel_world = object()
    scene = PythonDemoScene(
        id="t",
        title="T",
        category="C",
        summary="s",
        build=lambda: SceneSetup(world=sentinel_world),
    )
    # No callbacks, no panels, no renderable provider.
    assert _make_world_factory(scene)() == (None, None, None, None)


def test_world_factory_records_capture_metrics(tmp_path):
    state = {"frame": 0}

    def pre_step() -> None:
        state["frame"] += 1

    def capture_metrics() -> dict[str, object]:
        return {
            "frame": state["frame"],
            "infinite": float("inf"),
            "tuple": (1, 2.5),
        }

    scene = PythonDemoScene(
        id="metrics_scene",
        title="Metrics",
        category="C",
        summary="s",
        build=lambda: SceneSetup(
            pre_step=pre_step,
            info={CAPTURE_METRICS_INFO_KEY: capture_metrics},
        ),
    )
    log = tmp_path / "scene_metrics.jsonl"
    factory_pre_step, _force_drag, _panels, _provider = _make_world_factory(
        scene,
        capture_metrics_event_log=str(log),
    )()

    assert factory_pre_step is not None
    factory_pre_step()
    factory_pre_step()

    events = [json.loads(line) for line in log.read_text().splitlines()]
    assert len(events) == 2
    assert events[-1]["event"] == CAPTURE_METRICS_EVENT_NAME
    assert events[-1]["frame"] == 2
    assert events[-1]["scene"] == "metrics_scene"
    assert events[-1]["source"] == "py-demo-scene"
    assert events[-1]["metrics"] == {
        "frame": 2,
        "infinite": None,
        "tuple": [1, 2.5],
    }


def test_dartpy_gpu_bindings_are_safe_noops_without_cuda():
    sx = dart
    if not hasattr(sx, "set_accelerated_deformable_solve"):
        pytest.skip("simulation bindings are disabled in this build")
    if sx.is_accelerated_deformable_solve_available():
        # On a CUDA-enabled build the behavior is covered by the cuda env demos.
        return
    assert sx.set_accelerated_deformable_solve(True) is False
    assert sx.is_accelerated_deformable_solve_enabled() is False
