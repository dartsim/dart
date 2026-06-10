import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_plan083_gpu_parity_packet.py"
PACKET = (
    ROOT
    / "docs"
    / "plans"
    / "083-unified-newton-barrier-multibody"
    / "gpu-parity-packet.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_plan083_gpu_parity_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _base_row(**overrides):
    row = {
        "row_id": "psd-projection",
        "roadmap_item": "PSD projection",
        "owner": "PLAN-083",
        "status": "measured",
        "cpu_evidence_command": "pixi run cpu",
        "gpu_evidence_command": "pixi run -e cuda gpu",
        "parity_evidence_artifact": ".benchmark_results/plan083/gpu/psd.json",
        "benchmark_profile_artifact": ".benchmark_results/plan083/gpu/psd_bm.json",
        "same_scene_policy": "same fixtures",
        "tolerance_policy": "1e-9",
        "timing_policy": "full transfer and kernel timing",
        "speedup_policy": ">=1.25x",
        "public_api_policy": "No public CUDA or backend exposure.",
        "limitation_status": "measured packet",
        "notes_or_gap": "test row",
        "same_scene_cpu_gpu": True,
        "max_result_abs_error": 1e-12,
        "result_abs_error_tolerance": 1e-9,
        "speedup": 1.5,
        "min_speedup": 1.25,
        "timing_ns": {
            "setup": 1.0,
            "host_to_device": 2.0,
            "kernel": 3.0,
            "solve": 0.0,
            "device_to_host": 2.0,
            "readback": 1.0,
        },
    }
    row.update(overrides)
    return row


def _packet(row):
    return {
        "schema_version": 1,
        "source": {},
        "policy": {},
        "summary": {"row_count": 1, "status_counts": {row["status"]: 1}},
        "rows": [row],
    }


def test_plan083_gpu_parity_packet_manifest_is_complete() -> None:
    module = _load_module()
    packet = module.load_packet(PACKET)

    assert module.validate_packet(packet) == []


def test_plan083_gpu_parity_packet_accepts_measured_row(monkeypatch) -> None:
    module = _load_module()
    monkeypatch.setattr(module, "EXPECTED_ROW_IDS", {"psd-projection"})

    assert module.validate_packet(_packet(_base_row())) == []


def test_plan083_gpu_parity_packet_rejects_measured_row_without_same_scene(
    monkeypatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(module, "EXPECTED_ROW_IDS", {"psd-projection"})

    errors = module.validate_packet(_packet(_base_row(same_scene_cpu_gpu=False)))

    assert "psd-projection: measured rows require same_scene_cpu_gpu=true" in errors


def test_plan083_gpu_parity_packet_rejects_bad_timing_shape(monkeypatch) -> None:
    module = _load_module()
    monkeypatch.setattr(module, "EXPECTED_ROW_IDS", {"psd-projection"})
    row = _base_row(timing_ns={"setup": 1.0, "kernel": -1.0, "extra": 0.0})

    errors = module.validate_packet(_packet(row))

    assert any("timing_ns missing keys" in error for error in errors)
    assert "psd-projection: timing_ns has unexpected key 'extra'" in errors
    assert "psd-projection: timing_ns.kernel must be non-negative" in errors


def test_plan083_gpu_parity_packet_rejects_failed_accuracy_and_speedup(
    monkeypatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(module, "EXPECTED_ROW_IDS", {"psd-projection"})
    row = _base_row(max_result_abs_error=1e-5, speedup=1.1)

    errors = module.validate_packet(_packet(row))

    assert any("result error" in error for error in errors)
    assert any("speedup 1.100 is below required 1.250" in error for error in errors)


def test_plan083_gpu_parity_packet_rejects_public_api_leakage(monkeypatch) -> None:
    module = _load_module()
    monkeypatch.setattr(module, "EXPECTED_ROW_IDS", {"psd-projection"})
    row = _base_row(public_api_policy="Expose a CUDA stream in public API.")

    errors = module.validate_packet(_packet(row))

    assert "psd-projection: public_api_policy must state no public exposure" in errors
