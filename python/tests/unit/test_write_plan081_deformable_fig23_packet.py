from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_plan081_deformable_fig23_packet.py"
MANIFEST = (
    ROOT
    / "docs"
    / "plans"
    / "081-deformable-implicit-barrier-solver"
    / "fig23_deformable_statistics_corpus.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan081_deformable_fig23_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _benchmark_row(manifest_row: dict[str, object]) -> dict[str, object]:
    name = manifest_row["benchmark"]
    row: dict[str, object] = {
        "name": f"{name}_median",
        "run_name": f"{name}_median",
        "aggregate_name": "median",
        "real_time": 1.5,
        "cpu_time": 1.5,
        "time_unit": "ms",
    }
    # Every declared Fig-23 counter is present and strictly positive (so it
    # satisfies both the non-negative metric gate and any positive_fields gate).
    for offset, field in enumerate(manifest_row["metric_fields"]):
        row[field] = float(offset + 1)
    # zero_fields counters are present and exactly zero.
    for field in manifest_row.get("zero_fields", []):
        row[field] = 0.0
    return row


def _benchmark_data(manifest: dict[str, object]) -> dict[str, object]:
    rows = manifest["rows"]
    assert isinstance(rows, list)
    return {"benchmarks": [_benchmark_row(row) for row in rows]}


def _row_by_id(
    manifest: dict[str, object], row_id: str
) -> tuple[int, dict[str, object]]:
    for index, row in enumerate(manifest["rows"]):
        if row["row_id"] == row_id:
            return index, row
    raise AssertionError(f"manifest has no row {row_id}")


def test_manifest_loads_and_is_shape_parity():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)

    assert manifest["paper_scale"] is False
    assert manifest["rows"]
    families = {row["family"] for row in manifest["rows"]}
    # Both the solver-scaling and contact-statistics Fig-23 axes are covered.
    assert families == set(module.KNOWN_FAMILIES)


def test_packet_accepts_manifest_rows():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)

    packet = module.make_packet(_benchmark_data(manifest), manifest)

    metadata = packet[module.PACKET_KEY]
    assert metadata["paper_scale"] is False
    assert metadata["row_count"] == len(manifest["rows"])
    by_id = {row["row_id"]: row for row in metadata["rows"]}
    # The contact-statistics row carries the #3257 peak-contacts axis.
    contact = by_id["self-contact-barrier"]
    assert contact["family"] == "contact-statistics"
    assert "max_active_contacts" in contact["metrics"]
    # 1.5 ms recorded as a per-step wall time in nanoseconds.
    assert contact["per_step_time_ns"] == pytest.approx(1.5e6)
    # The matrix-free row's zero_fields are surfaced as enforced-zero evidence.
    matrix_free = by_id["fem-matrix-free-cg-bar"]
    assert matrix_free["metrics"]["hessian_nonzeros"] == 0.0
    assert matrix_free["metrics"]["hessian_storage_bytes"] == 0.0


def test_packet_rejects_missing_benchmark_row():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    data = _benchmark_data(manifest)
    data["benchmarks"].pop()

    with pytest.raises(
        module.Plan081DeformableFig23PacketError, match="missing benchmark row"
    ):
        module.make_packet(data, manifest)


def test_packet_prefers_median_over_raw_rows():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    data = _benchmark_data(manifest)
    first_row = manifest["rows"][0]
    metric = first_row["metric_fields"][0]

    # The median aggregate reports 7.0 for the first counter; a stray raw row
    # for the same benchmark reports 99.0 and must be ignored.
    data["benchmarks"][0][metric] = 7.0
    raw_row = _benchmark_row(first_row)
    raw_row["name"] = first_row["benchmark"]
    raw_row["run_name"] = first_row["benchmark"]
    raw_row.pop("aggregate_name")
    raw_row[metric] = 99.0
    data["benchmarks"].append(raw_row)

    packet = module.make_packet(data, manifest)

    first = packet[module.PACKET_KEY]["rows"][0]
    assert first["metrics"][metric] == 7.0


def test_packet_rejects_missing_metric_counter():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    data = _benchmark_data(manifest)
    dropped = manifest["rows"][0]["metric_fields"][0]
    data["benchmarks"][0].pop(dropped)

    with pytest.raises(module.Plan081DeformableFig23PacketError, match=dropped):
        module.make_packet(data, manifest)


def test_packet_rejects_negative_metric_counter():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    data = _benchmark_data(manifest)
    negated = manifest["rows"][0]["metric_fields"][0]
    data["benchmarks"][0][negated] = -1.0

    with pytest.raises(module.Plan081DeformableFig23PacketError, match="negative"):
        module.make_packet(data, manifest)


def test_packet_rejects_non_positive_required_field():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    index, row = _row_by_id(manifest, "self-contact-barrier")
    assert "min_active_contact_distance" in row["positive_fields"]
    data = _benchmark_data(manifest)
    # A zero minimum active-contact distance is the "no active contacts"
    # sentinel; it passes the non-negative gate but fails the strict > 0 gate.
    data["benchmarks"][index]["min_active_contact_distance"] = 0.0

    with pytest.raises(module.Plan081DeformableFig23PacketError, match="non-positive"):
        module.make_packet(data, manifest)


def test_packet_rejects_nonzero_zero_field():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    index, row = _row_by_id(manifest, "fem-matrix-free-cg-bar")
    assert "hessian_nonzeros" in row["zero_fields"]
    data = _benchmark_data(manifest)
    # A matrix-free solve that assembled a sparse Hessian violates its contract.
    data["benchmarks"][index]["hessian_nonzeros"] = 5.0

    with pytest.raises(module.Plan081DeformableFig23PacketError, match="nonzero"):
        module.make_packet(data, manifest)


def test_load_manifest_rejects_paper_scale_true(tmp_path):
    module = _load_module()
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    manifest["paper_scale"] = True
    bogus = tmp_path / "paper_scale_true.json"
    bogus.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(
        module.Plan081DeformableFig23PacketError, match="paper_scale must be false"
    ):
        module.load_manifest(bogus)


def test_load_manifest_rejects_non_integer_schema_version(tmp_path):
    module = _load_module()
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    manifest["schema_version"] = "v1"
    bogus = tmp_path / "bad_schema_version.json"
    bogus.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(
        module.Plan081DeformableFig23PacketError,
        match="schema_version must be a positive integer",
    ):
        module.load_manifest(bogus)
