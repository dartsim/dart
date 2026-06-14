import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan083_gpu_ccd_line_search_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan083_gpu_ccd_line_search_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _row(name: str, **counters):
    row = {
        "name": name,
        "real_time": 10.0,
        "cpu_time": 10.0,
        "time_unit": "ns",
        "aggregate_name": "median",
        "iterations": 1,
    }
    row.update(counters)
    return row


def _benchmark_data(**overrides):
    cpu = _row(
        "BM_Plan083CcdLineSearchCpu/1024",
        pairs=1024,
        hits=768,
        min_step_bound=0.5,
        max_result_abs_error=0.0,
    )
    gpu = _row(
        "BM_Plan083CcdLineSearchCuda/1024",
        real_time=5.0,
        cpu_time=5.0,
        pairs=1024,
        hits=768,
        gpu_hits=768,
        min_step_bound=0.5,
        max_result_abs_error=1e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        kernel_ns=3.0,
        device_to_host_ns=4.0,
    )
    edge_cpu = _row(
        "BM_Plan083EdgeEdgeCcdLineSearchCpu/1024",
        pairs=1024,
        hits=512,
        min_step_bound=0.25,
        max_result_abs_error=0.0,
    )
    edge_gpu = _row(
        "BM_Plan083EdgeEdgeCcdLineSearchCuda/1024",
        real_time=8.0,
        cpu_time=8.0,
        pairs=1024,
        hits=512,
        gpu_hits=512,
        min_step_bound=0.25,
        max_result_abs_error=2e-12,
        host_setup_ns=1.5,
        host_to_device_ns=2.5,
        kernel_ns=3.5,
        device_to_host_ns=4.5,
    )
    curved_cpu = _row(
        "BM_Plan083RigidCurvedPointTriangleCcdLineSearchCpu/1024",
        pairs=1024,
        segments=8192,
        samples_per_pair=8,
        hits=640,
        min_step_bound=0.125,
        max_result_abs_error=0.0,
        analytic_reference_pairs=64,
        analytic_reference_hits=48,
        analytic_reference_hit_mismatches=0,
        analytic_reference_min_step_bound=0.125,
        max_sampled_reference_overshoot=0.0,
        max_sampled_reference_conservative_gap=0.05,
        interval_reference_pairs=64,
        interval_reference_hits=48,
        interval_reference_hit_mismatches=0,
        interval_reference_min_step_bound=0.125,
        max_sampled_interval_reference_overshoot=0.0,
        max_sampled_interval_reference_conservative_gap=0.05,
    )
    curved_gpu = _row(
        "BM_Plan083RigidCurvedPointTriangleCcdLineSearchCuda/1024",
        real_time=7.0,
        cpu_time=7.0,
        pairs=1024,
        segments=8192,
        gpu_segments=8192,
        samples_per_pair=8,
        hits=640,
        gpu_hits=640,
        min_step_bound=0.125,
        max_result_abs_error=3e-12,
        analytic_reference_pairs=64,
        analytic_reference_hits=48,
        analytic_reference_hit_mismatches=0,
        analytic_reference_min_step_bound=0.125,
        max_sampled_reference_overshoot=0.0,
        max_sampled_reference_conservative_gap=0.05,
        interval_reference_pairs=64,
        interval_reference_hits=48,
        interval_reference_hit_mismatches=0,
        interval_reference_min_step_bound=0.125,
        max_sampled_interval_reference_overshoot=0.0,
        max_sampled_interval_reference_conservative_gap=0.05,
        host_setup_ns=1.25,
        host_to_device_ns=2.25,
        kernel_ns=3.25,
        device_to_host_ns=4.25,
    )
    curved_edge_cpu = _row(
        "BM_Plan083RigidCurvedEdgeEdgeCcdLineSearchCpu/1024",
        pairs=1024,
        segments=8192,
        samples_per_pair=8,
        hits=512,
        min_step_bound=0.375,
        max_result_abs_error=0.0,
        analytic_reference_pairs=64,
        analytic_reference_hits=48,
        analytic_reference_hit_mismatches=0,
        analytic_reference_min_step_bound=0.25,
        max_sampled_reference_overshoot=0.0,
        max_sampled_reference_conservative_gap=0.08,
        interval_reference_pairs=64,
        interval_reference_hits=48,
        interval_reference_hit_mismatches=0,
        interval_reference_min_step_bound=0.25,
        max_sampled_interval_reference_overshoot=0.0,
        max_sampled_interval_reference_conservative_gap=0.08,
    )
    curved_edge_gpu = _row(
        "BM_Plan083RigidCurvedEdgeEdgeCcdLineSearchCuda/1024",
        real_time=6.0,
        cpu_time=6.0,
        pairs=1024,
        segments=8192,
        gpu_segments=8192,
        samples_per_pair=8,
        hits=512,
        gpu_hits=512,
        min_step_bound=0.375,
        max_result_abs_error=4e-12,
        analytic_reference_pairs=64,
        analytic_reference_hits=48,
        analytic_reference_hit_mismatches=0,
        analytic_reference_min_step_bound=0.25,
        max_sampled_reference_overshoot=0.0,
        max_sampled_reference_conservative_gap=0.08,
        interval_reference_pairs=64,
        interval_reference_hits=48,
        interval_reference_hit_mismatches=0,
        interval_reference_min_step_bound=0.25,
        max_sampled_interval_reference_overshoot=0.0,
        max_sampled_interval_reference_conservative_gap=0.08,
        host_setup_ns=1.75,
        host_to_device_ns=2.75,
        kernel_ns=3.75,
        device_to_host_ns=4.75,
    )
    scene_pt_cpu = _row(
        "BM_Plan083SceneRuntimePointTriangleCcdLineSearchCpu/1024",
        pairs=256,
        hits=192,
        min_step_bound=0.4,
        max_result_abs_error=0.0,
        scene_bodies=1,
        scene_nodes=2560,
        scene_triangles=768,
        runtime_point_triangle_candidates=256,
        static_triangle_point_triangle_candidates=128,
        moving_triangle_point_triangle_candidates=128,
    )
    scene_pt_gpu = _row(
        "BM_Plan083SceneRuntimePointTriangleCcdLineSearchCuda/1024",
        real_time=4.0,
        cpu_time=4.0,
        pairs=256,
        hits=192,
        gpu_hits=192,
        min_step_bound=0.4,
        max_result_abs_error=5e-12,
        scene_bodies=1,
        scene_nodes=2560,
        scene_triangles=768,
        runtime_point_triangle_candidates=256,
        static_triangle_point_triangle_candidates=128,
        moving_triangle_point_triangle_candidates=128,
        host_setup_ns=1.1,
        host_to_device_ns=2.1,
        kernel_ns=3.1,
        device_to_host_ns=4.1,
    )
    scene_ee_cpu = _row(
        "BM_Plan083SceneRuntimeEdgeEdgeCcdLineSearchCpu/1024",
        pairs=384,
        hits=192,
        min_step_bound=0.35,
        max_result_abs_error=0.0,
        scene_bodies=1,
        scene_nodes=2560,
        scene_triangles=768,
        runtime_edge_edge_candidates=384,
    )
    scene_ee_gpu = _row(
        "BM_Plan083SceneRuntimeEdgeEdgeCcdLineSearchCuda/1024",
        real_time=7.0,
        cpu_time=7.0,
        pairs=384,
        hits=192,
        gpu_hits=192,
        min_step_bound=0.35,
        max_result_abs_error=6e-12,
        scene_bodies=1,
        scene_nodes=2560,
        scene_triangles=768,
        runtime_edge_edge_candidates=384,
        host_setup_ns=1.6,
        host_to_device_ns=2.6,
        kernel_ns=3.6,
        device_to_host_ns=4.6,
    )
    scene_combined_cpu = _row(
        "BM_Plan083SceneRuntimeCombinedCcdLineSearchCpu/1024",
        pairs=640,
        point_triangle_pairs=256,
        edge_edge_pairs=384,
        hits=384,
        point_triangle_hits=192,
        edge_edge_hits=192,
        min_step_bound=0.35,
        max_result_abs_error=0.0,
        scene_bodies=1,
        scene_nodes=2560,
        scene_triangles=768,
        runtime_point_triangle_candidates=256,
        static_triangle_point_triangle_candidates=128,
        moving_triangle_point_triangle_candidates=128,
        runtime_edge_edge_candidates=384,
    )
    scene_combined_gpu = _row(
        "BM_Plan083SceneRuntimeCombinedCcdLineSearchCuda/1024",
        real_time=4.0,
        cpu_time=4.0,
        pairs=640,
        point_triangle_pairs=256,
        edge_edge_pairs=384,
        hits=384,
        point_triangle_hits=192,
        edge_edge_hits=192,
        gpu_hits=384,
        gpu_point_triangle_hits=192,
        gpu_edge_edge_hits=192,
        min_step_bound=0.35,
        max_result_abs_error=7e-12,
        scene_bodies=1,
        scene_nodes=2560,
        scene_triangles=768,
        runtime_point_triangle_candidates=256,
        static_triangle_point_triangle_candidates=128,
        moving_triangle_point_triangle_candidates=128,
        runtime_edge_edge_candidates=384,
        host_setup_ns=2.7,
        host_to_device_ns=4.7,
        kernel_ns=6.7,
        device_to_host_ns=8.7,
    )
    for row in (
        gpu,
        edge_gpu,
        curved_gpu,
        curved_edge_gpu,
        scene_pt_gpu,
        scene_ee_gpu,
        scene_combined_gpu,
    ):
        row.update(overrides)
    return {
        "benchmarks": [
            cpu,
            gpu,
            edge_cpu,
            edge_gpu,
            curved_cpu,
            curved_gpu,
            curved_edge_cpu,
            curved_edge_gpu,
            scene_pt_cpu,
            scene_pt_gpu,
            scene_ee_cpu,
            scene_ee_gpu,
            scene_combined_cpu,
            scene_combined_gpu,
        ]
    }


def test_plan083_gpu_ccd_line_search_packet_accepts_parity_rows() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(),
        pair_count=1024,
        tolerance=1e-8,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_ccd_line_search_packet"]
    assert row["row_id"] == "ccd-line-search"
    assert row["same_scene_cpu_gpu"] is True
    assert row["pair_count"] == 5376
    assert row["segment_count"] == 19712
    assert row["hit_count"] == 3200
    assert row["analytic_reference_pair_count"] == 128
    assert row["analytic_reference_hit_count"] == 96
    assert row["max_sampled_reference_overshoot"] == 0.0
    assert row["max_sampled_reference_conservative_gap"] == 0.08
    assert row["interval_reference_pair_count"] == 128
    assert row["interval_reference_hit_count"] == 96
    assert row["max_sampled_interval_reference_overshoot"] == 0.0
    assert row["max_sampled_interval_reference_conservative_gap"] == 0.08
    assert row["min_step_bound"] == 0.125
    assert row["max_result_abs_error"] == 7e-12
    assert set(row["primitive_families"]) == {
        "point_triangle",
        "edge_edge",
        "rigid_curved_point_triangle",
        "rigid_curved_edge_edge",
        "scene_runtime_point_triangle",
        "scene_runtime_edge_edge",
        "scene_runtime_combined",
    }
    assert (
        row["primitive_families"]["rigid_curved_point_triangle"]["segment_count"]
        == 8192
    )
    assert (
        row["primitive_families"]["rigid_curved_point_triangle"]["samples_per_pair"]
        == 8
    )
    assert (
        row["primitive_families"]["rigid_curved_point_triangle"][
            "analytic_reference_method"
        ]
        == "rigid_ipc_curved_accd_prefix"
    )
    assert (
        row["primitive_families"]["rigid_curved_point_triangle"][
            "analytic_reference_pairs"
        ]
        == 64
    )
    assert (
        row["primitive_families"]["rigid_curved_point_triangle"][
            "analytic_reference_hit_mismatches"
        ]
        == 0
    )
    assert (
        row["primitive_families"]["scene_runtime_point_triangle"][
            "runtime_point_triangle_candidates"
        ]
        == 256
    )
    assert (
        row["primitive_families"]["scene_runtime_point_triangle"][
            "static_triangle_point_triangle_candidates"
        ]
        == 128
    )
    assert (
        row["primitive_families"]["scene_runtime_point_triangle"][
            "moving_triangle_point_triangle_candidates"
        ]
        == 128
    )
    assert (
        row["primitive_families"]["scene_runtime_edge_edge"][
            "runtime_edge_edge_candidates"
        ]
        == 384
    )
    combined = row["primitive_families"]["scene_runtime_combined"]
    assert combined["pair_count"] == 640
    assert combined["point_triangle_pairs"] == 256
    assert combined["edge_edge_pairs"] == 384
    assert combined["hit_count"] == 384
    assert combined["point_triangle_hits"] == 192
    assert combined["edge_edge_hits"] == 192
    assert combined["runtime_point_triangle_candidates"] == 256
    assert combined["runtime_edge_edge_candidates"] == 384
    assert row["meets_speedup_gate"] is True


def test_plan083_gpu_ccd_line_search_packet_rejects_accuracy_failure() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(max_result_abs_error=1e-3),
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "exceeds tolerance" in str(exc)
    else:
        raise AssertionError("expected accuracy failure")


def test_plan083_gpu_ccd_line_search_packet_rejects_hit_mismatch() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(gpu_hits=767),
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "hit count" in str(exc)
    else:
        raise AssertionError("expected hit-count failure")


def test_plan083_gpu_ccd_line_search_packet_rejects_missing_analytic_reference() -> (
    None
):
    module = _load_module()
    data = _benchmark_data()
    curved_gpu = next(
        row
        for row in data["benchmarks"]
        if row["name"] == "BM_Plan083RigidCurvedPointTriangleCcdLineSearchCuda/1024"
    )
    del curved_gpu["analytic_reference_pairs"]

    try:
        module.make_packet(
            data,
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "analytic_reference_pairs" in str(exc)
    else:
        raise AssertionError("expected missing analytic reference failure")


def test_plan083_gpu_ccd_line_search_packet_rejects_analytic_hit_mismatch() -> None:
    module = _load_module()
    data = _benchmark_data()
    curved_rows = [
        row
        for row in data["benchmarks"]
        if row["name"].startswith("BM_Plan083RigidCurvedPointTriangleCcdLineSearch")
    ]
    for row in curved_rows:
        row["analytic_reference_hit_mismatches"] = 1

    try:
        module.make_packet(
            data,
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "analytic reference hit mismatches" in str(exc)
    else:
        raise AssertionError("expected analytic reference mismatch failure")


def test_plan083_gpu_ccd_line_search_packet_rejects_analytic_overshoot() -> None:
    module = _load_module()
    data = _benchmark_data()
    curved_rows = [
        row
        for row in data["benchmarks"]
        if row["name"].startswith("BM_Plan083RigidCurvedPointTriangleCcdLineSearch")
    ]
    for row in curved_rows:
        row["max_sampled_reference_overshoot"] = 1e-3

    try:
        module.make_packet(
            data,
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "overshoots analytic reference" in str(exc)
    else:
        raise AssertionError("expected analytic overshoot failure")


def test_plan083_gpu_ccd_line_search_packet_rejects_missing_interval_reference() -> (
    None
):
    module = _load_module()
    data = _benchmark_data()
    curved_gpu = next(
        row
        for row in data["benchmarks"]
        if row["name"] == "BM_Plan083RigidCurvedPointTriangleCcdLineSearchCuda/1024"
    )
    del curved_gpu["interval_reference_pairs"]

    try:
        module.make_packet(
            data,
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "interval_reference_pairs" in str(exc)
    else:
        raise AssertionError("expected missing interval reference failure")


def test_plan083_gpu_ccd_line_search_packet_rejects_interval_hit_mismatch() -> None:
    module = _load_module()
    data = _benchmark_data()
    curved_rows = [
        row
        for row in data["benchmarks"]
        if row["name"].startswith("BM_Plan083RigidCurvedPointTriangleCcdLineSearch")
    ]
    for row in curved_rows:
        row["interval_reference_hit_mismatches"] = 1

    try:
        module.make_packet(
            data,
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "interval reference hit mismatches" in str(exc)
    else:
        raise AssertionError("expected interval reference mismatch failure")


def test_plan083_gpu_ccd_line_search_packet_rejects_interval_overshoot() -> None:
    module = _load_module()
    data = _benchmark_data()
    curved_rows = [
        row
        for row in data["benchmarks"]
        if row["name"].startswith("BM_Plan083RigidCurvedPointTriangleCcdLineSearch")
    ]
    for row in curved_rows:
        row["max_sampled_interval_reference_overshoot"] = 1e-3

    try:
        module.make_packet(
            data,
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "overshoots interval reference" in str(exc)
    else:
        raise AssertionError("expected interval overshoot failure")


def test_plan083_gpu_ccd_line_search_packet_records_speedup_gate_miss() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(real_time=20.0, cpu_time=20.0),
        pair_count=1024,
        tolerance=1e-8,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_ccd_line_search_packet"]
    assert row["speedup"] == 0.5
    assert row["meets_speedup_gate"] is False


def test_plan083_gpu_ccd_line_search_packet_rejects_combined_scene_mismatch() -> None:
    module = _load_module()
    data = _benchmark_data()
    combined_cpu = next(
        row
        for row in data["benchmarks"]
        if row["name"] == "BM_Plan083SceneRuntimeCombinedCcdLineSearchCpu/1024"
    )
    combined_gpu = next(
        row
        for row in data["benchmarks"]
        if row["name"] == "BM_Plan083SceneRuntimeCombinedCcdLineSearchCuda/1024"
    )
    for row in (combined_cpu, combined_gpu):
        row["pairs"] = 639
        row["point_triangle_pairs"] = 255
        row["runtime_point_triangle_candidates"] = 255
        row["moving_triangle_point_triangle_candidates"] = 127

    try:
        module.make_packet(
            data,
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "point-triangle counters" in str(exc)
    else:
        raise AssertionError("expected combined scene mismatch")
