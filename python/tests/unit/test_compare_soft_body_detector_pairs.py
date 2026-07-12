import importlib.util
import json
import os
import subprocess
import sys
from argparse import Namespace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "compare_soft_body_detector_pairs.py"


def _load_runner_module():
    script_dir = str(SCRIPT.parent)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    spec = importlib.util.spec_from_file_location(
        "compare_soft_body_detector_pairs",
        SCRIPT,
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_run(
    path: Path,
    *,
    detector: str = "dart",
    scene: str = "soft_cubes",
    scene_index: int = 1,
    threads: int = 1,
    cpu_time: float = 2.0,
    real_time: float = 3.0,
    time_unit: str = "ms",
):
    name = f"BM_SoftBodyStep/{scene_index}/{threads}/200"
    payload = {
        "context": {"executable": "/tmp/BM_INTEGRATION_soft_body"},
        "benchmarks": [
            {
                "name": name,
                "run_name": name,
                "run_type": "iteration",
                "label": f"scene={scene} detector={detector} threads={threads}",
                "cpu_time": cpu_time,
                "real_time": real_time,
                "time_unit": time_unit,
            },
            {
                "name": f"{name}_mean",
                "run_name": name,
                "run_type": "aggregate",
                "aggregate_name": "mean",
                "label": f"scene={scene} detector={detector} threads={threads}",
                "cpu_time": cpu_time * 100,
                "real_time": real_time / 100,
                "time_unit": time_unit,
            },
        ],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def _samples(runner, ratio_by_key=None, pair_count=20):
    ratio_by_key = ratio_by_key or {}
    samples = []
    schedule = runner.build_schedule(pair_count)
    for scene, threads in runner.expected_row_keys():
        ratio = ratio_by_key.get((scene, threads), 1.0)
        for pair, order in enumerate(schedule, start=1):
            samples.append(
                runner.PairSample(
                    scene=scene,
                    threads=threads,
                    pair=pair,
                    order=order,
                    reference_cpu_ms=100.0,
                    candidate_cpu_ms=100.0 * ratio,
                    reference_real_ms=1000.0,
                    candidate_real_ms=1.0,
                )
            )
    return samples


def test_schedule_alternates_and_balances_first_position():
    runner = _load_runner_module()
    schedule = runner.build_schedule(20)

    assert schedule[:4] == [
        ("dart", "native"),
        ("native", "dart"),
        ("dart", "native"),
        ("native", "dart"),
    ]
    assert sum(order[0] == "dart" for order in schedule) == 10
    assert sum(order[0] == "native" for order in schedule) == 10
    with pytest.raises(ValueError, match="positive"):
        runner.build_schedule(0)


def test_protocol_schedule_warms_each_row_immediately_before_measurement():
    runner = _load_runner_module()
    schedule = runner.build_protocol_schedule(
        ["soft_cubes", "soft_open_chain"], [1, 16], 1, 2
    )

    assert [(item.phase, item.scene, item.threads, item.pair) for item in schedule] == [
        ("warmup", "soft_cubes", 1, 1),
        ("measured", "soft_cubes", 1, 1),
        ("measured", "soft_cubes", 1, 2),
        ("warmup", "soft_cubes", 16, 1),
        ("measured", "soft_cubes", 16, 1),
        ("measured", "soft_cubes", 16, 2),
        ("warmup", "soft_open_chain", 1, 1),
        ("measured", "soft_open_chain", 1, 1),
        ("measured", "soft_open_chain", 1, 2),
        ("warmup", "soft_open_chain", 16, 1),
        ("measured", "soft_open_chain", 16, 1),
        ("measured", "soft_open_chain", 16, 2),
    ]


def test_benchmark_command_selects_one_cpu_timed_row(tmp_path):
    runner = _load_runner_module()
    command = runner.build_benchmark_command(
        Path("/tmp/BM_INTEGRATION_soft_body"),
        tmp_path / "run.json",
        "soft_open_chain",
        16,
        "0.5s",
        "0-15",
    )

    assert command[:3] == ["taskset", "-c", "0-15"]
    assert "--benchmark_filter=^BM_SoftBodyStep/3/16/200$" in command
    assert "--benchmark_min_time=0.5s" in command
    assert "--benchmark_repetitions=1" in command
    assert f"--benchmark_out={tmp_path / 'run.json'}" in command
    assert "--benchmark_out_format=json" in command


@pytest.mark.parametrize(
    ("value", "unit", "expected_ms"),
    [(2_000_000.0, "ns", 2.0), (2_000.0, "us", 2.0), (2.0, "ms", 2.0)],
)
def test_run_parser_uses_iteration_cpu_time_and_converts_units(
    tmp_path, value, unit, expected_ms
):
    runner = _load_runner_module()
    path = tmp_path / "run.json"
    _write_run(
        path,
        cpu_time=value,
        real_time=value * 10,
        time_unit=unit,
    )

    row = runner.load_run_cpu_time(path, "dart", "soft_cubes", 1)

    assert row["cpu_ms"] == pytest.approx(expected_ms)
    assert row["real_ms"] == pytest.approx(expected_ms * 10)


def test_run_parser_rejects_wrong_detector_and_duplicate_iterations(tmp_path):
    runner = _load_runner_module()
    path = tmp_path / "run.json"
    _write_run(path, detector="native")

    with pytest.raises(ValueError, match="detector label"):
        runner.load_run_cpu_time(path, "dart", "soft_cubes", 1)

    data = json.loads(path.read_text(encoding="utf-8"))
    data["benchmarks"][0]["label"] = "scene=soft_cubes detector=dart threads=1"
    data["benchmarks"].append(dict(data["benchmarks"][0]))
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="expected one iteration row, found 2"):
        runner.load_run_cpu_time(path, "dart", "soft_cubes", 1)


def test_run_parser_requires_executable_context(tmp_path):
    runner = _load_runner_module()
    path = tmp_path / "run.json"
    _write_run(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    del data["context"]
    path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="missing Google Benchmark executable"):
        runner.load_run_cpu_time(path, "dart", "soft_cubes", 1)


def test_warning_bearing_benchmark_run_is_rejected_and_recorded(tmp_path, monkeypatch):
    runner = _load_runner_module()
    binary = Path("/tmp/BM_INTEGRATION_soft_body")
    revision = runner.matrix.Revision(
        "current", "HEAD", "a" * 40, tmp_path, tmp_path / "build"
    )

    monkeypatch.setattr(
        runner,
        "environment_snapshot",
        lambda _root: {
            "local_dart_workloads": 0,
            "load_average": [0.0, 0.0, 0.0],
            "thermal_celsius": {},
        },
    )

    def fake_run(command, cwd, env, capture_path, timeout):
        del cwd, timeout
        raw_path = Path(
            next(
                item.split("=", 1)[1]
                for item in command
                if item.startswith("--benchmark_out=")
            )
        )
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        _write_run(raw_path, detector=env["COLLISION_DETECTOR"])
        output = "shape will be skipped by the native adapter"
        capture_path.parent.mkdir(parents=True, exist_ok=True)
        capture_path.write_text(output, encoding="utf-8")
        return subprocess.CompletedProcess(command, 0, output)

    monkeypatch.setattr(runner, "run_captured_command", fake_run)

    with pytest.raises(RuntimeError, match="unsupported-shape fallback warning"):
        runner.run_benchmark(
            root=tmp_path,
            revision=revision,
            binary=binary,
            output_dir=tmp_path / "artifact",
            phase="measured",
            scene="soft_cubes",
            threads=1,
            pair=1,
            position=1,
            detector="native",
            min_time="0.5s",
            cpu_list=None,
            timeout=1.0,
        )

    records = [
        json.loads(line)
        for line in (tmp_path / "artifact" / "runs.jsonl").read_text().splitlines()
    ]
    assert records[0]["status"] == "failed"
    assert "unsupported-shape fallback warning" in records[0]["error"]


def test_captured_command_times_out_and_terminates_process_group(tmp_path):
    runner = _load_runner_module()
    log_path = tmp_path / "timeout.log"

    with pytest.raises(subprocess.TimeoutExpired):
        runner.run_captured_command(
            [
                sys.executable,
                "-c",
                "import time; print('started', flush=True); time.sleep(30)",
            ],
            cwd=tmp_path,
            env=dict(os.environ),
            capture_path=log_path,
            timeout=0.05,
        )

    assert "started" in log_path.read_text(encoding="utf-8")


def test_exact_two_percent_boundary_passes_and_just_over_fails():
    runner = _load_runner_module()
    boundary = _samples(runner, {("soft_cubes", 1): 102.0 / 100.0})
    rows, failures = runner.evaluate_pairs(
        boundary,
        runner.REQUIRED_SCENES,
        runner.REQUIRED_THREADS,
        20,
        0.02,
    )
    boundary_row = next(
        row for row in rows if row["scene"] == "soft_cubes" and row["threads"] == 1
    )
    assert boundary_row["pass"]
    assert boundary_row["median_reference_cpu_ms"] == pytest.approx(100.0)
    assert boundary_row["median_candidate_cpu_ms"] == pytest.approx(102.0)
    assert boundary_row["min_cpu_ratio"] == pytest.approx(1.02)
    assert boundary_row["max_cpu_ratio"] == pytest.approx(1.02)
    assert boundary_row["reference_first"] == 10
    assert boundary_row["candidate_first"] == 10
    assert not failures

    over = _samples(runner, {("soft_cubes", 1): 102.000001 / 100.0})
    rows, failures = runner.evaluate_pairs(
        over,
        runner.REQUIRED_SCENES,
        runner.REQUIRED_THREADS,
        20,
        0.02,
    )
    over_row = next(
        row for row in rows if row["scene"] == "soft_cubes" and row["threads"] == 1
    )
    assert not over_row["pass"]
    assert any("soft_cubes/1" in failure for failure in failures)


def test_missing_pair_fails_instead_of_aggregating_nineteen():
    runner = _load_runner_module()
    samples = _samples(runner)
    samples = [
        sample
        for sample in samples
        if not (
            sample.scene == "soft_open_chain"
            and sample.threads == 16
            and sample.pair == 7
        )
    ]

    rows, failures = runner.evaluate_pairs(
        samples,
        runner.REQUIRED_SCENES,
        runner.REQUIRED_THREADS,
        20,
        0.02,
    )

    row = next(
        item
        for item in rows
        if item["scene"] == "soft_open_chain" and item["threads"] == 16
    )
    assert row["pairs"] == 19
    assert not row["pass"]
    assert "soft_open_chain/16: missing pairs 7" in failures


def test_out_of_range_pair_is_rejected_even_when_expected_pairs_are_complete():
    runner = _load_runner_module()
    samples = _samples(runner)
    samples.append(
        runner.PairSample(
            scene="soft_cubes",
            threads=1,
            pair=21,
            order=("dart", "native"),
            reference_cpu_ms=100.0,
            candidate_cpu_ms=100.0,
            reference_real_ms=100.0,
            candidate_real_ms=100.0,
        )
    )

    rows, failures = runner.evaluate_pairs(
        samples,
        runner.REQUIRED_SCENES,
        runner.REQUIRED_THREADS,
        20,
        0.02,
    )

    assert any("pair 21 is outside 1..20" in failure for failure in failures)
    assert all(row["pass"] for row in rows)


def test_evaluator_uses_median_of_paired_cpu_ratios():
    runner = _load_runner_module()
    reference = [1.0, 1.0, 100.0, 100.0]
    candidate = [0.9, 1.1, 90.0, 110.0]
    schedule = runner.build_schedule(4)
    samples = [
        runner.PairSample(
            scene="soft_cubes",
            threads=1,
            pair=index,
            order=schedule[index - 1],
            reference_cpu_ms=reference[index - 1],
            candidate_cpu_ms=candidate[index - 1],
            reference_real_ms=1.0,
            candidate_real_ms=1000.0,
        )
        for index in range(1, 5)
    ]

    rows, failures = runner.evaluate_pairs(samples, ["soft_cubes"], [1], 4, 0.02)

    assert not failures
    assert rows[0]["pass"]
    assert rows[0]["median_cpu_ratio"] == pytest.approx(1.0)
    assert statistics_median(candidate) / statistics_median(reference) != pytest.approx(
        rows[0]["median_cpu_ratio"]
    )


def statistics_median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    return (ordered[middle - 1] + ordered[middle]) / 2


def test_dry_run_serializes_schema_revision_and_full_schedule(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output-dir",
            str(tmp_path / "unused"),
            "--dry-run",
        ],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    plan = json.loads(result.stdout)

    assert plan["metadata"]["schema_version"] == "dart.soft_body_detector_pairs/v1"
    assert len(plan["metadata"]["protocol"]["schedule"]) == 20
    assert plan["metadata"]["protocol"]["schedule"][0] == ["dart", "native"]
    assert plan["metadata"]["protocol"]["schedule"][1] == ["native", "dart"]
    assert plan["metadata"]["protocol"]["correctness_steps"] == "200"
    assert plan["metadata"]["protocol"]["correctness_tolerance"] == 0.05
    assert plan["metadata"]["invocation_cwd"] == str(ROOT)
    assert isinstance(plan["metadata"]["host"]["thermal_available"], bool)
    reproduce = plan["metadata"]["reproduce_command"]
    assert "--dry-run" not in reproduce
    assert (
        reproduce[reproduce.index("--revision") + 1]
        == plan["metadata"]["revision"]["sha"]
    )
    assert reproduce[reproduce.index("--output-dir") + 1] == "OUTPUT_DIR"
    assert len(plan["row_keys"]) == 8
    assert not (tmp_path / "unused").exists()


def test_full_protocol_requires_canonical_scope_and_diagnostic_marks_nonconformance():
    runner = _load_runner_module()
    args = Namespace(
        pairs=2,
        warmup_pairs=1,
        benchmark_min_time="0.01s",
        tie_tolerance=0.02,
        cooldown=0.0,
        initial_idle_seconds=0.0,
        idle_timeout=1.0,
        idle_poll_interval=0.1,
        idle_max_load_1m=100.0,
        pair_max_load_rise_1m=100.0,
        correctness_steps="10",
        correctness_tolerance=1.0,
        thermal_max_celsius=100.0,
        thermal_max_rise_celsius=50.0,
        run_timeout=600.0,
        cpu_list=None,
        diagnostic=False,
    )

    with pytest.raises(ValueError, match="Full protocol requirements"):
        runner.validate_protocol(args, ["soft_cubes"], [1])

    args.diagnostic = True
    runner.validate_protocol(args, ["soft_cubes"], [1])


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("warmup_pairs", 2, "warmup pairs must be 1"),
        ("idle_max_load_1m", 1.01, "idle 1-minute load must be at most 1"),
        (
            "pair_max_load_rise_1m",
            0.51,
            "pair 1-minute load rise must be at most 0.5",
        ),
        ("correctness_steps", "199", "correctness steps must be 200"),
        (
            "correctness_tolerance",
            0.051,
            "correctness tolerance must be 0.05",
        ),
        ("thermal_max_celsius", 80.1, "thermal maximum must be at most 80 C"),
        (
            "thermal_max_rise_celsius",
            15.1,
            "thermal rise must be at most 15 C",
        ),
        ("run_timeout", 301.0, "run timeout must be at most 300s"),
    ],
)
def test_full_protocol_rejects_loosened_evidence_controls(
    tmp_path, field, value, message
):
    runner = _load_runner_module()
    args = runner.parse_args(["--output-dir", str(tmp_path / "unused")])
    setattr(args, field, value)

    with pytest.raises(ValueError, match=message):
        runner.validate_protocol(
            args,
            list(runner.REQUIRED_SCENES),
            list(runner.REQUIRED_THREADS),
        )


@pytest.mark.parametrize(
    "field",
    [
        "idle_max_load_1m",
        "pair_max_load_rise_1m",
        "thermal_max_celsius",
        "thermal_max_rise_celsius",
        "run_timeout",
    ],
)
def test_protocol_rejects_nan_evidence_controls(tmp_path, field):
    runner = _load_runner_module()
    args = runner.parse_args(["--diagnostic", "--output-dir", str(tmp_path / "unused")])
    setattr(args, field, float("nan"))

    with pytest.raises(ValueError, match="must be finite"):
        runner.validate_protocol(args, ["soft_cubes"], [1])


def test_idle_environment_gate_enforces_load_workloads_and_available_thermal_data():
    runner = _load_runner_module()
    clean = {
        "local_dart_workloads": 0,
        "load_average": [0.5, 0.5, 0.5],
        "thermal_celsius": {"coretemp": 50.0},
    }

    assert not runner.idle_environment_failures(clean, 1.0, None, 80.0, 15.0)
    assert not runner.idle_environment_failures(
        {**clean, "thermal_celsius": {}}, 1.0, None, 80.0, 15.0
    )
    failures = runner.idle_environment_failures(
        {
            **clean,
            "local_dart_workloads": 1,
            "load_average": [1.1, 0.5, 0.5],
            "thermal_celsius": {"coretemp": 81.0},
        },
        1.0,
        {"coretemp": 50.0},
        80.0,
        15.0,
    )
    assert any("local DART workloads=1" in failure for failure in failures)
    assert any("1-minute load=1.10" in failure for failure in failures)
    assert any("coretemp=81.00 C > 80 C" in failure for failure in failures)
    assert any("idle baseline 50.00 C + 15 C" in failure for failure in failures)

    unavailable = runner.idle_environment_failures(
        {**clean, "thermal_celsius": {}},
        1.0,
        {"coretemp": 50.0},
        80.0,
        15.0,
    )
    assert unavailable == ["thermal sensors became unavailable"]


def test_pair_environment_gate_checks_both_detectors_before_and_after(tmp_path):
    runner = _load_runner_module()
    args = runner.parse_args(["--diagnostic", "--output-dir", str(tmp_path / "unused")])
    clean = {
        "local_dart_workloads": 0,
        "load_average": [0.5, 0.5, 0.5],
        "thermal_celsius": {"coretemp": 50.0},
    }
    contaminated = {
        "local_dart_workloads": 0,
        "load_average": [9.0, 9.0, 9.0],
        "thermal_celsius": {"coretemp": 90.0},
    }
    records = {
        "dart": {
            "environment_before": clean,
            "environment_after": contaminated,
        },
        "native": {
            "environment_before": contaminated,
            "environment_after": contaminated,
        },
    }

    failures = runner.pair_environment_failures(
        records, ("dart", "native"), args, {"coretemp": 50.0}
    )

    assert any("dart after: 1-minute load=9.00" in failure for failure in failures)
    assert any("native before: 1-minute load=9.00" in failure for failure in failures)
    assert any(
        "native after: coretemp=90.00 C > 80 C" in failure for failure in failures
    )


def test_detector_equivalence_qualifies_every_thread_count(tmp_path, monkeypatch):
    runner = _load_runner_module()
    called = []

    def fake_equivalence(*, thread_count, **_kwargs):
        called.append(thread_count)
        native_ok = thread_count == 1
        return (
            {
                "dart": {"eligible": True, "reason": "reference detector"},
                "native": {
                    "eligible": native_ok,
                    "reason": "checksum-equivalent" if native_ok else "mismatch",
                },
            },
            ["dart", "native"] if native_ok else ["dart"],
        )

    monkeypatch.setattr(
        runner.matrix, "evaluate_detector_equivalence", fake_equivalence
    )
    revision = runner.matrix.Revision(
        "current", "HEAD", "a" * 40, tmp_path, tmp_path / "build"
    )

    results, eligible = runner.evaluate_detector_equivalence_all_threads(
        revision,
        tmp_path / "soft_body_headless",
        ["soft_cubes"],
        [1, 16],
        "200",
        0.05,
        tmp_path / "artifact",
    )

    assert called == [1, 16]
    assert eligible == ["dart"]
    assert not results["native"]["eligible"]
    assert "threads 16: mismatch" in results["native"]["reason"]


def test_headless_checksum_sets_requested_thread_count(tmp_path, monkeypatch):
    runner = _load_runner_module()
    captured = {}

    def fake_run(command, *, cwd, env, capture_path):
        captured.update(
            command=command,
            cwd=cwd,
            threads=env["THREADS"],
            capture_path=capture_path,
        )
        return subprocess.CompletedProcess(
            command,
            0,
            "step 200 skelPosL1 1 skelVelL1 2 pointPosL1 3 pointVelL1 4\n",
        )

    monkeypatch.setattr(runner.matrix, "run", fake_run)
    revision = runner.matrix.Revision(
        "current", "HEAD", "a" * 40, tmp_path, tmp_path / "build"
    )

    checksum, _output = runner.matrix.run_headless_checksum(
        revision,
        tmp_path / "soft_body_headless",
        "native",
        "soft_cubes",
        "200",
        tmp_path / "artifact",
        thread_count=16,
    )

    assert checksum["skelPosL1"] == 1.0
    assert captured["threads"] == "16"
    assert captured["capture_path"].name.endswith("-t16-headless.log")


def test_local_workload_probe_counts_sibling_benchmark_binary(monkeypatch):
    runner = _load_runner_module()
    workspace = runner.matrix.local_dart_workspace_root(ROOT)
    output = "\n".join(
        [
            f"BM_INTEGRATIO {workspace}/task_3/build/bin/BM_INTEGRATION_soft_body",
            f"python {workspace}/task_2/scripts/compare_soft_body_detector_pairs.py",
        ]
    )
    monkeypatch.setattr(
        runner.matrix.subprocess,
        "run",
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 0, output),
    )

    assert runner.matrix.count_local_dart_workloads(ROOT) == 1


def test_revision_owned_harness_is_not_grafted_from_the_live_checkout(
    tmp_path, monkeypatch
):
    runner = _load_runner_module()
    repo = tmp_path / "repo"
    repo.mkdir()
    subprocess.run(["git", "init", "-q"], cwd=repo, check=True)
    subprocess.run(
        ["git", "config", "user.email", "tests@example.com"], cwd=repo, check=True
    )
    subprocess.run(["git", "config", "user.name", "DART Tests"], cwd=repo, check=True)
    for relative in runner.matrix.HARNESS_FILES:
        path = repo / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"pinned:{relative}\n", encoding="utf-8")
    subprocess.run(["git", "add", "."], cwd=repo, check=True)
    subprocess.run(["git", "commit", "-qm", "pinned harness"], cwd=repo, check=True)

    def reject_graft(*args, **kwargs):
        raise AssertionError("live-checkout harness graft attempted")

    monkeypatch.setattr(runner.matrix, "ensure_harness", reject_graft)
    output_dir = tmp_path / "artifact"
    revision = runner.matrix.prepare_revision(repo, output_dir, "current", "HEAD", None)

    for relative in runner.matrix.HARNESS_FILES:
        assert (revision.source_dir / relative).read_text(encoding="utf-8") == (
            f"pinned:{relative}\n"
        )
    metadata = json.loads((output_dir / "current.json").read_text(encoding="utf-8"))
    assert metadata["sha"] == revision.sha
    assert not metadata["benchmark_harness_patched"]


def test_diagnostic_main_runs_adjacent_row_protocol_and_publishes_marker_last(
    tmp_path, monkeypatch
):
    runner = _load_runner_module()
    output_dir = tmp_path / "artifact"
    actions = []
    idle_contexts = []
    lock_stream = (tmp_path / "runner.lock").open("w", encoding="utf-8")

    monkeypatch.setattr(runner.matrix, "repo_root", lambda: ROOT)

    def fake_prepare(root, artifact, label, revision_sha, harness_source):
        assert root == ROOT
        assert artifact == output_dir
        assert label == "current"
        assert len(revision_sha) == 40
        assert harness_source is None
        return runner.matrix.Revision(
            label,
            revision_sha,
            revision_sha,
            ROOT,
            tmp_path / "build",
        )

    monkeypatch.setattr(runner.matrix, "prepare_revision", fake_prepare)
    monkeypatch.setattr(
        runner.matrix,
        "configure_and_build",
        lambda _revision: (
            tmp_path / "BM_INTEGRATION_soft_body",
            tmp_path / "soft_body_headless",
        ),
    )
    monkeypatch.setattr(
        runner,
        "evaluate_detector_equivalence_all_threads",
        lambda *_args, **_kwargs: (
            {
                "dart": {"eligible": True, "reason": "reference detector"},
                "native": {"eligible": True, "reason": "checksum-equivalent"},
            },
            ["dart", "native"],
        ),
    )
    snapshot = {
        "captured_at": "2026-07-12T00:00:00+00:00",
        "load_average": [0.0, 0.0, 0.0],
        "local_dart_workloads": 0,
        "affinity": [0],
        "cpu_governors": ["performance"],
        "thermal_available": False,
        "thermal_celsius": {},
    }

    def fake_wait(*_args, history_context=None, **_kwargs):
        idle_contexts.append(history_context)
        return snapshot

    monkeypatch.setattr(runner, "wait_for_stable_idle", fake_wait)
    monkeypatch.setattr(runner, "environment_snapshot", lambda _root: snapshot)

    def fake_pair(*, phase, scene, threads, pair, order, **_kwargs):
        actions.append((phase, scene, threads, pair, order))
        return runner.PairSample(
            scene=scene,
            threads=threads,
            pair=pair,
            order=order,
            reference_cpu_ms=100.0,
            candidate_cpu_ms=100.0,
            reference_real_ms=100.0,
            candidate_real_ms=100.0,
        )

    monkeypatch.setattr(runner, "run_pair", fake_pair)
    monkeypatch.setattr(
        runner,
        "acquire_benchmark_lock",
        lambda _root, _sha: (lock_stream, tmp_path / "runner.lock"),
    )

    result = runner.main(
        [
            "--diagnostic",
            "--scenes",
            "soft_cubes",
            "--threads",
            "1",
            "--pairs",
            "2",
            "--initial-idle-seconds",
            "0",
            "--cooldown",
            "0",
            "--idle-max-load-1m",
            "100",
            "--output-dir",
            str(output_dir),
        ]
    )

    assert result == 0
    assert [
        (phase, pair, order) for phase, _scene, _threads, pair, order in actions
    ] == [
        ("warmup", 1, ("dart", "native")),
        ("measured", 1, ("dart", "native")),
        ("measured", 2, ("native", "dart")),
    ]
    assert idle_contexts[0] == {"phase": "preflight"}
    assert [context["phase"] for context in idle_contexts[1:]] == [
        "warmup",
        "measured",
        "measured",
    ]
    marker = json.loads((output_dir / "COMPLETE.json").read_text())
    assert marker["status"] == "complete"
    assert marker["verdict"] == "DIAGNOSTIC"
    assert json.loads((output_dir / "summary.json").read_text())["verdict"] == (
        "DIAGNOSTIC"
    )
    assert len((output_dir / "pairs.jsonl").read_text().splitlines()) == 2


def test_completion_marker_is_written_last(tmp_path, monkeypatch):
    runner = _load_runner_module()
    args = runner.parse_args(["--diagnostic", "--output-dir", str(tmp_path / "unused")])
    metadata = runner.build_metadata(
        [], "HEAD", "a" * 40, [], ["soft_cubes"], [1], args
    )
    summary = {
        "schema_version": runner.SCHEMA_VERSION,
        "revision": metadata["revision"],
        "detector_equivalence": {},
        "rows": [],
        "failures": [],
        "verdict": "DIAGNOSTIC",
    }
    writes = []
    original_write_json = runner.write_json

    def tracked_write(path, payload):
        writes.append(path.name)
        original_write_json(path, payload)

    monkeypatch.setattr(runner, "write_json", tracked_write)
    runner.finalize_artifact(
        tmp_path,
        tmp_path / "metadata.json",
        metadata,
        summary,
    )

    assert writes[-1] == "COMPLETE.json"
