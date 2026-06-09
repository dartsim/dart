import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_cpp_benchmark.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("run_cpp_benchmark", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_run_cpp_benchmark_strips_argument_separator(monkeypatch):
    module = _load_module()
    captured = {}

    def fake_run(benchmark, build_type, run_args, *, cpu_affinity=None):
        captured["benchmark"] = benchmark
        captured["build_type"] = build_type
        captured["run_args"] = run_args
        captured["cpu_affinity"] = cpu_affinity
        return 0

    monkeypatch.setattr(module, "run", fake_run)

    result = module.main(
        [
            "--target",
            "helpers",
            "--build-type",
            "Debug",
            "--",
            "--benchmark_filter=BM_isNan",
        ]
    )

    assert result == 0
    assert captured == {
        "benchmark": "helpers",
        "build_type": "Debug",
        "run_args": ["--benchmark_filter=BM_isNan"],
        "cpu_affinity": None,
    }


def test_auto_cpu_prefers_quiet_high_capacity_cpu(monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "_read_thread_siblings", lambda cpu: {cpu})
    monkeypatch.setattr(
        module,
        "_sample_cpu_utilization",
        lambda cpus: {2: 0.80, 4: 0.0, 16: 0.0},
    )
    monkeypatch.setattr(
        module,
        "_read_cpu_max_frequency",
        lambda cpu: {2: 5000000, 4: 5200000, 16: 4000000}[cpu],
    )

    assert module._choose_auto_cpu([0, 2, 4, 16]) == 4


def test_auto_cpu_prefers_less_busy_cpu_before_capacity(monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "_read_thread_siblings", lambda cpu: {cpu})
    monkeypatch.setattr(
        module,
        "_sample_cpu_utilization",
        lambda cpus: {4: 0.10, 16: 0.0},
    )
    monkeypatch.setattr(
        module,
        "_read_cpu_max_frequency",
        lambda cpu: {4: 5200000, 16: 4000000}[cpu],
    )

    assert module._choose_auto_cpu([4, 16]) == 16


def test_auto_cpu_falls_back_to_previous_nonzero_choice(monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "_sample_cpu_utilization", lambda cpus: {})

    assert module._choose_auto_cpu([0, 1, 2, 3]) == 2


def test_auto_cpu_avoids_busy_thread_sibling(monkeypatch):
    module = _load_module()
    monkeypatch.setattr(
        module,
        "_sample_cpu_utilization",
        lambda cpus: {4: 0.0, 5: 0.80, 16: 0.05},
    )
    monkeypatch.setattr(
        module,
        "_read_cpu_max_frequency",
        lambda cpu: {4: 5200000, 5: 5200000, 16: 4000000}[cpu],
    )
    monkeypatch.setattr(
        module,
        "_read_thread_siblings",
        lambda cpu: {4: {4, 5}, 5: {4, 5}, 16: {16}}[cpu],
    )

    assert module._choose_auto_cpu([4, 5, 16]) == 16


def test_prewarm_cpu_affinity_restores_original_affinity(monkeypatch):
    module = _load_module()
    current_affinity = {0, 1}
    affinity_calls = []

    def fake_getaffinity(pid):
        assert pid == 0
        return set(current_affinity)

    def fake_setaffinity(pid, cpus):
        assert pid == 0
        affinity_calls.append(set(cpus))
        current_affinity.clear()
        current_affinity.update(cpus)

    times = iter([10.0, 10.2, 10.6])
    monkeypatch.setattr(module.os, "sched_getaffinity", fake_getaffinity, raising=False)
    monkeypatch.setattr(module.os, "sched_setaffinity", fake_setaffinity, raising=False)
    monkeypatch.setattr(module.time, "perf_counter", lambda: next(times))

    checksum = module._prewarm_cpu_affinity(3, seconds=0.5)

    assert checksum is not None
    assert affinity_calls == [{3}, {0, 1}]
    assert current_affinity == {0, 1}
