import importlib.util
import sys
from pathlib import Path


def load_check_module():
    repo_root = Path(__file__).resolve().parents[1]
    scripts_dir = repo_root / "scripts"
    sys.path.insert(0, str(scripts_dir))
    spec = importlib.util.spec_from_file_location(
        "dart_check_collision_benchmarks",
        scripts_dir / "check_collision_benchmarks.py",
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _capture_benchmark_command(module, monkeypatch, tmp_path, argv):
    calls = []
    build_dir = tmp_path / "build"
    binary = build_dir / "bin" / "bm_collision"

    monkeypatch.setattr(module, "_resolve_build_dir", lambda *_: build_dir)
    monkeypatch.setattr(module, "_find_binary", lambda *_: binary)
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda cmd, **kwargs: calls.append((cmd, kwargs)),
    )

    args = module.parse_args(
        [
            "--target",
            "bm_collision",
            "--output",
            str(tmp_path / "result.json"),
            *argv,
        ]
    )
    module._run_target(args)

    assert len(calls) == 2
    return calls[1][0]


def test_run_target_uses_stable_interleaved_measurement_defaults(monkeypatch, tmp_path):
    module = load_check_module()

    command = _capture_benchmark_command(module, monkeypatch, tmp_path, [])

    assert "--benchmark_min_time=1.0s" in command
    assert "--benchmark_min_warmup_time=0.1" in command
    assert "--benchmark_repetitions=3" in command
    assert "--benchmark_report_aggregates_only=true" in command
    assert "--benchmark_enable_random_interleaving=true" in command


def test_run_target_supports_explicit_sampling_and_interleaving_opt_out(
    monkeypatch, tmp_path
):
    module = load_check_module()

    command = _capture_benchmark_command(
        module,
        monkeypatch,
        tmp_path,
        [
            "--benchmark-min-time",
            "0.5s",
            "--benchmark-min-warmup-time",
            "0.2",
            "--benchmark-repetitions",
            "5",
            "--no-benchmark-random-interleaving",
        ],
    )

    assert "--benchmark_min_time=0.5s" in command
    assert "--benchmark_min_warmup_time=0.2" in command
    assert "--benchmark_repetitions=5" in command
    assert "--benchmark_enable_random_interleaving=true" not in command
