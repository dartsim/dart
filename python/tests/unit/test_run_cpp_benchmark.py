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

    def fake_run(benchmark, build_type, run_args):
        captured["benchmark"] = benchmark
        captured["build_type"] = build_type
        captured["run_args"] = run_args
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
    }
