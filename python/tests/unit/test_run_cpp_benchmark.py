import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_cpp_benchmark.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("run_cpp_benchmark", SCRIPT)
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
            "kinematics",
            "--build-type",
            "Debug",
            "--",
            "--benchmark_filter=BM_Kinematics/10",
        ]
    )

    assert result == 0
    assert captured == {
        "benchmark": "kinematics",
        "build_type": "Debug",
        "run_args": ["--benchmark_filter=BM_Kinematics/10"],
    }


def test_run_cpp_benchmark_resolves_release_branch_aliases():
    module = _load_module()

    assert module._resolve_target("boxes") == "BM_INTEGRATION_boxes"
    assert (
        module._resolve_target("contact-container")
        == "BM_INTEGRATION_contact_container"
    )
    assert module._resolve_target("BM_CUSTOM") == "BM_CUSTOM"
