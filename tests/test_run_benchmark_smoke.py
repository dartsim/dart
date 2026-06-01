import importlib.util
import subprocess
import sys
from pathlib import Path


def load_smoke_module():
    repo_root = Path(__file__).resolve().parents[1]
    scripts_dir = repo_root / "scripts"
    sys.path.insert(0, str(scripts_dir))
    spec = importlib.util.spec_from_file_location(
        "dart_run_benchmark_smoke", scripts_dir / "run_benchmark_smoke.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _CompletedStub:
    def __init__(self, stdout="", stderr="", returncode=0):
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode


def test_count_matching_benchmarks_counts_nonblank_stdout_lines(monkeypatch):
    module = load_smoke_module()
    captured = {}

    def fake_run(cmd, **kwargs):
        captured["cmd"] = cmd
        captured["kwargs"] = kwargs
        return _CompletedStub(stdout="BM_A/1\nBM_B/2\n\n")

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    count = module.count_matching_benchmarks("bin/bm", ["--benchmark_filter=BM_"])

    assert count == 2
    # The list invocation must ask Google Benchmark to enumerate, not run.
    assert captured["cmd"] == [
        "bin/bm",
        "--benchmark_list_tests=true",
        "--benchmark_filter=BM_",
    ]
    assert captured["kwargs"]["capture_output"] is True


def test_count_matching_benchmarks_raises_and_reemits_on_nonzero(monkeypatch, capsys):
    module = load_smoke_module()

    def fake_run(cmd, **kwargs):
        return _CompletedStub(
            stdout="partial stdout\n",
            stderr="CUDA driver init failed\n",
            returncode=134,
        )

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    try:
        module.count_matching_benchmarks("bin/bm", ["--benchmark_filter=BM_X$"])
        raised = False
    except module.BenchmarkEnumerationError as exc:
        raised = True
        assert "exited 134" in str(exc)

    assert raised
    # The binary's own diagnostic must reach the log, not be swallowed.
    out = capsys.readouterr()
    assert "partial stdout" in out.out
    assert "CUDA driver init failed" in out.err


def test_count_matching_benchmarks_raises_on_missing_binary(monkeypatch):
    module = load_smoke_module()

    def fake_run(cmd, **kwargs):
        raise FileNotFoundError(2, "No such file or directory")

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    try:
        module.count_matching_benchmarks("bin/missing", ["--benchmark_filter=BM_X$"])
        raised = False
    except module.BenchmarkEnumerationError as exc:
        raised = True
        assert "not found" in str(exc)

    assert raised


def test_run_smoke_fails_and_skips_run_when_no_match(monkeypatch):
    module = load_smoke_module()
    calls = []

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        # First call is the list invocation; report zero matches.
        return _CompletedStub(stdout="")

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    rc = module.run_smoke("bin/bm", ["--benchmark_filter=BM_Missing$"])

    assert rc == 1
    # Only the list invocation runs; the benchmark itself must not be executed.
    assert len(calls) == 1
    assert "--benchmark_list_tests=true" in calls[0]


def test_run_smoke_fails_and_skips_run_when_enumerate_errors(monkeypatch):
    module = load_smoke_module()
    calls = []

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        return _CompletedStub(stderr="boom\n", returncode=1)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    rc = module.run_smoke("bin/bm", ["--benchmark_filter=BM_X$"])

    assert rc == 1
    # The enumerate failure must abort before the real benchmark run.
    assert len(calls) == 1
    assert "--benchmark_list_tests=true" in calls[0]


def test_run_smoke_runs_and_propagates_exit_code_when_matched(monkeypatch):
    module = load_smoke_module()
    calls = []

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        if "--benchmark_list_tests=true" in cmd:
            return _CompletedStub(stdout="BM_Real/32\n")
        return _CompletedStub(returncode=7)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    rc = module.run_smoke(
        "bin/bm", ["--benchmark_filter=BM_Real/32$", "--benchmark_min_time=0.001s"]
    )

    assert rc == 7
    assert len(calls) == 2
    # The real run forwards the smoke args verbatim and excludes the list flag.
    assert calls[1] == [
        "bin/bm",
        "--benchmark_filter=BM_Real/32$",
        "--benchmark_min_time=0.001s",
    ]


def test_describe_filter_reports_filter_or_placeholder():
    module = load_smoke_module()

    assert module._describe_filter(["--benchmark_filter=BM_X$"]) == "BM_X$"
    assert module._describe_filter(["--benchmark_min_time=1ms"]).startswith("<no")


def test_main_forwards_remainder_args(monkeypatch):
    module = load_smoke_module()
    seen = {}

    def fake_run_smoke(binary, run_args):
        seen["binary"] = binary
        seen["run_args"] = list(run_args)
        return 0

    monkeypatch.setattr(module, "run_smoke", fake_run_smoke)

    rc = module.main(
        ["bin/bm", "--benchmark_filter=BM_X$", "--", "--benchmark_min_time=1ms"]
    )

    assert rc == 0
    assert seen["binary"] == "bin/bm"
    # The bare "--" separator is stripped; benchmark flags pass through.
    assert seen["run_args"] == [
        "--benchmark_filter=BM_X$",
        "--benchmark_min_time=1ms",
    ]


def test_real_subprocess_run_signature_is_available():
    # Guard against importing a stubbed subprocess module.
    assert subprocess.run is not None
