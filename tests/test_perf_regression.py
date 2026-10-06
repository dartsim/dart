"""Regression checks for performance comparison and local execution."""

import copy
import importlib.util
from pathlib import Path

import pytest


def test_compare_classification_and_thresholds():
    path = Path(__file__).resolve().parents[1] / "scripts/perf_regression.py"
    spec = importlib.util.spec_from_file_location("perf_regression", path)
    module = importlib.util.module_from_spec(spec)
    # Dataclasses resolve their defining module when importing a standalone script.
    import sys

    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    env = {
        "valgrind": "3.22.0",
        "compiler": "test",
        "glibc": "2.39",
        "preset": "perf-1",
        "fingerprint": "test",
    }
    metric = {
        "ir_per_step": 100_000,
        "allocs_per_step": 10,
        "guards": {
            "hash": "0x1",
            "contacts": 1,
            "finite": True,
            "cap_hit": True,
            "resting": "0/1",
        },
    }
    base = {
        "schema": "dart-perf/1",
        "run": {"commit": "base", "env": env},
        "results": [
            {
                "row": "a",
                "det": "dart",
                "version": 1,
                "gated": True,
                "status": "ok",
                "method": "slope",
                "head": metric,
            }
        ],
    }

    def check(
        *, ir=100_000, allocs=10, body="", changed=False, gated=True, status="ok"
    ):
        head = copy.deepcopy(base)
        row = head["results"][0]
        row.update(gated=gated, status=status)
        row["head"].update(ir_per_step=ir, allocs_per_step=allocs)
        if changed:
            row["head"]["guards"]["hash"] = "0x2"
        return module.compare(base, head, body)

    same = check()
    assert same["verdict"]["status"] == "PASS"
    assert same["results"][0]["delta"] == {
        "ir": 0,
        "allocs": 0,
        "guards_equal": True,
        "class": "gated",
    }
    assert check(ir=100_300)["verdict"]["status"] == "WARN"
    assert check(ir=100_500)["verdict"]["status"] == "FAIL"  # geomean boundary
    assert check(ir=101_000)["verdict"]["status"] == "FAIL"  # per-row boundary
    assert check(ir=99_000)["verdict"]["status"] == "PASS"
    assert check(allocs=11)["verdict"]["status"] == "FAIL"
    assert check(allocs=9)["verdict"]["status"] == "PASS"
    assert (
        check(
            ir=102_000,
            allocs=11,
            body="Perf-Regression-Rationale: a/dart: intended work",
        )["verdict"]["status"]
        != "FAIL"
    )
    assert (
        check(ir=102_000, body="Perf-Regression-Rationale: a/ode: intended work")[
            "verdict"
        ]["status"]
        == "FAIL"
    )
    assert check(changed=True)["results"][0]["delta"]["class"] == "behaviour-change"
    assert check(changed=True)["verdict"]["status"] == "FAIL"
    assert (
        check(
            changed=True,
            ir=101_000,
            body="Rebaseline-Rationale: a/dart: intended behaviour",
        )["verdict"]["status"]
        == "PASS"
    )
    for percentage in ("", "2.0%", "-2%", "+2.0%"):
        record = check(
            changed=True,
            ir=102_000,
            body=f"Rebaseline-Rationale: a/dart: {percentage} Ir; intended behaviour",
        )
        assert (record["verdict"]["status"] == "PASS") == percentage.startswith(
            ("+", "-")
        )
        assert record["verdict"]["ir_geomean"] is None
    assert check(status="broken")["results"][0]["delta"]["class"] == "broken"
    assert check(gated=False)["verdict"]["status"] == "FAIL"  # head cannot weaken base
    diagnostic = copy.deepcopy(base)
    diagnostic["results"][0]["gated"] = False
    assert (
        module.compare(diagnostic, diagnostic)["results"][0]["delta"]["class"]
        == "diagnostic"
    )
    new = copy.deepcopy(base)
    new["results"][0]["version"] = 2
    assert module.compare(base, new)["results"][0]["delta"]["class"] == "new"
    assert module.compare({**base, "results": []}, new)["verdict"]["status"] == "PASS"
    missing = copy.deepcopy(base)
    missing["results"][0]["head"].pop("ir_per_step")
    assert module.compare(base, missing)["verdict"]["status"] == "FAIL"
    missing["results"][0].update(expected_ir=False, method="native")
    missing["results"][0]["head"]["guards"]["hash"] = "0x2"
    assert (
        module.compare(
            base, missing, "Rebaseline-Rationale: a/dart: intended behaviour"
        )["verdict"]["status"]
        == "FAIL"
    )
    missing["results"] = []
    assert module.compare(base, missing)["verdict"]["status"] == "FAIL"
    nonfinite = copy.deepcopy(base)
    nonfinite["results"][0]["head"]["guards"]["finite"] = False
    assert (
        module.compare(
            base, nonfinite, "Rebaseline-Rationale: a/dart: intended behaviour"
        )["verdict"]["status"]
        == "FAIL"
    )
    multiple = copy.deepcopy(base)
    second = copy.deepcopy(multiple["results"][0])
    second["row"] = "b"
    multiple["results"].append(second)
    head = copy.deepcopy(multiple)
    for row in head["results"]:
        row["head"]["ir_per_step"] = 100_600
    assert (
        module.compare(
            multiple, head, "Perf-Regression-Rationale: a/dart: intended work"
        )["verdict"]["status"]
        == "FAIL"
    )
    assert (
        module.compare(
            multiple, head, "Perf-Regression-Rationale: a/dart, b/dart: intended work"
        )["verdict"]["status"]
        != "FAIL"
    )
    # A changed row never hides an unchanged row's regression.
    head["results"][0]["head"]["guards"]["hash"] = "0x2"
    assert (
        module.compare(
            multiple, head, "Rebaseline-Rationale: a/dart: intended behaviour"
        )["verdict"]["status"]
        == "FAIL"
    )
    assert (
        module.compare(
            nonfinite, base, "Rebaseline-Rationale: a/dart: intended behaviour"
        )["verdict"]["status"]
        == "FAIL"
    )
    empty = {**base, "results": []}
    assert module.compare(empty, empty)["verdict"]["status"] == "FAIL"
    assert "Wall time" in module.markdown(same)
    assert "-0.0001%" in module.markdown(check(ir=99_999.9))

    # Environment incompatibility rejects the entire comparison before deltas.
    for key in ("compiler", "valgrind", "pixi_lock_sha", "host_cpu", "harness_sha"):
        head = copy.deepcopy(base)
        head["run"]["env"].update({key: "different", "fingerprint": "different"})
        record = module.compare(base, head)
        assert record["verdict"]["status"] == "ERROR"
        assert record["results"] == []
        assert key in module.markdown(record)
    for value in (None, "different"):
        head = copy.deepcopy(base)
        head["run"]["env"]["fingerprint"] = value
        assert module.compare(base, head)["verdict"]["status"] == "ERROR"

    multiple["results"][1]["threads"] = 4
    report = module.markdown(module.compare(multiple, multiple))
    assert "1/4 threads" in report
    assert "| b/dart | 4 |" in report
    assert "base perturbation check passed" in report
    report = module.markdown(module.compare(diagnostic, diagnostic))
    assert "perturbation check not run" in report
    unstable = copy.deepcopy(diagnostic)
    unstable["results"][0]["perturbations"] = {"start4k": {"stable": False}}
    for parent, child in ((unstable, diagnostic), (diagnostic, unstable)):
        record = module.compare(parent, child)
        assert record["verdict"]["status"] == "FAIL"
        assert "perturbation check failed" in module.markdown(record)

    infrastructure = copy.deepcopy(base)
    infrastructure["results"][0].update(
        status="broken", error_kind="infrastructure", error="binary missing"
    )
    for parent, child in ((base, infrastructure), (infrastructure, base)):
        record = module.compare(parent, child)
        assert record["verdict"]["status"] == "ERROR"
        assert "binary missing" in module.markdown(record)


def _load_runner():
    import sys

    path = Path(__file__).resolve().parents[1] / "scripts/perf_regression.py"
    spec = importlib.util.spec_from_file_location("perf_regression", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("version", [1, 2])
def test_changed_input_requires_rebaseline_without_deltas(version):
    module = _load_runner()
    base = {
        "run": {"commit": "base", "env": {"fingerprint": "same"}},
        "results": [
            {
                "row": "pend",
                "det": "dart",
                "version": 1,
                "input_sha": "base input",
                "gated": True,
                "status": "ok",
                "method": "slope",
                "head": {
                    "ir_per_step": 100,
                    "allocs_per_step": 0,
                    "max_rss_kb": 100,
                    "guards": {
                        "hash": "same",
                        "finite": True,
                        "contacts": 0,
                        "cap_hit": False,
                        "resting": "0/1",
                    },
                },
            }
        ],
    }
    # Report metadata is identical across arms, as is the environment fingerprint.
    base["run"]["env"].update(
        valgrind="test", compiler="test", glibc="test", preset="perf-1"
    )
    head = copy.deepcopy(base)
    head["results"][0]["version"] = version
    head["results"][0]["input_sha"] = "head input"
    head["results"][0]["head"].update(
        ir_per_step=200, allocs_per_step=10, max_rss_kb=200
    )
    for body, status in (
        ("", "FAIL"),
        ("Perf-Regression-Rationale: pend/dart: changed input", "FAIL"),
        ("Rebaseline-Rationale: robot/dart: changed input", "FAIL"),
        ("Rebaseline-Rationale: pend/dart: changed input", "PASS"),
    ):
        record = module.compare(base, head, body)
        assert record["verdict"]["status"] == status
        assert record["verdict"]["ir_geomean"] is None
        assert record["verdict"]["warnings"] == []
        assert record["results"][0]["delta"] == {
            "ir": None,
            "allocs": None,
            "guards_equal": True,
            "class": "behaviour-change",
        }
        assert "input_sha changed" in module.markdown(record)


def test_local_resets_only_marked_default_output(monkeypatch, tmp_path):
    module = _load_runner()
    monkeypatch.setattr(module, "ROOT", tmp_path)
    monkeypatch.delenv("CONDA_PREFIX", raising=False)
    output = tmp_path / "build/perf-compare"
    args = module.parser().parse_args(["local"])
    # Stop before building; initialization still marks the default directory.
    for _ in range(2):
        with pytest.raises(ValueError, match="active Pixi environment"):
            module.local_arms(args)
        assert (output / ".perf-compare-owned").is_file()
        assert not (output / "stale").exists()
        (output / "stale").write_text("previous run", encoding="utf-8")
    (output / ".perf-compare-owned").unlink()
    with pytest.raises(ValueError, match="must be empty"):
        module.local_arms(args)
    assert (output / "stale").read_text(encoding="utf-8") == "previous run"
    (output / ".perf-compare-owned").symlink_to(output / "stale")
    with pytest.raises(ValueError, match="must be empty"):
        module.local_arms(args)
    assert (output / "stale").is_file()
    (output / ".perf-compare-owned").unlink()
    (output / ".perf-compare-owned").write_text("dart-perf/1\n", encoding="utf-8")
    alias = tmp_path / "alias"
    alias.symlink_to(output, target_is_directory=True)
    args.output_dir = alias
    with pytest.raises(ValueError, match="must be empty"):
        module.local_arms(args)
    assert (output / "stale").is_file()

    custom = tmp_path / "custom"
    custom.mkdir()
    (custom / ".perf-compare-owned").write_text("dart-perf/1\n", encoding="utf-8")
    args.output_dir = custom
    with pytest.raises(ValueError, match="must be empty"):
        module.local_arms(args)
    assert (custom / ".perf-compare-owned").is_file()


def test_run_requires_installed_commit():
    module = _load_runner()
    command = ["run", "--prefix", "/install", "--output-dir", "/output"]
    with pytest.raises(SystemExit) as error:
        module.parser().parse_args(command)
    assert error.value.code == 2
    assert (
        module.parser().parse_args([*command, "--commit", "installed"]).commit
        == "installed"
    )
    assert module.parser().parse_args(["local"]).head == "HEAD"


def test_measure_gates_on_recorded_or_current_perturbation_pass(monkeypatch, tmp_path):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--native-only",
        ]
    )
    row = module.select_rows("pend")[0]
    metrics = {"guards": {"finite": True}, "allocs": 0}
    monkeypatch.setattr(module, "native", lambda *args: copy.deepcopy(metrics))
    # A recorded row gates without rerunning the perturbations; others do not.
    assert row.key in module.QUALIFIED_ROWS
    assert module.measure(row, args, tmp_path)["gated"] is True
    monkeypatch.setattr(module, "QUALIFIED_ROWS", frozenset())
    assert module.measure(row, args, tmp_path)["gated"] is False
    args.perturb = True
    measured = module.measure(row, args, tmp_path)
    assert measured["gated"] is True
    assert set(measured["perturbations"]) == set(module.PERTURBATIONS)
    monkeypatch.setattr(
        module,
        "native",
        lambda row, args, world, config="": {**metrics, "allocs": int(bool(config))},
    )
    assert module.measure(row, args, tmp_path)["gated"] is False


@pytest.mark.parametrize(
    "error",
    [OSError("binary missing"), ValueError("timeout"), ValueError("Valgrind failed")],
)
def test_execution_errors_exit_two_with_reason(monkeypatch, tmp_path, capsys, error):
    module = _load_runner()
    argv = [
        "run",
        "--commit",
        "HEAD",
        "--prefix",
        str(tmp_path),
        "--output-dir",
        str(tmp_path),
    ]
    args = module.parser().parse_args(argv)

    def fail(*args):
        raise error

    monkeypatch.setattr(module, "native", fail)
    broken = module.measure(module.select_rows("pend")[0], args, tmp_path)
    assert broken["status"] == "broken"
    assert broken["error_kind"] == "infrastructure"
    assert broken["error"] == str(error)
    monkeypatch.setattr(module, "run_arm", lambda args: {"results": [broken]})
    assert module.main(argv) == 2
    assert str(error) in capsys.readouterr().out
    broken.pop("error_kind")
    assert module.main(argv) == 1


def test_compare_and_local_infrastructure_exit_two(monkeypatch, tmp_path, capsys):
    module = _load_runner()
    env = {
        "fingerprint": "same",
        "valgrind": "test",
        "compiler": "test",
        "glibc": "test",
        "preset": "perf-1",
    }
    record = {
        "schema": "dart-perf/1",
        "run": {"commit": "HEAD", "env": env},
        "results": [
            {
                "row": "pend",
                "det": "dart",
                "status": "broken",
                "method": "native",
                "error_kind": "infrastructure",
                "error": "timeout",
                "head": {},
            }
        ],
    }
    monkeypatch.setattr(module, "local_arms", lambda args: (record, record))
    assert module.main(["local"]) == 2
    assert "timeout" in capsys.readouterr().out
    base, head = tmp_path / "base.json", tmp_path / "head.json"
    module.write_json(base, record)
    altered = copy.deepcopy(record)
    altered["run"]["env"].update(fingerprint="different", compiler="other")
    module.write_json(head, altered)
    assert module.main(["compare", "--base", str(base), "--head", str(head)]) == 2
    assert "compiler" in capsys.readouterr().out


def _micro_record(module, name):
    row = module.select_rows(name)[0]
    cases = (
        ["solveNative/boxed_coupled_96", "solveNative/friction_32"]
        if name == "lcp"
        else ["BM_Dynamics/10"]
    )
    return {
        "run": {"commit": "HEAD", "env": {"fingerprint": "same"}},
        "results": [
            {
                "row": name,
                "det": "",
                "version": row.version,
                "status": "ok",
                "gated": True,
                "method": "slope",
                "head": {
                    "ir_per_step": 100,
                    "allocs_per_step": 0,
                    "cases": cases,
                    "guards": {
                        "hash": {case: "0x0123456789abcdef" for case in cases},
                        "finite": True,
                    },
                },
            }
        ],
    }


@pytest.mark.parametrize("name", ["dyn", "lcp"])
def test_micro_checksums_and_allocations_gate_comparison(name):
    module = _load_runner()
    base = _micro_record(module, name)
    same = module.compare(base, copy.deepcopy(base))
    assert same["verdict"]["status"] == "PASS"
    assert same["results"][0]["delta"]["class"] == "gated"
    assert same["results"][0]["delta"]["guards_equal"] is True
    for case in base["results"][0]["head"]["cases"]:
        head = copy.deepcopy(base)
        head["results"][0]["head"]["guards"]["hash"][case] = "0xfedcba9876543210"
        record = module.compare(base, head)
        assert record["verdict"]["status"] == "FAIL"
        assert record["results"][0]["delta"]["class"] == "behaviour-change"
        assert record["results"][0]["delta"]["guards_equal"] is False
    head = copy.deepcopy(base)
    head["results"][0]["head"]["allocs_per_step"] = 1
    record = module.compare(base, head)
    assert record["verdict"]["status"] == "FAIL"
    assert "allocations +1/step" in record["verdict"]["failures"][0]


@pytest.mark.parametrize("name", ["dyn", "lcp"])
@pytest.mark.parametrize("missing", ["base", "head", "both"])
def test_micro_instrumentation_availability(name, missing):
    module = _load_runner()
    base = _micro_record(module, name)
    head = copy.deepcopy(base)
    head["results"][0]["head"]["ir_per_step"] = 200
    for record in (
        [base, head] if missing == "both" else [base if missing == "base" else head]
    ):
        metrics = record["results"][0]["head"]
        metrics.update(
            micro_instrumented=False,
            guards=None,
            allocs=None,
            bytes=None,
            allocs_per_step=None,
            bytes_per_step=None,
        )
        assert module.complete(record["results"][0], metrics)
    result = module.compare(base, head, f"Rebaseline-Rationale: {name}: +100% intended")
    row = result["results"][0]
    assert row["delta"]["ir"] == 1
    assert row["delta"]["allocs"] is None
    assert row["delta"]["guards_equal"] is None
    assert result["verdict"]["ir_geomean"] is None
    if missing == "head":
        assert result["verdict"]["status"] == "FAIL"
        assert row["delta"]["class"] == "broken"
        assert row["failures"] == ["head lacks micro instrumentation present in base"]
        head["results"][0]["version"] += 1
        assert module.compare(base, head)["verdict"]["status"] == "FAIL"
    else:
        assert result["verdict"]["status"] == "PASS"
        assert row["delta"]["class"] == "diagnostic"
        assert row["gated"] is False
    for arm in (["base", "head"] if missing == "both" else [missing]):
        assert f"{arm} lacks micro instrumentation" in row["gate_reason"]
    result["run"]["env"].update(
        valgrind="test", compiler="test", glibc="test", preset="perf-1"
    )
    report = module.markdown(result)
    assert "unavailable" in report
    assert row["gate_reason"] in report


@pytest.mark.parametrize("name", ["dyn", "lcp"])
@pytest.mark.parametrize("arm", ["base", "head", "both"])
@pytest.mark.parametrize(
    "defect",
    [
        "missing allocation",
        "null allocation",
        "negative allocation",
        "missing guards",
        "null guards",
        "missing checksum",
        "partial checksum",
        "non-finite",
    ],
)
def test_micro_incomplete_metrics_never_pass(name, arm, defect):
    module = _load_runner()
    base = _micro_record(module, name)
    head = copy.deepcopy(base)
    for record in (
        [base, head] if arm == "both" else [base if arm == "base" else head]
    ):
        row = record["results"][0]
        metrics = row["head"]
        if defect == "missing allocation":
            metrics.pop("allocs_per_step")
        elif defect == "null allocation":
            metrics["allocs_per_step"] = None
        elif defect == "negative allocation":
            metrics["allocs_per_step"] = -1
        elif defect == "missing guards":
            metrics.pop("guards")
        elif defect == "null guards":
            metrics["guards"] = None
        elif defect == "missing checksum":
            metrics["guards"].pop("hash")
        elif defect == "partial checksum":
            metrics["guards"]["hash"].pop(metrics["cases"][0])
        else:
            metrics["guards"]["finite"] = False
        assert not module.complete(row, metrics)
    result = module.compare(base, head, f"Rebaseline-Rationale: {name}: intended")
    assert result["verdict"]["status"] == "FAIL"
    assert result["results"][0]["delta"]["class"] == "broken"


@pytest.mark.parametrize("name", ["dyn", "lcp"])
def test_micro_native_slope_and_explicit_windows(monkeypatch, tmp_path, name):
    module = _load_runner()
    row = module.select_rows(name)[0]
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--native-only",
        ]
    )
    args.bin_dir = tmp_path
    cases = _micro_record(module, name)["results"][0]["head"]["cases"]
    defect = ""

    def execute(command, env, log, timeout):
        assert env["PERF_WINDOW"] == "micro"
        assert env["PERF_MICRO"] == "1"
        assert env["PERF_WARMUP"] == "0"
        steps = int(
            next(
                item for item in command if item.startswith("--benchmark_min_time=")
            ).split("=")[1][:-1]
        )
        output = Path(
            next(
                item.split("=", 1)[1]
                for item in command
                if item.startswith("--benchmark_out=")
            )
        )
        module.write_json(
            output,
            {"benchmarks": [{"name": case, "iterations": steps} for case in cases]},
        )
        measured = (
            len(cases) if defect not in ("missing window", "uninstrumented") else 0
        )
        allocs, size = (
            (0, 0) if defect == "uninstrumented" else (50 + 3 * steps, 200 + 12 * steps)
        )
        text = f"STEPALLOC steps={measured} measured={measured} allocs={allocs} bytes={size} libdart={tmp_path}/libdart.so\n"
        if defect not in ("missing checksum", "uninstrumented"):
            text += "".join(
                f"PERFGUARD case={case} hash=0x0123456789abcdef finite=true\n"
                for case in cases
            )
        if defect == "duplicate checksum":
            text += f"PERFGUARD case={cases[0]} hash=0x0123456789abcdef finite=true\n"
        log.write_text(text)
        return text

    monkeypatch.setattr(module, "execute", execute)
    metrics = module.micro_perturb(row, args, tmp_path, "")
    assert metrics["allocs"] == 3 * row.steps
    assert metrics["bytes"] == 12 * row.steps
    assert metrics["allocs_per_step"] == 3
    assert metrics["bytes_per_step"] == 12
    assert metrics["cases"] == cases
    assert metrics["guards"] == module.native_guards(args, row)
    defect = "uninstrumented"
    result = module.measure(row, args, tmp_path)
    assert result["status"] == "ok"
    metrics = result["head"]
    assert metrics["micro_instrumented"] is False
    assert metrics["cases"] == cases
    assert all(
        metrics[key] is None
        for key in ("guards", "allocs", "bytes", "allocs_per_step", "bytes_per_step")
    )
    assert module.native_guards(args, row) is None
    assert module.complete(result, metrics)
    for defect in ("missing window", "missing checksum", "duplicate checksum"):
        with pytest.raises(ValueError):
            module.micro_perturb(row, args, tmp_path, "")


@pytest.mark.parametrize("name", ["dyn", "lcp"])
@pytest.mark.parametrize("changed", ["hash", "allocs"])
def test_micro_perturbations_compare_real_metrics(monkeypatch, tmp_path, name, changed):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--native-only",
            "--perturb",
        ]
    )
    baseline = _micro_record(module, name)["results"][0]["head"]
    baseline["allocs"] = 0

    def measure(row, args, world, config):
        metrics = copy.deepcopy(baseline)
        if config:
            if changed == "hash":
                metrics["guards"]["hash"][metrics["cases"][0]] = "0xfedcba9876543210"
            else:
                metrics["allocs"] = 1
        return metrics

    monkeypatch.setattr(module, "micro_perturb", measure)
    result = module.measure(module.select_rows(name)[0], args, tmp_path)
    assert result["status"] == "ok"
    assert result["gated"] is False
    assert all(not sample["stable"] for sample in result["perturbations"].values())


def test_local_uses_independent_source_and_cmake_caches(monkeypatch, tmp_path):
    import io
    import tarfile
    from types import SimpleNamespace

    module = _load_runner()
    args = module.parser().parse_args(
        ["local", "--base", "HEAD", "--head", "HEAD", "--output-dir", str(tmp_path)]
    )
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / "dependencies"))
    monkeypatch.setattr(module, "command_output", lambda command: "HEAD")
    monkeypatch.setattr(
        module.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(returncode=0)
    )
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w") as contents:
        entry = tarfile.TarInfo("CMakeLists.txt")
        entry.size = 0
        contents.addfile(entry, io.BytesIO())
    monkeypatch.setattr(
        module.subprocess, "check_output", lambda *args, **kwargs: archive.getvalue()
    )
    configurations = []

    def execute(command, env, log, timeout):
        if command[:3] == ["cmake", "-G", "Ninja"]:
            source = Path(command[command.index("-S") + 1])
            build = Path(command[command.index("-B") + 1])
            build.mkdir()
            if source.name.startswith("src-"):
                assert (source / "CMakeLists.txt").is_file()
                assert not (build / "CMakeCache.txt").exists()
                (build / "CMakeCache.txt").write_text(source.name)
                configurations.append((source, build))
        elif command[:2] == ["cmake", "--build"]:
            build = Path(command[2])
            if build.name.startswith("driver-"):
                (build / "portable_step_bench").write_text("driver")
        elif command[:2] == ["cmake", "--install"]:
            Path(command[command.index("--prefix") + 1]).mkdir()
        return ""

    monkeypatch.setattr(module, "execute", execute)
    monkeypatch.setattr(module, "run_arm", lambda arm: {"commit": arm.commit})
    assert len(module.local_arms(args)) == 2
    assert configurations == [
        (tmp_path / f"src-{arm}", tmp_path / f"build-{arm}") for arm in ("a", "b")
    ]
    for source, build in configurations:
        assert (build / "CMakeCache.txt").read_text() == source.name


@pytest.mark.parametrize("name", ["dyn", "lcp"])
def test_micro_callgrind_requires_native_guard_and_library_parity(
    monkeypatch, tmp_path, name
):
    module = _load_runner()
    row = module.select_rows(name)[0]
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
        ]
    )
    args.bin_dir = tmp_path
    cases = module.MICRO_CASES[name]
    native_text = (
        f"STEPALLOC steps=1 measured=1 allocs=0 bytes=0 libdart={tmp_path}/libdart.so\n"
    )
    native_text += "".join(
        f"PERFGUARD case={case} hash=0x0123456789abcdef finite=true\n" for case in cases
    )
    module.native_log(args, row).write_text(native_text)
    defect = ""

    def execute(command, env, log, timeout):
        assert env["PERF_MICRO"] == "1"
        assert f"--toggle-collect={module.COLLECTION_SIGNATURES[row.driver]}" in command
        output = next(
            item.split("=", 1)[1]
            for item in command
            if item.startswith("--callgrind-out-file=")
        )
        library = "other/libdart.so" if defect == "library" else "libdart.so"
        Path(output.replace("%p", "123")).write_text(
            f"events: Ir\nsummary: 1000\nob={tmp_path}/{library}\n"
        )
        return (
            native_text.replace("0x0123456789abcdef", "0xfedcba9876543210")
            if defect == "checksum"
            else native_text
        )

    monkeypatch.setattr(module, "execute", execute)
    assert module.callgrind(row, args, tmp_path, row.warmup + row.steps) == {"Ir": 1000}
    for defect in ("checksum", "library"):
        with pytest.raises(ValueError, match="differs|differ"):
            module.callgrind(row, args, tmp_path, row.warmup + row.steps)
    defect = ""
    native_text = native_text.split("PERFGUARD", 1)[0]
    module.native_log(args, row).write_text(native_text)
    assert module.callgrind(row, args, tmp_path, row.warmup + row.steps) == {"Ir": 1000}
