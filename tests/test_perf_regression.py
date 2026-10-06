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
