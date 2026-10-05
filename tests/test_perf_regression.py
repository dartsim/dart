"""One regression check for performance classification and rationale coverage."""

import copy
import importlib.util
from pathlib import Path


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
