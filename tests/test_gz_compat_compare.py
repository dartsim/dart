"""Tests for the Gazebo compatibility lane failure comparison."""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "gazebo" / "compat" / "compare_failures.py"

STEP_WORLD = "COMMON_TEST_simulation_features_dartsim"


def _load():
    spec = importlib.util.spec_from_file_location("compare_failures", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


compare_failures = _load()


def _write_run(results, suite, tests, log=""):
    """Writes one suite's CTest JUnit, GoogleTest XML and CTest log.

    tests maps a CTest test to {gtest case: (seconds, failed)}, or to a
    (CTest failure message, cases or None) pair for a test that crashed or
    timed out, leaving no XML (None) or XML from an earlier attempt.
    """
    junit = []
    gtest_dir = results / f"{suite}-gtest"
    gtest_dir.mkdir(parents=True, exist_ok=True)
    for test, outcome in tests.items():
        if isinstance(outcome, tuple):
            message, cases = outcome
        else:
            cases = outcome
            failed = any(f for _, f in cases.values())
            message = "Failed" if failed else None
        status = "run" if message is None else "fail"
        failure = f'<failure message="{message}"/>' if message else ""
        junit.append(
            f'<testcase name="{test}" classname="{test}" status="{status}">'
            f"{failure}</testcase>"
        )
        if cases is None:
            continue
        gtest_cases = []
        for case, (seconds, case_failed) in cases.items():
            classname, name = case.rsplit(".", 1)
            body = '<failure message="boom"/>' if case_failed else ""
            gtest_cases.append(
                f'<testcase name="{name}" classname="{classname}" status="run" '
                f'result="completed" time="{seconds}">{body}</testcase>'
            )
        (gtest_dir / f"{test}.xml").write_text(
            "<testsuites><testsuite>"
            + "".join(gtest_cases)
            + "</testsuite></testsuites>"
        )
    (results / f"{suite}.junit.xml").write_text(
        "<testsuite>" + "".join(junit) + "</testsuite>"
    )
    (results / f"{suite}.log").write_text(log)


def _write_lane(results, physics, sim, sim_log=""):
    _write_run(results, "gz-physics", physics)
    _write_run(results, "gz-sim", sim, sim_log)


def _attempts(test, *attempts):
    """CTest log lines for the attempts of a test run with --repeat.

    Each attempt is a CTest status, or a (status, failing cases) pair whose
    cases the attempt's output reports the way GoogleTest does.
    """
    lines = ""
    for number, attempt in enumerate(attempts):
        status, cases = attempt if isinstance(attempt, tuple) else (attempt, [])
        prefix = "1/1 " if number == 0 else "    "
        # CTest prints a crash outside its named categories without the
        # leading marker: "Subprocess aborted***Exception:".
        mark = "   " if status == "Passed" else "" if "***" in status else "***"
        lines += f"{prefix}Test #7: {test} ........{mark}{status}    1.00 sec\n"
        lines += "".join(f"[  FAILED  ] {case} (5 ms)\n" for case in cases)
        if cases:
            lines += f"[  FAILED  ] {len(cases)} test, listed below:\n"
    return lines


def _compare(results, expected, *extra):
    return compare_failures.main(
        ["--results", str(results), "--expected", str(expected), *extra]
    )


def _step_world(seconds):
    return {STEP_WORLD: {"Step/0.StepWorld": (seconds, False)}}


def test_baseline_lists_cases_and_only_caseless_failures_by_test(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text(
        f"gz-physics {STEP_WORLD} Ray.Unsupported  # accepted: upstream test update\n"
    )
    baseline = tmp_path / "baseline"
    _write_lane(
        baseline,
        {
            STEP_WORLD: {
                "Step/0.StepWorld": (2.7, False),
                "Ray.Ok": (0.1, False),
            },
            "COMMON_TEST_simulation_features_bullet": {
                "Step/0.StepWorld": (0.1, False)
            },
        },
        {
            "INTEGRATION_log_system": ("SEGFAULT", None),
            "INTEGRATION_user_commands": {
                "UserCommandsTest.Create": (1.0, True),
                "UserCommandsTest.Remove": (1.0, False),
            },
            "INTEGRATION_imu": {"Imu.Rotating": (1.0, False)},
        },
    )
    assert _compare(baseline, expected, "--write-baseline") == 0
    text = expected.read_text()
    assert "# accepted: upstream test update" in text
    assert "gz-sim INTEGRATION_log_system  # SEGFAULT\n" in text
    assert "gz-sim INTEGRATION_user_commands UserCommandsTest.Create\n" in text
    # A test explained by its failing cases gets no test-level entry, which
    # would accept any later crash of that test.
    assert "gz-sim INTEGRATION_user_commands\n" not in text
    assert f"max-seconds gz-physics {STEP_WORLD} Step/0.StepWorld 5.4\n" in text
    # Only DART's own StepWorld case gets a time limit.
    assert "COMMON_TEST_simulation_features_bullet" not in text

    # Fixing a 6.19.4 failure, hitting an accepted difference, and repeating
    # a 6.19.4 crash all pass, and the accepted difference stays visible.
    candidate = tmp_path / "candidate"
    _write_lane(
        candidate,
        {
            STEP_WORLD: {
                "Step/0.StepWorld": (2.3, False),
                "Ray.Unsupported": (0.1, True),
            }
        },
        {
            "INTEGRATION_log_system": ("SEGFAULT", None),
            "INTEGRATION_user_commands": {
                "UserCommandsTest.Create": (1.0, False),
                "UserCommandsTest.Remove": (1.0, False),
            },
        },
    )
    assert _compare(candidate, expected) == 0
    out = capsys.readouterr().out
    assert "FIXED      gz-sim INTEGRATION_user_commands UserCommandsTest.Create" in out
    assert (
        f"ACCEPTED   gz-physics {STEP_WORLD} Ray.Unsupported (upstream test update)"
        in out
    )
    assert "PASS" in out


def test_crash_or_timeout_is_new_even_if_cases_failed_on_6194(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text("gz-sim INTEGRATION_user_commands UserCommandsTest.Create\n")
    for name, outcome in {
        # Crashed without writing XML.
        "crash": ("SEGFAULT", None),
        # Timed out on the retry; the XML is the first attempt's, in which
        # the expected failure passed.
        "timeout": ("Timeout", {"UserCommandsTest.Create": (1.0, False)}),
        # gz-cmake's check_ test wrote a failure in place of the missing XML.
        "check": ("SEGFAULT", {"INTEGRATION_user_commands.test_ran": (1.0, True)}),
    }.items():
        candidate = tmp_path / name
        _write_lane(candidate, {}, {"INTEGRATION_user_commands": outcome})
        assert _compare(candidate, expected) == 1, name
        out = capsys.readouterr().out
        message = outcome[0]
        assert f"NEW        gz-sim INTEGRATION_user_commands ({message})\n" in out
        # Its case results are missing or stale, so nothing is FIXED or NEW.
        assert "FIXED" not in out and "test_ran" not in out


def test_every_attempt_is_read_from_the_ctest_log(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text("gz-sim INTEGRATION_user_commands UserCommandsTest.Create\n")
    test = "INTEGRATION_user_commands"
    # The last attempt, the only one the JUnit report and the GoogleTest XML
    # keep, failed just the expected case, or passed.
    expected_only = {test: {"UserCommandsTest.Create": (1.0, True)}}
    passing = {test: {"UserCommandsTest.Create": (1.0, False)}}
    for first, last, sim in [
        ("Exception: SegFault", "Failed", expected_only),
        ("Timeout", "Failed", expected_only),
        ("Exception: SegFault", "Passed", passing),
    ]:
        candidate = tmp_path / f"{first}-{last}"
        _write_lane(candidate, {}, sim, _attempts(test, first, last))
        assert _compare(candidate, expected) == 1, first
        out = capsys.readouterr().out
        assert f"NEW        gz-sim {test} ({first}, then {last})\n" in out
        assert "FIXED" not in out and "FLAKY" not in out
    # An abort reads like the crashes CTest names.
    candidate = tmp_path / "abort"
    log = _attempts(test, "Subprocess aborted***Exception:", "Failed")
    _write_lane(candidate, {}, expected_only, log)
    assert _compare(candidate, expected) == 1
    assert (
        f"NEW        gz-sim {test} (Exception: Subprocess aborted, then Failed)\n"
        in capsys.readouterr().out
    )

    # Two normal failures: a case that failed only on the first attempt is
    # listed from the CTest log without failing the lane, like a test that
    # passes on its retry; the same failures twice need no line.
    create = ("Failed", ["UserCommandsTest.Create"])
    for first, line in [
        (
            ("Failed", ["UserCommandsTest.Create", "UserCommandsTest.Remove"]),
            f"RETRIED    gz-sim {test} (Failed, then Failed; only an earlier "
            "attempt failed UserCommandsTest.Remove)\n",
        ),
        (
            "Failed",
            f"RETRIED    gz-sim {test} (Failed, then Failed; an earlier attempt's "
            "output names no failing case)\n",
        ),
        (create, None),
    ]:
        candidate = tmp_path / "failed-failed"
        _write_lane(candidate, {}, expected_only, _attempts(test, first, create))
        assert _compare(candidate, expected) == 0
        out = capsys.readouterr().out
        assert line in out if line else "RETRIED" not in out


def test_failure_without_a_failing_case_needs_a_test_level_entry(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text("gz-sim INTEGRATION_events  # Timeout\n")
    candidate = tmp_path / "candidate"
    _write_lane(
        candidate,
        {},
        {
            "INTEGRATION_events": ("Timeout", None),
            "INTEGRATION_quiet": {"Quiet.Case": (1.0, False)},
            # The executable failed after its cases passed.
            "INTEGRATION_exit": ("Failed", {"Exit.Case": (1.0, False)}),
        },
    )
    assert _compare(candidate, expected) == 1
    out = capsys.readouterr().out
    assert "NEW        gz-sim INTEGRATION_exit (failed without a failing case)\n" in out
    assert "INTEGRATION_events" not in out


def test_new_failures_and_slow_cases_fail(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text(f"max-seconds gz-physics {STEP_WORLD} Step/0.StepWorld 5.4\n")
    candidate = tmp_path / "candidate"
    _write_lane(
        candidate,
        _step_world(43.5),
        {"INTEGRATION_imu": {"Imu.Rotating": (1.0, True)}},
    )
    assert _compare(candidate, expected) == 1
    out = capsys.readouterr().out
    assert "NEW        gz-sim INTEGRATION_imu Imu.Rotating\n" in out
    assert (
        f"SLOW       gz-physics {STEP_WORLD} Step/0.StepWorld took 43.50 s "
        "(limit 5.40 s)\n" in out
    )

    # A slower host can scale the limits.
    assert _compare(candidate, expected, "--max-seconds-scale", "10") == 1
    assert "SLOW" not in capsys.readouterr().out


def test_base_results_gate_only_failures_the_candidate_introduces(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text(
        "gz-sim INTEGRATION_entity Entity.Cmd\n"
        f"gz-physics {STEP_WORLD} Ray.Unsupported  # accepted: upstream test update\n"
        f"max-seconds gz-physics {STEP_WORLD} Step/0.StepWorld 5.4\n"
    )
    base = tmp_path / "base"
    _write_lane(
        base,
        {
            STEP_WORLD: {
                "Step/0.StepWorld": (48.0, False),
                "Ray.Unsupported": (0.1, False),
            }
        },
        {
            "INTEGRATION_imu": {"Imu.Rotating": (1.0, True)},
            "INTEGRATION_entity": {"Entity.Cmd": (1.0, False)},
        },
    )
    candidate = tmp_path / "candidate"
    _write_lane(
        candidate,
        {
            STEP_WORLD: {
                "Step/0.StepWorld": (47.0, False),
                "Ray.Unsupported": (0.1, True),
            }
        },
        {
            "INTEGRATION_imu": {"Imu.Rotating": (1.0, True)},
            "INTEGRATION_entity": {"Entity.Cmd": (1.0, True)},
        },
    )
    assert _compare(candidate, expected, "--base-results", str(base)) == 0
    out = capsys.readouterr().out
    assert (
        "BASE       gz-sim INTEGRATION_imu Imu.Rotating (also fails on the base)" in out
    )
    assert "BASE SLOW  gz-physics" in out and "base 48.00 s" in out
    assert (
        "REGRESSED  gz-sim INTEGRATION_entity Entity.Cmd (passes on the base; also "
        "fails on DART 6.19.4)\n" in out
    )
    # DART 6.19.4 passes an accepted difference.
    assert (
        f"REGRESSED  gz-physics {STEP_WORLD} Ray.Unsupported (passes on the base)\n"
        in out
    )
    # Against DART 6.19.4 alone the same run fails.
    assert _compare(candidate, expected) == 1
    capsys.readouterr()

    # A failure the base does not have, and a further 2x slowdown, still gate.
    _write_lane(
        candidate,
        _step_world(100.0),
        {
            "INTEGRATION_imu": {"Imu.Rotating": (1.0, True)},
            "INTEGRATION_new": {"New.Case": (1.0, True)},
        },
    )
    assert _compare(candidate, expected, "--base-results", str(base)) == 1
    out = capsys.readouterr().out
    assert "NEW        gz-sim INTEGRATION_new New.Case\n" in out
    assert "SLOW       gz-physics" in out and "base 48.00 s" in out


def test_a_base_failure_covers_only_the_same_failure(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text("gz-sim INTEGRATION_user_commands UserCommandsTest.Create\n")
    test = "INTEGRATION_user_commands"
    candidate = tmp_path / "candidate"
    _write_lane(
        candidate,
        {},
        {
            test: {
                "UserCommandsTest.Create": (1.0, True),
                "UserCommandsTest.Remove": (1.0, True),
            }
        },
    )
    for name, outcome in {
        # The base crashed without XML, or timed out leaving an XML in which
        # Remove passed: Remove is not a base failure, and the base did not
        # pass Create, so neither is BASE or REGRESSED.
        "crash": ("SEGFAULT", None),
        "stale": ("Timeout", {"UserCommandsTest.Remove": (1.0, False)}),
        # The base did not run the test.
        "absent": None,
    }.items():
        base = tmp_path / name
        _write_lane(base, {}, {test: outcome} if outcome else {})
        assert _compare(candidate, expected, "--base-results", str(base)) == 1
        out = capsys.readouterr().out
        assert f"NEW        gz-sim {test} UserCommandsTest.Remove\n" in out, name
        assert "BASE" not in out and "REGRESSED" not in out, name


def test_retries_and_stale_accepted_entries_are_reported(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text(
        f"gz-physics {STEP_WORLD} Ray.Unsupported  # accepted: upstream test update\n"
    )
    candidate = tmp_path / "candidate"
    _write_lane(
        candidate,
        {STEP_WORLD: {"Ray.Unsupported": (0.1, False)}},
        {"INTEGRATION_reset": {"Reset.Case": (1.0, False)}},
        sim_log=_attempts("INTEGRATION_reset", ("Failed", ["Reset.Case"]), "Passed"),
    )
    assert _compare(candidate, expected) == 0
    out = capsys.readouterr().out
    assert (
        "FLAKY      gz-sim INTEGRATION_reset (Failed, then Passed; only an earlier "
        "attempt failed Reset.Case)\n" in out
    )
    assert (
        f"STALE      gz-physics {STEP_WORLD} Ray.Unsupported "
        "(accepted failure now passes)\n" in out
    )


def test_missing_results_is_an_error(tmp_path):
    assert _compare(tmp_path, tmp_path / "x.txt") == 2


def test_unreadable_results_are_an_error_naming_the_file(tmp_path, capsys):
    _write_lane(tmp_path, _step_world(2.7), {})
    # A GoogleTest XML that another process wrote over while it was written.
    xml = tmp_path / "gz-physics-gtest" / f"{STEP_WORLD}.xml"
    xml.write_text(xml.read_text() + "</testsuite>")
    assert _compare(tmp_path, tmp_path / "x.txt") == 2
    assert f"error: {xml}: " in capsys.readouterr().err
