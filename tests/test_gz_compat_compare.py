"""Tests for the Gazebo compatibility lane failure comparison."""

import importlib.util
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "gazebo" / "compat" / "compare_failures.py"
LANE = ROOT / "tools" / "gazebo" / "compat" / "lane.sh"

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
    # Like lane.sh, drop the XML of an earlier run into the same directory.
    shutil.rmtree(gtest_dir, ignore_errors=True)
    gtest_dir.mkdir(parents=True)
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
    assert "gz-sim INTEGRATION_user_commands" not in text.splitlines()
    assert f"max-seconds gz-physics {STEP_WORLD} Step/0.StepWorld 5.4\n" in text
    assert text.endswith(
        "# CTest tests of the baseline run (compare requires each one)\n"
        "test gz-physics COMMON_TEST_simulation_features_bullet\n"
        f"test gz-physics {STEP_WORLD}\n"
        "test gz-sim INTEGRATION_imu\n"
        "test gz-sim INTEGRATION_log_system\n"
        "test gz-sim INTEGRATION_user_commands\n"
    )
    # Only DART's own StepWorld case gets a time limit.
    assert "max-seconds gz-physics COMMON_TEST_simulation_features_bullet" not in text

    # Fixing a 6.19.4 failure, hitting an accepted difference, and repeating
    # a 6.19.4 crash all pass, and the accepted difference stays visible.
    candidate = tmp_path / "candidate"
    _write_lane(
        candidate,
        {
            STEP_WORLD: {
                "Step/0.StepWorld": (2.3, False),
                "Ray.Unsupported": (0.1, True),
            },
            "COMMON_TEST_simulation_features_bullet": {
                "Step/0.StepWorld": (0.1, False)
            },
        },
        {
            "INTEGRATION_log_system": ("SEGFAULT", None),
            "INTEGRATION_user_commands": {
                "UserCommandsTest.Create": (1.0, False),
                "UserCommandsTest.Remove": (1.0, False),
            },
            "INTEGRATION_imu": {"Imu.Rotating": (1.0, False)},
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
            "INTEGRATION_entity": {"Entity.Cmd": (1.0, True)},
            "INTEGRATION_new": {"New.Case": (1.0, True)},
        },
    )
    assert _compare(candidate, expected, "--base-results", str(base)) == 1
    out = capsys.readouterr().out
    assert "NEW        gz-sim INTEGRATION_new New.Case\n" in out
    assert "SLOW       gz-physics" in out and "base 48.00 s" in out


def test_a_crashed_base_leaves_no_base_time_for_a_slow_case(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text(f"max-seconds gz-physics {STEP_WORLD} Step/0.StepWorld 5.4\n")
    # The base's timed test crashed after an earlier attempt left 100 s of XML.
    base = tmp_path / "base"
    _write_lane(
        base, {STEP_WORLD: ("SEGFAULT", {"Step/0.StepWorld": (100.0, False)})}, {}
    )
    candidate = tmp_path / "candidate"
    _write_lane(candidate, _step_world(10.0), {})
    assert _compare(candidate, expected, "--base-results", str(base)) == 1
    out = capsys.readouterr().out
    assert f"SLOW       gz-physics {STEP_WORLD} Step/0.StepWorld took 10.00 s" in out
    assert "BASE SLOW" not in out


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


def test_the_lane_rejects_the_candidate_as_its_own_base(tmp_path):
    # compare would read the candidate's results as the base's, so every new
    # failure would be BASE.
    env = {k: v for k, v in os.environ.items() if not k.startswith("GZ_COMPAT_")}
    env.update(CONDA_PREFIX=str(tmp_path), GZ_COMPAT_DIR=str(tmp_path))
    for variant in ({}, {"GZ_COMPAT_VARIANT": "base"}):
        name = variant.get("GZ_COMPAT_VARIANT", "candidate")
        run = subprocess.run(
            ["bash", str(LANE), "ionic", "compare"],
            env={**env, **variant, "GZ_COMPAT_BASE_VARIANT": name},
            capture_output=True,
            text=True,
        )
        assert run.returncode == 2, name
        assert f"GZ_COMPAT_BASE_VARIANT names the candidate variant ({name})" in (
            run.stderr
        )


@pytest.mark.skipif(sys.platform != "linux", reason="compat lanes require Linux")
@pytest.mark.parametrize("pause", ["before-write", "during-write"])
def test_parallel_bench_variants_read_complete_worlds(tmp_path, pause):
    work = tmp_path / "work"
    (work / "src" / "gz-sim" / ".git").mkdir(parents=True)
    shim = tmp_path / "bin"
    shim.mkdir()
    (shim / "git").write_text(
        '#!/bin/bash\nif [ "$3" = describe ]; then echo gz-sim9_9.5.0; fi\n'
    )
    (shim / "cmake").write_text("#!/bin/bash\nexit 0\n")
    # Pause either before creating the file (both variants need generation)
    # or after opening it (a file exists, but is not ready for a consumer).
    (shim / "python").write_text(
        '#!/bin/bash\nmkdir -p "$3"\necho "$3" >> "$WORLD_CALLS"\n'
        'if [ "$WORLD_PAUSE" = during-write ]; then\n'
        '  echo partial > "$3/3k_shapes.sdf"\nfi\n'
        'touch "$WORLD_STARTED"\n'
        'if [ "$GZ_COMPAT_VARIANT" = base ]; then\n'
        '  touch "$WORLD_OBSERVED"\nelse\n'
        '  timeout 10 bash -c \'until [ -e "$WORLD_OBSERVED" ]; '
        "do sleep 0.01; done'\nfi\n"
        'echo complete > "$3/3k_shapes.sdf"\n'
    )
    for path in shim.iterdir():
        path.chmod(0o755)
    for variant in ("candidate", "base"):
        (work / variant / "gz-sim").mkdir(parents=True)
        bench = work / variant / "bench-build"
        bench.mkdir()
        driver = bench / "gz_sim_server_bench"
        driver.write_text(
            '#!/bin/bash\nworld="$(cat "$3")"\n'
            '[ "$GZ_COMPAT_VARIANT" != base ] || touch "$WORLD_OBSERVED"\n'
            '[ "$world" = complete ] || exit 1\n'
            'echo "read complete world"\n'
        )
        driver.chmod(0o755)
    env = {k: v for k, v in os.environ.items() if not k.startswith("GZ_COMPAT_")}
    env.update(
        CONDA_PREFIX=str(tmp_path),
        GZ_COMPAT_DIR=str(work),
        DART_PARALLEL_JOBS="1",
        PATH=str(shim) + os.pathsep + env["PATH"],
        WORLD_CALLS=str(tmp_path / "world-calls"),
        WORLD_STARTED=str(tmp_path / "world-started"),
        WORLD_OBSERVED=str(tmp_path / "world-observed"),
        WORLD_PAUSE=pause,
    )
    command = ["bash", str(LANE), "ionic", "bench-gz-sim"]
    runs = []
    try:
        for variant in ("candidate", "base"):
            runs.append(
                subprocess.Popen(
                    command,
                    env={**env, "GZ_COMPAT_VARIANT": variant},
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
            )
            if variant == "candidate":
                deadline = time.monotonic() + 10
                while not (tmp_path / "world-started").exists():
                    assert runs[0].poll() is None, runs[0].communicate()
                    assert time.monotonic() < deadline, "generator did not start"
                    time.sleep(0.01)
        for run in runs:
            stdout, stderr = run.communicate(timeout=20)
            assert run.returncode == 0, stdout + stderr
            assert "read complete world" in stdout
    finally:
        for run in runs:
            if run.poll() is None:
                run.kill()
                run.communicate()
    calls = (tmp_path / "world-calls").read_text().splitlines()
    assert len(calls) == len(set(calls)), "generators wrote into the same directory"


@pytest.mark.skipif(sys.platform != "linux", reason="compat lanes require Linux")
def test_parallel_variants_share_one_clean_tagged_clone(tmp_path):
    git = shutil.which("git")
    remote = tmp_path / "remote"
    subprocess.run([git, "init", "-q", str(remote)], check=True)
    subprocess.run(
        [
            git,
            "-C",
            str(remote),
            "-c",
            "user.name=Test",
            "-c",
            "user.email=test@example.org",
            "commit",
            "-qm",
            "fixture",
            "--allow-empty",
        ],
        check=True,
    )
    subprocess.run([git, "-C", str(remote), "tag", "gz-sim9_9.5.0"], check=True)
    shim = tmp_path / "bin"
    shim.mkdir()
    # Hold the first clone open long enough for both variants to reach it.
    # Only clone is redirected; the lane's tag and dirty-tree checks use git.
    (shim / "git").write_text(
        '#!/bin/bash\nif [ "$1" = clone ]; then\n'
        '  echo clone >> "$CLONE_CALLS"\n  sleep 0.5\n'
        '  exec "$REAL_GIT" clone -q --branch gz-sim9_9.5.0 '
        '"$CLONE_REMOTE" "${@: -1}"\nfi\nexec "$REAL_GIT" "$@"\n'
    )
    # The worlds step calls Python after cloning; this test covers the clone.
    (shim / "python").write_text(
        '#!/bin/bash\nmkdir -p "$3"\ntouch "$3/3k_shapes.sdf"\n'
    )
    for path in shim.iterdir():
        path.chmod(0o755)
    env = {k: v for k, v in os.environ.items() if not k.startswith("GZ_COMPAT_")}
    env.update(
        CONDA_PREFIX=str(tmp_path),
        GZ_COMPAT_DIR=str(tmp_path / "work"),
        DART_PARALLEL_JOBS="1",
        PATH=str(shim) + os.pathsep + env["PATH"],
        REAL_GIT=git,
        CLONE_REMOTE=str(remote),
        CLONE_CALLS=str(tmp_path / "clones"),
    )
    command = ["bash", str(LANE), "ionic", "worlds"]
    runs = [
        subprocess.Popen(
            command,
            env={**env, "GZ_COMPAT_VARIANT": variant},
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        for variant in ("candidate", "base")
    ]
    for run in runs:
        stdout, stderr = run.communicate(timeout=20)
        assert run.returncode == 0, stdout + stderr
    assert (tmp_path / "clones").read_text().splitlines() == ["clone"]

    source = tmp_path / "work" / "src" / "gz-sim"
    (source / "untracked.cc").touch()
    run = subprocess.run(command, env=env, capture_output=True, text=True)
    assert run.returncode == 1
    assert "has local changes" in run.stderr
    (source / "untracked.cc").unlink()
    subprocess.run([git, "-C", str(source), "tag", "-d", "gz-sim9_9.5.0"], check=True)
    run = subprocess.run(command, env=env, capture_output=True, text=True)
    assert run.returncode == 1
    assert "expected gz-sim9_9.5.0" in run.stderr


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


def test_missing_expected_is_an_error_unless_writing_baseline(tmp_path, capsys):
    _write_lane(tmp_path, {}, {})
    expected = tmp_path / "missing.txt"
    assert _compare(tmp_path, expected) == 2
    output = capsys.readouterr()
    assert str(expected) in output.err
    assert "PASS" not in output.out
    assert _compare(tmp_path, expected, "--write-baseline") == 0
    assert expected.read_text().endswith(
        "# CTest tests of the baseline run (compare requires each one)\n"
    )


@pytest.mark.parametrize("value", ["inf", "nan", "0", "-1", "1oops", "oops"])
@pytest.mark.parametrize(
    "option", ["--max-seconds-scale", "--max-seconds-factor", "env"]
)
def test_invalid_timing_multipliers_are_argparse_errors(
    tmp_path, capsys, monkeypatch, value, option
):
    _write_lane(tmp_path, _step_world(2.0), {})
    expected = tmp_path / "expected.txt"
    expected.write_text("")
    if option == "env":
        monkeypatch.setenv("GZ_COMPAT_MAX_SECONDS_SCALE", value)
        extra = []
    else:
        extra = [option, value]
    for mode in ([], ["--write-baseline"]):
        with pytest.raises(SystemExit) as error:
            _compare(tmp_path, expected, *extra, *mode)
        assert error.value.code == 2
        output = capsys.readouterr()
        assert "--max-seconds-" in output.err
        assert "PASS" not in output.out
        assert expected.read_text() == ""


@pytest.mark.parametrize("value", ["inf", "nan", "-1", "1oops"])
def test_invalid_expected_numbers_are_errors(tmp_path, capsys, value):
    _write_lane(tmp_path, _step_world(2.0), {})
    expected = tmp_path / "expected.txt"
    expected.write_text(
        f"max-seconds gz-physics {STEP_WORLD} Step/0.StepWorld {value}\n"
    )
    assert _compare(tmp_path, expected) == 2
    assert "PASS" not in capsys.readouterr().out


@pytest.mark.parametrize("seconds", [float("inf"), float("nan"), -1, "oops"])
def test_invalid_case_times_are_errors(tmp_path, capsys, seconds):
    _write_lane(tmp_path, _step_world(seconds), {})
    expected = tmp_path / "expected.txt"
    expected.write_text("")
    assert _compare(tmp_path, expected, "--write-baseline") == 2
    assert "PASS" not in capsys.readouterr().out
    assert expected.read_text() == ""


def test_missing_case_time_is_an_error(tmp_path, capsys):
    _write_lane(tmp_path, _step_world(2.0), {})
    expected = tmp_path / "expected.txt"
    expected.write_text("")
    xml = tmp_path / "gz-physics-gtest" / f"{STEP_WORLD}.xml"
    xml.write_text(xml.read_text().replace(' time="2.0"', ""))
    assert _compare(tmp_path, expected) == 2
    assert "time must be finite" in capsys.readouterr().err


def test_missing_gtest_directory_is_an_error(tmp_path, capsys):
    _write_lane(tmp_path, {}, {})
    expected = tmp_path / "expected.txt"
    expected.write_text("")
    shutil.rmtree(tmp_path / "gz-physics-gtest")
    assert _compare(tmp_path, expected) == 2
    assert "missing" in capsys.readouterr().err


def test_expected_directory_is_an_error_even_when_writing_baseline(tmp_path):
    _write_lane(tmp_path, {}, {})
    assert _compare(tmp_path, tmp_path) == 2
    assert _compare(tmp_path, tmp_path, "--write-baseline") == 2


def test_timing_overflow_is_an_error(tmp_path, capsys):
    _write_lane(tmp_path, _step_world(1e308), {})
    expected = tmp_path / "expected.txt"
    expected.write_text(f"max-seconds gz-physics {STEP_WORLD} Step/0.StepWorld 1e308\n")
    assert _compare(tmp_path, expected, "--max-seconds-scale", "2") == 2
    assert "scaled max-seconds must be finite" in capsys.readouterr().err
    previous = expected.read_text()
    assert _compare(tmp_path, expected, "--write-baseline") == 2
    assert expected.read_text() == previous


def test_a_test_or_timed_case_missing_from_the_results_is_an_error(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text(
        "gz-sim INTEGRATION_log_system  # SEGFAULT\n"
        f"max-seconds gz-physics {STEP_WORLD} Step/0.StepWorld 5.4\n"
    )
    sim = {"INTEGRATION_log_system": ("SEGFAULT", None)}
    candidate = tmp_path / "candidate"
    _write_lane(candidate, _step_world(2.0), sim)
    assert _compare(candidate, expected) == 0
    assert "PASS" in capsys.readouterr().out

    timed = f"gz-physics {STEP_WORLD} Step/0.StepWorld is missing"
    for name, physics, sim_tests, missing in [
        # A GTEST_FILTER left out the timed case, which then passes unjudged.
        ("filtered", {STEP_WORLD: {"Ray.Ok": (0.1, False)}}, sim, timed),
        ("no gz-physics tests", {}, sim, f"gz-physics {STEP_WORLD} is missing"),
        ("no gz-sim tests", _step_world(2.0), {}, "INTEGRATION_log_system is"),
    ]:
        _write_lane(candidate, physics, sim_tests)
        assert _compare(candidate, expected) == 2, name
        assert missing in capsys.readouterr().err, name

    # A crash leaves no timed case; it is a failure, not missing results.
    _write_lane(candidate, {STEP_WORLD: ("SEGFAULT", None)}, sim)
    assert _compare(candidate, expected) == 1
    assert f"NEW        gz-physics {STEP_WORLD} (SEGFAULT)\n" in capsys.readouterr().out


@pytest.mark.parametrize("replacement", [False, True])
def test_a_test_that_stops_registering_is_an_error(tmp_path, capsys, replacement):
    expected = tmp_path / "expected.txt"
    expected.write_text("test gz-sim INTEGRATION_entity\ntest gz-sim INTEGRATION_imu\n")
    imu = {"INTEGRATION_imu": {"Imu.Rotating": (1.0, False)}}
    sim = {**imu, "INTEGRATION_entity": {"Entity.Cmd": (1.0, False)}}
    base = tmp_path / "base"
    _write_lane(base, _step_world(2.0), sim)
    candidate = tmp_path / "candidate"
    _write_lane(candidate, _step_world(2.0), sim)
    assert _compare(candidate, expected, "--base-results", str(base)) == 0
    capsys.readouterr()

    # A passing test that no longer registers is in no expected-failure entry.
    remaining = dict(imu)
    if replacement:
        remaining["INTEGRATION_new"] = {"New.Case": (1.0, False)}
    _write_lane(candidate, _step_world(2.0), remaining)
    assert _compare(candidate, expected) == 2
    assert "missing from the results: INTEGRATION_entity" in capsys.readouterr().err
    expected.write_text("")
    assert _compare(candidate, expected, "--base-results", str(base)) == 2
    assert "missing from the results: INTEGRATION_entity" in capsys.readouterr().err


def test_extra_registered_tests_are_allowed(tmp_path, capsys):
    expected = tmp_path / "expected.txt"
    expected.write_text("test gz-sim INTEGRATION_imu\n")
    _write_lane(
        tmp_path,
        _step_world(2.0),
        {
            "INTEGRATION_imu": {"Imu.Rotating": (1.0, False)},
            "INTEGRATION_new": {"New.Case": (1.0, False)},
        },
    )
    assert _compare(tmp_path, expected) == 0
    assert "PASS" in capsys.readouterr().out


def test_unreadable_results_are_an_error_naming_the_file(tmp_path, capsys):
    _write_lane(tmp_path, _step_world(2.7), {})
    # A GoogleTest XML that another process wrote over while it was written.
    xml = tmp_path / "gz-physics-gtest" / f"{STEP_WORLD}.xml"
    xml.write_text(xml.read_text() + "</testsuite>")
    assert _compare(tmp_path, tmp_path / "x.txt") == 2
    assert f"error: {xml}: " in capsys.readouterr().err
