"""Tests for scripts/nightly_ci_report.py (the Nightly workflow's issue reporter)."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "nightly_ci_report", ROOT / "scripts" / "nightly_ci_report.py"
)
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


def job(name, conclusion):
    return {"id": hash(name), "name": name, "conclusion": conclusion}


def issue(number, group, branch="release-6.20"):
    return {"number": number, "body": f"{report.marker(branch, group)}\nbody"}


def test_plan_groups_jobs_by_nightly_caller():
    jobs = [
        job("linux / Release", "success"),
        job("linux / coverage", "failure"),
        job("macos / arm64-Release", "success"),
        job("freebsd / FreeBSD repro (VM)", "success"),
        job("toolchain / gcc (newest)", "skipped"),
        job("report", None),  # the running report job itself
    ]
    issues = [
        issue(7, "freebsd"),
        issue(8, "toolchain"),
        issue(9, "linux", "release-6.19"),
    ]
    actions = report.plan(jobs, issues, "release-6.20")
    assert [(a, g, n, [j["name"] for j in f]) for a, g, n, f in actions] == [
        ("create", "linux", None, ["linux / coverage"]),  # 6.19's issue doesn't count
        ("close", "freebsd", 7, []),
    ]  # a skipped group neither recovers nor fails


def test_plan_comments_on_an_open_issue_for_a_still_failing_group():
    actions = report.plan(
        [job("windows / windows-Release", "timed_out")],
        [issue(3, "windows")],
        "release-6.20",
    )
    assert [(a, n) for a, _, n, _ in actions] == [("comment", 3)]


def test_plan_keeps_a_group_failing_while_a_job_is_stale():
    jobs = [
        job("macos / arm64-Release", "success"),
        job("macos / arm64-Debug", "stale"),
    ]
    actions = report.plan(jobs, [issue(5, "macos")], "release-6.20")
    assert [(a, n) for a, _, n, _ in actions] == [("comment", 5)]


def test_job_log_retries_without_the_flag_older_gh_rejects(monkeypatch):
    def fake_gh(*args, stdin=None):
        if "--allow-escape-sequences" in args:
            raise report.subprocess.CalledProcessError(1, args)
        return "log text"

    monkeypatch.setattr(report, "gh", fake_gh)
    assert report.job_log("o/r", 1) == "log text"


def test_excerpt_keeps_the_lines_before_the_first_error():
    log = "\n".join(
        [f"2026-10-05T08:00:{i:02d}.0000000Z line {i}" for i in range(100)]
        + ["2026-10-05T08:01:40.0000000Z ##[error]Process completed with exit code 8."]
        + ["2026-10-05T08:01:41.0000000Z Post job cleanup."]
    )
    lines = report.excerpt(log).splitlines()
    assert lines[0] == "line 60" and lines[-1].startswith("##[error]")
    assert report.excerpt("2026-10-05T08:00:00.0Z only line") == "only line"
    assert report.excerpt("") == ""
    assert (
        report.excerpt("2026-10-05T08:00:00.0Z \x1b[36;1mcolored\x1b[0m") == "colored"
    )
