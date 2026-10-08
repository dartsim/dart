"""Regression checks for performance comparison and local execution."""

import copy
import fnmatch
import importlib.util
import json
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
import yaml


def test_nightly_rows_cover_canonical_windows_without_changing_quick_tier(tmp_path):
    module = _load_runner()
    quick = module.select_rows("")
    rows = module.select_rows("nightly")
    assert rows[: len(quick)] == quick
    assert len(rows) == len({row.key for row in rows})
    assert {
        (row.row, row.det, row.threads, row.steps) for row in module.NIGHTLY_ROWS
    } == {
        *{
            (f"S1-{objects}-t{threads}", det, threads, 200)
            for objects in (60, 120)
            for det in ("dart", "ode")
            for threads in (1, 16)
        },
        *{("S2", det, 1, 3000) for det in module.DETECTORS},
        *{
            (f"{scene}-t{threads}", det, threads, 300)
            for scene in ("S3", "S4")
            for det in module.DETECTORS
            for threads in (1, 4, 16)
        },
        *{("S5", det, 1, 300) for det in module.DETECTORS},
        ("S6", "dart", 1, 20000),
        ("mf", "dart", 1, 50),
    }
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--nightly",
        ]
    )
    args.bin_dir, args.source_dir = tmp_path, module.ROOT
    for row in module.NIGHTLY_ROWS:
        command = module.row_command(
            row, args, tmp_path / "world", row.warmup, row.steps
        )
        if row.row.startswith("S"):
            assert row.warmup == 0 and not row.ir and not row.perturb
            assert ("--disable-deactivation" in command) == row.row.startswith(
                ("S1", "S3")
            )
            assert ("--quiet" in command) == (row.row != "S6")
    s6 = module.select_rows("S6")[0]
    assert s6.checkpoint == 5000 and "--max-contacts" not in s6.args
    assert module.select_rows("mf")[0].warmup == 50


def test_canonical_guard_is_measured_once_and_does_not_require_perturbation(
    monkeypatch, tmp_path
):
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
        ]
    )
    calls = []

    def native(row, args, world, config=""):
        calls.append(config)
        return {"guards": {"finite": True}, "allocs": 0, "bytes": 0}

    monkeypatch.setattr(module, "native", native)
    row = module.measure(module.select_rows("S6")[0], args, tmp_path)
    assert calls == [""]
    assert row["status"] == "ok" and not row["gated"]
    assert row["qualification_required"] is False
    assert "ir_per_step" not in row["head"]
    monkeypatch.setattr(module, "run_arm", lambda args: {"results": [row]})
    assert (
        module.main(
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
        == 0
    )


@pytest.mark.parametrize(
    "missing,penetration", [(False, "0.3"), (True, "0.3"), (False, "inf")]
)
def test_s6_capture_keeps_penetration_checkpoints(
    monkeypatch, tmp_path, missing, penetration
):
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
        ]
    )
    args.bin_dir, args.source_dir = tmp_path, module.ROOT
    checkpoints = (5000, 10000, 15000) if missing else (5000, 10000, 15000, 20000)
    output = "\n".join(
        [
            f"STEPALLOC steps=20000 measured=20000 allocs=0 bytes=0 libdart={tmp_path}/libdart.so",
            "PERFTIME maxrss_kb=100",
            "Avg Step Time: 1 ms",
            "Final State Hash: 0x1",
            f"Final State Finite: {'true' if penetration != 'inf' else 'false'}",
            "Final Contacts: 1",
            "Final Contact Cap Hit: false",
            "Final Resting: 0/71",
            f"Final Max Penetration: {penetration}",
            *(
                f"step {step} rtf 1 contacts 1 max_penetration 0.2 mobile 71 resting 0 islands 1"
                for step in checkpoints
            ),
        ]
    )
    monkeypatch.setattr(module, "execute", lambda *args: output)
    monkeypatch.setattr(module, "perturb_environment", lambda *args: {})
    monkeypatch.setattr(module, "environment", lambda *args: {})
    if missing:
        with pytest.raises(ValueError, match="missing canonical checkpoints"):
            module.native(module.select_rows("S6")[0], args, tmp_path)
    else:
        if penetration == "inf":
            # A non-finite final penetration breaks the row on every detector.
            with pytest.raises(module.BenchmarkCaseError, match="non-finite final"):
                module.native(module.select_rows("S6")[0], args, tmp_path)
            return
        metric = module.native(module.select_rows("S6")[0], args, tmp_path)
        assert metric["max_penetration"] == 0.3
        assert [item["step"] for item in metric["checkpoints"]] == list(checkpoints)
        module.write_json(tmp_path / "metric.json", metric)


def test_s6_non_finite_checkpoint_breaks_the_row(monkeypatch, tmp_path):
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
        ]
    )
    args.bin_dir, args.source_dir = tmp_path, module.ROOT
    output = "\n".join(
        [
            f"STEPALLOC steps=20000 measured=20000 allocs=0 bytes=0 libdart={tmp_path}/libdart.so",
            "PERFTIME maxrss_kb=100",
            "Avg Step Time: 1 ms",
            "Final State Hash: 0x1",
            "Final State Finite: true",
            "Final Contacts: 1",
            "Final Contact Cap Hit: false",
            "Final Resting: 0/71",
            "Final Max Penetration: 0.3",
            *(
                f"step {step} rtf 1 contacts 1 max_penetration "
                f"{'nan' if step == 5000 else '0.2'} mobile 71 resting 0 islands 1"
                for step in (5000, 10000, 15000, 20000)
            ),
        ]
    )
    monkeypatch.setattr(module, "execute", lambda *args: output)
    monkeypatch.setattr(module, "perturb_environment", lambda *args: {})
    monkeypatch.setattr(module, "environment", lambda *args: {})
    # A finite final state does not cover the intermediate checkpoints.
    with pytest.raises(module.BenchmarkCaseError, match="non-finite checkpoint"):
        module.native(module.select_rows("S6")[0], args, tmp_path)


def test_nightly_table_fetches_main_for_an_unknown_published_commit(monkeypatch):
    module = _load_runner()
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command[3])
        merge_base = sum(call == "merge-base" for call in calls)
        code = 128 if command[3] == "merge-base" and merge_base == 1 else 0
        return subprocess.CompletedProcess(command, code, "", "")

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    previous = "Generated at 2026-10-08T00:00:00+00:00 for `" + "a" * 40 + "`.\n"
    record = {"run": {"commit": "b" * 40, "time": "2026-10-08T01:00:00+00:00"}}
    assert module.nightly_table_can_advance(record, previous)
    assert calls == ["merge-base", "fetch", "merge-base"]


def test_nightly_requires_complete_head_only_measurement(monkeypatch, tmp_path):
    module = _load_runner()
    args = module.parser().parse_args(
        ["local", "--nightly", "--output-dir", str(tmp_path)]
    )
    with pytest.raises(ValueError, match="requires --smoke"):
        module.local_arms(args)
    args.smoke, args.rows = True, "pend"
    with pytest.raises(ValueError, match="complete nightly row set"):
        module.local_arms(args)
    monkeypatch.setattr(
        module,
        "run_arm",
        lambda args: {
            "results": [
                {"row": "S6", "det": "dart", "status": "unsupported", "gated": False},
            ]
        },
    )
    assert (
        module.main(
            [
                "run",
                "--commit",
                "HEAD",
                "--prefix",
                str(tmp_path),
                "--output-dir",
                str(tmp_path),
                "--nightly",
            ]
        )
        == 1
    )


@pytest.mark.parametrize("pairs", [None, 0, 3])
def test_nightly_requires_contact_pairs_and_saves_failures(
    monkeypatch, tmp_path, pairs
):
    module = _load_runner()
    rows = [*module.NIGHTLY_ROWS, module.select_rows("gzb")[0]]
    monkeypatch.setattr(module, "select_rows", lambda selection: rows)
    monkeypatch.setattr(module, "command_output", lambda command: "a" * 40)
    monkeypatch.setattr(
        module, "fingerprint", lambda *args: _publication_fixture()["run"]["env"]
    )
    monkeypatch.setattr(
        module,
        "installed_provenance",
        lambda args: {
            "workload_sources": {module.CB: "contact", module.PB: "portable"}
        },
    )
    output = "\n".join(
        [
            "Final State Hash: 0x1",
            "Final State Finite: true",
            "Final Contacts: 3",
            "Final Contact Cap Hit: false",
            "Final Resting: 0/3",
            *([] if pairs is None else [f"Final Contact Pairs: {pairs}"]),
        ]
    )
    monkeypatch.setattr(
        module,
        "native",
        lambda *args: {
            "guards": module.guards(output),
            "allocs_per_step": 0,
            "bytes_per_step": 0,
        },
    )
    shim = tmp_path / "shim.so"
    shim.write_bytes(b"shim")
    assert module.main(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--shim",
            str(shim),
            "--nightly",
            "--native-only",
            "--no-perturb",
        ]
    ) == int(pairs is None)
    record = module.publication_record(tmp_path / "record.json", "nightly")
    for row in record["results"][:-1]:
        assert row["status"] == ("broken" if pairs is None else "ok")
        if pairs is None:
            assert row["error"] == "missing contact pair count"
            assert not row["gated"]
        else:
            assert row["head"]["guards"]["pairs"] == pairs
    # The portable driver does not emit pair counts on main.
    assert record["results"][-1]["status"] == "ok"
    module.write_publication(tmp_path / "pages", record)
    table = (tmp_path / "pages/performance/guards/main.md").read_text()
    if pairs is None:
        assert "missing contact pair count" in table and "broken" in table


@pytest.mark.parametrize(
    "paths,mode,smoke",
    [
        (["pixi.toml", "pixi.lock"], "skip", "false"),
        (["docs/README.md"], "skip", "false"),
        (["scripts/perf_regression.py"], "ab", "true"),
        (["tools/perf/allocshim.c"], "ab", "true"),
        ([".github/workflows/perf.yml"], "ab", "true"),
        (["dart/simulation/World.cpp", "pixi.lock"], "ab", "false"),
        (["tests/benchmark/worlds/3k_shapes.sdf.gz"], "ab", "true"),
    ],
)
def test_dispatch_scope_compares_first_parent_and_skips_pixi_only(
    tmp_path, paths, mode, smoke
):
    stub = """
    changed_paths=("$@")
    git() {
      case "$1" in
        rev-parse)
          if [[ "$*" == *^1* ]]; then printf '%s\\n' parent; else printf '%s\\n' head; fi ;;
        diff) printf '%s\\0' "${changed_paths[@]}" ;;
        merge-base|checkout|worktree) return 0 ;;
      esac
    }
    """
    env_file = tmp_path / "env"
    result = subprocess.run(
        [
            "bash",
            "-e",
            "-c",
            stub
            + _perf_step("record-measure", "Select base and measurement scope")["run"],
            "selector",
            *paths,
        ],
        cwd=Path(__file__).resolve().parents[1],
        env={
            **os.environ,
            "GITHUB_ENV": str(env_file),
            "GITHUB_OUTPUT": str(tmp_path / "output"),
            "GITHUB_STEP_SUMMARY": str(tmp_path / "summary"),
            "RUNNER_TEMP": str(tmp_path),
            "REQUESTED_HEAD": "",
            "REQUESTED_BASE": "",
            "GITHUB_EVENT_NAME": "workflow_dispatch",
            "PUSH_BEFORE": "",
        },
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr
    selected = dict(line.split("=", 1) for line in env_file.read_text().splitlines())
    assert (selected["PERF_BASE"], selected["PERF_HEAD"]) == ("parent", "head")
    assert (selected["PERF_MODE"], selected["PERF_SMOKE"]) == (mode, smoke)


@pytest.mark.parametrize(
    "event,before,requested,expected",
    [
        ("push", "start", "", "start"),
        ("push", "zero", "", "parent"),
        ("push", "", "", "parent"),
        ("push", "missing", "", "parent"),
        ("push", "side", "", None),
        ("workflow_dispatch", "start", "", "parent"),
        ("workflow_dispatch", "", "parent", "parent"),
        ("workflow_dispatch", "", "start", None),
        ("workflow_dispatch", "", "missing", None),
    ],
)
def test_merge_scope_uses_push_range_and_validates_ancestry(
    tmp_path, event, before, requested, expected
):
    source = tmp_path / "source"
    source.mkdir()

    def git(*arguments):
        return subprocess.run(
            ["git", "-C", str(source), *arguments],
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip()

    git("init", "--initial-branch=main")
    git("config", "user.name", "test")
    git("config", "user.email", "test@example.com")
    revisions = {"zero": "0" * 40, "missing": "f" * 40, "": ""}
    # A rebase merge whose first commit changes DART and last changes only docs.
    for name, path in (
        ("start", "README.md"),
        ("parent", "dart/change.cpp"),
        ("head", "docs/change.md"),
    ):
        target = source / path
        target.parent.mkdir(exist_ok=True)
        target.write_text(name)
        git("add", ".")
        git("commit", "-m", name)
        revisions[name] = git("rev-parse", "HEAD")
    git("update-ref", "refs/remotes/origin/main", revisions["head"])
    git("checkout", "-b", "side", revisions["start"])
    git("commit", "--allow-empty", "-m", "unrelated history")
    revisions["side"] = git("rev-parse", "HEAD")
    git("checkout", "main")
    env_file = tmp_path / "env"
    step = _perf_step("record-measure", "Select base and measurement scope")
    assert step["env"]["PUSH_BEFORE"] == "${{ github.event.before }}"
    result = subprocess.run(
        ["bash", "-e", "-o", "pipefail", "-c", step["run"]],
        cwd=source,
        env={
            **os.environ,
            "GITHUB_EVENT_NAME": event,
            "PUSH_BEFORE": revisions[before],
            "REQUESTED_HEAD": "",
            "REQUESTED_BASE": revisions[requested],
            "RUNNER_TEMP": str(tmp_path),
            "GITHUB_ENV": str(env_file),
            "GITHUB_OUTPUT": str(tmp_path / "output"),
            "GITHUB_STEP_SUMMARY": str(tmp_path / "summary"),
        },
        text=True,
        capture_output=True,
    )
    if expected is None:
        assert result.returncode != 0
        assert not env_file.exists()
    else:
        assert result.returncode == 0, result.stderr
        selected = dict(
            line.split("=", 1) for line in env_file.read_text().splitlines()
        )
        assert selected["PERF_BASE"] == revisions[expected]
        assert selected["PERF_HEAD"] == revisions["head"]
        assert selected["PERF_MODE"] == ("ab" if expected == "start" else "skip")


@pytest.mark.parametrize("ambiguity", [False, True])
def test_merge_pr_lookup_matches_merged_main_commit(tmp_path, ambiguity):
    pr = {
        "number": 3570,
        "merged_at": "date",
        "base": {"ref": "main"},
        "merge_commit_sha": "head",
        "body": "Perf-Regression-Rationale: s3w/ode: accepted work",
    }
    others = [
        {**pr, "number": 20, "merged_at": None},
        {**pr, "number": 21, "base": {"ref": "release-6.19"}},
        {**pr, "number": 22, "merge_commit_sha": "another"},
    ]
    if ambiguity:
        others.append({**pr, "number": 23})
    (tmp_path / "perf-pulls.json").write_text(json.dumps([[pr], others]))
    result = _run_perf_snippet(
        "record-measure",
        "Find the merged PR and its current rationale",
        tmp_path,
        {
            "RUNNER_TEMP": str(tmp_path),
            "PERF_HEAD": "head",
            "GITHUB_OUTPUT": str(tmp_path / "output"),
        },
    )
    if ambiguity:
        assert result.returncode != 0 and "ambiguous merged PR" in result.stderr
    else:
        assert result.returncode == 0, result.stderr
        assert (tmp_path / "perf-pr-body.txt").read_text() == pr["body"]
        assert (tmp_path / "output").read_text() == "number=3570\n"


def test_publication_permissions_keep_all_measurement_read_only():
    workflow = _perf_workflow()
    assert workflow["on"]["push"]["paths"] == workflow["on"]["pull_request"]["paths"]
    assert "pull_request_target" not in workflow["on"]
    assert "GH_TOKEN" not in workflow.get("env", {})
    for job in ("measure", "record-measure", "nightly-measure"):
        definition = workflow["jobs"][job]
        permissions = {"contents": "read"}
        if job == "record-measure":
            permissions["pull-requests"] = "read"
        assert definition["permissions"] == permissions
        assert "GH_TOKEN" not in definition.get("env", {})
        for step in definition["steps"]:
            if "GH_TOKEN" in step.get("env", {}):
                assert step["name"] == "Find the merged PR and its current rationale"
            if step.get("uses", "").startswith("actions/checkout@"):
                assert step["with"]["persist-credentials"] == "false"
    for job in ("record", "nightly"):
        definition = workflow["jobs"][job]
        assert definition["permissions"]["contents"] == "write"
        assert "refs/heads/main" in definition["if"]
        assert "pull_request" not in definition["if"]
        assert (
            definition["steps"][0]["run"].splitlines()[0]
            == 'test "$RUNNER_ENVIRONMENT" = github-hosted'
        )


def test_merge_writer_uses_only_main_publisher_and_saved_evidence():
    jobs = _perf_workflow()["jobs"]
    measure, writer = jobs["record-measure"], jobs["record"]
    assert writer["needs"] == "record-measure"
    assert "needs.record-measure.result == 'success'" in writer["if"]
    assert "needs.record-measure.outputs.mode == 'ab'" in writer["if"]
    assert writer["permissions"] == {
        "contents": "write",
        "pull-requests": "write",
    }
    assert "GH_TOKEN" not in writer["env"]
    assert writer["env"]["PERF_HEAD"] == "${{ needs.record-measure.outputs.head }}"
    assert writer["env"]["PERF_PR"] == "${{ needs.record-measure.outputs.pr }}"
    checkout = _perf_step("record", "Checkout main publisher")
    assert checkout["with"] == {"ref": "main", "persist-credentials": "false"}
    upload = _perf_step("record-measure", "Upload merge evidence")
    download = _perf_step(
        "record", "Download merge measurements (including earlier attempts)"
    )
    assert (
        upload["with"]["name"]
        == "perf-merge-${{ github.run_id }}-${{ github.run_attempt }}"
    )
    assert (
        measure["outputs"]["artifact"]
        == "${{ format('perf-merge-{0}-{1}', github.run_id, github.run_attempt) }}"
    )
    assert download["with"]["name"] == "${{ needs.record-measure.outputs.artifact }}"
    assert "overwrite" not in upload["with"]
    assert "${{ env.PERF_OUTPUT }}/perf.*" in upload["with"]["path"]
    assert download["with"]["path"] == "${{ runner.temp }}/perf"
    assert not any(
        step.get("uses", "").startswith("prefix-dev/") for step in writer["steps"]
    )
    for step in writer["steps"]:
        script = step.get("run", "")
        assert "pixi" not in script
        assert " local " not in script
        assert ("GH_TOKEN" in step.get("env", {})) == ("gh api" in script)
    assert not any(" publish " in step.get("run", "") for step in measure["steps"])
    publish = _perf_step("record", "Publish the merge record and chart")["run"]
    assert (
        'python3 scripts/perf_regression.py publish --record "$PERF_OUTPUT/perf.json"'
        in publish
    )
    comment_step = _perf_step("record", "Refresh the merged PR verdict comment")
    assert "steps.hosted.outcome == 'success'" in comment_step["if"]
    assert "steps.download.outcome == 'success'" in comment_step["if"]
    comment = comment_step["run"]
    assert 'marker="<!-- dart-perf-merge:$PERF_HEAD -->"' in comment
    assert "gh api --method PATCH" in comment and "gh api --method POST" in comment
    names = [step["name"] for step in writer["steps"]]
    assert (
        names.index("Publish the merge record and chart")
        < names.index("Refresh the merged PR verdict comment")
        < names.index("Summarize and enforce the merged verdict")
    )


@pytest.mark.parametrize("status,exit_code", [("PASS", 0), ("FAIL", 1), ("ERROR", 2)])
def test_merge_writer_enforces_saved_verdict(tmp_path, status, exit_code):
    (tmp_path / "perf.json").write_text(json.dumps({"verdict": {"status": status}}))
    result = _run_perf_snippet(
        "record",
        "Summarize and enforce the merged verdict",
        tmp_path,
        {"PERF_OUTPUT": str(tmp_path)},
    )
    assert result.returncode == exit_code, result.stderr


def test_merge_comment_refresh_requires_successful_publication():
    publish = _perf_step("record", "Publish the merge record and chart")
    refresh = _perf_step("record", "Refresh the merged PR verdict comment")
    assert publish["id"] == "publish"
    # A failed/skipped publisher must preserve the comment, regardless of verdict.
    # A published FAIL still needs to be reported before the enforcement step.
    assert refresh["if"] == (
        "${{ !cancelled() && steps.hosted.outcome == 'success' && "
        "steps.download.outcome == 'success' && steps.publish.outcome == 'success' }}"
    )


@pytest.mark.parametrize(
    "environment,exit_code", [("github-hosted", 0), ("self-hosted", 1)]
)
def test_merge_writer_refuses_self_hosted_before_checkout(
    tmp_path, environment, exit_code
):
    result = subprocess.run(
        [
            "bash",
            "-e",
            "-c",
            _perf_step("record", "Refuse self-hosted publication")["run"],
        ],
        env={
            **os.environ,
            "RUNNER_ENVIRONMENT": environment,
            "RUNNER_TEMP": str(tmp_path),
            "GITHUB_ENV": str(tmp_path / "env"),
        },
        text=True,
        capture_output=True,
    )
    assert result.returncode == exit_code, result.stderr
    assert (tmp_path / "env").exists() == (environment == "github-hosted")


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
        "compiler_provenance": "dart-perf-build/1",
        "compiler_sha": "1" * 64,
        "valgrind_sha": "1" * 64,
        "callgrind_sha": "1" * 64,
        "glibc": "2.39",
        "preset": "perf-1",
        "fingerprint": "test",
    }
    metric = {
        "ir_per_step": 100_000,
        "allocs_per_step": 10,
        "bytes_per_step": 100,
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
        "bytes": 0,
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
        == "ERROR"
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


def _fake_valgrind(module, monkeypatch, tmp_path):
    launcher = tmp_path / "valgrind/bin/valgrind"
    launcher.parent.mkdir(parents=True)
    launcher.write_bytes(b"launcher")
    tool = tmp_path / "valgrind/libexec/valgrind/callgrind-amd64-linux"
    tool.parent.mkdir(parents=True)
    tool.write_bytes(b"callgrind")
    link = tmp_path / "valgrind-link"
    link.symlink_to(launcher)
    monkeypatch.setattr(module, "VALGRIND", str(link))
    return launcher, tool


@pytest.mark.parametrize("defect", ["patched", "missing", "missing setting"])
def test_cmake_compiler_hashes_resolved_executable(tmp_path, defect):
    module = _load_runner()
    executable = tmp_path / "compiler"
    executable.write_bytes(b"compiler")
    link = tmp_path / "c++"
    link.symlink_to(executable)
    settings = tmp_path / "CMakeFiles/4.0/CMakeCXXCompiler.cmake"
    settings.parent.mkdir(parents=True)
    settings.write_text(
        f'set(CMAKE_CXX_COMPILER "{link}")\n'
        'set(CMAKE_CXX_COMPILER_ID "GNU")\n'
        'set(CMAKE_CXX_COMPILER_VERSION "13.3.0")\n'
    )
    first = module.cmake_compiler(tmp_path)
    assert first == {"compiler": "GNU 13.3.0", "compiler_sha": module.sha(b"compiler")}
    if defect == "patched":
        executable.write_bytes(b"patched compiler")
        second = module.cmake_compiler(tmp_path)
        assert second["compiler"] == first["compiler"]
        assert second["compiler_sha"] == module.sha(b"patched compiler")
    elif defect == "missing":
        executable.unlink()
        with pytest.raises(FileNotFoundError):
            module.cmake_compiler(tmp_path)
    else:
        settings.write_text(settings.read_text().split("\n", 1)[1])
        with pytest.raises(ValueError, match="missing CMake compiler executable"):
            module.cmake_compiler(tmp_path)


@pytest.mark.parametrize("directory", ["libexec/valgrind", "lib/valgrind"])
@pytest.mark.parametrize("missing", ["launcher", "tool", "tool target"])
def test_valgrind_hashes_require_launcher_and_tool(
    monkeypatch, tmp_path, directory, missing
):
    module = _load_runner()
    launcher, tool = _fake_valgrind(module, monkeypatch, tmp_path)
    alternate = launcher.parent.parent / directory / tool.name
    alternate.parent.mkdir(parents=True, exist_ok=True)
    tool.rename(alternate)
    first = module.valgrind_hashes()
    assert first["valgrind_sha"] == module.sha(b"launcher")
    assert first["callgrind_sha"] == module.sha(
        json.dumps(
            {f"{directory}/{tool.name}": module.sha(b"callgrind")}, sort_keys=True
        ).encode()
    )
    if missing == "launcher":
        launcher.unlink()
        with pytest.raises(FileNotFoundError):
            module.valgrind_hashes()
    elif missing == "tool":
        alternate.unlink()
        with pytest.raises(
            ValueError, match="missing Valgrind Callgrind tool provenance"
        ):
            module.valgrind_hashes()
    else:
        alternate.unlink()
        alternate.symlink_to(tmp_path / "missing")
        with pytest.raises(FileNotFoundError):
            module.valgrind_hashes()


def test_fingerprint_uses_installed_build_provenance(monkeypatch, tmp_path):
    module = _load_runner()
    launcher, tool = _fake_valgrind(module, monkeypatch, tmp_path)
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "installed",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--rows",
            "pend",
        ]
    )
    args.bin_dir = tmp_path / "bin"
    args.bin_dir.mkdir()
    for name in ("portable_step_bench", "contact_benchmark"):
        (args.bin_dir / name).write_bytes(b"measured driver")
    args.shim = tmp_path / "allocshim.so"
    args.shim.write_bytes(b"measured shim")
    args.heappad = tmp_path / "heappad.so"
    args.heappad.write_bytes(b"heappad")
    (tmp_path / "share/dart").mkdir(parents=True)
    (tmp_path / "lib").mkdir()
    library = tmp_path / "lib/libdart.so"
    library.write_bytes(b"measured artifact")
    stamp = {
        "schema": "dart-perf-build/1",
        "commit": "installed",
        "compiler": "Clang 18.1.8",
        "compiler_sha": "1" * 64,
        "pixi_lock_sha": "installed lock",
        "preset": "installed preset",
        "libdart_sha": module.sha(library.read_bytes()),
        "libraries": {"lib/libdart.so": module.sha(library.read_bytes())},
        "workload_sources": {
            driver: module.sha(b"installed workload")
            for driver in (module.CB, module.PB)
        },
        "binaries": {
            name: module.sha(b"measured driver")
            for name in ("portable_step_bench", "contact_benchmark")
        },
    }
    path = tmp_path / "share/dart/perf-build.json"
    module.write_json(path, stamp)
    monkeypatch.setattr(module, "execute", lambda *args: "Guest CPU: test\n")

    def command_output(command):
        assert command[0] in (module.VALGRIND, "getconf")
        return "valgrind-3.22.0" if command[0] == module.VALGRIND else "glibc 2.39"

    monkeypatch.setattr(module, "command_output", command_output)
    first = module.fingerprint(args)
    original_read = Path.read_text

    def different_cpu(path, *args, **kwargs):
        if str(path) == "/proc/cpuinfo":
            return "model name : Another hosted CPU\n"
        return original_read(path, *args, **kwargs)

    with monkeypatch.context() as cpu_patch:
        cpu_patch.setattr(Path, "read_text", different_cpu)
        other_cpu = module.fingerprint(args)
    assert other_cpu["host_cpu"] != first["host_cpu"]
    assert other_cpu["fingerprint"] == first["fingerprint"]
    for key in ("compiler", "compiler_sha", "pixi_lock_sha", "preset"):
        assert first[key] == stamp[key]
    for field, executable in (
        ("compiler_sha", None),
        ("valgrind_sha", launcher),
        ("callgrind_sha", tool),
    ):
        if executable:
            original = executable.read_bytes()
            executable.write_bytes(b"patched tool with the same version")
        else:
            stamp[field] = module.sha(b"patched compiler")
            module.write_json(path, stamp)
        changed = module.fingerprint(args)
        assert changed["compiler"] == first["compiler"]
        assert changed["valgrind"] == first["valgrind"]
        assert changed[field] != first[field]
        assert changed["fingerprint"] != first["fingerprint"]
        result = module.compare(
            {"run": {"commit": "base", "env": first}},
            {"run": {"commit": "head", "env": changed}},
        )
        assert result["verdict"]["status"] == "ERROR"
        assert field in result["verdict"]["failures"][0]
        if executable:
            executable.write_bytes(original)
        else:
            stamp[field] = first[field]
            module.write_json(path, stamp)
    stamp["compiler"] = "GNU 13.3.0"
    module.write_json(path, stamp)
    second = module.fingerprint(args)
    assert first["fingerprint"] != second["fingerprint"]
    result = module.compare(
        {"run": {"commit": "base", "env": first}},
        {"run": {"commit": "head", "env": second}},
    )
    assert result["verdict"]["status"] == "ERROR"
    assert "compiler" in result["verdict"]["failures"][0]
    driver = args.bin_dir / "contact_benchmark"
    driver.write_bytes(b"replaced driver")
    with pytest.raises(ValueError, match="driver hash differs"):
        module.fingerprint(args)
    driver.write_bytes(b"measured driver")
    args.shim.write_bytes(b"different shim")
    assert module.fingerprint(args)["fingerprint"] != second["fingerprint"]
    for key, value, reason in (
        ("compiler", "", "invalid installed build provenance"),
        ("compiler", None, "invalid installed build provenance"),
        ("compiler_sha", None, "invalid installed build provenance"),
        ("compiler_sha", "", "invalid installed build provenance"),
        ("compiler_sha", "invalid", "invalid installed build provenance"),
        ("schema", "other", "invalid installed build provenance"),
        ("binaries", None, "invalid installed build provenance"),
        ("workload_sources", None, "invalid installed build provenance"),
        ("workload_sources", {}, "invalid installed build provenance"),
        (
            "workload_sources",
            {module.CB: module.sha(b"installed workload")},
            "invalid installed build provenance",
        ),
        (
            "workload_sources",
            {module.CB: "invalid"},
            "invalid installed build provenance",
        ),
        ("pixi_lock_sha", "", "invalid installed build provenance"),
        ("preset", "", "invalid installed build provenance"),
        ("commit", "other", "commit differs"),
        ("libdart_sha", "stale", "libdart hash differs"),
    ):
        module.write_json(path, {**stamp, key: value})
        with pytest.raises(ValueError, match=reason):
            module.fingerprint(args)
    module.write_json(path, [])
    with pytest.raises(ValueError, match="invalid installed build provenance"):
        module.fingerprint(args)
    module.write_json(
        path, {key: value for key, value in stamp.items() if key != "workload_sources"}
    )
    with pytest.raises(ValueError, match="invalid installed build provenance"):
        module.fingerprint(args)
    module.write_json(path, stamp)
    library.write_bytes(b"replaced artifact")
    with pytest.raises(ValueError, match="libdart hash differs"):
        module.fingerprint(args)
    path.unlink()
    with pytest.raises(ValueError, match="missing installed compiler provenance"):
        module.fingerprint(args)


@pytest.mark.parametrize("defect", ["changed", "added", "removed", "missing manifest"])
def test_installed_provenance_verifies_all_dart_libraries(tmp_path, defect):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "installed",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
        ]
    )
    library = tmp_path / "lib/libdart.so"
    library.parent.mkdir()
    library.write_bytes(b"core")
    component = tmp_path / "lib/libdart-utils.so.6.20"
    component.write_bytes(b"utils")
    (tmp_path / "lib/libdart-utils.so").symlink_to(component.name)
    collision = tmp_path / "lib/libdart-collision-ode.so"
    collision.write_bytes(b"collision")
    stamp = {
        "schema": "dart-perf-build/1",
        "commit": "installed",
        "compiler": "GNU 13.3.0",
        "compiler_sha": "1" * 64,
        "pixi_lock_sha": "installed lock",
        "preset": "perf-1",
        "libdart_sha": module.sha(b"core"),
        "libraries": {
            "lib/libdart.so": module.sha(b"core"),
            "lib/libdart-utils.so": module.sha(b"utils"),
            "lib/libdart-utils.so.6.20": module.sha(b"utils"),
            "lib/libdart-collision-ode.so": module.sha(b"collision"),
        },
        "binaries": {},
        "workload_sources": {},
    }
    path = tmp_path / "share/dart/perf-build.json"
    path.parent.mkdir(parents=True)
    module.write_json(path, stamp)
    assert module.installed_provenance(args) == stamp
    if defect == "changed":
        component.write_bytes(b"stale utils")
    elif defect == "added":
        (tmp_path / "lib/libdart-collision-bullet.so").write_bytes(b"mixed install")
    elif defect == "removed":
        collision.unlink()
    else:
        stamp.pop("libraries")
        module.write_json(path, stamp)
    assert module.sha(library.read_bytes()) == stamp["libdart_sha"]
    reason = (
        "invalid installed build provenance"
        if defect == "missing manifest"
        else "DART library hashes differ"
    )
    with pytest.raises(ValueError, match=reason):
        module.installed_provenance(args)


def test_fingerprint_includes_active_runtime_environment(monkeypatch, tmp_path):
    module = _load_runner()
    _fake_valgrind(module, monkeypatch, tmp_path)
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "installed",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--rows",
            "pend",
        ]
    )
    args.bin_dir = tmp_path / "bin"
    args.bin_dir.mkdir()
    for name in ("portable_step_bench", "contact_benchmark"):
        (args.bin_dir / name).write_bytes(b"driver")
    args.shim = tmp_path / "allocshim.so"
    args.shim.write_bytes(b"shim")
    args.heappad = tmp_path / "heappad.so"
    args.heappad.write_bytes(b"heappad")
    stamp = {
        "schema": "dart-perf-build/1",
        "compiler": "GNU 13.3.0",
        "compiler_sha": "1" * 64,
        "pixi_lock_sha": "unchanged build lock",
        "preset": "perf-1",
        "binaries": {
            name: module.sha(b"driver")
            for name in ("portable_step_bench", "contact_benchmark")
        },
    }
    monkeypatch.setattr(module, "installed_provenance", lambda args: stamp)
    monkeypatch.setattr(module, "execute", lambda *args: "Guest CPU: test\n")
    monkeypatch.setattr(
        module,
        "command_output",
        lambda command: (
            "valgrind-3.22.0" if command[0] == module.VALGRIND else "glibc 2.39"
        ),
    )
    monkeypatch.setenv("PIXI_PROJECT_ROOT", str(tmp_path))
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "default")
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / ".pixi/envs/default"))
    lock = tmp_path / "pixi.lock"
    lock.write_bytes(b"runtime lock")
    first = module.fingerprint(args)
    assert first["runtime_pixi_lock_sha"] == module.sha(b"runtime lock")
    assert first["runtime_environment"] == "default"
    lock.write_bytes(b"different runtime lock")
    changed_lock = module.fingerprint(args)
    lock.write_bytes(b"runtime lock")
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "gazebo")
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / ".pixi/envs/gazebo"))
    changed_environment = module.fingerprint(args)
    for changed, field in (
        (changed_lock, "runtime_pixi_lock_sha"),
        (changed_environment, "runtime_environment"),
    ):
        assert changed["pixi_lock_sha"] == first["pixi_lock_sha"]
        assert changed["fingerprint"] != first["fingerprint"]
        result = module.compare(
            {"run": {"commit": "base", "env": first}},
            {"run": {"commit": "head", "env": changed}},
        )
        assert result["verdict"]["status"] == "ERROR"
        assert field in result["verdict"]["failures"][0]


@pytest.mark.parametrize("missing", ["base", "head", "both"])
@pytest.mark.parametrize(
    "field",
    [
        "compiler",
        "compiler_provenance",
        "compiler_sha",
        "valgrind_sha",
        "callgrind_sha",
    ],
)
def test_saved_records_require_compiler_provenance(missing, field):
    module = _load_runner()
    base = _micro_record(module, "dyn")
    base["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    head = copy.deepcopy(base)
    for record in (
        [base, head] if missing == "both" else [base if missing == "base" else head]
    ):
        record["run"]["env"].pop(field)
    result = module.compare(base, head)
    assert result["verdict"]["status"] == "ERROR"
    reason = (
        "compiler provenance"
        if field in ("compiler", "compiler_provenance")
        else "tool executable provenance"
    )
    assert reason in result["verdict"]["failures"][0]
    assert reason in module.markdown(result)


@pytest.mark.parametrize(
    "ir,allocs,gated,body,status,summary",
    [
        (100, 0, True, "", "PASS", "neutral"),
        (99, 0, True, "", "PASS", "improved"),
        (100.001, 0, True, "", "PASS", "neutral"),
        (100.3, 0, True, "", "WARN", "regressed"),
        (100.5, 0, True, "", "FAIL", "regressed"),
        (101, 0, True, "", "FAIL", "regressed"),
        (99, 1, True, "", "FAIL", "regressed"),
        (None, 1, True, "", "FAIL", "regressed"),
        (99, 0, False, "", "FAIL", "regressed"),
        (
            101,
            1,
            True,
            "Perf-Regression-Rationale: dyn: intended work",
            "WARN",
            "regressed",
        ),
    ],
)
def test_summary_separates_gated_regressions(ir, allocs, gated, body, status, summary):
    module = _load_runner()
    base = _micro_record(module, "dyn")
    base["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    head = copy.deepcopy(base)
    head["results"][0]["head"].update(ir_per_step=ir, allocs_per_step=allocs)
    head["results"][0]["gated"] = gated
    if ir is None:
        for record in (base, head):
            record["results"][0]["method"] = "native"
            record["results"][0]["head"].pop("ir_per_step")
    result = module.compare(base, head, body)
    assert result["verdict"]["status"] == status
    report = module.markdown(result)
    assert f"1 {summary}." in report
    if summary == "regressed":
        assert "1 neutral" not in report and "1 improved" not in report


def test_allocation_report_preserves_buffered_checkpoint_output(tmp_path):
    if sys.platform != "linux" or not all(
        Path(path).is_file() for path in ("/usr/bin/cc", "/usr/bin/time")
    ):
        pytest.skip("the harness shims require GNU libc and the system C compiler")
    module = _load_runner()
    step_symbol = "_ZN4dart10simulation5World4stepEb"
    library = tmp_path / "steps.c"
    library.write_text(f"void {step_symbol}(void* world, _Bool reset) {{}}\n")
    driver = tmp_path / "checkpoints.c"
    driver.write_text(
        f"#include <stdio.h>\nextern void {step_symbol}(void*, _Bool);\n"
        + """
int main(void)
{
  static char buffer[4096];
  if (setvbuf(stdout, buffer, _IOFBF, sizeof(buffer)))
    return 2;
  for (int step = 1; step <= 20000; ++step) {
    _ZN4dart10simulation5World4stepEb(NULL, 0);
    if (step % 5000 == 0)
      printf("step %d\\n", step);
  }
  for (int i = 0; i < 4200; ++i)
    putchar('x');
  puts("\\nFinal State Finite: true");
  return 0;
}
"""
    )
    shim = tmp_path / "allocshim.so"
    for source, target, options in (
        (module.ROOT / "tools/perf/allocshim.c", shim, ["-shared", "-fPIC", "-ldl"]),
        (library, tmp_path / "libsteps.so", ["-shared", "-fPIC"]),
        (
            driver,
            tmp_path / "checkpoints",
            [f"-L{tmp_path}", "-lsteps", f"-Wl,-rpath,{tmp_path}"],
        ),
    ):
        subprocess.run(
            ["/usr/bin/cc", "-O2", str(source), "-o", str(target), *options],
            check=True,
            capture_output=True,
        )
    env = os.environ.copy()
    for key in ("HEAPPAD", "PERF_WINDOW"):
        env.pop(key, None)
    env.update(LD_PRELOAD=str(shim), PERF_WARMUP="0")
    output = module.execute(
        ["/usr/bin/time", "-f", "PERFTIME maxrss_kb=%M", str(tmp_path / "checkpoints")],
        env,
        tmp_path / "checkpoints.log",
        30,
    )
    match = re.search(r"^STEPALLOC steps=(\d+) measured=(\d+)", output, re.MULTILINE)
    assert match and tuple(map(int, match.groups())) == (20000, 20000), output
    assert re.findall(r"^step (\d+)$", output, re.MULTILINE) == [
        "5000",
        "10000",
        "15000",
        "20000",
    ]
    assert "x" * 4200 + "\nFinal State Finite: true\n" in output


def test_allocation_shims_count_each_entry_once(tmp_path):
    import os
    import re
    import subprocess
    import sys

    if sys.platform != "linux" or not Path("/usr/bin/cc").is_file():
        pytest.skip("the harness shims require GNU libc and the system C compiler")
    sources = Path(__file__).resolve().parents[1] / "tools/perf"
    for name in ("allocshim", "heappad", "allocation_probe"):
        command = [
            "/usr/bin/cc",
            "-O2",
            "-o",
            str(tmp_path / name),
            str(sources / f"{name}.c"),
            "-ldl",
        ]
        if name != "allocation_probe":
            command += ["-shared", "-fPIC"]
        subprocess.run(command, check=True, capture_output=True)
    env = os.environ.copy()
    for key in ("LD_PRELOAD", "HEAPPAD", "PERF_WARMUP", "PERF_WINDOW"):
        env.pop(key, None)
    module = _load_runner()
    for mode in ("", "unset", *module.PERTURBATIONS):
        current = {**env, "LD_PRELOAD": str(tmp_path / "allocshim")}
        if mode == "tcache0":
            current["GLIBC_TUNABLES"] = "glibc.malloc.tcache_count=0"
        elif mode:
            current["LD_PRELOAD"] = f"{tmp_path}/allocshim:{tmp_path}/heappad"
            if mode != "unset":
                current["HEAPPAD"] = mode
        for entry in (
            "malloc",
            "calloc",
            "realloc",
            "memalign",
            "aligned_alloc",
            "posix_memalign",
        ):
            result = subprocess.run(
                [str(tmp_path / "allocation_probe"), entry],
                env=current,
                check=True,
                capture_output=True,
                text=True,
            )
            counts = re.search(
                r"STEPALLOC steps=(\d+) measured=(\d+) allocs=(\d+) bytes=(\d+)",
                result.stderr,
            )
            assert counts and tuple(map(int, counts.groups())) == (1, 1, 1, 64), (
                mode,
                entry,
                result.stderr,
            )
    print(
        "Allocation probe: all 6 entry points count once (64 requested bytes), baseline and all 7 perturbations"
    )


@pytest.mark.parametrize("version", [1, 2])
def test_changed_input_requires_rebaseline_without_deltas(version):
    module = _load_runner()
    base = {
        "run": {
            "commit": "base",
            "env": {
                "fingerprint": "same",
                "compiler": "test",
                "compiler_provenance": "dart-perf-build/1",
                "compiler_sha": "1" * 64,
                "valgrind_sha": "1" * 64,
                "callgrind_sha": "1" * 64,
            },
        },
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
                    "bytes_per_step": 0,
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
            "bytes": None,
            "guards_equal": True,
            "class": "behaviour-change",
        }
        assert "input_sha changed" in module.markdown(record)


@pytest.mark.parametrize("name", ["s1p/dart", "dyn", "lcp", "gzb", "robot"])
@pytest.mark.parametrize("changed", [False, True])
def test_run_workload_stamp_controls_rebaseline(monkeypatch, tmp_path, name, changed):
    module = _load_runner()
    _fake_valgrind(module, monkeypatch, tmp_path)
    row = module.select_rows(name)[0]
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "installed",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path / "run"),
            "--rows",
            name,
            "--no-perturb",
            "--shim",
            str(tmp_path / "allocshim.so"),
        ]
    )
    args.shim.write_bytes(b"shim")
    library = tmp_path / "lib/libdart.so"
    library.parent.mkdir()
    library.write_bytes(b"DART")
    drivers = {module.PB, row.driver}
    binary = tmp_path / "bin"
    binary.mkdir()
    for driver in drivers:
        (binary / driver).write_bytes(b"binary")
    stamp = {
        "schema": "dart-perf-build/1",
        "commit": "installed",
        "compiler": "test",
        "compiler_sha": "1" * 64,
        "pixi_lock_sha": "lock",
        "preset": "perf-1",
        "libdart_sha": module.sha(b"DART"),
        "libraries": {"lib/libdart.so": module.sha(b"DART")},
        "binaries": {driver: module.sha(b"binary") for driver in drivers},
        "workload_sources": {
            driver: module.sha(b"base workload")
            for driver in drivers & module.WORKLOAD_SOURCES.keys()
        },
    }
    path = tmp_path / "share/dart/perf-build.json"
    path.parent.mkdir(parents=True)
    module.write_json(path, stamp)
    monkeypatch.setattr(
        module,
        "command_output",
        lambda command: (
            "installed"
            if command[0] == "git"
            else "valgrind-3.22.0" if command[0] == module.VALGRIND else "glibc 2.39"
        ),
    )
    monkeypatch.setattr(module, "execute", lambda *args: "Guest CPU: test\n")
    metrics = (
        _micro_record(module, name)["results"][0]["head"]
        if not row.det
        else {
            "ir_per_step": 100,
            "allocs_per_step": 0,
            "bytes_per_step": 0,
            "guards": {
                "hash": "same",
                "finite": True,
                "contacts": 0,
                "cap_hit": False,
                "resting": "0/1",
            },
        }
    )
    monkeypatch.setattr(
        module,
        "measure",
        lambda row, args, world: {
            "row": row.row,
            "det": row.det,
            "version": row.version,
            "parity": "",
            "status": "ok",
            "gated": True,
            "method": "slope",
            "input_sha": "scene input",
            "head": copy.deepcopy(metrics),
        },
    )
    base = module.run_arm(args)
    # Separate install RUNPATHs change executable bytes without changing inputs.
    (binary / module.PB).write_bytes(b"head driver with different RUNPATH")
    stamp["binaries"][module.PB] = module.sha((binary / module.PB).read_bytes())
    if changed:
        stamp["workload_sources"] = {
            driver: module.sha(b"head workload") for driver in stamp["workload_sources"]
        }
    module.write_json(path, stamp)
    head = module.run_arm(args)
    assert base["run"]["env"]["harness_sha"] == head["run"]["env"]["harness_sha"]
    assert base["run"]["env"]["fingerprint"] == head["run"]["env"]["fingerprint"]
    result = module.compare(base, head)
    workload_changed = changed
    assert result["verdict"]["status"] == ("FAIL" if workload_changed else "PASS")
    delta = result["results"][0]["delta"]
    assert delta["class"] == ("behaviour-change" if workload_changed else "gated")
    assert head["results"][0]["workload_sha"] == stamp["workload_sources"][row.driver]
    assert head["results"][0]["input_sha"] == module.sha(
        json.dumps(["scene input", head["results"][0]["workload_sha"]]).encode()
    )
    if workload_changed:
        assert all(delta[key] is None for key in ("ir", "allocs", "bytes"))
        assert "Rebaseline-Rationale required" in result["verdict"]["failures"][0]
        assert (
            module.compare(
                base, head, f"Perf-Regression-Rationale: {row.key}: workload changed"
            )["verdict"]["status"]
            == "FAIL"
        )
        assert (
            module.compare(
                base, head, f"Rebaseline-Rationale: {row.key}: workload changed"
            )["verdict"]["status"]
            == "PASS"
        )
    stamp.pop("workload_sources")
    module.write_json(path, stamp)
    with pytest.raises(ValueError, match="invalid installed build provenance"):
        module.run_arm(args)


@pytest.mark.parametrize(
    "changed_path",
    sorted(
        {path for paths in _load_runner().WORKLOAD_SOURCES.values() for path in paths}
    ),
)
def test_workload_hashes_cover_sources_and_headers(tmp_path, changed_path):
    module = _load_runner()
    for paths in module.WORKLOAD_SOURCES.values():
        for name in paths:
            path = tmp_path / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"workload")
    drivers = [*module.WORKLOAD_SOURCES, module.PB]
    base = module.workload_hashes(tmp_path, drivers)
    assert module.PB in base
    # Library implementation changes are what the harness measures.
    library = tmp_path / "dart/library.cpp"
    library.parent.mkdir()
    library.write_bytes(b"changed DART")
    assert module.workload_hashes(tmp_path, drivers) == base
    path = tmp_path / changed_path
    path.write_bytes(b"changed workload")
    head = module.workload_hashes(tmp_path, drivers)
    for driver, paths in module.WORKLOAD_SOURCES.items():
        assert (base[driver] != head[driver]) == (changed_path in paths)
    if path.suffix == ".hpp":
        path.unlink()
        assert module.workload_hashes(tmp_path, drivers) != base
    added = tmp_path / "examples/contact_benchmark/new_case.hpp"
    added.write_bytes(b"new case")
    assert module.workload_hashes(tmp_path, drivers)[module.CB] != head[module.CB]
    # The target globs its sources, so a renamed entry point is a change, not
    # a missing source; a directory without sources is.
    main = tmp_path / "examples/contact_benchmark/main.cpp"
    main.rename(main.with_name("entry.cpp"))
    renamed = module.workload_hashes(tmp_path, drivers)[module.CB]
    assert renamed != head[module.CB]
    main.with_name("entry.cpp").unlink()
    with pytest.raises(ValueError, match="missing workload source"):
        module.workload_hashes(tmp_path, drivers)


@pytest.mark.parametrize("arguments", [False, True])
def test_workload_hashes_cover_compile_commands(tmp_path, arguments):
    module = _load_runner()
    drivers = list(module.WORKLOAD_SOURCES)

    def arm(label):
        source = tmp_path / label
        build, prefix = source / "build", source / "install"
        build.mkdir(parents=True)
        for paths in module.WORKLOAD_SOURCES.values():
            for name in paths:
                path = source / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"workload")
        # contact_benchmark globs every translation unit, not just main.cpp.
        (source / "examples/contact_benchmark/extra.cpp").write_bytes(b"extra")
        entries = []
        for path in sorted(source.rglob("*.cpp")):
            command = [
                "/usr/bin/c++",
                "-O3",
                f"-I{source}/include",
                f"-I{build}/include",
                f"-I{prefix}/include",
                "-o",
                str(build / (path.stem + ".o")),
                "-c",
                str(path),
            ]
            entries.append(
                {
                    "directory": str(build),
                    "file": os.path.relpath(path, build),
                    **(
                        {"arguments": command}
                        if arguments
                        else {"command": " ".join(command)}
                    ),
                }
            )
        entries.append(
            {
                "directory": str(build),
                "file": str(source / "dart/library.cpp"),
                "command": "/usr/bin/c++ -O3 -c dart/library.cpp",
            }
        )
        module.write_json(build / "compile_commands.json", entries)
        return source, build, prefix, entries

    source, build, prefix, entries = arm("base")
    base = module.workload_hashes(source, drivers, build, prefix)
    other_source, other_build, other_prefix, other_entries = arm("head")
    module.write_json(other_build / "compile_commands.json", other_entries[::-1])
    assert (
        module.workload_hashes(other_source, drivers, other_build, other_prefix) == base
    )

    # Library flags belong to measured code and must not rebaseline workloads.
    entries[-1]["command"] = "/usr/bin/c++ -O0 -c dart/library.cpp"
    module.write_json(build / "compile_commands.json", entries)
    assert module.workload_hashes(source, drivers, build, prefix) == base

    for entry in entries[:-1]:
        changed = copy.deepcopy(entries)
        index = entries.index(entry)
        if arguments:
            changed[index]["arguments"][1] = "-O2"
        else:
            changed[index]["command"] = entry["command"].replace("-O3", "-O2")
        module.write_json(build / "compile_commands.json", changed)
        head = module.workload_hashes(source, drivers, build, prefix)
        name = (build / entry["file"]).resolve().relative_to(source).as_posix()
        for driver in drivers:
            owns_source = name in module.WORKLOAD_SOURCES[driver] or (
                driver == module.CB and name.endswith("/extra.cpp")
            )
            assert (head[driver] != base[driver]) == owns_source

    # A root Release flag change reaches every driver's compilation.
    for entry in entries[:-1]:
        if arguments:
            entry["arguments"][1] = "-O2"
        else:
            entry["command"] = entry["command"].replace("-O3", "-O2")
    module.write_json(build / "compile_commands.json", entries)
    head = module.workload_hashes(source, drivers, build, prefix)
    assert all(head[driver] != base[driver] for driver in drivers)

    module.write_json(build / "compile_commands.json", entries[1:])
    with pytest.raises(ValueError, match="missing workload compile command"):
        module.workload_hashes(source, drivers, build, prefix)


@pytest.mark.parametrize("pairs", [None, 0, 3])
def test_contact_guard_values_survive_comparison_and_markdown(pairs):
    module = _load_runner()

    def guard(contacts, resting, cap_hit, pairs):
        text = (
            "Final State Hash: 0x1\nFinal State Finite: true\n"
            f"Final Contacts: {contacts}\nFinal Resting: {resting}\n"
            f"Final Contact Cap Hit: {cap_hit}\n"
        )
        if pairs is not None:
            text += f"Final Contact Pairs: {pairs}\n"
        return module.guards(text)

    base = _micro_record(module, "dyn")
    base["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    row = base["results"][0]
    row.update(row="contact", det="dart")
    row["head"]["guards"] = guard(7, "2 / 5 mobile", "false", pairs)
    head = copy.deepcopy(base)
    head_guard = head["results"][0]["head"]["guards"] = guard(
        9, "3 / 5 mobile", "true", None if pairs is None else pairs + 1
    )
    record = module.compare(base, head)
    saved = json.loads(json.dumps(record))["results"][0]
    assert saved["parent"]["guards"] == row["head"]["guards"]
    assert saved["head"]["guards"] == head_guard
    report = module.markdown(record)
    for label, values in (("base", (7, "2/5", "false")), ("head", (9, "3/5", "true"))):
        contacts, resting, cap_hit = values
        expected_pairs = (
            pairs if label == "base" else None if pairs is None else pairs + 1
        )
        text = f"{label}: contacts={contacts}, "
        if expected_pairs is not None:
            text += f"pairs={expected_pairs}, "
        assert text + f"resting={resting}, cap_hit={cap_hit}" in report
    if pairs is not None:
        # Pair accounting alone is also a correctness guard.
        head = copy.deepcopy(base)
        head["results"][0]["head"]["guards"]["pairs"] += 1
        assert (
            module.compare(base, head)["results"][0]["delta"]["class"]
            == "behaviour-change"
        )


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


@pytest.mark.parametrize(
    "changed",
    [
        None,
        "hash",
        "contacts",
        "pairs",
        "cap_hit",
        "resting",
        "finite",
        "max_penetration",
        "checkpoints",
    ],
)
def test_multithread_parity_compares_all_guards(monkeypatch, tmp_path, changed):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path / "run"),
            "--rows",
            "mt4-s3w",
            "--no-perturb",
            "--shim",
            str(tmp_path / "allocshim.so"),
        ]
    )
    args.shim.write_bytes(b"shim")
    monkeypatch.setattr(module, "command_output", lambda command: "HEAD")
    monkeypatch.setattr(module, "fingerprint", lambda args, provenance: {})
    monkeypatch.setattr(
        module,
        "installed_provenance",
        lambda args: {"workload_sources": {module.CB: module.sha(b"workload")}},
    )

    def measure(row, args, world):
        guards = dict(
            hash="same", contacts=1, pairs=1, cap_hit=False, resting="0/1", finite=True
        )
        metrics = {
            "guards": guards,
            "max_penetration": 0.1,
            "checkpoints": [{"step": 100, "max_penetration": 0.1}],
        }
        if row.parity and changed:
            target = (
                metrics if changed in ("max_penetration", "checkpoints") else guards
            )
            target[changed] = {
                "hash": "different",
                "contacts": 2,
                "pairs": 2,
                "cap_hit": True,
                "resting": "1/1",
                "finite": False,
                "max_penetration": 0.2,
                "checkpoints": [{"step": 100, "max_penetration": 0.2}],
            }[changed]
        return {
            "row": row.row,
            "det": row.det,
            "parity": row.parity,
            "status": "ok",
            "input_sha": "scene input",
            "head": metrics,
        }

    monkeypatch.setattr(module, "measure", measure)
    results = module.run_arm(args)["results"]
    assert len(results) == 2  # The serial counterpart is added automatically.
    parallel = next(result for result in results if result["parity"])
    serial = next(result for result in results if not result["parity"])
    assert serial["status"] == "ok"
    assert parallel["status"] == ("broken" if changed else "ok")
    if changed:
        assert parallel["error"] == "mt4 guard parity missing or unequal"


def test_measure_gates_only_on_its_own_perturbation_pass(monkeypatch, tmp_path):
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
            "--no-perturb",
        ]
    )
    row = module.select_rows("pend")[0]
    metrics = {"guards": {"finite": True}, "allocs": 0}
    monkeypatch.setattr(module, "native", lambda *args: copy.deepcopy(metrics))
    # Without this run's heap-layout checks, no row gates.
    assert module.measure(row, args, tmp_path)["gated"] is False
    # The checks run by default.
    assert (
        module.parser()
        .parse_args(["run", "--commit", "HEAD", "--prefix", ".", "--output-dir", "."])
        .perturb
    )
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
    # A perturbed run whose time stops advancing disqualifies the row too.
    monkeypatch.setattr(
        module,
        "native",
        lambda row, args, world, config="": {
            **metrics,
            "time_advanced": not config,
        },
    )
    assert module.measure(row, args, tmp_path)["gated"] is False
    # Requested bytes that change with the heap layout also disqualify the row.
    monkeypatch.setattr(
        module,
        "native",
        lambda row, args, world, config="": {**metrics, "bytes": 64 + bool(config)},
    )
    assert module.measure(row, args, tmp_path)["gated"] is False


@pytest.mark.parametrize("field", ["max_penetration", "checkpoints"])
def test_perturbation_checks_separately_stored_guard_evidence(
    monkeypatch, tmp_path, field
):
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
    original = {
        "guards": {"hash": "same", "finite": True},
        "allocs": 0,
        "bytes": 0,
        "max_penetration": 0.1,
        "checkpoints": [{"step": 100, "max_penetration": 0.1}],
    }
    altered = copy.deepcopy(original)
    altered[field] = (
        0.2 if field == "max_penetration" else [{"step": 100, "max_penetration": 0.2}]
    )
    monkeypatch.setattr(
        module,
        "native",
        lambda row, args, world, config="": copy.deepcopy(
            altered if config else original
        ),
    )
    measured = module.measure(module.select_rows("s3w/dart")[0], args, tmp_path)
    assert measured["gated"] is False
    assert all(
        not sample["stable"] and sample[field] == altered[field]
        for sample in measured["perturbations"].values()
    )


@pytest.mark.parametrize("field", ["max_penetration", "checkpoints"])
def test_compare_rebaselines_separately_stored_guard_evidence(field):
    module = _load_runner()
    base = _publication_fixture()
    base["run"]["env"].update(_micro_record(module, "dyn")["run"]["env"])
    base["results"][0]["head"].update(
        bytes_per_step=0,
        max_penetration=0.1,
        checkpoints=[{"step": 100, "max_penetration": 0.1}],
    )
    head = copy.deepcopy(base)
    head["results"][0]["head"][field] = (
        0.2 if field == "max_penetration" else [{"step": 100, "max_penetration": 0.2}]
    )
    result = module.compare(base, head)
    assert result["verdict"]["status"] == "FAIL"
    assert result["results"][0]["delta"]["guards_equal"] is False
    assert result["results"][0]["failures"] == [
        "guards changed; Rebaseline-Rationale required"
    ]
    assert (
        module.compare(
            base, head, "Rebaseline-Rationale: s3w/dart: intended guard change"
        )["verdict"]["status"]
        == "PASS"
    )


def test_environment_ignores_inherited_library_path(monkeypatch, tmp_path):
    module = _load_runner()
    monkeypatch.setenv("LD_LIBRARY_PATH", "/elsewhere/lib")
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / "env"))
    env = module.environment(tmp_path / "prefix")
    assert env["LD_LIBRARY_PATH"] == f"{tmp_path}/prefix/lib:{tmp_path}/env/lib"


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


@pytest.mark.parametrize(
    "returncode, guard_output, correctness",
    [
        (1, "complete", True),
        (1, "missing", False),
        (1, "partial", False),
        (1, "finite", False),
        (1, "invalid", False),
        # contact_benchmark exits 2 after printing complete non-finite guards.
        (2, "complete", True),
        (2, "finite", False),
        (2, "time", True),
        (2, "time-partial", False),
        (2, "time-invalid", False),
        (2, "time-nonfinite", True),
        (3, "missing", False),
    ],
)
def test_driver_guard_failure_exit_is_correctness_failure(
    monkeypatch, tmp_path, capsys, returncode, guard_output, correctness
):
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
        ]
    )
    args.bin_dir = tmp_path
    row = module.select_rows("gzb")[0]
    finite = "true"
    output = "complete"
    exit_status = 0

    def popen(command, **kwargs):
        text = (
            "Avg Step Time: 1.0 ms/step\n"
            f"STEPALLOC steps=5 measured=3 allocs=0 bytes=0 libdart={tmp_path}/lib/libdart.so\n"
            "PERFTIME maxrss_kb=100\n"
        )
        if output != "missing":
            text += (
                "Final State Hash: 0x0123456789abcdef\n"
                f"Final State Finite: {finite}\n"
                "Final Contacts: 1\n"
                "Final Contact Cap Hit: false\n"
            )
            if output not in ("partial", "time-partial"):
                text += "Final Resting: 0 / 1\n"
        if output.startswith("time"):
            text += "Time Advanced:      false\n"
        kwargs["stdout"].write(text)
        return module.argparse.Namespace(
            returncode=exit_status, wait=lambda **kwargs: None
        )

    monkeypatch.setattr(module.subprocess, "Popen", popen)
    monkeypatch.setattr(
        module, "callgrind", lambda row, args, world, steps: {"Ir": steps * 100}
    )
    supported = module.measure(row, args, tmp_path)
    assert supported["status"] == "ok"

    # A completed correctness failure must stop before additional measurements.
    def unexpected_callgrind(*args):
        pytest.fail("Callgrind ran after a correctness failure")

    monkeypatch.setattr(module, "callgrind", unexpected_callgrind)
    output, exit_status = guard_output, returncode
    if output in ("finite", "time", "time-partial"):
        finite = "true"
    elif output in ("invalid", "time-invalid"):
        finite = "invalid"
    else:
        finite = "false"
    broken = module.measure(row, args, tmp_path)
    assert broken["status"] == "broken"
    if correctness:
        assert broken["head"]["guards"] == module.guards(
            module.native_log(args, row).read_text()
        )
        assert broken["head"]["guards"]["finite"] == (finite == "true")
        assert broken["error"] == (
            "simulation time did not advance"
            if output == "time"
            else "non-finite state"
        )
        assert "error_kind" not in broken
        assert "perturbations" not in broken
    else:
        assert broken["error_kind"] == "infrastructure"
        assert broken["error"].startswith(f"exit {returncode}:")

    env = {
        "fingerprint": "same",
        "valgrind": "test",
        "compiler": "test",
        "compiler_provenance": "dart-perf-build/1",
        "compiler_sha": "1" * 64,
        "valgrind_sha": "1" * 64,
        "callgrind_sha": "1" * 64,
        "glibc": "test",
        "preset": "perf-1",
    }
    base = {
        "schema": "dart-perf/1",
        "run": {"commit": "base", "env": env},
        "results": [supported],
    }
    head = {
        "schema": "dart-perf/1",
        "run": {"commit": "head", "env": env},
        "results": [broken],
    }
    comparison = module.compare(base, head)
    status = "FAIL" if correctness else "ERROR"
    assert comparison["verdict"]["status"] == status
    assert comparison["results"][0]["delta"]["class"] == "broken"
    base_path, head_path = tmp_path / "base.json", tmp_path / "head.json"
    module.write_json(base_path, base)
    module.write_json(head_path, head)
    assert module.main(
        ["compare", "--base", str(base_path), "--head", str(head_path)]
    ) == (1 if correctness else 2)
    assert status in capsys.readouterr().out


@pytest.mark.parametrize(
    "phase", ["native", "perturb", "callgrind-before", "callgrind-after"]
)
def test_unsupported_driver_rows_are_not_infrastructure_errors(
    monkeypatch, tmp_path, capsys, phase
):
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
    args.bin_dir = tmp_path
    row = module.select_rows("gzb")[0]
    metrics = {
        "guards": {
            "hash": "0x1",
            "finite": True,
            "contacts": 1,
            "cap_hit": False,
            "resting": "0/1",
        },
        "allocs": 0,
        "allocs_per_step": 0,
        "bytes_per_step": 0,
    }

    def popen(command, **kwargs):
        kwargs["stdout"].write("UNSUPPORTED: maxNumContactsPerPair\n")
        return module.argparse.Namespace(returncode=3, wait=lambda **kwargs: None)

    monkeypatch.setattr(module.subprocess, "Popen", popen)
    native, callgrind = module.native, module.callgrind

    def run_native(row, args, world, config=""):
        if phase == "native" or phase == "perturb" and config:
            return native(row, args, world, config)
        return copy.deepcopy(metrics)

    def run_callgrind(row, args, world, steps):
        if phase == "callgrind-before" or steps == row.warmup + row.steps:
            return callgrind(row, args, world, steps)
        return {"Ir": 100}

    monkeypatch.setattr(module, "native", run_native)
    monkeypatch.setattr(module, "callgrind", run_callgrind)
    unsupported = module.measure(row, args, tmp_path)
    assert unsupported["status"] == "unsupported"
    assert unsupported["error"] == "maxNumContactsPerPair"
    assert "error_kind" not in unsupported
    assert not unsupported["head"] and not unsupported["perturbations"]
    monkeypatch.setattr(module, "run_arm", lambda args: {"results": [unsupported]})
    assert module.main(argv) == 0
    assert "maxNumContactsPerPair" in capsys.readouterr().out

    env = {
        "fingerprint": "same",
        "compiler": "test",
        "compiler_provenance": "dart-perf-build/1",
        "compiler_sha": "1" * 64,
        "valgrind_sha": "1" * 64,
        "callgrind_sha": "1" * 64,
    }
    base = {"run": {"commit": "base", "env": env}, "results": [unsupported]}
    unchanged = module.compare(base, copy.deepcopy(base))
    assert unchanged["verdict"]["status"] == "PASS"
    assert unchanged["verdict"]["ir_geomean"] is None
    assert unchanged["results"][0]["delta"]["class"] == "unsupported"
    assert unchanged["results"][0]["gated"] is False
    supported = copy.deepcopy(unsupported)
    supported.update(status="ok", gated=True, head={**metrics, "ir_per_step": 100})
    head = {"run": {"commit": "head", "env": env}, "results": [supported]}
    comparison = module.compare(base, head)
    assert comparison["verdict"]["status"] == "PASS"
    assert comparison["results"][0]["delta"]["class"] == "new"
    # Losing support in the head remains a policy failure, not an infrastructure error.
    assert module.compare(head, base)["verdict"]["status"] == "FAIL"


@pytest.mark.parametrize(
    "returncode, text", [(3, "API unavailable\n"), (1, "UNSUPPORTED: optional API\n")]
)
def test_unsupported_requires_exit_three_and_marker(
    monkeypatch, tmp_path, returncode, text
):
    module = _load_runner()

    def popen(command, **kwargs):
        kwargs["stdout"].write(text)
        return module.argparse.Namespace(
            returncode=returncode, wait=lambda **kwargs: None
        )

    monkeypatch.setattr(module.subprocess, "Popen", popen)
    with pytest.raises(ValueError, match=f"exit {returncode}") as error:
        module.execute(["driver"], {}, tmp_path / "driver.log", 10)
    assert not isinstance(error.value, module.UnsupportedRow)


@pytest.mark.parametrize(
    "layout",
    ["local", "fallback", "explicit", "missing-shim", "missing-heappad", "no-perturb"],
)
def test_run_resolves_shims_and_reports_missing_options(
    monkeypatch, tmp_path, capsys, layout
):
    module = _load_runner()
    root = tmp_path / "repo"
    world = root / "tests/benchmark/worlds/3k_shapes.sdf.gz"
    world.parent.mkdir(parents=True)
    world.write_bytes(
        (module.ROOT / "tests/benchmark/worlds/3k_shapes.sdf.gz").read_bytes()
    )
    monkeypatch.setattr(module, "ROOT", root)
    monkeypatch.setattr(module, "command_output", lambda command: "HEAD")
    monkeypatch.setattr(module, "fingerprint", lambda args, provenance: {})
    monkeypatch.setattr(
        module,
        "installed_provenance",
        lambda args: {"workload_sources": {module.PB: "driver source"}},
    )
    monkeypatch.setattr(
        module,
        "measure",
        lambda row, args, world: {
            "row": row.row,
            "det": row.det,
            "parity": "",
            "status": "ok",
            "gated": True,
            "input_sha": "scene input",
        },
    )
    prefix = tmp_path / "output/a"
    argv = [
        "run",
        "--commit",
        "HEAD",
        "--prefix",
        str(prefix),
        "--output-dir",
        str(tmp_path / "run"),
        "--rows",
        "gzb",
    ]
    chosen = {}
    for option, name in (("shim", "allocshim"), ("heappad", "heappad")):
        local = prefix.parent / "shims" / f"{name}.so"
        fallback = root / "build/perf" / f"lib{name}.so"
        path = fallback if layout in ("fallback", f"missing-{option}") else local
        if layout == "explicit":
            local.parent.mkdir(parents=True, exist_ok=True)
            local.write_bytes(b"local shim")
            path = tmp_path / f"custom-{name}.so"
            argv += [f"--{option}", str(path)]
        if layout != f"missing-{option}" and not (
            layout == "no-perturb" and option == "heappad"
        ):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"shim")
        chosen[option] = path
    if layout == "no-perturb":
        argv.append("--no-perturb")
    if layout.startswith("missing-"):
        option = layout.removeprefix("missing-")
        assert module.main(argv) == 2
        error = capsys.readouterr().err
        assert f"--{option}" in error and str(chosen[option]) in error
    else:
        args = module.parser().parse_args(argv)
        record = module.run_arm(args)
        assert record["results"][0]["status"] == "ok"
        assert args.shim == chosen["shim"]
        if layout != "no-perturb":
            assert args.heappad == chosen["heappad"]


def test_compare_and_local_infrastructure_exit_two(monkeypatch, tmp_path, capsys):
    module = _load_runner()
    env = {
        "fingerprint": "same",
        "valgrind": "test",
        "compiler": "test",
        "compiler_provenance": "dart-perf-build/1",
        "compiler_sha": "1" * 64,
        "valgrind_sha": "1" * 64,
        "callgrind_sha": "1" * 64,
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
        "run": {
            "commit": "HEAD",
            "env": {
                "fingerprint": "same",
                "compiler": "test",
                "compiler_provenance": "dart-perf-build/1",
                "compiler_sha": "1" * 64,
                "valgrind_sha": "1" * 64,
                "callgrind_sha": "1" * 64,
            },
        },
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
                    "bytes_per_step": 0,
                    "cases": cases,
                    "guards": {
                        "hash": {case: "0x0123456789abcdef" for case in cases},
                        "finite": True,
                    },
                },
            }
        ],
    }


def test_compare_rejects_mixed_measurement_methods(tmp_path, capsys):
    module = _load_runner()
    slope = {"schema": "dart-perf/1", **_micro_record(module, "dyn")}
    slope["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    native = copy.deepcopy(slope)
    native["results"][0].update(method="native", expected_ir=False)
    native["results"][0]["head"].pop("ir_per_step")
    for base, head in ((native, slope), (slope, native)):
        record = module.compare(base, head)
        assert record["verdict"]["status"] == "ERROR"
        assert record["results"] == []
        assert "incompatible measurement method/expected_ir" in module.markdown(record)
        base_path, head_path = tmp_path / "base.json", tmp_path / "head.json"
        module.write_json(base_path, base)
        module.write_json(head_path, head)
        assert (
            module.main(["compare", "--base", str(base_path), "--head", str(head_path)])
            == 2
        )
        assert "incompatible measurement method/expected_ir" in capsys.readouterr().out
    for field, value in (("method", "native"), ("expected_ir", False)):
        head = copy.deepcopy(slope)
        head["results"][0][field] = value
        assert module.compare(slope, head)["verdict"]["status"] == "ERROR"
    # Rows that are native on both sides collect no Ir and still compare.
    assert module.compare(native, native)["verdict"]["status"] == "PASS"


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


def test_requested_bytes_gate_report_and_missing_measurements():
    module = _load_runner()
    base = _micro_record(module, "dyn")
    base["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    base["results"][0]["head"]["bytes_per_step"] = 100
    head = copy.deepcopy(base)
    head["results"][0]["head"]["bytes_per_step"] = 128
    result = module.compare(base, head)
    assert result["verdict"]["status"] == "FAIL"
    assert result["results"][0]["delta"]["allocs"] == 0
    assert result["results"][0]["delta"]["bytes"] == 28
    assert result["verdict"]["failures"] == ["dyn: requested bytes +28/step"]
    report = module.markdown(result)
    assert "Bytes/step delta" in report and "| +28 |" in report
    assert "1 regressed." in report
    for row, status in (("dyn", "PASS"), ("lcp", "FAIL")):
        result = module.compare(
            base, head, f"Perf-Regression-Rationale: {row}: intended"
        )
        assert result["verdict"]["status"] == status
        assert "1 regressed." in module.markdown(result)
    head["results"][0]["head"]["bytes_per_step"] = 72
    result = module.compare(base, head)
    assert result["verdict"]["status"] == "PASS"
    assert result["results"][0]["delta"]["bytes"] == -28
    report = module.markdown(result)
    assert "| -28 |" in report and "1 improved." in report
    head["results"][0]["head"]["bytes_per_step"] = 128
    base["results"][0]["gated"] = False
    assert module.compare(base, head)["verdict"]["status"] == "PASS"
    base["results"][0]["gated"] = True

    # Missing/invalid bytes follow the same policy as allocation counts on either arm.
    for field in ("allocs_per_step", "bytes_per_step"):
        for invalid in ("missing", None, -1, float("nan"), float("inf")):
            for arms in (("base",), ("head",), ("base", "head")):
                records = {"base": copy.deepcopy(base), "head": copy.deepcopy(head)}
                for arm in arms:
                    row = records[arm]["results"][0]
                    if invalid == "missing":
                        row["head"].pop(field)
                    else:
                        row["head"][field] = invalid
                    assert not module.complete(row, row["head"])
                result = module.compare(
                    records["base"],
                    records["head"],
                    "Perf-Regression-Rationale: dyn: intended",
                )
                assert result["verdict"]["status"] == "FAIL"
                assert result["results"][0]["delta"]["class"] == "broken"
                assert result["results"][0]["delta"]["bytes"] is None
                assert "Bytes/step delta" in module.markdown(result)


@pytest.mark.parametrize("name", ["dyn", "lcp"])
def test_changed_workload_needs_rationale_without_micro_instrumentation(name):
    module = _load_runner()
    base = _micro_record(module, name)
    head = copy.deepcopy(base)
    base["results"][0]["head"].update(
        micro_instrumented=False,
        guards=None,
        allocs=None,
        bytes=None,
        allocs_per_step=None,
        bytes_per_step=None,
    )
    head["results"][0]["input_sha"] = "changed workload"
    result = module.compare(base, head)
    row = result["results"][0]
    assert result["verdict"]["status"] == "FAIL"
    assert row["delta"]["class"] == "behaviour-change"
    assert row["failures"] == ["input_sha changed; Rebaseline-Rationale required"]
    acknowledged = module.compare(
        base, head, f"Rebaseline-Rationale: {name}: new workload"
    )
    assert acknowledged["verdict"]["status"] == "PASS"
    assert acknowledged["results"][0]["gated"] is False


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


@pytest.mark.parametrize("defect", ["case error", "iterations", "malformed"])
def test_micro_case_errors_are_correctness_failures(
    monkeypatch, tmp_path, capsys, defect
):
    module = _load_runner()
    row = module.select_rows("lcp")[0]
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--no-perturb",
        ]
    )
    args.bin_dir = tmp_path
    case = module.MICRO_CASES["lcp"][0]

    def execute(command, env, log, timeout):
        output = Path(
            next(
                item.split("=", 1)[1]
                for item in command
                if item.startswith("--benchmark_out=")
            )
        )
        item = {"name": case, "iterations": 0}
        if defect == "case error":
            # Google Benchmark errors can omit the normal iteration metrics.
            item = {
                "name": case,
                "error_occurred": True,
                "error_message": "native solver failed",
            }
        elif defect == "malformed":
            item.pop("iterations")
        module.write_json(output, {"benchmarks": [item]})
        return f"STEPALLOC steps=0 measured=0 allocs=0 bytes=0 libdart={tmp_path}/libdart.so\n"

    monkeypatch.setattr(module, "execute", execute)
    broken = module.measure(row, args, tmp_path)
    assert broken["status"] == "broken"
    correctness = defect == "case error"
    assert (broken.get("error_kind") != "infrastructure") == correctness
    if correctness:
        assert case in broken["error"]
        assert "native solver failed" in broken["error"]
    base = {"schema": "dart-perf/1", **_micro_record(module, "lcp")}
    base["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    head = {**base, "results": [broken]}
    comparison = module.compare(base, head)
    assert comparison["verdict"]["status"] == ("FAIL" if correctness else "ERROR")
    assert comparison["results"][0]["delta"]["class"] == "broken"
    base_path, head_path = tmp_path / "base.json", tmp_path / "head.json"
    module.write_json(base_path, base)
    module.write_json(head_path, head)
    assert module.main(
        ["compare", "--base", str(base_path), "--head", str(head_path)]
    ) == (1 if correctness else 2)
    assert ("FAIL" if correctness else "ERROR") in capsys.readouterr().out
    monkeypatch.setattr(module, "run_arm", lambda args: head)
    assert module.main(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--nightly",
        ]
    ) == (1 if correctness else 2)
    head["run"] = _publication_fixture()["run"]
    module.write_json(head_path, head)
    if correctness:
        assert broken["head"] == {}
        published = module.publication_record(head_path, "nightly")
        module.write_publication(tmp_path / "pages", published)
        saved = list(
            (tmp_path / "pages/performance/records/main").glob("*/*-nightly.json")
        )
        assert len(saved) == 1
        assert (
            json.loads(saved[0].read_text())["results"][0]["error"] == broken["error"]
        )
    else:
        with pytest.raises(ValueError, match="invalid or incomplete publication row"):
            module.publication_record(head_path, "nightly")


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


@pytest.mark.parametrize("revision", ["HEAD", "annotated-release"])
@pytest.mark.parametrize("smoke", [False, True])
@pytest.mark.parametrize(
    "rows, portable_only, expected_drivers",
    [
        (
            "",
            False,
            {"contact_benchmark", "BM_INTEGRATION_kinematics", "BM_UNIT_dantzig_lcp"},
        ),
        ("gzb", False, set()),
        ("robot", False, set()),
        ("dyn", False, {"BM_INTEGRATION_kinematics"}),
        ("lcp", False, {"BM_UNIT_dantzig_lcp"}),
        ("mt4-s3w", False, {"contact_benchmark"}),
        (
            "dyn,lcp,pend",
            False,
            {"contact_benchmark", "BM_INTEGRATION_kinematics", "BM_UNIT_dantzig_lcp"},
        ),
        ("", True, set()),
        ("gzb", True, set()),
        ("dyn", True, {"BM_INTEGRATION_kinematics"}),
    ],
)
def test_local_uses_independent_source_and_cmake_caches(
    monkeypatch, tmp_path, revision, smoke, rows, portable_only, expected_drivers
):
    import io
    import tarfile
    from types import SimpleNamespace

    module = _load_runner()
    args = module.parser().parse_args(
        [
            "local",
            "--base",
            "missing-base" if smoke else revision,
            "--head",
            revision,
            "--output-dir",
            str(tmp_path),
            "--rows",
            rows,
            *(["--smoke"] if smoke else []),
        ]
    )
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / "dependencies"))

    resolved = []

    def command_output(command):
        # An annotated tag names a tag object unless explicitly peeled.
        resolved.append(command[-1])
        return "tag-object" if command[-1] == "annotated-release" else "commit"

    monkeypatch.setattr(module, "command_output", command_output)
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(returncode=int(portable_only)),
    )
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w") as contents:
        entry = tarfile.TarInfo("CMakeLists.txt")
        entry.size = 0
        contents.addfile(entry, io.BytesIO())
        for name in sorted(
            {path for paths in module.WORKLOAD_SOURCES.values() for path in paths}
        ):
            data = f"archived {name}".encode()
            entry = tarfile.TarInfo(name)
            entry.size = len(data)
            contents.addfile(entry, io.BytesIO(data))
    monkeypatch.setattr(
        module.subprocess, "check_output", lambda *args, **kwargs: archive.getvalue()
    )
    configurations = []
    portable_hashes = []

    def execute(command, env, log, timeout, **kwargs):
        assert kwargs.get("build", False)
        if command[:3] == ["cmake", "-G", "Ninja"]:
            assert "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" in command
            source = Path(command[command.index("-S") + 1])
            build = Path(command[command.index("-B") + 1])
            build.mkdir()
            compiler = build / "CMakeFiles/4.0/CMakeCXXCompiler.cmake"
            compiler.parent.mkdir(parents=True)
            (tmp_path / "c++").write_bytes(b"build compiler")
            compiler.write_text(
                f'set(CMAKE_CXX_COMPILER "{tmp_path / "c++"}")\n'
                'set(CMAKE_CXX_COMPILER_ID "GNU")\nset(CMAKE_CXX_COMPILER_VERSION "13.3.0")\n'
            )
            workload_source = source if source.name.startswith("src-") else module.ROOT
            compiled_drivers = (
                expected_drivers if source.name.startswith("src-") else {module.PB}
            )
            module.write_json(
                build / "compile_commands.json",
                [
                    {
                        "directory": str(build),
                        "file": str(workload_source / name),
                        "command": f"/usr/bin/c++ -O3 -I{workload_source} -I{build} -c {workload_source / name}",
                    }
                    for driver in sorted(compiled_drivers)
                    for name in module.WORKLOAD_SOURCES[driver]
                    if name.endswith(".cpp")
                ],
            )
            if source.name.startswith("src-"):
                assert (source / "CMakeLists.txt").is_file()
                assert not (build / "CMakeCache.txt").exists()
                (build / "CMakeCache.txt").write_text(source.name)
                configurations.append((source, build))
        elif command[:2] == ["cmake", "--build"]:
            build = Path(command[2])
            if build.name.startswith("driver-"):
                (build / "portable_step_bench").write_text(build.name)
            else:
                targets = set(command[command.index("--target") + 1 :])
                assert targets & module.WORKLOAD_SOURCES.keys() == expected_drivers
                libraries = {"dart-utils-urdf"}
                if module.CB not in expected_drivers:
                    libraries |= {
                        "dart-collision-ode",
                        "dart-collision-bullet",
                        "dart-gui-osg",
                    }
                assert targets == libraries | expected_drivers
                (build / "bin").mkdir()
                for driver in expected_drivers:
                    (build / "bin" / driver).write_bytes(b"archived driver")
        elif command[:2] == ["cmake", "--install"]:
            prefix = Path(command[command.index("--prefix") + 1])
            (prefix / "share/dart").mkdir(parents=True)
            (prefix / "lib").mkdir()
            (prefix / "lib/libdart.so").write_bytes(b"installed DART")
        return ""

    monkeypatch.setattr(module, "execute", execute)

    def run_arm(arm):
        assert arm.commit == "commit"
        stamp = module.installed_provenance(arm)
        assert stamp["compiler"] == "GNU 13.3.0"
        assert stamp["compiler_sha"] == module.sha(b"build compiler")
        assert stamp["pixi_lock_sha"] == module.sha(
            (module.ROOT / "pixi.lock").read_bytes()
        )
        source = tmp_path / f"src-{arm.prefix.name}"
        assert arm.source_dir == source
        assert arm.base_arm == (arm.prefix.name == "a" and not smoke)
        build = tmp_path / f"build-{arm.prefix.name}"
        driver_build = tmp_path / f"driver-{arm.prefix.name}"
        assert stamp["workload_sources"] == (
            module.workload_hashes(source, expected_drivers, build, arm.prefix)
            | module.workload_hashes(module.ROOT, [module.PB], driver_build, arm.prefix)
        )
        if expected_drivers:
            assert stamp["workload_sources"] != module.workload_hashes(
                module.ROOT, expected_drivers | {module.PB}
            )
        assert stamp["binaries"].keys() == expected_drivers | {module.PB}
        portable_hashes.append(stamp["binaries"][module.PB])
        arm.output_dir.mkdir()
        record = {"run": {"commit": arm.commit}}
        module.write_json(arm.output_dir / "record.json", record)
        return record

    monkeypatch.setattr(module, "run_arm", run_arm)
    base, head = module.local_arms(args)
    labels = ("a",) if smoke else ("a", "b")
    assert resolved == [f"{revision}^{{commit}}"] * len(labels)
    assert portable_hashes == [
        module.sha(f"driver-{label}".encode()) for label in labels
    ]
    if smoke:
        assert base is head
        assert head["run"]["mode"] == "smoke"
        assert (
            json.loads((tmp_path / "a-run/record.json").read_text())["run"]["mode"]
            == "smoke"
        )
    assert args.rows == ("gzb,robot" if portable_only and not rows else rows)
    assert configurations == [
        (tmp_path / f"src-{arm}", tmp_path / f"build-{arm}") for arm in labels
    ]
    for source, build in configurations:
        assert (build / "CMakeCache.txt").read_text() == source.name


@pytest.mark.parametrize(
    "phase", ["allocshim", "heappad", "configure", "build", "install"]
)
@pytest.mark.parametrize("kind", ["build", "timeout", "os", "tool", "other"])
@pytest.mark.parametrize("smoke", [False, True])
def test_local_records_only_head_build_failures(
    monkeypatch, tmp_path, capsys, phase, kind, smoke
):
    import io
    import tarfile
    from types import SimpleNamespace

    module = _load_runner()
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / "dependencies"))
    resolved = []

    def command_output(command):
        resolved.append(command[-1])
        return (
            "base-commit" if command[-1] == "base-revision^{commit}" else "head-commit"
        )

    monkeypatch.setattr(module, "command_output", command_output)
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(returncode=0),
    )
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w"):
        pass
    monkeypatch.setattr(
        module.subprocess, "check_output", lambda *args, **kwargs: archive.getvalue()
    )
    monkeypatch.setattr(module, "workload_hashes", lambda *args: {})
    monkeypatch.setattr(module, "cmake_compiler", lambda *args: {})
    monkeypatch.setattr(module, "library_hashes", lambda *args: {})

    error = {
        "build": module.BuildFailure,
        "timeout": ValueError,
        "os": OSError,
        "tool": FileNotFoundError,
        "other": RuntimeError,
    }[kind](f"simulated {phase} failure")

    def execute(command, env, log, timeout, *, build=False):
        head = log.name.startswith("a." if smoke else "b.")
        selected = {"configure": "-G", "build": "--build", "install": "--install"}.get(
            phase
        )
        if log.name == f"{phase}.log" or head and command[:2] == ["cmake", selected]:
            assert build
            raise error
        if command[:2] == ["cmake", "--install"]:
            prefix = Path(command[command.index("--prefix") + 1])
            (prefix / "share/dart").mkdir(parents=True)
            (prefix / "lib").mkdir()
            (prefix / "lib/libdart.so").write_bytes(b"DART")
        if command[:2] == ["cmake", "--build"] and "driver-" in command[2]:
            driver = Path(command[2])
            driver.mkdir()
            (driver / "portable_step_bench").write_bytes(b"driver")
        return ""

    monkeypatch.setattr(module, "execute", execute)

    def run_arm(arm):
        assert arm.base_arm
        assert not smoke and arm.commit == "base-commit"
        record = _micro_record(module, "dyn")
        record["run"]["commit"] = arm.commit
        record["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
        arm.output_dir.mkdir()
        module.write_json(arm.output_dir / "record.json", record)
        return record

    monkeypatch.setattr(module, "run_arm", run_arm)
    shim = phase in ("allocshim", "heappad")
    assert module.main(
        [
            "local",
            *(["--smoke"] if smoke else []),
            "--base",
            "missing-base" if smoke else "base-revision",
            "--head",
            "head-revision",
            "--rows",
            "gzb",
            "--output-dir",
            str(tmp_path),
        ]
    ) == (1 if not smoke and not shim and kind == "build" else 2)
    assert resolved == (["base-revision^{commit}"] if not smoke else []) + [
        "head-revision^{commit}"
    ]
    failure = tmp_path / "build-failure.json"
    if kind == "build" and smoke:
        assert json.loads(failure.read_text()) == {
            "commit": "head-commit",
            "error": f"simulated {phase} failure",
            "error_kind": "build",
        }
    else:
        assert not failure.exists()
    report = tmp_path / "perf.json"
    if kind == "build" and not smoke and not shim:
        record = json.loads(report.read_text())
        assert record["verdict"]["status"] == "FAIL"
        assert all(row["error_kind"] == "build" for row in record["results"])
    else:
        assert f"simulated {phase} failure" in capsys.readouterr().err
        assert not report.exists()
    assert (tmp_path / "a-run/record.json").exists() == (not smoke and not shim)


@pytest.mark.parametrize("defect", [None, "measurement", "perturbation", "missing"])
@pytest.mark.parametrize("perturb", [False, True])
def test_local_smoke_reports_measurement_failures(
    monkeypatch, tmp_path, capsys, defect, perturb
):
    module = _load_runner()
    record = _micro_record(module, "dyn")
    record["run"]["mode"] = "smoke"
    record["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    row = record["results"][0]
    if defect == "measurement":
        row.update(status="broken", error="invalid measurement", head={})
    elif defect == "perturbation":
        row.update(gated=False, perturbations={"start4k": {"stable": False}})
    elif defect == "missing":
        row["gated"] = False

    def local_arms(args):
        assert args.smoke
        return record, record

    monkeypatch.setattr(module, "local_arms", local_arms)
    path = tmp_path / "perf.json"
    failed = bool(defect and (defect != "missing" or perturb))
    status = "FAIL" if failed else "PASS"
    assert (
        module.main(
            [
                "local",
                "--smoke",
                "--json",
                str(path),
                *([] if perturb else ["--no-perturb"]),
            ]
        )
        == failed
    )
    assert capsys.readouterr().out.startswith(f"Perf smoke: HEAD — {status}\n")
    assert json.loads(path.read_text())["verdict"]["status"] == status


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


def test_compare_rejects_comparison_reports_as_inputs(tmp_path):
    module = _load_runner()
    record = {"schema": "dart-perf/1", **_micro_record(module, "dyn")}
    path = tmp_path / "record.json"
    module.write_json(path, record)
    assert module.read_record(path)["results"]
    # A comparison report shares the schema but carries the base's
    # qualification, so it must not stand in for a measurement.
    report = module.compare(record, record)
    module.write_json(path, report)
    with pytest.raises(ValueError, match="comparison report"):
        module.read_record(path)


def test_workload_hashes_cover_loaded_sample_scenes(tmp_path):
    module = _load_runner()
    for paths in module.WORKLOAD_SOURCES.values():
        for name in paths:
            path = tmp_path / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"workload")
    bench = tmp_path / "tests/benchmark/integration/bm_kinematics.cpp"
    bench.write_text('scenes.push_back("dart://sample/skel/test/a.skel");\n')
    scene = tmp_path / "data/skel/test/a.skel"
    scene.parent.mkdir(parents=True)
    scene.write_bytes(b"scene")
    drivers = ["BM_INTEGRATION_kinematics", "BM_UNIT_dantzig_lcp"]
    base = module.workload_hashes(tmp_path, drivers)
    # An edited scene changes the workload of the benchmark that loads it only.
    scene.write_bytes(b"edited scene")
    head = module.workload_hashes(tmp_path, drivers)
    assert head["BM_INTEGRATION_kinematics"] != base["BM_INTEGRATION_kinematics"]
    assert head["BM_UNIT_dantzig_lcp"] == base["BM_UNIT_dantzig_lcp"]
    # A scene the revision lacks still counts, as a missing input.
    scene.unlink()
    assert module.workload_hashes(tmp_path, drivers) != head


@pytest.mark.parametrize(
    "document",
    [
        "[]",
        "null",
        '"text"',
        "1",
        '{"schema": "dart-perf/1", "results": [null]}',
        '{"schema": "dart-perf/1", "results": {"row": "dyn"}}',
        '{"schema": "dart-perf/1", "results": [{"det": "dart"}]}',
    ],
)
def test_read_record_rejects_non_object_json(tmp_path, document):
    module = _load_runner()
    path = tmp_path / "record.json"
    path.write_text(document, encoding="utf-8")
    with pytest.raises(ValueError, match="measurement record"):
        module.read_record(path)


def test_interrupt_stops_running_benchmark_groups(tmp_path):
    module = _load_runner()
    # A worker's benchmark keeps running after the main thread is interrupted;
    # kill_running() must end it so the pool can shut down.
    result = {}

    def worker():
        try:
            module.execute(
                [sys.executable, "-c", "import time; time.sleep(60)"],
                dict(os.environ),
                tmp_path / "sleep.log",
                120,
            )
        except ValueError as error:
            result["error"] = error

    thread = threading.Thread(target=worker)
    thread.start()
    for _ in range(100):
        if module.RUNNING:
            break
        time.sleep(0.05)
    assert module.RUNNING
    module.kill_running()
    thread.join(timeout=10)
    assert not thread.is_alive()
    assert not module.RUNNING


@pytest.mark.parametrize("field, value", [("head", None), ("head", [])])
def test_compare_reports_malformed_nested_values_as_infrastructure(
    tmp_path, capsys, field, value
):
    module = _load_runner()
    record = {"schema": "dart-perf/1", **_micro_record(module, "dyn")}
    record["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    bad = copy.deepcopy(record)
    bad["results"][0][field] = value
    paths = []
    for name, item in (("base", record), ("head", bad)):
        path = tmp_path / f"{name}.json"
        module.write_json(path, item)
        paths.append(path)
    argv = ["compare", "--base", str(paths[0]), "--head", str(paths[1])]
    assert module.main(argv) == 2
    assert "measurement record" in capsys.readouterr().err


def test_read_record_rejects_repeated_rows(tmp_path):
    module = _load_runner()
    record = {"schema": "dart-perf/1", **_micro_record(module, "dyn")}
    record["results"].append(copy.deepcopy(record["results"][0]))
    path = tmp_path / "record.json"
    module.write_json(path, record)
    with pytest.raises(ValueError, match="repeats a row"):
        module.read_record(path)


def _perf_workflow():
    path = Path(__file__).resolve().parents[1] / ".github/workflows/perf.yml"
    # BaseLoader keeps GitHub's `on` key from becoming a YAML 1.1 boolean.
    return yaml.load(path.read_text(), Loader=yaml.BaseLoader)


def _perf_step(job, name):
    return next(
        step for step in _perf_workflow()["jobs"][job]["steps"] if step["name"] == name
    )


@pytest.mark.parametrize(
    "paths, mode, smoke",
    [
        (["dart/dynamics/World.cpp", "docs/README.md"], "ab", "false"),
        (["scripts/perf_regression.py"], "smoke", "true"),
        (["tools/perf/driver.cpp"], "smoke", "true"),
        (["tests/benchmark/worlds/test.world"], "smoke", "true"),
        ([".github/workflows/perf.yml"], "smoke", "true"),
        (["dart/dynamics/World.cpp", "scripts/perf_regression.py"], "ab", "true"),
        (["tools/perf/driver.cpp", "dart/dynamics/World.cpp"], "ab", "true"),
        (
            ["dart/dynamics/World.cpp", "tests/benchmark/worlds/test.world"],
            "ab",
            "true",
        ),
        ([".github/workflows/perf.yml", "dart/dynamics/World.cpp"], "ab", "true"),
    ],
)
def test_workflow_selects_smoke_and_ab_for_mixed_changes(tmp_path, paths, mode, smoke):
    env_path = tmp_path / "env"
    # Stub git's revision/diff output while executing the actual shell selector.
    stub = """
    changed_paths=("$@")
    git() {
      case "$1" in
        rev-parse) printf '%s\\n' revision ;;
        diff) printf '%s\\0' "${changed_paths[@]}" ;;
        worktree) return 0 ;;
      esac
    }
    """
    result = subprocess.run(
        [
            "bash",
            "-e",
            "-c",
            stub + _perf_step("measure", "Select smoke or A/B")["run"],
            "selector",
            *paths,
        ],
        env={
            **os.environ,
            "GITHUB_ENV": str(env_path),
            "GITHUB_OUTPUT": str(tmp_path / "output"),
            "RUNNER_TEMP": str(tmp_path),
        },
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr
    selected = dict(line.split("=", 1) for line in env_path.read_text().splitlines())
    assert selected["PERF_MODE"] == mode
    assert selected["PERF_SMOKE"] == smoke


def test_workflow_covers_every_workload_source_and_data_path():
    module = _load_runner()
    patterns = _perf_workflow()["on"]["pull_request"]["paths"]
    selector = _perf_step("measure", "Select smoke or A/B")["run"]
    cases = re.search(r"^\s+(.+?)\) mode=ab", selector, re.M)[1].split("|")
    # run_arm() reads the pinned 3k world from the harness checkout.
    assert "tests/benchmark/worlds/*|.github/workflows/perf.yml) smoke=true" in selector
    sources = {name for names in module.WORKLOAD_SOURCES.values() for name in names}
    sources |= {name for names in module.WORKLOAD_DATA.values() for name in names}
    # Atlas hashes only referenced assets; micro scenes use dart://sample.
    sources |= {
        path.relative_to(module.ROOT).as_posix()
        for path in module.robot_data_paths(module.ROOT)
    }
    assert "data/sdf/atlas/pelvis.stl" in sources
    assert "data/sdf/atlas/head.stl" not in sources
    sources |= {
        "data/" + uri
        for name in sources.copy()
        if (module.ROOT / name).is_file() and not name.startswith("data/")
        for uri in re.findall(
            r'"dart://sample/([^"]+)"', (module.ROOT / name).read_text()
        )
    }
    # The portable driver is built once from the harness checkout and run against
    # both arms, so a driver-only change gets the head harness's smoke run.
    driver = set(module.WORKLOAD_SOURCES[module.PB])
    for source in sorted(sources):
        assert any(
            re.fullmatch(
                re.escape(pattern).replace(r"\*\*", ".*").replace(r"\*", "[^/]*"),
                source,
            )
            for pattern in patterns
        ), source
        assert (source in driver) != any(
            fnmatch.fnmatchcase(source, pattern) for pattern in cases
        ), source


@pytest.mark.parametrize(
    "name, changed",
    [
        ("pend", "data/sdf/benchmark.world"),
        ("robot", "data/sdf/atlas/meshes/shape.stl"),
        ("robot", "data/sdf/atlas/meshes/ground.stl"),
    ],
)
def test_each_arm_reads_and_hashes_its_own_data(monkeypatch, tmp_path, name, changed):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--prefix",
            str(tmp_path),
            "--commit",
            "HEAD",
            "--output-dir",
            str(tmp_path),
            "--native-only",
            "--no-perturb",
        ]
    )
    args.bin_dir = tmp_path / "bin"
    row = module.select_rows(name)[0]
    paths = {
        module.WORKLOAD_DATA["pend"][0]: "original",
        module.WORKLOAD_DATA["robot"][0]: (
            '<robot><link><visual><geometry><mesh filename="file://meshes/ground.stl"/>'
            "</geometry></visual></link></robot>"
        ),
        module.WORKLOAD_DATA["robot"][1]: (
            "<sdf><model><link><collision><geometry><mesh>"
            "<uri>meshes/shape.stl</uri></mesh></geometry></collision></link></model></sdf>"
        ),
        "data/sdf/atlas/meshes/shape.stl": "original",
        "data/sdf/atlas/meshes/ground.stl": "original",
        "data/sdf/atlas/head.stl": "unrelated",
    }
    for arm in ("a", "b"):
        for path, content in paths.items():
            file = tmp_path / arm / path
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_text(content)
    seen = []

    def native(row, args, world):
        command = module.row_command(row, args, world, 0, 1)
        data = args.source_dir / "data"
        if name == "pend":
            assert command[1] == str(data / "sdf/benchmark.world")
            seen.append(Path(command[1]).read_text())
        else:
            assert command[command.index("--data-dir") + 1] == str(data)
            seen.append((args.source_dir / changed).read_text())
        return {"guards": {"finite": True}, "allocs": 0}

    monkeypatch.setattr(module, "native", native)
    args.source_dir = tmp_path / "a"
    base = module.measure(row, args, tmp_path)
    args.source_dir = tmp_path / "b"
    assert module.measure(row, args, tmp_path)["input_sha"] == base["input_sha"]
    (args.source_dir / "data/sdf/atlas/head.stl").write_text("unrelated edit")
    assert module.measure(row, args, tmp_path)["input_sha"] == base["input_sha"]
    (args.source_dir / changed).write_text("edited head")
    head = module.measure(row, args, tmp_path)
    assert head["input_sha"] != base["input_sha"]
    assert seen == ["original", "original", "original", "edited head"]
    args.source_dir = tmp_path / "a"
    assert module.measure(row, args, tmp_path)["input_sha"] == base["input_sha"]


@pytest.mark.parametrize(
    "name, path",
    [
        ("pend", "data/sdf/benchmark.world"),
        ("robot", "data/sdf/atlas/ground.urdf"),
        ("robot", "data/sdf/atlas/atlas_v3_no_head.sdf"),
        ("robot", "data/sdf/atlas/meshes/shape.stl"),
        ("robot", "data/sdf/atlas/meshes/ground.stl"),
    ],
)
@pytest.mark.parametrize("defect", ["missing", "unreadable"])
@pytest.mark.parametrize("arm", ["base", "head", "smoke"])
def test_revision_input_errors_are_recorded_per_row(
    monkeypatch, tmp_path, name, path, defect, arm
):
    module = _load_runner()
    source = tmp_path / "source"
    inputs = {
        "data/sdf/benchmark.world": "world",
        "data/sdf/atlas/ground.urdf": (
            '<robot><mesh filename="file://meshes/ground.stl"/></robot>'
        ),
        "data/sdf/atlas/atlas_v3_no_head.sdf": (
            "<sdf><mesh><uri>meshes/shape.stl</uri></mesh></sdf>"
        ),
        "data/sdf/atlas/meshes/shape.stl": "shape",
        "data/sdf/atlas/meshes/ground.stl": "ground",
    }
    for relative, content in inputs.items():
        file = source / relative
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(content)
    shim = tmp_path / "shim.so"
    shim.write_bytes(b"shim")
    args = module.parser().parse_args(
        [
            "run",
            "--prefix",
            str(tmp_path),
            "--commit",
            "HEAD",
            "--source-dir",
            str(source),
            "--output-dir",
            str(tmp_path / "valid-run"),
            "--shim",
            str(shim),
            "--rows",
            f"{name},gzb",
            "--native-only",
            "--no-perturb",
        ]
    )
    monkeypatch.setattr(module, "command_output", lambda command: "HEAD")
    env = _micro_record(module, "dyn")["run"]["env"]
    env.update(valgrind="test", glibc="test", preset="perf-1")
    monkeypatch.setattr(module, "fingerprint", lambda *args: env)
    monkeypatch.setattr(
        module,
        "installed_provenance",
        lambda args: {
            "workload_sources": {module.CB: "contact", module.PB: "portable"}
        },
    )
    measured = []

    def native(row, *args):
        measured.append(row.row)
        return {
            "allocs_per_step": 0,
            "bytes_per_step": 0,
            "guards": {
                "hash": "0x1",
                "finite": True,
                "contacts": 1,
                "cap_hit": True,
                "resting": "0/1",
            },
        }

    monkeypatch.setattr(module, "native", native)
    valid = module.run_arm(args)
    measured.clear()
    damaged = source / path
    if defect == "missing":
        damaged.unlink()
    else:
        # Simulate permissions independently of the test runner's user.
        original = Path.open

        def open_file(self, *args, **kwargs):
            if self == damaged:
                raise PermissionError(f"unreadable input: {self}")
            return original(self, *args, **kwargs)

        monkeypatch.setattr(Path, "open", open_file)
    args.base_arm = arm == "base"
    args.output_dir = tmp_path / "broken-run"
    broken = module.run_arm(args)
    row, healthy = broken["results"]
    assert row["status"] == "broken" and not row["gated"]
    assert row["input_sha"] is None
    assert "failed to load revision inputs" in row["error"]
    assert str(damaged) in row["error"]
    assert (row.get("error_kind") == "infrastructure") == (arm == "base")
    assert healthy["status"] == "ok" and measured == ["gzb"]
    assert json.loads((args.output_dir / "record.json").read_text()) == broken
    if arm == "smoke":
        broken["run"]["mode"] = "smoke"
        records = broken, broken
    else:
        records = (broken, valid) if arm == "base" else (valid, broken)
    monkeypatch.setattr(module, "local_arms", lambda args: records)
    report = tmp_path / "perf.json"
    assert module.main(
        [
            "local",
            "--json",
            str(report),
            "--no-perturb",
            *(["--smoke"] if arm == "smoke" else []),
        ]
    ) == (2 if arm == "base" else 1)
    verdict = json.loads(report.read_text())["verdict"]
    assert verdict["status"] == ("ERROR" if arm == "base" else "FAIL")
    assert any(row["error"] in failure for failure in verdict["failures"])


@pytest.mark.parametrize("defect", ["nonzero", "timeout", "missing", "signal"])
def test_only_completed_nonzero_builds_are_build_failures(tmp_path, defect):
    module = _load_runner()
    command = [
        sys.executable,
        "-c",
        "print('source.cpp:1: error: missing symbol'); raise SystemExit(1)",
    ]
    timeout = 5
    expected = module.BuildFailure
    if defect == "timeout":
        command[-1] = "import time; time.sleep(60)"
        timeout = 0.05
        expected = ValueError
    elif defect == "missing":
        command = [str(tmp_path / "missing-tool")]
        expected = FileNotFoundError
    elif defect == "signal":
        command[-1] = "import os, signal; os.kill(os.getpid(), signal.SIGTERM)"
        expected = ValueError
    with pytest.raises(expected) as raised:
        module.execute(
            command, dict(os.environ), tmp_path / "build.log", timeout, build=True
        )
    assert isinstance(raised.value, module.BuildFailure) == (defect == "nonzero")
    assert not module.RUNNING


@pytest.mark.parametrize(
    "text, source_error",
    [
        ("source.cpp:1: error: missing symbol", True),
        ("source.cpp:1: fatal error: missing header", True),
        ("ld: undefined reference to missing_symbol", True),
        ("CMake Error at CMakeLists.txt:1 (bad_command):", True),
        ("ninja: build stopped: subcommand failed.", False),
        ("", False),
        ("No space left on device", False),
        ("c++: fatal error: Killed signal terminated program cc1plus", False),
        ("source.cpp:1: error: No space left on device", False),
        ("CMake Error: Cannot allocate memory", False),
        ("source.cpp:1: error: virtual memory exhausted", False),
        ("ld: undefined reference to symbol\nsubprocess killed by signal 9", False),
    ],
)
def test_build_failure_requires_source_diagnostic_without_runner_failure(
    tmp_path, text, source_error
):
    module = _load_runner()
    command = [sys.executable, "-c", f"print({text!r}); raise SystemExit(1)"]
    with pytest.raises(ValueError) as raised:
        module.execute(command, dict(os.environ), tmp_path / "build.log", 5, build=True)
    assert isinstance(raised.value, module.BuildFailure) == source_error
    assert not module.RUNNING


def _run_perf_snippet(job, step, tmp_path, env):
    script = re.search(
        r"python3 - <<'PY'\n(.*?)\nPY", _perf_step(job, step)["run"], re.S
    )[1]
    return subprocess.run(
        [sys.executable, "-c", script],
        cwd=tmp_path,
        env={
            **os.environ,
            "PYTHONPATH": str(Path(__file__).resolve().parents[1]),
            **env,
        },
        text=True,
        capture_output=True,
    )


@pytest.mark.parametrize(
    "defect",
    [
        None,
        "missing",
        "extra",
        "renamed",
        "duplicate",
        "unqualified",
        "harness",
        "broken",
        "unperturbed",
    ],
)
@pytest.mark.parametrize("mixed", [False, True])
def test_verdict_snippet_requires_complete_qualified_smoke_rows(
    tmp_path, defect, mixed
):
    module = _load_runner()
    rows = [
        {
            "row": row.row,
            "det": row.det,
            "status": "ok",
            "gated": True,
            "perturbations": {"test": {"stable": True}},
        }
        for row in module.select_rows("")
    ]
    if defect == "missing":
        rows.pop()
    elif defect == "extra":
        rows.append({**rows[0], "row": "unexpected"})
    elif defect == "renamed":
        rows[0]["row"] = "renamed"
    elif defect == "duplicate":
        rows.append(copy.deepcopy(rows[0]))
    elif defect == "unqualified":
        rows[0]["gated"] = False
    elif defect == "broken":
        rows[0]["status"] = "broken"
    elif defect == "unperturbed":
        rows[0]["perturbations"] = {}
    arm = {"schema": "dart-perf/1", "run": {"commit": "head"}, "results": rows}
    record = {
        "mode": "smoke",
        "measurement_exit": 1 if defect == "harness" else 0,
        "wall_seconds": 1,
        "base": arm,
        "head": arm,
    }
    ab_head = {"schema": "dart-perf/1", "run": {"commit": "head"}, "results": []}
    if mixed:
        record = {
            "mode": "ab",
            "measurement_exit": 0,
            "wall_seconds": 1,
            "base": {**ab_head, "run": {"commit": "base"}},
            "head": ab_head,
            "smoke": record,
        }
    module.write_json(tmp_path / "perf.json", record)
    (tmp_path / "verdict-exit").write_text("2\n")
    result = _run_perf_snippet(
        "verdict",
        "Apply base rules to the live PR body",
        tmp_path,
        {"PERF_ARTIFACT": str(tmp_path), "PERF_HEAD": "head", "PERF_BASE": "base"},
    )
    assert result.returncode == 0, result.stderr
    # The comparison still runs for row diagnostics; the step's shell turns any
    # listed smoke failure into FAIL.
    assert json.loads((tmp_path / "head.json").read_text()) == (
        ab_head if mixed else arm
    )
    assert bool((tmp_path / "smoke-failures.txt").read_text()) == bool(defect)


@pytest.mark.parametrize("status", [0, 1, 2])
def test_measure_snippet_preserves_mixed_smoke_exit(tmp_path, status):
    module = _load_runner()
    output, artifact = tmp_path / "output", tmp_path / "artifact"
    artifact.mkdir()
    for name, commit in (("a-run", "base"), ("b-run", "head"), ("smoke/a-run", "head")):
        path = output / name / "record.json"
        path.parent.mkdir(parents=True)
        module.write_json(
            path, {"schema": "dart-perf/1", "run": {"commit": commit}, "results": []}
        )
    env = {
        "PERF_MODE": "ab",
        "PERF_SMOKE": "true",
        "PERF_SMOKE_STATUS": str(status),
        "PERF_OUTPUT": str(output),
        "PERF_ARTIFACT": str(artifact),
        "PERF_STATUS": "0",
        "PERF_SECONDS": "1",
        "PERF_HEAD": "head",
        "PERF_BASE": "base",
    }
    measured = _run_perf_snippet("measure", "Build and measure", tmp_path, env)
    assert measured.returncode == (2 if status == 2 else 0), measured.stderr
    record = json.loads((artifact / "perf.json").read_text())
    assert record["mode"] == "ab"
    assert record["measurement_exit"] == 0
    assert record["smoke"]["measurement_exit"] == status
    assert record["smoke"]["head"]["run"]["commit"] == "head"
    judged = _run_perf_snippet(
        "verdict", "Apply base rules to the live PR body", tmp_path, env
    )
    assert (judged.returncode != 0) == (status == 2), judged.stderr


@pytest.mark.parametrize("kind", ["build", "infrastructure", None])
@pytest.mark.parametrize("mode", ["smoke", "ab", "mixed"])
@pytest.mark.parametrize("status", [1, 2])
def test_measure_and_verdict_snippets_require_broken_build_kind(
    tmp_path, kind, mode, status
):
    module = _load_runner()
    output, artifact = tmp_path / "output", tmp_path / "artifact"
    output.mkdir()
    artifact.mkdir()
    if mode == "mixed":
        for name, commit in (("a-run", "base"), ("b-run", "head")):
            (output / name).mkdir()
            module.write_json(
                output / name / "record.json",
                {"schema": "dart-perf/1", "run": {"commit": commit}, "results": []},
            )
        (output / "smoke").mkdir()
    failure = {"commit": "head", "error": "head build failed: simulated"}
    if kind is not None:
        failure["error_kind"] = kind
    if mode in ("smoke", "mixed"):
        module.write_json(
            output / ("smoke" if mode == "mixed" else "") / "build-failure.json",
            failure,
        )
    else:
        (output / "a-run").mkdir()
        module.write_json(
            output / "a-run/record.json",
            {"schema": "dart-perf/1", "run": {"commit": "base"}, "results": []},
        )
        module.write_json(
            output / "perf.json", {"results": [{**failure, "row": "dyn", "det": ""}]}
        )
    env = {
        "PERF_MODE": "ab" if mode == "mixed" else mode,
        "PERF_SMOKE": "true" if mode == "mixed" else "false",
        "PERF_SMOKE_STATUS": str(status),
        "PERF_OUTPUT": str(output),
        "PERF_ARTIFACT": str(artifact),
        "PERF_STATUS": "0" if mode == "mixed" else str(status),
        "PERF_SECONDS": "1",
        "PERF_HEAD": "head",
        "PERF_BASE": "base",
    }
    measured = _run_perf_snippet("measure", "Build and measure", tmp_path, env)
    if kind != "build" or mode == "ab" and status == 2:
        assert measured.returncode != 0
        assert not (artifact / "perf.json").exists()
    else:
        assert measured.returncode == 0, measured.stderr
        record = json.loads((artifact / "perf.json").read_text())
        assert record["measurement_exit"] == 0
        if mode == "ab":
            assert record["head"]["results"][0]["error_kind"] == "build"
        elif mode == "mixed":
            assert record["smoke"]["measurement_exit"] == 0
            assert record["smoke"]["broken"] == failure
    if mode in ("smoke", "mixed"):
        # Exercise the base verdict independently, including a forged broken record.
        record = {
            "mode": "smoke",
            "measurement_exit": 0,
            "wall_seconds": 1,
            "broken": failure,
        }
        if mode == "mixed":
            record = {
                "mode": "ab",
                "measurement_exit": 0,
                "wall_seconds": 1,
                "base": {"run": {"commit": "base"}, "results": []},
                "head": {"run": {"commit": "head"}, "results": []},
                "smoke": record,
            }
        module.write_json(artifact / "perf.json", record)
        (tmp_path / "verdict-exit").write_text("2\n")
        judged = _run_perf_snippet(
            "verdict", "Apply base rules to the live PR body", tmp_path, env
        )
        assert (judged.returncode == 0) == (kind == "build"), judged.stderr
        if mode == "mixed":
            # A/B diagnostics must still run; the shell combines these failures.
            assert (tmp_path / "verdict-exit").read_text() == "2\n"
            if kind == "build":
                assert (
                    "head build failed" in (tmp_path / "smoke-failures.txt").read_text()
                )
                assert (tmp_path / "head.json").exists()
        else:
            assert (tmp_path / "verdict-exit").read_text() == (
                "1\n" if kind == "build" else "2\n"
            )


@pytest.mark.parametrize(
    "status,defect",
    [
        (2, None),
        (2, "missing"),
        (2, "infrastructure"),
        (2, "wrong-head"),
        (2, "invalid-json"),
        (2, "missing-error"),
        (0, "missing"),
        (1, "missing"),
    ],
)
def test_merge_smoke_build_failure_decision_distinguishes_infrastructure(
    tmp_path, status, defect
):
    module = _load_runner()
    failure = {"commit": "head", "error_kind": "build", "error": "compiler diagnostic"}
    if defect == "infrastructure":
        failure["error_kind"] = "infrastructure"
    elif defect == "wrong-head":
        failure["commit"] = "another-head"
    elif defect == "missing-error":
        failure.pop("error")
    if defect == "invalid-json":
        (tmp_path / "build-failure.json").write_text("{")
    elif defect != "missing":
        module.write_json(tmp_path / "build-failure.json", failure)
    if defect is None:
        assert module.smoke_build_failure(tmp_path, "head", status) == failure
    elif status != 2:
        assert module.smoke_build_failure(tmp_path, "head", status) is None
    else:
        with pytest.raises(ValueError):
            module.smoke_build_failure(tmp_path, "head", status)


@pytest.mark.parametrize("defect", [None, "missing", "infrastructure", "wrong-head"])
def test_merge_smoke_exit_two_saves_build_failure_verdict(tmp_path, defect):
    module = _load_runner()
    output = tmp_path / "perf"
    smoke = output / "smoke"
    smoke.mkdir(parents=True)
    harness = tmp_path / "perf-harness"
    harness.mkdir()
    record = _publication_fixture()
    record["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    record["results"][0]["gate_reason"] = "qualified"
    record["results"][0]["failures"] = []
    record["results"][0]["delta"]["bytes"] = 0
    module.write_json(output / "perf.json", record)
    failure = {
        "commit": record["run"]["commit"],
        "error_kind": "build",
        "error": "compiler diagnostic",
    }
    if defect == "infrastructure":
        failure["error_kind"] = "infrastructure"
    elif defect == "wrong-head":
        failure["commit"] = "another-head"
    if defect != "missing":
        module.write_json(smoke / "build-failure.json", failure)
    # Execute the shell guard and saved-verdict code with only measurement stubbed.
    binary = tmp_path / "bin"
    binary.mkdir()
    pixi = binary / "pixi"
    pixi.write_text(
        '#!/bin/bash\nfor arg in "$@"; do if [[ "$arg" == --smoke ]]; then exit 2; fi; done\nexit 0\n'
    )
    pixi.chmod(0o755)
    result = subprocess.run(
        [
            "bash",
            "-e",
            "-c",
            _perf_step("record-measure", "Measure and apply the parent rules")["run"],
        ],
        cwd=Path(__file__).resolve().parents[1],
        env={
            **os.environ,
            "PATH": f"{binary}:{os.environ['PATH']}",
            "PYTHONPATH": str(Path(__file__).resolve().parents[1]),
            "RUNNER_TEMP": str(tmp_path),
            "PERF_OUTPUT": str(output),
            "PERF_MODE": "ab",
            "PERF_SMOKE": "true",
            "PERF_HEAD": record["run"]["commit"],
            "PERF_BASE": record["run"]["parent"],
        },
        text=True,
        capture_output=True,
    )
    saved = json.loads((output / "perf.json").read_text())
    if defect is None:
        assert result.returncode == 0, result.stderr
        assert saved["verdict"]["status"] == "FAIL"
        assert saved["verdict"]["failures"] == [
            "head smoke build failed: compiler diagnostic"
        ]
        assert "head smoke build failed" in (output / "perf.md").read_text()
        module.publication_record(output / "perf.json", "merge")
    else:
        assert result.returncode != 0
        assert saved == record
        assert not (output / "perf.md").exists()


@pytest.mark.parametrize("identity_change", ["none", "input", "window"])
@pytest.mark.parametrize("verdict", ["PASS", "FAIL"])
@pytest.mark.parametrize(
    "row_change",
    [None, "added", "removed", "bad-status", "no-perturbations", "not-gated"],
)
def test_merge_record_publishes_changed_smoke_inputs_keeps_parent_verdict(
    tmp_path, identity_change, verdict, row_change
):
    module = _load_runner()
    output = tmp_path / "perf"
    smoke_dir = output / "smoke/a-run"
    smoke_dir.mkdir(parents=True)
    record = _publication_fixture()
    record["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    record["run"]["env"]["harness_sha"] = "1" * 64
    template = record["results"][0]
    record["results"] = []
    for spec in module.select_rows(""):
        row = copy.deepcopy(template)
        row.update(
            row=spec.row,
            det=spec.det,
            threads=spec.threads,
            workload_sha="old-workload",
            perturbations={"test": {"stable": True}},
            gate_reason="base perturbation check passed; base gate remains active",
            failures=[],
        )
        row["delta"]["bytes"] = 0
        record["results"].append(row)
    if verdict == "FAIL":
        record["results"][0]["failures"] = ["Ir +2.00% (limit +1.00%)"]
        record["verdict"].update(
            status="FAIL", failures=["s3w/dart: Ir +2.00% (limit +1.00%)"]
        )
    smoke = copy.deepcopy(record)
    smoke.pop("verdict")
    smoke["run"]["env"].update(fingerprint="2" * 64, harness_sha="2" * 64)
    smoke["results"].reverse()
    for row in smoke["results"]:
        for key in ("parent", "delta", "gate_reason", "failures"):
            row.pop(key)
        row["head"].update(
            ir_per_step=200_000,
            allocs_per_step=5,
            bytes_per_step=40,
            wall_ms_per_step=24.6,
        )
        if identity_change == "input" and row["row"] in ("dyn", "lcp"):
            row.update(input_sha="f" * 64, workload_sha="new-workload")
        elif identity_change == "window" and row["row"] in ("dyn", "lcp"):
            # Settings change without a workload change keeps input_sha.
            row["window"] = {"warmup": 7, "steps": 9}
    if row_change == "removed":
        smoke["results"].pop()
    elif row_change is not None:
        added = copy.deepcopy(smoke["results"][0])
        added.update(row="new-benchmark", det="")
        if row_change == "bad-status":
            added["status"] = "failed"
        elif row_change == "no-perturbations":
            added.pop("perturbations")
        elif row_change == "not-gated":
            added["gated"] = False
        smoke["results"].append(added)
    module.write_json(output / "perf.json", record)
    module.write_json(smoke_dir / "record.json", smoke)
    # Run the record-construction snippet, which imports the parent's rules in CI.
    script = re.findall(
        r"python3 - <<'PY'\n(.*?)\nPY",
        _perf_step("record-measure", "Measure and apply the parent rules")["run"],
        re.S,
    )[-1]
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=tmp_path,
        env={
            **os.environ,
            "PYTHONPATH": str(Path(__file__).resolve().parents[1]),
            "PERF_OUTPUT": str(output),
            "PERF_SMOKE": "true",
            "PERF_SMOKE_STATUS": "0",
            "PERF_HEAD": record["run"]["commit"],
            "PERF_BASE": record["run"]["parent"],
        },
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr
    saved = json.loads((output / "perf.json").read_text())
    assert saved["run"] == record["run"]
    expected_verdict = copy.deepcopy(record["verdict"])
    if row_change == "removed":
        expected_verdict["status"] = "FAIL"
        expected_verdict["failures"].append(
            "head smoke benchmark rows omit parent defaults or repeat rows"
        )
    elif row_change not in (None, "added"):
        expected_verdict["status"] = "FAIL"
        expected_verdict["failures"].append(
            "head smoke rows did not pass their perturbation checks"
        )
    assert saved["verdict"] == expected_verdict
    smoke_rows = {module.row_key(row): row for row in smoke["results"]}
    for original, row in zip(record["results"], saved["results"]):
        head = smoke_rows.get(module.row_key(original))
        if head is None or (head["input_sha"], head.get("window")) == (
            original["input_sha"],
            original.get("window"),
        ):
            assert row == original
        else:
            assert row["input_sha"] == head["input_sha"]
            assert row.get("window") == head.get("window")
            assert row["workload_sha"] == head["workload_sha"]
            assert row["head"] == head["head"]
            assert row["head_env"] == smoke["run"]["env"]
            assert row["perturbations"] == head["perturbations"]
            assert row["parent"] == original["parent"]
            assert row["failures"] == original["failures"]
            assert row["gated"] is False
            assert row["delta"] == {
                "ir": None,
                "allocs": None,
                "bytes": None,
                "guards_equal": None,
                "class": "behaviour-change",
            }
            assert "measurement identity changed" in row["gate_reason"]
            assert row["wall_ms_per_step"] == {
                "parent": 12.3,
                "head": 24.6,
                "advisory": True,
            }
    assert len(saved["results"]) == len(record["results"]) + (row_change == "added")
    if row_change == "added":
        added = saved["results"][-1]
        assert added["row"] == "new-benchmark"
        assert added["head"] == smoke_rows["new-benchmark"]["head"]
        assert added["head_env"] == smoke["run"]["env"]
        assert added["parent"] == {}
        assert added["gated"] is False
        assert added["perturbations"] == smoke_rows["new-benchmark"]["perturbations"]
        assert added["delta"] == {
            "ir": None,
            "allocs": None,
            "bytes": None,
            "guards_equal": None,
            "class": "new",
        }
        assert "new row" in (output / "perf.md").read_text()
    published = module.publication_record(output / "perf.json", "merge")
    pages = tmp_path / "pages"
    (pages / "performance/dart6").mkdir(parents=True)
    (pages / "performance/dart6/index.html").write_text("chart")
    module.chart_data(pages, published)
    chart = (pages / "performance/dart6-ir/data.js").read_text()
    point = json.loads(chart.removeprefix("window.BENCHMARK_DATA = "))["entries"][
        "DART 6 deterministic counts"
    ][0]
    assert point["commit"]["id"] == record["run"]["commit"]
    benches = {bench["name"]: bench["value"] for bench in point["benches"]}
    replaced = identity_change != "none"
    suffix = "ffffffff" if identity_change == "input" else "01234567"
    assert benches[f"dyn@1:{suffix} Ir"] == (200_000 if replaced else 100_000)
    assert benches[f"dyn@1:{suffix} allocations"] == (5 if replaced else 0)
    assert benches["s3w/dart@1:01234567 Ir"] == 100_000
    for bench in point["benches"]:
        smoke_derived = (
            replaced and bench["name"].startswith(("dyn@", "lcp@"))
        ) or bench["name"].startswith("new-benchmark@")
        expected_fingerprint = ("2" if smoke_derived else "1") * 64
        assert bench["fingerprint"] == expected_fingerprint
        assert f"fingerprint: {expected_fingerprint}" in bench["extra"]
    if row_change == "added":
        added_suffix = smoke_rows["new-benchmark"]["input_sha"][:8]
        assert benches[f"new-benchmark@1:{added_suffix} Ir"] == 200_000
        assert benches[f"new-benchmark@1:{added_suffix} allocations"] == 5
    # The next ordinary merge uses the head harness for all rows.
    following = copy.deepcopy(published)
    following["run"].update(commit="c" * 40, time="2026-10-08T09:00:00+00:00")
    following["run"]["env"] = smoke["run"]["env"]
    for row in following["results"]:
        row.pop("head_env", None)
    module.chart_data(pages, following)
    for bench in _chart_points(pages)[-1]["benches"]:
        smoke_derived = (
            replaced and bench["name"].startswith(("dyn@", "lcp@"))
        ) or bench["name"].startswith("new-benchmark@")
        assert ("fingerprint changed" in bench["extra"]) is not smoke_derived


def _publication_fixture():
    metrics = {
        "ir_per_step": 100_000,
        "allocs_per_step": 0,
        "wall_ms_per_step": 12.3,
        "guards": {
            "hash": "0x123456789abcdef0",
            "contacts": 3,
            "resting": "0/3",
            "finite": True,
            "cap_hit": False,
        },
    }
    return {
        "schema": "dart-perf/1",
        "run": {
            "tier": "local",
            "commit": "a" * 40,
            "parent": "b" * 40,
            "branch": "topic",
            "describe": "v6.19.4-270-gaaaaaaaaaaaa",
            "time": "2026-10-08T08:00:00Z",
            "env": {
                "fingerprint": "1" * 64,
                "runner": {"environment": "github-hosted", "name": "hosted"},
            },
            "accepted": [],
        },
        "results": [
            {
                "row": "s3w",
                "det": "dart",
                "version": 1,
                "input_sha": "01234567" + "a" * 56,
                "gated": True,
                "status": "ok",
                "method": "slope",
                "parent": copy.deepcopy(metrics),
                "head": metrics,
                "delta": {"ir": 0, "allocs": 0, "guards_equal": True, "class": "gated"},
            }
        ],
        "verdict": {"status": "PASS", "failures": [], "warnings": [], "ir_geomean": 0},
    }


def _trusted_publication(monkeypatch, event="push"):
    monkeypatch.setenv("RUNNER_ENVIRONMENT", "github-hosted")
    monkeypatch.setenv("GITHUB_EVENT_NAME", event)
    monkeypatch.setenv("GITHUB_REF", "refs/heads/main")


@pytest.mark.parametrize("tier", ["merge", "nightly"])
def test_publication_schema_normalizes_tier_and_advisory_wall(tmp_path, tier):
    module = _load_runner()
    path = tmp_path / "record.json"
    module.write_json(path, _publication_fixture())
    record = module.publication_record(path, tier, 3570)
    assert record["run"]["tier"] == tier
    assert record["run"]["branch"] == "main"
    assert record["run"]["pr"] == 3570
    assert record["run"]["time"] == "2026-10-08T08:00:00+00:00"
    row = record["results"][0]
    assert row["wall_ms_per_step"] == {
        "parent": 12.3 if tier == "merge" else None,
        "head": 12.3,
        "advisory": True,
    }
    if tier == "nightly":
        assert record["run"]["parent"] is None
        assert "parent" not in row and "delta" not in row
        assert "verdict" not in record


@pytest.mark.parametrize(
    "event,ref,runner,tier,allowed",
    [
        ("push", "main", "github-hosted", "merge", True),
        ("workflow_dispatch", "main", "github-hosted", "merge", True),
        ("schedule", "main", "github-hosted", "nightly", True),
        ("workflow_dispatch", "main", "github-hosted", "nightly", True),
        ("pull_request", "main", "github-hosted", "merge", False),
        ("pull_request", "main", "github-hosted", "nightly", False),
        ("workflow_call", "main", "github-hosted", "nightly", False),
        ("schedule", "topic", "github-hosted", "nightly", False),
        ("push", "main", "self-hosted", "merge", False),
        ("schedule", "main", "local", "nightly", False),
        ("schedule", "main", "github-hosted", "merge", False),
        ("push", "main", "github-hosted", "nightly", False),
    ],
)
def test_publication_context_refuses_untrusted_writes(
    monkeypatch, event, ref, runner, tier, allowed
):
    module = _load_runner()
    _trusted_publication(monkeypatch, event)
    monkeypatch.setenv("GITHUB_REF", f"refs/heads/{ref}")
    monkeypatch.setenv("RUNNER_ENVIRONMENT", runner)
    if allowed:
        module.publication_guard(tier)
    else:
        with pytest.raises(ValueError):
            module.publication_guard(tier)


@pytest.mark.parametrize(
    "defect",
    [
        "schema",
        "runner",
        "commit",
        "parent",
        "fingerprint",
        "head-fingerprint",
        "head-runner",
        "time",
        "verdict",
        "nan",
        "duplicate",
        "infrastructure",
    ],
)
def test_publication_rejects_invalid_measurement_before_git(tmp_path, defect):
    module = _load_runner()
    record = _publication_fixture()
    if defect == "schema":
        record["schema"] = "unexpected"
    elif defect == "runner":
        record["run"]["env"]["runner"]["environment"] = "self-hosted"
    elif defect in ("commit", "parent", "time"):
        record["run"][defect] = "invalid"
    elif defect == "fingerprint":
        record["run"]["env"]["fingerprint"] = "invalid"
    elif defect in ("head-fingerprint", "head-runner"):
        env = copy.deepcopy(record["run"]["env"])
        if defect == "head-fingerprint":
            env["fingerprint"] = "invalid"
        else:
            env["runner"]["environment"] = "self-hosted"
        record["results"][0]["head_env"] = env
    elif defect == "verdict":
        record["verdict"]["status"] = "ERROR"
    elif defect == "nan":
        record["results"][0]["head"]["ir_per_step"] = float("nan")
    elif defect == "duplicate":
        record["results"] *= 2
    elif defect == "infrastructure":
        record["results"][0]["error_kind"] = "infrastructure"
    path = tmp_path / "record.json"
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError):
        module.publication_record(path, "merge")


def _stock_chart_template(pages):
    template = pages / "performance/dart6/index.html"
    template.parent.mkdir(parents=True, exist_ok=True)
    template.write_text('<script src="data.js"></script>stock page\n')


def _chart_points(pages):
    data = (pages / "performance/dart6-ir/data.js").read_text()
    return json.loads(data.removeprefix("window.BENCHMARK_DATA = "))["entries"][
        "DART 6 deterministic counts"
    ]


@pytest.mark.parametrize(
    "status,previous_kind,method",
    [
        ("FAIL", "none", "POST"),
        ("PASS", "none", None),
        ("WARN", "none", None),
        ("PASS", "legacy", "PATCH"),
        ("WARN", "legacy", "PATCH"),
        ("PASS", "failed", "PATCH"),
        ("FAIL", "newer", None),
        ("FAIL", "same-time-pass", None),
        ("FAIL", "older-pass", "PATCH"),
        ("PASS", "identical", None),
    ],
)
def test_merge_comment_shell_refreshes_saved_verdict_without_stale_rollback(
    tmp_path, status, previous_kind, method
):
    module = _load_runner()
    record = _publication_fixture()
    record["verdict"]["status"] = status
    report = "Saved comparison report\n"
    previous = ""
    if previous_kind == "legacy":
        previous = (
            f"<!-- dart-perf-merge:{record['run']['commit']} -->\n"
            "Post-merge performance check failed.\n"
        )
    elif previous_kind != "none":
        saved = copy.deepcopy(record)
        saved["verdict"]["status"] = "FAIL" if previous_kind == "failed" else "PASS"
        if previous_kind in ("failed", "older-pass"):
            saved["run"]["time"] = "2026-10-08T07:00:00+00:00"
        elif previous_kind == "newer":
            saved["run"]["time"] = "2026-10-08T09:00:00+00:00"
        previous = module.merge_comment(saved, report, "existing comment")
    module.write_json(tmp_path / "perf.json", record)
    (tmp_path / "perf.md").write_text(report)
    (tmp_path / "previous.json").write_text(json.dumps({"body": previous}))
    stub = """
    gh() {
      if [[ "$*" == *--paginate* ]]; then
        if [[ "$HAS_PREVIOUS" == true ]]; then echo 42; fi
      elif [[ "$*" == *--method* ]]; then
        printf '%s\\n' "$*" > "$RUNNER_TEMP/mutation"
        cp "$RUNNER_TEMP/perf-comment.json" "$RUNNER_TEMP/submitted.json"
      else
        cat "$RUNNER_TEMP/previous.json"
      fi
    }
    """
    result = subprocess.run(
        [
            "bash",
            "-e",
            "-o",
            "pipefail",
            "-c",
            stub + _perf_step("record", "Refresh the merged PR verdict comment")["run"],
        ],
        cwd=Path(__file__).resolve().parents[1],
        env={
            **os.environ,
            "PERF_OUTPUT": str(tmp_path),
            "RUNNER_TEMP": str(tmp_path),
            "PERF_HEAD": record["run"]["commit"],
            "PERF_PR": "3570",
            "GH_REPO": "dartsim/dart",
            "HAS_PREVIOUS": "true" if previous else "false",
        },
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    mutation = tmp_path / "mutation"
    assert mutation.exists() is (method is not None)
    if method:
        assert f"--method {method}" in mutation.read_text()
        body = json.loads((tmp_path / "submitted.json").read_text())["body"]
        assert body.startswith(f"<!-- dart-perf-merge:{record['run']['commit']} -->")
        assert body.endswith(report)
        assert ("check failed" if status == "FAIL" else "check now passes") in body


@pytest.mark.parametrize(
    "change,rationales",
    [
        ("allocations", ["Perf-Regression-Rationale"]),
        ("bytes", ["Perf-Regression-Rationale"]),
        ("ir", ["Perf-Regression-Rationale"]),
        ("geomean", ["Perf-Regression-Rationale"]),
        ("input", ["Rebaseline-Rationale"]),
        ("guards", ["Rebaseline-Rationale"]),
        ("signed-percentage", ["Rebaseline-Rationale"]),
        ("perturbation", []),
        ("missing-head", []),
        ("missing-base", []),
        ("nonfinite", []),
        ("build", []),
        ("smoke-perturbation", []),
        ("empty", []),
        ("unknown", []),
        ("mixed", ["Perf-Regression-Rationale", "Rebaseline-Rationale"]),
    ],
)
def test_merge_failure_comment_guidance_follows_comparison_policy(change, rationales):
    module = _load_runner()
    base = _micro_record(module, "dyn")
    head = copy.deepcopy(base)
    head["run"].update(commit="a" * 40, time="2026-10-08T08:00:00Z")
    row = head["results"][0]
    body = ""
    if change == "allocations":
        row["head"]["allocs_per_step"] = 1
    elif change == "bytes":
        row["head"]["bytes_per_step"] = 1
    elif change in ("ir", "geomean"):
        row["head"]["ir_per_step"] = 102 if change == "ir" else 100.6
    elif change == "input":
        row["input_sha"] = "changed workload"
    elif change in ("guards", "signed-percentage"):
        row["head"]["guards"]["hash"] = {"BM_Dynamics/10": "0x1123456789abcdef"}
        if change == "signed-percentage":
            row["head"]["ir_per_step"] = 102
            body = "Rebaseline-Rationale: dyn: intended work"
    elif change == "perturbation":
        row.update(gated=False, perturbations={"start4k": {"stable": False}})
    elif change == "missing-head":
        row["head"].pop("ir_per_step")
    elif change == "missing-base":
        base["results"][0]["head"].pop("ir_per_step")
    elif change == "nonfinite":
        row["head"]["guards"]["finite"] = False
    elif change == "empty":
        base["results"] = head["results"] = []
    elif change == "mixed":
        row["head"]["allocs_per_step"] = 1
        changed_input = copy.deepcopy(row)
        changed_input.update(row="changed-input", input_sha="new workload")
        base["results"].append({**copy.deepcopy(row), "row": "changed-input"})
        head["results"].append(changed_input)
    record = module.compare(base, head, body)
    extra_failures = {
        "build": "head smoke build failed: compiler error",
        "mixed": "head smoke build failed: compiler error",
        "smoke-perturbation": "head smoke rows did not pass their perturbation checks",
        "unknown": "unrecognized evidence failure",
    }
    if change in extra_failures:
        record["verdict"]["failures"].append(extra_failures[change])
        record["verdict"]["status"] = "FAIL"
    assert record["verdict"]["status"] == "FAIL"
    comment = module.merge_comment(record, "Saved comparison report")
    guidance = comment.split("Rerun the full workflow", 1)[0]
    assert "Post-merge performance check failed" in guidance
    assert ("Add the applicable" in guidance) == bool(rationales)
    for kind in ("Perf-Regression-Rationale", "Rebaseline-Rationale"):
        assert (kind in guidance) == (kind in rationales)
    needs_fix = change in (
        "perturbation",
        "missing-head",
        "missing-base",
        "nonfinite",
        "build",
        "smoke-perturbation",
        "empty",
        "unknown",
        "mixed",
    )
    assert ("Fix the non-waivable failures" in guidance) == needs_fix
    if needs_fix:
        assert record["verdict"]["failures"][0 if change != "mixed" else -1] in guidance
    assert "including measurement" in comment


@pytest.mark.parametrize("change", [None, "commit", "fingerprint", "results", "guards"])
def test_nightly_records_skip_only_unchanged_latest_measurement(tmp_path, change):
    module = _load_runner()
    path = tmp_path / "record.json"
    module.write_json(path, _publication_fixture())
    record = module.publication_record(path, "nightly")
    module.write_publication(tmp_path, record)
    original = list((tmp_path / "performance/records/main/2026").glob("*.json"))[0]
    saved = original.read_bytes()
    record["run"]["time"] = "2026-10-09T08:00:00+00:00"
    if change == "commit":
        record["run"]["commit"] = "c" * 40
    elif change == "fingerprint":
        record["run"]["env"]["fingerprint"] = "2" * 64
    elif change == "results":
        record["results"][0]["head"]["ir_per_step"] += 1
    elif change == "guards":
        record["results"][0]["head"]["guards"]["contacts"] += 1
    # Ancestry is covered separately; isolate immutable-record selection here.
    module.nightly_table_can_advance = lambda *_: change != "commit"
    changed = module.write_publication(tmp_path, record)
    records = list((tmp_path / "performance/records/main/2026").glob("*.json"))
    assert len(records) == (1 if change is None else 2)
    assert original.read_bytes() == saved
    if change is None:
        assert changed == ["performance/guards/main.md"]
        assert record["run"]["time"] in (tmp_path / changed[0]).read_text()


def test_chart_writer_rerun_does_not_duplicate_commit_and_fingerprint(tmp_path):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    module.chart_data(tmp_path, record)
    data_path = tmp_path / "performance/dart6-ir/data.js"
    saved = data_path.read_bytes()
    record["run"]["time"] = "2026-10-09T08:00:00+00:00"
    module.chart_data(tmp_path, record)
    assert data_path.read_bytes() == saved


@pytest.mark.parametrize("writer", ["record", "chart"])
def test_smoke_provenance_distinguishes_reruns_and_preserves_counts(tmp_path, writer):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    record["run"]["tier"] = "merge"
    env = copy.deepcopy(record["run"]["env"])
    env.update(fingerprint="2" * 64, harness_sha="2" * 64)
    record["results"][0]["head_env"] = env
    write = module.write_publication if writer == "record" else module.chart_data
    write(tmp_path, record)
    record["run"]["time"] = "2026-10-08T09:00:00+00:00"
    env["runner"]["name"] = "another hosted runner"
    write(tmp_path, record)
    assert len(_chart_points(tmp_path)) == 1
    env.update(fingerprint="3" * 64, harness_sha="3" * 64)
    write(tmp_path, record)
    points = _chart_points(tmp_path)
    assert len(points) == 2
    assert (
        f"fingerprint changed: {'2' * 64} -> {'3' * 64}"
        in points[-1]["benches"][0]["extra"]
    )
    if writer == "record":
        assert (
            len(list((tmp_path / "performance/records/main/2026").glob("*.json"))) == 2
        )
    record["run"]["time"] = "2026-10-08T10:00:00+00:00"
    record["results"][0]["head"]["ir_per_step"] += 1
    with pytest.raises(ValueError, match="changed deterministic counts"):
        write(tmp_path, record)


def test_chart_equal_timestamps_have_the_same_order_and_fingerprint_annotations(
    tmp_path,
):
    module = _load_runner()
    records = [_publication_fixture(), _publication_fixture()]
    records[1]["run"]["commit"] = "c" * 40
    records[1]["run"]["env"]["fingerprint"] = "2" * 64
    scripts = []
    for name, ordered in (("forward", records), ("reverse", records[::-1])):
        pages = tmp_path / name
        _stock_chart_template(pages)
        for record in ordered:
            module.chart_data(pages, record)
        scripts.append((pages / "performance/dart6-ir/data.js").read_bytes())
    assert scripts[0] == scripts[1]


def test_nightly_changed_results_retain_history_even_when_matching_an_older_run(
    tmp_path,
):
    module = _load_runner()
    path = tmp_path / "record.json"
    module.write_json(path, _publication_fixture())
    record = module.publication_record(path, "nightly")
    for hour, contacts in ((8, 3), (9, 4), (10, 3)):
        record["run"]["time"] = f"2026-10-08T{hour:02d}:00:00+00:00"
        record["results"][0]["head"]["guards"]["contacts"] = contacts
        module.write_publication(tmp_path, record)
    records = list((tmp_path / "performance/records/main/2026").glob("*.json"))
    assert len(records) == 3
    table = (tmp_path / "performance/guards/main.md").read_bytes()
    record["run"]["time"] = "2026-10-08T09:00:00+00:00"
    record["results"][0]["head"]["guards"]["contacts"] = 4
    assert module.write_publication(tmp_path, record) == []
    assert (tmp_path / "performance/guards/main.md").read_bytes() == table
    assert len(list((tmp_path / "performance/records/main/2026").glob("*.json"))) == 3


def test_publication_merge_idempotence_fingerprint_flags_and_chart_window(tmp_path):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    path = tmp_path / "record.json"
    module.write_json(path, _publication_fixture())
    record = module.publication_record(path, "merge", 3570)
    module.write_publication(tmp_path, record)
    module.write_publication(tmp_path, record)
    points = _chart_points(tmp_path)
    assert len(points) == 1
    assert points[0]["tool"] == "customSmallerIsBetter"
    assert [bench["value"] for bench in points[0]["benches"]] == [100_000, 0]
    assert [bench["name"] for bench in points[0]["benches"]] == [
        "s3w/dart@1:01234567 Ir",
        "s3w/dart@1:01234567 allocations",
    ]
    assert (tmp_path / "performance/dart6-ir/index.html").read_text() == (
        tmp_path / "performance/dart6/index.html"
    ).read_text()
    record["run"]["time"] = "2026-10-08T08:01:00+00:00"
    assert module.write_publication(tmp_path, record) == []
    record["results"][0]["head"]["ir_per_step"] += 1
    with pytest.raises(ValueError, match="changed deterministic counts"):
        module.write_publication(tmp_path, record)
    record["run"]["env"]["fingerprint"] = "2" * 64
    module.write_publication(tmp_path, record)
    assert "fingerprint changed" in _chart_points(tmp_path)[-1]["benches"][0]["extra"]
    for minute in range(251):
        record["run"].update(commit=f"{minute:040x}", time="2026-10-09T00:00:00+00:00")
        module.chart_data(tmp_path, record)
    assert len(_chart_points(tmp_path)) == 250
    assert len(list((tmp_path / "performance/records/main/2026").glob("*.json"))) == 2


def test_chart_out_of_order_publication_keeps_newest_points_and_latest_value(tmp_path):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    start = module.datetime.fromisoformat(record["run"]["time"])
    # Fill the window backwards, then publish one more older measurement.
    for minute in range(250, -1, -1):
        record["run"].update(
            commit=f"{minute:040x}",
            time=(start + module.timedelta(minutes=minute)).isoformat(),
        )
        record["run"]["env"]["fingerprint"] = f"{minute:064x}"
        record["results"][0]["head"]["ir_per_step"] = minute
        module.chart_data(tmp_path, record)
    data = json.loads(
        (tmp_path / "performance/dart6-ir/data.js")
        .read_text()
        .removeprefix("window.BENCHMARK_DATA = ")
    )
    points = data["entries"]["DART 6 deterministic counts"]
    assert [point["commit"]["id"] for point in points] == [
        f"{minute:040x}" for minute in range(1, 251)
    ]
    assert data["lastUpdate"] == int(
        (start + module.timedelta(minutes=250)).timestamp() * 1000
    )
    assert points[-1]["benches"][0]["value"] == 250
    assert (
        f"fingerprint changed: {249:064x} -> {250:064x}"
        in points[-1]["benches"][0]["extra"]
    )


def test_nightly_out_of_order_publication_preserves_latest_table_and_records(
    monkeypatch, tmp_path
):
    module = _load_runner()
    source = tmp_path / "source"
    source.mkdir()

    def git(*arguments):
        return subprocess.run(
            ["git", "-C", str(source), *arguments],
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip()

    git("init")
    commits = []
    for index in range(3):
        git(
            "-c",
            "user.name=test",
            "-c",
            "user.email=test@example.com",
            "commit",
            "--allow-empty",
            "-m",
            str(index),
        )
        commits.append(git("rev-parse", "HEAD"))
    monkeypatch.setattr(module, "ROOT", source)
    path = tmp_path / "record.json"
    module.write_json(path, _publication_fixture())
    record = module.publication_record(path, "nightly")
    record["run"]["commit"] = commits[1]
    module.write_publication(tmp_path, record)
    table_path = tmp_path / "performance/guards/main.md"
    table = table_path.read_text()
    # A manual dispatch of an ancestor can be measured later than main.
    record["run"].update(commit=commits[0], time="2026-10-08T09:00:00+00:00")
    assert "performance/guards/main.md" not in module.write_publication(
        tmp_path, record
    )
    assert table_path.read_text() == table
    # Commit ancestry also wins when the newer run measured earlier but finished later.
    record["run"].update(commit=commits[2], time="2026-10-08T07:00:00+00:00")
    module.write_publication(tmp_path, record)
    table = table_path.read_text()
    assert f"for `{commits[2]}`" in table
    # An overlapping measurement of the same head must not replace its newer run.
    record["run"]["time"] = "2026-10-08T06:00:00+00:00"
    module.write_publication(tmp_path, record)
    assert table_path.read_text() == table
    records = list((tmp_path / "performance/records/main/2026").glob("*.json"))
    assert len(records) == 4  # The newest record by measurement time is the ancestor.
    saved = {p.name: p.read_bytes() for p in records}
    assert module.write_publication(tmp_path, record) == []
    assert {p.name: p.read_bytes() for p in records} == saved
    assert (
        _perf_step("nightly", "Checkout main publisher")["with"]["fetch-depth"] == "0"
    )


def test_nightly_latest_identity_dedup_and_guard_drift(tmp_path):
    module = _load_runner()
    fixture = _publication_fixture()
    fixture["results"][0]["row"] = "S6"
    fixture["results"][0]["window"] = {"warmup": 0, "steps": 2400}
    fixture["results"][0]["head"].update(
        max_penetration=0.123,
        checkpoints=[{"step": 2400, "max_penetration": 0.123, "resting": "0/71"}],
    )
    path = tmp_path / "record.json"
    module.write_json(path, fixture)
    record = module.publication_record(path, "nightly")
    module.write_publication(tmp_path, record)
    table = (tmp_path / "performance/guards/main.md").read_text()
    assert "0.123" in table and "2400" in table and "0/71" in table
    assert (
        "docs/dev_tasks/dart6_performance_generalization/01-baseline-evidence.md"
        in table
    )
    record["run"]["time"] = "2026-10-08T09:00:00+00:00"
    record["results"][0]["head"]["guards"]["resting"] = "71/71"
    record["results"][0]["head"]["checkpoints"][0]["max_penetration"] = 0.1
    module.write_publication(tmp_path, record)
    table = (tmp_path / "performance/guards/main.md").read_text()
    assert "S3 / S6 drift since the prior nightly" in table
    assert "S6/dart checkpoints" in table
    assert module.write_publication(tmp_path, record) == []
    assert (tmp_path / "performance/guards/main.md").read_text() == table
    record["run"]["env"]["fingerprint"] = "2" * 64
    module.write_publication(tmp_path, record)
    record["run"]["time"] = "2026-10-08T10:00:00+00:00"
    record["run"]["env"]["fingerprint"] = "1" * 64
    module.write_publication(tmp_path, record)
    records = list((tmp_path / "performance/records/main/2026").glob("*.json"))
    assert (
        len(records) == 4
    )  # Changed guards and fingerprint transitions stay immutable.
    assert not (tmp_path / "performance/dart6-ir").exists()


def test_merge_rationale_rerun_records_new_acknowledgment_without_chart_duplicate(
    tmp_path,
):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    path = tmp_path / "record.json"
    fixture = _publication_fixture()
    fixture["verdict"].update(status="FAIL", failures=["Ir rationale required"])
    module.write_json(path, fixture)
    record = module.publication_record(path, "merge", 3570)
    module.write_publication(tmp_path, record)
    failed = copy.deepcopy(record)
    record["run"]["accepted"] = [
        {
            "kind": "regression",
            "rows": ["s3w/dart"],
            "rationale": "Perf-Regression-Rationale: s3w/dart: intended work",
        }
    ]
    record["verdict"].update(status="PASS", failures=[])
    module.write_publication(tmp_path, record)
    records = sorted((tmp_path / "performance/records/main/2026").glob("*.json"))
    assert len(records) == 2
    assert json.loads(records[0].read_text())["verdict"]["status"] == "FAIL"
    assert (
        json.loads(records[1].read_text())["run"]["accepted"]
        == record["run"]["accepted"]
    )
    assert module.write_publication(tmp_path, record) == []
    assert module.write_publication(tmp_path, failed) == []
    assert len(list((tmp_path / "performance/records/main/2026").glob("*.json"))) == 2
    assert len(_chart_points(tmp_path)) == 1


def test_guard_table_shows_contact_pair_changes_as_drift(tmp_path):
    module = _load_runner()
    fixture = _publication_fixture()
    fixture["results"][0]["row"] = "S6"
    fixture["results"][0]["window"] = {"warmup": 0, "steps": 2400}
    fixture["results"][0]["head"]["guards"]["pairs"] = 79
    path = tmp_path / "record.json"
    module.write_json(path, fixture)
    record = module.publication_record(path, "nightly")
    module.write_publication(tmp_path, record)
    assert "| 79 |" in (tmp_path / "performance/guards/main.md").read_text()
    record["run"]["time"] = "2026-10-08T09:00:00+00:00"
    record["results"][0]["head"]["guards"]["pairs"] = 80
    module.write_publication(tmp_path, record)
    table = (tmp_path / "performance/guards/main.md").read_text()
    assert "| 80 |" in table
    assert "S3 / S6 drift since the prior nightly" in table


def test_merge_rerun_with_changed_allocations_is_not_deduplicated(tmp_path):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    path = tmp_path / "record.json"
    module.write_json(path, _publication_fixture())
    record = module.publication_record(path, "merge", 3570)
    module.write_publication(tmp_path, record)
    changed = copy.deepcopy(record)
    changed["results"][0]["head"]["allocs_per_step"] = 1.0
    with pytest.raises(ValueError, match="deterministic counts"):
        module.write_publication(tmp_path, changed)


@pytest.mark.parametrize("writer", ["record", "chart"])
@pytest.mark.parametrize(
    "section,key,value",
    [
        *(
            ("guards", key, value)
            for key, value in (
                ("hash", "changed"),
                ("contacts", 4),
                ("pairs", 2),
                ("resting", "3/3"),
                ("finite", False),
                ("cap_hit", True),
                ("max_penetration", 0.2),
            )
        ),
        ("head", "ir_per_step", 100_001),
        ("head", "allocs_per_step", 1),
        ("head", "bytes_per_step", 1),
        ("head", "max_penetration", 0.2),
        ("head", "checkpoints", [{"step": 1, "max_penetration": 0.2}]),
        ("head", "time_advanced", False),
        ("head", "cases", ["changed case"]),
        ("head", "micro_instrumented", False),
        ("head", "allocs", 1),
        ("head", "bytes", 1),
        ("head", "est_cycles_per_step", 101),
        ("row", "input_sha", "changed"),
        ("row", "version", 2),
        ("row", "window", {"warmup": 50, "steps": 100}),
        ("row", "threads", 4),
        ("row", "method", "native"),
        ("row", "collection_signature", "other()"),
        ("row", "status", "broken"),
        ("row", "gated", False),
        ("row", "qualification_required", False),
        (
            "row",
            "perturbations",
            {"start4k": {"stable": False, "guards": {"hash": "changed"}}},
        ),
    ],
)
def test_rerun_rejects_changed_deterministic_evidence(
    tmp_path, writer, section, key, value
):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    record["run"]["tier"] = "merge"
    row = record["results"][0]
    row.update(
        input_sha="original",
        threads=1,
        window={"warmup": 0, "steps": 100},
        collection_signature="step()",
    )
    row["head"].update(
        bytes_per_step=0,
        max_penetration=0.1,
        checkpoints=[{"step": 1, "max_penetration": 0.1}],
        time_advanced=True,
    )
    row["head"]["guards"].update(pairs=1, max_penetration=0.1)
    write = module.write_publication if writer == "record" else module.chart_data
    write(tmp_path, record)
    saved = {path: path.read_bytes() for path in tmp_path.rglob("*") if path.is_file()}
    changed = copy.deepcopy(record)
    row = changed["results"][0]
    target = (
        row
        if section == "row"
        else row["head"] if section == "head" else row["head"]["guards"]
    )
    target[key] = value
    changed["run"]["time"] = "2026-10-08T09:00:00+00:00"
    with pytest.raises(ValueError, match="deterministic counts or guards/inputs"):
        write(tmp_path, changed)
    assert {
        path: path.read_bytes() for path in tmp_path.rglob("*") if path.is_file()
    } == saved


@pytest.mark.parametrize("writer", ["record", "chart"])
def test_rerun_identity_excludes_advisory_metrics_and_install_paths(tmp_path, writer):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    record["run"]["tier"] = "merge"
    write = module.write_publication if writer == "record" else module.chart_data
    write(tmp_path, record)
    saved = {path: path.read_bytes() for path in tmp_path.rglob("*") if path.is_file()}
    record["run"]["time"] = "2026-10-08T09:00:00+00:00"
    record["results"][0]["head"].update(
        wall_ms_per_step=15, max_rss_kb=1000, libdart="/another/install/libdart.so"
    )
    record["results"][0]["parent"].update(
        wall_ms_per_step=20, max_rss_kb=2000, libdart="/base/install/libdart.so"
    )
    write(tmp_path, record)
    assert {
        path: path.read_bytes() for path in tmp_path.rglob("*") if path.is_file()
    } == saved


@pytest.mark.parametrize(
    "section,key,value",
    [
        ("run", "parent", "c" * 40),
        ("parent", "ir_per_step", 99_999),
        ("parent", "allocs_per_step", 1),
        ("parent", "bytes_per_step", 1),
        ("parent", "checkpoints", [{"step": 1, "max_penetration": 0.2}]),
        ("parent", "micro_instrumented", False),
        ("parent", "time_advanced", False),
        ("guards", "hash", "changed"),
        ("guards", "finite", False),
        ("guards", "contacts", 4),
    ],
)
def test_merge_rerun_rejects_changed_parent_but_chart_deduplicates_head(
    tmp_path, section, key, value
):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    record["run"]["tier"] = "merge"
    module.write_publication(tmp_path, record)
    saved = {path: path.read_bytes() for path in tmp_path.rglob("*") if path.is_file()}
    changed = copy.deepcopy(record)
    parent = changed["results"][0]["parent"]
    target = (
        changed["run"]
        if section == "run"
        else parent if section == "parent" else parent["guards"]
    )
    target[key] = value
    module.chart_data(tmp_path, changed)
    assert len(_chart_points(tmp_path)) == 1
    with pytest.raises(ValueError, match="deterministic counts or guards/inputs"):
        module.write_publication(tmp_path, changed)
    assert {
        path: path.read_bytes() for path in tmp_path.rglob("*") if path.is_file()
    } == saved


def test_merge_rerun_checks_all_matching_history(tmp_path):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    record["run"]["tier"] = "merge"
    module.write_publication(tmp_path, record)
    original = next((tmp_path / "performance/records/main/2026").glob("*.json"))
    record["verdict"]["status"] = "WARN"
    record["run"]["time"] = "2026-10-08T09:00:00+00:00"
    module.write_publication(tmp_path, record)
    older = json.loads(original.read_text())
    older["results"][0]["head"]["guards"]["hash"] = "inconsistent older run"
    module.write_json(original, older)
    with pytest.raises(ValueError, match="guards/inputs"):
        module.write_publication(tmp_path, record)


def test_nightly_same_timestamp_with_changed_evidence_keeps_both_records(tmp_path):
    module = _load_runner()
    record = _publication_fixture()
    record["run"]["tier"] = "nightly"
    module.write_publication(tmp_path, record)
    record["results"][0]["head"]["guards"]["contacts"] += 1
    changed = module.write_publication(tmp_path, record)
    assert len(changed) == 1 and changed[0].endswith("-nightly.json")
    records = sorted((tmp_path / "performance/records/main/2026").glob("*.json"))
    assert [
        json.loads(path.read_text())["results"][0]["head"]["guards"]["contacts"]
        for path in records
    ] == [3, 4]
    assert module.write_publication(tmp_path, record) == []


def test_guard_table_keeps_all_rows_before_errors_and_checkpoints():
    module = _load_runner()
    record = _publication_fixture()
    record["results"] = [copy.deepcopy(record["results"][0]) for _ in range(3)]
    for row, name in zip(record["results"], ("S3", "S6", "S1")):
        row["row"] = name
    record["results"][0].update(error="failed\nwith details", status="broken")
    record["results"][1]["head"]["checkpoints"] = [
        {"step": 5000, "max_penetration": 0.1}
    ]
    report = module.guard_table(record)
    lines = report.splitlines()
    start = next(i for i, line in enumerate(lines) if line.startswith("| Row |"))
    assert all(line.startswith("|") for line in lines[start : start + 5])
    assert [line.split("|")[1].strip() for line in lines[start + 2 : start + 5]] == [
        "S3",
        "S6",
        "S1",
    ]
    assert (
        report.index("| S1 |")
        < report.index("`S3/dart`: failed")
        < report.index("`S6/dart` checkpoints:")
    )


@pytest.mark.parametrize("reporter", ["markdown", "guard_table"])
def test_markdown_table_cells_escape_pipes_and_line_breaks(reporter):
    module = _load_runner()
    record = _publication_fixture()
    record["run"]["env"].update(
        valgrind="test", compiler="test", glibc="test", preset="test"
    )
    record["results"] *= 2
    row = record["results"][0]
    row.update(
        row="S6",
        status="broken|details\nnext",
        gate_reason="failed|details\r\nnext\rlast",
        failures=[],
    )
    row["delta"]["bytes"] = 0
    row["head"]["guards"]["resting"] = "0/3|details\nnext"
    report = getattr(module, reporter)(record)
    lines = report.splitlines()
    start = next(i for i, line in enumerate(lines) if line.startswith("| Row |"))
    table = lines[start : start + 4]
    assert all(line.startswith("|") for line in table)
    assert len({len(re.split(r"(?<!\\)\|", line)) for line in table}) == 1
    assert "\\|details<br>next" in table[2]


@pytest.mark.parametrize(
    "field,value",
    [
        ("threads", 4),
        ("window", {"warmup": 50, "steps": 100}),
        ("method", "native"),
        ("collection_signature", "new()"),
        ("micro_instrumented", False),
        ("fingerprint", "2" * 64),
    ],
)
def test_chart_annotates_comparability_changes_across_missing_rows(
    tmp_path, field, value
):
    module = _load_runner()
    record = _publication_fixture()
    record["results"][0].update(
        input_sha="01234567" + "a" * 56,
        threads=1,
        window={"warmup": 0, "steps": 100},
        collection_signature="old()",
    )
    stable = copy.deepcopy(record["results"][0])
    stable["row"] = "stable"
    record["results"].append(stable)
    records = [copy.deepcopy(record) for _ in range(3)]
    for i, point in enumerate(records):
        point["run"].update(
            commit=f"{i + 1:040x}", time=f"2026-10-08T{8 + i:02d}:00:00+00:00"
        )
    records[1]["results"][0]["status"] = "unsupported"
    if field == "fingerprint":
        records[2]["run"]["env"]["fingerprint"] = value
    elif field == "micro_instrumented":
        records[2]["results"][0]["head"][field] = value
    else:
        records[2]["results"][0][field] = value
    scripts = []
    for name, ordered in (("forward", records), ("reverse", records[::-1])):
        pages = tmp_path / name
        _stock_chart_template(pages)
        for point in ordered:
            module.chart_data(pages, point)
        points = _chart_points(pages)
        assert all(
            f"{field} changed:" in bench["extra"]
            for bench in points[-1]["benches"]
            if bench["name"].startswith("s3w/")
        )
        if field != "fingerprint":
            assert all(
                f"{field} changed:" not in bench["extra"]
                for point in points
                for bench in point["benches"]
                if bench["name"].startswith("stable/")
            )
        assert all("changed:" not in bench["extra"] for bench in points[0]["benches"])
        scripts.append((pages / "performance/dart6-ir/data.js").read_bytes())
    assert scripts[0] == scripts[1]


@pytest.mark.parametrize(
    "field,value",
    [
        ("version", 2),
        ("det", "ode"),
        ("row", "other"),
        ("input_sha", "89abcdef" + "b" * 56),
    ],
)
def test_chart_names_split_row_version_detector_and_input_changes(
    tmp_path, field, value
):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    module.chart_data(tmp_path, record)
    record["run"].update(commit="c" * 40, time="2026-10-08T09:00:00+00:00")
    record["results"][0][field] = value
    module.chart_data(tmp_path, record)
    first, second = _chart_points(tmp_path)
    assert {bench["name"] for bench in first["benches"]}.isdisjoint(
        bench["name"] for bench in second["benches"]
    )
    if field == "input_sha":
        assert {bench["name"] for bench in second["benches"]} == {
            "s3w/dart@1:89abcdef Ir",
            "s3w/dart@1:89abcdef allocations",
        }
        assert all("changed:" not in bench["extra"] for bench in second["benches"])


def test_chart_legacy_points_validate_counts_and_split_unknown_inputs(tmp_path):
    module = _load_runner()
    _stock_chart_template(tmp_path)
    record = _publication_fixture()
    module.chart_data(tmp_path, record)
    data_path = tmp_path / "performance/dart6-ir/data.js"
    data = json.loads(data_path.read_text().removeprefix("window.BENCHMARK_DATA = "))
    point = data["entries"]["DART 6 deterministic counts"][0]
    point.pop("measurement")
    for bench in point["benches"]:
        for field in (
            "input_sha",
            "threads",
            "window",
            "method",
            "collection_signature",
            "micro_instrumented",
        ):
            bench.pop(field)
    data_path.write_text("window.BENCHMARK_DATA = " + json.dumps(data))
    saved = data_path.read_bytes()
    module.chart_data(tmp_path, record)
    assert data_path.read_bytes() == saved
    changed = copy.deepcopy(record)
    changed["results"][0]["head"]["ir_per_step"] += 1
    with pytest.raises(ValueError, match="deterministic counts"):
        module.chart_data(tmp_path, changed)
    record["run"].update(commit="c" * 40, time="2026-10-08T09:00:00+00:00")
    record["results"][0]["input_sha"] = "89abcdef" + "b" * 56
    module.chart_data(tmp_path, record)
    assert all(
        ":89abcdef " in bench["name"] and "input_sha changed:" not in bench["extra"]
        for bench in _chart_points(tmp_path)[1]["benches"]
    )


@pytest.mark.parametrize(
    "outcome", ["race", "rejected", "identical", "no-chart", "nightly"]
)
def test_publish_regenerates_after_rejection_without_losing_concurrent_evidence(
    monkeypatch, tmp_path, outcome
):
    module = _load_runner()
    tier = "nightly" if outcome == "nightly" else "merge"
    _trusted_publication(monkeypatch, "schedule" if tier == "nightly" else "push")
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", os.devnull)
    real_run = subprocess.run

    def git(directory, *arguments):
        return real_run(
            ["git", "-C", str(directory), *arguments],
            check=True,
            text=True,
            capture_output=True,
        )

    remote, pages, competitor = (
        tmp_path / name for name in ("remote.git", "pages", "competitor")
    )
    real_run(
        ["git", "init", "--bare", "--initial-branch=gh-pages", str(remote)],
        check=True,
        capture_output=True,
    )
    real_run(["git", "clone", str(remote), str(pages)], check=True, capture_output=True)
    _stock_chart_template(pages)

    def commit(directory):
        git(directory, "add", "performance")
        git(
            directory,
            "-c",
            "user.name=test",
            "-c",
            "user.email=test@example.com",
            "commit",
            "-m",
            "test",
        )

    commit(pages)
    git(pages, "push", "origin", "HEAD:gh-pages")
    real_run(
        ["git", "clone", str(remote), str(competitor)], check=True, capture_output=True
    )
    baseline = git(remote, "rev-parse", "gh-pages").stdout.strip()
    path = tmp_path / "record.json"
    fixture = _publication_fixture()
    if outcome == "no-chart":
        # Distinct timestamps write disjoint files, so Git could rebase cleanly
        # while preserving duplicate evidence for the same merge identity.
        fixture["results"][0]["status"] = "failed"
    module.write_json(path, fixture)
    incoming = module.publication_record(path, tier)
    concurrent = copy.deepcopy(incoming)
    if outcome != "identical":
        concurrent["run"]["time"] = (
            "2026-10-08T09:00:00+00:00"
            if tier == "nightly"
            else "2026-10-08T07:00:00+00:00"
        )
    if outcome in ("race", "rejected"):
        concurrent["run"]["commit"] = "c" * 40
        concurrent["run"]["env"]["fingerprint"] = "2" * 64
    if tier == "nightly":
        concurrent["results"][0]["head"]["guards"]["contacts"] = 4
    pushes = []
    checkouts = []

    def raced_run(command, *args, **kwargs):
        if command[:3] == ["git", "-C", str(pages)]:
            if command[3] == "checkout":
                checkouts.append(command)
            if command[3] == "push":
                if outcome == "rejected":
                    pushes.append(command)
                    return subprocess.CompletedProcess(
                        command, 1, "", "simulated remote rejection"
                    )
                if not pushes:
                    module.write_publication(competitor, concurrent)
                    commit(competitor)
                    git(competitor, "push", "origin", "HEAD:gh-pages")
                pushes.append(command)
        return real_run(command, *args, **kwargs)

    monkeypatch.setattr(module.subprocess, "run", raced_run)
    args = module.parser().parse_args(
        ["publish", "--record", str(path), "--tier", tier, "--pages-dir", str(pages)]
    )
    if outcome == "rejected":
        with pytest.raises(
            RuntimeError, match="rejected after 5 fetch/regenerate attempts"
        ):
            module.publish(args)
        assert len(pushes) == 5
        assert git(remote, "rev-parse", "gh-pages").stdout.strip() == baseline
        return
    deduplicated = outcome in ("identical", "no-chart")
    assert module.publish(args) is not deduplicated
    assert len(pushes) == (1 if deduplicated else 2)
    assert len(checkouts) == 2
    assert all(
        not any("force" in argument for argument in command) for command in pushes
    )
    git(remote, "merge-base", "--is-ancestor", baseline, "gh-pages")
    if tier == "nightly":
        assert (pages / "performance/guards/main.md").read_text() == module.guard_table(
            concurrent
        )
    elif outcome == "no-chart":
        assert not (pages / "performance/dart6-ir/data.js").exists()
    else:
        points = _chart_points(pages)
        assert [point["commit"]["id"] for point in points] == (
            ["a" * 40] if outcome == "identical" else ["c" * 40, "a" * 40]
        )
        if outcome != "identical":
            assert "fingerprint changed" in points[-1]["benches"][0]["extra"]
    assert len(list((pages / "performance/records/main/2026").glob("*.json"))) == (
        1 if deduplicated else 2
    )
    assert module.publish(args) is False
