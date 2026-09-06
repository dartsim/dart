"""Review evidence and real Git pre-push behavior, including Git for Windows.

All remotes and worktrees are disposable fixtures. No network, models, or user
Git configuration are involved. Unlike the older commit-guard suite, these
tests deliberately run on native Windows as well as POSIX.
"""

import argparse
import ast
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
GATE = ROOT / "scripts" / "review_gate.py"
INSTALLER = ROOT / "scripts" / "install_git_hooks.py"


def run(repo, env, *args, input=None):
    return subprocess.run(
        args,
        cwd=repo,
        env=env,
        input=input,
        capture_output=True,
        encoding="utf-8",
        timeout=30,
    )


def git(repo, env, *args):
    result = run(repo, env, "git", *args)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout.strip()


def commit(repo, env, content="changed\n", path="input.txt"):
    (repo / path).write_text(content, encoding="utf-8")
    git(repo, env, "add", path)
    git(repo, env, "commit", "-qm", "Fixture change")
    return git(repo, env, "rev-parse", "HEAD")


@pytest.fixture
def repository(tmp_path):
    repo = tmp_path / "repository with spaces"
    repo.mkdir()
    remote = tmp_path / "remote.git"
    env = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith(("GIT_", "DART_HOOK_", "DART_SKIP_"))
    }
    env.update(
        {
            "GIT_CONFIG_GLOBAL": os.devnull,
            "GIT_CONFIG_SYSTEM": os.devnull,
            "GIT_AUTHOR_NAME": "DART Test",
            "GIT_COMMITTER_NAME": "DART Test",
            "GIT_AUTHOR_EMAIL": "test@example.invalid",
            "GIT_COMMITTER_EMAIL": "test@example.invalid",
            "DART_HOOK_PYTHON": str(Path(sys.executable)).replace("\\", "/"),
        }
    )
    git(repo, env, "init", "-q", "-b", "main")
    git(repo, env, "config", "core.autocrlf", "false")
    commit(repo, env, "base\n")
    git(repo, env, "init", "-q", "--bare", str(remote))
    git(repo, env, "remote", "add", "origin", str(remote))
    git(repo, env, "push", "-q", "origin", "main")
    git(repo, env, "switch", "-qc", "topic")
    commit(repo, env)
    return repo, remote, env


def prepare(repository, target="topic", authors=("author",)):
    repo, _, env = repository
    result = run(
        repo,
        env,
        sys.executable,
        str(GATE),
        "prepare",
        "--base",
        "origin/main",
        "--remote",
        "origin",
        "--target",
        "refs/heads/" + target,
        *[part for author in authors for part in ("--author-session", author)],
    )
    assert result.returncode == 0, result.stderr
    return result.stdout.strip()


def report(candidate_id, session="reviewer-a", scope="correctness", **changes):
    value = {
        "schema_version": 1,
        "candidate": candidate_id,
        "reviewer": {"session": session, "kind": "human"},
        "scope": scope,
        "status": "complete",
        "verdict": "clean",
        "summary": "Checked the complete candidate and its consumers.",
        "report": "Independent final report with inspected paths and evidence.",
        "coverage": ["Complete base-to-head diff and affected acceptance criteria"],
        "coverage_complete": True,
        "findings": [],
        "dispositions": [],
    }
    value.update(changes)
    return value


def record(repository, candidate, value, success=True):
    repo, _, env = repository
    source = repo.parent / "review.json"
    source.write_text(json.dumps(value), encoding="utf-8")
    result = run(repo, env, sys.executable, str(GATE), "record", candidate, str(source))
    assert (result.returncode == 0) == success, result.stdout + result.stderr
    return result


def pair(repository, candidate):
    record(repository, candidate, report(candidate))
    record(repository, candidate, report(candidate, "reviewer-b", "contracts"))


def install(repository):
    repo, _, env = repository
    result = run(repo, env, sys.executable, str(INSTALLER))
    assert result.returncode == 0, result.stderr


def install_custom_manager(repository, *, export=True):
    repo, _, env = repository
    runtime = repo / ".git" / "dart-review-gate.py"
    manager = repo.parent / "custom manager"
    manager.mkdir()
    guide = (ROOT / "docs" / "onboarding" / "ai-tools.md").read_text(encoding="utf-8")
    guidance = guide.split("### Custom Hook Managers\n", 1)[1]
    assert "pixi run install-hooks --custom-manager" in guidance
    handlers = re.findall(r"```sh\n(.*?)```", guidance, flags=re.DOTALL)
    assert len(handlers) == 2
    hook = manager / "pre-push"
    hook.write_bytes(handlers[1].encode("utf-8"))
    hook.chmod(0o755)
    git(repo, env, "config", "core.hooksPath", str(manager))
    if export:
        result = run(repo, env, sys.executable, str(INSTALLER), "--custom-manager")
        assert result.returncode == 0, result.stderr
    return runtime, manager


def directory_alias(path, target):
    try:
        path.symlink_to(target, target_is_directory=True)
    except OSError:
        if os.name != "nt":
            raise
        # Native Windows junctions do not require symbolic-link privileges.
        result = subprocess.run(
            ["cmd", "/c", "mklink", "/J", str(path), str(target)],
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stdout + result.stderr
    assert path.resolve() == target.resolve()


def push(repository, *refs, success=True):
    repo, _, env = repository
    result = run(repo, env, "git", "push", "origin", *(refs or ("topic",)))
    assert (result.returncode == 0) == success, result.stdout + result.stderr
    return result


def test_two_reviews_allow_first_push_and_repeat_prepare_preserves_them(repository):
    candidate = prepare(repository)
    pair(repository, candidate)
    assert prepare(repository) == candidate
    install(repository)
    result = push(repository)
    assert "two independent local reviews" in result.stderr


@pytest.mark.parametrize(
    "case", ["missing", "one", "same-session", "incomplete", "findings"]
)
def test_inadequate_evidence_blocks_actual_push(repository, case):
    candidate = prepare(repository)
    if case != "missing":
        record(repository, candidate, report(candidate))
    if case == "same-session":
        record(repository, candidate, report(candidate, scope="contracts"))
    if case == "incomplete":
        record(
            repository,
            candidate,
            report(candidate, "reviewer-b", "contracts", status="incomplete"),
        )
    if case == "findings":
        record(
            repository,
            candidate,
            report(
                candidate,
                "reviewer-b",
                "contracts",
                verdict="findings",
                findings=[
                    {
                        "id": "input-consumer",
                        "summary": "Input is untested",
                        "evidence": "Consumer skips the changed input",
                    }
                ],
            ),
        )
    install(repository)
    assert "BLOCKED" in push(repository, success=False).stderr


@pytest.mark.parametrize(
    "change, expected",
    [
        ({"reviewer": {"kind": "human", "session": "author"}}, "authoring session"),
        ({"coverage_complete": False}, "acceptance coverage"),
        ({"report": ""}, "final reviewer report"),
        (
            {"reviewer": {"kind": "agent", "session": "agent"}},
            "effective reviewer tool",
        ),
        ({"candidate": "f" * 64}, "different candidate"),
    ],
)
def test_invalid_reports_are_rejected(repository, change, expected):
    candidate = prepare(repository)
    result = record(repository, candidate, report(candidate, **change), success=False)
    assert expected in result.stderr


def test_same_report_cannot_be_counted_twice(repository):
    candidate = prepare(repository)
    value = report(candidate)
    record(repository, candidate, value)
    assert (
        "already recorded" in record(repository, candidate, value, success=False).stderr
    )


@pytest.mark.parametrize("damage", ["modified", "missing-last", "unindexed"])
def test_corrupt_or_missing_report_blocks_push(repository, damage):
    repo, _, _ = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    directory = repo / ".git" / "dart-review" / "candidates" / candidate / "reports"
    last = sorted(directory.glob("*.json"))[-1]
    if damage == "modified":
        value = json.loads(last.read_text())
        value["summary"] = "Modified report"
        last.write_text(json.dumps(value), encoding="utf-8")
    elif damage == "missing-last":
        last.unlink()
    else:
        (directory / "unexpected.json").write_text("{}", encoding="utf-8")
    install(repository)
    assert "report" in push(repository, success=False).stderr


def test_changed_outgoing_commit_and_moved_base_invalidate_reviews(repository):
    repo, _, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    old = git(repo, env, "rev-parse", "HEAD")
    new = commit(repo, env, "new substantive input\n")
    install(repository)
    assert "outgoing commit differs" in push(repository, success=False).stderr
    git(repo, env, "update-ref", "refs/remotes/origin/main", new)
    result = run(repo, env, sys.executable, str(GATE), "check", candidate)
    assert result.returncode and "base moved" in result.stderr
    assert git(repo, env, "rev-parse", old + "^{commit}") == old


def test_findings_survive_repair_candidate_until_independently_disposed(repository):
    repo, _, env = repository
    previous = prepare(repository)
    finding = {
        "id": "consumer-1",
        "summary": "Missed consumer",
        "evidence": "Consumer reads skipped input",
    }
    record(
        repository, previous, report(previous, verdict="findings", findings=[finding])
    )
    commit(repo, env, "consumer repaired\n")
    candidate = prepare(repository)
    pair(repository, candidate)
    install(repository)
    assert "unresolved findings: consumer-1" in push(repository, success=False).stderr
    record(
        repository,
        candidate,
        report(
            candidate,
            "reviewer-b",
            "contracts",
            dispositions=[
                {
                    "id": "consumer-1",
                    "status": "fixed",
                    "evidence": "Inspected updated consumer and regression case",
                }
            ],
        ),
    )
    push(repository)


def test_false_positive_can_be_rejected_without_a_new_commit(repository):
    candidate = prepare(repository)
    record(
        repository,
        candidate,
        report(
            candidate,
            verdict="findings",
            findings=[
                {
                    "id": "claim-1",
                    "summary": "Claim",
                    "evidence": "Claimed failure mechanism",
                }
            ],
        ),
    )
    record(
        repository,
        candidate,
        report(
            candidate,
            dispositions=[
                {
                    "id": "claim-1",
                    "status": "rejected",
                    "evidence": "Existing test disproves the claim",
                }
            ],
        ),
    )
    record(repository, candidate, report(candidate, "reviewer-b", "contracts"))
    install(repository)
    push(repository)


@pytest.mark.parametrize("revision", ["same", "authors-only", "repeated"])
def test_fixed_disposition_needs_a_changed_candidate_head(repository, revision):
    repo, _, env = repository
    previous = prepare(repository)
    record(
        repository,
        previous,
        report(
            previous,
            "finder",
            verdict="findings",
            findings=[{"id": "defect", "summary": "Defect", "evidence": "input.txt"}],
        ),
    )
    if revision == "authors-only":
        candidate = prepare(repository, authors=("author", "another-author"))
    elif revision == "repeated":
        commit(repo, env, "incomplete repair\n")
        candidate = prepare(repository)
        record(
            repository,
            candidate,
            report(
                candidate,
                "finder",
                verdict="findings",
                findings=[
                    {"id": "defect", "summary": "Still broken", "evidence": "input.txt"}
                ],
            ),
        )
    else:
        candidate = previous
    pair(repository, candidate)
    result = record(
        repository,
        candidate,
        report(
            candidate,
            dispositions=[
                {"id": "defect", "status": "fixed", "evidence": "Claimed repair"}
            ],
        ),
        success=False,
    )
    assert "changed head in a later candidate" in result.stderr
    install(repository)
    assert "unresolved findings: defect" in push(repository, success=False).stderr


@pytest.mark.parametrize("strategy", ["amend", "replace"])
def test_rewritten_unpublished_repair_retains_findings_and_requires_reviews(
    repository, strategy
):
    repo, _, env = repository
    previous = prepare(repository)
    old_head = git(repo, env, "rev-parse", "HEAD")
    record(
        repository,
        previous,
        report(
            previous,
            "finder",
            verdict="findings",
            findings=[{"id": "defect", "summary": "Defect", "evidence": "input.txt"}],
        ),
    )
    if strategy == "amend":
        (repo / "input.txt").write_text("amended repair\n", encoding="utf-8")
        git(repo, env, "add", "input.txt")
        git(repo, env, "commit", "--amend", "-qm", "Fixture repair")
    else:
        git(repo, env, "switch", "-C", "topic", "origin/main")
        commit(repo, env, "replacement repair\n")
    assert git(repo, env, "merge-base", old_head, "HEAD") != old_head
    candidate = prepare(repository)
    pair(repository, candidate)
    install(repository)
    assert "unresolved findings: defect" in push(repository, success=False).stderr
    record(
        repository,
        candidate,
        report(
            candidate,
            dispositions=[
                {
                    "id": "defect",
                    "status": "fixed",
                    "evidence": "Independent inspection verifies the rewritten consumer",
                }
            ],
        ),
    )
    push(repository)


@pytest.mark.parametrize("late", [False, True])
@pytest.mark.parametrize("status", ["fixed", "rejected"])
def test_active_author_dispositions_cannot_close_findings(repository, late, status):
    repo, _, env = repository
    previous = prepare(repository)
    record(
        repository,
        previous,
        report(
            previous,
            "finder",
            verdict="findings",
            findings=[{"id": "defect", "summary": "Defect", "evidence": "input.txt"}],
        ),
    )
    commit(repo, env, "first repair\n")
    interim = prepare(repository)
    disposition = report(
        interim,
        "future-author",
        dispositions=[
            {"id": "defect", "status": status, "evidence": "Inspected the consumer"}
        ],
    )
    if not late:
        record(repository, interim, disposition)
        pair(repository, interim)
    commit(repo, env, "additional authored repair\n")
    candidate = prepare(repository, authors=("author", "future-author"))
    pair(repository, candidate)
    if late:
        result = record(repository, interim, disposition, success=False)
        assert "active authoring session" in result.stderr
    install(repository)
    assert "unresolved findings: defect" in push(repository, success=False).stderr
    record(
        repository,
        candidate,
        report(
            candidate,
            dispositions=[
                {
                    "id": "defect",
                    "status": status,
                    "evidence": "Independent inspection of the active candidate",
                }
            ],
        ),
    )
    push(repository)


def test_destination_credentials_are_not_persisted_or_printed(repository):
    repo, _, env = repository
    location = "https://synthetic-user:synthetic-secret@example.invalid/repo?token=synthetic-query"
    git(repo, env, "remote", "set-url", "--push", "origin", location)
    candidate = prepare(repository)
    pair(repository, candidate)
    install(repository)
    head = git(repo, env, "rev-parse", "HEAD")
    payload = f"refs/heads/topic {head} refs/heads/topic {'0' * len(head)}\n"
    checker = repo / ".git" / "hooks" / "dart-review-gate.py"
    result = run(
        repo,
        env,
        sys.executable,
        str(checker),
        "pre-push",
        "origin",
        location,
        input=payload,
    )
    assert result.returncode == 0, result.stderr
    artifacts = b"".join(
        path.read_bytes() for path in (repo / ".git" / "dart-review").rglob("*.json")
    )
    for credential in (b"synthetic-user", b"synthetic-secret", b"synthetic-query"):
        assert credential not in artifacts
        assert credential.decode() not in result.stdout + result.stderr
    changed = run(
        repo,
        env,
        sys.executable,
        str(checker),
        "pre-push",
        "origin",
        location.replace("synthetic-secret", "different-secret"),
        input=payload,
    )
    assert changed.returncode != 0


def test_opaque_candidate_preserves_legacy_destination_history(repository, monkeypatch):
    monkeypatch.syspath_prepend(str(GATE.parent))
    import review_gate as gate

    repo, remote, env = repository
    initial = prepare(repository)
    store = gate.Store(repo)
    metadata = gate.read_json(store.candidate_dir(initial) / "candidate.json")
    metadata.pop("destination")
    metadata["location"] = str(remote)
    legacy = gate.digest(metadata)
    store.candidate_dir(initial).rename(store.candidate_dir(legacy))
    (store.candidate_dir(legacy) / "candidate.json").write_text(json.dumps(metadata))
    index = next((store.path / "targets").glob("*.json"))
    index.write_text(json.dumps({"candidate": legacy}))
    record(
        repository,
        legacy,
        report(
            legacy,
            "finder",
            verdict="findings",
            findings=[{"id": "legacy", "summary": "Defect", "evidence": "input.txt"}],
        ),
    )
    candidate = prepare(repository)
    current = gate.read_json(store.candidate_dir(candidate) / "candidate.json")
    assert candidate != legacy and current["previous"] == legacy
    assert "location" not in current
    pair(repository, candidate)
    install(repository)
    assert "unresolved findings: legacy" in push(repository, success=False).stderr
    commit(repo, env, "repair retained legacy finding\n")
    repaired = prepare(repository)
    pair(repository, repaired)
    record(
        repository,
        repaired,
        report(
            repaired,
            dispositions=[
                {
                    "id": "legacy",
                    "status": "fixed",
                    "evidence": "Verified changed consumer",
                }
            ],
        ),
    )
    push(repository)


def test_legacy_invalid_fixed_record_remains_open_and_recoverable(
    repository, monkeypatch
):
    monkeypatch.syspath_prepend(str(GATE.parent))
    import review_gate as gate

    repo, _, env = repository
    previous = prepare(repository)
    record(
        repository,
        previous,
        report(
            previous,
            "finder",
            verdict="findings",
            findings=[{"id": "legacy", "summary": "Defect", "evidence": "input.txt"}],
        ),
    )
    pair(repository, previous)
    invalid = report(
        previous,
        dispositions=[
            {
                "id": "legacy",
                "status": "fixed",
                "evidence": "Unchanged-head claim",
            }
        ],
    )
    store = gate.Store(repo)
    # The original checker accepted this report with a valid hash/manifest.
    # Preserve its bytes and prove the corrected checker can recover without
    # treating it as an effective disposition or deleting the journal.
    with store.lock():
        manifest = store.candidate_dir(previous) / "reports.json"
        names = gate.read_json(manifest)["reports"]
        path = (
            manifest.parent
            / "reports"
            / f"{len(names) + 1:06d}-{gate.digest(invalid)}.json"
        )
        gate.write_json_files(
            store.path,
            [
                (path, invalid),
                (manifest, {"reports": [*names, path.name]}),
            ],
        )
    install(repository)
    assert "unresolved findings: legacy" in push(repository, success=False).stderr
    commit(repo, env, "actual repair\n")
    candidate = prepare(repository)
    pair(repository, candidate)
    record(
        repository,
        candidate,
        report(
            candidate,
            dispositions=[
                {
                    "id": "legacy",
                    "status": "fixed",
                    "evidence": "Verified changed consumer",
                }
            ],
        ),
    )
    push(repository)
    assert gate.read_json(path) == invalid


def test_invalid_transaction_cannot_publish_an_earlier_valid_path(
    repository, monkeypatch
):
    monkeypatch.syspath_prepend(str(GATE.parent))
    import review_gate as gate

    repo, _, _ = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    store = gate.Store(repo)
    index = next((store.path / "targets").glob("*.json"))
    updates = [
        {
            "path": index.relative_to(store.path).as_posix(),
            "value": {"candidate": "f" * 64},
        },
        {"path": "../outside.json", "value": {"overwrite": True}},
    ]
    (store.path / "pending.json").write_text(
        json.dumps(
            {
                "updates": updates,
                "digest": gate.digest(updates),
            }
        )
    )
    original = evidence_bytes(store.path)
    with pytest.raises(gate.GateError, match="invalid pending evidence path"):
        with store.lock():
            pytest.fail("Invalid recovery should block access to the store")
    assert evidence_bytes(store.path) == original
    assert not (store.path.parent / "outside.json").exists()


@pytest.mark.parametrize("operation", ["prepare", "record"])
@pytest.mark.parametrize("stage", ["first", "last", "cleanup"])
def test_interrupted_evidence_update_recovers_without_discarding_history(
    repository, monkeypatch, operation, stage
):
    monkeypatch.syspath_prepend(str(GATE.parent))
    import review_gate as gate

    repo, _, env = repository
    previous = prepare(repository)
    pair(repository, previous)
    store = gate.Store(repo)
    original_reports = {
        path.name: path.read_bytes()
        for path in (store.candidate_dir(previous) / "reports").glob("*.json")
    }
    source = repo.parent / "next-review.json"
    source.write_text(
        json.dumps(report(previous, "another-reviewer")), encoding="utf-8"
    )
    if operation == "prepare":
        commit(repo, env, "next candidate\n")
    args = argparse.Namespace(
        base="origin/main",
        head="HEAD",
        remote="origin",
        target="refs/heads/topic",
        author_session=["author"],
    )
    error = KeyboardInterrupt if stage == "last" else OSError
    publish = gate.atomic_write
    unlink = Path.unlink

    def interrupted_write(path, contents):
        first = (
            path.name == "candidate.json"
            if operation == "prepare"
            else path.parent.name == "reports"
        )
        last = (
            path.parent.name == "targets"
            if operation == "prepare"
            else path.name == "reports.json"
        )
        if (stage == "first" and first) or (stage == "last" and last):
            raise error("injected publication interruption")
        return publish(path, contents)

    def interrupted_cleanup(path, *args, **kwargs):
        if stage == "cleanup" and path == store.path / "pending.json":
            raise error("injected cleanup interruption")
        return unlink(path, *args, **kwargs)

    with monkeypatch.context() as fault:
        fault.setattr(gate, "atomic_write", interrupted_write)
        fault.setattr(Path, "unlink", interrupted_cleanup)
        with pytest.raises(error, match="injected"):
            with store.lock():
                if operation == "prepare":
                    store.prepare(args)
                else:
                    store.record(previous, source)
    assert (store.path / "pending.json").is_file()
    recovered = gate.Store(repo)
    with recovered.lock():
        assert not (store.path / "pending.json").exists()
        if operation == "prepare":
            candidate = recovered.prepare(args)
            assert recovered.candidate(candidate)["previous"] == previous
        else:
            candidate = previous
            assert len(recovered.reports(candidate)) == 3
    for name, contents in original_reports.items():
        assert (
            store.candidate_dir(previous) / "reports" / name
        ).read_bytes() == contents
    if operation == "prepare":
        pair(repository, candidate)
    install(repository)
    push(repository)


def test_later_incomplete_review_does_not_leave_an_old_clean_pass(repository):
    candidate = prepare(repository)
    pair(repository, candidate)
    record(
        repository,
        candidate,
        report(
            candidate,
            "reviewer-b",
            "contracts",
            status="incomplete",
            verdict="findings",
            coverage_complete=False,
        ),
    )
    install(repository)
    push(repository, success=False)


def test_non_substantive_initial_and_baseline_update_exceptions(repository):
    repo, _, env = repository
    previous = prepare(repository)
    record(
        repository,
        previous,
        report(
            previous,
            scope="non-substantive",
            no_behavior_change=True,
            reason="Reviewed full diff; text-only spelling correction",
        ),
    )
    install(repository)
    push(repository)
    commit(repo, env, "spelling corrected\n")
    candidate = prepare(repository)
    record(
        repository,
        candidate,
        report(
            candidate,
            scope="non-substantive",
            no_behavior_change=True,
            reason="Reviewed delta to the clean baseline; no behavior change",
            baseline=previous,
        ),
    )
    assert "non-substantive assessment" in push(repository).stderr


def test_exception_cannot_skip_a_failed_baseline(repository):
    repo, _, env = repository
    previous = prepare(repository)
    commit(repo, env, "next change\n")
    candidate = prepare(repository)
    result = record(
        repository,
        candidate,
        report(
            candidate,
            scope="non-substantive",
            no_behavior_change=True,
            reason="Claimed formatting-only delta",
            baseline=previous,
        ),
        success=False,
    )
    assert "baseline lacks clean review" in result.stderr
    install(repository)
    push(repository, success=False)


def test_non_head_push_uses_outgoing_commit_even_with_dirty_worktree(repository):
    repo, _, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    install(repository)
    git(repo, env, "switch", "main")
    (repo / "input.txt").write_text("unrelated unstaged work\n", encoding="utf-8")
    push(repository, "topic:topic")
    assert (repo / "input.txt").read_text() == "unrelated unstaged work\n"


def test_multi_ref_failure_prevents_every_update(repository):
    repo, remote, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    git(repo, env, "branch", "unreviewed")
    install(repository)
    push(repository, "topic", "unreviewed", success=False)
    assert run(
        repo,
        env,
        "git",
        "--git-dir",
        str(remote),
        "show-ref",
        "--verify",
        "refs/heads/topic",
    ).returncode


def test_tags_and_deletions_do_not_require_pr_review(repository):
    repo, _, env = repository
    install(repository)
    git(repo, env, "tag", "fixture-tag")
    push(repository, "refs/tags/fixture-tag")
    push(repository, ":refs/tags/fixture-tag")


def test_foreign_hook_receives_original_args_and_stdin_and_failure_propagates(
    repository,
):
    repo, remote, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    hook = repo / ".git" / "hooks" / "pre-push"
    hook.write_text(
        '#!/bin/sh\nprintf "%s\\n" "$@" > "$DART_TEST_ARGS"\ncat > "$DART_TEST_INPUT"\nexit 7\n',
        encoding="utf-8",
        newline="\n",
    )
    hook.chmod(0o755)
    env["DART_TEST_ARGS"] = str(repo.parent / "arguments.txt")
    env["DART_TEST_INPUT"] = str(repo.parent / "input.txt")
    install(repository)
    install(repository)
    assert (hook.parent / "pre-push.local").read_bytes().startswith(b"#!/bin/sh\n")
    result = push(repository, success=False)
    assert "two independent local reviews" in result.stderr
    assert Path(env["DART_TEST_ARGS"]).read_text().splitlines() == [
        "origin",
        str(remote),
    ]
    fields = Path(env["DART_TEST_INPUT"]).read_text().split()
    assert fields[:3] == [
        "refs/heads/topic",
        git(repo, env, "rev-parse", "topic"),
        "refs/heads/topic",
    ]


def test_installed_checker_works_from_older_linked_worktree(repository):
    repo, remote, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    install(repository)
    linked = repo.parent / "older worktree"
    git(repo, env, "worktree", "add", "--detach", str(linked), "main")
    assert not (linked / "scripts" / "review_gate.py").exists()
    push((linked, remote, env), "topic:topic")


@pytest.mark.parametrize("custom_manager", [False, True])
def test_older_worktree_can_prepare_record_check_and_push(repository, custom_manager):
    repo, remote, env = repository
    if custom_manager:
        runtime, manager = install_custom_manager(repository)
    else:
        install(repository)
        runtime = repo / ".git" / "hooks" / "dart-review-gate.py"
    linked = repo.parent / "old source checkout"
    git(repo, env, "worktree", "add", "-b", "legacy-topic", str(linked), "main")
    assert not (linked / "scripts" / "review_gate.py").exists()
    assert not (linked / "pixi.toml").exists()
    commit(linked, env, "new change authored in the old checkout\n")
    old_repository = (linked, remote, env)
    push(old_repository, "legacy-topic", success=False)
    prepared = run(
        linked,
        env,
        sys.executable,
        str(runtime),
        "prepare",
        "--base",
        "origin/main",
        "--remote",
        "origin",
        "--target",
        "refs/heads/legacy-topic",
        "--author-session",
        "author",
    )
    assert prepared.returncode == 0, prepared.stderr
    candidate = prepared.stdout.strip()
    source = repo.parent / "legacy-review.json"
    for scope, reviewer in (("correctness", "reviewer-a"), ("contracts", "reviewer-b")):
        source.write_text(
            json.dumps(report(candidate, reviewer, scope)), encoding="utf-8"
        )
        result = run(
            linked, env, sys.executable, str(runtime), "record", candidate, str(source)
        )
        assert result.returncode == 0, result.stderr
    checked = run(linked, env, sys.executable, str(runtime), "check", candidate)
    assert checked.returncode == 0, checked.stderr
    push(old_repository, "legacy-topic")
    if custom_manager:
        assert git(linked, env, "config", "core.hooksPath") == str(manager)


@pytest.mark.parametrize(
    "damage", ["checker", "empty-checker", "replaced-checker", "interpreter"]
)
@pytest.mark.parametrize("custom_manager", [False, True])
def test_missing_runtime_fails_closed(repository, damage, custom_manager):
    repo, _, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    if custom_manager:
        runtime, _ = install_custom_manager(repository)
    else:
        install(repository)
        runtime = repo / ".git" / "hooks" / "dart-review-gate.py"
    if damage == "checker":
        runtime.unlink()
    elif damage in ("empty-checker", "replaced-checker"):
        runtime.write_bytes(
            b"" if damage == "empty-checker" else b"raise SystemExit(0)\n"
        )
    else:
        env["DART_HOOK_PYTHON"] = "/unavailable/python"
    assert "BLOCKED" in push(repository, success=False).stderr
    if custom_manager and damage != "interpreter":
        restored = run(repo, env, sys.executable, str(INSTALLER), "--custom-manager")
        assert restored.returncode == 0, restored.stderr
        push(repository)


def test_ambient_python_and_commit_bypass_flags_cannot_disable_push_gate(repository):
    repo, _, env = repository
    install(repository)
    custom = repo.parent / "python-customization"
    custom.mkdir()
    (custom / "sitecustomize.py").write_text("import os\nos._exit(0)\n")
    env.update(
        {
            "PYTHONPATH": str(custom),
            "PYTHONINSPECT": "1",
            "DART_SKIP_HOOKS": "1",
            "DART_HOOK_DRY_RUN": "1",
        }
    )
    assert "BLOCKED" in push(repository, success=False).stderr


def test_foreign_hook_collision_does_not_partly_replace_commit_hook(repository):
    repo, _, env = repository
    hooks = repo / ".git" / "hooks"
    for name in ("pre-commit", "pre-push", "pre-push.local"):
        (hooks / name).write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    before = (hooks / "pre-commit").read_bytes()
    result = run(repo, env, sys.executable, str(INSTALLER))
    assert result.returncode and "cannot be backed up" in result.stderr
    assert (hooks / "pre-commit").read_bytes() == before


def test_custom_hook_manager_is_preserved(repository):
    repo, _, env = repository
    managed = repo.parent / "custom-hooks"
    managed.mkdir()
    git(repo, env, "config", "core.hooksPath", str(managed))
    result = run(repo, env, sys.executable, str(INSTALLER))
    assert result.returncode and "core.hooksPath is set" in result.stderr
    assert not list(managed.iterdir())
    existing = managed / "pre-push"
    existing.write_bytes(b"#!/bin/sh\n# manager owns this handler\nexit 29\n")
    existing.chmod(0o755)
    hooks = repo / ".git" / "hooks"
    original = {path.name: path.read_bytes() for path in hooks.iterdir()}
    result = run(repo, env, sys.executable, str(INSTALLER), "--custom-manager")
    assert result.returncode == 0, result.stderr
    assert git(repo, env, "config", "core.hooksPath") == str(managed)
    assert existing.read_bytes().endswith(b"exit 29\n")
    assert list(managed.iterdir()) == [existing]
    assert {path.name: path.read_bytes() for path in hooks.iterdir()} == original
    exported = repo / ".git"
    assert (exported / "dart-review-pre-push").is_file()
    assert not (exported / "pre-commit").exists()


@pytest.mark.parametrize(
    "alias", ["configured", "relative-configured", "external", "default"]
)
def test_custom_export_preserves_aliased_hook_directories(repository, alias):
    repo, _, env = repository
    common = repo / ".git"
    legacy = common / "dart-review-runtime"
    manager = legacy if "configured" in alias else repo.parent / "shared manager"
    manager.mkdir()
    handler = b"#!/bin/sh\necho original-manager >&2\nexit 0\n"
    (manager / "pre-push").write_bytes(handler)
    (manager / "pre-push").chmod(0o755)
    default = common / "hooks"
    if alias == "external":
        directory_alias(legacy, manager)
    elif alias == "default":
        (default / "pre-push").write_bytes(handler)
        (default / "pre-push").chmod(0o755)
        directory_alias(legacy, default)
    configured = (
        ".git/dart-review-runtime" if alias == "relative-configured" else str(manager)
    )
    git(repo, env, "config", "core.hooksPath", configured)
    before_manager = {p.name: p.read_bytes() for p in manager.iterdir()}
    before_default = {p.name: p.read_bytes() for p in default.iterdir()}
    push(repository)
    commit(repo, env, "successor still governed by the original manager\n")
    nested = repo / "nested directory"
    nested.mkdir()
    run(nested, env, sys.executable, str(INSTALLER), "--custom-manager")
    assert {p.name: p.read_bytes() for p in manager.iterdir()} == before_manager
    assert {p.name: p.read_bytes() for p in default.iterdir()} == before_default
    assert git(repo, env, "config", "core.hooksPath") == configured
    assert "original-manager" in push(repository).stderr


def test_custom_export_is_not_redirected_by_legacy_directory_retargeting(
    repository, monkeypatch
):
    monkeypatch.syspath_prepend(str(INSTALLER.parent))
    import install_git_hooks as installer

    repo, _, env = repository
    legacy = repo / ".git" / "dart-review-runtime"
    legacy.mkdir()
    manager = repo.parent / "shared manager"
    manager.mkdir()
    original = b"#!/bin/sh\n# shared owner\nexit 0\n"
    (manager / "pre-push").write_bytes(original)
    (manager / "pre-push").chmod(0o755)
    git(repo, env, "config", "core.hooksPath", str(manager))
    publish = installer.publish_file
    retargeted = []

    def retarget(path, content, executable=False):
        if not retargeted:
            legacy.rename(legacy.with_name("runtime-before-move"))
            directory_alias(legacy, manager)
            retargeted.append(True)
        return publish(path, content, executable)

    monkeypatch.chdir(repo)
    monkeypatch.setattr(installer, "publish_file", retarget)
    assert installer.main(["--custom-manager"]) == 0
    assert retargeted
    assert list(manager.iterdir()) == [manager / "pre-push"]
    assert (manager / "pre-push").read_bytes() == original
    push(repository)


@pytest.mark.parametrize("refresh", [False, True])
@pytest.mark.parametrize("name", ["dart-review-gate.py", "dart-review-pre-push"])
@pytest.mark.parametrize(
    "phase, error", [("before", OSError), ("after", KeyboardInterrupt)]
)
def test_custom_export_interruption_blocks_unreviewed_push(
    repository, monkeypatch, refresh, name, phase, error
):
    monkeypatch.syspath_prepend(str(INSTALLER.parent))
    import install_git_hooks as installer

    repo, _, env = repository
    runtime, manager = install_custom_manager(repository, export=refresh)
    manager_bytes = (manager / "pre-push").read_bytes()
    push(repository, success=False)
    source = repo.parent / "new export source"
    source.mkdir()
    checker = GATE.read_bytes() + b"\n# refreshed fixture version\n"
    (source / "review_gate.py").write_bytes(checker)
    monkeypatch.setattr(installer, "__file__", str(source / "install_git_hooks.py"))
    monkeypatch.chdir(repo)
    replace = installer.os.replace
    interruptions = []

    def interrupted(temporary, destination):
        if destination.name == name:
            # Readers still enforce the old version or block missing/mismatched
            # files. The new file is complete before becoming visible.
            assert Path(temporary).read_bytes() == (
                checker
                if name == "dart-review-gate.py"
                else installer.pre_push_hook(checker, chain_local=False).encode("utf-8")
            )
            push(repository, success=False)
            if phase == "after":
                replace(temporary, destination)
            interruptions.append(destination)
            raise error("injected custom export interruption")
        return replace(temporary, destination)

    monkeypatch.setattr(installer.os, "replace", interrupted)
    with pytest.raises(error, match="injected custom export interruption"):
        installer.main(["--custom-manager"])
    assert interruptions == [runtime.parent / name]
    assert not (runtime.parent / ".dart-review-export-lock").exists()
    assert (manager / "pre-push").read_bytes() == manager_bytes
    assert git(repo, env, "config", "core.hooksPath") == str(manager)
    push(repository, success=False)
    monkeypatch.setattr(installer.os, "replace", replace)
    (source / "review_gate.py").write_bytes(checker + b"# next installer revision\n")
    assert installer.main(["--custom-manager"]) == 0
    push(repository, success=False)
    candidate = prepare(repository)
    pair(repository, candidate)
    push(repository)


def test_custom_export_does_not_chain_an_incidental_common_hook(repository):
    repo, _, _ = repository
    install_custom_manager(repository)
    incidental = repo / ".git" / "pre-push.local"
    original = b"#!/bin/sh\necho unrelated-common-hook >&2\nexit 37\n"
    incidental.write_bytes(original)
    incidental.chmod(0o755)
    candidate = prepare(repository)
    pair(repository, candidate)
    assert "unrelated-common-hook" not in push(repository).stderr
    assert incidental.read_bytes() == original


@pytest.mark.parametrize("name", ["dart-review-gate.py", "dart-review-pre-push"])
@pytest.mark.parametrize("kind", ["file", "directory", "hardlink"])
def test_custom_export_preserves_foreign_output_leaves(repository, name, kind):
    repo, _, env = repository
    runtime, manager = install_custom_manager(repository, export=False)
    destination = runtime.parent / name
    original = b"foreign output owner\n"
    if kind == "directory":
        destination.mkdir()
        (destination / "owned.txt").write_bytes(original)
    elif kind == "hardlink":
        external = repo.parent / "shared output.txt"
        external.write_bytes(original)
        os.link(external, destination)
    else:
        destination.write_bytes(original)
    result = run(repo, env, sys.executable, str(INSTALLER), "--custom-manager")
    assert result.returncode and "refusing" in result.stderr
    if kind == "directory":
        assert (destination / "owned.txt").read_bytes() == original
    else:
        assert destination.read_bytes() == original
    if kind == "hardlink":
        assert external.read_bytes() == original
        assert os.path.samefile(external, destination)
    for other in {"dart-review-gate.py", "dart-review-pre-push"} - {name}:
        assert not (runtime.parent / other).exists()
    assert git(repo, env, "config", "core.hooksPath") == str(manager)
    assert not (runtime.parent / ".dart-review-export-lock").exists()


@pytest.mark.parametrize(
    "alias", ["configured", "relative", "configured-alias", "default-alias"]
)
def test_custom_export_refuses_common_directory_hook_ownership(repository, alias):
    repo, _, env = repository
    common = repo / ".git"
    original = b"#!/bin/sh\n# common hook owner\nexit 29\n"
    (common / "pre-push").write_bytes(original)
    if alias == "default-alias":
        hooks = common / "hooks"
        hooks.rename(common / "original-hooks")
        directory_alias(hooks, common)
        manager = repo.parent / "separate manager"
        manager.mkdir()
        configured = str(manager)
    elif alias == "configured-alias":
        manager = repo.parent / "manager alias"
        directory_alias(manager, common)
        configured = str(manager)
    else:
        configured = ".git" if alias == "relative" else str(common)
    git(repo, env, "config", "core.hooksPath", configured)
    nested = repo / "nested directory"
    nested.mkdir()
    result = run(nested, env, sys.executable, str(INSTALLER), "--custom-manager")
    assert result.returncode and "hooks directory" in result.stderr
    assert (common / "pre-push").read_bytes() == original
    assert not (common / "dart-review-pre-push").exists()
    assert not (common / "dart-review-gate.py").exists()
    assert not (common / ".dart-review-export-lock").exists()
    assert git(repo, env, "config", "core.hooksPath") == configured


@pytest.mark.parametrize("damage", ["missing", "empty", "rewound"])
def test_target_index_damage_cannot_discard_findings(repository, damage):
    repo, _, env = repository
    previous = prepare(repository)
    commit(repo, env, "next revision\n")
    candidate = prepare(repository)
    record(
        repository,
        candidate,
        report(
            candidate,
            verdict="findings",
            findings=[
                {
                    "id": "open",
                    "summary": "Open defect",
                    "evidence": "Consumer still fails",
                }
            ],
        ),
    )
    index = next((repo / ".git" / "dart-review" / "targets").glob("*.json"))
    if damage == "missing":
        index.unlink()
    else:
        index.write_text(
            json.dumps({"candidate": "" if damage == "empty" else previous})
        )
    commit(repo, env, "another revision\n")
    result = run(
        repo,
        env,
        sys.executable,
        str(GATE),
        "prepare",
        "--base",
        "origin/main",
        "--remote",
        "origin",
        "--target",
        "refs/heads/topic",
        "--author-session",
        "author",
    )
    assert result.returncode and "BLOCKED" in result.stderr
    install(repository)
    push(repository, success=False)


@pytest.mark.parametrize("destination", ["branch", "url"])
def test_target_index_cannot_authorize_another_destination(repository, destination):
    repo, remote, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    indexes = repo / ".git" / "dart-review" / "targets"
    original = next(indexes.glob("*.json"))
    if destination == "branch":
        prepare(repository, "another")
        refs = ("topic:another",)
    else:
        second = remote.with_name("another.git")
        git(repo, env, "init", "-q", "--bare", str(second))
        git(repo, env, "remote", "set-url", "--push", "origin", str(second))
        prepare(repository)
        refs = ("topic",)
    other = next(path for path in indexes.glob("*.json") if path != original)
    other.write_bytes(original.read_bytes())
    install(repository)
    assert "mismatched" in push(repository, *refs, success=False).stderr


@pytest.mark.parametrize("trivial_descendant", [False, True])
def test_late_ancestor_findings_block_active_descendant(repository, trivial_descendant):
    repo, _, env = repository
    previous = prepare(repository)
    pair(repository, previous)
    commit(repo, env, "later revision\n")
    candidate = prepare(repository)
    if trivial_descendant:
        record(
            repository,
            candidate,
            report(
                candidate,
                scope="non-substantive",
                no_behavior_change=True,
                reason="Reviewed the spelling-only delta to the passed baseline",
                baseline=previous,
            ),
        )
    else:
        pair(repository, candidate)
    record(
        repository,
        previous,
        report(
            previous,
            "late-reviewer",
            verdict="findings",
            findings=[
                {
                    "id": "late",
                    "summary": "Late defect",
                    "evidence": "An affected consumer fails",
                }
            ],
        ),
    )
    install(repository)
    assert "unresolved findings: late" in push(repository, success=False).stderr
    record(
        repository,
        candidate,
        report(
            candidate,
            dispositions=[
                {
                    "id": "late",
                    "status": "rejected",
                    "evidence": "The active candidate's consumer proves the claim false",
                }
            ],
        ),
    )
    if trivial_descendant:
        record(repository, candidate, report(candidate, "reviewer-b", "contracts"))
    push(repository)


def test_unknown_disposition_is_rejected_without_poisoning_history(repository):
    candidate = prepare(repository)
    invalid = report(
        candidate,
        dispositions=[
            {
                "id": "unknown",
                "status": "fixed",
                "evidence": "Unmatched disposition",
            }
        ],
    )
    assert (
        "unknown finding"
        in record(repository, candidate, invalid, success=False).stderr
    )
    pair(repository, candidate)
    install(repository)
    push(repository)


@pytest.mark.parametrize("name", ["dart-review-gate.py", "pre-push"])
def test_reinstallation_keeps_enforcing_while_replacement_is_written(
    repository, monkeypatch, name
):
    monkeypatch.syspath_prepend(str(INSTALLER.parent))
    import install_git_hooks as installer

    repo, _, _ = repository
    install(repository)
    destination = repo / ".git" / "hooks" / name
    before = destination.read_bytes()
    replace = installer.os.replace
    calls = []

    def paused(source, target):
        calls.append(target)
        assert Path(source).read_bytes() == before
        assert destination.read_bytes() == before
        assert "BLOCKED" in push(repository, success=False).stderr
        return replace(source, target)

    monkeypatch.setattr(installer.os, "replace", paused)
    installer.publish_file(destination, before, executable=name == "pre-push")
    assert calls == [destination]
    push(repository, success=False)


def test_concurrent_installers_preserve_the_original_foreign_hook(
    repository, monkeypatch
):
    monkeypatch.syspath_prepend(str(INSTALLER.parent))
    import install_git_hooks as installer

    repo, _, env = repository
    hooks = repo / ".git" / "hooks"
    original = b"#!/bin/sh\n# foreign owner\nexit 0\n"
    (hooks / "pre-push").write_bytes(original)
    probe = installer.foreign_hook
    attempts = []

    def paused(path, sentinel):
        if not attempts:
            result = run(repo, env, sys.executable, str(INSTALLER))
            attempts.append(result)
            assert result.returncode and "another installer" in result.stderr
        return probe(path, sentinel)

    monkeypatch.setattr(installer, "resolve_hooks_dir", lambda: hooks)
    monkeypatch.setattr(installer, "foreign_hook", paused)
    assert installer.main([]) == 0
    assert attempts
    assert (hooks / "pre-push.local").read_bytes() == original


def test_dangling_foreign_backup_is_preserved(repository):
    repo, _, env = repository
    hooks = repo / ".git" / "hooks"
    hook = hooks / "pre-push"
    original = b"#!/bin/sh\nexit 0\n"
    hook.write_bytes(original)
    backup = hooks / "pre-push.local"
    try:
        backup.symlink_to("missing-foreign-hook")
    except OSError as exc:
        pytest.skip(f"Creating a symlink requires platform privileges: {exc}")
    result = run(repo, env, sys.executable, str(INSTALLER))
    assert result.returncode and "cannot be backed up" in result.stderr
    assert backup.is_symlink() and os.readlink(backup) == "missing-foreign-hook"
    assert hook.read_bytes() == original


def test_doctor_detects_missing_stale_and_disabled_review_installation(
    repository, monkeypatch
):
    monkeypatch.syspath_prepend(str(INSTALLER.parent))
    import review_gate as gate

    repo, _, _ = repository
    assert not gate.hook_inventory(repo)["installed"]
    install(repository)
    source = repo / "scripts" / "review_gate.py"
    source.parent.mkdir()
    source.write_bytes(GATE.read_bytes())
    assert gate.hook_inventory(repo)["installed"]
    assert gate.hook_inventory(repo)["checker_current"]
    source.write_bytes(GATE.read_bytes() + b"\n# source changed\n")
    assert not gate.hook_inventory(repo)["checker_current"]
    (repo / ".git" / "hooks" / "pre-push").write_bytes(b"#!/bin/sh\nexit 0\n")
    assert not gate.hook_inventory(repo)["installed"]


def test_installed_runtime_retains_python_311_syntax():
    # Formatting under the repository's newer Python must not break older
    # linked worktrees using the hook's documented minimum interpreter.
    for source in (GATE, INSTALLER):
        ast.parse(source.read_text(encoding="utf-8"), feature_version=(3, 11))


def test_withdrawn_ancestor_approval_revokes_a_trivial_exception(repository):
    repo, _, env = repository
    previous = prepare(repository)
    pair(repository, previous)
    commit(repo, env, "spelling update\n")
    candidate = prepare(repository)
    record(
        repository,
        candidate,
        report(
            candidate,
            "exception-reviewer",
            "non-substantive",
            no_behavior_change=True,
            reason="Only spelling changed from the reviewed baseline",
            baseline=previous,
        ),
    )
    record(
        repository,
        previous,
        report(previous, status="incomplete", coverage_complete=False),
    )
    install(repository)
    assert "baseline lacks clean review" in push(repository, success=False).stderr
    pair(repository, candidate)
    push(repository)


def test_added_author_does_not_poison_history_with_a_trivial_exception(repository):
    repo, _, env = repository
    previous = prepare(repository)
    pair(repository, previous)
    commit(repo, env, "spelling update\n")
    interim = prepare(repository)
    record(
        repository,
        interim,
        report(
            interim,
            "exception-reviewer",
            "non-substantive",
            no_behavior_change=True,
            reason="Only spelling changed from the reviewed baseline",
            baseline=previous,
        ),
    )
    candidate = prepare(repository, authors=("author", "reviewer-a"))
    install(repository)
    push(repository, success=False)
    record(repository, candidate, report(candidate, "reviewer-c"))
    record(repository, candidate, report(candidate, "reviewer-d", "contracts"))
    push(repository)


def test_stable_finding_accepts_updated_evidence_without_losing_history(repository):
    repo, _, env = repository
    previous = prepare(repository)
    finding = {
        "id": "consumer-contract",
        "summary": "Consumer still reads the excluded input",
        "evidence": "consumer.py:10 reads the input",
    }
    record(
        repository,
        previous,
        report(previous, verdict="findings", findings=[finding]),
    )
    history = repo / ".git" / "dart-review" / "candidates" / previous / "reports"
    original = {path.name: path.read_bytes() for path in history.glob("*.json")}
    commit(repo, env, "repair changed source locations\n")
    candidate = prepare(repository)
    updated = {
        **finding,
        "summary": "Consumer reads the input after the partial repair",
        "evidence": "consumer.py:17 still reads the input",
    }
    record(
        repository,
        candidate,
        report(candidate, verdict="findings", findings=[updated]),
    )
    assert {path.name: path.read_bytes() for path in history.glob("*.json")} == original
    pair(repository, candidate)
    install(repository)
    assert (
        "unresolved findings: consumer-contract"
        in push(repository, success=False).stderr
    )
    record(
        repository,
        candidate,
        report(
            candidate,
            dispositions=[
                {
                    "id": finding["id"],
                    "status": "rejected",
                    "evidence": "The current consumer contract is exercised by an equivalent retained check",
                }
            ],
        ),
    )
    push(repository)


def test_duplicate_finding_ids_in_one_report_are_rejected(repository):
    candidate = prepare(repository)
    finding = {"id": "duplicate", "summary": "First issue", "evidence": "first.py:1"}
    result = record(
        repository,
        candidate,
        report(
            candidate,
            verdict="findings",
            findings=[finding, {**finding, "summary": "Another issue"}],
        ),
        success=False,
    )
    assert "finding IDs must be unique" in result.stderr
    pair(repository, candidate)
    install(repository)
    push(repository)


@pytest.mark.parametrize("name", ["pre-commit", "pre-push"])
def test_failed_foreign_hook_adoption_keeps_the_original_active(
    repository, monkeypatch, name
):
    monkeypatch.syspath_prepend(str(INSTALLER.parent))
    import install_git_hooks as installer

    repo, _, env = repository
    hooks = repo / ".git" / "hooks"
    hook = hooks / name
    original = b"#!/bin/sh\necho kept-foreign-hook >&2\nexit 29\n"
    hook.write_bytes(original)
    hook.chmod(0o755)
    publish = installer.write_hook

    def fail_publication(path, template=installer.HOOK_TEMPLATE):
        if path.name == name:
            raise OSError("injected replacement failure")
        return publish(path, template)

    monkeypatch.setattr(installer, "resolve_hooks_dir", lambda: hooks)
    monkeypatch.setattr(installer, "write_hook", fail_publication)
    with pytest.raises(OSError, match="injected replacement failure"):
        installer.main([])
    assert hook.read_bytes() == original
    assert os.access(hook, os.X_OK)
    assert (hooks / (name + ".local")).read_bytes() == original
    assert not (hooks / ".dart-install-lock").exists()
    args = (
        ("push", "origin", "topic")
        if name == "pre-push"
        else ("commit", "--allow-empty", "-qm", "Blocked fixture commit")
    )
    result = run(repo, env, "git", *args)
    assert result.returncode and "kept-foreign-hook" in result.stderr


def test_backup_created_after_preflight_is_not_overwritten(repository, monkeypatch):
    monkeypatch.syspath_prepend(str(INSTALLER.parent))
    import install_git_hooks as installer

    repo, _, _ = repository
    hooks = repo / ".git" / "hooks"
    hook = hooks / "pre-push"
    original = b"#!/bin/sh\nexit 29\n"
    hook.write_bytes(original)
    preserve = installer.preserve_hook

    def intervening_backup(path, local):
        local.write_bytes(b"another owner's backup\n")
        return preserve(path, local)

    monkeypatch.setattr(installer, "preserve_hook", intervening_backup)
    monkeypatch.setattr(installer, "resolve_hooks_dir", lambda: hooks)
    with pytest.raises(FileExistsError):
        installer.main([])
    assert hook.read_bytes() == original
    assert (hooks / "pre-push.local").read_bytes() == b"another owner's backup\n"


def evidence_bytes(directory):
    return {
        path.relative_to(directory): path.read_bytes()
        for path in directory.rglob("*.json")
    }


@pytest.mark.parametrize("expansion", ["unicode", "indentation"])
def test_expanding_report_is_rejected_without_changing_evidence(repository, expansion):
    repo, _, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    directory = repo / ".git" / "dart-review"
    before = evidence_bytes(directory)
    value = report(candidate)
    if expansion == "unicode":
        value["report"] = "é" * 400_000
    else:
        value["coverage"] = ["x"] * 260_000
    raw = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    assert len(raw) < 2 * 1024 * 1024
    source = repo.parent / "expanded-review.json"
    source.write_bytes(raw)
    result = run(repo, env, sys.executable, str(GATE), "record", candidate, str(source))
    assert result.returncode and "oversized serialized evidence" in result.stderr
    assert evidence_bytes(directory) == before
    install(repository)
    push(repository)


def test_oversized_candidate_leaves_existing_history_readable(repository, monkeypatch):
    monkeypatch.syspath_prepend(str(GATE.parent))
    import review_gate as gate

    repo, _, _ = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    store = gate.Store(repo)
    before = evidence_bytes(store.path)
    # Scale the same producer/reader boundary without exceeding OS argv limits.
    monkeypatch.setattr(gate, "MAX_BYTES", 2_000)
    args = argparse.Namespace(
        base="origin/main",
        head="HEAD",
        remote="origin",
        target="refs/heads/topic",
        author_session=["é" * 400],
    )
    with store.lock():
        with pytest.raises(gate.GateError, match="oversized serialized evidence"):
            store.prepare(args)
        assert evidence_bytes(store.path) == before
        assert store.assess(candidate) == "two independent local reviews"
    install(repository)
    push(repository)


def test_full_manifest_rejects_append_before_publishing_report(repository, monkeypatch):
    monkeypatch.syspath_prepend(str(GATE.parent))
    import review_gate as gate

    repo, _, _ = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    store = gate.Store(repo)
    source = repo.parent / "journal-review.json"
    # Reach the real journal boundary with a bounded number of actual records.
    monkeypatch.setattr(gate, "MAX_BYTES", 2_000)
    with store.lock():
        for index in range(50):
            before = evidence_bytes(store.path)
            source.write_text(
                json.dumps(report(candidate, summary=f"Completed review {index}")),
                encoding="utf-8",
            )
            try:
                store.record(candidate, source)
            except gate.GateError as error:
                assert "oversized serialized evidence" in str(error)
                assert "reports.json" in str(error)
                assert evidence_bytes(store.path) == before
                break
        else:
            pytest.fail("The bounded manifest never reached its capacity")
        assert store.assess(candidate) == "two independent local reviews"
        assert all(
            b"\r\n" not in contents for contents in evidence_bytes(store.path).values()
        )
    install(repository)
    push(repository)
