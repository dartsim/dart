"""Review evidence and real Git pre-push behavior, including Git for Windows.

All remotes and worktrees are disposable fixtures. No network, models, or user
Git configuration are involved. Unlike the older commit-guard suite, these
tests deliberately run on native Windows as well as POSIX.
"""

import ast
import json
import os
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


def prepare(repository, target="topic"):
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
        "--author-session",
        "author",
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


@pytest.mark.parametrize(
    "damage", ["checker", "empty-checker", "replaced-checker", "interpreter"]
)
def test_missing_runtime_fails_closed(repository, damage):
    repo, _, env = repository
    candidate = prepare(repository)
    pair(repository, candidate)
    install(repository)
    if damage == "checker":
        (repo / ".git" / "hooks" / "dart-review-gate.py").unlink()
    elif damage in ("empty-checker", "replaced-checker"):
        (repo / ".git" / "hooks" / "dart-review-gate.py").write_bytes(
            b"" if damage == "empty-checker" else b"raise SystemExit(0)\n"
        )
    else:
        env["DART_HOOK_PYTHON"] = "/unavailable/python"
    assert "BLOCKED" in push(repository, success=False).stderr


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


def test_late_ancestor_findings_block_active_descendant(repository):
    repo, _, env = repository
    previous = prepare(repository)
    pair(repository, previous)
    commit(repo, env, "later revision\n")
    candidate = prepare(repository)
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
    assert installer.main() == 0
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
    from review_gate import hook_inventory

    repo, _, _ = repository
    assert not hook_inventory(repo)["installed"]
    install(repository)
    source = repo / "scripts" / "review_gate.py"
    source.parent.mkdir()
    source.write_bytes(GATE.read_bytes())
    assert hook_inventory(repo)["installed"]
    assert hook_inventory(repo)["checker_current"]
    source.write_bytes(GATE.read_bytes() + b"\n# source changed\n")
    assert not hook_inventory(repo)["checker_current"]
    (repo / ".git" / "hooks" / "pre-push").write_bytes(b"#!/bin/sh\nexit 0\n")
    assert not hook_inventory(repo)["installed"]


def test_installed_runtime_retains_python_311_syntax():
    # Formatting under the repository's newer Python must not break older
    # linked worktrees using the hook's documented minimum interpreter.
    for source in (GATE, INSTALLER):
        ast.parse(source.read_text(encoding="utf-8"), feature_version=(3, 11))
