"""Publication-path policy, CLI modes, and staged hook regression tests."""

import importlib.util
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_local_paths.py"
SPEC = importlib.util.spec_from_file_location("check_local_paths", SCRIPT)
assert SPEC and SPEC.loader
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


@pytest.mark.parametrize(
    "path",
    (
        ".sisyphus/plans/private.md",
        "../.sisyphus/notes/private.md",
        "checkout/.ab/control/results.json",
        ".ab/results.json",
        "/tmp/claude-session/notes.md",
        ".claude/projects/session/notes.md",
        "scratchpad/notes.md",
        "checkout/scratchpad/notes.md",
        "task_2/scripts/probe.py",
        "checkout/task_12-fix-simd",
        "/home/example/worktree/file.md",
        "/home/example",
        "/Users/example",
        r"C:\Users\example",
        r"C:\Users\Example User",
        "C:/Users/example",
        "/root",
        "/root/file.md",
        r"/root\file.md",
        "/Users/example/worktree/file.md",
        r"C:\Users\example\worktree\file.md",
        r"C:\Users\Example User\worktree\file.md",
        r"C:\\Users\\example\\worktree\\file.md",
        "C:/Users/example/worktree/file.md",
        r"checkout\.ab\results.json",
        r"task_3\scripts\probe.py",
    ),
)
def test_private_paths_are_reported_with_line_and_match(path, capsys):
    assert checker.scan_text(f"Public summary\nSee `{path}`.\n")
    reported = capsys.readouterr().out.strip()
    assert reported.startswith("2: ")
    assert path.endswith(reported.removeprefix("2: "))


@pytest.mark.parametrize(
    "text",
    (
        "/tmp/out.png",
        "/tmp/dart-visual-evidence",
        "/tmp/dart-agent-visual-smoke/smoke.png",
        "docs/plans/public.md",
        ".claude/commands/dart-pr.md",
        "task_2 is a logical task identifier",
        "multitask_2/file.md",
        "named-scratchpad/file.md",
        "directory.ab/file.md",
        r"(?:/home/|/Users/|~/|fbsource|arvr/libraries)",
        "https://github.com/dartsim/dart/pull/1234",
        "https://api.github.com/users/dartsim/repos",
        "https://example.com/home/docs/index.html",
        "https://github.com/example/scratchpad/issues/1",
        "https://github.com/org/repo/blob/main/task_2/script.py",
        "HTTP://example.com/.ab/results.json",
        "https://example.com/.sisyphus/plans/private.md",
        "https://example.com/.claude/projects/session/notes.md",
        "https://example.com/tmp/claude-session/notes.md",
        "https://example.com/root/notes.md",
        "https://example.com/C:/Users/example/notes.md",
        "example.com/home/docs",
        "C:/home/example",
        "/rooted/file.md",
        "/Root 1 0 R",
    ),
)
def test_public_examples_and_non_path_identifiers_pass(text, capsys):
    assert not checker.scan_text(text)
    assert not capsys.readouterr().out


def test_reports_all_leaks_and_file_line(capsys):
    assert checker.scan_text(
        "summary\n/home/example/checkout and .sisyphus/plans/private.md\n",
        "notes.md",
    )
    assert capsys.readouterr().out.splitlines() == [
        "notes.md:2: .sisyphus/plans/private.md",
        "notes.md:2: /home/example/checkout",
    ]


def test_urls_do_not_hide_adjacent_paths_or_file_urls(capsys):
    assert checker.scan_text(
        "https://example.com/scratchpad/public.md /home/example\n"
        "[public](https://example.com/task_2/file.py),scratchpad/private.md\n"
        "file:///Users/example/private.md\n"
    )
    assert capsys.readouterr().out.splitlines() == [
        "1: /home/example",
        "2: scratchpad/private.md",
        "3: /Users/example/private.md",
    ]


def test_allowlist_is_limited_to_exact_file_and_line():
    assert not checker.scan_text(".sisyphus/\n", ".gitignore")
    assert checker.scan_text(".sisyphus/plans/private.md\n", ".gitignore")
    assert checker.scan_text(".sisyphus/\n", "nested/.gitignore")
    assert not checker.scan_text(
        "/home/example/fixture\n", "tests/test_check_local_paths.py"
    )
    assert checker.scan_text("/home/example/fixture\n", "tests/test_other.py")
    assert checker.scan_text(".sisyphus/\n")


def _cli(*args, cwd, text=None):
    return subprocess.run(
        [sys.executable, str(SCRIPT), *map(str, args)],
        cwd=cwd,
        input=text,
        capture_output=True,
        text=True,
    )


def _git(repo, *args):
    return subprocess.run(
        ["git", *args],
        cwd=repo,
        check=True,
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "GIT_CONFIG_GLOBAL": os.devnull,
            "GIT_CONFIG_SYSTEM": os.devnull,
        },
    )


@pytest.fixture
def repo(tmp_path):
    _git(tmp_path, "init", "-q")
    return tmp_path


def test_stdin_and_text_file_scan_title_body_without_executing_shell(tmp_path):
    text = "$(touch injected)\n`touch injected`\n.sisyphus/plans/private.md\n"
    title_body = tmp_path / "pr-body.md"
    title_body.write_text(text)
    for args, input_text in ((("--stdin",), text), (("--text-file", title_body), None)):
        result = _cli(*args, cwd=tmp_path, text=input_text)
        assert result.returncode == 1, result.stderr
        assert result.stdout == "3: .sisyphus/plans/private.md\n"
        assert not (tmp_path / "injected").exists()
    assert _cli("--stdin", cwd=tmp_path, text="Public summary\n").returncode == 0


def test_free_text_file_has_no_fixture_exemption(tmp_path):
    fixture = tmp_path / "tests" / "test_check_local_paths.py"
    fixture.parent.mkdir()
    fixture.write_text("/home/example/fixture\n")
    assert _cli("--text-file", fixture, cwd=tmp_path).returncode == 1


@pytest.mark.parametrize(
    "summary, expected_output",
    [
        ("Public summary", ""),
        ("/home/example", "1: /home/example\n"),
        ("# See /home/example", "1: /home/example\n"),
        ("Public summary\n# See /Users/example", "2: /Users/example\n"),
    ],
)
def test_commit_msg_scans_hash_lines_but_ignores_verbose_diff(
    tmp_path, summary, expected_output
):
    message = tmp_path / "COMMIT_EDITMSG"
    message.write_text(
        f"{summary}\n\n# Please enter the commit message for your changes.\n"
        "# ------------------------ >8 ------------------------\n"
        "diff --git a/notes.md b/notes.md\n+/home/example/private.md\n"
    )
    result = _cli("--commit-msg-file", message, cwd=tmp_path)
    assert result.returncode == bool(expected_output), result.stderr
    assert result.stdout == expected_output


@pytest.mark.parametrize("mode", ["--staged", "--files", "--all-tracked"])
def test_file_names_are_scanned_even_with_public_contents(repo, mode):
    path = repo / "scratchpad" / "notes.md"
    path.parent.mkdir()
    path.write_text("Public summary\n")
    _git(repo, "add", ".")
    args = (mode, path) if mode == "--files" else (mode,)
    result = _cli(*args, cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == "scratchpad/notes.md: scratchpad/notes.md\n"


def test_filename_scan_has_no_content_allowlist_exception(repo, monkeypatch, capsys):
    path = repo / "scratchpad" / "notes.md"
    path.parent.mkdir()
    path.write_text("Public summary\n")
    monkeypatch.setitem(
        checker.ALLOWLIST, "scratchpad/notes.md", checker.re.compile(".*")
    )
    assert checker.scan_file(path, "scratchpad/notes.md")
    assert capsys.readouterr().out == "scratchpad/notes.md: scratchpad/notes.md\n"


@pytest.mark.skipif(os.name != "posix", reason="workflow shell is Bash")
def test_pr_text_uses_only_base_checker_and_handles_missing_checker(tmp_path):
    workflow = (ROOT / ".github/workflows/pr_text.yml").read_text()
    assert "  pull_request_target:\n" in workflow
    assert "types: [opened, edited, reopened, synchronize]" in workflow
    assert "permissions:\n  contents: read\n" in workflow
    assert "uses: actions/checkout@" in workflow
    assert "ref: ${{ github.event.pull_request.base.sha }}" in workflow
    assert "repository: ${{ github.repository }}" in workflow
    assert "sparse-checkout: scripts/check_local_paths.py" in workflow
    assert "sparse-checkout-cone-mode: false" in workflow
    assert "persist-credentials: false" in workflow
    assert "PR_TITLE: ${{ github.event.pull_request.title }}" in workflow
    assert "PR_BODY: ${{ github.event.pull_request.body }}" in workflow
    assert "pull_request.head" not in workflow
    command = textwrap.dedent(workflow.split("        run: |\n", 1)[1])
    env = {**os.environ, "PR_TITLE": "Public summary", "PR_BODY": "Public body"}
    missing = subprocess.run(
        ["bash", "-e", "-c", command],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
    )
    assert missing.returncode == 0, missing.stderr
    assert "::notice::" in missing.stdout
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "check_local_paths.py").write_bytes(SCRIPT.read_bytes())
    blocked = subprocess.run(
        ["bash", "-e", "-c", command],
        cwd=tmp_path,
        env={**env, "PR_BODY": "$(touch injected)\n/home/example"},
        capture_output=True,
        text=True,
    )
    assert blocked.returncode == 1, blocked.stderr
    assert "3: /home/example" in blocked.stdout
    assert not (tmp_path / "injected").exists()


def test_files_and_all_tracked_scan_worktree_and_ignore_untracked(repo):
    tracked = repo / "notes with spaces.md"
    tracked.write_text("Public summary\n")
    _git(repo, "add", tracked.name)
    tracked.write_text("Public summary\n/Users/example/checkout\n")
    (repo / "untracked.md").write_text("scratchpad/private.md\n")
    (repo / "binary.dat").write_bytes(b"\x00\xffpublic asset")
    _git(repo, "add", "binary.dat")
    for args in (("--files", tracked, repo / "binary.dat"), ("--all-tracked",)):
        result = _cli(*args, cwd=repo)
        assert result.returncode == 1, result.stderr
        assert result.stdout == "notes with spaces.md:2: /Users/example/checkout\n"
    tracked.write_text("Public summary\n")
    assert _cli("--all-tracked", cwd=repo).returncode == 0


def test_binary_asset_cannot_hide_embedded_path(repo):
    (repo / "binary.dat").write_bytes(b"\x00\xff/home/example/metadata\n")
    _git(repo, "add", "binary.dat")
    result = _cli("--all-tracked", cwd=repo)
    assert result.returncode == 1
    assert "binary.dat:1: /home/example/metadata" in result.stdout


def test_staged_binary_cannot_hide_embedded_path(repo):
    (repo / "binary.dat").write_bytes(b"\x00\xff/home/example/metadata\n")
    _git(repo, "add", "binary.dat")
    result = _cli("--staged", cwd=repo)
    assert result.returncode == 1
    assert "binary.dat:1: /home/example/metadata" in result.stdout


def test_missing_file_fails_closed(repo):
    result = _cli("--files", "missing.md", cwd=repo)
    assert result.returncode == 2
    assert "Local path check failed" in result.stderr


def test_files_scan_does_not_require_git(tmp_path):
    path = tmp_path / "notes.md"
    path.write_text("/home/example/checkout\n")
    result = _cli("--files", path, cwd=tmp_path)
    assert result.returncode == 1
    assert result.stdout == "notes.md:1: /home/example/checkout\n"


def test_staged_only_checks_added_index_lines_and_reports_new_line_numbers(repo):
    tracked = repo / "notes with spaces.md"
    tracked.write_text(".sisyphus/plans/existing.md\nremoved\npublic\n")
    _git(repo, "add", tracked.name)
    _git(
        repo,
        "-c",
        "user.name=DART Test",
        "-c",
        "user.email=test@example.com",
        "commit",
        "--no-verify",
        "-qm",
        "base",
    )
    tracked.write_text(".sisyphus/plans/existing.md\npublic\nnew summary\n")
    _git(repo, "add", tracked.name)
    tracked.write_text("/home/example/unstaged\n")
    assert _cli("--staged", cwd=repo).returncode == 0

    tracked.write_text(".sisyphus/plans/existing.md\npublic\n/tmp/claude-new/log\n")
    _git(repo, "add", tracked.name)
    tracked.write_text("Public worktree hides staged leak\n")
    result = _cli("--staged", cwd=repo)
    assert result.returncode == 1
    assert result.stdout == "notes with spaces.md:3: /tmp/claude-new/log\n"

    _git(repo, "rm", "-f", tracked.name)
    assert _cli("--staged", cwd=repo).returncode == 0


def test_staged_new_file_allows_gitignore_and_checker_fixtures(repo):
    (repo / ".gitignore").write_text(".sisyphus/\n")
    fixture = repo / "tests" / "test_check_local_paths.py"
    fixture.parent.mkdir()
    fixture.write_text("/home/example/fixture\n")
    _git(repo, "add", ".")
    assert _cli("--staged", cwd=repo).returncode == 0
    assert _cli("--all-tracked", cwd=repo).returncode == 0
    (repo / ".gitignore").write_text(".sisyphus/\n/home/example/ignored\n")
    _git(repo, "add", ".gitignore")
    assert _cli("--staged", cwd=repo).returncode == 1


@pytest.mark.skipif(os.name != "posix", reason="symlink creation needs privileges")
def test_tracked_symlink_scans_target_without_reading_external_file(repo):
    (repo / "link").symlink_to("/home/example/private.md")
    _git(repo, "add", "link")
    result = _cli("--all-tracked", cwd=repo)
    assert result.returncode == 1
    assert result.stdout == "link:1: /home/example/private.md\n"


def test_staged_hook_rejects_leak_outside_ai_infrastructure(repo):
    (repo / "source.cpp").write_text("// scratchpad/private.md\n")
    _git(repo, "add", "source.cpp")
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "check_agent_hook.py"),
            "--profile",
            "staged",
            "--repo-root",
            str(repo),
        ],
        cwd=repo,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert "source.cpp:1: scratchpad/private.md" in result.stdout
