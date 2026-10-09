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
        ".sisyphus/plans/example.md",
        "../.sisyphus/notes/example.md",
        "checkout/.ab/control/example.json",
        ".ab/example.json",
        "/tmp/claude-example/example/notes.md",
        ".claude/projects/example/notes.md",
        "scratchpad/example.md",
        "checkout/scratchpad/example.md",
        "task_2/scripts/example.py",
        "checkout/task_12-example/example",
        "/home/example/worktree/file.md",
        "/home/example",
        "/Users/example",
        "/mnt/c/Users/example",
        "cwd:/home/example/private.md",
        "../home/example/private.md",
        "../../Users/example/private.md",
        "/workspace/example/notes.md",
        "/workspace/example",
        "/workspaces/example/notes.md",
        "/workspaces/example",
        "/__w/example/example/notes.md",
        "/__w/example",
        r"D:\a\example\example\notes.md",
        "D:/a/example/example/notes.md",
        r"C:\a\example-repo\example-repo",
        "WORKDIR /workspaces/example/private.md",
        "Public note: WORKDIR /workspaces/example",
        "cwd:/workspace/example/notes.md",
        "../workspaces/example/notes.md",
        "../../__w/example/notes.md",
        r"\\corp-fs\Users\example\notes.md",
        "//corp-fs/Users/example/notes.md",
        "file:/home/example/private.md",
        "cwd:/root/example.md",
        "/mnt/c/Users/example/",
        "/mnt/d/Users/example/x.md",
        r"C:\Users\example",
        r"C:\Users\Example User",
        "C:/Users/example",
        "/" "root",
        "/root/example.md",
        r"/root\example.md",
        "/Users/example/worktree/file.md",
        r"C:\Users\example\worktree\file.md",
        r"C:\Users\Example User\worktree\file.md",
        r"C:\\Users\\example\\worktree\\file.md",
        "C:/Users/example/worktree/file.md",
        r"checkout\.ab\example.json",
        r"task_3\scripts\example.py",
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
        "/mnt/data/models/x.md",
        "/tmp/dart-visual-evidence",
        "/tmp/dart-agent-visual-smoke/smoke.png",
        "docs/plans/public.md",
        ".claude/commands/dart-pr.md",
        "task_2 is a logical task identifier",
        "multitask_2/file.md",
        "named-scratchpad/file.md",
        "directory.ab/file.md",
        r"(?:/home/|/Users/|~/|fbsource|arvr/libraries)",
        "~example/notes.md",
        "https://github.com/dartsim/dart/pull/1234",
        "https://api.github.com/users/dartsim/repos",
        "https://[2606:4700:4700::1111]/scratchpad/issue",
        "https://8.8.8.8/scratchpad/example.md",
        r"http://example.com\scratchpad\example.md",
        "http://localhost@github.com/example/scratchpad/issues/1",
        "https://example.com/home/docs/index.html",
        "https://bücher.de/home/docs/index.html",
        "https://xn--bcher-kva.de/home/docs/index.html",
        "https://github.com/example/scratchpad/issues/1",
        "https://github.com/org/repo/blob/main/task_2/script.py",
        "HTTP://example.com/.ab/example.json",
        "https://example.com/.sisyphus/plans/example.md",
        "https://example.com/.claude/projects/example/notes.md",
        "https://example.com/tmp/claude-example/example/notes.md",
        "https://example.com/root/notes.md",
        "https://example.com/C:/Users/example/notes.md",
        "example.com/home/docs",
        "example.com/mnt/c/Users/example",
        "C:/home/example",
        "C:/mnt/c/Users/example",
        "https://example.com/workspace/docs",
        "https://example.com/workspaces/docs",
        "https://example.com/__w/docs",
        "example.com/workspace/docs",
        "identifier/workspaces/example",
        "identifier/__w/example",
        "C:/workspace/example",
        r"C:\a\example\other\notes.md",
        "C:/a/example/other/notes.md",
        "C:/a/example",
        "C:/tools/example/example/notes.md",
        "WORKDIR /workspaces/example",
        "WORKDIR /workspace/example",
        "WORKDIR //workspace//example",
        "WORKDIR //workspaces//example",
        "C://home/example",
        "C://workspace/example",
        "/rooted/file.md",
        "/Root 1 0 R",
    ),
)
def test_public_examples_and_non_path_identifiers_pass(text, capsys):
    assert not checker.scan_text(text)
    assert not capsys.readouterr().out


@pytest.mark.parametrize(
    "url",
    (
        "http://localhost:8000/scratchpad/example.md",
        "http://printer.local/scratchpad/example.md",
        "http://127.1/scratchpad/example.md",
        r"http://127.0.0.1\scratchpad\example.md",
        r"https://127.0.0.1\home\example\private.md",
        "http://239.255.255.250/scratchpad/example.md",
        "http://[ff02::1]/scratchpad/example.md",
        "http://0x7f.1/scratchpad/example.md",
        "http://api.localhost/scratchpad/example.md",
        "http://LOCALHOST/scratchpad/example.md",
        "http://localhost./scratchpad/example.md",
        "http://github.com@127.0.0.1/scratchpad/example.md",
        "http://127.0.0.1/.claude/projects/example",
        "http://192.168.1.5/home/example/x",
        "http://169.254.1.5/home/example/x",
        "http://0.0.0.0/home/example/x",
        "http://[::1]/scratchpad/example",
        "http://[::]/home/example/x",
        "http://[fd00::1]/home/example/x",
        "http://[fe80::1]/home/example/x",
        "https:///home/example/x",
        "http://intranet/scratchpad/example",
        "http://[invalid]/scratchpad/example",
    ),
)
def test_local_or_malformed_urls_do_not_hide_private_paths(url, capsys):
    assert checker.scan_text(url)
    assert capsys.readouterr().out.startswith("1: ")


@pytest.mark.parametrize("path", ("/c/Users/example/x.md", "/d/Users/example"))
def test_git_bash_profiles_are_reported(path, capsys):
    assert checker.scan_text(path)
    assert capsys.readouterr().out == f"1: {path}\n"
    assert not checker.scan_text("/c/tools")


@pytest.mark.parametrize(
    "uri",
    (
        "file://localhost/home/example/x",
        "file://example.com/Users/example/x",
        "vscode://file/home/example/x",
    ),
)
def test_local_file_and_editor_uri_paths_are_reported(uri, capsys):
    assert checker.scan_text(uri)
    assert capsys.readouterr().out.startswith("1: /")


@pytest.mark.parametrize("suffix", ("home.arpa", "internal", "lan", "localdomain"))
def test_special_use_hosts_do_not_mask_paths(suffix, capsys):
    assert checker.scan_text(
        f"http://printer.{suffix}/scratchpad/example.md"
    )  # path-fixture
    assert capsys.readouterr().out == "1: scratchpad/example.md\n"  # path-fixture


@pytest.mark.parametrize(
    "host",
    [
        "app.test",
        "app.invalid",
        "app.example",
        "app.localhost",
        "app.local",
        "app.internal",
        "app.home.arpa",
        "foo..bar",
        "-foo.bar",
        "foo-.bar",
        "foo.-bar",
        "foo.bar-",
        ".foo.bar",
        "foo.bar..",
        "foo_bar.com",
        "a" * 64 + ".com",
        "bücher.local",
        "\u200d.com",
        "ü" * 58 + ".de",
    ],
)
def test_non_public_dns_hosts_do_not_mask_home_paths(host, capsys):
    assert checker.scan_text(f"http://{host}/home/example/private.md")
    assert capsys.readouterr().out == "1: /home/example/private.md\n"


@pytest.mark.parametrize(
    "path",
    [
        "/home//example/private.md",
        "//home///example/private.md",
        "/Users//example/private.md",
        "/workspace//example/private.md",
        "/workspaces//example/private.md",
        "/__w//example/private.md",
        "/mnt//c//Users//example/private.md",
        "/c//Users//example/private.md",
        "/tmp//claude-example/example/private.md",
        ".claude//projects//example/private.md",
        ".sisyphus//example.md",
        ".ab//example.md",
        "scratchpad//example.md",
        r"C:\\Users\\example\private.md",
        r"D:\\a\\example\\example\private.md",
        r"\\example\\Users\\example\private.md",
        r"\\wsl.localhost\\example\\home\\example\private.md",
    ],
)
def test_redundant_root_separators_do_not_hide_paths(path, capsys):
    assert checker.scan_text(path)
    assert capsys.readouterr().out.startswith("1: ")


@pytest.mark.parametrize(
    "mode,expected", [("strip", False), ("verbatim", True), ("whitespace", True)]
)
@pytest.mark.parametrize(
    "cleanup_option", ["--cleanup={mode}", "--cleanup {mode}", "--cle={mode}"]
)
@pytest.mark.parametrize(
    "strategy", [[], ["-s", "ort"], ["-sort"], ["-X", "ours"], ["-Xours"]]
)
def test_merge_cleanup_keeps_editor_appended_comments(
    mode, expected, cleanup_option, strategy
):
    text = "Public merge\n# Lines starting with '#' will be ignored,\n# /home/example/private.md\n"
    assert (
        checker.scan_commit_message(
            text,
            [
                "git",
                "merge",
                "--edit",
                *strategy,
                *cleanup_option.format(mode=mode).split(),
            ],
        )
        == expected
    )


@pytest.mark.parametrize("arguments", [[], ["--edit"], ["--no-edit"]])
@pytest.mark.parametrize("configured", [None, "default", "strip", "scissors"])
@pytest.mark.parametrize("explicit_default", [False, True])
def test_merge_comments_require_explicit_cleanup(
    repo, monkeypatch, arguments, configured, explicit_default
):
    if configured is not None:
        _git(repo, "config", "commit.cleanup", configured)
    monkeypatch.chdir(repo)
    message = "Public merge\n# Build /home/example/private.md\n"
    command = ["git", "merge", *arguments]
    if explicit_default:
        command.append("--cleanup=default")
    # Scissors keeps comments before its cut; strip is the only safe exemption.
    expected = explicit_default or configured != "strip"
    assert checker.scan_commit_message(message, command, "#") == expected


def test_public_url_with_balanced_parentheses_is_fully_masked(capsys):
    assert not checker.scan_text("https://example.com/a(b)/scratchpad/public.md")
    assert not capsys.readouterr().out


def test_public_url_leaves_unbalanced_markdown_parenthesis(capsys):
    assert checker.scan_text("[text](https://example.com/x)/home/example/x")
    assert capsys.readouterr().out == "1: /home/example/x\n"


@pytest.mark.parametrize("delimiter", ["|", ",", ";"])
@pytest.mark.parametrize("url_tail", ["", "/docs", "/a(b)"])
def test_public_url_does_not_hide_delimited_path(delimiter, url_tail, capsys):
    assert checker.scan_text(
        f"https://example.com{url_tail}{delimiter}/home/example/private.md"
    )
    assert capsys.readouterr().out == "1: /home/example/private.md\n"


@pytest.mark.parametrize("host", ["wsl.localhost", "wsl$"])
@pytest.mark.parametrize("separator", ["\\", "/"])
@pytest.mark.parametrize(
    "home", ["home/example", "home/example/private.md", "root", "root/example.md"]
)
def test_wsl_unc_homes_are_reported(host, separator, home, capsys):
    path = f"//{host}/example/{home}".replace("/", separator)
    assert checker.scan_text(f"Public summary\n`{path}`\n")
    assert capsys.readouterr().out == f"2: {path}\n"
    assert not checker.scan_text(path, "tests/test_check_local_paths.py")
    assert not capsys.readouterr().out


@pytest.mark.parametrize(
    "path",
    [
        r"\\example\example\home\example\private.md",
        "//wsl.localhost/example/usr/share/example",
        "//wsl$/example/rooted/example.md",
    ],
)
def test_other_unc_paths_still_pass(path, capsys):
    assert not checker.scan_text(path)
    assert not capsys.readouterr().out


def test_json_escaped_slashes_only_are_normalized(capsys):
    text = "Public summary\n" + r'{"note":"\n","path":"\/home\/example\/private.md"}'
    assert checker.scan_text(text, "notes.json")
    assert capsys.readouterr().out == "notes.json:2: /home/example/private.md\n"
    assert not checker.scan_text(r"https:\/\/example.com\/home\/example\/public.md")
    assert not checker.scan_text(r"\u002fhome\u002fexample\u002fprivate.md")
    assert not capsys.readouterr().out


def test_reports_all_leaks_and_file_line(capsys):
    assert checker.scan_text(
        "summary\n/home/example/checkout and .sisyphus/plans/example.md\n",
        "notes.md",
    )
    assert capsys.readouterr().out.splitlines() == [
        "notes.md:2: .sisyphus/plans/example.md",
        "notes.md:2: /home/example/checkout",
    ]


def test_urls_do_not_hide_adjacent_paths_or_file_urls(capsys):
    assert checker.scan_text(
        "https://example.com/scratchpad/public.md /home/example\n"
        "[public](https://example.com/task_2/file.py),scratchpad/example.md\n"
        "file:///Users/example/private.md\n"
        "https://example.com/public]/home/example\n"
    )
    assert capsys.readouterr().out.splitlines() == [
        "1: /home/example",
        "2: scratchpad/example.md",
        "3: ///Users/example/private.md",
        "3: /Users/example/private.md",
        "4: /home/example",
    ]


def test_allowlist_is_limited_to_exact_file_and_matched_fixture():
    assert not checker.scan_text(".sisyphus" "/\n", ".gitignore")  # path-fixture
    assert checker.scan_text(".sisyphus/plans/example.md\n", ".gitignore")
    assert checker.scan_text(".sisyphus" "/\n", "nested/.gitignore")  # path-fixture
    assert not checker.scan_text(
        "/home/example/fixture\n", "tests/test_check_local_paths.py"
    )
    assert checker.scan_text("/home/example/fixture\n", "tests/test_other.py")
    assert not checker.scan_text(
        "/root/path-fixture\n", "tests/test_check_local_paths.py"
    )
    assert checker.scan_text(".sisyphus" "/\n")  # path-fixture


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
    text = "$(touch injected)\n`touch injected`\n.sisyphus/plans/example.md\n"
    title_body = tmp_path / "pr-body.md"
    title_body.write_text(text)
    for args, input_text in ((("--stdin",), text), (("--text-file", title_body), None)):
        result = _cli(*args, cwd=tmp_path, text=input_text)
        assert result.returncode == 1, result.stderr
        assert result.stdout == "3: .sisyphus/plans/example.md\n"
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
        ("# See /home/example/x", "1: /home/example/x\n"),
        ("Public summary\n# See /Users/example", "2: /Users/example\n"),
    ],
)
def test_commit_msg_scans_hash_lines_and_literal_verbose_diff(
    tmp_path, summary, expected_output
):
    message = tmp_path / "COMMIT_EDITMSG"
    message.write_text(
        f"{summary}\n\n# Please enter the commit message for your changes.\n"
        "# ------------------------ >8 ------------------------\n"
        "diff --git a/notes.md b/notes.md\n+/home/example/private.md\n"
    )
    result = _cli("--commit-msg-file", message, cwd=tmp_path)
    assert result.returncode == 1, result.stderr
    assert result.stdout == (
        expected_output + f"{len(summary.splitlines()) + 5}: /home/example/private.md\n"
    )


@pytest.mark.parametrize("comment_char", ["#", ";", "!", "//", "REM"])
@pytest.mark.parametrize("cleanup", ["strip", "scissors"])
def test_commit_msg_editor_template_respects_cleanup(repo, comment_char, cleanup):
    message = repo / "COMMIT_EDITMSG"
    instruction = (
        f"{comment_char} Lines starting with '{comment_char}' will be ignored, and an empty message aborts the commit.\n"
        if cleanup == "strip"
        else f"{comment_char} Do not modify or remove the line above.\n"
    )
    scissors = f"{comment_char} ------------------------ >8 ------------------------\n"
    message.write_text(
        "Public summary\n"
        f"{comment_char}\tdeleted: scratchpad/example.md\n"
        f"{comment_char} See /home/example/x\n"
        + (instruction if cleanup == "strip" else scissors + instruction)
        + scissors
        + "+/home/example/private.md\n"
    )
    result = _cli("--commit-msg-file", message, cwd=repo)
    assert result.returncode == (0 if cleanup == "strip" else 1), result.stdout
    assert result.stdout == (
        "" if cleanup == "strip" else "2: scratchpad/example.md\n3: /home/example/x\n"
    )


@pytest.mark.parametrize("comment_char", ["#", ";", "!"])
@pytest.mark.parametrize("cleanup", ["strip", "scissors"])
def test_commit_msg_uses_real_git_editor_templates(repo, comment_char, cleanup):
    _git(repo, "symbolic-ref", "HEAD", "refs/heads/main")
    (repo / "notes.md").write_text("public\n")
    _commit(repo)
    (repo / "notes.md").write_text("updated\n")
    _git(repo, "add", "notes.md")
    result = subprocess.run(
        [
            "git",
            "-c",
            "user.name=Example",
            "-c",
            "user.email=example@example.com",
            "-c",
            f"core.commentChar={comment_char}",
            "commit",
            *(["--cleanup=scissors", "-v"] if cleanup == "scissors" else []),
        ],
        cwd=repo,
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "GIT_CONFIG_GLOBAL": os.devnull,
            "GIT_CONFIG_SYSTEM": os.devnull,
            "GIT_EDITOR": "cat",
            "LC_ALL": "C",
        },
    )
    assert result.returncode == 1, result.stderr
    template = (repo / ".git/COMMIT_EDITMSG").read_text()
    assert result.stdout == template
    if cleanup == "strip":
        assert (
            f"{comment_char} Please enter the commit message for your changes. Lines starting\n"
            f"{comment_char} with '{comment_char}' will be ignored, and an empty message aborts the commit.\n"
        ) in template
    else:
        assert (
            f"{comment_char} ------------------------ >8 ------------------------\n"
            f"{comment_char} Do not modify or remove the line above.\n"
            f"{comment_char} Everything below it will be ignored.\n"
        ) in template
        assert "diff --git a/notes.md b/notes.md\n" in template
    message = repo / "message.txt"
    message.write_text(
        f"Public summary\n{comment_char} See /home/example/notes.md\n" + template
    )
    scan = _cli("--commit-msg-file", message, cwd=repo)
    assert scan.returncode == (0 if cleanup == "strip" else 1), scan.stdout
    assert scan.stdout == ("" if cleanup == "strip" else "2: /home/example/notes.md\n")


@pytest.mark.parametrize(
    "instruction",
    [
        "# Do not modify or remove the line above.",
        "# Lines starting with ';' will be ignored,",
    ],
)
def test_commit_msg_incomplete_template_still_scans_comments(tmp_path, instruction):
    message = tmp_path / "COMMIT_EDITMSG"
    message.write_text(instruction + "\n# See /home/example/x\n")
    result = _cli("--commit-msg-file", message, cwd=tmp_path)
    assert result.returncode == 1, result.stderr
    assert result.stdout == "2: /home/example/x\n"


@pytest.mark.parametrize("comment_char", ["#", ";", "!", "//", "REM"])
def test_commit_msg_recognizes_gits_wrapped_template(tmp_path, comment_char):
    # Exact text Git writes for an editor commit (the instruction wraps).
    message = tmp_path / "COMMIT_EDITMSG"
    message.write_text(
        (
            "Remove a legacy example note\n\n"
            "# Please enter the commit message for your changes. Lines starting\n"
            "# with '#' will be ignored, and an empty message aborts the commit.\n"
            "#\n"
            "# On branch main\n"
            "# Changes to be committed:\n"
            "#\tdeleted:    scratchpad/example.md\n"
            "#\n"
        ).replace("#", comment_char)
    )
    assert _cli("--commit-msg-file", message, cwd=tmp_path).returncode == 0


def test_commit_msg_template_still_scans_published_lines(tmp_path):
    message = tmp_path / "COMMIT_EDITMSG"
    message.write_text(
        "# Lines starting with '#' will be ignored,\n"
        "See /home/example/x\n"
        "; See /home/example/y\n"
    )
    result = _cli("--commit-msg-file", message, cwd=tmp_path)
    assert result.returncode == 1, result.stderr
    assert result.stdout == "2: /home/example/x\n3: /home/example/y\n"


def test_commit_msg_scissors_follow_custom_comment_char(tmp_path):
    message = tmp_path / "COMMIT_EDITMSG"
    message.write_text(
        "Public summary\n\n"
        "; ------------------------ >8 ------------------------\n"
        "; Do not modify or remove the line above.\n"
        "diff --git a/notes.md b/notes.md\n+/home/example/private.md\n"
    )
    result = _cli("--commit-msg-file", message, cwd=tmp_path)
    assert result.returncode == 0, result.stdout


@pytest.mark.parametrize("comment_char", ["#", ";", "!"])
@pytest.mark.parametrize(
    "evidence",
    ["none", "nonadjacent", "mismatched"],
)
def test_commit_msg_literal_scissors_need_adjacent_matching_instruction(
    comment_char, evidence, capsys
):
    following = ""
    if evidence == "nonadjacent":
        following = f"\n{comment_char} Do not modify or remove the line above.\n"
    elif evidence == "mismatched":
        other_char = ";" if comment_char == "#" else "#"
        following = f"{other_char} Do not modify or remove the line above.\n"
    message = (
        f"Public summary\n{comment_char} ------------------------ >8 ------------------------\n"
        + following
        + "See /home/example/private.md\n"
    )
    assert checker.scan_commit_message(message)
    assert "/home/example/private.md" in capsys.readouterr().out


@pytest.mark.parametrize("option", ["-m", "-F"])
@pytest.mark.skipif(os.name != "posix", reason="commit-msg hook is a POSIX sh script")
def test_commit_msg_rejects_literal_status_line(repo, option):
    (repo / "scratchpad").mkdir()
    (repo / "scratchpad/example.md").write_text("Public summary\n")
    _commit(repo)
    _git(repo, "rm", "scratchpad/example.md")
    hook = repo / ".git/hooks/commit-msg"
    hook.write_text(
        f'#!/bin/sh\nexec "{sys.executable}" "{SCRIPT}" --commit-msg-file "$1"\n'
    )
    hook.chmod(0o755)
    message = "Public summary\n\n#\tdeleted: scratchpad/example.md\n"
    value = message
    if option == "-F":
        (repo / "message.txt").write_text(message)
        value = "message.txt"
    result = subprocess.run(
        [
            "git",
            "-c",
            "user.name=Example",
            "-c",
            "user.email=example@example.com",
            "commit",
            option,
            value,
        ],
        cwd=repo,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1, result.stderr
    assert "3: scratchpad/example.md" in result.stderr


def _commit(repo, message="Public fixture"):
    _git(repo, "add", ".")
    _git(
        repo,
        "-c",
        "user.name=Example",
        "-c",
        "user.email=example@example.com",
        "commit",
        "-qm",
        message,
    )
    return _git(repo, "rev-parse", "HEAD").stdout.strip()


@pytest.mark.parametrize("mode", ["--staged", "--commit-range"])
@pytest.mark.parametrize("root_commit", [False, True])
def test_gitlink_names_are_scanned_despite_diff_config(repo, mode, root_commit):
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    if root_commit:
        _git(repo, "checkout", "--orphan", "topic")
        _git(repo, "rm", "-r", "--cached", ".")
    filename = "scratchpad" + "/private-module"
    _git(repo, "config", "diff.ignoreSubmodules", "all")
    _git(repo, "update-index", "--add", "--cacheinfo", f"160000,{base},{filename}")
    args = (mode,)
    if mode == "--commit-range":
        _git(
            repo,
            "-c",
            "user.name=Example",
            "-c",
            "user.email=example@example.com",
            "commit",
            "-qm",
            "Public gitlink",
        )
        head = _git(repo, "rev-parse", "HEAD").stdout.strip()
        args = (mode, head if root_commit else f"{base}..{head}")
    result = _cli(*args, cwd=repo)
    assert result.returncode == 1, result.stderr
    assert filename in result.stdout


@pytest.mark.parametrize("mode", ["--staged", "--commit-range", "--all-tracked"])
@pytest.mark.parametrize("entry,expected", [("", 0), (" ", 1), ("private.md", 1)])
def test_crlf_ignore_entry_preserves_exact_allowlist(repo, mode, entry, expected):
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    (repo / ".gitignore").write_bytes((".sisyphus" + "/" + entry + "\r\n").encode())
    _git(repo, "add", ".gitignore")
    args = (mode,)
    if mode == "--commit-range":
        head = _commit(repo)
        args = (mode, f"{base}..{head}")
    result = _cli(*args, cwd=repo)
    assert result.returncode == expected, result.stdout + result.stderr


@pytest.mark.parametrize(
    "escaped_path, reported_path",
    [
        (r"\/home\/example\/private.md", "/home/example/private.md"),
        (
            r"\\\\wsl.localhost\\example\\home\\example\\private.md",
            r"\\\\wsl.localhost\\example\\home\\example\\private.md",
        ),
        (
            r"\\\\corp-fs\\Users\\example\\private.md",
            r"\\\\corp-fs\\Users\\example\\private.md",
        ),
        (r"C:\\Users\\example\\private.md", r"C:\\Users\\example\\private.md"),
    ],
)
@pytest.mark.parametrize(
    "mode",
    [
        "--stdin",
        "--text-file",
        "--commit-msg-file",
        "--files",
        "--all-tracked",
        "--staged",
        "--commit-range",
    ],
)
def test_json_escaped_paths_are_reported_in_every_mode(
    repo, mode, escaped_path, reported_path
):
    path = repo / "notes.json"
    path.write_text("Public summary\n")
    base = _commit(repo)
    text = "Public summary\n" + '{"path":"' + escaped_path + '"}\n'
    path.write_text(text)
    _git(repo, "add", "notes.json")
    args = (mode,)
    locations = ["2"]
    if mode in {"--text-file", "--commit-msg-file", "--files"}:
        args = (mode, "notes.json")
    if mode in {"--files", "--all-tracked", "--staged"}:
        locations = ["notes.json:2"]
    if mode == "--commit-range":
        head = _commit(repo, text)
        args = (mode, f"{base}..{head}")
        locations = [f"{head}:2", f"{head}:notes.json:2"]
    result = _cli(*args, cwd=repo, text=text if mode == "--stdin" else None)
    assert result.returncode == 1, result.stderr
    assert result.stdout.splitlines() == [
        f"{location}: {reported_path}" for location in locations
    ]


def test_commit_range_reports_leak_removed_before_tip(repo):
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    (repo / "notes.md").write_text("Public summary\n/home/example/private.md\n")
    leaked = _commit(repo)
    (repo / "notes.md").write_text("Public summary\n")
    head = _commit(repo)
    result = _cli("--commit-range", f"{base}..{head}", cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == f"{leaked}:notes.md:2: /home/example/private.md\n"


@pytest.mark.parametrize("mode", ["--staged", "--commit-range"])
def test_git_output_filenames_are_literal_pathspecs(repo, mode):
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    filename = ":(exclude)**"
    path = repo / filename
    path.write_text("Public summary\n/home/example/private.md\n")
    _git(repo, "--literal-pathspecs", "add", "--", filename)
    args = (mode,)
    prefix = ""
    if mode == "--commit-range":
        leaked = _commit(repo)
        path.unlink()
        head = _commit(repo)
        args = (mode, f"{base}..{head}")
        prefix = f"{leaked}:"
    result = _cli(*args, cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == f"{prefix}{filename}:2: /home/example/private.md\n"


@pytest.mark.parametrize("operation", ["delete", "rename", "copy"])
def test_commit_range_cleanup_ignores_removed_names(repo, operation):
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    legacy = repo / "scratchpad/example.md"
    legacy.parent.mkdir()
    legacy.write_text("Public summary\n")
    leaked = _commit(repo)
    if operation == "delete":
        _git(repo, "rm", "scratchpad/example.md")
    elif operation == "rename":
        _git(repo, "mv", "scratchpad/example.md", "public.md")
    else:
        (repo / "public.md").write_bytes(legacy.read_bytes())
        _git(repo, "config", "diff.renames", "copies")
    cleaned = _commit(repo)
    cleanup = _cli("--commit-range", f"{leaked}..{cleaned}", cwd=repo)
    assert cleanup.returncode == 0, cleanup.stdout
    introduced = _cli("--commit-range", f"{base}..{cleaned}", cwd=repo)
    assert introduced.returncode == 1, introduced.stderr
    assert (
        introduced.stdout
        == f"{leaked}:scratchpad/example.md:0: scratchpad/example.md\n"
    )


@pytest.mark.parametrize(
    "encoding, bom",
    [
        ("utf-8", b"\x00\xff"),
        ("utf-16-le", b"\xff\xfe"),
        ("utf-16-be", b"\xfe\xff"),
        ("utf-32-le", b"\xff\xfe\x00\x00"),
        ("utf-32-be", b"\x00\x00\xfe\xff"),
    ],
)
def test_commit_range_scans_binary_and_encoded_blobs(repo, encoding, bom):
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    (repo / "binary.dat").write_bytes(
        bom + "Public summary\n/home/example/private.md\n".encode(encoding)
    )
    leaked = _commit(repo)
    _git(repo, "rm", "binary.dat")
    head = _commit(repo)
    result = _cli("--commit-range", f"{base}..{head}", cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == f"{leaked}:binary.dat:2: /home/example/private.md\n"


def test_commit_range_checks_names_and_keeps_per_file_allowlist(repo):
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    (repo / ".gitignore").write_text(".sisyphus" "/\n")  # path-fixture
    (repo / "tests").mkdir()
    (repo / "tests/test_check_local_paths.py").write_text("/home/example/fixture\n")
    allowed = _commit(repo)
    assert _cli("--commit-range", f"{base}..{allowed}", cwd=repo).returncode == 0
    (repo / "scratchpad").mkdir()
    (repo / "scratchpad/example.md").write_text("Public summary\n")
    leaked = _commit(repo)
    _git(repo, "rm", "scratchpad/example.md")
    head = _commit(repo)
    result = _cli("--commit-range", f"{base}..{head}", cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == (
        f"{leaked}:scratchpad/example.md:0: scratchpad/example.md\n"
    )


def test_commit_range_scans_merge_resolution_and_side_commit(repo):
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    _git(repo, "checkout", "-qb", "side")
    (repo / "side.md").write_text("/home/example/side.md\n")
    side = _commit(repo)
    _git(repo, "checkout", "-qb", "topic", base)
    (repo / "topic.md").write_text("Public summary\n")
    _commit(repo)
    _git(
        repo,
        "-c",
        "user.name=Example",
        "-c",
        "user.email=example@example.com",
        "merge",
        "--no-commit",
        "--no-ff",
        "side",
    )
    (repo / "resolution.md").write_text("/home/example/resolution.md\n")
    merged = _commit(repo)
    result = _cli("--commit-range", f"{base}..{merged}", cwd=repo)
    assert result.returncode == 1, result.stderr
    assert f"{side}:side.md:1: /home/example/side.md" in result.stdout
    assert f"{merged}:resolution.md:1: /home/example/resolution.md" in result.stdout


@pytest.mark.parametrize("mode", ["--staged", "--files", "--all-tracked"])
def test_file_names_are_scanned_even_with_public_contents(repo, mode):
    path = repo / "scratchpad" / "example.md"
    path.parent.mkdir()
    path.write_text("Public summary\n")
    _git(repo, "add", ".")
    args = (mode, path) if mode == "--files" else (mode,)
    result = _cli(*args, cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == "scratchpad/example.md: scratchpad/example.md\n"


def test_filename_scan_has_no_content_allowlist_exception(repo, monkeypatch, capsys):
    path = repo / "scratchpad" / "example.md"
    path.parent.mkdir()
    path.write_text("Public summary\n")
    monkeypatch.setitem(
        checker.ALLOWLIST, "scratchpad/example.md", checker.re.compile(".*")
    )
    assert checker.scan_file(path, "scratchpad/example.md")
    assert capsys.readouterr().out == "scratchpad/example.md: scratchpad/example.md\n"


@pytest.mark.skipif(os.name != "posix", reason="workflow shell is Bash")
def test_pr_text_uses_only_base_checker_and_handles_missing_checker(tmp_path):
    workflow = (ROOT / ".github/workflows/pr_text.yml").read_text()
    assert "  pull_request_target:\n" in workflow
    assert "types: [opened, edited, reopened, synchronize]" in workflow
    assert "permissions:\n  contents: read\n" in workflow
    assert "  pull-requests: read\n" in workflow
    assert "uses: actions/checkout@" in workflow
    assert "ref: ${{ github.event.pull_request.base.sha }}" in workflow
    assert "repository: ${{ github.repository }}" in workflow
    assert "sparse-checkout: scripts/check_local_paths.py" in workflow
    assert "sparse-checkout-cone-mode: false" in workflow
    assert "persist-credentials: false" in workflow
    assert "fetch-depth: 0" in workflow
    assert (
        'git fetch --no-tags origin "$BASE_SHA" "+refs/pull/$PR_NUMBER/head:refs/remotes/pr/head"'
        in workflow
    )
    assert '--commit-range "$BASE_SHA..refs/remotes/pr/head"' in workflow
    assert "BASE_SHA: ${{ github.event.pull_request.base.sha }}" in workflow
    assert "PR_TITLE: ${{ github.event.pull_request.title }}" in workflow
    assert "PR_BODY: ${{ github.event.pull_request.body }}" in workflow
    assert "PR_NUMBER: ${{ github.event.pull_request.number }}" in workflow
    assert "GH_TOKEN: ${{ github.token }}" in workflow
    assert "        shell: bash\n" in workflow
    assert "pull_request.head" not in workflow
    command = textwrap.dedent(workflow.split("        run: |\n", 1)[1])
    command = command.replace("${{ github.repository }}", "example/repository")
    assert "${{" not in command
    env = {
        **os.environ,
        "PR_TITLE": "Public summary",
        "PR_BODY": "Public body",
        "PR_NUMBER": "1234",
        "PR_COMMITS": "2",
        "GH_TOKEN": "synthetic-token",
        "COMMIT_MESSAGES": "Public first commit\nPublic second commit",
        "GH_STATUS": "0",
        "PATH": str(tmp_path) + os.pathsep + os.environ["PATH"],
    }
    gh = tmp_path / "gh"
    gh.write_text(
        '#!/bin/sh\n[ "$GH_TOKEN" = synthetic-token ] || exit 2\n'
        'printf "%s\\n" "$@" > gh-args\n'
        'printf "%s\\n" "$COMMIT_MESSAGES"\nexit "$GH_STATUS"\n'
    )
    gh.chmod(0o755)
    missing = subprocess.run(
        ["bash", "-eo", "pipefail", "-c", command],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
    )
    assert missing.returncode == 0, missing.stderr
    assert "::notice::" in missing.stdout
    assert not (tmp_path / "gh-args").exists()
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "check_local_paths.py").write_bytes(SCRIPT.read_bytes())
    _git(tmp_path, "init", "-q")
    base = _commit(tmp_path)
    remote = tmp_path / ".git/origin.git"
    _git(tmp_path, "clone", "--bare", "-q", str(tmp_path), str(remote))
    _git(remote, "update-ref", "refs/pull/1234/head", base)
    _git(tmp_path, "remote", "add", "origin", str(remote))
    env["BASE_SHA"] = base
    for overrides, failures in (
        ({}, ()),
        ({"PR_BODY": "$(touch injected)\n/home/example"}, ("title/body",)),
        (
            {
                "COMMIT_MESSAGES": "Public first commit\n"
                "# ------------------------ >8 ------------------------\n"
                "$(touch injected)\n/home/example",
            },
            ("commit-message",),
        ),
        (
            {"PR_TITLE": "/home/example", "COMMIT_MESSAGES": "/home/example"},
            ("title/body", "commit-message"),
        ),
        ({"GH_STATUS": "1"}, ("commit-message",)),
        ({"BASE_SHA": "f" * 40}, ("commit-change",)),
    ):
        result = subprocess.run(
            ["bash", "-eo", "pipefail", "-c", command],
            cwd=tmp_path,
            env={**env, **overrides},
            capture_output=True,
            text=True,
        )
        assert result.returncode == bool(failures), result.stderr
        for scan in ("title/body", "commit-message", "commit-change"):
            assert (
                f"PR {scan} local-path scan failed." in result.stdout
                or f"PR {scan} fetch failed." in result.stdout
            ) == (scan in failures)
        assert (tmp_path / "gh-args").read_text().splitlines() == [
            "api",
            "--paginate",
            "repos/example/repository/pulls/1234/commits",
            "--jq",
            ".[].commit.message",
        ]
    assert not (tmp_path / "injected").exists()


@pytest.mark.skipif(os.name != "posix", reason="workflow shell is Bash")
def test_pr_text_scans_removed_leak_without_using_head_checker(repo):
    (repo / "scripts").mkdir()
    (repo / "scripts/check_local_paths.py").write_bytes(SCRIPT.read_bytes())
    (repo / "notes.md").write_text("Public summary\n")
    base = _commit(repo)
    (repo / "scripts/check_local_paths.py").write_text(
        "raise AssertionError('PR checker executed')\n"
    )
    (repo / "notes.md").write_text("/home/example/private.md\n")
    leaked = _commit(repo)
    (repo / "notes.md").write_text("Public summary\n")
    _commit(repo)
    _git(repo, "update-ref", "refs/pull/1234/head", "HEAD")
    tree = repo / ".git/trusted-checkout"
    _git(repo, "clone", "--no-checkout", "-q", str(repo), str(tree))
    _git(tree, "checkout", "-q", "--detach", base)
    bin_dir = repo / ".git/bin"
    bin_dir.mkdir()
    gh = bin_dir / "gh"
    gh.write_text("#!/bin/sh\nprintf 'Public commit message\\n'\n")
    gh.chmod(0o755)
    workflow = (ROOT / ".github/workflows/pr_text.yml").read_text()
    command = textwrap.dedent(workflow.split("        run: |\n", 1)[1])
    command = command.replace("${{ github.repository }}", "example/repository")
    result = subprocess.run(
        ["bash", "-eo", "pipefail", "-c", command],
        cwd=tree,
        env={
            **os.environ,
            "PR_TITLE": "Public summary",
            "PR_BODY": "Public body",
            "PR_NUMBER": "1234",
            "PR_COMMITS": "2",
            "BASE_SHA": base,
            "PATH": str(bin_dir) + os.pathsep + os.environ["PATH"],
        },
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1, result.stderr
    assert f"{leaked}:notes.md:1: /home/example/private.md" in result.stdout
    assert "PR commit-change local-path scan failed." in result.stdout
    assert (tree / "scripts/check_local_paths.py").read_bytes() == SCRIPT.read_bytes()
    assert _git(tree, "rev-parse", "HEAD").stdout.strip() == base


@pytest.mark.skipif(os.name != "posix", reason="workflow shell is Bash")
@pytest.mark.parametrize(
    "event, base_checker, leak, expected_status",
    (
        ("pull_request", True, True, 1),
        ("pull_request", True, False, 0),
        ("pull_request", False, True, 1),
        ("pull_request", False, False, 0),
        ("push", True, True, 0),
        ("push", False, True, 1),
    ),
)
def test_tracked_file_ci_uses_base_checker_with_tree_fallback(
    tmp_path, event, base_checker, leak, expected_status
):
    workflow = (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    step = workflow.split("      - name: Check tracked files for local paths\n", 1)[
        1
    ].split("\n      - name:", 1)[0]
    assert "BASE_SHA: ${{ github.event.pull_request.base.sha }}" in step
    command = textwrap.dedent(step.split("        run: |\n", 1)[1])
    assert "${{" not in command

    remote = tmp_path / "origin"
    remote.mkdir()
    _git(remote, "init", "-q")
    (remote / "README.md").write_text("Public summary\n")
    if base_checker:
        (remote / "scripts").mkdir()
        (remote / "scripts/check_local_paths.py").write_bytes(SCRIPT.read_bytes())
    _git(remote, "add", ".")
    _git(
        remote,
        "-c",
        "user.name=Example",
        "-c",
        "user.email=example@example.com",
        "commit",
        "-qm",
        "Base fixture",
    )
    base_sha = _git(remote, "rev-parse", "HEAD").stdout.strip()
    tree = tmp_path / "checkout"
    _git(tmp_path, "clone", "-q", str(remote), str(tree))
    (tree / "scripts").mkdir(exist_ok=True)
    (tree / "scripts/check_local_paths.py").write_bytes(
        b"raise SystemExit(0)\n" if base_checker else SCRIPT.read_bytes()
    )
    (tree / "notes.md").write_text("/home/example/x\n" if leak else "Public summary\n")
    _git(tree, "add", ".")
    if event == "push":
        _git(tree, "remote", "remove", "origin")
    runner_temp = tmp_path / "runner"
    runner_temp.mkdir()
    result = subprocess.run(
        ["bash", "-eo", "pipefail", "-c", command],
        cwd=tree,
        env={
            **os.environ,
            "GITHUB_EVENT_NAME": event,
            "BASE_SHA": base_sha,
            "RUNNER_TEMP": str(runner_temp),
        },
        capture_output=True,
        text=True,
    )
    assert result.returncode == expected_status, result.stderr
    assert ("notes.md:1: /home/example/x" in result.stdout) == bool(expected_status)
    assert ("::notice::" in result.stdout) == (
        event == "pull_request" and not base_checker
    )
    extracted = runner_temp / "check_local_paths.py"
    if event == "pull_request" and base_checker:
        assert extracted.read_bytes() == SCRIPT.read_bytes()
    elif event == "push":
        assert not extracted.exists()


def test_files_and_all_tracked_scan_worktree_and_ignore_untracked(repo):
    tracked = repo / "notes with spaces.md"
    tracked.write_text("Public summary\n")
    _git(repo, "add", tracked.name)
    tracked.write_text("Public summary\n/Users/example/checkout\n")
    (repo / "untracked.md").write_text("scratchpad/example.md\n")
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


@pytest.mark.parametrize(
    "data",
    [b"\x00\xff" * 20, b"\x00\x00" * 20, b"x\x00" * 5 + b"\xff\x00" * 5],
)
def test_utf16_heuristic_requires_printable_opposite_parity(data):
    assert checker.text_encoding(data) is None
    assert checker.decode(data) == data.decode("utf-8", errors="replace")


@pytest.mark.parametrize(
    "encoding,bom",
    [
        ("utf-16-le", b""),
        ("utf-16-be", b""),
        ("utf-32-le", b""),
        ("utf-32-be", b""),
        ("utf-16-le", b"\xff\xfe"),
        ("utf-16-be", b"\xfe\xff"),
        ("utf-32-le", b"\xff\xfe\x00\x00"),
        ("utf-32-be", b"\x00\x00\xfe\xff"),
    ],
)
@pytest.mark.parametrize(
    "mode",
    [
        "--files",
        "--all-tracked",
        "--staged",
        "--commit-range",
        "--text-file",
        "--commit-msg-file",
    ],
)
def test_unicode_files_cannot_hide_paths(repo, encoding, bom, mode):
    if mode == "--commit-range":
        (repo / "base.txt").write_text("Public summary\n")
        base = _commit(repo)
    path = repo / "notes.txt"
    path.write_bytes(
        bom + "Public summary\n/home/example/private.md\n".encode(encoding)
    )
    _git(repo, "add", path.name)
    if mode == "--staged":
        path.write_text("Public worktree hides staged leak\n")
    args = (
        (mode, path)
        if mode in {"--files", "--text-file", "--commit-msg-file"}
        else (mode,)
    )
    if mode == "--commit-range":
        head = _commit(repo, "Encoded fixture")
        args = (mode, f"{base}..{head}")
    result = _cli(*args, cwd=repo)
    assert result.returncode == 1, result.stderr
    prefix = head + ":" if mode == "--commit-range" else ""
    filename = "" if mode in {"--text-file", "--commit-msg-file"} else "notes.txt:"
    assert result.stdout == prefix + filename + "2: /home/example/private.md\n"
    if mode == "--commit-range":
        base = head
    path.write_bytes(bom + "Public summary\n".encode(encoding))
    _git(repo, "add", path.name)
    if mode == "--commit-range":
        head = _commit(repo)
        args = (mode, f"{base}..{head}")
    assert _cli(*args, cwd=repo).returncode == 0


@pytest.mark.parametrize(
    "encoding", ["utf-16-le", "utf-16-be", "utf-32-le", "utf-32-be"]
)
@pytest.mark.parametrize(
    "mode",
    [
        "--files",
        "--all-tracked",
        "--staged",
        "--commit-range",
        "--text-file",
        "--commit-msg-file",
        "--stdin",
    ],
)
def test_bomless_unicode_with_non_ascii_text_cannot_hide_paths(repo, encoding, mode):
    if mode == "--commit-range":
        (repo / "base.txt").write_text("Public summary\n")
        base = _commit(repo)
    path = repo / "notes.txt"
    data = ("日本語" * 100 + "\n/home/example/private.md\n").encode(encoding)
    path.write_bytes(data)
    _git(repo, "add", path.name)
    args = (
        (mode, path)
        if mode in {"--files", "--text-file", "--commit-msg-file"}
        else (mode,)
    )
    if mode == "--staged":
        path.write_text("Public worktree hides staged leak\n")
    if mode == "--commit-range":
        head = _commit(repo)
        args = (mode, f"{base}..{head}")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), *map(str, args)],
        cwd=repo,
        input=data if mode == "--stdin" else None,
        capture_output=True,
    )
    assert result.returncode == 1, result.stderr
    assert b"2: /home/example/private.md\n" in result.stdout


@pytest.mark.parametrize(
    "encoding", ["utf-8", "utf-16-le", "utf-16-be", "utf-32-le", "utf-32-be"]
)
def test_binary_match_keeps_valid_prefix_before_replacement(tmp_path, encoding):
    path = tmp_path / "asset.bin"
    invalid = (
        b"\xff"
        if encoding == "utf-8"
        else "\ud800".encode(encoding, errors="surrogatepass")
    )
    path.write_bytes(
        "/home/example/".encode(encoding) + invalid + "\n".encode(encoding)
    )
    result = _cli("--files", path, cwd=tmp_path)
    assert result.returncode == 1
    assert "/home/example/\n" in result.stdout
    path.write_bytes("/home/example/private.md\n".encode(encoding) + invalid)
    assert _cli("--files", path, cwd=tmp_path).returncode == 1


@pytest.mark.parametrize(
    "mode",
    [
        "--files",
        "--all-tracked",
        "--staged",
        "--commit-range",
        "--text-file",
        "--commit-msg-file",
        "--stdin",
    ],
)
def test_invalid_byte_after_path_keeps_valid_prefix(repo, mode):
    if mode == "--commit-range":
        (repo / "base.txt").write_text("Public summary\n")
        base = _commit(repo)
    data = b"header /home/example/private.md\xfftail"
    path = repo / "notes.txt"
    path.write_bytes(data)
    _git(repo, "add", path.name)
    args = (
        (mode, path)
        if mode in {"--files", "--text-file", "--commit-msg-file"}
        else (mode,)
    )
    if mode == "--commit-range":
        head = _commit(repo)
        args = (mode, f"{base}..{head}")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), *map(str, args)],
        cwd=repo,
        input=data if mode == "--stdin" else None,
        capture_output=True,
    )
    assert result.returncode == 1, result.stderr
    assert b"1: /home/example/private.md\n" in result.stdout
    assert b"tail" not in result.stdout


@pytest.mark.parametrize("path", [r"C:\Users\Example", "//example.local/Users/example"])
def test_replacement_delimits_profile_names_that_allow_spaces(path, capsys):
    assert checker.scan_text(path + "\ufffdtail/private.md")
    assert capsys.readouterr().out == f"1: {path}\n"


@pytest.mark.parametrize("mode", ["--staged", "--commit-range"])
def test_bare_cr_in_added_line_cannot_hide_path(repo, mode):
    (repo / "notes.txt").write_text("Public summary\n")
    base = _commit(repo)
    (repo / "notes.txt").write_bytes(b"prefix\r/home/example/private.md\n")
    _git(repo, "add", "notes.txt")
    args = (mode,)
    if mode == "--commit-range":
        head = _commit(repo)
        args = (mode, f"{base}..{head}")
    result = _cli(*args, cwd=repo)
    assert result.returncode == 1, result.stderr
    assert "notes.txt:1: /home/example/private.md\n" in result.stdout


def test_commit_range_scans_messages_without_cleanup(repo):
    (repo / "base.txt").write_text("Public summary\n")
    base = _commit(repo)
    (repo / "notes.txt").write_text("Public summary\n")
    head = _commit(repo, "Public subject\n\n# /home/example/private.md")
    result = _cli("--commit-range", f"{base}..{head}", cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == f"{head}:3: /home/example/private.md\n"


@pytest.mark.parametrize("comment_string", ["#", ";", "//"])
@pytest.mark.parametrize(
    "arguments,expected",
    [
        ([], False),
        (["-m", "public"], True),
        (["-F", "message.txt"], True),
        (["-C", "HEAD"], True),
        (["-c", "HEAD"], False),
        (["-cHEAD"], False),
        (["--reedit-message=HEAD"], False),
        (["--reedit-message", "HEAD"], False),
        (["--reed=HEAD"], False),
        (["--reed", "HEAD"], False),
        (["-c", "HEAD", "--no-edit"], True),
        (["--reed=HEAD", "--no-ed"], True),
        (["--no-edit", "-c", "HEAD"], True),
        (["--no-ed", "--reed=HEAD"], True),
        (["-F", "message.txt", "-e"], False),
        (["--file=message.txt", "--edit"], False),
        (["--no-edit"], True),
        (["-F", "message.txt", "-e", "--no-edit"], True),
        (["--no-edit", "--edit"], False),
        (["-F", "message.txt", "--cleanup=strip"], False),
        (["--cleanup=whitespace"], True),
        (["--cleanup=verbatim"], True),
        (["--cleanup=scissors"], True),
        (["--cleanup=default", "-mpublic"], True),
        (["--amend", "--no-ed"], True),
        (["--cle=whitespace"], True),
        (["--cle", "whitespace"], True),
        (["--mess", "public"], True),
        (["--fil=message.txt"], True),
        (["--no-ed", "--ed"], False),
        (["--cle=strip", "--mess=public"], False),
        (["--f", "message.txt"], False),
        (["--r", "HEAD"], False),
    ],
)
def test_commit_cleanup_uses_parent_command_not_localized_template(
    comment_string, arguments, expected
):
    message = (
        "Public summary\n"
        f"{comment_string} Veuillez saisir le message de validation.\n"
        f"{comment_string} Voir /home/example/private.md\n"
    )
    assert (
        checker.scan_commit_message(
            message,
            git_command=["git", "commit", *arguments],
            comment_string=comment_string,
        )
        == expected
    )


@pytest.mark.parametrize("configured", ["verbatim", "whitespace", "default"])
@pytest.mark.parametrize(
    "global_args,arguments,expected",
    [
        ([], [], True),
        (["-c", "commit.cleanup=strip"], [], False),
        (["-c", "commit.cleanup=whitespace"], [], True),
        (["-c", "commit.cleanup=verbatim"], [], True),
        (["-c", "commit.cleanup=strip", "-c", "commit.cleanup=verbatim"], [], True),
        ([], ["--cleanup=strip"], False),
        ([], ["--cleanup=default"], False),
    ],
)
def test_commit_cleanup_resolves_repository_config(
    repo, monkeypatch, configured, global_args, arguments, expected
):
    _git(repo, "config", "commit.cleanup", configured)
    monkeypatch.chdir(repo)
    if configured == "default" and not global_args and not arguments:
        expected = False
    assert (
        checker.scan_commit_message(
            "Public summary\n# /home/example/private.md\n",
            git_command=["git", *global_args, "commit", *arguments],
            comment_string="#",
        )
        == expected
    )


@pytest.mark.parametrize(
    "global_args",
    [
        ["-C", "commit"],
        ["-Ccommit"],
        ["--git-dir", "commit"],
        ["--git-dir=commit"],
        ["--work-tree", "commit"],
        ["--work-tree=commit"],
        ["--namespace", "commit"],
        ["--namespace=commit"],
        ["--exec-path"],
        ["--exec-path=commit"],
        ["--config-env", "example.key=EXAMPLE_VALUE"],
        ["--config-env=example.key=EXAMPLE_VALUE"],
        ["--super-prefix", "commit"],
        ["--super-prefix=commit"],
        ["--attr-source", "commit"],
        ["--attr-source=commit"],
        [
            "-p",
            "-P",
            "--paginate",
            "--no-pager",
            "--bare",
            "--no-replace-objects",
            "--no-lazy-fetch",
            "--literal-pathspecs",
            "--glob-pathspecs",
            "--noglob-pathspecs",
            "--icase-pathspecs",
            "--no-optional-locks",
            "--no-advice",
        ],
    ],
)
@pytest.mark.parametrize(
    "override", [["-c", "commit.cleanup=strip"], ["-ccommit.cleanup=strip"]]
)
def test_commit_cleanup_parses_global_options(repo, monkeypatch, global_args, override):
    monkeypatch.chdir(repo)
    _git(repo, "config", "commit.cleanup", "verbatim")
    assert checker.commit_cleanup(
        ["git", *global_args, *override, "commit", "-m", "public"]
    ) == ("strip", False)


@pytest.mark.parametrize(
    "command",
    [
        ["git", "-C", "commit"],
        ["git", "-c"],
        ["git", "--namespace"],
        ["git", "--unknown", "commit"],
        ["git", "--no-pager=value", "commit"],
        ["git", "status", "commit"],
        ["git"],
        [],
    ],
)
@pytest.mark.parametrize("template", [True, False])
def test_unparsable_git_command_falls_back_to_content(command, template):
    message = (
        "# Lines starting with '#' will be ignored,\n" if template else ""
    ) + "# /home/example/private.md\n"
    assert checker.scan_commit_message(message, git_command=command) == (not template)


@pytest.mark.parametrize("source", ["config", "option"])
@pytest.mark.parametrize("arguments", [[], ["-m", "public"], ["--no-edit"]])
def test_scissors_cleanup_only_truncates_editor_messages(
    repo, monkeypatch, source, arguments
):
    monkeypatch.chdir(repo)
    if source == "config":
        _git(repo, "config", "commit.cleanup", "scissors")
    else:
        arguments = ["--cleanup=scissors", *arguments]
    assert checker.scan_commit_message(
        "Public summary\n# ------------------------ >8 ------------------------\n"
        "/home/example/private.md\n",
        git_command=["git", "commit", *arguments],
        comment_string="#",
    ) == ("-m" in arguments or "--no-edit" in arguments)


@pytest.mark.parametrize(
    "arguments,expected",
    [
        ([], True),
        (["--cleanup=whitespace"], True),
        (["--cleanup=verbatim"], True),
        (["--cleanup=scissors"], False),
        (["-v"], False),
        (["--verbose"], False),
        (["--cleanup=strip", "-v"], False),
        (["-v", "--no-verbose"], True),
        (["--no-verbose", "-v"], False),
        (["--verb"], False),
        (["-v", "--no-verb"], True),
        (["-v", "--no-ver"], False),
    ],
)
def test_parent_cleanup_only_honors_scissors_in_scissors_or_verbose_mode(
    arguments, expected
):
    message = (
        "Public summary\n# ------------------------ >8 ------------------------\n"
        "diff --git a/notes.md b/notes.md\n+/home/example/private.md\n"
    )
    assert (
        checker.scan_commit_message(
            message,
            git_command=["git", "commit", *arguments],
            comment_string="#",
        )
        == expected
    )


@pytest.mark.parametrize(
    "configured,global_args,arguments,verbose",
    [
        ("true", [], [], True),
        ("2", [], [], True),
        ("false", [], [], False),
        ("true", [], ["--no-verbose"], False),
        ("true", [], ["--no-verbose", "-v"], True),
        ("false", [], ["--verbose"], True),
        ("true", ["-c", "commit.verbose=false"], [], False),
        ("false", ["-ccommit.verbose=true"], [], True),
    ],
)
def test_commit_verbose_config_precedes_command_overrides(
    repo, monkeypatch, configured, global_args, arguments, verbose
):
    _git(repo, "config", "commit.verbose", configured)
    monkeypatch.chdir(repo)
    command = ["git", *global_args, "commit", *arguments]
    assert checker.commit_cleanup(command) == ("strip", verbose)
    message = (
        "Public summary\n# ------------------------ >8 ------------------------\n"
        + "/home/"
        + "example/private.md\n"
    )
    assert checker.scan_commit_message(message, command, "#") == (not verbose)
    assert checker.commit_cleanup(["git", "merge", "--edit"]) == ("whitespace", False)


@pytest.mark.parametrize("template,expected", [(True, False), (False, True)])
def test_unreadable_git_parent_falls_back_to_english_template(
    tmp_path, template, expected
):
    message = tmp_path / "message.txt"
    message.write_text(
        ("# Lines starting with '#' will be ignored,\n" if template else "")
        + "# /home/example/private.md\n"
    )
    assert _cli(
        "--commit-msg-file", message, "--git-pid", "2147483647", cwd=tmp_path
    ).returncode == int(expected)


def test_read_git_command_uses_proc_nul_delimited_arguments(monkeypatch):
    monkeypatch.setattr(
        Path, "read_bytes", lambda self: b"git\0commit\0-F\0message with spaces.txt\0"
    )
    assert checker.read_git_command(123) == [
        "git",
        "commit",
        "-F",
        "message with spaces.txt",
    ]


def test_read_git_command_falls_back_to_ps(monkeypatch):
    def unreadable(self):
        raise OSError("process command line unavailable")

    monkeypatch.setattr(Path, "read_bytes", unreadable)

    def ps(command, **kwargs):
        assert command == ["ps", "-o", "args=", "-p", "123"]
        return subprocess.CompletedProcess(command, 0, "git commit --cleanup=strip\n")

    monkeypatch.setattr(checker.subprocess, "run", ps)
    assert checker.read_git_command(123) == ["git", "commit", "--cleanup=strip"]


def test_commit_cleanup_preserves_configured_comment_string_spaces(monkeypatch, capsys):
    monkeypatch.setattr(
        checker.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(
            command,
            1 if command[-1] in {"commit.cleanup", "commit.verbose"} else 0,
            "" if command[-1] in {"commit.cleanup", "commit.verbose"} else "// \n",
        ),
    )
    assert checker.scan_commit_message(
        "Public summary\n// See /home/example/private.md\n//See /home/example/private.md\n",
        git_command=["git", "commit"],
    )
    assert capsys.readouterr().out == "3: /home/example/private.md\n"


@pytest.mark.parametrize(
    "encoding,bom",
    [
        ("utf-16-le", b"\xff\xfe"),
        ("utf-16-be", b"\xfe\xff"),
        ("utf-32-le", b"\xff\xfe\x00\x00"),
        ("utf-32-be", b"\x00\x00\xfe\xff"),
        ("utf-16-le", b""),
        ("utf-16-be", b""),
    ],
)
def test_staged_utf16_scans_whole_blob_including_unchanged_lines(repo, encoding, bom):
    path = repo / "notes.txt"
    path.write_bytes(
        bom + "/home/example/private.md\nPublic summary\n".encode(encoding)
    )
    _git(repo, "add", path.name)
    _git(
        repo,
        "-c",
        "user.name=Example",
        "-c",
        "user.email=example@example.com",
        "commit",
        "-qm",
        "Initial fixture",
    )
    path.write_bytes(
        bom + "/home/example/private.md\nUpdated summary\n".encode(encoding)
    )
    _git(repo, "add", path.name)
    path.write_text("Public worktree hides staged leak\n")
    result = _cli("--staged", cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == "notes.txt:1: /home/example/private.md\n"


def test_staged_gitlink_checks_name_without_reading_missing_commit(repo):
    _git(repo, "update-index", "--add", "--cacheinfo", "160000", "a" * 40, "module")
    assert _cli("--staged", cwd=repo).returncode == 0
    _git(
        repo,
        "update-index",
        "--add",
        "--cacheinfo",
        "160000",
        "a" * 40,
        "scratchpad/example",
    )
    result = _cli("--staged", cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == "scratchpad/example: scratchpad/example\n"


def test_all_tracked_scans_gitlink_names_but_not_checkout_contents(repo):
    _git(
        repo,
        "update-index",
        "--add",
        "--cacheinfo",
        "160000,0123456789abcdef0123456789abcdef01234567,vendor/lib",
    )
    (repo / "vendor" / "lib").mkdir(parents=True)
    assert _cli("--all-tracked", cwd=repo).returncode == 0
    _git(
        repo,
        "update-index",
        "--add",
        "--cacheinfo",
        "160000,0123456789abcdef0123456789abcdef01234567,scratchpad/example",
    )
    (repo / "scratchpad" / "example").mkdir(parents=True)
    result = _cli("--all-tracked", cwd=repo)
    assert result.returncode == 1
    assert "scratchpad/example" in result.stdout


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
    tracked.write_text(".sisyphus/plans/example.md\nremoved\npublic\n")
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
    tracked.write_text(".sisyphus/plans/example.md\npublic\nnew summary\n")
    _git(repo, "add", tracked.name)
    tracked.write_text("/home/example/unstaged\n")
    assert _cli("--staged", cwd=repo).returncode == 0

    tracked.write_text(
        ".sisyphus/plans/example.md\npublic\n/tmp/claude-example/example/log\n"
    )
    _git(repo, "add", tracked.name)
    tracked.write_text("Public worktree hides staged leak\n")
    result = _cli("--staged", cwd=repo)
    assert result.returncode == 1
    assert result.stdout == "notes with spaces.md:3: /tmp/claude-example/example/log\n"

    _git(repo, "rm", "-f", tracked.name)
    assert _cli("--staged", cwd=repo).returncode == 0


def test_staged_new_file_allows_gitignore_and_checker_fixtures(repo):
    (repo / ".gitignore").write_text(".sisyphus" "/\n")  # path-fixture
    fixture = repo / "tests" / "test_check_local_paths.py"
    fixture.parent.mkdir()
    fixture.write_text("/home/example/fixture\n")
    _git(repo, "add", ".")
    assert _cli("--staged", cwd=repo).returncode == 0
    assert _cli("--all-tracked", cwd=repo).returncode == 0
    (repo / ".gitignore").write_text(".sisyphus/\n/home/example/ignored\n")
    _git(repo, "add", ".gitignore")
    assert _cli("--staged", cwd=repo).returncode == 1


@pytest.mark.parametrize("mode", ("--files", "--all-tracked", "--staged"))
@pytest.mark.parametrize(
    "beside", ["", "/home/example/fixture ", "example ", "# path-fixture "]
)
def test_checker_test_exemption_rejects_non_fixture_home(repo, mode, beside):
    fixture = repo / "tests" / "test_check_local_paths.py"
    fixture.parent.mkdir()
    # Construct a non-fixture user without publishing a real home path.
    path = "/home/" + "example"[::-1] + "/client/private.md"
    fixture.write_text(beside + path + "\n")
    _git(repo, "add", ".")
    args = (mode, fixture) if mode == "--files" else (mode,)
    result = _cli(*args, cwd=repo)
    assert result.returncode == 1, result.stderr
    assert result.stdout == f"tests/test_check_local_paths.py:1: {path}\n"


@pytest.mark.skipif(os.name != "posix", reason="symlink creation needs privileges")
def test_tracked_symlink_scans_target_without_reading_external_file(repo):
    (repo / "link").symlink_to("/home/example/private.md")
    _git(repo, "add", "link")
    result = _cli("--all-tracked", cwd=repo)
    assert result.returncode == 1
    assert result.stdout == "link:1: /home/example/private.md\n"
