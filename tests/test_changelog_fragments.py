"""Changelog fragment check and checkpoint assembly regression tests."""

import importlib.util
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "changelog_fragments", ROOT / "scripts" / "changelog_fragments.py"
)
assert SPEC and SPEC.loader
fragments = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fragments)

CHANGELOG = """\
# DART Changelog

## DART 7

### [DART 7.0.0 (Unreleased)](https://github.com/dartsim/dart/milestone/105)

* Breaking Changes

  * Breaking entry.
    [#100](https://github.com/dartsim/dart/pull/100)

* Python

  * Earlier Python entry.

## DART 6

### [DART 6.20.1 (2026-10-18)](https://github.com/dartsim/dart/milestone/106?closed=1)

* Tooling and Docs

  * Released entry.
"""


def fragment(category, *body):
    return "\n".join([f"Section: {category}", "", *body]) + "\n"


def link(number, comma=""):
    return f"    [#{number}](https://github.com/dartsim/dart/pull/{number}){comma}"


# Fragment name -> introducing PR for the fixture below; real Git is covered
# by the `git_repo` tests.
ORIGINS = {}


@pytest.fixture(autouse=True)
def no_ci_branch_context(monkeypatch):
    for name in ("GITHUB_BASE_REF", "GITHUB_REF_NAME"):
        monkeypatch.delenv(name, raising=False)


@pytest.fixture
def repo(tmp_path, monkeypatch):
    (tmp_path / "CHANGELOG.md").write_text(CHANGELOG, encoding="utf-8")
    (tmp_path / "changelog.d").mkdir()
    shutil.copy(ROOT / "changelog.d" / "README.md", tmp_path / "changelog.d")
    ORIGINS.clear()
    monkeypatch.setattr(fragments, "fragment_changes", lambda root: [])
    monkeypatch.setattr(
        fragments,
        "fragment_origins",
        lambda root, found: (
            {f.path.name: ORIGINS[f.path.name] for f in found},
            [],
        ),
    )
    return tmp_path


def write(repo, name, text, origin=None):
    (repo / "changelog.d" / name).write_text(text, encoding="utf-8")
    if origin:
        ORIGINS[name] = origin


def run(repo, command, capsys):
    code = fragments.main([command, "--root", str(repo)])
    out, err = capsys.readouterr()
    return code, out + err


def test_check_accepts_fragments_in_changelog_style(repo, capsys):
    write(repo, "linked.md", fragment("Python", "  * Entry.", link(5)))
    write(repo, "draft.md", fragment("Breaking Changes", "  * No PR yet,", "    ok."))
    write(
        repo,
        "new-category.md",
        fragment(
            "GUI",
            "  * Entry with a list:",
            "",
            "    * item,",
            "      wrapped.",
            "",
            "    Closing paragraph:",
            link(7, ","),
            "    [#8](https://github.com/dartsim/dart/issues/8),",
            link(9),
        ),
    )
    write(repo, "bom.md", "\ufeff" + fragment("Python", "  * Saved with a BOM."))
    write(repo, ".draft.md.swp", "editor swap file")
    write(repo, ".draft.md.swo", "editor swap file")

    assert run(repo, "check", capsys) == (0, "4 changelog fragment(s) OK\n")


@pytest.mark.parametrize("name", (".hidden.md", ".HIDDEN.MD", ".draft.md~"))
def test_check_rejects_hidden_fragments(repo, capsys, name):
    write(repo, name, fragment("Python", "  * Entry.", link(5)))

    code, output = run(repo, "check", capsys)

    assert code == 1
    assert f"changelog.d/{name}: " in output


def test_check_reports_every_problem(repo, capsys, monkeypatch):
    bad = {
        "no-section.md": ("  * Entry.\n", ":1: start with `Section: <Category>`"),
        "no-blank.md": ("Section: Python\n  * Entry.\n", ":2: follow the Section"),
        "two-blanks.md": ("Section: Python\n\n\n  * Entry.\n", ":2: follow the"),
        "old-category.md": (
            fragment("Tooling and Docs", "  * Entry."),
            ":1: unknown category `Tooling and Docs`",
        ),
        "bullet.md": (fragment("Python", "* Entry."), ":3: start the entry with"),
        "two-entries.md": (
            fragment("Python", "  * One.", "  * Two."),
            ":4: put each entry in its own fragment",
        ),
        "indent.md": (
            fragment("Python", "  * Entry", "  wrapped."),
            ":4: indent continuation lines 4 spaces",
        ),
        "mismatch.md": (
            fragment(
                "Python", "  * E.", "    [#5](https://github.com/dartsim/dart/pull/6)"
            ),
            ":4: write each link as",
        ),
        "no-comma.md": (fragment("Python", "  * E.", link(5), link(6)), ":4: write"),
        "last-comma.md": (fragment("Python", "  * E.", link(5, ",")), ":4: write"),
        "split-links.md": (
            fragment("Python", "  * E.", link(1, ","), "", link(30)),
            ":4: keep the links together",
        ),
        "indented-link.md": (
            fragment("Python", "  * E.", " " + link(5)),
            ":4: write each link as `    [#N](https://github.com/dartsim/dart/pull/N)`",
        ),
        "indented-split-links.md": (
            fragment("Python", "  * E.", "  " + link(1, ","), "", link(30)),
            ":4: keep the links together",
        ),
        "spaces.md": (fragment("Python", "  * Entry. "), ":3: remove trailing"),
        "Bad_Name.md": (fragment("Python", "  * Entry."), ": use a lowercase kebab"),
        "notes.txt": ("Section: Python\n", ": fragments are `<topic>.md` files"),
    }
    for name, (text, _) in bad.items():
        write(repo, name, text)
    utf16 = repo / "changelog.d" / "utf-16.md"
    utf16.write_text(fragment("Python", "  * Entry."), encoding="utf-16")

    code, output = run(repo, "check", capsys)

    assert code == 1
    missing = [
        f"changelog.d/{name}{message}"
        for name, (_, message) in bad.items()
        if f"changelog.d/{name}{message}" not in output
    ]
    assert "changelog.d/utf-16.md: save the fragment as UTF-8" in output

    # Reserved names cannot be written on Windows; exercise the parser without I/O.
    for name in (
        "con",
        "prn",
        "aux",
        "nul",
        *(f"com{i}" for i in range(1, 10)),
        *(f"lpt{i}" for i in range(1, 10)),
    ):
        with monkeypatch.context() as patch:
            patch.setattr(
                Path, "read_text", lambda *a, **kw: fragment("Python", "  * E.")
            )
            _, errors = fragments.parse_fragment(repo / "changelog.d" / f"{name}.md")
        message = f"changelog.d/{name}.md: choose another name; `{name}` is reserved on Windows"
        if message not in errors:
            missing.append(message)
    assert not missing


def test_assemble_moves_fragments_in_pr_order(repo, capsys):
    write(
        repo,
        "a-python.md",
        fragment("Python", "  * Later Python entry.", link(300)),
        300,
    )
    write(
        repo,
        "b-python.md",
        fragment(
            "Python",
            "  * Early Python entry,",
            "    wrapped.",
            link(200, ","),
            link(301),
        ),
        200,
    )
    write(repo, "breaking.md", fragment("Breaking Changes", "  * New.", link(150)), 150)
    write(repo, "gui.md", fragment("GUI", "  * GUI entry.", link(250)) + "\n", 250)
    write(
        repo,
        "behavior.md",
        fragment("Behavior Changes", "  * Behavior.", link(260)),
        260,
    )

    code, output = run(repo, "assemble", capsys)

    assert code == 0
    assert "changelog.d/b-python.md -> Python (#200)\n" in output
    assert "Created category `GUI`\n" in output
    assert (repo / "CHANGELOG.md").read_text(encoding="utf-8") == CHANGELOG.replace(
        """\
    [#100](https://github.com/dartsim/dart/pull/100)

* Python

  * Earlier Python entry.
""",
        """\
    [#100](https://github.com/dartsim/dart/pull/100)

  * New.
    [#150](https://github.com/dartsim/dart/pull/150)

* Behavior Changes

  * Behavior.
    [#260](https://github.com/dartsim/dart/pull/260)

* Python

  * Earlier Python entry.

  * Early Python entry,
    wrapped.
    [#200](https://github.com/dartsim/dart/pull/200),
    [#301](https://github.com/dartsim/dart/pull/301)

  * Later Python entry.
    [#300](https://github.com/dartsim/dart/pull/300)

* GUI

  * GUI entry.
    [#250](https://github.com/dartsim/dart/pull/250)
""",
    )
    assert [path.name for path in (repo / "changelog.d").iterdir()] == ["README.md"]

    assembled = (repo / "CHANGELOG.md").read_text(encoding="utf-8")
    assert run(repo, "assemble", capsys) == (0, "No changelog fragments to assemble\n")
    assert (repo / "CHANGELOG.md").read_text(encoding="utf-8") == assembled


def test_assemble_requires_pr_links(repo, capsys):
    write(repo, "linked.md", fragment("Python", "  * Linked.", link(5)), 5)
    write(repo, "draft.md", fragment("Python", "  * Draft."), 6)
    issue = "    [#4](https://github.com/dartsim/dart/issues/4)"
    write(repo, "issue-only.md", fragment("Python", "  * Issue only.", issue), 4)
    write(repo, "related.md", fragment("Python", "  * Related.", link(5)), 7)

    code, output = run(repo, "assemble", capsys)

    assert code == 1
    for name in ("draft.md", "issue-only.md", "related.md"):
        assert f"changelog.d/{name}: end the entry with its own PR link line" in output
        assert f"--format=%s -- changelog.d/{name}` ends with the PR" in output
    assert "linked.md" not in output
    assert (repo / "CHANGELOG.md").read_text(encoding="utf-8") == CHANGELOG
    assert (repo / "changelog.d" / "linked.md").is_file()


def test_assemble_stays_above_a_released_heading(repo, capsys):
    # After a release, the new unreleased heading sits directly above the
    # released one, which has the same categories.
    pending = (
        "### [DART 7.1.0 (Unreleased)](https://github.com/dartsim/dart/milestone/107)"
        "\n\n* Python\n"
    )
    released = CHANGELOG.replace("(Unreleased)", "(2026-12-01)").replace(
        "  * Earlier Python entry.", f"  * New Python entry.\n{link(5)}"
    )
    before = released.replace("### ", f"{pending}\n### ", 1)
    (repo / "CHANGELOG.md").write_text(before, encoding="utf-8")
    write(repo, "python.md", fragment("Python", "  * New Python entry.", link(5)), 5)
    write(repo, "gui.md", fragment("GUI", "  * New GUI entry.", link(6)), 6)

    assert run(repo, "assemble", capsys)[0] == 0
    assert (repo / "CHANGELOG.md").read_text(encoding="utf-8") == before.replace(
        pending,
        f"{pending}\n  * New Python entry.\n{link(5)}\n\n"
        f"* GUI\n\n  * New GUI entry.\n{link(6)}\n",
    )


def test_assemble_keeps_everything_when_the_write_fails(repo, capsys, monkeypatch):
    write(repo, "linked.md", fragment("Python", "  * Linked.", link(5)), 5)
    write_text = fragments.Path.write_text

    def fail_midway(self, text, *args, **kwargs):
        write_text(self, text[:10], *args, **kwargs)
        raise OSError("disk full")

    monkeypatch.setattr(fragments.Path, "write_text", fail_midway)

    assert run(repo, "assemble", capsys)[0] == 1
    assert (repo / "CHANGELOG.md").read_text(encoding="utf-8") == CHANGELOG
    assert (repo / "changelog.d" / "linked.md").is_file()
    assert not list(repo.glob(".CHANGELOG*"))


def test_assemble_retry_after_a_failed_deletion_is_safe(repo, capsys, monkeypatch):
    body = ["  * Linked.", link(5)]
    write(repo, "linked.md", fragment("Python", *body), 5)
    path = repo / "changelog.d" / "linked.md"
    unlink = Path.unlink

    def fail_once(self, *args, **kwargs):
        if self == path:
            raise PermissionError("fragment is locked")
        return unlink(self, *args, **kwargs)

    with monkeypatch.context() as patch:
        patch.setattr(Path, "unlink", fail_once)
        code, output = run(repo, "assemble", capsys)
    assert code == 1
    assert "fragment is locked" in output
    assert path.is_file()
    assembled = (repo / "CHANGELOG.md").read_text(encoding="utf-8")

    code, output = run(repo, "assemble", capsys)

    assert code == 0
    after = (repo / "CHANGELOG.md").read_text(encoding="utf-8")
    assert after.count("\n".join(body)) == 1
    assert after == assembled
    assert not path.exists()
    assert "entry already present" in output
    assert "deleting fragment only" in output


class GitRepo:
    """A real repository holding CHANGELOG.md and an empty changelog.d/."""

    def __init__(self, root):
        self.root = root
        (root / "empty-hooks").mkdir()
        (root / "CHANGELOG.md").write_text(CHANGELOG, encoding="utf-8")
        (root / "changelog.d").mkdir()
        self.git("init", "-b", "main")
        self.git("add", "CHANGELOG.md")
        self.git("commit", "-m", "Initial changelog")

    def git(self, *args):
        return subprocess.run(
            [
                "git",
                "-c",
                "user.name=Test",
                "-c",
                "user.email=test@example.com",
                "-c",
                f"core.hooksPath={self.root / 'empty-hooks'}",
                *args,
            ],
            cwd=self.root,
            check=True,
            capture_output=True,
            text=True,
        )

    def commit(self, subject):
        self.git("add", "-A", "changelog.d", "CHANGELOG.md")
        self.git("commit", "-m", subject)

    def add(self, name, pr, *links, category="Python", entry=None):
        refs = [link(n, "," if i < len(links) - 1 else "") for i, n in enumerate(links)]
        text = fragment(category, f"  * {entry or name}.", *refs)
        write(self.root, name, text)
        self.commit(f"Add {name} (#{pr})")
        return text

    def changelog(self):
        return (self.root / "CHANGELOG.md").read_text(encoding="utf-8")

    def fragment_names(self):
        return sorted(path.name for path in (self.root / "changelog.d").iterdir())


@pytest.fixture
def git_repo(tmp_path):
    return GitRepo(tmp_path)


def test_assemble_requires_committed_fragments(git_repo, capsys):
    repo = git_repo.root
    text = fragment("Python", "  * Linked.", link(5))
    write(repo, "linked.md", text)
    other = fragment("GUI", "  * Other.", link(6))
    write(repo, "other.md", other)

    code, output = run(repo, "assemble", capsys)

    assert code == 1
    assert "changelog.d/linked.md" in output
    assert "changelog.d/other.md" in output
    assert "commit or remove" in output
    assert git_repo.changelog() == CHANGELOG
    assert (repo / "changelog.d" / "linked.md").read_text(encoding="utf-8") == text
    assert (repo / "changelog.d" / "other.md").read_text(encoding="utf-8") == other
    assert not list(repo.glob(".CHANGELOG*"))

    git_repo.commit("Add fragments (#5)")
    write(repo, "linked.md", text.replace("Linked.", "Uncommitted edit."))
    for staged in (False, True):
        if staged:
            git_repo.git("add", "changelog.d")
        code, output = run(repo, "assemble", capsys)
        assert code == 1
        assert "changelog.d/linked.md" in output
        assert "commit or remove" in output
        assert git_repo.changelog() == CHANGELOG

    # One PR may introduce several fragments; each must link that PR.
    write(repo, "linked.md", fragment("Python", "  * Linked.", link(5)))
    write(repo, "other.md", fragment("GUI", "  * Other.", link(5, ","), link(6)))
    git_repo.commit("Fix fragment links (#5)")
    assert run(repo, "assemble", capsys)[0] == 0
    assert git_repo.changelog().count("  * Linked.") == 1
    assert git_repo.fragment_names() == []


def test_assemble_refuses_a_dirty_changelog(git_repo, capsys):
    repo = git_repo.root
    text = git_repo.add("linked.md", 5, 5)
    for staged in (False, True):
        edited = git_repo.changelog().replace("Earlier", f"Edited {staged}")
        (repo / "CHANGELOG.md").write_text(edited, encoding="utf-8")
        if staged:
            git_repo.git("add", "CHANGELOG.md")
        code, output = run(repo, "assemble", capsys)
        assert code == 1
        assert "CHANGELOG.md: commit or remove this change" in output
        assert git_repo.changelog() == edited
        assert (repo / "changelog.d" / "linked.md").read_text(encoding="utf-8") == text
        assert not list(repo.glob(".CHANGELOG*"))
        git_repo.git("reset", "--hard", "-q")


@pytest.mark.parametrize("pattern", ["changelog.d/linked.md", "changelog.d/"])
def test_assemble_refuses_ignored_local_drafts(git_repo, capsys, pattern):
    repo = git_repo.root
    git_repo.add("linked.md", 5, 5)
    (repo / "changelog.d" / "linked.md").unlink()
    git_repo.commit("Remove linked.md (#6)")
    (repo / ".git" / "info" / "exclude").write_text(f"{pattern}\n", encoding="utf-8")
    draft = fragment("Python", "  * Edited draft.", link(5))
    write(repo, "linked.md", draft)

    code, output = run(repo, "assemble", capsys)

    assert code == 1
    assert "changelog.d/" in output
    assert "commit or remove" in output
    assert git_repo.changelog() == CHANGELOG
    assert (repo / "changelog.d" / "linked.md").read_bytes() == draft.encode("utf-8")


def test_failed_deletion_needs_a_committed_checkpoint_before_retry(
    git_repo, capsys, monkeypatch
):
    repo = git_repo.root
    git_repo.add("first.md", 5, 5)
    git_repo.add("second.md", 6, 6, category="GUI")
    locked = repo / "changelog.d" / "second.md"
    unlink = Path.unlink

    def fail_second(self, *args, **kwargs):
        if self == locked:
            raise PermissionError("fragment is locked")
        return unlink(self, *args, **kwargs)

    with monkeypatch.context() as patch:
        patch.setattr(Path, "unlink", fail_second)
        code, output = run(repo, "assemble", capsys)
    assert code == 1
    assert "fragment is locked" in output
    assert "review and commit" in output
    assembled = git_repo.changelog()
    assert assembled != CHANGELOG
    names = git_repo.fragment_names()

    # The partial output is dirty: refuse, change nothing.
    code, output = run(repo, "assemble", capsys)
    assert code == 1
    assert "commit or remove this change" in output
    assert git_repo.changelog() == assembled
    assert git_repo.fragment_names() == names

    git_repo.commit("Checkpoint changelog")
    code, output = run(repo, "assemble", capsys)

    assert code == 0
    assert "deleting fragment only" in output
    assert git_repo.changelog() == assembled
    assert git_repo.changelog().count("  * first.") == 1
    assert git_repo.changelog().count("  * second.") == 1
    assert git_repo.fragment_names() == []


def test_assemble_requires_the_originating_pr_link(git_repo, capsys):
    repo = git_repo.root
    git_repo.add("linked.md", 300, 5)

    code, output = run(repo, "assemble", capsys)

    assert code == 1
    assert "changelog.d/linked.md: end the entry with its own PR link line" in output
    assert "[#300](https://github.com/dartsim/dart/pull/300)" in output
    assert git_repo.changelog() == CHANGELOG
    assert git_repo.fragment_names() == ["linked.md"]

    issue = "    [#300](https://github.com/dartsim/dart/issues/300)"
    write(repo, "linked.md", fragment("Python", "  * linked.md.", link(5, ","), issue))
    git_repo.commit("Link issue (#999)")
    assert run(repo, "assemble", capsys)[0] == 1

    write(
        repo, "linked.md", fragment("Python", "  * linked.md.", link(5, ","), link(300))
    )
    git_repo.commit("Link its PR (#999)")
    code, output = run(repo, "assemble", capsys)

    assert code == 0
    assert "changelog.d/linked.md -> Python (#300)\n" in output
    assert git_repo.fragment_names() == []


def test_assemble_orders_by_the_originating_pr(git_repo, capsys):
    repo = git_repo.root
    # Alphabetical, smallest-linked-PR, and origin order all differ.
    git_repo.add("a-late.md", 300, 5, 300, entry="Late")
    git_repo.add("z-early.md", 200, 200, entry="Early")

    code, output = run(repo, "assemble", capsys)

    assert code == 0
    assert "changelog.d/a-late.md -> Python (#300)\n" in output
    assert "(#5)" not in output
    assert git_repo.changelog().index("  * Early.") < git_repo.changelog().index(
        "  * Late."
    )


def test_assemble_keeps_the_introducing_pr_across_edits_and_renames(git_repo, capsys):
    repo = git_repo.root
    git_repo.add("old-name.md", 300, 300)
    write(repo, "old-name.md", fragment("Python", "  * Edited.", link(300)))
    git_repo.commit("Reword entry (#999)")
    git_repo.git("mv", "changelog.d/old-name.md", "changelog.d/new-name.md")
    git_repo.commit("Rename fragment (#1000)")

    code, output = run(repo, "assemble", capsys)

    assert code == 0
    assert "changelog.d/new-name.md -> Python (#300)\n" in output


@pytest.mark.parametrize(
    "subject",
    ("Add fragment", "Add fragment (#0)", "Add fragment (#12) now", "Merge pull #12"),
)
def test_assemble_rejects_unclear_provenance(git_repo, capsys, subject):
    repo = git_repo.root
    write(repo, "linked.md", fragment("Python", "  * Linked.", link(12)))
    git_repo.commit(subject)

    code, output = run(repo, "assemble", capsys)

    assert code == 1
    assert (
        "changelog.d/linked.md: the commit that added it must have a subject" in output
    )
    assert git_repo.changelog() == CHANGELOG
    assert git_repo.fragment_names() == ["linked.md"]


def test_assemble_rejects_a_shallow_clone(git_repo, tmp_path_factory, capsys):
    git_repo.add("linked.md", 5, 5)
    git_repo.add("other.md", 6, 6)
    clone = tmp_path_factory.mktemp("shallow")
    subprocess.run(
        ["git", "clone", "-q", "--depth=1", git_repo.root.as_uri(), str(clone / "c")],
        check=True,
        capture_output=True,
    )

    code, output = run(clone / "c", "assemble", capsys)

    assert code == 1
    assert "shallow" in output
    assert "git fetch --unshallow" in output
    assert (clone / "c" / "changelog.d" / "linked.md").is_file()


def test_assemble_rejects_a_missing_history(tmp_path, capsys):
    (tmp_path / "CHANGELOG.md").write_text(CHANGELOG, encoding="utf-8")
    (tmp_path / "changelog.d").mkdir()
    write(tmp_path, "linked.md", fragment("Python", "  * Linked.", link(5)))

    code, output = run(tmp_path, "assemble", capsys)

    assert code == 1
    assert "Git checkout" in output
    assert (tmp_path / "changelog.d" / "linked.md").is_file()


@pytest.mark.parametrize(
    ("env", "branch", "rejected"),
    (
        ({"GITHUB_BASE_REF": "release-6.20"}, "main", True),
        ({"GITHUB_BASE_REF": "main"}, "release-6.20", False),
        ({"GITHUB_BASE_REF": "feature/stack-parent"}, "release-6.20", False),
        ({"GITHUB_REF_NAME": "release-6.20"}, None, True),
        ({}, "release-6.20", True),
        ({}, "main", False),
        ({}, "topic", False),
    ),
)
def test_release_branches_have_no_fragments(
    git_repo, capsys, monkeypatch, env, branch, rejected
):
    repo = git_repo.root
    git_repo.add("linked.md", 5, 5)
    if branch is None:
        git_repo.git("checkout", "-q", "--detach")
    elif branch != "main":
        git_repo.git("checkout", "-q", "-b", branch)
    for name, value in env.items():
        monkeypatch.setenv(name, value)

    for command in ("check", "assemble"):
        code, output = run(repo, command, capsys)
        assert (code == 1) == rejected, output
        if rejected:
            assert "has no fragments" in output
    assert (repo / "changelog.d" / "linked.md").exists() == rejected
    if rejected:
        assert git_repo.changelog() == CHANGELOG


def test_a_release_branch_without_fragments_is_fine(git_repo, capsys):
    git_repo.git("checkout", "-q", "-b", "release-6.20")

    assert run(git_repo.root, "check", capsys) == (0, "0 changelog fragment(s) OK\n")
    assert run(git_repo.root, "assemble", capsys) == (
        0,
        "No changelog fragments to assemble\n",
    )


@pytest.mark.parametrize(
    "failure", (FileNotFoundError("git unavailable"), "not a git repository")
)
def test_assemble_reports_git_errors(tmp_path, capsys, monkeypatch, failure):
    (tmp_path / "CHANGELOG.md").write_text(CHANGELOG, encoding="utf-8")
    (tmp_path / "changelog.d").mkdir()
    write(tmp_path, "linked.md", fragment("Python", "  * Linked.", link(5)))

    def fail_git(*args, **kwargs):
        if isinstance(failure, OSError):
            raise failure
        return subprocess.CompletedProcess(args, 128, stdout="", stderr=failure)

    monkeypatch.setattr(subprocess, "run", fail_git)

    code, output = run(tmp_path, "assemble", capsys)

    assert code == 1
    assert ("install Git" if isinstance(failure, OSError) else "Git checkout") in output
    assert (tmp_path / "CHANGELOG.md").read_text(encoding="utf-8") == CHANGELOG
    assert (tmp_path / "changelog.d" / "linked.md").is_file()


def test_assemble_refuses_a_released_first_heading(repo, capsys):
    released = CHANGELOG.replace("(Unreleased)", "(2026-11-01)")
    (repo / "CHANGELOG.md").write_text(released, encoding="utf-8")
    write(repo, "linked.md", fragment("Python", "  * Linked.", link(5)), 5)

    code, output = run(repo, "assemble", capsys)

    assert code == 1
    assert "is released; to include these fragments in it, assemble" in output
    assert (repo / "CHANGELOG.md").read_text(encoding="utf-8") == released


def test_readme_documents_the_category_order():
    readme = (ROOT / "changelog.d" / "README.md").read_text(encoding="utf-8")
    assert ", ".join(fragments.CATEGORY_ORDER) in " ".join(readme.split())


def test_assemble_uses_any_category_under_the_unreleased_heading(repo, capsys):
    # The Section names a category bullet under the unreleased heading, even
    # one outside CATEGORY_ORDER; a CRLF fragment parses like an LF one.
    before = CHANGELOG.replace("## DART 6", "* Tests\n\n  * Test entry.\n\n## DART 6")
    (repo / "CHANGELOG.md").write_text(before, encoding="utf-8")
    text = fragment("Tests", "  * New test entry.", link(5)).replace("\n", "\r\n")
    (repo / "changelog.d" / "tests.md").write_bytes(text.encode())
    ORIGINS["tests.md"] = 5

    assert run(repo, "check", capsys) == (0, "1 changelog fragment(s) OK\n")
    assert run(repo, "assemble", capsys)[0] == 0
    assert (repo / "CHANGELOG.md").read_text(encoding="utf-8") == before.replace(
        "  * Test entry.\n", f"  * Test entry.\n\n  * New test entry.\n{link(5)}\n"
    )


def test_check_accepts_prose_that_starts_a_line_with_a_link(repo, capsys):
    issue = "[#3056](https://github.com/dartsim/dart/issues/3056)"
    write(
        repo,
        "prose.md",
        fragment(
            "Python",
            "  * Fix the case reported in",
            f"    {issue}, which",
            "    crashed.",
            link(5),
        ),
    )

    assert run(repo, "check", capsys) == (0, "1 changelog fragment(s) OK\n")


@pytest.mark.parametrize("command", ("check", "assemble"))
def test_accepts_link_led_prose_adjacent_to_reference(repo, capsys, command):
    body = [
        "  * Fix the case reported in",
        "    [#3056](https://github.com/dartsim/dart/issues/3056).",
        link(5),
    ]
    write(repo, "prose.md", fragment("Python", *body), 5)

    assert run(repo, command, capsys)[0] == 0
    if command == "assemble":
        assert (repo / "CHANGELOG.md").read_text(encoding="utf-8") == CHANGELOG.replace(
            "  * Earlier Python entry.\n",
            "  * Earlier Python entry.\n\n" + "\n".join(body) + "\n",
        )
        assert not (repo / "changelog.d" / "prose.md").exists()
