"""Tests for DART's deterministic AI infrastructure checks."""

import ast
import copy
import importlib.util
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


infra = _load(
    "check_ai_infrastructure", ROOT / "scripts" / "check_ai_infrastructure.py"
)
hook = _load("check_agent_hook", ROOT / "scripts" / "check_agent_hook.py")
bridge = _load("pretool_guard_bridge", ROOT / "scripts" / "pretool_guard_bridge.py")
setup = _load("setup_ai", ROOT / "scripts" / "setup_ai.py")


def test_repository_ai_infrastructure_is_valid():
    assert infra.run_checks(ROOT) == []


def test_repository_rejects_missing_release_target_row(monkeypatch):
    owner = ROOT / "docs/onboarding/release-management.md"
    text = owner.read_text(encoding="utf-8")
    row = next(line for line in text.splitlines() if line.startswith("| `"))
    original_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if path == owner:
            return text.replace(row, "", 1)
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)

    assert any("Release Target" in error for error in infra.run_checks(ROOT))


def test_inactive_cpp_test_policies_name_tracked_sources_and_owners():
    for relative, policy in infra.APPROVED_INACTIVE_CPP_TESTS.items():
        assert (ROOT / relative).is_file()
        assert (ROOT / policy["owner"]).is_file()


def test_static_ikfast_test_has_an_explicit_inactive_policy():
    assert infra.APPROVED_INACTIVE_CPP_TESTS["tests/integration/test_IkFast.cpp"] == {
        "owner": "tests/integration/CMakeLists.txt",
        "command": "dart_add_test",
        "scopes": (
            "if:TARGET dart-utils-urdf",
            "if:BUILD_SHARED_LIBS",
        ),
        "cache": {"BUILD_SHARED_LIBS": "OFF"},
    }


def test_repository_check_works_without_python_utf8_mode():
    env = {
        **os.environ,
        "LC_ALL": "C",
        "PYTHONCOERCECLOCALE": "0",
        "PYTHONUTF8": "0",
    }
    result = subprocess.run(
        [sys.executable, "scripts/check_ai_infrastructure.py", "--check"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_repository_sync_check_reads_utf8_without_python_utf8_mode():
    source = ROOT / ".claude" / "skills" / "dart-contribute" / "SKILL.md"
    assert "❌" in source.read_text(encoding="utf-8")
    env = {
        **os.environ,
        "LC_ALL": "C",
        "PYTHONCOERCECLOCALE": "0",
        "PYTHONUTF8": "0",
    }
    result = subprocess.run(
        [sys.executable, "scripts/sync_ai_commands.py", "--check"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
    )
    output = result.stdout + result.stderr

    assert "UnicodeDecodeError" not in output
    assert result.returncode == 0, output


def test_release_scenarios_are_exercisable():
    assert infra.exercise_scenarios(ROOT) == []


@pytest.fixture
def release_reference_target(tmp_path):
    path = tmp_path / "docs/onboarding/release-management.md"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "## Release Target\n\n"
        "| Branch | Phase | Next release |\n"
        "| ------ | ----- | ------------ |\n"
        "| `main` | Development | `7.0.0` |\n",
        encoding="utf-8",
    )
    return path


@pytest.mark.parametrize(
    ("relative", "text"),
    [
        ("AGENTS.md", "`main` develops DART 6.22.\n"),
        ("AGENTS.md", "`main` develops DART 7.0.\n"),
        ("docs/ai/README.md", "Next release: DART v12.0.0\n"),
        (
            "docs/ai/README.md",
            "When updating Python, prepare DART for the next release: 7.0.0.\n",
        ),
        (
            "docs/ai/README.md",
            "For DART, update nanobind before the upcoming release 7.0.0.\n",
        ),
        (
            "docs/ai/README.md",
            "When updating libdart, the next version is 7.0.0.\n",
        ),
        ("AGENTS.md", "Currently DART7.0\n"),
        ("AGENTS.md", "Currently release-12.0\n"),
        ("docs/ai/README.md", "Next release: DART 12.0.0\n"),
        ("docs/README.md", "| release-7.0 | Maintenance | 7.0.1 |\n"),
        ("docs/onboarding/contributing.md", "Use milestone DART 7.0.0\n"),
        (
            "docs/onboarding/contributing.md",
            "Release packaging targets release-7.0 directly.\n",
        ),
        ("GEMINI.md", "The next release is currently DART 6.22.\n"),
        ("AGENTS.md", "The next release is DART 6.22.0.\n"),
        ("docs/README.md", "Next release: **DART 6.22.0**\n"),
        ("docs/plans/dashboard.md", "The upcoming release will be DART 6.22.0.\n"),
        ("docs/ai/README.md", "Planned release = `DART 6.22.0`\n"),
        ("AGENTS.md", "Next release: `6.22.0`\n"),
        ("AGENTS.md", "Next release: 7.0.0\n"),
        ("AGENTS.md", "Prepare DART for the next release: 7.0.0\n"),
        ("AGENTS.md", "Prepare the Python-first DART for the next release: 7.0.0\n"),
        ("AGENTS.md", "For DART's Python API, the next release is 7.0.0.\n"),
        ("AGENTS.md", "Prepare branch-local DART for the next release: 7.0.0\n"),
        ("AGENTS.md", "For gz-physics, the next release is 7.0.0\n"),
        ("AGENTS.md", "When updating Python-first DART, the next version is 7.0.0\n"),
        ("AGENTS.md", "When updating nanobinder, the next version is 7.0.0\n"),
        ("AGENTS.md", "When updating support for Python, the next version is 7.8.0\n"),
        ("AGENTS.md", "Update gz-physics. Next release: 7.0.0\n"),
        ("AGENTS.md", "Update gz-physics! Next release: 7.0.0\n"),
        ("AGENTS.md", "Update gz-physics? Next release: 7.0.0\n"),
        ("AGENTS.md", "For gz-physics, the next DART release is 7.0.0\n"),
        ("AGENTS.md", "For gz-physics, the next release is DART 7.0.0\n"),
        ("docs/README.md", "The next release is 6.22.0.\n"),
        ("AGENTS.md", "Next version: DART 6.22.0\n"),
        ("AGENTS.md", "The next version is DART 6.22.0.\n"),
        ("docs/README.md", "Planned version = **DART 6.22.0**\n"),
        ("docs/plans/dashboard.md", "The upcoming version will be DART 6.22.0.\n"),
        ("docs/ai/README.md", "Next DART version: `6.22.0`\n"),
        ("docs/ai/README.md", "Next DART version: 8.0.0\n"),
        ("docs/ai/README.md", "Next DART release: v8.0.0\n"),
        ("docs/ai/README.md", "Next DART version: **v8.0.0**\n"),
        ("docs/ai/README.md", "Next DART version: DART 8.0.0\n"),
        ("docs/ai/README.md", "다음 DART 버전: 8.0.0\n"),
        ("AGENTS.md", "Next version: 6.22.0\n"),
        ("AGENTS.md", "Next version: 6.22.0rc1\n"),
        ("AGENTS.md", "Next version: 7.0.0rc1\n"),
        ("AGENTS.md", "Next version: DART7.0\n"),
        ("AGENTS.md", "Next version: release-7.0\n"),
        ("AGENTS.md", "Next version: v7.0.0\n"),
        ("docs/README.md", "**Next version**: **DART 6.22.0**\n"),
        ("docs/README.md", "**Next version:** ``6.22.0``\n"),
        ("docs/README.md", "``Next version`` = ``6.22.0``\n"),
        ("AGENTS.md", "Next version DART 6.22.0\n"),
        (
            ".codex/agents/dart_reviewer.toml",
            'description = "Review the DART 6.22 development branch."\n',
        ),
        (
            "docs/onboarding/building.md",
            "The current stable release is DART 6.21.0.\n",
        ),
        ("docs/plans/dashboard.md", "`main` develops DART 6.22.\n"),
        ("docs/background/README.md", "`main` develops DART 6.22.\n"),
        ("docs/design/README.md", "`main` develops DART 6.22.\n"),
        ("docs/ai/principles.md", "`main`, developing\nDART 6.22.\n"),
        ("docs/README.md", "`release-6.21` stabilizes DART 6.21.\n"),
        (
            "docs/onboarding/contributing.md",
            "Use the `DART 6.22.0`\nmilestone.\n",
        ),
        (
            ".claude/commands/dart-pr.md",
            "| `main` | `DART 6.22.0` |\n",
        ),
        ("AGENTS.md", "| `main` | Development | `6.22.0` |\n"),
        (
            ".claude/commands/dart-new-task.md",
            "| `release-6.21` | Stabilization | `6.21.0` |\n",
        ),
        (
            ".claude/skills/dart-contribute/SKILL.md",
            "| release-6.21 | Maintenance | 6.21.1 |\n",
        ),
        (
            ".claude/skills/dart-contribute/SKILL.md",
            'gh pr edit <PR#> --milestone "DART 6.22.0"\n',
        ),
        (
            ".claude/commands/dart-manage-pr.md",
            "Release packaging may target `release-6.21` directly.\n",
        ),
        (
            ".github/PULL_REQUEST_TEMPLATE.md",
            "- [ ] Milestone set (`DART 6.22.0` for `main`)\n",
        ),
        ("docs/onboarding/contributing.md", "Milestone: `DART 6.22.0`\n"),
        (
            "docs/readthedocs/index.rst",
            "The current stable release is DART 6.21.0.\n",
        ),
    ],
)
def test_release_references_reject_copied_routing(
    release_reference_target, tmp_path, relative, text
):
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    errors = []

    infra.check_release_references(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith(f"{relative}:1: resolve the release target")


@pytest.mark.parametrize(
    ("relative", "text"),
    [
        ("docs/readthedocs/index.rst", "DART 6.19.5 was released on 2026-10-04.\n"),
        ("docs/ai/README.md", "DART 6.20 collision backends\n"),
        ("docs/ai/README.md", "release-6.20 is the maintenance line.\n"),
        ("docs/ai/README.md", "DART 7.0 collision backends\n"),
        ("docs/ai/README.md", "release-7.0 is the maintenance line.\n"),
        ("docs/ai/README.md", "When updating nanobind, the next version is 3.1.0.\n"),
        ("docs/ai/README.md", "When updating libcurl, the next version is 7.86.0.\n"),
        (
            "docs/ai/README.md",
            "When updating libcurl from 7.85.0, the next version is 7.86.0.\n",
        ),
        (
            "docs/ai/README.md",
            "When updating gz-physics from version 7.7.0, the next version is "
            "7.8.0.\n",
        ),
        (
            "docs/ai/README.md",
            "When upgrading gz-physics from version 7.7.0 to version 7.8.0, the next "
            "version is 7.9.0.\n",
        ),
        (
            "docs/ai/README.md",
            "Use the release-6.20 pybind11 binder and select the milestone from the "
            "base branch.\n",
        ),
        ("docs/ai/README.md", "When updating gz-physics, the next version is 9.0.0.\n"),
        ("docs/ai/README.md", "When updating gz-physics, the next version is 7.8.0.\n"),
        (
            "docs/ai/README.md",
            "When updating gz-physics 7.7.0, the next version is v7.8.0.\n",
        ),
        ("docs/ai/README.md", "When updating EIGEN, the next version is 7.8.0.\n"),
        ("docs/ai/README.md", "Update gz-math, the next version is 7.8.0.\n"),
        (
            "docs/ai/README.md",
            "When upgrading to gz-cmake, the next version is 7.8.0.\n",
        ),
        ("docs/ai/README.md", "Upgrade Python, the next version is 7.8.0.\n"),
        ("docs/ai/README.md", "Bump gz-sim, the next version is 7.8.0.\n"),
        ("docs/ai/README.md", "When bumping to Python, the next version is 7.8.0.\n"),
        ("docs/ai/README.md", "When updating gz-sim, the next version is 8.0.0.\n"),
        ("docs/ai/README.md", "The next version is 16.0.\n"),
        (
            "docs/readthedocs/dart/developer_guide/build.rst",
            "DART 6.20 requires C++17 and the following dependency baselines.\n",
        ),
        (
            "docs/ai/README.md",
            "Python tutorials were introduced in DART 6.21.\n",
        ),
        ("docs/ai/README.md", "| Python tutorials | DART 6.21.0 |\n"),
        ("docs/ai/README.md", "| C++17 baseline | DART 6.20.0 |\n"),
        (
            "docs/onboarding/contributing.md",
            "The historical milestone was `DART 6.20.0`.\n",
        ),
        (
            "docs/onboarding/ci-cd.md",
            "On release-6.20 these lanes currently report the known DART 6.20 "
            "Gazebo regressions from issue #3056.\n",
        ),
        (
            "docs/plans/archive.md",
            "Archived decision: `main` develops DART 6.20.\n",
        ),
        ("CHANGELOG.md", "DART 6.21.0 milestone: Python tutorials.\n"),
        ("docs/ai/README.md", "The next version was DART 6.20.0 in that audit.\n"),
        ("docs/ai/README.md", "DART 6.20.0 was the next version at that time.\n"),
        (
            "docs/onboarding/building.md",
            "The next version requires C++17, as DART 6.20 did.\n",
        ),
    ],
)
def test_release_references_allow_history_and_dependency_floors(
    release_reference_target, tmp_path, relative, text
):
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    errors = []

    infra.check_release_references(tmp_path, errors)

    assert errors == []


@pytest.mark.parametrize(
    "rows",
    [
        (
            "| `main` | Development | `7.0.0` |",
            "| `main` | Development | `7.1.0` |",
        ),
        (
            "| `release-6.20` | Stabilization | `6.20.0` |",
            "| `release-6.20` | Maintenance | `6.20.1` |",
        ),
    ],
)
def test_release_references_accept_release_metadata_only_rollover(
    tmp_path, rows, monkeypatch
):
    monkeypatch.delenv("GITHUB_BASE_REF", raising=False)
    monkeypatch.delenv("GITHUB_REF_NAME", raising=False)
    monkeypatch.delenv("GITHUB_EVENT_PATH", raising=False)
    sources = [
        *infra.release_reference_paths(ROOT),
        ROOT / "docs/onboarding/release-management.md",
        ROOT / "CHANGELOG.md",
        ROOT / "package.xml",
        ROOT / "pixi.toml",
    ]
    for source in sources:
        if source.is_file():
            target = tmp_path / source.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    owner = tmp_path / "docs/onboarding/release-management.md"
    text = owner.read_text(encoding="utf-8")
    target_row = next(line for line in text.splitlines() if line.startswith("| `"))
    changelog = tmp_path / "CHANGELOG.md"
    initial_version = rows[0].split("|")[3].strip().strip("`")
    history = changelog.read_text(encoding="utf-8")
    initial_heading = re.search(
        rf"^### \[DART {re.escape(initial_version)} ", history, re.MULTILINE
    )
    assert initial_heading is not None
    major = f"## DART {initial_version.split('.')[0]}\n"
    changelog.write_text(
        history.split("## DART ", 1)[0]
        + f"{major}\n"
        + history[initial_heading.start() :],
        encoding="utf-8",
    )
    for row in rows:
        owner.write_text(
            text.replace(target_row, row, 1),
            encoding="utf-8",
        )
        version = row.split("|")[3].strip().strip("`")
        history = changelog.read_text(encoding="utf-8")
        section = re.search(
            rf"^### \[DART {re.escape(version)} \([^\n]*\n.*?(?=^### \[DART |^## |\Z)",
            history,
            re.MULTILINE | re.DOTALL,
        )
        heading = (
            section[0].strip()
            if section
            else f"### [DART {version} (Unreleased)](https://github.com/dartsim/dart/milestone/105)"
        )
        if section:
            history = history[: section.start()] + history[section.end() :]
        changelog.write_text(
            history.replace(major, f"{major}\n{heading}\n", 1),
            encoding="utf-8",
        )
        errors = []

        infra.check_release_target(tmp_path, errors)
        infra.check_release_references(tmp_path, errors)

        assert errors == []


@pytest.mark.parametrize(
    ("catalog", "line_number"),
    [
        ('msgid "Neutral source"\nmsgstr "현재 DART 6.22"\n', 2),
        ('msgid "Neutral source"\nmsgstr "현재 DART 7.0"\n', 2),
        (
            'msgid "Neutral source"\nmsgstr "release-7.0 브랜치는 DART 7.0를 개발합니다"\n',
            2,
        ),
        ('msgid "Neutral source"\nmsgstr "다음 릴리스는 DART 6.22.0입니다."\n', 2),
        ('msgid "Neutral source"\nmsgstr "차기 버전은 **DART 6.22**입니다."\n', 2),
        ('msgid "Neutral source"\nmsgstr "다음 릴리스: ``DART 6.22.0``"\n', 2),
        ('msgid "Neutral source"\nmsgstr "예정된 릴리스는 DART 6.22.0입니다."\n', 2),
        ('msgid "Neutral source"\nmsgstr "다음 릴리스는 ``6.22.0``입니다."\n', 2),
        ('msgid "Neutral source"\nmsgstr "차기 버전은 6.22.0입니다."\n', 2),
        ('msgid "Neutral source"\nmsgstr "현재 안정 버전은 DART 6.22.0"\n', 2),
        (
            'msgid "Neutral source"\nmsgstr "현재 안정 버전은 **DART 6.22.0** 입니다."\n',
            2,
        ),
        ('msgid "Neutral source"\nmsgstr "현재 ``DART 6.22`` 입니다."\n', 2),
        ('msgid "Neutral source"\nmsgstr "DART 6.22 개발 브랜치"\n', 2),
        ('msgid "Neutral source"\nmsgstr "main 브랜치는 DART 6.22를 개발합니다"\n', 2),
        ('msgid "Neutral source"\nmsgstr "DART 6.22.0 마일스톤을 사용하세요"\n', 2),
        ('msgid "Neutral source"\nmsgstr "마일스톤을 DART 6.22.0으로 설정하세요"\n', 2),
        ('msgid "Neutral source"\nmsgstr "currently DART 6.22"\n', 2),
        ('msgid "Neutral source"\nmsgstr "main develops DART 6.22"\n', 2),
        ('msgid "Neutral source"\nmsgstr ""\n"현재 "\n"DART 6."\n"22"\n', 2),
        ('msgid "Neutral source"\nmsgstr "현재\\nDART 6.22"\n', 2),
        ('msgid "Neutral source"\nmsgstr "\\"currently DART 6.22\\""\n', 2),
        (
            'msgid "one"\nmsgid_plural "many"\n'
            'msgstr[0] "Neutral"\nmsgstr[1] "현재 DART 6.22"\n',
            4,
        ),
    ],
)
def test_release_translations_reject_copied_routing(
    release_reference_target, tmp_path, catalog, line_number
):
    relative = "docs/readthedocs/locales/ko/LC_MESSAGES/index.po"
    path = tmp_path / relative
    path.parent.mkdir(parents=True)
    path.write_text(catalog, encoding="utf-8")
    errors = []

    infra.check_release_references(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith(f"{relative}:{line_number}: resolve the release target")


@pytest.mark.parametrize(
    "catalog",
    [
        'msgid "currently DART 6.22"\nmsgstr "현재 DART 6 호환성 계열"\n',
        'msgid ""\n"현재 DART 6.22"\nmsgstr "호환성 계열"\n',
        '# currently DART 6.22\nmsgid "Neutral"\nmsgstr "호환성 계열"\n',
        '#~ msgid "Neutral"\n#~ msgstr "현재 DART 6.22"\n',
        '#~ msgid "Neutral"\n#~ msgstr ""\n#~ "현재 DART "\n#~ "6.22"\n',
        'msgid "Neutral"\nmsgstr "2026-10-04: DART 버전 6.19.5 출시."\n',
        'msgid "Neutral"\nmsgstr "과거 마일스톤은 DART 6.20.0이었습니다."\n',
        'msgid "Neutral"\nmsgstr "DART 6.20에는 C++17이 필요합니다."\n',
        'msgid "currently DART 6.22"\nmsgstr ""\n',
    ],
)
def test_release_translations_allow_source_history_and_obsolete(tmp_path, catalog):
    path = tmp_path / "docs/readthedocs/locales/ko/LC_MESSAGES/index.po"
    path.parent.mkdir(parents=True)
    path.write_text(catalog, encoding="utf-8")
    errors = []

    infra.check_release_references(tmp_path, errors)

    assert errors == []


@pytest.mark.parametrize(
    "translation",
    ['msgstr "unclosed', 'msgstr ""\n"unclosed', "msgstr invalid"],
)
def test_release_translations_reject_malformed_strings(tmp_path, translation):
    path = tmp_path / "docs/readthedocs/locales/ko/LC_MESSAGES/index.po"
    path.parent.mkdir(parents=True)
    path.write_text(f'msgid "Neutral"\n{translation}\n', encoding="utf-8")
    errors = []

    infra.check_release_references(tmp_path, errors)

    assert len(errors) == 1
    assert "invalid PO translation string" in errors[0]


@pytest.fixture
def release_target_owner(tmp_path, monkeypatch):
    monkeypatch.delenv("GITHUB_BASE_REF", raising=False)
    monkeypatch.delenv("GITHUB_REF_NAME", raising=False)
    monkeypatch.delenv("GITHUB_EVENT_PATH", raising=False)
    path = tmp_path / "docs/onboarding/release-management.md"
    path.parent.mkdir(parents=True)
    path.write_text(
        "## Release Target\n\n"
        "| Branch | Phase | Next release |\n"
        "| ------ | ----- | ------------ |\n",
        encoding="utf-8",
    )
    (tmp_path / "CHANGELOG.md").write_text(
        "### [DART 6.22.0 (Unreleased)](https://github.com/dartsim/dart/milestones)\n\n"
        "### [DART 6.20.1 (Unreleased)](https://github.com/dartsim/dart/milestones)\n\n"
        "### [DART 6.20.0 (Unreleased)](https://github.com/dartsim/dart/milestones)\n\n"
        "### [DART 6.19.5 (2026-10-04)](https://github.com/dartsim/dart/milestones)\n",
        encoding="utf-8",
    )
    (tmp_path / "package.xml").write_text(
        "<package><version>6.19.5</version></package>\n", encoding="utf-8"
    )
    (tmp_path / "pixi.toml").write_text(
        '[workspace]\nversion = "6.19.5"\n', encoding="utf-8"
    )
    return path


@pytest.mark.parametrize(
    "row",
    [
        "| `main` | Development | `6.22.0` |",
        "| `main` | Development | `7.0.0` |",
        "| `release-7.0` | Stabilization | `7.0.0` |",
        "| `release-7.0` | Maintenance | `7.0.1` |",
        "| `release-6.20` | Stabilization | `6.20.0` |",
        "| `release-6.20` | Maintenance | `6.20.1` |",
    ],
)
def test_release_target_accepts_valid_rows(release_target_owner, tmp_path, row):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + row + "\n\n## History\n\n| main | Development | 6.19.0 |\n",
        encoding="utf-8",
    )
    version = row.split("|")[3].strip().strip("`")
    earlier = "6.19.0" if version == "6.20.0" else "6.20.0"
    (tmp_path / "CHANGELOG.md").write_text(
        f"### [DART {version} (Unreleased)](https://github.com/dartsim/dart/milestone/105)\n\n"
        f"### [DART {earlier} (Unreleased)](https://github.com/dartsim/dart/milestone/99)\n\n"
        "### [DART 6.19.5 (2026-10-04)](https://github.com/dartsim/dart/milestone/104?closed=1)\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == []


@pytest.mark.parametrize("leading", ["6.23.0", "6.21.0"])
def test_release_target_rejects_owner_row_without_leading_changelog_update(
    release_target_owner, tmp_path, leading
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    changelog = tmp_path / "CHANGELOG.md"
    changelog.write_text(
        f"### [DART {leading} (Unreleased)](https://github.com/dartsim/dart/milestone/105)\n\n"
        + changelog.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize("state", ["Unreleased", "2026-10-10"])
@pytest.mark.parametrize(
    ("target", "retained", "valid"),
    [
        ("6.21.0", "6.22.0", False),
        ("6.21.0", "6.100.0", False),
        ("6.100.0", "6.21.0", True),
        ("6.21.9", "6.21.10", False),
        ("6.21.10", "6.21.9", True),
        ("6.21.0", "6.19.5", True),
        ("7.0.0", "6.100.0", True),
        ("6.22.0", "7.0.0", False),
        ("7.0.0", "12.0.0", False),
    ],
)
def test_release_target_orders_retained_changelog_versions_numerically(
    release_target_owner, tmp_path, state, target, retained, valid
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + f"| main | Development | {target} |\n", encoding="utf-8"
    )
    link_suffix = "" if state == "Unreleased" else "?closed=1"
    retained_state = "Unreleased" if state == "Unreleased" else "2026-10-04"
    (tmp_path / "CHANGELOG.md").write_text(
        f"### [DART {target} ({state})](https://github.com/dartsim/dart/milestone/105{link_suffix})\n\n"
        "### [DART 6.20.0 (2026-10-03)](https://github.com/dartsim/dart/milestone/98?closed=1)\n\n"
        f"### [DART {retained} ({retained_state})](https://github.com/dartsim/dart/milestone/99{link_suffix})\n",
        encoding="utf-8",
    )
    if state != "Unreleased":
        (tmp_path / "package.xml").write_text(
            f"<package><version>{target}</version></package>\n", encoding="utf-8"
        )
        (tmp_path / "pixi.toml").write_text(
            f'[workspace]\nversion = "{target}"\n', encoding="utf-8"
        )
    errors = []

    infra.check_release_target(tmp_path, errors)

    if valid:
        assert errors == []
    else:
        assert len(errors) == 1
        assert errors[0].startswith(f"CHANGELOG.md: Release Target `{target}`")
        assert "newer retained" in errors[0]


@pytest.mark.parametrize(
    ("leading_section", "retained_section", "error"),
    [
        ("7", "6", None),
        ("6", "6", "DART 7.0.0 must be listed under `## DART 7`, not `## DART 6`"),
        (None, "6", "DART 7.0.0 must be listed under `## DART 7`"),
        ("7", "7", "DART 6.20.0 must be listed under `## DART 6`, not `## DART 7`"),
    ],
)
def test_release_target_requires_matching_major_sections(
    release_target_owner, tmp_path, leading_section, retained_section, error
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 7.0.0 |\n", encoding="utf-8"
    )
    leading = f"## DART {leading_section}\n\n" if leading_section else ""
    (tmp_path / "CHANGELOG.md").write_text(
        leading
        + "### [DART 7.0.0 (Unreleased)](https://github.com/dartsim/dart/milestone/105)\n\n"
        + f"## DART {retained_section}\n\n"
        + "### [DART 6.20.0 (2026-10-11)](https://github.com/dartsim/dart/milestone/99?closed=1)\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == ([] if error is None else [f"CHANGELOG.md: {error}"])


def test_release_target_ends_major_sections_at_other_level_two_headings(
    release_target_owner, tmp_path
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 7.0.0 |\n", encoding="utf-8"
    )
    (tmp_path / "CHANGELOG.md").write_text(
        "## DART 7\n\n"
        "## Notes\n\n"
        "### [DART 7.0.0 (Unreleased)](https://github.com/dartsim/dart/milestone/105)\n\n"
        "## DART 6\n\n"
        "### [DART 6.20.0 (2026-10-11)](https://github.com/dartsim/dart/milestone/99?closed=1)\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == ["CHANGELOG.md: DART 7.0.0 must be listed under `## DART 7`"]


@pytest.mark.parametrize(
    ("target", "package_version", "workspace_version", "valid"),
    [
        ("6.21.0", "6.20.1", "6.19.5", True),
        ("6.22.0", "6.9.0", "6.9.0", True),
        ("6.20.0", "6.21.0", "6.19.5", False),
        ("6.20.0", "6.19.5", "6.21.0", False),
        ("6.20.0", "6.20.1", "6.20.1", False),
        ("6.9.0", "6.22.0", "6.8.0", False),
        ("6.22.0", "6.19.5", "6.23.0.rc1", False),
        ("6.22.0", "5.19.5", "5.19.5", True),
        ("7.0.0", "6.19.5", "6.19.5", True),
        ("7.0.1", "7.0.0", "7.0.0", True),
        ("7.0.0", "7.0.0", "6.19.5", False),
        ("7.0.0", "6.19.5", "7.0.0", False),
        ("7.0.0", "7.1.0", "6.19.5", False),
        ("7.0.0", "6.19.5", "7.1.0.rc1", False),
        ("7.0.0", "6.19.5", "7.0.0.dev1", True),
        ("7.0.0", "6.19.5", "7.0.0.alpha2", True),
        ("7.0.0", "6.19.5", "7.0.0.beta3", True),
        ("7.0.0", "6.19.5", "7.0.0.rc4", True),
    ],
)
def test_release_target_orders_unreleased_source_versions_numerically(
    release_target_owner, tmp_path, target, package_version, workspace_version, valid
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + f"| main | Development | {target} |\n", encoding="utf-8"
    )
    (tmp_path / "CHANGELOG.md").write_text(
        f"### [DART {target} (Unreleased)](https://github.com/dartsim/dart/milestone/105)\n",
        encoding="utf-8",
    )
    (tmp_path / "package.xml").write_text(
        f"<package><version>{package_version}</version></package>\n", encoding="utf-8"
    )
    (tmp_path / "pixi.toml").write_text(
        f'[workspace]\nversion = "{workspace_version}"\n', encoding="utf-8"
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    if valid:
        assert errors == []
    else:
        assert len(errors) == 1
        assert errors[0].startswith(f"CHANGELOG.md: Release Target `{target}`")


@pytest.mark.parametrize("suffix", ["dev1", "alpha2", "beta3", "rc4"])
def test_release_target_accepts_same_core_prerelease_workspace_versions(
    release_target_owner, tmp_path, suffix
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    (tmp_path / "package.xml").write_text(
        "<package><version>6.19.5</version></package>\n", encoding="utf-8"
    )
    (tmp_path / "pixi.toml").write_text(
        f'[workspace]\nversion = "6.22.0.{suffix}"\n', encoding="utf-8"
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == []


@pytest.mark.parametrize("suffix", ["dev1", "alpha2", "beta3", "rc4"])
def test_release_target_rejects_prerelease_package_versions(
    release_target_owner, tmp_path, suffix
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    (tmp_path / "package.xml").write_text(
        f"<package><version>6.22.0.{suffix}</version></package>\n", encoding="utf-8"
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize(
    "package",
    [
        '<package><version format="semver">6.19.5</version></package>',
        "<package><version> 6.19.5</version></package>",
        "<package><version>6.19.5 </version></package>",
        "<package><version>\n6.19.5\n</version></package>",
    ],
)
def test_release_target_rejects_package_versions_outside_cmake_grammar(
    release_target_owner, tmp_path, package
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    (tmp_path / "package.xml").write_text(package, encoding="utf-8")
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize("relative", ["package.xml", "pixi.toml"])
@pytest.mark.parametrize("version", ["invalid", "6.21", "6.21.0.preview1", "7.0.0"])
def test_release_target_rejects_invalid_unreleased_source_versions(
    release_target_owner, tmp_path, relative, version
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    content = (
        f"<package><version>{version}</version></package>\n"
        if relative == "package.xml"
        else f'[workspace]\nversion = "{version}"\n'
    )
    (tmp_path / relative).write_text(content, encoding="utf-8")
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize(
    ("package_version", "workspace_version"),
    [("6.22.0", "6.22.0"), ("6.22.0", "6.19.5"), ("6.19.5", "6.22.0")],
)
def test_release_target_rejects_undated_packaging(
    release_target_owner, tmp_path, package_version, workspace_version
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    (tmp_path / "package.xml").write_text(
        f"<package><version>{package_version}</version></package>\n", encoding="utf-8"
    )
    (tmp_path / "pixi.toml").write_text(
        f'[workspace]\nversion = "{workspace_version}"\n', encoding="utf-8"
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize(
    ("relative", "content"),
    [
        ("package.xml", None),
        ("package.xml", "<package/>"),
        ("pixi.toml", None),
        ("pixi.toml", "workspace = 6220\n"),
        ("pixi.toml", '[workspace]\nversion = "   "\n'),
    ],
)
def test_release_target_requires_unreleased_source_metadata(
    release_target_owner, tmp_path, relative, content
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    path = tmp_path / relative
    if content is None:
        path.unlink()
    else:
        path.write_text(content, encoding="utf-8")
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize(
    ("branch", "phase", "version"),
    [
        ("main", "Development", "6.22.0"),
        ("main", "Development", "7.0.0"),
        ("release-7.0", "Stabilization", "7.0.0"),
        ("release-7.0", "Maintenance", "7.0.1"),
        ("release-6.20", "Stabilization", "6.20.0"),
        ("release-6.20", "Maintenance", "6.20.1"),
    ],
)
def test_release_target_accepts_dated_packaging_candidate(
    release_target_owner, tmp_path, branch, phase, version
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + f"| {branch} | {phase} | {version} |\n", encoding="utf-8"
    )
    (tmp_path / "CHANGELOG.md").write_text(
        f"### [DART {version} (2026-10-10)](https://github.com/dartsim/dart/milestone/105?closed=1)\n\n"
        "### [DART 6.19.5 (2026-10-04)](https://github.com/dartsim/dart/milestone/104?closed=1)\n",
        encoding="utf-8",
    )
    (tmp_path / "package.xml").write_text(
        f"<package><version>{version}</version></package>\n", encoding="utf-8"
    )
    (tmp_path / "pixi.toml").write_text(
        f'[workspace]\nversion = "{version}"\n', encoding="utf-8"
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == []


@pytest.fixture
def dated_release_target(release_target_owner, tmp_path):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    changelog = tmp_path / "CHANGELOG.md"
    changelog.write_text(
        "### [DART 6.22.0 (2026-10-10)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
        encoding="utf-8",
    )
    (tmp_path / "package.xml").write_text(
        "<package><version>6.22.0</version></package>\n", encoding="utf-8"
    )
    (tmp_path / "pixi.toml").write_text(
        '[workspace]\nversion = "6.22.0"\n', encoding="utf-8"
    )
    return changelog


@pytest.fixture
def unreleased_release_target(release_target_owner, tmp_path):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    changelog = tmp_path / "CHANGELOG.md"
    changelog.write_text(
        "### [DART 6.22.0 (Unreleased)](https://github.com/dartsim/dart/milestone/105)\n",
        encoding="utf-8",
    )
    return changelog


@pytest.mark.parametrize(
    ("event", "valid"),
    [
        (
            {
                "pull_request": {
                    "milestone": {
                        "title": "DART 6.22.0",
                        "number": 105,
                        "state": "open",
                    }
                }
            },
            True,
        ),
        (
            {
                "pull_request": {
                    "milestone": {
                        "title": "DART 6.21.0",
                        "number": 105,
                        "state": "open",
                    }
                }
            },
            False,
        ),
        (
            {
                "pull_request": {
                    "milestone": {"title": "DART 6.22.0", "number": 99, "state": "open"}
                }
            },
            False,
        ),
        (
            {
                "pull_request": {
                    "milestone": {
                        "title": "DART 6.22.0",
                        "number": 105,
                        "state": "closed",
                    }
                }
            },
            False,
        ),
        (
            {"pull_request": {"milestone": {"title": "DART 6.22.0", "number": 105}}},
            False,
        ),
        ({"pull_request": {"milestone": None}}, False),
        ({"pull_request": {}}, False),
        ({"pull_request": None}, False),
        ({"ref": "refs/heads/main"}, True),
        ([], False),
    ],
)
def test_release_target_checks_unreleased_pr_milestone(
    unreleased_release_target, tmp_path, monkeypatch, event, valid
):
    event_path = tmp_path / "event.json"
    event_path.write_text(json.dumps(event), encoding="utf-8")
    monkeypatch.setenv("GITHUB_EVENT_PATH", str(event_path))
    errors = []

    infra.check_release_target(tmp_path, errors)

    if valid:
        assert errors == []
    else:
        assert len(errors) == 1
        assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize("number", [0, -1, 1.0, True, "105", None])
def test_release_target_requires_positive_integer_unreleased_milestone_number(
    unreleased_release_target, tmp_path, monkeypatch, number
):
    unreleased_release_target.write_text(
        "### [DART 6.22.0 (Unreleased)](https://github.com/dartsim/dart/milestones)\n",
        encoding="utf-8",
    )
    event_path = tmp_path / "event.json"
    event_path.write_text(
        json.dumps(
            {
                "pull_request": {
                    "milestone": {
                        "title": "DART 6.22.0",
                        "number": number,
                        "state": "open",
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("GITHUB_EVENT_PATH", str(event_path))
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize("event", [None, {"ref": "refs/heads/main"}])
def test_release_target_accepts_unreleased_without_pr_event(
    unreleased_release_target, tmp_path, monkeypatch, event
):
    if event is not None:
        event_path = tmp_path / "event.json"
        event_path.write_text(json.dumps(event), encoding="utf-8")
        monkeypatch.setenv("GITHUB_EVENT_PATH", str(event_path))
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == []


@pytest.mark.parametrize("event", [None, "{"])
def test_release_target_rejects_unreadable_unreleased_event(
    unreleased_release_target, tmp_path, monkeypatch, event
):
    event_path = tmp_path / "event.json"
    if event is not None:
        event_path.write_text(event, encoding="utf-8")
    monkeypatch.setenv("GITHUB_EVENT_PATH", str(event_path))
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


def test_release_target_accepts_unreleased_pr_without_specific_changelog_link(
    unreleased_release_target, tmp_path, monkeypatch
):
    unreleased_release_target.write_text(
        "### [DART 6.22.0 (Unreleased)](https://github.com/dartsim/dart/milestones)\n",
        encoding="utf-8",
    )
    event_path = tmp_path / "event.json"
    event_path.write_text(
        json.dumps(
            {
                "pull_request": {
                    "milestone": {"title": "DART 6.22.0", "number": 99, "state": "open"}
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("GITHUB_EVENT_PATH", str(event_path))
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == []


@pytest.mark.parametrize(
    "link",
    [
        "https://github.com/dartsim/dart/milestone/105",
        "https://github.com/dartsim/dart/milestone/105?closed=0",
        "https://github.com/dartsim/dart/milestones?closed=1",
        "https://github.com/other/dart/milestone/105?closed=1",
        "https://example.com/milestone/105?closed=1",
        "http://github.com/dartsim/dart/milestone/105?closed=1",
        "https://github.com/dartsim/dart/milestone/0?closed=1",
        "https://github.com/dartsim/dart/milestone/not-a-number?closed=1",
        "",
    ],
)
def test_release_target_requires_closed_milestone_link(
    dated_release_target, tmp_path, link
):
    dated_release_target.write_text(
        f"### [DART 6.22.0 (2026-10-10)]({link})\n", encoding="utf-8"
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize(
    ("event", "valid"),
    [
        (
            json.dumps(
                {
                    "pull_request": {
                        "milestone": {
                            "title": "DART 6.22.0",
                            "number": 105,
                            "state": "open",
                        }
                    }
                }
            ),
            True,
        ),
        (
            json.dumps(
                {"pull_request": {"milestone": {"title": "DART 6.21.0", "number": 105}}}
            ),
            False,
        ),
        (
            json.dumps(
                {"pull_request": {"milestone": {"title": "DART 6.22.0", "number": 99}}}
            ),
            False,
        ),
        (json.dumps({"pull_request": {"milestone": None}}), False),
        (json.dumps({"pull_request": {}}), False),
        (json.dumps({"pull_request": "invalid"}), False),
        (json.dumps({"ref": "refs/heads/main"}), True),
        (json.dumps([]), False),
        ("{", False),
        (None, False),
    ],
)
def test_release_target_checks_packaging_pr_milestone(
    dated_release_target, tmp_path, monkeypatch, event, valid
):
    event_path = tmp_path / "event.json"
    if event is not None:
        event_path.write_text(event, encoding="utf-8")
    monkeypatch.setenv("GITHUB_EVENT_PATH", str(event_path))
    errors = []

    infra.check_release_target(tmp_path, errors)

    if valid:
        assert errors == []
    else:
        assert len(errors) == 1
        assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize("number", [1.0, True])
def test_release_target_requires_integer_pr_milestone_number(
    dated_release_target, tmp_path, monkeypatch, number
):
    dated_release_target.write_text(
        "### [DART 6.22.0 (2026-10-10)](https://github.com/dartsim/dart/milestone/1?closed=1)\n",
        encoding="utf-8",
    )
    event_path = tmp_path / "event.json"
    event_path.write_text(
        json.dumps(
            {"pull_request": {"milestone": {"title": "DART 6.22.0", "number": number}}}
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("GITHUB_EVENT_PATH", str(event_path))
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize("state", ["open", "closed"])
def test_release_target_accepts_packaging_pr_before_or_after_milestone_closure(
    dated_release_target, tmp_path, monkeypatch, state
):
    event_path = tmp_path / "event.json"
    event_path.write_text(
        json.dumps(
            {
                "pull_request": {
                    "milestone": {"title": "DART 6.22.0", "number": 105, "state": state}
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("GITHUB_EVENT_PATH", str(event_path))
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == []


@pytest.mark.parametrize(
    "pixi",
    [
        '[workspace]\nversion = "6.19.5"\n',
        None,
        '[workspace]\nversion = "6.22.0\n',
        "",
        "[workspace]\n",
        'workspace = "invalid"\n',
        "[workspace]\nversion = 6220\n",
    ],
)
def test_release_target_rejects_inconsistent_packaging_workspace(
    release_target_owner, tmp_path, pixi
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    (tmp_path / "CHANGELOG.md").write_text(
        "### [DART 6.22.0 (2026-10-10)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
        encoding="utf-8",
    )
    (tmp_path / "package.xml").write_text(
        "<package><version>6.22.0</version></package>\n", encoding="utf-8"
    )
    path = tmp_path / "pixi.toml"
    if pixi is None:
        path.unlink()
    else:
        path.write_text(pixi, encoding="utf-8")
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize(
    ("changelog", "package"),
    [
        (None, "<package><version>6.19.5</version></package>"),
        ("", "<package><version>6.19.5</version></package>"),
        (
            "### [DART 6.21.0 (Unreleased)](milestone)\n",
            "<package><version>6.19.5</version></package>",
        ),
        (
            "DART 6.22.0 (Unreleased) appears only in prose.\n",
            "<package><version>6.19.5</version></package>",
        ),
        (
            "### [DART 6.22.0 (Unreleased)](milestone)\n"
            "### [DART 6.22.0 (Unreleased)](milestone)\n",
            "<package><version>6.19.5</version></package>",
        ),
        (
            "### [DART 6.22.0 (Unreleased)](milestone)\n"
            "### [DART 6.22.0 (2026-10-10)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
            "<package><version>6.22.0</version></package>",
        ),
        (
            "### [DART 6.22.0 (2026-10-10)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
            "<package><version>6.19.5</version></package>",
        ),
        (
            "### [DART 6.22.0 (2026-10-10)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
            None,
        ),
        (
            "### [DART 6.22.0 (2026-10-10)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
            "<package><version>6.22.0</version>",
        ),
        (
            "### [DART 6.22.0 (2026-10-10)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
            "<package/>",
        ),
        (
            "### [DART 6.22.0 (2026-02-30)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
            "<package><version>6.22.0</version></package>",
        ),
        (
            "### [DART 6.22.0 (20261010)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
            "<package><version>6.22.0</version></package>",
        ),
        (
            "### [DART 6.22.0 (Preview)](https://github.com/dartsim/dart/milestone/105?closed=1)\n",
            "<package><version>6.22.0</version></package>",
        ),
    ],
)
def test_release_target_rejects_invalid_changelog(
    release_target_owner, tmp_path, changelog, package
):
    (tmp_path / "pixi.toml").write_text(
        '[workspace]\nversion = "6.22.0"\n', encoding="utf-8"
    )
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    for relative, content in (("CHANGELOG.md", changelog), ("package.xml", package)):
        path = tmp_path / relative
        if content is None:
            path.unlink()
        else:
            path.write_text(content, encoding="utf-8")
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert errors[0].startswith("CHANGELOG.md: Release Target `6.22.0`")


@pytest.mark.parametrize(
    "rows",
    [
        "",
        "| main | Development | 6.22.0 |\n| main | Development | 6.23.0 |",
        "| main | Development | 6.22.0 |\n| release-6.20 | Maintenance | 6.20.1 |",
        "| feature | Development | 6.22.0 |",
        "| main | Maintenance | 6.22.0 |",
        "| release-6.20 | Development | 6.20.0 |",
        "| release-6.20 | Preview | 6.20.0 |",
        "| release-6.20 | Stabilization | 6.20.1 |",
        "| release-6.20 | Maintenance | 6.20.0 |",
        "| main | Development | 6.22 |",
        "| release-6.20 | Maintenance | 6.21.1 |",
        "| main | Development | 6.22.0 | extra |",
        "| main | Development | 6.22.0 ||",
    ],
)
def test_release_target_rejects_invalid_rows(release_target_owner, tmp_path, rows):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(text + rows + "\n", encoding="utf-8")
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert "Release Target" in errors[0]


@pytest.mark.parametrize(
    ("branch", "version"),
    [("release-7.0", "7.1.0"), ("release-6.20", "7.0.0")],
)
def test_release_target_rejects_major_or_minor_mismatch(
    release_target_owner, tmp_path, branch, version
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + f"| {branch} | Stabilization | {version} |\n"
    )
    (tmp_path / "CHANGELOG.md").write_text(
        f"### [DART {version} (Unreleased)](https://github.com/dartsim/dart/milestones)\n"
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert errors == [
        "docs/onboarding/release-management.md: Release Target has an invalid "
        "branch, phase, or full DART version"
    ]


@pytest.mark.parametrize("mutation", ["missing", "duplicate", "header", "separator"])
def test_release_target_rejects_invalid_tables(
    release_target_owner, tmp_path, mutation
):
    text = release_target_owner.read_text(encoding="utf-8")
    text += "| main | Development | 6.22.0 |\n"
    if mutation == "missing":
        text = text.replace("## Release Target", "## History")
    elif mutation == "duplicate":
        text += "\n" + text
    elif mutation == "header":
        text = text.replace("Next release", "Package version")
    else:
        text = text.replace("| ------ |", "| invalid |")
    release_target_owner.write_text(text, encoding="utf-8")
    errors = []

    infra.check_release_target(tmp_path, errors)

    assert len(errors) == 1
    assert "Release Target" in errors[0]


@pytest.mark.parametrize(
    ("checkout", "base", "ref", "valid"),
    [
        ("docs/topic", "main", "123/merge", True),
        ("docs/topic", "release-6.20", "123/merge", False),
        ("main", "", "", True),
        ("release-6.20", "", "", False),
        ("release-7.0", "", "", False),
        ("", "", "main", True),
        ("", "", "release-6.20", False),
        ("", "", "release-7.0", False),
        ("docs/topic", "", "docs/topic", True),
        ("", "", "", True),
        ("release-6.20", "main", "", True),
        # A stacked PR's base is another topic branch.
        ("docs/topic", "refactor/topic", "123/merge", True),
    ],
)
def test_release_target_checks_known_base_branch(
    release_target_owner, tmp_path, monkeypatch, checkout, base, ref, valid
):
    text = release_target_owner.read_text(encoding="utf-8")
    release_target_owner.write_text(
        text + "| main | Development | 6.22.0 |\n", encoding="utf-8"
    )
    monkeypatch.setenv("GITHUB_BASE_REF", base)
    monkeypatch.setenv("GITHUB_REF_NAME", ref)
    monkeypatch.setattr(
        infra.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], 0, stdout=checkout
        ),
    )
    errors = []

    infra.check_release_target(tmp_path, errors)

    if valid:
        assert errors == []
    else:
        assert len(errors) == 1
        assert f"but the base branch is `{base or checkout or ref}`" in errors[0]


def test_release_scenarios_reject_structural_only_ai_completion_gate():
    data = copy.deepcopy(_scenario_data())
    orientation = next(
        scenario for scenario in data["scenarios"] if scenario["id"] == "orientation"
    )
    orientation["full_gates"] = [
        "pixi run python scripts/check_ai_infrastructure.py --check"
    ]

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(
        "structural-only AI checker is not completion evidence" in error
        for error in errors
    )


def test_simulation_scenario_routes_text_first_and_visual_policy():
    scenarios = {scenario["id"]: scenario for scenario in _scenario_data()["scenarios"]}
    scenario = scenarios["simulation-verification"]

    assert len(scenarios) == 8
    assert scenario["expected_route"]["name"] == "dart-verify-sim"
    assert scenario["focused_gates"] == ["pixi run test"]
    assert scenario["full_gates"] == ["pixi run test-py", "pixi run test-all"]
    assert {
        "dart/simulation",
        "dart/dynamics",
        "dart/collision",
        "dart/constraint",
        "dart/gui",
        "dart/utils",
        "python",
        "examples",
        "tutorials",
        "matching tests",
        "temporary claim-tied evidence",
    } <= set(scenario["permitted_scopes"])
    assert scenario["evidence_policy"] == (
        "text-first-with-claim-tied-visual-or-documented-exception"
    )
    assert scenario["semantic_review_policy"] == (
        "native-image-inspection-or-image-capable-handoff-with-explicit-limitation"
    )


def test_model_upgrade_scenario_routes_release_audit_and_durable_context():
    scenarios = {scenario["id"]: scenario for scenario in _scenario_data()["scenarios"]}
    scenario = scenarios["model-upgrade"]

    assert scenario["expected_route"]["name"] == "dart-model-upgrade"
    assert scenario["specialist_agent"] == "dart_release_auditor"
    assert "docs/plans/dashboard.md" in scenario["owner_docs"]
    assert "docs/dev_tasks/README.md" in scenario["owner_docs"]
    assert scenario["focused_gates"] == ["pixi run check-ai-infra"]
    assert scenario["full_gates"] == ["pixi run test-ai-infra", "pixi run lint"]


def test_release_maintenance_scenario_runs_release_tests():
    scenarios = {scenario["id"]: scenario for scenario in _scenario_data()["scenarios"]}

    assert scenarios["release-maintenance"]["focused_gates"] == ["pixi run test"]


def _scenario_data():
    return json.loads((ROOT / "docs" / "ai" / "agent-scenarios.json").read_text())


def test_branch_profile_and_scenario_keys_match_shared_schema():
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    scenarios = _scenario_data()

    assert set(profile) == infra.BRANCH_PROFILE_KEYS
    assert set(scenarios) == infra.SCENARIO_TOP_LEVEL_KEYS
    for scenario in scenarios["scenarios"]:
        assert infra.SCENARIO_KEYS.issubset(scenario)
        assert set(scenario) - infra.SCENARIO_KEYS <= infra.SCENARIO_OPTIONAL_KEYS
        assert set(scenario["expected_route"]) == infra.ROUTE_KEYS


def test_scenario_profile_must_match_compatibility_profile():
    data = copy.deepcopy(_scenario_data())
    data["profile"] = "main"

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert "scenario inventory: profile does not match branch profile" in errors


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        ("policy", "wrong evidence policy"),
        ("semantic_policy", "wrong semantic review policy"),
        ("prompt", "wrong claim-dependent prompt"),
        ("route", "must route to dart-verify-sim"),
        ("scope", "must cover simulation"),
        ("focused_gate", "focused correctness gate"),
        ("full_gate", "full correctness gates"),
    ],
)
def test_simulation_scenario_contract_mutations_are_rejected(mutation, expected):
    data = copy.deepcopy(_scenario_data())
    scenario = next(
        item for item in data["scenarios"] if item["id"] == "simulation-verification"
    )
    if mutation == "policy":
        scenario["evidence_policy"] = "screenshot-only"
    elif mutation == "semantic_policy":
        scenario["semantic_review_policy"] = "machine-check-only"
    elif mutation == "prompt":
        scenario["prompt_class"] = "take a screenshot"
    elif mutation == "route":
        scenario["expected_route"] = {
            "kind": "domain_skill",
            "name": "dart-test",
            "path": ".agents/skills/dart-test/SKILL.md",
        }
    elif mutation == "scope":
        scenario["permitted_scopes"].remove("dart/gui")
    elif mutation == "focused_gate":
        scenario["focused_gates"] = ["pixi run test-py"]
    else:
        scenario["full_gates"] = ["pixi run test-all"]

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    "marker",
    [
        "text correctness oracle",
        "agent-capture",
        "image-verdict",
        "sole correctness oracle",
        "bm-boxes-headless",
        "Xvfb",
        "--factory module:callable",
        "model/scene loading",
        "collision/contact/constraints",
        "simulation stepping",
        "Filament rendering",
        "visual example",
        "Filament capture is unavailable",
        "replacement evidence",
        "settled-contact",
        "text/geometry oracle",
        "test-agent-debug-overlay",
        "/tmp/dart-visual-evidence/capture_auto0.png",
        "semantic inspection",
        "native image viewer",
        "original detail",
        "do not average",
        "verification-bundle",
        "limiting horizontal or",
        "no-bounded-renderable",
    ],
)
def test_simulation_skill_contract_markers_are_required(monkeypatch, marker):
    source = ROOT / ".claude/skills/dart-verify-sim/SKILL.md"
    original = source.read_text()
    real_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if path == source:
            return original.replace(marker, "missing-marker")
        return real_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)

    errors = infra.exercise_scenarios(ROOT, emit=False)

    assert any("missing contract marker" in error for error in errors)


@pytest.mark.parametrize(
    ("relative", "marker"),
    [
        ("AGENTS.md", "docs/ai/verification.md"),
        (".codex/agents/dart_scout.toml", "text correctness oracle"),
        (".codex/agents/dart_reviewer.toml", "text-first evidence"),
        (".claude/commands/dart-new-task.md", "route through `dart-verify-sim`"),
        (
            ".claude/commands/dart-new-task.md",
            "record why it is unavailable or not applicable",
        ),
        (".claude/commands/dart-ultrawork.md", "routes through `dart-verify-sim`"),
        (".claude/commands/dart-resume.md", "route through `dart-verify-sim`"),
        (".claude/commands/dart-pr.md", "use `dart-verify-sim`"),
        (".claude/commands/dart-manage-pr.md", "Visual verification"),
        (
            ".claude/commands/dart-review-pr.md",
            "require the `dart-verify-sim` text oracle",
        ),
        (".claude/commands/dart-review-pr.md", "justified replacement"),
        (".claude/skills/dart-build/SKILL.md", "dart-verify-sim"),
        (".claude/skills/dart-test/SKILL.md", "load `dart-verify-sim`"),
        (
            ".claude/skills/dart-test/SKILL.md",
            "capture is unavailable or not applicable",
        ),
        (".claude/skills/dart-io/SKILL.md", "also load `dart-verify-sim`"),
        (
            ".claude/skills/dart-io/SKILL.md",
            "claim-tied Filament visual corroboration",
        ),
        (".claude/skills/dart-python/SKILL.md", "load `dart-verify-sim`"),
        (
            ".claude/skills/dart-python/SKILL.md",
            "focused Python text/behavior oracle",
        ),
        (
            ".claude/skills/dart-python/SKILL.md",
            "collision/contact/constraints",
        ),
        (".claude/skills/dart-python/SKILL.md", "GUI/Filament output"),
        (".claude/skills/dart-python/SKILL.md", "visual exception"),
        (".claude/skills/dart-ci/SKILL.md", "also load `dart-verify-sim`"),
        (".claude/skills/dart-ci/SKILL.md", "visual exception"),
        (
            ".claude/commands/dart-downstream-fix.md",
            "route through `dart-verify-sim`",
        ),
        (".claude/commands/dart-downstream-fix.md", "visual exception"),
        (
            ".claude/commands/dart-backport-pr.md",
            "release branch's `dart-verify-sim`",
        ),
        (".claude/commands/dart-backport-pr.md", "visual exception"),
        (".claude/commands/dart-release-ci-fix.md", "use `dart-verify-sim`"),
        (".claude/commands/dart-release-ci-fix.md", "visual exception"),
        (".claude/commands/dart-fix-ci.md", "use `dart-verify-sim`"),
        (
            ".claude/commands/dart-fix-ci.md",
            "collision/contact/constraints",
        ),
        (".claude/commands/dart-fix-ci.md", "visual exception"),
        ("docs/ai/verification.md", "required renderer"),
        ("docs/ai/verification.md", "Name the replacement"),
    ],
)
def test_simulation_consumer_routes_are_required(monkeypatch, relative, marker):
    path = ROOT / relative
    original = path.read_text()
    real_read_text = Path.read_text

    def read_text(candidate, *args, **kwargs):
        if candidate == path:
            return original.replace(marker, "missing-marker", 1)
        return real_read_text(candidate, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)

    errors = infra.exercise_scenarios(ROOT, emit=False)

    assert any(
        relative in error and "simulation route marker" in error for error in errors
    )


def test_boolean_schema_versions_are_rejected():
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    profile["schema_version"] = True
    profile_errors = []
    infra.check_branch_profile(ROOT, profile_errors, profile)
    scenarios = copy.deepcopy(_scenario_data())
    scenarios["schema_version"] = True

    scenario_errors = infra.exercise_scenarios(ROOT, scenarios, emit=False)

    assert any("schema_version" in error for error in profile_errors)
    assert scenario_errors == ["docs/ai/agent-scenarios.json: invalid schema"]


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("schema_version", 1, "`schema_version` must be 2"),
        ("profile", "main", "`profile` must be 'dart7'"),
        ("profile", "dart6", "`profile` must be 'dart7'"),
        ("base_ref", "origin/main", "keys must be"),
    ],
)
def test_branch_profile_rejects_old_branch_identity(field, value, expected):
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    profile[field] = value
    errors = []

    infra.check_branch_profile(ROOT, errors, profile)

    assert any(expected in error for error in errors)


def test_branch_profile_marker_mutations_are_rejected():
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    missing = copy.deepcopy(profile)
    missing["required_markers"].append("definitely missing release marker")
    forbidden = copy.deepcopy(profile)
    forbidden["forbidden_markers"].append("C++17")
    missing_errors = []
    forbidden_errors = []

    infra.check_branch_profile(ROOT, missing_errors, missing)
    infra.check_branch_profile(ROOT, forbidden_errors, forbidden)

    assert any("required marker not visible" in error for error in missing_errors)
    assert any("forbidden marker is visible" in error for error in forbidden_errors)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("python_binding", "pybind11"),
        ("optional_python_bindings", ["nanobind"]),
        ("optional_python_bindings", ["pybind11"]),
    ],
)
def test_branch_profile_requires_nanobind_only(field, value):
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    errors = []
    infra.check_branch_profile(ROOT, errors, profile)
    assert errors == []
    assert "python/dartpy" in profile["required_paths"]
    assert "python/dartpy/nanobind" in profile["forbidden_paths"]
    assert "python/dartpy_nanobind" in profile["forbidden_paths"]

    profile[field] = value
    infra.check_branch_profile(ROOT, errors, profile)

    assert any(f"`{field}` must be" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("required_markers", 1),
        ("required_paths", [1]),
        ("forbidden_markers", [""]),
        ("forbidden_paths", [None]),
        ("downstream_gates", {"gate": "pixi run test"}),
    ],
)
def test_branch_profile_malformed_lists_return_errors(field, value):
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    profile[field] = value
    errors = []

    infra.check_branch_profile(ROOT, errors, profile)

    assert any(
        f"`{field}` must be a non-empty string list" in error for error in errors
    )


@pytest.mark.parametrize("unsafe", ["/etc/passwd", "../outside", "docs/../AGENTS.md"])
def test_branch_profile_paths_cannot_escape_or_be_non_normalized(unsafe):
    profile = json.loads((ROOT / "docs" / "ai" / "branch-profile.json").read_text())
    profile["required_paths"] = [unsafe]
    errors = []

    infra.check_branch_profile(ROOT, errors, profile)

    assert any("invalid repository-relative required path" in error for error in errors)


def test_scoped_agent_instruction_files_are_required(tmp_path):
    for path in infra.required_paths(tmp_path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("placeholder\n")
    scoped = tmp_path / ".codex" / "AGENTS.md"
    scoped.unlink()
    errors = []

    infra.check_required_files(tmp_path, errors)

    assert errors == ["missing required file: .codex/AGENTS.md"]


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("start_dir", "missing/start", "missing start_dir"),
        ("instruction_chain", [], "instruction_chain"),
        ("permitted_scopes", [], "permitted_scopes"),
        ("focused_gates", [], "focused_gates"),
        ("full_gates", [], "full_gates"),
        ("recovery", "docs/missing.md", "missing recovery"),
        ("forbidden_paths", ["AGENTS.md"], "forbidden_paths"),
    ],
)
def test_scenario_contract_mutations_are_rejected(field, value, expected):
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0][field] = value

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("instruction_chain", [1], "instruction_chain"),
        ("owner_docs", [1], "owner_docs"),
        ("permitted_scopes", [1], "permitted_scopes"),
        ("focused_gates", [1], "focused_gates"),
        ("full_gates", [None], "full_gates"),
        ("forbidden_paths", [1], "forbidden_paths"),
    ],
)
def test_scenario_malformed_lists_return_errors(field, value, expected):
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0][field] = value

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("field", "unsafe", "expected"),
    [
        ("start_dir", "/", "start_dir"),
        ("instruction_chain", ["../AGENTS.md"], "instruction_chain"),
        ("owner_docs", ["/etc/passwd"], "owner doc"),
        ("recovery", "../outside.md", "recovery pointer"),
        ("forbidden_paths", ["docs/../unsupported"], "forbidden path"),
    ],
)
def test_scenario_paths_cannot_escape_or_be_non_normalized(field, unsafe, expected):
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0][field] = unsafe

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


def test_scenario_route_path_cannot_escape_repository():
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0]["expected_route"]["path"] = "/etc/passwd"

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any("repository-relative route path" in error for error in errors)


def test_unknown_scenario_route_is_rejected():
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][1]["expected_route"]["name"] = "dart-unsupported"

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any("unknown workflow route" in error for error in errors)


def test_scenario_extra_or_nested_agent_keys_are_rejected():
    extra = copy.deepcopy(_scenario_data())
    extra["scenarios"][0]["extra"] = True
    nested = copy.deepcopy(_scenario_data())
    nested["scenarios"][2]["expected_route"]["agent"] = "dart_scout"

    extra_errors = infra.exercise_scenarios(ROOT, extra, emit=False)
    nested_errors = infra.exercise_scenarios(ROOT, nested, emit=False)

    assert any("keys must contain" in error for error in extra_errors)
    assert any("expected_route keys must be" in error for error in nested_errors)


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ("pixi run lint && definitely-not-a-valid-gate", "invalid gate command"),
        (
            "pixi run python scripts/definitely_missing.py --check",
            "references missing file",
        ),
        (
            "pixi run python scripts/../AGENTS.md",
            "path escapes repository",
        ),
        (
            "pixi run python -m pytest /etc/passwd -q",
            "path escapes repository",
        ),
        ("pixi run lint unexpected-argument", "unexpected arguments"),
    ],
)
def test_scenario_gate_trailing_or_missing_commands_are_rejected(command, expected):
    data = copy.deepcopy(_scenario_data())
    data["scenarios"][0]["focused_gates"] = [command]

    errors = infra.exercise_scenarios(ROOT, data, emit=False)

    assert any(expected in error for error in errors)


def test_task_collection_includes_default_and_feature_tasks():
    data = {
        "tasks": {"default-task": {}},
        "feature": {"gazebo": {"tasks": {"feature-task": {}}}},
    }
    assert infra.collect_task_names(data) == {"default-task", "feature-task"}


def test_task_command_collection_includes_platform_variants():
    data = {
        "tasks": {"test": {"cmd": "ctest unix"}},
        "target": {"win-64": {"tasks": {"test": {"cmd": ["ctest", "windows"]}}}},
    }
    assert infra.collect_task_commands(data, "test") == [
        "ctest unix",
        "ctest windows",
    ]


def test_task_definition_collection_includes_platform_variants():
    data = {
        "tasks": {"test-all": {"cmd": "build unix", "env": {"BUILD_TYPE": "Release"}}},
        "target": {
            "win-64": {
                "tasks": {
                    "test-all": {
                        "cmd": "build windows",
                        "env": {"BUILD_TYPE": "Release"},
                    }
                }
            }
        },
    }

    assert [
        task["cmd"] for task in infra.collect_task_definitions(data, "test-all")
    ] == [
        "build unix",
        "build windows",
    ]


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ("cmake --build build/default --target ALL", True),
        (
            "cmake --build build/default --target test && "
            "cmake --build build/default --target ALL",
            False,
        ),
        ("cmake --build build/default --target test --target ALL", False),
        ("cmake --build build/default --target clean --target ALL", False),
        ('cmake --build "$PIXI_PROJECT_ROOT/build/default" --target ALL', True),
        ("cmake --build $PIXI_PROJECT_ROOT/build/default --target ALL", False),
        (
            "cmake --build build/default\nruntime-wrapper --target ALL",
            False,
        ),
        (
            'cmake --build "$(runtime-wrapper)" --target ALL',
            False,
        ),
        (
            "cmake --build `runtime-wrapper` --target ALL",
            False,
        ),
    ],
)
def test_single_cmake_all_build_contract(command, expected):
    assert infra.is_single_cmake_all_build(command) is expected


@pytest.mark.parametrize(
    ("command", "target", "expected"),
    [
        (
            'cmake --build "build/default" -j --target tests_and_run',
            "tests_and_run",
            True,
        ),
        (
            'cmake --build "build/default" --config "$BUILD_TYPE" '
            "-j --target tests_and_run",
            "tests_and_run",
            True,
        ),
        (
            'cmake --build "build/default" --target tests_and_run && ctest',
            "tests_and_run",
            False,
        ),
        (
            'cmake --build "build/default" --target tests_and_run ALL',
            "tests_and_run",
            False,
        ),
    ],
)
def test_single_cmake_target_build_contract(command, target, expected):
    assert infra.is_single_cmake_target_build(command, target) is expected


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ('cmake -S . -B "$PIXI_PROJECT_ROOT/build/default"', True),
        ("cmake -S . -B $PIXI_PROJECT_ROOT/build/default", False),
        (
            "cmake -G Ninja -S . -B build/default -DCMAKE_BUILD_TYPE=Release",
            True,
        ),
        ("cmake -S . -B build/default -DDART_BUILD_GUI=ON", True),
        ("cmake -S . -B build/default -DUNKNOWN_OPTION=OFF", False),
        (
            "cmake -S . -B build/default -DDART_BUILD_UTILS_ASSETS=ON",
            True,
        ),
        (
            "cmake -S . -B build/default -DDART_BUILD_UTILS_ASSETS=ON --build build/default",
            False,
        ),
        ("cmake -S . -B build/default\nninja test", False),
        ('cmake -S "$(python -m unittest)" -B build/default', False),
        (
            'cmake -E env sh -c "ninja -C build/default test" ' "-S . -B build/default",
            False,
        ),
        (
            "cmake -S . -B build/default --install build/default",
            False,
        ),
        (
            "cmake -S . -B build/default -C runtime-cache.cmake",
            False,
        ),
        ("cmake -S . -B build/default -N", False),
        (
            "cmake -S . -B build/default "
            "-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES=runtime.cmake",
            False,
        ),
        (
            'cmake -S . -B build/default -DCMAKE_PREFIX_PATH="x${IFS}-C"',
            False,
        ),
        (
            'cmake -S . -B build/default -DCMAKE_PREFIX_PATH="$UNKNOWN_PATH"',
            False,
        ),
        ("cmake -S runtime-project -B build/default", False),
    ],
)
def test_configuration_only_command_contract(command, expected):
    assert infra.is_configuration_only_command(command) is expected


def test_pixi_reference_check_includes_durable_resume_surfaces(tmp_path):
    (tmp_path / "pixi.toml").write_text(
        '[tasks]\nlint = { cmd = "true" }\n', encoding="utf-8"
    )
    resume = tmp_path / "docs/dev_tasks/example/RESUME.md"
    resume.parent.mkdir(parents=True)
    resume.write_text("Run `pixi run test-unit` before resuming.\n", encoding="utf-8")
    errors = []

    infra.check_pixi_references(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/RESUME.md:1: unknown Pixi task `test-unit`"
    ]


def test_pixi_reference_check_includes_numbered_task_context(tmp_path):
    (tmp_path / "pixi.toml").write_text(
        '[tasks]\nlint = { cmd = "true" }\n', encoding="utf-8"
    )
    packet = tmp_path / "docs/dev_tasks/example/07-work-packet.md"
    packet.parent.mkdir(parents=True)
    packet.write_text("Run `pixi run test-unit` before resuming.\n", encoding="utf-8")
    errors = []

    infra.check_pixi_references(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/07-work-packet.md:1: unknown Pixi task `test-unit`"
    ]


def _copy_test_gate_contract(root: Path) -> None:
    for relative in (
        "pixi.toml",
        "AGENTS.md",
        "CMakeLists.txt",
        "docs/onboarding/testing.md",
        "docs/onboarding/release-management.md",
        "tests/CMakeLists.txt",
        "python/CMakeLists.txt",
        "python/tests/CMakeLists.txt",
        "cmake/DARTRunCTest.cmake",
    ):
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            (ROOT / relative).read_text(encoding="utf-8"), encoding="utf-8"
        )


def _replace_required_pytest_command(cmake: Path, replacement: str) -> None:
    marker = (
        "    COMMAND\n"
        '      "${Python3_EXECUTABLE}" -I '
        '"${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"\n'
        '      --pythonpath "${DART_PYTHONPATH}" ${dartpy_test_files} -v\n'
    )
    text = cmake.read_text(encoding="utf-8")
    assert marker in text
    cmake.write_text(text.replace(marker, replacement, 1), encoding="utf-8")


@pytest.mark.parametrize(
    ("relative", "marker"),
    [
        ("AGENTS.md", "pixi run test         # Build and run C++ tests"),
        (
            "docs/onboarding/testing.md",
            "CMake's File API",
        ),
        (
            "docs/onboarding/release-management.md",
            "`test-all` does not format or check lint",
        ),
    ],
)
def test_test_gate_contract_rejects_operational_doc_drift(tmp_path, relative, marker):
    _copy_test_gate_contract(tmp_path)
    path = tmp_path / relative
    path.write_text(
        path.read_text(encoding="utf-8").replace(marker, "stale gate claim", 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(relative in error and marker in error for error in errors)


@pytest.mark.parametrize(
    "relative",
    (".codex/AGENTS.md", "docs/ai/README.md", "docs/ai/verification.md"),
)
def test_test_gate_contract_rejects_structural_only_completion_gate(tmp_path, relative):
    _copy_test_gate_contract(tmp_path)
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (ROOT / relative)
        .read_text(encoding="utf-8")
        .replace(
            "pixi run check-ai-infra",
            "pixi run python scripts/check_ai_infrastructure.py --check",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"{relative}: structural-only AI checker is presented as a completion "
        "gate" in error
        for error in errors
    )


def test_test_gate_contract_rejects_task_semantic_drift(tmp_path):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    pixi.write_text(
        pixi.read_text(encoding="utf-8").replace("--target ALL", "--target build"),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "pixi.toml: `test-all` task must retain command marker `--target ALL`"
    ]


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    [
        (
            "--target ALL",
            "--target ALL && ctest",
            "`test-all` runtime coverage must remain owned by the CMake `ALL` graph",
        ),
        (
            "--target ALL",
            "--target ALL test",
            "every `test-all` command must be one CMake build",
        ),
        (
            'depends-on = ["config"]',
            'depends-on = ["config", "test"]',
            "every `test-all` variant must depend only on",
        ),
        (
            'env = { BUILD_TYPE = "Release" }',
            'env = { BUILD_TYPE = "Debug" }',
            'every `test-all` variant must set only `BUILD_TYPE = "Release"`',
        ),
    ],
)
def test_test_gate_contract_rejects_test_all_runtime_semantics(
    tmp_path, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    test_all_offset = text.index("test-all =")
    text = text[:test_all_offset] + text[test_all_offset:].replace(old, new, 1)
    pixi.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    [
        (
            "--target tests_and_run",
            "--target tests",
            "`test` task must retain command marker `--target tests_and_run`",
        ),
        (
            "--target tests_and_run",
            "--target tests_and_run && ctest",
            "every `test` command must be one CMake build",
        ),
        (
            'depends-on = ["config"]',
            'depends-on = ["build-tests"]',
            "every `test` variant must depend only on",
        ),
        (
            'env = { BUILD_TYPE = "Release" }',
            'env = { BUILD_TYPE = "Debug" }',
            'every `test` variant must set only `BUILD_TYPE = "Release"`',
        ),
    ],
)
def test_test_gate_contract_rejects_focused_test_runtime_semantics(
    tmp_path, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    test_offset = text.index("\ntest = {")
    text = text[:test_offset] + text[test_offset:].replace(old, new, 1)
    pixi.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize("task", ("test-ai-infra", "test-agent-debug-overlay"))
def test_test_gate_contract_rejects_raw_canonical_pytest_tasks(tmp_path, task):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    task_offset = text.index(f"{task} =")
    text = text[:task_offset] + text[task_offset:].replace(
        '"scripts/run_pytest.py"',
        '"-m", "pytest"',
        1,
    )
    pixi.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"pixi.toml: `{task}` must use the guarded repository pytest runner" in error
        for error in errors
    )


def test_test_gate_contract_rejects_dependency_indirection(tmp_path):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    test_all_offset = text.index("test-all =")
    text = text[:test_all_offset] + text[test_all_offset:].replace(
        'depends-on = ["config"]',
        'depends-on = ["runtime-wrapper"]',
        1,
    )
    text += '\nruntime-wrapper = { cmd = "ctest", depends-on = ["test"] }\n'
    pixi.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "every `test-all` variant must depend only on" in error for error in errors
    )


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    [
        (
            "    cmake \\\n        -G Ninja",
            "    ctest && cmake \\\n        -G Ninja",
            "every `config` command used by `test-all` must only configure CMake",
        ),
        (
            '""", env = { DART_VERBOSE = "OFF", BUILD_TYPE = "Release" } }',
            '""", depends-on = ["runtime-wrapper"], '
            'env = { DART_VERBOSE = "OFF", BUILD_TYPE = "Release" } }',
            "`config` must not depend on other tasks",
        ),
        (
            'env = { DART_VERBOSE = "OFF", BUILD_TYPE = "Release" }',
            'env = { DART_VERBOSE = "OFF -C runtime.cmake", '
            'BUILD_TYPE = "Release" }',
            "every `config` variant must use the exact branch-owned environment",
        ),
    ],
)
def test_test_gate_contract_rejects_runtime_config_path(tmp_path, old, new, expected):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    assert old in text
    pixi.write_text(text.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    [
        (
            "--check --semantic-cmake",
            "--check",
            "must run the semantic CMake infrastructure check exactly",
        ),
        (
            '"config",\n], env = { BUILD_TYPE = "Release" } }\n'
            "exercise-agent-scenarios",
            '"config", "test",\n], env = { BUILD_TYPE = "Release" } }\n'
            "exercise-agent-scenarios",
            "must depend only on `config`",
        ),
        (
            '], env = { BUILD_TYPE = "Release" } }\nexercise-agent-scenarios',
            '], env = { BUILD_TYPE = "Debug" } }\nexercise-agent-scenarios',
            "must select only the Release configuration",
        ),
    ],
)
def test_test_gate_contract_rejects_ai_check_semantic_drift(
    tmp_path, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    check_offset = text.index("check-ai-infra =")
    prefix = text[:check_offset]
    check_and_after = text[check_offset:]
    assert old in check_and_after
    pixi.write_text(prefix + check_and_after.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    "flag",
    (
        "-DBUILD_TESTING=ON",
        "-DDART_BUILD_DARTPY=ON",
    ),
)
def test_test_gate_contract_requires_runtime_config_flags(tmp_path, flag):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    pixi.write_text(
        pixi.read_text(encoding="utf-8").replace(
            f"        {flag} \\\n",
            "",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"must pin `{flag.removeprefix('-D')}` exactly once" in error
        for error in errors
    )


@pytest.mark.parametrize(
    "flag",
    (
        "-DBUILD_TESTING=ON",
        "-DDART_BUILD_DARTPY=ON",
    ),
)
def test_test_gate_contract_rejects_conflicting_runtime_config_flags(tmp_path, flag):
    _copy_test_gate_contract(tmp_path)
    variable = flag.removeprefix("-D").split("=", maxsplit=1)[0]
    pixi = tmp_path / "pixi.toml"
    pixi.write_text(
        pixi.read_text(encoding="utf-8").replace(
            f"        {flag} \\\n",
            f"        {flag} \\\n        -D{variable}=OFF \\\n",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"must pin `{flag.removeprefix('-D')}` exactly once" in error
        for error in errors
    )


def test_test_gate_contract_requires_runtime_graph(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "list(APPEND all_target_candidates tests_and_run pytest)",
            "list(APPEND all_target_candidates tests)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "CMakeLists.txt: missing `test-all` graph marker" in error for error in errors
    )


def test_test_gate_contract_rejects_commented_runtime_graph(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "CMakeLists.txt"
    marker = "list(APPEND all_target_candidates tests_and_run pytest)"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(marker, f"# {marker}", 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"CMakeLists.txt: missing `test-all` graph marker `{marker}`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    "replacement",
    (
        'message("list(APPEND all_target_candidates tests_and_run pytest)")',
        (
            "message(\n"
            "  list(APPEND all_target_candidates tests_and_run pytest)\n"
            ")"
        ),
        (
            "if(FALSE)\n"
            "  list(APPEND all_target_candidates tests_and_run pytest)\n"
            "endif()"
        ),
        (
            "if(\n"
            "  FALSE\n"
            ")\n"
            "  list(APPEND all_target_candidates tests_and_run pytest)\n"
            "endif()"
        ),
    ),
)
def test_test_gate_contract_rejects_inactive_runtime_graph(tmp_path, replacement):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "CMakeLists.txt"
    marker = "list(APPEND all_target_candidates tests_and_run pytest)"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(marker, replacement, 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"CMakeLists.txt: missing `test-all` graph marker `{marker}`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("relative", "condition", "expected_marker"),
    (
        (
            "python/tests/CMakeLists.txt",
            "DARTPY_PYTEST_FOUND",
            "scripts/run_pytest.py",
        ),
    ),
)
def test_test_gate_contract_rejects_inactive_runtime_targets(
    tmp_path, relative, condition, expected_marker
):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / relative
    text = cmake.read_text(encoding="utf-8")
    condition_offset = text.index(f"if({condition})")
    text = text[:condition_offset] + text[condition_offset:].replace(
        f"if({condition})",
        f"if({condition})\n  if(FALSE)",
        1,
    )
    else_offset = text.index("else()", condition_offset)
    text = text[:else_offset] + "  endif()\n" + text[else_offset:]
    cmake.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"{relative}: missing `test-all` graph marker `{expected_marker}`" in error
        for error in errors
    )


def test_test_gate_contract_rejects_ctest_runner_drift(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "tests/CMakeLists.txt"
    marker = "${PROJECT_SOURCE_DIR}/cmake/DARTRunCTest.cmake"
    text = cmake.read_text(encoding="utf-8")
    assert marker in text
    cmake.write_text(
        text.replace(marker, "${PROJECT_SOURCE_DIR}/cmake/Noop.cmake", 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "tests/CMakeLists.txt: missing `test-all` graph marker "
        "`cmake/DARTRunCTest.cmake`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("relative", "old", "new", "expected"),
    (
        (
            "tests/CMakeLists.txt",
            '"-DDART_CTEST_EXCLUDE_LABEL=gui-device"',
            '"-DDART_CTEST_EXCLUDE_LABEL=unit"',
            "tests/CMakeLists.txt: missing `test-all` graph marker",
        ),
        (
            "tests/CMakeLists.txt",
            '"-DDART_CTEST_EXCLUDE_LABEL=gui-device"',
            '"-DDART_CTEST_EXCLUDE_LABEL=gui"',
            "tests/CMakeLists.txt: missing `test-all` graph marker",
        ),
        (
            "tests/CMakeLists.txt",
            '"-DDART_CTEST_EXCLUDE_LABEL=gui-device"',
            "",
            "tests/CMakeLists.txt: missing `test-all` graph marker",
        ),
        (
            "cmake/DARTRunCTest.cmake",
            '-LE "${DART_CTEST_EXCLUDE_LABEL}"',
            '-L "${DART_CTEST_EXCLUDE_LABEL}"',
            "cmake/DARTRunCTest.cmake: commands and lexical control flow",
        ),
    ),
)
def test_test_gate_contract_pins_the_gui_ctest_exclusion(
    tmp_path, relative, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    path = tmp_path / relative
    text = path.read_text(encoding="utf-8")
    assert old in text
    path.write_text(text.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("old", "new"),
    (
        ("-L gui", "-L filament"),
        ("-LE filament-native", "-LE gui-device"),
        ("&& ctest --test-dir", "&& echo --test-dir"),
        ("python/tests/unit/gui python/tests/integration", "python/tests/integration"),
        ("&& python -I scripts/run_pytest.py", "&& echo -I scripts/run_pytest.py"),
        ("test-gui = {", "test-gui-old = {"),
    ),
)
def test_test_gate_contract_requires_test_gui_to_own_gui_tests(tmp_path, old, new):
    _copy_test_gate_contract(tmp_path)
    pixi = tmp_path / "pixi.toml"
    text = pixi.read_text(encoding="utf-8")
    offset = text.index("test-gui = {")
    assert old in text[offset:]
    # Mutate every `test-gui` variant, including the Windows override.
    pixi.write_text(text[:offset] + text[offset:].replace(old, new), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any("`test-gui` must run the `gui` CTest label" in error for error in errors)


def test_default_pytest_target_leaves_rendering_modules_to_test_gui():
    sources = infra.default_pytest_sources(ROOT)

    assert all((ROOT / path).is_file() for path in infra.GUI_ONLY_PYTHON_TESTS)
    assert sources.isdisjoint(infra.GUI_ONLY_PYTHON_TESTS)
    assert "python/tests/unit/gui/test_gui_import.py" in sources


@pytest.mark.parametrize(
    ("name", "labels", "expected_error"),
    (
        ("test_FilamentBackend", ["gui", "gui-device"], False),
        ("examples_demos_capture", ["gui", "gui-device"], False),
        ("examples_demos_capture_extra", ["gui", "gui-device"], False),
        ("test_GuiViewer", ["gui"], False),
        ("examples_demos_cycle", ["gui"], False),
        ("test_GuiMesh", ["gui"], True),
        ("test_FilamentNative", ["gui-device"], True),
        ("test_GuiDescriptors", ["gui", "gui-device"], True),
        ("examples_demos_cycle", ["gui", "gui-device"], True),
    ),
)
def test_ctest_gui_labels_only_exclude_device_dependent_tests(
    name, labels, expected_error
):
    errors = []

    infra.check_ctest_gui_labels(
        {"name": name, "properties": [{"name": "LABELS", "value": labels}]}, errors
    )

    assert bool(errors) == expected_error


def test_ctest_broad_gate_runs_gui_logic_without_running_graphics(tmp_path):
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22)\n"
        "project(GuiLabels NONE)\n"
        "enable_testing()\n"
        "add_test(NAME gui_logic COMMAND ${CMAKE_COMMAND} -E touch logic-ran)\n"
        'set_tests_properties(gui_logic PROPERTIES LABELS "gui")\n'
        "add_test(NAME gui_render COMMAND ${CMAKE_COMMAND} -E false)\n"
        'set_tests_properties(gui_render PROPERTIES LABELS "gui;gui-device")\n',
        encoding="utf-8",
    )
    build = tmp_path / "build"
    configured = subprocess.run(
        ["cmake", "-S", str(tmp_path), "-B", str(build)], capture_output=True, text=True
    )
    assert configured.returncode == 0, configured.stdout + configured.stderr

    result = subprocess.run(
        [
            "cmake",
            f"-DDART_CTEST_COMMAND={infra.shutil.which('ctest')}",
            infra.GUI_CTEST_EXCLUSION,
            "-P",
            str(ROOT / "cmake/DARTRunCTest.cmake"),
        ],
        cwd=build,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert (build / "logic-ran").is_file()
    assert "1/1 Test" in result.stdout


@pytest.mark.parametrize(
    "injection",
    (
        "set(_dart_gtest_unsets)\n",
        "set(_dart_ctest_arguments --show-only)\n",
        "cmake_language(EXIT 0)\n",
    ),
)
def test_test_gate_contract_rejects_ctest_runner_dataflow_bypass(tmp_path, injection):
    _copy_test_gate_contract(tmp_path)
    runner = tmp_path / "cmake" / "DARTRunCTest.cmake"
    text = runner.read_text(encoding="utf-8")
    final_call = text.rfind("execute_process(")
    assert final_call > 0
    runner.write_text(
        text[:final_call] + injection + text[final_call:],
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "cmake/DARTRunCTest.cmake: commands and lexical control flow must "
        "exactly match" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("opener", "closer"),
    (
        ("if(FALSE)\n", "endif()\n"),
        ("function(unused_ctest_runner)\n", "endfunction()\n"),
    ),
)
def test_test_gate_contract_rejects_unreachable_ctest_execution(
    tmp_path, opener, closer
):
    _copy_test_gate_contract(tmp_path)
    runner = tmp_path / "cmake" / "DARTRunCTest.cmake"
    text = runner.read_text(encoding="utf-8")
    final_call = text.rfind("execute_process(")
    assert final_call > 0
    runner.write_text(
        text[:final_call] + opener + text[final_call:] + closer,
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "cmake/DARTRunCTest.cmake: commands and lexical control flow must "
        "exactly match" in error
        for error in errors
    )


def test_test_gate_contract_rejects_ctest_command_shadowing(tmp_path):
    _copy_test_gate_contract(tmp_path)
    runner = tmp_path / "cmake" / "DARTRunCTest.cmake"
    runner.write_text(
        "macro(execute_process)\n"
        "  set(_dart_environment_result 0)\n"
        "endmacro()\n" + runner.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "cmake/DARTRunCTest.cmake: commands and lexical control flow must "
        "exactly match" in error
        for error in errors
    )


def test_test_gate_contract_rejects_quoted_pytest_target_spoof(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/tests/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_custom_target(\n    pytest",
            'add_custom_target(\n    fake\n    "pytest COMMAND"',
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "python/tests/CMakeLists.txt: missing `test-all` graph marker "
        "`scripts/run_pytest.py`" in error
        for error in errors
    )


def test_test_gate_contract_rejects_quoted_pytest_command_spoof(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/tests/CMakeLists.txt"
    _replace_required_pytest_command(
        cmake,
        (
            "    COMMAND ${CMAKE_COMMAND} -E echo\n"
            '      "COMMAND ${CMAKE_COMMAND} -E env '
            "PYTHONPATH=${DART_PYTHONPATH} "
            "${Python3_EXECUTABLE} "
            "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py "
            '${dartpy_test_files} -v"\n'
        ),
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "python/tests/CMakeLists.txt: missing `test-all` graph marker "
        "`scripts/run_pytest.py`" in error
        for error in errors
    )


def test_test_gate_contract_rejects_bracket_pytest_command_spoof(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/tests/CMakeLists.txt"
    _replace_required_pytest_command(
        cmake,
        (
            "    [=[COMMAND ${CMAKE_COMMAND} -E env "
            "PYTHONPATH=${DART_PYTHONPATH} "
            "${Python3_EXECUTABLE} "
            "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py "
            "${dartpy_test_files} -v]=]\n"
        ),
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "python/tests/CMakeLists.txt: missing `test-all` graph marker "
        "`scripts/run_pytest.py`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("relative", "needle", "occurrence"),
    (
        (
            "CMakeLists.txt",
            "list(APPEND all_target_candidates tests_and_run pytest)",
            1,
        ),
        (
            "CMakeLists.txt",
            "add_custom_target(ALL DEPENDS ${all_targets})",
            1,
        ),
        (
            "tests/CMakeLists.txt",
            "add_custom_target(\n  tests_and_run",
            1,
        ),
        (
            "python/tests/CMakeLists.txt",
            "add_custom_target(\n    pytest",
            1,
        ),
        (
            "python/tests/CMakeLists.txt",
            "add_custom_target(\n    pytest",
            2,
        ),
    ),
)
def test_test_gate_contract_rejects_unreachable_runtime_graph(
    tmp_path, relative, needle, occurrence
):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / relative
    text = cmake.read_text(encoding="utf-8")
    offset = -1
    for _ in range(occurrence):
        offset = text.index(needle, offset + 1)
    cmake.write_text(
        text[:offset] + "return()\n  " + text[offset:],
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        f"{relative}: `return()` may bypass the required `test-all` graph" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("value", "separator", "expected"),
    (
        (
            "$<TARGET_FILE_DIR:dartpy>:/tmp/build/python",
            ":",
            ["$<TARGET_FILE_DIR:dartpy>", "/tmp/build/python"],
        ),
        (
            "$<TARGET_FILE_DIR:dartpy>;C:/build/python",
            ";",
            ["$<TARGET_FILE_DIR:dartpy>", "C:/build/python"],
        ),
        (
            r"$<TARGET_FILE_DIR:dartpy>\;C:/build/python",
            ";",
            ["$<TARGET_FILE_DIR:dartpy>", "C:/build/python"],
        ),
        (
            "$<IF:$<BOOL:1>,/tmp/a,/tmp/b>:/tmp/build/python",
            ":",
            ["$<IF:$<BOOL:1>,/tmp/a,/tmp/b>", "/tmp/build/python"],
        ),
        ("$<TARGET_FILE_DIR:dartpy", ":", []),
    ),
)
def test_split_cmake_path_list_preserves_generator_expressions(
    value, separator, expected
):
    assert infra.split_cmake_path_list(value, separator) == expected


def _configure_semantic_graph_fixture(
    root: Path,
    generator: str | None = None,
    force_windows: bool = False,
) -> Path:
    sources = {
        "CMakeLists.txt": """
cmake_minimum_required(VERSION 3.22)
project(test_graph C CXX)
set(DART_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(DART_VERSION 6.20.0)
set(DART_VERSION_OVERRIDE "" CACHE STRING "")
if(DART_VERSION_OVERRIDE)
  set(DART_VERSION "${DART_VERSION_OVERRIDE}")
endif()
find_package(Python3 COMPONENTS Interpreter REQUIRED)
enable_testing()
set(BUILD_TESTING ON CACHE BOOL "")
set(DART_BUILD_DARTPY ON CACHE BOOL "")
add_library(dartpy MODULE python/dartpy.c)
if(WIN32)
  set(
    DART_PYTHONPATH
    "$<TARGET_FILE_DIR:dartpy>\\;${CMAKE_BINARY_DIR}/python"
  )
else()
  set(DART_PYTHONPATH "$<TARGET_FILE_DIR:dartpy>:${CMAKE_BINARY_DIR}/python")
endif()
add_subdirectory(tests)
add_subdirectory(python/tests)
set(all_target_candidates dartpy)
if(BUILD_TESTING)
  list(APPEND all_target_candidates tests_and_run pytest)
endif()
foreach(target_candidate ${all_target_candidates})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
add_custom_target(ALL DEPENDS ${all_targets})
""".lstrip(),
        "tests/CMakeLists.txt": """
add_test(
  NAME test_DARTConfigVersion
  COMMAND
    "${CMAKE_COMMAND}"
    "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}"
    "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/cmake/DARTConfigVersion"
    "-DDART_VERSION=${DART_VERSION}"
    "-DCMAKE_SIZEOF_VOID_P=${CMAKE_SIZEOF_VOID_P}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_DARTConfigVersion.cmake"
)
add_subdirectory(integration)
add_subdirectory(regression)
add_subdirectory(unit)
add_custom_target(
  tests_and_run
  COMMAND
    ${CMAKE_COMMAND}
    "-DDART_CTEST_COMMAND=${CMAKE_CTEST_COMMAND}"
    "-DDART_CTEST_CONFIGURATION=$<CONFIG>"
    "-DDART_CTEST_EXCLUDE_LABEL=gui-device"
    -P "${PROJECT_SOURCE_DIR}/cmake/DARTRunCTest.cmake"
  DEPENDS ${integration_tests} ${regression_tests} ${unit_tests}
  WORKING_DIRECTORY "${DART_BINARY_DIR}"
)
""".lstrip(),
        "tests/integration/CMakeLists.txt": """
add_executable(INTEGRATION_semantic test_semantic.cpp)
add_test(NAME INTEGRATION_semantic COMMAND INTEGRATION_semantic)
set(integration_tests INTEGRATION_semantic PARENT_SCOPE)
""".lstrip(),
        "tests/regression/CMakeLists.txt": """
add_executable(test_semantic_regression test_semantic.cpp)
add_test(NAME test_semantic_regression COMMAND test_semantic_regression)
set(regression_tests test_semantic_regression PARENT_SCOPE)
""".lstrip(),
        "tests/unit/CMakeLists.txt": """
add_executable(UNIT_semantic test_semantic.cpp)
add_test(NAME UNIT_semantic COMMAND UNIT_semantic)
set(unit_tests UNIT_semantic PARENT_SCOPE)
""".lstrip(),
        "python/dartpy.c": "int dartpy_semantic_fixture(void) { return 0; }\n",
        "python/tests/CMakeLists.txt": """
set(DARTPY_PYTEST_FOUND TRUE)
set(dartpy_test_files test_semantic.py)
set(dartpy_test_utils)
if(DARTPY_PYTEST_FOUND)
  add_custom_target(
    pytest
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "Running pytest by: ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/scripts/run_pytest.py --pythonpath ${DART_PYTHONPATH} [sources]"
    COMMAND
      "${Python3_EXECUTABLE}" -I "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
      --pythonpath "${DART_PYTHONPATH}" ${dartpy_test_files} -v
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES
      ${dartpy_test_files}
      ${dartpy_test_utils}
      "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
    DEPENDS dartpy
  )
else()
  add_custom_target(
    pytest
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "Warning: Failed to run pytest because pytest is not found!"
  )
endif()
""".lstrip(),
        "python/tests/test_semantic.py": "def test_semantic():\n    pass\n",
        "cmake/DARTRunCTest.cmake": (ROOT / "cmake/DARTRunCTest.cmake").read_text(
            encoding="utf-8"
        ),
        "cmake/DARTConfigVersion.cmake.in": (
            ROOT / "cmake/DARTConfigVersion.cmake.in"
        ).read_text(encoding="utf-8"),
        "tests/cmake/test_DARTConfigVersion.cmake": (
            ROOT / "tests/cmake/test_DARTConfigVersion.cmake"
        ).read_text(encoding="utf-8"),
        "package.xml": "<package><version>6.20.0</version></package>\n",
        "scripts/run_pytest.py": (ROOT / "scripts/run_pytest.py").read_text(
            encoding="utf-8"
        ),
        "pyproject.toml": """
[tool.pytest.ini_options]
minversion = "6.0"
addopts = ["-ra", "--showlocals", "--strict-markers", "--strict-config"]
xfail_strict = true
filterwarnings = ["error"]
testpaths = ["tests"]
""".lstrip(),
    }
    for relative, text in sources.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    for relative in (
        "tests/integration/test_semantic.cpp",
        "tests/regression/test_semantic.cpp",
        "tests/unit/test_semantic.cpp",
    ):
        path = root / relative
        path.write_text("int main(void) { return 0; }\n", encoding="utf-8")

    build = root / "build"
    command = [
        "cmake",
        "-S",
        str(root),
        "-B",
        str(build),
        "-DCMAKE_BUILD_TYPE=Release",
    ]
    if generator is not None:
        command[1:1] = ["-G", generator]
    if force_windows:
        command.append("-DWIN32=TRUE")
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return build


def test_cmake_semantic_graph_probe_accepts_effective_runtime_graph(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


@pytest.mark.parametrize("override", ("", "7.0.0", "7.4.2", "8.0.0"))
def test_cmake_semantic_graph_accepts_effective_version_script_test(tmp_path, override):
    build = _configure_semantic_graph_fixture(tmp_path)
    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(tmp_path),
            "-B",
            str(build),
            f"-DDART_VERSION_OVERRIDE={override}",
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    (
        (
            "NAME test_DARTConfigVersion",
            "NAME missing_version_test",
            "required CMake script test",
        ),
        (
            "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}",
            "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}/wrong",
            "unapproved command arguments",
        ),
        (
            "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/cmake/DARTConfigVersion",
            "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/wrong",
            "unapproved command arguments",
        ),
        (
            "-DDART_VERSION=${DART_VERSION}",
            "-DDART_VERSION=5.0.0",
            "unapproved command arguments",
        ),
        (
            "-DCMAKE_SIZEOF_VOID_P=${CMAKE_SIZEOF_VOID_P}",
            "-DCMAKE_SIZEOF_VOID_P=0",
            "unapproved command arguments",
        ),
        (
            "cmake/test_DARTConfigVersion.cmake",
            "cmake/Noop.cmake",
            "unapproved command arguments",
        ),
        (
            '"${CMAKE_COMMAND}"\n    "-DSOURCE_DIR=',
            '"${CMAKE_CTEST_COMMAND}"\n    "-DSOURCE_DIR=',
            "must use the active CMake executable",
        ),
        (
            '-P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_DARTConfigVersion.cmake"',
            "--version",
            "uses non-executing option",
        ),
    ),
)
def test_cmake_semantic_graph_rejects_changed_version_script_test(
    tmp_path, old, new, expected
):
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake = tmp_path / "tests/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(old, new, 1), encoding="utf-8"
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("property_name", "value", "expected"),
    (
        ("DISABLED", "TRUE", "is disabled"),
        ("WILL_FAIL", "TRUE", "result-neutralizing"),
        ("LABELS", "gui-device", "must run in the broad gate"),
    ),
)
def test_cmake_semantic_graph_rejects_bypassed_version_script_test(
    tmp_path, property_name, value, expected
):
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake = tmp_path / "tests/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + f"\nset_tests_properties(test_DARTConfigVersion PROPERTIES {property_name} {value})\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        expected in error and "test_DARTConfigVersion" in error for error in errors
    )


def _add_contact_surface_probe_fixture(root: Path):
    eigen = root / "eigen"
    eigen.mkdir()
    eigen.joinpath("Eigen3Targets.cmake").write_text(
        "add_library(Eigen3::Eigen INTERFACE IMPORTED)\n"
        "set_target_properties(Eigen3::Eigen PROPERTIES "
        'INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/eigen/include")\n',
        encoding="utf-8",
    )
    cmake = root / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(tests)",
            'set(Eigen3_DIR "${CMAKE_SOURCE_DIR}/eigen" CACHE PATH "")\n'
            'include("${Eigen3_DIR}/Eigen3Targets.cmake")\n'
            "add_subdirectory(tests)",
            1,
        ),
        encoding="utf-8",
    )
    source = root / "tests/cmake/ContactSurfaceProbe.cpp"
    source.parent.mkdir(exist_ok=True)
    source.write_text(
        (ROOT / "tests/cmake/ContactSurfaceProbe.cpp").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    cmake = root / "tests/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nget_target_property(probe_eigen Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)\n"
        "if(MSVC)\n"
        "  set(probe_flags /std:c++17 /permissive- /Zc:twoPhase- /Zs)\n"
        "  set(probe_include_flag /I)\n"
        "else()\n"
        "  set(probe_flags -std=c++14 -fsyntax-only)\n"
        "  set(probe_include_flag -I)\n"
        "endif()\n"
        "foreach(include_dir ${PROJECT_SOURCE_DIR} ${DART_BINARY_DIR} ${probe_eigen})\n"
        '  list(APPEND probe_flags "${probe_include_flag}${include_dir}")\n'
        "endforeach()\n"
        "add_test(NAME ContactSurfaceProbe COMMAND "
        '"${CMAKE_CXX_COMPILER}" ${probe_flags} '
        '"${PROJECT_SOURCE_DIR}/tests/cmake/ContactSurfaceProbe.cpp")\n',
        encoding="utf-8",
    )
    result = subprocess.run(
        ["cmake", "-S", str(root), "-B", str(root / "build")],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return cmake, source


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    (
        (None, None, None),
        (
            "${probe_flags}",
            "${probe_flags} -std=c++20",
            "only the platform probe flags",
        ),
        (
            "${probe_flags}",
            "${probe_flags} -DPROBE_BYPASS",
            "only the platform probe flags",
        ),
        (
            "${probe_flags}",
            "${probe_flags} -I${PROJECT_SOURCE_DIR}/extra",
            "only the platform probe flags",
        ),
        (" ${probe_eigen}", "", "only the platform probe flags"),
        ("${CMAKE_CXX_COMPILER}", "${CMAKE_COMMAND}", "configured compiler"),
        (
            "add_test(NAME ContactSurfaceProbe",
            "add_test(NAME FakeContactSurfaceProbe",
            "ContactSurfaceProbe is not registered",
        ),
    ),
)
def test_cmake_semantic_graph_contact_surface_probe_command(
    tmp_path, old, new, expected
):
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake, _ = _add_contact_surface_probe_fixture(tmp_path)
    if old is not None:
        cmake.write_text(
            cmake.read_text(encoding="utf-8").replace(old, new),
            encoding="utf-8",
        )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    if expected is None:
        assert errors == []
    else:
        assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("compiler_id", "simulate_id"), (("MSVC", ""), ("Clang", "MSVC"))
)
@pytest.mark.parametrize("missing_flag", (None, "/permissive-", "/Zc:twoPhase-"))
def test_contact_surface_probe_requires_msvc_parsing_flags(
    tmp_path, monkeypatch, compiler_id, simulate_id, missing_flag
):
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake, _ = _add_contact_surface_probe_fixture(tmp_path)
    text = cmake.read_text(encoding="utf-8").replace("if(MSVC)", "if(TRUE)")
    if missing_flag is not None:
        text = text.replace(f" {missing_flag}", "")
    cmake.write_text(text, encoding="utf-8")
    original_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        text = original_read_text(path, *args, **kwargs)
        if path.name == "CMakeCXXCompiler.cmake" and path.is_relative_to(build):
            text += (
                f'\nset(CMAKE_CXX_COMPILER_ID "{compiler_id}")\n'
                f'set(CMAKE_CXX_SIMULATE_ID "{simulate_id}")\n'
            )
        return text

    monkeypatch.setattr(Path, "read_text", read_text)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    if missing_flag is None:
        assert errors == []
    else:
        assert any("only the platform probe flags" in error for error in errors)


def test_cmake_semantic_graph_contact_surface_probe_rejects_fake_source(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    _, source = _add_contact_surface_probe_fixture(tmp_path)
    source.write_text("int main() { return 0; }\n", encoding="utf-8")
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "source does not match the contact-surface probe" in error for error in errors
    )


@pytest.mark.parametrize(
    ("property_name", "value", "expected"),
    (
        ("DISABLED", "TRUE", "is disabled"),
        ("WILL_FAIL", "TRUE", "result-neutralizing property"),
        ("ENVIRONMENT", '"CPATH=extra"', "cannot modify its environment"),
    ),
)
def test_cmake_semantic_graph_contact_surface_probe_preserves_execution_checks(
    tmp_path, property_name, value, expected
):
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake, _ = _add_contact_surface_probe_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + f"\nset_tests_properties(ContactSurfaceProbe PROPERTIES {property_name} {value})\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(expected in error and "ContactSurfaceProbe" in error for error in errors)


@pytest.mark.parametrize("build_testing", (None, "ON", "OFF"))
def test_repository_ctest_initialization_generates_the_full_dart_subtree(
    tmp_path, build_testing
):
    ordering = [
        (name, arguments, scope)
        for name, arguments, scope in infra.cmake_scoped_commands(
            (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        )
        if (name, arguments) in {("include", "CTest"), ("add_subdirectory", "dart")}
    ]
    assert len(ordering) == 2
    assert all(scope == () for _, _, scope in ordering)
    tmp_path.joinpath("CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22)\n"
        "project(CtestInitialization NONE)\n"
        + "".join(f"{name}({arguments})\n" for name, arguments, _ in ordering)
        + "if(BUILD_TESTING)\n"
        "  add_test(NAME semantic COMMAND ${CMAKE_COMMAND} -E true)\n"
        "endif()\n",
        encoding="utf-8",
    )
    child = tmp_path / "dart/child"
    child.mkdir(parents=True)
    child.parent.joinpath("CMakeLists.txt").write_text(
        "add_subdirectory(child)\n", encoding="utf-8"
    )
    child.joinpath("CMakeLists.txt").write_text("# No tests here.\n", encoding="utf-8")
    build = tmp_path / "build"
    command = ["cmake", "-S", str(tmp_path), "-B", str(build)]
    if build_testing is not None:
        command.append(f"-DBUILD_TESTING={build_testing}")

    result = subprocess.run(command, capture_output=True, text=True)

    assert result.returncode == 0, result.stdout + result.stderr
    if build_testing == "OFF":
        assert not (build / "CTestTestfile.cmake").exists()
        assert not (build / "dart/child/CTestTestfile.cmake").exists()
    else:
        errors = []
        registrations = infra.generated_ctest_registrations(build, "Release", errors)
        assert errors == []
        assert set(registrations) == {"semantic"}
        assert (build / "dart/child/CTestTestfile.cmake").is_file()


def _add_demo_semantic_fixture(root: Path, labels: str = "gui", frames: str = "2"):
    cmake = root / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8") + "\nadd_subdirectory(examples/demos)\n",
        encoding="utf-8",
    )
    demos = root / "examples/demos"
    demos.mkdir(parents=True)
    demos.joinpath("main.cpp").write_text(
        "int main() { return 0; }\n", encoding="utf-8"
    )
    demos.joinpath("CMakeLists.txt").write_text(
        "add_executable(dart-demos main.cpp)\n"
        "add_dependencies(tests_and_run dart-demos)\n"
        f"add_test(NAME examples_demos_cycle COMMAND dart-demos --cycle-scenes --frames {frames})\n"
        f'set_tests_properties(examples_demos_cycle PROPERTIES LABELS "{labels}")\n',
        encoding="utf-8",
    )
    return demos


@pytest.mark.parametrize(
    ("labels", "frames", "expected"),
    (
        ("gui", "2", None),
        ("gui;gui-device", "2", "cannot have the `gui-device` label"),
        ("gui", "0", "unapproved command arguments"),
    ),
)
def test_cmake_semantic_graph_includes_demo_labels_and_arguments(
    tmp_path, labels, frames, expected
):
    build = _configure_semantic_graph_fixture(tmp_path)
    _add_demo_semantic_fixture(tmp_path, labels, frames)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    if expected is None:
        assert errors == []
    else:
        assert any(
            expected in error and "examples_demos_cycle" in error for error in errors
        )


def test_cmake_broad_gate_runs_demo_tests_from_the_dart_build_root(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    demos = _add_demo_semantic_fixture(tmp_path)
    demos.joinpath("main.cpp").write_text(
        '#include <fstream>\nint main() { std::ofstream("demo-ran") << "ran"; return 0; }\n',
        encoding="utf-8",
    )
    errors = []
    infra.check_cmake_test_graph(tmp_path, build, errors)
    assert errors == []

    result = subprocess.run(
        [
            "cmake",
            "--build",
            str(build),
            "--target",
            "tests_and_run",
            "--config",
            "Release",
            "--parallel",
            "2",
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert (build / "examples/demos/demo-ran").is_file()


@pytest.mark.parametrize("poison_variable", (False, True))
def test_cmake_semantic_graph_rejects_narrowed_ctest_working_directory(
    tmp_path, poison_variable
):
    build = _configure_semantic_graph_fixture(tmp_path)
    if poison_variable:
        cmake = tmp_path / "CMakeLists.txt"
        old = "set(DART_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})"
        new = "set(DART_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/tests)"
        expected = "expanded `tests_and_run` command does not invoke"
    else:
        cmake = tmp_path / "tests/CMakeLists.txt"
        old = 'WORKING_DIRECTORY "${DART_BINARY_DIR}"'
        new = 'WORKING_DIRECTORY "${DART_BINARY_DIR}/tests"'
        expected = "does not originate from its validated source contract"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(old, new, 1),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    ("enabled", "registered", "dependency", "expected"),
    (
        (False, True, True, None),
        (True, True, True, None),
        (True, False, True, "not registered with CTest"),
        (True, True, False, "`tests_and_run` omits"),
    ),
)
def test_cmake_semantic_graph_owns_opt_in_gui_smoke(
    tmp_path, enabled, registered, dependency, expected
):
    build = _configure_semantic_graph_fixture(tmp_path)
    root_cmake = tmp_path / "CMakeLists.txt"
    root_cmake.write_text(
        root_cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(tests)",
            f"set(DART_ENABLE_GUI_SMOKE_TESTS {'ON' if enabled else 'OFF'})\n"
            "add_subdirectory(tests)",
            1,
        ),
        encoding="utf-8",
    )
    tests_cmake = tmp_path / "tests/CMakeLists.txt"
    tests_cmake.write_text(
        tests_cmake.read_text(encoding="utf-8")
        + "\nadd_subdirectory(benchmark/integration)\n",
        encoding="utf-8",
    )
    smoke_dir = tmp_path / "tests/benchmark/integration"
    smoke_dir.mkdir(parents=True)
    smoke_dir.joinpath("offscreen_capture_smoke.cpp").write_text(
        "int main() { return 0; }\n", encoding="utf-8"
    )
    smoke_dir.joinpath("CMakeLists.txt").write_text(
        "if(DART_ENABLE_GUI_SMOKE_TESTS)\n"
        "  add_executable(offscreen_capture_smoke offscreen_capture_smoke.cpp)\n"
        + (
            "  add_test(NAME offscreen_capture_smoke COMMAND offscreen_capture_smoke)\n"
            '  set_tests_properties(offscreen_capture_smoke PROPERTIES LABELS "gui;gui-device")\n'
            if registered
            else ""
        )
        + (
            "  add_dependencies(tests_and_run offscreen_capture_smoke)\n"
            if dependency
            else ""
        )
        + "endif()\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    if expected is None:
        assert errors == []
    else:
        assert any(
            expected in error and "offscreen_capture_smoke" in error for error in errors
        )


def test_cmake_semantic_graph_probe_accepts_release_multi_config_graph(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path, "Ninja Multi-Config")
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_accepts_escaped_windows_path(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path, force_windows=True)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_accepts_legacy_multi_config_tests(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path, "Ninja Multi-Config")
    replacements = {
        "tests/integration/CMakeLists.txt": (
            "add_test(NAME INTEGRATION_semantic COMMAND INTEGRATION_semantic)",
            "add_test(INTEGRATION_semantic INTEGRATION_semantic)",
        ),
        "tests/regression/CMakeLists.txt": (
            "add_test(NAME test_semantic_regression COMMAND test_semantic_regression)",
            "add_test(test_semantic_regression test_semantic_regression)",
        ),
        "tests/unit/CMakeLists.txt": (
            "add_test(NAME UNIT_semantic COMMAND UNIT_semantic)",
            "add_test(UNIT_semantic UNIT_semantic)",
        ),
    }
    for relative, (old, new) in replacements.items():
        path = tmp_path / relative
        path.write_text(
            path.read_text(encoding="utf-8").replace(old, new, 1),
            encoding="utf-8",
        )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_rejects_unowned_legacy_registration(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path, "Ninja Multi-Config")
    cmake = tmp_path / "tests/regression/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8") + "\nadd_test(ghost_test ghost_test)\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest commands do not map to configured C++ test targets" in error
        and "ghost_test" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_external_build_directory(tmp_path):
    root = tmp_path / "repo"
    root.mkdir()
    errors = []

    infra.check_cmake_test_graph(root, tmp_path / "outside", errors)

    assert errors == [
        "CMake semantic test graph: build directory must be inside "
        f"`{root / 'build'}`"
    ]


def test_cmake_build_discovery_accepts_cmake_slash_spelling(tmp_path, monkeypatch):
    build = tmp_path / "build" / "default" / "cpp" / "Release"
    build.mkdir(parents=True)
    cache_home = str(tmp_path.resolve()).replace("/", "\\")
    (build / "CMakeCache.txt").write_text(
        f"CMAKE_HOME_DIRECTORY:INTERNAL={cache_home}\n",
        encoding="utf-8",
    )
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "default")
    monkeypatch.setenv("BUILD_TYPE", "Release")
    monkeypatch.setattr(infra.sys, "platform", "linux")

    assert infra.discover_cmake_build_dir(tmp_path) == build


def test_cmake_build_discovery_uses_windows_multi_config_owner(tmp_path, monkeypatch):
    base = tmp_path / "build" / "default" / "cpp"
    nested = base / "Release"
    nested.mkdir(parents=True)
    cache = f"CMAKE_HOME_DIRECTORY:INTERNAL={tmp_path.resolve()}\n"
    (base / "CMakeCache.txt").write_text(cache, encoding="utf-8")
    (nested / "CMakeCache.txt").write_text(cache, encoding="utf-8")
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "default")
    monkeypatch.setenv("BUILD_TYPE", "Release")
    monkeypatch.setattr(infra.sys, "platform", "win32")

    assert infra.discover_cmake_build_dir(tmp_path) == base


@pytest.mark.parametrize(
    ("compiler_id", "simulate_id", "expected", "flags"),
    (
        (
            "MSVC",
            "",
            True,
            ["/std:c++17", "/permissive-", "/Zc:twoPhase-", "/Zs"],
        ),
        (
            "Clang",
            "MSVC",
            True,
            ["/std:c++17", "/permissive-", "/Zc:twoPhase-", "/Zs"],
        ),
        ("GNU", "", False, ["-std=c++14", "-fsyntax-only"]),
    ),
)
def test_contact_surface_probe_flags_follow_configured_compiler_state(
    tmp_path, compiler_id, simulate_id, expected, flags
):
    build = tmp_path / "build"
    compiler = build / "CMakeFiles" / "4.3.3" / "CMakeCXXCompiler.cmake"
    compiler.parent.mkdir(parents=True)
    compiler.write_text(
        f'set(CMAKE_CXX_COMPILER_ID "{compiler_id}")\n'
        f'set(CMAKE_CXX_SIMULATE_ID "{simulate_id}")\n',
        encoding="utf-8",
    )
    cache = {
        "CMAKE_CACHE_MAJOR_VERSION": "4",
        "CMAKE_CACHE_MINOR_VERSION": "3",
        "CMAKE_CACHE_PATCH_VERSION": "3",
        "CMAKE_CXX_COMPILER": "configured-cxx",
        "Eigen3_DIR": str(tmp_path / "eigen"),
    }
    errors = []

    actual = infra.cmake_compiler_uses_msvc_abi(build, cache, errors)

    assert errors == []
    assert actual is expected

    source = tmp_path / "tests/cmake/ContactSurfaceProbe.cpp"
    source.parent.mkdir(parents=True)
    source.write_text(
        (ROOT / "tests/cmake/ContactSurfaceProbe.cpp").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    trace = [
        {
            "cmd": "set_target_properties",
            "file": str(tmp_path / "eigen/Eigen3Targets.cmake"),
            "args": [
                "Eigen3::Eigen",
                "PROPERTIES",
                "INTERFACE_INCLUDE_DIRECTORIES",
                str(tmp_path / "eigen/include"),
            ],
        }
    ]

    command = infra.contact_surface_probe_command(tmp_path, build, cache, trace, errors)

    assert errors == []
    assert command is not None
    assert command[1 : 1 + len(flags)] == flags
    cmake = (ROOT / "tests/CMakeLists.txt").read_text(encoding="utf-8")
    start = cmake.index("get_target_property(\n  contact_surface_eigen_includes")
    probe = cmake[start : cmake.index("dart_format_add(cmake/ContactSurfaceProbe.cpp)")]
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22)\n"
        "project(contact_surface_probe NONE)\n"
        f'include("{ROOT.as_posix()}/cmake/dart_defs.cmake")\n'
        "enable_testing()\n"
        f"set(MSVC {'TRUE' if expected else 'FALSE'})\n"
        'set(CMAKE_CXX_COMPILER "configured-cxx")\n'
        "set(DART_SOURCE_DIR ${PROJECT_SOURCE_DIR})\n"
        "set(DART_BINARY_DIR ${PROJECT_BINARY_DIR})\n"
        "add_library(Eigen3::Eigen INTERFACE IMPORTED)\n"
        "set_target_properties(Eigen3::Eigen PROPERTIES "
        'INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/eigen/include")\n'
        + probe,
        encoding="utf-8",
    )
    result = subprocess.run(
        ["cmake", "-S", str(tmp_path), "-B", str(build)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    generated = infra.generated_ctest_registrations(build, "Release", errors)
    assert errors == []
    assert generated["ContactSurfaceProbe"][0] == command


def test_cmake_semantic_graph_probe_requires_fresh_file_api_reply(
    tmp_path, monkeypatch
):
    build = _configure_semantic_graph_fixture(tmp_path)
    errors = []
    infra.check_cmake_test_graph(tmp_path, build, errors)
    assert errors == []

    monkeypatch.setattr(
        infra.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args[0], 0, "", ""),
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == [
        "CMake semantic test graph: CMake File API did not emit a fresh index"
    ]


def test_cmake_semantic_graph_probe_checks_post_configure_cache(tmp_path):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + '\nset(DART_BUILD_DARTPY OFF CACHE BOOL "" FORCE)\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "configured `DART_BUILD_DARTPY` must be `ON`" in error for error in errors
    )


def test_cmake_semantic_graph_probe_requires_release_configuration(tmp_path):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + '\nset(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("must use `CMAKE_BUILD_TYPE=Release`" in error for error in errors)


def test_cmake_semantic_graph_probe_rejects_omitted_test_subtree(tmp_path):
    cmake = tmp_path / "tests" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(regression)\n",
            "",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "configured test graph omits" in error and "tests/regression" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_unregistered_ctest_target(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_test(NAME test_semantic_regression COMMAND "
            "test_semantic_regression)\n",
            "",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("not registered with CTest" in error for error in errors)


def test_cmake_semantic_graph_probe_rejects_disabled_ctest_target(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression "
        "PROPERTIES DISABLED TRUE)\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest test `test_semantic_regression` is disabled" in error for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_list_only_ctest_target(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "COMMAND test_semantic_regression)",
            "COMMAND test_semantic_regression --gtest_list_tests)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest test `test_semantic_regression` uses non-executing option "
        "`--gtest_list_tests`" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_unapproved_gtest_environment(
    tmp_path,
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression "
        'PROPERTIES ENVIRONMENT "GTEST_FILTER=-*")\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest test `test_semantic_regression` has unapproved GTest "
        "environment ['GTEST_FILTER=-*']" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_gtest_environment_modification(
    tmp_path,
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression PROPERTIES "
        'ENVIRONMENT_MODIFICATION "GTEST_FILTER=set:-*")\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "CTest test `test_semantic_regression` has unapproved GTest "
        "environment modifications" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("property_name", "value"),
    (
        ("WILL_FAIL", "TRUE"),
        ("PASS_REGULAR_EXPRESSION", '".*"'),
        ("SKIP_REGULAR_EXPRESSION", '".*"'),
        ("SKIP_RETURN_CODE", "1"),
    ),
)
def test_cmake_semantic_graph_probe_rejects_result_neutralizing_properties(
    tmp_path, property_name, value
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression PROPERTIES "
        f"{property_name} {value})\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        f"CTest test `test_semantic_regression` uses result-neutralizing "
        f"property `{property_name}`" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_requires_zero_test_guard_for_selection(
    tmp_path, monkeypatch
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "COMMAND test_semantic_regression)",
            "COMMAND test_semantic_regression --gtest_filter=Semantic.*)",
            1,
        ),
        encoding="utf-8",
    )
    monkeypatch.setitem(
        infra.APPROVED_CTEST_SELECTIONS,
        "test_semantic_regression",
        {
            "arguments": ("--gtest_filter=Semantic.*",),
            "environment": (),
        },
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "selected CTest test `test_semantic_regression` does not fail on zero "
        "tests" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_ambiguous_zero_test_guard(
    tmp_path, monkeypatch
):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_test(NAME test_semantic_regression COMMAND "
            "test_semantic_regression)\n",
            "add_test(NAME test_semantic_regression COMMAND "
            "test_semantic_regression --gtest_filter=Semantic.*)\n"
            "set_tests_properties(test_semantic_regression PROPERTIES "
            'FAIL_REGULAR_EXPRESSION "0 tests")\n',
            1,
        ),
        encoding="utf-8",
    )
    monkeypatch.setitem(
        infra.APPROVED_CTEST_SELECTIONS,
        "test_semantic_regression",
        {
            "arguments": ("--gtest_filter=Semantic.*",),
            "environment": (),
        },
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "does not fail on zero tests with exact pattern" in error for error in errors
    )


def test_cmake_test_runner_clears_future_ambient_gtest_controls(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    source = tmp_path / "tests" / "regression" / "test_semantic.cpp"
    source.write_text(
        """
#include <cstdlib>
#include <fstream>

int main()
{
  if (std::getenv("GTEST_FUTURE_SELECTOR") != nullptr)
    return 0;
  std::ofstream("semantic-body-ran") << "ran";
  return 0;
}
""".lstrip(),
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["GTEST_FUTURE_SELECTOR"] = "skip-everything"

    result = subprocess.run(
        [
            "cmake",
            "--build",
            str(build),
            "--target",
            "tests_and_run",
            "--config",
            "Release",
        ],
        capture_output=True,
        text=True,
        env=environment,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert (build / "tests" / "regression" / "semantic-body-ran").is_file()


def _copy_test_runner_probe_contract(root: Path) -> None:
    for relative in (
        "pyproject.toml",
        "cmake/DARTRunCTest.cmake",
        "scripts/run_pytest.py",
    ):
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            (ROOT / relative).read_text(encoding="utf-8"),
            encoding="utf-8",
        )


def test_test_runner_semantic_probe_accepts_repository_contract(tmp_path):
    _copy_test_runner_probe_contract(tmp_path)
    errors = []

    infra.check_test_runner_semantics(tmp_path, errors)

    assert errors == []


def test_test_runner_semantic_probe_stays_on_repository_volume(tmp_path, monkeypatch):
    _copy_test_runner_probe_contract(tmp_path)
    original = infra.tempfile.TemporaryDirectory
    parents = []

    def tracked_temporary_directory(*args, **kwargs):
        parents.append(Path(kwargs["dir"]).resolve())
        return original(*args, **kwargs)

    monkeypatch.setattr(
        infra.tempfile,
        "TemporaryDirectory",
        tracked_temporary_directory,
    )
    errors = []

    infra.check_test_runner_semantics(tmp_path, errors)

    assert errors == []
    assert parents == [(tmp_path / "build").resolve()]


@pytest.mark.parametrize(
    ("relative", "old", "new", "expected"),
    (
        (
            "cmake/DARTRunCTest.cmake",
            "execute_process(\n  COMMAND\n",
            "set(_dart_ctest_arguments --show-only)\n" "execute_process(\n  COMMAND\n",
            "CTest did not execute a body",
        ),
        (
            "scripts/run_pytest.py",
            'name.upper().startswith("PYTEST_")',
            'name.upper().startswith("UNRELATED_")',
            "pytest did not execute a body",
        ),
    ),
)
def test_test_runner_semantic_probe_rejects_execution_bypass(
    tmp_path, relative, old, new, expected
):
    _copy_test_runner_probe_contract(tmp_path)
    path = tmp_path / relative
    text = path.read_text(encoding="utf-8")
    if relative.startswith("cmake/"):
        offset = text.rfind(old)
        assert offset > 0
        text = text[:offset] + text[offset:].replace(old, new, 1)
    else:
        assert old in text
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    errors = []

    infra.check_test_runner_semantics(tmp_path, errors)

    assert any(expected in error for error in errors)


def test_cmake_zero_test_guard_does_not_match_ten_tests(tmp_path, monkeypatch):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nset_tests_properties(test_semantic_regression PROPERTIES "
        'ENVIRONMENT "GTEST_FILTER=Semantic.*" '
        f'FAIL_REGULAR_EXPRESSION "{infra.SAFE_ZERO_TEST_PATTERN}")\n',
        encoding="utf-8",
    )
    (tmp_path / "tests" / "regression" / "test_semantic.cpp").write_text(
        """
#include <iostream>

int main()
{
  std::cout << "10 tests from 1 test suite ran." << std::endl;
  return 0;
}
""".lstrip(),
        encoding="utf-8",
    )
    monkeypatch.setitem(
        infra.APPROVED_CTEST_SELECTIONS,
        "test_semantic_regression",
        {
            "arguments": (),
            "environment": ("GTEST_FILTER=Semantic.*",),
        },
    )
    errors = []
    infra.check_cmake_test_graph(tmp_path, build, errors)
    assert errors == []

    result = subprocess.run(
        [
            "cmake",
            "--build",
            str(build),
            "--target",
            "tests_and_run",
            "--config",
            "Release",
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_cmake_pytest_target_ignores_repository_conftest(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    test_source = tmp_path / "python" / "tests" / "test_semantic.py"
    test_source.write_text(
        """
from pathlib import Path


def test_semantic():
    Path(__file__).with_name("semantic-body-ran").write_text("ran")
""".lstrip(),
        encoding="utf-8",
    )
    (test_source.parent / "conftest.py").write_text(
        """
def pytest_collection_modifyitems(items):
    items.clear()


def pytest_sessionfinish(session):
    session.exitstatus = 0
""".lstrip(),
        encoding="utf-8",
    )
    errors = []
    infra.check_cmake_test_graph(tmp_path, build, errors)
    assert errors == []

    result = subprocess.run(
        [
            "cmake",
            "--build",
            str(build),
            "--target",
            "pytest",
            "--config",
            "Release",
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert (test_source.parent / "semantic-body-ran").is_file()


def test_cmake_semantic_graph_probe_rejects_explicit_local_pytest_plugin(
    tmp_path,
):
    build = _configure_semantic_graph_fixture(tmp_path)
    test_source = tmp_path / "python" / "tests" / "test_semantic.py"
    test_source.write_text(
        'pytest_plugins = ["evil"]\n\n\ndef test_semantic():\n    pass\n',
        encoding="utf-8",
    )
    (test_source.parent / "evil.py").write_text(
        "def pytest_collection_modifyitems(items):\n    items.clear()\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "declares `pytest_plugins`; local plugin loading is not allowed" in error
        for error in errors
    )


def test_cmake_semantic_graph_ignores_unreferenced_stale_ctest_file(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    stale = build / "tests" / "stale"
    stale.mkdir()
    (stale / "CTestTestfile.cmake").write_text(
        'add_test([=[stale_list_only]=] "test_semantic" ' '"--gtest_list_tests")\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_rejects_unowned_test_source(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    (tmp_path / "tests" / "unit" / "test_orphan.cpp").write_text(
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "neither configured nor explicitly approved as inactive" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_nested_early_return(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        "return()\n" + cmake.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "neither configured nor explicitly approved as inactive" in error
        for error in errors
    )


@pytest.mark.parametrize(
    "inactive_declaration",
    (
        """
if(FALSE)
  add_executable(test_orphan test_orphan.cpp)
endif()
""",
        """
function(register_orphan)
  add_executable(test_orphan test_orphan.cpp)
endfunction()
""",
    ),
)
def test_cmake_semantic_graph_probe_rejects_inactive_source_owner(
    tmp_path, inactive_declaration
):
    cmake = tmp_path / "tests" / "unit" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8") + inactive_declaration,
        encoding="utf-8",
    )
    (tmp_path / "tests" / "unit" / "test_orphan.cpp").write_text(
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "neither configured nor explicitly approved as inactive" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_accepts_explicit_inactive_source_policy(
    tmp_path,
):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(tests)",
            'set(DART_USE_SYSTEM_IMGUI ON CACHE BOOL "" FORCE)\n'
            "add_library(dart-external-imgui INTERFACE)\n"
            "add_subdirectory(tests)",
            1,
        ),
        encoding="utf-8",
    )
    regression = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    regression.write_text(
        regression.read_text(encoding="utf-8")
        + """
if(TARGET dart-external-imgui)
  if(NOT DART_USE_SYSTEM_IMGUI)
    dart_add_test("regression" test_Issue2516)
  endif()
endif()
""",
        encoding="utf-8",
    )
    (tmp_path / "tests" / "regression" / "test_Issue2516.cpp").write_text(
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def _add_inactive_gui_sources(root: Path) -> Path:
    names = (
        "GuiDescriptors",
        "GuiInteractions",
        "GuiViewer",
        "GuiParity",
        "GuiImageDecoders",
        "GuiMesh",
        "FilamentBackend",
    )
    cmake = root / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(tests)",
            'set(DART_BUILD_GUI OFF CACHE BOOL "" FORCE)\n' "add_subdirectory(tests)",
            1,
        ),
        encoding="utf-8",
    )
    integration = root / "tests/integration/CMakeLists.txt"
    integration.write_text(
        integration.read_text(encoding="utf-8")
        + "\nif(TARGET dart-gui)\n"
        + "".join(f'  dart_add_test("integration" test_{name})\n' for name in names)
        + "endif()\n",
        encoding="utf-8",
    )
    for name in names:
        (integration.parent / f"test_{name}.cpp").write_text(
            "int main(void) { return 0; }\n", encoding="utf-8"
        )
    return integration


def test_cmake_semantic_graph_probe_accepts_gui_disabled_sources(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    _add_inactive_gui_sources(tmp_path)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


@pytest.mark.parametrize(
    ("old", "new"),
    (
        ('  dart_add_test("integration" test_GuiParity)', ""),
        ("if(TARGET dart-gui)", "if(FALSE)"),
        ("if(TARGET dart-gui)", "if(FALSE)\nif(TARGET dart-gui)"),
        ("if(TARGET dart-gui)", "return()\nif(TARGET dart-gui)"),
        (
            '  dart_add_test("integration" test_GuiParity)',
            '  dart_add_test("integration" test_GuiParity)\n'
            '  dart_add_test("integration" test_GuiParity)',
        ),
    ),
)
def test_cmake_semantic_graph_probe_rejects_gui_inactive_owner_drift(
    tmp_path, old, new
):
    build = _configure_semantic_graph_fixture(tmp_path)
    integration = _add_inactive_gui_sources(tmp_path)
    text = integration.read_text(encoding="utf-8").replace(old, new, 1)
    if new.startswith("if(FALSE)\n"):
        text += "endif()\n"
    integration.write_text(text, encoding="utf-8")
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "neither configured nor explicitly approved as inactive" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_gui_enabled_missing_targets(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    _add_inactive_gui_sources(tmp_path)
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "set(DART_BUILD_GUI OFF", "set(DART_BUILD_GUI ON", 1
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "neither configured nor explicitly approved as inactive" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_library_only_runtime_test(tmp_path):
    cmake = tmp_path / "tests" / "regression" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_executable(test_semantic_regression test_semantic.cpp)",
            "add_library(test_semantic_regression STATIC test_semantic.cpp)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("belong only to non-executable targets" in error for error in errors)


def _add_compile_only_semantic_fixture(
    root: Path, *, target_type: str = "OBJECT", dependency: bool = True
) -> None:
    unit = root / "tests" / "unit" / "CMakeLists.txt"
    dependency_name = " UNIT_math_LegacyConvexHullC" if dependency else ""
    unit.write_text(
        unit.read_text(encoding="utf-8").replace(
            "set(unit_tests UNIT_semantic PARENT_SCOPE)",
            "add_subdirectory(math)\n"
            f"set(unit_tests UNIT_semantic{dependency_name} PARENT_SCOPE)",
            1,
        ),
        encoding="utf-8",
    )
    math = root / "tests" / "unit" / "math"
    math.mkdir()
    (math / "CMakeLists.txt").write_text(
        "\n" * 10 + f"add_library(UNIT_math_LegacyConvexHullC {target_type} "
        "test_LegacyConvexHullC.c)\n",
        encoding="utf-8",
    )
    (math / "test_LegacyConvexHullC.c").write_text(
        "int legacy_convex_hull_c(void) { return 0; }\n",
        encoding="utf-8",
    )


def test_cmake_semantic_graph_accepts_exact_compile_only_contract(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    _add_compile_only_semantic_fixture(tmp_path)
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


@pytest.mark.parametrize(
    ("target_type", "dependency"),
    (("STATIC", True), ("OBJECT", False)),
)
def test_cmake_semantic_graph_rejects_compile_only_contract_drift(
    tmp_path, target_type, dependency
):
    build = _configure_semantic_graph_fixture(tmp_path)
    _add_compile_only_semantic_fixture(
        tmp_path, target_type=target_type, dependency=dependency
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "compile-only C++ test sources do not match their exact object-target "
        "contract" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_pytest_non_execution_flags(tmp_path):
    cmake = tmp_path / "python" / "tests" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "${dartpy_test_files} -v",
            "${dartpy_test_files} -v --collect-only",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "configured `pytest` does not originate from its validated source "
        "contract" in error
        or "expanded `pytest` command does not invoke" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("old", "new"),
    (
        (
            '"${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"',
            '"${PROJECT_SOURCE_DIR}/scripts/Noop.py"',
        ),
        (
            '"${Python3_EXECUTABLE}" -I '
            '"${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"',
            '"${Python3_EXECUTABLE}" -m pytest',
        ),
    ),
)
def test_cmake_semantic_graph_probe_rejects_unpinned_pytest_target(tmp_path, old, new):
    cmake = tmp_path / "python" / "tests" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    text = cmake.read_text(encoding="utf-8")
    assert old in text
    cmake.write_text(text.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "configured `pytest` does not originate from its validated source "
        "contract" in error
        or "expanded `pytest` command does not invoke" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_rejects_local_pytest_shadow(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    (tmp_path / "python" / "tests" / "pytest.py").write_text(
        '"""Local module that must not shadow the installed pytest package."""\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "pytest target resolution does not match the trusted" in error
        for error in errors
    )


def test_cmake_semantic_graph_probe_does_not_execute_spoofed_pytest(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    marker = tmp_path / "pytest-shadow-executed"
    (tmp_path / "python" / "tests" / "pytest.py").write_text(
        "from pathlib import Path\n"
        f"Path({str(marker)!r}).write_text('executed')\n"
        "__file__ = '/trusted/pixi/site-packages/pytest/__init__.py'\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "pytest target resolution does not match the trusted" in error
        for error in errors
    )
    assert not marker.exists()


def test_cmake_semantic_graph_probe_rejects_pytest_package_shadow(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    marker = tmp_path / "pytest-package-shadow-executed"
    package = tmp_path / "python" / "tests" / "pytest"
    package.mkdir()
    (package / "__init__.py").write_text(
        "from pathlib import Path\n" f"Path({str(marker)!r}).write_text('executed')\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "pytest target resolution does not match the trusted" in error
        for error in errors
    )
    assert not marker.exists()


def test_cmake_semantic_graph_probe_rejects_configured_pytest_collection_only(
    tmp_path,
):
    build = _configure_semantic_graph_fixture(tmp_path)
    (tmp_path / "pyproject.toml").write_text(
        '[tool.pytest.ini_options]\naddopts = ["--collect-only"]\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any(
        "pinned pytest options do not match the safe branch contract" in error
        for error in errors
    )


def test_cmake_semantic_graph_pins_root_pytest_config(tmp_path):
    build = _configure_semantic_graph_fixture(tmp_path)
    (tmp_path / "python" / "tests" / "pytest.ini").write_text(
        "[pytest]\naddopts = --collect-only\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)
    result = subprocess.run(
        ["cmake", "--build", str(build), "--target", "pytest"],
        capture_output=True,
        text=True,
    )

    assert errors == []
    assert result.returncode == 0, result.stdout + result.stderr
    assert "1 passed" in result.stdout + result.stderr


def test_cmake_semantic_graph_uses_exact_test_directory_boundaries(tmp_path):
    tests_cmake = tmp_path / "tests" / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    shadow = tmp_path / "tests" / "unit_shadow"
    shadow.mkdir()
    (shadow / "CMakeLists.txt").write_text(
        "add_executable(UNIT_shadow semantic.c)\n",
        encoding="utf-8",
    )
    (shadow / "semantic.c").write_text(
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    tests_cmake.write_text(
        tests_cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(unit)",
            "add_subdirectory(unit)\n" "add_subdirectory(unit_shadow)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_accepts_msvc_runtime_branch(tmp_path):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "add_subdirectory(tests)",
            "set(MSVC TRUE)\nadd_subdirectory(tests)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors == []


def test_cmake_semantic_graph_probe_rejects_pytest_fallback(tmp_path):
    cmake = tmp_path / "python/tests/CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "if(DARTPY_PYTEST_FOUND)",
            "set(DARTPY_PYTEST_FOUND FALSE)\nif(DARTPY_PYTEST_FOUND)",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("`pytest` does not depend on `dartpy`" in error for error in errors)
    assert any("validated source contract" in error for error in errors)


@pytest.mark.parametrize(
    ("needle", "injection"),
    (
        (
            "add_custom_target(ALL DEPENDS ${all_targets})",
            "cmake_language(CALL return)\n",
        ),
        (
            "add_custom_target(ALL DEPENDS ${all_targets})",
            "include_guard(GLOBAL)\ninclude_guard(GLOBAL)\n",
        ),
        (
            "add_custom_target(ALL DEPENDS ${all_targets})",
            "function(add_custom_target)\nendfunction()\n",
        ),
        (
            "if(BUILD_TESTING)",
            "function(list)\nendfunction()\n",
        ),
        (
            "if(BUILD_TESTING)",
            "set(BUILD_TESTING OFF)\n",
        ),
        (
            "add_custom_target(ALL DEPENDS ${all_targets})",
            (
                "add_custom_target(ALL)\n"
                "function(add_custom_target)\n"
                "endfunction()\n"
            ),
        ),
        (
            "add_subdirectory(tests)",
            (
                "function(add_custom_target)\n"
                '  if("${ARGV0}" STREQUAL "tests_and_run")\n'
                "    _add_custom_target(\n"
                "      tests_and_run\n"
                "      DEPENDS ${integration_tests} ${regression_tests} "
                "${unit_tests}\n"
                "    )\n"
                "  else()\n"
                "    _add_custom_target(${ARGV})\n"
                "  endif()\n"
                "endfunction()\n"
            ),
        ),
        (
            "add_subdirectory(tests)",
            'set(CMAKE_CTEST_COMMAND "${CMAKE_COMMAND};-E;true")\n',
        ),
        (
            "add_subdirectory(tests)",
            (
                'set(CMAKE_CTEST_COMMAND "${CMAKE_COMMAND}" '
                'CACHE FILEPATH "" FORCE)\n'
            ),
        ),
        (
            "add_subdirectory(python/tests)",
            'set(Python3_EXECUTABLE "${CMAKE_COMMAND}")\n',
        ),
        (
            "add_subdirectory(python/tests)",
            (
                'set(_Python3_EXECUTABLE "${CMAKE_COMMAND}" '
                'CACHE INTERNAL "" FORCE)\n'
                'set(Python3_EXECUTABLE "${CMAKE_COMMAND}")\n'
            ),
        ),
        (
            "add_subdirectory(python/tests)",
            'set(DART_PYTHONPATH "${CMAKE_BINARY_DIR}/unrelated")\n',
        ),
        (
            "add_subdirectory(python/tests)",
            'set(DART_PYTHONPATH "$<TARGET_FILE_DIR:dartpy>")\n',
        ),
    ),
)
def test_cmake_semantic_graph_probe_rejects_effective_bypasses(
    tmp_path, needle, injection
):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            needle,
            injection + needle,
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert errors
    assert all(error.startswith("CMake semantic test graph:") for error in errors)


def test_cmake_semantic_graph_probe_fails_closed_on_reconfigure_error(tmp_path):
    cmake = tmp_path / "CMakeLists.txt"
    build = _configure_semantic_graph_fixture(tmp_path)
    cmake.write_text(
        cmake.read_text(encoding="utf-8") + "\nunknown_graph_command()\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_cmake_test_graph(tmp_path, build, errors)

    assert any("configure probe failed" in error for error in errors)


def test_test_gate_contract_requires_multiconfig_ctest_selection(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "tests/CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "-DDART_CTEST_CONFIGURATION=$<CONFIG>",
            "-DDART_CTEST_CONFIGURATION=${CMAKE_BUILD_TYPE}",
            1,
        ),
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "tests/CMakeLists.txt: missing `test-all` graph marker "
        "`cmake/DARTRunCTest.cmake`" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("old", "new", "expected"),
    (
        (
            'set(DART_DARTPY_BUILD_DIR "$<TARGET_FILE_DIR:dartpy>")',
            'set(DART_DARTPY_BUILD_DIR "${DART_PYTHON_BUILD_DIR}/dartpy")',
            "`DART_DARTPY_BUILD_DIR` must derive from `$<TARGET_FILE_DIR:dartpy>`",
        ),
        (
            'set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}\\\\;'
            '${DART_PYTHON_BUILD_DIR}")',
            'set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}")',
            "`DART_PYTHONPATH` must combine the target output directory and "
            "Python build root under `if(WIN32)`",
        ),
        (
            'set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}:'
            '${DART_PYTHON_BUILD_DIR}")',
            'set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}")',
            "`DART_PYTHONPATH` must combine the target output directory and "
            "Python build root under the non-Windows branch",
        ),
    ),
)
def test_test_gate_contract_requires_config_aware_dartpy_path(
    tmp_path, old, new, expected
):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/CMakeLists.txt"
    text = cmake.read_text(encoding="utf-8")
    assert old in text
    cmake.write_text(text.replace(old, new, 1), encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(expected in error for error in errors)


def test_test_gate_contract_requires_dartpy_path_before_tests(tmp_path):
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/CMakeLists.txt"
    text = cmake.read_text(encoding="utf-8")
    output = 'set(DART_DARTPY_BUILD_DIR "$<TARGET_FILE_DIR:dartpy>")'
    condition = "add_subdirectory(dartpy)"
    assert output in text and condition in text
    text = text.replace(output, "", 1)
    cmake.write_text(
        text.replace(condition, f"{output}\n{condition}", 1), encoding="utf-8"
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert any(
        "define `dartpy`, derive its configuration-aware output path" in error
        for error in errors
    )


def test_dartpy_runtime_path_contract_requires_nanobind_target(tmp_path):
    binder = "dartpy"
    _copy_test_gate_contract(tmp_path)
    cmake = tmp_path / "python/CMakeLists.txt"
    text = cmake.read_text(encoding="utf-8")
    cmake.write_text(text.replace(f"add_subdirectory({binder})", "", 1))
    errors = []

    infra.check_dartpy_runtime_path_contract(tmp_path, errors)

    assert any(f"`add_subdirectory({binder})` must define" in error for error in errors)


def test_test_gate_contract_rejects_stale_task_handoff_semantics(tmp_path):
    _copy_test_gate_contract(tmp_path)
    handoff = tmp_path / "docs/dev_tasks/example/HANDOFF.md"
    handoff.parent.mkdir(parents=True)
    handoff.write_text(
        "- If `test-all` reports a Debug nanobind failure, clear its cache.\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/HANDOFF.md:1: `test-all` is a Release-only "
        "build task; remove Debug/nanobind guidance"
    ]


def test_test_gate_contract_rejects_unsupported_nanobind_cache_advice(tmp_path):
    _copy_test_gate_contract(tmp_path)
    packet = tmp_path / "docs/dev_tasks/example/07-work-packet.md"
    packet.parent.mkdir(parents=True)
    packet.write_text(
        '- Debug "nanobind not found" is the known poisoned-CMakeCache issue.\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/07-work-packet.md:1: remove unsupported "
        "nanobind cache guidance"
    ]


@pytest.mark.parametrize(
    "claim",
    [
        "`pixi run test-all` only builds and does not run tests.",
        "`test-all` does not run CTest or pytest.",
        "`test-all` is not runtime coverage.",
    ],
)
def test_test_gate_contract_rejects_false_no_runtime_claims(tmp_path, claim):
    _copy_test_gate_contract(tmp_path)
    handoff = tmp_path / "docs/dev_tasks/example/HANDOFF.md"
    handoff.parent.mkdir(parents=True)
    handoff.write_text(f"- {claim}\n", encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/HANDOFF.md:1: `test-all` must not be "
        "described as build-only or lacking runtime coverage"
    ]


@pytest.mark.parametrize(
    "claim",
    [
        "`pixi run test-all` is the full lint/build/test aggregate.",
        "`test-all` runs lint before building and testing.",
    ],
)
def test_test_gate_contract_rejects_false_lint_claims(tmp_path, claim):
    _copy_test_gate_contract(tmp_path)
    handoff = tmp_path / "docs/dev_tasks/example/HANDOFF.md"
    handoff.parent.mkdir(parents=True)
    handoff.write_text(f"- {claim}\n", encoding="utf-8")
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == [
        "docs/dev_tasks/example/HANDOFF.md:1: `test-all` must not be "
        "described as providing lint coverage"
    ]


def test_test_gate_contract_accepts_runtime_graph_claim(tmp_path):
    _copy_test_gate_contract(tmp_path)
    handoff = tmp_path / "docs/dev_tasks/example/HANDOFF.md"
    handoff.parent.mkdir(parents=True)
    handoff.write_text(
        "- `test-all` provides C++ and Python runtime coverage through the "
        "CMake `ALL` graph; run lint separately.\n",
        encoding="utf-8",
    )
    errors = []

    infra.check_test_gate_contract(tmp_path, errors)

    assert errors == []


def test_agent_hook_path_routing_is_bounded():
    assert hook.is_ai_infrastructure_path("docs/ai/README.md")
    assert hook.is_ai_infrastructure_path("docs/onboarding/architecture.md")
    assert hook.is_ai_infrastructure_path("docs/onboarding/testing.md")
    assert hook.is_ai_infrastructure_path(".codex/agents/dart_scout.toml")
    assert hook.is_ai_infrastructure_path(".github/workflows/ci_macos.yml")
    assert hook.is_ai_infrastructure_path("python/tests/unit/gui/test_agent_capture.py")
    assert hook.is_ai_infrastructure_path(
        "python/tests/unit/gui/test_agent_debug_overlay.py"
    )
    assert hook.is_ai_infrastructure_path(".gitignore")
    assert hook.is_ai_infrastructure_path("pixi.toml")
    assert hook.is_ai_infrastructure_path("dart/new_module/AGENTS.md")
    for path in (
        "CHANGELOG.md",
        "package.xml",
        "CMakeLists.txt",
        "cmake/DARTRunCTest.cmake",
        "python/CMakeLists.txt",
        "python/tests/CMakeLists.txt",
        "scripts/run_pytest.py",
        "tests/CMakeLists.txt",
    ):
        assert hook.is_ai_infrastructure_path(path)
    assert not hook.is_ai_infrastructure_path("dart/dynamics/BodyNode.cpp")


def test_nested_agent_instructions_are_drift_sources(tmp_path):
    nested = tmp_path / "dart" / "new_module" / "AGENTS.md"
    nested.parent.mkdir(parents=True)
    nested.write_text("nested instructions\n")

    assert nested in infra.source_paths(tmp_path)


def test_non_git_snapshot_enforces_nested_instruction_budget(tmp_path):
    root_agents = tmp_path / "AGENTS.md"
    nested = tmp_path / "dart" / "new_module" / "AGENTS.md"
    nested.parent.mkdir(parents=True)
    root_agents.write_text("r" * (infra.MAX_AGENT_INSTRUCTION_BYTES // 2 + 1))
    nested.write_text("n" * (infra.MAX_AGENT_INSTRUCTION_BYTES // 2 + 1))
    errors = []

    infra.check_instruction_budget(tmp_path, errors)

    assert any("instruction chain" in error for error in errors)


def test_untracked_personal_skill_is_not_related_dart_infrastructure(tmp_path):
    personal = tmp_path / ".agents" / "skills" / "personal-helper" / "SKILL.md"
    personal.parent.mkdir(parents=True)
    personal.write_text("---\nname: personal-helper\n---\n")

    assert not hook.is_related_worktree_ai_path(
        tmp_path, ".agents/skills/personal-helper/SKILL.md"
    )


def test_one_shot_setup_is_bounded_to_sync_and_hook_install():
    commands = setup.setup_commands()
    assert [command[1] for command in commands] == [
        "scripts/sync_ai_commands.py",
        "scripts/install_git_hooks.py",
        "scripts/check_ai_infrastructure.py",
    ]
    assert commands[-1][-1] == "--check"


def test_broken_relative_markdown_link_is_rejected(tmp_path):
    doc = tmp_path / "docs" / "ai" / "README.md"
    doc.parent.mkdir(parents=True)
    doc.write_text("[missing](../missing.md)\n")

    errors = infra.markdown_link_errors(tmp_path, doc)

    assert any("broken link" in error for error in errors)


def test_expected_agent_set_is_small_and_release_specific():
    assert infra.EXPECTED_AGENTS == {
        "dart_release_auditor",
        "dart_reviewer",
        "dart_scout",
    }


def test_model_upgrade_workflow_is_bounded_and_future_model_scalable():
    workflow = (ROOT / ".claude" / "commands" / "dart-model-upgrade.md").read_text()

    assert "workflow is itself an audit surface" in workflow
    assert "do not clone the workflow" in workflow
    assert "apply/adapt/omit" in workflow
    assert "durable project context" in workflow
    assert "representative DART physics investigation" in workflow
    assert "Images are never the" in workflow
    assert "sole correctness oracle" in workflow
    assert "configured CMake File API result" in workflow
    assert "source-marker matches" in workflow

    readme = (ROOT / "docs" / "ai" / "README.md").read_text()
    assert "## Updating Models And Coding Agents" in readme
    assert "**Codex — " in readme
    assert "**Claude Code — " in readme


@pytest.mark.parametrize("identity", ["DART 7", "DART 7.0", "DART 12.0.0"])
def test_model_upgrade_contract_accepts_versioned_identity(monkeypatch, identity):
    source = ROOT / ".claude/commands/dart-model-upgrade.md"
    original = source.read_text()
    changed = re.sub(
        r"\bDART(?= (?:physics investigation|nanobind binder))", identity, original
    )
    assert changed != original
    read_text = Path.read_text

    def changed_text(path, *args, **kwargs):
        if path == source:
            return changed
        return read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", changed_text)

    assert infra.exercise_scenarios(ROOT, emit=False) == []


def test_ultrawork_uses_explicit_delegation_and_lean_prompt_shape():
    workflow = (ROOT / ".claude" / "commands" / "dart-ultrawork.md").read_text()

    assert "Delegate only when the user explicitly requested it" in workflow
    assert "execute packets serially" in workflow
    assert "Do not repeat this workflow's logistics" in workflow
    assert "Sol Ultra" not in workflow


def _write_agent_profiles(root, *, malformed_instructions=False):
    agents_dir = root / ".codex" / "agents"
    agents_dir.mkdir(parents=True)
    for name in sorted(infra.EXPECTED_AGENTS):
        instructions = (
            "1"
            if malformed_instructions and name == "dart_scout"
            else '"Inputs: bounded evidence. Output: findings."'
        )
        (agents_dir / f"{name}.toml").write_text(
            f'name = "{name}"\n'
            'description = "Read-only test profile"\n'
            'sandbox_mode = "read-only"\n'
            f"developer_instructions = {instructions}\n"
        )


def test_malformed_codex_config_and_agent_toml_return_errors(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text('agents = "bad"\n')
    _write_agent_profiles(tmp_path, malformed_instructions=True)
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert ".codex/config.toml: agents must be a table" in errors
    assert any("instructions need Inputs and Output" in error for error in errors)


def test_boolean_codex_agent_limits_are_rejected(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        "[agents]\nmax_threads = 4\nmax_depth = true\n"
    )
    _write_agent_profiles(tmp_path)
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert ".codex/config.toml: agents.max_depth must be 1" in errors


@pytest.mark.parametrize(
    "key",
    (
        "model",
        "model_reasoning_effort",
        "review_model",
        "approval_policy",
        "sandbox_mode",
    ),
)
def test_project_codex_runtime_pins_are_rejected(tmp_path, key):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        f'{key} = "pinned"\n[agents]\nmax_threads = 4\nmax_depth = 1\n'
    )
    _write_agent_profiles(tmp_path)
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert any("project config must not pin" in error for error in errors)


def test_unexpected_project_codex_runtime_key_is_rejected(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        'experimental_override = "on"\n'
        "[agents]\n"
        "max_threads = 4\n"
        "max_depth = 1\n"
    )
    _write_agent_profiles(tmp_path)
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert ".codex/config.toml: root keys must equal agents" in errors


@pytest.mark.parametrize("key", ("model", "model_reasoning_effort", "review_model"))
def test_custom_agent_model_pins_are_rejected_and_reported(tmp_path, key):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        "[agents]\nmax_threads = 4\nmax_depth = 1\n"
    )
    _write_agent_profiles(tmp_path)
    profile = tmp_path / ".codex" / "agents" / "dart_scout.toml"
    profile.write_text(profile.read_text() + f'{key} = "pinned"\n')
    errors = []

    infra.check_codex_config(tmp_path, errors)
    pins = infra._model_harness_inventory(tmp_path)["model_pins"]["custom_agents"]

    assert any("keys must equal" in error for error in errors)
    assert any("inherit the parent model" in error for error in errors)
    assert pins == [
        {
            "path": ".codex/agents/dart_scout.toml",
            "keys": [key],
        }
    ]


def test_unexpected_custom_agent_key_is_rejected(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "config.toml").write_text(
        "[agents]\nmax_threads = 4\nmax_depth = 1\n"
    )
    _write_agent_profiles(tmp_path)
    profile = tmp_path / ".codex" / "agents" / "dart_scout.toml"
    profile.write_text(profile.read_text() + 'temperature = "high"\n')
    errors = []

    infra.check_codex_config(tmp_path, errors)

    assert any("keys must equal" in error for error in errors)


def test_doctor_report_inventories_model_context_and_visual_harness():
    report = infra.doctor_report(ROOT)

    assert report["schema_version"] == 1
    assert report["profile"] == {
        "name": "dart7",
        "cpp_standard": "C++17",
        "python_binding": "nanobind",
        "optional_python_bindings": [],
        "io_namespace": "dart::utils",
        "gui_backend": "Filament",
    }
    assert report["inventory"]["model_harness"]["model_pins"] == {
        "project": [],
        "project_agents": [],
        "custom_agents": [],
    }
    assert report["inventory"]["model_harness"]["generated_skill_metadata_chars"] > 0
    assert report["inventory"]["durable_context"]["owners"]["count"] == 5
    visual = report["inventory"]["visual_verification"]
    assert visual["backend"] == "Filament"
    assert "agent-capture" in visual["tasks"]
    assert "image-verdict" in visual["tasks"]
    assert "verification-bundle" in visual["tasks"]
    assert set(visual["capture_vulkan"]) == {"source", "icd_files", "readiness"}
    assert set(visual["native_window"]) == {"display", "xvfb_run"}
    json.dumps(report)


@pytest.mark.parametrize("variable", ("VK_DRIVER_FILES", "VK_ICD_FILENAMES"))
def test_visual_inventory_reports_selected_vulkan_icd(tmp_path, monkeypatch, variable):
    monkeypatch.setattr(infra.sys, "platform", "linux")
    monkeypatch.delenv("VK_DRIVER_FILES", raising=False)
    monkeypatch.delenv("VK_ICD_FILENAMES", raising=False)
    icd = tmp_path / "driver.json"
    icd.write_text("{}", encoding="utf-8")
    monkeypatch.setenv(
        variable, os.pathsep.join([str(tmp_path / "missing.json"), str(icd)])
    )
    monkeypatch.delenv("DISPLAY", raising=False)

    visual = infra._visual_verification_inventory(tmp_path)

    assert visual["capture_vulkan"] == {
        "source": variable,
        "icd_files": [str(tmp_path / "missing.json"), str(icd)],
        "readiness": "ICD available (device not probed)",
    }
    assert visual["native_window"]["display"] == ""


def test_visual_inventory_honors_missing_driver_override_before_pixis_icd(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(infra.sys, "platform", "linux")
    driver = tmp_path / "share/vulkan/icd.d/lvp_icd.x86_64.json"
    driver.parent.mkdir(parents=True)
    driver.write_text("{}", encoding="utf-8")
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path))
    monkeypatch.setenv("VK_ICD_FILENAMES", str(driver))
    missing = tmp_path / "missing.json"
    monkeypatch.setenv("VK_DRIVER_FILES", str(missing))

    vulkan = infra._visual_verification_inventory(tmp_path)["capture_vulkan"]

    assert vulkan == {
        "source": "VK_DRIVER_FILES",
        "icd_files": [str(missing)],
        "readiness": "selected ICD missing",
    }


def test_visual_inventory_discovers_pixis_lavapipe_icd(tmp_path, monkeypatch):
    monkeypatch.setattr(infra.sys, "platform", "linux")
    monkeypatch.delenv("VK_DRIVER_FILES", raising=False)
    monkeypatch.delenv("VK_ICD_FILENAMES", raising=False)
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path))
    driver = tmp_path / "share/vulkan/icd.d/lvp_icd.aarch64.json"
    driver.parent.mkdir(parents=True)
    driver.write_text("{}", encoding="utf-8")

    vulkan = infra._visual_verification_inventory(tmp_path)["capture_vulkan"]

    assert vulkan == {
        "source": "Pixi Lavapipe",
        "icd_files": [str(driver)],
        "readiness": "ICD available (device not probed)",
    }


def test_visual_inventory_keeps_unprobed_system_vulkan_readiness_unknown(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(infra.sys, "platform", "linux")
    for variable in ("VK_DRIVER_FILES", "VK_ICD_FILENAMES", "CONDA_PREFIX"):
        monkeypatch.delenv(variable, raising=False)

    vulkan = infra._visual_verification_inventory(tmp_path)["capture_vulkan"]

    assert vulkan["source"] == "unavailable"
    assert vulkan["icd_files"] == []
    assert vulkan["readiness"].startswith("unknown")


@pytest.mark.parametrize("platform", ("darwin", "win32"))
def test_visual_inventory_does_not_require_vulkan_outside_linux(
    tmp_path, monkeypatch, platform
):
    monkeypatch.setattr(infra.sys, "platform", platform)
    monkeypatch.setattr(infra.shutil, "which", lambda name: None)

    visual = infra._visual_verification_inventory(tmp_path)

    assert visual["capture_vulkan"]["readiness"] == "not required on this platform"


def test_doctor_print_distinguishes_capture_and_native_window_prerequisites(capsys):
    report = infra.doctor_report(ROOT)

    infra.print_doctor(report)

    output = capsys.readouterr().out
    assert "capture Vulkan=" in output
    assert "; native-window DISPLAY=" in output


@pytest.mark.parametrize(
    ("content", "expected"),
    [
        (None, "invalid JSON"),
        ("{", "invalid JSON"),
        ("null", "top level must be an object"),
        ("[]", "top level must be an object"),
        ('{"profile": "main"}', "`profile` must be 'dart7'"),
    ],
)
def test_doctor_does_not_claim_a_profile_when_invalid(
    monkeypatch, tmp_path, capsys, content, expected
):
    path = tmp_path / "docs" / "ai" / "branch-profile.json"
    if content is not None:
        path.parent.mkdir(parents=True)
        path.write_text(content)
    monkeypatch.setattr(infra, "run_checks", lambda root: [])

    report = infra.doctor_report(tmp_path)
    infra.print_doctor(report)

    assert report["profile"] is None
    assert report["ok"] is False
    assert any(expected in error for error in report["errors"])
    output = capsys.readouterr().out
    assert "DART AI infrastructure doctor: FAIL" in output
    assert "profile: unavailable" in output
    assert "profile: main" not in output


def test_malformed_hook_json_returns_errors_instead_of_tracebacks(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "hooks.json").write_text(
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {"matcher": "^Bash$", "hooks": [1]},
                    ]
                }
            }
        )
    )
    (tmp_path / ".claude").mkdir()
    (tmp_path / ".claude" / "settings.json").write_text(
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {"matcher": "Bash", "hooks": [{"command": 1}]},
                    ]
                }
            }
        )
    )
    errors = []

    infra.check_hooks(tmp_path, errors)

    assert ".codex/hooks.json: expected one bounded PreToolUse hook" in errors
    assert ".claude/settings.json: invalid or stale hook wiring" in errors


def test_hooks_require_cross_platform_commands(tmp_path):
    (tmp_path / ".codex").mkdir()
    (tmp_path / ".codex" / "hooks.json").write_text(
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {
                            "matcher": "^Bash$",
                            "hooks": [
                                {
                                    "type": "command",
                                    "command": infra.CODEX_HOOK_COMMAND,
                                    "timeout": 15,
                                    "statusMessage": "Checking DART commit command",
                                }
                            ],
                        }
                    ]
                }
            }
        )
    )
    (tmp_path / ".claude").mkdir()
    (tmp_path / ".claude" / "settings.json").write_text(
        json.dumps(
            {
                "hooks": {
                    "PreToolUse": [
                        {
                            "matcher": "Bash",
                            "hooks": [
                                {
                                    "type": "command",
                                    "command": "/usr/bin/python3 stale.py",
                                }
                            ],
                        }
                    ]
                }
            }
        )
    )
    for path in (
        tmp_path / "scripts" / "install_git_hooks.py",
        tmp_path / ".claude" / "hooks" / "pre-commit-guard.sh",
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("scripts/check_agent_hook.py --profile staged\n")
    errors = []

    infra.check_hooks(tmp_path, errors)

    assert ".codex/hooks.json: Windows hook command wiring is stale" in errors
    assert ".claude/settings.json: invalid or stale hook wiring" in errors


def test_release_hook_validation_rejects_unsafe_suffix_and_extra_keys(tmp_path):
    for relative in (
        ".codex/hooks.json",
        ".claude/settings.json",
        ".claude/hooks/pre-commit-guard.sh",
        ".claude/hooks/pre-commit-guard.ps1",
        "scripts/install_git_hooks.py",
        "scripts/pretool_guard_bridge.py",
    ):
        destination = tmp_path / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text((ROOT / relative).read_text())
    hooks_path = tmp_path / ".codex" / "hooks.json"
    data = json.loads(hooks_path.read_text())
    handler = data["hooks"]["PreToolUse"][0]["hooks"][0]
    handler["command"] += " && git push"
    handler["extra"] = "ignored"
    hooks_path.write_text(json.dumps(data))
    errors = []

    infra.check_hooks(tmp_path, errors)

    assert ".codex/hooks.json: hook command wiring is stale" in errors
    assert any("handler keys must equal" in error for error in errors)


def _write_visual_runtime_fixtures(root):
    for relative in (
        "python/tests/unit/gui/test_agent_debug_overlay.py",
        "python/tests/unit/gui/test_agent_capture.py",
    ):
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            (ROOT / relative).read_text(encoding="utf-8"), encoding="utf-8"
        )
    (root / "pixi.toml").write_text(
        "test-agent-debug-overlay =\n"
        "test_contacts_layer_marks_points_and_normals\n"
        "test_engine_rendered_overlay_changes_pixels\n"
        "test_run_capture_smoke_writes_stills_and_sidecar\n"
        "test_run_capture_debug_layers_change_pixels_end_to_end\n"
    )


def test_ci_wiring_requires_native_windows_hook_smoke(tmp_path):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    (workflows / "ci_ubuntu.yml").write_text(
        "pixi run check-lint\n"
        "pixi run test-ai-infra\n"
        "      - name: Agent visual verification smoke\n"
        "xvfb-run\n"
        "bash -eu -o pipefail <<'VISUAL_SMOKE'\n"
        "pixi run agent-capture\n"
        "--scene box_on_ground --steps 250 --focus box --auto-views 1\n"
        "--layers contacts collision_bounds labels\n"
        "--width 320 --height 240\n"
        "--out /tmp/dart-agent-visual-smoke --prefix smoke\n"
        "pixi run image-verdict\n"
        "/tmp/dart-agent-visual-smoke/smoke_auto0.png\n"
        "pixi run agent-capture\n"
        "--scene dart_shape_contacts --steps 500\n"
        "--camera-azimuth 1.5708 --camera-elevation 0.35\n"
        "--camera-distance 4.5 --camera-target 0 0 0.2\n"
        "--layers contacts collision_bounds labels\n"
        "--width 640 --height 480\n"
        "--out /tmp/dart-agent-visual-smoke --prefix dart\n"
        "pixi run image-verdict\n"
        "/tmp/dart-agent-visual-smoke/dart_main.png\n"
        "xvfb-run\n"
        "pixi run test-agent-debug-overlay\n"
    )
    (workflows / "ci_windows.yml").write_text(
        "Native Windows hook smoke\n"
        'pixi run python -c "import sys; print(sys.executable)"\n'
        "$launcher\n"
        "$hookCommand\n"
        "& (Join-Path (git rev-parse --show-toplevel)\n"
        '$ErrorActionPreference = "Continue"\n'
        '"git status"\n'
        "DART_HOOK_DRY_RUN\n"
        '"git commit --no-verify -m x"\n'
        "$diagnosticOutput\n"
        "$diagnosticExit -ne 0\n"
        "$successExit -ne 0\n"
        "-File $launcher\n"
        "$rawSuccessExit\n"
        "$rawBlockedExit\n"
        "cmd.exe /c exit 0\n"
    )
    _write_visual_runtime_fixtures(tmp_path)
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert errors == [
        ".github/workflows/ci_windows.yml: missing native hook smoke marker "
        "`$blockedExit -ne 2`"
    ]


def test_ci_wiring_requires_the_check_lint_aggregate(tmp_path):
    workflow = tmp_path / ".github" / "workflows" / "ci_ubuntu.yml"
    workflow.parent.mkdir(parents=True)
    workflow.write_text(
        (ROOT / ".github/workflows/ci_ubuntu.yml")
        .read_text(encoding="utf-8")
        .replace("pixi run check-lint", "pixi run check-lint-cpp"),
        encoding="utf-8",
    )
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any(
        ".github/workflows/ci_ubuntu.yml: missing AI check "
        "`pixi run check-lint`" in error
        for error in errors
    )


def test_ci_wiring_requires_check_lint_to_run_the_semantic_ai_check(tmp_path):
    workflow = tmp_path / ".github" / "workflows" / "ci_ubuntu.yml"
    workflow.parent.mkdir(parents=True)
    workflow.write_text(
        (ROOT / ".github/workflows/ci_ubuntu.yml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    (tmp_path / "pixi.toml").write_text(
        '[tasks]\ncheck-lint = { depends-on = ["check-ai-commands"] }\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert (
        "pixi.toml: `check-lint` must depend on `check-ai-infra` "
        "(CI Linux runs the AI checks through it)"
    ) in errors
    assert not any("must depend on `check-ai-commands`" in error for error in errors)


def test_ci_wiring_rejects_a_shorthand_check_lint_without_the_ai_checks(tmp_path):
    workflow = tmp_path / ".github" / "workflows" / "ci_ubuntu.yml"
    workflow.parent.mkdir(parents=True)
    workflow.write_text(
        (ROOT / ".github/workflows/ci_ubuntu.yml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    (tmp_path / "pixi.toml").write_text(
        '[tasks]\ncheck-lint = "codespell --config .codespellrc"\n',
        encoding="utf-8",
    )
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    for task in ("check-ai-commands", "check-ai-infra"):
        assert (
            f"pixi.toml: `check-lint` must depend on `{task}` "
            "(CI Linux runs the AI checks through it)"
        ) in errors


@pytest.mark.parametrize(
    "marker",
    (
        "Agent visual verification smoke",
        "bash -eu -o pipefail <<'VISUAL_SMOKE'",
        "pixi run agent-capture",
        "--scene box_on_ground --steps 250 --focus box --auto-views 1",
        "--scene dart_shape_contacts --steps 500",
        "--camera-azimuth 1.5708 --camera-elevation 0.35",
        "--camera-distance 4.5 --camera-target 0 0 0.2",
        "--layers contacts collision_bounds labels",
        "--width 320 --height 240",
        "--width 640 --height 480",
        "--out /tmp/dart-agent-visual-smoke --prefix smoke",
        "--out /tmp/dart-agent-visual-smoke --prefix dart",
        "pixi run image-verdict",
        "/tmp/dart-agent-visual-smoke/smoke_auto0.png",
        "/tmp/dart-agent-visual-smoke/dart_main.png",
        "pixi run test-agent-debug-overlay",
    ),
)
def test_ci_wiring_requires_visual_verification_smoke(tmp_path, marker):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    ubuntu = (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    if marker == "Agent visual verification smoke":
        mutated = ubuntu.replace(marker, "missing-visual-marker", 1)
    else:
        before, separator, visual_and_after = ubuntu.partition(
            "- name: Agent visual verification smoke"
        )
        assert separator
        mutated = (
            before
            + separator
            + visual_and_after.replace(marker, "missing-visual-marker", 1)
        )
    (workflows / "ci_ubuntu.yml").write_text(mutated)
    (workflows / "ci_windows.yml").write_text(
        (ROOT / ".github/workflows/ci_windows.yml").read_text()
    )
    _write_visual_runtime_fixtures(tmp_path)
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any("missing visual smoke" in error and marker in error for error in errors)


@pytest.mark.parametrize(
    ("line", "expected"),
    (
        ("if: false", "must run unconditionally"),
        ("if: inputs.nightly", "must run unconditionally"),
        ("continue-on-error: true", "must not use continue-on-error"),
    ),
)
def test_visual_smoke_cannot_be_skipped_or_soft_failed(tmp_path, line, expected):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    ubuntu = (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    step = "- name: Agent visual verification smoke"
    assert step in ubuntu
    ubuntu = ubuntu.replace(step, f"{step}\n        {line}", 1)
    (workflows / "ci_ubuntu.yml").write_text(ubuntu)
    (workflows / "ci_windows.yml").write_text(
        (ROOT / ".github/workflows/ci_windows.yml").read_text()
    )
    _write_visual_runtime_fixtures(tmp_path)
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any(expected in error for error in errors)


@pytest.mark.parametrize(
    "marker",
    (
        "viewer.setUpViewForCapture(240, 180)",
        "viewer.captureScreen(str(path))",
        "viewer.frame()",
        "assert np.abs(with_overlay - base).max() > 2",
        "assert np.abs(after_clear - base).max() <= 2",
    ),
)
def test_ci_wiring_requires_non_skippable_overlay_pixel_assertions(tmp_path, marker):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    (workflows / "ci_ubuntu.yml").write_text(
        (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    )
    (workflows / "ci_windows.yml").write_text(
        (ROOT / ".github/workflows/ci_windows.yml").read_text()
    )
    _write_visual_runtime_fixtures(tmp_path)
    path = tmp_path / "python/tests/unit/gui/test_agent_debug_overlay.py"
    path.write_text(
        (ROOT / "python/tests/unit/gui/test_agent_debug_overlay.py")
        .read_text()
        .replace(marker, "missing_overlay_marker")
    )
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any(
        "missing non-skippable overlay marker" in error and marker in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("relative", "marker", "expected"),
    (
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "test_run_capture_smoke_writes_stills_and_sidecar",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "test_run_capture_debug_layers_change_pixels_end_to_end",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'factory="claim_capture_scene:make_world"',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "sidecar = agent_capture.run_capture(args)",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'assert "--factory claim_capture_scene:make_world" in sidecar["reproduce"]',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'debug_layers = ["contacts", "collision_bounds", "labels"]',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'plain_artifact["camera"] == combined_artifact["camera"]',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "assert _changed_pixel_count(plain_image, combined_image) >= 128",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "agent_debug_overlay.CONTACT_POINT_RGB",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "assert contact_pixels >= 4",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "for layer in debug_layers",
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            'assert debug["layers"] == [layer]',
            "missing non-skippable factory-capture marker",
        ),
        (
            "python/tests/unit/gui/test_agent_capture.py",
            "assert _changed_pixel_count(plain_image, debug_image) >= 32",
            "missing non-skippable factory-capture marker",
        ),
        (
            "pixi.toml",
            "test-agent-debug-overlay =",
            "missing visual verification task marker",
        ),
        (
            "pixi.toml",
            "test_contacts_layer_marks_points_and_normals",
            "missing visual verification task marker",
        ),
        (
            "pixi.toml",
            "test_engine_rendered_overlay_changes_pixels",
            "missing visual verification task marker",
        ),
        (
            "pixi.toml",
            "test_run_capture_smoke_writes_stills_and_sidecar",
            "missing visual verification task marker",
        ),
        (
            "pixi.toml",
            "test_run_capture_debug_layers_change_pixels_end_to_end",
            "missing visual verification task marker",
        ),
    ),
)
def test_ci_wiring_requires_factory_capture_and_visual_task(
    tmp_path, relative, marker, expected
):
    workflows = tmp_path / ".github" / "workflows"
    workflows.mkdir(parents=True)
    (workflows / "ci_ubuntu.yml").write_text(
        (ROOT / ".github/workflows/ci_ubuntu.yml").read_text()
    )
    (workflows / "ci_windows.yml").write_text(
        (ROOT / ".github/workflows/ci_windows.yml").read_text()
    )
    _write_visual_runtime_fixtures(tmp_path)
    path = tmp_path / relative
    path.write_text(path.read_text().replace(marker, "missing_runtime_marker"))
    errors = []

    infra.check_ci_wiring(tmp_path, errors)

    assert any(expected in error and marker in error for error in errors)


@pytest.mark.parametrize(
    ("relative", "marker"),
    (
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$global:LASTEXITCODE = $null",
        ),
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$payload = [Console]::In.ReadToEnd()",
        ),
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$pipelineInput = @($input | ForEach-Object { $_ })",
        ),
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$pipelineInput.Count -gt 0",
        ),
        (
            ".claude/hooks/pre-commit-guard.ps1",
            "$OutputEncoding = New-Object System.Text.UTF8Encoding($false)",
        ),
        (
            "scripts/pretool_guard_bridge.py",
            'env["CLAUDE_PROJECT_DIR"] = str(root)',
        ),
        (
            "scripts/pretool_guard_bridge.py",
            "def may_invoke_git_commit(command: str) -> bool",
        ),
        (
            "scripts/pretool_guard_bridge.py",
            "if not may_invoke_git_commit(command):",
        ),
    ),
)
def test_windows_hook_components_cannot_be_missing_or_drifted(
    tmp_path, relative, marker
):
    for source in (
        ".codex/hooks.json",
        ".claude/settings.json",
        ".claude/hooks/pre-commit-guard.sh",
        ".claude/hooks/pre-commit-guard.ps1",
        "scripts/install_git_hooks.py",
        "scripts/pretool_guard_bridge.py",
    ):
        destination = tmp_path / source
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text((ROOT / source).read_text())
    path = tmp_path / relative
    original = path.read_text()
    path.unlink()
    missing_errors = []

    infra.check_hooks(tmp_path, missing_errors)

    assert any(relative in error for error in missing_errors)
    path.write_text(original.replace(marker, ""))
    drift_errors = []

    infra.check_hooks(tmp_path, drift_errors)

    assert any(marker in error for error in drift_errors)


def test_windows_launcher_does_not_shadow_native_exit_code():
    launcher = (ROOT / ".claude/hooks/pre-commit-guard.ps1").read_text()

    assert "$LASTEXITCODE = $null" not in launcher
    assert "$global:LASTEXITCODE = $null" in launcher
    assert "$nativeExitCode = $global:LASTEXITCODE" in launcher


def test_guard_hook_version_matches_installer(tmp_path):
    for source in (
        ".codex/hooks.json",
        ".claude/settings.json",
        ".claude/hooks/pre-commit-guard.sh",
        ".claude/hooks/pre-commit-guard.ps1",
        "scripts/install_git_hooks.py",
        "scripts/pretool_guard_bridge.py",
    ):
        destination = tmp_path / source
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text((ROOT / source).read_text())
    errors = []
    infra.check_hooks(tmp_path, errors)
    assert errors == []
    guard = tmp_path / ".claude/hooks/pre-commit-guard.sh"
    version = (
        (ROOT / "scripts/install_git_hooks.py")
        .read_text()
        .split('HOOK_VERSION = "', 1)[1]
        .split('"', 1)[0]
    )
    guard.write_text(
        guard.read_text().replace(
            f"DART-MANAGED-HOOK v{version} ", "DART-MANAGED-HOOK v0 "
        )
    )
    infra.check_hooks(tmp_path, errors)
    assert any("managed hook version differs" in error for error in errors)


@pytest.mark.parametrize("input_key", ("command", "cmd"))
def test_native_pretool_forwards_payload_to_shared_guard(
    tmp_path, monkeypatch, input_key
):
    guard = tmp_path / ".claude" / "hooks" / "pre-commit-guard.sh"
    guard.parent.mkdir(parents=True)
    guard.write_text("#!/bin/sh\nexit 0\n")
    bash = Path("C:/Program Files/Git/bin/bash.exe")
    calls = []
    monkeypatch.setattr(bridge, "find_git_bash", lambda: bash)
    monkeypatch.setattr(
        bridge.subprocess,
        "run",
        lambda args, **kwargs: calls.append((args, kwargs))
        or subprocess.CompletedProcess(args, 0, b"", b""),
    )
    payload = json.dumps({"tool_input": {input_key: "git commit -m x"}}).encode()

    assert bridge.forward(tmp_path, payload) == 0
    assert calls[0][0] == [
        str(bash),
        "--noprofile",
        "--norc",
        str(guard),
    ]
    assert calls[0][1]["input"] == payload
    assert calls[0][1]["cwd"] == tmp_path
    assert calls[0][1]["env"]["CLAUDE_PROJECT_DIR"] == str(tmp_path)
    assert calls[0][1]["env"]["CODEX_PROJECT_DIR"] == str(tmp_path)
    assert calls[0][1]["env"]["DART_HOOK_PYTHON"] == str(
        Path(sys.executable).resolve()
    ).replace("\\", "/")


@pytest.mark.parametrize("command", ("git status", "echo hello"))
def test_native_pretool_noncommit_bypasses_missing_git_bash(
    tmp_path, monkeypatch, command
):
    monkeypatch.setattr(bridge, "find_git_bash", lambda: None)
    payload = json.dumps({"tool_input": {"command": command}}).encode()

    assert bridge.forward(tmp_path, payload) == 0


@pytest.mark.parametrize(
    "command",
    (
        "GIT COMMIT -m x",
        "g---i---t\nc---o---m---m---i---t",
    ),
)
def test_native_pretool_commit_without_git_bash_fails_closed(
    tmp_path, monkeypatch, capsys, command
):
    monkeypatch.setattr(bridge, "find_git_bash", lambda: None)
    payload = json.dumps({"tool_input": {"command": command}}).encode()

    assert bridge.forward(tmp_path, payload) == 2
    assert "Git Bash is required" in capsys.readouterr().err


def test_native_pretool_maps_guard_failure_to_codex_block(
    tmp_path, monkeypatch, capsys
):
    guard = tmp_path / ".claude" / "hooks" / "pre-commit-guard.sh"
    guard.parent.mkdir(parents=True)
    guard.write_text("#!/bin/sh\nexit 2\n")
    monkeypatch.setattr(bridge, "find_git_bash", lambda: Path("/test/git-bash"))
    monkeypatch.setattr(
        bridge.subprocess,
        "run",
        lambda args, **kwargs: subprocess.CompletedProcess(
            args, 2, b"guard output\n", b"guard error\n"
        ),
    )
    payload = json.dumps({"tool_input": {"command": "git commit"}}).encode()

    assert bridge.forward(tmp_path, payload) == 2
    captured = capsys.readouterr()
    assert "guard output" in captured.err
    assert "guard error" in captured.err


def test_native_pretool_rejects_malformed_payload(tmp_path):
    assert bridge.forward(tmp_path, b"{}") == 2


@pytest.mark.parametrize(
    ("updated", "expected"),
    (
        (b"updated\r\n", 0),
        (b"updated \r\n", 2),
    ),
)
def test_staged_diff_check_accepts_crlf_but_rejects_trailing_space(
    tmp_path, updated, expected
):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    path = tmp_path / "legacy.txt"
    path.write_bytes(b"base\r\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "legacy.txt"], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    path.write_bytes(updated)
    subprocess.run(["git", "-C", str(tmp_path), "add", "legacy.txt"], check=True)

    assert hook.run_staged(tmp_path) == expected


def test_mixed_staged_and_unstaged_ai_file_is_rejected(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    path = tmp_path / "docs" / "ai" / "example.md"
    path.parent.mkdir(parents=True)
    path.write_text("base\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )

    path.write_text("staged\n")
    subprocess.run(
        ["git", "-C", str(tmp_path), "add", "docs/ai/example.md"], check=True
    )
    path.write_text("unstaged final\n")

    assert hook.run_staged(tmp_path) == 2


def test_staged_source_cannot_be_hidden_by_unstaged_generated_output(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    source = tmp_path / ".claude" / "skills" / "dart-example" / "SKILL.md"
    generated = tmp_path / ".agents" / "skills" / "dart-example" / "SKILL.md"
    source.parent.mkdir(parents=True)
    generated.parent.mkdir(parents=True)
    source.write_text("base source\n")
    generated.write_text("base generated\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )

    source.write_text("staged source\n")
    subprocess.run(
        [
            "git",
            "-C",
            str(tmp_path),
            "add",
            ".claude/skills/dart-example/SKILL.md",
        ],
        check=True,
    )
    generated.write_text("unstaged generated fix\n")

    assert hook.run_staged(tmp_path) == 2


def test_staged_ai_deletion_runs_infrastructure_checks(tmp_path, monkeypatch):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    path = tmp_path / "docs" / "ai" / "example.md"
    path.parent.mkdir(parents=True)
    path.write_text("base\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    path.unlink()
    subprocess.run(["git", "-C", str(tmp_path), "add", "-u"], check=True)
    calls = []

    def record(command, root):
        calls.append((command, root))
        return 0

    monkeypatch.setattr(hook, "run_checked", record)

    assert hook.run_staged(tmp_path) == 0
    assert len(calls) == 2


def test_staged_ai_deletion_with_untracked_replacement_is_rejected(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    path = tmp_path / "docs" / "ai" / "example.md"
    path.parent.mkdir(parents=True)
    path.write_text("base\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "rm", "-q", "docs/ai/example.md"], check=True
    )
    path.parent.mkdir(parents=True)
    path.write_text("untracked replacement\n")

    assert hook.run_staged(tmp_path) == 2


def test_staged_rename_out_of_ai_scope_still_runs_checks(tmp_path, monkeypatch):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    source = tmp_path / "docs" / "ai" / "example.md"
    source.parent.mkdir(parents=True)
    source.write_text("base\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    (tmp_path / "misc").mkdir()
    subprocess.run(
        [
            "git",
            "-C",
            str(tmp_path),
            "mv",
            "docs/ai/example.md",
            "misc/example.md",
        ],
        check=True,
    )
    calls = []

    def record(command, root):
        calls.append((command, root))
        return 0

    monkeypatch.setattr(hook, "run_checked", record)

    assert hook.run_staged(tmp_path) == 0
    assert len(calls) == 2


def test_unstaged_rename_out_of_ai_scope_blocks_other_staged_ai_work(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.email", "test@example.com"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(tmp_path), "config", "user.name", "DART Test"],
        check=True,
    )
    first = tmp_path / "docs" / "ai" / "first.md"
    second = tmp_path / "docs" / "ai" / "second.md"
    first.parent.mkdir(parents=True)
    first.write_text("first\n")
    second.write_text("second\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "commit", "-q", "-m", "base"], check=True
    )
    second.write_text("staged second\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "docs/ai/second.md"], check=True)
    (tmp_path / "misc").mkdir()
    first.rename(tmp_path / "misc" / "first.md")

    assert hook.run_staged(tmp_path) == 2


@pytest.mark.parametrize(
    "source",
    [
        "# assert with_overlay != base\n",
        '"assert with_overlay != base"\n',
        'message = "assert with_overlay != base"\n',
        "if False:\n    assert with_overlay != base\n",
    ],
)
def test_visual_python_marker_rejects_comment_and_string_spoofs(source):
    assert not infra.visual_python_marker(
        ast.parse(source), "assert with_overlay != base"
    )


def test_visual_python_marker_matches_assertions_independent_of_quote_style():
    assert infra.visual_python_marker(
        ast.parse("assert sidecar['renderer'] == 'filament'\n"),
        'assert sidecar["renderer"] == "filament"',
    )


@pytest.mark.parametrize("filename", ["ci_ubuntu.yml", "ci_windows.yml"])
def test_lint_workflows_refresh_pr_release_payload(filename):
    import yaml

    workflow = yaml.load(
        (ROOT / ".github/workflows" / filename).read_text(encoding="utf-8"),
        Loader=yaml.BaseLoader,
    )
    assert set(workflow["on"]["pull_request"]["types"]) == {
        "opened",
        "synchronize",
        "reopened",
        "edited",
        "milestoned",
        "demilestoned",
    }
    assert workflow["on"]["pull_request"]["branches"] == ["**"]
    build = workflow["jobs"]["build"]
    assert "if" not in build
    lint = next(step for step in build["steps"] if step.get("name") == "Check Lint")
    assert "pixi run check-lint" in lint["run"]
    assert lint.get("if", "") in {"", "matrix.part == 'cpp'"}


@pytest.fixture
def scenario_data():
    return _scenario_data()


@pytest.mark.parametrize(
    "scenario_id",
    [item["id"] for item in _scenario_data()["scenarios"]],
)
def test_every_scenario_rejects_pinned_prompt(scenario_data, scenario_id):
    scenario = next(
        item for item in scenario_data["scenarios"] if item["id"] == scenario_id
    )
    scenario["prompt_class"] = "DART 6.22"

    errors = infra.exercise_scenarios(ROOT, scenario_data, emit=False)

    assert (
        f"scenario `{scenario_id}`: prompt_class must be release-neutral; use DART"
        in errors
    )


@pytest.mark.parametrize(
    "version",
    [
        "6.22.0",
        "**DART 6.22.0**",
        "release-6.22",
        "v6.22.0",
        "DART6.22",
        "DART\n6.22",
        "7.0.0",
        "DART 7.0",
        "release-7.0",
        "v7.0.0",
        "DART7.0",
        "DART\n7.0",
        "DART 12.0",
        "6.22",
        "7.0",
        "DART v12.0.0",
        "6.22.0rc1",
        "7.0.0rc1",
    ],
)
def test_scenario_rejects_concrete_version_tokens(scenario_data, version):
    scenario_data["scenarios"][0]["prompt_class"] = f"Inspect {version} checkout"

    errors = infra.exercise_scenarios(ROOT, scenario_data, emit=False)

    assert (
        "scenario `orientation`: prompt_class must be release-neutral; use DART"
        in errors
    )


@pytest.mark.parametrize(
    "prompt",
    [
        "Inspect DART version: 7.0.0",
        "Inspect dartpy==7.0.0 compatibility",
        "Pin dartpy>=8.1 in a DART checkout",
        "Inspect DART_VERSION=6.22.0 compatibility",
        "Inspect DART_VERSION=7.0.0 compatibility",
        "Pin dartpy===7.0.0 for the DART checkout",
        "Inspect the DART checkout (version: 7.0.0)",
        "Inspect the DART checkout (version: 6.22.0)",
        "For DART, update nanobind before the upcoming release 7.0.0.",
        "When updating Python, main's next release is 7.0.0.",
        "Inspect the DART checkout: 7.0.0",
        "Inspect the DART library 7.0.0 checkout",
        "When updating libdart, the next version is 7.0.0",
        "Inspect libdart 6.20.0 compatibility in a DART checkout",
        "Inspect -DDARTPY_VERSION_INFO=6.20.0 in a DART checkout",
        "Pin libdart-dev=6.20.0 for the DART checkout",
        "Inspect -DDARTPY_VERSION_INFO:STRING=6.20.0 in the current DART checkout",
        "When updating Python, prepare DART for the next release: 7.0.0",
        "Pin dartpy_version=7.0.0 in a DART checkout",
        "Inspect DaRt_ReLeAsE=v12.0.0 compatibility",
        "Pin dartpy_VERSION=8.1 in a DART checkout",
        "Inspect gz-physics DART 7.0.0 compatibility",
        "Inspect gz-physics release-7.0 compatibility",
        "Inspect nanobinder 7.0.0 compatibility",
        "Prepare DART for the next release: 7.0.0",
        "Prepare the Python-first DART for the next release: 7.0.0",
        "For DART's Python API, the next release is 7.0.0.",
        "Inspect DART pre-release 7.0.0 compatibility",
        "Inspect DART pre-release 6.22.0 compatibility",
        "Inspect dartpy.__version__: 7.0.0 compatibility",
        "Inspect dart.__version__: 6.22.0 compatibility",
        "Inspect Python-first 7.0.0 compatibility",
        "Inspect branch-local 7.0.0 compatibility",
        "Inspect pre-Python 7.0.0 compatibility",
        "Inspect foo-bar-baz 7.8.0 compatibility in a DART checkout",
        "When updating gz-physics, the next version is DART 7.0.0.",
        "When updating gz-physics, the next DART version is 7.0.0.",
        "When updating gz-physics, the next version is 7.8.0; inspect 7.0.0 checkout.",
        "Update gz-physics. Next version: 7.0.0",
        "Use the next version: 6.22",
    ],
)
def test_scenario_rejects_qualified_version_prompts(scenario_data, prompt):
    scenario_data["scenarios"][0]["prompt_class"] = prompt

    assert infra.exercise_scenarios(ROOT, scenario_data, emit=False) == [
        "scenario `orientation`: prompt_class must be release-neutral; use DART"
    ]


@pytest.mark.parametrize(
    "prompt",
    [
        "Orient to the current DART checkout",
        "Inspect DART compatibility",
        "Inspect DART 6 compatibility",
        "Inspect Python 3.12 support in the current DART checkout",
        "Tune time_step=0.001 in a DART checkout",
        "Update a dependency to v12.0.0 in a DART checkout",
        "Tune friction=7.0 in a DART checkout",
        "Tune friction= 7.0 in a DART checkout",
        "Tune friction:7.0 in a DART checkout",
        "Tune friction: 7.0 in a DART checkout",
        "Update nanobind to 3.1.0 in a DART checkout",
        "Update nanobind to v3.1.0 in a DART checkout",
        "Update gz-physics to 9.0.0 in a DART checkout",
        "Inspect gz-sim 8.0.0 compatibility in the current DART checkout",
        "Inspect gz-physics 7.8.0 compatibility in a DART checkout",
        "Update gz-physics to 7.8.0 in a DART checkout",
        "Update gz-physics to v7.8.0 in a DART checkout",
        "Check gz-physics version: 7.8.0 in a DART checkout",
        "Inspect Codex CLI version: 0.162.0 in the current DART checkout",
        "Inspect gz-physics version 7.4.0 compatibility in the current DART checkout",
        "Update gz-physics to version 7.8.0 in a DART checkout",
        "Update gz-physics from 7.7.0 to 7.8.0 in a DART checkout",
        "Upgrade gz-physics from version 7.7.0 to version 7.8.0 in a DART checkout",
        "Update gz-physics to release 7.8.0 in a DART checkout",
        "Check compatibility with gz-math 7.4.0 and 7.5.0",
        "Update gz-physics to version: 7.8.0 in a DART checkout",
        "Check gz-math versions 7.4.0 and 7.5.0 compatibility in a DART checkout",
        "Check that the gz-physics version is 7.8.0 before building DART",
        "Check gz-math versions 7.3.0, 7.4.0, and 7.5.0 compatibility in the current DART checkout",
        "When upgrading gz-physics from version 7.7.0 to version 7.8.0, the next version is 7.9.0.",
        "When updating gz-physics from version 7.7.0, the next version is 7.8.0.",
        "When updating libcurl from 7.85.0, the next version is 7.86.0.",
        "Inspect libcurl 7.85.0 compatibility in a DART checkout",
        "Inspect CURL 7.85.0 compatibility in a DART checkout",
        "When upgrading Python, next version: 3.15.0",
        "When updating gz-physics, next version: 7.8.0",
        "Inspect gz-physics v7.8.0 compatibility in a DART checkout",
        "Inspect gz-foo-bar-baz 7.8.0 compatibility in a DART checkout",
        "Inspect GZ-MATH v7.8.0 compatibility in a DART checkout",
        "Inspect EIGEN v7.8.0 compatibility in a DART checkout",
        "Update gz-sim to 8.0.0 in a DART checkout",
        "Update Python to 3.15.0 in a DART checkout",
        "When updating gz-physics, the next version is 7.8.0.",
        "When updating gz-physics 7.7.0, the next version is v7.8.0.",
        "Check gravity 9.81 in a DART checkout",
        "Inspect 12.0.0 checkout",
        "Inspect 16.0 checkout",
        "Inspect v16.0 checkout",
    ],
)
def test_scenario_accepts_release_neutral_prompt(scenario_data, prompt):
    scenario_data["scenarios"][0]["prompt_class"] = prompt

    assert infra.exercise_scenarios(ROOT, scenario_data, emit=False) == []


@pytest.mark.parametrize(
    ("version", "pinned"),
    [
        ("7.1", True),
        ("9.0.0", False),
        ("v9.0.0", False),
        ("6.22", False),
        ("v6.22", False),
    ],
)
def test_release_detectors_follow_target_major(
    release_reference_target, tmp_path, scenario_data, monkeypatch, version, pinned
):
    text = release_reference_target.read_text(encoding="utf-8")
    release_reference_target.write_text(
        text.replace("7.0.0", "8.0.0"), encoding="utf-8"
    )
    original_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if path == ROOT / "docs/onboarding/release-management.md":
            path = release_reference_target
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)
    (tmp_path / "AGENTS.md").write_text(f"Next version: {version}\n", encoding="utf-8")
    errors = []

    infra.check_release_references(tmp_path, errors)

    assert len(errors) == int(pinned)
    scenario_data["scenarios"][0]["prompt_class"] = f"Inspect {version} checkout"
    assert infra.exercise_scenarios(ROOT, scenario_data, emit=False) == (
        ["scenario `orientation`: prompt_class must be release-neutral; use DART"]
        if pinned
        else []
    )


@pytest.mark.parametrize("row", ["", "| main | Development | invalid |\n"])
@pytest.mark.parametrize(
    ("version", "pinned"),
    [
        ("6.22", False),
        ("v7.0.0rc1", False),
        ("DART 12.0", True),
        ("DART7.0", True),
        ("release-7.0", True),
        ("DART version: 8.0.0", True),
        ("dartpy release = v8.0.0", True),
        ("DART_VERSION=6.22.0", True),
        ("dartpy_version=7.0.0", True),
        ("DART_RELEASE=v12.0.0", True),
        ("dartpy==8.0.0", True),
        ("dartpy>=8.1", True),
        ("DART<=8.1", True),
        ("dartpy~=8.1", True),
        ("DART!=8.1", True),
        ("dartpy<8.1", True),
        ("DART>8.1", True),
        ("dart.__version__: v12.0.0", True),
        ("dartpy.__version__>=8.1", True),
        ("DaRtPy.__version__ v16.0", True),
    ],
)
def test_release_detectors_require_target_for_bare_versions(
    release_reference_target, tmp_path, scenario_data, monkeypatch, row, version, pinned
):
    text = release_reference_target.read_text(encoding="utf-8")
    release_reference_target.write_text(
        text.replace("| `main` | Development | `7.0.0` |\n", row), encoding="utf-8"
    )
    original_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if path == ROOT / "docs/onboarding/release-management.md":
            path = release_reference_target
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)
    (tmp_path / "AGENTS.md").write_text(f"Next version: {version}\n", encoding="utf-8")
    errors = []

    infra.check_release_references(tmp_path, errors)

    assert len(errors) == int(pinned)
    scenario_data["scenarios"][0]["prompt_class"] = f"Inspect {version} checkout"
    assert infra.exercise_scenarios(ROOT, scenario_data, emit=False) == (
        ["scenario `orientation`: prompt_class must be release-neutral; use DART"]
        if pinned
        else []
    )


def test_full_infrastructure_check_rejects_pinned_scenario_prompt(
    scenario_data, monkeypatch
):
    scenario_data["scenarios"][0]["prompt_class"] = "DART 6.22"
    original_read_json = infra.read_json

    def read_json(path):
        if path == ROOT / "docs/ai/agent-scenarios.json":
            return scenario_data
        return original_read_json(path)

    monkeypatch.setattr(infra, "read_json", read_json)

    errors = infra.run_checks(ROOT)

    assert (
        "scenario `orientation`: prompt_class must be release-neutral; use DART"
        in errors
    )


@pytest.mark.parametrize(("version", "expected"), [("7.0.0", []), ("8.0.0", 1)])
def test_main_release_target_matches_branch_profile(tmp_path, version, expected):
    owner = tmp_path / "docs/onboarding/release-management.md"
    owner.parent.mkdir(parents=True)
    owner.write_text(
        (ROOT / "docs/onboarding/release-management.md")
        .read_text(encoding="utf-8")
        .replace("`7.0.0`", f"`{version}`", 1),
        encoding="utf-8",
    )
    profile = tmp_path / "docs/ai/branch-profile.json"
    profile.parent.mkdir(parents=True)
    profile.write_text(json.dumps({"profile": "dart7"}), encoding="utf-8")
    errors = []

    infra.check_release_target_profile(tmp_path, errors)

    if expected == []:
        assert errors == []
    else:
        assert len(errors) == 1 and "branch profile `dart7`" in errors[0]
