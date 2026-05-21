import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_pages_config.py"


def _pages_config(tmp_path: Path, **overrides) -> Path:
    config = {
        "build_type": "legacy",
        "html_url": "https://dartsim.github.io/dart/",
        "source": {"branch": "gh-pages", "path": "/"},
        "status": "built",
    }
    config.update(overrides)
    path = tmp_path / "pages.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    return path


def test_pages_config_accepts_expected_branch_source(tmp_path):
    config = _pages_config(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--pages-api-url",
            str(config),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "pages configuration verified" in result.stdout
    assert "gh-pages/" in result.stdout


def test_pages_config_rejects_unexpected_branch(tmp_path):
    config = _pages_config(tmp_path, source={"branch": "main", "path": "/"})

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--pages-api-url",
            str(config),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "source.branch" in result.stderr


def test_pages_config_rejects_unusable_status(tmp_path):
    config = _pages_config(tmp_path, status="errored")

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--pages-api-url",
            str(config),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "usable Pages status" in result.stderr
