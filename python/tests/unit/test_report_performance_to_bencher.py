import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "report_performance_to_bencher.py"


def test_report_performance_to_bencher_dry_run_redacts_key(tmp_path):
    first = tmp_path / "collision_check_a.json"
    second = tmp_path / "collision_check_b.json"
    first.write_text("{}", encoding="utf-8")
    second.write_text("{}", encoding="utf-8")

    env = os.environ.copy()
    env.update(
        {
            "BENCHER_API_KEY": "secret-key",
            "BENCHER_PROJECT": "project-dart",
            "BENCHER_HOST": "https://bencher.example.invalid",
        }
    )
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(tmp_path / "collision_check_*.json"),
            "--branch",
            "main",
            "--sha",
            "abc123",
            "--testbed",
            "github-Linux-X64",
            "--dry-run",
        ],
        check=True,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "secret-key" not in result.stdout
    assert result.stdout.count("bencher run") == 2
    assert "--adapter cpp_google" in result.stdout
    assert "--host https://bencher.example.invalid" in result.stdout
    assert str(first) in result.stdout
    assert str(second) in result.stdout


def test_report_performance_to_bencher_can_skip_when_unconfigured(tmp_path):
    env = os.environ.copy()
    env.pop("BENCHER_API_KEY", None)
    env.pop("BENCHER_PROJECT", None)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(tmp_path / "collision_check_*.json"),
            "--skip-if-unconfigured",
            "--dry-run",
        ],
        check=True,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "Skipping Bencher reporting" in result.stdout


def test_report_performance_to_bencher_skips_empty_inputs(tmp_path):
    valid = tmp_path / "valid.json"
    empty = tmp_path / "empty.json"
    valid.write_text("{}", encoding="utf-8")
    empty.write_text("", encoding="utf-8")

    env = os.environ.copy()
    env.update(
        {
            "BENCHER_API_KEY": "secret-key",
            "BENCHER_PROJECT": "project-dart",
        }
    )
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(tmp_path / "*.json"),
            "--dry-run",
        ],
        check=True,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert str(valid) in result.stdout
    assert str(empty) not in result.stdout
    assert f"Skipping empty benchmark input: {empty}" in result.stderr


def test_report_performance_to_bencher_can_skip_when_only_empty_inputs(tmp_path):
    empty = tmp_path / "empty.json"
    empty.write_text("", encoding="utf-8")

    env = os.environ.copy()
    env.update(
        {
            "BENCHER_API_KEY": "secret-key",
            "BENCHER_PROJECT": "project-dart",
        }
    )
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(tmp_path / "*.json"),
            "--skip-if-no-input",
            "--dry-run",
        ],
        check=True,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert "Skipping Bencher reporting" in result.stdout
    assert f"Skipping empty benchmark input: {empty}" in result.stderr
