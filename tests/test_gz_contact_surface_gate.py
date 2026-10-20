"""The gz-physics gate fails unless the ContactSurface.hpp probe was cached as 1."""

import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/check_gz_contact_surface.sh"
ENTRY = "DART_HAS_CONTACT_SURFACE_HEADER:INTERNAL"


def _check(build, cache):
    build.mkdir(exist_ok=True)
    if cache is not None:
        (build / "CMakeCache.txt").write_text(cache)
    return subprocess.run(
        ["bash", str(SCRIPT), str(build)], capture_output=True, text=True
    )


def test_cached_one_passes(tmp_path):
    assert _check(tmp_path, f"A:BOOL=ON\n{ENTRY}=1\n").returncode == 0


@pytest.mark.parametrize(
    "cache",
    [f"{ENTRY}=\n", f"{ENTRY}=0\n", f"{ENTRY}=10\n", "A:BOOL=ON\n", None],
)
def test_missing_or_failed_probe_fails(tmp_path, cache):
    run = _check(tmp_path, cache)
    assert run.returncode == 1
    assert "ContactSurface.hpp" in run.stderr


def test_failure_shows_probe_log(tmp_path):
    log = tmp_path / "CMakeFiles/CMakeConfigureLog.yaml"
    log.parent.mkdir()
    log.write_text(
        "  stdout: |\n    DART_HAS_CONTACT_SURFACE_HEADER\n    error: consteval\n"
    )
    run = _check(tmp_path, f"{ENTRY}=0\n")
    assert run.returncode == 1
    assert "consteval" in run.stderr
