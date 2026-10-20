"""Fetched Filament licenses must stay intact for the native wheel.

tests/test_dartpy_backend.py covers their placement in the wheel.
"""

import hashlib
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize("crlf", [False, True])
def test_filament_root_license_validation_preserves_upstream_bytes(tmp_path, crlf):
    root = Path(__file__).resolve().parents[1]
    canonical = (root / "tests/cmake/filament_notices/LICENSE").read_bytes()
    canonical = canonical.replace(b"\r\n", b"\n")
    expected = "02d70b593dd8e9355dd8e1ca8d13c303bee5bcaf3065b66ff6e4f534e1e6a2c1"
    assert hashlib.sha256(canonical).hexdigest() == expected
    contents = canonical.replace(b"\n", b"\r\n") if crlf else canonical
    license_file = tmp_path / "LICENSE"
    license_file.write_bytes(contents)
    script = tmp_path / "validate.cmake"
    script.write_text(
        f'include("{root.as_posix()}/cmake/DARTFilamentNotices.cmake")\n'
        f'dart_filament_validate_root_license("{license_file.as_posix()}" "{expected}")\n',
        encoding="utf-8",
    )
    subprocess.run(["cmake", "-P", str(script)], check=True, capture_output=True)
    assert license_file.read_bytes() == contents


@pytest.mark.parametrize("supplied_tree", [False, True])
def test_filament_notice_staging_rejects_unrelated_root_license(
    tmp_path, supplied_tree
):
    root = Path(__file__).resolve().parents[1]
    staging = tmp_path / "staging"
    staging.mkdir()
    license_file = staging / "LICENSE"
    license_file.write_bytes(b"Unrelated SDK or stale cached license\n")
    supplied = tmp_path / "notices"
    supplied.mkdir()
    source_argument = f' "{supplied.as_posix()}"' if supplied_tree else ""
    script = tmp_path / "stage.cmake"
    script.write_text(
        f'include("{root.as_posix()}/cmake/DARTFilamentNotices.cmake")\n'
        f'dart_filament_stage_notices("{staging.as_posix()}"{source_argument})\n',
        encoding="utf-8",
    )
    result = subprocess.run(
        ["cmake", "-P", str(script)], capture_output=True, text=True
    )
    assert result.returncode != 0
    assert "pinned Filament root license changed" in result.stderr
    assert license_file.read_bytes() == b"Unrelated SDK or stale cached license\n"
