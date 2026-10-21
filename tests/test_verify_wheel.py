"""verify_wheel.py accepts the packaged dartpy layout and rejects stray files."""

import importlib.util
import zipfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DIST_INFO = "dartpy-1.0.dist-info"
PACKAGE = {
    "dartpy/__init__.py": b"# package\n",
    "dartpy/_dartpy.abi3.so": b"native extension fixture",
    "dartpy/py.typed": b"",
    "dartpy/experimental/__init__.py": b'"""Outside SemVer."""\n',
    f"{DIST_INFO}/METADATA": b"Metadata-Version: 2.4\nName: dartpy\nVersion: 1.0\n",
    f"{DIST_INFO}/licenses/LICENSE": b"DART BSD license\n",
    f"{DIST_INFO}/licenses/filament/LICENSE": b"Filament Apache license\n",
    f"{DIST_INFO}/RECORD": b"",
}


def _load():
    spec = importlib.util.spec_from_file_location(
        "verify_wheel", ROOT / "scripts/verify_wheel.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _wheel(tmp_path, entries, name="dartpy-1.0-cp312-abi3-manylinux_2_28_x86_64.whl"):
    wheel = tmp_path / name
    with zipfile.ZipFile(wheel, "w") as archive:
        for entry, data in entries.items():
            archive.writestr(entry, data)
    return wheel


def test_accepts_package_with_vendored_libraries(tmp_path, capsys):
    wheel = _wheel(
        tmp_path,
        {
            **PACKAGE,
            "dartpy.libs/libfcl-54c78b3b.so.0.7.0": b"vendored",
            "dartpy/.dylibs/x": b"",
        },
    )
    assert _load().verify_wheel(wheel)
    assert "Found extension: dartpy/_dartpy.abi3.so" in capsys.readouterr().out


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"tests/__init__.py": b""}, "files outside the dartpy package: tests/"),
        ({"dartpy/_filament_licenses/LICENSE": b""}, "dartpy/_filament_licenses/"),
        ({"dartpy/py.typed": None}, "missing dartpy/py.typed"),
        ({"dartpy/experimental/__init__.py": None}, "missing dartpy/experimental/"),
        ({f"{DIST_INFO}/licenses/filament/LICENSE": None}, "licenses/filament/LICENSE"),
        ({"dartpy/_dartpy.abi3.so": None}, "No dartpy._dartpy extension"),
    ],
)
def test_rejects_layout_violations(tmp_path, capsys, change, message):
    entries = {**PACKAGE, **{k: v for k, v in change.items() if v is not None}}
    for key, value in change.items():
        if value is None:
            entries.pop(key)
    assert not _load().verify_wheel(_wheel(tmp_path, entries))
    assert message in capsys.readouterr().out


def test_rejects_version_specific_wheels(tmp_path, capsys):
    wheel = _wheel(tmp_path, PACKAGE, "dartpy-1.0-cp312-cp312-linux_x86_64.whl")
    assert not _load().verify_wheel(wheel)
    assert "Expected a cp312-abi3 wheel" in capsys.readouterr().out
