"""The PEP 517 wrapper moves the staged Filament license into the wheel's dist-info.

It exports only wheel builds: no sdist, no editable install, and no
``prepare_metadata_for_build_wheel``, so frontends read the wheel's METADATA.
"""

import base64
import csv
import hashlib
import importlib.util
import io
import zipfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
METADATA = (
    b"Metadata-Version: 2.4\nName: dartpy\nVersion: 1.0\n"
    b"License-File: LICENSE\n\nDescription body\n"
)


def _load_backend():
    spec = importlib.util.spec_from_file_location(
        "dartpy_backend", ROOT / "scripts/pep517/dartpy_backend.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _wheel(tmp_path, entries):
    wheel = tmp_path / "dartpy-1.0-cp312-abi3-linux_x86_64.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        for name, data in entries.items():
            archive.writestr(name, data)
    return wheel


def test_backend_exports_only_wheel_hooks():
    module = _load_backend()
    import scikit_build_core.build as skbuild

    assert module.build_wheel is not skbuild.build_wheel
    assert module.get_requires_for_build_wheel is skbuild.get_requires_for_build_wheel
    # Absent on purpose: a metadata-only build cannot list the Filament
    # licenses, so frontends must read METADATA from the built wheel.
    for hook in (
        "prepare_metadata_for_build_wheel",
        "prepare_metadata_for_build_editable",
        "get_requires_for_build_editable",
        "get_requires_for_build_sdist",
    ):
        assert not hasattr(module, hook), hook


def test_sdist_is_rejected(tmp_path):
    module = _load_backend()
    with pytest.raises(module.UnsupportedOperation, match="wheel-only"):
        module.build_sdist(str(tmp_path))
    assert not list(tmp_path.iterdir())


def test_editable_install_is_rejected(tmp_path):
    module = _load_backend()
    with pytest.raises(module.UnsupportedOperation, match="build-py-dev"):
        module.build_editable(str(tmp_path))
    assert not list(tmp_path.iterdir())


def test_build_wheel_relocates_the_staged_licenses(tmp_path, monkeypatch):
    module = _load_backend()

    def fake_build_wheel(
        wheel_directory, config_settings=None, metadata_directory=None
    ):
        wheel = _wheel(
            Path(wheel_directory),
            {
                "dartpy/__init__.py": b"# package\n",
                "dartpy/_filament_licenses/LICENSE": b"Filament\n",
                "dartpy-1.0.dist-info/METADATA": METADATA,
                "dartpy-1.0.dist-info/RECORD": b"stale\n",
            },
        )
        return wheel.name

    monkeypatch.setattr(module, "_build_wheel", fake_build_wheel)

    name = module.build_wheel(str(tmp_path), {"setting": "ignored"}, None)

    with zipfile.ZipFile(tmp_path / name) as archive:
        assert "dartpy-1.0.dist-info/licenses/filament/LICENSE" in archive.namelist()
        assert b"License-File: filament/LICENSE" in archive.read(
            "dartpy-1.0.dist-info/METADATA"
        )


@pytest.mark.parametrize("crlf", [False, True])
def test_staged_licenses_move_into_dist_info_with_headers_and_record(tmp_path, crlf):
    module = _load_backend()
    newline = b"\r\n" if crlf else b"\n"
    metadata = METADATA.replace(b"\n", newline)
    license_text = b"Filament Apache license" + newline
    notice = b"Upstream bundled Zstandard notice\r\n"
    wheel = _wheel(
        tmp_path,
        {
            "dartpy/__init__.py": b"# package\n",
            "dartpy/_dartpy.abi3.so": b"native extension fixture",
            "dartpy/_filament_licenses/LICENSE": license_text,
            "dartpy/_filament_licenses/notices/third_party/zstd/LICENSE": notice,
            "dartpy-1.0.dist-info/licenses/LICENSE": b"DART BSD license\n",
            "dartpy-1.0.dist-info/METADATA": metadata,
            "dartpy-1.0.dist-info/WHEEL": b"Wheel-Version: 1.0\n",
            "dartpy-1.0.dist-info/RECORD": b"stale\n",
        },
    )

    moved = module.relocate_filament_licenses(wheel)

    assert moved == [
        "dartpy-1.0.dist-info/licenses/filament/LICENSE",
        "dartpy-1.0.dist-info/licenses/filament/notices/third_party/zstd/LICENSE",
    ]
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        assert not any(name.startswith("dartpy/_filament_licenses/") for name in names)
        assert archive.read(moved[0]) == license_text
        assert archive.read(moved[1]) == notice
        assert archive.read("dartpy/_dartpy.abi3.so") == b"native extension fixture"
        assert archive.read("dartpy-1.0.dist-info/licenses/LICENSE") == (
            b"DART BSD license\n"
        )
        head, _, body = archive.read("dartpy-1.0.dist-info/METADATA").partition(
            newline * 2
        )
        assert body == b"Description body" + newline
        assert head.split(newline)[2:] == [
            b"Version: 1.0",
            b"License-File: LICENSE",
            b"License-File: filament/LICENSE",
            b"License-File: filament/notices/third_party/zstd/LICENSE",
        ]
        record = archive.read("dartpy-1.0.dist-info/RECORD").decode("utf-8")
        rows = {row[0]: row for row in csv.reader(io.StringIO(record))}
        assert set(rows) == set(names)
        assert names[-1] == "dartpy-1.0.dist-info/RECORD"
        assert rows[names[-1]] == [names[-1], "", ""]
        for name in names[:-1]:
            data = archive.read(name)
            digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest())
            assert rows[name] == [
                name,
                f"sha256={digest.rstrip(b'=').decode('ascii')}",
                str(len(data)),
            ]


def test_wheel_without_staged_licenses_is_left_alone(tmp_path):
    module = _load_backend()
    wheel = _wheel(
        tmp_path,
        {
            "dartpy/__init__.py": b"# package\n",
            "dartpy-1.0.dist-info/METADATA": METADATA,
            "dartpy-1.0.dist-info/RECORD": b"kept\n",
        },
    )
    before = wheel.read_bytes()

    assert module.relocate_filament_licenses(wheel) == []

    assert wheel.read_bytes() == before


def test_metadata_without_body_gets_a_terminated_header_block(tmp_path):
    module = _load_backend()
    wheel = _wheel(
        tmp_path,
        {
            "dartpy/_filament_licenses/LICENSE": b"Filament\n",
            "dartpy-1.0.dist-info/METADATA": b"Metadata-Version: 2.4\nName: dartpy\n",
            "dartpy-1.0.dist-info/RECORD": b"stale\n",
        },
    )

    module.relocate_filament_licenses(wheel)

    with zipfile.ZipFile(wheel) as archive:
        assert archive.read("dartpy-1.0.dist-info/METADATA") == (
            b"Metadata-Version: 2.4\nName: dartpy\nLicense-File: filament/LICENSE\n"
        )


def test_missing_record_names_the_wheel(tmp_path):
    module = _load_backend()
    wheel = _wheel(tmp_path, {"dartpy/__init__.py": b"# package\n"})

    with pytest.raises(ValueError, match=rf"{wheel.name}.*RECORD"):
        module.relocate_filament_licenses(wheel)


def test_missing_metadata_with_staged_licenses_names_the_wheel(tmp_path):
    module = _load_backend()
    wheel = _wheel(
        tmp_path,
        {
            "dartpy/_filament_licenses/LICENSE": b"Filament\n",
            "dartpy-1.0.dist-info/RECORD": b"stale\n",
        },
    )
    before = wheel.read_bytes()

    with pytest.raises(ValueError, match=rf"{wheel.name}.*METADATA"):
        module.relocate_filament_licenses(wheel)

    assert wheel.read_bytes() == before
