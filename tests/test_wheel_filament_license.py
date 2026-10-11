"""Fetched Filament licenses must accompany the native wheel."""

import hashlib
import runpy
import subprocess
import warnings
import zipfile
from pathlib import Path
from types import SimpleNamespace

import pytest
import setuptools
from setuptools import Distribution, Extension
from setuptools.command.build_ext import build_ext
from setuptools.warnings import SetuptoolsDeprecationWarning


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


@pytest.fixture
def setup_commands(monkeypatch):
    monkeypatch.setattr(setuptools, "setup", lambda **kwargs: None)
    return runpy.run_path(str(Path(__file__).resolve().parents[1] / "setup.py"))


@pytest.mark.parametrize(
    "system, machine, archflags, maxsize, pinned",
    [
        ("linux", "x86_64", "", 2**63 - 1, True),
        ("linux", "aarch64", "", 2**63 - 1, True),
        ("darwin", "arm64", "", 2**63 - 1, True),
        ("darwin", "arm64", "-arch arm64 -arch x86_64", 2**63 - 1, False),
        ("darwin", "x86_64", "", 2**63 - 1, False),
        ("win32", "AMD64", "", 2**63 - 1, True),
        ("win32", "x86", "", 2**31 - 1, False),
        ("freebsd", "x86_64", "", 2**63 - 1, False),
    ],
)
def test_pinned_filament_sdk_platforms(
    monkeypatch, setup_commands, system, machine, archflags, maxsize, pinned
):
    monkeypatch.setattr(setup_commands["sys"], "platform", system)
    monkeypatch.setattr(setup_commands["sys"], "maxsize", maxsize)
    monkeypatch.setattr(setup_commands["platform"], "machine", lambda: machine)
    monkeypatch.setenv("ARCHFLAGS", archflags)
    assert setup_commands["_has_pinned_filament_sdk"]() is pinned


@pytest.mark.parametrize("pinned", [False, True])
def test_wheel_always_builds_gui_with_available_sdk(
    tmp_path, monkeypatch, setup_commands, pinned
):
    command_class = setup_commands["CMakeBuild"]
    extension = setup_commands["CMakeExtension"]("dartpy")
    distribution = Distribution({"ext_modules": [extension]})
    command = command_class(distribution)
    command.ensure_finalized()
    command.build_temp = str(tmp_path / "build")
    command.build_lib = str(tmp_path / "lib")
    command.compiler = SimpleNamespace(compiler_type="unix")
    monkeypatch.setitem(
        command_class.build_extension.__globals__,
        "_has_pinned_filament_sdk",
        lambda: pinned,
    )
    monkeypatch.delenv("CMAKE_ARGS", raising=False)
    monkeypatch.delenv("DARTPY_CMAKE_ARGS_JSON", raising=False)
    calls = []
    monkeypatch.setattr(subprocess, "run", lambda cmd, **kwargs: calls.append(cmd))

    command.build_extension(extension)

    configure, build = calls
    assert "-DDART_BUILD_GUI=ON" in configure
    assert f"-DDART_USE_SYSTEM_FILAMENT={'OFF' if pinned else 'ON'}" in configure
    assert build[:5] == ["cmake", "--build", ".", "--target", "dartpy"]


@pytest.mark.parametrize("with_filament", [False, True])
@pytest.mark.parametrize("reuse_metadata", [False, True])
@pytest.mark.parametrize("abi3", [False, True])
def test_wheel_preserves_dart_license_and_copies_fetched_sdk(
    tmp_path, monkeypatch, setup_commands, with_filament, reuse_metadata, abi3
):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "LICENSE").write_text("DART BSD license\n", encoding="utf-8")
    (tmp_path / "setup.py").write_text("# Wheel fixture\n", encoding="utf-8")
    sdk_license = tmp_path / "sdk" / "LICENSE"
    sdk_license.parent.mkdir()
    sdk_license.write_text("Filament Apache license\n", encoding="utf-8")
    sdk_notice = sdk_license.parent / "notices" / "third_party" / "zstd" / "LICENSE"
    sdk_notice.parent.mkdir(parents=True)
    sdk_notice.write_bytes(b"Upstream bundled Zstandard notice\r\n")

    class FakeBuild(build_ext):
        def build_extension(self, ext):
            extension = Path(self.get_ext_fullpath(ext.name))
            extension.parent.mkdir(parents=True, exist_ok=True)
            extension.write_bytes(b"native extension fixture")
            manifest = Path(self.build_temp) / ext.name / "dartpy-filament-license.txt"
            manifest.parent.mkdir(parents=True)
            manifest.write_text(
                str(sdk_license) if with_filament else "", encoding="utf-8"
            )

    distribution = Distribution(
        {
            "name": "dartpy",
            "version": "1.0",
            "license_files": ["LICENSE"],
            "packages": [],
            "py_modules": [],
            "ext_modules": [Extension("dartpy", sources=[], py_limited_api=abi3)],
            "options": {"bdist_wheel": {"py_limited_api": "cp312"}} if abi3 else {},
            "cmdclass": {
                "build_ext": FakeBuild,
                "bdist_wheel": setup_commands["CMakeWheel"],
            },
        }
    )
    distribution.script_name = "setup.py"
    if reuse_metadata:
        prepared = tmp_path / "prepared" / "dartpy-1.0.dist-info"
        (prepared / "licenses").mkdir(parents=True)
        (prepared / "licenses" / "LICENSE").write_bytes(
            (tmp_path / "LICENSE").read_bytes()
        )
        (prepared / "METADATA").write_text(
            "Metadata-Version: 2.4\nName: dartpy\nVersion: 1.0\n"
            "License-File: LICENSE\n\nPrepared description\n",
            encoding="utf-8",
        )
        distribution.get_command_obj("bdist_wheel").dist_info_dir = str(prepared)
    # The wheel command itself invokes the deprecated legacy install command.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", SetuptoolsDeprecationWarning)
        distribution.run_command("bdist_wheel")

    wheel = next((tmp_path / "dist").glob("*.whl"))
    assert ("-cp312-abi3-" in wheel.name) is abi3
    with zipfile.ZipFile(wheel) as archive:
        prefix = "dartpy-1.0.dist-info/"
        assert archive.read(prefix + "licenses/LICENSE") == b"DART BSD license\n"
        metadata = archive.read(prefix + "METADATA").decode("utf-8")
        assert "License-File: LICENSE\n" in metadata
        if reuse_metadata:
            assert metadata.endswith("Prepared description\n")
        if with_filament:
            assert archive.read(prefix + "licenses/filament/LICENSE") == (
                sdk_license.read_bytes()
            )
            assert "License-File: filament/LICENSE\n" in metadata
            assert (
                archive.read(
                    prefix + "licenses/filament/notices/third_party/zstd/LICENSE"
                )
                == sdk_notice.read_bytes()
            )
            assert (
                "License-File: filament/notices/third_party/zstd/LICENSE\n" in metadata
            )
        else:
            assert prefix + "licenses/filament/LICENSE" not in archive.namelist()
            assert "License-File: filament/LICENSE\n" not in metadata
            assert not any("filament/notices/" in name for name in archive.namelist())


def test_wheel_rejects_missing_declared_sdk_license(tmp_path, setup_commands):
    distribution = Distribution(
        {
            "ext_modules": [Extension("dartpy", sources=[])],
            "cmdclass": {
                "build_ext": setup_commands["CMakeBuild"],
                "bdist_wheel": setup_commands["CMakeWheel"],
            },
        }
    )
    build = distribution.get_command_obj("build_ext")
    build.build_temp = str(tmp_path / "build")
    manifest = Path(build.build_temp) / "dartpy" / "dartpy-filament-license.txt"
    manifest.parent.mkdir(parents=True)
    manifest.write_text(str(tmp_path / "missing-LICENSE"), encoding="utf-8")
    dist_info = tmp_path / "dartpy.dist-info"
    dist_info.mkdir()
    wheel = distribution.get_command_obj("bdist_wheel")
    wheel.ensure_finalized()
    with pytest.raises(RuntimeError, match="Fetched Filament license is missing"):
        wheel.write_wheelfile(str(dist_info))
