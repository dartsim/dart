"""verify_wheel.py accepts the packaged dartpy layout and rejects stray files."""

import importlib.util
import re
import tomllib
import zipfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DIST_INFO = "dartpy-1.0.dist-info"
TLS_NOTICES = sorted(
    Path(path).name
    for path in tomllib.loads((ROOT / "pyproject.toml").read_text())["project"][
        "license-files"
    ]
    if path.startswith("python/dartpy/licenses/")
)
PACKAGE = {
    "dartpy/__init__.py": b"# package\n",
    "dartpy/_core.abi3.so": b"native extension fixture",
    "dartpy/py.typed": b"",
    "dartpy/experimental/__init__.py": b'"""Outside SemVer."""\n',
    f"{DIST_INFO}/METADATA": b"Metadata-Version: 2.4\nName: dartpy\nVersion: 1.0\n",
    f"{DIST_INFO}/licenses/LICENSE": b"DART BSD license\n",
    f"{DIST_INFO}/licenses/NOTICE": b"DART third-party notices\n",
    f"{DIST_INFO}/licenses/filament/LICENSE": b"Filament Apache license\n",
    **{
        f"{DIST_INFO}/licenses/python/dartpy/licenses/{name}": b"TLS notice\n"
        for name in TLS_NOTICES
    },
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
            "dartpy.libs/": b"",
            "dartpy.libs/libfcl-54c78b3b.so.0.7.0": b"vendored",
            f"{DIST_INFO}/licenses/python/dartpy/licenses/fcl.txt": b"FCL\n",
            "dartpy/.dylibs/x": b"",
        },
    )
    assert _load().verify_wheel(wheel)
    output = capsys.readouterr().out
    assert "Found extension: dartpy/_core.abi3.so" in output
    assert f"Wheel size: {wheel.stat().st_size:,} bytes compressed" in output
    assert "bytes unpacked" in output
    assert "Bundled library: dartpy.libs/libfcl-54c78b3b.so.0.7.0" in output
    assert "Bundled library: dartpy.libs/ (" not in output
    assert "Bundled library: dartpy/.dylibs/x" in output


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"tests/__init__.py": b""}, "files outside the dartpy package: tests/"),
        ({"dartpy/_filament_licenses/LICENSE": b""}, "dartpy/_filament_licenses/"),
        ({"dartpy/py.typed": None}, "missing dartpy/py.typed"),
        ({"dartpy/experimental/__init__.py": None}, "missing dartpy/experimental/"),
        ({f"{DIST_INFO}/licenses/NOTICE": None}, "licenses/NOTICE"),
        ({f"{DIST_INFO}/licenses/filament/LICENSE": None}, "licenses/filament/LICENSE"),
        ({"dartpy/_core.abi3.so": None}, "No dartpy._core extension"),
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


@pytest.mark.parametrize(
    "notice",
    [
        "LICENSE",
        "NOTICE",
        "filament/LICENSE",
        *[f"python/dartpy/licenses/{name}" for name in TLS_NOTICES],
    ],
)
def test_rejects_missing_notices(tmp_path, capsys, notice):
    entries = {**PACKAGE}
    del entries[f"{DIST_INFO}/licenses/{notice}"]
    assert not _load().verify_wheel(_wheel(tmp_path, entries))
    assert f"missing {DIST_INFO}/licenses/{notice}" in capsys.readouterr().out


@pytest.mark.parametrize(
    "platform",
    [
        "manylinux_2_28_x86_64",
        "manylinux_2_28_aarch64",
        "macosx_15_0_arm64",
        "win_amd64",
    ],
)
def test_platform_size_budget(tmp_path, capsys, platform):
    module = _load()
    wheel = _wheel(tmp_path, PACKAGE, f"dartpy-1.0-cp312-abi3-{platform}.whl")
    key = "linux" if platform.startswith("manylinux") else platform.split("_")[0]
    module.WHEEL_SIZE_LIMITS[key] = wheel.stat().st_size
    assert module.verify_wheel(wheel)
    module.WHEEL_SIZE_LIMITS[key] -= 1
    assert not module.verify_wheel(wheel)
    assert "exceeds the" in capsys.readouterr().out


def test_linux_wheel_needs_every_bundled_library_notice(tmp_path, capsys):
    # auditwheel names a bundled library <name>-<8 hex digits>.so.<version>.
    libraries = {
        f"dartpy.libs/{library}-0123abcd.so.1.2": notices
        for library, notices in _load().BUNDLED_NOTICES.items()
    }
    every_notice = sorted({n for notices in libraries.values() for n in notices})
    prefix = f"{DIST_INFO}/licenses/python/dartpy/licenses/"
    entries = {
        **PACKAGE,
        **dict.fromkeys(libraries, b"library"),
        **{prefix + notice: b"Notice\n" for notice in every_notice},
    }
    assert _load().verify_wheel(_wheel(tmp_path, entries))
    capsys.readouterr()
    for notice in every_notice:
        del entries[prefix + notice]
        assert not _load().verify_wheel(_wheel(tmp_path, entries))
        output = capsys.readouterr().out
        for library in (name for name, n in libraries.items() if notice in n):
            assert f"missing {prefix}{notice} for {library}." in output
        entries[prefix + notice] = b"Notice\n"


def test_linux_wheel_needs_the_notices_of_vendored_code():
    notices = _load().BUNDLED_NOTICES
    assert {
        "assimp-earcut.txt",
        "assimp-openddlparser.txt",
        "assimp-poly2tri.txt",
        "assimp-pugixml.txt",
        "assimp-rapidjson.txt",
    } <= set(notices["libassimp"])
    assert {"ode-opcode.txt", "ode-ou.txt"} <= set(notices["libode"])
    assert "libjpeg-turbo-ijg.txt" in notices["libjpeg"]


def test_linux_wheel_rejects_an_unmapped_bundled_library(tmp_path, capsys):
    library = "dartpy.libs/libunknown-0123abcd.so.1"
    wheel = _wheel(tmp_path, {**PACKAGE, library: b"library"})
    assert not _load().verify_wheel(wheel)
    assert f"No notice is mapped for the bundled library {library}" in (
        capsys.readouterr().out
    )


def test_bundled_library_notices_are_collected_or_listed():
    shipped = set(TLS_NOTICES)
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    shipped.update(re.findall(r'"\$DARTPY_LICENSES/([^"]+)"', dockerfile))
    required = {n for notices in _load().BUNDLED_NOTICES.values() for n in notices}
    assert required <= shipped


@pytest.mark.parametrize("platform", ["macosx_15_0_arm64", "win_amd64"])
def test_only_linux_wheels_need_bundled_library_notices(tmp_path, platform):
    wheel = _wheel(
        tmp_path,
        {**PACKAGE, "dartpy.libs/libunknown-0123abcd.dll": b"library"},
        f"dartpy-1.0-cp312-abi3-{platform}.whl",
    )
    assert _load().verify_wheel(wheel)


@pytest.mark.parametrize("platform", ["macosx_15_0_arm64", "win_amd64"])
def test_disabled_retriever_needs_no_tls_notices(tmp_path, capsys, platform):
    entries = {
        name: data
        for name, data in PACKAGE.items()
        if "/python/dartpy/licenses/" not in name
    }
    wheel = _wheel(tmp_path, entries, f"dartpy-1.0-cp312-abi3-{platform}.whl")
    assert _load().verify_wheel(wheel)
    entries["dartpy.libs/libcurl.dll"] = b"curl"
    assert not _load().verify_wheel(_wheel(tmp_path, entries, wheel.name))
    assert "licenses/curl.txt" in capsys.readouterr().out


def test_unknown_platform_has_no_implicit_size_budget(tmp_path, capsys):
    assert not _load().verify_wheel(
        _wheel(tmp_path, PACKAGE, "dartpy-1.0-cp312-abi3-any.whl")
    )
    assert "No wheel size budget" in capsys.readouterr().out


def test_size_failure_reports_inventory_and_remaining_checks(tmp_path, capsys):
    module = _load()
    module.WHEEL_SIZE_LIMITS["linux"] = 0
    entries = {**PACKAGE, "dartpy.libs/libcurl.so": b"curl", "stray": b""}
    del entries[f"{DIST_INFO}/licenses/NOTICE"]
    assert not module.verify_wheel(_wheel(tmp_path, entries))
    output = capsys.readouterr().out
    assert output.index("Wheel size:") < output.index("exceeds the")
    assert output.index("Bundled library:") < output.index("exceeds the")
    assert "licenses/NOTICE" in output
    assert "files outside the dartpy package: stray" in output


def test_documented_size_budgets_and_shipped_notices_match_verifier():
    module = _load()
    documentation = (ROOT / "docs/onboarding/python-bindings.md").read_text()
    rows = re.findall(
        r"\| (Linux|macOS|Windows) [^|]+\|[^|]+\| ([\d,]+) \|", documentation
    )
    assert len(rows) == 4
    keys = {"Linux": "linux", "macOS": "macosx", "Windows": "win"}
    for platform, limit in rows:
        assert int(limit.replace(",", "")) == module.WHEEL_SIZE_LIMITS[keys[platform]]
    assert set(TLS_NOTICES) == set(module.TLS_NOTICES) == {"curl.txt", "openssl.txt"}
