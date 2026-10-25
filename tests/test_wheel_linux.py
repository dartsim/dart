"""Guard the Linux wheel ABI policy and the platform workflow routing."""

import importlib.util
import re
import subprocess
import sys
import tomllib
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_wheel_matrix_and_platform_routing():
    workflow = yaml.load(
        (ROOT / ".github/workflows/publish_dartpy.yml").read_text(),
        Loader=yaml.BaseLoader,
    )
    job = workflow["jobs"]["build_wheels"]
    rows = job["strategy"]["matrix"]["include"]
    linux = [row for row in rows if row["os"] == "linux"]
    # One cp312-abi3 wheel per platform serves Python 3.12 and newer.
    assert {row["build"] for row in linux} == {
        f"cp312-abi3-manylinux_{arch}" for arch in ("x86_64", "aarch64")
    }
    for row in rows:
        assert "-cp312-abi3-" in f"-{row['build']}"
        assert row["abi3"] == "true"
        assert row["release_only"] == "false"
    for row in linux:
        assert row["runner"] == (
            "ubuntu-24.04-arm" if "aarch64" in row["build"] else "ubuntu-24.04"
        )
    assert [(row["runner"], row["build"]) for row in rows if row["os"] != "linux"] == [
        ("macos-15", "cp312-abi3-macosx_arm64"),
        ("windows-2025-vs2026", "cp312-abi3-win_amd64"),
    ]
    steps = {step.get("name"): step for step in job["steps"]}
    assert "matrix.os == 'linux'" in steps["Build and test manylinux wheel"]["if"]
    assert "scripts/wheel_linux.py" in steps["Build and test manylinux wheel"]["run"]
    for name in (
        "Install Pixi wheel environment",
        "Build wheel",
        "Repair wheel",
        "Verify wheel",
        "Test wheel",
        "Audit and test abi3 wheel on Python 3.13",
    ):
        assert "matrix.os != 'linux'" in steps[name]["if"]


def test_linux_driver_repairs_and_tests_in_clean_base(monkeypatch, tmp_path):
    python_tag = "cp312"
    spec = importlib.util.spec_from_file_location(
        "wheel_linux", ROOT / "scripts/wheel_linux.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    run = subprocess.run  # the driver's subprocess.run is replaced below
    monkeypatch.setattr(module, "__file__", str(tmp_path / "scripts/wheel_linux.py"))
    monkeypatch.setattr(module.platform, "machine", lambda: "aarch64")
    monkeypatch.setattr("sys.argv", ["wheel_linux.py", python_tag])
    monkeypatch.setattr(
        module.subprocess, "run", lambda cmd, **kwargs: calls.append((cmd, kwargs))
    )
    module.main()
    build, repair, *smokes = [cmd for cmd, _ in calls]
    assert "BASE_IMAGE=quay.io/pypa/manylinux_2_28_aarch64" in build
    # Repair strips the bundled libraries' symbol tables, which needs
    # auditwheel 6.8.0 or newer.
    assert "--plat manylinux_2_28_aarch64 --only-plat --strip" in repair[-1]
    assert "6.8.0" in repair[-1].split("auditwheel repair")[0]
    # auditwheel wraps --version over several lines; only the first counts.
    script = repair[-1]
    guard = script[
        script.index("auditwheel_version=") : script.index("auditwheel repair")
    ]
    for version, status in (("6.8.2", 0), ("6.10.0", 0), ("6.7.1", 1)):
        stub = (
            "auditwheel() { printf 'auditwheel %s installed at\\n/opt/auditwheel\\n"
            "(python 3.14)\\n'; }\n" % version
        )
        result = run(["bash", "-c", stub + guard], capture_output=True, text=True)
        assert result.returncode == status, (version, result.stderr)
    assert f"/opt/python/{python_tag}-{python_tag}/bin/python" in repair[-1]
    assert "--exclude=build" in repair[-1]
    assert "abi3audit --strict" in repair[-1]
    assert "scikit-build-core" in repair[-1]
    assert "setuptools" not in repair[-1]
    # The abi3 wheel is also tested on a newer interpreter.
    smoke_tags = [python_tag, "cp313"]
    assert len(smokes) == len(smoke_tags)
    for smoke, tag in zip(smokes, smoke_tags):
        assert "quay.io/pypa/manylinux_2_28_aarch64" in smoke
        assert "dartpy-manylinux-2-28-aarch64" not in smoke
        assert "LD_LIBRARY_PATH" in smoke
        assert "--test-ca-fallback" in smoke
        assert f"/opt/python/{tag}-{tag}/bin/python" in smoke
        assert smoke[-1].endswith(f"{python_tag}-*-manylinux_2_28_aarch64.whl")
    assert all(kwargs == {"check": True} for _, kwargs in calls)


@pytest.mark.parametrize("python_tag", ["cp311", "cp313", "cp314"])
def test_linux_driver_rejects_unsupported_python(monkeypatch, python_tag):
    spec = importlib.util.spec_from_file_location(
        "wheel_linux", ROOT / "scripts/wheel_linux.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr("sys.argv", ["wheel_linux.py", python_tag])
    with pytest.raises(SystemExit, match="2"):
        module.main()


def test_native_dependency_image_preserves_collision_backends():
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    assert "quay.io/pypa/manylinux_2_28_x86_64" in dockerfile
    assert "-DUSE_DOUBLE_PRECISION=ON" in dockerfile
    assert "-DENABLE_DOUBLE_PRECISION=ON" in dockerfile
    for setting in (
        "-DODE_DOUBLE_PRECISION=ON",
        "-DODE_WITH_LIBCCD=ON",
        "-DODE_WITH_LIBCCD_SYSTEM=ON",
        "-DODE_WITH_LIBCCD_BOX_CYL=OFF",
    ):
        assert setting in dockerfile
    assert "sha256sum -c -" in dockerfile
    assert "v6.0.5" in dockerfile
    # nanobind is fetched by CMake; the image no longer needs pybind11.
    assert "pybind11" not in dockerfile


def test_dependency_image_fetches_archives_without_git_clones():
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    assert "git clone" not in dockerfile


def test_wheels_enable_the_model_retriever_only_on_linux():
    config = tomllib.loads((ROOT / "pyproject.toml").read_text())
    assert (
        config["tool"]["scikit-build"]["cmake"]["define"]["DART_BUILD_UTILS_ASSETS"]
        == "OFF"
    )
    overrides = config["tool"]["scikit-build"]["overrides"]
    assert [
        rule["if"]["platform-system"]
        for rule in overrides
        if rule.get("cmake", {}).get("define", {}).get("DART_BUILD_UTILS_ASSETS")
        == "ON"
    ] == ["^linux"]
    # Keep conda-forge dependencies available for source builds and future wheels.
    pixi = tomllib.loads((ROOT / "pixi.toml").read_text())
    assert "libcurl" in pixi["dependencies"]
    assert "openssl" in pixi["dependencies"]
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    for setting in (
        "-DCURL_USE_OPENSSL=ON",
        "-DCURL_USE_LIBPSL=OFF",
        "-DCURL_DISABLE_LDAP=ON",
        "-DCURL_CA_BUNDLE=/etc/pki/tls/certs/ca-bundle.crt",
        "-DCURL_CA_PATH=none",
        "-fstack-protector-strong -D_FORTIFY_SOURCE=2",
        "-Wl,-z,relro,-z,now",
    ):
        assert setting in dockerfile
    spec = importlib.util.spec_from_file_location(
        "wheel_tls", ROOT / "scripts/test_wheel_tls.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert f"-DCURL_CA_BUNDLE={module.DEFAULT_CA_BUNDLE.as_posix()}" in dockerfile
    assert "--openssldir=/etc/dartpy/openssl" in dockerfile
    assert dockerfile.count("MODULESDIR=/usr/local/lib/dartpy/ossl-modules") == 2
    assert (
        "strip --strip-unneeded /usr/local/lib/libssl.so.3 /usr/local/lib/libcrypto.so.3"
        in dockerfile
    )
    assert "strip --strip-unneeded /usr/local/lib64/libcurl.so.4" in dockerfile


def test_model_retriever_wheel_smoke_runs_in_the_clean_environment(
    monkeypatch, tmp_path
):
    spec = importlib.util.spec_from_file_location(
        "test_wheel", ROOT / "scripts/test_wheel.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    monkeypatch.setattr(
        module, "run_command", lambda cmd, **kwargs: calls.append((cmd, kwargs))
    )
    monkeypatch.setattr(module.sys, "platform", "linux")
    monkeypatch.setenv("CURL_CA_BUNDLE", "build-machine-ca.pem")
    monkeypatch.setenv("SSL_CERT_FILE", "build-machine-ca.pem")
    wheel = tmp_path / "dartpy.whl"
    wheel.touch()
    module.test_wheel(wheel)
    cmd, options = calls[-1]
    assert options["stage"] == "local HTTPS smoke"
    assert Path(cmd[-1]).name == "test_wheel_tls.py"
    assert Path(cmd[-1]).is_file()
    assert "CURL_CA_BUNDLE" not in options["env"]
    assert "SSL_CERT_FILE" not in options["env"]
    assert options["timeout"] == 180
    module.test_wheel(wheel, test_ca_fallback=True)
    assert calls[-1][0][-1] == "--test-ca-fallback"


def test_dependency_image_builds_sdformat_against_urdfdom_6():
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    headers = "ros/urdfdom_headers/archive/refs/tags/3.0.1.tar.gz"
    parser = "ros/urdfdom/archive/refs/tags/6.0.1.tar.gz"
    assert headers in dockerfile
    assert parser in dockerfile
    # The trailing newline skips the "# sdformat embeds ..." comment above gz-cmake.
    sdformat = dockerfile.index("# sdformat\n")
    assert (
        dockerfile.index("# tinyxml2")
        < dockerfile.index("# console_bridge")
        < dockerfile.index(headers)
        < dockerfile.index(parser)
        < sdformat
    )
    assert (
        "-DUSE_INTERNAL_URDF=OFF" in dockerfile[sdformat : dockerfile.index("# Bullet")]
    )


def test_dependency_image_builds_the_sdformat_major_of_the_conda_wheels():
    pixi = tomllib.loads((ROOT / "pixi.toml").read_text())
    conda_major = re.match(r">=(\d+)\.", pixi["dependencies"]["libsdformat"])[1]
    dockerfile = (ROOT / "docker/wheels/Dockerfile.manylinux_2_28").read_text()
    # Every wheel reads the same SDF versions. sdformat 16 requires gz-cmake 5,
    # gz-math 9 and gz-utils 4, and gz-utils 4 vendors CLI11 only on request.
    assert conda_major == "16"
    for archive in ("gz-cmake5_5.", "gz-utils4_4.", "gz-math9_9.", "sdformat16_16."):
        assert f"/refs/tags/{archive}" in dockerfile
    assert "-DGZ_UTILS_VENDOR_CLI11=ON" in dockerfile


def write_nanobind_package(package):
    """Write a complete fake nanobind 3.1.x package at `package`."""
    (package / "nanobindConfig.cmake").write_text(
        f'set(NB_DIR "{package.as_posix()}")\n'
    )
    (package / "include/nanobind").mkdir(parents=True)
    (package / "include/nanobind/nanobind.h").write_text(
        "#define NB_VERSION_MAJOR 3\n"
        "#define NB_VERSION_MINOR 1\n"
        "#define NB_VERSION_PATCH 0\n"
    )
    (package / "src").mkdir()
    (package / "src/nb_internals.h").write_text("#  define NB_INTERNALS_VERSION 22\n")
    (package / "nanobindConfigVersion.cmake").write_text(
        'set(PACKAGE_VERSION "3.1.0")\n' "set(PACKAGE_VERSION_COMPATIBLE TRUE)\n"
    )


@pytest.mark.parametrize("wheels", [True, False])
@pytest.mark.parametrize("stable_abi", [True, False])
def test_python_discovery_needs_no_embedding_library(tmp_path, wheels, stable_abi):
    selected_python = (tmp_path / "selected-python").as_posix()
    (tmp_path / "FindPython.cmake").write_text(
        "if(Python_FOUND)\n"
        '  message(FATAL_ERROR "Python discovered twice")\n'
        "endif()\n"
        'if(NOT Python_FIND_VERSION STREQUAL "3.12")\n'
        '  message(FATAL_ERROR "Python floor was not requested")\n'
        "endif()\n"
        'if(NOT "Interpreter" IN_LIST Python_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Interpreter was not requested")\n'
        "endif()\n"
        'if("Development" IN_LIST Python_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Embedding library unavailable")\n'
        "endif()\n"
        'if(NOT "Development.Module" IN_LIST Python_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Extension headers were not requested")\n'
        "endif()\n"
        'if(DART_DARTPY_STABLE_ABI AND NOT "Development.SABIModule" IN_LIST Python_FIND_COMPONENTS)\n'
        '  message(FATAL_ERROR "Stable ABI headers were not requested")\n'
        "endif()\n"
        "set(Python_FOUND TRUE)\n"
        'if(NOT Python_EXECUTABLE OR Python_EXECUTABLE STREQUAL "legacy-python")\n'
        f'  set(Python_EXECUTABLE "{selected_python}")\n'
        "endif()\n"
    )
    (tmp_path / "FindPython3.cmake").write_text(
        'message(FATAL_ERROR "Duplicate Python3 discovery")\n'
    )
    write_nanobind_package(tmp_path)
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(PythonComponents NONE)\n"
        "macro(dart_find_package)\nendmacro()\n"
        "macro(dart_check_required_package)\nendmacro()\n"
        "macro(dart_check_optional_package)\nendmacro()\n"
        "function(add_subdirectory)\nendfunction()\n"
        f'set(CMAKE_MODULE_PATH "{tmp_path.as_posix()}")\n'
        "set(DART_BUILD_DARTPY ON)\n"
        f'set(DART_BUILD_WHEELS {"ON" if wheels else "OFF"})\n'
        f'set(DART_DARTPY_STABLE_ABI {"ON" if stable_abi else "OFF"})\n'
        "set(DART_USE_SYSTEM_ODE ON)\n"
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        f'include("{ROOT.as_posix()}/cmake/DARTFindDependencies.cmake")\n'
        "if(NOT DEFINED Python3_EXECUTABLE)\n"
        '  set(Python3_EXECUTABLE "legacy-python")\n'
        "endif()\n"
        "set(DART_USE_SYSTEM_NANOBIND ON)\n"
        f'set(nanobind_DIR "{tmp_path.as_posix()}")\n'
        f'include("{ROOT.as_posix()}/python/CMakeLists.txt")\n'
        "if(NOT DEFINED expected_python)\n"
        f'  set(expected_python "{selected_python}")\n'
        "endif()\n"
        "get_property(selected CACHE Python3_EXECUTABLE PROPERTY VALUE)\n"
        'if(NOT selected STREQUAL "${expected_python}")\n'
        '  message(FATAL_ERROR "Selected interpreter was not cached")\n'
        "endif()\n"
        'if(NOT Python3_EXECUTABLE STREQUAL "${expected_python}")\n'
        '  message(FATAL_ERROR "Legacy interpreter shadows selected Python")\n'
        "endif()\n"
        "get_property(last_forced CACHE _DART_PYTHON3_EXECUTABLE PROPERTY VALUE)\n"
        'if(NOT last_forced STREQUAL "${expected_python}")\n'
        '  message(FATAL_ERROR "Last forced interpreter was not cached")\n'
        "endif()\n"
    )
    result = subprocess.run(
        ["cmake", "-S", str(tmp_path), "-B", str(tmp_path / "build")],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "will be ignored" not in result.stderr

    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(tmp_path),
            "-B",
            str(tmp_path / "build"),
            "-DPython3_EXECUTABLE=different-python",
        ],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Python3_EXECUTABLE differs from Python_EXECUTABLE" in result.stderr
    assert "Set Python_EXECUTABLE to select the Python interpreter" in " ".join(
        result.stderr.split()
    )

    new_python = (tmp_path / "new-python").as_posix()
    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(tmp_path),
            "-B",
            str(tmp_path / "build"),
            f"-DPython_EXECUTABLE={new_python}",
            f"-Dexpected_python={new_python}",
        ],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "will be ignored" not in result.stderr


def test_builtin_findpython_primary_persists_across_configures(tmp_path):
    write_nanobind_package(tmp_path)
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(BuiltinFindPython NONE)\n"
        "function(add_subdirectory)\nendfunction()\n"
        "set(DART_BUILD_DARTPY ON)\n"
        "set(DART_DARTPY_STABLE_ABI OFF)\n"
        "set(DART_USE_SYSTEM_NANOBIND ON)\n"
        f'set(nanobind_DIR "{tmp_path.as_posix()}")\n'
        f'include("{ROOT.as_posix()}/python/CMakeLists.txt")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/selected.txt" "${Python_EXECUTABLE}")\n'
    )
    build = tmp_path / "build"

    def configure(*args):
        return subprocess.run(
            ["cmake", "-S", str(tmp_path), "-B", str(build), *args],
            capture_output=True,
            text=True,
            check=False,
            timeout=60,
        )

    def cached(name):
        for line in (build / "CMakeCache.txt").read_text().splitlines():
            if line.startswith(f"{name}:"):
                return line.split("=", 1)[1]
        return None

    result = configure()
    assert result.returncode == 0, result.stdout + result.stderr
    selected = (build / "selected.txt").read_text()
    assert Path(selected).is_file()
    for name in ("Python_EXECUTABLE", "Python3_EXECUTABLE", "_DART_PYTHON3_EXECUTABLE"):
        assert cached(name) == selected

    # A later alias must not seed an empty primary: the selection persists.
    result = configure("-DPython3_EXECUTABLE=/nonexistent/python")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Python3_EXECUTABLE differs from Python_EXECUTABLE" in result.stderr
    assert (build / "selected.txt").read_text() == selected
    for name in ("Python_EXECUTABLE", "Python3_EXECUTABLE", "_DART_PYTHON3_EXECUTABLE"):
        assert cached(name) == selected


def test_dartpy_disabled_does_not_discover_python(tmp_path):
    for name in ("Python", "Python3"):
        (tmp_path / f"Find{name}.cmake").write_text(
            f'message(FATAL_ERROR "Unexpected {name} discovery")\n'
        )
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(PythonDisabled NONE)\n"
        "macro(dart_find_package)\nendmacro()\n"
        "macro(dart_check_required_package)\nendmacro()\n"
        "macro(dart_check_optional_package)\nendmacro()\n"
        f'set(CMAKE_MODULE_PATH "{tmp_path.as_posix()}")\n'
        "set(DART_BUILD_DARTPY OFF)\n"
        "set(DART_USE_SYSTEM_ODE ON)\n"
        "set(DART_USE_SYSTEM_BULLET ON)\n"
        f'include("{ROOT.as_posix()}/cmake/DARTFindDependencies.cmake")\n'
        f'include("{ROOT.as_posix()}/python/CMakeLists.txt")\n'
    )
    result = subprocess.run(
        ["cmake", "-S", str(tmp_path), "-B", str(tmp_path / "build")],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_smoke_helper_does_not_require_python_on_path(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location(
        "test_wheel", ROOT / "scripts/test_wheel.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    wheel = tmp_path / "dartpy.whl"
    wheel.touch()
    tested = []
    monkeypatch.setenv("PATH", "")
    monkeypatch.setattr(module, "test_wheel", tested.append)
    assert module.main(["test_wheel.py", str(wheel)]) == 0
    assert tested == [wheel]


@pytest.mark.parametrize("platform", ["darwin", "win32"])
def test_disabled_retriever_skips_tls_smoke(monkeypatch, tmp_path, platform):
    spec = importlib.util.spec_from_file_location(
        "test_wheel", ROOT / "scripts/test_wheel.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    monkeypatch.setattr(module.sys, "platform", platform)
    monkeypatch.setattr(
        module, "run_command", lambda cmd, **kwargs: calls.append(kwargs["stage"])
    )
    wheel = tmp_path / "dartpy.whl"
    wheel.touch()
    module.test_wheel(wheel)
    assert "native smoke" in calls
    assert "local HTTPS smoke" not in calls


@pytest.mark.skipif(
    sys.platform not in {"linux", "darwin"}, reason="POSIX C++ toolchain"
)
@pytest.mark.parametrize("apple_wheel", [False, True])
@pytest.mark.parametrize("init_succeeds", [False, True])
def test_apple_wheel_ignores_compiled_ca_and_config(
    tmp_path, apple_wheel, init_succeeds
):
    openssl = tmp_path / "openssl"
    openssl.mkdir()
    (openssl / "crypto.h").write_text(
        "#pragma once\n#include <cassert>\n"
        "#define OPENSSL_INIT_NO_LOAD_CONFIG 128ULL\n"
        "inline int calls = 0;\n"
        "inline int OPENSSL_init_crypto(unsigned long long options, const void* config) {\n"
        "  assert(options == OPENSSL_INIT_NO_LOAD_CONFIG && config == nullptr);\n"
        "  ++calls;\n"
        f"  return {int(init_succeeds)};\n"
        "}\n"
    )
    source = tmp_path / "ca.cpp"
    initialization = (
        (
            f"  assert(dart::utils::detail::initializeWheelCrypto() == {str(init_succeeds).lower()});\n"
            * 2
            + "  assert(calls == 1);\n"
        )
        if apple_wheel
        else "  assert(calls == 0);\n"
    )
    source.write_text(
        "#include <openssl/crypto.h>\n"
        '#include "dart/utils/assets/detail/CaBundle.hpp"\n'
        "#include <cassert>\n"
        "int main() {\n"
        '  const char* bundle = "build-environment.pem";\n'
        f'  assert(dart::utils::detail::defaultCaBundle(bundle) == {"nullptr" if apple_wheel else "bundle"});\n'
        + initialization
        + "}\n"
    )
    import shutil

    compiler = shutil.which("c++")
    if not compiler:
        pytest.skip("C++ compiler unavailable")
    executable = tmp_path / "ca"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            f"-I{tmp_path}",
            f"-I{ROOT}",
            *(["-DDART_APPLE_WHEEL_TLS"] if apple_wheel else []),
            str(source),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
    )
    subprocess.run([str(executable)], check=True)


def test_linux_prs_run_wheel_tooling_and_failures_upload_artifacts():
    config = tomllib.loads((ROOT / "pixi.toml").read_text())
    tooling = config["tasks"]["test-wheel-tooling"]["cmd"]
    requirements = config["tasks"]["test-build-requirements"]["cmd"]
    for name in (
        "verify_wheel",
        "wheel_linux",
        "wheel_tls",
        "wheel_runtime_environment",
    ):
        assert f"tests/test_{name}.py" in tooling
        assert f"tests/test_{name}.py" in requirements
    workflow = yaml.load(
        (ROOT / ".github/workflows/ci_ubuntu.yml").read_text(), Loader=yaml.BaseLoader
    )
    assert "pull_request" in workflow["on"]
    assert any(
        step.get("run") == "pixi run test-wheel-tooling"
        for job in workflow["jobs"].values()
        for step in job["steps"]
    )
    publish = yaml.load(
        (ROOT / ".github/workflows/publish_dartpy.yml").read_text(),
        Loader=yaml.BaseLoader,
    )
    upload = next(
        step
        for step in publish["jobs"]["build_wheels"]["steps"]
        if step.get("uses", "").startswith("actions/upload-artifact@")
    )
    assert "!cancelled()" in upload["if"]
    assert "steps.wheel_gate.outputs.enabled == 'true'" in upload["if"]


def test_apple_crypto_guard_is_wired_to_cmake_and_constructor():
    import re

    cmake = (ROOT / "dart/utils/assets/CMakeLists.txt").read_text()
    assert re.search(
        r"if\(DART_BUILD_WHEELS AND APPLE\)\s+set_property\(\s+"
        r"SOURCE ModelResourceRetriever.cpp TARGET_DIRECTORY dart-io\s+"
        r"APPEND\s+PROPERTY COMPILE_DEFINITIONS DART_APPLE_WHEEL_TLS\s+\)\s+endif\(\)",
        cmake,
    )
    source = (ROOT / "dart/utils/assets/ModelResourceRetriever.cpp").read_text()
    constructor = source.split("ModelResourceRetriever::ModelResourceRetriever(", 1)[1]
    assert re.search(
        r"#ifdef DART_APPLE_WHEEL_TLS\s+if \(!detail::initializeWheelCrypto\(\)\)\s+"
        r'throw std::runtime_error\("OpenSSL initialization failed"\);\s+#endif',
        constructor,
    )


@pytest.mark.skipif(
    sys.platform not in {"linux", "darwin"}, reason="POSIX C++ toolchain"
)
def test_only_proxy_options_tolerate_unknown_option(tmp_path):
    import shutil

    compiler = shutil.which("c++")
    if not compiler:
        pytest.skip("C++ compiler unavailable")
    source = (ROOT / "dart/utils/assets/ModelResourceRetriever.cpp").read_text()
    options = source.split("  const auto setOption =", 1)[1].split(
        "  setOption(CURLOPT_URL", 1
    )[0]
    program = tmp_path / "options.cpp"
    program.write_text(
        "#include <cassert>\n#include <stdexcept>\n"
        "enum CURLcode { CURLE_OK, CURLE_UNKNOWN_OPTION, CURLE_FAILED_INIT };\n"
        "using CURLoption = int;\nCURLcode result;\n"
        "struct { void* get() { return nullptr; } } handle;\n"
        "CURLcode curl_easy_setopt(void*, CURLoption, long) { return result; }\n"
        "int main() {\n  const auto setOption ="
        + options
        + "  for (auto code : {CURLE_OK, CURLE_UNKNOWN_OPTION, CURLE_FAILED_INIT}) {\n"
        "    result = code;\n"
        "    bool failed = false;\n"
        "    try { setOption(0, 1L); } catch (const std::runtime_error&) { failed = true; }\n"
        "    assert(failed == (code != CURLE_OK));\n"
        "    failed = false;\n"
        "    try { setProxyOption(0, 1L); } catch (const std::runtime_error&) { failed = true; }\n"
        "    assert(failed == (code == CURLE_FAILED_INIT));\n"
        "  }\n}\n"
    )
    executable = tmp_path / "options"
    subprocess.run(
        [compiler, "-std=c++17", str(program), "-o", str(executable)], check=True
    )
    subprocess.run([str(executable)], check=True)
    import re

    assert not re.search(r"setOption\(\s*CURLOPT_PROXY_", source)
    assert not re.search(r"setProxyOption\(\s*CURLOPT_(?!PROXY_)", source)
