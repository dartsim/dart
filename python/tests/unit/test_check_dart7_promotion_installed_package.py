import importlib.util
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_dart7_promotion_installed_package.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_dart7_promotion_installed_package", SCRIPT
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(path: Path, text: str = "") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _write_minimal_prefix(prefix: Path) -> None:
    module = _load_module()
    for relpath in module.REQUIRED_INSTALLED_HEADERS:
        _write(prefix / "include" / relpath)
    _write(
        prefix
        / "share"
        / "dart"
        / "cmake"
        / "dart_simulationComponent.cmake",
        'set("dart_simulation_DEPENDENCIES" dart;io)\n',
    )
    _write(
        prefix / "share" / "dart" / "cmake" / "dart_simulationTargets.cmake",
        "add_library(dart-simulation SHARED IMPORTED)\n",
    )


def _messages(violations) -> list[str]:
    return [violation.message for violation in violations]


def test_minimal_installed_tree_passes(tmp_path):
    module = _load_module()
    _write_minimal_prefix(tmp_path)

    assert module.inspect_installed_tree(tmp_path) == []


def test_missing_required_header_fails(tmp_path):
    module = _load_module()
    _write_minimal_prefix(tmp_path)
    (tmp_path / "include" / module.REQUIRED_INSTALLED_HEADERS[0]).unlink()

    assert any(
        "required promoted header is missing" in message
        for message in _messages(module.inspect_installed_tree(tmp_path))
    )


def test_forbidden_internal_header_fails(tmp_path):
    module = _load_module()
    _write_minimal_prefix(tmp_path)
    _write(tmp_path / "include" / module.FORBIDDEN_INSTALLED_HEADERS[0])

    assert any(
        "internal header must not be installed" in message
        for message in _messages(module.inspect_installed_tree(tmp_path))
    )


def test_private_dependency_marker_fails(tmp_path):
    module = _load_module()
    _write_minimal_prefix(tmp_path)
    cmake_file = (
        tmp_path
        / "share"
        / "dart"
        / "cmake"
        / "dart_simulationTargets.cmake"
    )
    cmake_file.write_text(
        'set_target_properties(dart-simulation PROPERTIES\n'
        '  INTERFACE_LINK_LIBRARIES "EnTT::EnTT"\n'
        ")\n",
        encoding="utf-8",
    )

    assert any(
        "private implementation dependency leaked" in message
        for message in _messages(module.inspect_installed_tree(tmp_path))
    )


def test_component_private_dependency_marker_fails(tmp_path):
    module = _load_module()
    _write_minimal_prefix(tmp_path)
    component_file = (
        tmp_path
        / "share"
        / "dart"
        / "cmake"
        / "dart_simulationComponent.cmake"
    )
    component_file.write_text(
        'set("dart_simulation_DEPENDENCIES" dart;io;spdlog)\n',
        encoding="utf-8",
    )

    assert any(
        "simulation component dependencies" in message
        for message in _messages(module.inspect_installed_tree(tmp_path))
    )


def test_imported_link_dependency_marker_is_allowed(tmp_path):
    module = _load_module()
    _write_minimal_prefix(tmp_path)
    _write(
        tmp_path
        / "share"
        / "dart"
        / "cmake"
        / "dart_simulationTargets-release.cmake",
        'set_target_properties(dart-simulation PROPERTIES\n'
        '  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "spdlog::spdlog;dart-io"\n'
        ")\n",
    )

    assert module.inspect_installed_tree(tmp_path) == []


def test_runtime_env_includes_conda_runtime_libs(tmp_path, monkeypatch):
    module = _load_module()
    prefix = tmp_path / "prefix"
    conda_prefix = tmp_path / "conda"
    (prefix / "lib").mkdir(parents=True)
    (conda_prefix / "lib").mkdir(parents=True)
    monkeypatch.setenv("CONDA_PREFIX", str(conda_prefix))

    if sys.platform.startswith("linux"):
        monkeypatch.delenv("LD_LIBRARY_PATH", raising=False)
        env = module.runtime_env(prefix)
        runtime_paths = env["LD_LIBRARY_PATH"].split(os.pathsep)
        assert str(prefix / "lib") in runtime_paths
        assert str(conda_prefix / "lib") in runtime_paths
    else:
        env = module.runtime_env(prefix)
        assert env["CMAKE_PREFIX_PATH"].split(os.pathsep)[0] == str(prefix)


def test_strict_final_rejects_experimental_package_names(tmp_path):
    module = _load_module()
    _write_minimal_prefix(tmp_path)

    assert any(
        "final promotion must remove simulation" in message
        for message in _messages(module.inspect_installed_tree(tmp_path, strict_final=True))
    )
