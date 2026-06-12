import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_dart7_promotion_package_contract.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_dart7_promotion_package_contract", SCRIPT
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _write_minimal_contract(root: Path) -> None:
    _write(
        root / "CMakeLists.txt",
        """
dart_option(
  DART_ENABLE_EXPERIMENTAL_CUDA
  "Build opt-in CUDA support"
  OFF
  CATEGORY build
)
dart_option(
  DART_BUILD_DIFF
  "Build opt-in differentiable simulation support"
  OFF
  CATEGORY build
)
""",
    )
    _write(
        root / "dart" / "simulation" / "CMakeLists.txt",
        """
set(target_name ${PROJECT_NAME}-simulation)
set(component_name simulation)

target_link_libraries(
  ${target_name}
  PUBLIC dart Eigen3::Eigen
  PRIVATE EnTT::EnTT spdlog::spdlog
)
target_link_libraries(${target_name} PRIVATE Taskflow::Taskflow)

if(NOT BUILD_SHARED_LIBS)
  add_component_dependency_packages(
    ${PROJECT_NAME}
    ${component_name}
    EnTT
    Taskflow
    spdlog
  )
endif()

set(
  DART_SIMULATION_PUBLIC_HEADERS
  world.hpp
  CACHE INTERNAL
  "Promoted public headers"
)

install(FILES world.hpp DESTINATION include/dart/simulation)
""",
    )


def _messages(violations) -> list[str]:
    return [violation.message for violation in violations]


def test_current_repo_package_contract_passes():
    module = _load_module()

    assert module.find_violations(ROOT) == []


def test_minimal_contract_passes(tmp_path):
    module = _load_module()
    _write_minimal_contract(tmp_path)

    assert module.find_violations(tmp_path) == []


def test_reintroduced_experimental_build_option_fails(tmp_path):
    module = _load_module()
    _write_minimal_contract(tmp_path)
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + """
dart_option(
  DART_BUILD_SIMULATION_EXPERIMENTAL
  "Build staged DART 7 simulation module"
  ON
  CATEGORY build
)
""",
        encoding="utf-8",
    )

    assert any("was retired" in m for m in _messages(module.find_violations(tmp_path)))


def test_recursive_header_install_fails(tmp_path):
    module = _load_module()
    _write_minimal_contract(tmp_path)
    cmake = tmp_path / "dart" / "simulation" / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\ninstall(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION include)\n",
        encoding="utf-8",
    )

    assert any("install(DIRECTORY" in m for m in _messages(module.find_violations(tmp_path)))


def test_public_private_dependency_leak_fails(tmp_path):
    module = _load_module()
    _write_minimal_contract(tmp_path)
    cmake = tmp_path / "dart" / "simulation" / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8").replace(
            "PRIVATE EnTT::EnTT spdlog::spdlog",
            "PUBLIC EnTT::EnTT spdlog::spdlog",
        ),
        encoding="utf-8",
    )

    assert any("EnTT::EnTT must remain a PRIVATE" in m for m in _messages(module.find_violations(tmp_path)))


def test_unconditional_private_dependency_package_fails(tmp_path):
    module = _load_module()
    _write_minimal_contract(tmp_path)
    cmake = tmp_path / "dart" / "simulation" / "CMakeLists.txt"
    cmake.write_text(
        cmake.read_text(encoding="utf-8")
        + "\nadd_component_dependency_packages(${PROJECT_NAME} ${component_name} EnTT)\n",
        encoding="utf-8",
    )

    assert any(
        "allowed only under if(NOT BUILD_SHARED_LIBS)" in m
        for m in _messages(module.find_violations(tmp_path))
    )
