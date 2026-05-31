import importlib.util
import sys
from pathlib import Path


def load_test_all_module():
    repo_root = Path(__file__).resolve().parents[1]
    scripts_dir = repo_root / "scripts"
    sys.path.insert(0, str(scripts_dir))
    spec = importlib.util.spec_from_file_location(
        "dart_test_all", scripts_dir / "test_all.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_pixi_command_preserves_cuda_environment(monkeypatch):
    module = load_test_all_module()

    monkeypatch.setenv("PIXI_BIN", "/opt/pixi")
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "cuda")

    assert (
        module.pixi_command("test-cuda", "Release")
        == "/opt/pixi run -e cuda test-cuda Release"
    )


def test_pixi_command_omits_default_environment(monkeypatch):
    module = load_test_all_module()

    monkeypatch.setenv("PIXI_BIN", "/opt/pixi")
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "default")

    assert module.pixi_command("test-all") == "/opt/pixi run test-all"


def test_cuda_environment_detection_uses_pixi_environment(monkeypatch):
    module = load_test_all_module()

    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "cuda")
    monkeypatch.delenv("DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE", raising=False)

    assert module._cuda_environment_active()


def test_cuda_environment_detection_uses_override(monkeypatch):
    module = load_test_all_module()

    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "default")
    monkeypatch.setenv("DART_ENABLE_EXPERIMENTAL_CUDA_OVERRIDE", "ON")

    assert module._cuda_environment_active()


def test_configured_feature_disabled_by_override(monkeypatch):
    module = load_test_all_module()

    monkeypatch.setenv("DART_BUILD_EXAMPLES_OVERRIDE", "OFF")
    monkeypatch.setattr(module, "_cmake_option_enabled", lambda option: True)

    assert not module._configured_feature_enabled(
        "DART_BUILD_EXAMPLES_OVERRIDE", "DART_BUILD_EXAMPLES"
    )


def test_configured_feature_disabled_by_cmake_cache(monkeypatch):
    module = load_test_all_module()

    monkeypatch.delenv("DART_BUILD_EXAMPLES_OVERRIDE", raising=False)
    monkeypatch.setattr(module, "_cmake_option_enabled", lambda option: False)

    assert not module._configured_feature_enabled(
        "DART_BUILD_EXAMPLES_OVERRIDE", "DART_BUILD_EXAMPLES"
    )


def test_configured_feature_enabled_without_disable(monkeypatch):
    module = load_test_all_module()

    monkeypatch.delenv("DART_BUILD_EXAMPLES_OVERRIDE", raising=False)
    monkeypatch.setattr(module, "_cmake_option_enabled", lambda option: None)

    assert module._configured_feature_enabled(
        "DART_BUILD_EXAMPLES_OVERRIDE", "DART_BUILD_EXAMPLES"
    )
