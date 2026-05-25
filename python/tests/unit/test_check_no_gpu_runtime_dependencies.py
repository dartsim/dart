import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_no_gpu_runtime_dependencies.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_no_gpu_runtime_dependencies",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_no_gpu_runtime_dependency_check_passes_current_manifests():
    module = _load_module()

    assert module.find_no_gpu_runtime_dependency_violations() == []


def test_no_gpu_runtime_dependency_check_accepts_regular_pixi_dependencies():
    module = _load_module()

    violations = module.find_manifest_violations(
        ROOT / "pixi.toml",
        {
            "dependencies": {"numpy": ">=2"},
            "target": {"linux-64": {"dependencies": {"vulkan-headers": ">=1"}}},
        },
    )

    assert violations == []


def test_no_gpu_runtime_dependency_check_rejects_core_pixi_cuda_dependency():
    module = _load_module()

    violations = module.find_manifest_violations(
        ROOT / "pixi.toml",
        {"dependencies": {"cuda-version": "12.*"}},
    )

    assert [(violation.key_path, violation.term) for violation in violations] == [
        ("dependencies.cuda-version", "cuda"),
    ]


def test_no_gpu_runtime_dependency_check_allows_opt_in_pixi_cuda_feature():
    module = _load_module()

    violations = module.find_manifest_violations(
        ROOT / "pixi.toml",
        {
            "feature": {
                "cuda": {
                    "dependencies": {
                        "cuda-version": "12.*",
                        "cuda-cudart-dev": "12.*",
                    },
                },
            },
            "environments": {"cuda": {"features": ["cuda"]}},
        },
    )

    assert violations == []


def test_no_gpu_runtime_dependency_check_rejects_default_feature_cuda_dependency():
    module = _load_module()

    violations = module.find_manifest_violations(
        ROOT / "pixi.toml",
        {
            "feature": {
                "py-default": {
                    "dependencies": {
                        "cuda-version": "12.*",
                    },
                },
            },
            "environments": {
                "default": ["py-default"],
            },
        },
    )

    assert [(violation.key_path, violation.term) for violation in violations] == [
        ("feature.py-default.dependencies.cuda-version", "cuda"),
    ]


def test_no_gpu_runtime_dependency_check_rejects_default_target_cuda_dependency():
    module = _load_module()

    violations = module.find_manifest_violations(
        ROOT / "pixi.toml",
        {"target": {"linux-64": {"dependencies": {"cudatoolkit": "12.*"}}}},
    )

    assert [(violation.key_path, violation.term) for violation in violations] == [
        ("target.linux-64.dependencies.cudatoolkit", "cuda"),
    ]


def test_no_gpu_runtime_dependency_check_rejects_wheel_build_requires_gpu_runtime():
    module = _load_module()

    violations = module.find_manifest_violations(
        ROOT / "pyproject.toml",
        {"build-system": {"requires": ["cupy>=13"]}},
    )

    assert [(violation.key_path, violation.term) for violation in violations] == [
        ("build-system.requires.0", "cupy"),
    ]


def test_no_gpu_runtime_dependency_check_rejects_default_project_dependency():
    module = _load_module()

    violations = module.find_manifest_violations(
        ROOT / "pyproject.toml",
        {"project": {"dependencies": ["nvidia-cuda-runtime-cu12"]}},
    )

    assert [(violation.key_path, violation.term) for violation in violations] == [
        ("project.dependencies.0", "cuda"),
    ]


def test_no_gpu_runtime_dependency_check_rejects_default_wheel_gpu_define():
    module = _load_module()

    violations = module.find_manifest_violations(
        ROOT / "pyproject.toml",
        {
            "tool": {
                "scikit-build": {
                    "cmake": {
                        "define": {
                            "DART_BUILD_CUDA_BACKEND": "ON",
                        },
                    },
                },
            },
        },
    )

    assert [(violation.key_path, violation.term) for violation in violations] == [
        ("tool.scikit-build.cmake.define.DART_BUILD_CUDA_BACKEND", "cuda"),
    ]
