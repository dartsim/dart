import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_compute_backend_boundaries.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_compute_backend_boundaries",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_backend_boundary_check_passes_current_tree():
    module = _load_module()

    assert module.find_backend_boundary_violations() == []


def test_backend_boundary_check_allows_gpu_metadata_flag():
    module = _load_module()

    violations = module.find_backend_boundary_violations_in_text(
        "dart/simulation/experimental/compute/compute_stage_metadata.hpp",
        "enum class ComputeStageAcceleration { Gpu = 1u << 4u };",
    )

    assert violations == []


def test_backend_boundary_check_masks_comments_and_strings():
    module = _load_module()

    violations = module.find_backend_boundary_violations_in_text(
        "dart/simulation/experimental/world.hpp",
        """
        // CUDA and stream text in comments is not API.
        const char* diagnostic = "CudaDevice";
        class BackendNeutral {};
        """,
    )

    assert violations == []


def test_backend_boundary_check_rejects_public_cuda_type():
    module = _load_module()

    violations = module.find_backend_boundary_violations_in_text(
        "dart/simulation/experimental/world.hpp",
        "class CudaDevice {};",
    )

    assert [violation.token for violation in violations] == ["CudaDevice"]


def test_backend_boundary_check_rejects_public_sycl_function():
    module = _load_module()

    violations = module.find_backend_boundary_violations_in_text(
        "dart/simulation/experimental/world.hpp",
        "void set_sycl_stream(int stream_id);",
    )

    assert [violation.token for violation in violations] == [
        "set_sycl_stream",
        "stream_id",
    ]


def test_backend_boundary_check_rejects_public_memory_pool_type():
    module = _load_module()

    violations = module.find_backend_boundary_violations_in_text(
        "dart/simulation/experimental/world.hpp",
        "struct MemoryPoolHandle {};",
    )

    assert [violation.token for violation in violations] == ["MemoryPoolHandle"]
