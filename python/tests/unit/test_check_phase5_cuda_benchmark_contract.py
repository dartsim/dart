import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_phase5_cuda_benchmark_contract.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_phase5_cuda_benchmark_contract",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_current_repo_phase5_cuda_benchmark_contract_passes():
    module = _load_module()

    assert module.find_phase5_cuda_benchmark_contract_violations() == []


def test_phase5_cuda_benchmark_contract_ignores_non_cuda_benchmarks():
    module = _load_module()

    violations = module.find_violations_in_text(
        "tests/benchmark/simulation/experimental/bm_compute_graph.cpp",
        "BENCHMARK(BM_WorldStep)->Arg(1024);",
    )

    assert violations == []


def test_phase5_cuda_benchmark_contract_rejects_legacy_smoke_row_shape():
    module = _load_module()

    violations = module.find_violations_in_text(
        "tests/benchmark/simulation/experimental/bm_cuda_rigid_body_state_batch.cpp",
        """
        void BM_CudaRigidBodyStateBatchLinear(benchmark::State& state) {}
        BENCHMARK(BM_CudaRigidBodyStateBatchLinear)->Arg(256)->Arg(4096);
        """,
    )

    assert [violation.message for violation in violations] == [
        "CUDA benchmark files must register BM_Phase5RigidBodyBatchGpu",
        "BM_Phase5RigidBodyBatchGpu must include the 4096/128/100 manual go/no-go workload",
    ]


def test_phase5_cuda_benchmark_contract_ignores_commented_required_row():
    module = _load_module()

    violations = module.find_violations_in_text(
        "tests/benchmark/simulation/experimental/bm_cuda_rigid_body_state_batch.cpp",
        """
        // BENCHMARK(BM_Phase5RigidBodyBatchGpu)->Args({4096, 128, 100});
        void BM_CudaRigidBodyStateBatchLinear(benchmark::State& state) {}
        """,
    )

    assert len(violations) == 2


def test_phase5_cuda_benchmark_contract_accepts_packet_compatible_row():
    module = _load_module()

    violations = module.find_violations_in_text(
        "tests/benchmark/simulation/experimental/bm_cuda_rigid_body_state_batch.cpp",
        """
        void BM_Phase5RigidBodyBatchGpu(benchmark::State& state) {}
        BENCHMARK(BM_Phase5RigidBodyBatchGpu)
            ->Args({1024, 128, 10})
            ->Args({4096, 128, 100});
        """,
    )

    assert violations == []


def test_phase5_cuda_benchmark_contract_binds_workload_to_phase5_row():
    module = _load_module()

    violations = module.find_violations_in_text(
        "tests/benchmark/simulation/experimental/bm_cuda_rigid_body_state_batch.cpp",
        """
        void BM_Phase5RigidBodyBatchGpu(benchmark::State& state) {}
        void BM_CudaOther(benchmark::State& state) {}
        BENCHMARK(BM_Phase5RigidBodyBatchGpu)->Args({1024, 128, 10});
        BENCHMARK(BM_CudaOther)->Args({4096, 128, 100});
        """,
    )

    assert [violation.message for violation in violations] == [
        "BM_Phase5RigidBodyBatchGpu must include the 4096/128/100 manual go/no-go workload",
    ]
