#!/usr/bin/env python3
"""Check Phase 5 CUDA benchmark names against the go/no-go packet contract.

CUDA benchmark files are optional (the GPU go/no-go is measured on a CUDA host,
not in CI), but any that land must be shaped so their Google Benchmark JSON can
feed `check_phase5_gpu_packet.py` without ad hoc renaming. In practice this
means the file must register the matching Phase 5 GPU row name and full manual
go/no-go workload:

  BM_Phase5RigidBodyBatchGpu/4096/128/100

The routine CPU smoke row stays in `bm_compute_graph.cpp`; this checker only
guards optional CUDA benchmark files.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCHMARK_ROOT = REPO_ROOT / "tests" / "benchmark" / "simulation" / "experimental"

REQUIRED_GPU_BENCHMARK = "BM_Phase5RigidBodyBatchGpu"
REQUIRED_WORKLOAD = (4096, 128, 100)
CUDA_BENCHMARK_MARKERS = (
    "BM_Cuda",
    "CUDA",
    "Cuda",
    "DART_ENABLE_EXPERIMENTAL_CUDA",
    "cuda::",
)

SOURCE_SUFFIXES = {".cpp", ".cu", ".cuh"}
_BENCHMARK_REGISTRATION_RE = re.compile(
    r"\bBENCHMARK\s*\(\s*(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*\)(?P<chain>[^;]*);",
    re.DOTALL,
)


@dataclass(frozen=True)
class Violation:
    path: str
    message: str


def _mask_comments_and_literals(text: str) -> str:
    result: list[str] = []
    i = 0
    state = "code"
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""

        if state == "code":
            if ch == "/" and nxt == "/":
                result.extend((" ", " "))
                i += 2
                state = "line_comment"
            elif ch == "/" and nxt == "*":
                result.extend((" ", " "))
                i += 2
                state = "block_comment"
            elif ch == '"':
                result.append(" ")
                i += 1
                state = "string"
            elif ch == "'":
                result.append(" ")
                i += 1
                state = "char"
            else:
                result.append(ch)
                i += 1
        elif state == "line_comment":
            result.append("\n" if ch == "\n" else " ")
            i += 1
            if ch == "\n":
                state = "code"
        elif state == "block_comment":
            if ch == "*" and nxt == "/":
                result.extend((" ", " "))
                i += 2
                state = "code"
            else:
                result.append("\n" if ch == "\n" else " ")
                i += 1
        elif state == "string":
            if ch == "\\" and nxt:
                result.extend((" ", " "))
                i += 2
            else:
                result.append("\n" if ch == "\n" else " ")
                i += 1
                if ch == '"':
                    state = "code"
        elif state == "char":
            if ch == "\\" and nxt:
                result.extend((" ", " "))
                i += 2
            else:
                result.append("\n" if ch == "\n" else " ")
                i += 1
                if ch == "'":
                    state = "code"

    return "".join(result)


def _is_cuda_benchmark_source(path: Path, text: str) -> bool:
    if path.suffix not in SOURCE_SUFFIXES:
        return False
    if "cuda" in path.name.lower():
        return True
    return any(marker in text for marker in CUDA_BENCHMARK_MARKERS)


def _has_required_benchmark_name(masked: str) -> bool:
    return any(
        match.group("name") == REQUIRED_GPU_BENCHMARK
        for match in _BENCHMARK_REGISTRATION_RE.finditer(masked)
    )


def _has_required_workload(masked: str) -> bool:
    world_count, body_count, step_count = REQUIRED_WORKLOAD
    for match in _BENCHMARK_REGISTRATION_RE.finditer(masked):
        if match.group("name") != REQUIRED_GPU_BENCHMARK:
            continue
        if re.search(
            r"->\s*Args\s*\(\s*\{\s*"
            rf"{world_count}\s*,\s*{body_count}\s*,\s*{step_count}"
            r"\s*\}\s*\)",
            match.group("chain"),
            re.DOTALL,
        ):
            return True
    return False


def find_violations_in_text(rel_path: str, text: str) -> list[Violation]:
    path = Path(rel_path)
    if not _is_cuda_benchmark_source(path, text):
        return []

    masked = _mask_comments_and_literals(text)
    violations: list[Violation] = []
    if not _has_required_benchmark_name(masked):
        violations.append(
            Violation(
                rel_path,
                f"CUDA benchmark files must register {REQUIRED_GPU_BENCHMARK}",
            )
        )
    if not _has_required_workload(masked):
        workload = "/".join(str(value) for value in REQUIRED_WORKLOAD)
        violations.append(
            Violation(
                rel_path,
                f"{REQUIRED_GPU_BENCHMARK} must include the {workload} "
                "manual go/no-go workload",
            )
        )
    return violations


def iter_benchmark_sources(root: Path = BENCHMARK_ROOT) -> list[Path]:
    if not root.exists():
        return []
    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.suffix in SOURCE_SUFFIXES
    )


def find_phase5_cuda_benchmark_contract_violations() -> list[Violation]:
    violations: list[Violation] = []
    for path in iter_benchmark_sources():
        rel_path = path.relative_to(REPO_ROOT).as_posix()
        violations.extend(find_violations_in_text(rel_path, path.read_text()))
    return violations


def run_self_tests() -> None:
    if find_violations_in_text("bm_compute_graph.cpp", "BENCHMARK(BM_WorldStep);"):
        raise AssertionError("non-CUDA benchmark self-test failed")

    bad_source = """
    void BM_CudaRigidBodyStateBatchLinear(benchmark::State& state) {}
    BENCHMARK(BM_CudaRigidBodyStateBatchLinear)->Arg(4096);
    """
    if (
        len(find_violations_in_text("bm_cuda_rigid_body_state_batch.cpp", bad_source))
        != 2
    ):
        raise AssertionError("incompatible CUDA benchmark self-test failed")

    commented_source = """
    // BENCHMARK(BM_Phase5RigidBodyBatchGpu)->Args({4096, 128, 100});
    void BM_CudaRigidBodyStateBatchLinear(benchmark::State& state) {}
    """
    if (
        len(
            find_violations_in_text(
                "bm_cuda_rigid_body_state_batch.cpp", commented_source
            )
        )
        != 2
    ):
        raise AssertionError("comment masking self-test failed")

    good_source = """
    void BM_Phase5RigidBodyBatchGpu(benchmark::State& state) {}
    BENCHMARK(BM_Phase5RigidBodyBatchGpu)
      ->Args({1024, 128, 10})
      ->Args({4096, 128, 100});
    """
    if find_violations_in_text("bm_cuda_rigid_body_state_batch.cpp", good_source):
        raise AssertionError("valid CUDA benchmark self-test failed")

    mismatched_source = """
    void BM_Phase5RigidBodyBatchGpu(benchmark::State& state) {}
    void BM_CudaOther(benchmark::State& state) {}
    BENCHMARK(BM_Phase5RigidBodyBatchGpu)->Args({1024, 128, 10});
    BENCHMARK(BM_CudaOther)->Args({4096, 128, 100});
    """
    violations = find_violations_in_text(
        "bm_cuda_rigid_body_state_batch.cpp", mismatched_source
    )
    if len(violations) != 1 or "4096/128/100" not in violations[0].message:
        raise AssertionError("mismatched CUDA workload self-test failed")


def main() -> int:
    run_self_tests()
    violations = find_phase5_cuda_benchmark_contract_violations()
    if not violations:
        print("Phase 5 CUDA benchmark contract check passed.")
        return 0

    print("Phase 5 CUDA benchmark contract check failed:", file=sys.stderr)
    for violation in violations:
        print(f"{violation.path}: {violation.message}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
