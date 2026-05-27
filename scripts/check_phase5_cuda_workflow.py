#!/usr/bin/env python3
"""Check that the manual CUDA workflow produces Phase 5 packet evidence."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci_cuda.yml"

REQUIRED_RUNNER_LABELS = ("self-hosted", "dartsim", "cuda")
REQUIRED_POLICY_GATES = (
    "pixi run --locked -e cuda check-compute-backend-boundaries",
    "pixi run --locked -e cuda check-no-gpu-runtime-dependencies",
    "pixi run --locked -e cuda check-phase5-cuda-benchmark-contract",
)
REQUIRED_PACKET_FLAGS = (
    "--includes-transfer-setup-compute-readback",
    "--gpu-build-import-gate-passed",
    "--compute-backend-boundaries-passed",
    "--no-gpu-runtime-dependencies-passed",
    "--phase5-benchmark-contract-passed",
)
REQUIRED_ARTIFACT_PATHS = (
    ".benchmark_results/phase5_cuda_ci_full.json",
    ".benchmark_results/phase5_cuda_packet.json",
)


@dataclass(frozen=True)
class Violation:
    message: str


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workflow",
        type=Path,
        default=DEFAULT_WORKFLOW,
        help="CUDA workflow YAML file to check.",
    )
    return parser.parse_args(argv)


def _contains_all(text: str, needles: tuple[str, ...]) -> list[str]:
    return [needle for needle in needles if needle not in text]


def _find_step_block(text: str, name: str) -> str:
    pattern = re.compile(
        rf"(?ms)^ {{6}}- name: {re.escape(name)}\n" r".*?(?=^ {6}- name: |\Z)"
    )
    match = pattern.search(text)
    return match.group(0) if match else ""


def _find_job_block(text: str, job_id: str) -> str:
    pattern = re.compile(
        rf"(?ms)^  {re.escape(job_id)}:\n" r".*?(?=^  [A-Za-z0-9_-]+:\n|\Z)"
    )
    match = pattern.search(text)
    return match.group(0) if match else ""


def find_violations(workflow_path: Path = DEFAULT_WORKFLOW) -> list[Violation]:
    try:
        workflow_text = workflow_path.read_text(encoding="utf-8")
    except OSError as exc:
        return [Violation(str(exc))]

    violations: list[Violation] = []

    if "workflow_dispatch:" not in workflow_text:
        violations.append(Violation("CUDA workflow must be manually dispatched"))

    runtime_job = _find_job_block(workflow_text, "cuda-runtime")
    runtime_text = runtime_job or workflow_text
    if (
        runtime_job
        and "if: github.event_name == 'workflow_dispatch'" not in runtime_job
    ):
        violations.append(
            Violation("CUDA runtime job must only run on workflow_dispatch")
        )

    runner_match = re.search(
        r"(?m)^ {4}runs-on:\s*\[(?P<labels>[^\]]+)\]", runtime_text
    )
    runner_labels = set()
    if runner_match:
        runner_labels = {
            label.strip() for label in runner_match.group("labels").split(",")
        }
    if not runner_match or any(
        label not in runner_labels for label in REQUIRED_RUNNER_LABELS
    ):
        labels = ", ".join(REQUIRED_RUNNER_LABELS)
        violations.append(Violation(f"CUDA job must run on [{labels}]"))

    smoke_run = _find_step_block(runtime_text, "Run CUDA smoke tests")
    if "pixi run --locked -e cuda test-cuda" not in smoke_run:
        violations.append(Violation("CUDA workflow must run the test-cuda gate"))

    policy_run = _find_step_block(runtime_text, "Run Phase 5 policy gates")
    for gate in _contains_all(policy_run, REQUIRED_POLICY_GATES):
        violations.append(Violation(f"CUDA workflow is missing policy gate: {gate}"))

    full_run = _find_step_block(runtime_text, "Run Phase 5 CUDA go/no-go benchmark")
    if "pixi run --locked -e cuda bm-phase5-cuda-full" not in full_run:
        violations.append(
            Violation("CUDA workflow must run bm-phase5-cuda-full for the full row")
        )

    packet_run = _find_step_block(runtime_text, "Write Phase 5 CUDA packet")
    packet_requirements = (
        "pixi run --locked -e cuda bm-phase5-cuda-packet",
        "--benchmark-json .benchmark_results/phase5_cuda_ci_full.json",
        "--output .benchmark_results/phase5_cuda_packet.json",
        "pixi run --locked -e cuda bm-phase5-gpu-packet-check",
        "--input .benchmark_results/phase5_cuda_packet.json",
        *REQUIRED_PACKET_FLAGS,
    )
    for requirement in _contains_all(packet_run, packet_requirements):
        violations.append(
            Violation(f"CUDA workflow packet step is missing: {requirement}")
        )

    upload_step = _find_step_block(runtime_text, "Upload Phase 5 CUDA packet")
    if not upload_step:
        violations.append(
            Violation("CUDA workflow must upload Phase 5 packet artifacts")
        )
    elif "uses: actions/upload-artifact@" not in upload_step:
        violations.append(
            Violation("CUDA workflow packet upload must use upload-artifact")
        )
    if upload_step and "if: always()" not in upload_step:
        violations.append(Violation("CUDA workflow packet upload must run always"))
    for path in _contains_all(upload_step, REQUIRED_ARTIFACT_PATHS):
        violations.append(
            Violation(f"CUDA workflow artifact upload is missing: {path}")
        )

    return violations


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    violations = find_violations(args.workflow)
    if not violations:
        print("Phase 5 CUDA workflow check passed.")
        return 0

    print("Phase 5 CUDA workflow check failed:", file=sys.stderr)
    for violation in violations:
        print(f"- {violation.message}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
