#!/usr/bin/env python3
"""Check that the manual CUDA workflow produces Phase 5 packet evidence."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

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


def _load_workflow(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must be a YAML mapping")
    return data


def _trigger_mapping(workflow: dict) -> dict:
    # PyYAML follows YAML 1.1 and may parse the key `on` as boolean True.
    triggers = workflow.get("on", workflow.get(True, {}))
    return triggers if isinstance(triggers, dict) else {}


def _job(workflow: dict) -> dict:
    jobs = workflow.get("jobs", {})
    if not isinstance(jobs, dict):
        return {}
    cuda = jobs.get("cuda", {})
    return cuda if isinstance(cuda, dict) else {}


def _steps(job: dict) -> list[dict]:
    steps = job.get("steps", [])
    return (
        [step for step in steps if isinstance(step, dict)]
        if isinstance(steps, list)
        else []
    )


def _step_named(steps: list[dict], name: str) -> dict | None:
    for step in steps:
        if step.get("name") == name:
            return step
    return None


def _run_text(step: dict | None) -> str:
    if step is None:
        return ""
    run = step.get("run", "")
    return run if isinstance(run, str) else ""


def _path_text(step: dict | None) -> str:
    if step is None:
        return ""
    with_block = step.get("with", {})
    if not isinstance(with_block, dict):
        return ""
    path = with_block.get("path", "")
    return path if isinstance(path, str) else ""


def _contains_all(text: str, needles: tuple[str, ...]) -> list[str]:
    return [needle for needle in needles if needle not in text]


def find_violations(workflow_path: Path = DEFAULT_WORKFLOW) -> list[Violation]:
    try:
        workflow = _load_workflow(workflow_path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return [Violation(str(exc))]

    violations: list[Violation] = []

    workflow_text = workflow_path.read_text(encoding="utf-8")
    if "workflow_dispatch:" not in workflow_text:
        violations.append(Violation("CUDA workflow must be manually dispatched"))
    for forbidden in ("pull_request:", "push:"):
        if forbidden in workflow_text:
            violations.append(
                Violation(f"CUDA workflow must not trigger on {forbidden}")
            )

    triggers = _trigger_mapping(workflow)
    if "workflow_dispatch" not in triggers:
        violations.append(
            Violation("CUDA workflow trigger must include workflow_dispatch")
        )

    cuda_job = _job(workflow)
    runner_labels = cuda_job.get("runs-on", [])
    if not isinstance(runner_labels, list) or any(
        label not in runner_labels for label in REQUIRED_RUNNER_LABELS
    ):
        labels = ", ".join(REQUIRED_RUNNER_LABELS)
        violations.append(Violation(f"CUDA job must run on [{labels}]"))

    steps = _steps(cuda_job)

    smoke_run = _run_text(_step_named(steps, "Run CUDA smoke tests"))
    if "pixi run --locked -e cuda test-cuda" not in smoke_run:
        violations.append(Violation("CUDA workflow must run the test-cuda gate"))

    policy_run = _run_text(_step_named(steps, "Run Phase 5 policy gates"))
    for gate in _contains_all(policy_run, REQUIRED_POLICY_GATES):
        violations.append(Violation(f"CUDA workflow is missing policy gate: {gate}"))

    full_run = _run_text(_step_named(steps, "Run Phase 5 CUDA go/no-go benchmark"))
    if "pixi run --locked -e cuda bm-phase5-cuda-full" not in full_run:
        violations.append(
            Violation("CUDA workflow must run bm-phase5-cuda-full for the full row")
        )

    packet_run = _run_text(_step_named(steps, "Write Phase 5 CUDA packet"))
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

    upload_step = _step_named(steps, "Upload Phase 5 CUDA packet")
    if upload_step is None:
        violations.append(
            Violation("CUDA workflow must upload Phase 5 packet artifacts")
        )
    elif not str(upload_step.get("uses", "")).startswith("actions/upload-artifact@"):
        violations.append(
            Violation("CUDA workflow packet upload must use upload-artifact")
        )
    artifact_paths = _path_text(upload_step)
    for path in _contains_all(artifact_paths, REQUIRED_ARTIFACT_PATHS):
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
