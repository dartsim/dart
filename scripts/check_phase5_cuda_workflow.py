#!/usr/bin/env python3
"""Check the CUDA CI workflow keeps CUDA validation guarded and required.

The project maintains a self-hosted GPU runner (label ``ubuntu-latest-gpu``) so
the CUDA targets are built AND executed against a real NVIDIA GPU, rather than
only compiled as a build/import gate on a GitHub-hosted runner. Because that
runner is long-lived self-hosted hardware on a public repository, untrusted fork
PR code must never execute on it. Fork PRs still need CUDA validation, so the
same required ``cuda-build`` job falls back to GitHub-hosted ``ubuntu-latest`` to
compile the CUDA targets without running GPU-only steps.

This checker guards that contract: ``cuda-build`` stays wired to the
``ubuntu-latest-gpu`` runner for trusted events, routes fork PRs to the hosted
fallback without a job-level skip, runs the CUDA build gate for both paths, and
guards GPU-only test steps to trusted events. (Historical note: CUDA CI
previously stayed on a GitHub-hosted runner because no self-hosted GPU runner
existed; that runner now exists, so the policy is to use it — guarded.)
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci_cuda.yml"

# The self-hosted GPU runner label the cuda-build job must target.
EXPECTED_RUNNER = "ubuntu-latest-gpu"
EXPECTED_RUNNER_TOKEN = f"'{EXPECTED_RUNNER}'"
# The GitHub-hosted fallback runner fork PRs must use for CUDA compilation.
HOSTED_FORK_RUNNER = "ubuntu-latest"
HOSTED_FORK_RUNNER_TOKEN = f"'{HOSTED_FORK_RUNNER}'"
# The CUDA build gate (builds the CUDA targets in the cuda pixi environment).
BUILD_IMPORT_GATE = "pixi run --locked -e cuda build-cuda"
# Security guard: GPU-only steps may only run outside pull_request or for
# same-repo PRs. Fork PRs must route to HOSTED_FORK_RUNNER instead.
PULL_REQUEST_EVENT_GUARD = "github.event_name == 'pull_request'"
FORK_PR_FALLBACK = "github.event.pull_request.head.repo.full_name != github.repository"
NON_PR_EVENT_GUARD = "github.event_name != 'pull_request'"
SAME_REPO_PR_GUARD = (
    "github.event.pull_request.head.repo.full_name == github.repository"
)
FORK_PR_RUNNER_EXPRESSION = (
    f"{PULL_REQUEST_EVENT_GUARD} && {FORK_PR_FALLBACK} && "
    f"{HOSTED_FORK_RUNNER_TOKEN} || {EXPECTED_RUNNER_TOKEN}"
)
TRUSTED_GPU_GUARD = f"{NON_PR_EVENT_GUARD} || {SAME_REPO_PR_GUARD}"


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


def _find_job_block(text: str, job_id: str) -> str:
    pattern = re.compile(
        rf"(?ms)^  {re.escape(job_id)}:\n" r".*?(?=^  [A-Za-z0-9_-]+:\n|\Z)"
    )
    match = pattern.search(text)
    return match.group(0) if match else ""


def _find_step_block(job_text: str, step_name: str) -> str:
    pattern = re.compile(
        rf"(?ms)^      - name:\s*{re.escape(step_name)}\n"
        r".*?(?=^      - name:\s*|\Z)"
    )
    match = pattern.search(job_text)
    return match.group(0) if match else ""


def _has_step_if(step_text: str) -> bool:
    return bool(re.search(r"(?m)^ {8}if:\s*", step_text))


def _contains_expression(text: str, expression: str) -> bool:
    return expression in " ".join(text.split())


def find_violations(workflow_path: Path = DEFAULT_WORKFLOW) -> list[Violation]:
    try:
        workflow_text = workflow_path.read_text(encoding="utf-8")
    except OSError as exc:
        return [Violation(str(exc))]

    violations: list[Violation] = []

    build_job = _find_job_block(workflow_text, "cuda-build")
    if not build_job:
        violations.append(Violation("CUDA workflow must define a cuda-build job"))
        return violations

    # The job itself must not be skipped for fork PRs; otherwise a required check
    # can report Success without any CUDA validation. Route forks to the hosted
    # fallback through runs-on instead.
    if re.search(r"(?m)^ {4}if:\s*", build_job):
        violations.append(
            Violation(
                "cuda-build must not use a job-level `if:`; fork PRs must run "
                "the hosted CUDA compile fallback in the required check"
            )
        )

    # Trusted events need the self-hosted GPU runner; fork PRs need the hosted
    # fallback runner. Validate the actual routing expression so the fork and
    # trusted branches cannot be accidentally reversed while still mentioning both
    # labels.
    runner_match = re.search(
        r"(?ms)^ {4}runs-on:\s*(?P<runner>.*?)(?=^ {4}\S|\Z)", build_job
    )
    runner = runner_match.group("runner") if runner_match else ""
    if EXPECTED_RUNNER_TOKEN not in runner:
        violations.append(
            Violation(
                f"cuda-build must route trusted events to the self-hosted GPU "
                f"runner '{EXPECTED_RUNNER}'"
            )
        )

    if not _contains_expression(runner, FORK_PR_RUNNER_EXPRESSION):
        violations.append(
            Violation(
                "cuda-build runs-on must route fork PRs to the GitHub-hosted "
                f"'{HOSTED_FORK_RUNNER}' CUDA compile fallback before "
                f"defaulting trusted events to '{EXPECTED_RUNNER}'"
            )
        )

    if BUILD_IMPORT_GATE not in build_job:
        violations.append(
            Violation(f"cuda-build must run the CUDA build gate: {BUILD_IMPORT_GATE}")
        )
    else:
        build_step = _find_step_block(build_job, "Build CUDA targets")
        if not build_step:
            violations.append(
                Violation("cuda-build must define a Build CUDA targets step")
            )
        elif _has_step_if(build_step):
            violations.append(
                Violation(
                    "Build CUDA targets must run for both trusted events and "
                    "fork PR hosted fallback"
                )
            )

    if "environments: cuda" not in build_job:
        violations.append(Violation("cuda-build must set up the cuda pixi environment"))

    gpu_test_step = _find_step_block(build_job, "Run CUDA tests on GPU")
    if not gpu_test_step or "pixi run --locked -e cuda test-cuda" not in gpu_test_step:
        violations.append(Violation("cuda-build must run CUDA tests on the GPU path"))
    elif not _contains_expression(gpu_test_step, TRUSTED_GPU_GUARD):
        violations.append(
            Violation(
                "Run CUDA tests on GPU must be guarded to trusted non-PR and "
                "same-repo PR events"
            )
        )

    gpu_probe_step = _find_step_block(build_job, "Show GPU (nvidia-smi)")
    if not gpu_probe_step or "nvidia-smi" not in gpu_probe_step:
        violations.append(Violation("cuda-build must probe the GPU on the GPU path"))
    elif not _contains_expression(gpu_probe_step, TRUSTED_GPU_GUARD):
        violations.append(
            Violation(
                "Show GPU (nvidia-smi) must be guarded to trusted non-PR and "
                "same-repo PR events"
            )
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
