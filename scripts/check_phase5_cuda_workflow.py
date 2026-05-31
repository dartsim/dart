#!/usr/bin/env python3
"""Check the CUDA CI workflow runs on the guarded self-hosted GPU runner.

The project maintains a self-hosted GPU runner (label ``ubuntu-latest-gpu``) so
the CUDA targets are built AND executed against a real NVIDIA GPU, rather than
only compiled as a build/import gate on a GitHub-hosted runner. Because that
runner is long-lived self-hosted hardware on a public repository, the
``cuda-build`` job MUST be guarded so untrusted fork-PR code never executes on
it: it may run only for ``push``/``workflow_dispatch`` or pull requests whose
head is a branch in this same repository.

This checker guards that contract: ``cuda-build`` stays wired to the
``ubuntu-latest-gpu`` runner, keeps the fork-PR guard, runs the CUDA build gate,
and sets up the ``cuda`` pixi environment. (Historical note: CUDA CI previously
stayed on a GitHub-hosted runner because no self-hosted GPU runner existed; that
runner now exists, so the policy is to use it — guarded.)
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
# The CUDA build gate (builds the CUDA targets in the cuda pixi environment).
BUILD_IMPORT_GATE = "pixi run --locked -e cuda build-cuda"
# Security guard: the long-lived self-hosted GPU runner must never run untrusted
# fork-PR code, so cuda-build must restrict pull_request events to same-repo PRs.
FORK_PR_GUARD = "github.event.pull_request.head.repo.full_name == github.repository"


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

    # The CUDA build/test job must run on the self-hosted GPU runner so CUDA
    # targets execute against a real GPU.
    runner_match = re.search(r"(?m)^ {4}runs-on:\s*(?P<runner>\S+)", build_job)
    runner = runner_match.group("runner") if runner_match else ""
    if runner != EXPECTED_RUNNER:
        violations.append(
            Violation(
                f"cuda-build must run on the self-hosted GPU runner "
                f"'{EXPECTED_RUNNER}' (found '{runner or 'none'}')"
            )
        )

    # Security: untrusted fork-PR code must never run on the self-hosted GPU
    # runner. cuda-build must carry the same-repo / non-pull_request guard.
    if FORK_PR_GUARD not in build_job:
        violations.append(
            Violation(
                "cuda-build must guard the self-hosted GPU runner against fork "
                "PRs (an `if:` that restricts pull_request to same-repo PRs, "
                f"e.g. `{FORK_PR_GUARD}`)"
            )
        )

    if BUILD_IMPORT_GATE not in build_job:
        violations.append(
            Violation(f"cuda-build must run the CUDA build gate: {BUILD_IMPORT_GATE}")
        )

    if "environments: cuda" not in build_job:
        violations.append(Violation("cuda-build must set up the cuda pixi environment"))

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
