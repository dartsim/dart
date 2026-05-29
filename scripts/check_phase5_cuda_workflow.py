#!/usr/bin/env python3
"""Check the CUDA CI workflow keeps the build/import gate off self-hosted runners.

The project does not maintain a self-hosted GPU runner. Phase 5's GPU go/no-go
evidence is produced manually on a CUDA host and recorded in
``docs/design/scalable_compute_decisions.md``; CI only compiles the CUDA targets
on a GitHub-hosted runner as the build/import gate. This checker guards that
contract: the ``cuda-build`` build/import gate stays wired on a GitHub-hosted
ubuntu runner, and no job may reintroduce a self-hosted GPU runner.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci_cuda.yml"

BUILD_IMPORT_GATE = "pixi run --locked -e cuda build-cuda"


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

    # Policy: no job may run on a self-hosted runner. The project does not
    # maintain a self-hosted GPU runner; Phase 5 GPU go/no-go evidence is
    # produced manually on a CUDA host instead.
    if re.search(r"(?mi)^\s*runs-on:.*self-hosted", workflow_text):
        violations.append(
            Violation(
                "CUDA workflow must not use a self-hosted runner; GPU go/no-go "
                "evidence is produced manually on a CUDA host"
            )
        )

    # The build/import gate must stay wired on a GitHub-hosted ubuntu runner.
    build_job = _find_job_block(workflow_text, "cuda-build")
    if not build_job:
        violations.append(Violation("CUDA workflow must define a cuda-build job"))
        return violations

    runner_match = re.search(r"(?m)^ {4}runs-on:\s*(?P<runner>\S+)", build_job)
    runner = runner_match.group("runner") if runner_match else ""
    if not runner.startswith("ubuntu-"):
        violations.append(
            Violation("cuda-build must run on a GitHub-hosted ubuntu runner")
        )

    if BUILD_IMPORT_GATE not in build_job:
        violations.append(
            Violation(f"cuda-build must run the build/import gate: {BUILD_IMPORT_GATE}")
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
