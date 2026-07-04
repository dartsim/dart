#!/usr/bin/env python3
"""Check Linux CI keeps differentiable build-option coverage explicit.

PLAN-110 keeps differentiable simulation behind ``DART_BUILD_DIFF`` so the
default build stays zero-overhead. CI therefore needs both sides of the option:
the normal Release job remains the default-OFF coverage path, and a focused
Release job configures ``DART_BUILD_DIFF=ON`` before building and running the
diff C++ and Python binding tests.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci_ubuntu.yml"

DIFF_JOB_ID = "build-diff"
DEFAULT_OFF_JOB_ID = "build-release"
DIFF_ENV = "DART_BUILD_DIFF_OVERRIDE: ON"
DIFF_CONFIG = "pixi run config ON Release"
DIFF_CTEST = "-R '^test_diff_'"
DIFF_PYTEST = "pytest -q python/tests/unit/simulation/test_diff.py"
DIFF_PYTHONPATH = "PYTHONPATH=build/default/cpp/Release/python"
DEFAULT_RELEASE_TEST = "pixi run test ON Release"
DEFAULT_RELEASE_PYTEST = "pixi run test-py ON Release"
DEFAULT_OFF_PARITY_STEP = "Run default differentiable OFF parity test"
DEFAULT_OFF_PARITY_TARGET = "test_diff_zero_cost_parity"
DEFAULT_OFF_PARITY_CTEST = "-R '^test_diff_zero_cost_parity$'"

DIFF_TARGETS = (
    "dartpy",
    "test_diff_zero_cost_parity",
    "test_diff_smooth_jacobian",
    "test_diff_contact_jacobian",
    "test_diff_public_contact_jacobian",
    "test_diff_contact_gradient_modes",
    "test_diff_apply_step_vjp",
    "test_diff_parameter_jacobian",
    "test_diff_rollout",
    "test_diff_optimization",
    "test_diff_paper_experiments",
    "test_diff_dojo_style_ipm_spike",
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
        help="Linux CI workflow YAML file to check.",
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


def _contains(text: str, expected: str) -> bool:
    return expected in " ".join(text.split())


def find_violations(workflow_path: Path = DEFAULT_WORKFLOW) -> list[Violation]:
    try:
        workflow_text = workflow_path.read_text(encoding="utf-8")
    except OSError as exc:
        return [Violation(str(exc))]

    violations: list[Violation] = []

    release_job = _find_job_block(workflow_text, DEFAULT_OFF_JOB_ID)
    if not release_job:
        violations.append(Violation("Linux CI must define the build-release job"))
    else:
        if DIFF_ENV in release_job or "DART_BUILD_DIFF_OVERRIDE=ON" in release_job:
            violations.append(
                Violation(
                    "build-release must remain the default DART_BUILD_DIFF=OFF path"
                )
            )
        release_cpp = _find_step_block(release_job, "Run Release C++ tests")
        if not _contains(release_cpp, DEFAULT_RELEASE_TEST):
            violations.append(
                Violation("build-release must run the default Release C++ test path")
            )
        release_parity = _find_step_block(release_job, DEFAULT_OFF_PARITY_STEP)
        if DEFAULT_OFF_PARITY_TARGET not in release_parity:
            violations.append(
                Violation(
                    "build-release must build the default-OFF "
                    f"{DEFAULT_OFF_PARITY_TARGET} target"
                )
            )
        if DEFAULT_OFF_PARITY_CTEST not in release_parity:
            violations.append(
                Violation(
                    "build-release must run the default-OFF zero-cost parity test"
                )
            )
        release_py = _find_step_block(release_job, "Run Release Python tests")
        if not _contains(release_py, DEFAULT_RELEASE_PYTEST):
            violations.append(
                Violation("build-release must run the default Release Python test path")
            )

    diff_job = _find_job_block(workflow_text, DIFF_JOB_ID)
    if not diff_job:
        violations.append(Violation("Linux CI must define a build-diff job"))
        return violations

    for required in (
        "needs.changes.outputs.full_ci == 'true'",
        "github.event_name == 'schedule'",
        "github.event_name == 'workflow_dispatch'",
        "needs.changes.outputs.code == 'true'",
    ):
        if required not in diff_job:
            violations.append(
                Violation(f"build-diff must preserve the full-CI condition: {required}")
            )

    if DIFF_ENV not in diff_job:
        violations.append(Violation("build-diff must set DART_BUILD_DIFF_OVERRIDE: ON"))

    configure_step = _find_step_block(diff_job, "Configure differentiable build")
    if not _contains(configure_step, DIFF_CONFIG):
        violations.append(
            Violation(f"build-diff must configure the diff build with: {DIFF_CONFIG}")
        )

    build_step = _find_step_block(diff_job, "Build differentiable targets")
    if "cmake --build build/default/cpp/Release" not in build_step:
        violations.append(Violation("build-diff must build the Release tree"))
    for target in DIFF_TARGETS:
        if target not in build_step:
            violations.append(
                Violation(f"build-diff must build differentiable target {target}")
            )

    ctest_step = _find_step_block(diff_job, "Run differentiable C++ tests")
    if "ctest --test-dir build/default/cpp/Release" not in ctest_step:
        violations.append(Violation("build-diff must run CTest on the Release tree"))
    if DIFF_CTEST not in ctest_step:
        violations.append(
            Violation(f"build-diff must run the diff C++ tests with {DIFF_CTEST}")
        )

    pytest_step = _find_step_block(diff_job, "Run differentiable Python tests")
    if DIFF_PYTHONPATH not in pytest_step:
        violations.append(
            Violation("build-diff must run Python tests against the built dartpy")
        )
    if not _contains(pytest_step, DIFF_PYTEST):
        violations.append(
            Violation(f"build-diff must run the diff Python test: {DIFF_PYTEST}")
        )

    return violations


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    violations = find_violations(args.workflow)
    if not violations:
        print("Differentiable CI workflow check passed.")
        return 0

    print("Differentiable CI workflow check failed:", file=sys.stderr)
    for violation in violations:
        print(f"- {violation.message}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
