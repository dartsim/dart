#!/usr/bin/env python3
"""Verify the performance dashboard workflow is registered in GitHub Actions."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

import yaml

REQUIRED_PUSH_PATHS = {
    ".github/actions/**",
    ".github/workflows/performance_dashboard.yml",
    "CMakeLists.txt",
    "cmake/**",
    "dart/**",
    "data/**",
    "pixi.lock",
    "pixi.toml",
    "scripts/*performance_dashboard*.py",
    "scripts/check_collision_benchmarks.py",
    "scripts/report_performance_to_bencher.py",
    "scripts/run_cpp_benchmark.py",
    "scripts/summarize_performance_dashboard.py",
    "scripts/verify_performance_dashboard.py",
    "tests/benchmark/**",
}

REQUIRED_SCHEDULE_CRONS = {"30 3 * * 0,3"}

FLOATING_ACTION_REFS = {"HEAD", "main", "master"}


def _default_token() -> str | None:
    token = os.environ.get("GITHUB_TOKEN")
    if token:
        return token
    try:
        result = subprocess.run(
            ["gh", "auth", "token"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip() or None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo",
        default=os.environ.get("GITHUB_REPOSITORY", "dartsim/dart"),
        help="GitHub repository in owner/name form.",
    )
    parser.add_argument(
        "--api-url",
        default=os.environ.get("GITHUB_API_URL", "https://api.github.com"),
        help="GitHub API URL.",
    )
    parser.add_argument(
        "--token",
        default=_default_token(),
        help="GitHub token. Defaults to GITHUB_TOKEN or gh auth token.",
    )
    parser.add_argument(
        "--workflow-name",
        default="Performance Dashboard",
        help="Expected GitHub Actions workflow name.",
    )
    parser.add_argument(
        "--workflow-path",
        default=".github/workflows/performance_dashboard.yml",
        help="Expected workflow path in the repository.",
    )
    parser.add_argument(
        "--local-workflow",
        type=Path,
        default=Path(".github/workflows/performance_dashboard.yml"),
        help="Local workflow YAML to validate before checking GitHub.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("."),
        help="Repository root used to resolve local composite actions.",
    )
    parser.add_argument(
        "--skip-action-ref-existence",
        action="store_true",
        help="Skip network checks that external GitHub Action tags exist.",
    )
    parser.add_argument(
        "--workflows-json",
        type=Path,
        default=None,
        help="Local workflows API JSON fixture. Used by tests.",
    )
    parser.add_argument(
        "--allow-missing",
        action="store_true",
        help="Exit successfully when the workflow is not registered yet.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Per-request timeout in seconds.",
    )
    return parser.parse_args()


def _headers(token: str | None) -> dict[str, str]:
    headers = {
        "Accept": "application/vnd.github+json",
        "User-Agent": "dart-performance-dashboard",
    }
    if token:
        headers["Authorization"] = f"Bearer {token}"
    return headers


def _read_json_url(url: str, token: str | None, timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(url, headers=_headers(token))
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            data = json.loads(response.read().decode("utf-8"))
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Failed to read {url}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected JSON object from {url}.")
    return data


def _read_url_status(url: str, token: str | None, timeout: float) -> int:
    request = urllib.request.Request(url, headers=_headers(token))
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return int(response.status)
    except urllib.error.HTTPError as exc:
        return int(exc.code)
    except (OSError, urllib.error.URLError) as exc:
        raise RuntimeError(f"Failed to read {url}: {exc}") from exc


def _read_workflows(args: argparse.Namespace) -> list[dict[str, Any]]:
    if args.workflows_json:
        data = json.loads(args.workflows_json.read_text(encoding="utf-8"))
    else:
        repo = urllib.parse.quote(args.repo, safe="/")
        data = _read_json_url(
            f"{args.api_url.rstrip('/')}/repos/{repo}/actions/workflows?per_page=100",
            args.token,
            args.timeout,
        )
    workflows = data.get("workflows", [])
    if not isinstance(workflows, list):
        raise RuntimeError("Workflows API response did not contain a list.")
    return [workflow for workflow in workflows if isinstance(workflow, dict)]


def _read_local_workflow(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"Failed to read local workflow {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise RuntimeError(f"Invalid workflow YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected workflow YAML object in {path}.")
    return data


def _triggers(workflow: dict[str, Any]) -> Any:
    # PyYAML still applies YAML 1.1 booleans, so an unquoted "on" key can be True.
    return workflow.get("on", workflow.get(True))


def _trigger_names(workflow: dict[str, Any]) -> set[str]:
    triggers = _triggers(workflow)
    if isinstance(triggers, str):
        return {triggers}
    if isinstance(triggers, list):
        return {item for item in triggers if isinstance(item, str)}
    if isinstance(triggers, dict):
        return {str(key) for key in triggers}
    return set()


def _schedule_crons(workflow: dict[str, Any]) -> set[str]:
    triggers = _triggers(workflow)
    if not isinstance(triggers, dict):
        return set()
    schedules = triggers.get("schedule")
    if not isinstance(schedules, list):
        return set()
    return {
        str(item.get("cron"))
        for item in schedules
        if isinstance(item, dict) and item.get("cron")
    }


def _string_items(value: Any) -> set[str]:
    if isinstance(value, str):
        return {value}
    if isinstance(value, list):
        return {item for item in value if isinstance(item, str)}
    return set()


def _has_main_push_trigger(workflow: dict[str, Any]) -> bool:
    triggers = _triggers(workflow)
    if not isinstance(triggers, dict):
        return False
    push = triggers.get("push")
    if push is None:
        return False
    if not isinstance(push, dict):
        return True
    branches = _string_items(push.get("branches"))
    return "main" in branches


def _push_paths(workflow: dict[str, Any]) -> set[str]:
    triggers = _triggers(workflow)
    if not isinstance(triggers, dict):
        return set()
    push = triggers.get("push")
    if not isinstance(push, dict):
        return set()
    return _string_items(push.get("paths"))


def _permission_level(value: Any) -> int:
    return {
        "none": 0,
        "read": 1,
        "write": 2,
    }.get(str(value), -1)


def _has_permission(workflow: dict[str, Any], permission: str, minimum: str) -> bool:
    permissions = workflow.get("permissions")
    if permissions == "write-all":
        return True
    if permissions == "read-all":
        return _permission_level(minimum) <= _permission_level("read")
    if not isinstance(permissions, dict):
        return False
    return _permission_level(permissions.get(permission)) >= _permission_level(minimum)


def _missing_publish_permissions(workflow: dict[str, Any]) -> list[str]:
    required = {
        "actions": "read",
        "contents": "write",
        "pages": "write",
    }
    return [
        f"{permission}:{minimum}"
        for permission, minimum in required.items()
        if not _has_permission(workflow, permission, minimum)
    ]


def _uses_refs_from_steps(steps: Any) -> list[str]:
    if not isinstance(steps, list):
        return []

    uses_refs: list[str] = []
    for step in steps:
        if not isinstance(step, dict):
            continue
        step_uses = step.get("uses")
        if isinstance(step_uses, str):
            uses_refs.append(step_uses)
    return uses_refs


def _local_action_path(repo_root: Path, uses_ref: str) -> Path | None:
    if not uses_ref.startswith("./"):
        return None
    action_path = uses_ref[2:]
    if "@" in action_path:
        action_path = action_path.rsplit("@", 1)[0]
    return repo_root / action_path


def _local_action_file(action_path: Path) -> Path | None:
    if action_path.is_file():
        return action_path
    for filename in ("action.yml", "action.yaml"):
        candidate = action_path / filename
        if candidate.is_file():
            return candidate
    return None


def _read_local_action(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"Failed to read local action {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise RuntimeError(f"Invalid local action YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected local action YAML object in {path}.")
    return data


def _collect_local_action_uses(
    uses_ref: str, repo_root: Path, seen: set[Path]
) -> list[str]:
    local_path = _local_action_path(repo_root, uses_ref)
    if local_path is None:
        return []
    action_file = _local_action_file(local_path)
    if action_file is None:
        raise RuntimeError(
            f"Local performance dashboard workflow references missing local "
            f"action {uses_ref!r}."
        )
    resolved = action_file.resolve()
    if resolved in seen:
        return []
    seen.add(resolved)

    action = _read_local_action(action_file)
    runs = action.get("runs")
    steps = runs.get("steps") if isinstance(runs, dict) else []
    nested_refs = _uses_refs_from_steps(steps)
    collected = list(nested_refs)
    for nested_ref in nested_refs:
        collected.extend(_collect_local_action_uses(nested_ref, repo_root, seen))
    return collected


def _workflow_uses_refs(workflow: dict[str, Any], repo_root: Path) -> list[str]:
    jobs = workflow.get("jobs")
    if not isinstance(jobs, dict):
        return []

    uses_refs: list[str] = []
    seen_local_actions: set[Path] = set()
    for job in jobs.values():
        if not isinstance(job, dict):
            continue
        job_uses = job.get("uses")
        if isinstance(job_uses, str):
            uses_refs.append(job_uses)
            uses_refs.extend(
                _collect_local_action_uses(job_uses, repo_root, seen_local_actions)
            )
        steps = job.get("steps")
        step_refs = _uses_refs_from_steps(steps)
        uses_refs.extend(step_refs)
        for step_ref in step_refs:
            uses_refs.extend(
                _collect_local_action_uses(step_ref, repo_root, seen_local_actions)
            )
    return uses_refs


def _external_action_ref_parts(uses_ref: str) -> tuple[str, str, str] | None:
    if uses_ref.startswith("./") or uses_ref.startswith("docker://"):
        return None
    if "@" not in uses_ref:
        return None
    action, ref = uses_ref.rsplit("@", 1)
    parts = action.split("/")
    if len(parts) < 2:
        return None
    return parts[0], parts[1], ref


def _floating_bencher_refs(workflow: dict[str, Any], repo_root: Path) -> list[str]:
    bad_refs: list[str] = []
    for uses_ref in _workflow_uses_refs(workflow, repo_root):
        if not uses_ref.startswith("bencherdev/bencher@"):
            continue
        ref = uses_ref.rsplit("@", 1)[-1]
        if not ref or ref in FLOATING_ACTION_REFS:
            bad_refs.append(uses_ref)
    return bad_refs


def _floating_external_action_refs(
    workflow: dict[str, Any], repo_root: Path
) -> list[str]:
    bad_refs: list[str] = []
    for uses_ref in _workflow_uses_refs(workflow, repo_root):
        parts = _external_action_ref_parts(uses_ref)
        if parts is None:
            continue
        ref = parts[2]
        if not ref or ref in FLOATING_ACTION_REFS:
            bad_refs.append(uses_ref)
    return bad_refs


def _action_tag_exists(
    owner: str, repo: str, ref: str, args: argparse.Namespace
) -> bool:
    encoded_owner = urllib.parse.quote(owner, safe="")
    encoded_repo = urllib.parse.quote(repo, safe="")
    encoded_ref = urllib.parse.quote(ref, safe="")
    url = (
        f"{args.api_url.rstrip('/')}/repos/{encoded_owner}/{encoded_repo}"
        f"/git/ref/tags/{encoded_ref}"
    )
    status = _read_url_status(url, args.token, args.timeout)
    if status == 404:
        return False
    if 200 <= status < 300:
        return True
    if args.api_url.rstrip("/") == "https://api.github.com":
        return _github_tag_exists_with_git(owner, repo, ref, args.timeout)
    raise RuntimeError(
        f"Failed to verify GitHub Action tag {owner}/{repo}@{ref}: "
        f"{url} returned HTTP {status}"
    )


def _github_tag_exists_with_git(
    owner: str, repo: str, ref: str, timeout: float
) -> bool:
    url = f"https://github.com/{owner}/{repo}.git"
    try:
        result = subprocess.run(
            [
                "git",
                "ls-remote",
                "--exit-code",
                "--tags",
                url,
                f"refs/tags/{ref}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(
            f"Failed to verify GitHub Action tag {owner}/{repo}@{ref} with git: {exc}"
        ) from exc
    if result.returncode == 0:
        return True
    if result.returncode == 2:
        return False
    raise RuntimeError(
        f"Failed to verify GitHub Action tag {owner}/{repo}@{ref} with git: "
        f"{result.stderr.strip() or 'git ls-remote failed'}"
    )


def _missing_external_action_tags(
    workflow: dict[str, Any], args: argparse.Namespace
) -> list[str]:
    missing: list[str] = []
    checked: set[tuple[str, str, str]] = set()
    for uses_ref in _workflow_uses_refs(workflow, args.repo_root):
        parts = _external_action_ref_parts(uses_ref)
        if parts is None:
            continue
        owner, repo, ref = parts
        if ref in FLOATING_ACTION_REFS:
            continue
        key = (owner, repo, ref)
        if key in checked:
            continue
        checked.add(key)
        if not _action_tag_exists(owner, repo, ref, args):
            missing.append(uses_ref)
    return missing


def _workflow_steps(workflow: dict[str, Any]) -> list[dict[str, Any]]:
    jobs = workflow.get("jobs")
    if not isinstance(jobs, dict):
        return []

    steps: list[dict[str, Any]] = []
    for job in jobs.values():
        if not isinstance(job, dict):
            continue
        job_steps = job.get("steps")
        if isinstance(job_steps, list):
            steps.extend(step for step in job_steps if isinstance(step, dict))
    return steps


def _step_by_id(workflow: dict[str, Any], step_id: str) -> dict[str, Any] | None:
    for step in _workflow_steps(workflow):
        if step.get("id") == step_id:
            return step
    return None


def _step_by_name(workflow: dict[str, Any], name: str) -> dict[str, Any] | None:
    for step in _workflow_steps(workflow):
        if step.get("name") == name:
            return step
    return None


def _step_if(step: dict[str, Any]) -> str:
    return str(step.get("if", "")).strip()


def _step_run(step: dict[str, Any]) -> str:
    return str(step.get("run", ""))


def _step_continue_on_error(step: dict[str, Any]) -> bool:
    value = step.get("continue-on-error")
    return value is True or str(value).lower() == "true"


def _verify_bencher_steps(workflow: dict[str, Any]) -> None:
    setup_step = _step_by_name(workflow, "Setup Bencher CLI")
    if setup_step is None:
        raise RuntimeError(
            "Local performance dashboard workflow is missing the optional "
            "Bencher CLI setup step."
        )
    bencher_condition = "github.ref == 'refs/heads/main' && vars.BENCHER_PROJECT != ''"
    if _step_if(setup_step) != bencher_condition:
        raise RuntimeError(
            "Local performance dashboard Bencher setup step must be limited "
            "to main runs with BENCHER_PROJECT configured."
        )
    if not _step_continue_on_error(setup_step):
        raise RuntimeError(
            "Local performance dashboard Bencher setup step must be "
            "continue-on-error so optional service setup cannot block Pages "
            "publication."
        )

    report_step = _step_by_name(workflow, "Report benchmark JSON to Bencher")
    if report_step is None:
        raise RuntimeError(
            "Local performance dashboard workflow is missing the optional "
            "Bencher reporting step."
        )
    if _step_if(report_step) != bencher_condition:
        raise RuntimeError(
            "Local performance dashboard Bencher reporting step must be "
            "limited to main runs with BENCHER_PROJECT configured."
        )
    if not _step_continue_on_error(report_step):
        raise RuntimeError(
            "Local performance dashboard Bencher reporting step must be "
            "continue-on-error so optional external reporting cannot block "
            "Pages publication."
        )
    env = report_step.get("env")
    if not isinstance(env, dict) or env.get("BENCHER_API_KEY") != (
        "${{ secrets.BENCHER_API_KEY }}"
    ):
        raise RuntimeError(
            "Local performance dashboard Bencher reporting step must read "
            "BENCHER_API_KEY from the GitHub Actions secret."
        )
    report_run = _step_run(report_step)
    required_flags = {
        "--skip-if-unconfigured",
        "--skip-if-no-input",
    }
    missing_flags = sorted(flag for flag in required_flags if flag not in report_run)
    if missing_flags:
        raise RuntimeError(
            "Local performance dashboard Bencher reporting step is missing "
            "required non-blocking flags: " + ", ".join(missing_flags)
        )


def _verify_publication_steps(workflow: dict[str, Any]) -> None:
    seed_step = _step_by_name(workflow, "Download seed benchmark artifacts")
    if seed_step is not None:
        seed_run = _step_run(seed_step)
        if '--branch "${GITHUB_REF_NAME}"' not in seed_run:
            raise RuntimeError(
                "Local performance dashboard artifact seeding must filter by "
                "GITHUB_REF_NAME so main history is not seeded from PR or "
                "feature-branch artifacts."
            )

    generate_step = _step_by_name(workflow, "Generate performance dashboard")
    if generate_step is None:
        raise RuntimeError(
            "Local performance dashboard workflow is missing the dashboard "
            "generation step."
        )
    generate_run = _step_run(generate_step)
    required_generation_flags = {
        "--history gh-pages/performance/data.json",
        "--clean-output",
        "--allow-empty",
    }
    missing_generation_flags = sorted(
        flag for flag in required_generation_flags if flag not in generate_run
    )
    if missing_generation_flags:
        raise RuntimeError(
            "Local performance dashboard generation step is missing required "
            "publication safety flags: " + ", ".join(missing_generation_flags)
        )

    publish_step = _step_by_id(workflow, "publish_dashboard")
    if publish_step is None:
        raise RuntimeError(
            "Local performance dashboard workflow is missing the "
            "publish_dashboard step."
        )
    if _step_if(publish_step) != "github.ref == 'refs/heads/main'":
        raise RuntimeError(
            "Local performance dashboard publish step must be limited to "
            "github.ref == 'refs/heads/main'."
        )

    pages_step = _step_by_name(workflow, "Request GitHub Pages build")
    if pages_step is None:
        raise RuntimeError(
            "Local performance dashboard workflow is missing the GitHub Pages "
            "build request step."
        )
    pages_condition = _step_if(pages_step)
    if pages_condition != "github.ref == 'refs/heads/main'":
        raise RuntimeError(
            "Local performance dashboard workflow must request a Pages build "
            "on every main publisher run, not only when gh-pages changed."
        )
    if "steps.publish_dashboard.outputs.published" in pages_condition:
        raise RuntimeError(
            "Local performance dashboard Pages build request is incorrectly "
            "gated on the publish_dashboard output."
        )
    if '"/repos/${GITHUB_REPOSITORY}/pages/builds"' not in _step_run(pages_step):
        raise RuntimeError(
            "Local performance dashboard workflow does not call the GitHub "
            "Pages builds endpoint."
        )

    endpoint_step = _step_by_name(workflow, "Verify published dashboard endpoint")
    if endpoint_step is None:
        raise RuntimeError(
            "Local performance dashboard workflow is missing the hosted "
            "endpoint verification step."
        )
    if _step_if(endpoint_step) != "github.ref == 'refs/heads/main'":
        raise RuntimeError(
            "Local performance dashboard endpoint verification must be limited "
            "to github.ref == 'refs/heads/main'."
        )
    endpoint_run = _step_run(endpoint_step)
    required_flags = {
        "--expect-status-file gh-pages/performance/status.json",
        "--expect-data-file gh-pages/performance/data.json",
        "--expect-dashboard-file gh-pages/performance/index.html",
        "--expect-summary-file gh-pages/performance/summary.md",
        "--require-dashboard-page",
        "--require-summary",
        "--require-fresh",
        "--attempts 60",
        "--interval 15",
    }
    missing_flags = sorted(flag for flag in required_flags if flag not in endpoint_run)
    if missing_flags:
        raise RuntimeError(
            "Local performance dashboard hosted endpoint verification is "
            "missing required checks: " + ", ".join(missing_flags)
        )


def verify_local_workflow(workflow: dict[str, Any], args: argparse.Namespace) -> None:
    name = workflow.get("name")
    if name != args.workflow_name:
        raise RuntimeError(
            f"Expected local workflow name {args.workflow_name!r}, got {name!r}."
        )

    triggers = _trigger_names(workflow)
    missing = {"push", "schedule", "workflow_dispatch"} - triggers
    if missing:
        raise RuntimeError(
            "Local performance dashboard workflow is missing triggers: "
            + ", ".join(sorted(missing))
        )
    schedule_crons = _schedule_crons(workflow)
    if not schedule_crons:
        raise RuntimeError("Local performance dashboard workflow has no cron schedule.")
    missing_schedule_crons = sorted(REQUIRED_SCHEDULE_CRONS - schedule_crons)
    if missing_schedule_crons:
        raise RuntimeError(
            "Local performance dashboard workflow is missing required cron "
            "schedule(s): " + ", ".join(missing_schedule_crons)
        )
    if not _has_main_push_trigger(workflow):
        raise RuntimeError(
            "Local performance dashboard workflow has no main push trigger."
        )
    missing_push_paths = sorted(REQUIRED_PUSH_PATHS - _push_paths(workflow))
    if missing_push_paths:
        raise RuntimeError(
            "Local performance dashboard workflow is missing dashboard-relevant "
            "main push path filters: " + ", ".join(missing_push_paths)
        )
    missing_permissions = _missing_publish_permissions(workflow)
    if missing_permissions:
        raise RuntimeError(
            "Local performance dashboard workflow is missing publish permissions: "
            + ", ".join(missing_permissions)
        )
    floating_bencher_refs = _floating_bencher_refs(workflow, args.repo_root)
    if floating_bencher_refs:
        raise RuntimeError(
            "Local performance dashboard workflow uses floating Bencher action "
            "refs instead of a release tag: " + ", ".join(floating_bencher_refs)
        )
    floating_action_refs = _floating_external_action_refs(workflow, args.repo_root)
    if floating_action_refs:
        raise RuntimeError(
            "Local performance dashboard workflow uses floating GitHub Action "
            "refs instead of release tags: " + ", ".join(floating_action_refs)
        )
    if not args.skip_action_ref_existence:
        missing_action_tags = _missing_external_action_tags(workflow, args)
        if missing_action_tags:
            raise RuntimeError(
                "Local performance dashboard workflow references GitHub Action "
                "tags that do not resolve: " + ", ".join(missing_action_tags)
            )
    _verify_bencher_steps(workflow)
    _verify_publication_steps(workflow)


def _matching_workflows(
    workflows: list[dict[str, Any]], args: argparse.Namespace
) -> list[dict[str, Any]]:
    return [
        workflow
        for workflow in workflows
        if workflow.get("path") == args.workflow_path
        or workflow.get("name") == args.workflow_name
    ]


def verify_registered_workflow(
    workflows: list[dict[str, Any]], args: argparse.Namespace
) -> dict[str, Any] | None:
    matches = _matching_workflows(workflows, args)
    if not matches:
        if args.allow_missing:
            return None
        raise RuntimeError(
            "Performance dashboard workflow is not registered in GitHub Actions. "
            f"Expected {args.workflow_name!r} at {args.workflow_path!r}."
        )

    workflow = matches[0]
    if workflow.get("path") != args.workflow_path:
        raise RuntimeError(
            f"Expected registered workflow path {args.workflow_path!r}, "
            f"got {workflow.get('path')!r}."
        )
    if workflow.get("name") != args.workflow_name:
        raise RuntimeError(
            f"Expected registered workflow name {args.workflow_name!r}, "
            f"got {workflow.get('name')!r}."
        )
    if workflow.get("state") != "active":
        raise RuntimeError(
            f"Expected registered workflow state 'active', got {workflow.get('state')!r}."
        )
    return workflow


def main() -> int:
    try:
        args = parse_args()
        local_workflow = _read_local_workflow(args.local_workflow)
        verify_local_workflow(local_workflow, args)
        workflows = _read_workflows(args)
        registered = verify_registered_workflow(workflows, args)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    if registered is None:
        print(
            "local performance dashboard workflow verified; "
            "workflow is not registered on GitHub Actions yet"
        )
        return 0

    print(
        "performance dashboard workflow registered: "
        f"{registered.get('name')} ({registered.get('id')}) "
        f"{registered.get('state')} at {registered.get('path')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
