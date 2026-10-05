#!/usr/bin/env python3
"""Open, update, or close the nightly-failure issues for a Nightly CI run.

Run by the `report` job of .github/workflows/nightly.yml once every other job
has finished. Jobs are grouped by their nightly.yml caller job (`linux`,
`macos`, `freebsd`, ...; the part of the job name before " / "), and each group
gets at most one open issue (label `nightly-failure`) per branch: a failing
night opens it or comments on it, and a night where the group succeeds closes
it. Skipped groups are left alone. `--dry-run` prints the plan instead of
touching issues.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess

LABEL = "nightly-failure"
FAILED = {"failure", "timed_out", "cancelled", "startup_failure", "stale"}
EXCERPT_LINES = 40
EXCERPT_CHARS = 4000
EXCERPT_BUDGET = 40000  # GitHub caps issue bodies and comments at 65536 chars.
TIMESTAMP = re.compile(r"^\d{4}-\d\d-\d\dT[\d:.]+Z ")
ANSI = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")


def gh(*args: str, stdin: str | None = None) -> str:
    return subprocess.run(
        ["gh", *args], input=stdin, capture_output=True, text=True, check=True
    ).stdout


def job_log(repo: str, job_id: int) -> str:
    path = f"repos/{repo}/actions/jobs/{job_id}/logs"
    # Newer gh refuses to print logs with escape sequences unless this flag is
    # given; older gh rejects the flag but prints the log without it.
    for args in (["--allow-escape-sequences", path], [path]):
        try:
            return gh("api", *args)
        except subprocess.CalledProcessError:
            pass
    return ""  # e.g. startup failures have no log


def marker(branch: str, group: str) -> str:
    return f"<!-- nightly-ci:{branch}:{group} -->"


def plan(jobs: list[dict], issues: list[dict], branch: str) -> list[tuple]:
    """(action, group, issue number, failed jobs) for each group needing one."""
    groups: dict[str, list[dict]] = {}
    for job in jobs:
        if job["conclusion"] is not None:  # skip still-running jobs (this one)
            groups.setdefault(job["name"].split(" / ")[0], []).append(job)
    actions = []
    for group, group_jobs in groups.items():
        issue = next(
            (i["number"] for i in issues if marker(branch, group) in (i["body"] or "")),
            None,
        )
        failed = [j for j in group_jobs if j["conclusion"] in FAILED]
        if failed:
            actions.append(("comment" if issue else "create", group, issue, failed))
        elif issue and any(j["conclusion"] == "success" for j in group_jobs):
            actions.append(("close", group, issue, []))
    return actions


def excerpt(log: str) -> str:
    """Lines leading up to the first error annotation, else the log tail."""
    lines = [TIMESTAMP.sub("", ANSI.sub("", line)) for line in log.splitlines()]
    if not lines:
        return ""
    end = next(
        (i for i, line in enumerate(lines) if "##[error]" in line), len(lines) - 1
    )
    return "\n".join(lines[max(0, end - EXCERPT_LINES) : end + 1])[-EXCERPT_CHARS:]


def failure_report(run: dict, jobs: list[dict], logs: dict[int, str]) -> str:
    title = (run.get("head_commit") or {}).get("message", "").split("\n")[0]
    out = [
        f"**Run:** {run['html_url']} ({run['run_started_at'][:10]})",
        f"**Commit:** `{run['head_sha'][:12]}` {title}",
        "",
        "**Failing jobs:**",
    ]
    for job in jobs:
        steps = [
            s["name"] for s in job.get("steps") or [] if s["conclusion"] == "failure"
        ]
        step = f", step `{steps[0]}`" if steps else ""
        out.append(f"- [{job['name']}]({job['html_url']}) ({job['conclusion']}{step})")
    budget = EXCERPT_BUDGET
    for job in jobs:
        text = logs.get(job["id"], "")
        if not text or len(text) > budget:
            continue
        budget -= len(text)
        out += ["", f"<details><summary>{job['name']}: log excerpt</summary>", ""]
        out += ["````text", text, "````", "", "</details>"]
    return "\n".join(out)


def instructions(repo: str, run: dict, branch: str, group: str) -> str:
    run_id, sha = run["id"], run["head_sha"][:12]
    return f"""
### How to fix

1. Rule out a flake: `gh run rerun {run_id} --failed --repo {repo}`. If the
   rerun passes, comment with the job and test name; a recurring flake still
   needs a fix.
2. Read the failures: `gh run view {run_id} --log-failed --repo {repo}`.
   Other `{LABEL}` issues from the same run may share the root cause.
3. Reproduce on `{branch}` at `{sha}`. The `{group}` job in
   `.github/workflows/nightly.yml` names the workflow that failed; most of its
   steps are `pixi run <task>` and run the same way locally (see
   `docs/onboarding/ci-cd.md`). For a hosted-only platform (Windows, macOS,
   FreeBSD), push a branch and dispatch that workflow on it.
4. Fix the root cause in a PR against `{branch}` that references this issue.
   Don't skip, disable, or loosen a test or check to get green unless it is
   demonstrably wrong, and say why in the PR.

Each failing night adds a comment here with its run and log excerpts; the
first night on which `{group}` succeeds closes this issue.

### Prompt for an AI agent

> Fix the `{group}` nightly CI failure in {repo} tracked by this issue (branch
> `{branch}`). Start from the most recent failing run listed here (body or
> latest comment): read its failures with
> `gh run view <run-id> --log-failed --repo {repo}`, follow AGENTS.md,
> reproduce with the failing step's pixi task, and fix the root cause in a PR
> against `{branch}` that references this issue. Don't disable or weaken tests
> to get green; if a test is wrong, explain why in the PR. If the failure
> doesn't reproduce and passes on rerun, report it as flaky instead of
> changing code.
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--repo", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--branch", required=True, help="branch the issues track")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    repo = args.repo

    run = json.loads(gh("api", f"repos/{repo}/actions/runs/{args.run_id}"))
    jobs = [
        json.loads(line)
        for line in gh(
            "api",
            "--paginate",
            f"repos/{repo}/actions/runs/{args.run_id}/jobs?per_page=100",
            "--jq",
            ".jobs[] | @json",
        ).splitlines()
    ]
    issues = json.loads(
        gh(
            "issue",
            "list",
            "--repo",
            repo,
            "--label",
            LABEL,
            "--state",
            "open",
            "--json",
            "number,body",
            "--limit",
            "200",
        )
    )
    actions = plan(jobs, issues, args.branch)
    if not actions:
        print("nothing to report")
    for action, group, issue, failed in actions:
        logs = {job["id"]: excerpt(job_log(repo, job["id"])) for job in failed}
        if action == "close":
            body = f"`{group}` passed in {run['html_url']} on `{run['head_sha'][:12]}`; closing."
        else:
            body = failure_report(run, failed, logs)
        if action == "create":
            body = "\n".join(
                [
                    marker(args.branch, group),
                    body,
                    instructions(repo, run, args.branch, group),
                ]
            )
        print(f"== {action} {group} (issue {issue})\n{body}\n")
        if args.dry_run:
            continue
        if action == "create":
            gh(
                "label",
                "create",
                LABEL,
                "--repo",
                repo,
                "--force",
                "--color",
                "B60205",
                "--description",
                "Opened by the Nightly workflow; closes on a green night",
            )
            title = f"Nightly CI: {group} failing on {args.branch}"
            gh(
                "issue",
                "create",
                "--repo",
                repo,
                "--title",
                title,
                "--label",
                LABEL,
                "--body-file",
                "-",
                stdin=body,
            )
        elif action == "comment":
            gh(
                "issue",
                "comment",
                str(issue),
                "--repo",
                repo,
                "--body-file",
                "-",
                stdin=body,
            )
        else:
            gh("issue", "close", str(issue), "--repo", repo, "--comment", body)


if __name__ == "__main__":
    main()
