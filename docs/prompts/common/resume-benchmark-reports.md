# DART: Resume Benchmark Reports Task

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
Resume the benchmark reports task in the DART repo.

Context
- Repo: dartsim/dart
- Expected branch: bm-report (verify with git, do not switch unless needed and explained)
- Goal: local benchmark reports (pixi), PR comparison comment, and gh-pages trend dashboard.

Known implementation (verify in repo)
- Local reports: scripts/run_cpp_benchmark.py supports --report and scrubs host_name/executable, writes to benchmarks/reports/<benchmark>/<runner_id>/<timestamp>-<shortsha>.json
- Pixi task: pixi run bm-report
- PR compare: scripts/benchmark_report_compare.py and .github/workflows/benchmark_reports.yml
- Dashboard: scripts/benchmark_report_site.py and benchmarks/site/* with .github/workflows/benchmark_publish.yml
- Docs: docs/onboarding/testing.md and docs/dev_tasks/bm_report/README.md
- .gitignore ignores benchmarks/site/data.json

Safety/constraints
- Do not discard work: no git reset --hard, no git clean -fdx, no dropping stashes.
- Use repo entry points (pixi run ...) and follow AGENTS.md and onboarding docs.

Step 1: Recon
- git status -sb
- git branch -vv
- git log -10 --oneline --decorate
- git diff --stat (and git diff if needed)
- confirm working directory via git rev-parse --show-toplevel

Step 2: Confirm state
- Open docs/dev_tasks/bm_report/README.md to see current status and next steps.
- Summarize what is already implemented and any gaps.

Step 3: Continue
- If asked to run a real report: use DART_BENCH_RUNNER_ID and pixi run bm-report <bench>, then commit the new JSON under benchmarks/reports/.
- If asked to publish: ensure gh-pages is enabled in repo settings and trigger benchmark_publish.yml.
- If asked to improve: keep reports sanitized; do not add hostnames or executable paths.

Report back
- Summarize repo state, remaining tasks, and any commands you ran.
```
