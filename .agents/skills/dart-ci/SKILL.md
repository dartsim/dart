---
name: dart-ci
description: "DART CI: GitHub Actions, cache debugging, and platform-specific failures"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-ci/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART CI/CD Troubleshooting

Load this skill when debugging CI failures or working with GitHub Actions.

When the failing claim depends on model/scene structure, physics behavior, or
OSG output, also load `dart-verify-sim` and reproduce it with a text oracle plus
assessed visual evidence. Document a visual exception when OSG/Xvfb is
unavailable or not applicable in the failing environment.

## Quick Commands

```bash
# Monitor PR checks
gh pr checks <PR_NUMBER>
gh pr view <PR_NUMBER> --json headRefOid,mergeStateStatus,statusCheckRollup

# View run details
gh run list --branch <BRANCH> -e pull_request -L 20
gh run view <RUN_ID> --json status,conclusion,url

# Debug failures
gh run view <RUN_ID> --job <JOB_ID> --log-failed
gh run view <RUN_ID> --json jobs --jq '.jobs[] | {name, databaseId}'

# Rerun failed jobs only after explicit maintainer/user approval
gh run rerun <RUN_ID> --failed
gh run rerun <RUN_ID> --job <DATABASE_ID>
```

## Full Documentation

For complete CI/CD guide: `docs/onboarding/ci-cd.md`

## Common Failure Modes

| Failure Type         | Solution                                                 |
| -------------------- | -------------------------------------------------------- |
| Formatting fails     | `pixi run lint`; push only after approval                |
| Codecov patch fails  | Add tests for uncovered lines                            |
| FreeBSD RTTI fails   | Use type enums + `static_cast` instead of `dynamic_cast` |
| macOS ARM64 SEGFAULT | Replace `alloca()`/VLAs with `std::vector<T>`            |
| RTD build fails      | Use defensive `.get(key, default)` patterns              |
| gz-physics fails     | Reproduce with `pixi run -e gazebo test-gz`              |

## Workflow Architecture

| Workflow            | Purpose                                | Platforms |
| ------------------- | -------------------------------------- | --------- |
| `ci_ubuntu.yml`     | AI checks, lint, build, test, coverage | Ubuntu    |
| `ci_macos.yml`      | Lint, build, test                      | macOS     |
| `ci_windows.yml`    | Lint, build, test                      | Windows   |
| `ci_freebsd.yml`    | Build + test (VM)                      | FreeBSD   |
| `ci_gz_physics.yml` | Gazebo integration                     | Ubuntu    |

## Downstream Compatibility Policy

Package, exported-target, collision, constraint, and dependency changes can
affect Gazebo/gz-physics even when local unit tests pass. Reproduce those
failures with the Gazebo Pixi environment when practical:
`pixi run -e gazebo test-gz`.

## Fast Iteration Loop

1. Identify failing step from job logs
2. Reproduce locally with same build toggles
3. Fix the smallest failing test
4. Push only after explicit maintainer/user approval, then monitor:
   `gh run view <RUN_ID> --json status,conclusion,url`

## Caching And Duration

Treat the current run and job logs as authoritative. Inspect cache restore/save
steps and job timestamps before changing a key; do not rely on fixed duration
or cache-speed estimates in this skill.
