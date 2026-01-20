---
name: dart-ci
description: DART CI/CD troubleshooting - GitHub Actions, cache debugging, platform-specific failures
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-ci/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART CI/CD Troubleshooting

Load this skill when debugging CI failures or working with GitHub Actions.

## Quick Commands

```bash
# Monitor PR checks
gh pr checks <PR_NUMBER>
gh pr checks <PR_NUMBER> --watch --interval 30 --fail-fast

# View run details
gh run list --branch <BRANCH> -e pull_request -L 20
gh run watch <RUN_ID> --interval 30
gh run view <RUN_ID> --json status,conclusion,url

# Debug failures
gh run view <RUN_ID> --job <JOB_ID> --log-failed
gh run view <RUN_ID> --json jobs --jq '.jobs[] | {name, databaseId}'

# Rerun failed jobs
gh run rerun <RUN_ID> --failed
gh run rerun <RUN_ID> --job <DATABASE_ID>
```

## Full Documentation

For complete CI/CD guide: `docs/onboarding/ci-cd.md`

## Common Failure Modes

| Failure Type         | Solution                                                 |
| -------------------- | -------------------------------------------------------- |
| Formatting fails     | `pixi run lint` then push                                |
| Codecov patch fails  | Add tests for uncovered lines                            |
| FreeBSD RTTI fails   | Use type enums + `static_cast` instead of `dynamic_cast` |
| macOS ARM64 SEGFAULT | Replace `alloca()`/VLAs with `std::vector<T>`            |
| RTD build fails      | Use defensive `.get(key, default)` patterns              |
| gz-physics fails     | Reproduce with `pixi run -e gazebo test-gz`              |

## Workflow Architecture

| Workflow            | Purpose                 | Platforms |
| ------------------- | ----------------------- | --------- |
| `ci_lint.yml`       | Formatting              | Ubuntu    |
| `ci_ubuntu.yml`     | Build + test + coverage | Ubuntu    |
| `ci_macos.yml`      | Build + test            | macOS     |
| `ci_windows.yml`    | Build + test            | Windows   |
| `ci_freebsd.yml`    | Build + test (VM)       | FreeBSD   |
| `ci_gz_physics.yml` | Gazebo integration      | Ubuntu    |

## Fast Iteration Loop

1. Identify failing step from job logs
2. Reproduce locally with same build toggles
3. Fix the smallest failing test
4. Push and monitor: `gh run watch <RUN_ID>`

## Caching

- sccache/ccache reduces build time 50-70%
- Check cache hit rates in workflow logs
- Force cache bust by changing cache key if needed

## Expected CI Times

| Platform | Cached    | Uncached  |
| -------- | --------- | --------- |
| Ubuntu   | 20-30 min | 45-60 min |
| macOS    | 15-25 min | 30-45 min |
| Windows  | 15-20 min | 25-35 min |
