# DART: Fix Downstream Issue (gz-physics/Gazebo)

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## When to Use

Issues reported in downstream projects (gz-physics, gazebo, gz-sim) that trace back to DART behavior:

- Crashes/assertions in DART code triggered by downstream usage
- NaN/Inf propagation through DART APIs
- Missing input validation in DART that downstream relies on
- Performance issues in DART affecting downstream

## Prompt

```text
# DART: Fix Downstream Issue

Context
- Downstream issue: <DOWNSTREAM_URL> (e.g., https://github.com/gazebosim/gz-physics/issues/XXXX)
- Downstream project: <PROJECT> (gz-physics | gazebo | gz-sim | other)
- Symptom: <BRIEF_DESCRIPTION>
- Notes: <optional>

Workflow

1. Analyze downstream issue:
   - Read issue thoroughly: crash logs, stack traces, reproduction steps.
   - Identify the DART API/component involved.
   - Understand the usage pattern that triggers the bug.

2. Trace root cause in DART:
   - Search DART codebase for the failing code path.
   - Identify missing validation, edge case, or incorrect behavior.
   - Check if similar patterns exist elsewhere (grep for related APIs).
   - Review existing validation patterns: PRs #2444, #2442, #2440 for examples.

3. Plan the fix:
   - Determine minimal fix (add validation, guard, or correct behavior).
   - Identify test location (prefer unit test in relevant module).
   - Check if fix applies to both DART 6.16 and DART 7.

4. Implement on main branch first (DART 7):
   - Create branch: `fix/<downstream-project>-<issue-number>-<brief-description>`
   - Implement fix with proper warning/error logging.
   - Add regression test that reproduces the downstream scenario.
   - Run `pixi run lint` before committing.
   - Verify: `pixi run test-unit` (quick), `pixi run test-all` (final).

5. Create PR for main:
   - Push: `git push -u origin HEAD`
   - Create PR: `gh pr create --base main`
   - Reference downstream issue in PR body.
   - Set milestone: DART 7.0

6. Backport to release-6.16 (if applicable):
   - Create branch from release-6.16: `fix/<downstream-project>-<issue-number>-<brief-description>-6.16`
   - Cherry-pick or adapt the fix (watch for API differences).
   - DART 6.16 differences:
     - Use `dtwarn << "message"` instead of `DART_WARN()`.
     - Use `#include <dart/dart.hpp>` instead of `#include <dart/all.hpp>`.
     - Test CMakeLists may use `GLOB_SOURCES` pattern.
   - Run `pixi run lint` and verify tests pass.

7. Create PR for release-6.16:
   - Push: `git push -u origin HEAD`
   - Create PR: `gh pr create --base release-6.16`
   - Reference downstream issue and main PR in body.
   - Set milestone: DART 6.16.x

8. Monitor CI:
   - Watch both PRs: `gh pr checks <PR_NUMBER>`
   - Fix any lint or test failures.
   - Both PRs must pass before considering complete.

Rules
- Bug fixes require PRs to BOTH `release-6.16` AND `main` branches.
- Keep fixes minimal - no refactoring while fixing bugs.
- Add tests that reproduce the downstream scenario.
- Follow existing validation patterns in the codebase.
- Reference downstream issue in commit message and PR.

API Differences (DART 6 vs 7)
- Logging: DART 7 uses `DART_WARN()`, DART 6 uses `dtwarn << "..."`.
- Headers: DART 7 uses `<dart/All.hpp>`, DART 6 uses `<dart/dart.hpp>`.
- Some test infrastructure differs between branches.

Output
- Summary of root cause and fix.
- Files modified.
- PR links for both branches.
- CI status for both PRs.
- Link back to downstream issue for follow-up.
```

## Example Usage

```text
# DART: Fix Downstream Issue

Context
- Downstream issue: https://github.com/gazebosim/gz-physics/issues/XXXX
- Downstream project: gz-physics
- Symptom: Crash when setting invalid value in a plugin
- Notes: Assertion failure in GenericJoint, NaN propagation

Workflow
...
```

## Related Templates

- [new.md](new.md) - For issues reported directly in DART
- [../release/backport-pr.md](../release/backport-pr.md) - For backporting existing fixes
