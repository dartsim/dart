# ABI Compatibility Checks (Issue #1026)

## Status

- In progress: script, pixi task, and Linux CI job added; TODOs remain.

## Problem Statement

Best-effort ABI stability requires automated detection of ABI breaks. There is
no current CI or local workflow to flag ABI changes before merge.

## Scope

- Linux ABI checks for shared libraries.
- Local workflow plus CI job for PRs.
- From/to comparison against the latest release tag (or an explicit ref).

## Out of Scope

- Windows/macOS ABI checks.
- Template-only and header-only ABI changes.
- Side-by-side installation or symbol versioning.

## Recommendation

- Use libabigail (`abidiff`) on Linux.
- Build from/to libraries with debug info (RelWithDebInfo).
- Start with `libdart` only, then expand to other shared libraries if needed.
- Treat added symbols as compatible; fail on removed or changed ABI.

## Current Implementation

- Script: `scripts/abi_check.py`
- Pixi task: `pixi run abi-check`
- CI: Linux job in `ci_ubuntu.yml`

## Usage

- Default: `pixi run abi-check` (selects latest `v<major>.<minor>.<patch>` tag).
- Override from ref: `DART_ABI_FROM=vX.Y.Z pixi run abi-check`.
- Override to ref: `DART_ABI_TO=origin/main pixi run abi-check`.
- Configure libs: `DART_ABI_LIBS=dart,dart-utils pixi run abi-check`.
- Require a from ref: `DART_ABI_REQUIRE_BASELINE=ON pixi run abi-check`.
- If no tag exists for the current major version, the check skips unless
  `DART_ABI_REQUIRE_BASELINE=ON` is set.
- Cross-major comparisons are blocked unless `DART_ABI_ALLOW_CROSS_MAJOR=ON` or
  `--allow-cross-major` is provided.
- Compare arbitrary refs:
  `pixi run abi-check --from v6.16.0 --to v6.16.2`.
- List refs: `pixi run abi-check --list-refs`.
- Filter refs: `pixi run abi-check --list-refs --list-pattern '^v6\\.16\\.'`.

## TODO

- TODO: Flip CI to require a from tag once v7 tags exist.
- TODO: Expand the library list beyond `libdart` once CI signal is stable.
- TODO: Add suppressions only if recurring false positives appear.

## Plan

- Pick the from selection rule (latest release tag by default; allow an
  explicit override).
- Add a script under `scripts/` to build from/to, then run
  `abidiff` with consistent build options.
- Add a pixi task that runs the script locally.
- Add a Linux CI job that runs the same script on PRs.
- Define the exception process (suppression list or explicit approval path).

## Inputs / Outputs

- Inputs: from ref, build options, list of libraries in scope.
- Output: ABI report with a non-zero exit on incompatible changes.

## Validation / Success Criteria

- One command runs the ABI check locally.
- CI runs ABI checks on PRs and fails on breaking changes.
- Intentional ABI breaks follow a documented exception path.

## Risks / Trade-offs

- ABI checks add CI time and require debug info in builds.
- False positives may occur if build flags differ across from/to builds.
- ABI checks do not catch header-only or template-only changes.

## Open Questions

- Which additional libraries should be included beyond `libdart`?
- Do we need a suppression list for approved breaks?
- Should CI build the from tag each run or reuse cached artifacts?
