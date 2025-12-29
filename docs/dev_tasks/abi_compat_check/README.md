# ABI Compatibility Checks (Issue #1026)

## Status

- Tooling approach selected; implementation pending.

## Problem Statement

Best-effort ABI stability requires automated detection of ABI breaks. There is
no current CI or local workflow to flag ABI changes before merge.

## Scope

- Linux ABI checks for shared libraries.
- Local workflow plus CI job for PRs.
- Baseline comparison against the latest release tag (or an explicit tag).

## Out of Scope

- Windows/macOS ABI checks.
- Template-only and header-only ABI changes.
- Side-by-side installation or symbol versioning.

## Recommendation

- Use libabigail (`abidiff`) on Linux.
- Build baseline and current libraries with debug info (RelWithDebInfo).
- Start with `libdart` only, then expand to other shared libraries if needed.
- Treat added symbols as compatible; fail on removed or changed ABI.

## Plan

- Pick the baseline selection rule (latest release tag by default; allow an
  explicit override).
- Add a script under `scripts/` to build baseline and current, then run
  `abidiff` with consistent build options.
- Add a pixi task that runs the script locally.
- Add a Linux CI job that runs the same script on PRs.
- Define the exception process (suppression list or explicit approval path).

## Inputs / Outputs

- Inputs: baseline tag, build options, list of libraries in scope.
- Output: ABI report with a non-zero exit on incompatible changes.

## Validation / Success Criteria

- One command runs the ABI check locally.
- CI runs ABI checks on PRs and fails on breaking changes.
- Intentional ABI breaks follow a documented exception path.

## Risks / Trade-offs

- ABI checks add CI time and require debug info in builds.
- False positives may occur if build flags differ across baseline/current.
- ABI checks do not catch header-only or template-only changes.

## Open Questions

- Which additional libraries should be included beyond `libdart`?
- Do we need a suppression list for approved breaks?
- Should CI build the baseline tag each run or reuse cached artifacts?
