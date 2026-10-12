# Resume

## Current state (2026-10-11)

M0 is active. The GitHub milestone "DART 7.0.0" (#105, formerly "DART 6.21.0")
tracks the release, and main's release target reads 7.0.0.

| Pull request | Packet | State |
| --- | --- | --- |
| [#3697](https://github.com/dartsim/dart/pull/3697) | WP-7.0.1 and WP-7.0.2 | Merged |
| [#3698](https://github.com/dartsim/dart/pull/3698) | WP-7.0.3 | Merged |
| [#3717](https://github.com/dartsim/dart/pull/3717) | WP-7.0.3 backport to `release-6.20` | Merged, for DART 6.20.1 |
| [#3719](https://github.com/dartsim/dart/pull/3719) | WP-7.0.3b | Merged; four review findings remain open |
| [#3699](https://github.com/dartsim/dart/pull/3699) | WP-7.0.4 | Merged |
| [#3720](https://github.com/dartsim/dart/pull/3720) | WP-7.0.5 | Merged; the `dart7-base` pin is recorded right before the first M1 PR |
| [#3725](https://github.com/dartsim/dart/pull/3725) | WP-7.0.6: version file, gz shim test and contact-surface header probe | Merged |
| [#3729](https://github.com/dartsim/dart/pull/3729) | WP-7.0.6: gz API inventory | In review |
| [#3728](https://github.com/dartsim/dart/pull/3728) | WP-7.0.7 | In review |
| [#3704](https://github.com/dartsim/dart/pull/3704) | WP-7.0.8: 6.20.1 wheels for Python 3.12 to 3.14 | Merged |
| [#3730](https://github.com/dartsim/dart/pull/3730) | WP-7.0.8: Linux Python 3.15 wheels for 6.20.1 | Merged |
| [#3723](https://github.com/dartsim/dart/pull/3723) | Follow-up: ODE collision-callback exceptions | Merged; backported in [#3727](https://github.com/dartsim/dart/pull/3727) |
| [#3722](https://github.com/dartsim/dart/pull/3722) | Follow-up: analytical-IK clone and copy crashes | In review; its backport follows the merge |

macOS and Windows Python 3.15 wheels for 6.20.x wait for conda-forge's Python
3.15 builds of `bullet-cpp`. The GIL review of that line's pybind11 bindings
found no crash or hang during finalization on Python 3.15, so they are
unchanged.

M6a merged in [#3696](https://github.com/dartsim/dart/pull/3696). Main uses
Filament and the interim camelCase `dartpy.gui`, and
[#3713](https://github.com/dartsim/dart/pull/3713) applied WP-7.0.4's
finalization rule to the `python/dartpy/gui/` binding sites. M6b, sensor
rendering and the Python render/viewer API, remains pending.

## Next steps

1. Land #3729, #3728 and #3722, then backport #3722 to `release-6.20`.
2. Resolve #3719's open review findings in a follow-up PR on main: rotated
   inertial poses, the SDF 1.4–1.6 version gate, soft-shape poses other than
   boxes, and empty quaternion poses. See WP-7.0.3b in
   [verification.md](verification.md).
3. Pin `dart7-base` right before the first M1 PR, then start M1 with stage
   extraction (M1a), preceded by the default-detector switch (M1f) only if its
   prerequisites have passed. M5a's rename table starts when M0 exits.
4. Open the remaining follow-up packets: a tokenized release-reference check
   (the regular-expression matcher misreads Markdown inside tokens, unusual
   parameter separators and version suffixes, as its 6.x predecessor did), and
   a DART 7 wall-time dashboard (the DART 6 dashboard now tracks `release-6.*`
   only).
5. Add the macOS and Windows Python 3.15 wheels to 6.20.x once conda-forge's
   `bullet-cpp` builds for Python 3.15. Ask the maintainer whether they hold
   M0's exit, and with it M5a's start.
6. Ask the maintainer whether the API freeze `F` also waits for the additive
   must-haves, or they are exempt from the move to 7.1. The README's release
   rules record the question; it must be settled before `F` is declared.

## Risks and recovery

- Main moves quickly. Before every approved push to a published branch, merge
  the latest `origin/main` (never rebase a published branch) and re-run the
  packet's gates.
- The rollover integrates #3696's Filament AI-checker and test edits while
  retaining the DART 7 branch profile and release routing.
- dartpy has no Python 3.15 CI lane yet. Re-run the shutdown regression test
  against a 3.15 build when touching finalization code.

## Verification commands

- `pixi run lint`, then `pixi run check-lint`.
- `pixi run test-ai-infra` and `pixi run exercise-agent-scenarios` for AI-policy
  changes.
- `pixi run build`, then `pixi run test`.
- `pixi run build-py-dev`, then `pixi run test-py`.
