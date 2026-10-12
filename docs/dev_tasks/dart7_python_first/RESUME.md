# Resume

## Current state (2026-10-12)

M0 is active. The GitHub milestone "DART 7.0.0" (#105, formerly "DART 6.21.0")
tracks the release, and main's release target reads 7.0.0.

| Pull request | Packet | State |
| --- | --- | --- |
| [#3697](https://github.com/dartsim/dart/pull/3697) | WP-7.0.1 and WP-7.0.2 | Merged |
| [#3698](https://github.com/dartsim/dart/pull/3698) | WP-7.0.3 | Merged |
| [#3717](https://github.com/dartsim/dart/pull/3717) | WP-7.0.3 backport to `release-6.20` | Merged, for DART 6.20.1 |
| [#3719](https://github.com/dartsim/dart/pull/3719) | WP-7.0.3b | Merged; four review findings remain open |
| [#3699](https://github.com/dartsim/dart/pull/3699) | WP-7.0.4 | Merged |
| [#3720](https://github.com/dartsim/dart/pull/3720) | WP-7.0.5 | Merged; the sleeping rows and the `dart7-base` pin follow M0's exit |
| [#3725](https://github.com/dartsim/dart/pull/3725) | WP-7.0.6: version file, gz shim test and contact-surface header probe | Merged |
| [#3729](https://github.com/dartsim/dart/pull/3729) | WP-7.0.6: gz API inventory | In review |
| [#3731](https://github.com/dartsim/dart/pull/3731) | WP-7.0.6: C++14 contact-surface header tree, follow-up to #3725 | Draft |
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

1. Land #3728 (WP-7.0.7), #3729 (gz API inventory), #3731 (the C++14
   contact-surface follow-up to #3725) and #3722 (analytical-IK copy fix);
   M0 exits then. Backport #3722 to `release-6.20` after it merges.
2. After M0 exits, in order: a small PR adding nightly Gazebo-preset sleeping
   rows, which cover gz's custom-filter replay path; the `dart7-base` pin,
   following WP-7.0.5's baseline procedure in
   [verification.md](verification.md); then the M1a PR.
3. In parallel with step 2: M5a's rename table v0, the published shape and
   query capability matrix, and a null-check fix in the skel parser's detector
   fallback, which dereferences the factory's result without checking it. The
   matrix and M1b's strict shape rejection must land before M1f.
4. Resolve #3719's open review findings in a follow-up PR on main: rotated
   inertial poses, the SDF 1.4–1.6 version gate, soft-shape poses other than
   boxes, and empty quaternion poses. See WP-7.0.3b in
   [verification.md](verification.md). It does not hold M0's exit.
5. Open the remaining follow-up packets: a tokenized release-reference check
   (the regular-expression matcher misreads Markdown inside tokens, unusual
   parameter separators and version suffixes, as its 6.x predecessor did), and
   a DART 7 wall-time dashboard (the DART 6 dashboard now tracks `release-6.*`
   only).
6. Add the macOS and Windows Python 3.15 wheels to 6.20.x once conda-forge's
   `bullet-cpp` builds for Python 3.15. They do not hold M0's exit.

## Risks and recovery

- Main moves quickly. Before every approved push to a published branch, merge
  the latest `origin/main` (never rebase a published branch) and re-run the
  packet's gates.
- The rollover integrates #3696's Filament AI-checker and test edits while
  retaining the DART 7 branch profile and release routing.
- dartpy has no Python 3.15 CI lane yet. Re-run the shutdown regression test
  against a 3.15 build when touching finalization code.
- Main's two Windows C++ jobs fail `ContactSurfaceProbe` at `722f28ddd`,
  because MSVC rejects the include tree's C++17 nested namespaces in C++14
  mode; #3731 fixes them.

## Verification commands

- `pixi run lint`, then `pixi run check-lint`.
- `pixi run test-ai-infra` and `pixi run exercise-agent-scenarios` for AI-policy
  changes.
- `pixi run build`, then `pixi run test`.
- `pixi run build-py-dev`, then `pixi run test-py`.
