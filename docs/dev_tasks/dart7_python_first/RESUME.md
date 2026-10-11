# Resume

## Current state (2026-10-11)

M0 is active. The GitHub milestone "DART 7.0.0" (#105, formerly "DART 6.21.0")
tracks the release.

| Pull request | Packet | State |
| --- | --- | --- |
| [#3697](https://github.com/dartsim/dart/pull/3697) | WP-7.0.2 and WP-7.0.1 | Draft: the project home plus the release-line and AI-policy rollover; lands first |
| [#3698](https://github.com/dartsim/dart/pull/3698) | WP-7.0.3 | Draft; merges main after #3697 |
| [#3699](https://github.com/dartsim/dart/pull/3699) | WP-7.0.4 | Draft; merges main after #3697 |

The Filament backend effort owns M6a in
[#3696](https://github.com/dartsim/dart/pull/3696).

## Next steps

1. Land #3697 first. Until it merges, main's CI rejects pull requests with the
   "DART 7.0.0" milestone, because main's release target still reads 6.21.0.
2. Then merge main into #3698, #3699 and #3696 so their release-target checks
   pass.
3. After WP-7.0.3 merges, backport it to `release-6.20` with
   `dart-backport-pr`. WP-7.0.4 applies only to the nanobind bindings.
4. Open the follow-up packets: WP-7.0.3b (SDF pose attributes), the ODE
   callback exception fix, the analytical-IK clone crash, a tokenized
   release-reference check (the regular-expression matcher misreads Markdown
   inside tokens, unusual parameter separators and version suffixes, as its
   6.x predecessor did), and a DART 7 wall-time dashboard (the DART 6
   dashboard now tracks `release-6.*` only). Then start WP-7.0.5 to WP-7.0.8.

## Risks and recovery

- Main moves quickly. Before every approved push to a published branch, merge
  the latest `origin/main` (never rebase a published branch) and re-run the
  packet's gates.
- The rollover overlaps the Filament effort's AI-checker and test edits.
  Whichever lands second keeps both sides.
- dartpy has no Python 3.15 CI lane yet. Re-run the shutdown regression test
  against a 3.15 build when touching finalization code.

## Verification commands

- `pixi run lint`, then `pixi run check-lint`.
- `pixi run test-ai-infra` and `pixi run exercise-agent-scenarios` for AI-policy
  changes.
- `pixi run build`, then `pixi run test`.
- `pixi run build-py-dev`, then `pixi run test-py`.
