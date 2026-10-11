# Resume

## Current state (2026-10-11)

M0 is active. Nothing has been pushed; every push and PR needs maintainer
approval. The GitHub milestone "DART 7.0.0" (#105, formerly "DART 6.21.0")
tracks the release.

Local topic branches, each without upstream tracking, on main `1b5722176`:

| Branch | Packet | State |
| --- | --- | --- |
| `docs/dart7-program-home` | WP-7.0.2 | Ships inside the WP-7.0.1 PR |
| `ai/dart7-release-rollover` | WP-7.0.1 | Project-home commit plus rollover commit; final review passes |
| `fix/xml-vector-parse-bounds` | WP-7.0.3 | `31fd3d0f7`; two clean passes; ready to publish |
| `fix/nanobind-gil-finalization` | WP-7.0.4 | `96431bd38`; two clean passes; ready to publish |

The Filament backend effort owns M6a on its own fresh branch from main. It
reports its branch and revisions to this program.

## Next steps

1. Finish the final review passes and record them in
   [verification.md](verification.md).
2. With the maintainer's approval (given on 2026-10-11, once reviews are
   clean), push draft PRs with the "DART 7.0.0" milestone: first
   `ai/dart7-release-rollover`, carrying WP-7.0.2 and WP-7.0.1, then the two
   fixes. Merge main into the fixes after the first PR merges, because main's
   CI rejects that milestone until the rollover lands.
3. After WP-7.0.3 merges, backport it to `release-6.20` with
   `dart-backport-pr`. WP-7.0.4 applies only to the nanobind bindings.
4. Open the follow-up packets: WP-7.0.3b (SDF pose attributes), the ODE callback exception fix, the
   analytical-IK clone crash, and a tokenized release-reference check (the
   regular-expression matcher misreads Markdown inside tokens, unusual
   parameter separators and version suffixes, as its 6.x predecessor did).
   Then start WP-7.0.5 to WP-7.0.8.

## Risks and recovery

- Main moves quickly. Before every approved push to a published branch, merge
  the latest `origin/main` (never rebase a published branch) and re-run the
  packet's gates.
- The rollover and project-home branches both edit `docs/plans/dashboard.md`;
  stacking the rollover keeps them consistent.
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
