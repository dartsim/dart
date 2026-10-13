# Resume

## Current reality (2026-10-13)

This section supersedes the historical handoff below. The resumed checkout
started detached at `dart7-base`; the current local packet branch is
`fix/sdf-pose-followup`, initially based on fetched main `e378f985f`.
The local fix is committed as `6047aca4a`; current main `95282f1b9` is merged
before publication as draft [#3782](https://github.com/dartsim/dart/pull/3782).
Its two additions change hooks and handoff docs,
with no SDF or engine source delta.

- M1a merged in [#3775](https://github.com/dartsim/dart/pull/3775). The
  `dart7-base` nightly smoke has finished: all 78 rows pass; its pinned SHA,
  host and record digest are in [verification.md](verification.md#m1a-world-step-stages).
- Demo consolidation merged in [#3780](https://github.com/dartsim/dart/pull/3780).
  It supplies the managed viewer host; M6b's render API remains required.
- [#3728](https://github.com/dartsim/dart/pull/3728) (scope and acceptance)
  and [#3729](https://github.com/dartsim/dart/pull/3729) (gz API inventory)
  remain open. Their existing branches own those changes; do not duplicate
  their implementations. Their newer decisions replace date-relative gates
  with dependencies and retain FCL for C++ while gating the Python DART
  detector default on the capability matrix and M1b shape rejection.
- The recorded M0 exit waits for those two PRs even though M1a has merged.
  Reconcile that sequencing at their integration; do not infer that the
  completed M1a waived the remaining M0 acceptance.
- Draft [#3782](https://github.com/dartsim/dart/pull/3782) fixes the four
  confirmed WP-7.0.3b pose gaps:
  rotated inertia, non-box soft geometry, modern-version admission and empty
  poses. It changes only the SDF parser, regression evidence and its docs;
  public headers, class layouts and stepping kernels remain untouched. Core,
  Python, GUI and the patched forward Gazebo gates pass, and two independent
  reviews are clean. The verification owner records exact counts, skipped
  cases, matched native captures and the isolated baseline's limits.
- The full program still requires M1b through M1i, gated M2, M5, the remaining
  M6b and M7 release acceptance. Keep this project home until those criteria,
  required reviews and durable promotion are complete.

Immediate next action: monitor current-head CI and Codex review of #3782.
The independently reproduced World-local preparation bug is being fixed
locally on `fix/world-local-preparation`; the shared-Skeleton protection
must remain intact. This compatible fix can progress without waiving M0's
integration exit or claiming M1b completion. See
[the M1b evidence](verification.md#m1b-preparation-scope-spike).
Keep existing #3728 and #3729 branches as their integration owners. Continue
the missing packets; this draft publication is not program completion.
The user approved #3782's publication, attachment and Codex review request;
marking it ready and merging still require their separate lifecycle gates.
The PR's rendered attachment was downloaded and its digest matches the
assessed comparison under `build/sdf-pose-evidence/`.

## Historical handoff (2026-10-11)

M0 is active. The GitHub milestone "DART 7.0.0" (#105, formerly "DART 6.21.0")
tracks the release, and main's release target reads 7.0.0.

| Pull request | Packet | State |
| --- | --- | --- |
| [#3697](https://github.com/dartsim/dart/pull/3697) | WP-7.0.1 and WP-7.0.2 | Merged |
| [#3698](https://github.com/dartsim/dart/pull/3698) | WP-7.0.3 | Merged |
| [#3717](https://github.com/dartsim/dart/pull/3717) | WP-7.0.3 backport to `release-6.20` | Draft, for DART 6.20.1 |
| [#3699](https://github.com/dartsim/dart/pull/3699) | WP-7.0.4 | Merged |

M6a merged in [#3696](https://github.com/dartsim/dart/pull/3696). Main uses
Filament and the interim camelCase `dartpy.gui`, and
[#3713](https://github.com/dartsim/dart/pull/3713) applied WP-7.0.4's
finalization rule to the `python/dartpy/gui/` binding sites. M6b, sensor
rendering and the Python render/viewer API, remains pending.

Demo consolidation update (2026-10-12): WP-PD.18 retires the demo
project home after the Python catalog replaces the C++ app. Its durable
owners and M6b's remaining scope are linked in
[the program README](README.md#demo-consolidation-d31b). The deferred
[changelog drafts](changelog-drafts.md) wait for M6b's consolidation PR;
packet evidence is in [verification.md](verification.md#demo-consolidation-evidence).

## Historical next steps

1. Land #3717 and #3719 (WP-7.0.3b, SDF pose attributes), then publish
   WP-7.0.5 (performance baseline and many-World rows).
2. Open the follow-up packets: the ODE callback exception fix, the
   analytical-IK clone crash, a tokenized release-reference check (the
   regular-expression matcher misreads Markdown inside tokens, unusual
   parameter separators and version suffixes, as its 6.x predecessor did), and
   a DART 7 wall-time dashboard (the DART 6 dashboard now tracks `release-6.*`
   only).
3. Then WP-7.0.6, WP-7.0.7 and the rest of WP-7.0.8 (Python 3.15 wheels), and
   pin `dart7-base` right before the first M1
   PR.

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
