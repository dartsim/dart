# Local Review Gate

## Objective

Implement the approved pre-push review plan for `release-6.20` and `main`:
two independent clean local reviews for substantive changes, one recorded
independent non-substantive exception, immutable candidate evidence, a fast
cross-tool Git gate, and behavioral review validation.

## Decisions And Scope

- User approved both branches, checked push enforcement, all authoring tools,
  and recorded exceptions on 2026-09-06. This implements the plan following
  investigation of PRs #3484 and #3485.
- One writer implements release first, then adapts main. Read-only reviewers
  use GPT-6 Astra Max with fresh context. Parallel implementation is excluded.
- No C++/dartpy/ABI, installed package, solver, or Gazebo behavior changes.
- User subsequently approved both PR publications and their CI/hosted-review
  cycles, including required fixes, pushes and review requests. That approval
  does not include merging, ready transitions, force-pushes or branch deletion.
- The gate checks supplied review evidence; it cannot prove reviewer honesty
  or correctness. Models, network calls, and builds stay outside the hook.
- Durable policy belongs in `docs/onboarding/ai-reviews.md`, evidence rules in
  `docs/ai/verification.md`, runtime mechanics in `docs/onboarding/ai-tools.md`.

## Acceptance

- Exact outgoing head/base/remote identities and independent completed reports
  determine the gate. Findings survive candidate revisions.
- Real local Git pushes cover initial/update, non-HEAD and multiple refs,
  preserved foreign hooks, stale/missing evidence, and linked worktrees.
- Native Windows CI exercises the gate without the existing POSIX-only test
  module's blanket skip. Linux evidence cannot close that gate.
- Blind archived and contrasting review cases exercise instruction quality.
- Run branch-owned sync, infrastructure, scenario, lint and required full
  gates; distinguish baseline/external failures from passes.
- Two independent clean reviews cover each final branch candidate.

## Current State

The runtime, hook installer, diagnostics, workflow owners and generated adapters
are implemented on both isolated branches. The archive/contrast trial and its
remaining Python API miss are promoted to the review-policy owner's behavioral
replay section. Enforcement regressions exercise actual local Git pushes.

- Initial release candidate: adapter/infrastructure/scenario gates pass; 623 AI tests pass;
  `test-all` passes (158 CTest targets and 271 Python tests); full lint passes.
- Initial main candidate: adapter/infrastructure/scenario/docs-policy gates pass; 665 AI tests
  pass. Full CPU validation passes (229 unit and 81 simulation CTest targets,
  2,015 Python tests with 19 skips). The CUDA `test-all` command passes,
  including eight GPU runtime tests and benchmark smoke checks. Its configured
  profile disables dartpy, GUI and examples; two simulation tests are disabled.
- Published PRs are [#3492](https://github.com/dartsim/dart/pull/3492) for release
  and [#3494](https://github.com/dartsim/dart/pull/3494) for main. The release PR
  was marked ready externally; main remains draft. Preserve their live state.
- Both first hosted rounds completed with six confirmed defect families across
  their initial heads. Independent adverse reports are recorded in both local
  journals. The repairs cover disposition ancestry/author eligibility, opaque
  destination identities, recoverable evidence transactions, installed CLI
  discovery and custom-hook integration. New full validation and clean local
  reviews are required before publishing the repair batch.
- Native Windows ran the initial release review-gate step successfully at
  `3d7ecdb172`; other platform jobs remain in progress. This does not validate
  the changed repair runtime. Finish current-head Windows and hosted acceptance.
- The shared hook is installed; reinstallation follows any runtime update.
  Both doctors detect whether its installed bytes match their branch's source.
  The Git-common review store owns current candidate/report state; check it
  before starting additional review sessions.

This is an active implementation checkpoint, not a task-completion claim.
`RESUME.md` owns recovery and next actions. Native Windows and hosted acceptance
remain in scope. Complete those gates, promote the final durable conclusions,
and remove this folder in the completing PR before its final validation/reviews.

## Changelog decision

- Mode: finalize
- Base evidence: release-6.20 e8f5b9a267fd and main c5030dcab0fb.
- Scope evidence: local gate, installer, review workflow and test changes.
- Decision: entry required; contributor publication behavior changes.
- Target section: DART 6.20 Build; DART 7 Build, Packaging, and Developer Tooling.
- Entry text: Require two independent local reviews before substantive branch
  pushes, with recorded finding dispositions and an evidenced trivial-change
  exception enforced by the installed Git hook.
- PR-body note: Entries now have their branch-specific PR links locally.
- Follow-up: include the links in the validated repair batch.
