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
- GitHub publication, comments, review requests, and merges still require
  their existing explicit authorization. No such mutation has been requested.
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

- Release: adapter/infrastructure/scenario gates pass; 611 AI tests pass;
  `test-all` passes (158 CTest targets and 271 Python tests); full lint passes.
- Main: adapter/infrastructure/scenario/docs-policy gates pass; 641 AI tests
  pass; full lint passes. Full CPU and CUDA validation are in progress.
- Native Windows acceptance and current-head hosted review await approved
  draft publication. They are unexecuted, not passed.
- The shared hook is installed and both doctors detect its exact runtime.
  The Git-common review store owns current candidate/report state; check it
  before starting additional review sessions.

This is an active implementation checkpoint, not a task-completion claim.
`RESUME.md` owns recovery and next actions. Native Windows and hosted acceptance
remain in scope. Complete those gates, promote the final durable conclusions,
and remove this folder in the completing PR before its final validation/reviews.

## Changelog decision

- Mode: draft
- Base evidence: release-6.20 e8f5b9a267fd and main fda07ac56715.
- Scope evidence: local gate, installer, review workflow and test changes.
- Decision: entry required; contributor publication behavior changes.
- Target section: DART 6.20 Build; DART 7 Build, Packaging, and Developer Tooling.
- Entry text: Require two independent local reviews before substantive branch
  pushes, with recorded finding dispositions and an evidenced trivial-change
  exception enforced by the installed Git hook.
- PR-body note: Draft links pending publication.
- Follow-up: add each branch's PR link after approved PR creation.
