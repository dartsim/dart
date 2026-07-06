# Resume: Agent Sim Verification — Follow-up

## Last Session Summary

The core task shipped: two PRs (#3313 → `main`, #3314 → `release-6.20`) merged
2026-07-06, delivering the full text-first + visual agent verification tooling
on both branches, including Codex bot-review responses. This folder holds the
handoff for the remaining follow-up work; nothing here is a blocker.

## Current Branch

`main` (clean) — both PRs merged. DART 6 work happens on a `release-6.20`
checkout/worktree.

## Immediate Next Step

Read [`HANDOFF.md`](HANDOFF.md) fully, then `git checkout main && git pull` and
pick a follow-up (A–E in HANDOFF §5). Lowest-friction first pick: **A (stub
regeneration)** or **B (image A/B round 2)**.

## Context That Would Be Lost

- All shipped code is in merged PRs #3313 / #3314; the durable docs are
  `docs/onboarding/agent-sim-verification.md` and
  `docs/design/agent_sim_verification.md`.
- The operational gotchas in HANDOFF §4 (Filament frame-skip, backend-specific
  goldens, DART 6 dartpy import path, stub drift, no-conda-Xvfb, worktree
  scanner skip, Codex sandbox limits) will save the next agent hours.

## How to Resume

```bash
git checkout main && git pull
git status && git log -3 --oneline   # expect #3313 merge near HEAD
```

Then: read `HANDOFF.md`, invoke `$dart-verify-sim` / the `dart-verify-sim`
skill, and start the chosen follow-up. Verify with `pixi run test-all`,
`pixi run check-lint`, `check-api-boundaries`, `check-ai-commands`.
