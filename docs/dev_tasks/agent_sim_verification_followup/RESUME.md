# Resume: Agent Sim Verification — Follow-up

## Last Session Summary

The core task shipped: two PRs (#3313 → `main`, #3314 → `release-6.20`) merged
2026-07-06, delivering the full text-first + visual agent verification tooling
on both branches, including Codex bot-review responses. The follow-up branch has
now completed A, C, D, and E locally: committed dartpy stubs cover the new
verification symbols, PLAN-083 has a concrete visual-gate template, scene dumps
cover the remaining named shape categories defensively, `pixi run scene-diff`
exists, and `pixi run verification-bundle` packages text/image evidence for an
external VLM or reviewer call. Item B is only partially complete:
`pixi run image-ab-study`, `pixi run image-ab-round2`, and the durable protocol
are in place, but real blind-judge rows are still needed before recording
measured detection deltas.

## Current Branch

`docs/agent-sim-verification-followup-handoff` — contains the local follow-up
implementation for A/C/D/E and the B reducer/protocol. It may be ahead of
`origin/docs/agent-sim-verification-followup-handoff` until pushed. DART 6 work
happens on a `release-6.20` checkout/worktree if needed.

## Immediate Next Step

Run `pixi run image-ab-round2 -- generate --out <dir>`, collect or arrange the
real round-2 blind-judge rows for B without exposing `answer_key.json`, then
score them with `pixi run image-ab-round2 -- score ...`. Do not delete this
folder until B has measured detection deltas and the default-capture
recommendation is either updated or explicitly reaffirmed in the durable docs.

## Context That Would Be Lost

- All shipped code is in merged PRs #3313 / #3314; the durable docs are
  `docs/onboarding/agent-sim-verification.md` and
  `docs/design/agent_sim_verification.md`.
- The operational gotchas in HANDOFF §4 (Filament frame-skip, backend-specific
  goldens, DART 6 dartpy import path, stub drift, no-conda-Xvfb, worktree
  scanner skip, Codex sandbox limits) will save the next agent hours.
- Fixture rows in `python/tests/unit/test_image_ab_study.py` prove reducer math
  only; they are not B's measured study evidence.

## How to Resume

```bash
git checkout docs/agent-sim-verification-followup-handoff
git status && git log -3 --oneline
```

Then: read `HANDOFF.md`, invoke `$dart-verify-sim` / the `dart-verify-sim`
skill, and continue B. Verify the current local helper work with focused pytest
for `test_scene_dump.py`, `test_scene_diff.py`, `test_image_ab_study.py`, and
`test_verification_bundle.py`, plus `pixi run sync-ai-commands`,
`pixi run check-ai-commands`, and `pixi run lint` before any commit.
