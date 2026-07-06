# Agent Sim Verification — Follow-up (Dev Task)

## Current Status

- [x] Core task shipped: PR #3313 → `main` and PR #3314 → `release-6.20`
      merged 2026-07-06 (agent 3D-scene/physics verification tooling on both
      branches).
- [ ] A. dartpy stub regeneration (deferred housekeeping)
- [ ] B. Image A/B round 2 (multi-view + annotations)
- [ ] C. Adopt verdict/golden gates in solver plans
- [ ] D. Scene-dump shape coverage + scene-diff verdict
- [ ] E. VLM-in-the-loop verification (research)

## Goal

Land the remaining follow-ups (A–E) that extend/harden the shipped verification
tooling, without regressing the merged surface.

## The handoff

**Read [`HANDOFF.md`](HANDOFF.md) first** — it is the full self-contained resume
prompt (shipped inventory, key decisions + evidence, operational gotchas, and
each follow-up with its next step and acceptance evidence).

## Durable homes (source of truth)

- `docs/onboarding/agent-sim-verification.md` — the how-to.
- `docs/design/agent_sim_verification.md` — the research + design record.
- `.claude/skills/dart-verify-sim/SKILL.md` — the consolidated skill.

## Cleanup

Delete this folder in the PR that closes items A–E, per
`docs/dev_tasks/README.md`.
