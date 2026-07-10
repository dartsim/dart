# Fable 5 Visual Verification Workflow — Dev Task

## Current Status

- [x] Phase 0: Research merged DART 6/7 visual-reasoning PR lineage → `03-pr-research.md`
- [x] Phase 1: Audit current camera/debug/capture tooling → `02-design.md`
- [x] Phase 2: Design → `02-design.md`
- [x] Phase 3: Implementation (offscreen DebugScene channel, `_debug_layers`, `_view_quality`, agent-capture/image-compose/evidence-select/evidence-publish)
- [x] Phase 4: Tests / reproducible scenarios (35 Python tests + 1 C++ test + capture sidecars)
- [x] Phase 5: Merged-PR retrospective (#2984) → `04-retrospective-2984.md`, `capture_2984.py`
- [x] Phase 6: Docs + limitations → `05-limitations-next-steps.md`; cleanup of this folder happens in the completing PR

## Goal

Extend DART's agent simulation-verification stack so a headless agent can
actively control the camera (and detect/improve inadequate views), render
configurable debug layers beyond the plain GUI image, select a small
non-redundant evidence set tied to explicit claims, and publish that media
GitHub-hosted in PR bodies — validated by re-verifying a representative merged
PR that lacked visual evidence.

## Specification Intake

- **Value:** AI agents verifying 3D/physics changes today are limited to fixed
  bounds-fit captures with no internal-state overlays, no view-quality
  feedback, and no PR publication path; reviewers get weaker evidence.
- **Scope:** `dart/gui/` (offscreen debug overlay pass), `python/dartpy/`
  (render bridge + new evidence helpers), `scripts/` (view quality, capture,
  evidence selection, publication), `docs/onboarding/agent-sim-verification.md`,
  `.claude/skills/dart-verify-sim/`, tests under `python/tests/unit/gui/` and
  script tests.
- **Assumptions:** GitHub-hosted media without web-editor access uses a
  documented `gh`-scriptable backend (release-asset or gist) plus the existing
  manual user-attachments flow; PR-only media never lands in git history.
- **Traceability:** Fable 5 goal brief (session); PR lineage #2690 → #2984 →
  #2992 → #3063/#3084/#3090 → #3304/#3313/#3314/#3320.

## Non-Goals (for early phases)

- New renderer features beyond overlay routing (no new Filament materials).
- VLM/network dependencies in default gates.
- Rewriting merged-PR history; the retrospective is additive/demo-only.

## Acceptance Evidence

- Automated tests for view-quality assessment, adaptive viewpoint selection,
  debug-layer composition, evidence ranking, and publication manifest.
- Reproducible capture scenarios with deterministic settings.
- A merged-PR retrospective packet: original PR verification gap, new
  camera/debug captures, claims-to-artifact mapping, improved PR-body section.
- `pixi run lint` + relevant unit/py tests green.

## Gates

- `pixi run lint` — formatting gate (mandatory).
- `pixi run test-py` / targeted pytest — new Python tooling behavior.
- `pixi run build` — C++ offscreen debug overlay change compiles + binds.

## Open Decisions

- GitHub media backend for automated upload (release asset vs gist vs manual
  user-attachments); see decisions.md.

## Key Decisions

- See `decisions.md`.

## Immediate Next Steps

1. Complete design doc (02-design.md) from research + audit findings.
2. Implement offscreen `DebugScene` pass (C++) and Python view-quality /
   adaptive-viewpoint / evidence tooling.
