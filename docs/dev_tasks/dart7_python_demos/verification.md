# Verification

Each entry records the date, the packet, what changed, the checks that ran,
the review passes, and any gaps. Per [the README](README.md#gates), a packet
is done only after two clean review passes on its final state.

## 2026-10-12: WP-PD.0, decision and project home

- **Change:**
  - D31b in the DART 7 decisions, plus the program README notes on the
    milestone, release path and interim bindings.
  - A Rendering bullet in `docs/design/dart7_python_api.md`.
  - The PLAN-622 GUI-example policy, in the dashboard and in the
    deformable-body project home.
  - This project home.
- **Design evidence:**
  - read-only exploration (six slices plus a completeness critic);
  - a design workflow with four sections, an adversarial verifier per
    section, and an integration critic;
  - an independent Codex design and a Codex review of the plan.

  Every finding that changed the plan was verified against the source at
  `9e4bb4e6f`.
- **Checks:** `pixi run lint` and `pixi run check-lint` passed. The
  check-lint aggregate includes `check-ai-commands` and the semantic
  `check-ai-infra` checks. The pre-commit AI gate and the local-path gate also
  passed.
- **Reviews:** two consecutive clean passes on `25fc9ee65`, each a
  `codex review --base origin/main` with gpt-6.1-sol at ultra effort. Neither
  found an actionable issue; their checks covered whitespace, relative links,
  parity counts, local paths and generated-command sync.
