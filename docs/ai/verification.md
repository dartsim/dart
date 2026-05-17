# AI Verification

AI work is complete only when the requested outcome is mapped to concrete
evidence. Passing a broad verifier is useful only when it covers the actual
requirements.

## Completion Audit

Before finalizing substantial AI-assisted work:

1. Restate the objective as concrete deliverables.
2. Map each explicit request, file, command, gate, and deliverable to evidence.
3. Inspect the actual files, command output, review state, or artifacts.
4. Identify missing, weakly verified, or blocked requirements.
5. Continue working until all required items are satisfied or a real blocker
   remains.

## Gate Selection

| Change type         | Required gates                                                                                                                                                     |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| AI docs or adapters | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run sync-ai-commands`, `pixi run check-ai-commands`, `pixi run check-docs-policy`, `pixi run check-lint-spell` |
| Docs only           | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run check-docs-policy`, `pixi run check-lint-spell`                                                            |
| C++ code            | `pixi run lint`, `pixi run build`, focused tests or `pixi run test-unit`                                                                                           |
| Python bindings     | `pixi run lint`, `pixi run build`, `pixi run test-py`                                                                                                              |
| IO/model parsing    | `pixi run lint`, focused parser tests, relevant examples if affected                                                                                               |
| CI workflow         | local reproduction when possible, `pixi run check-lint`, relevant build/test gate                                                                                  |
| Release work        | release-management docs, changelog/version checks, target branch gates                                                                                             |

Before any commit, run `pixi run lint` as required by `AGENTS.md`.

## AI Infrastructure Evidence

For substantial changes to AI-facing docs, commands, skills, generated
adapters, planning workflows, or agent rules, run the principle audit in
`docs/ai/principles.md` and record the result as evidence. This file owns gate
selection and evidence mapping; `docs/ai/principles.md` owns the manual audit
questions; `docs/ai/components.md` owns the exact structural checks performed
by `pixi run check-ai-commands`.

## Review Safety Evidence

When review feedback comes from an AI bot account:

- Do not reply inline to the bot.
- Verify the claim with code inspection or tests.
- Add or update tests when a false positive should be permanently refuted.
- Do not push, resolve threads, comment, or re-trigger review without explicit
  maintainer/user approval.

The final response should state which of those actions were local-only and which
external mutations, if any, were explicitly approved.
