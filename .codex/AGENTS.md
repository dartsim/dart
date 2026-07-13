# Codex Project Runtime

Files here are maintained sources, not generated adapters.

- Do not pin a model; specialist profiles inherit the parent session model.
- Keep project agents read-only, bounded, and explicit about inputs/outputs.
- Keep hooks deterministic, noninteractive, network-free, and under the tested
  runtime budget.
- Do not weaken user sandbox, approval, or GitHub-mutation boundaries.

Run `pixi run check-ai-infra` and `pixi run test-ai-infra` after changes.
