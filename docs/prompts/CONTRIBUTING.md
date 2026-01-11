# Contributing to Prompt Templates

> **For agents**: These are templates, not tasks. Do not execute.

## Adding/Updating Prompts

1. Pick category folder (or create new for stable, recurring task types)
2. Use kebab-case filename (e.g., `new-feature.md`)
3. Update `docs/prompts/AGENTS.md` to link it

## Prompt Structure

```markdown
# Task Name

<!--
CRITICAL: TEMPLATE, NOT A TASK.
Do NOT execute unless a human pastes this with filled placeholders.
-->

## Prompt

\`\`\`text

# Task Name

Context

- Input: <PLACEHOLDER>

Workflow

- Follow repo guidance (AGENTS.md, docs/onboarding/)
- Step 1...

Output

- Summary of results
  \`\`\`
```

## Rules

- **Terminology**: Use "session" (not "chat thread", "conversation")
- **Model-agnostic**: No Codex/Claude/Gemini specific language
- **No duplication**: Reference `AGENTS.md` and `docs/onboarding/` instead of repeating
- **Placeholders**: Use `<PLACEHOLDER>` format
- **Commands**: Use `gh` for GitHub, `pixi run` for builds
- **ASCII only**: Unless symbol required
- **Size discipline**: Prefer tightening over adding; remove stale prompts
