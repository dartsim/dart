# Contributing to Prompt Templates

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This file describes how to ADD or UPDATE prompt templates.
These are NOT active tasks. Do NOT execute any prompt unless a human
explicitly pastes it into a new session with filled placeholders.
=======================================
-->

Thanks for improving the prompt library.

## Add or Update a Prompt

1. Pick the right category folder (or create a new one if it is a stable, recurring task type).
2. Add a new prompt file using kebab-case (e.g., `new-feature.md`).
3. Update the category `README.md` to link the new file.

## Prompt File Structure

Each prompt file should have:

1. A short title as H1
2. A `## Prompt` section with a fenced `text` code block
3. Placeholders like `<ISSUE>` or `<PR_URL>`

Example:

```markdown
# My Task Prompt

## Prompt

\`\`\`text

# My Task

Context

- Issue: <ISSUE_URL>
- Branch: <BRANCH>

Workflow

- Step 1...
- Step 2...

Output

- Summary of what was done.
  \`\`\`
```

## Style Guidelines

- Use ASCII only unless a prompt requires a specific Unicode symbol.
- Keep prompts short, task-scoped, and self-contained.
- Use explicit constraints; don't assume context from other prompts.
- Avoid hard-coded identifiers:
  - No run IDs, commit hashes, or timestamps
  - No usernames or machine-specific paths (e.g., `/home/...`, `/Users/...`)
  - Use repo-relative paths or placeholders instead
- Prefer `gh` commands for GitHub operations (issues, PRs, runs, API).
- Reference `pixi run ...` tasks for build/test operations.

## Size Discipline

- Prefer small, task-scoped prompts over all-purpose mega prompts.
- Prefer tightening or merging over adding new prompts.
- Remove stale, duplicative, or low-signal prompts when a tighter version exists.
- Avoid duplicating content across categories; link instead when reasonable.

## Testing Prompts

Before committing a new or updated prompt:

1. Use it in a real session to verify it works as intended.
2. Check that placeholders are clear and complete.
3. Ensure the prompt follows repo conventions (`AGENTS.md`, `CONTRIBUTING.md`).
