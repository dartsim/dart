# DART: Improve Docs

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Continuous Documentation Improvement

Task completed. Capture learnings so future work is faster and higher quality.

## Philosophy

Documentation should **prevent future friction**, not just record what happened.
Ask: "What would have helped me start faster or avoid mistakes?"

## Skip If (Check First)

STOP if ALL are true:
- No new patterns, gotchas, or failure modes discovered
- No workflow improvements identified
- Existing docs already cover everything encountered
- Test-only or routine bugfix with no behavioral insights

## Step 1: Classify Your Learnings

| Learning Type | Example | Route To |
|---------------|---------|----------|
| High-level module guidance | "Use Skeleton::create() factory" | Module `AGENTS.md` (brief) |
| Detailed API pattern | "SoftBodyNode clamps negative damping" | `docs/onboarding/*.md` |
| Cross-cutting workflow | "Always run lint before commit" | `docs/onboarding/*.md` |
| Build/test gotcha | "arm64 CI needs longer timeout" | `docs/onboarding/ci-cd.md` |
| Repeatable workflow | "Dual-PR for bugfixes" | `.claude/commands/` or skill |
| Tool/command tip | "gh run watch for CI monitoring" | Skill or `docs/onboarding/*.md` |
| Process improvement | "This template needs better X" | This file (`improve-docs.md`) |
| Agent compliance failure | "Agent skipped lint despite docs saying to" | Improve doc prominence/structure (see Note) |

> **CRITICAL: AGENTS.md files are POINTER BOARDS, not content repositories.**
> They should contain high-level guidance and pointers to detailed docs.
> Task-specific gotchas, implementation details, or specific file/function references
> belong in `docs/onboarding/*.md`, NOT in AGENTS.md.

> **Note on Agent Compliance Failures**: When an agent fails to follow documented rules,
> the issue is usually **visibility**, not missing content. Fix by: (1) moving the rule
> to a more prominent location (e.g., checklist section), (2) using scannable formatting
> (checkboxes, bold headers), or (3) adding to a "MANDATORY" section. For systematic
> analysis, use `docs/prompts/after-task/audit-agent-compliance.md`.

## Step 2: Locate Target Documentation

### Decision Tree: Where Does This Go?

```

Is this HIGH-LEVEL guidance (concepts, pointers, structure)?
├─ YES → Module's AGENTS.md (keep brief, 1-2 sentences per item)
│
└─ NO → Is this DETAILED content (gotchas, patterns, debugging)?
├─ YES → docs/onboarding/\*.md (NEVER in AGENTS.md)
│ ├─ dynamics.md - Dynamics API patterns
│ ├─ building.md - Build issues, dependencies
│ ├─ testing.md - Test patterns, debugging
│ ├─ ci-cd.md - CI failures, timeouts, caching
│ ├─ contributing.md - PR workflow, review process
│ └─ code-style.md - Formatting, conventions
│
└─ Is this a repeatable workflow that agents should follow?
├─ YES → Consider:
│ ├─ .claude/commands/ - New slash command
│ ├─ .claude/skills/ - Domain knowledge bundle
│ └─ docs/prompts/ - Prompt template
│
└─ NO → Probably skip (see Skip If above)

````

### Documentation Locations Quick Reference

| Location | Purpose | When to Update |
|----------|---------|----------------|
| `AGENTS.md` (root) | Entry point, pointers | Rarely (structure changes only) |
| `dart/*/AGENTS.md` | Module-specific pointers | High-level concepts, links to detailed docs |
| `python/AGENTS.md` | Python binding guidance | Python-specific patterns |
| `tests/AGENTS.md` | Test suite guidance | Testing patterns, fixtures |
| `docs/onboarding/*.md` | Detailed developer guides | Workflows, architecture, deep dives |
| `.claude/commands/` | Slash commands | New repeatable workflows |
| `.claude/skills/` | On-demand knowledge | Domain expertise bundles |
| `docs/prompts/` | Prompt templates | Session-starting prompts (follow `docs/prompts/CONTRIBUTING.md`, update index) |

> **Note**: Multiple AI tools use synced folders (`.claude/`, `.opencode/`, `.codex/`).
> See root `AGENTS.md` → "Tool Compatibility" for the full mapping.

**Meta-improvement**: If this template itself is missing guidance or has unclear routing,
update `docs/prompts/after-task/improve-docs.md` directly. The process should improve itself.

## Step 3: Search Before Adding

Before adding new content:
1. Search existing docs for related content
2. Check if updating existing content is better than adding new
3. Consolidate scattered related content when found

Commands:
```bash
# Find all AGENTS.md files
find . -name "AGENTS.md" -type f

# Search for existing coverage
grep -r "your keyword" docs/ .claude/ --include="*.md"
````

## Step 4: Apply Changes

### Improvement Modes (pick what fits)

- **Verify**: Confirm existing docs still accurate
- **Add**: New pattern not yet documented (last resort)
- **Update**: Enhance existing section with new insight
- **Remove**: Delete outdated or redundant content
- **Consolidate**: Merge scattered related content
- **Restructure**: Improve navigation/organization

**Bias**: Update/remove/consolidate >> Add new content

### Quality Rules

- No ephemeral IDs (branches, PRs, commits, usernames)
- Write general patterns, not task-specific chronicles
- 1-3 sentences per topic is ideal
- Code examples only when non-obvious
- Point to code as source of truth, not hardcoded lists
- Check if code changes invalidated existing docs

### AGENTS.md Style (CRITICAL)

**AGENTS.md files are POINTER BOARDS, not content repositories.**

Structure: Overview, Key Concepts, Code Patterns, Testing, See Also.
Length: Each section should be 3-5 bullet points max.

**What BELONGS in AGENTS.md:**

- High-level module purpose (1-2 sentences)
- Key abstractions/concepts (bullet list of names + one-liner)
- Common entry points (factory methods, main classes)
- Pointers to detailed docs (`@docs/onboarding/*.md`)

**What does NOT belong in AGENTS.md:**

- Task-specific gotchas from a particular fix
- References to specific file:function locations
- Detailed implementation patterns
- Debugging tips for specific failures
- Anything that references a PR, commit, or specific bug

**Anti-pattern example (DO NOT DO THIS):**

```markdown
## Gotchas

- **assertFiniteState in setters**: When fixing bug X, check Y before Z.
  See `generic_joint.hpp:updateAccelerationDynamic()` for the pattern.
```

**Correct pattern:**

```markdown
## Code Patterns

- Use factory methods (`Skeleton::create()`) not constructors
- Joint state modifications trigger validation in debug builds

## See Also

- @docs/onboarding/dynamics.md - Detailed API patterns and gotchas
```

For detailed content, create or update `docs/onboarding/*.md`.

## Output

Provide:

1. Summary of learnings captured
2. Files modified (or "No changes needed" with reason)
3. Any follow-up recommendations

```

```
