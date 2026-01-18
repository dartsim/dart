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
| Module-specific pattern | "SoftBodyNode clamps negative damping" | Module `AGENTS.md` |
| Cross-cutting workflow | "Always run lint before commit" | `docs/onboarding/*.md` |
| Build/test gotcha | "arm64 CI needs longer timeout" | `docs/onboarding/ci-cd.md` |
| API usage pattern | "Use Skeleton::create(), not constructor" | Module `AGENTS.md` + `docs/onboarding/*.md` |
| Repeatable workflow | "Dual-PR for bugfixes" | `.claude/commands/` or skill |
| Tool/command tip | "gh run watch for CI monitoring" | Skill or `docs/onboarding/*.md` |
| Process improvement | "This template needs better X" | This file (`improve-docs.md`) |

## Step 2: Locate Target Documentation

### Decision Tree: Where Does This Go?

```
Is this about a SPECIFIC module (dart/dynamics, dart/collision, etc.)?
├─ YES → Check module's AGENTS.md first
│        ├─ Add if it's a key pattern, gotcha, or testing tip
│        └─ Point to docs/onboarding/*.md for detailed explanation
│
└─ NO → Is this a cross-cutting concern (build, test, CI, workflow)?
        ├─ YES → docs/onboarding/*.md
        │        ├─ building.md - Build issues, dependencies
        │        ├─ testing.md - Test patterns, debugging
        │        ├─ ci-cd.md - CI failures, timeouts, caching
        │        ├─ contributing.md - PR workflow, review process
        │        └─ code-style.md - Formatting, conventions
        │
        └─ Is this a repeatable workflow that agents should follow?
                ├─ YES → Consider:
                │        ├─ .claude/commands/ - New slash command
                │        ├─ .claude/skills/ - Domain knowledge bundle
                │        └─ docs/prompts/ - Prompt template
                │
                └─ NO → Probably skip (see Skip If above)
```

### Documentation Locations Quick Reference

| Location | Purpose | When to Update |
|----------|---------|----------------|
| `AGENTS.md` (root) | Entry point, pointers | Rarely (structure changes only) |
| `dart/*/AGENTS.md` | Module-specific guidance | Patterns, gotchas for that module |
| `python/AGENTS.md` | Python binding guidance | Python-specific patterns |
| `tests/AGENTS.md` | Test suite guidance | Testing patterns, fixtures |
| `docs/onboarding/*.md` | Detailed developer guides | Workflows, architecture, deep dives |
| `.claude/commands/` | Slash commands | New repeatable workflows |
| `.claude/skills/` | On-demand knowledge | Domain expertise bundles |
| `docs/prompts/` | Prompt templates | Session-starting prompts |

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
```

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

### AGENTS.md Style
Keep concise: Overview, Key Concepts, Code Patterns, Testing, See Also.
For detailed content, point to `docs/onboarding/*.md`.

## Output

Provide:
1. Summary of learnings captured
2. Files modified (or "No changes needed" with reason)
3. Any follow-up recommendations
```
