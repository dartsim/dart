# Development Tasks

Working documentation for multi-phase development tasks in DART.

> **Key Rule**: `docs/dev_tasks/` is for **working documentation** during active development.
> When a task completes, the folder is **deleted** (not archived). Key insights go to `docs/onboarding/`.

## When to Create a dev_tasks Folder

**Create `docs/dev_tasks/<task>/` when:**

| Signal                             | Example                                                                |
| ---------------------------------- | ---------------------------------------------------------------------- |
| **Multi-phase work**               | raylib backend: architecture → API design → implementation → migration |
| **Spans multiple sessions**        | Work that will take multiple days/weeks with context handoff           |
| **Requires design decisions**      | New subsystem with tradeoffs to document                               |
| **Has dependencies between steps** | Step 2 blocked until step 1 reviewed                                   |
| **Needs milestone tracking**       | "Phase 1: vertical slice, Phase 2: feature parity"                     |

**Do NOT create for:**

- Single-session bug fixes
- Small refactors (< 1 day)
- Documentation-only changes
- Tasks with clear, linear steps

### Auto-Detection Checklist (for agents)

Before starting a task, ask:

1. Will this take more than one work session? → Consider dev_tasks
2. Are there design decisions that need review? → Create dev_tasks
3. Is there a risk of losing context between sessions? → Create dev_tasks
4. Does this involve API changes affecting multiple subsystems? → Create dev_tasks

## Folder Structure Template

```
docs/dev_tasks/<task>/
├── README.md           # Required: Status, goals, next steps
├── RESUME.md           # Required: Resume prompt for fresh sessions
├── 01-design.md        # Optional: Architecture decisions
├── 02-milestones.md    # Optional: Phase criteria
└── NN-<topic>.md       # Optional: Additional docs as needed
```

### RESUME.md — Session Continuity (IMPORTANT)

**Why this matters**: AI sessions can end unexpectedly (context limits, crashes, user closes session). Without a resume prompt, the next session starts from scratch and may misunderstand where work left off.

**Create `RESUME.md` when you create the task folder. Update it before ending each session.**

````markdown
# Resume: <Task Name>

## Last Session Summary

<2-3 sentences: what was accomplished, what state things are in>

## Current Branch

`<branch-name>` — <status: clean/uncommitted changes/unpushed commits>

## Immediate Next Step

<THE single most important thing to do next — be specific>

## Context That Would Be Lost

- <key insight or decision from this session>
- <file that was being edited and why>
- <blocker or dependency discovered>

## How to Resume

```bash
git checkout <branch>
# Verify state:
git status && git log -3 --oneline
```

Then: <specific instruction, e.g., "Continue implementing X in file Y" or "Run tests to verify fix for Z">
````

**Update discipline:**

- Update `RESUME.md` **before** ending your session
- Update after significant progress (don't wait until "done")
- Keep it current — stale resume prompts are worse than none

### README.md Template

```markdown
# <Task Name> — Dev Task

## Current Status

- [ ] Phase 1: <description>
- [ ] Phase 2: <description>

## Goal

<1-2 sentences: what success looks like>

## Non-Goals (for early phases)

- <thing explicitly out of scope>

## Key Decisions

- <decision>: <rationale>

## Immediate Next Steps

1. <specific action>
2. <specific action>
```

### Examples (Active Tasks)

- `raylib/` - Multi-phase GUI backend replacement (architecture, milestones, migration, testing docs)
- `world_split/` - ECS world separation (design, migration docs)

## Structure

- Each development task should live in its own subdirectory under `docs/dev_tasks/`.
- If you can't find a task folder (e.g., it was renamed or removed after completion), list what exists:

  ```bash
  find docs/dev_tasks -maxdepth 2 -type d -print
  ```

## Documentation Principles

**Task docs should**:

- ✅ Track **current status** and **next steps**
- ✅ Document **key decisions** and **why** (not just what)
- ✅ Point to **code as source of truth**
- ❌ Avoid hardcoded lists (file lists, dependency versions) that become outdated
- ❌ Avoid full history - focus on current state

## Task Completion Checklist (MANDATORY)

> ⚠️ **CRITICAL**: Cleanup happens in the **SAME PR** that completes the task, NOT after merge.
> Leaving orphaned folders is a compliance violation.

**When task is completed, agents MUST:**

1. [ ] **Extract key insights** → Add brief section to existing `docs/onboarding/<relevant>.md`
2. [ ] **Delete the entire folder** → `git rm -r docs/dev_tasks/<task>/`
3. [ ] **Include in completion PR** → Same PR that finishes the implementation

### Why This Matters

- `dev_tasks/` is **working documentation** — it has no value after completion
- Key decisions belong in `docs/onboarding/` where agents will find them
- Orphaned folders confuse future agents and waste context window

**Onboarding docs must stay lean**:

- ❌ Don't create new detailed files for every completed task
- ✅ Add brief sections to existing relevant docs
- ✅ Focus on key design decisions only (2-5 sentences)
- ✅ Point to code for implementation details
- ⚠️ LLMs struggle with bloated documentation - keep it minimal

**What NOT to include in onboarding docs**:

- ❌ **Hardcoded file lists** - Files change, become outdated
- ❌ **Code snippets** - Code evolves, docs won't
- ❌ **Detailed API documentation** - Code comments are source of truth
- ❌ **Step-by-step implementation guides** - Read the actual code
- ❌ **Performance numbers** - These change with optimizations
- ✅ **DO**: High-level design decisions with "why" rationale
- ✅ **DO**: Architectural patterns used (e.g., "hybrid approach", "threshold-based")
- ✅ **DO**: Directory pointers (e.g., "see `dart/math/lcp/pivoting/dantzig/`")

**Remember**: Code is the source of truth. Documentation explains _why_, code shows _how_.

**Before submitting PRs:**

1. Suggested (Unverified): Run `pixi run test-all` - comprehensive test suite
2. Fix any failures before pushing
3. **Important**: If GitHub CI fails but `test-all` passed locally, update `test-all` to catch that failure

**Before committing:**

- Suggested (Unverified): Run `pixi run lint` to catch common issues early
- Update task status in tracker
- No author names or ownership attribution

## Related

- [Main docs](../README.md)
- [Onboarding](../onboarding/README.md)
