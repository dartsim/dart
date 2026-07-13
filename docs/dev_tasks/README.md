# Development Tasks

Working documentation for multi-phase development tasks in DART.

> **Key Rule**: `docs/dev_tasks/` is for **working documentation** during active development.
> When a task completes, the folder is **deleted** (not archived). Key
> insights move to the durable owner selected by
> [`docs/information-architecture.md`](../information-architecture.md).

## When to Create a dev_tasks Folder

**Create `docs/dev_tasks/<task>/` when:**

| Signal                             | Example                                                       |
| ---------------------------------- | ------------------------------------------------------------- |
| **Multi-phase work**               | Filament GUI: MVP → architecture → implementation → migration |
| **Spans multiple sessions**        | Work that will take multiple days/weeks with context handoff  |
| **Requires design decisions**      | New subsystem with tradeoffs to document                      |
| **Has dependencies between steps** | Step 2 blocked until step 1 reviewed                          |
| **Needs milestone tracking**       | "Phase 1: vertical slice, Phase 2: feature parity"            |

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

### Autonomous Project Extension

Use `dart-ultrawork` for large, team-scale, multi-session, or explicitly
autonomous work. It uses this same `docs/dev_tasks/<task>/` folder as the
project home for both DART 7 and DART 6 maintenance tasks.

For autonomous projects, keep the required files above and add sidecars when
they make resumability or evidence clearer:

| File              | Required contents                                                                                                                          |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| `README.md`       | north star, final deliverable, acceptance criteria, scope, non-goals, constraints, risks, current milestone, blockers, next actions, gates |
| `RESUME.md`       | current branch/worktree state, immediate next step, risks, recovery notes, verification commands                                           |
| `decisions.md`    | date, decision, context, options considered, evidence, tradeoffs, result, revisit trigger                                                  |
| `verification.md` | date, chunk or milestone, what changed, checks run, review passes, GUI/demo evidence when relevant, results, known gaps, follow-up         |
| `progress-log.md` | chronological record of meaningful completed work and links to evidence                                                                    |

Do not create a separate generic project-home tree outside `docs/dev_tasks/`
unless a task-specific owner doc explicitly requires it. Before any session
ends, update the project-home docs enough that a fresh agent can continue
without hidden chat history.

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
- Treat branch/status sections as snapshots from the last update. Before acting
  on an existing `RESUME.md`, verify the current checkout with `git status` and
  the task's `README.md`/durable design docs. If the code has landed or the
  branch instructions are historical, add a short "Current Reality" note above
  the older branch-local instructions instead of deleting useful archaeology.

### README.md Template

```markdown
# <Task Name> — Dev Task

## Current Status

- [ ] Phase 1: <description>
- [ ] Phase 2: <description>

## Goal

<1-2 sentences: what success looks like>

## Specification Intake

- **Value:** <user, maintainer, research, or release value>
- **Scope:** <files/modules/surfaces expected to change>
- **Assumptions:** <defaults chosen from current evidence>
- **Traceability:** <issue, plan, design doc, PR, benchmark packet, or prompt>

## Non-Goals (for early phases)

- <thing explicitly out of scope>

## Acceptance Evidence

- <test, benchmark, doc update, visual artifact, or command output that proves
  success>
- <for behavior-bearing physics/simulation work: self-contained GUI or demos-app
  artifact, runnable command, and visual inspection or capture evidence>
- <review evidence: at least two clean independent or role-separated passes on
  the current post-fix state>

## Gates

- `pixi run <task>` — <why this covers the objective>

## Open Decisions

- None, or link to an owner-local `Decision needed` block.

## Key Decisions

- <decision>: <rationale>

## Immediate Next Steps

1. <specific action>
2. <specific action>
```

### Current Active Tasks

Current active tasks are the subdirectories under `docs/dev_tasks/`. Do not
maintain a hardcoded inventory in this README because task folders are removed
when their work completes. To inspect the current set:

```bash
find docs/dev_tasks -maxdepth 2 -type d -print
```

## Structure

- Each development task should live in its own subdirectory under `docs/dev_tasks/`.
- If you can't find a task folder, use the current-inventory command above
  instead of relying on a hardcoded list.

## Documentation Principles

**Task docs should**:

- ✅ Track **current status** and **next steps**
- ✅ Record **specification intake** for multi-session/design/API/research work:
  value, scope, assumptions, traceability, acceptance evidence, gates, and open
  decisions
- ✅ Document **key decisions** and **why** (not just what)
- ✅ Point to **code as source of truth**
- ❌ Avoid hardcoded lists (file lists, dependency versions) that become outdated
- ❌ Avoid full history - focus on current state

## Task Completion Checklist (MANDATORY)

> ⚠️ **CRITICAL**: Cleanup happens in the **SAME PR** that completes the task, NOT after merge.
> Leaving orphaned folders is a compliance violation.

**When a task is completed — or all of its still-doable work is done — agents MUST:**

1. [ ] **Relocate remaining / deferred / blocked work** → If plans remain that are
       blocked by a hard constraint or deferred by design, move them to a durable
       home (a `docs/design/` doc, a `docs/plans/` entry, or the dashboard) so they
       survive folder deletion. If such work genuinely cannot be completed now,
       **ask the human before retiring the folder** so it can be revisited later.
       Open-ended checklist items such as "continue expanding" need an explicit
       stopping condition or scope cap before cleanup; record the achieved scope
       and move further expansion to the durable follow-up home.
2. [ ] **Extract key insights** → Add a brief section to the existing durable
       owner selected by `docs/information-architecture.md`, usually
       `docs/onboarding/`, `docs/design/`, `docs/plans/`, or
       `docs/readthedocs/`.
3. [ ] **Delete the entire folder** → `git rm -r docs/dev_tasks/<task>/`
4. [ ] **Include in completion PR** → Same PR that finishes the implementation

### Why This Matters

- `dev_tasks/` is **working documentation** — it has no value after completion
- Key decisions belong in the durable owner selected by
  `docs/information-architecture.md`, where agents will find them
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

1. **Required**: Run `pixi run test-all` - the authoritative pre-PR gate per
   `AGENTS.md`. On Linux hosts with a visible NVIDIA CUDA runtime, also run
   `pixi run -e cuda test-all`.
2. Fix any failures before pushing
3. **Important**: If GitHub CI fails but `test-all` passed locally, update `test-all` to catch that failure

**Before committing:**

- **Required**: Run `pixi run lint` before every commit (MANDATORY per
  `AGENTS.md`). `pixi run install-hooks` adds a fast staged whitespace and
  AI-infrastructure pre-commit gate; it does not replace the full lint run.
- Update task status in tracker
- No author names or ownership attribution

## Related

- [Main docs](../README.md)
- [Onboarding](../onboarding/README.md)
