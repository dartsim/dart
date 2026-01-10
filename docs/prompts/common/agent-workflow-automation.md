# DART: Agent Workflow (Automation-First)

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: Agent Workflow (Automation-First)

Goal
- Execute tasks with maximum automation and minimal back-and-forth.

Principles
- Default to action: run needed commands directly when permitted.
- Ask questions only when intent is unclear or decisions are ambiguous.
- Keep progress moving; avoid stalling on approvals when not required.

Repo workflow references (dartsim/dart)
- Start at the repo root (usually the current directory).
- Read guidance in this order:
  1) `AGENTS.md` (entry points and constraints)
  2) `docs/**` (workflows and policies)
  3) `CONTRIBUTING.md` (coding and PR rules)

Automation expectations
- If permissions allow, run commands instead of asking for approval.
- If CI monitoring is requested, watch runs without artificial timeouts and iterate on failures until green.
- Do not claim "I'll keep monitoring" unless a blocking watch is actually running; if you must stop, say so and provide the exact command to resume.
- If a tool claims no permission but permissions are available, retry the command directly.
- Keep the loop tight: run -> observe -> fix -> push -> monitor.

When to ask
- Missing critical inputs (goal, scope, acceptance criteria).
- Multiple viable approaches with tradeoffs that affect users.
- Any destructive action not explicitly requested.

Output
- Short status summary.
- Commands executed.
- Open questions (only if required).

Rules
- Use ASCII only.
- Follow repo conventions and entry points; do not invent new ones.
```
