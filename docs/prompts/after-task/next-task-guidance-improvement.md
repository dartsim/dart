# After Finishing a Task - Next-Task Guidance Improvement

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# After Finishing a Task

We have just completed a task together in this same chat thread, and it is now finished (no further product/code changes are needed). Now do a "next-task guidance improvement" pass: update repo documentation and guidance so a fresh agent run on a future task will be faster and less error-prone, based ONLY on what we actually learned in this thread.

## Scope (Hard Rules)

- Source of truth is this chat thread + current repo contents. If something isn't explicitly evidenced, treat it as unknown.
- Do NOT resume work on the completed task and do NOT change product/code for it; docs/guidance only.
- Prefer existing workflows/entry points; use `pixi run ...` when that's what the repo uses.
- Keep `AGENTS.md` concise (pointers only). Place details in `docs/` or the relevant component README.
- Follow `CONTRIBUTING.md` conventions and existing doc style.
- Do NOT run tests/builds/benchmarks or attempt verification now. This pass is documentation-only.
- If you mention commands in docs, they must be labeled "Suggested (Unverified)" only.
- Do not include "Example (Used in this task)" blocks in docs.
- If you edit a section that already contains unlabeled commands, either:
  - leave them untouched (if you aren't changing that section), or
  - add labels only when provenance is known from this thread and they can be phrased as Suggested (Unverified).

## IDE Context Sanity

- If IDE context/open tabs reference files or directories, first confirm they exist in the current checkout before acting on them.
- If they don't exist, treat it as a context mismatch (unknown) and document a general "gotcha" (e.g., "verify paths before assuming").

## Generality Guardrails

- Default to GENERAL, reusable guidance; task-specific details belong only in the final report, not docs.
- Do not put ephemeral identifiers in docs (branch names, PR/issue numbers, run/job IDs, commit hashes, timestamps, usernames, machine-specific paths).
- Avoid file-by-file starting lists unless a file is a stable entrypoint.

## Doc Hygiene / Size Discipline

- Prefer replacing/condensing over appending; avoid growing docs when existing content can be tightened.
- If you add new guidance, remove or merge overlapping/duplicative guidance in the same doc.
- Remove stale, redundant, or low-signal content when it is clearly duplicative or contradicted by this thread or repo state.
- Keep command lists minimal and high-signal; remove weaker or redundant commands if you add stronger ones.
- If a doc is long, favor a short top summary plus links to deeper docs over long inline detail.

## What to Extract (from THIS thread only)

1. Task recap (2-5 sentences, general terms).
2. How we worked (repeatable playbook).
3. How to iterate next time (commands + success signals).
4. Gotchas (general failure modes).
5. Next-time accelerators (high-leverage tips).

## Where to Update

1. Prefer updating existing docs:
   - `docs/onboarding/ci-cd.md`
   - `docs/onboarding/build-system.md`
   - `docs/onboarding/building.md`
   - If component-specific, update that component's README/docs instead.
2. Improve discoverability when needed:
   - Add "Start here / If you're doing X, read Y" pointers in entry locations (repo root README, `AGENTS.md`, onboarding README).
   - If a doc is long, add a tight top-of-file summary + short "common tasks" section.
3. Update `AGENTS.md` only if discoverability is still poor (1-3 bullets max).

## Implementation Requirements

- Produce a concrete patch: minimal doc edits scoped to what we learned.
- If the completed task included merged PRs or pushed commits that require it, update `CHANGELOG.md` per repo policy.
- Add or update (do not duplicate) the following:
  - "Start here next time" navigation hints (general, stable entrypoints).
  - "Suggested (Unverified)" commands only when helpful, labeled clearly.
  - Common failure modes and how they were resolved (short, general).
  - A Fast iteration loop section (smallest repeatable cycle).
- Actively prune or consolidate to keep docs lean; net doc growth should be minimal or negative when possible.
- Do not add "Example (Used in this task)" blocks anywhere in docs.
- If docs already contain "Used in this task" blocks, remove them.

## Parallelism Guidance (Avoid Env-Specific Hardcoding)

- Never include fixed core counts (e.g., 12 or 16) anywhere in docs.
- Use a general rule of thumb: ~2/3 of logical cores.
- Linux example should compute a value via `$(nproc)` (Suggested (Unverified) only).

## Deliverable Format (Concise)

- Summary bullets (what you updated and why).
- Edited file list (paths).
- Any "Suggested (Unverified)" guidance called out explicitly.
- Short note on guidance usability:
  - What wasn't referenced / wasn't effective,
  - What you changed to make it more findable/skimmable,
  - What you pruned or consolidated to prevent bloat.
```
