# DART: Mechanical Refactor / Modernization

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Mechanical Refactor / Modernization

Goal
- Perform a mechanical refactor or modernization with no behavior change.

Context
- Scope: <FILES_OR_AREAS>
- Target branch: <TARGET_BRANCH or origin/main>
- Standard or API target (optional): <CXX_STANDARD or API_TARGET>
- Constraints: <optional>

Workflow
- Read `AGENTS.md`, `CONTRIBUTING.md`, and relevant `docs/**` guidance.
- Create a new branch from the latest <TARGET_BRANCH>.
- Prefer automated or scriptable edits when possible; keep diffs mechanical.
- Avoid drive-by refactors, feature work, or behavior changes.
- If reorganizing directories, update CMake, pixi tasks, and index docs accordingly.
- Keep commits focused by area or transformation type.
- Run the smallest relevant checks per repo guidance; call out anything not run.
- Open a PR and monitor CI if requested.

Output
- Summary of transformations and scope.
- Files touched.
- Tests run (or not run).
- PR URL (if created).
```
