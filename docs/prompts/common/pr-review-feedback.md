# DART: Address PR Review Feedback

> **Prefer command**: Use `/dart-review-pr` in Claude Code or OpenCode.
> This template is for tools without command support.

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Address PR Review Feedback

Context
- PR: <PR_URL>
- PR number: <PR_NUMBER or "infer from PR">
- Review comment(s): <COMMENT_URLS_OR_SUMMARY>
- Branch: <BRANCH or "infer">
- Constraints: <optional>

Workflow
- Start at repo root; read `AGENTS.md`, `CONTRIBUTING.md`, and relevant `docs/onboarding/**`.
- **AI-generated reviews** (usernames ending in `[bot]}` like `chatgpt-codex-connector[bot]`): NEVER reply via `gh pr comment` or any comment. Push fix silently, then `@codex review`. See `docs/onboarding/ai-tools.md`.
- Pull review context via `gh pr view <PR_NUMBER> --comments` and/or the comment links.
- If you need inline comment metadata or to resolve a review thread, use the comment URL with `gh api /repos/<OWNER>/<REPO>/pulls/comments/<COMMENT_ID>`; if thread IDs are required, use GraphQL.
- For each requested change, confirm intent and scope; ask if unclear.
- Apply the smallest change that resolves the feedback.
- Run the smallest relevant `pixi run ...` task, or state why it is skipped.
- Commit and push; update the PR.
- Monitor CI until green if it runs; fix failures before closing the loop.

Output
- Summary of fixes and which comments are resolved.
- Files touched.
- Commands run.
- CI status and how to resume if still running.
```
