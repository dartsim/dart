---
type: ai-principles
owner: self
---

# AI Principles

This compact file is loaded for every DART agent session. It owns DART's
AI-infra axioms. Canonical AI-facing terms live in
`docs/ai/terminology.md`; detailed gates live in `docs/ai/verification.md`;
generated surface checks live in `docs/ai/components.md`; tool compatibility
lives in `docs/onboarding/ai-tools.md`.

Keep this file short. It spends always-loaded agent context, so move examples,
procedures, and compatibility detail to the owner docs above.

## Axioms

1. **Repo-wide context guides focused changes.** Before substantial or
   consequential work, read enough of the north star, owner docs, affected
   modules and call paths, current plan state, and verification bar to
   understand the real invariant. Use that context to choose the smallest
   coherent change that moves DART toward the north star, not an ad hoc local
   patch or special case. Keep context proportionate for tiny fixes.
2. **Assumptions and unknowns must be surfaced.** Do not silently choose
   between materially different interpretations: state the assumption, ask when
   the decision is consequential, and record the evidence when the repo answers
   it. Before large or ambiguous work, also hunt for the unknowns you have not
   named yet and surface them up front rather than mid-implementation;
   `docs/ai/orchestration.md` owns the methods for doing so.
3. **Simplicity is a requirement.** Solve the current DART problem. Do not add
   speculative flexibility, hierarchy, abstraction, or configuration unless it
   removes real complexity or matches an established DART pattern. Rely on
   model/tool behavior shown to work; retain instructions for DART-specific
   constraints or observed gaps. Removing instructions must preserve outcome
   quality and task completeness; `docs/ai/components.md` owns the audit method.
4. **Changes must be surgical.** Every changed line should trace to the user
   request, a documented rule, or a verification finding. Preserve unrelated
   user edits and clean up only what the current change created.
5. **Single source of truth prevents bloat.** Mutable state gets one owner.
   Prefer links over copied fields. If two docs need the same fast-changing
   fact, pick one owner and make the other a pointer. Generated files must name
   their source.
6. **Goals need evidence.** Turn broad requests into concrete deliverables and
   gates. Work is done only when files, generated artifacts, command output,
   review findings, or PR state directly support the outcome.
7. **Decisions need proportionate evidence.** Before consequential choices,
   define the evidence needed to avoid likely false positives and false
   negatives. Use code inspection, logs, focused tests, benchmarks, source or
   literature research, GUI/visual evidence, or prototypes according to the
   claim. Do not settle major design or workflow choices from intuition when
   direct evidence is practical.
8. **Public paths stay portable.** AI workflows can accelerate work, but every
   DART workflow must map to tracked docs and `pixi run ...` commands usable
   without a specific AI tool.
9. **Shared state needs approval.** Local inspection, edits, and verification
   are allowed when the task calls for them. Pushes, PR updates, comments,
   review-thread changes, CI re-triggers, merges, and branch deletion require
   explicit maintainer/user approval. Authorization persists for its stated
   action, target, and scope across workflow steps; ask only for missing or
   changed authority. Approval for one action does not authorize adjacent
   actions, other targets, or a broader scope.
10. **Failures get root-caused, not hidden.** For unexpected in-scope failures,
    reproduce the smallest failing case and fix the cause with regression
    coverage. Preserve the real invariant at the right owner doc or module
    boundary; do not suppress logs, loosen checks, skip cases, or route around
    symptoms. For pre-existing or external failures, classify them with evidence
    and report or park them without weakening gates.

## Principle Audit

Before finalizing substantial work, check the change against these axioms and
the completion audit in `docs/ai/verification.md`. Record the result and any
unresolved exception in the final response or PR Testing section. Use existing
task evidence; do not create a separate audit log.
