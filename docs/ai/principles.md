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

These principles are informed by common coding-agent and code-review failure
modes: durable repo instructions, scoped context, verification loops,
independent review, root-cause analysis, and whole-system code health. This is
DART-owned wording and policy.

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
   removes real complexity or matches an established DART pattern.
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
   review-thread changes, CI re-triggers, and merges require explicit
   maintainer/user approval.
10. **Failures get root-caused, not hidden.** For unexpected in-scope failures,
    reproduce the smallest failing case and fix the cause with regression
    coverage. Preserve the real invariant at the right owner doc or module
    boundary; do not suppress logs, loosen checks, skip cases, or route around
    symptoms. For pre-existing or external failures, classify them with evidence
    and report or park them without weakening gates.

## Principle Audit

Before finalizing substantial AI-assisted work, check:

- Objective: deliverables and non-goals are clear.
- Context fit: substantial or consequential work names the north-star, owner
  docs, affected modules/call paths, and plan state it used; tiny fixes kept
  context proportionate.
- Assumptions: consequential ambiguity was asked about or resolved from
  evidence.
- Unknowns: material unknowns were surfaced before large work (see
  `docs/ai/orchestration.md`), not discovered mid-implementation.
- Simplicity: the change is no larger than the current problem requires.
- Durability: the solution fits the right long-term owner doc, module boundary,
  and invariant instead of narrowing or hiding the problem.
- Scope: touched files are necessary and unrelated edits are preserved.
- Source of truth: mutable state has one owner; other surfaces point to it.
- Decision evidence: consequential choices are backed by claim-appropriate
  evidence such as code inspection, logs, tests, benchmarks, research, visual
  evidence, or prototypes.
- Root cause: any unexpected failure was reproduced and fixed at the cause with
  regression coverage, or classified as external/pre-existing without weakening
  gates.
- Public path: workflow guidance maps to tracked docs and `pixi run ...` gates.
- Evidence: checks, review results, artifacts, or blockers are recorded.
- Shared-state safety: any external mutation was explicitly approved.

Record the audit result in the final response or PR Testing section. Do not add
a separate audit log unless the work already uses a dev-task folder.
