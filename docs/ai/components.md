---
type: ai-component-policy
owner: self
---

# AI Components

This document defines how DART maintains AI-facing components.

## Ownership Model

`docs/ai/README.md` owns the source map; `docs/ai/terminology.md` owns terms.
Edit workflow sources in `.claude/commands/` and domain skills in
`.claude/skills/`. Generated `.agents/skills/` entrypoints name their source
and must not be hand-edited. Sync owns only DART-generated adapters and must
preserve unrelated user/plugin skills.

Maintained `.codex/` sources bound concurrency/delegation and define read-only
specialists and fast trusted-project hooks. They must not pin models or weaken
user permissions. Hooks are incomplete interception; the cross-tool Git guard
and explicit full gates remain required. Tool-specific caveats belong in
`docs/onboarding/ai-tools.md`.

Markdown directly under `docs/ai/` has stable `type` and `owner` frontmatter.
Mutable status belongs in its plan/dashboard owner; add metadata only with a
consumer. Harness context includes the north star, living plans, active
`docs/dev_tasks/` handoffs, and the handbook/design/module owners routed into a
task. Audit representative live state without loading every linked document.

## Harness Upgrade Audit Contract

`dart-model-upgrade` audits the whole harness by default. A new model can expose
instruction conflicts or unnecessary context even when configuration and
adapters still validate. Inspect each surface below, using representative live
tasks rather than loading every owner document. Report a supported preserve,
update, remove/consolidate, or add verdict for each surface; a routing-only
change does not establish whole-harness readiness.

| Surface                      | Content and structure questions                                                                                | Evidence                                                                                                  |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| Entry points and skills      | Does the right request discover the right workflow? Is required reading staged by task and phase?              | Fresh direct, indirect, incomplete, and non-trigger requests; declared reading inventory and actual reads |
| Instructions and permissions | Do stops, intake, delegation, and review rules preserve supplied decisions and authorization?                  | Conflicting source rules; fresh authorized and unauthorized action cases                                  |
| Durable context and handoffs | Is each changing fact owned once? Can a new session identify current work before historical logs?              | Owner/index links, live dashboard and representative handoff, fresh continuation                          |
| Tools and agents             | Do installed interfaces support the requested behavior and model/effort restrictions for parents and children? | Primary guidance, version/config probes, recorded effective settings and tool calls                       |
| Hooks and safety boundaries  | Are the guard and runner invariants preserved without duplicating slow gates?                                  | Structural checks and behavioral runner probes; explicit permission boundaries                            |
| Verification                 | Does evidence test the claimed outcome, including simulation and unavailable capabilities?                     | Text oracles, assessed native image review, failure cases, relevant regression gates                      |
| Maintenance and portability  | Are source/generated ownership, public paths, branch differences, and cleanup clear?                           | Adapter parity, docs policy, manual command path, branch-local apply/adapt/omit assessment                |
| Upgrade workflow itself      | Would a bare invocation discover these responsibilities without extra coaching?                                | A fresh invocation with no audit findings or expected answers supplied                                    |

For each proposed change, record the observed failure or cost, owner, candidate,
comparison that could reject it, and preservation gates. Prefer consolidation
when it removes a demonstrated conflict or repeated loading. Keep new machinery
only when it owns a distinct responsibility or failure boundary. Static size
reductions are context-budget evidence, not proof of higher-quality answers;
record correctness and constraint adherence separately from latency and tokens.
Measure authored words and files before/after separately from generated copies
and formatting bytes. Name removed/merged responsibilities and their retained
owners. Deferred loading is not content removal; justify net additions and any
duplication preserved for discovery.

Keep DART-specific constraints, owner routes, acceptance evidence, and completion
criteria. Rely on the model and built-in harness for general engineering and
session mechanics they already perform reliably; test suspected redundancy with
fresh tasks before removing a safeguard. Prefer an outcome and its quality bar
over another generic plan/execute loop. Do not copy volatile product guidance
into skills: link to its authority and keep dated compatibility evidence in
`docs/onboarding/ai-tools.md`. Replace superseded evidence; retain only
compatibility history that still changes a decision.

Load startup essentials once, then the relevant owner sections when a decision
needs them. A workflow's `@file` entries declare unconditional reading; prose
pointers name conditional reading and its trigger. Keep placement and safety
requirements explicit even when their detail is deferred. `pixi run ai-doctor --json`
reports per-workflow declared reading bytes; these exclude conditional,
transitive, and tool-injected context and are not token or runtime measurements.

## Adding A Workflow

Create `.claude/commands/dart-<name>.md` with `description` and
`argument-hint` frontmatter, then `## Required Reading`, `## Workflow`, and
`## Output` in that order. Use `$ARGUMENTS` for invocation input. Keep the
procedure concise; put policy and reference material in their owners.
Do not pin `model` or `effort`.

Register the capability in `docs/ai/workflows.md` (public paths, required
reading, gates) and `docs/ai/capabilities.json` (status, category, gate profile).
Run `pixi run sync-ai-commands` and `pixi run check-ai-commands`.
New generated directories may be hidden by a personal global gitignore:
stage them explicitly with `git add -f` if needed and confirm tracking with
`git ls-files`; disk parity alone cannot prove a fresh clone has the adapter.

## Adding A Domain Skill

Create `.claude/skills/dart-<name>/SKILL.md` with `name` and a quoted
`description` beginning `DART <Name>:`. Describe when to load it, retain
non-obvious DART constraints and commands, and point to the full owner docs.
Follow the same catalog registration, generation, and tracking checks above.
Module-local `AGENTS.md` owns module context; a skill supplies on-demand
routing and procedure. Do not duplicate tutorials or mutable API examples.

## Improving AI Infra From Learnings

Use `dart-retro` after successful or unsatisfactory tasks to improve future
execution from the original request and observed session history. The workflow
owns reconstruction, causal analysis, and validation. Place accepted changes
using `docs/ai/README.md`'s source map or `docs/information-architecture.md`.
Add an agent, hook, script, or document only for a distinct stable responsibility
or failure boundary.

Make a rule discoverable through the owner index and relevant required-reading
path, then verify adapter sync. Do not copy it across entrypoints to compensate
for a missing link. Promote durable task output before retiring its temporary
home; plans own mutable sequence and gates, not permanent implementation detail.
Local retrospective edits follow task authorization; external mutations follow
`docs/ai/principles.md`.

## Public Path Requirement

Every capability maps to tracked docs and `pixi run ...` commands that a
contributor can use without an AI tool.

## Checks

| Command                      | Responsibility / implementation                                                                                                                                      |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `pixi run sync-ai-commands`  | Generate DART-owned adapters; `scripts/sync_ai_commands.py`                                                                                                          |
| `pixi run check-ai-commands` | Non-mutating parity, metadata/budgets, catalogs, required reading, public paths, approval wording, and capability references; same script                            |
| `pixi run check-ai-infra`    | Runtime/config/hook safety, instruction discovery, branch profiles, model-pin ownership, scenarios, and guarded runner wiring/probes; `scripts/ai_infrastructure.py` |
| `pixi run test-ai-infra`     | Focused regression suite under the guarded runner                                                                                                                    |
| `pixi run check-docs-policy` | Placement, dev-task/plan lifecycle, dashboard/archive shape, pilot frontmatter/links/discoverability, and research catalog; `scripts/check_docs_policy.py`           |
| `pixi run ai-doctor`         | Read-only setup, discovery, reading-size, and recovery diagnosis; `scripts/ai_doctor.py`                                                                             |
| `pixi run check-agent-hook`  | Bounded staged structural feedback, not completion evidence                                                                                                          |

Preserve these failure boundaries when changing the checks:

- Docs-update always loads `docs/AGENTS.md` and the information-architecture
  owner; new-task loads that owner before placing durable output.
- Capability references cover prose, invocation, and source-path forms.
  Deliberate non-capability names use the explicit
  `NON_CAPABILITY_DART_NAMES` ledger, never a build-system-derived exemption.
- Every tracked pytest/CTest gate crosses its canonical guarded runner.
  Completion probes prove real test execution, hostile-selector/plugin
  isolation, zero-body rejection, and failure propagation. Preserve the CTest
  wrapper's lexical flow contract; superficial command markers are insufficient.
- The simulation scenario retains the text oracle, claim-tied native image
  review, and honest unavailable exception.
- North-star freshness is advisory: changing a cited file does not itself
  invalidate a conclusion. Re-verify the owner evidence before refreshing dates.
  New advisories need an owned pilot and clean inventory or explicit baseline.

These structural checks do not prove instruction quality. Gate selection,
behavioral evidence, and completion belong to `docs/ai/verification.md`;
the principle audit supplies the judgment about simplicity and ownership.
