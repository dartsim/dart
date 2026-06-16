# PLAN-121: AI Docs Knowledge Graph Guardrails

- Operating state: `PLAN-121` in [`dashboard.md`](dashboard.md)
- Outcome: DART's existing AI-native documentation remains the source of
  truth, while lightweight structural checks make links, owner indexes,
  AI-doc identity metadata, research-reference catalog shape, and north-star
  evidence freshness visible to both humans and agents.
- Current evidence: `docs/ai/principles.md` owns the single-source and
  evidence axioms, `docs/ai/components.md` owns generated-surface checks,
  `scripts/check_docs_policy.py` already owns docs lifecycle policy, and
  `docs/readthedocs/papers.md` already declares itself the research catalog's
  single source of truth.

## Scope Decisions

This plan intentionally adapts the LLM-wiki / OKF-style graph idea without
adding a new documentation platform.

- Keep durable Markdown docs as the source of truth.
- Add strict metadata only for the small `docs/ai/*.md` pilot, and only for
  stable identity fields: `type` and `owner`.
- Keep mutable operating state in `docs/plans/dashboard.md`, research status
  in `docs/readthedocs/papers.md`, and generated adapter state in
  `docs/ai/capabilities.json`.
- Start graph-health checks on pilot-owned surfaces as advisory signals before
  expanding their scope or making any of them blocking.
- Do not commit derived graph JSON until a concrete consumer exists and CI
  verifies it regenerates byte-identically.
- Treat consequential ambiguity as owner-local planning state rather than a
  global queue.

## Work Packets

#### WP-121.1 Graph resolver and report-only internal-link lint [done — local implementation]

- Objective: pilot-owned Markdown surfaces report broken internal links without
  blocking unrelated changes during the first rollout.
- Scope: `scripts/check_docs_policy.py` and focused tests under `tests/`.
- Non-goals: external link checking, anchor validation, or making the signal
  blocking before the inventory is clean.
- Acceptance evidence: the shared resolver handles relative paths,
  repo-root-style paths, fragments, and external-link skips; the policy check
  scans top-level docs indexes and the new AI-graph guardrail surfaces as
  advisories while returning success when no blocking failures exist.
- Gates: `pixi run python -m pytest tests/test_check_docs_policy.py`,
  `pixi run check-docs-policy`, `pixi run lint`.
- Dependencies: none.

#### WP-121.2 Conservative owner-index discoverability [done — local implementation]

- Objective: pilot orphan/discoverability reporting where ownership is already
  clear, starting with direct `docs/ai/*.md` files and their owner index.
- Scope: `scripts/check_docs_policy.py`, `docs/ai/README.md`, and focused
  tests.
- Non-goals: repo-wide reachability, README link dumps, or enforcing every
  nested documentation page through one global index.
- Acceptance evidence: direct `docs/ai/*.md` files are checked against
  `docs/ai/README.md` as an advisory signal.
- Gates: `pixi run python -m pytest tests/test_check_docs_policy.py`,
  `pixi run check-docs-policy`, `pixi run lint`.
- Dependencies: WP-121.1.

#### WP-121.3 AI-doc frontmatter pilot [done — local implementation]

- Objective: the eight Markdown docs under `docs/ai/` carry a minimal,
  machine-checkable identity header.
- Scope: `docs/ai/*.md` and `scripts/check_docs_policy.py`.
- Non-goals: mutable `status`, mutable `last_verified`, relationship lists,
  generated `docs/ai/index.json`, or frontmatter rollout beyond `docs/ai/`.
- Acceptance evidence: `check-docs-policy` rejects missing or unsupported
  fields for the pilot and `docs/ai/components.md` documents the boundary.
- Gates: `pixi run python -m pytest tests/test_check_docs_policy.py`,
  `pixi run check-docs-policy`, `pixi run lint`.
- Dependencies: none.

#### WP-121.4 Papers catalog validation [done — local implementation]

- Objective: the existing `papers.md` catalog is structurally validated in
  place without introducing a second source of truth.
- Scope: `scripts/check_docs_policy.py`,
  `docs/readthedocs/papers.md`, and focused tests.
- Non-goals: generated `papers_graph.json`, mandatory new frontmatter on the
  catalog, or external URL health checks.
- Acceptance evidence: summary/detail ID and shared-field parity,
  legend-owned enum legality, duplicate IDs, detail property lines, and local
  catalog links are checked.
- Gates: `pixi run python -m pytest tests/test_check_docs_policy.py`,
  `pixi run check-docs-policy`, `pixi run lint`.
- Dependencies: none.

#### WP-121.5 North-star freshness advisory [done — local implementation]

- Objective: the north-star status evidence table exposes when cited committed
  local evidence has changed after its last manual verification.
- Scope: `docs/ai/north-star.md` and `scripts/check_docs_policy.py`.
- Non-goals: blocking CI on git-date freshness, parsing every dashboard code
  span as evidence, or moving freshness into frontmatter.
- Acceptance evidence: `docs/ai/north-star.md` carries an advisory
  `evidence-last-verified` marker and `check-docs-policy` reports newer
  cited evidence paths as advisories.
- Gates: `pixi run python -m pytest tests/test_check_docs_policy.py`,
  `pixi run check-docs-policy`, `pixi run lint`.
- Dependencies: none.

#### WP-121.6 Retrospect compounding rule [done — local implementation]

- Objective: durable lessons captured by `dart-retrospect` become linked owner
  doc improvements instead of orphaned session prose.
- Scope: `.claude/commands/dart-retrospect.md`, generated adapters, and
  `docs/ai/components.md`.
- Non-goals: requiring a new file for every learning or adding section-level
  frontmatter.
- Acceptance evidence: the source command and generated adapters instruct
  agents to update existing owner docs first and link any new durable file
  from the relevant owner index or plan.
- Gates: `pixi run sync-ai-commands`, `pixi run check-ai-commands`,
  `pixi run check-docs-policy`, `pixi run lint`.
- Dependencies: WP-121.2 and WP-121.3.

#### WP-121.7 Decision-needed block pattern [done — local implementation]

- Objective: blocked roadmap or owner-doc work records the exact missing
  decision, default, evidence need, and unblock condition where future agents
  will look.
- Scope: `docs/plans/README.md`.
- Non-goals: a global decision queue, session log, or mandatory block for
  routine uncertainty.
- Acceptance evidence: the planning workflow documents a compact
  owner-local `Decision needed` block and the cleanup rule after the decision
  is made.
- Gates: `pixi run check-docs-policy`, `pixi run lint`.
- Dependencies: none.

## Follow-Up Trigger

After at least one clean cycle with no advisory output from the report-only
checks, decide whether any advisory should become blocking. Make that decision
in this plan and keep the flip separate from ordinary docs cleanup.
