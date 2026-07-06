# WS-E — Infra + evidence lane

Everything the other lanes need to measure honestly and hand off cleanly.

#### WP-PG.01 — Round-2 baseline evidence packet

- Status: done — PR pending; **prerequisite baseline captured for all other
  packets**
- Objective: captured the full baseline matrix, then refreshed the durable
  guard rows on current `origin/release-6.20` @ `b9e6910c066` per the
  **exact cell matrix and canonical commands** in
  [01-baseline-evidence.md](01-baseline-evidence.md) (scenes S1–S6;
  original dashboard tooling run via
  `scripts/run_performance_dashboard_benchmarks.py` plus the scenario-CLI
  rows), with final hashes, scene dumps, profile share breakdowns
  (including the `Construct LCP` vs Dantzig-solve split), and host
  metadata (compiler, CPU, governor). Also triage the six
  `origin/perf/dart6-*` prior-art branches: record verdicts in the
  prior-art inventory and propose deletion of dead branches.
- Value: the durable regression reference for the whole round (round 1's
  guard table was deleted with #3199; this packet re-establishes it in a
  durable home).
- Scope: docs-only + benchmark artifacts; record guard rows in
  01-baseline-evidence.md; no code change.
- Acceptance evidence: complete tables committed; dashboard artifacts
  generated; explicit "guard rows" section marked as the reference for
  subsequent PRs.
- Dependencies: none.

#### WP-PG.02 — Extend the contact-container benchmark matrix

- Status: open
- Objective: extend `BM_INTEGRATION_contact_container` with fcl/bullet
  engines, a 900-object row, deactivation-enabled variants, and a threads
  sweep, keeping the #3230 dashboard schema stable (additive surfaces
  only); anchor benchmark filter regexes (`$`) in scripts.
- Value: the primary fixture then covers the full detector matrix that
  both this effort and WS-F's default-flip gates need.
- Scope: `tests/benchmark/integration/bm_contact_container.cpp`,
  dashboard scripts allowlist; CI runtime budget check (keep added rows
  within the dashboard job's time envelope — measure and record).
- Acceptance evidence: new rows produce stable (cv < ~5%) aggregates
  locally; dashboard workflow green; no schema break (merge/preview
  scripts run).
- Dependencies: WP-PG.01 (so old/new rows are comparable).

#### WP-PG.03 — DART 6 profiling documentation + Tracy config task

- Status: open
- Objective: author `docs/onboarding/profiling.md` **for 6.20** (the main
  doc documents DART 7-only APIs): `dart/common/Profile.hpp` text backend
  (`DART_BUILD_PROFILE`, default ON in the pixi config),
  `contact_benchmark --profile`, and the Tracy opt-in recipe; add a pixi
  `config-tracy` (profile env) task so a Tracy build is one command
  (currently manual reconfigure; note `TRACY_NO_EXIT=1` for short runs).
- Value: every future perf packet starts with a profile; today the
  instructions live in nobody's head but round-1 history.
- Scope: docs + pixi.toml task; no C++ change.
- Acceptance evidence: doc renders; commands verified end-to-end on a
  clean tree; `pixi run lint` (incl. toml/docs linters) green.
- Dependencies: none.

#### WP-PG.04 — Executor tooling decision (D4)

- Status: blocked on D4
- Objective: either (a) confirm `/dart-resume` + RESUME.md as the 6.20
  executor entry point (documenting the packet-claim flow in this
  folder), or (b) backport a 6.20-adapted `dart-execute-packet` command
  whose required-reading list points at this folder's dashboard instead
  of main's plans dashboard — regenerating `.codex`/`.opencode` surfaces
  (`scripts/sync_ai_commands.py --check`) in the same PR.
- Value: lets Claude/Codex sessions claim and execute packets with zero
  extra context, which is the stated goal of this planning round.
- Scope: `.claude/commands|skills` + generated surfaces (option b), or
  docs-only (option a).
- Acceptance evidence: a fresh session can pick up the next packet from
  RESUME.md (or the backported command) and find every gate command it
  needs without reading outside the folder.
- Dependencies: D4.
