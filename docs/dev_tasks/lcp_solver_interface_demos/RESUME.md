# Resume: LCP Solver Interface And Demos

## Current Reality - 2026-06-12 LcpProblem Contact Metadata

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable:
  `git fetch https://github.com/dartsim/dart.git main`
- `git merge --no-edit FETCH_HEAD` reported `Already up to date`, so the
  branch remains current with the PR #2986 DART 7 work-packet harness on
  `main` (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 89]`
- Last committed checkpoint:
  `71bfc925863 Expose LCP representative requirement coverage`
- Checkpoint target:
  `Expose LCP problem contact metadata`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 90 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `dart/math/lcp/lcp_types.hpp` adds
  `LcpProblem::getFrictionIndexRowCount()` and
  `LcpProblem::getFrictionIndexContactCount()`.
- `python/dartpy/math/lcp.cpp` exposes those methods as
  `get_friction_index_row_count()` and
  `get_friction_index_contact_count()`.
- `python/examples/demos/scenes/lcp_physics.py` records LCP dimension,
  FrictionIndex row count, and FrictionIndex contact count in the
  representative solver suite, and displays dimension/contact count in the
  summary table.
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now emits `contact_count`
  from `LcpProblem` itself for FrictionIndex runs through the shared benchmark
  runner.
- `python/tests/unit/math/test_lcp.py` replaces one brittle exact-vector
  assertion for a degenerate indefinite Baraff fallback with a direct LCP
  complementarity check, because the problem has multiple valid solutions.

Verification completed:

```bash
pixi run run-cpp-target UNIT_math_lcp_math_lcp_lcp_types
pixi run build-py-dev
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/math/test_lcp.py python/tests/unit/test_py_demo_panels.py -q
CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target BM_LCPSOLVER
pixi run lint
git diff --check
```

Results:

- C++ LCP type test: `15 tests` passed.
- dartpy rebuild: passed.
- Focused Python LCP and py-demo panel tests: `122 passed`.
- LCP benchmark target compile: passed.
- `pixi run lint`: passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is still uncommitted and no files changed after the verification
above, commit it with `Expose LCP problem contact metadata`. If files changed,
rerun `pixi run lint` and `git diff --check` first. Continue from a fresh
bounded DART 7 LCP interface/demo gap; avoid retrying the rejected SAP
FrictionIndex exact shortcut or ShockPropagation exact-path probe without a
materially different hypothesis. Do not push without explicit maintainer/user
approval.

## Current Reality - 2026-06-12 Representative Requirement Coverage

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main` (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 88]`
- Last committed checkpoint:
  `aa43914b348 Expose LCP solver selection guidance`
- Checkpoint target:
  `Expose LCP representative requirement coverage`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 89 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `python/examples/demos/scenes/lcp_physics.py` adds a Representative
  requirement coverage table to the LCP panel.
- The table maps named requirements to live packets, benchmark packets, metrics,
  and evidence cues: billiards symmetry/energy/momentum, high mass-ratio stack,
  thin card pile, scalability smoke, and friction coupling with active
  tangential bounds.
- `python/tests/unit/test_py_demo_panels.py` checks both setup metadata and
  headless panel rendering for the requirement map.

Verification completed:

```bash
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
git diff --check
```

Results:

- Focused py-demo panel tests: `43 passed`.
- `pixi run lint`: passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Expose LCP representative requirement coverage`. Continue from a fresh bounded
DART 7 LCP interface/demo gap; avoid retrying the rejected SAP FrictionIndex
exact shortcut or ShockPropagation exact-path probe without a materially
different hypothesis. Do not push without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Solver Selection Guidance Panel

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main` (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 87]`
- Last committed checkpoint:
  `5d9ac41a227 Expose LCP profile evidence coverage summary`
- Checkpoint target:
  `Expose LCP solver selection guidance`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 88 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `python/examples/demos/scenes/lcp_physics.py` adds a Solver selection guide
  table to the LCP panel.
- The table maps solver families to representative best-fit cases, strengths,
  tradeoffs, and current profile-evidence cues, making solver pros/cons visible
  in the demo instead of only in background docs.
- `python/tests/unit/test_py_demo_panels.py` checks both setup metadata and
  headless panel rendering for the guidance table.

Verification completed:

```bash
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
git diff --check
```

Results:

- Focused py-demo panel tests: `43 passed`.
- `pixi run lint`: passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Expose LCP solver selection guidance`. Continue from a fresh bounded DART 7
LCP interface/demo gap; avoid retrying the rejected SAP FrictionIndex exact
shortcut or ShockPropagation exact-path probe without a materially different
hypothesis. Do not push without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Profile Evidence Coverage Summary

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main` (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 86]`
- Last committed checkpoint:
  `128a054ff1a Expose LCP profile evidence schema`
- Checkpoint target:
  `Expose LCP profile evidence coverage summary`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 87 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `python/examples/demos/scenes/lcp_physics.py` reads the checked
  `docs/background/lcp/figures/performance_profile_evidence.csv` artifact and
  adds a `lcp_performance_profile_evidence_summary` table under Performance
  profiles.
- The table summarizes per-surface evidence coverage: row counts, solver
  counts, LCP dimensions, FrictionIndex contact counts, contract pass counts,
  max iterations, and max residual/complementarity/bound-violation counters.
- `python/tests/unit/test_py_demo_panels.py` checks both setup metadata and
  headless panel rendering for the new evidence summary.

Verification completed:

```bash
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
git diff --check
```

Results:

- Focused py-demo panel tests: `43 passed`.
- `pixi run lint`: passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Expose LCP profile evidence coverage summary`. Continue from a fresh bounded
DART 7 LCP interface/demo gap; avoid retrying the rejected SAP FrictionIndex
exact shortcut or ShockPropagation exact-path probe without a materially
different hypothesis. Do not push without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Profile Evidence Schema Panel

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main` (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 85]`
- Last committed checkpoint:
  `7298ccde11c Expose LCP representative challenge labels`
- Checkpoint target:
  `Expose LCP profile evidence schema`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 86 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `python/examples/demos/scenes/lcp_physics.py` adds a visible
  `lcp_performance_profile_evidence_schema` table under Performance profiles.
- The table documents the current checked evidence CSV counters that matter for
  the DART 7 harness: solver identity schema/version, solver manifest index,
  native support flags, problem-type counters, `lcp_dimension`, `contact_count`,
  `contract_ok`, `iterations`, `residual`, `complementarity`, and
  `bound_violation`.
- `python/tests/unit/test_py_demo_panels.py` checks both setup metadata and
  headless panel rendering for the evidence-schema rows.

Verification completed:

```bash
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
git diff --check
```

Results:

- Focused py-demo panel tests: `43 passed`.
- `pixi run lint`: passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Expose LCP profile evidence schema`. Continue from a fresh bounded DART 7 LCP
interface/demo gap; avoid retrying the rejected SAP FrictionIndex exact shortcut
or ShockPropagation exact-path probe without a materially different hypothesis.
Do not push without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Representative Challenge Labels

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS earlier in this continuation because
  SSH to `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 84]`
- Last committed checkpoint:
  `59595b8d2f1 Record LCP problem dimension evidence`
- Checkpoint target:
  `Expose LCP representative challenge labels`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 85 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `python/examples/demos/scenes/lcp_physics.py` adds a visible `Challenge`
  column to the Representative solver suite table.
- The table now exposes why each all-solver standalone case exists:
  mass-ratio conditioning, active bounds, rank-deficient degeneracy,
  friction-index active tangent coupling, and scalability smoke.
- `python/tests/unit/test_py_demo_panels.py` checks the table header and key
  challenge strings so future edits cannot hide that representative-example
  context again.

Verification completed:

```bash
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Results:

- Focused py-demo panel tests: `43 passed`.
- `pixi run lint`: passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Expose LCP representative challenge labels`. Continue from a fresh bounded DART
7 LCP interface/demo gap; avoid retrying the rejected SAP
FrictionIndex exact shortcut or ShockPropagation exact-path probe without a
materially different hypothesis. Do not push without explicit maintainer/user
approval.

## Current Reality - 2026-06-12 Problem Dimension Evidence Guard

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 83]`
- Last committed checkpoint:
  `de016b4969f Record LCP profile metric evidence`
- Checkpoint target:
  `Record LCP problem dimension evidence`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 84 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `scripts/lcp_performance_profile.py` retains the current `BM_LcpCompare`
  emitted `problem_size` counter as `lcp_dimension` and preserves
  `contact_count` for FrictionIndex profile rows.
- Profile `problem_size` remains the benchmark profile argument. It is the LCP
  row dimension for Standard/Boxed rows and contact count for FrictionIndex
  rows; FrictionIndex `lcp_dimension` must equal `3 * contact_count`.
- `scripts/check_lcp_solver_roster.py` validates those dimension/contact
  relationships as part of `lint-lcp-solver-roster`.
- Parser and roster tests cover dimension mismatch rejection.
- The checked evidence CSV was regenerated from the existing
  `build/lcp_profile_full.json` cache, not from a new benchmark run.

DART 7 harness rationale:

- PR #2986 requires benchmark/evidence packets to machine-record problem
  identity and metrics. This slice makes the profile artifact carry the emitted
  row dimension and friction contact-count evidence rather than relying only on
  the benchmark-name size token.

Verification completed:

```bash
PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --cache build/lcp_profile_full.json --output docs/background/lcp/figures
PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py -q
PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py
```

Results:

- Evidence regeneration: completed from cached profile JSON.
- Focused parser/roster tests: `12 passed`.
- Roster script: passed.
- `pixi run lint`: passed, including `lint-lcp-solver-roster` with the expanded
  dimension/contact evidence guard.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Record LCP problem dimension evidence`. Continue from a fresh bounded DART 7
LCP interface/demo gap; avoid retrying the rejected SAP
FrictionIndex exact shortcut or ShockPropagation exact-path probe without a
materially different hypothesis. Do not push without explicit maintainer/user
approval.

## Current Reality - 2026-06-12 Profile Metric Evidence Guard

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 82]`
- Last committed checkpoint:
  `8aaa8e4548b Validate LCP performance evidence roster identity`
- Checkpoint target:
  `Record LCP profile metric evidence`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 83 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `scripts/lcp_performance_profile.py` retains current `BM_LcpCompare`
  `iterations` and `bound_violation` counters and writes them into
  `performance_profile_evidence.csv`.
- `scripts/check_lcp_solver_roster.py` validates the row-level metric fields
  as part of `lint-lcp-solver-roster`: positive finite `time_ns`,
  `contract_ok=1`, nonnegative integer `iterations`, and finite nonnegative
  residual/complementarity/bound-violation values.
- `python/tests/unit/test_lcp_performance_profile.py` covers metric retention
  and CSV output.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers rejection of an
  invalid evidence metric.
- The checked evidence CSV was regenerated from the existing
  `build/lcp_profile_full.json` cache, not from a new benchmark run.

DART 7 harness rationale:

- PR #2986 added the work-packet harness and the solver-family intake rule
  that benchmark/evidence packets must machine-record resolved solver identity
  and shared metrics. This slice keeps the current LCP dev-task branch aligned
  with that rule without claiming a full PLAN-091 resolved-configuration
  report is implemented.

Verification completed:

```bash
git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main
git merge --no-edit origin/main
PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --cache build/lcp_profile_full.json --output docs/background/lcp/figures
PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py -q
PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py
```

Results:

- `origin/main` fetch/merge: up to date.
- Evidence regeneration: completed from cached profile JSON.
- Focused parser/roster tests: `10 passed`.
- Roster script: passed.
- `pixi run lint`: passed, including `lint-lcp-solver-roster` with the expanded
  evidence metric guard.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Record LCP profile metric evidence`. Continue from a fresh bounded DART 7 LCP
interface/demo gap; avoid retrying the rejected SAP
FrictionIndex exact shortcut or ShockPropagation exact-path probe without a
materially different hypothesis. Do not push without explicit maintainer/user
approval.

## Current Reality - 2026-06-12 Evidence CSV Roster Guard

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation; merging
  `origin/main` reported `Already up to date`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 81]`
- Last committed checkpoint:
  `27bd1f8fe8d Record LCP benchmark solver identity counters`
- Checkpoint target:
  `Validate LCP performance evidence roster identity`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 82 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `scripts/check_lcp_solver_roster.py` validates
  `performance_profile_evidence.csv` as part of the existing
  `lint-lcp-solver-roster` gate.
- The guard checks required evidence columns, category names, manifest solver
  names, positive problem sizes, solver identity schema/index counters, support
  counters, `solver_supports_problem`, and one-hot problem-type counters
  against the C++ LCP solver manifest.
- `python/tests/unit/test_check_lcp_solver_roster.py` now covers a mismatched
  manifest-index row to prove the guard rejects stale or inconsistent evidence.

Verification completed:

```bash
PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q
PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py
PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py python/tests/unit/test_lcp_performance_profile.py -q
pixi run lint
git diff --check
```

Results:

- Focused roster tests: `2 passed`.
- Roster script: passed.
- Focused roster/profile tests: `9 passed`.
- `pixi run lint`: passed, including `lint-lcp-solver-roster` with the new
  evidence CSV guard.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Validate LCP performance evidence roster identity`. Continue from a fresh
bounded DART 7 LCP interface/demo gap; avoid retrying the rejected SAP
FrictionIndex exact shortcut or ShockPropagation exact-path probe without a
materially different hypothesis. Do not push without explicit maintainer/user
approval.

## Current Reality - 2026-06-12 Solver Identity Counters

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the PR #2986 DART 7 harness remains present
  via merge commit `bb851f45360`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 80]`
- Last committed checkpoint:
  `f27d12c110d Add LCP performance profile evidence CSV`
- Checkpoint target:
  `Record LCP benchmark solver identity counters`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 81 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `tests/common/lcpsolver/lcp_solver_manifest.hpp` owns
  `kLcpSolverIdentitySchemaVersion = 1` and a stable 1-based manifest index
  helper.
- `BM_LcpCompare` rows emit `solver_identity_schema_version` and
  `solver_manifest_index` counters.
- `scripts/lcp_performance_profile.py` preserves both counters, rejects
  current-schema profile rows whose benchmark-name solver disagrees with the
  emitted manifest index, and writes both fields into
  `performance_profile_evidence.csv`.
- `python/examples/demos/scenes/lcp_physics.py` was refreshed to match the new
  full-profile summary and keeps the row-level evidence CSV visible in the
  performance-profile table.

Verification completed:

```bash
git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main
git merge --no-edit origin/main
PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q
cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"
pixi run bm lcp_compare -- --benchmark_filter='BM_LcpCompare/Standard/Dantzig/12$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_out=build/lcp_identity_counters_probe.json --benchmark_out_format=json
PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --run --cache build/lcp_profile_full.json --output docs/background/lcp/figures --benchmark-timeout 900
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_py_demo_panels.py -q
pixi run lint
git diff --check
```

Results:

- `origin/main` fetch/merge: up to date.
- Parser tests: `7 passed`.
- Focused benchmark target build: passed.
- Focused JSON probe:
  `BM_LcpCompare/Standard/Dantzig/12` emitted
  `solver_identity_schema_version=1.0`, `solver_manifest_index=1.0`,
  `solver_supports_problem=1.0`, and `problem_type_standard=1.0`.
- Full profile refresh: completed and regenerated Standard, Boxed,
  FrictionIndex, and evidence CSVs with populated identity counters.
- Focused parser/demo tests: `50 passed`.
- `pixi run lint`: passed and reformatted
  `scripts/lcp_performance_profile.py`.
- Focused parser/demo tests rerun after lint: `50 passed`.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Record LCP benchmark solver identity counters`. Continue from a fresh bounded
DART 7 LCP interface/demo gap; avoid retrying the rejected SAP FrictionIndex
exact shortcut or ShockPropagation exact-path probe without a materially
different hypothesis. Do not push without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Row-Level Profile Evidence

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed earlier in this continuation; merging
  `origin/main` reported "Already up to date." The recent DART 7 harness from
  PR #2986 is already in this branch via the earlier `origin/main` merge.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 79]`
- Last committed checkpoint:
  `a966dc95c9e Validate LCP benchmark form support evidence`
- Checkpoint target:
  `Add LCP performance profile evidence CSV`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 80 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `scripts/lcp_performance_profile.py` writes
  `performance_profile_evidence.csv` beside the profile curve CSVs.
- The evidence CSV records category, manifest solver, problem size, timing,
  contract metrics, emitted form-support counters, concrete
  `solver_supports_problem`, and emitted `problem_type_*` counters for each
  parsed profile row.
- `python/examples/demos/scenes/lcp_physics.py` exposes the evidence artifact
  in performance-profile metadata and the GUI table.
- The checked Standard, Boxed, FrictionIndex, and evidence CSVs were
  regenerated from current-schema `BM_LcpCompare/` benchmark JSON.

Verification completed:

```bash
pixi run lint
git diff --check
```

Progress before this docs update:

- Full profile refresh:
  `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --run --cache build/lcp_profile_full.json --output docs/background/lcp/figures --benchmark-timeout 900`
  completed and wrote the checked profile/evidence CSVs.
- Evidence CSV: 184 lines including header, with support and problem-type
  counters populated.
- Focused parser and demo metadata tests:
  `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_py_demo_panels.py -q`
  passed with `49 passed`.
- `pixi run lint`: passed and reformatted
  `scripts/lcp_performance_profile.py`; focused parser/demo tests were rerun
  afterwards and still passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Add LCP performance profile evidence CSV`. Continue from a fresh
bounded DART 7 LCP interface/demo gap; avoid retrying the rejected SAP
FrictionIndex exact shortcut or ShockPropagation exact-path probe without a
materially different hypothesis.

## Current Reality - 2026-06-12 Form Support Schema Guard

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed earlier in this continuation; merging
  `origin/main` reported "Already up to date." The recent DART 7 harness from
  PR #2986 is already in this branch via the earlier `origin/main` merge.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 78]`
- Last committed checkpoint:
  `3cd4d9382c9 Validate LCP benchmark problem type evidence`
- Checkpoint target:
  `Validate LCP benchmark form support evidence`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 79 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `scripts/lcp_performance_profile.py` retains the emitted
  `solver_supports_standard`, `solver_supports_boxed`, and
  `solver_supports_friction_index` counters.
- Current-schema profile ingestion now rejects rows whose
  `BM_LcpCompare/<problem-family>/...` category disagrees with the solver's
  form-level native support counter for that category.
- Historical cached rows without these form-level counters remain accepted.
- `python/tests/unit/test_lcp_performance_profile.py` covers the new
  form-support mismatch rejection in addition to the existing concrete support
  and problem-type checks.

Verification completed:

```bash
PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q
PYTHONPATH=python pixi run python - <<'PY'
import importlib.util
import json
import sys
from pathlib import Path

path = Path("scripts/lcp_performance_profile.py")
spec = importlib.util.spec_from_file_location("lcp_performance_profile", path)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
data = json.loads(Path("build/lcp_support_counters_probe.json").read_text())
results = module.parse_benchmark_results(data)
module.check_native_profile_coverage(
    results,
    {"Standard": {"Dantzig"}, "Boxed": set(), "FrictionIndex": set()},
)
PY
pixi run lint
git diff --check
```

Results:

- Python profile parser tests: `5 passed`.
- Focused parser acceptance check against
  `build/lcp_support_counters_probe.json`: passed.
- `pixi run lint`: passed and reformatted
  `scripts/lcp_performance_profile.py`; parser tests were rerun afterwards and
  still passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Validate LCP benchmark form support evidence`. Continue from a fresh bounded
DART 7 harness gap; avoid retrying the rejected SAP FrictionIndex exact shortcut
or ShockPropagation exact-path probe without a materially different hypothesis.

## Current Reality - 2026-06-12 Problem Type Schema Guard

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed earlier in this continuation; merging
  `origin/main` reported "Already up to date." The recent DART 7 harness from
  PR #2986 is already in this branch via the earlier `origin/main` merge.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 77]`
- Last committed checkpoint:
  `99b68ab7ca0 Record concrete LCP benchmark support evidence`
- Checkpoint target:
  `Validate LCP benchmark problem type evidence`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 78 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `scripts/lcp_performance_profile.py` retains all emitted `problem_type_*`
  counters.
- Current-schema profile ingestion now rejects rows whose
  `BM_LcpCompare/<problem-family>/...` category disagrees with the concrete
  problem-type counters.
- Historical cached rows without `problem_type_*` counters remain accepted.
- `python/tests/unit/test_lcp_performance_profile.py` covers the new mismatch
  rejection in addition to the existing support-counter checks.

Verification completed:

```bash
PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q
PYTHONPATH=python pixi run python - <<'PY'
import importlib.util
import json
import sys
from pathlib import Path

path = Path("scripts/lcp_performance_profile.py")
spec = importlib.util.spec_from_file_location("lcp_performance_profile", path)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
data = json.loads(Path("build/lcp_support_counters_probe.json").read_text())
results = module.parse_benchmark_results(data)
module.check_native_profile_coverage(
    results,
    {"Standard": {"Dantzig"}, "Boxed": set(), "FrictionIndex": set()},
)
PY
pixi run lint
git diff --check
```

Results:

- Python profile parser tests: `4 passed`.
- Focused parser acceptance check against
  `build/lcp_support_counters_probe.json`: passed.
- `pixi run lint`: passed and reformatted
  `scripts/lcp_performance_profile.py`; parser tests were rerun afterwards and
  still passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Validate LCP benchmark problem type evidence`. Continue from a fresh bounded
DART 7 harness gap; avoid retrying the rejected SAP FrictionIndex exact shortcut
or ShockPropagation exact-path probe without a materially different hypothesis.

## Current Reality - 2026-06-12 Concrete Benchmark Support Evidence

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- `origin/main` was refreshed over HTTPS because SSH to GitHub port 22 was not
  reachable in this environment; merging `origin/main` reported "Already up to
  date." The recent DART 7 harness from PR #2986 is already in this branch via
  the earlier `origin/main` merge.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 76]`
- Last committed checkpoint:
  `58dec3fd07a Record LCP post-APGD probe handoff`
- Checkpoint target:
  `Record concrete LCP benchmark support evidence`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 77 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` emits concrete solver/problem
  support counters on `BM_LcpCompare` rows:
  `solver_supports_standard`, `solver_supports_boxed`,
  `solver_supports_friction_index`, `solver_supports_problem`, and one-hot
  `problem_type_*` counters.
- `scripts/lcp_performance_profile.py` preserves `solver_supports_problem` and
  rejects current-schema rows where the solver reports no native support for
  the concrete problem. Missing counters remain allowed for historical cached
  benchmark JSON.
- `python/tests/unit/test_lcp_performance_profile.py` covers parser retention,
  current-schema rejection, and historical-cache compatibility.

Verification completed:

```bash
PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q
cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"
pixi run bm lcp_compare -- --benchmark_filter='BM_LcpCompare/Standard/Dantzig/12$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_out=build/lcp_support_counters_probe.json --benchmark_out_format=json
pixi run lint
git diff --check
```

Results:

- Python profile parser tests: `3 passed`.
- Focused benchmark target build: passed.
- Focused JSON probe:
  `BM_LcpCompare/Standard/Dantzig/12` emitted the new counters with
  `solver_supports_problem=1.0` and `problem_type_standard=1.0`.
- `pixi run lint`: passed.
- `git diff --check`: passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this slice is uncommitted, review the verification above and commit it with
`Record concrete LCP benchmark support evidence`. Continue from a fresh bounded
DART 7 harness gap; avoid retrying the rejected SAP FrictionIndex exact shortcut
or ShockPropagation exact-path probe without a materially different hypothesis.

## Current Reality - 2026-06-12 Post-APGD Probe Triage

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 75]`
- Last committed checkpoint:
  `c6eb366c126 Raise APGD exact gate for friction-index comparison`
- Worktree was clean before this docs hand-off update.
- There is no associated PR yet.
- This continuation has not pushed the branch.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What changed most recently:

- The APGD FrictionIndex gate slice is committed and remains the latest code
  checkpoint.
- Follow-up SAP and ShockPropagation probes were run under the same PR #2986
  packet-harness evidence rules. Neither produced an accepted code change.

Rejected and non-actionable follow-up evidence:

- SAP FrictionIndex exact shortcut was rejected:
  - Baseline: `build/findex_sap_exact_baseline.json`.
  - Probe: `build/findex_sap_exact_probe.json`.
  - Temporary edit added
    `detail::trySolveInteriorFrictionIndexLcp(...)` to `SapSolver` behind a
    192-row gate, then was reverted.
  - Timings: `Sap/4` `1376ns -> 1101ns`, `Sap/16` `11312ns -> 11402ns`,
    `Sap/64` `224838ns -> 234951ns`; all probe rows had `iterations=0`, but
    the medium/large packet evidence did not support keeping the change.
- ShockPropagation focus check was non-actionable:
  - Focus file: `build/shock_current_focus.json`.
  - Boxed and FrictionIndex rows already report `iterations=0` and
    `contract_ok=1`.
  - Current timings: Boxed `12/24/48` are `1170ns`, `3074ns`, `11466ns`;
    FrictionIndex `4/16/64` are `1142ns`, `11641ns`, `223086ns`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -8
```

If this hand-off docs update is uncommitted, run `pixi run lint`, run
`git diff --check`, and commit it. For further implementation, avoid retrying
the rejected Boxed `SymmetricPsor`/`RedBlackGaussSeidel` exact probes, the
rejected SAP FrictionIndex exact shortcut, or ShockPropagation exact-path
investigation without a materially different hypothesis. Current regenerated
profiles have no solver-family average above `1.6x`.

## Current Reality - 2026-06-12 APGD FrictionIndex Gate

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 74]`
- Last committed checkpoint:
  `63bfc8b349b Raise Jacobi standard exact gate and use LLT`
- Checkpoint target:
  `Raise APGD exact gate for friction-index comparison`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 75 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `ApgdSolver` allows validated exact shortcuts through 192 scalar rows so the
  64-contact FrictionIndex comparison packet can take the exact shortcut.
- The Standard 12/24/48 rows were already exact; their focused movement in this
  slice is benchmark noise. The Standard 96 row also becomes exact under the
  raised gate.
- The slice follows the PR #2986 harness stance captured below: evidence uses
  the current interim LCP solver identity path from `BM_LcpCompare` benchmark
  names, `contract_ok` counters, timing fields, manifest metadata, and native
  support checks.

Evidence:

- Baseline:
  `build/findex_apgd_gate_baseline.json`.
- Prior Standard focused reference:
  `build/standard_sap_apgd_llt_probe.json`.
- Accepted focused probe:
  `build/apgd_gate192_probe.json`.
- Focused FrictionIndex `Apgd` timings:
  - `Apgd/4`: `1119.00ns -> 1123.00ns`.
  - `Apgd/16`: `12956.00ns -> 11174.00ns`.
  - `Apgd/64`: `358923.00ns -> 231923.00ns`.
- Focused Standard `Apgd/96`: `58359.00ns -> 41387.00ns`; smaller Standard
  APGD rows were already exact.
- Latest regenerated profile highlights:
  - Standard: `Apgd 1.22`; no solver average is above `1.6x`; highest rows are
    `BoxedSemiSmoothNewton 1.51`, `MPRGP 1.41`, and
    `RedBlackGaussSeidel 1.34`.
  - Boxed: no solver average is above `1.6x`; highest rows are
    `SymmetricPsor 1.57`, `RedBlackGaussSeidel 1.53`,
    `ShockPropagation 1.52`, and `SubspaceMinimization 1.41`.
  - FrictionIndex: no solver average is above `1.6x`; `Apgd` moved to `1.38`;
    highest rows are `Sap 1.58`, `ShockPropagation 1.58`, `BGS 1.45`, and
    `Admm 1.41`.

Verification state:

- Completed so far:
  - Focused baseline and accepted focused probe for
    `BM_LcpCompare/FrictionIndex/Apgd/`, with Standard APGD rows included in
    the probe.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
- Final pre-commit checks after any final edits:
  - `pixi run lint`.
  - `git diff --check`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
```

If this checkpoint is still uncommitted, run final lint/diff checks and commit
with `Raise APGD exact gate for friction-index comparison`. If it is already
committed, investigate Boxed `SymmetricPsor 1.57`, Boxed
`RedBlackGaussSeidel 1.53`, or FrictionIndex `Sap 1.58` /
`ShockPropagation 1.58` under the same packet-like evidence rules. Do not push
without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Jacobi Standard Gate / LLT Path

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 73]`
- Last committed checkpoint:
  `343949c66f3 Use LLT for APGD and SAP standard exact paths`
- Checkpoint target:
  `Raise Jacobi standard exact gate and use LLT`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 74 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `JacobiSolver` allows Standard strict-interior exact solves through the 96-row
  comparison packet and routes that path through
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- A gate-only 48-row probe that kept the LU helper was rejected because it did
  not materially improve the 48-row path and worsened the 24/96 focused rows.
- The slice follows the PR #2986 harness stance captured below: evidence uses
  the current interim LCP solver identity path from `BM_LcpCompare` benchmark
  names, `contract_ok` counters, timing fields, manifest metadata, and native
  support checks.

Evidence:

- Baseline:
  `build/standard_jacobi_gate_baseline.json`.
- Rejected gate-only probe:
  `build/standard_jacobi_gate48_probe.json`.
- Accepted focused probe:
  `build/standard_jacobi_gate96_llt_probe.json`.
- Focused Standard `Jacobi` timings:
  - `Jacobi/12`: `825.00ns -> 781.00ns`.
  - `Jacobi/24`: `2659.00ns -> 2145.00ns`.
  - `Jacobi/48`: `10198.00ns -> 9424.00ns`.
  - `Jacobi/96`: `55301.00ns -> 40143.00ns`.
- Latest regenerated profile highlights:
  - Standard: `Jacobi 1.22`; no solver average is above `1.6x`; highest rows
    are `Baraff 1.57`, `MPRGP 1.44`, `Pgs 1.40`, and `Tgs 1.35`.
  - Boxed: no solver average is above `1.6x`; highest rows are
    `RedBlackGaussSeidel 1.51`, `ShockPropagation 1.48`,
    `SymmetricPsor 1.47`, and `NNCG 1.36`.
  - FrictionIndex: `Apgd 1.61`; next rows are `ShockPropagation 1.57`,
    `Sap 1.51`, and `Admm 1.45`.

Verification state:

- Completed so far:
  - Focused baseline, rejected gate-only probe, and accepted focused probe for
    `BM_LcpCompare/Standard/Jacobi/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
- Final pre-commit checks after any final edits:
  - `pixi run lint`.
  - `git diff --check`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
```

If this checkpoint is still uncommitted, run final lint/diff checks and commit
with `Raise Jacobi standard exact gate and use LLT`. If it is already
committed, investigate Boxed `RedBlackGaussSeidel 1.51`, Boxed
`ShockPropagation 1.48`, or FrictionIndex `Apgd 1.61` under the same
packet-like evidence rules. Do not push without explicit maintainer/user
approval.

## Current Reality - 2026-06-12 APGD/SAP Standard LLT Paths

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 72]`
- Last committed checkpoint:
  `4b9ad147c09 Use LLT for RedBlackGaussSeidel standard exact path`
- Checkpoint target:
  `Use LLT for APGD and SAP standard exact paths`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 73 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `ApgdSolver` and `SapSolver` route their Standard strict-interior exact paths
  through `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- The APGD 96-row focused benchmark remains on the iterative path; the observed
  timing movement there is benchmark noise, not acceptance evidence.
- The slice follows the PR #2986 harness stance captured below: evidence uses
  the current interim LCP solver identity path from `BM_LcpCompare` benchmark
  names, `contract_ok` counters, timing fields, manifest metadata, and native
  support checks.

Evidence:

- Baseline:
  `build/standard_sap_apgd_baseline.json`.
- Accepted focused probe:
  `build/standard_sap_apgd_llt_probe.json`.
- Focused Standard `Apgd` timings:
  - `Apgd/12`: `856.00ns -> 783.00ns`.
  - `Apgd/24`: `2893.00ns -> 2134.00ns`.
  - `Apgd/48`: `11232.00ns -> 8900.00ns`.
  - `Apgd/96`: `64871.00ns -> 58359.00ns`
    (iterative path; not used as acceptance evidence).
- Focused Standard `Sap` timings:
  - `Sap/12`: `958.00ns -> 777.00ns`.
  - `Sap/24`: `2843.00ns -> 2250.00ns`.
  - `Sap/48`: `11199.00ns -> 9113.00ns`.
  - `Sap/96`: `59969.00ns -> 40996.00ns`.
- Latest regenerated profile highlights:
  - Standard: no solver average is above `1.6x`; highest rows are
    `Jacobi 1.59`, `MPRGP 1.47`, `Sap 1.46`,
    `BoxedSemiSmoothNewton 1.43`, and `Apgd 1.40`.
  - Boxed: no solver average is above `1.6x`; highest rows are
    `RedBlackGaussSeidel 1.55`, `SymmetricPsor 1.54`,
    `ShockPropagation 1.43`, `NNCG 1.39`, and `Sap 1.35`.
  - FrictionIndex: no solver average is above `1.6x`; highest rows are
    `ShockPropagation 1.54`, `Sap 1.45`, `Apgd 1.44`, and
    `BoxedSemiSmoothNewton 1.37`.

Verification state:

- Completed so far:
  - Focused baseline and accepted focused probe for
    `BM_LcpCompare/Standard/(Sap|Apgd)/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
- Final pre-commit checks after any final edits:
  - `pixi run lint`.
  - `git diff --check`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
```

If this checkpoint is still uncommitted, run final lint/diff checks and commit
with `Use LLT for APGD and SAP standard exact paths`. If it is already
committed, investigate Standard `Jacobi 1.59`, Boxed
`RedBlackGaussSeidel 1.55`, or Boxed `SymmetricPsor 1.54` under the same
packet-like evidence rules. Do not push without explicit maintainer/user
approval.

## Current Reality - 2026-06-12 RedBlackGaussSeidel Standard LLT Path

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 71]`
- Last committed checkpoint:
  `2d1ae4bd041 Use LLT for BlockedJacobi standard exact path`
- Checkpoint target:
  `Use LLT for RedBlackGaussSeidel standard exact path`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 72 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `RedBlackGaussSeidelSolver` routes its Standard strict-interior exact path
  through `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- The 96-row focused benchmark remains on the iterative path; the observed
  timing movement there is benchmark noise, not acceptance evidence.
- The slice follows the PR #2986 harness stance captured below: evidence uses
  the current interim LCP solver identity path from `BM_LcpCompare` benchmark
  names, `contract_ok` counters, timing fields, manifest metadata, and native
  support checks.

Evidence:

- Baseline:
  `build/standard_redblack_baseline.json`.
- Accepted focused probe:
  `build/standard_redblack_llt_probe.json`.
- Focused Standard `RedBlackGaussSeidel` timings:
  - `RedBlackGaussSeidel/12`: `884.00ns -> 750.00ns`.
  - `RedBlackGaussSeidel/24`: `3096.00ns -> 1936.00ns`.
  - `RedBlackGaussSeidel/48`: `11944.00ns -> 8659.00ns`.
  - `RedBlackGaussSeidel/96`: `52695.00ns -> 56005.00ns`
    (iterative path; not used as acceptance evidence).
- Latest regenerated profile highlights:
  - Standard: `RedBlackGaussSeidel 1.39`; highest rows are `Sap 1.83`,
    `Apgd 1.64`, `MPRGP 1.59`, and `Jacobi 1.50`.
  - Boxed: highest rows are `NNCG 1.72`, `SymmetricPsor 1.52`,
    `Apgd 1.52`, `ShockPropagation 1.50`, and
    `RedBlackGaussSeidel 1.48`.
  - FrictionIndex: no solver average is above `1.6x`; highest rows are
    `Apgd 1.51`, `ShockPropagation 1.51`, `Sap 1.46`, and `Admm 1.40`.

Verification state:

- Completed so far:
  - Focused baseline and accepted focused probe for
    `BM_LcpCompare/Standard/RedBlackGaussSeidel/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
- Final pre-commit checks after any final edits:
  - `pixi run lint`.
  - `git diff --check`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
```

If this checkpoint is still uncommitted, run final lint/diff checks and commit
with `Use LLT for RedBlackGaussSeidel standard exact path`. If it is already
committed, investigate Standard `Sap 1.83`, Standard `Apgd 1.64`, or Boxed
`NNCG 1.72` under the same packet-like evidence rules. Do not push without
explicit maintainer/user approval.

## Current Reality - 2026-06-12 BlockedJacobi Standard LLT Path

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 70]`
- Last committed checkpoint:
  `f7a3a94dc09 Record PLAN-091 harness alignment for LCP work`
- Checkpoint target:
  `Use LLT for BlockedJacobi standard exact path`
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 71 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `BlockedJacobiSolver` routes its Standard strict-interior exact path through
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- A combined `Jacobi` + `BlockedJacobi` LLT-first probe was rejected for
  `Jacobi`; `JacobiSolver` stays on the existing LU-first helper in this slice.
- The slice follows the PR #2986 harness stance captured below: evidence uses
  the current interim LCP solver identity path from `BM_LcpCompare` benchmark
  names, `contract_ok` counters, timing fields, manifest metadata, and native
  support checks.

Evidence:

- Baseline:
  `build/standard_jacobi_blocked_baseline.json`.
- Rejected combined probe:
  `build/standard_jacobi_blocked_llt_probe.json`.
- Accepted focused probe:
  `build/standard_blocked_jacobi_llt_probe.json`.
- Focused Standard `BlockedJacobi` timings:
  - `BlockedJacobi/12`: `827.32ns -> 816.27ns`.
  - `BlockedJacobi/24`: `2887.44ns -> 2252.69ns`.
  - `BlockedJacobi/48`: `11828.52ns -> 9120.27ns`.
  - `BlockedJacobi/96`: `63323.76ns -> 40585.69ns`.
- Latest regenerated profile highlights:
  - Standard: `BlockedJacobi 1.18`; highest rows are `Jacobi 1.66`,
    `RedBlackGaussSeidel 1.63`, `Apgd 1.56`, and `Sap 1.51`.
  - Boxed: highest rows are `Sap 1.68`, `RedBlackGaussSeidel 1.53`,
    `SymmetricPsor 1.50`, and `ShockPropagation 1.48`.
  - FrictionIndex: highest rows are `ShockPropagation 1.65`, `Sap 1.64`,
    `Apgd 1.59`, `NNCG 1.53`, and `SubspaceMinimization 1.52`.

Verification state:

- Completed so far:
  - Focused baseline, rejected combined probe, and accepted focused probe for
    `BM_LcpCompare/Standard/(Jacobi|BlockedJacobi)/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
- Final pre-commit checks after any final edits:
  - `pixi run lint`.
  - `git diff --check`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
```

If this checkpoint is still uncommitted, run final lint/diff checks and commit
with `Use LLT for BlockedJacobi standard exact path`. If it is already
committed, investigate Standard `Jacobi 1.66`, Standard
`RedBlackGaussSeidel 1.63`, or Boxed `Sap 1.68` under the same packet-like
evidence rules. Do not push without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Post-PLAN-091 Merge / Harness Alignment

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 69]`
- Latest local commit:
  `b6b1b12544e Merge remote-tracking branch 'origin/main' into feature/lcp-solver-interface-demos`
- Latest LCP implementation checkpoint:
  `d35a3ffa099 Use LLT for Lemke and Baraff standard exact paths`
- Latest `origin/main` was fetched via HTTPS because SSH to GitHub was not
  reachable, then merged locally.
- The merged `origin/main` contains PR #2986:
  `bb851f45360 Add DART 7 architecture assessment, PLAN-091 hardening plan, and work-packet harness (#2986)`.
- There is no associated PR yet.
- This branch has not been pushed after the merge.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness constraints now active:

- Follow `docs/ai/orchestration.md` by keeping continuation slices packet-like:
  one objective, bounded scope, explicit acceptance evidence, and a clear
  handoff.
- Follow `docs/design/dart7_architecture_assessment.md` and
  `docs/plans/091-architecture-hardening.md` for the DART 7 gaps around solver
  contracts and apples-to-apples comparison substrate.
- Follow `docs/plans/solver-family-intake.md` items 9-10 for solver-family
  contract entry and machine-recorded benchmark identity.
- This LCP task predates PLAN-091. Until a durable resolved-identity schema
  lands, record the interim identity path explicitly: `BM_LcpCompare` benchmark
  names encode problem family, manifest solver name, and benchmark size;
  `scripts/lcp_performance_profile.py` derives profile rows from those names
  plus `contract_ok` and timing fields; `lcp_physics.py` exposes solver manifest
  metadata and native support checks for demo panels.

Latest verified checkpoint before the merge:

- `d35a3ffa099` moved `LemkeSolver` and `BaraffSolver` Standard
  strict-interior exact paths to
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- Pre-merge gates passed: focused Python demo metadata test, CSV shape check,
  focused C++ build/CTest for LCP validation, `pixi run build`,
  `pixi run test-unit` (`161/161`), `pixi run lint`, and `git diff --check`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
```

If continuing optimization, start with a focused baseline for Standard `Jacobi`
and `BlockedJacobi`, or choose Boxed `NNCG`. Any new evidence packet should
state the current interim solver identity path above. Do not push without
explicit maintainer/user approval.

## Current Reality - 2026-06-12 Lemke/Baraff Standard LLT Paths

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 75]`
- Last committed checkpoint:
  `e60b8563366 Use LLT for SubspaceMinimization standard exact path`.
- Checkpoint target:
  `Use LLT for Lemke and Baraff standard exact paths`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 76 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.
- Latest continuation resumed after the hand-off-only stop request and is
  closing this checkpoint locally.

Current dirty files before commit:

- `CHANGELOG.md`
- `dart/math/lcp/pivoting/baraff_solver.cpp`
- `dart/math/lcp/pivoting/lemke_solver.cpp`
- `docs/background/lcp/03_pivoting-methods.md`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `tests/unit/math/lcp/test_additional_solvers.cpp`
- `tests/unit/math/lcp/test_lcp_generated_coverage.cpp`

What this slice changes:

- `LemkeSolver` and `BaraffSolver` route their Standard strict-interior exact
  paths through `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- Existing pivot/barrier exact-path unit coverage exercises the same accepted
  strict-interior solve route.
- The regenerated full profile moves Standard `Baraff` to `1.15x` and Standard
  `Lemke` to `1.25x`.

Evidence:

- Baseline:
  `build/standard_lemke_baraff_baseline.json`.
- Accepted focused probe:
  `build/standard_lemke_baraff_llt_probe.json`.
- Focused Standard Baraff timings:
  - `Baraff/12`: `893.58ns -> 855.07ns`.
  - `Baraff/24`: `3105.49ns -> 2351.73ns`.
  - `Baraff/48`: `12212.39ns -> 9582.00ns`.
  - `Baraff/96`: `63965.92ns -> 42414.44ns`.
- Focused Standard Lemke timings:
  - `Lemke/12`: `908.48ns -> 724.84ns`.
  - `Lemke/24`: `3315.46ns -> 2169.25ns`.
  - `Lemke/48`: `12320.24ns -> 9214.86ns`.
  - `Lemke/96`: `64622.48ns -> 43291.53ns`.
- Latest regenerated profile highlights:
  - Standard: `Baraff 1.15`, `Lemke 1.25`; highest rows are
    `Jacobi 1.64`, `BlockedJacobi 1.62`, `Apgd 1.59`,
    `RedBlackGaussSeidel 1.57`, and `Sap 1.57`.
  - Boxed: highest rows are `NNCG 1.71`, `RedBlackGaussSeidel 1.66`,
    `SymmetricPsor 1.63`, `ShockPropagation 1.59`,
    `BoxedSemiSmoothNewton 1.55`, and `SubspaceMinimization 1.51`.
  - FrictionIndex: highest rows are `ShockPropagation 1.71`, `Admm 1.59`,
    `NNCG 1.58`, `Sap 1.56`, and `Apgd 1.53`.

Rejected probes from this continuation:

- Boxed `SymmetricPsor` projected-active-set exact helper:
  `build/boxed_symmetric_psor_baseline.json` vs
  `build/boxed_symmetric_psor_exact_probe.json`; it was slightly faster at 12
  rows but slower at 24 and 48 rows.
- Boxed `SymmetricPsor` relaxation sweep:
  `build/boxed_symmetric_psor_relaxation_sweep.json`; the existing
  `relaxation=1.0` row remained the best tracked boxed sweep point.
- Boxed `RedBlackGaussSeidel` projected-active-set exact helper:
  `build/boxed_redblack_baseline.json`,
  `build/boxed_redblack_exact_probe.json`, and
  `build/boxed_redblack_gate24_probe.json`; focused 12/24-row gains did not
  survive the aggregate profile, so the probe was reverted.

Verification state:

- Completed so far:
  - Focused baseline and LLT probe for
    `BM_LcpCompare/Standard/(Lemke|Baraff)/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Fixed stale Python demo profile metadata and assertions:
    removed `MPRGP` from the Standard `current_laggards` text, aligned Boxed
    `SubspaceMinimization` with the laggard text, and aligned FrictionIndex
    `Admm` with the refreshed next-largest-row text.
  - Focused Python demo metadata test:
    `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
    passed.
  - CSV shape check: `Boxed 15/200`, `FrictionIndex 16/200`,
    `Standard 23/200`.
  - `pixi run build` passed with `JOBS=5`.
  - Focused rebuild and CTest for
    `UNIT_math_lcp_math_lcp_additional_solvers` and
    `UNIT_math_lcp_math_lcp_lcp_generated_coverage` passed after aligning
    tests with exact-fast-path behavior.
  - `pixi run test-unit` passed: `161/161` tests.
  - `pixi run lint` passed with `JOBS=5`.
  - `git diff --check` passed.
- Resolved failures seen during this continuation:
  - `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  - Initial failure: the Standard demo metadata still said `MPRGP` is a
    next-largest row, while the test asserted `MPRGP` should not appear in that
    text.
  - `pixi run test-unit` initially failed in
    `UNIT_math_lcp_math_lcp_additional_solvers` and
    `UNIT_math_lcp_math_lcp_lcp_generated_coverage`; the max-iteration tests
    now warm-start the iterative path to avoid the cold exact fast paths, and
    the near-singular boxed generated case now treats residual and
    complementarity as the contract instead of over-constraining the expected
    vector.
- Still required before commit:
  - Create the local checkpoint commit.
- No push has been performed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, commit with
`Use LLT for Lemke and Baraff standard exact paths`. If this checkpoint is
already committed, investigate Standard `Jacobi 1.64` and Standard
`BlockedJacobi 1.62`, or Boxed `NNCG 1.71`. Do not push without explicit
maintainer/user approval.

## Current Reality - 2026-06-12 SubspaceMinimization Standard LLT Path

This is the latest hand-off. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 74]`
- Last committed checkpoint:
  `7d53d6a16d7 Use LLT for Dantzig and Symmetric PSOR exact paths`.
- Checkpoint target:
  `Use LLT for SubspaceMinimization standard exact path`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 75 commits.
- There is no associated PR yet.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

What this slice changes:

- `SubspaceMinimizationSolver` routes its Standard strict-interior exact path
  through `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- Unit coverage adds `SubspaceMinimizationSolver` to the 96-size Standard
  exact-path regression with `maxIterations = 1` and zero expected iterations.
- The regenerated full profile moves Standard `SubspaceMinimization` from the
  largest remaining Standard row to `1.19x`; no Standard solver average is now
  above `1.6x`.

Evidence:

- Baseline:
  `build/standard_subspace_baseline.json`.
- Accepted focused probe:
  `build/standard_subspace_llt_probe.json`.
- Focused Standard SubspaceMinimization timings:
  - `SubspaceMinimization/12`: `1999.23ns -> 952.73ns`.
  - `SubspaceMinimization/24`: `5719.05ns -> 2434.37ns`.
  - `SubspaceMinimization/48`: `23889.67ns -> 12128.22ns`.
  - `SubspaceMinimization/96`: `149798.16ns -> 48358.45ns`.
- Latest regenerated profile highlights:
  - Standard: no solver above `1.6x`; highest rows are `Baraff 1.59`,
    `BlockedJacobi 1.59`, `Lemke 1.56`, `RedBlackGaussSeidel 1.54`,
    `Sap 1.53`, `Apgd 1.49`, and `Jacobi 1.48`.
  - Boxed: `SymmetricPsor 2.08` is now the largest row; next rows are
    `RedBlackGaussSeidel 1.68`, `Jacobi 1.54`, `ShockPropagation 1.45`,
    `Apgd 1.41`, and `BlockedJacobi 1.39`.
  - FrictionIndex: no solver above `1.6x`; highest rows are
    `SubspaceMinimization 1.57`, `ShockPropagation 1.51`, `NNCG 1.51`,
    `Apgd 1.50`, `RedBlackGaussSeidel 1.47`, and `Sap 1.42`.

Verification state:

- Completed so far:
  - Focused baseline and LLT probe for
    `BM_LcpCompare/Standard/SubspaceMinimization/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test.
  - CSV shape check: `Boxed 15/200`, `FrictionIndex 16/200`,
    `Standard 23/200`.
- Final pre-commit checks after any final edits:
  - `pixi run lint`.
  - `git diff --check`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, run final lint/diff checks after any
edits and commit with `Use LLT for SubspaceMinimization standard exact path`.
If this checkpoint is already committed, investigate Boxed `SymmetricPsor 2.08`,
then Boxed `RedBlackGaussSeidel 1.68` or FrictionIndex
`SubspaceMinimization 1.57`. Do not push without explicit maintainer/user
approval.

## Current Reality - 2026-06-12 Dantzig/Symmetric PSOR Standard LLT Paths

This is the latest hand-off after resuming the previously stopped
Dantzig/Symmetric PSOR slice. Older sections below are historical checkpoints
and may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Local branch relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 73]`
- Last committed checkpoint:
  `ce7dbf7aa58 Raise ADMM friction exact gate`.
- Checkpoint target:
  `Use LLT for Dantzig and Symmetric PSOR exact paths`.
- There is no associated PR yet.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 73 commits, with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 74
  commits.
- This slice has not been pushed.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

Dirty worktree at hand-off:

- `dart/math/lcp/pivoting/dantzig_solver.cpp`
- `dart/math/lcp/projection/symmetric_psor_solver.cpp`
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`
- `docs/background/lcp/03_pivoting-methods.md`
- `docs/background/lcp/04_projection-methods.md`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `CHANGELOG.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

What this uncommitted slice changes:

- `DantzigSolver` routes its Standard strict-interior exact path through
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- `SymmetricPsorSolver` routes its Standard strict-interior exact path through
  the LLT-first helper and raises the Standard strict-interior gate from 48 to
  96 variables.
- Unit coverage adds `SymmetricPsorSolver` to the 96-size Standard exact-path
  regression with `maxIterations = 1` and zero expected iterations.
- Background LCP docs and Python demo metadata have been partially updated to
  reflect the latest regenerated profile.

Known unfinished item:

- No known focused validation failures remain after the metadata assertion fix.
- If this checkpoint is already committed, continue with the refreshed profile's
  highest remaining rows: Standard `SubspaceMinimization 1.64`, Standard
  `Baraff/Jacobi/Sap 1.57`, Standard `Lemke 1.54`, Boxed
  `ShockPropagation 1.51`, or FrictionIndex `ShockPropagation 1.53`.

Evidence already collected before the stop request:

- Baseline:
  `build/standard_dantzig_sympsor_baseline.json`.
- Accepted focused probe:
  `build/standard_dantzig_sympsor_llt_probe.json`.
- Focused Standard Dantzig timings:
  - `Dantzig/12`: `848.38ns -> 784.74ns`.
  - `Dantzig/24`: `2751.88ns -> 1959.32ns`.
  - `Dantzig/48`: `11071.67ns -> 8444.85ns`.
  - `Dantzig/96`: `59370.01ns -> 39535.71ns`.
- Focused Standard Symmetric PSOR timings:
  - `SymmetricPsor/12`: `991.57ns -> 878.19ns`.
  - `SymmetricPsor/24`: `3454.77ns -> 2760.61ns`.
  - `SymmetricPsor/48`: `12911.97ns -> 10933.11ns`.
  - `SymmetricPsor/96`: `51214.00ns -> 45922.66ns`.
- Latest regenerated profile highlights:
  - Standard: `Dantzig 1.02`, `SymmetricPsor 1.26`,
    `SubspaceMinimization 1.64`, `Baraff/Jacobi/Sap 1.57`,
    `Lemke 1.54`, `BlockedJacobi/RedBlackGaussSeidel 1.51`,
    and `Apgd 1.50`.
  - Boxed: no solver above `1.6x`; highest rows are
    `ShockPropagation 1.51`, `SymmetricPsor 1.47`,
    `RedBlackGaussSeidel 1.47`, `BGS 1.36`, and `BlockedJacobi 1.36`.
  - FrictionIndex: no solver above `1.6x`; highest rows are
    `ShockPropagation 1.53`, `Sap 1.44`, `Apgd 1.43`,
    `Admm 1.37`, and `SymmetricPsor 1.36`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
  pixi run lint
```

The CSV shape check reported `Boxed 15/200`, `FrictionIndex 16/200`, and
`Standard 23/200`. Run `git diff --check` after any final edits before
committing.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, run `git diff --check` after any final
edits and commit with `Use LLT for Dantzig and Symmetric PSOR exact paths`. If
this checkpoint is already committed, continue with Standard
`SubspaceMinimization 1.64`, Standard `Baraff/Jacobi/Sap 1.57`, Standard
`Lemke 1.54`, Boxed `ShockPropagation 1.51`, or FrictionIndex
`ShockPropagation 1.53`. Do not push without explicit maintainer/user approval.

## Current Reality - 2026-06-12 ADMM FrictionIndex Exact Gate

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `7dad05ea3aa Raise BGS friction exact gate`.
- Current checkpoint target:
  `Raise ADMM friction exact gate`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 72 commits with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 73
  commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

What this slice changes:

- `AdmmSolver` raises its strict-interior friction-index exact gate from 12 to
  48 variables. This covers the current 16-contact FrictionIndex comparison
  packet while leaving the large 64-contact row on the operator-splitting path.
- A 192-variable probe was rejected because it helped the medium row but did
  not materially improve the large row and had worse regression risk.
- Unit coverage adds a 16-contact FrictionIndex ADMM exact-path regression with
  `maxIterations = 1` and expects zero iterations.
- The regenerated full profile moved FrictionIndex `Admm` from `1.70x` to
  `1.41x`. The largest remaining rows are now Standard `Dantzig 1.73`,
  Standard `SymmetricPsor 1.71`, Standard `Baraff 1.61`, FrictionIndex
  `Apgd 1.57`, and FrictionIndex `ShockPropagation 1.55`.

Evidence:

- Baseline:
  `build/friction_index_admm_baseline.json`.
- Rejected large gate probe:
  `build/friction_index_admm_gate192_probe.json`.
- Accepted medium gate probe:
  `build/friction_index_admm_gate48_probe.json`.
- Focused FrictionIndex timings moved approximately:
  - `Admm/4`: `1103.20ns -> 1151.38ns`.
  - `Admm/16`: `19081.82ns -> 12127.16ns`.
  - `Admm/64`: `281190.62ns -> 283267.40ns`.
- Latest regenerated profile highlights:
  - Standard: `Dantzig 1.73`, `SymmetricPsor 1.71`, `Baraff 1.61`,
    `Apgd 1.56`, with `Admm 1.09`.
  - Boxed: no solver above `1.5x`; highest rows are
    `SymmetricPsor 1.46`, `ShockPropagation 1.43`, and
    `RedBlackGaussSeidel 1.42`.
  - FrictionIndex: no solver above `1.6x`; `Admm 1.41`; highest rows are
    `Apgd 1.57`, `ShockPropagation 1.55`, and `Sap 1.46`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
  pixi run lint
git diff --check
```

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, review the expected dirty files and
commit with `Raise ADMM friction exact gate`. If this checkpoint is already
committed, continue with Standard `Dantzig 1.73`, Standard
`SymmetricPsor 1.71`, Standard `Baraff 1.61`, FrictionIndex `Apgd 1.57`, or
FrictionIndex `ShockPropagation 1.55`. Do not push or mutate GitHub state
without explicit maintainer/user approval.

## Current Reality - 2026-06-12 BGS FrictionIndex Exact Gate

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `f27dad169a7 Use LLT for boxed active-set exact helper`.
- Current checkpoint target:
  `Raise BGS friction exact gate`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 71 commits with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 72
  commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

What this slice changes:

- `BgsSolver` raises its strict-interior exact gate from 96 to 192 variables so
  the current 64-contact FrictionIndex comparison packet exits through the
  shared LLT-first friction exact helper.
- Unit coverage adds a 64-contact FrictionIndex BGS exact-path regression with
  `maxIterations = 1` and expects zero iterations.
- The regenerated full profile moved FrictionIndex `BGS` from the largest row
  (`1.65x` before this slice) to `1.24x`. The largest remaining rows are now
  FrictionIndex `Admm 1.70`, Boxed `SymmetricPsor 1.62`, FrictionIndex
  `ShockPropagation 1.57`, FrictionIndex `Sap 1.55`, and Standard
  `Lemke 1.54`.

Evidence:

- Baseline:
  `build/friction_index_bgs_baseline.json`.
- Accepted focused probe:
  `build/friction_index_bgs_gate192_probe.json`.
- Focused FrictionIndex timings moved approximately:
  - `BGS/4`: `1629.88ns -> 1179.10ns`.
  - `BGS/16`: `12570.26ns -> 12323.68ns`.
  - `BGS/64`: `330609.46ns -> 254061.50ns`.
- Latest regenerated profile highlights:
  - Standard: no solver above `1.54x`; highest rows are `Lemke 1.54` and
    `Jacobi 1.53`.
  - Boxed: `SymmetricPsor 1.62` is the only solver above `1.6x`;
    `RedBlackGaussSeidel 1.51` and `ShockPropagation 1.51` are next.
  - FrictionIndex: `BGS 1.24`; highest rows are `Admm 1.70`,
    `ShockPropagation 1.57`, `Sap 1.55`, and `Apgd 1.42`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
  pixi run lint
git diff --check
```

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, review the expected dirty files and
commit with `Raise BGS friction exact gate`. If this checkpoint is already
committed, continue with FrictionIndex `Admm 1.70`, Boxed
`SymmetricPsor 1.62`, FrictionIndex `ShockPropagation 1.57`, FrictionIndex
`Sap 1.55`, or Standard `Lemke 1.54`. Do not push or mutate GitHub state
without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Boxed Exact Helper LLT Path

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Checkpoint note: this section records the boxed-helper LLT slice and its
focused validation evidence. The checkpoint commit should be titled
`Use LLT for boxed active-set exact helper`.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `13af6a5463c Use LLT for standard BGS and NNCG exact paths`.
- Current checkpoint target:
  `Use LLT for boxed active-set exact helper`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 70 commits with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 71
  commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.
- Pre-commit worktree for this checkpoint contained the boxed-helper slice and
  its generated docs/demo/profile metadata updates. Expected modified files
  before the checkpoint commit were `CHANGELOG.md`,
  `dart/math/lcp/lcp_validation.hpp`,
  `docs/background/lcp/04_projection-methods.md`,
  `docs/background/lcp/figures/performance_profile_boxed.csv`,
  `docs/background/lcp/figures/performance_profile_frictionindex.csv`,
  `docs/background/lcp/figures/performance_profile_standard.csv`,
  `docs/dev_tasks/lcp_solver_interface_demos/README.md`,
  `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`,
  `python/examples/demos/scenes/lcp_physics.py`, and
  `python/tests/unit/test_py_demo_panels.py`.

What this slice changes:

- The shared projected-active-set boxed-LCP exact helper tries LLT-based dense
  solves for the unconstrained and reduced free-row systems.
- If the LLT-based active-set candidate does not validate, the helper falls
  back to the previous LU path, preserving the validated acceptance contract.
- The refreshed Boxed profile has no solver above `1.51x`; the previous Boxed
  `ShockPropagation` and `BGS` above-`2x` rows are gone.

Evidence:

- Baseline:
  `build/boxed_exact_helper_baseline.json`.
- Accepted focused probe:
  `build/boxed_exact_helper_llt_probe.json`.
- Focused Boxed timings moved approximately:
  - `BGS/24`: `4004.38ns -> 3838.84ns`, `iterations 0 -> 0`.
  - `BGS/48`: `16886.30ns -> 13481.80ns`, `iterations 0 -> 0`.
  - `ShockPropagation/24`: `4153.02ns -> 3947.97ns`, `iterations 0 -> 0`.
  - `ShockPropagation/48`: `15515.32ns -> 13510.71ns`, `iterations 0 -> 0`.
  - `Dantzig/24`: `4368.56ns -> 2835.80ns`, `iterations 0 -> 0`.
  - `Sap/48`: `14455.75ns -> 11305.99ns`, `iterations 0 -> 0`.
- Latest regenerated profile highlights:
  - Standard: no solver above `1.55x`; highest row is `Sap 1.55`.
  - Boxed: no solver above `1.51x`; highest rows are
    `RedBlackGaussSeidel 1.51` and `ShockPropagation 1.50`.
  - FrictionIndex: no solver above `2x`; highest rows are `BGS 1.65`,
    `BoxedSemiSmoothNewton 1.52`, `Admm 1.50`, and
    `ShockPropagation 1.50`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
  pixi run lint
git diff --check
```

CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
columns.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, review the expected dirty files listed
above, run the required pre-commit checks, and commit with
`Use LLT for boxed active-set exact helper`. If this checkpoint is already
committed, continue with FrictionIndex `BGS 1.65`,
FrictionIndex `BoxedSemiSmoothNewton 1.52`, or Standard `Sap 1.55`. Do not
push or mutate GitHub state without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Standard BGS/NNCG LLT Exact Paths

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `b57e03cdee1 Use LLT for standard Newton exact paths`.
- Current checkpoint target:
  `Use LLT for standard BGS and NNCG exact paths`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 69 commits with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 70
  commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

What this slice changes:

- `NncgSolver` uses the validated LLT-first strict-interior Standard exact
  helper for default non-warm-started Standard rows.
- `BgsSolver` uses the same LLT-first Standard exact helper and raises its
  Standard exact gate to 96 variables so the current 96-row Standard comparison
  packet exits through the exact path.
- Unit coverage adds a 96-row Standard exact-path regression for both solvers.
- The regenerated profile now reports every Standard solver at or below
  `1.50x` average. Boxed `ShockPropagation 2.18` and Boxed `BGS 2.12` are the
  next profile-driven targets.

Evidence:

- Baseline:
  `build/standard_nncg_bgs_baseline.json`.
- Accepted focused probe:
  `build/standard_nncg_bgs_llt_probe.json`.
- Focused Standard timings moved approximately:
  - `BGS/12`: `917.19ns -> 865.36ns`, `iterations 0 -> 0`.
  - `BGS/24`: `3300.26ns -> 2380.74ns`, `iterations 0 -> 0`.
  - `BGS/48`: `13747.08ns -> 9523.33ns`, `iterations 0 -> 0`.
  - `BGS/96`: `74648.65ns -> 42148.97ns`, `iterations 5 -> 0`.
  - `NNCG/12`: `1153.89ns -> 875.81ns`, `iterations 0 -> 0`.
  - `NNCG/24`: `3267.90ns -> 2443.98ns`, `iterations 0 -> 0`.
  - `NNCG/48`: `12084.66ns -> 10303.36ns`, `iterations 0 -> 0`.
  - `NNCG/96`: `63547.04ns -> 45197.60ns`, `iterations 0 -> 0`.
- Latest regenerated profile highlights:
  - Standard: `BGS 1.12`, `NNCG 1.17`, and no Standard solver above `1.50x`.
  - Boxed: `ShockPropagation 2.18` and `BGS 2.12` are above `2x`.
  - FrictionIndex: no solver above `2x`; highest rows are `Admm 1.62`,
    `ShockPropagation 1.55`, and `Sap 1.50`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
columns.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, review and checkpoint it with
`Use LLT for standard BGS and NNCG exact paths`. If this checkpoint is already
committed, continue with Boxed `ShockPropagation 2.18` and Boxed `BGS 2.12`.
Do not push or mutate GitHub state without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Standard LLT Exact Paths

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `a1f83f6fe43 Extend Jacobi and RedBlackGaussSeidel friction exact paths`.
- Current checkpoint target:
  `Use LLT for standard Newton exact paths`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 68 commits, with the current
  Standard LLT slice uncommitted. After this checkpoint is committed, it should
  be ahead by 69 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

What this uncommitted slice changes:

- `dart/math/lcp/lcp_validation.hpp` keeps the default
  `trySolveStrictInteriorStandardLcp(...)` on the prior LU-based path.
- A new validated `trySolveStrictInteriorStandardLcpLltFirst(...)` path tries
  `Eigen::LLT` first and falls back to the LU solve if LLT fails or the
  candidate fails the existing acceptance checks.
- This LLT-first helper is now used by dense Standard exact fast paths in:
  `InteriorPointSolver`, `FischerBurmeisterNewtonSolver`,
  `MinimumMapNewtonSolver`, `PenalizedFischerBurmeisterNewtonSolver`, and
  `BoxedSemiSmoothNewtonSolver`.
- The broad shared-helper LLT change was intentionally narrowed because it
  improved dense Newton/interior-point rows but risked undesirable effects for
  low-overhead exact-path users. The default helper remains conservative.
- Profile CSVs, Python demo profile summary strings, Python metadata
  assertions, projection-method docs, changelog, and hand-off docs were
  refreshed.

Files changed by this checkpoint:

- `CHANGELOG.md`
- `dart/math/lcp/lcp_validation.hpp`
- `dart/math/lcp/other/interior_point_solver.cpp`
- `dart/math/lcp/newton/fischer_burmeister_newton_solver.cpp`
- `dart/math/lcp/newton/minimum_map_newton_solver.cpp`
- `dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.cpp`
- `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.cpp`
- `docs/background/lcp/04_projection-methods.md`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Profile evidence gathered before final verification:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
```

Relevant artifacts:

- Baseline:
  `build/standard_interior_penalized_baseline.json`.
- Broad shared-helper LLT probe, not the final narrowed implementation:
  `build/standard_llt_exact_probe.json`.
- Accepted narrowed focused probe:
  `build/standard_newton_llt_exact_probe.json`.
- Latest full profile:
  `build/lcp_profile_full.json`.

Focused Standard timings from the narrowed probe:

- `InteriorPoint/24`: `3219.35ns -> 2450.15ns`.
- `InteriorPoint/48`: `13020.26ns -> 10452.61ns`.
- `InteriorPoint/96`: `81475.85ns -> 46214.64ns`.
- `PenalizedFischerBurmeisterNewton/24`: `3268.03ns -> 2546.23ns`.
- `PenalizedFischerBurmeisterNewton/96`: `66040.66ns -> 47382.26ns`.
- `FischerBurmeisterNewton/24`: `3278.38ns -> 2452.76ns`.
- `FischerBurmeisterNewton/96`: `66017.55ns -> 47803.73ns`.
- `MinimumMapNewton/24`: `3352.45ns -> 2467.01ns`.
- `MinimumMapNewton/96`: `67322.47ns -> 46319.16ns`.

Latest regenerated profile highlights:

- Standard high rows after the LLT-first narrowing:
  `NNCG 2.00`, `BGS 1.97`, `SubspaceMinimization 1.62`,
  `Baraff 1.58`, `Lemke 1.56`, `Dantzig 1.49`, `Apgd 1.46`,
  `Jacobi 1.46`, `RedBlackGaussSeidel 1.46`, and `Sap 1.43`.
- Standard dense Newton/interior-point rows are now around:
  `FischerBurmeisterNewton 1.17`, `InteriorPoint 1.17`,
  `PenalizedFischerBurmeisterNewton 1.17`,
  `BoxedSemiSmoothNewton 1.16`, and `MinimumMapNewton 1.25`.
- Boxed high rows:
  `BGS 1.70`, `ShockPropagation 1.67`,
  `BoxedSemiSmoothNewton 1.65`, `Sap 1.65`, `NNCG 1.63`,
  and `Apgd 1.62`.
- FrictionIndex high rows:
  `Admm 1.64`, `Apgd 1.63`, `ShockPropagation 1.62`,
  `BGS 1.60`, `Sap 1.50`, and `NNCG 1.47`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
columns.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, review and checkpoint it with
`Use LLT for standard Newton exact paths`. If this checkpoint is already
committed, continue with the refreshed profile's highest remaining rows:
Standard `NNCG 2.00`, Standard `BGS 1.97`, Boxed `BGS 1.70`, Boxed
`ShockPropagation 1.67`, or FrictionIndex `Admm 1.64`. Do not push or mutate
GitHub state without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Jacobi/RBGS FrictionIndex Exact Paths

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `d00106704aa Optimize friction exact helper with LLT`.
- Current checkpoint target:
  `Extend Jacobi and RedBlackGaussSeidel friction exact paths`.
- Pre-commit state: the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 67 commits with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 68
  commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

What this slice changes:

- `JacobiSolver` and `RedBlackGaussSeidelSolver` now try the shared validated
  LLT-first strict-interior friction-index exact helper for default,
  non-warm-started solves up to 192 variables, covering the current
  64-contact FrictionIndex comparison packet.
- Their standard-LCP exact gates are unchanged. Boxed paths remain iterative
  because the current boxed profile does not justify adding dense boxed exact
  probes to these solvers.
- Unit coverage adds both solvers to the small FrictionIndex exact-path smoke
  coverage and adds large 64-contact exact-path regressions for each solver.
- The checked profile CSVs, Python demo profile summaries, metadata
  assertions, projection-method docs, changelog, and hand-off docs were
  refreshed.

Evidence:

- Baseline:
  `build/friction_index_rbg_jacobi_baseline.json`.
- Accepted focused probe:
  `build/friction_index_rbg_jacobi_exact_probe.json`.
- Focused FrictionIndex timings moved approximately:
  - `Jacobi/4`: `1748.16ns -> 1089.70ns`, `iterations 8 -> 0`.
  - `Jacobi/16`: `12519.10ns -> 11009.86ns`, `iterations 12 -> 0`.
  - `Jacobi/64`: `356776.39ns -> 236860.91ns`, `iterations 17 -> 0`.
  - `RedBlackGaussSeidel/4`: `1533.68ns -> 1047.22ns`,
    `iterations 5 -> 0`.
  - `RedBlackGaussSeidel/16`: `16208.84ns -> 12713.89ns`,
    `iterations 8 -> 0`.
  - `RedBlackGaussSeidel/64`: `295732.70ns -> 253038.00ns`,
    `iterations 7 -> 0`.
- Regenerated `build/lcp_profile_full.json` reports FrictionIndex
  `Jacobi 1.23` and `RedBlackGaussSeidel 1.27`. The largest FrictionIndex
  averages are now `ShockPropagation 1.88`, `Admm 1.64`, `Sap 1.62`,
  `BGS 1.55`, `Apgd 1.49`, `NNCG 1.48`, and
  `SubspaceMinimization 1.47`.
- The same regenerated profile reports Boxed `RedBlackGaussSeidel 1.95` and
  `ShockPropagation 1.81`, and Standard `InteriorPoint 2.68` with
  `PenalizedFischerBurmeisterNewton` around `2x`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(RedBlackGaussSeidel|Jacobi|Pgs|Tgs|Dantzig|Admm|BGS|Apgd|Sap)/' \
  --benchmark_min_time=0.1s \
  --benchmark_format=json > build/friction_index_rbg_jacobi_exact_probe.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
```

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

Finish this checkpoint by running the focused Python metadata test, CSV shape
check, required `pixi run lint`, and `git diff --check`. If clean, commit with
`Extend Jacobi and RedBlackGaussSeidel friction exact paths`. If this
checkpoint is already committed, continue with Standard `InteriorPoint 2.68`,
Standard `PenalizedFischerBurmeisterNewton ~2.00`, Boxed
`RedBlackGaussSeidel 1.95`, FrictionIndex `ShockPropagation 1.88`, or Boxed
`ShockPropagation 1.81`. Do not push or mutate GitHub state without explicit
maintainer/user approval.

## Current Reality - 2026-06-12 LLT-First FrictionIndex Exact Helper

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `e024c24a20a Raise ShockPropagation friction exact gate`.
- Current checkpoint target:
  `Optimize friction exact helper with LLT`.
- Pre-commit resume state: this branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 66 commits with the LLT helper
  slice uncommitted.
- After this checkpoint is committed, the branch should become ahead of
  `origin/feature/lcp-solver-interface-demos` by 67 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

What this slice changes:

- `detail::trySolveInteriorFrictionIndexLcp(...)` now tries an
  `Eigen::LLT` solve first for SPD strict-interior friction-index rows.
- If LLT fails or the candidate fails the existing finite, bound, or solution
  validation checks, the helper falls back to the previous `partialPivLu()`
  solve.
- This keeps the exact helper's acceptance contract unchanged while reducing
  the dense-solve cost for current SPD FrictionIndex comparison packets.
- The checked profile CSVs, Python demo profile summaries, metadata assertions,
  projection-method docs, changelog, and hand-off docs were refreshed.
- Files changed by this checkpoint:
  `CHANGELOG.md`, `dart/math/lcp/lcp_validation.hpp`,
  `docs/background/lcp/04_projection-methods.md`,
  `docs/background/lcp/figures/performance_profile_boxed.csv`,
  `docs/background/lcp/figures/performance_profile_frictionindex.csv`,
  `docs/background/lcp/figures/performance_profile_standard.csv`,
  `docs/dev_tasks/lcp_solver_interface_demos/README.md`,
  `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`,
  `python/examples/demos/scenes/lcp_physics.py`, and
  `python/tests/unit/test_py_demo_panels.py`.

Evidence:

- Focused probe:
  `build/friction_index_llt_exact_probe.json`.
- Focused 64-contact FrictionIndex exact rows improved approximately:
  - `BlockedJacobi`: `631253.79ns -> 254343.08ns`.
  - `BoxedSemiSmoothNewton`: `430872.80ns -> 248047.48ns`.
  - `NNCG`: `459364.02ns -> 254507.79ns`.
  - `ShockPropagation`: `466220.22ns -> 270455.13ns`.
- Regenerated `build/lcp_profile_full.json` reports no FrictionIndex solver
  average above `2x`; the largest FrictionIndex averages are now
  `RedBlackGaussSeidel 1.81`, `Jacobi 1.77`, `Admm 1.65`,
  `ShockPropagation 1.60`, `BGS 1.45`, `Sap 1.44`, and `Apgd 1.41`.
- CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
  columns.

Verification and checks already gathered for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(BlockedJacobi|BoxedSemiSmoothNewton|Dantzig|ShockPropagation|BGS|NNCG|SubspaceMinimization|Pgs|Tgs|SymmetricPsor|Sap|Apgd|Admm)/' \
  --benchmark_min_time=0.1s \
  --benchmark_format=json > build/friction_index_llt_exact_probe.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

A `pixi run lint` command completed successfully during the earlier hand-off
turn. Because this checkpoint edits hand-off docs afterward, rerun the required
pre-commit lint and diff checks before committing.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is still uncommitted, first checkpoint the LLT helper slice
with `Optimize friction exact helper with LLT`. If this checkpoint is already
committed, continue with the refreshed profile's highest remaining rows:
FrictionIndex `RedBlackGaussSeidel 1.81`,
FrictionIndex `Jacobi 1.77`, Boxed `ShockPropagation 1.66`, Standard
`Lemke 1.65`, and FrictionIndex `Admm 1.65`. Do not push or mutate GitHub state
without explicit maintainer/user approval.

## Current Reality - 2026-06-12 ShockPropagation Large FrictionIndex Exact Path

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `58bfb658ed7 Extend Symmetric PSOR exact path to friction rows`.
- Current checkpoint target:
  `Raise ShockPropagation friction exact gate`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 66 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

What this slice changes:

- `ShockPropagationSolver` now names its friction-index exact fast-path gate as
  `kMaxFrictionIndexExactFastPathSize` and raises it to 192 variables, covering
  the current 64-contact FrictionIndex comparison packet.
- The gate still only applies to non-warm-started strict-interior
  friction-index rows accepted by the shared validated exact helper.
- Active-bound, validator-rejected, warm-started, custom-partition invalid, and
  larger friction-index rows continue through the layered block path.
- Unit coverage adds a 64-contact `ShockPropagationUsesLargeLinearSolve`
  regression expecting the exact path to finish in zero iterations.
- The checked profile CSVs, Python demo profile summaries, metadata assertions,
  other-method docs, changelog, and hand-off docs were refreshed.

Evidence:

- Focused after-run:
  `build/friction_index_shock_gate192_after.json`.
- Focused FrictionIndex `ShockPropagation/64` accepted the exact path with
  `iterations=0`, exact residual, and `contract_ok=1.0`.
- Regenerated `build/lcp_profile_full.json` reports FrictionIndex
  `ShockPropagation 1.97`, below the previous above-`2x` average.
- The latest full-profile FrictionIndex above-`2x` rows are now
  `BlockedJacobi 2.30` and `BoxedSemiSmoothNewton 2.05`.
- CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
  columns.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(ShockPropagation|Pgs|Tgs|BGS|Sap|Dantzig)/' \
  --benchmark_min_time=0.1s \
  --benchmark_format=json > build/friction_index_shock_gate192_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

If this checkpoint is committed, continue with the refreshed profile's highest
remaining FrictionIndex target: `BlockedJacobi 2.30`, then inspect
`BoxedSemiSmoothNewton 2.05`, `ShockPropagation 1.97`, `BGS 1.89`, and
`NNCG 1.86`. Do not push or mutate GitHub state without explicit
maintainer/user approval.

## Current Reality - 2026-06-12 Symmetric PSOR FrictionIndex Exact Path

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `c514269d076 Extend APGD exact paths to boxed and friction rows`.
- Current checkpoint target:
  `Extend Symmetric PSOR exact path to friction rows`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 65 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

What this slice changes:

- `dart/math/lcp/projection/symmetric_psor_solver.cpp`
  extends the existing non-warm-started Symmetric PSOR exact fast path from
  strict-interior standard rows to medium friction-index rows via
  `detail::trySolveInteriorFrictionIndexLcp(...)`.
- The Symmetric PSOR fast path remains gated by
  `options.customOptions == nullptr`, `!options.warmStart`, and
  `n <= kMaxStrictInteriorFastPathSize`.
- No boxed Symmetric PSOR exact branch is present. A boxed probe was slower and
  should not be reintroduced without new evidence.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` adds
  `SymmetricPsorSolver` to the friction-index high-overhead exact-path smoke
  coverage and adds `SymmetricPsorUsesMediumLinearSolve` for a 16-contact
  friction-index row with `maxIterations = 1`.
- The checked profile CSVs, Python demo profile summaries, metadata assertions,
  projection-method docs, changelog, and hand-off docs were refreshed.

Evidence:

- Accepted focused probe:
  `build/symmetric_psor_findex_exact_probe.json`.
- Focused FrictionIndex `SymmetricPsor` timings moved approximately:
  - contacts 4: `1873.15ns`, `iterations=3` -> `1158.30ns`,
    `iterations=0`.
  - contacts 16: `16733.51ns`, `iterations=4` -> `13919.53ns`,
    `iterations=0`.
  - contacts 64: `288845.33ns`, `iterations=4` -> `267597.98ns`,
    `iterations=4`; this row stays iterative and contract-valid.
- Focused build and CTest passed for `BM_LCP_COMPARE`,
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`, and the focused
  validation CTest.
- A full profile regeneration completed. Its latest average-ratio snapshot was:
  - Standard: largest averages were
    `FischerBurmeisterNewton 1.62`, `MinimumMapNewton 1.59`,
    `PenalizedFischerBurmeisterNewton 1.58`, `Apgd 1.57`,
    `InteriorPoint 1.57`, and `SubspaceMinimization 1.55`.
  - Boxed: largest average was `ShockPropagation 1.93`; no Boxed solver
    average was above `2x`.
  - FrictionIndex: `ShockPropagation 2.09`,
    `BoxedSemiSmoothNewton 1.94`, `SubspaceMinimization 1.94`,
    `NNCG 1.88`, `Admm 1.80`, `Staggering 1.80`,
    `Jacobi 1.76`, `BlockedJacobi 1.74`, `SymmetricPsor 1.71`,
    `Dantzig 1.65`, and `Apgd 1.64`.
- CSV shape check showed 200 rows in each checked profile CSV, with
  15 Boxed solver columns, 16 FrictionIndex solver columns, and 23 Standard
  solver columns.

Rejected probes before the stop instruction:

- `ShockPropagationSolver` friction-index exact gate `48 -> 64` wrote
  `build/friction_index_shock_gate64_probe.json` and did not take the exact
  path for `ShockPropagation/64`; it was reverted.
- `SubspaceMinimizationSolver` friction exact gate `<= 48` wrote
  `build/friction_index_subspace_gate48_probe.json` and regressed
  `SubspaceMinimization/64` to roughly `1.34ms`; it was reverted.
- Symmetric PSOR boxed exact path was slower in
  `build/symmetric_psor_exact_probe.json`; only the friction-index exact path
  was kept.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
```

Still required before commit:

```bash
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git diff --stat
git diff -- dart/math/lcp/projection/symmetric_psor_solver.cpp \
  tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp
```

Then run any missing mandatory checks and commit with a message like:

```text
Extend Symmetric PSOR exact path to friction rows
```

Do not push or mutate GitHub state without explicit maintainer/user approval.

## Current Reality - 2026-06-12 APGD Boxed/Friction Exact Path

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `19a2250232c Delay ShockPropagation reset after exact path`.
- Current checkpoint target:
  `Extend APGD exact paths to boxed and friction rows`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 64 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push or mutate
  GitHub state without explicit maintainer/user approval.

What this slice changes:

- `ApgdSolver` uses the shared validated exact helpers for non-warm-started
  boxed active-set rows and medium friction-index rows when no custom APGD
  options are supplied.
- The existing 48-variable size gate keeps larger or rejected rows on iterative
  APGD.
- APGD restart-policy sweep rows keep their custom parameters and therefore
  still exercise iterative restart behavior.
- Unit coverage, profile CSVs, Python demo profile summaries, metadata
  assertions, projection-method docs, changelog, and hand-off docs were
  refreshed.

Evidence:

- Focused after-run:
  `build/friction_index_apgd_exact_after.json`.
- Focused FrictionIndex APGD timings moved approximately:
  - contacts 4: `1966.2ns -> 1188.1ns`.
  - contacts 16: `19687.3ns -> 14199.9ns`.
  - contacts 64: `350014.2ns -> 296428.5ns`.
- Regenerated `build/lcp_profile_full.json` reports FrictionIndex `Apgd 1.49`;
  no refreshed profile surface has a solver average above `2x`.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(Apgd|Pgs|Tgs|BGS|Sap)/' \
  --benchmark_min_time=0.1s \
  --benchmark_format=json > build/friction_index_apgd_exact_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

Then continue with the refreshed profile: FrictionIndex `ShockPropagation 1.87`
is now the highest average, followed by FrictionIndex
`SubspaceMinimization`, `BoxedSemiSmoothNewton`, `SymmetricPsor`,
`BlockedJacobi`, and `NNCG`. Do not push or mutate GitHub state without
explicit maintainer/user approval.

## Current Reality - 2026-06-12 ShockPropagation Delayed Reset

This is the latest state. Older sections below are historical checkpoints and
may retain their original "latest" wording from the time they were written.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `0c6e8bb7902 Tune boxed SSN friction exact gate`.
- Current checkpoint target:
  `Delay ShockPropagation reset after exact path`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 63 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Do not push or mutate
  GitHub state without explicit maintainer/user approval.

What this slice changes:

- `ShockPropagationSolver::solve()` delays non-warm-start initial-guess zeroing
  until after exact-fast-path attempts and block validation.
- Accepted exact candidates avoid the zero-vector write; layered fallback paths
  still reset the initial guess before fallback metrics and layer order.
- The performance-profile CSVs, Python demo profile summary, metadata
  assertions, background note, changelog, and hand-off docs were refreshed.

Evidence:

- Focused after-run:
  `build/boxed_shock_delay_reset_after.json`.
- Focused ShockPropagation timings were approximately:
  - contacts 12: `1376.3ns`, neutral versus the prior `1374.0ns`.
  - contacts 24: `4303.6ns`, improved versus the prior `7560.7ns`.
  - contacts 48: `15914.9ns`, improved versus the prior `16664.5ns`.
- Cached full-profile summary from `build/lcp_profile_full.json` now reports
  Boxed `ShockPropagation 1.75`, so there is no refreshed Boxed average above
  `2x`.
- FrictionIndex `Apgd 2.08` is now the only refreshed average above `2x`.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output /tmp/lcp_profile_check
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

Then continue with the refreshed profile: FrictionIndex `Apgd` is the only
above-`2x` average, with FrictionIndex `ShockPropagation`, `SymmetricPsor`,
`SubspaceMinimization`, `BoxedSemiSmoothNewton`, `BlockedJacobi`, and `NNCG`
as near-boundary follow-up targets. Do not push or mutate GitHub state without
explicit maintainer/user approval.

## Current Reality - 2026-06-12 Configurable Boxed SSN Friction Exact Gate

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint target:
  `Tune boxed SSN friction exact gate`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 62 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `BoxedSemiSmoothNewtonSolver::Parameters` now exposes
  `maxFrictionIndexExactSolveDimension`, defaulting to the previous
  conservative 48-variable strict-interior friction-index exact-solve gate.
- The canonical `BM_LcpCompare/FrictionIndex/BoxedSemiSmoothNewton` rows raise
  that gate to 192 variables so the 64-contact strict-interior comparison packet
  can use the already-validated exact solve.
- Other shared benchmark paths keep the default gate unless they explicitly opt
  in; this avoids adding a failed dense 192x192 solve to active/contact-derived
  friction-index rows.
- The new parameter is surfaced in dartpy, Python LCP demo metadata, benchmark
  counters, Newton-method background docs, changelog, and focused C++/Python
  tests.

Evidence:

- Focused baseline:
  `build/friction_index_bssn64_probe_baseline.json`.
- Focused after-run:
  `build/friction_index_bssn64_exact_gate_param_after.json`.
- `BoxedSemiSmoothNewton/64` improved from about `653075.1ns`,
  `iterations=1`, exact residual, no gate counter, to about `447664.5ns`,
  `iterations=0`, exact residual, `boxed_ssn_friction_index_exact_solve_dimension=192`.
- Scope check `build/friction_index_bssn_exact_gate_scope_check.json` showed:
  - `BM_LcpCompare/FrictionIndex/BoxedSemiSmoothNewton/64`:
    `exact_gate=192`, `iterations=0`, `contract_ok=1.0`.
  - `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep/FrictionIndex/DefaultSearch`:
    `exact_gate=48`, `iterations=0`, `contract_ok=1.0`.
- Regenerated full profile reports FrictionIndex averages:
  `ShockPropagation 1.98`, `Apgd 1.91`, `NNCG 1.89`,
  `SubspaceMinimization 1.85`, `BlockedJacobi 1.79`, `Dantzig 1.76`,
  `BoxedSemiSmoothNewton 1.75`, `Jacobi 1.71`, `Staggering 1.71`,
  `RedBlackGaussSeidel 1.70`, `SymmetricPsor 1.68`, `Admm 1.64`,
  `BGS 1.61`, `Sap 1.46`, `Tgs 1.07`, and `Pgs 1.00`.
- Regenerated full profile reports Boxed `ShockPropagation 2.10`; that row is
  now the clearest above-`2x` target.
- Regenerated full profile reports Standard `Dantzig 1.66`, `Lemke 1.61`, and
  `Baraff 1.55` as the largest averages.

Verification completed:

- `BM_LCP_COMPARE`, `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`, and
  `dartpy` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused dartpy/demo metadata tests passed:
  `5 passed`.
- Focused benchmark after-run and scope-check JSONs were written under
  `build/`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Final Python demo metadata test passed after summary assertion refresh.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

Continue from the refreshed profile. Do not push or mutate GitHub state without
explicit maintainer/user approval.

Current next targets after this slice:

- Boxed: `ShockPropagation` is above `2x` and is the main current target.
- FrictionIndex: no solver average is above `2x`; `ShockPropagation`, `Apgd`,
  `NNCG`, `SubspaceMinimization`, and `BlockedJacobi` are near-boundary or
  high-max rows.
- Standard: `Dantzig`, `Lemke`, and `Baraff` are the largest averages.

## Current Reality - 2026-06-12 Stop-Work Hand-Off

This section is the latest state; older sections below are historical
checkpoints. The user explicitly requested stopping implementation work and
preserving hand-off context without further verification.

Current branch:

- `feature/lcp-solver-interface-demos`
- Last committed checkpoint:
  `19686f271a2 Extend boxed SSN friction exact path to medium rows`.
- Before this hand-off-doc edit, the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 60 commits with a clean
  worktree.
- There is no associated PR yet.

What happened after the last committed checkpoint:

- No solver code changes were accepted after commit `19686f271a2`.
- A read/probe pass inspected the next likely target: large FrictionIndex
  `BoxedSemiSmoothNewton`.
- A focused benchmark probe was written to
  `build/friction_index_bssn64_probe_baseline.json` for:
  `BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/64`.
- The probe showed `BoxedSemiSmoothNewton/64` is slower because it performs one
  dense Newton step to tight solver tolerances, while `Pgs/64` and `Tgs/64`
  satisfy the benchmark contract with looser residuals:
  - `Pgs/64`: about `184735.9ns`, `iterations=8`,
    `complementarity=0.0001289616`, `contract_ok=1.0`.
  - `Tgs/64`: about `175096.2ns`, `iterations=8`,
    `complementarity=0.0001289616`, `contract_ok=1.0`.
  - `BoxedSemiSmoothNewton/64`: about `653075.1ns`, `iterations=1`,
    `complementarity=1.7e-13`, `contract_ok=1.0`,
    `boxed_ssn_pgs_warm_start_iterations=5`.

Important resume guidance:

- Do not weaken `BoxedSemiSmoothNewtonSolver` semantics by accepting a PGS/TGS
  warm start at benchmark-contract tolerance inside the solver unless the
  solver options are explicitly relaxed and that choice is documented.
- A safe solver-level optimization would need to accept a warm start only when
  it satisfies the solver's configured residual/complementarity tolerances.
- If large FrictionIndex `BoxedSemiSmoothNewton` does not yield a clean win,
  inspect near-boundary FrictionIndex `Dantzig`, `ShockPropagation`, or
  `BlockedJacobi` instead.
- No additional verification was run for this stop-work hand-off, by user
  request.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

Then first reconcile the branch state with the top section of this file and
`docs/dev_tasks/lcp_solver_interface_demos/README.md`. Do not push or mutate
GitHub state without explicit maintainer/user approval.

## Current Reality - 2026-06-12 Medium Boxed SSN Friction Exact Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint target:
  `Extend boxed SSN friction exact path to medium rows`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 60 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `BoxedSemiSmoothNewtonSolver` now lets the shared validated
  strict-interior friction-index exact shortcut handle non-warm-started packets
  up to 48 variables.
- The 16-contact FrictionIndex comparison packet takes the exact path; larger
  64-contact packets stay on the semi-smooth Newton path because the dense
  shortcut was not a focused benchmark win there.
- Unit coverage now checks that the 48-variable medium friction-index packet
  solves through the zero-iteration exact path.
- The checked performance profile CSVs, Python demo profile summary,
  Newton-methods background note, changelog, Python metadata assertions, and
  this hand-off were refreshed.

Evidence:

- Focused A/B
  `BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/` compared
  `build/friction_index_bssn_gate12_baseline.json` with
  `build/friction_index_bssn_gate48_after.json`.
- Focused `BoxedSemiSmoothNewton` timings were:
  - contacts 4: `1309.9ns -> 1268.7ns`
  - contacts 16: `26648.0ns -> 15050.6ns`
  - contacts 64: `696561.3ns -> 682964.4ns`
- Regenerated full profile reports FrictionIndex averages:
  `BoxedSemiSmoothNewton 2.55`, `Dantzig 1.98`, `ShockPropagation 1.95`,
  `BlockedJacobi 1.91`, `Apgd 1.89`, `NNCG 1.89`,
  `SymmetricPsor 1.83`, `SubspaceMinimization 1.83`, `Jacobi 1.78`,
  `RedBlackGaussSeidel 1.71`, `Staggering 1.71`, `BGS 1.67`,
  `Admm 1.62`, `Sap 1.47`, `Pgs 1.10`, and `Tgs 1.00`.
- The same regenerated profile reports all Boxed solvers below `2x`; largest
  Boxed averages are `Sap 1.63`, `Apgd 1.62`, and `ShockPropagation 1.60`.
- Standard's largest current averages are `Lemke 2.00`, `Baraff 1.99`,
  `InteriorPoint 1.87`, and `FischerBurmeisterNewton 1.78`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused A/B benchmark JSONs written to
  `build/friction_index_bssn_gate12_baseline.json` and
  `build/friction_index_bssn_gate48_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- FrictionIndex: large `BoxedSemiSmoothNewton` remains the main above-`2x`
  target; `Dantzig`, `ShockPropagation`, and `BlockedJacobi` are near-boundary.
- Boxed: no solver is above `2x`.
- Standard: `Lemke` and `Baraff` are near the `2x` boundary.

## Current Reality - 2026-06-12 ShockPropagation Exact Prevalidation

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint target:
  `Prevalidate ShockPropagation exact shortcuts`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 59 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `ShockPropagationSolver` now attempts its validated exact shortcut before full
  problem validation and layered block setup for non-empty standard, boxed, and
  friction-index problem forms.
- The shortcut still runs lightweight custom block/layer structure validation
  first, keeps zero-row problem validation before zero-row success, and still
  builds non-empty custom friction-index blocks before any exact acceptance so
  invalid custom friction partitions fail as before.
- The checked performance profile CSVs, Python demo profile summary,
  other-methods background note, changelog, and this hand-off were refreshed.

Evidence:

- Focused `BM_LcpCompare/Boxed/(ShockPropagation|Dantzig|Pgs|Tgs|Jacobi)/`
  wrote `build/boxed_shock_prevalidate_fast_path_after.json`.
- Focused `ShockPropagation` boxed results were:
  - rows 12: `1407.0ns`
  - rows 24: `4137.6ns`
  - rows 48: `16248.8ns`
- Regenerated full profile reports Boxed averages:
  `ShockPropagation 1.84`, `Apgd 1.69`, `Admm 1.64`, `BGS 1.64`,
  `SubspaceMinimization 1.64`, `NNCG 1.58`, `BlockedJacobi 1.57`,
  `BoxedSemiSmoothNewton 1.57`, `Sap 1.54`, `RedBlackGaussSeidel 1.49`,
  `Dantzig 1.44`, `SymmetricPsor 1.36`, `Jacobi 1.10`, `Pgs 1.08`, and
  `Tgs 1.00`.
- The same regenerated profile reports FrictionIndex averages:
  `BoxedSemiSmoothNewton 2.37`, `NNCG 2.02`, `BlockedJacobi 1.79`,
  `ShockPropagation 1.75`, `Apgd 1.70`, `SubspaceMinimization 1.61`,
  `Dantzig 1.59`, `Jacobi 1.56`, `Staggering 1.53`,
  `RedBlackGaussSeidel 1.51`, `SymmetricPsor 1.47`, `BGS 1.45`,
  `Admm 1.44`, `Sap 1.40`, `Pgs 1.28`, and `Tgs 1.01`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/boxed_shock_prevalidate_fast_path_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Boxed: no solver is above `2x`; `ShockPropagation` remains the largest
  moderate row.
- FrictionIndex: `BoxedSemiSmoothNewton` and `NNCG` are above `2x`.
- Standard: moderate spread only; no standard row is above `2x`.

## Current Reality - 2026-06-12 ShockPropagation Empty-Custom Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint target:
  `Skip empty ShockPropagation block prebuild`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 58 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `ShockPropagationSolver` no longer prebuilds friction-index block data before
  trying the validated exact shortcut when custom options are present but both
  `blockSizes` and `layers` are empty.
- Non-empty custom block/layer partitions still build and validate block data
  before the exact shortcut, preserving the existing invalid-partition failure
  behavior.
- The broader BGS-delegation experiment was rejected because it improved small
  rows but regressed the 64-contact FrictionIndex row.
- The checked performance profile CSVs, Python demo profile summary,
  other-methods background note, changelog, and this hand-off were refreshed.

Evidence:

- Focused
  `BM_LcpCompare/FrictionIndex/(ShockPropagation|BGS|Pgs|Tgs)/`
  wrote `build/friction_index_shock_skip_empty_prebuild_after.json`.
- Compared with the previous `build/lcp_profile_full.json` cache, focused
  `ShockPropagation` results were:
  - contacts 4: `2214.6ns -> 1743.3ns`
  - contacts 16: `22983.8ns -> 18426.3ns`
  - contacts 64: `447392.9ns -> 449624.5ns`
- Regenerated full profile reports FrictionIndex averages:
  `BoxedSemiSmoothNewton 2.75`, `Apgd 2.04`, `ShockPropagation 2.00`,
  `NNCG 1.93`, `SubspaceMinimization 1.92`, `BlockedJacobi 1.83`,
  `Admm 1.76`, `Jacobi 1.74`, `Staggering 1.74`, `BGS 1.73`,
  `Dantzig 1.73`, `RedBlackGaussSeidel 1.69`, `SymmetricPsor 1.66`,
  `Sap 1.54`, `Tgs 1.12`, and `Pgs 1.00`.
- The same regenerated profile reports Boxed `ShockPropagation 2.65`; that row
  is the next obvious boxed target.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/friction_index_shock_skip_empty_prebuild_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log --oneline --decorate -5
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Boxed: `ShockPropagation` is back above `2x` in the refreshed profile.
- FrictionIndex: `BoxedSemiSmoothNewton` and `Apgd` are above `2x`, with
  `ShockPropagation` exactly at `2x`.
- Standard: moderate spread only; no standard row is above `2x`.

## Current Reality - 2026-06-12 Boxed SSN Line-Search Early Acceptance

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint target:
  `Accept converged boxed SSN line-search steps`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 57 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `BoxedSemiSmoothNewtonSolver` now accepts a line-search step that already
  reaches the natural residual tolerance instead of spending one extra Newton
  loop only to observe convergence at the next iteration header.
- Unit coverage now exercises the Newton path directly with a warm-started
  standard row and `maxIterations=1`.
- The Python LCP demo profile summary, checked performance profile CSVs,
  Newton-method background docs, changelog, and this hand-off were refreshed.

Evidence:

- Focused
  `BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/`
  wrote `build/friction_index_bssn_line_search_after.json`.
- Compared with `build/friction_index_bssn_shock_baseline.json`, focused
  `BoxedSemiSmoothNewton` iterations dropped from `2` to `1` on 16- and
  64-contact FrictionIndex packets.
- Focused `BoxedSemiSmoothNewton` absolute times improved from `33270.0ns` to
  `26775.6ns` on 16 contacts and from `829245.2ns` to `754288.4ns` on 64
  contacts. The 4-contact row remains on the zero-iteration exact path.
- Regenerated full profile reports FrictionIndex averages:
  `BoxedSemiSmoothNewton 2.44`, `ShockPropagation 2.24`,
  `BlockedJacobi 2.02`, `Apgd 1.89`, `RedBlackGaussSeidel 1.89`,
  `SymmetricPsor 1.79`, `SubspaceMinimization 1.75`, `BGS 1.74`,
  `NNCG 1.73`, `Jacobi 1.72`, `Staggering 1.71`, `Admm 1.63`,
  `Dantzig 1.62`, `Sap 1.37`, `Tgs 1.10`, and `Pgs 1.00`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/friction_index_bssn_line_search_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- FrictionIndex: `ShockPropagation`, `BlockedJacobi`, and larger
  `BoxedSemiSmoothNewton` rows.
- Boxed: no solver above `2x` in the refreshed profile.
- Standard: moderate spread only; this slice does not claim Standard
  completion.

## Current Reality - 2026-06-12 Strict-Interior Friction-Index Fast Paths

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path strict-interior friction-index LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 56 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- Added a shared validated strict-interior friction-index exact solve in
  `dart/math/lcp/lcp_validation.hpp`.
- Dantzig, Blocked Jacobi, BGS, NNCG, Subspace Minimization,
  ShockPropagation, Staggering, small-packet ADMM, and small-packet
  BoxedSemiSmoothNewton now try that helper for non-warm-started
  strict-interior friction-index rows.
- Blocked Jacobi, BGS, and ShockPropagation local contact blocks also try the
  helper before Dantzig fallback.
- ShockPropagation still validates custom block/layer partitions before any
  top-level friction-index shortcut.
- Active-bound friction rows, warm starts, large size-sensitive rows, and
  validator-rejected rows remain on each solver's original iterative, block,
  or Newton path.
- SAP stays on its regularized Newton path for friction-index rows.
- The Python LCP demo profile summary now moves BGS, Staggering,
  SubspaceMinimization, Dantzig, NNCG, and ADMM into the close
  FrictionIndex cluster and identifies BoxedSemiSmoothNewton and
  ShockPropagation as the remaining above-`2x` rows.

Evidence:

- Focused
  `BM_LcpCompare/FrictionIndex/(Dantzig|BlockedJacobi|BGS|NNCG|SubspaceMinimization|ShockPropagation|Staggering|Admm|BoxedSemiSmoothNewton|Sap)/`
  wrote `build/friction_index_interior_fast_path_after.json`.
- Focused run reported `contract_ok=1.0` for all rows.
- Focused speedups over the previous full profile cache:
  - `BlockedJacobi`: `17.71x`, `11.39x`, `2.80x`.
  - `BGS`: `9.28x`, `3.39x`, `1.45x`.
  - `ShockPropagation`: `6.49x`, `3.21x`, `1.47x`.
  - `Staggering`: `6.13x`, `3.14x`, `3.14x`.
  - `SubspaceMinimization`: `3.27x`, `2.77x`, `2.70x`.
  - `NNCG`: `3.68x`, `2.53x`, `1.66x`.
  - `Dantzig`: `1.90x`, `1.50x`, `2.04x`.
- Regenerated full profile reports FrictionIndex averages:
  `BoxedSemiSmoothNewton 2.84`, `ShockPropagation 2.35`,
  `BlockedJacobi 2.00`, `Sap 1.94`, `Apgd 1.88`, `NNCG 1.83`,
  `Dantzig 1.83`, `Staggering 1.83`, `SubspaceMinimization 1.77`,
  `BGS 1.69`, and `Admm 1.63`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/friction_index_interior_fast_path_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- FrictionIndex: `BoxedSemiSmoothNewton`, `ShockPropagation`, then
  `BlockedJacobi` large-contact rows if the `2x` boundary remains important.
- Boxed: no solver above `2x` in the refreshed profile.
- Standard: moderate spread only.

## Current Reality - 2026-06-12 Remaining Boxed Active-Set Fast Paths

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path remaining boxed active-set LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 55 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `BoxedSemiSmoothNewtonSolver`, `SapSolver`, and
  `SubspaceMinimizationSolver` now try the shared projected-active-set boxed-LCP
  exact solve for non-warm-started boxed rows without friction-index coupling.
- `SapSolver` also now tries the shared strict-interior standard-LCP exact
  solve for non-warm-started high-level solves.
- Warm-started solves, coupled friction-index rows, and validator-rejected rows
  stay on each solver's existing semi-smooth Newton, SAP regularized Newton, or
  PGS-subspace refinement path.
- Unit coverage now checks lower/upper/free boxed BoxedSemiSmoothNewton, SAP,
  and SubspaceMinimization packets that solve in zero iterations, plus SAP's
  standard strict-interior fast path.
- The Python LCP demo profile summary now moves BoxedSemiSmoothNewton, SAP, and
  SubspaceMinimization into the close boxed exact-path cluster and records that
  no boxed row is above `2x` in the refreshed profile.

Evidence:

- Focused
  `BM_LcpCompare/Boxed/(BoxedSemiSmoothNewton|Sap|SubspaceMinimization)/`
  wrote `build/remaining_boxed_projected_active_set_after.json`.
- Focused SubspaceMinimization rows reported `1316.947ns`, `4338.628ns`, and
  `17578.805ns` for 12, 24, and 48 row packets.
- Focused SAP rows reported `1359.414ns`, `4524.363ns`, and `16282.916ns` for
  12, 24, and 48 row packets.
- Focused BoxedSemiSmoothNewton rows reported `1268.846ns`, `4536.356ns`, and
  `15964.235ns` for 12, 24, and 48 row packets.
- All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Regenerated full profile reports Boxed `BoxedSemiSmoothNewton` average ratio
  `1.54`, Boxed `Sap` average ratio `1.56`, and Boxed
  `SubspaceMinimization` average ratio `1.52`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/remaining_boxed_projected_active_set_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Boxed: no solver above `2x` in the refreshed profile.
- FrictionIndex: `BlockedJacobi`, `ShockPropagation`, `BGS`, `Staggering`,
  `SubspaceMinimization`, `NNCG`, `Dantzig`, `BoxedSemiSmoothNewton`, and
  `Admm`.
- Standard: moderate spread only; `FischerBurmeisterNewton`,
  `BoxedSemiSmoothNewton`, `Baraff`, `MinimumMapNewton`, `InteriorPoint`,
  `Apgd`, `Sap`, and `PenalizedFischerBurmeisterNewton` are the largest
  current rows.

## Current Reality - 2026-06-12 Boxed Block Projection Active-Set Fast Paths

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path boxed active-set block projection LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 54 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `BlockedJacobiSolver` and `BgsSolver` now try the shared
  projected-active-set boxed-LCP exact solve for non-warm-started boxed rows
  without friction-index coupling through their high-level `LcpProblem`
  interfaces.
- Explicit custom block options still build and validate block structure before
  any exact shortcut is accepted.
- Warm-started solves and friction-index rows stay on the existing block
  iteration paths.
- Unit coverage now checks lower/upper/free boxed BlockedJacobi and BGS packets
  that solve in zero iterations.
- The Python LCP demo profile summary now removes `BlockedJacobi` and `BGS`
  from Boxed laggards and lists them with the close boxed exact-path rows.

Evidence:

- Focused `BM_LcpCompare/Boxed/(BlockedJacobi|BGS)/` wrote
  `build/block_projection_boxed_projected_active_set_after.json`.
- Focused BlockedJacobi rows reported `1764.970ns`, `4257.723ns`, and
  `18685.986ns` for 12, 24, and 48 row packets.
- Focused BGS rows reported `1621.035ns`, `6811.200ns`, and `17464.305ns` for
  12, 24, and 48 row packets.
- All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Regenerated full profile reports Boxed `BlockedJacobi` average ratio `1.58`
  and Boxed `BGS` average ratio `1.66`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/block_projection_boxed_projected_active_set_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Boxed: `BoxedSemiSmoothNewton`, `Sap`, and `SubspaceMinimization`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`,
  `SubspaceMinimization`, `NNCG`, `BoxedSemiSmoothNewton`, `Dantzig`, and
  `Admm`.
- Standard: moderate `Dantzig`, `Baraff`, `Apgd`, `BGS`, `Lemke`,
  `MinimumMapNewton`, `BlockedJacobi`, and `NNCG`.

## Current Reality - 2026-06-12 Boxed NNCG Projected Active-Set Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path boxed active-set NNCG LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 53 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `NncgSolver` now tries the shared projected-active-set boxed-LCP exact solve
  for non-warm-started boxed rows without friction-index coupling through its
  high-level `LcpProblem` interface.
- Warm-started solves and friction-index rows stay on the existing
  PGS-preconditioned NNCG iteration path.
- Unit coverage now checks a lower/upper/free boxed NNCG packet that solves in
  zero iterations.
- The Python LCP demo profile summary now removes `Nncg` from Boxed laggards
  and lists it with the close boxed exact-path rows.

Evidence:

- Focused `BM_LcpCompare/Boxed/NNCG/` wrote
  `build/nncg_boxed_projected_active_set_after.json` with `1184.486ns`,
  `3822.184ns`, and `13505.529ns` for 12, 24, and 48 row packets.
- All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Regenerated full profile reports Boxed `NNCG` average ratio `1.68`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/nncg_boxed_projected_active_set_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Boxed: `BlockedJacobi`, `BGS`, `BoxedSemiSmoothNewton`, `Sap`, and
  `SubspaceMinimization`.
- FrictionIndex: `BlockedJacobi`, `ShockPropagation`, `Staggering`, `BGS`,
  `SubspaceMinimization`, `NNCG`, `BoxedSemiSmoothNewton`, `Dantzig`, and
  `Admm`.
- Standard: moderate `Sap`, `Baraff`, `Apgd`,
  `PenalizedFischerBurmeisterNewton`, `FischerBurmeisterNewton`,
  `MinimumMapNewton`, `NNCG`, and `BoxedSemiSmoothNewton`.

## Current Reality - 2026-06-12 Boxed Dantzig Projected Active-Set Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path boxed active-set Dantzig LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 52 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `DantzigSolver` now tries the shared projected-active-set boxed-LCP exact
  solve for non-warm-started boxed rows without friction-index coupling through
  its high-level `LcpProblem` interface.
- Warm-started solves and low-level matrix/scratch calls stay on the
  ODE-derived pivoting path, and friction-index rows stay on the existing
  pivot/refinement route.
- Unit coverage now checks a lower/upper/free boxed Dantzig packet that solves
  in zero iterations.
- The Python LCP demo profile summary now removes `Dantzig` from Boxed laggards
  and lists it with the close boxed exact-path rows.

Evidence:

- Focused `BM_LcpCompare/Boxed/Dantzig/` wrote
  `build/dantzig_boxed_projected_active_set_after.json` with `1281.450ns`,
  `3993.311ns`, and `14533.545ns` for 12, 24, and 48 row packets.
- All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Regenerated full profile reports Boxed `Dantzig` average ratio `1.46`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/dantzig_boxed_projected_active_set_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Boxed: `NNCG`, `BlockedJacobi`, `BGS`, `BoxedSemiSmoothNewton`, `Sap`, and
  `SubspaceMinimization`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`,
  `SubspaceMinimization`, `NNCG`, `Dantzig`, `BoxedSemiSmoothNewton`, and
  `Admm`.
- Standard: moderate `InteriorPoint`, `Dantzig`, `FischerBurmeisterNewton`,
  `PenalizedFischerBurmeisterNewton`, `Lemke`, `BoxedSemiSmoothNewton`, and
  `Sap`.

## Current Reality - 2026-06-12 Boxed ShockPropagation Projected Active-Set Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path boxed active-set ShockPropagation LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 51 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `ShockPropagationSolver` now tries the shared projected-active-set boxed-LCP
  exact solve for non-warm-started boxed rows without friction-index coupling.
- The boxed shortcut runs after lightweight custom block/layer structure
  validation and before block/layer data construction, so invalid custom
  structure is still rejected before any fast-path bypass.
- Warm-started, friction-index, and validator-rejected boxed rows stay on the
  existing block/layer sweep path.
- Unit coverage now checks both the zero-iteration boxed fast path and invalid
  boxed custom block-size rejection.
- The Python LCP demo profile summary now removes `ShockPropagation` from Boxed
  laggards and lists it as a close boxed exact-path row.

Evidence:

- Focused `BM_LcpCompare/Boxed/ShockPropagation/` wrote
  `build/shockprop_boxed_projected_active_set_after.json` with `1316.398ns`,
  `4229.786ns`, and `13883.343ns` for 12, 24, and 48 row packets.
- All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Regenerated full profile reports Boxed `ShockPropagation` average ratio
  `1.81`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/shockprop_boxed_projected_active_set_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Boxed: `Dantzig`, `NNCG`, `BlockedJacobi`, `BGS`,
  `BoxedSemiSmoothNewton`, `Sap`, and `SubspaceMinimization`.
- FrictionIndex: `BlockedJacobi`, `Staggering`, `ShockPropagation`, `BGS`,
  `SubspaceMinimization`, `NNCG`, `Dantzig`, `BoxedSemiSmoothNewton`, and
  `Admm`.
- Standard: moderate `RedBlackGaussSeidel`, `Baraff`, `MinimumMapNewton`,
  `Lemke`, `Apgd`, `BlockedJacobi`, `NNCG`, and Newton-family rows.

## Current Reality - 2026-06-12 Boxed ADMM Projected Active-Set Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path boxed active-set ADMM LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 50 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- Added a shared validated projected-active-set boxed-LCP exact-solve helper.
  It proposes active bounds from the unconstrained solve, solves the free block
  exactly, and accepts only candidates that pass the existing boxed solution
  validator.
- Default, non-warm-started `AdmmSolver` boxed LCPs without friction-index
  coupling now try that exact path before allocating ADMM iteration workspace.
- Warm-started, explicit custom-option, standard strict-interior, and coupled
  friction-index ADMM rows stay on their previous paths.
- Unit coverage now checks an active lower/upper/free boxed ADMM packet that
  solves in zero iterations.
- The Python LCP demo profile summary now moves `Admm` into Boxed leaders and
  removes it from Boxed laggards.

Evidence:

- Focused `BM_LcpCompare/Boxed/Admm/` wrote
  `build/admm_boxed_projected_active_set_after.json` with `1217.118ns`,
  `3806.814ns`, and `15540.370ns` for 12, 24, and 48 row packets.
- All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Regenerated full profile reports Boxed `Admm` average ratio `1.56` and
  Standard `Admm` average ratio `1.19`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/admm_boxed_projected_active_set_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check passed for 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Boxed: `ShockPropagation`, `NNCG`, `Dantzig`, `BlockedJacobi`,
  `BoxedSemiSmoothNewton`, `BGS`, `SubspaceMinimization`, and `Sap`.
- FrictionIndex: `BlockedJacobi`, `ShockPropagation`, `Staggering`, `BGS`,
  `SubspaceMinimization`, `NNCG`, `Dantzig`, `BoxedSemiSmoothNewton`, `Admm`,
  and `Apgd`.
- Standard: moderate `InteriorPoint`, `Sap`, `FischerBurmeisterNewton`,
  `MinimumMapNewton`, `PenalizedFischerBurmeisterNewton`,
  `RedBlackGaussSeidel`, `ShockPropagation`, and `BoxedSemiSmoothNewton`.

## Current Reality - 2026-06-12 Strict-Interior ShockPropagation Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path strict-interior ShockPropagation LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 49 commits.
- There is no associated PR yet.
- No push has been performed for this continuation. Pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `ShockPropagationSolver` now tries its strict-interior standard-LCP fast path
  before block/layer matrix construction, after lightweight validation of any
  custom block sizes or layer lists.
- The shortcut prefers an `Eigen::LLT` exact solve for SPD rows and falls back
  to the shared strict-interior helper if that candidate does not validate.
- Invalid custom block sizes and layers still fail with `InvalidProblem` before
  an exact solve is accepted.
- Warm-started, active-bound, non-standard, boxed, and friction-index rows stay
  on the existing block/iterative paths.
- Unit coverage now expects `SolvesWithCustomLayers` to use the zero-iteration
  exact solve on the strict-interior standard packet.
- The Python LCP demo profile summary now moves `ShockPropagation` into
  Standard leaders while keeping `Apgd` and `MPRGP` as moderate Standard
  laggards.

Evidence:

- Focused `BM_LcpCompare/Standard/ShockPropagation/` wrote
  `build/shockprop_strict_interior_after.json` and reported zero iterations
  with `contract_ok=1.0` for 12, 24, 48, and 96 row packets.
- Cached full-profile replay reports Standard `ShockPropagation` average ratio
  `1.21`.
- CSV shape check reports 15 Boxed columns, 16 FrictionIndex columns, 23
  Standard columns, and 200 rows per profile.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/shockprop_strict_interior_after.json`.
- Cached full-profile replay wrote `build/lcp_profile_full_check`.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: moderate `Apgd`, `FischerBurmeisterNewton`,
  `MinimumMapNewton`, `InteriorPoint`, `Lemke`, `MPRGP`, and
  `SubspaceMinimization` rows.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, `NNCG`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`,
  `NNCG`, and `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior Projection Iterator Fast Paths

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint:
  `Fast path strict-interior projection iterator LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 48 commits.
- No PR is associated with this branch yet.
- Pushes still require explicit maintainer/user approval.

What this slice changes:

- `ApgdSolver`, `JacobiSolver`, `SymmetricPsorSolver`, and
  `RedBlackGaussSeidelSolver` now try the shared validated strict-interior
  standard-LCP fast path for non-warm-started solves without explicit
  per-solve custom options.
- Each shortcut has a profile-shaped size guard so larger packets stay on the
  existing iterative projection path when the dense linear solve is not
  profitable.
- Unit coverage was extended so
  `StandardStrictInteriorFastPath.ProjectionAndBlockSolversUseLinearSolve`
  includes APGD, Jacobi, Symmetric PSOR, and Red-Black Gauss-Seidel.
- The Python LCP demo profile summary now removes APGD/Jacobi from Standard
  laggards and points remaining Standard tuning at layered/two-color and
  moderate iterative rows.

Evidence:

- Focused
  `BM_LcpCompare/Standard/(Apgd|Jacobi|SymmetricPsor|RedBlackGaussSeidel)/`
  after-run compared to the previous full profile cache:
  - `Apgd`: mean `0.745`; best `0.615`; worst `0.988`.
  - `Jacobi`: mean `0.825`; best `0.490`; worst `1.101`.
  - `RedBlackGaussSeidel`: mean `0.752`; best `0.413`; worst `1.024`.
  - `SymmetricPsor`: mean `0.683`; best `0.476`; worst `0.860`.
  - All focused rows reported `contract_ok=1.0`; fast-pathed rows reported
    `iterations=0`.
- Full regenerated Standard profile now reports `Apgd` `1.46`, `Jacobi`
  `1.44`, `RedBlackGaussSeidel` `1.74`, and `SymmetricPsor` `1.56`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/projection_iterators_strict_interior_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: `ShockPropagation`, `RedBlackGaussSeidel`, and moderate
  `FischerBurmeisterNewton`, `NNCG`, `Lemke`, `SymmetricPsor`,
  `InteriorPoint`, and similar mid-pack rows.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`,
  `Nncg`, and `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior ADMM Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint: `Fast path strict-interior ADMM LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 47 commits.
- No PR is associated with this branch yet.
- Pushes still require explicit maintainer/user approval.

What this slice changes:

- Non-warm-started, default-option `AdmmSolver` calls now try a validated
  strict-interior standard-LCP exact-solve path before allocating ADMM
  iteration workspace.
- The fast path prefers an LLT solve for SPD rows and falls back to the shared
  strict-interior linear-solve helper if the LLT candidate does not validate.
- Boxed, friction-index, warm-started, and explicit custom-option ADMM calls
  stay on the existing operator-splitting loop.
- Unit coverage was extended so
  `StandardStrictInteriorFastPath.OtherSolversUseLinearSolve` includes
  `AdmmSolver` with `warmStart=false`.
- The Python LCP demo profile summary now moves `Admm` from Standard laggards
  into Standard leaders and points remaining Standard tuning at moderate
  iterative rows.

Evidence:

- Focused `BM_LcpCompare/Standard/Admm/` after-run compared to the previous
  full profile cache:
  - 12 rows: `0.216`
  - 24 rows: `0.367`
  - 48 rows: `0.597`
  - 96 rows: `0.744`
  - Mean focused ratio `0.481`; best `0.216`; worst `0.744`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Full regenerated Standard profile now reports `Admm` average ratio `1.07`
  with 2 wins across 4 solved Standard profile rows.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to `build/admm_strict_interior_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: moderate `Apgd`, `Jacobi`, `SymmetricPsor`,
  `RedBlackGaussSeidel`, `ShockPropagation`, and `Sap` rows.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`,
  `Nncg`, and `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior MPRGP Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint: `Fast path strict-interior MPRGP LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 46 commits.
- No PR is associated with this branch yet.
- Pushes still require explicit maintainer/user approval.

What this slice changes:

- Default `MprgpSolver` calls now try the shared validated strict-interior
  standard-LCP fast path after MPRGP's symmetry and positive-definite checks.
- The fast path reuses the existing LLT factorization instead of factoring the
  matrix again, preserving MPRGP's SPD contract while removing the
  reduced-gradient loop on strict-interior rows.
- Custom-option and warm-started MPRGP calls stay on the iterative path so
  parameter stress tests and user tuning remain observable.
- Unit coverage was extended so
  `StandardStrictInteriorFastPath.OtherSolversUseLinearSolve` includes
  `MprgpSolver`.
- The Python LCP demo profile summary now removes `MPRGP` from Standard
  laggards and points remaining Standard tuning at ADMM and moderate iterative
  rows.

Evidence:

- Focused `BM_LcpCompare/Standard/MPRGP/` after-run compared to the previous
  full profile cache:
  - 12 rows: `0.390`
  - 24 rows: `0.481`
  - 48 rows: `0.580`
  - 96 rows: `0.458`
  - Mean focused ratio `0.477`; best `0.390`; worst `0.580`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Full regenerated Standard profile now reports `MPRGP` average ratio `1.70`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to `build/mprgp_strict_interior_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check reported 15 Boxed columns, 16 FrictionIndex columns,
  23 Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: `Admm`, plus moderate `Apgd`, `Jacobi`, `SymmetricPsor`,
  `RedBlackGaussSeidel`, and `ShockPropagation` rows.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `Staggering`, `ShockPropagation`, and
  `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior Dantzig Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint: `Fast path strict-interior Dantzig LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 45 commits.
- No PR is associated with this branch yet.
- Pushes still require explicit maintainer/user approval.

What this slice changes:

- `DantzigSolver` now tries the shared validated strict-interior standard-LCP
  fast path through the `LcpProblem` solver interface before allocating the
  ODE-derived pivot workspace.
- The fast path runs after problem validation and only when callers are not
  warm-starting.
- Boxed, friction-index, warm-started, and low-level matrix/scratch Dantzig
  calls stay on the existing pivoting path.
- Unit coverage was extended so
  `StandardStrictInteriorFastPath.PivotAndBarrierSolversUseLinearSolve`
  includes `DantzigSolver` with `warmStart=false`.
- The Python LCP demo profile summary now removes `Dantzig` from Standard
  laggards and points remaining Standard tuning at ADMM, MPRGP, and moderate
  iterative rows.

Evidence:

- Focused `BM_LcpCompare/Standard/Dantzig/` after-run reported
  `contract_ok=1.0` and `iterations=0` for 12-, 24-, 48-, and 96-row Standard
  Dantzig packets.
- Full regenerated Standard profile now reports `Dantzig` average ratio `1.24`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to `build/dantzig_strict_interior_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- CSV shape check reported 15 Boxed columns, 16 FrictionIndex columns,
  23 Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: `MPRGP`, `Admm`, plus moderate `Jacobi`, `RedBlackGaussSeidel`,
  `Apgd`, and `SymmetricPsor` rows.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `Staggering`, `ShockPropagation`, and
  `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior Boxed Semi-Smooth Newton Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Top local checkpoint: `Fast path strict-interior boxed Newton LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 44 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

What this slice changes:

- `BoxedSemiSmoothNewtonSolver` now tries the shared validated
  strict-interior standard-LCP fast path after parameter validation when the
  caller is not warm-starting.
- Warm-started boxed and friction-index rows stay on the existing
  residual-reducing PGS warm-start and semi-smooth Newton line-search path.
- Unit coverage was extended so
  `StandardStrictInteriorFastPath.NewtonSolversUseLinearSolve` includes
  `BoxedSemiSmoothNewtonSolver` with `warmStart=false`.
- The Python LCP demo profile summary now removes `BoxedSemiSmoothNewton` from
  Standard laggards and points remaining Standard tuning at ADMM, Dantzig,
  MPRGP, and moderate iterative rows.

Evidence:

- Focused `BM_LcpCompare/Standard/BoxedSemiSmoothNewton/` after-run compared to
  the previous full profile cache:
  - 12 rows: `0.365`
  - 24 rows: `0.384`
  - 48 rows: `0.731`
  - 96 rows: `0.416`
  - Mean focused ratio `0.474`; best `0.365`; worst `0.731`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Full regenerated Standard profile now reports `BoxedSemiSmoothNewton` average
  ratio `1.28`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/boxed_ssn_strict_interior_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- CSV shape check reported 15 Boxed columns, 16 FrictionIndex columns,
  23 Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed: `1 passed in 0.42s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: `Admm`, `Dantzig`, `MPRGP`, plus moderate `RedBlackGaussSeidel`,
  `Jacobi`, `Apgd`, and `SymmetricPsor` rows.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `Staggering`, `ShockPropagation`, and
  `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior BGS Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Current `HEAD`: `f97ce3b9bf6 Fast path strict-interior projection LCPs`.
- This section records the local checkpoint titled
  `Fast path strict-interior BGS LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 43 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

What this slice changes:

- `BgsSolver` now tries the shared validated strict-interior standard-LCP fast
  path for small and medium standard rows when the caller is not warm-starting.
- The fast path is capped at 48 rows because the dense strict-interior linear
  solve regressed the 96-row BGS comparison row relative to the existing scalar
  block sweep.
- Default-option solves can take the fast path before block construction.
  Explicit custom block partitions are still validated before the fast path is
  accepted.
- Unit coverage was extended so
  `StandardStrictInteriorFastPath.ProjectionAndBlockSolversUseLinearSolve`
  includes `BgsSolver`.
- The Python LCP demo profile summary now removes `BGS` from Standard laggards
  and points remaining Standard tuning at boxed semi-smooth Newton, ADMM,
  Dantzig, MPRGP, and moderate iterative rows.

Evidence:

- Focused `BM_LcpCompare/Standard/BGS/` after-run compared to the previous full
  profile cache:
  - 12 rows: `0.257`
  - 24 rows: `0.341`
  - 48 rows: `0.493`
  - 96 rows: `0.827`
  - Mean focused ratio `0.480`; best `0.257`; worst `0.827`.
  - All focused rows reported `contract_ok=1.0`; fast-path rows reported
    `iterations=0`, and the capped 96-row fallback reported `iterations=5`.
- Full regenerated Standard profile now reports `BGS` average ratio `1.39`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to `build/bgs_strict_interior_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- CSV shape check reported 15 Boxed columns, 16 FrictionIndex columns,
  23 Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed: `1 passed in 0.44s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: `BoxedSemiSmoothNewton`, `Admm`, `Dantzig`, `MPRGP`, plus moderate
  `Jacobi`, `Apgd`, and `SymmetricPsor` rows.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, and
  `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior Projection/Block Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Current `HEAD`: `2d74356e830 Fast path strict-interior Newton LCPs`.
- This section records the local checkpoint titled
  `Fast path strict-interior projection LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 42 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

What this slice changes:

- `NncgSolver` and `SubspaceMinimizationSolver` now try
  `detail::trySolveStrictInteriorStandardLcp()` after parameter validation and
  before their PGS setup when the caller is not warm-starting.
- `BlockedJacobiSolver` and `ShockPropagationSolver` now use a local fast-path
  helper. With default options, they try the strict-interior standard-LCP fast
  path before expensive block/layer construction. With custom options, they
  still validate the custom block/layer inputs first, then can take the fast
  path.
- Unit coverage was added as
  `StandardStrictInteriorFastPath.ProjectionAndBlockSolversUseLinearSolve`.
- Existing `BlockedJacobiSolverCoverage.ReportsMaxIterationsWhenNotConverged`
  was adjusted to use a boundary target so it still exercises the iteration
  path instead of the strict-interior fast path.
- The Python LCP demo profile summary was updated from the regenerated profile
  so the Standard leaders mention strict-interior pivot/barrier/Newton,
  projection, and block rows, with remaining Standard targets centered on
  `BGS`, `Admm`, `BoxedSemiSmoothNewton`, `Dantzig`, and `MPRGP`.
- Background docs and `CHANGELOG.md` now record the projection/block
  strict-interior fast-path behavior and validation boundaries.

Evidence:

- Rebuilt the relevant benchmark/unit targets and reran the focused validation
  CTest after the boundary-test adjustment; it passed with
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark filter:
  `BM_LcpCompare/Standard/(BlockedJacobi|NNCG|ShockPropagation|SubspaceMinimization)/`.
- After moving default `BlockedJacobi` and `ShockPropagation` fast paths before
  block/layer setup, focused ratios versus the previous full profile cache
  were:
  - `BlockedJacobi`: `0.1646`, `0.1742`, `0.4021`, `0.5214`
  - `NNCG`: `0.1483`, `0.2532`, `0.3607`, `0.5038`
  - `SubspaceMinimization`: `0.3274`, `0.3717`, `0.3653`, `0.3545`
  - `ShockPropagation`: `0.2511`, `0.3449`, `0.5894`, `0.8568`
  - Mean focused ratio `0.3743`; best `0.1483`; worst `0.8568`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Full profile regeneration completed and updated the checked CSVs. Standard
  profile snapshot after regeneration:
  - `Admm` `2.65`, `Apgd` `1.85`, `BGS` `2.73`, `Baraff` `1.42`,
    `BlockedJacobi` `1.26`, `BoxedSemiSmoothNewton` `2.63`, `Dantzig` `2.39`,
    `Direct` `1.00`, `FischerBurmeisterNewton` `1.52`, `InteriorPoint` `1.32`,
    `Jacobi` `1.92`, `Lemke` `1.36`, `MPRGP` `2.26`,
    `MinimumMapNewton` `1.41`, `NNCG` `1.48`,
    `PenalizedFischerBurmeisterNewton` `1.41`, `Pgs` `1.10`,
    `RedBlackGaussSeidel` `1.83`, `Sap` `1.48`, `ShockPropagation` `1.68`,
    `SubspaceMinimization` `1.32`, `SymmetricPsor` `1.82`, `Tgs` `1.13`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/projection_block_strict_interior_after.json`; all 16 focused rows
  reported `contract_ok=1.0` and `iterations=0`.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- CSV shape check reported 15 Boxed columns, 16 FrictionIndex columns,
  23 Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed: `1 passed in 0.44s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: `BGS`, `Admm`, `BoxedSemiSmoothNewton`, `Dantzig`, and `MPRGP`.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, and `Nncg`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, and
  `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior Newton Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Parent checkpoint: `9057b15a9bf Fast path strict-interior standard LCPs`
- Latest local checkpoint: `Fast path strict-interior Newton LCPs`
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 41 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

What this slice changes:

- Reuses `detail::trySolveStrictInteriorStandardLcp()` in
  `MinimumMapNewtonSolver`, `FischerBurmeisterNewtonSolver`, and
  `PenalizedFischerBurmeisterNewtonSolver`.
- The fast path runs only after solver-parameter validation and only when the
  caller is not warm-starting, preserving invalid custom option handling and
  warm-started Newton behavior.
- Adds unit coverage for zero-iteration strict-interior success on all three
  standard-LCP Newton solvers.
- Regenerates the LCP performance profile CSVs and updates demo/docs/changelog
  text.

Evidence:

- Focused `BM_LcpCompare/Standard/(FischerBurmeisterNewton|PenalizedFischerBurmeisterNewton|MinimumMapNewton)/`
  after-run compared to the previous full profile cache:
  - Minimum Map Newton ratios: `0.193`, `0.264`, `0.310`, `0.114`.
  - Fischer-Burmeister Newton ratios: `0.099`, `0.119`, `0.153`, `0.135`.
  - Penalized Fischer-Burmeister Newton ratios: `0.097`, `0.073`, `0.163`,
    `0.139`.
  - Mean focused ratio `0.155`; best `0.073`; worst `0.310`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Full regenerated profile snapshot:
  - Standard `MinimumMapNewton` average ratio `1.43`.
  - Standard `PenalizedFischerBurmeisterNewton` average ratio `1.55`.
  - Standard `FischerBurmeisterNewton` average ratio `1.57`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/newton_strict_interior_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- CSV shape check reported 15 Boxed columns, 16 FrictionIndex columns,
  23 Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed: `1 passed in 0.39s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: `BlockedJacobi`, `Nncg`, `ShockPropagation`, and
  `SubspaceMinimization`.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, and `Nncg`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, and
  `SubspaceMinimization`.

## Current Reality - 2026-06-12 Strict-Interior Standard Fast Path

This section is the latest state; older sections below are historical
checkpoints.

Checkpoint hand-off:

- The strict-interior standard-LCP fast path has been implemented and verified
  locally.
- This section records the checkpoint titled
  `Fast path strict-interior standard LCPs`.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

Current branch:

- `feature/lcp-solver-interface-demos`
- Parent checkpoint: `4e6e57e7ccb Reuse ADMM iteration workspace`
- Latest local checkpoint: `Fast path strict-interior standard LCPs`
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 40 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

What this slice changes:

- Adds shared `detail::trySolveStrictInteriorStandardLcp()`.
- `LemkeSolver`, `BaraffSolver`, and `InteriorPointSolver` use it only when the
  caller is not warm-starting.
- It accepts only strict positive standard-LCP linear solves that pass normal
  solution validation; active-bound, boxed, and friction-index cases keep the
  existing algorithm/fallback paths.
- Adds unit coverage for zero-iteration strict-interior success on all three
  solvers.
- Regenerates the LCP performance profile CSVs and updates demo/docs/changelog
  text.

Checkpoint surface:

- Core implementation: `dart/math/lcp/lcp_validation.hpp`,
  `dart/math/lcp/pivoting/lemke_solver.cpp`,
  `dart/math/lcp/pivoting/baraff_solver.cpp`, and
  `dart/math/lcp/other/interior_point_solver.cpp`.
- Coverage and user-facing metadata:
  `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/examples/demos/scenes/lcp_physics.py`, and `CHANGELOG.md`.
- Background/profile artifacts:
  `docs/background/lcp/03_pivoting-methods.md`,
  `docs/background/lcp/06_other-methods.md`, and
  `docs/background/lcp/figures/performance_profile_*.csv`.
- Hand-off docs: this file and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md`.

Evidence:

- Focused `BM_LcpCompare/Standard/(Lemke|Baraff|InteriorPoint)/` after-run
  compared to the previous full profile cache:
  - Lemke ratios: `0.058`, `0.033`, `0.018`, `0.004`.
  - Baraff ratios: `0.083`, `0.052`, `0.026`, `0.013`.
  - Interior Point ratios: `0.021`, `0.021`, `0.028`, `0.013`.
  - Mean focused ratio `0.031`; best `0.004`; worst `0.083`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.
- Full regenerated profile snapshot:
  - Standard `Lemke` average ratio `1.42`.
  - Standard `Baraff` average ratio `1.35`.
  - Standard `InteriorPoint` average ratio `1.44`.

Verification completed:

- Relevant benchmark and unit-test targets rebuilt.
- Focused additional/all-solvers/validation CTests passed:
  `100% tests passed, 0 tests failed out of 3`.
- Focused strict-interior validation CTest passed after adding the direct unit
  coverage: `100% tests passed, 0 tests failed out of 1`.
- Focused benchmark JSON written to
  `build/strict_interior_standard_after.json`.
- Full profile regenerated into `docs/background/lcp/figures`.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- CSV shape check reported 15 Boxed columns, 16 FrictionIndex columns,
  23 Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed: `1 passed in 0.41s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets after this slice:

- Standard: `PenalizedFischerBurmeisterNewton`,
  `FischerBurmeisterNewton`, `MinimumMapNewton`, `Nncg`, `BlockedJacobi`, and
  `ShockPropagation`.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, and `Nncg`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, and
  `SubspaceMinimization`.

## Current Reality - 2026-06-12 ADMM Workspace Reuse

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Latest local checkpoint: `Reuse ADMM iteration workspace`.
- Parent checkpoint:
  `e8e569ca5af Warm start boxed semi-smooth Newton profiles`
- At this checkpoint and before pushing, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 39 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation; pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `AdmmSolver` reuses its right-hand-side and projected-step vectors across
  iterations.
- The unused per-iteration `xPrev` copy was removed.
- The ADMM math, adaptive-rho path, convergence checks, and public parameter
  surface are unchanged.
- The checked performance profile CSVs were regenerated.
- The Python demo profile summary, panel metadata test, ADMM docs, changelog,
  and task docs were updated.

Evidence:

- Focused `BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Admm/` after-run
  compared to the previous full profile cache:
  - Standard ratios: `0.643`, `0.757`, `0.789`, `0.691`.
  - Boxed ratios: `0.688`, `0.626`, `0.679`.
  - FrictionIndex ratios: `0.492`, `0.532`, `0.486`.
  - Mean focused ratio `0.638`; best `0.486`; worst `0.789`.
  - All focused rows reported `contract_ok=1.0`.
- Full regenerated profile snapshot:
  - Standard `Admm` average ratio `2.61`.
  - Boxed `Admm` average ratio `14.90`.
  - FrictionIndex `Admm` average ratio `2.55`.

Verification completed:

- `BM_LCP_COMPARE` and
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` rebuilt.
- Focused ADMM compare JSON written to `build/admm_workspace_after.json`.
- Focused validation CTest passed:
  `100% tests passed, 0 tests failed out of 1`.
- Full profile regenerated into `docs/background/lcp/figures`.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- CSV shape check reported 15 Boxed columns, 16 FrictionIndex columns,
  23 Standard columns, and 200 rows per profile.
- Focused Python panel metadata test passed: `1 passed in 0.80s`.
- `pixi run build` passed.
- `pixi run lint` passed.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue from the refreshed profile. Do not push without explicit
maintainer/user approval.

Current next targets:

- Standard: `Lemke`, `InteriorPoint`, and `Baraff`.
- Boxed: `Admm`, `ShockPropagation`, `Dantzig`, and `Nncg`.
- FrictionIndex: `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, and
  `SubspaceMinimization`.

## Current Reality - 2026-06-12 Boxed SSN Warm-Start Verified

This section is the latest state; older sections below are historical
checkpoints.

Current branch:

- `feature/lcp-solver-interface-demos`
- Latest local checkpoint: `Warm start boxed semi-smooth Newton profiles`.
- The parent checkpoint is
  `ec2d642c945 Update LCP solver handoff after SAP checkpoint`.
- At this checkpoint and before pushing, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 38 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation; pushes still require
  explicit maintainer/user approval.

What this slice changes:

- `BoxedSemiSmoothNewtonSolver::Parameters` gained
  `maxPgsWarmStartIterations` and `pgsWarmStartRelaxation`.
- Solver implementation optionally runs a short `PgsSolver` warm start before
  Newton iterations and keeps it only if the boxed natural residual improves.
- `BM_LcpCompare` sets a 5-iteration PGS warm start for
  `BoxedSemiSmoothNewton` only on bounded/findex comparison packets.
- Benchmark counters expose the new warm-start settings.
- dartpy bindings, C++ tests, Python tests, demo metadata, background docs,
  profile CSV artifacts, and `CHANGELOG.md` have corresponding updates.

Benchmark evidence:

- Pre-warm-start focused baseline:
  `build/boxed_ssn_jacobian_copy_before.json`.
- Focused after-run for
  `BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BoxedSemiSmoothNewton/`:
  standard rows stayed neutral (`0.976`, `1.009`, `0.954`, `0.999`), boxed
  rows improved to `0.383`, `0.220`, `0.113`, and friction-index rows improved
  to `0.336`, `0.090`, `0.017`.
- Focused after-run mean ratio `0.5096`, best `0.0174`, worst `1.0086`; all
  rows reported `contract_ok=1.0`.
- Target build plus focused CTests passed before the stop request:
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` and
  `UNIT_math_lcp_math_lcp_all_solvers_smoke`, with
  `100% tests passed, 0 tests failed out of 2`.
- Full profile regeneration completed and refreshed
  `docs/background/lcp/figures/performance_profile_*.csv`.
- Regenerated profile snapshot after warm-start:
  - Standard `BoxedSemiSmoothNewton` average ratio `3.24`.
  - Boxed `BoxedSemiSmoothNewton` average ratio `3.67`, down from about
    `13.44`.
  - FrictionIndex `BoxedSemiSmoothNewton` average ratio `3.47`, down from
    about `48.88`.

Verification completed:

- Cached profile replay from `build/lcp_profile_full.json` completed and wrote
  `build/lcp_profile_full_check/performance_profile_*.csv`.
- CSV shape check reported 15 Boxed solver columns, 16 FrictionIndex solver
  columns, 23 Standard solver columns, and 200 rows per profile.
- `pixi run build` passed and rebuilt dartpy.
- Focused Python tests passed: `5 passed in 0.33s`.
- Focused CTests passed:
  `100% tests passed, 0 tests failed out of 2`.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

How to resume:

```bash
git checkout feature/lcp-solver-interface-demos
git status -sb
git log -5 --oneline --decorate
git diff --stat
```

Continue benchmark-driven optimization or interface audit from the refreshed
profile. Do not push without explicit maintainer/user approval.

Updated remaining profile targets after this slice:

- Standard: `Lemke`, `InteriorPoint`, `Baraff`, and Newton-family rows.
- Boxed: `Admm`, `ShockPropagation`, `Nncg`, `BGS`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `Staggering`, `BGS`,
  `ShockPropagation`, and `SubspaceMinimization`.

## Current Reality - 2026-06-12 Post-SAP Checkpoint

Current branch state after the SAP slice:

- Branch: `feature/lcp-solver-interface-demos`.
- Latest local checkpoint:
  `64777bcf7da Tune SAP boxed profile regularization`.
- Tracking state after the checkpoint: ahead of
  `origin/feature/lcp-solver-interface-demos` by 36 commits.
- Worktree was clean before this hand-off update.
- No PR is associated with this branch yet.
- No push has been performed in this continuation; pushes still require
  explicit maintainer/user approval.

Discarded follow-up investigation:

- Tried changing `BoxedSemiSmoothNewtonSolver` Jacobian assembly from
  `J.setZero()` plus interior-row copies to `J = A` plus active-row rewrites.
- Focused before/after
  `BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BoxedSemiSmoothNewton/` showed
  a clear regression: mean ratio `1.844`, best `1.080`, worst `2.271`, with
  every row slower.
- The draft was reverted completely and should not be reattempted without new
  evidence.
- The line-search sweep also did not justify a broad default change: more
  search steps only helped the 16-contact friction-index sweep row modestly
  while leaving other rows unchanged or slower.

Immediate next step:

- Continue from the refreshed profile after the SAP checkpoint. The clearest
  remaining targets are Standard `Lemke`, `InteriorPoint`, `Baraff`, and
  Newton-family rows; Boxed `Admm`, `BoxedSemiSmoothNewton`, and
  `ShockPropagation`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, `BGS`, `ShockPropagation`, and `Staggering`. For
  `BoxedSemiSmoothNewton`, avoid the reverted Jacobian-copy approach above.

## Current Reality - 2026-06-12 SAP Boxed Profile Regularization

After the ADMM checkpoint below, work continued on the next high-ratio bounded
profile target: `SapSolver` on boxed rows. The production solver default was
already `regularization = 1e-4`, but the main `BM_LcpCompare/*/Sap` profile was
overriding SAP to `1e-6` for every family. The focused regularization sweep
showed that boxed rows meet the comparison contract at `1e-4` and drop from
52 iterations to 3 on the 24-row boxed fixture, while standard and
friction-index rows were better kept at `1e-6`.

Latest implementation slice:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now configures SAP compare
  rows with `regularization = 1e-4` only for bounded non-findex packets, and
  keeps `regularization = 1e-6` for standard and friction-index packets.
- SAP benchmark counters now record `sap_regularization` and
  `sap_max_line_search_iterations` for main compare rows as well as contact
  rows, making cached benchmark JSON self-describing.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated.
- `docs/background/lcp/06_other-methods.md` documents the profile policy and
  keeps the regularization sweep as the source of evidence for stricter values.
- The Python demo profile summary and panel test were updated for the refreshed
  Standard and Boxed leader/laggard text.
- `CHANGELOG.md` records the SAP profile tuning.

Focused SAP before/after `BM_LcpCompare` evidence:

- Standard profile rows stayed effectively unchanged: runtime ratios `1.026`,
  `1.055`, `1.084`, and `1.057` for sizes 12, 24, 48, and 96 relative to the
  pre-change focused run; all still used `sap_regularization = 1e-6`.
- Boxed profile rows improved sharply: runtime ratios `0.060`, `0.038`, and
  `0.028` for sizes 12, 24, and 48; iterations changed from 34/52/70 to
  3/3/3; all used `sap_regularization = 1e-4`.
- FrictionIndex profile rows stayed effectively unchanged or improved within
  benchmark noise: runtime ratios `1.002`, `0.765`, and `0.956`; all still used
  `sap_regularization = 1e-6`.
- Mean focused ratio was `0.707`; best `0.028`; worst `1.084`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `Sap` average ratio is now `1.19`; current leaders are `Tgs`,
  `Pgs`, `Sap`, and tiny-row `Direct`; current high-ratio targets remain
  `Lemke`, `InteriorPoint`, `Baraff`, and Newton-family rows.
- Boxed: `Sap` average ratio improved from about `60.35` to `2.53`; current
  leaders are `Tgs`, `Pgs`, and `Jacobi`; current high-ratio targets are
  `Admm`, `BoxedSemiSmoothNewton`, and `ShockPropagation`.
- FrictionIndex: `Sap` average ratio is now `1.30`; current leaders are `Pgs`,
  `Tgs`, `Sap`, `SymmetricPsor`, and `Jacobi`; current high-ratio targets are
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `ShockPropagation`, and
  `Staggering`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpSapRegularizationSweep/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/sap_regularization_sweep_current.json
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Sap/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/sap_compare_before.json
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Sap/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/sap_compare_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Observed results:

- `BM_LCP_COMPARE` rebuilt successfully after the benchmark option change.
- The focused regularization sweep confirmed boxed `Reg1e_4` was the fast
  contract-passing row for the current 24-row boxed fixture.
- Focused SAP after-run showed all Standard, Boxed, and FrictionIndex profile
  rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- Focused Python LCP panel metadata test passed: `1 passed in 0.40s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Checkpoint this SAP profile tuning slice. After that, continue
  benchmark-driven optimization or interface audit from the refreshed profile.
  The clearest remaining profile targets are Standard `Lemke`, `InteriorPoint`,
  `Baraff`, and Newton-family rows; Boxed `Admm`,
  `BoxedSemiSmoothNewton`, and `ShockPropagation`; and FrictionIndex
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `ShockPropagation`, and
  `Staggering`.

## Current Reality - 2026-06-12 ADMM Default Rho Tuning

After the ShockPropagation checkpoint below, work continued from the refreshed
profile. A first attempt targeted `BoxedSemiSmoothNewtonSolver` by preallocating
line-search residual workspaces and skipping non-findex trial-bound copies, but
focused before/after benchmarking showed it was not worth keeping: mean ratio
`1.011`, best `0.878`, worst `1.082`, with large Standard and FrictionIndex
rows regressing. That draft was manually reverted before switching targets.

Latest implementation slice:

- `dart/math/lcp/other/admm_solver.hpp` now defaults
  `AdmmSolver::Parameters::rhoInit` to `4.0`.
- The change keeps the same projection, proximal linear solve, and adaptive
  residual-balancing algorithm; only the initial penalty for the existing
  adaptive path changes.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` now checks the new
  ADMM default parameter.
- `docs/background/lcp/06_other-methods.md` documents the default-rho decision
  and points to the DART-owned adaptive-rho sweep evidence.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated.
- The Python demo profile summary and panel test were updated for the refreshed
  Standard and Boxed leader/laggard text.
- `CHANGELOG.md` records the ADMM tuning.

Focused ADMM before/after `BM_LcpCompare` evidence:

- Standard profile rows: runtime ratios `0.806`, `0.653`, `0.566`, and `0.543`
  for sizes 12, 24, 48, and 96 relative to the previous default-rho profile
  cache.
- Boxed profile rows: runtime ratios `0.360`, `0.373`, and `0.364` for sizes
  12, 24, and 48.
- FrictionIndex profile rows: runtime ratios `0.903`, `0.548`, and `0.446` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.556`; best `0.360`; worst `0.903`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `Admm` average ratio is now `4.05`; current leaders are `Tgs`,
  tiny-row `Direct`, `Sap`, and `Pgs`; current high-ratio targets include
  `Lemke`, `InteriorPoint`, `Baraff`, and Newton-family rows.
- Boxed: `Admm` average ratio improved from about `31.06` to `14.51`; current
  leaders are `Pgs`, `Jacobi`, `Tgs`, and `SymmetricPsor`; current high-ratio
  targets are `Sap`, `Admm`, `BoxedSemiSmoothNewton`, and `ShockPropagation`.
- FrictionIndex: `Admm` average ratio is now `2.32`; current leaders are `Tgs`,
  `Pgs`, `Sap`, `SymmetricPsor`, and `Jacobi`; current high-ratio targets are
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `ShockPropagation`, and
  `Staggering`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Admm/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/admm_profile_rho4_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Observed results:

- Affected C++ targets rebuilt successfully.
- Comparison, generated coverage, and validation solver tests passed through
  CTest: `100% tests passed, 0 tests failed out of 3`.
- Focused ADMM after-run showed all Standard, Boxed, and FrictionIndex profile
  rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- Focused Python LCP panel metadata test passed: `1 passed in 0.41s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. The clearest remaining profile targets are Standard `Lemke`,
  `InteriorPoint`, `Baraff`, and Newton-family rows; Boxed `Sap`,
  `BoxedSemiSmoothNewton`, and `ShockPropagation`; and FrictionIndex
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `ShockPropagation`, and
  `Staggering`.

## Current Reality - 2026-06-12 ShockPropagation Feasible-Block Fast Path

The goal was resumed after the stop-only hand-off below. Work continued on the
next high-ratio profile target: `ShockPropagationSolver`.

Latest implementation slice:

- `dart/math/lcp/other/shock_propagation_solver.cpp` now tries a small
  `Eigen::FullPivLU` local solve for uncoupled fixed-bound blocks when the
  unconstrained candidate is finite and already feasible.
- Active-bound, singular, non-finite, larger-than-direct, and local `findex`
  blocks still use the existing Direct/Dantzig local subproblem fallback.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` covers a feasible
  boxed 3-row block that should solve directly in one outer iteration.
- `tests/unit/math/lcp/test_additional_solvers.cpp` keeps the non-standard
  fallback test active by forcing an upper-bound clamp.
- `docs/background/lcp/06_other-methods.md` documents the feasible-block fast
  path.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated from the optimized solver.
- The Python demo profile summary and panel test were updated for the refreshed
  Standard and FrictionIndex leader/laggard text.
- `CHANGELOG.md` records the `ShockPropagationSolver` optimization.

Focused before/after `BM_LcpCompare` evidence for this slice:

- Standard profile rows: runtime ratios `0.096`, `0.187`, `0.182`, and `0.196`
  for sizes 12, 24, 48, and 96 relative to the pre-change focused run.
- Boxed profile rows: runtime ratios `1.031`, `0.651`, and `0.614` for sizes
  12, 24, and 48; the smallest boxed row regressed within benchmark noise while
  larger rows improved.
- FrictionIndex profile rows: runtime ratios `0.463`, `0.586`, and `0.777` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.478`; best `0.096`; worst `1.031`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `ShockPropagation` average ratio is now `7.75`; current leaders are
  `Pgs`, tiny-row `Direct`, `Tgs`, and `SymmetricPsor`; current high-ratio
  targets include `Lemke`, `InteriorPoint`, `Baraff`, and Newton-family rows.
- Boxed: `ShockPropagation` average ratio is now `19.70`; current leaders are
  `Jacobi`, `Tgs`, `Apgd`, and `SymmetricPsor`; current high-ratio targets are
  `Sap`, `Admm`, `BoxedSemiSmoothNewton`, and `ShockPropagation`.
- FrictionIndex: `ShockPropagation` average ratio is now `6.94`; current
  leaders are `Tgs`, `Sap`, `Pgs`, `SymmetricPsor`, and `Jacobi`; current
  high-ratio targets are `BoxedSemiSmoothNewton`, `BlockedJacobi`,
  `Staggering`, `BGS`, and `ShockPropagation`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_additional_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(additional_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/ShockPropagation/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/shock_profile_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Observed results:

- The first corrected build attempt failed because `Eigen::FullPivLU` needed
  `#include <Eigen/LU>`; that include was added and the corrected build/test
  target set was rerun successfully.
- Affected C++ targets rebuilt successfully.
- Additional solver, comparison, generated coverage, and validation solver
  tests passed through CTest: `100% tests passed, 0 tests failed out of 4`.
- Focused ShockPropagation after-run showed all Standard, Boxed, and
  FrictionIndex profile rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- Focused Python LCP panel metadata test passed: `1 passed in 0.48s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. The clearest remaining profile targets are Standard `Lemke`,
  `InteriorPoint`, `Baraff`, Newton-family rows, and `BlockedJacobi`; Boxed
  `Sap`, `Admm`, and `BoxedSemiSmoothNewton`; and FrictionIndex
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `Staggering`, and `BGS`.

## Current Reality - 2026-06-12 Stop-Only Hand-Off During ShockPropagation Draft

The user explicitly stopped all further work and requested only hand-off docs,
with no further verification. Treat this as an interrupted state, not a
completed checkpoint. Do not continue implementation, verification, merging,
committing, pushing, or PR work unless the user explicitly asks to resume.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD:
  `8d42442e52c Optimize BGS singleton LCP blocks`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 33]`.
- Pre-existing worktree edits were present in three C++/test files:
  `dart/math/lcp/other/shock_propagation_solver.cpp`,
  `tests/unit/math/lcp/test_additional_solvers.cpp`, and
  `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`.
- This hand-off edit intentionally did not merge `main`, create a branch,
  commit, push, open/update a PR, or run any additional verification.

Uncommitted ShockPropagation draft currently in the worktree:

- `ShockPropagationSolver` has a new `solveUnconstrainedFeasibleBlock()` draft
  helper that tries a small `Eigen::FullPivLU` local solve for finite,
  uncoupled fixed-bound blocks when the unconstrained candidate is already
  feasible. Active-bound, singular, non-finite, larger-than-direct, and local
  `findex` blocks fall back to the existing Direct/Dantzig path.
- The solver loop calls that helper before constructing the local subproblem.
- `test_lcp_validation_and_solvers.cpp` adds
  `ShockPropagationSolverCoverage.SolvesFeasibleBoxedBlockDirectly`.
- `test_additional_solvers.cpp` adjusts
  `ShockPropagationSolver.NonStandardBlockUsesDantzig` so the boxed row clamps
  at the upper bound, preserving fallback coverage if the feasible-block fast
  path is kept.

Verification state for the interrupted draft:

- A focused ShockPropagation baseline benchmark was run before these draft
  edits and wrote `build/shock_profile_before.json`.
- A first focused build/test attempt after the draft edits used the wrong CMake
  target name, `UNIT_math_lcp_math_lcp_lcp_additional_solvers`. Ninja reported
  that target as unknown. Because that command did not fail fast, three older
  CTests still ran and passed; do not treat that as validation of the current
  draft.
- The correct target/test name discovered for the additional solver test is
  `UNIT_math_lcp_math_lcp_additional_solvers`.
- No correct compile, CTest, benchmark, lint, build, or `git diff --check` was
  run after the stop request or for this docs-only hand-off.

If a fresh session is explicitly asked to resume, first inspect the current
branch/worktree and reread `AGENTS.md`, `docs/ai/principles.md`,
`docs/dev_tasks/README.md`, this file, and the task `README.md`. Then decide
whether to keep, revise, or remove the unverified ShockPropagation draft. If
keeping it, the first required validation target set is:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_additional_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(additional_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
```

Watch for a possible missing Eigen LU include around `Eigen::FullPivLU`. If the
draft compiles, run the focused ShockPropagation before/after benchmark before
updating profile artifacts or docs. Do not push without explicit approval.

## Current Reality - 2026-06-12 BGS Singleton Fast Path

The goal was resumed after the stop-only hand-off below. Work continued on the
next high-ratio profile target: `BgsSolver`.

Latest implementation slice:

- `dart/math/lcp/projection/bgs_solver.cpp` now detects singleton blocks with
  no local `findex` coupling and solves the scalar fixed-bound subproblem with
  a direct projected Gauss-Seidel update.
- The fast path requires a finite positive diagonal and finite effective RHS.
  Non-singleton blocks, local `findex` blocks, non-positive diagonals, invalid
  scalar data, and coupled friction blocks still use the existing
  `DirectSolver`/`DantzigSolver` local block fallback.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` now covers BGS
  singleton boxed blocks that clamp against both lower and upper bounds.
- `docs/background/lcp/04_projection-methods.md` documents the singleton
  scalar BGS path.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated from the optimized solver.
- The Python demo profile summary and panel test were updated for the refreshed
  Boxed and FrictionIndex leader/laggard text.
- `CHANGELOG.md` records the `BgsSolver` optimization.

Focused before/after `BM_LcpCompare` evidence for this slice:

- Standard profile rows: runtime ratios `0.133`, `0.148`, `0.180`, and `0.292`
  for sizes 12, 24, 48, and 96 relative to the pre-change focused run.
- Boxed profile rows: runtime ratios `0.115`, `0.123`, and `0.098` for sizes
  12, 24, and 48.
- FrictionIndex profile rows: runtime ratios `0.623`, `0.815`, and `0.982` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.351`; best `0.098`; worst `0.982`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `BGS` average ratio is now `2.42`. Current high-ratio targets remain
  `Lemke`, `InteriorPoint`, `Baraff`, and `ShockPropagation`.
- Boxed: `BGS` average ratio is now `4.22`, no longer in the top laggard set.
  Current high-ratio targets are `Sap`, `Admm`, `BoxedSemiSmoothNewton`, and
  `ShockPropagation`.
- FrictionIndex: `BGS` average ratio is now `7.16`; current high-ratio targets
  are `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, and
  `SubspaceMinimization`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_lcp_projection_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_projection_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BGS/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/bgs_profile_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Observed results:

- Affected C++ targets rebuilt successfully.
- LCP projection, comparison, generated coverage, and validation solver tests
  passed through CTest: `100% tests passed, 0 tests failed out of 4`.
- Focused BGS after-run showed all Standard, Boxed, and FrictionIndex profile
  rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- Focused Python LCP panel metadata test passed: `1 passed in 0.48s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. The clearest remaining profile targets are Standard `Lemke`,
  `InteriorPoint`, `Baraff`, `ShockPropagation`; Boxed `Sap`, `Admm`,
  `BoxedSemiSmoothNewton`, `ShockPropagation`; and FrictionIndex
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `SubspaceMinimization`.
- Do not push without explicit approval.

## Current Reality - 2026-06-12 Stop-Only Hand-Off After Snapshot Product

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. Do not infer that lint, tests, builds, benchmarks,
implementation edits, commits, pushes, or PR work should continue from this
session. Resume implementation only after an explicit user request.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD:
  `4823706f0c9 Reuse BlockedJacobi fixed-bound snapshot products`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 32]`.
- Worktree: clean before this docs-only hand-off edit.
- Push was not performed in this stop-only response.
- No lint, test, build, benchmark, or other verification command was run after
  the stop request.

Latest completed local checkpoints:

- `4823706f0c9 Reuse BlockedJacobi fixed-bound snapshot products`
- `e7ea201e258 Optimize BlockedJacobi singleton LCP blocks`
- `3667401ad1f Optimize boxed semi-smooth Newton LCP step`
- `d27a0232c37 Show LCP performance profiles in py demo`
- `f857a380c20 Regenerate LCP performance profiles`
- `dcf0d835c1b Refresh LCP performance profile tooling`
- `7df9a018c31 Align LCP performance profile metadata`
- `00a58f7153e Align Staggering native capability metadata`

Immediate next step:

- Stop. In a fresh session, first inspect the current branch/worktree state and
  reread `AGENTS.md`, `docs/ai/principles.md`, `docs/dev_tasks/README.md`,
  this file, and the task `README.md`.
- The latest completed implementation slice is the BlockedJacobi snapshot
  product section immediately below. Its verification evidence is historical
  evidence from before this stop request, not work performed after it.
- The broad LCP solver/interface/demo goal remains incomplete. Existing profile
  data still suggests high-ratio future targets such as Standard `Lemke`,
  `InteriorPoint`, `Baraff`, `ShockPropagation`; Boxed `Sap`, `Admm`, `BGS`,
  `BoxedSemiSmoothNewton`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, `ShockPropagation`, `Staggering`, `BGS`. Do not start any
  of that work until explicitly asked.
- Do not push without explicit approval.

## Current Reality - 2026-06-12 BlockedJacobi Snapshot Product

After the singleton fast path checkpoint, work continued on the same
`BlockedJacobiSolver` profile target. The remaining hot loop repeated
`A.row(i).dot(xPrev)` inside every block update. Fixed-bound Standard and Boxed
problems now precompute the Jacobi snapshot product `A * xPrev` once per
iteration and reuse it for block right-hand sides. Coupled friction-index
problems keep the prior row-product path because an unconditional snapshot
slightly slowed those rows during focused testing.

Latest implementation slice:

- `dart/math/lcp/projection/blocked_jacobi_solver.cpp` passes an optional
  snapshot product into `solveBlock()`.
- Standard and Boxed fixed-bound problems use the snapshot product for singleton
  and non-singleton block RHS construction.
- Friction-index problems continue using per-row products inside their coupled
  block path.
- `docs/background/lcp/06_other-methods.md` documents the fixed-bound snapshot
  product.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated again.
- The Python demo profile summary and panel test were adjusted for the refreshed
  Boxed and FrictionIndex leader/laggard text.
- `CHANGELOG.md` records the follow-up `BlockedJacobiSolver` optimization.

Focused before/after `BM_LcpCompare` evidence for this slice:

- Standard profile rows: runtime ratios `0.989`, `0.838`, `0.811`, and `0.654`
  for sizes 12, 24, 48, and 96 relative to the pre-change focused run.
- Boxed profile rows: runtime ratios `1.023`, `0.923`, and `0.832` for sizes
  12, 24, and 48; the smallest row is within benchmark noise while larger rows
  improve.
- FrictionIndex profile rows: runtime ratios `0.912`, `0.858`, and `0.904` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.874`; best `0.654`; worst `1.023`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `BlockedJacobi` average ratio improved from `5.66` to `4.60`;
  current leaders are `Tgs`/`Sap`/`Pgs` on scalable rows and `Direct` on tiny
  rows.
- Boxed: `BlockedJacobi` average ratio improved from `3.59` to `3.30`; current
  leader is `Tgs`, with `Pgs`/`Jacobi` close.
- FrictionIndex: `BlockedJacobi` average ratio improved from `20.45` to
  `15.08`; it remains a future optimization target.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_lcp_projection_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_projection_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BlockedJacobi/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/blocked_jacobi_profile_ax_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
pixi run build
pixi run lint
git diff --check
```

Observed results:

- Affected C++ targets rebuilt successfully.
- LCP projection, comparison, generated coverage, and validation solver tests
  passed through CTest: `100% tests passed, 0 tests failed out of 4`.
- Focused BlockedJacobi after-run showed all Standard, Boxed, and
  FrictionIndex profile rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Focused Python LCP panel metadata test passed: `1 passed in 0.60s`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. Fresh high-ratio targets include Standard `Lemke`, `InteriorPoint`,
  `Baraff`, and `ShockPropagation`; Boxed `Sap`, `Admm`, `BGS`, and
  `BoxedSemiSmoothNewton`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, `ShockPropagation`, `Staggering`, and `BGS`.
- Do not push without explicit approval.

## Current Reality - 2026-06-12 BlockedJacobi Singleton Fast Path

After the stop-only hand-off below, work resumed on the selected
benchmark-driven target: `BlockedJacobiSolver`.

Latest implementation slice:

- `dart/math/lcp/projection/blocked_jacobi_solver.cpp` now detects singleton
  blocks with no local `findex` coupling and solves the scalar boxed subproblem
  with a direct projected update.
- The fast path requires a finite positive diagonal and finite effective RHS.
  Non-singleton blocks, local `findex` blocks, non-positive diagonals, and
  invalid scalar data still use the existing `DirectSolver`/`DantzigSolver`
  fallback path.
- `tests/unit/math/lcp/test_lcp_projection_solvers.cpp` now covers singleton
  boxed blocks that clamp against both lower and upper bounds.
- `docs/background/lcp/06_other-methods.md` documents the scalar singleton
  route and corrects the Jacobi block RHS sign in the pseudocode.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated from the optimized solver.
- `python/examples/demos/scenes/lcp_physics.py` and the panel metadata test now
  reflect the refreshed current leader/laggard families.
- `CHANGELOG.md` records the solver optimization.

Focused before/after `BM_LcpCompare` evidence for `BlockedJacobiSolver`:

- Standard profile rows: runtime ratios `0.088`, `0.085`, `0.134`, and `0.169`
  for sizes 12, 24, 48, and 96 relative to the pre-change focused run.
- Boxed profile rows: runtime ratios `0.069`, `0.082`, and `0.093` for sizes
  12, 24, and 48.
- FrictionIndex profile rows: runtime ratios `0.734`, `0.623`, and `0.661` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.274`; best `0.069`; worst `0.734`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `BlockedJacobi` average ratio `5.66`; current leaders are
  `Pgs`/`Sap`/`Tgs` on scalable rows and `Direct` on tiny rows.
- Boxed: `BlockedJacobi` average ratio `3.59`; current leaders are `Pgs`/`Tgs`
  with `Jacobi` close.
- FrictionIndex: `BlockedJacobi` average ratio `20.45`; this remains a future
  optimization target because coupled contact blocks still use the local block
  solve path.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_lcp_projection_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_projection_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BlockedJacobi/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/blocked_jacobi_profile_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
pixi run python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
pixi run build
pixi run lint
git diff --check
```

Observed results:

- Affected C++ targets rebuilt successfully.
- LCP projection, comparison, generated coverage, and validation solver tests
  passed through CTest: `100% tests passed, 0 tests failed out of 4`.
- Focused BlockedJacobi after-run showed all Standard, Boxed, and
  FrictionIndex profile rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Focused Python LCP panel metadata test passed: `1 passed in 0.78s`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. Fresh high-ratio targets include Standard `Lemke`, `InteriorPoint`,
  `Baraff`, and `ShockPropagation`; Boxed `Sap`, `BGS`, `Admm`, and
  `BoxedSemiSmoothNewton`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, `ShockPropagation`, and `BGS`.
- Do not push without explicit approval.

## Current Reality - 2026-06-12 Stop-Only Hand-Off After BlockedJacobi Recon

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. Do not infer that lint, tests, builds, benchmarks,
implementation edits, commits, or pushes were performed after that stop request.
This update is limited to `docs/dev_tasks/lcp_solver_interface_demos/README.md`
and this file.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `3667401ad1f Optimize boxed semi-smooth Newton LCP step`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 30]`.
- Worktree: clean before this docs-only hand-off edit.
- Push was not performed in this stop-only response.

Latest completed local checkpoints:

- `3667401ad1f Optimize boxed semi-smooth Newton LCP step`
- `d27a0232c37 Show LCP performance profiles in py demo`
- `f857a380c20 Regenerate LCP performance profiles`
- `dcf0d835c1b Refresh LCP performance profile tooling`
- `7df9a018c31 Align LCP performance profile metadata`
- `00a58f7153e Align Staggering native capability metadata`
- `1b4e3827618 Report Staggering native support precisely`
- `0e75c5c3985 Validate remaining LCP solver parameters`

Interrupted continuation slice:

- The next benchmark-driven target selected, but not implemented, was
  `BlockedJacobiSolver`, because the refreshed full profile still showed it as
  a high-ratio target across Standard, Boxed, and FrictionIndex rows.
- Context gathered before the stop request included
  `dart/math/lcp/projection/blocked_jacobi_solver.cpp`,
  `dart/math/lcp/projection/blocked_jacobi_solver.hpp`,
  `docs/background/lcp/02_overview.md`,
  `docs/background/lcp/06_other-methods.md`, and related LCP unit-test snippets.
- No BlockedJacobi implementation files were edited.
- No focused BlockedJacobi baseline benchmark was run.
- No lint, test, build, benchmark regeneration, commit, or push was run after
  the stop request.

Likely resume target, if the user explicitly resumes implementation:

- Reinspect the full `BlockedJacobiSolver` update loop before editing.
- The likely low-risk optimization to evaluate is a singleton-block fast path:
  default Standard and Boxed auto partitions can produce singleton blocks, and
  the current solver may be paying full local LCP-solver overhead for those rows.
  A scalar projected Jacobi update could preserve block-Jacobi semantics while
  avoiding repeated local solver setup.
- Measure before changing with a focused run such as:

```bash
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BlockedJacobi/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/blocked_jacobi_profile_before.json
```

Stop here until the user explicitly resumes. Do not push without explicit
approval.

## Current Reality - 2026-06-12 Boxed Semi-Smooth Newton Direct Step

After the Python demo performance-profile surface, work moved into a concrete
benchmark-driven optimization target: `BoxedSemiSmoothNewtonSolver`.

Latest implementation slice:

- `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.cpp` now tries the
  direct semi-smooth Newton system `J dx = -H` before falling back to the
  previous regularized least-squares normal-equation solve.
- Line-search trials now keep friction-index moving-bound updates local to the
  trial state, and only publish those effective bounds when a step is accepted.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated from the optimized solver.
- `docs/background/lcp/05_newton-methods.md` documents the direct
  Jacobian-solve common path and least-squares fallback.
- The Python LCP demo profile summary and panel test were updated to match the
  refreshed current leader/laggard families.
- `CHANGELOG.md` records the solver optimization.

Focused before/after `BM_LcpCompare` evidence for
`BoxedSemiSmoothNewtonSolver`:

- Standard profile rows: mean runtime ratio `0.577`, best `0.395`, worst
  `0.731` relative to the pre-change focused run.
- Boxed profile rows: mean runtime ratio `0.608`, best `0.502`, worst `0.696`.
- FrictionIndex profile rows: mean runtime ratio `0.557`, best `0.451`, worst
  `0.704`.
- All focused after rows reported `contract_ok=1.0`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BoxedSemiSmoothNewton/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/bssn_profile_after.json
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
pixi run python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
```

Observed results:

- `BM_LCP_COMPARE` rebuilt successfully.
- Focused BSSN after-run showed all Standard, Boxed, and FrictionIndex profile
  rows passing contract checks and running at roughly 40-70% of the pre-change
  runtime, depending on row.
- LCP comparison, generated coverage, and validation solver test executables
  all passed through CTest: `100% tests passed, 0 tests failed out of 3`.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Focused Python LCP panel metadata test passed: `1 passed in 0.47s`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.

## Immediate Next Step

Continue benchmark-driven optimization. The freshest full profile still shows
high-ratio targets:

- Standard: `Lemke`, `InteriorPoint`, `Baraff`, `BlockedJacobi`,
  `ShockPropagation`.
- Boxed: `Sap`, `BlockedJacobi`, `BGS`, `Admm`, `BoxedSemiSmoothNewton`.
- FrictionIndex: `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`,
  `Staggering`, `ShockPropagation`.

Pick one solver/problem family, measure it with the same profile tooling,
optimize or retune it, then update benchmark/demo evidence. Do not push without
explicit approval.

Start by inspecting current branch state:

```bash
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

## Current Reality - 2026-06-12 Python Demo Performance Profiles

After the stop-only hand-off and full profile refresh, work resumed on the next
demo-focused gap: making the refreshed performance-profile artifacts visible
from the Python LCP demo.

Latest implementation slice:

- `python/examples/demos/scenes/lcp_physics.py` exposes
  `performance_profile_rows` and `performance_profile_refresh_command` in the
  scene `info` metadata.
- The `lcp_physics` panel has a "Performance profiles" table pointing to the
  checked Standard, Boxed, and FrictionIndex CSV artifacts under
  `docs/background/lcp/figures`.
- The table summarizes current profile row families, problem sizes,
  leader/laggard families, and practical takeaways for Standard, Boxed, and
  FrictionIndex apples-to-apples solver profiles.
- `python/tests/unit/test_py_demo_panels.py` asserts the metadata, artifacts,
  refresh command, leader/laggard text, and panel table registration.
- `python/examples/demos/README.md` documents the checked profile summary
  surface and refresh command.
- `CHANGELOG.md` records the demo-profile metadata change.

Verification completed so far:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run lint
git diff --check
```

Observed results:

- Focused LCP panel metadata test passed: `1 passed in 0.43s`.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

## Immediate Next Step

Continue the broad objective with benchmark-driven optimization or a deeper
solver-interface audit. The freshest actionable performance targets are the
high-ratio rows surfaced in the demo summary: Standard
Lemke/InteriorPoint/Baraff/BlockedJacobi, Boxed
SAP/BlockedJacobi/BoxedSemiSmoothNewton, and FrictionIndex
BoxedSemiSmoothNewton/BlockedJacobi/ShockPropagation. Pick one concrete
solver/problem family, measure it with the checked profile tooling, optimize or
retune it, then update the matching benchmark/demo evidence. Do not push
without explicit approval.

Start by inspecting current branch state:

```bash
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

## Current Reality - 2026-06-12 Stop-Only Hand-Off After Full Profile Refresh

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. After that instruction, do not infer that lint, tests,
builds, benchmark listing, benchmark regeneration, implementation edits,
commits, or pushes were performed. This stop-only update is limited to
`docs/dev_tasks/lcp_solver_interface_demos/README.md` and this file.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `f857a380c20 Regenerate LCP performance profiles`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 28]`.
- Worktree: clean before this docs-only hand-off edit.
- Remote tracking ref still pointed at
  `737b9c95c11 Document final LCP handoff checkpoint` when inspected by
  `git log --decorate`.
- Push was not performed in this stop-only response.

Latest completed local checkpoints:

- `f857a380c20 Regenerate LCP performance profiles`
- `dcf0d835c1b Refresh LCP performance profile tooling`
- `7df9a018c31 Align LCP performance profile metadata`
- `00a58f7153e Align Staggering native capability metadata`
- `1b4e3827618 Report Staggering native support precisely`
- `0e75c5c3985 Validate remaining LCP solver parameters`
- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`

Interrupted/unstarted continuation slice:

- No implementation files were edited for the next slice after the stop
  request.
- No lint, tests, builds, benchmark listing, benchmark regeneration, commit, or
  push was run after the stop request.
- The session had selected, but not implemented, a likely next slice: expose
  the refreshed performance-profile leaders and laggards in the Python LCP demo
  metadata/panel so users can see solver pros/cons without manually parsing
  benchmark output.
- Context gathered before the stop request, from the current full profile cache,
  suggested the next explanatory/demo metadata should call out slow/high-ratio
  families such as Standard `Lemke`, `InteriorPoint`, `Baraff`, and
  `BlockedJacobi`; Boxed `Sap`, `BlockedJacobi`, and
  `BoxedSemiSmoothNewton`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, and `ShockPropagation`. Treat this as hand-off context, not
  a completed product change.

## Stop-Only Immediate Next Step

Do nothing until the user explicitly resumes implementation. If resumed, first
inspect the current branch state:

```bash
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

Then choose one concrete solver/interface/demo/performance gap. The most recent
unstarted slice is the Python LCP demo performance-profile summary described
above. Do not push without explicit approval.

## Current Reality - 2026-06-12 Full Performance Profile Refresh

After checkpoint `dcf0d835c1b Refresh LCP performance profile tooling`, work
continued into the next benchmark-focused gap: regenerating the checked
performance-profile CSV artifacts from current benchmark rows.

Latest implementation slice:

- A first full script run exposed that the profile script was launching every
  benchmark in `BM_LCP_COMPARE`, including many unrelated stress/sweep rows
  ignored by the profile parser; that unfiltered run timed out after 600s.
- `scripts/lcp_performance_profile.py` now defaults benchmark execution to the
  profile row family `BM_LcpCompare/`.
- The script now exposes `--benchmark-timeout` and reports timeout failures with
  an actionable message instead of a raw Python traceback.
- `docs/background/lcp/figures/performance_profile_standard.csv`,
  `docs/background/lcp/figures/performance_profile_boxed.csv`, and
  `docs/background/lcp/figures/performance_profile_frictionindex.csv` were
  regenerated from a current full `BM_LcpCompare/` JSON packet.
- The generated CSVs now have 23 Standard solver columns, 15 Boxed solver
  columns, and 16 FrictionIndex solver columns, matching native manifest
  coverage.

Verification completed so far:

```bash
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
pixi run python scripts/check_lcp_solver_roster.py
pixi run python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows, header[:4], header[-3:])
PY
```

Observed results:

- The current full profile row family completed and cached results to
  `build/lcp_profile_full.json`.
- Cached replay completed and wrote check copies under
  `build/lcp_profile_full_check`.
- LCP solver roster check passed:
  `24 solvers, 23 standard, 15 boxed, 16 findex`.
- Generated CSV shape check reported:
  - `performance_profile_standard.csv`: 23 solver columns, 200 rows.
  - `performance_profile_boxed.csv`: 15 solver columns, 200 rows.
  - `performance_profile_frictionindex.csv`: 16 solver columns, 200 rows.

## Immediate Next Step

After this checkpoint, continue the broad objective with the next concrete
solver/interface/demo/performance gap. The refreshed profiles now make the next
benchmark-driven optimization target visible: inspect the slow/high-ratio
solver families and decide whether to improve implementation performance, tune
benchmark parameters, or improve the py-demo explanations for solver pros/cons.
Do not push without explicit approval.

Start by inspecting current branch state:

```bash
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

## Current Reality - 2026-06-12 Performance Profile Regeneration Schema

After the stop-only hand-off, work resumed on the broad DART 7 LCP objective.
The latest slice fixed the profile-refresh path itself: the checked CSV headers
already used native solver metadata, but the regeneration script still parsed
the old benchmark naming scheme and the `BM_LCP_COMPARE` benchmark target still
had stale native support count assertions.

Latest implementation slice:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now asserts 24 total solvers,
  23 standard-native solvers, 15 boxed-native solvers, and 16
  friction-index-native solvers.
- `scripts/lcp_performance_profile.py` now parses current benchmark names of
  the form `BM_LcpCompare/<problem-family>/<solver>/<problem-size>`.
- Historical cached JSON with
  `BM_LcpCompare_<solver>_<problem-family>/<problem-size>` names still parses.
- Full profile generation now fails when benchmark JSON is missing any native
  solver for Standard, Boxed, or FrictionIndex. Use `--allow-partial` only for
  deliberate smoke packets.
- `--benchmark-filter` and `--benchmark-min-time` can now be passed through to
  Google Benchmark for focused profile smoke runs.
- Full current performance-profile artifacts were not regenerated in this
  slice.

Verification completed so far:

```bash
pixi run python - <<'PY'
from scripts.lcp_performance_profile import (
    check_native_profile_coverage,
    load_native_support_by_category,
    parse_benchmark_results,
    parse_benchmark_name,
)

assert parse_benchmark_name('BM_LcpCompare/Standard/BGS/12') == ('Standard', 'BGS', 12)
assert parse_benchmark_name('BM_LcpCompare/Boxed/NNCG/24') == ('Boxed', 'NNCG', 24)
assert parse_benchmark_name('BM_LcpCompare_FischerBurmeisterNewton_Standard/12') == ('Standard', 'FischerBurmeisterNewton', 12)
assert parse_benchmark_name('BM_LcpCompare_Dantzig_Scaled/12/0') is None
parsed = parse_benchmark_results({
    'benchmarks': [
        {'name': 'BM_LcpCompare/Standard/Bgs/12', 'cpu_time': 2.0, 'contract_ok': 1.0},
        {'name': 'BM_LcpCompare/Boxed/Nncg/24', 'cpu_time': 3.0, 'contract_ok': 1.0},
        {'name': 'BM_LcpCompare/FrictionIndex/Mprgp/4', 'cpu_time': 4.0, 'contract_ok': 1.0},
        {'name': 'BM_LcpCompare/Standard/BGS/12_mean', 'run_type': 'aggregate'},
    ]
})
assert ('BGS', 12) in parsed['Standard']
assert ('NNCG', 24) in parsed['Boxed']
assert ('MPRGP', 4) in parsed['FrictionIndex']
try:
    check_native_profile_coverage(parsed, load_native_support_by_category())
except RuntimeError as exc:
    assert 'missing native solvers' in str(exc)
else:
    raise AssertionError('expected missing coverage failure')
check_native_profile_coverage(parsed, load_native_support_by_category(), allow_partial=True)
print('profile parser smoke passed')
PY
cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"
pixi run python scripts/lcp_performance_profile.py --run \
  --benchmark-filter='BM_LcpCompare/(Standard|Boxed)/(Dantzig|Pgs)/12$' \
  --benchmark-min-time=0.001 \
  --cache build/lcp_profile_smoke.json \
  --output build/lcp_profile_smoke \
  --allow-partial
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_smoke.json \
  --output build/lcp_profile_smoke_failcheck
pixi run python - <<'PY'
import subprocess
from scripts.lcp_performance_profile import (
    check_native_profile_coverage,
    load_native_support_by_category,
    parse_benchmark_name,
)

completed = subprocess.run(
    [
        'build/default/cpp/Release/bin/BM_LCP_COMPARE',
        '--benchmark_list_tests=true',
        '--benchmark_filter=BM_LcpCompare/',
    ],
    check=True,
    capture_output=True,
    text=True,
)
results = {}
for name in completed.stdout.splitlines():
    parsed = parse_benchmark_name(name.strip())
    if parsed is None:
        continue
    category, solver, problem_size = parsed
    results.setdefault(category, {})[(solver, problem_size)] = {
        'time_ns': 1.0,
        'contract_ok': 1.0,
    }
check_native_profile_coverage(results, load_native_support_by_category())
counts = {category: len({solver for solver, _ in rows}) for category, rows in results.items()}
print(counts)
PY
pixi run python scripts/check_lcp_solver_roster.py
git diff --check
pixi run lint
```

Observed results:

- Synthetic parser smoke passed and proved strict coverage fails on incomplete
  benchmark JSON unless `--allow-partial` is set.
- `BM_LCP_COMPARE` rebuilt successfully from current source.
- Focused real benchmark smoke produced partial Standard and Boxed profile CSVs
  under `build/lcp_profile_smoke`.
- Strict mode rejected that partial smoke cache, as expected, with missing
  native solver coverage diagnostics.
- The rebuilt benchmark registration list covered exactly
  `{'Standard': 23, 'Boxed': 15, 'FrictionIndex': 16}` native solvers.
- LCP solver roster check passed:
  `24 solvers, 23 standard, 15 boxed, 16 findex`.
- `git diff --check` passed.
- `pixi run lint` passed, including the LCP roster gate.

## Immediate Next Step

After this checkpoint, continue the broad objective with another concrete
solver/interface/demo/performance gap. A likely next benchmark-focused slice is
to run the full unfiltered performance-profile generation from the rebuilt
benchmark target, review the current measurements, and update the checked
profile artifacts if the run is representative enough. Do not push without
explicit approval.

Start by inspecting current branch state:

```bash
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

## Current Reality - 2026-06-12 Stop-Only Hand-Off After Performance Profile Metadata

The user explicitly stopped further work again and requested only hand-off docs,
with no further verification. After that instruction, do not infer that lint,
tests, builds, benchmark listing, benchmark regeneration, commits, pushes, or
implementation edits were performed. This stop-only update is limited to
`docs/dev_tasks/lcp_solver_interface_demos/README.md` and this file.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `7df9a018c31 Align LCP performance profile metadata`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 26]`.
- Worktree: clean before this docs-only hand-off edit.
- No PR was associated with this branch when the already-running GitHub status
  command completed.
- Push was not performed in this stop-only response.

Latest completed local checkpoints:

- `7df9a018c31 Align LCP performance profile metadata`
- `00a58f7153e Align Staggering native capability metadata`
- `1b4e3827618 Report Staggering native support precisely`
- `0e75c5c3985 Validate remaining LCP solver parameters`
- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`

Interrupted continuation slice:

- No implementation files were edited after the stop request.
- No lint, tests, builds, benchmark listing, benchmark regeneration, commit, or
  push was run after the stop request.
- The session had only reconstructed current context and closed out an
  already-running `gh pr status` command. That command reported no PR
  associated with `feature/lcp-solver-interface-demos`.
- Do not treat the broad DART 7 LCP solver/interface/demo objective as
  complete. The latest completed slice only aligned performance-profile
  tooling and checked CSV headers with native solver support metadata.

## Stop-Only Immediate Next Step

Do nothing until the user explicitly resumes implementation. If resumed, first
inspect the current branch state:

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- this file

Continue only after selecting a concrete next solver/interface/demo/performance
gap. A likely next gap remains regenerating performance-profile artifacts from
current benchmark results rather than synthesizing missing solver columns from
older cached data. Do not push without explicit approval.

## Current Reality - 2026-06-12 LCP Performance Profile Native Support

After the stop-only hand-off and the Staggering capability-metadata checkpoint,
work resumed on the broad DART 7 LCP objective. The benchmark registration
audit found that the main benchmark registration paths already instantiate
solvers and call `supportsProblem(problem)` before registering concrete rows,
but the background performance-profile tooling and checked CSV artifacts still
allowed stale non-native solver columns.

Latest implementation slice:

- `scripts/lcp_performance_profile.py` now loads the checked C++ solver
  manifest and filters each Standard, Boxed, and FrictionIndex profile to
  solvers that are native for that problem form.
- The performance-profile parser canonicalizes acronym solver names such as
  `BGS`, `NNCG`, and `MPRGP` to the manifest spelling before computing profile
  ratios.
- `scripts/check_lcp_solver_roster.py` now checks the performance-profile CSV
  headers against native manifest support, catching stale columns such as
  delegated-only Staggering in standard or boxed profiles.
- `docs/background/lcp/figures/performance_profile_standard.csv` and
  `docs/background/lcp/figures/performance_profile_boxed.csv` no longer list
  Staggering as a native profile column. Existing cached measurements were not
  rerun in this slice, so missing newly supported profile columns remain a
  future benchmark-refresh task rather than synthesized data.
- `CHANGELOG.md` and the dev-task README describe the profile
  tooling/header alignment.

Verification completed so far:

```bash
pixi run python scripts/check_lcp_solver_roster.py
pixi run python - <<'PY'
from scripts.lcp_performance_profile import (
    compute_performance_ratios,
    parse_benchmark_results,
)

results = {
    "Standard": {
        ("NativeSolver", 2): {"time_ns": 1.0, "contract_ok": 1.0},
        ("DelegatedOnlySolver", 2): {"time_ns": 0.1, "contract_ok": 1.0},
    }
}
ratios, solvers, problems = compute_performance_ratios(
    results, "Standard", {"NativeSolver"}
)
assert solvers == ["NativeSolver"]
assert problems == [2]
assert ratios["NativeSolver"] == [1.0]
parsed = parse_benchmark_results({
    "benchmarks": [
        {"name": "BM_LcpCompare_Bgs_Standard/2", "cpu_time": 1.0, "contract_ok": 1.0},
        {"name": "BM_LcpCompare_Nncg_Boxed/2", "cpu_time": 1.0, "contract_ok": 1.0},
        {"name": "BM_LcpCompare_Mprgp_Standard/2", "cpu_time": 1.0, "contract_ok": 1.0},
    ]
})
assert ("BGS", 2) in parsed["Standard"]
assert ("NNCG", 2) in parsed["Boxed"]
assert ("MPRGP", 2) in parsed["Standard"]
print("performance profile native-support filter passed")
PY
pixi run lint
```

Observed results:

- LCP solver roster check passed:
  `24 solvers, 23 standard, 15 boxed, 16 findex`.
- Synthetic profile parsing/filter check passed.
- `pixi run lint` passed, including the updated LCP solver roster gate.

## Immediate Next Step

After this checkpoint, continue the broad objective with another concrete
solver/interface/demo/performance gap. A later benchmark-refresh slice should
regenerate the performance-profile artifacts from current benchmark results
rather than synthesizing missing solver columns from older cached data.

Start by inspecting current branch state:

```bash
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

## Current Reality - 2026-06-12 Stop-Only Hand-Off After Staggering Capability Metadata

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. After that instruction, do not infer that lint, tests,
builds, benchmark listing, commits, pushes, or implementation edits were
performed. This stop-only update is limited to
`docs/dev_tasks/lcp_solver_interface_demos/README.md` and this file.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `00a58f7153e Align Staggering native capability metadata`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 25]`.
- Worktree: clean before this docs-only hand-off edit.
- No PR was associated with this branch when checked earlier in the session.
- Push was not performed in this stop-only response.

Latest completed local checkpoints:

- `00a58f7153e Align Staggering native capability metadata`
- `1b4e3827618 Report Staggering native support precisely`
- `0e75c5c3985 Validate remaining LCP solver parameters`
- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`

Interrupted continuation slice:

- No implementation files were edited in the interrupted continuation slice.
- The session was only reconstructing context and choosing the next bounded
  gap after the Staggering capability-method checkpoint.
- The likely next audit target was benchmark/demo registration and metadata
  after Staggering stopped advertising native standard/boxed support. Candidate
  files to inspect on explicit resume include
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`,
  `python/examples/demos/scenes/lcp_physics.py`, and
  `python/tests/unit/test_py_demo_panels.py`.
- Do not treat the broad DART 7 LCP solver/interface/demo objective as
  complete. The latest completed slice only aligned Staggering native
  capability metadata and related tests.

## Stop-Only Immediate Next Step

Do nothing until the user explicitly resumes implementation. If resumed, first
inspect the current branch state:

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- this file

Continue only after selecting a concrete next solver/interface/demo/performance
gap. Do not push without explicit approval.

## Current Reality — 2026-06-12 Staggering Form-Level Native Capabilities

After checkpoint `1b4e3827618 Report Staggering native support precisely`, the
follow-up slice aligned Staggering's form-level native capability methods with
the per-problem predicate and demo metadata.

Latest implementation slice:

- `StaggeringSolver::supportsStandardLcp()` and
  `StaggeringSolver::supportsBoxedLcp()` now return false.
- `StaggeringSolver::supportsFrictionIndex()` remains true, while
  `supportsProblem(problem)` still reports native support only for
  friction-index packets containing both normal and friction rows.
- The C++ manifest, comparison-harness coverage table, Python demo metadata,
  dartpy tests, py-demo panel tests, and roster checker now agree on 24 total
  solvers, 23 standard-native solvers, 15 boxed-native solvers, and 16
  friction-index-native solvers.
- Standard and boxed no-friction packets remain solvable through fallback
  delegation.
- `CHANGELOG.md` and the dev-task README describe the capability-method
  alignment.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke \
  --gtest_filter='AllSolversSmokeTest.ManifestMatchesConstructedSolverMetadata:AllSolversSmokeTest.SolverCapabilityPredicatesClassifyProblemForms:AllSolversSmokeTest.SolverCapabilityPredicatesUseDefaultToleranceForNearStandardForm:AllSolversSmokeTest.StaggeringReportsOnlyFrictionBlockProblemsAsNative'
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_comparison_harness \
  --gtest_filter='LcpComparisonHarness.ManifestMatchesFixtureCoverage:LcpComparisonHarness.StaggeringOnStandardAndBoxedFixtures:LcpComparisonHarness.StaggeringOnFrictionIndexFixtures'
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
pixi run lint
```

Observed results:

- Focused C++ all-solvers smoke coverage passed 4/4 tests.
- Focused C++ comparison-harness coverage passed 3/3 tests.
- `python/tests/unit/math/test_lcp.py`: 78 passed.
- `python/tests/unit/test_py_demo_panels.py`: 43 passed.
- LCP solver roster check passed:
  `24 solvers, 23 standard, 15 boxed, 16 findex`.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `pixi run lint` passed, including the updated LCP solver roster gate.

## Immediate Next Step

After the checkpoint commit, continue the broad DART 7 LCP objective with the
next concrete solver/interface/demo/performance gap. Do not mark the overall
goal complete from this focused capability-method slice, and do not push
without explicit approval.

## Current Reality — 2026-06-12 Staggering Native Support Reporting

After the stop-only hand-off, work resumed on the broad LCP solver/interface
goal. The active slice tightens `StaggeringSolver::supportsProblem()` so native
support reporting matches the actual Staggering friction-block route rather
than its standard/boxed Dantzig fallback routes.

Latest implementation slice:

- `StaggeringSolver` now reports native support only for friction-index packets
  containing both normal and friction rows.
- Standard, boxed no-friction, near-standard, and large standard packets report
  false for Staggering native support, while `solve()` can still succeed
  through fallback delegation.
- C++ smoke tests cover the updated support predicates and an explicit
  standard-packet fallback solve.
- dartpy LCP tests mirror those expectations and keep a fallback solve check.
- `CHANGELOG.md` and the dev-task README describe the native-support-reporting
  behavior.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke \
  --gtest_filter='AllSolversSmokeTest.SolverCapabilityPredicatesClassifyProblemForms:AllSolversSmokeTest.SolverCapabilityPredicatesUseDefaultToleranceForNearStandardForm:AllSolversSmokeTest.StaggeringReportsOnlyFrictionBlockProblemsAsNative:AllSolversSmokeTest.NearSingularStandardProblemProducesExpectedIterates'
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
pixi run lint
```

Observed results:

- The C++ smoke target and dartpy rebuilt successfully.
- Focused C++ smoke coverage passed 4/4 tests.
- `python/tests/unit/math/test_lcp.py`: 78 passed.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `pixi run lint` passed, including the LCP solver roster gate.

## Previous Immediate Next Step

After the checkpoint commit, continue the broad DART 7 LCP objective with the
next concrete solver/interface/demo/performance gap. Do not mark the overall
goal complete from this focused support-reporting slice, and do not push
without explicit approval.

## Current Reality — 2026-06-12 Remaining Parameter Validation

The user resumed the broad LCP solver/interface/demo goal after the stop-only
hand-off. The remaining parameter-validation audit has been implemented and
targeted verification has passed locally.

Latest implementation slice:

- `PgsSolver`, `ApgdSolver`, and `TgsSolver` reject invalid
  `epsilonForDivision`; APGD also rejects negative `restartCheckInterval`
  while keeping `0` valid as the every-iteration restart-check mode.
- `MinimumMapNewtonSolver`, `FischerBurmeisterNewtonSolver`, and
  `PenalizedFischerBurmeisterNewtonSolver` reject invalid exposed line-search,
  gradient-descent warm-start, PGS warm-start, smoothing, and penalty
  parameters before numerical iteration.
- `InteriorPointSolver` rejects invalid `sigma` and `stepScale` instead of
  silently clamping them.
- C++ validation tests and dartpy LCP tests cover invalid custom parameter
  structs/objects across the affected solvers.
- Existing Newton warm-start tests now use valid line-search parameters, and
  invalid line-search counts are tested as `InvalidProblem` diagnostics.
- `CHANGELOG.md` and the dev-task README describe the new validation behavior.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
           UNIT_math_lcp_math_lcp_lcp_newton_solvers \
           dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --gtest_filter='PgsSolverCoverage.RejectsInvalidDivisionEpsilon:ApgdSolverCoverage.RejectsInvalidParameters:TgsSolverCoverage.RejectsInvalidDivisionEpsilon:MinimumMapNewtonCoverage.RejectsInvalidParameters:FischerBurmeisterNewtonCoverage.RejectsInvalidParameters:PenalizedFischerBurmeisterNewtonCoverage.RejectsInvalidParameters:InteriorPointSolverCoverage.RejectsInvalidParameters'
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_newton_solvers \
  --gtest_filter='MinimumMapNewtonSolver.RejectsInvalidLineSearchStepCount:MinimumMapNewtonSolver.GradientDescentWarmStartReducesMerit:MinimumMapNewtonSolver.PgsWarmStartReducesMerit:FischerBurmeisterNewtonSolver.RejectsInvalidLineSearchStepCount:FischerBurmeisterNewtonSolver.GradientDescentWarmStartReducesMerit:FischerBurmeisterNewtonSolver.PgsWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.RejectsInvalidLineSearchStepCount:PenalizedFischerBurmeisterNewtonSolver.GradientDescentWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.PgsWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.RejectsInvalidLambda'
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
pixi run lint
```

Observed results:

- Focused C++ validation executable: 7/7 tests passed.
- Focused C++ Newton executable: 10/10 tests passed.
- `python/tests/unit/math/test_lcp.py`: 77 passed.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `pixi run lint` passed, including the LCP solver roster gate.

Immediate next step:

- After the checkpoint, continue the broad DART 7 LCP objective with the next
  concrete solver/interface/demo/performance gap. Do not mark the overall goal
  complete from this focused validation slice.

## Current Reality — 2026-06-12 Stop-Only Hand-Off

The user explicitly requested a full stop and asked only to ensure the
hand-off docs, with no further verification. After that instruction, do not
infer that lint, tests, builds, commits, pushes, or implementation edits were
performed; this hand-off only updates
`docs/dev_tasks/lcp_solver_interface_demos/README.md` and this file.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `e3535141c5d Show all LCP solver parameters in py demo`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 22]`.
- Worktree: clean before this docs-only hand-off edit.
- Stash list: empty.
- No PR was associated with this branch when checked earlier in the session.

Latest completed local checkpoints:

- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`
- `748cef40ea0 Validate advanced LCP solver parameters`
- `06ac009b27c Show LCP solver parameters in py demo`
- `8cd98e2e553 Expose advanced LCP parameters in dartpy`

## Previous Stop-Only Immediate Next Step

Do nothing unless the user explicitly resumes implementation. If resumed, first
inspect the current branch state:

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- this file

## Interrupted Audit Slice

No implementation files were edited in the interrupted slice. The current work
was an audit for remaining solve-time parameter validation gaps in LCP solvers
whose parameter surfaces are now exposed through C++ and dartpy.

Audit findings to resume from:

- `dart/math/lcp/projection/pgs_solver.cpp`: validate
  `PgsSolverParameters::epsilonForDivision` before iteration; reject
  non-finite or non-positive values.
- `dart/math/lcp/projection/apgd_solver.cpp`: validate
  `ApgdSolverParameters::epsilonForDivision > 0` and
  `restartCheckInterval >= 0`; keep `0` valid as the every-iteration restart
  check mode.
- `dart/math/lcp/projection/tgs_solver.cpp`: validate
  `TgsSolverParameters::epsilonForDivision` before iteration; reject
  non-finite or non-positive values.
- `dart/math/lcp/newton/minimum_map_newton_solver.cpp`: validate line-search
  and warm-start parameters before iteration. Suggested checks:
  `maxLineSearchSteps > 0`, `stepReduction` finite in `(0, 1)`,
  `sufficientDecrease` finite in `[0, 1)`, `minStep > 0`, non-negative
  gradient-descent and PGS warm-start iteration counts, positive
  gradient-descent line-search steps, finite gradient-descent factors in the
  same ranges, and finite `pgsWarmStartRelaxation` in `(0, 2]`.
- `dart/math/lcp/newton/fischer_burmeister_newton_solver.cpp`: add the same
  Newton line-search and warm-start validation, plus positive finite
  `smoothingEpsilon`.
- `dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.cpp`: keep
  existing `lambda` validation in `(0, 1]`, and add the same Newton
  line-search, warm-start, and positive finite `smoothingEpsilon` validation.
- `dart/math/lcp/other/interior_point_solver.cpp`: stop silently clamping
  user-facing `sigma` and `stepScale`; validate finite `sigma` in `(0, 1)`
  and finite `stepScale` in `(0, 1]`, then use the validated values directly.

Test cautions for the next session:

- `tests/unit/math/lcp/test_lcp_newton_solvers.cpp` has coverage that may set
  `maxLineSearchSteps = 0` to drive numerical failure branches. If validation
  changes this behavior, update the expectations to `InvalidProblem` with a
  parameter-specific message.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` has
  interior-point coverage that sets `sigma = 1.5` and `stepScale = 1.5`; this
  likely reflects the old silent-clamping behavior.
- APGD tests that use `restartCheckInterval = 0` should stay valid.

Potential focused verification, only if implementation is explicitly resumed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
           dartpy \
  --parallel "$JOBS"
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run lint
```

Do not push or commit without a fresh explicit user instruction and the normal
DART pre-commit checks.

## Current Reality — 2026-06-11 Full Parameter Metadata In Py-Demo

Checkpoint `b60e20a8dc8 Expose LCP solver parameters in dartpy` is complete
locally. The next in-progress slice updates the Python LCP demo's
advanced-parameter table so it no longer reports only ADMM, SAP, and boxed
semi-smooth Newton.

Latest implementation slice:

- Generated `advanced_solver_parameter_rows` from the full dartpy parameter
  surface for projection, Newton, MPRGP, interior-point, shock propagation,
  ADMM, SAP, and boxed semi-smooth Newton solvers.
- Kept rows linked to representative benchmark filters from the LCP comparison
  benchmark metadata.
- Updated `python/tests/unit/test_py_demo_panels.py`, `CHANGELOG.md`, and this
  dev-task state.

Verification completed:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 passed.
- `pixi run lint` passed, including the LCP solver roster gate.

Immediate next step:

- Commit the py-demo metadata checkpoint.

## Current Reality — 2026-06-11 Full dartpy Parameter Surface

Checkpoint `748cef40ea0 Validate advanced LCP solver parameters` is complete
locally. The next in-progress slice expands dartpy from the three advanced
solver parameter objects to the full set of C++ LCP solvers that already expose
`setParameters()` / `getParameters()`.

Latest implementation slice:

- Added parameter classes and solver `parameters` properties for PGS, symmetric
  PSOR, Jacobi, red-black Gauss-Seidel, blocked Jacobi, BGS, NNCG, subspace
  minimization, APGD, TGS, minimum-map Newton, Fischer-Burmeister Newton,
  penalized Fischer-Burmeister Newton, interior-point, MPRGP, and shock
  propagation.
- Updated `python/stubs/dartpy/math.pyi`, `python/tests/unit/math/test_lcp.py`,
  `CHANGELOG.md`, and this dev-task state.
- Used `snake_case` Python field names; the penalized Fischer-Burmeister
  `lambda` field is exposed as `lambda_`.

Verification completed:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run build-py-dev
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run lint
```

Observed results:

- `pixi run build-py-dev` rebuilt and linked `dartpy`.
- `python/tests/unit/math/test_lcp.py`: 70 passed.
- `pixi run lint` passed, including the LCP solver roster gate.

Immediate next step:

- Commit the full dartpy parameter-surface slice.
- Then continue with the next concrete solver/interface/demo/performance gap;
  the broad LCP objective is still open.

## Current Reality — 2026-06-11 Advanced Parameter Validation Resumed

The user resumed the broad LCP solver/interface/demo goal after the previous
stop-only hand-off. The interrupted advanced solver parameter validation slice
has now been rebuilt and verified locally.

Latest implementation slice:

- `AdmmSolver` returns `InvalidProblem` before iteration for invalid
  `rhoInit`, `muProx`, or `adaptiveRhoTolerance` values.
- `SapSolver` returns `InvalidProblem` before iteration for invalid
  `regularization`, Armijo, backtracking, or line-search iteration settings.
- `BoxedSemiSmoothNewtonSolver` returns `InvalidProblem` before iteration for
  invalid line-search, sufficient-decrease, minimum-step, or Jacobian
  regularization settings.
- C++ coverage in the LCP smoke and validation tests exercises invalid custom
  parameter structs.
- dartpy coverage exercises invalid Python parameter objects through the solver
  `parameters` properties.
- `CHANGELOG.md` and the dev-task README describe the new validation behavior.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --gtest_filter='BoxedSemiSmoothNewtonSolverCoverage.*:AdmmSolverCoverage.*'
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
```

Observed results:

- Focused validation executable passed 4 tests after an explicit rebuild.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `python/tests/unit/math/test_lcp.py`: 69 passed.

Immediate next step:

- Run `pixi run lint`, then commit the validation slice if lint is clean.
- After that checkpoint, continue the broad DART 7 LCP audit with the next
  concrete solver/interface/demo/performance gap. Do not mark the overall goal
  complete from this focused validation slice.

## Current Reality — 2026-06-11 Stop-Only Hand-Off

The user explicitly requested a full stop and asked only to ensure the
hand-off docs, with no further verification. After that instruction, do not
infer that tests, lint, commits, or pushes were performed; this hand-off only
updates `docs/dev_tasks/lcp_solver_interface_demos/README.md` and this file.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `06ac009b27c Show LCP solver parameters in py demo`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 19]`.
- Worktree had uncommitted implementation edits in:
  - `CHANGELOG.md`
  - `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.cpp`
  - `dart/math/lcp/other/admm_solver.cpp`
  - `dart/math/lcp/other/sap_solver.cpp`
  - `python/tests/unit/math/test_lcp.py`
  - `tests/unit/math/lcp/test_all_solvers_smoke.cpp`
  - `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`

## Immediate Next Step

Do nothing unless the user explicitly resumes implementation. If resumed, first
inspect the current worktree and the uncommitted validation slice:

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

Then inspect:

- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- this file
- the uncommitted files listed above

## Interrupted Implementation Slice

The interrupted, uncommitted slice adds solve-time validation for advanced
solver parameter structs:

- `AdmmSolver`: rejects non-positive `rhoInit`, negative `muProx`, and
  `adaptiveRhoTolerance <= 1` or non-finite values with `InvalidProblem`.
- `SapSolver`: rejects non-positive `regularization`, invalid Armijo and
  backtracking factors outside `(0, 1)`, non-positive line-search iteration
  counts, and non-finite values with `InvalidProblem`.
- `BoxedSemiSmoothNewtonSolver`: rejects invalid line-search steps,
  `stepReduction`, `sufficientDecrease`, `minStep`, and
  `jacobianRegularization` values with `InvalidProblem`.
- C++ and dartpy tests were added or adjusted to assert invalid advanced
  parameters are rejected before solver iteration.
- `CHANGELOG.md` was updated for the new validation behavior.

Verification that happened before the stop-only instruction:

- Explicit build of `UNIT_math_lcp_math_lcp_all_solvers_smoke` plus `dartpy`
  succeeded after fixing a const-qualification issue in the smoke test.
- `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke`
  passed 17 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest
python/tests/unit/math/test_lcp.py -q` passed 69 tests.
- `pixi run test-lcpsolver` failed only
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`; the failure output still
  appeared to use the old test name and old expectations after the source was
  edited, so a fresh session should first rebuild that exact target and rerun
  the focused executable before drawing conclusions.

Suggested first verification only if implementation is resumed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --gtest_filter='BoxedSemiSmoothNewtonSolverCoverage.*:AdmmSolverCoverage.*'
```

Do not push or commit without a fresh explicit user instruction and the normal
DART pre-commit checks.

This stop-only state is historical after the current continuation.

## Current Reality — 2026-06-11 Advanced Solver Parameters

The user later explicitly resumed the broad LCP solver/interface/demo goal, so
the prior stop-only hand-off is now historical. This continuation resumes from
`17a994e3772 Reject negative LCP friction coefficients` on
`feature/lcp-solver-interface-demos` and adds a bounded dartpy interface slice.

Current implementation slice:

- Exposes `AdmmSolverParameters`, `SapSolverParameters`, and
  `BoxedSemiSmoothNewtonSolverParameters` through dartpy with DART 7 snake_case
  field names.
- Adds `parameters` properties to dartpy `AdmmSolver`, `SapSolver`, and
  `BoxedSemiSmoothNewtonSolver`, forwarding to the existing C++
  `setParameters()` / `getParameters()` APIs.
- Updates the manual dartpy type stubs and Python LCP tests so Python demos can
  tune the same advanced boxed and friction-index solver knobs used by C++
  benchmark sweeps such as `BM_LcpAdmmRhoSweep`,
  `BM_LcpSapRegularizationSweep`, and
  `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep`.
- Adds a Python LCP demo parameter table that reports those advanced solver
  knobs, defaults, and matching benchmark sweep filters in `setup.info` and
  the GUI panel.
- Updates `CHANGELOG.md`, focused py-demo tests, the LCP roster lint guard, the
  generated API boundary inventory, and these dev-task docs.

Verification completed for this slice:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run build-py-dev
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
pixi run lint
```

Observed results:

- `pixi run build-py-dev` rebuilt and linked `dartpy`.
- `python/tests/unit/math/test_lcp.py`: `66 passed`.
- `python/tests/unit/test_py_demo_panels.py`: `43 passed`.
- LCP solver roster check: `24 solvers, 24 standard, 16 boxed/findex`.
- `pixi run lint` passed.

Observed repository state before this slice:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD:
  `17a994e3772 Reject negative LCP friction coefficients`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 17]`.
- Recent local checkpoints:
  - `17a994e3772 Reject negative LCP friction coefficients`
  - `e3353bf04b7 Expose LCP solver sweep metadata`
  - `46700198d80 Expose LCP scale benchmark metadata`
  - `197b55c335a Use concrete LCP contact registration gates`
- No PR was associated with this branch when checked before the stop-only
  hand-off.

Fresh-session resume sequence:

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git log --oneline --decorate --max-count=15
```

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- this file

Do not treat this slice as completion of the broad LCP solver/interface/demo
objective. Continue one bounded DART 7 LCP interface, solver completeness,
demo, benchmark, or performance gap at a time.

## Latest Completed Implementation — 2026-06-11 Friction Coefficient Validation

The latest committed implementation slice:

- Rejects friction-index rows with negative stored upper coefficients `hi[i]`
  in shared problem validation. DART's public LCP docs define this slot as the
  non-negative `+mu` coefficient for effective tangent bounds.
- Makes `computeEffectiveBounds(...)` use `hi[i]` directly after validation
  instead of silently accepting a negative coefficient through `abs(hi[i])`.
- Makes `LcpProblem::hasFrictionIndex()` reject the same invalid metadata so
  classification, `isValid()`, `supportsProblem(problem)`, and solver dispatch
  agree.
- Adds C++ coverage in `test_lcp_types.cpp`,
  `test_lcp_validation_and_solvers.cpp`, and generated invalid-problem solver
  coverage.
- Adds dartpy coverage in `python/tests/unit/math/test_lcp.py`.

Verification completed for this slice:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
```

Observed results:

- C++ LCP suite: `100% tests passed, 0 tests failed out of 17`.
- The focused dartpy LCP test reported `62 passed`.
- `pixi run lint` passed before commit.

## Historical Hand-Off — 2026-06-11 Stop-Only Point

The previous user instruction was to stop all implementation work, do not run
further verification, ensure only the hand-off docs were current, and then
fully stop. After that instruction, the only edits were this file and
`docs/dev_tasks/lcp_solver_interface_demos/README.md`.

No lint, build, tests, benchmark-list commands, benchmark execution, solver
execution, code edits, commit, or push was run after that stop-only instruction.
Historical verification below belongs only to the named implementation
checkpoints that explicitly list it.

That stop-only hand-off recorded:

- Consolidated branch: `feature/lcp-solver-interface-demos`.
- Local HEAD before that docs-only hand-off edit:
  `e3353bf04b7 Expose LCP solver sweep metadata`.
- Tracking state before that docs-only hand-off edit:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 16]`.
- Recent local checkpoints:
  - `e3353bf04b7 Expose LCP solver sweep metadata`
  - `46700198d80 Expose LCP scale benchmark metadata`
  - `197b55c335a Use concrete LCP contact registration gates`
- The worktree was clean before that final docs-only hand-off edit.

The suspected non-finite lower-bound follow-up from that stop-only note was
rechecked during the current continuation. Existing tests intentionally use
`lo = -inf` or `lo = 0` storage on some friction-index rows while deriving
effective tangent bounds from `hi`; do not tighten that behavior without a
separate design decision.

## Previous Completed Implementation — 2026-06-11 Py-Demo Metadata

The latest completed implementation slices extend the Python LCP demo's
representative benchmark packet metadata:

- `active_set_scale` now points at the larger, stress, extreme, production,
  and production-batch active-set transition benchmark rows.
- `singular_degenerate_scale` now points at larger, stress, extreme, and
  serial/parallel batch singular-degenerate benchmark rows.
- `solver_parameter_sweeps` now points at solver-specific relaxation,
  line-search, pivoting-scale, block-partition, restart-policy,
  iteration-budget, shock-layer, SPD-check, path-following, ADMM rho, and SAP
  regularization benchmark sweeps.
- The existing representative filter union continues to derive
  `representative_benchmark_filter` and `representative_benchmark_command`
  from every packet row, so the demo now exposes the current scalability and
  hard-degeneracy benchmark surfaces without duplicating another command owner.
- `python/tests/unit/test_py_demo_panels.py`, `CHANGELOG.md`, and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe and cover the
  slice.

Verification completed for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(Larger|Stress|Extreme|Production)ActiveSetTransition|BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)|BM_Lcp(Larger|Stress|Extreme)SingularDegenerate|BM_LcpSingularDegenerate(FrictionIndexBatch|StandardBoxedBatch)(Serial|Parallel)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(PgsRelaxationSweep|SymmetricPsorRelaxationSweep|RedBlackGaussSeidelRelaxationSweep|BoxedSemiSmoothNewtonLineSearchSweep|PivotingScaleSweep|BlockPartitionSweep|ApgdRestartSweep|TgsIterationBudgetSweep|NncgPgsIterationsSweep|SubspaceMinimizationPgsIterationsSweep|ShockPropagationLayerSweep|MprgpSpdCheckSweep|InteriorPointPathSweep|AdmmRhoSweep|SapRegularizationSweep)'
pixi run lint
```

Observed results:

- The Python demo-panel test reported `43 passed`.
- The benchmark-list check rebuilt/linked `BM_LCP_COMPARE` and listed the
  newly exposed active-set scale/production and singular-degenerate
  scale/batch benchmark rows.
- A second benchmark-list check listed the newly exposed solver-specific
  tuning and robustness sweep rows.
- `pixi run lint` passed.

The latest implementation slice removes redundant manifest-family prechecks
from contact benchmark registration paths:

- `RegisterActiveFrictionIndexContactBenchmarks()`,
  `RegisterWorldContactBenchmarks()`, `RegisterWorldBoxContactBenchmarks()`,
  `RegisterWorldStackContactBenchmarks()`,
  `RegisterArticulatedUnifiedContactBenchmarks()`,
  `RegisterContactSolverComparisonSweepBenchmarks()`,
  `RegisterContactNormalStandardSweepBenchmarks()`,
  `RegisterWorldContactBatchBenchmarks()`, and
  `RegisterWorldBoxContactBatchBenchmarks()` now rely on concrete generated
  contact packets or support probes instead of static
  `LcpProblemSupport` prechecks.
- Dense world-box contact registrations still keep the explicit
  `SupportsDenseWorldBoxContactPatch(...)` solver scope before the concrete
  probe.
- The audited LCP benchmark/test/demo surface no longer has
  `dart::test::supportsProblem(solverEntry, LcpProblemSupport::...)` prechecks.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(ActiveFrictionIndexContact|WorldContact|WorldBoxContact|WorldStackContact|ArticulatedUnifiedContact|ContactSolverComparisonSweep|ContactNormalStandardSweep|WorldContactBatchSerial|WorldContactStressBatchSerial|WorldContactPipeline32BatchSerial|WorldBoxContactBatchSerial).*(Dantzig|Pgs|MPRGP|Baraff|BoxedSemiSmoothNewton)'
pixi run lint
```

Observed results:

- The benchmark-list check rebuilt and linked `BM_LCP_COMPARE`.
- It listed representative concrete rows for active friction-index contact,
  world contact, dense world-box contact, world stack contact, articulated
  unified contact, contact solver comparison, normal-standard contact, mixed
  world-contact batches, and dense world-box contact batches.
- `pixi run lint` passed, including the LCP solver roster check.

Resume status:

- The broader LCP solver/interface/demo objective is not complete. Resume from
  the next concrete support-routing, solver-domain, demo, benchmark, or
  performance gap after this slice lands.

## Previous Reality — 2026-06-11 Concrete Benchmark Helper Cleanup

That continuation built on
`38266f484e2 Route LCP smoke tests through concrete support` on
`feature/lcp-solver-interface-demos`.

The implementation slice removed redundant manifest-family prechecks from
concrete benchmark-routing helpers:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` no longer keeps
  `getMildIllConditionedProblemSupport(...)`,
  `getNearSingularProblemSupport(...)`,
  `getLargerActiveSetTransitionProblemSupport(...)`, or
  `getSingularDegenerateProblemSupport(...)`.
- The corresponding `SolverShouldRun...Benchmark(...)` helpers now rely on
  explicit solver scopes plus `SolverSupportsConcreteProblem(...)` for exact
  generated problems.
- The production active-set transition batch helper now uses the same simplified
  concrete-problem helper signature.

Verification completed for that slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(MildIllConditioned|NearSingular|LargerActiveSetTransition|ProductionActiveSetTransitionBatchSerial|SingularDegenerate|SingularDegenerateFrictionIndexBatchSerial|SingularDegenerateStandardBoxedBatchSerial)/(Standard32|Boxed16|FrictionIndex8|Standard8|Boxed8|CoupledFrictionIndex3|Standard16|CoupledFrictionIndex6|CoupledFrictionIndex8)/(Direct|MPRGP|Baraff|Dantzig|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run lint
```

Observed results:

- The first benchmark-list attempt caught an unused `testCase` parameter after
  the precheck removal; the helper signature and callers were simplified.
- The rerun rebuilt and linked `BM_LCP_COMPARE`, then listed concrete active
  set, production batch, mild/near-singular, and singular-degenerate rows for
  the scoped solver/problem combinations.
- `pixi run lint` passed, including the LCP solver roster check.

## Previous Reality — 2026-06-11 All-Solvers Smoke Checkpoint

That continuation built on
`29ece871105 Filter pivoting LCP sweeps concretely` on
`feature/lcp-solver-interface-demos`.

The implementation slice routed the generated all-solvers smoke-test skip
helper through concrete per-problem support:

- `tests/unit/math/lcp/test_all_solvers_smoke.cpp` removes the local
  `ProblemCategory` to `LcpProblemSupport` manifest-family mapping.
- `canSolve(...)` now constructs the solver instance and calls
  `instance->supportsProblem(problem.problem)`, returning false when solver
  creation fails.

Verification completed for that slice:

```bash
pixi run test-lcpsolver
pixi run lint
```

Observed results:

- The focused LCP test task rebuilt as needed and reported
  `100% tests passed, 0 tests failed out of 17`.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke` passed.
- `pixi run lint` passed, including the LCP solver roster check.

## Previous Reality — 2026-06-11 Pivoting Checkpoint

That continuation resumed from
`e86dbf2efa5 Filter singular LCP batches concretely` on
`feature/lcp-solver-interface-demos`.

The latest implementation slice filters pivoting scale sweep benchmark
registration through concrete generated-problem support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now precomputes the exact
  generated problem for each `BM_LcpPivotingScaleSweep` row and publishes it
  only when the selected solver's concrete `supportsProblem(problem)` predicate
  accepts the packet.
- Runtime pivoting scale sweep execution now uses the same concrete guard.
- The now-unused manifest-family `getProblemSupport(...)` helper was removed.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpPivotingScaleSweep/(Standard|Boxed|FrictionIndex)/(Direct|Lemke|Baraff|Dantzig)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpPivotingScaleSweep/Standard/Direct/Rows3|BM_LcpPivotingScaleSweep/Standard/Baraff/Rows8|BM_LcpPivotingScaleSweep/Boxed/Dantzig/Rows12|BM_LcpPivotingScaleSweep/FrictionIndex/Dantzig/Contacts4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The first benchmark-list attempt caught the now-unused
  `getProblemSupport(...)` helper under `-Werror`; that helper was removed.
- The rerun rebuilt `BM_LCP_COMPARE` and listed Direct standard rows 2 and 3,
  Lemke/Baraff/Dantzig standard rows, Dantzig boxed rows, and Dantzig
  friction-index rows.
- The short benchmark execution reported `contract_ok=1` for sampled Direct
  standard, Baraff standard, Dantzig boxed, and Dantzig friction-index rows.

The broader LCP solver/interface/demo objective is not complete. Resume from
the next concrete support-routing, solver-domain, demo, benchmark, or
performance gap after this slice lands.

## Previous Reality — 2026-06-11 Singular Batch Checkpoint

The current continuation resumes from
`911e9530c41 Document latest LCP handoff checkpoint` on
`feature/lcp-solver-interface-demos`.

The latest implementation slice filters singular-degenerate batch benchmark
registrations through exact generated-batch support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now precomputes the exact
  four-problem singular-degenerate friction-index batch for each batch row and
  requires every concrete problem to pass `supportsProblem(problem)` before
  publishing serial or parallel rows.
- Singular-degenerate standard/boxed serial and parallel batch rows now use the
  same exact generated-batch support check.
- Single singular-degenerate registrations now precompute the generated problem
  once per case and pass it to the shared concrete helper.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpSingularDegenerate(FrictionIndexBatch|StandardBoxedBatch)(Serial|Parallel)/(Standard16|Boxed16|CoupledFrictionIndex6|Standard128|Boxed128|CoupledFrictionIndex16|CoupledFrictionIndex256)/(Direct|MPRGP|Baraff|Admm|Sap|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpSingularDegenerateFrictionIndexBatchSerial/CoupledFrictionIndex6/Sap|BM_LcpSingularDegenerateFrictionIndexBatchParallel/CoupledFrictionIndex6/Sap|BM_LcpSingularDegenerateStandardBoxedBatchSerial/Standard16/Baraff|BM_LcpSingularDegenerateStandardBoxedBatchSerial/Boxed16/BoxedSemiSmoothNewton' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The benchmark-list check rebuilt `BM_LCP_COMPARE` and listed exact supported
  singular-degenerate batch rows for Admm, Sap, BoxedSemiSmoothNewton, and
  Baraff where their concrete generated batches are supported; Direct and MPRGP
  were absent from the scoped filter.
- The short benchmark execution reported `contract_ok=1` for sampled Sap
  friction-index serial and parallel batch rows, Baraff standard-batch rows,
  and BoxedSemiSmoothNewton boxed-batch rows.

The broader LCP solver/interface/demo objective is not complete. Resume from
the next concrete support-routing, solver-domain, demo, benchmark, or
performance gap after this slice lands.

## Critical Hand-Off — 2026-06-11 Final Stop Point

The latest user instruction was critical: stop implementation work and focus on
hand-off only, with no further verification. After that instruction, no lint,
build, tests, benchmark-list commands, benchmark execution, or solver execution
were run. Historical verification below belongs only to the implementation
checkpoints that explicitly list it.

Use this as the fresh-session entrypoint:

- Consolidated branch: `feature/lcp-solver-interface-demos`.
- Latest implementation checkpoint:
  `9a17ba85aa5 Filter conditioning LCP benchmarks concretely`.
- Previous checkpoint:
  `559cda91ace Filter active-set scale LCP benchmarks concretely`.
- `main` was fetched from `https://github.com/dartsim/dart.git` at
  `7d05d7b9ea72`; `git merge --no-edit FETCH_HEAD` reported
  `Already up to date.`.
- This checkout's `origin` remote is SSH. Previous successful publishes used
  HTTPS, so a fresh session should fetch the branch from GitHub before relying
  on local remote-tracking metadata.

Current branch contents to preserve:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` filters mildly
  ill-conditioned and near-singular benchmark registrations through concrete
  generated-problem support, including exact serial/parallel batch problem-list
  checks.
- `CHANGELOG.md` records the conditioning benchmark-routing checkpoint.
- `docs/dev_tasks/lcp_solver_interface_demos/README.md` and this resume file
  capture the full hand-off context.

Resume guidance:

1. Fetch and check out `feature/lcp-solver-interface-demos` from GitHub.
2. Read `docs/ai/principles.md`, `docs/dev_tasks/README.md`, this file, and
   the dev-task README before editing.
3. Inspect the current branch tip and continue one bounded remaining LCP
   interface/demo audit gap.
4. Do not treat this hand-off as completion of the broad LCP objective, and do
   not retire `docs/dev_tasks/lcp_solver_interface_demos/` from this checkpoint
   alone.

Likely next gaps:

- Continue auditing benchmark/demo/test surfaces that still summarize native
  solver support without checking concrete generated problems.
- Review remaining solver-domain predicates against the actual native paths and
  fallback/delegation behavior.
- Continue py-demo and benchmark apples-to-apples coverage work after the next
  session re-establishes local verification.

## Current Reality — 2026-06-11 Active Continuation

The current continuation resumes from
`c564b6b2ba2 Report concrete LCP demo native coverage` on
`feature/lcp-solver-interface-demos`.

The latest implementation slice filters mildly ill-conditioned and near-singular
benchmark registrations through concrete generated-problem support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now precomputes the generated
  problem for each `BM_LcpMildIllConditioned` and `BM_LcpNearSingular` row and
  only publishes solver rows whose concrete `supportsProblem(problem)`
  predicate accepts the packet.
- Mildly ill-conditioned and near-singular batch rows now precompute the exact
  four-problem batch and require every concrete batch problem to be supported
  before publishing serial or parallel rows.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the latest
  slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpMildIllConditioned/(Standard32|Boxed16|FrictionIndex8|ExtremeCoupledFrictionIndex256)/(Dantzig|Baraff|MPRGP|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpNearSingular/(Standard8|Boxed8|CoupledFrictionIndex3|CoupledFrictionIndex256)/(Dantzig|Baraff|MPRGP|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(MildIllConditioned|NearSingular)Batch(Serial|Parallel)/(Standard32|Boxed16|FrictionIndex8|CoupledFrictionIndex8|ExtremeCoupledFrictionIndex256|Standard8|Boxed8|CoupledFrictionIndex3)/(Dantzig|Baraff|MPRGP|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpMildIllConditioned/Standard32/Baraff|BM_LcpMildIllConditionedBatchSerial/CoupledFrictionIndex8/BoxedSemiSmoothNewton|BM_LcpNearSingular/Boxed8/BoxedSemiSmoothNewton|BM_LcpNearSingularBatchSerial/CoupledFrictionIndex3/ShockPropagation' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The mild row-list check rebuilt `BM_LCP_COMPARE` and listed scoped concrete
  rows for Standard32, Boxed16, FrictionIndex8, and
  ExtremeCoupledFrictionIndex256; MPRGP and Direct were absent.
- The near-singular row-list check listed scoped concrete rows for Standard8,
  Boxed8, CoupledFrictionIndex3, and CoupledFrictionIndex256; MPRGP, PGS, and
  Direct were absent.
- The batch row-list check listed serial and parallel mild/near-singular rows
  for the exact supported four-problem batches.
- The short benchmark execution reported `contract_ok=1` for the sampled
  mild/near-singular single and serial-batch rows.
- `pixi run lint`: passed.

Previous implementation slice: larger active-set transition benchmark
registrations were filtered through concrete generated-problem support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now precomputes the generated
  problem for larger, stress, extreme, and production active-set transition
  rows and only publishes solver rows whose concrete
  `supportsProblem(problem)` predicate accepts the packet.
- Production active-set transition batch rows now precompute the exact
  four-problem batch and require every concrete batch problem to be supported
  before publishing serial or parallel rows.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(Larger|Stress|Extreme|Production)ActiveSetTransition/(Standard32|Boxed32|CoupledFrictionIndex8|Standard64|Boxed64|CoupledFrictionIndex12|Standard128|Boxed128|CoupledFrictionIndex16|CoupledFrictionIndex24)/(MPRGP|Baraff|Direct|Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)/(Standard32|Boxed32|CoupledFrictionIndex8)/(MPRGP|Baraff|Direct|Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpLargerActiveSetTransition/Standard32/MPRGP|BM_LcpStressActiveSetTransition/Boxed64/Pgs|BM_LcpProductionActiveSetTransitionBatchSerial/CoupledFrictionIndex8/BoxedSemiSmoothNewton' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The single-row benchmark-list check rebuilt `BM_LCP_COMPARE` and listed
  concrete larger/stress/extreme/production active-set transition rows for
  Dantzig, PGS, MPRGP, and BoxedSemiSmoothNewton where supported; Direct and
  Baraff were absent from the filtered row set.
- The production-batch benchmark-list check listed serial and parallel rows for
  Standard32, Boxed32, and CoupledFrictionIndex8 for the supported solver set.
- The short benchmark execution reported `contract_ok=1` for the sampled
  larger active-set, stress active-set, and production batch rows.
- `pixi run lint`: passed.

The broader LCP solver/interface/demo objective is not complete. Resume from
the next concrete support-routing, solver-domain, demo, benchmark, or
performance gap after this slice lands.

## Previous Reality — 2026-06-11 Critical Hand-Off

The latest user instruction was to stop implementation and focus on hand-off
only, with no further verification. No lint, build, tests, benchmark listing,
solver execution, or additional implementation work was run after that
instruction. Historical verification notes below belong only to their named
implementation checkpoints.

This hand-off consolidates the current work on
`feature/lcp-solver-interface-demos`. Before the final hand-off checkpoint
commit, local implementation HEAD was:

```text
2cd337aa0cf Use concrete gates for LCP benchmark registration
```

The working tree at the stop point contained the Python demo concrete
native-case profile slice:

- `CHANGELOG.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

That slice makes the LCP py-demo solver profile report native coverage from
the concrete representative problem rows instead of only static manifest
surfaces. The panel now exposes partial native coverage such as Direct's
`standard 2/4` case count, while full-surface solvers report labels such as
`standard 4/4, boxed 3/3, findex 2/2`.

Before this hand-off checkpoint, local status showed this branch ahead of the
stale SSH tracking ref by five commits plus the uncommitted Python slice:

```text
## feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 5]
 M CHANGELOG.md
 M docs/dev_tasks/lcp_solver_interface_demos/README.md
 M docs/dev_tasks/lcp_solver_interface_demos/RESUME.md
 M python/examples/demos/scenes/lcp_physics.py
 M python/tests/unit/test_py_demo_panels.py
```

Do not treat `origin/feature/lcp-solver-interface-demos` in this checkout as
authoritative: `origin` is configured for SSH, while previous successful
publishes used HTTPS and did not refresh the local SSH remote-tracking ref. A
fresh session should fetch the remote branch directly from GitHub.

For this final hand-off, `main` was fetched over HTTPS from
`https://github.com/dartsim/dart.git` to `7d05d7b9ea72`, and
`git merge --no-edit FETCH_HEAD` reported `Already up to date.`

Base first-parent stack before this final hand-off checkpoint commit:

```text
2cd337aa0cf Use concrete gates for LCP benchmark registration
6c0763927a5 Filter grouped LCP batch rows concretely
20aaa23d0fe Document final LCP no-verification handoff
fe5b70b32cc Route generated LCP coverage by concrete support
f86e353df2f Gate heavyweight LCP contact benchmarks concretely
737b9c95c11 Document final LCP handoff checkpoint
8f0242c2442 Filter LCP contact benchmark rows concretely
4c63db30bd7 Filter LCP benchmark args concretely
be4643d1743 Document consolidated LCP handoff state
```

Resume from the next bounded gap in the broader LCP interface/demo audit
rather than repeating the completed benchmark-routing, generated-coverage, or
Python solver-profile slices. The broad LCP objective is not complete.

## Last Session Summary

The current work is consolidated on `feature/lcp-solver-interface-demos`.
Recent checkpoints added active friction-index demo/benchmark coverage, refined
Dantzig friction-index fixed-bound re-solves, and exposed direct LCP problem
validation diagnostics in C++ and dartpy. The latest checkpoint tightens
`supportsProblem(problem)` so solver-specific native limits are visible to
py-demos, starting with `DirectSolver` treating only n <= 3 standard packets as
native and marking larger standard packets as delegated.

The latest continuation exposed the broader DART 7 contact-pipeline benchmark
comparison sweeps in the LCP py-demo metadata while keeping the exact
`active_friction_index_contact` row pointed at its two-solver regression
benchmark. It then added `billiard_symmetry_error` to the live LCP py-demo so
billiard symmetry is tracked alongside momentum and kinetic-energy error. The
high-mass-ratio stack benchmark metadata now points at
`BM_LcpWorldStackContact/|BM_LcpWorldStackStep_BoxedLcp`. The live LCP panel
also plots billiard momentum, energy, and symmetry invariant histories for both
comparison worlds. The next checkpoint added
`representative_benchmark_filter` and `representative_benchmark_command` to the
LCP py-demo metadata, derived from the benchmark packet table while preserving
the existing smoke command. The current slice realigns LCP background docs with
DART 7 snake_case solver header paths and extends the LCP roster lint gate to
reject stale documented LCP header/source paths.

The user then gave a critical stop instruction: focus on hand-off only and run
no further verification. No code changes were made after that instruction. The
interrupted next-slice audit found a likely bounded follow-up in
`MprgpSolver::supportsProblem(...)`: MPRGP currently inherits the base
form-level predicate even though its native standard-LCP path rejects
non-symmetric matrices and, by default, matrices that fail an LLT positive
definite check before delegating to Dantzig.

The current continuation implements that MPRGP follow-up: MPRGP now overrides
`supportsProblem(problem, standardTolerance)` and reports native support only
for standard problems that satisfy its configured symmetry and
positive-definite native-path checks. Focused C++ and dartpy tests cover native
SPD support, boxed false support, non-symmetric false support, default
indefinite false support, and the C++ parameter case where
`checkPositiveDefinite = false`.

The next continuation applies the same native-route audit to Baraff: Baraff now
reports native support only for symmetric positive-semidefinite standard
packets, and non-standard, non-symmetric, or indefinite packets delegate to
Dantzig before the active-set loop. Focused C++ and dartpy tests cover SPD,
PSD, boxed, non-symmetric, indefinite, and fallback solve behavior.

The final update in this session is hand-off only. The user explicitly asked
to stop implementation and run no further verification. The branch was
refreshed against `origin/main`, was already up to date, and this document now
captures the next interrupted audit: benchmark/demo routing should be checked
against the new concrete `supportsProblem(problem)` predicates for MPRGP and
Baraff before any more solver-comparison rows are exposed as native.

The next continuation completed the first benchmark-routing slice:
singular-degenerate standard benchmark registration now intersects the scoped
solver allowlist with each solver's concrete `supportsProblem(problem)`
predicate. The generated standard packets are symmetric PSD but rank-deficient,
so Baraff remains listed as native while MPRGP's default positive-definite
native gate removes its fallback rows from the native benchmark list.

The next continuation completed the following benchmark-routing slice:
contact-normal standard sweep registration now builds each concrete normal-only
contact packet once and registers only solvers whose `supportsProblem(problem)`
accepts that packet. Current MPRGP and Baraff contact-normal rows remain
registered because the generated packets satisfy their native predicates, and
Direct remains limited to concrete 1-, 2-, and 3-row normal packets through the
same support predicate rather than a size special case.

The latest user instruction on 2026-06-11 was critical: stop implementation
and focus on hand-off only with no further verification. No code, lint, build,
test, benchmark-list, or solver-execution work was performed after that
instruction. This resume document and the dev-task README are the only intended
changes in the final hand-off checkpoint.

The current continuation resumes implementation from that hand-off and closes
the active-friction-index benchmark routing gap: `lcp_compare` now registers
manifest-driven `BM_LcpActiveFrictionIndexContact/FrictionIndex/<solver>` rows
for every friction-index-capable solver that accepts the concrete
`LcpProblemFactory::activeFrictionIndexContact()` packet, and the Python LCP
demo metadata now points its representative active-friction filter at that
main comparison row family instead of the older two-solver microbenchmark
surface.

The follow-up continuation also replaces the active-set transition benchmark's
hand-coded Direct skip with the same concrete `supportsProblem(problem)` gate.
For the standard 16-row active-set transition packet, Baraff and MPRGP remain
registered as native rows while Direct is excluded by its own native-support
predicate instead of a benchmark-local size special case.

The latest user instruction is another critical stop: do not continue
implementation and do not run further verification. The only intended final
work is this hand-off documentation checkpoint, followed by publishing the
single consolidated branch so a fresh session can resume there.

The current continuation resumed implementation from that pushed hand-off and
completed the next benchmark-routing slice: manifest-generated `BM_LcpCompare`
and serial/parallel `BM_LcpBatch` registrations now filter their candidate
argument rows through concrete generated-problem support instead of only the
manifest-level problem family. The existing row shape is preserved while Direct,
MPRGP, Baraff, and other solvers stay aligned with their
`supportsProblem(problem)` native-route predicates.

The latest continuation completed the next contact-routing slice: separated
world-contact, world stack-contact, and contact-solver comparison sweep
registrations now filter through concrete generated contact packets before
registering rows. Dense world-box contact, articulated unified contact, and
contact batch rows still use manifest-level friction-index gates because
eagerly constructing the largest 256-contact fixtures during benchmark
registration needs a separate strategy.

The final requested work is this no-verification hand-off checkpoint. It is
documentation-only and is intended to preserve the current branch state for a
fresh AI session.

The current continuation resumes implementation from that hand-off and closes
the remaining heavyweight contact benchmark registration gap. Dense box-contact
single rows, articulated unified-contact single rows, and dense box-contact
batch rows now use small concrete support probes before publishing their large
argument sets, while mixed world-contact batch rows check the exact generated
batch problem lists for the baseline, stress-stack, and contact-pipeline-32
families.

The follow-up continuation aligns generated LCP correctness coverage with that
same concrete support model: generated cases now skip by each solver's
`supportsProblem(testCase.problem)` result instead of a manifest-family check
plus a Direct-only size special case.

## Current Branch

`feature/lcp-solver-interface-demos` — consolidated branch for this work.

Current reality after the Python demo concrete native-case profile, benchmark
concrete-gate cleanup, grouped batch benchmark, heavyweight contact benchmark,
and generated coverage support-routing slices:

- The latest implementation slice changed `CHANGELOG.md`,
  `python/examples/demos/scenes/lcp_physics.py`,
  `python/tests/unit/test_py_demo_panels.py`, and these dev-task docs.
- The benchmark concrete-gate cleanup slice changed `CHANGELOG.md`,
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`, and these dev-task docs.
- The grouped batch implementation slice changed `CHANGELOG.md`,
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`, and these dev-task docs.
- The generated coverage implementation slice changed `CHANGELOG.md`,
  `tests/unit/math/lcp/test_lcp_generated_coverage.cpp`, and these dev-task
  docs.
- The heavyweight contact benchmark implementation slice changed `CHANGELOG.md`,
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`, and these dev-task docs.
- Local implementation HEAD before this hand-off checkpoint was
  `2cd337aa0cf Use concrete gates for LCP benchmark registration`; the Python
  demo native-case profile slice was still uncommitted when the final
  stop-and-handoff instruction arrived and is included in this checkpoint.
- The configured SSH remote-tracking ref still showed
  `origin/feature/lcp-solver-interface-demos` at
  `737b9c95c11 Document final LCP handoff checkpoint` before this hand-off
  commit; treat it as stale and fetch the branch directly from GitHub.
- `main` was fetched over HTTPS to `7d05d7b9ea72`; merging it with
  `git merge --no-edit FETCH_HEAD` reported `Already up to date.`
- No PR was associated with the branch when checked earlier in the session.

After this checkpoint is pushed, the single consolidated branch should remain
the source of truth for a fresh session. Merge latest `main` before any later
push.

## Immediate Next Step

Continue the broader LCP interface/demo audit from the next concrete gap. Do
not push without explicit approval. Do not treat the broad LCP objective as
complete.

## Context That Would Be Lost

- Public callers can now ask `LcpProblem::isValid()` /
  `getValidationMessage()` and Python `problem.is_valid()` /
  `problem.get_validation_message()` before solving.
- `detail::validateProblem(const LcpProblem&)` now delegates to the public
  diagnostic path, so solver rejection messages and demo diagnostics stay
  aligned for `LcpProblem` inputs.
- The raw validation overload in `lcp_validation.hpp` intentionally remains for
  raw matrix/vector validation call sites.
- The active friction-index contact packet is meant to exercise moving tangent
  bounds where a single Dantzig pass could leave stale fixed bounds.
- Current Direct support change: `DirectSolver` still solves larger standard
  packets through Dantzig fallback, but `supportsProblem()` returns false for
  those packets so demo native vs delegated rows match the actual route.
- `LcpSolver::supportsProblem(problem)` is virtual in the working tree so
  dartpy calls through the base binding can dispatch to `DirectSolver`'s
  override.
- `python/tests/unit/test_py_demo_panels.py` now expects Direct to be the only
  non-native solver for the 4-row near-singular standard case and the 12-row
  moderate-scale standard case.
- Verification for the Direct checkpoint passed: `pixi run lint`;
  clean rebuild of `UNIT_math_lcp_math_lcp_lcp_generated_coverage`; direct
  Ninja rebuild of all `UNIT_math_lcp_*` binaries after the clean; CTest
  `-R '^UNIT_math_lcp_'` passed 17/17; `python/tests/unit/math/test_lcp.py`
  passed 59 tests; and `python/tests/unit/test_py_demo_panels.py` passed
  43 tests.
- The first `pixi run test-lcpsolver` attempt exposed stale local vtable state
  after adding virtual methods to `LcpSolver`; a clean rebuild of the generated
  coverage binary removed the segfault.
- The branch should remain one additive published branch. Merge latest `main`
  before future pushes; avoid rebasing unless explicitly requested.
- Previous handoff note: no lint, build, test, benchmark listing, or solver
  execution was run after the earlier user instruction to stop and focus on
  hand-off.
- Current benchmark-routing audit: `bm_lcp_compare.cpp` still contains
  registrations
  that gate rows with manifest-level form support, for example
  `dart::test::supportsProblem(solverEntry, LcpProblemSupport::Standard)`.
  That is now too coarse for at least some MPRGP/Baraff rows because their
  concrete `supportsProblem(problem)` predicates report narrower native
  mathematical domains.
- Completed benchmark follow-up: singular-degenerate standard registration now
  calls a benchmark-local `SolverSupportsConcreteProblem(...)` helper after the
  scoped solver allowlist. The benchmark-list check for
  `BM_Lcp.*SingularDegenerate.*/(MPRGP|Baraff)` lists Baraff rows and no MPRGP
  rows.
- Completed benchmark follow-up: contact-normal standard registration now
  generates each concrete normal-only contact packet once and checks
  `supportsProblem(problem)` before registering rows. The benchmark-list check
  for `BM_LcpContactNormalStandardSweep/(MPRGP|Baraff|Direct)` keeps all
  Baraff/MPRGP rows and Direct only on concrete 1-, 2-, and 3-row normal
  packets.
- Completed benchmark follow-up: active friction-index contact registration now
  uses the shared active-contact factory packet in `bm_lcp_compare.cpp` and
  filters participating manifest solvers through concrete
  `supportsProblem(problem)` checks. The benchmark-list check for
  `BM_LcpActiveFrictionIndexContact` lists 16 friction-index-capable rows, and
  the short execution check reports `contract_ok=1` for all 16.
- Completed benchmark follow-up: active-set transition registration now checks
  concrete generated packets before registering native rows. The list check for
  `BM_LcpActiveSetTransition/Standard/(Direct|Baraff|MPRGP)` keeps Baraff and
  MPRGP and excludes Direct through its concrete support predicate.
- Completed benchmark follow-up: `BM_LcpCompare` and serial/parallel
  `BM_LcpBatch` registrations now compute candidate argument rows and keep only
  rows whose generated concrete problem or full generated batch is accepted by
  `supportsProblem(problem)`. Focused row-list checks covered Standard,
  Boxed, and FrictionIndex families, and a short execution check reported
  `contract_ok=1` on representative affected single, serial-batch, and
  parallel-batch rows.
- Completed benchmark follow-up: separated `BM_LcpWorldContact/FrictionIndex`,
  `BM_LcpWorldStackContact/FrictionIndex`, and
  `BM_LcpContactSolverComparisonSweep` registrations now prebuild the relevant
  concrete contact packets and register only solver/arg rows accepted by
  `supportsProblem(problem)`. Focused row-list checks covered representative
  Dantzig, PGS, BoxedSemiSmoothNewton rows and the scoped comparison solvers
  `Admm`, `Sap`, `BoxedSemiSmoothNewton`; a short execution check reported
  `contract_ok=1` on targeted rows and on two larger rows matched by the regex.
- Superseded contact registration gap: dense world-box contact, articulated
  unified contact, and contact batch rows previously used manifest-level
  friction-index gates because concrete filtering them naively would construct
  very large 256-contact fixtures during benchmark registration/listing.
- Completed benchmark follow-up: the heavyweight dense/contact registrations
  now use concrete solver support gates. `BM_LcpWorldBoxContact`,
  `BM_LcpArticulatedUnifiedContact`, and `BM_LcpWorldBoxContactBatch` use small
  concrete support probes to avoid registration-time construction of their
  largest fixtures; `BM_LcpWorldContactBatch` baseline, stress-stack, and
  contact-pipeline-32 families check the exact generated batch problem lists.
- Completed generated-coverage follow-up: `solverShouldRun(...)` in
  `tests/unit/math/lcp/test_lcp_generated_coverage.cpp` now uses each solver's
  concrete `supportsProblem(problem)` result, so Direct, Baraff, MPRGP, and
  future narrow native routes stay aligned with benchmark/demo support
  reporting. Focused verification rebuilt and passed
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage`; `pixi run lint` passed.
- Historical hand-off note: after the earlier 2026-06-11 stop instruction, no
  additional verification was run in that hand-off-only checkpoint, including
  `pixi run lint`. The later heavyweight contact support-gating continuation
  resumed implementation and ran focused benchmark checks plus `pixi run lint`.
- Generic standard rows that come from `MakeBenchmarkProblem(Standard, size)`,
  `MakeStandardSpdProblem`, MPRGP SPD-check sweeps, interior-point path sweeps,
  and mild/near-singular SPD builders may already be fine. Do not refactor all
  benchmark registration gates mechanically without inspecting the generated
  concrete packet.
- That active-friction-index metadata gap is now closed: the demo filter is
  `BM_LcpActiveFrictionIndexContact`, which matches the main `lcp_compare`
  benchmark row family added in this slice. The old two-solver rows remain in
  `bm_lcpsolver_solvers.cpp` as a narrower microbenchmark surface.
- Good source locations for that next audit are `kContactComparisonSolverNames`,
  `kContactNormalStandardSolverNames`, the world/contact benchmark functions,
  and the registration block near the end of `bm_lcp_compare.cpp`.
- `tests/benchmark/lcpsolver/bm_lcpsolver_solvers.cpp` appears to be a narrower
  older benchmark surface; confirm whether it is secondary before wiring demos
  to it.
- The contact-pipeline metadata checkpoint resolved the immediate missing
  py-demo rows for `BM_LcpContactSolverComparisonSweep`,
  `BM_LcpStaggeringContactPipelineSweep`, and
  `BM_LcpContactNormalStandardSweep`.
- Verification for that checkpoint passed:
  `python/tests/unit/test_py_demo_panels.py` passed 43 tests;
  `pixi run bm lcp_compare -- --benchmark_list_tests` listed the referenced
  sweep benchmarks; and `pixi run lint` passed.
- `billiard_symmetry_error` is the maximum lateral drift of either billiard
  ball from its initial collision line. The focused panel test passed 43 tests
  after adding the metric.
- The stack benchmark metadata now includes the manifest-driven stack contact
  benchmark across friction-index-capable solvers. The filter keeps the slash in
  `BM_LcpWorldStackContact/` to avoid matching the separate boxed contact
  assembly benchmark unintentionally.
- The billiard invariant plot checkpoint added GUI plots for sequential and
  boxed-LCP momentum error, energy error, and symmetry error histories. The
  focused panel test passed 43 tests after this update.
- Current smoke command metadata in
  `python/examples/demos/scenes/lcp_physics.py` is still
  `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`.
- The current slice adds `benchmark_smoke_filter`,
  `representative_benchmark_filter`, and `representative_benchmark_command` to
  `SceneSetup.info` from `lcp_physics.build()`.
- The representative filter is assembled from the union of every
  `_BENCHMARK_PACKET_ROWS[*]["benchmark_filter"]` token in row order. This keeps
  fresh sessions from having to manually concatenate active-set, contact,
  degeneracy, scaling, billiards, stack, card-pile, and articulated-contact
  benchmark filters.
- `python/tests/unit/test_py_demo_panels.py` should prove that splitting the
  representative filter on `|` equals the union of every benchmark packet row
  filter token, and keep the existing smoke assertion.
- Verification for the current slice passed:
  `python/tests/unit/test_py_demo_panels.py` passed 43 tests and
  `pixi run lint` passed.
- The source-layout docs slice committed as
  `7f4b0227eaf Align LCP docs with snake case headers` updates
  `docs/background/lcp/02_overview.md`,
  `docs/background/lcp/03_pivoting-methods.md`,
  `docs/background/lcp/05_newton-methods.md`, and
  `docs/background/lcp/06_other-methods.md` from legacy PascalCase-style header
  paths to DART 7 snake_case paths.
- `scripts/check_lcp_solver_roster.py` now scans LCP background docs for
  documented `dart/math/lcp` and LCP subdirectory header/source paths and fails
  if the referenced files do not exist.
- Focused verification for the current source-layout docs slice passed:
  `pixi run python scripts/check_lcp_solver_roster.py`.
- Final verification for the current source-layout docs slice passed:
  `pixi run lint`.
- Completed MPRGP predicate checkpoint:
  - `dart/math/lcp/other/mprgp_solver.hpp` now preserves the base overloads with
    `using LcpSolver::supportsProblem` and overrides the tolerance-aware
    predicate.
  - `dart/math/lcp/other/mprgp_solver.cpp` now checks base form support,
    empty-problem success, solver parameter validity, symmetry, and the
    configured LLT positive-definite gate.
  - `tests/unit/math/lcp/test_all_solvers_smoke.cpp` covers SPD, boxed,
    non-symmetric, indefinite, disabled positive-definite check, and invalid
    symmetry-tolerance predicate cases.
  - `python/tests/unit/math/test_lcp.py` covers the default dartpy-visible SPD,
    non-symmetric, indefinite, and boxed cases.
  - `docs/background/lcp/02_overview.md`,
    `docs/background/lcp/06_other-methods.md`,
    `docs/onboarding/python-bindings.md`, and `CHANGELOG.md` document the
    native-support behavior.
  - Focused verification passed:
    `cmake --build build/default/cpp/Release --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy`;
    `ctest -R '^UNIT_math_lcp_math_lcp_all_solvers_smoke$'`;
    `python/tests/unit/math/test_lcp.py` with 60 passed;
    `python/tests/unit/test_py_demo_panels.py` with 43 passed; and
    `pixi run python scripts/check_lcp_solver_roster.py`.
- Completed Baraff predicate checkpoint:
  - `dart/math/lcp/pivoting/baraff_solver.hpp` now preserves the base overloads
    with `using LcpSolver::supportsProblem` and overrides the tolerance-aware
    predicate.
  - `dart/math/lcp/pivoting/baraff_solver.cpp` now checks symmetric
    positive-semidefinite matrix support and delegates unsupported standard
    packets to Dantzig before the active-set loop.
  - `tests/unit/math/lcp/test_all_solvers_smoke.cpp` covers SPD, PSD, boxed,
    non-symmetric, indefinite, and fallback solve behavior.
  - `python/tests/unit/math/test_lcp.py` covers the same dartpy-visible support
    and fallback behavior.
  - `docs/background/lcp/02_overview.md`,
    `docs/background/lcp/03_pivoting-methods.md`,
    `docs/onboarding/python-bindings.md`, and `CHANGELOG.md` document the
    native-support behavior.
  - Focused verification passed:
    `cmake --build build/default/cpp/Release --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy`;
    `ctest -R '^UNIT_math_lcp_math_lcp_all_solvers_smoke$'`;
    `python/tests/unit/math/test_lcp.py` with 61 passed;
    `python/tests/unit/test_py_demo_panels.py` with 43 passed; and
    `pixi run python scripts/check_lcp_solver_roster.py`.

## How to Resume

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main
git log --oneline --decorate --max-count=12
```

Expected files in the latest final hand-off checkpoint:

- `CHANGELOG.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`

Expected files in the older Direct support checkpoint:

- `CHANGELOG.md`
- `dart/math/lcp/lcp_solver.hpp`
- `dart/math/lcp/pivoting/direct_solver.cpp`
- `dart/math/lcp/pivoting/direct_solver.hpp`
- `docs/background/lcp/02_overview.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `docs/onboarding/python-bindings.md`
- `python/tests/unit/math/test_lcp.py`
- `python/tests/unit/test_py_demo_panels.py`
- `tests/unit/math/lcp/test_all_solvers_smoke.cpp`

Expected files in the older source-layout docs checkpoint:

- `CHANGELOG.md`
- `docs/background/lcp/02_overview.md`
- `docs/background/lcp/03_pivoting-methods.md`
- `docs/background/lcp/05_newton-methods.md`
- `docs/background/lcp/06_other-methods.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `scripts/check_lcp_solver_roster.py`

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/background/lcp/02_overview.md`
- `docs/background/lcp/07_selection-guide.md`
- `docs/onboarding/python-bindings.md`
- `docs/onboarding/testing.md`

If the fetch succeeds and `origin/main` is not an ancestor of `HEAD`, merge it
before pushing or continuing:

```bash
git merge origin/main
```

Historical note: if a fresh checkout somehow lands before the Direct support
checkpoint, confirm the working tree matches the older Direct files above and
commit it with:

```text
Report Direct LCP native support precisely
```

Verification already run for that checkpoint:

```bash
pixi run lint
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_generated_coverage \
  --clean-first --parallel "$JOBS"
ninja -C build/default/cpp/Release -j "$JOBS" \
  UNIT_math_lcp_math_lcp_additional_solvers \
  UNIT_math_lcp_math_lcp_all_solvers_smoke \
  UNIT_math_lcp_math_lcp_dantzig_misc \
  UNIT_math_lcp_math_lcp_dantzig_solver \
  UNIT_math_lcp_math_lcp_dantzig_vs_ode \
  UNIT_math_lcp_math_lcp_lcp_comparison_harness \
  UNIT_math_lcp_math_lcp_lcp_edge_cases \
  UNIT_math_lcp_math_lcp_lcp_generated_coverage \
  UNIT_math_lcp_math_lcp_lcp_newton_solvers \
  UNIT_math_lcp_math_lcp_lcp_projection_solvers \
  UNIT_math_lcp_math_lcp_lcp_solvers_stress \
  UNIT_math_lcp_math_lcp_lcp_problems \
  UNIT_math_lcp_math_lcp_lcp_types \
  UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  UNIT_math_lcp_math_lcp_lemke \
  UNIT_math_lcp_math_lcp_pgs \
  UNIT_math_lcp_math_lcp_pivot_matrix
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^UNIT_math_lcp_' -j "$JOBS"
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Also run relevant LCP benchmark smoke rows if touching demo metadata, benchmark
manifests, or generated representative packets.
