# PLAN-020: Algorithm Extension Contracts and Baseline Comparisons

- Operating state: `PLAN-020` in `docs/plans/dashboard.md`

## Outcome

Researchers can implement algorithms from new papers and compare them against
DART baselines without rewriting shared foundations such as math, collision,
model loading, memory, threading, benchmarks, or tests.

## Scope

This plan covers the design path for research-facing algorithm extension
points, baseline comparison harnesses, and verification expectations.

Candidate algorithm families:

- LCP/contact/constraint solvers.
- Collision and distance query algorithms.
- Dynamics and integration algorithms.
- Optimization, inverse kinematics, and control methods.
- Batched or accelerated kernels that may later map to multi-core CPU or GPU
  backends.

Out of scope for this plan:

- Implementing every extension point up front.
- Exposing internal storage or backend implementation details as public API.
- Choosing GPU architecture; that belongs to `PLAN-030`.

## Current Evidence

- `docs/onboarding/architecture.md` describes DART's layered simulation core.
- `docs/onboarding/api-boundaries.md` defines supported, compatibility,
  experimental, and internal API boundaries.
- `docs/background/lcp/` already documents solver theory and selection context.
- `dart/math/lcp/` and `dart/constraint/` contain existing solver boundaries.
- `tests/benchmark/` contains benchmark infrastructure for baseline
  comparisons.

## Selected First Family

The first family to formalize is LCP/contact/constraint solving. It is the
right starting point because it already has theory docs, public solver types,
constraint integration, focused tests, and benchmark harnesses. It also connects
directly to compute-scalability questions in `PLAN-030`: Jacobi-style solvers
and contact grouping are the clearest current candidates for parallel or GPU
experiments.

## Design Criteria

Research-facing extension points should:

- accept data through stable public contracts, not internal storage layouts;
- expose enough hooks to implement a new paper faithfully;
- make baseline selection explicit and reproducible;
- support deterministic regression tests where possible;
- provide benchmark harnesses that compare against built-in DART baselines;
- keep threading, allocator, SIMD, and backend details behind internal
  boundaries unless they are intentionally public.

## Workstreams

### 1. Extension-Point Inventory

Inventory candidate extension points and classify each as:

| Classification             | Meaning                                    |
| -------------------------- | ------------------------------------------ |
| Supported public contract  | Ready for researchers to build on          |
| Experimental contract      | Useful but allowed to change with warnings |
| Compatibility-only surface | Kept for existing users, not preferred     |
| Internal detail            | Not safe for research-facing extension     |

The first inventory should prioritize one algorithm family rather than all of
DART. LCP/contact solving is the initial candidate because it already has
theory docs, implementations, and benchmark context.

Initial LCP/contact inventory:

| Surface                                                                            | Current classification                            | Evidence                                                                                                                              | Follow-up                                                                                                                                   |
| ---------------------------------------------------------------------------------- | ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| `dart::math::LcpProblem`, `LcpOptions`, `LcpResult`, `LcpSolver`                   | Supported public contract candidate               | Public headers in `dart/math/lcp/`, background docs in `docs/background/lcp/`, and unit tests in `tests/unit/math/lcp/`.              | Confirm Doxygen/user-facing docs describe ownership, tolerances, status values, and validation expectations clearly enough for researchers. |
| Concrete solver families under `dart/math/lcp/{pivoting,projection,newton,other}/` | Public baseline algorithms with varying maturity  | Solver headers are public, the selection guide names available methods, and current tests cover solver contracts and edge cases.      | Decide which solvers are supported extension baselines versus compatibility or experimental research surfaces.                              |
| `dart::constraint::BoxedLcpSolver` and constraint solver integration               | Integration contract candidate                    | `dart/constraint/` wraps LCP solving for contact and joint constraints; onboarding constraint docs describe solver use.               | Separate user-selectable solver policy from internal constraint grouping and contact storage details.                                       |
| `dart/math/lcp/pivoting/dantzig/*` helper APIs                                     | Internal or exposed implementation debt candidate | Dantzig detail helpers are in public-looking headers and have focused tests, but they expose algorithm storage and matrix operations. | Apply the API-boundary checklist before documenting or changing these as extension points.                                                  |
| `tests/common/lcpsolver` and `tests/benchmark/lcpsolver`                           | Internal harness and baseline evidence            | The benchmark docs point to the solver-agnostic `BM_LCP_COMPARE` path.                                                                | Keep harness output under the build tree and document representative benchmark filters for reproducible comparisons.                        |

### 2. Baseline Comparison Harness

For the first algorithm family, define:

- baseline algorithms to compare;
- input datasets or generated scenarios;
- correctness metrics;
- performance metrics;
- determinism requirements;
- commands to run focused tests and benchmarks.

Initial baseline path:

- Baselines: Dantzig, PGS/PSOR, BGS, Lemke, Newton-family solvers, and other
  `dart::math::LcpSolver` implementations listed in the LCP selection guide.
- Scenarios: standard SPD LCPs, boxed active-bounds problems, and friction-index
  contact-like problems from the solver-agnostic benchmark harness.
- Correctness metrics: solver status, finite iterates, bound/complementarity
  residuals, validation failures, and deterministic behavior for fixed seeds.
- Performance metrics: Google Benchmark `cpu_time` and `real_time`, compared
  with `scripts/compare_benchmarks.py` when tracking regressions.
- Focused test command: build and run `UNIT_math_lcp_math_lcp_all_solvers_smoke`
  plus the relevant solver-specific unit tests.
- Benchmark command: `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`.

### 3. Public API Boundary Review

Before promoting any extension point:

- check whether it leaks `detail/`, `internal/`, backend, or storage types;
- define replacement wrappers if needed;
- decide whether Python exposure is appropriate;
- document deprecation/removal conditions for compatibility-only APIs.

Initial boundary risks:

- Treat `dart::math::LcpProblem`, options, results, and the abstract solver
  contract as the candidate public research surface; do not require researchers
  to depend on Dantzig matrix storage, row pointers, or pivot bookkeeping.
- Keep `dart/math/lcp/pivoting/dantzig/*` helper APIs out of the research-facing
  contract until the API-boundary inventory decides whether they are supported
  public API, compatibility debt, or internal detail.
- Keep `dart::constraint` contact grouping, cached contacts, and backend
  collision internals behind integration-level solver policy. They are useful
  evidence for real workloads but are not the first extension contract.
- Do not expose LCP solver internals to Python just because C++ headers are
  installed. Python exposure should wait for a clear user-facing workflow and
  value wrappers that avoid internal storage or lifetime hazards.
- Record benchmark harnesses as verification infrastructure, not as public API.

Completed bounded task: LCP extension-contract v0

- Objective: document the supported research-facing LCP solver contract around
  `LcpProblem`, `LcpOptions`, `LcpResult`, and `LcpSolver`, then identify which
  concrete solvers are baseline algorithms rather than extension APIs.
- Scope: docs and tests around the public math/LCP contract; exclude Dantzig
  matrix helpers, constraint contact caches, collision backend internals, and
  Python bindings unless a user-facing workflow justifies them.
- Implementation evidence: `docs/onboarding/constraints.md` now defines the v0
  LCP solver extension contract, baseline solver role, API-boundary exclusions,
  focused unit-test targets, and benchmark command.
- Test evidence: existing focused LCP tests cover all-solver smoke behavior,
  boxed and friction-index handling, validation, status values, warm-start and
  custom option workflows, and solver-specific edge cases.
- Verification command: focused LCP unit tests, the LCP smoke benchmark
  (`pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`), and
  API-boundary review against `docs/onboarding/api-boundaries.md`.
- Local verification: `UNIT_math_lcp_math_lcp_all_solvers_smoke` passed through
  `ctest`, and `BM_LCP_COMPARE_SMOKE` passed with `contract_ok=1`,
  `iterations=1`, `bound_violation=0`, and residual/complementarity counters
  on the local Release build.
- Escalation: create `docs/dev_tasks/lcp_extension_contract/` only if future
  work expands into API changes, benchmark fixture redesign, or Python
  exposure.

### 4. Research Example Path

Provide a small example that demonstrates:

- registering or selecting an algorithm;
- running it against a baseline;
- collecting result metrics;
- interpreting failures or unsupported scenarios.

Current example path:

- Example: `examples/lcp_physics`
- Solver selection: `--solver dantzig` for the exact pivoting baseline or
  `--solver pgs` for an iterative comparison.
- Scenario selection: `--scenario mass`, `--scenario box`, `--scenario ball`,
  `--scenario domino`, or `--scenario incline`.
- Metrics: headless runs print scenario, solver, frame count, simulated time,
  average step time, and final contact count.
- Baseline comparison command shape:
  `pixi run ex lcp_physics --headless --scenario mass --solver dantzig --frames 120`
  followed by the same command with `--solver pgs`.
- Local verification: the same command shape passed with `--frames 1` for both
  `--solver dantzig` and `--solver pgs`, building `lcp_physics` and reporting
  scenario, solver, simulated time, average step time, and final contact count.
- Boundary note: this example selects solvers through
  `constraint::ConstraintSolver::setLcpSolver()` and does not expose Dantzig
  pivot matrices, contact caches, collision backend storage, threading, or
  accelerator internals.

## Acceptance Criteria

Active implementation is justified when:

- the first algorithm family is selected;
- its current extension surface is inventoried;
- baseline metrics and benchmark commands are named;
- API-boundary risks are listed.

This plan is complete for the first algorithm family when:

- researchers have a documented extension contract;
- at least one built-in baseline comparison is reproducible;
- focused tests and benchmarks cover the contract;
- public/private API boundaries are clear;
- follow-up algorithm families are reprioritized in the roadmap.

## Revision Triggers

Revise this plan when:

- a new research direction needs a different first algorithm family;
- a benchmark exposes a weak or misleading baseline;
- an extension point leaks implementation details;
- Python API needs differ from C++ API needs;
- compute-scaling decisions from `PLAN-030` change algorithm boundaries.
