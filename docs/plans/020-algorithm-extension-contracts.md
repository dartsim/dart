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

### 2. Baseline Comparison Harness

For the first algorithm family, define:

- baseline algorithms to compare;
- input datasets or generated scenarios;
- correctness metrics;
- performance metrics;
- determinism requirements;
- commands to run focused tests and benchmarks.

### 3. Public API Boundary Review

Before promoting any extension point:

- check whether it leaks `detail/`, `internal/`, backend, or storage types;
- define replacement wrappers if needed;
- decide whether Python exposure is appropriate;
- document deprecation/removal conditions for compatibility-only APIs.

### 4. Research Example Path

Provide a small example that demonstrates:

- registering or selecting an algorithm;
- running it against a baseline;
- collecting result metrics;
- interpreting failures or unsupported scenarios.

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
