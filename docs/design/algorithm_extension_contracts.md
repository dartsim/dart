# Algorithm Extension Contracts

## Status

This document owns durable design rules for research-facing algorithm
extension points. The receiving active plan in `docs/plans/dashboard.md` owns
execution; PLAN-040 coordinates readiness. PLAN-020 is completed background.

## Purpose

DART should let researchers implement algorithms from new papers and compare
them against built-in baselines without rebuilding shared foundations such as
model loading, math, collision, memory, threading, SIMD, tests, or benchmarks.

This design describes the contract shape for those extension points. Dashboard
entries and the solver-family intake determine when to apply them to another family.

## Contract Principles

Research-facing extension points should:

- accept data through stable public contracts, not internal storage layouts;
- expose enough hooks to implement a paper faithfully;
- make baseline selection explicit and reproducible;
- support deterministic regression tests where possible;
- provide benchmark harnesses that compare against DART baselines;
- keep threading, allocator, SIMD, ECS storage, and backend details behind
  internal boundaries unless they are intentionally public.

Solver and multi-physics methods use the plan-owned
[`solver-family intake checklist`](../plans/solver-family-intake.md) before
starting a solver paper or component slice. This durable contract supplies the
stable principles behind that checklist: route work to an existing family when
possible, inventory shared collision/kinematics/optimization/benchmark
components, define apples-to-apples evidence against DART incumbents and
reference implementations, and keep user-facing configuration simple,
validated, serializable where result-affecting, and diagnostic-rich.

## API Boundary Rules

Apply `docs/onboarding/api-boundaries.md` to every extension point:

- Supported public contracts need documentation, tests, and stable names.
- Experimental contracts may change faster, but still need warnings, tests, and
  ownership.
- Compatibility-only surfaces should name replacement APIs and removal
  conditions.
- Internal details must stay under `detail/`, `internal/`, private component
  storage, or source-local code and should not be mirrored into dartpy.

Python bindings should expose an extension point only when the Python workflow
is useful to end users and can avoid leaking C++ implementation details.

## Baseline Comparison Contract

Each extension family should define:

- built-in baseline algorithms to compare against;
- input datasets or generated scenarios;
- correctness metrics and tolerances;
- performance metrics and benchmark commands;
- determinism requirements;
- unsupported cases and fallback behavior.

Benchmarks should compare the new algorithm against an explicit DART baseline
instead of reporting isolated timing numbers.

## Existing Family Example

LCP/contact solving already has a formalized contract and multiple implementations.
Reuse `docs/design/lcp_solver_contract.md` and `docs/background/lcp/` as the
existing example; PLAN-020 is not an open family-selection decision. New
families receive active ownership through the solver intake before work starts.

Variant and selection-policy versions, supported physical/representation cells,
continuation state, interaction ownership and actual compute execution are part
of the comparison contract. Internal extension seams can evolve with evidence;
a public third-party plugin ABI remains a separate design decision.

## Verification Expectations

Before promoting an extension point:

- inventory the public, experimental, compatibility, and internal surfaces;
- check for leaks of `detail/`, `internal/`, backend, storage, or allocator
  types;
- define at least one reproducible baseline comparison;
- add focused tests for the contract and failure modes;
- document benchmark commands and unsupported scenarios.
