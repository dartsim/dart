# Algorithm Extension Contracts

## Status

Proposal. This document owns durable design rules for research-facing algorithm
extension points. Current sequencing for applying these rules lives in
`docs/plans/dashboard.md` under PLAN-020.

## Purpose

DART should let researchers implement algorithms from new papers and compare
them against built-in baselines without rebuilding shared foundations such as
model loading, math, collision, memory, threading, SIMD, tests, or benchmarks.

This design describes the contract shape for those extension points. Dashboard
entry PLAN-020 tracks when to apply the rules to another algorithm family.

## Contract Principles

Research-facing extension points should:

- accept data through stable public contracts, not internal storage layouts;
- expose enough hooks to implement a paper faithfully;
- make baseline selection explicit and reproducible;
- support deterministic regression tests where possible;
- provide benchmark harnesses that compare against DART baselines;
- keep threading, allocator, SIMD, ECS storage, and backend details behind
  internal boundaries unless they are intentionally public.

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

## Candidate Family Example

LCP/contact solving is a useful example candidate when applying these rules
because DART already has LCP theory docs, multiple solver implementations,
constraint integration, and benchmark context. PLAN-020 decides whether and
when it becomes the first formalized family.

Create a focused design doc such as `docs/design/lcp_solver_contract.md` only
after the plan formally selects that family and inventories the current solver
surface. Keep mathematical derivations and attribution-heavy background in
`docs/background/lcp/`; the design doc should own DART-specific interface and
comparison rules.

## Verification Expectations

Before promoting an extension point:

- inventory the public, experimental, compatibility, and internal surfaces;
- check for leaks of `detail/`, `internal/`, backend, storage, or allocator
  types;
- define at least one reproducible baseline comparison;
- add focused tests for the contract and failure modes;
- document benchmark commands and unsupported scenarios.
