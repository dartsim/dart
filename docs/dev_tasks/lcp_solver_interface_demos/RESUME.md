# Resume: LCP Solver Interface And Demos

## Last Session Summary

The current work is consolidated on `feature/lcp-solver-interface-demos`.
Recent checkpoints added active friction-index demo/benchmark coverage, refined
Dantzig friction-index fixed-bound re-solves, and exposed direct LCP problem
validation diagnostics in C++ and dartpy. The branch was pushed through
`Map LCP demo to active friction benchmarks`; this handoff checkpoint adds the
validation API, tests, docs, changelog, and this dev-task state.

## Current Branch

`feature/lcp-solver-interface-demos` — expected clean and pushed to
`origin/feature/lcp-solver-interface-demos` after the handoff commit.

## Immediate Next Step

Start a fresh audit for the next bounded DART 7 LCP gap. Do not assume the
broad LCP goal is complete. First verify branch state, fetch `origin/main`, and
merge it into the branch if main has moved.

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
- The branch should remain one additive published branch. Merge latest `main`
  before future pushes; avoid rebasing unless explicitly requested.

## How to Resume

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main
git log --oneline --decorate --max-count=12
```

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/background/lcp/02_overview.md`
- `docs/background/lcp/07_selection-guide.md`
- `docs/onboarding/python-bindings.md`
- `docs/onboarding/testing.md`

If `origin/main` is not an ancestor of `HEAD`, merge it before continuing:

```bash
git merge origin/main
```

For the next change, choose one bounded gap and keep the same verification
shape:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_types dartpy --parallel "$JOBS"
pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run lint
```

Also run `python/tests/unit/test_py_demo_panels.py` or relevant LCP benchmark
smoke rows if touching demo metadata, benchmark manifests, or generated
representative packets.
