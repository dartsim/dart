# Verification - DART 6 deformable body feature and performance

Updated: 2026-07-30

## Accepted merged slice

PR #3382 merged into `release-6.20` as
`6c88ac1d774a702b494643fb598be6b8af9385e1`.

The final hosted matrix completed successfully across:

- Linux Release, Debug, assertions, Eigen alignment, and AI infrastructure;
- Windows Release;
- macOS arm64 Release and Debug;
- FreeBSD reproduction;
- GCC and Clang toolchain jobs;
- scalar, SSE4.2, AVX, and AVX2 SIMD jobs;
- gz-physics and gz-sim;
- API documentation and English/Korean Read the Docs; and
- Codecov patch and project gates.

The accepted local evidence recorded by the merged PR includes:

- `test_SoftDynamics`;
- the full Release C++ suite;
- detector, collision, constraint, allocation, and ABI-focused tests;
- exact-v6.19.4-header compatibility canaries;
- gz-physics functional and performance checks; and
- the selected gz-sim headless integration smoke.

PR #3381 subsequently established the single built-in `dart` detector while
preserving FCL as the DART 6.20 default. PR #3407 removed the out-of-scope
volumetric-FEM subsystem.

## Evidence boundary

The merged PR and its hosted checks own the historical exact-head evidence.
Temporary local captures and interrupted benchmark directories are not durable
evidence. Do not infer a complete paired benchmark result without the runner's
completion marker and all required raw rows.

## Open evidence

PLAN-622 still requires:

- a same-controller rigid-foot versus deformable-foot comparison;
- a complete competitive-envelope decision;
- representative multicore scaling evidence or an approved negative
  disposition;
- the `dart` detector coverage, determinism, allocation, and same-host
  performance gates needed before any default proposal;
- a complete paired benchmark artifact or approved disposition; and
- verification for the separate `main` zero-DoF assertion fix.

Each new behavior-bearing packet must add its exact commands, revision,
results, review evidence, and visual oracle here before completion.
