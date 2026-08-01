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

## PR-3a soft-foot SIMBICON (merged 2026-08-01)

Revisions: #3408 as `fe9cb9ebd73b176794df2de7179f5d23f146cbe6`; #3423 as
`73cd91e69dba635cb17e93e60f33a2c245e629d0` (matched control rest inertia,
asset `<damp>` 1000 -> 4000 with recorded maintainer approval, and a
point-mass-aware SIMBICON COM sensor).

Commands: the registered ctest targets `test_SoftFootSimbiconModel`
(comparability, contact-spreading, finite-state gates) and
`test_SoftFootSimbiconPushSweep`
(`SoftFootSimbiconModelTest.MeasuresRecoverablePushForBothFeet`). Headless
reproduce (the #3423 evidence capture; swap `soft` for `rigid` for the
control arm):

```bash
DART_DEMO_SOFT_FOOT_FEET=soft DART_DEMO_SOFT_FOOT_PUSH_STEP=650 \
DART_DEMO_SOFT_FOOT_PUSH_N=6000 \
  ./build/default/cpp/Release/bin/dart-demos --scene soft_foot_simbicon \
  --headless --steps 2250 --shot end.png
```

Results: equal-mass arms, a single collision surface per foot, identical
rest tessellation, per-foot inertial equality to 1e-9, and an
independent-path observed-COM check; settled contacts 51.2 soft vs 15.64
rigid (gate at 1.5x); recoverable push 18000 N soft vs 8000 N rigid (gate:
soft >= rigid).

Review and visual evidence: Codex findings on #3423 (asset-vs-override
policy, controller COM sensor) were resolved at root cause; matched
before/after APNG strips and an endstate composite at 6000 N under the
corrected sensor are linked from the #3423 PR body (release tag
`verification-media-dart6-agent-evidence`).

## Open evidence

PLAN-622 still requires:

- gate evidence for the remaining Jain/Liu rows: the motor-noise push
  variant, noisy-floor biped, soft-contact walk, hand/arm manipulation, and
  the four-link flexible-foot comparison;
- the performance-acceptance evidence defined by the approved competitive
  envelope (`decisions.md` item 2, confirmed 2026-07-23);
- representative multicore scaling evidence or an approved negative
  disposition;
- the `dart` detector coverage, determinism, allocation, and same-host
  performance gates needed before any default proposal;
- a complete paired benchmark artifact or approved disposition; and
- verification for the separate `main` zero-DoF assertion fix.

Each new behavior-bearing packet must add its exact commands, revision,
results, review evidence, and visual oracle here before completion.
