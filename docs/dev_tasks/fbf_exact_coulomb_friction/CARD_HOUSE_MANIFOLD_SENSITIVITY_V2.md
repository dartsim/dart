# Card-House Native Manifold Sensitivity v2

## Why v2 exists

V1 was rejected exactly as designed: both modes emitted 600 rows, but
`FourPointPlanar` returned one with no aggregate exact failure because the
trace executable's final convergence gate found its last-group residual above
`1e-6`. V1 admitted return one only for an exact-failure prefix. This v2
protocol was frozen after that invalid artifact and before rerunning either
child. It changes only the represented process-exit classes and related runner
validation; it does not change the scene, solver, manifold modes, run order, or
commands.

## Claim boundary and hypothesis

This is a one-factor sensitivity test of Native `Compact` versus
`FourPointPlanar` manifolds in the reconstructed 26-card scene. It is not paper
parity, author-scene evidence, timing evidence, or a reproduction of the
unpublished implementation.

The preregistered directional hypothesis remains that `FourPointPlanar` will
retain more contacts per pair than `Compact` and will not improve strict exact
convergence. All outcomes must be reported without retuning.

## Frozen common contract

- `card_house_26_settle_projectile_full`, 600 steps at `dt=1/60 s`.
- 402 settle steps, then four reconstructed cube projectiles through step 600.
- Exact FBF, `paper_cpu`, Native collision, DART float64.
- One simulation thread pinned to logical CPU 8.
- Outer cap 200; tolerance `1e-6`; 10 fixed inner sweeps.
- Exact-metric local solve with one local iteration.
- Adaptive safe-bound gamma, scale 1, relaxation 1, persistence enabled,
  recovery factor 1.05, and previous-step warm start.
- Split impulse enabled.
- Retry, dense polish, diagonal seed, matrix-free seed, and boxed fallback
  disabled.
- Collision caps fixed at 512 total and 4 per body pair.
- Sample stride 1; `performance` trace scope.
- One run per mode, fixed order `Compact` then `FourPointPlanar`.

## Frozen commands

```bash
taskset --cpu-list 8 \
  build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace \
  card_house_26_settle_projectile_full exact_fbf 1 600 nan performance \
  default default 1 paper_cpu native default 0 0 compact
```

```bash
taskset --cpu-list 8 \
  build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace \
  card_house_26_settle_projectile_full exact_fbf 1 600 nan performance \
  default default 1 paper_cpu native default 0 0 four_point_planar
```

## Frozen 94-column trace contract

The default 83-column schema remains byte-identical. The explicit selector
appends the v1 instrumentation columns:

1. `manifold_sensitivity_contract`
2. `requested_native_contact_manifold_mode`
3. `actual_native_contact_manifold_mode`
4. `collision_max_contacts`
5. `collision_max_contacts_per_pair`
6. `step_exact_max_iterations_accepted`
7. `step_internal_fbf_status`
8. `step_internal_fbf_best_iteration`
9. `step_internal_fbf_best_residual`
10. `colliding_body_pair_labels`
11. `contact_multiplicity_by_body_pair`

Actual mode is read from the installed detector. Pair labels and
multiplicities are stable, sorted, positive, and constrained by 512/4; their
counts must match the aggregate row fields.

## Fail-closed gates and exit classes

- Fresh output directory; exact commands and run order.
- Bind and recheck protocol, runner, trace source/executable, identity-helper
  source, `ldd`, `taskset`, normalized resolution map, and every resolved
  regular shared library after each child.
- Exact 94-column header, exact requested/installed mode agreement, 512/4
  configured and observed bounds, contiguous rows, finite required solver and
  physical fields, at least one exact attempt per contact step, and zero
  fallback.
- Aggregate per-step accepted-cap and exact-failure counters determine strict
  step classification. Wrapper/internal status and best-iterate fields describe
  the last exact group only on multi-group steps.
- Exit 0 is valid only with 600 rows and zero aggregate exact failures.
- Exit 1 has exactly two admissible classes:
  1. an exact-failure prefix whose final row records the sole aggregate exact
     failure and after which no row exists; or
  2. all 600 rows, zero aggregate exact failures, and a final last-group
     residual above `1e-6`, matching the executable's terminal convergence
     gate.
- Reject timeouts, signals, other exits, or any other short/full-output shape.
- A strict trajectory requires 600 rows, zero aggregate accepted caps,
  failures, and fallbacks, plus no last-group residual above `1e-6`.
- Physical and timing verdicts remain null for non-strict trajectories. Raw
  wall time is retained but excluded from the comparison verdict.

## Frozen endpoints

For each mode and `FourPointPlanar - Compact`, report:

- process-exit class, emitted rows, first aggregate exact-failure step;
- strict-success rows, aggregate accepted-cap rows/groups, exact attempts and
  failures;
- last-group terminal status, iteration, best iteration, residual, and best
  residual with explicit scope;
- residual-component distributions;
- contact/pair ranges, pair-identity union/differences, graph transitions, and
  multiplicity distributions/transitions;
- warm-start match count/fraction and gamma-persistence use;
- finite-state and card-pose metrics; and
- raw wall time labeled diagnostic-only.

## No-retuning rule

Do not change any scene, solver, collision, affinity, schema, command, order,
or acceptance setting after observing either v2 child. Any further correction
requires v3.

## Frozen v2 result

The preregistration wording above records the source-availability state at the
time the contract was frozen. A public author reference is now pinned, but this
reconstructed DART scene has not become source-scene evidence and the frozen
numerical protocol is unchanged.

The authoritative provenance-complete current-source bundle is
`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/`.
`comparison_artifact_integrity_valid=true`; neither trajectory is strict, and
both physical and timing verdicts remain null.

`Compact` emitted 600 rows and returned zero, but every row contained at least
one aggregate accepted capped group: 3,495 capped groups across 5,757 exact
attempts. It had zero exact failures and fallbacks, contact range 39-155,
pair range 30-57, mean multiplicity 1.6905709475, 538 pair-identity transition
rows, and 542 multiplicity-transition rows. Its terminal last group succeeded
at residual `8.525678738415048e-7`; that does not make the earlier capped groups
strict successes. Whole-run strict-success rows: zero.

`FourPointPlanar` emitted 600 rows and used the frozen
`complete_terminal_convergence_gate_failure` exit class: zero aggregate exact
failures/fallbacks, but terminal last-group residual
`0.016582575623909489`. It recorded 682 capped groups across 745 exact
attempts, contact range 124-200, pair range 35-54, mean multiplicity
3.6454258446, 140 pair-identity transition rows, and 166
multiplicity-transition rows. Whole-run strict-success rows: zero.

The preregistered directional contact-multiplicity hypothesis is supported:
`FourPointPlanar - Compact` mean contacts is `+93.7983333333` and mean
per-pair multiplicity is `+1.9548548971`. The convergence outcome is mixed but
does not support improvement: FourPointPlanar has 2,813 fewer aggregate capped
groups and fewer graph transitions, yet its terminal last-group residual is
`0.0165817230560` higher and it fails the executable's terminal convergence
gate. This is reconstruction sensitivity only; the raw wall-time difference
is excluded from every verdict.

The frozen protocol-contract SHA-256 is
`eeb1c8c1d09a29e197a3c402217ecd0af9a9878749cb4756cf94bc714f2b60e9`.
Current source and artifact SHA-256 values are:

- runner: `e03356c772560f061e9b90fb4cd9f5df0c569631cd5e9fdd0857c337ff840562`
- runner tests: `cb6a41ca80ac153c284a7110a3db26236b8d9005a95b9dd534add09306141584`
- trace-contract tests: `aed7b7bff883a315980ecf340f6b62d7d64fb9f4249f03935a93c9365d7e68cc`
- trace source: `b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`
- trace executable: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`
- `compact/raw.csv`: `48a4591e8d43a80e517606f0a5d9ccbd0f4350d85177f60932c1506068ea6be5`
- `four_point_planar/raw.csv`: `b006135f4f2d4694d47918cbf0cd05766d23ab0a8562fb6f57c04f2d41e925a3`
- both stderr files: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`
- `invocation.json`: `6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0`
- `summary.json`: `52a082ab15e8b9c314d706474cc7be557ddfc58c4961faad0d3da9d347f59f4f`
- `comparison.json`: `051605c25ccd5aa4de2f243c4dafe547c82f7298f8018cee53a8701d018ff297`
- `metadata.json`: `5890ab138179f4d7aaee6cd04c63799086439bae58fbde4f994048785dd0b8ac`
- `artifact-index.json`: `1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627`
- `REPORT.md`: `0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e`

The earlier v1 bundle is explicitly invalid because its frozen exit taxonomy
could not represent the full-duration FourPointPlanar convergence-gate exit.
No solver, scene, collision, affinity, or order parameter changed between v1
and v2.

The un-suffixed v2 bundle is retained as superseded history. The `_r2` bundle
reran the unchanged frozen protocol after current trace and Native-collision
source identities advanced; its scientific result and non-paper claim boundary
are unchanged.
