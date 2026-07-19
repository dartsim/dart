# Card-House Native Manifold Sensitivity v1

## Status and claim boundary

This protocol was frozen before either authoritative child was run. It tests
one reconstructed-scene factor: Native `Compact` versus `FourPointPlanar`
contact manifolds for the same 26-card `paper_cpu` trajectory request.

This is diagnostic reconstruction-sensitivity evidence only. It is not a
paper-parity experiment, an author-scene reproduction, a valid timing row, or
a causal claim about the unpublished implementation. A provenance-complete
comparison remains scientifically useful even if one or both exact
trajectories fail closed.

## Preregistered question and hypothesis

The current strict card prefix shows slow dual/complementarity convergence,
accepted outer caps, contact-graph churn, and a terminal exact failure. The
question is whether Native contact-manifold multiplicity materially changes
that behavior when every solver and scene input is fixed.

The directional hypothesis is that `FourPointPlanar` will retain more contacts
per colliding pair than `Compact` and will not improve the strict convergence
trajectory. All outcomes, including no difference or improved convergence,
must be reported without retuning.

## Frozen common contract

- Scenario: `card_house_26_settle_projectile_full`.
- Requested trajectory: 600 steps at `dt=1/60 s`.
- Phase contract: 402 settle steps, then four reconstructed cube projectiles
  through step 600.
- Solver: exact FBF with `paper_cpu` parameters and Native collision.
- Precision: DART float64, explicitly not the paper's float32 backend.
- Simulation threads: one, pinned to logical CPU 8.
- Outer cap: 200; tolerance: `1e-6`; accepted capped solves remain above
  tolerance and are not strict successes.
- Inner solve: 10 fixed BGS sweeps, exact-metric local cone solve, one local
  iteration.
- Step size: automatic safe bound, adaptive updates, scale 1, relaxation 1,
  persistence enabled, recovery growth factor 1.05.
- Warm start: previous solution when contacts match.
- Split impulse: enabled by the scenario contract.
- Projected-gradient retry, dense residual polish, diagonal seed,
  matrix-free seed, and boxed-LCP fallback: disabled.
- Collision caps: 512 total contacts and 4 contacts per body pair.
- Sample stride: 1; trace scope: `performance`.
- Authoritative repetitions: one per mode, in fixed order `Compact` then
  `FourPointPlanar`.

The manifold selector is the only intended factor. It must be rejected outside
this exact scenario/solver/trace/step/thread/frontend/bootstrap contract.

## Frozen child commands

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

## Opt-in trace contract

The default 83-column trace must remain byte-identical. Only an explicit
manifold-sensitivity selector appends these ten columns:

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

The actual mode must be read back from the installed Native detector. Pair
labels must be stable and sorted. Multiplicities must be positive and sum to
the row contact count. The explicit sensitivity schema therefore has exactly
94 columns.

## Fail-closed artifact gates

- Use a fresh output directory.
- Bind the protocol contract, runner, trace source/executable, `ldd`,
  `taskset`, normalized resolution map, and every resolved regular shared
  library before execution and recheck them after each child.
- Require the exact 94-column header and exact frozen commands.
- Require requested and installed manifold modes to agree on every row.
- Require collision caps 512/4 on every row.
- Require every common solver/scene configuration field to match the frozen
  contract and each other across modes.
- Require rows numbered contiguously from 1 with no gaps or duplicates.
- Exit 0 requires exactly 600 rows. Exit 1 is admissible only as a scientific
  failed prefix whose terminal row records exactly one aggregate exact failure
  and after which no row exists. Reject timeouts, signals, other exits, or
  unexplained short output.
- Every contact step must contain at least one exact attempt and zero fallbacks.
  Record the per-step attempt count because disconnected constrained groups can
  legitimately produce more than one exact solve.
- The aggregate per-step accepted-cap and failure counters control strict-step
  classification. The wrapper status and public internal FBF
  status/best-iterate fields describe the last exact group only when a step
  contains multiple groups; record them but do not promote them to a
  whole-step subtype.
- Required state, residual, penetration, gamma, and physical fields must be
  finite wherever the schema contract requires them.
- A strict trajectory passes only with 600 rows, zero aggregate accepted caps,
  zero failures/fallbacks, and no last-group residual above `1e-6`.
- Physical-outcome and timing verdicts remain null for failed or
  above-tolerance trajectories. Wall time is retained as diagnostic metadata
  and excluded from the manifold-effect verdict.

## Frozen primary endpoints

Report each mode and `FourPointPlanar - Compact` deltas for:

- emitted prefix length and first failed step;
- strict-success, accepted-cap-row, exact-attempt, exact-failure, and fallback
  counts;
- wrapper/internal terminal status, outer/best iteration, and residual;
- primal, dual, and complementarity residual distributions;
- contact and unique-pair ranges;
- stable pair identities, per-pair multiplicity, and graph-transition counts;
- warm-start match counts/fractions and gamma persistence use; and
- finite-state and card-pose preservation metrics.

Raw wall time may be tabulated for transparency but cannot enter the causal,
real-time, paper, or performance verdict.

## No-retuning rule

Do not change the scene, collision caps, solver options, affinity, run order,
schema gates, or interpretation thresholds after observing either result. Any
later experiment requires a new protocol version and a new preregistered
hypothesis.

## Frozen v1 result

The authoritative attempt is retained at
`assets/paper_evidence/card_house_26_manifold_sensitivity_v1/` and is invalid
under this frozen protocol.

Both children emitted all 600 rows. `Compact` returned zero.
`FourPointPlanar` returned one with zero aggregate exact failures because the
trace executable's final convergence gate found its last-group residual above
`1e-6`. V1 admitted return one only for a terminal aggregate exact-failure
prefix, so the runner correctly rejected the comparison artifact rather than
reinterpreting the exit after observing it.

The invalid bytes may be used only to define a new preregistered exit class.
They do not support a v1 manifold-effect, timing, physical, or paper claim.
