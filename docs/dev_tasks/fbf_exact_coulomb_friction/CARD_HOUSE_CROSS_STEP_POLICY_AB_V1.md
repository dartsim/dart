# Card-house cross-step policy A/B v1

Status: frozen before either evidence child is executed.

This protocol asks one narrow engineering question: on DART's reconstructed
26-card scene, does the compound cross-step policy inspired by the public
author implementation produce a strict 90-step settle prefix where the current
DART policy does not? It does not test a paper-equivalent scene, reproduce the
author runtime, or isolate the causal effect of any one policy component.

## Source and claim boundary

The source inspiration is the MIT-licensed `matthcsong/fbf-sca-2026`
repository pinned at commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. Relevant implementation surfaces
are `fbf_solver/config.py`, `fbf_solver/gamma_controller.py`,
`fbf_solver/warm_start.py`, `fbf_solver/solver_fbf.py`, and
`paper_examples/card-house/run.py`.

The inspired arm deliberately retains these known non-equivalences:

- DART uses float64; the author implementation uses Warp/Newton float32.
- DART solves independently grouped constraint islands and matches gamma per
  group; the author implementation uses a global contact graph, cache, and
  gamma controller.
- DART uses its reconstructed 26-card, four-level scene at `dt=1/60`; the
  current public author card-house example has five levels, 40 cards, four
  cubes, and four `dt=1/240` substeps per displayed frame.
- DART uses Native Compact contact manifolds with 512 total and four per-pair
  contact caps. This is not the author's Newton collision stream.
- At the final backtrack limit, the author controller accepts its newly
  shrunken gamma without re-solving at that value. DART retains its fail-closed
  rejection behavior.
- DART's reported `residual` and `worst_exact_residual_to_date` are its scaled
  convergence metric. The warm-gamma cap and unconverged-cache decision use the
  separate unscaled natural-map norm
  `||lambda - Pi_K(lambda - v_tilde(lambda))||_2`.

The only admissible conclusion is a source-informed DART policy comparison.
No result may be labeled source-equivalent, paper-equivalent, or causal for an
individual matcher, cache, or gamma choice.

## Frozen execution contract

Both arms use one fresh process, in this order:

1. `dart_current`
2. `author_policy_inspired_b3f3c5c`

The exact command template is:

```text
taskset --cpu-list 8 <fbf_paper_trace> \
  card_house_26_settle_projectile_full exact_fbf 1 90 nan performance \
  default default 1 paper_cpu native default 0 0 default <policy>
```

The common contract is fixed as follows:

- exactly 90 simulation steps at `dt=1/60`, with one CSV row per step;
- the first 90 steps of the 402-step settle phase, so all rows have 26 cards
  and zero projectiles;
- Linux logical CPU 8, one requested and one actual simulation thread;
- DART float64 exact FBF, Native Compact collision, 512 total contacts, four
  contacts per pair, and split impulse enabled;
- `paper_cpu`: 200 outer iterations, tolerance `1e-6`, accepted capped iterates
  enabled in the executable but rejected by the strict evidence gate, ten
  fixed exact-metric block-GS sweeps, one local iteration, automatic gamma,
  adaptive stepping, and cross-step warm start/persistence enabled;
- projected-gradient retry, dense residual polish, diagonal seed,
  matrix-free seed, and boxed-LCP fallback disabled; and
- no warmup, repetition, initial-gamma override, local-solver override,
  bootstrap, post-bootstrap, or manifold-sensitivity override.

The executable rejects either explicit policy unless every command token above
matches. Omitting argv16 or passing `default` must preserve the historical
83-column performance schema byte-for-byte. The historical explicit manifold
selector must preserve its 94-column schema byte-for-byte.

## Arm A: current DART policy

The `dart_current` arm freezes the current defaults:

- either-body-local feature matching with reversed body-pair support;
- inclusive match distance `<= 0.025` m and local-normal cosine `>= 0.9`;
- world-space fallback when a body-local feature is unavailable;
- no cache-age eviction;
- fresh and persisted safe-bound scale `1`, no explicit gamma min/max range,
  no natural-residual warm gamma cap, and recovery growth `1.05`;
- accepted unconverged reactions retain the current cache behavior; and
- coupling threshold `0.9`, shrink factor `0.7`, and at most 20 shrink
  attempts.

## Arm B: author-policy-inspired compound

The `author_policy_inspired_b3f3c5c` arm changes the following together:

- ordered body-pair matching using only the body-B-local contact feature;
- greedy one-to-one matching with strict distance `< 0.02` m and local-normal
  cosine `>= 0.9`;
- cached impulses stored in body B's frame and rotated through its current
  pose;
- reaction-cache age three, while matched gamma identity may outlive an aged
  reaction;
- fresh and persisted safe-bound scale `10`, gamma range `[1e-6, 1e6]`, and
  cross-step recovery growth `1/0.7` after a rejection-free step;
- coupling threshold `0.9`, shrink factor `0.7`, and at most eight shrink
  attempts;
- if the initial natural-map residual is strictly below `1e-4`, cap the
  current gamma at `1e4` but persist its finite pre-cap value; and
- save an accepted unconverged reaction only when its final natural-map
  residual is strictly below its initial natural-map residual. Converged
  reactions are always eligible for the cache.

This is intentionally a compound arm. The run cannot attribute a result to any
single bullet.

## Additive trace contract

Only an explicit nondefault argv16 appends these 32 columns:

1. `cross_step_policy_contract`
2. `requested_cross_step_policy`
3. `actual_cross_step_policy`
4. `requested_native_contact_manifold_mode`
5. `actual_native_contact_manifold_mode`
6. `collision_max_contacts`
7. `collision_max_contacts_per_pair`
8. `step_exact_attempts`
9. `step_exact_max_iterations_accepted`
10. `step_warm_start_gamma_caps`
11. `step_unconverged_warm_start_cache_skips`
12. `worst_exact_residual_to_date`
13. `last_exact_diagnostics_contract`
14. `last_exact_initial_natural_map_residual`
15. `last_exact_final_natural_map_residual`
16. `last_exact_uncapped_initial_gamma`
17. `last_exact_warm_start_gamma_cap_applied`
18. `warm_start_match_mode`
19. `warm_start_match_distance`
20. `warm_start_normal_cosine`
21. `strict_warm_start_match_distance`
22. `warm_start_max_age`
23. `persistent_gamma_safe_bound_scale`
24. `minimum_adaptive_gamma`
25. `maximum_adaptive_gamma`
26. `warm_start_gamma_natural_residual_threshold`
27. `warm_start_gamma_cap`
28. `persist_uncapped_gamma_after_warm_cap`
29. `require_residual_improvement_for_unconverged_cache_save`
30. `coupling_variation_tolerance`
31. `shrink_factor`
32. `max_step_shrink_iterations`

Requested policy and Native mode are command-contract values. Actual policy is
classified from all relevant solver and sidecar readbacks; actual Native mode
is read from the installed detector. `step_exact_attempts`, accepted caps,
gamma caps, and cache skips are cumulative-counter deltas. Natural-map and
other last-solve diagnostics explicitly describe only the last exact group;
multi-group rows are labeled noncomparable for those fields.

## Structural validation and scientific negatives

The runner requires a fresh output directory and binds the frozen protocol,
runner, trace source, trace executable, identity-helper source, `taskset`,
`ldd`, normalized dependency-resolution map, and every resolved regular shared
library. It rechecks the complete execution identity after each child.

Every emitted row must have the exact unique schema; contiguous steps starting
at one; the frozen common configuration and requested/actual policy readbacks;
settle phase, 26 cards, zero projectiles, finite state; Native Compact and
512/4 caps; positive contacts and exact attempts; and finite required
diagnostics. Missing rows, malformed columns, identity drift, a timeout,
signal, or any exit other than zero or one invalidates the bundle.

Arm A must report NaN for its disabled minimum/maximum gamma,
natural-residual threshold, gamma cap, and last natural-map residual fields.
Those NaNs prove that the legacy arm did not add opt-in natural-map products;
they are not missing required diagnostics. Arm B must report its finite pinned
policy values and finite last natural-map residuals whenever a step attempted
an exact group. Both arms must report a finite uncapped initial gamma for an
attempted last group.

Exit zero is structurally valid only with exactly 90 rows. Exit one is a valid
scientific negative only as either:

1. an exact-failure prefix whose last row is the sole row with one or more
   aggregate exact failures and after which no row exists; or
2. all 90 rows, no aggregate exact failures, and a terminal last-group scaled
   residual above `1e-6`, matching the executable's terminal gate.

The runner always attempts both frozen arms in order. A structurally valid
negative is sealed and reported; it is not deleted or adjusted after the run.

## Strict gate and 600-step promotion

An arm is strict only if all of these hold:

- exit zero and exactly steps 1 through 90;
- every row is finite, in the settle phase, with 26 cards, zero projectiles,
  positive contacts, and at least one exact attempt;
- `step_exact_attempts == step_exact_solves` on every row;
- zero aggregate exact failures, boxed fallbacks, and accepted outer-iteration
  caps on every row;
- every cumulative `worst_exact_residual_to_date` is finite and no greater
  than `1e-6`; and
- every common configuration, collision, actual-thread, requested-policy,
  actual-policy, and execution-identity readback matches the frozen contract.

Warm-start gamma caps and unconverged-cache skips are intended policy events;
they are recorded but are not required to be zero.

Only a strict `author_policy_inspired_b3f3c5c` arm may set
`promotion_to_separately_preregistered_600=true`. Promotion authorizes writing
a new 600-step preregistration; it does not authorize this runner to start a
600-step child automatically. A strict or non-strict Arm A never promotes.

## Frozen endpoints

For each arm, report the exit class, emitted rows, first failing step, strict
gate, attempts/solves/failures/fallbacks/caps, scaled residual distribution and
whole-run worst, natural-map last-group diagnostics, gamma caps, cache skips,
warm-match fraction, gamma persistence, contact range, finite/card-pose
metrics, and terminal state.

The comparison reports `author-policy-inspired - current` for deterministic
solver/contact/pose summaries. Raw wall time is retained as
`diagnostic_only`; it is excluded from strictness, comparison verdicts,
promotion, and every scientific claim.

## No-retuning rule

Do not change any scene, solver, collision, affinity, schema, command, order,
policy, threshold, exit class, or gate after observing either child. Any
correction requires v2 and a new output directory. Do not modify or replace
the finalized manifold-sensitivity v2 bundle.

## Frozen v1 result

Executed once on 2026-07-21 UTC from merged commit `2d1e98cceda` into the
ignored local directory
`assets/card_house_cross_step_policy_ab_v1_20260720/`. Both children ran in
the frozen order with stable executable and dependency identities. The packet
is structurally invalid and authorizes no promotion.

The `dart_current` arm is a valid 90-step scientific negative: all 90 exact
attempts were accepted at the 200-iteration cap, the whole-run worst scaled
residual was `30.0279709536632`, and the terminal residual was
`0.002833607911319349`. It had zero exact failures and zero boxed fallbacks,
but it did not pass the strict gate.

The author-inspired child also emitted 90 rows, but the v1 runner rejected it
at step 81 with `exact diagnostics contract drifted`. That step had two exact
groups. The executable correctly labeled both the legacy
`exact_diagnostics_contract` field and the additive
`last_exact_diagnostics_contract` field as
`last_exact_group_only_multi_group_noncomparable`; the v1 validator
incorrectly required the legacy single-group contact-row label on every row.
This is a validator-contract defect, not an admissible scientific result.

Per the no-retuning rule, v1 is not repaired or rerun. Any corrected parser
must be preregistered as v2, retain the frozen simulation commands and policy
parameters unchanged, use a new output directory, and bind the multi-group
label to `step_exact_attempts > 1`.

The frozen protocol-contract SHA-256 is
`a2babf6d28f802fce85d49abf4231c5ac94f3db62747f73a093cbba5ee3db4b5`.
