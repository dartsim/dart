# Ten-Level Card House Current-Source Diagnosis

Status: active adapter and evidence lane. This document records what the
current public author source supports and prevents the ten-level DART demo from
being mislabeled as a recovered historical Tables 6-7 invocation.

## Claim Boundary

The pinned author commit is
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. Its card-house runner accepts an
arbitrary `--levels` value, so `--levels 10` is a public, source-supported
selection. The paper source tree does not provide the historical Tables 6-7
command, saved trajectory, renderer golden, or a corresponding video segment.

The current-source selection constructs 155 cards: 110 leaning cards and 45
bridges. It also constructs four initially kinematic cubes. Its relevant
defaults are:

- friction `mu=0.8`;
- card half extents `(1.25, 0.625, 0.02) m` and density `200 kg/m^3`;
- card lean `65 deg` from horizontal;
- cube half extent `0.4 m`, density `500 kg/m^3`, and drop height `1.0 m`;
- display step `1/60 s`, four substeps, and solver step `1/240 s`;
- 800 display frames and release at display frame 400;
- card/cube shape gap `0.005 m`, while the ground inherits Newton's
  `ModelBuilder.rigid_gap=0.1 m`; and
- exact-FBF contact capacity 4,096.

Direct inspection of the pinned builder resolves 160 shape-gap values: one
ground shape at `0.1 m` and 159 card/cube shapes at `0.005 m`. Newton sums the
two per-shape values, so the source detection thresholds are `0.010 m` for a
card/card pair and `0.105 m` for a card/ground pair. Cards and cubes also set
`ke=1e4` and `kd=1e3`; the ground inherits Newton's separate defaults.

The DART lane represents those numeric gap values through DART's Native
collision frontend and exact/boxed constraint solvers. Predictive checkpoint
`3647959a188` additionally allows an admitted separated contact to
close by its physical separation divided by `dt` before generating an impulse.
That matches the source's scalar separation-over-one-step velocity allowance
narrowly. It does not make DART's signed Native contact-gap approximation
equivalent to the Warp/Newton collision backend, contact selection/manifolds,
float32 arithmetic, or compliant stiffness/damping semantics. The adapter
therefore records that the values are represented while keeping broad source
gap semantics false. Exact and boxed results compare DART solver lanes under
one adapter; they do not prove paper, backend, trajectory, timing, renderer, or
solver-superiority parity.

## Pinned Source Controls

Both controls used the pinned source checkout, CPU device, exact FBF, ten
levels, and a release frame at the endpoint, so neither control released its
cubes.

### One display frame

Invocation:

```text
python -u paper_examples/card-house/run.py \
  --solvers fbf --levels 10 --frames 1 --drop-frame 1 --device cpu
```

The four substeps all reported convergence with 424 contacts, zero outer
iterations, and zero final residual. The top-card height changed by
`0.0017032623291015625 m`. This is a construction and first-step control only.

Retained local source directory:
`/tmp/fbf-card10-source.fOmff2/source/paper_examples/card-house/results/20260722T030931Z/`.

SHA-256:

- `fbf/history.json`:
  `d3ec8aa4f42b7811ec3f9fe7d0423ef54575ae9dafa2a843de94f5fd17e5d99e`;
- `fbf/result.json`:
  `5dac4343f275865c947afe6a51e7b2660b9a7bb79c4fa67afb4f1f5f936ab093`;
- `fbf/trajectory.npz`:
  `f9d71477761bc6b0b18b15e8f34ad2356d95cdf53a3e38e06958e1d193894dd8`;
- `metadata.json`:
  `b6c5704255757f9a3a89806ea5cb7cb5af60f2b68d8dd321ce38dcc4920e9169`;
- `sweep_results.json`:
  `f38be579a43dd0ec217a9d1293e8cf62f1bc87a034d4e3fae9329e03e816593d`.

### Thirty display frames

Invocation:

```text
python -u paper_examples/card-house/run.py \
  --solvers fbf --levels 10 --frames 30 --drop-frame 30 --device cpu
```

This 120-substep prefix completed and retained finite trajectory arrays. It
reported 33 converged and 87 non-converged substeps. The first non-converged
entry is zero-based `step_idx=33`, with 460 contacts, 60 outer iterations, and
final residual `0.3214742944001398`. Contact counts range from 421 to 870. The
last substep has 864 contacts, 50 outer iterations, and residual
`0.004141973671775864`. The top-card height changes by
`0.638458251953125 m`, from `22.9577579498291 m` to
`22.319299697875977 m`.

The source runner continues after `converged=false`. Consequently, this finite
prefix is continuation evidence, not a strict convergence golden or a DART
trajectory oracle.

Retained local source directory:
`/tmp/fbf-card10-source.fOmff2/source/paper_examples/card-house/results/20260722T031016Z/`.

SHA-256:

- `fbf/history.json`:
  `295d48fbf6083c1e6f04146fab2c3bc28839f8947a039a8b5b83637a770412da`;
- `fbf/result.json`:
  `35aaa5073851fc2bb97292d675f65a024108f27e291f2729a75b613389f804d6`;
- `fbf/trajectory.npz`:
  `a17dc64d628964eb998735f97bff343847b61c491042dba397d7fda36341715a`;
- `metadata.json`:
  `65bd6c94fa76d735c343c84b2aff8fa97646365a3d93f53ccfb05ab27231d9d8`;
- `sweep_results.json`:
  `b262134c13320d16500c9c632bd8610ae22c1a19266916f36476ee5c6ccd265f`.

## DART Gate

The separate DART scene and capture schedule must fail closed on solver
identity, source contract, shape-gap coverage, release step, capacity, and
timeline. Before a full exact/boxed video capture, the bounded gate is:

1. build and focused construction/contract tests;
2. collision-only contact-count audit at capacities 4,096 and 8,192;
3. one strict exact substep and one boxed substep; and
4. a short strict prefix with finite-state and residual telemetry.

Record a full video only if those gates justify the cost. A completed visual
continuation may be attached to the draft PR with explicit cap/failure
telemetry, but it cannot satisfy the strict zero-failure completion rule or be
called a Tables 6-7 reproduction.

### Pushed checkpoint result (historical)

The registered-scene result pushed at `ffe23d347b0` remains valid historical
checkpoint evidence. The formatted Release build, three focused ten-level C++
fixtures, `pixi run lint-cpp`, and `pixi run check-lint-cpp` passed. Under that
checkpoint, the strict exact lane failed closed at completed DART step 1
(`t=1/240 s`):

- 264 contacts are present in the final collision result, well below the
  configured 4,096-contact capacity;
- the solver records 18 exact attempts, 17 exact solves, one exact failure,
  zero accepted caps, and zero boxed fallbacks;
- the retained failure is a 39-contact group with build status `success`, FBF
  status `max_iterations`, 200 outer iterations, residual
  `8.891154359157548e-6`, and best residual `8.727149191711674e-6`, against the
  declared `1e-6` tolerance; and
- fail-fast reports `exact_failure` at completed step 1 even though a later
  contact group makes the solver's final per-group status `success`.

The same registered scene and Native gap frontend, switched to boxed LCP with
the demo's `e` action, complete step 1 and produce a 640x480 PNG. This is only a
one-step control; it establishes neither a full boxed trajectory nor a physical
outcome. The 4,096/8,192 capacity A/B and longer strict prefix are not promoted
as gates because strict exact already fails with substantial unused capacity.

Local generated evidence remains outside Git:

- exact timeline:
  `/tmp/card10-exact.wUN2ei/timeline.json`, SHA-256
  `c0af3c2b03d38b68bd30374394bebb83286b08e91414641796f8ff58ec202bbf`;
- boxed timeline:
  `/tmp/card10-boxed.Q4gt2g/timeline.json`, SHA-256
  `059d8d8c21db86df9b8708cf0da9b8bd63e024f2a9723d5a491c0bee2d3e78b0`;
- boxed step-1 PNG:
  `/tmp/card10-boxed.Q4gt2g/final.png`, SHA-256
  `3573bf9ce4fba68d46ddb4dc16967d09057918a6a6346549b7f31ada7c98c3bf`.

No ten-level video was justified by that strict gate. The checkpoint result is
a precise historical DART solver blocker, not a Tables 6-7 reproduction,
exact-versus-boxed superiority result, source trajectory match, or
paper-parity artifact.

### Predictive checkpoint

Commit `3647959a188` follows previous checkpoint `ffe23d347b0`. It keeps the
same gap activation and physical negative separation, but subtracts
`separation / dt`
from the velocity-phase contact right-hand side. Penetration correction remains
zero for separated contacts. This is a predictive speculative-contact
allowance, not Baumgarte correction or a compliant-contact port.

The corrected exact one-step gate now clears completed step 1 with 264 final
contacts, 18 exact attempts/solves, zero exact failures, zero accepted caps,
and zero boxed fallbacks. The paired boxed one-step control also completes.

A strict 40-step exact prefix then reaches completed step 31 before failing
closed on a 79-contact island. That island reaches 200 outer iterations with
residual and best residual `1.072805023427092e-5`, against the declared
`1e-6` tolerance. The saved final diagnostics record 558 exact attempts, 557
solves, one exact failure, zero accepted caps, zero boxed fallbacks, 540 warm
starts, and 304 final contacts. The paired boxed lane completes all 40 steps.

Final scoped local artifacts, all outside Git:

- corrected exact step-1 timeline:
  `/tmp/card10-predictive-scoped-exact1.0VwT5s/timeline.json`, SHA-256
  `bb1c352a3a2e35b7ee0796899dfbffb59358790fe8871cc8a936cbf62404066f`;
- corrected boxed step-1 timeline:
  `/tmp/card10-predictive-scoped-boxed1.Kye74m/timeline.json`, SHA-256
  `8947284c4719212722a67ef920d9e5e60892ae1ecf5195f4312703cb2061fbc7`;
- corrected exact 40-step request timeline:
  `/tmp/card10-predictive-scoped-exact40.YMlQ9q/timeline.json`, SHA-256
  `8154d5e4eeeec934e717717f9381dd1e5f300f2691702143881a6bcf047a2495`;
- corrected boxed 40-step timeline:
  `/tmp/card10-predictive-scoped-boxed40.JE0tj6/timeline.json`, SHA-256
  `b08bedc459bd0ea946c9b7a0bedf8030215c847b3377ebc3e127861ca5096b94`.

Final local gates for `3647959a188` pass:

- `ConstraintSolver`: 66/66;
- Native collision detector: 50/50;
- `SplitImpulse`: 13/13;
- exact solver: 38/38;
- paper fixtures: 36 passed with 3 explicit opt-in skips;
- visual runner: 332/332; and
- independent post-fix re-review: `ALLOW`.

The source row comparable to DART completed step 31 is zero-based
`step_idx=30`: it uses 422 contacts in one global workspace and converges at a
Coulomb-relative residual of `7.59e-7`. DART completed step 31 has 304 contacts
split across 18 islands and fails one 79-contact group. These are not the same
operator or contact problem. The source also uses colored BGS and checks inner
convergence every five sweeps; the current DART strict lane is sequential and
runs a fixed ten sweeps. The source's first later `converged=false` row remains
`step_idx=33`, with 460 contacts and continuation after failure.

### Source-continuation final exact member

The separately named scene and schedule
`fbf_author_card_house_10_impact_source_continuation_current_source` and
`card_house_author_10_impact_source_continuation_current_source` preserve the
same 155-card/four-cube geometry, Native `FourPointPlanar` frontend,
3,200-step clock, and step-1,600 `p` release. Only the exact lane requests the
explicit source-continuation policy. This lane does not alter or satisfy the
strict gate above.

The final exact reseal passes all 3,200 requested substeps. Timeline validation
passes with 3,201 represented steps, 401 captured/unique frames, and one
successful release action at step 1,600. The exact lane records:

- 7,702 exact attempts and 7,702 exact solves;
- zero exact failures and zero boxed-LCP fallbacks;
- 2,427 plateau accepts and 763 max-iteration accepts;
- zero line-search shrink caps, 310,880 total iterations, and 7,630 warm
  starts;
- 1,071 maximum observed contacts and 987 final contacts; and
- final residual `7.709159985211234e-8`, with trajectory-wide worst residual
  `0.59964511064890469`.

The decoded 660x506 H.264/yuv420p clip has 401 frames at 30 fps and duration
13.366667 s; full decode passes. Final panel/keyframe inspection confirms that
the full house is legible, the cubes release, post-release structural evolution
is visible, and lower structure remains at the endpoint. That inspection is
qualitative review only. The metadata deliberately records
`paper_comparable=false` and
`automated_semantic_outcome_validated=false`; there is no machine physical-
outcome oracle or source trajectory comparison.

The final local artifacts remain ignored outside Git under
`assets/pr_media_final/card_house_author_10_impact_source_continuation_current_source/`:

- timeline SHA-256:
  `7a4b7d878f73068e10c59073b8e1260444a02529db62ab42eaf5c46425a190ae`;
- clip SHA-256:
  `19637c4255c890f1f32383e7e7e680169688e5d8b071168bc6b4ffdebf33061d`;
- panel SHA-256:
  `e5ed0d63ca9818292c5a373f476f2841f280f3e01492e0065b2aec8eb95a74d6`;
- metadata SHA-256:
  `223e828a5284f9fc6aad0b7f57ef010d58db004d85759d036f47883b3753ed9f`.

The separate run summary `/tmp/card10_exact_final_summary.json` has SHA-256
`9a551a96176e5112fc9f1443586c8aee115e1c25c10f766d0088efe4a088e3b2`
and reports `pass=true`. Independent reuse verification also passes in
352.27 s. Its separate summary `/tmp/card10_exact_final_verify.json` has
SHA-256
`83f9e9db5d013ab8359d5ee5dfb2d05fb4a116082d090b168ec02708ea5a348e`;
it records kind `verification`, one result, no skips or groups, full-decode
success, and the matching metadata hash. The role-separated
`/tmp/ten-cont-final-review-verify.json` is byte-identical at the same SHA-256.
The exact schedule's blockers are empty only within this narrow continuation-
evidence boundary, while the boxed
runtime blocker remains explicit.

A clean boxed control completes all 80 requested steps in about 4 minutes 46
seconds with `BoxedLcpConstraintSolver`. Its timeline SHA-256 is
`ccbdc322791a06d5a8858818acae63e8540ca7770e635545e3c017d84bf96d7d`.
This control is a valid bounded solver-identity/completion result, not a
full trajectory or outcome.

The corresponding full boxed capture did not complete. After approximately
the same wall-time budget as the full exact capture (about 12 minutes 35
seconds), boxed LCP had reached only step 112 of 3,200. The process was
interrupted; it emitted no completed timeline sidecar. Its partial frames are
non-evidence and must not be reused for a paired clip or outcome claim. The
full boxed trajectory, physical outcome, and paired media therefore remain
blocked. This bounded runtime observation does not establish solver
superiority or a general performance result.

Next, resolve the full boxed-runtime gate before producing paired media. Upload
the final exact clip only through the PR browser composer and record the
resulting
`github.com/user-attachments/...` URL. Separately continue the
one-factor colored-scheduling and global-scope diagnostics. Do not loosen
tolerance, iteration caps, fallback, fail-fast, or accepted-cap policy to
force a strict pass. Neither this continuation run nor the corrected strict
prefix establishes source, trajectory, physical, Tables 6-7, paper, or
solver-superiority parity.

## Sibling Figure 7 Current-Head Control Boundary

Keep the unrelated literal 25-stone standing recapture out of this ten-level
claim. Its ignored root is `assets/pr_media_current_head_fig07/`; capture and
independent reuse verification pass for two 600-step members plus their group.
The external summary SHA-256 values are
`5a1de1f915d75c373f06aeb48978b92a540bc245427c55584f68ad178ea491bb` and
`e4b3d44d5f2afebef9f79bcb92b38ee282f5635c38fa4be2f4264a5f961acce5`.

The exact member records 600/600 exact solves, 96 contacts, zero accepted
caps/failures/fallbacks, final/worst residual
`9.778093504499096e-7` / `9.999807145410957e-7`, 599 warm starts, and 7,933
iterations. Its timeline/clip/panel/metadata hashes are
`6041addd27a79a747cdbcdaafb495f787d4e90a906ec0616aa16e5a33d9c9b74`,
`24c110421572500bec9f43a431061ae5a386e7e59940e86030e3059cc90d9676`,
`cd3498c90fd549365dadc9cf96908c4c3d86da81cbd5dd64ff4d891407b4ee6b`,
and `614704cc1ed70065d81b789c019e618ac54d7013f706b26a346ea236ef876802`.
The boxed member completes 600/600 with `BoxedLcpConstraintSolver`; its hashes
are `809ca91a475fdd0ebe3ad6b5cba73115c9ec2b3dd4a98e478beca229abf62321`,
`80e79fa6b356f951e9615dd94aad2de2f55d0bbab07d7b116cb07c1b3bef686c`,
`078990e4d7a950ba9e207102624d651d948f546f1153bf85077e8084c01b040a`,
and `944a6636bd6febb79ab9d6abda0a92165acd34ab3aa96b42f55f1c36731e6d45`.
The grouped clip/panel/metadata hashes are
`89c4d7372f68c6c9ad1a5d0e0e0388ffa1f198c2446e04fe30b9bc66325d8f9e`,
`5ce6efcebcb5f6a2385f6aea6de7933cccfa7446979b7ea0e46ef2aab5199633`,
and `5cc2513eb16db191454b27126783df70adbe3ad2617d3faa3f51597ab43966bb`.

Manual inspection finds both arches visibly standing, but every metadata record
sets `automated_semantic_outcome_validated=false`. This is no-projectile
capture/integrity and exact-telemetry evidence only, not an automated physical
oracle, source/paper trajectory or outcome, timing, solver superiority,
crown-impact result, Fig. 7 parity, or evidence about the ten-level card house.
The separate crown-impact adapter remains blocked at strict step 142, and the
group clip still needs manual browser-composer upload plus a recorded GitHub
user-attachment URL.
