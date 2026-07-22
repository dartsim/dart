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

The new DART lane may represent those numeric gap values through DART's Native
collision frontend and exact/boxed constraint solvers. DART's signed Native
contact-gap approximation is not equivalent to the Warp/Newton collision
backend or its compliant stiffness/damping semantics. The adapter therefore
records that the values are represented while keeping broad source gap
semantics false. Exact and boxed results compare DART solver lanes under one
adapter; they do not prove paper, backend, trajectory, timing, renderer, or
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

### Registered-scene result

The current registered-scene gate stops at item 3. The formatted Release build,
three focused ten-level C++ fixtures, `pixi run lint-cpp`, and
`pixi run check-lint-cpp` pass. The strict exact lane then fails closed at
completed DART step 1 (`t=1/240 s`):

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

No ten-level video is justified by this strict gate. The current result is a
precise DART solver blocker, not a Tables 6-7 reproduction, exact-versus-boxed
superiority result, source trajectory match, or paper-parity artifact.
