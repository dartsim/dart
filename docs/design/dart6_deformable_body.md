# DART 6.20 deformable-body compatibility decisions

This document owns the technical and compatibility decisions that should
survive the deformable-body development task. It does not own current priority,
PR state, or next actions; those remain in `docs/plans/dashboard.md` and the
active task home until closeout.

## Compatibility contract

- `dart::dynamics::SoftBodyNode` remains the DART 6 public deformable-body
  type. Work on the release branch preserves public headers, ABI-sensitive
  class layouts, and existing virtual interfaces.
- Optional behavior uses additive non-virtual controls. New runtime state stays
  behind implementation-private storage rather than public data members.
- Existing behavior remains the default. In particular, adaptive contact
  activation and soft face-interior contact coverage are opt-in; their disabled
  paths must preserve the established simulation arithmetic and checksums.
- DART 6 deformable-body work is CPU-first. GPU solver or offload APIs belong
  to a clean-break development line rather than the 6.20 compatibility branch.
- Collision, constraint, or default-policy changes require the focused
  deformable tests plus the Gazebo/gz-physics compatibility gate before any
  default change.

## Dynamics decisions

The DART 6 point masses are internal soft-body coordinates, not public
`Skeleton` generalized coordinates. Public mass and augmented-mass matrices
therefore assemble each column from the corresponding skeleton-coordinate
basis acceleration and must not depend on a retained point-mass simulation
acceleration. Physical point-mass acceleration remains part of inverse
dynamics.

Adaptive activation treats inactive point masses at rest as rigidly lumped
mass and inertia on the parent body. Contact selection seeds a local active
neighborhood; points leave that neighborhood only after the configured
rest/linger policy. The feature remains opt-in so existing applications keep
the all-active path.

## Data layout and performance evidence

Retained per-phase structure-of-arrays mirrors are not the DART 6 direction:
measured prototypes lost more time copying phase data than they recovered from
contiguous reads. A future layout proposal must maintain its storage at the
producer or redesign ownership; it must not reintroduce unconditional copy
passes. Contiguous `PointMass` object storage also remains a redesign rather
than unfinished work because the public type has virtual lifetime semantics.

Performance claims must retain revision SHAs, the exact command, raw timing
rows, detector eligibility, and enough host state to interpret noise. A manual
timing disposition does not turn a machine-readable evaluator `FAIL` into a
reproducible pass. Use a balanced or paired order when comparing backends that
share most of their kernels.

Normalized paper comparisons record model size, step size, simulated duration,
contact counts, deterministic final-state metrics, and same-host CPU results at
one thread and a host-capped multi-thread setting. Collision-dependent rows
compare only checksum-eligible backends. A reduced representative demo proves a
mechanism; it does not by itself establish paper-scale model, controller, or
performance parity.

## DART soft collision direction

The built-in `dart` detector owns the DART soft-contact kernels, cached soft
geometry, and broadphase. There is no second public detector or fallback bridge
for the same engine.

The `dart` detector must not become the preferred deformable backend until all
of these are true:

- representative soft scenes pass finite-state, physical-regression, and
  thread-determinism tests;
- DART and incumbent-backend checksums are stable, and comparisons classify
  only checksum-eligible backends as performance competitors;
- DART matches or beats eligible backends on same-host representative and
  contact-heavy scenes with reproducible raw evidence;
- steady-state allocation gates cover the representative soft scene set;
- representative scenes emit no unsupported-pair diagnostics; and
- downstream Gazebo/gz-physics collision and constraint gates pass.

The remaining architecture choice is whether DART-owned coverage uses full
triangle-mesh collision, adaptive active-contact neighborhoods, or both. That
choice stays open until a bounded follow-up proves coverage and scaling.

### DART-owned soft-kernel follow-up contract

The detector directly owns the five implemented soft pair families: soft
against plane, sphere, box, ellipsoid, and soft. Unsupported pair families keep
their current no-contact behavior until their own kernels and correctness
evidence exist. Extend coverage per pair family rather than by globally
changing soft-shape dispatch.

The DART collision object owns one retained deforming-geometry cache; pair
kernels must not maintain parallel mirrors. Each pair preserves object order,
contact point, normal, depth, soft-side face IDs, established non-finite-bounds
behavior, and the full configured per-pair contact budget at both generation
and emission. The rigid three-contact generation clamp must not truncate soft
contacts before emission. Cache access uses the canonical local vertex formula,
point position plus resting offset. A missing or mismatched cache view fails
loudly rather than falling back to `getLocalPosition()`, which does not provide
the same vertex formula. Soft contacts bypass the rigid persistent-manifold
cache because deforming local points violate its rigid-transform assumption;
rigid-rigid neighbors remain cache-eligible.

Land further work in independently gated stages:

1. extend unsupported pair families separately with parity tests and explicit
   no-contact guards where DART 6 previously omitted contacts;
2. add deterministic per-pair threading only after serial parity is proven;
3. change broadphase structure only if measured attribution still shows it is
   needed; and
4. keep additional checksum-changing contact coverage opt-in until its own
   reference tests and downstream evidence justify a compatibility decision.

Every parity stage requires both object orderings, contact-field comparison,
single- and multi-thread checksum evidence, zero steady-state allocation, and
manifold-cache ownership tests. The completed DART path must then pass the
paired same-host detector protocol, representative allocation and physical
regressions, and Gazebo/gz-physics gates before any preferred/default-backend
proposal. Measurement remains the gate for dispatch changes rather than an
assumed speedup.

## Paper-scope decisions

The DART 6 point-mass surface model is close to the Jain/Liu contact model but
is not the reduced volumetric FEM model used by Kim and Pollard. The approved
release-slice deferrals are:

- Kim/Pollard paper-scale volumetric-FEM characters: Fatman, starfish
  (including obstacle escape), fish, and worm;
- Jain/Liu SIMBICON/controller rows: biped push recovery, noisy floor, and
  biped walking; and
- Jain/Liu hand scenes: finger flick, arm fold, and pinch grasp.

The `soft_worm` and `adaptive_soft_contact` `dart-demos` scenes are
representative reduced evidence. They do not establish full paper-scale parity.
The Jain/Liu four-link flexible-rigid-foot versus deformable-foot comparison is
not in the approved deferral list and remains an explicit roadmap decision.

## Related owners

- Paper models and published metrics:
  `docs/background/deformable_body_paper_targets.md`
- Current roadmap state and gates: `docs/plans/dashboard.md`, PLAN-622
- Active implementation and PR evidence:
  `docs/dev_tasks/dart6_deformable_body_performance/`
- User-visible demos: `examples/demos/scenes/AdaptiveSoftContactScene.cpp` and
  `examples/demos/scenes/SoftWormScene.cpp`
