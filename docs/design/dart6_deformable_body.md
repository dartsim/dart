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

## Native soft collision direction

The DART and direct-native detector paths currently share DART-owned soft
contact kernels. Direct native keeps its own broadphase and bridges unsupported
soft/ellipsoid narrow-phase pairs through cached fallback objects. This is a
measured compatibility path, not evidence that native owns complete soft-mesh
collision.

Native must not become the preferred deformable backend until all of these are
true:

- representative soft scenes pass finite-state, physical-regression, and
  thread-determinism tests;
- direct-native and DART checksums are stable, and comparisons classify only
  checksum-eligible backends as performance competitors;
- native matches or beats eligible backends on same-host representative and
  contact-heavy scenes with reproducible raw evidence;
- steady-state allocation gates cover the representative soft scene set;
- representative scenes emit no unsupported-pair diagnostics; and
- downstream Gazebo/gz-physics collision and constraint gates pass.

The remaining architecture choice is whether native-owned coverage uses full
triangle-mesh collision, adaptive active-contact neighborhoods, or both. That
choice stays open until a bounded follow-up proves coverage and scaling; it is
not implied by the fallback bridge.

### Native-owned soft-kernel follow-up contract

The follow-up that removes the cached DART-object bridge must make native own
the five soft pair families that the bridge currently implements:
soft x plane, sphere, box, ellipsoid, and soft. Unsupported pair families must
keep their current no-contact behavior until their own kernels and correctness
evidence exist. The port is staged per pair family rather than by globally
flipping a soft-shape flag.

The native soft shape owns one retained deforming-geometry cache. Native and
still-bridged pairs must read that same cache during a staged rollout; they must
not maintain parallel mirrors. Each ported pair preserves object order, contact
point, normal, depth, soft-side face IDs, the full configured per-pair contact
budget, and the established non-finite-bounds behavior. Native soft contacts
bypass the rigid persistent-manifold cache because deforming local points
violate its rigid-transform assumption; this guard lands with the first pair
predicate change, while rigid-rigid neighbors remain cache-eligible.

Land the work in independently gated stages:

1. add the native soft shape/cache and bit-identical AABB refresh without
   changing dispatch;
2. port plane, sphere/box, soft-soft, and ellipsoid pairs separately, retaining
   the bridge for each pair until its native kernel passes parity;
3. add deterministic per-pair threading only after serial parity is proven;
4. change broadphase structure only if measured attribution still shows it is
   needed; and
5. add soft-face-interior coverage only as an opt-in, checksum-changing
   correctness extension with its own reference tests.

Every parity stage requires both object orderings, contact-field comparison,
single- and multi-thread checksum evidence, zero steady-state allocation, and
manifold-cache ownership tests. The completed native path must then pass the
paired same-host detector protocol, representative allocation and physical
regressions, and Gazebo/gz-physics gates before any preferred/default-backend
proposal. Profiling on the current bridge has not shown a structural native
penalty, so measurement remains the gate for dispatching this port rather than
an assumed speedup.

## Paper-scope decisions

The DART 6 point-mass surface model is close to the Jain/Liu contact model but
is not the reduced volumetric FEM model used by Kim and Pollard. The approved
release-slice deferrals are:

- Kim/Pollard paper-scale volumetric-FEM characters: Fatman, starfish
  (including obstacle escape), fish, and worm;
- Jain/Liu SIMBICON/controller rows: biped push recovery, noisy floor, and
  biped walking; and
- Jain/Liu hand scenes: finger flick, arm fold, and pinch grasp.

The `soft_worm` and `adaptive_soft_contact` examples are representative reduced
evidence. They do not establish full paper-scale parity. The Jain/Liu
four-link flexible-rigid-foot versus deformable-foot comparison is not in the
approved deferral list and remains an explicit roadmap decision.

## Related owners

- Paper models and published metrics:
  `docs/background/deformable_body_paper_targets.md`
- Current roadmap state and gates: `docs/plans/dashboard.md`, PLAN-622
- Active implementation and PR evidence:
  `docs/dev_tasks/dart6_deformable_body_performance/`
- User-visible examples: `examples/adaptive_soft_contact` and
  `examples/soft_worm`
