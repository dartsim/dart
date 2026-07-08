# DART 6 Contact Performance Decisions

This document owns the durable DART 6.20 contact-performance decisions that
survived the round-2 issue #3056 performance-generalization task. The temporary
packet board was useful while behavior-preserving PRs were landing; this file is
the post-task owner for the compatibility envelope and remaining decision gates.

## Outcome

The behavior-preserving round is closed. The merged stack established the DART 6
profiling and benchmark evidence surface, then landed low-risk optimizations
without changing default contact, solver, or sleeping semantics:

- baseline and guard evidence: #3263;
- bounded contact-container dashboard rows: #3327;
- DART 6 profiling workflow and Tracy setup: #3337;
- LCP stage profiling and island census: #3339;
- ODE contact-history span tracking: #3329;
- single-free-body classification cache: #3310;
- shallow-support no-candidate skip: #3341;
- frame allocation discipline and allocation gates: #3297 and #3307;
- DART-native broadphase SIMD consumer: #3299.

This is not a claim that the full issue #3056 performance target, D3 solver
policy, D7 sleeping/penetration behavior, or D8 manifold policy is complete. It
only retires the stale active packet board after the behavior-preserving packets
closed or became evidence-gated.

No unblocked implementation packet remains from that round. Future
contact-performance work needs either new profile evidence that reverses a
rejected candidate or explicit maintainer approval for a behavior-changing
solver, sleeping, or manifold policy.

## Compatibility Envelope

Default-on DART 6 performance changes must preserve bit-identical final-state
hashes, contact counts, pair counts, and resting counts on the relevant guard
scenes for unchanged detectors. They must preserve downstream Gazebo/gz-physics
behavior, public headers, installed package components, and frozen subclassing
surfaces unless a maintainer explicitly accepts a compatibility break.

Behavior-changing contact or solver work is a separate PR class. It needs a
tolerance rationale, old/new guard rows, an explicit re-baseline decision, and
the Gazebo gate. Do not mix that work with behavior-preserving cleanup.

Every performance PR should carry the #3307-style evidence shape documented in
`docs/onboarding/profiling.md`: exact baseline/current-base/head commits,
commands, host metadata, profile evidence, a full before/after matrix, guard
columns, and a strict regression gate or an explicit approved exception.

## Settled Decisions

**D1/D2, SIMD FP and ISA delivery.** DART 6 packaged builds keep baseline ISA
delivery. Do not add unconditional exported-target `-march` flags. SIMD kernels
on state-affecting paths must be bit-identical to their scalar path by default
unless a maintainer approves a recorded re-baseline. The first real SIMD
consumer carried its own evidence in #3299 instead of landing a docs-only
contract PR.

**D4, executor tooling.** The release branch uses the ordinary
`docs/dev_tasks/<task>/` plus resume/handoff workflow for multi-session work.
Do not backport a separate DART 7 packet-executor command just for this closed
round. If contact-performance work reopens, create a fresh task folder with the
current evidence and gates.

**D5, ODE lane depth.** The bounded behavior-preserving ODE history-span work
landed in #3329. The pair-keyed map/pruning and pose-push follow-ups are
evidence-gated after current-base rejection. ODE manifold reduction is governed
by D8, not by the behavior-preserving ODE lane.

**D6, deactivation default divergence.** DART 6.20 keeps its existing
deactivation posture. Main-branch defaults are not release-branch evidence.

## Parked Decisions

**D3, matrix-free large-island solver policy.** #3339 confirmed that the dense
container fixture is dominated by the Dantzig solve proper, not LCP construction
alone. A matrix-free or alternate large-island solver may be the only structural
solver lever for that shape, but default-on solver changes are behavior-changing
and require maintainer approval plus re-baseline evidence. An opt-in solver can
be proposed separately, but it does not close the default #3056 behavior by
itself.

**D7, penetration creep and sleeping policy.** The baseline S6 reproducer shows
unbounded penetration growth and no bodies resting under the current defaults.
Fixes may involve the contact error-reduction budget, large-island convergence,
or the island-rest veto. All of those alter contact depth or sleep transitions,
so implementation is parked until the accepted behavior envelope is decided.
The durable D7 reproducer is:

```bash
CB=./build/default/cpp/Release/bin/contact_benchmark
pixi run $CB --generate-container 71 --steps 20000 --checkpoint 5000 \
  --collision dart
```

The round-2 guard capture on `origin/release-6.20` @ `b9e6910c066` recorded:

| Row | RTF | Avg step ms | Contacts (cap) | Pairs | Resting | Finite | Max pen | Hash |
| --- | ---: | ---: | ---: | ---: | --- | --- | ---: | --- |
| S6_dart | 0.0939 | 10.65 | 160 (false) | 139 | 0/71 | true | 0.3624 | `0x6eb6ff3911ac9d04` |

The acceptance shape for future D7 work is old/new S6 evidence showing bounded
max penetration and all 71 mobile bodies resting under default settings, plus
the compatibility guard matrix for unchanged detectors.

**D8, current-detector manifold reduction.** Per-pair manifold reduction can
remove cap-flooding and tunneling failure modes, especially for ODE/FCL wrapper
paths, but it changes emitted contacts. Decide explicitly whether to pursue it
on current detectors or defer it to the native-collision port before starting
implementation.

## Reopening Criteria

Reopen DART 6 contact-performance implementation only when at least one of
these is true:

- a maintainer accepts a D3, D7, or D8 behavior-change envelope;
- new profile evidence makes an evidence-gated candidate strictly positive on
  the required guard matrix;
- the native-collision port reaches a phase that changes the ODE/FCL/default
  detector decision;
- a new downstream workload narrows the target enough to justify a fresh,
  bounded task.

When reopening, create a new `docs/dev_tasks/<task>/` folder instead of reviving
the retired packet board.
