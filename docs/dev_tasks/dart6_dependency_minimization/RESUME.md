# RESUME — DART 6.20 dependency minimization

Read [README.md](README.md) first (overall SSOT), then
[03-native-collision-port-scoping.md](03-native-collision-port-scoping.md)
(the plan of record for the remaining work), then refresh
[04-orchestration-dashboard.md](04-orchestration-dashboard.md) before
claiming anything.

## Where the task stands

Every workstream except the **native collision port** is complete:

- Dependency-reduction lane: complete (#3105 optimizer removal, #3107 plans).
- Native-replacement lane: complete (#3076 convhull, #3078 ikfast, #3081
  imgui, #3088 odelcpsolver, #3116 GLUT+lodepng, #3122 dart/integration).
- Native-collision-port lane (this folder's remaining executable work):
  phases 0–7 in `03-native-collision-port-scoping.md`. The perf task
  (`../dart6_performance_generalization/`) tracks this lane as **WS-F,
  external owner** — sequencing hooks: its WP-PG.42 (SoA broadphase) is
  gated on this lane's phase status; its D8 decision (manifold reduction)
  is re-cut at this lane's phase 3; its WS-B depth is re-reviewed at this
  lane's phase 5.

## Next step

**Phase 0 is captured and recaptured on the current base.** The baseline packet lives in
[05-phase0-baseline-packet.md](05-phase0-baseline-packet.md) (raw
evidence: [05-artifacts.md](05-artifacts.md)), recorded at branch head
`1e6a8332a730` after merging `release-6.20` = `949a9c2ff5ed`, with the
verdict "native default NOT allowed at this tip" and the phase-6 acceptance
envelope. Consume the committed row summaries and verdict for phase-1
sequencing; do **not** re-run the matrix just to start phase 1. For any
phase-6/default-flip tolerance gate, retrieve JSONL scene dumps matching
the recorded SHA-256 digests or recapture dumps on the flip PR's parent
and compare within that same recapture.

**Phases 0 and 1 are merged** (#3271 phase-0 packet, #3281 phase-1 native
math core — C++17, no EnTT, internal-only; FCL stays default). Follow-ups
fixed the `maxNumContacts==0` contract in native sphere-sphere on both
release-6.20 (#3298) and main (#3283).

**Phase 2 is complete.** The execution plan (merged, #3302) is
[07-phase2-adapter-scoping.md](07-phase2-adapter-scoping.md): add an
internal, non-default `dart::collision::NativeCollisionDetector` that
bridges DART 6's `CollisionDetector`/`CollisionGroup`/`CollisionObject`
contract to the ported `dart::collision::native` engine (BruteForce
broadphase → narrowphase dispatcher → DART 6 `Contact`), **bypassing the
EnTT world layer** by reusing DART 6's existing `shared_ptr`-based object
manager. FCL stays default; no new dependency; C++17. The plan slices
phase 2 into PRs **P1–P9** (+ P10 coverage), each gz-gated and
scope-diff-guarded.

- **P1** (BroadPhase base + BruteForce): **#3303** — **merged**.
- **P2** (narrowphase dispatcher, sphere/box only): **#3306** — **merged**
  (`dart/collision/native/narrow_phase/NarrowPhase.{hpp,cpp}` now on
  `release-6.20`).
- **P3a** (adapter skeleton + sphere/box shape conversion, intentionally
  unregistered): **#3318** — **merged**.
- **P3b** (bridge translation + `"native"` registration + `sphere_box` +
  parity): **#3319** — **merged**.
- **P4** (capsule primitive pairs): **#3321** — **merged**.
- **P5** (convex foundation + capsule-capsule): **#3322** — **merged**.
- **P6** (cylinder collision pairs): **#3324** — **merged**.
- **P7** (mesh collision pairs): **#3325** — **merged**.
- **P8/P9** (distance module + plane primitive/convex coverage): **#3343** —
  **merged**.
- **P10** (mixed-scene FCL/DART/native parity coverage): **#3350** —
  **merged**.
- **D1** (native detector distance adapter + native basename normalization):
  **#3352** — **merged**.
- **D2** (native detector raycast adapter + native-vs-Bullet raycast
  benchmarks): **#3355** — **merged**.
- **D3** (native VoxelGrid/compound support): **#3358** — **merged**.
- **D4** (native CCD support): **#3359** — **merged**.
- **D5** (native persistent manifold cache + cached-impulse seed/write-back):
  **#3360** — **merged**.
- **P11** (native contact-container dashboard rows): **#3362** — **merged**.
- **D6** (solver-facing native manifold cap with performance evidence):
  **#3364** — **merged**.

**Next: Phase 4 closeout / remaining measured optimization.** PR #3364 landed
the first native performance optimization: cap solver-facing native manifolds at
three contacts while honoring stricter per-pair caps. This targets the measured
solver row overhead from native's fourth contact on contact-rich scenes and
keeps the native detector opt-in.

Measured on `origin/release-6.20` parent `43e419638986` vs #3364:

| Benchmark row | Parent native | Slice native | Delta | Contacts |
| --- | ---: | ---: | ---: | ---: |
| `BM_ContactContainerActive/60/4/1_mean` | 202.383 ms | 202.918 ms | -0.3% | 84 -> 80 |
| `BM_ContactContainerActive/60/4/16_mean` | 204.547 ms | 203.067 ms | +0.7% | 84 -> 80 |
| `BM_ContactContainerActive/120/4/1_mean` | 2268.942 ms | 1118.032 ms | +50.7% | 282 -> 251 |
| `BM_ContactContainerActive/120/4/16_mean` | 2202.924 ms | 1124.906 ms | +48.9% | 282 -> 251 |
| `BM_ContactContainerActive/120/4/4_mean` | 2193.106 ms | 1167.577 ms | +46.8% | 282 -> 251 |
| `BM_ContactContainerDeactivation/60/4/16/iterations:1_mean` | 30.589 ms | 29.391 ms | +3.9% | 101 -> 97 |

Post-#3364 detector comparison on `BM_ContactContainerActive/120/*/1_mean`:
native 1118.032 ms / 251 contacts, DART 1452.013 ms / 242 contacts, FCL
1475.995 ms / 243 contacts, Bullet 1544.000 ms / 256 contacts.

Fresh-agent resume path:

1. Start from current `origin/release-6.20`; verify #3364 is merged before doing
   any new work. Do not reopen `feature/native-phase4-solver-manifolds`.
2. Re-run the contact-container dashboard and focused profile rows on the latest
   base. If native still clears the Phase 4 performance bar, record a Phase 4
   closeout packet and move to Phase 5. If not, do **one cohesive** follow-up PR
   for the next measured hot path rather than several small PRs.
3. Candidate Phase 4 follow-ups remain broadphase pair pruning, scratch/cache
   reuse, manifold reuse beyond #3364, scene-local allocation control, optional
   SIMD from #3229, and thread-safe contact aggregation. Every performance PR
   needs parent-vs-PR and detector-vs-detector evidence in the style of #3307.
4. Phase 5 is the compatibility facade decision: decide whether Bullet/ODE/FCL
   components stay real optional components or become facade-over-native
   compatibility components. Preserve gz-physics/gz-sim ABI/API expectations,
   especially subclassing of `BulletCollisionDetector` and `OdeCollisionDetector`.
5. Phase 6 is the default flip in both `ConstraintSolver` constructors. Do not
   flip defaults until the full A/B packet, scene-dump tolerance checks, local
   tests, and Gazebo gate pass on the current base.
6. Phase 7 is the actual dependency win: only after the default flip is accepted,
   decouple FCL from the core `dart` target and package dependency surface, then
   move Bullet/ODE/FCL packages out of default paths with package/export smoke
   tests.

Do **not** add a `CollisionDetectorType::Native` enum or touch `World` detector
defaults, `ConstraintSolver` detector defaults, `WorldConfig`, or
dependency/package metadata before the explicit phase that owns it; FCL remains
the default until phase 6.

See [HANDOFF.md](HANDOFF.md) for the full session handoff (merged/open
PRs, worktrees, gotchas, and exact Phase 4 next steps).

## Standing constraints

- gz gate every phase: `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`.
- `pixi run test-all` builds only — runtime coverage is `pixi run test` +
  `pixi run test-py` (maintainer clarification, see `02`).
- Shared hot files (`pixi.toml`/`pixi.lock`): merge `origin/release-6.20`
  before pushing; never rebase a published PR branch.
- Prefer fewer PRs from phase 3 onward: group cohesive capability wiring,
  parity tests, and mechanical native-file cleanup in one PR when validation
  remains tractable; split only for real review or risk boundaries.
- Topic branches: `git switch --no-track -c <type>/<topic>
  origin/release-6.20`.
- FCL default detector is created in *both* `ConstraintSolver`
  constructors (`dart/constraint/ConstraintSolver.cpp`, lines 336/353 as
  of `1e6a8332a730`; still FCL after #3281) — the phase-6 flip must change
  both.
- Do not pick up perf-lane packets (WP-PG.*) from this folder; that lane
  has its own owner and dashboard.
