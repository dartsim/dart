# RESUME — DART 6.20 dependency minimization

Read [README.md](README.md) first (overall SSOT), then
[03-native-collision-port-scoping.md](03-native-collision-port-scoping.md)
(the plan of record for the remaining work), then refresh
[04-orchestration-dashboard.md](04-orchestration-dashboard.md) before
claiming anything.

## Where the task stands

Every DART 6.20 implementation workstream in this folder is complete:

- Dependency-reduction lane: complete (#3105 optimizer removal, #3107 plans).
- Native-replacement lane: complete (#3076 convhull, #3078 ikfast, #3081
  imgui, #3088 odelcpsolver, #3116 GLUT+lodepng, #3122 dart/integration).
- Native-collision-port lane: phases 0–4 and the #3381 consolidation are
  complete; Phase 5 has an unresolved later-release maintainer decision, and
  phases 6–7 are deferred. The separate perf task
  (`../dart6_performance_generalization/`) owns any remaining measured
  performance work.

## Next step

**Current execution (2026-07-29): post-#3381 planning, not PR publication.**
PR #3381 is merged: final head
`64d476b68ad5ae0dcca4e98abb9bba15b6962b87`, release merge
`46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`. It consolidated the DART-owned
engine into `DARTCollisionDetector`, removed the unreleased `"native"`
selector, retained the released `DARTCollide` adapters, and kept FCL as the
6.20 default. No #3381 push, review request, hosted-CI follow-up, or merge
action remains.

The next bounded owner action is to refresh and obtain maintainer ratification
for the later-release Phase 5/6/7 sequence in
`08-phase5-facade-decision.md`, especially the ODE-facade versus coordinated
gz-physics decision. Do not start a default flip or dependency removal on
`release-6.20`; any future implementation must start from the then-current
target release, recapture the Phase 6 acceptance packet on that parent, and
rerun the full gz gate.

### Historical #3381 review trail (closed)

Local review-fix candidate evidence captured on 2026-07-29, after merging
`origin/release-6.20` at `6c88ac1d774` but before publishing the merge commit:

- no-cache Release build, all 155 C++ tests, and all 223 dartpy tests pass;
- `check-lint`, AI command/infrastructure checks, 357 focused AI tests, and all
  seven AI scenario probes pass;
- the pinned downstream aggregate passes all 199 gz-physics tests, all four
  gz-physics performance/symbol checks, and gz-sim
  `INTEGRATION_entity_system`;
- the two Linux visual-smoke scenes pass their image/view verdicts and the four
  engine-rendered overlay regressions; the explicit shape-contact capture
  reports detector `"dart"`, ellipsoid/capsule/cone contacts, no camera issues,
  and zero skipped contacts;
- exact old-header/current-library canaries preserve the pre-consolidation
  6.20 detector/group/object sizes (`32/376/320`) and pass guarded
  construction, clone, update, collision, and destruction 20/20; the released
  6.19.4 detector header also passes 20/20. All ten historical `DARTCollide`
  symbols remain exported, and the private object sidecar header is not
  installed;
- two alternating five-repetition, same-core contact-container A/Bs show no
  measurable incumbent-backend regression: base/head medians are ODE
  `868/867-868 ms`, FCL `257/255-256 ms`, and Bullet `267/266 ms`, with
  identical contact counts (`205/81/90`). CPU scaling was enabled, so only a
  regression beyond noise would be claimable; none was observed.

This was local candidate evidence at the time. It is retained as review
history, not as an instruction to resume publication; #3381 is now merged at
the hashes above.

The review of pushed head `fd96521bb14` found one additional P2: old 6.20
headers inline the DART group/object destructors, so sidecar cleanup could not
depend on new out-of-line destructor bodies. The correction anchors group and
object sidecar ownership in the released-layout `mCollisionDetector` and
`mCachedShape` shared-pointer members instead, restores the inline/defaulted
destructors, and adds a direct sidecar-count regression covering a stack group,
a downstream-style object subclass, and a shape swap. It was published as
`af2ac200e26`, and its review thread was resolved.

Fresh Codex review `4814689190` of that exact head then found one further P2:
the four function-local one-time shape-warning sets could race when independent
worlds converted unsupported or malformed shapes concurrently. The correction
routes all four warning categories through one mutex-protected
registry while leaving supported-shape paths lock-free, and adds an eight-thread
regression that repeatedly exercises every warning category. The complete
43-test consolidated-engine target passes in Release and assertions-enabled
builds. It was published as `2f2d45d99da`, and its review thread was resolved.

Fresh Codex review `4814842827` of that exact head found another P2: the
single-thread two-group Cartesian walk sent disjoint pairs to narrowphase
without the AABB rejection already present in the parallel branch. The final
correction rejects disjoint pairs before collision filters and narrowphase in
both paths, matching the released detector's broadphase ordering. Its
regression constructs eight isolated cross-group overlaps and verifies that
serial and parallel queries both visit exactly eight candidates rather than all
64 Cartesian pairs. The focused detector target passes in Release and
assertions-enabled builds. Fresh no-cache gates also pass all 155 C++ tests,
all 223 dartpy tests, lint/check-lint, all 199 gz-physics tests, all four
gz-physics performance/symbol checks, and the gz-sim entity-system integration
test. This implementation-only correction changes no header or ABI surface and
was included in the merged final head.

**Phase 0 was captured and recaptured on its then-current base.** The baseline
packet lives in [05-phase0-baseline-packet.md](05-phase0-baseline-packet.md)
(raw evidence: [05-artifacts.md](05-artifacts.md)), recorded at branch head
`1e6a8332a730` after merging the then-current `release-6.20` tip
`949a9c2ff5ed`. Its verdict was "native default NOT allowed at this tip" and it
defined the phase-6 acceptance envelope. For any later default-flip tolerance
gate, retrieve JSONL scene dumps matching the recorded SHA-256 digests or
recapture dumps on the flip PR's parent and compare within that same recapture;
the historical packet is not evidence for the current release tip.

**Phases 0 and 1 are merged** (#3271 phase-0 packet, #3281 phase-1 native
math core — C++17, no EnTT, internal-only; FCL stays default). Follow-ups
fixed the `maxNumContacts==0` contract in native sphere-sphere on both
release-6.20 (#3298) and main (#3283).

**Phase 2 is complete.** The following describes the historical
pre-consolidation implementation merged through #3302 and the P-series PRs.
The execution plan is
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
  (at that tip,
  `dart/collision/native/narrow_phase/NarrowPhase.{hpp,cpp}` was added to
  `release-6.20`; #3381 later folded this implementation into
  `dart/collision/dart/`).
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

**Historical Phase 4 evidence (closed for this lane; 2026-07-10 refresh).**
After #3364, the merged AABB-tree broadphase PR (#3368) replaced the O(n^2)
BruteForce pair loop with DART 7's dynamic AABB tree while keeping all eight
native guard rows bit-identical (fat-AABB cull + tight-AABB filter + sorted pair
order): S2 settled-3k 0.0809 -> 0.0340 ms/step (now ties the `dart` detector),
S3 active-3k 34.63 -> 12.28 ms/step, S4 generated-900 0.125 -> 0.072, GB
120/4/1 1404 -> 1232 ms; the gz gate was green. The raw `/tmp` artifacts were
session-local and are not resume dependencies; the durable results are
preserved here and in the merged PR.

Historical measured gaps against the phase-6 envelope (native >= fastest
incumbent per row), retained as input for the separately owned
`dart6_performance_generalization` lane:

1. S3 active-3k: native 12.28 vs dart 9.14 ms/step (16-thread) — remaining
   delta is narrowphase/manifold-cache side; profile on a quiet host first.
2. Small scenes: native ~1.21 vs dart 1.06 ms/step on S1-60; ~40% slower
   than dart on a small MJCF arm-scene bisect. Native small-scene collide
   overhead packet.
3. S6 dense-pile resting profile: RESOLVED as a documented re-scope
   (2026-07-11, maintainer-authorized best-path with self-run A/B). Five
   approaches were falsified with evidence: naive cap-4 (creep 0.158 m,
   S1-120 +102%), an ERV dead-zone inside the resting band (reintroduced
   #3209 creep: the 0.1 ERV push is simultaneously the anti-creep mechanism
   and the jitter pump), split impulse as shipped (untuned; pen 0.182),
   all cached-state toggles (warm-start/friction-basis/refresh-threshold:
   zero effect; the Dantzig fallback fires ~1/89k solves so impulse seeds
   are inert), and quad-area 4th point + cap 4 (fixed the naive-cap-4
   degeneracy and produced the only transient rest ever observed — 5/71 at
   20k steps — but 40k-horizon re-wakes to 0/71). Meanwhile the REAL
   acceptance surfaces all rest BETTER under native than legacy dart:
   S2 gz 3k 3003/3003, S4 900/900 (dart: 600/900), S5 90/90 (dart: 60/90);
   legacy dart itself fails the S6 fixture on 1/5 seeds, and rolling-shape
   piles never rest under ANY detector (physically correct without rolling
   resistance). The consolidated-detector S6 acceptance row becomes:
   bounded max_penetration within the dense-island band (anti-creep, the
   actual #3209 finding: 0.005 at 40k steps vs 0.36 unbounded pre-D7) +
   finite + documented non-resting attributable to rolling dynamics. A
   rolling-resistance contact parameter (condim>=4 analog; D7-style
   adaptive defaults) remains a possible later-release feature. Salvage: the
   quad-area 4th-point criterion fix for the manifold cache is
   independently correct (the volume criterion degenerates on coplanar
   face manifolds) and ships separately with its own hash A/B (cap stays
   3). The raw local artifacts are not durable; the numbers are preserved here
   and in the relevant PR bodies.

**Phase 5 decision is active (maintainer-directed 2026-07-10):**
[08-phase5-facade-decision.md](08-phase5-facade-decision.md) — the native
engine merges INTO the dart detector (`dart/collision/native/` folds into
`dart/collision/dart/`, `NativeCollisionDetector` merges into
`DARTCollisionDetector`, canonical key `"dart"`; the interim `"native"` key
never shipped and is removed in the consolidation PR). The unratified proposal
is to deprecate FCL/Bullet/ODE via messaged attributes on create()/ctors, then
remove external dependencies in a following release with facades over the dart
detector (only `OdeCollisionDetector` needs subclassability for gz). Remaining
ratification points: facade-vs-coordinated-gz-change for ODE and the target
release sequence. Note doc 03's
ConstraintSolver line numbers were stale: the FCL ctor sites are
`ConstraintSolver.cpp:416` and `:433`. The maintainer also set a total PR
budget for the whole effort (prefer fewer, larger cohesive PRs; ~10-20 total
across both dev tasks).

Historical context for #3364 (superseded rows above):

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

Current fresh-agent resume path:

1. Fetch the current target branches and verify this owner state against live
   repository/PR evidence; do not reopen any Phase 4 or #3381 branch.
2. Read `08-phase5-facade-decision.md` and obtain maintainer ratification for the
   ODE-facade versus coordinated gz-physics choice plus the target later-release
   sequence. This is the only active action in this task.
3. Keep general collision-performance investigation in
   `../dart6_performance_generalization/`; the historical gaps above do not
   authorize a follow-up from this dependency-minimization handoff.
4. Phase 6 remains deferred beyond DART 6.20. A later default-flip proposal must
   start from the approved release, change both `ConstraintSolver` constructors,
   and pass a newly captured full A/B packet, scene-dump tolerance checks, local
   tests, and the Gazebo gate.
5. Phase 7 is the dependency win only after that default flip is accepted:
   decouple FCL from the core `dart` target and package surface, then move
   Bullet/ODE/FCL packages out of default paths with package/export smoke tests.

Do **not** add a `CollisionDetectorType::Native` enum or touch `World` detector
defaults, `ConstraintSolver` detector defaults, `WorldConfig`, or
dependency/package metadata before the explicit phase that owns it; FCL remains
the default until phase 6.

**Detector consolidation executed (2026-07-11):** per maintainer direction
2026-07-10, recorded in
[08-phase5-facade-decision.md](08-phase5-facade-decision.md), the native
engine merged INTO the dart detector: `dart/collision/native/` no longer
exists (folded into `dart/collision/dart/`), and
`dart::collision::NativeCollisionDetector`/`Group`/`Object` are gone —
`DARTCollisionDetector`/`Group`/`Object` now denote the former native
engine, with `"dart"` as the only factory key; the unreleased `"native"` key
is removed without a transition alias. This is separate from the phase-6 default
flip, which is still pending and out of scope here. The legacy
narrowphase-only `DARTCollisionDetector` implementation was deleted as part
of this merge. Its released `DARTCollide.{hpp,cpp}` entry points remain as
thin compatibility wrappers over the consolidated detector. The two capability
gaps this opened were closed on the same branch before the flip PR:
SoftMeshShape support was ported into the consolidated detector
(`SoftCollision.*`, soft gates green with zero steady-state allocations,
serial soft-soft only) and EllipsoidShape gained a native conversion (exact
sphere for equal radii, icosphere convex hull otherwise).

**Historical state (2026-07-18):** the consolidation plus phase-6 default flip
was then carried by **PR #3381** (`feature/dart-detector-consolidation`, milestone
DART 6.20.0) at head `ebf33416626`. Earlier branch heads also carried MJCF
plane/stacked-axis behavior changes; those unrelated parser changes are
removed from the final consolidation scope. On
2026-07-18 the branch was refreshed by merging `origin/release-6.20`
(= `75306efe770`; two non-overlapping commits, #3388 pixi lockfile and
#3390 script formatting) because branch protection requires up-to-date
branches; CI re-runs on the new head. Maintainer decisions recorded
2026-07-18: the PR **merge itself stayed with the maintainer** (the agent kept
the branch green and merge-ready), and this task folder stays **ACTIVE**
after the merge (refresh docs; do not retire yet). Post-merge next steps:
refresh the repository-owned owner docs, then phase 7 (FCL decoupling from
core) remains the pending dependency win. The historical 6.21-deprecation /
6.22-facade sequence is a proposal pending target-release ratification, and the
ODE facade-vs-coordinated-gz choice is still open.

**Historical update 2026-07-23:** the maintainer directed that PR #3381 must
**not** change the default collision detector yet. The phase-6
default flip described above has been **reverted**: the
`ConstraintSolver` ctors, `WorldConfig::collisionDetector`,
`World::resolveCollisionDetector`, and `SkelParser`'s fallback are
back to the FCL default, byte-for-byte with `origin/release-6.20`,
and the default-driven test expectations were restored to FCL
values. **PR #3381 ultimately shipped the consolidation only** — the native
engine folded into `DARTCollisionDetector` (sole canonical key `"dart"`),
soft-body + ellipsoid + cone + capsule coverage, released `DARTCollide`
adapters, and the
built-in default remains **`fcl`**. The default flip is deferred to
a later PR, still gated on the phase-6 acceptance envelope. The
facts above that remain true after the 2026-07-29 merge: the task folder stays
**ACTIVE**, later default/dependency work remains deferred, and the milestone
was DART 6.20.0.

See [HANDOFF.md](HANDOFF.md) for the full session handoff, merged state,
remaining ratification points, and later-release gates.

## Standing constraints

- Run the gz gate for any later collision/default/dependency phase:
  `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`.
- Use the repository-owned Pixi tasks in `AGENTS.md`; `pixi run test-all`
  builds the default CMake `ALL` graph and, with the branch-pinned
  `BUILD_TESTING=ON`, runs its CTest and pytest targets. Run `pixi run lint`
  separately, and use `pixi run test` or `pixi run test-py` for focused
  reruns.
- Shared hot files (`pixi.toml`/`pixi.lock`): merge the latest approved target
  branch before pushing; never rebase a published PR branch.
- Prefer fewer PRs from phase 3 onward: group cohesive capability wiring,
  parity tests, and mechanical native-file cleanup in one PR when validation
  remains tractable; split only for real review or risk boundaries.
- Topic branches: after target-release approval, create a fresh branch from
  that release's current remote tip.
- FCL default detector is created in *both* `ConstraintSolver`
  constructors (`dart/constraint/ConstraintSolver.cpp`, lines 416/433 at the
  #3381 merge) — a future phase-6 flip must reverify and change both.
- Do not pick up perf-lane packets (WP-PG.*) from this folder; that lane
  has its own owner and dashboard.
