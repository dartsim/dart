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
math core — C++17, no EnTT, internal-only; FCL stays default). A follow-up
fixed the `maxNumContacts==0` contract in native sphere-sphere (#3283 on
main, #3298 on release-6.20).

**Phase 2 is in progress.** The execution plan (merged, #3302) is
[07-phase2-adapter-scoping.md](07-phase2-adapter-scoping.md): add an
internal, non-default `dart::collision::NativeCollisionDetector` that
bridges DART 6's `CollisionDetector`/`CollisionGroup`/`CollisionObject`
contract to the ported `dart::collision::native` engine (BruteForce
broadphase → narrowphase dispatcher → DART 6 `Contact`), **bypassing the
EnTT world layer** by reusing DART 6's existing `shared_ptr`-based object
manager. FCL stays default; no new dependency; C++17. The plan slices
phase 2 into PRs **P1–P9** (+ optional P10), each gz-gated and
scope-diff-guarded.

- **P1** (BroadPhase base + BruteForce): **#3303** — engine-only, all
  gates green (build, ctest, gz 199/4/1). Merging.
- **P2** (narrowphase dispatcher, sphere/box only): **#3306** — **merged**
  (`dart/collision/native/narrow_phase/narrow_phase.{hpp,cpp}` now on
  `release-6.20`).

**Next: P3a — adapter skeleton + `"native"` factory registration**, per
plan §1.1/§1.2/§1.6 and the P3a row in §3, once P1 (#3303) is merged
(P3a's adapter must compile against both the P1 broadphase and the P2
dispatcher). Then P3b (bridge translation + `sphere_box` + parity),
P4–P9 (pair coverage), then phases 3–7. **Do not** add a
`CollisionDetectorType::Native` enum or touch
`World`/`ConstraintSolver`/`WorldConfig` — selection is the factory
pointer only (`getFactory()->create("native")`).

See [HANDOFF.md](HANDOFF.md) for the full session handoff (merged/open
PRs, worktrees, gotchas, exact P3a starting steps).

## Standing constraints

- gz gate every phase: `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`.
- `pixi run test-all` builds only — runtime coverage is `pixi run test` +
  `pixi run test-py` (maintainer clarification, see `02`).
- Shared hot files (`pixi.toml`/`pixi.lock`): merge `origin/release-6.20`
  before pushing; never rebase a published PR branch.
- Topic branches: `git switch --no-track -c <type>/<topic>
  origin/release-6.20`.
- FCL default detector is created in *both* `ConstraintSolver`
  constructors (`dart/constraint/ConstraintSolver.cpp`, lines 336/353 as
  of `1e6a8332a730`; still FCL after #3281) — the phase-6 flip must change
  both.
- Do not pick up perf-lane packets (WP-PG.*) from this folder; that lane
  has its own owner and dashboard.
