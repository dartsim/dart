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
envelope. Do **not** re-capture it unless base/collision behavior changes
again; consume it.

**Next: phase 1 — native core skeleton** (C++17, no EnTT, internal-only,
FCL stays default) per `03`'s phased plan, once the packet's review PR
has landed. Deterministic unit tests are part of the phase-1 deliverable;
gz gate before merge.

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
  of `1e6a8332a730`) — the phase-6 flip must change both.
- Do not pick up perf-lane packets (WP-PG.*) from this folder; that lane
  has its own owner and dashboard.
