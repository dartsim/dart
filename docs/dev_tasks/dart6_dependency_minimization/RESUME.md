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

**Phase 0 — baseline evidence packet** (in flight 2026-07-04): capture the
A/B matrix on `release-6.20` head across `dart`/`fcl`/`bullet`/`ode`
(+ `default` continuity row) using the canonical guard-scene protocol in
`../dart6_performance_generalization/01-baseline-evidence.md` (S2/S3/S4/S5
plus this lane's g120/g3000 delta rows, `--max-contacts-per-pair 4`, scene
dumps), then record the packet as `05-phase0-baseline-packet.md` with an
explicit "native default allowed/not allowed" verdict. Cross-reference the
perf lane's WP-PG.01 packet (PR #3263) instead of duplicating its cells.

Phase 1 (native core skeleton, C++17, no EnTT, internal-only, FCL stays
default) starts only after the phase-0 packet is reviewed.

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
  of `b5b2a5bee77`) — the phase-6 flip must change both.
- Do not pick up perf-lane packets (WP-PG.*) from this folder; that lane
  has its own owner and dashboard.
