# AVBD Solver - Dev Task

Implementation tracking for PLAN-104's Augmented Vertex Block Descent work.
Plan: [`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md).
Paper audit:
[`../../plans/104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](../../plans/104-vertex-block-descent-solver/avbd-paper-gap-audit.md).
Corpus matrix:
[`../../plans/104-vertex-block-descent-solver/avbd-demo-corpus.md`](../../plans/104-vertex-block-descent-solver/avbd-demo-corpus.md).

## Current Status

- Latest local follow-up (2026-06-14): after explicit maintainer approval, PR
  #2991 branch `avbd/source-row-extraction-precheck` was pushed to origin at
  `fffb5bf45ba` after merging the latest `origin/main`. The Dynamic Friction
  panel-label thread from Codex review `4492237805` is covered on the pushed
  branch by deriving the plotted high-friction speed label from `max_friction`
  and by integration coverage for the
  `DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION` override. The older
  numeric-equivalent friction-env packet thread is also covered on the pushed
  branch by
  `test_avbd_friction_coefficient_sweep_packet_accepts_equivalent_capture_env`.
  The two Codex bot threads remain unresolved on GitHub because no bot replies
  or thread-resolution mutations were performed.
- Latest local follow-up (2026-06-14): hosted CI for pushed head `fffb5bf45ba`
  exposed a `DART GUI Smoke (Clang)` failure before ordinary compilation:
  CMake 4.3 emitted C++20 module dependency-scanning rules that invoked missing
  `clang-scan-deps` even though DART has no C++ module sources. The local
  follow-up disables `CMAKE_CXX_SCAN_FOR_MODULES` at the root project and adds
  a changelog note. `pixi run lint`, `pixi run build`, and the post-push
  `pixi run test-unit` passed. A fresh Clang/Ninja smoke configure without
  `clang-scan-deps` generated no `.ddi`, `clang-scan-deps`, or
  `CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS-NOTFOUND` Ninja rules and reached normal
  `.o` compilation for `dart-collision-native`; local Clang 22 then failed on
  missing standard C++ header `cmath`, which is a local toolchain issue rather
  than the hosted scanner failure. This CI fix is local only until another
  maintainer-approved push.
- Latest local follow-up (2026-06-14): the default sequential-impulse rigid
  contact friction solve now returns when a clamped tangent impulse delta is
  exactly zero and skips static/prescribed endpoint velocity writes for tangent
  impulses. This trims no-op work in the same Dynamic Friction coefficient
  sweep contact path without changing the normal solve or static endpoint
  semantics. Focused contact behavior, restitution, and baked allocator tests
  passed; a five-row `BM_AvbdDemo2dFrictionCoefficientSweep` smoke recorded
  medians of 7.107, 11.982, 15.833, 7.754, and 7.563 us for max friction 0,
  0.5, 1.0, 2.5, and 5.0 under load average `3.61, 4.55, 7.00` with CPU
  scaling enabled. `pixi run build` and `pixi run test-unit` passed. This is
  source-row path evidence only; it does not refresh the tracked packet, close
  the frictionless source-row CPU gap, resolve the GitHub review threads, or
  claim GPU parity. A guarded zero-restitution approach-velocity probe was
  rejected and reverted after the `/0` benchmark reran around 13.7 us median.
- Prior local follow-up (2026-06-14): the default sequential-impulse rigid
  contact assembly now shares each endpoint's `ContactMaterial` lookup between
  friction and restitution and skips `sqrt` for exact zero combined friction
  products. A source-shaped frictionless Dynamic Friction profile still showed
  `rigid_body_contact` dominating the step at 0.050 ms / 80.0% of a 0.063 ms
  wall step, and `/0` benchmark smokes recorded 7.924 us before this lookup
  cleanup and 7.891 us after it under high host load and CPU-scaling warnings.
  This is path smoke only; it does not refresh the tracked friction-sweep
  packet, close the frictionless source-row CPU gap, resolve the GitHub review
  threads, or claim GPU parity. Validation passed the focused
  `test_world` contact/allocator filter, `pixi run lint`, `pixi run build`,
  `pixi run test-unit`, and `pixi run -e cuda test-all` on the visible NVIDIA
  RTX 5000 Ada host. The CUDA run passed all seven categories and retained the
  existing four `dartpy._world_render_bridge` autodoc warnings during docs.
- Prior local follow-up (2026-06-14): the default sequential-impulse rigid
  contact assembly now skips prescribed/static endpoint arm and tangent
  effective-mass work while preserving dynamic endpoint impulse math. This
  extends the static-ground-side cleanup for the
  `BM_AvbdDemo2dFrictionCoefficientSweep/0` source-shaped row. Focused contact
  behavior and baked allocator tests passed, and a `/0` benchmark smoke
  recorded a 7.04 us median CPU step under load average `1.84, 2.00, 1.34`
  with CPU scaling enabled. Full validation
  passed `pixi run lint`, `pixi run build`, `pixi run test-unit`, focused
  Python regression, `git diff --check`, and `pixi run -e cuda test-all` on
  the visible NVIDIA RTX 5000 Ada host. This is local path and review-fix
  evidence only. It does not refresh the tracked friction-sweep packet, close
  the frictionless source-row CPU gap, resolve the GitHub review thread, or
  claim GPU parity. Afterward, `origin/main` at `c4c5ed87eae4` was merged
  locally; the only conflict was `scripts/capture_py_demo.py`, resolved by
  preserving both the PR branch's capture env/metadata overrides and `main`'s
  visual-verification scene-metrics manifest support. Post-merge validation
  passed `pixi run python -m py_compile scripts/capture_py_demo.py`, focused
  capture and Dynamic Friction Python regressions with explicit build
  `PYTHONPATH`, the focused contact/allocator `test_world` filter,
  `pixi run build`, `pixi run test-unit`, `pixi run lint`,
  `pixi run -e cuda test-all`, and the focused numeric-equivalent friction-env
  packet regression. No push has been performed.
- Latest local follow-up: PLAN-091 WP-091.1 relabels the AVBD contact-scene
  evidence rows: no `avbd-demo2d`/`avbd-demo3d` benchmark or py-demo scene
  emplaces the internal AVBD rigid-contact opt-in config
  (`comps::RigidAvbdContactConfig`), because AVBD contact is not
  facade-selectable, so every rigid contact in those scenes ran DART's default
  sequential-impulse contact path. The pure-contact rows (2D Dynamic Friction,
  Static Friction, Pyramid, Cards, Stack, and Stack Ratio; 3D Ground, Dynamic
  Friction, Static Friction, Pyramid, Stack, and Stack Ratio) timed no AVBD
  rows at all; the joint-plus-contact rows (2D Fracture, Soft Body, Joint
  Grid, and Net; 3D Soft Body, Bridge, and Breakable) timed AVBD
  point-joint/motor/spring rows while their ordinary contacts ran sequential
  impulse; incidental link-link contacts in the chain rows (2D Rod, Rope,
  Heavy Rope, and Hanging Rope; 3D Rope and Heavy Rope) also ran sequential
  impulse. Their faster/slower-than-native ratios are whole-pipeline
  `World::step` comparisons, not AVBD-contact-solver comparisons. New AVBD
  evidence packets must machine-record `resolved_solver_identity` at AVBD
  packet schema version 2 (`scripts/avbd_packet_schema.py`, enforced by
  `pixi run check-avbd-packets`); committed packet bytes are unchanged. This
  is an evidence-integrity relabel only; it does not close or reopen any AVBD
  solver, source-corpus CPU-win, GPU, or paper-number gate.
- Latest resumed checkpoint (2026-06-13): the default sequential-impulse rigid
  contact position correction now skips no-op position writes to
  prescribed/static bodies while preserving the same inverse-mass-weighted
  correction for dynamic endpoints. This targets the static-ground side of the
  `BM_AvbdDemo2dFrictionCoefficientSweep/0` source-shaped row. Focused contact
  behavior and baked allocator tests passed, and a `/0` benchmark smoke
  recorded a 7.78 us median CPU step under load average `19.85, 12.52, 9.51`
  with CPU scaling enabled. Full validation completed on 2026-06-13:
  `pixi run lint`, `pixi run build`, `pixi run test-unit`, and
  `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The CUDA run overlapped external DART/CUDA load, had CPU-scaling benchmark
  warnings, and retained the existing four `dartpy._world_render_bridge`
  autodoc warnings during docs. This is local path evidence only. It does not
  refresh the tracked friction-sweep packet, close the frictionless source-row
  CPU gap, or claim GPU parity.
- Latest resumed checkpoint (2026-06-12): the default sequential-impulse
  normal contact solve now records whether each side of a contact has a nonzero
  normal angular Jacobian term, skips the corresponding zero-angular
  approach-velocity and effective-mass work, avoids no-op angular velocity
  updates for centered normal rows, and returns early from normal-impulse
  application when the clamped impulse delta is zero. This targets the same
  `BM_AvbdDemo2dFrictionCoefficientSweep/0` source-shaped row. Focused contact
  behavior and baked allocator tests passed, and a lower-load `/0` benchmark
  smoke recorded median CPU step time moving from 7.96 us before the edit to
  7.49 us after under CPU scaling warnings. Full validation completed on
  2026-06-13: `pixi run lint`, `pixi run build`, `pixi run test-unit`, and
  `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The CUDA run overlapped external DART/CUDA load and retained the existing
  four `dartpy._world_render_bridge` autodoc warnings during docs. This is
  local path evidence only. This does not refresh the tracked friction-sweep
  packet, close the frictionless source-row CPU gap, or claim GPU parity.
- Latest resumed checkpoint (2026-06-12): the default sequential-impulse
  normal contact solve now stores each contact's `arm x normal` terms and
  reuses them for normal effective mass, restitution approach, and per-iteration
  normal approach velocity instead of rebuilding full contact-point velocities
  with angular cross products inside the normal loop. This targets the same
  `BM_AvbdDemo2dFrictionCoefficientSweep/0` source-shaped row. Focused contact
  behavior and baked allocator tests passed, and a high-load `/0` benchmark
  smoke recorded median CPU step time moving from 20.84 us before the edit to
  20.27 us after under load averages above 23 with CPU scaling enabled. This is
  local path evidence only. Full validation passed `pixi run lint`,
  `pixi run build`, `pixi run test-unit`, and `pixi run -e cuda test-all` on
  the visible NVIDIA RTX 5000 Ada host, with the existing four
  `dartpy._world_render_bridge` autodoc warnings during docs. This does not
  refresh the tracked friction-sweep packet, close the frictionless source-row
  CPU gap, or claim GPU parity.
- Latest resumed checkpoint (2026-06-12): the default sequential-impulse rigid
  contact path now skips inverse world-inertia factorization for centered,
  frictionless normal contacts whose angular contact Jacobian is exactly zero.
  This targets the `BM_AvbdDemo2dFrictionCoefficientSweep/0` source-shaped row,
  where 11 zero-friction boxes contact the ground through centered normal rows.
  Focused contact behavior and baked allocator tests passed, and a `/0`
  benchmark smoke recorded a 7.42 us median CPU step under load average
  `3.64, 4.33, 6.29` with CPU scaling enabled. This is local path evidence
  only; it does not refresh the tracked friction-sweep packet, close the
  frictionless source-row CPU gap, or claim GPU parity.
- Latest resumed checkpoint (2026-06-12): `RigidBodyContactStage::prepare()`
  now sizes ordinary and AVBD contact scratch from the conservative
  collision-shape capacity estimate and prewarms collision-query cache storage
  without generating prepare-time contacts. `execute()` remains the only path
  that assembles contact rows for the actual solve. Focused contact tests, the
  previously failing baked allocator regression in default and CUDA builds,
  `pixi run lint`, `pixi run build`, `pixi run test-unit`, full
  `pixi run -e cuda test-all`, and a `/0` friction-sweep benchmark smoke
  passed. This removes duplicate bake-time contact generation only; the `/0`
  benchmark smoke is path evidence, not a tracked source-row CPU-win refresh.
- Latest resumed checkpoint (2026-06-12): the friction-coefficient sweep owner
  row now matches the tracked packet evidence: DART is faster than the
  same-source native runner at max friction 0.5, 1.0, 2.5, and 5.0, but still
  slower at max friction 0.0, so the all-coefficient CPU-win gate remains open.
  The packet-writer regression now covers mixed faster/slower reference sweeps
  and preserves the remaining-gate text when any coefficient fails the CPU-win
  comparison. This is evidence hygiene only; it does not refresh benchmark
  timings, close the frictionless source-row gap, or claim GPU parity.
- Latest resumed checkpoint (2026-06-12): the default sequential-impulse rigid
  contact path now requests only basic private contact-query details when no
  rigid AVBD contact config component is present, while
  public `World::collide()` and private AVBD contact snapshots still request
  the full shape-index/local-point payload. The same slice caches
  per-contact velocity/transform component pointers, inverse effective mass,
  and unit normal impulse velocity deltas across the sequential normal solve,
  avoiding repeated registry lookups and normal impulse cross/matrix work
  inside the inner iterations. Local validation passed the focused `test_world`
  and `bm_avbd_rigid_fixed_joint` target build; the focused
  zero-friction/sliding contact filter; focused public collision-query
  local-detail coverage for shape-local transforms, cache updates, and compound
  rigid-body shapes; focused `/0` benchmark path checks; `pixi run lint`;
  `pixi run build`; the focused contact and public collision-query filters
  again after lint/build; the full `test_world` binary (314 tests); and
  `git diff --check`. Full CUDA validation also passed
  `pixi run -e cuda test-all` on the visible NVIDIA RTX 5000 Ada host. The
  path checks recorded medians around 6.74 us for the cached normal-impulse
  slice and 7.25 us for the basic-contact-query slice under CPU scaling and
  host load, so they validate behavior on the edited path only; they are not a
  tracked packet refresh or CPU-win comparison. The frictionless source-row gap,
  all-coefficient CPU-win gate, GPU parity, source-demo parity, and
  paper-number gates remain open.
- Latest resumed checkpoint (2026-06-12): the default sequential-impulse rigid
  contact stage now records whether any assembled contact can carry Coulomb
  friction and uses a normal-only Gauss-Seidel loop when every contact is
  frictionless. This keeps the normal impulse and penetration-correction path
  active for the `BM_AvbdDemo2dFrictionCoefficientSweep/0` source-shaped row
  while removing the per-contact friction branch from the inner iteration
  loop. Local validation passed the focused `test_world` and
  `bm_avbd_rigid_fixed_joint` target build; the focused
  `World.RigidBodyContactZeroFrictionPreservesSlidingVelocity`,
  `World.RigidBodyContactFrictionDeceleratesSlidingBody`, and
  `World.RigidBodyContactFrictionRollsSlidingSphere` filter; a focused `/0`
  benchmark path check; and the five-row `BM_AvbdDemo2dFrictionCoefficientSweep`.
  The five-row run recorded median CPU step times of 7.01 us, 7.51 us,
  18.13 us, 11.07 us, and 8.09 us for max friction 0, 0.5, 1.0, 2.5, and
  5.0 respectively under load average `4.76, 9.43, 16.93` with CPU scaling
  enabled. These benchmark reruns validate the path only; they do not refresh
  the tracked packet, compare same-source native timing, or close the
  frictionless CPU-win gate. Final verification also passed `pixi run lint`,
  `pixi run build`, the focused three-test filter after lint/build, the full
  `test_world` binary (314 tests), and `git diff --check`. The frictionless
  source-row gap, all-coefficient CPU-win gate, GPU parity, source-demo parity,
  and paper-number gates remain open.
- Latest resumed checkpoint (2026-06-12): the default sequential-impulse rigid
  contact stage now computes the combined Coulomb coefficient before tangent
  setup and skips tangent-basis/effective-mass construction plus friction solve
  calls when the coefficient is zero. This targets the remaining
  `BM_AvbdDemo2dFrictionCoefficientSweep/0` fixed overhead after profiling the
  source-shaped Dynamic Friction sweep showed `rigid_body_contact` dominated
  the frictionless step. Focused validation passed the `test_world` and
  `bm_avbd_rigid_fixed_joint` target build; the focused
  `World.RigidBodyContactZeroFrictionPreservesSlidingVelocity`,
  `World.RigidBodyContactFrictionDeceleratesSlidingBody`, and
  `World.RigidBodyContactFrictionRollsSlidingSphere` filter; the full
  `test_world` binary (314 tests); the five-row
  `BM_AvbdDemo2dFrictionCoefficientSweep`; and a focused `/0` rerun. The
  benchmark reruns were under high host load averages around
  `32.19, 29.17, 20.43` and `32.33, 29.42, 20.74`, so they validate the path
  but are not packet-refresh or CPU-win evidence. The frictionless source-row
  gap, all-coefficient CPU-win gate, GPU parity, source-demo parity, and
  paper-number gates remain open.
- Latest resumed checkpoint (2026-06-12): the contact-manifold row builder now
  short-circuits the empty Coulomb friction descriptor path after normal rows
  are built. When every active contact has zero friction-force limit, the
  builder clears/syncs stale friction inventory and returns before previous
  friction-direction lookup/sort and tangent-row construction. The focused
  regression now covers both the zero-friction-coefficient case and an active
  positive-coefficient contact with zero normal force. A fresh local
  `BM_AvbdDemo2dFrictionCoefficientSweep` run recorded DART median CPU step
  times of 6.87 us, 8.41 us, 18.60 us, 11.25 us, and 9.18 us for max friction
  0, 0.5, 1.0, 2.5, and 5.0 respectively. This run is benchmark-only local
  evidence and does not regenerate the tracked packet because the same-source
  native timing and capture artifacts were not rerun; it also does not close
  the frictionless CPU gap, all-coefficient CPU-win gate, GPU parity, or
  paper-number gates. Local validation passed the focused target build,
  focused three-test contact-manifold filter, benchmark target build,
  five-row friction-coefficient benchmark, `pixi run lint`, `pixi run build`,
  the focused three-test filter again, full `test_avbd_rigid_block` (97 tests),
  and `git diff --check`.
- Latest resumed checkpoint (2026-06-12): local session branch consolidation is
  complete. The single non-main local resume branch is
  `avbd/source-row-extraction-precheck`; it contains the PR #2977
  `avbd/source-row-perf-slice` work, the later friction-sweep evidence, and the
  current handoff docs. The local `avbd/source-row-perf-slice` branch was
  removed after confirming its head is already an ancestor of the consolidated
  branch. The local `feature/avbd-articulated-masked-rows` branch was removed
  as a superseded raw 33-hour checkpoint; its remote ref remains on origin as
  archive only and should not be used as the fresh-session starting point. The
  unrelated local `feature/free-joint-energy-benchmarks` branch was also
  removed locally and remains on origin. Fresh agents should resume with
  `git switch avbd/source-row-extraction-precheck`, not from the old branch
  names or stashes.
- Latest resumed checkpoint (2026-06-12): the contact-manifold row builder now
  skips the Coulomb tangent-row inventory entirely when every active contact has
  a zero friction-force limit. The implementation preserves the existing
  descriptor layout whenever any contact can carry positive friction, so mixed
  manifolds keep their current row-index assumptions. Focused C++ coverage in
  `RigidContactManifoldBuilderSkipsZeroLimitFrictionRows` verifies that normal
  rows persist while zero-limit friction rows and inventory records are removed.
  A refreshed `BM_AvbdDemo2dFrictionCoefficientSweep` run produced DART median
  CPU step times of 6.20 us, 6.79 us, 14.87 us, 8.85 us, and 7.00 us for max
  friction 0, 0.5, 1.0, 2.5, and 5.0 respectively. The tracked
  [`avbd-friction-coefficient-sweep-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-friction-coefficient-sweep-packet.json)
  now records DART faster than the same-source native runner at max friction
  0.5, 1.0, 2.5, and 5.0, but still slower at frictionless max friction 0.
  The benchmark stdout reported a high host load average during this run, so
  treat the packet as current evidence rather than a paper-number claim. Local
  validation passed the `test_avbd_rigid_block` target build, three
  focused contact-manifold tests, `bm_avbd_rigid_fixed_joint` target build, the
  five-row friction-coefficient benchmark, packet regeneration, and plot
  regeneration, `pixi run lint`, final focused target rebuild, focused
  contact-manifold test rerun, `pixi run build`, and `git diff --check`.
- Latest resumed checkpoint (2026-06-12): the paper/source-corpus friction
  coefficient comparison now has same-source reference timing and
  per-coefficient visual-capture evidence.
  `BM_AvbdDemo2dFrictionCoefficientSweep` reuses the source-shaped
  `avbd-demo2d` Dynamic Friction scene and sweeps maximum dynamic-box Coulomb
  friction values 0, 0.5, 1, 2.5, and 5 across the existing 11 sliding-box
  setup. The native `avbd-demo2d` timing runner accepts
  `--dynamic-friction-max-friction` for matching source timing. The py-demo
  scene reads `DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION`,
  `scripts/capture_py_demo.py` records caller-supplied env/metadata in
  `manifest.json`, and the tracked
  [`avbd-friction-coefficient-sweep-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-friction-coefficient-sweep-packet.json)
  validates five DART benchmark rows, five same-source native timing rows, the
  rendered timing plot, and five visual capture manifests. On this host DART is
  faster at max friction 0.5 and 5.0, but slower at 0, 1.0, and 2.5. Local
  validation for the resumed slice passed the focused capture/packet pytest,
  focused build-path py-demo checks, packet regeneration from the existing
  benchmark/reference/capture artifacts, `pixi run lint`, `pixi run build`, and
  `git diff --check`. This is not a full-coefficient CPU-win, GPU parity, or
  paper-number claim.
- Latest resumed checkpoint (2026-06-12): the public articulated breakable
  motor benchmark surface now has a validated scale packet,
  [`avbd-breakable-motor-scale-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-breakable-motor-scale-packet.json),
  generated from a real `BM_AvbdArticulatedBreakableMotorStep`,
  `BM_AvbdArticulatedPrismaticBreakableMotorStep`,
  `BM_AvbdArticulatedWorldPrismaticBreakableMotorStep`, and
  `BM_AvbdArticulatedWorldRevoluteBreakableMotorStep` run over 1, 8, and 32
  motors. The packet validates finite timing rows and exact `motors` plus
  `breakable_motors` counters for the same-multibody/world-link
  revolute/prismatic public motor row families. This is benchmark-only scale
  evidence for existing public motor break/reset rows; it is not a broad motor
  lifecycle corpus, visual fracture/breakable-wall corpus, CPU-win, GPU, or
  paper-number claim.
- Latest resumed checkpoint (2026-06-12): the public fixed/spherical
  breakable point-joint benchmark surface now has a validated scale packet,
  [`avbd-breakable-joint-scale-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-breakable-joint-scale-packet.json),
  generated from a real `BM_AvbdRigidBreakableJointStep`,
  `BM_AvbdRigidSphericalBreakableJointStep`,
  `BM_AvbdArticulatedBreakableJointStep`,
  `BM_AvbdArticulatedWorldSphericalBreakableJointStep`, and
  `BM_AvbdArticulatedSphericalPairBreakableJointStep` run over 1, 8, and 32
  breakable joints. The packet validates finite timing rows and exact
  `breakable_joints` counters for each fixed/spherical public row family.
  This is benchmark-only scale evidence for existing public break/reset rows;
  it is not a broad breakable-wall/fracture corpus, breakable-motor scale,
  CPU-win, GPU, or paper-number claim.
- Latest resumed checkpoint (2026-06-12): current-pose private
  movable-pair revolute and prismatic AVBD motor rows now have focused
  simulation-mode binary save/load regressions proving tiny restored effort
  limits remain effective after reload. The tests preserve the generated
  private `AvbdRigidWorldPointJointConfig`, command, effort-limit, mask, and
  non-cardinal basis state, then step the restored world to verify the hard
  anchor/hinge/masked rows stay active while the free-axis motor does not
  become an unbounded velocity target. Local validation passed the focused
  `test_variational_integration` target build, the two-test restored
  tiny-limit filter before and after lint, `pixi run lint`, `pixi run build`,
  and `git diff --check`. This is narrow persistent private articulated motor
  evidence only, not broad fracture/corpus, CPU-win, GPU, or paper-number
  evidence.
- Final stop handoff (2026-06-12): the user explicitly directed the session to
  stop working further, only ensure the hand-off docs, and then literally
  stop. Do not continue implementation, verification, hosted CI work, branch
  cleanup, PR mutation, push, merge, or review-comment handling unless a future
  user request explicitly asks for that specific action. The active checkout
  before this docs-only handoff edit was
  `avbd/source-row-extraction-precheck` at local HEAD `ba56bd46517`
  (`Add AVBD high-ratio sweep stability plot`), tracking
  `origin/avbd/source-row-extraction-precheck` and ahead by 17 local commits.
  The branch now includes the high-ratio iteration sweep benchmark, tracked
  packet JSON, and rendered SVG stability plot. This is still only benchmark
  and packet evidence for a focused 50-link/50,000:1 fixture; it is not a
  same-hardware paper-number comparison, broad articulated stability proof, or
  GPU parity claim. No fresh lint/build/test/CI, `git diff --check`, PR
  refresh, push, branch cleanup, merge, or stash operation was performed for
  this stop-only handoff by explicit user direction. `RESUME.md` is the source
  of truth for the current branch inventory, stash inventory, local-only
  commits, PR caveats, stopped plan, and future recovery commands.
- Latest resumed checkpoint (2026-06-12): the paper-scale high-ratio
  iteration sweep now has finite replay stability counters, a tracked
  benchmark/stability-plot packet,
  [`avbd-paper-scale-high-ratio-iteration-sweep-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-packet.json),
  and a rendered SVG plot,
  [`avbd-paper-scale-high-ratio-iteration-sweep-plot.svg`](../../plans/104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-plot.svg),
  generated from a real
  `BM_AvbdPaperScaleHighRatioChainIterationSweep` run over 25, 50, 100, and
  200 max iterations. The packet validates the 50-link/50,000:1 fixture
  counters, requires finite 32-step replay for each budget, and records sorted
  timing/stability plot data. This still does not provide a same-hardware
  paper-number comparison, broad articulated stability proof, or GPU parity.
  Local validation passed the focused packet/plot writer pytest, the focused
  benchmark target build, the actual sweep benchmark run with three
  repetitions, packet/plot generation, `pixi run lint`, `pixi run build`, and
  `git diff --check`.
- Latest resumed checkpoint (2026-06-12): the paper-scale high-ratio
  articulated-chain benchmark now includes
  `BM_AvbdPaperScaleHighRatioChainIterationSweep`, a dashboard-selected
  iteration-count sweep over 25, 50, 100, and 200 max iterations using the
  existing 50-link/50,000:1 fixture and replay-reset path. The display-name
  mapping and dashboard runner know about the row. This is only first
  parameter-sweep infrastructure; it does not provide a tracked packet, plot,
  same-hardware paper-number comparison, or GPU parity claim. Local validation
  passed the focused benchmark target build, benchmark registration list for
  `/25`, `/50`, `/100`, and `/200`, the focused Python dashboard tests,
  `pixi run lint`, `pixi run build`, and `git diff --check`.
- Latest current-state audit (2026-06-12): the next-gap wording was refreshed
  against the current C++/dartpy tests and benchmark rows. The evidence now
  shows that narrow breakable benchmark rows, public world-fixed reset,
  dartpy fixed/spherical break/skip/reset endpoint-shape checks, and
  non-cardinal dartpy same-multibody/world-link one-DOF reset checks are already
  covered. Keep the next local implementation work focused on genuinely open
  broader private articulated motor/fracture lifecycle/corpus coverage, rigid
  contact persistence completeness, or source-demo CPU/GPU parity. PR #2977's
  latest read-only CI refresh showed one failing visible check: Linux
  `Debug Tests` was cancelled at the job timeout while building/running Debug
  Python tests, after Debug C++ tests passed. No PR mutation or hosted CI rerun
  was performed.
- Latest resumed checkpoint (2026-06-12): direct private same-multibody
  fixed, prismatic, and spherical point-joint reset coverage now matches the
  neighboring direct private revolute movable-pair reset path. The new C++
  regressions use explicit off-origin anchors, cover all-axis fixed rows, a
  non-cardinal prismatic slider basis, and spherical linear-only rows, force
  broken-row drift, and prove the existing private
  `AvbdRigidWorldPointJointConfig` re-engages the expected masked hard rows
  across two movable endpoints. Local validation passed the focused
  `test_variational_integration` target build, a focused six-test movable-pair
  reset filter, `pixi run build`, `pixi run lint`, and `git diff --check`.
  This is narrow articulated lifecycle evidence only, not a CPU-win, GPU, or
  paper-number claim.
- Critical stop handoff (2026-06-12): the user explicitly directed the session
  to stop all further work and only ensure the hand-off docs. Do not continue
  implementation, verification, branch cleanup, PR mutation, hosted CI work,
  push, or merge from this checkpoint unless a future user request explicitly
  asks for it. The active checkout before this docs-only handoff edit was the
  consolidated continuation branch `avbd/source-row-extraction-precheck` at
  local HEAD `456b931a57b` (`Broaden AVBD articulated reset axis coverage`),
  tracking `origin/avbd/source-row-extraction-precheck` and ahead by eight
  local commits. This docs-only handoff intentionally has no fresh
  build/test/CI or `git diff --check` verification by explicit user direction;
  only the repository-required pre-commit `pixi run lint` is expected if the
  handoff is committed locally. `RESUME.md` is the current source of truth for
  the branch inventory, stash inventory, recent local commits, PR #2977
  caveats, and the next steps to use only if work is explicitly resumed.
- Latest resumed checkpoint (2026-06-11): same-multibody articulated one-DOF
  reset/re-engagement coverage now exercises non-cardinal axes in the public
  movable offset revolute path, the direct private movable revolute config
  path, and the generated current-pose movable revolute/prismatic config paths.
  The affected tests measure hinge/slider progress against the configured
  arbitrary axis instead of cardinal yaw/X-axis shortcuts. Local validation
  passed the focused `test_variational_integration` target build, the four-test
  reset filter, `pixi run build`, `pixi run lint`, and `git diff --check`. This
  is narrow articulated lifecycle evidence only, not a CPU-win, GPU, or
  paper-number claim.
- Latest resumed checkpoint (2026-06-11): direct private articulated
  revolute/prismatic velocity point-joint broken-state save/load/reset coverage
  now exercises non-cardinal free-axis bases for both child-link and parent-link
  world-link endpoint polarities. The strengthened C++ tests verify serialized
  `AvbdRigidWorldPointJointConfig` axis bases survive while broken and that
  reset re-engages the masked hard rows plus free-axis motor row with the
  correct endpoint polarity. Local validation passed the focused
  `test_variational_integration` target build, the four-test direct private
  one-DOF broken-state persistence filter, `pixi run build`, `pixi run lint`,
  and `git diff --check`. This is narrow articulated lifecycle evidence only,
  not a CPU-win, GPU, or paper-number claim.
- Literal stop handoff (2026-06-11): the user explicitly redirected this
  session to stop all further work and only ensure the hand-off docs. Do not
  continue implementation, branch cleanup, PR mutation, CI reruns, push, or
  verification from this checkpoint unless a future user request explicitly
  asks for it. The active checkout is the consolidated continuation branch
  `avbd/source-row-extraction-precheck` at local HEAD `533dc490d87` (commit
  subject: `Reuse AVBD rigid motor row scratch`), clean before this docs-only
  handoff edit and ahead of `origin/avbd/source-row-extraction-precheck` by
  five commits:
  `51eb9b48e08`, `13604d8b8b5`, `04a369222a7`, `77404f63496`, and
  `533dc490d87`. Treat `RESUME.md` as the detailed source of truth for the
  branch inventory, stash inventory, PR #2977 status, validation history, and
  future resume rules. No lint/build/test/CI or `git diff --check` verification
  was run for this stop-only docs update by explicit user direction.
- Latest resumed checkpoint (2026-06-11): combined rigid linear/angular motor
  source-row construction now has reusable private active-row scratch for the
  large-input path, and the World contact solve scratch reserves/reuses that
  storage across frames. The existing small-input stack path and public
  scratch-free overload remain intact. Local validation passed the focused
  rigid-block target build, the focused motor/world-solve filter (4 tests), the
  full `test_avbd_rigid_block` binary (96 tests), the focused
  `World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap` no-heap
  smoke, `pixi run lint`, `pixi run build`, and `git diff --check`. This is
  only source-row extraction overhead cleanup, not a source CPU-win, GPU, or
  paper-number claim. No push, PR mutation, hosted CI rerun, merge, or branch
  cleanup was performed.
- Latest resumed checkpoint (2026-06-11): work resumed after the prior
  stop-only handoff to finish the contact-manifold small-row scratch cleanup on
  the consolidated local branch `avbd/source-row-extraction-precheck`.
  `buildAvbdRigidContactManifoldRows()` now uses stack-backed active contact,
  normal descriptor, local-point, and friction descriptor storage for small
  contact manifolds, while larger manifolds keep using reusable scratch. Local
  validation passed `pixi run lint`, the affected rigid-block/contact target
  build, the focused rigid contact-manifold filter (2 tests), the focused
  boxed-LCP contact filter (17 tests), full `test_avbd_rigid_block` (95 tests),
  full `test_boxed_lcp_contact` (122 tests), `pixi run build`, and
  `git diff --check`. A follow-up audit tightened the small path so cached
  previous friction directions also use stack storage instead of the scratch
  vector. This checkpoint remains local; no push, PR mutation, hosted CI rerun,
  merge, or branch cleanup was performed. This is only source-row extraction
  overhead cleanup, not a source CPU-win, GPU, or paper-number claim.
- Latest resumed follow-up (2026-06-11): rigid distance-spring source-row
  extraction now has reusable private active-row scratch for the large-input
  path, and the world contact solve scratch reserves/reuses it alongside joint
  and motor row scratch. The existing small-input stack path remains unchanged.
  Local validation passed the focused rigid-block/world targets, the focused
  distance-spring/motor rigid-block filter (4 tests), the focused AVBD world
  filter (4 tests), the full `test_avbd_rigid_block` binary (95 tests),
  `pixi run build`, `pixi run lint`, and `git diff --check`. This is still
  only source-row extraction overhead cleanup, not a source CPU-win, GPU, or
  paper-number claim.
- Latest resumed validation (2026-06-11): the angular-motor source-row cleanup
  checkpoint `ada568afa85` has now been validated after the earlier
  no-verification handoff. The focused rigid-block target, the focused
  angular-motor filter (2 tests), the full `test_avbd_rigid_block` binary (95
  tests), `pixi run build`, `pixi run lint`, and `git diff --check` passed.
  This validates only the narrow standalone angular-motor source-row scratch
  cleanup; it is still not a source CPU-win, GPU, or paper-number claim.
- Latest critical handoff stop (2026-06-11): the user explicitly redirected
  this session to stop implementation and focus only on hand-off for all
  current work, with no further verification. The active checkout remains the
  consolidated continuation branch `avbd/source-row-extraction-precheck`. The
  branch has the local point-joint source-row cleanup commit
  `5a56acb0c4c` on top of pushed `f380bf9bc04`, plus the current
  angular-motor source-row cleanup and this handoff-doc update. No
  lint/build/test/CI or `git diff --check` was run after the stop instruction.
  Before the stop instruction, the angular-motor slice had already passed the
  focused rigid-block target, the focused angular-motor filter (2 tests), the
  full `test_avbd_rigid_block` binary (95 tests), and `pixi run lint`. Full
  `pixi run build` and `git diff --check` were not rerun for the angular-motor
  slice before this stop.
- Latest resumed follow-up (2026-06-12): standalone rigid angular-motor
  source-row construction now uses stack-backed active row/descriptor storage
  for small inputs and keeps pointers to source `AvbdRigidAngularMotor`
  configs instead of copying full motor configs into scratch. This is a narrow
  motor source-row extraction cleanup on top of the point-joint cleanup below,
  not a source CPU-win, GPU, or paper-number claim. Validation has now passed
  as recorded above.
- Latest resumed follow-up (2026-06-12): rigid point-joint linear and angular
  source-row builders now use stack-backed active row/descriptor storage for
  small inputs and keep only pointers to source joint configs instead of
  copying the full `AvbdRigidPointJoint` once per active axis. This is a narrow
  point-joint source-row extraction cleanup for the current consolidated branch,
  not a source CPU-win, GPU, or paper-number claim. Local validation passed the
  focused rigid-block target, the point-joint builder/config focused filter (11
  tests), the full `test_avbd_rigid_block` binary (95 tests), `pixi run lint`,
  `pixi run build`, and `git diff --check`.
- Latest critical handoff stop (2026-06-11): the user explicitly redirected
  this session to stop implementation and focus only on hand-off, with no
  further verification. The consolidated continuation branch remains
  `avbd/source-row-extraction-precheck`; this final handoff captures the
  current contact-manifold local-anchor reuse source/test/docs changes on top
  of `ae19e3b0822` (`Avoid redundant AVBD contact tangent anchor transform`).
  No lint/build/test/CI or `git diff --check` was run after the stop
  instruction. Treat the validation bullets below as pre-stop evidence only,
  and use `RESUME.md` as the source of truth for the branch inventory, stashes,
  PR #2977 state, and next fresh-session plan.
- Latest resumed follow-up (2026-06-12): rigid contact manifold row
  construction now computes each active contact's body-local anchors once and
  reuses them for both the normal row and the paired tangent friction rows.
  `AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows` now also
  verifies the normal/friction rows share those generated local anchors for a
  non-origin contact. This is only a narrow rigid contact source-row extraction
  cleanup and does not close any source CPU-win, GPU, or paper-number gate.
- Latest resumed validation (2026-06-12): the contact-manifold local-anchor
  reuse slice passed the focused rigid-block target, the focused
  `AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows` filter,
  the full `test_avbd_rigid_block` binary (95 tests), `pixi run lint`, and
  `pixi run build`. This is only validation for the narrow source-row
  extraction cleanup above.
- Latest resumed follow-up (2026-06-11): rigid contact manifold friction row
  construction now passes a zero step-start relative position when both tangent
  anchors are initialized from the same world contact point, avoiding a
  redundant local-anchor-to-world transform pair during contact source-row
  extraction. `AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows`
  now covers nonzero body-local anchors while verifying the generated tangent
  rows start with zero offset and zero tangent constraint value. The focused
  rigid-block target, the focused test, the full `test_avbd_rigid_block`
  binary, `pixi run lint`, `pixi run build`, and `git diff --check` passed
  locally. This is only a narrow rigid contact source-row overhead cleanup and
  does not close any source CPU-win, GPU, or paper-number gate.
- Latest critical handoff stop (2026-06-11): the user explicitly directed this
  session to stop implementation and focus only on hand-off for all current
  work, without any further verification. The current checkout is the
  consolidated continuation branch `avbd/source-row-extraction-precheck`, with
  pushed head `8fc57deb9d6` (`Checkpoint AVBD handoff state`) before this
  final handoff-doc commit. The branch already contains the point-attachment
  world-point reuse slice, which computes the point-attachment world anchor once
  in `addAvbdRigidPointAttachment()` and reuses it for both constraint value and
  6D direction assembly. Before the stop instruction, local validation had
  already passed `pixi run lint`, the focused rigid-block target/filter, full
  `test_avbd_rigid_block`, and `pixi run build`; `pixi run test-unit` had been
  started but no final result is claimed in this handoff. After the stop
  instruction, no further lint/build/test/CI or `git diff --check` verification
  was run. This remains a narrow helper overhead cleanup and does not close any
  source CPU-win, GPU, or paper-number gate.
- Latest resumed validation (2026-06-11): the handoff-captured
  point-attachment and distance-spring direction helper slice on
  `avbd/source-row-extraction-precheck` has now been validated locally.
  `avbdRigidWorldPointDirection()` is available to point attachments,
  point-attachment and distance-spring direction helpers now share the
  exact-origin path, and
  `AvbdRigidBlock.PointAttachmentOriginAnchorDirectionStaysTranslational`
  covers the attachment origin-anchor behavior. The focused rigid-block helper
  filter passed 9 tests, the full `test_avbd_rigid_block` binary passed 95
  tests, `pixi run build` passed, `pixi run lint` passed, `git diff --check`
  passed, and `pixi run test-unit` passed 161/161 tests. This is still a
  narrow helper overhead cleanup and does not close any source CPU-win, GPU, or
  paper-number gate.
- Previous critical handoff stop (2026-06-11): the user explicitly directed the
  session to stop implementation and focus only on hand-off for all current
  work, without any further verification. The checkpoint was preserved and
  pushed first; work has since resumed and the checkpointed helper slice now
  has the local validation evidence above.
- Latest resumed follow-up (2026-06-11): generic rigid point-pair and paired
  friction direction assembly now share `avbdRigidWorldPointDirection()`, the
  same exact-origin helper used by distance springs. Source rows whose rigid
  point-joint, motor, or friction anchors sit exactly at a body origin now skip
  angular-arm cross products during standalone row stamping and the serial
  rigid row driver, while off-origin rows still take the general path.
  `AvbdRigidBlock.PointPairOriginAnchorDirectionStaysTranslational` covers the
  helper behavior. The focused `test_avbd_rigid_block` target and rigid-block
  binary passed locally; `pixi run build` and `pixi run lint` also passed. This
  is still only a narrow helper overhead cleanup and does not close any source
  CPU-win, GPU, or paper-number gate.
- Previous resumed follow-up: distance-spring source rows now share
  a single exact-origin-anchor helper for both force direction and Hessian
  assembly. Center-anchor Spring rows skip angular-arm cross products and the
  generic world-point Jacobian multiply when the rigid anchor is exactly at the
  body origin, while off-origin Spring Ratio rows still take the general path.
  `AvbdRigidBlock.DistanceSpringOriginAnchorDirectionStaysTranslational` and
  `AvbdRigidBlock.DistanceSpringOriginAnchorHessianStaysTranslational` cover
  the helper behavior. Focused and full `test_avbd_rigid_block`, `pixi run
lint`, `pixi run build`, and `pixi run test-unit` passed locally; this is
  still only a narrow helper overhead cleanup and does not close any source
  CPU-win, GPU, or paper-number gate.
- Consolidated branch handoff: resume from
  `avbd/source-row-extraction-precheck`. It is the single branch for the next
  fresh Claude/Codex session and includes the #2977 source-row prep fix, the
  stacked source-row extraction precheck work, the current resume docs, the
  formerly stash-only `normalizeAvbdRigidOrientation()` squared-norm fast path,
  the origin-anchor `avbdRigidBodyWorldPoint()` fast path, and the latest
  origin-anchor distance-spring, generic point-pair, point-attachment, and
  distance-spring direction helper fast paths, plus the latest
  point-attachment world-point reuse cleanup. The point-attachment and final
  distance-spring direction edits now have local validation recorded in
  `RESUME.md`; the final handoff docs themselves intentionally have no fresh
  verification after the critical stop instruction. Local stashes may still
  exist as historical recovery points, but fresh work should not depend on
  them. The latest pushed continuation head before this final handoff update was
  `8fc57deb9d6` (`Checkpoint AVBD handoff state`). PR #2977
  (`avbd/source-row-perf-slice`, head `5297462d34b`) remains open in the latest
  known state; the latest read-only refresh showed all visible completed checks
  green/skipped/neutral and only Linux `Debug Tests` still in progress. It
  should only receive CI-fix commits if a fresh status refresh reveals a
  concrete failure. `RESUME.md` is the detailed source of truth for the current
  plan, branch inventory, validation, and branch-cleanup rules.
- Latest handoff-captured follow-up: `addAvbdRigidDistanceSpringHessianAtWorldPoint()`
  now stamps the translational 3x3 Hessian block directly when the world anchor
  is exactly the rigid body origin, avoiding the generic world-point Jacobian
  multiply for center-anchor Spring/Spring Ratio source rows.
  `AvbdRigidBlock.DistanceSpringOriginAnchorHessianStaysTranslational` covers
  the behavior. Focused and full `test_avbd_rigid_block` validation had passed
  before the final stop instruction, but no lint/build/test/CI verification was
  run after this handoff-doc update by explicit user request. This is only a
  narrow helper overhead cleanup; it does not close any source CPU-win, GPU, or
  paper-number gate.
- Prior handoff stop: the user previously redirected the session to hand-off
  only with no further verification, and that docs-only handoff was pushed.
  Work has since resumed on the same consolidated branch. Do not treat that
  older docs-only handoff as validation evidence; use the current validation in
  `RESUME.md`, refresh PR #2977 status in a fresh session, and resume from the
  consolidated branch above.
- Latest resumed follow-up: `avbdRigidBodyWorldPoint()` now returns the body
  position directly for exact origin anchors, avoiding quaternion normalization
  and rotation in source rows whose rigid-body point constraints or radial
  springs attach at body origins. `AvbdRigidBlock.BodyWorldPointKeepsOriginAnchorAtPosition`
  covers the behavior, and the full `test_avbd_rigid_block` binary passes with
  91 tests. This is only a narrow helper overhead cleanup; it does not close
  any source CPU-win, GPU, or paper-number gate.
- Latest resumed follow-up: `RigidBodyContactStage::prepare()` now reserves
  AVBD scratch only when contacts could use a contact-config storage path or
  when point-joint/distance-spring AVBD pair constraints exist. Contact solver
  scratch still reserves for ordinary contacts, but worlds with no possible
  AVBD rows avoid the AVBD snapshot/inventory/solve-scratch reserve path. The
  focused no-heap/contact-stage and rigid source-row tests pass. This is only
  source-row prepare-overhead cleanup; it does not close any source CPU-win,
  GPU, or paper-number gate.
- Latest local follow-up: rigid contact snapshot row assignment now avoids the
  duplicate per-contact row-counter claim during snapshot collection and fills
  contact row indices in one linear pass after canonical contact-row ordering,
  without writing the scratch row-counter vector. The focused row-order tests
  still pass, preserving warm-start row identity across contact and endpoint
  ordering. This is only contact snapshot overhead cleanup for rigid source
  rows; it does not close any source CPU-win, GPU, or paper-number gate.
- Latest local follow-up: `normalizeAvbdRigidOrientation()` now uses a
  squared-norm fast path, keeps exact unit quaternions exact without division,
  and preserves scaled-input normalization plus invalid-input identity fallback.
  Focused `test_avbd_rigid_block` validation passes. This trims a repeated
  rigid-row orientation helper used by contact, joint, motor, and spring source
  rows; it is only a narrow overhead cleanup and does not close any source
  CPU-win, GPU, or paper-number gate.
- Latest local follow-up: `RigidBodyContactStage::execute()` now uses
  storage-level AVBD point-joint and distance-spring prechecks before running
  the exact extraction views. This avoids scanning the same AVBD config sets
  twice in joint/spring source rows while still returning early without scratch
  allocation in worlds that have no private pair-constraint storage. Focused
  rigid AVBD contact-stage, distance-spring, and dartpy world tests pass. A
  selected same-filter benchmark toggle under high host load moved median CPU
  time lower for 2D Motor, Joint Grid, Rope, Heavy Rope, Hanging Rope, Spring,
  Spring Ratio, 3D Rope, and 3D Heavy Rope, while 3D Spring and Spring Ratio
  were slightly slower in the noisy smoke. This remains source-row overhead
  evidence only and does not close any CPU-win, GPU, or paper-number gate.
- Latest handoff follow-up: PR #2977 (`avbd/source-row-perf-slice`) has been
  merged forward to `origin/main` at `7d05d7b9ea7` after the hosted PR branch
  reported `BEHIND`. The two red hosted checks were self-hosted runner losses
  during build (`CUDA Build` on `dartsim-mark13-4` and CodeQL C++ on
  `dartsim-mark13-5`); their logs stop with the build step still in progress
  and no compiler/test failure recorded. The Codecov bot comment on
  `world_step_stage.cpp` reported one uncovered changed line while the Codecov
  statuses were passing. The PR follow-up keeps the no-contact prepare skip,
  but restores prepare-time `World::queryContacts()` cache warmup for worlds
  that can have rigid contacts and sizes AVBD contact scratch from the larger
  of the collision-shape estimate and warmed query capacity. Focused CUDA
  `test_world --gtest_filter=World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap`,
  `pixi run build`, and `pixi run test-unit` pass locally. Full CUDA
  `test-all` was previously started on the stacked extraction branch and
  intentionally stopped when the task changed to #2977 handoff. See
  `RESUME.md` for the branch/stash inventory and exact fresh-session plan.
- Latest local follow-up: after merging `origin/main` at `906a6c0241fe`, the
  narrow `RigidBodyContactStage::prepare()` source-row cleanup still builds and
  focused AVBD source-row tests plus `pixi run test-unit` pass. The cleanup
  avoids the collision-shape capacity scan when the contact query is already
  skipped, avoids running a prepare-time contact query that `execute()` repeats
  only to size AVBD contact scratch, and uses the distance-spring storage size
  for AVBD scratch reserve capacity instead of iterating the spring configs just
  to count them. Earlier same-branch benchmark smoke under lower host load moved
  `BM_AvbdDemo2dMotorStep_median` from about 9.34 us to 8.83 us,
  `BM_AvbdDemo2dSpringStep_median` from about 5.04 us to 4.28 us, and
  `BM_AvbdDemo2dSpringRatioStep_median` from about 45.3 us to 37.1 us. Coverage
  CI then caught that the no-query cleanup still needed contact-scene AVBD
  scratch reservations, so the kept variant reserves contact scratch from the
  collision-shape capacity estimate without restoring the duplicate
  `queryContacts()` call. This remains no-contact/source-row overhead evidence
  and does not close any source CPU-win, GPU, or paper-number gate.
- Latest local follow-up: C++ and dartpy public articulated AVBD stiffness
  persistence coverage now exercises fixed, revolute, prismatic, and spherical
  public articulated facades for both same-multibody link pairs and world-link
  endpoints. The C++ serialization test and dartpy unit test set finite
  start/linear/angular AVBD stiffness, save/load the world, mutate the restored
  stiffness values, and verify they remain visible after entering simulation
  mode. A focused variational-integration test now also verifies restored
  stiffness for those endpoint/type combinations feeds the private point-joint
  configs rebuilt at simulation entry. This is narrow public facade
  serialization/extraction coverage; it does not close broader articulated
  lifecycle, source-corpus CPU-win, GPU, or paper-number gates.
- Latest local follow-up: CUDA boxed-LCP PGS dense world-contact tests now keep
  the largest 128-box fixture as a cheap shape gate while bounding the default
  runtime-contract CUDA sweeps to smaller dense packets. #2973's CUDA failure
  timed out in the pre-existing `test_lcp_jacobi_batch_cuda` dense PGS suite;
  `main` passed the same binary but only with about 1099 seconds of runtime.
  This is CI-runtime calibration only; it does not close any AVBD solver,
  CPU-win, GPU, or paper-number gate.
- Latest local follow-up: dartpy public articulated fixed, spherical, cardinal
  one-DOF motor, and non-cardinal one-DOF motor break/reset coverage now
  rechecks endpoint shape, joint type, DOF count, and motor axis after reset
  re-engages the rows for same-multibody, world-link, and movable-pair cases;
  the non-cardinal direct cases now also recheck that shape while broken rows
  are being skipped before reset. Broken-state binary round-trip tests now also
  recheck the restored facade shape after reset re-engages
  same-multibody/world-link fixed, spherical, and one-DOF rows. The focused
  selected pytest filters passed. This is only a narrow public facade lifecycle
  assertion slice; it does not close broader articulated fracture, motor
  lifecycle, source-corpus, CPU-win, GPU, or paper-number gates.
- Latest local follow-up: small AVBD rigid world-contact snapshots no longer
  reserve the endpoint entity-index hash map while the body count is within the
  small-row linear-scan capacity. Focused snapshot-index tests passed, and a
  selected local Motor benchmark smoke moved from the earlier about 9.3 us mean
  to about 8.0 us mean CPU time under host load around 7. This is only
  no-contact source-row overhead evidence; it does not close any source-row
  CPU-win, GPU, or paper-number gate.
- Latest local follow-up: C++ and dartpy public articulated facade coverage now
  verifies that `World::clear()` / `World.clear()` invalidates existing
  articulated link/joint handles, drops the public point-joint facades, and
  resets the generated `joint_###` name sequence before a rebuilt world creates
  a fresh empty-name facade. This is only a narrow public facade
  lifecycle/name-counter guard; it does not close broader articulated motor
  lifecycle, fracture corpus, CPU-win, GPU, or paper-number gates.
- Latest local follow-up: Linux Release CI now runs the ASAN build with
  `DART_PARALLEL_JOBS=4` after the hosted runner killed the ASAN compilation
  with exit 137 while rebuilding the full test target set. This is CI resource
  calibration only; it does not close any AVBD solver, CPU-win, GPU, or
  paper-number gate.
- Latest local follow-up: the AVBD-only contact regressions in
  `test_boxed_lcp_contact` are excluded on MSVC after both target-level `/Od`
  and a source-local optimization guard still hit C1001 in MSVC 19.44. The
  ordinary BoxedLcp contact tests remain active on Windows, and the AVBD
  regressions remain active on Linux/macOS. This is CI portability only; it
  does not close any AVBD solver, CPU-win, GPU, or paper-number gate.
- Latest local follow-up: the IPC bake allocation regression now compares the
  unsupported plane scene against the same IPC contact-query-only setup with
  supported collision geometry instead of against the sequential solver's
  allocation baseline. This keeps the guard focused on accidental IPC query
  prewarm work after the sequential prepare path was tightened. This is CI-test
  calibration only; it does not close any AVBD solver, CPU-win, GPU, or
  paper-number gate.
- Latest local follow-up: `test_boxed_lcp_contact` now avoids a repeated MSVC
  19.44 internal compiler error in the primitive endpoint-row helper by using
  an explicit `PrimitiveRowKey` struct instead of a nested
  `std::pair<AvbdContactEndpointId, AvbdContactEndpointId>` alias in that
  section. The focused Linux target rebuild and full
  `test_boxed_lcp_contact` binary passed locally. This is CI portability only;
  it does not close any AVBD solver, CPU-win, GPU, or paper-number gate.
- Latest local follow-up: `RigidBodyContactStage::prepare()` now uses the same
  no-dynamic-collision-geometry skip predicate as execute before sizing contact
  scratch, so no-collision source rows such as the 2D Motor row avoid a
  redundant `World::queryContacts()` call during stage preparation. Focused
  world/contact-stage tests and step-profiling tests passed, and a selected
  Motor benchmark smoke improved from about 13.5 us to about 9.2 us median CPU
  time under changing host load. This is no-contact prepare-overhead evidence
  only; it does not close the Motor CPU-win, GPU, or paper-number gates.
- Latest local follow-up: point-pair and angular constraint caches now use exact
  component equality for repeated local anchors and target orientations instead
  of zero-tolerance approximate comparisons. This keeps the block-solver
  cache-hit path aligned with how joint rows copy shared descriptors, and
  focused block tests passed. A selected source-row benchmark smoke ran under
  host load average around 12.8 and is treated only as noisy overhead evidence;
  it does not close any CPU-win, GPU, or paper-number gate.
- Latest local follow-up: the rigid block solver's inlined angular row-state
  update now skips capped finite-stiffness angular rows before recomputing
  orientation error, matching the existing standalone angular updater and the
  finite point-pair fast path. Focused block tests passed. A same-session
  selected source-row benchmark smoke was run under high host load and is
  treated only as overhead evidence; it does not close any CPU-win, GPU, or
  paper-number gate.
- Latest local follow-up: rigid AVBD finite-stiffness point-joint row builders
  skip unused previous-constraint captures, and the block solver reuses
  consecutive point-pair world-anchor transforms during block assembly and
  row-state updates. Focused block tests passed, and selected local smoke
  reports `BM_AvbdDemo2dSoftBodyStep_median` around 2.03 ms, plus Rod,
  Joint Grid, Soft Body, Rope, and Net around 0.149 ms, 10.8 ms, 1.70 ms,
  0.113 ms, and 0.360 ms respectively under host load. This is overhead
  evidence only; it does not close any CPU-win, GPU, or paper-number gate.
- [x] Phase A0: scalar row foundation started
  - Internal row update utility:
    `dart/simulation/detail/deformable_vbd/avbd_constraint.hpp`.
  - Focused tests:
    `tests/unit/simulation/deformable_vbd/test_avbd_constraint.cpp`.
- [ ] Phase A1: row model and persistent inventory started, not complete
  - Deterministic scalar-row descriptors, keys, warm-start state cache, and
    role/axis separation:
    `dart/simulation/detail/deformable_vbd/avbd_row_inventory.hpp`.
    Private rigid friction rows can also retain their previous tangent
    direction metadata so warm-started duals can be projected when the tangent
    basis changes.
  - Still missing production row generation and storage for the full contact,
    friction, joint, motor, fracture, and attachment corpus.
- [ ] Phase A2: CPU deformable AVBD through the existing VBD body path
  - First active contact-normal kernel slice started:
    `addAvbdHalfSpaceContactNormal`,
    `updateAvbdHalfSpaceContactNormalRow`, and
    `blockDescentMassSpringAvbdGround`.
  - First non-contact hard attachment row slice started:
    `AvbdPointAttachmentRow`, `addAvbdPointAttachment`,
    `updateAvbdPointAttachmentRow`, and
    `blockDescentMassSpringAvbdAttachments`, with focused
    `VbdAttachment.*` tests for force direction, alpha regularization,
    per-axis warm-start persistence, and dual/stiffness growth.
  - Narrow World wiring started behind internal
    `DeformableVbdConfig::useAvbdContactNormalRows` for active static
    half-space contact-normal rows in supported serial mass-spring static
    contact scenes. The scratch cache now carries stable row IDs from
    body/entity, vertex, and static obstacle feature IDs. Static box obstacle
    feature IDs now distinguish faces, edges, and corners so AVBD normal and
    friction rows reset across box-manifold changes while warm-starting across
    small same-feature penetrations. Persisting static half-space friction rows
    also project their decayed tangential dual into the current tangent basis
    when a smooth obstacle normal changes, and persisting self-contact friction
    rows project their generalized tangential dual into the current 12D tangent
    stencil, with regression coverage in
    `VbdContact.AvbdBoxContactFeatureCodeSeparatesBoxManifolds`,
    `VbdContact.AvbdFrictionDualProjectionPreservesWorldImpulse`, and
    `VbdCombinedDescent.AvbdSelfContactFrictionDualProjectionPreservesGeneralizedImpulse`,
    plus
    `VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact`.
  - Narrow World wiring also started behind internal
    `DeformableVbdConfig::useAvbdAttachmentRows` for pinned or scripted
    deformable nodes in supported serial mass-spring scenes. It keeps
    attachment rows in per-body scratch storage, uses rest-position targets for
    ordinary pinned nodes and boundary targets for active Dirichlet anchors,
    can combine with the supported static-contact row envelope, and exposes
    only internal per-step row counters for tests such as
    `VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode`.
  - First finite-stiffness deformable row slice started:
    `AvbdSpringFiniteStiffnessRow`,
    `updateAvbdSpringFiniteStiffnessRow`, and
    `blockDescentMassSpringAvbdFiniteStiffness`, with focused
    `VbdFiniteStiffness.*` tests for stretch sign, ramp/cap behavior,
    finite-row warm starting, and driver updates.
  - First finite-stiffness tetrahedral material row slice started:
    `AvbdTetMaterialFiniteStiffnessRow`,
    `avbdTetMaterialConstraintValue`, and
    `blockDescentTetMeshAvbdFiniteStiffness`, with focused
    `VbdFiniteStiffness.*` tests for strain-norm row error, unit-scale
    ramp/cap behavior, and standalone tet-material driver updates.
  - Narrow World wiring also started behind internal
    `DeformableVbdConfig::useAvbdFiniteStiffnessRows` for progressive spring
    stiffness rows in supported serial mass-spring scenes, with row-counter and
    behavior coverage in
    `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain`.
  - The same internal finite-stiffness flag now also wires progressive
    tetrahedral material rows for supported serial, frictionless pure-tet
    scenes, with a dimensionless Lamé multiplier, a separate
    `vbdAvbdFiniteStiffnessTetRows` diagnostic counter, coexistence with the
    existing lagged VBD self-contact penalty, and regression coverage in
    `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenTetrahedralMaterial`.
    Contact-free finite-stiffness mass-spring scenes now stay on the AVBD row
    path when the material has a friction coefficient but no active contact or
    self-contact friction source, with coverage in
    `VbdWorldSolver.AvbdFiniteStiffnessRowsIgnoreUnusedFrictionCoefficient`.
    Pure-tet finite-stiffness scenes can now also combine AVBD self-contact
    normal rows and bounded self-contact friction tangent rows in the same
    serial tet solve when `useAvbdSelfContactNormalRows` is requested, with
    coverage in
    `VbdWorldSolver.AvbdTetRowsCombineSelfContactFrictionRows`.
  - The supported World mass-spring envelope now routes contact-normal,
    attachment, and finite-stiffness spring rows through one combined serial
    AVBD row solve, with regression coverage in
    `VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness`.
  - First bounded friction-tangent row slice started:
    `AvbdHalfSpaceFrictionRow`, `avbdFrictionTangentBounds`,
    `addAvbdHalfSpaceFrictionTangent`, and
    `updateAvbdHalfSpaceFrictionTangentRow`, with optional participation in
    `blockDescentMassSpringAvbdRows` and focused `VbdContact.Avbd*` coverage
    for Coulomb clamping, dual/stiffness updates, and tangential-motion
    resistance. Supported static-contact mass-spring World scenes now generate
    two bounded tangent rows per active AVBD contact normal, with
    `VbdWorldSolver.AvbdContactNormalRowsIncludeFrictionTangentRows` and
    `VbdWorldSolver.AvbdFrictionTangentRowsDecelerateSlidingBody` coverage.
    Adjacent tangent-row pairs now use the lagged tangential dual to switch
    between static sticking and dynamic sliding, and project the paired force to
    the circular Coulomb cone with focused
    `VbdContact.AvbdFrictionTangentPair*` coverage.
  - First self-contact normal row World slice started:
    `AvbdSelfContactNormalRow`, `avbdSelfContactNormalConstraintValue`,
    `addAvbdSelfContactNormal`, and `updateAvbdSelfContactNormalRow`. The
    kernel uses the IPC point-triangle / edge-edge barrier only to recover the
    local repulsion direction, then stamps an AVBD hard normal row with focused
    `VbdCombinedDescent.AvbdSelfContact*` coverage for direction, dual/stiffness
    updates, edge-edge rows, and inactive-band no-ops. Supported serial
    mass-spring self-contact scenes can opt in through
    `DeformableVbdConfig::useAvbdSelfContactNormalRows`; World generation keeps
    one scalar row per point-triangle / edge-edge primitive, stamps each
    incident local vertex through the lagged adjacency, reports
    `vbdAvbdSelfContactNormalRows`, and has regression coverage in
    `VbdWorldSolver.AvbdSelfContactNormalRowsPushSupportedSurfaceApart`.
  - First self-contact friction row slice wired:
    `AvbdSelfContactFrictionRow`,
    `addAvbdSelfContactFrictionTangent`, and
    `updateAvbdSelfContactFrictionTangentRow`. The row reuses the IPC
    point-triangle / edge-edge tangent stencils against lagged primitive
    positions, participates in `blockDescentMassSpringAvbdRows`, has supported
    World generation for serial mass-spring self-contact scenes, and has
    focused `VbdCombinedDescent.AvbdSelfContactFriction*` plus
    `VbdWorldSolver.AvbdSelfContactNormalRowsIncludeFrictionTangentRows`
    and `VbdWorldSolver.AvbdSelfContactFrictionRowsReduceTangentialMotion`
    coverage for tangent direction, static/dynamic pair switching,
    circular-cone projection, tangential-motion resistance, World row
    generation, and World-level tangential slip reduction. Static-contact and
    self-contact friction rows also have combined-row coexistence coverage in
    `VbdWorldSolver.AvbdContactAndSelfContactFrictionRowsCombine`.
  - Fallback coverage now guards the unsupported World envelopes:
    mixed spring-plus-tet topology, mass-spring self-contact without the
    self-contact AVBD flag, Chebyshev, Rayleigh damping, parallel settings, and
    unsupported requested row families all keep using the existing VBD path
    without reporting partial AVBD row counters.
  - Still missing full contact-manifold friction persistence beyond current
    static half-space, lagged self-contact, and private rigid tangent-projection
    rows. The first private
    dynamic/rigid contact identity helpers now pack feature kind/index values,
    canonicalize two-endpoint row keys, and create private normal/friction row
    descriptors. The first private rigid contact-manifold row builder can turn
    active contact points into warm-started rigid normal/friction rows and
    projects persisted paired tangent duals into the current tangent basis when
    the rigid contact normal rotates. The
    first private World-contact snapshot helper can translate rigid-body
    `World::collide()` contacts into those manifold points, run them through
    the private serial rigid row solve, write dynamic rigid-body state back to
    the ECS, and exercise the combined private build/solve/writeback wrapper in
    focused tests, but full row-family generation, articulated blocks, parallel
    dual/stiffness updates, and GPU parity remain missing.
- [ ] Phase A3: CPU 6-DOF rigid/articulated AVBD blocks.
  - First internal 6-DOF rigid block foundation started:
    `AvbdRigidBodyBlock`, quaternion tangent-step helpers,
    `addAvbdRigidBodyInertiaTerm`, `solveAvbdRigidBodyBlock`,
    `applyAvbdRigidBodyStep`, scalar rigid point-attachment rows, and private
    two-body rigid point-pair row stamping in
    `dart/simulation/detail/deformable_vbd/rigid_block_kernel.hpp`,
    with focused `AvbdRigidBlock.*` coverage. Point-pair rows now also carry a
    scalar offset, private constructors for rigid contact-normal and bounded
    contact-friction tangent rows, a paired tangent helper that switches
    between static sticking and dynamic sliding while projecting the force to a
    circular Coulomb cone, and a private serial row driver for point-attachment,
    contact-normal point-pair, and paired friction tangent rows. The paired
    friction path now shares precomputed regularized tangent constraint values
    across block assembly and row-state updates, and reuses shared transformed
    anchors for the common two-tangent contact-row case, instead of recomputing
    tangent anchors for force projection. Source
    Spring/Spring Ratio work also has a private finite-stiffness rigid radial
    distance-spring row primitive with focused
    `AvbdRigidBlock.PointPairDistanceSpring*` coverage, private serial rigid
    row-driver participation in
    `AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch`, and a first
    `World::addRigidBodyDistanceSpring` extraction path feeding the contact-stage
    rigid row solve with `RigidWorldDistanceSpringApiFeedsRadialRows` and
    Python API coverage. Capped rigid finite-stiffness point-pair, angular-pair,
    and distance-spring rows now skip row-state constraint recomputation once
    their effective stiffness is already at the material cap, with
    `AvbdRigidBlock.PointPairFiniteUpdateKeepsCappedStiffness`,
    `AvbdRigidBlock.RigidAngularFiniteUpdateKeepsCappedStiffness`, and
    `AvbdRigidBlock.PointPairDistanceSpringUpdateKeepsCappedStiffness`
    coverage. The first `avbd_demo2d_spring` and
    `avbd_demo2d_spring_ratio` source-scene harnesses, benchmark rows, and
    native source timing entrypoints now exist with tracked
    visual/DART-benchmark/native-reference packets. CPU performance resolution
    and GPU parity still do not exist. Future contact-manifold generation can
    reuse the same 6-DOF stamping path. The
    first private rigid contact-manifold builder now converts active contact
    points with stable endpoint feature IDs into warm-started normal rows and
    paired tangent rows for the private serial rigid row driver. The first
    private World-contact snapshot helper now extracts rigid contacts from
    `World::collide()` into that row-builder input and can drive the private
    serial rigid row solve plus dynamic rigid-body ECS writeback through a
    combined private wrapper. The first contact-stage AVBD activation now uses
    the internal `RigidAvbdContactConfig` for supported free rigid-body
    contacts: the stage predicts inertial targets from current velocities,
    solves the private 6-DOF rigid rows, and projects the solved displacement
    back into the velocity consumed by the standard position stage. Focused
    World coverage in
    `World.RigidBodyContactStageAvbdProjectsDynamicDynamicContactVelocity`
    and
    `World.RigidBodyContactStageAvbdProjectsStaticDynamicContactVelocity`
    now checks live dynamic/dynamic and static/dynamic contact-stage paths
    project separating velocities without directly moving poses,
    `World.RigidBodyContactStageAvbdProjectsDynamicPairWithSingleConfig`
    verifies dynamic/dynamic activation when only one endpoint is configured,
    `World.RigidBodyContactStageAvbdDynamicDynamicRunsThroughDefaultWorldStep`
    verifies both-configured dynamic/dynamic projection through the built-in
    `World::step()` schedule,
    `World.RigidBodyContactStageAvbdDynamicPairWithSingleConfigRunsThroughDefaultWorldStep`
    verifies single-config dynamic/dynamic projection through the built-in
    `World::step()` schedule,
    `World.RigidBodyContactStageAvbdFeedsRigidBodyPositionStage` verifies the
    projected velocity advances the dynamic body when the rigid position stage
    follows the contact stage,
    `World.RigidBodyContactStageAvbdRunsThroughDefaultWorldStep` verifies the
    same path through the built-in `World::step()` schedule, and
    `World.RigidBodyContactStageAvbdProjectsStaticOwnedContactConfig` verifies
    the static-owned static/dynamic opt-in path and
    `World.RigidBodyContactStageAvbdStaticOwnedRunsThroughDefaultWorldStep`
    verifies the same static-owned path through the built-in `World::step()`
    schedule.
    `World.RigidBodyContactStageAvbdIgnoresStoredStaticVelocity` verifies that
    stored static-body velocities do not perturb the static-owned AVBD
    projection.
    `World.RigidBodyContactStageAvbdTreatsKinematicBodyAsStaticObstacle`
    verifies kinematic contact endpoints stay fixed and their prescribed
    velocity does not perturb AVBD projection,
    `World.RigidBodyContactStageAvbdKinematicRunsThroughDefaultWorldStep`
    verifies the same prescribed-endpoint behavior through the built-in
    `World::step()` schedule, and
    `World.RigidBodyContactStageAvbdProjectsEnabledPeerWithDisabledConfig`
    verifies an explicitly disabled peer config does not veto an enabled
    endpoint.
    `World.RigidBodyContactStageAvbdEnabledPeerWithDisabledConfigRunsThroughDefaultWorldStep`
    verifies the same enabled-peer/disabled-peer activation through the
    built-in `World::step()` schedule. The
    `World.RigidBodyContactStageAvbdProjectsMultipleConfiguredContacts` checks
    the same velocity projection across a configured multi-contact static/dynamic
    set.
    `World.RigidBodyContactStageAvbdMultipleConfiguredContactsRunThroughDefaultWorldStep`
    verifies configured multi-contact projection through the built-in
    `World::step()` schedule. The companion
    `World.RigidBodyContactStageAvbdFallsBackForUnconfiguredContactSet`
    regression checks that a mixed live contact set with any unconfigured
    contact falls back as a whole instead of partially activating AVBD for the
    configured pair,
    `World.RigidBodyContactStageAvbdMixedConfigFallsBackThroughDefaultWorldStep`
    verifies the same all-or-nothing fallback through the built-in
    `World::step()` schedule, and
    `World.RigidBodyContactStageAvbdDisabledConfigFallsBack` checks that an
    explicitly disabled private contact config also opts out to the ordinary
    rigid contact response, while
    `World.RigidBodyContactStageAvbdDisabledConfigFallsBackThroughDefaultWorldStep`
    verifies the same disabled opt-out through the built-in `World::step()`
    schedule. The
    `World.RigidBodyContactStageAvbdWarmStartedFrictionReducesSlide` and
    `World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionReducesSlip`
    regressions check the live contact-stage friction tangent rows with
    retained normal-dual warm starting, comparing frictionless and frictional
    static ground/dynamic sphere and dynamic/dynamic sphere contacts and
    verifying reduced contact-point slip. The
    `World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionReducesSlide`
    regression verifies the same friction path when the private opt-in config
    is owned by the static endpoint. The
    `World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionReducesSlide`
    regression verifies the adjacent kinematic-owned path while checking that a
    prescribed kinematic tangential velocity is preserved but ignored as contact
    slip input. The
    `World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionReducesSlide`
    regression verifies that an explicitly disabled peer config does not veto
    warm-started friction rows when the other endpoint owns an enabled private
    config. The
    `World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionReducesSlide`
    regression verifies that simultaneous configured static/dynamic contacts
    both feed warm-started friction rows in one contact-stage solve. The
    `World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionReducesSlide`
    regression verifies the same warm-started contact-stage friction path on a
    live box-box manifold with multiple narrow-phase contact points. The
    `World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionReducesSlip`
    regression extends the contact-stage manifold evidence to two dynamic
    boxes, checking reduced relative contact-point slip against a frictionless
    baseline. The
    `World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionReducesSlip`
    regression now covers a stacked live contact set containing a
    static/dynamic box manifold and a dynamic/dynamic box manifold, checking
    that warm-started friction reduces upper/lower contact-point slip in that
    coupled stack. The
    `World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionReducesSlip`
    regression broadens that contact-stage evidence to two independently
    sliding upper dynamic boxes on one supported lower dynamic box, checking
    that warm-started friction reduces both upper/lower contact-point slips.
    The
    `World.RigidBodyContactStageAvbdWarmStartedMultiTopBoxPileManifoldFrictionReducesSlip`
    regression now checks the two-lower, two-upper pile topology from the
    private row-persistence tests through the live contact-stage path. The
    companion
    `World.RigidBodyContactStageAvbdWarmStartedFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
    `World.RigidBodyContactStageAvbdWarmStartedMultiTopBoxPileManifoldFrictionRunsThroughDefaultWorldStep`,
    and
    `World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionRunsThroughDefaultWorldStep`
    regressions verify that the built-in `World::step()` schedule reaches
    dynamic-owned static/dynamic, static-owned static/dynamic, kinematic-owned
    static/dynamic, enabled-peer/disabled-peer static/dynamic, simultaneous
    multi-contact static/dynamic, box-manifold static/dynamic,
    dynamic/dynamic box-manifold, stacked static/dynamic plus dynamic/dynamic
    box-manifold, multi-top stacked box-manifold, two-lower/two-upper box-pile,
    and dynamic/dynamic sphere warm-started friction rows; broader contact-stage
    coverage remains open.
    The private
    rigid contact snapshot now derives box face/edge/corner, cylinder
    side/cap/rim, and capsule side/top-cap/bottom-cap endpoint feature IDs,
    scopes row ordinals per canonical endpoint pair, and assigns same-feature
    manifold rows from deterministic canonical-local contact-point ordering so
    narrow-phase contact order and swapped contact body endpoints do not swap
    warm-start state, with actual `World::collide()` sphere/plane contact-point
    replay, live sphere/mesh-face, sphere/mesh-edge, mesh-vertex replay, and
    mesh-face/mesh-edge/mesh-vertex small-pose persistence,
    and live box-box manifold box-face feature coverage through
    `AvbdContact.WorldCollideSameFeatureRowsIgnoreContactOrder` and
    `AvbdContact.WorldCollideMeshFaceRowsIgnoreContactOrder`,
    `AvbdContact.WorldCollideMeshEdgeRowsIgnoreContactOrder`,
    `AvbdContact.WorldCollideMeshFaceRowsPersistAcrossSmallPose`,
    `AvbdContact.WorldCollideMeshEdgeRowsPersistAcrossSmallPose`,
    `AvbdContact.WorldCollideMeshVertexRowsIgnoreContactOrder`,
    `AvbdContact.WorldCollideMeshVertexRowsPersistAcrossSmallPose`,
    mesh face/edge/vertex endpoint-order warm-start coverage through
    `AvbdContact.WorldCollideMeshFaceRowsIgnoreEndpointOrder`,
    `AvbdContact.WorldCollideMeshEdgeRowsIgnoreEndpointOrder`, and
    `AvbdContact.WorldCollideMeshVertexRowsIgnoreEndpointOrder`,
    `AvbdContact.WorldCollideSameFeatureRowsIgnoreEndpointOrder`, plus
    live sphere/plane friction tangent warm-start mapping through
    `AvbdContact.WorldCollideFrictionRowsIgnoreContactOrder`,
    live sphere/plane normal-rotation friction tangent projection through
    `AvbdContact.WorldCollideFrictionRowsProjectAcrossChangingPlaneNormal`,
    `AvbdContact.WorldCollideLiveManifoldSameFeatureRowsIgnoreContactOrder`
    and
    `AvbdContact.WorldCollideLiveManifoldSameFeatureRowsIgnoreEndpointOrder`,
    live endpoint-swapped box-box manifold friction tangent projection through
    `AvbdContact.WorldCollideLiveManifoldFrictionRowsIgnoreEndpointOrder`,
    live positive/negative cylinder-cap/plane row-order coverage through
    `AvbdContact.WorldCollideCylinderCapRowsIgnoreContactOrder`, live
    positive/negative capsule-cap/plane row-order coverage through
    `AvbdContact.WorldCollideCapsuleCapRowsIgnoreContactOrder`,
    cylinder-side plus capsule-side row-order coverage through
    `AvbdContact.WorldCollideCylinderSideRowsIgnoreContactOrder` and
    `AvbdContact.WorldCollideCapsuleSideRowsIgnoreContactOrder`,
    positive/negative cylinder-rim row-order coverage through
    `AvbdContact.WorldCollideCylinderRimRowsIgnoreContactOrder`,
    cylinder/capsule cap/side/rim small-pose persistence through
    `AvbdContact.WorldCollideCylinderCapRowsPersistAcrossSmallPose`,
    `AvbdContact.WorldCollideCylinderSideRowsPersistAcrossSmallPose`,
    `AvbdContact.WorldCollideCylinderRimRowsPersistAcrossSmallPose`,
    `AvbdContact.WorldCollideCapsuleCapRowsPersistAcrossSmallPose`, and
    `AvbdContact.WorldCollideCapsuleSideRowsPersistAcrossSmallPose`,
    cylinder/capsule cap/side/rim endpoint-order warm-start coverage through
    `AvbdContact.WorldCollideCylinderCapRowsIgnoreEndpointOrder`,
    `AvbdContact.WorldCollideCylinderSideRowsIgnoreEndpointOrder`,
    `AvbdContact.WorldCollideCylinderRimRowsIgnoreEndpointOrder`,
    `AvbdContact.WorldCollideCapsuleCapRowsIgnoreEndpointOrder`, and
    `AvbdContact.WorldCollideCapsuleSideRowsIgnoreEndpointOrder`, and live
    box-box small-pose row-persistence coverage through
    `AvbdContact.WorldCollideLiveManifoldSameFeatureRowsPersistAcrossSmallPose`,
    plus live box-box manifold friction warm-start persistence through
    `AvbdContact.WorldCollideLiveManifoldFrictionRowsPersistAcrossSmallPose`;
    stacked static/dynamic and dynamic/dynamic box manifolds now also preserve
    row identity across a small pose-preserving nudge through
    `AvbdContact.WorldCollideStackedManifoldsPersistRowsAcrossSmallPose`, and
    their paired friction tangent rows now preserve warm-started dual state and
    per-pair Coulomb bounds through
    `AvbdContact.WorldCollideStackedManifoldsFrictionRowsPersistAcrossSmallPose`
    plus raw contact-order replay through
    `AvbdContact.WorldCollideStackedManifoldsFrictionRowsIgnoreContactOrder`
    plus endpoint-swapped physical tangent-dual projection through
    `AvbdContact.WorldCollideStackedManifoldsFrictionRowsIgnoreEndpointOrder`.
    `AvbdContact.WorldCollideBoxPileFrictionRowsPersistAcrossSmallPose` now
    broadens the private row-persistence evidence to a wider live box pile with
    two lower dynamic supports on a static ground and one top body spanning both
    supports, preserving per-pair row ordinals, friction coefficients, Coulomb
    bounds, and warm-started friction dual state across a small pose nudge.
    `AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsPersistAcrossSmallPose`
    adds a distinct pile topology with two lower dynamic supports and two
    independent upper dynamic bodies, preserving those same per-pair row
    ordinals, friction coefficients, Coulomb bounds, and warm-started friction
    duals across a small pose nudge.
    `AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsIgnoreContactOrder`
    now replays that same multi-top pile with the live contact vector reversed,
    preserving row ordinals, per-pair friction coefficients, Coulomb bounds,
    and warm-started normal/friction lambdas independent of raw narrow-phase
    contact order.
    `AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsIgnoreEndpointOrder`
    adds the endpoint-swapped companion for that multi-top pile, preserving
    per-pair Coulomb bounds and projecting paired tangent duals into the swapped
    physical tangent basis across independent upper/lower dynamic pairs and
    lower/ground supports.
    `AvbdContact.WorldCollideBoxPileFrictionRowsIgnoreContactOrder` adds the
    reversed-contact replay for that wider pile, preserving row ordinals,
    per-pair friction coefficients, Coulomb bounds, and warm-started normal/
    friction lambdas independent of raw narrow-phase contact order.
    `AvbdContact.WorldCollideBoxPileFrictionRowsIgnoreEndpointOrder` adds the
    endpoint-swapped companion for that wider pile, preserving the same
    per-pair Coulomb bounds and projecting paired tangent duals into the swapped
    physical tangent basis.
    `World.RigidBodyContactStageAvbdWarmStartedBoxPileManifoldFrictionReducesSlip`
    and
    `World.RigidBodyContactStageAvbdWarmStartedBoxPileManifoldFrictionRunsThroughDefaultWorldStep`
    now carry the same wider pile through the live contact stage and built-in
    default schedule, proving warm-started friction reduces top/lower tangential
    slip. These are still narrow pile-style evidence slices, not broad
    stack/pile parity.
    Known and
    unknown-index fallback contacts now map world points through body and
    collision-shape local transforms before feature coding, while explicit
    endpoint-A and endpoint-B compound shape-index contacts use the
    narrow-phase shape-local contact point before feature coding, with actual
    `World::collide()` sphere/cylinder/capsule/plane/mesh contacts now covered through
    `AvbdContact.PrimitiveFeatureKeysUseNarrowPhaseShapeLocalPoints` and
    `AvbdContact.PlaneAndMeshFeatureKeysUseNarrowPhaseShapeLocalPoints`,
    including live mesh vertex endpoint-feature evidence.
    Single-shape and uniquely containing compound-shape unknown-index fallback
    coverage preserves inferred offset-shape row identity. Unique compound
    plane contacts derive
    shape-scoped face IDs, triangle-mesh contacts derive face/edge/vertex
    feature IDs, and
    `AvbdRigidBlock.RigidWorldContactSnapshotMapsExplicitEndpointAShapeToShapeFrame`,
    `RigidWorldContactSnapshotMapsExplicitEndpointBShapeToShapeFrame`,
    `RigidWorldContactSnapshotUsesNarrowPhaseLocalPointForExplicitShapeFeature`,
    plus `RigidWorldContactSnapshotMapsSingleShapeFallbackToShapeFrame`,
    `RigidWorldContactSnapshotInfersUniqueCompoundShapeFallback`,
    `RigidWorldContactSnapshotLeavesAmbiguousCompoundShapeFallbackAtBody`, and
    plane/mesh inference coverage preserve offset-shape row identity and
    conservative ambiguous fallback. Persisting private rigid contact friction
    rows now store their previous tangent directions and project the
    warm-started paired dual into the current tangent basis when the contact
    normal rotates, with
    `AvbdRigidBlock.RigidContactManifoldFrictionProjectsWarmStartedDualAcrossTangentBasis`
    coverage.
    Private point-joint linear, angular, and combined row builders now create the
    first
    rigid joint-row families for fixed-anchor translation and orientation
    constraints through the existing rigid row driver, seed their step-start
    constraint values for AVBD alpha regularization, and can be appended to the
    private World rigid snapshot/solve/apply wrapper and combined step helper
    from world-space point-joint inputs, including a private fixed-joint ECS
    extractor and step-helper overload for rigid-body-linked joint entities.
    The internal contact-stage AVBD opt-in can also project those fixed-joint
    rows with or without active contact rows. A private current-pose bridge can
    now derive AVBD local anchors and target relative orientation for
    rigid-body-linked fixed joints. Missing private AVBD fixed-joint configs are
    initialized from the simulation-entry pose for opt-in rigid bodies; the same
    current-pose bridge now also accepts non-topology multibody-link point-joint
    entities, while still skipping tree-topology joints. The private
    point-joint builders now also accept per-axis linear and angular masks,
    preserving the all-axis
    fixed-joint behavior while letting future hinge/revolute and limited-DOF row
    configs reuse the same descriptors and warm-start inventory. The same
    private row path now has named revolute and prismatic point-joint builders
    that construct arbitrary joint-axis bases, leave the hinge or translation
    axis free, and preserve the configured axes/masks through World point-joint
    input, snapshot assembly, and solve coverage. Simulation-entry
    current-pose initialization and extraction now also cover private
    rigid-body ECS revolute/prismatic joint entities, deriving the same
    axes/masks from their configured joint axis while tree-topology joints
    remain skipped. The private endpoint layer
    now classifies free rigid bodies separately from multibody links, and
    explicitly hard fixed multibody-link point-joint configs can bridge into the
    variational articulated solve path.
    Explicitly hard masked multibody-link point-joint configs now also bridge
    into the same variational solve path for private AVBD revolute and
    prismatic rows, projecting only the active linear/angular axes while leaving
    the configured hinge or slider axis free. Focused
    `VariationalIntegration.Avbd*MaskedFloatingLinkEndpoint` tests cover the
    private floating-link extraction and solve behavior. Explicitly hard
    breakable multibody-link point-joint configs now also enter that projection
    path and mark their source joint broken when the projection load exceeds the
    configured threshold, with focused fixed mark/reset, masked-revolute, and
    prismatic-velocity-motor coverage for the first articulated fracture bridge.
    The variational articulated projection path now also handles public one-DOF
    `Velocity` actuators on revolute and prismatic topology joints as coordinate
    motor targets, with focused single and coupled-chain regression coverage.
    The same bridge now extracts private
    hard revolute and prismatic AVBD point-joint velocity actuators on
    articulated endpoints as projected free-axis angular/linear motor rows,
    with focused accumulation, child-/parent-endpoint revolute/prismatic
    command-update lifecycle including non-cardinal child-/parent-endpoint
    axis-basis coverage, and child-/parent-endpoint tiny positive effort-limit
    coverage including non-cardinal axis-basis cases in
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorDrivesMaskedFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorRespectsTinyTorqueLimit`,
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorRespectsTinyTorqueLimitOnNonCardinalAxis`,
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorParentEndpointRespectsTinyTorqueLimit`,
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorParentEndpointRespectsTinyTorqueLimitOnNonCardinalAxis`,
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorFollowsCommandUpdates`,
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorFollowsNonCardinalCommandUpdates`,
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorParentEndpointFollowsCommandUpdates`,
    `VariationalIntegration.AvbdRevolutePointJointVelocityActuatorParentEndpointFollowsNonCardinalCommandUpdates`,
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorFollowsCommandUpdates`,
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorFollowsNonCardinalCommandUpdates`,
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorParentEndpointFollowsCommandUpdates`,
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorParentEndpointFollowsNonCardinalCommandUpdates`,
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorRespectsTinyForceLimit`,
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorRespectsTinyForceLimitOnNonCardinalAxis`,
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorParentEndpointRespectsTinyForceLimit`,
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorParentEndpointRespectsTinyForceLimitOnNonCardinalAxis`,
    and
    `VariationalIntegration.AvbdPrismaticPointJointVelocityActuatorDrivesMaskedFloatingLinkEndpoint`,
    while
    `VariationalIntegration.AvbdBreakablePointJointConfigResetReengagesFixedRows`,
    `VariationalIntegration.AvbdBreakableChildPointJointConfigResetReengagesFixedRows`,
    `VariationalIntegration.AvbdBreakableSphericalPointJointConfigSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdBreakableSphericalChildPointJointConfigSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdBreakableRevoluteVelocityPointJointConfigResetReengagesMotorRows`,
    `VariationalIntegration.AvbdBreakableRevoluteVelocityParentPointJointConfigResetReengagesMotorRows`,
    `VariationalIntegration.AvbdBreakableRevoluteVelocityParentPointJointConfigSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdBreakablePrismaticVelocityParentPointJointConfigSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdBreakablePrismaticVelocityPointJointConfigMarksMaskedFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdBreakablePrismaticVelocityPointJointConfigResetReengagesMotorRows`,
    and
    `VariationalIntegration.AvbdBreakablePrismaticVelocityParentPointJointConfigResetReengagesMotorRows`
    cover private fixed/spherical binary save/load reset, fracture row
    indexing, revolute/prismatic parent/child world-link save/load/reset
    polarity, and child-/parent-endpoint revolute/prismatic reset
    re-engagement when those free-axis motor rows participate.
    Finite-stiffness private AVBD fixed point-joint configs on articulated
    endpoints now also contribute compliant variational forces through
    persistent per-axis finite-stiffness row state. The variational bridge
    warm-starts those rows across steps and ramps stiffness toward the
    configured cap from converged linear/angular residuals, with focused
    floating-link coverage in
    `VariationalIntegration.AvbdCompliantPointJointConfigPullsLinkEndpoint` and
    `VariationalIntegration.AvbdCompliantPointJointConfigRampsStiffnessAcrossSteps`.
    Simulation-entry current-pose extraction now generates hard fixed,
    revolute, prismatic, and spherical private point-joint configs for
    non-topology multibody-link endpoints, with focused coverage in
    `VariationalIntegration.AvbdFixedPointJointCurrentPoseExtractorWiresFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdFixedPointJointCurrentPoseExtractorWiresParentWorldLinkEndpoint`,
    `VariationalIntegration.AvbdSphericalPointJointCurrentPoseExtractorWiresWorldLinkEndpointPolarities`,
    `VariationalIntegration.AvbdRevolutePointJointCurrentPoseExtractorDrivesFloatingLinkEndpoint`,
    and
    `VariationalIntegration.AvbdPrismaticPointJointCurrentPoseExtractorDrivesFloatingLinkEndpoint`,
    where the generated fixed and spherical world-link coverage checks
    parent-link endpoint polarity and the generated revolute/prismatic motor
    coverage now uses non-cardinal axes and checks the generated free-axis
    basis column,
    while the public articulated revolute/prismatic floating-endpoint and
    selected off-origin-anchor facade drive coverage now also covers
    non-cardinal axes and checks the generated AVBD basis column before
    stepping, and selected same-multibody/world-link facade save/load coverage
    now preserves non-cardinal revolute/world-prismatic axes and checks the
    rebuilt AVBD basis columns after simulation entry; selected one-DOF
    broken-state save/load/reset coverage now also preserves non-cardinal
    revolute/prismatic axes and checks the rebuilt AVBD basis columns after
    reset re-engagement, and selected direct same-multibody/world-link
    one-DOF break/skip/reset coverage now also uses non-cardinal axes and
    checks the generated AVBD basis columns before/after re-engagement, with
    dartpy same-multibody pair and world-link revolute/prismatic reset coverage
    now also exercising non-cardinal explicit-anchor axes,
    plus private movable-pair fixed/revolute/prismatic current-pose break/reset
    coverage in
    `VariationalIntegration.AvbdFixedPointJointCurrentPoseExtractorResetReengagesMovablePair`,
    `VariationalIntegration.AvbdRevolutePointJointCurrentPoseExtractorResetReengagesMovablePair`
    and
    `VariationalIntegration.AvbdPrismaticPointJointCurrentPoseExtractorResetReengagesMovablePair`,
    and private movable-pair spherical break/reset coverage in
    `VariationalIntegration.AvbdSphericalPointJointCurrentPoseExtractorResetReengagesMovablePair`.
    The generated current-pose revolute and prismatic motor paths also have
    finite effort-limit coverage in
    `VariationalIntegration.AvbdRevolutePointJointCurrentPoseExtractorRespectsTinyTorqueLimitOnMovablePair`,
    and
    `VariationalIntegration.AvbdPrismaticPointJointCurrentPoseExtractorRespectsTinyForceLimitOnMovablePair`,
    proving their free-axis motor rows honor tiny finite caps while the captured
    anchor, hinge, and masked hard rows remain active.
    The public same-multibody/world-link revolute/prismatic one-DOF facade
    tiny-limit paths now also use non-cardinal axes, off-origin anchors, and
    generated AVBD basis-column assertions in
    `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeRespectsTinyTorqueLimit`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeRespectsTinyForceLimit`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteJointFacadeRespectsTinyTorqueLimit`,
    and
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticJointFacadeRespectsTinyForceLimit`,
    proving the free-axis motor remains bounded without relying on cardinal
    basis or origin-anchor assumptions.
    The public same-multibody movable-pair revolute/prismatic facade motor path
    now also has non-cardinal axis-basis and finite effort-limit coverage in
    `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeMovablePairRespectsTinyTorqueLimit`
    and
    `VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeMovablePairRespectsTinyForceLimit`,
    proving both movable endpoints preserve the hard rows while the bounded
    free-axis motor stays effectively capped.
    The variational projection now re-solves unbounded hard rows after clamping
    bounded free-axis motor impulses, and
    `test_simulation_world_articulated_non_cardinal_off_origin_tiny_effort_motors_from_python`
    verifies same-multibody/world-link revolute and prismatic Python facades
    keep off-origin hard rows locked under saturated tiny effort caps.
    The dartpy `World.save_binary(path)` and `World.load_binary(path)` bindings
    now expose the existing binary serializer to Python, and
    `test_simulation_world_articulated_binary_roundtrip_from_python` and
    `test_simulation_world_articulated_binary_roundtrip_from_python_completes_one_dof_endpoint_types`
    verify all same-multibody/world-link revolute/prismatic broken one-DOF
    motor rows preserve endpoint/type/axis/command/effort-limit state through
    Python round-trips before reset rebuilds the hard rows and reversed
    free-axis motor motion, while
    `test_simulation_world_articulated_fixed_spherical_binary_roundtrip_from_python`
    and
    `test_simulation_world_articulated_fixed_spherical_binary_roundtrip_from_python_completes_endpoint_types`
    verify all same-multibody/world-link fixed/spherical broken rows preserve
    restored endpoint/type shape through Python round-trips before reset
    rebuilds fixed all-axis rows and spherical linear rows while leaving
    spherical orientation free, and
    `test_simulation_world_articulated_design_binary_rebuild_from_python` saves
    unbroken same-multibody fixed, same-multibody/world-link
    revolute/prismatic velocity-motor, and world-link spherical public facades,
    while
    `test_simulation_world_articulated_design_binary_rebuild_from_python_completes_fixed_spherical_endpoint_types`
    saves the complementary world-link fixed and same-multibody spherical
    public facades before simulation mode, reloads them through dartpy, and
    proves simulation entry rebuilds the private all-axis fixed rows, one-DOF
    hard rows plus free-axis motor rows, and spherical linear rows while leaving
    spherical orientation free.
    The public same-multibody movable-pair revolute/prismatic broken-state
    binary save/load/reset path now has focused C++ coverage in
    `VariationalIntegration.AvbdPublicArticulatedMovableRevoluteBreakageSurvivesSaveLoadAndReset`
    and
    `VariationalIntegration.AvbdPublicArticulatedMovablePrismaticBreakageSurvivesSaveLoadAndReset`,
    preserving restored endpoint/type/axis/actuator command and effort-limit
    state and proving reset re-enables the hard rows plus updated free-axis
    motor motion across two movable links.
    The private generated current-pose movable-pair fixed broken-state binary
    save/load/reset path now has focused C++ coverage in
    `VariationalIntegration.AvbdFixedPointJointCurrentPoseExtractorBreakageSurvivesSaveLoad`,
    preserving restored endpoints and captured current-pose anchors before
    proving reset re-enables both the generated linear and angular hard rows.
    The private generated current-pose movable-pair revolute/prismatic
    broken-state binary save/load/reset path now has focused C++ coverage in
    `VariationalIntegration.AvbdRevolutePointJointCurrentPoseExtractorBreakageSurvivesSaveLoad`
    and
    `VariationalIntegration.AvbdPrismaticPointJointCurrentPoseExtractorBreakageSurvivesSaveLoad`,
    preserving restored endpoint/type/axis/actuator command, effort-limit
    state, and captured current-pose anchors before proving reset re-enables
    the generated hard rows plus updated free-axis motor motion across two
    movable links.
    Direct private world-link revolute/prismatic point-joint configs now also
    persist through broken-state binary save/load in
    `VariationalIntegration.AvbdBreakableRevoluteVelocityPointJointConfigSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdBreakableRevoluteVelocityParentPointJointConfigSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdBreakablePrismaticVelocityPointJointConfigSurvivesSaveLoadAndReset`,
    and
    `VariationalIntegration.AvbdBreakablePrismaticVelocityParentPointJointConfigSurvivesSaveLoadAndReset`,
    preserving the private `AvbdRigidWorldPointJointConfig` basis, masks,
    anchors, endpoint shape, actuator command, and effort limits across
    child-link and parent-link endpoint polarity before proving reset
    re-engages the hard rows plus updated free-axis motor motion. The direct
    in-memory reset/re-engagement path now also covers the parent-link
    world-link endpoint polarity in
    `VariationalIntegration.AvbdBreakableRevoluteVelocityParentPointJointConfigResetReengagesMotorRows`
    and
    `VariationalIntegration.AvbdBreakablePrismaticVelocityParentPointJointConfigResetReengagesMotorRows`,
    proving the opposite free-axis sign is rebuilt without a binary round trip.
    Generated current-pose same-multibody movable-pair prismatic finite-force
    coverage now uses a non-cardinal slider basis in
    `VariationalIntegration.AvbdPrismaticPointJointCurrentPoseExtractorRespectsTinyForceLimitOnMovablePair`,
    proving the generated hard rows preserve the captured pose while the
    finite motor row remains bounded off the cardinal axes.
    The private generated current-pose movable-pair spherical broken-state
    binary save/load/reset path now has focused C++ coverage in
    `VariationalIntegration.AvbdSphericalPointJointCurrentPoseExtractorBreakageSurvivesSaveLoad`,
    preserving restored endpoints and captured current-pose anchors before
    proving reset re-enables only the generated linear rows while relative
    orientation remains free. The generated private fixed/spherical
    current-pose paths now also cover parent-link world-link endpoint polarity
    in
    `VariationalIntegration.AvbdFixedPointJointCurrentPoseExtractorWiresParentWorldLinkEndpoint`
    and
    `VariationalIntegration.AvbdSphericalPointJointCurrentPoseExtractorWiresWorldLinkEndpointPolarities`,
    proving simulation entry builds all-axis fixed rows or linear-only
    spherical anchor rows while spherical orientation remains free. Direct
    private world-link fixed save/load/reset now covers both parent-link and
    child-link floating endpoint polarity in
    `VariationalIntegration.AvbdBreakableParentPointJointConfigResetReengagesFixedRows`
    and
    `VariationalIntegration.AvbdBreakableChildPointJointConfigResetReengagesFixedRows`
    while preserving all-axis fixed rows, endpoint shape, off-origin anchors,
    and parent-side target orientation before reset re-engages the fixed rows.
    Direct private world-link spherical save/load/reset likewise covers
    parent-link and child-link endpoint polarity in
    `VariationalIntegration.AvbdBreakableSphericalPointJointConfigSurvivesSaveLoadAndReset`
    and
    `VariationalIntegration.AvbdBreakableSphericalChildPointJointConfigSurvivesSaveLoadAndReset`,
    preserving spherical linear-only masks and endpoint shape before reset
    re-engages the appropriate row set while orientation remains free.
    Public DART 7 `World` facades now expose free rigid-body revolute and
    prismatic joints through C++ and dartpy, including generated stubs,
    focused tests, and the categorized `sx_rigid_limited_joints` py-demo.
    A first user-visible AVBD-specific `py-demos` scene,
    `avbd_rigid_fixed_joint_contact`, now shows the public fixed-joint rows
    preserving a captured rigid offset while ordinary rigid contact acts on the
    payload; this is only a narrow AVBD rigid-constraint showcase, not a
    source-demo or paper-scene reproduction.
    The next motor/fracture slice started with `AvbdRigidAngularMotor`,
    `AvbdRigidLinearMotor`, bounded motor row descriptors/builders, and rigid
    fracture dual-threshold/reset helpers in the same 6-DOF row kernel, with
    focused `AvbdRigidBlock.*` coverage. Public rigid-body revolute velocity
    actuators now extract to private AVBD angular motor rows, public rigid-body
    prismatic velocity actuators extract to the private AVBD linear motor row,
    and the contact-stage AVBD velocity projection keeps a persistent motor row
    inventory. The categorized `avbd_rigid_revolute_motor` and
    `avbd_rigid_prismatic_motor` py-demos show the public one-DOF actuator
    paths alongside tracked visual/benchmark packets and the existing
    fixed-joint/contact showcase. The performance
    dashboard's AVBD World slice now also records end-to-end
    `BM_AvbdEmptyWorldStep`,
    `BM_AvbdDemo2dGroundStep`,
    `BM_AvbdDemo2dMotorStep`,
    `BM_AvbdDemo2dHangingRopeStep`,
    `BM_AvbdDemo2dSpringStep`,
    `BM_AvbdDemo2dSpringRatioStep`,
    `BM_AvbdDemo2dFractureStep`,
    `BM_AvbdDemo2dDynamicFrictionStep`,
    `BM_AvbdDemo2dStaticFrictionStep`,
    `BM_AvbdDemo2dPyramidStep`,
    `BM_AvbdDemo2dCardsStep`,
    `BM_AvbdDemo2dStackStep`,
    `BM_AvbdDemo2dStackRatioStep`,
    `BM_AvbdDemo2dRodStep`,
    `BM_AvbdDemo2dSoftBodyStep`,
    `BM_AvbdDemo2dJointGridStep`,
    `BM_AvbdDemo2dRopeStep`,
    `BM_AvbdDemo2dHeavyRopeStep`,
    `BM_AvbdDemo2dNetStep`,
    `BM_AvbdDemo3dGroundStep`,
    `BM_AvbdDemo3dDynamicFrictionStep`,
    `BM_AvbdDemo3dStaticFrictionStep`,
    `BM_AvbdDemo3dPyramidStep`,
    `BM_AvbdDemo3dRopeStep`,
    `BM_AvbdDemo3dHeavyRopeStep`,
    `BM_AvbdDemo3dStackStep`,
    `BM_AvbdDemo3dStackRatioStep`,
    `BM_AvbdDemo3dBreakableStep`,
    `BM_AvbdRigidRevoluteMotorStep`,
    `BM_AvbdRigidPrismaticMotorStep`,
    `BM_AvbdArticulatedRevoluteMotorStep`,
    `BM_AvbdArticulatedPrismaticMotorStep`, and
    `BM_AvbdArticulatedBreakableMotorStep` rows for the public free-rigid and
    articulated one-DOF velocity motor paths, including the active break-force
    armed articulated motor path, plus
    `BM_AvbdRigidBreakableJointStep` and
    `BM_AvbdArticulatedBreakableJointStep` rows for the public free-rigid and
    articulated breakable fixed point-joint paths, with tracked
    `avbd-rigid-breakable-joint-packet.json` and
    `avbd-articulated-breakable-joint-packet.json` evidence, plus
    `BM_AvbdRigidSphericalBreakableJointStep`,
    `BM_AvbdArticulatedWorldSphericalBreakableJointStep`, and
    `BM_AvbdArticulatedSphericalPairBreakableJointStep` rows with matching
    spherical packet evidence. The narrow public fracture
    lifecycle now exposes a
    non-negative `Joint` break-force threshold and broken-state reset/accessors,
    maps that threshold into free rigid-body AVBD point-joint rows, marks joints
    broken when the solved row load reaches the threshold, skips broken joints on
    later extraction, and adds the categorized `avbd_rigid_breakable_joint`
    py-demo with focused break, release, reset, and captured-pose
    re-engagement coverage plus `avbd_rigid_spherical_breakable_joint` coverage
    for free-rigid spherical break/reset with orientation left free.
    The private endpoint classifier now separates free rigid-body endpoints from
    multibody links, explicitly hard fixed multibody-link point-joint configs
    can bridge into the variational articulated solve path, and
    `BM_AvbdRigidEndpointClassification` plus focused C++ variational tests
    cover that first bridge. The categorized
    `variational_endpoint_loop_closure` py-demo previews the related public
    multibody-link point-closure behavior, but it does not exercise the private
    AVBD point-joint config extractor. Simulation-entry extraction now covers
    non-topology multibody-link fixed/revolute/prismatic point-joint entities
    through focused C++ tests. Public DART 7 `World` articulated
    fixed/revolute/prismatic/spherical facades now create same-multibody
    link-link and world-link non-topology point joints through C++ and dartpy,
    expose lookup/list/count helpers, and feed that simulation-entry
    current-pose extractor; focused coverage in
    `VariationalIntegration.AvbdPublicArticulatedFixedJointFacadeWiresFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdPublicArticulatedFixedJointFacadeDrivesOffsetAnchorPair`,
    `VariationalIntegration.AvbdPublicArticulatedFixedJointFacadeSurvivesSaveLoad`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeDrivesFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeRespectsTinyTorqueLimit`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeFollowsCommandUpdates`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeDrivesMovableLinkPair`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeDrivesOffsetAnchorPair`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeSurvivesSaveLoad`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeDrivesFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeRespectsTinyForceLimit`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeFollowsCommandUpdates`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeDrivesMovableLinkPair`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeDrivesOffsetAnchorPair`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeSurvivesSaveLoad`,
    `VariationalIntegration.AvbdPublicArticulatedSphericalJointFacadeLeavesOrientationFree`,
    `VariationalIntegration.AvbdPublicArticulatedWorldSphericalJointFacadeDrivesOffsetAnchor`,
    `VariationalIntegration.AvbdPublicArticulatedWorldSphericalJointFacadeSurvivesSaveLoad`,
    `VariationalIntegration.AvbdPublicArticulatedLinkSphericalJointFacadeSurvivesSaveLoad`,
    `VariationalIntegration.AvbdPublicArticulatedSphericalBreakForceMarksLinearRowsAndSkipsLaterSteps`,
    `VariationalIntegration.AvbdPublicArticulatedWorldSphericalBreakForceResetReengagesLinearRows`,
    `VariationalIntegration.AvbdPublicArticulatedWorldSphericalBreakageSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdPublicArticulatedLinkSphericalBreakageSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdPublicArticulatedBreakForceMarksJointBrokenAndSkipsLaterSteps`,
    `VariationalIntegration.AvbdPublicArticulatedBreakForceResetReengagesJoint`,
    `VariationalIntegration.AvbdPublicArticulatedBreakageSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdPublicArticulatedBreakForceResetReengagesMovableLinkPair`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteBreakForceResetReengagesMovableOffsetPair`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticBreakForceResetReengagesMovableOffsetPair`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteBreakForceMarksMotorRowsAndSkipsLaterSteps`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteBreakForceResetReengagesMotorRows`,
    `VariationalIntegration.AvbdPublicArticulatedRevoluteBreakageSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticBreakForceMarksMotorRowsAndSkipsLaterSteps`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticBreakForceResetReengagesMotorRows`,
    `VariationalIntegration.AvbdPublicArticulatedPrismaticBreakageSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdPublicArticulatedWorldFixedJointFacadeWiresFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdPublicArticulatedWorldFixedJointFacadeDrivesOffsetAnchor`,
    `VariationalIntegration.AvbdPublicArticulatedWorldFixedJointFacadeSurvivesSaveLoad`,
    `VariationalIntegration.AvbdPublicArticulatedWorldFixedBreakForceResetReengagesJoint`,
    `VariationalIntegration.AvbdPublicArticulatedWorldFixedBreakageSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteJointFacadeDrivesFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteJointFacadeDrivesOffsetAnchor`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteJointFacadeSurvivesSaveLoad`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteJointFacadeRespectsTinyTorqueLimit`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteJointFacadeFollowsCommandUpdates`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteBreakForceMarksMotorRowsAndSkipsLaterSteps`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteBreakForceResetReengagesMotorRows`,
    `VariationalIntegration.AvbdPublicArticulatedWorldRevoluteBreakageSurvivesSaveLoadAndReset`,
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticJointFacadeDrivesFloatingLinkEndpoint`,
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticJointFacadeDrivesOffsetAnchor`,
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticJointFacadeSurvivesSaveLoad`,
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticJointFacadeRespectsTinyForceLimit`,
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticJointFacadeFollowsCommandUpdates`,
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticBreakForceMarksMotorRowsAndSkipsLaterSteps`,
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticBreakForceResetReengagesMotorRows`,
    `VariationalIntegration.AvbdPublicArticulatedWorldPrismaticBreakageSurvivesSaveLoadAndReset`,
    and
    `test_simulation_world_articulated_point_joint_facade_exposes_link_endpoints`
    plus
    `test_simulation_world_articulated_fixed_breakage_reset_reengages_from_python`
    and
    `test_simulation_world_articulated_motor_breakage_steps_from_python`
    and
    `test_simulation_world_articulated_motor_breakage_reset_reengages_from_python`
    and
    `test_simulation_world_articulated_complementary_motor_breakage_reset_from_python`
    and
    `test_simulation_world_articulated_pair_motor_breakage_reset_from_python`
    and
    `test_simulation_world_articulated_pair_revolute_breakage_reset_from_python`
    and
    `test_simulation_world_articulated_spherical_pair_breakage_reset_reengages_from_python`
    and
    `test_simulation_world_articulated_spherical_breakage_steps_from_python`
    cover fixed-pose hold, spherical linear-only pinned-anchor behavior with
    free orientation including explicit link-link and world-link anchors plus
    same-multibody/world-link fixed save/load rebuilding of the private
    all-axis rows and same-multibody/world-link spherical save/load rebuilding
    of the private AVBD linear rows,
    spherical break/skip and same-multibody/world-link reset re-engagement of
    explicit linear anchor rows,
    revolute/prismatic velocity motors, public free-axis effort bounds for
    same-multibody and world-anchored motors, same-multibody and world-anchored
    command updates, same-multibody/world-link revolute/prismatic motor
    save/load rebuilding plus restored same-multibody fixed/revolute/prismatic
    and world-link spherical/revolute/prismatic endpoint-shape assertions,
    same-multibody/world-link fixed/spherical/revolute/prismatic broken-state
    save/load/reset persistence including explicit-anchor fixed,
    movable-movable fixed, and one-DOF motor rows with restored effort-limit
    state,
    explicit local/world
    anchor binding, movable-movable off-origin revolute/prismatic motor
    break/reset in C++ and dartpy, same-multibody and world-anchored off-origin
    fixed/revolute/prismatic anchor overloads plus revolute/prismatic motor
    projection,
    movable-movable same-multibody link-pair revolute/prismatic motor
    projection and break/reset,
    the narrow same-multibody and world-anchored public break/reset lifecycle
    including fixed point-joint break/skip/reset stepping from Python for
    same-multibody, movable-movable same-multibody, and world-link explicit
    all-axis anchor rows with endpoint-shape assertions, world-fixed reset,
    same-multibody/world-anchored one-DOF
    motor-row break/skip and reset/re-engagement with Python stepping coverage
    for same-multibody/world-link revolute and prismatic explicit-anchor cases
    with post-reset endpoint/axis-shape assertions including non-cardinal
    same-multibody pair and world-link reset paths plus C++/dartpy non-cardinal off-origin
    finite-effort cap checks spanning same-multibody/world-link revolute and
    prismatic facades plus dartpy binary round-trips that preserve all
    same-multibody/world-link fixed/spherical/revolute/prismatic broken states
    before reset re-engagement,
    same-multibody/world-link spherical linear-row break/skip/reset stepping
    with endpoint-shape assertions, and
    one-DOF motor-row re-engagement,
    world-anchored point-joint variants, and Python endpoint access for the
    first public facade bridge.
    The categorized `avbd_empty_baseline` py-demo now provides the first
    runnable source-corpus baseline for the 2D/3D empty-scene rows, with focused
    Python smoke coverage in `test_avbd_empty_baseline_demo_steps_empty_world`.
    That smoke now records the `avbd-demo2d`/`avbd-demo3d` source revisions,
    scene indices/counts, default timestep/gravity/iteration metadata, and
    `sceneEmpty` zero-count invariant. The tracked
    [`../../plans/104-vertex-block-descent-solver/avbd-empty-baseline-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-empty-baseline-packet.json)
    packet records matching headless visual-capture hashes and the
    `BM_AvbdEmptyWorldStep` Google Benchmark row.
    The categorized `avbd_demo2d_ground` py-demo now ports the static
    `avbd-demo2d` Ground source row with source revision/scene metadata, one
    static slab, one rigid body, one collision shape, no joints, and no dynamic
    bodies. Focused Python coverage in
    `test_avbd_demo2d_ground_scene_matches_source_row` checks those source-row
    invariants and stable static stepping, the dashboard slice includes
    `BM_AvbdDemo2dGroundStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-ground-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-ground-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. After skipping static-only contact queries,
    no-op rigid dynamics stages in static-only worlds, clean frame-cache graph
    execution, and the clean no-work default step pipeline with a cheap scratch
    reset, the refreshed packet records DART at 16.3 ns median CPU time per step
    versus 24.5 ns for the native static source runner on this host, closing the
    narrow source row while broad ground-contact, GPU, and paper-number gates
    remain open.
    The categorized `avbd_demo2d_motor` py-demo now ports the first one-DOF motor
    `avbd-demo2d` source row, the reference Motor scene: it records the
    source revision, source scene index, timestep/gravity/iteration defaults,
    20 rad/s target speed, and 50 N m effort bound, and uses the source static
    ground body as the parent anchor proxy for DART's public revolute motor
    facade. Focused Python coverage in
    `test_avbd_demo2d_motor_scene_matches_source_row` checks those source-row
    invariants, and the dashboard slice includes `BM_AvbdDemo2dMotorStep`.
    The tracked
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-motor-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-motor-packet.json)
    packet now records a headless visual capture, the DART benchmark row, and a
    native `avbd-demo2d` source timing runner. The rigid contact stage now skips
    contact queries for worlds with no collision geometry in prepare/execute,
    and the refreshed packet records this no-collision-geometry DART row at 8.87
    us median CPU time per step versus 1.435 us for the native source runner on
    this host, so the source row and CPU reference comparison remain partial.
    The categorized `avbd_demo2d_hanging_rope` py-demo now ports the
    `avbd-demo2d` Hanging Rope source row with source revision/scene metadata,
    49 regular links, one 10 m endpoint block, 49 linear-only public spherical
    point joints, and 50 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_hanging_rope_scene_matches_source_row` checks those
    source-row invariants, the dashboard slice includes
    `BM_AvbdDemo2dHangingRopeStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-hanging-rope-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-hanging-rope-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The refreshed packet records DART at 0.392 ms
    median CPU time per step versus 50.1 us for the native source runner on
    this host, so this remains partial source-corpus evidence rather than a
    parity claim.
    The categorized `avbd_demo2d_fracture` py-demo now ports the
    `avbd-demo2d` Fracture source row with source revision/scene metadata, 11
    chain links, two dynamic supports, 15 falling blocks, 10 public breakable
    fixed joints, and 29 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_fracture_scene_matches_source_row` checks those
    source-row invariants, and
    `test_avbd_demo2d_fracture_scene_breaks_and_resets_source_joints` now
    verifies that the source-row fixed joints fracture, reset at a high break
    force, remain unbroken, and pull their anchor residuals down again. The
    dashboard slice includes
    `BM_AvbdDemo2dFractureStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-fracture-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-fracture-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. After a refreshed same-source timing run, the
    packet records DART at 51.2 us median CPU time per step versus 61.4 us for
    the native source runner on this host, closing only this narrow source-row
    CPU reference comparison. A later rigid-contact-stage allocation cleanup
    reuses extracted point-joint input scratch, AVBD solve-row scratch, and
    per-body row-index scratch, and skips copying already-predicted inertial
    targets. The contact stage now also replaces the old duplicate
    prepare-time collision query with collision-shape-count constraint prewarm;
    execute-time contact discovery remains authoritative. A later follow-up
    reuses a contact-stage AVBD scratch snapshot through an in-place
    `buildAvbdRigidWorldContactSnapshotInto` helper, preserving the
    value-returning helper for standalone callers while reducing allocation
    churn in contact-enabled rows; the same-command benchmark smoke was mixed
    and remains non-packet evidence. A later
    source-alignment cleanup mirrors the
    native solver's constrained-pair broadphase rule by suppressing live public
    rigid-body joint endpoint pairs in `World::collide()` while making broken
    AVBD joints collidable again. The live-pair set now reuses
    `CollisionQueryCache` scratch, no longer needs a `Name` component view,
    skips empty ignored-pair lookups, and the contact-stage preflight scan now
    combines collision-geometry and dynamic-endpoint checks. `test_world`
    checks warmed filtered queries do not allocate global heap. The Fracture
    source-row metadata, benchmark counter, and packet writer now record the 10
    joint-connected collision pairs. The diagnostic
    `/tmp/avbd_fracture_joint_filter_diagnostic.json` run reported about
    50.4 us median CPU time under high host load, so it also must not refresh
    the tracked packet; the later cached-scratch diagnostic
    `/tmp/avbd_fracture_joint_filter_cached_pairs.json` reported about
    98.9 us median CPU time under heavier host load, and
    `/tmp/avbd_demo2d_fracture_after_prepare_skip.json` reported about
    113 us median CPU time under similarly high load. A later unsaved
    same-host diagnostic after the query-filter micro-cleanup ran at about
    139 us median CPU time with host load above 22, so those noisy runs remain
    non-packet diagnostics.
    The categorized `avbd_demo2d_dynamic_friction` py-demo now ports the
    `avbd-demo2d` Dynamic Friction source row with source revision/scene
    metadata, 11 sliding boxes, a static ground, source friction coefficients
    from 5.0 down to 0.0, and 12 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_dynamic_friction_scene_matches_source_row` checks those
    source-row invariants, the dashboard slice includes
    `BM_AvbdDemo2dDynamicFrictionStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-dynamic-friction-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-dynamic-friction-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The packet records DART at 5.73 us median CPU
    time per step versus 10.49 us for the native source runner on this host,
    closing that narrow source row while leaving broad friction-scene, GPU, and
    paper-number gates open.
    The categorized `avbd_demo2d_static_friction` py-demo now ports the
    `avbd-demo2d` Static Friction source row with source revision/scene
    metadata, one rotated static ground slab, 11 rotated dynamic boxes, uniform
    source friction 1.0, and 12 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_static_friction_scene_matches_source_row` checks those
    source-row invariants, the dashboard slice includes
    `BM_AvbdDemo2dStaticFrictionStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-static-friction-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-static-friction-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The packet records DART at 5.44 us median CPU
    time per step versus 14.56 us for the native source runner on this host,
    closing that narrow source row while leaving broad friction-scene, GPU, and
    paper-number gates open.
    The categorized `avbd_demo2d_pyramid` py-demo now ports the `avbd-demo2d`
    Pyramid source row with source revision/scene metadata, a static ground, 210
    dynamic boxes in the source pyramid layout, and 211 collision shapes. Focused
    Python coverage in `test_avbd_demo2d_pyramid_scene_matches_source_row`
    checks those source-row invariants, the dashboard slice includes
    `BM_AvbdDemo2dPyramidStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-pyramid-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-pyramid-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The packet records DART at 0.251 ms median CPU
    time per step versus 2.47 ms for the 10,000-step native source runner on
    this host, closing that narrow Pyramid row while leaving broad rigid
    stacking, GPU, and paper-number gates open.
    The categorized `avbd_demo2d_stack_ratio` py-demo now ports the
    `avbd-demo2d` Stack Ratio source row with source revision/scene metadata,
    six geometric-size dynamic boxes over static ground, and 7 collision
    shapes. Focused Python coverage in
    `test_avbd_demo2d_stack_ratio_scene_matches_source_row` checks those
    source-row invariants, the dashboard slice includes
    `BM_AvbdDemo2dStackRatioStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-stack-ratio-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-stack-ratio-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The packet records DART at 3.62 us median CPU
    time per step versus 8.03 us for the native source runner on this host,
    closing that narrow Stack Ratio row while leaving broad rigid stacking,
    high-ratio stability, GPU, and paper-number gates open.
    The categorized `avbd_demo2d_cards` py-demo now ports the `avbd-demo2d`
    Cards source row with source revision/scene metadata, one source-shaped
    rigid ground slab, 40 thin source-shaped dynamic cards across five levels,
    and 41 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_cards_scene_matches_source_row` checks those source-row
    invariants, the dashboard slice includes `BM_AvbdDemo2dCardsStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-cards-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-cards-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The packet records DART at 0.517 ms median CPU
    time per step versus 96.2 us for the native source runner on this host, so
    this remains partial source-corpus evidence rather than a CPU parity claim.
    The categorized `avbd_demo2d_rod` py-demo now ports the `avbd-demo2d`
    Rod source row with source revision/scene metadata, 20 rigid links, one
    static anchor link, 19 dynamic links, 19 public all-axis fixed joints, and
    20 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_rod_scene_matches_source_row` checks those source-row
    invariants, the dashboard slice includes `BM_AvbdDemo2dRodStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-rod-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-rod-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The packet records DART at 0.253 ms median CPU
    time per step versus 26.4 us for the native source runner on this host, so
    this remains partial source-corpus evidence rather than a CPU parity claim.
    The categorized `avbd_demo2d_soft_body` py-demo now ports the
    `avbd-demo2d` Soft Body source row with source revision/scene metadata, one
    static ground slab, two 15x5 dynamic rigid-box lattices, 260
    finite-stiffness all-axis fixed joints, 224 diagonal ignored collision
    pairs, and 151 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_soft_body_scene_matches_source_row` checks those
    source-row invariants, the dashboard slice includes
    `BM_AvbdDemo2dSoftBodyStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-soft-body-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-soft-body-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The packet records DART at 3.58 ms median CPU
    time per step versus 0.568 ms for the native source runner on this host, so
    this remains partial source-corpus evidence rather than a CPU parity claim.
    The categorized `avbd_demo2d_joint_grid` py-demo now ports the
    `avbd-demo2d` Joint Grid source row with source revision/scene metadata,
    625 source-shaped rigid boxes, two static top-corner anchors, 1200 public
    all-axis fixed joints, and 625 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_joint_grid_scene_matches_source_row` checks those
    source-row invariants, including the source's 1152 diagonal
    `IgnoreCollision` pairs configured through DART's public per-pair collision
    filter. The dashboard slice includes
    `BM_AvbdDemo2dJointGridStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-joint-grid-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-joint-grid-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The packet records DART at 15.1 ms median CPU
    time per step versus 7.33 ms for the native source runner on this host, so
    this remains partial source-corpus evidence rather than a CPU parity claim.
    The categorized `avbd_demo2d_rope` py-demo now ports the `avbd-demo2d`
    Rope source row with source revision/scene metadata, 20 rigid links, 19
    dynamic links, one static link, 19 linear-only public spherical point
    joints, and 20 collision shapes. Focused Python coverage in
    `test_avbd_demo2d_rope_scene_matches_source_row` checks those source-row
    invariants, the dashboard slice includes `BM_AvbdDemo2dRopeStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-rope-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-rope-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. The refreshed packet records DART at 0.133 ms
    median CPU time per step versus 25.6 us for the native source runner on this
    host, so this remains partial source-corpus evidence rather than a parity
    claim.
    The categorized `avbd_demo2d_heavy_rope` py-demo now ports the
    `avbd-demo2d` Heavy Rope source row with source revision/scene metadata, 19
    regular links, one 30 m endpoint block, 19 dynamic links, one static link,
    19 linear-only public spherical point joints, and 20 collision shapes.
    Focused Python coverage in
    `test_avbd_demo2d_heavy_rope_scene_matches_source_row` checks those
    source-row invariants, the dashboard slice includes
    `BM_AvbdDemo2dHeavyRopeStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-heavy-rope-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-heavy-rope-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. After reusing per-body row-index scratch and
    caching snapshot body indices, the refreshed packet records DART at 0.149
    ms median CPU time per step versus 25.4 us for the native source runner on
    this host, so this remains partial source-corpus evidence rather than a
    parity claim.
    The categorized `avbd_demo2d_net` py-demo now ports the `avbd-demo2d` Net
    source row with source revision/scene metadata, one static ground slab, 40
    net links with static endpoints, 50 falling rigid blocks, 39 linear-only
    public spherical point joints, and 91 collision shapes. Focused Python
    coverage in `test_avbd_demo2d_net_scene_matches_source_row` checks those
    source-row invariants, and the dashboard slice includes
    `BM_AvbdDemo2dNetStep`. The tracked
    [`../../plans/104-vertex-block-descent-solver/avbd-demo2d-net-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo2d-net-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo2d` source timing. After caching snapshot body indices and using
    a stable full-rank linear basis for local-anchor spherical point-joint
    extraction, the refreshed packet records DART at 0.286 ms median CPU time
    per step versus 0.232 ms for the native source runner on this host, so this
    remains partial source-corpus evidence rather than a parity claim.
    The categorized `avbd_demo3d_ground` py-demo now ports the `avbd-demo3d`
    Ground source row with source revision/scene metadata, a static floor,
    falling rigid box, 2 rigid bodies, 2 collision shapes, and focused Python
    source-row coverage in
    `test_avbd_demo3d_ground_scene_matches_source_row`; the dashboard slice
    includes `BM_AvbdDemo3dGroundStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-ground-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-ground-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. The packet records DART at 5.49 us median CPU
    time per step versus 6.10 us for the native source runner on this host,
    closing that narrow Ground row while leaving broad rigid-contact, GPU, and
    paper-number gates open.
    The categorized `avbd_demo3d_dynamic_friction` py-demo now ports the
    `avbd-demo3d` Dynamic Friction source row with source revision/scene
    metadata, 11 sliding boxes, a static floor, 12 collision shapes, and focused
    Python source-row coverage in
    `test_avbd_demo3d_dynamic_friction_scene_matches_source_row`; the dashboard
    slice includes `BM_AvbdDemo3dDynamicFrictionStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-dynamic-friction-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-dynamic-friction-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. The packet records DART at 36.85 us median CPU
    time per step versus 51.98 us for the native source runner on this host,
    closing that narrow Dynamic Friction row while leaving broad
    contact-manifold friction persistence, stacking/friction sweeps, GPU, and
    paper-number gates open.
    The categorized `avbd_demo3d_static_friction` py-demo now ports the
    `avbd-demo3d` Static Friction source row with source revision/scene
    metadata, a static floor, an inclined static ramp, 11 sliding boxes, 13
    collision shapes, and focused Python source-row coverage in
    `test_avbd_demo3d_static_friction_scene_matches_source_row`; the dashboard
    slice includes `BM_AvbdDemo3dStaticFrictionStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-static-friction-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-static-friction-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. The packet records DART at 48.27 us median CPU
    time per step versus 51.97 us for the native source runner on this host,
    closing that narrow source row.
    The categorized `avbd_demo3d_pyramid` py-demo now ports the `avbd-demo3d`
    Pyramid source row with source revision/scene metadata, a static ground,
    136 dynamic boxes in the source triangular pile layout, 137 collision
    shapes, and focused Python source-row coverage in
    `test_avbd_demo3d_pyramid_scene_matches_source_row`; the dashboard slice
    includes `BM_AvbdDemo3dPyramidStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-pyramid-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-pyramid-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. The packet records DART at 0.991 ms median CPU
    time per step versus 2.80 ms for the native source runner on this host,
    closing that narrow Pyramid row while leaving broad rigid stacking, GPU, and
    paper-number gates open.
    The categorized `avbd_demo3d_rope` py-demo now ports the `avbd-demo3d`
    Rope source row with source revision/scene metadata, 20 rigid links, 19
    anchored linear-only public spherical point joints, 21 collision shapes, and
    focused Python source-row coverage in
    `test_avbd_demo3d_rope_scene_matches_source_row`; the dashboard slice
    includes `BM_AvbdDemo3dRopeStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-rope-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-rope-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. After reusing per-body row-index scratch and
    caching snapshot body indices, the refreshed packet records DART at 0.134
    ms median CPU time per step versus 35.9 us for the native source runner on
    this host, so this remains partial source-corpus evidence rather than a
    parity claim.
    The categorized `avbd_demo3d_heavy_rope` py-demo now ports the
    `avbd-demo3d` Heavy Rope source row with source revision/scene metadata, 19
    regular links, a 5 m endpoint block, 19 anchored linear-only public
    spherical point joints, the source row's intentional 0.5 m initial residual
    at the oversized endpoint block, 21 collision shapes, and focused Python
    source-row coverage in
    `test_avbd_demo3d_heavy_rope_scene_matches_source_row`; the dashboard slice
    includes `BM_AvbdDemo3dHeavyRopeStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-heavy-rope-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-heavy-rope-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. After reusing per-body row-index scratch and
    caching snapshot body indices, the refreshed packet records DART at 0.128
    ms median CPU time per step versus 33.4 us for the native source runner on
    this host, so this remains partial source-corpus evidence rather than a
    parity claim.
    The categorized `avbd_demo3d_stack` py-demo now ports the `avbd-demo3d`
    Stack source row with source revision/scene metadata, 10 vertical dynamic
    boxes over static ground, 11 collision shapes, and focused Python source-row
    coverage in `test_avbd_demo3d_stack_scene_matches_source_row`; the
    dashboard slice includes `BM_AvbdDemo3dStackStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-stack-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-stack-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. The packet records DART at 42.1 us median CPU
    time per step versus 75.7 us for the native source runner on this host,
    closing that narrow Stack row while leaving broad rigid stacking, high-ratio
    stability, GPU, and paper-number gates open.
    The categorized `avbd_demo3d_stack_ratio` py-demo now ports the
    `avbd-demo3d` Stack Ratio source row with source revision/scene metadata, 4
    geometric-size dynamic boxes over static ground, 5 collision shapes, and
    focused Python source-row coverage in
    `test_avbd_demo3d_stack_ratio_scene_matches_source_row`; the dashboard slice
    includes `BM_AvbdDemo3dStackRatioStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-stack-ratio-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-stack-ratio-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. The packet records DART at 18.1 us median CPU
    time per step versus 42.0 us for the native source runner on this host,
    closing that narrow Stack Ratio row while leaving broad size-ratio
    stability, GPU, and paper-number gates open.
    The categorized `avbd_demo3d_bridge` py-demo now ports the `avbd-demo3d`
    Bridge source row with source revision/scene metadata, 40 planks, 50 load
    boxes, 78 paired linear-only public spherical point joints, 91 collision
    shapes, and focused Python source-row coverage in
    `test_avbd_demo3d_bridge_scene_matches_source_row`; the dashboard slice
    includes `BM_AvbdDemo3dBridgeStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-bridge-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-bridge-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. After reusing per-body row-index scratch and
    caching snapshot body indices, the refreshed packet records DART at 0.746
    ms median CPU time per step versus 1.20 ms for the native source runner on
    this host, closing only this narrow Bridge source-row CPU comparison.
    The categorized `avbd_demo3d_breakable` py-demo now ports the
    `avbd-demo3d` Breakable source row with source revision/scene metadata, 19
    rigid bodies, 10 breakable fixed joints, 19 collision shapes, and focused
    Python source-row coverage in
    `test_avbd_demo3d_breakable_scene_matches_source_row`;
    `test_avbd_demo3d_breakable_scene_breaks_and_resets_source_joints` now
    verifies that the source-row fixed joints fracture, reset at a high break
    force, remain unbroken, and pull their anchor residuals down again. The
    dashboard slice
    includes `BM_AvbdDemo3dBreakableStep`, and
    [`../../plans/104-vertex-block-descent-solver/avbd-demo3d-breakable-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-demo3d-breakable-packet.json)
    records headless visual capture, DART benchmark JSON, and native
    `avbd-demo3d` source timing. The packet records DART at 79.5 us median CPU
    time per step versus 112.7 us for the native source runner on this host, so
    the narrow Breakable source-row CPU gate is closed.
    The categorized `avbd_articulated_revolute_motor` py-demo now exposes a
    public articulated revolute velocity motor with a command reversal through
    the same bridge, with focused Python demo coverage in
    `test_avbd_articulated_revolute_motor_demo_reverses_command` and tracked
    visual/benchmark evidence in
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-revolute-motor-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-revolute-motor-packet.json).
    The categorized `avbd_articulated_prismatic_motor` py-demo now exposes the
    matching public articulated prismatic velocity motor command reversal, with
    focused Python demo coverage in
    `test_avbd_articulated_prismatic_motor_demo_reverses_command` and tracked
    visual/benchmark evidence in
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-prismatic-motor-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-prismatic-motor-packet.json).
    The categorized `avbd_articulated_motor_breakable_joint` py-demo now
    exposes a same-multibody public articulated revolute velocity motor that
    breaks, releases under lateral load, and re-engages through the public reset
    path, with focused Python demo coverage in
    `test_avbd_articulated_motor_breakable_joint_demo_resets_motor_rows` and
    post-reset reversed-command plus weak re-arm coverage, plus
    tracked visual/benchmark evidence in
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-breakable-motor-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-breakable-motor-packet.json).
    The categorized
    `avbd_articulated_prismatic_pair_motor_breakable_joint` py-demo now exposes
    the same-multibody public articulated prismatic velocity motor break/reset
    path, with focused Python demo coverage in
    `test_avbd_articulated_prismatic_pair_motor_breakable_joint_demo_resets_rows`
    and post-reset reversed-command plus weak re-arm coverage, plus tracked
    visual/benchmark evidence in
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-prismatic-pair-breakable-motor-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-prismatic-pair-breakable-motor-packet.json).
    The categorized `avbd_articulated_prismatic_motor_breakable_joint` py-demo
    now exposes the matching world-anchored public articulated prismatic
    velocity motor break/reset path, with focused Python demo coverage in
    `test_avbd_articulated_prismatic_motor_breakable_joint_demo_resets_rows`
    and post-reset reversed-command plus weak re-arm coverage, plus tracked
    visual/benchmark evidence in
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-world-prismatic-breakable-motor-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-world-prismatic-breakable-motor-packet.json).
    The categorized
    `avbd_articulated_world_revolute_motor_breakable_joint` py-demo now exposes
    the world-anchored public articulated revolute velocity motor break/reset
    path, with focused Python demo coverage in
    `test_avbd_articulated_world_revolute_motor_breakable_joint_demo_resets_rows`
    and post-reset reversed-command plus weak re-arm coverage, plus tracked
    visual/benchmark evidence in
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-world-revolute-breakable-motor-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-world-revolute-breakable-motor-packet.json).
    A later focused guard now explicitly checks both world-link demo facades
    expose child-only public articulated joints from Python and removes the
    stale world-revolute remaining gate from the world-prismatic packet now
    that both tracked world-anchor motor packets exist.
    Another narrow lifecycle guard now proves `World::clear()` drops an already
    extracted world-link public articulated AVBD revolute facade, invalidates
    its handles, resets facade state, and allows a fresh world-link public
    prismatic facade to rebuild private AVBD rows from the recreated storage.
    The dartpy public API now mirrors that lifecycle with a Python regression
    proving `World.clear()` invalidates world-link articulated joint handles and
    permits a fresh world-link prismatic motor to be created and stepped after
    the reset.
    Public articulated facade coverage also now checks generated non-topology
    joint names in both C++ and dartpy: empty-name fixed/revolute/prismatic
    facades skip existing `joint_001`, appear through articulated
    name/list lookups, and reject duplicate names.
    Serialization coverage also now checks in C++ and dartpy that generated
    articulated facade joint names resume safely across save/load: after
    restoring explicit `joint_001`, generated `joint_002`, and the topology
    joint component, the next empty-name articulated facade becomes `joint_004`
    rather than reusing an existing public facade name.
    dartpy public facade coverage also now checks that articulated joint lists
    keep their temporary World alive after garbage collection, for both
    link-link and world-link entries.
    The dashboard's bounded AVBD World slice now also includes those public
    articulated revolute/prismatic motor paths plus the active break-force
    armed articulated motor path through
    `BM_AvbdArticulatedRevoluteMotorStep` and
    `BM_AvbdArticulatedPrismaticMotorStep` plus
    `BM_AvbdArticulatedBreakableMotorStep` plus
    `BM_AvbdArticulatedPrismaticBreakableMotorStep` plus
    `BM_AvbdArticulatedWorldPrismaticBreakableMotorStep` plus
    `BM_AvbdArticulatedWorldRevoluteBreakableMotorStep`, with the revolute and
    prismatic command `/1` rows and the break-force-armed
    revolute/prismatic/world-prismatic/world-revolute motor `/1` rows now
    backed by tracked packets.
    The categorized `avbd_articulated_breakable_joint` py-demo now exposes the
    public world-link articulated fixed point-joint break/reset lifecycle
    through the same variational bridge, with focused Python demo coverage in
    `test_avbd_articulated_breakable_joint_demo_marks_and_resets_joint`,
    including weak re-arm breakage after reset.
    The categorized `avbd_articulated_fixed_pair_breakable_joint` py-demo now
    exposes the same-multibody public articulated fixed point-joint break/reset
    lifecycle through the same variational bridge, backed by
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-fixed-pair-breakable-joint-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-fixed-pair-breakable-joint-packet.json)
    and focused Python demo coverage in
    `test_avbd_articulated_fixed_pair_breakable_joint_demo_resets_relative_pose`,
    including weak re-arm breakage after reset.
    The categorized `avbd_articulated_spherical_breakable_joint` py-demo now
    exposes the public world-link articulated spherical break/reset lifecycle
    through the linear-only row bridge, backed by
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-spherical-breakable-joint-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-spherical-breakable-joint-packet.json)
    and focused Python demo coverage in
    `test_avbd_articulated_spherical_breakable_joint_demo_resets_anchor_only`,
    including weak re-arm breakage after reset.
    The categorized `avbd_articulated_spherical_pair_breakable_joint` py-demo
    now exposes the same-multibody public articulated spherical break/reset
    lifecycle, backed by
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-spherical-pair-breakable-joint-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-spherical-pair-breakable-joint-packet.json),
    lifecycle through the same linear-only bridge, with focused Python demo
    coverage in
    `test_avbd_articulated_spherical_pair_breakable_joint_demo_resets_anchor_only`,
    including weak re-arm breakage after reset.
    The categorized `avbd_articulated_high_ratio_chain` py-demo now covers a
    five-link articulated variational chain with a 200:1 heavy tip, with
    focused Python demo coverage in
    `test_avbd_articulated_high_ratio_chain_demo_swings_and_resets`, and
    `BM_AvbdArticulatedHighRatioChainStep` adds the matching narrow dashboard
    row. The tracked
    [`../../plans/104-vertex-block-descent-solver/avbd-articulated-high-ratio-chain-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-high-ratio-chain-packet.json)
    records an 8-frame headless capture and 47.6 us median CPU time per step for
    that narrow five-link smoke without closing the full 50-body/50,000:1 paper
    target. `PaperScaleHighRatioChainStaysFiniteAndResets` now also exercises a
    50-link/50,000:1 articulated chain through configured `World::step()`
    variational solve budget fields (`200` iterations, `1e-9` tolerance) and
    verifies finite rollout plus reset, and
    `BM_AvbdPaperScaleHighRatioChainStep` adds the matching paper-scale CPU
    dashboard row. The tracked
    [`../../plans/104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-chain-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-chain-packet.json)
    records an 8-frame headless visual capture and 3.33 ms median CPU time per
    step for that 50-link/50,000:1 smoke; this is still not a same-hardware
    paper-number comparison, GPU parity, or broad articulated hard-constraint
    completion.
    Broader persistent private articulated motor lifecycle coverage beyond
    command-update, public one-DOF motor break/skip, one-DOF break/reset
    re-engagement checks, and the narrow direct/private and current-pose
    movable-pair revolute/prismatic/spherical reset regressions and
    world-link fixed/spherical current-pose endpoint-polarity coverage plus
    non-cardinal current-pose revolute/prismatic motor-axis checks plus public
    same-multibody/world-anchored articulated revolute/prismatic
    floating-endpoint and selected off-origin-anchor facade non-cardinal axis checks,
    current-pose revolute/prismatic tiny-limit checks, public
    same-multibody/world-link one-DOF finite-limit checks, and public
    same-multibody movable-pair finite-limit and broken-state save/load/reset
    checks plus private generated current-pose movable-pair fixed, one-DOF, and
    spherical broken-state save/load/reset checks plus direct private
    world-link one-DOF broken-state save/load/reset checks, plus broad
    articulated fracture lifecycle/corpus coverage beyond the now-covered 2D
    Fracture and
    3D Breakable source-demo fixed-joint break/reset rows, are
    still missing.
    Unsupported envelopes still fall back to sequential impulses. This is not
    full narrow-phase feature extraction, not full rigid contact/joint rows, not
    broad fracture-corpus support, not broad motor lifecycle or paper/reference
    benchmark-packet coverage, and not broad articulated joint support yet.
- [ ] Phase A4: contact/friction bounds, static/dynamic friction switching, and
      quasi-Newton Hessian approximation.
- [ ] Phase A5: joints, motors, fracture, and breakable constraints.
  - First private angular-motor row and fracture threshold/reset helper slice
    started in the rigid 6-DOF kernel. Public free-rigid-body revolute velocity
    motors now have narrow World wiring, a categorized py-demo, and one
    end-to-end AVBD World benchmark row. Public free-rigid-body prismatic
    velocity motors now extract to the private bounded linear motor row with
    focused C++ builder/extraction coverage, dartpy public stepping coverage, a
    categorized py-demo, one end-to-end AVBD World benchmark row, and a tracked
    visual/benchmark packet.
    Public articulated revolute/prismatic
    velocity motors now also have narrow AVBD World benchmark rows. Public
    free-rigid and articulated breakable fixed point-joint paths now also have
    narrow AVBD World benchmark rows and tracked visual/benchmark packets, and
    the free-rigid fixed and spherical breakable demos now cover
    reset/re-engagement of the captured fixed pose or anchor plus weak re-arm
    breakage after reset. Public empty-scene
    baseline coverage now
    has a runnable source-metadata py-demo, focused Python reference-invariant
    smoke, narrow AVBD World benchmark row, and tracked visual/benchmark packet.
    The `avbd-demo2d` Motor source row now has a first matched-metadata py-demo,
    focused Python source-row invariant, and `BM_AvbdDemo2dMotorStep` benchmark
    row, plus a tracked visual/DART-benchmark/native-reference timing packet
    that keeps the CPU-win gate open because the no-collision-geometry DART
    public World row is still about 6.18x slower than the native source runner
    on this host after skipping contact queries in contact-stage
    prepare/execute.
    The `avbd-demo2d` Ground source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo2dGroundStep`, and a
    tracked visual/DART-benchmark/native-source timing packet that keeps the
    CPU-win gate explicit because, after skipping static-only contact queries and
    no-op rigid dynamics stages in static-only worlds, clean frame-cache graph
    execution, and the clean no-work default step pipeline with a cheap scratch
    reset, DART is about 1.51x faster than the native static source runner on
    this host. Focused `World.StepSkipsCleanKinematicsGraph` coverage preserves
    dirty-parent refresh behavior; broad ground-contact, GPU, and paper-number
    gates remain open.
    The `avbd-demo2d` Hanging Rope source row now has a matched-metadata
    py-demo, focused Python source-row invariant,
    `BM_AvbdDemo2dHangingRopeStep`, and a tracked
    visual/DART-benchmark/native-source timing packet that keeps the CPU-win
    gate open because DART is about 7.84x slower than the native source runner
    on this host.
    The `avbd-demo2d` Fracture source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo2dFractureStep`, and a
    tracked visual/DART-benchmark/native-source timing packet recording DART
    about 1.20x faster than the native source runner on this host, closing only
    the narrow Fracture source row.
    The `avbd-demo2d` Dynamic Friction source row now has a matched-metadata
    py-demo, focused Python source-row invariant,
    `BM_AvbdDemo2dDynamicFrictionStep`, and a tracked
    visual/DART-benchmark/native-source timing packet recording DART about
    1.83x faster than the native source runner on this host.
    The `avbd-demo2d` Static Friction source row now has a matched-metadata
    py-demo, focused Python source-row invariant,
    `BM_AvbdDemo2dStaticFrictionStep`, and a tracked
    visual/DART-benchmark/native-source timing packet recording DART about
    2.68x faster than the native source runner on this host.
    The `avbd-demo2d` Pyramid source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo2dPyramidStep`, and a
    tracked visual/DART-benchmark/native-source timing packet recording DART
    about 9.84x faster than the 10,000-step native source runner on this host.
    The `avbd-demo2d` Stack source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo2dStackStep`, and a
    tracked visual/DART-benchmark/native-source timing packet recording DART
    about 2.17x faster than the native source runner on this host.
    The `avbd-demo2d` Stack Ratio source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo2dStackRatioStep`, and a
    tracked visual/DART-benchmark/native-source timing packet recording DART
    about 2.22x faster than the native source runner on this host.
    The `avbd-demo2d` Cards source row now has a matched-metadata py-demo with
    40 thin source-shaped dynamic cards over a source-shaped ground slab,
    focused Python source-row invariant, `BM_AvbdDemo2dCardsStep`, and a
    tracked visual/DART-benchmark/native-source timing packet that keeps the
    CPU-win gate open because DART is about 5.38x slower than the native source
    runner on this host.
    The `avbd-demo2d` Rod source row now has a matched-metadata py-demo with 20
    source-shaped rigid links, one static anchor link, 19 public all-axis fixed
    joints, focused Python source-row invariant, `BM_AvbdDemo2dRodStep`, and a
    tracked visual/DART-benchmark/native-source timing packet that keeps the
    CPU-win gate open because DART is about 9.58x slower than the native source
    runner on this host.
    The `avbd-demo2d` Soft Body source row now has a matched-metadata py-demo
    with one static ground slab, two 15x5 dynamic rigid-box lattices, 260
    finite-stiffness all-axis fixed joints, 224 diagonal ignored collision
    pairs, focused Python source-row invariant, `BM_AvbdDemo2dSoftBodyStep`,
    and a tracked visual/DART-benchmark/native-source timing packet that keeps
    the CPU-win gate open because DART is about 6.30x slower than the native
    source runner on this host.
    The `avbd-demo2d` Joint Grid source row now has a matched-metadata py-demo
    with 625 source-shaped rigid boxes, two static top-corner anchors, 1200
    public all-axis fixed joints, focused Python source-row invariant,
    `BM_AvbdDemo2dJointGridStep`, and a tracked
    visual/DART-benchmark/native-source timing packet that keeps the CPU-win
    gate open because DART is about 2.06x slower than the native source runner
    on this host after reusing per-body row-index scratch and caching snapshot
    body indices. The source's 1152 diagonal `IgnoreCollision` pairs are now
    configured through DART's public per-pair collision filter.
    The `avbd-demo2d` Rope source row now has a matched-metadata py-demo with
    20 source-shaped rigid links, 19 linear-only public spherical point joints,
    focused Python source-row invariant, `BM_AvbdDemo2dRopeStep`, and a tracked
    visual/DART-benchmark/native-source timing packet that keeps the CPU-win
    gate open because DART is about 5.20x slower than the native source runner
    on this host.
    The `avbd-demo2d` Heavy Rope source row now has a matched-metadata py-demo
    with 19 regular source-shaped rigid links, one 30 m endpoint block, 19
    linear-only public spherical point joints, focused Python source-row
    invariant, `BM_AvbdDemo2dHeavyRopeStep`, and a tracked
    visual/DART-benchmark/native-source timing packet that keeps the CPU-win
    gate open because DART is about 5.85x slower than the native source runner
    on this host.
    The `avbd-demo2d` Spring and Spring Ratio source rows now have
    matched-metadata py-demos over public radial rigid distance springs, focused
    Python source-row invariants, `BM_AvbdDemo2dSpringStep` and
    `BM_AvbdDemo2dSpringRatioStep`, and tracked
    visual/DART-benchmark/native-source timing packets. The packets keep the
    CPU-win gates open because DART is about 5.09x slower than the native Spring
    source runner and about 4.25x slower than the native Spring Ratio source
    runner on this host.
    The `avbd-demo2d` Net source row now has a matched-metadata py-demo with a
    static ground slab, 40 source-shaped net links with static endpoints, 50
    falling rigid blocks, 39 linear-only public spherical point joints, focused
    Python source-row invariant, `BM_AvbdDemo2dNetStep`, and a tracked
    visual/DART-benchmark/native-source timing packet that keeps the CPU-win
    gate open because DART is about 1.23x slower than the native source runner
    on this host after caching snapshot body indices and using a stable
    full-rank linear basis for local-anchor spherical point-joint extraction.
    The `avbd-demo3d` Ground source row now has a matched-metadata py-demo,
    focused Python source-row invariant that observes rigid ground contact,
    `BM_AvbdDemo3dGroundStep`, and a tracked visual/DART-benchmark/native-source
    timing packet recording DART about 1.11x faster than the native source
    runner on this host.
    The `avbd-demo3d` Dynamic Friction source row now has a matched-metadata
    py-demo, focused Python source-row invariant, `BM_AvbdDemo3dDynamicFrictionStep`,
    and a tracked visual/DART-benchmark/native-source timing packet recording
    DART about 1.41x faster than the native source runner on this host.
    The `avbd-demo3d` Static Friction source row now has a matched-metadata
    py-demo, focused Python source-row invariant, `BM_AvbdDemo3dStaticFrictionStep`,
    and a tracked visual/DART-benchmark/native-source timing packet that closes
    the CPU-win gate because DART is about 1.08x faster than the native source
    runner on this host.
    The `avbd-demo3d` Pyramid source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo3dPyramidStep`, and a
    tracked visual/DART-benchmark/native-source timing packet recording DART
    about 2.83x faster than the native source runner on this host.
    The `avbd-demo3d` Rope source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo3dRopeStep`, and a tracked
    visual/DART-benchmark/native-source timing packet that keeps the CPU-win
    gate open because DART is about 3.75x slower than the native source runner
    on this host.
    The `avbd-demo3d` Heavy Rope source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo3dHeavyRopeStep`, and a
    tracked visual/DART-benchmark/native-source timing packet that keeps the
    CPU-win gate open because DART is about 3.84x slower than the native source
    runner on this host.
    The `avbd-demo3d` Spring and Spring Ratio source rows now have
    matched-metadata py-demos over public radial rigid distance springs, focused
    Python source-row invariants, `BM_AvbdDemo3dSpringStep` and
    `BM_AvbdDemo3dSpringRatioStep`, and tracked
    visual/DART-benchmark/native-source timing packets. The packets keep the
    CPU-win gates open because DART is about 1.96x slower than the native Spring
    source runner and about 3.70x slower than the native Spring Ratio source
    runner on this host.
    The `avbd-demo3d` Stack source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo3dStackStep`, and a
    tracked visual/DART-benchmark/native-source timing packet recording DART
    about 1.80x faster than the native source runner on this host.
    The `avbd-demo3d` Stack Ratio source row now has a matched-metadata
    py-demo, focused Python source-row invariant,
    `BM_AvbdDemo3dStackRatioStep`, and a tracked
    visual/DART-benchmark/native-source timing packet recording DART about
    2.32x faster than the native source runner on this host.
    The `avbd-demo3d` Bridge source row now has a matched-metadata py-demo,
    focused Python source-row invariant, `BM_AvbdDemo3dBridgeStep`, and a
    tracked visual/DART-benchmark/native-source timing packet recording DART
    about 1.61x faster than the native source runner on this host.
    The `avbd-demo3d` Breakable source row now has a matched-metadata py-demo,
    focused Python source-row invariant, and `BM_AvbdDemo3dBreakableStep`
    benchmark row for its 19 rigid bodies, 10 breakable fixed joints, and 19
    collision shapes, plus a tracked visual/DART-benchmark/native-source timing
    packet that closes the narrow CPU-win gate because DART is about 1.42x
    faster than the native source runner on this host.
    Public
    free-rigid-body fixed/revolute/
    prismatic/spherical point joints now also have a narrow
    break-force/broken-state lifecycle with C++/dartpy tests, fixed/revolute/
    prismatic/spherical binary save/load broken-state contact-filter reset
    coverage, same-multibody/world-link articulated fixed/revolute/prismatic/
    spherical binary save/load broken-state reset coverage with
    revolute/prismatic velocity-actuator state preservation, free-rigid and
    articulated design-mode binary save/load coverage for public AVBD
    point-joint start/linear/angular stiffness facade state, direct C++/dartpy
    validation for public articulated stiffness defaults, finite setters, and
    invalid setter rejection, C++/dartpy endpoint-ownership rejection for same-link,
    cross-multibody, and cross-world articulated point-joint requests, and
    categorized py-demos; public
    same-multibody/world-link articulated revolute/prismatic/fixed point-joints
    now also have narrow command-update motor and break/reset py-demos over the
    variational bridge, plus focused movable-movable link-pair motor,
    same-multibody/world-anchored off-origin fixed/revolute/prismatic anchor
    overloads and revolute/prismatic anchor motors, same-multibody fixed
    break/reset plus movable-movable fixed break/save/load/reset, and
    world-fixed break/reset regressions, and same-multibody/world-anchored
    public one-DOF motor break/skip regressions including focused dartpy
    stepping tests for explicit-anchor variants of same-multibody/world-link
    revolute and prismatic cases with endpoint/axis-shape assertions plus
    focused dartpy reset/re-engagement regressions for those
    one-DOF motor rows and a categorized same-multibody breakable motor
    py-demo with a tracked visual/benchmark packet, plus focused dartpy
    coverage for same-multibody/world-link
    spherical linear-row break/skip/reset regressions that keep orientation
    free while asserting endpoint shapes, plus a categorized spherical
    break/reset py-demo over that public facade, plus a narrow high mass-ratio
    articulated-chain py-demo, dashboard row, tracked visual/benchmark packet,
    focused 50-link/50,000:1 finite/reset stability smoke through the
    configured `World::step()` solve-budget path, and a matching
    `BM_AvbdPaperScaleHighRatioChainStep` paper-scale dashboard row plus a
    tracked visual/benchmark packet.
    Broad motor lifecycle coverage, broad public articulated facade coverage
    beyond the new same-multibody link-link, world-link, and explicit-anchor
    point-joint entrypoints, GPU parity, broader reference visual artifacts, and
    same-hardware paper-number comparisons are still missing.
- [ ] Phase A6: unified soft/rigid AVBD coupling.
- [ ] Phase G: GPU parity for all row families, candidate generation, and
      benchmark scenes.
- [ ] Phase D/P: every paper/demo scene plus benchmark JSON proving DART beats
      reference and paper CPU/GPU numbers.
  - Durable corpus owner started:
    [`../../plans/104-vertex-block-descent-solver/avbd-demo-corpus.md`](../../plans/104-vertex-block-descent-solver/avbd-demo-corpus.md).
    It maps all `avbd-demo2d`, `avbd-demo3d`, paper, website/video, and
    performance rows to required DART evidence and current missing/partial
    status. It is a tracking surface only; it does not complete any corpus row.

## Goal

Implement AVBD as the hard-constraint continuation of DART's VBD solver family:
all paper/reference algorithms and features, CPU and GPU parity, complete
paper/site/video/demo reproduction in DART tests/benchmarks/`py-demos`, and
performance that beats the reference demo repositories and published paper
numbers.

## Non-Goals For Early Phases

- Do not expose AVBD, row storage, solver registries, CUDA types, or ECS details
  as public API.
- Do not vendor or runtime-link the AVBD demo repositories.
- Do not claim AVBD parity from the scalar row utility, a CPU-only slice, or one
  demo scene.

## Key Decisions

- **Reuse PLAN-104:** AVBD extends the VBD solver family, so the durable owner is
  PLAN-104 plus its AVBD paper gap audit rather than a duplicate plan.
- **Row foundation first:** The scalar row update equations are shared by hard
  contact, joints, attachments, friction limits, motors, fracture, and
  finite-stiffness ramping, so they are the first tested implementation slice.
- **Clean DART 7/8 architecture:** AVBD work may refactor internal solver,
  pipeline, row-storage, compute, and demo surfaces when that produces a cleaner
  long-term design.

## Immediate Next Steps

1. Continue the next bounded AVBD contact/friction or rigid-block slice:
   broader articulated motor lifecycle coverage, broader articulated fracture
   lifecycle/corpus coverage beyond the current private/public
   same-multibody, world-link, explicit-anchor, off-origin,
   binary-save/load/reset, spherical linear-row, and one-DOF motor regression
   set, including the new private world-link fixed/spherical current-pose
   endpoint-polarity check, public articulated World facade or demo-packet coverage for
   combinations not yet promoted into the corpus,
   full narrow-phase feature extraction for remaining realistic manifolds,
   broader rigid-contact feature persistence beyond the current
   box/sphere/cylinder/capsule/plane/mesh known/unknown shape-frame
   local-transform/inference evidence, endpoint-A/B explicit-shape local-point
   evidence, actual narrow-phase primitive-feature evidence including live
   mesh-vertex feature evidence, and same-feature sphere/plane plus
   sphere/mesh-face, sphere/mesh-edge, and mesh-vertex replay plus
   mesh-face/mesh-edge/mesh-vertex small-pose persistence and endpoint-order
   stability plus live
   cylinder-cap/plane and capsule-cap/plane plus cylinder-side/capsule-side and
   cylinder-rim row-order evidence, cylinder/capsule cap/side/rim small-pose
   persistence and endpoint-order stability evidence, live sphere/plane
   friction tangent warm-start mapping evidence, live sphere/plane
   normal-rotation friction tangent projection evidence, live endpoint-swapped
   box-box friction tangent projection evidence, live box-box row-order,
   endpoint-order, and small-pose
   persistence evidence, live box-box manifold friction warm-start persistence
   evidence, and broader stacked/frictional pile evidence beyond the current
   box-manifold, first stacked static/dynamic plus dynamic/dynamic, narrow
   multi-top and box-pile contact-stage/default-step, private spanning-top
   box-pile row-persistence/contact-order/endpoint-order, and private multi-top
   box-pile row-persistence/contact-order/endpoint-order slices, or the first
   implementation row from the newly promoted AVBD paper/source-demo corpus
   matrix are the preferred next gaps now that
   static box feature IDs,
   private dynamic/rigid contact feature IDs and descriptor
   helpers, static half-space tangent dual projection, self-contact
   tangent dual projection, private rigid contact tangent dual projection,
   static contact/friction, attachments,
   finite-stiffness rows, self-contact normals, pairwise static/dynamic friction
   switching, supported World self-contact friction rows, pure-tet self-contact
   friction rows, combined static/self-contact friction row coexistence, and the
   first private 6-DOF rigid block plus contact/friction point-pair row
   foundation, rigid point-pair friction-cone helper, private serial rigid row
   driver, private rigid contact-manifold row builder, private World-contact
   snapshot/solve/writeback helpers, combined private wrapper, first internal
   `RigidAvbdContactConfig` contact-stage velocity-projection activation,
   box-feature/pair-scoped rigid contact row identity, live sphere/plane and
   box-box manifold endpoint-order row identity, deterministic
   same-feature contact row ordering including actual `World::collide()`
   sphere/plane plus sphere/mesh-face, sphere/mesh-edge, mesh-vertex replay, and
   mesh-face/mesh-edge/mesh-vertex small-pose persistence plus endpoint-order
   stability, live
   cylinder-cap/plane and capsule-cap/plane plus cylinder-side/capsule-side and
   cylinder-rim row-order coverage, cylinder/capsule cap/side/rim small-pose
   persistence and endpoint-order stability coverage, and live sphere/plane friction tangent row
   warm-start mapping plus live
   sphere/plane normal-rotation friction tangent projection, live
   endpoint-swapped box-box friction tangent projection, live box-box manifold
   friction warm-start persistence, live stacked-box friction warm-start
   persistence and contact-order replay, live multi-top box-pile friction row-persistence,
   contact-stage multi-top stacked-box friction slip-reduction,
   default-step multi-top stacked-box friction slip-reduction, live stacked-box
   endpoint-swapped friction tangent projection, live box-pile friction
   row-persistence, contact-order replay, and endpoint-swapped friction tangent projection,
   live multi-top box-pile friction row-persistence, contact-order replay, and
   endpoint-swapped friction tangent projection,
   contact-stage box-pile and two-lower/two-upper box-pile friction
   slip-reduction, default-step box-pile and two-lower/two-upper box-pile
   friction slip-reduction,
   and box-face feature coverage,
   cylinder side/cap/rim and capsule side/top-cap/bottom-cap endpoint features,
   private rigid
   `World::collide()` sphere/cylinder/capsule/plane/mesh primitive-feature coverage,
   point-joint linear/angular/combined row builders with step-start previous
   constraint values, and private World
   snapshot/step point-joint append/solve/apply coverage plus fixed-joint ECS
   extraction through the step helper and the internal contact-stage velocity
   projection with or without active contacts, plus an explicit rigid-body
   fixed-joint current-pose config bridge and simulation-entry config
   initialization for opt-in rigid bodies, masked private point-joint row
   generation for constrained linear/angular axes, and private
   revolute/prismatic point-joint configs with arbitrary joint-axis bases plus
   private rigid-body ECS current-pose extraction for those one-DOF joint
   entities, and public free rigid-body revolute/prismatic facades with dartpy
   bindings, stubs, focused tests, and py-demo coverage, plus the first
   `avbd_rigid_fixed_joint_contact` fixed-joint/contact AVBD demo, the
   `avbd_rigid_revolute_motor` motor demo with a tracked visual/benchmark
   packet, and the
   `avbd_articulated_revolute_motor` and
   `avbd_articulated_prismatic_motor` articulated motor demos, the
   `avbd_articulated_motor_breakable_joint` same-multibody articulated motor
   break/reset demo, the
   `avbd_articulated_prismatic_pair_motor_breakable_joint` same-multibody
   articulated prismatic motor break/reset demo, the
   `avbd_articulated_prismatic_motor_breakable_joint` world-anchored
   articulated prismatic motor break/reset demo, the
   `avbd_articulated_world_revolute_motor_breakable_joint` world-anchored
   articulated revolute motor break/reset demo, the
   `avbd_rigid_breakable_joint` break-force demo with
   `avbd-rigid-breakable-joint-packet.json` packet evidence, the
   `avbd_rigid_spherical_breakable_joint` spherical break/reset demo with
   `avbd-rigid-spherical-breakable-joint-packet.json` packet evidence, the
   `avbd_articulated_breakable_joint` world-link articulated break/reset demo
   with `avbd-articulated-breakable-joint-packet.json` packet evidence,
   the `avbd_articulated_fixed_pair_breakable_joint` same-multibody
   articulated fixed break/reset demo with
   `avbd-articulated-fixed-pair-breakable-joint-packet.json` packet evidence,
   the `avbd_articulated_spherical_breakable_joint` world-link articulated
   spherical break/reset demo with
   `avbd-articulated-spherical-breakable-joint-packet.json` packet evidence, the
   `avbd_articulated_spherical_pair_breakable_joint` same-multibody
   articulated spherical break/reset demo with
   `avbd-articulated-spherical-pair-breakable-joint-packet.json` packet
   evidence,
   the `avbd_articulated_high_ratio_chain` high mass-ratio articulated-chain
   smoke demo, `BM_AvbdArticulatedHighRatioChainStep` dashboard row, and
   `avbd-articulated-high-ratio-chain-packet.json` visual/benchmark packet,
   plus `avbd_paper_scale_high_ratio_chain`,
   `PaperScaleHighRatioChainStaysFiniteAndResets` covering a 50-link/50,000:1
   finite/reset smoke through configured `World::step()` solve-budget fields
   and `BM_AvbdPaperScaleHighRatioChainStep` exposing the matching paper-scale
   CPU dashboard row with `avbd-paper-scale-high-ratio-chain-packet.json`
   visual/benchmark evidence,
   and the `avbd_empty_baseline` source-corpus metadata/reference smoke demo,
   plus the `avbd_demo2d_ground`, `avbd_demo2d_motor`,
   `avbd_demo2d_hanging_rope`,
   `avbd_demo2d_spring`, `avbd_demo2d_spring_ratio`,
   `avbd_demo2d_fracture`, `avbd_demo2d_dynamic_friction`,
   `avbd_demo2d_static_friction`,
   `avbd_demo2d_pyramid`,
   `avbd_demo2d_cards`,
   `avbd_demo2d_stack`,
   `avbd_demo2d_stack_ratio`,
   `avbd_demo2d_rod`,
   `avbd_demo2d_joint_grid`,
   `avbd_demo2d_rope`,
   `avbd_demo2d_heavy_rope`,
   `avbd_demo2d_net`,
   `avbd_demo3d_ground`,
   `avbd_demo3d_dynamic_friction`, `avbd_demo3d_static_friction`,
   `avbd_demo3d_pyramid`, `avbd_demo3d_rope`,
   `avbd_demo3d_heavy_rope`, `avbd_demo3d_spring`,
   `avbd_demo3d_spring_ratio`, and `avbd_demo3d_breakable`
   matched-metadata source-row smoke demos, and the
   `avbd-empty-baseline-packet.json`, `avbd-demo2d-ground-packet.json`,
   `avbd-demo2d-motor-packet.json`,
   `avbd-demo2d-hanging-rope-packet.json`,
   `avbd-demo2d-fracture-packet.json`,
   `avbd-demo2d-dynamic-friction-packet.json`,
   `avbd-demo2d-static-friction-packet.json`,
   `avbd-demo2d-pyramid-packet.json`,
   `avbd-demo2d-cards-packet.json`,
   `avbd-demo2d-stack-packet.json`,
   `avbd-demo2d-stack-ratio-packet.json`,
   `avbd-demo2d-rod-packet.json`,
   `avbd-demo2d-soft-body-packet.json`,
   `avbd-demo2d-joint-grid-packet.json`,
   `avbd-demo2d-rope-packet.json`,
   `avbd-demo2d-heavy-rope-packet.json`,
   `avbd-demo2d-net-packet.json`,
   `avbd-demo3d-ground-packet.json`,
   `avbd-demo3d-dynamic-friction-packet.json`,
   `avbd-demo3d-static-friction-packet.json`,
   `avbd-demo3d-pyramid-packet.json`,
   `avbd-demo3d-rope-packet.json`,
   `avbd-demo3d-heavy-rope-packet.json`,
   `avbd-demo3d-spring-packet.json`,
   `avbd-demo3d-spring-ratio-packet.json`,
   `avbd-demo3d-stack-packet.json`,
   `avbd-demo3d-stack-ratio-packet.json`, and
   `avbd-demo3d-breakable-packet.json` visual/benchmark/source-timing packets,
   the
   private free-rigid versus multibody-link endpoint classifier, endpoint
   benchmark row, narrow articulated motor, active breakable-motor, direct
   private movable-pair revolute break/reset/updated-command regression,
   private current-pose fixed all-axis movable-pair break/reset regression,
   private current-pose fixed movable-pair broken-state save/load/reset
   regression,
   private current-pose revolute/prismatic non-cardinal motor-axis regression,
   public same-multibody/world-anchored articulated revolute/prismatic
   floating-endpoint plus selected off-origin-anchor facade non-cardinal
   motor-axis regressions, selected same-multibody/world-link
   revolute/prismatic motor save/load and broken-state save/load/reset
   axis-basis regressions, selected direct one-DOF break/skip/reset
   non-cardinal axis-basis regressions, public same-multibody/world-link
   one-DOF non-cardinal off-origin finite-limit regressions,
   private current-pose revolute/prismatic movable-pair break/reset/updated-command
   regressions, private current-pose revolute/prismatic movable-pair
   tiny-limit regressions, private current-pose revolute/prismatic
   movable-pair broken-state save/load/reset regressions, public
   same-multibody movable-pair
   revolute/prismatic non-cardinal motor-axis and finite-limit regressions,
   private current-pose spherical movable-pair linear-row break/reset
   regression, private current-pose spherical movable-pair broken-state
   save/load/reset regression, and
   breakable fixed point-joint benchmark rows, focused hard fixed
   articulated endpoint bridge tests, related public variational endpoint
   loop-closure py-demo, and public articulated
   fixed/revolute/prismatic/spherical link-link (including movable link-pair
   motor plus spherical linear-only pinned-anchor and break/skip coverage) and
   world-link
   point-joint facades (including explicit spherical anchors and world-fixed
   break/reset plus spherical reset), have
   narrow CPU/user-visible or
   extractor-foundation paths. The next code slice should broaden private
   articulated motor lifecycle coverage beyond the narrow hard
   revolute/prismatic/spherical point-joint projections, one-DOF lifecycle checks, and
   the direct/private plus current-pose movable-pair fixed/revolute/prismatic
   reset/tiny-limit/save-load-reset regressions plus the current-pose spherical
   reset/save-load-reset regressions and the public same-multibody/world-link
   one-DOF off-origin plus same-multibody movable-pair non-cardinal finite-limit
   regressions,
   broaden articulated fracture coverage beyond the narrow hard point-joint
   projection threshold and movable link-pair reset, with the fixed-row,
   world-fixed, same-multibody/world-anchored one-DOF motor-row, fixed
   point-joint, and spherical linear-row reset evidence now treated as covered
   by the current C++/dartpy regression set. Continue source/demo fracture
   cases beyond the now-covered 2D Fracture and 3D Breakable break/reset rows,
   and expand public articulated World facade coverage beyond the current
   same-multibody link-link/world-link entrypoints, explicit anchors,
   same-multibody/world-anchor fixed/revolute/prismatic off-origin cases,
   spherical linear-only pinned-anchor rows including explicit anchor variants
   plus same-multibody/world-link fixed/spherical save/load rebuild coverage,
   same-multibody/world-link revolute/prismatic motor save/load rebuild coverage
   including selected non-cardinal axis-basis persistence,
   same-multibody/world-link fixed/spherical/revolute/prismatic broken-state
   save/load/reset persistence including explicit-anchor fixed and selected
   non-cardinal one-DOF motor rows with restored effort-limit state plus
   selected direct break/skip/reset non-cardinal basis checks,
   and movable link-pair motor
   projection while retaining explicit fallback
   coverage until each real articulated solve path exists. The next
   source-demo slice should either optimize the measured 2D Motor, Hanging
   Rope, 2D Cards, 2D Rod, 2D Soft Body, 2D Joint Grid, 2D Rope, 2D Heavy Rope,
   2D Net, 2D/3D Spring, 2D/3D Spring Ratio, 3D Rope, 3D Heavy Rope, or 3D Soft
   Body CPU
   source-runner gaps, or add Spring/Spring Ratio GPU evidence. The Spring and
   Spring Ratio source rows now have
   radial rigid distance-spring source-scene harnesses, benchmark rows, native
   source timing entrypoints, and tracked visual/DART-benchmark/native-reference
   packets; the refreshed packets improve the 2D Spring/Spring Ratio rows to
   about 3.82 us and 31.8 us, and the 3D Spring/Spring Ratio rows to about
   3.58 us and 29.1 us, but they still need CPU performance resolution and GPU
   parity before any source-row completion claim. The source Soft Body rows now port through
   the rigid point-joint facade because AVBD fixed/revolute/prismatic/
   spherical point-joint rows support finite linear and angular material
   stiffness through C++ and dartpy setters; those finite rows use raw
   residuals, do not carry persistent hard-constraint duals, and ramp row
   stiffness up to the configured material cap. The AVBD scalar row inventory
   now also has a stable-descriptor fast path that warm-starts unchanged row
   lists in place instead of rebuilding the previous-row map every step, and
   rigid World contact snapshots now assign contact/joint/spring row ordinals
   through reserved endpoint-pair hash counters instead of per-step tree maps;
   append paths seed the body-index cache when a fallback scan finds a
   preseeded snapshot entity, and the rigid row driver also skips per-body
   row-index scratch setup for row families that are absent in the current
   solve, reuses unchanged row-index layouts across frames, and routes
   single-family point-pair/angular row solves directly instead of copying them
   into combined work vectors, while one-new-row point-joint/distance-spring
   appends skip endpoint row-counter hash-map setup. The articulated
   variational point-joint extractor now builds a per-multibody link-index
   cache before scanning AVBD point-joint configs, avoiding per-joint
   structure/link ownership scans on same-multibody and world-link private row
   extraction; focused articulated C++ tests and `/32` articulated benchmark
   smoke cover the changed path. Rigid AVBD point-joint and distance-spring
   extraction now also shares and carries projectable-body metadata through the
   extracted inputs, so the pair-constraint hot path does not classify each
   endpoint and then fetch the same transform/mass/static state again during
   append, and contact-snapshot body insertion reuses the already-checked
   projectable transform, mass, and static tag when materializing new body
   state. The same metadata path now also skips static-static rigid point-joint
   and distance-spring pairs before input or row construction. The
   contact-stage AVBD path
   now reuses a scratch contact snapshot, skips point-joint/distance-spring
   extraction when no AVBD pair-constraint configs exist, and extracts/appends
   point-joint and distance-spring families independently when only one family
   exists. That private production path also requests local-anchor extraction
   for point-joint and distance-spring inputs, avoiding redundant world-anchor
   reconstruction immediately before the row appenders convert those anchors
   back to body-local coordinates; public extraction helpers still default to
   world-space anchors. The rigid snapshot solve now also clears absent
   row-family
   inventories directly instead of calling empty contact/joint/motor/spring
   builders, and small point-joint/motor/distance-spring row builders now use
   stack descriptor/active-row storage for up to 16 candidate rows instead of
   allocating temporary vectors. Large point-joint row builders also reserve
   heap descriptor/active-row scratch lazily, so linear-only spherical/rope
   joint sets clear stale angular rows without preallocating angular scratch for
   an empty axis family. Large rigid World pair-constraint snapshots now also
   activate their reserved entity-to-body index map immediately, so early
   bodies in rope, net, Joint Grid, and soft-body source rows do not pay repeated
   linear lookup before the hash map is populated. Rigid AVBD block assembly now
   also reuses the world anchor points, relative vector, and spring axis already
   computed for each point-pair/distance-spring row constraint value when
   stamping that row's body-local force/Hessian contribution, avoiding repeated
   local-to-world point transforms in large rope/net/Joint Grid/soft-body
   projection loops. The distance-spring Hessian helper now also accepts those
   precomputed world anchors, so direct helper callers and block assembly avoid
   recomputing the same local-to-world point just to build the point Jacobian.
   Friction tangent-pair assembly now likewise computes each tangent row's
   world anchors once, feeds precomputed constraint values through the shared
   static/dynamic friction force helper, and stamps both body directions from
   those same anchors. Friction tangent-pair row-state updates now also reuse
   the same regularized 2D tangent constraint vector for cone projection and
   stiffness growth, avoiding a second constraint evaluation per update. The
   common shared-anchor contact-row case also reuses the first tangent row's
   transformed anchors in the generic force helper, direct block assembly, and
   row-state update path while preserving the existing distinct-anchor fallback.
   Capped rigid finite-stiffness point-pair, angular-pair, and distance-spring
   rows also skip row-state constraint recomputation once the effective stiffness
   is at the material cap, preserving fixed-stiffness semantics while avoiding
   redundant geometry work in the per-iteration update loop.
   The same loop also reuses the shared angular orientation
   error across consecutive all-axis angular rows, avoiding repeated quaternion
   error evaluation for fixed-joint angular triplets during block assembly and
   row-state updates. The scalar row inventory now
   also has a generated descriptor sync path for large stable row families, so
   large point-joint,
   motor, and distance-spring builders compare cheap row keys first and avoid a
   per-step descriptor-vector allocation when row order persists. The
   kinematics stage now skips graph execution when all frame caches are already
   clean, avoiding repeated no-op frame-cache work for static-only and
   unchanged-frame steps, and its prepare step now reuses the cached graph until
   the world's frame-topology revision changes instead of rebuilding topology
   metadata every prepared step. Dirty-parent and stale-topology refresh remain
   covered in `World.StepSkipsCleanKinematicsGraph` and
   `World.StepRebuildsCachedKinematicsAfterFrameReparenting`; built-in step
   profiling bypasses the execution skip so nested graph profiles remain
   observable. The split rigid-body
   velocity stage also skips the entire default built-in step pipeline for
   clean no-work worlds after simulation-entry schedule preparation, unless
   built-in step profiling is enabled, and the split rigid-body velocity stage
   assembles force batches only for advanceable rigid bodies. Local profile
   smoke showed the Joint Grid contact stage at about 14.0 ms and Soft Body at
   about 3.0 ms after the row-assembly reuse. Current selected-row benchmark
   smoke after angular-row reuse reports `BM_AvbdDemo2dJointGridStep` at about
   10.1 ms, `BM_AvbdDemo2dSoftBodyStep` at about 2.14 ms,
   `BM_AvbdDemo2dRopeStep` at about 0.120 ms, and `BM_AvbdDemo2dNetStep` at
   about 0.389 ms per step; a later post-Hessian-reuse diagnostic under host
   load average around 5.5 reported about 10.1 ms, 3.78 ms, 0.207 ms, and
   0.359 ms respectively, so it remains non-packet overhead evidence rather
   than a CPU-win claim. A later friction-pair reuse diagnostic under host load
   average around 6.9 reported about 0.550 ms Cards, 10.7 ms Joint Grid,
   2.43 ms Soft Body, 0.135 ms Rope, and 0.442 ms Net, again as non-packet
   overhead evidence only. Current small-row
   benchmark smoke reports
   `BM_AvbdDemo2dMotorStep_median` at about 8.43 us,
   `BM_AvbdDemo2dSpringStep_median` at about 3.98 us, and
   `BM_AvbdDemo2dSpringRatioStep_median` at about 36.16 us; refreshed packet
   evidence now records about 3.82 us 2D Spring, 31.8 us 2D Spring Ratio,
   3.58 us 3D Spring, and 29.1 us 3D Spring Ratio, while keeping all four
   CPU-win gates open. A broader capped finite-row diagnostic over Rod,
   Joint Grid, Soft Body, and Spring rows ran under load average around 11.7 and
   is also non-packet overhead smoke. A later 2D Soft Body endpoint-uniqueness
   row-counter fast path was implemented experimentally and reverted because
   packet-style benchmark smoke did not support keeping the change. A later
   small-scene broad-sphere contact-query precheck was also prototyped and
   reverted: focused Spring/Spring Ratio benchmark smoke moved the medians from
   about 3.79 us / 34.26 us / 3.90 us / 28.94 us to about
   4.01 us / 33.51 us / 4.44 us / 29.75 us for 2D Spring, 2D Spring Ratio,
   3D Spring, and 3D Spring Ratio respectively, which did not justify adding a
   pre-query bound pass. The later point-pair anchor reuse cleanup reports
   `BM_AvbdDemo2dSoftBodyStep_median` around 2.03 ms in local selected-row
   smoke, with Rod/Joint Grid/Soft Body/Rope/Net around
   0.149 ms / 10.8 ms / 1.70 ms / 0.113 ms / 0.360 ms under load; this is
   still non-packet overhead evidence. Remaining
   checked packet CPU-win gates stay open until refreshed same-command packets
   beat the native runners. The 2D and 3D Soft Body source rows now have
   rigid-box lattice harnesses, packets, and CPU-reference comparisons; the
   refreshed 3D packet closes only its narrow CPU reference-comparison gate,
   while the 2D CPU-win gate and GPU evidence remain open.
   The 2D Ground row is closed only for its static source scene after the
   refreshed packet records it about 1.51x faster than native, the 3D Ground row
   is closed only for that narrow source scene, the 2D and 3D
   Dynamic Friction rows are closed only for those narrow source scenes rather
   than broad friction parity, the 2D Stack and Stack Ratio rows are closed
   only for those narrow source scenes rather than broad stacking/high-ratio
   parity, the 2D and 3D Pyramid rows are closed only for those narrow source scenes rather than broad stacking
   parity, the Fracture, 3D Soft Body, and 3D Bridge rows are closed only for
   their narrow source scenes, the Hanging Rope, 2D Cards, 2D Rod,
   2D Soft Body, 2D Joint Grid, 2D Rope, 2D Heavy Rope, 2D Net,
   2D/3D Spring, 2D/3D Spring Ratio, 3D Rope, and 3D Heavy
   Rope rows are source ports with open CPU-win gates rather than broad
   card-tower/rod/soft-body/joint-grid/rope/high-ratio/spring/fracture or
   coupled-constraint parity, and the 3D
   Stack/Stack Ratio rows are closed only for their narrow source scenes rather
   than broad stacking, high-ratio-stability, GPU, or paper-number parity.
2. In parallel planning, keep full friction cones, rigid/articulated rows, GPU
   parity, demos, and benchmark packets as open AVBD parity gates rather than
   completion claims.
